#include "ads126xAdc.h"

#include <limits.h>
#include <string.h>

#include "sdkconfig.h"
#include "esp_rom_sys.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "freertos/task.h"

#ifndef CONFIG_ADS126X_LOG_LEVEL
#define CONFIG_ADS126X_LOG_LEVEL 3
#endif

#ifndef LOG_LOCAL_LEVEL
#define LOG_LOCAL_LEVEL CONFIG_ADS126X_LOG_LEVEL
#endif

static const char *TAG = "ads126xAdc";

#define ADS126X_SPI_DUMMY_BYTE 0x00

#ifndef CONFIG_SENSORARRAY_SPI_MAX_TRANSFER_BYTES
#define ADS126X_SPI_MAX_TRANSFER_BYTES 64u
#else
#define ADS126X_SPI_MAX_TRANSFER_BYTES CONFIG_SENSORARRAY_SPI_MAX_TRANSFER_BYTES
#endif

/* Command opcodes from ADS1262/ADS1263 datasheet (ADC Commands table). */
#define ADS126X_CMD_NOP 0x00
#define ADS126X_CMD_RESET 0x06
#define ADS126X_CMD_START1 0x08
#define ADS126X_CMD_STOP1 0x0A
#define ADS126X_CMD_START2 0x0C
#define ADS126X_CMD_STOP2 0x0E
#define ADS126X_CMD_RDATA1 0x12
#define ADS126X_CMD_RDATA2 0x14
#define ADS126X_CMD_SYOCAL1 0x16
#define ADS126X_CMD_SYGCAL1 0x17
#define ADS126X_CMD_SFOCAL1 0x19
#define ADS126X_CMD_SYOCAL2 0x1B
#define ADS126X_CMD_SYGCAL2 0x1C
#define ADS126X_CMD_SFOCAL2 0x1E
#define ADS126X_CMD_RREG 0x20
#define ADS126X_CMD_WREG 0x40

/* Register addresses from datasheet register map. */
#define ADS126X_REG_ID 0x00
#define ADS126X_REG_POWER 0x01
#define ADS126X_REG_INTERFACE 0x02
#define ADS126X_REG_MODE0 0x03
#define ADS126X_REG_MODE1 0x04
#define ADS126X_REG_MODE2 0x05
#define ADS126X_REG_INPMUX 0x06
#define ADS126X_REG_REFMUX 0x0F

/* INTERFACE register bit definitions. */
#define ADS126X_INTERFACE_STATUS (1u << 2)
#define ADS126X_INTERFACE_CRC_MASK 0x03u

/* MODE2 register bit definitions. */
#define ADS126X_MODE2_BYPASS (1u << 7)
#define ADS126X_MODE2_GAIN_SHIFT 4
#define ADS126X_MODE2_DR_MASK 0x0Fu

#define ADS126X_ADC1_RAW_NEAR_FULL_SCALE ((int64_t)INT32_MAX * 90 / 100)
#define ADS126X_DRDY_FAST_YIELD_INTERVAL 64u

/* ID register bits. */
#define ADS126X_ID_DEV_ID_MASK 0xE0u
#define ADS126X_ID_DEV_ID_SHIFT 5
#define ADS126X_DEV_ID_ADS1262 0x00u
#define ADS126X_DEV_ID_ADS1263 0x01u

#define ADS126X_MAX_REG_READ_LEN 32u

/* Millisecond-level delays keep timing robust with RTOS tick granularity. */
#define ADS126X_RESET_PULSE_LOW_MS 2u
#define ADS126X_RESET_RELEASE_WAIT_MS 2u
#define ADS126X_RESET_COMMAND_DELAY_MS 2u
#define ADS126X_INTERNAL_REF_SETTLE_MS 50u

static esp_err_t ads126xAdcAllocSpiBuffers(ads126xAdcHandle_t *handle)
{
    size_t bufSize = ADS126X_SPI_MAX_TRANSFER_BYTES;
    if (bufSize < 8u) {
        bufSize = 8u;
    }

    handle->spiTxBuf = heap_caps_malloc(bufSize, MALLOC_CAP_DMA);
    handle->spiRxBuf = heap_caps_malloc(bufSize, MALLOC_CAP_DMA);
    if (!handle->spiTxBuf || !handle->spiRxBuf) {
        if (handle->spiTxBuf) {
            heap_caps_free(handle->spiTxBuf);
        }
        if (handle->spiRxBuf) {
            heap_caps_free(handle->spiRxBuf);
        }
        handle->spiTxBuf = heap_caps_malloc(bufSize, MALLOC_CAP_8BIT);
        handle->spiRxBuf = heap_caps_malloc(bufSize, MALLOC_CAP_8BIT);
        if (!handle->spiTxBuf || !handle->spiRxBuf) {
            if (handle->spiTxBuf) {
                heap_caps_free(handle->spiTxBuf);
            }
            if (handle->spiRxBuf) {
                heap_caps_free(handle->spiRxBuf);
            }
            return ESP_ERR_NO_MEM;
        }
        ESP_LOGW(TAG, "DMA buffers unavailable; using non-DMA buffers");
    }

    handle->spiBufSize = bufSize;
    return ESP_OK;
}

static bool ads126xAdcGainToCode(uint8_t gain, uint8_t *code)
{
    switch (gain) {
    case 1:
        *code = 0;
        return true;
    case 2:
        *code = 1;
        return true;
    case 4:
        *code = 2;
        return true;
    case 8:
        *code = 3;
        return true;
    case 16:
        *code = 4;
        return true;
    case 32:
        *code = 5;
        return true;
    default:
        return false;
    }
}

static bool ads126xAdcIsValidGain(uint8_t gain)
{
    uint8_t code = 0;
    return ads126xAdcGainToCode(gain, &code);
}

static uint8_t ads126xAdcNormalizeGainOrDefault(uint8_t gain, uint8_t fallback)
{
    if (ads126xAdcIsValidGain(gain)) {
        return gain;
    }
    return ads126xAdcIsValidGain(fallback) ? fallback : ADS126X_GAIN_1;
}

static uint8_t ads126xAdcNextLowerGain(uint8_t gain)
{
    switch (gain) {
    case ADS126X_GAIN_32:
        return ADS126X_GAIN_16;
    case ADS126X_GAIN_16:
        return ADS126X_GAIN_8;
    case ADS126X_GAIN_8:
        return ADS126X_GAIN_4;
    case ADS126X_GAIN_4:
        return ADS126X_GAIN_2;
    case ADS126X_GAIN_2:
    case ADS126X_GAIN_1:
    default:
        return ADS126X_GAIN_1;
    }
}

static bool ads126xAdcRawNearFullScale(int32_t rawCode)
{
    int64_t rawAbs = (rawCode == INT32_MIN) ? ((int64_t)INT32_MAX + 1LL) : (int64_t)(rawCode < 0 ? -rawCode : rawCode);
    return rawAbs > ADS126X_ADC1_RAW_NEAR_FULL_SCALE;
}

static uint8_t ads126xAdcSelectGainForUv(const ads126xAdcHandle_t *handle,
                                         int32_t microvolts,
                                         uint8_t minGain,
                                         uint8_t maxGain,
                                         uint8_t headroomPercent)
{
    if (!handle || handle->vrefMicrovolts == 0u) {
        return ADS126X_GAIN_1;
    }

    static const uint8_t gains[] = {
        ADS126X_GAIN_1,
        ADS126X_GAIN_2,
        ADS126X_GAIN_4,
        ADS126X_GAIN_8,
        ADS126X_GAIN_16,
        ADS126X_GAIN_32,
    };

    minGain = ads126xAdcNormalizeGainOrDefault(minGain, ADS126X_GAIN_1);
    maxGain = ads126xAdcNormalizeGainOrDefault(maxGain, ADS126X_GAIN_32);
    if (minGain > maxGain) {
        uint8_t tmp = minGain;
        minGain = maxGain;
        maxGain = tmp;
    }

    int64_t absUv = (microvolts == INT32_MIN) ? ((int64_t)INT32_MAX + 1LL)
                                              : (int64_t)(microvolts < 0 ? -microvolts : microvolts);
    int64_t limit = ((int64_t)handle->vrefMicrovolts * (int64_t)headroomPercent) / 100LL;
    uint8_t selected = minGain;
    for (size_t i = 0; i < (sizeof(gains) / sizeof(gains[0])); ++i) {
        uint8_t gain = gains[i];
        if (gain < minGain || gain > maxGain) {
            continue;
        }
        if (absUv == 0 || (absUv * (int64_t)gain) < limit) {
            selected = gain;
        }
    }
    return selected;
}

static ads126xDeviceType_t ads126xAdcDeviceFromId(uint8_t idReg, ads126xDeviceType_t forced)
{
    if (forced != ADS126X_DEVICE_AUTO) {
        return forced;
    }

    switch ((idReg & ADS126X_ID_DEV_ID_MASK) >> ADS126X_ID_DEV_ID_SHIFT) {
    case ADS126X_DEV_ID_ADS1262:
        return ADS126X_DEVICE_ADS1262;
    case ADS126X_DEV_ID_ADS1263:
        return ADS126X_DEVICE_ADS1263;
    default:
        return ADS126X_DEVICE_AUTO;
    }
}

static bool ads126xAdcIsValidForcedType(ads126xDeviceType_t forcedType)
{
    return forcedType == ADS126X_DEVICE_AUTO ||
           forcedType == ADS126X_DEVICE_ADS1262 ||
           forcedType == ADS126X_DEVICE_ADS1263;
}

/* Datasheet checksum: sum(data bytes) + 0x9B, lower 8 bits. */
static uint8_t ads126xAdcChecksum(const uint8_t *data, size_t len)
{
    uint16_t sum = 0x9Bu;
    for (size_t i = 0; i < len; i++) {
        sum += data[i];
    }
    return (uint8_t)sum;
}

/* CRC-8-ATM (HEC): x^8 + x^2 + x + 1, MSB-first. */
static uint8_t ads126xAdcCrc8(const uint8_t *data, size_t len)
{
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; bit++) {
            if (crc & 0x80) {
                crc = (uint8_t)((crc << 1) ^ 0x07);
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

static esp_err_t ads126xAdcSpiTransferLocked(ads126xAdcHandle_t *handle,
                                             const uint8_t *tx,
                                             size_t txLen,
                                             uint8_t *rx,
                                             size_t rxLen)
{
    if (!handle || !handle->spiDevice || !handle->mutex) {
        return ESP_ERR_INVALID_ARG;
    }

    size_t totalLen = txLen + rxLen;
    if (totalLen == 0) {
        return ESP_OK;
    }
    if (totalLen > handle->spiBufSize || !handle->spiTxBuf || !handle->spiRxBuf) {
        return ESP_ERR_INVALID_SIZE;
    }

    if (xSemaphoreTake(handle->mutex, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    if (txLen > 0 && tx) {
        memcpy(handle->spiTxBuf, tx, txLen);
    }
    if (rxLen > 0) {
        memset(handle->spiTxBuf + txLen, ADS126X_SPI_DUMMY_BYTE, rxLen);
    }
    memset(handle->spiRxBuf, 0, totalLen);

    spi_transaction_t trans = {0};
    trans.length = totalLen * 8;
    trans.rxlength = totalLen * 8;
    trans.tx_buffer = handle->spiTxBuf;
    trans.rx_buffer = handle->spiRxBuf;

    /* Keep CS asserted for the full command + data phase. */
    esp_err_t err = spi_device_transmit(handle->spiDevice, &trans);
    if (err == ESP_OK && rx && rxLen > 0) {
        memcpy(rx, handle->spiRxBuf + txLen, rxLen);
    }

    xSemaphoreGive(handle->mutex);
    return err;
}

static bool ads126xAdcIsAdc2Supported(const ads126xAdcHandle_t *handle)
{
#if CONFIG_ADS126X_HAS_ADC2
    return handle && handle->deviceType == ADS126X_DEVICE_ADS1263;
#else
    (void)handle;
    return false;
#endif
}

esp_err_t ads126xAdcInit(ads126xAdcHandle_t *handle, const ads126xAdcConfig_t *cfg)
{
    if (!handle || !cfg || !cfg->spiDevice) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!ads126xAdcIsValidForcedType(cfg->forcedType)) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(handle, 0, sizeof(*handle));
    handle->spiDevice = cfg->spiDevice;
    handle->drdyGpio = cfg->drdyGpio;
    handle->resetGpio = cfg->resetGpio;
    handle->forcedType = cfg->forcedType;
    handle->crcMode = cfg->crcMode;
    handle->enableStatusByte = cfg->enableStatusByte;
    handle->enableInternalRef = cfg->enableInternalRef;
    handle->vrefMicrovolts = cfg->vrefMicrovolts ? cfg->vrefMicrovolts : ADS126X_ADC_DEFAULT_VREF_UV;
    handle->pgaGain = cfg->pgaGain;
    handle->dataRateDr = cfg->dataRateDr;
    handle->drdyTimeoutMs = ADS126X_ADC_DEFAULT_DRDY_TIMEOUT_MS;

    handle->mutex = xSemaphoreCreateMutex();
    if (!handle->mutex) {
        return ESP_ERR_NO_MEM;
    }

    esp_err_t err = ads126xAdcAllocSpiBuffers(handle);
    if (err != ESP_OK) {
        ads126xAdcDeinit(handle);
        return err;
    }

    if (handle->resetGpio != GPIO_NUM_NC) {
        gpio_set_direction(handle->resetGpio, GPIO_MODE_OUTPUT);
        gpio_set_level(handle->resetGpio, 1);
    }
    if (handle->drdyGpio != GPIO_NUM_NC) {
        gpio_set_direction(handle->drdyGpio, GPIO_MODE_INPUT);
    }

    if (handle->resetGpio != GPIO_NUM_NC) {
        err = ads126xAdcHardwareReset(handle);
        if (err != ESP_OK) {
            ads126xAdcDeinit(handle);
            return err;
        }
    }

    err = ads126xAdcSendCommand(handle, ADS126X_CMD_RESET);
    if (err != ESP_OK) {
        ads126xAdcDeinit(handle);
        return err;
    }

    /*
     * Datasheet minimum is 8 * tCLK after RESET command before next command.
     * Use millisecond delay for margin across clock options and RTOS tick quantization.
     */
    vTaskDelay(pdMS_TO_TICKS(ADS126X_RESET_COMMAND_DELAY_MS));

    bool idReadOk = false;
    err = ads126xAdcGetIdRaw(handle, &handle->idRegRaw);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to read ID register, err=%d", err);
    } else {
        idReadOk = true;
    }

    ads126xDeviceType_t detectedType = ADS126X_DEVICE_AUTO;
    if (idReadOk) {
        detectedType = ads126xAdcDeviceFromId(handle->idRegRaw, ADS126X_DEVICE_AUTO);
        if (detectedType == ADS126X_DEVICE_AUTO) {
            ESP_LOGW(TAG, "Unknown ADS126x ID register value: 0x%02X", handle->idRegRaw);
        }
    }

    if (handle->forcedType != ADS126X_DEVICE_AUTO) {
        handle->deviceType = handle->forcedType;
        if (!idReadOk) {
            ESP_LOGW(TAG,
                     "Using forced device type %d with unreadable ID register",
                     (int)handle->forcedType);
        } else if (detectedType == ADS126X_DEVICE_AUTO) {
            ESP_LOGW(TAG,
                     "Using forced device type %d with unknown ID 0x%02X",
                     (int)handle->forcedType,
                     handle->idRegRaw);
        } else if (detectedType != handle->forcedType) {
            ESP_LOGW(TAG,
                     "Forced device type %d mismatches ID 0x%02X (detected type %d)",
                     (int)handle->forcedType,
                     handle->idRegRaw,
                     (int)detectedType);
        }
    } else if (!idReadOk || detectedType == ADS126X_DEVICE_AUTO) {
        /* Keep AUTO when ID cannot be trusted to avoid mis-classifying as ADS1262. */
        handle->deviceType = ADS126X_DEVICE_AUTO;
        if (!idReadOk) {
            ESP_LOGW(TAG, "ID register unavailable; keeping device type AUTO");
        } else {
            ESP_LOGW(TAG, "ID 0x%02X not recognized; keeping device type AUTO", handle->idRegRaw);
        }
    } else {
        handle->deviceType = detectedType;
    }

    err = ads126xAdcConfigure(handle,
                              cfg->enableInternalRef,
                              cfg->enableStatusByte,
                              cfg->crcMode,
                              cfg->pgaGain,
                              cfg->dataRateDr);
    if (err != ESP_OK) {
        ads126xAdcDeinit(handle);
        return err;
    }

    if (cfg->enableInternalRef) {
        /* Select internal reference on REFMUX when INTREF is enabled. */
        err = ads126xAdcSetRefMux(handle, 0x00);
        if (err != ESP_OK) {
            ads126xAdcDeinit(handle);
            return err;
        }
    }

    /* Default to AIN0/AIN1; applications should set the real channel. */
    err = ads126xAdcSetInputMux(handle, 0x00, 0x01);
    if (err != ESP_OK) {
        ads126xAdcDeinit(handle);
        return err;
    }

    if (cfg->enableInternalRef) {
        /*
         * REFOUT startup is datasheet-sensitive; with 1-uF reference capacitor, allow
         * conservative settling time so the first conversion is less likely to be invalid.
         */
        vTaskDelay(pdMS_TO_TICKS(ADS126X_INTERNAL_REF_SETTLE_MS));
    }

    return ESP_OK;
}

esp_err_t ads126xAdcDeinit(ads126xAdcHandle_t *handle)
{
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }

    if (handle->mutex) {
        vSemaphoreDelete(handle->mutex);
    }
    if (handle->spiTxBuf) {
        heap_caps_free(handle->spiTxBuf);
    }
    if (handle->spiRxBuf) {
        heap_caps_free(handle->spiRxBuf);
    }

    memset(handle, 0, sizeof(*handle));
    return ESP_OK;
}

esp_err_t ads126xAdcHardwareReset(ads126xAdcHandle_t *handle)
{
    if (!handle || handle->resetGpio == GPIO_NUM_NC) {
        return ESP_ERR_NOT_SUPPORTED;
    }

    /* RESET/PWDN is active low; hold low briefly then release high. */
    gpio_set_level(handle->resetGpio, 0);
    vTaskDelay(pdMS_TO_TICKS(ADS126X_RESET_PULSE_LOW_MS));
    gpio_set_level(handle->resetGpio, 1);
    vTaskDelay(pdMS_TO_TICKS(ADS126X_RESET_RELEASE_WAIT_MS));

    return ESP_OK;
}

esp_err_t ads126xAdcSendCommand(ads126xAdcHandle_t *handle, uint8_t cmd)
{
    return ads126xAdcSpiTransferLocked(handle, &cmd, 1, NULL, 0);
}

esp_err_t ads126xAdcReadRegisters(ads126xAdcHandle_t *handle, uint8_t startAddr, uint8_t *data, size_t len)
{
    if (!handle || !data || len == 0 || len > ADS126X_MAX_REG_READ_LEN) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t cmd[2] = {
        (uint8_t)(ADS126X_CMD_RREG | (startAddr & 0x1Fu)),
        (uint8_t)(len - 1u),
    };

    return ads126xAdcSpiTransferLocked(handle, cmd, sizeof(cmd), data, len);
}

esp_err_t ads126xAdcWriteRegisters(ads126xAdcHandle_t *handle, uint8_t startAddr, const uint8_t *data, size_t len)
{
    if (!handle || !data || len == 0 || len > ADS126X_MAX_REG_READ_LEN) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t cmd[2] = {
        (uint8_t)(ADS126X_CMD_WREG | (startAddr & 0x1Fu)),
        (uint8_t)(len - 1u),
    };

    /* Combine opcode + data in a single CS assertion. */
    uint8_t temp[2 + ADS126X_MAX_REG_READ_LEN];
    memcpy(temp, cmd, sizeof(cmd));
    memcpy(temp + sizeof(cmd), data, len);

    return ads126xAdcSpiTransferLocked(handle, temp, sizeof(cmd) + len, NULL, 0);
}

esp_err_t ads126xAdcGetIdRaw(ads126xAdcHandle_t *handle, uint8_t *idReg)
{
    return ads126xAdcReadRegisters(handle, ADS126X_REG_ID, idReg, 1);
}

esp_err_t ads126xAdcConfigure(ads126xAdcHandle_t *handle,
                              bool enableInternalRef,
                              bool enableStatusByte,
                              ads126xCrcMode_t crcMode,
                              uint8_t pgaGain,
                              uint8_t dataRateDr)
{
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }
    if (crcMode > ADS126X_CRC_CRC8 || dataRateDr > ADS126X_MODE2_DR_MASK) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t gainCode = 0;
    if (!ads126xAdcGainToCode(pgaGain, &gainCode)) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Read-modify-write to preserve VBIAS and reserved bits. */
    uint8_t power = 0;
    esp_err_t err = ads126xAdcReadRegisters(handle, ADS126X_REG_POWER, &power, 1);
    if (err != ESP_OK) {
        return err;
    }
    if (enableInternalRef) {
        power |= ADS126X_POWER_INTREF;
    } else {
        power &= (uint8_t)~ADS126X_POWER_INTREF;
    }
    err = ads126xAdcWriteRegisters(handle, ADS126X_REG_POWER, &power, 1);
    if (err != ESP_OK) {
        return err;
    }

    uint8_t iface = 0;
    err = ads126xAdcReadRegisters(handle, ADS126X_REG_INTERFACE, &iface, 1);
    if (err != ESP_OK) {
        return err;
    }
    iface &= (uint8_t)~(ADS126X_INTERFACE_STATUS | ADS126X_INTERFACE_CRC_MASK);
    if (enableStatusByte) {
        iface |= ADS126X_INTERFACE_STATUS;
    }
    iface |= (uint8_t)crcMode;
    err = ads126xAdcWriteRegisters(handle, ADS126X_REG_INTERFACE, &iface, 1);
    if (err != ESP_OK) {
        return err;
    }

    uint8_t mode2 = (uint8_t)((gainCode << ADS126X_MODE2_GAIN_SHIFT) | (dataRateDr & ADS126X_MODE2_DR_MASK));
    mode2 &= (uint8_t)~ADS126X_MODE2_BYPASS;
    err = ads126xAdcWriteRegisters(handle, ADS126X_REG_MODE2, &mode2, 1);
    if (err != ESP_OK) {
        return err;
    }

    handle->enableInternalRef = enableInternalRef;
    handle->enableStatusByte = enableStatusByte;
    handle->crcMode = crcMode;
    handle->pgaGain = pgaGain;
    handle->dataRateDr = dataRateDr;

    return ESP_OK;
}

esp_err_t ads126xAdcConfigureVoltageMode(ads126xAdcHandle_t *handle,
                                         uint8_t gain,
                                         uint8_t dataRateDr,
                                         bool enableStatusByte,
                                         bool enableCrc)
{
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!ads126xAdcIsValidGain(gain) || dataRateDr > ADS126X_MODE2_DR_MASK) {
        return ESP_ERR_INVALID_ARG;
    }

    return ads126xAdcConfigure(handle,
                               handle->enableInternalRef,
                               enableStatusByte,
                               enableCrc ? ADS126X_CRC_CRC8 : ADS126X_CRC_OFF,
                               gain,
                               dataRateDr);
}

esp_err_t ads126xAdcSetRefMux(ads126xAdcHandle_t *handle, uint8_t refmuxValue)
{
    return ads126xAdcWriteRegisters(handle, ADS126X_REG_REFMUX, &refmuxValue, 1);
}

esp_err_t ads126xAdcSetInputMux(ads126xAdcHandle_t *handle, uint8_t muxp, uint8_t muxn)
{
    uint8_t value = (uint8_t)(((muxp & 0x0Fu) << 4) | (muxn & 0x0Fu));
    return ads126xAdcWriteRegisters(handle, ADS126X_REG_INPMUX, &value, 1);
}

esp_err_t ads126xAdcSetVbiasEnabled(ads126xAdcHandle_t *handle, bool enableVbias)
{
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t power = 0u;
    esp_err_t err = ads126xAdcReadRegisters(handle, ADS126X_REG_POWER, &power, 1);
    if (err != ESP_OK) {
        return err;
    }

    if (enableVbias) {
        power |= ADS126X_POWER_VBIAS;
    } else {
        power &= (uint8_t)~ADS126X_POWER_VBIAS;
    }

    return ads126xAdcWriteRegisters(handle, ADS126X_REG_POWER, &power, 1);
}

esp_err_t ads126xAdcReadCoreRegisters(ads126xAdcHandle_t *handle,
                                      uint8_t *outPower,
                                      uint8_t *outInterface,
                                      uint8_t *outMode2,
                                      uint8_t *outInpmux,
                                      uint8_t *outRefmux)
{
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t power = 0u;
    uint8_t iface = 0u;
    uint8_t mode2 = 0u;
    uint8_t inpmux = 0u;
    uint8_t refmux = 0u;

    esp_err_t err = ads126xAdcReadRegisters(handle, ADS126X_REG_POWER, &power, 1);
    if (err == ESP_OK) {
        err = ads126xAdcReadRegisters(handle, ADS126X_REG_INTERFACE, &iface, 1);
    }
    if (err == ESP_OK) {
        err = ads126xAdcReadRegisters(handle, ADS126X_REG_MODE2, &mode2, 1);
    }
    if (err == ESP_OK) {
        err = ads126xAdcReadRegisters(handle, ADS126X_REG_INPMUX, &inpmux, 1);
    }
    if (err == ESP_OK) {
        err = ads126xAdcReadRegisters(handle, ADS126X_REG_REFMUX, &refmux, 1);
    }
    if (err != ESP_OK) {
        return err;
    }

    if (outPower) {
        *outPower = power;
    }
    if (outInterface) {
        *outInterface = iface;
    }
    if (outMode2) {
        *outMode2 = mode2;
    }
    if (outInpmux) {
        *outInpmux = inpmux;
    }
    if (outRefmux) {
        *outRefmux = refmux;
    }
    return ESP_OK;
}

esp_err_t ads126xAdcReadSingleDiffUv(ads126xAdcHandle_t *handle,
                                     uint8_t muxp,
                                     uint8_t muxn,
                                     bool start1EveryRead,
                                     uint32_t settleMs,
                                     uint8_t discardCount,
                                     int32_t *outRaw,
                                     int32_t *outUv,
                                     uint8_t *outStatus)
{
    if (!handle || (!outRaw && !outUv)) {
        return ESP_ERR_INVALID_ARG;
    }

    muxp &= 0x0Fu;
    muxn &= 0x0Fu;
    if (outRaw) {
        *outRaw = 0;
    }
    if (outUv) {
        *outUv = 0;
    }
    if (outStatus) {
        *outStatus = 0u;
    }

    esp_err_t err = ads126xAdcSetInputMux(handle, muxp, muxn);
    if (err != ESP_OK) {
        return err;
    }

    if (settleMs > 0u) {
        vTaskDelay(pdMS_TO_TICKS(settleMs));
    }

    if (start1EveryRead) {
        err = ads126xAdcStartAdc1(handle);
        if (err != ESP_OK) {
            return err;
        }
    }

    for (uint8_t discardIdx = 0u; discardIdx < discardCount; ++discardIdx) {
        int32_t discardRaw = 0;
        err = ads126xAdcReadAdc1Raw(handle, &discardRaw, NULL);
        if (err != ESP_OK) {
            return err;
        }
    }

    int32_t raw = 0;
    uint8_t statusByte = 0u;
    err = ads126xAdcReadAdc1Raw(handle, &raw, &statusByte);
    if (err != ESP_OK) {
        return err;
    }

    if (outRaw) {
        *outRaw = raw;
    }
    if (outUv) {
        *outUv = ads126xAdcRawToMicrovolts(handle, raw);
    }
    if (outStatus) {
        *outStatus = statusByte;
    }
    return ESP_OK;
}

esp_err_t ads126xAdcStartAdc1(ads126xAdcHandle_t *handle)
{
    return ads126xAdcSendCommand(handle, ADS126X_CMD_START1);
}

esp_err_t ads126xAdcStopAdc1(ads126xAdcHandle_t *handle)
{
    return ads126xAdcSendCommand(handle, ADS126X_CMD_STOP1);
}

esp_err_t ads126xAdcWaitDrdy(ads126xAdcHandle_t *handle, uint32_t timeoutMs)
{
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }

    if (handle->drdyGpio == GPIO_NUM_NC) {
        if (timeoutMs > 0) {
            vTaskDelay(pdMS_TO_TICKS(timeoutMs));
        }
        return ESP_OK;
    }

    TickType_t start = xTaskGetTickCount();
    TickType_t timeoutTicks = (timeoutMs > 0) ? pdMS_TO_TICKS(timeoutMs) : 0;
    while (gpio_get_level(handle->drdyGpio) != 0) {
        if (timeoutTicks == 0) {
            return ESP_ERR_TIMEOUT;
        }
        if ((xTaskGetTickCount() - start) >= timeoutTicks) {
            return ESP_ERR_TIMEOUT;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    return ESP_OK;
}

esp_err_t ads126xAdcWaitDrdyFastUs(ads126xAdcHandle_t *handle, uint32_t timeoutUs)
{
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }

    if (handle->drdyGpio == GPIO_NUM_NC) {
        if (timeoutUs > 0u) {
            esp_rom_delay_us(timeoutUs);
        }
        return ESP_OK;
    }

    int64_t startUs = esp_timer_get_time();
    uint32_t pollCount = 0u;
    while (gpio_get_level(handle->drdyGpio) != 0) {
        if (timeoutUs == 0u || (uint64_t)(esp_timer_get_time() - startUs) >= (uint64_t)timeoutUs) {
            return ESP_ERR_TIMEOUT;
        }
        pollCount++;
        if ((pollCount % ADS126X_DRDY_FAST_YIELD_INTERVAL) == 0u) {
            taskYIELD();
        }
    }

    return ESP_OK;
}

static esp_err_t ads126xAdcReadAdc1RawAfterDrdy(ads126xAdcHandle_t *handle,
                                                int32_t *rawCode,
                                                uint8_t *statusByteOptional)
{
    if (!handle || !rawCode) {
        return ESP_ERR_INVALID_ARG;
    }

    size_t frameLen = 4;
    if (handle->enableStatusByte) {
        frameLen += 1;
    }
    if (handle->crcMode != ADS126X_CRC_OFF) {
        frameLen += 1;
    }

    uint8_t frame[6] = {0};
    uint8_t cmd = ADS126X_CMD_RDATA1;
    esp_err_t err = ads126xAdcSpiTransferLocked(handle, &cmd, 1, frame, frameLen);
    if (err != ESP_OK) {
        return err;
    }

    size_t idx = 0;
    if (handle->enableStatusByte) {
        if (statusByteOptional) {
            *statusByteOptional = frame[idx];
        }
        idx++;
    }

    uint8_t *dataBytes = &frame[idx];
    if (handle->crcMode != ADS126X_CRC_OFF) {
        uint8_t expected = frame[frameLen - 1];
        uint8_t actual = (handle->crcMode == ADS126X_CRC_CRC8)
                             ? ads126xAdcCrc8(dataBytes, 4)
                             : ads126xAdcChecksum(dataBytes, 4);
        if (expected != actual) {
            ESP_LOGW(TAG, "ADC1 CRC/CHK mismatch (exp=0x%02X calc=0x%02X)", expected, actual);
            return ESP_ERR_INVALID_RESPONSE;
        }
    }

    uint32_t raw = ((uint32_t)dataBytes[0] << 24) |
                   ((uint32_t)dataBytes[1] << 16) |
                   ((uint32_t)dataBytes[2] << 8) |
                   (uint32_t)dataBytes[3];
    *rawCode = (int32_t)raw;

    return ESP_OK;
}

esp_err_t ads126xAdcReadAdc1Raw(ads126xAdcHandle_t *handle,
                                int32_t *rawCode,
                                uint8_t *statusByteOptional)
{
    if (!handle || !rawCode) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = ads126xAdcWaitDrdy(handle, handle->drdyTimeoutMs);
    if (err != ESP_OK) {
        return err;
    }

    return ads126xAdcReadAdc1RawAfterDrdy(handle, rawCode, statusByteOptional);
}

int32_t ads126xAdcRawToMicrovolts(const ads126xAdcHandle_t *handle, int32_t rawCode)
{
    if (!handle || handle->pgaGain == 0 || handle->vrefMicrovolts == 0) {
        return 0;
    }

    /* V = code * Vref / (Gain * 2^31). */
    int64_t numerator = (int64_t)rawCode * (int64_t)handle->vrefMicrovolts;
    int64_t denominator = (int64_t)handle->pgaGain * (1LL << 31);
    return (int32_t)(numerator / denominator);
}

esp_err_t ads126xAdcReadVoltageMicrovoltsFast(ads126xAdcHandle_t *handle,
                                              const ads126xAdcVoltageReadConfig_t *cfg,
                                              ads126xAdcVoltageSample_t *outSample)
{
    if (!handle || !cfg || !outSample) {
        return ESP_ERR_INVALID_ARG;
    }

    *outSample = (ads126xAdcVoltageSample_t){
        .gain = handle->pgaGain,
        .err = ESP_FAIL,
    };

    uint8_t oversampleCount = cfg->oversampleCount;
    if (oversampleCount == 0u) {
        oversampleCount = 1u;
    }

    esp_err_t err = ESP_OK;
    if (cfg->stopBeforeMuxChange) {
        err = ads126xAdcStopAdc1(handle);
        if (err != ESP_OK) {
            outSample->err = err;
            return err;
        }
    }

    err = ads126xAdcSetInputMux(handle, (uint8_t)(cfg->muxp & 0x0Fu), (uint8_t)(cfg->muxn & 0x0Fu));
    if (err != ESP_OK) {
        outSample->err = err;
        return err;
    }

    if (cfg->settleAfterMuxUs > 0u) {
        esp_rom_delay_us(cfg->settleAfterMuxUs);
    }

    if (cfg->startAfterMuxChange) {
        err = ads126xAdcStartAdc1(handle);
        if (err != ESP_OK) {
            outSample->err = err;
            return err;
        }
    }

    if (cfg->discardFirst) {
        int32_t discardRaw = 0;
        err = ads126xAdcWaitDrdyFastUs(handle, cfg->drdyTimeoutUs);
        if (err == ESP_OK) {
            err = ads126xAdcReadAdc1RawAfterDrdy(handle, &discardRaw, NULL);
        }
        if (err != ESP_OK) {
            outSample->err = err;
            return err;
        }
    }

    int64_t rawSum = 0;
    uint8_t samplesUsed = 0u;
    int32_t lastRaw = 0;
    for (uint8_t i = 0u; i < oversampleCount; ++i) {
        int32_t raw = 0;
        err = ads126xAdcWaitDrdyFastUs(handle, cfg->drdyTimeoutUs);
        if (err == ESP_OK) {
            err = ads126xAdcReadAdc1RawAfterDrdy(handle, &raw, NULL);
        }
        if (err != ESP_OK) {
            outSample->err = err;
            outSample->samplesUsed = samplesUsed;
            return err;
        }
        rawSum += (int64_t)raw;
        lastRaw = raw;
        samplesUsed++;
    }

    int32_t averagedRaw = (samplesUsed > 0u) ? (int32_t)(rawSum / (int64_t)samplesUsed) : lastRaw;
    outSample->rawCode = averagedRaw;
    outSample->microvolts = ads126xAdcRawToMicrovolts(handle, averagedRaw);
    outSample->gain = handle->pgaGain;
    outSample->samplesUsed = samplesUsed;
    outSample->clippedOrNearFullScale = ads126xAdcRawNearFullScale(averagedRaw);
    outSample->err = ESP_OK;
    return ESP_OK;
}

esp_err_t ads126xAdcSelectAutoGainForVoltage(ads126xAdcHandle_t *handle,
                                             const ads126xAdcVoltageReadConfig_t *readCfg,
                                             const ads126xAdcAutoGainConfig_t *gainCfg,
                                             ads126xAdcAutoGainResult_t *outResult)
{
    if (!handle || !readCfg || !gainCfg || !outResult) {
        return ESP_ERR_INVALID_ARG;
    }

    *outResult = (ads126xAdcAutoGainResult_t){
        .selectedGain = ADS126X_GAIN_1,
        .err = ESP_FAIL,
    };

    uint8_t minGain = ads126xAdcNormalizeGainOrDefault(gainCfg->minGain, ADS126X_GAIN_1);
    uint8_t maxGain = ads126xAdcNormalizeGainOrDefault(gainCfg->maxGain, ADS126X_GAIN_32);
    if (minGain > maxGain) {
        uint8_t tmp = minGain;
        minGain = maxGain;
        maxGain = tmp;
    }

    uint8_t candidateGain = ads126xAdcNormalizeGainOrDefault(gainCfg->initialGain, ADS126X_GAIN_1);
    if (candidateGain < minGain) {
        candidateGain = minGain;
    }
    if (candidateGain > maxGain) {
        candidateGain = maxGain;
    }

    uint8_t headroomPercent = gainCfg->headroomPercent;
    if (headroomPercent < 1u || headroomPercent > 100u) {
        headroomPercent = 75u;
    }

    uint8_t maxIterations = gainCfg->maxIterations ? gainCfg->maxIterations : 4u;
    uint8_t originalGain = handle->pgaGain;
    uint8_t originalDr = handle->dataRateDr;
    bool originalStatus = handle->enableStatusByte;
    ads126xCrcMode_t originalCrc = handle->crcMode;
    esp_err_t err = ESP_OK;

    for (uint8_t iteration = 0u; iteration < maxIterations; ++iteration) {
        err = ads126xAdcConfigure(handle,
                                  handle->enableInternalRef,
                                  handle->enableStatusByte,
                                  handle->crcMode,
                                  candidateGain,
                                  handle->dataRateDr);
        if (err != ESP_OK) {
            outResult->err = err;
            break;
        }
        if (gainCfg->settleAfterGainChangeUs > 0u) {
            esp_rom_delay_us(gainCfg->settleAfterGainChangeUs);
        }

        ads126xAdcVoltageSample_t sample = {0};
        err = ads126xAdcReadVoltageMicrovoltsFast(handle, readCfg, &sample);
        outResult->iterations = (uint8_t)(iteration + 1u);
        outResult->selectedGain = candidateGain;
        outResult->sampleRaw = sample.rawCode;
        outResult->sampleMicrovolts = sample.microvolts;
        outResult->clippedOrNearFullScale = sample.clippedOrNearFullScale;
        outResult->err = err;
        if (err != ESP_OK) {
            break;
        }

        uint8_t selectedGain = ads126xAdcSelectGainForUv(handle,
                                                         sample.microvolts,
                                                         minGain,
                                                         maxGain,
                                                         headroomPercent);
        if (sample.clippedOrNearFullScale) {
            uint8_t lower = ads126xAdcNextLowerGain(candidateGain);
            if (lower < minGain) {
                lower = minGain;
            }
            if (selectedGain >= candidateGain) {
                selectedGain = lower;
            }
        }

        outResult->selectedGain = selectedGain;
        if (selectedGain == candidateGain) {
            err = ESP_OK;
            outResult->err = ESP_OK;
            break;
        }
        candidateGain = selectedGain;
    }

    if (err == ESP_OK && gainCfg->keepConfiguredOnSuccess) {
        if (handle->pgaGain != outResult->selectedGain) {
            err = ads126xAdcConfigure(handle,
                                      handle->enableInternalRef,
                                      handle->enableStatusByte,
                                      handle->crcMode,
                                      outResult->selectedGain,
                                      handle->dataRateDr);
            outResult->err = err;
        }
    } else {
        esp_err_t restoreErr = ads126xAdcConfigure(handle,
                                                   handle->enableInternalRef,
                                                   originalStatus,
                                                   originalCrc,
                                                   ads126xAdcNormalizeGainOrDefault(originalGain, ADS126X_GAIN_1),
                                                   originalDr);
        if (err == ESP_OK && restoreErr != ESP_OK) {
            err = restoreErr;
            outResult->err = restoreErr;
        }
    }

    return outResult->err;
}

esp_err_t ads126xAdcSelfOffsetCal(ads126xAdcHandle_t *handle)
{
    return ads126xAdcSendCommand(handle, ADS126X_CMD_SFOCAL1);
}

esp_err_t ads126xAdcSelfGainCal(ads126xAdcHandle_t *handle)
{
    /* ADS126x does not offer a dedicated self-gain command; map to system gain. */
    return ads126xAdcSendCommand(handle, ADS126X_CMD_SYGCAL1);
}

esp_err_t ads126xAdcSystemOffsetCal(ads126xAdcHandle_t *handle)
{
    return ads126xAdcSendCommand(handle, ADS126X_CMD_SYOCAL1);
}

esp_err_t ads126xAdcSystemGainCal(ads126xAdcHandle_t *handle)
{
    return ads126xAdcSendCommand(handle, ADS126X_CMD_SYGCAL1);
}

esp_err_t ads126xAdcSelfCal(ads126xAdcHandle_t *handle)
{
    /* Run offset then gain; the gain step still requires an external full-scale input. */
    esp_err_t err = ads126xAdcSelfOffsetCal(handle);
    if (err != ESP_OK) {
        return err;
    }
    err = ads126xAdcWaitDrdy(handle, handle->drdyTimeoutMs);
    if (err != ESP_OK) {
        return err;
    }
    err = ads126xAdcSelfGainCal(handle);
    if (err != ESP_OK) {
        return err;
    }
    return ads126xAdcWaitDrdy(handle, handle->drdyTimeoutMs);
}

esp_err_t ads126xAdcStartAdc2(ads126xAdcHandle_t *handle)
{
    if (!ads126xAdcIsAdc2Supported(handle)) {
        return ESP_ERR_NOT_SUPPORTED;
    }
    return ads126xAdcSendCommand(handle, ADS126X_CMD_START2);
}

esp_err_t ads126xAdcStopAdc2(ads126xAdcHandle_t *handle)
{
    if (!ads126xAdcIsAdc2Supported(handle)) {
        return ESP_ERR_NOT_SUPPORTED;
    }
    return ads126xAdcSendCommand(handle, ADS126X_CMD_STOP2);
}

esp_err_t ads126xAdcReadAdc2Raw(ads126xAdcHandle_t *handle,
                                int32_t *raw24,
                                uint8_t *statusOptional)
{
    if (!handle || !raw24) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!ads126xAdcIsAdc2Supported(handle)) {
        return ESP_ERR_NOT_SUPPORTED;
    }

    esp_err_t err = ads126xAdcWaitDrdy(handle, handle->drdyTimeoutMs);
    if (err != ESP_OK) {
        return err;
    }

    /*
     * ADS1263 ADC2 data-byte sequence (RDATA2):
     *   [status?] [data2_msb] [data2_mid] [data2_lsb] [pad=0x00] [crc/chk?]
     * If status is disabled, bytes are left-shifted and start at data2_msb.
     */
    size_t frameLen = 4; /* 3 ADC2 data bytes + 1 mandatory zero pad byte. */
    if (handle->enableStatusByte) {
        frameLen += 1;
    }
    if (handle->crcMode != ADS126X_CRC_OFF) {
        frameLen += 1;
    }

    uint8_t frame[6] = {0};
    uint8_t cmd = ADS126X_CMD_RDATA2;
    err = ads126xAdcSpiTransferLocked(handle, &cmd, 1, frame, frameLen);
    if (err != ESP_OK) {
        return err;
    }

    size_t idx = 0;
    if (handle->enableStatusByte) {
        if (statusOptional) {
            *statusOptional = frame[idx];
        }
        idx++;
    }

    uint8_t *dataBytes = &frame[idx];
    uint8_t padByte = dataBytes[3];
    if (padByte != 0x00u) {
        ESP_LOGW(TAG, "ADC2 pad byte unexpected: 0x%02X", padByte);
    }

    if (handle->crcMode != ADS126X_CRC_OFF) {
        uint8_t expected = frame[frameLen - 1];
        uint8_t actual = (handle->crcMode == ADS126X_CRC_CRC8)
                              ? ads126xAdcCrc8(dataBytes, 3)
                              : ads126xAdcChecksum(dataBytes, 3);
        if (expected != actual) {
            ESP_LOGW(TAG, "ADC2 CRC/CHK mismatch (exp=0x%02X calc=0x%02X)", expected, actual);
            return ESP_ERR_INVALID_RESPONSE;
        }
    }

    uint32_t raw = ((uint32_t)dataBytes[0] << 16) |
                   ((uint32_t)dataBytes[1] << 8) |
                   (uint32_t)dataBytes[2];
    /* ADC2 output code is 24-bit two's complement; sign-extend to int32_t. */
    if (raw & 0x00800000u) {
        raw |= 0xFF000000u;
    }
    *raw24 = (int32_t)raw;

    return ESP_OK;
}

#if CONFIG_ADS126X_HELPER_CREATE_SPI
esp_err_t ads126xAdcHelperCreateSpiDevice(spi_device_handle_t *outDevice)
{
    if (!outDevice) {
        return ESP_ERR_INVALID_ARG;
    }

    spi_bus_config_t busCfg = {
        .mosi_io_num = CONFIG_BOARD_SPI_MOSI_GPIO,
        .miso_io_num = CONFIG_BOARD_SPI_MISO_GPIO,
        .sclk_io_num = CONFIG_BOARD_SPI_SCLK_GPIO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = (int)ADS126X_SPI_MAX_TRANSFER_BYTES,
    };

#if CONFIG_SENSORARRAY_SPI_USE_DMA
    int dmaChan = SPI_DMA_CH_AUTO;
#else
    int dmaChan = 0;
#endif

    esp_err_t err = spi_bus_initialize(CONFIG_BOARD_SPI_HOST, &busCfg, dmaChan);
    if (err == ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "SPI bus already initialized");
        err = ESP_OK;
    }
    if (err != ESP_OK) {
        return err;
    }

    spi_device_interface_config_t devCfg = {
        .clock_speed_hz = CONFIG_ADS126X_SPI_CLOCK_HZ,
        .mode = 1,
        .spics_io_num = CONFIG_BOARD_ADS126X_CS_GPIO,
        .queue_size = 1,
    };

    return spi_bus_add_device(CONFIG_BOARD_SPI_HOST, &devCfg, outDevice);
}

esp_err_t ads126xAdcHelperDestroySpiDevice(spi_device_handle_t device)
{
    if (!device) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t err = spi_bus_remove_device(device);
    if (err != ESP_OK) {
        return err;
    }
    return spi_bus_free(CONFIG_BOARD_SPI_HOST);
}
#endif

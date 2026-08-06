#include "ads126xAdc.h"

#include <stdio.h>
#include <string.h>

#include "sdkconfig.h"
#include "esp_attr.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_memory_utils.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "freertos/task.h"
#include "boardSupport.h"

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
#define ADS126X_REG_OFCAL0 0x07
#define ADS126X_REG_FSCAL0 0x0A
#define ADS126X_REG_REFMUX 0x0F
#define ADS126X_REG_ADC2CFG 0x15
#define ADS126X_REG_ADC2MUX 0x16
#define ADS126X_REG_ADC2OFC0 0x17
#define ADS126X_REG_ADC2FSC0 0x19

/* INTERFACE register bit definitions. */
#define ADS126X_INTERFACE_STATUS (1u << 2)
#define ADS126X_INTERFACE_CRC_MASK 0x03u

/* MODE2 register bit definitions. */
#define ADS126X_MODE2_BYPASS (1u << 7)
#define ADS126X_MODE2_GAIN_SHIFT 4
#define ADS126X_MODE2_GAIN_MASK (0x07u << ADS126X_MODE2_GAIN_SHIFT)
#define ADS126X_MODE2_DR_MASK 0x0Fu
#define ADS126X_MODE1_FILTER_MASK 0xE0u

#ifndef CONFIG_ADS126X_MUTEX_TIMEOUT_MS
#define CONFIG_ADS126X_MUTEX_TIMEOUT_MS 100
#endif

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

static esp_err_t ads126xAdcNoteMode2(ads126xAdcHandle_t *handle,
                                     uint8_t mode2);

static esp_err_t ads126xAdcAllocSpiBuffers(ads126xAdcHandle_t *handle)
{
    size_t bufSize = ADS126X_SPI_MAX_TRANSFER_BYTES;
    if (bufSize < 8u) {
        bufSize = 8u;
    }

    handle->spiTxBuf = heap_caps_malloc(bufSize, MALLOC_CAP_DMA);
    handle->spiRxBuf = heap_caps_malloc(bufSize, MALLOC_CAP_DMA);
    handle->spiDmaCapable = handle->spiTxBuf && handle->spiRxBuf &&
                            esp_ptr_dma_capable(handle->spiTxBuf) &&
                            esp_ptr_dma_capable(handle->spiRxBuf);
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
        handle->spiDmaCapable = false;
    }

    handle->spiBufSize = bufSize;
    return ESP_OK;
}

static bool ads126xAdcGainToCode(uint8_t gain, uint8_t *code)
{
    if (!code) {
        return false;
    }
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

static TickType_t ads126xAdcMutexTimeoutTicks(void)
{
    TickType_t ticks = pdMS_TO_TICKS(CONFIG_ADS126X_MUTEX_TIMEOUT_MS);
    return ticks == 0u ? 1u : ticks;
}

static const char *ads126xAdcPowerReqName(bool update, bool enable)
{
    if (!update) {
        return "KEEP";
    }
    return enable ? "ON" : "OFF";
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

    if (xSemaphoreTake(handle->mutex, ads126xAdcMutexTimeoutTicks()) != pdTRUE) {
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
    handle->adc2Gain = 1u;
    handle->adc2DataRate = 0u;
    handle->adc2Reference = ADS126X_ADC2_REF_INTERNAL;
    handle->adc2VrefMicrovolts = ADS126X_ADC_DEFAULT_VREF_UV;
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

    if (!cfg->skipResetOnInit) {
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
    }

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

    if (idReadOk && detectedType != ADS126X_DEVICE_AUTO) {
        handle->deviceType = detectedType;
        if (handle->forcedType != ADS126X_DEVICE_AUTO &&
            handle->forcedType != detectedType) {
            ESP_LOGW(TAG,
                     "Configured device type %d mismatches ID 0x%02X; using detected type %d",
                     (int)handle->forcedType,
                     handle->idRegRaw,
                     (int)detectedType);
        }
    } else if (handle->forcedType != ADS126X_DEVICE_AUTO) {
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

    if (!cfg->skipConfigureOnInit) {
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
    }

    return ESP_OK;
}

esp_err_t ads126xAdcDeinit(ads126xAdcHandle_t *handle)
{
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }

    if (handle->drdyNotificationReady && handle->drdyGpio != GPIO_NUM_NC) {
        (void)gpio_isr_handler_remove(handle->drdyGpio);
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
    handle->adc1Running = false;
    handle->adc2Running = false;

    return ESP_OK;
}

esp_err_t ads126xAdcSendCommand(ads126xAdcHandle_t *handle, uint8_t cmd)
{
    esp_err_t err = ads126xAdcSpiTransferLocked(handle, &cmd, 1, NULL, 0);
    if (err == ESP_OK && handle) {
        if (cmd == ADS126X_CMD_START1) {
            handle->adc1Running = true;
        } else if (cmd == ADS126X_CMD_STOP1) {
            handle->adc1Running = false;
        } else if (cmd == ADS126X_CMD_START2) {
            handle->adc2Running = true;
        } else if (cmd == ADS126X_CMD_STOP2) {
            handle->adc2Running = false;
        } else if (cmd == ADS126X_CMD_RESET) {
            handle->adc1Running = false;
            handle->adc2Running = false;
        }
    }
    return err;
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

static esp_err_t ads126xAdcSpiTransferDmaLocked(ads126xAdcHandle_t *handle,
                                                const uint8_t *tx,
                                                size_t txLen,
                                                uint8_t *rx,
                                                size_t rxLen)
{
    if (!handle || !handle->spiDevice || !handle->mutex || !handle->spiDmaCapable) {
        return ESP_ERR_INVALID_STATE;
    }
    size_t totalLen = txLen + rxLen;
    if (totalLen == 0u || totalLen > handle->spiBufSize) {
        return totalLen == 0u ? ESP_OK : ESP_ERR_INVALID_SIZE;
    }
    if (xSemaphoreTake(handle->mutex, ads126xAdcMutexTimeoutTicks()) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    if (txLen > 0u && tx) {
        memcpy(handle->spiTxBuf, tx, txLen);
    }
    if (rxLen > 0u) {
        memset(handle->spiTxBuf + txLen, ADS126X_SPI_DUMMY_BYTE, rxLen);
    }
    memset(handle->spiRxBuf, 0, totalLen);
    handle->dmaTransaction = (spi_transaction_t){
        .length = totalLen * 8u,
        .rxlength = totalLen * 8u,
        .tx_buffer = handle->spiTxBuf,
        .rx_buffer = handle->spiRxBuf,
    };

    esp_err_t err = spi_device_queue_trans(handle->spiDevice,
                                           &handle->dmaTransaction,
                                           portMAX_DELAY);
    spi_transaction_t *completed = NULL;
    if (err == ESP_OK) {
        err = spi_device_get_trans_result(handle->spiDevice, &completed, portMAX_DELAY);
    }
    if (err == ESP_OK && completed != &handle->dmaTransaction) {
        err = ESP_ERR_INVALID_STATE;
    }
    if (err == ESP_OK && rx && rxLen > 0u) {
        memcpy(rx, handle->spiRxBuf + txLen, rxLen);
    }
    xSemaphoreGive(handle->mutex);
    return err;
}

static esp_err_t ads126xAdcParseAdc1Frame(ads126xAdcHandle_t *handle,
                                          const uint8_t *frame,
                                          size_t frameLen,
                                          int32_t *rawCode,
                                          uint8_t *statusByteOptional)
{
    if (!handle || !frame || !rawCode) {
        return ESP_ERR_INVALID_ARG;
    }
    size_t idx = 0u;
    if (handle->enableStatusByte) {
        if (frameLen < 5u) {
            return ESP_ERR_INVALID_SIZE;
        }
        if (statusByteOptional) {
            *statusByteOptional = frame[idx];
        }
        idx++;
    }
    if ((idx + 4u) > frameLen) {
        return ESP_ERR_INVALID_SIZE;
    }
    const uint8_t *dataBytes = &frame[idx];
    if (handle->crcMode != ADS126X_CRC_OFF) {
        uint8_t expected = frame[frameLen - 1u];
        uint8_t actual = (handle->crcMode == ADS126X_CRC_CRC8)
                             ? ads126xAdcCrc8(dataBytes, 4u)
                             : ads126xAdcChecksum(dataBytes, 4u);
        if (expected != actual) {
            return ESP_ERR_INVALID_CRC;
        }
    }
    uint32_t raw = ((uint32_t)dataBytes[0] << 24) |
                   ((uint32_t)dataBytes[1] << 16) |
                   ((uint32_t)dataBytes[2] << 8) |
                   (uint32_t)dataBytes[3];
    *rawCode = (int32_t)raw;
    return ESP_OK;
}

esp_err_t ads126xAdcReadPowerRegister(ads126xAdcHandle_t *handle, uint8_t *outPower)
{
    if (!outPower) {
        return ESP_ERR_INVALID_ARG;
    }
    return ads126xAdcReadRegisters(handle, ADS126X_REG_POWER, outPower, 1);
}

esp_err_t ads126xAdcWritePowerRegister(ads126xAdcHandle_t *handle, uint8_t power)
{
    return ads126xAdcWriteRegisters(handle, ADS126X_REG_POWER, &power, 1);
}

esp_err_t ads126xAdcClearResetFlag(ads126xAdcHandle_t *handle)
{
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t power = 0u;
    esp_err_t err = ads126xAdcReadPowerRegister(handle, &power);
    if (err != ESP_OK) {
        return err;
    }
    uint8_t target = (uint8_t)(power & (uint8_t)~ADS126X_POWER_RESET);
    if (target != power) {
        err = ads126xAdcWritePowerRegister(handle, target);
        if (err != ESP_OK) {
            return err;
        }
    }
    uint8_t readback = 0u;
    err = ads126xAdcReadPowerRegister(handle, &readback);
    if (err != ESP_OK) {
        return err;
    }
    return (readback & ADS126X_POWER_RESET) == 0u ?
        ESP_OK : ESP_ERR_INVALID_RESPONSE;
}

esp_err_t ads126xAdcApplyPowerPolicy(ads126xAdcHandle_t *handle,
                                     bool updateInternalRef,
                                     bool enableInternalRef,
                                     bool updateVbias,
                                     bool enableVbias,
                                     uint8_t *outPowerBefore,
                                     uint8_t *outPowerAfter)
{
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t powerBefore = 0u;
    uint8_t powerTarget = 0u;
    uint8_t powerAfter = 0u;

    esp_err_t err = ads126xAdcReadPowerRegister(handle, &powerBefore);
    if (err != ESP_OK) {
        printf("DBGADSPOWER,stage=apply_power_policy,intrefReq=%s,vbiasReq=%s,powerBefore=na,powerTarget=na,"
               "powerAfter=na,err=%ld,status=read_power_error\n",
               ads126xAdcPowerReqName(updateInternalRef, enableInternalRef),
               ads126xAdcPowerReqName(updateVbias, enableVbias),
               (long)err);
        return err;
    }

    powerTarget = powerBefore;
    if (updateInternalRef) {
        if (enableInternalRef) {
            powerTarget |= ADS126X_POWER_INTREF;
        } else {
            powerTarget &= (uint8_t)~ADS126X_POWER_INTREF;
        }
    }
    if (updateVbias) {
        if (enableVbias) {
            powerTarget |= ADS126X_POWER_VBIAS;
        } else {
            powerTarget &= (uint8_t)~ADS126X_POWER_VBIAS;
        }
    }

    if (powerTarget != powerBefore) {
        err = ads126xAdcWritePowerRegister(handle, powerTarget);
        if (err != ESP_OK) {
            printf("DBGADSPOWER,stage=apply_power_policy,intrefReq=%s,vbiasReq=%s,powerBefore=0x%02X,"
                   "powerTarget=0x%02X,powerAfter=na,err=%ld,status=write_power_error\n",
                   ads126xAdcPowerReqName(updateInternalRef, enableInternalRef),
                   ads126xAdcPowerReqName(updateVbias, enableVbias),
                   powerBefore,
                   powerTarget,
                   (long)err);
            return err;
        }
    }

    err = ads126xAdcReadPowerRegister(handle, &powerAfter);
    if (err != ESP_OK) {
        printf("DBGADSPOWER,stage=apply_power_policy,intrefReq=%s,vbiasReq=%s,powerBefore=0x%02X,"
               "powerTarget=0x%02X,powerAfter=na,err=%ld,status=readback_power_error\n",
               ads126xAdcPowerReqName(updateInternalRef, enableInternalRef),
               ads126xAdcPowerReqName(updateVbias, enableVbias),
               powerBefore,
               powerTarget,
               (long)err);
        return err;
    }

    bool policyOk = true;
    if (updateInternalRef) {
        policyOk = policyOk && (((powerAfter & ADS126X_POWER_INTREF) != 0u) == enableInternalRef);
    }
    if (updateVbias) {
        policyOk = policyOk && (((powerAfter & ADS126X_POWER_VBIAS) != 0u) == enableVbias);
    }

    if (outPowerBefore) {
        *outPowerBefore = powerBefore;
    }
    if (outPowerAfter) {
        *outPowerAfter = powerAfter;
    }
    handle->enableInternalRef = (powerAfter & ADS126X_POWER_INTREF) != 0u;

    /* Successful policy changes are frequent during safe resistance row
     * switching.  MODE?/STATE? provide the authoritative snapshot, so only a
     * failed readback is logged here to avoid synchronous per-row output. */
    if (!policyOk) {
        printf("DBGADSPOWER,stage=apply_power_policy,intrefReq=%s,vbiasReq=%s,powerBefore=0x%02X,powerTarget=0x%02X,"
               "powerAfter=0x%02X,err=%ld,status=policy_mismatch\n",
               ads126xAdcPowerReqName(updateInternalRef, enableInternalRef),
               ads126xAdcPowerReqName(updateVbias, enableVbias),
               powerBefore,
               powerTarget,
               powerAfter,
               (long)ESP_ERR_INVALID_STATE);
    }

    return policyOk ? ESP_OK : ESP_ERR_INVALID_STATE;
}

esp_err_t ads126xAdcSetInternalReference(ads126xAdcHandle_t *handle, bool enableInternalRef)
{
    return ads126xAdcApplyPowerPolicy(handle, true, enableInternalRef, false, false, NULL, NULL);
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

    uint8_t mode2 = 0u;
    if (!ads126xAdcBuildMode2(false, pgaGain, dataRateDr, &mode2)) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t powerBefore = 0u;
    uint8_t powerAfter = 0u;
    esp_err_t err = ads126xAdcApplyPowerPolicy(handle,
                                               true,
                                               enableInternalRef,
                                               false,
                                               false,
                                               &powerBefore,
                                               &powerAfter);
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

    err = ads126xAdcConfigureAdc1Mode(handle,
                                      ADS126X_MODE0_CONTINUOUS_CHOP_OFF_DELAY_0,
                                      ADS126X_MODE1_FILTER_SINC1);
    if (err != ESP_OK) {
        return err;
    }
    err = ads126xAdcWriteRegisters(handle, ADS126X_REG_MODE2, &mode2, 1);
    if (err != ESP_OK) {
        return err;
    }

    uint8_t ifaceReadback = 0u;
    uint8_t mode2Readback = 0u;
    err = ads126xAdcReadRegisters(handle, ADS126X_REG_INTERFACE, &ifaceReadback, 1u);
    if (err == ESP_OK) {
        err = ads126xAdcReadRegisters(handle, ADS126X_REG_MODE2, &mode2Readback, 1u);
    }
    if (err != ESP_OK) {
        return err;
    }
    if (ifaceReadback != iface || mode2Readback != mode2) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    handle->enableInternalRef = enableInternalRef;
    handle->enableStatusByte = enableStatusByte;
    handle->crcMode = crcMode;
    handle->pgaGain = pgaGain;
    handle->dataRateDr = dataRateDr;
    handle->pgaBypassed = false;

    return ESP_OK;
}

esp_err_t ads126xAdcSetRefMux(ads126xAdcHandle_t *handle, uint8_t refmuxValue)
{
    /* Register-only helper. Use ads126xAdcSetRefMuxWithVref() before voltage conversion. */
    return ads126xAdcWriteRegisters(handle, ADS126X_REG_REFMUX, &refmuxValue, 1);
}

esp_err_t ads126xAdcSetRefMuxWithVref(ads126xAdcHandle_t *handle,
                                      uint8_t refmuxValue,
                                      uint32_t vrefMicrovolts)
{
    if (!handle || vrefMicrovolts == 0u) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t err = ads126xAdcSetRefMux(handle, refmuxValue);
    uint8_t readback = 0u;
    if (err == ESP_OK) {
        err = ads126xAdcReadRegisters(handle, ADS126X_REG_REFMUX, &readback, 1u);
    }
    if (err == ESP_OK && readback != refmuxValue) {
        err = ESP_ERR_INVALID_RESPONSE;
    }
    if (err == ESP_OK) {
        handle->vrefMicrovolts = vrefMicrovolts;
    }
    return err;
}

esp_err_t ads126xAdcSetInputMux(ads126xAdcHandle_t *handle, uint8_t muxp, uint8_t muxn)
{
    uint8_t value = (uint8_t)(((muxp & 0x0Fu) << 4) | (muxn & 0x0Fu));
    return ads126xAdcWriteRegisters(handle, ADS126X_REG_INPMUX, &value, 1);
}

esp_err_t ads126xAdcSetInputMuxVerified(ads126xAdcHandle_t *handle,
                                       uint8_t muxp,
                                       uint8_t muxn)
{
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t expected = (uint8_t)(((muxp & 0x0Fu) << 4) | (muxn & 0x0Fu));
    esp_err_t err = ads126xAdcWriteRegisters(handle, ADS126X_REG_INPMUX, &expected, 1u);
    uint8_t readback = 0u;
    if (err == ESP_OK) {
        err = ads126xAdcReadRegisters(handle, ADS126X_REG_INPMUX, &readback, 1u);
    }
    return err == ESP_OK && readback != expected ? ESP_ERR_INVALID_RESPONSE : err;
}

bool ads126xAdcPgaGainSupported(uint8_t gain)
{
    uint8_t code = 0u;
    return ads126xAdcGainToCode(gain, &code);
}

bool ads126xAdcMode2PgaBypassed(uint8_t mode2)
{
    return (mode2 & ADS126X_MODE2_BYPASS) != 0u;
}

bool ads126xAdcMode2DecodePgaGain(uint8_t mode2, uint8_t *outGain)
{
    if (!outGain || (mode2 & ADS126X_MODE2_BYPASS) != 0u) {
        return false;
    }
    uint8_t code = (uint8_t)((mode2 & ADS126X_MODE2_GAIN_MASK) >>
                             ADS126X_MODE2_GAIN_SHIFT);
    if (code >= 6u) {
        return false;
    }
    *outGain = (uint8_t)(1u << code);
    return true;
}

esp_err_t ads126xAdcSetPgaGain(ads126xAdcHandle_t *handle, uint8_t gain)
{
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t gainCode = 0u;
    if (!ads126xAdcGainToCode(gain, &gainCode)) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t mode2 = 0u;
    esp_err_t err = ads126xAdcReadRegisters(handle, ADS126X_REG_MODE2, &mode2, 1u);
    if (err != ESP_OK) {
        return err;
    }
    uint8_t expected = (uint8_t)((mode2 &
        (uint8_t)~(ADS126X_MODE2_BYPASS | ADS126X_MODE2_GAIN_MASK)) |
        (uint8_t)(gainCode << ADS126X_MODE2_GAIN_SHIFT));
    err = ads126xAdcWriteRegisters(handle, ADS126X_REG_MODE2, &expected, 1u);
    uint8_t readback = 0u;
    if (err == ESP_OK) {
        err = ads126xAdcReadRegisters(handle, ADS126X_REG_MODE2, &readback, 1u);
    }
    if (err == ESP_OK && readback != expected) {
        err = ESP_ERR_INVALID_RESPONSE;
    }
    if (err == ESP_OK) {
        handle->pgaGain = gain;
        handle->pgaBypassed = false;
    }
    return err;
}

esp_err_t ads126xAdcSetPgaBypass(ads126xAdcHandle_t *handle, bool bypass)
{
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t mode2 = 0u;
    esp_err_t err = ads126xAdcReadRegisters(handle, ADS126X_REG_MODE2, &mode2, 1u);
    if (err != ESP_OK) {
        return err;
    }
    uint8_t expected = bypass ?
        (uint8_t)(mode2 | ADS126X_MODE2_BYPASS) :
        (uint8_t)(mode2 & (uint8_t)~ADS126X_MODE2_BYPASS);
    if (expected != mode2) {
        err = ads126xAdcWriteRegisters(handle, ADS126X_REG_MODE2, &expected, 1u);
    }
    uint8_t readback = 0u;
    if (err == ESP_OK) {
        err = ads126xAdcReadRegisters(handle, ADS126X_REG_MODE2, &readback, 1u);
    }
    if (err == ESP_OK && readback != expected) {
        err = ESP_ERR_INVALID_RESPONSE;
    }
    if (err == ESP_OK && bypass) {
        /* PGA bypass has unity transfer; keep raw-to-voltage scaling explicit. */
        handle->pgaGain = 1u;
    }
    if (err == ESP_OK) {
        handle->pgaBypassed = bypass;
    }
    return err;
}

esp_err_t ads126xAdcSetVbiasEnabled(ads126xAdcHandle_t *handle, bool enableVbias)
{
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }

    return ads126xAdcApplyPowerPolicy(handle, false, false, true, enableVbias, NULL, NULL);
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

    /* Read POWER..REFMUX under one command. The board ties CS low; five
     * separate short RREG commands exposed the last REFMUX read to a
     * repeatable one-bit DOUT error while ADC1 was converting at 38.4 kSPS.
     * A block read is explicitly supported by the ADS126x and keeps the
     * register snapshot atomic with fewer command boundaries. */
    uint8_t registers[ADS126X_REG_REFMUX - ADS126X_REG_POWER + 1u] = {0};
    esp_err_t err = ads126xAdcReadRegisters(handle,
                                             ADS126X_REG_POWER,
                                             registers,
                                             sizeof(registers));
    if (err != ESP_OK) {
        return err;
    }

    uint8_t power = registers[ADS126X_REG_POWER - ADS126X_REG_POWER];
    uint8_t iface = registers[ADS126X_REG_INTERFACE - ADS126X_REG_POWER];
    uint8_t mode2 = registers[ADS126X_REG_MODE2 - ADS126X_REG_POWER];
    uint8_t inpmux = registers[ADS126X_REG_INPMUX - ADS126X_REG_POWER];
    uint8_t refmux = registers[ADS126X_REG_REFMUX - ADS126X_REG_POWER];

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

static void ads126xAdcDecodeAdc1RegisterSnapshot(
    const uint8_t registers[16],
    ads126xAdc1RegisterSnapshot_t *outSnapshot)
{
    *outSnapshot = (ads126xAdc1RegisterSnapshot_t){
        .id = registers[0x00],
        .power = registers[0x01],
        .interface = registers[0x02],
        .mode0 = registers[0x03],
        .mode1 = registers[0x04],
        .mode2 = registers[0x05],
        .inpmux = registers[0x06],
        .offsetCal = {registers[0x07], registers[0x08], registers[0x09]},
        .fullScaleCal = {registers[0x0A], registers[0x0B], registers[0x0C]},
        .refmux = registers[0x0F],
    };
}

esp_err_t ads126xAdcReadAdc1RegisterSnapshot(
    ads126xAdcHandle_t *handle,
    ads126xAdc1RegisterSnapshot_t *outSnapshot)
{
    if (!handle || !outSnapshot) {
        return ESP_ERR_INVALID_ARG;
    }
    /* CS is tied low on the production board, so an isolated DOUT bit error
     * cannot be cleared by pulsing CS. Never use one unverified read as the
     * source for a later transaction restore: read twice, and use a third
     * read only to resolve a disagreement. This is outside the cell hot path. */
    uint8_t first[16] = {0};
    uint8_t second[16] = {0};
    uint8_t third[16] = {0};
    esp_err_t err = ads126xAdcReadRegisters(handle, ADS126X_REG_ID,
                                             first, sizeof(first));
    if (err != ESP_OK) {
        return err;
    }
    err = ads126xAdcReadRegisters(handle, ADS126X_REG_ID,
                                   second, sizeof(second));
    if (err != ESP_OK) {
        return err;
    }
    const uint8_t *verified = NULL;
    if (memcmp(first, second, sizeof(first)) == 0) {
        verified = second;
    } else {
        err = ads126xAdcReadRegisters(handle, ADS126X_REG_ID,
                                       third, sizeof(third));
        if (err != ESP_OK) {
            return err;
        }
        if (memcmp(first, third, sizeof(first)) == 0) {
            verified = first;
        } else if (memcmp(second, third, sizeof(second)) == 0) {
            verified = second;
        } else {
            return ESP_ERR_INVALID_RESPONSE;
        }
    }
    ads126xAdcDecodeAdc1RegisterSnapshot(verified, outSnapshot);
    return ESP_OK;
}

esp_err_t ads126xAdcRestoreAdc1RegisterSnapshot(
    ads126xAdcHandle_t *handle,
    const ads126xAdc1RegisterSnapshot_t *snapshot,
    uint32_t vrefMicrovolts,
    bool adc1WasRunning)
{
    if (!handle || !snapshot || vrefMicrovolts == 0u ||
        (snapshot->interface & ADS126X_INTERFACE_CRC_MASK) == 3u) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t err = ads126xAdcStopAdc1(handle);
    if (err == ESP_OK) {
        err = ads126xAdcWriteRegisters(handle, ADS126X_REG_POWER,
                                       &snapshot->power, 1u);
    }
    uint8_t adc1Group[5] = {
        snapshot->interface,
        snapshot->mode0,
        snapshot->mode1,
        snapshot->mode2,
        snapshot->inpmux,
    };
    if (err == ESP_OK) {
        err = ads126xAdcWriteRegisters(handle, ADS126X_REG_INTERFACE,
                                       adc1Group, sizeof(adc1Group));
    }
    uint8_t calibration[6] = {
        snapshot->offsetCal[0], snapshot->offsetCal[1], snapshot->offsetCal[2],
        snapshot->fullScaleCal[0], snapshot->fullScaleCal[1],
        snapshot->fullScaleCal[2],
    };
    if (err == ESP_OK) {
        err = ads126xAdcWriteRegisters(handle, ADS126X_REG_OFCAL0,
                                       calibration, sizeof(calibration));
    }
    if (err == ESP_OK) {
        err = ads126xAdcWriteRegisters(handle, ADS126X_REG_REFMUX,
                                       &snapshot->refmux, 1u);
    }
    if (err != ESP_OK) {
        return err;
    }

    handle->enableStatusByte =
        (snapshot->interface & ADS126X_INTERFACE_STATUS) != 0u;
    handle->crcMode = (ads126xCrcMode_t)(snapshot->interface &
                                         ADS126X_INTERFACE_CRC_MASK);
    handle->mode0 = snapshot->mode0;
    handle->mode1 = snapshot->mode1;
    handle->enableInternalRef =
        (snapshot->power & ADS126X_POWER_INTREF) != 0u;
    handle->vrefMicrovolts = vrefMicrovolts;
    err = ads126xAdcNoteMode2(handle, snapshot->mode2);
    if (err != ESP_OK) {
        return err;
    }

    ads126xAdc1RegisterSnapshot_t readback = {0};
    err = ads126xAdcReadAdc1RegisterSnapshot(handle, &readback);
    bool registerMatch = err == ESP_OK &&
        readback.id == snapshot->id &&
        readback.power == snapshot->power &&
        readback.interface == snapshot->interface &&
        readback.mode0 == snapshot->mode0 &&
        readback.mode1 == snapshot->mode1 &&
        readback.mode2 == snapshot->mode2 &&
        readback.inpmux == snapshot->inpmux &&
        readback.refmux == snapshot->refmux &&
        memcmp(readback.offsetCal, snapshot->offsetCal,
               sizeof(readback.offsetCal)) == 0 &&
        memcmp(readback.fullScaleCal, snapshot->fullScaleCal,
               sizeof(readback.fullScaleCal)) == 0;
    if (!registerMatch) {
        return err == ESP_OK ? ESP_ERR_INVALID_RESPONSE : err;
    }
    return adc1WasRunning ? ads126xAdcStartAdc1(handle) : ESP_OK;
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
    esp_err_t err = ads126xAdcSendCommand(handle, ADS126X_CMD_START1);
    if (err == ESP_OK && handle) {
        handle->adc1Running = true;
    }
    return err;
}

esp_err_t ads126xAdcStopAdc1(ads126xAdcHandle_t *handle)
{
    esp_err_t err = ads126xAdcSendCommand(handle, ADS126X_CMD_STOP1);
    if (err == ESP_OK && handle) {
        handle->adc1Running = false;
    }
    return err;
}

bool ads126xAdcIsAdc1Running(const ads126xAdcHandle_t *handle)
{
    return handle && handle->adc1Running;
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

esp_err_t ads126xAdcConfigureAdc1Mode(ads126xAdcHandle_t *handle,
                                      uint8_t mode0,
                                      uint8_t mode1)
{
    if (!handle || (mode1 & ADS126X_MODE1_FILTER_MASK) > ADS126X_MODE1_FILTER_FIR) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t expected[2] = {mode0, mode1};
    esp_err_t err = ads126xAdcWriteRegisters(handle, ADS126X_REG_MODE0,
                                              expected, sizeof(expected));
    uint8_t readback[2] = {0u, 0u};
    if (err == ESP_OK) {
        err = ads126xAdcReadRegisters(handle, ADS126X_REG_MODE0,
                                      readback, sizeof(readback));
    }
    if (err == ESP_OK && memcmp(expected, readback, sizeof(expected)) != 0) {
        err = ESP_ERR_INVALID_RESPONSE;
    }
    if (err == ESP_OK) {
        handle->mode0 = mode0;
        handle->mode1 = mode1;
    }
    return err;
}

bool ads126xAdcBuildMode2(bool pgaBypassed,
                          uint8_t gain,
                          uint8_t dataRateDr,
                          uint8_t *outMode2)
{
    if (!outMode2 || dataRateDr > ADS126X_MODE2_DR_MASK) {
        return false;
    }
    uint8_t gainCode = 0u;
    if (!ads126xAdcGainToCode(gain, &gainCode)) {
        return false;
    }
    *outMode2 = (uint8_t)((pgaBypassed ? ADS126X_MODE2_BYPASS : 0u) |
                          (gainCode << ADS126X_MODE2_GAIN_SHIFT) |
                          dataRateDr);
    return true;
}

static esp_err_t ads126xAdcNoteMode2(ads126xAdcHandle_t *handle, uint8_t mode2)
{
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t gainCode = (uint8_t)((mode2 & ADS126X_MODE2_GAIN_MASK) >>
                                 ADS126X_MODE2_GAIN_SHIFT);
    if (gainCode >= 6u) {
        return ESP_ERR_INVALID_ARG;
    }
    handle->pgaBypassed = ads126xAdcMode2PgaBypassed(mode2);
    handle->pgaGain = (uint8_t)(1u << gainCode);
    handle->dataRateDr = mode2 & ADS126X_MODE2_DR_MASK;
    return ESP_OK;
}

esp_err_t ads126xAdcSetMode2Fast(ads126xAdcHandle_t *handle, uint8_t mode2)
{
    if (!handle || ((mode2 & ADS126X_MODE2_GAIN_MASK) >>
                    ADS126X_MODE2_GAIN_SHIFT) >= 6u) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t err = ads126xAdcWriteRegisters(handle, ADS126X_REG_MODE2, &mode2, 1u);
    return err == ESP_OK ? ads126xAdcNoteMode2(handle, mode2) : err;
}

esp_err_t ads126xAdcSetMode2Verified(ads126xAdcHandle_t *handle, uint8_t mode2)
{
    esp_err_t err = ads126xAdcSetMode2Fast(handle, mode2);
    uint8_t readback = 0u;
    if (err == ESP_OK) {
        err = ads126xAdcReadRegisters(handle, ADS126X_REG_MODE2, &readback, 1u);
    }
    return err == ESP_OK && readback != mode2 ? ESP_ERR_INVALID_RESPONSE : err;
}

static void IRAM_ATTR ads126xAdcDrdyIsr(void *arg)
{
    ads126xAdcHandle_t *handle = (ads126xAdcHandle_t *)arg;
    if (!handle) {
        return;
    }
    handle->drdyGeneration++;
    TaskHandle_t task = (TaskHandle_t)handle->drdyWaitTask;
    if (task) {
        BaseType_t higherPriorityTaskWoken = pdFALSE;
        vTaskNotifyGiveFromISR(task, &higherPriorityTaskWoken);
        if (higherPriorityTaskWoken == pdTRUE) {
            portYIELD_FROM_ISR();
        }
    }
}

esp_err_t ads126xAdcEnableDrdyNotification(ads126xAdcHandle_t *handle)
{
    if (!handle || handle->drdyGpio == GPIO_NUM_NC) {
        return ESP_ERR_INVALID_ARG;
    }
    if (handle->drdyNotificationReady) {
        return ESP_OK;
    }
    gpio_config_t config = {
        .pin_bit_mask = 1ULL << (uint32_t)handle->drdyGpio,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };
    esp_err_t err = gpio_config(&config);
    if (err != ESP_OK) {
        return err;
    }
    err = sensorarrayBoardEnsureGpioIsrService();
    if (err != ESP_OK) {
        return err;
    }
    err = gpio_isr_handler_add(handle->drdyGpio, ads126xAdcDrdyIsr, handle);
    if (err == ESP_ERR_INVALID_STATE) {
        (void)gpio_isr_handler_remove(handle->drdyGpio);
        err = gpio_isr_handler_add(handle->drdyGpio, ads126xAdcDrdyIsr, handle);
    }
    if (err == ESP_OK) {
        handle->drdyNotificationReady = true;
        sensorarrayBoardNoteGpioIsrHandlerAdd(BOARD_SUPPORT_GPIO_ISR_HANDLER_ADS);
    }
    return err;
}

void ads126xAdcClearDrdyNotifications(ads126xAdcHandle_t *handle)
{
    if (!handle) {
        return;
    }
    TaskHandle_t current = xTaskGetCurrentTaskHandle();
    if (current) {
        (void)xTaskNotifyStateClear(current);
        while (ulTaskNotifyTake(pdTRUE, 0) != 0u) {
        }
    }
}

uint32_t ads126xAdcGetDrdyGeneration(const ads126xAdcHandle_t *handle)
{
    return handle ? handle->drdyGeneration : 0u;
}

esp_err_t ads126xAdcWaitDrdyNotificationUs(ads126xAdcHandle_t *handle, uint32_t timeoutUs)
{
    if (!handle || !handle->drdyNotificationReady || handle->drdyGpio == GPIO_NUM_NC) {
        return ESP_ERR_INVALID_STATE;
    }
    if (gpio_get_level(handle->drdyGpio) == 0) {
        return ESP_OK;
    }

    TaskHandle_t current = xTaskGetCurrentTaskHandle();
    handle->drdyWaitTask = current;
    (void)xTaskNotifyStateClear(current);
    while (ulTaskNotifyTake(pdTRUE, 0) != 0u) {
    }
    if (gpio_get_level(handle->drdyGpio) == 0) {
        handle->drdyWaitTask = NULL;
        return ESP_OK;
    }
    TickType_t ticks = pdMS_TO_TICKS((timeoutUs + 999u) / 1000u);
    if (ticks == 0u) {
        ticks = 1u;
    }
    uint32_t notified = ulTaskNotifyTake(pdTRUE, ticks);
    handle->drdyWaitTask = NULL;
    return (notified != 0u || gpio_get_level(handle->drdyGpio) == 0) ?
        ESP_OK : ESP_ERR_TIMEOUT;
}

esp_err_t ads126xAdcWaitDrdyGenerationUs(ads126xAdcHandle_t *handle,
                                         uint32_t startGeneration,
                                         uint32_t timeoutUs,
                                         uint32_t *outGeneration)
{
    if (!handle || !handle->drdyNotificationReady || handle->drdyGpio == GPIO_NUM_NC) {
        return ESP_ERR_INVALID_STATE;
    }

    uint32_t currentGeneration = handle->drdyGeneration;
    if (currentGeneration != startGeneration) {
        if (outGeneration) {
            *outGeneration = currentGeneration;
        }
        return ESP_OK;
    }

    TaskHandle_t currentTask = xTaskGetCurrentTaskHandle();
    handle->drdyWaitTask = currentTask;
    ads126xAdcClearDrdyNotifications(handle);
    currentGeneration = handle->drdyGeneration;
    if (currentGeneration != startGeneration) {
        handle->drdyWaitTask = NULL;
        if (outGeneration) {
            *outGeneration = currentGeneration;
        }
        return ESP_OK;
    }

    TickType_t ticks = pdMS_TO_TICKS((timeoutUs + 999u) / 1000u);
    if (ticks == 0u) {
        ticks = 1u;
    }
    uint32_t notified = ulTaskNotifyTake(pdTRUE, ticks);
    currentGeneration = handle->drdyGeneration;
    handle->drdyWaitTask = NULL;
    if (currentGeneration != startGeneration) {
        if (outGeneration) {
            *outGeneration = currentGeneration;
        }
        return ESP_OK;
    }
    return notified != 0u ? ESP_ERR_INVALID_RESPONSE : ESP_ERR_TIMEOUT;
}

bool ads126xAdcStatusByteHasAdc1NewData(const ads126xAdcHandle_t *handle, uint8_t statusByte)
{
    if (!handle) {
        return false;
    }
    if (!handle->enableStatusByte) {
        return true;
    }
    return (statusByte & ADS126X_STATUS_ADC1_NEW_DATA) != 0u;
}

bool ads126xAdcStatusByteHasReferenceAlarm(uint8_t statusByte)
{
    return (statusByte & ADS126X_STATUS_REFERENCE_ALARM) != 0u;
}

bool ads126xAdcStatusByteHasPgaAlarm(uint8_t statusByte)
{
    return (statusByte & ADS126X_STATUS_PGA_ALARM_MASK) != 0u;
}

esp_err_t ads126xAdcSetInputMuxFast(ads126xAdcHandle_t *handle, uint8_t muxp, uint8_t muxn)
{
    uint8_t command[3] = {
        (uint8_t)(ADS126X_CMD_WREG | ADS126X_REG_INPMUX),
        0u,
        (uint8_t)(((muxp & 0x0Fu) << 4) | (muxn & 0x0Fu)),
    };
    return ads126xAdcSpiTransferLocked(handle, command, sizeof(command), NULL, 0u);
}

esp_err_t ads126xAdcReadAdc1RawDma(ads126xAdcHandle_t *handle,
                                   uint32_t drdyTimeoutUs,
                                   int32_t *rawCode,
                                   uint8_t *statusByteOptional,
                                   uint32_t *outReadUs)
{
    if (!handle || !rawCode) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t err = ads126xAdcWaitDrdyNotificationUs(handle, drdyTimeoutUs);
    if (err != ESP_OK) {
        return err;
    }
    return ads126xAdcReadAdc1RawDmaReady(handle,
                                         rawCode,
                                         statusByteOptional,
                                         outReadUs);
}

esp_err_t ads126xAdcReadAdc1RawDmaReady(ads126xAdcHandle_t *handle,
                                        int32_t *rawCode,
                                        uint8_t *statusByteOptional,
                                        uint32_t *outReadUs)
{
    if (!handle || !rawCode) {
        return ESP_ERR_INVALID_ARG;
    }
    size_t frameLen = 4u + (handle->enableStatusByte ? 1u : 0u) +
                      (handle->crcMode != ADS126X_CRC_OFF ? 1u : 0u);
    uint8_t frame[6] = {0};
    uint8_t cmd = ADS126X_CMD_RDATA1;
    int64_t startUs = esp_timer_get_time();
    esp_err_t err = ads126xAdcSpiTransferDmaLocked(handle, &cmd, 1u, frame, frameLen);
    if (outReadUs) {
        int64_t elapsedUs = esp_timer_get_time() - startUs;
        *outReadUs = elapsedUs > 0 ? (uint32_t)elapsedUs : 0u;
    }
    return err == ESP_OK ?
        ads126xAdcParseAdc1Frame(handle, frame, frameLen, rawCode, statusByteOptional) :
        err;
}

uint32_t ads126xAdcDataRateCodeToSps(uint8_t drCode)
{
    static const uint32_t rates[] = {
        3u, 5u, 10u, 17u, 20u, 50u, 60u, 100u,
        400u, 1200u, 2400u, 4800u, 7200u, 14400u, 19200u, 38400u,
    };
    return rates[drCode & ADS126X_MODE2_DR_MASK];
}

uint32_t ads126xAdcExpectedConversionPeriodUs(uint8_t drCode)
{
    uint32_t sps = ads126xAdcDataRateCodeToSps(drCode);
    return sps ? (1000000u + sps - 1u) / sps : 0u;
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

    size_t frameLen = 4;
    if (handle->enableStatusByte) {
        frameLen += 1;
    }
    if (handle->crcMode != ADS126X_CRC_OFF) {
        frameLen += 1;
    }

    uint8_t frame[6] = {0};
    uint8_t cmd = ADS126X_CMD_RDATA1;
    err = ads126xAdcSpiTransferLocked(handle, &cmd, 1, frame, frameLen);
    if (err != ESP_OK) {
        return err;
    }

    return ads126xAdcParseAdc1Frame(handle, frame, frameLen, rawCode, statusByteOptional);
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

esp_err_t ads126xAdcReadCalibrationRegisters(ads126xAdcHandle_t *handle,
                                             uint8_t offsetCal[3],
                                             uint8_t fullScaleCal[3])
{
    if (!handle || !offsetCal || !fullScaleCal) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t err = ads126xAdcReadRegisters(handle, ADS126X_REG_OFCAL0, offsetCal, 3u);
    if (err == ESP_OK) {
        err = ads126xAdcReadRegisters(handle, ADS126X_REG_FSCAL0, fullScaleCal, 3u);
    }
    return err;
}

uint8_t ads126xAdcGetDevId(const ads126xAdcHandle_t *handle)
{
    return handle ?
        (uint8_t)((handle->idRegRaw & ADS126X_ID_DEV_ID_MASK) >> ADS126X_ID_DEV_ID_SHIFT) :
        0xFFu;
}

uint8_t ads126xAdcGetRevId(const ads126xAdcHandle_t *handle)
{
    return handle ? (uint8_t)(handle->idRegRaw & 0x1Fu) : 0xFFu;
}

bool ads126xAdcHasAdc2(const ads126xAdcHandle_t *handle)
{
    return ads126xAdcIsAdc2Supported(handle);
}

esp_err_t ads126xAdcProbeAdc2(ads126xAdcHandle_t *handle, bool *outHasAdc2)
{
    if (!handle || !outHasAdc2) {
        return ESP_ERR_INVALID_ARG;
    }
    if (handle->deviceType == ADS126X_DEVICE_ADS1262) {
        *outHasAdc2 = false;
        return ESP_OK;
    }
    if (handle->deviceType == ADS126X_DEVICE_ADS1263) {
        *outHasAdc2 = true;
        return ESP_OK;
    }

#if CONFIG_ADS126X_HAS_ADC2
    ads126xDeviceType_t previousType = handle->deviceType;
    handle->deviceType = ADS126X_DEVICE_ADS1263;
    int32_t raw = 0;
    esp_err_t err = ads126xAdcStartAdc2(handle);
    if (err == ESP_OK) {
        err = ads126xAdcReadAdc2Raw(handle, &raw, NULL);
    }
    esp_err_t stopErr = ads126xAdcStopAdc2(handle);
    if (err == ESP_OK && stopErr != ESP_OK) {
        err = stopErr;
    }
    *outHasAdc2 = err == ESP_OK;
    handle->deviceType = *outHasAdc2 ? ADS126X_DEVICE_ADS1263 : previousType;
    return ESP_OK;
#else
    *outHasAdc2 = false;
    return ESP_OK;
#endif
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

bool ads126xAdcIsAdc2Running(const ads126xAdcHandle_t *handle)
{
    return handle && ads126xAdcIsAdc2Supported(handle) && handle->adc2Running;
}

esp_err_t ads126xAdcSetAdc2Config(ads126xAdcHandle_t *handle,
                                  uint8_t dataRate,
                                  uint8_t reference,
                                  uint8_t gain,
                                  uint32_t vrefMicrovolts)
{
    if (!ads126xAdcIsAdc2Supported(handle)) {
        return ESP_ERR_NOT_SUPPORTED;
    }
    if (dataRate > 3u || reference > 4u || gain > 7u || vrefMicrovolts == 0u) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t value = (uint8_t)((dataRate << 6u) | (reference << 3u) | gain);
    esp_err_t err = ads126xAdcWriteRegisters(handle, ADS126X_REG_ADC2CFG, &value, 1u);
    uint8_t readback = 0u;
    if (err == ESP_OK) {
        err = ads126xAdcReadRegisters(handle, ADS126X_REG_ADC2CFG, &readback, 1u);
    }
    if (err != ESP_OK) {
        return err;
    }
    if (readback != value) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    handle->adc2DataRate = dataRate;
    handle->adc2Reference = reference;
    handle->adc2Gain = (uint8_t)(1u << gain);
    handle->adc2VrefMicrovolts = vrefMicrovolts;
    return ESP_OK;
}

esp_err_t ads126xAdcSetAdc2InputMux(ads126xAdcHandle_t *handle,
                                    uint8_t muxp,
                                    uint8_t muxn)
{
    if (!ads126xAdcIsAdc2Supported(handle)) {
        return ESP_ERR_NOT_SUPPORTED;
    }
    if (muxp > 0x0Fu || muxn > 0x0Fu) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t value = (uint8_t)((muxp << 4u) | muxn);
    return ads126xAdcWriteRegisters(handle, ADS126X_REG_ADC2MUX, &value, 1u);
}

static esp_err_t ads126xAdcParseAdc2Frame(ads126xAdcHandle_t *handle,
                                           const uint8_t *frame,
                                           size_t frameLen,
                                           int32_t *raw24,
                                           uint8_t *statusOptional)
{
    size_t idx = 0u;
    if (handle->enableStatusByte) {
        if (statusOptional) {
            *statusOptional = frame[idx];
        }
        idx++;
    }

    const uint8_t *dataBytes = &frame[idx];
    uint8_t padByte = dataBytes[3];
    if (padByte != 0x00u) {
        ESP_LOGW(TAG, "ADC2 pad byte unexpected: 0x%02X", padByte);
    }
    if (handle->crcMode != ADS126X_CRC_OFF) {
        uint8_t expected = frame[frameLen - 1u];
        uint8_t actual = handle->crcMode == ADS126X_CRC_CRC8 ?
            ads126xAdcCrc8(dataBytes, 3u) : ads126xAdcChecksum(dataBytes, 3u);
        if (expected != actual) {
            return ESP_ERR_INVALID_RESPONSE;
        }
    }

    uint32_t raw = ((uint32_t)dataBytes[0] << 16u) |
                   ((uint32_t)dataBytes[1] << 8u) |
                   (uint32_t)dataBytes[2];
    if ((raw & 0x00800000u) != 0u) {
        raw |= 0xFF000000u;
    }
    *raw24 = (int32_t)raw;
    return ESP_OK;
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

    return ads126xAdcParseAdc2Frame(handle, frame, frameLen, raw24, statusOptional);
}

esp_err_t ads126xAdcReadAdc2RawDma(ads126xAdcHandle_t *handle,
                                   uint32_t drdyTimeoutUs,
                                   int32_t *raw24,
                                   uint8_t *statusOptional,
                                   uint32_t *outReadUs)
{
    if (!handle || !raw24) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!ads126xAdcIsAdc2Supported(handle)) {
        return ESP_ERR_NOT_SUPPORTED;
    }
    esp_err_t err = ads126xAdcWaitDrdyNotificationUs(handle, drdyTimeoutUs);
    if (err != ESP_OK) {
        return err;
    }
    return ads126xAdcReadAdc2RawDmaReady(handle,
                                         raw24,
                                         statusOptional,
                                         outReadUs);
}

esp_err_t ads126xAdcReadAdc2RawDmaReady(ads126xAdcHandle_t *handle,
                                        int32_t *raw24,
                                        uint8_t *statusOptional,
                                        uint32_t *outReadUs)
{
    if (!handle || !raw24) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!ads126xAdcIsAdc2Supported(handle)) {
        return ESP_ERR_NOT_SUPPORTED;
    }
    size_t frameLen = 4u + (handle->enableStatusByte ? 1u : 0u) +
                      (handle->crcMode != ADS126X_CRC_OFF ? 1u : 0u);
    uint8_t frame[6] = {0};
    uint8_t command = ADS126X_CMD_RDATA2;
    int64_t startUs = esp_timer_get_time();
    esp_err_t err = ads126xAdcSpiTransferDmaLocked(handle,
                                                   &command,
                                                   1u,
                                                   frame,
                                                   frameLen);
    if (outReadUs) {
        int64_t elapsedUs = esp_timer_get_time() - startUs;
        *outReadUs = elapsedUs > 0 ? (uint32_t)elapsedUs : 0u;
    }
    return err == ESP_OK ?
        ads126xAdcParseAdc2Frame(handle, frame, frameLen, raw24, statusOptional) : err;
}

uint32_t ads126xAdcAdc2ExpectedConversionPeriodUs(uint8_t dataRate)
{
    static const uint32_t rates[] = {10u, 100u, 400u, 800u};
    uint32_t sps = rates[dataRate & 0x03u];
    return (1000000u + sps - 1u) / sps;
}

int32_t ads126xAdcAdc2RawToMicrovolts(const ads126xAdcHandle_t *handle,
                                      int32_t rawCode)
{
    if (!handle || handle->adc2Gain == 0u || handle->adc2VrefMicrovolts == 0u) {
        return 0;
    }
    int64_t numerator = (int64_t)rawCode * (int64_t)handle->adc2VrefMicrovolts;
    int64_t denominator = (int64_t)handle->adc2Gain * (1LL << 23);
    return (int32_t)(numerator / denominator);
}

esp_err_t ads126xAdcSystemOffsetCalAdc2(ads126xAdcHandle_t *handle)
{
    if (!ads126xAdcIsAdc2Supported(handle)) {
        return ESP_ERR_NOT_SUPPORTED;
    }
    return ads126xAdcSendCommand(handle, ADS126X_CMD_SYOCAL2);
}

esp_err_t ads126xAdcReadAdc2CalibrationRegisters(ads126xAdcHandle_t *handle,
                                                 uint8_t offsetCal[2],
                                                 uint8_t fullScaleCal[2])
{
    if (!ads126xAdcIsAdc2Supported(handle)) {
        return ESP_ERR_NOT_SUPPORTED;
    }
    if (!offsetCal || !fullScaleCal) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t err = ads126xAdcReadRegisters(handle,
                                             ADS126X_REG_ADC2OFC0,
                                             offsetCal,
                                             2u);
    if (err == ESP_OK) {
        err = ads126xAdcReadRegisters(handle,
                                      ADS126X_REG_ADC2FSC0,
                                      fullScaleCal,
                                      2u);
    }
    return err;
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

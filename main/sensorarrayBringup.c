#include "sensorarrayBringup.h"

#include <string.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "sensorarrayConfig.h"
#include "sensorarrayLog.h"

static void sensorarrayDelayMs(uint32_t delayMs)
{
    if (delayMs > 0u) {
        vTaskDelay(pdMS_TO_TICKS(delayMs));
    }
}

static gpio_num_t sensorarrayToGpio(int gpio)
{
    return gpio < 0 ? GPIO_NUM_NC : (gpio_num_t)gpio;
}

static esp_err_t sensorarrayInitSpi(spi_device_handle_t *outDevice)
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
        .max_transfer_sz = (int)CONFIG_SENSORARRAY_SPI_MAX_TRANSFER_BYTES,
    };

#if CONFIG_SENSORARRAY_SPI_USE_DMA
    int dmaChan = SPI_DMA_CH_AUTO;
#else
    int dmaChan = 0;
#endif

    esp_err_t err = spi_bus_initialize(CONFIG_BOARD_SPI_HOST, &busCfg, dmaChan);
    if (err == ESP_ERR_INVALID_STATE) {
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

static esp_err_t sensorarrayApplyFdcModePolicy(Fdc2214CapDevice_t *dev, uint8_t channels)
{
    channels = sensorarrayBringupNormalizeFdcChannels(channels);

    if (channels == 1u) {
        return Fdc2214CapSetSingleChannelMode(dev, FDC2214_CH0);
    }

    uint8_t rrSequence = 2u;
    if (channels == 2u) {
        rrSequence = 0u;
    } else if (channels == 3u) {
        rrSequence = 1u;
    }

    return Fdc2214CapSetAutoScanMode(dev, rrSequence, FDC2214_DEGLITCH_10MHZ);
}

static esp_err_t sensorarrayProbeFdcAddress(const BoardSupportI2cCtx_t *i2cCtx,
                                            uint8_t i2cAddr,
                                            uint16_t *outManufacturerId)
{
    if (!i2cCtx) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t reg = SENSORARRAY_FDC_REG_MANUFACTURER_ID;
    uint8_t rx[2] = {0};
    esp_err_t err = boardSupportI2cWriteRead((void *)i2cCtx, i2cAddr, &reg, sizeof(reg), rx, sizeof(rx));
    if (err == ESP_OK && outManufacturerId) {
        *outManufacturerId = (uint16_t)(((uint16_t)rx[0] << 8) | rx[1]);
    }
    return err;
}

uint8_t sensorarrayBringupNormalizeFdcChannels(uint8_t channels)
{
    if (channels == 0u) {
        channels = 1u;
    }
    if (channels > 4u) {
        channels = 4u;
    }
    return channels;
}

void sensorarrayBringupResetFdcState(sensorarrayFdcDeviceState_t *fdcState,
                                     const char *label,
                                     uint8_t i2cAddr)
{
    if (!fdcState) {
        return;
    }

    fdcState->label = label;
    fdcState->i2cCtx = NULL;
    fdcState->i2cAddr = i2cAddr;
    fdcState->handle = NULL;
    fdcState->ready = false;
    fdcState->haveIds = false;
    fdcState->manufacturerId = 0;
    fdcState->deviceId = 0;
}

bool sensorarrayBringupParseI2cAddress(uint32_t configuredAddress, uint8_t *outAddress)
{
    if (!outAddress || configuredAddress > 0x7Fu) {
        return false;
    }
    *outAddress = (uint8_t)configuredAddress;
    return true;
}

esp_err_t sensorarrayBringupAdsSetRefMux(sensorarrayState_t *state, uint8_t refmuxValue)
{
    if (!state) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = ads126xAdcSetRefMux(&state->ads, refmuxValue);
    if (err == ESP_OK) {
        state->adsRefMux = refmuxValue;
        state->adsRefMuxValid = true;
        sensorarrayLogDbgExtraSetRefMux(refmuxValue);
    }
    return err;
}

esp_err_t sensorarrayBringupInitAds(sensorarrayState_t *state)
{
    if (!state) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = sensorarrayInitSpi(&state->spiDevice);
    if (err != ESP_OK) {
        return err;
    }

    ads126xAdcConfig_t cfg = {0};
    cfg.spiDevice = state->spiDevice;
    cfg.drdyGpio = sensorarrayToGpio(CONFIG_BOARD_ADS126X_DRDY_GPIO);
    cfg.resetGpio = sensorarrayToGpio(CONFIG_BOARD_ADS126X_RESET_GPIO);
#if CONFIG_SENSORARRAY_ADS1262
    cfg.forcedType = ADS126X_DEVICE_ADS1262;
#elif CONFIG_SENSORARRAY_ADS1263
    cfg.forcedType = ADS126X_DEVICE_ADS1263;
#else
    cfg.forcedType = ADS126X_DEVICE_AUTO;
#endif
    cfg.crcMode = ADS126X_CRC_OFF;
    cfg.enableStatusByte = false;
    cfg.enableInternalRef = true;
    cfg.vrefMicrovolts = ADS126X_ADC_DEFAULT_VREF_UV;
    cfg.pgaGain = 1;
    cfg.dataRateDr = 0;

    err = ads126xAdcInit(&state->ads, &cfg);
    if (err != ESP_OK) {
        return err;
    }

    err = ads126xAdcStartAdc1(&state->ads);
    if (err != ESP_OK) {
        return err;
    }
    state->adsAdc1Running = true;

    int32_t raw = 0;
    return ads126xAdcReadAdc1Raw(&state->ads, &raw, NULL);
}

esp_err_t sensorarrayBringupPrepareAdsRefPath(sensorarrayState_t *state)
{
    if (!state) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = ads126xAdcConfigure(&state->ads, true, false, ADS126X_CRC_OFF, 1, 0);
    if (err != ESP_OK) {
        return err;
    }

    err = sensorarrayBringupAdsSetRefMux(state, 0x00);
    if (err != ESP_OK) {
        return err;
    }

    sensorarrayDelayMs(SENSORARRAY_REF_SETTLE_MS);

    err = ads126xAdcStartAdc1(&state->ads);
    if (err != ESP_OK) {
        return err;
    }
    state->adsAdc1Running = true;

    int32_t raw = 0;
    return ads126xAdcReadAdc1Raw(&state->ads, &raw, NULL);
}

void sensorarrayBringupProbeFdcBus(const sensorarrayFdcDeviceState_t *fdcState)
{
#if CONFIG_SENSORARRAY_FDC_STARTUP_PROBE
    if (!fdcState || !fdcState->i2cCtx) {
        return;
    }

    static const uint8_t probeAddresses[] = {
        SENSORARRAY_FDC_I2C_ADDR_LOW,
        SENSORARRAY_FDC_I2C_ADDR_HIGH,
    };

    for (size_t i = 0; i < (sizeof(probeAddresses) / sizeof(probeAddresses[0])); ++i) {
        sensorarrayFdcDeviceState_t probeState = *fdcState;
        probeState.i2cAddr = probeAddresses[i];

        uint16_t manufacturerId = 0;
        esp_err_t err = sensorarrayProbeFdcAddress(fdcState->i2cCtx, probeState.i2cAddr, &manufacturerId);
        sensorarrayLogStartupFdc("fdc_probe",
                                 &probeState,
                                 err,
                                 (err == ESP_OK) ? "probe_ack" : "probe_no_ack",
                                 (int32_t)probeState.i2cAddr,
                                 (err == ESP_OK),
                                 manufacturerId,
                                 0,
                                 "probe_mfg_reg_0x7E");
    }
#else
    (void)fdcState;
#endif
}

void sensorarrayBringupInitFdcDiag(sensorarrayFdcInitDiag_t *diag)
{
    if (!diag) {
        return;
    }
    diag->status = "unknown";
    diag->haveIds = false;
    diag->manufacturerId = 0;
    diag->deviceId = 0;
    diag->detail = 0;
}

esp_err_t sensorarrayBringupInitFdcDevice(const BoardSupportI2cCtx_t *i2cCtx,
                                          uint8_t i2cAddr,
                                          uint8_t channels,
                                          Fdc2214CapDevice_t **outDev,
                                          sensorarrayFdcInitDiag_t *outDiag)
{
    sensorarrayBringupInitFdcDiag(outDiag);

#if !CONFIG_FDC2214CAP_ENABLE
    (void)i2cCtx;
    (void)i2cAddr;
    (void)channels;
    (void)outDev;
    if (outDiag) {
        outDiag->status = "component_disabled";
    }
    return ESP_ERR_NOT_SUPPORTED;
#else
    if (!i2cCtx || !outDev) {
        if (outDiag) {
            outDiag->status = "invalid_args";
        }
        return ESP_ERR_INVALID_ARG;
    }

    *outDev = NULL;

    Fdc2214CapBusConfig_t busCfg = {
        .I2cAddress7 = i2cAddr,
        .UserCtx = (void *)i2cCtx,
        .WriteRead = boardSupportI2cWriteRead,
        .Write = boardSupportI2cWrite,
        .IntGpio = -1,
    };

    Fdc2214CapDevice_t *dev = NULL;
    esp_err_t err = Fdc2214CapCreate(&busCfg, &dev);
    if (err != ESP_OK) {
        if (outDiag) {
            outDiag->status = "create_failure";
        }
        return err;
    }

    err = Fdc2214CapReset(dev);
    if (err != ESP_OK) {
        if (outDiag) {
            outDiag->status = "reset_failure";
        }
        Fdc2214CapDestroy(dev);
        return err;
    }

    uint16_t manufacturer = 0;
    uint16_t deviceId = 0;
    err = Fdc2214CapReadId(dev, &manufacturer, &deviceId);
    if (err != ESP_OK) {
        if (outDiag) {
            outDiag->status = "read_id_failure";
        }
        Fdc2214CapDestroy(dev);
        return err;
    }

    if (outDiag) {
        outDiag->haveIds = true;
        outDiag->manufacturerId = manufacturer;
        outDiag->deviceId = deviceId;
    }

    if (manufacturer != SENSORARRAY_FDC_EXPECTED_MANUFACTURER_ID ||
        deviceId != SENSORARRAY_FDC_EXPECTED_DEVICE_ID) {
        if (outDiag) {
            outDiag->status = "id_mismatch";
        }
        Fdc2214CapDestroy(dev);
        return ESP_ERR_INVALID_RESPONSE;
    }

    channels = sensorarrayBringupNormalizeFdcChannels(channels);

    Fdc2214CapChannelConfig_t chCfg = {
        .Rcount = 0xFFFF,
        .SettleCount = 0x0400,
        .Offset = 0x0000,
        .ClockDividers = 0x0001,
        .DriveCurrent = 0xA000,
    };

    for (uint8_t ch = 0; ch < channels; ++ch) {
        err = Fdc2214CapConfigureChannel(dev, (Fdc2214CapChannel_t)ch, &chCfg);
        if (err != ESP_OK) {
            if (outDiag) {
                outDiag->status = "channel_config_failure";
                outDiag->detail = (int32_t)ch;
            }
            Fdc2214CapDestroy(dev);
            return err;
        }
    }

    err = sensorarrayApplyFdcModePolicy(dev, channels);
    if (err != ESP_OK) {
        if (outDiag) {
            outDiag->status = "mode_config_failure";
        }
        Fdc2214CapDestroy(dev);
        return err;
    }

    *outDev = dev;
    if (outDiag) {
        outDiag->status = "ok";
        outDiag->detail = (int32_t)channels;
    }
    return ESP_OK;
#endif
}

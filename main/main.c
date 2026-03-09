#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "sdkconfig.h"

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "ads126xAdc.h"
#include "boardSupport.h"
#include "fdc2214Cap.h"
#include "tmuxSwitch.h"

#define SENSORARRAY_RESIST_REF_OHMS 10000u
#define SENSORARRAY_RESIST_EXCITATION_UV 2500000u

#define SENSORARRAY_REF_SETTLE_MS 120u
#define SENSORARRAY_SETTLE_AFTER_COLUMN_MS 2u
#define SENSORARRAY_SETTLE_AFTER_PATH_MS 2u
#define SENSORARRAY_SETTLE_AFTER_SW_MS 2u
#define SENSORARRAY_LOOP_DELAY_MS 250u

#define SENSORARRAY_FDC_EXPECTED_MANUFACTURER_ID 0x5449u
#define SENSORARRAY_FDC_EXPECTED_DEVICE_ID 0x3055u
#define SENSORARRAY_FDC_REQUIRED_CHANNELS 4u

#define SENSORARRAY_ADS_MUX_AINCOM 0x0Au

#define SENSORARRAY_S1 1u
#define SENSORARRAY_S2 2u

#define SENSORARRAY_D1 1u
#define SENSORARRAY_D4 4u
#define SENSORARRAY_D8 8u

#define SENSORARRAY_NA "na"

typedef enum {
    SENSORARRAY_PATH_RESISTIVE = 0,
    SENSORARRAY_PATH_CAPACITIVE = 1,
} sensorarrayPath_t;

typedef struct {
    spi_device_handle_t spiDevice;
    ads126xAdcHandle_t ads;
    bool adsReady;
    bool adsRefReady;

    Fdc2214CapDevice_t *fdcPrimary;   // D1..D4 (C1..C4)
    Fdc2214CapDevice_t *fdcSecondary; // D5..D8 (C5..C8)
    bool fdcPrimaryReady;
    bool fdcSecondaryReady;
    uint8_t fdcConfiguredChannels;

    bool boardReady;
    bool tmuxReady;
} sensorarrayState_t;

static sensorarrayState_t s_state = {0};

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

static uint8_t sensorarrayNormalizeFdcChannels(uint8_t channels)
{
    if (channels == 0u) {
        channels = 1u;
    }
    if (channels > 4u) {
        channels = 4u;
    }
    return channels;
}

static const char *sensorarrayRefState(void)
{
    if (!s_state.adsReady) {
        return "UNAVAILABLE";
    }
    return s_state.adsRefReady ? "READY" : "NOT_READY";
}

static const char *sensorarrayFmtI32(char *buf, size_t bufSize, bool valid, int32_t value)
{
    if (!valid) {
        return SENSORARRAY_NA;
    }
    snprintf(buf, bufSize, "%ld", (long)value);
    return buf;
}

static const char *sensorarrayFmtU32(char *buf, size_t bufSize, bool valid, uint32_t value)
{
    if (!valid) {
        return SENSORARRAY_NA;
    }
    snprintf(buf, bufSize, "%lu", (unsigned long)value);
    return buf;
}

static const char *sensorarrayFmtBool(char *buf, size_t bufSize, bool valid, bool value)
{
    if (!valid) {
        return SENSORARRAY_NA;
    }
    snprintf(buf, bufSize, "%d", value ? 1 : 0);
    return buf;
}

static void sensorarrayLogDbg(const char *point,
                              const char *kind,
                              const char *column,
                              const char *dline,
                              const char *sw,
                              const char *mode,
                              const char *value,
                              const char *valueUv,
                              const char *valueMohm,
                              const char *raw,
                              const char *wd,
                              const char *amp,
                              esp_err_t err,
                              const char *status)
{
    printf("DBG,point=%s,kind=%s,column=%s,dline=%s,sw=%s,ref=%s,mode=%s,value=%s,valueUv=%s,"
           "valueMohm=%s,raw=%s,wd=%s,amp=%s,err=%ld,status=%s\n",
           point ? point : SENSORARRAY_NA,
           kind ? kind : SENSORARRAY_NA,
           column ? column : SENSORARRAY_NA,
           dline ? dline : SENSORARRAY_NA,
           sw ? sw : SENSORARRAY_NA,
           sensorarrayRefState(),
           mode ? mode : SENSORARRAY_NA,
           value ? value : SENSORARRAY_NA,
           valueUv ? valueUv : SENSORARRAY_NA,
           valueMohm ? valueMohm : SENSORARRAY_NA,
           raw ? raw : SENSORARRAY_NA,
           wd ? wd : SENSORARRAY_NA,
           amp ? amp : SENSORARRAY_NA,
           (long)err,
           status ? status : SENSORARRAY_NA);
}

static void sensorarrayLogStartup(const char *mode, esp_err_t err, const char *status, int32_t detailValue)
{
    char valueBuf[24];
    sensorarrayLogDbg("INIT",
                      "startup",
                      SENSORARRAY_NA,
                      SENSORARRAY_NA,
                      SENSORARRAY_NA,
                      mode,
                      sensorarrayFmtI32(valueBuf, sizeof(valueBuf), true, detailValue),
                      SENSORARRAY_NA,
                      SENSORARRAY_NA,
                      SENSORARRAY_NA,
                      SENSORARRAY_NA,
                      SENSORARRAY_NA,
                      err,
                      status);
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

static esp_err_t sensorarrayInitAds(ads126xAdcHandle_t *handle, spi_device_handle_t *spiDevice)
{
    if (!handle || !spiDevice) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = sensorarrayInitSpi(spiDevice);
    if (err != ESP_OK) {
        return err;
    }

    ads126xAdcConfig_t cfg = {0};
    cfg.spiDevice = *spiDevice;
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

    err = ads126xAdcInit(handle, &cfg);
    if (err != ESP_OK) {
        return err;
    }

    // Basic conversion smoke-test before the runtime loop.
    err = ads126xAdcStartAdc1(handle);
    if (err != ESP_OK) {
        return err;
    }
    int32_t raw = 0;
    return ads126xAdcReadAdc1Raw(handle, &raw, NULL);
}

static esp_err_t sensorarrayPrepareAdsRefPath(ads126xAdcHandle_t *handle)
{
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }

    /*
     * SW=HIGH drives the path from ADS REF output in current hardware.
     * Keep this explicit: resistor-mode reads are only trusted after REF is
     * configured and given extra settle time.
     */
    esp_err_t err = ads126xAdcConfigure(handle, true, false, ADS126X_CRC_OFF, 1, 0);
    if (err != ESP_OK) {
        return err;
    }

    err = ads126xAdcSetRefMux(handle, 0x00);
    if (err != ESP_OK) {
        return err;
    }

    sensorarrayDelayMs(SENSORARRAY_REF_SETTLE_MS);

    err = ads126xAdcStartAdc1(handle);
    if (err != ESP_OK) {
        return err;
    }

    int32_t raw = 0;
    return ads126xAdcReadAdc1Raw(handle, &raw, NULL);
}

static esp_err_t sensorarrayApplyFdcModePolicy(Fdc2214CapDevice_t *dev, uint8_t channels)
{
    channels = sensorarrayNormalizeFdcChannels(channels);

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

static esp_err_t sensorarrayInitFdcDevice(const BoardSupportI2cCtx_t *i2cCtx,
                                          uint8_t channels,
                                          Fdc2214CapDevice_t **outDev)
{
#if !CONFIG_FDC2214CAP_ENABLE
    (void)i2cCtx;
    (void)channels;
    (void)outDev;
    return ESP_ERR_NOT_SUPPORTED;
#else
    if (!i2cCtx || !outDev) {
        return ESP_ERR_INVALID_ARG;
    }

    Fdc2214CapBusConfig_t busCfg = {
        .I2cAddress7 = CONFIG_FDC2214CAP_I2C_ADDR,
        .UserCtx = (void *)i2cCtx,
        .WriteRead = boardSupportI2cWriteRead,
        .Write = boardSupportI2cWrite,
        .IntGpio = -1,
    };

    Fdc2214CapDevice_t *dev = NULL;
    esp_err_t err = Fdc2214CapCreate(&busCfg, &dev);
    if (err != ESP_OK) {
        return err;
    }

    err = Fdc2214CapReset(dev);
    if (err != ESP_OK) {
        Fdc2214CapDestroy(dev);
        return err;
    }

    uint16_t manufacturer = 0;
    uint16_t deviceId = 0;
    err = Fdc2214CapReadId(dev, &manufacturer, &deviceId);
    if (err != ESP_OK) {
        Fdc2214CapDestroy(dev);
        return err;
    }
    if (manufacturer != SENSORARRAY_FDC_EXPECTED_MANUFACTURER_ID ||
        deviceId != SENSORARRAY_FDC_EXPECTED_DEVICE_ID) {
        Fdc2214CapDestroy(dev);
        return ESP_ERR_INVALID_RESPONSE;
    }

    channels = sensorarrayNormalizeFdcChannels(channels);

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
            Fdc2214CapDestroy(dev);
            return err;
        }
    }

    err = sensorarrayApplyFdcModePolicy(dev, channels);
    if (err != ESP_OK) {
        Fdc2214CapDestroy(dev);
        return err;
    }

    *outDev = dev;
    return ESP_OK;
#endif
}

static bool sensorarrayAdsMuxForDLine(uint8_t dLine, uint8_t *muxp, uint8_t *muxn)
{
    if (!muxp || !muxn || dLine < 1u || dLine > 8u) {
        return false;
    }

    *muxp = (uint8_t)(8u - dLine); // D1->AIN7 ... D8->AIN0
    *muxn = SENSORARRAY_ADS_MUX_AINCOM;
    return true;
}

static bool sensorarrayCapPathOnSelEnabled(uint8_t dLine)
{
    /*
     * From current schematic routing, even D lines (D2/D4/D6/D8) route to C*
     * when SEL is enabled; odd D lines route to C* when SEL is disabled.
     */
    return (dLine % 2u) == 0u;
}

static esp_err_t sensorarrayApplyRoute(uint8_t sColumn,
                                       uint8_t dLine,
                                       sensorarrayPath_t path,
                                       tmux1108Source_t swSource)
{
    if (!s_state.tmuxReady) {
        return ESP_ERR_INVALID_STATE;
    }
    if (sColumn < 1u || sColumn > 8u || dLine < 1u || dLine > 8u) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = tmuxSwitchSelectRow((uint8_t)(sColumn - 1u)); // TMUX1108 selects S1..S8
    if (err != ESP_OK) {
        return err;
    }
    sensorarrayDelayMs(SENSORARRAY_SETTLE_AFTER_COLUMN_MS);

    bool capPathWhenSelEnabled = sensorarrayCapPathOnSelEnabled(dLine);
    bool targetSelEnabled = (path == SENSORARRAY_PATH_CAPACITIVE) ? capPathWhenSelEnabled
                                                                   : !capPathWhenSelEnabled;

    bool selA = false;
    bool selB = false;
    if (dLine <= 4u) {
        selA = targetSelEnabled;
    } else {
        selB = targetSelEnabled;
    }

    err = tmuxSwitchSetSelAEnabled(selA);
    if (err != ESP_OK) {
        return err;
    }
    err = tmuxSwitchSetSelBEnabled(selB);
    if (err != ESP_OK) {
        return err;
    }
    sensorarrayDelayMs(SENSORARRAY_SETTLE_AFTER_PATH_MS);

    err = tmuxSwitchSet1108Source(swSource); // SW controls REF/GND source state.
    if (err != ESP_OK) {
        return err;
    }
    sensorarrayDelayMs(SENSORARRAY_SETTLE_AFTER_SW_MS);

    return ESP_OK;
}

static esp_err_t sensorarrayReadAdsUv(uint8_t dLine, bool discardFirst, int32_t *outRaw, int32_t *outUv)
{
    if (!outRaw || !outUv || !s_state.adsReady) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t muxp = 0;
    uint8_t muxn = 0;
    if (!sensorarrayAdsMuxForDLine(dLine, &muxp, &muxn)) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = ads126xAdcSetInputMux(&s_state.ads, muxp, muxn);
    if (err != ESP_OK) {
        return err;
    }

    err = ads126xAdcStartAdc1(&s_state.ads);
    if (err != ESP_OK) {
        return err;
    }

    if (discardFirst) {
        int32_t throwaway = 0;
        err = ads126xAdcReadAdc1Raw(&s_state.ads, &throwaway, NULL);
        if (err != ESP_OK) {
            return err;
        }
    }

    err = ads126xAdcReadAdc1Raw(&s_state.ads, outRaw, NULL);
    if (err != ESP_OK) {
        return err;
    }

    *outUv = ads126xAdcRawToMicrovolts(&s_state.ads, *outRaw);
    return ESP_OK;
}

static bool sensorarrayUvToResistanceMohm(int32_t uv, int32_t *outMohm)
{
    if (!outMohm) {
        return false;
    }
    if (uv <= 0 || (uint32_t)uv >= SENSORARRAY_RESIST_EXCITATION_UV) {
        return false;
    }

    int64_t numerator = (int64_t)SENSORARRAY_RESIST_REF_OHMS * 1000 * (int64_t)uv;
    int64_t denominator = (int64_t)SENSORARRAY_RESIST_EXCITATION_UV - (int64_t)uv;
    if (denominator == 0) {
        return false;
    }

    *outMohm = (int32_t)(numerator / denominator);
    return true;
}

static Fdc2214CapDevice_t *sensorarrayFdcDeviceForDLine(uint8_t dLine)
{
    if (dLine >= 1u && dLine <= 4u) {
        return s_state.fdcPrimary;
    }
    if (dLine >= 5u && dLine <= 8u) {
        return s_state.fdcSecondary;
    }
    return NULL;
}

static bool sensorarrayFdcReadyForDLine(uint8_t dLine)
{
    if (dLine >= 1u && dLine <= 4u) {
        return s_state.fdcPrimaryReady;
    }
    if (dLine >= 5u && dLine <= 8u) {
        return s_state.fdcSecondaryReady;
    }
    return false;
}

static bool sensorarrayFdcChannelForDLine(uint8_t dLine, Fdc2214CapChannel_t *outChannel)
{
    if (!outChannel || dLine < 1u || dLine > 8u) {
        return false;
    }

    uint8_t indexInBank = (uint8_t)((dLine - 1u) % 4u);
    *outChannel = (Fdc2214CapChannel_t)indexInBank;
    return true;
}

static esp_err_t sensorarrayReadFdcSample(Fdc2214CapDevice_t *dev,
                                          Fdc2214CapChannel_t ch,
                                          bool discardFirst,
                                          Fdc2214CapSample_t *outSample)
{
    if (!dev || !outSample) {
        return ESP_ERR_INVALID_ARG;
    }

    if (discardFirst) {
        Fdc2214CapSample_t throwaway = {0};
        esp_err_t err = Fdc2214CapReadSample(dev, ch, &throwaway);
        if (err != ESP_OK) {
            return err;
        }
    }

    return Fdc2214CapReadSample(dev, ch, outSample);
}

static void sensorarrayDebugReadResistorS1D1(void)
{
    char valueBuf[24];
    char uvBuf[24];
    char mohmBuf[24];
    char rawBuf[24];

    esp_err_t err = ESP_OK;
    int32_t uv = 0;
    int32_t raw = 0;
    int32_t mohm = 0;
    bool haveUv = false;
    bool haveRaw = false;
    bool haveMohm = false;
    const char *status = "ok";

    if (!s_state.adsReady) {
        status = "skip_ads_unavailable";
    } else if (!s_state.adsRefReady) {
        status = "skip_ref_not_ready";
    } else {
        err = sensorarrayApplyRoute(SENSORARRAY_S1, SENSORARRAY_D1, SENSORARRAY_PATH_RESISTIVE, TMUX1108_SOURCE_REF);
        if (err != ESP_OK) {
            status = "route_error";
        } else {
            err = sensorarrayReadAdsUv(SENSORARRAY_D1, true, &raw, &uv);
            if (err != ESP_OK) {
                status = "ads_read_error";
            } else {
                haveRaw = true;
                haveUv = true;
                if (sensorarrayUvToResistanceMohm(uv, &mohm)) {
                    haveMohm = true;
                } else {
                    status = "res_calc_invalid";
                }
            }
        }
    }

    sensorarrayLogDbg("S1D1",
                      "res",
                      "S1",
                      "D1",
                      "HIGH",
                      "ads",
                      sensorarrayFmtI32(valueBuf, sizeof(valueBuf), haveMohm, mohm),
                      sensorarrayFmtI32(uvBuf, sizeof(uvBuf), haveUv, uv),
                      sensorarrayFmtI32(mohmBuf, sizeof(mohmBuf), haveMohm, mohm),
                      sensorarrayFmtI32(rawBuf, sizeof(rawBuf), haveRaw, raw),
                      SENSORARRAY_NA,
                      SENSORARRAY_NA,
                      err,
                      status);
}

static void sensorarrayDebugReadPiezoCap(const char *pointLabel, uint8_t sColumn, uint8_t dLine)
{
    char valueBuf[24];
    char rawBuf[24];
    char wdBuf[8];
    char ampBuf[8];

    esp_err_t err = ESP_OK;
    Fdc2214CapSample_t sample = {0};
    bool haveSample = false;
    bool haveFlags = false;
    const char *status = "ok";

    if (!sensorarrayFdcReadyForDLine(dLine)) {
        status = "skip_fdc_unavailable";
    } else {
        err = sensorarrayApplyRoute(sColumn, dLine, SENSORARRAY_PATH_CAPACITIVE, TMUX1108_SOURCE_GND);
        if (err != ESP_OK) {
            status = "route_error";
        } else {
            Fdc2214CapChannel_t channel = FDC2214_CH0;
            if (!sensorarrayFdcChannelForDLine(dLine, &channel)) {
                err = ESP_ERR_INVALID_ARG;
                status = "mapping_error";
            } else {
                Fdc2214CapDevice_t *dev = sensorarrayFdcDeviceForDLine(dLine);
                err = sensorarrayReadFdcSample(dev, channel, true, &sample);
                if (err != ESP_OK) {
                    status = "fdc_read_error";
                } else {
                    haveSample = true;
                    haveFlags = true;
                    if (sample.ErrWatchdog || sample.ErrAmplitude) {
                        status = "warn_sensor_flags";
                    }
                }
            }
        }
    }

    sensorarrayLogDbg(pointLabel,
                      "cap",
                      (sColumn == SENSORARRAY_S1) ? "S1" : "S2",
                      (dLine == SENSORARRAY_D4) ? "D4" : "D8",
                      "LOW",
                      "fdc",
                      sensorarrayFmtU32(valueBuf, sizeof(valueBuf), haveSample, sample.Raw28),
                      SENSORARRAY_NA,
                      SENSORARRAY_NA,
                      sensorarrayFmtU32(rawBuf, sizeof(rawBuf), haveSample, sample.Raw28),
                      sensorarrayFmtBool(wdBuf, sizeof(wdBuf), haveFlags, sample.ErrWatchdog),
                      sensorarrayFmtBool(ampBuf, sizeof(ampBuf), haveFlags, sample.ErrAmplitude),
                      err,
                      status);
}

static void sensorarrayDebugReadPiezoVolt(const char *pointLabel, uint8_t sColumn, uint8_t dLine)
{
    char valueBuf[24];
    char uvBuf[24];
    char rawBuf[24];

    esp_err_t err = ESP_OK;
    int32_t uv = 0;
    int32_t raw = 0;
    bool haveUv = false;
    bool haveRaw = false;
    const char *status = "ok";

    if (!s_state.adsReady) {
        status = "skip_ads_unavailable";
    } else {
        err = sensorarrayApplyRoute(sColumn, dLine, SENSORARRAY_PATH_RESISTIVE, TMUX1108_SOURCE_GND);
        if (err != ESP_OK) {
            status = "route_error";
        } else {
            err = sensorarrayReadAdsUv(dLine, true, &raw, &uv);
            if (err != ESP_OK) {
                status = "ads_read_error";
            } else {
                haveUv = true;
                haveRaw = true;
            }
        }
    }

    sensorarrayLogDbg(pointLabel,
                      "volt",
                      (sColumn == SENSORARRAY_S1) ? "S1" : "S2",
                      (dLine == SENSORARRAY_D4) ? "D4" : "D8",
                      "LOW",
                      "ads",
                      sensorarrayFmtI32(valueBuf, sizeof(valueBuf), haveUv, uv),
                      sensorarrayFmtI32(uvBuf, sizeof(uvBuf), haveUv, uv),
                      SENSORARRAY_NA,
                      sensorarrayFmtI32(rawBuf, sizeof(rawBuf), haveRaw, raw),
                      SENSORARRAY_NA,
                      SENSORARRAY_NA,
                      err,
                      status);
}

static void sensorarrayRunBringupLoop(void)
{
    while (true) {
        // 1) S1D1 resistor via ADS (SW HIGH, REF-ready required)
        sensorarrayDebugReadResistorS1D1();

        // 2) S2D4 piezo capacitance via FDC (SW LOW)
        sensorarrayDebugReadPiezoCap("S2D4", SENSORARRAY_S2, SENSORARRAY_D4);

        // 3) S2D4 piezo voltage via ADS (SW LOW)
        sensorarrayDebugReadPiezoVolt("S2D4", SENSORARRAY_S2, SENSORARRAY_D4);

        // 4) S2D8 piezo capacitance via FDC (SW LOW)
        sensorarrayDebugReadPiezoCap("S2D8", SENSORARRAY_S2, SENSORARRAY_D8);

        // 5) S2D8 piezo voltage via ADS (SW LOW)
        sensorarrayDebugReadPiezoVolt("S2D8", SENSORARRAY_S2, SENSORARRAY_D8);

        sensorarrayDelayMs(SENSORARRAY_LOOP_DELAY_MS);
    }
}

void app_main(void)
{
    uint8_t requestedChannels = sensorarrayNormalizeFdcChannels((uint8_t)CONFIG_FDC2214CAP_CHANNELS);
    if (requestedChannels < SENSORARRAY_FDC_REQUIRED_CHANNELS) {
        requestedChannels = SENSORARRAY_FDC_REQUIRED_CHANNELS;
    }
    s_state.fdcConfiguredChannels = requestedChannels;

    sensorarrayLogStartup("app", ESP_OK, "start", 0);
    sensorarrayLogStartup("fdc_channels", ESP_OK, "policy_applied", (int32_t)requestedChannels);

    esp_err_t err = boardSupportInit();
    s_state.boardReady = (err == ESP_OK);
    sensorarrayLogStartup("board", err, s_state.boardReady ? "ok" : "init_failed", (int32_t)s_state.boardReady);

    err = tmuxSwitchInit();
    s_state.tmuxReady = (err == ESP_OK);
    sensorarrayLogStartup("tmux", err, s_state.tmuxReady ? "ok" : "init_failed", (int32_t)s_state.tmuxReady);

    if (s_state.tmuxReady) {
        esp_err_t tmuxErr = tmuxSwitchSelectRow(0);
        if (tmuxErr == ESP_OK) {
            tmuxErr = tmuxSwitchSetSelAEnabled(false);
        }
        if (tmuxErr == ESP_OK) {
            tmuxErr = tmuxSwitchSetSelBEnabled(false);
        }
        if (tmuxErr == ESP_OK) {
            tmuxErr = tmuxSwitchSet1108Source(TMUX1108_SOURCE_GND);
        }
        sensorarrayLogStartup("tmux_defaults",
                              tmuxErr,
                              (tmuxErr == ESP_OK) ? "ok" : "set_failed",
                              (int32_t)(tmuxErr == ESP_OK));
    }

    err = sensorarrayInitAds(&s_state.ads, &s_state.spiDevice);
    s_state.adsReady = (err == ESP_OK);
    sensorarrayLogStartup("ads", err, s_state.adsReady ? "ok" : "init_failed", (int32_t)s_state.adsReady);

    if (s_state.adsReady) {
        err = sensorarrayPrepareAdsRefPath(&s_state.ads);
        s_state.adsRefReady = (err == ESP_OK);
        sensorarrayLogStartup("ads_ref",
                              err,
                              s_state.adsRefReady ? "ready" : "not_ready",
                              (int32_t)s_state.adsRefReady);
    } else {
        s_state.adsRefReady = false;
        sensorarrayLogStartup("ads_ref", ESP_ERR_INVALID_STATE, "skip_ads_unavailable", 0);
    }

    if (s_state.boardReady) {
        err = sensorarrayInitFdcDevice(boardSupportGetI2cCtx(), requestedChannels, &s_state.fdcPrimary);
        s_state.fdcPrimaryReady = (err == ESP_OK);
        sensorarrayLogStartup("fdc_primary",
                              err,
                              s_state.fdcPrimaryReady ? "ok" : "init_failed",
                              (int32_t)s_state.fdcPrimaryReady);

        const BoardSupportI2cCtx_t *i2c1Ctx = boardSupportGetI2c1Ctx();
        if (i2c1Ctx) {
            err = sensorarrayInitFdcDevice(i2c1Ctx, requestedChannels, &s_state.fdcSecondary);
            s_state.fdcSecondaryReady = (err == ESP_OK);
            sensorarrayLogStartup("fdc_secondary",
                                  err,
                                  s_state.fdcSecondaryReady ? "ok" : "init_failed",
                                  (int32_t)s_state.fdcSecondaryReady);
        } else {
            s_state.fdcSecondaryReady = false;
            sensorarrayLogStartup("fdc_secondary", ESP_ERR_NOT_SUPPORTED, "skip_i2c1_unavailable", 0);
        }
    } else {
        s_state.fdcPrimaryReady = false;
        s_state.fdcSecondaryReady = false;
        sensorarrayLogStartup("fdc_primary", ESP_ERR_INVALID_STATE, "skip_board_unavailable", 0);
        sensorarrayLogStartup("fdc_secondary", ESP_ERR_INVALID_STATE, "skip_board_unavailable", 0);
    }

    sensorarrayRunBringupLoop();
}

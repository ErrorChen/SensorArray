#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

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

#ifndef CONFIG_SENSORARRAY_FDC_PRIMARY_I2C_ADDR
#define CONFIG_SENSORARRAY_FDC_PRIMARY_I2C_ADDR 0x2Bu
#endif

#ifndef CONFIG_SENSORARRAY_FDC_SECONDARY_I2C_ADDR
#define CONFIG_SENSORARRAY_FDC_SECONDARY_I2C_ADDR 0x2Au
#endif

#ifndef CONFIG_SENSORARRAY_FDC_STARTUP_PROBE
#define CONFIG_SENSORARRAY_FDC_STARTUP_PROBE 1
#endif

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
#define SENSORARRAY_FDC_I2C_ADDR_LOW 0x2Au
#define SENSORARRAY_FDC_I2C_ADDR_HIGH 0x2Bu
#define SENSORARRAY_FDC_REG_MANUFACTURER_ID 0x7Eu

#define SENSORARRAY_ADS_MUX_AINCOM 0x0Au

#define SENSORARRAY_S1 1u
#define SENSORARRAY_S2 2u

#define SENSORARRAY_D1 1u
#define SENSORARRAY_D4 4u
#define SENSORARRAY_D8 8u

#define SENSORARRAY_NA "na"

typedef enum {
    SENSORARRAY_FDC_DEV_PRIMARY = 0,
    SENSORARRAY_FDC_DEV_SECONDARY = 1,
} sensorarrayFdcDeviceId_t;

typedef enum {
    SENSORARRAY_PATH_RESISTIVE = 0,
    SENSORARRAY_PATH_CAPACITIVE = 1,
} sensorarrayPath_t;

typedef struct {
    uint8_t sColumn;
    uint8_t dLine;
    sensorarrayPath_t path;
    bool selAEnabled;
    bool selBEnabled;
    const char *mapLabel;
} sensorarrayRouteMap_t;

typedef struct {
    uint8_t dLine;
    sensorarrayFdcDeviceId_t devId;
    Fdc2214CapChannel_t channel;
    const char *mapLabel;
} sensorarrayFdcDLineMap_t;

typedef struct {
    const char *label;
    const BoardSupportI2cCtx_t *i2cCtx;
    uint8_t i2cAddr;
    Fdc2214CapDevice_t *handle;
    bool ready;
    bool haveIds;
    uint16_t manufacturerId;
    uint16_t deviceId;
} sensorarrayFdcDeviceState_t;

typedef enum {
    SENSORARRAY_RES_CONVERT_OK = 0,
    SENSORARRAY_RES_CONVERT_SIGNED_INPUT,
    SENSORARRAY_RES_CONVERT_MODEL_INVALID,
} sensorarrayResConvertResult_t;

typedef struct {
    const char *status;
    bool haveIds;
    uint16_t manufacturerId;
    uint16_t deviceId;
    int32_t detail;
} sensorarrayFdcInitDiag_t;

typedef struct {
    spi_device_handle_t spiDevice;
    ads126xAdcHandle_t ads;
    bool adsReady;
    bool adsRefReady;

    sensorarrayFdcDeviceState_t fdcPrimary;   // D1..D4 (C1..C4)
    sensorarrayFdcDeviceState_t fdcSecondary; // D5..D8 (C5..C8)
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

static const char *sensorarrayRefReadyBit(void)
{
    if (!s_state.adsReady) {
        return SENSORARRAY_NA;
    }
    return s_state.adsRefReady ? "1" : "0";
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

static const char *sensorarrayFmtHexU8(char *buf, size_t bufSize, bool valid, uint8_t value)
{
    if (!valid) {
        return SENSORARRAY_NA;
    }
    snprintf(buf, bufSize, "0x%02X", value);
    return buf;
}

static const char *sensorarrayFmtHexU16(char *buf, size_t bufSize, bool valid, uint16_t value)
{
    if (!valid) {
        return SENSORARRAY_NA;
    }
    snprintf(buf, bufSize, "0x%04X", value);
    return buf;
}

static const char *sensorarrayFmtI2cPort(char *buf, size_t bufSize, const BoardSupportI2cCtx_t *ctx)
{
    if (!ctx) {
        return SENSORARRAY_NA;
    }
    snprintf(buf, bufSize, "%d", (int)ctx->Port);
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
                              const char *fdcDev,
                              const char *i2cPort,
                              const char *i2cAddr,
                              const char *idMfg,
                              const char *idDev,
                              const char *map,
                              esp_err_t err,
                              const char *status)
{
    printf("DBG,point=%s,kind=%s,column=%s,dline=%s,sw=%s,ref=%s,mode=%s,value=%s,valueUv=%s,"
           "valueMohm=%s,raw=%s,wd=%s,amp=%s,fdcDev=%s,i2cPort=%s,i2cAddr=%s,idMfg=%s,idDev=%s,"
           "refReady=%s,map=%s,err=%ld,status=%s\n",
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
           fdcDev ? fdcDev : SENSORARRAY_NA,
           i2cPort ? i2cPort : SENSORARRAY_NA,
           i2cAddr ? i2cAddr : SENSORARRAY_NA,
           idMfg ? idMfg : SENSORARRAY_NA,
           idDev ? idDev : SENSORARRAY_NA,
           sensorarrayRefReadyBit(),
           map ? map : SENSORARRAY_NA,
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
                      SENSORARRAY_NA,
                      SENSORARRAY_NA,
                      SENSORARRAY_NA,
                      SENSORARRAY_NA,
                      SENSORARRAY_NA,
                      SENSORARRAY_NA,
                      err,
                      status);
}

static void sensorarrayLogStartupFdc(const char *mode,
                                     const sensorarrayFdcDeviceState_t *fdcState,
                                     esp_err_t err,
                                     const char *status,
                                     int32_t detailValue,
                                     bool hasIds,
                                     uint16_t manufacturerId,
                                     uint16_t deviceId,
                                     const char *map)
{
    char valueBuf[24];
    char portBuf[12];
    char addrBuf[12];
    char idMfgBuf[12];
    char idDevBuf[12];

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
                      fdcState ? fdcState->label : SENSORARRAY_NA,
                      sensorarrayFmtI2cPort(portBuf, sizeof(portBuf), fdcState ? fdcState->i2cCtx : NULL),
                      sensorarrayFmtHexU8(addrBuf, sizeof(addrBuf), fdcState != NULL, fdcState ? fdcState->i2cAddr : 0),
                      sensorarrayFmtHexU16(idMfgBuf, sizeof(idMfgBuf), hasIds, manufacturerId),
                      sensorarrayFmtHexU16(idDevBuf, sizeof(idDevBuf), hasIds, deviceId),
                      map,
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
    // Keep app-level autoscan policy explicit for bring-up:
    // 1ch->single CH0, 2ch->rr=0, 3ch->rr=1, 4ch->rr=2.
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

static const sensorarrayRouteMap_t s_sensorarrayRouteMap[] = {
    // TMUX1108 selects S-line path. TMUX1134 SELA/SELB route cap/volt branches for debug points.
    { SENSORARRAY_S1, SENSORARRAY_D1, SENSORARRAY_PATH_RESISTIVE, true, false, "S1D1_res_selA_on" },
    { SENSORARRAY_S2, SENSORARRAY_D4, SENSORARRAY_PATH_CAPACITIVE, true, false, "S2D4_cap_selA_on" },
    { SENSORARRAY_S2, SENSORARRAY_D4, SENSORARRAY_PATH_RESISTIVE, false, false, "S2D4_volt_selA_off" },
    { SENSORARRAY_S2, SENSORARRAY_D8, SENSORARRAY_PATH_CAPACITIVE, false, true, "S2D8_cap_selB_on" },
    { SENSORARRAY_S2, SENSORARRAY_D8, SENSORARRAY_PATH_RESISTIVE, false, false, "S2D8_volt_selB_off" },
};

static const sensorarrayFdcDLineMap_t s_sensorarrayFdcDLineMap[] = {
    { 1u, SENSORARRAY_FDC_DEV_PRIMARY, FDC2214_CH0, "D1_primary_ch0" },
    { 2u, SENSORARRAY_FDC_DEV_PRIMARY, FDC2214_CH1, "D2_primary_ch1" },
    { 3u, SENSORARRAY_FDC_DEV_PRIMARY, FDC2214_CH2, "D3_primary_ch2" },
    { 4u, SENSORARRAY_FDC_DEV_PRIMARY, FDC2214_CH3, "D4_primary_ch3" },
    { 5u, SENSORARRAY_FDC_DEV_SECONDARY, FDC2214_CH0, "D5_secondary_ch0" },
    { 6u, SENSORARRAY_FDC_DEV_SECONDARY, FDC2214_CH1, "D6_secondary_ch1" },
    { 7u, SENSORARRAY_FDC_DEV_SECONDARY, FDC2214_CH2, "D7_secondary_ch2" },
    { 8u, SENSORARRAY_FDC_DEV_SECONDARY, FDC2214_CH3, "D8_secondary_ch3" },
};

static sensorarrayFdcDeviceState_t *sensorarrayGetFdcState(sensorarrayFdcDeviceId_t devId)
{
    switch (devId) {
    case SENSORARRAY_FDC_DEV_PRIMARY:
        return &s_state.fdcPrimary;
    case SENSORARRAY_FDC_DEV_SECONDARY:
        return &s_state.fdcSecondary;
    default:
        return NULL;
    }
}

static const sensorarrayRouteMap_t *sensorarrayFindRouteMap(uint8_t sColumn,
                                                            uint8_t dLine,
                                                            sensorarrayPath_t path)
{
    for (size_t i = 0; i < (sizeof(s_sensorarrayRouteMap) / sizeof(s_sensorarrayRouteMap[0])); ++i) {
        const sensorarrayRouteMap_t *entry = &s_sensorarrayRouteMap[i];
        if (entry->sColumn == sColumn && entry->dLine == dLine && entry->path == path) {
            return entry;
        }
    }
    return NULL;
}

static const sensorarrayFdcDLineMap_t *sensorarrayFindFdcDLineMap(uint8_t dLine)
{
    for (size_t i = 0; i < (sizeof(s_sensorarrayFdcDLineMap) / sizeof(s_sensorarrayFdcDLineMap[0])); ++i) {
        if (s_sensorarrayFdcDLineMap[i].dLine == dLine) {
            return &s_sensorarrayFdcDLineMap[i];
        }
    }
    return NULL;
}

static bool sensorarrayParseI2cAddress(uint32_t configuredAddress, uint8_t *outAddress)
{
    if (!outAddress || configuredAddress > 0x7Fu) {
        return false;
    }
    *outAddress = (uint8_t)configuredAddress;
    return true;
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

static void sensorarrayProbeFdcBus(const sensorarrayFdcDeviceState_t *fdcState)
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

static void sensorarrayInitFdcDiag(sensorarrayFdcInitDiag_t *diag)
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

static esp_err_t sensorarrayInitFdcDevice(const BoardSupportI2cCtx_t *i2cCtx,
                                          uint8_t i2cAddr,
                                          uint8_t channels,
                                          Fdc2214CapDevice_t **outDev,
                                          sensorarrayFdcInitDiag_t *outDiag)
{
    sensorarrayInitFdcDiag(outDiag);

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

static bool sensorarrayAdsMuxForDLine(uint8_t dLine, uint8_t *muxp, uint8_t *muxn)
{
    if (!muxp || !muxn || dLine < 1u || dLine > 8u) {
        return false;
    }

    *muxp = (uint8_t)(8u - dLine); // D1->AIN7 ... D8->AIN0
    *muxn = SENSORARRAY_ADS_MUX_AINCOM;
    return true;
}

static esp_err_t sensorarrayApplyRoute(uint8_t sColumn,
                                       uint8_t dLine,
                                       sensorarrayPath_t path,
                                       tmux1108Source_t swSource,
                                       const char **outMapLabel)
{
    if (outMapLabel) {
        *outMapLabel = SENSORARRAY_NA;
    }
    if (!s_state.tmuxReady) {
        return ESP_ERR_INVALID_STATE;
    }
    if (sColumn < 1u || sColumn > 8u || dLine < 1u || dLine > 8u) {
        return ESP_ERR_INVALID_ARG;
    }

    const sensorarrayRouteMap_t *routeMap = sensorarrayFindRouteMap(sColumn, dLine, path);
    if (!routeMap) {
        return ESP_ERR_NOT_SUPPORTED;
    }

    esp_err_t err = tmuxSwitchSelectRow((uint8_t)(sColumn - 1u)); // TMUX1108 selects S1..S8
    if (err != ESP_OK) {
        return err;
    }
    sensorarrayDelayMs(SENSORARRAY_SETTLE_AFTER_COLUMN_MS);

    err = tmuxSwitchSetSelAEnabled(routeMap->selAEnabled);
    if (err != ESP_OK) {
        return err;
    }
    err = tmuxSwitchSetSelBEnabled(routeMap->selBEnabled);
    if (err != ESP_OK) {
        return err;
    }
    sensorarrayDelayMs(SENSORARRAY_SETTLE_AFTER_PATH_MS);

    err = tmuxSwitchSet1108Source(swSource); // SW controls REF/GND source state.
    if (err != ESP_OK) {
        return err;
    }
    sensorarrayDelayMs(SENSORARRAY_SETTLE_AFTER_SW_MS);

    if (outMapLabel) {
        *outMapLabel = routeMap->mapLabel;
    }
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

static sensorarrayResConvertResult_t sensorarrayTryResistanceMohm(int32_t uv, int32_t *outMohm)
{
    if (!outMohm) {
        return SENSORARRAY_RES_CONVERT_MODEL_INVALID;
    }
    if (uv < 0) {
        return SENSORARRAY_RES_CONVERT_SIGNED_INPUT;
    }
    if (uv == 0 || (uint32_t)uv >= SENSORARRAY_RESIST_EXCITATION_UV) {
        return SENSORARRAY_RES_CONVERT_MODEL_INVALID;
    }

    int64_t numerator = (int64_t)SENSORARRAY_RESIST_REF_OHMS * 1000 * (int64_t)uv;
    int64_t denominator = (int64_t)SENSORARRAY_RESIST_EXCITATION_UV - (int64_t)uv;
    if (denominator == 0) {
        return SENSORARRAY_RES_CONVERT_MODEL_INVALID;
    }

    *outMohm = (int32_t)(numerator / denominator);
    return SENSORARRAY_RES_CONVERT_OK;
}

static const sensorarrayFdcDLineMap_t *sensorarrayGetFdcMapForDLine(uint8_t dLine)
{
    return sensorarrayFindFdcDLineMap(dLine);
}

static sensorarrayFdcDeviceState_t *sensorarrayGetFdcStateForDLine(uint8_t dLine,
                                                                    const sensorarrayFdcDLineMap_t **outMap)
{
    const sensorarrayFdcDLineMap_t *map = sensorarrayGetFdcMapForDLine(dLine);
    if (outMap) {
        *outMap = map;
    }
    if (!map) {
        return NULL;
    }
    return sensorarrayGetFdcState(map->devId);
}

static const char *sensorarrayBuildMapLabel(char *buf,
                                            size_t bufSize,
                                            const char *routeMap,
                                            const char *fdcMap)
{
    if (!buf || bufSize == 0u) {
        return SENSORARRAY_NA;
    }

    const char *route = routeMap ? routeMap : SENSORARRAY_NA;
    const char *fdc = fdcMap ? fdcMap : SENSORARRAY_NA;
    bool routeNa = (strcmp(route, SENSORARRAY_NA) == 0);
    bool fdcNa = (strcmp(fdc, SENSORARRAY_NA) == 0);

    if (routeNa && fdcNa) {
        return SENSORARRAY_NA;
    }
    if (fdcNa) {
        return route;
    }
    if (routeNa) {
        return fdc;
    }

    snprintf(buf, bufSize, "%s|%s", route, fdc);
    return buf;
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
    char mapBuf[72];

    esp_err_t err = ESP_OK;
    int32_t uv = 0;
    int32_t raw = 0;
    int32_t mohm = 0;
    bool haveUv = false;
    bool haveRaw = false;
    bool haveMohm = false;
    const char *status = "res_ok";
    const char *routeMap = SENSORARRAY_NA;

    if (!s_state.adsReady) {
        status = "skip_ads_unavailable";
    } else if (!s_state.adsRefReady) {
        status = "skip_ref_not_ready";
    } else {
        err = sensorarrayApplyRoute(SENSORARRAY_S1,
                                    SENSORARRAY_D1,
                                    SENSORARRAY_PATH_RESISTIVE,
                                    TMUX1108_SOURCE_REF,
                                    &routeMap);
        if (err != ESP_OK) {
            status = (err == ESP_ERR_NOT_SUPPORTED) ? "route_map_missing" : "route_error";
        } else {
            err = sensorarrayReadAdsUv(SENSORARRAY_D1, true, &raw, &uv);
            if (err != ESP_OK) {
                status = "ads_read_error";
            } else {
                haveRaw = true;
                haveUv = true;
                sensorarrayResConvertResult_t resResult = sensorarrayTryResistanceMohm(uv, &mohm);
                if (resResult == SENSORARRAY_RES_CONVERT_OK) {
                    haveMohm = true;
                    status = "res_ok";
                } else if (resResult == SENSORARRAY_RES_CONVERT_SIGNED_INPUT) {
                    status = "adc_ok_signed_value";
                } else {
                    status = "adc_ok_but_res_model_invalid";
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
                      SENSORARRAY_NA,
                      SENSORARRAY_NA,
                      SENSORARRAY_NA,
                      SENSORARRAY_NA,
                      SENSORARRAY_NA,
                      sensorarrayBuildMapLabel(mapBuf, sizeof(mapBuf), routeMap, SENSORARRAY_NA),
                      err,
                      status);
}

static void sensorarrayDebugReadPiezoCap(const char *pointLabel, uint8_t sColumn, uint8_t dLine)
{
    char valueBuf[24];
    char rawBuf[24];
    char wdBuf[8];
    char ampBuf[8];
    char portBuf[12];
    char addrBuf[12];
    char idMfgBuf[12];
    char idDevBuf[12];
    char mapBuf[72];
    char columnBuf[6];
    char dLineBuf[6];

    esp_err_t err = ESP_OK;
    Fdc2214CapSample_t sample = {0};
    bool haveSample = false;
    bool haveFlags = false;
    const sensorarrayFdcDLineMap_t *fdcMap = NULL;
    sensorarrayFdcDeviceState_t *fdcState = sensorarrayGetFdcStateForDLine(dLine, &fdcMap);
    const char *routeMap = SENSORARRAY_NA;
    const char *status = "ok";

    if (!fdcMap || !fdcState) {
        err = ESP_ERR_INVALID_ARG;
        status = "mapping_error";
    } else if (!fdcState->ready || !fdcState->handle) {
        status = "skip_fdc_unavailable";
    } else {
        err = sensorarrayApplyRoute(sColumn,
                                    dLine,
                                    SENSORARRAY_PATH_CAPACITIVE,
                                    TMUX1108_SOURCE_GND,
                                    &routeMap);
        if (err != ESP_OK) {
            status = (err == ESP_ERR_NOT_SUPPORTED) ? "route_map_missing" : "route_error";
        } else {
            err = sensorarrayReadFdcSample(fdcState->handle, fdcMap->channel, true, &sample);
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

    snprintf(columnBuf, sizeof(columnBuf), "S%u", (unsigned)sColumn);
    snprintf(dLineBuf, sizeof(dLineBuf), "D%u", (unsigned)dLine);

    sensorarrayLogDbg(pointLabel,
                      "cap",
                      columnBuf,
                      dLineBuf,
                      "LOW",
                      "fdc",
                      sensorarrayFmtU32(valueBuf, sizeof(valueBuf), haveSample, sample.Raw28),
                      SENSORARRAY_NA,
                      SENSORARRAY_NA,
                      sensorarrayFmtU32(rawBuf, sizeof(rawBuf), haveSample, sample.Raw28),
                      sensorarrayFmtBool(wdBuf, sizeof(wdBuf), haveFlags, sample.ErrWatchdog),
                      sensorarrayFmtBool(ampBuf, sizeof(ampBuf), haveFlags, sample.ErrAmplitude),
                      fdcState ? fdcState->label : SENSORARRAY_NA,
                      sensorarrayFmtI2cPort(portBuf, sizeof(portBuf), fdcState ? fdcState->i2cCtx : NULL),
                      sensorarrayFmtHexU8(addrBuf, sizeof(addrBuf), fdcState != NULL, fdcState ? fdcState->i2cAddr : 0),
                      sensorarrayFmtHexU16(idMfgBuf,
                                           sizeof(idMfgBuf),
                                           (fdcState != NULL) && fdcState->haveIds,
                                           fdcState ? fdcState->manufacturerId : 0),
                      sensorarrayFmtHexU16(idDevBuf,
                                           sizeof(idDevBuf),
                                           (fdcState != NULL) && fdcState->haveIds,
                                           fdcState ? fdcState->deviceId : 0),
                      sensorarrayBuildMapLabel(mapBuf,
                                               sizeof(mapBuf),
                                               routeMap,
                                               fdcMap ? fdcMap->mapLabel : SENSORARRAY_NA),
                      err,
                      status);
}

static void sensorarrayDebugReadPiezoVolt(const char *pointLabel, uint8_t sColumn, uint8_t dLine)
{
    char valueBuf[24];
    char uvBuf[24];
    char rawBuf[24];
    char mapBuf[72];
    char columnBuf[6];
    char dLineBuf[6];

    esp_err_t err = ESP_OK;
    int32_t uv = 0;
    int32_t raw = 0;
    bool haveUv = false;
    bool haveRaw = false;
    const char *status = "ok";
    const char *routeMap = SENSORARRAY_NA;

    if (!s_state.adsReady) {
        status = "skip_ads_unavailable";
    } else {
        err = sensorarrayApplyRoute(sColumn,
                                    dLine,
                                    SENSORARRAY_PATH_RESISTIVE,
                                    TMUX1108_SOURCE_GND,
                                    &routeMap);
        if (err != ESP_OK) {
            status = (err == ESP_ERR_NOT_SUPPORTED) ? "route_map_missing" : "route_error";
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

    snprintf(columnBuf, sizeof(columnBuf), "S%u", (unsigned)sColumn);
    snprintf(dLineBuf, sizeof(dLineBuf), "D%u", (unsigned)dLine);

    sensorarrayLogDbg(pointLabel,
                      "volt",
                      columnBuf,
                      dLineBuf,
                      "LOW",
                      "ads",
                      sensorarrayFmtI32(valueBuf, sizeof(valueBuf), haveUv, uv),
                      sensorarrayFmtI32(uvBuf, sizeof(uvBuf), haveUv, uv),
                      SENSORARRAY_NA,
                      sensorarrayFmtI32(rawBuf, sizeof(rawBuf), haveRaw, raw),
                      SENSORARRAY_NA,
                      SENSORARRAY_NA,
                      SENSORARRAY_NA,
                      SENSORARRAY_NA,
                      SENSORARRAY_NA,
                      SENSORARRAY_NA,
                      SENSORARRAY_NA,
                      sensorarrayBuildMapLabel(mapBuf, sizeof(mapBuf), routeMap, SENSORARRAY_NA),
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

static void sensorarrayResetFdcState(sensorarrayFdcDeviceState_t *fdcState,
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

void app_main(void)
{
    uint8_t requestedChannels = sensorarrayNormalizeFdcChannels((uint8_t)CONFIG_FDC2214CAP_CHANNELS);
    if (requestedChannels < SENSORARRAY_FDC_REQUIRED_CHANNELS) {
        requestedChannels = SENSORARRAY_FDC_REQUIRED_CHANNELS;
    }
    s_state.fdcConfiguredChannels = requestedChannels;

    sensorarrayResetFdcState(&s_state.fdcPrimary,
                             "primary",
                             (uint8_t)(CONFIG_SENSORARRAY_FDC_PRIMARY_I2C_ADDR & 0xFFu));
    sensorarrayResetFdcState(&s_state.fdcSecondary,
                             "secondary",
                             (uint8_t)(CONFIG_SENSORARRAY_FDC_SECONDARY_I2C_ADDR & 0xFFu));

    bool primaryAddrValid =
        sensorarrayParseI2cAddress((uint32_t)CONFIG_SENSORARRAY_FDC_PRIMARY_I2C_ADDR, &s_state.fdcPrimary.i2cAddr);
    bool secondaryAddrValid = sensorarrayParseI2cAddress((uint32_t)CONFIG_SENSORARRAY_FDC_SECONDARY_I2C_ADDR,
                                                         &s_state.fdcSecondary.i2cAddr);

    sensorarrayLogStartup("app", ESP_OK, "start", 0);
    sensorarrayLogStartup("fdc_channels", ESP_OK, "policy_applied", (int32_t)requestedChannels);
    sensorarrayLogStartupFdc("fdc_cfg",
                             &s_state.fdcPrimary,
                             primaryAddrValid ? ESP_OK : ESP_ERR_INVALID_ARG,
                             primaryAddrValid ? "configured" : "invalid_addr_config",
                             (int32_t)CONFIG_SENSORARRAY_FDC_PRIMARY_I2C_ADDR,
                             false,
                             0,
                             0,
                             "D1..D4");
    sensorarrayLogStartupFdc("fdc_cfg",
                             &s_state.fdcSecondary,
                             secondaryAddrValid ? ESP_OK : ESP_ERR_INVALID_ARG,
                             secondaryAddrValid ? "configured" : "invalid_addr_config",
                             (int32_t)CONFIG_SENSORARRAY_FDC_SECONDARY_I2C_ADDR,
                             false,
                             0,
                             0,
                             "D5..D8");

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
        s_state.fdcPrimary.i2cCtx = boardSupportGetI2cCtx();
        s_state.fdcSecondary.i2cCtx = boardSupportGetI2c1Ctx();

        if (!primaryAddrValid) {
            sensorarrayLogStartupFdc("fdc_init",
                                     &s_state.fdcPrimary,
                                     ESP_ERR_INVALID_ARG,
                                     "skip_invalid_addr_config",
                                     (int32_t)CONFIG_SENSORARRAY_FDC_PRIMARY_I2C_ADDR,
                                     false,
                                     0,
                                     0,
                                     "D1..D4");
        } else if (!s_state.fdcPrimary.i2cCtx) {
            sensorarrayLogStartupFdc("fdc_init",
                                     &s_state.fdcPrimary,
                                     ESP_ERR_NOT_SUPPORTED,
                                     "skip_i2c0_unavailable",
                                     0,
                                     false,
                                     0,
                                     0,
                                     "D1..D4");
        } else {
            sensorarrayProbeFdcBus(&s_state.fdcPrimary);

            sensorarrayFdcInitDiag_t diag = {0};
            err = sensorarrayInitFdcDevice(s_state.fdcPrimary.i2cCtx,
                                           s_state.fdcPrimary.i2cAddr,
                                           requestedChannels,
                                           &s_state.fdcPrimary.handle,
                                           &diag);
            s_state.fdcPrimary.ready = (err == ESP_OK);
            s_state.fdcPrimary.haveIds = diag.haveIds;
            s_state.fdcPrimary.manufacturerId = diag.manufacturerId;
            s_state.fdcPrimary.deviceId = diag.deviceId;
            sensorarrayLogStartupFdc("fdc_init",
                                     &s_state.fdcPrimary,
                                     err,
                                     diag.status,
                                     (err == ESP_OK) ? (int32_t)requestedChannels : diag.detail,
                                     diag.haveIds,
                                     diag.manufacturerId,
                                     diag.deviceId,
                                     "D1..D4");
        }

        if (!secondaryAddrValid) {
            sensorarrayLogStartupFdc("fdc_init",
                                     &s_state.fdcSecondary,
                                     ESP_ERR_INVALID_ARG,
                                     "skip_invalid_addr_config",
                                     (int32_t)CONFIG_SENSORARRAY_FDC_SECONDARY_I2C_ADDR,
                                     false,
                                     0,
                                     0,
                                     "D5..D8");
        } else if (!s_state.fdcSecondary.i2cCtx) {
            sensorarrayLogStartupFdc("fdc_init",
                                     &s_state.fdcSecondary,
                                     ESP_ERR_NOT_SUPPORTED,
                                     "skip_i2c1_unavailable",
                                     0,
                                     false,
                                     0,
                                     0,
                                     "D5..D8");
        } else {
            sensorarrayProbeFdcBus(&s_state.fdcSecondary);

            sensorarrayFdcInitDiag_t diag = {0};
            err = sensorarrayInitFdcDevice(s_state.fdcSecondary.i2cCtx,
                                           s_state.fdcSecondary.i2cAddr,
                                           requestedChannels,
                                           &s_state.fdcSecondary.handle,
                                           &diag);
            s_state.fdcSecondary.ready = (err == ESP_OK);
            s_state.fdcSecondary.haveIds = diag.haveIds;
            s_state.fdcSecondary.manufacturerId = diag.manufacturerId;
            s_state.fdcSecondary.deviceId = diag.deviceId;
            sensorarrayLogStartupFdc("fdc_init",
                                     &s_state.fdcSecondary,
                                     err,
                                     diag.status,
                                     (err == ESP_OK) ? (int32_t)requestedChannels : diag.detail,
                                     diag.haveIds,
                                     diag.manufacturerId,
                                     diag.deviceId,
                                     "D5..D8");
        }
    } else {
        sensorarrayLogStartupFdc("fdc_init",
                                 &s_state.fdcPrimary,
                                 ESP_ERR_INVALID_STATE,
                                 "skip_board_unavailable",
                                 0,
                                 false,
                                 0,
                                 0,
                                 "D1..D4");
        sensorarrayLogStartupFdc("fdc_init",
                                 &s_state.fdcSecondary,
                                 ESP_ERR_INVALID_STATE,
                                 "skip_board_unavailable",
                                 0,
                                 false,
                                 0,
                                 0,
                                 "D5..D8");
    }

    sensorarrayRunBringupLoop();
}

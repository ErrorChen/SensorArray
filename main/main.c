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

#ifndef CONFIG_SENSORARRAY_DEBUG_MODE_ROUTE_IDLE
#define CONFIG_SENSORARRAY_DEBUG_MODE_ROUTE_IDLE 0
#endif
#ifndef CONFIG_SENSORARRAY_DEBUG_MODE_ROUTE_FIXED_STATE
#define CONFIG_SENSORARRAY_DEBUG_MODE_ROUTE_FIXED_STATE 0
#endif
#ifndef CONFIG_SENSORARRAY_DEBUG_MODE_ROUTE_STEP_ONCE
#define CONFIG_SENSORARRAY_DEBUG_MODE_ROUTE_STEP_ONCE 0
#endif
#ifndef CONFIG_SENSORARRAY_DEBUG_MODE_ROUTE_SCAN_LOOP
#define CONFIG_SENSORARRAY_DEBUG_MODE_ROUTE_SCAN_LOOP 1
#endif
#ifndef CONFIG_SENSORARRAY_DEBUG_MODE_ADS_SELFTEST
#define CONFIG_SENSORARRAY_DEBUG_MODE_ADS_SELFTEST 0
#endif
#ifndef CONFIG_SENSORARRAY_DEBUG_MODE_FDC_SELFTEST
#define CONFIG_SENSORARRAY_DEBUG_MODE_FDC_SELFTEST 0
#endif
#ifndef CONFIG_SENSORARRAY_DEBUG_MODE_S1D1_RESISTOR
#define CONFIG_SENSORARRAY_DEBUG_MODE_S1D1_RESISTOR 0
#endif
#ifndef CONFIG_SENSORARRAY_DEBUG_S1D1_ROUTE_ONLY
#define CONFIG_SENSORARRAY_DEBUG_S1D1_ROUTE_ONLY 1
#endif
#ifndef CONFIG_SENSORARRAY_DEBUG_S1D1_ROUTE_AND_ADS_READ
#define CONFIG_SENSORARRAY_DEBUG_S1D1_ROUTE_AND_ADS_READ 0
#endif
#ifndef CONFIG_SENSORARRAY_DEBUG_S1D1_ADS_INTERVAL_MS
#define CONFIG_SENSORARRAY_DEBUG_S1D1_ADS_INTERVAL_MS 1000
#endif
#ifndef CONFIG_SENSORARRAY_DEBUG_S1D1_ADS_DISCARD_FIRST
#define CONFIG_SENSORARRAY_DEBUG_S1D1_ADS_DISCARD_FIRST 1
#endif
#ifndef CONFIG_SENSORARRAY_S1D1_RESISTOR_STATIC_DEBUG
#define CONFIG_SENSORARRAY_S1D1_RESISTOR_STATIC_DEBUG 0
#endif
#ifndef CONFIG_SENSORARRAY_S1D1_STATIC_ROUTE_ONLY
#define CONFIG_SENSORARRAY_S1D1_STATIC_ROUTE_ONLY 0
#endif
#ifndef CONFIG_SENSORARRAY_S1D1_STATIC_ROUTE_AND_READ
#define CONFIG_SENSORARRAY_S1D1_STATIC_ROUTE_AND_READ 1
#endif
#ifndef CONFIG_SENSORARRAY_S1D1_STATIC_LOOP_DELAY_MS
#define CONFIG_SENSORARRAY_S1D1_STATIC_LOOP_DELAY_MS 1000
#endif
#ifndef CONFIG_SENSORARRAY_S1D1_STATIC_REAPPLY_ROUTE_EACH_LOOP
#define CONFIG_SENSORARRAY_S1D1_STATIC_REAPPLY_ROUTE_EACH_LOOP 0
#endif
#ifndef CONFIG_SENSORARRAY_DEBUG_ROUTE_STEP_DELAY_MS
#define CONFIG_SENSORARRAY_DEBUG_ROUTE_STEP_DELAY_MS 2
#endif
#ifndef CONFIG_SENSORARRAY_DEBUG_FIXED_HOLD_FOREVER
#define CONFIG_SENSORARRAY_DEBUG_FIXED_HOLD_FOREVER 1
#endif
#ifndef CONFIG_SENSORARRAY_DEBUG_FIXED_HOLD_MS
#define CONFIG_SENSORARRAY_DEBUG_FIXED_HOLD_MS 0
#endif
#ifndef CONFIG_SENSORARRAY_DEBUG_FIXED_S_COLUMN
#define CONFIG_SENSORARRAY_DEBUG_FIXED_S_COLUMN 1
#endif
#ifndef CONFIG_SENSORARRAY_DEBUG_FIXED_D_LINE
#define CONFIG_SENSORARRAY_DEBUG_FIXED_D_LINE 1
#endif
#ifndef CONFIG_SENSORARRAY_DEBUG_FIXED_PATH_RESISTIVE
#define CONFIG_SENSORARRAY_DEBUG_FIXED_PATH_RESISTIVE 1
#endif
#ifndef CONFIG_SENSORARRAY_DEBUG_FIXED_PATH_CAPACITIVE
#define CONFIG_SENSORARRAY_DEBUG_FIXED_PATH_CAPACITIVE 0
#endif
#ifndef CONFIG_SENSORARRAY_DEBUG_FIXED_PATH_VOLTAGE
#define CONFIG_SENSORARRAY_DEBUG_FIXED_PATH_VOLTAGE 0
#endif
#ifndef CONFIG_SENSORARRAY_DEBUG_FIXED_SW_SOURCE_GND
#define CONFIG_SENSORARRAY_DEBUG_FIXED_SW_SOURCE_GND 1
#endif
#ifndef CONFIG_SENSORARRAY_DEBUG_FIXED_SW_SOURCE_REF
#define CONFIG_SENSORARRAY_DEBUG_FIXED_SW_SOURCE_REF 0
#endif
#ifndef CONFIG_SENSORARRAY_DEBUG_FIXED_SELA_LEVEL
#define CONFIG_SENSORARRAY_DEBUG_FIXED_SELA_LEVEL 0
#endif
#ifndef CONFIG_SENSORARRAY_DEBUG_FIXED_SELB_LEVEL
#define CONFIG_SENSORARRAY_DEBUG_FIXED_SELB_LEVEL 0
#endif
#ifndef CONFIG_SENSORARRAY_DEBUG_FIXED_SKIP_ADS_READ
#define CONFIG_SENSORARRAY_DEBUG_FIXED_SKIP_ADS_READ 1
#endif
#ifndef CONFIG_SENSORARRAY_DEBUG_FIXED_SKIP_FDC_READ
#define CONFIG_SENSORARRAY_DEBUG_FIXED_SKIP_FDC_READ 1
#endif
#ifndef CONFIG_SENSORARRAY_DEBUG_SCAN_LOOP_DELAY_MS
#define CONFIG_SENSORARRAY_DEBUG_SCAN_LOOP_DELAY_MS 250
#endif
#ifndef CONFIG_SENSORARRAY_DEBUG_STEP_ONCE_DELAY_MS
#define CONFIG_SENSORARRAY_DEBUG_STEP_ONCE_DELAY_MS 50
#endif
#ifndef CONFIG_SENSORARRAY_DEBUG_ADS_FORCE_MUXP
#define CONFIG_SENSORARRAY_DEBUG_ADS_FORCE_MUXP 7
#endif
#ifndef CONFIG_SENSORARRAY_DEBUG_ADS_FORCE_MUXN
#define CONFIG_SENSORARRAY_DEBUG_ADS_FORCE_MUXN 10
#endif
#ifndef CONFIG_SENSORARRAY_DEBUG_ADS_FORCE_REFMUX
#define CONFIG_SENSORARRAY_DEBUG_ADS_FORCE_REFMUX 0x00
#endif
#ifndef CONFIG_SENSORARRAY_DEBUG_ADS_STOP1_BEFORE_MUX
#define CONFIG_SENSORARRAY_DEBUG_ADS_STOP1_BEFORE_MUX 1
#endif
#ifndef CONFIG_SENSORARRAY_DEBUG_ADS_SETTLE_AFTER_MUX_MS
#define CONFIG_SENSORARRAY_DEBUG_ADS_SETTLE_AFTER_MUX_MS 2
#endif
#ifndef CONFIG_SENSORARRAY_DEBUG_ADS_START1_BEFORE_READ
#define CONFIG_SENSORARRAY_DEBUG_ADS_START1_BEFORE_READ 1
#endif
#ifndef CONFIG_SENSORARRAY_DEBUG_ADS_DISCARD_COUNT
#define CONFIG_SENSORARRAY_DEBUG_ADS_DISCARD_COUNT 1
#endif
#ifndef CONFIG_SENSORARRAY_DEBUG_ADS_READ_RETRY_COUNT
#define CONFIG_SENSORARRAY_DEBUG_ADS_READ_RETRY_COUNT 1
#endif
#ifndef CONFIG_SENSORARRAY_DEBUG_ADS_SAMPLE_COUNT
#define CONFIG_SENSORARRAY_DEBUG_ADS_SAMPLE_COUNT 1
#endif
#ifndef CONFIG_SENSORARRAY_DEBUG_FDC_D_LINE
#define CONFIG_SENSORARRAY_DEBUG_FDC_D_LINE 5
#endif
#ifndef CONFIG_SENSORARRAY_DEBUG_FDC_DISCARD_FIRST
#define CONFIG_SENSORARRAY_DEBUG_FDC_DISCARD_FIRST 1
#endif
#ifndef CONFIG_SENSORARRAY_DEBUG_FDC_SAMPLE_COUNT
#define CONFIG_SENSORARRAY_DEBUG_FDC_SAMPLE_COUNT 1
#endif
#ifndef CONFIG_SENSORARRAY_ADS_READ_STOP1_BEFORE_MUX
#define CONFIG_SENSORARRAY_ADS_READ_STOP1_BEFORE_MUX 0
#endif
#ifndef CONFIG_SENSORARRAY_ADS_READ_SETTLE_AFTER_MUX_MS
#define CONFIG_SENSORARRAY_ADS_READ_SETTLE_AFTER_MUX_MS 0
#endif
#ifndef CONFIG_SENSORARRAY_ADS_READ_START1_EVERY_READ
#define CONFIG_SENSORARRAY_ADS_READ_START1_EVERY_READ 1
#endif
#ifndef CONFIG_SENSORARRAY_ADS_READ_BASE_DISCARD_COUNT
#define CONFIG_SENSORARRAY_ADS_READ_BASE_DISCARD_COUNT 0
#endif
#ifndef CONFIG_SENSORARRAY_ADS_READ_RETRY_COUNT
#define CONFIG_SENSORARRAY_ADS_READ_RETRY_COUNT 0
#endif

#define SENSORARRAY_RESIST_REF_OHMS 10000u
#define SENSORARRAY_RESIST_EXCITATION_UV 2500000u

#define SENSORARRAY_REF_SETTLE_MS 120u
#define SENSORARRAY_SETTLE_AFTER_COLUMN_MS 2u
#define SENSORARRAY_SETTLE_AFTER_PATH_MS 2u
#define SENSORARRAY_SETTLE_AFTER_SW_MS 2u

#define SENSORARRAY_ADS_REG_ID 0x00u
#define SENSORARRAY_ADS_REG_POWER 0x01u
#define SENSORARRAY_ADS_REG_INTERFACE 0x02u
#define SENSORARRAY_ADS_REG_MODE2 0x05u
#define SENSORARRAY_ADS_REG_INPMUX 0x06u
#define SENSORARRAY_ADS_REG_REFMUX 0x0Fu

#define SENSORARRAY_FDC_EXPECTED_MANUFACTURER_ID 0x5449u
#define SENSORARRAY_FDC_EXPECTED_DEVICE_ID 0x3055u
#define SENSORARRAY_FDC_REQUIRED_CHANNELS 4u
#define SENSORARRAY_FDC_I2C_ADDR_LOW 0x2Au
#define SENSORARRAY_FDC_I2C_ADDR_HIGH 0x2Bu
#define SENSORARRAY_FDC_REG_MANUFACTURER_ID 0x7Eu

#define SENSORARRAY_ADS_MUX_AINCOM 0x0Au

#define SENSORARRAY_S1 1u
#define SENSORARRAY_S4 4u
#define SENSORARRAY_S5 5u
#define SENSORARRAY_S8 8u

#define SENSORARRAY_D1 1u
#define SENSORARRAY_D4 4u
#define SENSORARRAY_D5 5u
#define SENSORARRAY_D7 7u
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

typedef enum {
    SENSORARRAY_DEBUG_PATH_RESISTIVE = 0,
    SENSORARRAY_DEBUG_PATH_CAPACITIVE,
    SENSORARRAY_DEBUG_PATH_VOLTAGE,
} sensorarrayDebugPath_t;

typedef enum {
    SENSORARRAY_DEBUG_MODE_ROUTE_IDLE = 0,
    SENSORARRAY_DEBUG_MODE_ROUTE_FIXED_STATE,
    SENSORARRAY_DEBUG_MODE_ROUTE_STEP_ONCE,
    SENSORARRAY_DEBUG_MODE_ROUTE_SCAN_LOOP,
    SENSORARRAY_DEBUG_MODE_ADS_SELFTEST,
    SENSORARRAY_DEBUG_MODE_FDC_SELFTEST,
    SENSORARRAY_DEBUG_MODE_S1D1_RESISTOR,
} sensorarrayDebugMode_t;

typedef struct {
    uint8_t sColumn;
    uint8_t dLine;
    sensorarrayPath_t path;
    bool selALevel;
    bool selBLevel;
    const char *mapLabel;
} sensorarrayRouteMap_t;

typedef struct {
    bool stopBeforeMuxChange;
    uint32_t settleAfterMuxMs;
    bool startEveryRead;
    uint8_t baseDiscardCount;
    uint8_t readRetryCount;
} sensorarrayAdsReadPolicy_t;

typedef struct {
    uint8_t sColumn;
    uint8_t dLine;
    sensorarrayDebugPath_t path;
    tmux1108Source_t swSource;
    bool selALevel;
    bool selBLevel;
    bool skipAdsRead;
    bool skipFdcRead;
    uint32_t delayAfterRowMs;
    uint32_t delayAfterSelAMs;
    uint32_t delayAfterSelBMs;
    uint32_t delayAfterSwMs;
    bool holdForever;
    uint32_t holdMs;
    const char *label;
} sensorarrayDebugFixedRoute_t;

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
    bool adsAdc1Running;
    bool adsRefMuxValid;
    uint8_t adsRefMux;

    sensorarrayFdcDeviceState_t fdcPrimary;   // D1..D4 (C1..C4)
    sensorarrayFdcDeviceState_t fdcSecondary; // D5..D8 (C5..C8)
    uint8_t fdcConfiguredChannels;

    bool boardReady;
    bool tmuxReady;
} sensorarrayState_t;

static sensorarrayState_t s_state = {0};

typedef struct {
    bool muxValid;
    uint8_t muxp;
    uint8_t muxn;
    bool refmuxValid;
    uint8_t refmux;
    bool discardCountValid;
    uint8_t discardCount;
    bool ctrlValid;
    tmuxSwitchControlState_t ctrl;
} sensorarrayDbgExtra_t;

static sensorarrayDbgExtra_t s_dbgExtra = {0};

static const sensorarrayAdsReadPolicy_t s_adsReadPolicy = {
    .stopBeforeMuxChange = (CONFIG_SENSORARRAY_ADS_READ_STOP1_BEFORE_MUX != 0),
    .settleAfterMuxMs = (uint32_t)CONFIG_SENSORARRAY_ADS_READ_SETTLE_AFTER_MUX_MS,
    .startEveryRead = (CONFIG_SENSORARRAY_ADS_READ_START1_EVERY_READ != 0),
    .baseDiscardCount = (uint8_t)CONFIG_SENSORARRAY_ADS_READ_BASE_DISCARD_COUNT,
    .readRetryCount = (uint8_t)CONFIG_SENSORARRAY_ADS_READ_RETRY_COUNT,
};

#if CONFIG_SENSORARRAY_DEBUG_MODE_ROUTE_IDLE
#define SENSORARRAY_ACTIVE_DEBUG_MODE SENSORARRAY_DEBUG_MODE_ROUTE_IDLE
#elif CONFIG_SENSORARRAY_DEBUG_MODE_ROUTE_FIXED_STATE
#define SENSORARRAY_ACTIVE_DEBUG_MODE SENSORARRAY_DEBUG_MODE_ROUTE_FIXED_STATE
#elif CONFIG_SENSORARRAY_DEBUG_MODE_ROUTE_STEP_ONCE
#define SENSORARRAY_ACTIVE_DEBUG_MODE SENSORARRAY_DEBUG_MODE_ROUTE_STEP_ONCE
#elif CONFIG_SENSORARRAY_DEBUG_MODE_ADS_SELFTEST
#define SENSORARRAY_ACTIVE_DEBUG_MODE SENSORARRAY_DEBUG_MODE_ADS_SELFTEST
#elif CONFIG_SENSORARRAY_DEBUG_MODE_FDC_SELFTEST
#define SENSORARRAY_ACTIVE_DEBUG_MODE SENSORARRAY_DEBUG_MODE_FDC_SELFTEST
#elif CONFIG_SENSORARRAY_DEBUG_MODE_S1D1_RESISTOR
#define SENSORARRAY_ACTIVE_DEBUG_MODE SENSORARRAY_DEBUG_MODE_S1D1_RESISTOR
#else
#define SENSORARRAY_ACTIVE_DEBUG_MODE SENSORARRAY_DEBUG_MODE_ROUTE_SCAN_LOOP
#endif

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

static const char *sensorarrayFmtU8(char *buf, size_t bufSize, bool valid, uint8_t value)
{
    if (!valid) {
        return SENSORARRAY_NA;
    }
    snprintf(buf, bufSize, "%u", (unsigned)value);
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

static const char *sensorarrayFmtGpioLevel(char *buf, size_t bufSize, bool valid, int level)
{
    if (!valid || level < 0) {
        return SENSORARRAY_NA;
    }
    snprintf(buf, bufSize, "%d", level ? 1 : 0);
    return buf;
}

static const char *sensorarrayPathName(sensorarrayPath_t path)
{
    return (path == SENSORARRAY_PATH_CAPACITIVE) ? "cap" : "res";
}

static const char *sensorarrayDebugPathName(sensorarrayDebugPath_t path)
{
    switch (path) {
    case SENSORARRAY_DEBUG_PATH_CAPACITIVE:
        return "cap";
    case SENSORARRAY_DEBUG_PATH_VOLTAGE:
        return "volt";
    case SENSORARRAY_DEBUG_PATH_RESISTIVE:
    default:
        return "res";
    }
}

static const char *sensorarraySwSourceName(tmux1108Source_t source)
{
    return (source == TMUX1108_SOURCE_REF) ? "HIGH" : "LOW";
}

static const char *sensorarrayDebugModeName(sensorarrayDebugMode_t mode)
{
    switch (mode) {
    case SENSORARRAY_DEBUG_MODE_ROUTE_IDLE:
        return "ROUTE_IDLE";
    case SENSORARRAY_DEBUG_MODE_ROUTE_FIXED_STATE:
        return "ROUTE_FIXED_STATE";
    case SENSORARRAY_DEBUG_MODE_ROUTE_STEP_ONCE:
        return "ROUTE_STEP_ONCE";
    case SENSORARRAY_DEBUG_MODE_ROUTE_SCAN_LOOP:
        return "ROUTE_SCAN_LOOP";
    case SENSORARRAY_DEBUG_MODE_ADS_SELFTEST:
        return "ADS_SELFTEST";
    case SENSORARRAY_DEBUG_MODE_FDC_SELFTEST:
        return "FDC_SELFTEST";
    case SENSORARRAY_DEBUG_MODE_S1D1_RESISTOR:
        return "S1D1_RESISTOR_DEBUG";
    default:
        return "UNKNOWN";
    }
}

static void sensorarrayDbgExtraReset(void)
{
    memset(&s_dbgExtra, 0, sizeof(s_dbgExtra));
}

static void sensorarrayDbgExtraSetMux(uint8_t muxp, uint8_t muxn)
{
    s_dbgExtra.muxValid = true;
    s_dbgExtra.muxp = muxp;
    s_dbgExtra.muxn = muxn;
}

static void sensorarrayDbgExtraSetRefMux(uint8_t refmux)
{
    s_dbgExtra.refmuxValid = true;
    s_dbgExtra.refmux = refmux;
}

static void sensorarrayDbgExtraSetDiscardCount(uint8_t discardCount)
{
    s_dbgExtra.discardCountValid = true;
    s_dbgExtra.discardCount = discardCount;
}

static void sensorarrayDbgExtraCaptureCtrl(void)
{
    tmuxSwitchControlState_t ctrl = {0};
    if (tmuxSwitchGetControlState(&ctrl) == ESP_OK) {
        s_dbgExtra.ctrlValid = true;
        s_dbgExtra.ctrl = ctrl;
    } else {
        s_dbgExtra.ctrlValid = false;
    }
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
    char muxpBuf[12];
    char muxnBuf[12];
    char refmuxBuf[12];
    char discardBuf[12];
    char ctrlA0Buf[8];
    char ctrlA1Buf[8];
    char ctrlA2Buf[8];
    char ctrlSwBuf[8];
    char ctrlSel1Buf[8];
    char ctrlSel2Buf[8];
    char ctrlSel3Buf[8];
    char ctrlSel4Buf[8];
    char ctrlEnBuf[8];
    const tmuxSwitchControlState_t *ctrl = s_dbgExtra.ctrlValid ? &s_dbgExtra.ctrl : NULL;

    printf("DBG,point=%s,kind=%s,column=%s,dline=%s,sw=%s,ref=%s,mode=%s,value=%s,valueUv=%s,"
           "valueMohm=%s,raw=%s,wd=%s,amp=%s,fdcDev=%s,i2cPort=%s,i2cAddr=%s,idMfg=%s,idDev=%s,"
           "refReady=%s,map=%s,err=%ld,status=%s,muxp=%s,muxn=%s,refmux=%s,discardCount=%s,"
           "ctrlA0=%s,ctrlA1=%s,ctrlA2=%s,ctrlSW=%s,ctrlSel1=%s,ctrlSel2=%s,ctrlSel3=%s,"
           "ctrlSel4=%s,ctrlEn=%s\n",
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
           status ? status : SENSORARRAY_NA,
           sensorarrayFmtU8(muxpBuf, sizeof(muxpBuf), s_dbgExtra.muxValid, s_dbgExtra.muxp),
           sensorarrayFmtU8(muxnBuf, sizeof(muxnBuf), s_dbgExtra.muxValid, s_dbgExtra.muxn),
           sensorarrayFmtHexU8(refmuxBuf, sizeof(refmuxBuf), s_dbgExtra.refmuxValid, s_dbgExtra.refmux),
           sensorarrayFmtU8(discardBuf, sizeof(discardBuf), s_dbgExtra.discardCountValid, s_dbgExtra.discardCount),
           sensorarrayFmtGpioLevel(ctrlA0Buf, sizeof(ctrlA0Buf), ctrl != NULL, ctrl ? ctrl->a0Level : -1),
           sensorarrayFmtGpioLevel(ctrlA1Buf, sizeof(ctrlA1Buf), ctrl != NULL, ctrl ? ctrl->a1Level : -1),
           sensorarrayFmtGpioLevel(ctrlA2Buf, sizeof(ctrlA2Buf), ctrl != NULL, ctrl ? ctrl->a2Level : -1),
           sensorarrayFmtGpioLevel(ctrlSwBuf, sizeof(ctrlSwBuf), ctrl != NULL, ctrl ? ctrl->swLevel : -1),
           sensorarrayFmtGpioLevel(ctrlSel1Buf, sizeof(ctrlSel1Buf), ctrl != NULL, ctrl ? ctrl->sel1Level : -1),
           sensorarrayFmtGpioLevel(ctrlSel2Buf, sizeof(ctrlSel2Buf), ctrl != NULL, ctrl ? ctrl->sel2Level : -1),
           sensorarrayFmtGpioLevel(ctrlSel3Buf, sizeof(ctrlSel3Buf), ctrl != NULL, ctrl ? ctrl->sel3Level : -1),
           sensorarrayFmtGpioLevel(ctrlSel4Buf, sizeof(ctrlSel4Buf), ctrl != NULL, ctrl ? ctrl->sel4Level : -1),
           sensorarrayFmtGpioLevel(ctrlEnBuf, sizeof(ctrlEnBuf), ctrl != NULL, ctrl ? ctrl->enLevel : -1));
    sensorarrayDbgExtraReset();
}

static int sensorarrayReadOptionalGpioLevel(int gpio)
{
    if (gpio < 0) {
        return -1;
    }
    return gpio_get_level((gpio_num_t)gpio);
}

static void sensorarrayLogControlGpio(const char *stage, const char *point)
{
    char a0Buf[8];
    char a1Buf[8];
    char a2Buf[8];
    char swBuf[8];
    char sel1Buf[8];
    char sel2Buf[8];
    char sel3Buf[8];
    char sel4Buf[8];
    char enBuf[8];
    char drdyBuf[8];
    char resetBuf[8];

    tmuxSwitchControlState_t ctrl = {0};
    bool haveCtrl = (tmuxSwitchGetControlState(&ctrl) == ESP_OK);
    int drdyLevel = sensorarrayReadOptionalGpioLevel(CONFIG_BOARD_ADS126X_DRDY_GPIO);
    int resetLevel = sensorarrayReadOptionalGpioLevel(CONFIG_BOARD_ADS126X_RESET_GPIO);

    printf("DBGCTRL,stage=%s,point=%s,a0=%s,a1=%s,a2=%s,sw=%s,sel1=%s,sel2=%s,sel3=%s,sel4=%s,en=%s,"
           "drdy=%s,adsReset=%s\n",
           stage ? stage : SENSORARRAY_NA,
           point ? point : SENSORARRAY_NA,
           sensorarrayFmtGpioLevel(a0Buf, sizeof(a0Buf), haveCtrl, haveCtrl ? ctrl.a0Level : -1),
           sensorarrayFmtGpioLevel(a1Buf, sizeof(a1Buf), haveCtrl, haveCtrl ? ctrl.a1Level : -1),
           sensorarrayFmtGpioLevel(a2Buf, sizeof(a2Buf), haveCtrl, haveCtrl ? ctrl.a2Level : -1),
           sensorarrayFmtGpioLevel(swBuf, sizeof(swBuf), haveCtrl, haveCtrl ? ctrl.swLevel : -1),
           sensorarrayFmtGpioLevel(sel1Buf, sizeof(sel1Buf), haveCtrl, haveCtrl ? ctrl.sel1Level : -1),
           sensorarrayFmtGpioLevel(sel2Buf, sizeof(sel2Buf), haveCtrl, haveCtrl ? ctrl.sel2Level : -1),
           sensorarrayFmtGpioLevel(sel3Buf, sizeof(sel3Buf), haveCtrl, haveCtrl ? ctrl.sel3Level : -1),
           sensorarrayFmtGpioLevel(sel4Buf, sizeof(sel4Buf), haveCtrl, haveCtrl ? ctrl.sel4Level : -1),
           sensorarrayFmtGpioLevel(enBuf, sizeof(enBuf), haveCtrl, haveCtrl ? ctrl.enLevel : -1),
           sensorarrayFmtGpioLevel(drdyBuf, sizeof(drdyBuf), true, drdyLevel),
           sensorarrayFmtGpioLevel(resetBuf, sizeof(resetBuf), true, resetLevel));
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

static esp_err_t sensorarrayAdsSetRefMux(ads126xAdcHandle_t *handle, uint8_t refmuxValue)
{
    esp_err_t err = ads126xAdcSetRefMux(handle, refmuxValue);
    if (err == ESP_OK) {
        s_state.adsRefMux = refmuxValue;
        s_state.adsRefMuxValid = true;
        sensorarrayDbgExtraSetRefMux(refmuxValue);
    }
    return err;
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
    s_state.adsAdc1Running = true;
    int32_t raw = 0;
    return ads126xAdcReadAdc1Raw(handle, &raw, NULL);
}

static esp_err_t sensorarrayPrepareAdsRefPath(ads126xAdcHandle_t *handle)
{
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }

    /*
     * SW high state drives the ADS REF-derived path in current hardware.
     * Keep REF preparation explicit before entering the runtime loop.
     */
    esp_err_t err = ads126xAdcConfigure(handle, true, false, ADS126X_CRC_OFF, 1, 0);
    if (err != ESP_OK) {
        return err;
    }

    err = sensorarrayAdsSetRefMux(handle, 0x00);
    if (err != ESP_OK) {
        return err;
    }

    sensorarrayDelayMs(SENSORARRAY_REF_SETTLE_MS);

    err = ads126xAdcStartAdc1(handle);
    if (err != ESP_OK) {
        return err;
    }
    s_state.adsAdc1Running = true;

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
    { SENSORARRAY_S4, SENSORARRAY_D4, SENSORARRAY_PATH_RESISTIVE, false, false, "S4D4_res_selA0_selB0" },
    { SENSORARRAY_S5, SENSORARRAY_D5, SENSORARRAY_PATH_CAPACITIVE, false, false, "S5D5_cap_selA0_selB0" },
    { SENSORARRAY_S8, SENSORARRAY_D7, SENSORARRAY_PATH_CAPACITIVE, false, false, "S8D7_cap_selA0_selB0" },
    { SENSORARRAY_S8, SENSORARRAY_D7, SENSORARRAY_PATH_RESISTIVE, false, true, "S8D7_volt_selA0_selB1" },
    { SENSORARRAY_S8, SENSORARRAY_D8, SENSORARRAY_PATH_CAPACITIVE, false, true, "S8D8_cap_selA0_selB1" },
    { SENSORARRAY_S8, SENSORARRAY_D8, SENSORARRAY_PATH_RESISTIVE, false, false, "S8D8_volt_selA0_selB0" },
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

static void sensorarrayAuditRouteMap(void)
{
    for (size_t i = 0; i < (sizeof(s_sensorarrayRouteMap) / sizeof(s_sensorarrayRouteMap[0])); ++i) {
        const sensorarrayRouteMap_t *entry = &s_sensorarrayRouteMap[i];
        printf("DBGROUTEMAP,index=%u,sColumn=%u,dLine=%u,path=%s,selALevel=%u,selBLevel=%u,label=%s\n",
               (unsigned)i,
               (unsigned)entry->sColumn,
               (unsigned)entry->dLine,
               sensorarrayPathName(entry->path),
               entry->selALevel ? 1u : 0u,
               entry->selBLevel ? 1u : 0u,
               entry->mapLabel ? entry->mapLabel : SENSORARRAY_NA);
    }
}

static void sensorarrayAuditFdcDLineMap(void)
{
    for (size_t i = 0; i < (sizeof(s_sensorarrayFdcDLineMap) / sizeof(s_sensorarrayFdcDLineMap[0])); ++i) {
        const sensorarrayFdcDLineMap_t *entry = &s_sensorarrayFdcDLineMap[i];
        const char *devLabel =
            (entry->devId == SENSORARRAY_FDC_DEV_PRIMARY) ? "primary" : "secondary";
        printf("DBGFDCMAP,index=%u,dLine=%u,fdcDev=%s,channel=%u,label=%s\n",
               (unsigned)i,
               (unsigned)entry->dLine,
               devLabel,
               (unsigned)entry->channel,
               entry->mapLabel ? entry->mapLabel : SENSORARRAY_NA);
    }
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

static void sensorarrayLogRouteStep(const char *stage,
                                    const char *label,
                                    uint8_t sColumn,
                                    uint8_t dLine,
                                    sensorarrayDebugPath_t path,
                                    tmux1108Source_t swSource,
                                    bool selALevel,
                                    bool selBLevel,
                                    esp_err_t err,
                                    const char *status)
{
    printf("DBGROUTE,stage=%s,label=%s,sColumn=%u,dLine=%u,path=%s,sw=%s,selALevel=%u,selBLevel=%u,err=%ld,"
           "status=%s\n",
           stage ? stage : SENSORARRAY_NA,
           label ? label : SENSORARRAY_NA,
           (unsigned)sColumn,
           (unsigned)dLine,
           sensorarrayDebugPathName(path),
           sensorarraySwSourceName(swSource),
           selALevel ? 1u : 0u,
           selBLevel ? 1u : 0u,
           (long)err,
           status ? status : SENSORARRAY_NA);
    sensorarrayLogControlGpio(stage, label);
}

static sensorarrayDebugPath_t sensorarrayPathToDebugPath(sensorarrayPath_t path, tmux1108Source_t swSource)
{
    if (path == SENSORARRAY_PATH_CAPACITIVE) {
        return SENSORARRAY_DEBUG_PATH_CAPACITIVE;
    }
    if (swSource == TMUX1108_SOURCE_REF) {
        return SENSORARRAY_DEBUG_PATH_VOLTAGE;
    }
    return SENSORARRAY_DEBUG_PATH_RESISTIVE;
}

static esp_err_t sensorarrayApplyRouteLevels(uint8_t sColumn,
                                             uint8_t dLine,
                                             sensorarrayDebugPath_t path,
                                             tmux1108Source_t swSource,
                                             bool selALevel,
                                             bool selBLevel,
                                             uint32_t delayAfterRowMs,
                                             uint32_t delayAfterSelAMs,
                                             uint32_t delayAfterSelBMs,
                                             uint32_t delayAfterSwMs,
                                             const char *label)
{
    if (!s_state.tmuxReady) {
        return ESP_ERR_INVALID_STATE;
    }
    if (sColumn < 1u || sColumn > 8u || dLine < 1u || dLine > 8u) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = tmuxSwitchSelectRow((uint8_t)(sColumn - 1u));
    sensorarrayLogRouteStep("row", label, sColumn, dLine, path, swSource, selALevel, selBLevel, err, "set_row");
    if (err != ESP_OK) {
        return err;
    }
    sensorarrayDelayMs(delayAfterRowMs);

    err = tmux1134SelectSelALevel(selALevel);
    sensorarrayLogRouteStep("selA", label, sColumn, dLine, path, swSource, selALevel, selBLevel, err, "set_selA");
    if (err != ESP_OK) {
        return err;
    }
    sensorarrayDelayMs(delayAfterSelAMs);

    err = tmux1134SelectSelBLevel(selBLevel);
    sensorarrayLogRouteStep("selB", label, sColumn, dLine, path, swSource, selALevel, selBLevel, err, "set_selB");
    if (err != ESP_OK) {
        return err;
    }
    sensorarrayDelayMs(delayAfterSelBMs);

    err = tmuxSwitchSet1108Source(swSource);
    sensorarrayLogRouteStep("sw", label, sColumn, dLine, path, swSource, selALevel, selBLevel, err, "set_sw");
    if (err != ESP_OK) {
        return err;
    }
    sensorarrayDelayMs(delayAfterSwMs);

    sensorarrayDbgExtraCaptureCtrl();
    return ESP_OK;
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

    esp_err_t err = sensorarrayApplyRouteLevels(sColumn,
                                                dLine,
                                                sensorarrayPathToDebugPath(path, swSource),
                                                swSource,
                                                routeMap->selALevel,
                                                routeMap->selBLevel,
                                                SENSORARRAY_SETTLE_AFTER_COLUMN_MS,
                                                0u,
                                                SENSORARRAY_SETTLE_AFTER_PATH_MS,
                                                SENSORARRAY_SETTLE_AFTER_SW_MS,
                                                routeMap->mapLabel);
    if (err != ESP_OK) {
        return err;
    }

    if (outMapLabel) {
        *outMapLabel = routeMap->mapLabel;
    }
    return ESP_OK;
}

static esp_err_t sensorarrayAdsReadRawWithRetry(int32_t *outRaw, uint8_t retryCount, bool *outTimedOut)
{
    if (!outRaw) {
        return ESP_ERR_INVALID_ARG;
    }
    if (outTimedOut) {
        *outTimedOut = false;
    }

    for (uint8_t attempt = 0; attempt <= retryCount; ++attempt) {
        esp_err_t err = ads126xAdcReadAdc1Raw(&s_state.ads, outRaw, NULL);
        if (err == ESP_OK) {
            return ESP_OK;
        }

        if (err == ESP_ERR_TIMEOUT && outTimedOut) {
            *outTimedOut = true;
        }

        printf("DBGADSRETRY,attempt=%u,maxRetry=%u,err=%ld\n",
               (unsigned)attempt,
               (unsigned)retryCount,
               (long)err);

        if (attempt == retryCount) {
            return err;
        }
        sensorarrayDelayMs(1u);
    }

    return ESP_FAIL;
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

    sensorarrayDbgExtraSetMux(muxp, muxn);
    if (s_state.adsRefMuxValid) {
        sensorarrayDbgExtraSetRefMux(s_state.adsRefMux);
    }

    uint16_t totalDiscard = (uint16_t)s_adsReadPolicy.baseDiscardCount + (discardFirst ? 1u : 0u);
    if (totalDiscard > 255u) {
        totalDiscard = 255u;
    }
    sensorarrayDbgExtraSetDiscardCount((uint8_t)totalDiscard);

    printf("DBGADSSEQ,dLine=%u,step=set_input_mux,muxp=%u,muxn=%u,discardFirst=%u,discardCount=%u\n",
           (unsigned)dLine,
           (unsigned)muxp,
           (unsigned)muxn,
           discardFirst ? 1u : 0u,
           (unsigned)totalDiscard);

    if (s_adsReadPolicy.stopBeforeMuxChange && s_state.adsAdc1Running) {
        esp_err_t stopErr = ads126xAdcStopAdc1(&s_state.ads);
        if (stopErr == ESP_OK) {
            s_state.adsAdc1Running = false;
        } else {
            printf("DBGADSSEQ,dLine=%u,step=stop1,err=%ld,status=stop_error\n", (unsigned)dLine, (long)stopErr);
            return stopErr;
        }
    }

    esp_err_t err = ads126xAdcSetInputMux(&s_state.ads, muxp, muxn);
    if (err != ESP_OK) {
        printf("DBGADSSEQ,dLine=%u,step=set_input_mux,err=%ld,status=set_mux_error\n", (unsigned)dLine, (long)err);
        return err;
    }

    if (s_adsReadPolicy.settleAfterMuxMs > 0u) {
        sensorarrayDelayMs(s_adsReadPolicy.settleAfterMuxMs);
    }

    if (s_adsReadPolicy.startEveryRead || !s_state.adsAdc1Running) {
        printf("DBGADSSEQ,dLine=%u,step=start1,muxp=%u,muxn=%u\n", (unsigned)dLine, (unsigned)muxp, (unsigned)muxn);
        err = ads126xAdcStartAdc1(&s_state.ads);
        if (err != ESP_OK) {
            printf("DBGADSSEQ,dLine=%u,step=start1,err=%ld,status=start_error\n", (unsigned)dLine, (long)err);
            return err;
        }
        s_state.adsAdc1Running = true;
    }

    for (uint16_t discardIdx = 0; discardIdx < totalDiscard; ++discardIdx) {
        int32_t throwaway = 0;
        printf("DBGADSSEQ,dLine=%u,step=discard,index=%u,muxp=%u,muxn=%u\n",
               (unsigned)dLine,
               (unsigned)discardIdx,
               (unsigned)muxp,
               (unsigned)muxn);
        err = sensorarrayAdsReadRawWithRetry(&throwaway, s_adsReadPolicy.readRetryCount, NULL);
        if (err != ESP_OK) {
            printf("DBGADSSEQ,dLine=%u,step=discard,index=%u,err=%ld,status=discard_error\n",
                   (unsigned)dLine,
                   (unsigned)discardIdx,
                   (long)err);
            return err;
        }
    }

    printf("DBGADSSEQ,dLine=%u,step=read,muxp=%u,muxn=%u,discardFirst=%u,discardCount=%u\n",
           (unsigned)dLine,
           (unsigned)muxp,
           (unsigned)muxn,
           discardFirst ? 1u : 0u,
           (unsigned)totalDiscard);

    bool readTimedOut = false;
    err = sensorarrayAdsReadRawWithRetry(outRaw, s_adsReadPolicy.readRetryCount, &readTimedOut);
    if (err != ESP_OK) {
        printf("DBGADSREAD,status=error,dLine=%u,muxp=%u,muxn=%u,refmux=0x%02X,discardCount=%u,drdyTimeout=%u,err=%ld\n",
               (unsigned)dLine,
               (unsigned)muxp,
               (unsigned)muxn,
               s_state.adsRefMuxValid ? s_state.adsRefMux : 0u,
               (unsigned)totalDiscard,
               readTimedOut ? 1u : 0u,
               (long)err);
        return err;
    }

    *outUv = ads126xAdcRawToMicrovolts(&s_state.ads, *outRaw);
    printf("DBGADSSEQ,dLine=%u,step=read_done,muxp=%u,muxn=%u,raw=%ld,uv=%ld,discardFirst=%u,discardCount=%u\n",
           (unsigned)dLine,
           (unsigned)muxp,
           (unsigned)muxn,
           (long)*outRaw,
           (long)*outUv,
           discardFirst ? 1u : 0u,
           (unsigned)totalDiscard);
    sensorarrayDbgExtraCaptureCtrl();
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

static const char *sensorarrayDebugPathKind(sensorarrayDebugPath_t path)
{
    switch (path) {
    case SENSORARRAY_DEBUG_PATH_CAPACITIVE:
        return "cap";
    case SENSORARRAY_DEBUG_PATH_VOLTAGE:
        return "volt";
    case SENSORARRAY_DEBUG_PATH_RESISTIVE:
    default:
        return "res";
    }
}

static esp_err_t sensorarrayDebugApplyFixedRoute(const sensorarrayDebugFixedRoute_t *cfg)
{
    if (!cfg) {
        return ESP_ERR_INVALID_ARG;
    }

    char columnBuf[6];
    char dLineBuf[6];
    char valueBuf[24];
    char uvBuf[24];
    char rawBuf[24];
    char wdBuf[8];
    char ampBuf[8];
    char mapBuf[96];

    snprintf(columnBuf, sizeof(columnBuf), "S%u", (unsigned)cfg->sColumn);
    snprintf(dLineBuf, sizeof(dLineBuf), "D%u", (unsigned)cfg->dLine);

    sensorarrayLogRouteStep("begin",
                            cfg->label,
                            cfg->sColumn,
                            cfg->dLine,
                            cfg->path,
                            cfg->swSource,
                            cfg->selALevel,
                            cfg->selBLevel,
                            ESP_OK,
                            "apply_fixed_route");

    esp_err_t err = sensorarrayApplyRouteLevels(cfg->sColumn,
                                                cfg->dLine,
                                                cfg->path,
                                                cfg->swSource,
                                                cfg->selALevel,
                                                cfg->selBLevel,
                                                cfg->delayAfterRowMs,
                                                cfg->delayAfterSelAMs,
                                                cfg->delayAfterSelBMs,
                                                cfg->delayAfterSwMs,
                                                cfg->label);
    if (err != ESP_OK) {
        return err;
    }

    const char *kind = sensorarrayDebugPathKind(cfg->path);
    if (!cfg->skipAdsRead) {
        int32_t raw = 0;
        int32_t uv = 0;
        esp_err_t readErr = sensorarrayReadAdsUv(cfg->dLine, false, &raw, &uv);
        sensorarrayLogDbg(cfg->label ? cfg->label : "FIXED",
                          kind,
                          columnBuf,
                          dLineBuf,
                          sensorarraySwSourceName(cfg->swSource),
                          "ads_fixed",
                          sensorarrayFmtI32(valueBuf, sizeof(valueBuf), readErr == ESP_OK, uv),
                          sensorarrayFmtI32(uvBuf, sizeof(uvBuf), readErr == ESP_OK, uv),
                          SENSORARRAY_NA,
                          sensorarrayFmtI32(rawBuf, sizeof(rawBuf), readErr == ESP_OK, raw),
                          SENSORARRAY_NA,
                          SENSORARRAY_NA,
                          SENSORARRAY_NA,
                          SENSORARRAY_NA,
                          SENSORARRAY_NA,
                          SENSORARRAY_NA,
                          SENSORARRAY_NA,
                          sensorarrayBuildMapLabel(mapBuf, sizeof(mapBuf), cfg->label, SENSORARRAY_NA),
                          readErr,
                          (readErr == ESP_OK) ? "ads_read_ok" : "ads_read_error");
    }

    if (!cfg->skipFdcRead) {
        const sensorarrayFdcDLineMap_t *fdcMap = NULL;
        sensorarrayFdcDeviceState_t *fdcState = sensorarrayGetFdcStateForDLine(cfg->dLine, &fdcMap);
        Fdc2214CapSample_t sample = {0};
        esp_err_t readErr = ESP_ERR_INVALID_STATE;
        if (fdcState && fdcState->ready && fdcState->handle && fdcMap) {
            readErr = sensorarrayReadFdcSample(fdcState->handle, fdcMap->channel, false, &sample);
        } else if (!fdcMap || !fdcState) {
            readErr = ESP_ERR_INVALID_ARG;
        }

        sensorarrayDbgExtraCaptureCtrl();
        sensorarrayLogDbg(cfg->label ? cfg->label : "FIXED",
                          kind,
                          columnBuf,
                          dLineBuf,
                          sensorarraySwSourceName(cfg->swSource),
                          "fdc_fixed",
                          sensorarrayFmtU32(valueBuf, sizeof(valueBuf), readErr == ESP_OK, sample.Raw28),
                          SENSORARRAY_NA,
                          SENSORARRAY_NA,
                          sensorarrayFmtU32(rawBuf, sizeof(rawBuf), readErr == ESP_OK, sample.Raw28),
                          sensorarrayFmtBool(wdBuf, sizeof(wdBuf), readErr == ESP_OK, sample.ErrWatchdog),
                          sensorarrayFmtBool(ampBuf, sizeof(ampBuf), readErr == ESP_OK, sample.ErrAmplitude),
                          fdcState ? fdcState->label : SENSORARRAY_NA,
                          SENSORARRAY_NA,
                          SENSORARRAY_NA,
                          SENSORARRAY_NA,
                          SENSORARRAY_NA,
                          sensorarrayBuildMapLabel(mapBuf,
                                                   sizeof(mapBuf),
                                                   cfg->label,
                                                   fdcMap ? fdcMap->mapLabel : SENSORARRAY_NA),
                          readErr,
                          (readErr == ESP_OK) ? "fdc_read_ok" : "fdc_read_error");
    }

    if (cfg->holdForever) {
        sensorarrayLogRouteStep("hold_forever",
                                cfg->label,
                                cfg->sColumn,
                                cfg->dLine,
                                cfg->path,
                                cfg->swSource,
                                cfg->selALevel,
                                cfg->selBLevel,
                                ESP_OK,
                                "fixed_state_latched");
        while (true) {
            sensorarrayDelayMs(1000u);
        }
    }

    if (cfg->holdMs > 0u) {
        sensorarrayLogRouteStep("hold_ms",
                                cfg->label,
                                cfg->sColumn,
                                cfg->dLine,
                                cfg->path,
                                cfg->swSource,
                                cfg->selALevel,
                                cfg->selBLevel,
                                ESP_OK,
                                "holding_final_state");
        sensorarrayDelayMs(cfg->holdMs);
    }

    return ESP_OK;
}

static void sensorarrayDebugReadResistor(const char *pointLabel, uint8_t sColumn, uint8_t dLine)
{
    char valueBuf[24];
    char uvBuf[24];
    char mohmBuf[24];
    char rawBuf[24];
    char mapBuf[72];
    char columnBuf[6];
    char dLineBuf[6];

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

    snprintf(columnBuf, sizeof(columnBuf), "S%u", (unsigned)sColumn);
    snprintf(dLineBuf, sizeof(dLineBuf), "D%u", (unsigned)dLine);

    sensorarrayLogDbg(pointLabel,
                      "res",
                      columnBuf,
                      dLineBuf,
                      "LOW",
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
                                    TMUX1108_SOURCE_REF,
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
                      "HIGH",
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
                                    TMUX1108_SOURCE_REF,
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
                      "HIGH",
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

static tmux1108Source_t sensorarrayRouteMapDefaultSwSource(const sensorarrayRouteMap_t *route)
{
    if (!route) {
        return TMUX1108_SOURCE_GND;
    }
    if (route->path == SENSORARRAY_PATH_CAPACITIVE) {
        return TMUX1108_SOURCE_REF;
    }
    if (route->mapLabel && strstr(route->mapLabel, "volt") != NULL) {
        return TMUX1108_SOURCE_REF;
    }
    return TMUX1108_SOURCE_GND;
}

static sensorarrayDebugPath_t sensorarrayConfiguredFixedPath(void)
{
#if CONFIG_SENSORARRAY_DEBUG_FIXED_PATH_CAPACITIVE
    return SENSORARRAY_DEBUG_PATH_CAPACITIVE;
#elif CONFIG_SENSORARRAY_DEBUG_FIXED_PATH_VOLTAGE
    return SENSORARRAY_DEBUG_PATH_VOLTAGE;
#else
    return SENSORARRAY_DEBUG_PATH_RESISTIVE;
#endif
}

static tmux1108Source_t sensorarrayConfiguredFixedSwSource(void)
{
#if CONFIG_SENSORARRAY_DEBUG_FIXED_SW_SOURCE_REF
    return TMUX1108_SOURCE_REF;
#else
    return TMUX1108_SOURCE_GND;
#endif
}

static void sensorarrayIdleForever(const char *reason)
{
    sensorarrayLogControlGpio("idle", reason);
    while (true) {
        sensorarrayDelayMs(1000u);
    }
}

static esp_err_t sensorarrayAdsReadRegister(uint8_t reg, uint8_t *outValue)
{
    if (!outValue || !s_state.adsReady) {
        return ESP_ERR_INVALID_STATE;
    }
    return ads126xAdcReadRegisters(&s_state.ads, reg, outValue, 1);
}

static esp_err_t sensorarrayDumpAdsKeyRegisters(const char *stage)
{
    uint8_t id = 0;
    uint8_t power = 0;
    uint8_t iface = 0;
    uint8_t mode2 = 0;
    uint8_t inpmux = 0;
    uint8_t refmux = 0;

    esp_err_t err = sensorarrayAdsReadRegister(SENSORARRAY_ADS_REG_ID, &id);
    if (err == ESP_OK) {
        err = sensorarrayAdsReadRegister(SENSORARRAY_ADS_REG_POWER, &power);
    }
    if (err == ESP_OK) {
        err = sensorarrayAdsReadRegister(SENSORARRAY_ADS_REG_INTERFACE, &iface);
    }
    if (err == ESP_OK) {
        err = sensorarrayAdsReadRegister(SENSORARRAY_ADS_REG_MODE2, &mode2);
    }
    if (err == ESP_OK) {
        err = sensorarrayAdsReadRegister(SENSORARRAY_ADS_REG_INPMUX, &inpmux);
    }
    if (err == ESP_OK) {
        err = sensorarrayAdsReadRegister(SENSORARRAY_ADS_REG_REFMUX, &refmux);
    }

    if (err == ESP_OK) {
        s_state.adsRefMux = refmux;
        s_state.adsRefMuxValid = true;
    }

    printf("DBGADSREG,stage=%s,id=0x%02X,power=0x%02X,interface=0x%02X,mode2=0x%02X,inpmux=0x%02X,refmux=0x%02X,"
           "err=%ld,status=%s\n",
           stage ? stage : SENSORARRAY_NA,
           id,
           power,
           iface,
           mode2,
           inpmux,
           refmux,
           (long)err,
           (err == ESP_OK) ? "ok" : "read_error");
    return err;
}

static void sensorarrayRunAdsSelftestMode(void)
{
    if (!s_state.adsReady) {
        sensorarrayLogStartup("ads_selftest", ESP_ERR_INVALID_STATE, "skip_ads_unavailable", 0);
        sensorarrayIdleForever("ads_unavailable");
        return;
    }

    uint8_t muxp = (uint8_t)(CONFIG_SENSORARRAY_DEBUG_ADS_FORCE_MUXP & 0x0Fu);
    uint8_t muxn = (uint8_t)(CONFIG_SENSORARRAY_DEBUG_ADS_FORCE_MUXN & 0x0Fu);
    uint8_t refmux = (uint8_t)(CONFIG_SENSORARRAY_DEBUG_ADS_FORCE_REFMUX & 0xFFu);
    uint8_t retryCount = (uint8_t)CONFIG_SENSORARRAY_DEBUG_ADS_READ_RETRY_COUNT;
    uint8_t discardCount = (uint8_t)CONFIG_SENSORARRAY_DEBUG_ADS_DISCARD_COUNT;
    uint8_t sampleCount = (uint8_t)CONFIG_SENSORARRAY_DEBUG_ADS_SAMPLE_COUNT;
    bool drdyTimedOut = false;

    sensorarrayDumpAdsKeyRegisters("pre_cfg");

    esp_err_t err = ads126xAdcConfigure(&s_state.ads, true, false, ADS126X_CRC_OFF, 1, 0);
    sensorarrayLogStartup("ads_selftest_cfg", err, (err == ESP_OK) ? "ok" : "configure_error", 0);
    if (err != ESP_OK) {
        sensorarrayIdleForever("ads_cfg_fail");
        return;
    }

    sensorarrayDumpAdsKeyRegisters("post_cfg");

    if (CONFIG_SENSORARRAY_DEBUG_ADS_STOP1_BEFORE_MUX && s_state.adsAdc1Running) {
        err = ads126xAdcStopAdc1(&s_state.ads);
        sensorarrayLogStartup("ads_selftest_stop1", err, (err == ESP_OK) ? "ok" : "stop_error", 0);
        if (err != ESP_OK) {
            sensorarrayIdleForever("ads_stop_fail");
            return;
        }
        s_state.adsAdc1Running = false;
    }

    err = sensorarrayAdsSetRefMux(&s_state.ads, refmux);
    sensorarrayLogStartup("ads_selftest_refmux", err, (err == ESP_OK) ? "ok" : "refmux_error", refmux);
    if (err != ESP_OK) {
        sensorarrayIdleForever("ads_refmux_fail");
        return;
    }

    err = ads126xAdcSetInputMux(&s_state.ads, muxp, muxn);
    sensorarrayLogStartup("ads_selftest_inpmux",
                          err,
                          (err == ESP_OK) ? "ok" : "inpmux_error",
                          ((int32_t)muxp << 8) | (int32_t)muxn);
    if (err != ESP_OK) {
        sensorarrayIdleForever("ads_inpmux_fail");
        return;
    }

    sensorarrayDelayMs((uint32_t)CONFIG_SENSORARRAY_DEBUG_ADS_SETTLE_AFTER_MUX_MS);

    if (CONFIG_SENSORARRAY_DEBUG_ADS_START1_BEFORE_READ || !s_state.adsAdc1Running) {
        err = ads126xAdcStartAdc1(&s_state.ads);
        sensorarrayLogStartup("ads_selftest_start1", err, (err == ESP_OK) ? "ok" : "start_error", 0);
        if (err != ESP_OK) {
            sensorarrayIdleForever("ads_start_fail");
            return;
        }
        s_state.adsAdc1Running = true;
    }

    for (uint8_t i = 0; i < discardCount; ++i) {
        int32_t throwaway = 0;
        err = sensorarrayAdsReadRawWithRetry(&throwaway, retryCount, &drdyTimedOut);
        printf("DBGADSSELF,stage=discard,index=%u,raw=%ld,muxp=%u,muxn=%u,refmux=0x%02X,discardCount=%u,"
               "discardFirst=%u,drdyTimeout=%u,err=%ld,status=%s\n",
               (unsigned)i,
               (long)throwaway,
               (unsigned)muxp,
               (unsigned)muxn,
               refmux,
               (unsigned)discardCount,
               (discardCount > 0u) ? 1u : 0u,
               drdyTimedOut ? 1u : 0u,
               (long)err,
               (err == ESP_OK) ? "discard_ok" : "discard_error");
        if (err != ESP_OK) {
            sensorarrayIdleForever("ads_discard_fail");
            return;
        }
    }

    for (uint8_t i = 0; i < sampleCount; ++i) {
        int32_t raw = 0;
        drdyTimedOut = false;
        err = sensorarrayAdsReadRawWithRetry(&raw, retryCount, &drdyTimedOut);
        int32_t uv = (err == ESP_OK) ? ads126xAdcRawToMicrovolts(&s_state.ads, raw) : 0;
        printf("DBGADSSELF,stage=sample,index=%u,raw=%ld,uv=%ld,muxp=%u,muxn=%u,refmux=0x%02X,discardCount=%u,"
               "discardFirst=%u,drdyTimeout=%u,err=%ld,status=%s\n",
               (unsigned)i,
               (long)raw,
               (long)uv,
               (unsigned)muxp,
               (unsigned)muxn,
               refmux,
               (unsigned)discardCount,
               (discardCount > 0u) ? 1u : 0u,
               drdyTimedOut ? 1u : 0u,
               (long)err,
               (err == ESP_OK) ? "sample_ok" : "sample_error");
    }

    sensorarrayDumpAdsKeyRegisters("post_read");
    sensorarrayIdleForever("ads_selftest_done");
}

static void sensorarrayRunFdcSelftestMode(void)
{
    uint8_t dLine = (uint8_t)CONFIG_SENSORARRAY_DEBUG_FDC_D_LINE;
    const sensorarrayFdcDLineMap_t *fdcMap = NULL;
    sensorarrayFdcDeviceState_t *fdcState = sensorarrayGetFdcStateForDLine(dLine, &fdcMap);

    if (!fdcMap || !fdcState || !fdcState->ready || !fdcState->handle) {
        sensorarrayLogStartup("fdc_selftest", ESP_ERR_INVALID_STATE, "fdc_not_ready", (int32_t)dLine);
        sensorarrayIdleForever("fdc_unavailable");
        return;
    }

    bool discardFirst = (CONFIG_SENSORARRAY_DEBUG_FDC_DISCARD_FIRST != 0);
    uint8_t sampleCount = (uint8_t)CONFIG_SENSORARRAY_DEBUG_FDC_SAMPLE_COUNT;
    if (sampleCount == 0u) {
        sampleCount = 1u;
    }

    for (uint8_t i = 0; i < sampleCount; ++i) {
        Fdc2214CapSample_t sample = {0};
        bool doDiscard = discardFirst && (i == 0u);
        esp_err_t err = sensorarrayReadFdcSample(fdcState->handle, fdcMap->channel, doDiscard, &sample);
        printf("DBGFDCSELF,index=%u,dLine=%u,fdcDev=%s,channel=%u,discardFirst=%u,raw=%lu,wd=%u,amp=%u,err=%ld,"
               "status=%s\n",
               (unsigned)i,
               (unsigned)dLine,
               fdcState->label ? fdcState->label : SENSORARRAY_NA,
               (unsigned)fdcMap->channel,
               doDiscard ? 1u : 0u,
               (unsigned long)sample.Raw28,
               sample.ErrWatchdog ? 1u : 0u,
               sample.ErrAmplitude ? 1u : 0u,
               (long)err,
               (err == ESP_OK) ? "ok" : "read_error");
    }

    sensorarrayIdleForever("fdc_selftest_done");
}

static bool sensorarrayIsS1D1RouteOnlyBehavior(void)
{
#if CONFIG_SENSORARRAY_DEBUG_S1D1_ROUTE_AND_ADS_READ
    return false;
#else
    return true;
#endif
}

static const char *sensorarrayS1D1BehaviorName(void)
{
    return sensorarrayIsS1D1RouteOnlyBehavior() ? "ROUTE_ONLY" : "ROUTE_AND_ADS_READ";
}

static bool sensorarrayIsS1D1StaticRouteOnlyBehavior(void)
{
#if CONFIG_SENSORARRAY_S1D1_STATIC_ROUTE_AND_READ
    return false;
#else
    return true;
#endif
}

static const char *sensorarrayS1D1StaticBehaviorName(void)
{
    return sensorarrayIsS1D1StaticRouteOnlyBehavior() ? "ROUTE_ONLY" : "ROUTE_AND_READ";
}

static void sensorarrayLogS1D1ControlState(const char *mapLabel)
{
    char rowBuf[12];
    char a0Buf[8];
    char a1Buf[8];
    char a2Buf[8];
    char swBuf[8];
    char selABuf[8];
    char selBBuf[8];

    tmuxSwitchControlState_t ctrl = {0};
    bool haveCtrl = (tmuxSwitchGetControlState(&ctrl) == ESP_OK);
    printf("DBGCTRL,point=S1D1,row=%s,a0=%s,a1=%s,a2=%s,sw=%s,selA=%s,selB=%s,map=%s\n",
           sensorarrayFmtU8(rowBuf, sizeof(rowBuf), haveCtrl, haveCtrl ? ctrl.row : 0u),
           sensorarrayFmtGpioLevel(a0Buf, sizeof(a0Buf), haveCtrl, haveCtrl ? ctrl.a0Level : -1),
           sensorarrayFmtGpioLevel(a1Buf, sizeof(a1Buf), haveCtrl, haveCtrl ? ctrl.a1Level : -1),
           sensorarrayFmtGpioLevel(a2Buf, sizeof(a2Buf), haveCtrl, haveCtrl ? ctrl.a2Level : -1),
           sensorarrayFmtGpioLevel(swBuf, sizeof(swBuf), haveCtrl, haveCtrl ? ctrl.swLevel : -1),
           sensorarrayFmtGpioLevel(selABuf, sizeof(selABuf), haveCtrl, haveCtrl ? ctrl.sel1Level : -1),
           sensorarrayFmtGpioLevel(selBBuf, sizeof(selBBuf), haveCtrl, haveCtrl ? ctrl.sel2Level : -1),
           mapLabel ? mapLabel : SENSORARRAY_NA);
}

static esp_err_t sensorarrayDebugReadAdsS1D1Once(bool discardFirst, int32_t *outRaw, int32_t *outUv)
{
    if (!outRaw || !outUv) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_state.adsReady) {
        return ESP_ERR_INVALID_STATE;
    }
    *outRaw = 0;
    *outUv = 0;

    uint8_t muxp = 0;
    uint8_t muxn = 0;
    if (!sensorarrayAdsMuxForDLine(SENSORARRAY_D1, &muxp, &muxn)) {
        return ESP_ERR_INVALID_ARG;
    }

    printf("DBGADS1D1,point=S1D1,step=begin,dLine=1,muxp=%u,muxn=%u,discardFirst=%u\n",
           (unsigned)muxp,
           (unsigned)muxn,
           discardFirst ? 1u : 0u);

    esp_err_t err = sensorarrayReadAdsUv(SENSORARRAY_D1, discardFirst, outRaw, outUv);
    printf("DBGADS1D1,point=S1D1,step=done,dLine=1,muxp=%u,muxn=%u,raw=%ld,uv=%ld,discardFirst=%u,err=%ld,status=%s\n",
           (unsigned)muxp,
           (unsigned)muxn,
           (long)*outRaw,
           (long)*outUv,
           discardFirst ? 1u : 0u,
           (long)err,
           (err == ESP_OK) ? "ok" : "read_error");
    return err;
}

/*
 * Temporary single-point debug mode for isolating S1D1 resistive routing and ADS behaviour.
 * This mode deliberately avoids matrix scanning and FDC read paths.
 */
static void sensorarrayRunSingleResistorS1D1Mode(void)
{
    const sensorarrayRouteMap_t *routeMap =
        sensorarrayFindRouteMap(SENSORARRAY_S1, SENSORARRAY_D1, SENSORARRAY_PATH_RESISTIVE);
    const char *mapLabel = SENSORARRAY_NA;
    if (!routeMap) {
        sensorarrayLogStartup("s1d1_mode", ESP_ERR_NOT_SUPPORTED, "s1d1_route_missing", 0);
        sensorarrayIdleForever("s1d1_route_missing");
        return;
    }

    esp_err_t err = sensorarrayApplyRoute(SENSORARRAY_S1,
                                          SENSORARRAY_D1,
                                          SENSORARRAY_PATH_RESISTIVE,
                                          TMUX1108_SOURCE_GND,
                                          &mapLabel);
    printf("DBGROUTEFIX,point=S1D1,sColumn=1,row=0,dLine=1,path=res,map=%s,selAReq=%u,selBReq=%u,sw=%s,mode=%s,err=%ld,"
           "status=%s\n",
           mapLabel ? mapLabel : SENSORARRAY_NA,
           routeMap->selALevel ? 1u : 0u,
           routeMap->selBLevel ? 1u : 0u,
           sensorarraySwSourceName(TMUX1108_SOURCE_GND),
           sensorarrayS1D1BehaviorName(),
           (long)err,
           (err == ESP_OK) ? "route_applied" : "route_error");
    sensorarrayLogS1D1ControlState(mapLabel);
    if (err != ESP_OK) {
        sensorarrayIdleForever("s1d1_route_error");
        return;
    }

    if (sensorarrayIsS1D1RouteOnlyBehavior()) {
        sensorarrayIdleForever("s1d1_route_only");
        return;
    }

    while (true) {
        char valueBuf[24];
        char uvBuf[24];
        char rawBuf[24];
        int32_t raw = 0;
        int32_t uv = 0;

        err = sensorarrayDebugReadAdsS1D1Once((CONFIG_SENSORARRAY_DEBUG_S1D1_ADS_DISCARD_FIRST != 0), &raw, &uv);
        sensorarrayLogDbg("S1D1",
                          "res",
                          "S1",
                          "D1",
                          sensorarraySwSourceName(TMUX1108_SOURCE_GND),
                          "ads_s1d1",
                          sensorarrayFmtI32(valueBuf, sizeof(valueBuf), err == ESP_OK, uv),
                          sensorarrayFmtI32(uvBuf, sizeof(uvBuf), err == ESP_OK, uv),
                          SENSORARRAY_NA,
                          sensorarrayFmtI32(rawBuf, sizeof(rawBuf), err == ESP_OK, raw),
                          SENSORARRAY_NA,
                          SENSORARRAY_NA,
                          SENSORARRAY_NA,
                          SENSORARRAY_NA,
                          SENSORARRAY_NA,
                          SENSORARRAY_NA,
                          SENSORARRAY_NA,
                          mapLabel ? mapLabel : SENSORARRAY_NA,
                          err,
                          (err == ESP_OK) ? "ads_read_ok" : "ads_read_error");
        sensorarrayDelayMs((uint32_t)CONFIG_SENSORARRAY_DEBUG_S1D1_ADS_INTERVAL_MS);
    }
}

static esp_err_t sensorarrayApplyS1D1StaticRoute(const sensorarrayRouteMap_t *routeMap,
                                                 bool reapplyEachLoop,
                                                 const char **outMapLabel)
{
    const char *mapLabel = SENSORARRAY_NA;
    esp_err_t err = sensorarrayApplyRoute(SENSORARRAY_S1,
                                          SENSORARRAY_D1,
                                          SENSORARRAY_PATH_RESISTIVE,
                                          TMUX1108_SOURCE_GND,
                                          &mapLabel);

    if (outMapLabel) {
        *outMapLabel = mapLabel ? mapLabel : SENSORARRAY_NA;
    }

    printf("DBGROUTEFIX,mode=s1d1_static,point=S1D1,kind=res,column=S1,dline=D1,pathIntent=resistive,"
           "swIntent=GND_LOW,map=%s,selARequested=%u,selBRequested=%u,reapplyEachLoop=%u,behaviour=%s,routeErr=%ld,"
           "status=%s\n",
           mapLabel ? mapLabel : SENSORARRAY_NA,
           (routeMap != NULL && routeMap->selALevel) ? 1u : 0u,
           (routeMap != NULL && routeMap->selBLevel) ? 1u : 0u,
           reapplyEachLoop ? 1u : 0u,
           sensorarrayS1D1StaticBehaviorName(),
           (long)err,
           (err == ESP_OK) ? "route_applied" : "route_error");
    sensorarrayLogS1D1ControlState(mapLabel);
    sensorarrayLogControlGpio("s1d1_static_route_applied", "S1D1");

    return err;
}

/*
 * Developer note:
 * menuconfig -> SensorArray Project -> Debug routing and self-test ->
 * Enable S1D1 static resistor debug mode.
 * ROUTE_ONLY applies/holds S1D1 resistive route and logs heartbeat only.
 * ROUTE_AND_READ keeps the same route and reads ADS D1 each loop.
 * Re-apply route each loop toggles one-time route hold vs per-loop re-assert.
 */
static void sensorarrayRunS1D1StaticResistorDebug(void)
{
    const sensorarrayRouteMap_t *routeMap =
        sensorarrayFindRouteMap(SENSORARRAY_S1, SENSORARRAY_D1, SENSORARRAY_PATH_RESISTIVE);
    const bool routeOnly = sensorarrayIsS1D1StaticRouteOnlyBehavior();
    const bool reapplyEachLoop = (CONFIG_SENSORARRAY_S1D1_STATIC_REAPPLY_ROUTE_EACH_LOOP != 0);
    const uint32_t loopDelayMs = (uint32_t)CONFIG_SENSORARRAY_S1D1_STATIC_LOOP_DELAY_MS;
    const char *mapLabel = SENSORARRAY_NA;
    uint32_t heartbeatCount = 0u;

    printf("=== S1D1 STATIC RESISTOR DEBUG MODE ACTIVE ===\n");
    printf("DBGSTATIC,mode=s1d1_resistor_static,status=active,point=S1D1,kind=res,column=S1,dline=D1,"
           "pathIntent=resistive,swIntent=GND_LOW,behaviour=%s,reapplyEachLoop=%u,loopDelayMs=%u\n",
           sensorarrayS1D1StaticBehaviorName(),
           reapplyEachLoop ? 1u : 0u,
           (unsigned)loopDelayMs);

    if (!routeMap) {
        sensorarrayLogStartup("s1d1_static", ESP_ERR_NOT_SUPPORTED, "s1d1_route_missing", 0);
        sensorarrayIdleForever("s1d1_static_route_missing");
        return;
    }

    sensorarrayLogStartup("s1d1_static",
                          ESP_OK,
                          routeOnly ? "route_only" : "route_and_read",
                          (int32_t)loopDelayMs);

    if (!reapplyEachLoop) {
        esp_err_t routeErr = sensorarrayApplyS1D1StaticRoute(routeMap, false, &mapLabel);
        if (routeErr != ESP_OK) {
            sensorarrayIdleForever("s1d1_static_route_error");
            return;
        }
    } else {
        mapLabel = routeMap->mapLabel ? routeMap->mapLabel : SENSORARRAY_NA;
    }

    while (true) {
        esp_err_t routeErr = ESP_OK;
        char heartbeatBuf[24];

        if (reapplyEachLoop) {
            routeErr = sensorarrayApplyS1D1StaticRoute(routeMap, true, &mapLabel);
        }
        if (routeErr != ESP_OK) {
            sensorarrayLogDbg("S1D1",
                              "res",
                              "S1",
                              "D1",
                              sensorarraySwSourceName(TMUX1108_SOURCE_GND),
                              routeOnly ? "s1d1_static_route_only" : "s1d1_static_route_and_read",
                              sensorarrayFmtU32(heartbeatBuf, sizeof(heartbeatBuf), true, heartbeatCount),
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
                              mapLabel ? mapLabel : SENSORARRAY_NA,
                              routeErr,
                              "route_error");
            sensorarrayDelayMs(loopDelayMs);
            ++heartbeatCount;
            continue;
        }

        if (routeOnly) {
            sensorarrayLogDbg("S1D1",
                              "res",
                              "S1",
                              "D1",
                              sensorarraySwSourceName(TMUX1108_SOURCE_GND),
                              "s1d1_static_route_only",
                              sensorarrayFmtU32(heartbeatBuf, sizeof(heartbeatBuf), true, heartbeatCount),
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
                              mapLabel ? mapLabel : SENSORARRAY_NA,
                              ESP_OK,
                              "heartbeat");
        } else {
            char valueBuf[24];
            char uvBuf[24];
            char rawBuf[24];
            int32_t raw = 0;
            int32_t uv = 0;

            esp_err_t readErr = sensorarrayDebugReadAdsS1D1Once((CONFIG_SENSORARRAY_DEBUG_S1D1_ADS_DISCARD_FIRST != 0),
                                                                &raw,
                                                                &uv);
            sensorarrayLogDbg("S1D1",
                              "res",
                              "S1",
                              "D1",
                              sensorarraySwSourceName(TMUX1108_SOURCE_GND),
                              "s1d1_static_route_and_read",
                              sensorarrayFmtI32(valueBuf, sizeof(valueBuf), readErr == ESP_OK, uv),
                              sensorarrayFmtI32(uvBuf, sizeof(uvBuf), readErr == ESP_OK, uv),
                              SENSORARRAY_NA,
                              sensorarrayFmtI32(rawBuf, sizeof(rawBuf), readErr == ESP_OK, raw),
                              SENSORARRAY_NA,
                              SENSORARRAY_NA,
                              SENSORARRAY_NA,
                              SENSORARRAY_NA,
                              SENSORARRAY_NA,
                              SENSORARRAY_NA,
                              SENSORARRAY_NA,
                              mapLabel ? mapLabel : SENSORARRAY_NA,
                              readErr,
                              (readErr == ESP_OK) ? "ads_read_ok" : "ads_read_error");
        }

        sensorarrayDelayMs(loopDelayMs);
        ++heartbeatCount;
    }
}

static void sensorarrayRunBringupLoop(void)
{
    while (true) {
        // Keep bring-up output focused on S1D1 only.
        sensorarrayDebugReadResistor("S1D1", SENSORARRAY_S1, SENSORARRAY_D1);

        sensorarrayDelayMs((uint32_t)CONFIG_SENSORARRAY_DEBUG_SCAN_LOOP_DELAY_MS);
    }
}

static void sensorarrayRunRouteIdleMode(void)
{
    sensorarrayLogStartup("debug_mode", ESP_OK, "route_idle", 0);
    sensorarrayIdleForever("route_idle");
}

static void sensorarrayRunRouteFixedStateMode(void)
{
    sensorarrayDebugFixedRoute_t cfg = {
        .sColumn = (uint8_t)CONFIG_SENSORARRAY_DEBUG_FIXED_S_COLUMN,
        .dLine = (uint8_t)CONFIG_SENSORARRAY_DEBUG_FIXED_D_LINE,
        .path = sensorarrayConfiguredFixedPath(),
        .swSource = sensorarrayConfiguredFixedSwSource(),
        .selALevel = (CONFIG_SENSORARRAY_DEBUG_FIXED_SELA_LEVEL != 0),
        .selBLevel = (CONFIG_SENSORARRAY_DEBUG_FIXED_SELB_LEVEL != 0),
        .skipAdsRead = (CONFIG_SENSORARRAY_DEBUG_FIXED_SKIP_ADS_READ != 0),
        .skipFdcRead = (CONFIG_SENSORARRAY_DEBUG_FIXED_SKIP_FDC_READ != 0),
        .delayAfterRowMs = (uint32_t)CONFIG_SENSORARRAY_DEBUG_ROUTE_STEP_DELAY_MS,
        .delayAfterSelAMs = (uint32_t)CONFIG_SENSORARRAY_DEBUG_ROUTE_STEP_DELAY_MS,
        .delayAfterSelBMs = (uint32_t)CONFIG_SENSORARRAY_DEBUG_ROUTE_STEP_DELAY_MS,
        .delayAfterSwMs = (uint32_t)CONFIG_SENSORARRAY_DEBUG_ROUTE_STEP_DELAY_MS,
        .holdForever = (CONFIG_SENSORARRAY_DEBUG_FIXED_HOLD_FOREVER != 0),
        .holdMs = (uint32_t)CONFIG_SENSORARRAY_DEBUG_FIXED_HOLD_MS,
        .label = "ROUTE_FIXED",
    };

    esp_err_t err = sensorarrayDebugApplyFixedRoute(&cfg);
    sensorarrayLogStartup("route_fixed", err, (err == ESP_OK) ? "done" : "apply_error", (int32_t)cfg.dLine);
    sensorarrayIdleForever("route_fixed_done");
}

static void sensorarrayRunRouteStepOnceMode(void)
{
    for (size_t i = 0; i < (sizeof(s_sensorarrayRouteMap) / sizeof(s_sensorarrayRouteMap[0])); ++i) {
        const sensorarrayRouteMap_t *route = &s_sensorarrayRouteMap[i];
        tmux1108Source_t source = sensorarrayRouteMapDefaultSwSource(route);
        sensorarrayDebugFixedRoute_t cfg = {
            .sColumn = route->sColumn,
            .dLine = route->dLine,
            .path = sensorarrayPathToDebugPath(route->path, source),
            .swSource = source,
            .selALevel = route->selALevel,
            .selBLevel = route->selBLevel,
            .skipAdsRead = (route->path == SENSORARRAY_PATH_CAPACITIVE),
            .skipFdcRead = (route->path != SENSORARRAY_PATH_CAPACITIVE),
            .delayAfterRowMs = (uint32_t)CONFIG_SENSORARRAY_DEBUG_ROUTE_STEP_DELAY_MS,
            .delayAfterSelAMs = (uint32_t)CONFIG_SENSORARRAY_DEBUG_ROUTE_STEP_DELAY_MS,
            .delayAfterSelBMs = (uint32_t)CONFIG_SENSORARRAY_DEBUG_ROUTE_STEP_DELAY_MS,
            .delayAfterSwMs = (uint32_t)CONFIG_SENSORARRAY_DEBUG_ROUTE_STEP_DELAY_MS,
            .holdForever = false,
            .holdMs = (uint32_t)CONFIG_SENSORARRAY_DEBUG_STEP_ONCE_DELAY_MS,
            .label = route->mapLabel,
        };

        esp_err_t err = sensorarrayDebugApplyFixedRoute(&cfg);
        if (err != ESP_OK) {
            sensorarrayLogStartup("route_step_once", err, "step_error", (int32_t)i);
            break;
        }
    }
    sensorarrayIdleForever("route_step_once_done");
}

static void sensorarrayRunSelectedDebugMode(void)
{
    sensorarrayDebugMode_t mode = (sensorarrayDebugMode_t)SENSORARRAY_ACTIVE_DEBUG_MODE;
    sensorarrayLogStartup("debug_mode", ESP_OK, sensorarrayDebugModeName(mode), (int32_t)mode);

    switch (mode) {
    case SENSORARRAY_DEBUG_MODE_ROUTE_IDLE:
        sensorarrayRunRouteIdleMode();
        return;
    case SENSORARRAY_DEBUG_MODE_ROUTE_FIXED_STATE:
        sensorarrayRunRouteFixedStateMode();
        return;
    case SENSORARRAY_DEBUG_MODE_ROUTE_STEP_ONCE:
        sensorarrayRunRouteStepOnceMode();
        return;
    case SENSORARRAY_DEBUG_MODE_ROUTE_SCAN_LOOP:
        sensorarrayRunBringupLoop();
        return;
    case SENSORARRAY_DEBUG_MODE_ADS_SELFTEST:
        sensorarrayRunAdsSelftestMode();
        return;
    case SENSORARRAY_DEBUG_MODE_FDC_SELFTEST:
        sensorarrayRunFdcSelftestMode();
        return;
    case SENSORARRAY_DEBUG_MODE_S1D1_RESISTOR:
        sensorarrayRunSingleResistorS1D1Mode();
        return;
    default:
        sensorarrayRunBringupLoop();
        return;
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
    sensorarrayDbgExtraReset();
    s_state.adsAdc1Running = false;
    s_state.adsRefMuxValid = false;
    s_state.adsRefMux = 0u;

    sensorarrayDebugMode_t activeMode = (sensorarrayDebugMode_t)SENSORARRAY_ACTIVE_DEBUG_MODE;
    bool s1d1StaticMode = (CONFIG_SENSORARRAY_S1D1_RESISTOR_STATIC_DEBUG != 0);
    bool s1d1StaticRouteOnly = s1d1StaticMode && sensorarrayIsS1D1StaticRouteOnlyBehavior();
    bool s1d1ResMode = (activeMode == SENSORARRAY_DEBUG_MODE_S1D1_RESISTOR);
    bool s1d1RouteOnly = s1d1StaticRouteOnly || (s1d1ResMode && sensorarrayIsS1D1RouteOnlyBehavior());
    bool s1d1SkipFdcMode = s1d1StaticMode || s1d1ResMode;

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
    sensorarrayAuditRouteMap();
    sensorarrayAuditFdcDLineMap();

    esp_err_t err = boardSupportInit();
    s_state.boardReady = (err == ESP_OK);
    sensorarrayLogStartup("board", err, s_state.boardReady ? "ok" : "init_failed", (int32_t)s_state.boardReady);

    err = tmuxSwitchInit();
    s_state.tmuxReady = (err == ESP_OK);
    sensorarrayLogStartup("tmux", err, s_state.tmuxReady ? "ok" : "init_failed", (int32_t)s_state.tmuxReady);

    if (s_state.tmuxReady) {
        esp_err_t tmuxErr = tmuxSwitchSelectRow(0);
        if (tmuxErr == ESP_OK) {
            tmuxErr = tmux1134SelectSelALevel(false);
        }
        if (tmuxErr == ESP_OK) {
            tmuxErr = tmux1134SelectSelBLevel(false);
        }
        if (tmuxErr == ESP_OK) {
            tmuxErr = tmuxSwitchSet1108Source(TMUX1108_SOURCE_GND);
        }
        if (tmuxErr == ESP_OK) {
            tmuxErr = tmux1134SetEnLogicalState(true);
        }
        sensorarrayLogStartup("tmux_defaults",
                              tmuxErr,
                              (tmuxErr == ESP_OK) ? "ok" : "set_failed",
                              (int32_t)(tmuxErr == ESP_OK));
        sensorarrayLogControlGpio("tmux_defaults", "INIT");
    }

    if (s1d1RouteOnly) {
        s_state.adsReady = false;
        s_state.adsRefReady = false;
        s_state.adsAdc1Running = false;
        s_state.adsRefMuxValid = false;
        sensorarrayLogStartup("ads", ESP_ERR_INVALID_STATE, "skip_route_only_mode", 0);
        sensorarrayLogStartup("ads_ref", ESP_ERR_INVALID_STATE, "skip_route_only_mode", 0);
    } else {
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
            s_state.adsAdc1Running = false;
            s_state.adsRefMuxValid = false;
            sensorarrayLogStartup("ads_ref", ESP_ERR_INVALID_STATE, "skip_ads_unavailable", 0);
        }
    }

    if (s1d1SkipFdcMode) {
        const char *fdcSkipStatus = s1d1StaticMode ? "skip_s1d1_static_mode" : "skip_s1d1_resistor_mode";
        sensorarrayLogStartupFdc("fdc_init",
                                 &s_state.fdcPrimary,
                                 ESP_ERR_NOT_SUPPORTED,
                                 fdcSkipStatus,
                                 0,
                                 false,
                                 0,
                                 0,
                                 "D1..D4");
        sensorarrayLogStartupFdc("fdc_init",
                                 &s_state.fdcSecondary,
                                 ESP_ERR_NOT_SUPPORTED,
                                 fdcSkipStatus,
                                 0,
                                 false,
                                 0,
                                 0,
                                 "D5..D8");
    } else if (s_state.boardReady) {
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

    if (s1d1StaticMode) {
        sensorarrayRunS1D1StaticResistorDebug();
        return;
    }

    sensorarrayRunSelectedDebugMode();
}

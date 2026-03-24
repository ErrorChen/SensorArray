#include "sensorarrayLog.h"

#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"

#include "sensorarrayBoardMap.h"
#include "sensorarrayConfig.h"

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

static bool s_adsReady = false;
static bool s_adsRefReady = false;
static sensorarrayDbgExtra_t s_dbgExtra = {0};

static const char *sensorarrayRefState(void)
{
    if (!s_adsReady) {
        return "UNAVAILABLE";
    }
    return s_adsRefReady ? "READY" : "NOT_READY";
}

static const char *sensorarrayRefReadyBit(void)
{
    if (!s_adsReady) {
        return SENSORARRAY_NA;
    }
    return s_adsRefReady ? "1" : "0";
}

static int sensorarrayReadOptionalGpioLevel(int gpio)
{
    if (gpio < 0) {
        return -1;
    }
    return gpio_get_level((gpio_num_t)gpio);
}

void sensorarrayLogSetAdsState(bool adsReady, bool adsRefReady)
{
    s_adsReady = adsReady;
    s_adsRefReady = adsRefReady;
}

void sensorarrayLogDbgExtraReset(void)
{
    memset(&s_dbgExtra, 0, sizeof(s_dbgExtra));
}

void sensorarrayLogDbgExtraSetMux(uint8_t muxp, uint8_t muxn)
{
    s_dbgExtra.muxValid = true;
    s_dbgExtra.muxp = muxp;
    s_dbgExtra.muxn = muxn;
}

void sensorarrayLogDbgExtraSetRefMux(uint8_t refmux)
{
    s_dbgExtra.refmuxValid = true;
    s_dbgExtra.refmux = refmux;
}

void sensorarrayLogDbgExtraSetDiscardCount(uint8_t discardCount)
{
    s_dbgExtra.discardCountValid = true;
    s_dbgExtra.discardCount = discardCount;
}

void sensorarrayLogDbgExtraCaptureCtrl(void)
{
    tmuxSwitchControlState_t ctrl = {0};
    if (tmuxSwitchGetControlState(&ctrl) == ESP_OK) {
        s_dbgExtra.ctrlValid = true;
        s_dbgExtra.ctrl = ctrl;
    } else {
        s_dbgExtra.ctrlValid = false;
    }
}

const char *sensorarrayLogFmtI32(char *buf, size_t bufSize, bool valid, int32_t value)
{
    if (!valid) {
        return SENSORARRAY_NA;
    }
    snprintf(buf, bufSize, "%ld", (long)value);
    return buf;
}

const char *sensorarrayLogFmtU32(char *buf, size_t bufSize, bool valid, uint32_t value)
{
    if (!valid) {
        return SENSORARRAY_NA;
    }
    snprintf(buf, bufSize, "%lu", (unsigned long)value);
    return buf;
}

const char *sensorarrayLogFmtU8(char *buf, size_t bufSize, bool valid, uint8_t value)
{
    if (!valid) {
        return SENSORARRAY_NA;
    }
    snprintf(buf, bufSize, "%u", (unsigned)value);
    return buf;
}

const char *sensorarrayLogFmtBool(char *buf, size_t bufSize, bool valid, bool value)
{
    if (!valid) {
        return SENSORARRAY_NA;
    }
    snprintf(buf, bufSize, "%d", value ? 1 : 0);
    return buf;
}

const char *sensorarrayLogFmtHexU8(char *buf, size_t bufSize, bool valid, uint8_t value)
{
    if (!valid) {
        return SENSORARRAY_NA;
    }
    snprintf(buf, bufSize, "0x%02X", value);
    return buf;
}

const char *sensorarrayLogFmtHexU16(char *buf, size_t bufSize, bool valid, uint16_t value)
{
    if (!valid) {
        return SENSORARRAY_NA;
    }
    snprintf(buf, bufSize, "0x%04X", value);
    return buf;
}

const char *sensorarrayLogFmtI2cPort(char *buf, size_t bufSize, const BoardSupportI2cCtx_t *ctx)
{
    if (!ctx) {
        return SENSORARRAY_NA;
    }
    snprintf(buf, bufSize, "%d", (int)ctx->Port);
    return buf;
}

const char *sensorarrayLogFmtGpioLevel(char *buf, size_t bufSize, bool valid, int level)
{
    if (!valid || level < 0) {
        return SENSORARRAY_NA;
    }
    snprintf(buf, bufSize, "%d", level ? 1 : 0);
    return buf;
}

const char *sensorarrayLogSwSourceName(tmux1108Source_t source)
{
    return (source == TMUX1108_SOURCE_REF) ? "HIGH" : "LOW";
}

const char *sensorarrayLogSwSourceLogicalName(tmux1108Source_t source)
{
    return (source == TMUX1108_SOURCE_REF) ? "REF" : "GND";
}

const char *sensorarrayLogAdsMuxName(uint8_t mux)
{
    switch (mux & 0x0Fu) {
    case 0x00:
        return "AIN0";
    case 0x01:
        return "AIN1";
    case 0x02:
        return "AIN2";
    case 0x03:
        return "AIN3";
    case 0x04:
        return "AIN4";
    case 0x05:
        return "AIN5";
    case 0x06:
        return "AIN6";
    case 0x07:
        return "AIN7";
    case 0x08:
        return "AIN8";
    case 0x09:
        return "AIN9";
    case SENSORARRAY_ADS_MUX_AINCOM:
        return "AINCOM";
    default:
        return "UNKNOWN";
    }
}

const char *sensorarrayLogDebugModeName(sensorarrayDebugMode_t mode)
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
    case SENSORARRAY_DEBUG_MODE_S1D1_FORCE_ADS_HOLD:
        return "S1D1_FORCE_ADS_HOLD";
    default:
        return "UNKNOWN";
    }
}

const char *sensorarrayLogDebugPathName(sensorarrayDebugPath_t path)
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

const char *sensorarrayLogBuildMapLabel(char *buf,
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

void sensorarrayLogDbg(const char *point,
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
           sensorarrayLogFmtU8(muxpBuf, sizeof(muxpBuf), s_dbgExtra.muxValid, s_dbgExtra.muxp),
           sensorarrayLogFmtU8(muxnBuf, sizeof(muxnBuf), s_dbgExtra.muxValid, s_dbgExtra.muxn),
           sensorarrayLogFmtHexU8(refmuxBuf, sizeof(refmuxBuf), s_dbgExtra.refmuxValid, s_dbgExtra.refmux),
           sensorarrayLogFmtU8(discardBuf, sizeof(discardBuf), s_dbgExtra.discardCountValid, s_dbgExtra.discardCount),
           sensorarrayLogFmtGpioLevel(ctrlA0Buf, sizeof(ctrlA0Buf), ctrl != NULL, ctrl ? ctrl->a0Level : -1),
           sensorarrayLogFmtGpioLevel(ctrlA1Buf, sizeof(ctrlA1Buf), ctrl != NULL, ctrl ? ctrl->a1Level : -1),
           sensorarrayLogFmtGpioLevel(ctrlA2Buf, sizeof(ctrlA2Buf), ctrl != NULL, ctrl ? ctrl->a2Level : -1),
           sensorarrayLogFmtGpioLevel(ctrlSwBuf, sizeof(ctrlSwBuf), ctrl != NULL, ctrl ? ctrl->swLevel : -1),
           sensorarrayLogFmtGpioLevel(ctrlSel1Buf, sizeof(ctrlSel1Buf), ctrl != NULL, ctrl ? ctrl->selaLevel : -1),
           sensorarrayLogFmtGpioLevel(ctrlSel2Buf, sizeof(ctrlSel2Buf), ctrl != NULL, ctrl ? ctrl->selbLevel : -1),
           sensorarrayLogFmtGpioLevel(ctrlSel3Buf, sizeof(ctrlSel3Buf), ctrl != NULL, ctrl ? ctrl->sel3Level : -1),
           sensorarrayLogFmtGpioLevel(ctrlSel4Buf, sizeof(ctrlSel4Buf), ctrl != NULL, ctrl ? ctrl->sel4Level : -1),
           sensorarrayLogFmtGpioLevel(ctrlEnBuf, sizeof(ctrlEnBuf), ctrl != NULL, ctrl ? ctrl->enLevel : -1));
    sensorarrayLogDbgExtraReset();
}

void sensorarrayLogStartup(const char *mode, esp_err_t err, const char *status, int32_t detailValue)
{
    char valueBuf[24];
    sensorarrayLogDbg("INIT",
                      "startup",
                      SENSORARRAY_NA,
                      SENSORARRAY_NA,
                      SENSORARRAY_NA,
                      mode,
                      sensorarrayLogFmtI32(valueBuf, sizeof(valueBuf), true, detailValue),
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

void sensorarrayLogStartupFdc(const char *mode,
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
                      sensorarrayLogFmtI32(valueBuf, sizeof(valueBuf), true, detailValue),
                      SENSORARRAY_NA,
                      SENSORARRAY_NA,
                      SENSORARRAY_NA,
                      SENSORARRAY_NA,
                      SENSORARRAY_NA,
                      fdcState ? fdcState->label : SENSORARRAY_NA,
                      sensorarrayLogFmtI2cPort(portBuf, sizeof(portBuf), fdcState ? fdcState->i2cCtx : NULL),
                      sensorarrayLogFmtHexU8(addrBuf, sizeof(addrBuf), fdcState != NULL, fdcState ? fdcState->i2cAddr : 0),
                      sensorarrayLogFmtHexU16(idMfgBuf, sizeof(idMfgBuf), hasIds, manufacturerId),
                      sensorarrayLogFmtHexU16(idDevBuf, sizeof(idDevBuf), hasIds, deviceId),
                      map,
                      err,
                      status);
}

void sensorarrayLogControlGpio(const char *stage, const char *point)
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
           sensorarrayLogFmtGpioLevel(a0Buf, sizeof(a0Buf), haveCtrl, haveCtrl ? ctrl.a0Level : -1),
           sensorarrayLogFmtGpioLevel(a1Buf, sizeof(a1Buf), haveCtrl, haveCtrl ? ctrl.a1Level : -1),
           sensorarrayLogFmtGpioLevel(a2Buf, sizeof(a2Buf), haveCtrl, haveCtrl ? ctrl.a2Level : -1),
           sensorarrayLogFmtGpioLevel(swBuf, sizeof(swBuf), haveCtrl, haveCtrl ? ctrl.swLevel : -1),
           sensorarrayLogFmtGpioLevel(sel1Buf, sizeof(sel1Buf), haveCtrl, haveCtrl ? ctrl.sel1Level : -1),
           sensorarrayLogFmtGpioLevel(sel2Buf, sizeof(sel2Buf), haveCtrl, haveCtrl ? ctrl.sel2Level : -1),
           sensorarrayLogFmtGpioLevel(sel3Buf, sizeof(sel3Buf), haveCtrl, haveCtrl ? ctrl.sel3Level : -1),
           sensorarrayLogFmtGpioLevel(sel4Buf, sizeof(sel4Buf), haveCtrl, haveCtrl ? ctrl.sel4Level : -1),
           sensorarrayLogFmtGpioLevel(enBuf, sizeof(enBuf), haveCtrl, haveCtrl ? ctrl.enLevel : -1),
           sensorarrayLogFmtGpioLevel(drdyBuf, sizeof(drdyBuf), true, drdyLevel),
           sensorarrayLogFmtGpioLevel(resetBuf, sizeof(resetBuf), true, resetLevel));
}

void sensorarrayLogSelaRouteDecision(const char *stage,
                                     const char *label,
                                     sensorarraySelaRoute_t requestRoute,
                                     int selaWriteLevel,
                                     int selaReadLevel,
                                     bool resolvedValid,
                                     sensorarraySelaRoute_t resolvedRoute)
{
    printf("[ROUTE] request=%s, selaWrite=%d, selaRead=%d, resolved=%s, stage=%s, label=%s\n",
           sensorarrayBoardMapSelaRouteName(requestRoute),
           selaWriteLevel,
           selaReadLevel,
           resolvedValid ? sensorarrayBoardMapSelaRouteName(resolvedRoute) : "UNKNOWN",
           stage ? stage : SENSORARRAY_NA,
           label ? label : SENSORARRAY_NA);
}

void sensorarrayLogSelaReadbackMismatch(const char *stage, const char *label, int wroteLevel, int readLevel)
{
    printf("[ROUTE][WARN] SELA readback mismatch: wrote=%d read=%d, stage=%s, label=%s\n",
           wroteLevel,
           readLevel,
           stage ? stage : SENSORARRAY_NA,
           label ? label : SENSORARRAY_NA);
}

void sensorarrayLogRouteStep(const char *stage,
                             const char *label,
                             uint8_t sColumn,
                             uint8_t dLine,
                             sensorarrayDebugPath_t path,
                             tmux1108Source_t swSource,
                             sensorarraySelaRoute_t selaRoute,
                             bool selBLevel,
                             esp_err_t err,
                             const char *status)
{
    char selaWriteBuf[8];
    int selaWriteLevel = -1;
    const bool haveSelaWrite = sensorarrayBoardMapSelaRouteToGpioLevel(selaRoute, &selaWriteLevel);

    printf("DBGROUTE,stage=%s,label=%s,sColumn=%u,dLine=%u,path=%s,sw=%s,selaRequest=%s,selaWrite=%s,"
           "selBLevel=%u,err=%ld,status=%s\n",
           stage ? stage : SENSORARRAY_NA,
           label ? label : SENSORARRAY_NA,
           (unsigned)sColumn,
           (unsigned)dLine,
           sensorarrayLogDebugPathName(path),
           sensorarrayLogSwSourceName(swSource),
           haveSelaWrite ? sensorarrayBoardMapSelaRouteName(selaRoute) : SENSORARRAY_NA,
           sensorarrayLogFmtGpioLevel(selaWriteBuf, sizeof(selaWriteBuf), haveSelaWrite, selaWriteLevel),
           selBLevel ? 1u : 0u,
           (long)err,
           status ? status : SENSORARRAY_NA);
    sensorarrayLogControlGpio(stage, label);
}

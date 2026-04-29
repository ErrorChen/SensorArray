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
    return (source == TMUX1108_SOURCE_REF) ? "REF" : "GND";
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
    case SENSORARRAY_DEBUG_MODE_S5D5_CAP_FDC_SECONDARY:
        return "S5D5_CAP_FDC_SECONDARY";
    case SENSORARRAY_DEBUG_MODE_FDC_I2C_DISCOVERY:
        return "FDC_I2C_DISCOVERY";
    default:
        return "UNKNOWN";
    }
}

const char *sensorarrayLogDebugPathName(sensorarrayDebugPath_t path)
{
    switch (path) {
    case SENSORARRAY_DEBUG_PATH_CAPACITIVE:
        return "CAPACITIVE";
    case SENSORARRAY_DEBUG_PATH_VOLTAGE:
        return "PIEZO_VOLTAGE";
    case SENSORARRAY_DEBUG_PATH_RESISTIVE:
    default:
        return "RESISTIVE";
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
    char cmdA0Buf[8];
    char cmdA1Buf[8];
    char cmdA2Buf[8];
    char cmdSwBuf[8];
    char cmdSel1Buf[8];
    char cmdSel2Buf[8];
    char cmdSel3Buf[8];
    char cmdSel4Buf[8];
    char cmdEnBuf[8];
    char obsA0Buf[8];
    char obsA1Buf[8];
    char obsA2Buf[8];
    char obsSwBuf[8];
    char obsSel1Buf[8];
    char obsSel2Buf[8];
    char obsSel3Buf[8];
    char obsSel4Buf[8];
    char obsEnBuf[8];
    const tmuxSwitchControlState_t *ctrl = s_dbgExtra.ctrlValid ? &s_dbgExtra.ctrl : NULL;

    printf("DBG,point=%s,kind=%s,column=%s,dline=%s,sw=%s,ref=%s,mode=%s,value=%s,valueUv=%s,"
           "valueMohm=%s,raw=%s,wd=%s,amp=%s,fdcDev=%s,i2cPort=%s,i2cAddr=%s,idMfg=%s,idDev=%s,"
           "refReady=%s,map=%s,err=%ld,status=%s,muxp=%s,muxn=%s,refmux=%s,discardCount=%s,"
           "cmdA0=%s,cmdA1=%s,cmdA2=%s,cmdSW=%s,cmdSELA=%s,cmdSELB=%s,cmdSEL3=%s,cmdSEL4=%s,cmdEN=%s,"
           "obsA0=%s,obsA1=%s,obsA2=%s,obsSW=%s,obsSELA=%s,obsSELB=%s,obsSEL3=%s,obsSEL4=%s,obsEN=%s\n",
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
           sensorarrayLogFmtGpioLevel(cmdA0Buf, sizeof(cmdA0Buf), ctrl != NULL, ctrl ? ctrl->cmdA0Level : -1),
           sensorarrayLogFmtGpioLevel(cmdA1Buf, sizeof(cmdA1Buf), ctrl != NULL, ctrl ? ctrl->cmdA1Level : -1),
           sensorarrayLogFmtGpioLevel(cmdA2Buf, sizeof(cmdA2Buf), ctrl != NULL, ctrl ? ctrl->cmdA2Level : -1),
           sensorarrayLogFmtGpioLevel(cmdSwBuf, sizeof(cmdSwBuf), ctrl != NULL, ctrl ? ctrl->cmdSwLevel : -1),
           sensorarrayLogFmtGpioLevel(cmdSel1Buf, sizeof(cmdSel1Buf), ctrl != NULL, ctrl ? ctrl->cmdSelaLevel : -1),
           sensorarrayLogFmtGpioLevel(cmdSel2Buf, sizeof(cmdSel2Buf), ctrl != NULL, ctrl ? ctrl->cmdSelbLevel : -1),
           sensorarrayLogFmtGpioLevel(cmdSel3Buf, sizeof(cmdSel3Buf), ctrl != NULL, ctrl ? ctrl->cmdSel3Level : -1),
           sensorarrayLogFmtGpioLevel(cmdSel4Buf, sizeof(cmdSel4Buf), ctrl != NULL, ctrl ? ctrl->cmdSel4Level : -1),
           sensorarrayLogFmtGpioLevel(cmdEnBuf, sizeof(cmdEnBuf), ctrl != NULL, ctrl ? ctrl->cmdEnLevel : -1),
           sensorarrayLogFmtGpioLevel(obsA0Buf, sizeof(obsA0Buf), ctrl != NULL, ctrl ? ctrl->obsA0Level : -1),
           sensorarrayLogFmtGpioLevel(obsA1Buf, sizeof(obsA1Buf), ctrl != NULL, ctrl ? ctrl->obsA1Level : -1),
           sensorarrayLogFmtGpioLevel(obsA2Buf, sizeof(obsA2Buf), ctrl != NULL, ctrl ? ctrl->obsA2Level : -1),
           sensorarrayLogFmtGpioLevel(obsSwBuf, sizeof(obsSwBuf), ctrl != NULL, ctrl ? ctrl->obsSwLevel : -1),
           sensorarrayLogFmtGpioLevel(obsSel1Buf, sizeof(obsSel1Buf), ctrl != NULL, ctrl ? ctrl->obsSelaLevel : -1),
           sensorarrayLogFmtGpioLevel(obsSel2Buf, sizeof(obsSel2Buf), ctrl != NULL, ctrl ? ctrl->obsSelbLevel : -1),
           sensorarrayLogFmtGpioLevel(obsSel3Buf, sizeof(obsSel3Buf), ctrl != NULL, ctrl ? ctrl->obsSel3Level : -1),
           sensorarrayLogFmtGpioLevel(obsSel4Buf, sizeof(obsSel4Buf), ctrl != NULL, ctrl ? ctrl->obsSel4Level : -1),
           sensorarrayLogFmtGpioLevel(obsEnBuf, sizeof(obsEnBuf), ctrl != NULL, ctrl ? ctrl->obsEnLevel : -1));
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
    char cmdA0Buf[8];
    char cmdA1Buf[8];
    char cmdA2Buf[8];
    char cmdSwBuf[8];
    char cmdSel1Buf[8];
    char cmdSel2Buf[8];
    char cmdSel3Buf[8];
    char cmdSel4Buf[8];
    char cmdEnBuf[8];
    char obsA0Buf[8];
    char obsA1Buf[8];
    char obsA2Buf[8];
    char obsSwBuf[8];
    char obsSel1Buf[8];
    char obsSel2Buf[8];
    char obsSel3Buf[8];
    char obsSel4Buf[8];
    char obsEnBuf[8];
    char drdyBuf[8];
    char resetBuf[8];

    tmuxSwitchControlState_t ctrl = {0};
    bool haveCtrl = (tmuxSwitchGetControlState(&ctrl) == ESP_OK);
    int drdyLevel = sensorarrayReadOptionalGpioLevel(CONFIG_BOARD_ADS126X_DRDY_GPIO);
    int resetLevel = sensorarrayReadOptionalGpioLevel(CONFIG_BOARD_ADS126X_RESET_GPIO);

    printf("DBGCTRL,stage=%s,point=%s,cmdA0=%s,cmdA1=%s,cmdA2=%s,cmdSW=%s,cmdSELA=%s,cmdSELB=%s,cmdSEL3=%s,"
           "cmdSEL4=%s,cmdEN=%s,obsA0=%s,obsA1=%s,obsA2=%s,obsSW=%s,obsSELA=%s,obsSELB=%s,obsSEL3=%s,"
           "obsSEL4=%s,obsEN=%s,drdy=%s,adsReset=%s\n",
           stage ? stage : SENSORARRAY_NA,
           point ? point : SENSORARRAY_NA,
           sensorarrayLogFmtGpioLevel(cmdA0Buf, sizeof(cmdA0Buf), haveCtrl, haveCtrl ? ctrl.cmdA0Level : -1),
           sensorarrayLogFmtGpioLevel(cmdA1Buf, sizeof(cmdA1Buf), haveCtrl, haveCtrl ? ctrl.cmdA1Level : -1),
           sensorarrayLogFmtGpioLevel(cmdA2Buf, sizeof(cmdA2Buf), haveCtrl, haveCtrl ? ctrl.cmdA2Level : -1),
           sensorarrayLogFmtGpioLevel(cmdSwBuf, sizeof(cmdSwBuf), haveCtrl, haveCtrl ? ctrl.cmdSwLevel : -1),
           sensorarrayLogFmtGpioLevel(cmdSel1Buf, sizeof(cmdSel1Buf), haveCtrl, haveCtrl ? ctrl.cmdSelaLevel : -1),
           sensorarrayLogFmtGpioLevel(cmdSel2Buf, sizeof(cmdSel2Buf), haveCtrl, haveCtrl ? ctrl.cmdSelbLevel : -1),
           sensorarrayLogFmtGpioLevel(cmdSel3Buf, sizeof(cmdSel3Buf), haveCtrl, haveCtrl ? ctrl.cmdSel3Level : -1),
           sensorarrayLogFmtGpioLevel(cmdSel4Buf, sizeof(cmdSel4Buf), haveCtrl, haveCtrl ? ctrl.cmdSel4Level : -1),
           sensorarrayLogFmtGpioLevel(cmdEnBuf, sizeof(cmdEnBuf), haveCtrl, haveCtrl ? ctrl.cmdEnLevel : -1),
           sensorarrayLogFmtGpioLevel(obsA0Buf, sizeof(obsA0Buf), haveCtrl, haveCtrl ? ctrl.obsA0Level : -1),
           sensorarrayLogFmtGpioLevel(obsA1Buf, sizeof(obsA1Buf), haveCtrl, haveCtrl ? ctrl.obsA1Level : -1),
           sensorarrayLogFmtGpioLevel(obsA2Buf, sizeof(obsA2Buf), haveCtrl, haveCtrl ? ctrl.obsA2Level : -1),
           sensorarrayLogFmtGpioLevel(obsSwBuf, sizeof(obsSwBuf), haveCtrl, haveCtrl ? ctrl.obsSwLevel : -1),
           sensorarrayLogFmtGpioLevel(obsSel1Buf, sizeof(obsSel1Buf), haveCtrl, haveCtrl ? ctrl.obsSel1Level : -1),
           sensorarrayLogFmtGpioLevel(obsSel2Buf, sizeof(obsSel2Buf), haveCtrl, haveCtrl ? ctrl.obsSel2Level : -1),
           sensorarrayLogFmtGpioLevel(obsSel3Buf, sizeof(obsSel3Buf), haveCtrl, haveCtrl ? ctrl.obsSel3Level : -1),
           sensorarrayLogFmtGpioLevel(obsSel4Buf, sizeof(obsSel4Buf), haveCtrl, haveCtrl ? ctrl.obsSel4Level : -1),
           sensorarrayLogFmtGpioLevel(obsEnBuf, sizeof(obsEnBuf), haveCtrl, haveCtrl ? ctrl.obsEnLevel : -1),
           sensorarrayLogFmtGpioLevel(drdyBuf, sizeof(drdyBuf), true, drdyLevel),
           sensorarrayLogFmtGpioLevel(resetBuf, sizeof(resetBuf), true, resetLevel));
}

void sensorarrayLogSelaRouteDecision(const char *stage,
                                     const char *label,
                                     sensorarraySelaRoute_t requestRoute,
                                     int selaWriteLevel,
                                     int selaCmdLevel,
                                     int selaObsLevel,
                                     bool obsResolvedValid,
                                     sensorarraySelaRoute_t obsResolvedRoute)
{
    printf("[ROUTE] request=%s,selaExpected=%d,cmdSELA=%d,obsSELA=%d,obsResolved=%s,stage=%s,label=%s,"
           "note=obsSELA_is_MCU_gpio_observation_only\n",
           sensorarrayBoardMapSelaRouteName(requestRoute),
           selaWriteLevel,
           selaCmdLevel,
           selaObsLevel,
           obsResolvedValid ? sensorarrayBoardMapSelaRouteName(obsResolvedRoute) : "UNKNOWN",
           stage ? stage : SENSORARRAY_NA,
           label ? label : SENSORARRAY_NA);
}

void sensorarrayLogSelaReadbackMismatch(const char *stage, const char *label, int wroteLevel, int readLevel)
{
    printf("[ROUTE][WARN] SELA GPIO observation mismatch: expected=%d obs=%d, stage=%s, label=%s,"
           "action=continue_functional_checks\n",
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
    char expectedSwBuf[8];
    char cmdSwBuf[8];
    char obsSwBuf[8];
    int selaWriteLevel = -1;
    const bool haveSelaWrite = sensorarrayBoardMapSelaRouteToGpioLevel(selaRoute, &selaWriteLevel);
    int expectedSwLevel = tmuxSwitch1108SourceToSwLevel(swSource);
    tmuxSwitchControlState_t ctrl = {0};
    bool haveCtrl = (tmuxSwitchGetControlState(&ctrl) == ESP_OK);

    printf("DBGROUTE,stage=%s,label=%s,sColumn=%u,dLine=%u,path=%s,swSource=%s,expectedSwLevel=%s,"
           "cmdSwLevel=%s,obsSwLevel=%s,selaRequest=%s,selaWrite=%s,selBLevel=%u,err=%ld,status=%s\n",
           stage ? stage : SENSORARRAY_NA,
           label ? label : SENSORARRAY_NA,
           (unsigned)sColumn,
           (unsigned)dLine,
           sensorarrayLogDebugPathName(path),
           sensorarrayLogSwSourceName(swSource),
           sensorarrayLogFmtGpioLevel(expectedSwBuf, sizeof(expectedSwBuf), expectedSwLevel >= 0, expectedSwLevel),
           sensorarrayLogFmtGpioLevel(cmdSwBuf, sizeof(cmdSwBuf), haveCtrl, haveCtrl ? ctrl.cmdSwLevel : -1),
           sensorarrayLogFmtGpioLevel(obsSwBuf, sizeof(obsSwBuf), haveCtrl, haveCtrl ? ctrl.obsSwLevel : -1),
           haveSelaWrite ? sensorarrayBoardMapSelaRouteName(selaRoute) : SENSORARRAY_NA,
           sensorarrayLogFmtGpioLevel(selaWriteBuf, sizeof(selaWriteBuf), haveSelaWrite, selaWriteLevel),
           selBLevel ? 1u : 0u,
           (long)err,
           status ? status : SENSORARRAY_NA);
    sensorarrayLogControlGpio(stage, label);
}

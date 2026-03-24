#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#include "sensorarrayTypes.h"

void sensorarrayLogSetAdsState(bool adsReady, bool adsRefReady);

void sensorarrayLogDbgExtraReset(void);
void sensorarrayLogDbgExtraSetMux(uint8_t muxp, uint8_t muxn);
void sensorarrayLogDbgExtraSetRefMux(uint8_t refmux);
void sensorarrayLogDbgExtraSetDiscardCount(uint8_t discardCount);
void sensorarrayLogDbgExtraCaptureCtrl(void);

const char *sensorarrayLogFmtI32(char *buf, size_t bufSize, bool valid, int32_t value);
const char *sensorarrayLogFmtU32(char *buf, size_t bufSize, bool valid, uint32_t value);
const char *sensorarrayLogFmtU8(char *buf, size_t bufSize, bool valid, uint8_t value);
const char *sensorarrayLogFmtBool(char *buf, size_t bufSize, bool valid, bool value);
const char *sensorarrayLogFmtHexU8(char *buf, size_t bufSize, bool valid, uint8_t value);
const char *sensorarrayLogFmtHexU16(char *buf, size_t bufSize, bool valid, uint16_t value);
const char *sensorarrayLogFmtI2cPort(char *buf, size_t bufSize, const BoardSupportI2cCtx_t *ctx);
const char *sensorarrayLogFmtGpioLevel(char *buf, size_t bufSize, bool valid, int level);

const char *sensorarrayLogSwSourceName(tmux1108Source_t source);
const char *sensorarrayLogSwSourceLogicalName(tmux1108Source_t source);
const char *sensorarrayLogAdsMuxName(uint8_t mux);
const char *sensorarrayLogDebugModeName(sensorarrayDebugMode_t mode);
const char *sensorarrayLogDebugPathName(sensorarrayDebugPath_t path);

const char *sensorarrayLogBuildMapLabel(char *buf,
                                        size_t bufSize,
                                        const char *routeMap,
                                        const char *fdcMap);

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
                       const char *status);

void sensorarrayLogStartup(const char *mode, esp_err_t err, const char *status, int32_t detailValue);

void sensorarrayLogStartupFdc(const char *mode,
                              const sensorarrayFdcDeviceState_t *fdcState,
                              esp_err_t err,
                              const char *status,
                              int32_t detailValue,
                              bool hasIds,
                              uint16_t manufacturerId,
                              uint16_t deviceId,
                              const char *map);

void sensorarrayLogControlGpio(const char *stage, const char *point);

void sensorarrayLogSelaRouteDecision(const char *stage,
                                     const char *label,
                                     sensorarraySelaRoute_t requestRoute,
                                     int selaWriteLevel,
                                     int selaReadLevel,
                                     bool resolvedValid,
                                     sensorarraySelaRoute_t resolvedRoute);
void sensorarrayLogSelaReadbackMismatch(const char *stage, const char *label, int wroteLevel, int readLevel);

void sensorarrayLogRouteStep(const char *stage,
                             const char *label,
                             uint8_t sColumn,
                             uint8_t dLine,
                             sensorarrayDebugPath_t path,
                             tmux1108Source_t swSource,
                             bool selaGpioLevel,
                             bool selBLevel,
                             esp_err_t err,
                             const char *status);

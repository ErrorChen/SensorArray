#pragma once

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#include "sensorarrayTypes.h"

typedef struct {
    sensorarrayMeasurementMode_t mode;
    sensorarraySelaRoute_t selaRoute;
    bool selBLevel;
    tmux1108Source_t swLogicalSource;
    sensorarraySwPhysicalLevel_t swPhysicalLevel;
    bool matrixExcitationEnabled;
    sensorarrayAdsIntRefPolicy_t intRef;
    sensorarrayAdsVbiasPolicy_t vbias;
    sensorarrayAdsReferenceSource_t adsReferenceSource;
    uint8_t adsRefMux;
} sensorarrayBoardRouteProfile_t;

bool sensorarrayBoardMapAdsMuxForDLine(uint8_t dLine, uint8_t *muxp, uint8_t *muxn);

const sensorarrayRouteMap_t *sensorarrayBoardMapFindRoute(uint8_t sColumn,
                                                           uint8_t dLine,
                                                           sensorarrayPath_t path);

const sensorarrayFdcDLineMap_t *sensorarrayBoardMapFindFdcByDLine(uint8_t dLine);

size_t sensorarrayBoardMapRouteCount(void);
const sensorarrayRouteMap_t *sensorarrayBoardMapRouteAt(size_t index);

size_t sensorarrayBoardMapFdcCount(void);
const sensorarrayFdcDLineMap_t *sensorarrayBoardMapFdcAt(size_t index);

const char *sensorarrayBoardMapSelaRouteName(sensorarraySelaRoute_t route);
bool sensorarrayBoardMapSelaRouteToGpioLevel(sensorarraySelaRoute_t route, int *outLevel);
bool sensorarrayBoardMapSelaRouteFromGpioLevel(int gpioLevel, sensorarraySelaRoute_t *outRoute);
bool sensorarrayBoardMapFdcSelBLevel(bool *outLevel);

const char *sensorarrayBoardMapPathName(sensorarrayPath_t path);
sensorarrayRoutePathKind_t sensorarrayBoardMapPathToRoutePath(sensorarrayPath_t path, tmux1108Source_t swSource);
tmux1108Source_t sensorarrayBoardMapDefaultSwSource(const sensorarrayRouteMap_t *route);

bool sensorarrayBoardMapGetRouteProfile(sensorarrayMeasurementMode_t mode,
                                        sensorarrayBoardRouteProfile_t *outProfile);
bool sensorarrayBoardMapGetSafeRailMonitorProfile(
    sensorarrayBoardRouteProfile_t *outProfile);
const char *sensorarrayBoardMapMatrixExcitationName(bool enabled);

void sensorarrayBoardMapAudit(void);

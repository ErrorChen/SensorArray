#pragma once

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#include "sensorarrayTypes.h"

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

const char *sensorarrayBoardMapPathName(sensorarrayPath_t path);
sensorarrayDebugPath_t sensorarrayBoardMapPathToDebugPath(sensorarrayPath_t path, tmux1108Source_t swSource);
tmux1108Source_t sensorarrayBoardMapDefaultSwSource(const sensorarrayRouteMap_t *route);

void sensorarrayBoardMapAudit(void);

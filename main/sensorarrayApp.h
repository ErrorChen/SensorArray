#pragma once

#include <stdbool.h>

#include "tmuxSwitch.h"
#include "sensorarrayTypes.h"

typedef enum {
    SENSORARRAY_APP_MODE_PIEZO_READ = 0,
    SENSORARRAY_APP_MODE_RESISTANCE_READ,
    SENSORARRAY_APP_MODE_DEBUG,
} sensorarrayAppMode_t;

sensorarrayAppMode_t sensorarrayAppCompiledMode(void);
const char *sensorarrayAppModeName(sensorarrayAppMode_t mode);
const char *sensorarrayAppModeEntryName(sensorarrayAppMode_t mode);
const char *sensorarrayAppSwSourceName(tmux1108Source_t source);
tmux1108Source_t sensorarrayAppModeRequiredSource(sensorarrayAppMode_t mode);
const char *sensorarrayAppModeSourceReason(sensorarrayAppMode_t mode);

bool sensorarrayAppDebugModeRequiredSource(sensorarrayDebugMode_t debugMode,
                                           tmux1108Source_t *outSource,
                                           const char **outRouteMode,
                                           const char **outReason);

void sensorarrayAppRun(void);

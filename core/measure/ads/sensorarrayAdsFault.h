#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SENSORARRAY_ADS_FAULT_LINE_MAX 384u
#define SENSORARRAY_ADS_FAULT_RATE_LIMIT_US 1000000u

typedef enum {
    SENSORARRAY_ADS_FAULT_STAGE_MATRIX_ROUTE = 0,
    SENSORARRAY_ADS_FAULT_STAGE_MATRIX_READ,
    SENSORARRAY_ADS_FAULT_STAGE_MATRIX_READBACK,
    SENSORARRAY_ADS_FAULT_STAGE_MATRIX_DRDY,
    SENSORARRAY_ADS_FAULT_STAGE_BATTERY_GAP,
    SENSORARRAY_ADS_FAULT_STAGE_BATTERY_RESTORE,
    SENSORARRAY_ADS_FAULT_STAGE_RAIL_MONITOR,
    SENSORARRAY_ADS_FAULT_STAGE_PROFILE_TRANSITION,
    SENSORARRAY_ADS_FAULT_STAGE_RECOVERY_START,
    SENSORARRAY_ADS_FAULT_STAGE_RECOVERY_ATTEMPT,
    SENSORARRAY_ADS_FAULT_STAGE_RECOVERY_RESUME,
    SENSORARRAY_ADS_FAULT_STAGE_RECOVERY_FAILED,
    SENSORARRAY_ADS_FAULT_STAGE_UNKNOWN,
} sensorarrayAdsFaultStage_t;

#define SENSORARRAY_ADS_FAULT_STAGE_COUNT \
    (SENSORARRAY_ADS_FAULT_STAGE_UNKNOWN + 1u)

typedef enum {
    SENSORARRAY_ADS_FAULT_OUTCOME_NONE = 0,
    SENSORARRAY_ADS_FAULT_OUTCOME_STARTED,
    SENSORARRAY_ADS_FAULT_OUTCOME_ATTEMPT,
    SENSORARRAY_ADS_FAULT_OUTCOME_RESUMED,
    SENSORARRAY_ADS_FAULT_OUTCOME_FAILED,
} sensorarrayAdsFaultOutcome_t;

typedef struct {
    sensorarrayAdsFaultStage_t stage;
    esp_err_t err;
    uint32_t bootId;
    uint32_t bootCount;
    uint32_t seq;
    uint32_t modeGeneration;
    uint32_t profileGeneration;
    uint32_t rowGeneration;
    uint32_t rowRequestId;
    uint32_t attempt;
    sensorarrayAdsFaultOutcome_t outcome;
    const char *mode;
    const char *profile;
    const char *route;
    const char *owner;
    const char *reference;
    uint32_t drdyGeneration;
    uint32_t configGeneration;
    int32_t railUv;
    int32_t restoreExpected;
    int32_t restoreActual;
    bool railValid;
    bool restoreExpectedValid;
    bool restoreActualValid;
} sensorarrayAdsFaultEvent_t;

const char *sensorarrayAdsFaultStageName(sensorarrayAdsFaultStage_t stage);
const char *sensorarrayAdsFaultOutcomeName(sensorarrayAdsFaultOutcome_t outcome);

size_t sensorarrayAdsFaultFormat(const sensorarrayAdsFaultEvent_t *event,
                                 char *buffer,
                                 size_t bufferSize);

typedef void (*sensorarrayAdsFaultSink_t)(const char *line,
                                          size_t length,
                                          void *context);

size_t sensorarrayAdsFaultEmit(const sensorarrayAdsFaultEvent_t *event,
                               uint64_t nowUs,
                               sensorarrayAdsFaultSink_t sink,
                               void *context);

#ifdef __cplusplus
}
#endif

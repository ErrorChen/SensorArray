#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#include "sensorarrayAdsMath.h"
#include "sensorarrayBoardMap.h"
#include "sensorarrayTypes.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    sensorarrayMeasurementMode_t mode;
    sensorarrayBoardRouteProfile_t profile;
    uint32_t generation;
    uint32_t lastError;
    uint64_t transitionDurationUs;
    uint8_t row;
    uint8_t power;
    uint8_t mode2;
    uint8_t inpmux;
    uint8_t refmux;
    uint8_t pgaGain;
    int32_t avddUv;
    int32_t avssUv;
    uint32_t railAgeFrames;
    bool railValid;
    bool gpioReadbackValid;
    bool adsReadbackValid;
    bool safe;
} sensorarrayRouteSnapshot_t;

typedef struct {
    sensorarrayState_t *state;
    volatile uint32_t snapshotVersion;
    sensorarrayRouteSnapshot_t snapshot;
} sensorarrayRouteController_t;

esp_err_t sensorarrayRouteControllerInit(sensorarrayRouteController_t *controller,
                                         sensorarrayState_t *state);
esp_err_t sensorarrayRouteControllerEnterSafe(sensorarrayRouteController_t *controller,
                                              const char *reason);
esp_err_t sensorarrayRouteControllerApplyMode(sensorarrayRouteController_t *controller,
                                              sensorarrayMeasurementMode_t mode,
                                              const sensorarrayAdsRailSplit_t *rail,
                                              uint64_t *outTransitionDurationUs);
esp_err_t sensorarrayRouteControllerSelectRow(sensorarrayRouteController_t *controller,
                                             uint8_t row);
bool sensorarrayRouteControllerCopySnapshot(const sensorarrayRouteController_t *controller,
                                            sensorarrayRouteSnapshot_t *outSnapshot);
void sensorarrayRouteControllerUpdateRailSnapshot(sensorarrayRouteController_t *controller,
                                                  const sensorarrayAdsRailSplit_t *rail);

#ifdef __cplusplus
}
#endif

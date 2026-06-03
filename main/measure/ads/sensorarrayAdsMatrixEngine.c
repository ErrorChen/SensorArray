#include "sensorarrayAdsMatrixEngine.h"

#include "sensorarrayFrameBuilder.h"

esp_err_t sensorarrayAdsMatrixEngineInit(sensorarrayAdsMatrixEngine_t *engine,
                                         sensorarrayState_t *state)
{
    if (!engine || !state) {
        return ESP_ERR_INVALID_ARG;
    }
    *engine = (sensorarrayAdsMatrixEngine_t){
        .state = state,
    };
    return ESP_OK;
}

esp_err_t sensorarrayAdsMatrixEngineReadFrame(sensorarrayAdsMatrixEngine_t *engine,
                                              const sensorarrayScanPlan_t *plan,
                                              sensorarrayFrame_t *frame)
{
    (void)engine;
    (void)plan;
    sensorarrayFrameBuilderInitInvalid(frame);
    return ESP_ERR_NOT_SUPPORTED;
}


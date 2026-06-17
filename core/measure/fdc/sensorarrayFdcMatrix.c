#include "sensorarrayFdcMatrix.h"

#include "sensorarrayFdcInternal.h"
#include "sensorarrayFdcSweep.h"
#include "sensorarrayMeasure.h"

esp_err_t sensorarrayFdcMatrixEngineInit(sensorarrayFdcMatrixEngine_t *engine,
                                         sensorarrayState_t *state)
{
    if (!engine || !state) {
        return ESP_ERR_INVALID_ARG;
    }

    *engine = (sensorarrayFdcMatrixEngine_t){
        .state = state,
    };
    return ESP_OK;
}

esp_err_t sensorarrayFdcMatrixEngineRunBootSweep(sensorarrayFdcMatrixEngine_t *engine,
                                                 const sensorarrayScanPlan_t *plan,
                                                 sensorarrayFdcBootSummary_t *outSummary)
{
    (void)plan;
    if (!engine || !engine->state) {
        return ESP_ERR_INVALID_ARG;
    }

    sensorarrayFdcBootSummary_t summary = {0};
    esp_err_t err = sensorarrayFdcSweepRunBoot(engine->state, &summary);
    engine->bootSummary = summary;
    engine->bootSweepOk = (err == ESP_OK && summary.quality == SENSORARRAY_FDC_BOOT_QUALITY_OK);
    engine->diagnosticMode = (err != ESP_OK || summary.quality == SENSORARRAY_FDC_BOOT_QUALITY_FAIL);
    if (outSummary) {
        *outSummary = summary;
    }
    return err;
}

esp_err_t sensorarrayFdcMatrixEngineReadFrame(sensorarrayFdcMatrixEngine_t *engine,
                                              const sensorarrayScanPlan_t *plan,
                                              sensorarrayFrame_t *frame)
{
    if (!engine || !engine->state || !frame) {
        return ESP_ERR_INVALID_ARG;
    }

    if (plan) {
        return sensorarrayMeasureReadFdcMatrixFrameSnapshot(engine->state,
                                                           frame,
                                                           &plan->configSnapshot);
    }
    return sensorarrayMeasureReadFdcMatrixFrame(engine->state, frame);
}

esp_err_t sensorarrayFdcMatrixEngineRunFullRescue(sensorarrayFdcMatrixEngine_t *engine,
                                                  const char *reason)
{
    if (!engine || !engine->state) {
        return ESP_ERR_INVALID_ARG;
    }

    return sensorarrayFdcSweepRunFullRescueAll(engine->state,
                                              reason ? reason : "runtime_full_rescue");
}

void sensorarrayFdcMatrixEngineSetDiagnosticMode(sensorarrayFdcMatrixEngine_t *engine,
                                                 bool enabled)
{
    if (engine) {
        engine->diagnosticMode = enabled;
    }
}

bool sensorarrayFdcMatrixEngineDiagnosticMode(const sensorarrayFdcMatrixEngine_t *engine)
{
    return engine && engine->diagnosticMode;
}

const sensorarrayFdcBootSummary_t *sensorarrayFdcMatrixEngineBootSummary(const sensorarrayFdcMatrixEngine_t *engine)
{
    return engine ? &engine->bootSummary : NULL;
}

sensorarrayState_t *sensorarrayFdcMatrixEngineState(sensorarrayFdcMatrixEngine_t *engine)
{
    return engine ? engine->state : NULL;
}

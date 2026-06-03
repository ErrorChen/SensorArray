#include "sensorarrayFdcMatrixEngine.h"

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
                                                 const sensorarrayScanPlan_t *plan)
{
    (void)plan;
    if (!engine || !engine->state) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = sensorarrayFdcSweepRunBoot(engine->state);
    engine->bootSweepOk = (err == ESP_OK);
    engine->diagnosticMode = (err != ESP_OK);
    return err;
}

esp_err_t sensorarrayFdcMatrixEngineReadFrame(sensorarrayFdcMatrixEngine_t *engine,
                                              const sensorarrayScanPlan_t *plan,
                                              sensorarrayFrame_t *frame)
{
    (void)plan;
    if (!engine || !engine->state || !frame) {
        return ESP_ERR_INVALID_ARG;
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

sensorarrayState_t *sensorarrayFdcMatrixEngineState(sensorarrayFdcMatrixEngine_t *engine)
{
    return engine ? engine->state : NULL;
}


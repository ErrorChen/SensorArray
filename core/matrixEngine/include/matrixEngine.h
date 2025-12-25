#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "ads126xAdc.h"
#include "esp_err.h"
#include "fdc2214Cap.h"
#include "tmuxSwitch.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    MATRIX_ENGINE_IO_READ = 0,
    MATRIX_ENGINE_IO_WRITE = 1,
} matrixEngineIo_t;

typedef enum {
    MATRIX_ENGINE_MEASURE_VOLTAGE_UV = 0,
    MATRIX_ENGINE_MEASURE_CAP_RAW = 1,
    MATRIX_ENGINE_MEASURE_RESISTANCE_MOHM = 2,
} matrixEngineMeasure_t;

typedef struct {
    uint16_t row;
    uint16_t col;
    uint16_t rows;
    uint16_t cols;
} matrixEngineRegion_t;

typedef esp_err_t (*matrixEngineSelectRowFn)(uint16_t row, void *userCtx);
typedef esp_err_t (*matrixEngineSelectColGroupFn)(uint16_t group, void *userCtx);
typedef esp_err_t (*matrixEngineDriveFn)(tmux1108Source_t source, void *userCtx);

typedef struct {
    ads126xAdcHandle_t *adc;
    Fdc2214CapDevice_t *cap;
    Fdc2214CapDevice_t *const *capDevices;
    size_t capDeviceCount;
    const uint8_t *adcMuxPByCol;
    const uint8_t *adcMuxNByCol;
    const uint8_t *capDeviceIndexByCol;
    const uint8_t *capChannelByCol;
    uint16_t colGroupSize;
    uint8_t oversample;
    uint32_t resistanceRefOhms;
    uint32_t resistanceExcitationUv;
    matrixEngineSelectRowFn selectRow;
    matrixEngineSelectColGroupFn selectColGroup;
    matrixEngineDriveFn drive;
    void *userCtx;
} matrixEngineConfig_t;

typedef struct {
    matrixEngineIo_t io;
    matrixEngineMeasure_t measure;
    tmux1108Source_t driveSource;
} matrixEngineRequest_t;

esp_err_t matrixEngineInit(const matrixEngineConfig_t *cfg);
void matrixEngineDeinit(void);

// outValues is required for MATRIX_ENGINE_IO_READ; values are row-major.
esp_err_t matrixEngineRegionIo(const matrixEngineRegion_t *region,
                               const matrixEngineRequest_t *req,
                               int32_t *outValues,
                               size_t outCount);

#ifdef __cplusplus
}
#endif

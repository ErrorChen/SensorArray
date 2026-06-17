#pragma once

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#include "esp_err.h"

#define SENSORARRAY_SCAN_CONFIG_MAX_ROWS 8u
#define SENSORARRAY_SCAN_CONFIG_COLS_PER_ROW 8u

typedef struct {
    uint8_t rows;
    uint8_t cells;
    uint8_t rowMask;
    uint32_t generation;
    uint32_t requestId;
} sensorarrayFrameConfigSnapshot_t;

typedef struct {
    uint32_t requestId;
    uint8_t oldRows;
    uint8_t requestedRows;
    uint32_t activeGeneration;
} sensorarrayScanConfigRequestResult_t;

typedef struct {
    bool applied;
    uint32_t requestId;
    uint8_t oldRows;
    uint8_t newRows;
    uint32_t generation;
} sensorarrayScanConfigApplyResult_t;

esp_err_t sensorarrayScanConfigInit(void);
esp_err_t sensorarrayScanConfigRequestRows(uint8_t rows,
                                           sensorarrayScanConfigRequestResult_t *outResult);
uint8_t sensorarrayScanConfigGetActiveRows(void);
uint8_t sensorarrayScanConfigGetPendingRows(void);
sensorarrayFrameConfigSnapshot_t sensorarrayScanConfigGetFrameSnapshot(void);
void sensorarrayScanConfigApplyPendingAtFrameBoundary(sensorarrayScanConfigApplyResult_t *outResult);
esp_err_t sensorarrayScanConfigHandleCommand(const char *command,
                                             size_t length,
                                             char *response,
                                             size_t responseSize);

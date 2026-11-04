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
    uint32_t lastRequestId;
    uint32_t lastError;
    bool lastTerminal;
    bool lastApplied;
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

/* 本轮新增: one arbiter/ID namespace for MODE, ROWS, ROWMODES across sources.
 * A busy configuration request is rejected before ACK, never superseded. */
uint32_t sensorarrayScanConfigAllocateRequestId(void);
esp_err_t sensorarrayScanConfigBeginConfigurationRequest(uint32_t *outRequestId);
void sensorarrayScanConfigEndConfigurationRequest(uint32_t requestId);

/* 本轮新增: transport writes stage through the Core 1 command mailbox. */
typedef esp_err_t (*sensorarrayScanConfigRequestCallback_t)(uint8_t rows,
    sensorarrayScanConfigRequestResult_t *outResult);
void sensorarrayScanConfigSetRequestCallback(sensorarrayScanConfigRequestCallback_t callback);
esp_err_t sensorarrayScanConfigStageAcceptedRows(uint8_t rows, uint32_t requestId);

void sensorarrayScanConfigRecordRejected(uint32_t requestId, uint32_t errorCode);

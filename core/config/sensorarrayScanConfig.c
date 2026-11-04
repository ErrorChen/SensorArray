#include "sensorarrayScanConfig.h"

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "sdkconfig.h"

#ifndef CONFIG_SENSORARRAY_DEFAULT_ACTIVE_ROWS
#define CONFIG_SENSORARRAY_DEFAULT_ACTIVE_ROWS 8
#endif

static uint8_t sensorarrayScanConfigDefaultRows(void)
{
    uint8_t rows = (uint8_t)CONFIG_SENSORARRAY_DEFAULT_ACTIVE_ROWS;
    return (rows >= 1u && rows <= 8u) ? rows : 8u;
}

static portMUX_TYPE s_scanConfigMux = portMUX_INITIALIZER_UNLOCKED;
static uint8_t s_activeRows = CONFIG_SENSORARRAY_DEFAULT_ACTIVE_ROWS;
static uint8_t s_pendingRows = CONFIG_SENSORARRAY_DEFAULT_ACTIVE_ROWS;
static bool s_pendingRowsValid;
static uint32_t s_nextRequestId = 1u;
static uint32_t s_pendingRequestId;
static uint32_t s_appliedRequestId;
static uint32_t s_generation = 1u;
static uint32_t s_configurationRequestId;
static uint32_t s_lastRowsRequestId;
static uint32_t s_lastRowsError;
static bool s_lastRowsApplied;
static bool s_lastRowsTerminal;
static bool s_initialized;
static sensorarrayScanConfigRequestCallback_t s_requestCallback;

static uint8_t sensorarrayScanConfigRowMask(uint8_t rows)
{
    if (rows >= SENSORARRAY_SCAN_CONFIG_MAX_ROWS) {
        return 0xFFu;
    }
    return (uint8_t)((1u << rows) - 1u);
}

static sensorarrayFrameConfigSnapshot_t sensorarrayScanConfigMakeSnapshotLocked(void)
{
    uint8_t rows = s_activeRows;
    if (rows < 1u || rows > SENSORARRAY_SCAN_CONFIG_MAX_ROWS) {
        rows = sensorarrayScanConfigDefaultRows();
    }
    return (sensorarrayFrameConfigSnapshot_t){
        .rows = rows,
        .cells = (uint8_t)(rows * SENSORARRAY_SCAN_CONFIG_COLS_PER_ROW),
        .rowMask = sensorarrayScanConfigRowMask(rows),
        .generation = s_generation,
        .requestId = s_appliedRequestId,
        .lastRequestId = s_lastRowsRequestId, .lastError = s_lastRowsError,
        .lastTerminal = s_lastRowsTerminal, .lastApplied = s_lastRowsApplied,
    };
}

esp_err_t sensorarrayScanConfigInit(void)
{
    portENTER_CRITICAL(&s_scanConfigMux);
    if (s_initialized) {
        portEXIT_CRITICAL(&s_scanConfigMux);
        return ESP_OK;
    }
    s_initialized = true;
    uint8_t rows = sensorarrayScanConfigDefaultRows();
    s_activeRows = rows;
    s_pendingRows = rows;
    s_pendingRowsValid = false;
    s_nextRequestId = 1u;
    s_pendingRequestId = 0u;
    s_appliedRequestId = 0u;
    s_generation = 1u;
    portEXIT_CRITICAL(&s_scanConfigMux);
    return ESP_OK;
}

esp_err_t sensorarrayScanConfigRequestRows(uint8_t rows,
                                           sensorarrayScanConfigRequestResult_t *outResult)
{
    if (rows < 1u || rows > SENSORARRAY_SCAN_CONFIG_MAX_ROWS) {
        return ESP_ERR_INVALID_ARG;
    }
    portENTER_CRITICAL(&s_scanConfigMux);
    if (s_configurationRequestId != 0u) {
        portEXIT_CRITICAL(&s_scanConfigMux);
        return ESP_ERR_INVALID_STATE;
    }
    uint32_t requestId = s_nextRequestId++;
    if (s_nextRequestId == 0u) {
        s_nextRequestId = 1u;
    }
    s_configurationRequestId = requestId;
    uint8_t oldRows = s_activeRows;
    s_pendingRows = rows;
    s_pendingRowsValid = true;
    s_pendingRequestId = requestId;
    uint32_t activeGeneration = s_generation;
    if (outResult) {
        *outResult = (sensorarrayScanConfigRequestResult_t){
            .requestId = requestId,
            .oldRows = oldRows,
            .requestedRows = rows,
            .activeGeneration = activeGeneration,
        };
    }
    portEXIT_CRITICAL(&s_scanConfigMux);
    return ESP_OK;
}

uint8_t sensorarrayScanConfigGetActiveRows(void)
{
    portENTER_CRITICAL(&s_scanConfigMux);
    uint8_t rows = s_activeRows;
    portEXIT_CRITICAL(&s_scanConfigMux);
    return rows;
}

uint8_t sensorarrayScanConfigGetPendingRows(void)
{
    portENTER_CRITICAL(&s_scanConfigMux);
    uint8_t rows = s_pendingRows;
    portEXIT_CRITICAL(&s_scanConfigMux);
    return rows;
}

sensorarrayFrameConfigSnapshot_t sensorarrayScanConfigGetFrameSnapshot(void)
{
    portENTER_CRITICAL(&s_scanConfigMux);
    sensorarrayFrameConfigSnapshot_t snapshot = sensorarrayScanConfigMakeSnapshotLocked();
    portEXIT_CRITICAL(&s_scanConfigMux);
    return snapshot;
}

void sensorarrayScanConfigApplyPendingAtFrameBoundary(sensorarrayScanConfigApplyResult_t *outResult)
{
    sensorarrayScanConfigApplyResult_t result = {0};

    portENTER_CRITICAL(&s_scanConfigMux);
    if (s_pendingRowsValid) {
        uint8_t oldRows = s_activeRows;
        s_activeRows = s_pendingRows;
        s_appliedRequestId = s_pendingRequestId;
        s_lastRowsRequestId = s_appliedRequestId;
        s_lastRowsError = 0u;
        s_lastRowsTerminal = true;
        s_lastRowsApplied = true;
        s_pendingRowsValid = false;
        if (s_configurationRequestId == s_appliedRequestId) {
            s_configurationRequestId = 0u;
        }
        s_generation++;
        if (s_generation == 0u) {
            s_generation = 1u;
        }
        result = (sensorarrayScanConfigApplyResult_t){
            .applied = true,
            .requestId = s_appliedRequestId,
            .oldRows = oldRows,
            .newRows = s_activeRows,
            .generation = s_generation,
        };
    }
    portEXIT_CRITICAL(&s_scanConfigMux);

    if (outResult) {
        *outResult = result;
    }
}

static void sensorarrayScanConfigNormalize(char *text)
{
    char *read = text;
    while (*read && isspace((unsigned char)*read)) {
        read++;
    }
    char *write = text;
    while (*read && *read != '\r' && *read != '\n') {
        *write++ = (char)toupper((unsigned char)*read++);
    }
    while (write > text && isspace((unsigned char)write[-1])) {
        write--;
    }
    *write = '\0';
}

esp_err_t sensorarrayScanConfigHandleCommand(const char *command,
                                             size_t length,
                                             char *response,
                                             size_t responseSize)
{
    if (!command || length == 0u || length >= 48u || !response || responseSize == 0u) {
        return ESP_ERR_INVALID_ARG;
    }
    char text[48];
    memcpy(text, command, length);
    text[length] = '\0';
    sensorarrayScanConfigNormalize(text);
    if (strcmp(text, "ROWS?") == 0) {
        portENTER_CRITICAL(&s_scanConfigMux);
        uint8_t activeRows = s_activeRows;
        uint8_t pendingRows = s_pendingRowsValid ? s_pendingRows : s_activeRows;
        uint32_t pendingRequestId = s_pendingRowsValid ? s_pendingRequestId : 0u;
        uint32_t generation = s_generation;
        uint32_t appliedRequestId = s_appliedRequestId;
        portEXIT_CRITICAL(&s_scanConfigMux);
        snprintf(response,
                 responseSize,
                 "ROWS,active=%u,pending=%u,requestId=%lu,appliedId=%lu,generation=%lu\n",
                 (unsigned)activeRows,
                 (unsigned)pendingRows,
                 (unsigned long)pendingRequestId,
                 (unsigned long)appliedRequestId,
                 (unsigned long)generation);
        return ESP_OK;
    }
    const char *value = NULL;
    if (strncmp(text, "ROWS=", 5u) == 0) {
        value = &text[5];
    } else if (strncmp(text, "ROWLIMIT=", 9u) == 0) {
        value = &text[9];
    } else if (strncmp(text, "SCANROWS=", 9u) == 0) {
        value = &text[9];
    } else {
        return ESP_ERR_NOT_SUPPORTED;
    }
    char *end = NULL;
    unsigned long rows = strtoul(value, &end, 10);
    if (end == value || *end != '\0') {
        snprintf(response, responseSize, "ERR,cmd=ROWS,reason=parse\n");
        return ESP_ERR_INVALID_ARG;
    }
    if (rows < 1u || rows > SENSORARRAY_SCAN_CONFIG_MAX_ROWS) {
        snprintf(response, responseSize, "ERR,cmd=ROWS,reason=range,min=1,max=8\n");
        return ESP_ERR_INVALID_ARG;
    }
    sensorarrayScanConfigRequestResult_t request = {0};
    portENTER_CRITICAL(&s_scanConfigMux);
    sensorarrayScanConfigRequestCallback_t callback = s_requestCallback;
    portEXIT_CRITICAL(&s_scanConfigMux);
    esp_err_t err = callback ? callback((uint8_t)rows, &request) : ESP_ERR_INVALID_STATE;
    if (err != ESP_OK) {
        snprintf(response, responseSize, "ERR,cmd=ROWS,reason=config_busy\n");
        return err;
    }
    snprintf(response,
             responseSize,
             "RCMD,id=%lu,old=%u,req=%lu,status=accepted,generation=%lu\n",
             (unsigned long)request.requestId,
             (unsigned)request.oldRows,
             rows,
             (unsigned long)request.activeGeneration);
    return ESP_OK;
}

uint32_t sensorarrayScanConfigAllocateRequestId(void)
{
    portENTER_CRITICAL(&s_scanConfigMux);
    uint32_t id = s_nextRequestId++;
    if (s_nextRequestId == 0u) { s_nextRequestId = 1u; }
    portEXIT_CRITICAL(&s_scanConfigMux);
    return id;
}

esp_err_t sensorarrayScanConfigBeginConfigurationRequest(uint32_t *outRequestId)
{
    if (!outRequestId) { return ESP_ERR_INVALID_ARG; }
    portENTER_CRITICAL(&s_scanConfigMux);
    if (s_configurationRequestId != 0u) {
        portEXIT_CRITICAL(&s_scanConfigMux);
        return ESP_ERR_INVALID_STATE;
    }
    s_configurationRequestId = s_nextRequestId++;
    if (s_nextRequestId == 0u) { s_nextRequestId = 1u; }
    *outRequestId = s_configurationRequestId;
    portEXIT_CRITICAL(&s_scanConfigMux);
    return ESP_OK;
}

void sensorarrayScanConfigEndConfigurationRequest(uint32_t requestId)
{
    portENTER_CRITICAL(&s_scanConfigMux);
    if (requestId != 0u && s_configurationRequestId == requestId) {
        s_configurationRequestId = 0u;
    }
    portEXIT_CRITICAL(&s_scanConfigMux);
}

void sensorarrayScanConfigSetRequestCallback(sensorarrayScanConfigRequestCallback_t callback)
{
    portENTER_CRITICAL(&s_scanConfigMux);
    s_requestCallback = callback;
    portEXIT_CRITICAL(&s_scanConfigMux);
}

esp_err_t sensorarrayScanConfigStageAcceptedRows(uint8_t rows, uint32_t requestId)
{
    if (rows < 1u || rows > 8u || requestId == 0u) { return ESP_ERR_INVALID_ARG; }
    portENTER_CRITICAL(&s_scanConfigMux);
    if (s_configurationRequestId != requestId || s_pendingRowsValid) {
        portEXIT_CRITICAL(&s_scanConfigMux);
        return ESP_ERR_INVALID_STATE;
    }
    s_pendingRows = rows;
    s_pendingRowsValid = true;
    s_pendingRequestId = requestId;
    portEXIT_CRITICAL(&s_scanConfigMux);
    return ESP_OK;
}

void sensorarrayScanConfigRecordRejected(uint32_t requestId, uint32_t errorCode)
{
    portENTER_CRITICAL(&s_scanConfigMux);
    s_lastRowsRequestId = requestId;
    s_lastRowsError = errorCode;
    s_lastRowsTerminal = true;
    s_lastRowsApplied = false;
    portEXIT_CRITICAL(&s_scanConfigMux);
}

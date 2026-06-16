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

esp_err_t sensorarrayScanConfigInit(void)
{
    portENTER_CRITICAL(&s_scanConfigMux);
    uint8_t rows = sensorarrayScanConfigDefaultRows();
    s_activeRows = rows;
    s_pendingRows = rows;
    portEXIT_CRITICAL(&s_scanConfigMux);
    return ESP_OK;
}

esp_err_t sensorarrayScanConfigRequestRows(uint8_t rows)
{
    if (rows < 1u || rows > 8u) {
        return ESP_ERR_INVALID_ARG;
    }
    portENTER_CRITICAL(&s_scanConfigMux);
    s_pendingRows = rows;
    portEXIT_CRITICAL(&s_scanConfigMux);
    printf("SCANCFG,rows=%u,cells=%u,apply=next_frame\n",
           (unsigned)rows, (unsigned)(rows * 8u));
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

void sensorarrayScanConfigApplyPendingAtFrameBoundary(void)
{
    portENTER_CRITICAL(&s_scanConfigMux);
    s_activeRows = s_pendingRows;
    portEXIT_CRITICAL(&s_scanConfigMux);
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
        uint8_t rows = sensorarrayScanConfigGetPendingRows();
        snprintf(response, responseSize, "ACK,cmd=ROWS,value=%u,cells=%u\n",
                 (unsigned)rows, (unsigned)(rows * 8u));
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
    if (rows < 1u || rows > 8u) {
        snprintf(response, responseSize, "ERR,cmd=ROWS,reason=range,min=1,max=8\n");
        return ESP_ERR_INVALID_ARG;
    }
    (void)sensorarrayScanConfigRequestRows((uint8_t)rows);
    snprintf(response, responseSize, "ACK,cmd=ROWS,value=%lu,cells=%lu\n",
             rows, rows * 8u);
    return ESP_OK;
}

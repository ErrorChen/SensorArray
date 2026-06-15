#pragma once

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

esp_err_t sensorarrayScanConfigInit(void);
esp_err_t sensorarrayScanConfigRequestRows(uint8_t rows);
uint8_t sensorarrayScanConfigGetActiveRows(void);
uint8_t sensorarrayScanConfigGetPendingRows(void);
void sensorarrayScanConfigApplyPendingAtFrameBoundary(void);
esp_err_t sensorarrayScanConfigHandleCommand(const char *command,
                                             size_t length,
                                             char *response,
                                             size_t responseSize);

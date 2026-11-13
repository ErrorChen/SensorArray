#pragma once
#include <stddef.h>
#include <stdbool.h>
#include "esp_err.h"
/* 本轮新增: read-only USB diagnostics and explicit ROM restart request. */
esp_err_t sensorarrayUsbControlHandle(const char *command, char *response, size_t size);

bool sensorarrayUsbControlHandles(const char *command);

const char *sensorarrayUsbControlCapabilities(void);

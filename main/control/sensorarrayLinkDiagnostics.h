#pragma once
#include <stddef.h>
#include "esp_err.h"
/* 本轮新增: actual negotiated state and local SDK service observations. */
esp_err_t sensorarrayLinkDiagnosticsFormat(const char *command, char *response, size_t size);

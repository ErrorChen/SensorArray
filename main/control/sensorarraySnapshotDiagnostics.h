#pragma once
#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"

/* 本轮新增: read-only per-layer protected-frame accounting, not delivery ACKs. */
bool sensorarraySnapshotDiagnosticsHandles(const char *command);
esp_err_t sensorarraySnapshotDiagnosticsFormat(const char *command,
                                               char *response, size_t size);

#pragma once
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

/* 本轮新增: low-frequency, read-only heap observations and the one scan-task
 * allocation bracket. Counts are bytes; separate heaps are not atomic. */
void sensorarrayMemoryDiagnosticsScanBegin(uint32_t stackBytes);
void sensorarrayMemoryDiagnosticsScanEnd(esp_err_t result);
esp_err_t sensorarrayMemoryDiagnosticsFormat(char *response, size_t size);

/* 本轮新增: register the official allocation-failure observer once per boot,
 * after identity is known and before network/scan startup. It never changes
 * allocation, reset or retry policy. Repeated calls retain the first origin
 * and registration result. MEMFAIL? is a separate read-only bounded reply. */
esp_err_t sensorarrayMemoryDiagnosticsInit(uint64_t sessionId, uint32_t bootId,
    const uint8_t elfSha256[32]);
esp_err_t sensorarrayMemoryDiagnosticsFailureFormat(char *response, size_t size);

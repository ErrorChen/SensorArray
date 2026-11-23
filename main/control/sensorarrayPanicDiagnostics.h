#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"
#include "esp_system.h"

/* 本轮新增: last retained panic evidence, never a crash-injection command.
 * Call once, before command RX; ELF bytes are copied in normal task context.
 * PANIC? is metadata; PANIC?1/2 contain at most 96 detail bytes as hex. */
void sensorarrayPanicDiagnosticsInit(uint64_t sessionId, uint32_t bootId,
    const uint8_t elfSha256[32], esp_reset_reason_t resetReason);
bool sensorarrayPanicDiagnosticsHandles(const char *command);
esp_err_t sensorarrayPanicDiagnosticsFormat(const char *command,
    char *response, size_t size);

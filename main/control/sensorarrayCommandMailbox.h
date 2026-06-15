#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    SENSORARRAY_COMMAND_BLE_CAP_PERIOD = 0,
    SENSORARRAY_COMMAND_TRACE_ENABLE,
    SENSORARRAY_COMMAND_CALIBRATE_ZERO,
    SENSORARRAY_COMMAND_CALIBRATE_RAIL,
    SENSORARRAY_COMMAND_CALIBRATE_ALL,
} sensorarrayCommandType_t;

typedef struct {
    sensorarrayCommandType_t type;
    uint32_t value;
} sensorarrayCommand_t;

esp_err_t sensorarrayCommandMailboxInit(void);
esp_err_t sensorarrayCommandMailboxPostText(const uint8_t *text, size_t length);
bool sensorarrayCommandMailboxTryReceive(sensorarrayCommand_t *outCommand);
void sensorarrayCommandMailboxCommit(const sensorarrayCommand_t *command);
uint32_t sensorarrayCommandMailboxGetBleCapPeriod(void);
bool sensorarrayCommandMailboxTraceEnabled(void);
const char *sensorarrayCommandMailboxTypeName(sensorarrayCommandType_t type);

#ifdef __cplusplus
}
#endif

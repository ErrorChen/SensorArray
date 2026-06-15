#include "sensorarrayCommandMailbox.h"

#include <ctype.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "sensorarrayConfig.h"

#define SENSORARRAY_COMMAND_QUEUE_LENGTH 8u
#define SENSORARRAY_COMMAND_TEXT_MAX 48u
#define SENSORARRAY_BLE_CAP_PERIOD_MAX 100000u

static StaticQueue_t s_commandQueueStruct;
static uint8_t s_commandQueueStorage[SENSORARRAY_COMMAND_QUEUE_LENGTH *
                                     sizeof(sensorarrayCommand_t)];
static QueueHandle_t s_commandQueue;
static portMUX_TYPE s_commandStateMux = portMUX_INITIALIZER_UNLOCKED;
static uint32_t s_bleCapPeriod;
static bool s_traceEnabled;

static void sensorarrayCommandNormalize(char *text)
{
    if (!text) {
        return;
    }

    char *readCursor = text;
    while (*readCursor != '\0' && isspace((unsigned char)*readCursor)) {
        readCursor++;
    }

    char *writeCursor = text;
    while (*readCursor != '\0') {
        unsigned char character = (unsigned char)*readCursor++;
        if (character == '\r' || character == '\n') {
            break;
        }
        *writeCursor++ = (char)toupper(character);
    }
    while (writeCursor > text && isspace((unsigned char)writeCursor[-1])) {
        writeCursor--;
    }
    *writeCursor = '\0';
}

static esp_err_t sensorarrayCommandParseUnsigned(const char *value,
                                                  uint32_t maximum,
                                                  uint32_t *outValue)
{
    if (!value || !outValue || *value == '\0') {
        return ESP_ERR_INVALID_ARG;
    }

    char *end = NULL;
    unsigned long parsed = strtoul(value, &end, 10);
    if (end == value || *end != '\0' || parsed > maximum) {
        return ESP_ERR_INVALID_ARG;
    }
    *outValue = (uint32_t)parsed;
    return ESP_OK;
}

static esp_err_t sensorarrayCommandParse(const uint8_t *text,
                                         size_t length,
                                         sensorarrayCommand_t *outCommand)
{
    if (!text || !outCommand || length == 0u || length >= SENSORARRAY_COMMAND_TEXT_MAX) {
        return ESP_ERR_INVALID_ARG;
    }

    char commandText[SENSORARRAY_COMMAND_TEXT_MAX];
    memcpy(commandText, text, length);
    commandText[length] = '\0';
    sensorarrayCommandNormalize(commandText);

    const char *value = NULL;
    if (strncmp(commandText, "BLECAP=", 7u) == 0) {
        value = &commandText[7];
    } else if (strncmp(commandText, "CAP=", 4u) == 0) {
        value = &commandText[4];
    }
    if (value) {
        uint32_t period = 0u;
        esp_err_t err = sensorarrayCommandParseUnsigned(value,
                                                         SENSORARRAY_BLE_CAP_PERIOD_MAX,
                                                         &period);
        if (err != ESP_OK) {
            return err;
        }
        *outCommand = (sensorarrayCommand_t){
            .type = SENSORARRAY_COMMAND_BLE_CAP_PERIOD,
            .value = period,
        };
        return ESP_OK;
    }

    if (strncmp(commandText, "TRACE=", 6u) == 0) {
        uint32_t enabled = 0u;
        esp_err_t err = sensorarrayCommandParseUnsigned(&commandText[6], 1u, &enabled);
        if (err != ESP_OK) {
            return err;
        }
        *outCommand = (sensorarrayCommand_t){
            .type = SENSORARRAY_COMMAND_TRACE_ENABLE,
            .value = enabled,
        };
        return ESP_OK;
    }

    sensorarrayCommandType_t calibrationType;
    if (strcmp(commandText, "CAL=ZERO") == 0) {
        calibrationType = SENSORARRAY_COMMAND_CALIBRATE_ZERO;
    } else if (strcmp(commandText, "CAL=RAIL") == 0) {
        calibrationType = SENSORARRAY_COMMAND_CALIBRATE_RAIL;
    } else if (strcmp(commandText, "CAL=ALL") == 0) {
        calibrationType = SENSORARRAY_COMMAND_CALIBRATE_ALL;
    } else {
        return ESP_ERR_NOT_SUPPORTED;
    }

    *outCommand = (sensorarrayCommand_t){
        .type = calibrationType,
        .value = 1u,
    };
    return ESP_OK;
}

esp_err_t sensorarrayCommandMailboxInit(void)
{
    if (s_commandQueue) {
        return ESP_OK;
    }

    s_commandQueue = xQueueCreateStatic(SENSORARRAY_COMMAND_QUEUE_LENGTH,
                                        sizeof(sensorarrayCommand_t),
                                        s_commandQueueStorage,
                                        &s_commandQueueStruct);
    if (!s_commandQueue) {
        return ESP_ERR_NO_MEM;
    }

    portENTER_CRITICAL(&s_commandStateMux);
    s_bleCapPeriod = CONFIG_SENSORARRAY_BLE_CAP_TEXT_EVERY_N_FRAMES;
    s_traceEnabled = false;
    portEXIT_CRITICAL(&s_commandStateMux);
    return ESP_OK;
}

esp_err_t sensorarrayCommandMailboxPostText(const uint8_t *text, size_t length)
{
    if (!s_commandQueue) {
        return ESP_ERR_INVALID_STATE;
    }

    sensorarrayCommand_t command;
    esp_err_t err = sensorarrayCommandParse(text, length, &command);
    if (err != ESP_OK) {
        return err;
    }

    if (xQueueSend(s_commandQueue, &command, 0) == pdTRUE) {
        return ESP_OK;
    }

    /* Commands are operator intent. If the host outruns Core1, discard the
     * oldest pending intent and retain the newest value instead of blocking a
     * BLE callback in the Output/System domain. */
    sensorarrayCommand_t discarded;
    (void)xQueueReceive(s_commandQueue, &discarded, 0);
    return xQueueSend(s_commandQueue, &command, 0) == pdTRUE ? ESP_OK : ESP_ERR_TIMEOUT;
}

bool sensorarrayCommandMailboxTryReceive(sensorarrayCommand_t *outCommand)
{
    return s_commandQueue && outCommand &&
           xQueueReceive(s_commandQueue, outCommand, 0) == pdTRUE;
}

void sensorarrayCommandMailboxCommit(const sensorarrayCommand_t *command)
{
    if (!command) {
        return;
    }

    portENTER_CRITICAL(&s_commandStateMux);
    if (command->type == SENSORARRAY_COMMAND_BLE_CAP_PERIOD) {
        s_bleCapPeriod = command->value;
    } else if (command->type == SENSORARRAY_COMMAND_TRACE_ENABLE) {
        s_traceEnabled = command->value != 0u;
    }
    portEXIT_CRITICAL(&s_commandStateMux);
}

uint32_t sensorarrayCommandMailboxGetBleCapPeriod(void)
{
    portENTER_CRITICAL(&s_commandStateMux);
    uint32_t period = s_bleCapPeriod;
    portEXIT_CRITICAL(&s_commandStateMux);
    return period;
}

bool sensorarrayCommandMailboxTraceEnabled(void)
{
    portENTER_CRITICAL(&s_commandStateMux);
    bool enabled = s_traceEnabled;
    portEXIT_CRITICAL(&s_commandStateMux);
    return enabled;
}

const char *sensorarrayCommandMailboxTypeName(sensorarrayCommandType_t type)
{
    switch (type) {
    case SENSORARRAY_COMMAND_BLE_CAP_PERIOD:
        return "blecap";
    case SENSORARRAY_COMMAND_TRACE_ENABLE:
        return "trace";
    case SENSORARRAY_COMMAND_CALIBRATE_ZERO:
        return "cal_zero";
    case SENSORARRAY_COMMAND_CALIBRATE_RAIL:
        return "cal_rail";
    case SENSORARRAY_COMMAND_CALIBRATE_ALL:
        return "cal_all";
    default:
        return "unknown";
    }
}

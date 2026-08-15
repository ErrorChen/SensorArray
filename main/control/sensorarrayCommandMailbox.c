#include "sensorarrayCommandMailbox.h"

#include <ctype.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "sensorarrayConfig.h"
#include "sensorarrayRowModeProfile.h"
#include "sensorarrayTransportGuaranteedText.h"

#define SENSORARRAY_COMMAND_QUEUE_LENGTH 8u
#define SENSORARRAY_COMMAND_TEXT_MAX 48u
#define SENSORARRAY_BLE_CAP_PERIOD_MAX 100000u

static StaticQueue_t s_commandQueueStruct;
static uint8_t s_commandQueueStorage[SENSORARRAY_COMMAND_QUEUE_LENGTH *
                                     sizeof(sensorarrayCommand_t)];
static QueueHandle_t s_commandQueue;
static portMUX_TYPE s_commandStateMux = portMUX_INITIALIZER_UNLOCKED;
static uint32_t s_bleCapPeriod;
static uint32_t s_captureFpsCap;
static uint32_t s_outputFpsCap;
static bool s_traceEnabled;
static bool s_adsDebugEnabled;
static uint32_t s_nextRequestId;

typedef struct {
    bool inUse;
    bool ackPublished;
    bool terminalEmitted;
    bool commandPending;
    uint32_t requestId;
    sensorarrayCommand_t command;
} sensorarrayRowModesReservation_t;

static sensorarrayRowModesReservation_t s_rowModesReservation;

static esp_err_t sensorarrayCommandMailboxQueue(const sensorarrayCommand_t *command)
{
    if (!s_commandQueue || !command) {
        return ESP_ERR_INVALID_STATE;
    }
    /* All producers funnel through this choke point, so the capacity check
     * and enqueue of a reserved ROWMODES transaction are atomic against
     * concurrent serial/BLE/Wi-Fi command sources. */
    BaseType_t sent;
    portENTER_CRITICAL(&s_commandStateMux);
    sent = xQueueSend(s_commandQueue, command, 0);
    portEXIT_CRITICAL(&s_commandStateMux);
    return sent == pdTRUE ? ESP_OK : ESP_ERR_NO_MEM;
}

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

static esp_err_t sensorarrayCommandParseFpsCap(const char *text,
                                               const char *prefix,
                                               sensorarrayCommandType_t type,
                                               sensorarrayCommand_t *outCommand)
{
    size_t prefixLength = strlen(prefix);
    if (strncmp(text, prefix, prefixLength) != 0) {
        return ESP_ERR_NOT_SUPPORTED;
    }
    const char *value = text + prefixLength;
    uint32_t fps = 0u;
    if (strcmp(value, "OFF") == 0) {
        fps = 0u;
    } else if (strncmp(value, "ON,", 3u) == 0) {
        esp_err_t err = sensorarrayCommandParseUnsigned(value + 3u, 200u, &fps);
        if (err != ESP_OK || fps == 0u) {
            return ESP_ERR_INVALID_ARG;
        }
    } else {
        return ESP_ERR_INVALID_ARG;
    }
    *outCommand = (sensorarrayCommand_t){
        .type = type,
        .value = fps,
    };
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

    if (strncmp(commandText, "ADSDBG=", 7u) == 0) {
        uint32_t enabled = 0u;
        esp_err_t err = sensorarrayCommandParseUnsigned(&commandText[7],
                                                         1u,
                                                         &enabled);
        if (err != ESP_OK) {
            return err;
        }
        *outCommand = (sensorarrayCommand_t){
            .type = SENSORARRAY_COMMAND_ADS_DEBUG,
            .value = enabled,
        };
        return ESP_OK;
    }

    esp_err_t capErr = sensorarrayCommandParseFpsCap(commandText,
                                                     "FPSCAP=",
                                                     SENSORARRAY_COMMAND_CAPTURE_FPS_CAP,
                                                     outCommand);
    if (capErr != ESP_ERR_NOT_SUPPORTED) {
        return capErr;
    }
    capErr = sensorarrayCommandParseFpsCap(commandText,
                                           "OUTCAP=",
                                           SENSORARRAY_COMMAND_OUTPUT_FPS_CAP,
                                           outCommand);
    if (capErr != ESP_ERR_NOT_SUPPORTED) {
        return capErr;
    }

    if (strcmp(commandText, "ADSGAP=OFF") == 0 ||
        strcmp(commandText, "ADSGAP=ON") == 0 ||
        strcmp(commandText, "ADSGAP=RAIL") == 0 ||
        strcmp(commandText, "ADSGAP=BAT") == 0 ||
        strcmp(commandText, "ADSGAP=ZERO") == 0) {
        uint32_t mode = 1u;
        if (strcmp(commandText, "ADSGAP=OFF") == 0) {
            mode = 0u;
        } else if (strcmp(commandText, "ADSGAP=RAIL") == 0) {
            mode = 2u;
        } else if (strcmp(commandText, "ADSGAP=BAT") == 0) {
            mode = 3u;
        } else if (strcmp(commandText, "ADSGAP=ZERO") == 0) {
            mode = 4u;
        }
        *outCommand = (sensorarrayCommand_t){
            .type = SENSORARRAY_COMMAND_ADS_GAP_MODE,
            .value = mode,
        };
        return ESP_OK;
    }

    sensorarrayCommandType_t calibrationType;
    if (strcmp(commandText, "CAL=SAVE") == 0) {
        calibrationType = SENSORARRAY_COMMAND_CALIBRATE_SAVE;
    } else if (strcmp(commandText, "CAL=LOAD") == 0) {
        calibrationType = SENSORARRAY_COMMAND_CALIBRATE_LOAD;
    } else if (strcmp(commandText, "CAL=ZERO") == 0) {
        calibrationType = SENSORARRAY_COMMAND_CALIBRATE_ZERO;
    } else if (strcmp(commandText, "CAL=RAIL") == 0 ||
               strcmp(commandText, "RAILCAL") == 0) {
        calibrationType = SENSORARRAY_COMMAND_CALIBRATE_RAIL;
    } else if (strcmp(commandText, "CAL=ALL") == 0 ||
               strcmp(commandText, "BATCAL") == 0) {
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
    s_captureFpsCap =
        CONFIG_SENSORARRAY_CAPTURE_FPS_LIMIT_ENABLE ? CONFIG_SENSORARRAY_CAPTURE_FPS_LIMIT : 0u;
    s_outputFpsCap =
        CONFIG_SENSORARRAY_OUTPUT_RATE_LIMIT_ENABLE ? CONFIG_SENSORARRAY_OUTPUT_RATE_LIMIT : 0u;
    s_traceEnabled = false;
    s_adsDebugEnabled = false;
    s_nextRequestId = 1u;
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

    /* Commands are operator intent. A full queue rejects the newest command
     * instead of silently discarding an older accepted request; callers only
     * acknowledge a request after it has been enqueued successfully. */
    return sensorarrayCommandMailboxQueue(&command);
}

esp_err_t sensorarrayCommandMailboxPostMeasurementMode(
    sensorarrayMeasurementMode_t mode,
    uint32_t *outRequestId)
{
    if (!s_commandQueue || !outRequestId ||
        !sensorarrayMeasurementModeIsDataMode(mode)) {
        return ESP_ERR_INVALID_ARG;
    }
    portENTER_CRITICAL(&s_commandStateMux);
    uint32_t requestId = s_nextRequestId++;
    if (s_nextRequestId == 0u) {
        s_nextRequestId = 1u;
    }
    portEXIT_CRITICAL(&s_commandStateMux);
    sensorarrayCommand_t command = {
        .type = SENSORARRAY_COMMAND_MEASUREMENT_MODE,
        .value = (uint32_t)mode,
        .requestId = requestId,
    };
    esp_err_t err = sensorarrayCommandMailboxQueue(&command);
    if (err == ESP_OK) {
        *outRequestId = requestId;
    }
    return err;
}

esp_err_t sensorarrayCommandMailboxPostRowModes(const char *profile,
                                                uint32_t *outRequestId)
{
    sensorarrayMeasurementMode_t modes[SENSORARRAY_ROW_MODE_PROFILE_ROWS];
    if (!s_commandQueue || !outRequestId ||
        !sensorarrayRowModeProfileParse(profile,
                                        profile ? strlen(profile) : 0u,
                                        modes)) {
        return ESP_ERR_INVALID_ARG;
    }
    /* Reserve the non-droppable terminal lane before acknowledging.  Only one
     * ROWMODES profile transaction can be pending at a time, so one reserved
     * slot bounds both the terminal pipeline and the acceptance window. */
    esp_err_t slotErr = sensorarrayTransportGuaranteedTextReserve();
    if (slotErr != ESP_OK) {
        return slotErr;
    }

    sensorarrayCommand_t command = {
        .type = SENSORARRAY_COMMAND_ROW_MODES,
    };
    uint32_t requestId = 0u;
    bool accepted = false;
    portENTER_CRITICAL(&s_commandStateMux);
    if (!s_rowModesReservation.inUse) {
        requestId = s_nextRequestId++;
        if (s_nextRequestId == 0u) {
            s_nextRequestId = 1u;
        }
        memcpy(command.rowModes, profile, 8u);
        command.rowModes[8] = '\0';
        command.requestId = requestId;
        s_rowModesReservation = (sensorarrayRowModesReservation_t){
            .inUse = true,
            .requestId = requestId,
            .commandPending = true,
            .command = command,
        };
        accepted = true;
    }
    portEXIT_CRITICAL(&s_commandStateMux);
    if (!accepted) {
        (void)sensorarrayTransportGuaranteedTextCancel();
        return ESP_ERR_NO_MEM;
    }
    *outRequestId = requestId;
    return ESP_OK;
}

bool sensorarrayCommandMailboxCommitRowModesAck(uint32_t requestId)
{
    bool known = false;
    portENTER_CRITICAL(&s_commandStateMux);
    if (s_rowModesReservation.inUse &&
        s_rowModesReservation.requestId == requestId &&
        !s_rowModesReservation.ackPublished) {
        s_rowModesReservation.ackPublished = true;
        /* Hand the stored command to Core 1 only now, after the RMACK was
         * actually published.  Enqueue and the commandPending fallback are
         * resolved under one critical section so the command is delivered
         * exactly once: if the shared queue is momentarily full, Core 1
         * drains it from the reservation in TryReceive instead. */
        if (xQueueSend(s_commandQueue,
                       &s_rowModesReservation.command,
                       0) == pdTRUE) {
            s_rowModesReservation.commandPending = false;
        }
        known = true;
    }
    portEXIT_CRITICAL(&s_commandStateMux);
    if (known) {
        (void)sensorarrayTransportGuaranteedTextRelease();
    }
    return known;
}

bool sensorarrayCommandMailboxCancelRowModesAck(uint32_t requestId)
{
    bool known = false;
    portENTER_CRITICAL(&s_commandStateMux);
    if (s_rowModesReservation.inUse &&
        s_rowModesReservation.requestId == requestId) {
        memset(&s_rowModesReservation, 0, sizeof(s_rowModesReservation));
        known = true;
    }
    portEXIT_CRITICAL(&s_commandStateMux);
    if (known) {
        (void)sensorarrayTransportGuaranteedTextAbort();
    }
    return known;
}

esp_err_t sensorarrayCommandMailboxEmitRowModesTerminal(uint32_t requestId,
                                                        const char *text,
                                                        size_t length)
{
    if (!text || length == 0u) {
        (void)sensorarrayCommandMailboxCancelRowModesAck(requestId);
        return ESP_ERR_INVALID_ARG;
    }
    if (length >= SENSORARRAY_TRANSPORT_GUARANTEED_TEXT_MAX) {
        (void)sensorarrayCommandMailboxCancelRowModesAck(requestId);
        return ESP_ERR_INVALID_SIZE;
    }
    portENTER_CRITICAL(&s_commandStateMux);
    if (!s_rowModesReservation.inUse ||
        s_rowModesReservation.requestId != requestId) {
        portEXIT_CRITICAL(&s_commandStateMux);
        return ESP_ERR_NOT_FOUND;
    }
    if (s_rowModesReservation.terminalEmitted) {
        portEXIT_CRITICAL(&s_commandStateMux);
        return ESP_ERR_INVALID_STATE;
    }
    portEXIT_CRITICAL(&s_commandStateMux);
    esp_err_t err = sensorarrayTransportGuaranteedTextPublish(text, length);
    if (err != ESP_OK) {
        (void)sensorarrayCommandMailboxCancelRowModesAck(requestId);
        return err;
    }
    /* Reservation state changes only after the guaranteed publish succeeded,
     * so a publish failure leaves the lane recoverable by the cancel path and
     * the terminal can never be marked emitted without its content. */
    portENTER_CRITICAL(&s_commandStateMux);
    if (s_rowModesReservation.inUse &&
        s_rowModesReservation.requestId == requestId &&
        !s_rowModesReservation.terminalEmitted) {
        s_rowModesReservation.terminalEmitted = true;
        if (s_rowModesReservation.ackPublished) {
            s_rowModesReservation.inUse = false;
        }
    }
    portEXIT_CRITICAL(&s_commandStateMux);
    return ESP_OK;
}

uint32_t sensorarrayCommandMailboxRowModesOutstanding(void)
{
    portENTER_CRITICAL(&s_commandStateMux);
    uint32_t outstanding = s_rowModesReservation.inUse ? 1u : 0u;
    portEXIT_CRITICAL(&s_commandStateMux);
    return outstanding;
}

esp_err_t sensorarrayCommandMailboxPostRailCalibration(
    int32_t avddUv,
    int32_t avssUv,
    uint32_t *outRequestId)
{
    if (!s_commandQueue || !outRequestId || avddUv <= 0 || avssUv >= 0) {
        return ESP_ERR_INVALID_ARG;
    }
    portENTER_CRITICAL(&s_commandStateMux);
    uint32_t requestId = s_nextRequestId++;
    if (s_nextRequestId == 0u) {
        s_nextRequestId = 1u;
    }
    portEXIT_CRITICAL(&s_commandStateMux);
    sensorarrayCommand_t command = {
        .type = SENSORARRAY_COMMAND_SET_RAIL_CALIBRATION,
        .requestId = requestId,
        .signedValue = avddUv,
        .signedValue2 = avssUv,
    };
    esp_err_t err = sensorarrayCommandMailboxQueue(&command);
    if (err == ESP_OK) {
        *outRequestId = requestId;
    }
    return err;
}

static esp_err_t sensorarrayCommandMailboxPostRequest(sensorarrayCommandType_t type,
                                                       uint32_t value,
                                                       int32_t signedValue,
                                                       uint32_t *outRequestId)
{
    if (!s_commandQueue || !outRequestId) {
        return ESP_ERR_INVALID_ARG;
    }
    portENTER_CRITICAL(&s_commandStateMux);
    uint32_t requestId = s_nextRequestId++;
    if (s_nextRequestId == 0u) {
        s_nextRequestId = 1u;
    }
    portEXIT_CRITICAL(&s_commandStateMux);
    sensorarrayCommand_t command = {
        .type = type,
        .value = value,
        .requestId = requestId,
        .signedValue = signedValue,
    };
    esp_err_t err = sensorarrayCommandMailboxQueue(&command);
    if (err == ESP_OK) {
        *outRequestId = requestId;
    }
    return err;
}

esp_err_t sensorarrayCommandMailboxPostAdsCheck(uint32_t sampleCount,
                                                uint32_t *outRequestId)
{
    if (sampleCount < 1u || sampleCount > 1000u) {
        return ESP_ERR_INVALID_ARG;
    }
    return sensorarrayCommandMailboxPostRequest(SENSORARRAY_COMMAND_ADS_CHECK,
                                                 sampleCount,
                                                 0,
                                                 outRequestId);
}

esp_err_t sensorarrayCommandMailboxPostBatteryNow(bool diagnostic,
                                                  uint32_t *outRequestId)
{
    return sensorarrayCommandMailboxPostRequest(
        diagnostic ? SENSORARRAY_COMMAND_BATTERY_DIAGNOSTIC :
                     SENSORARRAY_COMMAND_BATTERY_NOW,
        1u,
        0,
        outRequestId);
}

esp_err_t sensorarrayCommandMailboxPostBatteryPeriod(bool enabled,
                                                     uint32_t periodMs,
                                                     uint32_t *outRequestId)
{
    if (enabled && (periodMs < 100u || periodMs > 600000u)) {
        return ESP_ERR_INVALID_ARG;
    }
    return sensorarrayCommandMailboxPostRequest(SENSORARRAY_COMMAND_BATTERY_PERIOD,
                                                 periodMs,
                                                 enabled ? 1 : 0,
                                                 outRequestId);
}

esp_err_t sensorarrayCommandMailboxPostResSettle(uint32_t settleUs,
                                                 uint32_t *outRequestId)
{
    if (settleUs > 10000u) {
        return ESP_ERR_INVALID_ARG;
    }
    return sensorarrayCommandMailboxPostRequest(SENSORARRAY_COMMAND_RES_SETTLE,
                                                 settleUs,
                                                 0,
                                                 outRequestId);
}

bool sensorarrayCommandMailboxTryReceive(sensorarrayCommand_t *outCommand)
{
    if (!s_commandQueue || !outCommand) {
        return false;
    }
    bool received = false;
    portENTER_CRITICAL(&s_commandStateMux);
    if (xQueueReceive(s_commandQueue, outCommand, 0) == pdTRUE) {
        received = true;
    } else if (s_rowModesReservation.inUse &&
               s_rowModesReservation.ackPublished &&
               s_rowModesReservation.commandPending &&
               !s_rowModesReservation.terminalEmitted) {
        *outCommand = s_rowModesReservation.command;
        s_rowModesReservation.commandPending = false;
        received = true;
    }
    portEXIT_CRITICAL(&s_commandStateMux);
    return received;
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
    } else if (command->type == SENSORARRAY_COMMAND_CAPTURE_FPS_CAP) {
        s_captureFpsCap = command->value;
    } else if (command->type == SENSORARRAY_COMMAND_OUTPUT_FPS_CAP) {
        s_outputFpsCap = command->value;
    } else if (command->type == SENSORARRAY_COMMAND_ADS_DEBUG) {
        s_adsDebugEnabled = command->value != 0u;
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

bool sensorarrayCommandMailboxAdsDebugEnabled(void)
{
    portENTER_CRITICAL(&s_commandStateMux);
    bool enabled = s_adsDebugEnabled;
    portEXIT_CRITICAL(&s_commandStateMux);
    return enabled;
}

uint32_t sensorarrayCommandMailboxGetCaptureFpsCap(void)
{
    portENTER_CRITICAL(&s_commandStateMux);
    uint32_t cap = s_captureFpsCap;
    portEXIT_CRITICAL(&s_commandStateMux);
    return cap;
}

uint32_t sensorarrayCommandMailboxGetOutputFpsCap(void)
{
    portENTER_CRITICAL(&s_commandStateMux);
    uint32_t cap = s_outputFpsCap;
    portEXIT_CRITICAL(&s_commandStateMux);
    return cap;
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
    case SENSORARRAY_COMMAND_CALIBRATE_SAVE:
        return "cal_save";
    case SENSORARRAY_COMMAND_CALIBRATE_LOAD:
        return "cal_load";
    case SENSORARRAY_COMMAND_ADS_GAP_MODE:
        return "adsgap";
    case SENSORARRAY_COMMAND_CAPTURE_FPS_CAP:
        return "fpscap";
    case SENSORARRAY_COMMAND_OUTPUT_FPS_CAP:
        return "outcap";
    case SENSORARRAY_COMMAND_MEASUREMENT_MODE:
        return "mode";
    case SENSORARRAY_COMMAND_SET_RAIL_CALIBRATION:
        return "rail_calibration";
    case SENSORARRAY_COMMAND_ADS_CHECK:
        return "ads_check";
    case SENSORARRAY_COMMAND_BATTERY_NOW:
        return "battery_now";
    case SENSORARRAY_COMMAND_BATTERY_DIAGNOSTIC:
        return "battery_diagnostic";
    case SENSORARRAY_COMMAND_BATTERY_PERIOD:
        return "battery_period";
    case SENSORARRAY_COMMAND_RES_SETTLE:
        return "res_settle";
    case SENSORARRAY_COMMAND_ADS_DEBUG:
        return "ads_debug";
    case SENSORARRAY_COMMAND_ROW_MODES:
        return "rowmodes";
    default:
        return "unknown";
    }
}

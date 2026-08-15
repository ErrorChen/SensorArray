#include "sensorarrayEarlyRecovery.h"

#include <stdlib.h>
#include <string.h>

sensorarrayEarlyRecoveryKind_t sensorarrayEarlyRecoveryParse(
    const char *text,
    uint32_t *outRecoveryLevel)
{
    if (outRecoveryLevel) {
        *outRecoveryLevel = 0u;
    }
    if (!text) {
        return SENSORARRAY_EARLY_KIND_UNSUPPORTED;
    }
    if (strcmp(text, "BOOT?") == 0) {
        return SENSORARRAY_EARLY_KIND_BOOT_QUERY;
    }
    if (strcmp(text, "STATE?") == 0) {
        return SENSORARRAY_EARLY_KIND_STATE_QUERY;
    }
    if (strcmp(text, "MODE?") == 0) {
        return SENSORARRAY_EARLY_KIND_MODE_QUERY;
    }
    if (strcmp(text, "READY?") == 0) {
        return SENSORARRAY_EARLY_KIND_READY_QUERY;
    }
    if (strcmp(text, "PROTO?") == 0) {
        return SENSORARRAY_EARLY_KIND_PROTO_QUERY;
    }
    if (strcmp(text, "RESTART") == 0) {
        return SENSORARRAY_EARLY_KIND_RESTART;
    }
    if (strcmp(text, "RECOVER") == 0) {
        if (outRecoveryLevel) {
            *outRecoveryLevel = SENSORARRAY_EARLY_RECOVERY_LEVEL_FULL;
        }
        return SENSORARRAY_EARLY_KIND_RECOVER;
    }
    if (strncmp(text, "RECOVER=", 8u) == 0) {
        char *end = NULL;
        unsigned long parsed = strtoul(text + 8u, &end, 10);
        if (end == text + 8u || *end != '\0' ||
            parsed > SENSORARRAY_EARLY_RECOVERY_LEVEL_MAX) {
            return SENSORARRAY_EARLY_KIND_RECOVER_INVALID;
        }
        if (outRecoveryLevel) {
            *outRecoveryLevel = (uint32_t)parsed;
        }
        return SENSORARRAY_EARLY_KIND_RECOVER;
    }
    if (strncmp(text, "ROWMODES=", 9u) == 0) {
        return SENSORARRAY_EARLY_KIND_ROW_MODES_REJECT;
    }
    return SENSORARRAY_EARLY_KIND_UNSUPPORTED;
}

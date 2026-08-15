#pragma once

#include <stdint.h>

#define SENSORARRAY_EARLY_RECOVERY_LEVEL_FULL 1u
#define SENSORARRAY_EARLY_RECOVERY_LEVEL_MAX 2u

typedef enum {
    SENSORARRAY_EARLY_KIND_UNSUPPORTED = 0,
    SENSORARRAY_EARLY_KIND_BOOT_QUERY,
    SENSORARRAY_EARLY_KIND_STATE_QUERY,
    SENSORARRAY_EARLY_KIND_MODE_QUERY,
    SENSORARRAY_EARLY_KIND_READY_QUERY,
    SENSORARRAY_EARLY_KIND_PROTO_QUERY,
    SENSORARRAY_EARLY_KIND_RECOVER,
    SENSORARRAY_EARLY_KIND_RECOVER_INVALID,
    SENSORARRAY_EARLY_KIND_RESTART,
    SENSORARRAY_EARLY_KIND_ROW_MODES_REJECT,
} sensorarrayEarlyRecoveryKind_t;

/* Pure classification for the pre-acquisition recovery-safe command set.
 * BOOT?/STATE?/MODE?/READY?/PROTO? are readable, RECOVER/RESTART are
 * receivable, and acquisition commands such as ROWMODES= are recognised so
 * the early handler can reject them with an explicit not-ready error instead
 * of routing them into the measurement mailbox. */
sensorarrayEarlyRecoveryKind_t sensorarrayEarlyRecoveryParse(
    const char *text,
    uint32_t *outRecoveryLevel);

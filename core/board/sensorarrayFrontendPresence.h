#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "sensorarrayTypes.h"

/* 本轮新增. Communication evidence, never a claim of physical isolation or
 * proof that a component has been physically unplugged. */
typedef enum {
    SENSORARRAY_FRONTEND_LINK_UNKNOWN = 0,
    SENSORARRAY_FRONTEND_LINK_PRESENT,
    SENSORARRAY_FRONTEND_LINK_UNAVAILABLE,
    SENSORARRAY_FRONTEND_LINK_BUSY,
    SENSORARRAY_FRONTEND_LINK_INHIBITED,
} sensorarrayFrontendLink_t;

typedef enum {
    SENSORARRAY_FRONTEND_PRESENCE_MONITOR = 0,
    SENSORARRAY_FRONTEND_PRESENCE_SUSPECT,
    SENSORARRAY_FRONTEND_PRESENCE_WAIT,
    SENSORARRAY_FRONTEND_PRESENCE_RESTART_REQUIRED,
} sensorarrayFrontendPresencePhase_t;

typedef enum {
    SENSORARRAY_FRONTEND_PRESENCE_NONE = 0,
    SENSORARRAY_FRONTEND_PRESENCE_ENTER_WAIT,
    SENSORARRAY_FRONTEND_PRESENCE_REQUEST_RESTART,
} sensorarrayFrontendPresenceAction_t;

enum {
    SENSORARRAY_FRONTEND_PRIMARY = 0,
    SENSORARRAY_FRONTEND_SECONDARY = 1,
    SENSORARRAY_FRONTEND_ADS = 2,
    SENSORARRAY_FRONTEND_COUNT = 3,
    SENSORARRAY_FRONTEND_ALL_MASK = 7,
};

typedef struct {
    sensorarrayFrontendLink_t link;
    esp_err_t error;
    esp_err_t cleanupError;
    uint16_t manufacturerId;       /* FDC first 7E read; ADS ID uses deviceId. */
    uint16_t deviceId;             /* FDC first 7F read. */
    uint16_t manufacturerIdSecond; /* FDC second 7E read. */
    uint16_t deviceIdSecond;       /* FDC second 7F read. */
    uint8_t idReadMask;            /* Completed FDC 7E/7F/7E/7F reads, bits 0..3. */
    uint8_t power;
    uint8_t interface;
    uint32_t clockHz;         /* Clock used by the identity probe. */
    uint32_t restoredClockHz; /* Confirmed clock after cleanup; zero if unknown. */
} sensorarrayFrontendPresenceDevice_t;

typedef struct {
    uint64_t startedUs;
    uint64_t endedUs;
    bool ownerQuiescent;
    bool fdcShutdownHigh;
    bool lowSpeed;
    bool quarantined;
    bool adsWasInitialized;
    sensorarrayFrontendPresenceDevice_t devices[SENSORARRAY_FRONTEND_COUNT];
} sensorarrayFrontendPresenceEvidence_t;

typedef struct {
    bool initialized;
    sensorarrayFrontendPresencePhase_t phase;
    uint32_t probes;
    uint32_t absentRun;
    uint32_t presentRun;
    uint8_t knownMask;
    uint8_t presentMask;
    uint64_t nextProbeUs;
    uint64_t waitEnteredUs;
    sensorarrayFrontendPresenceEvidence_t evidence;
} sensorarrayFrontendPresenceSnapshot_t;

typedef struct {
    sensorarrayFrontendPresenceSnapshot_t published;
    uint16_t recoveryIds[SENSORARRAY_FRONTEND_COUNT];
} sensorarrayFrontendPresence_t;

/* One acquisition coordinator writes a context. CopySnapshot/IsWaiting are
 * safe for control tasks; bus I/O and logging never run under the state lock.
 * Probe calls do not reset frontends, switch matrix routing, enable INTREF,
 * clear RESET, create matrix jobs, or change scientific-data caches. */
void sensorarrayFrontendPresenceInit(sensorarrayFrontendPresence_t *presence,
                                    uint64_t nowUs);
/* 本轮新增: the coordinator has already paused after an unqualified startup
 * or late owner release. Latch suspicion without inventing missing IDs or
 * RESET evidence. Repeated calls do not reset an existing recovery run. */
void sensorarrayFrontendPresenceLatchHold(sensorarrayFrontendPresence_t *presence,
                                         uint64_t nowUs);
bool sensorarrayFrontendPresenceDue(const sensorarrayFrontendPresence_t *presence,
                                   uint64_t nowUs);
bool sensorarrayFrontendPresenceIsWaiting(const sensorarrayFrontendPresence_t *presence);
bool sensorarrayFrontendPresenceCopySnapshot(const sensorarrayFrontendPresence_t *presence,
    sensorarrayFrontendPresenceSnapshot_t *outSnapshot);
const char *sensorarrayFrontendPresencePhaseName(sensorarrayFrontendPresencePhase_t phase);
const char *sensorarrayFrontendPresenceLinkName(sensorarrayFrontendLink_t link);

/* 本轮新增 pure policy entry for replay/fault injection. A round is consumed
 * only when due. The FIRST complete unavailable round (or observed ADS RESET)
 * enters SUSPECT and requests immediate pause/release of the reference clamp.
 * A failed clock/state cleanup also enters SUSPECT; failed cleanup cannot
 * count as present. Three unavailable rounds confirm WAIT; two stable,
 * clean all-present rounds after any suspicion request a controlled restart.
 * Replayed/stale evidence cannot advance a consecutive run. No in-place
 * reinitialization is authorized by this module. */
sensorarrayFrontendPresenceAction_t sensorarrayFrontendPresenceObserve(
    sensorarrayFrontendPresence_t *presence,
    const sensorarrayFrontendPresenceEvidence_t *evidence, uint64_t nowUs);

/* fdcSdHigh=false must mean the caller knows SD is low, not an unknown
 * default. ownerQuiescent must cover startup ADS, ADS helper/matrix ownership,
 * both FDC workers, I2C recovery, and all pending DMA. If either condition is
 * unavailable, the round is BUSY/INHIBITED and cannot establish absence.
 * Deadline admission is bounded; existing board I2C calls retain their own
 * finite lock/recovery bounds. A late round is rejected, never called timely.
 * I2C uses selected clocks normally, <=100kHz only in suspect/wait rounds,
 * and restores the saved clock before returning. ADS presence is read-only. */
sensorarrayFrontendPresenceAction_t sensorarrayFrontendPresencePoll(
    sensorarrayFrontendPresence_t *presence, sensorarrayState_t *state,
    uint64_t nowUs, bool ownerQuiescent, bool fdcSdHigh,
    sensorarrayFrontendPresenceSnapshot_t *outSnapshot);

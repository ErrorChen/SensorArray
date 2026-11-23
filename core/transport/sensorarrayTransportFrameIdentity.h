#pragma once

#include <stdbool.h>
#include <stdint.h>

/* 本轮新增: producer-owned identity copied with a successful real encoder
 * packet. Header parsing alone must never confer this provenance. */
typedef struct {
    uint64_t sessionId;
    uint32_t bootId;
    uint32_t revision;
    uint32_t sequence;
    uint32_t modeGeneration;
    uint32_t modeRequestId;
    uint32_t rowsGeneration;
    uint32_t rowsRequestId;
    uint32_t profileGeneration;
    uint32_t profileRequestId;
    uint8_t rows;
} sensorarrayTransportFrameIdentity_t;

/* 本轮新增: metadata supplied with the real encoder's immutable frame.
 * Legacy gen/rid are tag-specific, not aliases of identity.modeGeneration.
 * M already emits rgen/rrid and pgen/prid from the row/profile snapshots;
 * retain those values even when diagnosing disagreeing identity domains. */
typedef struct {
    sensorarrayTransportFrameIdentity_t identity;
    uint32_t legacyGeneration;
    uint32_t legacyRequestId;
    uint32_t mixedProfileGeneration;
    uint32_t mixedProfileRequestId;
    char tag;
} sensorarrayTransportFrameMetadata_t;

#define SENSORARRAY_FRAME_PROTECT_FIRST_CONFIG 1u
#define SENSORARRAY_FRAME_PROTECT_PERIODIC 2u
#define SENSORARRAY_FRAME_PROTECT_MASK \
    (SENSORARRAY_FRAME_PROTECT_FIRST_CONFIG | SENSORARRAY_FRAME_PROTECT_PERIODIC)

static inline bool sensorarrayTransportFrameSameConfig(
    const sensorarrayTransportFrameIdentity_t *a,
    const sensorarrayTransportFrameIdentity_t *b)
{
    return a && b && a->sessionId == b->sessionId && a->bootId == b->bootId &&
        a->revision == b->revision && a->modeGeneration == b->modeGeneration &&
        a->modeRequestId == b->modeRequestId && a->rowsGeneration == b->rowsGeneration &&
        a->rowsRequestId == b->rowsRequestId && a->profileGeneration == b->profileGeneration &&
        a->profileRequestId == b->profileRequestId && a->rows == b->rows;
}

/* Half-range sequence order has meaning only inside one session/boot. */
static inline bool sensorarrayTransportFrameOlderOrSame(
    const sensorarrayTransportFrameIdentity_t *candidate,
    const sensorarrayTransportFrameIdentity_t *last)
{
    if (!candidate || !last || candidate->sessionId != last->sessionId ||
        candidate->bootId != last->bootId) {
        return false;
    }
    uint32_t distance = candidate->sequence - last->sequence;
    return distance == 0u || distance >= UINT32_C(0x80000000);
}

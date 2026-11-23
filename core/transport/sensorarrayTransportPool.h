#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "sensorarrayTransportChannels.h"
#include "sensorarrayTransportFrameIdentity.h"

#define SENSORARRAY_TRANSPORT_POOL_TEXT_MAX 2304u
#define SENSORARRAY_TRANSPORT_POOL_NORMAL_SLOT_COUNT 4u
#define SENSORARRAY_TRANSPORT_POOL_PROTECTED_SLOT_COUNT 2u
#define SENSORARRAY_TRANSPORT_POOL_SLOT_COUNT 7u
#define SENSORARRAY_TRANSPORT_POOL_STORAGE_STRIDE \
    (SENSORARRAY_TRANSPORT_POOL_TEXT_MAX + 1u)
#define SENSORARRAY_TRANSPORT_POOL_STORAGE_BYTES \
    (SENSORARRAY_TRANSPORT_POOL_SLOT_COUNT * SENSORARRAY_TRANSPORT_POOL_STORAGE_STRIDE)
#define SENSORARRAY_TRANSPORT_POOL_LOG_SLOT_LIMIT 2u
#define SENSORARRAY_TRANSPORT_POOL_LIFECYCLE_SLOT_INDEX \
    SENSORARRAY_TRANSPORT_POOL_NORMAL_SLOT_COUNT
#define SENSORARRAY_TRANSPORT_POOL_PROTECTED_FIRST_INDEX \
    (SENSORARRAY_TRANSPORT_POOL_LIFECYCLE_SLOT_INDEX + 1u)

typedef struct {
    uint32_t slotGeneration;
    uint32_t frameSeq;
    uint32_t generation;
    uint32_t requestId;
    uint32_t bleConnectionGeneration;
    uint32_t wifiEpoch;
    uint64_t wifiDeadlineUs;
    uint64_t sessionId;
    uint32_t bootId;
    uint32_t configRevision;
    uint32_t modeGeneration;
    uint32_t modeRequestId;
    uint32_t rowsGeneration;
    uint32_t rowsRequestId;
    uint32_t profileGeneration;
    uint32_t profileRequestId;
    uint16_t length;
    uint8_t channel;
    uint8_t rows;
    uint8_t cells;
    bool inUse;
    bool hasFrameMeta;
    bool trustedFullFrame;
    sensorarrayTransportFrameIdentity_t frameIdentity;
    uint32_t protectionFlags;
    uint64_t protectedDeadlineUs;
    uint64_t protectedNextAttemptUs;
    bool protectedWifiDone;
    bool protectedWifiTarget; /* Enabled and ready at enqueue, not peer receipt. */
    /* CPU-only lifetime storage; metadata/ownership remain internal. Each
     * bound slice includes one private NUL byte outside the wire contract. */
    char *data;
} sensorarrayTransportPayloadSlot_t;

typedef struct {
    uint8_t slotIndex;
    uint8_t channel;
    uint16_t length;
    uint32_t slotGeneration;
} sensorarrayTransportDescriptor_t;

typedef struct {
    uint32_t used;
    uint32_t highWater;
    uint32_t allocFail;
    uint32_t releaseMismatch;
    uint32_t staleDescriptor;
} sensorarrayTransportPoolStats_t;

typedef struct {
    sensorarrayTransportPayloadSlot_t slots[SENSORARRAY_TRANSPORT_POOL_SLOT_COUNT];
    sensorarrayTransportPoolStats_t stats;
    char *storage;
    size_t storageBytes;
} sensorarrayTransportPool_t;

_Static_assert(sizeof(sensorarrayTransportDescriptor_t) <= 32u,
               "Transport queue must contain descriptors, not payloads");
_Static_assert(SENSORARRAY_TRANSPORT_POOL_LOG_SLOT_LIMIT <
                   SENSORARRAY_TRANSPORT_POOL_SLOT_COUNT,
               "Transport pool must reserve payload capacity for DATA");
_Static_assert(SENSORARRAY_TRANSPORT_POOL_LIFECYCLE_SLOT_INDEX <
                   SENSORARRAY_TRANSPORT_POOL_SLOT_COUNT,
               "Lifecycle slot must be inside the transport pool");

/* These functions intentionally do not lock. Firmware callers protect only
 * the short state transition with their portMUX; payload copy/parse occurs
 * after allocation and outside that critical section. */
/* 本轮新增 binding contract: pool must initially be zero-initialized and
 * storage must remain alive for the pool's entire lifetime. Same-storage
 * repeated calls preserve all generations/owned slots; rebinding is denied.
 * Allocation policy stays with the firmware caller; this module never allocates. */
bool sensorarrayTransportPoolInit(sensorarrayTransportPool_t *pool,
    char *storage, size_t storageBytes);
bool sensorarrayTransportPoolAllocate(sensorarrayTransportPool_t *pool,
                                      sensorarrayTransportChannel_t channel,
                                      size_t length,
                                      sensorarrayTransportDescriptor_t *outDescriptor);
bool sensorarrayTransportPoolAllocateProtected(sensorarrayTransportPool_t *pool,
    size_t length, sensorarrayTransportDescriptor_t *outDescriptor);
bool sensorarrayTransportPoolValidate(sensorarrayTransportPool_t *pool,
                                      const sensorarrayTransportDescriptor_t *descriptor);
sensorarrayTransportPayloadSlot_t *sensorarrayTransportPoolGetSlot(
    sensorarrayTransportPool_t *pool,
    const sensorarrayTransportDescriptor_t *descriptor);
bool sensorarrayTransportPoolRelease(sensorarrayTransportPool_t *pool,
                                     const sensorarrayTransportDescriptor_t *descriptor);

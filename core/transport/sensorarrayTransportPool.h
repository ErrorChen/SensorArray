#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "sensorarrayTransportChannels.h"

#define SENSORARRAY_TRANSPORT_POOL_TEXT_MAX 1536u
#define SENSORARRAY_TRANSPORT_POOL_SLOT_COUNT 4u
#define SENSORARRAY_TRANSPORT_POOL_LOG_SLOT_LIMIT 2u

typedef struct {
    uint32_t slotGeneration;
    uint32_t frameSeq;
    uint32_t generation;
    uint32_t requestId;
    uint32_t bleConnectionGeneration;
    uint16_t length;
    uint8_t channel;
    uint8_t rows;
    uint8_t cells;
    bool inUse;
    bool hasFrameMeta;
    /* The extra byte is private NUL storage, not part of the wire contract. */
    char data[SENSORARRAY_TRANSPORT_POOL_TEXT_MAX + 1u];
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
} sensorarrayTransportPool_t;

_Static_assert(sizeof(sensorarrayTransportDescriptor_t) <= 32u,
               "Transport queue must contain descriptors, not payloads");
_Static_assert(SENSORARRAY_TRANSPORT_POOL_LOG_SLOT_LIMIT <
                   SENSORARRAY_TRANSPORT_POOL_SLOT_COUNT,
               "Transport pool must reserve payload capacity for DATA");

/* These functions intentionally do not lock. Firmware callers protect only
 * the short state transition with their portMUX; payload copy/parse occurs
 * after allocation and outside that critical section. */
void sensorarrayTransportPoolInit(sensorarrayTransportPool_t *pool);
bool sensorarrayTransportPoolAllocate(sensorarrayTransportPool_t *pool,
                                      sensorarrayTransportChannel_t channel,
                                      size_t length,
                                      sensorarrayTransportDescriptor_t *outDescriptor);
bool sensorarrayTransportPoolValidate(sensorarrayTransportPool_t *pool,
                                      const sensorarrayTransportDescriptor_t *descriptor);
sensorarrayTransportPayloadSlot_t *sensorarrayTransportPoolGetSlot(
    sensorarrayTransportPool_t *pool,
    const sensorarrayTransportDescriptor_t *descriptor);
bool sensorarrayTransportPoolRelease(sensorarrayTransportPool_t *pool,
                                     const sensorarrayTransportDescriptor_t *descriptor);

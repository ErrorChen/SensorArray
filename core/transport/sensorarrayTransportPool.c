#include "sensorarrayTransportPool.h"

#include <string.h>

static uint32_t sensorarrayTransportPoolNextGeneration(uint32_t generation)
{
    generation++;
    return generation == 0u ? 1u : generation;
}

void sensorarrayTransportPoolInit(sensorarrayTransportPool_t *pool)
{
    if (pool) {
        memset(pool, 0, sizeof(*pool));
    }
}

bool sensorarrayTransportPoolAllocate(sensorarrayTransportPool_t *pool,
                                      sensorarrayTransportChannel_t channel,
                                      size_t length,
                                      sensorarrayTransportDescriptor_t *outDescriptor)
{
    if (!pool || !outDescriptor || channel > SENSORARRAY_TRANSPORT_CHANNEL_LOG ||
        length == 0u || length > SENSORARRAY_TRANSPORT_POOL_TEXT_MAX) {
        return false;
    }

    uint8_t logSlotsUsed = 0u;
    if (channel == SENSORARRAY_TRANSPORT_CHANNEL_LOG) {
        for (uint8_t index = 0u; index < SENSORARRAY_TRANSPORT_POOL_SLOT_COUNT; ++index) {
            if (pool->slots[index].inUse &&
                pool->slots[index].channel == SENSORARRAY_TRANSPORT_CHANNEL_LOG) {
                logSlotsUsed++;
            }
        }
    }
    if (logSlotsUsed >= SENSORARRAY_TRANSPORT_POOL_LOG_SLOT_LIMIT) {
        pool->stats.allocFail++;
        return false;
    }

    for (uint8_t index = 0u; index < SENSORARRAY_TRANSPORT_POOL_SLOT_COUNT; ++index) {
        sensorarrayTransportPayloadSlot_t *slot = &pool->slots[index];
        if (slot->inUse) {
            continue;
        }
        slot->inUse = true;
        slot->slotGeneration =
            sensorarrayTransportPoolNextGeneration(slot->slotGeneration);
        slot->channel = (uint8_t)channel;
        slot->length = (uint16_t)length;
        slot->bleConnectionGeneration = 0u;
        slot->hasFrameMeta = false;
        *outDescriptor = (sensorarrayTransportDescriptor_t){
            .slotIndex = index,
            .channel = (uint8_t)channel,
            .length = (uint16_t)length,
            .slotGeneration = slot->slotGeneration,
        };
        pool->stats.used++;
        if (pool->stats.used > pool->stats.highWater) {
            pool->stats.highWater = pool->stats.used;
        }
        return true;
    }

    pool->stats.allocFail++;
    return false;
}

bool sensorarrayTransportPoolValidate(sensorarrayTransportPool_t *pool,
                                      const sensorarrayTransportDescriptor_t *descriptor)
{
    if (!pool || !descriptor ||
        descriptor->slotIndex >= SENSORARRAY_TRANSPORT_POOL_SLOT_COUNT ||
        descriptor->channel > SENSORARRAY_TRANSPORT_CHANNEL_LOG ||
        descriptor->length == 0u ||
        descriptor->length > SENSORARRAY_TRANSPORT_POOL_TEXT_MAX) {
        if (pool) {
            pool->stats.staleDescriptor++;
        }
        return false;
    }

    const sensorarrayTransportPayloadSlot_t *slot =
        &pool->slots[descriptor->slotIndex];
    bool current = slot->inUse &&
                   slot->slotGeneration == descriptor->slotGeneration &&
                   slot->channel == descriptor->channel &&
                   slot->length == descriptor->length;
    if (!current) {
        pool->stats.staleDescriptor++;
    }
    return current;
}

sensorarrayTransportPayloadSlot_t *sensorarrayTransportPoolGetSlot(
    sensorarrayTransportPool_t *pool,
    const sensorarrayTransportDescriptor_t *descriptor)
{
    if (!pool || !descriptor ||
        descriptor->slotIndex >= SENSORARRAY_TRANSPORT_POOL_SLOT_COUNT) {
        return NULL;
    }
    return &pool->slots[descriptor->slotIndex];
}

bool sensorarrayTransportPoolRelease(sensorarrayTransportPool_t *pool,
                                     const sensorarrayTransportDescriptor_t *descriptor)
{
    if (!pool || !descriptor ||
        descriptor->slotIndex >= SENSORARRAY_TRANSPORT_POOL_SLOT_COUNT) {
        if (pool) {
            pool->stats.releaseMismatch++;
        }
        return false;
    }

    sensorarrayTransportPayloadSlot_t *slot = &pool->slots[descriptor->slotIndex];
    if (!slot->inUse ||
        slot->slotGeneration != descriptor->slotGeneration ||
        slot->channel != descriptor->channel ||
        slot->length != descriptor->length) {
        pool->stats.releaseMismatch++;
        return false;
    }

    slot->inUse = false;
    slot->bleConnectionGeneration = 0u;
    slot->hasFrameMeta = false;
    if (pool->stats.used != 0u) {
        pool->stats.used--;
    }
    return true;
}

#include "sensorarrayTransportPool.h"

static uint32_t sensorarrayTransportPoolNextGeneration(uint32_t generation)
{
    generation++;
    return generation == 0u ? 1u : generation;
}

bool sensorarrayTransportPoolInit(sensorarrayTransportPool_t *pool,
    char *storage, size_t storageBytes)
{
    if (!pool || !storage || storageBytes != SENSORARRAY_TRANSPORT_POOL_STORAGE_BYTES) {
        return false;
    }
    if (pool->storage) {
        return pool->storage == storage && pool->storageBytes == storageBytes;
    }
    if (pool->storageBytes != 0u || pool->stats.used != 0u) { return false; }
    for (size_t i = 0u; i < SENSORARRAY_TRANSPORT_POOL_SLOT_COUNT; ++i) {
        if (pool->slots[i].inUse || pool->slots[i].data) { return false; }
    }
    for (size_t i = 0u; i < SENSORARRAY_TRANSPORT_POOL_SLOT_COUNT; ++i) {
        pool->slots[i].data = storage + i * SENSORARRAY_TRANSPORT_POOL_STORAGE_STRIDE;
    }
    pool->storageBytes = storageBytes;
    pool->storage = storage;
    return true;
}

static bool sensorarrayTransportPoolSlotReady(const sensorarrayTransportPool_t *pool,
    size_t index)
{
    return pool && pool->storage &&
        pool->storageBytes == SENSORARRAY_TRANSPORT_POOL_STORAGE_BYTES &&
        index < SENSORARRAY_TRANSPORT_POOL_SLOT_COUNT &&
        pool->slots[index].data == pool->storage + index * SENSORARRAY_TRANSPORT_POOL_STORAGE_STRIDE;
}

bool sensorarrayTransportPoolAllocate(sensorarrayTransportPool_t *pool,
                                      sensorarrayTransportChannel_t channel,
                                      size_t length,
                                      sensorarrayTransportDescriptor_t *outDescriptor)
{
    if (!pool || !outDescriptor ||
        (channel != SENSORARRAY_TRANSPORT_CHANNEL_DATA &&
         channel != SENSORARRAY_TRANSPORT_CHANNEL_LOG &&
         channel != SENSORARRAY_TRANSPORT_CHANNEL_LIFECYCLE) ||
        length == 0u || length > SENSORARRAY_TRANSPORT_POOL_TEXT_MAX) {
        return false;
    }

    if (!pool->storage) { pool->stats.allocFail++; return false; }

    if (channel == SENSORARRAY_TRANSPORT_CHANNEL_LIFECYCLE) {
        sensorarrayTransportPayloadSlot_t *slot =
            &pool->slots[SENSORARRAY_TRANSPORT_POOL_LIFECYCLE_SLOT_INDEX];
        if (slot->inUse || !sensorarrayTransportPoolSlotReady(pool,
                SENSORARRAY_TRANSPORT_POOL_LIFECYCLE_SLOT_INDEX)) {
            pool->stats.allocFail++;
            return false;
        }
        slot->inUse = true;
        slot->slotGeneration =
            sensorarrayTransportPoolNextGeneration(slot->slotGeneration);
        slot->channel = (uint8_t)channel;
        slot->length = (uint16_t)length;
        slot->bleConnectionGeneration = 0u;
        slot->wifiEpoch = 0u;
        slot->wifiDeadlineUs = 0u;
        slot->hasFrameMeta = false;
        slot->trustedFullFrame = false;
        slot->protectionFlags = 0u;
        slot->protectedWifiDone = false;
        slot->protectedWifiTarget = false;
        *outDescriptor = (sensorarrayTransportDescriptor_t){
            .slotIndex = SENSORARRAY_TRANSPORT_POOL_LIFECYCLE_SLOT_INDEX,
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

    for (uint8_t index = 0u; index < SENSORARRAY_TRANSPORT_POOL_NORMAL_SLOT_COUNT; ++index) {
        sensorarrayTransportPayloadSlot_t *slot = &pool->slots[index];
        if (slot->inUse || !sensorarrayTransportPoolSlotReady(pool, index)) {
            continue;
        }
        slot->inUse = true;
        slot->slotGeneration =
            sensorarrayTransportPoolNextGeneration(slot->slotGeneration);
        slot->channel = (uint8_t)channel;
        slot->length = (uint16_t)length;
        slot->bleConnectionGeneration = 0u;
        slot->wifiEpoch = 0u;
        slot->wifiDeadlineUs = 0u;
        slot->hasFrameMeta = false;
        slot->trustedFullFrame = false;
        slot->protectionFlags = 0u;
        slot->protectedWifiDone = false;
        slot->protectedWifiTarget = false;
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

bool sensorarrayTransportPoolAllocateProtected(sensorarrayTransportPool_t *pool,
    size_t length, sensorarrayTransportDescriptor_t *outDescriptor)
{
    if (!pool || !outDescriptor || length == 0u || length > SENSORARRAY_TRANSPORT_POOL_TEXT_MAX) {
        return false;
    }
    if (!pool->storage) { pool->stats.allocFail++; return false; }
    for (uint8_t index = SENSORARRAY_TRANSPORT_POOL_PROTECTED_FIRST_INDEX;
         index < SENSORARRAY_TRANSPORT_POOL_SLOT_COUNT; ++index) {
        sensorarrayTransportPayloadSlot_t *slot = &pool->slots[index];
        if (slot->inUse || !sensorarrayTransportPoolSlotReady(pool, index)) { continue; }
        slot->inUse = true;
        slot->slotGeneration = sensorarrayTransportPoolNextGeneration(slot->slotGeneration);
        slot->channel = SENSORARRAY_TRANSPORT_CHANNEL_DATA;
        slot->length = (uint16_t)length;
        slot->bleConnectionGeneration = 0u;
        slot->wifiEpoch = 0u;
        slot->wifiDeadlineUs = 0u;
        slot->hasFrameMeta = false;
        slot->trustedFullFrame = false;
        slot->protectionFlags = 0u;
        slot->protectedWifiDone = false;
        slot->protectedWifiTarget = false;
        *outDescriptor = (sensorarrayTransportDescriptor_t){
            .slotIndex = index, .channel = SENSORARRAY_TRANSPORT_CHANNEL_DATA,
            .length = (uint16_t)length, .slotGeneration = slot->slotGeneration,
        };
        pool->stats.used++;
        if (pool->stats.used > pool->stats.highWater) { pool->stats.highWater = pool->stats.used; }
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
        (descriptor->channel != SENSORARRAY_TRANSPORT_CHANNEL_DATA &&
         descriptor->channel != SENSORARRAY_TRANSPORT_CHANNEL_LOG &&
         descriptor->channel != SENSORARRAY_TRANSPORT_CHANNEL_LIFECYCLE) ||
        descriptor->length == 0u ||
        descriptor->length > SENSORARRAY_TRANSPORT_POOL_TEXT_MAX) {
        if (pool) {
            pool->stats.staleDescriptor++;
        }
        return false;
    }

    const sensorarrayTransportPayloadSlot_t *slot =
        &pool->slots[descriptor->slotIndex];
    bool current = sensorarrayTransportPoolSlotReady(pool, descriptor->slotIndex) && slot->inUse &&
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
        !sensorarrayTransportPoolSlotReady(pool, descriptor->slotIndex)) {
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
    if (!sensorarrayTransportPoolSlotReady(pool, descriptor->slotIndex) || !slot->inUse ||
        slot->slotGeneration != descriptor->slotGeneration ||
        slot->channel != descriptor->channel ||
        slot->length != descriptor->length) {
        pool->stats.releaseMismatch++;
        return false;
    }

    slot->inUse = false;
    slot->bleConnectionGeneration = 0u;
        slot->wifiEpoch = 0u;
        slot->wifiDeadlineUs = 0u;
    slot->hasFrameMeta = false;
    slot->trustedFullFrame = false;
    slot->protectionFlags = 0u;
    slot->protectedWifiDone = false;
    slot->protectedWifiTarget = false;
    if (pool->stats.used != 0u) {
        pool->stats.used--;
    }
    return true;
}

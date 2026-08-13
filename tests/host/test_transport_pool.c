#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "sensorarrayTransportPolicy.h"
#include "sensorarrayTransportPool.h"

#define TEST_DESCRIPTOR_QUEUE_DEPTH 2u

typedef struct {
    sensorarrayTransportDescriptor_t items[TEST_DESCRIPTOR_QUEUE_DEPTH];
    uint8_t head;
    uint8_t count;
} testDescriptorQueue_t;

static bool testQueueSend(testDescriptorQueue_t *queue,
                          const sensorarrayTransportDescriptor_t *descriptor)
{
    if (!queue || !descriptor || queue->count >= TEST_DESCRIPTOR_QUEUE_DEPTH) {
        return false;
    }
    uint8_t tail = (uint8_t)((queue->head + queue->count) %
                             TEST_DESCRIPTOR_QUEUE_DEPTH);
    queue->items[tail] = *descriptor;
    queue->count++;
    return true;
}

static bool testQueueReceive(testDescriptorQueue_t *queue,
                             sensorarrayTransportDescriptor_t *outDescriptor)
{
    if (!queue || !outDescriptor || queue->count == 0u) {
        return false;
    }
    *outDescriptor = queue->items[queue->head];
    queue->head = (uint8_t)((queue->head + 1u) % TEST_DESCRIPTOR_QUEUE_DEPTH);
    queue->count--;
    return true;
}

static void testAllocateReleaseAndPriorityReserve(void)
{
    sensorarrayTransportPool_t pool;
    sensorarrayTransportPoolInit(&pool);

    sensorarrayTransportDescriptor_t logA;
    sensorarrayTransportDescriptor_t logB;
    sensorarrayTransportDescriptor_t logRejected;
    sensorarrayTransportDescriptor_t dataA;
    sensorarrayTransportDescriptor_t dataB;
    sensorarrayTransportDescriptor_t lifecycle;
    assert(sensorarrayTransportPoolAllocate(
        &pool, SENSORARRAY_TRANSPORT_CHANNEL_LOG, 100u, &logA));
    assert(sensorarrayTransportPoolAllocate(
        &pool, SENSORARRAY_TRANSPORT_CHANNEL_LOG, 101u, &logB));
    assert(!sensorarrayTransportPoolAllocate(
        &pool, SENSORARRAY_TRANSPORT_CHANNEL_LOG, 102u, &logRejected));

    /* A LOG burst cannot consume the two slots reserved for measurement DATA. */
    assert(sensorarrayTransportPoolAllocate(
        &pool, SENSORARRAY_TRANSPORT_CHANNEL_DATA, 200u, &dataA));
    assert(sensorarrayTransportPoolAllocate(
        &pool, SENSORARRAY_TRANSPORT_CHANNEL_DATA, 201u, &dataB));
    assert(sensorarrayTransportPoolAllocate(
        &pool, SENSORARRAY_TRANSPORT_CHANNEL_LIFECYCLE, 202u, &lifecycle));
    assert(pool.stats.used == 5u);
    assert(pool.stats.highWater == 5u);

    sensorarrayTransportPayloadSlot_t *slot =
        sensorarrayTransportPoolGetSlot(&pool, &dataA);
    assert(slot != NULL);
    memcpy(slot->data, "DATA", 4u);
    slot->data[4] = '\0';
    assert(strcmp(slot->data, "DATA") == 0);
    assert(sensorarrayTransportPoolValidate(&pool, &dataA));

    /* This models xQueueSend(..., 0) failure cleanup: ownership is released
     * immediately, so a full descriptor queue cannot leak a payload slot. */
    assert(sensorarrayTransportPoolRelease(&pool, &dataB));
    assert(pool.stats.used == 4u);
    assert(sensorarrayTransportPoolRelease(&pool, &dataA));
    assert(sensorarrayTransportPoolRelease(&pool, &logA));
    assert(sensorarrayTransportPoolRelease(&pool, &logB));
    assert(sensorarrayTransportPoolRelease(&pool, &lifecycle));
    assert(pool.stats.used == 0u);
}

static void testGenerationAndReleaseGuards(void)
{
    sensorarrayTransportPool_t pool;
    sensorarrayTransportPoolInit(&pool);
    sensorarrayTransportDescriptor_t oldDescriptor;
    assert(sensorarrayTransportPoolAllocate(
        &pool, SENSORARRAY_TRANSPORT_CHANNEL_DATA, 32u, &oldDescriptor));
    assert(sensorarrayTransportPoolRelease(&pool, &oldDescriptor));
    assert(!sensorarrayTransportPoolRelease(&pool, &oldDescriptor));
    assert(pool.stats.releaseMismatch == 1u);

    sensorarrayTransportDescriptor_t currentDescriptor;
    assert(sensorarrayTransportPoolAllocate(
        &pool, SENSORARRAY_TRANSPORT_CHANNEL_DATA, 32u, &currentDescriptor));
    assert(currentDescriptor.slotIndex == oldDescriptor.slotIndex);
    assert(currentDescriptor.slotGeneration != oldDescriptor.slotGeneration);
    assert(!sensorarrayTransportPoolValidate(&pool, &oldDescriptor));
    assert(pool.stats.staleDescriptor == 1u);

    sensorarrayTransportDescriptor_t invalidDescriptor = currentDescriptor;
    invalidDescriptor.slotIndex = SENSORARRAY_TRANSPORT_POOL_SLOT_COUNT;
    assert(!sensorarrayTransportPoolValidate(&pool, &invalidDescriptor));
    assert(pool.stats.staleDescriptor == 2u);
    assert(sensorarrayTransportPoolRelease(&pool, &currentDescriptor));
}

static void testQueueFullAndDisconnectCleanup(void)
{
    sensorarrayTransportPool_t pool;
    sensorarrayTransportPoolInit(&pool);
    testDescriptorQueue_t queue = {0};
    sensorarrayTransportDescriptor_t first;
    sensorarrayTransportDescriptor_t second;
    sensorarrayTransportDescriptor_t rejected;
    assert(sensorarrayTransportPoolAllocate(
        &pool, SENSORARRAY_TRANSPORT_CHANNEL_DATA, 40u, &first));
    assert(sensorarrayTransportPoolAllocate(
        &pool, SENSORARRAY_TRANSPORT_CHANNEL_DATA, 41u, &second));
    assert(sensorarrayTransportPoolAllocate(
        &pool, SENSORARRAY_TRANSPORT_CHANNEL_DATA, 42u, &rejected));
    assert(testQueueSend(&queue, &first));
    assert(testQueueSend(&queue, &second));
    assert(!testQueueSend(&queue, &rejected));
    /* Firmware mirrors this exact non-blocking failure ownership rule. */
    assert(sensorarrayTransportPoolRelease(&pool, &rejected));

    sensorarrayTransportPayloadSlot_t *firstSlot =
        sensorarrayTransportPoolGetSlot(&pool, &first);
    assert(firstSlot != NULL);
    firstSlot->bleConnectionGeneration = 7u;
    /* Connection A disconnected and B is now generation 8: A's payload must
     * be treated as stale, then released while draining the queue. */
    assert(firstSlot->bleConnectionGeneration != 8u);

    sensorarrayTransportDescriptor_t drained;
    while (testQueueReceive(&queue, &drained)) {
        assert(sensorarrayTransportPoolRelease(&pool, &drained));
    }
    assert(pool.stats.used == 0u);
    assert(queue.count == 0u);
}

static void testBleChannelPolicy(void)
{
    for (unsigned dataEnabled = 0u; dataEnabled <= 1u; ++dataEnabled) {
        for (unsigned logEnabled = 0u; logEnabled <= 1u; ++logEnabled) {
            assert(sensorarrayTransportBlePolicyAllows(
                       true, true, dataEnabled != 0u, logEnabled != 0u,
                       SENSORARRAY_TRANSPORT_CHANNEL_DATA) ==
                   (dataEnabled != 0u));
            assert(sensorarrayTransportBlePolicyAllows(
                       true, true, dataEnabled != 0u, logEnabled != 0u,
                       SENSORARRAY_TRANSPORT_CHANNEL_LOG) ==
                   (logEnabled != 0u));
        }
    }
    assert(!sensorarrayTransportBlePolicyAllows(
        false, true, true, true, SENSORARRAY_TRANSPORT_CHANNEL_DATA));
    assert(!sensorarrayTransportBlePolicyAllows(
        true, false, true, true, SENSORARRAY_TRANSPORT_CHANNEL_LOG));
    assert(!sensorarrayTransportBlePolicyAllows(
        true, true, true, true, SENSORARRAY_TRANSPORT_CHANNEL_CTRL));
}

int main(void)
{
    testAllocateReleaseAndPriorityReserve();
    testGenerationAndReleaseGuards();
    testQueueFullAndDisconnectCleanup();
    testBleChannelPolicy();
    printf("TRANSPORT_POOL_TESTS,passed=1,slots=%u,descriptorBytes=%u,slotBytes=%u\n",
           (unsigned)SENSORARRAY_TRANSPORT_POOL_SLOT_COUNT,
           (unsigned)sizeof(sensorarrayTransportDescriptor_t),
           (unsigned)sizeof(sensorarrayTransportPayloadSlot_t));
    return 0;
}

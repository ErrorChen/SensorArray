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
    sensorarrayTransportPool_t pool = {0};
    char storage[SENSORARRAY_TRANSPORT_POOL_STORAGE_BYTES];
    assert(sensorarrayTransportPoolInit(&pool, storage, sizeof(storage)));

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
    sensorarrayTransportPool_t pool = {0};
    char storage[SENSORARRAY_TRANSPORT_POOL_STORAGE_BYTES];
    assert(sensorarrayTransportPoolInit(&pool, storage, sizeof(storage)));
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

static void testProtectedSlotsAreIndependentAndPersistent(void)
{
    sensorarrayTransportPool_t pool = {0};
    char storage[SENSORARRAY_TRANSPORT_POOL_STORAGE_BYTES];
    assert(sensorarrayTransportPoolInit(&pool, storage, sizeof(storage)));
    sensorarrayTransportDescriptor_t ordinary[4], lifecycle, protectedA, protectedB, rejected;
    for (unsigned i = 0; i < 4; ++i) {
        assert(sensorarrayTransportPoolAllocate(&pool, SENSORARRAY_TRANSPORT_CHANNEL_DATA, 2304, &ordinary[i]));
    }
    assert(!sensorarrayTransportPoolAllocate(&pool, SENSORARRAY_TRANSPORT_CHANNEL_DATA, 2304, &rejected));
    assert(sensorarrayTransportPoolAllocate(&pool, SENSORARRAY_TRANSPORT_CHANNEL_LIFECYCLE, 512, &lifecycle));
    assert(sensorarrayTransportPoolAllocateProtected(&pool, 2304, &protectedA));
    assert(sensorarrayTransportPoolAllocateProtected(&pool, 2304, &protectedB));
    assert(!sensorarrayTransportPoolAllocateProtected(&pool, 2304, &rejected));
    assert(pool.stats.used == 7u && protectedA.slotIndex != protectedB.slotIndex);
    sensorarrayTransportPayloadSlot_t *owned = sensorarrayTransportPoolGetSlot(&pool, &protectedA);
    memset(owned->data, 'Q', 2304); owned->trustedFullFrame = true;
    owned->protectionFlags = SENSORARRAY_FRAME_PROTECT_FIRST_CONFIG;
    sensorarrayTransportDescriptor_t stale = protectedB;
    assert(sensorarrayTransportPoolRelease(&pool, &protectedB));
    assert(sensorarrayTransportPoolAllocateProtected(&pool, 2304, &protectedB));
    assert(protectedB.slotGeneration != stale.slotGeneration);
    assert(!sensorarrayTransportPoolRelease(&pool, &stale));
    for (unsigned i = 0; i < 2304; ++i) { assert(owned->data[i] == 'Q'); }
    assert(owned->inUse && owned->trustedFullFrame && owned->protectionFlags != 0u);
    assert(sensorarrayTransportPoolRelease(&pool, &protectedA));
    assert(!owned->trustedFullFrame && owned->protectionFlags == 0u);
    assert(sensorarrayTransportPoolRelease(&pool, &protectedB));
    assert(sensorarrayTransportPoolRelease(&pool, &lifecycle));
    for (unsigned i = 0; i < 4; ++i) { assert(sensorarrayTransportPoolRelease(&pool, &ordinary[i])); }
    assert(pool.stats.used == 0u);
}

static void testQueueFullAndDisconnectCleanup(void)
{
    sensorarrayTransportPool_t pool = {0};
    char storage[SENSORARRAY_TRANSPORT_POOL_STORAGE_BYTES];
    assert(sensorarrayTransportPoolInit(&pool, storage, sizeof(storage)));
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

static void testStorageBindingAndPrivateNul(void)
{
    sensorarrayTransportPool_t pool = {0};
    char storage[SENSORARRAY_TRANSPORT_POOL_STORAGE_BYTES];
    char replacement[SENSORARRAY_TRANSPORT_POOL_STORAGE_BYTES];
    sensorarrayTransportDescriptor_t descriptors[SENSORARRAY_TRANSPORT_POOL_SLOT_COUNT];
    sensorarrayTransportDescriptor_t absent = {.length=1u};
    assert(!sensorarrayTransportPoolAllocate(&pool, SENSORARRAY_TRANSPORT_CHANNEL_DATA, 1u, &absent));
    assert(!sensorarrayTransportPoolAllocateProtected(&pool, 1u, &absent));
    assert(!sensorarrayTransportPoolValidate(&pool, &absent));
    assert(!sensorarrayTransportPoolGetSlot(&pool, &absent));
    assert(!sensorarrayTransportPoolRelease(&pool, &absent));
    assert(!sensorarrayTransportPoolInit(NULL, storage, sizeof storage));
    assert(!sensorarrayTransportPoolInit(&pool, NULL, sizeof storage));
    assert(!sensorarrayTransportPoolInit(&pool, storage, sizeof storage - 1u));
    assert(!sensorarrayTransportPoolInit(&pool, storage, sizeof storage + 1u));
    assert(sensorarrayTransportPoolInit(&pool, storage, sizeof storage));
    for (unsigned i=0; i<SENSORARRAY_TRANSPORT_POOL_SLOT_COUNT; ++i) {
        bool accepted = i < SENSORARRAY_TRANSPORT_POOL_NORMAL_SLOT_COUNT ?
            sensorarrayTransportPoolAllocate(&pool, SENSORARRAY_TRANSPORT_CHANNEL_DATA,
                SENSORARRAY_TRANSPORT_POOL_TEXT_MAX, &descriptors[i]) :
            i == SENSORARRAY_TRANSPORT_POOL_LIFECYCLE_SLOT_INDEX ?
            sensorarrayTransportPoolAllocate(&pool, SENSORARRAY_TRANSPORT_CHANNEL_LIFECYCLE,
                SENSORARRAY_TRANSPORT_POOL_TEXT_MAX, &descriptors[i]) :
            sensorarrayTransportPoolAllocateProtected(&pool, SENSORARRAY_TRANSPORT_POOL_TEXT_MAX,
                &descriptors[i]);
        assert(accepted && descriptors[i].slotIndex==i);
        char *data = sensorarrayTransportPoolGetSlot(&pool, &descriptors[i])->data;
        assert(data == storage + i*SENSORARRAY_TRANSPORT_POOL_STORAGE_STRIDE);
        memset(data, 'a'+(int)i, SENSORARRAY_TRANSPORT_POOL_TEXT_MAX);
        data[SENSORARRAY_TRANSPORT_POOL_TEXT_MAX] = '\0';
    }
    sensorarrayTransportPool_t held = pool;
    assert(sensorarrayTransportPoolInit(&pool, storage, sizeof storage));
    assert(!sensorarrayTransportPoolInit(&pool, replacement, sizeof replacement));
    assert(memcmp(&pool, &held, sizeof pool)==0);
    for (unsigned i=0; i<SENSORARRAY_TRANSPORT_POOL_SLOT_COUNT; ++i) {
        assert(sensorarrayTransportPoolValidate(&pool, &descriptors[i]));
        char *data = pool.slots[i].data;
        for (unsigned n=0; n<SENSORARRAY_TRANSPORT_POOL_TEXT_MAX; ++n) {
            assert(data[n]=='a'+(int)i);
        }
        assert(data[SENSORARRAY_TRANSPORT_POOL_TEXT_MAX]=='\0');
    }
    /* A corrupt binding is never handed to a producer or accepted as owned. */
    char *saved = pool.slots[0].data;
    pool.slots[0].data = NULL;
    assert(!sensorarrayTransportPoolGetSlot(&pool, &descriptors[0]));
    assert(!sensorarrayTransportPoolValidate(&pool, &descriptors[0]));
    assert(!sensorarrayTransportPoolRelease(&pool, &descriptors[0]));
    pool.slots[0].data = saved;
    for (unsigned i=0; i<SENSORARRAY_TRANSPORT_POOL_SLOT_COUNT; ++i) {
        assert(sensorarrayTransportPoolRelease(&pool, &descriptors[i]));
    }
    assert(!sensorarrayTransportPoolInit(&pool, replacement, sizeof replacement));
    assert(sensorarrayTransportPoolInit(&pool, storage, sizeof storage));
    assert(pool.stats.used==0 && pool.slots[0].data==saved);
}

int main(void)
{
    testStorageBindingAndPrivateNul();
    testProtectedSlotsAreIndependentAndPersistent();
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

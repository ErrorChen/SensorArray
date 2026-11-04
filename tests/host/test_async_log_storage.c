#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "sensorarrayAsyncLogStorage.h"

static bool psramAvailable, internalAvailable;
static unsigned allocations;
static uint32_t capabilities[8];

void *heap_caps_calloc(size_t count, size_t size, uint32_t caps)
{
    assert(count == 1u && size != 0u && (caps & MALLOC_CAP_8BIT));
    assert(allocations < 8u);
    capabilities[allocations++] = caps;
    if (((caps & MALLOC_CAP_SPIRAM) && psramAvailable) ||
        ((caps & MALLOC_CAP_INTERNAL) && internalAvailable)) {
        return calloc(count, size);
    }
    return NULL;
}

static void reset(bool psram, bool internal)
{
    psramAvailable = psram;
    internalAvailable = internal;
    allocations = 0u;
    memset(capabilities, 0, sizeof(capabilities));
}

int main(void)
{
    const size_t bytes = 13536u;
    sensorarrayAsyncLogStorage_t storage = {0};
    reset(true, true);
    assert(sensorarrayAsyncLogStorageEnsure(&storage, bytes, true));
    assert(storage.memory && storage.usesPsram && storage.bytes == bytes);
    assert(allocations == 1u && capabilities[0] == (MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    for (size_t i = 0u; i < bytes; ++i) { assert(((unsigned char *)storage.memory)[i] == 0u); }
    void *original = storage.memory;
    memset(original, 0xA5, bytes);
    /* Retry after task-create failure / repeated calls never allocate or clear. */
    for (unsigned i = 0u; i < 10000u; ++i) {
        assert(sensorarrayAsyncLogStorageEnsure(&storage, bytes, false));
    }
    assert(allocations == 1u && storage.memory == original && storage.usesPsram);
    assert(((unsigned char *)storage.memory)[bytes - 1u] == 0xA5);
    assert(!sensorarrayAsyncLogStorageEnsure(&storage, bytes + 1u, true));
    assert(allocations == 1u && storage.memory == original);
    free(storage.memory);

    storage = (sensorarrayAsyncLogStorage_t){0}; reset(false, true);
    assert(sensorarrayAsyncLogStorageEnsure(&storage, bytes, true));
    assert(!storage.usesPsram && allocations == 2u);
    assert(capabilities[1] == (MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
    free(storage.memory);

    storage = (sensorarrayAsyncLogStorage_t){0}; reset(false, false);
    assert(!sensorarrayAsyncLogStorageEnsure(&storage, bytes, true));
    assert(!storage.memory && storage.bytes == 0u && !storage.usesPsram && allocations == 2u);
    psramAvailable = true; /* memory later available: bounded retry succeeds */
    assert(sensorarrayAsyncLogStorageEnsure(&storage, bytes, true));
    assert(storage.memory && storage.usesPsram && allocations == 3u);
    assert(sensorarrayAsyncLogStorageEnsure(&storage, bytes, true) && allocations == 3u);
    free(storage.memory);

    storage = (sensorarrayAsyncLogStorage_t){0}; reset(true, true);
    assert(sensorarrayAsyncLogStorageEnsure(&storage, bytes, false));
    assert(!storage.usesPsram && allocations == 1u);
    assert(capabilities[0] == (MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
    free(storage.memory);
    /* First pool succeeds, second fails: retry must only allocate the missing
     * pool, preserving the first pool and any data already stored there. */
    sensorarrayAsyncLogStorage_t workspace = {0}, framePool = {0};
    reset(true, false);
    assert(sensorarrayAsyncLogStorageEnsure(&workspace, bytes, true));
    original = workspace.memory;
    memset(original, 0x5A, bytes);
    psramAvailable = false;
    assert(!sensorarrayAsyncLogStorageEnsure(&framePool, 28608u, true));
    assert(allocations == 3u && !framePool.memory);
    internalAvailable = true;
    assert(sensorarrayAsyncLogStorageEnsure(&workspace, bytes, true));
    assert(allocations == 3u && workspace.memory == original);
    assert(sensorarrayAsyncLogStorageEnsure(&framePool, 28608u, true));
    assert(allocations == 5u && framePool.memory && !framePool.usesPsram);
    assert(((unsigned char *)workspace.memory)[bytes - 1u] == 0x5A);
    assert(((unsigned char *)framePool.memory)[28607u] == 0u);
    for (unsigned i = 0u; i < 10000u; ++i) {
        assert(sensorarrayAsyncLogStorageEnsure(&workspace, bytes, true));
        assert(sensorarrayAsyncLogStorageEnsure(&framePool, 28608u, true));
    }
    assert(allocations == 5u);
    free(workspace.memory); free(framePool.memory);
    assert(!sensorarrayAsyncLogStorageEnsure(NULL, bytes, true));
    storage = (sensorarrayAsyncLogStorage_t){0};
    assert(!sensorarrayAsyncLogStorageEnsure(&storage, 0u, true));
    puts("ASYNC_LOG_STORAGE_TESTS,passed=1,psram=1,fallback=1,allfail=1,retry=1,noHotAlloc=1,secondPoolRetry=1");
    return 0;
}

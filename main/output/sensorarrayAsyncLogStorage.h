#pragma once

#include <stdbool.h>
#include <stddef.h>
#include "esp_heap_caps.h"

/* 本轮新增: startup-owned, permanent CPU-only formatting/frame storage. No DMA,
 * queue control, or critical-section object may be placed in this allocation. */
typedef struct {
    void *memory;
    size_t bytes;
    bool usesPsram;
} sensorarrayAsyncLogStorage_t;

/* Called only by the single startup owner, before the log task is created.
 * A failed allocation leaves a retryable empty state. A successful allocation
 * is never replaced (including after a subsequent task-create failure). */
static inline bool sensorarrayAsyncLogStorageEnsure(
    sensorarrayAsyncLogStorage_t *storage, size_t bytes, bool preferPsram)
{
    if (!storage || bytes == 0u) {
        return false;
    }
    if (storage->memory) {
        return storage->bytes == bytes;
    }
    void *memory = NULL;
    bool usesPsram = false;
    if (preferPsram) {
        memory = heap_caps_calloc(1u, bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        usesPsram = memory != NULL;
    }
    if (!memory) {
        memory = heap_caps_calloc(1u, bytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    }
    if (!memory) {
        return false;
    }
    storage->memory = memory;
    storage->bytes = bytes;
    storage->usesPsram = usesPsram;
    return true;
}

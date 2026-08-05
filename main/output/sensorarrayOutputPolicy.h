#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    SENSORARRAY_OUTPUT_CONGESTION_USE_FREE_SLOT = 0,
    SENSORARRAY_OUTPUT_CONGESTION_RECLAIM_OLDEST,
    SENSORARRAY_OUTPUT_CONGESTION_DROP_INCOMING,
} sensorarrayOutputCongestionDecision_t;

/* Pure decision point shared by the production TextFrameBus and host tests.
 * Reclaim is permitted only when a queued frame really exists; a slot that is
 * merely reserved by a producer must never be stolen. */
static inline sensorarrayOutputCongestionDecision_t
sensorarrayOutputCongestionDecide(bool freeSlotAvailable,
                                  bool queuedFrameAvailable,
                                  bool dropOldestEnabled)
{
    if (freeSlotAvailable) {
        return SENSORARRAY_OUTPUT_CONGESTION_USE_FREE_SLOT;
    }
    if (dropOldestEnabled && queuedFrameAvailable) {
        return SENSORARRAY_OUTPUT_CONGESTION_RECLAIM_OLDEST;
    }
    return SENSORARRAY_OUTPUT_CONGESTION_DROP_INCOMING;
}

#ifdef __cplusplus
}
#endif


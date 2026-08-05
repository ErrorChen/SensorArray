#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int logicalSource;
    int swLevel;
    int selaLevel;
    int selbLevel;
} sensorarrayRouteExpectedControl_t;

typedef struct {
    int commandedSource;
    int commandedSwLevel;
    int observedSwLevel;
    int commandedSelaLevel;
    int observedSelaLevel;
    int commandedSelbLevel;
    int observedSelbLevel;
} sensorarrayRouteObservedControl_t;

/* Pure comparison used by the hardware controller and fault-injection tests.
 * An MCU-side observation cannot prove analogue conduction, but any mismatch
 * is sufficient to reject the route before a sample can become valid. */
static inline bool sensorarrayRouteControlReadbackMatches(
    const sensorarrayRouteExpectedControl_t *expected,
    const sensorarrayRouteObservedControl_t *observed)
{
    return expected && observed &&
           observed->commandedSource == expected->logicalSource &&
           observed->commandedSwLevel == expected->swLevel &&
           observed->observedSwLevel == expected->swLevel &&
           observed->commandedSelaLevel == expected->selaLevel &&
           observed->observedSelaLevel == expected->selaLevel &&
           observed->commandedSelbLevel == expected->selbLevel &&
           observed->observedSelbLevel == expected->selbLevel;
}

#ifdef __cplusplus
}
#endif


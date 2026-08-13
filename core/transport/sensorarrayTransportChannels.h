#pragma once

typedef enum {
    SENSORARRAY_TRANSPORT_CHANNEL_DATA = 0,
    SENSORARRAY_TRANSPORT_CHANNEL_LOG = 1,
    SENSORARRAY_TRANSPORT_CHANNEL_CTRL = 2,
    /* Deferred protocol lifecycle events share the LOG sinks but have their
     * own bounded queue and a reserved payload slot. */
    SENSORARRAY_TRANSPORT_CHANNEL_LIFECYCLE = 3,
} sensorarrayTransportChannel_t;

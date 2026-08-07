#include "sensorarrayTransportPolicy.h"

bool sensorarrayTransportBlePolicyAllows(
    bool streamWantsBle,
    bool connected,
    bool dataChannelSendable,
    bool logChannelSendable,
    sensorarrayTransportChannel_t channel)
{
    if (!streamWantsBle || !connected) {
        return false;
    }
    if (channel == SENSORARRAY_TRANSPORT_CHANNEL_DATA) {
        return dataChannelSendable;
    }
    if (channel == SENSORARRAY_TRANSPORT_CHANNEL_LOG) {
        return logChannelSendable;
    }
    return false;
}

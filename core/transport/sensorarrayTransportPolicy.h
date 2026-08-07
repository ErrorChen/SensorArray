#pragma once

#include <stdbool.h>

#include "sensorarrayTransportChannels.h"

/* Pure, host-testable channel policy. The caller supplies the active stream
 * decision and the per-characteristic CCCD state; this function deliberately
 * cannot collapse DATA and LOG into a global "any subscription" flag. */
bool sensorarrayTransportBlePolicyAllows(
    bool streamWantsBle,
    bool connected,
    bool dataChannelSendable,
    bool logChannelSendable,
    sensorarrayTransportChannel_t channel);

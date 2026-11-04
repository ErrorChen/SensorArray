#pragma once
#include "sensorarrayTransportPool.h"
/* Added in this repair. Parse header fields only, never current global config. */
void sensorarrayTransportParseFrameMeta(sensorarrayTransportPayloadSlot_t *slot);

bool sensorarrayTransportBuildPreview(const sensorarrayTransportPayloadSlot_t *slot, char *out, size_t outSize);

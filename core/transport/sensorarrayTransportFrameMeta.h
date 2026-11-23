#pragma once
#include "sensorarrayTransportPool.h"
/* Added in this repair. Parse header fields only, never current global config. */
void sensorarrayTransportParseFrameMeta(sensorarrayTransportPayloadSlot_t *slot);

/* 本轮新增: constant-size producer metadata checks/copy, not a wire/CRC
 * validator. Only the successful real encoder's packet/frame pair may use it. */
bool sensorarrayTransportFrameMetadataValid(const sensorarrayTransportFrameMetadata_t *metadata,
    const char *data, size_t length);
bool sensorarrayTransportApplyFrameMetadata(sensorarrayTransportPayloadSlot_t *slot,
    const sensorarrayTransportFrameMetadata_t *metadata);

bool sensorarrayTransportBuildPreview(const sensorarrayTransportPayloadSlot_t *slot, char *out, size_t outSize);

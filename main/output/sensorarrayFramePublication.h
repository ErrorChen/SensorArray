#pragma once

#include "sensorarrayFrame.h"
#include "sensorarrayTransportFrameIdentity.h"

/* 本轮新增: preserve the acquisition snapshot; do not query current globals. */
static inline sensorarrayTransportFrameIdentity_t sensorarrayFramePublicationIdentity(
    const sensorarrayFrame_t *frame)
{
    const sensorarrayConfigIdentity_t *source = &frame->identity;
    return (sensorarrayTransportFrameIdentity_t){
        .sessionId = source->sessionId, .bootId = source->bootId,
        .revision = source->revision, .sequence = frame->sequence,
        .modeGeneration = source->modeGeneration, .modeRequestId = source->modeRequestId,
        .rowsGeneration = source->rowsGeneration, .rowsRequestId = source->rowsRequestId,
        .profileGeneration = source->profileGeneration, .profileRequestId = source->profileRequestId,
        .rows = frame->activeRows,
    };
}

/* 本轮新增: tag comes from the successful encoder packet, and every value
 * comes from that packet's source frame. This does not parse or rewrite wire
 * bytes, nor establish that an arbitrary caller-provided packet is valid. */
static inline bool sensorarrayFramePublicationMetadata(
    const sensorarrayFrame_t *frame, char tag, sensorarrayTransportFrameMetadata_t *out)
{
    if (!frame || !out || frame->activeRows < 1u || frame->activeRows > 8u) {
        return false;
    }
    char expected = frame->mixedProfile ? 'M' :
        frame->measurement.mode == SENSORARRAY_MEASUREMENT_MODE_VOLTAGE ? 'V' :
        frame->measurement.mode == SENSORARRAY_MEASUREMENT_MODE_RESISTANCE ? 'R' : 'C';
    if (tag != expected) { return false; }
    bool ads = tag == 'V' || tag == 'R';
    *out = (sensorarrayTransportFrameMetadata_t){
        .identity = sensorarrayFramePublicationIdentity(frame),
        .legacyGeneration = ads ? frame->measurement.modeGeneration : frame->configSnapshot.generation,
        .legacyRequestId = ads ? frame->measurement.modeRequestId : frame->configSnapshot.requestId,
        .mixedProfileGeneration = frame->rowProfileGeneration,
        .mixedProfileRequestId = frame->rowProfileRequestId,
        .tag = tag,
    };
    return true;
}

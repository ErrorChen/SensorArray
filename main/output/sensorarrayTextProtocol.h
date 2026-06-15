#pragma once

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#include "sensorarrayFrame.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SENSORARRAY_TEXT_PACKET_MAX 1536u

typedef struct {
    uint32_t sequence;
    uint16_t length;
    char data[SENSORARRAY_TEXT_PACKET_MAX];
} sensorarrayTextPacket_t;

esp_err_t sensorarrayTextProtocolBuildCapFrame(const sensorarrayFrame_t *frame,
                                                sensorarrayTextPacket_t *outPacket);

#ifdef __cplusplus
}
#endif

#pragma once

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#include "sensorarrayFrame.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SENSORARRAY_TEXT_PACKET_MAX 1536u
#define SENSORARRAY_TEXT_MEASUREMENT_VALUE_MAX INT64_C(999999999999)

/* Worst legal frame: maximum sequence/timestamp/generation fields, four
 * 16-cell D chunks with signed 12-digit fixed-point values, four packed PGA
 * chunks, the K trailer, and the expected/acquired acquisition masks. */
#define SENSORARRAY_TEXT_MEASUREMENT_WORST_CASE 1522u
_Static_assert(SENSORARRAY_TEXT_MEASUREMENT_WORST_CASE <=
                   SENSORARRAY_TEXT_PACKET_MAX,
               "measurement text frame does not fit the fixed TextFrameBus slot");

typedef struct {
    uint32_t sequence;
    uint16_t length;
    char data[SENSORARRAY_TEXT_PACKET_MAX];
} sensorarrayTextPacket_t;

esp_err_t sensorarrayTextProtocolBuildCapFrame(const sensorarrayFrame_t *frame,
                                                sensorarrayTextPacket_t *outPacket);
esp_err_t sensorarrayTextProtocolBuildMeasurementFrame(
    const sensorarrayFrame_t *frame,
    sensorarrayTextPacket_t *outPacket);
esp_err_t sensorarrayTextProtocolBuildMixedFrame(
    const sensorarrayFrame_t *frame,
    sensorarrayTextPacket_t *outPacket);
esp_err_t sensorarrayTextProtocolBuildFrame(const sensorarrayFrame_t *frame,
                                            sensorarrayTextPacket_t *outPacket);
bool sensorarrayTextProtocolSelfTest(sensorarrayFrame_t *scratchFrame,
                                     uint32_t *outChecks);

#ifdef __cplusplus
}
#endif

#pragma once

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define PROTOCOL_WIRE_POINT_COUNT 64u
// Upper 4 bits of each 32-bit data word carry the tag.
#define PROTOCOL_WIRE_TAG_BITS 4u
#define PROTOCOL_WIRE_PAYLOAD_BITS 28u
#define PROTOCOL_WIRE_TAG_SHIFT 28u
#define PROTOCOL_WIRE_TAG_MASK 0x0Fu
#define PROTOCOL_WIRE_PAYLOAD_MASK 0x0FFFFFFFu

#define PROTOCOL_WIRE_FRAME_BYTES \
    (2u + 4u + 8u + (PROTOCOL_WIRE_POINT_COUNT * 2u) + (PROTOCOL_WIRE_POINT_COUNT * 4u) + 2u)

typedef enum {
    PROTOCOL_WIRE_DATA_TAG_NONE = 0x0u,
    PROTOCOL_WIRE_DATA_TAG_FDC2214 = 0x1u,
} protocolWireDataTag_t;

typedef struct {
    uint16_t seq;
    uint32_t t0;
    uint64_t validMask;
    uint16_t offset[PROTOCOL_WIRE_POINT_COUNT];
    uint32_t data[PROTOCOL_WIRE_POINT_COUNT];
} protocolWireFrame_t;

static inline uint32_t protocolWirePackTaggedU28(uint8_t tag, uint32_t payload28)
{
    return ((uint32_t)(tag & PROTOCOL_WIRE_TAG_MASK) << PROTOCOL_WIRE_TAG_SHIFT)
        | (payload28 & PROTOCOL_WIRE_PAYLOAD_MASK);
}

static inline uint8_t protocolWireGetTag(uint32_t value)
{
    return (uint8_t)((value >> PROTOCOL_WIRE_TAG_SHIFT) & PROTOCOL_WIRE_TAG_MASK);
}

static inline uint32_t protocolWireGetPayload(uint32_t value)
{
    return value & PROTOCOL_WIRE_PAYLOAD_MASK;
}

// Pack a frame into outBuf in little-endian order. CRC16 is CCITT-FALSE (0x1021, init 0xFFFF).
// outLen must be at least PROTOCOL_WIRE_FRAME_BYTES.
esp_err_t protocolWirePackFrame(const protocolWireFrame_t *frame, uint8_t *outBuf, size_t outLen);

#ifdef __cplusplus
}
#endif

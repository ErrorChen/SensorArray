#include "protocolWire.h"

#include "sdkconfig.h"

#ifndef CONFIG_PROTOCOL_ENABLE_CRC16
#define CONFIG_PROTOCOL_ENABLE_CRC16 1
#endif

static void protocolWireWriteU16LE(uint8_t *dst, uint16_t value)
{
    dst[0] = (uint8_t)(value & 0xFFu);
    dst[1] = (uint8_t)((value >> 8) & 0xFFu);
}

static void protocolWireWriteU32LE(uint8_t *dst, uint32_t value)
{
    dst[0] = (uint8_t)(value & 0xFFu);
    dst[1] = (uint8_t)((value >> 8) & 0xFFu);
    dst[2] = (uint8_t)((value >> 16) & 0xFFu);
    dst[3] = (uint8_t)((value >> 24) & 0xFFu);
}

static void protocolWireWriteU64LE(uint8_t *dst, uint64_t value)
{
    dst[0] = (uint8_t)(value & 0xFFu);
    dst[1] = (uint8_t)((value >> 8) & 0xFFu);
    dst[2] = (uint8_t)((value >> 16) & 0xFFu);
    dst[3] = (uint8_t)((value >> 24) & 0xFFu);
    dst[4] = (uint8_t)((value >> 32) & 0xFFu);
    dst[5] = (uint8_t)((value >> 40) & 0xFFu);
    dst[6] = (uint8_t)((value >> 48) & 0xFFu);
    dst[7] = (uint8_t)((value >> 56) & 0xFFu);
}

#if CONFIG_PROTOCOL_ENABLE_CRC16
static uint16_t protocolWireCrc16(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFFu;
    for (size_t i = 0; i < len; ++i) {
        crc ^= (uint16_t)data[i] << 8;
        for (int bit = 0; bit < 8; ++bit) {
            if (crc & 0x8000u) {
                crc = (uint16_t)((crc << 1) ^ 0x1021u);
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}
#endif

esp_err_t protocolWirePackFrame(const protocolWireFrame_t *frame, uint8_t *outBuf, size_t outLen)
{
    if (!frame || !outBuf) {
        return ESP_ERR_INVALID_ARG;
    }
    if (outLen < PROTOCOL_WIRE_FRAME_BYTES) {
        return ESP_ERR_INVALID_SIZE;
    }

    uint8_t *p = outBuf;
    protocolWireWriteU16LE(p, frame->seq);
    p += 2;
    protocolWireWriteU32LE(p, frame->t0);
    p += 4;
    protocolWireWriteU64LE(p, frame->validMask);
    p += 8;
    for (size_t i = 0; i < PROTOCOL_WIRE_POINT_COUNT; ++i) {
        protocolWireWriteU16LE(p, frame->offset[i]);
        p += 2;
    }
    for (size_t i = 0; i < PROTOCOL_WIRE_POINT_COUNT; ++i) {
        protocolWireWriteU32LE(p, frame->data[i]);
        p += 4;
    }

#if CONFIG_PROTOCOL_ENABLE_CRC16
    uint16_t crc = protocolWireCrc16(outBuf, PROTOCOL_WIRE_FRAME_BYTES - 2u);
#else
    uint16_t crc = 0u;
#endif
    protocolWireWriteU16LE(p, crc);

    return ESP_OK;
}

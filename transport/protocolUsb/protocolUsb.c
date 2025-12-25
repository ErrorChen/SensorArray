#include "protocolUsb.h"

#include <stdbool.h>
#include <string.h>

#define PROTOCOL_USB_SYNC_LSB ((uint8_t)(PROTOCOL_USB_SYNC_WORD & 0xFFu))
#define PROTOCOL_USB_SYNC_MSB ((uint8_t)((PROTOCOL_USB_SYNC_WORD >> 8) & 0xFFu))

static void protocolUsbWriteU16LE(uint8_t *dst, uint16_t value)
{
    dst[0] = (uint8_t)(value & 0xFFu);
    dst[1] = (uint8_t)((value >> 8) & 0xFFu);
}

static uint16_t protocolUsbReadU16LE(const uint8_t *src)
{
    return (uint16_t)src[0] | (uint16_t)((uint16_t)src[1] << 8);
}

static uint16_t protocolUsbCrc16(const uint8_t *data, size_t len)
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

static size_t protocolUsbFindSync(const uint8_t *buf, size_t len)
{
    if (len < 2u) {
        return (size_t)-1;
    }
    for (size_t i = 0; i + 1u < len; ++i) {
        if (buf[i] == PROTOCOL_USB_SYNC_LSB && buf[i + 1u] == PROTOCOL_USB_SYNC_MSB) {
            return i;
        }
    }
    return (size_t)-1;
}

esp_err_t protocolUsbBuildFrame(const uint8_t *payload,
                                size_t payloadLen,
                                uint8_t *outBuf,
                                size_t outLen)
{
    if (!payload || !outBuf) {
        return ESP_ERR_INVALID_ARG;
    }
    if (payloadLen == 0u || payloadLen > PROTOCOL_USB_MAX_PAYLOAD_BYTES) {
        return ESP_ERR_INVALID_SIZE;
    }
    if (outLen < (PROTOCOL_USB_HEADER_BYTES + payloadLen)) {
        return ESP_ERR_INVALID_SIZE;
    }

    protocolUsbWriteU16LE(outBuf, PROTOCOL_USB_SYNC_WORD);
    protocolUsbWriteU16LE(outBuf + 2u, (uint16_t)payloadLen);
    uint16_t crc = protocolUsbCrc16(payload, payloadLen);
    protocolUsbWriteU16LE(outBuf + 4u, crc);
    memcpy(outBuf + PROTOCOL_USB_HEADER_BYTES, payload, payloadLen);

    return ESP_OK;
}

void protocolUsbParserInit(protocolUsbParser_t *parser,
                           protocolUsbFrameHandler_t onFrame,
                           void *userCtx)
{
    if (!parser) {
        return;
    }
    memset(parser, 0, sizeof(*parser));
    parser->onFrame = onFrame;
    parser->userCtx = userCtx;
}

void protocolUsbParserReset(protocolUsbParser_t *parser)
{
    if (!parser) {
        return;
    }
    parser->bufferedLen = 0u;
    parser->badCrcCount = 0u;
    parser->badLenCount = 0u;
    parser->dropCount = 0u;
}

size_t protocolUsbParserFeed(protocolUsbParser_t *parser, const uint8_t *data, size_t len)
{
    if (!parser || (!data && len > 0u)) {
        return 0u;
    }

    size_t frames = 0u;
    size_t offset = 0u;
    while (offset < len) {
        size_t space = sizeof(parser->buffer) - parser->bufferedLen;
        if (space == 0u) {
            parser->buffer[0] = parser->buffer[parser->bufferedLen - 1u];
            parser->bufferedLen = 1u;
            parser->dropCount += 1u;
            space = sizeof(parser->buffer) - parser->bufferedLen;
        }

        size_t chunk = len - offset;
        if (chunk > space) {
            chunk = space;
        }

        memcpy(parser->buffer + parser->bufferedLen, data + offset, chunk);
        parser->bufferedLen += chunk;
        offset += chunk;

        bool progress = true;
        while (progress) {
            progress = false;

            size_t syncIndex = protocolUsbFindSync(parser->buffer, parser->bufferedLen);
            if (syncIndex == (size_t)-1) {
                if (parser->bufferedLen > 1u) {
                    parser->dropCount += parser->bufferedLen - 1u;
                    parser->buffer[0] = parser->buffer[parser->bufferedLen - 1u];
                    parser->bufferedLen = 1u;
                    progress = true;
                }
                break;
            }

            if (syncIndex > 0u) {
                parser->dropCount += syncIndex;
                memmove(parser->buffer, parser->buffer + syncIndex, parser->bufferedLen - syncIndex);
                parser->bufferedLen -= syncIndex;
                progress = true;
            }

            if (parser->bufferedLen < PROTOCOL_USB_HEADER_BYTES) {
                break;
            }

            uint16_t frameLen = protocolUsbReadU16LE(parser->buffer + 2u);
            uint16_t frameCrc = protocolUsbReadU16LE(parser->buffer + 4u);
            if (frameLen == 0u || frameLen > PROTOCOL_USB_MAX_PAYLOAD_BYTES) {
                parser->badLenCount += 1u;
                memmove(parser->buffer, parser->buffer + 1u, parser->bufferedLen - 1u);
                parser->bufferedLen -= 1u;
                progress = true;
                continue;
            }

            size_t frameTotal = PROTOCOL_USB_HEADER_BYTES + frameLen;
            if (parser->bufferedLen < frameTotal) {
                break;
            }

            uint16_t calcCrc = protocolUsbCrc16(parser->buffer + PROTOCOL_USB_HEADER_BYTES, frameLen);
            if (calcCrc != frameCrc) {
                parser->badCrcCount += 1u;
                memmove(parser->buffer, parser->buffer + 1u, parser->bufferedLen - 1u);
                parser->bufferedLen -= 1u;
                progress = true;
                continue;
            }

            if (parser->onFrame) {
                parser->onFrame(parser->buffer + PROTOCOL_USB_HEADER_BYTES, frameLen, parser->userCtx);
            }
            frames += 1u;

            memmove(parser->buffer, parser->buffer + frameTotal, parser->bufferedLen - frameTotal);
            parser->bufferedLen -= frameTotal;
            progress = true;
        }
    }

    return frames;
}

#pragma once

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define PROTOCOL_USB_SYNC_WORD 0xA55Au
#define PROTOCOL_USB_HEADER_BYTES 6u

#ifndef PROTOCOL_USB_MAX_PAYLOAD_BYTES
#define PROTOCOL_USB_MAX_PAYLOAD_BYTES 512u
#endif

typedef void (*protocolUsbFrameHandler_t)(const uint8_t *payload,
                                          uint16_t payloadLen,
                                          void *userCtx);

typedef struct {
    uint8_t buffer[PROTOCOL_USB_HEADER_BYTES + PROTOCOL_USB_MAX_PAYLOAD_BYTES];
    size_t bufferedLen;
    protocolUsbFrameHandler_t onFrame;
    void *userCtx;
    uint32_t badCrcCount;
    uint32_t badLenCount;
    uint32_t dropCount;
} protocolUsbParser_t;

// Build a USB frame: [sync(2) | len(2) | crc16(2) | payload].
// Payload length must be <= PROTOCOL_USB_MAX_PAYLOAD_BYTES.
esp_err_t protocolUsbBuildFrame(const uint8_t *payload,
                                size_t payloadLen,
                                uint8_t *outBuf,
                                size_t outLen);

// Initialize or reset a parser; onFrame can be NULL to parse and drop frames.
void protocolUsbParserInit(protocolUsbParser_t *parser,
                           protocolUsbFrameHandler_t onFrame,
                           void *userCtx);
void protocolUsbParserReset(protocolUsbParser_t *parser);

// Feed raw USB bytes into the parser; returns number of valid frames parsed.
size_t protocolUsbParserFeed(protocolUsbParser_t *parser, const uint8_t *data, size_t len);

#ifdef __cplusplus
}
#endif

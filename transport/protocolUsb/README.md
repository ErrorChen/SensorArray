# protocolUsb

USB framing helper for SensorArray streaming. USB-CDC may split data into small
packets, so this module adds a sync word, length, and CRC16 to each frame.

## Frame layout (little-endian)
- syncWord: 16-bit (0xA55A, bytes on wire: 0x5A 0xA5)
- frameLen: 16-bit payload length in bytes
- frameCrc16: 16-bit CRC16-CCITT-FALSE over payload
- payload: frameLen bytes (e.g., protocolWire 400-byte frame)

Total: 6 + frameLen bytes.

## CRC16
CRC16 is CCITT-FALSE (poly 0x1021, init 0xFFFF).

## Basic usage
Build a frame:
```c
uint8_t out[PROTOCOL_USB_HEADER_BYTES + PROTOCOL_WIRE_FRAME_BYTES];
protocolUsbBuildFrame(payload, payloadLen, out, sizeof(out));
```

Parse a byte stream:
```c
static void onFrame(const uint8_t *payload, uint16_t len, void *userCtx)
{
    // TODO: decode payload (e.g., protocolWire frame) or enqueue for processing.
}

protocolUsbParser_t parser;
protocolUsbParserInit(&parser, onFrame, NULL);
protocolUsbParserFeed(&parser, rxBytes, rxLen);
```

## Notes
- `PROTOCOL_USB_MAX_PAYLOAD_BYTES` defaults to 512; override at build time if
  you need a different maximum.
- When CRC/length errors occur, the parser resynchronizes by scanning for the
  next syncWord.

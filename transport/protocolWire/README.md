# protocolWire

Binary framing helper for SensorArray transport. It packs a fixed 400-byte frame
that can be sent over BLE/WiFi/USB.

## Frame layout (little-endian)
- seq: 16-bit (2B)
- t0: 32-bit (4B)
- validMask: 64-bit (8B)
- offset[64]: 64 x 16-bit (128B)
- data[64]: 64 x 32-bit (256B)
- crc16: 16-bit (2B, optional)

Total: 400 bytes per frame.

## Data tag packing
The upper 4 bits of each 32-bit data word carry a tag, the lower 28 bits carry
the payload. For FDC2214 28-bit samples:

```c
uint32_t packed = protocolWirePackTaggedU28(PROTOCOL_WIRE_DATA_TAG_FDC2214, raw28);
```

On the receiver:
```c
uint8_t tag = protocolWireGetTag(packed);
uint32_t payload = protocolWireGetPayload(packed);
```

## CRC16
When `CONFIG_PROTOCOL_ENABLE_CRC16` is enabled, CRC16 is CCITT-FALSE
(poly 0x1021, init 0xFFFF). If disabled, crc16 is written as 0.

## Basic usage
```c
protocolWireFrame_t frame = {0};
frame.seq = seq;
frame.t0 = t0;
frame.validMask = validMask;
memcpy(frame.offset, offsets, sizeof(frame.offset));
for (size_t i = 0; i < PROTOCOL_WIRE_POINT_COUNT; ++i) {
    frame.data[i] = protocolWirePackTaggedU28(PROTOCOL_WIRE_DATA_TAG_FDC2214, raw28[i]);
}

uint8_t out[PROTOCOL_WIRE_FRAME_BYTES];
protocolWirePackFrame(&frame, out, sizeof(out));
```

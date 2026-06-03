# protocolWire / fixed-size wire payload

## 目录 / Table of contents

- [中文说明 / Chinese documentation](#中文说明--chinese-documentation)
- [Australian English documentation](#australian-english-documentation)

## 中文说明 / Chinese documentation

`transport/protocolWire` packs a fixed-size 64-point wire payload. It is transport-independent and does not know board routing, sampling policy or current text output.

Current frame fields:

```text
seq          2 bytes
t0           4 bytes
validMask    8 bytes
warnMask     8 bytes
errorMask    8 bytes
offset[64]   128 bytes
data[64]     256 bytes
crc16        2 bytes
total        416 bytes
```

For FDC matrix frames, `data[]` carries integer `freqHz` values, not raw28.

Tag helpers use the upper 4 bits of each 32-bit data word:

```text
protocolWirePackTaggedU28(tag, payload28)
protocolWireGetTag(value)
protocolWireGetPayload(value)
```

`protocolWirePackFrame()` writes little-endian fields and appends CCITT-FALSE CRC16 when `CONFIG_PROTOCOL_ENABLE_CRC16` is enabled.

## Australian English documentation

`transport/protocolWire` packs a fixed-size 64-point wire payload. It is independent of board routing, sampling policy, and current text output.

The packed frame is `416` bytes in the current source tree and includes `validMask`, `warnMask`, `errorMask`, 64 offsets, 64 data words, and CRC16. It is not the default runtime output path; current firmware output is text `MATRIXFDC_*` lines.

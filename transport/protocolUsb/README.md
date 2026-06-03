# protocolUsb / USB byte-stream framing

## 目录 / Table of contents

- [中文说明 / Chinese documentation](#中文说明--chinese-documentation)
- [Australian English documentation](#australian-english-documentation)

## 中文说明 / Chinese documentation

`transport/protocolUsb` 提供 USB/UART-like byte stream frame wrapper 和 parser。它只处理 byte framing，不解释 SensorArray payload 语义。

Frame layout:

```text
syncWord   2 bytes, little-endian, default 0xA55A
frameLen   2 bytes, little-endian
crc16      2 bytes, CCITT-FALSE over payload
payload    frameLen bytes, max PROTOCOL_USB_MAX_PAYLOAD_BYTES
```

真实 API：

| API | 作用 |
|---|---|
| `protocolUsbBuildFrame()` | Build `[sync | len | crc16 | payload]`. Payload length must be non-zero and <= `512` unless max is overridden. |
| `protocolUsbParserInit()` | Initialise parser and optional frame callback. |
| `protocolUsbParserReset()` | Clear parser buffer and error counters. |
| `protocolUsbParserFeed()` | Feed bytes, resynchronise on sync word, verify length/CRC, call callback and return parsed frame count. |

Parser diagnostics are stored in `badCrcCount`, `badLenCount` and `dropCount`.

## Australian English documentation

`transport/protocolUsb` provides byte-stream framing and parsing for USB/UART-like transports. It handles framing only and does not interpret SensorArray payload semantics.

It is not the current default runtime output path. The default firmware frame output is printf text from `main/output`.

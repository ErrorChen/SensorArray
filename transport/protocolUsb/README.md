# 二进制字节流封装 / Binary byte-stream envelope

本库可以给任意字节流增加包边界。每包小端六字节头依次为同步字 0xA55A（线上 5A A5）、负载长度、负载的 CRC-16/CCITT-FALSE（0x1021、初值 0xFFFF），随后为 1..512 字节负载，最大值可在构建时覆盖。项目的运行时 USB-OTG 文本与控制通路见独立组件。 / This optional library delimits packets on a byte stream. Its six-byte little-endian header contains sync word 0xA55A (5A A5 on the wire), payload length and CRC-16/CCITT-FALSE over the payload (0x1021, initial 0xFFFF). The payload follows, 1..512 bytes by default with a build-time maximum. The running firmware's USB-OTG text and control paths are documented separately. [头文件 / Header](include/protocolUsb.h) · [USB-OTG](../../components/sensorarrayUsbOtg/README.md)

```c
uint8_t packet[PROTOCOL_USB_HEADER_BYTES + sizeof(payload)];
esp_err_t err = protocolUsbBuildFrame(payload, sizeof(payload), packet, sizeof(packet));
protocolUsbParser_t parser;
protocolUsbParserInit(&parser, on_frame, user_context);
if (err == ESP_OK) protocolUsbParserFeed(&parser, packet, sizeof(packet));
```

示例中 `payload` 是非空字节数组，`on_frame` 符合 `protocolUsbFrameHandler_t`。分次喂入数据时重复使用同一个 parser；回调中的 payload 指向 parser 自身缓存，若需保留请在回调返回前复制。 / Here payload is a nonempty byte array and on_frame has type protocolUsbFrameHandler_t. Retain the same parser across chunks. Its callback borrows parser-owned storage and must copy retained data before returning.

## [protocolUsb.c](protocolUsb.c)

### 接口与回调 / Interfaces and callbacks

| C 签名 / C signature | 行为与参数 / Behaviour and arguments | 调用举例 / Representative caller |
|---|---|---|
| `esp_err_t protocolUsbBuildFrame(const uint8_t *payload, size_t payloadLen, uint8_t *outBuf, size_t outLen)` | 写头与 payload，空指针返回 INVALID_ARG，长度或容量不符返回 INVALID_SIZE。 / Write header and payload. Return ESP_OK, INVALID_ARG for null storage or INVALID_SIZE for payload/capacity errors. | 公开入口 / Callable API |
| `void protocolUsbParserInit(protocolUsbParser_t *parser, protocolUsbFrameHandler_t onFrame, void *userCtx)` | 清空 parser 并安装回调及上下文；onFrame 可空以仅统计包数。 / Clear parser and install the callback/context. A null callback still permits counting valid frames. | 公开入口 / Callable API |
| `void protocolUsbParserReset(protocolUsbParser_t *parser)` | 清除残包和 badCrcCount、badLenCount、dropCount，保留回调及上下文。 / Clear partial data and error/drop counters while retaining callback and context. | 公开入口 / Callable API |
| `size_t protocolUsbParserFeed(protocolUsbParser_t *parser, const uint8_t *data, size_t len)` | 逐块拼包，遇坏长度或 CRC 重新找同步字。返回本次完整有效包数。 / Accumulate chunks, resynchronise after length/CRC failures and return the valid frame count for this feed. | 公开入口 / Callable API |

### 内部 / Internal

| C 签名 / C signature | 行为与参数 / Behaviour and arguments | 调用举例 / Representative caller |
|---|---|---|
| `static void protocolUsbWriteU16LE(uint8_t *dst, uint16_t value)` | 把 value 的低字节、高字节依次写入 dst。 / Store the low and high bytes of value into dst. | [protocolUsbBuildFrame](protocolUsb.c#L49) |
| `static uint16_t protocolUsbReadU16LE(const uint8_t *src)` | 从 src 的两个小端字节还原 16 位值。 / Decode two little-endian bytes at src. | [protocolUsbParserFeed](protocolUsb.c#L96) |
| `static uint16_t protocolUsbCrc16(const uint8_t *data, size_t len)` | 返回 data 前 len 字节的 CCITT-FALSE 校验值。 / Compute CCITT-FALSE over len bytes at data. | [protocolUsbBuildFrame](protocolUsb.c#L49) |
| `static size_t protocolUsbFindSync(const uint8_t *buf, size_t len)` | 查找第一组 5A A5，找不到返回 SIZE_MAX。 / Return the first 5A A5 offset or SIZE_MAX when absent. | [protocolUsbParserFeed](protocolUsb.c#L96) |

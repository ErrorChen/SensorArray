# 64 点二进制负载 / 64-point binary payload

这个独立编码库将 `protocolWireFrame_t` 写入固定 416 字节，不按 C 结构体内存布局序列化。字段均为小端，偏移从零开始。FDC 标签 1 表示整数 Hz。 / This standalone encoder serialises a protocolWireFrame_t as 416 bytes independent of C struct padding. All fields are little-endian and offsets are zero-based. FDC tag 1 denotes integer Hz. [头文件 / Header](include/protocolWire.h) · [构建选项 / Kconfig](Kconfig) · [运行时传输 / Runtime transport](../../core/transport/)

| 字节偏移 / Byte offset | 字段 / Field | 宽度 / Width |
|---|---|---|
| 0 | seq | 2 |
| 2 | t0 | 4 |
| 6, 14, 22 | validMask, warnMask, errorMask | 每个 8 / 8 each |
| 30 | offset[64] | 64 × 2 |
| 158 | data[64] | 64 × 4 |
| 414 | CRC-16/CCITT-FALSE | 2 |

调用者填充掩码、时间、偏移和科学值。构建器仅做编码。CRC 覆盖前 414 字节；关闭 CRC 时尾字段写零（本源文件对未定义的配置宏默认启用）。 / The caller supplies masks, timestamps, offsets and scientific values. The encoder packs those fields. The CRC covers the first 414 bytes. A zero CRC field is written when the preprocessor option evaluates to zero. The source defaults an undefined option to enabled.

```c
protocolWireFrame_t frame = {0};
uint8_t encoded[PROTOCOL_WIRE_FRAME_BYTES];
frame.validMask = 1;
frame.data[0] = protocolWirePackTaggedU28(PROTOCOL_WIRE_DATA_TAG_FDC2214_FREQ_HZ,
                                          frequency_hz);
esp_err_t err = protocolWirePackFrame(&frame, encoded, sizeof(encoded));
```

## [include/protocolWire.h](include/protocolWire.h)

### 接口与回调 / Interfaces and callbacks

| C 签名 / C signature | 行为与参数 / Behaviour and arguments | 调用举例 / Representative caller |
|---|---|---|
| `static inline uint32_t protocolWirePackTaggedU28(uint8_t tag, uint32_t payload28)` | 将 tag 低四位放在字高四位，与 payload28 低 28 位组合。 / Pack the low 4 tag bits above the low 28 payload bits and return the word. | 见本文件的回调或条件分支 / File-local hook or build branch |
| `static inline uint8_t protocolWireGetTag(uint32_t value)` | 提取 32 位值的高四位标签。 / Return the top four tag bits. | 见本文件的回调或条件分支 / File-local hook or build branch |
| `static inline uint32_t protocolWireGetPayload(uint32_t value)` | 提取 32 位值的低 28 位负载。 / Return the low 28 payload bits. | 见本文件的回调或条件分支 / File-local hook or build branch |

## [protocolWire.c](protocolWire.c)

### 接口与回调 / Interfaces and callbacks

| C 签名 / C signature | 行为与参数 / Behaviour and arguments | 调用举例 / Representative caller |
|---|---|---|
| `esp_err_t protocolWirePackFrame(const protocolWireFrame_t *frame, uint8_t *outBuf, size_t outLen)` | 按字段顺序写入 outBuf。空指针返回 INVALID_ARG，空间不足返回 INVALID_SIZE，成功返回 ESP_OK。 / Serialise fields into outBuf. Return INVALID_ARG for null storage, INVALID_SIZE for insufficient space, or ESP_OK. | 公开入口 / Callable API |

### 内部 / Internal

| C 签名 / C signature | 行为与参数 / Behaviour and arguments | 调用举例 / Representative caller |
|---|---|---|
| `static void protocolWireWriteU16LE(uint8_t *dst, uint16_t value)` | 将 16 位 value 写入两个小端字节。 / Store value in two little-endian bytes. | [protocolWirePackFrame](protocolWire.c#L53) |
| `static void protocolWireWriteU32LE(uint8_t *dst, uint32_t value)` | 将 32 位 value 写入四个小端字节。 / Store value in four little-endian bytes. | [protocolWirePackFrame](protocolWire.c#L53) |
| `static void protocolWireWriteU64LE(uint8_t *dst, uint64_t value)` | 将 64 位 value 写入八个小端字节。 / Store value in eight little-endian bytes. | [protocolWirePackFrame](protocolWire.c#L53) |
| `static uint16_t protocolWireCrc16(const uint8_t *data, size_t len)` | 计算给定字节范围的 CRC-16/CCITT-FALSE。 / Compute CRC-16/CCITT-FALSE for the supplied byte range. | [protocolWirePackFrame](protocolWire.c#L53) |

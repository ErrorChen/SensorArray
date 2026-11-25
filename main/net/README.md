# 应用到网络传输的适配 / Application-to-network adapter

应用在启动时向传输层登记 legacy 命令回调。输出侧把已编码的文本包放入数据或日志通道。对于科学数据，适配器检查包与帧的 sequence、标签及长度，使用原帧构造元数据后交给 core/transport 的有界队列。

At startup this adapter registers the legacy command callback with transport. The output side submits encoded packets to data or log channels. For scientific data it checks sequence, tag and length against the source frame, derives metadata from that frame and hands it to the bounded core/transport queue.

## 调用方式 / Calling the module

```c
esp_err_t err = sensorarrayNetFramePublish(packet, frame, protectionFlags);
```

packet 和 frame 必须来自同一编码操作，保护位由输出策略给出。allowBle 是旧数据接口保留的参数，当前实现由传输策略决定各 sink。返回码描述本机投递。

packet and frame must come from the same encode operation. The output policy supplies protection flags. The legacy data entry retains allowBle in its signature and delegates sink selection to transport policy. Return values describe local submission.

## 源码和头文件 / Source and headers

| 文件 / File | 职责与声明 / Responsibility and declarations |
| --- | --- |
| [sensorarrayNetStatus.c](sensorarrayNetStatus.c) · [sensorarrayNetStatus.h](sensorarrayNetStatus.h) | 将已编码的帧及其原始身份传给 transport，将入站兼容命令投到采集邮箱。 / Hands encoded frames and their original identity to transport and submits inbound legacy commands to acquisition. `sensorarrayNetSinkStats_t` |

## 函数 / Functions

签名按源文件排列，参数顺序即调用顺序。下表标出生产代码中的调用者或回调注册者（最多两个）。`static` 限于所在编译单元。错误码列于对应行为后，只列函数直接返回的常量；硬件与下层调用可传播其他错误。

Signatures are grouped by source in call-argument order. The last column names up to two production callers or callback registrars. `static` entries are local to their translation unit. Constants after a description are explicit error returns. Underlying operations may propagate additional errors.

### sensorarrayNetStatus.c

| C 调用签名 / C signature | 行为及返回 / Behaviour and return | 调用位置 / Called by |
| --- | --- | --- |
| `static esp_err_t sensorarrayNetLegacyCommand(const uint8_t *data, size_t length, void *context)` [9](sensorarrayNetStatus.c#L9) | 将兼容网络命令原字节和显式长度投递到采集邮箱。 / Post legacy network command bytes with their supplied length to the acquisition mailbox. | `sensorarrayNetStatusInit` |
| `esp_err_t sensorarrayNetStatusInit(void)` [17](sensorarrayNetStatus.c#L17) | 登记兼容命令回调，随后初始化传输后端。 / Register the legacy command callback and initialise transport backends. | `app_main` |
| `esp_err_t sensorarrayNetTextPublish(const sensorarrayTextPacket_t *packet, bool allowBle)` [23](sensorarrayNetStatus.c#L23) | 将非空兼容数据包提交给传输层，由当前策略选择 sink；allowBle 保留兼容。 / Submit a nonempty legacy data packet using current sink policy. allowBle is retained for compatibility. `ESP_ERR_INVALID_ARG` | 独立接口 / Standalone entry |
| `esp_err_t sensorarrayNetLogPublish(const sensorarrayTextPacket_t *packet)` [33](sensorarrayNetStatus.c#L33) | 验证日志包非空并提交到传输层 LOG 通道。 / Check for a nonempty log packet and submit it to the transport LOG channel. `ESP_ERR_INVALID_ARG` | `sensorarrayAsyncLogPrintCompactSummary` |
| `esp_err_t sensorarrayNetFramePublish(const sensorarrayTextPacket_t *packet, const sensorarrayFrame_t *frame, uint32_t protectionFlags)` [41](sensorarrayNetStatus.c#L41) | 核对 packet 的标签和 sequence，使用原 frame 的元数据提交同一数据包。 / Verify packet tag/sequence and submit the packet with metadata from the original frame. `ESP_ERR_INVALID_ARG` | `sensorarrayAsyncLogTask` |
| `void sensorarrayNetGetSinkStats(sensorarrayNetSinkStats_t *outStats)` [55](sensorarrayNetStatus.c#L55) | 由传输计数汇总 Wi-Fi/BLE 数据与日志的提交、丢弃及阻塞。 / Aggregate Wi-Fi/BLE data and log submissions, drops and blocking from transport counters. | `sensorarrayAsyncLogSummarySetBaseline` / `sensorarrayAsyncLogSummaryBegin` |

## 参数索引 / Parameter index

相同名称按模块共享解释。完整类型、数组维度与输入输出方向以对应函数签名和调用点为准。指针参数所需存储由调用者提供，异步作业另受上面的所有权约束。

Entries share parameter descriptions within this module. Refer to the signature and its caller for the precise type, array shape and direction. The caller supplies pointer storage, subject to the ownership rules above for asynchronous work.

| 参数 / Parameter | 含义 / Meaning |
| --- | --- |
| `allowBle` | 兼容接口的布尔参数；本适配层由传输策略选择实际 sink。 / Legacy adapter flag. Actual sink selection follows transport policy. |
| `context` | 回调或任务创建时绑定的上下文；在该调用的有效期内保留其存储。 / Context bound at callback registration or task creation. Keep its backing object alive for the operation. |
| `data` | 按该函数声明传入的文本、命令、操作码或数据字节，显式 length 限定可读范围。 / Text, command, opcode or data bytes as declared. Where supplied, length bounds the readable range. |
| `frame` | 当前采集帧。const 指针供读取/编码，可写指针供对应采集步骤产生或更新数据。 / Capture frame. A const pointer is an input to readers/encoders. A writable pointer receives or updates acquisition results. |
| `length` | 输入、载荷、底层存储或读写传输的字节数。 / Byte length of input, payload, backing storage or transfer. |
| `outStats` | 函数签名注明的结果、快照或统计结构，调用者分配复制目标。 / Result, snapshot or statistics object of the declared type, with caller-owned output storage. |
| `packet` | 编码器产生的文本包，length 标记实际使用字节，sequence 关联源帧。 / Encoded text packet. length bounds used bytes and sequence identifies the source frame. |
| `protectionFlags` | 调用域对应的标志位；帧保护位在编码、入队和重试间保留。 / Domain-specific bit flags. Frame protection flags survive encoding, queuing and retries. |

[项目总览 / Project guide](../../README.md)

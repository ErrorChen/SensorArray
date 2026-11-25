# 行数配置与校准记录 / Scan configuration and calibration records

ROWS 请求进入启动会话内共享的请求编号域。控制任务接受后，采集任务在帧边界应用并公布行数域的 generation/requestId。持久校准使用专用 calib NVS 分区和有版本的二进制记录。板号与硬件版本在固件内定义，CRC32 覆盖记录头和载荷。

ROWS requests share the configuration request-ID allocator within a session. The control side accepts them and acquisition applies them at a frame boundary, publishing a rows-domain generation and applied request ID. Persistent calibration uses the calib NVS partition and a versioned binary record. Firmware supplies the board and hardware identifiers. CRC32 covers the header and payload.

## 调用方式 / Calling the module

```c
sensorarrayFrameConfigSnapshot_t rows = sensorarrayScanConfigGetFrameSnapshot();
uint8_t active = rows.rows;
```

rows 范围 1..8。该示例只读当前行数快照；构建整帧权威身份还需 mode、profile 和 boot/session。校准记录整数为小端，载荷至多 512 字节。加载者检查 identity、长度、CRC 和载荷回调，再使用校准。

Rows range from 1 to 8. The example reads the applied row configuration. A full frame identity also requires mode, profile and boot/session. Calibration integers use little-endian encoding with up to 512 payload bytes. Loading validates identity, length, CRC and the supplied payload validator before use.

## 源码和头文件 / Source and headers

| 文件 / File | 职责与声明 / Responsibility and declarations |
| --- | --- |
| [sensorarrayCalibration.c](sensorarrayCalibration.c) · [sensorarrayCalibration.h](sensorarrayCalibration.h) | 将经校验的校准载荷编码到 calib/record，保存或装载前核对版本、板号、长度、CRC 和有效性。 / Encodes calibration into calib/record and validates version, board identity, length, CRC and payload on the persistence path. `sensorarrayCalibrationRecordInfo_t` `sensorarrayCalibrationStatus_t` |
| [sensorarrayScanConfig.c](sensorarrayScanConfig.c) · [sensorarrayScanConfig.h](sensorarrayScanConfig.h) | 分配配置请求顺序；保存行数的 pending、applied 和上一次终态，在帧边界更新快照。 / Orders configuration requests, keeps pending/applied row counts and the last terminal outcome, and snapshots them at frame boundaries. `sensorarrayFrameConfigSnapshot_t` `sensorarrayScanConfigRequestResult_t` |

## 函数 / Functions

签名按源文件排列，参数顺序即调用顺序。下表标出生产代码中的调用者或回调注册者（最多两个）。`static` 限于所在编译单元。错误码列于对应行为后，只列函数直接返回的常量；硬件与下层调用可传播其他错误。

Signatures are grouped by source in call-argument order. The last column names up to two production callers or callback registrars. `static` entries are local to their translation unit. Constants after a description are explicit error returns. Underlying operations may propagate additional errors.

[sensorarrayCalibration.c](#sensorarraycalibrationc) · [sensorarrayScanConfig.c](#sensorarrayscanconfigc)

### sensorarrayCalibration.c

| C 调用签名 / C signature | 行为及返回 / Behaviour and return | 调用位置 / Called by |
| --- | --- | --- |
| `static void sensorarrayCalibrationWriteU32Le(uint8_t *out, uint32_t value)` [8](sensorarrayCalibration.c#L8) | 以小端顺序写出一个 uint32_t。 / Store one uint32_t in little-endian byte order. | `sensorarrayCalibrationRecordEncode` |
| `static uint32_t sensorarrayCalibrationReadU32Le(const uint8_t *bytes)` [16](sensorarrayCalibration.c#L16) | 从四个小端字节还原 uint32_t。 / Reconstruct a uint32_t from four little-endian bytes. | `sensorarrayCalibrationRecordReadInfo` / `sensorarrayCalibrationRecordDecode` |
| `uint32_t sensorarrayCalibrationCrc32(const uint8_t *bytes, size_t length)` [24](sensorarrayCalibration.c#L24) | 计算记录头和载荷所用的 CRC32。 / Compute the CRC32 for the record header and payload. | `sensorarrayCalibrationRecordEncode` / `sensorarrayCalibrationRecordDecode` |
| `esp_err_t sensorarrayCalibrationRecordEncode(uint32_t source, const void *payload, size_t payloadLength, uint8_t *outBytes, size_t outCapacity, size_t *outLength)` [37](sensorarrayCalibration.c#L37) | 把记录头、载荷与 CRC32 写入有容量限制的输出缓冲区。 / Write the record header, payload and CRC32 into a capacity-checked buffer. `ESP_ERR_INVALID_ARG` `ESP_ERR_INVALID_SIZE` | `sensorarrayCalibrationSave` |
| `static bool sensorarrayCalibrationRecordReadInfo(const uint8_t *bytes, size_t length, sensorarrayCalibrationRecordInfo_t *outInfo, size_t *outPayloadLength)` [77](sensorarrayCalibration.c#L77) | 从记录头提取版本、板号、来源和载荷长度。 / Extract version, board identity, source and payload length from the record header. | `sensorarrayCalibrationRecordDecode` |
| `esp_err_t sensorarrayCalibrationRecordDecode(const uint8_t *bytes, size_t length, sensorarrayCalibrationRecordInfo_t *outInfo, void *outPayload, size_t payloadCapacity)` [109](sensorarrayCalibration.c#L109) | 解读小端记录身份与长度并验证 CRC，再按可选目标容量复制有效载荷。 / Decode little-endian identity and length, validate CRC, and copy a valid payload into the optional capacity-checked destination. `ESP_ERR_INVALID_ARG` `ESP_ERR_INVALID_SIZE` `ESP_ERR_INVALID_STATE` | `sensorarrayCalibrationLoad` / `sensorarrayCalibrationQuery` |
| `esp_err_t sensorarrayCalibrationInit(void)` [151](sensorarrayCalibration.c#L151) | 初始化专用校准 NVS 分区，报告分区或驱动初始化失败。 / Initialise the calibration NVS partition and report partition or driver errors. | `sensorarrayCalibrationSave` / `sensorarrayCalibrationLoad` |
| `static void sensorarrayCalibrationStatusFromInfo(sensorarrayCalibrationStatus_t *status, const sensorarrayCalibrationRecordInfo_t *info)` [162](sensorarrayCalibration.c#L162) | 把已解码记录头投影到控制查询的校准状态结构。 / Map the decoded record header to the status returned by calibration queries. | `sensorarrayCalibrationLoad` / `sensorarrayCalibrationQuery` |
| `static esp_err_t sensorarrayCalibrationReadRecord(uint8_t *outRecord, size_t capacity, size_t *outLength)` [179](sensorarrayCalibration.c#L179) | 从 NVS 读取当前有界校准 blob 并返回实际字节数。 / Read the bounded calibration blob from NVS and return its length. | `sensorarrayCalibrationLoad` / `sensorarrayCalibrationQuery` |
| `esp_err_t sensorarrayCalibrationSave(const void *payload, size_t payloadLength, sensorarrayCalibrationStatus_t *outStatus)` [214](sensorarrayCalibration.c#L214) | 把编码后的校准记录保存到专用 NVS 分区并返回状态。 / Persist the encoded record in the calibration NVS partition and return status. `ESP_ERR_INVALID_ARG` | `sensorarrayApplyPendingCommands` |
| `esp_err_t sensorarrayCalibrationLoad(sensorarrayCalibrationPayloadValidateFn validatePayload, void *outPayload, size_t payloadCapacity, size_t *outPayloadLength, sensorarrayCalibrationStatus_t *outStatus)` [272](sensorarrayCalibration.c#L272) | 从 calib NVS 读取记录并执行结构、CRC 和调用方提供的载荷验证。 / Load the calib NVS record and apply structural, CRC and caller-supplied payload validation. `ESP_ERR_INVALID_ARG` `ESP_ERR_NOT_FOUND` `ESP_ERR_INVALID_STATE` `ESP_ERR_INVALID_SIZE` | `sensorarrayApplyPendingCommands` / `sensorarrayLoadPersistedMatrixCalibration` |
| `esp_err_t sensorarrayCalibrationQuery(sensorarrayCalibrationStatus_t *outStatus)` [354](sensorarrayCalibration.c#L354) | 在不应用校准的情况下查询存储记录的可用性与身份。 / Inspect the stored record's availability and identity without applying it. `ESP_ERR_INVALID_ARG` | `sensorarrayRuntimeQueryCommand` |

### sensorarrayScanConfig.c

| C 调用签名 / C signature | 行为及返回 / Behaviour and return | 调用位置 / Called by |
| --- | --- | --- |
| `static uint8_t sensorarrayScanConfigDefaultRows(void)` [15](sensorarrayScanConfig.c#L15) | 从构建配置选取启动时默认活动行数。 / Select the initial active row count from build configuration. | `sensorarrayScanConfigMakeSnapshotLocked` / `sensorarrayScanConfigInit` |
| `static uint8_t sensorarrayScanConfigRowMask(uint8_t rows)` [37](sensorarrayScanConfig.c#L37) | 为从第一行开始的 rows 个活动行生成位掩码。 / Build a contiguous activity mask covering the first rows rows. | `sensorarrayScanConfigMakeSnapshotLocked` |
| `static sensorarrayFrameConfigSnapshot_t sensorarrayScanConfigMakeSnapshotLocked(void)` [45](sensorarrayScanConfig.c#L45) | 持配置锁从同一状态复制当前行数、掩码、请求和终态。 / Copy rows, masks, request and terminal state while holding the configuration lock. | `sensorarrayScanConfigGetFrameSnapshot` |
| `esp_err_t sensorarrayScanConfigInit(void)` [62](sensorarrayScanConfig.c#L62) | 建立行配置与最初已应用快照，清除启动前的 pending 请求。 / Initialise row configuration and its first applied snapshot with no pending requests. | `sensorarrayTransportInitOwned` |
| `esp_err_t sensorarrayScanConfigRequestRows(uint8_t rows, sensorarrayScanConfigRequestResult_t *outResult)` [82](sensorarrayScanConfig.c#L82) | 验证请求行数并通过共享受理路径登记，返回请求 ID 和配置快照。 / Validate requested rows and submit via shared admission, returning its request ID and configuration snapshot. `ESP_ERR_INVALID_ARG` `ESP_ERR_INVALID_STATE` | 独立接口 / Standalone entry |
| `uint8_t sensorarrayScanConfigGetActiveRows(void)` [115](sensorarrayScanConfig.c#L115) | 取得采集侧当前已生效的行数。 / Read the row count currently applied by acquisition. | `sensorarrayMeasureFdcTheoreticalFrameFpsX100` / `sensorarrayRuntimeQueryCommand` |
| `uint8_t sensorarrayScanConfigGetPendingRows(void)` [123](sensorarrayScanConfig.c#L123) | 取得已受理、尚待边界应用的行数。 / Read the accepted row count awaiting boundary application. | 独立接口 / Standalone entry |
| `sensorarrayFrameConfigSnapshot_t sensorarrayScanConfigGetFrameSnapshot(void)` [131](sensorarrayScanConfig.c#L131) | 在配置锁内复制行数、行掩码、世代、已应用请求与最后终态。 / Copy row count, row mask, generation, applied request and last terminal state under the configuration lock. | `sensorarrayMeasureReadFdcMatrixFrame` / `sensorarrayMeasureReadFdcMatrixFrameRows` |
| `void sensorarrayScanConfigApplyPendingAtFrameBoundary(sensorarrayScanConfigApplyResult_t *outResult)` [139](sensorarrayScanConfig.c#L139) | 仅在帧边界应用 pending 行数并更新行数域身份、终态与结果。 / Commit pending rows at a frame boundary and update rows-domain identity and terminal result. | `sensorarrayApplyPendingCommands` / `sensorarrayRunOneFrame` |
| `static void sensorarrayScanConfigNormalize(char *text)` [175](sensorarrayScanConfig.c#L175) | 规范化 扫描配置。 / Normalise scan config. | `sensorarrayScanConfigHandleCommand` |
| `esp_err_t sensorarrayScanConfigHandleCommand(const char *command, size_t length, char *response, size_t responseSize)` [191](sensorarrayScanConfig.c#L191) | 解析 ROWS 查询或设置请求，写入有容量限制的协议回复。 / Parse a ROWS query or update and write its bounded protocol reply. `ESP_ERR_INVALID_ARG` `ESP_ERR_NOT_SUPPORTED` | `sensorarrayTransportHandleControlCommand` |
| `uint32_t sensorarrayScanConfigAllocateRequestId(void)` [260](sensorarrayScanConfig.c#L260) | 在共享配置域中分配下一个非零请求 ID。 / Allocate the next nonzero request ID in the shared configuration domain. | `sensorarrayCommandMailboxPostRailCalibration` / `sensorarrayCommandMailboxPostRequest` |
| `esp_err_t sensorarrayScanConfigBeginConfigurationRequest(uint32_t *outRequestId)` [269](sensorarrayScanConfig.c#L269) | 预留共享配置请求槽并把请求 ID 写入调用方。 / Reserve the shared configuration request and return its ID to the caller. `ESP_ERR_INVALID_ARG` `ESP_ERR_INVALID_STATE` | `sensorarrayCommandMailboxPostReservedConfig` / `sensorarrayCommandMailboxPostRowModes` |
| `void sensorarrayScanConfigEndConfigurationRequest(uint32_t requestId)` [284](sensorarrayScanConfig.c#L284) | 只释放与当前保留 ID 匹配的配置请求槽。 / Release the shared configuration reservation only when its ID matches. | `sensorarrayCommandMailboxPostRowModes` / `sensorarrayCommandMailboxCancelRowModesAck` |
| `void sensorarrayScanConfigSetRequestCallback(sensorarrayScanConfigRequestCallback_t callback)` [293](sensorarrayScanConfig.c#L293) | 设置 请求回调。 / Set request callback. | `sensorarrayCommandMailboxInit` / `sensorarrayInitRuntime` |
| `esp_err_t sensorarrayScanConfigStageAcceptedRows(uint8_t rows, uint32_t requestId)` [300](sensorarrayScanConfig.c#L300) | 把已受理行数与请求 ID 放入待应用状态。 / Stage an already accepted row count and ID for boundary application. `ESP_ERR_INVALID_ARG` `ESP_ERR_INVALID_STATE` | `sensorarrayApplyPendingCommands` |
| `void sensorarrayScanConfigRecordRejected(uint32_t requestId, uint32_t errorCode)` [315](sensorarrayScanConfig.c#L315) | 保存被拒绝配置请求的编号和错误码。 / Retain the rejected configuration request ID and its error. | `sensorarrayRejectHeldCommand` / `sensorarrayApplyPendingCommands` |

## 参数索引 / Parameter index

相同名称按模块共享解释。完整类型、数组维度与输入输出方向以对应函数签名和调用点为准。指针参数所需存储由调用者提供，异步作业另受上面的所有权约束。

Entries share parameter descriptions within this module. Refer to the signature and its caller for the precise type, array shape and direction. The caller supplies pointer storage, subject to the ownership rules above for asynchronous work.

| 参数 / Parameter | 含义 / Meaning |
| --- | --- |
| `bytes`, `outPayload`, `payload` | 校准记录/载荷或原始字节；outPayload 指向允许写入的目标。 / Calibration record/payload or raw bytes. outPayload names caller-writable storage. |
| `callback`, `validatePayload` | 使用头文件定义签名的回调。提供的上下文与回调在注册或同步调用期间有效。 / Callback with the signature declared in the header. Keep its context valid while registered or used. |
| `capacity` | 按签名和对象度量的可用容量；数组以元素计，文本缓冲区以字节计。 / Available capacity in typed elements for arrays or bytes for text buffers. |
| `command`, `text` | 按该函数声明传入的文本、命令、操作码或数据字节，显式 length 限定可读范围。 / Text, command, opcode or data bytes as declared. Where supplied, length bounds the readable range. |
| `errorCode` | 操作、首个根因或独立清理结果的错误码。保留调用层提供的具体错误。 / Error for the operation, first cause or separate cleanup, preserving the code from the reporting layer. |
| `info`, `status` | 声明的诊断/状态记录，或从器件读回的原始状态字。 / Diagnostic/status record of the declared type or a raw status word returned by the device. |
| `length`, `payloadLength` | 输入、载荷、底层存储或读写传输的字节数。 / Byte length of input, payload, backing storage or transfer. |
| `out`, `response` | 调用者提供的回复、暂存或结果缓冲区，使用随同的容量/类型。 / Caller-provided reply, scratch or result storage with the accompanying capacity/type. |
| `outBytes` | 接收结果。校准记录/载荷或原始字节；outPayload 指向允许写入的目标。 / Output. Calibration record/payload or raw bytes. outPayload names caller-writable storage. |
| `outCapacity` | 调用者提供的可写空间，文本/字节缓冲按字节计，数组按本函数的元素类型计。 / Capacity supplied by the caller. Text/byte buffers use bytes and typed arrays use elements. |
| `outInfo`, `outStatus` | 接收结果。声明的诊断/状态记录，或从器件读回的原始状态字。 / Output. Diagnostic/status record of the declared type or a raw status word returned by the device. |
| `outLength`, `outPayloadLength` | 接收结果。输入、载荷、底层存储或读写传输的字节数。 / Output. Byte length of input, payload, backing storage or transfer. |
| `outRecord` | 接收结果。按签名为有界探测记录的索引、诊断记录或序列化记录字节。 / Output. Index of a bounded probe observation, diagnostic record or serialised record storage as declared. |
| `outRequestId` | 本地请求或事务标识，out 参数接收新分配或选中的 ID。 / Local request or transaction identifier. An out parameter receives the allocated or selected ID. |
| `outResult` | 函数签名注明的结果、快照或统计结构，调用者分配复制目标。 / Result, snapshot or statistics object of the declared type, with caller-owned output storage. |
| `payloadCapacity` | 载荷容量。 / Payload capacity. |
| `requestId` | 对应配置、恢复触发或 profile 域内的请求标识，由接收方与提交时身份配对。 / Request identifier for the indicated configuration, recovery trigger or row profile, paired with the identity at admission. |
| `responseSize` | 接收文本或字节缓冲区的容量。文本输出还需预留 NUL。 / Capacity of the supplied byte/text buffer. A text output must also accommodate its NUL terminator. |
| `rows` | 行数值或行配置快照，由该函数的参数类型决定。 / Active count or row-configuration snapshot as specified by its declared type. |
| `source` | 路由或测量的来源标记，区分外部实测、内部推算或对应开关源的枚举。 / Route/measurement source discriminator, such as external, internal estimate or switch source for the declared enum. |
| `value` | 值。 / Value. |

[项目总览 / Project guide](../../README.md)

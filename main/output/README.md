# 帧编码、异步记录和 USB 输出 / Frame encoding, asynchronous logging and USB output

生产编码器按 CAP、VOLT/RES、MIXED 选择文本布局，附加同帧身份及 K 校验尾。异步 logger 用固定槽隔离采集和格式化/输出的等待，分别跟踪丢弃、队列水位、保护帧与已发布序号。USB sink 区分数据与调试流，控制接收另有所有者。

The production encoder selects the CAP, VOLT/RES or MIXED layout and adds the frame's identity and K checksum trailer. Fixed asynchronous logger slots decouple acquisition from formatting and sink waits. Drops, queue watermarks, protected frames and published sequences have separate counters. USB output distinguishes data and debug streams and has a separate control reader.

## 调用方式 / Calling the module

```c
sensorarrayTextPacket_t packet;
esp_err_t err = sensorarrayTextProtocolBuildFrame(frame, &packet);
```

编码调用读取已完成的 frame，在 outPacket.length 范围内提交字节。最坏帧大小有静态断言。编码时保留有效、新鲜、预期和采得掩码及错误单元 Xhh。sinkQueued 和 driverAcceptedBytes 分别描述排队与驱动接收，主机重算 CRC 并收齐 K 才能记为完整实收。

The encoder reads a completed frame. Submit exactly outPacket.length bytes on success. A static assertion bounds worst-case packet size. Encoding preserves validity, freshness, expected/acquired masks and Xhh cell errors. sinkQueued describes local admission and driverAcceptedBytes describes driver acceptance. A host verifies the CRC and receives the K trailer before counting a complete frame.

## 源码和头文件 / Source and headers

| 文件 / File | 职责与声明 / Responsibility and declarations |
| --- | --- |
| [sensorarrayAcqEvent.c](sensorarrayAcqEvent.c) · [sensorarrayAcqEvent.h](sensorarrayAcqEvent.h) | 将采集侧格式化文本归到协议生命周期或普通异步日志，避免在硬件热路径直接写网络。 / Classifies formatted acquisition events as protocol lifecycle or ordinary asynchronous logs to keep network I/O outside hardware hot paths. |
| [sensorarrayAsyncLog.c](sensorarrayAsyncLog.c) · [sensorarrayAsyncLog.h](sensorarrayAsyncLog.h) | 重用固定缓冲区传递帧快照或事件，由 logger 编码发往各输出并统计丢弃和高水位。 / Reuses fixed buffers for frame snapshots and events. The logger encodes them for sinks and records drops and watermarks. `sensorarrayAsyncLogStats_t` `sensorarrayAsyncLogProtectionStats_t` |
| [sensorarrayAsyncLogStorage.h](sensorarrayAsyncLogStorage.h) | 根据内存能力一次建立长期使用的异步输出缓冲并记录初始化错误。 / Establishes long-lived asynchronous output storage in an eligible heap and retains initialisation failures. `sensorarrayAsyncLogStorage_t` |
| [sensorarrayFrameOutput.c](sensorarrayFrameOutput.c) · [sensorarrayFrameOutput.h](sensorarrayFrameOutput.h) | 将采集帧编码为完整文本包并按 Serial 策略提交到本地数据队列。 / Encodes a capture as a complete packet and submits it to the local serial data queue when the sink is enabled. |
| [sensorarrayFramePublication.h](sensorarrayFramePublication.h) | 从原帧构造 transport 所需的完整身份与各标签元数据。 / Derives the complete transport identity and per-tag metadata from the frame that generated the packet. |
| [sensorarrayOutputPolicy.h](sensorarrayOutputPolicy.h) | 根据是否有可用槽、队列内已有帧与丢弃策略决定接纳、替换或拒绝。 / Selects admission, replacement or rejection from free-slot availability, queued frames and drop policy. `sensorarrayOutputCongestionDecision_t` |
| [sensorarrayTextProtocol.c](sensorarrayTextProtocol.c) · [sensorarrayTextProtocol.h](sensorarrayTextProtocol.h) | 按真实线上格式编码 C/V/R 或 M/MR 与 K 尾帧并计算 CRC32。 / Encodes C/V/R or M/MR and the K trailer in wire format and computes CRC32. `sensorarrayTextPacket_t` |
| [sensorarrayUsbSink.c](sensorarrayUsbSink.c) · [sensorarrayUsbSink.h](sensorarrayUsbSink.h) | 向 Serial/JTAG 或 USB-OTG 后端投递调试与数据文本，跟踪本地排队、丢弃和流选择。 / Submits debug and data text to the selected Serial/JTAG or USB-OTG backend and tracks local queuing, drops and stream selection. `sensorarrayUsbStreamMode_t` `sensorarrayUsbStreamProfile_t` |

## 函数 / Functions

签名按源文件排列，参数顺序即调用顺序。下表标出生产代码中的调用者或回调注册者（最多两个）。`static` 限于所在编译单元。错误码列于对应行为后，只列函数直接返回的常量；硬件与下层调用可传播其他错误。

Signatures are grouped by source in call-argument order. The last column names up to two production callers or callback registrars. `static` entries are local to their translation unit. Constants after a description are explicit error returns. Underlying operations may propagate additional errors.

[sensorarrayAcqEvent.c](#sensorarrayacqeventc) · [sensorarrayAsyncLog.c](#sensorarrayasynclogc) · [sensorarrayAsyncLogStorage.h](#sensorarrayasynclogstorageh) · [sensorarrayFrameOutput.c](#sensorarrayframeoutputc) · [sensorarrayFramePublication.h](#sensorarrayframepublicationh) · [sensorarrayOutputPolicy.h](#sensorarrayoutputpolicyh) · [sensorarrayTextProtocol.c](#sensorarraytextprotocolc) · [sensorarrayUsbSink.c](#sensorarrayusbsinkc)

### sensorarrayAcqEvent.c

| C 调用签名 / C signature | 行为及返回 / Behaviour and return | 调用位置 / Called by |
| --- | --- | --- |
| `static bool sensorarrayAcqEventIsProtocolLifecycle(const char *text)` [13](sensorarrayAcqEvent.c#L13) | 按文本前缀识别应走生命周期优先通道的消息。 / Recognise lifecycle-priority text by its protocol prefix. | `sensorarrayAcqEventPrintf` |
| `int sensorarrayAcqEventPrintf(const char *format, ...)` [32](sensorarrayAcqEvent.c#L32) | 用有界缓冲格式化采集日志并按消息类别提交到异步输出。 / Format an acquisition event in bounded storage and submit by message class. | `sensorarrayAdsMatrixLogCrcEvidence` / `sensorarrayAdsMatrixEngineReadFrame` |

### sensorarrayAsyncLog.c

| C 调用签名 / C signature | 行为及返回 / Behaviour and return | 调用位置 / Called by |
| --- | --- | --- |
| `static uint64_t sensorarrayAsyncLogElapsedPositiveUs(int64_t startUs)` [334](sensorarrayAsyncLog.c#L334) | 返回经过正端µs。 / Return elapsed positive µs. `elapsedUs > 0 ? (uint64_t)elapsedUs : 0u` | `sensorarrayAsyncLogTask` |
| `static size_t sensorarrayAsyncLogTextAppend(char *buffer, size_t bufferSize, size_t position, const char *format, ...)` [340](sensorarrayAsyncLog.c#L340) | 在剩余空间中追加格式化文本并推进偏移。 / Append formatted text within remaining capacity and advance the offset. | `sensorarrayAsyncLogPrintCompactSummary` |
| `static uint64_t sensorarrayAsyncLogEstimateI2cBits(uint32_t writeCount, uint32_t readCount, uint32_t writeBytes, uint32_t readBytes)` [359](sensorarrayAsyncLog.c#L359) | 估算 I²C位。 / Estimate I²C bits. `dataBitsWithAck + addressBitsWithAck + framingBits` | `sensorarrayAsyncLogMaybePrintSummary` |
| `static sensorarrayAsyncLogSharedStats_t sensorarrayAsyncLogReadStats(void)` [370](sensorarrayAsyncLog.c#L370) | 读取 统计。 / Read stats. | `sensorarrayAsyncLogGetStats` / `sensorarrayAsyncLogPrintCompactSummary` |
| `void sensorarrayAsyncLogGetStats(sensorarrayAsyncLogStats_t *outStats)` [379](sensorarrayAsyncLog.c#L379) | 查找 统计。 / Look up stats. | `sensorarrayRuntimeQueryCommand` |
| `void sensorarrayAsyncLogGetProtectionStats(sensorarrayAsyncLogProtectionStats_t *outStats)` [398](sensorarrayAsyncLog.c#L398) | 查找 保护统计。 / Look up protection stats. | `sensorarraySnapshotDiagnosticsFormat` |
| `static void sensorarrayAsyncLogNotifyTask(void)` [406](sensorarrayAsyncLog.c#L406) | 唤醒 logger 以处理事件或可读帧槽。 / Wake the logger to drain pending events and frame slots. | `sensorarrayAsyncLogPublishFrameSnapshot` / `sensorarrayAsyncLogPublishEvent` |
| `static void sensorarrayAsyncLogIncrementDroppedEvent(void)` [413](sensorarrayAsyncLog.c#L413) | 统计事件本地投递失败造成的丢弃。 / Increment the event drop counter after local admission failure. | `sensorarrayAsyncLogPublishEvent` / `sensorarrayAsyncLogPublishProtocolEvent` |
| `static uint8_t sensorarrayAsyncLogFindFreeSlotLocked(void)` [420](sensorarrayAsyncLog.c#L420) | 在持锁状态下寻找可用于普通帧的空槽。 / Locate a free ordinary-frame slot while holding the slot lock. | `sensorarrayAsyncLogPublishFrameSnapshot` |
| `static uint8_t sensorarrayAsyncLogFindFreeProtectedSlotLocked(void)` [430](sensorarrayAsyncLog.c#L430) | 在持锁状态下寻找可容纳保护快照的预留槽。 / Locate a reserved protected-snapshot slot under the slot lock. | `sensorarrayAsyncLogPublishFrameSnapshot` |
| `static void sensorarrayAsyncLogRemovePendingLocked(uint8_t offset)` [439](sensorarrayAsyncLog.c#L439) | 在持锁状态下从待发队列移除给定项。 / Remove the selected item from the pending queue under its lock. | `sensorarrayAsyncLogDropOldestQueuedFrameLocked` / `sensorarrayAsyncLogSupersedeProtectedLocked` |
| `static bool sensorarrayAsyncLogDropOldestQueuedFrameLocked(void)` [454](sensorarrayAsyncLog.c#L454) | 在持锁队列中淘汰最旧的可丢数据帧并归还槽。 / Evict the oldest eligible queued frame under lock and reclaim its slot. | `sensorarrayAsyncLogPublishFrameSnapshot` |
| `static bool sensorarrayAsyncLogSupersedeProtectedLocked(const sensorarrayTransportFrameIdentity_t *identity)` [466](sensorarrayAsyncLog.c#L466) | 在锁内以同域更新的快照取代允许被替换的保护槽。 / Replace an eligible protected slot with a newer compatible snapshot under its lock. | `sensorarrayAsyncLogPublishFrameSnapshot` |
| `static uint32_t sensorarrayAsyncLogProtectionFlagsLocked(const sensorarrayFrame_t *frame, const sensorarrayTransportFrameIdentity_t *identity, int64_t nowUs)` [483](sensorarrayAsyncLog.c#L483) | 持槽锁计算配置切换后首帧和周期快照的保护位。 / Derive first-configuration and periodic snapshot protection under the slot lock. | `sensorarrayAsyncLogPublishFrameSnapshot` |
| `static bool sensorarrayAsyncLogPopFrame(sensorarrayFrame_t *outFrame, sensorarrayTextPacket_t *outTextPacket, uint64_t *outMeasureFrameUs, int64_t *outPublishedUs, uint32_t *outQueueDepth, uint32_t *outProtectionFlags)` [500](sensorarrayAsyncLog.c#L500) | 领取待发送帧快照及其队列时序、保护标志。 / Take a pending frame snapshot with queue timing and protection flags. | `sensorarrayAsyncLogTask` |
| `static void sensorarrayAsyncLogPrintEvent(const sensorarrayAsyncLogEvent_t *event)` [541](sensorarrayAsyncLog.c#L541) | 按事件类型格式化并投递该条异步消息。 / Format and dispatch one asynchronous event by class. | `sensorarrayAsyncLogDrainEvents` / `sensorarrayAsyncLogDrainProtocolEvents` |
| `static void sensorarrayAsyncLogDrainEvents(void)` [638](sensorarrayAsyncLog.c#L638) | 从普通事件队列批量领取有界数量的待发日志。 / Drain bounded ordinary log/event work from its queue. | `sensorarrayAsyncLogTask` |
| `static void sensorarrayAsyncLogDrainProtocolEvents(void)` [657](sensorarrayAsyncLog.c#L657) | 从协议优先队列领取并分派生命周期文本。 / Drain and dispatch lifecycle text from the protocol event queue. | `sensorarrayAsyncLogTask` |
| `static void sensorarrayAsyncLogSummarySetBaseline(sensorarrayAsyncLogSummary_t *summary, const sensorarrayAsyncLogSharedStats_t *stats)` [675](sensorarrayAsyncLog.c#L675) | 保存各 sink、采集及队列当前累积值作为下个统计窗口的起点。 / Save cumulative sink, acquisition and queue counters as the next interval baseline. | `sensorarrayAsyncLogPrintCompactSummary` / `sensorarrayAsyncLogMaybePrintSummary` |
| `static void sensorarrayAsyncLogSummaryBegin(sensorarrayAsyncLogSummary_t *summary, const sensorarrayFrame_t *frame)` [715](sensorarrayAsyncLog.c#L715) | 初始化本次统计窗口的起始时刻和计数基线。 / Start an interval with its timestamp and counter baselines. | `sensorarrayAsyncLogUpdateSummary` |
| `static void sensorarrayAsyncLogUpdateSummary(sensorarrayAsyncLogSummary_t *summary, const sensorarrayFrame_t *frame, uint64_t measureFrameUs, uint64_t frameAgeUs, uint64_t outputUs, uint32_t queueDepth, bool emitted)` [773](sensorarrayAsyncLog.c#L773) | 把新帧的采集、输出时长和队列水位计入当前统计窗口。 / Accumulate capture/output durations and queue depth for this frame in the current window. | `sensorarrayAsyncLogTask` |
| `static const char *sensorarrayBatteryReasonName(sensorarrayBatteryInvalidReason_t reason)` [997](sensorarrayAsyncLog.c#L997) | 返回原因的显示名称。 / Return the display name for reason. | `sensorarrayAsyncLogPrintCompactSummary` |
| `static void sensorarrayAsyncLogPrintCompactSummary(sensorarrayAsyncLogSummary_t *summary)` [1052](sensorarrayAsyncLog.c#L1052) | 读取窗口增量，报告采集、输出和各 sink 的吞吐及压力。 / Report interval deltas for capture, output and per-sink throughput/congestion. | `sensorarrayAsyncLogMaybePrintSummary` |
| `static void sensorarrayAsyncLogMaybePrintSummary(sensorarrayAsyncLogSummary_t *summary)` [1485](sensorarrayAsyncLog.c#L1485) | 只在配置的窗口或帧间隔到期后发布汇总。 / Emit the aggregate when the configured reporting window is due. | `sensorarrayAsyncLogTask` |
| `static void sensorarrayAsyncLogTask(void *arg)` [1805](sensorarrayAsyncLog.c#L1805) | 排空日志/生命周期与帧槽，编码并送往各 sink，更新窗口统计。 / Drain event and frame slots, encode for sinks and update interval statistics. | `sensorarrayAsyncLogInit` |
| `esp_err_t sensorarrayAsyncLogInit(void)` [1892](sensorarrayAsyncLog.c#L1892) | 建立长期输出缓冲、事件队列和 logger 任务并登记资源失败。 / Establish persistent output buffers, event queues and the logger task and record resource failures. `ESP_ERR_NOT_SUPPORTED` `ESP_ERR_INVALID_STATE` `ESP_ERR_NO_MEM` | `sensorarrayInitAsyncLogging` |
| `esp_err_t sensorarrayAsyncLogStartUsbSink(void)` [2023](sensorarrayAsyncLog.c#L2023) | 启动 USB 输出后端并向 logger 返回初始化结果。 / Start the USB output backend and return initialisation status to the logger. `ESP_ERR_INVALID_STATE` | `app_main` |
| `void sensorarrayAsyncLogEnableAcquisition(void)` [2043](sensorarrayAsyncLog.c#L2043) | 更新 logger 的采集就绪状态并使挂起工作可见。 / Enable acquisition-side publication in the logger's state. | `sensorarrayScanTask` |
| `bool sensorarrayAsyncLogIsRunning(void)` [2050](sensorarrayAsyncLog.c#L2050) | 查询异步 logger 是否已经初始化并拥有运行任务。 / Check whether the asynchronous logger has its running task. | `sensorarrayAcqEventPrintf` / `sensorarrayAsyncLogPublishFrameSnapshot` |
| `esp_err_t sensorarrayAsyncLogPublishFrameSnapshot(const sensorarrayFrame_t *frame, uint64_t measureFrameUs)` [2059](sensorarrayAsyncLog.c#L2059) | 为本帧分配异步槽并复制采集快照，通过有界输出任务发送。 / Reserve an asynchronous slot and copy the capture snapshot for bounded output processing. `ESP_ERR_INVALID_ARG` `ESP_ERR_INVALID_STATE` | `sensorarrayRunMainLoop` |
| `static esp_err_t sensorarrayAsyncLogPublishEvent(const sensorarrayAsyncLogEvent_t *event)` [2200](sensorarrayAsyncLog.c#L2200) | 将带类型和载荷的事件提交到有界 logger 队列。 / Submit the typed event to the bounded logger queue. `ESP_ERR_INVALID_ARG` `ESP_ERR_INVALID_STATE` `ESP_ERR_TIMEOUT` | `sensorarrayAsyncLogPublishOverrun` / `sensorarrayAsyncLogPublishFrameError` |
| `esp_err_t sensorarrayAsyncLogPublishOverrun(uint32_t sequence, int64_t elapsedUs, int64_t periodUs)` [2216](sensorarrayAsyncLog.c#L2216) | 记录实际帧时长超出周期预算的事件。 / Publish an event for capture duration exceeding its period budget. | `sensorarrayDelayFramePeriodSince` |
| `esp_err_t sensorarrayAsyncLogPublishFrameError(const sensorarrayFrame_t *frame, esp_err_t readErr, bool allRawZero, bool bootOk)` [2231](sensorarrayAsyncLog.c#L2231) | 提交带序号与错误码的帧失败事件。 / Submit a frame-error event with its sequence and error code. `ESP_ERR_INVALID_ARG` | `sensorarrayRunMainLoop` |
| `esp_err_t sensorarrayAsyncLogPublishCommandApplied(uint32_t sequence, const sensorarrayCommand_t *command)` [2282](sensorarrayAsyncLog.c#L2282) | 将已经由采集端应用的命令作为异步事件报告。 / Publish a command that acquisition has already applied. `ESP_ERR_INVALID_ARG` | `sensorarrayApplyPendingCommands` |
| `esp_err_t sensorarrayAsyncLogPublishTextEvent(const char *text, size_t length)` [2299](sensorarrayAsyncLog.c#L2299) | 将普通文本按长度复制到异步事件路径。 / Copy ordinary text into the bounded asynchronous event path. `ESP_ERR_INVALID_ARG` | `sensorarrayScanTask` / `sensorarrayAcqEventPrintf` |
| `esp_err_t sensorarrayAsyncLogPublishProtocolEvent(const char *text, size_t length)` [2315](sensorarrayAsyncLog.c#L2315) | 将协议生命周期文本提交到优先队列。 / Submit protocol lifecycle text to the priority queue. `ESP_ERR_INVALID_ARG` `ESP_ERR_INVALID_STATE` `ESP_ERR_TIMEOUT` | `sensorarrayEmitPayloadTooLargeDrop` / `sensorarrayScanTask` |

### sensorarrayAsyncLogStorage.h

| C 调用签名 / C signature | 行为及返回 / Behaviour and return | 调用位置 / Called by |
| --- | --- | --- |
| `static inline bool sensorarrayAsyncLogStorageEnsure(sensorarrayAsyncLogStorage_t *storage, size_t bytes, bool preferPsram)` [18](sensorarrayAsyncLogStorage.h#L18) | 按内存能力分配长期缓冲，避免为每帧重新分配。 / Allocate persistent buffers in eligible memory, avoiding per-frame allocation. | `sensorarrayAsyncLogInit` |

### sensorarrayFrameOutput.c

| C 调用签名 / C signature | 行为及返回 / Behaviour and return | 调用位置 / Called by |
| --- | --- | --- |
| `esp_err_t sensorarrayFrameOutputPrint(const sensorarrayFrame_t *frame)` [7](sensorarrayFrameOutput.c#L7) | 在 Serial 数据 sink 启用时就地编码完整帧并将单个数据包提交至本地 Serial 队列。 / When the serial data sink is enabled, encode the complete frame and submit it as one packet to the local serial queue. `ESP_ERR_INVALID_ARG` | `sensorarrayRunMainLoop` |

### sensorarrayFramePublication.h

| C 调用签名 / C signature | 行为及返回 / Behaviour and return | 调用位置 / Called by |
| --- | --- | --- |
| `static inline sensorarrayTransportFrameIdentity_t sensorarrayFramePublicationIdentity(const sensorarrayFrame_t *frame)` [7](sensorarrayFramePublication.h#L7) | 从源帧复制 session、boot、revision、各域 ID 和序号到传输身份。 / Copy session, boot, revision, domain IDs and sequence from the captured frame to transport identity. | `sensorarrayAsyncLogPublishFrameSnapshot` / `sensorarrayFramePublicationMetadata` |
| `static inline bool sensorarrayFramePublicationMetadata(const sensorarrayFrame_t *frame, char tag, sensorarrayTransportFrameMetadata_t *out)` [24](sensorarrayFramePublication.h#L24) | 为 C/V/R/M 标签选择其兼容字段域并同时复制完整配置身份。 / Select the legacy field domain for the C/V/R/M tag and copy the complete capture identity. | `sensorarrayNetFramePublish` |

### sensorarrayOutputPolicy.h

| C 调用签名 / C signature | 行为及返回 / Behaviour and return | 调用位置 / Called by |
| --- | --- | --- |
| `static inline sensorarrayOutputCongestionDecision_t sensorarrayOutputCongestionDecide(bool freeSlotAvailable, bool queuedFrameAvailable, bool dropOldestEnabled)` [19](sensorarrayOutputPolicy.h#L19) | 根据空槽、已有待发帧和淘汰策略决定接纳、替换或丢弃。 / Choose admission, replacement or drop from free slots, queued work and eviction policy. | `sensorarrayAsyncLogPublishFrameSnapshot` |

### sensorarrayTextProtocol.c

| C 调用签名 / C signature | 行为及返回 / Behaviour and return | 调用位置 / Called by |
| --- | --- | --- |
| `static size_t sensorarrayTextAppend(char *buffer, size_t bufferSize, size_t position, const char *format, ...)` [17](sensorarrayTextProtocol.c#L17) | 在剩余容量内格式化并推进已写字节数，拒绝截断的帧。 / Append formatted text within remaining capacity, advancing used bytes and rejecting truncation. | `sensorarrayTextAppendIdentity` / `sensorarrayTextProtocolBuildMixedFrame` |
| `static size_t sensorarrayTextAppendIdentity(char *buffer, size_t size, size_t position, const sensorarrayFrame_t *frame, bool trailer)` [39](sensorarrayTextProtocol.c#L39) | 追加源帧的 session、revision、行数/模式/profile 独立身份字段。 / Append source session/revision and independent mode, row-count and row-profile identity fields. | `sensorarrayTextProtocolBuildMixedFrame` / `sensorarrayTextProtocolBuildCapFrame` |
| `static uint32_t sensorarrayTextCrc32(const uint8_t *data, size_t length)` [69](sensorarrayTextProtocol.c#L69) | 对实际编码字节计算用于 K trailer 的 CRC32。 / Compute the K-trailer CRC32 over encoded bytes. | `sensorarrayTextProtocolBuildMixedFrame` / `sensorarrayTextProtocolBuildCapFrame` |
| `static int32_t sensorarrayTextCapFixed(const sensorarrayFrame_t *frame, size_t cell)` [99](sensorarrayTextProtocol.c#L99) | 将 pF 数值转为 CAP 线上使用的定点整数。 / Convert pF to the fixed-point integer used by the CAP wire format. | `sensorarrayTextMixedValue` / `sensorarrayTextProtocolBuildCapFrame` |
| `static uint8_t sensorarrayTextMixedRowMask(uint64_t mask, uint8_t row)` [117](sensorarrayTextProtocol.c#L117) | 为指定行提取该行在 64 位矩阵掩码中的部分。 / Extract the selected row's portion of a 64-bit matrix mask. | `sensorarrayTextProtocolBuildMixedFrame` |
| `static char sensorarrayTextMixedProfileChar(sensorarrayMeasurementMode_t mode)` [122](sensorarrayTextProtocol.c#L122) | 将该行测量模式映射为混合帧中的 C、V、R 或空位标识。 / Map the row mode to the mixed frame's C/V/R or unused marker. | `sensorarrayTextProtocolBuildMixedFrame` |
| `static const char *sensorarrayTextMixedModeName(sensorarrayMeasurementMode_t mode)` [129](sensorarrayTextProtocol.c#L129) | 返回混合模式的显示名称。 / Return the display name for mixed mode. | `sensorarrayTextProtocolBuildMixedFrame` |
| `static int64_t sensorarrayTextMixedValue(const sensorarrayFrame_t *frame, sensorarrayMeasurementMode_t mode, size_t index)` [136](sensorarrayTextProtocol.c#L136) | 按该行模式取出正确的定点科学数值。 / Select the correct fixed-point scientific value for the row's mode. | `sensorarrayTextProtocolBuildMixedFrame` |
| `static uint64_t sensorarrayTextActiveMask(uint8_t rows)` [148](sensorarrayTextProtocol.c#L148) | 把活动行数展开为低位连续的测点有效范围掩码。 / Expand the active-row count to the contiguous low-bit cell range. | `sensorarrayTextProtocolBuildMixedFrame` / `sensorarrayTextProtocolBuildCapFrame` |
| `esp_err_t sensorarrayTextProtocolBuildMixedFrame(const sensorarrayFrame_t *frame, sensorarrayTextPacket_t *outPacket)` [155](sensorarrayTextProtocol.c#L155) | 将逐行模式、单位与掩码连同 M/MR 数据和整帧 K 校验编码。 / Encode per-row mode, units and masks in M/MR records and the frame's K integrity trailer. `ESP_ERR_INVALID_ARG` `ESP_ERR_INVALID_SIZE` | `sensorarrayTextProtocolBuildFrame` / `sensorarrayTextProtocolSelfTest` |
| `esp_err_t sensorarrayTextProtocolBuildCapFrame(const sensorarrayFrame_t *frame, sensorarrayTextPacket_t *outPacket)` [267](sensorarrayTextProtocol.c#L267) | 将 CAP 帧的值、有效掩码和配置身份编码为 C/D/K 文本包。 / Encode CAP values, masks and configuration identity into a C/D/K packet. `ESP_ERR_INVALID_ARG` `ESP_ERR_INVALID_SIZE` | `sensorarrayTextProtocolBuildFrame` / `sensorarrayTextProtocolSelfTest` |
| `static uint32_t sensorarrayTextCountBits64(uint64_t value)` [359](sensorarrayTextProtocol.c#L359) | 统计 64 位掩码中置位的数量，用于帧计数。 / Count set bits in a 64-bit mask for frame accounting. | `sensorarrayTextProtocolBuildMeasurementFrame` |
| `esp_err_t sensorarrayTextProtocolBuildMeasurementFrame(const sensorarrayFrame_t *frame, sensorarrayTextPacket_t *outPacket)` [369](sensorarrayTextProtocol.c#L369) | 编码同一 VOLT 或 RES 帧的定点值、Xhh、PGA、掩码与 K 校验。 / Encode fixed-point VOLT or RES values, Xhh, PGA, masks and K integrity trailer. `ESP_ERR_INVALID_ARG` `ESP_ERR_INVALID_SIZE` | `sensorarrayTextProtocolBuildFrame` / `sensorarrayTextProtocolSelfTest` |
| `esp_err_t sensorarrayTextProtocolBuildFrame(const sensorarrayFrame_t *frame, sensorarrayTextPacket_t *outPacket)` [533](sensorarrayTextProtocol.c#L533) | 根据已采帧的模式分派至 CAP、VOLT/RES 或 MIXED 真实编码器。 / Dispatch a completed frame to the CAP, VOLT/RES or MIXED wire encoder. `ESP_ERR_INVALID_ARG` | `sensorarrayAsyncLogPublishFrameSnapshot` / `sensorarrayFrameOutputPrint` |
| `bool sensorarrayTextProtocolSelfTest(sensorarrayFrame_t *scratchFrame, uint32_t *outChecks)` [560](sensorarrayTextProtocol.c#L560) | 用调用者的整帧工作区构造代表性输入，通过真实编码器核对输出并返回检查数。 / Build representative inputs in caller scratch, verify real encoder output and return the check count. | `sensorarrayInitRuntime` |

### sensorarrayUsbSink.c

| C 调用签名 / C signature | 行为及返回 / Behaviour and return | 调用位置 / Called by |
| --- | --- | --- |
| `esp_err_t sensorarrayUsbSinkInit(void)` [35](sensorarrayUsbSink.c#L35) | 初始化本地 USB 输出统计与所选后端的 stream profile。 / Initialise local USB output counters and the selected backend's stream profile. | `sensorarrayAsyncLogStartUsbSink` |
| `static esp_err_t sensorarrayUsbSinkPublishClass(const sensorarrayTextPacket_t *packet, bool diagnostic)` [65](sensorarrayUsbSink.c#L65) | 按数据或诊断类别选择本地队列并记录接纳结果。 / Select the local queue for data or diagnostics and account for admission. `ESP_ERR_INVALID_ARG` `ESP_ERR_INVALID_STATE` | `sensorarrayUsbSinkPublish` / `sensorarrayUsbSinkPublishDiagnostic` |
| `esp_err_t sensorarrayUsbSinkPublish(const sensorarrayTextPacket_t *packet)` [90](sensorarrayUsbSink.c#L90) | 按数据类别把已编码文本交给 USB 本地队列。 / Submit an encoded data packet to the local USB queue. | `sensorarrayAsyncLogTask` |
| `esp_err_t sensorarrayUsbSinkPublishDiagnostic(const sensorarrayTextPacket_t *packet)` [95](sensorarrayUsbSink.c#L95) | 将诊断文本单独投递到调试/日志通道。 / Submit diagnostic text through the debug/log channel. | `sensorarrayAsyncLogPrintCompactSummary` |
| `void sensorarrayUsbSinkGetStats(sensorarrayUsbSinkStats_t *outStats)` [100](sensorarrayUsbSink.c#L100) | 查找 统计。 / Look up stats. | `sensorarrayRuntimeQueryCommand` / `sensorarrayAsyncLogSummarySetBaseline` |
| `esp_err_t sensorarrayUsbSinkSetStreamProfile(sensorarrayUsbStreamMode_t mode, uint32_t dataEvery, uint32_t diagEvery)` [113](sensorarrayUsbSink.c#L113) | 校验所请求流模式及数据/诊断分频，原子更新 USB 输出策略。 / Validate requested stream mode and data/diagnostic cadence and update USB output policy atomically. `ESP_ERR_INVALID_ARG` | `sensorarrayRuntimeQueryCommand` |
| `sensorarrayUsbStreamProfile_t sensorarrayUsbSinkGetStreamProfile(void)` [150](sensorarrayUsbSink.c#L150) | 复制当前 USB 数据/调试流模式与各自输出间隔。 / Copy current USB data/debug stream settings and output intervals. | `sensorarrayRuntimeQueryCommand` / `sensorarrayAsyncLogPrintCompactSummary` |
| `bool sensorarrayUsbSinkShouldEmitData(uint32_t sequence)` [159](sensorarrayUsbSink.c#L159) | 按 USB 后端与当前 profile 判定给定序号的数据是否获准输出。 / Apply USB backend and stream profile to data emission for the supplied sequence. | `sensorarrayAsyncLogTask` |
| `const char *sensorarrayUsbStreamModeName(sensorarrayUsbStreamMode_t mode)` [172](sensorarrayUsbSink.c#L172) | 返回USB流模式的显示名称。 / Return the display name for USB stream mode. | `sensorarrayRuntimeQueryCommand` |

## 参数索引 / Parameter index

相同名称按模块共享解释。完整类型、数组维度与输入输出方向以对应函数签名和调用点为准。指针参数所需存储由调用者提供，异步作业另受上面的所有权约束。

Entries share parameter descriptions within this module. Refer to the signature and its caller for the precise type, array shape and direction. The caller supplies pointer storage, subject to the ownership rules above for asynchronous work.

| 参数 / Parameter | 含义 / Meaning |
| --- | --- |
| `...`, `format` | printf 风格的格式串与匹配格式的可变参数。 / printf-style format string and its matching variadic arguments. |
| `allRawZero` | 全部原始码零点。 / All raw zero. |
| `arg` | 回调或任务创建时绑定的上下文；在该调用的有效期内保留其存储。 / Context bound at callback registration or task creation. Keep its backing object alive for the operation. |
| `bootOk` | 启动通过。 / Boot ok. |
| `buffer`, `out` | 调用者提供的回复、暂存或结果缓冲区，使用随同的容量/类型。 / Caller-provided reply, scratch or result storage with the accompanying capacity/type. |
| `bufferSize`, `size` | 接收文本或字节缓冲区的容量。文本输出还需预留 NUL。 / Capacity of the supplied byte/text buffer. A text output must also accommodate its NUL terminator. |
| `bytes` | 校准记录/载荷或原始字节；outPayload 指向允许写入的目标。 / Calibration record/payload or raw bytes. outPayload names caller-writable storage. |
| `cell` | 测点。 / Cell. |
| `command`, `data`, `text` | 按该函数声明传入的文本、命令、操作码或数据字节，显式 length 限定可读范围。 / Text, command, opcode or data bytes as declared. Where supplied, length bounds the readable range. |
| `dataEvery` | 数据周期。 / Data every. |
| `diagEvery` | 诊断周期。 / Diagnostics every. |
| `diagnostic`, `summary` | 本次启动、采样或调度的汇总/诊断记录，类型见对应函数。 / Summary/diagnostic evidence for this startup, sample or scheduling operation, typed by the function. |
| `dropOldestEnabled` | 丢弃最旧启用。 / Drop oldest enabled. |
| `elapsedUs` | 经过，单位 µs。 / Elapsed in µs. |
| `emitted` | 已输出。 / Emitted. |
| `event` | 事件。 / Event. |
| `frame` | 当前采集帧。const 指针供读取/编码，可写指针供对应采集步骤产生或更新数据。 / Capture frame. A const pointer is an input to readers/encoders. A writable pointer receives or updates acquisition results. |
| `frameAgeUs` | 帧龄期，单位 µs。 / Frame age in µs. |
| `freeSlotAvailable` | 空闲槽可用。 / Free slot available. |
| `identity` | 帧的不可变配置身份或同一版本的完整权威快照。 / Immutable frame configuration identity or the authority snapshot for that revision. |
| `index` | 索引。 / Index. |
| `length` | 输入、载荷、底层存储或读写传输的字节数。 / Byte length of input, payload, backing storage or transfer. |
| `mask` | 作用域内的行、通道或测点选择位。完整科学帧的每个测点各占一位。 / Bitset of selected rows, channels or cells. A full science frame uses one bit per cell. |
| `measureFrameUs` | 测量帧，单位 µs。 / Measure frame in µs. |
| `mode` | 对应调用域的模式枚举，保留目标、当前帧或恢复上下文的含义。 / Mode enum in the declared domain, selecting the target, frame or recovery mode as indicated. |
| `nowUs`, `startUs` | 单调时钟的事件时间，单位 µs。 / Event time on the monotonic clock in µs. |
| `offset` | 偏移。 / Offset. |
| `outChecks` | 接收检查项。 / Receives checks. |
| `outFrame` | 接收结果。当前采集帧。const 指针供读取/编码，可写指针供对应采集步骤产生或更新数据。 / Output. Capture frame. A const pointer is an input to readers/encoders. A writable pointer receives or updates acquisition results. |
| `outMeasureFrameUs` | 接收测量帧，单位 µs。 / Receives Measure frame in µs. |
| `outPacket` | 接收结果。编码器产生的文本包，length 标记实际使用字节，sequence 关联源帧。 / Output. Encoded text packet. length bounds used bytes and sequence identifies the source frame. |
| `outProtectionFlags` | 接收结果。调用域对应的标志位；帧保护位在编码、入队和重试间保留。 / Output. Domain-specific bit flags. Frame protection flags survive encoding, queuing and retries. |
| `outPublishedUs` | 接收已发布，单位 µs。 / Receives Published in µs. |
| `outputUs` | 输出，单位 µs。 / Output in µs. |
| `outQueueDepth` | 接收队列深度。 / Receives queue depth. |
| `outStats`, `stats` | 函数签名注明的结果、快照或统计结构，调用者分配复制目标。 / Result, snapshot or statistics object of the declared type, with caller-owned output storage. |
| `outTextPacket` | 接收文本数据包。 / Receives text packet. |
| `packet` | 编码器产生的文本包，length 标记实际使用字节，sequence 关联源帧。 / Encoded text packet. length bounds used bytes and sequence identifies the source frame. |
| `periodUs` | 周期，单位 µs。 / Period in µs. |
| `position` | 位置。 / Position. |
| `preferPsram` | 优先PSRAM。 / Prefer psram. |
| `queueDepth` | 队列深度。 / Queue depth. |
| `queuedFrameAvailable` | 排队帧可用。 / Queued frame available. |
| `readBytes` | 读取，单位 字节。 / Read in bytes. |
| `readCount` | 读取数量。 / Read count. |
| `readErr` | 读取错误码。 / Read err. |
| `reason` | 操作或拒绝原因字符串；outReason 用于返回原因。 / Operation/rejection reason string. outReason receives the reason. |
| `row` | 行或列的坐标。通用 matrixEngineRegion_t 的原点为零；以板级 S/D 位置操作的接口从 1 编号。 / Row or column coordinate. Generic matrixEngineRegion_t origins are zero-based. Board S/D coordinates are one-based. |
| `rows` | 行数值或行配置快照，由该函数的参数类型决定。 / Active count or row-configuration snapshot as specified by its declared type. |
| `scratchFrame` | 编码器自检可写的整帧工作区。 / Caller-provided writable full-frame scratch space for the encoder self-test. |
| `sequence` | 对应采集帧的序号，必须与提供的帧、事件或缓存上下文属于同次启动。 / Capture sequence belonging to the same boot as the supplied frame, event or cache context. |
| `storage` | 接收、发送或长期存储缓冲；与签名中的容量或传输长度配对。 / Receive, transmit or backing buffer paired with the supplied capacity/transfer length. |
| `tag` | 标签。 / Tag. |
| `trailer` | 尾帧。 / Trailer. |
| `value` | 值。 / Value. |
| `writeBytes` | 写入，单位 字节。 / Write in bytes. |
| `writeCount` | 写入数量。 / Write count. |

[项目总览 / Project guide](../../README.md)

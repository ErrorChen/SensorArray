# BLE GATT 传输与完整快照 / BLE GATT transport and complete snapshots

GATT 服务 0x00FF 提供 FF10 命令写入、FF11 控制回复、FF20 数据、FF30 日志与生命周期事件。应用在独立控制任务处理命令，在 TX owner 串行执行 GATT 写。FAST 需要启用通知的 CCCD。SAFE 在可 indication 时使用带确认的 indication，只有 notify 订阅时仍可使用通知。 / GATT service 0x00FF exposes FF10 for command writes, FF11 for control replies, FF20 for data and FF30 for logs/lifecycle events. A control task dispatches commands and a separate TX owner serialises GATT writes. FAST requires notification subscription. SAFE uses confirmed indications when subscribed for indications and also supports notify-only clients. [公共 API / Public API](include/sensorarrayBle.h) · [调度接口 / Output scheduler](sensorarrayBleOutputSchedule.h) · [传输层 / Transport](../../core/transport/)

## 发布完整帧 / Publishing complete frames

生产者从编码器取得 `sensorarrayBleFramePublication_t`，填充完整负载、普通负载和原始 sessionId、bootId、revision、sequence。复用下述连接代提交。完整数据和普通数据可以相同，若普通数据为预览，保护帧始终取完整数据。函数在本地准入成功前复制所选字节。 / Supply the encoder's full bundle, normal payload and original sessionId, bootId, revision and sequence in sensorarrayBleFramePublication_t. Use the same captured connection generation for admission. Normal data may be the full bundle or a preview. Protected records always use the full version. Successful local admission copies the selected bytes.

```c
esp_err_t offer_frame(const sensorarrayBleFramePublication_t *frame)
{
    uint32_t generation;
    if (!sensorarrayBleGetSendGeneration(SENSORARRAY_BLE_CH_DATA, &generation))
        return ESP_ERR_INVALID_STATE;
    return sensorarrayBleNotifyFrameForGeneration(frame, generation);
}
```

活动 DATA stream 的新配置有首帧保护。一秒内若没有更新序号的完整数据本地发送完成，则再次要求完整快照。发送顺序为控制/生命周期、保护帧、普通 DATA、普通 LOG。DATA 活动时抑制新普通日志。普通帧在连续三个有效窗口的本地容量拒绝率均超过 20% 后，每步降低五个百分点；实际干净窗口累计十秒可升一步。限速时仍有探测帧。连接、采集和主机接收各自有独立状态。 / A new configuration on an active DATA stream has first-frame protection. If no sequence-advancing full bundle completes locally for one second, another full snapshot becomes due. The owner services control/lifecycle, protected frames, ordinary DATA and then ordinary LOG. Active DATA suppresses new ordinary logs. Three valid windows each above 20% local capacity rejection can reduce ordinary selection by five percentage points. Ten seconds of actual clean windows permit a step up. Limited streams retain probes. Link state, acquisition and host receipt are independently observed.

消息上限 2304 字节，保留控制池上限 512 字节。较长消息按协商 ATT MTU 分片，并带消息级 CRC32 供重组。诊断的 SDK 接受、CONF 事件或本地整帧完成分别表示对应层的观测。策略主动跳过返回 `ESP_ERR_NOT_ALLOWED`。固定槽耗尽、连接代过期或守护隔离有各自计数。守护在 500 ms 的 SDK 调用期限后可隔离，观察周期 100 ms，仍持有正在调用的资源。 / Messages are limited to 2304 bytes, with a 512-byte reserved control pool. Larger-than-MTU messages use ATT-aware fragments and a whole-message CRC32 for reassembly. SDK acceptance, CONF events and local full completion describe their respective layers. An intentional policy skip yields ESP_ERR_NOT_ALLOWED. Slot exhaustion, stale epochs and quarantine have separate counters. A 100 ms observer can quarantine calls past the 500 ms SDK budget while retaining ownership of in-flight resources.

| 参数或输出 / Argument or output | 使用约定 / Interpretation |
|---|---|
| `channel`, `data`, `length` | 枚举选 DATA/LOG/CTRL，长度是字节数，不包括 C 字符串末尾 NUL。 / Select DATA, LOG or CTRL. Pass payload bytes and their length excluding the C-string terminator. |
| `expectedConnectionGeneration`, `streamGeneration` | 前者绑定同一客户端连接；后者由有效 DATA 订阅变化推进。只有同一 session/boot 内才能排序 sequence。 / The connection generation binds the client connection. Changes to effective DATA subscription advance the stream generation. Compare sequence only within the same session and boot. |
| `userContext`, `callback`, `outSnapshot` | 回调上下文由注册者保持有效；借用的 RX 数据须在回调返回前用完或复制。快照复制到调用者的结构。 / Keep the registered callback context live. Consume or copy borrowed RX data before returning. Snapshot destinations belong to the caller. |

## [sensorarrayBle.c](sensorarrayBle.c)

### 接口与回调 / Interfaces and callbacks

| C 签名 / C signature | 行为与参数 / Behaviour and arguments | 调用举例 / Representative caller |
|---|---|---|
| `void sensorarrayBleLogHeap(const char *stage)` | 按调用者给定阶段打印当前 BLE 堆诊断。 / Log BLE heap diagnostics under the caller's stage label. | 公开入口 / Callable API |
| `const char *sensorarrayBleTxModeName(sensorarrayBleTxMode_t mode)` | 将枚举映射成 fast 或 safe。 / Return fast or safe for the selected TX mode. | [sensorarrayTransportHandleRuntimeCommand](../../core/transport/sensorarrayTransportCommand.c#L90) |
| `void sensorarrayBleSetTxMode(sensorarrayBleTxMode_t mode)` | 验证并切换 FAST/SAFE，重新判断 DATA stream 是否活动。 / Select a valid FAST/SAFE mode and resynchronise effective DATA activity. | [sensorarrayTransportHandleRuntimeCommand](../../core/transport/sensorarrayTransportCommand.c#L90) |
| `sensorarrayBleTxMode_t sensorarrayBleGetTxMode(void)` | 读取当前发送模式。 / Return the current transmit mode. | [sensorarrayTransportHandleRuntimeCommand](../../core/transport/sensorarrayTransportCommand.c#L90) |
| `bool sensorarrayBleCanSend(sensorarrayBleChannel_t channel)` | 判定该通道当前是否可发送。 / Test whether the channel currently admits sends. | [sensorarrayTransportBleChannelEnabled](../../core/transport/sensorarrayTransportStats.c#L169) |
| `bool sensorarrayBleGetSendGeneration(sensorarrayBleChannel_t channel, uint32_t *outConnectionGeneration)` | 按通道检查可发送条件，将当前连接代写入输出。 / Capture the current connection generation for a sendable channel. | [sensorarrayTransportDrainGuaranteedText](../../core/transport/sensorarrayTransport.c#L735) |
| `esp_err_t sensorarrayBleInit(const sensorarrayBleConfig_t *config)` | 按设备名、偏好 MTU/PHY/功率创建持久队列、任务、守护与 BLE 栈。 / Initialise queues, tasks, guard and BLE stack using device name and preferred MTU, PHY and power. | [sensorarrayTransportInitOwned](../../core/transport/sensorarrayTransport.c#L1127) |
| `esp_err_t sensorarrayBleNotify(sensorarrayBleChannel_t channel, const uint8_t *data, size_t length)` | 给当前通道提交完整 ASCII 消息，返回本地入队结果。 / Submit a complete ASCII message on the current channel and return local admission status. | [sensorarrayTransportPublishControlReply](../../core/transport/sensorarrayTransport.c#L1383) |
| `esp_err_t sensorarrayBleNotifyForGeneration( sensorarrayBleChannel_t channel, const uint8_t *data, size_t length, uint32_t expectedConnectionGeneration)` | 仅对给定非零连接代提交消息。 / Submit the message for the specified nonzero connection generation. | [sensorarrayTransportProcessDescriptor](../../core/transport/sensorarrayTransport.c#L610) |
| `esp_err_t sensorarrayBleNotifyFrameForGeneration( const sensorarrayBleFramePublication_t *publication, uint32_t expectedConnectionGeneration)` | 按身份和调度策略选择普通或完整数据，复制后再提交到对应槽。 / Choose normal/full bytes by identity and schedule, then copy and publish the corresponding slot. | [sensorarrayTransportProcessDescriptor](../../core/transport/sensorarrayTransport.c#L610) |
| `esp_err_t sensorarrayBleNotifyLifecycleForGeneration( const uint8_t *data, size_t length, uint32_t expectedConnectionGeneration)` | 用 FF30 格式经保留控制资源发送生命周期文本，需非零连接代。 / Use reserved control resources for FF30 lifecycle text with a nonzero connection generation. | [sensorarrayTransportProcessDescriptor](../../core/transport/sensorarrayTransport.c#L610) |
| `void sensorarrayBleSetControlRxCallback(sensorarrayBleControlRxCallback_t callback, void *userContext)` | 提供命令处理函数与 userContext，供独立接收任务调用。 / Register the command handler and userContext for invocation from the control task. | [sensorarrayTransportInitOwned](../../core/transport/sensorarrayTransport.c#L1127) |
| `bool sensorarrayBleIsReady(void)` | 返回控制器、主机、GATT 和广播标志是否同时就绪。 / Require controller, host, GATT and advertising readiness flags. | 公开入口 / Callable API |
| `bool sensorarrayBleIsConnected(void)` | 读取是否存在活动连接。 / Return the observed connection state. | [sensorarrayTransportBleChannelEnabled](../../core/transport/sensorarrayTransportStats.c#L169) |
| `bool sensorarrayBleIsSubscribed(sensorarrayBleChannel_t channel)` | 查询 channel 的 CCCD 是否至少开启通知或 indication。 / Test whether channel's CCCD has notification or indication enabled. | [sensorarrayAsyncLogPrintCompactSummary](../../main/output/sensorarrayAsyncLog.c#L1052) |
| `bool sensorarrayBleIsCongested(void)` | 读取当前 GATT 拥塞标志。 / Read the current GATT congestion flag. | [sensorarrayTransportProcessDescriptor](../../core/transport/sensorarrayTransport.c#L610) |
| `void sensorarrayBleGetStats(sensorarrayBleStats_t *outStats)` | 复制消息、分片、槽、控制队列统计及任务栈剩余字节。 / Copy message, fragment, slot, control queue and task stack headroom statistics. | [sensorarrayTransportPrintNetInit](../../core/transport/sensorarrayTransport.c#L592) |
| `void sensorarrayBleGetLinkSnapshot(sensorarrayBleLinkSnapshot_t *outSnapshot)` | 复制连接代、MTU/PHY、请求与回调结果及 SDK 墙钟计数。 / Copy connection generation, MTU/PHY, negotiation evidence and SDK call wall-time counters. | [sensorarrayLinkDiagnosticsFormat](../../main/control/sensorarrayLinkDiagnostics.c#L17) |
| `void sensorarrayBleGetGuardSnapshot(sensorarrayBleGuardSnapshot_t *outSnapshot)` | 复制发送超时与隔离证据、是否已有返回及是否需重启。 / Copy guard timeout/quarantine evidence, known completion and restart requirement. | [sensorarrayLinkDiagnosticsFormat](../../main/control/sensorarrayLinkDiagnostics.c#L17) |
| `void sensorarrayBleGetLogPolicySnapshot(sensorarrayBleLogPolicySnapshot_t *outSnapshot)` | 在同一锁内读取 DATA 活动状态、普通日志准入及被抑制计数。 / Snapshot active DATA, ordinary LOG admission and policy-drop counters under one lock. | [sensorarrayLinkDiagnosticsFormat](../../main/control/sensorarrayLinkDiagnostics.c#L17) |
| `void sensorarrayBleGetFlowSnapshot(sensorarrayBleFlowSnapshot_t *outSnapshot)` | 复制普通帧选择、拒绝率、周期和首帧保护、完整帧间隔指标。 / Copy ordinary selection, rejection, first/periodic protection and full-completion gap evidence. | [sensorarrayLinkDiagnosticsFormat](../../main/control/sensorarrayLinkDiagnostics.c#L17) |

### 组件内部接口 / Component interface

| C 签名 / C signature | 行为与参数 / Behaviour and arguments | 调用举例 / Representative caller |
|---|---|---|
| `void sensorarrayBleObserveSendDeadline(void)` | 定时器调用的受锁守护检查入口。 / Run the locked send-deadline observation from the watchdog timer. | [sensorarrayBleWatchdogTick](sensorarrayBleWatchdog.c#L13) |

### 内部 / Internal

| C 签名 / C signature | 行为与参数 / Behaviour and arguments | 调用举例 / Representative caller |
|---|---|---|
| `static void sensorarrayBleStatsIncrement(uint32_t *counter)` | 在 BLE 临界区内递增给定计数器。 / Increment the supplied counter under the BLE critical section. | [sensorarrayBlePlanFragments](sensorarrayBle.c#L1175) |
| `static void sensorarrayBleStatsNoteDrop(sensorarrayBleChannel_t channel)` | 按 channel 累加丢弃和总消息丢弃计数。 / Count one channel drop and one dropped message. | [sensorarrayBleNoteDrop](sensorarrayBle.c#L985) |
| `static void sensorarrayBleStatsNoteMessageSent(sensorarrayBleChannel_t channel, bool countFragment)` | 按通道记录完整消息成功，可同时计入一个分片。 / Count a completed message on channel and optionally a fragment. | [sensorarrayBleSendPayload](sensorarrayBle.c#L1573) |
| `static void sensorarrayBleStatsSetInitError(esp_err_t err)` | 保存可查询的初始化错误码。 / Publish the initialisation error for statistics readers. | [sensorarrayBleStartAdvertising](sensorarrayBle.c#L2027) |
| `static uint32_t sensorarrayBleNextGeneration(uint32_t value)` | 递增有限代号，回绕跳过零。 / Advance the generation, skipping zero on wrap. | [sensorarrayBleFrameFlowSyncLocked](sensorarrayBle.c#L527) |
| `static uint32_t sensorarrayBleCountUsedSlotsLocked(void)` | 在锁内统计 DATA、LOG、CTRL、保护池占用总数。 / Count used DATA, LOG, CTRL and protected slots with the lock held. | [sensorarrayBleUpdateSlotUseLocked](sensorarrayBle.c#L488) |
| `static void sensorarrayBleUpdateSlotUseLocked(void)` | 重新统计槽占用量并更新历史高水位。 / Refresh current slot occupancy and its high-water mark. | [sensorarrayBleAllocateTxDescriptor](sensorarrayBle.c#L629) |
| `static bool sensorarrayBleDataActiveLocked(void)` | 结合 guard、GATT、连接、模式和 DATA CCCD 判定有效订阅。 / Test guard, GATT, connection, TX mode and DATA CCCD for effective subscription. | [sensorarrayBleFrameFlowSyncLocked](sensorarrayBle.c#L527) |
| `static bool sensorarrayBleFrameSameBoot(const sensorarrayBleFrameIdentity_t *a, const sensorarrayBleFrameIdentity_t *b)` | 比较 sessionId 和 bootId 是否都相同。 / Compare both sessionId and bootId for equality. | [sensorarrayBleFrameSameConfig](sensorarrayBle.c#L513) |
| `static bool sensorarrayBleFrameSameConfig(const sensorarrayBleFrameIdentity_t *a, const sensorarrayBleFrameIdentity_t *b)` | 要求同一启动身份及相同配置 revision。 / Require matching boot identity and config revision. | [sensorarrayBleFrameProtectedRepresentedLocked](sensorarrayBle.c#L570) |
| `static bool sensorarrayBleFrameSequenceBefore(uint32_t a, uint32_t b)` | 按 32 位序号半区间比较；恰好相距半周时不排序。 / Compare same-boot serial numbers by the 32-bit half range, leaving the exact half-range unordered. | [sensorarrayBleFrameFullCompletedLocked](sensorarrayBle.c#L611) |
| `static void sensorarrayBleFrameFlowSyncLocked(void)` | 活动状态变化时推进 streamGeneration，重置首帧和周期保护证据。 / Advance stream generation on effective activity changes and reset first/periodic protection evidence. | [sensorarrayBleAllocateTxDescriptor](sensorarrayBle.c#L629) |
| `static void sensorarrayBleFrameFlowObserveLocked(uint64_t nowUs)` | 将当前流的身份、本地计数及单调时间提交给调度器。 / Feed stream identity, local counters and monotonic time into the output scheduler. | [sensorarrayBleSendDescriptor](sensorarrayBle.c#L1725) |
| `static bool sensorarrayBleFrameProtectedRepresentedLocked(uint8_t flag)` | 检查匹配代号的保留槽是否已代表给定的保护义务。 / Test if an existing reserved slot already represents this protection flag and generation. | [sensorarrayBleNotifyFrameForGeneration](sensorarrayBle.c#L2715) |
| `static void sensorarrayBleFramePeriodicLocked(uint64_t nowUs)` | 根据 nowUs 推进一秒快照期限，累计错过的周期。 / Advance one-second snapshot obligations from nowUs and count missed intervals. | [sensorarrayBleNotifyFrameForGeneration](sensorarrayBle.c#L2715) |
| `static void sensorarrayBleFrameFullCompletedLocked( const sensorarrayBleFrameIdentity_t *identity, uint64_t completedUs)` | 当前配置的序号前移且完整发送后更新本地完成时间及下一期限。 / Advance full-completion time and the next deadline for a newer whole bundle in the current config. | [sensorarrayBleSendDescriptor](sensorarrayBle.c#L1725) |
| `static esp_err_t sensorarrayBleAllocateTxDescriptor( sensorarrayBleChannel_t channel, size_t length, uint32_t crc, uint32_t expectedConnectionGeneration, bool priority, sensorarrayBleTxDescriptor_t *outDesc)` | 检查订阅、代号、优先级和容量，保留槽并在 outDesc 绑定长度及 CRC。 / Check subscription, generation, priority and capacity, then reserve a slot and bind length/CRC in outDesc. | [sensorarrayBleNotifyInternal](sensorarrayBle.c#L2648) |
| `static uint8_t *sensorarrayBleDescriptorData(const sensorarrayBleTxDescriptor_t *desc)` | 在描述符与槽代号匹配时返回持有的字节存储。 / Locate retained payload storage using descriptor slot identity and generation. | [sensorarrayBleSendDescriptor](sensorarrayBle.c#L1725) |
| `static bool sensorarrayBleDescriptorCurrent(const sensorarrayBleTxDescriptor_t *desc)` | 核验槽代号、连接、流身份和允许发送的状态。 / Verify slot, connection and applicable stream identity before use. | [sensorarrayBleSendDescriptor](sensorarrayBle.c#L1725) |
| `static void sensorarrayBleReleaseTxDescriptor(const sensorarrayBleTxDescriptor_t *desc)` | 按原描述符和代号归还正确的槽，失配则记数。 / Return a matching slot by descriptor/generation and count release mismatches. | [sensorarrayBleTxTask](sensorarrayBle.c#L1874) |
| `static void sensorarrayBleLogHeapDetail(const char *stage, const char *reason)` | 记录阶段、原因及所用堆内存指标。 / Log heap observations tagged by stage and reason. | [sensorarrayBleLogHeap](sensorarrayBle.c#L886) |
| `static void sensorarrayBleLogBoot(const char *stage, esp_err_t err)` | 报告启动阶段、ESP 错误及相关堆指标。 / Log the startup stage, ESP result and associated heap observations. | [sensorarrayBleStartAdvertising](sensorarrayBle.c#L2027) |
| `static void sensorarrayBleBuildAdvData(void)` | 使用服务 UUID 和受长度限制的名称组织广播字节。 / Build advertising bytes with service UUID and a length-bounded device name. | [sensorarrayBleInit](sensorarrayBle.c#L2482) |
| `static int sensorarrayBleCccdChannel(uint16_t handle)` | 将 GATT CCCD 句柄映射到 DATA、LOG、CTRL，未知返回 −1。 / Map a CCCD attribute handle to DATA, LOG or CTRL, or −1 if unknown. | [sensorarrayBleGattCallback](sensorarrayBle.c#L2254) |
| `static uint16_t sensorarrayBleValueHandle(sensorarrayBleChannel_t channel)` | 从逻辑通道得到 TX 值句柄，未知返回零。 / Return the TX value handle for channel or zero for unknown. | [sensorarrayBleSendFragment](sensorarrayBle.c#L1494) |
| `static char sensorarrayBleEnvelopeChannel(sensorarrayBleChannel_t channel)` | 将通道转换成碎片包头的 D、L 或 C。 / Map the channel to the fragment-envelope code D, L or C. | [sensorarrayBleValidateAscii](sensorarrayBle.c#L990) |
| `static uint32_t sensorarrayBleCrc32(const uint8_t *data, size_t length)` | 计算完整消息字节的 IEEE CRC32。 / Compute the IEEE CRC32 over the whole message bytes. | [sensorarrayBleSendPayload](sensorarrayBle.c#L1573) |
| `static void sensorarrayBleNoteDrop(sensorarrayBleChannel_t channel)` | 将此次丢弃交给按通道计数的统计入口。 / Report a dropped message to per-channel accounting. | [sensorarrayBleSendPayload](sensorarrayBle.c#L1573) |
| `static bool sensorarrayBleValidateAscii(sensorarrayBleChannel_t channel, const uint8_t *data, size_t length, uint32_t messageId, uint32_t fragmentIndex)` | 检查给定片段的每个字节是否为 7 位 ASCII，失效时按消息号和偏移报告。 / Check each byte is 7-bit ASCII and report the message/fragment offset of a violation. | [sensorarrayBleSendFragment](sensorarrayBle.c#L1494) |
| `static bool sensorarrayBleSnapshotSendContext( sensorarrayBleChannel_t channel, uint32_t expectedConnectionGeneration, sensorarrayBleSendContext_t *outContext)` | 在一个锁区间内验证期望连接代并复制 MTU、CCCD、模式等发送上下文。 / Validate the expected connection generation and snapshot MTU, CCCD, mode and other send state under one lock. | [sensorarrayBleCanSend](sensorarrayBle.c#L1111) |
| `static bool sensorarrayBleSendContextMatches( sensorarrayBleChannel_t channel, const sensorarrayBleSendContext_t *context, bool *outCongested)` | 比较快照与当前可发送连接，必要时返回拥塞状态。 / Compare a send context with the active channel and optionally return congestion. | [sensorarrayBleSendContextCurrent](sensorarrayBle.c#L1102) |
| `static bool sensorarrayBleSendContextCurrent( sensorarrayBleChannel_t channel, const sensorarrayBleSendContext_t *context)` | 仅查询给定发送上下文是否仍可用。 / Report whether the given channel send context is still current. | [sensorarrayBleSendFragment](sensorarrayBle.c#L1494) |
| `static size_t sensorarrayBleDecimalChars(uint32_t value)` | 返回十进制无符号数所占字符数。 / Count decimal digits for fragment-header sizing. | [sensorarrayBleMaxPayloadSize](sensorarrayBle.c#L1143) |
| `static size_t sensorarrayBleMaxPayloadSize(size_t maximum, char envChannel, uint32_t messageId, uint32_t index, uint32_t count, size_t chunk, size_t messageLength, uint32_t crc)` | 扣除包含通道、序号、总长与 CRC 的实际包头后计算 MTU 可容纳字节数。 / Compute usable MTU capacity after sizing the channel, IDs, lengths and CRC envelope. | [sensorarrayBlePlanFragments](sensorarrayBle.c#L1175) |
| `static bool sensorarrayBlePlanFragments(size_t length, size_t maximum, char envChannel, uint32_t messageId, uint32_t crc, uint32_t *outCount)` | 尝试有限分片布局，成功时写出 outCount。 / Try bounded fragment counts and return a fitting count via outCount. | [sensorarrayBleSendPayload](sensorarrayBle.c#L1573) |
| `static size_t sensorarrayBleTwoFragmentFirstChunk(size_t length, size_t maximum, char envChannel, uint32_t messageId, uint32_t crc)` | 为满足两片 DATA 的特定 MTU 边界选择第一片长度，不适用时返回零。 / Choose the first payload length for the special two-fragment DATA layout, returning zero when unsuitable. | [sensorarrayBleSendPayload](sensorarrayBle.c#L1573) |
| `static void sensorarrayBlePrepareConfirmation( const sensorarrayBleSendContext_t *context, uint16_t handle)` | 清上次通知，登记当前任务、连接代及 GATT 句柄以等 indication 确认。 / Clear old notification state and register task, connection generation and handle for an indication. | [sensorarrayBleSendGattValue](sensorarrayBle.c#L1356) |
| `static void sensorarrayBleClearConfirmation(TaskHandle_t task)` | 确认等待者仍为 task 时清除登记。 / Remove the confirmation waiter only when it belongs to task. | [sensorarrayBleSendGattValue](sensorarrayBle.c#L1356) |
| `static esp_err_t sensorarrayBleAwaitConfirmation( const sensorarrayBleSendContext_t *context, uint16_t handle)` | 有界等待匹配 indication 确认，处理超时、状态码及连接切换。 / Wait for a matching indication confirmation with a timeout, checking connection and GATT result. | [sensorarrayBleSendGattValue](sensorarrayBle.c#L1356) |
| `static void sensorarrayBleObserveSendDeadlineLocked(uint64_t nowUs)` | 检测单次 SDK 发送是否超时，首次超时冻结故障并隔离。 / Check the active SDK send deadline and freeze first-fault quarantine evidence. | [sensorarrayBleObserveSendDeadline](sensorarrayBle.c#L1348) |
| `static esp_err_t sensorarrayBleSendGattValue( const sensorarrayBleSendContext_t *context, sensorarrayBleChannel_t channel, uint16_t handle, uint16_t length, uint8_t *value, bool confirm)` | 执行一次 GATT 值提交，跟踪 SDK 时间、返回和可选确认；优先消息有界重试。 / Submit a GATT value with SDK timing/result tracking, optional confirmation and bounded priority retries. | [sensorarrayBleSendFragment](sensorarrayBle.c#L1494) |
| `static esp_err_t sensorarrayBleSendFragment( const sensorarrayBleSendContext_t *context, sensorarrayBleChannel_t channel, char envChannel, uint32_t messageId, uint32_t fragmentIndex, uint32_t fragmentCount, const uint8_t *data, size_t chunk, size_t messageLength, uint32_t crc, bool confirm)` | 构造带消息号、片号与整条 CRC 的 ASCII 包头，发送单片并计数。 / Build the ASCII envelope with message ID, fragment index and full CRC, then send and account for this fragment. | [sensorarrayBleSendPayload](sensorarrayBle.c#L1573) |
| `static esp_err_t sensorarrayBleSendPayload(sensorarrayBleChannel_t channel, const uint8_t *data, uint16_t length, uint32_t enqueueCrc, uint32_t expectedConnectionGeneration, bool priority, uint32_t expectedStreamGeneration)` | 验证入队 CRC、ASCII、订阅与连接，再按 MTU 整条或分片发送。 / Verify enqueue CRC, ASCII and active connection/subscription, then send whole or fragment for the MTU. | [sensorarrayBleSendDescriptor](sensorarrayBle.c#L1725) |
| `static esp_err_t sensorarrayBleSendDescriptor(const sensorarrayBleTxDescriptor_t *desc)` | 验证槽、发送字节并更新本地保护、耗时和队列进度统计。 / Validate the slot, send retained bytes and record local completion, protection and queue age. | [sensorarrayBleTxTask](sensorarrayBle.c#L1874) |
| `static bool sensorarrayBleReceiveProtected(sensorarrayBleTxDescriptor_t *desc)` | 在保留池中选一个待发描述符，从 pending 改为 owner 持有。 / Select a pending reserved descriptor and transfer ownership to TX. | [sensorarrayBleReceiveNext](sensorarrayBle.c#L1856) |
| `static bool sensorarrayBleReceiveNext(sensorarrayBleTxDescriptor_t *desc)` | 按控制、保护帧、普通 DATA、普通 LOG 的优先顺序非阻塞取工作。 / Take work without waiting in CTRL, protected, ordinary DATA then ordinary LOG order. | [sensorarrayBleTxTask](sensorarrayBle.c#L1874) |
| `static void sensorarrayBleTxTask(void *arg)` | 唯一 TX owner 发送并释放描述符，在工作边界检查 CPU 预算，空闲等待唤醒。 / The sole TX owner sends and releases descriptors, accounts for CPU budget and waits for work when idle. | [sensorarrayBleInit](sensorarrayBle.c#L2482) |
| `static esp_err_t sensorarrayBleQueueTxDescriptor(sensorarrayBleTxDescriptor_t *desc)` | 将已保留描述符送往对应有界队列，失败则回收或计数。 / Place the retained descriptor on its bounded lane queue and handle eviction/reclamation on failure. | [sensorarrayBleNotifyInternal](sensorarrayBle.c#L2648) |
| `static void sensorarrayBleDrainTxQueues(void)` | 排出各待发队列并按原代号归还相应槽。 / Drain pending TX lanes and reclaim each corresponding descriptor slot. | [sensorarrayBleGattCallback](sensorarrayBle.c#L2254) |
| `static void sensorarrayBleDrainControlRxQueue(void)` | 排出已失效的接收控制命令并计为 stale。 / Discard queued stale control commands and count them. | [sensorarrayBleGattCallback](sensorarrayBle.c#L2254) |
| `static void sensorarrayBleControlTask(void *arg)` | 在独立任务中按连接代验证 FF10 命令后，调用上层控制回调。 / Validate FF10 command generations and invoke the application callback from the control task. | [sensorarrayBleInit](sensorarrayBle.c#L2482) |
| `static void sensorarrayBleQueueControlCommand(const esp_ble_gatts_cb_param_t *param)` | 把 GATT 写参数中的命令及连接代复制到控制队列。 / Copy the command from the GATT write event with its connection generation into the RX queue. | [sensorarrayBleGattCallback](sensorarrayBle.c#L2254) |
| `static void sensorarrayBleStartAdvertising(void)` | 在广告数据和 GATT 状态满足要求时请求开始广播。 / Request advertising when advertisement data and GATT state are ready. | [sensorarrayBleGapCallback](sensorarrayBle.c#L2210) |
| `static void sensorarrayBleLogReadySummary(void)` | 根据当前 BLE 初始化、广播和连接状态打印就绪诊断。 / Emit the readiness summary from BLE initialisation and advertising state. | [sensorarrayBleGapCallback](sensorarrayBle.c#L2210) |
| `static void sensorarrayBleResetLinkObservationLocked(void)` | 为新的连接代重置本连接的 PHY、参数、DLE 观测。 / Reset per-link PHY, connection-parameter and data-length evidence for a new generation. | [sensorarrayBleGattCallback](sensorarrayBle.c#L2254) |
| `static void sensorarrayBleRequestLinkParameters(void)` | 对当前连接请求 PHY、连接参数和数据长度更新，记录请求结果。 / Request PHY, connection and data-length settings and retain result codes. | [sensorarrayBleGattCallback](sensorarrayBle.c#L2254) |
| `static bool sensorarrayBleObserveLinkEvent(esp_gap_ble_cb_event_t event, const esp_ble_gap_cb_param_t *param)` | 验证事件归属并记录协商完成证据，包含无法归属特定连接的 DLE 事件。 / Update negotiation evidence with peer attribution where available, retaining unattributed DLE events separately. | [sensorarrayBleGapCallback](sensorarrayBle.c#L2210) |
| `static void sensorarrayBleGapCallback(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)` | 处理 GAP 广播配置、广播状态及连接协商事件。 / Handle GAP advertising setup/status and link parameter events. | [sensorarrayBleInit](sensorarrayBle.c#L2482) |
| `static void sensorarrayBleGattCallback(esp_gatts_cb_event_t event, esp_gatt_if_t gattsIf, esp_ble_gatts_cb_param_t *param)` | 处理 GATT 表注册、连接、MTU、CCCD、命令写入、拥塞和确认回调。 / Handle GATT table registration, connection, MTU, CCCD, command writes, congestion and confirmations. | [sensorarrayBleInit](sensorarrayBle.c#L2482) |
| `static esp_power_level_t sensorarrayBleHighPower(void)` | 返回配置高功率时使用的 ESP_PWR_LVL_P9。 / Return ESP_PWR_LVL_P9 as the preferred high TX power setting. | [sensorarrayBleInit](sensorarrayBle.c#L2482) |
| `static esp_err_t sensorarrayBleStep(const char *stage, esp_err_t err)` | 给初始化步骤记录阶段和失败信息并原样返回错误。 / Annotate the initialisation result with its stage and return the original error. | [sensorarrayBleInit](sensorarrayBle.c#L2482) |
| `static esp_err_t sensorarrayBleFinishInit(esp_err_t error)` | 保存本次初始化结果，只在成功完成时置 initialized。 / Publish the initialisation result and mark initialised only on success. | [sensorarrayBleInit](sensorarrayBle.c#L2482) |
| `static esp_err_t sensorarrayBleInitPayloadStorage(void)` | 一次分配生命周期持久的字节池并分配给各类发送槽。 / Allocate persistent backing storage once and partition it across the TX slot classes. | [sensorarrayBleInit](sensorarrayBle.c#L2482) |
| `static esp_err_t sensorarrayBleNotifyInternal( sensorarrayBleChannel_t channel, const uint8_t *data, size_t length, uint32_t expectedConnectionGeneration, bool priority)` | 验证通道及长度、计算 CRC、复制到预留槽并入队。 / Validate channel and size, calculate CRC, copy to a reserved slot and enqueue. | [sensorarrayBleNotify](sensorarrayBle.c#L2691) |

## [sensorarrayBleOutputSchedule.c](sensorarrayBleOutputSchedule.c)

### 组件内部接口 / Component interface

| C 签名 / C signature | 行为与参数 / Behaviour and arguments | 调用举例 / Representative caller |
|---|---|---|
| `void sensorarrayBleOutputScheduleInit(sensorarrayBleOutputSchedule_t *s, uint64_t nowUs)` | 初始化为普通帧 1000‰ 和给定单调时刻。 / Initialise at full ordinary selection, 1000 permille, and record current time. | [sensorarrayBleFrameFlowObserveLocked](sensorarrayBle.c#L553) |
| `bool sensorarrayBleOutputScheduleObserve(sensorarrayBleOutputSchedule_t *s, uint64_t nowUs, const sensorarrayBleOutputObservation_t *observation)` | 校验同代计数增量，更新窗口压力、完成速率和限速或恢复步进。 / Check same-epoch counter deltas and update pressure, completion rate and step-down/recovery evidence. | [sensorarrayBleFrameFlowSyncLocked](sensorarrayBle.c#L527) |
| `sensorarrayBleOutputTicket_t sensorarrayBleOutputScheduleDecide( sensorarrayBleOutputSchedule_t *s, uint64_t nowUs, const sensorarrayBleOutputEpoch_t *epoch, const sensorarrayBleOutputFrameKey_t *frame, uint8_t protectionFlags)` | 对指定帧发放带代号票据，区分保护、普通、主动跳过或无活动流。 / Decide protected, ordinary, policy skip or inactive for the supplied frame and return its epoch-bound ticket. | [sensorarrayBleNotifyFrameForGeneration](sensorarrayBle.c#L2715) |
| `uint8_t sensorarrayBleOutputScheduleNoteCompletion( sensorarrayBleOutputSchedule_t *s, uint64_t nowUs, const sensorarrayBleOutputTicket_t *completed, const sensorarrayBleOutputTicket_t *pendingProtection, bool wholeFrameAccepted)` | 核验整帧本地成功及票据身份，返回可解除的保护标志位。 / Verify whole local completion against the matching ticket and return protection flags eligible to clear. | [sensorarrayBleSendDescriptor](sensorarrayBle.c#L1725) |
| `void sensorarrayBleOutputScheduleCopySnapshot( const sensorarrayBleOutputSchedule_t *s, sensorarrayBleOutputScheduleSnapshot_t *snapshot)` | 将状态机当前的观测值复制到调用者结构。 / Copy the scheduler's current diagnostic snapshot. | 公开入口 / Callable API |

### 内部 / Internal

| C 签名 / C signature | 行为与参数 / Behaviour and arguments | 调用举例 / Representative caller |
|---|---|---|
| `static bool epochEqual(const sensorarrayBleOutputEpoch_t *a, const sensorarrayBleOutputEpoch_t *b)` | 同时比较 session、boot、连接代与流代。 / Compare session, boot, connection generation and stream generation. | [sensorarrayBleOutputScheduleObserve](sensorarrayBleOutputSchedule.c#L81) |
| `static void advanceGeneration(sensorarrayBleOutputSchedule_t *s)` | 推进调度策略代，回绕跳过零。 / Advance policy generation while skipping zero. | [observeTime](sensorarrayBleOutputSchedule.c#L48) |
| `static void updateMode(sensorarrayBleOutputSchedule_t *s)` | 根据活动状态、普通帧比例与容量拒绝率计算调度状态。 / Derive the scheduler mode from activity, ordinary rate and capacity rejection. | [resetEvidence](sensorarrayBleOutputSchedule.c#L27) |
| `static void resetEvidence(sensorarrayBleOutputSchedule_t *s)` | 清窗口基线、压力及恢复证据，并更新调度状态。 / Clear window baseline, pressure and recovery evidence and refresh the mode. | [observeTime](sensorarrayBleOutputSchedule.c#L48) |
| `static void pauseRecovery(sensorarrayBleOutputSchedule_t *s, uint64_t nowUs)` | 无有效观测时暂停恢复累积，超过有效期后清掉干净窗口计时。 / Pause recovery without evidence and clear clean-time credit when its evidence expires. | [observeTime](sensorarrayBleOutputSchedule.c#L48) |
| `static bool observeTime(sensorarrayBleOutputSchedule_t *s, uint64_t nowUs)` | 检测时钟回退，必要时重置代、证据和普通帧选择积分。 / Detect backward time and reset generation, evidence and ordinary credit when needed. | [sensorarrayBleOutputScheduleObserve](sensorarrayBleOutputSchedule.c#L81) |
| `static void setBaseline(sensorarrayBleOutputSchedule_t *s, uint64_t nowUs, const sensorarrayBleOutputCounters_t *counters)` | 保存计数器与窗口起始时间。 / Store counters and time as the next observation-window baseline. | [sensorarrayBleOutputScheduleObserve](sensorarrayBleOutputSchedule.c#L81) |

## [sensorarrayBleWatchdog.c](sensorarrayBleWatchdog.c)

### 组件内部接口 / Component interface

| C 签名 / C signature | 行为与参数 / Behaviour and arguments | 调用举例 / Representative caller |
|---|---|---|
| `esp_err_t sensorarrayBleWatchdogInit(void)` | 启动一次发送守护定时器并保存初始化返回值。 / Start the periodic send guard once and retain its initialisation result. | [sensorarrayBleInit](sensorarrayBle.c#L2482) |

### 内部 / Internal

| C 签名 / C signature | 行为与参数 / Behaviour and arguments | 调用举例 / Representative caller |
|---|---|---|
| `static void sensorarrayBleWatchdogTick(void *context)` | 周期定时器回调观察发送截止时间。 / Observe the current send deadline from the periodic timer callback. | [sensorarrayBleWatchdogInit](sensorarrayBleWatchdog.c#L19) |

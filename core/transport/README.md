# 传输调度与控制接收 / Transport scheduling and control ingress

有界 payload 池持有不可变帧字节与采集身份，队列只传 slot/generation 描述符。发送任务检查代数再读槽。控制回复和生命周期消息有独立的预留/排空路径。BLE 模式、Wi-Fi 就绪和 USB 接口类型决定各 sink 的投递，不应由发送任务给旧帧重盖当前配置身份。

A bounded pool holds encoded frame bytes and capture identity. Queues carry slot/generation descriptors and the sender checks the generation before reading. Reserved slots and draining give control replies and lifecycle events independent admission. BLE mode, Wi-Fi readiness and USB interface class determine sink delivery. Queued frames retain their capture identity.

## 调用方式 / Calling the module

```c
esp_err_t err = sensorarrayTransportPublishFrameMetadata(packet->data,
    packet->length, &metadata, protectionFlags);
```

metadata 来自构造 packet 的同一 frame。ESP_OK 报本地接受或相应接口定义的空操作。驱动接收字节、数据策略跳过、队列拒绝、BLE 接纳及远端完整实收是不同指标。串口适配器对 DATA/DEBUG、读取与有界写入分别记账。Wi-Fi 控制以请求 ID 查询最终状态。关键帧重试有期限，上传策略与硬件采样节奏相互独立。

Build metadata from the same frame that produced packet. ESP_OK denotes local acceptance or the documented no-op for that entry. Driver-accepted bytes, policy skips, queue rejection, BLE admission and complete host frames are distinct metrics. Serial I/O accounts for DATA/DEBUG, reads and bounded writes separately. Wi-Fi requests have queryable terminal IDs. Protected-frame retries have a deadline and output policy is independent of acquisition cadence.

## 源码和头文件 / Source and headers

| 文件 / File | 职责与声明 / Responsibility and declarations |
| --- | --- |
| [sensorarrayCriticalText.c](sensorarrayCriticalText.c) | 为运行故障构造有界的高优先级消息并投递到本地关键文本路径。 / Formats a bounded priority runtime-fault message for local critical-text admission. |
| [sensorarraySerialIo.c](sensorarraySerialIo.c) · [sensorarraySerialIo.h](sensorarraySerialIo.h) | 实现 Serial/OTG 数据与调试端点的读、写、优先槽、部分写和驱动接收统计。 / Implements Serial/OTG data and debug endpoint reads, writes, priority slots, partial-write handling and driver acceptance counters. `sensorarraySerialIoStats_t` |
| [sensorarrayTransport.c](sensorarrayTransport.c) · [sensorarrayTransport.h](sensorarrayTransport.h) | 建立有界队列和任务，用不可变描述符分发完整帧，保护终态和关键快照并过期超时重试。 / Creates bounded queues and tasks, dispatches complete frames by immutable descriptor and expires retries while protecting terminals and key snapshots. `sensorarrayTransportReplyKind_t` `sensorarrayTransportTxMode_t` |
| [sensorarrayTransportChannels.h](sensorarrayTransportChannels.h) | 定义控制、数据、日志与生命周期通道。 / Defines transport channels including data, log and lifecycle. `sensorarrayTransportChannel_t` |
| [sensorarrayTransportCommand.c](sensorarrayTransportCommand.c) | 规范化和解析来自 Serial、BLE、Wi-Fi 的控制文本，将回复定向到原目标并调用运行时命令处理器。 / Normalises Serial, BLE and Wi-Fi commands, targets replies to the originating peer and invokes runtime command handlers. |
| [sensorarrayTransportFrameIdentity.h](sensorarrayTransportFrameIdentity.h) | 比较同一配置与会话内帧的新旧，供丢弃过时和合并保护帧使用。 / Compares frame configuration and ordering within a session for obsolescence and protected-frame supersession. `sensorarrayTransportFrameIdentity_t` `sensorarrayTransportFrameMetadata_t` |
| [sensorarrayTransportFrameMeta.c](sensorarrayTransportFrameMeta.c) · [sensorarrayTransportFrameMeta.h](sensorarrayTransportFrameMeta.h) | 校验编码源附带的 metadata 或解析兼容文本头，按完整帧标记和身份生成 BLE 预览。 / Validates encoder-supplied metadata or parses legacy text headers and derives BLE previews with the original tag and identity. |
| [sensorarrayTransportGuaranteedText.c](sensorarrayTransportGuaranteedText.c) · [sensorarrayTransportGuaranteedText.h](sensorarrayTransportGuaranteedText.h) | 以预留、发布、领取、释放或撤销的生命周期保存控制回复和关键事件的本地槽。 / Holds local control/critical text slots through reservation, publication, drain, release or abort. |
| [sensorarrayTransportInternal.h](sensorarrayTransportInternal.h) | 内部共享描述符、控制回调及传输状态。 / Internal descriptor, control-callback and transport-state declarations. |
| [sensorarrayTransportPolicy.c](sensorarrayTransportPolicy.c) · [sensorarrayTransportPolicy.h](sensorarrayTransportPolicy.h) | 按连接、通道可发状态、类别与发布策略判断 BLE 接纳。 / Determines BLE admission from connection, channel sendability, message class and publication policy. |
| [sensorarrayTransportPool.c](sensorarrayTransportPool.c) · [sensorarrayTransportPool.h](sensorarrayTransportPool.h) | 为 payload 分配可复用的固定槽，使用 slot/generation 验证描述符，拒绝旧代释放。 / Allocates reusable fixed payload slots, validates descriptors by slot/generation and rejects stale releases. `sensorarrayTransportPayloadSlot_t` `sensorarrayTransportDescriptor_t` |
| [sensorarrayTransportStats.c](sensorarrayTransportStats.c) | 收集 sink/队列计数并保存发送模式、流配置与回调，提供只读查询。 / Collects sink and queue counters, stores transmit modes, stream selection and callbacks, and exposes queries. |
| [sensorarrayTransportWifiControl.c](sensorarrayTransportWifiControl.c) · [sensorarrayTransportWifiControl.h](sensorarrayTransportWifiControl.h) | 解析显式 Wi-Fi 开关与查询，按 session 和 request ID 格式化异步状态。 / Dispatches explicit Wi-Fi mode requests and queries and formats asynchronous state keyed by session and request ID. |

## 函数 / Functions

签名按源文件排列，参数顺序即调用顺序。下表标出生产代码中的调用者或回调注册者（最多两个）。`static` 限于所在编译单元。错误码列于对应行为后，只列函数直接返回的常量；硬件与下层调用可传播其他错误。

Signatures are grouped by source in call-argument order. The last column names up to two production callers or callback registrars. `static` entries are local to their translation unit. Constants after a description are explicit error returns. Underlying operations may propagate additional errors.

[sensorarrayCriticalText.c](#sensorarraycriticaltextc) · [sensorarraySerialIo.c](#sensorarrayserialioc) · [sensorarrayTransport.c](#sensorarraytransportc) · [sensorarrayTransportCommand.c](#sensorarraytransportcommandc) · [sensorarrayTransportFrameIdentity.h](#sensorarraytransportframeidentityh) · [sensorarrayTransportFrameMeta.c](#sensorarraytransportframemetac) · [sensorarrayTransportGuaranteedText.c](#sensorarraytransportguaranteedtextc) · [sensorarrayTransportPolicy.c](#sensorarraytransportpolicyc) · [sensorarrayTransportPool.c](#sensorarraytransportpoolc) · [sensorarrayTransportStats.c](#sensorarraytransportstatsc) · [sensorarrayTransportWifiControl.c](#sensorarraytransportwificontrolc)

### sensorarrayCriticalText.c

| C 调用签名 / C signature | 行为及返回 / Behaviour and return | 调用位置 / Called by |
| --- | --- | --- |
| `esp_err_t sensorarrayTransportPublishRuntimeFault(const char *format, ...)` [9](sensorarrayCriticalText.c#L9) | 把格式化运行故障投递到有界关键文本路径，返回本地接纳错误。 / Submit a formatted runtime fault to the bounded critical-text path and return local admission status. `ESP_ERR_INVALID_ARG` `ESP_ERR_INVALID_SIZE` | `sensorarrayRunAdsTimingCommand` / `sensorarrayEnterFrontendHold` |

### sensorarraySerialIo.c

| C 调用签名 / C signature | 行为及返回 / Behaviour and return | 调用位置 / Called by |
| --- | --- | --- |
| `static char *sensorarraySerialBytes(unsigned index)` [92](sensorarraySerialIo.c#L92) | 按槽索引定位该记录的有效负载字节区。 / Resolve a slot index to its payload byte storage. | `sensorarraySerialQueueRecord` / `sensorarraySerialWrite` |
| `static sensorarraySerialSlot_t *sensorarraySerialSlot(unsigned index)` [98](sensorarraySerialIo.c#L98) | 按索引定位对应的槽元数据结构。 / Resolve an index to its slot metadata. | `sensorarraySerialPublishSlot` / `sensorarraySerialLegacyTxTask` |
| `static uint8_t sensorarraySerialClaimSlot(unsigned lane)` [134](sensorarraySerialIo.c#L134) | 为指定优先级/数据通道领取可用槽，避免覆盖正在发送的记录。 / Claim an available slot for a lane without overwriting in-flight records. | `sensorarraySerialQueueRecord` / `sensorarraySerialWrite` |
| `static void sensorarraySerialReleaseSlot(uint8_t index)` [149](sensorarraySerialIo.c#L149) | 归还已消费或被拒绝记录的槽。 / Return storage for a consumed or rejected record. | `sensorarraySerialPublishSlot` / `sensorarraySerialClose` |
| `static esp_err_t sensorarraySerialPublishSlot(uint8_t index, size_t length, unsigned lane)` [156](sensorarraySerialIo.c#L156) | 提交已有槽及长度到对应有界队列，记录排队或拒绝。 / Submit the filled slot and length to its bounded lane and account for admission or rejection. | `sensorarraySerialQueueRecord` / `sensorarraySerialWrite` |
| `static esp_err_t sensorarraySerialQueueRecord(const char *data, size_t length, unsigned lane)` [189](sensorarraySerialIo.c#L189) | 验证长度、按通道预留槽、拷贝记录并提交。 / Validate length, reserve a lane slot, copy one record and submit it. `ESP_ERR_INVALID_SIZE` `ESP_ERR_INVALID_STATE` | `sensorarraySerialIoQueue` / `sensorarraySerialIoQueueData` |
| `esp_err_t sensorarraySerialIoQueue(const char *data, size_t length, bool priority)` [208](sensorarraySerialIo.c#L208) | 按 priority 选择关键或普通日志通道，提交一条记录。 / Choose the priority or ordinary log lane and submit one record. | `sensorarrayTransportPublishRuntimeFault` / `sensorarrayTransportDrainGuaranteedText` |
| `esp_err_t sensorarraySerialIoQueueData(const char *data, size_t length)` [213](sensorarraySerialIo.c#L213) | 向专用数据通道提交一个完整科学数据包。 / Submit one complete science packet to the dedicated data lane. | `sensorarrayFrameOutputPrint` / `sensorarrayUsbSinkPublishClass` |
| `static void sensorarraySerialLineDrop(size_t bytes, bool oversize, bool sinkCounted, bool capacityRejected)` [218](sensorarraySerialIo.c#L218) | 区分超长、容量耗尽等原因，累计本地被丢字节与行数。 / Account for dropped bytes and lines by cause, including oversize and capacity rejection. | `sensorarraySerialClose` / `sensorarraySerialWrite` |
| `static int sensorarraySerialOpen(const char *path, int flags, int mode)` [229](sensorarraySerialIo.c#L229) | 将 VFS 路径 /out 和 /err 映射到独立行装配器，其他路径返回 ENOENT。 / Map /out and /err to distinct VFS line assemblers and return ENOENT for other paths. | `sensorarraySerialIoInit` |
| `static int sensorarraySerialClose(int fd)` [237](sensorarraySerialIo.c#L237) | 关闭一个 VFS 行装配器，释放未完成行的槽并登记丢弃。 / Close one VFS line assembler, releasing and accounting for an incomplete line. | `sensorarraySerialIoInit` |
| `static int sensorarraySerialFstat(int fd, struct stat *st)` [248](sensorarraySerialIo.c#L248) | 报告此 VFS 的字符设备类型和设备标识。 / Report the VFS character-device type and device identifier. | `sensorarraySerialIoInit` |
| `static bool sensorarraySerialStreamBound(FILE *stream)` [257](sensorarraySerialIo.c#L257) | 用 fstat 检查 FILE 是否已经绑定到该输出 VFS。 / Use fstat to check that a FILE belongs to this output VFS. | `sensorarraySerialIoInit` |
| `static esp_err_t sensorarraySerialInitResult(esp_err_t err)` [264](sensorarraySerialIo.c#L264) | 在统计锁内保存初始化错误并原样返回。 / Record initialisation status under the stats lock and return it unchanged. | `sensorarraySerialIoInit` |
| `static ssize_t sensorarraySerialWrite(int fd, const void *data, size_t size)` [271](sensorarraySerialIo.c#L271) | 将 stdout/stderr 的字节装配为有界日志行，按换行投递并统计超长丢弃。 / Assemble stdout/stderr bytes into bounded log lines, enqueue on newline and count oversize drops. | `sensorarraySerialIoInit` |
| `static void sensorarraySerialTxCheckpoint(void)` [322](sensorarraySerialIo.c#L322) | 执行 CPU0 发送工作预算检查并统计主动让出次数。 / Check the CPU0 transmit budget and count cooperative yields. | `sensorarraySerialLegacyTxTask` / `sensorarraySerialOtgTxTask` |
| `static void sensorarraySerialLegacyTxTask(void *arg)` [333](sensorarraySerialIo.c#L333) | 通过 Serial/JTAG 驱动发送有界记录，分别统计部分写、过期和已接纳字节。 / Drain bounded records through Serial/JTAG and account for partial writes, expiry and accepted bytes. | `sensorarraySerialTxTask` |
| `static void sensorarraySerialOtgTxTask(void *arg)` [482](sensorarraySerialIo.c#L482) | 将数据与调试记录投到各自 CDC 接口并在期限内处理剩余字节。 / Drain data and debug records to their CDC interfaces and handle remaining bytes within their lifetime. | `sensorarraySerialTxTask` |
| `static void sensorarraySerialTxTask(void *arg)` [601](sensorarraySerialIo.c#L601) | 根据所选 USB 后端进入唯一相应的发送循环。 / Enter the single transmit loop for the selected USB backend. | `sensorarraySerialIoInit` |
| `esp_err_t sensorarraySerialIoInit(void)` [607](sensorarraySerialIo.c#L607) | 安装所选唯一 USB 后端及发送任务，绑定输出 VFS 并保留初始化结果。 / Establish the selected USB backend and transmitter task, bind the output VFS and retain initialisation status. | `sensorarrayTransportInitOwned` |
| `int sensorarraySerialIoReadWithEpoch(void *buffer, size_t length, uint32_t *epoch)` [722](sensorarraySerialIo.c#L722) | 从当前控制接口读取字节并返回读入时的连接世代供命令装配使用。 / Read control bytes with their connection epoch so the parser can discard partial commands on reconnect. | `sensorarraySerialIoRead` / `sensorarrayTransportSerialControlTask` |
| `int sensorarraySerialIoRead(void *buffer, size_t length)` [742](sensorarraySerialIo.c#L742) | 从所选 USB 控制入口读取限定数量的字节。 / Read a bounded byte count from the selected USB control endpoint. | 独立接口 / Standalone entry |
| `void sensorarraySerialIoNoteOversize(void)` [745](sensorarraySerialIo.c#L745) | 累计无法容纳的输出记录。 / Count a record that exceeds output capacity. | `sensorarrayTransportSerialControlTask` |
| `void sensorarraySerialIoGetStats(sensorarraySerialIoStats_t *outStats)` [752](sensorarraySerialIo.c#L752) | 复制本地排队、驱动接收、部分写、超时和 RX/TX 时间统计。 / Copy local queue, driver acceptance, partial-write, timeout and RX/TX timestamp counters. | `sensorarrayUsbControlHandle` / `sensorarrayFormatSerialReply` |

### sensorarrayTransport.c

| C 调用签名 / C signature | 行为及返回 / Behaviour and return | 调用位置 / Called by |
| --- | --- | --- |
| `static void sensorarrayTransportLogSerialCommandError(const uint8_t *data, size_t length, const char *reason)` [137](sensorarrayTransport.c#L137) | 输出 串行命令错误。 / Report serial command error. | `sensorarrayTransportSerialControlTask` |
| `static esp_err_t sensorarrayTransportNormaliseSerialCommand(const char *input, size_t inputLength, char *out, size_t outSize, const char **outReason)` [178](sensorarrayTransport.c#L178) | 规范化 串行命令。 / Normalise serial command. `ESP_ERR_INVALID_ARG` `ESP_ERR_INVALID_RESPONSE` `ESP_ERR_INVALID_SIZE` `ESP_ERR_NOT_FOUND` | `sensorarrayTransportSerialControlTask` |
| `static bool sensorarrayTransportFindFieldU32(const char *line, const char *key, uint32_t *outValue)` [250](sensorarrayTransport.c#L250) | 按字段名从兼容文本记录中提取有限的无符号值。 / Extract a bounded unsigned value from a named legacy text field. | `sensorarrayTransportQueueFrameInternal` |
| `static void sensorarrayTransportSyncPoolStatsLocked(void)` [274](sensorarrayTransport.c#L274) | 从固定槽池同步占用、高水位和错误计数，调用时须持有传输锁。 / Synchronise occupancy, watermark and failure counters from the pool under the transport lock. | `sensorarrayTransportAllocateDescriptor` / `sensorarrayTransportDescriptorCurrent` |
| `static bool sensorarrayTransportAllocateDescriptor(sensorarrayTransportChannel_t channel, size_t length, sensorarrayTransportDescriptor_t *outDescriptor)` [285](sensorarrayTransport.c#L285) | 在普通或保护槽池中建立带当前 generation 的描述符。 / Allocate a normal or protected payload descriptor with the slot's current generation. | `sensorarrayTransportQueueFrameInternal` |
| `static bool sensorarrayTransportDescriptorCurrent(const sensorarrayTransportDescriptor_t *descriptor)` [300](sensorarrayTransport.c#L300) | 用 slot 与 generation 检查描述符是否仍指向同一份载荷。 / Check the slot and generation against the payload descriptor. | `sensorarrayTransportExpireOwnedProtected` / `sensorarrayTransportProcessDescriptor` |
| `static sensorarrayTransportPayloadSlot_t *sensorarrayTransportDescriptorSlot(const sensorarrayTransportDescriptor_t *descriptor)` [310](sensorarrayTransport.c#L310) | 检查描述符并返回对应槽，旧代或越界返回空。 / Resolve a valid descriptor to its slot or return NULL for stale or out-of-range descriptors. | `sensorarrayTransportHoldProtectedRetry` / `sensorarrayTransportWaitTicks` |
| `static bool sensorarrayTransportReleaseDescriptor(const sensorarrayTransportDescriptor_t *descriptor)` [316](sensorarrayTransport.c#L316) | 按 slot/generation 交还已处理的描述符并更新池统计。 / Release the processed slot/generation descriptor and update pool counters. | `sensorarrayTransportHoldProtectedRetry` / `sensorarrayTransportExpireOwnedProtected` |
| `static void sensorarrayTransportFillFrameMeta(sensorarrayTransportPayloadSlot_t *item)` [326](sensorarrayTransport.c#L326) | 从该槽已有的编码文本首行提取兼容帧元数据。 / Extract legacy frame metadata from the existing encoded text header in this slot. | `sensorarrayTransportQueueFrameInternal` |
| `static void sensorarrayTransportProtectionCount(uint32_t *counter)` [331](sensorarrayTransport.c#L331) | 在保护计数器临界区增加指定计数。 / Increment the indicated protected-frame counter under its guard. | `sensorarrayTransportHoldProtectedRetry` / `sensorarrayTransportProcessDescriptor` |
| `void sensorarrayTransportGetProtectionStats(sensorarrayTransportProtectionStats_t *outStats)` [338](sensorarrayTransport.c#L338) | 查找 保护统计。 / Look up protection stats. | `sensorarraySnapshotDiagnosticsFormat` |
| `static bool sensorarrayTransportHoldProtectedRetry(const sensorarrayTransportDescriptor_t *descriptor, sensorarrayTransportPayloadSlot_t *slot, uint64_t nowUs)` [346](sensorarrayTransport.c#L346) | 保留仍可重试的保护帧并建立下一尝试时间，记录取代与重试。 / Retain an eligible protected frame, schedule its next attempt and account for supersession/retries. | `sensorarrayTransportProcessDescriptor` |
| `static TickType_t sensorarrayTransportWaitTicks(void)` [370](sensorarrayTransport.c#L370) | 把 Wi-Fi 服务与保护帧下个重试/到期点换算为任务等待上限。 / Derive the task wait limit from Wi-Fi service and protected-frame retry/expiry deadlines. | `sensorarrayTransportTask` |
| `static void sensorarrayTransportBuildNames(const uint8_t mac[6])` [386](sensorarrayTransport.c#L386) | 由设备地址建立网络可识别的设备名称。 / Build the advertised device names from the device address. | `sensorarrayTransportInitOwned` |
| `static void sensorarrayTransportBuildWifiConfig(sensorarrayWifiConfig_t *wifiConfig)` [394](sensorarrayTransport.c#L394) | 以板级选项和设备名称构建 Wi-Fi 后端配置。 / Build Wi-Fi backend settings from the configured board options and device name. | `sensorarrayTransportStartWifiAp` |
| `static esp_err_t sensorarrayTransportStartWifiAp(uint32_t *requestId)` [410](sensorarrayTransport.c#L410) | 按生成的 AP 配置启动 Wi-Fi 适配层并返回后端结果。 / Start the Wi-Fi adapter with its generated AP configuration and return backend status. | `sensorarrayTransportReleaseWifiStartupGate` / `sensorarrayTransportRequestWifiMode` |
| `void sensorarrayTransportConfigureWifiBoot(uint64_t sessionId, uint32_t bootId)` [417](sensorarrayTransport.c#L417) | 登记 Wi-Fi 所有者的会话/启动身份并建立启动准入条件。 / Register session and boot identity for the Wi-Fi owner and establish its startup admission gate. | `app_main` |
| `void sensorarrayTransportReleaseWifiStartupGate(void)` [422](sensorarrayTransport.c#L422) | 固定任务资源预留完成后允许懒启动的 Wi-Fi 后端运行。 / Open lazy Wi-Fi startup after fixed task resources have been reserved. | `sensorarrayWifiStartupReservationComplete` |
| `static void sensorarrayTransportStatResult(sensorarrayTransportChannel_t channel, bool ble, esp_err_t err)` [436](sensorarrayTransport.c#L436) | 按接纳或失败将一次本机投递计入对应计数器。 / Account for a local delivery attempt as admitted or failed in the supplied counters. | `sensorarrayTransportWifiCompleted` / `sensorarrayTransportProcessDescriptor` |
| `static void sensorarrayTransportWifiCompleted(const sensorarrayWifiTxResult_t *result, void *context)` [466](sensorarrayTransport.c#L466) | 接收 Wi-Fi 后端报告的发送结果并更新对应通道统计。 / Account for Wi-Fi backend send completion on the corresponding channel. | `sensorarrayTransportInitOwned` |
| `static bool sensorarrayTransportExpireOwnedProtected(const sensorarrayTransportDescriptor_t *descriptor)` [487](sensorarrayTransport.c#L487) | 检查归当前任务所有的保护槽是否已到期并累计尚未完成的目标。 / Check an owned protected slot for expiry and count targets still outstanding. | `sensorarrayTransportProcessDescriptor` / `sensorarrayTransportExpireProtectedRetry` |
| `static void sensorarrayTransportBleControl(const uint8_t *data, size_t length, void *context)` [509](sensorarrayTransport.c#L509) | 把 BLE 收到的控制字节连同 BLE 回复目标交给共享命令处理。 / Dispatch incoming BLE control bytes with a BLE reply target. | `sensorarrayTransportInitOwned` |
| `static void sensorarrayTransportWifiControl(const uint8_t *data, size_t length, const sensorarrayWifiPeer_t *peer, void *context)` [519](sensorarrayTransport.c#L519) | 将 Wi-Fi 对端及控制载荷一起交给共享解析器。 / Dispatch control bytes and their originating Wi-Fi peer to the shared parser. | `sensorarrayTransportInitOwned` |
| `static void sensorarrayTransportSerialControlTask(void *arg)` [535](sensorarrayTransport.c#L535) | 从所配置的 USB/Serial 控制入口读取并按完整命令分派，独立于数据流选择。 / Read the configured USB/Serial control endpoint and dispatch complete commands independently of data stream selection. | `sensorarrayTransportInitOwned` |
| `static void sensorarrayTransportPrintNetInit(esp_err_t wifiErr)` [592](sensorarrayTransport.c#L592) | 输出 网络初始化。 / Print net init. | `sensorarrayTransportTask` |
| `static void sensorarrayTransportProcessDescriptor(const sensorarrayTransportDescriptor_t *descriptor)` [610](sensorarrayTransport.c#L610) | 确认描述符代数、读取原始元数据，按 sink 状态处理本地投递与有界保护重试。 / Validate descriptor generation, inspect capture metadata and perform sink admission and bounded protected retry. | `sensorarrayTransportTask` |
| `static void sensorarrayTransportNotifyTask(void)` [728](sensorarrayTransport.c#L728) | 唤醒传输任务以检查新增控制、数据或待重试工作。 / Wake the transport task to examine new control, data or retry work. | `sensorarrayTransportTask` / `sensorarrayTransportInitOwned` |
| `static void sensorarrayTransportDrainGuaranteedText(void)` [735](sensorarrayTransport.c#L735) | 领取已发布的关键文本，按保留目标分派并释放本地槽。 / Drain published priority text to its retained destinations and release the local slot. | `sensorarrayTransportTask` |
| `static void sensorarrayTransportExpireProtectedRetry(void)` [772](sensorarrayTransport.c#L772) | 检查已持有的保护重试是否过期并收回本地所有权。 / Check the held retry deadline and reclaim local ownership on expiration. | `sensorarrayTransportReceiveNext` |
| `static void sensorarrayTransportExpireProtectedPending(void)` [787](sensorarrayTransport.c#L787) | 清理已超过重试期限的待发送保护帧。 / Expire pending protected frames that outlive their retry window. | `sensorarrayTransportReceiveNext` |
| `static bool sensorarrayTransportReceiveNext(sensorarrayTransportDescriptor_t *outDescriptor, uint32_t *dataBudget)` [820](sensorarrayTransport.c#L820) | 从可投递的控制、保护及普通数据工作中领取下一个描述符。 / Select the next eligible descriptor from pending protected and ordinary work. | `sensorarrayTransportTask` |
| `static void sensorarrayTransportTask(void *arg)` [863](sensorarrayTransport.c#L863) | 优先排空控制/终态文本，处理描述符并按期限继续或撤销保护重试。 / Drain priority/control text, dispatch descriptors and retry or expire protected work by deadline. | `sensorarrayTransportInitOwned` |
| `static bool sensorarrayTransportSupersedePendingProtected(const sensorarrayTransportFrameIdentity_t *identity)` [892](sensorarrayTransport.c#L892) | 按原身份与序号判定新保护帧能否取代等待中的旧帧。 / Use capture identity and sequence to decide whether a pending protected frame can be superseded. | `sensorarrayTransportQueueFrameInternal` |
| `static esp_err_t sensorarrayTransportQueueFrameInternal(sensorarrayTransportChannel_t channel, const char *data, size_t length, bool *outQueued, const sensorarrayTransportFrameIdentity_t *identity, uint32_t protectionFlags, const sensorarrayTransportFrameMetadata_t *metadata)` [909](sensorarrayTransport.c#L909) | 分配与帧身份匹配的槽，复制 payload 与元数据并处理保护/普通队列接纳。 / Allocate a slot, copy payload and frame metadata, and handle protected/ordinary queue admission. `ESP_ERR_INVALID_ARG` `ESP_ERR_INVALID_STATE` `ESP_ERR_TIMEOUT` | `sensorarrayTransportQueue` / `sensorarrayTransportPublishFrame` |
| `static esp_err_t sensorarrayTransportQueue(sensorarrayTransportChannel_t channel, const char *data, size_t length, bool *outQueued)` [1096](sensorarrayTransport.c#L1096) | 把指定类别、目标和长度的本地文本交给内部有界排队路径。 / Pass the specified channel, target and payload length to bounded queue admission. | `sensorarrayTransportPublishData` / `sensorarrayTransportPublishLog` |
| `static esp_err_t sensorarrayTransportEnsurePayloadStorage(void)` [1102](sensorarrayTransport.c#L1102) | 为固定 payload 池建立足够的长期存储并报告所在堆。 / Establish long-lived storage for fixed payload slots and record its heap placement. | `sensorarrayTransportInitOwned` |
| `static esp_err_t sensorarrayTransportInitOwned(void)` [1127](sensorarrayTransport.c#L1127) | 在传输初始化所有者上下文建立各 sink、存储和任务。 / Establish sink backends, storage and tasks under the transport initialisation owner. `ESP_ERR_NO_MEM` | `sensorarrayTransportInit` |
| `esp_err_t sensorarrayTransportInit(void)` [1250](sensorarrayTransport.c#L1250) | 建立传输任务、有限队列与控制接收入口，并初始化已配置后端。 / Create transport workers, bounded queues and control ingress and initialise configured backends. `ESP_ERR_INVALID_STATE` | `sensorarrayNetStatusInit` |
| `void sensorarrayTransportGetMemoryDiagnostics(sensorarrayTransportMemoryDiagnostics_t *outDiagnostics)` [1266](sensorarrayTransport.c#L1266) | 复制池大小、描述符代价和内部/外部堆占用的结果。 / Copy pool sizes, descriptor costs and internal/external heap accounting. | `sensorarrayMemoryDiagnosticsFormat` / `sensorarrayAsyncLogInit` |
| `static uint32_t sensorarrayTransportStackMinimumRemainingBytes(TaskHandle_t task)` [1294](sensorarrayTransport.c#L1294) | 把 FreeRTOS 栈水位换算为可报告的最小剩余字节数。 / Return the minimum remaining task stack as reportable bytes. | `sensorarrayTransportGetTaskStackStats` |
| `void sensorarrayTransportGetTaskStackStats(sensorarrayTransportTaskStackStats_t *outStats)` [1303](sensorarrayTransport.c#L1303) | 查询传输任务和 Serial 控制任务的栈配置与剩余水位。 / Report configured stacks and remaining low-water marks for transport and serial control tasks. | `sensorarrayAsyncLogPrintCompactSummary` |
| `esp_err_t sensorarrayTransportRequestWifiMode(sensorarrayTransportWifiMode_t mode, uint32_t *requestId)` [1319](sensorarrayTransport.c#L1319) | 申请 Wi-Fi 后端模式变更并返回可查询的请求编号。 / Request a Wi-Fi backend mode change and return its queryable request ID. | `sensorarrayTransportApplyWifiMode` / `sensorarrayTransportWifiCommand` |
| `esp_err_t sensorarrayTransportApplyWifiMode(sensorarrayTransportWifiMode_t mode)` [1327](sensorarrayTransport.c#L1327) | 提交 Wi-Fi 模式切换并按后端能力报告提交错误。 / Submit a Wi-Fi mode change and return backend admission errors. | 独立接口 / Standalone entry |
| `esp_err_t sensorarrayTransportPublishData(const char *data, size_t length)` [1332](sensorarrayTransport.c#L1332) | 向数据通道提交长度明确的兼容文本载荷。 / Submit length-delimited legacy text to the data channel. | `sensorarrayNetTextPublish` |
| `esp_err_t sensorarrayTransportPublishFrame(const char *data, size_t length, const sensorarrayTransportFrameIdentity_t *identity, uint32_t protectionFlags)` [1337](sensorarrayTransport.c#L1337) | 提交带编码时身份的整帧，兼容入口从文本提取其他元数据。 / Submit a complete frame with encoder-time identity and parse remaining metadata from legacy text. `ESP_ERR_INVALID_ARG` | 独立接口 / Standalone entry |
| `esp_err_t sensorarrayTransportPublishFrameMetadata(const char *data, size_t length, const sensorarrayTransportFrameMetadata_t *metadata, uint32_t protectionFlags)` [1348](sensorarrayTransport.c#L1348) | 将真实编码器的字节和同帧的快照元数据拷入有限队列并保存保护位。 / Enqueue real encoder bytes with metadata from that same frame and retain protection flags. `ESP_ERR_INVALID_ARG` | `sensorarrayNetFramePublish` |
| `esp_err_t sensorarrayTransportPublishLog(const char *data, size_t length)` [1359](sensorarrayTransport.c#L1359) | 向日志通道提交长度明确的文本记录。 / Submit a length-delimited text record to the log channel. | `sensorarrayNetLogPublish` |
| `esp_err_t sensorarrayTransportPublishLifecycle(const char *data, size_t length)` [1364](sensorarrayTransport.c#L1364) | 把生命周期消息放入本地优先路径供已配置的 sink 排空。 / Admit a lifecycle message to the local priority path for enabled sinks. | `sensorarrayAsyncLogPrintEvent` |
| `esp_err_t sensorarrayTransportPublishLifecycleObserved(const char *data, size_t length, bool *outQueued)` [1374](sensorarrayTransport.c#L1374) | 本地入队生命周期文本，把是否实际提交网络描述符写入 outQueued。 / Admit lifecycle text locally and set outQueued when a network descriptor was actually submitted. `ESP_ERR_INVALID_ARG` | `sensorarrayTransportPublishRuntimeFault` / `sensorarrayTransportWifiDrainTerminal` |
| `esp_err_t sensorarrayTransportPublishControlReply(const sensorarrayTransportReplyTarget_t *target, const char *data, size_t length)` [1383](sensorarrayTransport.c#L1383) | 面向原命令来源提交控制回复并报告本地接纳错误。 / Submit a reply to the originating command target and report local admission failure. `ESP_ERR_INVALID_ARG` `ESP_ERR_INVALID_SIZE` `ESP_ERR_INVALID_STATE` | `sensorarrayTransportSerialControlTask` / `sensorarrayTransportHandleControlCommand` |

### sensorarrayTransportCommand.c

| C 调用签名 / C signature | 行为及返回 / Behaviour and return | 调用位置 / Called by |
| --- | --- | --- |
| `static bool sensorarrayTransportSerialUnknownShouldReply(const uint8_t *data, size_t length, const char *reason)` [17](sensorarrayTransportCommand.c#L17) | 判断未识别的 Serial 输入是否应产生未知命令回复。 / Decide whether unrecognised serial input merits an unknown-command reply. | `sensorarrayTransportHandleControlCommand` |
| `static void sensorarrayTransportCommandNormalize(char *text)` [59](sensorarrayTransportCommand.c#L59) | 清理空白和大小写，为各传输使用同一命令语法。 / Normalise whitespace and case for the shared command grammar. | `sensorarrayTransportCommandParse` |
| `static esp_err_t sensorarrayTransportCommandParse(const uint8_t *data, size_t length, char *text, size_t textSize)` [75](sensorarrayTransportCommand.c#L75) | 按公共语法识别输入命令并解析该命令所需的设置值。 / Recognise the command grammar and parse the settings required by the matched command. `ESP_ERR_INVALID_ARG` | `sensorarrayTransportHandleControlCommand` |
| `static esp_err_t sensorarrayTransportHandleRuntimeCommand(const char *text, char *response, size_t responseSize)` [90](sensorarrayTransportCommand.c#L90) | 把运行时命令交给已注册查询回调并处理本地回复。 / Dispatch a runtime command to the registered query handler and process the local reply. `ESP_ERR_INVALID_ARG` `ESP_ERR_NOT_SUPPORTED` | `sensorarrayTransportHandleControlCommand` |
| `esp_err_t sensorarrayTransportHandleControlCommand(const uint8_t *data, size_t length, const sensorarrayTransportReplyTarget_t *replyTarget)` [180](sensorarrayTransportCommand.c#L180) | 按显式长度解析 Serial/BLE/Wi-Fi 请求，向指定回复目标提交受理或错误文本。 / Parse a length-delimited Serial/BLE/Wi-Fi command and send local acceptance or failure to its reply target. `ESP_ERR_INVALID_ARG` `ESP_ERR_INVALID_SIZE` | `sensorarrayTransportBleControl` / `sensorarrayTransportWifiControl` |

### sensorarrayTransportFrameIdentity.h

| C 调用签名 / C signature | 行为及返回 / Behaviour and return | 调用位置 / Called by |
| --- | --- | --- |
| `static inline bool sensorarrayTransportFrameSameConfig(const sensorarrayTransportFrameIdentity_t *a, const sensorarrayTransportFrameIdentity_t *b)` [40](sensorarrayTransportFrameIdentity.h#L40) | 逐域检查 session、boot、revision、mode/rows/profile 的 ID、世代和活动行数。 / Compare session, boot, revision, domain IDs/generations and active rows. | `sensorarrayTransportHoldProtectedRetry` / `sensorarrayTransportSupersedePendingProtected` |
| `static inline bool sensorarrayTransportFrameOlderOrSame(const sensorarrayTransportFrameIdentity_t *candidate, const sensorarrayTransportFrameIdentity_t *last)` [52](sensorarrayTransportFrameIdentity.h#L52) | 在同一 session/boot 内使用 uint32 半区间顺序判断旧帧或重号。 / Use uint32 half-range ordering within the same session/boot to detect older or duplicate frames. | `sensorarrayTransportProcessDescriptor` |

### sensorarrayTransportFrameMeta.c

| C 调用签名 / C signature | 行为及返回 / Behaviour and return | 调用位置 / Called by |
| --- | --- | --- |
| `static bool field(const char *text, const char *key, int base, uint64_t maximum, uint64_t *value)` [7](sensorarrayTransportFrameMeta.c#L7) | 按键、基数和最大值检查文本字段，成功时写入整数结果。 / Parse a named numeric field in the specified base and reject values outside the supplied bound. | `sensorarrayTransportBuildPreview` |
| `void sensorarrayTransportParseFrameMeta(sensorarrayTransportPayloadSlot_t *slot)` [51](sensorarrayTransportFrameMeta.c#L51) | 读取首行的标签、域身份与掩码相关字段，供兼容文本传输定位帧。 / Parse the first-line tag, domain identity and frame fields for legacy transport metadata. | `sensorarrayTransportFillFrameMeta` |
| `bool sensorarrayTransportFrameMetadataValid(const sensorarrayTransportFrameMetadata_t *metadata, const char *data, size_t length)` [119](sensorarrayTransportFrameMeta.c#L119) | 检查显式元数据与编码帧的标签、长度及取值约束。 / Validate supplied metadata, encoded tag, size and field ranges. | `sensorarrayTransportPublishFrameMetadata` / `sensorarrayTransportApplyFrameMetadata` |
| `bool sensorarrayTransportApplyFrameMetadata(sensorarrayTransportPayloadSlot_t *slot, const sensorarrayTransportFrameMetadata_t *metadata)` [128](sensorarrayTransportFrameMeta.c#L128) | 核对传入的编码时元数据后填入槽，保留原帧和兼容标签的身份字段。 / Validate encoder-time metadata and fill the slot with original and tag-specific identities. | `sensorarrayTransportQueueFrameInternal` |
| `bool sensorarrayTransportBuildPreview(const sensorarrayTransportPayloadSlot_t *slot, char *out, size_t outSize)` [157](sensorarrayTransportFrameMeta.c#L157) | 从已知帧元数据构造有界文本预览并保留源帧身份。 / Build a bounded preview from frame metadata while retaining its original identity. | `sensorarrayTransportProcessDescriptor` |

### sensorarrayTransportGuaranteedText.c

| C 调用签名 / C signature | 行为及返回 / Behaviour and return | 调用位置 / Called by |
| --- | --- | --- |
| `static void sensorarrayTransportGuaranteedTextNotify(void)` [19](sensorarrayTransportGuaranteedText.c#L19) | 若注册了通知回调，提醒发送任务关键槽的状态已发生变化。 / Invoke the registered wake callback when the critical slot changes state. | `sensorarrayTransportGuaranteedTextRelease` / `sensorarrayTransportGuaranteedTextPublish` |
| `esp_err_t sensorarrayTransportGuaranteedTextReserve(void)` [26](sensorarrayTransportGuaranteedText.c#L26) | 原子预留唯一关键文本槽，已占用返回 ESP_ERR_NO_MEM。 / Atomically reserve the critical text slot or return ESP_ERR_NO_MEM when occupied. `ESP_ERR_NO_MEM` | `sensorarrayCommandMailboxPostReservedConfig` / `sensorarrayCommandMailboxPostRowModes` |
| `esp_err_t sensorarrayTransportGuaranteedTextCancel(void)` [40](sensorarrayTransportGuaranteedText.c#L40) | 取消尚未发布的预留，允许下一请求使用该槽。 / Cancel an unpublished reservation so the next request can use it. `ESP_ERR_INVALID_STATE` | `sensorarrayCommandMailboxPostReservedConfig` / `sensorarrayCommandMailboxPostRowModes` |
| `esp_err_t sensorarrayTransportGuaranteedTextRelease(void)` [52](sensorarrayTransportGuaranteedText.c#L52) | 打开已预留记录的排空门并唤醒发送方。 / Open the drain gate for the reserved record and wake the sender. `ESP_ERR_INVALID_STATE` | `sensorarrayCommandMailboxCommitRowModesAck` / `sensorarrayRecoveryControlAck` |
| `esp_err_t sensorarrayTransportGuaranteedTextAbort(void)` [65](sensorarrayTransportGuaranteedText.c#L65) | 清空当前预留及关联文本，供失败收尾。 / Clear the reservation and associated text during failure cleanup. `ESP_ERR_INVALID_STATE` | `sensorarrayCommandMailboxCancelRowModesAck` / `sensorarrayRecoveryControlPost` |
| `esp_err_t sensorarrayTransportGuaranteedTextPublish(const char *data, size_t length)` [77](sensorarrayTransportGuaranteedText.c#L77) | 把限定长度的关键文本拷贝入预留槽并标为 ready。 / Copy bounded critical text into the reserved slot and mark it ready. `ESP_ERR_INVALID_ARG` `ESP_ERR_INVALID_STATE` | `sensorarrayCommandMailboxEmitRowModesTerminal` / `sensorarrayRecoveryControlComplete` |
| `bool sensorarrayTransportGuaranteedTextTakeDrain(char *outText, size_t outSize, size_t *outLength)` [98](sensorarrayTransportGuaranteedText.c#L98) | 同时满足 ready 与 released 且缓冲足够时复制文本、长度并收回槽。 / Copy text and length and reclaim the slot only once ready, released and the destination fits. | `sensorarrayTransportDrainGuaranteedText` |
| `bool sensorarrayTransportGuaranteedTextIsReserved(void)` [118](sensorarrayTransportGuaranteedText.c#L118) | 在锁内查询关键文本槽是否已被预留。 / Read the critical slot's reservation flag under its lock. | 独立接口 / Standalone entry |
| `void sensorarrayTransportGuaranteedTextSetNotify(sensorarrayTransportGuaranteedTextNotify_t notify)` [126](sensorarrayTransportGuaranteedText.c#L126) | 登记关键槽可供排空时的唤醒回调。 / Register the wake callback for a drainable critical slot. | `sensorarrayTransportInitOwned` |

### sensorarrayTransportPolicy.c

| C 调用签名 / C signature | 行为及返回 / Behaviour and return | 调用位置 / Called by |
| --- | --- | --- |
| `bool sensorarrayTransportBlePolicyAllows(bool streamWantsBle, bool connected, bool dataChannelSendable, bool logChannelSendable, sensorarrayTransportChannel_t channel)` [3](sensorarrayTransportPolicy.c#L3) | 综合 stream、连接、通道可发、数据类别等标志决定是否准入 BLE。 / Gate BLE admission on stream selection, connection, channel sendability and message class. | `sensorarrayTransportBleChannelEnabled` |

### sensorarrayTransportPool.c

| C 调用签名 / C signature | 行为及返回 / Behaviour and return | 调用位置 / Called by |
| --- | --- | --- |
| `static uint32_t sensorarrayTransportPoolNextGeneration(uint32_t generation)` [3](sensorarrayTransportPool.c#L3) | 推进槽代数并保留零值作为无效代数。 / Advance slot generation, reserving zero as invalid. | `sensorarrayTransportPoolAllocate` / `sensorarrayTransportPoolAllocateProtected` |
| `bool sensorarrayTransportPoolInit(sensorarrayTransportPool_t *pool, char *storage, size_t storageBytes)` [9](sensorarrayTransportPool.c#L9) | 把后备字节区按固定步长关联至各载荷槽并建立初始代数。 / Bind backing storage to fixed-stride payload slots and initialise generation state. | `sensorarrayTransportEnsurePayloadStorage` |
| `static bool sensorarrayTransportPoolSlotReady(const sensorarrayTransportPool_t *pool, size_t index)` [30](sensorarrayTransportPool.c#L30) | 确认指定槽的索引、后备存储与容量可投入分配。 / Check that the slot has an admissible index and backing storage. | `sensorarrayTransportPoolAllocate` / `sensorarrayTransportPoolAllocateProtected` |
| `bool sensorarrayTransportPoolAllocate(sensorarrayTransportPool_t *pool, sensorarrayTransportChannel_t channel, size_t length, sensorarrayTransportDescriptor_t *outDescriptor)` [39](sensorarrayTransportPool.c#L39) | 为消息类别与长度领取普通槽并生成对应代数的描述符。 / Claim a normal slot for channel/length and issue its generation-tagged descriptor. | `sensorarrayTransportAllocateDescriptor` |
| `bool sensorarrayTransportPoolAllocateProtected(sensorarrayTransportPool_t *pool, size_t length, sensorarrayTransportDescriptor_t *outDescriptor)` [137](sensorarrayTransportPool.c#L137) | 在受保护槽范围内分配并返回 slot/generation 描述符。 / Allocate from the protected range and return its slot/generation descriptor. | `sensorarrayTransportQueueFrameInternal` |
| `bool sensorarrayTransportPoolValidate(sensorarrayTransportPool_t *pool, const sensorarrayTransportDescriptor_t *descriptor)` [172](sensorarrayTransportPool.c#L172) | 核对范围、占用和代数，拒绝已释放或属于其他帧的描述符。 / Check bounds, use state and generation to reject released or reassigned descriptors. | `sensorarrayTransportDescriptorCurrent` |
| `sensorarrayTransportPayloadSlot_t *sensorarrayTransportPoolGetSlot(sensorarrayTransportPool_t *pool, const sensorarrayTransportDescriptor_t *descriptor)` [200](sensorarrayTransportPool.c#L200) | 验证描述符后返回由该帧持有的槽地址。 / Validate the descriptor before returning its payload-slot address. | `sensorarrayTransportDescriptorSlot` |
| `bool sensorarrayTransportPoolRelease(sensorarrayTransportPool_t *pool, const sensorarrayTransportDescriptor_t *descriptor)` [211](sensorarrayTransportPool.c#L211) | 只归还匹配代数的已占用槽并累计不一致释放。 / Return only an occupied matching-generation slot and count mismatched releases. | `sensorarrayTransportReleaseDescriptor` |

### sensorarrayTransportStats.c

| C 调用签名 / C signature | 行为及返回 / Behaviour and return | 调用位置 / Called by |
| --- | --- | --- |
| `static bool sensorarrayTransportStreamHas(sensorarrayTransportStream_t stream, sensorarrayTransportStream_t sink)` [23](sensorarrayTransportStats.c#L23) | 根据当前 stream 枚举识别是否包含目标通道。 / Test whether the selected stream includes the requested channel. | `sensorarrayTransportSerialSinkEnabled` / `sensorarrayTransportBleChannelEnabled` |
| `void sensorarrayTransportGetStats(sensorarrayTransportStats_t *outStats)` [29](sensorarrayTransportStats.c#L29) | 查找 统计。 / Look up stats. | `sensorarraySnapshotDiagnosticsFormat` / `sensorarrayRuntimeQueryCommand` |
| `void sensorarrayTransportNoteSerialData(bool sent)` [39](sensorarrayTransportStats.c#L39) | 按提交结果累计 Serial DATA 接纳或丢弃。 / Record serial DATA admission or drop from submission status. | 独立接口 / Standalone entry |
| `void sensorarrayTransportNoteSerialLog(bool sent)` [50](sensorarrayTransportStats.c#L50) | 按提交结果累计 Serial 日志接纳或丢弃。 / Record serial log admission or drop from submission status. | `sensorarrayTransportDrainGuaranteedText` |
| `void sensorarrayTransportSetLegacyCommandCallback(sensorarrayTransportLegacyCommandCallback_t callback, void *context)` [61](sensorarrayTransportStats.c#L61) | 设置 兼容命令回调。 / Set legacy command callback. | `sensorarrayNetStatusInit` |
| `void sensorarrayTransportSetRuntimeQueryCallback(sensorarrayTransportRuntimeQueryCallback_t callback, void *context)` [69](sensorarrayTransportStats.c#L69) | 设置 运行期查询回调。 / Set runtime query callback. | `sensorarrayInitFrontends` / `app_main` |
| `void sensorarrayTransportSetControlReplyPublishedCallback(sensorarrayTransportControlReplyPublishedCallback_t callback)` [77](sensorarrayTransportStats.c#L77) | 设置 控制回复已发布回调。 / Set control reply published callback. | `app_main` |
| `void sensorarrayTransportSetControlReplyFailedCallback(sensorarrayTransportControlReplyFailedCallback_t callback)` [83](sensorarrayTransportStats.c#L83) | 设置 控制回复失败回调。 / Set control reply failed callback. | `app_main` |
| `void sensorarrayTransportSetTxMode(sensorarrayTransportTxMode_t mode)` [89](sensorarrayTransportStats.c#L89) | 设置 发送模式。 / Set tx mode. | `sensorarrayTransportInitOwned` / `sensorarrayTransportHandleRuntimeCommand` |
| `sensorarrayTransportTxMode_t sensorarrayTransportGetTxMode(void)` [99](sensorarrayTransportStats.c#L99) | 查找 发送模式。 / Look up tx mode. | `sensorarrayTransportProcessDescriptor` / `sensorarrayTransportHandleRuntimeCommand` |
| `void sensorarrayTransportSetStream(sensorarrayTransportStream_t stream)` [107](sensorarrayTransportStats.c#L107) | 设置 流。 / Set stream. | `sensorarrayTransportInitOwned` / `sensorarrayTransportHandleRuntimeCommand` |
| `sensorarrayTransportStream_t sensorarrayTransportGetStream(void)` [124](sensorarrayTransportStats.c#L124) | 查找 流。 / Look up stream. | `sensorarrayTransportHandleRuntimeCommand` / `sensorarrayTransportSerialSinkEnabled` |
| `void sensorarrayTransportSetWifiMode(sensorarrayTransportWifiMode_t mode)` [132](sensorarrayTransportStats.c#L132) | 设置 Wi-Fi模式。 / Set Wi-Fi mode. | `sensorarrayTransportInitOwned` |
| `sensorarrayTransportWifiMode_t sensorarrayTransportGetWifiMode(void)` [142](sensorarrayTransportStats.c#L142) | 查找 Wi-Fi模式。 / Look up Wi-Fi mode. `requestedAp ? SENSORARRAY_TRANSPORT_WIFI_AP : SENSORARRAY_TRANSPORT_WIFI_OFF` | `sensorarrayTransportPrintNetInit` / `sensorarrayTransportWifiSinkEnabled` |
| `bool sensorarrayTransportSerialSinkEnabled(void)` [154](sensorarrayTransportStats.c#L154) | 判断串行输出端启用。 / Test serial sink enabled. | `sensorarrayFrameOutputPrint` / `sensorarrayUsbSinkPublishClass` |
| `bool sensorarrayTransportBleSinkEnabled(void)` [163](sensorarrayTransportStats.c#L163) | 判断BLE输出端启用。 / Test BLE sink enabled. | 独立接口 / Standalone entry |
| `bool sensorarrayTransportBleChannelEnabled(sensorarrayTransportChannel_t channel)` [169](sensorarrayTransportStats.c#L169) | 判断BLE通道启用。 / Test BLE channel enabled. 调用 / Uses `sensorarrayTransportGetStream`, `sensorarrayTransportStreamHas`. | `sensorarrayTransportProcessDescriptor` / `sensorarrayTransportDrainGuaranteedText` |
| `bool sensorarrayTransportWifiSinkEnabled(void)` [184](sensorarrayTransportStats.c#L184) | 判断Wi-Fi输出端启用。 / Test Wi-Fi sink enabled. | `sensorarrayTransportProcessDescriptor` / `sensorarrayTransportDrainGuaranteedText` |
| `const char *sensorarrayTransportTxModeName(sensorarrayTransportTxMode_t mode)` [195](sensorarrayTransportStats.c#L195) | 返回发送模式的显示名称。 / Return the display name for tx mode. | `sensorarrayTransportHandleRuntimeCommand` / `sensorarrayAsyncLogPrintCompactSummary` |
| `const char *sensorarrayTransportStreamName(sensorarrayTransportStream_t stream)` [208](sensorarrayTransportStats.c#L208) | 返回流的显示名称。 / Return the display name for stream. | `sensorarrayTransportHandleRuntimeCommand` / `sensorarrayAsyncLogPrintCompactSummary` |
| `const char *sensorarrayTransportWifiModeName(sensorarrayTransportWifiMode_t mode)` [241](sensorarrayTransportStats.c#L241) | 返回Wi-Fi模式的显示名称。 / Return the display name for Wi-Fi mode. | `sensorarrayTransportPrintNetInit` |

### sensorarrayTransportWifiControl.c

| C 调用签名 / C signature | 行为及返回 / Behaviour and return | 调用位置 / Called by |
| --- | --- | --- |
| `static esp_err_t wifiFormat(char *out, size_t size, const char *format, ...)` [10](sensorarrayTransportWifiControl.c#L10) | 按容量写入 Wi-Fi 回复并在格式化失败或超长时返回错误。 / Write a Wi-Fi reply within its buffer and return formatting or length errors. `ESP_ERR_INVALID_ARG` `ESP_ERR_INVALID_SIZE` | `wifiFormatRequest` / `sensorarrayTransportWifiCommand` |
| `static const char *wifiBackendName(const sensorarrayWifiOwnerSnapshot_t *s)` [24](sensorarrayTransportWifiControl.c#L24) | 返回后端的显示名称。 / Return the display name for backend. | `sensorarrayTransportWifiCommand` |
| `static bool wifiPending(sensorarrayWifiRequestOutcome_t state)` [30](sensorarrayTransportWifiControl.c#L30) | 检查指定 Wi-Fi 生命周期结果是否仍待处理。 / Check whether a Wi-Fi lifecycle result remains pending. | `wifiFormatRequest` / `sensorarrayTransportWifiCommand` |
| `static void wifiU64(char *out, size_t size, bool known, uint64_t value)` [35](sensorarrayTransportWifiControl.c#L35) | 在给定格式和容量内输出 uint64 数值。 / Format an unsigned 64-bit value within the supplied capacity. | `wifiFormatRequest` / `sensorarrayTransportWifiCommand` |
| `static esp_err_t wifiFormatRequest(const char *prefix, const sensorarrayWifiOwnerSnapshot_t *s, uint32_t id, const sensorarrayWifiRequestResult_t *r, char *response, size_t size)` [41](sensorarrayTransportWifiControl.c#L41) | 格式化特定 Wi-Fi 请求的受理、执行和终态身份。 / Format the admission, execution and terminal identity of a Wi-Fi request. | `sensorarrayTransportWifiCommand` / `sensorarrayTransportWifiDrainTerminal` |
| `bool sensorarrayTransportWifiCommandHandles(const char *text)` [64](sensorarrayTransportWifiControl.c#L64) | 识别 CAPS2、Wi-Fi 申请、状态及结果查询命令。 / Recognise CAPS2 and Wi-Fi request, state and result queries. | `sensorarrayTransportHandleRuntimeCommand` / `sensorarrayTransportWifiCommand` |
| `esp_err_t sensorarrayTransportWifiCommand(const char *text, char *response, size_t size)` [72](sensorarrayTransportWifiControl.c#L72) | 执行匹配的 Wi-Fi 控制或查询语法并生成回复。 / Execute a matched Wi-Fi command or query and generate its reply. `ESP_ERR_INVALID_ARG` | `sensorarrayTransportHandleRuntimeCommand` |
| `void sensorarrayTransportWifiDrainTerminal(void)` [167](sensorarrayTransportWifiControl.c#L167) | 从 Wi-Fi 所有者领取终态并提交到传输层。 / Drain terminal outcomes from the Wi-Fi owner into transport. | `sensorarrayTransportTask` |

## 参数索引 / Parameter index

相同名称按模块共享解释。完整类型、数组维度与输入输出方向以对应函数签名和调用点为准。指针参数所需存储由调用者提供，异步作业另受上面的所有权约束。

Entries share parameter descriptions within this module. Refer to the signature and its caller for the precise type, array shape and direction. The caller supplies pointer storage, subject to the ownership rules above for asynchronous work.

| 参数 / Parameter | 含义 / Meaning |
| --- | --- |
| `...`, `format` | printf 风格的格式串与匹配格式的可变参数。 / printf-style format string and its matching variadic arguments. |
| `a`, `b` | 数值比较、取中位值或身份比较的输入操作数，按相应签名确定类型。 / Operand for a numeric comparison, median or identity comparison as declared. |
| `arg`, `context` | 回调或任务创建时绑定的上下文；在该调用的有效期内保留其存储。 / Context bound at callback registration or task creation. Keep its backing object alive for the operation. |
| `base` | 基准。 / Base. |
| `ble` | BLE。 / BLE. |
| `bootId`, `sessionId` | 会话、启动身份或启动计数，查询与终态依赖所属启动域。 / Session identifier, boot identifier or boot count scoping queries and terminal outcomes. |
| `buffer`, `out`, `response` | 调用者提供的回复、暂存或结果缓冲区，使用随同的容量/类型。 / Caller-provided reply, scratch or result storage with the accompanying capacity/type. |
| `bytes` | 校准记录/载荷或原始字节；outPayload 指向允许写入的目标。 / Calibration record/payload or raw bytes. outPayload names caller-writable storage. |
| `callback` | 使用头文件定义签名的回调。提供的上下文与回调在注册或同步调用期间有效。 / Callback with the signature declared in the header. Keep its context valid while registered or used. |
| `candidate` | 候选。 / Candidate. |
| `capacityRejected` | 容量被拒绝。 / Capacity rejected. |
| `channel` | 芯片通道编号或传输通道枚举。FDC 硬件通道为 0..3。 / Chip channel number or transport-channel enum. FDC hardware channels are 0..3. |
| `connected` | 已连接。 / Connected. |
| `counter` | 计数器。 / Counter. |
| `data`, `text` | 按该函数声明传入的文本、命令、操作码或数据字节，显式 length 限定可读范围。 / Text, command, opcode or data bytes as declared. Where supplied, length bounds the readable range. |
| `dataBudget` | 数据预算。 / Data budget. |
| `dataChannelSendable` | 数据通道可发送。 / Data channel sendable. |
| `descriptor`, `pool`, `slot` | 固定载荷池、带代数的队列描述符或其槽位。 / Fixed payload pool, generation-tagged queue descriptor or backing slot. |
| `epoch` | 轮次。 / Epoch. |
| `err` | 操作、首个根因或独立清理结果的错误码。保留调用层提供的具体错误。 / Error for the operation, first cause or separate cleanup, preserving the code from the reporting layer. |
| `fd`, `flags`, `path` | VFS 描述符、open 标志或路径，或对应类型的矩阵路径。 / VFS descriptor, open flags or path, or a matrix route if the declared type selects one. |
| `generation` | 驱动边沿、缓存或槽位的世代，按本函数的对象匹配，跨对象不能直接混用。 / Driver-edge, cache or slot generation. Match it to the object identified by the function. |
| `id` | 本地请求或事务标识，out 参数接收新分配或选中的 ID。 / Local request or transaction identifier. An out parameter receives the allocated or selected ID. |
| `identity` | 帧的不可变配置身份或同一版本的完整权威快照。 / Immutable frame configuration identity or the authority snapshot for that revision. |
| `index` | 索引。 / Index. |
| `input` | 输入。 / Input. |
| `inputLength`, `length`, `storageBytes` | 输入、载荷、底层存储或读写传输的字节数。 / Byte length of input, payload, backing storage or transfer. |
| `item` | 条目。 / Item. |
| `key` | 关键。 / Key. |
| `known` | 表示证据已知、数据有效、功能启用或操作成功的布尔标志，按参数名独立解释。 / Boolean representing known evidence, validity, enablement or success as named. |
| `lane`, `priority`, `stream` | 传输数据/调试/优先通道或其选择条件。 / Stream/lane selection or priority determining which local data/debug queue is used. |
| `last` | 上一次。 / Last. |
| `line` | 线。 / Line. |
| `logChannelSendable` | 日志通道可发送。 / Log channel sendable. |
| `mac` | MAC。 / Mac. |
| `maximum` | 最大。 / Maximum. |
| `metadata` | 从整帧编码时提取的固定元数据，不可改盖成发送时的配置。 / Fixed metadata taken at frame encoding and retained through transmission. |
| `mode` | 对应调用域的模式枚举，保留目标、当前帧或恢复上下文的含义。 / Mode enum in the declared domain, selecting the target, frame or recovery mode as indicated. |
| `notify` | 通知。 / Notify. |
| `nowUs` | 单调时钟的事件时间，单位 µs。 / Event time on the monotonic clock in µs. |
| `outDescriptor` | 接收结果。固定载荷池、带代数的队列描述符或其槽位。 / Output. Fixed payload pool, generation-tagged queue descriptor or backing slot. |
| `outDiagnostics` | 接收诊断。 / Receives diagnostics. |
| `outLength` | 接收结果。输入、载荷、底层存储或读写传输的字节数。 / Output. Byte length of input, payload, backing storage or transfer. |
| `outQueued` | 返回本次调用是否向网络描述符队列实际投递。 / Receives whether the call actually queued a network descriptor. |
| `outReason`, `reason` | 操作或拒绝原因字符串；outReason 用于返回原因。 / Operation/rejection reason string. outReason receives the reason. |
| `outSize`, `responseSize`, `size`, `textSize` | 接收文本或字节缓冲区的容量。文本输出还需预留 NUL。 / Capacity of the supplied byte/text buffer. A text output must also accommodate its NUL terminator. |
| `outStats`, `result` | 函数签名注明的结果、快照或统计结构，调用者分配复制目标。 / Result, snapshot or statistics object of the declared type, with caller-owned output storage. |
| `outText` | 接收结果。按该函数声明传入的文本、命令、操作码或数据字节，显式 length 限定可读范围。 / Output. Text, command, opcode or data bytes as declared. Where supplied, length bounds the readable range. |
| `outValue` | 接收值。 / Receives value. |
| `oversize` | 超长。 / Oversize. |
| `peer` | 对端。 / Peer. |
| `prefix` | 前缀。 / Prefix. |
| `protectionFlags` | 调用域对应的标志位；帧保护位在编码、入队和重试间保留。 / Domain-specific bit flags. Frame protection flags survive encoding, queuing and retries. |
| `r` | 记录。 / R. |
| `replyTarget` | 回复目标。 / Reply target. |
| `requestId` | 对应配置、恢复触发或 profile 域内的请求标识，由接收方与提交时身份配对。 / Request identifier for the indicated configuration, recovery trigger or row profile, paired with the identity at admission. |
| `s` | S。 / S. |
| `sent` | 已发送。 / Sent. |
| `sink` | 输出端。 / Sink. |
| `sinkCounted` | 输出端已计数。 / Sink counted. |
| `st` | 状态结构。 / St. |
| `state` | 所声明类型的状态对象或枚举。传入前端状态指针时，其生命周期覆盖总线或采集操作。 / State object or enum of the declared type. A frontend-state pointer must outlive the bus or capture operation. |
| `storage` | 接收、发送或长期存储缓冲；与签名中的容量或传输长度配对。 / Receive, transmit or backing buffer paired with the supplied capacity/transfer length. |
| `streamWantsBle` | 流要求BLE。 / Stream wants BLE. |
| `target` | 此操作的目标，可能为目标帧、路由/测点结构或 sink，以签名类型为准。 / Target frame, route/cell or sink, according to the declared type. |
| `task` | 任务。 / Task. |
| `value` | 值。 / Value. |
| `wifiConfig` | Wi-Fi配置。 / Wi-Fi config. |
| `wifiErr` | Wi-Fi错误码。 / Wi-Fi err. |

[项目总览 / Project guide](../../README.md)

# 命令与只读诊断 / Command handling and read-only diagnostics

解析器把控制文本转换为有界的 mailbox 项。配置、RECOVER、RESTART/USBBOOT 和 ADS 时序工作各有接受、领取和终态。硬件作业由采集所有者执行，网络回调只投递或复制快照。预留的重启路径可以在普通采集尚未就绪时使用。内存和 panic 诊断将首次故障证据保存为有界记录。

Parsers turn control text into bounded mailbox entries. Configuration, RECOVER, RESTART/USBBOOT and ADS timing work have explicit post, take and terminal stages. The acquisition owner performs hardware work. Transport callbacks submit requests or read snapshots. The restart lane is available before ordinary acquisition is ready. Memory and panic diagnostics retain bounded first-failure evidence.

## 调用方式 / Calling the module

```c
uint8_t command[] = "TRACE=0";
esp_err_t err = sensorarrayCommandMailboxPostText(command, sizeof(command) - 1u);
```

上例通过带长度的邮箱入口请求关闭跟踪。只读查询通常由 transport/runtime query 分派。调用者传入明确长度，格式化输出需可写缓冲区。RECOVER 保存提交时的配置身份；过期请求不操作新配置。RESTART/USBBOOT 依赖本地 ACK 接纳，是否收到或完成重新枚举应由主机独立确认。

The example requests trace-off through the length-delimited mailbox entry. Normal read-only protocol queries first pass through the transport/runtime query dispatcher. Supply an explicit command length and writable output buffers. RECOVER retains the configuration identity captured at admission. A stale request cannot act on a new configuration. RESTART/USBBOOT depend on local ACK admission. The host independently checks reply receipt and re-enumeration.

## 源码和头文件 / Source and headers

| 文件 / File | 职责与声明 / Responsibility and declarations |
| --- | --- |
| [sensorarrayAdsTimingControl.c](sensorarrayAdsTimingControl.c) · [sensorarrayAdsTimingControl.h](sensorarrayAdsTimingControl.h) | 为 ADS 时序诊断保留一次有界请求、ACK 与工作结果，支持 boot 内的按 ID 查询。 / Holds one bounded ADS timing diagnostic request, its ACK and worker result for boot-local lookup by ID. `sensorarrayAdsTimingRequest_t` `sensorarrayAdsTimingTake_t` |
| [sensorarrayCommandMailbox.c](sensorarrayCommandMailbox.c) · [sensorarrayCommandMailbox.h](sensorarrayCommandMailbox.h) | 解析参数、预留全局顺序并把已接受的控制操作传递到采集任务。 / Parses arguments, reserves request ordering and transfers accepted control operations to the acquisition task. `sensorarrayCommandType_t` `sensorarrayCommand_t` |
| [sensorarrayEarlyRecovery.c](sensorarrayEarlyRecovery.c) · [sensorarrayEarlyRecovery.h](sensorarrayEarlyRecovery.h) | 在完整应用尚未启动时解析有限的恢复命令以保持只读查询或受控恢复入口。 / Recognises the limited recovery command grammar before full application startup. `sensorarrayEarlyRecoveryKind_t` |
| [sensorarrayLinkDiagnostics.c](sensorarrayLinkDiagnostics.c) · [sensorarrayLinkDiagnostics.h](sensorarrayLinkDiagnostics.h) | 格式化 BLE 链路、流控、保护与工作预算查询。 / Formats BLE link, flow, guard and work-budget queries. |
| [sensorarrayMemoryDiagnostics.c](sensorarrayMemoryDiagnostics.c) · [sensorarrayMemoryDiagnostics.h](sensorarrayMemoryDiagnostics.h) | 保存内存分配失败的第一次和最近证据，并报告堆、栈与扫描任务的预留情况。 / Captures first and recent allocation-failure evidence and reports heap, stack and scan-task reservations. |
| [sensorarrayPanicDiagnostics.c](sensorarrayPanicDiagnostics.c) · [sensorarrayPanicDiagnostics.h](sensorarrayPanicDiagnostics.h) | 在固定存储中记录 panic 核心与寄存器上下文、校验标记和启动来源，控制任务可只读格式化。 / Captures panic core/register context, integrity markers and boot provenance in fixed storage for later read-only formatting. |
| [sensorarrayRecoveryControl.c](sensorarrayRecoveryControl.c) · [sensorarrayRecoveryControl.h](sensorarrayRecoveryControl.h) | RECOVER 事务绑定当前配置身份，经过 ACK、领取、执行及带根因和清理错误的终态。 / Binds RECOVER to the current identity through ACK, take, execution and a terminal result with first and cleanup errors. `sensorarrayRecoverState_t` `sensorarrayRecoverRequest_t` |
| [sensorarrayRestartControl.c](sensorarrayRestartControl.c) · [sensorarrayRestartControl.h](sensorarrayRestartControl.h) | 为显式重启或 ROM 下载保留独立事务；本地 ACK 后由所有者领取并公布结果。 / Reserves a separate explicit restart/ROM-download transaction for the owner to take and publish after local ACK admission. |
| [sensorarraySnapshotDiagnostics.c](sensorarraySnapshotDiagnostics.c) · [sensorarraySnapshotDiagnostics.h](sensorarraySnapshotDiagnostics.h) | 格式化异步输出与传输层的快照保护和投递计数。 / Formats asynchronous output and transport snapshot protection/admission counters. |
| [sensorarrayUsbControl.c](sensorarrayUsbControl.c) · [sensorarrayUsbControl.h](sensorarrayUsbControl.h) | 解释 USB 能力、流选择及状态请求，通过输出层切换或查询对应接口。 / Interprets USB capability, stream-selection and status requests against the output interface. |

## 函数 / Functions

签名按源文件排列，参数顺序即调用顺序。下表标出生产代码中的调用者或回调注册者（最多两个）。`static` 限于所在编译单元。错误码列于对应行为后，只列函数直接返回的常量；硬件与下层调用可传播其他错误。

Signatures are grouped by source in call-argument order. The last column names up to two production callers or callback registrars. `static` entries are local to their translation unit. Constants after a description are explicit error returns. Underlying operations may propagate additional errors.

[sensorarrayAdsTimingControl.c](#sensorarrayadstimingcontrolc) · [sensorarrayCommandMailbox.c](#sensorarraycommandmailboxc) · [sensorarrayEarlyRecovery.c](#sensorarrayearlyrecoveryc) · [sensorarrayLinkDiagnostics.c](#sensorarraylinkdiagnosticsc) · [sensorarrayMemoryDiagnostics.c](#sensorarraymemorydiagnosticsc) · [sensorarrayPanicDiagnostics.c](#sensorarraypanicdiagnosticsc) · [sensorarrayRecoveryControl.c](#sensorarrayrecoverycontrolc) · [sensorarrayRestartControl.c](#sensorarrayrestartcontrolc) · [sensorarraySnapshotDiagnostics.c](#sensorarraysnapshotdiagnosticsc) · [sensorarrayUsbControl.c](#sensorarrayusbcontrolc)

### sensorarrayAdsTimingControl.c

| C 调用签名 / C signature | 行为及返回 / Behaviour and return | 调用位置 / Called by |
| --- | --- | --- |
| `void sensorarrayAdsTimingControlInit(uint64_t session, uint32_t bootId)` [53](sensorarrayAdsTimingControl.c#L53) | 初始化会话与启动身份，预分配诊断工作区和结果区，保留存储初始化状态。 / Bind the session/boot and preallocate diagnostic workspace and result storage. | `app_main` |
| `esp_err_t sensorarrayAdsTimingControlStorageError(void)` [77](sensorarrayAdsTimingControl.c#L77) | 返回初始化结果，供调用方区分诊断区未就绪、分配失败或可用。 / Return the storage initialization status to prospective diagnostic callers. | `sensorarrayRuntimeQueryCommand` |
| `esp_err_t sensorarrayAdsTimingControlPost(const sensorarrayConfigAuthority_t *authority, uint64_t nowUs, uint32_t *outId)` [86](sensorarrayAdsTimingControl.c#L86) | 按默认 TIMING 类型申请诊断，向 outId 交付关联号。 / Reserve a regular timing probe and return its request ID. | 独立接口 / Standalone entry |
| `esp_err_t sensorarrayAdsTimingControlPostKind(const sensorarrayConfigAuthority_t *authority, sensorarrayAdsTimingProbeKind_t kind, uint64_t nowUs, uint32_t *outId)` [93](sensorarrayAdsTimingControl.c#L93) | 校验权威配置、boot 和 storage，预留所选 timing/tail 命令与截止时间，拒绝同时占用的请求。 / Validate identity and reserve one diagnostic of the requested kind with an expiry. | `sensorarrayAdsTimingControlPost` / `sensorarrayRuntimeQueryCommand` |
| `bool sensorarrayAdsTimingControlAck(uint32_t id, esp_err_t error)` [125](sensorarrayAdsTimingControl.c#L125) | 按 id 登记本地 ACK 发布结果；成功才开放执行，失败撤销预留。 / Arm the reserved request on local ACK success or cancel it when publication fails. | `sensorarrayAdsTimingControlExpire` / `sensorarrayControlReplyPublished` |
| `sensorarrayAdsTimingTake_t sensorarrayAdsTimingControlTake(uint32_t id, const sensorarrayConfigAuthority_t *authority, bool known, uint64_t nowUs, sensorarrayAdsTimingRequest_t *outRequest)` [155](sensorarrayAdsTimingControl.c#L155) | 对已 ACK 请求检查到期时间和配置身份，返回 READY、EXPIRED、STALE 或 NONE。 / Take an acknowledged request only with matching ID, time budget and capture identity. | `sensorarrayRunAdsTimingCommand` |
| `sensorarrayAdsTimingProbeResult_t *sensorarrayAdsTimingControlWorkResult(void)` [179](sensorarrayAdsTimingControl.c#L179) | 返回持久 work 结果指针，无法取得存储时为 NULL。 / Return the persistent producer workspace if diagnostic storage exists. | `sensorarrayRunAdsTimingCommand` |
| `esp_err_t sensorarrayAdsTimingControlComplete(uint32_t id, const char *outcome, const char *reason, esp_err_t originalError, esp_err_t cleanupError, const sensorarrayConfigAuthority_t *authority, bool known, bool keepResult)` [182](sensorarrayAdsTimingControl.c#L182) | 固化结果、首错和恢复证据，并把该请求的 ADSTERM 放入终态路径。 / Cache the completed probe and publish its bounded terminal with independent cleanup evidence. | `sensorarrayAdsTimingControlAck` / `sensorarrayAdsTimingControlExpire` |
| `void sensorarrayAdsTimingControlExpire(uint64_t nowUs, const sensorarrayConfigAuthority_t *authority, bool known)` [222](sensorarrayAdsTimingControl.c#L222) | 终结超过等待期限且尚未执行的诊断，保留已分配请求的关联号。 / Expire eligible pending work and report the original request identity. | `sensorarrayApplyPendingCommands` |
| `bool sensorarrayAdsTimingControlHandles(const char *command)` [235](sensorarrayAdsTimingControl.c#L235) | 识别 ADSTIMING? 命令及分页前缀，由 Format 再核验具体语法。 / Recognize the timing-query family before validating the requested page. | `sensorarrayEarlyRuntimeQueryCommand` / `sensorarrayRuntimeQueryCommand` |
| `static bool sensorarrayTimingParseUnsigned(const char **text, uint32_t *value)` [238](sensorarrayAdsTimingControl.c#L238) | 读取查询中的无符号十进制分量并核验上界与分隔符。 / Parse and validate a bounded unsigned field in the paged query. | `sensorarrayAdsTimingControlFormat` |
| `esp_err_t sensorarrayAdsTimingControlFormat(const char *command, char *response, size_t responseSize)` [250](sensorarrayAdsTimingControl.c#L250) | 解析请求号和页索引，从已发布的元数据、组、寄存器或原始记录生成有界回复。 / Format a selected metadata, group, raw or register page from retained results. | `sensorarrayEarlyRuntimeQueryCommand` / `sensorarrayRuntimeQueryCommand` |

### sensorarrayCommandMailbox.c

| C 调用签名 / C signature | 行为及返回 / Behaviour and return | 调用位置 / Called by |
| --- | --- | --- |
| `static esp_err_t sensorarrayCommandMailboxQueue(const sensorarrayCommand_t *command)` [41](sensorarrayCommandMailbox.c#L41) | 在共用生产者临界区中以零等待复制一条命令，容量耗尽返回错误。 / Enqueue one copied command under producer serialization without blocking on space. | `sensorarrayCommandMailboxPostText` / `sensorarrayCommandMailboxPostRailCalibration` |
| `static void sensorarrayCommandNormalize(char *text)` [56](sensorarrayCommandMailbox.c#L56) | 原地去除命令首尾空白并规范 ASCII 大小写，以便匹配字面量。 / Trim and normalize a mutable command line before matching literals. | `sensorarrayCommandParse` |
| `static esp_err_t sensorarrayCommandParseUnsigned(const char *value, uint32_t maximum, uint32_t *outValue)` [81](sensorarrayCommandMailbox.c#L81) | 将十进制参数解析到输出变量，拒绝越界或尾部无效字符。 / Parse the unsigned argument through the specified maximum, rejecting invalid syntax. | `sensorarrayCommandParseFpsCap` / `sensorarrayCommandParse` |
| `static esp_err_t sensorarrayCommandParseFpsCap(const char *text, const char *prefix, sensorarrayCommandType_t type, sensorarrayCommand_t *outCommand)` [98](sensorarrayCommandMailbox.c#L98) | 将给定前缀下的速率设定转为指定采集/输出命令类型及 FPS。 / Parse a frame-rate cap under the supplied prefix into the requested mailbox command. | `sensorarrayCommandParse` |
| `static esp_err_t sensorarrayCommandParse(const uint8_t *text, size_t length, sensorarrayCommand_t *outCommand)` [126](sensorarrayCommandMailbox.c#L126) | 从带长度的输入构造已校验的命令对象；未识别或参数错误时直接返回错误。 / Decode bounded input into a typed command or return a parser error. | `sensorarrayCommandMailboxPostText` |
| `esp_err_t sensorarrayCommandMailboxInit(void)` [249](sensorarrayCommandMailbox.c#L249) | 创建静态队列，初始化预算/调试开关并注册行数请求回调，重复调用可复用。 / Initialize the static queue, runtime controls and row-request callback once. | `sensorarrayInitRuntime` |
| `esp_err_t sensorarrayCommandMailboxPostText(const uint8_t *text, size_t length)` [276](sensorarrayCommandMailbox.c#L276) | 解析完整文本再投递；仅成功入队后允许来源发布接受。 / Parse and enqueue the complete text before local acceptance can be issued. | `sensorarrayRuntimeQueryCommand` / `sensorarrayNetLegacyCommand` |
| `static esp_err_t sensorarrayCommandMailboxPostReservedConfig(sensorarrayCommandType_t type, uint32_t value, uint32_t *outRequestId)` [294](sensorarrayCommandMailbox.c#L294) | 为配置命令取得请求 ID、保留 ACK/终态容量，并在失败时释放相关预约。 / Reserve request identity and publication capacity for a configuration operation. | `sensorarrayCommandMailboxPostMeasurementMode` / `sensorarrayCommandMailboxPostRows` |
| `esp_err_t sensorarrayCommandMailboxPostMeasurementMode(sensorarrayMeasurementMode_t mode, uint32_t *outRequestId)` [316](sensorarrayCommandMailbox.c#L316) | 将目标测量模式作为配置域请求提交给帧边界 owner。 / Submit a target mode using the shared configuration-request path. | `sensorarrayRuntimeQueryCommand` |
| `esp_err_t sensorarrayCommandMailboxPostRows(uint8_t rows, sensorarrayScanConfigRequestResult_t *outResult)` [324](sensorarrayCommandMailbox.c#L324) | 检查行数范围，提交 ROWS 并将请求号及接受结果交回调用者。 / Submit a validated row count and return its admission result. | `sensorarrayCommandMailboxInit` / `sensorarrayFrontendRowsRequest` |
| `esp_err_t sensorarrayCommandMailboxPostRowModes(const char *profile, uint32_t *outRequestId)` [340](sensorarrayCommandMailbox.c#L340) | 校验八行 profile，在多通路共用的预约状态中准备请求和关联号。 / Validate the eight-row profile and reserve a request across control transports. | `sensorarrayRuntimeQueryCommand` |
| `bool sensorarrayCommandMailboxCommitRowModesAck(uint32_t requestId)` [391](sensorarrayCommandMailbox.c#L391) | 收到对应本地 ACK 发布证据后使被预留的操作可被消费者领取。 / Release the reserved operation to the consumer after its matching ACK was published. | `sensorarrayCommandMailboxCommitConfigAck` |
| `bool sensorarrayCommandMailboxCancelRowModesAck(uint32_t requestId)` [418](sensorarrayCommandMailbox.c#L418) | 来源 ACK 未发布时取消相应预约并回收未使用容量。 / Cancel an unpublished reservation without letting the setter reach the consumer. | `sensorarrayCommandMailboxEmitRowModesTerminal` / `sensorarrayCommandMailboxCancelConfigAck` |
| `esp_err_t sensorarrayCommandMailboxEmitRowModesTerminal(uint32_t requestId, const char *text, size_t length)` [435](sensorarrayCommandMailbox.c#L435) | 按请求号向保留的终态槽发布文本，解锁时机取决于是否仍有相同操作在途。 / Publish terminal text for the matching reservation and update its ownership. | `sensorarrayCommandMailboxEmitConfigTerminal` |
| `uint32_t sensorarrayCommandMailboxRowModesOutstanding(void)` [480](sensorarrayCommandMailbox.c#L480) | 在临界区读取当前是否存在未结束的预约。 / Report whether a reserved operation remains outstanding. | 独立接口 / Standalone entry |
| `esp_err_t sensorarrayCommandMailboxPostRailCalibration(int32_t avddUv, int32_t avssUv, uint32_t *outRequestId)` [488](sensorarrayCommandMailbox.c#L488) | 将带符号的外部 AVDD/AVSS 微伏参数提交为帧边界事务。 / Queue the external per-rail microvolt readings for boundary application. | `sensorarrayRuntimeQueryCommand` |
| `static esp_err_t sensorarrayCommandMailboxPostRequest(sensorarrayCommandType_t type, uint32_t value, int32_t signedValue, uint32_t *outRequestId)` [512](sensorarrayCommandMailbox.c#L512) | 为调用方传入的操作建立关联号并投递，失败时返回具体错误。 / Assign a request identity and enqueue the caller's bounded action. | `sensorarrayCommandMailboxPostAdsCheck` / `sensorarrayCommandMailboxPostBatteryNow` |
| `esp_err_t sensorarrayCommandMailboxPostAdsCheck(uint32_t sampleCount, uint32_t *outRequestId)` [536](sensorarrayCommandMailbox.c#L536) | 申请下一合适边界执行一次 ADS 主动检查并返回请求号。 / Queue an ADC active check and return its ID. | `sensorarrayRuntimeQueryCommand` |
| `esp_err_t sensorarrayCommandMailboxPostAdsTiming(uint32_t samplesPerGroup, uint32_t *outRequestId)` [548](sensorarrayCommandMailbox.c#L548) | 以现有诊断 request ID 交付 TIMING 作业。 / Enqueue the timing diagnostic with its reserved ID. | `sensorarrayAdsTimingControlPostKind` |
| `esp_err_t sensorarrayCommandMailboxPostAdsTail(uint32_t samplesPerGroup, uint32_t *outRequestId)` [556](sensorarrayCommandMailbox.c#L556) | 以现有诊断 request ID 交付 TAIL 末位比较作业。 / Enqueue the final-bit comparison with its reserved ID. | `sensorarrayAdsTimingControlPostKind` |
| `esp_err_t sensorarrayCommandMailboxPostBatteryNow(bool diagnostic, uint32_t *outRequestId)` [564](sensorarrayCommandMailbox.c#L564) | 请求一次电池采样，结果在后续允许的采集边界发布。 / Request one battery measurement at an eligible boundary. | `sensorarrayRuntimeQueryCommand` |
| `esp_err_t sensorarrayCommandMailboxPostBatteryPeriod(bool enabled, uint32_t periodMs, uint32_t *outRequestId)` [575](sensorarrayCommandMailbox.c#L575) | 校验并提交电池调度周期间隔，返回操作关联号。 / Submit the battery sampling interval for boundary application. | `sensorarrayRuntimeQueryCommand` |
| `esp_err_t sensorarrayCommandMailboxPostResSettle(uint32_t settleUs, uint32_t *outRequestId)` [588](sensorarrayCommandMailbox.c#L588) | 提交按微秒计的 RES 建立时间，超范围请求返回错误。 / Submit the resistance settling interval in microseconds within its allowed range. | `sensorarrayRuntimeQueryCommand` |
| `esp_err_t sensorarrayCommandMailboxPostFdcIsolation(bool enabled, uint32_t *outRequestId)` [600](sensorarrayCommandMailbox.c#L600) | 提交 FDC SD 隔离请求，由硬件 owner 在边界执行。 / Queue a shutdown-isolation request for the hardware owner. | `sensorarrayRuntimeQueryCommand` |
| `bool sensorarrayCommandMailboxTryReceive(sensorarrayCommand_t *outCommand)` [609](sensorarrayCommandMailbox.c#L609) | 以非阻塞方式取出一条满足预约条件的命令并复制给调用者。 / Take one eligible queued command into caller storage without waiting. | `sensorarrayApplyPendingCommands` |
| `void sensorarrayCommandMailboxCommit(const sensorarrayCommand_t *command)` [630](sensorarrayCommandMailbox.c#L630) | 按已执行的命令类型更新运行期速率、跟踪或诊断开关。 / Commit runtime controls for the completed command type. | `sensorarrayApplyPendingCommands` |
| `uint32_t sensorarrayCommandMailboxGetBleCapPeriod(void)` [651](sensorarrayCommandMailbox.c#L651) | 返回已提交的 BLE CAP 发布周期。 / Read the currently committed BLE capacitance publication period. | 独立接口 / Standalone entry |
| `bool sensorarrayCommandMailboxTraceEnabled(void)` [659](sensorarrayCommandMailbox.c#L659) | 查询运行时是否启用跟踪日志。 / Read the trace-enable control used by producers. | `sensorarrayAsyncLogTask` |
| `bool sensorarrayCommandMailboxAdsDebugEnabled(void)` [667](sensorarrayCommandMailbox.c#L667) | 查询是否启用 ADS 额外诊断。 / Read the ADC debug-output enable control. | `sensorarrayRuntimeQueryCommand` / `sensorarrayAsyncLogPrintCompactSummary` |
| `uint32_t sensorarrayCommandMailboxGetCaptureFpsCap(void)` [675](sensorarrayCommandMailbox.c#L675) | 返回当前采集上限 FPS，零表示不限制。 / Read the capture-rate ceiling in frames per second; zero means uncapped. | `sensorarrayFrameTargetFps` / `sensorarrayRuntimeQueryCommand` |
| `uint32_t sensorarrayCommandMailboxGetOutputFpsCap(void)` [683](sensorarrayCommandMailbox.c#L683) | 返回当前输出上限 FPS，独立于采集上限。 / Read the output-rate cap independently of capture pacing. | `sensorarrayRuntimeQueryCommand` / `sensorarrayAsyncLogTask` |
| `const char *sensorarrayCommandMailboxTypeName(sensorarrayCommandType_t type)` [691](sensorarrayCommandMailbox.c#L691) | 将命令类型枚举映射为日志及拒绝事件使用的稳定名称。 / Return a command-type label for diagnostics and rejected operations. | `sensorarrayRejectHeldCommand` / `sensorarrayAsyncLogPrintEvent` |
| `bool sensorarrayCommandMailboxCommitConfigAck(uint32_t requestId)` [746](sensorarrayCommandMailbox.c#L746) | 根据已发布 ACK 的 id 提交相应配置预约。 / Commit the configuration reservation matching a locally published ACK. | `sensorarrayAdsTimingControlAck` / `sensorarrayControlReplyPublished` |
| `bool sensorarrayCommandMailboxCancelConfigAck(uint32_t requestId)` [748](sensorarrayCommandMailbox.c#L748) | 取消与失败 ACK 对应的配置预约。 / Cancel the matching configuration reservation after ACK publication failed. | `sensorarrayAdsTimingControlAck` / `sensorarrayAdsTimingControlComplete` |
| `esp_err_t sensorarrayCommandMailboxEmitConfigTerminal(uint32_t requestId, const char *text, size_t length)` [750](sensorarrayCommandMailbox.c#L750) | 使用请求绑定的终态容量发布格式化的配置结果。 / Publish the formatted result through the terminal reservation associated with the request. | `sensorarrayAdsTimingControlComplete` / `sensorarrayEmitConfigTerminal` |

### sensorarrayEarlyRecovery.c

| C 调用签名 / C signature | 行为及返回 / Behaviour and return | 调用位置 / Called by |
| --- | --- | --- |
| `sensorarrayEarlyRecoveryKind_t sensorarrayEarlyRecoveryParse(const char *text, uint32_t *outRecoveryLevel)` [6](sensorarrayEarlyRecovery.c#L6) | 分类启动期间允许的查询和 RECOVER/RESTART 命令，并返回解析出的恢复级别。 / Classify commands available during early startup and extract the requested recovery level. | `sensorarrayEarlyRuntimeQueryCommand` |

### sensorarrayLinkDiagnostics.c

| C 调用签名 / C signature | 行为及返回 / Behaviour and return | 调用位置 / Called by |
| --- | --- | --- |
| `bool sensorarrayLinkDiagnosticsHandles(const char *cmd)` [8](sensorarrayLinkDiagnostics.c#L8) | 检查是否属于 BLE、保护快照或工作预算的只读诊断命令。 / Route supported read-only link, protection and work-budget queries to this formatter. | `sensorarrayEarlyRuntimeQueryCommand` / `sensorarrayRuntimeQueryCommand` |
| `esp_err_t sensorarrayLinkDiagnosticsFormat(const char *cmd, char *response, size_t size)` [17](sensorarrayLinkDiagnostics.c#L17) | 从 BLE 链路/流控/保护和工作预算快照格式化只读回复。 / Format read-only replies from BLE link/flow/guard and work-budget snapshots. `ESP_ERR_INVALID_ARG` `ESP_ERR_NOT_SUPPORTED` | `sensorarrayEarlyRuntimeQueryCommand` / `sensorarrayRuntimeQueryCommand` |

### sensorarrayMemoryDiagnostics.c

| C 调用签名 / C signature | 行为及返回 / Behaviour and return | 调用位置 / Called by |
| --- | --- | --- |
| `static void IRAM_ATTR sensorarrayMemoryAllocationFailed(size_t requestedSize, uint32_t caps, const char *functionName)` [63](sensorarrayMemoryDiagnostics.c#L63) | 在分配失败钩子中以不等待的临界区尝试记录证据，保存统计且避免再次分配。 / Attempt a nonblocking critical section in the allocation-failure hook and retain evidence without allocating. | `sensorarrayMemoryDiagnosticsInit` |
| `esp_err_t sensorarrayMemoryDiagnosticsInit(uint64_t sessionId, uint32_t bootId, const uint8_t elfSha256[32])` [88](sensorarrayMemoryDiagnostics.c#L88) | 按 boot/session 注册内存诊断及分配失败观察，防止重复注册。 / Initialize boot-scoped heap diagnostics and the failed-allocation observation hook. | `app_main` |
| `esp_err_t sensorarrayMemoryDiagnosticsFailureFormat(char *response, size_t size)` [124](sensorarrayMemoryDiagnostics.c#L124) | 从缓存的失败次数、分配参数与完整性标志产生 MEMFAIL 回复。 / Format allocator-failure evidence, including whether observations were missed. | `sensorarrayEarlyRuntimeQueryCommand` / `sensorarrayRuntimeQueryCommand` |
| `void sensorarrayMemoryDiagnosticsScanBegin(uint32_t stackBytes)` [182](sensorarrayMemoryDiagnostics.c#L182) | 在创建 scan task 前记录请求栈字节数、内部空闲及最大连续块。 / Capture internal heap and requested stack size before creating the scan task. | `sensorarrayStartScanTask` |
| `void sensorarrayMemoryDiagnosticsScanEnd(esp_err_t result)` [193](sensorarrayMemoryDiagnostics.c#L193) | 保存任务创建返回码和创建后的内存观察，结束这一组对照。 / Record the creation result and the matching after-allocation observation. | `sensorarrayStartScanTask` |
| `static const char *sensorarrayMemoryNumber(bool known, uint32_t value, char text[16])` [207](sensorarrayMemoryDiagnostics.c#L207) | 把已知整数写成十进制文本，未知时返回 unknown。 / Format a known counter or return the literal unknown. | `sensorarrayMemoryDiagnosticsFormat` |
| `esp_err_t sensorarrayMemoryDiagnosticsFormat(char *response, size_t size)` [215](sensorarrayMemoryDiagnostics.c#L215) | 输出 MEM 查询中的 heap 分区、最大块与扫描任务存储证据。 / Format capability-specific heap and scan-task allocation evidence. | `sensorarrayEarlyRuntimeQueryCommand` / `sensorarrayRuntimeQueryCommand` |

### sensorarrayPanicDiagnostics.c

| C 调用签名 / C signature | 行为及返回 / Behaviour and return | 调用位置 / Called by |
| --- | --- | --- |
| `static bool IRAM_ATTR panicInternalRange(const void *pointer, size_t length, size_t alignment)` [50](sensorarrayPanicDiagnostics.c#L50) | 核对待读取地址范围是否落在允许的内部内存区域。 / Bound panic-time memory access to permitted internal address ranges. | `panicCapture` |
| `static uint32_t IRAM_ATTR panicRecordCrc(const volatile sensorarrayPanicRecord_t *record)` [60](sensorarrayPanicDiagnostics.c#L60) | 对保留的 panic 记录计算校验值供下次启动核验。 / Compute the integrity check for the retained diagnostic record. | `panicCapture` / `panicRecordValid` |
| `static void IRAM_ATTR panicCapture(const panic_info_t *info)` [74](sensorarrayPanicDiagnostics.c#L74) | 在 panic 上下文用固定内部存储记录故障现场与 CRC。 / Capture failure context and CRC in fixed internal storage on the panic path. | `__wrap_esp_panic_handler` |
| `void IRAM_ATTR __wrap_esp_panic_handler(panic_info_t *info)` [145](sensorarrayPanicDiagnostics.c#L145) | 先保存故障记录，再按原签名调用 IDF 的 panic 处理器。 / Capture failure evidence and forward to the IDF panic handler using its original signature. | 链接器包装 / Linker wrapper |
| `static bool panicRecordValid(void)` [151](sensorarrayPanicDiagnostics.c#L151) | 验证记录头和 CRC；只有通过才可对主机发布。 / Check format and checksum before exposing retained exception data. | `sensorarrayPanicDiagnosticsInit` |
| `void sensorarrayPanicDiagnosticsInit(uint64_t sessionId, uint32_t bootId, const uint8_t elfSha256[32], esp_reset_reason_t resetReason)` [165](sensorarrayPanicDiagnostics.c#L165) | 初始化异常诊断的当前查询身份与镜像信息，并检查保留的来源记录。 / Establish current image/session metadata and validate the retained panic observation. | `app_main` |
| `bool sensorarrayPanicDiagnosticsHandles(const char *command)` [198](sensorarrayPanicDiagnostics.c#L198) | 区分 PANIC?、PANIC?1 和 PANIC?2 三个只读页。 / Recognize the three supported panic query pages. | `sensorarrayPanicDiagnosticsFormat` / `sensorarrayEarlyRuntimeQueryCommand` |
| `static void panicHex(const uint8_t *bytes, size_t length, char *output)` [204](sensorarrayPanicDiagnostics.c#L204) | 将有界的原始详情字节编码成查询使用的十六进制文本。 / Encode a bounded slice of retained detail bytes as hexadecimal. | `sensorarrayPanicDiagnosticsFormat` |
| `esp_err_t sensorarrayPanicDiagnosticsFormat(const char *command, char *response, size_t size)` [214](sensorarrayPanicDiagnostics.c#L214) | 生成当前/原 fault 身份、寄存器或分页详情；无有效记录时显式输出 unknown。 / Return the requested retained diagnostic page and distinguish its origin from the querying boot. | `sensorarrayEarlyRuntimeQueryCommand` / `sensorarrayRuntimeQueryCommand` |

### sensorarrayRecoveryControl.c

| C 调用签名 / C signature | 行为及返回 / Behaviour and return | 调用位置 / Called by |
| --- | --- | --- |
| `static const char *sensorarrayRecoverStateName(sensorarrayRecoverState_t state)` [35](sensorarrayRecoveryControl.c#L35) | 返回恢复请求阶段的 wire 名称，用于缓存和状态查询。 / Map a recovery phase to the stable reply label. | `sensorarrayRecoveryControlComplete` / `sensorarrayRecoveryControlFormat` |
| `void sensorarrayRecoveryControlInit(uint64_t sessionId, uint32_t bootId)` [47](sensorarrayRecoveryControl.c#L47) | 初始化本 boot 的恢复控制快照与单请求状态。 / Initialize the recovery request state for the current boot. | `app_main` |
| `esp_err_t sensorarrayRecoveryControlPost(uint32_t level, const sensorarrayConfigAuthority_t *authority, bool known, uint32_t *outId)` [56](sensorarrayRecoveryControl.c#L56) | 保存请求级别及配置身份并预留终态；同时只受理一个恢复。 / Reserve and bind a single recovery request to the specified level and configuration. | `sensorarrayRecoveryPost` |
| `bool sensorarrayRecoveryControlAck(uint32_t id, esp_err_t publishError)` [88](sensorarrayRecoveryControl.c#L88) | 收到该请求的本地 ACK 结果后允许执行或撤销操作。 / Arm or cancel recovery according to the actual local ACK outcome. | `sensorarrayControlReplyPublished` / `sensorarrayControlReplyFailed` |
| `sensorarrayRecoverTake_t sensorarrayRecoveryControlTake(const sensorarrayConfigAuthority_t *authority, bool known, sensorarrayRecoverRequest_t *outRequest)` [115](sensorarrayRecoveryControl.c#L115) | 领取下一条已 ACK 且身份可用的恢复操作；旧配置返回 STALE。 / Take an acknowledged recovery only for its bound configuration. | `sensorarrayRecoveryTake` |
| `esp_err_t sensorarrayRecoveryControlComplete(uint32_t id, sensorarrayRecoverState_t outcome, const char *reason, esp_err_t originalError, esp_err_t cleanupError, const sensorarrayConfigAuthority_t *authority, bool known, bool routeApplied, bool routeKnown, bool routeVerified)` [140](sensorarrayRecoveryControl.c#L140) | 缓存恢复终态、首错和清理证据，并发布给优先生命周期路径。 / Cache and publish the recovery terminal with its separate first and cleanup errors. | `sensorarrayRecoveryTake` / `sensorarrayCompleteRecoveryRequest` |
| `esp_err_t sensorarrayRecoveryControlFormat(char *response, size_t responseSize)` [201](sensorarrayRecoveryControl.c#L201) | 将活动请求和最后终态格式化为无副作用的 RECOVERY? 快照。 / Return a bounded, side-effect-free snapshot of active and last recovery results. | `sensorarrayEarlyRuntimeQueryCommand` / `sensorarrayRuntimeQueryCommand` |

### sensorarrayRestartControl.c

| C 调用签名 / C signature | 行为及返回 / Behaviour and return | 调用位置 / Called by |
| --- | --- | --- |
| `static const char *sensorarrayRestartPhaseName(sensorarrayRestartPhase_t phase)` [51](sensorarrayRestartControl.c#L51) | 把重启请求阶段转换为协议标签。 / Name the current restart-control phase for diagnostic replies. | `sensorarrayRestartControlFormat` |
| `void sensorarrayRestartControlInit(uint64_t sessionId, uint32_t bootId)` [62](sensorarrayRestartControl.c#L62) | 用当前启动身份初始化独立的手动重启请求状态。 / Initialize the manual-restart request state for this boot. | `app_main` |
| `static esp_err_t sensorarrayRestartControlPostTarget(const sensorarrayConfigAuthority_t *authority, bool known, uint32_t *outId, bool romDownload)` [71](sensorarrayRestartControl.c#L71) | 为应用重启或 ROM 下载目标预留唯一请求和终态发布容量。 / Reserve a single restart intent with its application or downloader target. | `sensorarrayRestartControlPost` / `sensorarrayRestartControlPostRom` |
| `esp_err_t sensorarrayRestartControlPost(const sensorarrayConfigAuthority_t *authority, bool known, uint32_t *outId)` [94](sensorarrayRestartControl.c#L94) | 提交普通应用重启意图并返回 request ID。 / Post an application-restart request and return its identity. | `sensorarrayRestartPost` |
| `esp_err_t sensorarrayRestartControlPostRom(const sensorarrayConfigAuthority_t *authority, bool known, uint32_t *outId)` [98](sensorarrayRestartControl.c#L98) | 提交目标为 ROM 下载的请求并返回 request ID。 / Post a ROM download handoff under a distinct request ID. | `sensorarrayUsbControlHandle` |
| `bool sensorarrayRestartControlIsRom(uint32_t id)` [102](sensorarrayRestartControl.c#L102) | 根据已接受请求号核对本次重启是否指向下载 ROM。 / Check whether the matching accepted request targets the ROM. | `sensorarrayPerformManualRestart` |
| `bool sensorarrayRestartControlAck(uint32_t id, esp_err_t publishError)` [112](sensorarrayRestartControl.c#L112) | 依据本地 RACK 发布结果开放或撤销该重启意图。 / Arm or cancel the restart according to local RACK publication. | `sensorarrayControlReplyPublished` / `sensorarrayControlReplyFailed` |
| `bool sensorarrayRestartControlTake(uint32_t *outId)` [131](sensorarrayRestartControl.c#L131) | 在边界领取已 ACK 的重启请求到调用方，防止重复执行。 / Take the acknowledged restart at the boundary for single execution. | `sensorarrayRestartTake` |
| `esp_err_t sensorarrayRestartControlPublish(uint32_t id, const sensorarrayConfigAuthority_t *authority, bool known)` [142](sensorarrayRestartControl.c#L142) | 在重启前缓存意图及本地队列提交证据并尝试发布终态。 / Publish and cache the restart intent and local queue evidence before reboot. | `sensorarrayPerformManualRestart` |
| `esp_err_t sensorarrayRestartControlFormat(char *response, size_t responseSize)` [196](sensorarrayRestartControl.c#L196) | 返回 RESTART? 快照，保留各发布层级的结果和未知状态。 / Format the restart request, terminal and per-path publication state for read-only recovery. | `sensorarrayEarlyRuntimeQueryCommand` / `sensorarrayRuntimeQueryCommand` |

### sensorarraySnapshotDiagnostics.c

| C 调用签名 / C signature | 行为及返回 / Behaviour and return | 调用位置 / Called by |
| --- | --- | --- |
| `bool sensorarraySnapshotDiagnosticsHandles(const char *command)` [8](sensorarraySnapshotDiagnostics.c#L8) | 匹配 SNAPSRC? 或 SNAPNET?，分别查看源端和 transport 的保护帧状态。 / Route the source and network protected-snapshot diagnostic pages. | `sensorarrayLinkDiagnosticsHandles` / `sensorarrayLinkDiagnosticsFormat` |
| `esp_err_t sensorarraySnapshotDiagnosticsFormat(const char *command, char *response, size_t size)` [14](sensorarraySnapshotDiagnostics.c#L14) | 从异步输出和网络保护帧计数构造快照状态回复。 / Format snapshot protection counters from asynchronous logging and transport. `ESP_ERR_INVALID_ARG` `ESP_ERR_NOT_FOUND` `ESP_ERR_INVALID_SIZE` | `sensorarrayLinkDiagnosticsFormat` |

### sensorarrayUsbControl.c

| C 调用签名 / C signature | 行为及返回 / Behaviour and return | 调用位置 / Called by |
| --- | --- | --- |
| `const char *sensorarrayUsbControlCapabilities(void)` [9](sensorarrayUsbControl.c#L9) | 返回已编入固件的 USB 控制能力字符串。 / Return the USB control capabilities compiled into the firmware. | `sensorarrayEarlyRuntimeQueryCommand` / `sensorarrayRuntimeQueryCommand` |
| `bool sensorarrayUsbControlHandles(const char *c)` [16](sensorarrayUsbControl.c#L16) | 识别由 USB 控制模块处理的命令字符串。 / Recognise command strings handled by USB control. | `sensorarrayEarlyRuntimeQueryCommand` / `sensorarrayRuntimeQueryCommand` |
| `esp_err_t sensorarrayUsbControlHandle(const char *command, char *response, size_t size)` [22](sensorarrayUsbControl.c#L22) | 按命令查询或修改 USB 输出配置并写入有容量限制的文本回复。 / Query or change USB output configuration and write a bounded reply. `ESP_ERR_INVALID_ARG` `ESP_ERR_INVALID_SIZE` `ESP_ERR_NOT_SUPPORTED` | `sensorarrayEarlyRuntimeQueryCommand` / `sensorarrayRuntimeQueryCommand` |

## 参数索引 / Parameter index

相同名称按模块共享解释。完整类型、数组维度与输入输出方向以对应函数签名和调用点为准。指针参数所需存储由调用者提供，异步作业另受上面的所有权约束。

Entries share parameter descriptions within this module. Refer to the signature and its caller for the precise type, array shape and direction. The caller supplies pointer storage, subject to the ownership rules above for asynchronous work.

| 参数 / Parameter | 含义 / Meaning |
| --- | --- |
| `alignment` | 异常上下文中允许读指针所需的字节对齐。 / Required byte alignment for panic-time pointer validation. |
| `authority` | 帧的不可变配置身份或同一版本的完整权威快照。 / Immutable frame configuration identity or the authority snapshot for that revision. |
| `avddUv`, `avssUv` | 模拟供轨、矩阵激励或转换参考，单位 µV。须保留参考点及来源。 / Analogue rail, matrix excitation or ADC reference in µV. Retain its reference node and provenance. |
| `bootId`, `sessionId` | 会话、启动身份或启动计数，查询与终态依赖所属启动域。 / Session identifier, boot identifier or boot count scoping queries and terminal outcomes. |
| `bytes`, `record` | 长度由签名给出；异常路径使用受控范围的原始字节或保留记录。 / Bounded bytes or retained panic record as specified by the signature. |
| `c` | 待匹配的完整 USB 命令字符串。 / Complete USB control command to classify. |
| `caps` | 内存分配请求的 ESP-IDF capability 位掩码。 / Capability mask of the failed allocation. |
| `cleanupError`, `error`, `originalError` | 操作、首个根因或独立清理结果的错误码。保留调用层提供的具体错误。 / Error for the operation, first cause or separate cleanup, preserving the code from the reporting layer. |
| `cmd`, `command`, `text` | 按该函数声明传入的文本、命令、操作码或数据字节，显式 length 限定可读范围。 / Text, command, opcode or data bytes as declared. Where supplied, length bounds the readable range. |
| `diagnostic` | 区分操作员诊断请求与普通电池采样。 / Mark battery sampling as operator-requested diagnostics. |
| `elfSha256` | 正在运行的固件 ELF 摘要，用于跨重启关联诊断。 / Running ELF digest for retained diagnostic correlation. |
| `enabled`, `known` | 表示证据已知、数据有效、功能启用或操作成功的布尔标志，按参数名独立解释。 / Boolean representing known evidence, validity, enablement or success as named. |
| `functionName` | 失败分配钩子收到的 SDK 函数名称指针。 / SDK function-name pointer received by the allocation hook. |
| `id`, `outId`, `outRequestId` | 本地请求或事务标识，out 参数接收新分配或选中的 ID。 / Local request or transaction identifier. An out parameter receives the allocated or selected ID. |
| `info` | IDF 提供的 panic_info_t，上下文仅在异常处理期间有效。 / IDF panic information available to the exception handler. |
| `keepResult` | 是否将 work 区中的结果保存为终态对应的分页查询数据。 / Retain the workspace result for paged reads of this request. |
| `kind` | ADC 诊断类型，决定 timing 或 tail 分组。 / ADC diagnostic kind selecting timing or final-bit groups. |
| `length` | 输入、载荷、底层存储或读写传输的字节数。 / Byte length of input, payload, backing storage or transfer. |
| `level` | 请求的 RECOVER 恢复级别。 / Requested RECOVER level. |
| `maximum` | 无符号输入可接受的最大值，包含端点。 / Inclusive upper bound for parsed unsigned input. |
| `mode` | 对应调用域的模式枚举，保留目标、当前帧或恢复上下文的含义。 / Mode enum in the declared domain, selecting the target, frame or recovery mode as indicated. |
| `nowUs` | 单调时钟的事件时间，单位 µs。 / Event time on the monotonic clock in µs. |
| `outcome` | 写入诊断或恢复终态的结果标签。 / Outcome label to retain in the terminal result. |
| `outCommand` | 接收结果。按该函数声明传入的文本、命令、操作码或数据字节，显式 length 限定可读范围。 / Output. Text, command, opcode or data bytes as declared. Where supplied, length bounds the readable range. |
| `output` | 用于输出原始字节 hex 表示的调用者缓冲。 / Caller buffer for the hexadecimal representation. |
| `outRecoveryLevel` | 接收早期恢复命令解析出的级别。 / Receives the level parsed from the early recovery command. |
| `outRequest` | 接收与已确认请求 ID 对应的持久请求副本。 / Destination for the admitted request snapshot. |
| `outResult`, `result` | 函数签名注明的结果、快照或统计结构，调用者分配复制目标。 / Result, snapshot or statistics object of the declared type, with caller-owned output storage. |
| `outValue` | 接收通过语法和范围检查的无符号参数。 / Receives the parsed in-range unsigned value. |
| `periodMs` | 请求的电池采样间隔，单位为毫秒。 / Requested battery sampling interval in milliseconds. |
| `phase` | 待显示的重启控制阶段枚举。 / Restart-control phase to format. |
| `pointer` | 异常处理程序准备读取的地址，须先核验内部地址范围。 / Address to validate before panic-time access. |
| `prefix` | 被解析的帧率控制前缀，用以区分 capture 和 output。 / Literal prefix identifying the rate control being parsed. |
| `profile` | 配置档或八个物理行的模式列表。outModes 接收解析结果。 / Configuration profile or mode selection for the eight physical rows. outModes receives parsed values. |
| `publishError` | 本地 ACK 发布返回码，决定是否开放硬件操作。 / Local ACK publication result gating the action. |
| `reason` | 操作或拒绝原因字符串；outReason 用于返回原因。 / Operation/rejection reason string. outReason receives the reason. |
| `requestedSize` | 分配器报告失败请求的字节数。 / Requested byte count captured by the allocator-failure hook. |
| `requestId` | 对应配置、恢复触发或 profile 域内的请求标识，由接收方与提交时身份配对。 / Request identifier for the indicated configuration, recovery trigger or row profile, paired with the identity at admission. |
| `resetReason` | 此次启动的 IDF reset reason，用于关联保留的异常。 / IDF reset cause for the querying boot. |
| `response` | 调用者提供的回复、暂存或结果缓冲区，使用随同的容量/类型。 / Caller-provided reply, scratch or result storage with the accompanying capacity/type. |
| `responseSize` | 字符输出缓冲区总字节数，含结尾 NUL 的空间。 / Total capacity of the text buffer in bytes, including space for its final NUL. |
| `romDownload` | 为真时重启目标为下载 ROM，为假时为应用。 / Select the ROM downloader or the application reboot target. |
| `routeApplied`, `routeKnown`, `routeVerified` | 分别表示控制动作已执行、状态已知、核验通过的独立条件。 / Separate evidence that route controls were applied, are known, or passed verification. |
| `rows` | 行数值或行配置快照，由该函数的参数类型决定。 / Active count or row-configuration snapshot as specified by its declared type. |
| `sampleCount` | 本次操作允许的尝试、样本或重试数量。 / Attempt, sample or retry count budgeting this operation. |
| `samplesPerGroup` | 每个诊断组的主读数量；协议版 timing/tail 要求 200。 / Primary reads per diagnostic group; the timing/tail commands require 200. |
| `session` | 命令受理或诊断存储所属的启动会话标识。 / Boot-session nonce binding diagnostic storage or request admission. |
| `settleUs` | 应用到 RES 路由的建立时间，单位为 µs。 / Requested resistance-route settling interval in microseconds. |
| `signedValue` | 第二个有符号命令参数，允许承载负电源轨等负值。 / Signed request argument, including negative values where supported. |
| `size` | 调用者提供的可写空间，文本/字节缓冲按字节计，数组按本函数的元素类型计。 / Capacity supplied by the caller. Text/byte buffers use bytes and typed arrays use elements. |
| `stackBytes` | 将创建的扫描任务所申请的栈字节数。 / Requested stack size in bytes for the scan task allocation observation. |
| `state` | 所声明类型的状态对象或枚举。传入前端状态指针时，其生命周期覆盖总线或采集操作。 / State object or enum of the declared type. A frontend-state pointer must outlive the bus or capture operation. |
| `type` | 解析或预留的邮箱命令枚举，决定接收者执行的操作。 / Mailbox operation kind controlling subsequent dispatch. |
| `value` | 文本解析位置处的数值或已解析的整数操作参数，以签名区分。 / Numeric input string or parsed operation argument, as declared. |

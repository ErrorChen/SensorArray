# 应用启动、采集与诊断 / Application startup, acquisition and diagnostics

ESP-IDF 从 app_main 启动。它先初始化启动身份、早期诊断和受控重启接口，再建立输出与控制面及固定的采集任务。应用上下文持有双 FDC 引擎、ADS 引擎、混合工作区、模式/行配置和路由/恢复状态。采集循环只在合适的帧边界执行配置与辅助工作，再把帧的身份随结果交给异步输出。

ESP-IDF enters at app_main. It establishes boot identity, early diagnostics and controlled restart, then brings up output, control ingress and the acquisition task. The application context owns the FDC and ADS engines, mixed-frame workspace, mode and row state, routes and recovery. The capture loop admits configuration and auxiliary work at eligible frame boundaries and hands capture identity and values to asynchronous output.

[控制 / Control](control/README.md) · [输出与编码 / Output and encoding](output/README.md) · [网络适配 / Network adapter](net/README.md) · [测量引擎 / Measurement engines](../core/measure/README.md)

## 调用方式 / Calling the module

```c
esp_err_t err = sensorarrayCommandMailboxInit();
```

应用启动和硬件操作由固件任务调用。示例建立控制邮箱，入队后由采集侧执行，ACK、Applied 和首帧分开表示。采集使用 1..8 行、每行 8 列。sensorarrayTypes.h 定义公共类型与矩阵索引，sensorarrayConfig.h 汇总编译配置，sensorarrayConfigDerived.h 计算派生项目。Kconfig.projbuild 引入 kconfigs 下的板级、运行、输出、日志、专家及兼容选项。

Firmware tasks own startup and hardware operations. The example initialises the command mailbox. Acquisition executes queued requests and reports acceptance, application and first data separately. A capture has 1..8 rows of eight columns. sensorarrayTypes.h defines shared types and matrix indexing. sensorarrayConfig.h gathers build settings and sensorarrayConfigDerived.h derives dependent settings. Kconfig.projbuild includes the board, runtime, output, logging, expert and compatibility menus in kconfigs.

## 源码和头文件 / Source and headers

| 文件 / File | 职责与声明 / Responsibility and declarations |
| --- | --- |
| [main.c](main.c) | 拥有整个启动和采集生命周期，协调前端、控制、恢复、身份发布、异步日志及运行性能。 / Owns startup and capture lifetime and coordinates frontends, controls, recovery, identity publication, asynchronous logging and runtime performance. |
| [sensorarrayConfig.h](sensorarrayConfig.h) | 汇总 Kconfig 派生的应用层编译选项。 / Collects application build options derived from Kconfig. |
| [sensorarrayConfigDerived.h](sensorarrayConfigDerived.h) | 从芯片与板级选项派生编译期配置与边界。 / Derives compile-time settings and bounds from board/chip options. |
| [sensorarrayLog.c](sensorarrayLog.c) · [sensorarrayLog.h](sensorarrayLog.h) | 格式化板级参考、开关、电池、矩阵点与 I²C 诊断，供启动与采集状态记录。 / Formats board reference, switch, battery, matrix-cell and I²C diagnostics for startup and acquisition reporting. |
| [sensorarrayTypes.h](sensorarrayTypes.h) | 定义跨引擎帧、掩码与位置类型，将 1 起始的 S/D 坐标映射为行优先下标。 / Defines shared frame/mask and coordinate types and maps one-based S/D coordinates to row-major indices. `sensorarrayFdcDeviceId_t` `sensorarrayPath_t` |

## 函数 / Functions

签名按源文件排列，参数顺序即调用顺序。下表标出生产代码中的调用者或回调注册者（最多两个）。`static` 限于所在编译单元。错误码列于对应行为后，只列函数直接返回的常量；硬件与下层调用可传播其他错误。

Signatures are grouped by source in call-argument order. The last column names up to two production callers or callback registrars. `static` entries are local to their translation unit. Constants after a description are explicit error returns. Underlying operations may propagate additional errors.

[main.c](#mainc) · [sensorarrayLog.c](#sensorarraylogc) · [sensorarrayTypes.h](#sensorarraytypesh)

### main.c

| C 调用签名 / C signature | 行为及返回 / Behaviour and return | 调用位置 / Called by |
| --- | --- | --- |
| `static bool sensorarrayFrontendsHeld(void)` [201](main.c#L201) | 以 acquire 顺序读取跨任务的暂停标志；返回 true 时前端尚不可接受普通采样。 / Read the frontend hold flag with acquire ordering; true gates acquisition. | `sensorarrayFrontendRowsRequest` / `sensorarrayApplyPendingCommands` |
| `static esp_err_t sensorarrayFrontendRowsRequest(uint8_t rows, sensorarrayScanConfigRequestResult_t *outResult)` [206](main.c#L206) | 拒绝暂停期间的行数变更，否则交给共用邮箱并将接受结果写入 outResult。 / Reject row changes during frontend hold or submit the count to the shared mailbox and return its result. | `sensorarrayInitRuntime` |
| `static const char *sensorarrayResetReasonName(esp_reset_reason_t reason)` [288](main.c#L288) | 把 IDF 复位枚举映射为 BOOT/RST 使用的稳定字符串。 / Map the IDF reset-reason enum to the string used in BOOT and RST. | `sensorarrayBootBreadcrumbStart` / `sensorarrayFormatBootReply` |
| `static void sensorarrayBootBreadcrumbSetStage(const char *stage, esp_err_t err, const sensorarrayAppContext_t *ctx)` [326](main.c#L326) | 在受保护的启动记录中更新阶段、错误、heap 和已知帧序号。 / Record stage, error and available progress counters under the breadcrumb lock. | `sensorarrayBootBreadcrumbStart` / `sensorarrayEnterFrontendHold` |
| `static void sensorarrayBootBreadcrumbStart(void)` [350](main.c#L350) | 读取保留记录及本次 reset reason，开始新的启动记录并报告前次阶段与重启来源。 / Open a new boot breadcrumb from retained state and report the previous stage and reset origin. | `app_main` |
| `static bool sensorarrayBootBreadcrumbCopy(sensorarrayBootBreadcrumb_t *outCopy)` [417](main.c#L417) | 在锁内复制启动记录并校验 magic/version；有效时返回 true。 / Copy the retained record under lock and validate its magic and format version. | `sensorarrayEmitAdsFault` / `sensorarrayPublishConfigAuthority` |
| `static void sensorarrayAdsFaultSink(const char *line, size_t length, void *context)` [429](main.c#L429) | formatter 的同步输出回调。line 为 NUL 结尾文本，经 printf 输出；length 和 context 在此 sink 中未使用。 / Print the formatter's terminated text synchronously. This sink ignores length and context. | `sensorarrayEmitAdsFault` |
| `static void sensorarrayAdsFaultFillContext(sensorarrayAppContext_t *ctx, sensorarrayAdsFaultEvent_t *event)` [438](main.c#L438) | 从帧身份或对应的活跃快照补入故障前的模式、路由、寄存器代数、参考和 rail 来源。 / Enrich the fault using frame identity or the relevant live snapshots before cleanup changes state. | `sensorarrayEmitAdsRecoveryFault` / `sensorarrayRunAdsTimingCommand` |
| `static void sensorarrayEmitAdsFault(sensorarrayAppContext_t *ctx, sensorarrayAdsFaultEvent_t *event)` [497](main.c#L497) | 关联 boot、首错上下文和单调时间，调用有界格式化器同步发布，随后撤销临时上下文指针。 / Attach boot and engine context for bounded synchronous emission, then release the borrowed context pointer. | `sensorarrayEmitAdsRecoveryFault` / `sensorarrayRunAdsTimingCommand` |
| `static bool sensorarrayAdsRegisterCacheIsVerified(const sensorarrayAdsRegisterCache_t *cache)` [525](main.c#L525) | 要求 Vref、PGA 和运行状态均已知，并验证每个寄存器属于当前 cache generation。 / Require known Vref/PGA/run state and a verified entry in the current generation for every register. | `sensorarrayRunAdsRestoreRecoveryAttempt` |
| `static const char *sensorarrayAdsRecoveryOwner(const sensorarrayMeasurementRecovery_t *recovery)` [543](main.c#L543) | 按恢复触发源选 battery 或 ADSCHK owner 名称，供故障记录使用。 / Select the battery or active-check owner label for a restore fault. | `sensorarrayEmitAdsRecoveryFault` |
| `static void sensorarrayEmitAdsRecoveryFault(sensorarrayAppContext_t *ctx, sensorarrayAdsFaultStage_t stage, uint32_t attempt, sensorarrayAdsFaultOutcome_t outcome, esp_err_t err, uint32_t sequence)` [553](main.c#L553) | 把阶段、尝试次数、结果、原始错误及序号合成恢复事件并保留调用现场。 / Emit a recovery event with stage, attempt, outcome, first error and capture sequence. | `sensorarrayBeginAdsRestoreRecovery` / `sensorarrayFinishAdsRestoreRecovery` |
| `static bool sensorarrayBeginAdsRestoreRecovery(sensorarrayAppContext_t *ctx, sensorarrayMeasurementRecoveryTrigger_t trigger, esp_err_t error, uint32_t triggerRequestId)` [578](main.c#L578) | 锁存恢复触发和目标模式，将状态置为恢复中，申请 SAFE 并使 ADS 缓存失效。 / Latch the trigger and prior mode, publish recovery state, request SAFE and invalidate cached ADC data. | `sensorarrayRunPendingAdsCheck` / `sensorarrayRunBatteryBoundary` |
| `static void sensorarrayBootBreadcrumbMarkStableProgress(bool healthyFrame)` [617](main.c#L617) | 以连续健康帧的个数和时间跨度重授自动重启预算；间断或坏帧重新计时。 / Replenish the automatic restart budget only after sustained healthy frame progress. | `sensorarrayEnterFrontendHold` / `sensorarrayRunFrontendPresenceBoundary` |
| `static bool sensorarrayBootPrepareRestart(uint32_t restartKind)` [660](main.c#L660) | 在保留状态中记录重启类别，对自动重启检查并消耗预算，返回是否准许。 / Record the restart kind, check and charge the automatic restart budget, and return admission. | `sensorarrayRunFrontendPresenceBoundary` / `sensorarrayPerformManualRestart` |
| `static void sensorarrayBootEmitRestartMarker(const char *marker)` [692](main.c#L692) | 将传入的重启标记交给本地串口队列并短暂让出发送时间。 / Queue the caller's restart marker locally and allow a bounded publication opportunity. | `sensorarrayRunFrontendPresenceBoundary` / `sensorarrayScanTask` |
| `static bool sensorarrayRecoveryPost(uint32_t level, uint32_t *outRequestId)` [704](main.c#L704) | 用当前权威配置提交指定级别的 RECOVER，向 outRequestId 返回关联号。 / Submit the requested recovery level along with configuration authority and return the request ID. | `sensorarrayEarlyRuntimeQueryCommand` / `sensorarrayRuntimeQueryCommand` |
| `static bool sensorarrayRecoveryTake(uint32_t *outLevel, uint32_t *outRequestId)` [712](main.c#L712) | 将已接受恢复与当前身份核对，返回可执行的 level/id，已过期的请求产生独立终态。 / Take an admitted recovery only for its matching configuration and terminate a stale request. | `sensorarrayRunRecoveryBoundary` / `sensorarrayScanTask` |
| `static bool sensorarrayRestartPost(uint32_t *outRequestId)` [731](main.c#L731) | 为手动 RESTART 保存当前身份，返回分配的请求号。 / Submit a manual restart scoped to the current authority and return the allocated ID. | `sensorarrayEarlyRuntimeQueryCommand` / `sensorarrayRuntimeQueryCommand` |
| `static bool sensorarrayRestartTake(uint32_t *outRequestId)` [738](main.c#L738) | 从独立的重启控制状态领取可以执行的 request ID。 / Take an executable request ID from the restart control state. | `sensorarrayRunRecoveryBoundary` / `sensorarrayScanTask` |
| `static const char *sensorarrayAppFdcBootQualityName(sensorarrayFdcBootQuality_t quality)` [743](main.c#L743) | 将 FDC 启动质量映射为 ok/degraded/fail，供启动诊断输出。 / Format the FDC boot quality for startup diagnostics. | `sensorarrayRunBootCalibration` / `sensorarrayScanTask` |
| `static void sensorarrayLogRuntimeMemoryDiag(const char *stage, const sensorarrayAppContext_t *ctx)` [756](main.c#L756) | 在给定启动阶段记录 context 大小、任务剩余栈和当前/最低 heap。 / Log context size and task stack/heap observations at the named stage. | `sensorarrayRunQueuedFullSweep` / `sensorarrayRunMainLoop` |
| `static void sensorarrayLogRuntimeMemorySummary(const sensorarrayAppContext_t *ctx, uint32_t freeBefore, uint32_t minFreeBefore)` [774](main.c#L774) | 以初始化前记录的 heap 为参照，输出 runtime 分配后的内存和栈摘要。 / Compare post-initialization memory with the supplied pre-init observations. | `sensorarrayInitRuntime` |
| `static void sensorarrayLogStackHighWater(const char *stage)` [792](main.c#L792) | 对当前任务读取栈高水位并随 stage 输出 APP_STACK。 / Tag the current task's stack high-water observation with its stage. | `sensorarrayRunBootCalibration` / `sensorarrayRunQueuedFullSweep` |
| `static uint32_t sensorarrayFrameTargetFps(sensorarrayMeasurementMode_t mode)` [800](main.c#L800) | 返回邮箱设置的采集 FPS 上限；mode 参数保留用于调用接口。 / Return the runtime capture cap from the mailbox; mode is retained in the interface. | `sensorarrayFramePeriodMs` / `sensorarrayRuntimeQueryCommand` |
| `static uint32_t sensorarrayFrameBudgetFps(sensorarrayMeasurementMode_t mode)` [808](main.c#L808) | 根据 CAP/VOLT/RES 模式选择内部预算所用的目标率。 / Select the configured mode-specific rate used for internal timing budgets. | `sensorarrayRuntimeQueryCommand` |
| `static uint32_t sensorarrayFramePeriodMs(sensorarrayMeasurementMode_t mode)` [818](main.c#L818) | 将运行时上限换算成毫秒周期；未开启上限时返回零。 / Convert the runtime cap to a period in milliseconds, returning zero for uncapped capture. | `sensorarrayDelayFramePeriodSince` |
| `static uint32_t sensorarrayFdcReferenceFramePeriodMs(void)` [828](main.c#L828) | 从 FDC 目标率计算参考周期，保证至少 1 ms。 / Compute the diagnostic FDC reference period, with a one-millisecond floor. | `sensorarrayDelayFramePeriodSince` |
| `static void sensorarrayDelayFramePeriodSince(sensorarrayAppContext_t *ctx, int64_t frameStartUs, uint32_t sequence, sensorarrayMeasurementMode_t mode)` [834](main.c#L834) | 用实际帧起点计算剩余等待，必要时记录超周期；仅按适用上限节拍。 / Pace from the given frame start and report overruns against the applicable period. | `sensorarrayRunMainLoop` |
| `static bool sensorarrayFrameRawAllZero(const sensorarrayFrame_t *frame)` [871](main.c#L871) | 仅在全矩阵均 fresh、硬件 raw 全零且没有未就绪解释时置 true。 / Recognize a fully fresh all-zero hardware frame independently of premature zero reads. | `sensorarrayRunMainLoop` |
| `static bool sensorarrayFdcWaveDebugEnabled(void)` [880](main.c#L880) | 汇总编译时波形调试选项，决定是否进入专门的测量循环。 / Check build-time wave-debug selections before entering the diagnostic loop. | `sensorarrayScanTask` |
| `static const char *sensorarrayFdcWaveDebugModeName(void)` [889](main.c#L889) | 返回已选择的路由、固定行、单器件等波形实验名称。 / Name the compiled wave-debug route or device selection. | `sensorarrayRunFdcWaveDebugLoop` |
| `static esp_err_t sensorarrayFdcWaveDebugSetSleep(sensorarrayFdcDeviceState_t *fdcState, bool sleepEnabled)` [909](main.c#L909) | 对可用 FDC 执行 write-only 进入/退出 sleep，返回驱动错误。 / Apply a write-only sleep transition to the available device and propagate its result. | `sensorarrayRunFdcWaveDebugLoop` |
| `static esp_err_t sensorarrayFdcWaveDebugReadDevice(sensorarrayFdcDeviceState_t *fdcState, sensorarrayFdcDeviceId_t devId, bool singleChannel)` [920](main.c#L920) | 设置被观测的 FDC 读窗口，根据 singleChannel 选单通道或四通道 burst，返回总线结果。 / Mark the diagnostic read window, then perform either one relaxed channel read or a four-channel burst. | `sensorarrayRunFdcWaveDebugLoop` |
| `static void sensorarrayFdcMapVerifyDebug(void)` [944](main.c#L944) | 遍历 D 线，将预期器件/通道与板级映射逐项比对并报告。 / Compare each D-line mapping with its expected device and channel and log discrepancies. | `sensorarrayScanTask` |
| `static void sensorarrayRunFdcWaveDebugLoop(sensorarrayAppContext_t *ctx)` [974](main.c#L974) | 配置选行、时序 GPIO 及 FDC 读取方式，循环产生可供示波器观察的标记。 / Drive the selected diagnostic route and FDC read sequence with scope timing strobes. | `sensorarrayScanTask` |
| `static void sensorarrayApplyTmuxDefaults(sensorarrayState_t *state)` [1089](main.c#L1089) | 按板级定义初始化 TMUX1108 行/公共端、SELA/B 和 TMUX1134 enable 并记录结果。 / Apply board-defined multiplexer defaults and report switch setup status. | `sensorarrayInitBoardAndRouting` |
| `static esp_err_t sensorarrayInitRuntime(sensorarrayAppContext_t *ctx)` [1124](main.c#L1124) | 分配持久工作区并初始化模式、恢复、profile、邮箱与自检；初始化失败返回相应错误。 / Allocate runtime workspace and initialize state machines, mailbox and self-tests before scanning. | `sensorarrayInitSystem` |
| `static bool sensorarrayBuildAdsRailSplit(sensorarrayAppContext_t *ctx, sensorarrayAdsRailSplit_t *outRail)` [1245](main.c#L1245) | 按下一帧序号与 rail 来源策略复制可用电源轨到 outRail；无合法 split 返回 false。 / Obtain a usable rail split for the next frame under the source-specific freshness policy. | `sensorarrayRefreshVoltageRailIfNeeded` / `sensorarrayApplyMeasurementMode` |
| `static esp_err_t sensorarrayRefreshVoltageRailIfNeeded(sensorarrayAppContext_t *ctx)` [1264](main.c#L1264) | 判断当前布局、参考是否已可用和板级隔离能力，符合条件才执行内部刷新并恢复原路由。 / Check mode, reference freshness and isolation capability before a bounded rail refresh and route restoration. | `sensorarrayRunMainLoop` |
| `static void sensorarrayPublishLastMeasurement(sensorarrayAppContext_t *ctx)` [1330](main.c#L1330) | 向并发只读查询发布最后完成的一帧测量记录。 / Publish the completed measurement snapshot for concurrent cell queries. | `sensorarrayRunOneFrame` |
| `static bool sensorarrayCopyLastMeasurementCell(const sensorarrayAppContext_t *ctx, uint8_t row, uint8_t dLine, sensorarrayMeasurementCellSnapshot_t *outSnapshot)` [1342](main.c#L1342) | 检查物理行列边界并一致复制上一帧对应的点、错误、增益和身份到 outSnapshot。 / Validate the physical coordinate and copy the corresponding last-frame evidence consistently. | `sensorarrayRuntimeQueryCommand` |
| `static sensorarrayConfigIdentity_t sensorarrayPublishConfigAuthority(sensorarrayAppContext_t *ctx, uint32_t frameBoundary)` [1398](main.c#L1398) | 读取三个配置域，捕获同一会话与边界并公布一次权威快照。 / Read all three configuration domains, capture their session and boundary and publish one authority snapshot. | `sensorarrayBeginAdsRestoreRecovery` / `sensorarrayApplyMeasurementMode` |
| `static void sensorarrayCaptureFrameIdentity(sensorarrayAppContext_t *ctx, const sensorarrayMeasurementModeSnapshot_t *mode, const sensorarrayRowModeProfile_t *profile, uint32_t frameBoundary, sensorarrayConfigIdentity_t *outIdentity)` [1418](main.c#L1418) | 保存本帧采集开始时的配置及边界，供后续帧发布使用。 / Snapshot configuration and boundary at capture for later publication of that frame. | `sensorarrayRunOneFrame` |
| `static esp_err_t sensorarrayApplyMeasurementMode(sensorarrayAppContext_t *ctx, sensorarrayMeasurementMode_t targetMode, uint32_t requestId, bool emitEvent)` [1441](main.c#L1441) | 执行模式切换所需的路由、引擎重配与核验并保存耗时和错误。 / Apply the mode's route and engine settings, verify them and retain elapsed time and errors. `ESP_ERR_INVALID_ARG` `ESP_ERR_INVALID_STATE` | `sensorarrayApplyPendingCommands` / `sensorarrayRunAdsRestoreRecoveryAttempt` |
| `static void sensorarrayEmitConfigTerminal(sensorarrayAppContext_t *ctx, uint32_t requestId, uint32_t frameBoundary, const char *format, ...)` [1556](main.c#L1556) | 在终态文本中附加 session/boot/revision 与应用边界，按请求号交给邮箱的终态发布路径。 / Append the committed identity and boundary and publish the terminal through its request slot. | `sensorarrayRejectHeldCommand` / `sensorarrayApplyPendingCommands` |
| `static void sensorarrayRejectHeldCommand(sensorarrayAppContext_t *ctx, const sensorarrayCommand_t *command)` [1592](main.c#L1592) | 前端暂停时按命令域拒绝变更并发出带原请求号的终态。 / Reject a command during hold and record the domain-specific terminal against its request ID. | `sensorarrayApplyPendingCommands` |
| `static void sensorarrayApplyPendingCommands(sensorarrayAppContext_t *ctx)` [1623](main.c#L1623) | 在帧边界处理可领命令，包括配置、校准和诊断；把各自结果归还请求域。 / Drain eligible mailbox actions at a frame boundary, including calibration and diagnostic jobs. | `sensorarrayRunFrontendPresenceBoundary` / `sensorarrayRunMainLoop` |
| `static bool sensorarrayFrontendOwnersQuiescent(const sensorarrayAppContext_t *ctx)` [1974](main.c#L1974) | 汇总 ADS 选速/探测、驱动隔离和双 FDC worker 所有权，检查能否重新进入硬件。 / Require idle, nonquarantined frontend owners before entering a new hardware transaction. | `sensorarrayRunAdsTimingCommand` / `sensorarrayEnterFrontendHold` |
| `static void sensorarrayRunAdsTimingCommand(sensorarrayAppContext_t *ctx, const sensorarrayCommand_t *command)` [1988](main.c#L1988) | 按请求身份和路由门禁执行时序/末位实验，保存分页结果和终态；restore 失败进入故障处理。 / Execute a qualified timing probe and cache its paged result and terminal, including restore failure. | `sensorarrayApplyPendingCommands` |
| `static void sensorarrayEnterFrontendHold(sensorarrayAppContext_t *ctx, const char *reason)` [2069](main.c#L2069) | 锁存暂停原因，进入 frontend-wait，处理可安全失效的缓存并停止继续发布普通测量。 / Latch frontend hold, enter the wait route and invalidate safely owned state. | `sensorarrayRunFrontendPresenceBoundary` / `sensorarrayScanTask` |
| `static bool sensorarrayRunFrontendPresenceBoundary(sensorarrayAppContext_t *ctx)` [2145](main.c#L2145) | 在各所有者已知状态下推进三芯片身份检查，按结果暂停、等待或请求受控重启。 / Advance three-device identity checks using known owner state, then pause, wait or request controlled restart. | `sensorarrayRunMainLoop` / `sensorarrayScanTask` |
| `static void sensorarrayRunPendingAdsCheck(sensorarrayAppContext_t *ctx)` [2221](main.c#L2221) | 运行一次已排队的 ADS 自检，记录诊断结果；根据恢复结果选择继续、重试或隔离。 / Run the queued active check and dispatch restore failures to recovery or quarantine. | `sensorarrayRunMainLoop` |
| `static void sensorarrayRunBatteryBoundary(sensorarrayAppContext_t *ctx, sensorarrayMeasurementMode_t frameMode)` [2283](main.c#L2283) | 在给定帧模式的边界执行电池事务，分别处理电池有效性、BAPP 与 ADS 恢复错误。 / Run battery sampling at the applicable boundary and separate battery results from restore failures. | `sensorarrayRunMainLoop` |
| `static void sensorarrayFinishAdsRestoreRecovery(sensorarrayAppContext_t *ctx)` [2342](main.c#L2342) | 在恢复失败终结时，将模式置 DEGRADED、申请 SAFE，按 BATTERY/ADSCHK 发布 MFAULT。 / Finalize a failed restore recovery by degrading the mode, requesting SAFE and reporting the triggering owner. | `sensorarrayRunAdsRestoreRecoveryAttempt` |
| `static void sensorarrayRunAdsRestoreRecoveryAttempt(sensorarrayAppContext_t *ctx)` [2374](main.c#L2374) | 领取预算内的一次恢复，主动核验 ADS 及 shadow，再恢复先前模式或保留失败终态。 / Spend one recovery attempt to verify hardware/cache and reapply the saved mode when possible. | `sensorarrayRunMainLoop` |
| `static esp_err_t sensorarrayFormatBootReply(char *response, size_t responseSize, const sensorarrayAppContext_t *ctx)` [2458](main.c#L2458) | 将保留启动记录、复位来源、heap 和 guard 格式化为 BOOT 回复，容量不足返回错误。 / Build a bounded BOOT reply from the retained breadcrumb and live memory observations. | `sensorarrayEarlyRuntimeQueryCommand` / `sensorarrayRuntimeQueryCommand` |
| `static esp_err_t sensorarrayFormatFrontendReply(char *response, size_t responseSize)` [2502](main.c#L2502) | 输出三前端存在性、探测阶段与 hold 快照以供只读查询。 / Format the presence monitor's per-frontend and hold/phase evidence. | `sensorarrayEarlyRuntimeQueryCommand` / `sensorarrayRuntimeQueryCommand` |
| `static esp_err_t sensorarrayFormatSerialReply(char *response, size_t responseSize)` [2535](main.c#L2535) | 从 serial I/O 统计输出本地准入、拒收、过期及短写。 / Return serial I/O counters for local admission, expiry, rejection and short writes. | `sensorarrayEarlyRuntimeQueryCommand` / `sensorarrayRuntimeQueryCommand` |
| `static esp_err_t sensorarrayFormatSchedulerReply(char *response, size_t responseSize)` [2556](main.c#L2556) | 从工作预算快照返回 idle 代数、年龄与调度参数，对缺失观察使用 unknown。 / Format scheduler budgets and observed idle progress with explicit unknowns. | `sensorarrayEarlyRuntimeQueryCommand` / `sensorarrayRuntimeQueryCommand` |
| `static esp_err_t sensorarrayFormatBuildReply(char *response, size_t responseSize)` [2610](main.c#L2610) | 从实际 app descriptor 构造 BUILD 回复，含项目、IDF 和完整 ELF 标识。 / Read the running app descriptor and format image/IDF identity within the reply buffer. | `sensorarrayEarlyRuntimeQueryCommand` / `sensorarrayRuntimeQueryCommand` |
| `static esp_err_t sensorarrayEarlyRuntimeQueryCommand(const char *command, char *response, size_t responseSize, void *context)` [2638](main.c#L2638) | 在测量系统尚未就绪时处理有限启动查询与恢复控制。 / Serve the limited early-control surface while normal measurement state is unavailable. | `app_main` |
| `static void sensorarrayControlReplyPublished(const char *data, size_t length)` [2775](main.c#L2775) | 根据已在本地发布的 ACK 关联 request ID，开放对应配置/恢复/诊断操作。 / Arm the matching pending operation after local ACK publication. | `app_main` |
| `static void sensorarrayControlReplyFailed(esp_err_t error, const char *data, size_t length)` [2804](main.c#L2804) | 将本地 ACK 失败反馈给相应管理器，撤销尚未开放的操作。 / Cancel or disarm the pending action when its acknowledgement cannot be published. | `app_main` |
| `static esp_err_t sensorarrayRuntimeQueryCommand(const char *command, char *response, size_t responseSize, void *context)` [2835](main.c#L2835) | 分派运行期查询及 setter，将同步回复和待帧边界执行的工作分别交给其所有者。 / Dispatch runtime commands to bounded snapshot queries or their owning action mailboxes. | `sensorarrayInitFrontends` |
| `static esp_err_t sensorarrayInitBoardAndRouting(sensorarrayAppContext_t *ctx)` [3649](main.c#L3649) | 初始化总线和模拟开关、审查板级映射、应用启动路由，并传播首次初始化错误。 / Bring up board support and switches, audit mapping and apply initial routing. | `sensorarrayInitSystem` |
| `static bool sensorarrayCalibrationMatrixPayloadValid(const void *payload, size_t payloadLength)` [3680](main.c#L3680) | 校验待装载矩阵校准载荷的类型/内容是否满足 ADS 校验规则。 / Validate the candidate matrix calibration payload before applying it to the ADC engine. | `sensorarrayApplyPendingCommands` / `sensorarrayLoadPersistedMatrixCalibration` |
| `static void sensorarrayCalibrationLogOutcome(const char *tag, const char *state, const char *reason, const sensorarrayCalibrationStatus_t *status, esp_err_t err)` [3690](main.c#L3690) | 输出带状态、原因、schema 和来源的校准持久化诊断。 / Report the persistence operation with status, reason, source and schema. | `sensorarrayApplyPendingCommands` / `sensorarrayLoadPersistedMatrixCalibration` |
| `static const char *sensorarrayCalibrationLoadReason(const sensorarrayCalibrationStatus_t *status, esp_err_t err)` [3713](main.c#L3713) | 将加载结果映射为缺失、硬件/板号不符、schema 或有效等可查询原因。 / Convert calibration-load outcomes to stable diagnostic reasons. | `sensorarrayApplyPendingCommands` / `sensorarrayLoadPersistedMatrixCalibration` |
| `static void sensorarrayLoadPersistedMatrixCalibration(sensorarrayAppContext_t *ctx)` [3738](main.c#L3738) | 从持久存储读取并验证与板匹配的校准，再尝试装入 ADS 引擎，记录最终来源。 / Load, validate and apply a board-compatible calibration and report the selected source. | `sensorarrayInitFrontends` |
| `static esp_err_t sensorarrayInitFrontends(sensorarrayAppContext_t *ctx)` [3767](main.c#L3767) | 运行前端启动选速；初始化并绑定 FDC、ADS、路由、校准和 gap cache。 / Qualify the frontends, then bind engines, route controller, calibration and shared ADC shadow. | `sensorarrayInitSystem` |
| `static void sensorarrayInitAsyncLogging(sensorarrayAppContext_t *ctx)` [3841](main.c#L3841) | 根据构建配置启动异步日志并报告初始化失败及所选回退。 / Initialize asynchronous logging and report the configured fallback on failure. | `sensorarrayInitSystem` / `app_main` |
| `static void sensorarrayBuildDefaultScanPlan(sensorarrayAppContext_t *ctx)` [3867](main.c#L3867) | 将默认 FDC 矩阵扫描计划写入上下文，供启动及帧调度使用。 / Populate the application context with the default FDC scan plan. | `sensorarrayInitSystem` |
| `static esp_err_t sensorarrayRunBootCalibration(sensorarrayAppContext_t *ctx)` [3874](main.c#L3874) | 运行 FDC 启动扫频，按质量决定后续诊断与正常采集路径。 / Run the boot sweep and route control according to its quality result. | `sensorarrayScanTask` |
| `static esp_err_t sensorarrayRunOneFrame(sensorarrayAppContext_t *ctx)` [3980](main.c#L3980) | 捕获配置身份，让计划所选的引擎写入目标帧，再恢复同一采集身份和序号。 / Capture configuration identity, run the engine selected by the plan and attach that capture's identity and sequence to the result. `ESP_ERR_INVALID_ARG` `ESP_ERR_INVALID_STATE` | `sensorarrayRunMainLoop` |
| `static void sensorarrayRuntimeRescueTick(sensorarrayAppContext_t *ctx)` [4248](main.c#L4248) | 在 FDC worker 可用时服务矩阵救援状态，按现有模式门禁推进。 / Service pending FDC rescue only when worker ownership permits it. | `sensorarrayRunMainLoop` |
| `static void sensorarrayRuntimeI2cFallbackTick(sensorarrayAppContext_t *ctx)` [4264](main.c#L4264) | 只在合适的 FDC 状态和诊断条件下检查 I²C 回退，避免重配仍有 worker 的总线。 / Gate I²C fallback on device state and quiescent worker ownership. | `sensorarrayRunMainLoop` |
| `static void sensorarrayRunQueuedFullSweep(sensorarrayAppContext_t *ctx)` [4308](main.c#L4308) | 领取适用于当前配置的 FDC 全扫请求，执行救援并记录结果。 / Take an eligible full sweep and record its rescue outcome. | `sensorarrayRunMainLoop` |
| `static void sensorarrayRunDiagnosticTick(sensorarrayAppContext_t *ctx)` [4382](main.c#L4382) | 在诊断模式中按期限检查 FDC 设备并选择转储、重试或继续等待。 / Service timed diagnostic retries and register dumps for available FDCs. | `sensorarrayRunMainLoop` |
| `static void sensorarrayCompleteRecoveryRequest(sensorarrayAppContext_t *ctx, uint32_t requestId, sensorarrayRecoverState_t outcome, const char *reason, esp_err_t originalError, esp_err_t cleanupError)` [4424](main.c#L4424) | 把请求结果、当前配置和路由证据交给恢复控制器缓存并发布。 / Commit the recovery terminal together with route and configuration evidence. | `sensorarrayRunRecoveryBoundary` / `sensorarrayScanTask` |
| `static esp_err_t sensorarrayStopAfterRecoveryFailure(sensorarrayAppContext_t *ctx, esp_err_t originalError, esp_err_t cleanupError, const char *reason)` [4444](main.c#L4444) | 原错单独保留；恢复失败后停止 ADS、申请 SAFE 并发布降级配置，返回清理结果。 / Degrade the active configuration and request SAFE while keeping original and cleanup errors separate. | `sensorarrayRunAdsTimingCommand` / `sensorarrayRunRecoveryBoundary` |
| `static esp_err_t sensorarrayRunRecoveryLevel(sensorarrayAppContext_t *ctx, uint32_t level, bool *outHardwareTouched, esp_err_t *outCleanupError)` [4466](main.c#L4466) | 在所有者静止且配置适用时执行软/完整恢复；向调用者分别报告是否改动硬件与清理错误。 / Attempt soft or full recovery and return hardware-touch and cleanup evidence separately. | `sensorarrayRunRecoveryBoundary` |
| `static void sensorarrayPerformManualRestart(sensorarrayAppContext_t *ctx, uint32_t requestId)` [4527](main.c#L4527) | 发布手动重启意图后给队列一个有限发送机会，按请求选择应用或 ROM 重启。 / Publish manual restart intent, give local output a bounded opportunity, then enter the requested boot path. | `sensorarrayRunRecoveryBoundary` / `sensorarrayScanTask` |
| `static bool sensorarrayRunRecoveryBoundary(sensorarrayAppContext_t *ctx)` [4541](main.c#L4541) | 在边界领取手动重启或 RECOVER，根据 hold、级别和预算执行并完成其终态。 / Dispatch admitted restart/recover operations at the frame boundary under the applicable hold and budget gates. | `sensorarrayRunMainLoop` |
| `static void sensorarrayEmitPayloadTooLargeDrop(uint32_t sequence)` [4597](main.c#L4597) | 增加超长 payload 的丢弃数，并限频发出带序号的 TXDROP。 / Count overlength records and rate-limit sequence-tagged drop diagnostics. | `sensorarrayRunMainLoop` |
| `static void sensorarrayRunMainLoop(sensorarrayAppContext_t *ctx)` [4621](main.c#L4621) | 执行帧边界控制和恢复、采集、发布及节拍控制，维护真实进度计数。 / Run boundary commands and recovery, acquire and publish frames, and account for acquisition progress and pacing. | `sensorarrayScanTask` |
| `static void sensorarrayWifiStartupReservationComplete(uint32_t completed)` [4899](main.c#L4899) | 登记一次启动资源预留完成，在满足门槛后开放 Wi-Fi owner。 / Release this participant's Wi-Fi startup reservation gate. | `sensorarrayScanTask` / `app_main` |
| `static void sensorarrayScanTask(void *arg)` [4908](main.c#L4908) | 承担系统初始化、存在性故障处置、启动校准和持续采集的任务入口。 / Own scan initialization, startup recovery/calibration and entry to the continuous frame loop. | `sensorarrayStartScanTask` |
| `static esp_err_t sensorarrayStartScanTask(sensorarrayAppContext_t *ctx)` [5045](main.c#L5045) | 创建并固定扫描任务，记录创建前后的内存证据，失败返回 NO_MEM。 / Create the pinned scan task and retain allocation observations around task creation. | `app_main` |
| `static void sensorarrayLogBootHeap(const char *stage)` [5068](main.c#L5068) | 输出该启动阶段按内存能力分类的可用空间和大块状况。 / Record capability-specific free memory and largest-block observations at the given boot stage. | `app_main` |
| `static esp_err_t sensorarrayInitSystem(sensorarrayAppContext_t *ctx)` [5077](main.c#L5077) | 按依赖顺序安装 runtime、输出、板级设备和前端，构建初始扫描计划。 / Assemble runtime, board resources, outputs and frontends in initialization order. | `sensorarrayScanTask` |
| `void app_main(void)` [5118](main.c#L5118) | 初始化启动身份和受限恢复控制，预留 BLE 与输出任务资源，创建采集任务并记录启动故障。 / Initialise boot identity and recovery control, reserve BLE and output resources, start acquisition and record startup failures. | ESP-IDF 启动 / SDK startup |

### sensorarrayLog.c

| C 调用签名 / C signature | 行为及返回 / Behaviour and return | 调用位置 / Called by |
| --- | --- | --- |
| `static const char *sensorarrayRefState(void)` [27](sensorarrayLog.c#L27) | 根据已知/就绪状态返回参考源的日志标签，未知时标 unavailable。 / Format the observation state of the ADC reference for diagnostics. | `sensorarrayLogDbg` |
| `static const char *sensorarrayRefReadyBit(void)` [35](sensorarrayLog.c#L35) | 把参考就绪观测编码成 0、1 或缺失标记。 / Represent reference readiness as a logged bit or unavailable value. | `sensorarrayLogDbg` |
| `static int sensorarrayReadOptionalGpioLevel(int gpio)` [43](sensorarrayLog.c#L43) | 从可用 GPIO 读电平，对未配置的引脚返回缺失值。 / Read a configured GPIO, returning the unavailable sentinel for a missing pin. | `sensorarrayLogControlGpio` |
| `void sensorarrayLogSetAdsState(bool adsReady, bool adsRefReady)` [51](sensorarrayLog.c#L51) | 设置供后续日志使用的 ADS 可用及参考就绪状态。 / Set the cached ADC availability/reference-readiness evidence for logging. | `sensorarrayFrontendStartupRun` / `sensorarrayMeasureApplyRefPolicy` |
| `void sensorarrayLogDbgExtraReset(void)` [57](sensorarrayLog.c#L57) | 清空调试点的附加上下文，防止沿用先前测点的信息。 / Reset auxiliary diagnostic fields before recording another point. | `sensorarrayInitRuntime` / `sensorarrayLogDbg` |
| `void sensorarrayLogDbgExtraSetMux(uint8_t muxp, uint8_t muxn)` [62](sensorarrayLog.c#L62) | 保存当前 ADS 输入 MUX 诊断字段及其有效性。 / Store the ADC input-multiplexer observation for diagnostic output. | `sensorarrayMeasureReadAdsPairUv` |
| `void sensorarrayLogDbgExtraSetRefMux(uint8_t refmux)` [69](sensorarrayLog.c#L69) | 保存当前 REFMUX 寄存器观测值及有效标志。 / Store the reference-multiplexer readback and its validity. | `sensorarrayBringupAdsSetRefMux` / `sensorarrayMeasureReadAdsPairUv` |
| `void sensorarrayLogDbgExtraSetDiscardCount(uint8_t discardCount)` [75](sensorarrayLog.c#L75) | 记录本次输入建立过程中丢弃的样本数量。 / Record how many samples were discarded for the current diagnostic point. | `sensorarrayMeasureReadAdsPairUv` |
| `void sensorarrayLogDbgExtraCaptureCtrl(void)` [81](sensorarrayLog.c#L81) | 从开关状态采集当前 SW 和选择线控制信息以供后续日志使用。 / Capture switch control evidence in the auxiliary debug record. | `sensorarrayMeasureApplyRouteLevels` / `sensorarrayMeasureApplyRoute` |
| `const char *sensorarrayLogFmtI32(char *buf, size_t bufSize, bool valid, int32_t value)` [92](sensorarrayLog.c#L92) | 将可用值以有符号十进制写入 buf；缺失时返回日志占位符。 / Format an available value as signed decimal in caller storage, otherwise return the unavailable marker. | 独立接口 / Standalone entry |
| `const char *sensorarrayLogFmtU32(char *buf, size_t bufSize, bool valid, uint32_t value)` [101](sensorarrayLog.c#L101) | 将可用值以无符号十进制写入 buf；缺失时返回日志占位符。 / Format an available value as unsigned decimal in caller storage, otherwise return the unavailable marker. | 独立接口 / Standalone entry |
| `const char *sensorarrayLogFmtU8(char *buf, size_t bufSize, bool valid, uint8_t value)` [110](sensorarrayLog.c#L110) | 将可用值以8 位无符号十进制写入 buf；缺失时返回日志占位符。 / Format an available value as unsigned byte in caller storage, otherwise return the unavailable marker. | `sensorarrayLogDbg` |
| `const char *sensorarrayLogFmtBool(char *buf, size_t bufSize, bool valid, bool value)` [119](sensorarrayLog.c#L119) | 将可用值以布尔 0/1写入 buf；缺失时返回日志占位符。 / Format an available value as boolean digit in caller storage, otherwise return the unavailable marker. | 独立接口 / Standalone entry |
| `const char *sensorarrayLogFmtHexU8(char *buf, size_t bufSize, bool valid, uint8_t value)` [128](sensorarrayLog.c#L128) | 将可用值以0x 加两位十六进制写入 buf；缺失时返回日志占位符。 / Format an available value as two hexadecimal digits after 0x in caller storage, otherwise return the unavailable marker. | `sensorarrayMeasureApplyRefPolicy` / `sensorarrayLogDbg` |
| `const char *sensorarrayLogFmtHexU16(char *buf, size_t bufSize, bool valid, uint16_t value)` [137](sensorarrayLog.c#L137) | 将可用值以0x 加四位十六进制写入 buf；缺失时返回日志占位符。 / Format an available value as four hexadecimal digits after 0x in caller storage, otherwise return the unavailable marker. | `sensorarrayLogStartupFdc` |
| `const char *sensorarrayLogFmtI2cPort(char *buf, size_t bufSize, const BoardSupportI2cCtx_t *ctx)` [146](sensorarrayLog.c#L146) | 将已知 I²C 上下文的端口格式化到调用者缓冲，未知时返回缺失标记。 / Format the known bus port or return the diagnostic unavailable marker. | `sensorarrayLogStartupFdc` |
| `const char *sensorarrayLogFmtGpioLevel(char *buf, size_t bufSize, bool valid, int level)` [155](sensorarrayLog.c#L155) | 将 GPIO 电平格式化为 0、1 或缺失标记。 / Format a digital GPIO observation with an explicit unavailable state. | `sensorarrayMeasureApplyRefPolicy` / `sensorarrayLogDbg` |
| `const char *sensorarrayLogSwSourceName(tmux1108Source_t source)` [164](sensorarrayLog.c#L164) | 将逻辑 REF/GND 选择按板级极性换算为引脚 HIGH/LOW 名称。 / Map the logical source through the configured SW polarity to its physical level name. | `sensorarrayMeasureApplyRoute` / `sensorarrayMeasureApplyRefPolicy` |
| `const char *sensorarrayLogSwSourceLogicalName(tmux1108Source_t source)` [171](sensorarrayLog.c#L171) | 将 TMUX1108 逻辑源映射为 REF 或 GND 名称。 / Name the logical TMUX1108 source selection. | 独立接口 / Standalone entry |
| `const char *sensorarrayLogAdsMuxName(uint8_t mux)` [176](sensorarrayLog.c#L176) | 把 ADS 输入编号格式化为 AIN0…AIN9、AINCOM 或 UNKNOWN。 / Name an ADC input selector for diagnostics. | `sensorarrayMeasureReadAdsPairUv` |
| `const char *sensorarrayLogRoutePathName(sensorarrayRoutePathKind_t path)` [206](sensorarrayLog.c#L206) | 把 CAP/VOLT/RES 路由类型映射为诊断路径标签。 / Name the selected measurement route in a diagnostic record. | `sensorarrayMeasureApplyRoute` / `sensorarrayLogRouteStepEx` |
| `const char *sensorarrayLogBuildMapLabel(char *buf, size_t bufSize, const char *routeMap, const char *fdcMap)` [219](sensorarrayLog.c#L219) | 组合 routeMap 与 fdcMap；仅一个可用时返回原字符串，两者都有时写入 buf。 / Join the two available mapping labels into buf, or return the only available label. | 独立接口 / Standalone entry |
| `void sensorarrayLogDbg(const char *point, const char *kind, const char *column, const char *dline, const char *sw, const char *mode, const char *value, const char *valueUv, const char *valueMohm, const char *raw, const char *wd, const char *amp, const char *fdcDev, const char *i2cPort, const char *i2cAddr, const char *idMfg, const char *idDev, const char *map, esp_err_t err, const char *status)` [247](sensorarrayLog.c#L247) | 将一个调试点连同原始值、路由和可用的附加证据写入日志。 / Emit the point record with route, raw measurement and available debug context. | 独立接口 / Standalone entry |
| `void sensorarrayLogStartup(const char *mode, esp_err_t err, const char *status, int32_t detailValue)` [344](sensorarrayLog.c#L344) | 输出启动步骤、状态和可选说明，供逐阶段定位。 / Log the named startup step, status and detail. | `sensorarrayFrontendStartupRun` / `sensorarrayApplyTmuxDefaults` |
| `void sensorarrayLogStartupFdc(const char *mode, const sensorarrayFdcDeviceState_t *fdcState, esp_err_t err, const char *status, int32_t detailValue, bool hasIds, uint16_t manufacturerId, uint16_t deviceId, const char *map)` [353](sensorarrayLog.c#L353) | 输出带地址/器件状态的 FDC 启动记录。 / Emit startup evidence for the selected FDC. | `sensorarrayBringupProbeFdcBus` / `sensorarrayInitFdcDevice` |
| `void sensorarrayLogControlGpio(const char *stage, const char *point)` [381](sensorarrayLog.c#L381) | 记录控制引脚配置、所需电平与观察到的 GPIO 值。 / Log control pin configuration and the available digital observations. | `sensorarrayLogRouteStepEx` |
| `void sensorarrayLogSelaRouteDecision(const char *stage, const char *label, sensorarraySelaRoute_t requestRoute, int selaWriteLevel, int selaCmdLevel, int selaObsLevel, bool obsResolvedValid, sensorarraySelaRoute_t obsResolvedRoute)` [436](sensorarrayLog.c#L436) | 记录请求的 SELA 路由、写值和实际回读，并带上调用阶段与标签。 / Log requested SELA routing and its commanded and observed levels. | `sensorarrayMeasureWriteSela` |
| `void sensorarrayLogSelaReadbackMismatch(const char *stage, const char *label, int wroteLevel, int readLevel)` [456](sensorarrayLog.c#L456) | 仅作为诊断报告 SELA 写入与实际 GPIO 观察不一致。 / Report a mismatch between the requested and observed SELA level. | `sensorarrayMeasureWriteSela` |
| `void sensorarrayLogRouteStep(const char *stage, const char *label, uint8_t sColumn, uint8_t dLine, sensorarrayRoutePathKind_t path, tmux1108Source_t swSource, sensorarraySelaRoute_t selaRoute, bool selBLevel, esp_err_t err, const char *status)` [466](sensorarrayLog.c#L466) | 以普通路由步骤参数调用扩展日志入口。 / Forward a route step to the extended diagnostic formatter. | `sensorarrayMeasureApplyRouteLevels` / `sensorarrayMeasureApplyRoute` |
| `void sensorarrayLogRouteStepEx(const char *stage, const char *label, uint8_t sColumn, uint8_t dLine, sensorarrayRoutePathKind_t path, tmux1108Source_t swSource, sensorarraySelaRoute_t selaRoute, bool selBLevel, esp_err_t err, const char *status, const char *reason)` [490](sensorarrayLog.c#L490) | 记录含 SELA、SELB、SW 与输入映射等上下文的详细路由步骤。 / Emit the extended step with switch levels and route context. | `sensorarrayMeasureSetSwForRoute` / `sensorarrayLogRouteStep` |

### sensorarrayTypes.h

| C 调用签名 / C signature | 行为及返回 / Behaviour and return | 调用位置 / Called by |
| --- | --- | --- |
| `static inline size_t sensorarrayMatrixIndex(uint8_t sIndex, uint8_t dIndex)` [47](sensorarrayTypes.h#L47) | 把从 1 起编号的 (S,D) 换算为从 0 起的行主序索引；应先核对范围。 / Convert one-based row/column coordinates to a zero-based row-major index. | `sensorarrayAdsMatrixMeasureCellImpl` / `sensorarrayAdsMatrixMeasureCell` |
| `static inline bool sensorarrayMatrixIndexIsValid(uint8_t sIndex, uint8_t dIndex)` [52](sensorarrayTypes.h#L52) | 检查 S、D 的上下界及计算所得索引是否在矩阵内。 / Validate both one-based coordinates and the resulting matrix index. | `sensorarrayMeasureApplyFdcCellRoute` / `sensorarrayMeasureAccumulateFdcHealth` |

## 参数索引 / Parameter index

相同名称按模块共享解释。完整类型、数组维度与输入输出方向以对应函数签名和调用点为准。指针参数所需存储由调用者提供，异步作业另受上面的所有权约束。

Entries share parameter descriptions within this module. Refer to the signature and its caller for the precise type, array shape and direction. The caller supplies pointer storage, subject to the ownership rules above for asynchronous work.

| 参数 / Parameter | 含义 / Meaning |
| --- | --- |
| `...`, `format` | printf 风格的格式串与匹配格式的可变参数。 / printf-style format string and its matching variadic arguments. |
| `adsReady` | ADC 驱动是否已可供该日志点报告状态。 / Whether an ADC observation is available for this log point. |
| `adsRefReady` | 参考就绪的数字状态，须结合 adsReady 解释。 / Cached reference-ready indication, meaningful only when the ADC is available. |
| `amp` | 调用方提供的 FDC amplitude 诊断文本，缺失用日志占位符。 / Caller-formatted amplitude diagnostic, or the unavailable marker. |
| `arg`, `context` | 回调或任务创建时绑定的上下文；在该调用的有效期内保留其存储。 / Context bound at callback registration or task creation. Keep its backing object alive for the operation. |
| `attempt` | 此故障记录所属的恢复尝试序号。 / Recovery attempt number to associate with the fault. |
| `buf`, `response` | 调用者提供的回复、暂存或结果缓冲区，使用随同的容量/类型。 / Caller-provided reply, scratch or result storage with the accompanying capacity/type. |
| `bufSize`, `responseSize` | 字符输出缓冲区总字节数，含结尾 NUL 的空间。 / Total capacity of the text buffer in bytes, including space for its final NUL. |
| `cache` | 该入口操作的寄存器、数值或其他缓存，作用域遵循声明类型。 / Register, value or other cache of the specific type in the signature. |
| `cleanupError`, `err`, `error`, `originalError` | 操作、首个根因或独立清理结果的错误码。保留调用层提供的具体错误。 / Error for the operation, first cause or separate cleanup, preserving the code from the reporting layer. |
| `column` | 调试输出中的矩阵行选择标签。 / Column selector label as supplied to the debug record. |
| `command`, `data` | 按该函数声明传入的文本、命令、操作码或数据字节，显式 length 限定可读范围。 / Text, command, opcode or data bytes as declared. Where supplied, length bounds the readable range. |
| `completed` | 是否已有完成的恢复结果，供终态发布使用。 / Completion flag supplied when publishing recovery evidence. |
| `ctx` | 所声明类型的应用、总线或调度上下文，供调用链共享。 / Application, bus or scheduler context of the declared type, shared with the call chain. |
| `detailValue` | 启动步骤随附的有符号整数诊断值。 / Per-step signed diagnostic value in the startup record. |
| `deviceId`, `manufacturerId` | 从 FDC 或 ADS 身份寄存器得到的器件标识。 / Part/manufacturer identifier obtained from device identity registers. |
| `devId`, `fdcState` | 对应驱动句柄、FDC 主/次设备 ID 或 ADS/FDC 状态；按签名类型选取当前板上同一实例。 / Corresponding driver handle, primary/secondary FDC ID or ADS/FDC state. Pass the board instance matching the declared type. |
| `dIndex`, `sIndex` | 在该实现中指向 S/D 位置的索引，按调用点的板级坐标/数组约定传入。 / S/D position index in the board coordinate or array convention used by the call site. |
| `discardCount` | 已丢弃的转换样本数，作为调试记录的附加字段。 / Number of discarded conversions recorded with the diagnostic. |
| `dLine`, `sColumn` | 板级 S 列或 D 线编号，合法位置为 1..8。 / Physical S column or D line, numbered 1..8. |
| `dline` | 已格式化的 D1…D8 标签，表示被观测的列。 / Preformatted physical D-line label for the point under observation. |
| `emitEvent` | 为 true 时发布相应模式切换事件。 / Request publication of the corresponding mode-transition event. |
| `event` | 本次 ADS 故障的可写事件；填入上下文后在回调返回前使用。 / Mutable fault event used synchronously during emission. |
| `fdcDev` | 调试点关联的 primary/secondary FDC 文本标识。 / Display label for the FDC serving the point. |
| `fdcMap` | 此点所用 FDC/通道映射文字，允许缺失。 / Optional text describing the selected FDC channel mapping. |
| `frame` | 按完整签名的类型传入目标帧、子帧或特定操作的目标。帧对象由调用者分配。 / Destination frame, child frame or operation-specific target according to the declared type. The caller owns frame storage. |
| `frameBoundary` | 配置生效时的帧边界序号。 / Capture boundary sequence associated with the applied configuration. |
| `frameMode`, `mode`, `targetMode` | 对应调用域的模式枚举，保留目标、当前帧或恢复上下文的含义。 / Mode enum in the declared domain, selecting the target, frame or recovery mode as indicated. |
| `frameStartUs` | 对应帧或各模式子组的起止时刻，单位 µs。 / Frame or mode-group start/end timestamps in µs. |
| `freeBefore` | runtime 初始化前的可用 heap 字节数。 / Free heap bytes measured before runtime initialization. |
| `gpio` | IDF GPIO 编号，负数表示未配置信号。 / IDF pin number; a negative value marks an absent pin. |
| `hasIds` | 指出提供的 FDC manufacturerId/deviceId 是否来自有效读回。 / Validity of the supplied FDC identification readbacks. |
| `healthyFrame` | 本次观测是否满足连续健康采集条件。 / Whether this frame contributes to the stable-progress window. |
| `i2cAddr` | 7 位 I²C 设备地址，匹配对应总线实例。 / Seven-bit I²C address on the selected bus. |
| `i2cPort` | 已格式化的 I²C 总线号；表示为 DBG 的文本字段。 / Formatted bus instance used in the DBG record. |
| `idDev` | 已格式化的芯片设备标识，缺失保留占位。 / Formatted device ID for the point record. |
| `idMfg` | 已格式化的芯片厂商标识，缺失保留占位。 / Formatted manufacturer ID for the point record. |
| `kind` | DBG 记录种类字符串，由调用者选择。 / Caller-supplied diagnostic record kind. |
| `label` | 定位路由操作的短标签，随阶段写入日志。 / Route-step label recorded with the stage. |
| `length`, `payloadLength` | 输入、载荷、底层存储或读写传输的字节数。 / Byte length of input, payload, backing storage or transfer. |
| `level` | RECOVER 的级别或 GPIO 的读数，由签名中的调用位置确定。 / Recovery level for RECOVER, or an observed digital level for GPIO formatting. |
| `line` | ADS 故障格式化器提供的 NUL 结尾诊断行。 / Terminated diagnostic text from the ADS fault formatter. |
| `map` | 对本条 INIT/DBG 有效的板级位置映射说明。 / Board mapping text for the current INIT or DBG record. |
| `marker` | 交给串口队列的完整重启标记文本。 / Terminal marker text to queue before restart. |
| `minFreeBefore` | 初始化前观察到的历史最低 heap 字节数。 / Minimum-ever free heap bytes observed before initialization. |
| `mux` | 要转换成 AIN 标签的 ADS 输入选择码。 / ADC input selector to translate to its AIN label. |
| `muxn`, `muxp` | ADS 正、负模拟输入选择码，请按板级 D 线映射。指针形态用于接收选择码。 / ADS positive/negative input selector from the board D-line map. Pointer forms receive the selector. |
| `obsResolvedRoute` | 由数字引脚读回推断的 SELA 路由枚举。 / Route enum resolved from digital SELA readback. |
| `obsResolvedValid` | 是否有足够回读证据解释 obsResolvedRoute。 / Whether the observed SELA route can be interpreted. |
| `outCleanupError` | 接收结果。操作、首个根因或独立清理结果的错误码。保留调用层提供的具体错误。 / Output. Error for the operation, first cause or separate cleanup, preserving the code from the reporting layer. |
| `outcome` | 恢复尝试/ADS 故障的结果分类，写入对应事件。 / Outcome enum to record with this recovery or fault. |
| `outCopy` | 调用者提供的启动记录副本目标。 / Caller-owned destination for the boot breadcrumb. |
| `outHardwareTouched` | 接收本次恢复是否已经操作前端硬件。 / Set when this recovery attempt reached hardware. |
| `outIdentity` | 接收结果。帧的不可变配置身份或同一版本的完整权威快照。 / Output. Immutable frame configuration identity or the authority snapshot for that revision. |
| `outLevel` | 接收从控制邮箱领取的 RECOVER 级别。 / Destination for the level of the admitted recovery. |
| `outRail` | 接收与目标帧和来源相符的 AVDD/AVSS 快照。 / Destination for the rail split valid for the target frame and source. |
| `outRequestId` | 本地请求或事务标识，out 参数接收新分配或选中的 ID。 / Local request or transaction identifier. An out parameter receives the allocated or selected ID. |
| `outResult`, `outSnapshot` | 函数签名注明的结果、快照或统计结构，调用者分配复制目标。 / Result, snapshot or statistics object of the declared type, with caller-owned output storage. |
| `path` | CAP/VOLT/RES 路由类型，供记录选择的测量分支。 / Measurement route enum for logging the selected path. |
| `payload` | 校准记录/载荷或原始字节；outPayload 指向允许写入的目标。 / Calibration record/payload or raw bytes. outPayload names caller-writable storage. |
| `point` | 调用者给此物理测点或 GPIO 观测设置的标签。 / Caller-supplied label for the measurement point or pin observation. |
| `profile` | 配置档或八个物理行的模式列表。outModes 接收解析结果。 / Configuration profile or mode selection for the eight physical rows. outModes receives parsed values. |
| `quality` | 双 FDC 启动校准给出的 OK、DEGRADED 或 FAIL 分类。 / FDC boot quality classification. |
| `raw` | ADC 原始整数码或 FDC 原始频率码；单位与定点物理值分开。 / Integer ADC code or FDC frequency code, separate from fixed-point physical units. |
| `readLevel` | 读回的数字 GPIO 电平，与 wroteLevel 分开记录。 / Digital readback to compare with the commanded level. |
| `reason` | 操作或拒绝原因字符串；outReason 用于返回原因。 / Operation/rejection reason string. outReason receives the reason. |
| `recovery` | 保存触发来源、目标模式及尝试状态的恢复对象。 / Recovery state describing the trigger, saved target and attempt. |
| `refmux` | ADS 对应的原始寄存器字节，比较由该事务的读回掩码限定。 / Raw ADS register byte. Readback comparisons use the mask for that transaction. |
| `requestId`, `triggerRequestId` | 对应配置、恢复触发或 profile 域内的请求标识，由接收方与提交时身份配对。 / Request identifier for the indicated configuration, recovery trigger or row profile, paired with the identity at admission. |
| `requestRoute` | 由调用者请求的 SELA 逻辑路由枚举。 / Requested logical SELA connection. |
| `restartKind` | 自动或手动重启分类，决定保留预算的处理方式。 / Automatic or manual restart class for retained budget accounting. |
| `routeMap` | 描述当前矩阵路由的文字标签，可与 fdcMap 合并。 / Optional route label that can be combined with the FDC mapping. |
| `row` | 行或列的坐标。通用 matrixEngineRegion_t 的原点为零；以板级 S/D 位置操作的接口从 1 编号。 / Row or column coordinate. Generic matrixEngineRegion_t origins are zero-based. Board S/D coordinates are one-based. |
| `rows` | 行数值或行配置快照，由该函数的参数类型决定。 / Active count or row-configuration snapshot as specified by its declared type. |
| `selaCmdLevel` | 控制器快照中的 SELA 命令电平。 / SELA command level held in the controller snapshot. |
| `selaObsLevel` | GPIO 实际读回的 SELA 电平。 / Observed SELA pin level. |
| `selaRoute` | 预期/观测的路由、寄存器或状态值，按该签名中声明的类型比较。 / Requested/observed route, register or state values, compared in the types shown by the signature. |
| `selaWriteLevel` | 从请求路由得到的 SELA 目标写值。 / SELA level calculated for the requested route. |
| `selBLevel` | 此次路由步骤要求的 SELB 数字电平。 / Requested SELB digital level for this route step. |
| `sequence` | 对应采集帧的序号，必须与提供的帧、事件或缓存上下文属于同次启动。 / Capture sequence belonging to the same boot as the supplied frame, event or cache context. |
| `singleChannel` | 波形实验中为 true 时走单通道读取，否则读四通道 burst。 / Select the single-channel read in the waveform experiment; false selects a four-channel burst. |
| `sleepEnabled` | 为 true 时让可用 FDC 进入 sleep，false 时退出。 / Select the requested FDC sleep state. |
| `source` | TMUX1108 的 REF/GND 逻辑源或声明所指的来源枚举。 / Logical REF/GND switch source or source enum as declared. |
| `stage` | 本步启动、路由或故障处理的名称或枚举。 / Stage string or enum for the current startup, route or fault operation. |
| `state` | 所声明类型的状态对象或枚举。传入前端状态指针时，其生命周期覆盖总线或采集操作。 / State object or enum of the declared type. A frontend-state pointer must outlive the bus or capture operation. |
| `status` | 调用者提供的步骤状态字符串，随错误码输出。 / Caller-supplied operation status to print with the error. |
| `sw` | 已格式化的 SW/参考开关状态，供 DBG 输出。 / Formatted switch state for the DBG record. |
| `swSource` | TMUX1108 公共端所选的逻辑源。 / Logical source selected at the multiplexer common terminal. |
| `tag` | 标识校准操作的输出记录名称。 / Record name identifying the calibration operation. |
| `trigger` | 启动本次 ADS 恢复的 battery/ADSCHK 等触发原因。 / Reason for starting this ADC restore recovery. |
| `valid` | 表示证据已知、数据有效、功能启用或操作成功的布尔标志，按参数名独立解释。 / Boolean representing known evidence, validity, enablement or success as named. |
| `value` | 已格式化的测点值，或由所选格式函数转换的整数/布尔输入。 / Formatted reading or value to encode in the specified log format. |
| `valueMohm` | 以毫欧表示的测点调试文本。 / Measurement text in milliohms for the debug record. |
| `valueUv` | 值，单位 µV。 / Value in µV. |
| `wd` | 调用方传入 DBG 的 wd 文本；缺失时保持占位符。 / Caller-supplied wd field, preserved as unavailable if absent. |
| `wroteLevel` | 记录实际命令写入的数字电平，与 GPIO 观测对照。 / Commanded output level to compare with the digital observation. |

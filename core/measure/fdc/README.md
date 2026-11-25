# 双 FDC2214 行采集 / Dual-FDC row acquisition

CAP 采集按行对两个 FDC2214 并行发作业，每个器件处理该行的四个 D 通道。协调器维护持久槽、epoch 和阶段，通知只唤醒 worker。INTB、必要时检查 STATUS、确认 DRDY 并读四通道。质量判定、调谐缓存和恢复策略分别记录，单点诊断能追溯到具体器件和通道。

CAP acquisition posts concurrent row jobs to two FDC2214 devices, each serving four D channels. The coordinator keeps persistent slots with an epoch and phase. Notifications wake workers. The read path uses INTB, consults STATUS when required, establishes readiness and reads four channels. Quality, tuning caches and recovery are tracked separately so a cell can be traced to its device and channel.

## 调用方式 / Calling the module

```c
esp_err_t err = sensorarrayFdcMatrixEngineReadFrame(&ctx->fdcEngine,
                                                     &ctx->scanPlan,
                                                     &ctx->frame);
```

调用者先初始化引擎与计划，使用长期存活的状态及帧存储。已接受作业只在截止时间内等待，超时触发取消/收回或隔离。旧 epoch 的 DONE 不能释放新作业。空闲等待与受理后的收尾使用不同期限。有效量由新鲜度与状态共同决定，缺失值保留掩码。全扫描恢复、启动参数扫描和运行期帧采集分别有入口。

Initialise the engine and plan first and supply long-lived state and frame storage. Accepted jobs have deadlines. Timeout requires cancellation and reclamation, or quarantine. A DONE from an old epoch cannot release a new job. Idle waiting and accepted-job cleanup have different bounds. Freshness and status govern validity and missing cells retain their masks. Boot tuning, full rescue and ordinary frames have separate entry points.

## 行时间线与数据 / Row timing and data

两组 I²C 总线各服务四列。取出新频率码后，按通道的 CLOCK_DIVIDERS 与选定参考时钟计算频率，再用 `C = 1 / ((2πf)² L)` 得到该点的总谐振电容。原始码、频率、电容、valid/fresh/warn/error 掩码保留不同含义。启动和完整恢复使用逐行扫描，旧的单通道参数扫描入口对合法输入返回 ESP_ERR_NOT_SUPPORTED。

Each I²C bus serves four columns. Convert a fresh frequency code using that channel's CLOCK_DIVIDERS and the selected reference clock, then derive the cell’s total resonant capacitance using the LC equation above. Raw code, frequency, capacitance and valid/fresh/warn/error masks remain distinct. Boot and full rescue tune by row. The legacy single-channel tuning entry returns ESP_ERR_NOT_SUPPORTED for valid inputs.

## 源码和头文件 / Source and headers

| 文件 / File | 职责与声明 / Responsibility and declarations |
| --- | --- |
| [sensorarrayFdcCache.c](sensorarrayFdcCache.c) | 将扫描所得寄存器配置和实测时序写入逐点缓存，评估预期参数对应的行和整帧预算。 / Transfers measured register settings and timing into per-cell cache and relates profiles to row and frame budgets. |
| [sensorarrayFdcFrame.c](sensorarrayFdcFrame.c) | 将双路行结果并入 CAP 帧，汇总状态、新鲜度、警告、耗时和总电容。 / Assembles dual-device row results into a CAP frame and aggregates health, freshness, warnings, timing and total capacitance. |
| [sensorarrayFdcInternal.h](sensorarrayFdcInternal.h) | 内部共享的 worker、时序与采集接口；由本目录多个实现组成一条生产调用链。 / Shares internal worker, timing and acquisition declarations between the production FDC translation units. `sensorarrayFdcAutoscanSamples_t` `sensorarrayFdcFrameHealth_t` |
| [sensorarrayFdcMatrix.c](sensorarrayFdcMatrix.c) · [sensorarrayFdcMatrix.h](sensorarrayFdcMatrix.h) | 对外提供 FDC 引擎初始化、启动扫描、读帧、诊断模式和整矩阵救援入口。 / Exposes engine initialisation, boot sweep, frame acquisition, diagnostic mode and full-matrix rescue. `sensorarrayFdcMatrixEngine_t` |
| [sensorarrayFdcQuality.c](sensorarrayFdcQuality.c) | 解释 FDC 状态位与数据质量，结合参考时钟、分频和电感把 28 位原始频率码换算为 pF。 / Interprets FDC status and data quality and converts 28-bit frequency codes to pF using reference clock, dividers and inductance. |
| [sensorarrayFdcRead4.c](sensorarrayFdcRead4.c) | 根据已确认的 ready 证据读取四通道并生成每通道的有效、新鲜、错误和警告标记。 / Reads four channels from established ready evidence and assigns per-channel valid, fresh, error and warning masks. |
| [sensorarrayFdcReady.c](sensorarrayFdcReady.c) | 控制休眠退出，解释 INTB、DRDY、unread 和受限 STATUS 回退；按剩余截止时间判断是否允许读数据。 / Handles sleep exit, INTB, DRDY, unread and bounded STATUS fallback and admits reads against the remaining deadline. |
| [sensorarrayFdcRescue.c](sensorarrayFdcRescue.c) · [sensorarrayFdcRescue.h](sensorarrayFdcRescue.h) | 从当前帧的质量与连续失败记录判断救援级别、冷却与后续动作。 / Decides rescue action and cooldown from frame quality and consecutive failures. `sensorarrayFdcRescueContext_t` |
| [sensorarrayFdcRescuePolicy.c](sensorarrayFdcRescuePolicy.c) | 该编译单元保留拆分边界。运行期救援策略实现在同目录 sensorarrayFdcRescue.c。 / This translation unit reserves the split boundary. Runtime rescue policy resides in sensorarrayFdcRescue.c. |
| [sensorarrayFdcRowEpoch.c](sensorarrayFdcRowEpoch.c) | 协调两条总线的持久 worker 槽，安排 ACK、RUN、DONE 与超时回收，并保留迟到和隔离证据。 / Coordinates persistent worker slots for two buses through ACK, RUN, DONE and timeout reclamation, retaining late/quarantine evidence. |
| [sensorarrayFdcSweep.c](sensorarrayFdcSweep.c) · [sensorarrayFdcSweep.h](sensorarrayFdcSweep.h) | 在配置候选中按实际读数、频率、告警与时间预算筛选寄存器设置，生成 boot/运行期参数与缓存。 / Selects register settings using measured codes, frequency, warnings and time budgets, producing boot/runtime profiles and cache entries. `sensorarrayFdcSweepReason_t` `sensorarrayFdcFaultReason_t` |
| [sensorarrayFdcTiming.c](sensorarrayFdcTiming.c) | 估计 I²C 传输量并累加每行和每个器件的阶段耗时，归纳限速阶段。 / Estimates I²C traffic, accumulates per-device and per-row phase durations and identifies timing bottlenecks. |
| [sensorarrayFdcWorkerLifecycle.h](sensorarrayFdcWorkerLifecycle.h) | 在持久槽内以原子阶段转换处理投递、开始、取消、释放与隔离。 / Applies atomic phase transitions to posting, running, cancellation, release and quarantine of a persistent job. `sensorarrayFdcJobPhase_t` `sensorarrayFdcJobLifecycle_t` |

## 函数 / Functions

签名按源文件排列，参数顺序即调用顺序。下表标出生产代码中的调用者或回调注册者（最多两个）。`static` 限于所在编译单元。错误码列于对应行为后，只列函数直接返回的常量；硬件与下层调用可传播其他错误。

Signatures are grouped by source in call-argument order. The last column names up to two production callers or callback registrars. `static` entries are local to their translation unit. Constants after a description are explicit error returns. Underlying operations may propagate additional errors.

[sensorarrayFdcCache.c](#sensorarrayfdccachec) · [sensorarrayFdcFrame.c](#sensorarrayfdcframec) · [sensorarrayFdcMatrix.c](#sensorarrayfdcmatrixc) · [sensorarrayFdcQuality.c](#sensorarrayfdcqualityc) · [sensorarrayFdcRead4.c](#sensorarrayfdcread4c) · [sensorarrayFdcReady.c](#sensorarrayfdcreadyc) · [sensorarrayFdcRescue.c](#sensorarrayfdcrescuec) · [sensorarrayFdcRowEpoch.c](#sensorarrayfdcrowepochc) · [sensorarrayFdcSweep.c](#sensorarrayfdcsweepc) · [sensorarrayFdcTiming.c](#sensorarrayfdctimingc) · [sensorarrayFdcWorkerLifecycle.h](#sensorarrayfdcworkerlifecycleh)

### sensorarrayFdcCache.c

| C 调用签名 / C signature | 行为及返回 / Behaviour and return | 调用位置 / Called by |
| --- | --- | --- |
| `void sensorarrayMeasureRuntimeConfigsFromApplied(sensorarrayState_t *state, sensorarrayFdcDeviceId_t devId, sensorarrayFdcRuntimeChannelConfig_t configs[4])` [4](sensorarrayFdcCache.c#L4) | 把已应用的寄存器配置复制为用于行预算的运行时通道配置。 / Translate applied register settings into the runtime channel configurations used for row budgeting. | `sensorarrayMeasureFdcPrepareDeviceWhileSleeping` / `sensorarrayMeasureFdcRunDeviceEpochAfterSleep` |
| `uint32_t sensorarrayMeasureFdcTheoreticalFrameFpsX100(uint32_t roundUs)` [31](sensorarrayFdcCache.c#L31) | 由每轮转换时间和行数预算估算理论帧率，结果按 FPS×100 表示。 / Estimate theoretical frame rate from conversion-round timing and row budget in FPS × 100. | `sensorarrayMeasureLogFdcPfuDiag` / `sensorarrayMeasureLogFdcPfuFastProfile` |
| `const char *sensorarrayMeasureFdcProfileTooSlowActionName(void)` [40](sensorarrayFdcCache.c#L40) | 返回配置档过慢动作的显示名称。 / Return the display name for profile too slow action. | `sensorarrayMeasureApplyFdcCachedRowConfig` / `sensorarrayMeasurePrintFdcProfileSummary` |
| `void sensorarrayMeasureLogFdcPfuDiag(sensorarrayFdcDeviceId_t devId, uint8_t row, const char *action, uint32_t roundUs, uint32_t targetRoundUs, uint32_t rowBudgetUs, const uint16_t rCount[4], const uint16_t settleCount[4])` [51](sensorarrayFdcCache.c#L51) | 输出当前 profile 的计数和建立参数，供分析是否超出行预算。 / Report counter and settling parameters for comparing the active profile to the row budget. | `sensorarrayMeasureApplyFdcCachedRowConfig` |
| `void sensorarrayMeasureLogFdcPfuFastProfile(sensorarrayFdcDeviceId_t devId, uint8_t row, const char *action, uint32_t oldRoundUs, uint32_t newRoundUs, uint32_t targetRoundUs, uint32_t rowBudgetUs, const uint16_t oldRcount[4], const uint16_t newRcount[4], const uint16_t oldSettle[4], const uint16_t newSettle[4], const char *decisionReason, uint32_t diffWriteCount)` [101](sensorarrayFdcCache.c#L101) | 报告降低转换时间预算后选择的 FDC 快速参数。 / Report the chosen faster FDC parameters after conversion-budget adjustment. | `sensorarrayMeasureApplyFdcCachedRowConfig` |
| `void sensorarrayMeasureLogFdcPfuApplyResult(sensorarrayFdcDeviceId_t devId, uint8_t row, const char *why, const char *action, uint32_t oldRoundUs, uint32_t newRoundUs, uint32_t targetRoundUs, uint32_t rowBudgetUs, uint16_t oldDriveCurrent0, uint16_t newDriveCurrent0, bool profileTooSlow, uint32_t diffWriteCount)` [151](sensorarrayFdcCache.c#L151) | 记录候选参数是否写入以及由此改变的实际寄存器数量。 / Report whether a proposed profile was applied and how many register writes changed. | `sensorarrayMeasureApplyFdcCachedRowConfig` |
| `bool sensorarrayMeasurePromoteFdcExpectedProfileToCellCache(sensorarrayState_t *state, uint8_t row, sensorarrayFdcDeviceId_t devId, sensorarrayFdcAppliedRowConfig_t *expected, uint32_t effectiveFclkHz)` [183](sensorarrayFdcCache.c#L183) | 将已接受的器件通道配置写入相应测点的期望参数缓存。 / Promote the accepted device/channel profile into the corresponding cell cache. | `sensorarrayMeasureApplyFdcCachedRowConfig` |
| `esp_err_t sensorarrayMeasureApplyFdcCachedRowConfig(sensorarrayState_t *state, uint8_t row, sensorarrayFdcDeviceId_t devId, const char *reason, bool forceWrite, sensorarrayFdcDeviceTiming_t *timing)` [229](sensorarrayFdcCache.c#L229) | 为选定行和器件应用缓存配置并记录差异写入与时间。 / Apply cached settings for one device/row, accounting for changed writes and elapsed time. `ESP_ERR_INVALID_ARG` `ESP_ERR_INVALID_STATE` `ESP_ERR_NOT_FOUND` | `sensorarrayMeasureFdcDeviceLevelResync` / `sensorarrayMeasureFdcPrepareDeviceWhileSleeping` |

### sensorarrayFdcFrame.c

| C 调用签名 / C signature | 行为及返回 / Behaviour and return | 调用位置 / Called by |
| --- | --- | --- |
| `void sensorarrayMeasureFillFdcMatrixRow(sensorarrayFdcMatrixFrame_t *frame, uint8_t sIndex, const sensorarrayFdcAutoscanSamples_t *primary, const sensorarrayFdcAutoscanSamples_t *secondary, const sensorarrayFdcRuntimeChannelConfig_t configs[2][4], uint8_t *outValidMask8, uint8_t *outWarnMask8, uint8_t *outErrorMask8)` [19](sensorarrayFdcFrame.c#L19) | 把两颗器件各四通道映射到本行 D1..D8，返回有效、警告和错误位图。 / Map four channels from each device to D1..D8 and return the row validity, warning and error masks. | `sensorarrayMeasureReadFdcMatrixFrameSnapshot` |
| `void sensorarrayMeasureAccumulateFdcHealth(sensorarrayFdcFrameHealth_t *health, uint8_t sIndex, const sensorarrayFdcAutoscanSamples_t *primary, const sensorarrayFdcAutoscanSamples_t *secondary, const sensorarrayFdcRuntimeChannelConfig_t configs[2][4], const sensorarrayFdcMatrixFrame_t *frame)` [141](sensorarrayFdcFrame.c#L141) | 汇总本帧新鲜度、状态、就绪与总线故障的计数。 / Accumulate this frame's freshness, status, readiness and bus-fault counters. | `sensorarrayMeasureReadFdcMatrixFrameSnapshot` |
| `uint32_t sensorarrayMeasureUpdateFdcRuntimeProfiles(sensorarrayState_t *state, const sensorarrayFdcFrameHealth_t *health, uint8_t activeRows)` [215](sensorarrayFdcFrame.c#L215) | 根据本帧活动测点的质量更新运行期参数和缓存调整请求。 / Update runtime profiles and cache-adjustment requests using the active cells' frame health. | `sensorarrayMeasureReadFdcMatrixFrameSnapshot` |
| `uint32_t sensorarrayMeasureCountFdcFrameWarnings(const sensorarrayFdcFrameHealth_t *health, sensorarrayFdcTimingSummary_t *timing, uint8_t activeRows)` [499](sensorarrayFdcFrame.c#L499) | 按活动行统计告警并归入本帧时序/健康摘要。 / Count warnings on active rows and incorporate them in frame accounting. | `sensorarrayMeasureReadFdcMatrixFrameSnapshot` |
| `void sensorarrayMeasureInitFdcMatrixFrame(sensorarrayFdcMatrixFrame_t *frame)` [526](sensorarrayFdcFrame.c#L526) | 清空目标 CAP 帧并为不可用频率/电容预填无效值。 / Clear the CAP frame and initialise unavailable frequency and capacitance fields. | `sensorarrayMeasureReadFdcMatrixFrameSnapshot` |
| `bool sensorarrayFdcMatrixFrameRawAllZero(const sensorarrayFdcMatrixFrame_t *frame)` [546](sensorarrayFdcFrame.c#L546) | 检查目标活动范围内的 CAP 原始读数是否全部为零。 / Check for all-zero CAP raw codes across the supplied active region. | `sensorarrayMeasureReadFdcMatrixFrameSnapshot` |
| `uint64_t sensorarrayMeasureComputeFdcFrameCapTotalPf(sensorarrayFdcMatrixFrame_t *frame)` [558](sensorarrayFdcFrame.c#L558) | 逐点把有效频率换算为总电容 pF，写入逐点有效位图并返回计算耗时 µs。 / Convert each valid frequency to total cell capacitance in pF, update its validity bit and return elapsed µs. | `sensorarrayMeasureReadFdcMatrixFrameSnapshot` |
| `void sensorarrayMeasureMarkFdcMatrixCellEx(sensorarrayFdcMatrixFrame_t *frame, uint8_t sIndex, uint8_t dIndex, uint32_t raw28, double freqHz, const sensorarrayFdcRuntimeChannelConfig_t *config, bool fresh, bool valid, bool warning, bool error, bool notReady, bool zeroBeforeReady, bool zeroAfterDrdy, bool i2cError)` [596](sensorarrayFdcFrame.c#L596) | 按 S/D 写入原始频率码、配置、异常状态和有效/警告标志。 / Write raw frequency code, configuration, fault state and masks for one S/D cell. | `sensorarrayMeasureFillFdcMatrixRow` |
| `esp_err_t sensorarrayMeasureCheckFdcMatrixReady(sensorarrayState_t *state)` [680](sensorarrayFdcFrame.c#L680) | 检查两颗 FDC 的总线与设备状态，拒绝不可投入采样的上下文。 / Check bus/device readiness for the FDCs before accepting an acquisition context. `ESP_ERR_INVALID_ARG` `ESP_ERR_INVALID_STATE` | `sensorarrayMeasureReadFdcMatrixFrameSnapshot` |

### sensorarrayFdcMatrix.c

| C 调用签名 / C signature | 行为及返回 / Behaviour and return | 调用位置 / Called by |
| --- | --- | --- |
| `esp_err_t sensorarrayFdcMatrixEngineInit(sensorarrayFdcMatrixEngine_t *engine, sensorarrayState_t *state)` [7](sensorarrayFdcMatrix.c#L7) | 绑定板级状态并清零引擎的扫描摘要和诊断态。 / Bind board state and initialise engine sweep summary and diagnostic state. `ESP_ERR_INVALID_ARG` | `sensorarrayInitFrontends` |
| `esp_err_t sensorarrayFdcMatrixEngineRunBootSweep(sensorarrayFdcMatrixEngine_t *engine, const sensorarrayScanPlan_t *plan, sensorarrayFdcBootSummary_t *outSummary)` [20](sensorarrayFdcMatrix.c#L20) | 按启动计划调谐 FDC 并复制每个活动行的有效性摘要。 / Tune the FDCs according to the boot plan and return the activity/validity summary. `ESP_ERR_INVALID_ARG` | `sensorarrayRunBootCalibration` |
| `esp_err_t sensorarrayFdcMatrixEngineReadFrame(sensorarrayFdcMatrixEngine_t *engine, const sensorarrayScanPlan_t *plan, sensorarrayFrame_t *frame)` [40](sensorarrayFdcMatrix.c#L40) | 用传入计划运行 FDC 逐行采集并生成新鲜度、有效与错误掩码。 / Execute FDC row acquisition from the supplied plan and produce freshness, validity and error masks. `ESP_ERR_INVALID_ARG` | `sensorarrayMixedRowEngineReadFrame` / `sensorarrayRunOneFrame` |
| `esp_err_t sensorarrayFdcMatrixEngineRunFullRescue(sensorarrayFdcMatrixEngine_t *engine, const char *reason)` [56](sensorarrayFdcMatrix.c#L56) | 在允许的引擎状态下执行整矩阵救援，并返回调谐结果。 / Run full-matrix rescue from an eligible engine state and return tuning outcome. `ESP_ERR_INVALID_ARG` | `sensorarrayRunQueuedFullSweep` / `sensorarrayRunRecoveryLevel` |
| `void sensorarrayFdcMatrixEngineSetDiagnosticMode(sensorarrayFdcMatrixEngine_t *engine, bool enabled)` [67](sensorarrayFdcMatrix.c#L67) | 更新该引擎的诊断运行标志。 / Set the diagnostic operation flag for this engine. | `sensorarrayRunBootCalibration` / `sensorarrayRunQueuedFullSweep` |
| `bool sensorarrayFdcMatrixEngineDiagnosticMode(const sensorarrayFdcMatrixEngine_t *engine)` [75](sensorarrayFdcMatrix.c#L75) | 判断该 FDC 实例是否在诊断模式。 / Return whether the FDC engine is in diagnostic mode. | `sensorarrayRunMainLoop` |
| `const sensorarrayFdcBootSummary_t *sensorarrayFdcMatrixEngineBootSummary(const sensorarrayFdcMatrixEngine_t *engine)` [80](sensorarrayFdcMatrix.c#L80) | 返回引擎持有的启动扫描摘要指针。 / Return the engine-owned boot-sweep summary. | 独立接口 / Standalone entry |
| `sensorarrayState_t *sensorarrayFdcMatrixEngineState(sensorarrayFdcMatrixEngine_t *engine)` [85](sensorarrayFdcMatrix.c#L85) | 返回引擎绑定的板级状态供同一采集所有者使用。 / Return the engine's bound board state for the same acquisition owner. | 独立接口 / Standalone entry |

### sensorarrayFdcQuality.c

| C 调用签名 / C signature | 行为及返回 / Behaviour and return | 调用位置 / Called by |
| --- | --- | --- |
| `sensorarrayFdcSampleStatus_t sensorarrayMeasureMapFdcStatus(Fdc2214CapSampleStatus_t sampleStatus)` [6](sensorarrayFdcQuality.c#L6) | 将 FDC 驱动返回的状态映射成测量层的每样本分类。 / Translate FDC driver status to measurement-layer sample classification. | `sensorarrayMeasureReadFdcSampleDiagWithReader` |
| `const char *sensorarrayMeasureFdcSampleStatusName(sensorarrayFdcSampleStatus_t status)` [29](sensorarrayFdcQuality.c#L29) | 返回样本状态的显示名称。 / Return the display name for sample status. | `sensorarrayFdcSweepEvaluateSample` |
| `esp_err_t sensorarrayMeasureReadFdcSampleDiagWithReader(sensorarrayFdcReadSampleFn_t readFn, Fdc2214CapDevice_t *dev, Fdc2214CapChannel_t ch, bool discardFirst, bool idOk, bool configOk, bool relaxedMode, sensorarrayFdcReadDiag_t *outDiag)` [55](sensorarrayFdcQuality.c#L55) | 调用传入的读函数获取原始样本，并按严格或放宽策略填充诊断。 / Invoke the supplied sample reader and fill diagnostics under strict or relaxed policy. `ESP_ERR_INVALID_ARG` | `sensorarrayMeasureReadFdcSampleDiag` / `sensorarrayMeasureReadFdcSampleDiagRelaxed` |
| `esp_err_t sensorarrayMeasureReadFdcSampleDiag(Fdc2214CapDevice_t *dev, Fdc2214CapChannel_t ch, bool discardFirst, bool idOk, bool configOk, sensorarrayFdcReadDiag_t *outDiag)` [182](sensorarrayFdcQuality.c#L182) | 使用常规读取策略采样并保存 FDC 身份、配置和状态证据。 / Acquire a sample through the normal reader and retain device/configuration/status evidence. | `sensorarrayMeasureReadFdcSample` |
| `esp_err_t sensorarrayMeasureReadFdcSampleDiagRelaxed(Fdc2214CapDevice_t *dev, Fdc2214CapChannel_t ch, bool discardFirst, bool idOk, bool configOk, sensorarrayFdcReadDiag_t *outDiag)` [199](sensorarrayFdcQuality.c#L199) | 以诊断用放宽策略读取样本并保留每项判定。 / Read a sample under the relaxed diagnostic policy, retaining classification evidence. | `sensorarrayFdcSweepReadOneSampleBounded` |
| `esp_err_t sensorarrayMeasureReadFdcSample(Fdc2214CapDevice_t *dev, Fdc2214CapChannel_t ch, bool discardFirst, Fdc2214CapSample_t *outSample)` [216](sensorarrayFdcQuality.c#L216) | 通过诊断读取入口获取指定通道的样本及驱动状态。 / Acquire the selected channel through the diagnostic reader and return the sample and driver status. `ESP_ERR_INVALID_ARG` | 独立接口 / Standalone entry |
| `double sensorarrayMeasureFdcRawToFrequencyHz(uint32_t raw28, uint32_t refClockHz)` [234](sensorarrayFdcQuality.c#L234) | 按给定参考时钟把 28 位转换码映射为 Hz。 / Convert a 28-bit code to Hz using the provided reference clock. | `sensorarrayMeasureFdcComputeFrequencyDiag` |
| `const char *sensorarrayMeasureFdcRefClockSourceName(sensorarrayFdcRefClockSource_t source)` [242](sensorarrayFdcQuality.c#L242) | 返回参考时钟来源的显示名称。 / Return the display name for reference clock source. | `sensorarrayBringupDumpFdcClock` / `sensorarrayBringupVerifyFdcActiveState` |
| `sensorarrayFdcRefClockSource_t sensorarrayMeasureFdcEffectiveRefClockSource(void)` [254](sensorarrayFdcQuality.c#L254) | 返回当前构建配置实际选择的内部或外部 FDC 时钟源。 / Return the internal/external FDC clock source selected by this build. | `sensorarrayBringupDumpFdcClock` / `sensorarrayBringupVerifyFdcActiveState` |
| `uint32_t sensorarrayMeasureFdcEffectiveFclkHz(void)` [263](sensorarrayFdcQuality.c#L263) | 按当前时钟选择给出用于换算的 fCLK 频率。 / Return the effective fCLK frequency selected for conversion. | `sensorarrayBringupDumpFdcInitRegisters` / `sensorarrayBringupDumpFdcClock` |
| `bool sensorarrayMeasureFdcDecodeClockDividers(uint16_t clockDividers, uint8_t *outFinSelCode, uint8_t *outFinFactor, uint16_t *outFrefDivider, const char **outStatus)` [276](sensorarrayFdcQuality.c#L276) | 从 CLOCK_DIVIDERS 字提取 FIN_SEL、输入倍率和参考分频。 / Decode FIN_SEL, input multiplier and reference divider from CLOCK_DIVIDERS. | `sensorarrayBringupDumpFdcClock` / `sensorarrayMeasureFdcFinFactorFromClockDiv` |
| `double sensorarrayMeasureFdcFinFactorFromClockDiv(uint16_t clockDividers)` [318](sensorarrayFdcQuality.c#L318) | 从 CLOCK_DIVIDERS 获取传感器输入频率的倍率。 / Extract the sensor input-frequency factor from CLOCK_DIVIDERS. | `sensorarrayMeasureFdcRaw28ToFreqHz` |
| `double sensorarrayMeasureFdcFrefDividerFromClockDiv(uint16_t clockDividers)` [337](sensorarrayFdcQuality.c#L337) | 从 CLOCK_DIVIDERS 获取参考频率分频比。 / Extract the reference frequency divider from CLOCK_DIVIDERS. | `sensorarrayMeasureFdcRaw28ToFreqHz` / `sensorarrayMeasureFdcEstimateAutoscanReadyTimeoutUsWithSnapshot` |
| `double sensorarrayMeasureFdcRaw28ToFreqHz(uint32_t raw28, uint32_t effectiveFclkHz, uint16_t clockDividers)` [356](sensorarrayFdcQuality.c#L356) | 用指定参考时钟及分频把 28 位转换码换算为输入频率 Hz。 / Convert a 28-bit conversion code to input frequency in Hz using the reference clock and dividers. | `sensorarrayMeasureFillFdcMatrixRow` / `sensorarrayMeasureFdcComputeFrequencyDiag` |
| `bool sensorarrayMeasureFdcComputeFrequencyDiag(uint32_t raw28, uint16_t clockDividers, sensorarrayFdcFrequencyDiag_t *outDiag)` [374](sensorarrayFdcQuality.c#L374) | 为该通道保存计算频率所用的时钟源、寄存器和分频诊断。 / Record clock source, register and divider evidence used for the channel's frequency. | `sensorarrayMeasureFdcRawToSensorFrequencyHz` / `sensorarrayFdcSweepReadKeyRegs` |
| `double sensorarrayMeasureFdcRawToSensorFrequencyHz(uint32_t raw28, uint16_t clockDividers)` [419](sensorarrayFdcQuality.c#L419) | 用有效时钟和通道分频把 raw28 换算为传感器频率。 / Convert raw28 to sensor frequency using the effective clock and channel divider settings. | 独立接口 / Standalone entry |
| `bool sensorarrayMeasureFdcTryCapacitancePf(double frequencyHz, uint32_t inductorUh, double *outCapPf)` [428](sensorarrayFdcQuality.c#L428) | 检查输入频率与电感是否有效，并按 LC 谐振关系计算电容 pF。 / Validate frequency and inductance and derive capacitance in pF from LC resonance. | `sensorarrayFdcSweepCapPfFromFreq` |
| `bool sensorarrayMeasureFdcComputeCapacitancePf(double frequencyHz, double inductorValueUh, double *outCapPf)` [433](sensorarrayFdcQuality.c#L433) | 对有效频率和电感按 LC 谐振公式计算总电容，成功写入 outCapPf。 / Calculate total capacitance from valid frequency and inductance using LC resonance and write outCapPf on success. | `sensorarrayMeasureComputeFdcFrameCapTotalPf` / `sensorarrayMeasureFdcTryCapacitancePf` |

### sensorarrayFdcRead4.c

| C 调用签名 / C signature | 行为及返回 / Behaviour and return | 调用位置 / Called by |
| --- | --- | --- |
| `bool sensorarrayFdcRead4IsDataCompleteGood(const sensorarrayFdcDeviceRead4Result_t *read4, uint8_t reqMask)` [4](sensorarrayFdcRead4.c#L4) | 判断读四通道是否完整且其数据质量满足采纳条件。 / Check completeness and data quality of the four-channel read. | `sensorarrayMeasureFdcRead4DiagnosticName` / `sensorarrayMeasureBuildFdcRead4Result` |
| `bool sensorarrayMeasureFdcReadyStatusSaysReadable(const sensorarrayFdcReadyState_t *ready, uint8_t reqMask)` [30](sensorarrayFdcRead4.c#L30) | 从现有 ready/STATUS 证据判定是否达到所需的读取条件。 / Decide whether existing ready/STATUS evidence meets the required read condition. | `sensorarrayMeasureReadFdcAutoscan4chMasked` / `sensorarrayMeasureMarkFdcNoFreshSamples` |
| `bool sensorarrayMeasureFdcReadyAllowsNormalDataRead(const sensorarrayFdcReadyState_t *ready, uint8_t reqMask)` [47](sensorarrayFdcRead4.c#L47) | 检查当前 ready 类型与策略能否进入正常数据读取。 / Gate the normal data read on the observed ready classification and policy. | `sensorarrayMeasureReadFdcAutoscan4chMasked` / `sensorarrayMeasureFdcRunDeviceEpochAfterSleep` |
| `esp_err_t sensorarrayMeasureReadFdcAutoscan4chMasked(sensorarrayFdcDeviceState_t *fdcState, uint8_t freshMask4, const sensorarrayFdcReadyState_t *ready, sensorarrayFdcAutoscanSamples_t *outSamples)` [82](sensorarrayFdcRead4.c#L82) | 按所需通道掩码读取自动扫描寄存器并保留逐通道状态。 / Read the autoscan register sequence for requested channels and retain per-channel status. `ESP_ERR_INVALID_ARG` `ESP_ERR_INVALID_STATE` | `sensorarrayMeasureReadFdcAutoscan4ch` / `sensorarrayMeasureFdcRunDeviceEpochAfterSleep` |
| `esp_err_t sensorarrayMeasureReadFdcAutoscan4ch(sensorarrayFdcDeviceState_t *fdcState, sensorarrayFdcAutoscanSamples_t *outSamples)` [237](sensorarrayFdcRead4.c#L237) | 使用四通道掩码读取一颗 FDC 当前自动扫描的样本。 / Read a device's current autoscan samples with all four channels selected. | `sensorarrayMeasureDiscardFdcAutoscanRow` |
| `void sensorarrayMeasureMarkFdcNoFreshSamples(sensorarrayFdcAutoscanSamples_t *outSamples, const sensorarrayFdcReadyState_t *ready, bool i2cError)` [246](sensorarrayFdcRead4.c#L246) | 为本轮未取得的新样本设置无效原因和 freshness 掩码。 / Mark missing fresh samples with the current round's invalid reason and freshness mask. | `sensorarrayMeasureFdcRunDeviceEpochAfterSleep` / `sensorarrayMeasureReadFdcMatrixRowSerialEpoch` |
| `const char *sensorarrayMeasureFdcRead4DiagnosticName(const sensorarrayFdcDeviceRead4Result_t *read4)` [308](sensorarrayFdcRead4.c#L308) | 返回读取4诊断的显示名称。 / Return the display name for read 4 diagnostic. | `sensorarrayMeasureBuildFdcRead4Result` / `sensorarrayMeasureFdcLogWorkerTimeout` |
| `void sensorarrayMeasureBuildFdcRead4Result(uint8_t row, uint32_t epochId, sensorarrayFdcDeviceId_t devId, const sensorarrayFdcAutoscanSamples_t *samples, const sensorarrayFdcRuntimeChannelConfig_t configs[4], const sensorarrayFdcReadyState_t *ready, esp_err_t readErr, uint32_t readUs, bool logNormal, sensorarrayFdcDeviceRead4Result_t *outRead4)` [356](sensorarrayFdcRead4.c#L356) | 把就绪证据、读回错误/耗时和逐通道结果组合为本次 read4 结果。 / Combine ready evidence, transfer outcome/timing and individual samples into one read4 result. | `sensorarrayMeasureFdcRunDeviceEpochAfterSleep` |

### sensorarrayFdcReady.c

| C 调用签名 / C signature | 行为及返回 / Behaviour and return | 调用位置 / Called by |
| --- | --- | --- |
| `bool sensorarrayMeasureFdcDeviceReadyForIo(const sensorarrayFdcDeviceState_t *fdcState)` [5](sensorarrayFdcReady.c#L5) | 检查器件、驱动和对应总线是否满足 FDC I/O 前提。 / Check that device, driver and associated bus are usable for FDC I/O. | `sensorarrayMeasureCheckFdcMatrixReady` / `sensorarrayMeasureFdcRunDeviceEpochAfterSleep` |
| `void sensorarrayMeasureLogFdcSecondaryUnavailableOnce(void)` [10](sensorarrayFdcReady.c#L10) | 次 FDC 不可用时只发一次诊断，避免每点重复输出。 / Report secondary FDC unavailability once instead of on every cell. | `sensorarrayMeasureCheckFdcMatrixReady` / `sensorarrayMeasureReadFdcMatrixRowSerialEpoch` |
| `esp_err_t sensorarrayMeasureFdcSetSleepMode(sensorarrayFdcDeviceState_t *fdcState, bool enable, sensorarrayFdcDeviceTiming_t *timing)` [19](sensorarrayFdcReady.c#L19) | 更新器件 CONFIG 的 SLEEP 位，并记录 I²C 开销和真实配置。 / Change the CONFIG sleep bit and account for bus timing and resulting configuration. `ESP_ERR_INVALID_ARG` | `sensorarrayMeasureFdcDeviceLevelResync` / `sensorarrayMeasureFdcRunDeviceEpochAfterSleep` |
| `const char *sensorarrayMeasureFdcRrSequenceName(uint8_t rrSequence)` [53](sensorarrayFdcReady.c#L53) | 返回循环扫描序号的显示名称。 / Return the display name for round-robin sequence. | `sensorarrayMeasureFdcVerifySleepExit` |
| `esp_err_t sensorarrayMeasureFdcVerifySleepExit(sensorarrayState_t *state, uint8_t row, uint32_t epochId, sensorarrayFdcDeviceId_t devId, sensorarrayFdcDeviceTiming_t *timing)` [67](sensorarrayFdcReady.c#L67) | 读取 CONFIG 核对该器件确已退出休眠，将异常关联本行与 epoch。 / Read CONFIG to verify sleep exit and attribute a mismatch to the row and epoch. `ESP_ERR_INVALID_ARG` `ESP_ERR_INVALID_STATE` | `sensorarrayMeasureFdcDeviceLevelResync` / `sensorarrayMeasureFdcRunDeviceEpochAfterSleep` |
| `const char *sensorarrayMeasureFdcReadyKindName(sensorarrayFdcReadyKind_t kind)` [203](sensorarrayFdcReady.c#L203) | 返回就绪类型的显示名称。 / Return the display name for ready kind. | `sensorarrayMeasureFdcLogWorkerTimeout` |
| `bool sensorarrayMeasureFdcShouldLogNormalFrame(uint32_t frameSeq)` [252](sensorarrayFdcReady.c#L252) | 按帧序号和配置间隔决定常态诊断是否到期。 / Apply the configured frame interval to normal diagnostics. | `sensorarrayMeasureFdcShouldLogNormalPollSuccess` / `sensorarrayFdcWaitDeviceReady` |
| `bool sensorarrayMeasureFdcShouldLogNormalPollSuccess(uint32_t frameSeq)` [261](sensorarrayFdcReady.c#L261) | 决定成功的常态就绪检查是否应记入详细日志。 / Decide whether to emit a normal successful readiness poll. | `sensorarrayMeasureFdcRunDeviceEpochAfterSleep` |
| `bool sensorarrayMeasureFdcRead4IsFullSuccess(const sensorarrayFdcDeviceRead4Result_t *read4)` [267](sensorarrayFdcReady.c#L267) | 要求该读四通道操作成功并满足本轮所需掩码。 / Require a successful four-channel read and the mask for the current round. | `sensorarrayMeasureFdcWorkerResultIsGood` / `sensorarrayMeasureFdcFrameTrackerNoteRead` |
| `bool sensorarrayMeasureFdcWorkerResultIsGood(const sensorarrayFdcWorkerResult_t *result, uint8_t expectedRow, uint32_t expectedEpoch, sensorarrayFdcDeviceId_t expectedDev)` [273](sensorarrayFdcReady.c#L273) | 验证单个 worker 的结果与期望行、器件和 epoch 相符且可采纳。 / Check worker result eligibility against the expected row, device and epoch. | `sensorarrayMeasureReadFdcMatrixRowParallelEpoch` |
| `void sensorarrayMeasureFdcFrameTrackerNoteRead(sensorarrayFdcFrameReadTracker_t *tracker, uint8_t row, uint32_t epochId, sensorarrayFdcDeviceId_t devId, const sensorarrayFdcDeviceRead4Result_t *read4, const char *reason)` [286](sensorarrayFdcReady.c#L286) | 把该次 read4 的就绪、错误和耗时计入本帧跟踪器。 / Accumulate ready, error and duration evidence for this read4 in the frame tracker. | `sensorarrayMeasureFdcRunDeviceEpochAfterSleep` |
| `uint32_t sensorarrayMeasureClampU32(uint32_t value, uint32_t minValue, uint32_t maxValue)` [315](sensorarrayFdcReady.c#L315) | 把有界整数限制到指定最小和最大值之间。 / Clamp the unsigned value between the supplied lower and upper bounds. | `sensorarrayMeasureFdcEstimateAutoscanReadyTimeoutUsWithSnapshot` |
| `uint32_t sensorarrayMeasureFdcCeilDivU64(uint64_t numerator, uint64_t denominator)` [326](sensorarrayFdcReady.c#L326) | 执行无符号除法并向上取整，用于时序预算。 / Divide unsigned values with ceiling rounding for timing budgets. | `sensorarrayMeasureFdcEstimateAutoscanReadyTimeoutUsWithSnapshot` |
| `uint32_t sensorarrayMeasureFdcEstimateAutoscanReadyTimeoutUsWithSnapshot(const sensorarrayFdcRuntimeChannelConfig_t configs[4], uint8_t requiredUnreadMask, uint32_t *outEstimatedRoundUs, sensorarrayFdcProfileSnapshot_t *snapshot)` [334](sensorarrayFdcReady.c#L334) | 从当前通道参数快照估算完成所需自动扫描的等待预算。 / Estimate the autoscan readiness budget from a snapshot of channel parameters. | `sensorarrayMeasureApplyFdcCachedRowConfig` / `sensorarrayMeasureFdcEstimateAutoscanReadyTimeoutUsFromConfigs` |
| `uint32_t sensorarrayMeasureFdcEstimateAutoscanReadyTimeoutUsFromConfigs(const sensorarrayFdcRuntimeChannelConfig_t configs[4], uint8_t requiredUnreadMask, uint32_t *outEstimatedRoundUs)` [418](sensorarrayFdcReady.c#L418) | 从四路通道配置和目标 unread 掩码计算扫描轮时长及就绪预算。 / Calculate round duration and ready budget from four channel settings and the required unread mask. | `sensorarrayMeasureFdcEstimateAppliedRowReadyTimeoutUs` / `sensorarrayMeasureFdcRunDeviceEpochAfterSleep` |
| `uint32_t sensorarrayMeasureFdcEstimateAppliedRowReadyTimeoutUs(const sensorarrayState_t *state, sensorarrayFdcDeviceId_t devId, uint8_t requiredUnreadMask, uint32_t *outEstimatedRoundUs)` [429](sensorarrayFdcReady.c#L429) | 基于该行已应用的寄存器参数估算所需 ready 等待 µs。 / Estimate row-ready wait in µs from the settings actually applied to the row. | `sensorarrayMeasureFdcDeviceLevelResync` |
| `const char *sensorarrayMeasureFdcReadyDiagnosticName(const sensorarrayFdcReadyState_t *ready, uint8_t requiredUnreadMask)` [461](sensorarrayFdcReady.c#L461) | 返回就绪诊断的显示名称。 / Return the display name for ready diagnostic. | `sensorarrayMeasureFdcReadStatusAndAckIntbForReady` / `sensorarrayMeasureFdcReadStatusForWatchdogDiagOnly` |
| `void sensorarrayMeasureFdcUpdateReadyTiming(const sensorarrayFdcReadyState_t *ready, sensorarrayFdcDeviceTiming_t *timing, bool fallbackAttempted)` [491](sensorarrayFdcReady.c#L491) | 把实际等待来源、回退是否发生和诊断计数计入器件时序。 / Accumulate actual wait source, fallback use and ready counters into device timing. | `sensorarrayFdcWaitDeviceReady` |
| `const char *sensorarrayMeasureFdcDeviceToken(sensorarrayFdcDeviceId_t devId)` [612](sensorarrayFdcReady.c#L612) | 返回日志使用的主/次 FDC 简写。 / Return the short diagnostic token for the primary/secondary FDC. | `sensorarrayMeasureLogFdcPfuDiag` / `sensorarrayMeasureLogFdcPfuFastProfile` |
| `sensorarrayFdcReadyPolicy_t sensorarrayMeasureFdcConfiguredReadyPolicy(void)` [617](sensorarrayFdcReady.c#L617) | 把当前 Kconfig 选项归并为 FDC ready 检查策略枚举。 / Resolve Kconfig selections to a concrete FDC readiness policy. | `sensorarrayMeasureFdcReadyPolicyForDevice` |
| `sensorarrayFdcReadyPolicy_t sensorarrayMeasureFdcReadyPolicyForDevice(sensorarrayFdcDeviceId_t devId, const sensorarrayFdcWorkerContext_t *ctx)` [630](sensorarrayFdcReady.c#L630) | 结合构建配置与该 worker 的 INTB 能力选择该器件的 ready 路径。 / Select the device's readiness path from build policy and worker INTB availability. | `sensorarrayFdcWaitDeviceReady` |
| `const char *sensorarrayMeasureFdcReadyPolicyName(sensorarrayFdcReadyPolicy_t policy)` [648](sensorarrayFdcReady.c#L648) | 返回就绪策略的显示名称。 / Return the display name for ready policy. | `sensorarrayFdcWaitDeviceReady` |
| `const char *sensorarrayMeasureFdcWatchdogReasonName(sensorarrayFdcWatchdogReason_t reason)` [663](sensorarrayFdcReady.c#L663) | 返回看门狗原因的显示名称。 / Return the display name for watchdog reason. | `sensorarrayFdcHandleRowDeviceWatchdog` |
| `uint32_t sensorarrayMeasureFdcRowDeviceWatchdogHardTimeoutUs(void)` [691](sensorarrayFdcReady.c#L691) | 计算该器件本行等待的硬截止预算。 / Calculate the hard watchdog wait budget for a device in one row. | `sensorarrayFdcHandleRowDeviceWatchdog` / `sensorarrayMeasureFdcComputeActualIntbWaitUs` |
| `esp_err_t sensorarrayFdcHandleRowDeviceWatchdog(sensorarrayState_t *state, sensorarrayFdcDeviceId_t devId, uint8_t row, uint32_t epochId, sensorarrayFdcWatchdogReason_t reason, sensorarrayFdcWorkerResult_t *result, sensorarrayFdcDeviceTiming_t *timing)` [714](sensorarrayFdcReady.c#L714) | 保存超时阶段与现有状态并在允许的诊断边界执行硬看门狗回退。 / Record timeout phase and available status and run watchdog fallback on its admitted diagnostic boundary. | `sensorarrayMeasureFdcRunDeviceEpochAfterSleep` |
| `TickType_t sensorarrayMeasureFdcTicksForUs(uint32_t waitUs)` [766](sensorarrayFdcReady.c#L766) | 把微秒等待预算换算为 FreeRTOS tick。 / Convert a microsecond wait budget to FreeRTOS ticks. | `sensorarrayFdcWaitDeviceReady` |
| `uint32_t sensorarrayMeasureFdcRemainingDeadlineUs(uint64_t deadlineUs)` [776](sensorarrayFdcReady.c#L776) | 用当前单调时钟计算到绝对期限还剩的 µs。 / Return microseconds remaining until an absolute monotonic deadline. | `sensorarrayMeasureFdcComputeActualIntbWaitUs` / `sensorarrayFdcWaitDeviceReady` |
| `const char *sensorarrayMeasureFdcEstimateKindName(uint8_t requiredUnreadMask)` [789](sensorarrayFdcReady.c#L789) | 返回估算类型的显示名称。 / Return the display name for estimate kind. | `sensorarrayFdcWaitDeviceReady` |
| `uint32_t sensorarrayMeasureFdcAddSaturateU32(uint32_t a, uint32_t b)` [801](sensorarrayFdcReady.c#L801) | 执行 uint32 加法并在溢出时饱和。 / Add two uint32 values with saturation on overflow. | `sensorarrayMeasureFdcComputeActualIntbWaitUs` |
| `uint32_t sensorarrayMeasureFdcComputeActualIntbWaitUs(uint32_t estimatedRoundUs, uint64_t rowDeviceDeadlineUs, sensorarrayFdcReadyState_t *ready)` [807](sensorarrayFdcReady.c#L807) | 由进入与退出时间计算本次 INTB 实际等待微秒数。 / Calculate actual INTB wait in µs from entry and return timestamps. | `sensorarrayFdcWaitDeviceReady` |
| `void sensorarrayMeasureFdcDrainCurrentTaskNotify(void)` [857](sensorarrayFdcReady.c#L857) | 清除该任务的旧唤醒通知，为新的 INTB 等待做准备。 / Drain stale task wakeups before arming the next INTB wait. | `sensorarrayMeasureFdcArmCurrentTaskForIntb` |
| `void sensorarrayMeasureFdcArmCurrentTaskForIntb(sensorarrayFdcWorkerContext_t *ctx, uint32_t epochId)` [864](sensorarrayFdcReady.c#L864) | 登记当前等待任务与 INTB 边沿基线，建立本次唤醒关联。 / Register the waiting task and baseline INTB edge count for this wakeup. | `sensorarrayFdcWaitDeviceReady` / `sensorarrayMeasureFdcRunDeviceEpochAfterSleep` |
| `const char *sensorarrayMeasureFdcReadyPollToken(const sensorarrayFdcReadyState_t *ready, uint8_t requiredUnreadMask)` [877](sensorarrayFdcReady.c#L877) | 按 DRDY、unread 掩码及读回状态选择轮询诊断标记。 / Select a poll diagnostic token from DRDY, unread coverage and status evidence. | `sensorarrayMeasureFdcReadStatusAndAckIntbForReady` |
| `void sensorarrayMeasureFdcLogReadyCounts(sensorarrayFdcDeviceId_t devId, uint8_t row, uint32_t epochId, const sensorarrayFdcReadyState_t *ready)` [904](sensorarrayFdcReady.c#L904) | 按行、设备和 epoch 输出 ready 检查的计数。 / Report readiness counters keyed by row, device and epoch. | `sensorarrayFdcWaitDeviceReady` |
| `esp_err_t sensorarrayMeasureFdcReadStatusAndAckIntbForReady(sensorarrayFdcDeviceState_t *fdcState, sensorarrayFdcDeviceId_t devId, uint8_t row, uint32_t epochId, uint8_t requiredUnreadMask, bool afterIntb, bool afterIntbRecheck, bool fallback, bool hardTimeout, bool preWaitCheck, bool logNormalSuccess, sensorarrayFdcReadyState_t *ready, sensorarrayFdcDeviceTiming_t *timing, bool *bestPartialSeen, sensorarrayFdcReadyDecoded_t *bestPartial)` [943](sensorarrayFdcReady.c#L943) | 在需要的就绪判定路径读取 STATUS 并确认相应 INTB，更新本轮证据。 / Read STATUS and acknowledge INTB on the required readiness path, updating round evidence. | `sensorarrayFdcWaitDeviceReady` |
| `esp_err_t sensorarrayMeasureFdcReadStatusForWatchdogDiagOnly(sensorarrayFdcDeviceState_t *fdcState, sensorarrayFdcDeviceId_t devId, uint8_t row, uint32_t epochId, const char *reason, sensorarrayFdcReadyState_t *ready, sensorarrayFdcDeviceTiming_t *timing)` [1063](sensorarrayFdcReady.c#L1063) | 只在看门狗诊断路径读取 STATUS 并记录观察值。 / Read and record STATUS on the watchdog diagnostic path. `ESP_ERR_INVALID_ARG` | 独立接口 / Standalone entry |
| `bool sensorarrayMeasureFdcReadyIsUnreadFullNoDrdy(const sensorarrayFdcReadyState_t *ready, uint8_t requiredUnreadMask)` [1186](sensorarrayFdcReady.c#L1186) | 识别目标 unread 位齐全但 DRDY 未证实的状态组合。 / Recognise full requested unread coverage without confirmed DRDY. | `sensorarrayMeasureFdcClassifyUnreadFullNoDrdySoft` / `sensorarrayFdcWaitDeviceReady` |
| `bool sensorarrayMeasureFdcClassifyUnreadFullNoDrdySoft(sensorarrayFdcDeviceId_t devId, uint8_t row, uint32_t epochId, const char *source, sensorarrayFdcReadyState_t *ready)` [1202](sensorarrayFdcReady.c#L1202) | 对缺少 DRDY 的完整 unread 状态分类，供有限回退使用。 / Classify full-unread/no-DRDY evidence for bounded fallback. | `sensorarrayFdcWaitDeviceReady` |
| `esp_err_t sensorarrayFdcWaitDeviceReady(sensorarrayState_t *state, sensorarrayFdcDeviceId_t devId, uint8_t row, uint32_t epochId, uint32_t frameSeq, uint8_t requiredUnreadMask, uint32_t timeoutUs, uint32_t estimatedRoundUs, uint64_t rowDeviceDeadlineUs, bool allowDirectData, sensorarrayFdcReadyState_t *ready, sensorarrayFdcDeviceTiming_t *timing)` [1261](sensorarrayFdcReady.c#L1261) | 在器件期限内等待 INTB 并按策略核对 STATUS/DRDY，将 ready 证据写回。 / Wait for INTB within the device deadline, examine STATUS/DRDY as required and return ready evidence. `ESP_ERR_INVALID_ARG` `ESP_ERR_INVALID_STATE` `ESP_ERR_TIMEOUT` | `sensorarrayMeasureFdcDeviceLevelResync` / `sensorarrayMeasureFdcRunDeviceEpochAfterSleep` |

### sensorarrayFdcRescue.c

| C 调用签名 / C signature | 行为及返回 / Behaviour and return | 调用位置 / Called by |
| --- | --- | --- |
| `void sensorarrayFdcRescueReset(sensorarrayFdcRescueContext_t *ctx)` [10](sensorarrayFdcRescue.c#L10) | 清空当前恢复上下文的连续失败和冷却状态。 / Reset consecutive-failure and cooldown state in the rescue context. | `sensorarrayApplyMeasurementMode` / `sensorarrayRunAdsRestoreRecoveryAttempt` |
| `static uint8_t sensorarrayFdcRescueFirstFailedRow(const sensorarrayFrame_t *frame)` [17](sensorarrayFdcRescue.c#L17) | 优先返回帧记录的首个失败行，否则寻找首个电容有效位图不完整的行。 / Use the recorded first bad row or find the first row with incomplete capacitance validity. | `sensorarrayFdcRescueTick` |
| `esp_err_t sensorarrayFdcRescueTick(sensorarrayFdcMatrixEngine_t *engine, const sensorarrayFrame_t *frame, sensorarrayFdcRescueContext_t *ctx)` [34](sensorarrayFdcRescue.c#L34) | 依据本帧表现、冷却时间和失败次数选择下一项 FDC 恢复动作。 / Choose the next FDC rescue action from frame health, cooldown and failure counts. `ESP_ERR_INVALID_ARG` | `sensorarrayRuntimeRescueTick` |

### sensorarrayFdcRowEpoch.c

| C 调用签名 / C signature | 行为及返回 / Behaviour and return | 调用位置 / Called by |
| --- | --- | --- |
| `esp_err_t sensorarrayMeasureFdcDeviceLevelResync(sensorarrayState_t *state, uint8_t row, uint32_t epochId, uint32_t frameSeq, sensorarrayFdcDeviceId_t devId, const sensorarrayFdcReadyState_t *readyBefore, sensorarrayFdcReadyState_t *readyAfter, sensorarrayFdcDeviceTiming_t *timing)` [3](sensorarrayFdcRowEpoch.c#L3) | 在设备级别重新同步 FDC 状态，记录操作原因和错误。 / Resynchronise FDC device state and track cause and driver error. `ESP_ERR_INVALID_ARG` `ESP_ERR_INVALID_STATE` | `sensorarrayMeasureFdcRunDeviceEpochAfterSleep` |
| `esp_err_t sensorarrayMeasureFdcPrepareDeviceWhileSleeping(sensorarrayState_t *state, uint8_t row, uint32_t epochId, uint32_t frameSeq, sensorarrayFdcDeviceId_t devId, sensorarrayFdcRuntimeChannelConfig_t outConfigs[4], sensorarrayFdcDeviceTiming_t *timing)` [124](sensorarrayFdcRowEpoch.c#L124) | 在器件休眠窗口准备本轮参数与就绪证据。 / Prepare the round's settings and ready evidence while the device sleeps. `ESP_ERR_INVALID_ARG` | `sensorarrayMeasureFdcRunDeviceEpochAfterSleep` / `sensorarrayMeasureReadFdcMatrixRowParallelEpoch` |
| `esp_err_t sensorarrayMeasureFdcDrainStaleUnread(sensorarrayFdcDeviceState_t *fdcState, sensorarrayFdcDeviceId_t devId, uint8_t row, uint32_t epochId, const sensorarrayFdcReadyState_t *ready, sensorarrayFdcDeviceTiming_t *timing)` [175](sensorarrayFdcRowEpoch.c#L175) | 清理前一轮残留的 unread 数据，使其不能作为新的行样本。 / Drain unread data left by an older round before the new row can claim freshness. `ESP_ERR_INVALID_ARG` | `sensorarrayMeasureFdcRunDeviceEpochAfterSleep` |
| `esp_err_t sensorarrayMeasureFdcRunDeviceEpochAfterSleep(sensorarrayState_t *state, uint8_t row, uint32_t epochId, uint32_t frameSeq, sensorarrayFdcDeviceId_t devId, sensorarrayFdcAutoscanSamples_t *outSamples, sensorarrayFdcRuntimeChannelConfig_t outConfigs[4], sensorarrayFdcReadyState_t *outReady, sensorarrayFdcDeviceRead4Result_t *outRead4, sensorarrayFdcDeviceTiming_t *timing, sensorarrayFdcFrameReadTracker_t *readTracker, const char *readReason, uint64_t rowDeviceDeadlineOverrideUs)` [275](sensorarrayFdcRowEpoch.c#L275) | 唤醒指定器件、确认本轮 ready，再对该行读四通道并记录每个阶段。 / Wake one FDC, establish readiness for its epoch and read its four row channels with timing. `ESP_ERR_INVALID_ARG` `ESP_ERR_INVALID_STATE` | `sensorarrayMeasureFdcWorkerTask` / `sensorarrayMeasureReadFdcMatrixRowSerialEpoch` |
| `EventBits_t sensorarrayMeasureFdcWorkerAckBit(sensorarrayFdcDeviceId_t devId)` [1012](sensorarrayFdcRowEpoch.c#L1012) | 返回对应 FDC worker 的 ACK 事件位。 / Return the ACK event bit assigned to this FDC worker. | `sensorarrayMeasureFdcWorkerTask` |
| `EventBits_t sensorarrayMeasureFdcWorkerDoneBit(sensorarrayFdcDeviceId_t devId)` [1018](sensorarrayFdcRowEpoch.c#L1018) | 返回对应 FDC worker 的 DONE 事件位。 / Return the DONE event bit assigned to this FDC worker. | `sensorarrayMeasureFdcWorkerTask` |
| `bool sensorarrayMeasureFdcWorkersQuiescent(void)` [1024](sensorarrayFdcRowEpoch.c#L1024) | 核对两个 worker 是否均已释放本轮引用，供模式切换与重探准入。 / Check release/quiescence of both workers for reconfiguration and device probes. | `sensorarrayMeasureReadFdcMatrixRowParallelEpoch` / `sensorarrayMeasurePrepareFdcMatrixPath` |
| `bool sensorarrayMeasureFdcWorkersQuarantined(void)` [1030](sensorarrayFdcRowEpoch.c#L1030) | 检查是否有任何 worker 仍处于隔离状态。 / Check whether either worker remains quarantined. | `sensorarrayMeasureReadFdcMatrixRowParallelEpoch` / `sensorarrayMeasureReadFdcMatrixFrameSnapshot` |
| `static TickType_t sensorarrayFdcDeadlineTicks(uint64_t deadlineUs)` [1038](sensorarrayFdcRowEpoch.c#L1038) | 将单调截止时间换算为余下可等待的 RTOS tick。 / Convert a monotonic deadline into the remaining RTOS wait ticks. | `sensorarrayFdcWorkersWait` / `sensorarrayMeasureFdcWorkerTask` |
| `static bool sensorarrayFdcWorkersWait(bool released, uint64_t deadlineUs)` [1048](sensorarrayFdcRowEpoch.c#L1048) | 检查两个 worker 的阶段或 release 标志，等待受绝对期限限制。 / Wait on the required phase or release evidence from both workers up to the absolute deadline. | `sensorarrayFdcCancelAndReclaim` / `sensorarrayMeasureReadFdcMatrixRowParallelEpoch` |
| `static bool sensorarrayFdcCancelAndReclaim(uint64_t reclaimDeadlineUs)` [1071](sensorarrayFdcRowEpoch.c#L1071) | 请求取消并等待释放证据；在收尾期限仍不能收回时隔离槽。 / Request cancellation, await positive release evidence and quarantine unclaimed slots at the cleanup deadline. | `sensorarrayMeasureReadFdcMatrixRowParallelEpoch` |
| `void sensorarrayMeasureFdcWorkerTask(void *arg)` [1093](sensorarrayFdcRowEpoch.c#L1093) | 依据持久槽的阶段与 epoch 领取作业、等待 RUN、执行器件 I/O，最后确认完成或取消。 / Take a persistent slot by phase/epoch, wait for RUN, perform device I/O and acknowledge completion or cancellation. | `sensorarrayMeasureEnsureFdcWorkers` |
| `esp_err_t sensorarrayMeasureEnsureFdcWorkers(void)` [1163](sensorarrayFdcRowEpoch.c#L1163) | 为主次器件建立持久 worker、槽与同步对象并检查创建结果。 / Establish persistent workers, slots and signalling for both devices and check creation outcomes. `ESP_ERR_NO_MEM` | `sensorarrayMeasureReadFdcMatrixRowParallelEpoch` / `sensorarrayMeasureReadFdcMatrixFrameSnapshot` |
| `static uint64_t sensorarrayMeasureFdcSummarySpanUs(uint64_t startA, uint64_t doneA, uint64_t startB, uint64_t doneB)` [1254](sensorarrayFdcRowEpoch.c#L1254) | 计算两个 worker 时间区间覆盖的总跨度。 / Calculate elapsed span covering both workers' intervals. | `sensorarrayMeasureAccumulateRowEpochTiming` |
| `static uint64_t sensorarrayMeasureFdcSummaryOverlapUs(uint64_t serialUs, uint64_t spanUs)` [1267](sensorarrayFdcRowEpoch.c#L1267) | 由两组起止时间计算双方实际重叠时长。 / Calculate the actual overlap between the two start/end intervals. | `sensorarrayMeasureAccumulateRowEpochTiming` |
| `void sensorarrayMeasureAccumulateRowEpochTiming(sensorarrayFdcTimingSummary_t *summary, const sensorarrayFdcRowTiming_t *rowTiming, const sensorarrayFdcDeviceTiming_t *primaryTiming, const sensorarrayFdcDeviceTiming_t *secondaryTiming)` [1273](sensorarrayFdcRowEpoch.c#L1273) | 将双 worker 的时间、重叠及每阶段开销累入帧摘要。 / Accumulate worker duration, overlap and phase costs from the row into the frame summary. | `sensorarrayMeasureReadFdcMatrixFrameSnapshot` |
| `esp_err_t sensorarrayMeasureSelectFdcRowWhileSleeping(sensorarrayState_t *state, uint8_t row, uint32_t epochId, uint32_t frameSeq, sensorarrayFdcRowTiming_t *rowTiming)` [1517](sensorarrayFdcRowEpoch.c#L1517) | 在两个前端允许的休眠状态下切换行并执行建立等待。 / Switch rows and allow settling while the frontend sleep state admits it. | `sensorarrayMeasureReadFdcMatrixRowSerialEpoch` / `sensorarrayMeasureReadFdcMatrixRowParallelEpoch` |
| `esp_err_t sensorarrayMeasureReadFdcMatrixRowSerialEpoch(sensorarrayState_t *state, uint8_t row, uint32_t epochId, uint32_t frameSeq, sensorarrayFdcAutoscanSamples_t *primarySamples, sensorarrayFdcAutoscanSamples_t *secondarySamples, sensorarrayFdcRuntimeChannelConfig_t runtimeConfigs[2][4], sensorarrayFdcDeviceTiming_t *primaryTiming, sensorarrayFdcDeviceTiming_t *secondaryTiming, sensorarrayFdcRowTiming_t *rowTiming, sensorarrayFdcFrameReadTracker_t *readTracker)` [1563](sensorarrayFdcRowEpoch.c#L1563) | 在串行诊断/回退路径处理同一行的两颗 FDC，并保存两组独立结果。 / Acquire the two FDCs for one row in the serial diagnostic/fallback path and retain independent results. `ESP_ERR_INVALID_ARG` `ESP_ERR_INVALID_STATE` | `sensorarrayMeasureReadFdcMatrixFrameSnapshot` |
| `void sensorarrayMeasureFdcLogWorkerTimeout(const sensorarrayFdcWorkerTrace_t *trace, const char *timeoutStage, const sensorarrayFdcWorkerResult_t *result)` [1685](sensorarrayFdcRowEpoch.c#L1685) | 记录超时阶段以及具有行/设备/epoch 身份的 worker 轨迹。 / Report the timeout phase and worker trace with row, device and epoch. | 独立接口 / Standalone entry |
| `void sensorarrayMeasureFdcLogLateDone(const sensorarrayFdcWorkerTrace_t *trace)` [1725](sensorarrayFdcRowEpoch.c#L1725) | 记录晚到的 DONE 与工作轨迹以供判断旧作业。 / Report a late DONE and its job trace for stale-work diagnosis. | 独立接口 / Standalone entry |
| `void sensorarrayMeasureFdcDiscardWorkerResult(sensorarrayFdcWorkerResult_t *result, sensorarrayFdcAutoscanSamples_t *samples, sensorarrayFdcWorkerTrace_t *trace, const char *reason)` [1747](sensorarrayFdcRowEpoch.c#L1747) | 清除丢弃作业的采集/诊断结果，防止迟到样本合入下一轮。 / Clear results for a discarded job so late samples cannot enter another epoch. | `sensorarrayMeasureReadFdcMatrixRowParallelEpoch` |
| `void sensorarrayMeasureFdcDeferDeviceRepairAfterParallel(sensorarrayState_t *state, uint8_t row, uint32_t epochId, uint32_t frameSeq, sensorarrayFdcDeviceId_t devId, sensorarrayFdcAutoscanSamples_t *samples, const sensorarrayFdcWorkerResult_t *result, sensorarrayFdcRowTiming_t *rowTiming, const char *reason)` [1769](sensorarrayFdcRowEpoch.c#L1769) | 记录并行处理后的器件修复需求，交由后续安全边界处理。 / Record device repair required after parallel work for a later safe boundary. | `sensorarrayMeasureReadFdcMatrixRowParallelEpoch` |
| `uint32_t sensorarrayMeasureFdcAbsDeltaUs(uint64_t a, uint64_t b)` [1835](sensorarrayFdcRowEpoch.c#L1835) | 计算两时间戳绝对差，限制于诊断字段可表示范围。 / Compute the absolute time difference bounded to the diagnostic field. | `sensorarrayMeasureFdcFinalizeParallelTiming` / `sensorarrayMeasureReadFdcMatrixRowParallelEpoch` |
| `uint64_t sensorarrayMeasureFdcSpanUs(uint64_t startA, uint64_t doneA, uint64_t startB, uint64_t doneB)` [1844](sensorarrayFdcRowEpoch.c#L1844) | 计算两个工作区间共同覆盖的首尾跨度。 / Calculate total elapsed span covering two work intervals. | `sensorarrayMeasureFdcFinalizeParallelTiming` |
| `void sensorarrayMeasureFdcFinalizeParallelTiming(uint8_t row, uint32_t epochId, const sensorarrayFdcDeviceTiming_t *primaryTiming, const sensorarrayFdcDeviceTiming_t *secondaryTiming, sensorarrayFdcRowTiming_t *rowTiming, esp_err_t primaryErr, esp_err_t secondaryErr)` [1857](sensorarrayFdcRowEpoch.c#L1857) | 将两个 worker 的开始、结束、重叠及串行等效耗时合并成行统计。 / Combine both workers' start/end, overlap and serial-equivalent times into row timing. | `sensorarrayMeasureReadFdcMatrixRowParallelEpoch` |
| `esp_err_t sensorarrayMeasureReadFdcMatrixRowParallelEpoch(sensorarrayState_t *state, uint8_t row, uint32_t epochId, uint32_t frameSeq, sensorarrayFdcAutoscanSamples_t *primarySamples, sensorarrayFdcAutoscanSamples_t *secondarySamples, sensorarrayFdcRuntimeChannelConfig_t runtimeConfigs[2][4], sensorarrayFdcDeviceTiming_t *primaryTiming, sensorarrayFdcDeviceTiming_t *secondaryTiming, sensorarrayFdcRowTiming_t *rowTiming, sensorarrayFdcFrameReadTracker_t *readTracker, bool *outWorkerTimeout)` [1956](sensorarrayFdcRowEpoch.c#L1956) | 协调两颗 FDC 的一行并行读；截至期限回收有证据释放的槽，其余隔离。 / Coordinate concurrent reads for one row on both FDCs, reclaim slots proven released by deadline and quarantine unresolved work. `ESP_ERR_INVALID_ARG` `ESP_ERR_INVALID_STATE` `ESP_ERR_TIMEOUT` | `sensorarrayMeasureReadFdcMatrixFrameSnapshot` |
| `esp_err_t sensorarrayMeasureReadFdcRuntimeChannelConfigs(sensorarrayState_t *state, sensorarrayFdcRuntimeChannelConfig_t configs[2][4])` [2344](sensorarrayFdcRowEpoch.c#L2344) | 读取主次器件各四个通道的实际参数作为运行期快照。 / Snapshot the running parameters of four channels on each FDC. `ESP_ERR_INVALID_ARG` | 独立接口 / Standalone entry |
| `esp_err_t sensorarrayMeasureDiscardFdcAutoscanRow(sensorarrayState_t *state, uint8_t sIndex, sensorarrayFdcDeviceId_t devId, uint8_t discardCount, const char *reason)` [2413](sensorarrayFdcRowEpoch.c#L2413) | 在指定器件与行上丢弃要求数量的自动扫描结果。 / Discard the requested number of autoscan rounds for the selected device/row. `ESP_ERR_INVALID_ARG` `ESP_ERR_INVALID_STATE` | 独立接口 / Standalone entry |

### sensorarrayFdcSweep.c

| C 调用签名 / C signature | 行为及返回 / Behaviour and return | 调用位置 / Called by |
| --- | --- | --- |
| `static const char *sensorarrayFdcSweepDevName(sensorarrayFdcDeviceId_t devId)` [162](sensorarrayFdcSweep.c#L162) | 返回器件的显示名称。 / Return the display name for device. | `sensorarrayFdcSweepApplyDirectSafeLock` / `sensorarrayFdcSweepCaptureCandidate` |
| `static const char *sensorarrayFdcSweepDevFullName(sensorarrayFdcDeviceId_t devId)` [167](sensorarrayFdcSweep.c#L167) | 返回器件完整的显示名称。 / Return the display name for device full. | `sensorarrayFdcSweepApplyDirectSafeLock` / `sensorarrayFdcSweepCaptureCandidate` |
| `static const char *sensorarrayFdcBootQualityName(sensorarrayFdcBootQuality_t quality)` [172](sensorarrayFdcSweep.c#L172) | 返回启动质量的显示名称。 / Return the display name for boot quality. | `sensorarrayFdcSweepRunBoot` |
| `static uint8_t sensorarrayFdcSweepDLineForDeviceChannel(sensorarrayFdcDeviceId_t devId, Fdc2214CapChannel_t channel)` [185](sensorarrayFdcSweep.c#L185) | 由主/次 FDC 和 0..3 通道定位物理 D 线。 / Resolve a physical D line from the primary/secondary FDC and channel 0..3. | `sensorarrayFdcSweepRunDevice` / `sensorarrayFdcSweepBuildRowDriveCandidate` |
| `static uint8_t sensorarrayFdcSweepRegForChannel(uint8_t base, Fdc2214CapChannel_t channel)` [192](sensorarrayFdcSweep.c#L192) | 由通道 0 的基址计算该通道的寄存器地址。 / Derive the register address for the specified channel from the channel-zero base. | `sensorarrayFdcSweepDumpDeviceRegs` / `sensorarrayFdcSweepReadKeyRegs` |
| `static double sensorarrayFdcSweepAbsDouble(double value)` [197](sensorarrayFdcSweep.c#L197) | 返回浮点数的绝对值。 / Return the absolute value of the floating-point operand. | `sensorarrayFdcSweepCandidateHugeJump` |
| `static uint32_t sensorarrayFdcSweepClampQualitySamples(uint32_t requested)` [202](sensorarrayFdcSweep.c#L202) | 将候选质量评估样本数限制在支持的范围。 / Bound the number of samples used to assess one candidate. | `sensorarrayFdcSweepCaptureCandidate` |
| `static uint16_t sensorarrayFdcSweepClampDrive(uint16_t drive)` [213](sensorarrayFdcSweep.c#L213) | 将驱动电流码限制在 FDC 有效范围。 / Clamp the drive-current code to the FDC's valid range. | `sensorarrayFdcSweepBuildFastDriveOrderForCell` / `sensorarrayFdcSweepApplyCandidateToDevice` |
| `static uint16_t sensorarrayFdcSweepAddDriveOffset(uint16_t center, int32_t offset)` [229](sensorarrayFdcSweep.c#L229) | 对中心驱动码加搜索偏移并限制合法范围。 / Apply a search offset to the centre drive code and clamp to a legal setting. | `sensorarrayFdcSweepBuildFastDriveOrderForCell` |
| `static bool sensorarrayFdcSweepCapPfFromFreq(double freqHz, double *outCapPf)` [241](sensorarrayFdcSweep.c#L241) | 按通道电感与谐振频率计算电容 pF。 / Compute capacitance in pF from resonant frequency and inductance. | `sensorarrayFdcSweepUpdateCalibrationFromCandidate` / `sensorarrayFdcSweepLogCapResult` |
| `static sensorarrayFdcCellCalibration_t *sensorarrayFdcSweepCell(uint8_t sIndex, uint8_t dIndex)` [252](sensorarrayFdcSweep.c#L252) | 返回指定 S/D 位置的逐点调谐记录。 / Return the per-cell tuning entry for the specified S/D position. | `sensorarrayFdcSweepGetCellCalibration` / `sensorarrayFdcSweepInitCalibrationCell` |
| `sensorarrayFdcCellCalibration_t *sensorarrayFdcSweepGetCellCalibration(uint8_t sIndex, uint8_t dIndex)` [260](sensorarrayFdcSweep.c#L260) | 返回指定 S/D 的调谐记录，以便取得该点的启动值或最近有效值。 / Return the tuning record for the given S/D to inspect boot or last-good parameters. | `sensorarrayMeasureUpdateFdcRuntimeProfiles` |
| `esp_err_t sensorarrayFdcSweepRequestForceFullSweepAll(void)` [266](sensorarrayFdcSweep.c#L266) | 登记整矩阵强制重扫请求，交由调度边界领取。 / Post a full-matrix forced sweep request for the scheduler to consume. | `sensorarrayFdcRescueTick` |
| `bool sensorarrayFdcSweepConsumeForceFullSweepAll(void)` [294](sensorarrayFdcSweep.c#L294) | 领取并清除待处理的全矩阵重扫标志。 / Consume and clear the pending full-matrix sweep flag. | `sensorarrayRunQueuedFullSweep` |
| `void sensorarrayFdcSweepDiscardPendingForConfigChange(void)` [303](sensorarrayFdcSweep.c#L303) | 配置改变时清理旧身份下待执行的强制扫描请求。 / Discard pending forced-sweep requests from the superseded configuration. | `sensorarrayApplyMeasurementMode` / `sensorarrayApplyPendingCommands` |
| `esp_err_t sensorarrayFdcSweepRequestForceFullSweepCell(uint8_t sIndex, uint8_t dIndex)` [314](sensorarrayFdcSweep.c#L314) | 校验 S/D 地址后登记该测点触发的强制重扫。 / Validate S/D coordinates and post a cell-triggered forced sweep. `ESP_ERR_INVALID_ARG` | 独立接口 / Standalone entry |
| `static bool sensorarrayFdcSweepConsumeForceFullSweep(uint8_t sIndex, uint8_t dIndex)` [333](sensorarrayFdcSweep.c#L333) | 领取并清除与指定 S/D 匹配的强制重扫请求。 / Consume the pending forced sweep matching the supplied S/D coordinates. | `sensorarrayFdcSweepMeasureCell` |
| `static void sensorarrayFdcSweepInitCalibrationCell(uint8_t sIndex, uint8_t dIndex)` [344](sensorarrayFdcSweep.c#L344) | 用测点位置和默认调谐参数建立一条校准记录。 / Initialise one calibration record with its position and default tuning values. | `sensorarrayFdcSweepInitCalibrationTable` |
| `static void sensorarrayFdcSweepInitCalibrationTable(void)` [365](sensorarrayFdcSweep.c#L365) | 逐个建立所有 S/D 的默认校准记录。 / Initialise a default calibration record for every S/D position. | `sensorarrayFdcSweepEnsureCalibrationTable` / `sensorarrayFdcSweepRunBoot` |
| `static void sensorarrayFdcSweepEnsureCalibrationTable(void)` [375](sensorarrayFdcSweep.c#L375) | 按需建立并初始化逐点校准表。 / Ensure that the per-cell calibration table has been initialised. | `sensorarrayFdcSweepMeasureCell` / `sensorarrayFdcSweepRow` |
| `const char *sensorarrayFdcSweepReasonName(sensorarrayFdcSweepReason_t reason)` [382](sensorarrayFdcSweep.c#L382) | 返回原因的显示名称。 / Return the display name for reason. | 独立接口 / Standalone entry |
| `const char *sensorarrayFdcSweepFaultReasonName(sensorarrayFdcFaultReason_t reason)` [408](sensorarrayFdcSweep.c#L408) | 返回故障原因的显示名称。 / Return the display name for fault reason. | 独立接口 / Standalone entry |
| `bool sensorarrayFdcSweepIsRescueInProgress(void)` [434](sensorarrayFdcSweep.c#L434) | 返回当前是否已有恢复扫描占用工作状态。 / Report whether rescue scanning is already in progress. | 独立接口 / Standalone entry |
| `void sensorarrayFdcSweepReportAllInvalidFrame(uint64_t validMask, uint64_t errorMask, uint32_t zeroRawCount, uint32_t notReadyCount, uint32_t zeroBeforeReadyCount, uint32_t zeroAfterDrdyCount, uint32_t i2cErrorCount, uint32_t diagReadyButRejectedCount, uint32_t intbMissButStatusReadyCount, uint32_t statusFallbackAcceptedCount, uint32_t waitBudgetTooShortCount, uint32_t levelLowButEdgeMissCount, uint32_t actualDataReadSkippedDespiteStatusReadyCount)` [439](sensorarrayFdcSweep.c#L439) | 记录全无效帧及其序号以供恢复策略判定。 / Record an all-invalid frame and sequence for rescue policy. | `sensorarrayMeasureReadFdcMatrixFrameSnapshot` |
| `static uint32_t sensorarrayFdcSweepNowMs(void)` [581](sensorarrayFdcSweep.c#L581) | 取得单调时钟的当前毫秒计数。 / Read the current monotonic time in milliseconds. | `sensorarrayFdcSweepRunCellSweep` / `sensorarrayFdcSweepMeasureCell` |
| `static uint32_t sensorarrayFdcSweepStepTimeoutMs(void)` [586](sensorarrayFdcSweep.c#L586) | 给出单次搜索步骤允许的毫秒预算。 / Return the millisecond budget for one tuning search step. | `sensorarrayFdcSweepCaptureCandidate` |
| `static uint32_t sensorarrayFdcSweepTotalTimeoutMs(void)` [592](sensorarrayFdcSweep.c#L592) | 给出整个搜索/恢复操作的总毫秒预算。 / Return the overall search/rescue budget in milliseconds. | `sensorarrayFdcSweepRunCellSweep` / `sensorarrayFdcSweepRow` |
| `static uint32_t sensorarrayFdcSweepRemainingMsUntil(int64_t deadlineUs)` [598](sensorarrayFdcSweep.c#L598) | 计算距绝对截止点剩余的毫秒数。 / Calculate the milliseconds remaining until the absolute deadline. | `sensorarrayFdcSweepReadOneSampleBounded` / `sensorarrayFdcSweepCaptureCandidate` |
| `static TickType_t sensorarrayFdcSweepMsToTicksAtLeastOne(uint32_t delayMs)` [607](sensorarrayFdcSweep.c#L607) | 将非零毫秒等待向可调度的 RTOS tick 换算。 / Convert millisecond delay to a schedulable RTOS tick count. | `sensorarrayFdcSweepDelayMs` |
| `static void sensorarrayFdcSweepDelayMs(uint32_t delayMs)` [613](sensorarrayFdcSweep.c#L613) | 让出任务，按给定毫秒预算等待 FDC 建立。 / Yield the task for the supplied millisecond delay. | `sensorarrayFdcSweepReadOneSampleBounded` / `sensorarrayFdcSweepApplyDirectSafeLock` |
| `static esp_err_t sensorarrayFdcSweepDumpDeviceRegs(sensorarrayFdcDeviceState_t *fdc, const char *stage, const char *reason)` [623](sensorarrayFdcSweep.c#L623) | 读取单颗 FDC 的诊断寄存器并给每条记录附阶段与原因。 / Read one FDC's diagnostic registers and attribute output to stage and cause. `ESP_ERR_INVALID_STATE` | `sensorarrayFdcSweepDumpAllDeviceRegs` / `sensorarrayFdcSweepRestoreAutoscan` |
| `esp_err_t sensorarrayFdcSweepDumpAllDeviceRegs(sensorarrayState_t *state, const char *stage, const char *reason)` [723](sensorarrayFdcSweep.c#L723) | 对主次 FDC 各自尝试寄存器转储并汇总错误。 / Attempt register dumps for both FDCs and combine the error outcome. `ESP_ERR_INVALID_ARG` | `sensorarrayFdcSweepRunFullRescueAll` / `sensorarrayFdcSweepRunBoot` |
| `static esp_err_t sensorarrayFdcSweepLogBootDeviceRegs(sensorarrayFdcDeviceState_t *fdc, const char *deviceName)` [742](sensorarrayFdcSweep.c#L742) | 记录启动诊断所用的单器件配置和身份寄存器。 / Report device configuration and identity registers used in boot diagnostics. `ESP_ERR_INVALID_STATE` | `sensorarrayFdcSweepRunBoot` |
| `static const sensorarrayFdcDeglitchCandidate_t *sensorarrayFdcSweepFindDeglitchCandidate(uint8_t deglitchCode)` [774](sensorarrayFdcSweep.c#L774) | 按寄存器码在去毛刺带宽候选表中查找对应项。 / Look up a deglitch-bandwidth candidate by register code. | `sensorarrayFdcSweepDeglitchNameFromCode` / `sensorarrayFdcSweepDeglitchEnum` |
| `static const char *sensorarrayFdcSweepDeglitchNameFromCode(uint8_t deglitchCode)` [786](sensorarrayFdcSweep.c#L786) | 把寄存器去毛刺码映射为诊断用的带宽名称。 / Map a deglitch register code to a human-readable bandwidth name. | `sensorarrayFdcSweepRestoreAutoscan` |
| `static Fdc2214CapDeglitch_t sensorarrayFdcSweepDeglitchEnum(uint8_t deglitchCode)` [793](sensorarrayFdcSweep.c#L793) | 将选中的滤波器候选映射为驱动接受的枚举值。 / Map the chosen filter candidate to the driver's bandwidth enum. | `sensorarrayFdcSweepApplyDirectSafeLock` / `sensorarrayFdcSweepRestoreAutoscan` |
| `static void sensorarrayFdcSweepAddUniqueDrive(uint16_t *drives, uint32_t *count, uint32_t capacity, uint16_t drive)` [800](sensorarrayFdcSweep.c#L800) | 只在容量允许且列表尚无此值时添加驱动电流码。 / Append a drive-current code only when unique and capacity remains. | `sensorarrayFdcSweepBuildDriveOrder` / `sensorarrayFdcSweepBuildFastDriveOrderForCell` |
| `static uint32_t sensorarrayFdcSweepBuildDriveOrder(const sensorarrayFdcSweepProfile_t *profile, uint16_t *drives, uint32_t capacity)` [818](sensorarrayFdcSweep.c#L818) | 按 profile 与中心值构造去重的电流候选搜索顺序。 / Build a deduplicated search order of drive-current candidates for the profile. | 无直接调用记录 / No direct call found |
| `static void sensorarrayFdcSweepAddUniqueDeglitch(const sensorarrayFdcDeglitchCandidate_t **order, uint32_t *count, uint32_t capacity, const sensorarrayFdcDeglitchCandidate_t *candidate)` [833](sensorarrayFdcSweep.c#L833) | 仅添加尚未出现且有容量的去毛刺带宽候选。 / Append a deglitch bandwidth candidate only once and within capacity. | `sensorarrayFdcSweepBuildDeglitchOrder` / `sensorarrayFdcSweepBuildFastDeglitchOrderForCell` |
| `static uint32_t sensorarrayFdcSweepBuildDeglitchOrder(double predictedFreqHz, const sensorarrayFdcDeglitchCandidate_t **order, uint32_t capacity)` [850](sensorarrayFdcSweep.c#L850) | 按 profile 和已有参数建立滤波带宽的候选顺序。 / Order candidate filter bandwidths from the profile and existing settings. | `sensorarrayFdcSweepBuildFullDeglitchOrderForCell` |
| `static esp_err_t sensorarrayFdcSweepReadKeyRegs(Fdc2214CapDevice_t *dev, Fdc2214CapChannel_t channel, sensorarrayFdcSweepCandidateResult_t *candidate)` [880](sensorarrayFdcSweep.c#L880) | 读回验证候选工作状态所需的关键寄存器。 / Read the key hardware registers used to verify a candidate state. `ESP_ERR_INVALID_ARG` | `sensorarrayFdcSweepCaptureCandidate` |
| `static bool sensorarrayFdcSweepCandidateIsWorking(const sensorarrayFdcSweepCandidateResult_t *candidate)` [961](sensorarrayFdcSweep.c#L961) | 以有效读数和质量证据判定候选是否能正常采集。 / Decide whether measured validity and quality qualify a candidate for operation. | `sensorarrayFdcSweepCandidatePriority` / `sensorarrayFdcSweepCaptureCandidate` |
| `static bool sensorarrayFdcSweepCandidateIsFallbackUsable(const sensorarrayFdcSweepCandidateResult_t *candidate)` [966](sensorarrayFdcSweep.c#L966) | 检查候选是否至少达到允许作为回退配置的条件。 / Check whether a candidate meets the minimum fallback criteria. | `sensorarrayFdcSweepCandidatePriority` / `sensorarrayFdcSweepCaptureCandidate` |
| `static bool sensorarrayFdcSweepCandidateHasRawFrequency(const sensorarrayFdcSweepCandidateResult_t *candidate)` [981](sensorarrayFdcSweep.c#L981) | 检查候选是否包含实际采得的原始码和频率证据。 / Check that the candidate has measured raw-code and frequency evidence. | `sensorarrayFdcSweepCandidatePriority` / `sensorarrayFdcSweepRunCellSweep` |
| `static uint8_t sensorarrayFdcSweepCandidatePriority(const sensorarrayFdcSweepCandidateResult_t *candidate)` [989](sensorarrayFdcSweep.c#L989) | 确定一个候选所属的质量优先级。 / Assign a quality-priority class to the candidate. | `sensorarrayFdcSweepCandidateBetter` / `sensorarrayFdcSweepFallbackBetter` |
| `static int32_t sensorarrayFdcSweepCandidateScore(const sensorarrayFdcSweepCandidateResult_t *candidate)` [1003](sensorarrayFdcSweep.c#L1003) | 依据实际样本统计计算用于比较候选配置的分数。 / Score the candidate from its measured sample statistics. | `sensorarrayFdcSweepCandidateBetter` / `sensorarrayFdcSweepFallbackBetter` |
| `static bool sensorarrayFdcSweepCandidateBetter(const sensorarrayFdcSweepCandidateResult_t *candidate, const sensorarrayFdcSweepCandidateResult_t *best)` [1062](sensorarrayFdcSweep.c#L1062) | 按质量优先级和得分比较当前候选与已有最佳项。 / Compare the candidate with the current best by priority and measured score. | `sensorarrayFdcSweepRunCellSweep` |
| `static bool sensorarrayFdcSweepFallbackBetter(const sensorarrayFdcSweepCandidateResult_t *candidate, const sensorarrayFdcSweepCandidateResult_t *best)` [1090](sensorarrayFdcSweep.c#L1090) | 比较不完全合格的候选以选择可接受回退。 / Compare partially qualified candidates to choose the most usable fallback. | `sensorarrayFdcSweepRunCellSweep` |
| `static sensorarrayFdcSampleEval_t sensorarrayFdcSweepEvaluateSample(const sensorarrayFdcReadDiag_t *diag)` [1116](sensorarrayFdcSweep.c#L1116) | 从样本及 FDC 状态给出对该次候选评估的分类。 / Classify one sample and FDC status for candidate evaluation. | `sensorarrayFdcSweepReadOneSampleBounded` |
| `static esp_err_t sensorarrayFdcSweepReadOneSampleBounded(Fdc2214CapDevice_t *dev, const char *deviceName, Fdc2214CapChannel_t channel, uint32_t timeoutMs, sensorarrayFdcReadDiag_t *outDiag, sensorarrayFdcSampleEval_t *outEval)` [1175](sensorarrayFdcSweep.c#L1175) | 在本搜索步骤预算内获取一个 FDC 样本并返回质量证据。 / Acquire an FDC sample and its quality evidence within the current step budget. `ESP_ERR_INVALID_ARG` `ESP_ERR_TIMEOUT` | `sensorarrayFdcSweepCaptureCandidate` |
| `static void sensorarrayFdcSweepFinalizeRawStats(sensorarrayFdcSweepCandidateResult_t *candidate, uint64_t rawSum)` [1334](sensorarrayFdcSweep.c#L1334) | 汇总候选样本的原始码统计。 / Finalise candidate raw-code statistics from accumulated samples. | `sensorarrayFdcSweepCaptureCandidate` |
| `static void sensorarrayFdcSweepSortFrequencies(double *values, uint32_t count)` [1351](sensorarrayFdcSweep.c#L1351) | 原位排序有限的频率样本，用于中位数等统计。 / Sort the bounded frequency sample array in place for order statistics. | `sensorarrayFdcSweepFinalizeFrequencyStats` |
| `static void sensorarrayFdcSweepFinalizeFrequencyStats(sensorarrayFdcSweepCandidateResult_t *candidate, double *freqSamples, uint32_t freqCount)` [1367](sensorarrayFdcSweep.c#L1367) | 排序并汇总候选的频率观测值。 / Sort and summarise a candidate's observed frequencies. | `sensorarrayFdcSweepCaptureCandidate` |
| `static bool sensorarrayFdcSweepCandidateHasStableSamples(const sensorarrayFdcSweepCandidateResult_t *candidate)` [1414](sensorarrayFdcSweep.c#L1414) | 根据有效次数及分布检查候选采样是否稳定。 / Check candidate stability from good-sample count and distribution. | `sensorarrayFdcSweepCandidateIsWorking` / `sensorarrayFdcSweepCandidateScore` |
| `esp_err_t sensorarrayFdcSweepApplyDirectSafeLock(sensorarrayState_t *state, sensorarrayFdcDeviceId_t devId, Fdc2214CapChannel_t channel, uint8_t deglitchCode, uint16_t driveCurrent, bool highCurrent, const char *reason)` [1444](sensorarrayFdcSweep.c#L1444) | 按直接读取所需的频道、驱动及滤波设置配置器件。 / Apply the channel, drive and filter settings for the direct-read diagnostic. `ESP_ERR_INVALID_STATE` | `sensorarrayFdcSweepCaptureCandidate` / `sensorarrayFdcSweepRunCellSweep` |
| `static esp_err_t sensorarrayFdcSweepCaptureCandidate(sensorarrayState_t *state, sensorarrayFdcDeviceId_t devId, Fdc2214CapChannel_t channel, const sensorarrayFdcDeglitchCandidate_t *deglitch, uint16_t driveCurrent, bool highCurrent, uint8_t sIndex, uint8_t dIndex, const char *stage, uint32_t candidateIndex, uint32_t candidateCount, bool discardFirst, uint32_t sampleTarget, sensorarrayFdcSweepCandidateResult_t *outCandidate)` [1622](sensorarrayFdcSweep.c#L1622) | 采集某一驱动/带宽设定下的样本与告警作为比较证据。 / Capture samples and warnings for one drive/bandwidth setting as comparison evidence. `ESP_ERR_INVALID_ARG` `ESP_ERR_TIMEOUT` | `sensorarrayFdcSweepRunCellSweep` / `sensorarrayFdcSweepRunDirectReadForCell` |
| `esp_err_t sensorarrayFdcSweepApplyResult(sensorarrayState_t *state, const sensorarrayFdcSweepResult_t *result)` [1961](sensorarrayFdcSweep.c#L1961) | 把选中的调谐结果写回器件配置并保存应用状态。 / Apply the selected tuning result to device configuration and retain the outcome. `ESP_ERR_INVALID_ARG` `ESP_ERR_INVALID_STATE` | 独立接口 / Standalone entry |
| `esp_err_t sensorarrayFdcSweepRunChannel(sensorarrayState_t *state, sensorarrayFdcDeviceId_t devId, Fdc2214CapChannel_t channel, const char *reason, sensorarrayFdcSweepResult_t *outResult)` [1996](sensorarrayFdcSweep.c#L1996) | 旧单通道扫描入口。合法参数返回 ESP_ERR_NOT_SUPPORTED 并填入拒绝原因；生产调谐调用行扫描。 / Legacy single-channel entry. Valid arguments return ESP_ERR_NOT_SUPPORTED with the rejection reason. Production tuning uses row sweeps. `ESP_ERR_INVALID_ARG` `ESP_ERR_NOT_SUPPORTED` | `sensorarrayFdcSweepRunDevice` |
| `static uint32_t sensorarrayFdcSweepDeglitchBandwidthForProfile(const sensorarrayFdcSweepProfile_t *profile)` [2182](sensorarrayFdcSweep.c#L2182) | 选择与给定 profile 对应的去毛刺带宽。 / Select the deglitch bandwidth for the requested profile. | `sensorarrayFdcSweepRestoreAutoscan` |
| `esp_err_t sensorarrayFdcSweepRestoreAutoscan(sensorarrayState_t *state, sensorarrayFdcDeviceId_t devId, const char *reason)` [2190](sensorarrayFdcSweep.c#L2190) | 为指定 FDC 恢复自动扫描配置，使后续行读回到常规路径。 / Restore autoscan on the selected FDC before subsequent row acquisition. `ESP_ERR_INVALID_STATE` | `sensorarrayFdcRescueTick` / `sensorarrayFdcSweepRunDevice` |
| `esp_err_t sensorarrayFdcSweepRunDevice(sensorarrayState_t *state, sensorarrayFdcDeviceId_t devId, uint8_t channelMask, const char *reason)` [2253](sensorarrayFdcSweep.c#L2253) | 遍历指定通道调用旧单通道入口，统计被拒通道，恢复 autoscan 并保留首错。 / Call the legacy entry for each selected channel, count rejected channels, restore autoscan and retain the first error. `ESP_ERR_INVALID_ARG` `ESP_ERR_INVALID_STATE` | 独立接口 / Standalone entry |
| `static sensorarrayFdcCacheSource_t sensorarrayFdcSweepCacheSourceForMode(sensorarrayFdcCellSweepMode_t mode)` [2326](sensorarrayFdcSweep.c#L2326) | 将扫描模式映射为逐点缓存记录的来源。 / Translate the sweep mode to the provenance stored in the cell cache. | `sensorarrayFdcSweepRunCellSweep` / `sensorarrayFdcSweepRow` |
| `static const char *sensorarrayFdcSweepCacheSourceName(sensorarrayFdcCacheSource_t source)` [2340](sensorarrayFdcSweep.c#L2340) | 返回缓存来源的显示名称。 / Return the display name for cache source. | `sensorarrayFdcSweepUpdateStateCacheFromCandidate` |
| `static void sensorarrayFdcSweepIncrementFailCount(uint8_t *count)` [2357](sensorarrayFdcSweep.c#L2357) | 饱和递增 8 位失败计数，避免回绕使连续失败消失。 / Increment the eight-bit failure counter with saturation. | `sensorarrayFdcSweepRunCellSweep` / `sensorarrayFdcSweepRunDirectReadForCell` |
| `static void sensorarrayFdcSweepIncrementFailCount16(uint16_t *count)` [2364](sensorarrayFdcSweep.c#L2364) | 饱和递增 16 位失败计数。 / Increment the 16-bit failure counter with saturation. | `sensorarrayFdcSweepRunCellSweep` / `sensorarrayFdcSweepRunDirectReadForCell` |
| `static bool sensorarrayFdcSweepCooldownElapsed(uint32_t nowMs, uint32_t lastMs, uint32_t cooldownMs)` [2371](sensorarrayFdcSweep.c#L2371) | 基于当前、上次时间和冷却时长判断是否可重探。 / Test elapsed time against the configured cooldown before probing again. | `sensorarrayFdcSweepMeasureCell` |
| `static esp_err_t sensorarrayFdcSweepPrepareCellPath(sensorarrayState_t *state, const sensorarrayFdcCellCalibration_t *cal, const char *reason)` [2376](sensorarrayFdcSweep.c#L2376) | 为待测 S/D 单元建立 CAP 路由。 / Establish the CAP route for the selected S/D cell. `ESP_ERR_INVALID_ARG` | `sensorarrayFdcSweepRunCellSweep` / `sensorarrayFdcSweepRunDirectReadForCell` |
| `static void sensorarrayFdcSweepUpdateCalibrationFromCandidate(sensorarrayFdcCellCalibration_t *cal, const sensorarrayFdcSweepCandidateResult_t *candidate, bool bootSweep, bool lastGood)` [2391](sensorarrayFdcSweep.c#L2391) | 将候选的驱动、频率和电容证据写回该点启动/最近合格记录。 / Store candidate drive, frequency and capacitance evidence in the cell's boot/last-good calibration. | `sensorarrayFdcSweepRunCellSweep` / `sensorarrayFdcSweepRunDirectReadForCell` |
| `static void sensorarrayFdcSweepUpdateDeviceProfileFromCandidate(sensorarrayState_t *state, sensorarrayFdcDeviceId_t devId, Fdc2214CapChannel_t channel, const sensorarrayFdcSweepCandidateResult_t *candidate)` [2437](sensorarrayFdcSweep.c#L2437) | 将选中的通道参数转入对应 FDC 的运行 profile。 / Apply the selected channel settings to the corresponding FDC runtime profile. | `sensorarrayFdcSweepRunCellSweep` / `sensorarrayFdcSweepRow` |
| `static void sensorarrayFdcSweepUpdateStateCacheFromCandidate(sensorarrayState_t *state, const sensorarrayFdcCellCalibration_t *cal, const sensorarrayFdcSweepCandidateResult_t *candidate, sensorarrayFdcCacheSource_t source)` [2464](sensorarrayFdcSweep.c#L2464) | 将候选读数、设置及来源写入该 S/D 的状态缓存。 / Store candidate readings, settings and provenance in the S/D state cache. | `sensorarrayFdcSweepRunCellSweep` / `sensorarrayFdcSweepRunDirectReadForCell` |
| `static void sensorarrayFdcSweepLogCapResult(const sensorarrayFdcCellCalibration_t *cal, const sensorarrayFdcSweepCandidateResult_t *candidate)` [2545](sensorarrayFdcSweep.c#L2545) | 报告候选的频率、电容和最近调谐状态。 / Report the candidate frequency, capacitance and tuning context. | `sensorarrayFdcSweepRunDirectReadForCell` / `sensorarrayFdcSweepMeasureCell` |
| `static bool sensorarrayFdcSweepCandidateHugeJump(const sensorarrayFdcCellCalibration_t *cal, const sensorarrayFdcSweepCandidateResult_t *candidate)` [2568](sensorarrayFdcSweep.c#L2568) | 按已有校准和本轮频率判断是否出现需要复核的大跳变。 / Compare candidate frequency with calibration history to flag a jump requiring review. | `sensorarrayFdcSweepMeasureCell` |
| `static uint32_t sensorarrayFdcSweepBuildFastDriveOrderForCell(const sensorarrayFdcCellCalibration_t *cal, uint16_t *drives, uint32_t capacity, uint16_t *outCenter)` [2582](sensorarrayFdcSweep.c#L2582) | 以最近合格或启动电流为中心，生成有限的快速搜索顺序。 / Build a bounded fast search around the last good or boot drive current. | `sensorarrayFdcSweepRunCellSweep` / `sensorarrayFdcSweepRow` |
| `static uint32_t sensorarrayFdcSweepBuildFullDriveOrderForCell(const sensorarrayFdcCellCalibration_t *cal, uint16_t *drives, uint32_t capacity)` [2623](sensorarrayFdcSweep.c#L2623) | 为该点建立较广的去重电流候选序列。 / Build the wider deduplicated drive-current search for the cell. | `sensorarrayFdcSweepRunCellSweep` / `sensorarrayFdcSweepRow` |
| `static uint32_t sensorarrayFdcSweepBuildFastDeglitchOrderForCell(const sensorarrayFdcCellCalibration_t *cal, const sensorarrayFdcDeglitchCandidate_t **order, uint32_t capacity, uint8_t *outCenter)` [2641](sensorarrayFdcSweep.c#L2641) | 以该点历史带宽为中心生成快速滤波候选顺序。 / Build a fast filter-bandwidth search around the cell's tuning history. | `sensorarrayFdcSweepRunCellSweep` / `sensorarrayFdcSweepRow` |
| `static uint32_t sensorarrayFdcSweepBuildFullDeglitchOrderForCell(const sensorarrayFdcCellCalibration_t *cal, const sensorarrayFdcDeglitchCandidate_t **order, uint32_t capacity)` [2670](sensorarrayFdcSweep.c#L2670) | 生成该点完整的去毛刺带宽候选顺序。 / Build the full deglitch-bandwidth candidate order for the cell. | `sensorarrayFdcSweepRunCellSweep` / `sensorarrayFdcSweepRow` |
| `static const char *sensorarrayFdcSweepModeStage(sensorarrayFdcCellSweepMode_t mode)` [2683](sensorarrayFdcSweep.c#L2683) | 将调谐模式映射为日志和阶段跟踪用的字符串。 / Map tuning mode to a diagnostic stage string. | `sensorarrayFdcSweepRunCellSweep` / `sensorarrayFdcSweepRow` |
| `static esp_err_t sensorarrayFdcSweepRunCellSweep(sensorarrayState_t *state, sensorarrayFdcCellCalibration_t *cal, sensorarrayFdcCellSweepMode_t mode, const char *reason, sensorarrayFdcSweepCandidateResult_t *outBest, bool *outAccepted)` [2697](sensorarrayFdcSweep.c#L2697) | 在一个测点上执行指定范围的驱动与带宽扫描。 / Sweep the selected drive and bandwidth range for one cell. `ESP_ERR_INVALID_ARG` `ESP_ERR_INVALID_STATE` | `sensorarrayFdcSweepMeasureCell` |
| `static esp_err_t sensorarrayFdcSweepRunDirectReadForCell(sensorarrayState_t *state, sensorarrayFdcCellCalibration_t *cal, sensorarrayFdcSweepCandidateResult_t *outCandidate, bool *outAccepted)` [2967](sensorarrayFdcSweep.c#L2967) | 用该点最近合格的设置直接读取，再决定本次候选能否接受。 / Read directly using the cell's last-good settings and test candidate acceptance. `ESP_ERR_INVALID_ARG` `ESP_ERR_INVALID_STATE` | `sensorarrayFdcSweepMeasureCell` |
| `esp_err_t sensorarrayFdcSweepMeasureCell(sensorarrayState_t *state, uint8_t sIndex, uint8_t dIndex, uint32_t *outRaw28, bool *outValid)` [3055](sensorarrayFdcSweep.c#L3055) | 为指定 S/D 采集 28 位码，并将 outValid 与驱动返回分开报告。 / Acquire a 28-bit code for the selected S/D and return outValid separately from driver status. `ESP_ERR_INVALID_ARG` | 独立接口 / Standalone entry |
| `static uint8_t sensorarrayFdcSweepUnreadMaskFromStatus(const Fdc2214CapStatus_t *status)` [3168](sensorarrayFdcSweep.c#L3168) | 从 FDC STATUS 中提取 CH0..CH3 的未读位图。 / Extract the CH0..CH3 unread mask from FDC STATUS. | `sensorarrayFdcSweepWaitDeviceAutoscan` |
| `static const char *sensorarrayFdcSweepRrName(uint8_t rrSequence)` [3184](sensorarrayFdcSweep.c#L3184) | 返回循环扫描的显示名称。 / Return the display name for round-robin. | `sensorarrayFdcSweepApplyCandidateToDevice` |
| `static sensorarrayFdcDeviceState_t *sensorarrayFdcSweepDeviceState(sensorarrayState_t *state, sensorarrayFdcDeviceId_t devId)` [3199](sensorarrayFdcSweep.c#L3199) | 用器件 ID 定位主/次 FDC 的运行上下文。 / Resolve the primary/secondary runtime state from a device ID. | `sensorarrayFdcSweepApplyCandidateToDevice` |
| `esp_err_t sensorarrayFdcSweepRouteRowForCap(sensorarrayState_t *state, uint8_t sIndex, const char *reason)` [3205](sensorarrayFdcSweep.c#L3205) | 核对行号并为 CAP 调谐接通指定物理行。 / Validate the row and select its physical route for CAP tuning. `ESP_ERR_INVALID_ARG` | `sensorarrayFdcSweepRow` |
| `static esp_err_t sensorarrayFdcSweepApplyCandidateToDevice(sensorarrayState_t *state, sensorarrayFdcDeviceId_t devId, const sensorarrayFdcDeglitchCandidate_t *deglitch, const uint16_t driveCurrent[4], bool highCurrent, const char *reason)` [3229](sensorarrayFdcSweep.c#L3229) | 把四通道驱动电流、去毛刺和高电流标志写入一颗 FDC。 / Program four drive currents, deglitch and high-current selection on one FDC. `ESP_ERR_INVALID_ARG` `ESP_ERR_INVALID_RESPONSE` | `sensorarrayFdcSweepRow` |
| `static esp_err_t sensorarrayFdcSweepStartOrRestoreAutoscan(sensorarrayState_t *state, sensorarrayFdcDeviceId_t devId, const char *reason)` [3327](sensorarrayFdcSweep.c#L3327) | 通过恢复入口建立当前候选所需的自动扫描状态。 / Establish autoscan for the current candidate through its restore path. | `sensorarrayFdcSweepRow` |
| `static esp_err_t sensorarrayFdcSweepWaitDeviceAutoscan(sensorarrayFdcDeviceState_t *fdcState, sensorarrayFdcDeviceId_t devId, uint8_t sIndex, uint32_t timeoutMs, uint16_t *outStatus)` [3334](sensorarrayFdcSweep.c#L3334) | 在扫描调谐的超时预算内等待读取条件并返回最后 STATUS。 / Wait for candidate-sweep autoscan readiness up to the timeout and return the final STATUS. `ESP_ERR_INVALID_STATE` | `sensorarrayFdcSweepRow` |
| `static esp_err_t sensorarrayFdcSweepReadDeviceChannels(sensorarrayFdcDeviceState_t *fdcState, sensorarrayFdcDeviceId_t devId, uint8_t sIndex, sensorarrayFdcSweepDeviceRead4_t *outRead)` [3378](sensorarrayFdcSweep.c#L3378) | 读取该器件的四通道并保存用于比较候选的 raw/状态/频率。 / Read the device's four channels and retain raw/status/frequency evidence for candidate comparison. `ESP_ERR_INVALID_ARG` | `sensorarrayFdcSweepRow` |
| `static void sensorarrayFdcSweepBuildCandidateFromRead(const sensorarrayFdcSweepDeviceRead4_t *read, sensorarrayFdcDeviceId_t devId, uint8_t channel, const sensorarrayFdcDeglitchCandidate_t *deglitch, bool highCurrent, sensorarrayFdcSweepCandidateResult_t *outCandidate)` [3480](sensorarrayFdcSweep.c#L3480) | 从同一次四通道读数中为指定通道创建带配置和质量的候选。 / Build one channel's candidate settings and quality from a single four-channel observation. | `sensorarrayFdcSweepRow` |
| `static bool sensorarrayFdcSweepRowCandidateUsable(const sensorarrayFdcSweepCandidateResult_t *candidate)` [3555](sensorarrayFdcSweep.c#L3555) | 按候选的完整性、读数和告警决定其是否可用于该行。 / Check completeness, readings and alerts before using the row candidate. | `sensorarrayFdcSweepRowCandidateBetter` / `sensorarrayFdcSweepRow` |
| `static const char *sensorarrayFdcSweepRowRejectReason(const sensorarrayFdcSweepCandidateResult_t *candidate)` [3570](sensorarrayFdcSweep.c#L3570) | 从行候选的质量、就绪或读错统计选取拒绝原因。 / Select the rejection reason from row-candidate quality, readiness or read errors. | `sensorarrayFdcSweepRow` |
| `static bool sensorarrayFdcSweepRowCandidateBetter(const sensorarrayFdcSweepCandidateResult_t *candidate, const sensorarrayFdcSweepCandidateResult_t *best)` [3606](sensorarrayFdcSweep.c#L3606) | 按可用性和质量证据比较并选择行候选。 / Compare row candidates using admissibility and quality observations. | `sensorarrayFdcSweepRow` |
| `static void sensorarrayFdcSweepBuildRowDriveCandidate(uint8_t sIndex, uint16_t drive, sensorarrayFdcDeviceId_t devId, uint16_t outDrive[4])` [3632](sensorarrayFdcSweep.c#L3632) | 把所选驱动设置转换为本器件四个通道的候选数组。 / Build four channel drive-current candidates for the row/device. | `sensorarrayFdcSweepRow` |
| `static esp_err_t sensorarrayFdcSweepRow(sensorarrayState_t *state, uint8_t sIndex, sensorarrayFdcCellSweepMode_t mode, const char *reason, uint32_t *outOkCount, uint32_t *outFailCount)` [3650](sensorarrayFdcSweep.c#L3650) | 按行建立 CAP 路由、联合评估主次器件候选并返回成功/失败测点数。 / Route one row for CAP, evaluate candidates for both FDCs and return success/failure counts. `ESP_ERR_INVALID_ARG` `ESP_ERR_INVALID_STATE` | `sensorarrayFdcSweepRunFastRescueRow` / `sensorarrayFdcSweepRunFullRescueRow` |
| `esp_err_t sensorarrayFdcSweepRunFastRescueRow(sensorarrayState_t *state, uint8_t sIndex, const char *reason)` [4058](sensorarrayFdcSweep.c#L4058) | 对一行执行较窄的候选搜索作为快速救援。 / Run a narrower candidate search to rescue one row. | 独立接口 / Standalone entry |
| `esp_err_t sensorarrayFdcSweepRunFullRescueRow(sensorarrayState_t *state, uint8_t sIndex, const char *reason)` [4070](sensorarrayFdcSweep.c#L4070) | 在单行展开完整候选搜索并保存每通道结果。 / Run the full candidate search for a row and retain per-channel results. | 独立接口 / Standalone entry |
| `esp_err_t sensorarrayFdcSweepForceFullSweepCell(sensorarrayState_t *state, uint8_t sIndex, uint8_t dIndex)` [4082](sensorarrayFdcSweep.c#L4082) | 将指定单元作为触发源交给整行完整恢复入口。 / Use the specified cell as the trigger for full row rescue. | 独立接口 / Standalone entry |
| `esp_err_t sensorarrayFdcSweepRunCellSweepTarget(sensorarrayState_t *state, const sensorarrayFdcCellTarget_t *target, const char *reason, bool allowFullSweep)` [4089](sensorarrayFdcSweep.c#L4089) | 从触发测点定位实际要重扫的行，按 allowFullSweep 决定是否可升级。 / Resolve the trigger cell to a row sweep and allow escalation only when allowFullSweep is set. `ESP_ERR_INVALID_ARG` | `sensorarrayFdcSweepRunCellFastRescue` |
| `esp_err_t sensorarrayFdcSweepRunCellFastRescue(sensorarrayState_t *state, const sensorarrayFdcCellTarget_t *target, const char *reason)` [4148](sensorarrayFdcSweep.c#L4148) | 以目标测点触发该行快速救援并关闭完整扫描升级。 / Trigger fast rescue for the target cell's row with full-sweep escalation disabled. | 独立接口 / Standalone entry |
| `esp_err_t sensorarrayFdcSweepRunFullRescueCell(sensorarrayState_t *state, uint8_t sIndex, uint8_t dIndex, const char *reason)` [4155](sensorarrayFdcSweep.c#L4155) | 核对 S/D 后以该点触发所在行的完整恢复并报告结果。 / Validate S/D coordinates and run full rescue on the row containing the trigger cell. `ESP_ERR_INVALID_ARG` | `sensorarrayFdcSweepForceFullSweepCell` |
| `esp_err_t sensorarrayFdcSweepRunFullRescueAll(sensorarrayState_t *state, const char *reason)` [4198](sensorarrayFdcSweep.c#L4198) | 遍历所有需要救援的行或测点并汇总受限搜索的结果。 / Visit the rows/cells requiring full rescue and aggregate bounded search results. `ESP_ERR_INVALID_ARG` `ESP_ERR_INVALID_STATE` | `sensorarrayFdcMatrixEngineRunFullRescue` |
| `static uint8_t sensorarrayFdcSweepBootRowCacheMask(const sensorarrayState_t *state, uint8_t row)` [4260](sensorarrayFdcSweep.c#L4260) | 检查某行八个位置的启动校准缓存并返回已填位图。 / Return the eight-bit mask of populated boot-calibration entries for a row. | `sensorarrayFdcSweepBootCacheFilledCount` / `sensorarrayFdcSweepRunBoot` |
| `static uint8_t sensorarrayFdcSweepBootCacheFilledCount(const sensorarrayState_t *state)` [4275](sensorarrayFdcSweep.c#L4275) | 计数整矩阵中实际填入的启动校准测点。 / Count populated boot-calibration cells across the matrix. | `sensorarrayFdcSweepRunBoot` |
| `esp_err_t sensorarrayFdcSweepRunBoot(sensorarrayState_t *state, sensorarrayFdcBootSummary_t *outSummary)` [4293](sensorarrayFdcSweep.c#L4293) | 对启动计划执行实际调谐，保存需要的每行质量、通道参数和摘要。 / Tune the boot plan and retain row quality, channel parameters and the boot summary. `ESP_ERR_INVALID_ARG` | `sensorarrayFdcMatrixEngineRunBootSweep` |

### sensorarrayFdcTiming.c

| C 调用签名 / C signature | 行为及返回 / Behaviour and return | 调用位置 / Called by |
| --- | --- | --- |
| `uint64_t sensorarrayMeasureElapsedUs(int64_t startUs)` [3](sensorarrayFdcTiming.c#L3) | 计算两个时间戳之间的非负微秒差值。 / Calculate the nonnegative elapsed microseconds between timestamps. | `sensorarrayMeasureApplyFdcCachedRowConfig` / `sensorarrayMeasureComputeFdcFrameCapTotalPf` |
| `uint64_t sensorarrayMeasureEstimateI2cBits(uint32_t writeCount, uint32_t readCount, uint32_t writeBytes, uint32_t readBytes)` [9](sensorarrayFdcTiming.c#L9) | 按读写次数和字节数估算总线使用的位数。 / Estimate I²C bus bits from transfer counts and byte totals. | `sensorarrayMeasureMergeFdcI2cStats` / `sensorarrayMeasurePrintFdcTimingAggregate` |
| `void sensorarrayMeasureMergeFdcI2cStats(const Fdc2214CapI2cStats_t *primaryStats, const Fdc2214CapI2cStats_t *secondaryStats, const BoardSupportI2cBusInfo_t *primaryBus, const BoardSupportI2cBusInfo_t *secondaryBus, sensorarrayFdcTimingSummary_t *summary)` [25](sensorarrayFdcTiming.c#L25) | 合并主次 FDC 驱动及板级 I²C 计数，用于区分器件与总线耗时。 / Combine FDC driver and board I²C counters for both buses for timing attribution. | `sensorarrayMeasureReadFdcMatrixFrameSnapshot` |
| `void sensorarrayMeasurePrintFdcTimingSummary(const sensorarrayFdcTimingSummary_t *summary, uint32_t sequence)` [113](sensorarrayFdcTiming.c#L113) | 报告一个帧序号对应的 FDC 采集分阶段汇总。 / Report FDC phase timings for the specified frame sequence. | `sensorarrayMeasureReadFdcMatrixFrameSnapshot` |
| `uint8_t sensorarrayMeasureFdcProfileSlowestChannel(const sensorarrayFdcProfileSnapshot_t *snapshot)` [223](sensorarrayFdcTiming.c#L223) | 找出运行档中限制该器件扫描周期的通道。 / Identify the channel limiting this device's scan period in the selected profile. | `sensorarrayMeasurePrintFdcProfileSummary` |
| `void sensorarrayMeasureFdcPrintProfileSnapshotDetail(uint32_t sequence, uint8_t row, sensorarrayFdcDeviceId_t devId, const sensorarrayFdcProfileSnapshot_t *snapshot, const char *reason)` [239](sensorarrayFdcTiming.c#L239) | 按行和器件打印 profile 参数快照及记录原因。 / Report profile settings and reason for one row and device. | `sensorarrayMeasurePrintFdcProfileSummary` |
| `void sensorarrayMeasurePrintFdcProfileSummary(uint32_t sequence)` [291](sensorarrayFdcTiming.c#L291) | 输出当前 profile 中最慢通道与其帧预算。 / Report the limiting channel of the active profile and its timing budget. | `sensorarrayMeasureReadFdcMatrixFrameSnapshot` |
| `uint64_t sensorarrayMeasureAvgU64(uint64_t total, uint32_t count)` [381](sensorarrayFdcTiming.c#L381) | 在有样本时计算 64 位累计量的平均值。 / Average a 64-bit accumulator when its sample count is nonzero. | `sensorarrayMeasurePrintFdcTimingSummary` / `sensorarrayMeasurePrintFdcTimingAggregate` |
| `void sensorarrayMeasurePrintFdcTimingAggregate(const sensorarrayFdcTimingAggregate_t *agg)` [386](sensorarrayFdcTiming.c#L386) | 从已累计窗口计算平均与最慢阶段并输出。 / Summarise averages and limiting phases for the accumulated timing window. | `sensorarrayMeasureUpdateFdcTimingAggregate` |
| `void sensorarrayMeasureUpdateFdcTimingAggregate(const sensorarrayFdcTimingSummary_t *summary, uint32_t sequence)` [1048](sensorarrayFdcTiming.c#L1048) | 将当前帧摘要合入窗口，到期时发布窗口统计。 / Merge the current frame summary into the timing window and report when due. | `sensorarrayMeasureReadFdcMatrixFrameSnapshot` |
| `void sensorarrayMeasurePrintFdcBottleneck(const sensorarrayFdcTimingSummary_t *summary, uint32_t sequence)` [1293](sensorarrayFdcTiming.c#L1293) | 按本帧最大耗时阶段报告可能限制采集的环节。 / Report the phase consuming the largest part of this frame's timing budget. | `sensorarrayMeasureReadFdcMatrixFrameSnapshot` |
| `void sensorarrayMeasurePrintFdcRowTiming(uint32_t sequence, const sensorarrayFdcRowTiming_t *rowTiming)` [1364](sensorarrayFdcTiming.c#L1364) | 输出该帧一行的切换、等待和工作重叠时间。 / Report switch, wait and worker-overlap timing for one row in this frame. | `sensorarrayMeasureReadFdcMatrixFrameSnapshot` |
| `void sensorarrayMeasurePrintFdcDeviceTiming(uint32_t sequence, const sensorarrayFdcDeviceTiming_t *deviceTiming)` [1419](sensorarrayFdcTiming.c#L1419) | 按主/次器件输出本帧的阶段耗时和就绪诊断。 / Report phase timing and readiness diagnostics for the selected FDC. | `sensorarrayMeasureReadFdcMatrixFrameSnapshot` |

### sensorarrayFdcWorkerLifecycle.h

| C 调用签名 / C signature | 行为及返回 / Behaviour and return | 调用位置 / Called by |
| --- | --- | --- |
| `static inline sensorarrayFdcJobPhase_t sensorarrayFdcJobPhase(const sensorarrayFdcJobLifecycle_t *life)` [24](sensorarrayFdcWorkerLifecycle.h#L24) | 以 acquire 顺序读取槽当前阶段。 / Read the slot's current phase with acquire ordering. | `sensorarrayMeasureFdcWorkersQuarantined` / `sensorarrayFdcWorkersWait` |
| `static inline bool sensorarrayFdcJobChange(sensorarrayFdcJobLifecycle_t *life, sensorarrayFdcJobPhase_t from, sensorarrayFdcJobPhase_t to)` [30](sensorarrayFdcWorkerLifecycle.h#L30) | 只在预期阶段仍相符时以原子操作切到目标阶段。 / Atomically transition to the target phase only if the expected phase still holds. | `sensorarrayMeasureFdcWorkerTask` / `sensorarrayMeasureReadFdcMatrixRowParallelEpoch` |
| `static inline bool sensorarrayFdcJobReleased(const sensorarrayFdcJobLifecycle_t *life)` [38](sensorarrayFdcWorkerLifecycle.h#L38) | 检查 worker 已对最后一次槽访问发布 release 证据。 / Check for the worker's release publication following its last slot access. | `sensorarrayMeasureFdcWorkersQuiescent` / `sensorarrayFdcWorkersWait` |
| `static inline bool sensorarrayFdcJobPost(sensorarrayFdcJobLifecycle_t *life)` [44](sensorarrayFdcWorkerLifecycle.h#L44) | 为本轮作业建立 POSTED 阶段并撤销旧释放证据。 / Prepare the new job's POSTED state and clear the previous release evidence. | `sensorarrayMeasureReadFdcMatrixRowParallelEpoch` |
| `static inline void sensorarrayFdcJobCancel(sensorarrayFdcJobLifecycle_t *life)` [55](sensorarrayFdcWorkerLifecycle.h#L55) | 根据当前作业阶段原子登记取消。 / Request cancellation atomically for the current job phase. | `sensorarrayFdcCancelAndReclaim` / `sensorarrayMeasureFdcWorkerTask` |
| `static inline void sensorarrayFdcJobFinish(sensorarrayFdcJobLifecycle_t *life)` [67](sensorarrayFdcWorkerLifecycle.h#L67) | 发布 DONE 或 CANCEL_ACKED 后标记 worker 已结束所有槽访问。 / Publish DONE or CANCEL_ACKED, then release the worker's final reference to the slot. | `sensorarrayMeasureFdcWorkerTask` |
| `static inline bool sensorarrayFdcJobReclaim(sensorarrayFdcJobLifecycle_t *life)` [82](sensorarrayFdcWorkerLifecycle.h#L82) | 只有明确释放且 DONE 或 CANCEL_ACKED 时把槽置为 RECLAIMED。 / Move a released DONE or CANCEL_ACKED slot to RECLAIMED. | `sensorarrayFdcCancelAndReclaim` / `sensorarrayMeasureReadFdcMatrixRowParallelEpoch` |
| `static inline void sensorarrayFdcJobQuarantine(sensorarrayFdcJobLifecycle_t *life)` [92](sensorarrayFdcWorkerLifecycle.h#L92) | 发布 QUARANTINED，阻止协调器复用仍可能被引用的槽。 / Publish QUARANTINED to exclude a potentially referenced slot from reuse. | `sensorarrayFdcCancelAndReclaim` |

## 参数索引 / Parameter index

相同名称按模块共享解释。完整类型、数组维度与输入输出方向以对应函数签名和调用点为准。指针参数所需存储由调用者提供，异步作业另受上面的所有权约束。

Entries share parameter descriptions within this module. Refer to the signature and its caller for the precise type, array shape and direction. The caller supplies pointer storage, subject to the ownership rules above for asynchronous work.

| 参数 / Parameter | 含义 / Meaning |
| --- | --- |
| `a`, `b` | 数值比较、取中位值或身份比较的输入操作数，按相应签名确定类型。 / Operand for a numeric comparison, median or identity comparison as declared. |
| `action`, `kind`, `stage` | 对应操作、诊断或状态机的枚举/标签，采用签名中所列类型的常量。 / Operation, diagnostic or state-machine discriminator from its declared enum or label domain. |
| `activeRows` | 生效或申请的活动行数，范围为 1..8。 / Applied or requested number of active rows, 1..8. |
| `actualDataReadSkippedDespiteStatusReadyCount` | 实测数据读取跳过尽管状态就绪数量。 / Actual data read skipped despite status ready count. |
| `afterIntb` | 之后INTB。 / After INTB. |
| `afterIntbRecheck` | 之后INTB复查。 / After INTB recheck. |
| `agg` | 汇总。 / Agg. |
| `allowDirectData` | 允许直接数据。 / Allow direct data. |
| `allowFullSweep` | 允许完整扫描。 / Allow full sweep. |
| `arg` | 回调或任务创建时绑定的上下文；在该调用的有效期内保留其存储。 / Context bound at callback registration or task creation. Keep its backing object alive for the operation. |
| `base` | 基准。 / Base. |
| `best` | 最佳。 / Best. |
| `bestPartial` | 最佳部分。 / Best partial. |
| `bestPartialSeen` | 最佳部分已观测。 / Best partial seen. |
| `bootSweep`, `cal`, `lastGood` | 启动调谐、最近有效设置或正在操作的校准记录。 / Boot tuning, last-good settings or the calibration record being used. |
| `candidate` | 候选。 / Candidate. |
| `candidateCount` | 候选数量。 / Candidate count. |
| `candidateIndex` | 候选索引。 / Candidate index. |
| `capacity` | 按签名和对象度量的可用容量；数组以元素计，文本缓冲区以字节计。 / Available capacity in typed elements for arrays or bytes for text buffers. |
| `center` | 中心。 / Center. |
| `ch`, `channel` | 芯片通道编号或传输通道枚举。FDC 硬件通道为 0..3。 / Chip channel number or transport-channel enum. FDC hardware channels are 0..3. |
| `channelMask`, `errorMask`, `freshMask4`, `reqMask`, `requiredUnreadMask`, `validMask` | 通道/测点位图，按接口区分所需、有效、出错、新鲜或未读位置。 / Channel/cell bitmask identifying required, valid, failed, fresh or unread positions for the operation. |
| `clockDividers`, `rCount`, `settleCount` | FDC 转换计数、建立计数或分频配置的寄存器值。 / FDC conversion-count, settling-count or divider register value. |
| `config`, `policy` | 该入口声明的配置或策略结构，含所选阈值、时序或回调。 / Configuration or policy of the declared type holding the selected thresholds, timing or callbacks. |
| `configOk` | 配置通过。 / Config ok. |
| `configs` | 配置。 / Configs. |
| `cooldownMs` | 冷却，单位 ms。 / Cooldown in ms. |
| `count` | 数量。 / Count. |
| `ctx` | 所声明类型的应用、总线或调度上下文，供调用链共享。 / Application, bus or scheduler context of the declared type, shared with the call chain. |
| `deadlineUs`, `reclaimDeadlineUs`, `rowDeviceDeadlineOverrideUs`, `rowDeviceDeadlineUs` | 单调时钟绝对截止时间，单位 µs。 / Absolute deadline on the monotonic microsecond clock. |
| `decisionReason` | 决定原因。 / Decision reason. |
| `deglitch`, `deglitchCode` | FDC 输入去毛刺带宽的枚举、寄存器码或预计读回。 / FDC input deglitch bandwidth enum, register code or expected readback. |
| `delayMs` | 延迟，单位 ms。 / Delay in ms. |
| `denominator` | 分母。 / Denominator. |
| `dev`, `devId`, `fdc`, `fdcState` | 对应驱动句柄、FDC 主/次设备 ID 或 ADS/FDC 状态；按签名类型选取当前板上同一实例。 / Corresponding driver handle, primary/secondary FDC ID or ADS/FDC state. Pass the board instance matching the declared type. |
| `deviceName` | 设备名称。 / Device name. |
| `deviceTiming`, `primaryTiming`, `rowTiming`, `secondaryTiming`, `timing` | 本轮采集的各阶段起止与累计用时记录，主、次 FDC 分别保留。 / Per-stage timestamps and accumulated time for this acquisition, retained separately for the primary/secondary FDC. |
| `diag`, `outSummary`, `summary` | 本次启动、采样或调度的汇总/诊断记录，类型见对应函数。 / Summary/diagnostic evidence for this startup, sample or scheduling operation, typed by the function. |
| `diagReadyButRejectedCount` | 诊断就绪但被拒绝数量。 / Diagnostics ready but rejected count. |
| `diffWriteCount` | 差异写入数量。 / Diff write count. |
| `dIndex`, `sIndex` | FDC 物理 D/S 位置在该调用点的表示，参见真实调用者；传入 matrixIndex 的接口另用线性下标。 / D/S position used by this FDC call. Consult its actual caller for representation. matrixIndex entries use a separate linear index. |
| `discardCount` | 丢弃数量。 / Discard count. |
| `discardFirst` | 丢弃首个。 / Discard first. |
| `doneA` | 完成甲。 / Done a. |
| `doneB` | 完成乙。 / Done b. |
| `drive`, `driveCurrent`, `drives`, `highCurrent` | FDC 激励电流设置或候选表，使用驱动定义的编码。 / FDC drive-current setting, high-current flag or candidate list using driver encoding. |
| `effectiveFclkHz`, `refClockHz` | 原始码换算频率所需的参考时钟与通道 CLOCK_DIVIDERS 字。 / Reference clock and channel CLOCK_DIVIDERS register used to convert raw code to frequency. |
| `enable` | 使能。 / Enable. |
| `enabled`, `valid` | 表示证据已知、数据有效、功能启用或操作成功的布尔标志，按参数名独立解释。 / Boolean representing known evidence, validity, enablement or success as named. |
| `engine` | 已初始化且与本次采集绑定的引擎实例。 / Initialised engine instance bound to this acquisition. |
| `epochId` | 本行 FDC worker 的工作轮次，只有同一槽位及 epoch 的结果可消费。 / FDC worker's row epoch. Only results matching the slot and epoch are eligible. |
| `error` | 操作、首个根因或独立清理结果的错误码。保留调用层提供的具体错误。 / Error for the operation, first cause or separate cleanup, preserving the code from the reporting layer. |
| `estimatedRoundUs`, `roundUs`, `targetRoundUs` | 当前、预期或目标 FDC 扫描一轮的微秒耗时。 / Actual, estimated or target duration of an FDC scan round in µs. |
| `expected` | 核验时所需的配置/身份与实际寄存器观测值，使用该接口指定的类型和掩码。 / Desired configuration/identity and actual evidence compared with the type and masks for this interface. |
| `expectedDev` | 预期器件。 / Expected device. |
| `expectedEpoch` | 预期轮次。 / Expected epoch. |
| `expectedRow` | 预期行。 / Expected row. |
| `fallback` | 回退。 / Fallback. |
| `fallbackAttempted` | 回退已尝试。 / Fallback attempted. |
| `forceWrite` | 强制写入。 / Force write. |
| `frame` | 当前采集帧。const 指针供读取/编码，可写指针供对应采集步骤产生或更新数据。 / Capture frame. A const pointer is an input to readers/encoders. A writable pointer receives or updates acquisition results. |
| `frameSeq`, `sequence` | 对应采集帧的序号，必须与提供的帧、事件或缓存上下文属于同次启动。 / Capture sequence belonging to the same boot as the supplied frame, event or cache context. |
| `freqCount` | 频率数量。 / Freq count. |
| `freqHz` | 频率，单位 Hz。 / Freq in Hz. |
| `freqSamples` | 频率样本。 / Freq samples. |
| `frequencyHz` | 总线或器件时钟，Hz。请求值与驱动实际速率分别记录。 / Bus or device clock in Hz. Requested and actual driver clock are tracked separately. |
| `fresh` | 采集新鲜标志、一致性或测点位图。掩码按物理测点下标定位。 / Freshness flag, coherence flag or per-cell bitset. A cell mask is indexed by physical position. |
| `from`, `life`, `to` | 持久 worker 槽及预期/目标阶段，用于原子状态转换。 / Persistent worker slot and expected/target phases used by the atomic transition. |
| `hardTimeout`, `timeoutMs`, `timeoutUs` | 有限等待预算或硬超时标志，时间单位见参数后缀。 / Bounded waiting budget or hard-timeout flag. Time units follow the suffix. |
| `health` | 健康。 / Health. |
| `i2cError` | 索引2丙错误。 / I 2 c error. |
| `i2cErrorCount`, `notReadyCount`, `waitBudgetTooShortCount`, `zeroRawCount` | 该轮候选评估中的 I²C 错误、等待预算不足、零码或尚未就绪数量。 / Per-round I²C, inadequate wait budget, zero-code or not-ready counts for candidate evaluation. |
| `idOk` | 标识通过。 / ID ok. |
| `inductorUh`, `inductorValueUh` | 计算 LC 电容所用的电感值，单位 µH。 / Tank inductance for LC capacitance conversion in µH. |
| `intbMissButStatusReadyCount` | INTB未命中但状态就绪数量。 / INTB miss but status ready count. |
| `lastMs`, `nowMs` | 调用者提供的当前或上次时间，单位 ms。 / Current or previous caller-supplied timestamp in ms. |
| `levelLowButEdgeMissCount` | 电平低但边沿未命中数量。 / Level low but edge miss count. |
| `logNormal` | 日志常规。 / Log normal. |
| `logNormalSuccess` | 日志常规成功。 / Log normal success. |
| `maxValue`, `minValue` | 判定或限幅所用阈值，以参数名指定的数值域和单位为准。 / Threshold for admission or clamping in the numeric domain and units named by the parameter. |
| `mode` | 对应调用域的模式枚举，保留目标、当前帧或恢复上下文的含义。 / Mode enum in the declared domain, selecting the target, frame or recovery mode as indicated. |
| `newDriveCurrent0` | 新驱动当前0。 / New drive current 0. |
| `newRcount`, `newSettle`, `oldRcount`, `oldSettle` | 原档位与候选档位的 FDC RCOUNT 或 SETTLECOUNT。 / Original and proposed FDC RCOUNT or SETTLECOUNT for a profile. |
| `newRoundUs` | 新轮次，单位 µs。 / New round in µs. |
| `notReady` | 未就绪。 / Not ready. |
| `numerator` | 分子。 / Numerator. |
| `offset` | 偏移。 / Offset. |
| `oldDriveCurrent0` | 原驱动当前0。 / Old drive current 0. |
| `oldRoundUs` | 原轮次，单位 µs。 / Old round in µs. |
| `order` | 顺序。 / Order. |
| `outAccepted` | 接收受理的。 / Receives accepted. |
| `outBest` | 接收最佳。 / Receives best. |
| `outCandidate` | 接收候选。 / Receives candidate. |
| `outCapPf` | 接收CAP，单位 pF。 / Receives Cap in pF. |
| `outCenter` | 接收中心。 / Receives center. |
| `outConfigs` | 接收配置。 / Receives configs. |
| `outDiag` | 接收结果。本次启动、采样或调度的汇总/诊断记录，类型见对应函数。 / Output. Summary/diagnostic evidence for this startup, sample or scheduling operation, typed by the function. |
| `outDrive` | 接收结果。FDC 激励电流设置或候选表，使用驱动定义的编码。 / Output. FDC drive-current setting, high-current flag or candidate list using driver encoding. |
| `outErrorMask8` | 接收错误掩码8。 / Receives error mask 8. |
| `outEstimatedRoundUs` | 接收结果。当前、预期或目标 FDC 扫描一轮的微秒耗时。 / Output. Actual, estimated or target duration of an FDC scan round in µs. |
| `outEval` | 接收评估。 / Receives eval. |
| `outFailCount`, `outOkCount` | 接收本轮探测、采集或候选比较中的成功/失败数量。 / Receive counts of successful or failed probes, captures or candidates for this round. |
| `outFinFactor` | 接收输入频率因子。 / Receives fIN factor. |
| `outFinSelCode` | 接收输入频率选择代码。 / Receives fIN sel code. |
| `outFrefDivider` | 接收参考频率分压。 / Receives fREF divider. |
| `outRaw28`, `raw28` | FDC 28 位原始转换值。 / FDC 28-bit raw conversion result. |
| `outRead` | 接收读取。 / Receives read. |
| `outRead4` | 接收读取4。 / Receives read 4. |
| `outReady` | 接收就绪。 / Receives ready. |
| `outResult`, `result`, `snapshot` | 函数签名注明的结果、快照或统计结构，调用者分配复制目标。 / Result, snapshot or statistics object of the declared type, with caller-owned output storage. |
| `outSample` | 接收结果。要归约或校验的一组测量，数组长度由相邻 count 参数或签名给出。 / Output. Measurements to reduce or validate. The adjacent count or declared array bound gives their extent. |
| `outSamples`, `primarySamples`, `secondarySamples` | 四个 FDC 通道的读数/有效标志。主次样本各对应本行的四个 D 位置。 / Four FDC-channel samples or validity bits. Primary and secondary sample groups each cover four D positions in a row. |
| `outStatus` | 接收结果。声明的诊断/状态记录，或从器件读回的原始状态字。 / Output. Diagnostic/status record of the declared type or a raw status word returned by the device. |
| `outValid` | 接收结果。表示证据已知、数据有效、功能启用或操作成功的布尔标志，按参数名独立解释。 / Output. Boolean representing known evidence, validity, enablement or success as named. |
| `outValidMask8` | 接收有效掩码8。 / Receives valid mask 8. |
| `outWarnMask8` | 接收警告掩码8。 / Receives warn mask 8. |
| `outWorkerTimeout` | 接收工作任务超时。 / Receives worker timeout. |
| `plan` | 活动行的采样操作计划或逐行配置，所有组使用同一帧的配置快照。 / Acquisition plan or row selections for the active frame. All groups use the same capture snapshot. |
| `predictedFreqHz` | 预测频率，单位 Hz。 / Predicted freq in Hz. |
| `preWaitCheck` | 预先等待检查。 / Pre wait check. |
| `primary`, `secondary` | 主/次总线、器件、采样结果或选择位，其类型由完整签名指定。 / Primary/secondary bus, device, sample or selector as indicated by its type. |
| `primaryBus` | 主级总线。 / Primary bus. |
| `primaryErr` | 主级错误码。 / Primary err. |
| `primaryStats` | 主级统计。 / Primary stats. |
| `profile` | 配置档或八个物理行的模式列表。outModes 接收解析结果。 / Configuration profile or mode selection for the eight physical rows. outModes receives parsed values. |
| `profileTooSlow` | 配置档过慢。 / Profile too slow. |
| `quality` | 质量。 / Quality. |
| `rawSum` | 原始码求和。 / Raw sum. |
| `read` | 读取。 / Read. |
| `read4` | 读取4。 / Read 4. |
| `readBytes` | 读取，单位 字节。 / Read in bytes. |
| `readCount` | 读取数量。 / Read count. |
| `readErr` | 读取错误码。 / Read err. |
| `readFn` | 使用头文件定义签名的回调。提供的上下文与回调在注册或同步调用期间有效。 / Callback with the signature declared in the header. Keep its context valid while registered or used. |
| `readReason` | 读取原因。 / Read reason. |
| `readTracker` | 读取跟踪器。 / Read tracker. |
| `readUs` | 读取，单位 µs。 / Read in µs. |
| `ready` | 就绪。 / Ready. |
| `readyAfter` | 就绪之后。 / Ready after. |
| `readyBefore` | 就绪开始前。 / Ready before. |
| `reason`, `why` | 操作或拒绝原因字符串；outReason 用于返回原因。 / Operation/rejection reason string. outReason receives the reason. |
| `relaxedMode` | 放宽判定模式。 / Relaxed mode. |
| `released` | 已释放。 / Released. |
| `requested` | 请求的。 / Requested. |
| `row` | 行或列的坐标。通用 matrixEngineRegion_t 的原点为零；以板级 S/D 位置操作的接口从 1 编号。 / Row or column coordinate. Generic matrixEngineRegion_t origins are zero-based. Board S/D coordinates are one-based. |
| `rowBudgetUs` | 行预算，单位 µs。 / Row budget in µs. |
| `rrSequence` | 循环扫描序号。 / Round-robin sequence. |
| `runtimeConfigs` | 运行期配置。 / Runtime configs. |
| `samples`, `values` | 要归约或校验的一组测量，数组长度由相邻 count 参数或签名给出。 / Measurements to reduce or validate. The adjacent count or declared array bound gives their extent. |
| `sampleStatus` | 样本状态。 / Sample status. |
| `sampleTarget` | 候选所需样本量、允许尝试数或当前尝试索引。 / Candidate sample target, allowed number of attempts or current attempt index. |
| `secondaryBus` | 次级总线。 / Secondary bus. |
| `secondaryErr` | 次级错误码。 / Secondary err. |
| `secondaryStats` | 次级统计。 / Secondary stats. |
| `serialUs` | 串行，单位 µs。 / Serial in µs. |
| `source` | 路由或测量的来源标记，区分外部实测、内部推算或对应开关源的枚举。 / Route/measurement source discriminator, such as external, internal estimate or switch source for the declared enum. |
| `spanUs` | 跨度，单位 µs。 / Span in µs. |
| `startA` | 起始甲。 / Start a. |
| `startB` | 起始乙。 / Start b. |
| `startUs` | 单调时钟的事件时间，单位 µs。 / Event time on the monotonic clock in µs. |
| `state` | 所声明类型的状态对象或枚举。传入前端状态指针时，其生命周期覆盖总线或采集操作。 / State object or enum of the declared type. A frontend-state pointer must outlive the bus or capture operation. |
| `status` | 声明的诊断/状态记录，或从器件读回的原始状态字。 / Diagnostic/status record of the declared type or a raw status word returned by the device. |
| `statusFallbackAcceptedCount` | 状态回退受理的数量。 / Status fallback accepted count. |
| `target` | 此操作的目标，可能为目标帧、路由/测点结构或 sink，以签名类型为准。 / Target frame, route/cell or sink, according to the declared type. |
| `timeoutStage` | 超时阶段。 / Timeout stage. |
| `total` | 总计。 / Total. |
| `trace` | 跟踪。 / Trace. |
| `tracker` | 跟踪器。 / Tracker. |
| `value` | 值。 / Value. |
| `waitUs` | 等待，单位 µs。 / Wait in µs. |
| `warning` | 警告。 / Warning. |
| `writeBytes` | 写入，单位 字节。 / Write in bytes. |
| `writeCount` | 写入数量。 / Write count. |
| `zeroAfterDrdy` | 零点之后DRDY。 / Zero after DRDY. |
| `zeroAfterDrdyCount` | 零点之后DRDY数量。 / Zero after DRDY count. |
| `zeroBeforeReady` | 零点开始前就绪。 / Zero before ready. |
| `zeroBeforeReadyCount` | 零点开始前就绪数量。 / Zero before ready count. |

[项目总览 / Project guide](../../../README.md)

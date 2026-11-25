# 测量、路由与配置身份 / Measurement, routing and frame identity

采集任务在帧边界应用行数、全局模式和逐行模式，并保存三种配置域各自的身份。计划选择 CAP 的双 FDC、VOLT/RES 的 ADC1 或逐行混合引擎。route controller 按板级连线安排停止转换、切换参考与 GPIO、核对寄存器及等待建立。帧初始化设置无效哨兵与掩码，完成标志由预期、已采、有效与新鲜掩码分别给出。

At frame boundaries the capture task applies row count, global mode and row profile and retains each domain's identity. A plan selects the dual-FDC CAP engine, ADC1 for VOLT/RES, or the mixed-row engine. The route controller orders conversion stop, reference and GPIO changes, register verification and settling according to the board map. Frame initialisation establishes invalid sentinels and masks. Expected, acquired, valid and fresh masks describe distinct properties.

[ADS 数据和辅助事务 / ADS data and auxiliary transactions](ads/README.md) · [双 FDC / Dual FDC](fdc/README.md) · [逐行混合 / Mixed rows](mixed/README.md)

## 调用方式 / Calling the module

```c
sensorarrayConfigAuthority_t snapshot;
if (sensorarrayConfigIdentityCopy(&snapshot)) {
    uint32_t applied = snapshot.identity.appliedFrameBoundary;
    (void)applied;
}
```

控制读者可复制一次已发布的权威状态；只有单一采集所有者在应用后发布。接收者用 session/boot、revision 和各域 ID 解读 CONFIG?，MACK 表示接受，MAPP/RAPP/RMAPP 报告各自终态。安全路由的 requested、applied、known、verified 分别记录，模拟隔离需要板级能力。每行 8 个测点，公开 S/D 坐标从 1 编号，索引映射用 sensorarrayMatrixIndex。

Control readers can copy one published authority value. Only the acquisition owner publishes after application. Consumers interpret CONFIG? using session/boot, revision and domain IDs. MACK reports acceptance and MAPP/RAPP/RMAPP report their respective terminal applications. Requested, applied, known and verified are tracked separately for safe routing. Analogue isolation requires board capability. Each row has eight cells. Public S/D coordinates are one-based and sensorarrayMatrixIndex maps them to storage.

## 源码和头文件 / Source and headers

| 文件 / File | 职责与声明 / Responsibility and declarations |
| --- | --- |
| [sensorarrayConfigIdentity.c](sensorarrayConfigIdentity.c) · [include/sensorarrayConfigIdentity.h](include/sensorarrayConfigIdentity.h) | 在单一采集写者下建立并发布会话、模式、行数和 profile 的原子权威快照供 CONFIG? 查询。 / Captures and publishes one atomic session/mode/rows/profile authority from a single acquisition writer for CONFIG? queries. `sensorarrayConfigIdentity_t` `sensorarrayConfigAuthority_t` |
| [include/sensorarrayFrame.h](include/sensorarrayFrame.h) | 定义帧、测量载荷、有效位图与诊断元数据。 / Defines frames, measurement payloads, validity masks and diagnostic metadata. |
| [sensorarrayFrameBuilder.c](sensorarrayFrameBuilder.c) · [include/sensorarrayFrameBuilder.h](include/sensorarrayFrameBuilder.h) | 初始化无效值与掩码，并由 expected/acquired 推导采集完成和组间偏移。 / Initialises invalid values and masks and derives acquisition completeness and inter-group timing skew. |
| [sensorarrayMeasure.c](sensorarrayMeasure.c) · [include/sensorarrayMeasure.h](include/sensorarrayMeasure.h) | 连接矩阵坐标、模拟路由、FDC 时序/回退与 ADS 兼容操作，并提供共享的测量策略和诊断钩子。 / Connects matrix coordinates, analogue routes, FDC timing/fallback and ADS helpers and supplies shared measurement policies and diagnostic hooks. `sensorarrayFdcRefClockSource_t` `sensorarrayFdcFrequencyDiag_t` |
| [sensorarrayMeasurementMode.c](sensorarrayMeasurementMode.c) · [include/sensorarrayMeasurementMode.h](include/sensorarrayMeasurementMode.h) | 区分请求受理、切换、应用、失败、运行故障和有界恢复，向控制任务提供模式域快照。 / Distinguishes acceptance, transition, application, rejection, runtime fault and bounded recovery and exposes mode-domain snapshots. `sensorarrayMeasurementMode_t` `sensorarrayMeasurementState_t` |
| [sensorarrayMeasurementSelfTest.c](sensorarrayMeasurementSelfTest.c) · [include/sensorarrayMeasurementSelfTest.h](include/sensorarrayMeasurementSelfTest.h) | 对模式、路由、规划、ADS 数学与缓存等纯逻辑执行固件内自检并累计检查项。 / Runs firmware self-checks on pure mode, route, plan, ADS math and cache logic and counts checks. `sensorarrayMeasurementSelfTestResult_t` |
| [sensorarrayRouteController.c](sensorarrayRouteController.c) · [include/sensorarrayRouteController.h](include/sensorarrayRouteController.h) | 依照板图控制 FDC SD/休眠、TMUX、ADS 电源与参考，独立记录指令、读回、已知状态和安全收尾错误。 / Controls FDC SD/sleep, TMUX, ADS power and reference according to the board map and tracks intent, readback, known state and cleanup errors separately. `sensorarrayRouteSnapshot_t` `sensorarrayRouteController_t` |
| [include/sensorarrayRoutePolicy.h](include/sensorarrayRoutePolicy.h) | 以同一判断核对指令与 GPIO 观测电平，供生产路由及主机策略测试调用。 / Compares commanded and observed GPIO levels in a policy shared by production routing and host policy tests. `sensorarrayRouteExpectedControl_t` `sensorarrayRouteObservedControl_t` |
| [sensorarrayRowModeProfile.c](sensorarrayRowModeProfile.c) · [include/sensorarrayRowModeProfile.h](include/sensorarrayRowModeProfile.h) | 解析八位 C/V/R 行字符串，维护 profile 域的受理、应用、拒绝和世代。 / Parses the eight-position C/V/R row string and maintains profile-domain acceptance, application, rejection and generation. `sensorarrayRowModeProfileState_t` `sensorarrayRowModeProfile_t` |
| [sensorarrayScanPlan.c](sensorarrayScanPlan.c) · [include/sensorarrayScanPlan.h](include/sensorarrayScanPlan.h) | 构建活动行、逐行模式与测量单元计划并保存关联的 profile 身份。 / Builds active-row, per-row mode and measurement plans with the associated profile identity. `sensorarrayCellOpKind_t` `sensorarrayCellOp_t` |

## 函数 / Functions

签名按源文件排列，参数顺序即调用顺序。下表标出生产代码中的调用者或回调注册者（最多两个）。`static` 限于所在编译单元。错误码列于对应行为后，只列函数直接返回的常量；硬件与下层调用可传播其他错误。

Signatures are grouped by source in call-argument order. The last column names up to two production callers or callback registrars. `static` entries are local to their translation unit. Constants after a description are explicit error returns. Underlying operations may propagate additional errors.

[sensorarrayRoutePolicy.h](#sensorarrayroutepolicyh) · [sensorarrayConfigIdentity.c](#sensorarrayconfigidentityc) · [sensorarrayFrameBuilder.c](#sensorarrayframebuilderc) · [sensorarrayMeasure.c](#sensorarraymeasurec) · [sensorarrayMeasurementMode.c](#sensorarraymeasurementmodec) · [sensorarrayMeasurementSelfTest.c](#sensorarraymeasurementselftestc) · [sensorarrayRouteController.c](#sensorarrayroutecontrollerc) · [sensorarrayRowModeProfile.c](#sensorarrayrowmodeprofilec) · [sensorarrayScanPlan.c](#sensorarrayscanplanc)

### sensorarrayRoutePolicy.h

| C 调用签名 / C signature | 行为及返回 / Behaviour and return | 调用位置 / Called by |
| --- | --- | --- |
| `static inline bool sensorarrayRouteControlReadbackMatches(const sensorarrayRouteExpectedControl_t *expected, const sensorarrayRouteObservedControl_t *observed)` [29](include/sensorarrayRoutePolicy.h#L29) | 逐项核对逻辑源、SW、SELA、SELB 的命令值与 GPIO 观测。 / Compare the logical source and commanded/observed SW, SELA and SELB control levels. | `sensorarrayMeasurementSelfTestRun` / `sensorarrayRouteGpioMatches` |

### sensorarrayConfigIdentity.c

| C 调用签名 / C signature | 行为及返回 / Behaviour and return | 调用位置 / Called by |
| --- | --- | --- |
| `static bool sensorarrayConfigIdentitySame(const sensorarrayConfigIdentity_t *a, const sensorarrayConfigIdentity_t *b)` [10](sensorarrayConfigIdentity.c#L10) | 比较两次配置的会话、各域身份、模式/行数状态与八行 profile。 / Compare session, domain identities, mode/rows state and all eight row modes across captures. | `sensorarrayConfigIdentityCapture` |
| `void sensorarrayConfigIdentityCapture(uint64_t sessionId, uint32_t bootId, const sensorarrayMeasurementModeSnapshot_t *mode, const sensorarrayFrameConfigSnapshot_t *rows, const sensorarrayRowModeProfile_t *profile, uint32_t frameBoundary, sensorarrayConfigIdentity_t *outIdentity)` [22](sensorarrayConfigIdentity.c#L22) | 将 session、boot、mode、rows 与 profile 合成候选配置身份，相同配置沿用 revision 和首次生效边界。 / Combine session, boot, mode, rows and profile into a candidate identity, preserving revision and first applied boundary for identical configurations. | `sensorarrayPublishConfigAuthority` / `sensorarrayCaptureFrameIdentity` |
| `void sensorarrayConfigIdentityPublish(const sensorarrayConfigIdentity_t *identity, const sensorarrayMeasurementModeSnapshot_t *mode, const sensorarrayRowModeProfile_t *profile, const sensorarrayFrameConfigSnapshot_t *rows)` [57](sensorarrayConfigIdentity.c#L57) | 在短临界区内同时公布已应用身份和三个配置域的最后终态。 / Publish the applied identity and last terminals for all three configuration domains in one short critical section. | `sensorarrayPublishConfigAuthority` |
| `bool sensorarrayConfigIdentityCopy(sensorarrayConfigAuthority_t *outSnapshot)` [82](sensorarrayConfigIdentity.c#L82) | 在同一临界区复制整个权威对象；返回值指示是否已有发布结果。 / Copy the entire authority object under one critical section and return whether it has been published. | `sensorarrayConfigIdentityFormat` / `sensorarrayUsbControlHandle` |
| `static const char *sensorarrayConfigOutcomeName(sensorarrayConfigOutcome_t outcome)` [93](sensorarrayConfigIdentity.c#L93) | 将配置请求的最后结果编码为 CONFIG 查询所用的稳定标签。 / Map a domain's last outcome to the stable CONFIG response label. | `sensorarrayConfigIdentityFormat` |
| `esp_err_t sensorarrayConfigIdentityFormat(char *response, size_t responseSize)` [99](sensorarrayConfigIdentity.c#L99) | 格式化一次原子快照为 CONFIG 回复，尚未发布或缓冲区不足会返回错误。 / Format one atomic snapshot as a CONFIG reply and report unavailable state or insufficient buffer space. `ESP_ERR_INVALID_ARG` `ESP_ERR_INVALID_STATE` `ESP_ERR_INVALID_SIZE` | `sensorarrayRuntimeQueryCommand` |

### sensorarrayFrameBuilder.c

| C 调用签名 / C signature | 行为及返回 / Behaviour and return | 调用位置 / Called by |
| --- | --- | --- |
| `void sensorarrayFrameBuilderInitInvalid(sensorarrayFrame_t *frame)` [10](sensorarrayFrameBuilder.c#L10) | 将整帧初始化为未采集并设置每个测点的无效哨兵与错误原因。 / Initialise the frame as unacquired and seed cell invalid sentinels and reasons. | `sensorarrayAdsMatrixEngineReadFrame` / `sensorarrayMixedRowEngineReadFrame` |
| `uint64_t sensorarrayFrameBuilderActiveCellMask(uint8_t rowMask)` [26](sensorarrayFrameBuilder.c#L26) | 将 8 位活动行掩码展开为每行 8 位的 64 位测点掩码。 / Expand an eight-bit row mask to a 64-bit cell mask with eight bits per row. | `sensorarrayAdsMatrixFinishFrame` / `sensorarrayAdsMatrixCompleteCrcDisposition` |
| `bool sensorarrayFrameBuilderAcquisitionComplete(uint64_t acquiredMask, uint64_t expectedMask)` [37](sensorarrayFrameBuilder.c#L37) | 只有 acquiredMask 与 expectedMask 完全相等时才返回 true。 / Return true only when acquiredMask equals expectedMask. | `sensorarrayMixedRowEngineReadFrame` / `sensorarrayMeasureReadFdcMatrixFrameSnapshot` |
| `uint64_t sensorarrayFrameBuilderMaxGroupSkewUs(uint64_t capStartUs, uint64_t capEndUs, uint64_t voltStartUs, uint64_t voltEndUs, uint64_t resStartUs, uint64_t resEndUs)` [43](sensorarrayFrameBuilder.c#L43) | 忽略无时间记录的组，从非零起止时间中取最大减最小，单位 µs。 / Ignore missing timestamps and subtract the earliest nonzero group bound from the latest in µs. | `sensorarrayAdsMatrixFinishFrame` / `sensorarrayMixedRowEngineReadFrame` |

### sensorarrayMeasure.c

| C 调用签名 / C signature | 行为及返回 / Behaviour and return | 调用位置 / Called by |
| --- | --- | --- |
| `bool sensorarrayMeasureDebugGpioIsValid(int gpio)` [50](sensorarrayMeasure.c#L50) | 检查配置的调试 GPIO 编号是否可用。 / Check whether the configured timing/debug GPIO number is usable. | `sensorarrayMeasureDebugTimingGpioInit` / `sensorarrayMeasureDebugPulse` |
| `void sensorarrayMeasureDebugTimingGpioInit(void)` [55](sensorarrayMeasure.c#L55) | 初始化调试用的行、帧及读窗 GPIO。 / Initialise the optional row, frame and read-window timing GPIOs. | `sensorarrayMeasureDebugTimingGpioPrepare` / `sensorarrayMeasureReadFdcMatrixFrameSnapshot` |
| `void sensorarrayMeasureDebugPulse(int gpio)` [96](sensorarrayMeasure.c#L96) | 在启用时序观测且引脚有效时产生短 GPIO 脉冲，供外部示波器定位事件。 / Emit a scope strobe on an enabled and valid diagnostic pin. | `sensorarrayMeasureSelectFdcRowWhileSleeping` / `sensorarrayMeasureDebugPulseRowStrobe` |
| `void sensorarrayMeasureDebugReadWindow(sensorarrayFdcDeviceId_t devId, bool active)` [108](sensorarrayMeasure.c#L108) | 为选定 FDC 设置读窗口 GPIO，使外部仪器可区分 primary/secondary 的访问时段。 / Mark the selected FDC read interval on its diagnostic pin. | `sensorarrayMeasureFdcRunDeviceEpochAfterSleep` / `sensorarrayMeasureDebugSetReadWindow` |
| `void sensorarrayMeasureDebugTimingGpioPrepare(void)` [119](sensorarrayMeasure.c#L119) | 按配置初始化帧、行及读取窗口的调试 GPIO 以备实验使用。 / Prepare the configured timing pins before marking rows, frames or reads. | `sensorarrayRunFdcWaveDebugLoop` |
| `void sensorarrayMeasureDebugPulseRowStrobe(void)` [124](sensorarrayMeasure.c#L124) | 在行边界脉冲所配置的 row-strobe 引脚。 / Pulse the configured row strobe at a row boundary. | `sensorarrayRunFdcWaveDebugLoop` |
| `void sensorarrayMeasureDebugPulseFrameStrobe(void)` [129](sensorarrayMeasure.c#L129) | 在帧边界脉冲所配置的 frame-strobe 引脚。 / Pulse the configured frame strobe at a frame boundary. | `sensorarrayRunFdcWaveDebugLoop` |
| `void sensorarrayMeasureDebugSetReadWindow(sensorarrayFdcDeviceId_t devId, bool active)` [134](sensorarrayMeasure.c#L134) | 将指定器件的读窗口标记为开始或结束。 / Set or clear the per-device scope read-window marker. | `sensorarrayFdcWaveDebugReadDevice` |
| `esp_err_t sensorarrayMeasureEnsureLock(void)` [174](sensorarrayMeasure.c#L174) | 按需建立测量策略使用的共享互斥量。 / Ensure that the shared measurement mutex exists. | `sensorarrayMeasureTakeLock` |
| `esp_err_t sensorarrayMeasureTakeLock(void)` [189](sensorarrayMeasure.c#L189) | 取得测量互斥量并返回等待或初始化错误。 / Acquire the measurement mutex and propagate acquisition/initialisation error. | `sensorarrayMeasureReadFdcMatrixFrameSnapshot` |
| `void sensorarrayMeasureGiveLock(void)` [203](sensorarrayMeasure.c#L203) | 释放调用方已持有的测量互斥量。 / Release the measurement mutex owned by the caller. | `sensorarrayMeasureReadFdcMatrixFrameSnapshot` |
| `void sensorarrayDelayMs(uint32_t delayMs)` [210](sensorarrayMeasure.c#L210) | 将非零毫秒延迟转换成 RTOS ticks 让出任务；零值立即返回。 / Convert a nonzero millisecond delay to an RTOS task delay. | `sensorarrayMeasureWriteSela` |
| `static const sensorarrayAdsReadPolicy_t *sensorarrayReadPolicyOrDefault(const sensorarrayAdsReadPolicy_t *policy)` [217](sensorarrayMeasure.c#L217) | 选取调用方读策略；未提供时取固件默认值。 / Select the supplied read policy or the firmware default when absent. | `sensorarrayMeasureReadAdsPairUv` |
| `static const char *sensorarrayMatrixDSourceName(sensorarrayMatrixDSourcePolicy_t dSource)` [229](sensorarrayMeasure.c#L229) | 返回当前 D 线信号来源的诊断名称，用于标识路由。 / Name the selected D-line source for route diagnostics. | `sensorarrayMeasureApplyRefPolicy` |
| `static const char *sensorarrayAdsIntRefPolicyName(sensorarrayAdsIntRefPolicy_t policy)` [234](sensorarrayMeasure.c#L234) | 区分 ADS 内部参考的配置策略，以供路由日志显示。 / Name the ADC internal-reference policy used by the route. | `sensorarrayMeasureApplyRefPolicy` |
| `static const char *sensorarrayAdsVbiasPolicyName(sensorarrayAdsVbiasPolicy_t policy)` [247](sensorarrayMeasure.c#L247) | 将 VBIAS 策略转换为输出标签，便于与 INTREF 策略分别核对。 / Format the VBIAS policy independently of the reference setting. | `sensorarrayMeasureApplyRefPolicy` |
| `bool sensorarrayAdsIntRefPolicyUpdates(sensorarrayAdsIntRefPolicy_t policy)` [260](sensorarrayMeasure.c#L260) | 判断参考策略是否要求改变 INTREF 状态及目标值。 / Determine whether the policy changes INTREF and provide the requested state. | `sensorarrayMeasureApplyRefPolicy` |
| `bool sensorarrayAdsVbiasPolicyUpdates(sensorarrayAdsVbiasPolicy_t policy)` [265](sensorarrayMeasure.c#L265) | 判断当前策略是否要求改变 VBIAS 及其目标值。 / Determine whether the policy changes VBIAS and provide the requested state. | `sensorarrayMeasureApplyRefPolicy` |
| `int sensorarrayReadResetGpioLevel(void)` [270](sensorarrayMeasure.c#L270) | 读取板级 ADS RESET 引脚电平；没有配置引脚时返回 −1。 / Read the configured ADS RESET pin, returning −1 when absent. | `sensorarrayMeasureApplyRefPolicy` |
| `esp_err_t sensorarrayMeasureApplyRefPolicy(sensorarrayState_t *state, const char *stage, const char *mode, sensorarrayMatrixDSourcePolicy_t dSource, sensorarrayAdsIntRefPolicy_t intrefPolicy, sensorarrayAdsVbiasPolicy_t vbiasPolicy, const char *reason)` [278](sensorarrayMeasure.c#L278) | 按参考策略设置 INTREF 与 VBIAS，并登记实际配置。 / Apply the requested INTREF/VBIAS policy and track the resulting configuration. `ESP_ERR_INVALID_ARG` | `sensorarrayMeasureApplyFdcCapAdsPolicy` / `sensorarrayMeasureApplyRouteLevels` |
| `esp_err_t sensorarrayMeasureStopAdsBeforeRoute(sensorarrayState_t *state)` [355](sensorarrayMeasure.c#L355) | 在需改参考/路由前停止可能影响切换的 ADS 转换。 / Stop ADS conversion before a route/reference change that requires it. | `sensorarrayMeasureApplyRouteLevels` / `sensorarrayMeasureApplyRoute` |
| `tmux1108Source_t sensorarrayMeasureSourceForSwPhysicalLevel(sensorarraySwPhysicalLevel_t level)` [368](sensorarrayMeasure.c#L368) | 将板上 SW 的物理输出电平映射回 REF/GND 逻辑源。 / Decode a physical SW pin level to the logical REF/GND source. | `sensorarrayMeasureSetSwPhysicalLevel` |
| `esp_err_t sensorarrayMeasureSetSwPhysicalLevel(sensorarrayState_t *state, sensorarraySwPhysicalLevel_t level, const char *reason)` [377](sensorarrayMeasure.c#L377) | 按给定物理电平驱动 SW 并保存读回或错误。 / Drive SW to the physical level and retain readback or error. `ESP_ERR_INVALID_STATE` `ESP_ERR_INVALID_ARG` | 独立接口 / Standalone entry |
| `void sensorarrayMeasureDelayUs(uint32_t delayUs)` [392](sensorarrayMeasure.c#L392) | 非零微秒间隔使用 ROM 微秒延迟，供短硬件建立时间使用。 / Apply a nonzero short hardware-settling delay using the ROM microsecond timer. | `sensorarrayMeasureSelectFdcRowWhileSleeping` / `sensorarrayMeasureSetSelaPathQuiet` |
| `esp_err_t sensorarrayMeasureSetSelaPathQuiet(sensorarrayState_t *state, sensorarraySelaRoute_t selaRoute, uint32_t settleDelayUs)` [399](sensorarrayMeasure.c#L399) | 在 CAP 路径准备中设置 SELA，减少热路径的重复诊断输出。 / Set SELA during CAP path preparation with restrained hot-path logging. `ESP_ERR_INVALID_STATE` `ESP_ERR_INVALID_ARG` | `sensorarrayMeasurePrepareFdcMatrixPath` |
| `esp_err_t sensorarrayMeasureSetFdcSelBPathQuiet(sensorarrayState_t *state)` [419](sensorarrayMeasure.c#L419) | 按板级 FDC 映射设置 SELB，并返回驱动结果。 / Apply the FDC board mapping to SELB and return the switch result. `ESP_ERR_INVALID_STATE` `ESP_ERR_INVALID_ARG` | `sensorarrayMeasurePrepareFdcMatrixPath` |
| `esp_err_t sensorarrayMeasureDisableIntrefKeepVbiasForPassiveMatrix(sensorarrayState_t *state)` [432](sensorarrayMeasure.c#L432) | 为无源矩阵关闭 INTREF 并按策略保持所需的输入偏置。 / Disable INTREF for passive acquisition while retaining the required input bias. `ESP_ERR_INVALID_STATE` | `sensorarrayMeasureKeepAdsBiasForPassiveMatrix` / `sensorarrayMeasureForceAdsReferenceOff` |
| `esp_err_t sensorarrayMeasureKeepAdsBiasForPassiveMatrix(sensorarrayState_t *state)` [458](sensorarrayMeasure.c#L458) | 使用被动矩阵策略停用 INTREF，并保留 ADS 偏置。 / Apply the passive matrix's reference-off, bias-retained ADC policy. | 独立接口 / Standalone entry |
| `esp_err_t sensorarrayMeasureForceAdsReferenceOff(sensorarrayState_t *state)` [463](sensorarrayMeasure.c#L463) | 在隔离或恢复前要求 ADS 内部参考关闭并保留错误。 / Request ADS internal reference off for isolation/recovery and retain errors. | 独立接口 / Standalone entry |
| `int sensorarrayMeasureSwPhysicalReadbackFromControl(const tmuxSwitchControlState_t *ctrl)` [468](sensorarrayMeasure.c#L468) | 从开关控制快照取出 SW 的物理读回电平。 / Extract the physically observed SW level from the switch-control snapshot. | `sensorarrayMeasurePrepareFdcMatrixPath` / `sensorarrayMeasureEnsureFdcMatrixPath` |
| `void sensorarrayMeasureReadFdcPathControl(tmuxSwitchControlState_t *ctrl)` [473](sensorarrayMeasure.c#L473) | 读取 FDC 路径控制引脚，作为随后路由核对的输入。 / Collect FDC path-control pin evidence for route verification. | `sensorarrayMeasureSelectFdcRowWhileSleeping` / `sensorarrayMeasurePrepareFdcMatrixPath` |
| `bool sensorarrayMeasureFdcPathControlMatches(const tmuxSwitchControlState_t *ctrl)` [492](sensorarrayMeasure.c#L492) | 核对所需 FDC 路径与 SW/SELA/SELB 命令、读回的组合。 / Check the FDC route against commanded and observed SW/SELA/SELB state. | `sensorarrayMeasurePrepareFdcMatrixPath` / `sensorarrayMeasureEnsureFdcMatrixPath` |
| `esp_err_t sensorarrayMeasurePrepareFdcMatrixPath(sensorarrayState_t *state, const char *reason)` [517](sensorarrayMeasure.c#L517) | 准备 CAP 矩阵所需的无源参考与开关路径。 / Prepare the reference and switches for passive CAP matrix acquisition. `ESP_ERR_INVALID_STATE` | `sensorarrayFdcSweepRouteRowForCap` / `sensorarrayFdcSweepRunBoot` |
| `esp_err_t sensorarrayMeasureEnsureFdcMatrixPath(sensorarrayState_t *state, const char *reason)` [645](sensorarrayMeasure.c#L645) | 核对当前 CAP 路径，需要时修复并重新读回。 / Check the CAP route and, when needed, reapply it and verify readback. `ESP_ERR_INVALID_ARG` | `sensorarrayMeasureReadFdcMatrixFrameSnapshot` |
| `esp_err_t sensorarrayMeasureSetSwForRoute(sensorarrayState_t *state, const char *stage, uint8_t sColumn, uint8_t dLine, sensorarrayRoutePathKind_t path, tmux1108Source_t swSource, sensorarraySelaRoute_t selaRoute, bool selBLevel, const char *label, const char *status, const char *reason)` [686](sensorarrayMeasure.c#L686) | 检查 TMUX 就绪后将公共开关切至指定源，并报告结果与路由步骤。 / Check switch readiness, apply the requested common source and report the route step. | `sensorarrayMeasureApplyRouteLevels` / `sensorarrayMeasureApplyRoute` |
| `esp_err_t sensorarrayMeasureWriteSela(sensorarrayState_t *state, sensorarraySelaRoute_t requestRoute, uint32_t settleDelayMs, const char *stage, const char *label)` [717](sensorarrayMeasure.c#L717) | 把逻辑 SELA 路径转换为板级电平再写入；无法映射时拒绝。 / Translate the requested SELA route through the board map before driving the pin. | `sensorarrayMeasureSetSelaPath` |
| `sensorarrayFdcDeviceState_t *sensorarrayMeasureGetFdcState(sensorarrayState_t *state, sensorarrayFdcDeviceId_t devId)` [764](sensorarrayMeasure.c#L764) | 从应用状态取指定的主/次 FDC 句柄；没有对应器件时返回 NULL。 / Select the primary or secondary FDC state, returning NULL for an invalid selection. | `sensorarrayMeasureRuntimeConfigsFromApplied` / `sensorarrayMeasureApplyFdcCachedRowConfig` |
| `sensorarrayFdcDeviceState_t *sensorarrayMeasureGetFdcStateForDLine(sensorarrayState_t *state, uint8_t dLine, const sensorarrayFdcDLineMap_t **outMap)` [781](sensorarrayMeasure.c#L781) | 由板级 D 线映射找到实际 FDC 上下文和相应通道。 / Resolve a physical D line to its FDC context and associated channel. | 独立接口 / Standalone entry |
| `const char *sensorarrayMeasureFdcDeviceName(sensorarrayFdcDeviceId_t devId)` [796](sensorarrayMeasure.c#L796) | 将主/次器件选择转换为稳定的诊断标签。 / Format the selected FDC identifier for logs and traces. | `sensorarrayMeasureApplyFdcCellRoute` / `sensorarrayMeasureApplyFdcCachedRowConfig` |
| `const char *sensorarrayMeasureEspErrName(esp_err_t err)` [801](sensorarrayMeasure.c#L801) | 为 ESP 错误码生成诊断标签，用于区分总线、参数和状态错误。 / Translate the ESP error code for diagnostic reporting. | `sensorarrayMeasureFdcDeviceLevelResync` / `sensorarrayMeasureFdcFormalPrecheckDevice` |
| `bool sensorarrayMeasureDecodeMatrixIndex(uint8_t matrixIndex, uint8_t *outSColumn, uint8_t *outDLine)` [823](sensorarrayMeasure.c#L823) | 把行优先测点下标还原为 1 起始的 S/D 坐标。 / Decode a row-major index into one-based S/D coordinates. | `sensorarrayMeasureComputeFdcFrameCapTotalPf` / `sensorarrayMeasureRequestFdcCellRescue` |
| `bool sensorarrayMeasureMakeFdcCellTarget(sensorarrayState_t *state, uint8_t sColumn, uint8_t dLine, sensorarrayFdcCellTarget_t *outTarget)` [836](sensorarrayMeasure.c#L836) | 从 S、D 坐标和板图创建包含目标器件、通道等信息的定位结构。 / Resolve S/D coordinates through the board map into a device/channel cell target. | `sensorarrayMeasureUpdateFdcRuntimeProfiles` / `sensorarrayFdcSweepCaptureCandidate` |
| `sensorarrayFdcCellConfigCache_t *sensorarrayMeasureGetFdcCellCache(sensorarrayState_t *state, const sensorarrayFdcCellTarget_t *target)` [862](sensorarrayMeasure.c#L862) | 取得指定 S/D 测点的调谐配置缓存。 / Return the per-cell tuning configuration cache for the supplied S/D coordinates. | `sensorarrayMeasureUpdateFdcRuntimeProfiles` / `sensorarrayFdcSweepUpdateStateCacheFromCandidate` |
| `bool sensorarrayMeasureFdcReasonEquals(const char *reason, const char *expected)` [874](sensorarrayMeasure.c#L874) | 安全比较两个 FDC 原因标签，以决定是否适用某一恢复策略。 / Compare the FDC reason labels before classifying a recovery. | `sensorarrayMeasureFdcRescueReasonIsManual` / `sensorarrayMeasureFdcRescueReasonIsProfileTooSlow` |
| `bool sensorarrayMeasureFdcRescueReasonIsManual(const char *reason)` [879](sensorarrayMeasure.c#L879) | 识别操作员强制扫频的原因标签。 / Recognize manual forced-sweep reasons. | `sensorarrayMeasureRequestFdcCellRescue` |
| `bool sensorarrayMeasureFdcRescueReasonIsProfileTooSlow(const char *reason)` [888](sensorarrayMeasure.c#L888) | 识别配置转换周期超过当前帧目标的原因标签。 / Recognize diagnostic reasons for a profile slower than its frame target. | `sensorarrayMeasureFdcRescueReasonIsFastSweep` / `sensorarrayMeasureFdcRescueReasonIsHard` |
| `bool sensorarrayMeasureFdcRescueReasonIsFastSweep(const char *reason)` [894](sensorarrayMeasure.c#L894) | 按原因及配置判断是否交给 fast-sweep 恢复，慢 profile 受专用开关控制。 / Classify reasons eligible for fast-sweep rescue under the configured policy. | `sensorarrayMeasureRequestFdcCellRescue` |
| `bool sensorarrayMeasureFdcRescueReasonIsHard(const char *reason)` [918](sensorarrayMeasure.c#L918) | 把原因归类为硬故障路径；慢 profile 由策略单独决定。 / Classify reasons that warrant hard rescue under the configured slow-profile policy. | `sensorarrayMeasureRequestFdcCellRescue` |
| `esp_err_t sensorarrayMeasureRequestFdcCellRescue(sensorarrayState_t *state, uint8_t matrixIndex, const char *reason)` [955](sensorarrayMeasure.c#L955) | 登记测点恢复请求与原因，交由合适的后续边界处理。 / Record a cell rescue request and its cause for handling at an eligible boundary. `ESP_ERR_INVALID_ARG` `ESP_ERR_INVALID_STATE` | `sensorarrayMeasureApplyFdcCachedRowConfig` / `sensorarrayMeasureUpdateFdcRuntimeProfiles` |
| `esp_err_t sensorarrayMeasureFdcDiscardStaleSamples(sensorarrayState_t *state, const sensorarrayFdcCellTarget_t *target, uint8_t discardCount, const char *reason)` [1063](sensorarrayMeasure.c#L1063) | 丢弃重新配置后的旧转换，阻止来自前一状态的样本进入当前结果。 / Discard stale conversions following reconfiguration before admitting current results. `ESP_ERR_INVALID_ARG` `ESP_ERR_INVALID_STATE` | `sensorarrayMeasureApplyFdcCellRoute` / `sensorarrayFdcSweepCaptureCandidate` |
| `sensorarrayFdcDeviceId_t sensorarrayMeasureFdcDeviceIdFromState(const sensorarrayFdcDeviceState_t *fdcState)` [1099](sensorarrayMeasure.c#L1099) | 按传入的 FDC 状态上下文区分主/次器件。 / Derive primary/secondary device identity from the supplied FDC state. | `sensorarrayMeasureWaitFdcAutoscanFrameReady` |
| `uint8_t sensorarrayMeasureFdcUnreadMaskFromStatus(const Fdc2214CapStatus_t *status)` [1107](sensorarrayMeasure.c#L1107) | 从已读取的 STATUS 字提取四个通道的 unread 位。 / Extract the four unread channel bits from an observed STATUS word. | `sensorarrayMeasureFdcReadStatusForWatchdogDiagOnly` / `sensorarrayMeasureFdcBuildReadyState` |
| `sensorarrayFdcReadyDecoded_t sensorarrayMeasureFdcBuildReadyState(const Fdc2214CapStatus_t *status, uint8_t requiredMask)` [1122](sensorarrayMeasure.c#L1122) | 解释 STATUS 的 DRDY 和 unread 等位为结构化的就绪状态。 / Decode DRDY and channel-unread evidence from STATUS into a structured readiness state. | `sensorarrayMeasureFdcReadStatusAndAckIntbForReady` / `sensorarrayMeasureFdcReadStatusForWatchdogDiagOnly` |
| `void sensorarrayMeasureFdcApplyReadyDecoded(sensorarrayFdcReadyState_t *ready, const sensorarrayFdcReadyDecoded_t *decoded)` [1142](sensorarrayMeasure.c#L1142) | 把读回的就绪证据累入本轮轨迹与计数。 / Apply decoded readiness evidence to the current result and counters. | `sensorarrayMeasureFdcReadStatusAndAckIntbForReady` / `sensorarrayMeasureFdcReadStatusForWatchdogDiagOnly` |
| `bool sensorarrayMeasureFdcDeglitchCodeValid(uint8_t deglitchCode)` [1158](sensorarrayMeasure.c#L1158) | 仅接受 FDC 实现支持的去毛刺带宽寄存器码。 / Validate the hardware deglitch selector against the supported settings. | `sensorarrayMeasureSelectedFdcDeglitch` / `sensorarrayMeasureFdcBuildMuxConfig` |
| `Fdc2214CapDeglitch_t sensorarrayMeasureSelectedFdcDeglitch(const sensorarrayFdcDeviceState_t *fdcState)` [1171](sensorarrayMeasure.c#L1171) | 取得该通道应使用的去毛刺滤波设置。 / Get the deglitch filter setting selected for this channel. | `sensorarrayMeasureEnsureFdcAutoscan4ch` |
| `uint8_t sensorarrayMeasureFdcRegForChannel(uint8_t base, Fdc2214CapChannel_t channel)` [1196](sensorarrayMeasure.c#L1196) | 从基础寄存器地址和通道编号取得 FDC 通道寄存器。 / Resolve an FDC channel register from its base address and channel index. | `sensorarrayMeasureApplyFdcCachedRowConfig` / `sensorarrayMeasureReadFdcRuntimeChannelConfigs` |
| `uint32_t sensorarrayMeasureFdcDeglitchBandwidthHz(uint8_t deglitchCode)` [1201](sensorarrayMeasure.c#L1201) | 将支持的去毛刺码换算为物理带宽 Hz，供 profile 诊断。 / Convert a deglitch register code to its filter bandwidth in hertz. | `sensorarrayFdcMergeDeglitchForRowDevice` |
| `uint8_t sensorarrayMeasureFdcSafeDefaultDeglitch(void)` [1217](sensorarrayMeasure.c#L1217) | 按构建的时钟和板级参数选择保守滤波配置。 / Choose the conservative default deglitch setting for the configured board/clock. | `sensorarrayMeasureApplyFdcCachedRowConfig` / `sensorarrayMeasureFdcVerifySleepExit` |
| `void sensorarrayMeasureMarkFdcAppliedCellDirty(sensorarrayState_t *state, const sensorarrayFdcCellTarget_t *target)` [1222](sensorarrayMeasure.c#L1222) | 标记测点的已应用配置需要重新写入或核验。 / Mark the cell's applied configuration dirty for reapplication or verification. | `sensorarrayMeasureUpdateFdcRuntimeProfiles` |
| `bool sensorarrayMeasureAppliedRowConfigMatches(const sensorarrayFdcAppliedRowConfig_t *applied, const sensorarrayFdcAppliedRowConfig_t *expected)` [1237](sensorarrayMeasure.c#L1237) | 检查当前硬件状态缓存与目标行寄存器配置相符。 / Compare cached applied hardware state against the target row register settings. | `sensorarrayMeasureApplyFdcCachedRowConfig` |
| `uint16_t sensorarrayMeasureFdcBuildMuxConfig(uint8_t deglitchCode)` [1264](sensorarrayMeasure.c#L1264) | 合成四通道 round-robin/autoscan MUX 配置字；无效去毛刺参数使用安全默认。 / Build the four-channel autoscan MUX register, substituting a supported filter code if needed. | `sensorarrayMeasureApplyFdcCachedRowConfig` / `sensorarrayMeasureFdcVerifySleepExit` |
| `uint16_t sensorarrayMeasureFdcConfigBaseWithoutSleep(const sensorarrayFdcDeviceState_t *fdcState)` [1276](sensorarrayMeasure.c#L1276) | 清除配置字中的 SLEEP 位，取得可比较的活动状态基值。 / Strip the SLEEP bit to obtain the comparable active configuration word. | `sensorarrayMeasureApplyFdcCachedRowConfig` / `sensorarrayMeasureFdcSetSleepMode` |
| `uint32_t sensorarrayMeasureFdcConfigFingerprint(const sensorarrayFdcAppliedRowConfig_t *config)` [1301](sensorarrayMeasure.c#L1301) | 从实际影响采集的设置形成配置指纹，供识别缓存变化。 / Construct a fingerprint of acquisition settings for cache-change detection. | `sensorarrayMeasurePromoteFdcExpectedProfileToCellCache` / `sensorarrayMeasureApplyFdcCachedRowConfig` |
| `sensorarrayFdcCellConfigCache_t *sensorarrayMeasureFdcRowDeviceCache(sensorarrayState_t *state, uint8_t row, sensorarrayFdcDeviceId_t devId, uint8_t ch, uint8_t *outDIndex)` [1325](sensorarrayMeasure.c#L1325) | 定位行与 FDC 器件组合的配置缓存。 / Find the configuration cache for this row/device combination. | `sensorarrayMeasurePromoteFdcExpectedProfileToCellCache` / `sensorarrayMeasureApplyFdcCachedRowConfig` |
| `uint8_t sensorarrayFdcMergeDeglitchForRowDevice(sensorarrayState_t *state, uint8_t row, sensorarrayFdcDeviceId_t devId, uint8_t safeDefaultDeglitch)` [1346](sensorarrayMeasure.c#L1346) | 在同一器件共用的滤波寄存器上合并各通道的带宽要求。 / Combine per-channel bandwidth requirements for a device-shared deglitch register. | `sensorarrayMeasureApplyFdcCachedRowConfig` |
| `bool sensorarrayMeasureFdcAutoscanConfigLooksCurrent(const sensorarrayFdcDeviceState_t *fdcState, uint8_t expectedDeglitch)` [1390](sensorarrayMeasure.c#L1390) | 将缓存的 muxConfigReg 等项与期望的四通道 autoscan 及滤波设置比对。 / Check whether the cached MUX settings describe the required autoscan profile. | 独立接口 / Standalone entry |
| `esp_err_t sensorarrayMeasureVerifyFdcChannelConfigApplied(sensorarrayFdcDeviceState_t *fdcState)` [1415](sensorarrayMeasure.c#L1415) | 读取通道寄存器并检查写入后的参数与预期一致。 / Read channel registers and compare their applied settings with the intended profile. `ESP_ERR_INVALID_STATE` | `sensorarrayMeasureEnsureFdcAutoscan4ch` |
| `esp_err_t sensorarrayMeasureEnsureFdcAutoscan4ch(sensorarrayState_t *state, sensorarrayFdcDeviceId_t devId)` [1469](sensorarrayMeasure.c#L1469) | 为目标 FDC 建立或恢复四通道自动扫描设置。 / Establish or restore four-channel autoscan on the selected FDC. `ESP_ERR_INVALID_STATE` `ESP_ERR_INVALID_RESPONSE` | 独立接口 / Standalone entry |
| `esp_err_t sensorarrayMeasureWaitFdcAutoscanFrameReady(sensorarrayFdcDeviceState_t *fdcState, uint8_t row, uint32_t timeoutMs, uint16_t *outStatus)` [1536](sensorarrayMeasure.c#L1536) | 在给定预算内检查指定 FDC 的 autoscan 就绪并保留读取错误或超时。 / Wait within timeoutMs for one device's autoscan readiness and preserve the observed error. | `sensorarrayMeasureDiscardFdcAutoscanRow` |
| `void sensorarrayMeasurePollFdcReady(sensorarrayFdcDeviceState_t *fdcState, sensorarrayFdcReadyState_t *ready)` [1589](sensorarrayMeasure.c#L1589) | 在调用者给予的时限内检查 FDC ready 并保留观测结果。 / Check FDC readiness within the supplied wait budget and retain evidence. | `sensorarrayMeasureWaitBothFdcAutoscanFrameReady` |
| `esp_err_t sensorarrayMeasureWaitBothFdcAutoscanFrameReady(sensorarrayState_t *state, uint8_t row, uint32_t timeoutMs, sensorarrayFdcReadyState_t *primaryReady, sensorarrayFdcReadyState_t *secondaryReady)` [1615](sensorarrayMeasure.c#L1615) | 使用共同期限检查两颗 FDC，在两个输出对象中分别保留 ready/error。 / Check both devices against one deadline and return separate readiness observations. | 独立接口 / Standalone entry |
| `sensorarrayFdcWorkerContext_t *sensorarrayMeasureFdcWorkerContext(sensorarrayFdcDeviceId_t devId)` [1689](sensorarrayMeasure.c#L1689) | 定位主或次 FDC 对应的持久 worker 上下文。 / Resolve the persistent worker context for the primary or secondary FDC. | `sensorarrayMeasureFdcVerifySleepExit` / `sensorarrayMeasureFdcReadStatusAndAckIntbForReady` |
| `void sensorarrayMeasureRecordFirstErr(esp_err_t err, esp_err_t *firstErr)` [1694](sensorarrayMeasure.c#L1694) | 仅在未记首错时保存本次失败，后续清理错误不会盖写。 / Retain the first failure, preserving it through later cleanup. | `sensorarrayMeasureFdcVerifySleepExit` / `sensorarrayMeasureFdcDeviceLevelResync` |
| `esp_err_t sensorarrayMeasureFdcFormalPrecheckDevice(sensorarrayState_t *state, sensorarrayFdcDeviceId_t devId)` [1701](sensorarrayMeasure.c#L1701) | 在生产并行采集前检查该器件配置、可达性与就绪先决条件。 / Check configuration, access and ready prerequisites for one device before production parallel reads. | `sensorarrayMeasureRunFdcFormalPrecheck` |
| `esp_err_t sensorarrayMeasureRunFdcFormalPrecheck(sensorarrayState_t *state)` [1811](sensorarrayMeasure.c#L1811) | 对主、次 FDC 分别执行正式采样前置检查，保留最早错误。 / Precheck both devices before production acquisition, preserving the first error. | `sensorarrayMeasureReadFdcMatrixFrameSnapshot` |
| `void IRAM_ATTR sensorarrayMeasureFdcIntbIsr(void *arg)` [1821](sensorarrayMeasure.c#L1821) | 在 GPIO 中断中记录 INTB 边沿并唤醒等待任务。 / Record an INTB GPIO edge in interrupt context and wake the waiting task. | `sensorarrayMeasureEnsureFdcIntb` |
| `esp_err_t sensorarrayMeasureEnsureFdcIntb(sensorarrayFdcWorkerContext_t *ctx)` [1843](sensorarrayMeasure.c#L1843) | 安装或确认 FDC INTB GPIO/ISR 资源供 worker 使用。 / Establish the FDC INTB GPIO/ISR resources used by acquisition workers. `ESP_ERR_INVALID_ARG` | `sensorarrayMeasureEnsureFdcWorkers` |
| `void sensorarrayMeasureFdcPrepareIntbEpoch(sensorarrayFdcWorkerContext_t *ctx, uint32_t epochId)` [1910](sensorarrayMeasure.c#L1910) | 保存本轮 INTB 边沿基线与目标任务，拒绝过期唤醒。 / Arm the target task and edge baseline for this INTB epoch. | `sensorarrayMeasureReadFdcMatrixRowSerialEpoch` / `sensorarrayMeasureReadFdcMatrixRowParallelEpoch` |
| `uint32_t sensorarrayMeasureFdcWorkerEdgeCount(sensorarrayFdcDeviceId_t devId)` [1925](sensorarrayMeasure.c#L1925) | 返回指定 worker 记录到的 INTB 边沿总数，未知上下文返回零。 / Read the selected worker's INTB edge counter, returning zero for an absent context. | `sensorarrayMeasureReadFdcMatrixFrameSnapshot` |
| `bool sensorarrayMeasureResolveWorkerCore(const char *workerName, int configuredCore, int defaultCore, BaseType_t *outCore, bool *outPinned, const char **outReason)` [1931](sensorarrayMeasure.c#L1931) | 核对请求的核心与默认核心，给出实际绑核设置。 / Resolve configured/default core settings to the actual worker affinity. | `sensorarrayMeasureEnsureFdcWorkers` |
| `void sensorarrayMeasureCleanupFdcWorkers(void)` [1977](sensorarrayMeasure.c#L1977) | 检查两个 worker 均已静止后再清理任务/同步对象。 / Reclaim worker infrastructure only after both workers are quiescent. | `sensorarrayMeasureEnsureFdcWorkers` |
| `const char *sensorarrayMeasureFdcReadyResultName(sensorarrayFdcReadyResult_t result)` [1995](sensorarrayMeasure.c#L1995) | 将 ready 结果分类转换为现场可读的诊断标签。 / Name the observed readiness class for row diagnostics. | `sensorarrayMeasureBuildFdcRead4Result` / `sensorarrayMeasureFdcDeferDeviceRepairAfterParallel` |
| `bool sensorarrayMeasureFdcReadyResultIsSoftInvalid(const sensorarrayFdcReadyState_t *ready)` [2030](sensorarrayMeasure.c#L2030) | 检查未就绪分类是否允许把该点标为软无效继续处理。 / Check whether readiness evidence permits a soft-invalid sample classification. | `sensorarrayMeasureReadFdcAutoscan4chMasked` / `sensorarrayMeasureMarkFdcNoFreshSamples` |
| `uint32_t sensorarrayMeasureFdcReadyGuardDeadlineUs(uint32_t estimatedRoundUs)` [2036](sensorarrayMeasure.c#L2036) | 计算时限内保留保护余量后的最迟就绪判定时间。 / Derive the ready-check deadline with its guard margin. | `sensorarrayMeasureFdcComputeActualIntbWaitUs` / `sensorarrayMeasureFdcClassifyUnreadFullNoDrdySoft` |
| `static const char *sensorarrayMeasureFdcParallelFallbackReason(bool configEnabled, bool primaryBusEnabled, bool secondaryBusEnabled, bool sameBus, esp_err_t err)` [2043](sensorarrayMeasure.c#L2043) | 按并行开关、两条总线的可用性、是否共享总线和返回码，解释本次回退。 / Classify fallback by build policy, bus availability/topology and the returned error. | `sensorarrayMeasureReadFdcMatrixFrameSnapshot` |
| `void sensorarrayMeasureFillFdcDeviceI2cDelta(const Fdc2214CapI2cStats_t *before, const Fdc2214CapI2cStats_t *after, sensorarrayFdcDeviceTiming_t *timing)` [2071](sensorarrayMeasure.c#L2071) | 对起止总线计数求差，形成该器件本轮 I²C 使用统计。 / Subtract starting bus counters from ending values for per-device I²C usage. | `sensorarrayMeasureReadFdcMatrixFrameSnapshot` |
| `esp_err_t sensorarrayMeasureReadFdcMatrixFrame(sensorarrayState_t *state, sensorarrayFdcMatrixFrame_t *outFrame)` [2087](sensorarrayMeasure.c#L2087) | 由默认活动行设置进入 FDC 矩阵采集路径，返回填好的目标帧或错误。 / Invoke matrix acquisition for the full default row selection and fill the supplied frame. | `sensorarrayFdcMatrixEngineReadFrame` |
| `esp_err_t sensorarrayMeasureReadFdcMatrixFrameRows(sensorarrayState_t *state, sensorarrayFdcMatrixFrame_t *outFrame, uint8_t requestedRows)` [2094](sensorarrayMeasure.c#L2094) | 用指定行数建立相应配置快照后执行 FDC 矩阵采集。 / Build a row-count selection and acquire the corresponding FDC matrix. | 独立接口 / Standalone entry |
| `esp_err_t sensorarrayMeasureReadFdcMatrixFrameSnapshot(sensorarrayState_t *state, sensorarrayFdcMatrixFrame_t *outFrame, const sensorarrayFrameConfigSnapshot_t *snapshot)` [2108](sensorarrayMeasure.c#L2108) | 采集传入 rows 快照所规定的活动行，并保留其原始配置域。 / Acquire rows from the supplied snapshot while preserving its configuration domain. `ESP_ERR_INVALID_ARG` `ESP_ERR_INVALID_STATE` | `sensorarrayFdcMatrixEngineReadFrame` / `sensorarrayMeasureReadFdcMatrixFrame` |
| `bool sensorarrayFastSpeedIsEnabled(void)` [3022](sensorarrayMeasure.c#L3022) | 查询当前高速采集策略状态供调度和诊断使用。 / Read the active fast-operation policy. | 独立接口 / Standalone entry |
| `void sensorarrayFastSpeedSetEnabled(bool enabled)` [3027](sensorarrayMeasure.c#L3027) | 更新高速操作开关，供之后的采集路径查询。 / Set the runtime fast-operation flag used by subsequent acquisitions. | `sensorarrayInitRuntime` |
| `void sensorarrayMeasureFdcProfileSetSummary(bool enabled)` [3032](sensorarrayMeasure.c#L3032) | 选择是否发布 FDC 时序/配置汇总诊断。 / Enable or disable summary-level FDC profiling. | 独立接口 / Standalone entry |
| `void sensorarrayMeasureFdcProfileSetRow(bool enabled)` [3037](sensorarrayMeasure.c#L3037) | 设置逐行 FDC profile 诊断输出开关。 / Enable or disable per-row FDC profiling. | 独立接口 / Standalone entry |
| `void sensorarrayMeasureFdcProfileSetDevice(bool enabled)` [3042](sensorarrayMeasure.c#L3042) | 设置逐器件 FDC profile 诊断输出开关。 / Enable or disable per-device FDC profiling. | 独立接口 / Standalone entry |
| `void sensorarrayMeasureFdcProfileSetSummaryEvery(uint32_t everyNFrames)` [3047](sensorarrayMeasure.c#L3047) | 配置每隔多少帧统计或打印一次 profile 汇总。 / Set the summary interval in frames. | 独立接口 / Standalone entry |
| `bool sensorarrayMeasureFdcProfileSummaryEnabled(void)` [3052](sensorarrayMeasure.c#L3052) | 查询当前是否启用 FDC profile 汇总。 / Read the summary-profile enable flag. | 独立接口 / Standalone entry |
| `bool sensorarrayMeasureFdcProfileRowEnabled(void)` [3057](sensorarrayMeasure.c#L3057) | 查询当前是否启用逐行 profile。 / Read the per-row profile enable flag. | 独立接口 / Standalone entry |
| `bool sensorarrayMeasureFdcProfileDeviceEnabled(void)` [3062](sensorarrayMeasure.c#L3062) | 查询当前是否启用逐器件 profile。 / Read the per-device profile enable flag. | 独立接口 / Standalone entry |
| `uint32_t sensorarrayMeasureFdcProfileSummaryEvery(void)` [3067](sensorarrayMeasure.c#L3067) | 返回 profile 汇总当前使用的帧数间隔。 / Return the configured summary cadence, measured in frames. | 独立接口 / Standalone entry |
| `esp_err_t sensorarrayMeasureFdcSetDiscardFrames(uint8_t discardFrames)` [3072](sensorarrayMeasure.c#L3072) | 限制丢弃帧数在 0–8 之内，再保存设置供采集路径使用。 / Reject values above eight before changing the configured discard count. | 独立接口 / Standalone entry |
| `uint8_t sensorarrayMeasureFdcDiscardFrames(void)` [3081](sensorarrayMeasure.c#L3081) | 返回当前配置的 FDC 丢弃帧数。 / Return the configured number of FDC frames to discard. | 独立接口 / Standalone entry |

### sensorarrayMeasurementMode.c

| C 调用签名 / C signature | 行为及返回 / Behaviour and return | 调用位置 / Called by |
| --- | --- | --- |
| `static void sensorarrayMeasurementModeWriteBegin(sensorarrayMeasurementModeContext_t *context)` [5](sensorarrayMeasurementMode.c#L5) | 将快照版本推进到写入中，读者据此重试；调用者负责单写者语义。 / Advance the sequence to mark a mode snapshot update in progress. | `sensorarrayMeasurementModeEnterRecovery` / `sensorarrayMeasurementModeAccept` |
| `static void sensorarrayMeasurementModeWriteEnd(sensorarrayMeasurementModeContext_t *context)` [10](sensorarrayMeasurementMode.c#L10) | 完成 mode 快照写入并推进版本，供读者判断整次复制的一致性。 / Publish the completed mode snapshot by advancing its sequence again. | `sensorarrayMeasurementModeEnterRecovery` / `sensorarrayMeasurementModeAccept` |
| `bool sensorarrayMeasurementModeIsDataMode(sensorarrayMeasurementMode_t mode)` [15](sensorarrayMeasurementMode.c#L15) | 只接受三种可采集物理模式 CAP/VOLT/RES。 / Recognize the three data-producing physical modes. | `sensorarrayMixedRowPrepareRailAtBoundary` / `sensorarrayConfigIdentityFormat` |
| `uint8_t sensorarrayMeasurementCellCount(uint8_t rows)` [22](sensorarrayMeasurementMode.c#L22) | 对合法 1–8 行返回 rows×8 单元数，越界返回零。 / Compute the active cell count; return zero for an invalid row count. | `sensorarrayMeasurementSelfTestRun` / `sensorarrayTextProtocolSelfTest` |
| `sensorarrayMeasurementState_t sensorarrayMeasurementStateForMode(sensorarrayMeasurementMode_t mode)` [27](sensorarrayMeasurementMode.c#L27) | 将目标数据模式映射到状态机对应的活动状态。 / Select the active state corresponding to the target measurement mode. | `sensorarrayMeasurementModeCompleteTransition` |
| `void sensorarrayMeasurementModeInit(sensorarrayMeasurementModeContext_t *context)` [43](sensorarrayMeasurementMode.c#L43) | 清空模式上下文并从 SAFE、active/pending NONE 开始。 / Initialize the mode snapshot in SAFE with no active or pending mode. | `sensorarrayMeasurementSelfTestRun` / `sensorarrayInitRuntime` |
| `void sensorarrayMeasurementModeEnterRecovery(sensorarrayMeasurementModeContext_t *context, uint32_t errorCode)` [57](sensorarrayMeasurementMode.c#L57) | 保存旧模式并进入 RECOVERY，撤销待处理模式并登记首个错误。 / Preserve the old mode, enter RECOVERY, clear pending mode and retain the error. | `sensorarrayBeginAdsRestoreRecovery` / `sensorarrayRunAdsRestoreRecoveryAttempt` |
| `bool sensorarrayMeasurementModeAccept(sensorarrayMeasurementModeContext_t *context, sensorarrayMeasurementMode_t requestedMode, uint32_t requestId)` [74](sensorarrayMeasurementMode.c#L74) | 受理目标模式并记录 pending/request ID，等待采集任务实际应用。 / Accept the requested mode and retain the pending request ID until acquisition applies it. | `sensorarrayMeasurementSelfTestRun` / `sensorarrayApplyMeasurementMode` |
| `bool sensorarrayMeasurementModeBeginTransition(sensorarrayMeasurementModeContext_t *context)` [90](sensorarrayMeasurementMode.c#L90) | 核对已受理的请求并开始帧边界模式切换。 / Check the accepted request and begin the frame-boundary transition. | `sensorarrayMeasurementSelfTestRun` / `sensorarrayApplyMeasurementMode` |
| `bool sensorarrayMeasurementModeCompleteTransition(sensorarrayMeasurementModeContext_t *context, uint32_t appliedFrameSequence, uint64_t transitionDurationUs)` [104](sensorarrayMeasurementMode.c#L104) | 将已成功切换的模式登记为 active，记录 applied ID、边界与耗时。 / Commit a successful mode change as active and record the applied ID, boundary and elapsed time. | `sensorarrayMeasurementSelfTestRun` / `sensorarrayApplyMeasurementMode` |
| `void sensorarrayMeasurementModeFailTransition(sensorarrayMeasurementModeContext_t *context, uint32_t errorCode, uint64_t transitionDurationUs)` [133](sensorarrayMeasurementMode.c#L133) | 将切换失败与该请求 ID 记录为拒绝终态并保存错误。 / Record a failed transition and its request ID as a rejected terminal with the error. | `sensorarrayMeasurementSelfTestRun` / `sensorarrayApplyMeasurementMode` |
| `void sensorarrayMeasurementModeRejectRequest(sensorarrayMeasurementModeContext_t *context, uint32_t requestId, uint32_t errorCode)` [154](sensorarrayMeasurementMode.c#L154) | 保留被拒请求的终态 ID 和错误，同时维持已应用身份。 / Record the rejected request's terminal ID and error while preserving the applied ID. | `sensorarrayRejectHeldCommand` / `sensorarrayApplyPendingCommands` |
| `void sensorarrayMeasurementModeRecordRuntimeFault(sensorarrayMeasurementModeContext_t *context, uint32_t errorCode)` [165](sensorarrayMeasurementMode.c#L165) | 记录运行故障及帧序号，登记实际退化状态。 / Record a runtime fault and its frame sequence and publish the degraded state. | `sensorarrayApplyPendingCommands` / `sensorarrayEnterFrontendHold` |
| `bool sensorarrayMeasurementModeCopySnapshot(const sensorarrayMeasurementModeContext_t *context, sensorarrayMeasurementModeSnapshot_t *outSnapshot)` [179](sensorarrayMeasurementMode.c#L179) | 在版本稳定时复制一个完整的模式域快照。 / Return a complete mode-domain snapshot after checking version stability. | `sensorarrayMeasurementSelfTestRun` / `sensorarrayAdsFaultFillContext` |
| `void sensorarrayMeasurementRecoveryInit(sensorarrayMeasurementRecovery_t *recovery)` [200](sensorarrayMeasurementMode.c#L200) | 清空恢复状态，使新的有界恢复从无活动尝试开始。 / Reset recovery bookkeeping before accepting a recovery trigger. | `sensorarrayInitRuntime` / `sensorarrayEnterFrontendHold` |
| `bool sensorarrayMeasurementRecoveryStart(sensorarrayMeasurementRecovery_t *recovery, sensorarrayMeasurementMode_t resumeMode, uint32_t resumeRequestId, sensorarrayMeasurementRecoveryTrigger_t trigger, uint32_t error, uint32_t sequence, uint32_t triggerRequestId)` [209](sensorarrayMeasurementMode.c#L209) | 建立包含触发原因、目标模式和请求身份的一次有界恢复。 / Start bounded recovery with a trigger, resume mode and request identity. | `sensorarrayBeginAdsRestoreRecovery` |
| `bool sensorarrayMeasurementRecoveryIsActive(const sensorarrayMeasurementRecovery_t *recovery)` [239](sensorarrayMeasurementMode.c#L239) | 非空恢复对象且 active 标志已置位时返回 true。 / Report whether an admitted recovery is currently active. | `sensorarrayRunAdsTimingCommand` / `sensorarrayRunAdsRestoreRecoveryAttempt` |
| `bool sensorarrayMeasurementRecoveryIsTerminal(const sensorarrayMeasurementRecovery_t *recovery)` [245](sensorarrayMeasurementMode.c#L245) | 非空恢复对象且终态标志已置位时返回 true。 / Report whether recovery has reached a terminal outcome. | `sensorarrayRunAdsRestoreRecoveryAttempt` |
| `bool sensorarrayMeasurementRecoveryBeginAttempt(sensorarrayMeasurementRecovery_t *recovery)` [251](sensorarrayMeasurementMode.c#L251) | 检查状态及预算后登记新的恢复尝试。 / Admit a new recovery attempt and account for its budget. | `sensorarrayRunAdsRestoreRecoveryAttempt` |
| `void sensorarrayMeasurementRecoveryComplete(sensorarrayMeasurementRecovery_t *recovery, bool success, uint32_t error)` [263](sensorarrayMeasurementMode.c#L263) | 区分成功、可重试及终态失败并记录恢复和清理证据。 / Classify success, retry or terminal failure and retain recovery/cleanup evidence. | `sensorarrayRunAdsRestoreRecoveryAttempt` |
| `const char *sensorarrayMeasurementRecoveryTriggerName(sensorarrayMeasurementRecoveryTrigger_t trigger)` [288](sensorarrayMeasurementMode.c#L288) | 将电池恢复、ADS 恢复或无触发映射为 wire/日志标签。 / Distinguish battery restore, ADC restore and absent triggers in diagnostics. | 独立接口 / Standalone entry |
| `const char *sensorarrayMeasurementRecoveryOutcomeName(sensorarrayMeasurementRecoveryOutcome_t outcome)` [302](sensorarrayMeasurementMode.c#L302) | 将 started、attempt、resumed 或 failed 等恢复状态编码为诊断文本。 / Format progress and terminal recovery outcomes. | 独立接口 / Standalone entry |
| `sensorarrayMeasurementRecoveryTrigger_t sensorarrayMeasurementRecoveryTriggerForBattery(bool restoreFailed)` [320](sensorarrayMeasurementMode.c#L320) | 仅按电池事务的恢复失败证据选择矩阵恢复触发原因。 / Select the matrix recovery trigger based on battery-transaction restoration failure. | `sensorarrayRunBatteryBoundary` |
| `const char *sensorarrayMeasurementModeName(sensorarrayMeasurementMode_t mode)` [328](sensorarrayMeasurementMode.c#L328) | 将 CAP、VOLT、RES 等模式枚举格式化为协议名称。 / Return the wire label for the measurement mode. | `sensorarrayAdsMatrixCaptureFault` / `sensorarrayAdsMatrixLogCrcEvidence` |
| `const char *sensorarrayMeasurementStateName(sensorarrayMeasurementState_t state)` [343](sensorarrayMeasurementMode.c#L343) | 将 SAFE、TRANSITION、RECOVERY 等状态枚举格式化为主机标签。 / Return the protocol label for the current state-machine phase. | `sensorarrayConfigIdentityFormat` / `sensorarrayRuntimeQueryCommand` |
| `const char *sensorarrayMeasurementUnitName(sensorarrayMeasurementUnit_t unit)` [368](sensorarrayMeasurementMode.c#L368) | 根据测量单位返回 pF、V、ohm 或 none。 / Map the unit enum to the corresponding physical wire unit. | `sensorarrayRuntimeQueryCommand` / `sensorarrayTextProtocolBuildMeasurementFrame` |
| `const char *sensorarrayAdsReferenceSourceName(sensorarrayAdsReferenceSource_t source)` [383](sensorarrayMeasurementMode.c#L383) | 将参考源编码为 INTREF、AVDD_AVSS、EXTERNAL 或 NONE。 / Report the selected ADC reference by its protocol label. | `sensorarrayAdsFaultFillContext` / `sensorarrayRuntimeQueryCommand` |
| `const char *sensorarrayCellErrorName(sensorarrayCellError_t error)` [398](sensorarrayMeasurementMode.c#L398) | 将点错误枚举转为诊断原因，索引超范围时返回 unknown。 / Format the cell error reason and preserve an unknown out-of-range value. | `sensorarrayRuntimeQueryCommand` |

### sensorarrayMeasurementSelfTest.c

| C 调用签名 / C signature | 行为及返回 / Behaviour and return | 调用位置 / Called by |
| --- | --- | --- |
| `bool sensorarrayMeasurementSelfTestRun(sensorarrayMeasurementSelfTestResult_t *outResult)` [20](sensorarrayMeasurementSelfTest.c#L20) | 运行板上测量逻辑自检，向 outResult 提供检查数及首次失败位置。 / Exercise measurement logic and return the check count and first failed location. | `sensorarrayInitRuntime` |

### sensorarrayRouteController.c

| C 调用签名 / C signature | 行为及返回 / Behaviour and return | 调用位置 / Called by |
| --- | --- | --- |
| `static bool sensorarrayRouteFdcSdGpioValid(void)` [20](sensorarrayRouteController.c#L20) | 确认配置的 FDC shutdown 引脚在 ESP32-S3 的有效 GPIO 编号范围内。 / Check that the selected shutdown pin is within the S3 GPIO range. | `sensorarrayRouteControllerPrepareFdcSdGpio` / `sensorarrayRouteControllerInit` |
| `esp_err_t sensorarrayRouteControllerPrepareFdcSdGpio(void)` [26](sensorarrayRouteController.c#L26) | 初始化板级 FDC SD 引脚并核对其可操作性。 / Initialise the FDC shutdown pin and verify its availability. `ESP_ERR_NOT_SUPPORTED` | `sensorarrayRouteControllerInit` / `sensorarrayInitFrontends` |
| `static void sensorarrayRouteWriteBegin(sensorarrayRouteController_t *controller)` [62](sensorarrayRouteController.c#L62) | 在更新路由快照之前推进写入版本，供并发查询检测变化。 / Mark an in-progress route snapshot update for concurrent readers. | `sensorarrayRouteRecordError` / `sensorarrayRouteSleepFdcFrontends` |
| `static void sensorarrayRouteWriteEnd(sensorarrayRouteController_t *controller)` [67](sensorarrayRouteController.c#L67) | 在所有路由字段更新后推进发布版本。 / Complete publication of the route snapshot after storing its fields. | `sensorarrayRouteRecordError` / `sensorarrayRouteSleepFdcFrontends` |
| `static void sensorarrayRouteRecordError(sensorarrayRouteController_t *controller, esp_err_t err)` [72](sensorarrayRouteController.c#L72) | 在路由快照中保存本次操作返回的错误。 / Publish the route operation's error in the route snapshot. | `sensorarrayRouteControllerEnterSafe` / `sensorarrayRouteControllerApplyMode` |
| `static esp_err_t sensorarrayRouteSetFdcSleep(sensorarrayFdcDeviceState_t *fdc, bool sleep, bool *outVerified)` [83](sensorarrayRouteController.c#L83) | 请求单个 FDC 进入或退出 sleep 并核对返回。 / Request sleep/wake for one FDC and check the resulting state. | `sensorarrayRouteSleepFdcFrontends` / `sensorarrayRouteWakeFdcFrontends` |
| `static esp_err_t sensorarrayRouteSleepFdcFrontends(sensorarrayRouteController_t *controller, const char **outStage)` [102](sensorarrayRouteController.c#L102) | 协调两颗 FDC 休眠与可用性，保存已知/读回状态。 / Coordinate FDC sleep across devices, retaining known/readback evidence. `ESP_ERR_INVALID_ARG` `ESP_ERR_INVALID_STATE` | `sensorarrayRouteStopFrontends` |
| `static esp_err_t sensorarrayRouteStopFrontends(sensorarrayRouteController_t *controller, const char **outStage)` [171](sensorarrayRouteController.c#L171) | 按目标模式停止相关转换并按可隔离能力处理闲置前端。 / Stop affected conversions for the target route and account for available frontend isolation. `ESP_ERR_INVALID_ARG` `ESP_ERR_INVALID_STATE` | `sensorarrayRouteControllerEnterSafe` / `sensorarrayRouteControllerApplyMode` |
| `static esp_err_t sensorarrayRouteWakeFdcFrontends(sensorarrayRouteController_t *controller)` [216](sensorarrayRouteController.c#L216) | 对两颗 FDC 退出 sleep 并分别核验，保留首错。 / Wake both FDCs and retain individual verification and the first error. | `sensorarrayRouteControllerApplyMode` |
| `static esp_err_t sensorarrayRouteDisableMatrixReference(sensorarrayRouteController_t *controller)` [247](sensorarrayRouteController.c#L247) | 停用矩阵内部激励参考并在后续钳位动作前核对其状态。 / Disable the internal excitation reference and check its state before later clamping operations. `ESP_ERR_INVALID_ARG` `ESP_ERR_INVALID_STATE` | `sensorarrayRouteApplyControlProfile` / `sensorarrayRouteControllerEnterSafe` |
| `static bool sensorarrayRouteGpioMatches(const sensorarrayBoardRouteProfile_t *profile, const tmuxSwitchControlState_t *control)` [277](sensorarrayRouteController.c#L277) | 比较路由 profile 所需的选通、SW 与实际 GPIO 读回。 / Compare requested switch and selector state against GPIO readback. | `sensorarrayRouteCaptureAndVerify` / `sensorarrayRouteControllerSelectRow` |
| `static bool sensorarrayRouteAdsMatches(const sensorarrayBoardRouteProfile_t *profile, uint8_t power, uint8_t mode2, uint8_t refmux, uint8_t expectedGain)` [307](sensorarrayRouteController.c#L307) | 比较当前路径所需的 ADS 电源、参考选择及运行状态。 / Check ADS power, reference selection and run state required by the route. | `sensorarrayRouteCaptureAndVerify` |
| `static esp_err_t sensorarrayRouteApplyControlProfile(sensorarrayRouteController_t *controller, const sensorarrayBoardRouteProfile_t *profile)` [325](sensorarrayRouteController.c#L325) | 在规定次序写入板级 SW/SELA/SELB，并把控制错误回传给路由事务。 / Apply SW/SELA/SELB in routing order and propagate control errors to the transaction. | `sensorarrayRouteControllerApplyMode` / `sensorarrayRouteControllerEnterSafeRailMonitor` |
| `static esp_err_t sensorarrayRouteConfigureAds(sensorarrayRouteController_t *controller, const sensorarrayBoardRouteProfile_t *profile, const sensorarrayAdsRailSplit_t *rail)` [353](sensorarrayRouteController.c#L353) | 按目标模式设置 ADC1 的参考、电源和采集配置。 / Configure ADC1 reference, power and acquisition for the target mode. `ESP_ERR_INVALID_STATE` `ESP_ERR_NOT_SUPPORTED` | `sensorarrayRouteControllerApplyMode` / `sensorarrayRouteControllerEnterSafeRailMonitor` |
| `static esp_err_t sensorarrayRouteCaptureAndVerify(sensorarrayRouteController_t *controller, const sensorarrayBoardRouteProfile_t *profile, const sensorarrayAdsRailSplit_t *rail, bool verifyAds)` [409](sensorarrayRouteController.c#L409) | 取得控制脚及所需 ADS 回读，比较目标配置，保存本次可核验的路由证据。 / Capture control pins and required ADS readback, compare to the target and save the verifiable route evidence. | `sensorarrayRouteControllerEnterSafe` / `sensorarrayRouteControllerApplyMode` |
| `esp_err_t sensorarrayRouteControllerInit(sensorarrayRouteController_t *controller, sensorarrayState_t *state)` [470](sensorarrayRouteController.c#L470) | 将路由控制器绑定到板级状态，初始化控制快照和默认建立时间。 / Bind the controller to board state and initialize routing evidence and settling settings. | `sensorarrayInitFrontends` |
| `esp_err_t sensorarrayRouteControllerEnterFrontendWait(sensorarrayRouteController_t *controller, sensorarrayState_t *state)` [489](sensorarrayRouteController.c#L489) | 暂停矩阵并建立用于低速身份重探的前端等待状态，保存已知控制证据。 / Pause matrix work and establish the frontend wait state for low-rate reprobe, preserving known controls. `ESP_ERR_INVALID_ARG` `ESP_ERR_INVALID_STATE` | `sensorarrayEnterFrontendHold` |
| `esp_err_t sensorarrayRouteControllerEnterSafe(sensorarrayRouteController_t *controller, const char *reason)` [551](sensorarrayRouteController.c#L551) | 按依赖停止前端与激励，执行安全控制动作和回读，并区分首错、清理错误及是否已核验。 / Stop frontends and excitation in dependency order, apply and read back safe controls, and distinguish first error, cleanup and verification. `ESP_ERR_INVALID_ARG` `ESP_ERR_INVALID_STATE` | `sensorarrayAdsMatrixEngineReadFrame` / `sensorarrayMixedRowPrepareRailAtBoundary` |
| `esp_err_t sensorarrayRouteControllerApplyMode(sensorarrayRouteController_t *controller, sensorarrayMeasurementMode_t mode, const sensorarrayAdsRailSplit_t *rail, uint64_t *outTransitionDurationUs)` [617](sensorarrayRouteController.c#L617) | 在采集所有者上下文应用目标模式并完成控制/寄存器核验与建立等待。 / Apply the target route on the acquisition owner, verify controls/registers and allow settling. `ESP_ERR_INVALID_ARG` `ESP_ERR_INVALID_STATE` `ESP_ERR_NOT_SUPPORTED` | `sensorarrayMixedRowPrepareRailAtBoundary` / `sensorarrayMixedRowEngineReadFrame` |
| `esp_err_t sensorarrayRouteControllerEnterSafeRailMonitor(sensorarrayRouteController_t *controller, uint64_t *outTransitionDurationUs)` [716](sensorarrayRouteController.c#L716) | 仅在板级提供隔离路径时应用内部 rail 监测路由并核验参考状态。 / Admit the internal rail-monitor route only when the board offers isolation and verify its reference state. `ESP_ERR_INVALID_ARG` `ESP_ERR_NOT_SUPPORTED` | `sensorarrayMixedRowPrepareRailAtBoundary` / `sensorarrayRefreshVoltageRailIfNeeded` |
| `esp_err_t sensorarrayRouteControllerSelectRow(sensorarrayRouteController_t *controller, uint8_t row)` [759](sensorarrayRouteController.c#L759) | 按模式设置行地址、建立时间和必要路由检查，失败时保留故障行证据。 / Set the row address, account for mode-specific settling, check the route and retain failing-row evidence. `ESP_ERR_INVALID_ARG` `ESP_ERR_INVALID_STATE` | `sensorarrayAdsMatrixEngineReadFrame` / `sensorarrayMixedRowPrepareRailAtBoundary` |
| `bool sensorarrayRouteControllerCopySnapshot(const sensorarrayRouteController_t *controller, sensorarrayRouteSnapshot_t *outSnapshot)` [935](sensorarrayRouteController.c#L935) | 复制路由状态、请求/应用/已知/核验标志和最近错误，供控制侧查询。 / Copy route state, requested/applied/known/verified flags and recent errors for control queries. | `sensorarrayAdsMatrixRefreshRegisterShadow` / `sensorarrayAdsMatrixEngineReadFrame` |
| `void sensorarrayRouteControllerUpdateRailSnapshot(sensorarrayRouteController_t *controller, const sensorarrayAdsRailSplit_t *rail)` [955](sensorarrayRouteController.c#L955) | 把已取得的 rail 各分量复制到路由快照并发布版本。 / Publish the supplied rail observations into the route snapshot. | `sensorarrayAdsMatrixEngineReadFrame` |
| `bool sensorarrayRouteControllerSetRowSettleUs(sensorarrayRouteController_t *controller, uint32_t settleUs)` [969](sensorarrayRouteController.c#L969) | 接受 0–10000 µs 的行建立时间并更新控制器，非法参数返回 false。 / Set the per-row settle interval, rejecting values over 10000 µs. | `sensorarrayApplyPendingCommands` |
| `uint32_t sensorarrayRouteControllerGetRowSettleUs(const sensorarrayRouteController_t *controller)` [979](sensorarrayRouteController.c#L979) | 返回当前行建立时间（µs），控制器为空时返回零。 / Return the row settle interval in microseconds or zero for a null controller. | `sensorarrayRuntimeQueryCommand` |
| `esp_err_t sensorarrayRouteControllerForceFdcShutdown(sensorarrayRouteController_t *controller, bool *outVerified)` [985](sensorarrayRouteController.c#L985) | 通过板级 SD 控制强制 FDC 关断，核对电平并失效受影响缓存。 / Assert FDC shutdown through SD, check the pin and invalidate affected state. `ESP_ERR_INVALID_ARG` `ESP_ERR_NOT_SUPPORTED` `ESP_ERR_INVALID_STATE` | `sensorarrayApplyPendingCommands` |

### sensorarrayRowModeProfile.c

| C 调用签名 / C signature | 行为及返回 / Behaviour and return | 调用位置 / Called by |
| --- | --- | --- |
| `static void sensorarrayRowModeProfileWriteBegin(sensorarrayRowModeProfile_t *profile)` [5](sensorarrayRowModeProfile.c#L5) | 将 profile 快照版本标为更新中，读者在改变时重新复制。 / Mark the start of a single-writer row-profile snapshot update. | `sensorarrayRowModeProfileAccept` / `sensorarrayRowModeProfileCompleteTransition` |
| `static void sensorarrayRowModeProfileWriteEnd(sensorarrayRowModeProfile_t *profile)` [10](sensorarrayRowModeProfile.c#L10) | 更新完全部 profile 字段后发布稳定版本。 / Make the completed row-profile snapshot visible to readers. | `sensorarrayRowModeProfileAccept` / `sensorarrayRowModeProfileCompleteTransition` |
| `static sensorarrayMeasurementMode_t sensorarrayRowModeFromChar(char value)` [15](sensorarrayRowModeProfile.c#L15) | 将 C、V、R 字符解析为单行物理测量模式。 / Decode one profile character into a physical measurement mode. | `sensorarrayRowModeProfileParse` |
| `static char sensorarrayRowModeToChar(sensorarrayMeasurementMode_t mode)` [25](sensorarrayRowModeProfile.c#L25) | 将单行测量模式编码回 profile 使用的字符。 / Encode a row's measurement mode into the profile alphabet. | `sensorarrayRowModeProfileFormat` |
| `void sensorarrayRowModeProfileInit(sensorarrayRowModeProfile_t *profile, sensorarrayMeasurementMode_t defaultMode)` [35](sensorarrayRowModeProfile.c#L35) | 把八个物理行初始化为默认数据模式，并设置初始 profile generation。 / Initialize all physical rows to the supplied mode and set the initial generation. | `sensorarrayInitRuntime` |
| `bool sensorarrayRowModeProfileParse(const char *text, size_t length, sensorarrayMeasurementMode_t outModes[SENSORARRAY_ROW_MODE_PROFILE_ROWS])` [51](sensorarrayRowModeProfile.c#L51) | 将固定长度的 C/V/R 字符串解析为八行模式枚举，非法字符或长度返回错误。 / Parse the fixed C/V/R string into eight row modes, rejecting invalid length or letters. | `sensorarrayCommandMailboxPostRowModes` / `sensorarrayApplyPendingCommands` |
| `bool sensorarrayRowModeProfileFormat(const sensorarrayMeasurementMode_t modes[SENSORARRAY_ROW_MODE_PROFILE_ROWS], char outText[SENSORARRAY_ROW_MODE_PROFILE_TEXT_LENGTH + 1u])` [68](sensorarrayRowModeProfile.c#L68) | 将八行模式枚举编码为带结尾 NUL 的八字符行 profile。 / Encode eight row modes into a terminated eight-character profile. | `sensorarrayConfigIdentityCapture` / `sensorarrayRuntimeQueryCommand` |
| `bool sensorarrayRowModeProfileAccept(sensorarrayRowModeProfile_t *profile, const sensorarrayMeasurementMode_t modes[SENSORARRAY_ROW_MODE_PROFILE_ROWS], uint32_t requestId)` [86](sensorarrayRowModeProfile.c#L86) | 验证并缓存八行的 C/V/R 设置及请求 ID，等待采集边界。 / Validate and retain all eight C/V/R selections and the request ID for boundary application. | `sensorarrayApplyPendingCommands` |
| `bool sensorarrayRowModeProfileBeginTransition(sensorarrayRowModeProfile_t *profile)` [111](sensorarrayRowModeProfile.c#L111) | 检查 pending 配置并将其推进到实际切换阶段。 / Check the pending row profile and advance it to the transition phase. | `sensorarrayRowModeProfileCompleteTransition` |
| `bool sensorarrayRowModeProfileCompleteTransition(sensorarrayRowModeProfile_t *profile, uint32_t appliedFrameSequence, uint64_t transitionDurationUs)` [117](sensorarrayRowModeProfile.c#L117) | 提交行模式、profile generation 和已应用 ID，保存帧边界。 / Commit row modes, profile generation and applied request ID with the frame boundary. | `sensorarrayApplyPendingCommands` / `sensorarrayRunOneFrame` |
| `void sensorarrayRowModeProfileRejectRequest(sensorarrayRowModeProfile_t *profile, uint32_t requestId, uint32_t errorCode)` [143](sensorarrayRowModeProfile.c#L143) | 记录被拒绝请求的终态身份与错误。 / Retain the rejected request identity and error as the terminal outcome. | `sensorarrayRejectHeldCommand` / `sensorarrayApplyPendingCommands` |
| `void sensorarrayRowModeProfileFailTransition(sensorarrayRowModeProfile_t *profile, uint32_t errorCode, uint64_t transitionDurationUs)` [154](sensorarrayRowModeProfile.c#L154) | 记录本次行模式应用失败并使终态保留正确的请求 ID。 / Record row-profile application failure under the corresponding request ID. | `sensorarrayApplyPendingCommands` / `sensorarrayEnterFrontendHold` |
| `bool sensorarrayRowModeProfileCopy(const sensorarrayRowModeProfile_t *profile, sensorarrayRowModeProfile_t *outSnapshot)` [173](sensorarrayRowModeProfile.c#L173) | 在版本稳定的情况下返回完整八行 profile 和域状态。 / Copy the eight-row profile and domain state from a stable snapshot version. | `sensorarrayAdsFaultFillContext` / `sensorarrayRefreshVoltageRailIfNeeded` |
| `bool sensorarrayRowModeProfileIsHomogeneous(const sensorarrayMeasurementMode_t modes[SENSORARRAY_ROW_MODE_PROFILE_ROWS])` [193](sensorarrayRowModeProfile.c#L193) | 检查全部八个行模式是否相同。 / Check whether all eight row modes are identical. | `sensorarrayRowModeProfileHomogeneousMode` / `sensorarrayRefreshVoltageRailIfNeeded` |
| `sensorarrayMeasurementMode_t sensorarrayRowModeProfileHomogeneousMode(const sensorarrayMeasurementMode_t modes[SENSORARRAY_ROW_MODE_PROFILE_ROWS])` [207](sensorarrayRowModeProfile.c#L207) | 在行模式相同时返回该单一模式，否则返回 NONE。 / Return the common mode when the profile is homogeneous, otherwise NONE. | 独立接口 / Standalone entry |
| `uint8_t sensorarrayRowModeProfileMaskForMode(const sensorarrayMeasurementMode_t modes[SENSORARRAY_ROW_MODE_PROFILE_ROWS], sensorarrayMeasurementMode_t mode)` [214](sensorarrayRowModeProfile.c#L214) | 逐行比较并为指定模式产生八位行掩码。 / Build an eight-bit mask of rows assigned the requested mode. | 独立接口 / Standalone entry |
| `const char *sensorarrayRowModeProfileStateName(sensorarrayRowModeProfileState_t state)` [230](sensorarrayRowModeProfile.c#L230) | 将 profile 的稳定、待处理或过渡等状态转换为查询标签。 / Return the wire label for the row-profile state. | `sensorarrayRuntimeQueryCommand` |

### sensorarrayScanPlan.c

| C 调用签名 / C signature | 行为及返回 / Behaviour and return | 调用位置 / Called by |
| --- | --- | --- |
| `static void sensorarrayScanPlanBuildRows(sensorarrayScanPlan_t *plan, sensorarrayCellOpKind_t defaultOp)` [8](sensorarrayScanPlan.c#L8) | 按活动行数建立每行八个 S/D 操作及指定的默认采集类型。 / Build eight S/D operations per active row using the selected default operation. | `sensorarrayScanPlanBuildDefaultFdcMatrix` / `sensorarrayScanPlanBuildAdsMatrix` |
| `void sensorarrayScanPlanBuildDefaultFdcMatrix(sensorarrayScanPlan_t *plan)` [52](sensorarrayScanPlan.c#L52) | 为活动行构造统一 CAP 的 FDC 计划。 / Build a uniform FDC CAP plan for the active rows. | `sensorarrayScanPlanBuildRowProfile` / `sensorarrayBuildDefaultScanPlan` |
| `void sensorarrayScanPlanBuildAdsMatrix(sensorarrayScanPlan_t *plan, sensorarrayMeasurementMode_t mode)` [57](sensorarrayScanPlan.c#L57) | 为活动行构造 VOLT 或 RES 的 ADC 矩阵计划。 / Build a uniform VOLT or RES ADC plan for the active rows. | `sensorarrayRunOneFrame` |
| `void sensorarrayScanPlanBuildMixedExample(sensorarrayScanPlan_t *plan)` [65](sensorarrayScanPlan.c#L65) | 构建内置 RVVCCVVR 八行示例计划。 / Build the built-in RVVCCVVR eight-row example plan. | 独立接口 / Standalone entry |
| `void sensorarrayScanPlanBuildRowProfile(sensorarrayScanPlan_t *plan, const sensorarrayMeasurementMode_t rowModes[8], uint32_t profileGeneration, uint32_t profileRequestId)` [80](sensorarrayScanPlan.c#L80) | 将逐行 C/V/R 选择展开成测点操作，并保留 profile 世代与请求 ID。 / Expand row selections into cell operations and retain profile generation and request ID. | `sensorarrayScanPlanBuildMixedExample` / `sensorarrayRunOneFrame` |

## 参数索引 / Parameter index

相同名称按模块共享解释。完整类型、数组维度与输入输出方向以对应函数签名和调用点为准。指针参数所需存储由调用者提供，异步作业另受上面的所有权约束。

Entries share parameter descriptions within this module. Refer to the signature and its caller for the precise type, array shape and direction. The caller supplies pointer storage, subject to the ownership rules above for asynchronous work.

| 参数 / Parameter | 含义 / Meaning |
| --- | --- |
| `a` | 第一份待比较的不可变配置身份。 / First immutable configuration identity to compare. |
| `acquiredMask`, `expectedMask` | 采集新鲜标志、一致性或测点位图。掩码按物理测点下标定位。 / Freshness flag, coherence flag or per-cell bitset. A cell mask is indexed by physical position. |
| `active` | 将本器件的示波器读窗口置为活动或结束。 / Assert or clear the scope marker for the selected device. |
| `after` | 本次读操作后取得的 I²C 统计快照。 / I²C counter snapshot taken after this operation. |
| `applied` | 已写入行配置的缓存，用于与所需寄存器逐项核对。 / Cache of the previously applied row-register profile. |
| `appliedFrameSequence`, `frameBoundary` | 配置生效时的帧边界序号。 / Capture boundary sequence associated with the applied configuration. |
| `arg`, `context` | 回调或任务创建时绑定的上下文；在该调用的有效期内保留其存储。 / Context bound at callback registration or task creation. Keep its backing object alive for the operation. |
| `b` | 另一份待比较的配置身份，须核对会话与配置域。 / Second identity in the session/domain comparison. |
| `base` | 某通道寄存器组的基础地址，用通道编号推导目标。 / Base register address for the per-channel register group. |
| `before` | 本次读操作前取得的 I²C 统计快照。 / I²C counters captured immediately before the operation. |
| `bootId`, `sessionId` | 会话、启动身份或启动计数，查询与终态依赖所属启动域。 / Session identifier, boot identifier or boot count scoping queries and terminal outcomes. |
| `capEndUs`, `capStartUs`, `resEndUs`, `resStartUs`, `voltEndUs`, `voltStartUs` | 对应帧或各模式子组的起止时刻，单位 µs。 / Frame or mode-group start/end timestamps in µs. |
| `ch`, `channel` | 通道。 / Channel. |
| `config`, `policy` | 该入口声明的配置或策略结构，含所选阈值、时序或回调。 / Configuration or policy of the declared type holding the selected thresholds, timing or callbacks. |
| `configEnabled` | 配置启用。 / Config enabled. |
| `configuredCore` | 已配置核心。 / Configured core. |
| `control` | TMUX 控制证据，用于比较请求路由与 GPIO 数字观察。 / Multiplexer state for comparing requested settings and digital readback. |
| `controller` | 持有目标路由、实测控制回读与所有权状态的控制器实例。 / Controller instance holding the target route, control readback and ownership state. |
| `ctrl` | 保存或接收 TMUX 命令与引脚观察值的状态对象。 / Switch command and digital-observation state. |
| `ctx` | 所声明类型的应用、总线或调度上下文，供调用链共享。 / Application, bus or scheduler context of the declared type, shared with the call chain. |
| `decoded` | 已从 FDC STATUS 解出的就绪与未读通道证据。 / Readiness and unread-channel evidence decoded from FDC STATUS. |
| `defaultCore` | 未提供可用核心设置时所用的默认核心号。 / Fallback core selection for this worker. |
| `defaultMode`, `mode`, `requestedMode`, `resumeMode` | 对应调用域的模式枚举，保留目标、当前帧或恢复上下文的含义。 / Mode enum in the declared domain, selecting the target, frame or recovery mode as indicated. |
| `defaultOp` | 生成行计划时采用的默认测量操作。 / Default operation for building a row scan plan. |
| `deglitchCode` | 去毛刺代码。 / Deglitch code. |
| `delayMs` | 延迟，单位 ms。 / Delay in ms. |
| `delayUs` | 延迟，单位 µs。 / Delay in µs. |
| `devId`, `fdc`, `fdcState` | 对应驱动句柄、FDC 主/次设备 ID 或 ADS/FDC 状态；按签名类型选取当前板上同一实例。 / Corresponding driver handle, primary/secondary FDC ID or ADS/FDC state. Pass the board instance matching the declared type. |
| `discardCount` | 丢弃数量。 / Discard count. |
| `discardFrames` | 丢弃帧数。 / Discard frames. |
| `dLine`, `sColumn` | 板级 S 列或 D 线编号，合法位置为 1..8。 / Physical S column or D line, numbered 1..8. |
| `dSource` | 该行测量要使用的 D 线前端来源枚举。 / D-line frontend source selected for the route. |
| `enabled`, `success` | 表示证据已知、数据有效、功能启用或操作成功的布尔标志，按参数名独立解释。 / Boolean representing known evidence, validity, enablement or success as named. |
| `epochId` | 本行 FDC worker 的工作轮次，只有同一槽位及 epoch 的结果可消费。 / FDC worker's row epoch. Only results matching the slot and epoch are eligible. |
| `err`, `error`, `errorCode`, `firstErr` | 操作、首个根因或独立清理结果的错误码。保留调用层提供的具体错误。 / Error for the operation, first cause or separate cleanup, preserving the code from the reporting layer. |
| `estimatedRoundUs` | 估算轮次，单位 µs。 / Estimated round in µs. |
| `everyNFrames` | 周期负帧数。 / Every n frames. |
| `expected`, `observed`, `selaRoute` | 预期/观测的路由、寄存器或状态值，按该签名中声明的类型比较。 / Requested/observed route, register or state values, compared in the types shown by the signature. |
| `expectedDeglitch` | 预期去毛刺。 / Expected deglitch. |
| `expectedGain` | ADS PGA 增益或预期增益；bypass 和数值增益分别编码。 / ADS PGA gain or expected gain. Bypass has its own encoding. |
| `frame`, `target` | 按完整签名的类型传入目标帧、子帧或特定操作的目标。帧对象由调用者分配。 / Destination frame, child frame or operation-specific target according to the declared type. The caller owns frame storage. |
| `gpio` | 用于输出时序标记的 GPIO 编号。 / Pin used for timing instrumentation. |
| `identity` | 帧的不可变配置身份或同一版本的完整权威快照。 / Immutable frame configuration identity or the authority snapshot for that revision. |
| `intrefPolicy` | INTREF策略。 / INTREF policy. |
| `label` | 关联路由写入和 GPIO 回读的诊断标签。 / Diagnostic label associating a route write with its readback. |
| `length` | 输入、载荷、底层存储或读写传输的字节数。 / Byte length of input, payload, backing storage or transfer. |
| `level` | 由板级 SW 极性解释的引脚电平。 / Pin level interpreted using configured switch polarity. |
| `matrixIndex` | 行优先数组中的线性测点下标。 / Linear cell index in row-major storage. |
| `mode2`, `power`, `refmux` | ADS 对应的原始寄存器字节，比较由该事务的读回掩码限定。 / Raw ADS register byte. Readback comparisons use the mask for that transaction. |
| `modes`, `outModes`, `profile`, `rowModes` | 配置档或八个物理行的模式列表。outModes 接收解析结果。 / Configuration profile or mode selection for the eight physical rows. outModes receives parsed values. |
| `outcome` | 终态。 / Outcome. |
| `outCore` | 接收核心。 / Receives core. |
| `outDIndex` | 接收结果。在该实现中指向 S/D 位置的索引，按调用点的板级坐标/数组约定传入。 / Output. S/D position index in the board coordinate or array convention used by the call site. |
| `outDLine`, `outSColumn` | 接收结果。板级 S 列或 D 线编号，合法位置为 1..8。 / Output. Physical S column or D line, numbered 1..8. |
| `outFrame`, `outTarget` | 接收结果。按完整签名的类型传入目标帧、子帧或特定操作的目标。帧对象由调用者分配。 / Output. Destination frame, child frame or operation-specific target according to the declared type. The caller owns frame storage. |
| `outIdentity` | 接收结果。帧的不可变配置身份或同一版本的完整权威快照。 / Output. Immutable frame configuration identity or the authority snapshot for that revision. |
| `outMap` | 接收指定物理 D 线的 FDC/通道映射。 / Destination for the FDC and channel mapping of the requested D line. |
| `outPinned` | 接收绑核。 / Receives pinned. |
| `outReason`, `reason` | 操作或拒绝原因字符串；outReason 用于返回原因。 / Operation/rejection reason string. outReason receives the reason. |
| `outResult`, `outSnapshot`, `result`, `snapshot` | 函数签名注明的结果、快照或统计结构，调用者分配复制目标。 / Result, snapshot or statistics object of the declared type, with caller-owned output storage. |
| `outStage` | 接收阶段。 / Receives stage. |
| `outStatus` | 接收状态。 / Receives status. |
| `outText` | 接收结果。按该函数声明传入的文本、命令、操作码或数据字节，显式 length 限定可读范围。 / Output. Text, command, opcode or data bytes as declared. Where supplied, length bounds the readable range. |
| `outTransitionDurationUs` | 接收切换耗时，单位 µs。 / Receives Transition duration in µs. |
| `outVerified` | 接收已核验。 / Receives verified. |
| `path` | 表示当前测量分支的路由路径枚举。 / Path enum identifying the measurement branch. |
| `plan` | 本帧的已应用行、模式分组和活动掩码计划。 / Applied row selection, mode grouping and activity masks for this capture. |
| `primaryBusEnabled` | 主级总线启用。 / Primary bus enabled. |
| `primaryReady` | 主级就绪。 / Primary ready. |
| `profileGeneration` | 配置档世代。 / Profile generation. |
| `profileRequestId`, `requestId`, `resumeRequestId`, `triggerRequestId` | 对应配置、恢复触发或 profile 域内的请求标识，由接收方与提交时身份配对。 / Request identifier for the indicated configuration, recovery trigger or row profile, paired with the identity at admission. |
| `rail` | 有效性、来源和电压分量等 rail 快照。 / Supply-rail snapshot with voltage, validity and provenance. |
| `ready` | FDC ready 观察，按函数签名读取或填充。 / Per-device readiness evidence, consumed or populated as declared. |
| `recovery` | 包含触发源、尝试预算与终态的持久恢复上下文。 / Recovery state holding trigger, attempt budget and outcome. |
| `requestedRows` | 生效或申请的活动行数，范围为 1..8。 / Applied or requested number of active rows, 1..8. |
| `requestRoute` | 请求路由。 / Request route. |
| `requiredMask` | 必需掩码。 / Required mask. |
| `response` | 调用者提供的回复、暂存或结果缓冲区，使用随同的容量/类型。 / Caller-provided reply, scratch or result storage with the accompanying capacity/type. |
| `responseSize` | 字符输出缓冲区总字节数，含结尾 NUL 的空间。 / Total capacity of the text buffer in bytes, including space for its final NUL. |
| `restoreFailed` | 恢复失败。 / Restore failed. |
| `row` | 行或列的坐标。通用 matrixEngineRegion_t 的原点为零；以板级 S/D 位置操作的接口从 1 编号。 / Row or column coordinate. Generic matrixEngineRegion_t origins are zero-based. Board S/D coordinates are one-based. |
| `rowMask` | 以 bit 0 对应 S1 的八位物理行选择掩码。 / Eight-bit physical row mask with bit zero selecting S1. |
| `rows` | 行数值或行配置快照，由该函数的参数类型决定。 / Active count or row-configuration snapshot as specified by its declared type. |
| `safeDefaultDeglitch` | 安全默认去毛刺。 / Safe default deglitch. |
| `sameBus` | 指两颗 FDC 是否共用同一 I²C 总线，影响是否能并行。 / Whether the two FDCs share an I²C bus, constraining parallelism. |
| `secondaryBusEnabled` | 次级总线启用。 / Secondary bus enabled. |
| `secondaryReady` | 次级就绪。 / Secondary ready. |
| `selBLevel` | 此路由要求的 TMUX1134 SELB 数字电平。 / TMUX1134 SELB level requested by the route. |
| `sequence` | 对应采集帧的序号，必须与提供的帧、事件或缓存上下文属于同次启动。 / Capture sequence belonging to the same boot as the supplied frame, event or cache context. |
| `settleDelayMs` | 建立延迟，单位 ms。 / Settle delay in ms. |
| `settleDelayUs` | 建立延迟，单位 µs。 / Settle delay in µs. |
| `settleUs` | 建立，单位 µs。 / Settle in µs. |
| `sleep` | 为真时使 FDC 进入 sleep，为假时请求唤醒。 / Select FDC sleep entry or wake-up. |
| `source` | 选用的 ADS 参考源枚举。 / ADC reference-source enum to interpret. |
| `stage` | 标识路由/参考操作的位置，供错误诊断。 / Diagnostic stage identifying the route or reference operation. |
| `state` | 所声明类型的状态对象或枚举。传入前端状态指针时，其生命周期覆盖总线或采集操作。 / State object or enum of the declared type. A frontend-state pointer must outlive the bus or capture operation. |
| `status` | FDC STATUS 的观测或调用者提供的日志标签，见参数类型。 / Observed FDC status or a textual log tag, depending on the signature. |
| `swSource` | TMUX1108 选择的 GND/REF 逻辑源。 / Logical GND or REF source for the row multiplexer. |
| `text` | 按该函数声明传入的文本、命令、操作码或数据字节，显式 length 限定可读范围。 / Text, command, opcode or data bytes as declared. Where supplied, length bounds the readable range. |
| `timeoutMs` | 有限等待预算或硬超时标志，时间单位见参数后缀。 / Bounded waiting budget or hard-timeout flag. Time units follow the suffix. |
| `timing` | 接收该器件 I²C 计数差值的本次时序记录。 / Timing record that receives the device's I²C counter deltas. |
| `transitionDurationUs` | 切换耗时，单位 µs。 / Transition duration in µs. |
| `trigger` | 提交 ADS 恢复的事件来源，例如电池事务未能恢复。 / Event that requested ADC recovery, such as a failed battery restore. |
| `unit` | 测量量纲枚举，与帧内小数位约定配合使用。 / Measurement-unit enum used with the frame's decimal scale. |
| `value` | 待读取的单个行模式字符，按 profile 字母表解释。 / Row-mode character to interpret using the profile alphabet. |
| `vbiasPolicy` | 目标路由所需的 ADS VBIAS 状态策略。 / ADC bias policy to apply for the requested route. |
| `verifyAds` | 核验ADS。 / Verify ADS. |
| `workerName` | 工作任务名称。 / Worker name. |

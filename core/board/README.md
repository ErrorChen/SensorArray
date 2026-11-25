# 板级连线与前端探测 / Board routing and frontend discovery

这里将逻辑 S/D 坐标换算为 TMUX 选择、FDC 地址/通道和 ADS 输入。启动时 ADS 的 SPI 资格测试与 FDC 的 I²C 检测重叠执行。持续监测用三颗前端的身份读数决定是否暂停采集。只有所有总线所有者均已释放后才可重探。GPIO 回读描述控制脚，参考节点隔离能力取决于实装连线。

This layer maps S/D coordinates to TMUX controls, FDC addresses and channels, and ADS inputs. Startup overlaps ADS SPI qualification with FDC I²C discovery. Runtime presence checks use the identities of all three frontends to decide when to pause capture and require all bus owners to be quiescent before probing. GPIO readback describes a control pin. Reference-node isolation is a board capability.

## 调用方式 / Calling the module

```c
uint8_t muxp, muxn;
bool mapped = sensorarrayBoardMapAdsMuxForDLine(1, &muxp, &muxn);
```

例中 D1 使用 AIN0 对 AINCOM，D1..D8 使用 1..8 编号。启动扫描将跳过高于配置上限的 SPI 候选；高段候选间隔 100 kHz，低段主要为 50 kHz，最低 125 kHz。候选需要探测与确认，校验失败受千分比上限和奇偶样本覆盖约束。记录请求与驱动实测速率，成功还需恢复保存状态。运行期先暂停；三次完整失联轮次确认等待，恢复时两轮稳定身份读数触发受控重启，启动扫描随新启动重跑。

D1 maps to AIN0 against AINCOM in this example. D1..D8 are one-based. The boot probe skips SPI candidates above the configured limit. The upper steps are 100 kHz, most lower steps are 50 kHz, and the lowest candidate is 125 kHz. Qualification requires probe and confirmation passes with bounded CRC errors and good samples of both parities. It records requested and actual clocks and must restore the saved state. At runtime, suspicion pauses capture, three complete absent rounds confirm the wait, and two clean rounds of stable identities trigger a controlled restart and startup qualification.

## 时钟资格与等待 / Clock qualification and waiting

当前构建每个候选先读 400 个样本，再用 1,600 个确认。CRC 容许上限为 25‰，另需足够的奇偶世代有效样本。带校验错误的转换只计入诊断。芯片身份读取、状态错误、DRDY 超时、候选设置/恢复错误有独立结果。启动作业与运行期 presence 保留各自的所有权交接记录。

The checked-in configuration uses 400 probe samples and 1,600 confirmation samples per candidate. Its CRC allowance is 25‰, alongside minimum good counts on both generation parities. CRC failures remain diagnostic observations. Identity, status, DRDY timeout, clock setup and restore each have distinct failure evidence. Startup and runtime presence maintain their own ownership handoffs.

[板级配置 / Board configuration](../../main/kconfigs/Kconfig.board) · [工程选项 / Project settings](../../sdkconfig.defaults.esp32s3)

## 源码和头文件 / Source and headers

| 文件 / File | 职责与声明 / Responsibility and declarations |
| --- | --- |
| [sensorarrayAdsSpiProbe.c](sensorarrayAdsSpiProbe.c) · [sensorarrayAdsSpiProbe.h](sensorarrayAdsSpiProbe.h) | 在同一启动所有权与总期限内比较寄存器、采样两种 DRDY 世代奇偶、统计 CRC、确认速率并恢复现场。 / Qualifies a candidate clock with register checks, DRDY-generation parity and CRC evidence under one boot owner and deadline, then restores the saved state. `sensorarrayAdsSpiProbeResult_t` |
| [sensorarrayBoardMap.c](sensorarrayBoardMap.c) · [sensorarrayBoardMap.h](sensorarrayBoardMap.h) | 把 S/D、SELA、SELB、REF 和两颗 FDC/ADS 输入转换为物理路径；输出每种模式的控制和参考能力。 / Maps S/D, SELA, SELB, REF and FDC/ADS inputs to physical routes and describes mode-specific control and reference capabilities. `sensorarrayBoardRouteProfile_t` |
| [sensorarrayBringup.c](sensorarrayBringup.c) · [sensorarrayBringup.h](sensorarrayBringup.h) | 配置、寻址并读取前端的身份与关键寄存器，绑定 I²C/SPI/GPIO；在原有所有权内进行限时 presence 检测。 / Attaches I²C/SPI/GPIO resources, discovers device identities, checks key registers and performs bounded presence checks under existing ownership. `sensorarrayFdcDiscoveryStatus_t` `sensorarrayFdcProbeDiag_t` |
| [sensorarrayFrontendPresence.c](sensorarrayFrontendPresence.c) · [sensorarrayFrontendPresence.h](sensorarrayFrontendPresence.h) | 用时间戳、连续轮次、三颗器件读到的身份与恢复结果判定 MONITOR/SUSPECT/WAIT/RESTART_REQUIRED。 / Advances MONITOR/SUSPECT/WAIT/RESTART_REQUIRED from timestamped rounds, three device identities and verified probe cleanup. `sensorarrayFrontendLink_t` `sensorarrayFrontendPresencePhase_t` |
| [sensorarrayFrontendStartup.c](sensorarrayFrontendStartup.c) · [sensorarrayFrontendStartup.h](sensorarrayFrontendStartup.h) | 在持久启动作业中并行组织 ADS SPI 资格测试和 FDC 时钟检查，向控制面提供不可变的 SPI 结果。 / Coordinates parallel startup ADS SPI qualification and FDC clock checks using a persistent job and publishes an immutable SPI result for queries. |

## 函数 / Functions

签名按源文件排列，参数顺序即调用顺序。下表标出生产代码中的调用者或回调注册者（最多两个）。`static` 限于所在编译单元。错误码列于对应行为后，只列函数直接返回的常量；硬件与下层调用可传播其他错误。

Signatures are grouped by source in call-argument order. The last column names up to two production callers or callback registrars. `static` entries are local to their translation unit. Constants after a description are explicit error returns. Underlying operations may propagate additional errors.

[sensorarrayAdsSpiProbe.c](#sensorarrayadsspiprobec) · [sensorarrayBoardMap.c](#sensorarrayboardmapc) · [sensorarrayBringup.c](#sensorarraybringupc) · [sensorarrayFrontendPresence.c](#sensorarrayfrontendpresencec) · [sensorarrayFrontendStartup.c](#sensorarrayfrontendstartupc)

### sensorarrayAdsSpiProbe.c

| C 调用签名 / C signature | 行为及返回 / Behaviour and return | 调用位置 / Called by |
| --- | --- | --- |
| `static bool sensorarrayAdsSpiProbeRegistersMatch(const ads126xAdc1RegisterSnapshot_t *expected, const ads126xAdc1RegisterSnapshot_t *actual)` [47](sensorarrayAdsSpiProbe.c#L47) | 比较已保存和读回的 ADC1 配置与校准寄存器。 / Compare saved and observed ADC1 configuration and calibration registers. | `sensorarrayAdsSpiProbeVerify` |
| `static esp_err_t sensorarrayAdsSpiProbeVerify(ads126xAdcHandle_t *ads, const ads126xAdc1RegisterSnapshot_t *expected)` [62](sensorarrayAdsSpiProbe.c#L62) | 重新读取寄存器并核对预期值，保存读回错误。 / Read back the registers, compare against the expectation and retain readback failure. | `sensorarrayAdsSpiProbePass` / `sensorarrayAdsSpiProbeRun` |
| `static void sensorarrayAdsSpiProbeAccumulate(sensorarrayAdsSpiProbeResult_t *result, const sensorarrayAdsSpiProbePass_t *pass)` [74](sensorarrayAdsSpiProbe.c#L74) | 把单轮选速的样本、CRC、状态和超时计数累加到总结果。 / Accumulate per-pass sample, CRC, status and timeout counts into the overall result. | `sensorarrayAdsSpiProbePass` |
| `static esp_err_t sensorarrayAdsSpiProbePass(ads126xAdcHandle_t *ads, ads126xAdc1RegisterSnapshot_t *expected, uint32_t requestedHz, bool bypass, bool confirmation, uint32_t sampleTarget, uint64_t workDeadlineUs, sensorarrayAdsSpiProbeResult_t *result)` [87](sensorarrayAdsSpiProbe.c#L87) | 在一个候选时钟采够指定数量的新转换，按 CRC 比例和奇偶校验覆盖判定是否通过。 / Test fresh conversions at one candidate clock against the CRC fraction and good-sample parity requirements. | `sensorarrayAdsSpiProbeRun` |
| `esp_err_t sensorarrayAdsSpiProbeRun(sensorarrayState_t *state, uint64_t absoluteDeadlineUs, sensorarrayAdsSpiProbeResult_t *outResult)` [209](sensorarrayAdsSpiProbe.c#L209) | 在绝对截止时间内保存 ADC1 状态、探测并确认时钟、恢复原配置，填写起因和独立清理错误。 / Save ADC1 state, probe and confirm a clock, restore the saved configuration and report causal and cleanup errors within the absolute deadline. `ESP_ERR_INVALID_ARG` | `sensorarrayBootSpiTask` |

### sensorarrayBoardMap.c

| C 调用签名 / C signature | 行为及返回 / Behaviour and return | 调用位置 / Called by |
| --- | --- | --- |
| `const char *sensorarrayBoardMapSelaRouteName(sensorarraySelaRoute_t route)` [39](sensorarrayBoardMap.c#L39) | 返回SELA路由的显示名称。 / Return the display name for SELA route. | `sensorarrayBoardMapAudit` / `sensorarrayMeasureApplyRoute` |
| `bool sensorarrayBoardMapSelaRouteToGpioLevel(sensorarraySelaRoute_t route, int *outLevel)` [51](sensorarrayBoardMap.c#L51) | 将选定的 SELA 逻辑路径编码成要驱动的 GPIO 电平。 / Encode a logical SELA route as its commanded GPIO level. | `sensorarrayBoardMapAudit` / `sensorarrayMeasureSetSelaPath` |
| `bool sensorarrayBoardMapSelaRouteFromGpioLevel(int gpioLevel, sensorarraySelaRoute_t *outRoute)` [78](sensorarrayBoardMap.c#L78) | 将 SELA 实际 GPIO 电平解码为板级逻辑路径。 / Decode a SELA GPIO level to its board route. | `sensorarrayMeasureWriteSela` |
| `bool sensorarrayBoardMapFdcSelBLevel(bool *outLevel)` [96](sensorarrayBoardMap.c#L96) | 返回板级 FDC 支路所需的 SELB 高电平。 / Return the high SELB level required for the board FDC path. | `sensorarrayMeasureApplyFdcCellRoute` / `sensorarrayMeasureSetFdcSelBPathQuiet` |
| `bool sensorarrayBoardMapAdsMuxForDLine(uint8_t dLine, uint8_t *muxp, uint8_t *muxn)` [106](sensorarrayBoardMap.c#L106) | 将 D1..D8 映射到 ADS 正负输入码；写入 muxp 与 muxn。 / Map D1..D8 into ADS positive/negative input codes written to muxp and muxn. | `sensorarrayAdsMatrixMeasureCellImpl` / `sensorarrayMeasureReadAdsUv` |
| `const sensorarrayRouteMap_t *sensorarrayBoardMapFindRoute(uint8_t sColumn, uint8_t dLine, sensorarrayPath_t path)` [118](sensorarrayBoardMap.c#L118) | 定位与 S、D 和目标路径同时匹配的板级表项。 / Find the board-map entry matching the S coordinate, D line and route. | `sensorarrayMeasureApplyRoute` |
| `const sensorarrayFdcDLineMap_t *sensorarrayBoardMapFindFdcByDLine(uint8_t dLine)` [131](sensorarrayBoardMap.c#L131) | 查出指定 D 线的 FDC 器件和通道映射。 / Find the FDC device and channel for a D line. | `sensorarrayFdcSweepInitCalibrationCell` / `sensorarrayMeasureGetFdcStateForDLine` |
| `size_t sensorarrayBoardMapRouteCount(void)` [141](sensorarrayBoardMap.c#L141) | 返回板级路由表的元素数量。 / Return the number of board-route entries. | `sensorarrayBoardMapRouteAt` / `sensorarrayBoardMapAudit` |
| `const sensorarrayRouteMap_t *sensorarrayBoardMapRouteAt(size_t index)` [146](sensorarrayBoardMap.c#L146) | 返回索引对应的只读路由，越界时返回空指针。 / Return the indexed read-only route or NULL outside the table. | `sensorarrayBoardMapAudit` |
| `size_t sensorarrayBoardMapFdcCount(void)` [154](sensorarrayBoardMap.c#L154) | 返回板上 D 线至 FDC 通道映射的元素数量。 / Return the number of board D-line to FDC-channel mappings. | `sensorarrayBoardMapFdcAt` / `sensorarrayBoardMapAudit` |
| `const sensorarrayFdcDLineMap_t *sensorarrayBoardMapFdcAt(size_t index)` [159](sensorarrayBoardMap.c#L159) | 返回给定下标的只读 D 线/FDC 项，越界返回空指针。 / Return the indexed D-line/FDC mapping or NULL outside the table. | `sensorarrayBoardMapAudit` |
| `const char *sensorarrayBoardMapPathName(sensorarrayPath_t path)` [167](sensorarrayBoardMap.c#L167) | 返回路径的显示名称。 / Return the display name for path. | `sensorarrayBoardMapAudit` |
| `sensorarrayRoutePathKind_t sensorarrayBoardMapPathToRoutePath(sensorarrayPath_t path, tmux1108Source_t swSource)` [172](sensorarrayBoardMap.c#L172) | 结合测量路径和开关源识别实际选择的模拟路径类别。 / Combine the measurement path and switch source to classify the selected analogue route. | `sensorarrayMeasureApplyRoute` |
| `tmux1108Source_t sensorarrayBoardMapDefaultSwSource(const sensorarrayRouteMap_t *route)` [183](sensorarrayBoardMap.c#L183) | 由路由表给出默认的 GND/REF 开关源。 / Resolve the default GND/REF switch source from the route entry. | 独立接口 / Standalone entry |
| `bool sensorarrayBoardMapGetRouteProfile(sensorarrayMeasurementMode_t mode, sensorarrayBoardRouteProfile_t *outProfile)` [197](sensorarrayBoardMap.c#L197) | 将目标模式展开为 SELA/SELB、SW/Q1、激励与前端所需状态。 / Expand a target mode into required SELA/SELB, SW/Q1, excitation and frontend states. | `sensorarrayBoardMapGetSafeRailMonitorProfile` / `sensorarrayRouteControllerEnterSafe` |
| `bool sensorarrayBoardMapGetSafeRailMonitorProfile(sensorarrayBoardRouteProfile_t *outProfile)` [266](sensorarrayBoardMap.c#L266) | 依据实装能力决定是否有隔离的内部监测路由；未具备隔离返回 false。 / Report whether the board has a physically isolated internal monitor route, returning false when isolation is unavailable. | `sensorarrayRouteControllerEnterSafeRailMonitor` |
| `bool sensorarrayBoardMapHasIsolatedRailMonitor(void)` [280](sensorarrayBoardMap.c#L280) | 给出板级参考节点的硬件隔离能力。 / Report the board's hardware isolation capability for the reference node. | `sensorarrayAdsReadRailMonitorUv` / `sensorarrayMixedRowPrepareRailAtBoundary` |
| `const char *sensorarrayBoardMapMatrixExcitationName(bool enabled)` [285](sensorarrayBoardMap.c#L285) | 返回矩阵激励的显示名称。 / Return the display name for matrix excitation. | `sensorarrayRuntimeQueryCommand` |
| `void sensorarrayBoardMapAudit(void)` [290](sensorarrayBoardMap.c#L290) | 遍历并输出路由和 FDC 通道映射，供核对实装连线。 / Enumerate and report routes and FDC-channel mappings for wiring review. | `sensorarrayInitBoardAndRouting` |

### sensorarrayBringup.c

| C 调用签名 / C signature | 行为及返回 / Behaviour and return | 调用位置 / Called by |
| --- | --- | --- |
| `static bool sensorarrayAdsDedicatedSpiOwner(spi_device_handle_t device)` [44](sensorarrayBringup.c#L44) | 判定该 SPI 设备句柄是否满足专用、空闲且未隔离的探测条件。 / Check whether the SPI handle qualifies as dedicated, idle and free of quarantine. | `sensorarrayBringupInitAds` |
| `bool sensorarrayBringupAdsPresenceBusy(void)` [56](sensorarrayBringup.c#L56) | 检查身份探测的 busy/pending 标记。 / Check whether an identity probe is busy or has a pending transfer. | `sensorarrayAdsDedicatedSpiOwner` / `sensorarrayFrontendOwnersQuiescent` |
| `bool sensorarrayBringupAdsPresenceQuarantined(void)` [62](sensorarrayBringup.c#L62) | 读取持久的身份探测隔离标志。 / Read the persistent identity-probe quarantine flag. | `sensorarrayAdsDedicatedSpiOwner` / `sensorarrayBringupProbeAdsPresence` |
| `static TickType_t sensorarrayAdsPresenceTicks(uint64_t deadlineUs)` [67](sensorarrayBringup.c#L67) | 将距绝对期限的剩余微秒向上换算为有限 RTOS tick；已过期返回零。 / Convert remaining microseconds to bounded RTOS ticks, rounding up and returning zero on expiry. | `sensorarrayAdsPresenceReadSlot` |
| `static esp_err_t sensorarrayAdsPresenceReadSlot(spi_device_handle_t device, uint64_t deadlineUs, uint32_t clockHz, uint8_t registers[3])` [75](sensorarrayBringup.c#L75) | 通过长期存活的 DMA 槽读取身份寄存器；未回收的传输阻止复用。 / Read identity registers through a persistent DMA slot, withholding reuse while a transfer is outstanding. `ESP_ERR_INVALID_STATE` `ESP_ERR_NO_MEM` `ESP_ERR_TIMEOUT` | `sensorarrayBringupProbeAdsPresence` |
| `esp_err_t sensorarrayBringupProbeAdsPresence(sensorarrayState_t *state, uint64_t absoluteDeadlineUs, sensorarrayAdsPresenceResult_t *outResult)` [116](sensorarrayBringup.c#L116) | 按指定时钟收集 ADS 可达性证据并保留总线清理结果。 / Gather ADS reachability at the specified clock and preserve bus-cleanup outcome. `ESP_ERR_INVALID_ARG` | `sensorarrayFrontendPresencePoll` |
| `static void sensorarrayDelayMs(uint32_t delayMs)` [275](sensorarrayBringup.c#L275) | 等待 ms。 / Wait for ms. | `sensorarrayBringupPrepareAdsRefPath` |
| `static gpio_num_t sensorarrayToGpio(int gpio)` [282](sensorarrayBringup.c#L282) | 将板级 GPIO 编号转换为驱动使用的 gpio_num_t。 / Convert the board pin number to the driver's gpio_num_t representation. | `sensorarrayBringupInitAds` / `sensorarrayBringupAttachAdsNoReset` |
| `static esp_err_t sensorarrayBringupLogAdsCoreRegisters(sensorarrayState_t *state, const char *stage)` [287](sensorarrayBringup.c#L287) | 输出 ADS核心寄存器。 / Report ADS core registers. `ESP_ERR_INVALID_ARG` | `sensorarrayBringupPrepareAdsRefPath` |
| `static esp_err_t sensorarrayInitSpi(spi_device_handle_t *outDevice)` [316](sensorarrayBringup.c#L316) | 按板级引脚和主机号初始化 ADS SPI 总线并建立器件句柄。 / Initialise the ADS SPI bus from board pins and host selection and attach the device handle. `ESP_ERR_INVALID_ARG` | `sensorarrayBringupInitAds` / `sensorarrayBringupAttachAdsNoReset` |
| `static Fdc2214CapRefClockSource_t sensorarrayBringupFdcRefClockSource(void)` [372](sensorarrayBringup.c#L372) | 由板级构建配置选择 FDC 内外参考时钟。 / Select the configured internal or external clock source for the FDC. | `sensorarrayBringupFdcBuildFinalConfig` / `sensorarrayBringupDumpFdcInitRegisters` |
| `static const char *sensorarrayBringupFdcRefClockName(Fdc2214CapRefClockSource_t source)` [381](sensorarrayBringup.c#L381) | 返回FDC参考时钟的显示名称。 / Return the display name for FDC reference clock. | `sensorarrayBringupDumpFdcInitRegisters` |
| `static bool sensorarrayBringupFdcAutoScanForChannels(uint8_t channels)` [386](sensorarrayBringup.c#L386) | 根据启用的通道数量选择 FDC 的自动扫描配置。 / Select FDC autoscan configuration for the requested channel count. | `sensorarrayBringupFdcExpectedMuxConfig` / `sensorarrayBringupInitFdcDevice` |
| `static uint8_t sensorarrayBringupFdcRrSequenceForChannels(uint8_t channels)` [391](sensorarrayBringup.c#L391) | 根据活动通道数选择 round-robin 扫描序列。 / Select the round-robin channel sequence for the number of active channels. | `sensorarrayBringupFdcExpectedMuxConfig` / `sensorarrayBringupInitFdcDevice` |
| `static Fdc2214CapChannelConfig_t sensorarrayBringupFdcMatrixChannelProfile(uint8_t channelIndex)` [402](sensorarrayBringup.c#L402) | 从板级参数构造 FDC 通道的初始测量设置。 / Build initial FDC channel measurement settings from board configuration. | `sensorarrayBringupInitFdcDevice` |
| `static uint16_t sensorarrayBringupFdcExpectedMuxConfig(uint8_t channels)` [420](sensorarrayBringup.c#L420) | 从通道和扫描策略构造预期 MUX_CONFIG 字。 / Construct the expected MUX_CONFIG word for the channel/scan policy. | `sensorarrayBringupInitFdcDevice` / `sensorarrayBringupInitFdcSingleChannel` |
| `static uint16_t sensorarrayBringupFdcExpectedStatusConfig(void)` [433](sensorarrayBringup.c#L433) | 构造预期的 FDC 状态/中断寄存器配置。 / Construct the expected FDC status/interrupt configuration word. | `sensorarrayBringupInitFdcDevice` / `sensorarrayBringupInitFdcSingleChannel` |
| `static uint16_t sensorarrayBringupFdcBuildFinalConfig(Fdc2214CapChannel_t activeChannel)` [438](sensorarrayBringup.c#L438) | 由所选通道、扫描与参考源生成最终配置寄存器值。 / Assemble the final configuration word for the selected channel, scan and reference. | `sensorarrayBringupInitFdcDevice` / `sensorarrayBringupInitFdcSingleChannel` |
| `static esp_err_t sensorarrayBringupDumpFdcInitRegisters(const BoardSupportI2cCtx_t *i2cCtx, uint8_t i2cAddr, const char *fdcLabel, const char *stage, Fdc2214CapDevice_t *dev, uint16_t manufacturerId, uint16_t deviceId)` [452](sensorarrayBringup.c#L452) | 转储 FDC初始化寄存器。 / Dump FDC init registers. `ESP_ERR_INVALID_ARG` | `sensorarrayBringupInitFdcDevice` |
| `static void sensorarrayBringupDumpFdcClock(Fdc2214CapDevice_t *dev, const char *fdcLabel, const char *stage, Fdc2214CapChannel_t channel)` [494](sensorarrayBringup.c#L494) | 转储 FDC时钟。 / Dump FDC clock. | `sensorarrayBringupInitFdcDevice` / `sensorarrayBringupInitFdcSingleChannel` |
| `static esp_err_t sensorarrayBringupVerifyFdcActiveState(Fdc2214CapDevice_t *dev, uint8_t channels, uint16_t expectedStatusConfig, uint16_t expectedConfig, uint16_t expectedMuxConfig)` [550](sensorarrayBringup.c#L550) | 回读寄存器核对 FDC 已在预期活动模式，返回首个错误。 / Read back FDC configuration to verify the expected active state and return failure. `ESP_ERR_INVALID_ARG` `ESP_ERR_INVALID_STATE` `ESP_ERR_INVALID_RESPONSE` | `sensorarrayBringupInitFdcDevice` / `sensorarrayBringupInitFdcSingleChannel` |
| `esp_err_t sensorarrayBringupProbeFdcAddress(const BoardSupportI2cCtx_t *i2cCtx, uint8_t i2cAddr, uint16_t *outManufacturerId)` [620](sensorarrayBringup.c#L620) | 通过 I²C 地址探测指定器件，向调用者返回驱动状态。 / Probe the designated I²C address and return the driver status. `ESP_ERR_INVALID_ARG` | 独立接口 / Standalone entry |
| `esp_err_t sensorarrayBringupReadFdcIdsRaw(const BoardSupportI2cCtx_t *i2cCtx, uint8_t i2cAddr, uint16_t *outManufacturerId, uint16_t *outDeviceId)` [637](sensorarrayBringup.c#L637) | 从制造商和器件 ID 寄存器读取两个 16 位身份字。 / Read the two 16-bit manufacturer and device identity registers. `ESP_ERR_INVALID_ARG` | `sensorarrayBringupProbeFdcCandidate` / `sensorarrayProbeFdcIdsAtClock` |
| `const char *sensorarrayBringupFdcDiscoveryStatusName(sensorarrayFdcDiscoveryStatus_t status)` [664](sensorarrayBringup.c#L664) | 返回FDC识别状态的显示名称。 / Return the display name for FDC discovery status. | `sensorarrayBringupProbeFdcBus` |
| `static void sensorarrayBringupInitFdcProbeDiag(sensorarrayFdcProbeDiag_t *diag)` [679](sensorarrayBringup.c#L679) | 初始化 FDC探测诊断。 / Initialise FDC probe diagnostics. | `sensorarrayBringupProbeFdcCandidate` |
| `esp_err_t sensorarrayBringupProbeFdcCandidate(const BoardSupportI2cCtx_t *i2cCtx, uint8_t i2cAddr, sensorarrayFdcProbeDiag_t *outDiag)` [697](sensorarrayBringup.c#L697) | 探测一个候选地址并按实际制造商/器件 ID 分类。 / Probe one candidate address and classify its observed manufacturer/device identity. `ESP_ERR_INVALID_ARG` | `sensorarrayBringupProbeFdcBus` |
| `uint8_t sensorarrayBringupNormalizeFdcChannels(uint8_t channels)` [738](sensorarrayBringup.c#L738) | 将请求的通道数限定到 FDC 支持的配置。 / Normalise the requested channel count to a supported FDC configuration. | `sensorarrayBringupInitFdcDevice` / `sensorarrayInitRuntime` |
| `void sensorarrayBringupResetFdcState(sensorarrayFdcDeviceState_t *fdcState, const char *label, uint8_t i2cAddr)` [749](sensorarrayBringup.c#L749) | 重置该 FDC 的运行状态结构以便重新识别或初始化。 / Reset the device's runtime state before rediscovery or initialisation. | `sensorarrayInitRuntime` |
| `bool sensorarrayBringupParseI2cAddress(uint32_t configuredAddress, uint8_t *outAddress)` [777](sensorarrayBringup.c#L777) | 检查设置的地址是否为支持的 7 位 I²C 值并输出字节。 / Validate the configured seven-bit I²C address and return its byte value. | `sensorarrayInitRuntime` |
| `esp_err_t sensorarrayBringupAdsSetRefMux(sensorarrayState_t *state, uint8_t refmuxValue)` [786](sensorarrayBringup.c#L786) | 向 ADS 写入指定参考选择并保留驱动返回值。 / Apply the requested ADS reference selector and preserve the driver result. `ESP_ERR_INVALID_ARG` | `sensorarrayBringupPrepareAdsRefPath` |
| `esp_err_t sensorarrayBringupInitAds(sensorarrayState_t *state)` [801](sensorarrayBringup.c#L801) | 设置 ADS 引脚/SPI、识别芯片并建立 ADC1 初始配置。 / Set up ADS pins/SPI, identify the chip and establish initial ADC1 configuration. `ESP_ERR_INVALID_ARG` | `sensorarrayFrontendStartupRun` |
| `esp_err_t sensorarrayBringupAttachAdsNoReset(sensorarrayState_t *state)` [875](sensorarrayBringup.c#L875) | 绑定已存在的 ADS 配置，避免附着操作重置芯片。 / Attach to the existing ADS configuration without resetting the device. `ESP_ERR_INVALID_ARG` `ESP_ERR_NOT_SUPPORTED` | 独立接口 / Standalone entry |
| `esp_err_t sensorarrayBringupPrepareAdsRefPath(sensorarrayState_t *state)` [920](sensorarrayBringup.c#L920) | 准备 ADS 参考输入与板级开关路径，返回配置结果。 / Prepare the ADS reference selection and board switch path and return setup status. `ESP_ERR_INVALID_ARG` `ESP_ERR_INVALID_STATE` | 独立接口 / Standalone entry |
| `void sensorarrayBringupProbeFdcBus(const sensorarrayFdcDeviceState_t *fdcState)` [990](sensorarrayBringup.c#L990) | 在对应 I²C 总线上探测候选 FDC 地址并保存分类与诊断。 / Probe candidate FDC addresses on the specified I²C bus and retain classification/evidence. | `sensorarrayInitFdcDevice` |
| `void sensorarrayBringupInitFdcDiag(sensorarrayFdcInitDiag_t *diag)` [1031](sensorarrayBringup.c#L1031) | 以诊断模式初始化指定 FDC 并保存可读到的身份和设置。 / Initialise the selected FDC for diagnostics and retain available identity/settings. | `sensorarrayBringupInitFdcDevice` / `sensorarrayBringupInitFdcSingleChannel` |
| `void sensorarrayBringupApplyFdcInitResult(sensorarrayFdcDeviceState_t *fdcState, Fdc2214CapDevice_t *deviceHandle, esp_err_t initErr, const sensorarrayFdcInitDiag_t *diag)` [1050](sensorarrayBringup.c#L1050) | 将驱动句柄与初始化结果提交到该器件的状态对象。 / Publish the resulting device handle and initialisation status into its state object. | `sensorarrayInitFdcDevice` |
| `esp_err_t sensorarrayBringupInitFdcDevice(const BoardSupportI2cCtx_t *i2cCtx, uint8_t i2cAddr, uint8_t channels, Fdc2214CapDevice_t **outDev, sensorarrayFdcInitDiag_t *outDiag)` [1073](sensorarrayBringup.c#L1073) | 为给定总线和地址配置 FDC 回调、寄存器及所需通道。 / Configure FDC callbacks, registers and requested channels on the supplied bus/address. `ESP_ERR_NOT_SUPPORTED` `ESP_ERR_INVALID_ARG` `ESP_ERR_INVALID_RESPONSE` | `sensorarrayBringupInitFdcSingleChannel` / `sensorarrayInitFdcDevice` |
| `esp_err_t sensorarrayBringupInitFdcSingleChannel(const BoardSupportI2cCtx_t *i2cCtx, uint8_t i2cAddr, Fdc2214CapChannel_t channel, Fdc2214CapDevice_t **outDev, sensorarrayFdcInitDiag_t *outDiag)` [1307](sensorarrayBringup.c#L1307) | 为诊断路径建立单通道 FDC 采集配置。 / Initialise the single-channel FDC configuration for diagnostics. `ESP_ERR_NOT_SUPPORTED` `ESP_ERR_INVALID_ARG` | 独立接口 / Standalone entry |

### sensorarrayFrontendPresence.c

| C 调用签名 / C signature | 行为及返回 / Behaviour and return | 调用位置 / Called by |
| --- | --- | --- |
| `static uint64_t frontendAddTime(uint64_t time, uint64_t interval)` [21](sensorarrayFrontendPresence.c#L21) | 对微秒时刻和间隔作饱和加法，避免回绕成过去。 / Add a microsecond interval to a timestamp with saturation. | `sensorarrayFrontendPresenceObserve` / `sensorarrayFrontendPresencePoll` |
| `static uint32_t frontendIncrement(uint32_t value)` [26](sensorarrayFrontendPresence.c#L26) | 使用饱和加法递增连续探测计数。 / Increment a probe-run counter with saturation. | `sensorarrayFrontendPresenceObserve` |
| `void sensorarrayFrontendPresenceInit(sensorarrayFrontendPresence_t *presence, uint64_t nowUs)` [31](sensorarrayFrontendPresence.c#L31) | 清空连续轮次并以当前单调时间建立首次连通监测。 / Reset probe-run history and schedule initial presence monitoring from monotonic time. | `app_main` |
| `void sensorarrayFrontendPresenceLatchHold(sensorarrayFrontendPresence_t *presence, uint64_t nowUs)` [43](sensorarrayFrontendPresence.c#L43) | 在所有者尚未释放或启动未验证时登记疑似状态并暂停采集。 / Latch suspicion and hold capture after unqualified startup or delayed owner release. | `sensorarrayEnterFrontendHold` |
| `bool sensorarrayFrontendPresenceCopySnapshot(const sensorarrayFrontendPresence_t *presence, sensorarrayFrontendPresenceSnapshot_t *outSnapshot)` [57](sensorarrayFrontendPresence.c#L57) | 在状态锁内复制三颗前端的最新观测与计数。 / Copy latest evidence and counters for the three frontends under the state lock. | `sensorarrayFrontendPresenceDue` / `sensorarrayFrontendPresenceIsWaiting` |
| `bool sensorarrayFrontendPresenceDue(const sensorarrayFrontendPresence_t *presence, uint64_t nowUs)` [67](sensorarrayFrontendPresence.c#L67) | 比较当前时间和下次探测的单调期限，判断是否应探测。 / Compare current time with the monotonic next-probe deadline. | `sensorarrayFrontendPresencePoll` / `sensorarrayRunFrontendPresenceBoundary` |
| `bool sensorarrayFrontendPresenceIsWaiting(const sensorarrayFrontendPresence_t *presence)` [76](sensorarrayFrontendPresence.c#L76) | 查询当前的前端等待/恢复阶段。 / Determine whether frontend presence is in a wait/recovery stage. | `sensorarrayRunFrontendPresenceBoundary` |
| `const char *sensorarrayFrontendPresencePhaseName(sensorarrayFrontendPresencePhase_t phase)` [83](sensorarrayFrontendPresence.c#L83) | 返回阶段的显示名称。 / Return the display name for phase. | `sensorarrayRunFrontendPresenceBoundary` / `sensorarrayFormatFrontendReply` |
| `const char *sensorarrayFrontendPresenceLinkName(sensorarrayFrontendLink_t link)` [94](sensorarrayFrontendPresence.c#L94) | 返回链路的显示名称。 / Return the display name for link. | `sensorarrayFormatFrontendReply` |
| `sensorarrayFrontendPresenceAction_t sensorarrayFrontendPresenceObserve(sensorarrayFrontendPresence_t *presence, const sensorarrayFrontendPresenceEvidence_t *evidence, uint64_t nowUs)` [105](sensorarrayFrontendPresence.c#L105) | 消费一轮唯一、到期的三芯片证据，推进连续计数并返回协调器下一动作。 / Consume one distinct due round of three-device evidence, advance consecutive-run counters and return the coordinator action. | `sensorarrayFrontendPresencePoll` |
| `static bool frontendBeforeDeadline(uint64_t deadlineUs, uint64_t admissionUs)` [185](sensorarrayFrontendPresence.c#L185) | 检查实时时钟是否仍在探测轮次的截止时间内。 / Check the probe-round deadline against the live clock. | `frontendReadFdc` |
| `static void frontendReadFdc(const sensorarrayFdcDeviceState_t *fdc, bool lowSpeed, uint64_t deadlineUs, sensorarrayFrontendPresenceDevice_t *out)` [191](sensorarrayFrontendPresence.c#L191) | 读取 FDC 身份对并将时钟和清理结果写入该器件的轮次证据。 / Read the FDC identity pair and retain probe clocks and cleanup outcome in the device evidence. | `sensorarrayFrontendPresencePoll` |
| `sensorarrayFrontendPresenceAction_t sensorarrayFrontendPresencePoll(sensorarrayFrontendPresence_t *presence, sensorarrayState_t *state, uint64_t nowUs, bool ownerQuiescent, bool fdcSdHigh, sensorarrayFrontendPresenceSnapshot_t *outSnapshot)` [271](sensorarrayFrontendPresence.c#L271) | 核对所有权与 FDC SD，执行只读身份探测，恢复探测时钟并提交本轮证据。 / Check ownership and FDC SD, perform read-only identity probes, restore probe clocks and submit the evidence round. | `sensorarrayRunFrontendPresenceBoundary` |

### sensorarrayFrontendStartup.c

| C 调用签名 / C signature | 行为及返回 / Behaviour and return | 调用位置 / Called by |
| --- | --- | --- |
| `static const char *sensorarrayBootSpiPhaseName(sensorarrayBootSpiPhase_t phase)` [58](sensorarrayFrontendStartup.c#L58) | 返回阶段的显示名称。 / Return the display name for phase. | `sensorarrayFrontendStartupFormatSpi` / `sensorarrayFrontendStartupFormatSpiSource` |
| `static sensorarrayBootSpiSnapshot_t sensorarrayBootSpiCopy(void)` [70](sensorarrayFrontendStartup.c#L70) | 在保护区内复制已发布的启动 SPI 结果。 / Copy the published boot SPI result under its guard. | `sensorarrayFrontendStartupFormatSpi` / `sensorarrayFrontendStartupFormatSpiSource` |
| `static void sensorarrayBootSpiTask(void *arg)` [80](sensorarrayFrontendStartup.c#L80) | 在启动作业槽中执行 ADS 选速，并在退出前发布完成和恢复证据。 / Run ADS qualification from the boot job slot and publish completion and restoration evidence. | `sensorarrayBootSpiStart` |
| `static esp_err_t sensorarrayBootSpiStart(sensorarrayState_t *state)` [115](sensorarrayFrontendStartup.c#L115) | 预留持久启动作业并在独立任务开始 ADS 速率资格测试。 / Reserve the persistent startup job and start ADS qualification in a separate task. `ESP_ERR_INVALID_STATE` | `sensorarrayFrontendStartupRun` |
| `static void sensorarrayBootSpiNoteI2c(bool starting)` [159](sensorarrayFrontendStartup.c#L159) | 记录 FDC 启动检测的起止时间，供核对与 SPI 探测的重叠区间。 / Record start/end of FDC discovery to identify overlap with SPI qualification. | `sensorarrayFrontendStartupRun` |
| `static esp_err_t sensorarrayBootSpiJoin(void)` [173](sensorarrayFrontendStartup.c#L173) | 等待启动作业移交 ADS 所有权或返回有界失败。 / Join the boot job for ADS ownership handoff or bounded failure. | `sensorarrayFrontendStartupRun` |
| `esp_err_t sensorarrayFrontendStartupFormatSpi(char *response, size_t responseSize)` [224](sensorarrayFrontendStartup.c#L224) | 输出启动扫描候选、时钟、错误计数与完成状态。 / Format startup candidates, clock, error counters and completion state. | `sensorarrayEarlyRuntimeQueryCommand` / `sensorarrayRuntimeQueryCommand` |
| `esp_err_t sensorarrayFrontendStartupFormatSpiSource(char *response, size_t responseSize)` [254](sensorarrayFrontendStartup.c#L254) | 输出启动时捕获的 ADC 读策略与接口证据。 / Format the boot-time ADC read policy, source and interface evidence. `ESP_ERR_INVALID_ARG` `ESP_ERR_INVALID_SIZE` | `sensorarrayEarlyRuntimeQueryCommand` / `sensorarrayRuntimeQueryCommand` |
| `static void sensorarrayBootSpiLogJoin(esp_err_t err)` [285](sensorarrayFrontendStartup.c#L285) | 报告 ADS 启动作业的汇合结果与错误状态。 / Report ADS startup job completion and join status. | `sensorarrayFrontendStartupRun` |
| `static bool sensorarrayAppFdcIdMatches(uint16_t manufacturerId, uint16_t deviceId)` [299](sensorarrayFrontendStartup.c#L299) | 比较读出的制造商和器件 ID 是否为期望 FDC。 / Check the manufacturer and device IDs against the expected FDC. | `sensorarrayProbeFdcIdsAtClock` / `sensorarraySweepFdcI2cClockUnderLoad` |
| `static esp_err_t sensorarrayProbeFdcIdsAtClock(sensorarrayFdcDeviceState_t *fdcState, uint32_t clockHz, uint32_t attempts, uint32_t *outGoodCount, uint32_t *outBadCount, uint16_t *outLastManufacturerId, uint16_t *outLastDeviceId, const char **outReason)` [305](sensorarrayFrontendStartup.c#L305) | 在给定 I²C 时钟重复读取 FDC 身份，报告好/坏次数及最后身份。 / Repeatedly read FDC identity at the requested I²C clock and retain pass/fail counts and last IDs. `ESP_ERR_INVALID_ARG` | `sensorarrayFrontendStartupRecheckFdcClock` |
| `void sensorarrayFrontendStartupRecheckFdcClock(sensorarrayFdcDeviceState_t *fdcState, const char *mapLabel)` [393](sensorarrayFrontendStartup.c#L393) | 在 FDC 静止时复核已选 I²C 时钟并保存重新探测结果。 / Recheck the chosen I²C clock on a quiescent FDC and retain the outcome. | `sensorarrayInitFdcDevice` / `sensorarrayRuntimeI2cFallbackTick` |
| `static void sensorarraySweepFdcI2cClockUnderLoad(sensorarrayFdcDeviceState_t *fdcState, const char *mapLabel)` [488](sensorarrayFrontendStartup.c#L488) | 在实际 FDC 读取负载下评估候选 I²C 时钟并选定可用值。 / Evaluate candidate I²C clocks under real FDC read load and select a usable rate. | `sensorarrayInitFdcDevice` |
| `static void sensorarrayInitFdcDevice(sensorarrayFdcDeviceState_t *fdcState, bool addressValid, uint8_t requestedChannels, const char *mapLabel)` [603](sensorarrayFrontendStartup.c#L603) | 初始化 FDC设备。 / Initialise FDC device. 调用 / Uses `sensorarrayLogStartupFdc`, `sensorarrayFrontendStartupRecheckFdcClock`. | `sensorarrayFrontendStartupRun` |
| `static void sensorarrayLogFdcParallelCfg(void)` [754](sensorarrayFrontendStartup.c#L754) | 输出双总线并行参数与 worker 绑核配置供启动诊断。 / Report dual-bus parallel settings and worker affinity during startup. | `sensorarrayFrontendStartupRun` |
| `esp_err_t sensorarrayFrontendStartupRun(sensorarrayState_t *state, bool primaryAddressValid, bool secondaryAddressValid, uint8_t channels)` [793](sensorarrayFrontendStartup.c#L793) | 在启动协调任务中重叠执行 ADS SPI 与 FDC I²C 速率验证，等 ADS 作业交回所有权。 / Overlap ADS SPI and FDC I²C qualification on the startup coordinator and join the ADS ownership handoff. `ESP_ERR_INVALID_ARG` | `sensorarrayInitFrontends` |
| `bool sensorarrayFrontendStartupAdsOwnerQuiescent(void)` [896](sensorarrayFrontendStartup.c#L896) | 检查启动任务是否已经释放其 ADS 所有权；DMA 隔离另查。 / Check whether the boot task released ADS ownership. DMA quarantine is checked separately. | `sensorarrayFrontendOwnersQuiescent` / `sensorarrayEnterFrontendHold` |

## 参数索引 / Parameter index

相同名称按模块共享解释。完整类型、数组维度与输入输出方向以对应函数签名和调用点为准。指针参数所需存储由调用者提供，异步作业另受上面的所有权约束。

Entries share parameter descriptions within this module. Refer to the signature and its caller for the precise type, array shape and direction. The caller supplies pointer storage, subject to the ownership rules above for asynchronous work.

| 参数 / Parameter | 含义 / Meaning |
| --- | --- |
| `absoluteDeadlineUs`, `deadlineUs`, `workDeadlineUs` | 单调时钟绝对截止时间，单位 µs。 / Absolute deadline on the monotonic microsecond clock. |
| `activeChannel`, `channel`, `channelIndex` | 芯片通道编号或传输通道枚举。FDC 硬件通道为 0..3。 / Chip channel number or transport-channel enum. FDC hardware channels are 0..3. |
| `actual`, `expected` | 核验时所需的配置/身份与实际寄存器观测值，使用该接口指定的类型和掩码。 / Desired configuration/identity and actual evidence compared with the type and masks for this interface. |
| `addressValid` | 地址有效。 / Address valid. |
| `admissionUs`, `nowUs` | 单调时钟的事件时间，单位 µs。 / Event time on the monotonic clock in µs. |
| `ads`, `dev`, `device`, `fdc`, `fdcState` | 对应驱动句柄、FDC 主/次设备 ID 或 ADS/FDC 状态；按签名类型选取当前板上同一实例。 / Corresponding driver handle, primary/secondary FDC ID or ADS/FDC state. Pass the board instance matching the declared type. |
| `arg` | 回调或任务创建时绑定的上下文；在该调用的有效期内保留其存储。 / Context bound at callback registration or task creation. Keep its backing object alive for the operation. |
| `attempts`, `sampleTarget` | 候选所需样本量、允许尝试数或当前尝试索引。 / Candidate sample target, allowed number of attempts or current attempt index. |
| `bypass` | 当前事务使用或核验的旁路/输入/电源及参考设置，原始寄存器参数使用器件编码。 / Bypass/input/mode/reference settings used or checked by the transaction, using device encoding for raw register arguments. |
| `channels`, `requestedChannels` | 本器件要求启用的 FDC 通道数或通道选择，按入口契约。 / Number or selection of FDC channels requested for this device. |
| `clockHz`, `requestedHz` | 总线或器件时钟，Hz。请求值与驱动实际速率分别记录。 / Bus or device clock in Hz. Requested and actual driver clock are tracked separately. |
| `configuredAddress`, `i2cAddr` | 7 位 I²C 设备地址，匹配对应总线实例。 / Seven-bit I²C address on the selected bus. |
| `confirmation`, `pass` | 该候选的单轮选速统计或指示是否处于确认轮的标志。 / Per-candidate pass statistics or flag selecting the confirmation pass. |
| `delayMs` | 延迟，单位 ms。 / Delay in ms. |
| `deviceHandle` | 设备处理。 / Device handle. |
| `deviceId`, `manufacturerId` | 从 FDC 或 ADS 身份寄存器得到的器件标识。 / Part/manufacturer identifier obtained from device identity registers. |
| `diag` | 本次启动、采样或调度的汇总/诊断记录，类型见对应函数。 / Summary/diagnostic evidence for this startup, sample or scheduling operation, typed by the function. |
| `dLine`, `sColumn` | 板级 S 列或 D 线编号，合法位置为 1..8。 / Physical S column or D line, numbered 1..8. |
| `enabled` | 表示证据已知、数据有效、功能启用或操作成功的布尔标志，按参数名独立解释。 / Boolean representing known evidence, validity, enablement or success as named. |
| `err` | 操作、首个根因或独立清理结果的错误码。保留调用层提供的具体错误。 / Error for the operation, first cause or separate cleanup, preserving the code from the reporting layer. |
| `evidence` | 证据。 / Evidence. |
| `expectedConfig` | 预期配置。 / Expected config. |
| `expectedMuxConfig` | 预期多路选择配置。 / Expected MUX config. |
| `expectedStatusConfig` | 预期状态配置。 / Expected status config. |
| `fdcLabel` | FDC标签。 / FDC label. |
| `fdcSdHigh`, `ownerQuiescent` | 根据实际总线所有权或 FDC SD 状态提供的观测条件。未知不能当作已静止或 SD 已释放。 / Observed owner quiescence or FDC shutdown-pin state. Eligibility requires known hardware/owner state. |
| `gpio` | ESP-IDF GPIO 编号或 I²C 控制器端口号。 / ESP-IDF GPIO number or I²C controller port. |
| `gpioLevel` | 对应控制脚读到或要输出的数字电平。 / Requested or observed digital level of the indicated control pin. |
| `i2cCtx` | 对应板级 I²C 总线的驱动上下文。 / Board I²C context for the selected bus. |
| `index` | 索引。 / Index. |
| `initErr` | 初始化错误码。 / Init err. |
| `interval` | 间隔。 / Interval. |
| `label` | 标签。 / Label. |
| `link` | 链路。 / Link. |
| `lowSpeed` | 低速率。 / Low speed. |
| `mapLabel` | 映射标签。 / Map label. |
| `mode` | 对应调用域的模式枚举，保留目标、当前帧或恢复上下文的含义。 / Mode enum in the declared domain, selecting the target, frame or recovery mode as indicated. |
| `muxn`, `muxp` | ADS 正、负模拟输入选择码，请按板级 D 线映射。指针形态用于接收选择码。 / ADS positive/negative input selector from the board D-line map. Pointer forms receive the selector. |
| `out`, `response` | 调用者提供的回复、暂存或结果缓冲区，使用随同的容量/类型。 / Caller-provided reply, scratch or result storage with the accompanying capacity/type. |
| `outAddress` | 接收地址。 / Receives address. |
| `outBadCount`, `outGoodCount` | 接收本轮探测、采集或候选比较中的成功/失败数量。 / Receive counts of successful or failed probes, captures or candidates for this round. |
| `outDev`, `outDevice` | 接收结果。对应驱动句柄、FDC 主/次设备 ID 或 ADS/FDC 状态；按签名类型选取当前板上同一实例。 / Output. Corresponding driver handle, primary/secondary FDC ID or ADS/FDC state. Pass the board instance matching the declared type. |
| `outDeviceId`, `outManufacturerId` | 接收结果。从 FDC 或 ADS 身份寄存器得到的器件标识。 / Output. Part/manufacturer identifier obtained from device identity registers. |
| `outDiag` | 接收结果。本次启动、采样或调度的汇总/诊断记录，类型见对应函数。 / Output. Summary/diagnostic evidence for this startup, sample or scheduling operation, typed by the function. |
| `outLastDeviceId` | 接收上一次设备标识。 / Receives last device ID. |
| `outLastManufacturerId` | 接收上一次厂商标识。 / Receives last manufacturer ID. |
| `outLevel` | 接收结果。对应控制脚读到或要输出的数字电平。 / Output. Requested or observed digital level of the indicated control pin. |
| `outProfile` | 接收结果。配置档或八个物理行的模式列表。outModes 接收解析结果。 / Output. Configuration profile or mode selection for the eight physical rows. outModes receives parsed values. |
| `outReason` | 操作或拒绝原因字符串；outReason 用于返回原因。 / Operation/rejection reason string. outReason receives the reason. |
| `outResult`, `outSnapshot`, `result` | 函数签名注明的结果、快照或统计结构，调用者分配复制目标。 / Result, snapshot or statistics object of the declared type, with caller-owned output storage. |
| `outRoute` | 接收结果。预期/观测的路由、寄存器或状态值，按该签名中声明的类型比较。 / Output. Requested/observed route, register or state values, compared in the types shown by the signature. |
| `path` | VFS 描述符、open 标志或路径，或对应类型的矩阵路径。 / VFS descriptor, open flags or path, or a matrix route if the declared type selects one. |
| `phase`, `stage` | 对应操作、诊断或状态机的枚举/标签，采用签名中所列类型的常量。 / Operation, diagnostic or state-machine discriminator from its declared enum or label domain. |
| `presence` | 连通状态。 / Presence. |
| `primaryAddressValid` | 主级地址有效。 / Primary address valid. |
| `refmuxValue` | REFMUX值。 / REFMUX value. |
| `registers` | 对应寄存器字节或一批要读取的寄存器地址。 / Register bytes or the list of register addresses to read. |
| `responseSize` | 接收文本或字节缓冲区的容量。文本输出还需预留 NUL。 / Capacity of the supplied byte/text buffer. A text output must also accommodate its NUL terminator. |
| `route` | 预期/观测的路由、寄存器或状态值，按该签名中声明的类型比较。 / Requested/observed route, register or state values, compared in the types shown by the signature. |
| `secondaryAddressValid` | 次级地址有效。 / Secondary address valid. |
| `source` | 路由或测量的来源标记，区分外部实测、内部推算或对应开关源的枚举。 / Route/measurement source discriminator, such as external, internal estimate or switch source for the declared enum. |
| `starting` | 正在启动。 / Starting. |
| `state` | 所声明类型的状态对象或枚举。传入前端状态指针时，其生命周期覆盖总线或采集操作。 / State object or enum of the declared type. A frontend-state pointer must outlive the bus or capture operation. |
| `status` | 声明的诊断/状态记录，或从器件读回的原始状态字。 / Diagnostic/status record of the declared type or a raw status word returned by the device. |
| `swSource` | 板级模拟开关或输入源选择，可选值来自声明的 TMUX/路由类型。 / Board analogue switch or source selector from the declared TMUX/route type. |
| `time` | 时间。 / Time. |
| `value` | 值。 / Value. |

[项目总览 / Project guide](../../README.md)

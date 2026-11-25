# ADS126x 矩阵与辅助事务 / ADS126x matrix and auxiliary transactions

ADC1 依次扫描活动行的 D1..D8；D 通道相对 AINCOM 采样。生产 VOLT 为整数 µV，RES 为整数 mΩ。状态字、SPI 校验、配置世代、DRDY、参考电压与共模约束决定样本能否更新自动 PGA 与缓存。register shadow、逐模式/测点的输入档位、噪声/数值缓存及 rail fingerprint 各有独立失效条件。精度帧与寄存器健康检查使用各自的周期。

ADC1 scans D1..D8 for each active row against AINCOM. VOLT outputs integer µV and RES integer mΩ. Status bits, SPI integrity, configuration generation, DRDY, reference and common-mode limits gate admission to automatic PGA and caches. The register shadow, per-mode/cell input profile, value/noise cache and rail fingerprint have separate invalidation rules. Precision frames and register-health reads run on separate intervals.

## 调用方式 / Calling the module

```c
esp_err_t err = sensorarrayAdsMatrixEngineReadFrame(&ctx->adsEngine,
                                                     &ctx->scanPlan,
                                                     &ctx->frame);
```

示例位于已取得 ADC 所有权并应用路由的采集侧。健康检查基于引擎累计计数，保留首次失败条件和清理结果。AIN8 电池读数是分压点对 AINCOM 的差值，换算需要合法 rail 和偏置。外部 RAILCFG 为对地实测 AVDD/AVSS 的 RAM 校准；内部 span 的分轨值带估算来源。只有板级条件允许的边界事务才可用内部参考。RES 用参考到 AVSS 的激励、取样节点与参考电阻计算，OPEN/SHORT/不稳独立标记。校准持久化由 core/config 承担。

The example runs on the acquisition owner after routing and ownership are established. Health checks use the cumulative engine frame count and preserve the first failure and cleanup result. Battery input AIN8 measures a divider against AINCOM and needs usable rail and bias evidence for conversion. External RAILCFG holds ground-referenced AVDD/AVSS measurements in RAM. Splitting an internal span yields estimates with a source. Internal reference use requires an admitted boundary transaction and suitable routing. RES uses reference-to-AVSS excitation, the sampled node and the reference resistor. OPEN, SHORT and instability have distinct statuses. core/config owns calibration persistence.

## 量值、时间与保护 / Values, timing and protection

- **VOLT** 用 `Vnode = V(Dn) − V(AINCOM) + V(AINCOM 对 GND)`。AIN9 与 AINCOM 相连，校准使用 ADS 供电脚处的对地读数。 / VOLT adds the ground-referenced AINCOM estimate to the measured Dn−AINCOM differential. AIN9 is tied to AINCOM. External rail calibration uses ground-referenced readings at the ADS supply pins.
- **RES** 求 `Rx = Rref × (Vexcitation − Vnode) / (Vnode − AVSS) − pathOffset`，输出整数 mΩ。参考源规格给的是 REFOUT 对 AVSS 的跨度。高阻开路的复核读取保留测点的原始分类和掩码。 / RES uses the divider equation shown and reports integer mΩ. The reference span is specified from REFOUT to AVSS. High-impedance confirmation retains the original cell classification and masks.
- 当前工程默认 每 16 个引擎帧安排精度采样、每 64 个引擎帧安排寄存器健康检查。rail 周期设置为 500 帧，实际准入还需当前参考/路径和来源允许。电池基本周期 1,000 ms，重复相同失败会退避。 / The checked-in settings schedule precision sampling every 16 engine frames and register health every 64. The rail interval is 500 frames, subject to reference/route and source admission. Battery uses a 1,000 ms base period with backoff on repeated identical failures.
- 在该板上 U5 连接 REF，Q1 对地钳位。进入可用内部参考状态要求 SW 命令及回读为低。GPIO 核验只证明控制状态。程序选择外部校准后，普通周期监测保留外部来源。 / U5 connects to REF and Q1 clamps it to ground. Internal reference use requires low commanded and observed SW. GPIO readback is control evidence. Periodic jobs retain the selected external calibration source.

[工程选项 / Project settings](../../../sdkconfig.defaults.esp32s3) · [版图映射 / Board routing](../../board/README.md) · [校准记录 / Calibration persistence](../../config/README.md)

## 源码和头文件 / Source and headers

| 文件 / File | 职责与声明 / Responsibility and declarations |
| --- | --- |
| [sensorarrayAdsAutoRange.c](sensorarrayAdsAutoRange.c) · [sensorarrayAdsAutoRange.h](sensorarrayAdsAutoRange.h) | 根据输入共模、满量程占比和状态告警在合法的 PGA 增益间步进，输出下一档决策。 / Steps through supported PGA gains from common-mode margin, full-scale utilisation and status alarms. `sensorarrayAdsAutoRangeAction_t` `sensorarrayAdsAutoRangeConfig_t` |
| [sensorarrayAdsCache.c](sensorarrayAdsCache.c) · [sensorarrayAdsCache.h](sensorarrayAdsCache.h) | 维护独立的寄存器、档位、噪声数值与 rail 指纹，按配置世代判定命中，提供读回掩码校验。 / Maintains register, gain/profile, value/noise and rail-fingerprint caches, gates hits on generation and performs masked readback comparisons. `sensorarrayAdsInputMode_t` `sensorarrayAdsCellProfile_t` |
| [sensorarrayAdsFault.c](sensorarrayAdsFault.c) · [sensorarrayAdsFault.h](sensorarrayAdsFault.h) | 将首次失败阶段、事务结果、起因与清理状态、供电来源编码为有界诊断文本，交给调用者提供的 sink。 / Encodes first failure stage, transaction outcome, causal/cleanup errors and rail provenance into bounded text for the supplied sink. `sensorarrayAdsFaultStage_t` `sensorarrayAdsFaultOutcome_t` |
| [sensorarrayAdsGap.c](sensorarrayAdsGap.c) · [sensorarrayAdsGap.h](sensorarrayAdsGap.h) | 串行保存 ADC1、执行 rail/电池/归零/主动检查、恢复并校验，向诊断输出事务时间与前后寄存器证据。 / Serialises ADC1 save, rail/battery/zero/active checks, restore and verification, exposing transaction times and register evidence. `sensorarrayAdsGapMode_t` `sensorarrayAdsRailSource_t` |
| [sensorarrayAdsMath.c](sensorarrayAdsMath.c) · [sensorarrayAdsMath.h](sensorarrayAdsMath.h) | 对合法 rail、AINCOM、节点电压、RES 分压及电池分压进行带范围判断的定点换算，分类开短路、过载与过期。 / Performs range-checked fixed-point conversion of rails, AINCOM, node voltage and resistance/battery dividers, classifying opens, shorts, saturation and stale samples. `sensorarrayAdsRailInput_t` `sensorarrayAdsRailSplit_t` |
| [sensorarrayAdsMatrix.c](sensorarrayAdsMatrix.c) · [sensorarrayAdsMatrix.h](sensorarrayAdsMatrix.h) | 按计划将路由、rail 快照、自动 PGA、寄存器健康和新鲜转换组成一帧，分别记录逐点与整帧故障。 / Builds a planned frame from routing, rail snapshot, automatic PGA, register-health checks and fresh conversions, keeping cell and frame failures distinct. `sensorarrayAdsMatrixCalibration_t` `sensorarrayAdsMatrixEngine_t` |
| [sensorarrayAdsMeasure.inc](sensorarrayAdsMeasure.inc) | sensorarrayMeasure.c 文本包含的辅助实现。选择模拟路径并读取原始 ADS 成对数据，供兼容测量与诊断使用。 / Included by sensorarrayMeasure.c. Selects analogue paths and acquires ADS raw pairs for measurement helpers and diagnostics. |
| [sensorarrayAdsRegisters.inc](sensorarrayAdsRegisters.inc) | sensorarrayMeasure.c 包含的寄存器诊断，通过驱动读取 ADS 关键配置并供 dump 消费。 / Included by sensorarrayMeasure.c. Reads key ADS configuration through the driver for diagnostic dumps. |
| [sensorarrayAdsTimingProbe.c](sensorarrayAdsTimingProbe.c) · [sensorarrayAdsTimingProbe.h](sensorarrayAdsTimingProbe.h) | 排程有限数量的 ADC1 采样组，统计配置与 DRDY、读取、CRC 及恢复耗时，并输出探测摘要。 / Runs bounded ADC1 sample groups and reports configuration, DRDY, read and CRC observations plus restore timing. `sensorarrayAdsTimingProbeKind_t` `sensorarrayAdsTimingReadStats_t` |
| [sensorarrayBatteryScheduler.c](sensorarrayBatteryScheduler.c) · [sensorarrayBatteryScheduler.h](sensorarrayBatteryScheduler.h) | 使用单调微秒时钟计算到期、间隙容纳、帧边界补测、失败退避和下次探测时间。 / Uses monotonic microseconds for due times, gap admission, boundary fallback, failure backoff and subsequent probes. `sensorarrayBatteryDecision_t` `sensorarrayBatteryScheduler_t` |

## 函数 / Functions

签名按源文件排列，参数顺序即调用顺序。下表标出生产代码中的调用者或回调注册者（最多两个）。`static` 限于所在编译单元。错误码列于对应行为后，只列函数直接返回的常量；硬件与下层调用可传播其他错误。

Signatures are grouped by source in call-argument order. The last column names up to two production callers or callback registrars. `static` entries are local to their translation unit. Constants after a description are explicit error returns. Underlying operations may propagate additional errors.

[sensorarrayAdsAutoRange.c](#sensorarrayadsautorangec) · [sensorarrayAdsCache.c](#sensorarrayadscachec) · [sensorarrayAdsFault.c](#sensorarrayadsfaultc) · [sensorarrayAdsGap.c](#sensorarrayadsgapc) · [sensorarrayAdsMath.c](#sensorarrayadsmathc) · [sensorarrayAdsMatrix.c](#sensorarrayadsmatrixc) · [sensorarrayAdsMeasure.inc](#sensorarrayadsmeasureinc) · [sensorarrayAdsRegisters.inc](#sensorarrayadsregistersinc) · [sensorarrayAdsTimingProbe.c](#sensorarrayadstimingprobec) · [sensorarrayBatteryScheduler.c](#sensorarraybatteryschedulerc)

### sensorarrayAdsAutoRange.c

| C 调用签名 / C signature | 行为及返回 / Behaviour and return | 调用位置 / Called by |
| --- | --- | --- |
| `static int sensorarrayAdsAutoRangeGainIndex(uint8_t gain)` [9](sensorarrayAdsAutoRange.c#L9) | 将支持的 PGA 增益映射为档位表下标。 / Map a supported PGA gain to its table index. | `sensorarrayAdsAutoRangeGainSupported` / `sensorarrayAdsAutoRangeLowerGain` |
| `bool sensorarrayAdsAutoRangeGainSupported(uint8_t gain)` [19](sensorarrayAdsAutoRange.c#L19) | 检查请求的增益是否属于驱动支持的 1、2、4、8、16、32 档。 / Check whether the gain is one of the supported 1, 2, 4, 8, 16 or 32 settings. | `sensorarrayAdsAutoRangeCommonModeSafe` / `sensorarrayAdsAutoRangeDecide` |
| `uint8_t sensorarrayAdsAutoRangeLowerGain(uint8_t gain)` [24](sensorarrayAdsAutoRange.c#L24) | 沿支持的增益表向下选一档，保持在最低档以内。 / Select the next lower supported gain, bounded by the lowest setting. | `sensorarrayAdsAutoRangeDecide` |
| `uint8_t sensorarrayAdsAutoRangeHigherGain(uint8_t gain)` [30](sensorarrayAdsAutoRange.c#L30) | 沿支持的增益表向上选一档，保持在最高档以内。 / Select the next higher supported gain, bounded by the highest setting. | `sensorarrayAdsAutoRangeDecide` |
| `bool sensorarrayAdsAutoRangeCommonModeSafe(int32_t positiveInputUv, int32_t negativeInputUv, int32_t avddUv, int32_t avssUv, uint8_t gain, int32_t railMarginUv)` [37](sensorarrayAdsAutoRange.c#L37) | 同时检查两个输入在设定供轨和 PGA 余量内是否可用。 / Check both inputs against the rails and PGA common-mode margin. | `sensorarrayAdsAutoRangeDecide` / `sensorarrayAdsMatrixMeasureCellImpl` |
| `static uint16_t sensorarrayAdsAutoRangeUtilisationPermille(int32_t rawCode)` [61](sensorarrayAdsAutoRange.c#L61) | 计算绝对原始码相对满量程的千分比。 / Express absolute raw-code utilisation as permille of full scale. | `sensorarrayAdsAutoRangeDecide` |
| `sensorarrayAdsAutoRangeDecision_t sensorarrayAdsAutoRangeDecide(const sensorarrayAdsAutoRangeConfig_t *config, const sensorarrayAdsAutoRangeInput_t *input)` [70](sensorarrayAdsAutoRange.c#L70) | 结合状态告警、使用率和共模条件决定保持、升档、降档或拒绝。 / Choose hold, upshift, downshift or rejection from alarms, utilisation and common-mode constraints. | `sensorarrayAdsMatrixMeasureCellImpl` / `sensorarrayMeasurementSelfTestRun` |

### sensorarrayAdsCache.c

| C 调用签名 / C signature | 行为及返回 / Behaviour and return | 调用位置 / Called by |
| --- | --- | --- |
| `static sensorarrayAdsCellProfile_t *sensorarrayAdsProfileCell(sensorarrayAdsProfileCache_t *cache, sensorarrayMeasurementMode_t mode, uint8_t cellIndex)` [8](sensorarrayAdsCache.c#L8) | 按 VOLT/RES 模式和测点下标定位输入档位缓存项。 / Locate the input-profile cache entry for the VOLT/RES mode and cell index. | `sensorarrayAdsProfileCacheInvalidateCell` / `sensorarrayAdsProfileCacheGet` |
| `static uint32_t sensorarrayAdsAbsoluteDelta32(int32_t left, int32_t right)` [25](sensorarrayAdsCache.c#L25) | 以宽整数计算两个原始码之间的绝对差。 / Compute the absolute difference of two codes using widened arithmetic. | `sensorarrayAdsValueCacheShouldFastAccept` / `sensorarrayAdsValueCacheObserve` |
| `void sensorarrayAdsProfileCacheInit(sensorarrayAdsProfileCache_t *cache)` [34](sensorarrayAdsCache.c#L34) | 建立逐模式测点档位缓存并初始化世代。 / Initialise per-mode cell profiles and their generation. | `sensorarrayAdsMatrixEngineInit` / `sensorarrayMeasurementSelfTestRun` |
| `void sensorarrayAdsProfileCacheInvalidate(sensorarrayAdsProfileCache_t *cache)` [43](sensorarrayAdsCache.c#L43) | 使所有已记忆的输入档位失效并推进缓存世代。 / Invalidate stored input profiles and advance the cache generation. | `sensorarrayAdsMatrixRefreshRegisterShadow` / `sensorarrayAdsMatrixEngineSetCalibration` |
| `void sensorarrayAdsProfileCacheInvalidateCell(sensorarrayAdsProfileCache_t *cache, sensorarrayMeasurementMode_t mode, uint8_t cellIndex)` [54](sensorarrayAdsCache.c#L54) | 只撤销指定模式与测点的旧 PGA/bypass 档。 / Invalidate the remembered PGA/bypass profile for one mode and cell. | 独立接口 / Standalone entry |
| `bool sensorarrayAdsProfileCacheGet(sensorarrayAdsProfileCache_t *cache, sensorarrayMeasurementMode_t mode, uint8_t cellIndex, sensorarrayAdsCellProfile_t *outProfile)` [66](sensorarrayAdsCache.c#L66) | 读取该世代内已知的测点输入档，将档位输出给调用者。 / Read the cell profile for this generation and supply its input setting. | `sensorarrayAdsMatrixMeasureCellImpl` / `sensorarrayMeasurementSelfTestRun` |
| `bool sensorarrayAdsProfileCacheStore(sensorarrayAdsProfileCache_t *cache, sensorarrayMeasurementMode_t mode, uint8_t cellIndex, sensorarrayAdsInputMode_t inputMode, uint8_t gain, uint32_t frameSequence)` [79](sensorarrayAdsCache.c#L79) | 校验模式、测点与输入设置后记住已验证的 PGA/bypass 档。 / Validate the mode, cell and input setting and store the accepted PGA/bypass profile. | `sensorarrayAdsMatrixMeasureCellImpl` / `sensorarrayMeasurementSelfTestRun` |
| `void sensorarrayAdsProfileCacheNoteSuccess(sensorarrayAdsProfileCache_t *cache, sensorarrayMeasurementMode_t mode, uint8_t cellIndex, uint32_t frameSequence)` [103](sensorarrayAdsCache.c#L103) | 登记当前档位成功使用并更新该测点的状态计数。 / Record a successful use of the profile and update its cell counters. | `sensorarrayAdsMatrixMeasureCellImpl` |
| `void sensorarrayAdsProfileCacheNoteFailure(sensorarrayAdsProfileCache_t *cache, sensorarrayMeasurementMode_t mode, uint8_t cellIndex, bool overrange, uint8_t invalidationThreshold)` [120](sensorarrayAdsCache.c#L120) | 更新失败/超量程记录，到达条件时使该测点档位失效。 / Account for profile failure/overrange and invalidate the cell profile when its threshold is met. | `sensorarrayAdsMatrixMeasureCellImpl` |
| `void sensorarrayAdsValueCacheInit(sensorarrayAdsValueCache_t *cache)` [145](sensorarrayAdsCache.c#L145) | 初始化逐模式测点数值、噪声与稳定次数的存储。 / Initialise per-mode cell value, noise and stability history. | `sensorarrayAdsMatrixEngineInit` |
| `void sensorarrayAdsValueCacheInvalidate(sensorarrayAdsValueCache_t *cache)` [154](sensorarrayAdsCache.c#L154) | 撤销旧配置下保存的 raw/node/value 与噪声状态。 / Invalidate raw/node/value and noise history from an earlier configuration. | `sensorarrayAdsMatrixRefreshRegisterShadow` / `sensorarrayAdsMatrixEngineSetCalibration` |
| `sensorarrayAdsCellValueCache_t *sensorarrayAdsValueCacheGet(sensorarrayAdsValueCache_t *cache, sensorarrayMeasurementMode_t mode, uint8_t cellIndex)` [165](sensorarrayAdsCache.c#L165) | 按当前模式和测点返回对应数值与噪声缓存项。 / Return the value/noise cache entry for the selected mode and cell. | `sensorarrayAdsValueCacheAgeFrame` / `sensorarrayAdsMatrixMeasureCellImpl` |
| `uint32_t sensorarrayAdsValueCacheThresholdRaw(const sensorarrayAdsCellValueCache_t *cell, uint32_t minimumRawThreshold, uint32_t noiseMultiplier)` [182](sensorarrayAdsCache.c#L182) | 由最小阈值和该测点噪声估计计算原始码判定阈值。 / Calculate the raw-code threshold from the minimum floor and the cell's noise estimate. | `sensorarrayAdsValueCacheShouldFastAccept` |
| `bool sensorarrayAdsValueCacheShouldFastAccept(const sensorarrayAdsCellValueCache_t *cell, int32_t currentRaw, uint32_t minimumRawThreshold, uint32_t noiseMultiplier, uint32_t minimumStableStreak, bool profileValid, bool statusClean, bool freshConversion, bool precisionFrame, bool transitionSensitive)` [198](sensorarrayAdsCache.c#L198) | 核对缓存、新鲜转换、状态、精度帧和噪声条件后决定是否可单样本接纳。 / Gate single-sample acceptance on cache history, fresh status, frame type and noise bounds. | `sensorarrayAdsMatrixReadAdaptive` |
| `void sensorarrayAdsValueCacheObserve(sensorarrayAdsCellValueCache_t *cell, int32_t raw, int32_t nodeUv, int64_t valueFixed, uint32_t spreadRaw, uint32_t frameSequence, bool fullSample)` [219](sensorarrayAdsCache.c#L219) | 把本次合格转换的原始码、节点、定点数值及极差计入缓存。 / Update the cell history with accepted raw code, node voltage, fixed-point value and spread. | `sensorarrayAdsMatrixMeasureCellImpl` |
| `void sensorarrayAdsValueCacheAgeFrame(sensorarrayAdsValueCache_t *cache, sensorarrayMeasurementMode_t mode, uint8_t activeCellCount)` [259](sensorarrayAdsCache.c#L259) | 在帧边界递增已缓存测点的龄期。 / Advance the age of cached cells at a frame boundary. | 独立接口 / Standalone entry |
| `void sensorarrayAdsRailFingerprintInit(sensorarrayAdsRailFingerprint_t *fingerprint)` [277](sensorarrayAdsCache.c#L277) | 清空前次供轨/参考指纹与失效历史。 / Initialise the rail/reference fingerprint and invalidation history. | `sensorarrayAdsMatrixEngineInit` / `sensorarrayAdsMatrixEngineSetCalibration` |
| `static sensorarrayAdsRailInvalidationReason_t sensorarrayAdsRailFingerprintReason(const sensorarrayAdsRailFingerprint_t *fingerprint, bool railValid, bool railStatusGood, int32_t railUv, int32_t avddUv, int32_t avssUv, uint8_t railSource, uint8_t referenceSource, uint32_t calibrationGeneration, uint32_t invalidationThresholdUv, bool adsReset)` [286](sensorarrayAdsCache.c#L286) | 判定参考、rail 来源/幅值、校准世代或复位是否要求失效。 / Determine whether reference, rail source/span, calibration generation or reset invalidates the fingerprint. | `sensorarrayAdsRailFingerprintUpdate` |
| `bool sensorarrayAdsRailFingerprintUpdate(sensorarrayAdsRailFingerprint_t *fingerprint, bool railValid, bool railStatusGood, int32_t railUv, int32_t avddUv, int32_t avssUv, uint8_t railSource, uint8_t referenceSource, uint32_t calibrationGeneration, uint32_t invalidationThresholdUv, uint32_t hysteresisUv, bool adsReset, sensorarrayAdsRailInvalidationReason_t *outReason)` [336](sensorarrayAdsCache.c#L336) | 评估实际变化与滞回边界，记录新指纹并返回是否应清缓存。 / Apply hysteresis to observed changes, update the fingerprint and signal cache invalidation. | `sensorarrayAdsMatrixEngineReadFrame` |
| `const char *sensorarrayAdsRailInvalidationReasonName(sensorarrayAdsRailInvalidationReason_t reason)` [395](sensorarrayAdsCache.c#L395) | 返回供轨失效原因的显示名称。 / Return the display name for rail invalidation reason. | 独立接口 / Standalone entry |
| `void sensorarrayAdsRegisterCacheInit(sensorarrayAdsRegisterCache_t *cache)` [413](sensorarrayAdsCache.c#L413) | 初始化寄存器影像、世代和 ADC 所有权状态。 / Initialise register shadow entries, generation and ADC ownership state. | `sensorarrayAdsMatrixEngineInit` |
| `void sensorarrayAdsRegisterCacheInvalidate(sensorarrayAdsRegisterCache_t *cache)` [422](sensorarrayAdsCache.c#L422) | 使各寄存器已知/核验状态失效，要求后续操作重新建立证据。 / Invalidate register knowledge/verification so later operations must re-establish evidence. | `sensorarrayAdsRestoreState` / `sensorarrayAdsReadBatteryTransaction` |
| `bool sensorarrayAdsRegisterCacheNeedsWrite(sensorarrayAdsRegisterCache_t *cache, sensorarrayAdsRegisterId_t registerId, uint8_t value)` [435](sensorarrayAdsCache.c#L435) | 比较期望字节与已知影像，决定是否必须发出物理写入。 / Compare the requested byte with known shadow state to decide whether hardware needs a write. | `sensorarrayAdsMatrixApplyInputProfile` |
| `void sensorarrayAdsRegisterCacheNoteWrite(sensorarrayAdsRegisterCache_t *cache, sensorarrayAdsRegisterId_t registerId, uint8_t value, bool verified)` [450](sensorarrayAdsCache.c#L450) | 记录实际写入寄存器的值及核验标记。 / Record a hardware register write and its verification flag. | `sensorarrayAdsReadBatteryTransaction` / `sensorarrayAdsMatrixApplyInputProfile` |
| `uint8_t sensorarrayAdsRegisterCompareMask(sensorarrayAdsRegisterId_t registerId)` [465](sensorarrayAdsCache.c#L465) | 返回对应 ADS 寄存器允许比较的有效位掩码。 / Return the meaningful readback mask for the ADS register. | `sensorarrayAdsRegisterCacheNoteReadback` / `sensorarrayAdsRegisterHealthCompare` |
| `bool sensorarrayAdsRegisterHealthDue(uint32_t frameCount, uint32_t interval, bool mode0Valid)` [475](sensorarrayAdsCache.c#L475) | 根据引擎累计帧数与配置周期判断是否到达健康读回帧。 / Determine the health-read frame from cumulative engine count and configured interval. | `sensorarrayAdsMatrixEngineReadFrame` |
| `bool sensorarrayAdsRegisterCacheNoteReadback(sensorarrayAdsRegisterCache_t *cache, sensorarrayAdsRegisterId_t registerId, uint8_t value)` [481](sensorarrayAdsCache.c#L481) | 用有意义的位核对读取值，再更新已知/核验状态或记下不一致。 / Compare meaningful readback bits and update knowledge/verification or record the mismatch. | `sensorarrayAdsPopulateRestoredRegisterCache` / `sensorarrayAdsMatrixNoteRegisterSnapshot` |
| `bool sensorarrayAdsRegisterCacheAcquire(sensorarrayAdsRegisterCache_t *cache, sensorarrayAdsOwner_t owner)` [501](sensorarrayAdsCache.c#L501) | 仅从可用状态取得指明的 ADS owner。 / Acquire the specified ADS owner only from an admissible state. | `sensorarrayAdsGapRefreshRailAtBoundary` / `sensorarrayAdsReadBatteryTransaction` |
| `bool sensorarrayAdsRegisterCacheRelease(sensorarrayAdsRegisterCache_t *cache, sensorarrayAdsOwner_t owner)` [512](sensorarrayAdsCache.c#L512) | 仅由匹配的当前 owner 归还寄存器/ADC 所有权。 / Release register/ADC ownership only for the matching current owner. | `sensorarrayAdsGapRefreshRailAtBoundary` / `sensorarrayAdsReadBatteryTransaction` |
| `const char *sensorarrayAdsOwnerName(sensorarrayAdsOwner_t owner)` [522](sensorarrayAdsCache.c#L522) | 返回所有者的显示名称。 / Return the display name for owner. | `sensorarrayAdsRunJob` / `sensorarrayAdsMatrixCaptureFault` |
| `sensorarrayAdsHealthComparison_t sensorarrayAdsRegisterHealthCompare(const sensorarrayAdsRegisterCache_t *cache, const uint8_t actual[SENSORARRAY_ADS_REGISTER_COUNT], uint8_t expectedPower, uint8_t expectedRefmux, uint8_t expectedMode0, uint8_t expectedMode1)` [535](sensorarrayAdsCache.c#L535) | 按寄存器掩码区分 RESET、POWER、REFMUX 与模式不匹配并返回首个比较结果。 / Return the first RESET, POWER, REFMUX or mode discrepancy using register-specific masks. | `sensorarrayAdsMatrixRefreshRegisterShadow` |

### sensorarrayAdsFault.c

| C 调用签名 / C signature | 行为及返回 / Behaviour and return | 调用位置 / Called by |
| --- | --- | --- |
| `const char *sensorarrayAdsFaultStageName(sensorarrayAdsFaultStage_t stage)` [46](sensorarrayAdsFault.c#L46) | 返回故障阶段的显示名称。 / Return the display name for fault stage. | `sensorarrayAdsFaultFormat` |
| `const char *sensorarrayAdsFaultOutcomeName(sensorarrayAdsFaultOutcome_t outcome)` [54](sensorarrayAdsFault.c#L54) | 返回故障终态的显示名称。 / Return the display name for fault outcome. | `sensorarrayAdsFaultFormat` |
| `static const char *sensorarrayAdsFaultText(const char *text)` [62](sensorarrayAdsFault.c#L62) | 针对空指针或空字段选择诊断文本占位值。 / Supply the diagnostic fallback when the text field is unavailable. | `sensorarrayAdsFaultFormat` |
| `size_t sensorarrayAdsFaultFormat(const sensorarrayAdsFaultEvent_t *event, char *buffer, size_t bufferSize)` [67](sensorarrayAdsFault.c#L67) | 按有效标志输出首次故障与明确的 railSpanUv/avddUv/avssUv、来源和启动身份。 / Format the first failure with explicit railSpanUv/avddUv/avssUv, source and boot identity according to known flags. | `sensorarrayAdsFaultEmit` |
| `size_t sensorarrayAdsFaultEmit(const sensorarrayAdsFaultEvent_t *event, uint64_t nowUs, sensorarrayAdsFaultSink_t sink, void *context)` [155](sensorarrayAdsFault.c#L155) | 按阶段与限流策略格式化 ADSFAULT，并调用给定的输出回调。 / Apply stage/rate policy, format ADSFAULT and invoke the supplied output callback. | `sensorarrayAdsRunJob` / `sensorarrayEmitAdsFault` |

### sensorarrayAdsGap.c

| C 调用签名 / C signature | 行为及返回 / Behaviour and return | 调用位置 / Called by |
| --- | --- | --- |
| `void sensorarrayAdsGapBindBootIdentity(uint32_t bootId, uint32_t bootCount)` [246](sensorarrayAdsGap.c#L246) | 把已发布的启动标识与辅助事务关联，用于故障和诊断归属。 / Bind the published boot identity to auxiliary transactions for fault and diagnostic attribution. | `sensorarrayInitFrontends` |
| `uint32_t sensorarrayAdsGapTransactionGeneration(void)` [252](sensorarrayAdsGap.c#L252) | 读取辅助事务的世代供矩阵缓存识别边界变更。 / Read the auxiliary transaction generation so matrix caches can detect boundary changes. | `sensorarrayAdsMatrixEngineReadFrame` |
| `void sensorarrayAdsGapCopyTransactionEvidence(sensorarrayAdsTransactionEvidence_t *rail, sensorarrayAdsTransactionEvidence_t *battery, sensorarrayAdsTransactionEvidence_t *check)` [257](sensorarrayAdsGap.c#L257) | 复制最近 rail、电池或 ADSCHK 事务及结束时间等证据。 / Copy recent rail, battery or ADSCHK transaction evidence including completion times. | `sensorarrayAdsMatrixCaptureFault` / `sensorarrayAdsMatrixGetRail` |
| `void sensorarrayAdsGapNoteFirstValidFrame(uint64_t frameEndUs)` [267](sensorarrayAdsGap.c#L267) | 把恢复后第一张有效矩阵帧的时间记录到待完成的辅助事务证据。 / Attach the first valid post-restore frame timestamp to the pending auxiliary evidence. | `sensorarrayAdsMatrixEngineReadFrame` |
| `static void sensorarrayAdsTransactionBegin(sensorarrayAdsTransactionEvidence_t *t)` [280](sensorarrayAdsGap.c#L280) | 记录当前辅助事务的 ID 和起始时间。 / Record the auxiliary transaction ID and start time. | `sensorarrayAdsGapRefreshRailAtBoundary` / `sensorarrayAdsReadBatteryTransaction` |
| `static void sensorarrayAdsTransactionEnd(sensorarrayAdsTransactionEvidence_t *t, esp_err_t originalError, esp_err_t cleanupError)` [287](sensorarrayAdsGap.c#L287) | 登记结束时间、原始与清理错误，并更新可查询的事务证据。 / Commit completion time, original/cleanup errors and queryable transaction evidence. | `sensorarrayAdsGapRefreshRailAtBoundary` / `sensorarrayAdsReadBatteryTransaction` |
| `static void sensorarrayAdsFaultSink(const char *line, size_t length, void *context)` [297](sensorarrayAdsGap.c#L297) | 将已格式化的辅助事务故障投递给本地事件/诊断通道。 / Route a formatted auxiliary-transaction failure to the local diagnostic path. | `sensorarrayAdsRunJob` |
| `static bool sensorarrayAdsSnapshotFromRegisterCache(const sensorarrayAdsRegisterCache_t *cache, const ads126xAdcHandle_t *ads, ads126xAdc1RegisterSnapshot_t *outSnapshot)` [312](sensorarrayAdsGap.c#L312) | 将已核验的 register shadow 投影成可保存的 ADC1 寄存器快照。 / Build an ADC1 register snapshot from the verified register shadow. | `sensorarrayAdsBatterySaveVerifiedState` / `sensorarrayAdsGapRunTimingProbeKind` |
| `static uint8_t sensorarrayAdsBatteryRegisterMask(uint8_t reg)` [352](sensorarrayAdsGap.c#L352) | 选择电池保存/恢复时该寄存器需要比较的位。 / Select the register bits used in battery save/restore comparisons. | `sensorarrayAdsBatterySnapshotsMatch` / `sensorarrayAdsBatterySaveVerifiedState` |
| `static bool sensorarrayAdsBatterySnapshotsMatch(const ads126xAdc1RegisterSnapshot_t *expected, const ads126xAdc1RegisterSnapshot_t *actual, sensorarrayAdsBatteryCompare_t *comparison)` [360](sensorarrayAdsGap.c#L360) | 按电池事务掩码核对两份快照，报告首个不一致。 / Compare two snapshots using battery-transaction masks and report the first mismatch. | `sensorarrayAdsBatterySaveVerifiedState` / `sensorarrayAdsVerifyRailMonitorProfile` |
| `static esp_err_t sensorarrayAdsBatterySaveVerifiedState(ads126xAdcHandle_t *ads, const sensorarrayAdsRegisterCache_t *cache, ads126xAdc1RegisterSnapshot_t *saved, sensorarrayAdsBatteryDiag_t *diag)` [392](sensorarrayAdsGap.c#L392) | 为电池事务取得已核验的 ADC1 状态基线。 / Obtain a verified ADC1 baseline for the battery transaction. `ESP_ERR_INVALID_RESPONSE` | `sensorarrayAdsReadBatteryTransaction` |
| `static bool sensorarrayAdsCoreRegistersMatch(const ads126xAdc1RegisterSnapshot_t *expected, uint8_t power, uint8_t mode2, uint8_t inpmux, uint8_t refmux)` [432](sensorarrayAdsGap.c#L432) | 按事务设置比较 ADC1 核心寄存器和输入/参考选择。 / Compare the ADC1 core registers and input/reference selection against transaction settings. | `sensorarrayAdsRestoreBatteryCoreState` |
| `static esp_err_t sensorarrayAdsRestoreBatteryCoreState(ads126xAdcHandle_t *ads, const ads126xAdc1RegisterSnapshot_t *saved, uint32_t savedVrefUv, bool adc1WasRunning, uint8_t transactionPower, uint8_t transactionMode2, uint8_t transactionInpmux, uint8_t transactionRefmux, uint8_t *outPowerAfter)` [445](sensorarrayAdsGap.c#L445) | 恢复电池采样前的 ADC1 配置并核对实际读回。 / Restore the pre-battery ADC1 configuration and verify readback. `ESP_ERR_INVALID_ARG` | `sensorarrayAdsReadBatteryTransaction` |
| `static bool sensorarrayAdsRailUsableAsReference(int64_t railUv)` [516](sensorarrayAdsGap.c#L516) | 检查供轨跨度是否在用作参考的允许工作范围内。 / Check whether a rail span lies inside the allowed reference operating range. | `sensorarrayAdsUpdateRail` / `sensorarrayAdsGapSetExternalRailCalibration` |
| `static bool sensorarrayAdsRailLastGoodFresh(uint32_t frameSequence)` [522](sensorarrayAdsGap.c#L522) | 用帧号差核对最近一次合格供轨测量的帧龄上限。 / Check the last good rail sample against the maximum age in frames. | `sensorarrayAdsUpdateRail` |
| `static bool sensorarrayAdsInternalReferenceIsUnclamped(void)` [532](sensorarrayAdsGap.c#L532) | 检查允许使用内部参考所需的板级开关状态。 / Check the board switch state required to use the internal reference. | `sensorarrayAdsReadRailMonitorUv` / `sensorarrayAdsSelectJob` |
| `static bool sensorarrayAdsUpdateRail(int32_t rawCode, int32_t monitorUv, uint32_t frameSequence)` [545](sensorarrayAdsGap.c#L545) | 检查原始跨度、运行范围和最近有效值，分别更新诊断与可用状态。 / Evaluate raw span, operating range and last good evidence, updating diagnostic and usable rail states separately. | `sensorarrayAdsGapRefreshRailAtBoundary` |
| `static bool sensorarrayAdsCalculateAincomGnd(int32_t *outUv)` [629](sensorarrayAdsGap.c#L629) | 从有效 rail/VBIAS 证据计算 AINCOM 对地电压。 / Calculate AINCOM-to-ground voltage from usable rail and VBIAS evidence. | `sensorarrayAdsUpdateBatteryValidity` |
| `static uint16_t sensorarrayAdsChipValue(const ads126xAdcHandle_t *ads)` [661](sensorarrayAdsGap.c#L661) | 将实际读取的 ADS 型号映射为诊断编号。 / Map the observed ADS variant to its diagnostic identifier. | `sensorarrayAdsBootCalibration` / `sensorarrayAdsGapRunActiveCheck` |
| `static int32_t sensorarrayAdsMedian3I32(int32_t a, int32_t b, int32_t c)` [672](sensorarrayAdsGap.c#L672) | 返回三个有符号样本的中位数。 / Return the median of three signed samples. | `sensorarrayAdsReadBatteryTransaction` |
| `static void sensorarrayAdsRecordFreshSample(const sensorarrayAdsFreshSample_t *sample, esp_err_t err)` [683](sensorarrayAdsGap.c#L683) | 保存 ADC1 最新样本的 raw/status 及关联的转换观测。 / Record raw/status and associated conversion evidence for a fresh ADC1 sample. | `sensorarrayAdsReadFreshAdc1` / `sensorarrayAdsReadRailFreshAdc1` |
| `static esp_err_t sensorarrayAdsReadFreshAdc1(ads126xAdcHandle_t *ads, uint32_t timeoutUs, sensorarrayAdsFreshSample_t *outSample)` [707](sensorarrayAdsGap.c#L707) | 等待本事务配置后的新 DRDY，读取 ADC1 并检查状态。 / Wait for DRDY after the transaction configuration and read/status-check the ADC1 sample. `ESP_ERR_INVALID_ARG` | `sensorarrayAdsReadBatteryFreshAdc1` / `sensorarrayAdsReadDma` |
| `static esp_err_t sensorarrayAdsReadBatteryFreshAdc1(ads126xAdcHandle_t *ads, uint32_t timeoutUs, sensorarrayAdsFreshSample_t *outSample, uint8_t *inOutRetryCount)` [753](sensorarrayAdsGap.c#L753) | 读取本配置下新的 ADC1 电池样本，仅在支持的瞬态错误时有限重试。 / Acquire a new ADC1 battery sample for this configuration with bounded eligible transient retries. `ESP_ERR_INVALID_ARG` | `sensorarrayAdsReadBatteryTransaction` |
| `static const char *sensorarrayAdsBatteryReasonName(sensorarrayBatteryInvalidReason_t reason)` [789](sensorarrayAdsGap.c#L789) | 返回电池原因的显示名称。 / Return the display name for battery reason. | `sensorarrayAdsRunJob` / `sensorarrayAdsGapFormatBattery` |
| `static sensorarrayBatteryInvalidReason_t sensorarrayAdsBatteryMathReason(sensorarrayAdsBatteryMathError_t error)` [844](sensorarrayAdsGap.c#L844) | 将电池定点换算的失败类别转为报告所用原因。 / Translate battery conversion failures to diagnostic reason codes. | `sensorarrayAdsUpdateBatteryValidity` |
| `static void sensorarrayAdsUpdateZero(int32_t zeroUv, uint32_t frameSequence)` [884](sensorarrayAdsGap.c#L884) | 登记零差分测量或其错误，更新下一事务可用的零点证据。 / Record zero-differential measurement or failure for later zero-reference evidence. | `sensorarrayAdsRunJob` |
| `static void sensorarrayAdsUpdateBatteryValidity(esp_err_t adcReadErr, const sensorarrayAdsBatteryDiag_t *diag, uint32_t frameSequence)` [902](sensorarrayAdsGap.c#L902) | 按实测结果更新电池有效性、连续计数与最近成功值。 / Update battery validity, counters and last-good evidence from the measured outcome. | `sensorarrayAdsGapInit` / `sensorarrayAdsRunJob` |
| `static esp_err_t sensorarrayAdsSaveState(ads126xAdcHandle_t *ads, sensorarrayAdsSavedState_t *saved)` [971](sensorarrayAdsGap.c#L971) | 在辅助事务改变 ADC1 前读取并保存寄存器、参考与运行状态。 / Capture registers, reference and run state before an auxiliary transaction changes ADC1. `ESP_ERR_INVALID_ARG` | `sensorarrayAdsReadRailMonitorUv` / `sensorarrayAdsRunJob` |
| `static esp_err_t sensorarrayAdsWriteCalibration(ads126xAdcHandle_t *ads, const uint8_t offsetCal[3], const uint8_t fullScaleCal[3])` [1004](sensorarrayAdsGap.c#L1004) | 将保存的偏移及满量程校准寄存器字节写回 ADC1。 / Write saved offset and full-scale calibration bytes back to ADC1. `ESP_ERR_INVALID_ARG` | `sensorarrayAdsReadRailMonitorUv` / `sensorarrayAdsBootCalibration` |
| `static esp_err_t sensorarrayAdsRestoreState(ads126xAdcHandle_t *ads, const sensorarrayAdsSavedState_t *saved)` [1024](sensorarrayAdsGap.c#L1024) | 恢复已保存的硬件寄存器、参考和转换运行状态，核对恢复结果。 / Restore saved registers, reference and conversion run state and check restoration outcome. `ESP_ERR_INVALID_ARG` | `sensorarrayAdsReadRailMonitorUv` / `sensorarrayAdsRunJob` |
| `static esp_err_t sensorarrayAdsVerifyRailMonitorProfile(ads126xAdcHandle_t *ads, const sensorarrayAdsSavedState_t *saved, uint8_t mode2, sensorarrayAdsRailDiag_t *diag)` [1072](sensorarrayAdsGap.c#L1072) | 读取必要寄存器并核对供轨监测所需的电源和输入/参考设置。 / Read and compare the power, input and reference configuration needed for a rail measurement. `ESP_ERR_INVALID_RESPONSE` | `sensorarrayAdsReadRailMonitorUv` |
| `static esp_err_t sensorarrayAdsReadRailFreshAdc1(ads126xAdcHandle_t *ads, uint8_t *retryBudget, sensorarrayAdsRailDiag_t *diag, sensorarrayAdsFreshSample_t *outSample)` [1112](sensorarrayAdsGap.c#L1112) | 等待新供轨转换并执行状态/完整性检查，保留有界重试。 / Wait for a fresh rail conversion, check status/integrity and retain bounded retry evidence. | `sensorarrayAdsReadRailMonitorUv` |
| `static esp_err_t sensorarrayAdsReadRailMonitorUv(sensorarrayState_t *state, int32_t *outRaw, int32_t *outMonitorUv, sensorarrayAdsRailDiag_t *outDiag)` [1139](sensorarrayAdsGap.c#L1139) | 在允许的状态下读取内部供轨跨度，单位 µV，并返回真实驱动错误。 / Measure the internal rail span in µV in an admitted state and preserve the driver error. `ESP_ERR_INVALID_ARG` `ESP_ERR_INVALID_STATE` | `sensorarrayAdsGapRefreshRailAtBoundary` |
| `static esp_err_t sensorarrayAdsBootCalibration(sensorarrayState_t *state)` [1302](sensorarrayAdsGap.c#L1302) | 在启动阶段执行允许的辅助校准并保存供轨、零点及恢复观测。 / Run admitted boot calibration and retain rail, zero and restore observations. | `sensorarrayAdsGapInit` |
| `esp_err_t sensorarrayAdsGapInit(sensorarrayState_t *state)` [1389](sensorarrayAdsGap.c#L1389) | 初始化辅助状态、未知值、调度周期与 rail/电池快照。 / Initialise auxiliary state, unknown sentinels, schedule and rail/battery snapshots. `ESP_ERR_NOT_SUPPORTED` | `sensorarrayInitFrontends` |
| `void sensorarrayAdsGapBindRegisterCache(sensorarrayAdsRegisterCache_t *registerCache)` [1477](sensorarrayAdsGap.c#L1477) | 把矩阵引擎持有的寄存器影像交给同一 ADS 辅助事务路径同步。 / Bind the matrix-owned register shadow to the auxiliary save/restore path. | `sensorarrayInitFrontends` |
| `esp_err_t sensorarrayAdsGapRefreshRailAtBoundary(sensorarrayState_t *state, uint32_t frameSequence)` [1482](sensorarrayAdsGap.c#L1482) | 在已准入的帧边界执行 rail 监测，跟踪所有权、读数来源与恢复结果。 / Run admitted boundary rail monitoring and retain ownership, source and restoration evidence. `ESP_ERR_INVALID_STATE` | `sensorarrayAdsRunJob` / `sensorarrayAdsMatrixGetRail` |
| `esp_err_t sensorarrayAdsGapSetExternalRailCalibration(int32_t avddUv, int32_t avssUv, uint32_t frameSequence)` [1582](sensorarrayAdsGap.c#L1582) | 验证独立实测的对地 AVDD 和 AVSS 并作为外部来源保存在 RAM。 / Validate independently measured ground-referenced AVDD and AVSS and retain them in RAM as external calibration. `ESP_ERR_INVALID_ARG` | `sensorarrayApplyPendingCommands` |
| `bool sensorarrayAdsGapHasExternalRailCalibration(void)` [1628](sensorarrayAdsGap.c#L1628) | 检查当前有效的 rail 来源是否为独立外部校准。 / Check whether an active rail calibration has external provenance. | 独立接口 / Standalone entry |
| `bool sensorarrayAdsGapCopyRailSplit(uint32_t frameSequence, uint32_t maximumAgeFrames, sensorarrayAdsRailSplit_t *outRail)` [1635](sensorarrayAdsGap.c#L1635) | 复制满足来源、合法性与龄期要求的 rail 快照。 / Copy a rail split meeting source, operating-range and age requirements. | `sensorarrayAdsReadBatteryTransaction` / `sensorarrayAdsGapFormatRail` |
| `static sensorarrayAdsJob_t sensorarrayAdsSelectJob(uint32_t frameSequence)` [1690](sensorarrayAdsGap.c#L1690) | 综合强制请求、帧周期、参考钳位和外部校准来源，选取下个可运行作业。 / Choose eligible work from forced requests, frame intervals, reference-clamp state and external calibration provenance. | `sensorarrayAdsGapRunPendingAtBoundary` |
| `static esp_err_t sensorarrayAdsReadDma(sensorarrayState_t *state, uint8_t muxp, uint8_t muxn, int32_t *outRaw, int32_t *outUv)` [1736](sensorarrayAdsGap.c#L1736) | 从已准备的 ADS DMA 转换读取结果，保留驱动错误与状态。 / Read a prepared ADS conversion through DMA, retaining driver errors and status. | `sensorarrayAdsReadPreferred` |
| `static esp_err_t sensorarrayAdsReadPreferred(sensorarrayState_t *state, uint8_t muxp, uint8_t muxn, int32_t *outRaw, int32_t *outUv)` [1771](sensorarrayAdsGap.c#L1771) | 选择配置的 ADC 读取方式并把结果传给辅助事务。 / Use the configured ADC read path and return its result to the auxiliary transaction. | `sensorarrayAdsRunJob` |
| `static void sensorarrayAdsPrintBatteryDiag(const sensorarrayAdsBatteryDiag_t *diag)` [1782](sensorarrayAdsGap.c#L1782) | 输出一次电池分压、输入和参考源的诊断观测。 / Report battery-divider, input and reference observations for one diagnostic transaction. | `sensorarrayAdsRunJob` |
| `static esp_err_t sensorarrayAdsReadBatteryTransaction(sensorarrayState_t *state, uint32_t frameSequence, int32_t *outRaw, int32_t *outUv, sensorarrayAdsBatteryDiag_t *outDiag)` [1822](sensorarrayAdsGap.c#L1822) | 保存 ADC1 核心状态，切到 AIN8/AINCOM 与所需偏置，读取新样本并验证恢复。 / Save ADC1 core state, select AIN8/AINCOM and bias, acquire fresh samples and verify restoration. `ESP_ERR_INVALID_STATE` | `sensorarrayAdsRunJob` |
| `static void sensorarrayAdsLogBatteryRegisters(uint32_t seq, const char *phase, const ads126xAdc1RegisterSnapshot_t *r, uint32_t generation, bool known)` [2204](sensorarrayAdsGap.c#L2204) | 记录电池事务保存、配置与恢复三阶段的寄存器证据。 / Report saved, configured and restored battery-transaction register evidence. | `sensorarrayAdsRunJob` |
| `static const char *sensorarrayAdsBatteryRestoreName(const sensorarrayAdsBatteryDiag_t *diag)` [2219](sensorarrayAdsGap.c#L2219) | 返回电池恢复的显示名称。 / Return the display name for battery restore. | `sensorarrayAdsLogBatteryEvidence` / `sensorarrayAdsRunJob` |
| `static void sensorarrayAdsLogBatteryEvidence(uint32_t seq, const sensorarrayAdsBatteryDiag_t *diag)` [2225](sensorarrayAdsGap.c#L2225) | 输出电池采样时间、原始码、状态与有效性原因。 / Report battery sample timings, raw codes, status and validity reason. | `sensorarrayAdsRunJob` |
| `static esp_err_t sensorarrayAdsRunJob(sensorarrayState_t *state, sensorarrayAdsJob_t job, uint32_t frameSequence, bool boundaryFallback)` [2250](sensorarrayAdsGap.c#L2250) | 对选中的零点、rail 或电池作业管理 ADS 所有权、执行与恢复收尾。 / Manage ADS ownership, work and restore cleanup for the selected zero, rail or battery job. `ESP_ERR_INVALID_STATE` | `sensorarrayAdsGapRunPendingAtBoundary` / `sensorarrayAdsGapRunBatteryAtBoundary` |
| `void sensorarrayAdsGapRequestCalibration(bool requestZero, bool requestRail)` [2414](sensorarrayAdsGap.c#L2414) | 登记零点/rail 重新校准请求，由后续合格帧边界消费。 / Post a zero/rail recalibration request for an eligible frame boundary. | `sensorarrayApplyPendingCommands` |
| `void sensorarrayAdsGapRequestBatteryDiagnostic(void)` [2428](sensorarrayAdsGap.c#L2428) | 登记详细电池诊断请求供下次边界执行。 / Request detailed battery diagnostics at a later frame boundary. | `sensorarrayApplyPendingCommands` |
| `void sensorarrayAdsGapRequestBatteryNow(void)` [2436](sensorarrayAdsGap.c#L2436) | 让电池调度器在下一可用完整帧边界运行。 / Mark the battery scheduler due for the next eligible frame boundary. | `sensorarrayApplyPendingCommands` |
| `bool sensorarrayAdsGapConfigureBatteryPeriod(bool enabled, uint32_t periodMs)` [2443](sensorarrayAdsGap.c#L2443) | 设置墙钟调度周期并据此更新下次到期时间。 / Set the wall-clock battery period and recompute its next deadline. | `sensorarrayApplyPendingCommands` |
| `void sensorarrayAdsGapSetMode(sensorarrayAdsGapMode_t mode)` [2467](sensorarrayAdsGap.c#L2467) | 验证新的辅助模式并更新允许的边界作业选择。 / Validate the auxiliary mode and update boundary job selection. | `sensorarrayApplyPendingCommands` |
| `sensorarrayAdsGapMode_t sensorarrayAdsGapGetMode(void)` [2478](sensorarrayAdsGap.c#L2478) | 返回当前允许的辅助作业模式。 / Return the currently selected auxiliary job mode. | `sensorarrayRuntimeQueryCommand` |
| `const char *sensorarrayAdsGapModeName(sensorarrayAdsGapMode_t mode)` [2483](sensorarrayAdsGap.c#L2483) | 返回模式的显示名称。 / Return the display name for mode. | `sensorarrayAdsGapFormatAds` / `sensorarrayRuntimeQueryCommand` |
| `const char *sensorarrayAdsRailStatusName(sensorarrayAdsRailStatus_t status)` [2500](sensorarrayAdsGap.c#L2500) | 返回供轨状态的显示名称。 / Return the display name for rail status. | `sensorarrayAdsGapFormatBattery` / `sensorarrayAdsGapFormatRail` |
| `const char *sensorarrayAdsRailSourceName(sensorarrayAdsRailSource_t source)` [2513](sensorarrayAdsGap.c#L2513) | 返回供轨来源的显示名称。 / Return the display name for rail source. | `sensorarrayAdsRunJob` / `sensorarrayAdsGapFormatRail` |
| `void sensorarrayAdsGapTryRun(sensorarrayState_t *state, uint64_t expectedFdcReadyUs, uint32_t frameSequence, uint8_t row)` [2526](sensorarrayAdsGap.c#L2526) | 保留历史行间调用入口并直接返回；完整辅助事务只由帧边界函数执行。 / Retain the legacy row-gap entry as a no-op. Boundary functions perform complete auxiliary transactions. | 独立接口 / Standalone entry |
| `esp_err_t sensorarrayAdsGapRunPendingAtBoundary(sensorarrayState_t *state, uint32_t frameSequence)` [2536](sensorarrayAdsGap.c#L2536) | 在当前帧边界领取并执行待处理的辅助作业。 / Admit pending auxiliary work at the current frame boundary. `ESP_ERR_INVALID_STATE` | `sensorarrayRunMainLoop` |
| `esp_err_t sensorarrayAdsGapRunBatteryAtBoundary(sensorarrayState_t *state, uint32_t frameSequence, bool capacitanceMode, uint32_t *outDurationUs, bool *outRan)` [2560](sensorarrayAdsGap.c#L2560) | 按调度器准入执行边界电池事务，单独记录测量有效性与配置恢复。 / Run a scheduler-admitted boundary battery transaction and report measurement validity separately from configuration restoration. `ESP_ERR_INVALID_STATE` | `sensorarrayRunBatteryBoundary` |
| `bool sensorarrayAdsGapConsumeRestoreFailure(void)` [2623](sensorarrayAdsGap.c#L2623) | 领取锁存的恢复失败供采集协调器处置。 / Consume the latched restore failure for the capture coordinator. | `sensorarrayRunAdsTimingCommand` / `sensorarrayRunBatteryBoundary` |
| `void sensorarrayAdsGapCopySnapshot(sensorarrayAdsGapSnapshot_t *outSnapshot, uint32_t frameSequence)` [2630](sensorarrayAdsGap.c#L2630) | 把已发布的电池、rail 与 ADC 辅助事务状态复制到调用方结构。 / Copy published battery, rail and ADC auxiliary state into caller storage. | `sensorarrayAdsGapCopyRailSplit` / `sensorarrayAdsGapFormatBattery` |
| `size_t sensorarrayAdsGapFormatBattery(char *buffer, size_t bufferSize, uint32_t frameSequence)` [2685](sensorarrayAdsGap.c#L2685) | 将电池有效性、来源、龄期、调度与恢复证据格式化为 BAT 回复。 / Format battery validity, source, age, scheduling and restoration evidence for BAT. | `sensorarrayRuntimeQueryCommand` |
| `size_t sensorarrayAdsGapFormatRail(char *buffer, size_t bufferSize, uint32_t frameSequence)` [2743](sensorarrayAdsGap.c#L2743) | 将供轨数值、来源、已知状态和龄期格式化为 RAIL 回复。 / Format rail values, provenance, known status and age for RAIL. | `sensorarrayRuntimeQueryCommand` |
| `size_t sensorarrayAdsGapFormatAds(char *buffer, size_t bufferSize)` [2784](sensorarrayAdsGap.c#L2784) | 将现有芯片、ADC 和寄存器观测格式化为只读 ADS 状态。 / Format existing device, ADC and register observations as read-only ADS status. | `sensorarrayRuntimeQueryCommand` |
| `static void sensorarrayAdsPopulateRestoredRegisterCache(sensorarrayAdsRegisterCache_t *cache, const ads126xAdc1RegisterSnapshot_t *snapshot, uint32_t vrefUv, bool adc1Running)` [2811](sensorarrayAdsGap.c#L2811) | 用已核验恢复的寄存器快照重建共享 register shadow。 / Republish a verified restored register snapshot into the shared shadow. | `sensorarrayAdsRestoreState` / `sensorarrayAdsReadBatteryTransaction` |
| `esp_err_t sensorarrayAdsGapRunActiveCheck(sensorarrayState_t *state, sensorarrayAdsRegisterCache_t *registerCache, uint32_t requestId, uint32_t sampleCount, sensorarrayAdsActiveCheckResult_t *outResult)` [2854](sensorarrayAdsGap.c#L2854) | 在单一边界事务内读寄存器和新 DRDY 数据、恢复 ADC1 并保存 ADSCHK 诊断。 / Read registers and fresh DRDY data, restore ADC1 and retain ADSCHK diagnostics within one boundary transaction. `ESP_ERR_INVALID_ARG` `ESP_ERR_INVALID_STATE` | `sensorarrayRunPendingAdsCheck` / `sensorarrayRunAdsRestoreRecoveryAttempt` |
| `esp_err_t sensorarrayAdsGapRunTimingProbe(sensorarrayState_t *state, sensorarrayAdsRegisterCache_t *registerCache, uint32_t requestId, uint32_t frameSequence, sensorarrayAdsTimingProbeResult_t *r)` [3036](sensorarrayAdsGap.c#L3036) | 在诊断所有权下运行默认类型的有限 ADS 时序探测。 / Run the default bounded ADS timing probe under diagnostic ownership. | 独立接口 / Standalone entry |
| `esp_err_t sensorarrayAdsGapRunTimingProbeKind(sensorarrayState_t *state, sensorarrayAdsRegisterCache_t *registerCache, uint32_t requestId, uint32_t frameSequence, sensorarrayAdsTimingProbeKind_t kind, sensorarrayAdsTimingProbeResult_t *r)` [3044](sensorarrayAdsGap.c#L3044) | 领取 ADS 调试所有权执行请求的时序探测，返回工作和恢复证据。 / Acquire the ADS diagnostic owner for the requested timing probe and return work and restore evidence. `ESP_ERR_INVALID_ARG` | `sensorarrayAdsGapRunTimingProbe` / `sensorarrayRunAdsTimingCommand` |
| `static const char *sensorarrayAdsFilterName(uint8_t mode1)` [3216](sensorarrayAdsGap.c#L3216) | 返回滤波器的显示名称。 / Return the display name for filter. | `sensorarrayAdsGapFormatActiveCheck` |
| `static uint32_t sensorarrayAdsDelayCodeUs(uint8_t mode0)` [3228](sensorarrayAdsGap.c#L3228) | 由 MODE0 延迟选择值解码对应的 µs。 / Decode a MODE0 delay selection into microseconds. | `sensorarrayAdsGapFormatActiveCheck` |
| `static const char *sensorarrayAdsReferenceName(uint8_t refmux)` [3238](sensorarrayAdsGap.c#L3238) | 返回参考的显示名称。 / Return the display name for reference. | `sensorarrayAdsGapFormatActiveCheck` |
| `size_t sensorarrayAdsGapFormatActiveCheck(const sensorarrayAdsActiveCheckResult_t *result, char *buffer, size_t bufferSize)` [3246](sensorarrayAdsGap.c#L3246) | 格式化上一次 ADSCHK 的新鲜样本、错误及恢复结果。 / Format fresh-sample, failure and restore outcomes of the last ADSCHK. | `sensorarrayRunPendingAdsCheck` |
| `size_t sensorarrayAdsGapFormatActiveCheckTiming(const sensorarrayAdsActiveCheckResult_t *result, char *buffer, size_t bufferSize)` [3297](sensorarrayAdsGap.c#L3297) | 输出上次主动检查的配置、DRDY、读取与恢复时间。 / Format configuration, DRDY, read and restore timings from the last active check. | `sensorarrayRunPendingAdsCheck` |

### sensorarrayAdsMath.c

| C 调用签名 / C signature | 行为及返回 / Behaviour and return | 调用位置 / Called by |
| --- | --- | --- |
| `static int32_t sensorarrayAdsMathMedianInPlace(int32_t *values, size_t count)` [5](sensorarrayAdsMath.c#L5) | 原位排序后返回样本中位值，输入数组会被改写。 / Sort samples in place and return their median, modifying the input array. | `sensorarrayAdsMathSamplesStable` |
| `bool sensorarrayAdsMathSplitRail(const sensorarrayAdsRailInput_t *input, sensorarrayAdsRailSplit_t *outSplit)` [23](sensorarrayAdsMath.c#L23) | 用已知跨度和板级比例估算两条对地供轨，保留估算来源。 / Estimate ground-referenced rails from span and board ratio and retain estimated provenance. | `sensorarrayAdsGapCopyRailSplit` / `sensorarrayMeasurementSelfTestRun` |
| `bool sensorarrayAdsMathVoltageFromDifferential(int32_t differentialUv, const sensorarrayAdsRailSplit_t *rail, const sensorarrayAdsVoltageLimits_t *limits, int32_t *outNodeUv, sensorarrayCellError_t *outError)` [59](sensorarrayAdsMath.c#L59) | 把相对 AINCOM 的差分码电压转换为对地节点电压并检查范围。 / Convert an AINCOM-referenced differential voltage to ground-referenced node voltage and check limits. | `sensorarrayAdsMatrixEngineReadFrame` / `sensorarrayMeasurementSelfTestRun` |
| `sensorarrayAdsResistanceResult_t sensorarrayAdsMathResistanceDivider(int32_t matrixReferenceUv, int32_t nodeUv, int32_t avssUv, const sensorarrayAdsResistanceConfig_t *config)` [91](sensorarrayAdsMath.c#L91) | 按参考激励、Vnode、AVSS 与 Rref 求 Rx，分类开路、短路和范围错误。 / Solve Rx from excitation, Vnode, AVSS and Rref and classify open, short and range failures. | `sensorarrayAdsMatrixEngineReadFrame` / `sensorarrayMeasurementSelfTestRun` |
| `bool sensorarrayAdsMathSamplesStable(const int32_t *samples, size_t sampleCount, uint32_t maximumSpreadUv, int32_t *outMedianUv)` [156](sensorarrayAdsMath.c#L156) | 求中位数及峰峰差并与允许 spread 阈值比较。 / Find the median and peak-to-peak spread and test the permitted stability bound. | `sensorarrayAdsMathConfirmOpenSet` / `sensorarrayAdsMatrixReadAdaptive` |
| `bool sensorarrayAdsMathConfigurationGenerationCurrent(uint32_t sampleConfigurationGeneration, uint32_t currentConfigurationGeneration)` [181](sensorarrayAdsMath.c#L181) | 比较转换所属与当前配置世代以拒绝旧样本。 / Compare sample generation with the current configuration to exclude stale conversions. | `sensorarrayAdsMatrixReadAdaptive` / `sensorarrayAdsMatrixConfirmHighZ` |
| `bool sensorarrayAdsMathHighZOpenCandidate(const sensorarrayAdsHighZCandidateInput_t *input)` [189](sensorarrayAdsMath.c#L189) | 从节点样本和当前参考判断是否需高阻开路复核。 / Use node samples and reference evidence to decide whether high-impedance open confirmation is needed. | `sensorarrayAdsMatrixMeasureCellImpl` |
| `sensorarrayAdsOpenConfirm_t sensorarrayAdsMathConfirmOpenSet(const int32_t *nodeUvSamples, size_t sampleCount, int32_t avssUv, const sensorarrayAdsResistanceConfig_t *config, uint32_t maximumSpreadUv, int32_t *outMedianNodeUv)` [204](sensorarrayAdsMath.c#L204) | 用一组重新建立后的节点样本确认或撤销高阻开路分类。 / Confirm or reject a high-impedance open using re-established node samples. | `sensorarrayAdsMatrixConfirmHighZ` |
| `uint8_t sensorarrayAdsMathCombineStatusBytes(uint8_t primaryStatus, const uint8_t *additionalStatus, size_t additionalCount)` [243](sensorarrayAdsMath.c#L243) | 把主样本和附加样本的 ADC 状态位按位汇总。 / Combine ADC status bits across the primary and additional samples. | `sensorarrayAdsMatrixConfirmHighZ` |
| `uint64_t sensorarrayAdsMathHighZLatchUpdate(uint64_t latchMask, size_t cellIndex, sensorarrayAdsOpenSemantic_t openSemantic)` [257](sensorarrayAdsMath.c#L257) | 按本轮开路语义更新该测点的高阻锁存位。 / Update the cell's high-impedance latch bit from this round's open classification. | `sensorarrayAdsMatrixMeasureCellImpl` |
| `sensorarrayCellError_t sensorarrayAdsMathClassifySampleHealth(const sensorarrayAdsSampleHealth_t *health)` [275](sensorarrayAdsMath.c#L275) | 按新鲜、传输和状态/饱和条件选择该样本的首个质量错误。 / Classify the sample's first quality failure from freshness, transfer, status and saturation. | `sensorarrayMeasurementSelfTestRun` |
| `sensorarrayAdsBatteryMathResult_t sensorarrayAdsMathBatteryVoltage(const sensorarrayAdsBatteryMathInput_t *input)` [311](sensorarrayAdsMath.c#L311) | 将 AIN8 差分、AINCOM 及电池分压/校准换算成具有原因码的结果。 / Convert AIN8 differential input and AINCOM through divider/calibration into a classified battery result. | `sensorarrayAdsUpdateBatteryValidity` |

### sensorarrayAdsMatrix.c

| C 调用签名 / C signature | 行为及返回 / Behaviour and return | 调用位置 / Called by |
| --- | --- | --- |
| `bool sensorarrayAdsMatrixCalibrationValid(const sensorarrayAdsMatrixCalibration_t *calibration)` [76](sensorarrayAdsMatrix.c#L76) | 检查校准版本、参考电阻、激励跨度以及全局/分组/逐点偏置合法范围。 / Check calibration version, reference resistor, excitation span and global/bank/cell offset ranges. | `sensorarrayAdsMatrixEngineSetCalibration` / `sensorarrayAdsMatrixEngineGetCalibration` |
| `static uint32_t sensorarrayAdsMatrixConfigurationGeneration(const sensorarrayAdsMatrixEngine_t *engine)` [105](sensorarrayAdsMatrix.c#L105) | 合并矩阵校准、profile、寄存器与数值缓存世代，并避免返回零世代。 / Combine calibration, profile, register and value-cache generations, reserving zero. | `sensorarrayAdsMatrixReadFreshAfter` / `sensorarrayAdsMatrixCaptureFault` |
| `static void sensorarrayAdsMatrixSetCellError(sensorarrayFrame_t *frame, size_t index, sensorarrayCellError_t error, bool fresh)` [118](sensorarrayAdsMatrix.c#L118) | 清除指定测点的可用值并按原因设置错误、开短路、饱和或不稳掩码。 / Invalidate the cell value and set the relevant error/open/short/saturation/instability masks. | `sensorarrayAdsMatrixEngineReadFrame` |
| `static void sensorarrayAdsMatrixSetCellValid(sensorarrayFrame_t *frame, size_t index, int64_t valueFixed, const sensorarrayAdsMatrixCellResult_t *result)` [148](sensorarrayAdsMatrix.c#L148) | 存放合格的定点值、原始码和节点电压并更新该点的新鲜/有效位。 / Store the accepted fixed-point value, raw code and node voltage and update valid/fresh bits. | `sensorarrayAdsMatrixEngineReadFrame` |
| `static esp_err_t sensorarrayAdsMatrixReadFreshAfter(sensorarrayAdsMatrixEngine_t *engine, uint32_t startGeneration, uint32_t configurationGeneration, sensorarrayMeasurementPayload_t *telemetry, sensorarrayAdsMatrixSample_t *outSample)` [172](sensorarrayAdsMatrix.c#L172) | 只接纳给定 DRDY 及配置世代后的 ADC1 数据，并记录有界重试。 / Admit ADC1 data only after the specified DRDY/configuration generation and record bounded retries. `ESP_ERR_INVALID_ARG` | `sensorarrayAdsMatrixReadAdaptive` / `sensorarrayAdsMatrixConfirmHighZ` |
| `static void sensorarrayAdsMatrixCaptureFault(sensorarrayAdsMatrixEngine_t *engine, sensorarrayAdsFaultStage_t stage, esp_err_t originalError, uint8_t reg, uint8_t expected, uint8_t actual, uint8_t mask)` [228](sensorarrayAdsMatrix.c#L228) | 在硬件收尾前保存本帧第一个故障阶段、期望/实测值及比较掩码。 / Retain the frame's first fault stage, expected/observed values and mask before cleanup. | `sensorarrayAdsMatrixEngineRecordMixedBoundaryFault` / `sensorarrayAdsMatrixNoteRegisterSnapshot` |
| `void sensorarrayAdsMatrixEngineRecordMixedBoundaryFault(sensorarrayAdsMatrixEngine_t *engine, uint32_t sequence, esp_err_t originalError, esp_err_t cleanupError)` [256](sensorarrayAdsMatrix.c#L256) | 在 MIXED 前置检查失败时保存本序号的首次失败及清理结果，记录尚未开始转换。 / Record preflight/cleanup errors against this mixed-frame sequence before any conversion starts. | `sensorarrayMixedRowEngineReadFrame` |
| `bool sensorarrayAdsMatrixEngineCopyFaultContext(const sensorarrayAdsMatrixEngine_t *engine, sensorarrayAdsMatrixFaultContext_t *outContext)` [280](sensorarrayAdsMatrix.c#L280) | 复制当前帧在 SAFE/恢复收尾之前保留的故障现场。 / Copy the frame's pre-cleanup fault evidence retained before SAFE/recovery. | `sensorarrayEmitAdsFault` |
| `static bool sensorarrayAdsMatrixNoteRegisterSnapshot(sensorarrayAdsMatrixEngine_t *engine, const ads126xAdc1RegisterSnapshot_t *snapshot)` [291](sensorarrayAdsMatrix.c#L291) | 按缓存的已知/核验状态记录读回字节并保留缓存不一致证据。 / Record returned register bytes against known/verified shadow state and retain mismatch evidence. | `sensorarrayAdsMatrixRefreshRegisterShadow` |
| `static esp_err_t sensorarrayAdsMatrixRefreshRegisterShadow(sensorarrayAdsMatrixEngine_t *engine, sensorarrayMeasurementPayload_t *telemetry)` [329](sensorarrayAdsMatrix.c#L329) | 读取两份 ADS 寄存器快照，区分读取不稳、复位和配置不匹配，成功后更新 shadow。 / Read ADS register snapshots, distinguish unstable reads, reset and mismatches, and update the shadow on success. `ESP_ERR_INVALID_ARG` | `sensorarrayAdsMatrixEngineReadFrame` |
| `static esp_err_t sensorarrayAdsMatrixApplyInputProfile(sensorarrayAdsMatrixEngine_t *engine, uint8_t muxp, uint8_t muxn, sensorarrayAdsInputMode_t inputMode, uint8_t gain, sensorarrayMeasurementPayload_t *telemetry, bool forceRoute, uint32_t *outStartGeneration)` [418](sensorarrayAdsMatrix.c#L418) | 应用测点的 MUX 与 PGA/bypass，跟踪实际变化并建立下一新转换的世代边界。 / Apply cell MUX and PGA/bypass, track physical changes and establish the next fresh conversion boundary. `ESP_ERR_INVALID_ARG` | `sensorarrayAdsMatrixConfirmHighZ` / `sensorarrayAdsMatrixMeasureCellImpl` |
| `static uint32_t sensorarrayAdsMatrixRawSpread(const int32_t values[3])` [502](sensorarrayAdsMatrix.c#L502) | 计算本组原始样本的最大值与最小值之差。 / Calculate maximum minus minimum across the raw sample group. | `sensorarrayAdsMatrixReadAdaptive` / `sensorarrayAdsMatrixConfirmHighZ` |
| `static esp_err_t sensorarrayAdsMatrixReadAdaptive(sensorarrayAdsMatrixEngine_t *engine, uint32_t firstGeneration, sensorarrayAdsCellValueCache_t *valueCache, bool profileValid, bool pgaBypassed, bool precisionFrame, bool transitionSensitive, sensorarrayMeasurementPayload_t *telemetry, sensorarrayAdsMatrixCellResult_t *outResult)` [518](sensorarrayAdsMatrix.c#L518) | 按缓存、精度帧与噪声条件选择单样本或多次新样本，并报告极差。 / Choose a single sample or further fresh conversions from cache, precision-frame and noise conditions, reporting spread. `ESP_ERR_INVALID_ARG` `ESP_ERR_INVALID_RESPONSE` | `sensorarrayAdsMatrixMeasureCellImpl` |
| `static sensorarrayCellError_t sensorarrayAdsMatrixIoError(esp_err_t err)` [624](sensorarrayAdsMatrix.c#L624) | 将 ADC 驱动错误映射为逐点测量原因码。 / Map an ADC I/O error to a per-cell measurement reason. | `sensorarrayAdsMatrixConfirmHighZ` / `sensorarrayAdsMatrixMeasureCellImpl` |
| `static esp_err_t sensorarrayAdsMatrixConfirmHighZ(sensorarrayAdsMatrixEngine_t *engine, const sensorarrayAdsRailSplit_t *rail, uint8_t muxp, uint8_t muxn, sensorarrayAdsInputMode_t inputMode, uint8_t gain, sensorarrayMeasurementPayload_t *telemetry, sensorarrayAdsMatrixCellResult_t *sample)` [635](sensorarrayAdsMatrix.c#L635) | 对疑似高阻开路测点执行有限的换路复测并给出分类结果。 / Perform bounded re-routing/reacquisition to classify a suspected high-impedance open. `ESP_ERR_INVALID_ARG` `ESP_ERR_INVALID_RESPONSE` | `sensorarrayAdsMatrixMeasureCellImpl` |
| `static esp_err_t sensorarrayAdsMatrixMeasureCellImpl(sensorarrayAdsMatrixEngine_t *engine, const sensorarrayAdsRailSplit_t *rail, uint8_t row, uint8_t dLine, bool precisionFrame, bool transitionSensitive, sensorarrayMeasurementPayload_t *telemetry, sensorarrayAdsMatrixCellResult_t *outResult)` [787](sensorarrayAdsMatrix.c#L787) | 应用指定输入档、核对 ADC 状态与供轨边界、换算一个 VOLT/RES 测点。 / Apply input profile and ADC/rail checks and convert one VOLT or RES cell. `ESP_ERR_INVALID_ARG` `ESP_ERR_NOT_FOUND` `ESP_ERR_INVALID_RESPONSE` `ESP_ERR_INVALID_SIZE` `ESP_ERR_TIMEOUT` | `sensorarrayAdsMatrixMeasureCell` |
| `static esp_err_t sensorarrayAdsMatrixMeasureCell(sensorarrayAdsMatrixEngine_t *engine, const sensorarrayAdsRailSplit_t *rail, uint8_t row, uint8_t dLine, bool precisionFrame, bool transitionSensitive, sensorarrayMeasurementPayload_t *telemetry, sensorarrayAdsMatrixCellResult_t *outResult)` [1312](sensorarrayAdsMatrix.c#L1312) | 采一组 S/D 坐标，按当前模式设置输入与 PGA，并仅将通过校验的新读数交给缓存。 / Acquire one S/D cell under the selected mode/input/PGA and admit only verified fresh data to caches. `ESP_ERR_INVALID_ARG` | `sensorarrayAdsMatrixEngineReadFrame` |
| `static void sensorarrayAdsMatrixLogCrcEvidence(sensorarrayAdsMatrixEngine_t *engine, uint32_t sequence, esp_err_t frameError)` [1378](sensorarrayAdsMatrix.c#L1378) | 记录 CRC 被拒时的校验字节、读取路径和有界重试证据。 / Report CRC bytes, read path and bounded retry evidence for a rejected conversion. | `sensorarrayAdsMatrixEngineReadFrame` |
| `static esp_err_t sensorarrayAdsMatrixGetRail(sensorarrayAdsMatrixEngine_t *engine, sensorarrayAdsRailSplit_t *outRail, sensorarrayAdsGapSnapshot_t *outGap, esp_err_t *outCleanupError, bool allowRefresh)` [1436](sensorarrayAdsMatrix.c#L1436) | 为当前帧取得可用的供轨快照并根据调用策略处理刷新许可。 / Obtain usable rail evidence for the current frame subject to the caller's refresh policy. `ESP_ERR_INVALID_STATE` | `sensorarrayAdsMatrixEngineReadFrame` |
| `static void sensorarrayAdsMatrixFinishFrame(sensorarrayFrame_t *frame, uint8_t rows, int64_t startUs, bool coherent)` [1486](sensorarrayAdsMatrix.c#L1486) | 收束活动、已采得及有效掩码，未采测点保留无效原因并记录结束时间。 / Finalise active, acquired and valid masks, preserve reasons on unacquired cells and record end time. | `sensorarrayAdsMatrixEngineReadFrame` |
| `static bool sensorarrayAdsMatrixCompleteCrcDisposition(sensorarrayAdsMatrixEngine_t *engine, sensorarrayFrame_t *frame, uint64_t crcRejectedMask, bool coherent)` [1570](sensorarrayAdsMatrix.c#L1570) | 根据 CRC 拒绝掩码与帧一致性确定哪些测点可发布为有效。 / Resolve frame coherence and publishable cells using the CRC rejection mask. | `sensorarrayAdsMatrixEngineReadFrame` |
| `esp_err_t sensorarrayAdsMatrixEngineInit(sensorarrayAdsMatrixEngine_t *engine, sensorarrayState_t *state)` [1599](sensorarrayAdsMatrix.c#L1599) | 绑定硬件状态并建立矩阵校准、缓存及初始 NONE 模式。 / Bind hardware state and initialise matrix calibration, caches and NONE mode. `ESP_ERR_INVALID_ARG` | `sensorarrayInitFrontends` |
| `esp_err_t sensorarrayAdsMatrixEngineSetCalibration(sensorarrayAdsMatrixEngine_t *engine, const sensorarrayAdsMatrixCalibration_t *calibration)` [1625](sensorarrayAdsMatrix.c#L1625) | 仅在引擎模式 NONE 接受经版本和范围核验的矩阵校准，并使旧缓存失效。 / Apply version/range-checked matrix calibration only in NONE mode and invalidate stale cache state. `ESP_ERR_INVALID_ARG` `ESP_ERR_INVALID_STATE` | `sensorarrayApplyPendingCommands` / `sensorarrayLoadPersistedMatrixCalibration` |
| `bool sensorarrayAdsMatrixEngineGetCalibration(const sensorarrayAdsMatrixEngine_t *engine, sensorarrayAdsMatrixCalibration_t *outCalibration)` [1647](sensorarrayAdsMatrix.c#L1647) | 把当前引擎校准复制给调用者并返回其合法性。 / Copy engine calibration to the caller and return whether it is valid. | `sensorarrayApplyPendingCommands` |
| `void sensorarrayAdsMatrixEngineBindRouteController(sensorarrayAdsMatrixEngine_t *engine, sensorarrayRouteController_t *routeController)` [1658](sensorarrayAdsMatrix.c#L1658) | 将该引擎与同一板级状态的路由控制器关联。 / Associate the engine with the route controller for its board state. | `sensorarrayInitFrontends` |
| `esp_err_t sensorarrayAdsMatrixEngineSetMode(sensorarrayAdsMatrixEngine_t *engine, sensorarrayMeasurementMode_t mode)` [1667](sensorarrayAdsMatrix.c#L1667) | 在模式改变时更新引擎与缓存状态，保存模式入口时的累计 frameCount。 / Change engine mode and cache state and remember the cumulative frameCount on entry. `ESP_ERR_INVALID_ARG` | `sensorarrayMixedRowEngineReadFrame` / `sensorarrayBeginAdsRestoreRecovery` |
| `void sensorarrayAdsMatrixEngineSetFrameSequenceHint(sensorarrayAdsMatrixEngine_t *engine, uint32_t sequence)` [1688](sensorarrayAdsMatrix.c#L1688) | 将协调器分配的序号提前交给采集/前置故障路径。 / Supply the coordinator's sequence for acquisition and preflight fault attribution. | `sensorarrayRunOneFrame` |
| `void sensorarrayAdsMatrixEngineInvalidateGainCache(sensorarrayAdsMatrixEngine_t *engine)` [1696](sensorarrayAdsMatrix.c#L1696) | 撤销各模式、测点缓存的自动 PGA 档位。 / Invalidate automatic PGA profiles cached by mode and cell. | `sensorarrayApplyMeasurementMode` / `sensorarrayApplyPendingCommands` |
| `void sensorarrayAdsMatrixEngineInvalidateCaches(sensorarrayAdsMatrixEngine_t *engine)` [1706](sensorarrayAdsMatrix.c#L1706) | 撤销共享寄存器、逐点档位与数值等缓存的旧证据。 / Invalidate stale register, cell-profile and value cache evidence. | `sensorarrayAdsMatrixConfirmHighZ` / `sensorarrayAdsMatrixMeasureCellImpl` |
| `sensorarrayAdsRegisterCache_t *sensorarrayAdsMatrixEngineRegisterCache(sensorarrayAdsMatrixEngine_t *engine)` [1719](sensorarrayAdsMatrix.c#L1719) | 取得此引擎持有的寄存器缓存，供辅助事务同步。 / Return the engine-owned register cache shared with auxiliary restoration. | `sensorarrayAdsFaultFillContext` / `sensorarrayRunAdsTimingCommand` |
| `esp_err_t sensorarrayAdsMatrixEngineReadFrame(sensorarrayAdsMatrixEngine_t *engine, const sensorarrayScanPlan_t *plan, sensorarrayFrame_t *frame)` [1725](sensorarrayAdsMatrix.c#L1725) | 建立本帧上下文，核对健康周期与 rail，按行读新转换，汇总逐点状态、CRC 和帧掩码。 / Establish frame context, admit scheduled health checks and rail evidence, acquire fresh conversions row by row and finalise masks, CRC and per-cell status. `ESP_ERR_INVALID_STATE` | `sensorarrayMixedRowEngineReadFrame` / `sensorarrayRunOneFrame` |

### sensorarrayAdsMeasure.inc

| C 调用签名 / C signature | 行为及返回 / Behaviour and return | 调用位置 / Called by |
| --- | --- | --- |
| `esp_err_t sensorarrayMeasureSetSelaPath(sensorarrayState_t *state, sensorarraySelaRoute_t selaRoute, uint32_t settleDelayMs, const char *stage, const char *label)` [2](sensorarrayAdsMeasure.inc#L2) | 把选定的逻辑路径写入 SELA 并返回板级驱动结果。 / Apply the requested logical route to SELA and return board-driver status. `ESP_ERR_INVALID_STATE` `ESP_ERR_INVALID_ARG` | `sensorarrayMeasureApplyRouteLevels` / `sensorarrayMeasureApplyRoute` |
| `esp_err_t sensorarrayMeasureApplyFdcCapAdsPolicy(sensorarrayState_t *state, const char *stage, const char *mode, const char *reason)` [20](sensorarrayAdsMeasure.inc#L20) | 在 CAP 路径上落实所需的 ADS 参考及偏置策略。 / Apply the ADS reference and bias policy for CAP routing. `ESP_ERR_INVALID_ARG` | `sensorarrayMeasureApplyRouteLevels` / `sensorarrayMeasureApplyRoute` |
| `esp_err_t sensorarrayMeasureApplyRouteLevels(sensorarrayState_t *state, uint8_t sColumn, uint8_t dLine, sensorarrayRoutePathKind_t path, tmux1108Source_t swSource, sensorarraySelaRoute_t selaRoute, bool selBLevel, uint32_t delayAfterRowMs, uint32_t delayAfterSelAMs, uint32_t delayAfterSelBMs, uint32_t delayAfterSwMs, const char *label)` [50](sensorarrayAdsMeasure.inc#L50) | 按选择的 SELA、SELB、SW 和延迟切换模拟路径。 / Set SELA, SELB and SW with the requested route-settling delays. `ESP_ERR_INVALID_STATE` `ESP_ERR_INVALID_ARG` | `sensorarrayMeasureApplyFdcCellRoute` |
| `esp_err_t sensorarrayMeasureApplyRoute(sensorarrayState_t *state, uint8_t sColumn, uint8_t dLine, sensorarrayPath_t path, tmux1108Source_t swSource, const char **outMapLabel)` [204](sensorarrayAdsMeasure.inc#L204) | 以 S/D 和路径类型查板图，再应用控制和建立时序。 / Look up S/D and route type in the board map and apply its control and settling sequence. `ESP_ERR_INVALID_STATE` `ESP_ERR_INVALID_ARG` `ESP_ERR_NOT_SUPPORTED` | 独立接口 / Standalone entry |
| `esp_err_t sensorarrayMeasureApplyFdcCellRoute(sensorarrayState_t *state, const sensorarrayFdcCellTarget_t *target, const char *reason)` [406](sensorarrayAdsMeasure.inc#L406) | 定位一个指定 FDC 测点并连接其 CAP 路径。 / Resolve a physical FDC cell and select its CAP route. `ESP_ERR_INVALID_ARG` `ESP_ERR_INVALID_STATE` | `sensorarrayFdcSweepCaptureCandidate` / `sensorarrayFdcSweepPrepareCellPath` |
| `esp_err_t sensorarrayMeasureReadAdsRawWithRetry(sensorarrayState_t *state, int32_t *outRaw, uint8_t retryCount, bool *outTimedOut, uint8_t *outStatusByte)` [462](sensorarrayAdsMeasure.inc#L462) | 读取 ADS 原始样本，按给定次数重试并输出超时和状态字节。 / Read an ADS raw sample with bounded retries, exposing timeout and status byte. `ESP_ERR_INVALID_ARG` `ESP_FAIL` | 独立接口 / Standalone entry |
| `esp_err_t sensorarrayMeasureReadAdsPairUv(sensorarrayState_t *state, const sensorarrayAdsReadPolicy_t *policy, uint8_t muxp, uint8_t muxn, bool discardFirst, int32_t *outRaw, int32_t *outUv, uint8_t *outStatusByte)` [502](sensorarrayAdsMeasure.inc#L502) | 选择给定正负输入并按策略读取一组差分电压，单位 µV。 / Read the specified positive/negative ADS input pair in µV under its read policy. `ESP_ERR_INVALID_STATE` | `sensorarrayMeasureReadAdsUv` |
| `esp_err_t sensorarrayMeasureReadAdsUv(sensorarrayState_t *state, const sensorarrayAdsReadPolicy_t *policy, uint8_t dLine, bool discardFirst, int32_t *outRaw, int32_t *outUv)` [619](sensorarrayAdsMeasure.inc#L619) | 将所选通道对应的 ADS 测量交给差分读数路径并返回 µV。 / Route the selected ADS channel to the differential reader and return µV. `ESP_ERR_INVALID_STATE` `ESP_ERR_INVALID_ARG` | 独立接口 / Standalone entry |
| `sensorarrayResConvertResult_t sensorarrayMeasureTryResistanceMohm(int32_t uv, int32_t *outMohm)` [639](sensorarrayAdsMeasure.inc#L639) | 按配置的分压模型计算电阻或说明本次换算无法得到有效 mΩ。 / Compute resistance from the configured divider model or mark mΩ conversion unavailable. | `sensorarrayMeasureDividerModelStatus` |
| `const char *sensorarrayMeasureDividerModelStatus(int32_t uv, int32_t *outMohm, bool *outHaveMohm)` [661](sensorarrayAdsMeasure.inc#L661) | 返回当前分压模型可用性的诊断状态。 / Return the diagnostic availability state of the divider model. | 独立接口 / Standalone entry |

### sensorarrayAdsRegisters.inc

| C 调用签名 / C signature | 行为及返回 / Behaviour and return | 调用位置 / Called by |
| --- | --- | --- |
| `esp_err_t sensorarrayMeasureAdsReadRegister(sensorarrayState_t *state, uint8_t reg, uint8_t *outValue)` [2](sensorarrayAdsRegisters.inc#L2) | 读取一个指定 ADS 寄存器并把原始字节交还调用者。 / Read one ADS register and return its raw byte. `ESP_ERR_INVALID_STATE` | `sensorarrayMeasureReadAdsKeyRegisterSnapshot` |
| `esp_err_t sensorarrayMeasureReadAdsKeyRegisterSnapshot(sensorarrayState_t *state, sensorarrayAdsRegSnapshot_t *outSnapshot)` [10](sensorarrayAdsRegisters.inc#L10) | 顺序读取诊断需要的关键寄存器并保留一次快照。 / Read key diagnostic registers into one snapshot. `ESP_ERR_INVALID_ARG` | `sensorarrayMeasureDumpAdsKeyRegisters` |
| `esp_err_t sensorarrayMeasureDumpAdsKeyRegisters(sensorarrayState_t *state, const char *stage)` [37](sensorarrayAdsRegisters.inc#L37) | 输出关键寄存器快照及其操作阶段。 / Report a key-register snapshot with its operation stage. | 独立接口 / Standalone entry |

### sensorarrayAdsTimingProbe.c

| C 调用签名 / C signature | 行为及返回 / Behaviour and return | 调用位置 / Called by |
| --- | --- | --- |
| `const char *sensorarrayAdsTimingProbeKindName(sensorarrayAdsTimingProbeKind_t kind)` [15](sensorarrayAdsTimingProbe.c#L15) | 返回探测类型的显示名称。 / Return the display name for probe kind. | `sensorarrayAdsTimingProbeFormatSummary` / `sensorarrayAdsTimingProbeFormatGroup` |
| `static uint32_t sensorarrayAdsTimingU32(uint64_t value)` [21](sensorarrayAdsTimingProbe.c#L21) | 把 64 位计数或耗时饱和压入 uint32_t 使用的诊断字段。 / Saturate a wide counter or duration into a uint32 diagnostic field. | `sensorarrayAdsTimingRead` / `sensorarrayAdsTimingProbeRun` |
| `static bool sensorarrayAdsTimingRegistersMatch(const ads126xAdc1RegisterSnapshot_t *a, const ads126xAdc1RegisterSnapshot_t *b)` [26](sensorarrayAdsTimingProbe.c#L26) | 核对探测中实际寄存器与预期快照是否一致。 / Compare registers observed during the probe against the expected snapshot. | `sensorarrayAdsTimingVerify` |
| `static esp_err_t sensorarrayAdsTimingVerify(ads126xAdcHandle_t *ads, const ads126xAdc1RegisterSnapshot_t *expected)` [40](sensorarrayAdsTimingProbe.c#L40) | 重新读取关键 ADC1 配置以确认本探测阶段的寄存器状态。 / Read back the ADC1 register settings required by the probe phase. | `sensorarrayAdsTimingProbeRun` |
| `static void sensorarrayAdsTimingError(sensorarrayAdsTimingProbeResult_t *r, sensorarrayAdsTimingGroup_t *g, esp_err_t error, const char *reason, bool isolate)` [53](sensorarrayAdsTimingProbe.c#L53) | 保留首错，更新探测与当前分组的终态错误、原因及隔离需求。 / Preserve the first error and update probe/group terminal status, reason and isolation requirement. | `sensorarrayAdsTimingRead` / `sensorarrayAdsTimingProbeRun` |
| `static void sensorarrayAdsTimingRecord(sensorarrayAdsTimingProbeResult_t *r, unsigned group, const sensorarrayAdsTimingRecord_t *record, uint32_t *categories)` [63](sensorarrayAdsTimingProbe.c#L63) | 将有限数量的逐尝试采样证据保存进探测结果。 / Retain a bounded per-attempt sample trace in the probe result. | `sensorarrayAdsTimingRead` |
| `static void sensorarrayAdsTimingNoteCrc(sensorarrayAdsTimingReadStats_t *stats, sensorarrayAdsTimingRecord_t *record, const ads126xAdcCrcFault_t *fault)` [88](sensorarrayAdsTimingProbe.c#L88) | 把驱动 CRC、来源或奇偶相关证据计入当前读统计。 / Account for driver CRC and read-source/parity evidence in read statistics. | `sensorarrayAdsTimingRead` |
| `static bool sensorarrayAdsTimingBadStatus(uint8_t status, int32_t raw)` [111](sensorarrayAdsTimingProbe.c#L111) | 由状态字与原始码检查此读数是否含不良转换证据。 / Check the status byte and raw code for conversion faults. | `sensorarrayAdsTimingRead` |
| `static esp_err_t sensorarrayAdsTimingRead(ads126xAdcHandle_t *ads, sensorarrayAdsTimingProbeResult_t *r, unsigned group, uint32_t attempt, bool held, uint32_t floor, uint32_t before, uint32_t startCallUs, bool priorRawValid, int32_t priorRaw, uint32_t *categories, uint32_t *outGeneration, bool *outRawValid, int32_t *outRaw)` [118](sensorarrayAdsTimingProbe.c#L118) | 采集一个有明确组与尝试编号的新样本，保存 DRDY、CRC、状态及耗时。 / Acquire one sample under a group/attempt identity and retain DRDY, CRC, status and timing. | `sensorarrayAdsTimingProbeRun` |
| `esp_err_t sensorarrayAdsTimingProbeRun(ads126xAdcHandle_t *ads, uint64_t workDeadlineUs, sensorarrayAdsTimingProbeResult_t *r)` [232](sensorarrayAdsTimingProbe.c#L232) | 在工作截止时间内执行分组 ADC1 采集并收集反映读取阶段的具体证据。 / Run grouped ADC1 acquisitions before the work deadline and retain per-read-phase evidence. `ESP_ERR_INVALID_ARG` `ESP_ERR_INVALID_STATE` `ESP_ERR_NOT_SUPPORTED` | `sensorarrayAdsGapRunTimingProbeKind` |
| `static size_t sensorarrayAdsTimingFormatted(char *buffer, size_t size, int count)` [364](sensorarrayAdsTimingProbe.c#L364) | 核对 snprintf 的返回长度能否装入目标缓冲区。 / Check whether the formatted result fits the destination buffer. | `sensorarrayAdsTimingProbeFormatSummary` / `sensorarrayAdsTimingProbeFormatGroup` |
| `size_t sensorarrayAdsTimingProbeFormatSummary(const sensorarrayAdsTimingProbeResult_t *r, char *b, size_t n)` [373](sensorarrayAdsTimingProbe.c#L373) | 将本次探测的 ID、各组计数、测量及恢复结果写入摘要回复。 / Format the probe identity, group counts, measurement and restoration outcomes. | `sensorarrayAdsTimingControlFormat` |
| `size_t sensorarrayAdsTimingProbeFormatGroup(const sensorarrayAdsTimingProbeResult_t *r, uint32_t group, char *b, size_t n)` [391](sensorarrayAdsTimingProbe.c#L391) | 输出指定采样组的工作参数与结果。 / Format settings and results for the selected probe group. | `sensorarrayAdsTimingControlFormat` |
| `size_t sensorarrayAdsTimingProbeFormatGroupTiming(const sensorarrayAdsTimingProbeResult_t *r, uint32_t group, char *b, size_t n)` [411](sensorarrayAdsTimingProbe.c#L411) | 按采样组报告建立、等待和读取的耗时分布。 / Format settling, wait and read timing for the selected sample group. | `sensorarrayAdsTimingControlFormat` |
| `size_t sensorarrayAdsTimingProbeFormatRegisters(const sensorarrayAdsTimingProbeResult_t *r, bool restored, char *b, size_t n)` [426](sensorarrayAdsTimingProbe.c#L426) | 按请求输出保存或恢复时的 ADC1 寄存器证据。 / Format saved or restored ADC1 register evidence as requested. | `sensorarrayAdsTimingControlFormat` |
| `uint32_t sensorarrayAdsTimingProbeRecordCount(const sensorarrayAdsTimingProbeResult_t *r)` [440](sensorarrayAdsTimingProbe.c#L440) | 返回本次探测实际可查询的记录条数。 / Return the number of retained probe records available for queries. | `sensorarrayAdsTimingProbeFormatSummary` |
| `size_t sensorarrayAdsTimingProbeFormatRecord(const sensorarrayAdsTimingProbeResult_t *r, uint32_t index, char *b, size_t n)` [450](sensorarrayAdsTimingProbe.c#L450) | 按下标读取已保存的有限观测并格式化一条采样报告。 / Format one retained sample observation selected by record index. | `sensorarrayAdsTimingControlFormat` |
| `size_t sensorarrayAdsTimingProbeFormatTailRecord(const sensorarrayAdsTimingProbeResult_t *r, uint32_t index, char *b, size_t n)` [484](sensorarrayAdsTimingProbe.c#L484) | 输出指定记录中的读取尾部/校验诊断。 / Format read-tail and integrity evidence for the selected record. | `sensorarrayAdsTimingControlFormat` |
| `size_t sensorarrayAdsTimingProbeFormatTailIoRecord(const sensorarrayAdsTimingProbeResult_t *r, uint32_t index, char *b, size_t n)` [514](sensorarrayAdsTimingProbe.c#L514) | 输出指定尾部观测的驱动 I/O 时序及结果。 / Format driver I/O timing and outcome for the selected tail observation. | `sensorarrayAdsTimingControlFormat` |

### sensorarrayBatteryScheduler.c

| C 调用签名 / C signature | 行为及返回 / Behaviour and return | 调用位置 / Called by |
| --- | --- | --- |
| `static uint64_t sensorarrayBatteryMillisecondsToMicroseconds(uint32_t milliseconds)` [9](sensorarrayBatteryScheduler.c#L9) | 加宽后将周期从 ms 换算为 µs 以供单调调度。 / Widen and convert a millisecond period to microseconds for monotonic scheduling. | `sensorarrayBatterySchedulerInit` / `sensorarrayBatterySchedulerConfigure` |
| `static uint64_t sensorarrayBatteryAddSaturating(uint64_t left, uint64_t right)` [14](sensorarrayBatteryScheduler.c#L14) | 在 uint64 时间域做饱和求和，防止截止时间回绕。 / Add timestamps/intervals with uint64 saturation to prevent deadline wrap. | `sensorarrayBatterySchedulerInit` / `sensorarrayBatterySchedulerConfigure` |
| `void sensorarrayBatterySchedulerInit(sensorarrayBatteryScheduler_t *scheduler, bool enabled, uint32_t periodMs, uint32_t maximumDeferMs, uint64_t nowUs)` [19](sensorarrayBatteryScheduler.c#L19) | 初始化电池调度上下文和首个以单调时钟表示的到期时间。 / Initialise the battery scheduler and its first monotonic deadline. | `sensorarrayAdsGapInit` |
| `bool sensorarrayBatterySchedulerConfigure(sensorarrayBatteryScheduler_t *scheduler, bool enabled, uint32_t periodMs, uint64_t nowUs)` [39](sensorarrayBatteryScheduler.c#L39) | 更新周期、最大延后时间和调度状态。 / Update the period, maximum deferral and scheduler state. | `sensorarrayAdsGapConfigureBatteryPeriod` |
| `void sensorarrayBatterySchedulerRequestNow(sensorarrayBatteryScheduler_t *scheduler, uint64_t nowUs)` [66](sensorarrayBatteryScheduler.c#L66) | 置 forcePending，使下一有效边界可以接纳主动请求。 / Set forcePending to admit an explicit request at the next eligible boundary. | `sensorarrayAdsGapRequestBatteryDiagnostic` / `sensorarrayAdsGapRequestBatteryNow` |
| `bool sensorarrayBatterySchedulerIsDue(sensorarrayBatteryScheduler_t *scheduler, uint64_t nowUs)` [78](sensorarrayBatteryScheduler.c#L78) | 检查强制请求或周期的绝对截止是否使电池事务到期。 / Determine whether a forced request or periodic absolute deadline makes battery work due. | `sensorarrayAdsSelectJob` / `sensorarrayAdsGapRunBatteryAtBoundary` |
| `static void sensorarrayBatterySchedulerMarkDue(sensorarrayBatteryScheduler_t *scheduler, uint64_t nowUs)` [91](sensorarrayBatteryScheduler.c#L91) | 记录首次到期时间和待处理状态供边界调度。 / Retain the initial due timestamp for pending boundary admission. | `sensorarrayBatterySchedulerEvaluateGap` / `sensorarrayBatterySchedulerEvaluateBoundary` |
| `sensorarrayBatteryDecision_t sensorarrayBatterySchedulerEvaluateGap(sensorarrayBatteryScheduler_t *scheduler, uint64_t nowUs, uint32_t availableGapUs, uint32_t estimatedJobUs, uint32_t guardUs)` [104](sensorarrayBatteryScheduler.c#L104) | 登记帧内空隙观测并推迟到完整边界，不在两次行转换间启动电池事务。 / Observe the intra-frame gap and defer the battery transaction to a complete frame boundary. | 独立接口 / Standalone entry |
| `sensorarrayBatteryDecision_t sensorarrayBatterySchedulerEvaluateBoundary(sensorarrayBatteryScheduler_t *scheduler, uint64_t nowUs, bool capacitanceMode)` [138](sensorarrayBatteryScheduler.c#L138) | 在完整帧边界检查请求/到期时间，准入整个保存、测量、恢复事务。 / At a complete frame boundary, check forced/due state and admit the entire save/measure/restore transaction. | `sensorarrayAdsGapRunBatteryAtBoundary` |
| `void sensorarrayBatterySchedulerRecordRun(sensorarrayBatteryScheduler_t *scheduler, uint64_t completedUs, uint32_t durationUs, bool boundaryFallback, bool restoreOk)` [159](sensorarrayBatteryScheduler.c#L159) | 从前一绝对期限推进周期、跳过错过的时隙，并记录时长与恢复状态。 / Advance from the previous absolute deadline, skip missed slots and record duration and restore outcome. | `sensorarrayAdsGapRunBatteryAtBoundary` |
| `uint32_t sensorarrayBatterySchedulerAgeMs(const sensorarrayBatteryScheduler_t *scheduler, uint64_t nowUs)` [205](sensorarrayBatteryScheduler.c#L205) | 按最后运行时间计算经过的毫秒数，缺少历史时保留未知哨兵。 / Compute age in milliseconds since the last run, preserving the unknown sentinel before history exists. | `sensorarrayAdsGapCopySnapshot` |
| `void sensorarrayBatterySchedulerRecordValidity(sensorarrayBatteryScheduler_t *scheduler, bool valid, uint8_t reason, uint64_t completedUs)` [216](sensorarrayBatteryScheduler.c#L216) | 按相同失败原因的连续次数退避；成功或原因变化重新累计。 / Back off for repeated identical failures and restart the count on success or a changed reason. | `sensorarrayAdsGapRunBatteryAtBoundary` |

## 参数索引 / Parameter index

相同名称按模块共享解释。完整类型、数组维度与输入输出方向以对应函数签名和调用点为准。指针参数所需存储由调用者提供，异步作业另受上面的所有权约束。

Entries share parameter descriptions within this module. Refer to the signature and its caller for the precise type, array shape and direction. The caller supplies pointer storage, subject to the ownership rules above for asynchronous work.

| 参数 / Parameter | 含义 / Meaning |
| --- | --- |
| `a`, `b`, `c`, `left`, `right` | 数值比较、取中位值或身份比较的输入操作数，按相应签名确定类型。 / Operand for a numeric comparison, median or identity comparison as declared. |
| `activeCellCount` | 活动测点数量。 / Active cell count. |
| `actual`, `expected` | 核验时所需的配置/身份与实际寄存器观测值，使用该接口指定的类型和掩码。 / Desired configuration/identity and actual evidence compared with the type and masks for this interface. |
| `adc1Running` | ADC1运行。 / ADC 1 running. |
| `adc1WasRunning` | ADC1原运行。 / ADC 1 was running. |
| `adcReadErr` | ADC读取错误码。 / ADC read err. |
| `additionalCount` | 补充数量。 / Additional count. |
| `additionalStatus` | 补充状态。 / Additional status. |
| `ads` | 对应驱动句柄、FDC 主/次设备 ID 或 ADS/FDC 状态；按签名类型选取当前板上同一实例。 / Corresponding driver handle, primary/secondary FDC ID or ADS/FDC state. Pass the board instance matching the declared type. |
| `adsReset` | ADS复位。 / ADS reset. |
| `allowRefresh` | 允许刷新。 / Allow refresh. |
| `attempt` | 候选所需样本量、允许尝试数或当前尝试索引。 / Candidate sample target, allowed number of attempts or current attempt index. |
| `availableGapUs`, `estimatedJobUs`, `guardUs` | 准入时使用的保护余量、可用窗口和预计事务耗时，均为 µs。 / Guard, available window and estimated transaction cost at admission, all in µs. |
| `avddUv`, `avssUv`, `matrixReferenceUv`, `vrefUv` | 模拟供轨、矩阵激励或转换参考，单位 µV。须保留参考点及来源。 / Analogue rail, matrix excitation or ADC reference in µV. Retain its reference node and provenance. |
| `battery` | 电池。 / Battery. |
| `before` | 事务前后读取的计数、世代或状态快照，按签名进行比较。 / Counter, generation or state snapshot taken before/after the operation for comparison. |
| `bootCount`, `bootId` | 会话、启动身份或启动计数，查询与终态依赖所属启动域。 / Session identifier, boot identifier or boot count scoping queries and terminal outcomes. |
| `boundaryFallback`, `restoreOk` | 辅助事务的配置恢复结果或是否在完整帧边界补测。 / Configuration restoration outcome or whether the auxiliary job used a complete-frame fallback boundary. |
| `buffer` | 调用者提供的回复、暂存或结果缓冲区，使用随同的容量/类型。 / Caller-provided reply, scratch or result storage with the accompanying capacity/type. |
| `bufferSize`, `size` | 接收文本或字节缓冲区的容量。文本输出还需预留 NUL。 / Capacity of the supplied byte/text buffer. A text output must also accommodate its NUL terminator. |
| `cache`, `registerCache`, `valueCache` | 该入口操作的寄存器、数值或其他缓存，作用域遵循声明类型。 / Register, value or other cache of the specific type in the signature. |
| `calibration` | 校准。 / Calibration. |
| `calibrationGeneration`, `configurationGeneration`, `currentConfigurationGeneration`, `sampleConfigurationGeneration` | 校准、当前配置或样本所属世代，用于阻止旧样本更新新状态。 / Calibration, current-configuration or sample generation used to exclude stale samples. |
| `capacitanceMode` | 电容模式。 / Capacitance mode. |
| `categories` | 类别。 / Categories. |
| `cell` | 测点。 / Cell. |
| `cellIndex` | 行优先数组中的线性测点下标。 / Linear cell index in row-major storage. |
| `check` | 检查。 / Check. |
| `cleanupError`, `err`, `error`, `originalError` | 操作、首个根因或独立清理结果的错误码。保留调用层提供的具体错误。 / Error for the operation, first cause or separate cleanup, preserving the code from the reporting layer. |
| `coherent`, `fresh` | 采集新鲜标志、一致性或测点位图。掩码按物理测点下标定位。 / Freshness flag, coherence flag or per-cell bitset. A cell mask is indexed by physical position. |
| `comparison` | 接收首个寄存器不匹配的位置、预期值、实际值和有效位掩码。 / Receives the first differing register, expected/actual byte and comparison mask. |
| `completedUs`, `frameEndUs`, `nowUs`, `startUs` | 单调时钟的事件时间，单位 µs。 / Event time on the monotonic clock in µs. |
| `config`, `policy` | 该入口声明的配置或策略结构，含所选阈值、时序或回调。 / Configuration or policy of the declared type holding the selected thresholds, timing or callbacks. |
| `context` | 回调或任务创建时绑定的上下文；在该调用的有效期内保留其存储。 / Context bound at callback registration or task creation. Keep its backing object alive for the operation. |
| `count` | 数量。 / Count. |
| `crcRejectedMask` | CRC被拒绝掩码。 / CRC rejected mask. |
| `currentRaw` | 当前原始码。 / Current raw. |
| `delayAfterRowMs` | 延迟之后行，单位 ms。 / Delay after row in ms. |
| `delayAfterSelAMs` | 延迟之后选择甲，单位 ms。 / Delay after sel a in ms. |
| `delayAfterSelBMs` | 延迟之后选择乙，单位 ms。 / Delay after sel b in ms. |
| `delayAfterSwMs` | 延迟之后SW，单位 ms。 / Delay after sw in ms. |
| `diag` | 本次启动、采样或调度的汇总/诊断记录，类型见对应函数。 / Summary/diagnostic evidence for this startup, sample or scheduling operation, typed by the function. |
| `differentialUv`, `negativeInputUv`, `nodeUv`, `positiveInputUv`, `railMarginUv` | ADS 输入、共模余量、节点或差分电压，单位 µV。 / ADS terminal voltage, common-mode margin, node voltage or differential voltage in µV. |
| `discardFirst` | 丢弃首个。 / Discard first. |
| `dLine`, `sColumn` | 板级 S 列或 D 线编号，合法位置为 1..8。 / Physical S column or D line, numbered 1..8. |
| `durationUs` | 耗时，单位 µs。 / Duration in µs. |
| `enabled`, `known`, `valid` | 表示证据已知、数据有效、功能启用或操作成功的布尔标志，按参数名独立解释。 / Boolean representing known evidence, validity, enablement or success as named. |
| `engine` | 已初始化且与本次采集绑定的引擎实例。 / Initialised engine instance bound to this acquisition. |
| `event` | 事件。 / Event. |
| `expectedFdcReadyUs` | 预期FDC就绪，单位 µs。 / Expected fdc ready in µs. |
| `expectedMode0`, `expectedMode1`, `expectedPower`, `expectedRefmux` | 该寄存器在当前路由或健康检查中预期的配置字节。 / Expected register byte for the current route or health check. |
| `fault` | 故障。 / Fault. |
| `fingerprint` | 指纹。 / Fingerprint. |
| `firstGeneration`, `generation`, `outGeneration`, `outStartGeneration`, `startGeneration` | 驱动边沿、缓存或槽位的世代，按本函数的对象匹配，跨对象不能直接混用。 / Driver-edge, cache or slot generation. Match it to the object identified by the function. |
| `floor` | 本次等候的新转换所基于的 DRDY 世代下界。 / Lower bound on DRDY generation for the conversion being awaited. |
| `forceRoute` | 强制路由。 / Force route. |
| `frame` | 当前采集帧。const 指针供读取/编码，可写指针供对应采集步骤产生或更新数据。 / Capture frame. A const pointer is an input to readers/encoders. A writable pointer receives or updates acquisition results. |
| `frameCount` | ADS 引擎累计帧数，健康周期相对于此计数。 / Cumulative ADS engine frame count used to determine health-read boundaries. |
| `frameError` | 帧错误。 / Frame error. |
| `frameSequence`, `seq`, `sequence` | 对应采集帧的序号，必须与提供的帧、事件或缓存上下文属于同次启动。 / Capture sequence belonging to the same boot as the supplied frame, event or cache context. |
| `freshConversion`, `precisionFrame`, `statusClean`, `transitionSensitive` | 明确转换世代、状态无告警、定期精度帧或切换敏感状态，影响单样本准入。 / Inputs to single-sample admission describing fresh generation, clean status, precision frame or transition sensitivity. |
| `fullSample` | 完整样本。 / Full sample. |
| `fullScaleCal` | 完整量程校准。 / Full scale cal. |
| `g` | 增益。 / G. |
| `gain` | ADS PGA 增益或预期增益；bypass 和数值增益分别编码。 / ADS PGA gain or expected gain. Bypass has its own encoding. |
| `group` | 分组。 / Group. |
| `health` | 健康。 / Health. |
| `held` | 已保持。 / Held. |
| `hysteresisUv`, `invalidationThreshold`, `minimumStableStreak`, `noiseMultiplier` | 噪声倍数、稳定样本数、失效阈值或供轨滞回，用于缓存有效性判定。 / Noise multiplier, stable-sample count, invalidation threshold or rail hysteresis used for cache admission. |
| `index` | 索引。 / Index. |
| `inOutRetryCount` | 在输出重试数量。 / In out retry count. |
| `inpmux`, `mode0`, `mode1`, `mode2`, `refmux` | 当前事务使用或核验的旁路/输入/电源及参考设置，原始寄存器参数使用器件编码。 / Bypass/input/mode/reference settings used or checked by the transaction, using device encoding for raw register arguments. |
| `input` | 输入。 / Input. |
| `inputMode` | 输入模式。 / Input mode. |
| `interval` | 间隔。 / Interval. |
| `invalidationThresholdUv` | 失效阈值，单位 µV。 / Invalidation threshold in µV. |
| `isolate` | 隔离。 / Isolate. |
| `job` | 作业。 / Job. |
| `kind`, `outcome`, `phase`, `stage` | 对应操作、诊断或状态机的枚举/标签，采用签名中所列类型的常量。 / Operation, diagnostic or state-machine discriminator from its declared enum or label domain. |
| `label` | 标签。 / Label. |
| `latchMask` | 锁存掩码。 / Latch mask. |
| `length` | 输入、载荷、底层存储或读写传输的字节数。 / Byte length of input, payload, backing storage or transfer. |
| `limits` | 范围。 / Limits. |
| `line` | 线。 / Line. |
| `mask` | 作用域内的行、通道或测点选择位。完整科学帧的每个测点各占一位。 / Bitset of selected rows, channels or cells. A full science frame uses one bit per cell. |
| `maximumAgeFrames` | 内部 rail 读数或缓存允许的帧龄，以经过的帧数计。 / Allowed internal-rail or cache age in elapsed frames. |
| `maximumDeferMs` | 最大延后，单位 ms。 / Maximum defer in ms. |
| `maximumSpreadUv`, `minimumRawThreshold` | 判定或限幅所用阈值，以参数名指定的数值域和单位为准。 / Threshold for admission or clamping in the numeric domain and units named by the parameter. |
| `milliseconds` | 毫秒。 / Milliseconds. |
| `mode` | 对应调用域的模式枚举，保留目标、当前帧或恢复上下文的含义。 / Mode enum in the declared domain, selecting the target, frame or recovery mode as indicated. |
| `mode0Valid` | 模式0有效。 / Mode 0 valid. |
| `monitorUv` | 监测，单位 µV。 / Monitor in µV. |
| `muxn`, `muxp` | ADS 正、负模拟输入选择码，请按板级 D 线映射。指针形态用于接收选择码。 / ADS positive/negative input selector from the board D-line map. Pointer forms receive the selector. |
| `n` | 负。 / N. |
| `nodeUvSamples` | 节点µV样本。 / Node µV samples. |
| `offsetCal` | 偏移校准。 / Offset cal. |
| `openSemantic` | 开路类型。 / Open semantic. |
| `outCalibration` | 接收校准。 / Receives calibration. |
| `outCleanupError`, `outError` | 接收结果。操作、首个根因或独立清理结果的错误码。保留调用层提供的具体错误。 / Output. Error for the operation, first cause or separate cleanup, preserving the code from the reporting layer. |
| `outContext` | 接收结果。回调或任务创建时绑定的上下文；在该调用的有效期内保留其存储。 / Output. Context bound at callback registration or task creation. Keep its backing object alive for the operation. |
| `outDiag` | 接收结果。本次启动、采样或调度的汇总/诊断记录，类型见对应函数。 / Output. Summary/diagnostic evidence for this startup, sample or scheduling operation, typed by the function. |
| `outDurationUs` | 接收耗时，单位 µs。 / Receives Duration in µs. |
| `outGap` | 接收间隙。 / Receives gap. |
| `outHaveMohm` | 接收具有mΩ。 / Receives have mΩ. |
| `outMapLabel` | 接收映射标签。 / Receives map label. |
| `outMedianNodeUv` | 接收中位值节点，单位 µV。 / Receives Median node in µV. |
| `outMedianUv` | 接收中位值，单位 µV。 / Receives Median in µV. |
| `outMohm` | 接收mΩ。 / Receives mΩ. |
| `outMonitorUv` | 接收监测，单位 µV。 / Receives Monitor in µV. |
| `outNodeUv` | 接收结果。ADS 输入、共模余量、节点或差分电压，单位 µV。 / Output. ADS terminal voltage, common-mode margin, node voltage or differential voltage in µV. |
| `outPowerAfter` | 接收电源之后。 / Receives power after. |
| `outProfile` | 接收结果。配置档或八个物理行的模式列表。outModes 接收解析结果。 / Output. Configuration profile or mode selection for the eight physical rows. outModes receives parsed values. |
| `outRail` | 接收结果。当前分析所用的供轨/跨度、有效性与读数状态。railUv 的单位为 µV。 / Output. Rail/supply span and validity/status evidence used by this operation. railUv is in µV. |
| `outRan` | 接收已运行。 / Receives ran. |
| `outRaw`, `priorRaw`, `raw`, `rawCode` | ADC 原始整数码或 FDC 原始频率码；单位与定点物理值分开。 / Integer ADC code or FDC frequency code, separate from fixed-point physical units. |
| `outRawValid` | 接收原始码有效。 / Receives raw valid. |
| `outReason`, `reason` | 操作或拒绝原因字符串；outReason 用于返回原因。 / Operation/rejection reason string. outReason receives the reason. |
| `outResult`, `outSnapshot`, `result`, `snapshot`, `stats` | 函数签名注明的结果、快照或统计结构，调用者分配复制目标。 / Result, snapshot or statistics object of the declared type, with caller-owned output storage. |
| `outSample` | 接收结果。要归约或校验的一组测量，数组长度由相邻 count 参数或签名给出。 / Output. Measurements to reduce or validate. The adjacent count or declared array bound gives their extent. |
| `outSplit` | 接收分配。 / Receives split. |
| `outStatusByte` | 接收状态字节。 / Receives status byte. |
| `outTimedOut` | 接收计时输出。 / Receives timed out. |
| `outUv` | 接收µV。 / Receives μV. |
| `outValue` | 接收值。 / Receives value. |
| `overrange` | 超量程。 / Overrange. |
| `owner` | 申请或释放 ADS 时所声明的所有者枚举。 / Named ADS transaction owner to acquire or release. |
| `path` | VFS 描述符、open 标志或路径，或对应类型的矩阵路径。 / VFS descriptor, open flags or path, or a matrix route if the declared type selects one. |
| `periodMs` | 周期，单位 ms。 / Period in ms. |
| `pgaBypassed` | 是否使用硬件 PGA bypass。此状态和 PGA 增益 1 分别处理。 / Selects or records hardware PGA bypass, distinct from gain 1. |
| `plan` | 活动行的采样操作计划或逐行配置，所有组使用同一帧的配置快照。 / Acquisition plan or row selections for the active frame. All groups use the same capture snapshot. |
| `power` | ADS 对应的原始寄存器字节，比较由该事务的读回掩码限定。 / Raw ADS register byte. Readback comparisons use the mask for that transaction. |
| `primaryStatus` | 主级状态。 / Primary status. |
| `priorRawValid` | 之前原始码有效。 / Prior raw valid. |
| `profileValid` | 配置档有效。 / Profile valid. |
| `r` | 记录。 / R. |
| `rail`, `railStatusGood`, `railUv`, `railValid` | 当前分析所用的供轨/跨度、有效性与读数状态。railUv 的单位为 µV。 / Rail/supply span and validity/status evidence used by this operation. railUv is in µV. |
| `railSource` | 供轨读数的真实来源，如外部校准或内部 span 估算。 / Provenance of the rail value, such as external calibration or an internal span estimate. |
| `record` | 按签名为有界探测记录的索引、诊断记录或序列化记录字节。 / Index of a bounded probe observation, diagnostic record or serialised record storage as declared. |
| `referenceSource` | ADC 用于转换的参考源枚举。 / ADC reference source used for conversion. |
| `reg` | 寄存器。 / Register. |
| `registerId` | ADS 寄存器缓存枚举索引，用于选择待写或读回核验的寄存器。 / ADS register-cache index selecting a register for write or readback check. |
| `requestId` | 对应配置、恢复触发或 profile 域内的请求标识，由接收方与提交时身份配对。 / Request identifier for the indicated configuration, recovery trigger or row profile, paired with the identity at admission. |
| `requestRail`, `requestZero` | 请求在调度允许时进行归零或供轨辅助事务。 / Requests a zero or rail transaction when scheduling admits it. |
| `restored`, `saved` | 事务开始时保存或在恢复之后读回的状态/布尔标记。 / State captured before the transaction or observed after restore, or a flag selecting that stage. |
| `retryBudget`, `retryCount`, `sampleCount` | 本次操作允许的尝试、样本或重试数量。 / Attempt, sample or retry count budgeting this operation. |
| `routeController` | 持有目标路由、实测控制回读与所有权状态的控制器实例。 / Controller instance holding the target route, control readback and ownership state. |
| `row` | 行或列的坐标。通用 matrixEngineRegion_t 的原点为零；以板级 S/D 位置操作的接口从 1 编号。 / Row or column coordinate. Generic matrixEngineRegion_t origins are zero-based. Board S/D coordinates are one-based. |
| `rows` | 行数值或行配置快照，由该函数的参数类型决定。 / Active count or row-configuration snapshot as specified by its declared type. |
| `sample`, `samples`, `values` | 要归约或校验的一组测量，数组长度由相邻 count 参数或签名给出。 / Measurements to reduce or validate. The adjacent count or declared array bound gives their extent. |
| `savedVrefUv` | 已保存参考电压，单位 µV。 / Saved vref in µV. |
| `scheduler` | 保存电池周期到期、最近一次运行、退避和强制请求的持久调度上下文。 / Persistent battery scheduler holding periodic deadline, last run, backoff and forced requests. |
| `selaRoute` | 预期/观测的路由、寄存器或状态值，按该签名中声明的类型比较。 / Requested/observed route, register or state values, compared in the types shown by the signature. |
| `selBLevel` | 对应控制脚读到或要输出的数字电平。 / Requested or observed digital level of the indicated control pin. |
| `settleDelayMs` | 建立延迟，单位 ms。 / Settle delay in ms. |
| `sink` | 输出端。 / Sink. |
| `source` | 路由或测量的来源标记，区分外部实测、内部推算或对应开关源的枚举。 / Route/measurement source discriminator, such as external, internal estimate or switch source for the declared enum. |
| `spreadRaw` | 极差原始码。 / Spread raw. |
| `startCallUs` | 起始调用，单位 µs。 / Start call in µs. |
| `state` | 所声明类型的状态对象或枚举。传入前端状态指针时，其生命周期覆盖总线或采集操作。 / State object or enum of the declared type. A frontend-state pointer must outlive the bus or capture operation. |
| `status` | 声明的诊断/状态记录，或从器件读回的原始状态字。 / Diagnostic/status record of the declared type or a raw status word returned by the device. |
| `swSource` | 板级模拟开关或输入源选择，可选值来自声明的 TMUX/路由类型。 / Board analogue switch or source selector from the declared TMUX/route type. |
| `t` | 时刻。 / T. |
| `target` | 此操作的目标，可能为目标帧、路由/测点结构或 sink，以签名类型为准。 / Target frame, route/cell or sink, according to the declared type. |
| `telemetry` | 当前帧或事务性能/健康计数的写入目标。 / Performance or health telemetry for the current frame/transaction. |
| `text` | 按该函数声明传入的文本、命令、操作码或数据字节，显式 length 限定可读范围。 / Text, command, opcode or data bytes as declared. Where supplied, length bounds the readable range. |
| `timeoutUs` | 有限等待预算或硬超时标志，时间单位见参数后缀。 / Bounded waiting budget or hard-timeout flag. Time units follow the suffix. |
| `transactionInpmux` | 事务INPMUX。 / Transaction INPMUX. |
| `transactionMode2` | 事务模式2。 / Transaction mode 2. |
| `transactionPower` | 事务电源。 / Transaction power. |
| `transactionRefmux` | 事务REFMUX。 / Transaction REFMUX. |
| `uv` | µV。 / ΜV. |
| `value` | 值。 / Value. |
| `valueFixed` | 值定点。 / Value fixed. |
| `verified` | 已核验。 / Verified. |
| `workDeadlineUs` | 单调时钟绝对截止时间，单位 µs。 / Absolute deadline on the monotonic microsecond clock. |
| `zeroUv` | 零点，单位 µV。 / Zero in µV. |

[项目总览 / Project guide](../../../README.md)

# ADS matrix measurement / ADS 矩阵测量

## 中文

`sensorarrayAdsMatrix` 只使用 ADS1262/ADS1263 都具备的 ADC1，所以运行时识别为
ADS1262 时仍可执行 VOLT/RES；ADC2 API 只在实际 ADS1263 上可用，不会静默假装
存在。D1..D8 到 AIN 的映射来自 `sensorarrayBoardMapAdsMuxForDLine()`，当前板级
映射为 AIN0..AIN7 对 AINCOM，驱动层不知道 S/D 坐标或 frame 格式。

每个 cell 的有界流程是：进入安全路由、选择 row、选择 D-line input mux、应用
缓存的 PGA/bypass profile、记录当前 DRDY generation、等待并读取一个真正的新
conversion，然后按 cell noise/status 决定是否再读两个 fresh conversion 并取中位数。
SPI、DRDY timeout、stale/reset、
ADS status reference/PGA alarm、full-scale saturation、rail/common-mode 和稳定性
都必须通过后才设置 valid/fresh bit。无效值在内存中为 `INT64_MIN`，wire 上为
`Xhh` 原因码，绝不伪装成 0。

原理图确认 TMUX1108 EN 仅由 10 kΩ 上拉、未连接 MCU；因此不能假装执行
disable/address/enable。生产默认 RES 快路径依赖器件的 break-before-make，在
SW 已为 REF、INTREF/REFOUT 已安全开启的整个 RES session 内只改 A0..A2，随后
执行可配置 row settle，并核对 row/source/SW、POWER 和 REFMUX。若该能力关闭，
保留原有 STOP1、关闭 INTREF、SW/Q1 clamp、换行、释放 clamp、恢复并 readback
的保守路径。`RESSETTLE` sweep 只有在 S1D1/S8D8 同步万用表数据满足误差要求时
才选择更短 settle，否则恢复原值。

VOLT/RES Kconfig target 默认 `0`（unlimited）；只有显式 `FPSCAP=ON,<fps>` 才让
Core 1 pacing。瞬态 timeout/stale 仍有有界同-cell恢复，header `ir` 报告成功
恢复的次数。

### Four cache layers and adaptive sampling

1. register shadow 跟踪 POWER/INTERFACE/MODE0/1/2/INPMUX/REFMUX/OFCAL/FSCAL、
   VREF、PGA 和 ADC running state；相同值不写，完整 readback 只在启动、transition、
   ADSCHK、外部 transaction、错误/reset 和周期 health frame 执行。
2. input profile 按 VOLT/RES 和 cell 隔离，结构化保存 PGA 或 bypass。确认过的
   VOLT bypass 下帧直接命中，不再每帧从 gain 1 失败一次。
3. rail fingerprint 使用可配置绝对阈值与 hysteresis；正常微伏抖动不会清空 profile。
4. value/noise cache 保存 raw/node/value、noise estimate 和 streak。稳定 cell 的
   dynamic raw threshold 为 `max(minRaw, noise*multiplier)`。

cache miss、alarm/saturation、rail/reference/calibration/reset 变化或周期 precision
frame 会强制三样本。默认每 16 帧的 precision frame 对所有活动 cell 三样本；其余
稳定 cell 单样本，但每帧仍满足 `freshCells == activeRows*8`。

### Active check and battery transaction

`ADS?` 是轻量 snapshot，未知 ID 不默认成 ADS1262。`ADSCHK[=N]` 在 Core 1 帧边界
主动读取全部关键寄存器、START ADC1、等待 N 个新 DRDY、检查 new-data/status/reset，
统计 period/changed/SPI/timeout/stale，再恢复并完整回读；失败使 matrix cache 失效并
进入 SAFE/DEGRADED。

周期电池复用 `sensorarrayAdsGap` 和同一 driver/owner。scheduler 以
`esp_timer_get_time()` 计时；CAP gap 不足则 defer，超过最大延迟才 boundary fallback；
VOLT/RES 只在完整帧后保存/恢复 ADS。测量为 AIN8-AINCOM，加有效 rail/VBIAS 得到
AINCOM-GND，再按 divider numerator/denominator、ppm/offset 做 checked int64 换算。
restore failure 会 invalidate shadow/profile 并禁止下一普通有效矩阵帧。

### Voltage

ADS 测量 `V(Dn) - V(AINCOM)`；`AINCOM` 由 VBIAS 维持在模拟电源中点。
运行时先通过 ADS supply monitor 得到 `AVDD-AVSS`，再结合经过审核的板级
AVDD/AVSS 分配 profile 推导有 age/validity 的 AVDD、AVSS、AINCOM。最终节点
电压以微伏输出，必须同时通过配置的电压范围、输入绝对范围、PGA 共模和输出
摆幅检查。不能把标称 3.3 V/-1.8 V 当作实时测量值。

### Resistance

本板已核对的近似路径为：

```text
ADS REFOUT (Vref) -> TMUX1108 -> Rx -> TMUX1134 B -> Dn/AINn (Vnode)
                    -> Rref -> AVSS (Vss)
```

因此仅在该采样节点和路径 readback 有效时使用：

```text
Rx = Rref * (Vref - Vnode) / (Vnode - Vss) - calibratedPathOffset
```

`Vnode` 来自新鲜 ADC1 conversion；`Vss` 来自有 age 限制的 rail snapshot；
`Vref` 是校准的 REFOUT-to-AVSS span 所确定的矩阵激励，不等同于 AVDD；`Rref`
来自 Kconfig/校准结构。近零分母、负值、overflow、short、open、range、unstable
和 stale 各有独立错误。TMUX 的数据手册典型 Ron 不会直接作为“精确补偿”。

`sensorarrayAdsMatrixCalibration_t` 带版本并支持 Rref、matrix reference span、
全局/bank/per-cell path offset。setter 只在 SAFE/NONE 接受通过范围检查的数据。
当前仓库没有把它写入 NVS；持久化校准仍未实现，避免引入无版本/无 CRC 格式。

### Automatic PGA

候选增益集中为驱动支持的 1/2/4/8/16/32。初次使用 gain 1；纯逻辑决策同时看
code utilisation、PGA absolute/differential alarms、AVDD/AVSS、输入共模和 rail
margin。alarm/饱和立即降档；只有安全且低于升档阈值才升档；双阈值提供
hysteresis。每次改档都要 register readback、settle、discard fresh conversion，
并受最大尝试次数约束。cache 按 mode/cell 分开并在 route/reference/calibration/
ADS reset/连续 overrange 时失效。

若 gain 1 仍报告 PGA absolute/differential/common-mode alarm，但原始输入仍在
AVDD/AVSS 外扩 `CONFIG_SENSORARRAY_ADS_BYPASS_INPUT_MARGIN_UV` 的审核范围内，引擎
可切换到经过 MODE2 readback 的 PGA bypass。wire `P` 值 `00` 明确表示 bypass，
不会被当作 gain 1 或未知状态。

## Australian English

The ADS matrix engine uses ADC1, which is available on both ADS1262 and
ADS1263. ADC2 is used only when the detected part supports it. Board mapping,
not the driver, maps D1..D8 to AIN0..AIN7 against AINCOM.

Each cell obtains at least one new DRDY-generation conversion per frame. A
stable cell uses that single sample; a changing, noisy, alarmed or precision-
frame cell obtains two more fresh samples and uses the median. Four coherent
caches cover registers, per-mode/cell PGA-or-bypass profiles, rail fingerprints,
and cell values/noise. The default resistance route keeps INTREF active for the
RES session and uses the TMUX1108 break-before-make address transition because
the schematic shows its EN pin is not MCU-controlled. The old clamp-and-restart
sequence remains the configurable safety fallback.
Voltage is emitted as integer microvolts. Resistance is emitted as integer
milliohms using the confirmed divider nodes and calibrated Rref/reference/path
data. Invalid samples carry an explicit reason instead of zero. Calibration
persistence is deliberately not implemented until a versioned, range-checked,
CRC-protected store is available.

## Battery freshness and noise gate

The ordinary AIN8 transaction uses ADS1262 ADC1 at 1200 SPS, confirms VBIAS
and the AIN8/AINCOM/reference registers, discards one new conversion, and takes
the median of three further new conversions. A timeout or stale new-data bit
may wait once for a later DRDY generation; SPI/status/reference/reset alarms
are never retried into validity. All three fresh samples are still mandatory.

The raw spread gate defaults to 2,000,000 codes. At the measured 5.188 V rail
this is about 4.83 mV at AIN8 and 9.66 mV after the default 2:1 divider. The
gate therefore tolerates the observed CAP-boundary coupling while remaining
below the 10 mV hardware acceptance floor. `batteryValidRunCount` and
`batteryInvalidRunCount` are cumulative transaction outcomes; retry, unstable,
terminal timeout, last spread, and maximum spread are independently retained
for long-dwell auditing.

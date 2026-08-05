# ADS matrix measurement / ADS 矩阵测量

## 中文

`sensorarrayAdsMatrix` 只使用 ADS1262/ADS1263 都具备的 ADC1，所以运行时识别为
ADS1262 时仍可执行 VOLT/RES；ADC2 API 只在实际 ADS1263 上可用，不会静默假装
存在。D1..D8 到 AIN 的映射来自 `sensorarrayBoardMapAdsMuxForDLine()`，当前板级
映射为 AIN0..AIN7 对 AINCOM，驱动层不知道 S/D 坐标或 frame 格式。

每个 cell 的有界流程是：进入安全路由、选择 row、选择 D-line input mux、应用
候选 PGA、核对寄存器、等待 settle、丢弃配置数量的真正新 DRDY generation、
有限次 oversample、中位数聚合和 spread 检查。SPI、DRDY timeout、stale/reset、
ADS status reference/PGA alarm、full-scale saturation、rail/common-mode 和稳定性
都必须通过后才设置 valid/fresh bit。无效值在内存中为 `INT64_MIN`，wire 上为
`Xhh` 原因码，绝不伪装成 0。

RES 换 row 时先停止 ADC1 和 INTREF，再用 SW high 钳住 REFOUT，完成 row 选择后
才释放 SW、恢复 INTREF 并核对 POWER/REFMUX；这避免把工作中的 REFOUT 短接到
GND。瞬态 timeout/stale 最多执行 `CONFIG_SENSORARRAY_ADS_MATRIX_IO_RETRY_COUNT`
次同 cell restart，默认一次且不放大 DRDY timeout。VOLT/RES 分别按独立的默认
3 FPS 目标 pacing；header `ir` 报告已恢复的 retry。

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

Each cell has bounded route, settle, fresh-generation discard, oversampling,
median, stability, status, rail, common-mode, saturation, and autorange checks.
Resistance row changes stop ADC1 and INTREF before clamping REFOUT, then restore
excitation only after the new row is selected. Transient fresh-data failures
have a bounded same-cell restart, and gain-1 PGA alarms may use a readback-
verified bypass only inside the configured absolute-input margin.
Voltage is emitted as integer microvolts. Resistance is emitted as integer
milliohms using the confirmed divider nodes and calibrated Rref/reference/path
data. Invalid samples carry an explicit reason instead of zero. Calibration
persistence is deliberately not implemented until a versioned, range-checked,
CRC-protected store is available.

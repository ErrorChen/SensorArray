# 硬件概览

## 8×8 矩阵

矩阵按 `S1..S8` 行、`D1..D8` 列命名，wire order 固定为 row-major：

```text
S1D1, S1D2, ... S1D8, S2D1, ... S8D8
```

`ROWS=1..8` 启用对应数量的物理行和 `rows*8` 个 cell。未启用的行不属于当前 frame；mixed-row profile 则通过 `ROWMODES=<8 chars C|V|R>` 为每个 S 行选择独立测量模式。

## FDC 与 D-line 映射

| D-line | FDC | Channel | 默认 I2C 地址 |
| --- | --- | --- | --- |
| D1 | primary | CH0 | `0x2B` |
| D2 | primary | CH1 | `0x2B` |
| D3 | primary | CH2 | `0x2B` |
| D4 | primary | CH3 | `0x2B` |
| D5 | secondary | CH0 | `0x2A` |
| D6 | secondary | CH1 | `0x2A` |
| D7 | secondary | CH2 | `0x2A` |
| D8 | secondary | CH3 | `0x2A` |

SELA 服务 D1..D4，SELB 服务 D5..D8。TMUX1134 `SEL=1` 选择电容/FDC 分支，`SEL=0` 选择电阻/ADS 分支。

## SW、REF 与 ADS 状态

以下状态必须分开理解：

- SW physical level：MCU 输出的 high/low；
- SW logical source：矩阵公共节点连接 GND 或 REF；
- matrix excitation：是否把 `REFOUT` 送入电阻矩阵；
- ADS `INTREF`：芯片内部 reference 电路开关；
- ADS `REFMUX`：ADC conversion 选用的 reference；
- ADS `VBIAS`：AINCOM bias 状态。

在本板上，SW high 使外部 Q1 把共享 REF/REFOUT 节点 clamp 到 GND；因此该状态下不能同时开启会把 REFOUT 驱入 GND 的内部 reference 路径。

## 电阻测量拓扑

RES 使用：

```text
REFOUT -> Rx -> Vnode -> Rref -> AVSS
```

默认 `Rref` 配置为 10 kΩ，但它是板级参数，不是待测件的假设。生产算法不能把调试板 S1D1/S8D8 的标称 10 kΩ 写死。

当前 RES 调试板已知：

- S1D1：约 10 kΩ；
- S8D8：约 10 kΩ；
- 其他未连接 cell 可能产生 `X0D`（open），这是当前接线下的合法候选现象。

验收脚本可以用宽松范围做 pipeline sanity，但绝对精度仍需 DMM 同步读数与元件公差。

## 电池测量拓扑

```text
VBAT -> R -> AIN8 -> R -> GND
```

当前 divider 为 `2:1`。ADS 读取 `AIN8-AINCOM`，再结合有效 rail/VBIAS 得到 `AIN8-GND`：

```text
ain8Gnd = (AIN8 - AINCOM) + AINCOM_GND
vbat = ain8Gnd * dividerNumerator / dividerDenominator
```

诊断时应同步测量：

- `VBAT-GND`；
- `AIN8-GND`；
- `AINCOM-GND`；
- 实际 divider ratio。

没有这些证据时，只能验证 transaction、freshness、scheduler 与 restore，不能宣称电池电压绝对精度通过。

## 硬件证据边界

- GPIO readback 只能证明数字控制值，不能证明模拟通路导通；
- register readback 只能证明 ADS 配置，不能代替 DMM/示波器；
- 标称 10 kΩ 不能代替实际测量值；
- ADS1262 已有板上证据不等于所有硬件均为 ADS1262，固件仍按运行时 ID 区分 ADS1262/ADS1263 能力。

# tmuxSwitch / TMUX GPIO Control Primitive Layer

## 目录 / Table of contents

- [中文说明 / Chinese documentation](#中文说明--chinese-documentation)
- [Australian English documentation](#australian-english-documentation)
- [Kconfig](#kconfig)

## 中文说明 / Chinese documentation

### 职责

`components/tmuxSwitch` 是 TMUX1108/TMUX1134 的 GPIO/control primitive layer。它负责初始化 GPIO、选择 TMUX1108 row index、设置 TMUX1108 SW source、设置 TMUX1134 SELA/SELB/EN 逻辑电平，并提供 MCU-side control/readback snapshot。

它不知道：

- 哪个 row index 在业务上叫 S1/S2。
- 哪个 SELA/SELB 组合在板级语义上是 ADS 或 FDC。
- 哪个 D-line 属于 primary/secondary FDC。
- FDC frame 如何构建。
- 何时需要 rescue 或 sweep。

板级含义由 `core/board/sensorarrayBoardMap.c` 给出，测量策略由 `core/measure` 给出。

### 真实 API

| API | 作用 |
|---|---|
| `tmuxSwitchInit()` | Configure TMUX GPIO and initial software-commanded state. |
| `tmux1108SelectRow()`, `tmuxSwitchSelectRow()` | Select row index `0..7`. Measurement/board layers map this to S1..S8. |
| `tmux1108GetRow()` | Return last software-commanded row index. |
| `tmux1108SetSource()`, `tmuxSwitchSet1108Source()` | Select `TMUX1108_SOURCE_GND` or `TMUX1108_SOURCE_REF`. |
| `tmux1108GetSource()` | Return last software-commanded SW source. |
| `tmux1134SelectSelALevel()`, `tmux1134SelectSelBLevel()` | Drive raw SELA/SELB logic levels. |
| `tmux1134SetEnLogicalState()`, `tmux1134GetEnLogicalState()` | Control/read logical EN state. |
| `tmux1134SetSelAEnabled()`, `tmux1134SetSelBEnabled()` | Backward-compatible enabled wrappers using configured enabled levels. |
| `tmux1134SetAllOff()`, `tmux1134SetAllOn()` | Convenience wrappers. |
| `tmuxSwitchGetControlState()` | Capture last commanded state plus MCU GPIO observations. Observations do not prove analogue conduction. |

### 状态结构

`tmuxSwitchControlState_t` contains:

- software-commanded state: `cmdRow`, `cmdSource`, `cmdSwLevel`, `cmdSelaLevel`, `cmdSelbLevel`, `cmdTmux1134EnLogicalOn`;
- MCU-side observations: `obsA0Level`, `obsA1Level`, `obsA2Level`, `obsSwLevel`, `obsSelaLevel`, `obsSelbLevel`, `obsEnLevel`;
- EN controllability and readback validity flags.

### 层级关系

```text
core/board
  defines SELA_ROUTE_ADS1263/FDC2214 -> raw GPIO level
  defines board FDC SELB policy
core/measure
  decides route safety, settle delays, ADS/FDC mutual exclusion
components/tmuxSwitch
  only drives and observes GPIO-level control signals
```

当前板级 `CONFIG_TMUX1108_SW_REF_LEVEL=n` 表示 logical REF 对应 SW low，GND
对应 SW high。电路图中 SW high 使外部 Q1 将 TMUX1108 common/REFOUT 节点拉到
GND；SW low 释放该节点供 REFOUT 激励。因此 CAP/VOLT 使用 high/GND/no
excitation，RES 使用 low/REF/excitation。这个关系只由 `core/board` 的显式 mode
profile 消费，primitive layer 不从 high/low 自行决定业务模式。

TMUX1134 的板级真值为 SEL=1 选 A/FDC、SEL=0 选 B/ADS；SELA 管 D1..D4，
SELB 管 D5..D8。TMUX1134 支持 break-before-make，但 measurement layer 仍先撤销
激励、切 route、等待配置的 settle，再允许 conversion。GPIO readback 只证明 MCU
脚的观测值，不能替代模拟路径或示波器验证。

## Australian English documentation

### Responsibility

`components/tmuxSwitch` is the GPIO/control primitive layer for TMUX1108 and TMUX1134. It initialises GPIO, selects a TMUX1108 row index, selects the TMUX1108 SW source, drives TMUX1134 SELA/SELB/EN logic levels, and captures MCU-side control/readback snapshots.

It does not know business row names, ADS/FDC route meaning, D-line ownership, FDC frame building, or rescue policy. Board meaning comes from `core/board/sensorarrayBoardMap.c`; measurement policy comes from `core/measure`.

### API groups

- Initialisation: `tmuxSwitchInit()`.
- TMUX1108 row/source: `tmux1108SelectRow()`, `tmuxSwitchSelectRow()`, `tmux1108SetSource()`, `tmuxSwitchSet1108Source()`.
- TMUX1134 raw logic levels: `tmux1134SelectSelALevel()`, `tmux1134SelectSelBLevel()`, `tmux1134SetEnLogicalState()`.
- Compatibility wrappers: `tmux1134SetSelAEnabled()`, `tmux1134SetSelBEnabled()`, `tmux1134SetAllOff()`, `tmux1134SetAllOn()`.
- Diagnostics: `tmuxSwitchGetControlState()`.

GPIO observations are MCU pin observations only. They are useful diagnostics but do not prove the external analogue path is conducting correctly.

For this board, `CONFIG_TMUX1108_SW_REF_LEVEL=n` maps logical REF to physical
low and GND to physical high. The external Q1 makes high a grounded/passive
matrix state; low releases the REFOUT excitation path. The board map, not this
component, assigns those levels to CAP/VOLT/RES. TMUX1134 SEL=1 selects the
A/FDC branch and SEL=0 the B/ADS branch; SELA serves D1..D4 and SELB D5..D8.
Measurement policy still removes excitation and waits for analogue settling
around route changes even though the switch provides break-before-make.

## Kconfig

| Option | Default | Notes |
|---|---:|---|
| `CONFIG_TMUX1108_A0_GPIO`, `CONFIG_TMUX1108_A1_GPIO`, `CONFIG_TMUX1108_A2_GPIO` | `4`, `5`, `6` | TMUX1108 address pins. |
| `CONFIG_TMUX1108_SW_GPIO` | `7` | TMUX1108 SW source select pin. |
| `CONFIG_TMUX1108_SW_REF_LEVEL` | n | Current-board polarity: REF=low, GND=high. Change only with a reviewed board profile/schematic change. |
| `CONFIG_TMUX1108_DEFAULT_SOURCE` | `0` | Default source, `0=GND`, `1=REF`. |
| `CONFIG_TMUX1108_SWITCH_ROW_SAFE_MODE` | y | Optional safe row-switch policy in the primitive layer. |
| `CONFIG_TMUX1108_SAFE_SOURCE` | `0` | Safe source when safe mode is enabled. |
| `CONFIG_TMUX1108_SWITCH_DELAY_US` | `0` | Primitive switch delay. Measurement layer has separate FDC row settle timing. |
| `CONFIG_TMUX1134_SEL1_GPIO`, `CONFIG_TMUX1134_SEL2_GPIO`, `CONFIG_TMUX1134_SEL3_GPIO`, `CONFIG_TMUX1134_SEL4_GPIO` | `1`, `2`, `-1`, `-1` | TMUX1134 SEL GPIOs. SEL1 is SELA, SEL2 is SELB. |
| `CONFIG_TMUX1134_EN_GPIO` | `-1` | EN GPIO, or tied off when `-1`. |
| `CONFIG_TMUX1134_DEFAULT_ALL_OFF`, `CONFIG_TMUX1134_DEFAULT_SELA_ENABLED`, `CONFIG_TMUX1134_DEFAULT_SELB_ENABLED` | n, n, n | Primitive default state options. |
| `CONFIG_TMUX1134_SELA_ENABLED_LEVEL`, `CONFIG_TMUX1134_SELB_ENABLED_LEVEL`, `CONFIG_TMUX1134_EN_OFF_LEVEL` | `1`, `1`, `0` | Compatibility wrapper levels. Board route meaning is still defined in `core/board`. |

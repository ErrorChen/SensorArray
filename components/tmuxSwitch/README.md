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

## Kconfig

| Option | Default | Notes |
|---|---:|---|
| `CONFIG_TMUX1108_A0_GPIO`, `CONFIG_TMUX1108_A1_GPIO`, `CONFIG_TMUX1108_A2_GPIO` | `4`, `5`, `6` | TMUX1108 address pins. |
| `CONFIG_TMUX1108_SW_GPIO` | `7` | TMUX1108 SW source select pin. |
| `CONFIG_TMUX1108_SW_REF_LEVEL` | n | Defines which physical SW level selects REF. |
| `CONFIG_TMUX1108_DEFAULT_SOURCE` | `0` | Default source, `0=GND`, `1=REF`. |
| `CONFIG_TMUX1108_SWITCH_ROW_SAFE_MODE` | y | Optional safe row-switch policy in the primitive layer. |
| `CONFIG_TMUX1108_SAFE_SOURCE` | `0` | Safe source when safe mode is enabled. |
| `CONFIG_TMUX1108_SWITCH_DELAY_US` | `0` | Primitive switch delay. Measurement layer has separate FDC row settle timing. |
| `CONFIG_TMUX1134_SEL1_GPIO`, `CONFIG_TMUX1134_SEL2_GPIO`, `CONFIG_TMUX1134_SEL3_GPIO`, `CONFIG_TMUX1134_SEL4_GPIO` | `1`, `2`, `-1`, `-1` | TMUX1134 SEL GPIOs. SEL1 is SELA, SEL2 is SELB. |
| `CONFIG_TMUX1134_EN_GPIO` | `-1` | EN GPIO, or tied off when `-1`. |
| `CONFIG_TMUX1134_DEFAULT_ALL_OFF`, `CONFIG_TMUX1134_DEFAULT_SELA_ENABLED`, `CONFIG_TMUX1134_DEFAULT_SELB_ENABLED` | n, n, n | Primitive default state options. |
| `CONFIG_TMUX1134_SELA_ENABLED_LEVEL`, `CONFIG_TMUX1134_SELB_ENABLED_LEVEL`, `CONFIG_TMUX1134_EN_OFF_LEVEL` | `1`, `1`, `0` | Compatibility wrapper levels. Board route meaning is still defined in `core/board`. |

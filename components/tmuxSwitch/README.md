# tmuxSwitch / TMUX GPIO Control

`tmuxSwitch` 统一封装 TMUX1108 与 TMUX1134 的 GPIO 控制：行选择、SW 源切换、SELA/SELB 逻辑级设置、EN 控制和控制状态读取。它是通用控制层，不承载板级 route table。

## Main APIs

- `tmuxSwitchInit`
- `tmuxSwitchSelectRow`
- `tmuxSwitchSet1108Source`
- `tmux1134SelectSelALevel`
- `tmux1134SelectSelBLevel`
- `tmux1134SetEnLogicalState`
- `tmuxSwitchGetControlState`

## SW Source Semantics

- `TMUX1108_SOURCE_GND`: select the GND source in software terms.
- `TMUX1108_SOURCE_REF`: select the REF source in software terms.
- `RESISTANCE_READ / 电阻读取`: ADS126x voltage scan with `SW=REF`.
- `PIEZO_READ / 压电读取`: ADS126x voltage scan with `SW=GND`.
- `DEBUG / CAPACITIVE / FDC2214`: FDC2214 capacitive read with `SW=GND` unless a debug submode explicitly overrides the source.

Current SensorArray board polarity:

```text
SW = LOW  -> REF
SW = HIGH -> GND
```

Function mapping on this board:

```text
tmuxSwitchSet1108Source(TMUX1108_SOURCE_REF)
  -> drives SW LOW

tmuxSwitchSet1108Source(TMUX1108_SOURCE_GND)
  -> drives SW HIGH
```

On the current board, SW drives Q1 (`2N7002`) gate through R46 with R47 pulldown. `SW HIGH` turns Q1 on and pulls `REF/D` toward GND. Firmware must therefore keep ADS126x INTREF/REFOUT and VBIAS off whenever commanding `TMUX1108_SOURCE_GND`; otherwise REFOUT (~0.7 V vs system GND when AVSS=-1.8 V) fights the Q1 pulldown.

SW 高低电平由 `CONFIG_TMUX1108_SW_REF_LEVEL` 和底层实现决定，当前默认 `CONFIG_TMUX1108_SW_REF_LEVEL=n`，也就是 REF 为 LOW。调用者不要直接写 GPIO，也不要假设 raw GPIO level 等于某个模拟源。业务层应调用 `tmuxSwitchSet1108Source(TMUX1108_SOURCE_GND/REF)`。

## Readback Boundary

`tmuxSwitchGetControlState()` 同时返回软件命令状态和 MCU 侧 GPIO readback。GPIO readback 只能证明 MCU 输出/读回状态，不能证明外部模拟节点真的连到 GND、REF 或 `0.75 V` midpoint。若 ADS readback OK 但示波器上的 REF/MID 不对，应继续检查外部模拟网络、负压、焊接、buffer、SW 极性和 TMUX 实际通路。

## Integration Boundary

- 本驱动只关心 GPIO 与开关控制。
- `S/D/path` 到 SELA/SELB/SW 的板级策略属于 `main/sensorarrayBoardMap.c`、`main/sensorarrayVoltageScan.c` 和 debug/app 层。
- SELA 逻辑 route 到 GPIO level 的转换应通过 `sensorarrayBoardMapSelaRouteToGpioLevel()` 完成，不应在调用处硬编码。

## Kconfig Notes

- `CONFIG_TMUX1108_SW_REF_LEVEL` controls which SW GPIO level means REF. Current board default is false/low.
- Safe row switching can temporarily move SW to a configured safe source.
- TMUX1134 EN can be optional depending on board wiring.

# SensorArray architecture / SensorArray 架构

## 中文说明

当前源码树采用分层结构：

| 层 | 路径 | 责任 |
|---|---|---|
| 应用层 | `main/main.c` | 初始化编排、boot sweep 调用、主循环、safe idle、诊断模式和 frame output 调用。 |
| 输出层 | `main/output` | 当前 text frame output。 |
| 板级映射 | `core/board` | S/D、SELA/SELB、ADS mux 和 FDC D-line ownership 的硬件语义。 |
| 资源层 | `core/boardSupport` | I2C/SPI/GPIO resource lifecycle、FDC I2C callbacks 和 guarded I2C recovery。 |
| 测量层 | `core/measure` | ADS/FDC route policy、FDC row epoch、frame build、sweep/cache/rescue。 |
| 芯片驱动 | `components/*` | FDC2214 I2C、ADS126x SPI、TMUX GPIO primitives。 |
| transport | `transport/*` | protocol/transport scaffolding；当前不是默认 frame output。 |

关键边界：

- `main` 不直接访问 FDC2214/ADS126x registers。
- `components/fdc2214Cap` 不知道 rows、D-lines、TMUX route 或 rescue。
- `components/ads126xAdc` 不决定 FDC mode 下 ADS 何时关闭。
- `components/tmuxSwitch` 不知道 SELA/SELB 的板级业务含义。
- `core/board` 是硬件语义真源。
- `core/measure` 是测量策略真源。

## Australian English Documentation

The current source tree is layered:

- `main/main.c`: application lifecycle, boot sweep call, main loop, safe idle, diagnostic mode, and frame output call.
- `main/output`: current text frame output.
- `core/board`: hardware meaning for S/D, SELA/SELB, ADS mux, and FDC D-line ownership.
- `core/boardSupport`: I2C/SPI/GPIO resource lifecycle, FDC I2C callbacks, and guarded I2C recovery.
- `core/measure`: ADS/FDC route policy, FDC row epoch, frame build, sweep/cache/rescue.
- `components/*`: chip-level FDC2214 I2C, ADS126x SPI, and TMUX GPIO primitives.
- `transport/*`: protocol/transport scaffolding, not the default frame output path.

The main rule is that board meaning belongs in `core/board`, measurement policy belongs in `core/measure`, and chip drivers stay unaware of matrix semantics.

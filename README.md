# SensorArray Firmware / SensorArray 固件

## 1) Project Overview / 项目概览

**中文**

这是一个基于 ESP-IDF 的 ESP32-S3 固件仓库。当前代码结构已经按职责拆分为四层：

- `components/`: 可复用通用驱动（ADS126x、FDC2214、TMUX）。
- `core/`: 可复用项目辅助层（板级总线支持、矩阵引擎、电源占位层）。
- `main/`: 当前板卡与应用编排层（启动顺序、板级映射、测量与调试模式调度）。
- `transport/`: 协议与传输辅助层（wire 帧、USB 封装、占位传输模块）。

当前默认应用仍以 bring-up / debug 为主，不是最终量产业务流。

**English**

This repository is an ESP-IDF firmware project for ESP32-S3. The codebase is now split by responsibility into four layers:

- `components/`: reusable generic drivers (ADS126x, FDC2214, TMUX).
- `core/`: reusable project helpers (board bus support, matrix engine, power placeholder).
- `main/`: board/app orchestration (boot order, board mapping, measurement and debug mode dispatch).
- `transport/`: framing and transport helpers (wire frame, USB wrapper, placeholder transports).

The default app is still a bring-up/debug-oriented app, not a finished production pipeline.

## 2) Quick Navigation / 快速导航

| Path | Purpose (中文 / English) |
| --- | --- |
| `main/main.c` | 薄入口，仅调用 `sensorarrayAppRun()` / Thin entry that only calls `sensorarrayAppRun()` |
| `main/sensorarrayApp.c` | 应用层启动编排 / Application startup orchestration |
| `main/sensorarrayBoardMap.c` | 板级映射单一真相源 / Single source of truth for board mapping |
| `main/sensorarrayBringup.c` | SPI/ADS/FDC/TMUX bring-up / SPI/ADS/FDC/TMUX bring-up |
| `main/sensorarrayMeasure.c` | 路由应用与读数流程 / Route application and measurement flows |
| `main/sensorarrayDebug.c` | Debug 调度与路由模式 / Debug dispatch and route modes |
| `main/sensorarrayDebugSelftest.c` | ADS/FDC 自检模式 / ADS/FDC self-test modes |
| `main/sensorarrayDebugS1d1.c` | S1D1 专项调试模式 / S1D1 dedicated debug modes |
| `components/ads126xAdc` | 通用 ADS1262/1263 SPI 驱动 / Generic ADS1262/1263 SPI driver |
| `components/fdc2214Cap` | 通用 FDC2214 I2C 驱动 / Generic FDC2214 I2C driver |
| `components/tmuxSwitch` | 通用 TMUX GPIO 控制 / Generic TMUX GPIO control |
| `core/boardSupport` | 板级 I2C 初始化与回调适配 / Board I2C init and callback adapters |
| `core/matrixEngine` | 矩阵区域 I/O 引擎 / Matrix region I/O engine |
| `transport/protocolWire` | 400-byte 固定 wire 帧 / 400-byte fixed wire frame |
| `transport/protocolUsb` | USB 头封装与流解析 / USB wrapper header and stream parser |

## 3) Architecture (Text) / 架构文本图

```text
app_main (main/main.c)
  -> sensorarrayAppRun (main/sensorarrayApp.c)
     -> boardSupportInit / tmuxSwitchInit / ADS/FDC bring-up
     -> board mapping audit logs
     -> selected debug mode dispatch

Reusable layers:
  components (generic drivers)
    ads126xAdc   fdc2214Cap   tmuxSwitch

  core (project helpers)
    boardSupport   matrixEngine   powerCtrl(placeholder)

  transport (framing/transport)
    protocolWire   protocolUsb   transportWire(placeholder)   bleTransport(placeholder)
```

## 4) Canonical Board Mapping / 当前权威板级映射

> 本节是当前仓库映射的权威定义；`main/sensorarrayBoardMap.c` 与相关 README 必须与此一致。  
> This section is the canonical mapping definition; `main/sensorarrayBoardMap.c` and related READMEs must stay aligned.

### ADS Path / ADS 路径

- `D1..D8 -> AIN0..AIN7`（差分负端统一对 `AINCOM`）。
- `AIN8` 用于电池电压观测（当前应用调试未默认扫描）。
- `AIN9` 与 `AINCOM` 在当前板上短接，用作参考/辅助观测对。

- `D1..D8 -> AIN0..AIN7` (negative input uses `AINCOM` in current app flows).
- `AIN8` is reserved for battery voltage observation (not part of default scan loop).
- `AIN9` is shorted to `AINCOM` on this board for reference/auxiliary observation pairs.

### FDC Path / FDC 路径

- `D1..D4 -> primary FDC (SELA side), CH0..CH3`。
- `D5..D8 -> secondary FDC (SELB side), CH0..CH3`。

- `D1..D4 -> primary FDC (SELA side), CH0..CH3`.
- `D5..D8 -> secondary FDC (SELB side), CH0..CH3`.

### TMUX Path / TMUX 路径

- `TMUX1108`: 负责 `S1..S8` 行选择与 `SW` 源切换（REF/GND）。
- `TMUX1134`: 负责列组/路径相关选择（SELA/SELB 逻辑级）。

- `TMUX1108`: selects `S1..S8` rows and controls `SW` source (REF/GND).
- `TMUX1134`: controls column/path selection through SELA/SELB logic levels.

## 5) Build / 构建

**中文**

```bash
idf.py set-target esp32s3
idf.py menuconfig
idf.py build
```

**English**

```bash
idf.py set-target esp32s3
idf.py menuconfig
idf.py build
```

说明 / Notes:

- 已验证当前重构版本可完成 `idf.py build`。
- 若出现 `ESP_ROM_ELF_DIR` 的 gdbinit 警告，不影响固件二进制生成。

- The refactored code has been validated with `idf.py build`.
- An `ESP_ROM_ELF_DIR` gdbinit warning may appear; it does not block firmware binary generation.

## 6) Module Boundaries / 模块边界

**中文**

- 通用驱动边界：`components/*` 不承载板级路由语义。
- 板级映射边界：`main/sensorarrayBoardMap.c` 是映射单一真相源。
- 应用编排边界：`main/sensorarrayApp.c` 负责启动与模式调度。
- 测量边界：`main/sensorarrayMeasure.c` 负责 route/read/retry/discard/换算。
- 调试边界：`main/sensorarrayDebug*.c` 负责各 debug mode 执行体。

**English**

- Generic driver boundary: `components/*` does not carry board-specific route semantics.
- Board mapping boundary: `main/sensorarrayBoardMap.c` is the single source of mapping truth.
- App orchestration boundary: `main/sensorarrayApp.c` handles startup and mode dispatch.
- Measurement boundary: `main/sensorarrayMeasure.c` owns route/read/retry/discard/conversion flow.
- Debug boundary: `main/sensorarrayDebug*.c` owns debug mode execution logic.

## 7) Current Status / 当前状态

**中文**

- `main/main.c` 已瘦身为薄入口。
- 启动、映射、测量、日志、调试逻辑已分拆到独立文件。
- `matrixEngine` 默认 ADS 映射已与 canonical mapping 对齐为 `AIN0..AIN7` 顺序。

**English**

- `main/main.c` has been reduced to a thin entry.
- Startup, mapping, measurement, logging, and debug logic are split into dedicated files.
- `matrixEngine` default ADS mapping now matches canonical `AIN0..AIN7` order.

## 8) Next TODO / 后续建议

**中文**

- 将 route map 从静态调试条目扩展为完整 8x8 业务映射（若硬件与算法确认）。
- 在 `main/` 增加自动化单元测试/集成测试桩（Host 或 HIL）。
- 把 `transportWire`、`bleTransport` 从占位实现推进到真实链路。

**English**

- Expand route map from debug-centric entries to full 8x8 production mapping once hardware/algorithm is finalized.
- Add unit/integration test scaffolding for `main/` (host-side or HIL).
- Evolve `transportWire` and `bleTransport` from placeholders into real transport paths.

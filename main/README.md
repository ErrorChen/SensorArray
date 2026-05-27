# main / 应用层入口

## 1) Purpose / 目标

**中文**

`main/` 只负责系统生命周期编排：
- `app_main` 入口
- startup 顺序
- debug task 启动

`main/main.c` 只保留 `app_main -> sensorarrayAppRun()` 薄入口。

**English**

`main/` owns system lifecycle orchestration only:
- `app_main` entry
- startup sequencing
- debug task launch

`main/main.c` is intentionally a thin entry: `app_main -> sensorarrayAppRun()`.

## 2) File Split / 文件拆分

- `sensorarrayApp.c/.h`: 顶层流程编排与模式分流。
- `sensorarrayConfig.h`: 本层共享配置默认值与常量。
- `sensorarrayTypes.h`: 跨文件共享类型定义。
- `../core/board/sensorarrayBoardMap.c/.h`: 板级映射单一真相源。
- `../core/board/sensorarrayBringup.c/.h`: SPI/ADS/FDC/TMUX bring-up。
- `../core/measure/sensorarrayMeasure.c/.h`: route 应用、采样、重试/丢弃、换算。
- `sensorarrayLog.c/.h`: `INIT,...` / `DBG,...` / `DBGCTRL,...` 日志辅助。
- `../core/debug/sensorarrayDebug.c/.h`: debug task 调度器 + 通用路由调试模式。
- `../core/debug/sensorarrayDebugFdcSelbS5d5.c/.h`: `S5D5` / SELB / secondary FDC2214 专用 bring-up 调试模式。
- `../core/debug/sensorarrayDebugSelftest.c/.h`: ADS/FDC 自检模式。
- `../core/debug/sensorarrayDebugS1d1.c/.h`: S1D1 电阻专项调试模式。

## 3) Canonical Mapping / 权威映射

- ADS: `D1..D8 -> AIN0..AIN7`, `AIN8=battery`, `AIN9 shorted to AINCOM`.
- FDC: `D1..D4 -> primary(SELA) CH0..CH3`, `D5..D8 -> secondary(SELB) CH0..CH3`.
- TMUX: `TMUX1108` 行与 SW 源切换；`TMUX1134` 列组/路径 SEL 控制。
- Capacitive routes must use the logical `FDC2214` SELA branch and let `sensorarrayBoardMap.c` translate that into raw GPIO levels.

## 4) Debug Modes / 调试模式

由 `main/Kconfig.projbuild` 控制，包含：

- `ROUTE_IDLE`
- `ROUTE_FIXED_STATE`
- `ROUTE_STEP_ONCE`
- `ROUTE_SCAN_LOOP`
- `ADS_SELFTEST`
- `FDC_SELFTEST`
- `S1D1_RESISTOR_DEBUG`
- `S5D5_CAP_FDC_SECONDARY`
- `FDC_I2C_DISCOVERY`

## 5) FDC Clock / FDC 时钟

- FDC2214/FDC2212 defaults to external `CLKIN` at 40 MHz:
  `CONFIG_SENSORARRAY_FDC_REF_CLOCK_USE_EXTERNAL=y` and
  `CONFIG_SENSORARRAY_FDC_EXTERNAL_CLOCK_HZ=40000000`.
- External mode sets CONFIG register `0x1A` bit9 (`REF_CLK_SRC`) and all raw to
  frequency/capacitance conversion uses the selected effective FCLK.
- Internal oscillator mode can still be selected through Kconfig/fallback
  settings for debug, but it is no longer the default.
- Hardware must provide the 40 MHz signal on the FDC `CLKIN` pin. The firmware
  does not generate this clock from the ESP32; without the external clock,
  conversions may fail in external-clock mode.

## 6) Boundary Rules / 边界规则

**中文**

- 板级映射与 bring-up 只放在 `core/board`。
- 通用驱动不吸收板级 route/debug 逻辑。
- 可复用测量策略放 `core/measure`，避免散落在调试模式里。
- debug/scan/sweep 执行体放 `core/debug`，复杂模式必须独立 task 化。
- SELA 逻辑路径到 raw GPIO 电平的翻译只允许留在 board-map 层。

**English**

- Board mapping and bring-up live only in `core/board`.
- Generic drivers must not absorb board-specific route/debug logic.
- Reusable measurement policy stays in `core/measure` instead of being duplicated in debug code.
- Debug/scan/sweep bodies stay in `core/debug`; complex modes must run in their own task.
- Logical SELA route to raw GPIO translation must stay in the board-map layer.

## 7) Current Status / 当前状态

- 入口已完成去 god-file 化重构。
- 调试执行体已按“调度 / 自检 / 单点电容 / S1D1 专项”拆分。
- 复杂 debug/scan/sweep 由独立 FreeRTOS task 执行。
- 新增 `FDC_I2C_DISCOVERY`：持续轮询 `I2C0/I2C1 x 0x2A/0x2B`，仅用于确认 I2C ACK 与 ID 可读性。

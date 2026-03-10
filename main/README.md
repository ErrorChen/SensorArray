# main / 应用层入口

## 1) Purpose / 目标

**中文**

`main/` 负责当前板卡的应用层编排：
- 启动顺序
- 板级映射
- 测量策略
- debug 模式调度

`main/main.c` 只保留 `app_main -> sensorarrayAppRun()` 薄入口。

**English**

`main/` owns board/app orchestration for the current hardware:
- startup sequencing
- board mapping
- measurement policy
- debug mode dispatch

`main/main.c` is intentionally a thin entry: `app_main -> sensorarrayAppRun()`.

## 2) File Split / 文件拆分

- `sensorarrayApp.c/.h`: 顶层流程编排与模式分流。
- `sensorarrayConfig.h`: 本层共享配置默认值与常量。
- `sensorarrayTypes.h`: 跨文件共享类型定义。
- `sensorarrayBoardMap.c/.h`: 板级映射单一真相源。
- `sensorarrayBringup.c/.h`: SPI/ADS/FDC/TMUX bring-up。
- `sensorarrayMeasure.c/.h`: route 应用、采样、重试/丢弃、换算。
- `sensorarrayLog.c/.h`: `INIT,...` / `DBG,...` / `DBGCTRL,...` 日志辅助。
- `sensorarrayDebug.c/.h`: 调度器 + 路由调试模式。
- `sensorarrayDebugSelftest.c`: ADS/FDC 自检模式。
- `sensorarrayDebugS1d1.c`: S1D1 专项调试模式。

## 3) Canonical Mapping / 权威映射

- ADS: `D1..D8 -> AIN0..AIN7`, `AIN8=battery`, `AIN9 shorted to AINCOM`.
- FDC: `D1..D4 -> primary(SELA) CH0..CH3`, `D5..D8 -> secondary(SELB) CH0..CH3`.
- TMUX: `TMUX1108` 行与 SW 源切换；`TMUX1134` 列组/路径 SEL 控制。

## 4) Debug Modes / 调试模式

由 `main/Kconfig.projbuild` 控制，包含：

- `ROUTE_IDLE`
- `ROUTE_FIXED_STATE`
- `ROUTE_STEP_ONCE`
- `ROUTE_SCAN_LOOP`
- `ADS_SELFTEST`
- `FDC_SELFTEST`
- `S1D1_RESISTOR_DEBUG`
- 以及独立 `S1D1_STATIC` / `ADS_S1D1_ONLY` 路径

## 5) Boundary Rules / 边界规则

**中文**

- 板级映射只在 `sensorarrayBoardMap.c`。
- 通用驱动不吸收板级 route/debug 逻辑。
- 测量策略放 `sensorarrayMeasure.c`，避免散落在调试模式里。

**English**

- Board mapping lives only in `sensorarrayBoardMap.c`.
- Generic drivers must not absorb board-specific route/debug logic.
- Measurement policy stays in `sensorarrayMeasure.c` instead of being duplicated in debug code.

## 6) Current Status / 当前状态

- 入口已完成去 god-file 化重构。
- 调试执行体已按“调度 / 自检 / S1D1 专项”拆分。
- 构建已通过（`idf.py build`）。

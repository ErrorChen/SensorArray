# matrixEngine / 矩阵引擎

## 1) Scope / 模块范围

**中文**

`matrixEngine` 提供同步矩形区域 I/O（READ/WRITE）执行框架：
- 调用行选择回调
- 调用列组选择回调
- 调用 ADS/FDC 读数
- 输出 row-major 数据

它是可复用引擎，不应固化某块板的完整业务路由策略。

**English**

`matrixEngine` provides a synchronous rectangular region I/O executor (READ/WRITE):
- row-select callback orchestration
- column-group callback orchestration
- ADS/FDC reads
- row-major output

It is reusable engine logic and should not hardcode a full board-specific business route strategy.

## 2) Default Behavior / 默认行为

当未提供自定义映射/回调时，默认行为是：

- 行：`row 0..7` 对应 TMUX1108 行选择。
- 列组：默认 4 列一组（group0/group1）。
- ADS 默认列映射：`col0..col7 -> AIN0..AIN7`，`MUXN=AINCOM`。
- FDC 默认：每 4 列一颗器件，通道 `col % 4`。

When custom mappings/callbacks are not provided:

- Rows: `row 0..7` through TMUX1108 row selection.
- Column group: default 4 columns per group.
- ADS default mapping: `col0..col7 -> AIN0..AIN7`, `MUXN=AINCOM`.
- FDC default: one device per 4-column bank, channel `col % 4`.

## 3) Public API / 对外接口

Header: `core/matrixEngine/include/matrixEngine.h`

- `matrixEngineInit(const matrixEngineConfig_t *cfg)`
- `matrixEngineRegionIo(const matrixEngineRegion_t *region, const matrixEngineRequest_t *req, int32_t *outValues, size_t outCount)`
- `matrixEngineDeinit(void)`

## 4) Boundary with App Layer / 与应用层边界

**中文**

- `matrixEngine` 是可选引擎，当前默认 `main` 应用未接入主流程。
- 当前板 canonical mapping 由 `main/sensorarrayBoardMap.c` 维护；若你要用 `matrixEngine` 处理同一板，请在 `matrixEngineConfig_t` 中显式传入一致映射。

**English**

- `matrixEngine` is optional and not currently the default app execution path.
- Canonical board mapping is maintained in `main/sensorarrayBoardMap.c`; pass explicit tables in `matrixEngineConfig_t` if you need strict alignment.

## 5) Current Status / 当前状态

- Core API is usable.
- Recommended next step: integrate with transport pipeline and task scheduling when production scan flow is defined.

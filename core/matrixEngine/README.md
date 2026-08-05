# matrixEngine / 矩阵区域 I/O 引擎

## 目录 / Table of contents

- [中文说明 / Chinese documentation](#中文说明--chinese-documentation)
- [Australian English documentation](#australian-english-documentation)
- [Kconfig](#kconfig)

## 中文说明 / Chinese documentation

`core/matrixEngine` 是一个可复用的同步矩形区域 I/O executor。它可以在 caller 提供配置后，按 row-major 顺序执行 voltage、raw capacitance 或 resistance 读取。当前 `main` 默认生命周期没有使用它；FDC production path 使用 `core/measure/fdc` 的 row epoch 实现。

三模式实现也没有把它强行接入生产路径：这个模块的单 mutex、同步 callback
执行模型无法保持既有双 FDC worker row epoch，也不提供模式 accepted/applied、
route readback、fresh DRDY generation、ADS autorange 或单次 fixed-slot 输出契约。
生产 CAP 继续走 `core/measure/fdc`，VOLT/RES 走
`core/measure/ads/sensorarrayAdsMatrix`；此处只适合作为脱离生产并发域的通用 I/O
工具。其默认 `col -> AIN` 假设不得取代 `core/board` 的正式映射。

### API

| API | 作用 |
|---|---|
| `matrixEngineInit(const matrixEngineConfig_t *cfg)` | 保存配置，安装默认 row/column/drive callbacks，创建 mutex。 |
| `matrixEngineRegionIo(const matrixEngineRegion_t *region, const matrixEngineRequest_t *req, int32_t *outValues, size_t outCount)` | 对矩形区域执行 READ/WRITE；READ 输出 row-major values。 |
| `matrixEngineDeinit()` | 删除 mutex 并清空 engine state。 |

### 边界

- 默认 row callback 调用 `tmuxSwitchSelectRow(row)`。
- 默认 column group callback 用 TMUX1134 SELA/SELB enabled wrappers。
- 默认 ADS mux 假设 `col0..col7 -> AIN0..AIN7` against AINCOM。
- 默认 FDC raw path按 4-column bank 和 `col % 4` 选择 FDC channel。
- Canonical board map 仍在 `core/board/sensorarrayBoardMap.c`；如果需要严格板级映射，应通过 `matrixEngineConfig_t` 显式传入 mapping/callback。

## Australian English documentation

`core/matrixEngine` is a reusable synchronous rectangular region I/O executor. After a caller provides configuration, it can perform voltage, raw capacitance, or resistance reads in row-major order. It is not the default `main` lifecycle path; the production FDC path uses the row epoch implementation in `core/measure/fdc`.

The three-mode runtime deliberately does not force this synchronous,
single-mutex callback model into production. It does not preserve the dual-FDC
worker epoch contract or provide accepted/applied mode state, route readback,
fresh DRDY generations, ADS autorange, and one fixed-slot formatter. Production
CAP therefore stays in `core/measure/fdc`, while VOLT/RES use
`core/measure/ads/sensorarrayAdsMatrix`. Its default column mapping is not a
substitute for the canonical board map.

Use this module as a generic engine only. Do not put board-specific route meaning or rescue policy into it.

## Kconfig

| Option | Default | Notes |
|---|---:|---|
| `CONFIG_MATRIX_ROWS`, `CONFIG_MATRIX_COLS` | derive from SensorArray defaults | Region validation limits. |
| `CONFIG_MATRIX_FRAME_PERIOD_MS` | `CONFIG_SENSORARRAY_FRAME_PERIOD_MS` | Generic matrix setting, not current FDC production timing. |
| `CONFIG_MATRIX_OVERSAMPLE` | `CONFIG_SENSORARRAY_OVERSAMPLE` | Default oversample when config does not set `oversample`. |
| `CONFIG_MATRIX_USE_RINGBUFFER` | y | Reserved/shared matrix transport option. |
| `CONFIG_MATRIX_SCAN_TASK_*`, `CONFIG_MATRIX_COMM_TASK_*` | derive from SensorArray task defaults | Scheduling defaults for future integration. |

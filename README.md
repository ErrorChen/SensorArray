# SensorArray ESP32-S3 固件 / SensorArray ESP32-S3 Firmware

## 目录 / Table of contents

- [当前源码树状态 / Current source-tree status](#当前源码树状态--current-source-tree-status)
- [项目目标 / Project purpose](#项目目标--project-purpose)
- [当前实现状态 / Current implementation status](#当前实现状态--current-implementation-status)
- [硬件拓扑 / Hardware topology](#硬件拓扑--hardware-topology)
- [软件架构 / Software architecture](#软件架构--software-architecture)
- [函数级生命周期 / Function-level lifecycle](#函数级生命周期--function-level-lifecycle)
- [配置项 / Configuration options](#配置项--configuration-options)
- [关键数据结构 / Key data structures](#关键数据结构--key-data-structures)
- [FDC 矩阵读取路径 / FDC matrix read path](#fdc-矩阵读取路径--fdc-matrix-read-path)
- [路由安全与 ADS/FDC 互斥 / Route safety and ADS/FDC mutual exclusion](#路由安全与-adsfdc-互斥--route-safety-and-adsfdc-mutual-exclusion)
- [帧格式与无效值策略 / Frame format and invalid data policy](#帧格式与无效值策略--frame-format-and-invalid-data-policy)
- [调试日志与控制命令 / Diagnostic logs and console commands](#调试日志与控制命令--diagnostic-logs-and-console-commands)
- [编译、烧录和监视 / Build, flash, and monitor](#编译烧录和监视--build-flash-and-monitor)
- [已知约束 / Known constraints](#已知约束--known-constraints)
- [文件地图 / File map](#文件地图--file-map)

## 当前源码树状态 / Current source-tree status

### 中文

本文档描述当前源码树的主要固件结构、运行链路、配置项和调试方法。最终真源是源码本身，尤其是 `main/Kconfig.projbuild`、`main/sensorarrayConfig.h`、`main/main.c`、`core/measure/`、`core/board/` 和各组件头文件。README 正文不写死具体 commit 号。

### Australian English

This document describes the firmware structure, runtime flow, configuration options, and diagnostic workflow for the current source tree. The final source of truth is the code itself, especially `main/Kconfig.projbuild`, `main/sensorarrayConfig.h`, `main/main.c`, `core/measure/`, `core/board/`, and the component headers. The README body does not pin a specific commit hash.

## 项目目标 / Project purpose

### 中文

SensorArray 固件运行在 ESP32-S3 上，当前主路径是读取 8 x 8 FDC2214 电容矩阵。固件把板级路由、芯片驱动、测量策略和应用调度拆开：板级映射定义硬件语义，组件驱动只访问芯片，`core/measure` 负责测量策略，`main` 负责生命周期编排和输出调用。

### Australian English

SensorArray firmware runs on ESP32-S3. The current main path reads an 8 x 8 FDC2214 capacitance matrix. The firmware keeps board routing, chip drivers, measurement policy, and application scheduling separate: board mapping defines hardware meaning, component drivers only access chips, `core/measure` owns measurement policy, and `main` orchestrates lifecycle and output calls.

## 当前实现状态 / Current implementation status

| 路径 / Path | 当前状态 / Current status | 说明 / Notes |
|---|---|---|
| FDC2214 8 x 8 cap matrix | 主运行路径 / main runtime path | `sensorarrayFdcMatrixEngineReadFrame()` delegates to `sensorarrayMeasureReadFdcMatrixFrame()`. |
| ADS1262/ADS1263 matrix | 初始化和 API 存在，frame read returns unsupported / initialisation and APIs exist, frame read returns unsupported | `sensorarrayAdsMatrixEngineReadFrame()` initialises an invalid frame and returns `ESP_ERR_NOT_SUPPORTED`. |
| Mixed row mode | Thin pass-through to FDC path / thin pass-through to FDC path | `sensorarrayMixedRowEngineReadFrame()` currently calls the FDC matrix engine. |
| Text output | 默认主输出 / default main output | `main/output/sensorarrayFrameOutput.c` prints `FDC_FRAME_OUTPUT`, `MATRIXFDC_CAP`, optionally `MATRIXFDC_FREQ` and `DEBUGFDC_RAW`. |
| Binary transport | 非默认且未实现发送 / not default and send is unsupported | `sensorarrayFastSpeedSetEnabled(false)` runs at init; `sensorarrayTransportSendFdcMatrixFrame()` returns `ESP_ERR_NOT_SUPPORTED`. |
| Runtime console commands | 当前源码树未发现注册 / not found in this source tree | No `esp_console_cmd_register` call is present. Profile and discard setters are C APIs, not serial commands. |

## 硬件拓扑 / Hardware topology

### 中文

板级映射是硬件语义的唯一真源。D-line、FDC 通道、SELA/SELB 逻辑电平和 ADS mux 的含义由 `core/board/sensorarrayBoardMap.c` 定义，不由芯片驱动定义。

### Australian English

The board map is the single source of truth for hardware meaning. D-line ownership, FDC channels, SELA/SELB logic levels, and ADS mux meaning are defined by `core/board/sensorarrayBoardMap.c`, not by chip drivers.

| 硬件 / Hardware | 在本项目中的作用 / Role in this project |
|---|---|
| ESP32-S3 | 主 MCU，运行 ESP-IDF 应用、FreeRTOS 任务、I2C/SPI/GPIO 控制和 printf 输出。 / Main MCU running the ESP-IDF app, FreeRTOS tasks, I2C/SPI/GPIO control, and printf output. |
| TMUX1108 | S1..S8 行选择和 SW GND/REF 源选择；`tmuxSwitchSelectRow(row - 1)` 选择当前行。 / Selects S1..S8 rows and SW GND/REF source; `tmuxSwitchSelectRow(row - 1)` selects the current row. |
| TMUX1134 | 前端路由开关；SELA 选择 ADS1263 或 FDC2214 分支，SELB 按板级 FDC 策略选择，EN 使能路由。 / Front-end route switch; SELA selects ADS1263 or FDC2214 branch, SELB follows the board FDC policy, and EN enables the route. |
| FDC2214 primary | D1-D4，CH0-CH3，默认 I2C 地址 `0x2B`。 / D1-D4, CH0-CH3, default I2C address `0x2B`. |
| FDC2214 secondary | D5-D8，CH0-CH3，默认 I2C 地址 `0x2A`。 / D5-D8, CH0-CH3, default I2C address `0x2A`. |
| ADS1262/ADS1263 | ADS 前端和 SPI driver 已存在；FDC matrix mode 下由 measurement layer 停止转换、关闭 internal reference 和 VBIAS。 / ADS frontend and SPI driver exist; in FDC matrix mode the measurement layer stops conversion and turns internal reference and VBIAS off. |
| LM27762 / negative rail | 原理图/资料中的模拟电源背景；当前源码树没有专用运行时 driver。 / Analogue power context from schematic/datasheets; no dedicated runtime driver in this source tree. |
| TPS631000 / 3.3 V rail | 电源架构背景；`core/powerCtrl` 只提供 GPIO abstraction。 / Power architecture context; `core/powerCtrl` only provides GPIO abstraction. |
| BQ24074 / charger | 充电器硬件背景；当前源码树没有 charger protocol driver。 / Charger hardware context; no charger protocol driver in this source tree. |
| USB Type-C | 供电/串口监视入口，实际传输仍以 printf text 为默认。 / Power and serial monitor entry; actual runtime transport defaults to printf text. |
| FPC/FFC connector | 传感阵列连接器，S/D 语义由 board map 解释。 / Sensor-array connector; S/D meaning is interpreted by the board map. |

矩阵索引为行优先：

```text
index = (sIndex - 1) * 8 + (dIndex - 1)
S1D1, S1D2, ..., S1D8, S2D1, ..., S8D8
D1-D4 -> primary FDC2214 CH0-CH3
D5-D8 -> secondary FDC2214 CH0-CH3
```

## 软件架构 / Software architecture

```mermaid
flowchart LR
    Board[core/board<br/>Board map] --> Measure[core/measure<br/>Measurement policy]
    Support[core/boardSupport<br/>I2C/SPI/GPIO resources] --> Drivers[components<br/>Chip drivers]
    Drivers --> Measure
    Measure --> Main[main/main.c<br/>Lifecycle scheduler]
    Main --> Output[main/output<br/>Text frame output]
```

| 层 / Layer | 路径 / Path | 负责 / Owns | 不负责 / Does not own |
|---|---|---|---|
| Application | `main/main.c` | app lifecycle, safe idle, boot calibration call, main loop, frame output call | chip registers, matrix routing algorithm, board map |
| Output | `main/output` | text frame formatting and future output adapter boundary | measurement state mutation |
| Board map | `core/board` | S/D mapping, SELA/SELB meaning, ADS mux meaning, route audit logs | generic chip access |
| Board support | `core/boardSupport` | I2C bus install/delete, I2C callbacks, guarded timeout recovery | FDC rescue policy |
| Measurement | `core/measure` | route policy, ADS/FDC mutual exclusion, FDC frame builder, row epoch, rescue decisions | low-level register names as public policy |
| FDC engine facade | `core/measure/fdc/sensorarrayFdcMatrix.c` | thin engine wrapper | actual scanning algorithm |
| FDC sweep/rescue | `core/measure/fdc/sensorarrayFdcSweep.c`, `sensorarrayFdcRescue.c` | boot sweep, cache construction, rescue throttling | app lifecycle |
| Chip drivers | `components/fdc2214Cap`, `components/ads126xAdc`, `components/tmuxSwitch` | FDC I2C, ADS SPI, GPIO primitives | rows, D-line meaning, frame output, rescue strategy |
| Transport | `transport/*` | protocol/transport scaffolding | current production frame output |

## 函数级生命周期 / Function-level lifecycle

```mermaid
flowchart TD
    A[app_main] --> B[sensorarrayInitSystem]
    B --> C[sensorarrayInitRuntime]
    B --> D[sensorarrayInitBoardAndRouting]
    B --> E[sensorarrayInitFrontends]
    B --> F[sensorarrayBuildDefaultScanPlan]
    A --> G[sensorarrayRunBootCalibration]
    G --> H[sensorarrayFdcMatrixEngineRunBootSweep]
    H --> I[sensorarrayFdcSweepRunBoot]
    A --> J[sensorarrayRunMainLoop]
    J --> K[sensorarrayRunQueuedFullSweep]
    J --> L[sensorarrayRunOneFrame]
    L --> M[sensorarrayFdcMatrixEngineReadFrame]
    M --> N[sensorarrayMeasureReadFdcMatrixFrame]
    J --> O[sensorarrayFrameOutputPrint]
    J --> P[sensorarrayRuntimeRescueTick]
    J --> Q[sensorarrayDelayFramePeriodSince]
```

| 函数 / Function | 输入 / Input | 主要副作用 / Main side effects | 失败处理 / Failure handling | 相关配置 / Related config |
|---|---|---|---|---|
| `app_main()` | global `s_appContext` | clears context, runs init, boot calibration, main loop | init failure prints `APP_FATAL` and enters safe idle; boot failure sets diagnostic mode when boot sweep is required | `CONFIG_SENSORARRAY_FDC_BOOT_SWEEP_REQUIRED` |
| `sensorarrayInitRuntime()` | `sensorarrayAppContext_t *ctx` | clears ctx, disables fast-speed output, sets `runtimeMode = SENSORARRAY_RUNTIME_MODE_FDC_MATRIX`, parses FDC I2C addresses and channel count | invalid ctx returns `ESP_ERR_INVALID_ARG` | `CONFIG_SENSORARRAY_FDC_PRIMARY_I2C_ADDR`, `CONFIG_SENSORARRAY_FDC_SECONDARY_I2C_ADDR`, `CONFIG_FDC2214CAP_CHANNELS` |
| `sensorarrayInitBoardAndRouting()` | app context state | runs `boardSupportInit()`, `tmuxSwitchInit()`, `sensorarrayBoardMapAudit()`, default TMUX route | board support failure returns fatal init error; TMUX failure returns error | `CONFIG_BOARD_I2C*`, `CONFIG_TMUX*` |
| `sensorarrayInitFrontends()` | app context state | initialises ADS, assigns FDC I2C contexts, initialises primary/secondary FDC, logs parallel bus config, initialises engines and rescue context | engine init failure aborts init; individual FDC state is tracked in `state.fdcPrimary/Secondary.ready` | `CONFIG_SENSORARRAY_FDC_PARALLEL_DUAL_BUS_READ`, `CONFIG_ADS126X_*` |
| `sensorarrayBuildDefaultScanPlan()` | app context | builds 8 rows x 8 cells with `SENSORARRAY_CELL_OP_FDC_CAP` | no error return | `CONFIG_SENSORARRAY_MATRIX_ROWS`, `CONFIG_SENSORARRAY_MATRIX_COLS` |
| `sensorarrayRunBootCalibration()` | app context | checks FDC readiness, runs boot sweep, stores `fdcBootSummary`, sets `fdcBootSweepOk`, `fdcDegradedMode` and `fdcDiagnosticMode` | primary missing is fatal; secondary missing is fatal only when dual FDC is required; required boot quality must be `OK` | `CONFIG_SENSORARRAY_REQUIRE_DUAL_FDC_FOR_BOOT`, `CONFIG_SENSORARRAY_FDC_BOOT_SWEEP_REQUIRED`, `CONFIG_SENSORARRAY_FDC_BOOT_MIN_VALID_CELLS` |
| `sensorarrayRunMainLoop()` | app context | consumes full-sweep requests, reads frames, prints diagnostics/output, ticks rescue, delays to frame period | diagnostic mode prints `MATRIXFDC_DIAG`; all-invalid frame triggers rescue tick; frame error logs `FRAME_ERROR` | `CONFIG_SENSORARRAY_FDC_MATRIX_PERIOD_MS`, rescue and timing configs |
| `sensorarrayRunQueuedFullSweep()` | app context | runs queued full matrix rescue via `sensorarrayFdcMatrixEngineRunFullRescue()` | skips while running, inside cooldown, or after max failed full sweeps; can force diagnostic mode | `CONFIG_SENSORARRAY_FDC_FULL_SWEEP_REQUEST_COOLDOWN_MS`, `CONFIG_SENSORARRAY_FDC_MAX_CONSECUTIVE_FULL_SWEEP_FAILS` |
| `sensorarrayRunOneFrame()` | app context | dispatches by `runtimeMode` | unsupported ADS mode returns `ESP_ERR_NOT_SUPPORTED`; unknown mode returns invalid state | runtime mode enum |
| `sensorarrayDelayFramePeriodSince()` | frame start timestamp, sequence | sleeps remaining period or prints compact `OV` overrun diagnostics | no return value | `CONFIG_SENSORARRAY_FDC_MATRIX_PERIOD_MS` |

## 配置项 / Configuration options

### 说明 / Notes

### 中文

`main/Kconfig.projbuild` 是项目自定义配置的主入口。`sensorarrayConfig.h` 为缺失配置提供编译 fallback；当 ESP-IDF Kconfig 生成 `sdkconfig.h` 时，以 Kconfig/defaults 为准。`sdkconfig.defaults` 和 `sdkconfig.defaults.esp32s3` 会覆盖部分 Kconfig default，例如当前默认 FDC frame period 为 `50 ms`，目标约 `20 fps`。如果把 period 改成 `250 ms`，目标帧率约为每秒 4 帧；真实帧率仍受 I2C、row settle、FDC conversion、日志输出和 rescue 活动影响。

### Australian English

`main/Kconfig.projbuild` is the main project configuration entry. `sensorarrayConfig.h` provides compile-time fallbacks for missing symbols; when ESP-IDF generates `sdkconfig.h`, Kconfig/defaults are authoritative. `sdkconfig.defaults` and `sdkconfig.defaults.esp32s3` override some Kconfig defaults. The current default FDC frame period is `50 ms`, or roughly a `20 fps` target. If the period is changed to `250 ms`, the target rate is about four frames per second; actual frame rate still depends on I2C, row settling, FDC conversion, logging, and rescue activity.

### Build and target configuration / 编译与目标配置

| 配置项 / Option | 类型 / Type | 默认值 / Default | 作用 / Purpose | 影响的函数 / Affected functions | 调整建议 / Tuning notes |
|---|---:|---:|---|---|---|
| `CONFIG_IDF_TARGET` | string | sdkconfig defaults: `esp32s3` | Selects ESP-IDF target. | ESP-IDF build system | Keep `esp32s3` for this board. |
| `CONFIG_ESPTOOLPY_FLASHSIZE_16MB` | bool | sdkconfig defaults: y | Sets flash size profile. | ESP-IDF flashing/partition handling | Match physical module flash. |
| `CONFIG_ESP_MAIN_TASK_STACK_SIZE` | int | sdkconfig defaults: `16384` | Main task stack size. | `app_main()`, FreeRTOS startup | Lower only after stack high-water logs are reviewed. |
| `CONFIG_FREERTOS_CHECK_STACKOVERFLOW_CANARY` | bool | sdkconfig defaults: y | Enables stack overflow checking. | FreeRTOS runtime | Keep enabled during bring-up. |
| `CONFIG_FREERTOS_WATCHPOINT_END_OF_STACK` | bool | sdkconfig defaults: y | Uses watchpoint at stack end. | FreeRTOS runtime | Keep enabled for debug builds. |
| `CONFIG_HEAP_POISONING_LIGHT` | bool | sdkconfig defaults: y | Adds heap corruption checks. | ESP-IDF heap | Useful for diagnostics; may add overhead. |
| `CONFIG_COMPILER_STACK_CHECK_MODE_STRONG` | choice | sdkconfig defaults: strong | Compiler stack checking. | all compiled C code | Keep strong unless measuring release performance. |
| `CONFIG_SENSORARRAY_ENABLE_WIRED` | bool | Kconfig: y | Enables wired transport option. | transport build paths | Current default output still uses printf text. |
| `CONFIG_SENSORARRAY_ENABLE_BLE` | bool | Kconfig: y | Enables BLE transport option. | BLE transport build paths | Disable when BLE task/memory budget is not required. |
| `CONFIG_SENSORARRAY_SCAN_TASK_CORE`, `CONFIG_SENSORARRAY_SCAN_TASK_STACK`, `CONFIG_SENSORARRAY_SCAN_TASK_PRIO` | int | `1`, `8192`, `12` | Scheduling defaults for scan work. | worker/task config defaults | Keep scan work away from comm/BLE unless core affinity needs debugging. |
| `CONFIG_SENSORARRAY_COMM_TASK_CORE`, `CONFIG_SENSORARRAY_COMM_TASK_STACK`, `CONFIG_SENSORARRAY_COMM_TASK_PRIO` | int | `0`, `4096`, `8` | Scheduling defaults for communication work. | transport/task config defaults | Increase stack if future transport code adds deeper call chains. |
| `CONFIG_SENSORARRAY_BLE_TASK_CORE` | int | `0` | BLE task core when BLE is enabled. | BLE transport task config | Current BLE transport is placeholder only. |

### FDC matrix configuration / FDC 矩阵配置

| 配置项 / Option | 类型 / Type | 默认值 / Default | 作用 / Purpose | 影响的函数 / Affected functions | 调整建议 / Tuning notes |
|---|---:|---:|---|---|---|
| `CONFIG_SENSORARRAY_FDC_PRIMARY_I2C_ADDR` | hex | Kconfig: `0x2B` | Primary FDC for D1-D4. | `sensorarrayInitRuntime()`, `sensorarrayInitFdcDevice()` | Change only if hardware address strap changes. |
| `CONFIG_SENSORARRAY_FDC_SECONDARY_I2C_ADDR` | hex | Kconfig: `0x2A` | Secondary FDC for D5-D8. | `sensorarrayInitRuntime()`, `sensorarrayInitFdcDevice()` | Change only if hardware address strap changes. |
| `CONFIG_SENSORARRAY_FDC_STARTUP_PROBE` | bool | Kconfig: y | Enables startup probe path where used by bring-up. | `sensorarrayBringupProbeFdcBus()` | Keep enabled while validating hardware. |
| `CONFIG_FDC2214CAP_CHANNELS` | int | component Kconfig: `4` | Requested FDC channel count. | `sensorarrayInitRuntime()`, `sensorarrayBringupNormalizeFdcChannels()` | Current matrix expects 4 channels per FDC. Fewer channels reduce matrix completeness and are normalised upward by app init. |
| `CONFIG_SENSORARRAY_FDC_TANK_INDUCTOR_NH` | int | Kconfig and defaults: `18000` | LC tank inductor for `freqHz -> capTotalPf`. | `sensorarrayMeasureComputeFdcFrameCapTotalPf()`, `sensorarrayMeasureFdcComputeCapacitancePf()` | Must match hardware. Wrong L shifts pF values but does not change `raw28`. Formula: `C = 1 / ((2 * pi * f)^2 * L)`. |
| `CONFIG_SENSORARRAY_FDC_TANK_INDUCTOR_UH` | int | Kconfig: `0` | Legacy/optional sweep conversion input. | `sensorarrayFdcSweep.c` | Prefer `CONFIG_SENSORARRAY_FDC_TANK_INDUCTOR_NH` for MATRIXFDC pF output. |
| `CONFIG_SENSORARRAY_FDC_REF_CLOCK_USE_EXTERNAL`, `CONFIG_SENSORARRAY_FDC_REF_CLOCK_USE_EXTERNAL_VALUE` | bool/int | Kconfig: external y, value `1` | Selects external FDC reference clock path. | `sensorarrayConfig.h`, FDC frequency conversion helpers | Keep aligned with actual CLKIN. |
| `CONFIG_SENSORARRAY_FDC_EXTERNAL_CLOCK_HZ` | int | Kconfig: `40000000` | External CLKIN frequency in Hz. | `sensorarrayMeasureFdcEffectiveFclkHz()` | Wrong clock value skews frequency and capacitance conversion. |
| `CONFIG_SENSORARRAY_MATRIX_ROWS`, `CONFIG_SENSORARRAY_MATRIX_COLS` | int | Kconfig: `8`, `8` | Logical matrix size. | `sensorarrayScanPlanBuildDefaultFdcMatrix()`, frame arrays | Current frame type is fixed for 64 cells; treat changes as architecture work. |
| `CONFIG_SENSORARRAY_FRAME_PERIOD_MS` | int | Kconfig: `CONFIG_SENSORARRAY_FDC_MATRIX_PERIOD_MS`; header fallback `250` if absent | Generic frame-period fallback. | `sensorarrayConfig.h` | For current FDC path, tune `CONFIG_SENSORARRAY_FDC_MATRIX_PERIOD_MS` directly. |
| `CONFIG_SENSORARRAY_OVERSAMPLE` | int | Kconfig: `1` | Generic oversample default. | scan/matrix defaults | Current FDC production path uses row epoch logic rather than a generic oversample loop. |

### FDC timing and frame-rate configuration / FDC 时序与帧率配置

| 配置项 / Option | 类型 / Type | 默认值 / Default | 作用 / Purpose | 影响的函数 / Affected functions | 调整建议 / Tuning notes |
|---|---:|---:|---|---|---|
| `CONFIG_SENSORARRAY_FDC_MATRIX_PERIOD_MS` | int | Kconfig and defaults: `50` | Target period for one 64-cell text frame. | `sensorarrayFdcFramePeriodMs()`, `sensorarrayDelayFramePeriodSince()` | Lower values raise target fps but increase overrun risk. `250 ms` means about `4 fps`. |
| `CONFIG_SENSORARRAY_FDC_MATRIX_TARGET_FPS` | int | Kconfig/defaults: `20` | Timing budget target used in profiling. | `SENSORARRAY_FDC_TARGET_FRAME_US`, timing logs | Keep consistent with frame period when comparing `SCAN_TIMING_*`. |
| `CONFIG_SENSORARRAY_FDC_MATRIX_SETTLE_US` | int | Kconfig/defaults: `1000` | Settle delay for some path/ADS helper paths. | `sensorarrayMeasurePrepareFdcMatrixPath()`, ADS helper paths, sweep paths | Lower only after verifying route stability and invalid-frame rate. |
| `CONFIG_SENSORARRAY_FDC_ROW_SWITCH_SETTLE_US` | int | Kconfig/defaults: `50` | Row settle delay while FDC devices are sleeping. | `sensorarrayMeasureSelectFdcRowWhileSleeping()` | Too low can increase row-to-row mixing, amplitude warnings, or invalid rows. |
| `CONFIG_SENSORARRAY_FDC_MATRIX_DISCARD_SAMPLES` | int | Kconfig: `0`, sdkconfig.defaults: `1` | Discard count used by legacy/direct paths. | ADS/FDC helper code | Keep low in row epoch mode; excessive discard hurts fps. |
| `CONFIG_SENSORARRAY_FDC_DISCARD_FRAMES_AFTER_ROW_SWITCH` | int | Kconfig/defaults: `0` | Runtime autoscan frame discard count after row switch. | `sensorarrayMeasureFdcDiscardFrames()`, discard helper | Production sleep-epoch path defaults to zero discard frames. |
| `CONFIG_SENSORARRAY_FDC_ROW_EPOCH_RESTART_ENABLE` | bool | Kconfig/defaults: y | Enables per-row conversion epoch restart. | row epoch helpers | Keep enabled for current row isolation strategy. |
| `CONFIG_SENSORARRAY_FDC_ROW_EPOCH_RESTART_METHOD_SLEEP` | bool | Kconfig/defaults: y | Uses `CONFIG.SLEEP_MODE_EN` for row restart. | `sensorarrayMeasureFdcSetSleepMode()` | Current stable path uses sleep entry/exit rather than SD pin reset. |
| `CONFIG_SENSORARRAY_FDC_DIFF_CACHE_APPLY` | bool | Kconfig/defaults: y | Applies only changed cached FDC row registers while devices are sleeping. | `sensorarrayMeasureApplyFdcCachedRowConfig()` | Keep enabled to reduce per-row I2C writes. |
| `CONFIG_SENSORARRAY_FDC_FORMAL_FAST_PROFILE_ENABLE` | bool | Kconfig/defaults: n | Enables experimental runtime RCOUNT reduction when cached profile timing is too slow. | `sensorarrayMeasureApplyFdcCachedRowConfig()` | Keep disabled for normal matrix reads; enable only for controlled fast-profile debugging. |
| `CONFIG_SENSORARRAY_FDC_FORMAL_FAST_TARGET_ROUND_US` | int | Kconfig/defaults: `4000` | Target round time for the optional formal fast profile. | PFU/P5 diagnostics, optional fast-profile tuning | With fast profile disabled, this is diagnostic only and does not mutate cached RCOUNT. |
| `CONFIG_SENSORARRAY_FDC_PROFILE_TOO_SLOW_RESCUE_ENABLE` | bool | Kconfig/defaults: n | Allows `profile_too_slow` to request rescue/sweep. | `sensorarrayMeasureRequestFdcCellRescue()`, row watchdog/cache apply gates | Keep disabled unless running an explicit debug experiment; slow estimated timing is not an electrical failure. |
| `CONFIG_SENSORARRAY_FDC_ALLOW_SAFE_DEFAULT_FORMAL_READ` | bool | Kconfig: n | Allows uncalibrated safe-default row configs during formal reads. | `sensorarrayMeasureApplyFdcCachedRowConfig()` | Keep disabled for normal operation. A missing cache now marks cells invalid, logs `FDC_CACHE_MISS`, and requests fast rescue. |
| `CONFIG_SENSORARRAY_LOG_CACHE_APPLY_VERBOSE` | bool | Kconfig: n | Prints verbose cache apply details. | `sensorarrayFdcCacheApply.inc` | Enable only when debugging cache fingerprints; it adds log load. |
| `CONFIG_SENSORARRAY_FDC_CACHE_APPLY_VERBOSE_LOG` | bool | defaults: n | Legacy compatibility alias for cache apply logging. | compatibility fallback | Prefer `CONFIG_SENSORARRAY_LOG_CACHE_APPLY_VERBOSE` in new builds. |
| `CONFIG_SENSORARRAY_FDC_SETTLECOUNT_DEFAULT` | hex | Kconfig: `0x0080` | Conservative default FDC SETTLECOUNT. | cache/sweep channel config | Lower only after valid conversions are stable. |
| `CONFIG_SENSORARRAY_FDC_ROW_WAIT_SAFETY_US` | int | Kconfig: `2000` | Safety margin after one autoscan cycle. | FDC wait/sweep helpers | Keeps actual INTB wait at or above the estimated autoscan round plus margin unless clamped by the row-device hard deadline. |
| `CONFIG_SENSORARRAY_FDC_READY_POLICY_INTB_STRICT_LEVEL` | choice bool | Kconfig/defaults: y | Default formal ready policy: wait for INTB low, then read STATUS once to ack/verify readiness. | `sensorarrayFdcWaitDeviceReady()` | Production path; no STATUS fallback before INTB and no legacy fallback recovery. |
| `CONFIG_SENSORARRAY_FDC_READY_POLICY_INTB_WITH_POLL_FALLBACK` | choice bool | Kconfig/defaults: n | Legacy diagnostic policy: wait INTB, verify STATUS, then allow bounded STATUS fallback. | `sensorarrayFdcWaitDeviceReady()` | Enable only to compare old fallback behaviour; fallback recovery appears as `src=FB`. |
| `CONFIG_SENSORARRAY_FDC_READY_POLICY_INTB_THEN_STATUS` | choice bool | Kconfig/defaults: n | Wait INTB then verify STATUS once, without legacy fallback. | `sensorarrayFdcWaitDeviceReady()` | Diagnostic mode for missed-INTB validation. |
| `CONFIG_SENSORARRAY_FDC_READY_POLICY_POLL_ONLY` | choice bool | Kconfig/defaults: n | Poll STATUS/unread directly instead of using INTB. | `sensorarrayFdcWaitDeviceReady()` | Debug/compatibility mode; not the default frame-rate path. |
| `CONFIG_SENSORARRAY_FDC_AFTER_INTB_RECHECK_ENABLE` | bool | Kconfig/defaults: y | Allows short STATUS rechecks only after INTB when unread is full but `DRDY=0`. | `sensorarrayFdcWaitDeviceReady()` | Normal recovery source is `src=AR`; it never replaces waiting for INTB. |
| `CONFIG_SENSORARRAY_FDC_AFTER_INTB_RECHECK_MAX` | int | Kconfig/defaults: `3` | Maximum after-INTB STATUS rechecks. | `sensorarrayFdcWaitDeviceReady()` | Keep bounded so one row-device cannot dominate frame time. |
| `CONFIG_SENSORARRAY_FDC_AFTER_INTB_RECHECK_INTERVAL_US` | int | Kconfig/defaults: `250` | Delay between after-INTB STATUS rechecks. | `sensorarrayFdcWaitDeviceReady()` | Lower increases I2C pressure; higher increases ready latency. |
| `CONFIG_SENSORARRAY_FDC_AFTER_INTB_RECHECK_DEADLINE_US` | int | Kconfig/defaults: `1000` | Total after-INTB recheck deadline. | `sensorarrayFdcWaitDeviceReady()` | Hard cap for the `src=AR` path. |
| `CONFIG_SENSORARRAY_FDC_INTB_FALLBACK_POLLING` | bool | Kconfig/defaults: n | Compatibility switch for legacy INTB-unavailable fallback handling. | `sensorarrayFdcWaitDeviceReady()` | Keep disabled for strict formal runs. |
| `CONFIG_SENSORARRAY_FDC_READY_GUARD_US` | int | Kconfig/defaults: `3000` | Guard time used by poll-only and legacy fallback diagnostics. | `sensorarrayFdcWaitDeviceReady()` | Does not extend strict INTB hard timeout. |
| `CONFIG_SENSORARRAY_FDC_POLL_FALLBACK_MAX_POLLS` | int | Kconfig/defaults: `3` | Maximum STATUS polls in fallback/poll-only mode. | `sensorarrayFdcWaitDeviceReady()` | Keeps fallback bounded so a bad row cannot dominate frame time. |
| `CONFIG_SENSORARRAY_FDC_READY_MAX_POLLS_AFTER_UNREAD_BEFORE_DRDY` | int | Kconfig/defaults: `3` | Maximum short guard polls after unread bits are full but `DRDY=0`. | `sensorarrayFdcWaitDeviceReady()` | Lets the intermediate `unread=full,DRDY=0` state recover without entering rescue. |
| `CONFIG_SENSORARRAY_FDC_READY_POLL_INTERVAL_US` | int | Kconfig/defaults: `1000` | Delay between guarded STATUS ready polls. | `sensorarrayFdcWaitDeviceReady()` | Lower increases I2C load; higher increases missed-frame latency. |
| `CONFIG_SENSORARRAY_FDC_REQUIRE_DRDY_FOR_VALID` | bool | Kconfig/defaults: y | Requires `DRDY=1` before data registers are accepted. | `sensorarrayFdcWaitDeviceReady()`, `sensorarrayMeasureReadFdcAutoscan4chMasked()` | Keep enabled. `unread=full && DRDY=0` is `WAIT_DRDY`, not a hard invalid sample. |
| `CONFIG_SENSORARRAY_FDC_SUPPRESS_STATUS_READ_BEFORE_INTB` | bool | Kconfig/defaults: y | Records suppression of pre-INTB STATUS reads in INTB modes. | `sensorarrayFdcWaitDeviceReady()` | Keep enabled to avoid long STATUS polling before INTB. |
| `CONFIG_SENSORARRAY_FDC_DISABLE_INTB_FOR_DEBUG` | bool | Kconfig/defaults: n | Forces INTB output disabled for debug builds. | `sensorarrayMeasureFdcConfigBaseWithoutSleep()`, bring-up/sweep config builders | Enable only when intentionally testing poll-only behaviour. |

### FDC sweep and rescue configuration / FDC 扫描与救援配置

| 配置项 / Option | 类型 / Type | 默认值 / Default | 作用 / Purpose | 影响的函数 / Affected functions | 调整建议 / Tuning notes |
|---|---:|---:|---|---|---|
| `CONFIG_SENSORARRAY_FDC_BOOT_SWEEP_REQUIRED` | bool | Kconfig: y | Requires boot sweep success before normal output. | `sensorarrayRunBootCalibration()`, `sensorarrayFdcSweepRunBoot()` | Disable only for degraded bring-up; normal operation should require a valid boot cache. |
| `CONFIG_SENSORARRAY_FDC_BOOT_MIN_VALID_CELLS` | int | Kconfig: `48` | Minimum accepted cells for an OK boot sweep. | `sensorarrayFdcSweepRunBoot()`, `sensorarrayRunBootCalibration()` | Raise for stricter production gating; lower only when intentionally accepting partial hardware. |
| `CONFIG_SENSORARRAY_FDC_BOOT_ALLOW_DEGRADED` | bool | Kconfig: y | Allows a non-zero but incomplete boot sweep to enter degraded mode instead of hard fail. | `sensorarrayFdcSweepRunBoot()`, `sensorarrayRunBootCalibration()` | With boot sweep required, degraded quality still enters diagnostic mode unless app policy accepts it. |
| `CONFIG_SENSORARRAY_FDC_BOOT_REQUIRED_ROWS_MASK` | hex | Kconfig: `0xFF` | Required S-row mask for boot quality. | `sensorarrayFdcSweepRunBoot()` | Keep `0xFF` for the full 8-row matrix. |
| `CONFIG_SENSORARRAY_REQUIRE_DUAL_FDC_FOR_BOOT` | bool | Kconfig/defaults: y | Requires both FDC devices at boot. | `sensorarrayRunBootCalibration()` | If disabled, secondary absence allows primary-only D1-D4 output while D5-D8 are invalid. |
| `CONFIG_SENSORARRAY_FDC_SWEEP_STEP_TIMEOUT_MS` | int | Kconfig: `180` | Timeout per sweep candidate. | `sensorarrayFdcSweep.c` | Increase for slow/noisy oscillators; too high can block boot/rescue. |
| `CONFIG_SENSORARRAY_FDC_SWEEP_TOTAL_TIMEOUT_MS` | int | Kconfig: `2500` | Bounded timeout per channel sweep. | `sensorarrayFdcSweep.c` | Higher values improve chance of finding a valid candidate but delay recovery. |
| `CONFIG_SENSORARRAY_FDC_SWEEP_SETTLE_MS` | int | Kconfig: `20` | Settle after applying sweep candidate. | `sensorarrayFdcSweep.c` | Lower for speed only after raw and warning stability are confirmed. |
| `CONFIG_SENSORARRAY_FDC_SWEEP_SAMPLE_TIMEOUT_MS` | int | Kconfig: `80` | Wait timeout for sweep sample. | `sensorarrayMeasureWaitFdcAutoscanFrameReady()`, sweep reads | Increase if sweep samples time out before unread bits appear. |
| `CONFIG_SENSORARRAY_FDC_SWEEP_SAMPLE_POLL_MS` | int | Kconfig: `2` | Poll interval for sweep sample. | `sensorarrayFdcSweep.c` | Lower increases bus/log pressure; higher slows sweep. |
| `CONFIG_SENSORARRAY_FDC_DIRECT_FAIL_THRESHOLD` | int | Kconfig: `3` | Direct-read failures before fast sweep consideration. | `sensorarrayFdcSweep.c` | Raise to avoid unnecessary sweeps; lower to recover faster. |
| `CONFIG_SENSORARRAY_FDC_CELL_FAST_SWEEP_FAIL_THRESHOLD` | int | Kconfig/defaults: `2` | Cell failures before bounded fast sweep escalation. | `sensorarrayFdcSweepRunFullRescueCell()` | Tune with hard-error rate. |
| `CONFIG_SENSORARRAY_FDC_FAST_FAIL_THRESHOLD` | int | Kconfig: `2` | Fast sweep failures before full rescue. | FDC sweep/rescue helpers | Lower makes full rescue more aggressive. |
| `CONFIG_SENSORARRAY_FDC_FAST_SWEEP_COOLDOWN_MS` | int | Kconfig/defaults: `30000` | Cooldown between fast sweeps for a cell. | `sensorarrayFdcSweep.c` | Increase to reduce runtime disruption; lower for faster recovery. |
| `CONFIG_SENSORARRAY_FDC_FAST_SWEEP_MIN_COOLDOWN_MS` | int | Kconfig: `30000`, sdkconfig.defaults: `1000` | Minimum automatic runtime fast-sweep cooldown. | `sensorarrayFdcSweep.c` | The sdkconfig default intentionally allows quicker runtime recovery than the Kconfig default. |
| `CONFIG_SENSORARRAY_FDC_RUNTIME_FAST_SWEEP_ENABLE` | bool | Kconfig/defaults: y | Allows automatic runtime fast sweeps. | `sensorarrayMeasureRequestFdcCellRescue()` | Disable to study raw failure behaviour without automatic fast rescue. |
| `CONFIG_SENSORARRAY_FDC_FULL_SWEEP_REQUEST_COOLDOWN_MS` | int | Kconfig/defaults: `5000` | Cooldown for queued full sweep requests. | `sensorarrayRunQueuedFullSweep()`, `sensorarrayFdcSweepReportAllInvalidFrame()` | Prevents repeated full sweeps from blocking main loop. |
| `CONFIG_SENSORARRAY_FDC_MAX_CONSECUTIVE_FULL_SWEEP_FAILS` | int | Kconfig/defaults: `3` | Full rescue failures before diagnostic mode. | `sensorarrayRunQueuedFullSweep()` | Raise only if hardware often recovers after repeated full sweeps. |
| `CONFIG_SENSORARRAY_FDC_FULL_RESCUE_COOLDOWN_MS` | int | Kconfig/defaults: `3000` | Full-matrix rescue cooldown in sweep layer. | `sensorarrayFdcSweep.c` | Co-ordinate with queued full-sweep cooldown. |
| `CONFIG_SENSORARRAY_FDC_FULL_SWEEP_RESCUE_COOLDOWN_MS` | int | Kconfig: `10000` | Cell/full-sweep rescue cooldown legacy path. | `sensorarrayFdcSweep.c` | Keep higher than fast local rescue to avoid heavy scans. |
| `CONFIG_SENSORARRAY_FDC_NO_OSC_RESCUE_COOLDOWN_MS` | int | Kconfig: `500` | No-oscillation rescue cooldown. | FDC sweep/rescue helpers | Lower only when false no-oscillation is rare. |
| `CONFIG_SENSORARRAY_FDC_ALL_INVALID_RESCUE_THRESHOLD` | int | Kconfig/defaults: `3` | All-invalid threshold for rescue policy. | `sensorarrayFdcRescueTick()`, `sensorarrayFdcSweep.c` | Lower triggers full recovery sooner but can mask timing issues. |
| `CONFIG_SENSORARRAY_FDC_ALL_INVALID_FULL_SWEEP_THRESHOLD` | int | Kconfig/defaults: `3` | All-invalid threshold before full sweep queue. | `sensorarrayFdcSweepReportAllInvalidFrame()` | Keep aligned with rescue threshold unless debugging. |
| `CONFIG_SENSORARRAY_FDC_ALL_INVALID_RESTART_THRESHOLD` | int | Kconfig: `3` | Legacy/diagnostic threshold for repeated invalid frames. | FDC rescue paths | Treat as diagnostic guard. |
| `CONFIG_SENSORARRAY_FDC_ROW_RESCUE_FAIL_COUNT` | int | Kconfig/defaults: `4` | Per-row fail count hint for rescue diagnostics. | FDC sweep/rescue diagnostics | Adjust for row-level noisy hardware. |
| `CONFIG_SENSORARRAY_FDC_RESCUE_TASK_STACK` | int | Kconfig/defaults: `12288` | Reserved rescue task stack bytes. | rescue task paths if used | Keep large enough for register dump/sweep diagnostics. |

### FDC I2C and parallel-worker configuration / FDC I2C 与并行 worker 配置

| 配置项 / Option | 类型 / Type | 默认值 / Default | 作用 / Purpose | 影响的函数 / Affected functions | 调整建议 / Tuning notes |
|---|---:|---:|---|---|---|
| `CONFIG_BOARD_I2C_PORT`, `CONFIG_BOARD_I2C_SDA_GPIO`, `CONFIG_BOARD_I2C_SCL_GPIO` | int | `0`, `9`, `10` | Primary I2C bus pins and port. | `boardSupportInit()` | Match board wiring. |
| `CONFIG_BOARD_I2C_FREQ_HZ`, `CONFIG_BOARD_I2C0_FREQ_HZ` | int | defaults: `337500` | Primary bus frequency. | `boardSupportInit()`, timing estimates | Higher is not always more stable; watch NACK, timeout, and SCL stretch. |
| `CONFIG_BOARD_I2C1_ENABLE`, `CONFIG_BOARD_I2C1_PORT`, `CONFIG_BOARD_I2C1_SDA_GPIO`, `CONFIG_BOARD_I2C1_SCL_GPIO`, `CONFIG_BOARD_I2C1_FREQ_HZ` | bool/int | y, `1`, `11`, `12`, `337500` | Optional second I2C bus for secondary FDC. | `boardSupportIsI2c1Enabled()`, `sensorarrayLogFdcParallelCfg()` | True parallel worker mode needs a valid second bus on a different port. |
| `CONFIG_SENSORARRAY_FDC_PARALLEL_DUAL_BUS_READ` | bool | Kconfig/defaults: y | Enables worker-based primary/secondary row reads when dual buses are available. | `sensorarrayMeasureReadFdcMatrixFrame()`, row epoch workers | Same bus or worker init failure falls back to serial. |
| `CONFIG_SENSORARRAY_FDC_PARALLEL_DUAL_BUS_READ_SAFE` | bool | Kconfig/defaults: y | Requires the guarded worker handoff before parallel row reads are used. | `sensorarrayMeasureReadFdcMatrixFrame()`, row epoch workers | Keep enabled. Worker timeout/fallback now waits for the worker to go idle before serial fallback. |
| `CONFIG_SENSORARRAY_FDC_FORCE_SINGLE_THREAD_READ` | bool | Kconfig: n | Forces serial row reads even when dual buses and workers are available. | `sensorarrayMeasureReadFdcMatrixFrame()` | Enable only to isolate worker scheduling or shared-handle issues. |
| `CONFIG_SENSORARRAY_FDC_WORKER_TASK_STACK` | int | Kconfig: `6144` | Static worker stack bytes. | `sensorarrayMeasureEnsureFdcWorkers()` | Increase if worker stack logs show low margin. |
| `CONFIG_SENSORARRAY_FDC_WORKER_TASK_PRIO` | int | Kconfig: `CONFIG_SENSORARRAY_SCAN_TASK_PRIO` | Worker task priority. | worker task creation | Keep near scan priority to reduce skew. |
| `CONFIG_SENSORARRAY_FDC_WORKER_TASK_CORE` | int | Kconfig: `-1` | Worker core affinity; `-1` means unpinned. | `sensorarrayMeasureEnsureFdcWorkers()` | `-1` uses unpinned static task creation; pinned mode needs valid CPU ID. |
| `CONFIG_SENSORARRAY_FDC_WORKER_SYNC_TIMEOUT_MS` | int | Kconfig: `25` | Queue/sleep/done sync timeout. | `sensorarrayMeasureReadFdcMatrixRowParallelEpoch()` | Increase if workers time out despite valid I2C. |
| `CONFIG_SENSORARRAY_I2C_RECOVERY_ENABLED` | bool | boardSupport/defaults: y | Guarded I2C bus recovery after timeout and stuck bus pins. | `boardSupportI2cShouldRecover()`, `boardSupportRecoverI2cBus()` | Recovery is not attempted for plain NACKs. |
| `CONFIG_SENSORARRAY_I2C_RECOVERY_COOLDOWN_MS` | int | defaults: `1000` | Cooldown between recovery attempts. | `boardSupportRecoverI2cBus()` | Lower only for lab recovery tests. |
| `CONFIG_SENSORARRAY_I2C_RECOVERY_MAX_FAILS` | int | defaults: `3` | Recovery failures before marking bus offline. | `boardSupportI2cMarkOffline()` | Raise if physical bus recovery is slow but possible. |
| `CONFIG_SENSORARRAY_I2C_RECOVERY_TOGGLE_CLOCKS` | int | defaults: `9` | SCL pulses during recovery. | `boardSupportI2cPulseRecoveryClock()` | Match common I2C recovery practice unless board demands otherwise. |
| `CONFIG_SENSORARRAY_I2C_RECOVER_ON_TIMEOUT` | bool | Kconfig: n, header fallback `0` | Legacy project-level timeout recovery symbol. | `sensorarrayConfig.h` compatibility | Current board recovery path uses `CONFIG_SENSORARRAY_I2C_RECOVERY_ENABLED` and related boardSupport options. |

### Output and logging configuration / 输出与日志配置

| 配置项 / Option | 类型 / Type | 默认值 / Default | 作用 / Purpose | 影响的函数 / Affected functions | 调整建议 / Tuning notes |
|---|---:|---:|---|---|---|
| `CONFIG_SENSORARRAY_FDC_EMIT_CAP_TOTAL_PF` | bool | Kconfig/defaults: y | Legacy compatibility symbol. | Kconfig compatibility | Use text output unit choice for new builds. |
| `CONFIG_SENSORARRAY_FDC_TEXT_OUTPUT_CAP_TOTAL_PF` | choice bool | default selected | Emits `MATRIXFDC_CAP`. | `sensorarrayFrameOutputPrintText()` | Default host path. |
| `CONFIG_SENSORARRAY_FDC_TEXT_OUTPUT_FREQ_HZ` | choice bool | default n | Emits only `MATRIXFDC_FREQ`. | `sensorarrayFrameOutputPrintText()` | Use for frequency diagnostics; pF output disabled in this mode. |
| `CONFIG_SENSORARRAY_FDC_TEXT_OUTPUT_BOTH_SEPARATE` | choice bool | default n | Emits separate cap and freq lines. | `sensorarrayFrameOutputPrintText()` | Doubles text output volume. |
| `CONFIG_SENSORARRAY_FDC_TEXT_OUTPUT_BOTH_INLINE_DEBUG` | choice bool | default n | Emits inline debug line with freq and cap. | `sensorarrayFrameOutputPrintText()` | Debug only; high serial load. |
| `CONFIG_SENSORARRAY_FDC_RAW_DEBUG_LOG` | bool | defaults: n | Emits `DEBUGFDC_RAW`. | `sensorarrayFrameOutputPrintText()` | Enable only for raw-data debugging. |
| `CONFIG_SENSORARRAY_LOG_FRAME_SUMMARY` | bool | Kconfig: y | Emits compact `S` frame summary before text matrix output. | `sensorarrayFrameOutputPrintText()` | Keep enabled for host-side validity checks without parsing the full 64-cell array. |
| `CONFIG_SENSORARRAY_LOG_ROW_SUMMARY` | bool | Kconfig: n | Enables compact per-row summaries when row-level logs are needed. | `sensorarrayMeasureFillFdcMatrixRow()`, row cache-miss path | Leave disabled for normal compact output. |
| `CONFIG_SENSORARRAY_LOG_I2C_STATS_EVERY_N_FRAMES` | int | Kconfig/defaults: `5` | Compact I2C aggregate period. | `sensorarrayMeasurePrintFdcTimingAggregate()` | Set 0 to suppress periodic I2C aggregate output. |
| `CONFIG_SENSORARRAY_LOG_TIMING_STATS_EVERY_N_FRAMES` | int | Kconfig/defaults: `5` | Compact timing aggregate period. | `sensorarrayMeasurePrintFdcTimingAggregate()` | Set 0 to suppress periodic timing aggregate output. |
| `CONFIG_SENSORARRAY_FDC_TIMING_SUMMARY_EVERY_N_FRAMES` | int | Kconfig/defaults: `5` | Runtime timing summary interval default. | `sensorarrayMeasureFdcProfileSetSummaryEvery()` | Lower increases log density. |
| `CONFIG_SENSORARRAY_FDC_TIMING_SUMMARY_PERIOD_FRAMES` | int | Kconfig/defaults: `5` | Compact aggregate period for `T5/R5/Q5/I5`. | `sensorarrayMeasureUpdateFdcTimingAggregate()` | Set 0 to suppress aggregate summary. |
| `CONFIG_SENSORARRAY_FDC_TIMING_SUMMARY_AGGREGATE` | bool | Kconfig/defaults: y | Enables aggregate timing summaries. | timing aggregate functions | Keep enabled during performance work. |
| `CONFIG_SENSORARRAY_FDC_TIMING_OVERRUN_IMMEDIATE_LOG` | bool | Kconfig/defaults: y | Prints bottleneck on frame overrun. | `sensorarrayMeasurePrintFdcBottleneck()` | Useful when testing lower frame periods. |
| `CONFIG_SENSORARRAY_FDC_TIMING_VERBOSE_PER_FRAME` | bool | defaults: n | Enables per-frame timing summary when profile summary is on. | `sensorarrayMeasurePrintFdcTimingSummary()` | High log volume; affects frame rate. |
| `CONFIG_SENSORARRAY_FDC_PROFILE_ROW_DEFAULT`, `CONFIG_SENSORARRAY_FDC_PROFILE_DEVICE_DEFAULT` | bool | defaults: n | Default row/device timing logs. | `sensorarrayMeasurePrintFdcRowTiming()`, `sensorarrayMeasurePrintFdcDeviceTiming()` | Enable only when detailed timing is needed. |
| `CONFIG_SENSORARRAY_FDC_LOG_FORMAT_COMPACT` | choice bool | Kconfig/defaults: y | Enables compact hot-path tokens such as `RB`, `RP`, `RR`, `SR`, `RWD`, `D4`, `T5`, `R5`, `Q5`, `I5`, `P5`, and `OT`. | row epoch, read4, frame timing and output logs | Recommended while profiling 20 fps output because it reduces serial load. |
| `CONFIG_SENSORARRAY_FDC_LOG_FORMAT_VERBOSE` | choice bool | Kconfig/defaults: n | Keeps longer diagnostic log names where available. | row epoch and frame timing logs | Use only when human-readable hot-path logs matter more than frame-rate impact. |
| `CONFIG_SENSORARRAY_FDC_LOG_READY_EVERY_ROW` | bool | Kconfig/defaults: n | Emits compact ready diagnostics for every row/device. | `sensorarrayFdcWaitDeviceReady()` | Enable for INTB/STATUS bring-up; leave off for normal output. |
| `CONFIG_SENSORARRAY_FDC_LOG_ROW_PARALLEL_TIMING` | bool | Kconfig/defaults: n | Emits sampled per-row primary/secondary timing skew when row log level allows it. | row epoch parallel path | Enable only when validating worker skew and INTB timing. |
| `CONFIG_SENSORARRAY_FDC_LOG_FULL_CAP_FRAME` | bool | Kconfig/defaults: y | Emits full `MATRIXFDC_CAP` frame lines. | frame output | Disable only if host tooling uses compact frame logs instead. |
| `CONFIG_SENSORARRAY_FDC_LOG_COMPACT_CAP_FRAME` | bool | Kconfig/defaults: n | Reserved compact frame-output switch. | frame output | Enable only for host-side parsers that understand compact matrix output. |
| `CONFIG_SENSORARRAY_FDC_I2C_TRACE_RING_SIZE` | int | defaults: `128` | Ring size for FDC I2C trace records. | `Fdc2214CapI2cTrace*()` | Larger rings use more RAM; trace dumps occur on errors/overruns when enabled. |
| `CONFIG_SENSORARRAY_LOG_LOW_LEVEL_I2C_XFER` | bool | Kconfig: n | Prints board-level I2C transaction begin/end lines. | `boardSupportI2cWriteRead()`, `boardSupportI2cWrite()`, `boardSupportI2cRead()`, `boardSupportI2cProbeAddress()` | Keep disabled during normal matrix reads; I2C errors and recovery still log when disabled. |
| `CONFIG_SENSORARRAY_LOG_FDC_REGISTER_TRACE` | bool | Kconfig: n | Project-level switch reserved for FDC register trace diagnostics. | FDC diagnostics | Keep disabled unless tracing register traffic. |
| `CONFIG_SENSORARRAY_LOG_SWEEP_CANDIDATE_VERBOSE` | bool | Kconfig: n | Prints every sweep candidate line. | `sensorarrayFdcSweep.c` | Use only for sweep tuning; rejected best candidates are still logged compactly when this is disabled. |
| `CONFIG_SENSORARRAY_LOG_WORKER_VERBOSE` | bool | Kconfig: n | Enables verbose worker logs. | row epoch workers | Keep disabled unless debugging worker scheduling. |
| `CONFIG_FDC2214CAP_LOW_LEVEL_I2C_TRACE`, `CONFIG_FDC2214CAP_RAW_I2C_TRACE` | bool | component Kconfig: n | Driver-level I2C/register printf trace. | `components/fdc2214Cap/fdc2214Cap.c` | Avoid in normal matrix reads because printf can dominate timing. |

### ADS configuration / ADS 配置

| 配置项 / Option | 类型 / Type | 默认值 / Default | 作用 / Purpose | 影响的函数 / Affected functions | 调整建议 / Tuning notes |
|---|---:|---:|---|---|---|
| `CONFIG_SENSORARRAY_ADS1262`, `CONFIG_SENSORARRAY_ADS1263` | choice | default `ADS1262` | Selects ADS126x variant. | ADS bring-up and conditional ADC2 handling | Match installed chip; ADC2 stop is conditional for ADS1263. |
| `CONFIG_ADS126X_LOG_LEVEL` | int | `3` | ADS component log level. | `components/ads126xAdc/ads126xAdc.c` | Raise only for driver diagnostics. |
| `CONFIG_ADS126X_SPI_CLOCK_HZ` | int | `2000000` | ADS SPI clock. | ADS SPI transactions | Increase only after DRDY/read reliability is verified. |
| `CONFIG_ADS126X_HAS_ADC2` | bool | y | Builds ADC2 API support. | `ads126xAdcStartAdc2()`, `ads126xAdcStopAdc2()` | ADS1262 returns not supported for ADC2 use. |
| `CONFIG_ADS126X_HELPER_CREATE_SPI` | bool | n | Builds helper SPI create/destroy functions. | `ads126xAdcHelperCreateSpiDevice()` | Keep disabled in project integration. |
| `CONFIG_SENSORARRAY_SPI_USE_DMA` | bool | y | Allows SPI DMA. | ADS SPI helper/device config | Keep enabled unless diagnosing DMA issues. |
| `CONFIG_SENSORARRAY_SPI_MAX_TRANSFER_BYTES` | int | `64` | ADS SPI transfer buffer size. | `ads126xAdcInit()` | Increase only if larger SPI transactions are added. |
| `CONFIG_BOARD_SPI_HOST`, `CONFIG_BOARD_SPI_SCLK_GPIO`, `CONFIG_BOARD_SPI_MOSI_GPIO`, `CONFIG_BOARD_SPI_MISO_GPIO`, `CONFIG_BOARD_ADS126X_CS_GPIO`, `CONFIG_BOARD_ADS126X_DRDY_GPIO`, `CONFIG_BOARD_ADS126X_RESET_GPIO` | int | host `2`, SCLK `47`, MOSI `21`, MISO `14`, CS `-1`, DRDY `13`, RESET `38` | Board SPI and ADS control pins. | board bring-up and ADS init | Match board wiring. |
| `CONFIG_SENSORARRAY_ADS_READ_STOP1_BEFORE_MUX`, `CONFIG_SENSORARRAY_ADS_READ_SETTLE_AFTER_MUX_MS`, `CONFIG_SENSORARRAY_ADS_READ_START1_EVERY_READ`, `CONFIG_SENSORARRAY_ADS_READ_BASE_DISCARD_COUNT`, `CONFIG_SENSORARRAY_ADS_READ_RETRY_COUNT` | bool/int | n, `0`, y, `0`, `0` | ADS read sequencing policy. | `sensorarrayMeasureReadAdsPairUv()`, `sensorarrayMeasureReadAdsUv()` | These affect ADS measurement paths, not the current FDC production frame. |

### Mixed-mode configuration / 混合模式配置

| 配置项 / Option | 类型 / Type | 默认值 / Default | 作用 / Purpose | 影响的函数 / Affected functions | 调整建议 / Tuning notes |
|---|---:|---:|---|---|---|
| `CONFIG_SENSORARRAY_MATRIX_ROWS`, `CONFIG_SENSORARRAY_MATRIX_COLS` | int | `8`, `8` | Shared scan-plan dimensions. | `sensorarrayScanPlanBuildMixedExample()` | Current mixed engine delegates to FDC read; changing dimensions needs frame redesign. |
| `CONFIG_SENSORARRAY_FRAME_PERIOD_MS`, `CONFIG_SENSORARRAY_OVERSAMPLE` | int | FDC period, `1` | Generic matrix defaults. | matrix/mixed Kconfig defaults | No separate mixed-mode runtime Kconfig was found in the current source tree. |
| `CONFIG_MATRIX_ROWS`, `CONFIG_MATRIX_COLS`, `CONFIG_MATRIX_FRAME_PERIOD_MS`, `CONFIG_MATRIX_OVERSAMPLE`, `CONFIG_MATRIX_USE_RINGBUFFER` | int/bool | derive from SensorArray defaults, ringbuffer y | Legacy/shared matrix engine settings. | `core/matrixEngine` | Current production path does not use matrixEngine for FDC frame output. |

### Safety and diagnostic configuration / 安全与诊断配置

| 配置项 / Option | 类型 / Type | 默认值 / Default | 作用 / Purpose | 影响的函数 / Affected functions | 调整建议 / Tuning notes |
|---|---:|---:|---|---|---|
| `CONFIG_SENSORARRAY_MEASURE_LOCK_TIMEOUT_MS` | int | `5000` | Measurement mutex timeout. | `sensorarrayMeasureTakeLock()` | Increase only if legitimate long measurements hold the lock. |
| `CONFIG_SENSORARRAY_FDC_SUPPRESS_ALL_ZERO_FRAMES` | bool | Kconfig n, header fallback `1` when absent | Suppresses all-zero invalid frames where code path uses it. | FDC invalid-frame handling | Prefer diagnostics over suppression during bring-up. |
| `CONFIG_SENSORARRAY_FDC_VERBOSE_REG_DUMP` | bool | Kconfig: y | Dumps FDC registers on boot/rescue failure. | `sensorarrayFdcSweepDumpAllDeviceRegs()` | Disable to reduce failure-time logs. |
| `CONFIG_SENSORARRAY_FDC_DIAG_DUMP_REGS` | bool | defaults: n | Dumps FDC registers while diagnostic mode is active. | `sensorarrayRunDiagnosticTick()` | Enable when remote diagnosis needs register snapshots. |
| `CONFIG_SENSORARRAY_FDC_DIAG_DUMP_INTERVAL_MS` | int | defaults: `5000` | Diagnostic register dump period. | `sensorarrayRunDiagnosticTick()` | Lower increases serial and I2C load. |
| `CONFIG_SENSORARRAY_FDC_DIAG_DUMP_SKIP_OFFLINE_BUS` | bool | defaults: y | Skips diagnostic dump when a bus is offline. | `sensorarrayRunDiagnosticTick()` | Keep enabled to avoid repeated offline-bus errors. |
| `CONFIG_SENSORARRAY_FDC_VERIFY_MODE_FULL`, `CONFIG_SENSORARRAY_FDC_VERIFY_MODE_STARTUP_ONLY`, `CONFIG_SENSORARRAY_FDC_VERIFY_MODE_NONE` | choice | default `STARTUP_ONLY` | Runtime register readback policy. | FDC cache/sweep apply paths | `FULL` is for debug; `NONE` is high-speed experimentation only. |
| `CONFIG_SENSORARRAY_FDC_HIGH_SPEED_PROFILE` | bool | defaults: n | Experimental lower RCOUNT/SETTLECOUNT candidates. | `sensorarrayFdcSweep.c` | Can improve speed at the cost of noise, warnings, and invalid frames. |
| `CONFIG_TMUX1108_*`, `CONFIG_TMUX1134_*` | int/bool | component Kconfig defaults | TMUX GPIO and default logic levels. | `tmuxSwitchInit()`, route functions | Board-specific; wrong polarity breaks FDC/ADS route safety. |
| `CONFIG_POWER_*` | int | `-1` defaults | Optional power-control GPIO abstraction. | `core/powerCtrl` | Current main lifecycle does not actively manage charger/rail ICs through this layer. |

### Low-frequency diagnostic options / 低频调试配置

| 配置项 / Option | 类型 / Type | 默认值 / Default | 作用 / Purpose | 影响的函数 / Affected functions | 调整建议 / Tuning notes |
|---|---:|---:|---|---|---|
| `CONFIG_SENSORARRAY_FDC_VERBOSE_SCAN_LOG` | bool | defaults: n | Verbose per-row scan logs. | FDC row/scan helpers | Enable briefly; text output can reduce actual fps. |
| `CONFIG_SENSORARRAY_FDC_INTB_ENABLE` | bool | defaults: y | Enables FDC2214 INTB wake hints for formal matrix reads. | `sensorarrayMeasureEnsureFdcIntb()`, `sensorarrayMeasureFdcConfigBaseWithoutSleep()` | INTB is only a wake hint; STATUS `DRDY` plus unread bits remain authoritative. |
| `CONFIG_SENSORARRAY_FDC_INTB1_GPIO`, `CONFIG_SENSORARRAY_FDC_INTB2_GPIO` | int | `17`, `18` | Primary/secondary INTB GPIO numbers. | INTB setup/logging helpers | Match wiring; does not replace STATUS/unread validation. |
| `CONFIG_SENSORARRAY_FDC_INTB_ACTIVE_LOW`, `CONFIG_SENSORARRAY_FDC_INTB_TRIGGER_FALLING_EDGE`, `CONFIG_SENSORARRAY_FDC_INTB_TRIGGER_ANYEDGE`, `CONFIG_SENSORARRAY_FDC_INTB_WEAK_PULLUP` | bool | y, y, n, y | INTB GPIO polarity, interrupt edge, and weak pull-up policy. | INTB ISR setup and `FDC_INTB_GPIO` logs | Prefer active-low falling edge; any-edge is for diagnostics only. |
| `CONFIG_SENSORARRAY_FDC_INTB_WAIT_TIMEOUT_US` | int | `10000` | Per row-device ready wait timeout. | `sensorarrayFdcWaitDeviceReady()`, parallel worker done wait | Increase when valid unread bits arrive late; lower to fail faster. |
| `CONFIG_SENSORARRAY_FDC_REAPPLY_CACHE_ON_WARNING` | bool | defaults: n | Allows cache reapply after amplitude warnings. | `sensorarrayMeasureFillFdcMatrixRow()` / frame build rescue decision | Enable only when amplitude warnings are persistent and fresh. |
| `CONFIG_SENSORARRAY_FDC_WARNING_REAPPLY_THRESHOLD`, `CONFIG_SENSORARRAY_FDC_WARNING_REAPPLY_ONCE_PER_FINGERPRINT`, `CONFIG_SENSORARRAY_FDC_WARNING_REAPPLY_COOLDOWN_FRAMES` | int/bool | `2`, y, `50` | Controls warning-driven cache sanity reapply. | FDC frame build warning path | Tune to avoid repeated reapply loops. |
| `CONFIG_SENSORARRAY_FDC_AMPLITUDE_FAST_SWEEP_THRESHOLD`, `CONFIG_SENSORARRAY_FDC_WARNING_FAST_SWEEP_COOLDOWN_MS` | int | `4`, `1000` | Fresh amplitude warnings before fast sweep and its cooldown. | FDC warning rescue decision | Lower only when warnings are strong predictors of invalid data. |
| `CONFIG_SENSORARRAY_FDC_RESCUE_HARD_ERROR_THRESHOLD` | int | `4` | Hard runtime errors before rescue scheduling. | `sensorarrayMeasureRequestFdcCellRescue()` | Lower makes rescue more aggressive. |
| `CONFIG_SENSORARRAY_FDC_DIRECT_QUALITY_SAMPLES` | int | `6` | Direct-read quality sample count. | `sensorarrayFdcSweep.c` | More samples improve quality scoring but slow sweeps. |
| `CONFIG_SENSORARRAY_FDC_TIMING_LOG_EVERY_N_FRAMES` | int | deprecated alias, `10` | Deprecated timing interval alias. | compatibility fallback | Use `CONFIG_SENSORARRAY_FDC_TIMING_SUMMARY_EVERY_N_FRAMES` instead. |

## 关键数据结构 / Key data structures

### `sensorarrayAppContext_t`

| 字段 / Field | 生命周期与含义 / Lifecycle and meaning |
|---|---|
| `runtimeMode` | Set in `sensorarrayInitRuntime()` to FDC matrix. Controls `sensorarrayRunOneFrame()` dispatch. |
| `state` | Holds board, ADS, FDC and cache state. Updated by board/front-end init and measurement paths. |
| `scanPlan` | Built by `sensorarrayBuildDefaultScanPlan()` as 8 rows x 8 FDC cap operations. |
| `frame` | Current `sensorarrayFrame_t`, filled by measurement path and printed by output path. |
| `fdcEngine`, `adsEngine` | Engine facades initialised in `sensorarrayInitFrontends()`. FDC facade delegates to measurement/sweep code. |
| `fdcRescue` | Runtime all-invalid rescue context, reset during frontend init and ticked after each frame. |
| `primaryAddrValid`, `secondaryAddrValid` | Results of FDC I2C address parsing during runtime init. |
| `requestedFdcChannels` | Normalised FDC autoscan channel count; current matrix requires 4. |
| `fdcBootSweepOk` | Set true only when boot transport succeeds and boot quality is `OK`. Used in diagnostics. |
| `fdcBootSummary` | Last boot-sweep quality summary: valid/failed/cache-filled counts, row masks, quality and reason. |
| `fdcDegradedMode` | Set when the boot path allows partial hardware or the boot sweep reports degraded quality. |
| `fdcDiagnosticMode` | Set on primary/required-secondary/required-boot failures, non-OK required boot quality, or after too many full rescue failures. |
| `fdcFrameCounter` | Incremented once per main-loop frame read attempt. Also gates periodic stack/memory logs. |
| `failedRescueCount`, `rescueEpoch`, `lastFullRescueTimeUs`, `rescueRunning` | Full-sweep rescue throttle and diagnostic state used by `sensorarrayRunQueuedFullSweep()`. |

### `sensorarrayState_t`

| 字段 / Field | 生命周期与含义 / Lifecycle and meaning |
|---|---|
| `spiDevice`, `ads` | ADS SPI device/handle owned by board bring-up and ADS driver. |
| `adsReady`, `adsRefReady`, `adsAdc1Running`, `adsRefMuxValid`, `adsRefMux` | ADS state tracked so FDC route preparation can stop conversion and turn reference/VBIAS off. |
| `fdcPrimary`, `fdcSecondary` | Per-device FDC state for D1-D4 and D5-D8. |
| `fdcConfiguredChannels` | Requested/normalised FDC channel count. |
| `fdcCellCache[8][8]` | Per-cell FDC cache built by sweep/rescue paths. |
| `fdcAppliedRow[2]` | Per-device applied row-config shadow used for diff-only cache apply. |
| `boardReady`, `tmuxReady` | Set during board/routing init. Required by matrix readiness check. |

### `sensorarrayFdcDeviceState_t`

| 字段 / Field | 生命周期与含义 / Lifecycle and meaning |
|---|---|
| `label`, `i2cCtx`, `i2cAddr`, `handle`, `ready` | Device identity, bus context, I2C address, driver handle and readiness. |
| `haveIds`, `manufacturerId`, `deviceId`, `configVerified` | Bring-up diagnostics and ID/config verification state. |
| `refClockKnown`, `refClockSource`, `refClockHz` | Frequency conversion context. |
| `statusConfigReg`, `configReg`, `muxConfigReg` | Cached core registers used by runtime config and diagnostics. |
| `sweepProfile[4]` | Per-channel sweep profile information used during calibration/rescue. |

### `sensorarrayFrame_t` / `sensorarrayFdcMatrixFrame_t`

| 字段 / Field | 生命周期与含义 / Lifecycle and meaning |
|---|---|
| `timestampUs`, `sequence` | Initialised in `sensorarrayMeasureInitFdcMatrixFrame()`. |
| `freqHz[64]`, `capTotalPf[64]`, `raw28[64]` | Row-major cell data. Invalid `freqHz`/`capTotalPf` starts at `-1`. |
| `clockDividers[64]`, `driveCurrent[64]`, `deglitchCode[64]`, `effectiveFclkHz[64]` | Per-cell runtime config snapshot. |
| `validMask`, `capValidMask`, `freshMask`, `warnMask`, `errorMask` | Bit `i` maps to `index = (s - 1) * 8 + (d - 1)`. |
| `hardwareZeroRawCount`, `placeholderZeroCount`, `validCount`, `freshCount` | Frame quality counters. |
| `firstReadErr`, `firstBadRow`, `firstBadDevice`, `firstBadStatus`, `firstBadUnread` | First error diagnostics used by all-invalid logs and rescue. |

### FDC cache and rescue fields / FDC cache 与救援字段

`sensorarrayFdcCellConfigCache_t` stores per-cell cached RCOUNT, SETTLECOUNT, CLOCK_DIVIDERS, DRIVE_CURRENT, deglitch code, quality score, warning/error counters, timestamps, pending reapply/rescue flags, and degraded state. `sensorarrayFdcAppliedRowConfig_t` stores the last applied per-row/per-device register set and fingerprint so unchanged registers are skipped. `sensorarrayFdcRescueContext_t::allInvalidSequence` counts consecutive all-invalid frames and selects restore/resync/full-sweep actions.

## FDC 矩阵读取路径 / FDC matrix read path

### 函数级拆解 / Function-level breakdown

`sensorarrayMeasureReadFdcMatrixFrame()` is the production frame reader. Its current flow is:

1. Validate `outFrame`, initialise it with invalid sentinels, then validate `state`.
2. Take the global measurement mutex using `sensorarrayMeasureTakeLock()`.
3. Run `sensorarrayMeasureCheckFdcMatrixReady()`: board, TMUX, ADS and primary FDC must be ready; secondary absence is logged and treated as primary-only degraded operation.
4. Reset FDC I2C stats and read primary/secondary bus metadata.
5. Decide parallel eligibility from `CONFIG_SENSORARRAY_FDC_PARALLEL_DUAL_BUS_READ`, `CONFIG_SENSORARRAY_FDC_PARALLEL_DUAL_BUS_READ_SAFE`, `CONFIG_SENSORARRAY_FDC_FORCE_SINGLE_THREAD_READ`, both buses, different ports and worker availability.
6. Run `sensorarrayMeasureEnsureFdcMatrixPath(state, "fdc_matrix_frame")`; abort with `MATRIXFDC_DIAG,stage=read_abort` if route preparation fails.
7. Initialise workers on first eligible parallel frame. On worker init/queue/read failure, print `FDC_PARALLEL_FALLBACK`; worker timeout/fallback waits for the queued worker to become idle before serial fallback touches shared FDC handles or output buffers.
8. Run the formal precheck once when secondary is available.
9. For each row S1..S8, create one row epoch, apply cached row config with diff writes, and read primary D1-D4 plus secondary D5-D8 by parallel or serial row-epoch helper. A missing formal cache logs `FDC_CACHE_MISS`, marks affected cells invalid, requests fast rescue, and skips the normal read for that device.
10. Call `sensorarrayMeasureFillFdcMatrixRow()` to merge row samples into masks, raw values, frequencies and warning/error fields.
11. Accumulate timing, warning, I2C and rescue health.
12. Compute `capTotalPf` from `freqHz` and `CONFIG_SENSORARRAY_FDC_TANK_INDUCTOR_NH`.
13. Print row, frame, timing, cache, I2C, worker and validity summaries when profile settings require them.
14. Release the measurement mutex.
15. If `validMask == 0`, print all-invalid diagnostics, report all-invalid frame to sweep/rescue, and return an error. Otherwise return the first row/path error, or `ESP_OK`.

### Row epoch state machine / Row epoch 状态机

One row equals one conversion epoch. Parallel mode aligns primary and secondary worker jobs for the same row, but it is I2C/task parallelism, not a mathematical guarantee that both chips sampled at the exact same instant.

```mermaid
stateDiagram-v2
    [*] --> EnsureFdcPath
    EnsureFdcPath --> EnterSleep
    EnterSleep --> SelectRowWhileSleeping
    SelectRowWhileSleeping --> RowSettle
    RowSettle --> ApplyCachedRowConfig
    ApplyCachedRowConfig --> ExitSleep
    ExitSleep --> WaitStatusUnread
    WaitStatusUnread --> ReadPrimaryD1D4
    WaitStatusUnread --> ReadSecondaryD5D8
    ReadPrimaryD1D4 --> MergeRow
    ReadSecondaryD5D8 --> MergeRow
    MergeRow --> ComputeCapTotalPf
    ComputeCapTotalPf --> UpdateMasksAndTiming
    UpdateMasksAndTiming --> [*]
```

Relevant functions include `sensorarrayMeasureReadFdcMatrixRowParallelEpoch()`, `sensorarrayMeasureReadFdcMatrixRowSerialEpoch()`, `sensorarrayMeasureFdcSetSleepMode()`, `sensorarrayMeasureSelectFdcRowWhileSleeping()`, `sensorarrayMeasureApplyFdcCachedRowConfig()`, `sensorarrayMeasureFdcRunDeviceEpochAfterSleep()`, `sensorarrayFdcWaitDeviceReady()`, `sensorarrayMeasureReadFdcAutoscan4chMasked()` and `sensorarrayMeasureFillFdcMatrixRow()`.

### Data/output pipeline / 数据输出流水线

```mermaid
flowchart LR
    LC[LC tank] --> FDC[FDC2214 raw28]
    FDC --> Freq[freqHz]
    Freq --> Cap[capTotalPf]
    Cap --> Frame[64-cell row-major frame]
    Frame --> Text[MATRIXFDC_CAP printf output]
```

### INTB and STATUS / INTB 与 STATUS

The default formal matrix ready path is strict INTB-level first, with STATUS used only after INTB to acknowledge and verify readiness:

- FDC DRDY-to-INTB output is enabled for formal matrix reads unless `CONFIG_SENSORARRAY_FDC_DISABLE_INTB_FOR_DEBUG` or poll-only ready policy is selected.
- The authoritative ready gate is `DRDY=1` plus all required unread bits set after INTB low.
- In strict INTB mode the worker arms INTB notification before `sleep_exit`, waits for INTB first, then performs one STATUS ack/verify. Repeated STATUS polling before INTB is suppressed.
- If INTB arrives but STATUS shows `unread=0xF,DRDY=0`, a short after-INTB recheck can recover as `src=AR`.
- Missed INTB in the formal strict path is `src=IT` and is handed to the row-device watchdog instead of silently recovering through STATUS polling.
- `CONFIG_SENSORARRAY_FDC_READY_POLICY_INTB_WITH_POLL_FALLBACK` and `CONFIG_SENSORARRAY_FDC_READY_POLICY_POLL_ONLY` are legacy/debug policies. Formal strict logs should not show `src=FB`.

中文：默认正式矩阵读取已经切换为 INTB-first。INTB 只是唤醒提示，最终仍以 `DRDY=1 && required unread full` 为有效就绪条件；`DRDY=0` 时即使 unread 为满也会标记为 unsafe，不读取数据寄存器。

## 路由安全与 ADS/FDC 互斥 / Route safety and ADS/FDC mutual exclusion

`sensorarrayMeasurePrepareFdcMatrixPath()` and `sensorarrayMeasureEnsureFdcMatrixPath()` are the FDC capacitance route safety gate.

FDC mode requires:

```text
SW -> GND source
SELA -> FDC2214 path
SELB -> board-defined FDC policy
TMUX1134 EN -> enabled
ADS conversion stopped
ADS internal reference off
ADS VBIAS off
```

`sensorarrayMeasureEnsureFdcMatrixPath()` first reads the current commanded/observed control state and ADS ref state. It only calls `sensorarrayMeasurePrepareFdcMatrixPath()` when it finds a mismatch. Relevant logs include:

```text
FDC_PATH,stage=ads_stop
FDC_PATH,stage=ads_stop2
FDC_PATH,stage=ads_ref_off
FDC_PATH,stage=ads_vbias_off
FDC_PATH,stage=tmux1134_fdc
FDC_PATH,stage=selb_fdc
FDC_PATH,stage=sw_gnd
FDC_PATH,stage=tmux1108_enable
FDC_PATH,stage=prepare_done
FDC_PATH,stage=ensure_mismatch
FDC_PATH,stage=ensure_ok        # only when verbose scan logging is enabled
```

This is a measurement-layer policy. The ADS driver does not decide when ADS must be off for FDC; it only exposes `ads126xAdcStopAdc1()`, `ads126xAdcStopAdc2()`, `ads126xAdcSetRefMux()`, `ads126xAdcSetVbiasEnabled()` and related chip-level APIs.

## 帧格式与无效值策略 / Frame format and invalid data policy

Default text output is capacitance:

```text
S,s=<n>,vc=<n>,ic=<n>,fc=<n>,cm=0x...,vm=0x...,wm=0x...,em=0x...,br=<n>,bd=<n>
MATRIXFDC_CAP,s=<n>,t=<us>,pa=<0|1>,q=<F|P>,cm=0x...,fm=0x...,wm=0x...,em=0x...,iv=-1.000000,pf=[...64 values with 6 decimals by default...]
```

Optional frequency output is compiled when `CONFIG_SENSORARRAY_FDC_TEXT_OUTPUT_FREQ_HZ` or `CONFIG_SENSORARRAY_FDC_TEXT_OUTPUT_BOTH_SEPARATE` is selected:

```text
MATRIXFDC_FREQ,seq=<n>,timestampUs=<us>,validMask=0x...,warnMask=0x...,errorMask=0x...,freqHz=[...64 values...]
```

Raw debug output is compiled only when `CONFIG_SENSORARRAY_FDC_RAW_DEBUG_LOG` is enabled:

```text
DEBUGFDC_RAW,seq=<n>,timestampUs=<us>,raw28=[...64 values...]
```

Invalid policy:

- `-1.000000` is the invalid sentinel for `capTotalPf` and invalid `freqHz` initialisation.
- `0 pF` must not be treated as invalid unless a separate display layer explicitly documents it as a placeholder.
- `raw28 == 0` from a fresh sample increments `hardwareZeroRawCount` and is invalid for FDC capacitance.
- `capValidMask` is set only after valid raw/frequency data successfully converts to pF.
- `warnMask` marks degraded but still usable data, such as amplitude warning cases that remain fresh and non-zero.
- `errorMask` marks cells with I2C errors, missing unread conversion, zero raw, watchdog fault, saturation, or failed capacitance conversion.

## 调试日志与控制命令 / Diagnostic logs and console commands

### 实际日志 / Actual logs

The current source tree emits these relevant log tags or lines:

```text
APP_INIT
APP_FATAL
APP_FDC_BOOT
APP_FDC
APP_MEM
APP_STACK
BOARD_I2C_CFG
BOARD_I2C_READY
BOARD_I2C_XFER
BOARD_I2C_ERR
BOARD_I2C_RECOVERY
ROUTEMAP
FDCMAP
FDC_FATAL
FDC_BUS_WARN
FDC_BOOT_MATRIX_SWEEP_DONE
FDC_SWEEP
FDC_ROW_SWEEP_RESULT
FDC_ROW_SWEEP_REJECT
FDC_PATH
FDC_PARALLEL_CFG
FDC_PARALLEL_WARN
FDC_PARALLEL_FALLBACK
FDC_WORKER_TIMEOUT
FDC_STALE_RESULT_DROPPED
FDC_FORMAL_PRECHECK
FDC_INTB_CFG
FDC_INTB_CONFIG_BAD
FDC_INTB_GPIO
FDC_ROW_EPOCH
FDC_READY
RB
RP
RR
SR
RWD
FB
RS
RE
PS
D4
D4C
FDC_RESULT_MERGE_BUG
FDC_CACHE_MISS
FDC_DEFERRED_REPAIR
FDC_RESCUE
FDC_RESCUE_DECISION
FDC_RESCUE_SUPPRESSED
S
OT
OV
BN
PFU
P5
PR
R5
T5
Q5
I5
MATRIXFDC_CAP
MATRIXFDC_FREQ
MATRIXFDC_DIAG
DEBUGFDC_RAW
FRAME_ERROR
```

### Compact FDC log tokens / FDC 紧凑日志符号

The compact tokens are intended for high-frame-rate timing work, where long log lines can distort the result:

| Token | Meaning | Important fields |
|---|---|---|
| `FDC_INTB_CFG` | Formal precheck readback for FDC INTB configuration. | `config`, `statusConfig`, `drdy2int`, `intbDis`, `verified` |
| `FDC_INTB_CONFIG_BAD` | INTB output was expected but CONFIG/STATUS_CONFIG readback did not match, so that device degrades to poll-only readiness. | `dev`, `config`, `statusConfig`, `drdy2int`, `intbDis` |
| `FDC_INTB_GPIO` | INTB GPIO setup for a device. | `dev`, `gpio`, `level`, `edgeCount`, `pullup`, `intr` |
| `RB` | Ready wait begins for one FDC device. | `r`, `d`, `pol`, `to` |
| `RP` | One STATUS observation used after INTB, after-INTB recheck, legacy fallback, or poll-only diagnostics. | `k`, `st`, `u`, `drdy`, `full`, `ok` |
| `RR` | Ready wait result. | `err`, `ok`, `kind`, `src`, `iw`, `su`, `fb` |
| `SR` | STATUS-read counters for the ready wait. | `bi`, `ai`, `ar`, `pd`, `fb`, `supp` |
| `RWD` | Row-device watchdog action after a hard row-device fault. | `r`, `d`, `why`, `act`, `hard` |
| `D4` | Read4 anomaly or sampled diagnostic. | `r`, `d`, `err`, `vm`, `fm`, `um`, `raw0` |
| `D4C` | Read4 consistency detail. | `r`, `d`, `zm`, `sat`, `warn`, `drdy` |
| `RE` | Per-row parallel primary/secondary timing alignment. | `span`, `ser`, `eff`, `sleepDx`, `readyDx`, `intbDx`, `statDx`, `readDx` |
| `T5` | Compact aggregate timing summary for the last timing window. | `n`, `fps`, `ready`, `worker`, `op`, `ov` |
| `R5` | Compact ready-state summary. | `n`, `ok`, `it`, `ar`, `si`, `fb` |
| `Q5` | Compact frame-quality/sweep summary. | `n`, `full`, `part`, `rescue`, `zero`, `sat` |
| `I5` | Compact I2C summary. | `n`, `wr`, `rd`, `err`, `nack`, `to` |
| `P5` | Compact profile summary. | `n`, `avg`, `max`, `target`, `rowBudget`, `profileTooSlow`, `profileTooSlowAction`, `fpsMax` |
| `PR` | Per-row/device profile detail when profile is slow or row profile logging is enabled. | `r`, `d`, `ch`, `round`, `rc`, `sc`, `cd`, `dc`, `dg` |
| `PFU` | FDC Profile Update / Profile Fast or diagnostic Update for a row/device. | `r`, `d`, `why`, `action`, `round` or `oldRound/newRound`, `target`, `rowBudget`, `rCount`, `newRcount`, `decisionReason` |
| `FDC_EPOCH` | Sleep/row/cache/exit-sleep epoch diagnostic. | `stage`, `row`, `dev`, `cfg`, `mux`, `status`, `unread`, `drdy`, `intbLevel` |
| `FDC_PARALLEL_FALLBACK` | Parallel worker fallback/join diagnostic. | `reason`, `workerDeadlineUs`, `workerJoinUs`, `parentWaitUs`, `workerTimedOut`, `workerLateDoneUs`, `staleDiscarded`, `rowHardDeadlineUs` |
| `FDC_RESULT_MERGE_BUG` | A guard detected an attempt to invalidate or lose a row-device whose final read4 data is complete. | `dev`, `row`, `epoch`, `validMask`, `freshMask`, `unread`, `drdy` |
| `FDC_DEFERRED_REPAIR` | A failed secondary/primary device path was not repaired inline in the realtime row path; affected cells are marked invalid and rescue is requested later. | `row`, `dev`, `reason`, `status`, `unread`, `drdy`, `action` |

Readiness diagnostics:

- `src=IL` or `src=IE` means INTB level/event arrived and one STATUS ack verified `DRDY=1` plus required unread bits.
- `src=AR` means only the short after-INTB recheck path recovered `unread=0xF,DRDY=0`.
- `src=IT` means INTB timed out after final STATUS poll/retry did not prove readiness; `k` distinguishes `intb_timeout_before_estimated_round`, `wait_clamped_by_hard_deadline`, `intb_timeout_final_status_not_ready`, and final-poll I2C errors.
- `src=FP` means an INTB timeout was recovered by the mandatory final STATUS poll. `src=FR` means the bounded retry recovered readiness.
- `src=DI` means STATUS after INTB was inconsistent and is treated as a hard row-device fault.
- `src=FB` should only appear when the legacy fallback policy is selected.
- `bi` should stay at 0 in strict INTB mode; `ai` is the single after-INTB ack read, and `ar` counts bounded after-INTB rechecks.
- A final read4 result is data-complete-good only when read/I2C errors are clear, `DRDY=1`, required unread/fresh/valid masks are full, raw data is not all zero, and zero-before/after-DRDY masks are clear.

Parallel row epoch logic:

- Both workers first enter FDC sleep and acknowledge the row epoch.
- The scan task switches the row while both FDCs are sleeping, applies both cached row configs, then releases both workers.
- Workers perform `sleep_exit -> INTB wait/status verify -> read4` in parallel, which keeps primary and secondary measurements in the same row epoch.
- Parallel arbitration uses the final read4 data-complete-good predicate; hard ready/read4 faults are handled by the row-device watchdog before merge.
- Runtime repair/resync is deferred out of the realtime row path only for true hard faults; recovered full read4 samples are kept for frame fill.

### Console commands / 控制命令

Current source search found no `esp_console_cmd_register` call. Therefore these command names are not documented as user-callable console commands in this source tree:

| 名称 / Name | 当前源码树状态 / Current source-tree status |
|---|---|
| `force_full_sweep` | Not present as a registered console command / 当前未发现注册为 console command |
| `fdc_diag` | Not present as a registered console command / 当前未发现注册为 console command |
| `fdc_boot_sweep` | Not present as a registered console command / 当前未发现注册为 console command |
| `fdc_rescue` | Not present as a registered console command / 当前未发现注册为 console command |
| `fdc_period_ms` | Not present as a registered console command / 当前未发现注册为 console command |
| `fdc_profile` | Not present as a registered console command; profile setters exist as C APIs / 未注册为 console command；存在 C API setter |
| `fdc_i2c_trace` | Not present as a registered console command; trace control exists as C APIs in `fdc2214Cap` / 未注册为 console command；trace 控制是 C API |
| `fdc_discard_frames` | Not present as a registered console command; `sensorarrayMeasureFdcSetDiscardFrames()` exists as C API / 未注册为 console command；存在 C API |

## 编译、烧录和监视 / Build, flash, and monitor

### Windows PowerShell ESP-IDF 环境 / Windows PowerShell ESP-IDF environment

This is the recommended Windows/PowerShell workflow for this project:

```powershell
$env:IDF_PATH = "C:\Espressif\frameworks\esp-idf-v5.5.1"
$env:IDF_PYTHON_ENV_PATH = "C:\Espressif\python_env\idf5.5_py3.11_env"

$pythonDir = "C:\Espressif\python_env\idf5.5_py3.11_env\Scripts"
$cmakeDir = "C:\Espressif\tools\cmake\3.30.2\bin"
$ninjaDir = "C:\Espressif\tools\ninja\1.12.1"
$gccDir = "C:\Espressif\tools\xtensa-esp-elf\esp-14.2.0_20241119\xtensa-esp-elf\bin"

$env:Path = "$pythonDir;$cmakeDir;$ninjaDir;$gccDir;C:\Espressif\frameworks\esp-idf-v5.5.1\tools;$env:Path"

function idf {
    & "$pythonDir\python.exe" "$env:IDF_PATH\tools\idf.py" @args
}

idf --version
python --version
cmake --version
ninja --version
xtensa-esp32s3-elf-gcc --version

cd C:\ESP32\SensorArray
idf build
idf -p COMx flash monitor
```

New-board flash:

```powershell
idf -p COMx erase-flash
idf -p COMx flash monitor
```

Readback verification:

```powershell
python -m esptool --chip esp32s3 -p COMx read_flash 0x0 0x100 flash_0x0_after.bin
```

正常情况下 readback 不应全是 `FF`。ESP image header 常见起始 byte 为 `E9`。

In normal cases the readback should not be all `FF`. ESP image headers commonly start with byte `E9`.

## 已知约束 / Known constraints

- Current production output is text. Binary transport is not the default and the current send stub returns unsupported.
- Primary FDC absence is a serious boot error. Secondary FDC absence can be primary-only only when dual-FDC boot is not required.
- ADS matrix read is not a production frame path in this source tree.
- Mixed row mode currently delegates to FDC matrix read.
- Formal FDC matrix readiness is `INTB_STRICT_LEVEL` by default; STATUS is read once after INTB as the ack/verify gate, and legacy fallback is disabled.
- High-density text logs affect frame rate, serial stability, and timing measurements.
- FDC pF values depend on `CONFIG_SENSORARRAY_FDC_TANK_INDUCTOR_NH`; raw28 does not.
- Changing matrix dimensions, binary output, or mixed ADS/FDC scheduling is architecture work, not a README-level setting.

## 文件地图 / File map

| 路径 / Path | 说明 / Description |
|---|---|
| `main/main.c` | Application lifecycle orchestration, boot calibration call, main loop, safe idle. |
| `main/output/` | Text frame output. |
| `main/Kconfig.projbuild` | Main project Kconfig. |
| `main/sensorarrayConfig.h` | Compile-time fallback defaults and hardware constants. |
| `main/sensorarrayTypes.h` | Shared types, frame fields, board/FDC/ADS state. |
| `core/board/` | Board map and bring-up helpers. |
| `core/boardSupport/` | I2C/SPI/GPIO resource ownership and I2C recovery. |
| `core/measure/sensorarrayMeasure.c` | Measurement route policy, FDC frame reader, ADS helper includes, profile setters. |
| `core/measure/fdc/sensorarrayFdcMatrix.c` | Thin FDC engine facade. |
| `core/measure/fdc/sensorarrayFdcSweep.c` | Boot sweep, cache calibration, rescue sweep implementation. |
| `core/measure/fdc/sensorarrayFdcRescue.c` | Runtime all-invalid rescue policy. |
| `core/measure/fdc/*.inc` | FDC row epoch, cache apply, read4, frame build, conversion helpers. |
| `core/measure/ads/` | ADS measurement helpers and ADS matrix engine stub. |
| `core/measure/mixed/` | Mixed row engine stub delegating to FDC path. |
| `components/fdc2214Cap/` | Chip-level FDC2214/FDC2212 I2C driver. |
| `components/ads126xAdc/` | Chip-level ADS126x SPI driver. |
| `components/tmuxSwitch/` | GPIO/control primitive layer for TMUX1108/TMUX1134. |
| `transport/` | Transport/protocol scaffolding, not current default frame output. |
| `docs/architecture.md` | Short architecture notes. |
| `datasheets/` | Reference PDFs and schematic images. |
| `example/` | Standalone examples, not part of main firmware lifecycle. |

## 2026-06 FDC Runtime Ready, Watchdog, Profile, And Compact Logs

This section documents the current FDC runtime behaviour after the INTB/STATUS and log-pressure fix. The code source of truth is still `main/Kconfig.projbuild`, `main/sensorarrayConfig.h`, `core/measure/sensorarrayMeasure.c`, `core/measure/fdc/*.inc`, and `main/output/sensorarrayFrameOutput.c`.

### FDC Ready State Machine

The formal matrix read path now defaults to `SENSORARRAY_FDC_READY_POLICY_INTB_STRICT_LEVEL`.

- INTB is treated as level-latched active-low, not only as a falling edge.
- The waiter arms the current task first, clears stale notifications, records the edge counter, then reads the initial INTB level.
- If INTB is already low, the row-device is treated as latched-ready and the code reads STATUS once.
- If INTB is high, the code waits for an edge or a final low level using `actualIntbWaitUs = max(CONFIG_SENSORARRAY_FDC_INTB_WAIT_TIMEOUT_US, estimatedRoundUs + CONFIG_SENSORARRAY_FDC_ROW_WAIT_SAFETY_US)`, clamped by the row-device hard deadline.
- If INTB never goes low, the strict path still performs a final STATUS poll plus CONFIG/MUX_CONFIG/STATUS_CONFIG/INTB diagnostics. A final `DRDY=1` with the required unread mask is accepted and DATA is read; otherwise one bounded retry is allowed when the hard deadline still has room.
- Once INTB is confirmed low, STATUS is read once as an acknowledge and verify operation. STATUS is not a harmless peek.
- `DRDY=1` plus the required unread mask means DATA can be read.
- `DRDY=0` plus the required unread mask is allowed only in the after-INTB micro recheck window.
- Other STATUS states after INTB low are treated as inconsistent and go to row-device watchdog handling.
- `INTB_WITH_POLL_FALLBACK` and `POLL_ONLY` remain available only as legacy or diagnostic modes.

After-INTB recheck defaults:

| Option | Default | Meaning |
|---|---:|---|
| `CONFIG_SENSORARRAY_FDC_AFTER_INTB_RECHECK_ENABLE` | `y` | Enables the short post-INTB recheck window. |
| `CONFIG_SENSORARRAY_FDC_AFTER_INTB_RECHECK_MAX` | `3` | Maximum STATUS rechecks after confirmed INTB. |
| `CONFIG_SENSORARRAY_FDC_AFTER_INTB_RECHECK_INTERVAL_US` | `250` | Delay between rechecks. |
| `CONFIG_SENSORARRAY_FDC_AFTER_INTB_RECHECK_DEADLINE_US` | `1000` | Total recheck deadline. |

STATUS read accounting and the actual wait budget are visible in compact ready logs:

```text
RB,d=p,r=2,e=5050,to=30088,iw=15544,est=13544,hard=62500,hardRemain=58000,waitSource=estimated_round,estKind=autoscan_4ch_round,req=F,lvl=1,edge=10,mode=INTB_STRICT_LEVEL,intb=1
RR,d=p,r=2,e=5050,src=FP,st=C04F,u=F,dr=1,wu=15620,pc=1,k=intb_timeout_recovered_by_final_status,ack=1,ib0=1,ib1=1,err=0,iw=15544,est=13544,hardRemain=42000,elapsed=15620,fp=1,fr=0,fSt=C04F,fU=F,fDr=1,fCfg=1601,fMux=C20D,fErrCfg=0001,fIg=1
SR,d=p,r=2,e=5050,b=0,a=0,ar=0,fb=0,fp=1,pd=0,supp=1,ack=1,ib0=1,ib1=1
```

`ack=1` or higher means STATUS was read as an acknowledge. `ib0=0,ib1=1` means INTB was low before STATUS and high after STATUS, which is expected when STATUS clears the latch.
`estKind=autoscan_4ch_round` is required for the normal `requestedMask=0xF` autoscan path; single-channel estimates must not be used for 4-channel unread waits.

### Row-Device Watchdog And Recovery

The row-device watchdog replaces the old "missed INTB then STATUS fallback poll" production path.

Hard timeout formula:

```text
singleRowBudgetUs = CONFIG_SENSORARRAY_TARGET_FRAME_PERIOD_US / rowCount
rowDeviceWatchdogHardTimeoutUs = singleRowBudgetUs * CONFIG_SENSORARRAY_FDC_ROW_DEVICE_WATCHDOG_MULTIPLIER
```

In this firmware the target frame budget is derived from `CONFIG_SENSORARRAY_FDC_MATRIX_TARGET_FPS`. At the default 20 fps and 8 rows:

```text
targetFrameUs = 50000
singleRowBudgetUs = 6250
multiplier = 10
hardTimeoutUs = 62500
```

This is not a fixed 100 ms timeout and it is not a STATUS polling timeout. `CONFIG_SENSORARRAY_FDC_ROW_DEVICE_WATCHDOG_OVERRIDE_US=0` keeps the dynamic formula. A non-zero override uses the override value.

Watchdog reasons include:

| Reason | Typical source |
|---|---|
| `intb_timeout` | INTB did not go low before the strict wait deadline. |
| `drdy_not_closed_after_intb` | INTB was confirmed but `unread=F,DRDY=0` did not close inside the micro recheck window. |
| `status_inconsistent_after_intb` | STATUS after confirmed INTB did not match a valid read state. |
| `read4_i2c_error` | DATA register read failed. |
| `zero_after_drdy` | DATA returned zero after ready was confirmed. |
| `raw_all_zero` | All four raw channels were zero. |
| `amplitude_warning` | Fresh amplitude warning crossed policy threshold. |
| `watchdog_fault` | FDC sensor watchdog fault was seen. |
| `saturated` | Raw value reached the saturation threshold. |
| `profile_too_slow` | The shadow profile round estimate exceeds the warn budget. This is a timing diagnostic by default, not an electrical failure or automatic rescue reason. |

The compact watchdog line includes the configured retry limit and whether this row path had actually attempted a retry:

```text
RWD,d=p,r=2,e=5050,why=intb_timeout,rowBudget=6250,mul=10,hard=62500,override=0,retryMax=1,retryActual=0,rescueAction=request_cell_rescue
```

### FDC Profile Logging

The formal read path keeps a shadow profile snapshot when cached row config is applied. It does not read back all FDC config registers in the DATA hot path.

The snapshot records:

- `RCOUNT`
- `SETTLECOUNT`
- `CLOCK_DIVIDERS`
- `DRIVE_CURRENT`
- deglitch code
- effective FCLK
- per-channel settle, convert, switch, and total time
- 4-channel autoscan round estimate
- expected ready timeout
- mux, STATUS_CONFIG, and CONFIG shadow values

The estimate follows the FDC timing model used by the code:

```text
settleUs  = ceil(SETTLECOUNT * 16 / fREF)
convertUs = ceil(RCOUNT * 16 / fREF)
roundUs   = sum(settleUs + convertUs + switchUs for required channels)
```

This is how a line such as `est=13544` is explained: `P5` and `PR` show the register values and per-channel timing that sum to the round estimate.

Default profile logs:

```text
P5,s=630,n=5,cnt=16,avg=13544,max=13544,row=1,d=p,ch=2,target=4000,rowBudget=6250,profileTooSlow=1,profileTooSlowAction=diag_only,theoreticalMaxFps=9.22,rc=[2089,2089,2089,2089],sc=[0080,0080,0080,0080],cd=[1001,1001,1001,1001],dc=[7800,7800,7800,7800],dg=3,round=13544
PR,s=630,d=p,r=1,src=shadow,why=profile_too_slow,rc=[2089,2089,2089,2089],sc=[0080,0080,0080,0080],cd=[1001,1001,1001,1001],dc=[7800,7800,7800,7800],dg=3,fh=[40000000,40000000,40000000,40000000],su=[...],cu=[...],tu=[...],round=13544,to=30088
PFU,d=p,r=1,why=profile_too_slow,action=disabled_diag_only,round=13544,target=4000,rowBudget=6250,rCount=[0x2089,0x2089,0x2089,0x2089],settle=[0x0080,0x0080,0x0080,0x0080]
```

Readback verification is reserved for boot/init, cache writes, sweep updates, diagnostics, or non-hot-path checks. Do not add config register readbacks between confirmed INTB and DATA reads.

### Formal Fast Profile

The firmware keeps formal fast profile metadata alongside the stable boot/cache profile:

| Field | Meaning |
|---|---|
| `bootStableProfile` | Conservative profile derived from boot/full/fast calibration cache. |
| `formalFastProfile` | Formal read profile metadata used to judge high-speed readiness. |
| `CONFIG_SENSORARRAY_FDC_FORMAL_FAST_PROFILE_ENABLE` | Disabled by default. When enabled, permits experimental runtime RCOUNT reduction. |
| `CONFIG_SENSORARRAY_FDC_FORMAL_FAST_TARGET_ROUND_US` | Default target is `4000 us`; with fast profile disabled it is diagnostic only. |
| `CONFIG_SENSORARRAY_FDC_PROFILE_TOO_SLOW_WARN_US` | Default warning budget is `6250 us`, the 20 fps row budget. |
| `CONFIG_SENSORARRAY_FDC_PROFILE_TOO_SLOW_RESCUE_ENABLE` | Disabled by default. Allows slow-profile diagnostics to request rescue only when explicitly enabled. |

If `autoscanRoundUs` exceeds the warning budget, the default cache-apply path treats this as a timing diagnostic. It logs PFU with `action=disabled_diag_only` or `profile_too_slow_diag_only`, keeps the cached profile unchanged, does not reduce RCOUNT, and does not request rescue or sweep. `4000 us` is the formal fast target, not a measured round value; with fast profile disabled it is shown only for comparison and no longer forces a `13544 -> 4000` mutation.

Default example:

```text
PFU,d=p,r=1,why=profile_too_slow,action=disabled_diag_only,round=13544,target=4000,rowBudget=6250,rCount=[0x2089,0x2089,0x2089,0x2089],settle=[0x0080,0x0080,0x0080,0x0080]
```

When `CONFIG_SENSORARRAY_FDC_FORMAL_FAST_PROFILE_ENABLE=y`, PFU actions distinguish actual writes and cache promotion:

- `apply_fast_profile_and_write`: fast profile changed the expected profile and wrote FDC registers.
- `fast_profile_already_applied_no_write`: fast profile was computed but already matched applied registers.
- `promote_fast_profile_cache_only`: the runtime fast profile was written back to per-cell cache without a register write.
- `promote_fast_profile_and_write`: the runtime fast profile was written back to per-cell cache and registers were written.
- `apply_cached_profile_write`: normal cached profile write, not fast profile.
- `no_write_already_applied`: expected and applied profiles already matched.

### Quality-Based Sweep

Runtime fast/full sweep decisions are quality based, not only invalid-frame based. Fast sweep can be requested by:

- persistent fresh amplitude warnings
- sensor watchdog fault
- saturation
- zero raw after DRDY
- all-zero raw
- I2C/read4 error
- repeated ready-state recovery or row-device watchdog activity
- drive/current/profile margin issues

`profile_too_slow` is excluded from automatic fast/full sweep by default. It can request rescue only when `CONFIG_SENSORARRAY_FDC_PROFILE_TOO_SLOW_RESCUE_ENABLE=y`, which is intended for explicit debug experiments.

Full sweep remains reserved for broader or repeated failures, such as repeated fast sweep failure, multiple row/channel faults, large-area invalid/stale frames, or persistent profile quality loss. A single `unread=F,DRDY=0` after INTB does not directly force full sweep; it first goes through after-INTB recheck and row-device recovery.

For all-invalid frames, escalation is deterministic: sequence 1 restores autoscan/exit-sleep state, sequence 2 forces a soft resync/cache dirty path, and sequence 3 or later queues a full sweep unless an explicit cooldown, pending request, max-fail, or manual-disable policy is active. `-1` in `MATRIXFDC_CAP` means no trustworthy fresh conversion was obtained, not a physical capacitance of `-1`; repeated `-1` frames drive this rescue escalation.

### Compact Log Dictionary

Default log format is compact key/value. `MATRIXFDC_CAP` is kept by default for GUI compatibility, but the field names are shorter. The `pf` array still defaults to the original 6 decimal places.

Tag dictionary:

| Legacy or concept | Compact tag |
|---|---|
| `MATRIXFDC_CAP` | `MATRIXFDC_CAP` by default, or `C` if legacy cap tag is disabled |
| `FDC_FRAME_OUTPUT` | `FO` in legacy mode only |
| `FDC_FRAME_SUMMARY` | `S` |
| `FDC_OUTPUT_TIMING` | `OT` |
| `SCAN_TIMING_OVERRUN` | `OV` |
| `SCAN_BOTTLENECK` | `BN` |
| `FRB` | `RB` |
| `FRP` | `RP` |
| `FRR` | `RR` |
| `SRC` | `SR` |
| `FPR` | `RE` |
| `FDC_ROW_SUMMARY` | `RS` |
| `FDC_DEVICE_READ4` | `D4` |
| read consistency detail | `D4C` |
| row-device watchdog | `RWD` |
| profile summary | `P5` |
| profile row detail | `PR` |
| profile update | `PFU` |
| ready summary | `R5` |
| timing summary | `T5` |
| quality/sweep summary | `Q5` |
| I2C summary | `I5` |

Field dictionary:

| Field | Meaning |
|---|---|
| `s` | sequence |
| `s0`, `s1` | sequence window start/end |
| `t` | timestamp us |
| `r` | row |
| `e` | epoch |
| `d` | device token, `p` or `s` |
| `q` | frame quality, `F` full or `P` partial |
| `pa` | partial |
| `cv`, `fm`, `vm`, `wm`, `em` | cap valid, fresh, valid, warning, error masks |
| `pf` | capacitance pF array, default 6 decimals |
| `fq` | frequency Hz |
| `raw` | raw28 values |
| `st` | STATUS register |
| `u` | unread mask |
| `dr` | DRDY |
| `wu`, `ru` | wait and read time us |
| `pc` | poll/read count |
| `ed` | INTB edge delta |
| `rm` | ready mode |
| `er` | error code |
| `to` | timeout |
| `nr`, `z0`, `zd`, `az` | not ready, zero before ready, zero after DRDY, raw all zero |
| `op` | output printf time us |
| `fu`, `pu` | frame and period time us |
| `ml`, `ww` | measure lock held, worker done wait |
| `rc`, `sc`, `cd`, `dc`, `dg` | RCOUNT, SETTLECOUNT, CLOCK_DIVIDERS, DRIVE_CURRENT, deglitch |
| `round` | estimated autoscan round us |
| `ack` | STATUS read acknowledge count |
| `ib0`, `ib1` | INTB level before and after STATUS |
| `src` | ready source: `IL`, `IE`, `AR`, `IT`, `FB`, `PD` |

Mask shorthand:

| Token | Meaning |
|---|---|
| `*` | all valid/all fresh/all ones |
| `0` | no warning/no error/all zero |
| `F` | 4-bit channel mask all set |
| `FF` | 8-bit row mask all set |

Normal per-frame output:

```text
MATRIXFDC_CAP,s=631,t=813405664,q=F,pf=[100.302000,58.098000,56.765000,...,118.495000]
```

Every 5 frames by default:

```text
T5,s0=626,s1=630,n=5,fps=18.42,fa=54281,rw=6120,rmax=13544,ww=6040,rp=28120,i2c=22100,op=6400,prof=13544,pe=0
R5,s0=626,s1=630,n=5,full=80,ar=2,part=0,none=0,it=0,ack=82,b=0,a=82,fb=0,supp=80,uwd=2,dp=0
Q5,s0=626,s1=630,n=5,fw=0,sw=0,tw=0,z0=0,zd=0,nr=0,invRow=0,invDev=0,fs=0,sweepReq=0,sweepQ=0,def=0
I5,s0=626,s1=630,n=5,w=120,r=240,vr=0,retry=0,nack=0,to=0,rec=0,b0=60/120/9000,b1=60/120/9100,avg0=50,avg1=50,maxRead=2300
```

### Backward Compatibility

Relevant defaults:

| Option | Default |
|---|---:|
| `CONFIG_SENSORARRAY_LOG_FORMAT_COMPACT_KV` | `y` |
| `CONFIG_SENSORARRAY_LOG_KEEP_LEGACY_CAP_TAG` | `y` |
| `CONFIG_SENSORARRAY_FDC_CAP_PRINT_DECIMALS` | `6` |
| `CONFIG_SENSORARRAY_LOG_FRAME_OUTPUT_LEGACY` | `n` |
| `CONFIG_SENSORARRAY_FDC_DEVICE_READ4_LOG_LEVEL` | `1` |
| `CONFIG_SENSORARRAY_FDC_ROW_LOG_LEVEL` | `1` |
| `CONFIG_SENSORARRAY_FDC_PROFILE_LOG_EVERY_N_FRAMES` | `5` |
| `CONFIG_SENSORARRAY_FDC_READY_LOG_EVERY_N_FRAMES` | `5` |
| `CONFIG_SENSORARRAY_FDC_TIMING_COMPACT_EVERY_N_FRAMES` | `5` |
| `CONFIG_SENSORARRAY_LOG_FRAME_SUMMARY_EVERY_N_FRAMES` | `5` |
| `CONFIG_SENSORARRAY_FDC_QUALITY_LOG_EVERY_N_FRAMES` | `5` |
| `CONFIG_SENSORARRAY_FDC_I2C_LOG_EVERY_N_FRAMES` | `5` |

To support an older GUI, keep `CONFIG_SENSORARRAY_LOG_KEEP_LEGACY_CAP_TAG=y`; the tag remains `MATRIXFDC_CAP`. To restore old verbose logs, select `CONFIG_SENSORARRAY_LOG_FORMAT_LEGACY=y` and enable `CONFIG_SENSORARRAY_LOG_FRAME_OUTPUT_LEGACY`, `CONFIG_SENSORARRAY_FDC_LOG_READY_EVERY_ROW`, or higher read4/row log levels as needed.

### Runtime Checks

Use these checks in monitor output:

- Strict INTB is active when startup/formal logs show `readyMode=INTB_STRICT_LEVEL`.
- Formal path no longer recovers by `src=FB` unless legacy diagnostic policy is selected.
- After-INTB recheck is only shown as `src=AR` after INTB was confirmed.
- Row-device watchdog default is `rowBudget=6250,mul=10,hard=62500` at 20 fps and 8 rows.
- `P5` explains slow estimates such as `13544 us` using `rc/sc/cd/dc/dg` and per-channel detail in `PR`.
- `P5/R5/T5/Q5/I5` carry `n=5` by default. If Kconfig changes the period, trust the `n` field.
- `OT` gives output `op`; falling `op` should correlate with lower `OV/BN` pressure.
- `MATRIXFDC_CAP` still exists by default and `pf` uses 6 decimals unless `CONFIG_SENSORARRAY_FDC_CAP_PRINT_DECIMALS` is changed.

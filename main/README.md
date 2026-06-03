# main component - 应用层生命周期编排 / Application-layer Lifecycle Orchestration

---

## 中文说明

### 概述

`main/main.c` 是 SensorArray 固件的 **顶层应用编排层**，负责系统初始化、生命周期调度、主循环以及故障处理。

**本层不应承载的责任：**

- FDC 驱动寄存器访问 → 由 `components/fdc2214Cap` 处理
- ADS 驱动寄存器访问 → 由 `components/ads126xAdc` 处理
- TMUX GPIO 底层控制 → 由 `components/tmuxSwitch` 处理
- 矩阵测量策略（扫描计划、救援触发、缓存管理）→ 由 `core/measure` 处理
- 板级 S/D 映射、GPIO 定义、硬件路由语义 → 由 `core/board/sensorarrayBoardMap.c` 处理

### 关键职责 - 应用编排与生命周期

| 功能 | 函数 | 说明 |
|---|---|---|
| **运行时初始化** | `sensorarrayInitRuntime()` | 清空全局上下文，配置运行模式为 FDC_MATRIX |
| **板级与路由初始化** | `sensorarrayInitBoardAndRouting()` | 初始化 `boardSupport`、TMUX GPIO、应用默认路由 |
| **前端初始化** | `sensorarrayInitFrontends()` | 初始化 ADS（ref/VBIAS 关闭）、两颗 FDC2214、矩阵引擎、救援上下文 |
| **Boot 校准** | `sensorarrayRunBootCalibration()` | 运行启动校准扫描（sweep），验证芯片就绪 |
| **主循环** | `sensorarrayRunMainLoop()` | 无限循环：读帧 → 输出 → 救援检查 → 定时 |
| **诊断循环** | `sensorarrayRunDiagnosticTick()` | 故障状态下 1 秒心跳输出诊断信息 |

### 应用初始化流程

**Source**: [main/main.c#app_main](main/main.c)

```c
void app_main(void)
{
  // 1. 系统初始化（三个子步骤）
  esp_err_t initErr = sensorarrayInitSystem(&s_appContext);
  if (initErr != ESP_OK) {
    // 致命错误：初始化失败
    // 输出 APP_FATAL 诊断帧，进入安全空闲（无重启）
    while (true) {
      printf("APP_FATAL,...\n");
      vTaskDelay(pdMS_TO_TICKS(1000u));
    }
  }

  // 2. Boot 校准
  esp_err_t bootErr = sensorarrayRunBootCalibration(&s_appContext);
  if (bootErr != ESP_OK && CONFIG_SENSORARRAY_FDC_BOOT_SWEEP_REQUIRED) {
    // Boot sweep 失败且被要求必须成功
    // → 进入诊断模式或安全空闲
  } else if (bootErr != ESP_OK) {
    // Boot sweep 失败但不要求必须成功
    // → 告警，继续主循环（可启用诊断模式）
  }

  // 3. 进入主循环
  sensorarrayRunMainLoop(&s_appContext);
  // 主循环不返回
}
```

### sensorarrayInitSystem 详解

三阶段初始化顺序很关键，必须按顺序执行：

```c
static esp_err_t sensorarrayInitSystem(sensorarrayAppContext_t *ctx)
{
  // 第 1 阶段：运行时初始化
  esp_err_t err = sensorarrayInitRuntime(ctx);
  if (err != ESP_OK) return err;
  // 结果：全局上下文清空，运行模式设置为 FDC_MATRIX

  // 第 2 阶段：板级与路由初始化
  err = sensorarrayInitBoardAndRouting(ctx);
  if (err != ESP_OK) return err;
  // 结果：boardSupport 初始化，TMUX GPIO 就绪，应用默认路由应用

  // 第 3 阶段：前端初始化
  err = sensorarrayInitFrontends(ctx);
  if (err != ESP_OK) return err;
  // 结果：ADS、FDC primary/secondary 就绪，矩阵引擎就绪

  // 第 4 步：构建默认扫描计划
  sensorarrayBuildDefaultScanPlan(ctx);
  return ESP_OK;
}
```

**各阶段失败处理**

| 阶段 | 失败时 | 操作 |
|---|---|---|
| InitRuntime | 几乎不会失败 | 返回错误代码 |
| InitBoardAndRouting | I2C 初始化失败、TMUX GPIO 失败 | 返回错误代码，进入安全空闲 |
| InitFrontends | ADS 初始化失败、FDC 初始化失败 | 非致命（可单独失败）；但若都失败则无法运行 |
| BuildDefaultScanPlan | 不失败 | 构造扫描计划数据结构 |

### Boot 校准流程

**Source**: `sensorarrayRunBootCalibration()`

```
启动流程：

1. 检查 Primary FDC 是否就绪
   ├─ 否 → 致命错误 (PRIMARY_NOT_READY)
   └─ 是 → 继续

2. 检查 Secondary FDC 是否就绪
   ├─ 否 且 CONFIG_SENSORARRAY_REQUIRE_DUAL_FDC_FOR_BOOT = 1
   │   → 致命错误 (SECONDARY_NOT_READY)
   ├─ 否 但允许单 FDC 运行
   │   → 告警，继续（D5-D8 将标记为无效）
   └─ 是 → 继续

3. 调用 sensorarrayFdcMatrixEngineRunBootSweep()
   ├─ 返回 ESP_OK
   │   → ctx->fdcBootSweepOk = true，诊断模式关闭
   └─ 返回错误
       ├─ 若 CONFIG_SENSORARRAY_FDC_BOOT_SWEEP_REQUIRED = 1
       │   → 致命错误，APP_FATAL
       └─ 若允许不要求
           → 告警，诊断模式关闭（非诊断），继续主循环
```

### 主循环流程

**Source**: `sensorarrayRunMainLoop()`

```c
static void sensorarrayRunMainLoop(sensorarrayAppContext_t *ctx)
{
  while (true) {
    // 1. 检查是否有排队的完整救援请求
    sensorarrayRunQueuedFullSweep(ctx);
    // （可选）触发完整矩阵救援扫描（受冷却时间约束）

    // 2. 检查是否进入诊断模式
    if (ctx->fdcDiagnosticMode || ...) {
      sensorarrayRunDiagnosticTick(ctx);
      // 进入诊断模式，定期输出 MATRIXFDC_DIAG 帧
      // 跳过正常读取
      continue;
    }

    // 3. 记录帧开始时间戳
    int64_t frameStartUs = esp_timer_get_time();

    // 4. 读一帧
    esp_err_t err = sensorarrayRunOneFrame(ctx);
    ctx->fdcFrameCounter++;

    // 5. 检查是否全无效
    bool allInvalid = ctx->frame.capValidMask == 0u;
    if (allInvalid) {
      // 输出诊断帧
      printf("MATRIXFDC_DIAG,stage=all_invalid_frame,...\n");
    } else if (err != ESP_OK) {
      ESP_LOGE("SensorArray", "FRAME_ERROR,...\n");
    }

    // 6. 输出该帧
    (void)sensorarrayFrameOutputPrint(&ctx->frame);

    // 7. 检查是否需要触发救援
    sensorarrayRuntimeRescueTick(ctx);

    // 8. 调整帧周期（定时）
    sensorarrayDelayFramePeriodSince(frameStartUs, ctx->frame.sequence);
  }
}
```

**关键的帧读取函数：sensorarrayRunOneFrame**

```c
static esp_err_t sensorarrayRunOneFrame(sensorarrayAppContext_t *ctx)
{
  // 根据运行模式选择读取实现
  switch (ctx->runtimeMode) {
  case SENSORARRAY_RUNTIME_MODE_FDC_MATRIX:
    return sensorarrayFdcMatrixEngineReadFrame(&ctx->fdcEngine,
                                               &ctx->scanPlan,
                                               &ctx->frame);
  case SENSORARRAY_RUNTIME_MODE_ADS_MATRIX:
    return sensorarrayAdsMatrixEngineReadFrame(&ctx->adsEngine,
                                               &ctx->scanPlan,
                                               &ctx->frame);
  case SENSORARRAY_RUNTIME_MODE_MIXED_ROW:
    return sensorarrayMixedRowEngineReadFrame(&ctx->fdcEngine,
                                              &ctx->adsEngine,
                                              &ctx->scanPlan,
                                              &ctx->frame);
  default:
    return ESP_ERR_INVALID_STATE;
  }
}
```

**分帧周期调整**

```c
static void sensorarrayDelayFramePeriodSince(int64_t frameStartUs, uint32_t sequence)
{
  uint32_t periodMs = sensorarrayFdcFramePeriodMs();
  int64_t periodUs = (int64_t)periodMs * 1000LL;
  int64_t elapsedUs = esp_timer_get_time() - frameStartUs;
  int64_t remainingUs = periodUs - elapsedUs;
  
  if (remainingUs > 0) {
    // 还有时间，sleep 直到期满
    uint32_t delayMs = (uint32_t)((remainingUs + 999LL) / 1000LL);
    vTaskDelay(pdMS_TO_TICKS(delayMs));
  } else {
    // 帧读取超过周期 → 输出 SCAN_TIMING_OVERRUN 诊断
    printf("SCAN_TIMING_OVERRUN,seq=%lu,frameUs=%lld,periodUs=%lld,overrun=1\n", ...);
  }
}
```

### 诊断模式

当初始化失败或 boot sweep 失败（但未致命）时，可能进入诊断模式。

诊断模式行为：

```c
static void sensorarrayRunDiagnosticTick(sensorarrayAppContext_t *ctx)
{
  // 输出诊断帧
  printf("MATRIXFDC_DIAG,stage=diagnostic_mode,bootOk=%d,...\n", ctx->fdcBootSweepOk);

  // （可选）若启用寄存器转储，按周期转储 FDC 寄存器
  if (CONFIG_SENSORARRAY_FDC_DIAG_DUMP_REGS) {
    // 按 CONFIG_SENSORARRAY_FDC_DIAG_DUMP_INTERVAL_MS 周期转储
    esp_err_t dumpErr = sensorarrayFdcSweepDumpAllDeviceRegs(...);
  }

  // 延时 1 秒
  vTaskDelay(pdMS_TO_TICKS(1000u));
}
```

诊断模式下，主循环输出 `MATRIXFDC_DIAG` 帧而不是正常的 `MATRIXFDC_CAP` 帧，便于远程排查。

### 全局上下文结构

```c
typedef struct {
  sensorarrayRuntimeMode_t runtimeMode;        // FDC_MATRIX / ADS_MATRIX / MIXED_ROW
  sensorarrayState_t state;                    // 板状态、FDC/ADS 就绪标志
  sensorarrayScanPlan_t scanPlan;              // 扫描计划（默认 8×8 FDC）
  sensorarrayFrame_t frame;                    // 当前帧输出
  sensorarrayFdcMatrixEngine_t fdcEngine;      // FDC 矩阵引擎状态
  sensorarrayAdsMatrixEngine_t adsEngine;      // ADS 矩阵引擎状态
  sensorarrayFdcRescueContext_t fdcRescue;     // FDC 救援上下文
  
  bool primaryAddrValid;                       // Primary FDC I2C 地址是否有效
  bool secondaryAddrValid;                     // Secondary FDC I2C 地址是否有效
  uint8_t requestedFdcChannels;                // 请求的 FDC 通道数
  bool fdcBootSweepOk;                         // Boot sweep 是否成功
  bool fdcDiagnosticMode;                      // 是否进入诊断模式
  uint32_t fdcFrameCounter;                    // 帧计数器（递增）
  uint32_t failedRescueCount;                  // 连续失败的救援次数
  uint32_t rescueEpoch;                        // 救援纪元编号
  int64_t lastFullRescueTimeUs;                // 上次完整救援时间
  bool rescueRunning;                          // 救援是否正在运行
} sensorarrayAppContext_t;
```

---

## Australian English Documentation

### Overview

`main/main.c` is the **top-level application orchestration layer** of the SensorArray firmware, responsible for system initialisation, lifecycle scheduling, main loop, and failure handling.

**Responsibilities that should NOT be in this layer:**

- FDC driver register access → handled by `components/fdc2214Cap`
- ADS driver register access → handled by `components/ads126xAdc`
- TMUX GPIO low-level control → handled by `components/tmuxSwitch`
- Matrix measurement strategy (scan plan, rescue triggering, cache management) → handled by `core/measure`
- Board-level S/D mapping, GPIO definitions, hardware routing semantics → handled by `core/board/sensorarrayBoardMap.c`

### Key responsibilities

| Phase | Function | Description |
|---|---|---|
| **Runtime initialisation** | `sensorarrayInitRuntime()` | Clear global context, set runtime mode to FDC_MATRIX |
| **Board and routing init** | `sensorarrayInitBoardAndRouting()` | Initialise `boardSupport`, TMUX GPIO, application default routing |
| **Frontend initialisation** | `sensorarrayInitFrontends()` | Initialise ADS (ref/VBIAS off), both FDC2214 devices, matrix engine, rescue context |
| **Scan plan** | `sensorarrayBuildDefaultScanPlan()` | Build default 8×8 FDC matrix scan plan |
| **Boot calibration** | `sensorarrayRunBootCalibration()` | Run startup calibration sweep, verify device readiness |
| **Main loop** | `sensorarrayRunMainLoop()` | Infinite loop: read frame → output → rescue check → timing |
| **Failure handling** | Various error routes | Init failure → safe idle; boot failure → diagnostic or continue per config |

### Application initialisation flow

**Source**: [main/main.c#app_main](main/main.c)

```c
void app_main(void)
{
  // 1. System initialisation (three sub-steps)
  esp_err_t initErr = sensorarrayInitSystem(&s_appContext);
  if (initErr != ESP_OK) {
    // Fatal error: initialisation failed
    // Output APP_FATAL diagnostic frame, enter safe idle (no restart)
    while (true) {
      printf("APP_FATAL,...\n");
      vTaskDelay(pdMS_TO_TICKS(1000u));
    }
  }

  // 2. Boot calibration
  esp_err_t bootErr = sensorarrayRunBootCalibration(&s_appContext);
  if (bootErr != ESP_OK && CONFIG_SENSORARRAY_FDC_BOOT_SWEEP_REQUIRED) {
    // Boot sweep failed and must succeed
    // → Enter diagnostic mode or safe idle
  } else if (bootErr != ESP_OK) {
    // Boot sweep failed but not required
    // → Warning, continue main loop (may enable diagnostic mode)
  }

  // 3. Enter main loop
  sensorarrayRunMainLoop(&s_appContext);
  // Main loop never returns
}
```

### sensorarrayInitSystem explained

Three-phase initialisation order is critical; must execute sequentially:

```c
static esp_err_t sensorarrayInitSystem(sensorarrayAppContext_t *ctx)
{
  // Phase 1: Runtime initialisation
  esp_err_t err = sensorarrayInitRuntime(ctx);
  if (err != ESP_OK) return err;
  // Result: global context cleared, runtime mode set to FDC_MATRIX

  // Phase 2: Board and routing initialisation
  err = sensorarrayInitBoardAndRouting(ctx);
  if (err != ESP_OK) return err;
  // Result: boardSupport initialised, TMUX GPIO ready, default routing applied

  // Phase 3: Frontend initialisation
  err = sensorarrayInitFrontends(ctx);
  if (err != ESP_OK) return err;
  // Result: ADS, FDC primary/secondary ready, matrix engine ready

  // Step 4: Build default scan plan
  sensorarrayBuildDefaultScanPlan(ctx);
  return ESP_OK;
}
```

**Per-phase failure handling**

| Phase | On failure | Action |
|---|---|---|
| InitRuntime | Seldom fails | Return error code |
| InitBoardAndRouting | I2C init failure, TMUX GPIO failure | Return error code, enter safe idle |
| InitFrontends | ADS init failure, FDC init failure | Non-fatal individually; but both failing prevents operation |
| BuildDefaultScanPlan | Never fails | Construct scan plan data structure |

### Boot calibration flow

**Source**: `sensorarrayRunBootCalibration()`

```
Boot sequence:

1. Check if Primary FDC ready
   ├─ No → Fatal error (PRIMARY_NOT_READY)
   └─ Yes → Continue

2. Check if Secondary FDC ready
   ├─ No and CONFIG_SENSORARRAY_REQUIRE_DUAL_FDC_FOR_BOOT = 1
   │   → Fatal error (SECONDARY_NOT_READY)
   ├─ No but single-FDC allowed
   │   → Warning, continue (D5-D8 will be marked invalid)
   └─ Yes → Continue

3. Call sensorarrayFdcMatrixEngineRunBootSweep()
   ├─ Returns ESP_OK
   │   → ctx->fdcBootSweepOk = true, diagnostic mode off
   └─ Returns error
       ├─ If CONFIG_SENSORARRAY_FDC_BOOT_SWEEP_REQUIRED = 1
       │   → Fatal error, APP_FATAL
       └─ If not required
           → Warning, diagnostic mode off (non-diagnostic), continue main loop
```

### Main loop flow

**Source**: `sensorarrayRunMainLoop()`

```c
static void sensorarrayRunMainLoop(sensorarrayAppContext_t *ctx)
{
  while (true) {
    // 1. Check for queued full-rescue request
    sensorarrayRunQueuedFullSweep(ctx);
    // (Optional) trigger full matrix rescue sweep (subject to cooldown)

    // 2. Check if diagnostic mode enabled
    if (ctx->fdcDiagnosticMode || ...) {
      sensorarrayRunDiagnosticTick(ctx);
      // Enter diagnostic mode, periodically output MATRIXFDC_DIAG frames
      // Skip normal reads
      continue;
    }

    // 3. Record frame start timestamp
    int64_t frameStartUs = esp_timer_get_time();

    // 4. Read one frame
    esp_err_t err = sensorarrayRunOneFrame(ctx);
    ctx->fdcFrameCounter++;

    // 5. Check if all-invalid
    bool allInvalid = ctx->frame.capValidMask == 0u;
    if (allInvalid) {
      // Output diagnostic frame
      printf("MATRIXFDC_DIAG,stage=all_invalid_frame,...\n");
    } else if (err != ESP_OK) {
      ESP_LOGE("SensorArray", "FRAME_ERROR,...\n");
    }

    // 6. Output frame
    (void)sensorarrayFrameOutputPrint(&ctx->frame);

    // 7. Check if rescue needed
    sensorarrayRuntimeRescueTick(ctx);

    // 8. Adjust frame period (timing)
    sensorarrayDelayFramePeriodSince(frameStartUs, ctx->frame.sequence);
  }
}
```

**Key frame read function: sensorarrayRunOneFrame**

```c
static esp_err_t sensorarrayRunOneFrame(sensorarrayAppContext_t *ctx)
{
  // Select read implementation based on runtime mode
  switch (ctx->runtimeMode) {
  case SENSORARRAY_RUNTIME_MODE_FDC_MATRIX:
    return sensorarrayFdcMatrixEngineReadFrame(&ctx->fdcEngine,
                                               &ctx->scanPlan,
                                               &ctx->frame);
  case SENSORARRAY_RUNTIME_MODE_ADS_MATRIX:
    return sensorarrayAdsMatrixEngineReadFrame(&ctx->adsEngine,
                                               &ctx->scanPlan,
                                               &ctx->frame);
  case SENSORARRAY_RUNTIME_MODE_MIXED_ROW:
    return sensorarrayMixedRowEngineReadFrame(&ctx->fdcEngine,
                                              &ctx->adsEngine,
                                              &ctx->scanPlan,
                                              &ctx->frame);
  default:
    return ESP_ERR_INVALID_STATE;
  }
}
```

**Frame period adjustment**

```c
static void sensorarrayDelayFramePeriodSince(int64_t frameStartUs, uint32_t sequence)
{
  uint32_t periodMs = sensorarrayFdcFramePeriodMs();
  int64_t periodUs = (int64_t)periodMs * 1000LL;
  int64_t elapsedUs = esp_timer_get_time() - frameStartUs;
  int64_t remainingUs = periodUs - elapsedUs;
  
  if (remainingUs > 0) {
    // Still have time, sleep until period expires
    uint32_t delayMs = (uint32_t)((remainingUs + 999LL) / 1000LL);
    vTaskDelay(pdMS_TO_TICKS(delayMs));
  } else {
    // Frame read exceeded period → output SCAN_TIMING_OVERRUN diagnostic
    printf("SCAN_TIMING_OVERRUN,seq=%lu,frameUs=%lld,periodUs=%lld,overrun=1\n", ...);
  }
}
```

### Diagnostic mode

May be entered when initialisation fails or boot sweep fails (but non-fatal).

Diagnostic mode behaviour:

```c
static void sensorarrayRunDiagnosticTick(sensorarrayAppContext_t *ctx)
{
  // Output diagnostic frame
  printf("MATRIXFDC_DIAG,stage=diagnostic_mode,bootOk=%d,...\n", ctx->fdcBootSweepOk);

  // (Optional) if register dump enabled, dump FDC registers periodically
  if (CONFIG_SENSORARRAY_FDC_DIAG_DUMP_REGS) {
    // Dump per CONFIG_SENSORARRAY_FDC_DIAG_DUMP_INTERVAL_MS period
    esp_err_t dumpErr = sensorarrayFdcSweepDumpAllDeviceRegs(...);
  }

  // Delay 1 second
  vTaskDelay(pdMS_TO_TICKS(1000u));
}
```

In diagnostic mode, main loop outputs `MATRIXFDC_DIAG` frames instead of normal `MATRIXFDC_CAP` frames, facilitating remote troubleshooting.

### Global context structure

```c
typedef struct {
  sensorarrayRuntimeMode_t runtimeMode;        // FDC_MATRIX / ADS_MATRIX / MIXED_ROW
  sensorarrayState_t state;                    // Board state, FDC/ADS readiness flags
  sensorarrayScanPlan_t scanPlan;              // Scan plan (default 8×8 FDC)
  sensorarrayFrame_t frame;                    // Current frame output
  sensorarrayFdcMatrixEngine_t fdcEngine;      // FDC matrix engine state
  sensorarrayAdsMatrixEngine_t adsEngine;      // ADS matrix engine state
  sensorarrayFdcRescueContext_t fdcRescue;     // FDC rescue context
  
  bool primaryAddrValid;                       // Primary FDC I2C address valid?
  bool secondaryAddrValid;                     // Secondary FDC I2C address valid?
  uint8_t requestedFdcChannels;                // Requested FDC channels
  bool fdcBootSweepOk;                         // Boot sweep succeeded?
  bool fdcDiagnosticMode;                      // Entered diagnostic mode?
  uint32_t fdcFrameCounter;                    // Frame counter (increments)
  uint32_t failedRescueCount;                  // Consecutive failed rescue attempts
  uint32_t rescueEpoch;                        // Rescue epoch number
  int64_t lastFullRescueTimeUs;                // Last full rescue time
  bool rescueRunning;                          // Rescue currently running?
} sensorarrayAppContext_t;
```

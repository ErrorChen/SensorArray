# SensorArray ESP32-S3 固件 / SensorArray ESP32-S3 Firmware

**源码版本**: 5c22f780d53991d4a8b220b4d6accaf6eb5d1770  
**分析方法**: 逐函数源码追踪，记录真实执行流程而非规划意图

---

## 中文说明 / Chinese Documentation

### 1. 核心执行流程与生命周期

本项目是基于 **ESP32-S3** 的 SensorArray 固件系统，用于读取 **FDC2214 8×8 电容矩阵**。默认实际运行参数是 **250 ms 帧周期 = 4 fps**（非规划目标的 20 fps）。

#### 应用入口点 (app_main)

```c
void app_main(void) {
    // 清空全局应用上下文
    memset(&s_appContext, 0, sizeof(s_appContext));
    
    // 三阶段初始化
    esp_err_t initErr = sensorarrayInitSystem(&s_appContext);
    if (initErr != ESP_OK) {
        // 致命错误处理 → 安全空闲心跳 (1 秒间隔)
        while(true) {
            printf("APP_FATAL,stage=idle,...\n");
            vTaskDelay(pdMS_TO_TICKS(1000u));
        }
    }
    
    // 启动校准验证
    esp_err_t bootErr = sensorarrayRunBootCalibration(&s_appContext);
    if (bootErr != ESP_OK && CONFIG_SENSORARRAY_FDC_BOOT_SWEEP_REQUIRED) {
        s_appContext.fdcDiagnosticMode = true;
    }
    
    // 进入主循环 (不返回)
    sensorarrayRunMainLoop(&s_appContext);
}
```

#### 三阶段初始化 (sensorarrayInitSystem)

**第一阶段：运行时准备** (sensorarrayInitRuntime)
```c
清空 ctx 结构
设置 runtimeMode = SENSORARRAY_RUNTIME_MODE_FDC_MATRIX  // 默认
配置 FDC 通道数 (来自 CONFIG_FDC2214CAP_CHANNELS)
解析 I2C 地址：
  • 主地址：CONFIG_SENSORARRAY_FDC_PRIMARY_I2C_ADDR   (0x2B)
  • 副地址：CONFIG_SENSORARRAY_FDC_SECONDARY_I2C_ADDR (0x2A)
```

**第二阶段：硬件资源初始化** (sensorarrayInitBoardAndRouting)
```c
boardSupportInit()          // I2C Bus 0 & 1 配置 (400 kHz)
tmuxSwitchInit()            // GPIO 初始化：S0-S2 (行选), SELA, SELB, EN, SW
sensorarrayBoardMapAudit()  // 验证 D1-D8 到芯片的映射
sensorarrayApplyTmuxDefaults() // 安全默认路由
  ├─ 行选择 → S0
  ├─ SW 源 → GND
  ├─ SELA → ADS (临时)
  ├─ SELB → false
  └─ EN → true
```

**第三阶段：前端初始化** (sensorarrayInitFrontends)
```c
sensorarrayBringupInitAds()     // ADS1263 初始化 (ref OFF, VBIAS OFF)
初始化主 FDC (0x2B, D1-D4)
初始化副 FDC (0x2A, D5-D8)
sensorarrayFdcMatrixEngineInit()
sensorarrayFdcRescueReset()
```

#### 启动校准 (sensorarrayRunBootCalibration)

```c
检查点 1: 主 FDC (0x2B) 必须就绪
  if(!ready) → FATAL_ERROR

检查点 2: 副 FDC (0x2A) 可选
  if(CONFIG_SENSORARRAY_REQUIRE_DUAL_FDC_FOR_BOOT && !ready) 
    → FATAL_ERROR
  else if(!ready)
    → PRIMARY_ONLY 模式允许继续

步骤: 运行启动扫描
  sensorarrayFdcMatrixEngineRunBootSweep()
  → 设置 fdcBootSweepOk 标志
  → 失败时设置 fdcDiagnosticMode = true
```

#### 主循环 (sensorarrayRunMainLoop)

```c
while(true) {
    // 步骤 1: 队列检查
    sensorarrayRunQueuedFullSweep(&ctx)     // 异步救援请求处理
    
    // 步骤 2: 诊断输出 (每 100 帧)
    if((fdcFrameCounter % 100u) == 0u) {
        printf("APP_STACK,freeWords=%lu\n", uxTaskGetStackHighWaterMark(NULL));
    }
    
    // 步骤 3: 诊断模式检查
    if(fdcDiagnosticMode || sensorarrayFdcMatrixEngineDiagnosticMode(...)) {
        sensorarrayRunDiagnosticTick(&ctx);  // 1 秒心跳输出诊断
        continue;  // 跳过帧读取
    }
    
    // 步骤 4: 读取一帧
    int64_t frameStartUs = esp_timer_get_time();
    esp_err_t err = sensorarrayRunOneFrame(&ctx);  // 分发到具体引擎
    ctx->fdcFrameCounter++;
    
    // 步骤 5: 全无效检查
    if(ctx->frame.capValidMask == 0u) {
        printf("MATRIXFDC_DIAG,stage=all_invalid_frame,seq=%lu\n", 
               ctx->frame.sequence);
    }
    
    // 步骤 6: 输出帧数据 (printf 文本格式)
    sensorarrayFrameOutputPrint(&ctx->frame);
    
    // 步骤 7: 行级救援检查
    sensorarrayRuntimeRescueTick(&ctx);
    
    // 步骤 8: 帧周期控制
    sensorarrayDelayFramePeriodSince(frameStartUs, ctx->frame.sequence);
}
```

**关键参数**:
- 实际帧周期：**250 ms** (4 fps) 不是目标的 20 fps
- 诊断模式：由 failedRescueCount ≥ 3 触发，1 秒心跳循环
- 启动强制：主 FDC 必须就绪，副 FDC 可选（取决于配置）
- 并行读取：两条独立 I2C 总线（I2C0 + I2C1）时自动启用
- 救援冷却：5 秒内最多一次完整扫描


### 2. 核心帧读取实现 (sensorarrayMeasureReadFdcMatrixFrame)

核心帧读取函数每 250ms 被调用一次，执行完整的 8×8 矩阵扫描。

#### 入口与初始化

```c
esp_err_t sensorarrayMeasureReadFdcMatrixFrame(sensorarrayState_t *state,
                                               sensorarrayFdcMatrixFrame_t *outFrame) {
    // 初始化输出帧为空
    sensorarrayMeasureInitFdcMatrixFrame(outFrame);
    
    // 获取全局互斥锁 (线程安全)
    esp_err_t err = sensorarrayMeasureTakeLock();
    if (err != ESP_OK) return err;
    
    // 检查矩阵是否就绪
    esp_err_t firstErr = sensorarrayMeasureCheckFdcMatrixReady(state);
    if (firstErr != ESP_OK) {
        sensorarrayMeasureGiveLock();
        return firstErr;
    }
```

#### 并行 vs 串行模式决策

根据硬件配置自动选择：
- **并行模式**: 两条独立 I2C 总线（I2C0 + I2C1），主线程读副 FDC 同时异步读主 FDC
- **串行模式**: 同一总线或配置禁用，顺序读主、读副

```c
bool parallelEligible = 
    CONFIG_SENSORARRAY_FDC_PARALLEL_DUAL_BUS_READ &&  // 配置启用
    primaryBusEnabled && secondaryBusEnabled &&        // 两条总线就绪
    !sameBus &&                                         // 总线不同
    s_fdcWorkersAvailable;                              // 工作队列可用

if (parallelEligible) {
    // 异步启动主 FDC 读任务
    // 主线程同步读副 FDC
    // 等待主任务完成 (25ms 超时)
} else {
    // 顺序：主 FDC, 再副 FDC
}
```

#### 行迭代读取

```c
for (uint8_t s = 1u; s <= 8u; ++s) {  // 8 行
    // 行前准备
    sensorarrayFdcReadRowDeviceSamples(...primary...);
    sensorarrayFdcReadRowDeviceSamples(...secondary...);
    
    // 数据验证和合并
    // 检查有效掩码, 警告掩码, 错误掩码
    // 计算电容: C = 1/((2πf)² × L)
}
```

#### 帧聚合与输出

```c
// 帧级汇总
outFrame->sequence = ++s_fdcMatrixSequence;
outFrame->timestampUs = frameStartUs;

// 检查全无效帧
if (outFrame->capValidMask == 0u) {
    // 输出诊断帧并触发救援逻辑
}

// 释放互斥锁
sensorarrayMeasureGiveLock();
return outFrame->capValidMask != 0u ? ESP_OK : ESP_ERR_INVALID_RESPONSE;
```

### 3. 硬件架构与矩阵映射

**核心微控制器与芯片**

```
┌─────────────────────────────────────────────────────────────┐
│ ESP32-S3 (MCU)                                              │
├─────────────────────────────────────────────────────────────┤
│  I2C Bus 0 (400 kHz)                                        │
│    └─→ FDC2214 (主, 0x2B, D1-D4 通道)                       │
│  I2C Bus 1 (400 kHz, 可选并行)                              │
│    └─→ FDC2214 (副, 0x2A, D5-D8 通道)                       │
│  SPI Bus                                                     │
│    └─→ ADS1262/ADS1263 (电压/电阻/压电)                      │
│  GPIO                                                        │
│    ├─→ TMUX1108 (行选 S1-S8)                               │
│    ├─→ TMUX1134 (前端切换 / ADS ref/VBIAS)                  │
│    └─→ INTB 中断 (GPIO 17/18)                              │
└─────────────────────────────────────────────────────────────┘
```

**矩阵映射与 D-line 定义**

```
行（Row）← TMUX1108 选择 → S1 ... S8
列（Column）← 传感器阵列 → D1 D2 D3 D4 D5 D6 D7 D8

D-line 到芯片/通道映射：
  D1,D2,D3,D4  →  主 FDC2214 (I2C 0x2B)   CH0-CH3
  D5,D6,D7,D8  →  副 FDC2214 (I2C 0x2A)   CH0-CH3

矩阵帧索引公式（行优先）：
  index = (sIndex - 1) × 8 + (dIndex - 1)
  示例：S1D1=0, S1D2=1, ..., S1D8=7, S2D1=8, ..., S8D8=63
```

**FDC 读取时的硬件状态要求**

在每个 FDC 矩阵帧读取前，`sensorarrayMeasurePrepareFdcMatrixPath()` 必须强制执行：

```
✓ SW 源 → GND
✓ ADS1263 内部参考关闭
✓ ADS1263 VBIAS 关闭
✓ ADS 转换停止
✓ SELA 路由到 FDC2214
✓ SELB 按板级策略设置
```

### 4. 架构边界与组件职责

**分层设计原则：芯片驱动无业务逻辑，业务层无底层细节**

| 层级 | 模块路径 | 职责 | 不负责 |
|---|---|---|---|
| **应用层** | `main/main.c` | 初始化编排、生命周期调度、主循环 | 测量策略、板级路由 |
| **板级映射** | `core/board/sensorarrayBoardMap.c` | S/D 映射、GPIO 定义 | 通用测量算法 |
| **测量层** | `core/measure/{fdc,ads}/` | 矩阵读取策略、救援逻辑 | 芯片寄存器 |
| **驱动层** | `components/{fdc2214Cap,ads126xAdc,tmuxSwitch}/` | I2C/SPI 事务、GPIO 原语 | 板级路由 |
| **资源层** | `boardSupport/` | I2C 总线、GPIO 保护 | 业务策略 |

### 5. 出帧格式

**默认输出：总 LC 坦克电容（pF）**

```
MATRIXFDC_CAP,seq=12,ts=345678901,validMask=0xFFFF...,capValidMask=0xFFFF...,d1=123.45,d2=124.56,...,d64=122.34
```

**帧字段说明**

| 字段 | 说明 |
|---|---|
| `seq` | 帧序列号（单调递增） |
| `ts` | 微秒时间戳 |
| `validMask` | 64 位有效掩码 |
| `capValidMask` | 64 位电容有效掩码 |
| `d1-d64` | 每个单元格的电容值 (pF) 或 -1 (无效) |

**电容计算公式**

```
f (Hz) ← FDC2214 测得频率
L = CONFIG_SENSORARRAY_FDC_TANK_INDUCTOR_NH (默认 18000 nH)

C = 1 / ((2π × f)² × L) [单位: 法拉]
  = 1e12 / ((2π × f)² × L×1e-9) [单位: pF]

示例：f = 500 kHz, L = 18000 nH → C ≈ 56.2 pF
```

**诊断输出（全无效或故障）**

```
MATRIXFDC_DIAG,stage=all_invalid_frame,seq=11,errorMask=0xFFFF...,bootOk=1,failedCount=0
```

### 6. 重要限制与已知约束

- **默认 FPS 低于目标**: 250 ms = 4 fps (目标 20 fps)，原因是 I2C 吞吐量限制
- **INTB 仅为提示**: 不能用作同步机制，必须检查 STATUS.DRDY
- **副 FDC 故障隔离**: 副 FDC 故障时，D5-D8 标记为 -1，主 FDC 数据不受影响
- **ADS 与 FDC 互斥**: 同一时刻只能读一种
- **诊断模式不可逆**: 需要重启才能恢复

### 7. 配置参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| CONFIG_SENSORARRAY_FDC_MATRIX_PERIOD_MS | 250 | 实际帧周期，ms |
| CONFIG_SENSORARRAY_FDC_PRIMARY_I2C_ADDR | 0x2B | 主 FDC |
| CONFIG_SENSORARRAY_FDC_SECONDARY_I2C_ADDR | 0x2A | 副 FDC |
| CONFIG_FDC2214CAP_CHANNELS | 4 | Autoscan 通道数 |
| CONFIG_SENSORARRAY_FDC_TANK_INDUCTOR_NH | 18000 | 谐振电感, nH |
| CONFIG_SENSORARRAY_FDC_PARALLEL_DUAL_BUS_READ | 1 | 启用并行读 |
| CONFIG_SENSORARRAY_FDC_BOOT_SWEEP_REQUIRED | 1 | 启动验证强制 |
| CONFIG_SENSORARRAY_REQUIRE_DUAL_FDC_FOR_BOOT | 1 | 双 FDC 强制 |

### 8. 故障排查

**场景 1：启动时主 FDC 不就绪**
- 现象: APP_FATAL,stage=init
- 排查: 验证 I2C0 硬件连接，检查 FDC 地址配置

**场景 2：连续全无效帧**
- 现象: MATRIXFDC_DIAG,stage=all_invalid_frame
- 排查: 检查 FDC_PATH 诊断（SW, SELA, SELB 状态），查看 I2C 总线状态

**场景 3：并行读取未启用**
- 现象: FDC_PARALLEL_CFG,enabled=0,reason=...
- 排查: 检查 CONFIG_SENSORARRAY_FDC_PARALLEL_DUAL_BUS_READ，确认两条 I2C 总线配置

## Australian English Documentation / 澳洲英文说明

### 1. Project Overview

This project is an **ESP32-S3-based** SensorArray firmware system designed to build high-precision multi-parameter sensing matrices. The current production path is **FDC2214 8×8 capacitance matrix readout**, targeting stable **20 fps** output.

**Current supported readout modes:**

| Path | Status | Description |
|---|---|---|
| FDC2214 8×8 capacitance matrix | ✓ Production-ready | Dual FDC2214 via parallel I2C/I2C1; TMUX1108 row selection; TMUX1134 front-end switching |
| ADS1262/ADS1263 voltage/resistance/piezo | ✓ Architecture support | Protocol framework ready; clear boundary maintained with FDC path |
| Mixed row mode | ⏳ Planned | Architecture support pending completion |
| High-speed binary transport | ⏳ Future extension | Text printf output only by default |

**Key performance metrics:**

- Default frame period: 250 ms (5 fps), configurable to 100 ms (10 fps) target
- Long-term goal: 100 fps (requires I2C optimisation and parallel ADS support)
- Invalid cell handling: `-1` sentinel marking
- I2C is primary performance and stability constraint

### 2. Hardware Model

**Core MCU and device topology**

```
┌─────────────────────────────────────────────────────────────┐
│ ESP32-S3 (MCU)                                              │
├─────────────────────────────────────────────────────────────┤
│  I2C Bus 0 (400 kHz)                                        │
│    ├─→ FDC2214 (Primary, 0x2B, D1-D4 Channels)            │
│    └─→ (Secondary fallback)                                │
│  I2C Bus 1 (400 kHz, optional parallel)                     │
│    └─→ FDC2214 (Secondary, 0x2A, D5-D8 Channels)          │
│  SPI Bus                                                     │
│    └─→ ADS1262/ADS1263 (voltage/resistance/piezo channels) │
│  GPIO                                                        │
│    ├─→ TMUX1108 (row/source select S1-S8)                 │
│    ├─→ TMUX1134 (front-end switching / ADS ref/VBIAS ctrl) │
│    └─→ INTB lines (FDC2214 data-ready hints)              │
└─────────────────────────────────────────────────────────────┘
```

**Matrix mapping and D-line definition**

```
Rows (Row) ← TMUX1108 selection → S1 ... S8
Columns ← Sensor array → D1 D2 D3 D4 D5 D6 D7 D8

D-line to device/channel mapping:
  D1,D2,D3,D4  →  Primary FDC2214 (I2C 0x2B)   CH0-CH3
  D5,D6,D7,D8  →  Secondary FDC2214 (I2C 0x2A) CH0-CH3

Frame index formula (row-major):
  index = (sIndex - 1) × 8 + (dIndex - 1)
  
  Example: S1D1=0, S1D2=1, ..., S1D8=7, S2D1=8, ..., S8D8=63
```

**Critical analogue routing signals**

| Signal | Purpose | Safe State |
|---|---|---|
| **SW** | Source switching (GND / ADS VBIAS) | GND (during FDC readout) |
| **SELA** | FDC/ADS front-end select | FDC-routed during FDC mode |
| **SELB** | ADS internal reference control | Off (`false`) |
| **EN** | TMUX1134 enable | Always `true` |
| **INTB1/INTB2** | FDC interrupt lines (GPIO 17/18) | Interrupt or fallback polling |

**FDC readout hardware state requirements**

Before each FDC matrix frame read, `sensorarrayMeasurePrepareFdcMatrixPath()` must enforce:

```
✓ SW source → GND
✓ ADS1263 internal reference OFF
✓ ADS1263 VBIAS OFF
✓ ADS conversion stopped
✓ SELA routed to FDC2214 (via board-level GPIO mapping)
✓ SELB configured per board FDC policy
```

Otherwise, front-ends may interfere, causing data invalidity or channel crosstalk.

### 3. Architecture Boundaries and Component Responsibilities

**Layering principle: Chip drivers are business-agnostic; business layer is driver-agnostic**

| Layer | Module Path | Responsibility | Not Responsible For |
|---|---|---|---|
| **Application** | `main/main.c` | Initialisation orchestration, lifecycle scheduling, main loop, safe idle | Measurement strategy, board routing |
| **Board Mapping** | `core/board/sensorarrayBoardMap.c` | S/D to device channel mapping, GPIO definitions, hardware semantics | Generic measurement algorithm |
| **Measurement** | `core/measure/{fdc,ads}/` | Matrix readout strategy, rescue logic, cache management, frame output | Chip register access |
| **Driver** | `components/{fdc2214Cap,ads126xAdc,tmuxSwitch}/` | Chip registers, I2C/SPI transactions, GPIO primitives | Board routing, matrix concepts |
| **Resource** | `boardSupport/` | I2C bus initialisation, GPIO mapping, lock protection | Business logic |

**Key component notes**

- **`components/fdc2214Cap/`**: Pure chip driver; knows only FDC2214 registers and I2C transactions; unaware of rows, D-lines, TMUX routing, or rescue policy.

- **`components/ads126xAdc/`**: Pure chip driver; knows only ADS registers and SPI transactions; unaware of D-line mapping, analogue routing, or frame format.

- **`components/tmuxSwitch/`**: Pure GPIO abstraction; knows only how to toggle TMUX pins; unaware of which S/D combination maps to what business meaning.

- **`core/measure/fdc/`**: FDC matrix strategy implementation; knows scan plans, TMUX switching timing, INTB waiting, cache application, rescue triggering. Calls `fdc2214Cap` driver and queries `core/board` for GPIO mappings.

- **`core/board/`**: Single source of truth for board mapping; all D-line to device/channel, I2C addresses, GPIO pins, TMUX semantics centralised here. **When modifying the board, prioritise changes to this module; avoid changing drivers or measurement algorithms.**

- **`main/`**: Thin application orchestration; initialisation and scheduling only; does not carry heavy business logic.

### 4. Lifecycle call chain

Full initialisation → Boot calibration → Main loop → Frame output/rescue.

**Source**: [main/main.c](main/main.c)

```
app_main()
  │
  ├─→ sensorarrayInitSystem()
  │     ├─→ sensorarrayInitRuntime()           [Clear context, configure runtime mode]
  │     ├─→ sensorarrayInitBoardAndRouting()   [Initialise board support, TMUX GPIO]
  │     └─→ sensorarrayInitFrontends()         [Initialise ADS, FDC primary/secondary]
  │           ├─→ sensorarrayBringupInitAds()
  │           ├─→ sensorarrayBringupInitFdcDevice() × 2
  │           ├─→ sensorarrayFdcMatrixEngineInit()
  │           └─→ sensorarrayFdcRescueReset()
  │
  ├─→ sensorarrayBuildDefaultScanPlan()        [Create 8×8 FDC scan plan]
  │
  ├─→ sensorarrayRunBootCalibration()          [Start boot calibration sweep]
  │     └─→ sensorarrayFdcMatrixEngineRunBootSweep()
  │           └─→ sensorarrayFdcSweepRunBoot()
  │
  └─→ sensorarrayRunMainLoop()                 [Enter infinite main loop]
        ├─→ sensorarrayRunOneFrame()
        │     └─→ sensorarrayFdcMatrixEngineReadFrame()
        │           └─→ sensorarrayMeasureReadFdcMatrixFrame()
        │                 ├─→ sensorarrayMeasurePrepareFdcMatrixPath()
        │                 ├─→ [Row loop] row select, TMUX switch
        │                 ├─→ [In-row loop] INTB wait, FDC read
        │                 └─→ sensorarrayFrameBuilder*()
        ├─→ sensorarrayFrameOutputPrint()       [Output frame text]
        ├─→ sensorarrayRuntimeRescueTick()      [Evaluate rescue necessity]
        └─→ sensorarrayDelayFramePeriodSince()  [Adjust frame period]
```

**Startup failure handling**

```
Initialisation failure → APP_FATAL → Safe idle (no restart)
Boot Sweep failure
  ├─ If CONFIG_SENSORARRAY_FDC_BOOT_SWEEP_REQUIRED = 1
  │   → APP_FATAL → Diagnostic mode or safe idle
  └─ If CONFIG_SENSORARRAY_FDC_BOOT_SWEEP_REQUIRED = 0
      → Warning → Continue main loop (but enable diagnostic mode)

Runtime read failure → Frame output with errorMask / Trigger rescue
```

### 5. FDC Row Readout State Machine (Row Epoch)

Each row capacitance matrix readout follows a strict analogue routing and timing sequence.

**Source**: Reference `core/measure/fdc/sensorarrayFdcMatrix.c`

```
State transitions:

  [SleepFdc]
    ↓ Send sleep command to both FDC devices
    ↓ (Wait for FDC to stop conversion, release I2C)
  
  [SwitchRow]
    ↓ TMUX1108 switches to new row (S1-S8)
    ↓ 
  
  [SettleAnalogue]
    ↓ Wait CONFIG_SENSORARRAY_FDC_MATRIX_SETTLE_US (default 1000 µs)
    ↓ (Analogue path stabilises, no crosstalk)
  
  [ApplyDiffCache]
    ↓ If CONFIG_SENSORARRAY_FDC_DIFF_CACHE_APPLY enabled
    ↓ Apply per-cell parameter difference cache for this row
    ↓ (Calibration compensation between boot row and current row)
  
  [WakeFdc]
    ↓ Send wake/restart conversion to both FDC devices
    ↓ 
  
  [WaitReady]
    ↓ Option 1: INTB interrupt arrives (GPIO 17/18 edge-triggered)
    ↓ Option 2: Polling timeout STATUS.DRDY (CONFIG_SENSORARRAY_FDC_INTB_WAIT_TIMEOUT_US)
    ↓ 
    ├─ INTB is only a "data-ready" hint, not a completeness guarantee
    └─ Must check STATUS.DRDY and return data
  
  [ReadPrimary]
    ↓ I2C read Primary FDC (D1-D4 CH0-CH3, 4 raw values)
  
  [ReadSecondary]
    ├─ Optional: Parallel task reads Secondary FDC (D5-D8 CH0-CH3)
    └─ Or: Wait, then read serially
  
  [MergeRow]
    ↓ Merge Primary + Secondary raw values → 8 cells
    ↓ Check DRDY/error flags, mark invalid cells as -1
  
  [BuildFrame]
    ↓ If all 64 cells of frame readout complete
    ↓ Trigger frequency conversion, capacitance calculation, mask building
    ↓ Output sensorarrayFdcMatrixFrame_t
```

**Important reminders**

- **INTB is a hint, not a guarantee**: INTB edge means "may have data," but FDC may not have fully converted or state may be invalid.
- **Must check STATUS**: After each read, check STATUS.DRDY and error flags.
- **Invalid values use -1 sentinel**: When any cell fails, record `capTotalPf[i] = -1.0`, clear `capValidMask` bit i, set `errorMask` bit i.
- **All-invalid frames still emit output**: Even if all 64 cells are `-1`, firmware sends a `MATRIXFDC_DIAG` diagnostic frame instead of silent failure.

### 6. Output frame format

**Default output: Total LC tank capacitance (pF)**

```
MATRIXFDC_CAP,seq=12,timestampUs=345678901,partial=0,frameQuality=full,capValidMask=0xFFFFFFFFFFFFFFFF,freshMask=0xFFFFFFFFFFFFFFFF,warnMask=0x0000000000000000,errorMask=0x0000000000000000,invalidSentinel=-1.000000,capTotalPf=[18.692341,18.693266,19.341023,...,20.156432]
```

**Frame field explanation**

| Field | Type | Description |
|---|---|---|
| `seq` | u32 | Frame sequence number (increments by 1 per frame) |
| `timestampUs` | i64 | Frame start timestamp (`esp_timer_get_time()`) |
| `partial` | bool | 0 = full 64 cells, 1 = partial row read |
| `frameQuality` | enum | `full` = no errors, `partial` = some failures |
| `capValidMask` | u64 | 64-bit; bit i = 1 means `capTotalPf[i]` valid |
| `freshMask` | u64 | 64-bit; bit i = 1 means cell freshly read this frame (not cached) |
| `warnMask` | u64 | 64-bit; bit i = 1 means amplitude warning (non-blocking) |
| `errorMask` | u64 | 64-bit; bit i = 1 means complete cell failure / timeout / invalid status |
| `invalidSentinel` | f64 | Invalid cell value (fixed -1.0) |
| `capTotalPf[64]` | f64×64 | Total equivalent LC tank capacitance (pF); -1.0 for invalid |

**Capacitance calculation formula**

```
f (Hz) ← FDC2214 measured frequency
L = CONFIG_SENSORARRAY_FDC_TANK_INDUCTOR_NH (default 18000 nH)

C = 1 / ((2π × f)² × L) [unit: Farad]
  = 1e12 / ((2π × f)² × L×1e-9) [unit: pF]

Example: f = 500 kHz, L = 18000 nH
  C ≈ 56.2 pF (total tank including sensor, inductor, PCB parasitic, TMUX parasitic, etc.)
```

**Optional output: Frequency (Hz)**

Only when `SENSORARRAY_FDC_TEXT_OUTPUT_FREQ_HZ` or `SENSORARRAY_FDC_TEXT_OUTPUT_BOTH_SEPARATE` configured:

```
MATRIXFDC_FREQ,seq=12,timestampUs=345678901,validMask=0xFFFFFFFFFFFFFFFF,warnMask=0x0000000000000000,errorMask=0x0000000000000000,freqHz=[500234,500456,...,501023]
```

**Debug output: Raw 28-bit codes**

```
DEBUGFDC_RAW,seq=12,timestampUs=345678901,raw28=[9215955,9215731,9217834,...,9218012]
```

**Diagnostic output (all-invalid or failure)**

```
MATRIXFDC_DIAG,stage=all_invalid_frame,seq=11,errorMask=0xFFFFFFFFFFFFFFFF,readErr=0x109,bootOk=1,freshCount=0,hardwareZeroRawCount=64,invalidSentinelCount=64,rawAllZero=1
```

**Important**: Host tools **must** treat `-1.000000` as invalid sentinel, **not** as a physical capacitance value. Must exclude from heatmap autoscale, statistics, filtering, trend calculations.

### 7. Build, flash and monitor

**Windows PowerShell environment setup (recommended)**

Create PowerShell profile `~\Documents\PowerShell\profile.ps1` or temporary session script:

```powershell
# Configure ESP-IDF paths
$env:IDF_PATH = "C:\Espressif\frameworks\esp-idf-v5.5.1"
$env:IDF_PYTHON_ENV_PATH = "C:\Espressif\python_env\idf5.5_py3.11_env"

# Add tool paths to PATH
$pythonDir = "$env:IDF_PYTHON_ENV_PATH\Scripts"
$cmakeDir = "C:\Espressif\tools\cmake\3.30.2\bin"
$ninjaDir = "C:\Espressif\tools\ninja\1.12.1"
$gccDir = "C:\Espressif\tools\xtensa-esp-elf\esp-14.2.0_20241119\xtensa-esp-elf\bin"
$idfTools = "$env:IDF_PATH\tools"

$env:Path = "$pythonDir;$cmakeDir;$ninjaDir;$gccDir;$idfTools;$env:Path"

# Define convenience function
function idf {
    & "$pythonDir\python.exe" "$env:IDF_PATH\tools\idf.py" @args
}

# Verify toolchain
Write-Host "ESP-IDF Tools Verification:"
idf --version
python --version
cmake --version
ninja --version
xtensa-esp32s3-elf-gcc --version
```

**Build and flash workflow**

```powershell
# Enter project directory
cd C:\ESP32\SensorArray

# Clean old build
idf fullclean

# Configure (optional)
idf menuconfig
# - Adjust flash size, I2C frequency, TMUX GPIO, ADS options if needed

# Build
idf build

# List available COM ports
Get-SerialPort  # Windows 11+ or use `Get-CimInstance Win32_SerialPort`

# Flash + monitor (single command)
idf -p COMx flash monitor
# Where COMx = COM3 or COM4 etc.

# Or step-by-step
idf -p COMx erase-flash          # Erase entire flash
idf -p COMx flash                # Write bootloader/partition/app
idf -p COMx monitor              # Start monitoring
```

**First flash or diagnostics**

New board or known flash issues:

```powershell
# 1. Erase entire flash
idf -p COMx erase-flash

# 2. Build and flash from scratch
idf -p COMx flash monitor

# 3. Check startup logs, confirm app_main entry
```

**Flash address and content verification**

Normal flash contents:

```
0x00000:     Bootloader (2 MB / 4 MB depending on config)
0x08000:     Partition table
0x10000:     App image
```

Verify correct programming (using esptool):

```powershell
python -m esptool --chip esp32s3 -p COMx read_flash 0x0 0x100 flash_0x0_dump.bin

# View hex dump
[System.IO.File]::ReadAllBytes("flash_0x0_dump.bin") | 
  % { [Convert]::ToString($_, 16).PadLeft(2, '0') } | 
  Join-String -Separator " " -OutputPrefix "0x00: "

# Bootloader normally starts with 0xE9
# App also starts with 0xE9
```

If all `FF`, flash not programmed correctly or connection issue.

### 8. Console commands and diagnostics

**Available commands overview**

Run in serial monitor (`idf -p COMx monitor`):

| Command | Format | Description |
|---|---|---|
| `fdc_diag` | `fdc_diag` | Dump current FDC register values; useful for anomaly diagnostics or no-oscillation |
| `force_full_sweep` | `force_full_sweep` | Trigger full matrix rescue sweep immediately; subject to cooldown |
| `fdc_boot_sweep` | `fdc_boot_sweep` | Re-run boot calibration procedure (debug) |
| `fdc_rescue` | `fdc_rescue` | Manually trigger FDC rescue |
| `fdc_period_ms <N>` | To be confirmed | Set frame period (milliseconds) |
| `fdc_profile` | To be confirmed | Output frame readout timing analysis |
| `fdc_i2c_trace` | To be confirmed | Enable I2C transaction logging (diagnostics only; high performance impact) |
| `fdc_discard_frames <N>` | To be confirmed | Discard next N frames (for stabilisation) |

**Diagnostic workflow**

```
Symptom: All-invalid matrix (-1)
  → Run `fdc_diag` to check register state
  → Inspect CONFIG/STATUS fields
  → Verify INTB GPIO wiring (GPIO 17/18)
  → Try `force_full_sweep` to trigger rescue

Symptom: Specific row/column anomalies
  → Run `fdc_diag` and compare FDC settings for that row
  → Check SELA/SELB/SW routing correctness
  → Review board map (core/board/sensorarrayBoardMap.c)

Symptom: Intermittent timeouts
  → Enable `fdc_i2c_trace` (diagnostics only, not recommended long-term)
  → Observe I2C transaction log for NACKs or timeouts
  → Check I2C clock frequency, pull-up resistors
  → Or run `force_full_sweep` to attempt recovery
```

### 9. Known constraints and limitations

**Performance constraints**

- **Default target**: Stable **20 fps** (frame period 50 ms)
- **Long-term goal**: 100 fps (frame period 10 ms), requires:
  - True hardware-parallel dual-I2C readout (currently TMUX row-synchronised)
  - I2C frequency increase (currently 400 kHz)
  - ADS parallel conversion or fast-speed binary transport
- **Current primary bottleneck**: I2C transaction latency and TMUX row-switching timing

**I2C reliability**

- Primary FDC (I2C 0x2B) and Secondary FDC (I2C 0x2A) can use separate buses (I2C/I2C1) for parallel operation
- If both devices share one bus, reads are serial
- I2C timeouts and NACKs handled by `boardSupport`; failures don't cause restart; enter diagnostic mode
- Long-term I2C recovery mechanism implemented in `boardSupport` (register-level soft reset)

**Flash size caution**

⚠️ **To be confirmed**: Current hardware detected as 16 MB flash, but sdkconfig still configured as 2 MB.
  - Does not affect app image load (app_main runs normally)
  - But OTA/NVS/partition table calculations may be incorrect
  - **Recommendation**: Future board-level configuration adjust sdkconfig to 16 MB

**Output and transport**

- **Default**: Text printf output, no special binary framing needed
- **Binary mode**: Currently disabled (`sensorarrayFastSpeedIsEnabled()` returns false)
- **Future**: Binary fast-speed mode enabled by explicit host command

**FDC and ADS mutual exclusion**

- FDC readout path requires ADS ref/VBIAS off, SW source → GND
- ADS readout path (future) requires SELA select ADS, FDC conversion stopped
- Current firmware only implements FDC production path; mixed mode (Mixed Row) awaits architecture confirmation

**Startup sequence**

- Boot sweep must succeed (configurable via `CONFIG_SENSORARRAY_FDC_BOOT_SWEEP_REQUIRED`)
- On failure: if allowed, enable diagnostic mode; if required, enter safe idle
- Diagnostic mode periodically outputs `MATRIXFDC_DIAG` frames for remote troubleshooting

---

## 反馈与改进 / Feedback and Improvement

如遇到 README 中不符合源码的说明，或有 `待确认 / To be confirmed` 标记的内容，请提交反馈。

If you find descriptions in this README that don't align with the source code, or have content marked as `待确认 / To be confirmed`, please provide feedback.

Startup performs a visible boot full sweep before normal matrix output. The sweep is row/device based:

- select one S row once
- apply a candidate deglitch/drive/high-current setting to the primary FDC and secondary FDC
- run both FDC2214 devices in CH0-CH3 autoscan
- read primary CH0-CH3 for D1-D4 and secondary CH0-CH3 for D5-D8
- cache the best result per cell from those row reads

Boot sweep failures no longer permanently block normal matrix output. If no valid oscillation is found during boot, the firmware restores CH0-CH3 autoscan and enters the normal matrix loop in degraded mode. Only a path/device failure that prevents both FDC devices from being usable is treated as fatal.

At runtime, rescue is also row based. A pending cell rescue triggers a fast sweep of that cell's containing row, not repeated single-cell route/lock/sweep cycles. Full rescue sweeps all rows. Amplitude warnings enter `warnMask` first and remain usable when raw28 is non-zero, not saturated, watchdog-free, and converts to a plausible frequency. The runtime path classifies amplitude warnings as fresh, stale, or transient. Only persistent fresh warnings from the current row epoch can request a fast sweep; stale and row-switch transient warnings are logged and suppressed.

Useful console commands:

- `force_full_sweep`: queue a full row/device sweep for all rows.
- `force_full_sweep s=5 d=5`: queue a full sweep for the row containing that cell, for example row S5.
- `fdc_diag`: dump STATUS, CONFIG, MUX_CONFIG, IDs, RCOUNT, SETTLECOUNT, CLOCK_DIVIDERS, and DRIVE_CURRENT for both FDC2214 devices.
- `fdc_boot_sweep`: rerun the protected boot sweep synchronously.
- `fdc_rescue`: run synchronous full row/device no-oscillation rescue across the matrix.
- `fdc_period_ms 50`: override the text frame period at runtime. The first-stage default is 50 ms, or 20 fps.
- `fdc_profile summary on|off`: enable or disable `SCAN_TIMING_10`.
- `fdc_profile row on|off`: enable or disable `SCAN_ROW_TIMING`.
- `fdc_profile device on|off`: enable or disable `SCAN_DEVICE_TIMING`.
- `fdc_profile_every <N>`: set the summary interval. Default is 10 frames.
- `fdc_i2c_trace on|off|dump|clear`: control the FDC I2C trace ring. It records transactions without real-time per-register printf.
- `fdc_discard_frames 0|1|2`: runtime experiment for row-switch autoscan discard frames. The production row epoch path defaults to zero discard frames because FDC conversion is restarted with `CONFIG.SLEEP_MODE_EN`.

## FDC matrix row epoch state machine

Each matrix row has a unique conversion epoch. The coordinator sends both FDC
worker tasks into `CONFIG.SLEEP_MODE_EN=1`, waits for both sleep acknowledgments,
switches the TMUX row while both FDC devices are sleeping, waits
`CONFIG_SENSORARRAY_FDC_ROW_SWITCH_SETTLE_US`, applies only changed cached FDC
registers while still sleeping, exits sleep, waits for INTB/STATUS.DRDY, reads
STATUS and DATA, and merges primary D1-D4 with secondary D5-D8.

Primary and secondary workers are permanent tasks. The primary worker only uses
the primary FDC on I2C0; the secondary worker only uses the secondary FDC on
I2C1. If worker initialization fails, the firmware logs the reason and falls
back to a serial sleep-epoch path.

## Known recovery behaviour / Worker core affinity

`CONFIG_SENSORARRAY_FDC_WORKER_TASK_CORE=-1` means no affinity. That value must
not be passed to `xTaskCreateStaticPinnedToCore`; the firmware creates unpinned
static worker tasks for no-affinity mode, and only calls the pinned API with a
normalized core ID in the valid CPU range. On unicore builds the worker core is
resolved to core 0.

Worker creation failures are nonfatal. The firmware logs
`FDC_WORKER_CREATE_FAIL` / `FDC_PARALLEL_FALLBACK` and continues with the serial
sleep-epoch row path, so successful boot cache construction can still lead to
normal `MATRIXFDC_CAP` frames.

Boot full sweep is a startup cache-building phase. After
`FDC_BOOT_MATRIX_SWEEP_DONE` succeeds, repeated boot sweep plus reset usually
means a later panic/assert should be investigated first, especially task core
affinity or worker creation, before changing sweep parameters.

INTB is a data-ready hint only. GPIO ISR handlers record edge count, level,
timestamp, and epoch, then notify the worker task. They do not use I2C and do
not print. The worker still reads STATUS and requires DRDY or the CH0-CH3 unread
mask before DATA is accepted as fresh.

Diff-only cache apply keeps a per-device applied-register shadow. RCOUNT,
SETTLECOUNT, CLOCK_DIVIDERS, DRIVE_CURRENT, MUX_CONFIG, STATUS_CONFIG, and the
CONFIG base are written only when the desired value differs from the last
applied value. Sleep entry/exit writes are separate CONFIG writes and are the
normal per-row restart mechanism; the SD pin and RESET_DEV are not used for row
restart.

This is only an example test setup used to reproduce row-switch transient and
warning behaviour. The firmware must not depend on these specific cells or
external components.

## FDC throughput profiling

The first-stage FDC matrix target is 20 fps:

- `targetFrameUs = 50000`
- `targetRowUs = 6250`

The long-term reference target is 100 fps, but that requires later architecture
work. This revision keeps default output as text `MATRIXFDC_CAP` and does not enable
binary output.

`SCAN_TIMING_10` reports the default 10-frame aggregate: target fps, actual fps,
budget use, overrun count, row timing, sleep-before-row-switch timing, row
settle, diff apply, sleep-exit-to-INTB, worker job timing, dual-bus skew, cache
diff writes, warning counts, INTB counts, and FDC I2C write/read/timeout counts.
`SCAN_BOTTLENECK` is printed immediately on frame overrun with the top timing
contributors. Row and device timing are disabled by default and can be enabled
with the runtime commands.

Runtime FDC register verify defaults to `STARTUP_ONLY`: boot/cache-building paths
can still use readback verification, while the steady matrix loop checks write
`esp_err_t` and skips per-row full readback verify. `FULL` is for debug/bring-up;
`NONE` is for high-speed experiments. The high-speed profile option is disabled
by default because lower RCOUNT/SETTLECOUNT can improve frame rate while raising
frequency noise, stability, and amplitude-warning risk.

The default I2C frequency is 337500 Hz. If this increases NACK, timeout, or retry
counts, return to 325000 Hz before trying 350000, 375000, or 400000 Hz. Summary
bus timing is estimated from configured frequency; confirm limits with a logic
analyzer.

Primary and secondary FDC2214 devices must enter sleep before the matrix task
switches to the next row, and both workers must finish or time out before the
coordinator merges that row. This revision does not introduce I2C DMA, binary
output, or row-parameter reuse based on adjacent rows looking identical.
Cell-specific cache keys remain row/S/D/index/device/channel specific.

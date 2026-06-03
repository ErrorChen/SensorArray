# tmuxSwitch / TMUX GPIO Control Layer

---

## 中文说明

### 概述

`tmuxSwitch` 提供 **GPIO 级别的 TMUX 开关控制**，统一了 TMUX1108（行/源选择器）和 TMUX1134（模拟前端切换器）的接口。

**核心特性：纯 GPIO 驱动，不知道 S/D 映射或业务含义。它仅知道如何拉/放 GPIO 引脚。**

### 主要接口

```c
// 初始化
esp_err_t tmuxSwitchInit(void);
esp_err_t tmuxSwitchDeinit(void);

// TMUX1108 行选择 (S1-S8)
esp_err_t tmuxSwitchSelectRow(uint8_t row_index);  // row_index = 0..7 → S1..S8

// TMUX1108 源选择
esp_err_t tmuxSwitchSet1108Source(uint8_t source_level);
// source_level: 0 = GND, 1 = VBIAS/高电平

// TMUX1134 SELA 逻辑控制
esp_err_t tmux1134SelectSelALevel(bool high);
// high = true → SELA = 逻辑 1 (FDC 路由)
// high = false → SELA = 逻辑 0 (ADS 路由)

// TMUX1134 SELB 逻辑控制
esp_err_t tmux1134SelectSelBLevel(bool high);

// TMUX1134 EN (使能) 逻辑控制
esp_err_t tmux1134SetEnLogicalState(bool enabled);

// 状态读取
esp_err_t tmuxSwitchGetControlState(tmuxControlState_t *state_out);
```

### 工作流程

```c
// 初始化
tmuxSwitchInit();  // 配置所有 GPIO，设置初始态

// 选择行（TMUX1108）
// 读取 S1D1..S1D8（S1 行）
tmuxSwitchSelectRow(0);  // 选择第一行
vTaskDelay(pdMS_TO_TICKS(50));  // 等待模拟路径稳定

// 稍后，读取 S2 行
tmuxSwitchSelectRow(1);  // 选择第二行
vTaskDelay(pdMS_TO_TICKS(50));

// 选择前端（TMUX1134）
// 在 FDC 读取模式中
tmux1134SelectSelALevel(true);  // SELA → FDC 路由
tmux1134SelectSelBLevel(false); // SELB → OFF
tmux1134SetEnLogicalState(true);// EN → 使能

// 在 ADS 读取模式中（未来）
tmux1134SelectSelALevel(false); // SELA → ADS 路由

// 读取当前状态（诊断用）
tmuxControlState_t state = {0};
tmuxSwitchGetControlState(&state);
printf("Row:%u, SelA:%u, SelB:%u, EN:%u\n",
       state.current_row, state.sela_level, state.selb_level, state.en_level);

// 清理
tmuxSwitchDeinit();
```

### GPIO 映射

典型的 GPIO 配置（可在 Kconfig 中修改）：

```
TMUX1108 (行/源选择):
  S0   → GPIO X
  S1   → GPIO Y
  S2   → GPIO Z
  (3 位地址线, 对应 8 行)
  SW   → GPIO A
  (1 位源选择)

TMUX1134 (前端切换):
  SELA → GPIO B
  SELB → GPIO C
  EN   → GPIO D
```

### 设计边界

**tmuxSwitch 知道：**
- GPIO 引脚号
- 逻辑电平的物理表示
- TMUX 地址编码

**tmuxSwitch 不知道：**
- 哪个行是 S1、S2 等（业务层定义）
- 哪个前端配置对应 FDC vs ADS（业务层定义）
- 模拟路径的生效时间（业务层定义）
- 救援策略或行切换顺序（业务层定义）

### 分层示例

```
应用层 (core/measure/fdc)
  ├─ 知道 S1 行对应 TMUX 行 0
  ├─ 知道 D1-D4 对应 Primary FDC
  ├─ 调用 tmuxSwitchSelectRow(0)
  └─ 等待定时后进行 I2C 读取
       ↓
TMUX 驱动 (tmuxSwitch)
  ├─ 不知道 "S1" 的含义
  ├─ 只知道 "row_index=0" → GPIO addr = 0b000
  ├─ 拉低 S0/S1/S2 GPIO
  └─ 返回成功
```

### 当前状态

- ? TMUX1108 行选择 (8 行)
- ? TMUX1108 源选择 (GND / VBIAS)
- ? TMUX1134 SELA/SELB/EN 控制
- ? 状态读取与诊断
- ? GPIO 中断保护（可选）

---

## Australian English Documentation

### Overview

`tmuxSwitch` provides **GPIO-level TMUX switch control**, unifying TMUX1108 (row/source selector) and TMUX1134 (analogue front-end switcher) interfaces.

**Core principle: pure GPIO driver, unaware of S/D mapping or business semantics. It only knows how to pull/release GPIO pins.**

### Main APIs

```c
// Initialisation
esp_err_t tmuxSwitchInit(void);
esp_err_t tmuxSwitchDeinit(void);

// TMUX1108 row selection (S1-S8)
esp_err_t tmuxSwitchSelectRow(uint8_t row_index);  // row_index = 0..7 → S1..S8

// TMUX1108 source selection
esp_err_t tmuxSwitchSet1108Source(uint8_t source_level);
// source_level: 0 = GND, 1 = VBIAS/high

// TMUX1134 SELA logic control
esp_err_t tmux1134SelectSelALevel(bool high);
// high = true → SELA = logic 1 (FDC routed)
// high = false → SELA = logic 0 (ADS routed)

// TMUX1134 SELB logic control
esp_err_t tmux1134SelectSelBLevel(bool high);

// TMUX1134 EN (enable) logic control
esp_err_t tmux1134SetEnLogicalState(bool enabled);

// State readback
esp_err_t tmuxSwitchGetControlState(tmuxControlState_t *state_out);
```

### Workflow

```c
// Initialisation
tmuxSwitchInit();  // Configure all GPIO, set initial states

// Select row (TMUX1108)
// Read S1D1..S1D8 (S1 row)
tmuxSwitchSelectRow(0);  // Select first row
vTaskDelay(pdMS_TO_TICKS(50));  // Wait for analogue path stabilisation

// Later, read S2 row
tmuxSwitchSelectRow(1);  // Select second row
vTaskDelay(pdMS_TO_TICKS(50));

// Select front-end (TMUX1134)
// In FDC readout mode
tmux1134SelectSelALevel(true);   // SELA → FDC routed
tmux1134SelectSelBLevel(false);  // SELB → OFF
tmux1134SetEnLogicalState(true); // EN → enabled

// In ADS readout mode (future)
tmux1134SelectSelALevel(false); // SELA → ADS routed

// Read current state (diagnostics)
tmuxControlState_t state = {0};
tmuxSwitchGetControlState(&state);
printf("Row:%u, SelA:%u, SelB:%u, EN:%u\n",
       state.current_row, state.sela_level, state.selb_level, state.en_level);

// Cleanup
tmuxSwitchDeinit();
```

### GPIO mapping

Typical GPIO configuration (modifiable via Kconfig):

```
TMUX1108 (row/source selection):
  S0   → GPIO X
  S1   → GPIO Y
  S2   → GPIO Z
  (3-bit address lines, mapping to 8 rows)
  SW   → GPIO A
  (1-bit source selection)

TMUX1134 (front-end switching):
  SELA → GPIO B
  SELB → GPIO C
  EN   → GPIO D
```

### Design boundary

**tmuxSwitch knows:**
- GPIO pin numbers
- Physical representation of logic levels
- TMUX address encoding

**tmuxSwitch does NOT know:**
- Which row is S1, S2, etc. (application-defined)
- Which front-end config maps to FDC vs ADS (application-defined)
- Analogue path settling time (application-defined)
- Rescue strategy or row-switch sequence (application-defined)

### Layering example

```
Application layer (core/measure/fdc)
  ├─ Knows S1 row maps to TMUX row 0
  ├─ Knows D1-D4 map to Primary FDC
  ├─ Calls tmuxSwitchSelectRow(0)
  └─ Waits per timing, then performs I2C reads
       ↓
TMUX driver (tmuxSwitch)
  ├─ Does not know meaning of "S1"
  ├─ Only knows "row_index=0" → GPIO addr = 0b000
  ├─ Pulls S0/S1/S2 GPIO low
  └─ Returns success
```

### Current status

- ? TMUX1108 row selection (8 rows)
- ? TMUX1108 source selection (GND / VBIAS)
- ? TMUX1134 SELA/SELB/EN control
- ? State readback and diagnostics
- ? GPIO interrupt protection (optional)

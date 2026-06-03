# boardSupport / 板级资源管理层

---

## 中文说明

### 概述

`boardSupport` 是 ESP32-S3 硬件资源的统一管理层，负责 **I2C 总线初始化与生命周期管理**、**GPIO 映射与驱动回调适配**、以及 **I2C 故障恢复保护**。

它是一个 **资源层**，不承载业务逻辑或测量策略。

### 主要职责

| 职责 | 说明 |
|---|---|
| **I2C 总线初始化** | 初始化 primary (I2C0) 和 optional secondary (I2C1) 总线 |
| **I2C 回调适配** | 为 FDC2214 驱动提供符合其签名的 I2C 读写回调 |
| **I2C 事务保护** | 使用互斥锁保护 I2C 事务，防止并发冲突 |
| **I2C 故障诊断** | 跟踪 I2C 总线状态（ONLINE / OFFLINE / RECOVERING）、事务计数器、错误计数 |
| **I2C 恢复机制** | NACK 不触发恢复（设备级故障）；仅 TIMEOUT 且 SDA/SCL 确认低电平时触发恢复 |
| **GPIO 映射** | 提供 GPIO pin 编号查询（后续支持不同板级配置） |

### 公开 API

```c
// 初始化与反初始化
esp_err_t boardSupportInit(void);
esp_err_t boardSupportDeinit(void);

// 总线状态查询
bool boardSupportIsI2c1Enabled(void);
i2c_master_bus_handle_t boardSupportGetI2cCtx(void);
i2c_master_bus_handle_t boardSupportGetI2c1Ctx(void);

// 总线诊断信息
void boardSupportGetI2cBusInfo(bool secondary, BoardSupportI2cBusInfo_t *info);
// → info->Enabled, info->Port, info->Offline, info->TransactionCount, etc.

// I2C 回调适配（由驱动调用）
esp_err_t boardSupportI2cWriteRead(
    const Fdc2214CapBusConfig_t *cfg,
    const uint8_t *write_buffer,
    size_t write_size,
    uint8_t *read_buffer,
    size_t read_size);

esp_err_t boardSupportI2cWrite(
    const Fdc2214CapBusConfig_t *cfg,
    const uint8_t *write_buffer,
    size_t write_size);
```

### I2C 总线配置

**Primary Bus (I2C0)**
- 频率：400 kHz
- 主用途：Primary FDC2214、fallback Secondary FDC2214
- GPIO：`CONFIG_SENSORARRAY_I2C0_SDA` / `CONFIG_SENSORARRAY_I2C0_SCL`

**Secondary Bus (I2C1)** （可选）
- 频率：400 kHz
- 主用途：Secondary FDC2214（并行读取）
- 启用条件：`CONFIG_SENSORARRAY_I2C1_ENABLE` 且 GPIO 可用
- GPIO：`CONFIG_SENSORARRAY_I2C1_SDA` / `CONFIG_SENSORARRAY_I2C1_SCL`

**Fallback 策略**

若 Secondary 总线初始化失败，Secondary FDC2214 自动回落到 Primary 总线（串行读取）。

### I2C 故障处理

**普通 NACK**

- 特征：设备不应答
- 原因：设备未连接、地址错误、设备故障
- 处理：返回错误代码，**不触发驱动重装**
- 恢复：由上层（`core/measure` 或诊断命令）决定

**I2C 超时 + SDA/SCL 低电平**

- 特征：I2C 事务超时且时钟线被拉低
- 原因：设备 hang 住、从设备故障导致时钟伸展过长
- 处理：
  1. 设置 RECOVERING 状态
  2. 启动驱动软重置（寄存器级）
  3. 重新初始化 I2C 驱动
  4. 清除 OFFLINE 标志，恢复 ONLINE
- 冷却机制：恢复后有冷却时间，避免频繁重装

**诊断日志**

```
I2C0_STATUS,online=1,transactionCount=1234,errorCount=5,recovering=0
I2C1_STATUS,online=0,transactionCount=456,errorCount=10,recovering=1,recoveryTime=2000ms
```

### 边界说明

**boardSupport 不负责：**

- D-line 映射（`core/board/sensorarrayBoardMap.c`）
- I2C 地址配置的业务含义（应用层）
- FDC/ADS 特定的初始化顺序（`main/main.c`）
- 测量策略或救援逻辑（`core/measure`）

**boardSupport 的职责边界：**

```
应用层 (main)
  ↓ 调用 boardSupportInit()
  ↓
boardSupport
  ├─ 初始化 I2C 硬件
  ├─ 提供 GPIO pin 查询
  ├─ 提供 I2C 回调适配
  └─ 管理 I2C 故障恢复
  ↓ 暴露驱动调用的回调
  ↓
components (fdc2214Cap, ads126xAdc)
  ├─ 不知道 board 映射
  └─ 不知道应用路由
```

### 当前状态

- ✓ Primary I2C (I2C0) 稳定
- ✓ Optional Secondary I2C (I2C1) 支持
- ✓ 基础 NACK 处理
- ⏳ I2C 恢复机制（SDA/SCL 检测与驱动软重置）
- ⏳ GPIO pin 配置多板化（当前仅 default 板）

---

## Australian English Documentation

### Overview

`boardSupport` is the unified hardware resource management layer for ESP32-S3, responsible for **I2C bus initialisation and lifecycle management**, **GPIO mapping and driver callback adaptation**, and **I2C fault recovery protection**.

It is a **resource layer**, not carrying business logic or measurement strategy.

### Key responsibilities

| Responsibility | Description |
|---|---|
| **I2C bus initialisation** | Initialise primary (I2C0) and optional secondary (I2C1) buses |
| **I2C callback adaptation** | Provide I2C read/write callbacks matching FDC2214 driver signatures |
| **I2C transaction protection** | Protect I2C transactions with mutex to prevent concurrency conflicts |
| **I2C fault diagnostics** | Track I2C bus state (ONLINE / OFFLINE / RECOVERING), transaction counts, error counts |
| **I2C recovery mechanism** | NACK does not trigger recovery (device-level failure); only TIMEOUT with confirmed low SDA/SCL triggers recovery |
| **GPIO mapping** | Provide GPIO pin number queries (future support for multi-board configs) |

### Public API

```c
// Initialisation and deinitialisation
esp_err_t boardSupportInit(void);
esp_err_t boardSupportDeinit(void);

// Bus state queries
bool boardSupportIsI2c1Enabled(void);
i2c_master_bus_handle_t boardSupportGetI2cCtx(void);
i2c_master_bus_handle_t boardSupportGetI2c1Ctx(void);

// Bus diagnostic information
void boardSupportGetI2cBusInfo(bool secondary, BoardSupportI2cBusInfo_t *info);
// → info->Enabled, info->Port, info->Offline, info->TransactionCount, etc.

// I2C callback adaptation (called by driver)
esp_err_t boardSupportI2cWriteRead(
    const Fdc2214CapBusConfig_t *cfg,
    const uint8_t *write_buffer,
    size_t write_size,
    uint8_t *read_buffer,
    size_t read_size);

esp_err_t boardSupportI2cWrite(
    const Fdc2214CapBusConfig_t *cfg,
    const uint8_t *write_buffer,
    size_t write_size);
```

### I2C bus configuration

**Primary Bus (I2C0)**
- Frequency: 400 kHz
- Primary use: Primary FDC2214, fallback Secondary FDC2214
- GPIO: `CONFIG_SENSORARRAY_I2C0_SDA` / `CONFIG_SENSORARRAY_I2C0_SCL`

**Secondary Bus (I2C1)** (optional)
- Frequency: 400 kHz
- Primary use: Secondary FDC2214 (parallel readout)
- Enable condition: `CONFIG_SENSORARRAY_I2C1_ENABLE` and GPIO available
- GPIO: `CONFIG_SENSORARRAY_I2C1_SDA` / `CONFIG_SENSORARRAY_I2C1_SCL`

**Fallback strategy**

If Secondary bus initialisation fails, Secondary FDC2214 automatically falls back to Primary bus (serial readout).

### I2C fault handling

**Generic NACK**

- Characteristic: Device not acknowledging
- Cause: Device not connected, wrong address, device fault
- Handling: Return error code, **do not trigger driver reinstall**
- Recovery: Decided by upper layer (`core/measure` or diagnostic command)

**I2C timeout + SDA/SCL low**

- Characteristic: I2C transaction timeout and clock line pulled low
- Cause: Device hang-up, slave device fault causing excessive clock stretching
- Handling:
  1. Set RECOVERING state
  2. Start driver soft reset (register-level)
  3. Reinitialise I2C driver
  4. Clear OFFLINE flag, resume ONLINE
- Cooldown mechanism: After recovery, cooldown period avoids frequent reinstalls

**Diagnostic logging**

```
I2C0_STATUS,online=1,transactionCount=1234,errorCount=5,recovering=0
I2C1_STATUS,online=0,transactionCount=456,errorCount=10,recovering=1,recoveryTime=2000ms
```

### Boundary explanation

**boardSupport does NOT handle:**

- D-line mapping (`core/board/sensorarrayBoardMap.c`)
- I2C address configuration business semantics (application layer)
- FDC/ADS-specific initialisation sequencing (`main/main.c`)
- Measurement strategy or rescue logic (`core/measure`)

**boardSupport responsibility boundary:**

```
Application layer (main)
  ↓ Calls boardSupportInit()
  ↓
boardSupport
  ├─ Initialise I2C hardware
  ├─ Provide GPIO pin queries
  ├─ Provide I2C callback adaptation
  └─ Manage I2C fault recovery
  ↓ Expose callbacks for driver invocation
  ↓
components (fdc2214Cap, ads126xAdc)
  ├─ Unaware of board mapping
  └─ Unaware of application routing
```

### Current status

- ✓ Primary I2C (I2C0) stable
- ✓ Optional Secondary I2C (I2C1) supported
- ✓ Basic NACK handling
- ⏳ I2C recovery mechanism (SDA/SCL detection and driver soft reset)
- ⏳ GPIO pin configuration multi-board (currently default board only)

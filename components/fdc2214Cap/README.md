# fdc2214Cap / FDC2214 Generic Driver

---

## 中文说明

### 概述

`fdc2214Cap` 是 **芯片级 I2C 驱动**，专门处理 FDC2214/FDC2212 电容感应芯片的寄存器访问和样本读取。

**核心特性：不知道矩阵、行、D-line、TMUX 路由或救援策略。它仅知道如何与芯片通信。**

### 主要接口

```c
// 生命周期
Fdc2214CapDevice_t *dev = NULL;
Fdc2214CapCreate(&bus_config, &dev);     // 创建设备上下文
Fdc2214CapReset(dev);                    // 硬件复位
Fdc2214CapReadId(dev, &mfr_id, &dev_id); // 读制造商/设备 ID
Fdc2214CapDestroy(&dev);                 // 销毁上下文

// 配置
Fdc2214CapConfigureChannel(dev, FDC2214_CH0, &channel_cfg);
Fdc2214CapSetAutoScanMode(dev, 2, FDC2214_DEGLITCH_10MHZ);  // 2 通道自动扫描
Fdc2214CapSetSingleChannelMode(dev, FDC2214_CH0);            // 单通道模式

// 数据读取
Fdc2214CapReadSample(dev, FDC2214_CH0, &sample);      // 读单个通道样本
Fdc2214CapReadChannelsRaw(dev, ch_mask, raw_array);   // 批量读取
```

### 工作流程

```c
// 典型初始化流程
Fdc2214CapBusConfig_t bus = {
  .i2c_ctx = i2c_handle,
  .i2c_addr = 0x2B,
  .write_fn = boardSupportI2cWrite,
  .read_fn = boardSupportI2cRead,
};

Fdc2214CapDevice_t *dev = NULL;
Fdc2214CapCreate(&bus, &dev);

// 验证设备
Fdc2214CapReset(dev);
uint16_t mfr_id, dev_id;
Fdc2214CapReadId(dev, &mfr_id, &dev_id);
// 应检查 mfr_id = 0x5449 (TI), dev_id = 0x3055 (FDC2214)

// 配置通道
Fdc2214CapChannelConfig_t ch_cfg = {
  .fref = 43360,      // 参考频率 (Hz)
  .idrive = 10,       // 驱动电流 (μA)
  .settlecount = 128, // 稳定时间
};
Fdc2214CapConfigureChannel(dev, FDC2214_CH0, &ch_cfg);
Fdc2214CapConfigureChannel(dev, FDC2214_CH1, &ch_cfg);
// ... 配置 CH2, CH3

// 启用自动扫描
Fdc2214CapSetAutoScanMode(dev, 4, FDC2214_DEGLITCH_10MHZ);  // 4 通道、10 MHz deglitch

// 读取样本
Fdc2214CapSample_t sample;
Fdc2214CapReadSample(dev, FDC2214_CH0, &sample);
// sample.raw28    - 28 位原始码
// sample.freq_hz  - 换算后的频率

// 清理
Fdc2214CapDestroy(&dev);
```

### I2C 事务

本驱动的所有 I2C 通信都通过 `boardSupport` 提供的回调完成。调用者负责：

- I2C 总线初始化与管理
- 互斥锁保护（防止并发冲突）
- 超时处理
- 错误恢复

### 设计边界

**fdc2214Cap 知道：**
- FDC2214 寄存器地址与字段定义
- I2C 读写事务
- 样本转换（raw28 → frequency）

**fdc2214Cap 不知道：**
- TMUX 行选择
- D-line 映射或通道意义
- 矩阵扫描计划
- 全局救援策略
- 用户按下了哪一行

### 当前支持

- ✓ FDC2214/FDC2212 系列（根据 Kconfig）
- ✓ CH0-CH3 通道
- ✓ 单通道模式
- ✓ 4 通道自动扫描
- ✓ 原始 28 位码读取
- ✓ ID 验证

---

## Australian English Documentation

### Overview

`fdc2214Cap` is a **chip-level I2C driver** for FDC2214/FDC2212 capacitance-sensing chips, handling register access and sample reads.

**Core principle: knows nothing about matrices, rows, D-lines, TMUX routing, or rescue strategy. It only knows how to communicate with the chip.**

### Main APIs

```c
// Lifecycle
Fdc2214CapDevice_t *dev = NULL;
Fdc2214CapCreate(&bus_config, &dev);      // Create device context
Fdc2214CapReset(dev);                     // Hardware reset
Fdc2214CapReadId(dev, &mfr_id, &dev_id);  // Read manufacturer/device ID
Fdc2214CapDestroy(&dev);                  // Destroy context

// Configuration
Fdc2214CapConfigureChannel(dev, FDC2214_CH0, &channel_cfg);
Fdc2214CapSetAutoScanMode(dev, 2, FDC2214_DEGLITCH_10MHZ);  // 2-channel autoscan
Fdc2214CapSetSingleChannelMode(dev, FDC2214_CH0);            // Single-channel mode

// Data reads
Fdc2214CapReadSample(dev, FDC2214_CH0, &sample);     // Read single-channel sample
Fdc2214CapReadChannelsRaw(dev, ch_mask, raw_array);  // Batch read
```

### Workflow

```c
// Typical initialisation flow
Fdc2214CapBusConfig_t bus = {
  .i2c_ctx = i2c_handle,
  .i2c_addr = 0x2B,
  .write_fn = boardSupportI2cWrite,
  .read_fn = boardSupportI2cRead,
};

Fdc2214CapDevice_t *dev = NULL;
Fdc2214CapCreate(&bus, &dev);

// Verify device
Fdc2214CapReset(dev);
uint16_t mfr_id, dev_id;
Fdc2214CapReadId(dev, &mfr_id, &dev_id);
// Should check: mfr_id = 0x5449 (TI), dev_id = 0x3055 (FDC2214)

// Configure channels
Fdc2214CapChannelConfig_t ch_cfg = {
  .fref = 43360,      // Reference frequency (Hz)
  .idrive = 10,       // Drive current (μA)
  .settlecount = 128, // Settle time
};
Fdc2214CapConfigureChannel(dev, FDC2214_CH0, &ch_cfg);
Fdc2214CapConfigureChannel(dev, FDC2214_CH1, &ch_cfg);
// ... configure CH2, CH3

// Enable autoscan
Fdc2214CapSetAutoScanMode(dev, 4, FDC2214_DEGLITCH_10MHZ);  // 4-channel, 10 MHz deglitch

// Read sample
Fdc2214CapSample_t sample;
Fdc2214CapReadSample(dev, FDC2214_CH0, &sample);
// sample.raw28    - 28-bit raw code
// sample.freq_hz  - converted frequency

// Cleanup
Fdc2214CapDestroy(&dev);
```

### I2C transactions

All I2C communication is via callbacks provided by `boardSupport`. Caller responsible for:

- I2C bus initialisation and management
- Mutex protection (prevent concurrent conflicts)
- Timeout handling
- Error recovery

### Design boundary

**fdc2214Cap knows:**
- FDC2214 register addresses and fields
- I2C read/write transactions
- Sample conversion (raw28 → frequency)

**fdc2214Cap does NOT know:**
- TMUX row selection
- D-line mapping or channel semantics
- Matrix scan plan
- Global rescue strategy
- Which row the user pressed

### Current support

- ✓ FDC2214/FDC2212 family (per Kconfig)
- ✓ CH0-CH3 channels
- ✓ Single-channel mode
- ✓ 4-channel autoscan
- ✓ Raw 28-bit code reads
- ✓ ID verification

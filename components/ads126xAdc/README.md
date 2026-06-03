# ads126xAdc / ADS126x SPI Driver

---

## 中文说明

### 概述

`ads126xAdc` 是 **芯片级 SPI 驱动**，支持 ADS1262 和 ADS1263 精密 ADC，处理寄存器访问、转换控制、样本读取和单位转换。

**核心特性：不知道 D-line、模拟路由或应用策略。它仅知道如何与芯片通信。**

### 主要接口

```c
// 生命周期
ads126xAdcHandle_t adc = {0};
ads126xAdcConfig_t cfg = { ... };
ads126xAdcInit(&adc, &cfg);              // 初始化 ADC 上下文
ads126xAdcDeinit(&adc);                  // 反初始化

// 配置
ads126xAdcConfigure(&adc, true, false, ADS126X_CRC_OFF, 1, 0);
ads126xAdcSetInputMux(&adc, muxp, muxn); // 输入多路选择
ads126xAdcSetRefMux(&adc, ref_sel);      // 参考电压多路选择

// 转换控制
ads126xAdcStartAdc1(&adc);     // 启动 ADC1 转换
ads126xAdcStopAdc1(&adc);      // 停止 ADC1 转换

// 数据读取
int32_t raw = 0;
ads126xAdcReadAdc1Raw(&adc, &raw, NULL); // 读原始值
int32_t microvolt = ads126xAdcRawToMicrovolts(&adc, raw);
```

### ADS1263 的 ADC2 支持

若使用 ADS1263 且需要 ADC2（温度传感器、内部基准等）：

```c
ads126xAdcStartAdc2(&adc);
int32_t raw2 = 0;
ads126xAdcReadAdc2Raw(&adc, &raw2, NULL);
```

### 工作流程

```c
// SPI 总线初始化（由 boardSupport 负责）
SPI 配置：
  - 时钟频率：通常 5-20 MHz
  - 模式：SPI mode 1 (CPOL=0, CPHA=1)
  - 字长：8 bit

// ADC 初始化
ads126xAdcConfig_t cfg = {
  .spi_ctx = spi_handle,
  .cs_gpio = GPIO_NUM_XX,
  .drdy_gpio = GPIO_NUM_XX,
};
ads126xAdcInit(&adc, &cfg);

// 配置 ADC1
ads126xAdcConfigure(&adc,
  true,               // 启用 CRC
  false,              // 禁用 DACDAC
  ADS126X_CRC_16,    // CRC 类型
  1,                 // ADC 时钟分频器
  0);                // 保留

// 配置输入多路
// ADS1263: AIN0..AIN9, AINCOM
ads126xAdcSetInputMux(&adc, AIN0, AINCOM); // AIN0 vs 公共

// 配置参考电压
// 选项：VREF (外部), AVDD/AVSS (电源), 内部基准等
ads126xAdcSetRefMux(&adc, ADS126X_VREF_EXT);

// 启动转换
ads126xAdcStartAdc1(&adc);

// 等待 DRDY 或轮询
vTaskDelay(pdMS_TO_TICKS(10));  // 典型转换时间 ~5 ms

// 读取原始值
int32_t raw_code = 0;
ads126xAdcReadAdc1Raw(&adc, &raw_code, NULL);

// 转换为微伏
int32_t microvolt = ads126xAdcRawToMicrovolts(&adc, raw_code);
// 假设 VREF = 2.5V: raw_code = 0x7FFFFF → 2.5V = 2500000 ?V

// 停止
ads126xAdcStopAdc1(&adc);
ads126xAdcDeinit(&adc);
```

### SPI 事务

本驱动的 SPI 通信由 `boardSupport` 提供的回调完成。调用者负责：

- SPI 总线初始化与管理
- CS GPIO 控制（可选，取决于 SPI 总线配置）
- DRDY GPIO 中断（可选，可用轮询替代）
- 超时与错误处理

### 与 FDC 的互斥关系

**重要**：在 FDC 矩阵读取路径中，ADS 必须：

```
? 内部参考关闭 (REF_MUX)
? VBIAS 关闭
? ADC 转换停止
? 不使用测量路径中的任何模拟资源
```

这由 `core/measure/fdc` 中的 `sensorarrayMeasurePrepareFdcMatrixPath()` 强制执行。

### 设计边界

**ads126xAdc 知道：**
- ADS1262/ADS1263 寄存器地址与字段
- SPI 读写事务
- 参考电压与输入多路配置
- 原始码到微伏转换

**ads126xAdc 不知道：**
- D1..D8 D-line 映射
- 模拟路由 (SELA/SELB/SW)
- 矩阵扫描策略
- 何时应启用/禁用 ADS
- 与 FDC 的互斥规则

### 当前支持

- ? ADS1262 (单 ADC)
- ? ADS1263 (双 ADC + 温度传感器)
- ? 外部参考 (VREF)
- ? 电源参考 (AVDD/AVSS)
- ? 内部基准
- ? CRC 校验

---

## Australian English Documentation

### Overview

`ads126xAdc` is a **chip-level SPI driver** for ADS1262 and ADS1263 precision ADCs, handling register access, conversion control, sample reads, and unit conversion.

**Core principle: knows nothing about D-lines, analogue routing, or application strategy. It only knows how to communicate with the chip.**

### Main APIs

```c
// Lifecycle
ads126xAdcHandle_t adc = {0};
ads126xAdcConfig_t cfg = { ... };
ads126xAdcInit(&adc, &cfg);               // Initialise ADC context
ads126xAdcDeinit(&adc);                   // Deinitialise

// Configuration
ads126xAdcConfigure(&adc, true, false, ADS126X_CRC_OFF, 1, 0);
ads126xAdcSetInputMux(&adc, muxp, muxn); // Input multiplexing
ads126xAdcSetRefMux(&adc, ref_sel);      // Reference multiplexing

// Conversion control
ads126xAdcStartAdc1(&adc);     // Start ADC1 conversion
ads126xAdcStopAdc1(&adc);      // Stop ADC1 conversion

// Data reads
int32_t raw = 0;
ads126xAdcReadAdc1Raw(&adc, &raw, NULL); // Read raw value
int32_t microvolt = ads126xAdcRawToMicrovolts(&adc, raw);
```

### ADS1263 ADC2 support

If using ADS1263 and need ADC2 (temperature sensor, internal reference, etc.):

```c
ads126xAdcStartAdc2(&adc);
int32_t raw2 = 0;
ads126xAdcReadAdc2Raw(&adc, &raw2, NULL);
```

### Workflow

```c
// SPI bus initialisation (handled by boardSupport)
SPI configuration:
  - Clock frequency: typically 5-20 MHz
  - Mode: SPI mode 1 (CPOL=0, CPHA=1)
  - Word length: 8 bit

// ADC initialisation
ads126xAdcConfig_t cfg = {
  .spi_ctx = spi_handle,
  .cs_gpio = GPIO_NUM_XX,
  .drdy_gpio = GPIO_NUM_XX,
};
ads126xAdcInit(&adc, &cfg);

// Configure ADC1
ads126xAdcConfigure(&adc,
  true,               // Enable CRC
  false,              // Disable DACDAC
  ADS126X_CRC_16,    // CRC type
  1,                 // ADC clock divider
  0);                // Reserved

// Configure input mux
// ADS1263: AIN0..AIN9, AINCOM
ads126xAdcSetInputMux(&adc, AIN0, AINCOM); // AIN0 vs common

// Configure reference
// Options: VREF (external), AVDD/AVSS (supply), internal reference, etc.
ads126xAdcSetRefMux(&adc, ADS126X_VREF_EXT);

// Start conversion
ads126xAdcStartAdc1(&adc);

// Wait for DRDY or poll
vTaskDelay(pdMS_TO_TICKS(10));  // Typical conversion time ~5 ms

// Read raw value
int32_t raw_code = 0;
ads126xAdcReadAdc1Raw(&adc, &raw_code, NULL);

// Convert to microvolts
int32_t microvolt = ads126xAdcRawToMicrovolts(&adc, raw_code);
// Example: VREF = 2.5V: raw_code = 0x7FFFFF → 2.5V = 2500000 ?V

// Stop
ads126xAdcStopAdc1(&adc);
ads126xAdcDeinit(&adc);
```

### SPI transactions

All SPI communication via callbacks provided by `boardSupport`. Caller responsible for:

- SPI bus initialisation and management
- CS GPIO control (optional, depends on SPI bus config)
- DRDY GPIO interrupt (optional, can substitute polling)
- Timeout and error handling

### Mutual exclusion with FDC

**Important**: During FDC matrix readout path, ADS must:

```
? Internal reference OFF (REF_MUX)
? VBIAS OFF
? ADC conversion stopped
? No use of any analogue resources in measurement path
```

This is enforced by `sensorarrayMeasurePrepareFdcMatrixPath()` in `core/measure/fdc`.

### Design boundary

**ads126xAdc knows:**
- ADS1262/ADS1263 register addresses and fields
- SPI read/write transactions
- Reference voltage and input mux configuration
- Raw-to-microvolt conversion

**ads126xAdc does NOT know:**
- D1..D8 D-line mapping
- Analogue routing (SELA/SELB/SW)
- Matrix scan strategy
- When to enable/disable ADS
- Mutual exclusion rules with FDC

### Current support

- ? ADS1262 (single ADC)
- ? ADS1263 (dual ADC + temperature sensor)
- ? External reference (VREF)
- ? Supply reference (AVDD/AVSS)
- ? Internal reference
- ? CRC checking

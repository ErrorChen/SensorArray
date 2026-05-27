# fdc2214Cap / FDC2214 Driver

## 1) Scope / 模块范围

**中文**

`fdc2214Cap` 是通用 FDC2214 I2C 驱动，负责设备创建、复位、ID 读取、通道配置、单通道/自动扫描模式设置和样本读取。
不包含当前电路板或应用层的路由语义。

**English**

`fdc2214Cap` is a generic FDC2214 I2C driver.
It handles device creation/reset, ID readback, channel configuration, single/auto-scan mode, and sample reads.
It does not include board-specific or application routing semantics.

## 2) Main APIs / 主要接口

- `Fdc2214CapCreate` / `Fdc2214CapDestroy`
- `Fdc2214CapReset`
- `Fdc2214CapReadId`
- `Fdc2214CapConfigureChannel`
- `Fdc2214CapSetSingleChannelMode`
- `Fdc2214CapSetAutoScanMode`
- `Fdc2214CapReadSample`
- `Fdc2214CapReadChannelsRaw`

## 3) Integration Boundary / 集成边界

**中文**

- 本组件仅提供芯片能力。
- 板级映射和应用扫描策略由 driver 之外的上层模块定义。

**English**

- This component provides only chip-level capability.
- Board mapping and application scan policy are defined outside this driver by higher layers.

## 4) Kconfig Notes / Kconfig 说明

- I2C callbacks are provided by board/app layer (`boardSupport`).
- Channel count/mode policy for startup is app-layer behavior.

## 5) Typical Flow / 典型调用流程

```c
Fdc2214CapBusConfig_t bus = { ... };
Fdc2214CapDevice_t *dev = NULL;
Fdc2214CapCreate(&bus, &dev);
Fdc2214CapReset(dev);
Fdc2214CapReadId(dev, &mid, &did);
Fdc2214CapConfigureChannel(dev, FDC2214_CH0, &cfg);
Fdc2214CapSetAutoScanMode(dev, 2, FDC2214_DEGLITCH_10MHZ);
Fdc2214CapReadSample(dev, FDC2214_CH0, &sample);
```

## 6) Current Status / 当前状态

- Used by current app startup and matrix measurement code with ID check and sample reads.
- Kept generic and reusable.

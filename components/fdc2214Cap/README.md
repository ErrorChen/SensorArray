# fdc2214Cap / FDC2214 Chip-level I2C Driver

## 目录 / Table of contents

- [中文说明 / Chinese documentation](#中文说明--chinese-documentation)
- [Australian English documentation](#australian-english-documentation)
- [Kconfig](#kconfig)

## 中文说明 / Chinese documentation

### 职责

`components/fdc2214Cap` 是 FDC2214/FDC2212 芯片级 I2C driver。它只知道寄存器、I2C 事务、设备 handle、通道配置、autoscan/single-channel mode、STATUS 解码和 raw28 读取。

它不知道：

- S1-S8 行。
- D1-D8 的板级意义。
- primary/secondary FDC 对应哪些 D-line。
- TMUX1108/TMUX1134 路由。
- ADS/FDC 互斥。
- row epoch、boot sweep、rescue 策略。
- 64-cell frame 格式。
- `raw28 -> freqHz -> capTotalPf` 的矩阵级换算策略。

这些属于 `core/board` 和 `core/measure`。

### 主要类型

| 类型 | 作用 |
|---|---|
| `Fdc2214CapDevice_t` | opaque device handle。 |
| `Fdc2214CapBusConfig_t` | I2C address、user context、write/read callback、INT GPIO metadata。 |
| `Fdc2214CapChannel_t` | `FDC2214_CH0` 到 `FDC2214_CH3`。 |
| `Fdc2214CapChannelConfig_t` | `Rcount`、`SettleCount`、`Offset`、`ClockDividers`、`DriveCurrent`。 |
| `Fdc2214CapConfigOptions_t` | CONFIG register builder input。 |
| `Fdc2214CapStatus_t` | Decoded STATUS fields including DRDY, unread conversion and error flags。 |
| `Fdc2214CapSample_t` | Single-channel raw28 sample with decoded status/config context。 |
| `Fdc2214CapFastChannelSample_t` | Fast autoscan channel sample used by matrix path。 |
| `Fdc2214CapI2cStats_t` | Driver I2C counters and timing. |

### 真实 API

| API | 作用 |
|---|---|
| `Fdc2214CapCreate()`, `Fdc2214CapDestroy()` | Create/destroy device handle and mutex. |
| `Fdc2214CapReset()`, `Fdc2214CapReadId()` | Reset chip and read manufacturer/device IDs. |
| `Fdc2214CapConfigureChannel()`, `Fdc2214CapConfigureChannelWriteOnly()`, `Fdc2214CapConfigureChannelWithResult()` | Write channel registers; variants trade readback/detail for runtime speed. |
| `Fdc2214CapReadbackVerifyChannelConfig()`, `Fdc2214CapReadbackVerifyChannelConfigWithResult()` | Verify channel register configuration. |
| `Fdc2214CapBuildConfig()`, `Fdc2214CapEnterSleep()`, `Fdc2214CapExitSleep()` | Build CONFIG and explicitly enter/exit sleep mode. |
| `Fdc2214CapDecodeStatusRaw()`, `Fdc2214CapReadStatus()`, `Fdc2214CapClearStatus()` | STATUS decode, read and clear helpers. |
| `Fdc2214CapReadCoreRegs()`, `Fdc2214CapReadDebugSnapshot()` | Diagnostic register snapshots. |
| `Fdc2214CapSetSingleChannelMode()`, `Fdc2214CapSetAutoScanMode()`, `Fdc2214CapSetAutoScanModeWriteOnly()` | Configure conversion mode. |
| `Fdc2214CapReadSample()`, `Fdc2214CapReadSampleRelaxed()`, `Fdc2214CapReadChannelRawWithStatus()` | Single-channel raw sample reads. |
| `Fdc2214CapReadChannelsRaw()`, `Fdc2214CapReadAutoscan4RawFast()`, `Fdc2214CapReadChannelsDataRegsFast()` | Multi-channel/autoscan raw reads. |
| `Fdc2214CapReadRawRegisters()`, `Fdc2214CapWriteRawRegisters()` | Raw 16-bit register access. |
| `Fdc2214CapResetI2cStats()`, `Fdc2214CapGetI2cStats()` | I2C profiling counters. |
| `Fdc2214CapI2cTraceSetEnabled()`, `Fdc2214CapI2cTraceIsEnabled()`, `Fdc2214CapI2cTraceClear()`, `Fdc2214CapI2cTraceDump()` | Trace ring controls. |

### 最小调用形状

```c
Fdc2214CapBusConfig_t bus = {
    .I2cAddress7 = 0x2B,
    .UserCtx = (void *)boardSupportGetI2cCtx(),
    .WriteRead = boardSupportI2cWriteRead,
    .Write = boardSupportI2cWrite,
    .IntGpio = -1,
};

Fdc2214CapDevice_t *dev = NULL;
ESP_ERROR_CHECK(Fdc2214CapCreate(&bus, &dev));
ESP_ERROR_CHECK(Fdc2214CapReset(dev));

uint16_t manufacturerId = 0;
uint16_t deviceId = 0;
ESP_ERROR_CHECK(Fdc2214CapReadId(dev, &manufacturerId, &deviceId));

Fdc2214CapChannelConfig_t cfg = {
    .Rcount = 0x2089,
    .SettleCount = 0x0080,
    .Offset = 0x0000,
    .ClockDividers = 0x2001,
    .DriveCurrent = 0x7800,
};
ESP_ERROR_CHECK(Fdc2214CapConfigureChannel(dev, FDC2214_CH0, &cfg));
```

Drive-current values are normalised to the FDC2214 register mask before write. Use `0x7800` for the current project default; `0x7C00` includes bits outside the accepted drive-current field and will be masked by the driver.

## Australian English documentation

### Responsibility

`components/fdc2214Cap` is the chip-level I2C driver for FDC2214/FDC2212. It knows register addresses, I2C transactions, device handles, channel configuration, autoscan/single-channel modes, STATUS decoding, and raw28 reads.

It does not know rows, D-line meaning, primary/secondary board ownership, TMUX routing, ADS/FDC mutual exclusion, row epochs, boot sweep, rescue policy, 64-cell frame layout, or matrix-level capacitance conversion. Those belong to `core/board` and `core/measure`.

### API groups

- Lifecycle: `Fdc2214CapCreate()`, `Fdc2214CapDestroy()`.
- Identity/reset: `Fdc2214CapReset()`, `Fdc2214CapReadId()`.
- Channel config: `Fdc2214CapConfigureChannel*()`, `Fdc2214CapReadbackVerifyChannelConfig*()`.
- Conversion mode: `Fdc2214CapSetSingleChannelMode()`, `Fdc2214CapSetAutoScanMode*()`.
- Sleep epoch support: `Fdc2214CapBuildConfig()`, `Fdc2214CapEnterSleep()`, `Fdc2214CapExitSleep()`.
- Status and diagnostics: `Fdc2214CapReadStatus()`, `Fdc2214CapReadCoreRegs()`, `Fdc2214CapReadDebugSnapshot()`.
- Data reads: `Fdc2214CapReadSample*()`, `Fdc2214CapReadChannelsRaw()`, `Fdc2214CapReadAutoscan4RawFast()`.
- Raw registers: `Fdc2214CapReadRawRegisters()`, `Fdc2214CapWriteRawRegisters()`.
- I2C diagnostics: `Fdc2214CapGetI2cStats()` and the `Fdc2214CapI2cTrace*()` functions.

The driver reports raw data and chip status. Frequency and pF conversion used by `MATRIXFDC_CAP` are measurement-layer responsibilities.

Drive-current writes are masked to the FDC2214 drive-current field. The SensorArray default is `0x7800`; avoid documenting or tuning with `0x7C00` because the extra bit is discarded before the register write.

## Kconfig

| Option | Default | Notes |
|---|---:|---|
| `CONFIG_FDC2214CAP_ENABLE` | y | Enables the component. |
| `CONFIG_FDC2214CAP_LOG_LEVEL` | `3` | Component log level. |
| `CONFIG_FDC2214CAP_MUTEX_TIMEOUT_MS` | `200` | Internal mutex lock timeout. |
| `CONFIG_FDC2214CAP_LOW_LEVEL_I2C_TRACE` | n | Verbose per-transaction printf; keep off for normal matrix reads. |
| `CONFIG_FDC2214CAP_RAW_I2C_TRACE` | n | Raw register trace for diagnostics. |
| `CONFIG_FDC2214CAP_I2C_ADDR` | `0x2A` | Generic component default; the app uses primary/secondary project config instead. |
| `CONFIG_FDC2214CAP_CHANNELS` | `4` | Current matrix expects CH0-CH3 on each FDC. |

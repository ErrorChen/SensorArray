# ads126xAdc / ADS126x Chip-level SPI Driver

## 目录 / Table of contents

- [中文说明 / Chinese documentation](#中文说明--chinese-documentation)
- [Australian English documentation](#australian-english-documentation)
- [Kconfig](#kconfig)

## 中文说明 / Chinese documentation

### 职责

`components/ads126xAdc` 是 ADS1262/ADS1263 芯片级 SPI driver。它负责 SPI register read/write、ADC1/ADC2 conversion control、input mux、reference mux、POWER register policy、DRDY wait、raw ADC read 和 raw-to-microvolts conversion。

它不知道：

- D1-D8 D-line 的板级意义。
- SELA/SELB/SW 路由含义。
- 什么时候为了 FDC mode 关闭 ADS。
- FDC matrix scan/rescue 策略。
- SensorArray 64-cell frame 格式。

FDC matrix mode 下关闭 ADS conversion、internal reference 和 VBIAS 是 measurement-layer policy，不是 ADS driver 自己决定。

### 主要类型

| 类型 | 作用 |
|---|---|
| `ads126xAdcConfig_t` | SPI handle、DRDY/RESET GPIO、device type、CRC、status byte、reference、PGA/data-rate defaults。 |
| `ads126xAdcHandle_t` | Semi-opaque runtime handle with SPI buffers, mutex and chip settings. |
| `ads126xDeviceType_t` | `ADS126X_DEVICE_AUTO`、`ADS126X_DEVICE_ADS1262`、`ADS126X_DEVICE_ADS1263`。 |
| `ads126xCrcMode_t` | `ADS126X_CRC_OFF`、`ADS126X_CRC_CHECKSUM`、`ADS126X_CRC_CRC8`。 |

### 真实 API

| API | 作用 |
|---|---|
| `ads126xAdcInit()`, `ads126xAdcDeinit()` | Initialise/deinitialise the handle. |
| `ads126xAdcHardwareReset()`, `ads126xAdcSendCommand()` | Reset and command helpers. |
| `ads126xAdcReadRegisters()`, `ads126xAdcWriteRegisters()` | Raw register block access. |
| `ads126xAdcGetIdRaw()`, `ads126xAdcReadPowerRegister()`, `ads126xAdcWritePowerRegister()` | Identity and POWER register access. |
| `ads126xAdcApplyPowerPolicy()`, `ads126xAdcSetInternalReference()`, `ads126xAdcSetVbiasEnabled()` | Internal reference and VBIAS control. |
| `ads126xAdcConfigure()` | Core ADC configuration. |
| `ads126xAdcSetRefMux()`, `ads126xAdcSetInputMux()` | Reference and input mux selection. |
| `ads126xAdcReadCoreRegisters()` | Diagnostic register snapshot. |
| `ads126xAdcReadSingleDiffUv()` | One differential ADC1 read with mux/settle/discard/final conversion. |
| `ads126xAdcStartAdc1()`, `ads126xAdcStopAdc1()`, `ads126xAdcWaitDrdy()`, `ads126xAdcReadAdc1Raw()` | ADC1 conversion control and readout. |
| `ads126xAdcRawToMicrovolts()` | Raw ADC code to microvolts. |
| `ads126xAdcSelfOffsetCal()`, `ads126xAdcSelfGainCal()`, `ads126xAdcSystemOffsetCal()`, `ads126xAdcSystemGainCal()`, `ads126xAdcSelfCal()` | Calibration commands. |
| `ads126xAdcStartAdc2()`, `ads126xAdcStopAdc2()`, `ads126xAdcReadAdc2Raw()` | ADC2 APIs, useful on ADS1263; ADS1262 use may return not supported. |

### FDC mode safety

`sensorarrayMeasurePrepareFdcMatrixPath()` uses ADS driver APIs as part of FDC route safety:

```text
ads126xAdcStopAdc1()
ads126xAdcStopAdc2() when ADS1263 support is compiled
ads126xAdcApplyPowerPolicy() / related helpers to turn internal reference and VBIAS off
ads126xAdcSetRefMux() in measurement-layer route policy where needed
```

The driver exposes chip operations. The measurement layer decides when those operations are required.

## Australian English documentation

### Responsibility

`components/ads126xAdc` is the chip-level SPI driver for ADS1262/ADS1263. It owns SPI register access, conversion control, input and reference mux control, POWER register policy, DRDY waiting, raw reads, and raw-to-microvolts conversion.

It does not know D-line mapping, SELA/SELB/SW route meaning, when ADS must be disabled for FDC, FDC matrix scan strategy, or SensorArray frame layout.

Turning ADS conversion, internal reference, and VBIAS off during FDC matrix mode is a measurement-layer policy, not an ADS driver policy.

### API groups

- Lifecycle: `ads126xAdcInit()`, `ads126xAdcDeinit()`.
- Register access: `ads126xAdcReadRegisters()`, `ads126xAdcWriteRegisters()`.
- Power/reference: `ads126xAdcApplyPowerPolicy()`, `ads126xAdcSetInternalReference()`, `ads126xAdcSetVbiasEnabled()`, `ads126xAdcSetRefMux()`.
- Input selection: `ads126xAdcSetInputMux()`.
- ADC1 conversion: `ads126xAdcStartAdc1()`, `ads126xAdcStopAdc1()`, `ads126xAdcWaitDrdy()`, `ads126xAdcReadAdc1Raw()`, `ads126xAdcReadSingleDiffUv()`.
- ADC2 conversion: `ads126xAdcStartAdc2()`, `ads126xAdcStopAdc2()`, `ads126xAdcReadAdc2Raw()`.
- Conversion helpers: `ads126xAdcRawToMicrovolts()`.
- Calibration: self and system offset/gain calibration functions.

## Kconfig

| Option | Default | Notes |
|---|---:|---|
| `CONFIG_ADS126X_LOG_LEVEL` | `3` | Driver log level. |
| `CONFIG_ADS126X_SPI_CLOCK_HZ` | `2000000` | SPI clock in Hz. |
| `CONFIG_ADS126X_HAS_ADC2` | y | Builds ADC2 APIs. ADS1262 hardware does not provide ADC2. |
| `CONFIG_ADS126X_HELPER_CREATE_SPI` | n | Optional helper SPI create/destroy functions for quick validation. |
| `CONFIG_SENSORARRAY_SPI_USE_DMA` | y | Project SPI DMA option consumed by the ADS implementation. |
| `CONFIG_SENSORARRAY_SPI_MAX_TRANSFER_BYTES` | `64` | SPI transfer buffer sizing. |

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

CAP/VOLT/RES 下是否停止 conversion、开启 INTREF/VBIAS 以及选择哪个 REFMUX，
都是 measurement-layer 的显式 board profile，不是 ADS driver 自己决定。矩阵
激励 REF 也不是本 driver 的一个 bool。

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
| `ads126xAdcClearResetFlag()` | Clear the POWER.RESET sticky bit and verify readback before normal conversions. |
| `ads126xAdcApplyPowerPolicy()`, `ads126xAdcSetInternalReference()`, `ads126xAdcSetVbiasEnabled()` | Internal reference and VBIAS control. |
| `ads126xAdcConfigure()` | Core ADC configuration. |
| `ads126xAdcSetRefMux()`, `ads126xAdcSetInputMux()` | Reference and input mux selection. |
| `ads126xAdcSetPgaGain()`, `ads126xAdcSetPgaBypass()`, `ads126xAdcSetInputMuxVerified()` | Bounded write/readback for PGA, verified MODE2 bypass, and input-mux updates. |
| `ads126xAdcBuildMode2()`, `ads126xAdcSetMode2Fast()`, `ads126xAdcSetMode2Verified()` | Reuse the driver MODE2 encoder for cached hot-path writes or explicit readback verification. |
| `ads126xAdcSetInputMuxFast()` | Write a changed matrix INPMUX without unconditional per-cell readback; the measurement shadow decides when verification is required. |
| `ads126xAdcReadCoreRegisters()` | Critical POWER/MODE2/INPMUX/REFMUX diagnostic/readback snapshot. |
| `ads126xAdcReadAdc1RegisterSnapshot()`, `ads126xAdcRestoreAdc1RegisterSnapshot()` | Save/restore and fully verify POWER, INTERFACE, MODE0/1/2, INPMUX, REFMUX, OFCAL and FSCAL around battery/ADSCHK transactions. |
| `ads126xAdcIsAdc1Running()`, `ads126xAdcIsAdc2Running()` | Report driver-tracked running state so an ADS1263 transaction can preserve ADC2 while ADS1262 remains ADC1-only. |
| `ads126xAdcStatusByteHasAdc1NewData()`, `ads126xAdcStatusByteHasReferenceAlarm()`, `ads126xAdcStatusByteHasPgaAlarm()` | Decode fresh conversion, reference, and PGA absolute/differential status. |
| `ads126xAdcReadSingleDiffUv()` | One differential ADC1 read with mux/settle/discard/final conversion. |
| `ads126xAdcStartAdc1()`, `ads126xAdcStopAdc1()`, `ads126xAdcWaitDrdy()`, `ads126xAdcReadAdc1Raw()` | ADC1 conversion control and readout. |
| `ads126xAdcRawToMicrovolts()` | Raw ADC code to microvolts. |
| `ads126xAdcSelfOffsetCal()`, `ads126xAdcSelfGainCal()`, `ads126xAdcSystemOffsetCal()`, `ads126xAdcSystemGainCal()`, `ads126xAdcSelfCal()` | Calibration commands. |
| `ads126xAdcStartAdc2()`, `ads126xAdcStopAdc2()`, `ads126xAdcReadAdc2Raw()` | ADC2 APIs, useful on ADS1263; ADS1262 use may return not supported. |

### Mode safety boundary

`sensorarrayRouteController` uses ADS driver APIs as part of CAP/VOLT/RES route safety:

```text
ads126xAdcStopAdc1()
ads126xAdcStopAdc2() when ADS1263 support is compiled
ads126xAdcApplyPowerPolicy() for the selected independent INTREF/VBIAS policy
ads126xAdcSetRefMux() and critical-register readback
```

The driver exposes chip operations. The measurement layer decides when they
are required. VOLT/RES use ADC1, which is available on ADS1262 and ADS1263;
ADC2 calls return an explicit unsupported result when the detected device lacks
ADC2. Runtime `ADSBOOT` identity, not a compile-time label, is authoritative.

## Australian English documentation

### Responsibility

`components/ads126xAdc` is the chip-level SPI driver for ADS1262/ADS1263. It owns SPI register access, conversion control, input and reference mux control, POWER register policy, DRDY waiting, raw reads, and raw-to-microvolts conversion.

It does not know D-line mapping, SELA/SELB/SW route meaning, when ADS must be disabled for FDC, FDC matrix scan strategy, or SensorArray frame layout.

Stopping conversion and independently selecting INTREF, VBIAS and REFMUX for
CAP/VOLT/RES are measurement-layer policies, not driver policies. Matrix
excitation REF is a separate board route outside this component.

### API groups

- Lifecycle: `ads126xAdcInit()`, `ads126xAdcDeinit()`.
- Register access: `ads126xAdcReadRegisters()`, `ads126xAdcWriteRegisters()`.
- Power/reference: `ads126xAdcClearResetFlag()`, `ads126xAdcApplyPowerPolicy()`, `ads126xAdcSetInternalReference()`, `ads126xAdcSetVbiasEnabled()`, `ads126xAdcSetRefMux()`.
- Verification/status: critical-register readback, verified PGA updates, and
  status helpers for reference and PGA absolute/differential alarms.
- Input/PGA selection: `ads126xAdcSetInputMux()`, `ads126xAdcSetPgaGain()`, and verified `ads126xAdcSetPgaBypass()`.
- ADC1 conversion: `ads126xAdcStartAdc1()`, `ads126xAdcStopAdc1()`, `ads126xAdcWaitDrdy()`, `ads126xAdcReadAdc1Raw()`, `ads126xAdcReadSingleDiffUv()`.
- ADC2 conversion: `ads126xAdcStartAdc2()`, `ads126xAdcStopAdc2()`, `ads126xAdcReadAdc2Raw()`.
- Conversion helpers: `ads126xAdcRawToMicrovolts()`.
- Calibration: self and system offset/gain calibration functions.

## Kconfig

| Option | Default | Notes |
|---|---:|---|
| `CONFIG_ADS126X_LOG_LEVEL` | `3` | Driver log level. |
| `CONFIG_ADS126X_SPI_CLOCK_HZ` | `2000000` | SPI clock in Hz. |
| `CONFIG_ADS126X_MUTEX_TIMEOUT_MS` | `100` | Finite maximum wait for the driver SPI mutex. |
| `CONFIG_ADS126X_HAS_ADC2` | y | Builds ADC2 APIs. ADS1262 hardware does not provide ADC2. |
| `CONFIG_ADS126X_HELPER_CREATE_SPI` | n | Optional helper SPI create/destroy functions for quick validation. |
| `CONFIG_SENSORARRAY_SPI_USE_DMA` | y | Project SPI DMA option consumed by the ADS implementation. |
| `CONFIG_SENSORARRAY_SPI_MAX_TRANSFER_BYTES` | `64` | SPI transfer buffer sizing. |

The driver mutex uses a finite wait. It does not contain matrix coordinates,
resistance math, autorange policy, or C/V/R frame formatting; those belong in
`core/measure/ads`.

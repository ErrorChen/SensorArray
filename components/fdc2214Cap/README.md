# fdc2214Cap / FDC2214 Driver

`fdc2214Cap` 是通用 FDC2214 I2C driver。默认应用是 `PIEZO_READ / 压电读取`，也可切换到 `RESISTANCE_READ / 电阻读取`；这两个 ADS126x 读取模式都不初始化、不轮询 FDC2214。FDC2214 源码和旧 S5D5 debug 仍保留，但只能通过 `DEBUG / bring-up` mode 进入。

## Scope

组件只包含 FDC2214 芯片级能力：device create/reset/ID、channel config、single-channel/auto-scan mode、sample/status read、frequency/capacitance conversion、drive-current sweep/lock。

组件不包含：

- `S5D5`、`S1D1` 等矩阵点语义。
- `SELA`、`SELB`、`SDA11/SCL12` 等板级路由/引脚归属。
- `D1..D4 -> primary`、`D5..D8 -> secondary` 的板级 mapping。
- `PIEZO_READ`、`RESISTANCE_READ` 或 ADS126x 读取模式策略。

这些仍属于 `main/sensorarrayBoardMap.c` 和 debug/app 层。

Board-layer reminder: current SensorArray SW polarity is `SW LOW -> REF` and `SW HIGH -> GND`. Capacitive / FDC2214 routes require the GND source, so the board layer must command `TMUX1108_SOURCE_GND` and observe SW HIGH before FDC samples are trusted.

## Solidified Helper APIs

新增/整理的通用 helper：

- `fdc2214CapConfigureSingleChannelContinuousDefault()`
- `fdc2214CapApplySingleChannelProfile()`
- `fdc2214CapReadSampleRelaxed()`
- `fdc2214CapReadSampleWithStatus()`
- `fdc2214CapRaw28ToSensorFrequencyHz()`
- `fdc2214CapFrequencyToCapacitancePf()`
- `fdc2214CapSweepDriveCurrent()`
- `fdc2214CapLockDriveCurrent()`

默认 single-channel profile 使用调试阶段验证过的通用寄存器值：

```text
RCOUNT=0x2089
SETTLECOUNT=0x000A
OFFSET=0x0000
CLOCK_DIVIDERS=0x2001
DRIVE_CURRENT=0x7C00
STATUS_CONFIG=0x3800
DEGLITCH=10MHz
```

这些值不编码任何板级点位，只是 FDC2214 channel profile。

## Typical Single-Channel Flow

```c
Fdc2214CapBusConfig_t bus = { ... };
Fdc2214CapDevice_t *dev = NULL;
Fdc2214CapCreate(&bus, &dev);
Fdc2214CapReset(dev);
Fdc2214CapReadId(dev, &manufacturer, &deviceId);

fdc2214CapSingleChannelResult_t result = {0};
fdc2214CapConfigureSingleChannelContinuousDefault(dev, FDC2214_CH0, &result);

Fdc2214CapStatus_t status = {0};
Fdc2214CapSample_t sample = {0};
fdc2214CapReadSampleWithStatus(dev, FDC2214_CH0, true, &sample, &status);
```

## Frequency And Capacitance

Raw 28-bit code to sensor frequency：

```text
frequency_hz = raw28 * ref_clock_hz / 2^28
```

LC capacitance estimate：

```text
cap_pf = 1 / ((2*pi*f)^2 * L_henry) * 1e12
```

Use this only when the tank inductor value and clock source are known.

## Drive Current Sweep / Lock

`fdc2214CapSweepDriveCurrent()` tries high-current settings and drive-current candidates, reads several samples, scores candidates using I2C success, valid/non-zero raw, unread/data-ready state, watchdog/amplitude faults, and raw span, then returns the best result. `fdc2214CapLockDriveCurrent()` applies a selected high-current + drive-current pair and verifies/write-backs the effective drive-current register.

The helper is generic: the app/debug layer decides which FDC device and channel correspond to a board D-line.

## Restore FDC Debug Mode

Default firmware does not enter FDC debug. To recover the previous workflows:

```bash
idf.py menuconfig
```

Select:

- `SensorArray application mode -> Debug / bring-up modes`
- `Debug execution mode -> S5D5 FDC secondary` or `FDC I2C discovery` / `FDC self-test`

Then build/flash/monitor as usual.

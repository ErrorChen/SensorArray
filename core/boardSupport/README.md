# boardSupport / 板级资源管理层

## 目录 / Table of contents

- [中文说明 / Chinese documentation](#中文说明--chinese-documentation)
- [Australian English documentation](#australian-english-documentation)
- [Kconfig](#kconfig)

## 中文说明 / Chinese documentation

`core/boardSupport` 是板级 I2C/SPI/GPIO 资源层。当前实现重点是 I2C0 和可选 I2C1 的初始化、事务互斥、FDC driver callback 适配、总线状态记录，以及受保护的 I2C timeout recovery。

它不负责 D-line 映射、FDC/ADS 路由策略、矩阵扫描、rescue 或 frame 输出。

### 真实 API

| API | 作用 |
|---|---|
| `boardSupportInit()` | Initialise primary I2C and optional secondary I2C. |
| `boardSupportDeinit()` | Delete installed I2C drivers and clear init state. |
| `boardSupportIsI2c1Enabled()` | Returns true only when I2C1 is requested, pins are valid and port differs from primary. |
| `boardSupportGetI2cCtx()` | Returns primary `BoardSupportI2cCtx_t`. |
| `boardSupportGetI2c1Ctx()` | Returns secondary ctx only when enabled, installed and not offline. |
| `boardSupportGetI2cBusInfo()` | Copies bus diagnostics into `BoardSupportI2cBusInfo_t`. |
| `boardSupportRecoverI2cBus()` | Performs guarded bus recovery after timeout/stuck-bus conditions. |
| `boardSupportI2cWriteRead()`, `boardSupportI2cWrite()`, `boardSupportI2cRead()` | FDC-compatible I2C callbacks. |
| `boardSupportI2cProbeAddress()` | Address probe transaction. |

### I2C recovery policy

- NACK is treated as a device/address failure and does not reinstall the I2C driver.
- Recovery is considered only for `ESP_ERR_TIMEOUT`.
- Recovery requires SDA/SCL to be observed stuck low.
- Recovery is throttled by cooldown and max-failure counters.
- Repeated recovery failure marks the bus offline.

Key logs include `BOARD_I2C_CFG`, `BOARD_I2C_INIT`, `BOARD_I2C_READY`, `BOARD_I2C_XFER`, `BOARD_I2C_ERR`, `BOARD_I2C_RECOVERY`, `BOARD_I2C_OFFLINE` and `I2C_REJECT`.

## Australian English documentation

`core/boardSupport` is the board-level I2C/SPI/GPIO resource layer. The current implementation focuses on primary I2C and optional secondary I2C initialisation, transaction mutexes, FDC driver callback adaptation, bus diagnostics, and guarded I2C timeout recovery.

It does not own D-line mapping, FDC/ADS route policy, matrix scanning, rescue policy, or frame output.

### Runtime behaviour

`boardSupportInit()` configures I2C0 first. If I2C1 is requested and valid, it initialises I2C1 on a different ESP-IDF I2C port. If secondary setup is invalid or fails, the secondary bus is disabled and the application can continue primary-only where higher layers allow it.

The I2C callbacks validate installed/offline/recovering state, lock the bus mutex, run the ESP-IDF I2C transaction, update counters, log result state, and optionally trigger guarded recovery.

## Kconfig

| Option | Default | Notes |
|---|---:|---|
| `CONFIG_BOARD_I2C_PORT` | `0` | Primary I2C port. |
| `CONFIG_BOARD_I2C_SDA_GPIO`, `CONFIG_BOARD_I2C_SCL_GPIO` | `9`, `10` | Primary bus pins. |
| `CONFIG_BOARD_I2C_FREQ_HZ`, `CONFIG_BOARD_I2C0_FREQ_HZ` | `337500` | Primary bus frequency. |
| `CONFIG_BOARD_I2C1_ENABLE` | y | Enables optional secondary bus when pins/port are valid. |
| `CONFIG_BOARD_I2C1_PORT` | `1` | Secondary I2C port. |
| `CONFIG_BOARD_I2C1_SDA_GPIO`, `CONFIG_BOARD_I2C1_SCL_GPIO` | `11`, `12` | Secondary bus pins. |
| `CONFIG_BOARD_I2C1_FREQ_HZ` | `337500` | Secondary bus frequency. |
| `CONFIG_SENSORARRAY_I2C_RECOVERY_ENABLED` | y | Enables guarded timeout recovery. |
| `CONFIG_SENSORARRAY_I2C_RECOVERY_COOLDOWN_MS` | `1000` | Cooldown between recovery attempts. |
| `CONFIG_SENSORARRAY_I2C_RECOVERY_MAX_FAILS` | `3` | Recovery failures before offline. |
| `CONFIG_SENSORARRAY_I2C_RECOVERY_TOGGLE_CLOCKS` | `9` | SCL pulses during recovery. |

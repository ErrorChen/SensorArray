# SensorArray Architecture

## Layering

- `main`: app lifecycle, initialization scheduling, diagnostic/safe idle, and frame output calls.
- `core/measure`: measurement core and public frame/scan-plan APIs.
- `core/measure/fdc`: FDC matrix readout, sweep/cache management, and rescue throttling.
- `core/measure/ads`: ADS matrix readout entry points.
- `core/measure/mixed`: mixed-row scheduling entry points.
- `main/output`: text frame output and future protocol output adapters.
- `components/*`: low-level chip drivers only.
- `boardSupport`: board resource lifecycle, GPIO/SPI/I2C ownership, and guarded legacy I2C recovery.

## FDC Flow

`app_main` initializes runtime storage, board support, frontends, and scan plans.
FDC work then flows through `sensorarrayFdcMatrixEngineRunBootSweep`,
`sensorarrayFdcMatrixEngineReadFrame`, `sensorarrayMeasureReadFdcMatrixFrame`,
FDC sweep/cache/rescue helpers, `fdc2214Cap`, and finally the `boardSupportI2c`
transaction wrappers.

`main.c` does not call the FDC2214 low-level driver directly. `core/measure`
does not install or delete I2C drivers. The FDC2214 driver does not know matrix
rows, D-lines, or frame output.

## ADS Flow

`app_main` initializes the ADS frontend through board bring-up. Runtime reads
flow through `sensorarrayAdsMatrixEngineReadFrame`, ADS measurement helpers,
`ads126xAdc`, and board-owned SPI/GPIO resources.

## FDC File Split Rule

FDC code should stay in a few high-cohesion files:

- `sensorarrayFdcMatrix.c/.h`: FDC matrix engine facade.
- `sensorarrayFdcSweep.c/.h`: sweep/cache calibration and full-matrix rescue implementation.
- `sensorarrayFdcRescue.c/.h`: runtime all-invalid rescue policy.

Small helpers used by only one of those files should remain static or local
include fragments. Add a new FDC file only for an independent state machine or a
stable public API.

## I2C Recovery

`boardSupport` owns legacy I2C driver install/delete and all I2C transactions.
Each bus has a mutex, installed/offline/recovering state, counters, and recovery
cooldown. Plain NACKs are device-level failures and do not trigger driver
delete/reinstall. Recovery is only attempted after `ESP_ERR_TIMEOUT` when SDA or
SCL is confirmed stuck low.

## Output

Frame output reads a completed frame and prints host-compatible text. It must
not mutate measurement state.

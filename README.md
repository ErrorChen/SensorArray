# SensorArray ESP-IDF firmware

This firmware is organized around a formal FDC2214 8x8 matrix read path.

## Architecture boundaries

- `components/fdc2214Cap/`: FDC2214 chip-level driver only. It owns register definitions, I2C register access, reset/ID reads, clock/channel/autoscan/deglitch/drive-current configuration, status reads, raw28 sample reads, and optional raw conversion helpers. It does not know board S/D lines, SW, SELA/SELB, or ADS1263 policy.
- `core/board/`: board mapping single source of truth. It owns row selection meaning, D-line to FDC device/channel mapping, D-line to ADS channel mapping, SELA route to GPIO level mapping, SELB policy, and board-level helpers.
- `core/measure/`: production measurement logic. It prepares the FDC matrix hardware state, enforces ADS/FDC mutual exclusion, reads the 8x8 FDC matrix, builds frames, and emits frames.
- `main/`: thin app orchestration. It initializes board support, TMUX, ADS, both FDC2214 devices, then runs the FDC matrix loop.
- `transport/`: existing wire/USB/BLE transport placeholders. Binary FDC matrix output is reserved for explicit fast-speed mode only.

Temporary single-point and FDC discovery bring-up debug entry points have been removed from the default firmware.

## Board configuration notes

Current hardware bring-up logs have shown 16 MB flash detected while the image header is still configured for 2 MB. This is not the I2C boot-panic root cause because the app image loads and enters `app_main()`, but a later board-config pass should change menuconfig/sdkconfig flash size to 16 MB before relying on NVS, partition, or OTA space calculations.

## FDC D-line mapping

- D1 -> primary FDC2214 CH0
- D2 -> primary FDC2214 CH1
- D3 -> primary FDC2214 CH2
- D4 -> primary FDC2214 CH3
- D5 -> secondary FDC2214 CH0
- D6 -> secondary FDC2214 CH1
- D7 -> secondary FDC2214 CH2
- D8 -> secondary FDC2214 CH3

Both FDC2214 devices are initialized for CH0-CH3 autoscan.

## FDC matrix frame

The production frame type is `sensorarrayFdcMatrixFrame_t`:

- `timestampUs`: from `esp_timer_get_time()`
- `sequence`: increments once per generated frame
- `freqHz[64]`: primary payload, converted from each cell's current `raw28` and FDC clock-divider context
- `capTotalPf[64]`: optional CPU-derived total LC tank capacitance payload
- `raw28[64]`: debug payload only
- `validMask`: bit `i` means `freqHz[i]` is valid
- `capValidMask`: bit `i` means `capTotalPf[i]` is valid
- `warnMask`: bit `i` means that cell had a non-blocking warning such as amplitude warning
- `errorMask`: bit `i` means that cell failed, timed out, or had invalid status/data

Frame order is row-major:

`[S1D1,S1D2,S1D3,S1D4,S1D5,S1D6,S1D7,S1D8,S2D1,...,S8D8]`

Index formula:

`index = (sIndex - 1) * 8 + (dIndex - 1)`

## FDC matrix hardware state

Before each FDC matrix frame, `sensorarrayMeasurePrepareFdcMatrixPath()` enforces:

- SW source GND
- ADS1263 internal reference off
- ADS1263 VBIAS off
- ADS conversion stopped
- SELA routed to FDC2214 via board-map GPIO mapping
- SELB set through the board-map FDC policy

SW physical high is controlled by `sensorarrayMeasureSetSwPhysicalLevel()`. It only controls the SW GPIO physical level and does not enable ADS internal reference or VBIAS.

## Output policy

Default output is one pF printf text line per frame:

`MATRIXFDC_CAP,seq=<sequence>,timestampUs=<timestampUs>,partial=<0|1>,frameQuality=<full|partial>,capValidMask=0x<16hex>,freshMask=0x<16hex>,warnMask=0x<16hex>,errorMask=0x<16hex>,invalidSentinel=-1.000000,capTotalPf=[<64 pF values>]`

Frequency output is optional debug output and is emitted as a separate line when
`SENSORARRAY_FDC_TEXT_OUTPUT_FREQ_HZ` or
`SENSORARRAY_FDC_TEXT_OUTPUT_BOTH_SEPARATE` is selected:

`MATRIXFDC_FREQ,seq=<sequence>,timestampUs=<timestampUs>,validMask=0x<16hex>,warnMask=0x<16hex>,errorMask=0x<16hex>,freqHz=[<64 Hz values>]`

`capTotalPf` is computed from the same `freqHz[64]` values with
`C = 1 / ((2*pi*f)^2 * L)`, using `CONFIG_SENSORARRAY_FDC_TANK_INDUCTOR_NH`
(`18000` nH by default). It adds no FDC I2C read, no sweep, and no extra ready
wait. It is the total equivalent LC tank capacitance, including sensor,
inductor parasitics, FDC input parasitics, TMUX parasitics, PCB/FPC/cable
parasitics, and fixture capacitance. It is not a parasitic-subtracted pure
sensor capacitance. Future delta reporting should use a per-cell baseline:
`capDeltaPf = capTotalPf - capBaselinePf`.

Example:

`MATRIXFDC_CAP,seq=12,timestampUs=345678901,partial=0,frameQuality=full,capValidMask=0xFFFFFFFFFFFFFFFF,freshMask=0xFFFFFFFFFFFFFFFF,warnMask=0x0000000000000000,errorMask=0x0000000000000000,invalidSentinel=-1.000000,capTotalPf=[18.692341,18.693266,...]`

Raw FDC2214 codes are emitted only as a separate debug line:

`DEBUGFDC_RAW,seq=12,timestampUs=345678901,raw28=[9215955,9215731,...]`

Binary output is not enabled by default. `sensorarrayFastSpeedIsEnabled()` currently defaults false; the binary sender is reserved and returns `ESP_ERR_NOT_SUPPORTED` until an explicit host fast-speed/binary command path is added.

`MATRIXFDC_CAP` remains the normal periodic output, even for degraded frames. Invalid cells always use `capTotalPf=-1.000000`, `capValidMask` bit 0, and `errorMask` bit 1. Host tools must treat `-1.000000` as an invalid sentinel, not a physical capacitance value; it must be excluded from heatmap autoscale, statistics, filters, and trend calculations. If every cell is invalid, the firmware still emits the row-major frame with sentinel-filled invalid entries and also emits `MATRIXFDC_DIAG` so host tools can distinguish no-oscillation or status-invalid frames from normal data.

## FDC boot and rescue

Startup performs a visible boot full sweep before normal matrix output. The sweep is row/device based:

- select one S row once
- apply a candidate deglitch/drive/high-current setting to the primary FDC and secondary FDC
- run both FDC2214 devices in CH0-CH3 autoscan
- read primary CH0-CH3 for D1-D4 and secondary CH0-CH3 for D5-D8
- cache the best result per cell from those row reads

Boot sweep failures no longer permanently block normal matrix output. If no valid oscillation is found during boot, the firmware restores CH0-CH3 autoscan and enters the normal matrix loop in degraded mode. Only a path/device failure that prevents both FDC devices from being usable is treated as fatal.

At runtime, rescue is also row based. A pending cell rescue triggers a fast sweep of that cell's containing row, not repeated single-cell route/lock/sweep cycles. Full rescue sweeps all rows. Amplitude warnings enter `warnMask` first and remain usable when raw28 is non-zero, not saturated, watchdog-free, and converts to a plausible frequency. The runtime path classifies amplitude warnings as fresh, stale, or transient. Only persistent fresh warnings from the current row epoch can request a fast sweep; stale and row-switch transient warnings are logged and suppressed.

Useful console commands:

- `force_full_sweep`: queue a full row/device sweep for all rows.
- `force_full_sweep s=5 d=5`: queue a full sweep for the row containing that cell, for example row S5.
- `fdc_diag`: dump STATUS, CONFIG, MUX_CONFIG, IDs, RCOUNT, SETTLECOUNT, CLOCK_DIVIDERS, and DRIVE_CURRENT for both FDC2214 devices.
- `fdc_boot_sweep`: rerun the protected boot sweep synchronously.
- `fdc_rescue`: run synchronous full row/device no-oscillation rescue across the matrix.
- `fdc_period_ms 50`: override the text frame period at runtime. The first-stage default is 50 ms, or 20 fps.
- `fdc_profile summary on|off`: enable or disable `SCAN_TIMING_10`.
- `fdc_profile row on|off`: enable or disable `SCAN_ROW_TIMING`.
- `fdc_profile device on|off`: enable or disable `SCAN_DEVICE_TIMING`.
- `fdc_profile_every <N>`: set the summary interval. Default is 10 frames.
- `fdc_i2c_trace on|off|dump|clear`: control the FDC I2C trace ring. It records transactions without real-time per-register printf.
- `fdc_discard_frames 0|1|2`: runtime experiment for row-switch autoscan discard frames. The production row epoch path defaults to zero discard frames because FDC conversion is restarted with `CONFIG.SLEEP_MODE_EN`.

## FDC matrix row epoch state machine

Each matrix row has a unique conversion epoch. The coordinator sends both FDC
worker tasks into `CONFIG.SLEEP_MODE_EN=1`, waits for both sleep acknowledgments,
switches the TMUX row while both FDC devices are sleeping, waits
`CONFIG_SENSORARRAY_FDC_ROW_SWITCH_SETTLE_US`, applies only changed cached FDC
registers while still sleeping, exits sleep, waits for INTB/STATUS.DRDY, reads
STATUS and DATA, and merges primary D1-D4 with secondary D5-D8.

Primary and secondary workers are permanent tasks. The primary worker only uses
the primary FDC on I2C0; the secondary worker only uses the secondary FDC on
I2C1. If worker initialization fails, the firmware logs the reason and falls
back to a serial sleep-epoch path.

## Known recovery behaviour / Worker core affinity

`CONFIG_SENSORARRAY_FDC_WORKER_TASK_CORE=-1` means no affinity. That value must
not be passed to `xTaskCreateStaticPinnedToCore`; the firmware creates unpinned
static worker tasks for no-affinity mode, and only calls the pinned API with a
normalized core ID in the valid CPU range. On unicore builds the worker core is
resolved to core 0.

Worker creation failures are nonfatal. The firmware logs
`FDC_WORKER_CREATE_FAIL` / `FDC_PARALLEL_FALLBACK` and continues with the serial
sleep-epoch row path, so successful boot cache construction can still lead to
normal `MATRIXFDC_CAP` frames.

Boot full sweep is a startup cache-building phase. After
`FDC_BOOT_MATRIX_SWEEP_DONE` succeeds, repeated boot sweep plus reset usually
means a later panic/assert should be investigated first, especially task core
affinity or worker creation, before changing sweep parameters.

INTB is a data-ready hint only. GPIO ISR handlers record edge count, level,
timestamp, and epoch, then notify the worker task. They do not use I2C and do
not print. The worker still reads STATUS and requires DRDY or the CH0-CH3 unread
mask before DATA is accepted as fresh.

Diff-only cache apply keeps a per-device applied-register shadow. RCOUNT,
SETTLECOUNT, CLOCK_DIVIDERS, DRIVE_CURRENT, MUX_CONFIG, STATUS_CONFIG, and the
CONFIG base are written only when the desired value differs from the last
applied value. Sleep entry/exit writes are separate CONFIG writes and are the
normal per-row restart mechanism; the SD pin and RESET_DEV are not used for row
restart.

This is only an example test setup used to reproduce row-switch transient and
warning behaviour. The firmware must not depend on these specific cells or
external components.

## FDC throughput profiling

The first-stage FDC matrix target is 20 fps:

- `targetFrameUs = 50000`
- `targetRowUs = 6250`

The long-term reference target is 100 fps, but that requires later architecture
work. This revision keeps default output as text `MATRIXFDC_CAP` and does not enable
binary output.

`SCAN_TIMING_10` reports the default 10-frame aggregate: target fps, actual fps,
budget use, overrun count, row timing, sleep-before-row-switch timing, row
settle, diff apply, sleep-exit-to-INTB, worker job timing, dual-bus skew, cache
diff writes, warning counts, INTB counts, and FDC I2C write/read/timeout counts.
`SCAN_BOTTLENECK` is printed immediately on frame overrun with the top timing
contributors. Row and device timing are disabled by default and can be enabled
with the runtime commands.

Runtime FDC register verify defaults to `STARTUP_ONLY`: boot/cache-building paths
can still use readback verification, while the steady matrix loop checks write
`esp_err_t` and skips per-row full readback verify. `FULL` is for debug/bring-up;
`NONE` is for high-speed experiments. The high-speed profile option is disabled
by default because lower RCOUNT/SETTLECOUNT can improve frame rate while raising
frequency noise, stability, and amplitude-warning risk.

The default I2C frequency is 337500 Hz. If this increases NACK, timeout, or retry
counts, return to 325000 Hz before trying 350000, 375000, or 400000 Hz. Summary
bus timing is estimated from configured frequency; confirm limits with a logic
analyzer.

Primary and secondary FDC2214 devices must enter sleep before the matrix task
switches to the next row, and both workers must finish or time out before the
coordinator merges that row. This revision does not introduce I2C DMA, binary
output, or row-parameter reuse based on adjacent rows looking identical.
Cell-specific cache keys remain row/S/D/index/device/channel specific.

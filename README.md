# SensorArray ESP-IDF firmware

This firmware is organized around a formal FDC2214 8x8 matrix read path.

## Architecture boundaries

- `components/fdc2214Cap/`: FDC2214 chip-level driver only. It owns register definitions, I2C register access, reset/ID reads, clock/channel/autoscan/deglitch/drive-current configuration, status reads, raw28 sample reads, and optional raw conversion helpers. It does not know board S/D lines, SW, SELA/SELB, or ADS1263 policy.
- `core/board/`: board mapping single source of truth. It owns row selection meaning, D-line to FDC device/channel mapping, D-line to ADS channel mapping, SELA route to GPIO level mapping, SELB policy, and board-level helpers.
- `core/measure/`: production measurement logic. It prepares the FDC matrix hardware state, enforces ADS/FDC mutual exclusion, reads the 8x8 FDC matrix, builds frames, and emits frames.
- `main/`: thin app orchestration. It initializes board support, TMUX, ADS, both FDC2214 devices, then runs the FDC matrix loop.
- `transport/`: existing wire/USB/BLE transport placeholders. Binary FDC matrix output is reserved for explicit fast-speed mode only.

Temporary single-point and FDC discovery bring-up debug entry points have been removed from the default firmware.

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

Default output is one printf text line per frame:

`MATRIXFDC,seq=<sequence>,timestampUs=<timestampUs>,unit=freqHz,validMask=0x<16hex>,warnMask=0x<16hex>,errorMask=0x<16hex>,freqHz=[<64 Hz values>]`

When `CONFIG_SENSORARRAY_FDC_EMIT_CAP_TOTAL_PF=y`, the same text line includes
`capValidMask` and `capTotalPf[64]`:

`MATRIXFDC,seq=<sequence>,timestampUs=<timestampUs>,unit=freqHz+capTotalPf,validMask=0x<16hex>,warnMask=0x<16hex>,errorMask=0x<16hex>,capValidMask=0x<16hex>,freqHz=[<64 Hz values>],capTotalPf=[<64 pF values>]`

`capTotalPf` is computed from the same `freqHz[64]` values with
`C = 1 / ((2*pi*f)^2 * L)`, using `CONFIG_SENSORARRAY_FDC_TANK_INDUCTOR_NH`
(`18000` nH by default). It adds no FDC I2C read, no sweep, and no extra ready
wait. It is the total equivalent LC tank capacitance, including sensor,
inductor parasitics, FDC input parasitics, TMUX parasitics, PCB/FPC/cable
parasitics, and fixture capacitance. It is not a parasitic-subtracted pure
sensor capacitance. Future delta reporting should use a per-cell baseline:
`capDeltaPf = capTotalPf - capBaselinePf`.

Example:

`MATRIXFDC,seq=12,timestampUs=345678901,unit=freqHz,validMask=0xFFFFFFFFFFFFFFFF,warnMask=0x0000000000000000,errorMask=0x0000000000000000,freqHz=[2746569.0,2746501.0,...]`

Raw FDC2214 codes are emitted only as a separate debug line:

`DEBUGFDC_RAW,seq=12,timestampUs=345678901,raw28=[9215955,9215731,...]`

Binary output is not enabled by default. `sensorarrayFastSpeedIsEnabled()` currently defaults false; the binary sender is reserved and returns `ESP_ERR_NOT_SUPPORTED` until an explicit host fast-speed/binary command path is added.

`MATRIXFDC` remains the normal periodic output, even for degraded frames. If every cell is invalid, the firmware still emits the row-major `MATRIXFDC` frame with zeroed invalid `freqHz` entries and also emits `MATRIXFDC_DIAG` so host tools can distinguish no-oscillation or status-invalid frames from normal data.

## FDC boot and rescue

Startup performs a visible boot full sweep before normal matrix output. The sweep is row/device based:

- select one S row once
- apply a candidate deglitch/drive/high-current setting to the primary FDC and secondary FDC
- run both FDC2214 devices in CH0-CH3 autoscan
- read primary CH0-CH3 for D1-D4 and secondary CH0-CH3 for D5-D8
- cache the best result per cell from those row reads

Boot sweep failures no longer permanently block normal matrix output. If no valid oscillation is found during boot, the firmware restores CH0-CH3 autoscan and enters the normal matrix loop in degraded mode. Only a path/device failure that prevents both FDC devices from being usable is treated as fatal.

At runtime, rescue is also row based. A pending cell rescue triggers a fast sweep of that cell's containing row, not repeated single-cell route/lock/sweep cycles. Full rescue sweeps all rows. Amplitude warnings enter `warnMask` first and remain usable when raw28 is non-zero, not saturated, watchdog-free, and converts to a plausible frequency; repeated warnings can still trigger drive-current rescue.

Useful console commands:

- `force_full_sweep`: queue a full row/device sweep for all rows.
- `force_full_sweep s=5 d=5`: queue a full sweep for the row containing that cell, for example row S5.
- `fdc_diag`: dump STATUS, CONFIG, MUX_CONFIG, IDs, RCOUNT, SETTLECOUNT, CLOCK_DIVIDERS, and DRIVE_CURRENT for both FDC2214 devices.
- `fdc_boot_sweep`: rerun the protected boot sweep synchronously.
- `fdc_rescue`: run synchronous full row/device no-oscillation rescue across the matrix.
- `fdc_period_ms 50`: override the text frame period at runtime. The first-stage default is 50 ms, or 20 fps.
- `fdc_profile summary on|off`: enable or disable `SCAN_TIMING_SUMMARY`.
- `fdc_profile row on|off`: enable or disable `SCAN_ROW_TIMING`.
- `fdc_profile device on|off`: enable or disable `SCAN_DEVICE_TIMING`.
- `fdc_profile_every <N>`: set the summary interval. Default is 10 frames.
- `fdc_i2c_trace on|off|dump|clear`: control the FDC I2C trace ring. It records transactions without real-time per-register printf.
- `fdc_discard_frames 0|1|2`: runtime experiment for row-switch autoscan discard frames.

## FDC throughput profiling

The first-stage FDC matrix target is 20 fps:

- `targetFrameUs = 50000`
- `targetRowUs = 6250`

The long-term reference target is 100 fps, but that requires later architecture
work. This revision keeps default output as text `MATRIXFDC` and does not enable
binary output.

`SCAN_TIMING_SUMMARY` reports target fps, actual fps, budget use, overrun,
row min/max/average, cache apply breakdown, discard/wait/read timing, cap compute
time, and FDC I2C write/read/verify/retry/NACK/timeout counts. Row and device
timing are disabled by default and can be enabled with the runtime commands.

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

Primary and secondary FDC2214 devices must finish the current row before the
matrix task switches to the next row. This revision logs `FDC_PARALLEL_CFG` and
keeps a single matrix task; it does not introduce I2C DMA, binary output, or
row-parameter reuse based on adjacent rows looking identical. Cell-specific cache
keys remain row/S/D/index/device/channel specific.

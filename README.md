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
- `raw28[64]`: primary payload
- `validMask`: bit `i` means `raw28[i]` is valid
- `errorMask`: bit `i` means that cell failed, timed out, or had invalid status/data

Frame order is row-major:

`[S1D1,S1D2,S1D3,S1D4,S1D5,S1D6,S1D7,S1D8,S2D1,...,S8D8]`

Index formula:

`index = (sIndex - 1) * 8 + (dIndex - 1)`

## FDC matrix hardware state

Before each FDC matrix frame, `sensorarrayMeasurePrepareFdcMatrixPath()` enforces:

- SW physical high
- ADS1263 internal reference off
- ADS1263 VBIAS off
- ADS conversion stopped
- SELA routed to FDC2214 via board-map GPIO mapping
- SELB set through the board-map FDC policy

SW physical high is controlled by `sensorarrayMeasureSetSwPhysicalLevel()`. It only controls the SW GPIO physical level and does not enable ADS internal reference or VBIAS.

## Output policy

Default output is one printf text line per frame:

`MATRIXFDC,seq=<sequence>,timestampUs=<timestampUs>,validMask=0x<16hex>,errorMask=0x<16hex>,raw28=[<64 uint32 values>]`

Example:

`MATRIXFDC,seq=12,timestampUs=345678901,validMask=0xFFFFFFFFFFFFFFFF,errorMask=0x0000000000000000,raw28=[123,456,789,...]`

Binary output is not enabled by default. `sensorarrayFastSpeedIsEnabled()` currently defaults false; the binary sender is reserved and returns `ESP_ERR_NOT_SUPPORTED` until an explicit host fast-speed/binary command path is added.

`MATRIXFDC` is now emitted only when at least one `raw28` entry is semantically valid. If every cell is invalid and all raw values are zero, the firmware emits `MATRIXFDC_DIAG` and suppresses the normal frame so host tools do not treat a no-oscillation condition as a valid measurement.

## FDC boot and rescue

Startup performs a required visible boot full sweep before normal matrix output. The protected boot points include S5D5 and S1D1, with S5D5 treated as the current 200 pF validation point. S5D5 with 200 pF is expected near 2.4-2.6 MHz, but that range is a diagnostic hint, not a hard-coded validity gate.

At runtime, direct reads and fast sweeps are tried first from cached lock settings. A full sweep is used when fast/direct recovery fails or when no oscillation is detected. An all-invalid all-zero matrix frame triggers immediate no-oscillation rescue and register dumps instead of continuing to print normal all-zero frames.

Useful console commands:

- `force_full_sweep`: queue a full sweep for all cells.
- `force_full_sweep s=5 d=5`: queue a full sweep for one cell, for example S5D5.
- `fdc_diag`: dump STATUS, CONFIG, MUX_CONFIG, IDs, RCOUNT, SETTLECOUNT, CLOCK_DIVIDERS, and DRIVE_CURRENT for both FDC2214 devices.
- `fdc_boot_sweep`: rerun the protected boot sweep synchronously.
- `fdc_rescue`: run synchronous full no-oscillation rescue across the matrix.
- `fdc_period_ms 50`: override the text frame period at runtime without changing the default 250 ms configuration.

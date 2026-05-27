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

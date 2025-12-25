# matrixEngine

Matrix scan helper for SensorArray. It coordinates row select (TMUX1108),
column-group select (TMUX1134), and measurement via ADS126x and/or FDC2214.
The main API is a single call that reads or writes a rectangular region.

## Default hardware assumptions

These defaults are used if you do not provide custom callbacks or mappings:

- Rows: TMUX1108 A0/A1/A2 select row 0..7 (S1..S8).
- Col groups: TMUX1134 selects a group of 4 columns.
  - Group 0 -> SELA, group 1 -> SELB.
  - For cap reads, the TMUX1134 path is driven to the C side.
  - For voltage/resistance reads, the TMUX1134 path is driven to the R side.
- ADS126x:
  - R1..R8 map to AIN7..AIN0.
  - MUXN defaults to AINCOM.
- FDC2214:
  - C1..C4 on device 0, C5..C8 on device 1.
  - Channel defaults to col % 4 if not overridden.

If your wiring differs, provide custom callbacks and/or mapping arrays in
matrixEngineConfig_t.

## API summary

Header: `core/matrixEngine/include/matrixEngine.h`

- `matrixEngineInit(const matrixEngineConfig_t *cfg)`
  - Stores configuration, installs default callbacks if missing, and creates
    a mutex.
  - Calls `tmuxSwitchInit()` if default row/col/drive callbacks are used.
- `matrixEngineRegionIo(...)`
  - Single entry point for region read or write.
  - Row-major ordering for output values.
  - Uses a mutex to serialize access.
- `matrixEngineDeinit()`
  - Releases internal resources.

## Configuration

Key fields in `matrixEngineConfig_t`:

- `adc`: ADS126x handle (required for voltage/resistance reads).
- `cap`: Single FDC2214 handle (optional if using `capDevices`).
- `capDevices` + `capDeviceCount`: Use when you have multiple FDC2214s.
- `adcMuxPByCol` / `adcMuxNByCol`: Optional explicit MUX tables per column.
- `capDeviceIndexByCol` / `capChannelByCol`: Optional explicit cap mapping.
- `colGroupSize`: Defaults to 4 (matches TMUX1134 grouping).
- `oversample`: Defaults to `CONFIG_MATRIX_OVERSAMPLE` (>= 1).
- `resistanceRefOhms` / `resistanceExcitationUv`: Used for resistance math.
- `selectRow`, `selectColGroup`, `drive`: Override if wiring differs.

## Example

```c
static Fdc2214CapDevice_t *caps[] = { cap0, cap1 };

matrixEngineConfig_t cfg = {
    .adc = &adc,
    .capDevices = caps,
    .capDeviceCount = 2,
    .colGroupSize = 4,
    .resistanceRefOhms = 10000,
    .resistanceExcitationUv = 3300000,
};

matrixEngineInit(&cfg);

matrixEngineRegion_t region = { .row = 0, .col = 0, .rows = 1, .cols = 4 };
matrixEngineRequest_t req = {
    .io = MATRIX_ENGINE_IO_READ,
    .measure = MATRIX_ENGINE_MEASURE_VOLTAGE_UV,
};

int32_t values[4] = {0};
matrixEngineRegionIo(&region, &req, values, 4);
```

## Notes

- ADS126x must be initialized and ADC1 conversions started before reads.
- FDC2214 devices must be configured before cap reads.
- Resistance uses a divider model:
  `R_unknown = R_ref * Vmeas / (Vexc - Vmeas)`, output is in mOhm.
- If you want to read battery voltage on AIN8, call the ADS126x driver
  directly or add a small helper; the default matrix mapping does not use
  AIN8/AIN9.

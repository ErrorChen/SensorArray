# main component

`main/main.c` is the SensorArray top-level lifecycle scheduler. It owns the
visible application lifecycle and keeps measurement implementation details out
of the main component:

- initialize board support and I2C buses
- initialize TMUX control GPIOs
- initialize ADS with internal reference and VBIAS off by default
- initialize primary and secondary FDC2214 devices for CH0-CH3 autoscan
- build the default 8x8 FDC scan plan
- run boot FDC sweep/calibration
- enter the production matrix loop or diagnostic/safe idle path
- emit each frame through `sensorarrayFrameOutputPrint()`
- run the FDC rescue tick

`main/sensorarrayApp.c` and `main/sensorarrayApp.h` were removed; the normal
entry point is `main/main.c`.

Measurement code now lives under `core/measure`:

- `core/measure/fdc`: FDC matrix readout, sweep/cache, and rescue control
- `core/measure/ads`: ADS matrix measurement entry points
- `core/measure/mixed`: mixed-row scheduling entry points
- `core/measure/include`: public measurement/frame/scan-plan headers

Frame text output lives under `main/output`. `components/fdc2214Cap` and
`components/ads126xAdc` remain low-level chip drivers; they do not know matrix
rows, frames, or rescue policy. `boardSupport` owns board resource lifecycle and
legacy I2C install/delete/transaction/recovery protection.

The default app mode is no longer a single-point debug path. Legacy single-point, FDC discovery, locked-sample, and register-dump bring-up entries have been removed.

Default frame output remains human-readable printf text. The main payload is total LC tank capacitance in pF:

`MATRIXFDC_CAP,seq=<sequence>,timestampUs=<timestampUs>,partial=<0|1>,frameQuality=<full|partial>,capValidMask=0x<16hex>,freshMask=0x<16hex>,warnMask=0x<16hex>,errorMask=0x<16hex>,invalidSentinel=-1.000000,capTotalPf=[<64 values>]`

Invalid cells are emitted as `capTotalPf=-1.000000` with the corresponding `capValidMask` bit cleared and `errorMask` bit set. Host software should treat that value as an invalid sentinel, not as a measured `-1 pF` capacitance.

Frequency output is optional debug text and is emitted separately:

`MATRIXFDC_FREQ,seq=<sequence>,timestampUs=<timestampUs>,validMask=0x<16hex>,warnMask=0x<16hex>,errorMask=0x<16hex>,freqHz=[<64 values>]`

Raw FDC2214 codes are debug-only:

`DEBUGFDC_RAW,seq=<sequence>,timestampUs=<timestampUs>,raw28=[<64 values>]`

Binary output is reserved for an explicit fast-speed/binary host command path and is disabled by default.

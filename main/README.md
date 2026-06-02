# main component

`main/main.c` is intentionally thin and only calls `sensorarrayAppRun()`.

`main/sensorarrayApp.c` performs application orchestration:

- initialize board support and I2C buses
- initialize TMUX control GPIOs
- initialize ADS with internal reference and VBIAS off by default
- initialize primary and secondary FDC2214 devices for CH0-CH3 autoscan
- enter the production FDC matrix loop
- emit each frame through `sensorarrayFdcMatrixEmitFrame()`

The default app mode is no longer a single-point debug path. Legacy single-point, FDC discovery, locked-sample, and register-dump bring-up entries have been removed.

Default frame output remains human-readable printf text. The main payload is total LC tank capacitance in pF:

`MATRIXFDC_CAP,seq=<sequence>,timestampUs=<timestampUs>,capValidMask=0x<16hex>,warnMask=0x<16hex>,errorMask=0x<16hex>,capTotalPf=[<64 values>]`

Frequency output is optional debug text and is emitted separately:

`MATRIXFDC_FREQ,seq=<sequence>,timestampUs=<timestampUs>,validMask=0x<16hex>,warnMask=0x<16hex>,errorMask=0x<16hex>,freqHz=[<64 values>]`

Raw FDC2214 codes are debug-only:

`DEBUGFDC_RAW,seq=<sequence>,timestampUs=<timestampUs>,raw28=[<64 values>]`

Binary output is reserved for an explicit fast-speed/binary host command path and is disabled by default.

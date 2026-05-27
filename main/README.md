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

Default frame output remains human-readable printf text:

`MATRIXFDC,seq=<sequence>,timestampUs=<timestampUs>,validMask=0x<16hex>,errorMask=0x<16hex>,raw28=[<64 values>]`

Binary output is reserved for an explicit fast-speed/binary host command path and is disabled by default.

# transportWire / wired transport placeholder

## 中文说明

`transport/transportWire` 当前是有线传输占位模块。`transportWire.h` 还没有 stable public API，`transportWire.c` 只保留实现占位并明确不定义 `app_main()`。

预期职责是 UART/USB-CDC port writes、TX queue/backpressure 和 recovery，并可能承载 `protocolWire` 或 `protocolUsb` payload。当前默认 SensorArray frame 输出不经过该模块。

## Australian English Documentation

`transport/transportWire` is currently a wired-transport placeholder. `transportWire.h` has no stable public API yet, and `transportWire.c` only keeps an implementation placeholder and explicitly does not define `app_main()`.

The intended role is UART/USB-CDC port writing, TX queue/backpressure, and recovery, potentially carrying `protocolWire` or `protocolUsb` payloads. Current SensorArray frame output does not use this module.

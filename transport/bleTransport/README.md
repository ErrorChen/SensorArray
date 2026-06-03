# bleTransport / BLE 传输占位层

## 中文说明

`transport/bleTransport` 当前是占位模块。`bleTransport.h` 还没有 stable public API，`bleTransport.c` 只保留实现占位并明确不定义 `app_main()`。

预期职责是 BLE connection、MTU 分片、notification pacing 和 reconnect policy。当前默认 SensorArray frame 输出仍是 `main/output` 的 printf text，不经过 BLE transport。

## Australian English Documentation

`transport/bleTransport` is currently a placeholder. `bleTransport.h` has no stable public API yet, and `bleTransport.c` only keeps an implementation placeholder and explicitly does not define `app_main()`.

The intended role is BLE connection handling, MTU chunking, notification pacing, and reconnect policy. Current SensorArray frame output defaults to printf text from `main/output`, not BLE transport.

# 有线传输配置占位组件 / Wired transport configuration placeholder

此 ESP-IDF 组件注册目录与 UART、可选 USB-CDC 的 Kconfig 项。`transportWire.c` 和公共头文件中没有函数或运行时状态。要发送实际数据或接收控制命令，请使用 core/transport 的正式传输层。 / This ESP-IDF component registers a source directory and Kconfig entries for UART and optional USB-CDC. Its C file and public header define no functions or runtime state. To send data or receive commands, use the runtime transport layer. [组件注册 / Registration](CMakeLists.txt) · [实现 / Source](transportWire.c) · [头文件 / Header](include/transportWire.h) · [运行时传输 / Runtime transport](../../core/transport/)

| 配置 / Option | 默认 / Default | 用途 / Meaning |
|---|---|---|
| `WIRE_UART_ENABLE` | y | 有线功能配置中的 UART 项 / UART selection under wired transport |
| `WIRE_UART_PORT` | 1 | UART 外设编号 / UART peripheral number |
| `WIRE_UART_TX_GPIO`, `WIRE_UART_RX_GPIO` | 17, 18 | 索引所给的收发管脚 / Configured UART TX/RX pins |
| `WIRE_UART_BAUD` | 2000000 | UART 线速率 / UART line rate |
| `WIRE_USB_CDC_ENABLE` | n | 可选 CDC 配置 / Optional CDC selection |

UART 参数依赖 `WIRE_UART_ENABLE`，有线开关依赖 `SENSORARRAY_ENABLE_WIRED`。正式 USB-OTG 后端有自己的构建选项、设备描述符和单一 I/O owner。 / UART settings depend on WIRE_UART_ENABLE. The wired enable options depend on SENSORARRAY_ENABLE_WIRED. The active USB-OTG backend has its own build choices, descriptors and I/O owner. [此组件选项 / Placeholder Kconfig](Kconfig) · [USB-OTG](../../components/sensorarrayUsbOtg/README.md)

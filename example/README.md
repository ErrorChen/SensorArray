# BLE 吞吐示例 / BLE throughput example

这是 Espressif 独立 GATT 示例源码。它使用自己的 app_main、profile 与配套 throughput_client。生产工程使用 components/sensorarrayBle 和 main。若复现实验，把示例作为 ESP-IDF 吞吐示例应用的入口，并在该应用选择相应 EXAMPLE 吞吐方向。 / This stand-alone Espressif GATT server uses the companion throughput client and its own entry point. Configure the server notification or client write experiment in that example application. Values measure BLE payload throughput.

| 函数签名 / Signature | 调用、参数、结果 / Caller, inputs and effects |
|---|---|
| `void app_main(void)` | IDF 入口，无参数；初始化 NVS、BT 控制器、Bluedroid、GAP/GATTS 回调和 profile。 / IDF bootstrap initializes the Bluetooth stack, registers callbacks and starts the selected benchmark task. |
| `uint8_t check_sum(uint8_t *addr, uint16_t count)` | 模块内使用。对 addr 的 count 字节做进位折叠并逐位取反；空输入返回零。 / Computes the folded one's-complement byte for the test payload. |
| `void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)` | GAP 调用，按 event 读取对应 param 成员，处理广播配置、开始及连接参数。 / GAP callback dispatches advertising and connection events using the tagged parameter union. |
| `void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)` | SDK 回调；登记接口并将事件转发到匹配 profile。 / Registers the GATT interface and dispatches the SDK event and parameter union to matching profiles. |
| `void gatts_profile_a_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)` | Profile A 处理服务/属性、连接、读写与拥塞，借助接口号向 SDK 回复。 / Profile handler creates attributes, handles reads/writes and updates connection and congestion state. |
| `void example_write_event_env(esp_gatt_if_t gatts_if, prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param)` | 写事件传入接口、可变暂存区及 SDK write 参数。核对 offset/len，回复并保存需要的分段。 / Checks a prepared write's bounds, replies as requested and buffers the provided fragment. |
| `void example_exec_write_event_env(prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param)` | Execute/cancel 事件结束暂存写并释放资源。 / Finishes the pending write according to the execute flag and releases the buffer. |
| `void throughput_server_task(void *param)` | 通知吞吐任务；param 未使用。根据连接、信号量和可发 packet 数发送带 checksum 的数据。 / RTOS task sends test notifications when connected and stack capacity permits. |
| `void throughput_cal_task(void *param)` | 写吞吐任务；param 未使用。按累计字节与微秒计时计算 byte/s 和 bit/s。 / Reports client-write throughput from bytes and elapsed microseconds. |

[示例源码 / Source](example_ble_server_throughput.c) · [生产 BLE 组件 / Firmware BLE component](../components/sensorarrayBle/README.md)

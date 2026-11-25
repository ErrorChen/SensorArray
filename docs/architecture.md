# 固件架构 / Firmware architecture

采集核心持有活动路由、前端状态和帧边界。控制与传输任务接收请求、发布不可变记录并按 sink 隔离背压。 / The acquisition core owns routing, frontends and frame boundaries. Control and transport tasks enqueue requests and deliver immutable records under independent backpressure.

```text
CDC / BLE FF10 / Wi-Fi control → command handler → bounded mailbox
                                                        ↓
                           frame boundary → route owner → FDC A+B / ADS ADC1
                                                               ↓
                           configuration snapshot + cells → frame encoder
                                                               ↓
                                   USB formal | BLE | Wi-Fi | diagnostic log
```

| 所有者 / Owner | 数据、同步与超时 / Data, synchronization and timeout |
|---|---|
| Core 1 scan | 工作者领取带 epoch 的持久行槽；两颗 FDC 并行。超时后须由 worker 确认释放或隔离。 / Both FDC workers use persistent epoch-tagged slots. Reuse requires release evidence. |
| ADS driver | 同一事务中串行 SPI、DRDY 与读回，保留 DMA 描述符直到回收。reference、MUX、滤波器及转换新鲜度分别判断。 / The ADC owner retains queued descriptors until reclaimed and tracks reference, input, filter and conversion readiness separately. |
| Mode/config | 请求 ID 跨控制通路仲裁。帧开始取不可变快照；排队数据沿用旧身份。 / One arbiter orders requests. Headers, trailers and descriptors use the capture-time identity. |
| Transport / BLE | 优先回复与保护帧有独立容量。slot 有代数和所有者，拥塞和过期分别计数。 / Reserved queues prioritize control and complete snapshots. Generation checks prevent stale slot release. |
| Wi-Fi SDK | 拷贝入持久 slot → owner 出队 → 释放锁执行 SDK → 回调统计。quarantine 保留尚未返回的对象；观察任务只计时并锁存状态。 / Allocation, SDK work and completion callbacks occur outside the metadata lock. A watchdog observes overruns while unresolved ownership stays fenced. |
| USB / logging | CDC 正式数据与调试分别准入；只提交能完整容纳的记录。统计 driver 接受、队列过期及 host 实收。 / Formal records and expendable debug output have distinct admission rules. Host receipts supply the end-to-end measure. |

GPIO 回读、寄存器回读和物理节点是三个观察层。SAFE 保留 requested/applied/known/verified 和独立的 cleanupError。受控重启的预算依赖真实采集进展。 / GPIO, device registers and analog nodes have different observability. SAFE reports state and cleanup evidence. Recovery budgets rely on actual acquisition progress.

[启动与任务 / Application](../main/README.md) · [路由和采集 / Measurement](../core/measure/README.md) · [输出 / Output](../main/output/README.md) · [传输 / Transport](../core/transport/README.md)

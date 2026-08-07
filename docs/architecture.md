# 软件架构

## 设计目标

固件把硬件采集与输出传输分成两个异步域：

```text
Serial / BLE FF10 / Wi-Fi CTRL
              |
              v
      共享 command handler                 Core 0
              |
              v
      有界 CommandMailbox
              |
              v
完整帧边界 -> mode/route owner -> FDC 或 ADS -> frame builder   Core 1
                                           |
                                           v
                               TextFrameBus / EventRing
                                           |
                         +-----------------+----------------+
                         v                 v                v
                      USB sink      transport DATA/LOG   async log
                                         |
                                  BLE / Wi-Fi sinks          Core 0
```

核心原则：

- Core 1 独占测量模式、矩阵路由与 ADS/FDC acquisition；
- BLE callback、Serial control task 和 Wi-Fi control task 不能直接改硬件；
- 一帧只属于一个 mode/generation/request ID；
- DATA/LOG 发送有界且非阻塞，网络拥塞不能反压 acquisition；
- control reply 使用独立路径，不与 measurement DATA 争抢通用 transport payload slot。

## 模块职责

| 模块 | 职责 | 不应承担 |
| --- | --- | --- |
| `core/board` | 引脚、D-line、SELA/SELB、SW、matrix REF 与 ADS profile 真源 | 文本协议与网络策略 |
| `core/boardSupport` | I2C、SPI、GPIO、中断服务与资源生命周期 | mode 业务语义 |
| `core/measure` | mode 状态机、安全 transition、FDC/ADS frame 语义 | BLE/Wi-Fi 发送 |
| `core/measure/fdc` | 双 FDC worker、row epoch、freshness、cache/rescue | ADS 矩阵算法 |
| `core/measure/ads` | ADS 矩阵、rail、battery、math、PGA/autorange | transport 选择 |
| `core/config` | active/pending rows 与 generation | 直接执行 row scan |
| `core/transport` | DATA/LOG sink 选择、slot/descriptor、统计与 control reply | 改变模拟路由 |
| `components/sensorarrayBle` | GATT、CCCD、分片、BLE slot/descriptor 与连接生命周期 | 解析业务命令 |
| `components/sensorarrayWifi` | SoftAP、UDP socket 与 peer reply | STA 配网持久化 |
| `main/control` | 跨 transport 的命令 mailbox | 在 Core 0 应用硬件命令 |
| `main/output` | 文本格式、异步日志、USB sink | 改变采集时序 |
| `main/main.c` | 生命周期、帧边界 apply、engine 调度 | 重复芯片 driver 逻辑 |

## 命令生命周期

命令按以下优先级解析：

1. `sensorarrayScanConfigHandleCommand()`：`ROWS*`；
2. transport runtime handler：`TX/ST/BTX/WIFI`；
3. main runtime query callback：`MODE/STATE/RAIL/CELL/BAT/ADS/...`；
4. legacy mailbox callback：兼容性与低频诊断命令。

查询通常立即读取 snapshot。会改变采集或诊断状态的命令先返回 accepted，再由 Core 1 在完整帧边界处理。CommandMailbox 满时保留最新 operator intent，不允许阻塞 BLE callback。

## Transport payload 所有权

通用 DATA/LOG transport 使用预分配静态 payload pool 和小 descriptor queue：

- 4 个 payload slot；
- DATA descriptor queue 深度 2；
- LOG descriptor queue 深度 2；
- consumer 同时可独占 1 个 slot；
- consumer 始终先取 DATA；
- LOG 同时占用的 payload slot 硬上限为 2，因此 4-slot pool 始终至少为 measurement DATA 保留 2 个 slot；
- enqueue 为 `xQueueSend(..., 0)`，pool/queue 满时按 channel 统计并丢弃，不等待网络。

producer 在短临界区内只分配 `inUse/generation`，1536-byte copy 和 metadata 解析在临界区外完成。consumer 校验 slot index、generation、length 与 ownership，发送完毕后释放。禁止在 hot path 使用 `malloc(1536)`。

BLE 组件内部继续使用自己的 fixed slot + descriptor queue；通用 transport 不复制或破坏 BLE 的连接 generation 所有权模型。

## Async logger workspace

`sensorarrayLogTask` 使用一个 task-owned 静态 workspace 保存 summary、frame 与 text packet scratch。只有该 task 可以直接访问 workspace；按对象生命周期 reset，不能每帧 `memset` 整个 workspace。这样可以避免多个 1536-byte packet 和 frame 同时常驻 task stack。

## Backpressure 与优先级

```text
control reply > measurement DATA > diagnostic LOG
```

- control reply 使用 BLE CTRL slot 或源端 peer reply；
- DATA/LOG 使用通用 transport pool，但 DATA 有独立 queue 并优先消费；
- BLE/Wi-Fi/USB 各自统计 sent/drop；
- disconnect 会清订阅、递增 connection generation，并 drain BLE queues；
- stale descriptor、release mismatch 与 CRC mismatch 必须计数，不能静默忽略。

## 状态一致性

mode request 的 accepted 与 applied 是两个事件。Core 1 transition 会先撤销激励、停止 conversion、切换前端、配置并 readback，再更新 generation。失败时进入 SAFE/DEGRADED，不发布伪造的正常帧。

`MODE?`/`STATE?` 读取不可变 snapshot，不触碰硬件；host 在切换后必须丢弃旧 generation 数据。

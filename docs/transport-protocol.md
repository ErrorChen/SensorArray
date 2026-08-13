# Transport 协议

## 通道模型

固件区分四类逻辑通道：

| Channel | 内容 | 优先级 |
| --- | --- | ---: |
| CTRL | 命令与 source-local reply | 最高；独立路径 |
| DATA | CAP/VOLT/RES measurement message | 高 |
| LOG | summary、health、diagnostic event | 低 |
| LIFECYCLE | MAPP/RAPP/BAPP/RMAPP/MERR/MFAULT 等 deferred protocol event | 高于 LOG，低于 DATA |

同一条控制命令在 Serial、BLE `FF10` 和 Wi-Fi UDP `3335` 进入 `sensorarrayTransportHandleControlCommand()`。回复回到命令来源，不广播到其他 transport。

## Transport 映射

| Transport | DATA | LOG | CTRL RX | CTRL TX |
| --- | --- | --- | --- | --- |
| Serial | USB Serial/JTAG 文本 | USB Serial/JTAG 文本 | 行输入 | 同一 Serial |
| BLE | `FF20` | `FF30` | `FF10` Write / Write Without Response | `FF11` Notify/Indicate |
| Wi-Fi SoftAP | UDP `3333` | UDP `3334` | UDP `3335` | 回复原 UDP peer |

DATA/LOG 最大完整 message 为 1536 bytes，BLE CTRL TX 最大完整 reply 为 512 bytes，BLE CTRL RX command 上限为 128 bytes。Serial 与 Wi-Fi 收到完整文本 message；BLE 根据 ATT payload 决定 raw 或 `G,...` 分片，详见 [BLE 协议](ble-protocol.md)。

## 非阻塞所有权模型

通用网络 transport 不在 FreeRTOS queue 中复制 1536-byte object，而使用：

```text
5 个静态 payload slots（4 个普通 slot + 1 个 lifecycle 保留 slot）
        +
DATA descriptor queue（depth=2）
LIFECYCLE descriptor queue（depth=1，保留 slot）
LOG descriptor queue（depth=2）
```

4 个普通 slot 覆盖旧设计“最多 3 个排队 payload 加 1 个 consumer-owned payload”的峰值容量，第 5 个 slot 专供 lifecycle descriptor。descriptor 只携带 slot index、channel、length 与 slot generation；payload 与 frame metadata 只保留一份。LOG 同时占用的普通 slot 硬上限为 2，因此即使两条 compact summary LOG 合法排队，也不会挤占 DATA 或 lifecycle。

producer：

```text
短临界区：找 free slot -> inUse=1 -> generation++
临界区外：copy payload -> 填 metadata
xQueueSend(..., 0)
失败：release slot -> channel drop++ -> 返回 bounded error
```

consumer：

```text
    优先收 DATA descriptor，再收 LIFECYCLE，最后收 LOG descriptor
校验 index / inUse / generation / length
发送到当前 eligible sinks
release slot
```

设计结果：

- acquisition/logger 不等待 BLE/Wi-Fi；
- LOG 同时占用的普通 slot 不超过 2 个，lifecycle 有独立 reserved slot；
- DATA queue 始终先消费；
- measurement DATA 不会被 `SF50/ADS50/...` LOG 全部挤出；
- stale descriptor、double/mismatched release 和 pool full 都有统计；
- hot path 不进行 heap allocation。

## `ST` 的精确语义

BLE 在 producer 入队前同时检查“stream wants BLE”和 channel 当前可发送。Wi-Fi 的 AUTO 与显式选择略有不同，见下表。

| Mode | Serial | BLE DATA | BLE LOG | Wi-Fi DATA/LOG |
| --- | --- | --- | --- | --- |
| `ST=AUTO` | 始终 eligible | connected + `FF20` CCCD 与 TX mode 兼容 | connected + `FF30` CCCD 与 TX mode 兼容 | Wi-Fi mode 非 OFF + ready |
| `ST=SER` | 是 | 否 | 否 | 否 |
| `ST=BLE` | 否 | 同上 | 同上 | 否 |
| `ST=WIFI` | 否 | 否 | 否 | producer eligibility 只要求 Wi-Fi mode 非 OFF；consumer 发送前再检查 ready |
| `ST=ALL` | 是 | 同上 | 同上 | producer eligibility 同 `ST=WIFI`；consumer 发送前再检查 ready |

关键约束：

- 只订阅 `FF20` 只能使 DATA eligible，不能让 LOG 进入 BLE transport path；
- 只订阅 `FF30` 只能使 LOG eligible；
- `ST=BLE`/`ST=ALL` 也不能绕过 connection 与 channel-specific CCCD；
- AUTO 下未 ready 的 Wi-Fi 不分配通用 payload slot；
- 显式 `ST=WIFI`/`ST=ALL` 在 Wi-Fi mode 非 OFF 但 backend 暂未 ready 时仍可能入队，consumer 会在发送前跳过该 Wi-Fi sink。显式 stream selection 不是 backend-ready 保证，应用应先用 `WIFI?` 检查 `ok=1`。

## `TX` 的精确语义

| Mode | 当前行为 |
| --- | --- |
| `TX=SHORT` / `TX=RT` | 对可解析的 BLE CAP DATA 生成 `B20`；V/R 和其他 transport 保持完整 message |
| `TX=REL` | 完整 message |
| `TX=FULL` | 完整 message；当前与 REL 的 wire payload 相同，保留兼容名称 |

`TX` 不是 BLE Notify/Indicate 选择；后者由 `BTX=FAST|SAFE` 控制。

## CTRL 路径

CTRL 不使用 DATA/LOG transport pool：

- Serial control task 读取并规范化 ASCII 行；
- BLE callback 只复制至小型 CTRL RX queue，专用 BLE control task 再调用共享 handler；
- Wi-Fi control task 保存 source peer，以便定向回复；
- BLE CTRL TX 使用 2 个独立 512-byte slots 和高优先级 descriptor queue。

因此 DATA/LOG 拥塞不能饿死正常的 `STATE?` reply。BLE 客户端仍需订阅 `FF11`。

需要区分两级 pool：

| 层级 | DATA | LOG | CTRL |
| --- | --- | --- | --- |
| Core transport | 5 个共享 1536-byte slot；其中 lifecycle 专用 1 个，LOG 普通 slot 最多同时占 2 个 | 同左 | 独立 source-local 路径 |
| BLE component | 4 个独立 1536-byte slot | 2 个独立 1536-byte slot | 2 个独立 512-byte slot |

Core transport 的共享 5-slot pool 负责 network sink 排队、DATA reserve 与 lifecycle reserve；BLE 收到 eligible payload 后复制到自己的 `DATA 4 + LOG 2` pool，再把小 descriptor 放入容量 6 的普通 TX FIFO。CTRL 使用另一条优先 queue，所以这里的 BLE `4 + 2` 不能误写成“共享 6 个任意 channel slot”。

BLE CTRL TX 仅在 reply 尚未被 Bluedroid 接受时进行 bounded retry：每 10 ms 一次，总窗口 500 ms，并且只重试 `ESP_FAIL`。其他 error 不重试。SAFE 下 `send_indicate(..., confirm=true)` 一旦返回 `ESP_OK`，reply 已被接受；随后等待 confirmation 即使 timeout 也不重发，以免一个 setter 的 source-local reply 在客户端重复出现。该机制不重新执行 command，只重试相同 reply bytes；DATA/LOG 仍保持单次、非阻塞发送。

## Drop 与 telemetry

至少应观察：

- `transportSlotAllocFail`；
- `transportSlotHighWater`；
- `transportSlotReleaseMismatch`；
- `transportStaleDescriptor`；
- `transportQueueDropData`；
- `transportQueueDropLog`；
- BLE/Wi-Fi 各 channel sent/drop；
- BLE congestion、fragment error 与 CRC mismatch。

`TXDROP` 表示有界 drop，不应通过把 send 改为 `portMAX_DELAY` 来消除。LIFECYCLE 使用独立 queue/reserved slot，普通 LOG 不能挤掉 MAPP/RAPP/BAPP；`eventPublished/eventDropped` 与 `queueDropLifecycle` 应在长跑中观察。长跑验收应在 warmup 后比较 free heap、minimum free heap、slot high-water 与 drops，确认不存在随 frame count 持续增长的 leak。

## Wi-Fi 现状

只支持 SoftAP/UDP。`WIFI=OFF` 关闭 transport eligibility，但当前不会销毁已启动 AP；STA/APSTA、扫描、凭据保存、连接和 forget 都返回 `sta_nyi`。详见 [命令参考](command-reference.md)。

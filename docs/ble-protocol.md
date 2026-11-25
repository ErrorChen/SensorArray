# BLE 协议 / BLE protocol

## 服务、方向和订阅 / Service, directions and subscriptions

服务使用 Bluetooth Base UUID。例如 FF10 在 Bleak 中的全名为 `0000ff10-0000-1000-8000-00805f9b34fb`。每次新连接应重新订阅所需通道。 / Expand each 16-bit UUID against the Bluetooth Base UUID. Subscribe again for each connection.

| UUID | 角色 / Role | 方向与属性 / Direction and properties |
| --- | --- | --- |
| `00FF` | SensorArray 主服务 / Primary service | 在广播或服务发现时识别 / Discover on the peripheral |
| `FF10` | 控制输入 / Command RX | 主机写入 ASCII 命令，支持 Write 和 Write Without Response / Host writes one ASCII command per GATT write |
| `FF11` | 控制回复 / CTRL | Read, Notify, Indicate。请求来源的 ACK/ERR 或查询结果 / Source-local replies and query results |
| `FF20` | 科学数据 / DATA | Read, Notify, Indicate。完整 C/V/R/M 或 B20 摘要 / Complete C/V/R/M records or a B20 preview |
| `FF30` | 日志及生命周期 / LOG and lifecycle | Read, Notify, Indicate。包括应用终态和故障 / Includes applied terminals and faults |

CCCD 为每个 TX characteristic 单独维护的小端 16 位值。`0000` 关闭该通道，`0001` 启用 Notify，`0002` 启用 Indicate，`0003` 两者都启用。 / Each TX characteristic has an independent, little-endian CCCD. Enable the matching bit on FF11 for replies, FF20 for data and FF30 for lifecycle events.

| 设置 / Setting | 发送方式 / Send behavior | 接收端使用 / Host use |
| --- | --- | --- |
| `BTX=FAST` | Notify，需 `0001` 或 `0003` / Notifications require the notify bit. | 固件可在拥塞时丢弃，按完整帧计收 / Count complete received frames. |
| `BTX=SAFE` | 有 Indicate bit 时等待确认。仅有 Notify bit 时用 Notify。 / Indicate when available, otherwise use notifications for a notify-only subscriber. | 固件先切换 TX 模式，再用新模式回复。双向切换时可先设 `0003`。 / Set compatible CCCDs before writing the setter. Query `BTX?` to recover its current value. |

## 帧大小与分片 / Message size and fragmentation

CTRL RX 命令最多 128 B，CTRL 回复最多 512 B，完整 DATA 或 LOG 最多 2304 B。单个属性值上限为 `min(negotiated ATT MTU − 3, 512)`。能容纳时发送原始 ASCII message。较大的消息按以下语法切片，尖括号表示占位。 / Limits apply to byte lengths. An unfragmented attribute value contains the complete ASCII message. A fragmented value starts with this LF-terminated ASCII envelope followed by the given bytes.

```text
G,<channel>,<messageId>,<fragmentIndex>,<fragmentCount>,<chunkLength>,<messageLength>,<crc32><LF>
<payload bytes>
```

| 字段 / Field | 含义与校验 / Meaning and validation |
| --- | --- |
| `channel` | `C` 对应 FF11，`D` 对应 FF20，`L` 对应 FF30。 / Require agreement with the receiving characteristic. |
| `messageId` | 以当前连接实例和 channel 区分一条完整消息。 / Key by connection instance, channel and message ID. Clear incomplete groups on disconnect. |
| `fragmentIndex/fragmentCount` | 十进制，索引从 0 到 count−1。 / Require positive count and every index. Reject conflicting duplicates and inconsistent group metadata. |
| `chunkLength/messageLength` | 当前 LF 后有效字节数及重组总长度。 / Validate the actual chunk length and total bounded message length. Join by index. |
| `crc32` | 对拼接后原始 message 的 IEEE CRC-32，8 位十六进制。 / Check the whole-message CRC. For full science records then independently check the inner K CRC, masks and identity. |

同一 packet 的 payload 可含多个 LF。接收器先按 attribute value 区分 G envelope 和原始 message，再按已知长度处理。重组缓存应限制组数、长度和保留时间。 / Preserve original bytes through both CRC checks. Apply bounds to pending groups. A fragment or notification alone contributes zero complete science frames. [数据格式 / Science frames](output-format.md)

## 发送优先级和完整快照 / Send priority and full snapshots

CTRL 与关键生命周期优先，其次为保护完整 DATA、普通 DATA、普通 LOG。普通 DATA 数量为 `CONFIG_SENSORARRAY_BLE_TX_QUEUE_LEN`，普通 LOG 有两个槽，优先 CTRL/lifecycle 有两个槽，保护帧有两个槽。正常 DATA 订阅生效时，拒绝新普通 LOG 并计数。保护发送可用较新的完整帧替换仍处于 PENDING 的帧。已被 writer 或 SDK 持有的缓冲需待所有者释放。 / The BLE layer owns bounded message storage separately from the generic transport pool. Its descriptor queues carry references. Priority service and separate protected slots keep critical traffic eligible under ordinary DATA pressure.

配置或有效订阅改变时，要匹配新身份的首个有效完整快照。周期默认 1 秒。同一身份且序号前进的普通完整帧在本地成功完成时可满足周期要求。超期后保护下一份完整编码帧。完整帧保留原始无效掩码和 Xhh，科学点须逐点验证。`TX=SHORT` 或 `TX=RT` 下的普通 CAP `B20,previewOnly=1` 只用于预览，不计入完整快照。VOLT、RES、MIXED 及受保护的 CAP 仍发完整记录。 / After a configuration or effective-stream change, look for a valid complete frame of the new identity. Ordinary full-frame completion can renew the periodic deadline. A protected slot isolates scheduling capacity. Actual reception still requires a complete, valid record at the host.

`BLEFLOW?` 报告有效订阅、容量拒绝和普通 DATA 选择比例。正常时选择全部普通候选。三个连续有效的 1–2 秒窗口容量拒绝率严格超过 20% 后，限速步长为 50‰，间隔至少 3 秒。累计 10 秒有效干净观察再提高 50‰，证据新鲜期 3 秒，限制期间用真实普通帧按秒探测。采集继续独立运行。 / Adaptation uses observed local capacity rejection. A 2.5% host loss tolerance does not command deliberate drops. `SNAPSRC?`, `SNAPNET?` and `BLESNAP?` describe separate local protection stages. Compare observations within one boot, connection, effective stream and configuration. [字段与计数 / Fields and counters](integration/firmware_host_contract.md#protected-snapshot-v1)

## 拥塞、确认和恢复 / Congestion, confirmation and recovery

优先记录或 CTRL 可在本地尚未被接受时重试相同字节。可重试的 SDK 返回值是 `ESP_FAIL`，间隔 10 ms，外层窗口 500 ms。单次已进入 SDK 的调用可能超过此外层预算。Indicate 获得 `ESP_OK` 后等待匹配确认最长 1000 ms，确认缺失时不会重发该已接受的 indication。 / Retrying retained bytes does not run the command again. An application deadline bounds new attempts. A stalled SDK call retains its slot until ownership is resolved. `BLEGUARD?` exposes quarantine and late-return evidence and can also be read over a working wired link.

`BLELINK?` 查协商结果，`BLETX?` 查本地 GATT 服务，`BLESVC?` 区分 SDK 返回和 waiter 匹配计数。Notify 的本地完成、Indicate 的 GATT 确认和主机完整校验是不同观察点。终态未收到时先查 `CONFIG?` 或该功能精确请求查询。重连后复查 boot/session 和 revision，再接纳新数据。 / Recover unknown outcomes by read-only query. Confirm complete receipt independently at the host by sequence, original bytes, CRC and immutable identity. [命令表 / Commands](command-reference.md) · [BLE 实现 / Component](../components/sensorarrayBle/README.md)

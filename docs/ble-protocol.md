# BLE 协议

## UUID 与方向

16-bit UUID 在 Windows/Bleak 中通常展开为 Bluetooth Base UUID，例如 `FF10`：

```text
0000ff10-0000-1000-8000-00805f9b34fb
```

| UUID | 名称 | 方向 | Properties | Permission | CCCD |
| --- | --- | --- | --- | --- | --- |
| `0x00FF` | SensorArray Service | - | Primary Service | Read | - |
| `0xFF10` | CTRL RX | host -> device | Write、Write Without Response | Write | 无 |
| `0xFF11` | CTRL TX | device -> host | Read、Notify、Indicate | Read | 有 |
| `0xFF20` | DATA TX | device -> host | Read、Notify、Indicate | Read | 有 |
| `0xFF30` | LOG TX | device -> host | Read、Notify、Indicate | Read | 有 |

这些 UUID 属于上位机兼容契约，不能因 transport/stack 修复而改变。

## CCCD

CCCD 小端值：

| 值 | 含义 |
| --- | --- |
| `0x0000` | Disable |
| `0x0001` | Enable Notify |
| `0x0002` | Enable Indicate |
| `0x0003` | Enable Notify + Indicate |

`BTX=FAST` 使用 `confirm=false` 的 Notify，客户端应启用 Notify bit。`BTX=SAFE` 使用 `confirm=true` 的 Indicate，客户端应启用 Indicate bit。CCCD gating 按当前 TX mode 严格执行：`0x0001` 只允许 FAST，`0x0002` 只允许 SAFE，`0x0003` 同时允许两者。连接中需要往返切换 FAST/SAFE 时推荐使用 `0x0003`。

每个 TX characteristic 独立 gating：

- `FF11` CCCD 只控制 CTRL reply；
- `FF20` CCCD 只控制 DATA；
- `FF30` CCCD 只控制 LOG；
- 订阅 `FF20` 不能隐式启用 `FF30`。

## FAST 与 SAFE

| Mode | GATT send | 特点 |
| --- | --- | --- |
| `BTX=FAST` | Notify | 无 per-packet confirmation；拥塞时有界 drop |
| `BTX=SAFE` | Indicate | 等待 `ESP_GATTS_CONF_EVT`；超时有界，不能永久占用 slot |

安全切换流程：

```text
1. FF11 CCCD 先开启 Indicate（推荐 0x0003）
2. SAFE 下需要 DATA/LOG 时，FF20/FF30 也先开启 Indicate
3. FF10 <- BTX=SAFE\n
4. FF11 -> ACK,cmd=BTX,v=SAFE（Indicate）
5. FF10 <- BTX?\n
6. FF11 -> ACK,cmd=BTX,v=SAFE（Indicate）
```

`BTX=SAFE` 会先更新 firmware TX mode，再格式化并发送本条命令的 source-local 回复。因此，如果 FF11 当时只有 Notify bit `0x0001`，命令可能已经生效，但 ACK 会被新的 SAFE/Indicate gating 拒绝，客户端看起来像“写入后无回复”。无 ACK 不能直接推断命令失败；先为 FF11 开启 Indicate，再发送 `BTX?`，或从 Serial 查询当前模式。不要盲目重复会改变状态的 setter。

切回 FAST 也应在发送 `BTX=FAST` 前确保 FF11 Notify bit 已开启；使用 `0x0003` 可避免切换后的回复落入不兼容 CCCD。HIL 必须真实验证 Windows/Bleak 与 nRF Connect 对 Notify/Indicate 的订阅行为，不能只看 characteristic properties。

## MTU 与 message 上限

- GATT attribute value 上限：512 bytes；
- DATA/LOG 完整 message 上限：1536 bytes；
- CTRL TX 完整 message 上限：512 bytes；
- CTRL RX 命令上限：128 bytes；
- 实际单次 payload 由 negotiated MTU 的 ATT payload 决定。

如果完整 message 能放入当前 ATT payload，firmware 直接发送 raw ASCII；否则分片。

## BLE 内部 slot 与 descriptor

BLE component 在通用 transport pool 之后拥有自己的静态 payload pools。当前有效配置为：

| Channel | Payload slot | 数量 | Queue |
| --- | ---: | ---: | --- |
| DATA / `FF20` | 1536 bytes | 4 | DATA/LOG 共用普通 descriptor FIFO，总容量 6 |
| LOG / `FF30` | 1536 bytes | 2 | 同上 |
| CTRL TX / `FF11` | 512 bytes | 2 | 独立高优先级 descriptor queue |

DATA 与 LOG 是 `4 + 2` 两个独立 pool，不会因 FF20 流量占用 FF30 的两个 slot，也不会因连续 compact-summary/ADS LOG 占用四个 DATA slot。queue 中只保存不超过 24 bytes 的 descriptor；payload 由 slot 独占，发送结束、drop、stale generation 或断连 drain 后释放。CTRL 不使用 DATA/LOG slot。

BLE CTRL reply 在 Bluedroid 接受 payload 前采用有界重试：

- 只对 CTRL channel 生效；DATA/LOG 保持单次、非阻塞尝试；
- retry interval 为 10 ms，总 retry window 为 500 ms；
- 只把 `ESP_FAIL` 视为可重试的“尚未接受”结果，其他 error 立即返回；
- 如果 SAFE indication 的 `esp_ble_gatts_send_indicate(..., confirm=true)` 已返回 `ESP_OK`，说明 Bluedroid 已接受该 reply；之后即使 confirmation timeout，也绝不重发，避免客户端收到重复的控制回复。

该 retry 不改变 command handler 的 exactly-once 边界：setter 在共享 handler 中只执行一次，重试的只是尚未被 BLE stack 接受的 source-local reply bytes。

## 分片 envelope

兼容 wire format：

```text
G,<channel>,<messageId>,<fragmentIndex>,<fragmentCount>,<chunkLength>,<messageLength>,<crc>\n
<payload bytes>
```

channel：

| 字符 | Channel |
| --- | --- |
| `D` | DATA / `FF20` |
| `L` | LOG / `FF30` |
| `C` | CTRL / `FF11` |

字段规则：

- `fragmentIndex` 从 0 开始；
- `fragmentCount` 是该 message 的总 fragment 数；
- `chunkLength` 必须等于 newline 后当前 fragment 的实际 bytes；
- `messageLength` 是所有 chunk 拼接后的总 bytes；
- `crc` 是完整 message 的 IEEE CRC-32，十六进制表示；
- reassembly key 是 `(channel, messageId)`，不能只用 message ID。

示例：

```text
G,D,42,0,4,210,812,89ABCDEF
<210 bytes>
```

## Host 重组算法

1. raw payload 不以 `G,` 开头时，直接作为该 characteristic 的完整 message；
2. 解析 envelope，拒绝未知 channel、越界 index/count 与长度不符；
3. 按 `(channel,messageId)` 保存 chunk；
4. 检查 index `0..count-1` 全部存在；
5. 按 index 拼接；
6. 检查总长度等于 `messageLength`；
7. 计算 IEEE CRC-32 并比较 envelope CRC；
8. DATA message 再按 [输出格式](output-format.md) 验证内部 `K,...crc=...`。

只统计 notification 数量不构成 BLE DATA 验收。

## 连接生命周期

连接时：

- `subscribed[]` 全部清零；
- 保存 conn ID；
- `connectionGeneration++`；
- 请求 2M PHY 和短 connection interval，失败时允许回退 1M。

断开时：

- connected/congested 清零；
- `subscribed[]` 清零；
- `connectionGeneration++`；
- drain CTRL RX、CTRL TX 与 DATA/LOG TX queues；
- release slots；
- 重新开始 advertising。

descriptor 在发送前必须匹配 slot generation 与 connection generation。旧连接遗留 descriptor 只能计为 stale/drop，不能发到新连接。

## 自动 HIL 最低覆盖

- connect 不订阅；
- FF11 only、FF20 only、FF30 only；
- FF11+FF20、FF11+FF30、FF20+FF30、全部三项；
- FAST 与 SAFE；
- fragment length + envelope CRC + DATA inner CRC；
- subscribe/unsubscribe；
- reconnect；
- CAP/RES/VOLT mode switch；
- RES 长跑；
- BLE 测试期间同时监视 Serial panic/reset。

完整流程见 [验证](validation.md)。

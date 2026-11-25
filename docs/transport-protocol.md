# 传输通道 / Transport channels

| 接口 / Interface | 命令与数据 / Commands and data |
|---|---|
| Formal OTG CDC | 303a:4042 的 CDC0 承载正式 ASCII DATA 与 CTRL。同一记录的尾部和后续记录不会拼接成有效科学帧。 / CDC0 multiplexes complete science records with control. Check original bytes and trailer independently. |
| Debug / recovery | CDC1 为诊断输出。应用恢复与 ROM 下载可能重新枚举为 Serial/JTAG。 / Debug/data roles are distinct. Match USB identities after reset or re-enumeration. |
| BLE | FF10 写命令，FF11 收回复，FF20 收 DATA，FF30 收日志/生命周期。按 characteristic 独立订阅。 / Subscribe to each required channel. Prioritized control and protected complete snapshots remain eligible under load. |
| Wi-Fi | SoftAP/UDP；查询 WIFI? 和带请求号的 WIFIREQ?。OFF 关闭输出资格；已就绪 backend 可复用。 / Peer-directed control and datagram output use the Wi-Fi owner. OFF is an output gate. |

`ST` 选择输出目的（含有线 `ST=SER`），`TX` 选择消息形式，`BTX` 选择 BLE GATT 发送策略。OTG 镜像的 `USBSTREAM?` 报告正式接口，`SERIAL?` 报告调试计数；`USBSTREAM=DEBUG/FULL` 只适用于旧 Serial/JTAG 路径，在 OTG 上返回 `legacy_usj_only`。 / Select the sink with ST, payload form with TX, and BLE confirmation policy with BTX. Query the formal OTG and debug roles separately. The legacy USBSTREAM setters apply to the Serial/JTAG backend. Query CONFIG? for the authoritative configuration.

采集、emit、sink 准入、驱动接受、回调、主机完整帧各有分母。使用同会话的计数差值。BLE 弱链路可以降低普通数据频率并保留首个有效完整帧及周期全量；SAFE indication 仅给出链路级确认。USB 无人读取时过期记录释放，热路径保持有界。 / Report capture, emission, admission, driver acceptance, callbacks and host receipt separately. Under congestion the BLE scheduler reduces ordinary output and prioritizes complete snapshots. USB reader stalls consume bounded buffering.

[BLE 分片 / BLE fragments](ble-protocol.md) · [命令 / Commands](command-reference.md) · [主机契约 / Host contract](integration/firmware_host_contract.md)

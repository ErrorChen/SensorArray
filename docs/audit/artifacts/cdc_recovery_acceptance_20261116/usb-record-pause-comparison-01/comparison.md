# 650 ms formal-reader pause comparison / 停读实录对比

本比较在两次录制、BLE及debug observer全部关闭后离线重放原始数据；不改原gate、阈值或固件。 / Both recordings and observers were closed before raw replay. Original gates and thresholds remain unchanged.

| 项目 / Measurement | 旧 / bss-pause-volt2-01 | 新 / record-pause-volt2-01 |
| --- | --- | --- |
| 完整ELF / Full ELF | `179f9183e2c31b35c906c9b811bb2ca1cf4d16dbca43ee3a1c6c4371c0f56a2d` | `892f586bdb131e77614c1ca72723c9247159f9d3a7bf5b49917004383f252677` |
| session / boot | 05DD194E2065ADB0 / 2 | 0119031D7DEF4971 / 1 |
| Capture秒 / seconds | 120.000087864 | 120.000083020 |
| 实际应用停读秒 / Actual application pause | 0.650093844 | 0.650079173 |
| USB 完整可信帧 / trusted complete | 5033 | 5225 |
| USB 内部缺帧 / endpoint sequence missing | 12 | 0 |
| USB CRC / identity / wire errors | 0/0/0 | 0/0/0 |
| USB invalid点 / points | 3 | 3 |
| BLE 完整可信帧 / trusted complete | 5045 | 5225 |
| BLE 内部缺帧 / endpoint sequence missing | 0 | 0 |
| BLE CRC / identity / wire errors | 0/0/0 | 0/0/0 |
| BLE invalid点 / points | 3 | 3 |
| 设备实际FPS / device sequence-timestamp FPS | 42.040172569 | 43.540909814 |
| USB PC完整帧FPS / receipt FPS | 41.942798302 | 43.543548600 |
| USB 最大完整接收间隔秒 / maximum complete-receipt interval | 0.676887718 | 0.676319039 |
| BLE PC完整帧FPS / receipt FPS | 42.041394158 | 43.544590474 |
| BLE 最大完整接收间隔秒 / maximum complete-receipt interval | 0.059098801 | 0.054104268 |
| USBQ dDrop Δ | 7 | 0 |
| USBQ expired Δ | 5 | 0 |
| USBQ partial Δ | 1 | 0 |
| USBQ cDrop Δ | 0 | 0 |
| USBQ cExpired Δ | 0 | 0 |
| USBQ dSubmit Δ | 5043 | 5234 |
| USBQ cSubmit Δ | 115 | 115 |
| queueMaxUs boot highwater 前→后 | 243329→243329 | 987842→987842 |
| USBQ dQueueCap advertised | NOT_REPORTED | 100 |
| USBQ dLifetimeUs advertised | NOT_REPORTED | 1000000 |
| CAP8恢复 / recovery | CAP8_QUERY_VERIFIED | CAP8_QUERY_VERIFIED |
| 原gate / original gate | FAIL | PASS_CASE_SCREEN |

实际暂停只停止formal应用读取；Linux TTY/USB栈、控制器和设备仍可能缓冲，不能称为USB总线停机650 ms。设备FPS由同身份有序sequence增量/设备timestamp跨度算出；PC FPS由可信完整帧数与各自首末完成接收时间算出，均不是配置目标值。 / The injected pause affects application reads only. Device FPS and PC receipt FPS use distinct measured clocks and denominators.

queueMaxUs是boot级最大已服务记录年龄，未在capture或pause开始清零，不能当本窗口最大值，也不能用前后差当年龄。USBQ计数来自包围capture的不同查询时刻；normalExpired包括DATA/LOG，不能把计数直接指派给某个sequence。 / Queue age is a boot highwater, not a pause-window maximum. Counter queries have separate boundaries and are not per-sequence traces.

旧窗12个USB完整帧缺口与DATA拒绝+7、normal过期+5数值吻合；partial+1是部分发送的另一计数维度，不能再加成13。新窗未通过降低实测capture FPS消除缺口，但两个独立boot的FPS差不能单独归因于本修复。 / The old gap count matches seven DATA rejections plus five normal expiries; its one partial record is not an additional thirteenth loss. The new run's measured capture rate is higher, without establishing the cause of the rate difference.

## bss-pause-volt2-01

USB sequence `85787..90831`；缺口 `[{"after": 86515, "before": 86528, "missing": 12}]`。BLE sequence `85787..90831`。共同原wire一致 **5033**，差异 **0**；capture截断 **1**。完整帧保留invalid点；零完整帧CRC错误不证明没有截断。

独立debug ADSHEALTH窗：CRC拒绝 **0/108299** checked reads，首末receipt跨度 **118.737348077秒**，含重试读数，非帧分母；crcRetries末端token不作推断。

FAULT/RESET观察：`[]` / `[]`；两observer正常关闭，CAP8实际cleanup回复保存在[comparison.json](comparison.json)。

| 原始记录 / Original record | 未压缩字节 / Bytes | 未压缩SHA256 |
| --- | ---: | --- |
| [formal_authority_raw](../../../../../docs/audit/artifacts/cdc_recovery_acceptance_20261116/bss-pause-volt2-01/bss-pause-volt2-01-volt2/bss-pause-volt2-01-volt2.rx.bin.gz) | 3810367 | `be5c6dd8fa3709cf9a38e5e7d72987fd1e3a5fe54f943d9ddaf5c3af6643d2d0` |
| [formal_authority_events](../../../../../docs/audit/artifacts/cdc_recovery_acceptance_20261116/bss-pause-volt2-01/bss-pause-volt2-01-volt2/bss-pause-volt2-01-volt2.events.jsonl.gz) | 2684135 | `2bdff16be08c8f48a877b3a9a967ea13d3dee176bda841b671570415ba703d09` |
| [debug_raw](../../../../../docs/audit/artifacts/cdc_recovery_acceptance_20261116/bss-pause-volt2-01/bss-pause-volt2-01-volt2/bss-pause-volt2-01-volt2.debug.rx.bin.gz) | 311218 | `587261ecf0e026a0b1c835f472c1c03646eebab2932b2dfb8c8c3119ab549b5c` |
| [debug_events](../../../../../docs/audit/artifacts/cdc_recovery_acceptance_20261116/bss-pause-volt2-01/bss-pause-volt2-01-volt2/bss-pause-volt2-01-volt2.debug.events.jsonl.gz) | 281134 | `4c829bf389d6ea69f7f7a3438f865a9c159df47fe772b89bd771646158b9d54e` |
| [BLE_journal](../../../../../docs/audit/artifacts/cdc_recovery_acceptance_20261116/bss-pause-volt2-01/bss-pause-volt2-01-volt2/bss-pause-volt2-01-volt2.ble.notifications.jsonl.gz) | 7819073 | `517c786b2174d8575a7d572812b091d878102dd268be54f3a3d41d9f3b4768a4` |
| [gate](../../../../../docs/audit/artifacts/cdc_recovery_acceptance_20261116/bss-pause-volt2-01/bss-pause-volt2-01-volt2/bss-pause-volt2-01-volt2.gate.json) | 241742 | `b1e6efde364efbe8791651f16fbb58ab1df9369a8ecc2b9abcffc2567eac8d1b` |

## record-pause-volt2-01

USB sequence `981..6205`；缺口 `[]`。BLE sequence `981..6205`。共同原wire一致 **5225**，差异 **0**；capture截断 **0**。完整帧保留invalid点；零完整帧CRC错误不证明没有截断。

独立debug ADSHEALTH窗：CRC拒绝 **0/109666** checked reads，首末receipt跨度 **119.013146868秒**，含重试读数，非帧分母；crcRetries末端token不作推断。

FAULT/RESET观察：`[]` / `[]`；两observer正常关闭，CAP8实际cleanup回复保存在[comparison.json](comparison.json)。

| 原始记录 / Original record | 未压缩字节 / Bytes | 未压缩SHA256 |
| --- | ---: | --- |
| [formal_authority_raw](../../../../../docs/audit/artifacts/cdc_recovery_acceptance_20261116/record-pause-volt2-01/record-pause-volt2-01-volt2/record-pause-volt2-01-volt2.rx.bin.gz) | 3837116 | `9366fc53e7a27c1ccaafa5c69b355c4803a2350dbb86416e5e7fee06245ad32d` |
| [formal_authority_events](../../../../../docs/audit/artifacts/cdc_recovery_acceptance_20261116/record-pause-volt2-01/record-pause-volt2-01-volt2/record-pause-volt2-01-volt2.events.jsonl.gz) | 2820890 | `989d815dc5414e6921acd0f62b2d4e898ebfcdafee85bdc336b9be718230bb55` |
| [debug_raw](../../../../../docs/audit/artifacts/cdc_recovery_acceptance_20261116/record-pause-volt2-01/record-pause-volt2-01-volt2/record-pause-volt2-01-volt2.debug.rx.bin.gz) | 288098 | `72019f0a13eddb13d09cdcb32e1cc144fea2d86472d64bdfc99394c6f75b4cad` |
| [debug_events](../../../../../docs/audit/artifacts/cdc_recovery_acceptance_20261116/record-pause-volt2-01/record-pause-volt2-01-volt2/record-pause-volt2-01-volt2.debug.events.jsonl.gz) | 267725 | `bf66a4f65fb7ad01fcba5bba8f81e528211570e2f110b98a732233188a091498` |
| [BLE_journal](../../../../../docs/audit/artifacts/cdc_recovery_acceptance_20261116/record-pause-volt2-01/record-pause-volt2-01-volt2/record-pause-volt2-01-volt2.ble.notifications.jsonl.gz) | 7910660 | `59b98d6c21e6bf250a6896759bf877b8589063e90f150ed3b9519dc44950cf23` |
| [gate](../../../../../docs/audit/artifacts/cdc_recovery_acceptance_20261116/record-pause-volt2-01/record-pause-volt2-01-volt2/record-pause-volt2-01-volt2.gate.json) | 240051 | `c07e782abe7f5593663c77f95fd5c5fe87797c5837f0f1222ed1bc1ffc9c068e` |

## Reproduce / 重放

```sh
python3 /home/csc/esp/workspace/SensorArray/docs/audit/artifacts/cdc_recovery_acceptance_20261116/usb-record-pause-comparison-01/compare.py
```

脚本先验证双方闭合，再核对原始SHA并用固定版本actual parser/analyzer重放；stdout为JSON，`--write`只更新本证据目录的comparison.json和comparison.md。两次不同ELF/session的结果不能拼为同一boot；有限暂停筛查不建立所有stall情况下的交付保证、模拟精度或整体验收。 / These are separate image/session observations. They do not establish universal delivery guarantees, analog precision or overall acceptance.

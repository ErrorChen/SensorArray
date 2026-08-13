# 故障排查

## BLE 一订阅 FF20/FF30 就重启

先看 Serial 的第一个 panic，而不是后续 cache/panic-handler 二次异常。历史故障的第一证据是：

```text
Stack canary watchpoint triggered (sensorarrayLogT)
PC=memset
A4=0x618
sensorarrayTransportQueue
sensorarrayTransportPublishLog
sensorarrayAsyncLogTask
```

旧通用 transport item 把 1536-byte payload 与 metadata 放在一个 1560-byte object 中；producer stack、FreeRTOS queue storage 与 consumer stack 都保存完整 object。BLE 全局 gating 还会使仅订阅 DATA 的连接错误地把 LOG 送入通用 queue。修复后的检查点：

- queue item 必须是小 descriptor；
- payload 位于 5-slot 静态 pool，其中 1 个 slot 专供 lifecycle event；
- FF20 与 FF30 独立 gating；
- DATA/LOG drop 与 slot high-water 可见；
- log task minimum remaining >= 2048 bytes。

如果仍重现，保存完整 log、ELF 与 backtrace，检查 `STK<n>`、slot mismatch、BLE CCCD 和 first crash；不要只把 log stack 增加到 24/32 KiB。

## RES 未订阅 BLE 仍长跑重启

`BL50,conn=1,sub=000` 证明此路径不依赖 FF20/FF30。RES 的独立触发是：

```text
RES frame formatting
+ compact summary
+ ADS50 / ADST50 / AB50
+ battery / rail / profile statistics
+ logger resident objects
-> sensorarrayLogTask peak stack
```

检查 async logger 是否仍在 task stack 同时放置 summary、frame 和多个 1536-byte packet。正确实现应使用 task-owned workspace，并按生命周期 reset，而不是每帧清零整个 workspace。

`X0D` 是 open cell 错误码；当前未连接 cell 出现它不等于 reboot 根因。

## BLE 已连接但收不到 CTRL 回复

检查：

1. 写入的是 `FF10`；
2. `FF11` CCCD 已启用；
3. FAST 使用 Notify bit `0x0001`；
4. SAFE 优先使用 Indicate bit `0x0002`；只有 Notify 的 Ubuntu/BlueZ 客户端会使用 SAFE Notify fallback；
5. 命令不超过 128 bytes；
6. Serial 是否出现 `BLERXERR`；
7. `BTX?` 与客户端 CCCD 是否一致。

命令可能已经进入 mailbox，但 FF11 未订阅会导致 source-local reply 发不出来。不要重复发送会改变硬件的 setter，先订阅 FF11 并用 query 确认状态。

## 只订阅 FF20 却出现大量 LOG drop

这是 channel gating 异常。`ST=AUTO|BLE|ALL` 都必须满足：

```text
DATA -> connected + FF20 compatible CCCD
LOG  -> connected + FF30 compatible CCCD
```

检查 `BL50.sub`、transport DATA/LOG drop 和 slot high-water。FF20 不能通过 `DATA || LOG` 全局条件启用 LOG path。

## BLE 分片无法重组或 CRC 失败

- Windows UUID 应兼容 16-bit 与 Base UUID；
- 按 `(channel,messageId)` 分组；
- index 从 0 开始；
- `chunkLength` 只计算 header newline 后 payload；
- assembled length 必须等于 `messageLength`；
- envelope CRC 是完整 message 的 IEEE CRC-32；
- DATA message 还要验证内部 `K` CRC。

`BLECORRUPT` 表示 firmware slot 内容在 enqueue/dequeue 间不一致；这是固件错误，不应由 host 忽略。

## SAFE 模式卡住

SAFE 在有 Indicate bit 时使用 confirmation；只有 Notify 的客户端使用有界 Notify fallback。Indicate 路径应检查 `ESP_GATTS_CONF_EVT` 是否到达；confirmation timeout 必须有界并释放 slot。长期 queue 增长说明 lifecycle/CCCD 有问题。

Ubuntu/BlueZ 的 Bleak `StartNotify` 无法在同时支持 Notify/Indicate 的 characteristic 上选择 CCCD 的 Indicate bit，因此不要把 Linux 上的 SAFE Notify fallback 误判为固件故障。具备 CCCD 写入选择能力的客户端应使用 `0x0003`。恢复步骤：

1. 为 FF11 开启 Indicate，推荐 CCCD `0x0003`；
2. SAFE 下要收 DATA/LOG 时，也为 FF20/FF30 开启 Indicate；
3. 发送只读 `BTX?` 确认当前模式，或从 Serial 查询；
4. 切回 FAST 前先确保 FF11 Notify bit 已开启。

不要仅因缺 ACK 就反复发送 setter。若客户端无法在当前连接正确修改 CCCD，断开重连后先按当前 mode 订阅相应 bit，再查询状态。

## Wi-Fi 命令返回 `sta_nyi`

当前只支持 SoftAP/UDP。以下不是可用功能：STA、APSTA、scan、SSID/password、save、connect、forget。使用 `WIFI=AP`，或改用 Serial/BLE；不要把 NYI response 当作临时网络错误。

## Mode accepted 但数据未切换

`MACK` 只表示 queued。等待同一 request ID 的 `MAPP`，再等新 `gen/rid` 的完整 CRC frame。启动阶段不能只等较早的 `ADSBOOT`；应等 boot sweep 后第一帧完整、fresh CAP frame 再切 mode。

## VOLT 无法进入或 `RAILCFG` 被拒绝

普通 VOLT 会自动进入 `SAFE_RAIL_MONITOR`，通过 ADS126x internal analog supply monitor 获取 rail span；不需要 `RAILCFG`。只有需要外部 DMM debug override 时，才在 CAP/RES 下按正/负 µV 发送成对 `RAILCFG`，并等待匹配的 `RACK`/`RAPP`。

在 VOLT 内发送 `RAILCFG` 会按设计返回：

```text
ERR,cmd=RAILCFG,reason=apply_before_volt
```

先回 CAP/RES，再重新测量和应用。若未应用 external rail 就发送 `MODE=VOLT`，`MACK` 仍可能只表示请求已排队，随后帧边界会输出 `MERR` 并进入 SAFE。

`RAIL?` 和 `AB50.rail` 是内部 monitor/reference telemetry，不是外部 DMM 精度证据。若自动 monitor 失败，应检查 ADS、TMUX readback、供电与隔离 route；若 override 异常，再检查 DMM 接地、AVDD/AVSS 符号和 µV 单位。

## 电池已断开

AIN8 的浮动、长周期 PWM 或异常范围属于预期的 battery diagnostic invalid。看 `valid=0,reason=...` 与 `lastGoodValid/lastGoodFresh/lastGoodAgeMs`；不要把 `bt=-1`、异常 raw 或 `reason=absent_or_open` 当作矩阵采集、ADS route 或 transport 故障。

## RES 值不合理

检查：

- route snapshot：SW low/REF、matrixRef REFOUT、INTREF on、REFMUX internal；
- `RAIL?`、`CELL?=SxDy` 的 raw/nodeUv/vref/vss/rref/PGA；
- `RESSETTLE`；
- open/short/error mask；
- S1D1/S8D8 实际 DMM 值与接线。

不要把标称 10 kΩ 写入生产算法。宽松 5–20 kΩ 只适合当前调试板 pipeline sanity。

## 怀疑内存泄漏

记录 warmup 后和测试结束时的 free heap；同时看 minimum heap、transport/BLE slot high-water、stale descriptor、release mismatch。一次性 lazy allocation 可以接受；每 N 帧持续下降不能接受。

## 再次 panic 的处理顺序

1. 保存完整 Serial log；
2. 标记第一个 `Guru Meditation`/stack canary；
3. 用本次 build ELF decode backtrace；
4. 确认 task；
5. 对照 stack high-water 与大 local object；
6. 修复 ownership/lifetime；
7. 重跑完整 Serial + BLE regression。

stack corruption 后出现的 `Cache_Resume_ICache`、`panic_enable_cache`、`CORRUPTED` 或 “Panic handler entered multiple times” 通常是二次后果，不应先重写 flash/cache 层。

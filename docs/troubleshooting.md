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
- payload 位于 4-slot 静态 pool；
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
4. SAFE 使用 Indicate bit `0x0002`；
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

SAFE 使用 Indicate confirmation。检查客户端是否启用了 Indicate 而不是只启用 Notify，并确认 `ESP_GATTS_CONF_EVT` 到达。confirmation timeout 必须有界并释放 slot；长期 queue 增长说明 lifecycle/CCCD 有问题。

一个容易误判的顺序是：FF11 只有 Notify (`0x0001`) 时发送 `BTX=SAFE`。firmware 先应用 SAFE，再按新模式的 Indicate gating 发送 ACK，所以命令可能已经生效但客户端收不到回复。恢复步骤：

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

VOLT 必须使用当前供电/接线下同步外部 DMM 测得的 rail split。正确顺序是在 CAP/RES 下测量 `AVDD -> GND` 和 `AVSS -> GND`，按正/负 µV 发送成对 `RAILCFG`，等待匹配的 `RACK` 与 `RAPP,source=external,state=applied`，再发送 `MODE=VOLT`。

在 VOLT 内发送 `RAILCFG` 会按设计返回：

```text
ERR,cmd=RAILCFG,reason=apply_before_volt
```

先回 CAP/RES，再重新测量和应用。若未应用 external rail 就发送 `MODE=VOLT`，`MACK` 仍可能只表示请求已排队，随后帧边界会输出 `MERR` 并进入 SAFE。

不要把 `RAIL?`、`AB50.rail` 或 ADS supply-monitor 值复制成 `RAILCFG`。monitor rail 是内部运行健康数据，在 VOLT clamp 条件下不能刷新，也没有外部 DMM 的同步性和精度证据。若读数异常，应检查 DMM 接地、AVDD/AVSS 符号、单位是否为 µV、供电或接线是否在 DMM 测量后发生变化。

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

# 命令参考

## 共同规则

命令是 ASCII 文本，不区分大小写；前后空白和行末 CR/LF 会被规范化。三种入口共用同一个核心 handler：

- Serial：USB Serial/JTAG 行输入，回复写回 Serial；
- BLE：写 `FF10`，回复发到 `FF11`；
- Wi-Fi CTRL：UDP `3335`，回复发回原 peer。

表中“BLE FF10=是”只代表命令可以写入；客户端还必须为 `FF11` 配置兼容的 CCCD，才能收到回复。Wi-Fi CTRL 目前只在 SoftAP/UDP backend 可达时可用。

所有 setter 都是易失状态，重启后恢复编译配置默认值；当前没有命令把运行时配置写入 NVS。

## Transport 与输出

| Command | Type | Serial | BLE FF10 | Wi-Fi CTRL | Arguments | Response | Apply timing | Persistent | Status | Notes |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `TX?` | Query | 是 | 是 | 是 | 无 | `ACK,cmd=TX,v=<short|rel|full>` | 立即 snapshot | - | 支持 | 查询 transport text mode。 |
| `TX=REL` / `TX=RT` / `TX=SHORT` / `TX=FULL` | Config | 是 | 是 | 是 | 枚举值 | `ACK,cmd=TX,v=<...>` | Core 0 立即 | 否 | 支持；`RT` 是 `SHORT` alias | 当前 `SHORT/RT` 只把可压缩的 BLE CAP DATA 改为 `B20`；V/R、Wi-Fi、Serial 不因此改帧。`REL` 与 `FULL` 当前都转发完整 payload。 |
| `ST?` | Query | 是 | 是 | 是 | 无 | `ACK,cmd=ST,v=<...>` | 立即 snapshot | - | 支持 | 查询 sink selection。 |
| `ST=AUTO` | Config | 是 | 是 | 是 | 无 | `ACK,cmd=ST,v=auto` | Core 0 立即 | 否 | 支持 | Serial 始终 eligible；BLE 还需 connected 且当前 DATA/LOG channel 的 CCCD 与 FAST/SAFE 兼容；Wi-Fi 还需 mode enabled 且 ready。 |
| `ST=SER` | Config | 是 | 是 | 是 | 无 | `ACK,cmd=ST,v=ser` | Core 0 立即 | 否 | 支持 | 只选 Serial。 |
| `ST=BLE` | Config | 是 | 是 | 是 | 无 | `ACK,cmd=ST,v=ble` | Core 0 立即 | 否 | 支持 | 只希望发送 BLE；仍要求 connected 和对应 characteristic 已订阅，不能把未订阅 channel 入队。 |
| `ST=WIFI` | Config | 是 | 是 | 是 | 无 | `ACK,cmd=ST,v=wifi` | Core 0 立即 | 否 | 支持 | 只选择 Wi-Fi 且要求 mode 非 OFF；producer 不以 ready 为前置，consumer 发送前才检查 ready。先用 `WIFI?` 确认 `ok=1`。 |
| `ST=ALL` | Config | 是 | 是 | 是 | 无 | `ACK,cmd=ST,v=all` | Core 0 立即 | 否 | 支持 | Serial + BLE + Wi-Fi；BLE 严格按 channel/CCCD gating，Wi-Fi mode 非 OFF 即可入队并在 consumer 发送前检查 ready。 |
| `BTX?` | Query | 是 | 是 | 是 | 无 | `ACK,cmd=BTX,v=<FAST|SAFE>` | 立即 snapshot | - | 支持 | 查询 BLE TX mode。 |
| `BTX=FAST` / `BTX=SAFE` | Config | 是 | 是 | 是 | 枚举值 | `ACK,cmd=BTX,v=<FAST|SAFE>` | Core 0 立即 | 否 | 支持 | FAST=Notify；SAFE=Indicate + confirmation。客户端 CCCD 必须匹配。 |
| `WIFI?` | Query | 是 | 是 | 是 | 无 | `ACK,cmd=WIFI,v=<OFF|AP>,ok=<0|1>` | 立即 snapshot | - | 支持 | `ok` 表示 Wi-Fi backend ready，不等于 STA 已连接。 |
| `WIFI=AP` | Config | 是 | 是 | 是 | 无 | `ACK,cmd=WIFI,v=AP,ok=1` 或 `ERR,...` | 调用时初始化/启用 SoftAP | 否 | 支持 | DATA/LOG/CTRL 使用 UDP 3333/3334/3335。 |
| `WIFI=OFF` | Config | 是 | 是 | 是 | 无 | `ACK,cmd=WIFI,v=OFF,ok=1` | Core 0 立即 | 否 | 支持但语义有限 | 关闭 transport 的 Wi-Fi eligibility；当前实现不会销毁已经启动的 AP driver/socket。 |

## Scan 与测量

| Command | Type | Serial | BLE FF10 | Wi-Fi CTRL | Arguments | Response | Apply timing | Persistent | Status | Notes |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `ROWS?` | Query | 是 | 是 | 是 | 无 | `ROWS,active=...,pending=...,requestId=...,appliedId=...,generation=...` | 立即 snapshot | - | 支持 | 不触碰硬件。 |
| `ROWS=<N>` / `ROWLIMIT=<N>` / `SCANROWS=<N>` | Config | 是 | 是 | 是 | `N=1..8` | `RCMD,...status=accepted`，随后 `RAPP,...status=applied` | 下一完整帧开始 | 否 | 支持；后两者为 alias | `RAPP` 后的新 frame 才使用新 rows/generation。 |
| `ROWMODES?` | Query | 是 | 是 | 是 | 无 | `ROWMODES,active=...,pending=...,gen=...,rid=...,rows=8,state=...` | 立即 seqlock snapshot | - | 支持 | 查询每个物理 S 行的当前/待应用 profile。 |
| `ROWMODES=<8 chars>` | Config | 是 | 是 | 是 | 每字节为 `C`/`V`/`R`，例如 `CVVRRVVC` | `RMACK,...state=accepted`，随后 `RMAPP,...profile=...,state=applied` 或 `RMERR` | 下一完整帧开始 | 否 | 支持 | profile 原子切换；同一帧不会混用旧/新 profile。 |
| `MODE?` / `STATE?` | Query | 是 | 是 | 是 | 无 | `MODE,state=...,active=...,route=...` | 立即 seqlock snapshot | - | 支持 | 两个命令当前返回同一完整 snapshot；不发起新 conversion。 |
| `MODE=CAP` / `MODE=CAPACITANCE` | Config | 是 | 是 | 是 | CAP alias | `MACK,...state=accepted`，随后 `MAPP,...state=applied` | 完整帧边界 | 否 | 支持 | 切到双 FDC CAP。 |
| `MODE=VOLT` / `MODE=VOLTAGE` | Config | 是 | 是 | 是 | VOLT alias | `MACK`，随后自动 monitor rail 并输出 `MAPP`；失败输出 `MERR,...state=SAFE` | 完整帧边界 | 否 | 支持 | 不要求普通用户 `RAILCFG`；`SAFE_RAIL_MONITOR` 使用 ADS126x internal analog supply monitor。 |
| `MODE=RES` / `MODE=RESISTANCE` | Config | 是 | 是 | 是 | RES alias | `MACK`，随后 `MAPP` | 完整帧边界 | 否 | 支持 | 切到 ADS + REFOUT 电阻路径。 |
| `RAILCFG=<AVDD_UV>,<negative_AVSS_UV>` | Config | 是 | 是 | 是 | AVDD 正数；AVSS 负数 | `RACK,...state=accepted`，随后 `RAPP,...state=applied`；已在 VOLT 时返回 `ERR,cmd=RAILCFG,reason=apply_before_volt` | 完整帧边界 | 否 | 支持 | 可选外部 DMM debug override；不再是普通 VOLT 前置条件。 |
| `CELL?=S<row>D<col>` | Query | 是 | 是 | 是 | row/col 都为 1..8，例如 `CELL?=S1D1` | `CELL,seq=...,mode=...,value=...,valid=...` | 立即读取最后完整 snapshot | - | 支持 | 不触发新测量。 |
| `RESSETTLE?` | Query | 是 | 是 | 是 | 无 | `RESSETTLE,settleUs=...,phases=2,source=runtime` | 立即 snapshot | - | 支持 | 查询 RES row settle。 |
| `RESSETTLE=<microseconds>` | Config | 是 | 是 | 是 | `0..10000` | `ACK,cmd=RESSETTLE,...status=accepted`，随后 `RESSETTLE,...status=applied|rejected` | 完整帧边界 | 否 | 支持 | 不应把调试 sweep 结果固化为生产校准。 |

## ADS、rail 与 battery

| Command | Type | Serial | BLE FF10 | Wi-Fi CTRL | Arguments | Response | Apply timing | Persistent | Status | Notes |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `BAT?` | Query | 是 | 是 | 是 | 无 | `ABAT,...` | 立即读取 cache | - | 支持 | 使用 latest `valid/fresh/ageMs/reason` 与 `lastGoodMv/lastGoodFresh/lastGoodAgeMs`，不能只看 mV。断电池的浮动/PWM AIN8 属于可解释 invalid diagnostic。 |
| `RAIL?` | Query | 是 | 是 | 是 | 无 | `ARL,...` | 立即读取 cache | - | 支持 | 不发起 rail transaction。 |
| `ADS?` | Query | 是 | 是 | 是 | 无 | `ADS,...` | 立即读取 identity/register snapshot | - | 支持 | 未确认芯片时必须是 `chip=unknown,valid=0`。 |
| `ADSCHK` / `ADSCHK=<samples>` | Action | 是 | 是 | 是 | 默认 100；范围 `1..1000` | `ACK,cmd=ADSCHK,id=...,status=accepted`，随后 `ADSCHK` 与 `ADSCHKSTAT` | 下一完整帧之后 | 否 | 支持 | 主动读 SPI/DRDY 并验证 restore，不复用 measurement cache。 |
| `BATNOW` | Action | 是 | 是 | 是 | 无 | `ACK,cmd=BATNOW,id=...,status=accepted`，随后 `BAPP`/`ABAT` telemetry | 下一安全 gap 或完整帧边界 | 否 | 支持 | 单次普通 battery transaction。 |
| `BATD` / `BATD=VBIAS_ON` / `BATD,MODE=VBIAS_ON` | Action | 是 | 是 | 是 | 等价 alias | `ACK,cmd=BATD,id=...,status=accepted`，随后 `BATD`/`BAPP` | 下一安全完整 transaction 点 | 否 | 支持；后两者为诊断 alias | 强制 VBIAS-on 诊断，仍必须 restore 原 ADS 状态。 |
| `BATPERIOD?` | Query | 是 | 是 | 是 | 无 | `BATPERIOD,enabled=...,periodMs=...,due=...,ageMs=...` | 立即 snapshot | - | 支持 | 周期单位为真实毫秒，不是 frame count。 |
| `BATPERIOD=OFF` | Config | 是 | 是 | 是 | 无 | `ACK,cmd=BATPERIOD,...status=accepted`，随后 `BATPERIOD,...status=applied` | 完整帧边界 | 否 | 支持 | 关闭 periodic scheduler。 |
| `BATPERIOD=<milliseconds>` / `BATPERIOD=ON,<milliseconds>` | Config | 是 | 是 | 是 | `100..600000` | 同上 | 完整帧边界 | 否 | 支持 | 首次/后续 deadline 基于 wall clock。 |
| `FPS?` | Query | 是 | 是 | 是 | 无 | `FPS,cfcap=...,ofcap=...,adsgap=...` | 立即 snapshot | - | 支持 | 查询 capture/output caps 与 ADS gap mode。 |
| `ADSDBG?` | Query | 是 | 是 | 是 | 无 | `ADSDBG,enabled=...,summaryFrames=...` | 立即 snapshot | - | 支持；诊断 | 查询 ADS debug summary。 |
| `ADSDBG=0` / `ADSDBG=1` | Config | 是 | 是 | 是 | bool | `ACK,cmd=ADSDBG,v=<0|1>,status=accepted`，随后 applied event | 完整帧边界 | 否 | 支持；诊断 | 不应作为规避 stack/transport 问题的发布配置。 |

## Legacy / 低频诊断命令

这些命令仍由共享 control handler 接收，但经 legacy mailbox callback 排队。立即回复通常是：

```text
ACK,cmd=LEGACY,state=queued
```

应用后会产生 `E,...type=cmd,...state=applied`。它们不是持久配置。

| Command | Type | Serial | BLE FF10 | Wi-Fi CTRL | Arguments | Response | Apply timing | Persistent | Status | Notes |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `BLECAP=<N>` / `CAP=<N>` | Legacy config | 是 | 是 | 是 | `0..100000` | generic legacy ACK | 完整帧边界 | 否 | 兼容 | 低频 CAP/BLE 周期兼容 alias；不是 `MODE=CAP`。 |
| `TRACE=0` / `TRACE=1` | Diagnostic config | 是 | 是 | 是 | bool | generic legacy ACK | 完整帧边界 | 否 | 诊断 | runtime trace 状态。 |
| `FPSCAP=OFF` / `FPSCAP=ON,<fps>` | Runtime config | 是 | 是 | 是 | fps=`1..200` | generic legacy ACK | 完整帧边界 | 否 | 支持；legacy parser path | acquisition-side pacing；默认 off。 |
| `OUTCAP=OFF` / `OUTCAP=ON,<fps>` | Runtime config | 是 | 是 | 是 | fps=`1..200` | generic legacy ACK | 完整帧边界 | 否 | 支持；legacy parser path | 只限制 Core 0 emitted output，不反压 acquisition。 |
| `ADSGAP=OFF` / `ADSGAP=ON` / `ADSGAP=RAIL` / `ADSGAP=BAT` / `ADSGAP=ZERO` | Diagnostic config | 是 | 是 | 是 | 枚举值 | generic legacy ACK | 完整帧边界 | 否 | 诊断 | 选择允许的 ADS gap job；不能用于掩盖 battery/summary crash。 |
| `CAL=ZERO` | Action | 是 | 是 | 是 | 无 | generic legacy ACK | 请求在安全 ADS slack/boundary 执行 | 否 | 诊断 | zero residual calibration request。 |
| `CAL=RAIL` / `RAILCAL` | Action | 是 | 是 | 是 | alias | generic legacy ACK | 安全 ADS slack/boundary | 否 | 兼容/诊断 | rail calibration request。 |
| `CAL=ALL` / `BATCAL` | Action | 是 | 是 | 是 | alias | generic legacy ACK | 安全 ADS slack/boundary | 否 | 兼容/诊断 | 请求 zero + rail；`BATCAL` 名称保留但不是持久 battery calibration。 |

## 明确未实现的 Wi-Fi STA 命令

以下命令都会返回：

```text
ERR,cmd=WIFI,reason=sta_nyi
```

| Command | Status | Notes |
| --- | --- | --- |
| `WIFI=STA` | NYI | 未实现 STA mode。 |
| `WIFI=APSTA` | NYI | 未实现 AP+STA。 |
| `WIFI_SCAN` | NYI | 未实现 scan。 |
| `WIFI_STA_SSID=<ssid>` | NYI | 未保存 SSID；输入目前还会被命令规范化为大写，不能视为可用凭据接口。 |
| `WIFI_STA_PASS=<password>` | NYI | 未保存 password。 |
| `WIFI_SAVE=1` | NYI | 未写 NVS。 |
| `WIFI_CONNECT=1` | NYI | 未实现连接。 |
| `WIFI_FORGET=1` | NYI | 未实现删除凭据。 |

## 不存在的 console 命令

当前源码没有 `esp_console_cmd_register()`。`force_full_sweep`、`fdc_diag`、`fdc_boot_sweep`、`fdc_rescue`、`fdc_period_ms`、`fdc_profile`、`fdc_i2c_trace` 和 `fdc_discard_frames` 不是用户可调用命令；同名或相近 C API 不代表存在文本命令。

## 错误与回复限制

- 未支持命令：`ERR,cmd=UNKNOWN,reason=unsupported`；
- Serial 对重复未知输入会使用 `CMDERR_SUM` 限流；
- BLE 命令即使已进入 mailbox，如果 `FF11` 未订阅，source-local reply 仍可能发送失败；
- command accepted 不等于硬件 applied；
- BLE control reply/CTRL TX slot contract 最大 512 bytes，当前 handler 也使用有界 512-byte formatter scratch；新增或扩展 snapshot 必须保持在同一 contract 内，不能静默越界。

## 文档一致性检查

命令表由源码 parser literal 反向检查：

```powershell
python tools/check_command_docs.py
```

新增、删除或重命名 command literal 后，未同步本页会使检查失败。

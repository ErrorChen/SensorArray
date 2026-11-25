# 命令参考 / Command reference

命令表列出字面量、入口、参数、回复、应用时机与持久化属性。先区分请求受理、本地应用和首个有效完整帧。 / The tables specify the literal command, transport, arguments, response, timing and persistence. 是 = yes, 否 = no, 无 = none, 立即 = immediate, 帧边界 = frame boundary. Match the terminal by session, request ID and domain. Recover a lost response by read-only query.

## 共同规则 / Command conventions

命令使用 ASCII，解析器规范化大小写及首尾空白。下列入口共用调度和请求仲裁。 / Commands use normalized ASCII text. All inputs use the shared dispatcher and request arbitration.

- 有线端由 `USB?` 确定。双 CDC 的 CDC0 接收命令和回复，CDC1 输出调试。可选 vendor 模式使用 CTRL 端点。旧 Serial/JTAG 使用其串口行输入。 / Wired control uses formal CDC0, the vendor CTRL endpoint, or the legacy Serial/JTAG reader according to the backend.
- BLE 写 `FF10`，回复经独立订阅的 `FF11`。 / Write FF10 and subscribe to FF11 for replies.
- Wi-Fi 控制为 SoftAP UDP `3335`，回复发回原 peer。 / Send UDP control to port 3335 of the active SoftAP and listen on the requesting socket.

先订阅 BLE FF11，再发请求。Wi-Fi CTRL 只有在 SoftAP 及 UDP 可达时才有收发路径。 / A supported command can be admitted even if its reply is subsequently lost.

MODE、ROWS、ROWMODES、RAILCFG 及输出设置驻留 RAM。`CAL=SAVE` 显式写 calib NVS，`CAL=LOAD` 读取。 / These configuration and output settings are volatile. Only an explicit save persists matrix calibration.

## 传输和输出 / Transport and output

USB-OTG keeps the ASCII command/frame grammar. With `usb_cdc_data_v1`, the “Serial” command column maps to **CDC0 formal DATA/CTRL** and CDC1 is debug output only. The optional `usb_vendor_bulk_v1` build instead uses the vendor CTRL endpoint and a single debug CDC. `ST=SER` selects the active wired DATA backend; either OTG backend sends full frames independently of legacy `USBSTREAM`. / 双 CDC 构建中下表 Serial 命令列对应正式 CDC0；CDC1 仅调试输出。可选 vendor 构建仍走独立 CTRL 端点。`ST=SER` 选择当前有线 DATA，OTG 固定完整帧，不受旧 `USBSTREAM` 限流。

| Command | Type | Serial | BLE FF10 | Wi-Fi CTRL | Arguments | Response | Apply timing | Persistent | Status | Notes |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `USB?` | Query | 是 | 是 | 是 | 无 | `USB,backend=...,init=...,mounted=...,dma=...,epoch=...` | 立即快照 | 否 | 支持 | USB stack observations; mounted is SET_CONFIGURATION evidence, not PC receipt. `romAvailable` checks eFuse restrictions without burning them. / 读取 USB 控制器、DTR、设备 epoch 及 ROM 能力。 |
| `USBQ?` | Query | 是 | 是 | 是 | 无 | `USBQ,dDrop=...,cDrop=...,dBytes=...,dCallbackBytes=...` | 立即快照 | 否 | 支持 | Separate whole-record submissions/rejections, FIFO accepted bytes and transfer callback bytes; callback result/PC receipt unknown. / 查询本地整条提交、拒绝、过期、FIFO 接收字节和可归属回调。 |
| `USBBOOT` | Action | 是 | 是 | 是 | 无 | `RACK,cmd=RESTART,id=...,target=rom_download` | ACK 本地入队后，在主循环边界 | 否 | 支持 | Explicit firmware-download restart, not a query. Uses RESTART request identity and ACK gate; owner fences DMA and holds OTG reset before USJ/ROM handoff. Failed local ACK cancels. / 申请 ROM 下载重启并以本地 ACK 接纳开门。 |
| `BLELINK?` | Query | 是 | 是 | 是 | 无 | `BLELINK,gen=...,phyKnown=...,connKnown=...` | 立即快照 | 否 | 支持 | Actual negotiated PHY and connection interval, request/completion errors. DLE completion has no peer ID in IDF; `dlePeerKnown=0` is intentional. / 报告实际 PHY、连接间隔及各字段是否已知。 |
| `BLETX?` | Query | 是 | 是 | 是 | 无 | `BLETX,dDequeued=...,sdkActive=...,sdkMaxUs=...` | 立即快照 | 否 | 支持 | Local priority service and GATT SDK timing; SDK admission does not prove radio or PC completion. / 查询优先发送与 SDK 调用时序。 |

In OTG builds `SERIAL?` explicitly reports `backend=otg_cdc,scope=debug`; `USBSTREAM?` reports `v=CDC` or `v=BULK`, with `scope=otg_data,dataEvery=1` and `control=shared_cdc` or `bulk_ctrl`. `USBSTREAM=DEBUG/FULL` returns `legacy_usj_only` and changes nothing. `USB?` identifies `otg_cdc_v1` or `otg_bulk_v1`; DTR fields report line state, never PC receipt. `USBQ` CDC callback-byte fields are `unknown` because the shared endpoint callback cannot attribute bytes to DATA/CTRL. Legacy Serial/JTAG builds retain their existing fields and commands. `BUILD?` appends running-image `version` and `elfSha256`, not host checkout identity. / OTG 不静默沿用旧 Serial 统计含义；CDC 正式口共享 DATA/CTRL 字节流但只在完整记录边界仲裁。旧 USJ 构建保留既有契约。`BUILD?` 提供运行镜像版本和 ELF SHA256。


| Command | Type | Serial | BLE FF10 | Wi-Fi CTRL | Arguments | Response | Apply timing | Persistent | Status | Notes |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `TX?` | Query | 是 | 是 | 是 | 无 | `ACK,cmd=TX,v=<short\|rel\|full>` | 立即 snapshot | - | 支持 | 查询 transport text mode。 / Read the current text output policy. |
| `TX=REL` / `TX=RT` / `TX=SHORT` / `TX=FULL` | Config | 是 | 是 | 是 | 枚举值 | `ACK,cmd=TX,v=<...>` | Core 0 立即 | 否 | 支持；`RT` 是 `SHORT` alias | 当前 `SHORT/RT` 只把可压缩的 BLE CAP DATA 改为 `B20`；V/R、Wi-Fi、Serial 不因此改帧。`REL` 与 `FULL` 当前都转发完整 payload。 / REL and FULL publish complete records. Ordinary short-mode CAP uses B20. Protected CAP remains full. |
| `ST?` | Query | 是 | 是 | 是 | 无 | `ACK,cmd=ST,v=<...>` | 立即 snapshot | - | 支持 | 查询 sink selection。 / Read the selected output sinks. |
| `ST=AUTO` | Config | 是 | 是 | 是 | 无 | `ACK,cmd=ST,v=auto` | Core 0 立即 | 否 | 支持 | Serial 始终 eligible；BLE 还需 connected 且当前 DATA/LOG channel 的 CCCD 与 FAST/SAFE 兼容；Wi-Fi 还需 mode enabled 且 ready。 / AUTO selects the wired path and eligible BLE/Wi-Fi sinks. |
| `ST=SER` | Config | 是 | 是 | 是 | 无 | `ACK,cmd=ST,v=ser` | Core 0 立即 | 否 | 支持 | 只选 Serial。 / Select only the active wired backend. |
| `ST=BLE` | Config | 是 | 是 | 是 | 无 | `ACK,cmd=ST,v=ble` | Core 0 立即 | 否 | 支持 | 只希望发送 BLE；仍要求 connected 和对应 characteristic 已订阅，不能把未订阅 channel 入队。 / Select an eligible, subscribed BLE channel. |
| `ST=WIFI` | Config | 是 | 是 | 是 | 无 | `ACK,cmd=ST,v=wifi` | Core 0 立即 | 否 | 支持 | 只选择 Wi-Fi 且要求 mode 非 OFF；producer 不以 ready 为前置，consumer 发送前才检查 ready。先用 `WIFI?` 确认 `ok=1`。 / Select the Wi-Fi data sink. Check the AP gate and backend. |
| `ST=ALL` | Config | 是 | 是 | 是 | 无 | `ACK,cmd=ST,v=all` | Core 0 立即 | 否 | 支持 | Serial + BLE + Wi-Fi；BLE 严格按 channel/CCCD gating，Wi-Fi mode 非 OFF 即可入队并在 consumer 发送前检查 ready。 / Select every eligible sink. |
| `BTX?` | Query | 是 | 是 | 是 | 无 | `ACK,cmd=BTX,v=<FAST\|SAFE>` | 立即 snapshot | - | 支持 | 查询 BLE TX mode。 / Read BLE notify/indicate policy. |
| `BTX=FAST` / `BTX=SAFE` | Config | 是 | 是 | 是 | 枚举值 | `ACK,cmd=BTX,v=<FAST\|SAFE>` | Core 0 立即 | 否 | 支持 | FAST=Notify；SAFE 优先 Indicate + confirmation，Notify-only 客户端使用有界 Notify fallback。 / Set the required CCCD bits before switching. |
| `WIFI?` | Query | 是 | 是 | 是 | 无 | `ACK,cmd=WIFI,v=...,ok=...,ver=2,id=...,outputGate=...,backend=...,pending=...` | 立即 snapshot | 否 | `wifi_owner_v1` | 已应用的 OFF 可为 ok=1。AP 的 ok 还要求 gate 开、backend ready、未隔离。 / Query requested mode, output eligibility and backend separately. |
| `WIFI=AP` | Config | 是 | 是 | 是 | 无 | `ACK,cmd=WIFI,v=AP,ok=...,ver=2,accepted=1,pending=...,id=...` 或 ERR，随后 WTERM | 异步 / Asynchronous | 否 | `wifi_owner_v1` | owner 启动 SoftAP/UDP。用 WIFIREQ?<id> 查精确结果。 / Local admission may precede AP readiness. DATA/LOG/CTRL use UDP 3333/3334/3335. |
| `WIFI=OFF` | Config | 是 | 是 | 是 | 无 | `ACK,cmd=WIFI,v=OFF,ok=...,ver=2,accepted=1,id=...` 或 ERR，随后 WTERM | 输出 gate 切换 / Gate change | 否 | `wifi_owner_v1` | 关闭 Wi-Fi 输出资格，驱动物理状态为 unknown。 / OFF controls the output gate. Inspect the exact request for outcome. |

## 扫描和测量 / Scan and measurement

启动/恢复诊断：`SPI?` 返回本 boot 的原子选速率结果，包括 requested
`hz`、驱动观测 `actualHz`、`qualified/restored/joined/timedOut`、全部候选累计
`attempted/samples/crc` 和 SPI/I²C `overlapUs`。累计错误率不是选中档位错误率，
后者看逐 profile/phase 的 `SPI_SWEEP`。`FRONTENDS?` 返回三芯片探测快照、
`hold`、各设备错误/清理错误和 `aHz/aRestoreHz`；`sdInhibited` 表示不能安全
开始探测（含 SD 未知），不等于实测 SD 高电平。两者在 READY 前也可查询，
不触发硬件 I/O，支持 Serial/BLE/Wi-Fi，均不写持久配置。详见
[启动与恢复接口 / Startup and recovery](integration/firmware_host_contract.md)。

`hold=1` 时暂停测量，MODE/ROWS/ROWMODES 及辅助测量操作会明确拒绝；查询、
输出控制和 RESTART 仍可用。通信稳定恢复触发受预算约束的整机重启；重新
连接后按新的 session 查询 CONFIG，旧 RAM rail 校准和旧帧身份不能沿用。 / SPI? and FRONTENDS? can be queried before READY. During hold, queries, output controls and explicit restart remain available. Restart requalifies the frontends and invalidates volatile calibration.

| Command | Type | Serial | BLE FF10 | Wi-Fi CTRL | Arguments | Response | Apply timing | Persistent | Status | Notes |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `CONFIG?` | Query | 是 | 是 | 是 | 无 | `CONFIG,session=...,bootId=...,rev=...,boundary=...,mgen=...,rgen=...,pgen=...` | 原子权威 snapshot | - | 支持 | 用于终态丢失/迟到后的重建；各域 ID 和最后 MODE/profile 终态来自同一 revision。 / Read the atomic configuration and last per-domain terminals after a lost or late reply. |
| `ROUTE?` | Query | 是 | 是 | 是 | 无 | `ROUTE,requested=...,applied=...,known=...,verified=...,physicalSafe=unknown` | route snapshot | - | 支持 | verified 只表示寄存器/GPIO 证据；不表示模拟隔离实测。包含 originalError、cleanupError 和 railMonitor capability。 / Read software intent, applied route, digital evidence and cleanup status. Analog isolation remains unknown. |
| `ROWS?` | Query | 是 | 是 | 是 | 无 | `ROWS,active=...,pending=...,requestId=...,appliedId=...,generation=...` | 立即 snapshot | - | 支持 | 不触碰硬件。 / Read active/pending row count and the last applied row ID. |
| `ROWS=<N>` / `ROWLIMIT=<N>` / `SCANROWS=<N>` | Config | 是 | 是 | 是 | `N=1..8` | `RCMD,...status=accepted`，随后 `RAPP,...status=applied` | 下一完整帧开始 | 否 | 支持；后两者为 alias | `RAPP` 后的新 frame 才使用新 rows/generation。 / Apply 1–8 active physical S rows at a complete frame boundary. |
| `ROWMODES?` | Query | 是 | 是 | 是 | 无 | `ROWMODES,active=...,pending=...,gen=...,rid=...,rows=8,state=...` | 立即 seqlock snapshot | - | 支持 | 查询每个物理 S 行的当前/待应用 profile。 / Read active/pending eight-row profiles. |
| `ROWMODES=<8 chars>` | Config | 是 | 是 | 是 | 每字节为 `C`/`V`/`R`，例如 `CVVRRVVC` | `RMACK,...state=accepted`，随后 `RMAPP,...profile=...,state=applied` 或 `RMERR` | 下一完整帧开始 | 否 | 支持 | profile 原子切换；同一帧不会混用旧/新 profile。 / Set exactly eight C/V/R letters. The profile becomes active at a frame boundary. |
| `MODE?` / `STATE?` | Query | 是 | 是 | 是 | 无 | `MODE,state=...,active=...,route=...,fdcSd=<high\|low>,fdcSdVerified=<0\|1>,fdcRestartRequired=<0\|1>` | 立即 seqlock snapshot | - | 支持 | 两个命令当前返回同一完整 snapshot；不发起新 conversion。`fdcSd` 与 `FDCISO?` 一致；高电平表示共享 FDC SD 已关断，必须 `RESTART` 才能恢复 CAP。 / Read the current mode, route, shutdown and restart requirement. |
| `MODE=CAP` / `MODE=CAPACITANCE` | Config | 是 | 是 | 是 | CAP alias | `MACK,...state=accepted`，随后 `MAPP,...state=applied` | 完整帧边界 | 否 | 支持 | 切到双 FDC CAP。 / Select parallel-FDC capacitance acquisition. |
| `MODE=VOLT` / `MODE=VOLTAGE` | Config | 是 | 是 | 是 | VOLT alias | `MACK`，随后 `MAPP` 或 `MERR` | 完整帧边界 | 否 | 支持 | 当前板型无透明 monitor 隔离能力；需合法外部 rail 校准，不能秘密施加参考。 / Select voltage acquisition after valid rail calibration. |
| `MODE=RES` / `MODE=RESISTANCE` | Config | 是 | 是 | 是 | RES alias | `MACK`，随后 `MAPP` | 完整帧边界 | 否 | 支持 | 切到 ADS + REFOUT 电阻路径。 / Select resistance with the ADS REFOUT excitation path. |
| `RAILCFG=<AVDD_UV>,<negative_AVSS_UV>` | Config | 是 | 是 | 是 | AVDD 正数；AVSS 负数 | `RACK,...state=accepted`，随后 `RAPP,...state=applied`；已在 VOLT 时返回 `ERR,cmd=RAILCFG,reason=apply_before_volt` | 完整帧边界 | 否 | 支持 | 外部 DMM 实测校准；当前板型进入 VOLT 的安全路径，不按帧数自动过期，掉电丢失。 / Provide both DMM readings in µV before VOLT. Values are volatile and have no frame-age expiry. |
| `CELL?=S<row>D<col>` | Query | 是 | 是 | 是 | row/col 都为 1..8，例如 `CELL?=S1D1` | `CELL,seq=...,mode=...,value=...,valid=...` | 立即读取最后完整 snapshot | - | 支持 | 不触发新测量。 / Read the requested physical cell from the last frame. This query performs no conversion. |
| `RESSETTLE?` | Query | 是 | 是 | 是 | 无 | `RESSETTLE,settleUs=...,phases=2,source=runtime` | 立即 snapshot | - | 支持 | 查询 RES row settle。 / Read resistance row settling time. |
| `RESSETTLE=<microseconds>` | Config | 是 | 是 | 是 | `0..10000` | `ACK,cmd=RESSETTLE,...status=accepted`，随后 `RESSETTLE,...status=applied\|rejected` | 完整帧边界 | 否 | 支持 | 不应把调试 sweep 结果固化为生产校准。 / Apply a resistance row settling delay in µs at a boundary. |
| `FDCISO?` | Query | 是 | 是 | 是 | 无 | `FDCISO,sd=<high\|low>,verified=<0\|1>,restartRequired=<0\|1>` | 立即 seqlock snapshot | - | 支持；实验诊断 | 只记录命令施加的 FDC SD 电平；不能证明模拟隔离。 / Read FDC shutdown level and available digital verification. |
| `FDCISO=ON` | Config | 是 | 是 | 是 | 无 | `FACK,id=...,sd=low,state=accepted,restartRequired=0`，随后 `FAPP,id=...,seq=...,sd=high,verified=1,restartRequired=1,state=applied` 或 `FERR` | 完整帧边界 | 否 | 支持；实验诊断 | 仅在 route 非 SAFE、route.mode 与 active homogeneous RES/VOLT 一致且无 pending row profile 时允许；先停止 ADS 并 read-back 验证 FDC sleep，再驱动共享 SD=GPIO8 高。Schematic 显示 U8/U9 SD 与 ESP32 IO8 共享 SD 网，未看到 pull-down；SD 高请求 FDC shutdown，恢复 CAP 必须 `RESTART`。 / Request shared FDC shutdown for eligible homogeneous RES/VOLT. A subsequent CAP return requires RESTART. |
| `FDCISO=OFF` | Config | 是 | 是 | 是 | 无 | SD 高时 `FERR,cmd=FDCISO,sd=high,reason=restart_required,state=rejected,restartRequired=1`；SD 低时 `FACK,...state=unchanged` | 无硬件动作 | 否 | 支持；显式拒绝 | 不假装可在不重新初始化 FDC 的情况下恢复 CAP；只能 `RESTART`。 / Return unchanged when SD is low, or reject with restart_required when high. |

## ADC、供轨和电池 / ADC, rails and battery

| Command | Type | Serial | BLE FF10 | Wi-Fi CTRL | Arguments | Response | Apply timing | Persistent | Status | Notes |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `BAT?` | Query | 是 | 是 | 是 | 无 | `ABAT,...` | 立即读取 cache | - | 支持 | 使用 latest `valid/fresh/ageMs/reason` 与 `lastGoodMv/lastGoodFresh/lastGoodAgeMs`，不能只看 mV。断电池的浮动/PWM AIN8 属于可解释 invalid diagnostic。 / Read latest and last-good battery status with validity, age and restore evidence. |
| `RAIL?` | Query | 是 | 是 | 是 | 无 | `ARL,...` | 立即读取 cache | - | 支持 | 不发起 rail transaction。 / Read cached rail source, split and validity. |
| `ADS?` | Query | 是 | 是 | 是 | 无 | `ADS,...` | 立即读取 identity/register snapshot | - | 支持 | 未确认芯片时必须是 `chip=unknown,valid=0`。 / Read cached converter identity and register evidence. |
| `ADSCHK` / `ADSCHK=<samples>` | Action | 是 | 是 | 是 | 默认 100；范围 `1..1000` | `ACK,cmd=ADSCHK,id=...,status=accepted`，随后 `ADSCHK` 与 `ADSCHKSTAT` | 下一完整帧之后 | 否 | 支持 | 主动读 SPI/DRDY 并验证 restore，不复用 measurement cache。 / Run the active ADS test at a boundary. Check its restoration result. |
| `BATNOW` | Action | 是 | 是 | 是 | 无 | `ACK,cmd=BATNOW,id=...,status=accepted`，随后 `BAPP`/`ABAT` telemetry | 下一安全 gap 或完整帧边界 | 否 | 支持 | 单次普通 battery transaction。 / Request one battery sample transaction. |
| `BATD` / `BATD=VBIAS_ON` / `BATD,MODE=VBIAS_ON` | Action | 是 | 是 | 是 | 等价 alias | `ACK,cmd=BATD,id=...,status=accepted`，随后 `BATD`/`BAPP` | 下一安全完整 transaction 点 | 否 | 支持；后两者为诊断 alias | 强制 VBIAS-on 诊断，仍必须 restore 原 ADS 状态。 / Request a VBIAS-on battery diagnostic with restoration. |
| `BATPERIOD?` | Query | 是 | 是 | 是 | 无 | `BATPERIOD,enabled=...,periodMs=...,due=...,ageMs=...` | 立即 snapshot | - | 支持 | 周期单位为真实毫秒，不是 frame count。 / Read the wall-clock battery period and scheduler state. |
| `BATPERIOD=OFF` | Config | 是 | 是 | 是 | 无 | `ACK,cmd=BATPERIOD,...status=accepted`，随后 `BATPERIOD,...status=applied` | 完整帧边界 | 否 | 支持 | 关闭 periodic scheduler。 / Disable periodic battery sampling. |
| `BATPERIOD=<milliseconds>` / `BATPERIOD=ON,<milliseconds>` | Config | 是 | 是 | 是 | `100..600000` | 同上 | 完整帧边界 | 否 | 支持 | 首次/后续 deadline 基于 wall clock。 / Set the periodic sample interval in milliseconds. |
| `FPS?` | Query | 是 | 是 | 是 | 无 | `FPS,cfcap=...,ofcap=...,adsgap=...` | 立即 snapshot | - | 支持 | 查询 capture/output caps 与 ADS gap mode。 / Read acquisition and output pacing separately. |
| `ADSDBG?` | Query | 是 | 是 | 是 | 无 | `ADSDBG,enabled=...,summaryFrames=...` | 立即 snapshot | - | 支持；诊断 | 查询 ADS debug summary。 / Read the ADS diagnostic output settings. |
| `ADSDBG=0` / `ADSDBG=1` | Config | 是 | 是 | 是 | bool | `ACK,cmd=ADSDBG,v=<0\|1>,status=accepted`，随后 applied event | 完整帧边界 | 否 | 支持；诊断 | 不应作为规避 stack/transport 问题的发布配置。 / Request a boundary-applied summary toggle. |

## 系统和生命周期 / System and lifecycle

| Command | Type | Serial | BLE FF10 | Wi-Fi CTRL | Arguments | Response | Apply timing | Persistent | Status | Notes |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `BOOT?` | Query | 是 | 是 | 是 | 无 | `BOOT,boot=...,bootId=...,reset=...,stage=...,err=0x...,seq=...,heap=...,heapMin=...,prevStage=...,prevErr=0x...,prevHeap=...,guard=...,autoRestarts=...,windowAgeS=...,ready=...` | 立即 snapshot | - | 支持 | boot breadcrumb 与 auto-restart guard 状态；`guard=recovery_safe` 是自动重启预算耗尽的历史 guard 名称；不证明 GPIO/模拟节点已安全。 / Read current/previous boot breadcrumbs and restart budget. |
| `READY?` | Query | 是 | 是 | 是 | 无 | `READY,ready=<0\|1>,stage=...,err=0x...,bootId=...,boot=...` | 立即 snapshot | - | 支持 | 只反映系统 readiness，不发起任何初始化。 / Read software readiness. |
| `PROTO?` | Query | 是 | 是 | 是 | 无 | `PROTO,version=...,wires=ascii,ctrlMax=...,dataMax=...,channels=CTRL/DATA/LOG/LIFECYCLE` | 立即 snapshot | - | 支持 | 返回当前协议版本与 wire 上限；`ctrlMax`/`dataMax` 是完整 message byte 上限。 / Negotiate wire version, message ceilings and capabilities. |
| `BUILD?` | Query | 是 | 是 | 是 | 无 | `BUILD,idf=...,target=...,project=SensorArray,proto=...` | 立即 snapshot | - | 支持 | 报告构建时的 ESP-IDF 版本、target 与协议版本。 / Read running IDF/target/project, image version and ELF SHA256. |
| `PERF?` | Query | 是 | 是 | 是 | 无 | `PERF,pub=...,fresh=...,stale=...,mixed=...,dropOut=...,dropEvent=...,startAvgUs=...,startN=...,usbSent=...,usbDrop=...,usbBlock=...,usbBytes=...,usbWriteMaxUs=...,usbQMax=...,usbStackMin=...,lifePub=...,lifeDrop=...,queueDrop=...,frames=...,heap=...,heapMin=...` | 立即 snapshot | - | 支持 | 运行时 async log、USB sink 与 transport 统计快照。 / Read local publication, heap and transport counters. Verify host receipt separately. |
| `USBSTREAM?` | Query | 是 | 是 | 是 | 无 | `USBSTREAM,v=CDC` 或 `BULK,scope=otg_data,dataEvery=1`，旧 USJ 为 DEBUG/FULL | 立即 snapshot | 否 | 支持 | 按运行 backend 解码。 / OTG reports formal data role and control path. Legacy reports its serial publication profile. |
| `USBSTREAM=DEBUG` | Config | 是 | 是 | 是 | 无 | 旧 USJ 返回 `USBSTREAM,v=DEBUG,...,state=applied`；OTG 返回 `ERR,cmd=USBSTREAM,reason=legacy_usj_only,...` | 立即 / Immediate | 否 | 限旧 USJ / Legacy USJ | 分别按 dataEvery、diagEvery 抽取。 / Decimates legacy DATA and diagnostics. |
| `USBSTREAM=FULL` | Config | 是 | 是 | 是 | 无 | 旧 USJ 返回 `USBSTREAM,v=FULL,...,state=applied`；OTG 返回 `ERR,cmd=USBSTREAM,reason=legacy_usj_only,...` | 立即 / Immediate | 否 | 限旧 USJ / Legacy USJ | 正式 OTG 有独立发布规则。 / Legacy FULL enables all eligible science frames. |
| `RECOVER` / `RECOVER=<level>` | Action | 是 | 是 | 是 | 默认 full；`0..2`（0=soft，1=full，2=restart） | `RACK,cmd=RECOVER,id=...,level=...,state=accepted`，随后 `RAPP,cmd=RECOVER,id=...,level=...,state=applied\|rejected\|restarting` | 下一安全完整帧边界 | 否 | 支持 | 仅同一 session/boot/配置 revision 的已接受请求可执行；`RECOVER=2` 受重启预算约束，耗尽则 DEGRADED/NONE 并记录 cleanup，不能保证重启。`FDCISO=ON` 隔离后 RECOVER 直接拒绝（`reason=fdc_restart_required`），只能 `RESTART`。 / Request soft/full/budgeted restart recovery using levels 0/1/2 and match its terminal. |
| `RECOVERY?` | Query | 是 | 是 | 是 | 无 | `RECOVERY,session=...,bootId=...,busy=...,phase=...,pendingId=...,lastId=...,last=...,originalError=...,cleanupError=...` | 立即 cache snapshot | 否 |  `recovery_query_v1` | 无副作用；同一 boot 最后一个已接受 RECOVER 的终态与当前 pending/running；终态丢失后可查询，不能跨 session 比较 ID。 / Recover the last accepted recovery result without executing again. |
| `SERIAL?` | Query | 是 | 是 | 是 | 无 | `SERIAL,normalSubmitted=...,prioritySubmitted=...,dataSubmitted=...,logSubmitted=...,dataRejected=...,logRejected=...,normalExpired=...,priorityExpired=...,priorityRejected=...,driverShortWrite=...,driverWriteError=...,txYield=...,maxQueueAgeUs=...,driverTxBufferBytes=...,driverAcceptedBytes=...,partialWrite=...` | 立即统计 snapshot | 否 |  `serial_sink_v1`，分类字段 `serial_classes_v1` | 单次原子串口统计；submitted/accepted 只表示本地驱动接受，非 PC 接收。独立于 USBSTREAM；按通路区分科学帧与调试。 / For legacy USJ read per-class counters. For OTG the same command returns a debug-only CDC record. |
| `SCHED?` | Query | 是 | 是 | 是 | 无 | `SCHED,initError=...,registered=...,idleKnown=...,tickHz=...,budgetTicks=...,maxUnits=...,graceUnits=...,backstopTicks=...,quantumTicks=...,idleGen=...,idleAgeTicks=...,maxIdleGapTicks=...,bt=.../.../...,tr=.../.../...,log=.../.../...,ser=.../.../...,bc=.../.../...,sc=.../.../...,wc=.../.../...` | 立即观测 snapshot | 否 |  `work_budget_v1/work_quantum_v2` | CPU0 真实 idle hook 与各 worker 的 checkpoint/yield/maxBurst 三元组；字段独立取样，不是 CPU 利用率或原子配置。未知 idle 年龄为 unknown。quantumTicks=2 为本地工作预算，budgetTicks/backstopTicks=4 为真实 idle 保护，graceUnits=0；主动等待不冒充 idle。 / Read work-budget configuration, idle evidence and per-role budget counters. |
| `RESTART?` | Query | 是 | 是 | 是 | 无 | `RESTART,session=...,bootId=...,busy=...,phase=...,pendingId=...,lastId=...,last=...,reason=...,originalError=...,publishError=...,terminalQueued=...,serialQueued=...,serialError=...,netQueued=...,netError=...` | 立即 cache snapshot | 否 |  `restart_query_v1` | 无副作用；独立的显式重启请求/最后结果，不占 MODE/RECOVER 槽。新 boot 清缓存，restarting 不是 reset/Host 接收证明。 / Read the current restart request and its last local outcome. |
| `RESTART` | Action | 是 | 是 | 是 | 无 | `RACK,cmd=RESTART,id=...,state=accepted`，随后 `RAPP,cmd=RESTART,id=...,state=restarting,kind=manual` | 下一安全完整帧边界 | 否 | 支持 | 独立单 pending，重复返回 `ERR,cmd=RESTART,reason=pending`；本地 ACK 发布失败则取消，查询 RESTART? 可见拒绝原因。受控重启；自动 HIL 未执行受控 reset 流程时 reboot/recovery 必须标 `NOT RUN`/`BLOCKED`。 / Request an explicit reboot. Verify the new session after reconnect. |

`RACK/RAPP` 用于 `RECOVER`/`RESTART` 的 deferred lifecycle：RACK 的本地排队成功才开放 RECOVER 执行，RAPP（或既有前置拒绝 RERR）的 `applied/rejected/restarting` 是终态或重启意图，均不证明 Host 已收到。RECOVER 在 ACK 前与 MODE/ROWS/ROWMODES 共用一个有界终态保留槽；占用时后续请求返回 busy/pending，不覆盖已接受请求。同一 RECOVER 文本再次提交是新请求，忙时拒绝；查询结果应使用无副作用 `RECOVERY?`，不能用重发 RECOVER 代替查询。

RECOVERY 缓存只保留当前请求和本 boot 最后一个已接受请求的结果；`acceptedRev` 属于接受时配置，`rev/boundary` 属于完成时原子配置。任何 session/boot/revision 或 mode/rows/profile 域变化使旧请求终结为 `stale_config`，不操作更新配置。`terminalQueued=1` 仅说明终态曾进入固件保留槽；槽已排空、断连或对端丢失后仍保持 1，不能当接收确认。失败和预算耗尽分别保存 `originalError` 与 `cleanupError`，`physicalSafe=unknown`；cleanup 失败时结果的 routeApplied/routeKnown/routeVerified 都为 0。`BOOT.guard=recovery_safe` 是兼容旧字段，不能代替这些错误与 route 证据。

`RECOVER=2` 只有预算允许才请求重启，`RESTART` 是显式手动重启。`state=restarting` 只是有界排队后发起重启的意图，不能保证对端在 reset 前收到；新 session 的 RST/BOOT/CONFIG 才是重启后的权威身份。所有查询回复检查完整 snprintf 结果，容量不足返回错误，不输出截断统计。新字段详细单位和回绕规则见 [firmware_host_contract.md](integration/firmware_host_contract.md)。

## 兼容命令与低频诊断 / Legacy controls and diagnostics

这些命令仍由共享 control handler 接收，但经 legacy mailbox callback 排队。立即回复通常是：

```text
ACK,cmd=LEGACY,state=queued
```

应用后会产生 `E,...type=cmd,...state=applied`。 / The acquisition mailbox emits the applied event later. These settings are volatile.

| Command | Type | Serial | BLE FF10 | Wi-Fi CTRL | Arguments | Response | Apply timing | Persistent | Status | Notes |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `BLECAP=<N>` / `CAP=<N>` | Legacy config | 是 | 是 | 是 | `0..100000` | generic legacy ACK | 完整帧边界 | 否 | 兼容 | 低频 CAP/BLE 周期兼容 alias；不是 `MODE=CAP`。 / Change the legacy CAP/BLE period control. |
| `TRACE=0` / `TRACE=1` | Diagnostic config | 是 | 是 | 是 | bool | generic legacy ACK | 完整帧边界 | 否 | 诊断 | runtime trace 状态。 / Set runtime trace output. |
| `FPSCAP=OFF` / `FPSCAP=ON,<fps>` | Runtime config | 是 | 是 | 是 | fps=`1..200` | generic legacy ACK | 完整帧边界 | 否 | 支持；legacy parser path | acquisition-side pacing；默认 off。 / Limit acquisition FPS. |
| `OUTCAP=OFF` / `OUTCAP=ON,<fps>` | Runtime config | 是 | 是 | 是 | fps=`1..200` | generic legacy ACK | 完整帧边界 | 否 | 支持；legacy parser path | 只限制 Core 0 emitted output，不反压 acquisition。 / Limit published output FPS independently of acquisition. |
| `ADSGAP=OFF` / `ADSGAP=ON` / `ADSGAP=RAIL` / `ADSGAP=BAT` / `ADSGAP=ZERO` | Diagnostic config | 是 | 是 | 是 | 枚举值 | generic legacy ACK | 完整帧边界 | 否 | 诊断 | 选择允许的 ADS gap job；不能用于掩盖 battery/summary crash。 / Select eligible auxiliary ADS work. |
| `CAL=ZERO` | Action | 是 | 是 | 是 | 无 | generic legacy ACK | 请求在安全 ADS slack/boundary 执行 | 否 | 诊断 | zero residual calibration request。 / Request a zero-residual calibration. |
| `CAL=RAIL` / `RAILCAL` | Action | 是 | 是 | 是 | alias | generic legacy ACK | 安全 ADS slack/boundary | 否 | 兼容/诊断 | rail calibration request。 / Request a rail calibration under available hardware capability. |
| `CAL=ALL` / `BATCAL` | Action | 是 | 是 | 是 | alias | generic legacy ACK | 安全 ADS slack/boundary | 否 | 兼容/诊断 | 请求 zero + rail；`BATCAL` 名称保留但不是持久 battery calibration。 / Request zero and rail calibration. |

## 校准持久化 / Calibration persistence

matrix calibration 显式持久化到独立 `calib` NVS 分区。记录包含
`magic/schemaVersion/boardId/hardwareRevision/source/payloadLength/payload/CRC32`；
固件没有 board revision source，因此当前固定使用
`boardId=0x53415231`、`hardwareRevision=1`。`CAL=LOAD` 对缺失、CRC、
schema、board/hardware revision 或 payload 校验失败一律保持
default/uncalibrated，不会安装半有效记录；保存只发生在显式 `CAL=SAVE`，
普通帧采集不会写 NVS。 / CAL=SAVE writes the matrix record to the dedicated calib partition. CAL=LOAD validates magic, schema, board/hardware identity, payload and CRC before applying. An invalid record leaves the default uncalibrated state.

| Command | Type | Serial | BLE FF10 | Wi-Fi CTRL | Arguments | Response | Apply timing | Persistent | Status | Notes |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `CAL?` | Query | 是 | 是 | 是 | 无 | `CAL,source=...,schema=...,valid=...,boardId=...,hardwareRev=...,payloadLength=...` | 立即 | 否 | 支持 | 只读查询 calib 记录状态；无记录时 `source=0`、`valid=0`。 / Inspect the calibration record and provenance. |
| `CAL=SAVE` | Action | 是 | 是 | 是 | 无 | `ACK,cmd=CAL,v=SAVE,status=accepted`，随后 `CALSV,...` | 完整帧边界（acquisition core） | 是 | 支持 | 显式将当前 matrix calibration 写入 calib NVS。 / Explicitly persist the current matrix calibration. |
| `CAL=LOAD` | Action | 是 | 是 | 是 | 无 | `ACK,cmd=CAL,v=LOAD,status=accepted`，随后 `CALLD,...` | 完整帧边界（acquisition core） | 否 | 支持 | 从 calib NVS 加载并应用；任何 mismatch 保持 default/uncalibrated。 / Load and validate the stored calibration before use. |

## 未实现的 STA 命令 / Unsupported station commands

以下命令均返回此错误。 / These recognized station commands return the same explicit error.

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

## Console 入口 / Console names

当前源码没有 `esp_console_cmd_register()`。`force_full_sweep`、`fdc_diag`、`fdc_boot_sweep`、`fdc_rescue`、`fdc_period_ms`、`fdc_profile`、`fdc_i2c_trace` 和 `fdc_discard_frames` 不是用户可调用命令；同名或相近 C API 不代表存在文本命令。 / These are internal names. Use the exposed text commands in this reference.

## 错误与回复 / Errors and replies

- 未支持命令：`ERR,cmd=UNKNOWN,reason=unsupported`；
- Serial 对重复未知输入会使用 `CMDERR_SUM` 限流；
- BLE 命令即使已进入 mailbox，如果 `FF11` 未订阅，source-local reply 仍可能发送失败；
- command accepted 不等于硬件 applied；
- BLE control reply/CTRL TX slot contract 最大 512 bytes，当前 handler 也使用有界 512-byte formatter scratch；扩展 snapshot 必须保持在同一 contract 内，不能静默越界。

## 文档一致性 / Parser consistency

在仓库根目录运行 parser 对照检查。 / Run this source-to-document check from the repository root.

```powershell
python tools/check_command_docs.py
```

新增、删除或重命名 command literal 后，未同步本页会使检查失败。

## 协议 v2 与能力 / Protocol v2 and capabilities

`PROTO?` 公布 `config_identity_v1/config_query_v1/session64`、`legacyGen=tag_domain`、`dataMax=2304`、`railMonitor=unavailable` 和 `adcCrc=crc8`。旧 `gen/rid` 含义不变；请按 [固件主机契约](integration/firmware_host_contract.md) 更新接收端，禁止跨域/跨 session 比较大小。`MACK`/`RCMD`/`RMACK` 表示接受，Applied 指配置边界。 / Discover PROTO v2 capabilities and match each legacy generation to its own domain. Validate a complete matching frame to establish host receipt.

旧 Serial/JTAG 的 `USBSTREAM?` 包含 sinkQueued、sinkDropped、priorityQueued、driverAcceptedBytes、rxBytes、partialWrite、timeout、driverDrained、rxOversize、lastRxUs/lastTxUs、driverInstalled、sofPresent。SOF 只表示驱动观察到 USB 帧活动；enumerated 和 pcReceived 保持 unknown。计数取差值比较同一 session 的区间，不能把历史累计 drop 当作每帧实时丢弃。DEBUG/FULL 调整旧 USJ 的 DATA/DIAG 抽取。命令 RX 和优先终态持续可用。 / Legacy USJ reports SOF and local queue/driver counters. Use same-session deltas. On OTG, query USB?, USBQ? and the debug-only SERIAL? page.

## 诊断命令与能力页 / Diagnostic commands and capability pages

下列命令经各专用 handler 分发，格式与作用域以 PROTO? 和支持的能力页为准。先查后执行，保留原始错误和单独的恢复结果。 / Query capabilities first. Read-only pages return cached or bounded snapshots. A diagnostic transaction is a separate state-changing operation.

| Command | Arguments | Response / scope | 调用与解释 / Use |
|---|---|---|---|
| `USB?`, `USBQ?` | 无 / none | USB, USBQ | 查询实际 USB 角色、队列容量、期限和本地计数。 / Inspect controller/interface state and bounded per-class delivery statistics. |
| `USBBOOT` | 无 / none | reboot intent | 已授权下载时使用，随后核对 ROM 枚举。 / Requests ROM handoff. Confirm the actual download device at the host. |
| `CAPS2?` | 无 / none | capability page | 查询额外能力（含 Wi-Fi owner）。 / Discover additional runtime extensions. |
| `MEM?`, `MEMFAIL?` | 无 / none | MEM, MEMFAIL | 查看内部/外部 heap 和本次启动所观测到的分配失败。 / Query heap capabilities and boot-scoped allocation-failure observations. |
| `PANIC?`, `PANIC?1`, `PANIC?2` | page 0/1/2 | PANIC | 区分当前查询 boot 和故障源 boot，按 ELF 解析。 / Read retained metadata and bounded detail pages for the original panic image. |
| `CPUOWNER?<role>`, `SCHEDWHY?` | role=0…7，无空格 / no spaces | CPUOWNER, SCHEDWHY | 例如 `CPUOWNER?7`，必须有一位角色编号。`SCHEDWHY?` 单独查让出原因。 / Request exactly one role digit. Roles 0–7 are BLE_TX, TRANSPORT, ASYNC_LOG, SERIAL_TX, BLE_CTRL, SERIAL_CTRL, WIFI_CTRL, WIFI_OWNER. |
| `BLELINK?`, `BLETX?`, `BLESVC?` | 无 / none | link / TX / service | 查询链路、分片与本地服务证据。 / Inspect connection, fragmentation and local service. |
| `BLEFLOW?`, `BLESNAP?`, `BLEGUARD?`, `BLELOG?` | 无 / none | local diagnostics | 普通发送率、保护完成、SDK 等待和普通日志准入。 / Query adaptive ordinary rate, protected completions, SDK waits and log admission. |
| `SNAPSRC?`, `SNAPNET?` | 无 / none | protected pipeline | 各层对保护帧的准入/过期统计。 / Query source and transport protection stages. |
| `WIFIGUARD?`, `WIFIQ?D`, `WIFIQ?L`, `WIFIQ?C`, `WIFIREQ?`, `WIFIREQ?<id>` | channel or decimal uint32 ID | guard / queue / cached request | 无参返回最近一次，ID 非零时精确查询。 / Use D/L/C for channel counts. With no suffix, WIFIREQ? uses the latest request ID. With a suffix, use decimal 1…4294967295. |
| `SPISOURCE?`, `SPI?`, `FRONTENDS?` | 无 / none | boot probe / presence | 读取已发布的启动选速、SCLK 来源和前端存在位。 / These read-only snapshots report boot probe evidence and frontend state. `SPISOURCE?` carries `runtime=unobserved,physicalVerified=0`. |
| `ADSCHK=TIMING,200`, `ADSCHK=TAIL,200` | 固定参数 200 / fixed count | ACK → ADSTERM | 进入有界诊断，保存/恢复并验证配置。解析器识别前缀 `ADSCHK=TIMING`、`ADSCHK=TAIL`；缺少逗号和样本数会报语法错误。 / The full literal including count is required. Match the accepted request to its terminal. |
| `ADSTIMING?`, `ADSTIMING?RESULT,<id>` | 无或精确 ID / none or exact ID | metadata / result | `ADSTIMING?GROUP,<id>,<0..3>`、`ADSTIMING?TIME,<id>,<0..3>`、`ADSTIMING?REG,<id>,<0..1>`、`ADSTIMING?RECORD,<id>,<index>`。TAIL 的 `ADSTIMING?TAIL,<id>,<index>` 和 `ADSTIMING?IO,<id>,<index>` 使用相同平面索引。 / Query immutable cached pages. Recheck session, last ID and revision after reading multiple pages. |

[详细诊断字段、终态与分页 / Field definitions, terminals and paging](integration/firmware_host_contract.md)

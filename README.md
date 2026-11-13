# SensorArray — ESP32-S3 固件 / ESP32-S3 Firmware

SensorArray 面向 8×8 测量矩阵，支持 **1–8 个活动 S 行、每行 8 个 D 通道**。每行可选择电容 `CAP/C`、电压 `VOLT/V` 或电阻 `RES/R`；`ROWMODES` 可将不同模式组合在同一帧。模式切换的最小单位是一行，不是固定四点。

SensorArray is firmware for an 8×8 measurement matrix with **1–8 active S rows and 8 D channels per row**. Each row can measure capacitance (`CAP/C`), voltage (`VOLT/V`), or resistance (`RES/R`); `ROWMODES` combines these modes in one frame. A row is the smallest mode-switching unit; acquisition is not restricted to a fixed four-point scan.

仓库 / Repository: [ErrorChen/SensorArray](https://github.com/ErrorChen/SensorArray). 上位机兼容改动属于独立的 `ErrorChen/SensorArray_Software` 任务。 / Host compatibility changes belong to the separate `ErrorChen/SensorArray_Software` task.

## 当前失败基线与验收边界 / Current failed baseline and acceptance limits

**修复待验收：以下记录0d8f60f烧录镜像的修复前基线，未宣告任务完成。USB截断与CPU0 IDLE0看门狗问题需修复后以新版本重测；本页保留旧证据供对照。** / **Fixes await acceptance: this records the pre-fix baseline of the flashed 0d8f60f image, not task completion. USB truncation and the CPU0 IDLE0 watchdog issue require fixes and new-version retesting; prior evidence remains for comparison.**

| 项目 / Item | 已核实内容 / Verified information |
| --- | --- |
| 烧录基线源码 / Flashed baseline source | `0d8f60fd9c29b9ce12e64ed2e6534ee37db41704`；本文档起点 HEAD / documentation starting HEAD: `334fb2a081d70731985a6c2d5c2b6f02548ec56e` |
| 已烧录 app / Flashed app | 编译版本 / compiled version `0d8f60f-dirty`，1,552,752 bytes；dirty 为文档/证据，源码输入 hash 独立保存。 / Dirty entries were documentation/evidence; source-input hashes were saved separately. |
| app SHA256 | `232ca3976f6b8ea38efd5e09de8279f35c9bfbbd0c605c8dd84e5169ddd1dbaa` |
| 实际环境 / Tested environment | ESP-IDF **v5.5.5**；ESP32-S3 rev0.2，Flash 16 MB，PSRAM 8 MB；实机 ADS1262，ID `03`。 / Actual device: ESP32-S3 rev0.2, 16 MB flash, 8 MB PSRAM, ADS1262 with ID `03`. |
| 构建与 host 回归 / Build and host regression | 实际IDF构建通过；最终47个host测试入口全部exit 0，另保留早期42入口清单。不是47项硬件验收。 / The actual IDF build passed; all 47 final host test entry points exited 0, with the earlier 42-entry manifest retained. These are not 47 hardware acceptance tests. |
| 当前结论 / Current conclusion | 启动选速、配置身份及有界局部CRC失效已有证据；**双输出32配置仅31项采满120s；VOLT1在15.065s触发CPU0 IDLE0任务看门狗，压力FAIL。运行中仍有ADC SPI CRC拒绝，不能由启动≤2.5%资格推导长期错误率已验收；USB尾截断仍存在，绝对精度未验收。** / Evidence covers startup clock selection, configuration identity, and bounded local CRC rejection. **Only 31 of 32 dual-output configurations completed 120 s; VOLT1 triggered the CPU0 IDLE0 task watchdog after 15.065 s and failed the stress test. Runtime ADC SPI CRC rejections remain; the ≤2.5% startup qualification does not establish an accepted long-term error rate. USB tail truncation remains, and absolute accuracy is not accepted.** |

版本关联见 [构建清单 / build manifest](docs/audit/artifacts/boot_autotune_20261113/final-03-build-manifest.json)、[早期42入口 / earlier 42-test manifest](docs/audit/artifacts/boot_autotune_20261113/final-04-tests-manifest.json)、[最终47入口 / final 47-test manifest](docs/audit/artifacts/full_benchmark_20261113/final-01-tests-manifest.json)、[烧录记录 / flash result](docs/audit/artifacts/boot_autotune_20261113/final-03-flash-result.json) 和 [启动/恢复验收 / startup and recovery report](docs/audit/frontend_startup_recovery_20261113.md)。**`BUILD?` 当前只返回 IDF/target/project/proto，不返回 app commit 或 SHA**；新采集的镜像关联依赖烧录 SHA、持续相同 session/boot 和本地源码输入校验，不能由 BUILD? 单独证明。旧失败实录保留；编译、控制切换或 ASCII CRC 通过不等于测量有效或物理安全。

The linked manifests and report associate source, build, tests, and flash. **`BUILD?` currently returns IDF/target/project/proto, not the app commit or SHA**. New captures are associated with the image through the flash SHA, continuing session/boot, and local source-input checks, not BUILD? alone. Earlier failed captures remain evidence. A successful build, control transition, or ASCII CRC does not establish measurement validity or physical safety.

## 快速开始 / Quick start

### 环境、构建与连接 / Environment, build, and connection

复现当前结果请使用已验证的 **ESP-IDF v5.5.5**、ESP32-S3 工具链和数据 USB 线。早期推荐 v5.5.1；本轮没有把 v5.5.5 的结果外推成 v5.5.1 已复测。Python host/HIL 工具宜使用独立虚拟环境。 / Reproduce the current results with the tested **ESP-IDF v5.5.5**, ESP32-S3 toolchain, and a data-capable USB cable. Earlier documentation recommended v5.5.1; this run does not establish fresh v5.5.1 validation. A separate Python virtual environment is recommended for host/HIL tools.

在已激活 ESP-IDF 的 shell 中，对新检出的仓库执行；已有工作区先保护本地 sdkconfig 和改动。 / Run in an activated ESP-IDF shell for a fresh checkout; preserve local sdkconfig and edits in an existing workspace first.

```sh
git clone https://github.com/ErrorChen/SensorArray.git
cd SensorArray
idf.py set-target esp32s3
idf.py build
idf.py -p PORT flash
idf.py -p PORT monitor
```

`PORT` 替换为实际端口，如 `COMx` 或 `/dev/ttyACM*`。一次只让一个程序读取同一串口，采集前关闭 monitor；多候选设备先确认身份。Native USB Serial/JTAG 的波特率设置不是外部 UART 的吞吐上限。 / Replace `PORT` with the actual device, such as `COMx` or `/dev/ttyACM*`. Use one reader per serial port and close the monitor before capture; identify the device if multiple ports exist. The native USB Serial/JTAG baud setting is not an external UART throughput limit.

### Serial 查询与配置 / Serial queries and configuration

每条 ASCII 命令以 LF 结束。连接或重启后先保存原始查询回复；`CONFIG?` 是完整配置身份的权威查询。 / Terminate each ASCII command with LF. After connection or restart, save raw query replies; `CONFIG?` is authoritative for the complete configuration identity.

```text
PROTO?
BUILD?
CONFIG?
MODE?
STATE?
ROWS?
ROWMODES?
ROUTE?
SPI?
FRONTENDS?
READY?
BOOT?
PERF?
USBSTREAM?
TX?
ST?
BTX?
WIFI?
```

```text
USBSTREAM=DEBUG
MODE=CAP
ROWS=8
CONFIG?
USBSTREAM=FULL
```

setter 应逐条发送，等待匹配终态并确认后再发下一配置请求。`DEBUG/FULL` 控制串口数据/诊断策略，不关闭命令 RX 和关键终态。查询标签不总等于命令名，例如 `TX?` 返回 `ACK,cmd=TX,...`，Host 应按真实契约匹配。 / Send setters one at a time, waiting for the matching terminal result before the next configuration request. `DEBUG/FULL` controls serial data/diagnostic policy without disabling command RX or critical terminals. Reply tags do not always equal command names: `TX?`, for example, returns `ACK,cmd=TX,...`; match the actual contract.

`MODE=VOLT`、`MODE=RES` 与 `ROWMODES=CVRCVRCV` 仅在对应电气条件满足后使用。VOLT 需要合法 rail：在非 VOLT 状态以**真实 DMM 读数**发送 `RAILCFG=<AVDD_UV>,<negative_AVSS_UV>`，等待结果并核对 `ROUTE?/CONFIG?`。这是数值模板，不是通用标称校准。 / Use `MODE=VOLT`, `MODE=RES`, and `ROWMODES=CVRCVRCV` only with their required electrical conditions. VOLT needs valid rails: outside VOLT, supply **actual DMM readings** through `RAILCFG=<AVDD_UV>,<negative_AVSS_UV>`, wait for the result, and check `ROUTE?/CONFIG?`. This is a value template, not a universal nominal calibration.

`RECOVER` 默认 full/level 1，也支持 `RECOVER=0|1|2`；level 2 和 `RESTART` 会真实重启。新 session 需重新查询，RAM RAILCFG 需重新提供；显式重启后的 boot 变化不是未解释的崩溃。 / `RECOVER` defaults to full/level 1 and accepts `RECOVER=0|1|2`; level 2 and `RESTART` perform a real restart. Re-query the new session and reapply RAM RAILCFG. A boot change after an explicit restart is not an unexplained crash.

### BLE 与 Wi-Fi / BLE and Wi-Fi

| 步骤或通道 / Step or channel | 用法 / Usage |
| --- | --- |
| BLE 连接 / Connection | 扫描 `CscArray_*`，连接 Service `0x00FF`；支持时请求 MTU 247。 / Scan for `CscArray_*`, connect to service `0x00FF`, and request MTU 247 if supported. |
| BLE 订阅 / Subscription | `FF11` 控制回复、`FF20` 数据、`FF30` 日志；支持时均开启 Notify+Indicate，CCCD=`0x0003`。 / Subscribe to control replies on FF11, data on FF20, and logs on FF30; enable Notify+Indicate (`0x0003`) when supported. |
| BLE 命令 / Commands | 向 `FF10` 写入 `STATE?\n` 等命令，回复在 FF11。 / Write commands such as `STATE?\n` to FF10; replies arrive on FF11. |
| BLE TX | `BTX=FAST` 用 Notify；`BTX=SAFE` 优先 Indicate，Notify-only 客户端有界回退 Notify。用 `BTX?` 核对；SAFE 不是测量有效性或端到端送达保证。 / FAST uses Notify; SAFE prefers Indicate with bounded Notify fallback for Notify-only clients. Confirm with BTX?; SAFE does not guarantee measurement validity or end-to-end delivery. |
| BLE 大消息 / Large messages | `G,...` envelope 按通道重组，检查总长度、message CRC32 和内层测量帧 CRC。 / Reassemble G envelopes by channel and check total length, message CRC32, and inner measurement-frame CRC. |
| Wi-Fi | 仅 SoftAP+UDP：DATA `3333`、LOG `3334`、CTRL `3335`；STA/APSTA、扫描、凭据保存和自动连接未实现，返回 `sta_nyi`。 / SoftAP+UDP only: DATA 3333, LOG 3334, CTRL 3335. STA/APSTA, scanning, credential persistence, and auto-connect are not implemented and return sta_nyi. |

完整步骤见 [BLE 协议 / BLE protocol](docs/ble-protocol.md)、[transport 协议 / transport protocol](docs/transport-protocol.md) 和 [验证 / validation](docs/validation.md)。 / See these documents for the complete procedures.

## 固件如何运转 / How the firmware operates

### 双核与双 FDC / Dual cores and dual FDCs

```text
Serial / BLE FF10 / Wi-Fi CTRL
              |
 shared command handler -> bounded mailbox
                                  |
                   full-frame boundary / Core 1
                                  |
                 route + configuration owner
                          /              \
                 two FDC workers       ADS ADC1
                          \              /
                           frame builder
                                  |
                      bounded frame/event queues
                                  |
               encoder / log / transport / Core 0
                        /         |         \
                      USB        BLE        UDP
```

Core 1 在完整帧边界应用模式、行数、profile 和辅助事务；命令回调不直接改模拟路由。Core 0 主要负责编码、日志和传输。固定容量队列隔开慢接收端和采集等待，饱和按策略丢弃并计数；不保证没有 CPU/调度开销或无丢帧。大输出池启动时优先分配 PSRAM，采样热路径不反复分配整帧大缓冲。

Core 1 applies mode, row count, profile, and auxiliary operations at full-frame boundaries; command callbacks do not directly change analogue routes. Core 0 primarily handles encoding, logs, and transport. Fixed-capacity queues separate slow receivers from acquisition waits; saturation causes counted drops, not a guarantee of zero CPU/scheduling overhead or zero missing frames. Large output pools prefer PSRAM at startup rather than repeated whole-frame allocations in the sampling hot path.

CAP 每行由独立 I²C 总线上的两颗 FDC2214 并行采四通道，按同一 row/epoch 合并成八通道。持久 slot 保存 job/result/trace，阶段为 `IDLE → POSTED → ACKED → RUNNING → DONE`；取消为 `CANCEL_REQUESTED → CANCEL_ACKED → RECLAIMED`，释放无法确认则 `QUARANTINED`。通知只唤醒，状态和 generation/epoch 决定语义。已接受任务的 ACK/RUN/完成/收尾使用单调 deadline；旧 DONE 不满足新任务，未释放 slot 不复用，不盲删可能持有 I²C 锁的 worker。

For each CAP row, two FDC2214 devices on separate I²C buses acquire four channels concurrently and merge eight channels under the same row/epoch. Persistent slots own job/result/trace storage. Stages are `IDLE → POSTED → ACKED → RUNNING → DONE`; cancellation follows `CANCEL_REQUESTED → CANCEL_ACKED → RECLAIMED`, or `QUARANTINED` if release is unconfirmed. Notifications only wake tasks; state and generation/epoch carry meaning. Accepted jobs use monotonic deadlines for ACK/RUN/completion/cleanup. Old DONE cannot satisfy a new job, unreleased slots are not reused, and workers potentially holding I²C locks are not blindly deleted.

正常路径保留 `INTB → 必要 STATUS → DRDY → read4`，不用常规 STATUS 轮询提前清中断，unread 位也不单独证明新数据。启动创建失败与运行中 quarantine 不同；后者不能用串行回退掩盖仍在运行的旧 worker。 / The normal path retains `INTB → necessary STATUS → DRDY → read4`; routine STATUS polling must not clear interrupts early, and unread bits alone do not prove new data. Startup creation failure differs from runtime quarantine; the latter cannot be hidden by serial fallback while an old worker may still run.

### ADS shadow、cache、PGA 与新鲜度 / ADS shadow, caches, PGA, and freshness

| 层 / Layer | 机制与边界 / Mechanism and boundary |
| --- | --- |
| Driver shadow | 保存已知寄存器/运行状态；物理快照按定义 mask 比较，RESET 单独处理。shadow 不是实测电压。 / Tracks known registers/run state; physical snapshots use defined comparison masks and separate RESET handling. A shadow is not a voltage measurement. |
| Register/profile cache | 避免重复写入，按测点复用已验证 MUX/PGA profile；模式、辅助事务、恢复及状态变化使相关 cache 失效。 / Avoids redundant writes and reuses verified per-cell MUX/PGA profiles; mode, auxiliary operations, recovery, and state changes invalidate affected caches. |
| Value cache / adaptive read | 稳定条件可接纳一笔**新转换**；否则三样本稳定性检查及周期 precision frame。旧值不冒充新采样，未放宽 spread 门限。 / Stable conditions may accept one **new conversion**; otherwise use three-sample stability checks and periodic precision frames. Old values never masquerade as new samples, and spread thresholds are not relaxed. |
| 自动 PGA / Automatic PGA | 检查差分范围、共模、参考与 STATUS；必要时有界进入 bypass，重新读回/建立/取新样本。诊断 gain=0 表示 bypass，不是电压零。 / Checks differential range, common mode, reference, and STATUS; bounded bypass fallback requires new readback, settling, and samples. Diagnostic gain=0 means bypass, not zero volts. |
| DRDY/config generation | 结果需属于当前 MUX/PGA/reference 配置及新 DRDY generation；清通知或 DRDY 已低不证明新鲜。 / Results must belong to the current MUX/PGA/reference configuration and a new DRDY generation; cleared notifications or an already-low DRDY do not prove freshness. |
| 健康回读 / Health readback | 当前每64个 ADS engine帧检查，计数跨模式保留；初次/失效shadow也可能触发。与 host seq、precision/rail/battery 周期分别解释。 / Current checks occur every 64 ADS engine frames and retain their count across modes; initial/invalid shadow may also trigger a check. Keep this distinct from host sequence and precision/rail/battery periods. |

实机 ADS1262 只使用 ADC1，不能借不存在的 ADC2 解决并发。SPI owner、lock、queue/result、DRDY 和有限 retry 消耗相应事务 deadline。超时不等于 DMA 已取消；持久描述符/缓冲在驱动确认回收前不能释放或复用，完成状态未知则隔离并受控升级。 / The actual ADS1262 uses ADC1; a nonexistent ADC2 cannot solve concurrency. SPI ownership, locking, queue/result completion, DRDY, and bounded retries consume the applicable transaction deadline. Timeout does not cancel DMA. Persistent descriptors/buffers cannot be freed or reused before driver-confirmed reclamation; unknown completion requires quarantine and controlled escalation.

### CRC 部分帧与全局故障 / CRC partial frames and global faults

ADC 数据 CRC8 与 ASCII 测量帧 CRC32 各自验证不同层，不能互相替代；ADC STATUS 不受该数据 CRC8 保护。当前每 cell 共用一次 I/O retry，受100 ms cell 总deadline约束，坏样本不更新PGA/有效cache。originalError 与 cleanupError 分开，`ADSFAULT*` 保存清理前上下文。

ADC data CRC8 and ASCII measurement-frame CRC32 protect different layers and cannot replace one another; ADC STATUS is outside the data CRC8 coverage. Each cell currently shares one I/O retry within a 100 ms cell deadline. Bad samples do not update PGA decisions or valid caches. originalError and cleanupError remain separate, and ADSFAULT diagnostics preserve pre-cleanup context.

`0d8f60f` 的 `boundedCrcFailure` 只允许**扫段已完成、软件所检查的路由/owner/寄存器状态仍一致、缺失仅由采样 CRC 拒绝引起**的部分帧输出；该许可不是模拟隔离或供电完整性的实测证明。点仍为 X02，valid/fresh/acquired 不伪造；首个可信转换前失败可为fresh=0，已有新转换后失败可为fresh=1，但均不是有效数值。后续timeout、RESET、配置变化、reference/owner/route错误或DMA未回收会否决许可，不能用首个CRC掩盖后错。

The `boundedCrcFailure` disposition in 0d8f60f permits a partial frame only when **the sweep completed, software-checked route/owner/register state remains coherent, and missing conversions are due solely to sample CRC rejection**. This permission is not measured proof of analogue isolation or supply integrity. Points remain X02 with truthful valid/fresh/acquired bits. Failure before a trusted conversion may have fresh=0, while failure after a new conversion may retain fresh=1; neither is a valid value. A later timeout, RESET, configuration change, reference/owner/route error, or unreclaimed DMA revokes permission. An initial CRC error must not hide a later fault.

连续三次活动点全因CRC拒绝的 **ADS engine扫段**停止继续发布；这不是三个主机 MIXED 帧。CAP/SetMode 不清预算，完成非CRC的ADS测量结论（含合法OPEN）才可清除。`ADSDISPOSITION` 公开 scope=ads_sweep、拒绝位图、真实 acquired/expected、allCrcRun 和 firstError/abortError。 / Three consecutive **ADS engine sweeps** whose active points are all CRC-rejected stop continuation; this is not three host MIXED frames. CAP/SetMode does not clear the budget; a completed non-CRC ADS conclusion, including legitimate OPEN, can clear it. ADSDISPOSITION exposes sweep scope, rejection masks, actual acquired/expected masks, allCrcRun, and firstError/abortError.

### 逐行 C/V/R、参考与辅助事务 / Per-row C/V/R, reference, and auxiliary transactions

| 模式 / Mode | 测量与输出 / Measurement and output | 参考与激励命令 / Reference and excitation commands |
| --- | --- | --- |
| CAP / C | 双FDC，LC总电容，pF；`C/D/K`。 / Dual FDC, total LC capacitance in pF; C/D/K. | INTREF OFF，不主动施加REFOUT。 / INTREF OFF, no intentional REFOUT excitation. |
| VOLT / V | ADS ADC1节点电压，µV；`V/D/P/K`。 / ADS ADC1 node voltage in µV; V/D/P/K. | INTREF OFF，AVDD/AVSS reference，需合法rail模型。 / INTREF OFF, AVDD/AVSS reference, valid rail model required. |
| RES / R | ADS ADC1、参考电阻与REFOUT；线上整数mΩ；`R/D/P/K`。 / ADS ADC1, reference resistor and REFOUT; integer mΩ on the wire; R/D/P/K. | 有意开启INTREF/REFOUT；参考相对AVSS，不是默认对地2.5V。 / Intentional INTREF/REFOUT; reference is relative to AVSS, not automatically 2.5V to ground. |
| ROWMODES / MIXED | 各物理S行有自己的模式/单位；完整8项profile异构时使用`M/MR/K`，即使活动行恰好只有一种模式。只有完整8项同质profile才走单模式分支。 / Each physical S row retains its mode/units. A heterogeneous full eight-entry profile uses M/MR/K even if all active rows happen to share one mode. Only a homogeneous full eight-entry profile uses the single-mode branch. | 帧内按CAP、VOLT、RES分组，保留S行位置和分组时间/偏斜，不是全点同时采样。 / Groups run in CAP, VOLT, RES order, retaining S-row positions and group timing/skew; points are not simultaneous. |

驱动写POWER将已知参考OFF→ON时开始reference-ready计时；此前参考状态未知、或读到原以为关闭的INTREF已开启时，也从观察时刻保守重建等待期限，不伪称测到了物理开启边沿。当前图纸C42=1µF，驱动采用至少50ms要求，不在每点重复等；实装电容/电路修订仍需单独验证。参考、PGA/MUX输入、滤波器建立和新鲜DRDY使用各自的软件等待/检查条件，不能当成示波器已测得稳定。Q1钳位前必须取得INTREF OFF的寄存器证据；GPIO回读、停止转换或软件SAFE均不是模拟隔离实测。

Reference-ready timing starts when a POWER write changes a known reference state from OFF to ON. If the prior reference state was unknown, or a read finds INTREF enabled when it was believed OFF, the driver conservatively rebuilds the waiting deadline from that observation; it does not claim to have measured the physical enabling edge. For the schematic's C42=1µF, the driver applies at least 50ms rather than repeating it at every point; assembled capacitor/circuit revisions still require separate validation. Reference, PGA/MUX input, filter settling, and new DRDY have separate software waits/checks, not oscilloscope-verified stability. Register evidence of INTREF OFF is required before Q1 clamps. GPIO readback, stopped conversion, or software SAFE is not measured analogue isolation.

当前图纸与board map未提供已验证的透明隔离rail monitor能力，报告unavailable；明确请求RES时可复用本来有意开启的参考。MIXED若需内部刷新，在**首点之前**先选择本profile的RES行、进入受控RES、完成事务、恢复原路由并通过参考等待条件；子段禁止自行刷新。无活动RES或真实隔离能力时，不为CAP/VOLT偷开INTREF。

The current schematic and board map provide no verified isolation for transparent rail monitoring and report unavailable. Explicitly requested RES can reuse its intentional reference excitation. When MIXED needs an internal refresh, it selects a RES row from its profile **before the first point**, establishes controlled RES, completes the transaction, restores the previous route, and satisfies the reference-wait condition. Child segments cannot refresh independently. Without active RES or actual isolation capability, CAP/VOLT must not secretly enable INTREF.

rail/battery/ADSCHK 在单一ADS owner下、完整帧边界执行保存→配置→采样→恢复→读回/稳定，再允许新帧。rail专用profile显式PGA enabled/gain1、chop OFF，不继承最后测点bypass；5个合格转换共享一次CRC重试及内部250ms owner deadline。MIXED外层路由进入/恢复另计，不能把250ms称为整个外层事务上限。

Rail/battery/ADSCHK run under one ADS owner at full-frame boundaries: save → configure → sample → restore → readback/settling, then admit a new frame. The dedicated rail profile explicitly enables PGA/gain1 and disables chop rather than inheriting the last cell's bypass. Five accepted conversions share one CRC retry and an internal 250ms owner deadline. MIXED's outer route entry/restoration is additional; 250ms is not a limit for the entire outer transaction.

### 启动并行选速 / Parallel startup qualification

`sensorarrayFrontendStartup` 在Core0运行ADS SPI资格worker，同时Core1执行FDC I²C ID/负载扫描、选频和初始化。**两颗FDC的启动初始化本身仍顺序执行**，与运行期双FDC并行不同。后续route/matrix/gap初始化必须等SPI完成及恢复；SPI?成功需done/joined、未超时、qualified/restored且err=0同时成立。

sensorarrayFrontendStartup runs ADS SPI qualification on Core0 while Core1 performs FDC I²C ID/load checks, clock selection, and initialization. **The two FDC devices are initialized sequentially at startup**, unlike their parallel runtime acquisition. Subsequent route/matrix/gap initialization waits for SPI completion and restoration. SPI? success requires done/joined, no timeout, qualified/restored, and err=0 together.

| 参数 / Parameter | 当前默认 / Current default |
| --- | --- |
| SPI候选 / Candidates | 配置上限以内：2MHz→1MHz每100kHz，950kHz→150kHz每50kHz，最后125kHz。 / Within the configured ceiling: 2MHz→1MHz in 100kHz steps, 950kHz→150kHz in 50kHz steps, then 125kHz. |
| 初筛/确认 / Screen/confirm | 每档400/1600次，各平分PGA gain1与bypass。 / 400/1600 attempts per candidate, each split equally between PGA gain1 and bypass. |
| CRC资格 / CRC qualification | 每profile/阶段各自≤2.5%：200中5次、800中20次，不能用阶段平均隐藏失败profile。 / Each profile/phase independently permits ≤2.5%: 5/200 or 20/800; averaging cannot hide a failing profile. |
| 覆盖/拒绝 / Coverage/rejection | 合格CRC字节末位0/1需足够覆盖；坏CRC不计valid/奇偶/raw范围。寄存器、RESET、STATUS、饱和、事务错误独立拒绝。 / Accepted CRC LSB0/1 need sufficient coverage; rejected CRCs do not count as valid/parity/raw-range samples. Register, RESET, STATUS, saturation, and transaction errors independently reject. |
| 总预算 / Total budget | SPI全候选共享30s单调期限，含调度/初筛/确认/恢复，预留500ms cleanup；原I²C初始化总时长另计。 / All SPI candidates share a 30s monotonic deadline including scheduling/screening/confirmation/restore, reserving 500ms for cleanup; total I²C startup duration is separate. |

测试把AINCOM接两差分输入，INTREF OFF、AVDD/AVSS reference，等偏置建立并要求新DRDY，不是外部精密源校准。选最高已测通过档，不把未测最低档当成功。requestedHz与IDF分频actualHz分开；actualHz不是示波器测量。SPI?累计CRC含淘汰档，选中档应看对应四段SPI_SWEEP。**2.5%仅是有限启动样本资格，不是长期错误率保证，更不允许坏样本变有效值。**

Qualification connects AINCOM to both differential inputs, keeps INTREF OFF with AVDD/AVSS reference, waits for bias settling, and requires new DRDY; it is not external precision calibration. Select the highest tested passing candidate, never an untested fallback. requestedHz differs from IDF-divided actualHz; the latter is not an oscilloscope measurement. SPI? CRC totals include rejected candidates; use the four selected-candidate SPI_SWEEP passes. **2.5% qualifies finite startup samples; it neither guarantees a long-term error rate nor allows corrupt samples to become valid values.**

### 全前端失联与恢复 / Frontend loss and recovery

通信证据是两FDC的Manufacturer/Device ID与ADS稳定ID/核心寄存器。ready=false、CRC错误、busy/unknown或主动SD shutdown不能直接当作三芯片掉电。owner空闲可用时才探测，不读会清FDC中断的STATUS；FRONTENDS?只复制发布快照，不在命令线程访问总线。

Presence evidence uses both FDC Manufacturer/Device IDs and stable ADS ID/core registers. ready=false, CRC errors, busy/unknown, or intentional SD shutdown do not directly prove three-chip power loss. Probes require an available quiescent owner and avoid interrupt-clearing FDC STATUS. FRONTENDS? copies a published snapshot rather than accessing buses from the command thread.

同轮三芯片首次unavailable、已初始化ADS出现RESET或probe cleanup失败即暂停进入SUSPECT，连续三轮全失联进入WAIT。normal/SUSPECT每1s、WAIT每2s探测，每轮600ms预算；ADS临时125kHz后恢复选定档，FDC在SUSPECT/WAIT临时≤100kHz。两轮ID稳定、全present且cleanup成功后，受预算约束地重启ESP并重新扫描，不复用旧worker/DMA/DRDY/cache。

The first same-round all-unavailable observation, RESET on initialized ADS, or failed probe cleanup pauses acquisition in SUSPECT; three consecutive all-absent rounds enter WAIT. Probes run every 1s in normal/SUSPECT and every 2s in WAIT with a 600ms round budget. ADS temporarily uses 125kHz before restoring its selected clock; FDC temporarily uses ≤100kHz in SUSPECT/WAIT. Two rounds of stable IDs, all-present, and successful cleanup request a budgeted ESP restart and new startup scan rather than reuse old worker/DMA/DRDY/cache state.

等待期控制查询/显式RESTART仍可用，mode/rows/辅助测量被拒且有终态。自动重启预算耗尽后可查询等待，不无限复位；真实健康采集进度才可重新授予相应预算。路由为UNKNOWN、physicalSafe=unknown：回电默认状态及识别延迟仍有物理风险。实际三芯片断电/复电不能由host状态机回归替代。

Queries and explicit RESTART remain available while waiting; mode/rows/auxiliary measurements are rejected with terminal results. Exhausted automatic-restart budgets leave a queryable wait state, not a reset loop; actual healthy acquisition progress is required to requalify the relevant budget. The route is UNKNOWN with physicalSafe=unknown because power-return defaults and detection latency remain physical risks. Host state-machine tests do not replace actual three-chip power removal/restoration.

### 原子配置身份与终态 / Atomic configuration identity and terminal results

MODE/ROWS/ROWMODES共用跨transport仲裁，一次接受一个未完成配置事务。MACK/RCMD/RMACK只表示accepted；Core1帧边界应用后才有MAPP/RAPP/RMAPP或拒绝终态。相同mode再次请求仍有新身份，失败ID不覆盖成功applied ID。终态有本地有界保留/优先通道和最后结果缓存，不保证网络exactly-once或PC必收。

MODE/ROWS/ROWMODES share arbitration across transports with one outstanding configuration transaction. MACK/RCMD/RMACK means accepted; MAPP/RAPP/RMAPP or rejection follows application at a Core1 frame boundary. Re-requesting the same mode gets a new identity, and failed IDs do not replace successful applied IDs. Terminals have bounded local retention, priority paths, and last-result caches, not network exactly-once or guaranteed PC receipt.

| 身份 / Identity | 含义 / Meaning |
| --- | --- |
| session/bootId/rev/boundary | 启动会话、旧RTC计数、完整配置revision、允许采样边界；Applied不是首个有效帧已到达。 / Boot session, legacy RTC count, complete configuration revision, and sampling-admission boundary; Applied is not arrival of the first valid frame. |
| mgen/mrid | MODE成功应用域。 / Successfully applied MODE domain. |
| rgen/rrid | ROWS成功应用域。 / Successfully applied ROWS domain. |
| pgen/prid | ROWMODES profile成功应用域。 / Successfully applied ROWMODES profile domain. |
| 旧gen/rid / Legacy gen/rid | C属ROWS，V/R属MODE，M有明确rows/profile字段；不能跨域比大小。 / C uses ROWS, V/R uses MODE, M has explicit rows/profile fields; do not order values across domains. |

采样开始捕获不可变快照，数据/trailer/排队descriptor沿用同一身份，发送不盖最新全局标签。CONFIG?原子复制mode/rows/profile、各域成功ID及最后终态；不能拼接独立STATE?/ROWS?/ROWMODES?为从未存在的配置。终态丢失/晚到或数据先到时，以CONFIG的session/rev/域ID/boundary重建，不关闭门控或给旧帧换标签。

An immutable snapshot is captured at acquisition start and retained by data/trailer/queued descriptor; sending does not stamp the latest global identity. CONFIG? atomically copies mode/rows/profile, successful domain IDs, and last terminals. Do not combine separate STATE?/ROWS?/ROWMODES? replies into a configuration that never existed. Recover from missing/late terminals or early data using CONFIG's session/rev/domain IDs/boundary, not by disabling gating or relabelling old frames.

session是随机64-bit启动nonce，不承诺绝对唯一；跨session不比序号/generation。32-bit配置计数回绕跳零，帧序号模运算可到零；排序限同域同session且距离<2³¹，否则重新查询。 / Session is a random 64-bit boot nonce, not an absolute uniqueness guarantee. Do not compare sequence/generation across sessions. Configuration counters skip zero on 32-bit wrap, while frame sequence wraps through zero. Ordering requires the same domain/session and distance <2³¹; otherwise re-query.

### rail、电池、校准与无效点 / Rails, battery, calibration, and invalid points

RAILCFG外部AVDD/AVSS来自实际测量，RAM保存、不按帧龄过期、重启丢失。内部monitor实测span；比例推算AVDD/AVSS与AINCOM标estimated，不是两个独立实测轨。AIN9接AINCOM，不是GND。来源/时间/不确定度/rail合法性影响换算，不把异常rail clamp成合法值。

External AVDD/AVSS supplied by RAILCFG are measured inputs stored in RAM, without frame-age expiry, and lost on restart. The internal monitor measures span; proportional AVDD/AVSS and AINCOM remain estimated rather than two independently measured rails. AIN9 connects to AINCOM, not GND. Source/time/uncertainty/rail validity affect conversion; abnormal rails are not clamped into valid values.

电池用ADC1、AIN8分压和带偏置AINCOM，需要合法rail与建立。unstable、rail_invalid、SPI/CRC和restore failure分开；无效但恢复verified仅battery unavailable，状态未知/恢复失败则隔离ADS。同类失败有界退避并再探测。LastGood保留实测时间，失败不刷新年龄；未知sentinel按known字段解释，不当真实超长时间。

Battery measurement uses ADC1, the AIN8 divider, and biased AINCOM, requiring valid rails and settling. unstable, rail_invalid, SPI/CRC, and restore failure are distinct. An invalid result with verified restore only makes battery unavailable; unknown state or failed restore isolates ADS. Repeated failures use bounded backoff and re-probing. LastGood retains its real measurement time, failures do not refresh its age, and unknown sentinels are interpreted through known flags, not enormous real ages.

按expected/acquired/fresh/valid/error位图及Xhh解释科学数据。X02为采样SPI/CRC类失败，X12是十六进制0x12（UNSTABLE），不能合计成CRC。OPEN/SHORT、饱和、SETTLING、未采到和有效零各不同；不补零、不暗用旧值、不平滑删除跃迁。CAP是含寄生LC总电容，需对应空载baseline再与裸元件比较。图纸R38–R45的约10kΩ负轨负载若仍在实装中，VOLT加载误差受源阻抗影响，固定offset不能普遍消除；实装修订需单独核对。

Interpret scientific data using expected/acquired/fresh/valid/error masks and Xhh. X02 denotes a sample SPI/CRC-class failure; X12 is hexadecimal 0x12 (UNSTABLE), not another CRC count. OPEN/SHORT, saturation, SETTLING, missing acquisition, and valid zero differ. Do not zero-fill, silently reuse old values, or smooth away transitions. CAP is total LC capacitance including parasitics and requires a corresponding unloaded baseline before bare-component comparison. If the schematic's approximately 10kΩ R38–R45 loads to the negative rail remain assembled, VOLT loading error depends on source impedance and cannot generally be removed by a fixed offset; verify the assembled revision separately.

mode/rows/输出/BTX/RAILCFG易失，但**matrix calibration有显式CAL=SAVE/LOAD持久化**，不能说“所有设置都不写NVS”。独立calib分区校验schema/板标识/硬件版本/payload/CRC32，不接纳半有效记录，只显式SAVE写入而非逐帧写。本轮性能测试不自动归零、保存校准或覆盖科学数据；baseline/offset不能冒充硬件修复。

Mode/rows/output/BTX/RAILCFG are volatile, but **matrix calibration has explicit CAL=SAVE/LOAD persistence**; it is incorrect to say no settings write NVS. The dedicated calib partition validates schema/board identity/hardware revision/payload/CRC32, rejecting partial records and writing only on explicit SAVE, not every frame. This performance run does not automatically zero, save calibration, or overwrite scientific data; baseline/offset is not a hardware repair.

目前校准记录的boardId=`0x53415231`、hardwareRevision=`1`是编译常量，尚无实装版本检测接口；记录匹配仅证明与这些常量一致，不证明当前PCB/BOM已识别或校准仍物理适用。 / Calibration boardId=`0x53415231` and hardwareRevision=`1` are currently compile-time constants without an assembled-revision detection interface. A matching record agrees with those constants; it does not identify the actual PCB/BOM or prove that the calibration remains physically applicable.

## 传输与分层背压 / Transport and layered backpressure

PROTO2保留原域/单位，扩展配置身份/查询、session64、启动SPI资格和frontend presence。先协商PROTO?/BUILD?/CONFIG?。完整DATA/LOG message最大2304 bytes、CTRL reply 512 bytes，旧1536-byte Host缓冲须升级。容量拒绝、驱动timeout、接收缺口与无效点独立计数，不相加成一个“丢包率”。 / PROTO2 retains existing domains/units and adds configuration identity/query, session64, boot SPI qualification, and frontend presence. Negotiate PROTO?/BUILD?/CONFIG? first. Complete DATA/LOG messages are bounded at 2304 bytes and CTRL at 512 bytes; old 1536-byte host buffers need upgrading. Capacity rejection, driver timeout, receive gaps, and invalid points are separate counts, not one combined loss rate.

| 层 / Layer | 可证明的事实与限制 / Evidence and limits |
| --- | --- |
| Capture | 设备完成扫帧及时间，不证明transport提交。 / Device sweep completion/timing, not transport submission. |
| Emitted/frame bus | 编码/本地队列接纳；原identity保留。 / Encoding/local admission; original identity retained. |
| USB | 唯一RX reader/TX owner；stdout/stderr分别组到LF后整行排队。拥塞/超长整行丢弃，lineDropped/lineOversize/discardBytes与partialWrite/timeout分开。 / One RX reader/TX owner; stdout/stderr separately assemble to LF before whole-line queuing. Congested/oversize lines are dropped; lineDropped/lineOversize/discardBytes differ from partialWrite/timeout. |
| USBSTREAM? | sinkQueued是本地提交，driverAcceptedBytes是驱动接收；stdioBound/driverInstalled/SOF不证明PC接收或COM枚举，enumerated=unknown。超过512B明确response_too_long，不截断伪统计。 / sinkQueued is local admission, driverAcceptedBytes is driver acceptance; stdioBound/driverInstalled/SOF do not prove PC receipt or COM enumeration, which remains unknown. Over-512B replies report response_too_long instead of truncated statistics. |
| BLE | 有界队列、分片、独立DATA/LOG/CTRL计数；Notify排队≠PC接收，Indicate确认≠应用处理。B20预览保留原identity，不替完整科学帧。 / Bounded queues, fragments, and separate DATA/LOG/CTRL counts; queued Notify is not PC receipt, nor Indicate acknowledgment application processing. Identity-preserving B20 previews do not replace scientific frames. |
| Wi-Fi UDP | 有界按通道提交，不保证送达/顺序/exactly-once。 / Bounded per-channel submission without guaranteed delivery/order/exactly-once. |
| Host接收 / Host receive | 原字节、完整帧CRC/identity、序号缺口与接收时间；与同窗口设备计数比较。 / Raw bytes, complete-frame CRC/identity, sequence gaps, receive times; compare with device counts from the same window. |

整行/整帧本地入队**不保证线上原子交付**。USB driver分块写入达到deadline后，已经接收的前缀不能撤回，尾部可被丢弃；随后补LF只恢复下一条的边界，不修复前帧。Host必须拒绝并保留截断帧，不能补CRC/identity，也不能把缺少trailer字段直接解释为固件配置域发生了变化。 / Whole-line/frame local queuing **does not guarantee atomic wire delivery**. If chunked USB driver writes reach their deadline, an accepted prefix cannot be withdrawn and the tail may be lost. A following LF restores the next record boundary, not the damaged frame. The host must reject and preserve truncated frames without inventing CRC/identity fields or treating a missing trailer field as direct proof of a firmware configuration change.

控制/生命周期有保留优先路径，DATA有界轮转。无USB主机或慢BLE不能造成无限TX等待，但共享CPU/缓冲/链路仍会产生背压。比较同session同窗口的**增量**；累计drop不等于当前每帧丢失。COM不存在只证明此次打开失败，不证明整机死亡。 / Control/lifecycle has reserved priority paths and DATA uses bounded rotation. An absent USB host or slow BLE must not cause infinite TX waiting, but shared CPU/buffers/link capacity still cause backpressure. Compare **deltas** from the same session/window; accumulated drops do not mean every current frame is lost. A missing COM port proves that open failed, not that the whole device died.

## 基线实测结果 / Baseline test results

### 已完成的0d8f60f窗口 / Completed 0d8f60f runs

下表是实录，不是模拟。FPS按各报告口径，短/不等长窗口不构成严格性能A/B；无MFAULT不等于每点有效。此前FAIL保留，不用新窗口覆盖。 / These are device captures, not simulations. FPS follows each report's definition; short/unequal windows are not controlled performance A/B. No MFAULT does not mean all points valid. Earlier FAIL records are retained, not overwritten by newer windows.

| 实录 / Capture | 时长、完整帧、速率 / Duration, complete frames, rate | 结果与限制 / Result and limit |
| --- | --- | --- |
| boot-03 | SPI requested/actual 900/909kHz；SPI/I²C overlap 804,751µs | screen CRC 2/200、4/200；confirm 10/800、11/800，各≤2.5%；不是精度PASS。 / Each pass meets 2.5%, not accuracy acceptance. |
| volt1-02 | 139.650s；13,042 V帧；capture≈93.57、received≈93.39FPS | MFAULT/协议CRC/identity/结构错误0，缺25帧；661个X02/631帧，8296个X12。 / Zero MFAULT/protocol CRC/identity/structure errors, 25 gaps; 661 X02 points in 631 frames, 8296 X12 points. |
| volt8-02 | 139.890s；2534 V帧；≈18.12FPS | 协议CRC/identity/缺口/MFAULT为0；760个X02/402帧，3199个X12。 / Zero protocol CRC/identity/gaps/MFAULT; 760 X02 points in 402 frames, 3199 X12 points. |
| res8-02 | 139.662s；1391 R帧；≈9.96FPS | 协议CRC/identity/缺口/MFAULT为0；S1D1/S8D8有51/2个X02，有效均值10164.95/10166.69Ω，绝对精度未验收。 / Zero protocol CRC/identity/gaps/MFAULT; S1D1/S8D8 have 51/2 X02 points, valid means 10164.95/10166.69Ω; absolute accuracy not accepted. |
| cap8-02 | 139.849s；1590 C帧；≈11.37FPS | 全点有效；两电容点σ=0.01568/0.01262pF、峰峰=0.16134/0.13653pF，未空载归零/裸33pF校准。 / All points valid; two capacitor points have those σ/peak-to-peak values; no unloaded zero/bare-33pF calibration. |
| mixed8-03 | 139.699s；777 M帧；≈5.56FPS；RCCCCCCR | 协议CRC/identity/缺口/MFAULT为0；S1D1和两电容点全有效，S8D8有3个X02；旧profile不同于新CVRCVRCV。 / Zero protocol CRC/identity/gaps/MFAULT; S1D1 and capacitor points valid, S8D8 has 3 X02; this older profile differs from new CVRCVRCV. |
| switch50-02 | 50次CAP/RES，每次3帧 / 50 CAP/RES changes, 3 frames each | CONTROL_PASS_MEASUREMENT_UNCALIBRATED；身份通过，不替长期稳定性/精度。 / Identity passes, not long-run stability or accuracy acceptance. |

真实VOLT1 seq2693的S1D2=X02，expected=FF而acquired/fresh/valid=FD；下一帧2694为FF，session/revision不变。两帧保留真实encoder字节和线上CRC，见 [实录golden manifest / recorded golden manifest](tests/fixtures/reference_capture/crc_partial_volt1_20261113.manifest.json)。它证明局部失效分类，不证明后续所有CRC都可忽略。 / Actual VOLT1 seq2693 has S1D2=X02 with expected=FF and acquired/fresh/valid=FD; seq2694 returns to FF under the same session/revision. The linked real-encoder/wire-CRC evidence validates local rejection, not permission to ignore all later CRC faults.

VOLT1首末ADSSPI的CRC/retry增量4986/4299含窗口内ADC操作，不能除以帧数当wire错误率。USB查询窗口sinkDropped+55、partialWrite/timeout+0，含准备/清理，不能独自解释25帧缺口。health128/256/384/512有实际err=0且覆盖模式前累计计数。 / VOLT1 endpoint ADSSPI CRC/retry deltas 4986/4299 cover ADC operations, not a wire-error rate obtained by division by frames. The USB query window has sinkDropped+55 and partialWrite/timeout+0, includes preparation/cleanup, and cannot alone explain 25 gaps. Real health128/256/384/512 records report err=0 and cover counts accumulated before mode entry.

完整结果、42入口分类及旧e339a1d的VOLT1/VOLT8 MFAULT FAIL见 [启动/恢复报告 / startup/recovery report](docs/audit/frontend_startup_recovery_20261113.md)。更早版本证据见 [可靠性报告 / reliability report](docs/audit/firmware_reliability_fix_report.md)、[11月13日夹具复测 / fixture recheck](docs/audit/reference_fixture_recheck_20261113.md) 和 [11月4/5日HIL / earlier HIL](docs/audit/reference_fixture_hil_20261104.md)。 / These reports retain full results, host-test categories, earlier failures, and historical evidence without converting them into current PASS.

### 0d8f60f：600秒与32配置失败基线 / 0d8f60f: 600-second run and failed 32-configuration baseline

**USB-only的600s已完成；双输出32配置中31项采满120s，VOLT1因原生任务看门狗报错在15.065s停止，压力测试FAIL。完整窗口不等于协议或精度PASS。** 原始证据保存在 [本轮目录 / current evidence directory](docs/audit/artifacts/full_benchmark_20261113/)。32配置覆盖CAP/VOLT/RES/MIXED各1–8行，**同时接收USB与BLE**；Wi-Fi本轮NOT_RUN。准备、终态等待与收尾不计入采集窗口。本轮没有新固件、烧录、NVS写入或校准改动；收尾查询确认同session/boot7、CAP8与USB DEBUG。

**The USB-only 600 s run completed. Of 32 dual-output configurations, 31 completed 120 s; VOLT1 stopped after 15.065 s with a native task-watchdog report and is a stress-test FAIL. A complete window does not mean protocol or accuracy PASS.** Raw evidence is retained in the linked directory. The sweep covers CAP/VOLT/RES/MIXED at 1–8 rows with **simultaneous USB and BLE reception**; Wi-Fi is NOT_RUN. Preparation, terminal waits, and cleanup are outside the capture window. No new firmware, flash, NVS write, or calibration change occurred. Postflight queries confirmed the same session/boot 7, CAP8, and USB DEBUG.

| 测试 / Test | 目标窗口 / Target window | 完成结果 / Completed result |
| --- | --- | --- |
| [stability-volt1-01](docs/audit/artifacts/full_benchmark_20261113/stability-usb-independent-review.json) | VOLT/1 row，600s，USB-only | 实际窗口600.000s（离线分析600.000063s），55,773完整USB帧；达到目标窗口，但有数据缺口与无效点。 / Actual window 600.000 s (offline analysis: 600.000063 s), 55,773 complete USB frames; required duration met with receive gaps and invalid points. |

| 稳定性指标 / Stability metric | 实际结果及口径 / Observed result and scope |
| --- | --- |
| 身份与范围 / Identity and range | session=`0DCB1029EC3CCA0D`，boot=7，rev=71，seq100309–156162；未跨boot合并。 / Same session, boot 7 and rev 71, sequence 100309–156162; no cross-boot aggregation. |
| 速率 / Rates | device capture **93.09094 FPS**；USB实收 **92.95469 FPS**。前者按序号进展/设备时间差，后者按完整帧数减1/首末K接收时间差。 / Device capture **93.09094 FPS**, USB receive **92.95469 FPS**; sequence progression/device-time span versus (complete frames−1)/first-to-last K receipt span. |
| 链路与运行 / Link and runtime | ASCII CRC=0、identity错误=0、MFAULT=0、reset=0；**缺81帧**，不能称零丢失。 / Zero ASCII CRC/identity errors, MFAULT, or reset; **81 missing frames**, not lossless reception. |
| 测点质量 / Point quality | 共446,184点；有效407,974，无效38,210，其中X02=2,754、X12=35,456；17点fresh/acquired=0真实保留，未补零或改成已采集。 / 446,184 points: 407,974 valid and 38,210 invalid, comprising 2,754 X02 and 35,456 X12; 17 points retain fresh/acquired=0, without zero-filling or false acquisition claims. |
| 内存 / Memory | 起止PERF非尾字段free heap同为8,343,552 bytes；v2完整STK50确认生命周期minimum=8,336,952 bytes，PERF尾字段heapMin保留unknown。不是CPU或最大连续块测量。 / Nonterminal free-heap fields in endpoint PERF queries both report 8,343,552 bytes; v2 complete STK50 confirms a lifetime minimum of 8,336,952 bytes, while PERF’s terminal heapMin remains unknown. This does not measure CPU or the largest contiguous block. |
| 验收边界 / Acceptance limit | 完成600s持续运行观察；ADC CRC仍有拒绝，精度状态NOT_EVALUATED。不能将ASCII CRC=0解释为ADC SPI零错。 / The 600 s runtime observation completed; ADC CRC rejections remain and precision is NOT_EVALUATED. Zero ASCII CRC errors do not mean zero ADC SPI errors. |

该表来自只读离线 [独立逐帧复核 / independent per-frame review](docs/audit/artifacts/full_benchmark_20261113/stability-usb-independent-review.json)，只统计header与K接收均位于明确采集窗口内的完整帧；准备/收尾和跨边界帧不混入。最终分析复核同样的55,773可信帧，18,058帧全点valid/fresh/acquired。 / This table comes from read-only offline analysis and counts complete frames whose header and K receipts both lie inside the explicit capture window. Preparation, cleanup, and boundary-straddling frames are excluded. Final analysis confirms the same 55,773 trusted frames; 18,058 have every point valid, fresh, and acquired.

<!-- BEGIN BENCHMARK PERFORMANCE -->
#### 32配置吞吐与完整性 / Throughput and integrity for all 32 configurations

主数据源为 [基线分析v2 / baseline analysis v2](docs/audit/artifacts/full_benchmark_20261113/benchmark-final-v2.json) 和 [窗口选择v2 / window selection v2](docs/audit/artifacts/full_benchmark_20261113/benchmark-selection-v2.json)。[v1原报告 / original v1 report](docs/audit/artifacts/full_benchmark_20261113/benchmark-final.json)保留；v2更正截断telemetry的采纳与BL50窗口名称，可信帧/FPS/测点质量和选择不变。 / The primary sources are baseline analysis v2 and its selection manifest. The original v1 report remains; v2 corrects acceptance of truncated telemetry and the BL50 window label without changing trusted frames, FPS, point quality, or selection.

每个有数值的主表行都完成120s；VOLT1未完成，短窗数值单列在失败说明中。默认选择`suite-01-<mode><rows>`；VOLT2选择`retry-01-volt2`。选择规则保留已采满的首次窗口（包括错误），只对未完成窗口使用最早的完整补测，**不按FPS或错误率挑选**。

Every populated comparison row completed 120 s. VOLT1 did not; its short-window evidence is reported separately below. The default selection is `suite-01-<mode><rows>`, with `retry-01-volt2` used for VOLT2. Selection retains the first completed window, including errors; only an incomplete window can use the earliest complete retry. **Selection never favors FPS or error rate.**

`U/B`表示USB/BLE，N为同时通过ASCII CRC、完整header/K身份及请求帧契约的`trusted_complete_frames`，不是仅解析出K的帧数。Capture按可信序号进展/设备时间差估算；各端FPS=(N−1)/该端首末可信K接收时间差。Missing是同身份组、该端首末可信帧之间缺少的序号数，可能包含被拒收帧，不能再与identity错误相加当独立丢失。

`U/B` means USB/BLE. N is `trusted_complete_frames`: ASCII CRC, complete header/K identity, and requested wire contract all pass; a parsed K alone is insufficient. Capture is estimated from trusted sequence progression over device time. Each receiver FPS is (N−1) divided by its first-to-last trusted K receipt span. Missing counts absent sequence observations within that receiver’s trusted endpoints and identity group; rejected frames can contribute, so adding identity errors would double-count losses.

| 配置 / Config | Capture FPS | USB FPS | BLE FPS | 可信N / Trusted N U/B | Missing U/B | ASCII CRC错误 / errors U/B | Identity错误 / errors U/B |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| CAP1 | 64.3864 | 62.5658 | 64.3941 | 7,508 / 7,727 | 219 / 0 | 0 / 0 | 0 / 0 |
| CAP2 | 36.7538 | 36.7561 | 36.7563 | 4,411 / 4,410 | 0 / 0 | 0 / 0 | 0 / 0 |
| CAP3 | 26.4698 | 26.4709 | 26.4739 | 3,176 / 3,177 | 0 / 0 | 0 / 0 | 0 / 0 |
| CAP4 | 20.6786 | 20.6814 | 20.6828 | 2,482 / 2,482 | 0 / 0 | 0 / 0 | 0 / 0 |
| CAP5 | 16.5503 | 16.5514 | 16.5521 | 1,986 / 1,986 | 0 / 0 | 0 / 0 | 0 / 0 |
| CAP6 | 14.0734 | 14.0739 | 14.0739 | 1,689 / 1,688 | 0 / 0 | 0 / 0 | 0 / 0 |
| CAP7 | 12.2392 | 12.2397 | 12.2396 | 1,469 / 1,469 | 0 / 0 | 0 / 0 | 0 / 0 |
| CAP8 | 10.7733 | 10.7736 | 10.7733 | 1,293 / 1,292 | 0 / 0 | 0 / 0 | 0 / 0 |
| VOLT1 **FAIL** | — | — | — | — | — | — | — |
| VOLT2 | 41.2456 | 33.1760 | 41.2519 | 3,981 / 4,949 | 969 / 0 | 0 / 0 | 10 / 0 |
| VOLT3 | 32.4983 | 31.3703 | 32.5026 | 3,764 / 3,899 | 136 / 0 | 0 / 0 | 2 / 0 |
| VOLT4 | 25.6297 | 25.3434 | 25.6261 | 3,041 / 3,075 | 34 / 0 | 0 / 0 | 0 / 0 |
| VOLT5 | 21.8982 | 21.7982 | 21.8993 | 2,616 / 2,628 | 12 / 0 | 0 / 0 | 0 / 0 |
| VOLT6 | 18.4290 | 18.3404 | 18.4396 | 2,200 / 2,212 | 12 / 0 | 0 / 0 | 0 / 0 |
| VOLT7 | 16.3499 | 16.3007 | 16.3505 | 1,956 / 1,962 | 6 / 0 | 0 / 0 | 0 / 0 |
| VOLT8 | 14.4524 | 14.4506 | 14.4506 | 1,734 / 1,734 | 0 / 0 | 0 / 0 | 0 / 0 |
| RES1 | 48.1181 | 45.2079 | 48.1206 | 5,425 / 5,774 | 349 / 0 | 0 / 0 | 9 / 0 |
| RES2 | 29.3127 | 29.2754 | 29.3169 | 3,513 / 3,518 | 5 / 0 | 0 / 0 | 0 / 0 |
| RES3 | 21.6303 | 21.6264 | 21.6369 | 2,595 / 2,596 | 1 / 0 | 0 / 0 | 0 / 0 |
| RES4 | 17.1145 | 17.1135 | 17.1133 | 2,053 / 2,053 | 0 / 0 | 0 / 0 | 0 / 0 |
| RES5 | 14.0631 | 14.0659 | 14.0657 | 1,688 / 1,688 | 0 / 0 | 0 / 0 | 0 / 0 |
| RES6 | 11.6792 | 11.6871 | 11.6871 | 1,401 / 1,401 | 0 / 0 | 0 / 0 | 0 / 0 |
| RES7 | 10.0999 | 10.1016 | 10.1014 | 1,212 / 1,212 | 0 / 0 | 0 / 0 | 0 / 0 |
| RES8 | 8.8915 | 8.8987 | 8.8988 | 1,067 / 1,067 | 0 / 0 | 0 / 0 | 0 / 0 |
| MIXED1 | 38.6716 | 38.6754 | 38.6751 | 4,641 / 4,640 | 0 / 0 | 0 / 0 | 0 / 0 |
| MIXED2 | 19.7432 | 19.7450 | 19.7447 | 2,369 / 2,369 | 0 / 0 | 0 / 0 | 0 / 0 |
| MIXED3 | 7.5966 | 7.5977 | 7.5977 | 911 / 911 | 0 / 0 | 0 / 0 | 0 / 0 |
| MIXED4 | 7.0076 | 7.0087 | 7.0090 | 841 / 841 | 0 / 0 | 0 / 0 | 0 / 0 |
| MIXED5 | 6.1781 | 6.1790 | 6.1792 | 742 / 742 | 0 / 0 | 0 / 0 | 0 / 0 |
| MIXED6 | 5.4554 | 5.4558 | 5.4557 | 655 / 655 | 0 / 0 | 0 / 0 | 0 / 0 |
| MIXED7 | 5.1548 | 5.1552 | 5.1553 | 618 / 618 | 0 / 0 | 0 / 0 | 0 / 0 |
| MIXED8 | 4.7021 | 4.7025 | 4.7029 | 564 / 564 | 0 / 0 | 0 / 0 | 0 / 0 |

31个完整窗口共接收USB可信73,601帧、BLE可信75,339帧；73,594个共同identity/seq的原始科学帧SHA256全部相同。两端边界可能相差一帧；“另一端无可信匹配”也可能是截断或被拒收，不等于原字节完全没到达。所选31窗的BLE DATA/LOG重组缺片、冲突/乱序、重复、message CRC失败均为0，序号missing为0；这**不包括VOLT1压力失败窗**，不支持整体无背压的结论。

Across the 31 complete windows, USB received 73,601 trusted frames and BLE 75,339. All 73,594 common identity/sequence scientific frames have identical original-byte SHA256 hashes. Receiver boundaries can differ by one frame; a missing trusted match can also mean truncation or rejection, not absence of all bytes. In these 31 selected windows, BLE DATA/LOG reassembly reports zero missing/conflicting/out-of-order/duplicate fragments or message CRC failures, and zero sequence-missing frames. This **excludes the failed VOLT1 stress window** and does not establish absence of backpressure overall.

![修复前基线：各活动行数的Capture、USB与BLE FPS / Pre-fix baseline: capture, USB and BLE FPS by active row count](docs/audit/artifacts/full_benchmark_20261113/firmware-benchmark-fps-v2.png)

图中横轴为活动行数，纵轴为FPS：蓝色圆点=设备Capture，橙色方块=USB实收，绿色三角=BLE实收。VOLT1留空代表压力FAIL/未完成120s，不是0FPS；带协议错误但采满的窗口仍绘出，实收FPS只用可信帧。完整矢量图见 [SVG](docs/audit/artifacts/full_benchmark_20261113/firmware-benchmark-fps-v2.svg)，数据/telemetry独立复核见 [基线复核v2 / baseline review v2](docs/audit/artifacts/full_benchmark_20261113/final-review-v2.json)。

The horizontal axis is active rows and the vertical axis is FPS: blue circles are device capture, orange squares USB receive, and green triangles BLE receive. Blank VOLT1 means stress FAIL/incomplete 120 s, not zero FPS. Completed windows with protocol errors remain plotted, using only trusted frames for receive FPS. The linked SVG and independent baseline review retain the underlying scope and checks.

#### 逐点质量 / Point quality

以下只统计各端可信帧中的原始测点状态。`X02`为SPI错误类别（本轮包括CRC拒绝），`X12`为UNSTABLE稳定性检查失败，`X0D/OPEN`为无有效有限电阻的状态；OPEN保留为无效数值，不能填零，也不等同ADC通信失败。有效+无效=活动点总数。最后一列分别统计fresh=0和acquired=0，本批两者计数相同；这些点未被包装为已采样。绝对精度未验收。

These totals use original point statuses only from each receiver’s trusted frames. `X02` is the SPI-error category (including CRC rejection here), `X12` a failed UNSTABLE stability check, and `X0D/OPEN` a state without a valid finite resistance. OPEN remains an invalid numeric value, is never zero-filled, and is not equivalent to ADC communication failure. Valid plus invalid equals all active points. The final column counts fresh=0 and acquired=0 separately; their totals happen to match in these captures. Such points remain unacquired. Absolute accuracy is not accepted.

| 配置 / Config | USB 有效/无效 / valid/invalid | BLE 有效/无效 / valid/invalid | X02 U/B | X12 U/B | X0D OPEN U/B | fresh=0、acquired=0 各自 / each U/B |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| CAP1 | 60,064 / 0 | 61,816 / 0 | 0 / 0 | 0 / 0 | 0 / 0 | 0 / 0 |
| CAP2 | 70,576 / 0 | 70,560 / 0 | 0 / 0 | 0 / 0 | 0 / 0 | 0 / 0 |
| CAP3 | 76,224 / 0 | 76,248 / 0 | 0 / 0 | 0 / 0 | 0 / 0 | 0 / 0 |
| CAP4 | 79,424 / 0 | 79,424 / 0 | 0 / 0 | 0 / 0 | 0 / 0 | 0 / 0 |
| CAP5 | 79,440 / 0 | 79,440 / 0 | 0 / 0 | 0 / 0 | 0 / 0 | 0 / 0 |
| CAP6 | 81,072 / 0 | 81,024 / 0 | 0 / 0 | 0 / 0 | 0 / 0 | 0 / 0 |
| CAP7 | 82,264 / 0 | 82,264 / 0 | 0 / 0 | 0 / 0 | 0 / 0 | 0 / 0 |
| CAP8 | 82,752 / 0 | 82,688 / 0 | 0 / 0 | 0 / 0 | 0 / 0 | 0 / 0 |
| VOLT1 **FAIL** | — | — | — | — | — | — |
| VOLT2 | 60,894 / 2,802 | 75,684 / 3,500 | 476 / 602 | 2,326 / 2,898 | 0 / 0 | 115 / 166 |
| VOLT3 | 87,570 / 2,766 | 90,695 / 2,881 | 580 / 616 | 2,186 / 2,265 | 0 / 0 | 140 / 163 |
| VOLT4 | 94,943 / 2,369 | 95,996 / 2,404 | 606 / 619 | 1,763 / 1,785 | 0 / 0 | 121 / 132 |
| VOLT5 | 102,472 / 2,168 | 102,944 / 2,176 | 646 / 648 | 1,522 / 1,528 | 0 / 0 | 123 / 125 |
| VOLT6 | 103,698 / 1,902 | 104,252 / 1,924 | 625 / 638 | 1,277 / 1,286 | 0 / 0 | 107 / 117 |
| VOLT7 | 107,772 / 1,764 | 108,103 / 1,769 | 674 / 676 | 1,090 / 1,093 | 0 / 0 | 90 / 92 |
| VOLT8 | 108,069 / 2,907 | 108,069 / 2,907 | 758 / 758 | 2,149 / 2,149 | 0 / 0 | 77 / 77 |
| RES1 | 5,366 / 38,034 | 5,704 / 40,488 | 59 / 70 | 0 / 0 | 37,975 / 40,418 | 3 / 3 |
| RES2 | 3,464 / 52,744 | 3,468 / 52,820 | 49 / 50 | 0 / 0 | 52,695 / 52,770 | 1 / 1 |
| RES3 | 2,558 / 59,722 | 2,559 / 59,745 | 37 / 37 | 0 / 0 | 59,685 / 59,708 | 2 / 2 |
| RES4 | 2,013 / 63,683 | 2,013 / 63,683 | 40 / 40 | 0 / 0 | 63,643 / 63,643 | 0 / 0 |
| RES5 | 1,651 / 65,869 | 1,651 / 65,869 | 37 / 37 | 0 / 0 | 65,832 / 65,832 | 1 / 1 |
| RES6 | 1,361 / 65,887 | 1,361 / 65,887 | 40 / 40 | 0 / 0 | 65,847 / 65,847 | 0 / 0 |
| RES7 | 1,181 / 66,691 | 1,181 / 66,691 | 31 / 31 | 0 / 0 | 66,660 / 66,660 | 0 / 0 |
| RES8 | 2,081 / 66,207 | 2,081 / 66,207 | 53 / 53 | 0 / 0 | 66,154 / 66,154 | 0 / 0 |
| MIXED1 | 37,128 / 0 | 37,120 / 0 | 0 / 0 | 0 / 0 | 0 / 0 | 0 / 0 |
| MIXED2 | 37,506 / 398 | 37,506 / 398 | 398 / 398 | 0 / 0 | 0 / 0 | 2 / 2 |
| MIXED3 | 14,459 / 7,405 | 14,459 / 7,405 | 117 / 117 | 0 / 0 | 7,288 / 7,288 | 2 / 2 |
| MIXED4 | 20,048 / 6,864 | 20,048 / 6,864 | 136 / 136 | 0 / 0 | 6,728 / 6,728 | 0 / 0 |
| MIXED5 | 23,540 / 6,140 | 23,540 / 6,140 | 204 / 204 | 0 / 0 | 5,936 / 5,936 | 0 / 0 |
| MIXED6 | 20,758 / 10,682 | 20,758 / 10,682 | 202 / 202 | 0 / 0 | 10,480 / 10,480 | 0 / 0 |
| MIXED7 | 24,552 / 10,056 | 24,552 / 10,056 | 168 / 168 | 0 / 0 | 9,888 / 9,888 | 0 / 0 |
| MIXED8 | 26,820 / 9,276 | 26,820 / 9,276 | 252 / 252 | 0 / 0 | 9,024 / 9,024 | 0 / 0 |

<!-- END BENCHMARK PERFORMANCE -->

完整请求profile=CVRCVRCV，只启用前N行；M header非活动行用N，CONFIG保留完整profile。**1行只有CAP活动测点，但仍是MIXED调度和M/MR/K帧，不等于MODE=CAP、ROWS=1的调用路径。** 源码用完整8项判断同质性：`sensorarrayRunOneFrame`仍调用MixedRow引擎，每帧对活动CAP组执行ApplyMode/路由回读，再由同一个FDC引擎采样并合并；V/R组为空而跳过，rail预备因无活动ADS行直接返回。此为调用路径事实，不能单凭它量化或归因USB实收FPS差异。

The full requested profile is CVRCVRCV with the first N rows active. Inactive M-header rows use N, while CONFIG retains the full profile. **One row has CAP-only active points but still uses MIXED scheduling and M/MR/K frames; it is not the same call path as MODE=CAP, ROWS=1.** Homogeneity is tested across all eight entries. sensorarrayRunOneFrame still calls the MixedRow engine, which applies/verifies the active CAP route on every frame before using the same FDC engine and merging its result. Empty V/R groups are skipped, and rail preparation returns immediately because no ADS row is active. These are call-path facts, not a measured attribution or quantification of USB receive-FPS differences.

本轮旧collector曾按“1行应为C帧”的错误假设对MIXED1报ERROR；该假设与真实encoder不符。原始ERROR保留；独立analyzer已按真实M/MR/K、完整profile身份和活动行复核通过：USB 4,641可信帧、BLE 4,640可信帧，wire contract/identity错误均0，不修改原始帧来满足工具假设。 / The earlier collector reported ERROR for this round's MIXED1 by incorrectly expecting a C frame at one row. That assumption conflicts with the real encoder. The original ERROR remains; independent validation against actual M/MR/K, full-profile identity, and active rows passed: 4,641 trusted USB and 4,640 trusted BLE frames with zero wire-contract or identity errors. No raw frame was changed to satisfy the tool's assumption.

新profile的S1=C、S4=C、S5=V、S8=V，与旧`RCCCCCCR`夹具测试不同：S1D1/S8D8上的10kΩ件不再由这两行执行RES测量，S5D5的33pF件也不由该行执行CAP测量。必须按实际行模式解释读数；本轮MIXED不能据此宣称两只10kΩ和两只33pF均通过精度验收。 / In the new profile S1=C, S4=C, S5=V and S8=V, unlike the earlier RCCCCCCR fixture test. The 10 kΩ components at S1D1/S8D8 are no longer measured as RES on those rows, and the 33 pF component at S5D5 is not measured as CAP there. Interpret readings according to the actual row mode; this MIXED run cannot establish accuracy acceptance for both 10 kΩ and both 33 pF components.

USB与BLE同时运行会改变CPU/队列负载及可能的capture FPS；上表是**双输出条件下的链路对照**，不能与USB-only稳定性直接作独立接口性能A/B。BLE后端报告MTU=23，而固件BL50报告mtu=247；两种来源均保留，未抓取ATT协商波形，不按23臆算链路上限。 / Running USB and BLE together changes CPU/queue load and may affect capture FPS. These tables compare **links under dual-output load**, not independent interface performance against the USB-only stability run. The BLE backend reported MTU 23 while firmware BL50 reported mtu 247. Both observations retain their provenance; ATT negotiation was not captured, and a link ceiling is not inferred from 23.

<!-- BEGIN BENCHMARK TELEMETRY -->
#### 提交、背压与ADC计数窗口 / Submission, backpressure, and ADC counter windows

本表是各计数器自己的首末接收时间窗口，不是统一120s边界。USB与PERF来自主动查询；BL50和ADSHEALTH来自周期诊断日志。USB-only行的BL50处于conn=0，不是BLE实收测试。`s; Δ...`中的s为对应窗口秒数。USB queued包括数据、诊断和回复，driver bytes仅表示驱动接收，不是PC确认；sinkDropped已含lineDropped，不能重复相加。PERF.pub是入队前的发布调用，**不是成功emitted或ADC转换总数**；emitQueued没有固件字段，标NOT_REPORTED。31个完整窗口的PERF.dropOut/dropEvent/queueDrop增量均0，不代表USB/BLE无丢失。

Each counter uses its own first/last receipt window, not a common exact 120 s boundary. USB and PERF come from active queries; BL50 and ADSHEALTH from periodic diagnostics. BL50 in the USB-only row has conn=0 and is not a BLE receive test. In `s; Δ...`, s is that counter window in seconds. USB queued includes data, diagnostics, and replies; driver bytes mean driver acceptance, not PC acknowledgment. sinkDropped already includes lineDropped, so do not add them again. PERF.pub counts publish calls before admission, **not successful emission or ADC conversion count**. Firmware does not report emitQueued, which remains NOT_REPORTED. PERF.dropOut/dropEvent/queueDrop deltas are zero in the 31 complete windows; this does not mean zero USB/BLE loss.

| 配置 / Config | USB query s | ΔUSB queued / driver bytes | ΔUSB partial / timeout / sinkDropped | BL50 s; ΔdropD/L/C | ADSHEALTH s; ΔCRC/retry | PERF s; Δpub |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| stability USB-only | 600.027 | 76,538 / 42,812,206 | 0 / 0 / 219 | 599.740; 0 / 0 / 0 | 599.465; 20,775 / 17,927 | 600.028; 55,856 |
| CAP1 | 120.040 | 9,400 / 3,674,551 | 0 / 0 / 670 | 119.813; 0 / 0 / 0 | — | 120.040; 7,729 |
| CAP2 | 120.019 | 5,768 / 2,519,362 | 0 / 0 / 191 | 119.713; 0 / 0 / 0 | — | 120.030; 4,412 |
| CAP3 | 120.081 | 4,306 / 2,081,584 | 0 / 0 / 128 | 119.571; 0 / 0 / 0 | — | 120.080; 3,179 |
| CAP4 | 120.017 | 3,465 / 1,824,022 | 0 / 0 / 98 | 119.203; 0 / 0 / 0 | — | 120.016; 2,482 |
| CAP5 | 120.017 | 2,873 / 1,627,279 | 0 / 0 / 80 | 118.723; 0 / 0 / 0 | — | 120.018; 1,986 |
| CAP6 | 120.026 | 2,517 / 1,520,198 | 0 / 0 / 68 | 118.651; 0 / 0 / 0 | — | 120.043; 1,689 |
| CAP7 | 120.018 | 2,254 / 1,444,358 | 0 / 0 / 60 | 118.463; 0 / 0 / 0 | — | 120.017; 1,469 |
| CAP8 | 120.016 | 2,040 / 1,384,195 | 0 / 0 / 52 | 117.406; 0 / 0 / 0 | — | 120.016; 1,293 |
| VOLT1 **FAIL** | — | — | — | — | — | — |
| VOLT2 | 120.094 | 7,107 / 3,775,717 | 479 / 676 / 1,835 | 119.106; 0 / 0 / 0 | 119.437; 6,361 / 5,762 | 120.073; 4,953 |
| VOLT3 | 120.045 | 6,872 / 3,948,837 | 177 / 197 / 333 | 119.524; 0 / 0 / 0 | 118.232; 6,613 / 6,010 | 120.045; 3,901 |
| VOLT4 | 120.032 | 5,919 / 3,533,784 | 46 / 47 / 95 | 119.600; 0 / 0 / 0 | 117.392; 6,955 / 6,340 | 120.054; 3,076 |
| VOLT5 | 120.018 | 5,388 / 3,346,535 | 14 / 14 / 24 | 119.396; 0 / 0 / 0 | 116.846; 7,250 / 6,620 | 120.018; 2,628 |
| VOLT6 | 120.049 | 4,754 / 3,048,833 | 11 / 11 / 40 | 119.144; 0 / 0 / 0 | 117.784; 7,624 / 7,003 | 120.046; 2,213 |
| VOLT7 | 120.036 | 4,333 / 2,921,580 | 5 / 6 / 12 | 119.307; 0 / 0 / 0 | 117.263; 7,850 / 7,188 | 120.036; 1,963 |
| VOLT8 | 120.046 | 4,149 / 2,796,755 | 0 / 0 / 0 | 117.580; 0 / 0 / 0 | 115.220; 8,539 / 7,804 | 120.046; 1,735 |
| RES1 | 120.064 | 7,056 / 4,089,909 | 63 / 77 / 457 | 119.817; 0 / 0 / 0 | 117.102; 351 / 280 | 120.064; 5,777 |
| RES2 | 120.036 | 4,765 / 2,853,453 | 11 / 11 / 19 | 119.363; 0 / 0 / 0 | 117.910; 223 / 173 | 120.036; 3,519 |
| RES3 | 120.030 | 3,687 / 2,284,843 | 1 / 1 / 1 | 119.465; 0 / 0 / 0 | 118.302; 184 / 146 | 120.030; 2,597 |
| RES4 | 120.039 | 3,047 / 1,931,140 | 0 / 0 / 0 | 118.600; 0 / 0 / 0 | 115.936; 155 / 116 | 120.039; 2,054 |
| RES5 | 120.041 | 2,633 / 1,705,308 | 0 / 0 / 0 | 118.762; 0 / 0 / 0 | 113.750; 160 / 121 | 120.038; 1,689 |
| RES6 | 120.021 | 2,318 / 1,507,686 | 0 / 0 / 0 | 118.590; 0 / 0 / 0 | 114.986; 167 / 128 | 120.021; 1,401 |
| RES7 | 120.021 | 2,080 / 1,385,724 | 0 / 0 / 0 | 118.777; 0 / 0 / 0 | 113.932; 149 / 119 | 120.021; 1,212 |
| RES8 | 120.024 | 1,999 / 1,308,311 | 0 / 0 / 0 | 118.099; 0 / 0 / 0 | 107.976; 224 / 174 | 120.024; 1,068 |
| MIXED1 | 120.050 | 10,700 / 3,375,803 | 8 / 10 / 199 | 119.326; 0 / 0 / 0 | — | 120.050; 4,643 |
| MIXED2 | 120.028 | 11,957 / 3,258,421 | 0 / 0 / 96 | 119.019; 0 / 0 / 0 | 119.927; 3,257 / 2,859 | 120.028; 2,370 |
| MIXED3 | 120.036 | 6,685 / 1,747,520 | 0 / 0 / 36 | 118.452; 0 / 0 / 0 | 119.907; 1,117 / 1,000 | 120.036; 912 |
| MIXED4 | 120.020 | 6,320 / 1,776,764 | 0 / 0 / 32 | 117.003; 0 / 0 / 0 | 119.974; 1,240 / 1,104 | 120.020; 841 |
| MIXED5 | 120.023 | 5,860 / 1,737,892 | 0 / 0 / 30 | 118.130; 0 / 0 / 0 | 119.925; 2,149 / 1,946 | 120.023; 742 |
| MIXED6 | 120.024 | 5,293 / 1,638,136 | 0 / 0 / 26 | 116.407; 0 / 0 / 0 | 119.873; 1,857 / 1,655 | 120.024; 655 |
| MIXED7 | 120.033 | 4,995 / 1,647,480 | 0 / 0 / 26 | 116.394; 0 / 0 / 0 | 119.920; 1,697 / 1,529 | 120.033; 618 |
| MIXED8 | 120.020 | 4,851 / 1,644,643 | 0 / 0 / 24 | 116.968; 0 / 0 / 0 | 119.939; 2,411 / 2,159 | 120.020; 564 |

ADC CRC/retry是驱动操作计数，包含矩阵及其间辅助操作，不能除以可信帧数当SPI误码率；health未观察到的CAP/MIXED1条目为—，不是零错误证明。`crcValidEven/Odd`统计通过校验的CRC字节末位覆盖，既非电压符号也非所有SPI字节的错误率。详细first/last字段、原偏移、其它计数与窗口保存在主JSON中。

ADC CRC/retry counters count driver operations, including matrix and intervening auxiliary operations; dividing by trusted frame count does not produce an SPI bit/error rate. Unobserved CAP/MIXED1 health counters are —, not proof of zero errors. crcValidEven/Odd counts accepted CRC-byte LSB coverage, not voltage sign or an error rate over all SPI bytes. The main JSON retains exact first/last fields, raw offsets, additional counters, and windows.

#### 内存统计与分析器更正 / Memory statistics and analyzer correction

v1分析曾把截断STK50尾部的`8336`当作heap minimum。v2按完整字段结构拒绝该行，不按“数值太小”过滤；合法小值仍可保留。PERF最后的heapMin没有后继标记/CRC，v2保留原token作诊断并将权威值标unknown。以下heap最小值只来自完整STK50；完整结构检查仍不等于带CRC的遥测。

The v1 analysis interpreted a truncated STK50 suffix `8336` as a heap minimum. V2 rejects the incomplete field layout, rather than filtering values because they are small; legitimate small values remain admissible. PERF’s terminal heapMin has no following marker or CRC, so v2 retains the original token diagnostically and marks the authoritative value unknown. Heap minima below use only complete STK50 records. Structural completeness still is not checksummed telemetry.

| 指标 / Metric | 实测与限制 / Observation and limit |
| --- | --- |
| 600s USB-only heap | 完整STK50采集窗最小free=8,343,552 B，生命周期minimum=8,336,952 B；起止PERF的非尾字段free相符。 / Complete capture-window STK50 reports minimum observed free heap 8,343,552 B and lifetime minimum 8,336,952 B; the nonterminal free-heap fields in endpoint PERF queries agree. |
| 31完整窗口 heap / 31 complete windows | 每窗最小observed free范围8,340,140–8,341,148 B；生命周期minimum最终8,336,464 B。采集窗接受4,284条完整STK50、拒绝189条截断布局。 / Per-window minimum observed free heap ranges from 8,340,140 to 8,341,148 B; the final lifetime minimum is 8,336,464 B. Capture windows contain 4,284 accepted complete STK50 records and 189 rejected incomplete layouts. |
| 栈余量 / Remaining stack | 完整STK50观测到的生命周期最低余量：log12,792、transport3,204、bleTx1,948、bleCtrl5,236、serialCtrl1,808 B；USB独立查询txStackMinBytes=3,092 B。不是每个任务/瞬时CPU采样。 / Observed lifetime low-water remaining bytes from complete STK50: log12,792, transport3,204, bleTx1,948, bleCtrl5,236, serialCtrl1,808; separate USB queries report txStackMinBytes=3,092 B. These do not measure every task or instantaneous CPU load. |
| 未测 / Not measured | CPU利用率、最大连续heap块、完整raw ADC样本与示波器模拟节点仍NOT_RUN/不可由现有帧重建。没有据此宣称无内存泄漏或物理安全。 / CPU utilization, largest contiguous heap block, complete raw ADC samples, and oscilloscope analogue nodes remain NOT_RUN or unreconstructable from these frames. These results do not establish absence of memory leaks or physical safety. |

<!-- END BENCHMARK TELEMETRY -->

#### 失败、重测与工具边界 / Failures, retries, and harness limits

| 窗口 / Window | 结果与证据含义 / Result and meaning |
| --- | --- |
| `preflight-01` | 工具将TX回复错期待为`TX`标签，实际是`ACK,cmd=TX`，导致准备超时；修正后只读查询通过。原ERROR保留，不归为固件失败。 / The harness expected a TX tag instead of the actual `ACK,cmd=TX`, causing a preparation timeout. Corrected read-only queries passed. The original ERROR remains and is not a firmware failure. |
| `suite-01-volt1` / `suite-01-volt2` | 准备回复超时，正式采集0s；当时仍持续收到原始字节。没有把0s当120s，也没有因端口仍有数据而忽略控制回复丢失。 / Preparation replies timed out; the formal capture was 0 s while raw bytes continued. These are not 120 s captures, and continuing data does not excuse lost control replies. |
| `retry-01-volt2` / `suite-01-volt3` / `suite-01-res1` | 采满120s，但分别有10/2/9个K尾身份不完整帧被拒收；ASCII CRC可仍通过，因为K身份尾部不属于其测量payload CRC覆盖。原始截断、缺失字段和collector ERROR均保留；不是可用科学帧，也不能直接证明设备配置域改变。 / Completed 120 s, but 10/2/9 frames respectively were rejected for incomplete K-trailer identity. ASCII CRC can still pass because the identity suffix is outside the measurement-payload CRC. Original truncation, missing fields, and collector ERROR remain; these are not usable scientific frames or direct proof of a device configuration change. |
| `suite-01-mixed1` | 120s完成；旧工具误期待C帧，独立复核真实M/MR/K通过，属于保留原ERROR的harness假阳性。 / Completed 120 s. The older harness wrongly expected C; independent validation of actual M/MR/K passed. This is a harness false positive whose original ERROR is retained. |
| `retry-01-volt1` | **真实压力FAIL**：正式窗口15.065057s，原生task WDT报告CPU0的IDLE0未及时喂狗；当时任务快照CPU0=`btController`、CPU1=`sensorarrayScan`。该快照不能证明连续霸核或根因。PANIC未开启，未观察到MFAULT/reset；随后CAP8恢复查询通过，停止压力复测。 / **Real stress FAIL**: the formal window lasted 15.065057 s; native task WDT reported CPU0 IDLE0 missing its feed. The task snapshot showed CPU0=`btController` and CPU1=`sensorarrayScan`; this alone does not prove continuous CPU monopolization or root cause. PANIC was disabled and no MFAULT/reset was observed. CAP8 recovery queries then passed and stress retesting stopped. |

VOLT1失败窗的USB/BLE可信帧为16/858，各自首末可信帧范围内missing为510/126，范围不同，不能直接相减。USB测点有效/无效=115/13（X02=3、X12=10）；BLE=6,282/582（X02=75、X12=507，13点fresh/acquired=0）。完整帧ASCII CRC和identity错误均0，并不消除嵌入WDT文本的截断V帧。BL50首末日志接收间隔7.044092s内`dropD +60`、`dropL/dropC +0/+0`；这是周期日志窗口，不是主动查询或整个15.065s窗口。USB查询间隔29.066543s内`partialWrite +15`、`timeout +23`、`sinkDropped +2819`，包含其自己的准备/收尾范围，不能折算为同窗帧丢失率。详见 [独立WDT复核 / independent WDT review](docs/audit/artifacts/full_benchmark_20261113/retry-01-volt1-watchdog-independent-review.json) 和 [WDT诊断 / WDT diagnosis](docs/audit/artifacts/full_benchmark_20261113/retry-01-volt1-watchdog-diagnosis.json)。

The failed VOLT1 window contains 16/858 trusted USB/BLE frames, with 510/126 missing sequences within each receiver’s own trusted endpoints. Their ranges differ and cannot be directly subtracted. USB valid/invalid points are 115/13 (X02=3, X12=10); BLE has 6,282/582 (X02=75, X12=507, with 13 points fresh/acquired=0). Zero ASCII CRC/identity errors among complete frames does not remove the truncated V frame containing WDT text. Across 7.044092 s between the first and last received BL50 logs, `dropD` increased by 60 and `dropL/dropC` by 0/0. This is a periodic-log window, not active queries or the full 15.065 s capture. USB queries span 29.066543 s and report `partialWrite +15`, `timeout +23`, and `sinkDropped +2819`; their own preparation/cleanup scope cannot be converted into the capture-window frame-loss rate. The linked independent review and diagnosis retain the exact evidence.

#### 软件回归与证据保存 / Software regressions and evidence preservation

最终47个`tools/test_*.py`入口实际全部exit 0，清单记录命令、退出码、耗时、日志、HEAD、dirty hash和源码输入前后SHA；源码输入未改变。此前42入口记录属于早期构建阶段，二者不相加成89项。Host测试覆盖生产函数提取、协议/身份/超时/恢复回归及采集分析器；通过不撤销上述实机压力FAIL，也不等于47项HIL或精度验收。见 [最终47入口清单 / final 47-entry manifest](docs/audit/artifacts/full_benchmark_20261113/final-01-tests-manifest.json)。

All 47 final `tools/test_*.py` entry points actually exited 0. The manifest records commands, exit codes, durations, logs, HEAD, dirty hash, and before/after source-input SHA hashes; source inputs were unchanged. The earlier 42-entry record belongs to the earlier build phase and is not added to produce 89 tests. Host tests cover extracted production functions, protocol/identity/timeout/recovery regressions, and capture analysis. Passing them does not cancel the hardware stress FAIL or constitute 47 HIL or accuracy acceptance tests.

原始数据未覆盖。第二次归档实际exit 0，39项均完成gzip解压逐字节SHA回验，见 [证据索引02 / evidence index 02](docs/audit/artifacts/full_benchmark_20261113/evidence-index-02.json)。首次归档失败原件保留：稳定性`c.run`结束后又有4,687 bytes的final_restore数据，旧summary计数范围不含它，而完整文件SHA相符；见 [计数字节范围复核 / byte-count scope review](docs/audit/artifacts/full_benchmark_20261113/stability-byte-count-scope-review.json)。范围修正没有删除或重构原始帧。

Raw data was not overwritten. The second archive run exited 0 and verified all 39 items by decompressing gzip and matching original bytes/SHA; see evidence index 02. The first failed archive remains: 4,687 bytes of final_restore data followed stability `c.run`, outside the old summary counter scope, while the whole-file SHA matched. The linked scope review explains the correction without deleting or reconstructing raw frames.

### 测试方法与判据 / Method and interpretation

1. **固定输入 / Fix inputs.** 保存HEAD、dirty/source hash、BUILD/PROTO/session、sdkconfig、芯片/Flash/PSRAM、选定SPI/I²C、夹具和rail来源。只用授权被动夹具/实测轨，不自动NVS写入或危险输入注入。 / Record HEAD, dirty/source hashes, BUILD/PROTO/session, sdkconfig, device/flash/PSRAM, selected buses, fixture, and rail provenance. Use authorized passive fixtures/measured rails without automatic NVS writes or dangerous injection.
2. **配置确认 / Confirm configuration.** 逐条请求，按匹配终态和原子CONFIG确认mode/rows/profile/revision；跟踪session/boot、旧epoch与边界。 / Submit requests serially and confirm mode/rows/profile/revision with matching terminals and atomic CONFIG; track session/boot, old epochs, and boundaries.
3. **原始证据 / Raw evidence.** 保存原字节、逐帧时间、rail/battery/ADS/transport事件、命令与退出码；压缩需逐字节解压核对和SHA。不补齐截断行，不用降采样曲线代实录。 / Save raw bytes, frame times, rail/battery/ADS/transport events, commands and exit codes; verify archives byte-for-byte and by SHA. Do not reconstruct truncated lines or replace raw evidence with downsampled plots.
4. **分层计数 / Layered metrics.** 报实际窗口、完整帧、device capture、emitted/提交、USB/BLE各自Host received FPS/序号缺口/协议CRC/identity/结构错误，以及valid/fresh/acquired/Xhh、MFAULT/reboot、计数增量。BLE原notification与重组message分别保存并检查分片完整性，不能把PC实收FPS当capture FPS。 / Report actual duration, complete frames, device capture, emitted/submitted and separate USB/BLE received FPS/gaps/protocol CRC/identity/structure errors, plus valid/fresh/acquired/Xhh, MFAULT/reboots, and counter deltas. Preserve BLE notifications and reassembled messages separately, check fragment integrity, and never equate PC-received FPS with capture FPS.
5. **质量与性能 / Quality and performance.** 同mode/rail/source/profile/校准区间才统计均值/σ/峰峰/无效率，不跨rail跃迁拟合offset。heap最小/最大连续块、栈余量、CPU、恢复时间只报实际测量，缺失标未测。 / Compute mean/σ/peak-to-peak/invalid rates within matching mode/rail/source/profile/calibration intervals, not across rail jumps. Report measured heap minima/largest blocks, stack margins, CPU and recovery time; label missing metrics unmeasured.
6. **独立结论 / Independent outcomes.** 分开build、host、协议/控制、持续运行、吞吐和精度；PASS写明范围。早停保留实际时长，不算完成600/120s。NOT_RUN=未执行，BLOCKED=缺条件，NOT_VERIFIED=缺验收依据。 / Judge build, host, protocol/control, continuity, throughput, and accuracy separately and scope each PASS. Early stops retain actual duration, not completed600/120s. NOT_RUN means unexecuted, BLOCKED means missing prerequisites, and NOT_VERIFIED means insufficient acceptance evidence.

本版真实三芯片断电/复电、故障DMA/I²C时限、示波器SPI/电源边沿、Windows重新枚举、Wi-Fi高负载、多transport同时控制、CPU/最大连续heap块仍未完成实测。BLE高负载已包含于本轮双输出测试，VOLT1失败及BLE丢弃保留；同时接收两链路不等于同时发控制命令，也不证明物理安全。 / Current-version actual three-chip power removal/restoration, fault DMA/I²C bounds, oscilloscope SPI/power edges, Windows re-enumeration, Wi-Fi high load, simultaneous multi-transport control, and CPU/largest heap block remain untested. This round exercised BLE under dual-output load and retains the VOLT1 failure and BLE drops. Receiving two links together is not simultaneous multi-transport control and does not establish physical safety.

## 文档与源码导航 / Documentation and source map

| 文档 / Document | 内容 / Contents |
| --- | --- |
| [快速入门 / Getting started](docs/getting-started.md) | 环境/构建/连接。 / Environment/build/connection. |
| [架构 / Architecture](docs/architecture.md) · [模式 / Modes](docs/measurement-modes.md) | 任务、路由、采样职责。 / Tasks/routes/acquisition ownership. |
| [命令 / Commands](docs/command-reference.md) · [排查 / Troubleshooting](docs/troubleshooting.md) | 完整命令和错误处理；当前边界以本README/契约为准。 / Full commands/troubleshooting; this README/contract states current limits. |
| [输出 / Output](docs/output-format.md) · [Transport](docs/transport-protocol.md) · [BLE](docs/ble-protocol.md) | 帧、分片、通道、计数。 / Frames/fragments/channels/counters. |
| [固件–Host契约 / Firmware–host contract](docs/integration/firmware_host_contract.md) · [软件集成 / Software integration](docs/software-integration.md) | 能力、身份域、终态、兼容性。 / Capabilities/identity/terminals/compatibility. |
| [验证 / Validation](docs/validation.md) · [启动恢复实测 / Startup/recovery HIL](docs/audit/frontend_startup_recovery_20261113.md) | 可重复检查、host/HIL边界、实际结果。 / Reproducible checks, host/HIL boundaries, actual results. |
| [硬件 / Hardware](docs/hardware-overview.md) · [整改 / Revision notes](docs/hardware/reliability_revision_notes.md) · [图纸 / Schematic](datasheets/circuit.pdf) | 连通性、未实施硬件整改、测量限制。 / Connectivity/unimplemented hardware changes/measurement limits. |

| 目录 / Directory | 职责 / Responsibility |
| --- | --- |
| components/ | ADS/FDC/BLE/Wi-Fi/TMUX驱动。 / Device and link drivers. |
| core/board/ · core/boardSupport/ | 板级profile、启动/探测、I²C/SPI/GPIO/中断资源。 / Board profiles, startup/presence, bus/GPIO/interrupt resources. |
| core/config/ · core/measure/ | 配置身份、mode/rows、FDC/ADS/MIXED、校准/数学。 / Identity, mode/rows, FDC/ADS/MIXED, calibration/math. |
| core/transport/ | 有界DATA/LOG/CTRL队列及sink统计。 / Bounded DATA/LOG/CTRL queues and sink statistics. |
| main/control/ · main/output/ · main/ | 共享mailbox、encoder/异步日志/USB、生命周期/调度。 / Shared mailbox, encoder/async logs/USB, lifecycle/scheduling. |
| tools/ · tests/host/ · tests/fixtures/ | Host parser、实际生产函数回归、明确标记的合成/实录夹具。 / Host parsers, production-function regressions, labelled synthetic/recorded fixtures. |
| docs/ | 用户、集成、硬件和审计证据。 / User/integration/hardware/audit documentation. |

## 协作与许可证 / Contribution and license

改命令/协议/硬件相关逻辑时同步更新源码、最小回归和文档，并实际执行检查；硬件结论附设备证据，缺失明确未验证。保护已有改动和原始科学数据，不自动reset仓库、push/发布/保存校准。 / Update source, focused regressions, and documentation together for command/protocol/hardware changes, and actually run checks. Hardware claims need device evidence; mark missing validation explicitly. Preserve existing edits/raw scientific data; do not automatically reset the repository, push/publish, or save calibration.

```sh
python tools/check_command_docs.py
```

许可证见 [LICENSE](LICENSE)。 / See [LICENSE](LICENSE) for the license.

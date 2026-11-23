# 软件架构

## 设计目标

固件把硬件采集与输出传输分成两个异步域：

```text
Serial / BLE FF10 / Wi-Fi CTRL
              |
              v
      共享 command handler                 Core 0
              |
              v
      有界 CommandMailbox
              |
              v
完整帧边界 -> mode/route owner -> FDC 或 ADS -> frame builder   Core 1
                                           |
                                           v
                               TextFrameBus / EventRing
                                           |
                         +-----------------+----------------+
                         v                 v                v
                      USB sink      transport DATA/LOG   async log
                                         |
                              +----------+----------+
                              v                     v
                        BLE TX owner       Wi-Fi persistent queue
                                                    |
                                                    v
                                         Wi-Fi SDK owner    Core 0
```

核心原则：

- Core 1 独占测量模式、矩阵路由与 ADS/FDC acquisition；
- BLE callback、Serial control task 和 Wi-Fi control task 不能直接改硬件；
- 一帧携带采集开始时不可变的 session/boot/revision 和 mode/rows/profile 各域身份；MIXED 的模式最小单位仍为一行；
- source入队、payload容量和应用调度有界，采集与输出分离；Wi-Fi SDK工作由独立owner隔离，不能以MSG_DONTWAIT或外层期限声称底层调用可取消；
- control reply 使用独立路径，不与 measurement DATA 争抢通用 transport payload slot。

## 模块职责

| 模块 | 职责 | 不应承担 |
| --- | --- | --- |
| `core/board` | 引脚、D-line、SELA/SELB、SW、matrix REF 与 ADS profile 真源 | 文本协议与网络策略 |
| `core/boardSupport` | I2C、SPI、GPIO、中断服务与资源生命周期 | mode 业务语义 |
| `core/measure` | mode 状态机、安全 transition、FDC/ADS frame 语义 | BLE/Wi-Fi 发送 |
| `core/measure/fdc` | 双 FDC worker、row epoch、freshness、cache/rescue | ADS 矩阵算法 |
| `core/measure/ads` | ADS 矩阵、rail、battery、math、PGA/autorange | transport 选择 |
| `core/config` | active/pending rows 与 generation | 直接执行 row scan |
| `core/transport` | DATA/LOG sink 选择、slot/descriptor、统计与 control reply | 改变模拟路由 |
| `components/sensorarrayBle` | GATT、CCCD、分片、BLE slot/descriptor 与连接生命周期 | 解析业务命令 |
| `components/sensorarrayWifi` | 持久SDK owner、独立RX、SoftAP/UDP backend、调用隔离与有限槽所有权 | 在共享Transport/控制任务中执行SDK；宣称timeout已取消 |
| `main/control` | 跨 transport 的命令 mailbox | 在 Core 0 应用硬件命令 |
| `main/output` | 文本格式、异步日志、USB sink | 改变采集时序 |
| `main/main.c` | 生命周期、帧边界 apply、engine 调度 | 重复芯片 driver 逻辑 |

## 命令生命周期

命令按以下优先级解析：

1. `sensorarrayScanConfigHandleCommand()`：`ROWS*`；
2. transport runtime handler：`TX/ST/BTX/WIFI`；
3. main runtime query callback：`MODE/STATE/RAIL/CELL/BAT/ADS/...`；
4. legacy mailbox callback：兼容性与低频诊断命令。

查询通常立即读取 snapshot。会改变采集或诊断状态的命令先返回 accepted，再由 Core 1 在完整帧边界处理。CommandMailbox 满时保留最新 operator intent，不允许阻塞 BLE callback。

`WIFI=AP/OFF`有独立owner请求域；AP接受后由Core0的Wi-Fi owner执行，不能借用Core1测量状态机。OFF只改变软件输出gate，终态scope明确；一个active和两个terminal缓存使丢失/迟到WTERM可用精确WIFIREQ查询恢复。CTRL提交和查询不执行SDK。 / Wi-Fi commands have a separate owner-request domain: AP execution belongs to the Core0 Wi-Fi owner, not measurement transitions. OFF changes only the output gate. One active plus two cached terminal records support exact, side-effect-free result queries; shared control does not enter the SDK.

## Wi-Fi owner 隔离与锁顺序 / Wi-Fi owner isolation and lock order

**实机阻塞 / Hardware blocker:** [wifi-control-01](audit/artifacts/cdc_recovery_acceptance_20261116/wifi-control-01/wifi-control-01.summary.json)在AP id1 accepted/running后USB EOF；[只读恢复](audit/artifacts/cdc_recovery_acceptance_20261116/wifi-failure-usj-01.summary.json)确认同9e1e… ELF panic重启至boot2/session370C59E1DF8015F5、CAP8/READY1及Serial/JTAG控制。根因未明，先修复，最终长期/矩阵/切换/SHORT仍NOT_RUN。 / The AP attempt fails and subsequent queries confirm a same-image panic reboot with recovered Serial/JTAG control. Cause remains under diagnosis; final acceptance is held for repair. New-boot idle/zero counters do not prove the previous call was canceled.

AP尝试前wifi-owner-02已[构建/启动核验](audit/artifacts/cdc_recovery_acceptance_20261116/wifi-owner-02-boot-verification.json)，ELF `9e1ee9f9812dcd21a1295e315f08c10bdf98ae55d309e67b1f84b97b3c825573`、session `0ACE6CD97F8649DC`/boot1 CAP8/READY1，Wi-Fi默认OFF/resources0；这不证明AP/UDP、手动重启或新镜像吞吐通过。上次完整吞吐观察仍image06/screen21，新镜像长期/32配置/50切换/SHORT未跑。 / Before the AP attempt, wifi-owner-02 was build/boot verified in the stated identity, with Wi-Fi still OFF and uninitialized. This does not qualify AP/UDP, manual restart or new-image throughput; the last complete throughput screen remains image06, and new-image final tests remain unrun.

本轮新增`components/sensorarrayWifi/sensorarrayWifiOwner.{c,h}`、`sensorarrayWifiWatchdog.{c,h}`和`core/transport/sensorarrayTransportWifiControl.{c,h}`。`sensorarrayWifi.c`保留backend SDK/lwIP操作；Transport只深拷贝提交，真实send返回由完成callback计入旧Wi-Fi sent统计。MSG_DONTWAIT仅是socket flag，lwIP内部mailbox/semaphore仍可能等待；独立任务限制影响范围，不证明SDK本身有界。

The new owner/watchdog/control modules isolate backend SDK/lwIP operations. Transport submits copies; actual send-return callbacks update existing Wi-Fi sent counters. MSG_DONTWAIT does not bound internal mailbox/semaphore waits. A separate task limits the impact of such waiting without claiming the SDK call itself is bounded.

```text
Transport / BLE CTRL / USB CTRL
       | reserve (owner mux), unlock, copy, lock/recheck, commit
       v
 persistent slots: WRITING -> PENDING -> OWNED
       | dequeue with generation/epoch proof; unlock
       v
 Wi-Fi owner -> SDK/lwIP -> actual return -> release/update
       | unlock before completion callback
       v
 Transport statistics mux

100 ms observer -> owner mux -> latch timeout/quarantine -> unlock
                 (no SDK / log / allocation / free / reset)
Independent RX -> UDP command -> shared command handler
```

以上路径不嵌套owner mux与Transport stats锁，不在锁内copy2304 B、SDK调用、日志或callback。应用Wi-Fi事件handler只记录station事件；SDK默认handler仍在SDK线程。独立RX任务不因OFF被盲删或close后视作已停止。WRITING/OWNED永不因超时复用；真实晚返回可释放该次owned slot并记结果，但quarantine保持。未返回的SDK owner/socket/payload继续保留，避免悬空指针和伪取消。

Owner and Transport-statistics locks are not nested. Large copies, SDK calls, logging and callbacks stay outside the owner lock. Application events only record station state; SDK default handlers retain their own thread. OFF does not blindly delete RX or assume closing a socket stopped it. Timeout never recycles WRITING/OWNED storage. An actual late return permits release and completion accounting, but quarantine persists. Nonreturned SDK ownership remains retained.

默认OFF无owner task/timer/payload分配，启动gate要求真实session/boot和两个资源预留参与者完成。首次AP永久分配7,424 B PSRAM\|8BIT，无内部fallback：两个2,304 B普通DATA/LOG槽、一个2,304 B生命周期槽、一个512 B CTRL槽；owner/RX各4,096 B内部栈，Core0/priority5，metadata/mux仍内部。有限资源失败保留查询结果，不重复创建部分活对象；driver_init错误可能隐藏SDK内部cleanup失败，相关对象未知则隔离。原HIGH/BLE_COMPAT配置不做自动降档重试。

Default OFF creates no owner task/timer/payload. The start gate requires real session/boot and both startup resource reservations. First AP allocates7,424 permanent PSRAM bytes without internal fallback: two ordinary slots, one lifecycle and one CTRL slot. Owner/RX each use a4096-byte internal stack on Core0 at priority5; metadata/mux remain internal. Bootstrap failures remain observable and do not recreate partial live objects. Uncertain driver initialization cleanup quarantines ownership. No automatic HIGH-to-BLE_COMPAT retry is added.

AP执行5s＋最多1s cleanup、总6s，发送500ms是应用观测期限；100ms timer只锁内读取/置状态。Wi-Fi自己的1s send-start TTL取保护包原deadline较早值，不刷新，禁止过期后启动；持续CTRL/隔离时**不承诺pending在1s内释放**。共享Transport保护descriptor则每dispatch有限清理；二者不同。第一err与cleanupErr分离，真实sdkErr/errno/returned及late不能冒充取消或radio off。默认OFF及WIFI=OFF都不是物理断电证据。

AP execution/cleanup/total budgets are5/1/6 seconds and send observation500ms; the100ms timer only observes/latches state. Wi-Fi's own one-second send-start TTL retains an earlier protected deadline and forbids stale starts, without promising pending-slot reclamation during control starvation or quarantine. Transport performs its separate bounded expiry housekeeping. First error, cleanup error and actual/late SDK return evidence remain distinct. Neither default OFF nor WIFI=OFF proves physical power removal.

新`CPUOWNER?7=WIFI_OWNER`独立于6=WIFI_CTRL；原SCHED/SCHEDWHY七组字段不改。WIFIQ只返回owner完成工作后缓存的stackMinBytes，查询不扫描挂起任务。接口/字段与cap分页见[主机契约](integration/firmware_host_contract.md#wifi-owner-contract-v1)。[host02](audit/artifacts/cdc_recovery_acceptance_20261116/wifi-owner-host-02/manifest.json)14入口通过，最后完整吞吐筛查仍image06/screen21；新AP、内存、栈、并存吞吐和物理恢复待新候选单独验收。

CPUOWNER role7 measures the Wi-Fi owner separately from role6 control. Existing SCHED fields are unchanged. Stack queries use cached owner observations rather than scanning blocked tasks. The linked contract defines capability paging and fields. All14 host02 entries pass; new AP/memory/stack/coexistence/recovery HIL remains separate from the last completed image06 screen.

## 显式重启与SDK shutdown / Explicit restart and SDK shutdown

本轮新增`sensorarrayUsbOtgManualRestart(bool)`只由已受理RESTART/USBBOOT执行点调用；RACK本地接受和RAPP意图仍先记录。USB owner准备的250ms期限/原DMA fence不变；成功经已有linker wrapper进入当前S3/IDF5.5.5 noos，避开esp_restart先运行的潜在阻塞Wi-Fi shutdown handler。generic/AUTO仍走原esp_restart；USB准备失败仍走原emergency Core Reset/USJ恢复，未知owner不释放。

The new manual entry serves accepted RESTART/USBBOOT only. It retains local acceptance/intent records and the250ms USB preparation/DMA fence, then uses the existing wrapped S3/IDF5.5.5 noos path to bypass potentially blocking shutdown handlers. Generic/AUTO still uses esp_restart; failed USB preparation retains the emergency Core Reset/USJ path and unknown ownership.

此终止路径绑定SDK私有API；RTC watchdog不是精确1s物理恢复保证，跳过最终wall-clock同步影响日历连续性。科学帧时间仍按boot域，不把重启/USB寄存器回读称模拟安全。源码/74个host场景见[独立审查](audit/artifacts/cdc_recovery_acceptance_20261116/manual-restart-independent-review-01.json)；最终ELF wrapper和实机RESTART/USBBOOT/AP条件恢复必须另证。

This terminal path depends on a pinned private SDK API. RTC watchdog setup does not establish an exact physical one-second bound; bypassing final wall-clock synchronization affects calendar continuity. Science time remains boot-scoped, and reset/USB readback is not analog safety. The linked74-scenario source/host review does not replace final-target or physical recovery evidence.

## Transport payload 所有权

通用 DATA/LOG transport 使用预分配静态 payload pool 和小 descriptor queue：

- 7 个 payload slot：4 个普通 DATA/LOG、1 个 lifecycle、2 个完整保护快照（各2304 bytes）；
- DATA / LOG / LIFECYCLE / protected descriptor queue 深度为 2 / 2 / 1 / 2；
- consumer 可持有一个待重试保护owner，并处理另一个descriptor；最多2个保护槽总量不会随重试增加；
- 每次dispatch先做常数有界到期清理，再按 lifecycle → protected → 到期可重试owner → 普通DATA → LOG 消费；控制回复和guaranteed终态另保留优先通路；
- LOG 最多占2个普通槽；不能占lifecycle或保护槽；
- enqueue不等待网络。pending被dequeue后才归consumer所有，存活项按原FIFO放回；WRITING/SDK-held对象不回收。

Transport has seven slots: four ordinary, one lifecycle and two protected full packets. Dispatch performs bounded expiry housekeeping before lifecycle, protected, due retry, ordinary DATA and LOG selection. Reserved control/guaranteed terminals remain ahead. A task-owned retry and pending newer frame can coexist within the same two protected slots. Only a dequeued descriptor transfers ownership; surviving entries retain FIFO order, and WRITING/SDK-owned objects cannot be reclaimed.

保护network descriptor的1秒TTL从入队开始，包含本层等待和NO_MEM重试；每次dispatch最多检查两个pending和一个明确持有的retry，不调用SDK/lwIP。expired每descriptor一次，bleExpired/wifiExpired按入队捕获资格及未提交状态分别计数；Wi-Fi已尝试不重复发送/计数。普通network descriptor不因此自动获得同一TTL。新Wi-Fi owner接受副本后另有自己的发送起始期限，SDK mailbox/sem等待不由任何排队清理证明有界。

The one-second TTL covers only protected network descriptors from enqueue, including local waiting and NO_MEM retries. Each dispatch inspects at most two pending entries and one owned retry without SDK/lwIP calls. Per-descriptor and per-sink expiry counters are distinct; an attempted Wi-Fi copy is not retried or counted twice. Ordinary network descriptors do not inherit this TTL. The new Wi-Fi owner has a separate send-start deadline after admission; queue cleanup does not prove bounded SDK waits.

producer 在短临界区内只分配 `inUse/generation`，2304-byte copy 和 metadata 解析在临界区外完成。consumer 校验 slot index、generation、length 与 ownership，发送完毕后释放。禁止在 hot path 使用 `malloc(2304)`。

BLE另有4普通DATA、2LOG、2CTRL及2完整保护槽。typed publication同步深拷贝原完整encoder包或普通preview，携带connection/stream generation和不可变配置身份；不会将header解析当作完整帧/CRC证明。 / BLE owns separate pools and synchronously copies the selected full/preview bytes. Header parsing alone never proves a complete CRC-valid frame.

build04新增BLE内部all-full完成依据：仅当前stream/同boot/config且seq前进的typed完整包成功可更新周期due=完成＋1秒；普通FULL可满足，SHORT/raw/failed不能。source/Transport仍保留自己的2保护槽与周期候选，但PERIODIC不强制另起BLE时钟；NET bleAccepted只说明候选被BLE准入，最终可归普通。BLESNAP仍protected-only，不把其age/gap当all-full时钟；已accepted owner不取消。 / Build04 adds private advancing typed-full completion evidence. Ordinary FULL satisfies the periodic obligation; preview/raw/failure does not. Source/Transport reservations remain, while upstream PERIODIC does not create another BLE cadence. NET admission need not be BLE protected classification. Existing BLESNAP age/gap remain protected-only, and accepted owners are retained.

screen19在build04的FULL实测中，源/NET候选仍增长而BLE保护queued/localComplete/staleNormal/deadlineMiss均无新增，PC完整帧持续到达；BLESNAP保护age约125秒不能解释成数据停顿。正常缺口仍3.233734%>2.5%，本地capacityReject增加228，不能宣称该调度已无瓶颈。完整窗口/身份/限制见[结果](../README.md#screen19-coalescing-results)。 / In build04's FULL hardware screen, source/network candidates grow while BLE protected counters remain unchanged and full PC receipt continues. A roughly 125-second protected age is not a data outage. Normal missing-frame acceptance still fails at 3.233734%, with 228 local capacity rejections; the architecture is not yet proven free of throughput bottlenecks. See the linked scoped results.

build06本轮新增`FramePublicationMetadata → PublishFrameMetadata → ApplyFrameMetadata`从成功encoder配对frame直接传不可变POD到私有Transport slot，保持各generation/request域和SHORT；raw/旧API仍parser。POD不认证任意wire，不能绕过producer成功编码要求。build05内部2,304 B工作副本因未证明收益已撤回。screen21仅正常缺口数值通过，Wi-Fi SDK owner隔离与最终长测仍未闭合，见[完整证据](../README.md#screen21-typed-metadata-results)。 / Build06 passes typed immutable metadata from the successful encoder/frame pair, preserving domains and SHORT while legacy/raw APIs still parse. It does not authenticate arbitrary input. The unproven internal working copy is removed. Numeric screen21 improvement is not overall or Wi-Fi-isolation acceptance.

build05增加单BLE TX owner的2,304 B内部工作副本，永久slot仍直到真实send返回才释放，未知SDK owner不因scratch复制可回收。队列/身份/协议不变；screen20仍2.636043%缺口FAIL且BLE_TX CPU未下降，尚无性能收益证明，内部连续块代价也保留。见[对照证据](../README.md#screen20-working-copy-results)。 / Build05 adds one internal 2,304-byte TX working copy while retaining permanent-slot ownership through actual send return. It does not make unknown SDK owners reclaimable. Protocol/queue semantics remain; screen20 still fails, with no demonstrated CPU benefit and explicit internal-memory cost.

## Async logger workspace

source现行配置为4普通＋2保护槽；first-config/每1秒下一份真实full候选优先保留，USB按同一source FIFO消费。过载可替代明确排队的旧保护项并计数，不能覆盖WRITING或consumer-owned对象。 / Source reserves two additional full-frame slots; USB preserves source FIFO, and only clearly queued objects can be superseded with explicit counters.

`sensorarrayLogTask` 使用启动时一次性分配、永久保留的task-owned workspace保存summary/frame/text packet scratch，并独立永久分配完整frame slots；现行配置优先SPIRAM|8BIT，失败才显式internal fallback。日志栈用WithCaps请求PSRAM，但TCB仍在内部。按对象生命周期reset，不每帧memset整个workspace，也不在热路径分配2304-byte packet。575c… ELF的target DWARF为workspace13,536 B、frame slot7160 B×6；APP_LOG_INIT的实际memory字段才证明该次启动的放置位置。

The logger uses boot-allocated, permanent task-owned workspace and separate permanent full-frame slots, preferring SPIRAM|8BIT with an explicit internal fallback. Its WithCaps stack requests PSRAM while the TCB remains internal. Object-specific reset avoids whole-workspace clearing and hot-path packet allocation. Target DWARF for ELF 575c… reports a 13,536-byte workspace and six 7,160-byte slots; actual APP_LOG_INIT memory fields establish placement for a particular boot.

普通scan task仍需要内部连续栈16,384 B及独立TCB。BLE新增保护机制的575c…候选因scan_task_create_failed/NO_MEM未启动采集，总heap约8.29MB不能证明内部最大块足够。新内存修复源码已把BLE CPU payload的18,432 B迁入一次性SPIRAM|8BIT分配，不移动锁/队列/metadata/DMA或采集栈；Transport内部7槽池17,216 B保持。heap_v1下MEM?从锁外低频读取internal8bit free/largest/min及PSRAM指标，并保存scan创建前后snapshot；readout=sequential不保证原子多heap观测。build03已独立实证scan创建和READY1，持续HIL仍按各自窗口验收，详见[审计报告](audit/firmware_reliability_fix_report.md)。

The ordinary scan task still needs a 16,384-byte contiguous internal stack plus a separate TCB. Protected-snapshot candidate 575c… fails with scan_task_create_failed/NO_MEM before acquisition; approximately 8.29 MB aggregate heap does not prove sufficient internal largest block. The corrected source places 18,432 CPU-only BLE payload bytes in one permanent SPIRAM|8BIT allocation, retaining locks, queues, metadata, DMA and acquisition stacks. Transport's internal 17,216-byte seven-slot pool stays unchanged. Under heap_v1, MEM? reads internal8bit free/largest/min and PSRAM metrics at low frequency outside snapshot locks, with saved scan-create snapshots. readout=sequential is not an atomic multi-heap observation. Build03 independently verifies scan creation and READY1; sustained HIL remains scoped to its own observation windows.

## Backpressure 与优先级

```text
control/lifecycle > protected full snapshot > ordinary DATA > diagnostic LOG
```

- control reply 使用 BLE CTRL slot 或源端 peer reply；
- DATA/LOG 使用通用 transport pool，但 DATA 有独立 queue 并优先消费；
- BLE/Wi-Fi/USB 各自统计 sent/drop；
- disconnect清订阅并推进connection/stream身份，drain明确排队owner；正在SDK调用中的owner必须等已知完成，不把timeout当取消；
- stale descriptor、release mismatch 与 CRC mismatch 必须计数，不能静默忽略。

## 状态一致性

mode request 的 accepted 与 applied 是两个事件。Core 1 transition 会先撤销激励、停止 conversion、切换前端、配置并 readback，再更新 generation。失败时进入 SAFE/DEGRADED，不发布伪造的正常帧。

权威恢复使用原子 `CONFIG?` 的session/boot/revision/各域ID与appliedFrameBoundary；`MODE?`/`STATE?`不触碰硬件，但分次查询不能拼成未曾存在的配置。CAP旧gen/rid属于rows，V/R属于mode，M另有profile；不得用一个generation数值比较所有模式，更不能跨boot排序。旧排队帧保留原身份或明确丢弃，禁止重盖新标签。

Authoritative recovery uses atomic CONFIG identity and applied boundaries. Separate queries are not one atomic configuration. CAP legacy gen/rid is rows-owned, V/R is mode-owned, and MIXED adds profile identity. Never compare all domains through one generation or order across boots. Queued data retains its original identity or is explicitly dropped.

BLE输出策略见[主机契约](integration/firmware_host_contract.md#protected-snapshot-v1)：正常FULL不主动丢2.5%，只有本地容量连续压力才渐进降低普通BLE输出；采集独立。1秒BLE周期是due/miss义务，CTRL洪流可能延后first，不是交付deadline。 / BLE adapts only ordinary output to sustained local capacity pressure; acquisition stays independent. The BLE one-second period is a due/miss obligation, not a delivery deadline.

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

`BTX=FAST` 使用 `confirm=false` 的 Notify，客户端应启用 Notify bit。`BTX=SAFE` 在 Indicate bit 可用时使用 `confirm=true` 的 Indicate；若客户端只启用了 Notify，固件使用 `confirm=false` 的有界 Notify fallback。CCCD gating 按当前 TX mode 执行：`0x0001` 允许 FAST 和 SAFE fallback，`0x0002` 只允许 SAFE，`0x0003` 同时允许 FAST 与带确认的 SAFE。连接中需要往返切换 FAST/SAFE 时推荐使用 `0x0003`。

每个 TX characteristic 独立 gating：

- `FF11` CCCD 只控制 CTRL reply；
- `FF20` CCCD 只控制 DATA；
- `FF30` CCCD 只控制 LOG；
- 订阅 `FF20` 不能隐式启用 `FF30`。

## FAST 与 SAFE

| Mode | GATT send | 特点 |
| --- | --- | --- |
| `BTX=FAST` | Notify | 无 per-packet confirmation；拥塞时有界 drop |
| `BTX=SAFE` | Indicate；Notify-only 时 fallback 为 Notify | Indicate 路径等待 `ESP_GATTS_CONF_EVT`；超时有界，不能永久占用 slot |

安全切换流程：

```text
1. FF11 CCCD 先开启 Indicate（推荐 0x0003）
2. SAFE 下需要 DATA/LOG 时，FF20/FF30 也先开启 Indicate
3. FF10 <- BTX=SAFE\n
4. FF11 -> ACK,cmd=BTX,v=SAFE（Indicate；Notify-only fallback 时为 Notify）
5. FF10 <- BTX?\n
6. FF11 -> ACK,cmd=BTX,v=SAFE（Indicate；Notify-only fallback 时为 Notify）
```

`BTX=SAFE` 会先更新 firmware TX mode，再格式化并发送本条命令的 source-local 回复。FF11 有 Indicate bit 时 ACK 使用确认 Indicate；只有 Notify bit `0x0001` 时 ACK 使用 SAFE Notify fallback，不会因 CCCD 不匹配而丢失。不要盲目重复会改变状态的 setter。

切回 FAST 也应在发送 `BTX=FAST` 前确保 FF11 Notify bit 已开启；使用 `0x0003` 可避免切换后的回复落入不兼容 CCCD。HIL 必须真实验证 Windows/Bleak 与 nRF Connect 对 Notify/Indicate 的订阅行为，不能只看 characteristic properties。

## MTU 与 message 上限

- GATT attribute value 上限：512 bytes；
- DATA/LOG 完整 message 上限：2304 bytes；
- CTRL TX 完整 message 上限：512 bytes；
- CTRL RX 命令上限：128 bytes；
- 实际单次 payload 由 negotiated MTU 的 ATT payload 决定。

如果完整 message 能放入当前 ATT payload，firmware 直接发送 raw ASCII；否则分片。

## BLE 内部 slot 与 descriptor

BLE component 在通用 transport pool 之后拥有自己的静态 payload pools。当前有效配置为：

| Channel | Payload slot | 数量 | Queue |
| --- | ---: | ---: | --- |
| 普通 DATA / `FF20` | 2304 bytes | 4 | 独立DATA FIFO，深度4 |
| 普通 LOG / `FF30` | 2304 bytes | 2 | 独立LOG FIFO，深度2 |
| CTRL / priority lifecycle | 512 bytes | 2 | 独立高优先级queue |
| 保护完整 DATA / protected full DATA | 2304 bytes | 2 | 独立FREE/WRITING/PENDING/OWNED槽与ticket |

各pool独立，descriptor大小上限40 bytes，完整payload不放入descriptor queue。发送选择顺序为CTRL/关键生命周期 → protected → 普通DATA → 普通LOG。普通LOG在有效DATA订阅期间拒绝新准入并计数；已拥有对象不取消。保护槽最多容纳一个消费者持有的旧帧和一个待发送新帧；只可替代明确PENDING的对象，不能回收WRITING/OWNED或未返回的SDK对象。

Pools are independent; descriptors are at most 40 bytes and do not contain full payloads. Dispatch prioritizes CTRL/critical lifecycle, protected full frames, ordinary DATA, then ordinary LOG. Effective DATA subscription rejects and counts new ordinary LOG admissions without canceling existing owners. Protected slots can retain a consumer-owned older frame and a pending newer frame. Only clearly pending objects may be superseded; WRITING/OWNED and nonreturned SDK objects cannot be reclaimed.

BLE CTRL reply 在 Bluedroid 接受 payload 前采用有界重试：

- 只对 CTRL channel 的返回后重试生效；DATA/LOG不因此重放。单次SDK调用可能内部阻塞，不能称为强制非阻塞；
- retry interval 为 10 ms，总 retry window 为 500 ms；
- 只把 `ESP_FAIL` 视为可重试的“尚未接受”结果，其他 error 立即返回；
- 如果 SAFE indication 的 `esp_ble_gatts_send_indicate(..., confirm=true)` 已返回 `ESP_OK`，说明 Bluedroid 已接受该 reply；之后即使 confirmation timeout，也绝不重发，避免客户端收到重复的控制回复。

该retry不重新调用业务setter，只重试尚未被BLE stack接受的source-local reply bytes。它不提供网络exactly-once：Host超时重发命令仍须遵循requestId/终态查询契约。 / Reply retry never re-executes the setter and does not provide network exactly-once; host retries must follow request-ID and terminal-query rules.

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
- 单连接内reassembly key为 `(channel,messageId)`；跨连接必须加入connection/stream实例并清旧未完成重组，不能只用messageId。

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
- 仅release明确排队/已完成的slots；SDK未返回的owner保留隔离，不盲删任务；
- 重新开始 advertising。

descriptor发送前校验slot generation、connection generation；typed DATA还校验effective stream generation。有效退订/重订或模式资格变化即使未断连接也推进stream边界，旧descriptor不能满足新first/periodic义务。跨boot/session不比较generation大小。 / Typed DATA also validates effective stream generation, isolating resubscription without disconnect. Old descriptors cannot satisfy new obligations; generations are never ordered across boots.

## 完整保护快照与只读诊断 / Protected full frames and diagnostics

本轮新增 `sensorarrayBleNotifyFrameForGeneration` 同时接收原完整encoder包、普通输出包（可为SHORT预览）、不可变session/boot/revision/sequence及保护flags。组件在同一短锁下决定stream/保护资格并保留WRITING槽，锁外计算CRC/深拷贝，重新核验身份后发布；返回前完成复制，不保留调用者栈preview指针。`ESP_OK`只代表BLE准入；`ESP_ERR_NOT_ALLOWED`表示有计数的普通策略跳过，`NO_MEM`为本地容量拒绝，stale/guard/CRC错误不能伪装为capacity证据。既有G envelope和科学K CRC保持原字节。

The new typed API receives the original full encoder packet, an ordinary packet that may be a SHORT preview, immutable identity and protection flags together. Under one short lock it selects stream/protection eligibility and reserves a WRITING slot. CRC calculation and deep copy run outside the lock; publication then revalidates identity. Copying finishes before return, with no retained caller-stack preview. ESP_OK means BLE admission only; NOT_ALLOWED denotes counted ordinary policy skipping, NO_MEM denotes capacity rejection, and stale/guard/CRC failures are not capacity evidence. G envelopes and scientific K CRCs retain their original byte semantics.

首配置独立优先保护；build04周期依据最近成功且seq前进的同身份typed完整帧完成＋1秒，普通FULL可满足，SHORT预览不可满足，逾期才保护下一份真实full帧，保留invalid/Xhh；“完整”不要求所有点有效。普通CAP SHORT仍为preview，保护必为full。CTRL持续繁忙可能让first长时间PENDING；BLE1秒为due/miss而非完成期限，`deadlineMiss`必须公开。Transport另外实施保护network descriptor的共享1秒TTL，source USB FIFO独立；到期逐sink计数不代表PC收件。

First-configuration protection stays independent. From build04, an advancing same-identity typed-full local completion anchors periodic due one second later. Ordinary FULL qualifies, SHORT preview does not, and an overdue obligation protects the next real full frame while retaining invalid/Xhh. Complete is distinct from all-point validity. Ordinary CAP SHORT remains a preview; protected traffic stays full. Continuous CTRL can leave first pending indefinitely: BLE's one-second interval is due/miss scheduling, with deadlineMiss exposed, not a completion deadline. Transport separately applies a shared one-second protected-network-descriptor TTL; source USB FIFO stays independent. Per-sink expiry is not peer receipt.

正常 `BLEFLOW.stage=full`不主动丢2.5%。仅当连续3个有效1–2秒窗的本地容量拒绝率严格>20%，才每至少3秒降50‰；累计10秒有效clean窗升50‰，clean证据TTL3秒，受限时至多每秒一次真实候选探测。`peerLoss/rssi`保持unknown。四个v1只读能力查询 `BLEFLOW? / BLESNAP? / SNAPSRC? / SNAPNET?` 及全部字段/未知值/窗口解释见[主机契约](integration/firmware_host_contract.md#protected-snapshot-v1)。PROTO2新增ble_flow_v1用于BLEFLOW，ble_snapshot_v1用于BLESNAP/SNAPSRC/SNAPNET；无新setter，缺cap或旧固件unsupported不能当零。

Normal full policy does not deliberately drop 2.5%. Three valid 1–2-second windows strictly above 20% local capacity rejection permit 50‰ reductions at least three seconds apart. Ten accumulated valid clean seconds permit a 50‰ increase; clean evidence expires after three seconds, with at most one real-frame recovery probe per second while limited. peerLoss/rssi remain unknown. The [host contract](integration/firmware_host_contract.md#protected-snapshot-v1) defines all four v1 read-only query fields, unknowns and windows. PROTO2 advertises ble_flow_v1 for BLEFLOW and ble_snapshot_v1 for BLESNAP/SNAPSRC/SNAPNET. There is no new setter; absent capabilities or unsupported old firmware do not imply zero.

`BLESNAP.failed`不包含所有drop，不能用queued−localComplete−failed算队列深度。localComplete只计BLE保护分类的完整本地完成，不是全部full或Host应用确认；普通FULL持续成功时保护age可能增长，不用于推断PC周期缺失；没有PC ACK，不承诺1Hz必达或exactly-once。500ms guard隔离未返回SDK对象，不代表取消；普通Wi-Fi MSG_DONTWAIT也不能证明SDK mailbox/sem不阻塞。

BLESNAP.failed excludes some drop paths, so queued−localComplete−failed is not queue depth. localComplete counts protected-class whole local completion, not all-full traffic or host acknowledgment. Protected age may grow during successful ordinary FULL streaming and cannot diagnose PC period gaps. Without PC ACK there is no guaranteed 1 Hz delivery or exactly-once. The 500 ms guard quarantines a nonreturned SDK owner rather than canceling it; Wi-Fi MSG_DONTWAIT likewise cannot prove nonblocking SDK mailbox/semaphore internals.

screen18在FULL 1000‰、intentionalSkip0的采样端点仍有3.015075%完整帧缺口，超过正常2.5%预算；本地保护complete239与PC实收不可互换，119/129/239各层差异后来已用真实源码host复现重复周期，未归因全部缺帧。实测/窗口/原始证据见[README](../README.md#screen18-protected-results)，弱链路、SHORT和配置首帧保护仍未验收。 / Screen18 exceeds the normal gap budget despite sampled full-rate/zero intentional skips. Local protected completion is not PC receipt, and later actual-source host timing reproduces duplicate cadence without attributing all PC gaps to it. See the linked measured results; weak-link, SHORT and first-configuration protection remain unaccepted.

上游PERIODIC仅为保护候选，BLE可合并已有周期并按普通FULL/SHORT规则处理当前帧；NET bleAccepted不保证BLE最终class。内部all-full依据不暴露为新字段，旧查询字节/语义不重释。旧真实源码错相反例及build04独立回归见[机制说明](../README.md#ble-periodic-coalescing)；screen19已闭合，周期合并生效但正常缺口预算仍FAIL。 / Upstream PERIODIC is a reservation candidate that BLE may coalesce into ordinary handling. NET acceptance does not promise final BLE class. Private all-full evidence adds no public-field reinterpretation; screen19 now closes with working coalescing but another normal gap-budget failure.

screen19同a6b0…镜像/session B4828F8EF45001F1录制120.000139309秒：USB7,728连续，BLE7,481缺250，3.233734%>2.5% FAIL。BLE保护新增queued/complete/staleNormal/deadlineMiss均0，lastSeq609/age125,788,271µs；PC完整帧仍有最大相邻42.942640ms。Source周期+119、NET候选+124不要求变成BLE保护；FLOWcapacityReject+228/attempts+7,738、intentionalSkip0仍需修复，不能从250扣减或称RF丢包。完整包含502/471无效USB/BLE点，ADC独立0/82,589；CAP8收尾核验。详见[闭合结果与原件归档](../README.md#screen19-coalescing-results)。

Screen19 records 120.000139309 seconds on the same a6b0… image/session: 7,728 consecutive USB frames and 7,481 BLE frames with 250 missing observations, **3.233734% > 2.5%, FAIL**. No new BLE protected queued/completed/stale/deadline-miss events occur, and protected age reaches 125,788,271 µs; PC full frames still have a maximum adjacent interval of 42.942640 ms. Source +119 and NET +124 candidates need not become BLE protected frames. Remaining capacity rejection +228 over 7,738 attempts with zero intentional skips requires further repair; it cannot be subtracted from 250 or called RF loss. Complete frames preserve 502/471 invalid USB/BLE points; the separate ADC window is 0/82,589. CAP8 recovery is verified. See the [closed results and archive](../README.md#screen19-coalescing-results).

build04的[CAP8 SHORT专项](../README.md#short-snapshot-results)已观察118个完整保护快照/120.000123756秒，PC最大完整间隔1.028126934秒，USB1416连续完整帧、118原包相同。普通预览导致完整序号跳跃不套FULL2.5%缺口预算；完整周期是due调度，不承诺严格每秒必达。历史TX恢复闭合经原始ACK/query核验，未补造日志；这不是主FULL吞吐PASS。 / The CAP8 SHORT run observes 118 full snapshots in 120.000123756 seconds, at a maximum adjacent full interval of 1.028126934 seconds. USB has 1,416 consecutive full frames and 118 common original-byte matches. Preview-induced full-sequence skips are not the normal FULL loss denominator. Due scheduling is not strict one-second delivery; verified historical restoration does not turn the main FULL screen into PASS.

build06仅替换内部重复包头解析的typed metadata路径并撤回build05 scratch，不改本wire/CRC/域/SHORT/保护分类。[screen21](../README.md#screen21-typed-metadata-results)141/7710=1.828794%只通过正常缺口预算；所有完整帧仍按真实CRC/identity验证，invalid保留，周期/first/弱链路和整体长测不因此自动通过。 / Build06 changes the internal metadata handoff and removes scratch without changing this wire/protection contract. Screen21 meets only the numeric normal gap budget; integrity, periodic/first-frame, weak-link and broader acceptance remain separate.

build05内部发送工作副本不改本协议或保护分类；[screen20](../README.md#screen20-working-copy-results)仍2.636043%正常缺口FAIL。FLOW sampled pressure→full时两端rate1000，pressure本身不是主动降频。旧build04专项记录不作为新session连续交付证明。 / Build05's internal working copy leaves the wire/protection contract unchanged. Screen20 still fails normal gap acceptance; sampled pressure→full retains rate1000, so pressure itself is not decimation. Earlier build04 observations do not prove continuous delivery in the new session.

## 分页能力与Wi-Fi控制独立性 / Paged capabilities and isolated Wi-Fi control

**实机阻塞 / Hardware blocker:** [wifi-control-01](audit/artifacts/cdc_recovery_acceptance_20261116/wifi-control-01/wifi-control-01.summary.json)在AP id1 accepted/running后USB EOF；[只读恢复](audit/artifacts/cdc_recovery_acceptance_20261116/wifi-failure-usj-01.summary.json)确认同9e1e… ELF panic重启至boot2/session370C59E1DF8015F5、CAP8/READY1及Serial/JTAG控制。根因未明，先修复，最终长期/矩阵/切换/SHORT仍NOT_RUN。 / The AP attempt fails and subsequent queries confirm a same-image panic reboot with recovered Serial/JTAG control. Cause remains under diagnosis; final acceptance is held for repair. New-boot idle/zero counters do not prove the previous call was canceled.

AP尝试前wifi-owner-02已[构建/启动核验](audit/artifacts/cdc_recovery_acceptance_20261116/wifi-owner-02-boot-verification.json)，ELF `9e1ee9f9812dcd21a1295e315f08c10bdf98ae55d309e67b1f84b97b3c825573`、session `0ACE6CD97F8649DC`/boot1 CAP8/READY1，Wi-Fi默认OFF/resources0；这不证明AP/UDP、手动重启或新镜像吞吐通过。上次完整吞吐观察仍image06/screen21，新镜像长期/32配置/50切换/SHORT未跑。 / Before the AP attempt, wifi-owner-02 was build/boot verified in the stated identity, with Wi-Fi still OFF and uninitialized. This does not qualify AP/UDP, manual restart or new-image throughput; the last complete throughput screen remains image06, and new-image final tests remain unrun.

本轮新增Wi-Fi owner接口可通过现有FF10写命令、FF11收control回复；不增加UUID、不改CCCD、G envelope或科学K CRC。先从PROTO的`caps2_v1`协商`CAPS2?`，再查`wifi_owner_v1`；能力存在不表示AP已启动。`WIFI? ver=2`采用单owner snapshot，区分requested、outputGate、backend及pending；不能按旧同步ACK把ok0直接当最终失败。当前cap页为`CAPS2,v=1,page=1,caps=wifi_owner_v1,next=none`。

The new Wi-Fi owner commands use existing FF10 writes and FF11 control replies, with no UUID, CCCD, G-envelope or scientific K-CRC change. Negotiate caps2_v1 through PROTO, then wifi_owner_v1 through CAPS2. Capability is not AP readiness. Version2 WIFI snapshots separate requested mode, output gate, backend and pending state; ok0 is not automatically a terminal failure under the new asynchronous contract.

`WIFI=AP`受理后返回独立id/epoch，WTERM可先于或晚于ACK；精确无副作用`WIFIREQ?<非零u32>`恢复该结果，不重发动作模拟查询。一个active＋两个terminal缓存，已淘汰返回unknown。OFF立即应用软件gate，**不证明radio off或取消SDK**，不清quarantine。WTERM最多3次200ms间隔本地提交，FF11 notify/网络队列成功不是PC ACK；CTRL被保护仍不能保证断线期间必达。`WIFIGUARD?`和`WIFIQ?D/L/C`只读缓存，不让BLE控制调用Wi-Fi SDK。

AP acceptance has a separate ID/epoch, and WTERM may arrive before or after ACK. Query the exact nonzero request ID without replaying the action. One active plus two terminal records are cached; eviction returns unknown. OFF applies only the output gate and does not prove radio shutdown, SDK cancellation or quarantine recovery. Up to three local WTERM submissions200ms apart are not peer acknowledgements. Cached guard/channel queries do not enter the Wi-Fi SDK from BLE control.

新Wi-Fi SDK owner隔离可能阻塞的lwIP/AP调用；Transport只提交持久深拷贝。其send500ms观察期限不表示cancel，自己的1s send-start TTL也不等于BLE周期due或Transport保护descriptor TTL。driverAcceptedBytes/sent以实际sendto返回计，peerReceipt保持unknown。CPUOWNER7为新WIFI_OWNER，旧0–6及SCHED七字段不重释；stack查询是owner完成工作后缓存值。全部字段/单位/unknown/回绕与兼容门禁见[主机契约](integration/firmware_host_contract.md#wifi-owner-contract-v1)。

An isolated Wi-Fi owner executes potentially blocking lwIP/AP work; shared Transport submits persistent copies. Its500ms send observation is not cancellation, and its one-second send-start TTL differs from BLE periodic due scheduling and Transport protected expiry. Actual sendto returns drive byte/sent counters; peer receipt stays unknown. New CPUOWNER7 is WIFI_OWNER without renumbering existing roles/SCHED fields. Stack observations are cached after owner work. The linked contract defines all fields, units, unknowns, wraps and compatibility gates.

已受理的RESTART/USBBOOT另有本轮新增manual入口：同250ms USB准备成功后，经过现有wrapper进入当前S3/IDF5.5.5 noos，避开可能阻塞的Wi-Fi shutdown handler；generic/AUTO不改。RAPP restarting只表示意图，重连后仍须核对新BUILD/session/READY，不按旧seq/generation跨boot排序。源码/host已验证，新目标ELF/实际BLE请求重启、ROM枚举及AP并存仍需新HIL；最后完成筛查仍image06/screen21，不能据其数值预算通过宣称新Wi-Fi/重启通过。

Accepted RESTART/USBBOOT uses the new manual entry after the same250ms USB preparation fence, entering the pinned wrapped noos path to bypass potentially blocking Wi-Fi shutdown handlers. Generic/AUTO remains unchanged. Restart intent is not completed reset; reconnect must verify fresh BUILD/session/READY without ordering sequence/generation across boots. Source/host evidence does not qualify the new target's physical restart, ROM enumeration or AP coexistence. The last completed image06 screen's numeric pass is not new Wi-Fi/restart acceptance.

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

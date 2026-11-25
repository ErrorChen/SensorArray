# 固件与主机契约 / Firmware–host contract

接入时保存原始回复、接收接口与主机单调时间。运行镜像以 `BUILD?.elfSha256` 辨识，修复提交和历代验证记录见[迭代记录 / Revision history](../history/README.md)。 / Identify the deployed ELF from BUILD and associate every observation with its receiving interface and host monotonic time.

## 能力协商 / Capability negotiation

查询 `PROTO?`、`BUILD?`、`CONFIG?`。协议 v2 的 `config_identity_v1/config_query_v1/session64` 支持原子配置身份。`dataMax=2304` B，CTRL 最多 512 B。 / Negotiate capabilities before using their fields. Size the DATA/LOG reassembly buffer for 2304 bytes and the control reply buffer for 512 bytes.

按 tag 解释旧 `gen/rid`，按名字解析扩展字段。`K.crc` 后可有身份字段。 / Legacy parsers need sufficient buffer capacity and must accept named trailer fields after CRC. When the identity extension is unavailable, preserve only the documented domain of each legacy field.

## 字段归属 / Immutable identity domains

| 报文/字段 | 所属域与来源 | 对应终态 |
| --- | --- | --- |
| C `gen/rid`，对应 K `gen/rid` | **ROWS**；`frame.configSnapshot.generation/requestId` | 行数 RAPP |
| V/R `gen/rid`，对应 K `gen/rid` | **MODE**；`frame.measurement.modeGeneration/modeRequestId` | MAPP；同质 ROWMODES 也应用 MODE |
| M `rgen/rrid`，对应 K | **ROWS**；`frame.configSnapshot` | 行数 RAPP |
| M `pgen/prid`，对应 K | **ROWMODES profile**；`frame.rowProfileGeneration/RequestId` | RMAPP；MODE 同步同质 profile |
| MR | 没有独立配置计数；继承本 M/K message 的身份，`s` 为实际 S 行，`m/unit/scale` 属于该行 | 同本 M |
| 带身份的 C/V/R/M header 的 `mgen/mrid` | 显式 MODE 域 | MAPP/CONFIG |
| 带身份的 C/V/R/M header 的 `rgen/rrid` | 显式 ROWS 域 | 行数 RAPP/CONFIG |
| 带身份的 C/V/R/M header 的 `pgen/prid` | 显式 profile 域 | RMAPP/CONFIG |
| 扩展 header 与 K 的 `session/bootId/rev` | 启动会话、旧启动计数、完整配置 revision | CONFIG、带相同身份的终态 |
| ROWS? `generation/appliedId` | ROWS；pending/requestId 是待处理行请求 | RCMD/RAPP |
| MODE?/STATE? `gen/rid/seq` | MODE；seq 是其应用边界，不是最新成功接收帧 | MAPP |
| ROWMODES? `gen/rid` | profile 域 | RMAPP |
| MAPP `id/gen/seq` | MODE 请求、成功 generation、允许新配置采样的边界 | 终态 |
| RMAPP `id/gen/seq` | profile 请求、成功 generation、当前 profile 的边界 | 终态 |
| 行数 RAPP `id/gen/seq/old/new` | ROWS 请求与边界 | 终态 |
| RAILCFG 的 RAPP `avdd/avss/source` | rail 校准事务；不是 ROWS | 不能用来推进 ROWS generation |
| RECOVER/RESTART 的 RAPP `cmd/level/state` | 恢复生命周期；不是 ROWS/MODE 成功 | 查询 CONFIG 重建 |

布局同质性检查完整的八项配置。`ROWMODES=CVRCVRCV` 与 `ROWS=1` 产生 `M(profile=CNNNNNNN)`、一个 CAP `MR` 及 `K`。`CONFIG.profile` 仍是 `CVRCVRCV`。

Homogeneity is evaluated over all eight entries. A single active CAP row from this mixed profile emits M/MR/K. Interpret the enclosing row and profile domains, even when all active cells are CAP.

采样 owner 在每次 engine 调用之前，从已选定的 mode、rows、profile 和 plan 捕获 `sensorarrayConfigIdentity_t`（协议扩展）。engine 对 frame 的清零不会使它丢失：调度器在返回后复制同一局部 snapshot，encoder 和 transport descriptor 仅解析/携带该 frame。发送阶段不得读取最新全局配置盖章。`rev` 在完整配置或安全状态改变时前进；正常逐帧不前进。profile 准备期间采样使用候选 identity，只有成功应用后发布相同 revision 的权威快照；失败帧无效，权威状态保留真实失败状态。

CAP 的 MODE 关联使用显式 `mgen/mrid`。 / The acquisition owner captures the mode, rows, profile and plan before the engine runs. The encoder and transport carry that exact snapshot through initialization, queuing and delivery. Match CAP to its mode request using the explicit mode pair.

## CONFIG? 原子查询 / Atomic query

`CONFIG?`（协议扩展）在短临界区内复制一个已发布的完整对象；它不执行硬件操作，不拼接 MODE?/ROWS?/ROWMODES? 的独立读数。未发布时返回 `ERR,cmd=CONFIG,reason=not_published`。查询字段包括：

- `session/bootId` 为 16 个十六进制数字表示的 64-bit 会话值及 RTC 启动计数。 / Preserve both the 64-bit random session and legacy boot counter.
- `rev/boundary` 为配置 revision 及其允许采样的首个 frame boundary。 / First boundary eligible for this published configuration.
- `state/mode/rows/profile` 是采集 owner 的状态，mixed 的 mode 为 MIXED，profile 总是八项。 / The authoritative profile retains all eight entries. M suppresses inactive rows with N.
- `mgen/mrid,rgen/rrid,pgen/prid` 是各域最近成功应用的 generation 和 request ID。 / Last successfully applied identity per mode, row-count and profile domain.
- `mtermId/mterm/merr,ptermId/pterm/perr,rtermId/rterm/rerr` 是各域最近终态的 ID、结果与错误。 / Per-domain last terminal. Outcomes are none, applied and rejected.
- `err` 是当前模式状态的错误。 / Current mode-state error. Use the corresponding terminal error for a request.

`CONFIG?` 提供采集 owner 一次发布的完整快照。失败请求 ID 与仍有效的已应用 ID 分开。 / Use one atomic CONFIG to reconstruct authority after a gap. Independent MODE?/STATE?, ROWS? and ROWMODES? reads can straddle a revision. If only individual queries are available, keep uncertainty explicit and retry when the observed identities disagree.

## 接受、应用与同步 / Acceptance, application and resynchronization

MODE、ROWS、ROWMODES 在所有 transport 共用配置 ID allocator 和单个 outstanding 配置事务。忙时在接受前拒绝，Host 可查询并稍后重试；不静默替代已接受请求。其它有界诊断命令仍可排队。MACK、RCMD、RMACK 是 accepted，绝不是 Applied。终态的 `identityKnown=0/rev=0` 表示当时无法取得完整快照，仍保留真实 session；此时重试 CONFIG?，不能将零 revision 当成功配置。

ACK 本地提交成功后，配置命令才对 Core 1 可见；ACK 提交失败则释放 reservation。Core 1 在完整帧边界应用。终态存入专用有界保留槽，优先于普通诊断发送；最后结果另存权威快照。这里的本地提交和保留不能证明 PC 收到，不承诺 BLE/UDP exactly-once。物理断连期间仍可能没有任何有效接收端。

同一 CAP 再次请求仍有新的 request ID 和成功 generation；不能无声忽略。MAPP 表示硬件/配置已应用，`boundary` 指定允许新配置采样的边界，**不保证首个有效数据已传到 Host**。RMAPP 接受完整新扫帧或已验证配置下的有界 CRC 部分帧；它不保证所有点有效。局部 CRC 重试耗尽仍输出 X02，valid/fresh/acquired 保持真实缺失，Host 应按点处理位图，不能因为配置已 applied 就将缺失点补零或沿用旧值。ROWS RAPP 只确认行配置，SAFE 中也可完成，不表示矩阵已恢复运行。

主机重建步骤 / Host resynchronization

1. 保存 accepted 请求的 session、域、ID 以及先前数据的原始身份。 / Record the accepted request with its session/domain and preserve the preceding data identity.
2. 新数据早于终态时，有界暂存，校验 CONFIG 的 session、revision、各域身份和边界。 / Buffer early data within a limit and verify the complete identity and boundary.
3. 终态丢失且 CAP 持续时，以 CONFIG 的 mrid、mtermId、结果与 CAP.mgen 恢复。C.gen/rid 仅验证 ROWS。 / Resolve missing MAPP by the mode authority while separately checking the row identity.
4. 旧终态迟到时按 session、请求域、ID 与 revision 关联。 / Correlate delayed terminals with their own requests and preserve the newer authority.
5. 比较两个 CAP 请求的 mrid/mgen。 / Reapplying CAP creates a new request and generation even if the mode string is identical.
6. 按原身份解释仍在队列内的老帧。如有按边界丢弃的策略，记录计数。 / Keep old queued frames labeled with their acquisition identity and account for any boundary-based drops.

## 启动、计数器与时间 / Sessions, counters and units

`RECOVER` 绑定受理时身份是否已知、session、boot、revision 和全部三域的 generation/request ID。有一项改变时 `RERR,reason=stale_config`，包括仅 revision 变化。 / The recovery owner validates the accepted identity before execution. A busy request has one local outcome. `RECOVERY?` reads the pending/last result without re-executing it.

`session` 每次启动随机生成；RTC bootId 遇断电可从 1 开始。 / A new 64-bit nonce identifies each boot with probabilistic uniqueness. Re-read CONFIG on reconnect. Scope sequence, generation, counters and revision comparisons to a verified session.

32 位配置 generation、revision、请求 ID 分配器回绕跳过 0。frame sequence 可回到 0。仅同会话同域、间距小于 2³¹ 时可作顺序判断。 / Match request IDs by exact equality. Use unsigned modular ordering only within a known half-range. Re-query when the interval is ambiguous.

`ts,dur,tr,transitionUs,*Us` 为微秒，`*Ms` 毫秒，`seq,frameCount,ageFrames` 为帧数。`4294967295` 在相应 age/lastGood 字段可为未知哨兵。 / Keep wall-clock durations and frame counts separate. Decode unknown values using the schema and validity flags.

## CRC 与无效点 / CRC and invalid cells

完整 ASCII frame CRC32 使用 reflected polynomial `0xEDB88320`、初值 `0xFFFFFFFF`、final xor `0xFFFFFFFF`。覆盖从 C/V/R/M header 首字节到 K 之前最后一个换行的所有字节，包括 header identity。K 自身不在该 CRC 内；同时检查 K 的 seq、legacy 域、session/bootId/rev 与 header 一致。BLE G envelope 的总长及 CRC32 是额外的重组校验，不能代替 ADC SPI 校验。

CAP pf6 是 pF × 10^6，既有无效 sentinel 和 masks 保留。VOLT fixed value 是 µV（scale=-6），RES fixed value 是 mΩ（scale=-3）；MR 各行独立 unit/scale。`Xhh` 是明确无效原因，不能填 0 或用于 PGA/cache。expected 是计划掩码，acquired 是实际已完成采集的掩码；健康检查在任何点转换前失败时 acquired=0，不等于执行了 64 次转换失败。

TX=SHORT/RT 的 B20 是 `previewOnly=1` 摘要，保留来自原队列 payload 的 session/revision/各域身份；无效端点为 na。B20 没有完整科学 frame 的 ASCII CRC，不可替代 C/D/K，也不能仅凭预览声称全部点已采集。

The legacy C and K gen/rid identify the row-count request. V/R identify the measurement-mode request. M/K separately carry row and profile identities, and MR inherits its enclosing frame. Explicit m/r/p fields identify mode, row count and profile across layouts. The acquisition snapshot supplies session, boot and revision through the encoder and queues. The 64-bit session scopes all comparisons. Sequence may wrap through zero; generation, revision and request allocators skip zero. Order a 32-bit counter only within its domain and a distance under half its range.

For a mode or layout change, track local acceptance, boundary-applied identity and the first valid complete data frame separately. On a lost or late terminal, query CONFIG and match the active domain IDs. An already captured frame retains its original identity after a later change. A repeated request selects a new request identity according to the arbiter. Re-read CONFIG around any compound legacy query and retry on revision or session change. Times ending in Us are microseconds and Ms are milliseconds. An unknown last-good age has no numerical duration.

## 测量和诊断 / Science and diagnostics

Raw ASCII CRC32 covers all bytes from the frame header through the LF preceding K. Verify K identity too. A G envelope has a separate CRC32. ADC SPI CRC8 protects conversion data. A corrupt reading cannot become a numeric cell. CAP is fixed-point pF × 10⁶, VOLT uses µV, RES uses mΩ, and MR rows each carry their own units. Planned, acquired, fresh and valid masks answer separate questions. SHORT/B20 carries a preview and has no full scientific frame CRC.

透明内部 rail 监测是否可用必须看 capability/硬件隔离。当前板型无已证实隔离时报告 unavailable，外部 DMM 校准是受限 VOLT 的合法路径；不要在 Host 自动发起未授权 INTREF 激励。rail span、AVDD、AVSS 分开解释，source 与 estimated/valid/age 分开显示；span 实测不等于两条对地轨均独立实测。外部校准不能因 matrix 帧龄而自动变为内部实测。rail/battery 的 valid=0 保留未知和真实 last-good age，不沿用旧值冒充本次有效值。

错误码必须按实际 BUILD 的 ESP-IDF 解释；ESP-IDF 5.5.5 `ESP_ERR_INVALID_RESPONSE=0x108`，`ESP_ERR_TIMEOUT=0x107`，不得把所有 0x108 称为超时。ADSFAULT 的 originalError 与 cleanupError 分开，阶段指向首次比较/读取失败；SAFE_REQUESTED 或 GPIO 回读不代表模拟节点已验证安全。故障多行按 bootId + seq 关联，不能把截断行补为原始数据：

| 报文 | 字段 / Fields |
| --- | --- |
| ADSFAULT | 首次 stage 及收尾前 mode/route；未知 bootId 明确 unknown |
| ADSFAULTRAIL | railSpanUv、avddUv、avssUv、source、valid、ref |
| ADSFAULTCTX | originalError、cleanupError、engineFrameCount、modeEntryFrameCount、healthInterval、healthReadbackDue、precisionFrame、attempted、acquired |
| ADSFAULTREG | reg、expected、actual、mask、cacheGen、physicalGen、configGen |
| ADSFAULTTXN | kind=rail/battery/ADSCHK、id、startUs、completeUs、firstValidFrameUs、originalError、cleanupError |
| ABAT | 旧 UINT_MAX sentinel 保留；lastGoodKnown，unknown 不是实测年龄 |
| ARL | split=unknown/estimated/external_measured、aincom=estimated、uncertaintyUv=unknown、calUs |

Interpret ADSFAULT context before SAFE cleanup, with separate original and cleanup errors. Register snapshots record masks, expected/actual and generation; auxiliary transactions record start, completion and first subsequent valid frame time. RAIL reports source and uncertainty; external dual-rail measurements, internally estimated splits and unknown last-good times are distinct. Keep battery availability and verified restore status separate. VOLT needs a valid reference split. This board has no qualified transparent internal rail route.

## 通路与能力查询 / Transport and diagnostic extensions

| 接口 / Query | 含义 / Interpretation |
|---|---|
| USB? / USBQ? | CDC0 为正式记录，CDC1 为调试。按数据/优先记录准入及过期；FIFO、DMA 完成或 SOF 只证明对应本地阶段。 / Match the advertised USB role. Distinguish sink acceptance, driver progress and complete host receipt. Queue depth and retention are reported at runtime. |
| BLE / BTX / protected full | FF11 CTRL、FF30 生命周期优先，然后 protected full、普通 DATA、LOG。配置切换后的首个有效完整帧和周期完整帧要由接收端按原始数据验证。 / Protect control and complete snapshots under pressure. Reassemble raw packets or G fragments at the actual MTU. Check individual receiver gaps; ordinary CAP previews do not satisfy a periodic complete snapshot. |
| SNAPSRC? / SNAPNET? / BLEGUARD? / BLELOG? | 按 PROTO 能力页查询各阶段计数、最近保护身份、SDK 调用及普通 LOG 准入。 / Local protection, congestion, confirmation and watchdog counters explain scheduling. Receipt is measured by the peer. |
| WIFI / WIFIREQ / WTERM | AP 请求在独立 SDK owner 上执行。用 exact ID 查缓存终态。OFF 关闭输出 gate；区分 local publish、SDK 返回和 UDP 接收。 / Request IDs and actual return evidence scope the AP state. OFF reports the gate. An absent reply can be recovered by query while retained. |
| SPI? / SPISOURCE? | 区分请求/实际速率、测试分母、选择、读数来源和所用时钟。运行时电源恢复重新探测三前端并扫描。 / Inspect requested versus achieved clocks, sample counts and read provenance. Slow presence probes and requalification run on frontend return. |
| ADSCHK / ADSTIMING / ADSTERM | 有界时序/末位诊断按请求号和配置身份执行。区分短接输入、锁存重复读、正式转换及末位 GPIO 采样。 / Separate fresh conversions from repeated latch reads and separate hardware SPI bits from the sampled GPIO bit. Preserve masks, status and restore results. |
| MEM / MEMFAIL / PANIC / CPUOWNER | 以启动为域记录分配、空间、水位及保留的异常信息；各角色 CPU 时间按采样窗口计算。 / Use validity flags, exact ELF and matching boot. Heap capability classes, largest blocks, stack minima and owner runtime have different denominators. |
| RECOVERY? / RESTART? / USBBOOT | 查询不会重复执行；重启意图与新会话成功上线分别确认。 / Read the cached result and independently verify re-enumeration and the new session. |
| CAL / baseline | 持久化操作显式请求；独立记录校准来源、应用身份及有效范围。 / Keep external calibration, runtime estimates and saved calibration separate. |

<a id="usb-roles-v1"></a>

### 有线角色、USBQ 与下载 / Wired roles, USBQ and ROM handoff

| 能力 / Capability | 端点与命令 / Endpoints and commands |
| --- | --- |
| `usb_cdc_data_v1` | 应用开发号 `303A:4042`。CDC0（IF0/1）输出正式 DATA 与 CTRL，并接收命令。CDC1（IF2/3）单独输出调试 LOG。 / Open formal CDC0 for ordered science and control. CDC1 is debug output. Discover interfaces after each enumeration. |
| `usb_vendor_bulk_v1` | 可选开发号 `303A:4041`，vendor DATA 和 CTRL 有独立端点，另有一个调试 CDC。 / Use the vendor endpoints according to their descriptor roles. |
| `serial_sink_v1/serial_classes_v1` | 旧 Serial/JTAG 的 RX 和关键终态始终独立于 DEBUG/FULL 抽取。 / Legacy USJ is a separate backend with its own serial counters and publication settings. |

`USB?` 的 `backend,init,err,mounted,suspended,dma,epoch,resets,mounts,unmounts` 指控制器状态。`rxBytes,rxDrop,rxOver,lastRxUs,lastTxUs` 是本机 RX/TX 观测，`dataDtr,debugDtr` 为对应端口的 line state，`pcReceipt=unknown`。总线 reset/unplug 改变 epoch。 / A bus reset or unplug invalidates old queued records. A DTR transition alone keeps the connection epoch. Match host interface roles and a current `BUILD?/CONFIG?` before resuming a session.

| `USBQ?` 字段 / Field | 类型与含义 / Type and interpretation |
| --- | --- |
| `dDrop,cDrop` | DATA、priority 本地准入拒绝，单位条。 / Counts whole records rejected at local admission. |
| `dSubmit,cSubmit` | 各类记录的全部字节已经本地驱动接纳，单位条。 / Complete local record submission, distinct from queue admission and PC receipt. |
| `expired,cExpired,partial` | normal（DATA 与 LOG）和 priority 过期/过 epoch 回收。partial 为残缺写及异常前缀。 / Normal and priority expiry/stale retirement, plus partial-write observations. |
| `dBytes,cBytes,dCallbackBytes,cCallbackBytes,callbackResult` | 前两者为 TinyUSB 每类 FIFO 实际接收 B。双 CDC 回调无法归属 DATA/CTRL 字节，为 unknown。vendor 有端点回调字节，结果仍 unknown。 / Compare bytes and records within their own observation layer. |
| `dQueueCap,dLifetimeUs,queueMaxUs,stackMinBytes` | 运行中普通 DATA 队列深度与绝对寿命、所见最大排队时间、最小剩余 TX 栈 B。 / Read effective capacity and original-deadline retention from the current boot. |

正式 CDC 可使用 PSRAM 的 100 个排队位置及 101 个持久 payload 槽（含正在服务的一条），每条至多 2304 B，入队寿命 1,000,000 µs。小池、vendor 和旧 USJ 使用 4 个排队位置、5 槽、250,000 µs。普通 LOG 独立 4/5、250,000 µs，优先通路有 4 个不超过 512 B 的槽、1,000,000 µs。 / One wired TX owner services the persistent records. A zero-byte write keeps the original deadline and acquisition identity. Selection happens between complete formal records. TinyUSB copies from CPU storage into its own FIFO and endpoint DMA buffers.

CDC0 的正式 DATA 仅在单个 FIFO 可容纳整条记录时提交。未读或空间不足时返回零并按原期限保留。异常正数短写视为残缺记录，释放后插入换行边界。CTRL、debug CDC 及 vendor 可有有界部分写。 / An unread formal interface can exhaust its bounded queue. Other sinks and command RX run independently. On the shared formal endpoint an already-started record holds the stream, including across retries, to prevent CTRL bytes from splitting a science record. Host-validated K/CRC and sequence remain the end-to-end measurement.

OTG 的 `SERIAL?` 返回 `backend=otg_cdc,scope=debug,logRejected,logSubmitted,lineDropped,lineOversize,discardBytes,driverAcceptedBytes,rxCommands=unavailable,pcReceipt=unknown`。`USBSTREAM?` 返回 `v=CDC` 或 `BULK,scope=otg_data,dataEvery=1,debug=cdc,control=shared_cdc` 或 `bulk_ctrl`。DEBUG/FULL setter 在 OTG 上返回 `legacy_usj_only`。 / OTG always makes eligible complete science records available on its formal interface. Legacy DEBUG/FULL remains a separate USJ text publication setting.

`USBBOOT` 经同一 RESTART 管理器申请 `target=rom_download`。ROM 能力看 `USB?.romAvailable`，静止状态看 `quiesced`。`RESTART?` 的 `pendingId,lastId,target,terminalQueued,serialQueued,netQueued,publishError` 区分意图、排队和发布失败。 / After an intentional download request, confirm ROM enumeration on the host. Following application restart, obtain a new BUILD/CONFIG and check READY. Local ACK or quiescence alone cannot establish ROM enumeration or PC receipt. [USB 控制实现 / Control](../../main/control/sensorarrayUsbControl.c) · [有线发送实现 / TX owner](../../core/transport/sensorarraySerialIo.c)

<a id="wifi-owner-contract-v1"></a>

### Wi-Fi owner 与请求页 / Wi-Fi owner and request pages

先在 `PROTO?.caps` 查 `caps2_v1`，然后发送 `CAPS2?`，当前回复为 `CAPS2,v=1,page=1,caps=wifi_owner_v1,next=none`。这是能力分页，不是ready证明；旧固件没有该cap时不推断新异步语义。跟随 `next` 指示读取下一页，`next=none` 表示结束。

First require `caps2_v1` in PROTO, then query CAPS2 for `wifi_owner_v1`. The current reply above advertises capability, not readiness. Do not infer the new asynchronous contract on older firmware without the capability. Follow the advertised next-page indicator until `next=none`.

| 命令 / Command | 语义 / Meaning |
| --- | --- |
| `WIFI?` | `ACK,cmd=WIFI,...,ver=2`，一次owner snapshot；requested、outputGate、backend、resources及request分开。 / One owner snapshot with separate requested mode, output gate, backend, resources and request state. |
| `WIFI=AP` | accepted1不等于applied；未完成ok0/pending1。任务可能先完成，ACK可携带该确切已缓存终态。相同ready配置产生新id的幂等applied、不重复SDK调用。busy/quarantine/bootstrap失败为accepted0/id0和真实err。 / Acceptance is not application; completion may race ACK formatting. Same-ready requests are idempotently applied under a new ID; rejection has no accepted ID. |
| `WIFI=OFF` | 新id/epoch，立即应用软件输出gate；不stop/close/deinit、不删除任务、不清quarantine或持有中的SDK对象。scope为output_gate_only，physicalOff始终unknown。 / Immediate software-gate result under a new ID/epoch; not radio shutdown, cancellation or quarantine recovery. |
| `WIFIREQ?` | 当前最后请求，无副作用。 / Read the latest request without side effects. |
| `WIFIREQ?<id>` | 精确非零十进制u32；例如`WIFIREQ?17`，显式0无效。旧结果被淘汰时known0/outcomeunknown/reasonnot_cached，不代入另一个请求。 / Query an exact nonzero decimal u32 ID; an evicted result is unknown rather than a substituted request. |
| `WIFIGUARD?` | SDK调用/隔离的缓存证据，不进入SDK、不测radio/bus。 / Cached SDK-call/quarantine evidence, without querying the SDK or physical radio/bus. |
| `WIFIQ?D`, `WIFIQ?L`, `WIFIQ?C` | 三条精确只读命令，分别返回DATA/LOG/CTRL的boot域计数。 / Three exact read-only per-channel queries with boot-scoped counters. |

`WIFIREQ`及`WTERM`字段：`v=1,session,boot,bootKnown,id,known,epoch,requested,outcome,err,cleanupErr,acceptedUs,completedUs,terminalAttempts,terminalQueued,terminalPending,scope,peerReceipt=unknown`。outcome为`none/pending/running/applied/failed/superseded/quarantined`；unknown精确查询另标unknown/not_cached。一个active加两个terminal缓存，terminal只commit一次；OFF可supersede活动AP，不能改写旧failed/quarantined/applied结果。pending/running的completedUs为unknown。AP applied要求对应epoch的必要SDK调用、socket和RX task实际创建成功，scope=backend_and_gate；不证明已有station或PC。WTERM走本地Serial优先＋可观察network lifecycle，最多3次、相隔200ms，可早于或晚于ACK；terminalQueued只证明本地接纳。精确查询优先于重发setter。

| WIFIGUARD字段 / Field group | 定义 / Definition |
| --- | --- |
| `v=1,session,boot,bootKnown` | boot身份，unknown不填0。 / Boot identity; unknown is not fabricated zero. |
| `quarantined,active,ownerBusy,phase,callId` | active指隔离owner的SDK调用，不是radio状态或RX等待；phase是最近执行阶段。 / Active describes the owner's SDK call, not radio state or RX waiting; phase names the execution stage. |
| `startUs,deadlineUs,ageUs,detectedUs,pollUs=100000` | 单调µs。无调用的start/deadline/age及未隔离的detected为unknown；100ms观察周期不是硬实时保证。 / Monotonic microseconds; absent call/quarantine times remain unknown. Poll period is not a hard-real-time guarantee. |
| `completionKnown,err,cleanupErr,sdkErr,errno,returned` | 首个事务/guard错误独立于cleanup；sdkErr/errno/returned来自真实SDK返回。隔离后completionKnown保留该调用，晚返回只补证据，不清隔离。 / First error and cleanup are separate; actual-return evidence is distinct from timeout. Late completion updates evidence without clearing quarantine. |
| `held,writing,timeouts,late,recovery=explicit_manual_restart_attempt` | held包含WRITING/PENDING/OWNED；未知owner保留。显式重启只是恢复尝试。 / Held includes WRITING/PENDING/OWNED; unknown ownership is retained. Explicit restart is a recovery attempt. |

phase覆盖`idle/netif_init/event_loop/netif_new/netif_attach/handlers/driver_init/event_register/set_mode/set_config/start/set_power/set_ps/set_protocol/set_bandwidth/socket/setsockopt/bind/send/station_query/close/event_unregister/stop/deinit/clear_handlers/netif_destroy/rx_task_create`；这是软件调用阶段，不是物理安全状态。

`WIFIQ`字段：`v=1,session,boot,channel,scope=boot,queued,rejected,sent,dropped,expired,stale,partial,driverAcceptedBytes,terminalNoSink,terminalEvicted,stackKnown,stackMinBytes,stackScope=owner_after_job,sentScope=full_sendto_return,peerReceipt=unknown`。此回复没有bootKnown字段。queued表示深拷贝入owner队列；rejected是准入失败；sent及旧Transport的wifiDataSent/wifiLogSent/wifiCtrlTx仅在实际完整sendto返回后增加，晚返回记一次。不把ESP_OK入队当sent，不把完整返回当PC收到。dropped含expired/stale/partial/SDK失败，子计数重叠不能相加；在途timeout尚不证明datagram丢失。driverAcceptedBytes为u64，其他计数u32模回绕；仅同boot、有界间隔计算delta。terminalNoSink/terminalEvicted是终态保留/提交边界。stackMinBytes是owner在自身工作真实返回后缓存的最小剩余栈字节，未采样stackKnown0/unknown；不是RX或全系统栈，也不扫描被阻塞的其他任务。

资源与期限：默认OFF不创建任务/timer/payload；真实boot身份和两个启动资源预留参与者完成后才开启动gate。owner永久7,424 B PSRAM\|8BIT（2×2304普通、1×2304 lifecycle、1×512 CTRL），无internal fallback；owner/RX 栈分别6144/4096 B，执行/初始化工作区为owner生命周期，metadata/锁/栈仍内部。copy/SDK/callback在锁外，WRITING/OWNED不可覆盖。AP执行5s、cleanup最多1s、总6s；send500ms；100ms观察只置隔离，不调用SDK/free/reset。Wi-Fi队列1s send-start TTL取原protected deadline较早值，不续期，**不保证优先级洪流或隔离时1s内回收pending**。Transport保护descriptor的1s逐dispatch到期清理是另一层。原始err不被cleanup覆盖；driver_init返回错误可能隐藏内部deinit失败，此时保留对象隔离；不重建部分存活资源，不自动HIGH→BLE_COMPAT重试，不把timeout当取消。

Resources and deadlines are local isolation policies. Default OFF allocates no owner task/timer/payload until boot identity and both startup reservations are established. Persistent PSRAM-only storage totals7,424 bytes, with 6144/4096-byte owner/RX stacks, owner-lifetime execution/init workspace and internal metadata/locks/stacks. Copies, SDK calls and callbacks run outside the owner lock. AP execution/cleanup/total limits are5/1/6 seconds; send is500ms and observation100ms. The one-second Wi-Fi send-start TTL preserves any earlier protected deadline and forbids starting stale work; it does not promise reclaiming pending slots during priority starvation or quarantine. Transport expiry is a different layer. Unknown ownership and partial initialization remain retained; neither timeout nor an initialization error proves safe cancellation/deinitialization.

<a id="memory-failure-v1"></a>

### 分配失败页 / Allocation pages

`MEMFAIL?` 在早期及正常控制路径可查本次注册后的分配失败。回调以内部静态内存保留首末记录，锁忙时标遗漏，重启后重新统计。 / This read-only query preserves boot-scoped first/last allocator-hook evidence, including missed or incomplete records. An unsupported query leaves failure counts unknown.

| 字段 / Fields | 解释 / Meaning |
| --- | --- |
| `v,scope,session,bootId,identityKnown,elfSha256` | `scope=boot`；使用同session/boot/full ELF比较，不拼接跨重启值。 / Compare only within the same full image and boot identity. |
| `attempted,regKnown,registered,regErr` | 尝试注册、注册结果是否已知、是否成功及原错误；不能把未注册解释为未失败。 / Registration status is independent of whether failures were observed. |
| `known,firstSize,firstCaps,firstFn,lastSize,lastCaps,lastFn` | known0的各记录为unknown；known1提供首末SDK callback参数。Fn是opaque API-string地址，必须按同ELF离线解码，**不是caller PC**。 / Known records preserve callback arguments; function pointers identify allocator API strings, not the caller. |
| `count,saturated,incomplete` | recorded callback计数；达到u32上限置saturated，锁忙漏记置incomplete。count0不证明注册前或未捕获范围无失败。 / Saturation and incomplete capture remain explicit. |
| `fn,countScope,coverage` | 固定`opaque_pointer,recorded,since_register`；不是全部尝试次数或完整heap trace。 / This is a bounded observation, not a complete allocation ledger. |

Preference allocators call the hook only on ultimate failure and report the last attempted capabilities. Pinned calloc_prefer reports size rather than n×size; direct calloc reports the product. Sizes exclude alignment/poisoning overhead. Separate internal8 memory snapshots are not instantaneous DMA statistics, and aggregate free memory does not guarantee a contiguous block with the required capabilities.

该分配 hook 只观察已注册后收到的最终失败。首末记录中的地址按相同 ELF 解读；free heap 总量不能替代指定能力的最大连续块。 / Preserve capability and allocation-size context.
<a id="panic-diagnostics-v1"></a>

### 保留异常页 / Retained panic pages

| 查询 / Query | 字段与解释 / Fields and meaning |
| --- | --- |
| 全部 / All | `PANIC,v=1,page,session,boot,known,record`。session/boot是**当前查询boot**。无有效记录为`known=0,record=unknown,reason=no_valid_retained_panic`，不代表没有发生panic。 / Current query identity; unknown retained evidence is not evidence that no panic occurred. |
| `PANIC?`，page0 | `originKnown,originSession,originBoot,elf`属于**原panic发生域**；elf为完整64hex。`reset`是当前boot的IDF reset enum，不能当原panic原因。`record`为该记录CRC32的8hex身份，检测破坏而非认证/全局唯一序号。 / Origin identity/ELF is distinct from current reset; record CRC detects corruption, not authenticity or global uniqueness. |
| page0调用信息 / Info | `infoKnown,core,exception,abort,pseudo,addr,reasonPtr,detailPtr`。指针只是原始地址，不在Host解引用；infoKnown约束core/exception/pseudo/addr/reasonPtr，abort独立记录SDK abort标志，detailPtr是abort详情原始指针。 / Info-known gates the info-derived fields; abort independently records the SDK flag, while detailPtr is a raw abort-detail pointer. No pointer is host-dereferenceable. |
| page0异常帧 / Frame | `frameKnown,pc,ra,sp,cause,vaddr`，寄存器为无0x前缀的8hex；frameKnown1才使用，ra保留Xtensa ABI位，不把它直接当规范化return PC。 / Registers require frameKnown; the raw Xtensa return address retains ABI bits. |
| page0详情 / Detail | `detailKnown,detailLen,truncated`；只复制可限定在internal DRAM内的abort详情，最多192字节，不读取flash/PSRAM字符串或遍历栈。 / At most192 bounded internal-DRAM abort-detail bytes; no flash/PSRAM string read or stack walk. |
| `PANIC?1`、`PANIC?2` | `detailKnown,offset,length,total,truncated,encoding=hex,text`；offset分别0/96、每页最多96原始字节。text是这些字节的hex，不自动推断编码/完整文本；detailKnown0时text=unknown，合法空页可length0。 / Hex pages preserve bounded raw bytes; unknown and an empty valid page remain distinct. |

Host先取page0，再取page1/2并重查page0；所有页必须同current session/boot/record，且解析符号使用**origin elf**对应的完整ELF。record CRC可能相同，不能单独当跨boot请求ID或因果依据；originKnown0/infoKnown0/frameKnown0/detailKnown0各自撤销对应解释，不把编码零值当已知。最后保留panic可早于当前reset；不能仅因当前reset=panic或record存在就把它归给最近一次WIFI=AP。

Read metadata, detail pages and metadata again; require matching current session/boot/record throughout and symbolize against the full **origin ELF**. CRC equality alone is not a cross-boot identifier or causal link. Each known flag independently gates interpretation. A retained panic may predate the current reset, so neither reset=panic nor record existence by itself attributes it to the latest AP request.

实现先在正常启动、command RX安装前复制不可变查询snapshot并保存当前origin；panic wrapper只做有界RTC写入、范围检查、CRC与最后commit，然后调用原SDK handler，保留本boot首次capture，不加锁/堆分配/RTOS/日志/外设操作。poweron/brownout/unknown reset令保留记录无效；magic/version/size/CRC等不合格也是unknown。它依赖当前ESP32-S3/IDF5.5.5异常帧，不能宣称整个SDK panic路径都在IRAM或任意硬件损坏均能保留。无watchdog、reset、安全或科学帧语义变更。真实格式器最大含NUL为473/349/350 B，容量512 B；短buffer明确失败，不发布截断记录。

Startup copies an immutable query snapshot before command RX and records the running origin. The wrapper performs bounded RTC capture/checksum/commit, then forwards to the original SDK handler; the first capture in a boot is retained without locks, allocation, RTOS, logging or peripheral work. Power-on/brownout/unknown resets or invalid record checks yield unknown. This is pinned to the current S3/IDF exception frame, not an all-IRAM SDK or arbitrary-fault retention guarantee. Watchdog/reset/safety/scientific-frame behavior is unchanged. Actual formatter maxima including NUL are473/349/350 bytes within512; short buffers fail explicitly.

<a id="protected-snapshot-v1"></a>

### 保护快照页 / Snapshot pages

`ble_flow_v1` 协商 `BLEFLOW?`；`ble_snapshot_v1` 协商 `BLESNAP?`、`SNAPSRC?`、`SNAPNET?`。四者均 `v=1`。 / Negotiate these two capabilities through PROTO. An explicit unsupported response is distinguishable from an unanswered query. Full science frames retain the C/V/R/M/MR/K format.

`gen/stream` 是 BLE connection/effective-DATA-stream 域，与 mode/rows/profile generation 不同。取消有效订阅、重订阅或 FAST/SAFE 有效资格变化会隔离旧 stream；即使连接没断也不能把旧完成用于新义务。完整源帧在采集开始捕获不可变 session/boot/revision/mgen/mrid/rgen/rrid/pgen/prid/sequence；各层只携带此身份，不在发送时查询最新配置。CAP 的旧 gen/rid 仍来自 rows 域，V/R 来自 mode 域，M 的 rgen/pgen 保持原定义。跨 boot/session 不比较大小；同 boot 的有限 sequence 用半范围规则，generation 只在其域内验证。恢复配置用原子 `CONFIG?` 和原 applied boundary，不以 MACK 或某一个 gen 数值替代。

source/Transport 查询本身没有完整 session 字段；应与 BUILD/CONFIG 身份、主机查询时间一起保存。SNAPNET 两份本地统计分开读取，四个查询也不是跨层原子快照。比较增量时核验同 boot/session、同连接/stream、同配置与窗口；查询期间身份变化则重取，不拼接不同实例。boot 总计按各字段 uint32/uint64 无符号模差处理，超过半范围或身份缺失视为不确定。scope/counters=boot 不表示连接重置时计数全部归零。

Source/Transport replies do not themselves carry complete session identity; retain BUILD/CONFIG identity and host query timestamps with them. SNAPNET reads two local snapshots sequentially, and the four queries are not one cross-layer atomic snapshot. Validate boot/session, connection/stream, configuration and observation windows before comparing deltas; retry if identities change. Use unsigned uint32/uint64 modular deltas within a verified boot, treating ambiguous range/identity as unknown. Boot scope does not promise all counters reset on reconnect.

正常 full 策略选择全部普通候选。连续三个有效 1–2 秒窗口容量拒绝率严格超过 20% 后，以至少 3 秒间隔每次下调 50‰。累计 10 秒有效干净证据提高 50‰，证据有效期 3 秒，受限时每秒至多探测一份真实普通帧。默认完整快照周期 1 秒，同身份的较新普通完整帧本地成功完成也能满足。 / Under pressure the BLE sink reduces ordinary output independently of acquisition. A 2.5% loss budget does not cause deliberate dropping. On expiry, protect the next real encoded full frame with its original invalid masks and Xhh. A SHORT preview does not satisfy that obligation. Check actual host delivery independently.

<a id="ble-guard-v1"></a>

### BLE SDK 观察页 / SDK guard pages

通过独立只读 `BLEGUARD?` 查询、收到完整 `BLEGUARD,v=1,scope=send_observer,...` 回复后确认支持；不扩展现有PROTO capability列表或改科学帧版本。旧固件可返回不支持，超时/丢回复不能直接等同不支持；隔离时优先通过USB查询，BLE回复本身可能不可发送。只读snapshot不解除隔离、不取消SDK、不触发重启。

Probe support with read-only `BLEGUARD?` and a complete `BLEGUARD,v=1,scope=send_observer,...` response. The existing PROTO capability list and scientific-frame version are unchanged. Older firmware may report unsupported; timeout or a missing reply alone does not prove lack of support. Prefer USB queries when quarantined because the BLE reply itself may be unsendable. Reading the snapshot does not clear quarantine, cancel the SDK or trigger restart.

After quarantine, callId/startUs/detectedUs/completionKnown describe the frozen fault call rather than a new connection’s call. A late return still records the actual SDK outcome/counters and may set completionKnown=1, but quarantine remains. While return is unknown, the slot, buffer and task stay owned. Neither the outer timeout nor this guard cancels an indefinite BTC queue wait inside the SDK; the host must not infer withdrawal from timeout. Idle, legitimate pause and CONF waiting are not active-call timeouts. Observer scheduling can be delayed; 500+100 ms is not a hard-real-time bound. / 隔离后保留故障 callId、原始槽和所有权。迟到返回可令 completionKnown=1，隔离状态仍保留。

`BLEGUARD.rejected` 仅计进入 BLE guard 的拒绝。通用 transport 无有效网络 sink 时可直接返回。 / A generic transport success can occur before BLE admission. Compare BLE submission, rejection and host sequence evidence separately.

<a id="spi-probe-v1"></a>

### 启动选速与恢复 / Rate qualification and return

- `SPI?`：在 READY 前也可读取同一个已发布启动结果。成功需要
  `done=1,joined=1,timedOut=0,qualified=1,restored=1,err=0`，不能单看 qualified。
  `hz` 是请求频率，`actualHz` 为 IDF 观测频率（kHz 分辨率换算为 Hz），不是实测
  SCLK 波形；`overlapUs` 是 SPI 工作区间和 I²C 初始化区间的交集。`attempted/samples/crc`
  的 `scope=all_candidates` 包含被淘汰档位，不能据此计算选中档的错误率。
- `SPI_SWEEP`：每档分别记录 screen/confirm、pga1/bypass。默认每个 profile
  200/800 次 wire attempts，最多 5/20 次 CRC 拒绝；坏样本不计 valid、奇偶覆盖或
  rawMin/rawMax。2.5% 是有限启动样本的资格门限，不是长期错误概率保证。
- `FRONTENDS?`：原子探测快照，不触发总线操作。`known/present` 是三位掩码：
  bit0=primary FDC，bit1=secondary FDC，bit2=ADS。`unknown/busy/inhibited` 不是
  物理失联证据；`sdInhibited=1` 也可能是 SD 未知，不能解释为实测高电平。
  `aHz/aRestoreHz` 区分低速探测时钟和恢复后的时钟；原始错误与各 cleanup 分开。
  ADS 每轮以 125 kHz 探测后恢复选定频率；FDC 正常时使用现有频率，SUSPECT/WAIT
  才临时降到不高于 100 kHz。`lowSpeed` 描述此等待期策略，不表示正常轮的 ADS 没有临时降速。
- 首次同轮三芯片 unavailable、已初始化 ADS 的 RESET，或探测恢复失败会进入
  SUSPECT 并停止采集。3 轮连续全失联确认 WAIT；两轮清理成功、ID 稳定且全 present
  后请求受预算约束的 ESP 重启。启动未完成也可进入相同等待，不伪造“三芯片断电”证据。
  正常/SUSPECT 每 1 秒探测，WAIT 每 2 秒探测；每轮有独立有界事务，等待期间可长期
  保持合法 idle，不能由健康任务伪造采集进度。FDC SD 主动关闭不当作全失联。
- hold 期间 MODE/ROWS/ROWMODES 和测量辅助操作被拒绝，已接受的请求有拒绝终态；
  控制接收、查询和显式 RESTART 仍有效。恢复采用整机重启重新调用独立 Startup 模块，
  不复用旧 worker/DMA/DRDY/cache。自动重启预算耗尽后保持可查询等待，需要显式处置。

SPI?/FRONTENDS? 是诊断快照，不是新的帧身份权威；接纳数据仍以 CONFIG 的
session/revision/域计数为准。重启后先重新查询 PROTO/BUILD/CONFIG，再恢复上位机配置。
临时 RAILCFG 随 RAM 丢失，不能自动把旧电源周期的轨校准套到新会话。等待的路由为
UNKNOWN，`physicalSafe=unknown`；GPIO 回读和 ID 恢复均不证明模拟隔离或供电合规。

Read SPI? and FRONTENDS? without bus I/O, including before READY. The boot result requires done/joined/qualified/restored, no timeout and zero error. Per-profile screen and confirm count wire attempts. known/present mask bits are primary FDC, secondary FDC and ADS. Three consecutive all-unavailable rounds confirm WAIT. Two clean stable-ID all-present rounds request a budgeted restart. Reconnect and renegotiate PROTO/BUILD/CONFIG, including volatile external rail calibration. Unknown digital or physical state remains unknown.

<a id="ads-timing-v1"></a>

### ADS 时序与数据来源 / Timing and provenance

`ADSCHK=TIMING,200` 与 `ADSCHK=TAIL,200` 使用共用配置请求 ID 和单事务仲裁，保留受理的 session/boot/全部域身份。仅已知 CAP/VOLT、前端空闲、INTREF 已关闭、无 START、寄存器及 shadow 与所需状态相符时在完整帧边界执行。 / Both actions capture the accepted identity. After local ACK admission, the acquisition owner verifies route, quiescence, VBIAS, CRC/STATUS, AVDD/AVSS reference and hardware/shadow registers. A stale or unknown state is rejected. Neither diagnostic enables the internal reference.

ACK 格式为 `ACK,cmd=ADSCHK,timing=1,id=…,perGroup=200,groups=4,status=accepted`。它表示本地接受，不是执行完成。未执行请求在边界检查中有 10 秒排队预算；已接受的硬件事务使用同一 2 秒绝对期限，其中保留 500 ms 用于恢复。四组之间、恢复和验证期间不穿插新帧、其他 ADS helper 或配置命令。PULSE 下额外的 `held_repeat` 是重复读取同一锁存结果，始终 `fresh=0`，不计入 800 次主读分母；不得把它当作新鲜采样。超时不等于 DMA 已取消，现有隔离和有界恢复规则仍适用。

The ACK is `ACK,cmd=ADSCHK,timing=1,id=…,perGroup=200,groups=4,status=accepted`, indicating local acceptance rather than completion. Boundary checks enforce a ten-second pending-request budget. The hardware transaction has one two-second absolute deadline including a 500 ms restoration reserve. Frames, other ADS helpers and configuration commands cannot interleave with the groups or restoration. Additional PULSE `held_repeat` reads revisit the same latch, always have `fresh=0`, and are excluded from the 800-primary-read denominator. A timeout does not cancel DMA; existing quarantine and bounded recovery rules remain in force.

终态先写本地最后结果缓存，再通过既有优先控制终态通路发布 `ADSTERM,id=…,state=completed|failed|rejected,reason=…,session=…,bootId=…,acceptedRev=…,rev=…,boundary=…,originalError=…,cleanupError=…,result=0|1,diagnostic=1,query=ADSTIMING`。首错与 cleanup 分开保留。若已知坏 CRC 被拒绝而其余事务完成且恢复验证通过，`state=completed` 仍可带非零 originalError；它不表示所有读数有效。需要隔离的 probe fault 与恢复失败分为 `probe_fault_requires_isolation` / `restore_failed`，并停采、发布 DEGRADED。`MFAULT,source=ADSTIMING` 同时给出 probeReason、restoreVerified 和 cleanupError，不能仅按 requiresIsolation 推定根因是恢复失败。

The immutable last result is cached before priority terminal publication: `ADSTERM` contains the request ID, completed/failed/rejected outcome, reason, session/boot, accepted/current revision, boundary, originalError, cleanupError and result availability. A completed transaction may retain a CRC first error when corrupt reads were rejected and restoration verified; it does not mark every read valid. `probe_fault_requires_isolation` distinguishes a probe fault requiring isolation from `restore_failed`. Either stops matrix acquisition and publishes DEGRADED. The accompanying ADSTIMING MFAULT carries probeReason, restoreVerified and cleanupError; requiresIsolation alone does not establish a restore root cause.

| 无副作用查询 / Read-only query | 内容 / Response |
| --- | --- |
| `ADSTIMING?` | session/boot、pendingId/phase/ackLocal、lastId/outcome、acceptedRev/current rev/boundary、首错/cleanup、result 和 terminalQueued；原子元数据快照 / atomic metadata snapshot |
| `ADSTIMING?RESULT,<id>` | `ADSTIMING,diagnostic=1,science=0,…`：整事务起止 µs、实际/请求 SCLK、配置 generation 与 CRC 计数端点、restoreKnown/Verified、records / transaction summary |
| `ADSTIMING?GROUP,<id>,<0..3>` | `ADSTIMINGGROUP`：主读 attempted/valid/CRC/parity/xor方向/status/freshness/timeout；held 独立计数 / separate primary and held counters |
| `ADSTIMING?TIME,<id>,<0..3>` | `ADSTIMINGTIME`：组起止 µs、generation、保留/省略记录数与 held 细分 / group timing and retained/omitted record counts |
| `ADSTIMING?REG,<id>,<0\|1>` | `ADSTIMINGREG`：0=saved、1=restored；known 与 ID/POWER/INTERFACE/MODE0/1/2/INPMUX/REFMUX/OCAL/FSCAL / register evidence |
| `ADSTIMING?RECORD,<id>,<index>` | `ADSTIMINGRAW`：平面索引 0…records−1，最多 32 条；raw/status/CRC/fresh、时间、配置/DRDY generation 与有界原始字节 / bounded raw diagnostic records |

详情只接受当前最后结果的精确 ID；结果被后续完成请求取代后，旧 ID 查询明确失败。主机应在详情集合前后查询 ADSTIMING?/CONFIG?，核对 session、lastId 和结果 revision，避免拼接不同结果；不要因 ACK/终态丢失重发动作。元数据 `resultKnown` 表示终态时配置身份可知，`result` 表示诊断详情缓存可用；二者都不等于 restoreVerified。`terminalQueued` 只表示本地终态槽接受，不证明 USB/BLE/Wi-Fi 或 PC 收到。跨 boot 不比较 ID/sequence/generation；有限 ID 回绕跳过 0，仅以当前 session 和最后结果上下文解释。

Detail queries require the exact last-result ID; an overwritten result fails explicitly. Query ADSTIMING?/CONFIG? before and after a detail set and compare session, lastId and result revision. Do not replay the action when ACK or terminal receipt is unknown. Metadata resultKnown means final configuration identity is known; result means cached diagnostic details are available. Neither means restoreVerified. `terminalQueued` describes local terminal-slot acceptance, not USB/BLE/Wi-Fi delivery or PC receipt. Do not order IDs/sequences/generations across boots. Finite IDs wrap while skipping zero and are interpreted only within the current session and last-result context.

最多每组保留 8 条分类代表记录，不是 800 条全量原始捕获；recordsOmitted 明示容量不足。好样本只保留驱动实际返回的 raw，`wireKnown=0` 且 bytes/computed/received/xor=unknown；不得补造“好帧”原始 CRC。坏 CRC 保存驱动实际错误字节，raw/status 仍是不可信诊断值，不能进入科学曲线。所有详情行有界 512 B，终态槽 384 B；完整行放不下返回错误且缓冲为空，不把截断行称为完整结果。

At most eight category-representative records per group are retained, not an 800-read raw capture; recordsOmitted exposes capacity omissions. Good reads retain only the driver's returned raw value and report wireKnown=0 with unknown wire bytes/CRC fields. No good wire frame is reconstructed. Bad-CRC records preserve actual driver fault bytes, while their raw/status remain untrusted diagnostics and cannot enter scientific plots. Detail lines are bounded by 512 bytes and the terminal slot by 384 bytes; insufficient capacity returns an error with an empty buffer rather than a truncated result.

`ads_tail_probe_v1` 协商 `ADSCHK=TAIL,200`。与 TIMING 共享请求门控、完整帧边界、2 秒整体期限和 500 ms 恢复预留。TAIL 全程 CONT，四组依次为 STD/gain1、GPIOtail/gain1、STD/bypass、GPIOtail/bypass，各 200 次主读，无 held 重读。 / TIMING compares CONT/PULSE. TAIL compares standard and GPIO-tail sources at gain 1 and bypass. Both retain restoration and source evidence separately from science measurements.

kind 随接受时的持久请求进入 mailbox 与结果缓存，不能在发送时重贴新标签。ACK 保留既有前缀 `ACK,cmd=ADSCHK,timing=1,id=…`，追加 `probe=TIMING|TAIL`；这里 timing=1 是这套诊断控制族的兼容标记。ADSTERM 和所有详情行同样有 probe。`ADSTIMING?` 分别给 `pendingProbe` 和 `lastProbe`，无对应请求时为 none。主机必须按 session/请求 ID/probe 一致性取回结果；一个 TAIL 请求不能因后到的 TIMING 请求被解释成另一组实验。二者相互忙拒绝，不覆盖已经接受的请求。

The accepted persistent request carries its kind through the mailbox and immutable result cache; transmission never relabels it. The existing ACK prefix ACK,cmd=ADSCHK,timing=1,id=… gains probe=TIMING|TAIL; timing=1 remains a compatibility marker for this diagnostic control family. ADSTERM and every detail line also identify probe. ADSTIMING? reports pendingProbe and lastProbe independently, using none when absent. Retrieve results only when session, request ID and probe agree. A later TIMING request cannot reinterpret a TAIL experiment. The two kinds share busy rejection and never overwrite an accepted request.

`ADSTIMING?TAIL,<id>,<index>` 返回 `ADSTIMINGTAIL`，平面索引与 RECORD 相同，仅支持最后的 TAIL 结果。 / Require a matching cached TAIL ID and valid index. For a standard group the source identifies the normal 56-bit SPI path. The split groups separately report the 55-bit transfer and observed GPIO bit.

TAIL 补充记录区分 `tailApplicable`、`spiBits`、部分 `prefix`、`gpioKnown/gpioBit`、`assembledKnown/assembledCrc`、attempted/spiDone/frameDone/routeRestored。STD 组 tailApplicable=0、spiBits=56，不能把默认 tail 字段当成实际执行；GPIOtail 组 spiBits=55，prefix 最后一字节的未采位由 `lsbKnown=0` 标明，另由 gpioKnown 判断独立末位是否观测到。startUs 为单调微秒时间，spiUs/riseUs/sampleUs/fallUs/restoreUs 为相对 startUs 的微秒偏移。`routeRestored` 只描述驱动 pinmux/软件收尾证据；`physicalVerified=0` 保留，没有示波器时序或模拟安全保证。

The supplemental record distinguishes tailApplicable, spiBits, partial prefix, gpioKnown/gpioBit, assembledKnown/assembledCrc and attempted/spiDone/frameDone/routeRestored. Standard groups have tailApplicable=0 and spiBits=56; default tail fields are not execution evidence. GPIOtail uses spiBits=55; lsbKnown=0 marks the unobserved bit in the partial SPI prefix, while gpioKnown separately identifies whether the GPIO bit was sampled. startUs is monotonic microseconds; spiUs/riseUs/sampleUs/fallUs/restoreUs are microsecond offsets from it. routeRestored describes driver pinmux/software cleanup evidence only. physicalVerified=0 remains explicit; no oscilloscope timing or analog-safety guarantee is inferred.

`ads_tail_io_v1` 协商只读 `ADSTIMING?IO,<id>,<index>`。返回 `ADSTIMINGIO`，ID、kind、平面索引必须匹配最后的 TAIL 结果。 / IO pages share the bounded RECORD/TAIL index and last-result identity. Check the capability before requesting them.

IO reports `stage`, `clockKnown/misoKnown`, `clkFunc/clkSig/clkIE/clkOE/clkPeriph/clkInv/clkOD`, the corresponding `miso*` fields, and `clkLevelKnown/clkLevel`. Configuration and level describe software GPIO observations at admission; stage identifies the first failure or completion. `OE=1,Periph=1` delegates output enable to the peripheral and does not prove that the pad was actively driven. `physicalVerified=0` remains explicit: these fields verify neither electrical safety, sampling edges nor analog isolation. STD records use stage=not_applicable and known=0 for unobserved values. / IO 页保存 stage、GPIO 配置及读到的电平。时钟与 MISO 各有 known 位，未观测值保留 unknown。

失败终态的 `acceptedRev` 与 `rev` 可以不同：例如隔离 ADS 后，配置权威状态合法进入新的 DEGRADED revision。Host 必须保留接受时 revision、终态 revision 和 closing CONFIG，仍按同一 session、请求 ID 和 probe 读取全部缓存；不能因失败产生的新 revision 而丢弃错误证据，也不能将该失败标为成功。成功诊断仍要求配置身份一致、完整 schema、所有组实际完成、terminal/cleanup=0 且恢复已知已验证。CRC-only 的 first=0x109 保留，不能据此自动宣告整项诊断执行失败或科学有效。

A failed terminal may have different `acceptedRev` and `rev`: isolating the ADS can legitimately publish a new DEGRADED configuration revision. Preserve the accepted revision, terminal revision and closing CONFIG, and continue collecting the same session/request/probe cache. A failure-induced revision must neither discard fault evidence nor be called success. Successful diagnostic completion still requires consistent configuration identity, complete schemas, completed groups, zero terminal/cleanup errors and known, verified restoration. CRC-only first=0x109 remains visible; it alone establishes neither failed diagnostic execution nor scientific validity.

`adc_finalbit_v1` 公布读数来源及 CRC 拒绝证据。 / Distinguish a directly captured SPI frame from a software-assembled result. Retain any rejection as diagnostic evidence and preserve scientific validity masks.

`SPISOURCE?` returns a separate `SPISOURCE,scope=boot_probe` record containing `phase/sourceKnown/policy/source/interfaceKnown/interface/hz/actualHz/qualified/restored/joined/timedOut/err/cleanup`, with explicit `runtime=unobserved,physicalVerified=0`. It copies only the boot worker's immutable, mutex-published result; the control task never reads live driver fields. An unfinished or unknown source reports sourceKnown=0 and source/policy=unknown; an unconfirmed INTERFACE reports interfaceKnown=0 and interface=unknown. Policies are spi_bytes or gpio_tail_required. Sources are spi_bytes, spi56, spi55_gpio1 or unknown, distinguishing generic byte transfers, standard 56-bit ADC1 reads and separate SPI55/GPIO1 observations. Existing SPI? `scope=all_candidates` counters retain their meanings. SPISOURCE is not runtime telemetry; bracket it with matching CONFIG sessions to prevent joining results across boots. / 此页取自启动快照。hz/actualHz 为请求和驱动观测频率，sourceKnown/interfaceKnown 标出实际已知的来源和接口，不能据此推断运行时 SCLK 波形。

Standard `ADSSPIBYTES` retains its bytes field and adds the actual source and wireKnown=1. Split records must report `source=spi55_gpio1,wireKnown=0,bytes=unknown`, with separate `assembledBytes/spiPrefix/gpioKnown/gpioBit/tailStage`. assembledBytes contains six bytes: STATUS, four data bytes and the received CRC. spiPrefix contains seven bytes of partial SPI storage: byte zero belongs to command-phase DOUT and is not ADC payload; the final byte's bit zero is an unreceived zero placeholder that only the actual gpioBit supplies. Treat that bit as observed only when gpioKnown=1. Unknown observations produce unknown assembledBytes/spiPrefix/gpioBit, never initialized zero bytes presented as a capture. RAILCRC uses the same provenance fields and retains its seq/txn association. A complete split CRC-rejection record requires tailStage=complete; another stage is incomplete diagnostic evidence, not a complete ADC CRC frame. / 分步读取的 assembledBytes 是 STATUS、四数据字节及收到的 CRC，spiPrefix 的首字节属命令阶段，最后一位须以 GPIO 实际观测为准。RAILCRC 使用同一来源语义，并携带 seq/txn。

### BLE 服务和 CPU 角色 / BLE service and CPU owners

`ble_service_obs_v1` 的 `BLESVC?` 返回启动域 SDK `started/completed,wallUs,idleChanged/idleSame/idleUnknown`。`confOk/confError` 计 GATT 状态，`confMatched/confUnmatched` 计匹配 waiter 的结果。`confMatch=waiter_predicate,confEpoch=unknown,pcReceipt=unknown`。`work_reason_v1` 的 `SCHEDWHY?` 将 bt/tr/log/ser/bc/sc/wc 的 quantum/backstop/units 触发条件并列，`overlap=1` 表明条件可同时满足。 / Confirmation counts classify the same SDK event independently by status and waiter match. Use boot-scoped modular differences, including across reconnects in that boot. SDK wall time can include waits or preemption. Optional absent fields remain unknown.

| role | 应用 owner / Application owner |
|---|---|
| 0 | BLE_TX |
| 1 | TRANSPORT |
| 2 | ASYNC_LOG |
| 3 | SERIAL_TX |
| 4 | BLE_CTRL |
| 5 | SERIAL_CTRL |
| 6 | WIFI_CTRL |
| 7 | WIFI_OWNER |

`CPUOWNER?<role>` 须紧跟单个十进制 0…7（例如 `CPUOWNER?7`）。返回 `CPUOWNER,v=1,session,role,enabled,available,known,conflict,stale,reason,taskNumber,runUs,t0Us,t1Us,observedAtUs,ageUs,sampleMinUs,staleAfterUs,scope,isr,sdk,blocked`。 / Query one role at a time. Decode unknown numeric fields as the literal unknown. `scope=switch_out,isr=may_include,sdk=not_isolated,blocked=unknown` describe the runtime accounting boundary.

Each role self-samples only at its natural CPU0 checkpoint, with at least one second between completed samples. The kernel call `vTaskGetInfo(NULL, …, pdFALSE, eReady)` runs without the application mux, performs no stack-watermark scan and does not publish the placeholder eReady state. The dual-core kernel mux protects the U64 copy, followed by short-lock publication. Queries never read a live task handle; the cache retains no name, stack or handle pointer. The interface adds no tasks/delays or changes to existing budgets/priorities. With diagnostics OFF, checkpoints add no timer/kernel/mux calls. Profiling and ON-path observation still cost time/memory and can affect existing elapsed-time conditions; disturbance-free behavior is not claimed. Current target local static storage is 456 bytes, excluding global trace/TCB changes.

Host 差分须逐角色核同 session、同 taskNumber、known/available、无 conflict、未 stale、runUs 不倒退且采样 bracket 有序；时间和 runtime 不跨 boot 拼接，U64 倒退当错误，不擅自按 32 位补回绕。每角色使用自己的 `observedAtUs` 时间跨度，保留 t0/t1 不确定边界；多个查询不是原子跨任务 CPU 快照，比例不可直接求和。runUs 不含未切出的运行片段，可能含 ISR、SDK 内外所有本任务工作；没有 SDK/controller/IDLE task 样本，不可算全核 residual 或把 SDK wall−owner run 叫 blocked。BLESVC 的 cpu/blocked 继续 unknown，网络确认/PC 实收语义不变。

Hosts validate each delta independently: same session/task number, known/available, no conflict, fresh samples, nonregressing runtime and ordered sampling brackets. Times/runtime never span boots, and a U64 regression is an error, not an assumed 32-bit wrap. Each role uses its own observedAtUs span and retains t0/t1 uncertainty; separate queries are not an atomic cross-task CPU snapshot, so ratios must not be summed. Runtime excludes an open running slice and may include ISR and all task work inside/outside the SDK. Without SDK/controller/idle samples, neither full-core residuals nor SDK wall−owner run as blocked time are defined. BLESVC cpu/blocked remain unknown, and no network/PC receipt meaning changes.

[完整命令 / Commands](../command-reference.md) · [数据格式 / Wire format](../output-format.md) · [传输接口 / Transports](../transport-protocol.md)

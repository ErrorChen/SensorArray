# 固件与上位机契约：配置身份与可靠性修复


本轮 AP 修复提交为 `f8d8424492c855a402838737ee8718e32f5135d6`。验收镜像在提交前构建，设备报告 `ad0f9ef-dirty`。202 个受 Git 管理的构建输入均与该提交逐字节匹配，生成的 sdkconfig 和完整 ELF 有各自哈希。 / Firmware commit `f8d8424` contains all 202 tracked build inputs of the tested image. Its pre-commit BUILD version is `ad0f9ef-dirty`. [Image and source identity](../audit/artifacts/cdc_recovery_acceptance_20261116/wifi-bss-01-source-identity.json).
> **2026-11-25 当前实录：AP08与USB/BLE控制共存成功，UDP实收待测 / Current observation: AP08 starts alongside USB/BLE control; UDP receipt remains untested.** [wifi-bss-01构建](../audit/artifacts/cdc_recovery_acceptance_20261116/wifi-bss-01-validated-identity.json)与[READY核验](../audit/artifacts/cdc_recovery_acceptance_20261116/wifi-bss-ready-01.summary.json)绑定完整ELF`179f9183e2c31b35c906c9b811bb2ca1cf4d16dbca43ee3a1c6c4371c0f56a2d`、session`00B44EE26D0A4A0F`/boot1/CAP8/READY1；80入口/构建通过、203生产输入一致。SDK外部BSS及8,704 B任务校准表合计迁移29,116 B，保留既有采集速率与缓冲容量。[AP08](../audit/artifacts/cdc_recovery_acceptance_20261116/wifi-control-08/wifi-control-08.summary.json)的AP/AP/OFF/AP/OFF五请求全applied，首次AP53.432ms；最终同boot outputGate0/backend ready/quarantined0。internal free/largest前93,356/47,092 B、后29,008/20,468 B、min27,952 B；MEMFAIL0→0、incomplete0，无观察fault/reset，BLE observer正常关闭。**OFF只关输出gate，不证明radio断电；Wi-Fi UDP真实上位机实收、正式FPS及600秒/32配置/50切换/SHORT仍NOT_RUN。** AP04/AP05的NO_MEM、AP06分配失败与AP07 panic历史保留。 / The exact image boots CAP8/READY1 and passes all 80 entries/build. Supported external BSS placement moves 29,116 bytes, including the task-owned calibration table, without reducing acquisition rate or buffer capacity. All five AP/AP/OFF/AP/OFF requests apply; the first AP completes in 53.432 ms while USB/BLE control coexists. The backend remains ready with the output gate closed, no observed allocation failure, fault or reset, and a clean BLE observer shutdown. OFF does not prove radio power-down. Actual host UDP receipt, formal throughput and final acceptance remain untested; earlier failures remain preserved.

> **2026-11-25 保留AP07启动panic / Preserved AP07 startup panic.** [wifi-dma-headroom-01](../audit/artifacts/cdc_recovery_acceptance_20261116/wifi-dma-headroom-01-validated-identity.json)关闭SDK可选Wi-Fi IRAM speed两项后，map回收18,432 B共用SRAM；完整ELF`2fed45c22153b580aef27584107809d15cdabdd8afe6d6bd90cf6d5a282c07a7`、80入口/构建通过、203生产输入一致。[AP07](../audit/artifacts/cdc_recovery_acceptance_20261116/wifi-control-07/wifi-control-07.summary.json)前session`DE8FA9C044C728B7`/boot1，internal free/largest64,324/31,732 B；随后USB EOF/BLE断连，AP终态和OFF恢复未知。[USJ保留panic](../audit/artifacts/cdc_recovery_acceptance_20261116/dma-panic-01.summary.json)的record`C3FA0326`匹配该origin/同ELF；新session`DC4763F0436668C0`/boot2，PC`420C9C8D`位于`ieee80211_hostap_attach`，cause0x1C/vaddr0x2C。**AP07未取得post-MEMFAIL，不推定具体allocator；boot2的count0不能回填boot1。** 该run未证明AP/UDP可用；AP04/05/06旧证据保留，AP08结果单列于上方。 / The optional-code placement change releases 18,432 bytes in the map and passes all 80 entries/build. AP07 starts with more internal memory but loses USB and BLE. A retained panic matches its origin and exact ELF, locating ieee80211_hostap_attach in AP startup. No post-attempt MEMFAIL exists; the new boot's zero counter does not identify an allocator or fill the missing prior-boot observation. This run does not establish AP/UDP operation; the later AP08 result is recorded separately above.

> **2026-11-25 保留AP06分配失败观察 / Preserved AP06 allocation-failure observation.** [heap-failure-02构建](../audit/artifacts/cdc_recovery_acceptance_20261116/heap-failure-02-validated-identity.json)与[启动核验](../audit/artifacts/cdc_recovery_acceptance_20261116/heap-failure-ready-02.summary.json)绑定完整ELF `90f10fdaa5a0b859995caba70b1f1c65d3c7f9e19e85b39c81366e699c6e2736`、session `EE880CC8223D06C2`/boot1/CAP8/READY1，80入口/构建通过、203生产输入一致。[BLE共存AP06](../audit/artifacts/cdc_recovery_acceptance_20261116/wifi-control-06/wifi-control-06.summary.json)的`MEMFAIL?`由count0增至1、incomplete0：**1×1,604 B，caps0x80C（INTERNAL|DMA|8BIT）**；同ELF解码fn `0x3C1506DC`为`heap_caps_malloc`。[静态二进制路径](../audit/artifacts/cdc_recovery_acceptance_20261116/wifi-memory-source-map-01.md)强匹配Wi-Fi static RX初始化，**不是运行时caller/backtrace证明**。AP仍在34.050ms返回NO_MEM并隔离，cleanup0x103未知保留；internal free/largest前44,856/31,732 B、后28,480/9,716 B、min7,996 B。OFF gate同boot可查，无观察fault/reset；AP04/AP05旧值保留。**本轮未证明AP/UDP可用，无正式FPS窗口，600秒/32配置/50切换/SHORT仍NOT_RUN。** / The exact diagnostic image is boot-verified and all 80 entries/build pass with 203 matching inputs. During BLE coexistence, AP06 records one failed 1,604-byte internal DMA allocation. The decoded allocator API and static RX initialization path strongly match; the hook does not prove the runtime caller. AP still returns NO_MEM in 34.050 ms, with uncertain cleanup retained. Same-boot OFF queries remain available, with no observed fault/reset. AP04/AP05 history stays intact. AP/UDP availability, formal throughput and final acceptance are not established.

> **AP05历史状态 / Historical AP05 status:** [bt-host-memory-01闭合证据](../audit/artifacts/cdc_recovery_acceptance_20261116/bt-host-memory-control-evidence-01.json)核验2f74…镜像/CAP8/READY1。Bluedroid host优先PSRAM后内部free较上一镜像增加5,528 B，AP05仍返回NO_MEM并隔离；两条WTERM在PC的L通道实收。旧采集器仅保留recorded SHA、无独立源码归档，原始raw/events另行绑定；不伪造FPS，最终600/32/50/SHORT仍NOT_RUN。 / The new image has 5,528 more observed internal bytes, yet AP still returns NO_MEM. Both exact terminals reach the PC on L. The historical collector source is unavailable beyond its recorded SHA; raw/events are independently bound. Formal throughput and final acceptance remain unrun.

> **2026-11-21 保留首次AP失败观察（当时原因未知） / Preserved first AP failure observation, before diagnosis.** 同`wifi-owner-02` ELF `9e1ee9f9812dcd21a1295e315f08c10bdf98ae55d309e67b1f84b97b3c825573`的[AP实录](../audit/artifacts/cdc_recovery_acceptance_20261116/wifi-control-01/wifi-control-01.summary.json)中，id1先accepted/pending，随后查询running；AP写入后约10ms内出现USB EOF，终态未收到，OFF恢复FAILED_OR_UNKNOWN（主机事件时间，不是精确panic时刻）。[后续只读证据](../audit/artifacts/cdc_recovery_acceptance_20261116/wifi-failure-usj-01.summary.json)确认同ELF `reset=panic`、boot2 / session `370C59E1DF8015F5`、CAP8/READY1，控制在`serial_jtag`恢复通路可查询。**根因仍待诊断；先修复，600秒/32配置/50切换/SHORT及手动重启验收均NOT_RUN。** 新boot的idle/零guard计数不能证明旧SDK调用已取消，旧session的最终测试参数已失效。 / Request1 was accepted and observed running, followed by USB EOF roughly10ms after the AP write, with no terminal and unverified OFF restoration. Subsequent read-only queries confirm a same-image panic reboot and accessible Serial/JTAG recovery in the new boot. The root cause is not established; repair precedes final testing. New-boot idle counters do not prove cancellation of the previous call, and old-session capture arguments are no longer valid.

> **2026-11-21 AP尝试前的构建/启动核验记录 / Preserved build and boot verification before the AP attempt:** `wifi-owner-02` / ELF **`9e1ee9f9812dcd21a1295e315f08c10bdf98ae55d309e67b1f84b97b3c825573`**、bin `55b25e40cc20de1b619f000080d5fe984752e1e693b2327e6e2dda4de87c2c9e`，[78入口与构建](../audit/artifacts/cdc_recovery_acceptance_20261116/wifi-owner-02-validated-identity.json)全部exit0、201生产输入一致，app1,628,976 B；[完整镜像归档](../audit/artifacts/cdc_recovery_acceptance_20261116/wifi-owner-02-image-archive.json)另存。[烧录后启动证据](../audit/artifacts/cdc_recovery_acceptance_20261116/wifi-owner-02-boot-verification.json)：session `0ACE6CD97F8649DC` / boot1，CAP8/READY1，SPI启动资格2MHz、2000样本CRC/status/reg0；这是启动扫描，不是持续稳定性。MEM当前internal free/largest/min为27,408/11,252/21,044 B，scan创建前largest25,588、创建后8,180 B，查询sequential。Wi-Fi仍默认OFF、resources0、backend inactive，AP与显式重启/ROM枚举尚未实测；新镜像600秒/32配置/50切换/SHORT尚未运行。重启后的session必须重新实录，不能复用本次身份。 / All78 host entries and the build pass with201 matching production inputs and a1,628,976-byte app. Archived images and post-flash observations verify CAP8/READY1 in the stated session. The2MHz/2000-sample zero-error startup sweep is not a sustained run. Internal-memory observations are sequential and retain the scan-allocation cost. Wi-Fi remains uninitialized/OFF; AP, explicit restart/ROM enumeration and the new image’s long-run/matrix/transition/SHORT tests remain unrun. A later restart requires freshly recorded identity.

> **2026-11-21 最近已闭合吞吐筛查仅通过正常BLE数值预算 / Last closed throughput screen meets only the normal BLE numeric budget:** `ble-snapshot-06` / ELF **`c47a3710a770506abd030d03790b58c137150c9c1cba86a596f07962114a3c4e`** 已[启动核验](../audit/artifacts/cdc_recovery_acceptance_20261116/ble-snapshot-06-boot-verification.json)，session `630DC17010A097FF` / boot1；195生产输入一致，[73入口与构建](../audit/artifacts/cdc_recovery_acceptance_20261116/ble-snapshot-06-validated-identity.json)均exit0。[screen21](../audit/artifacts/cdc_recovery_acceptance_20261116/screen-21/screen-21-verdict.json)完成 **120.000119494秒** VOLT1 USB＋BLE：Capture/USB/BLE **64.195756/64.233205/63.079023 FPS**，USB7,707连续，BLE7,569缺141，[同criteria v2](../audit/artifacts/cdc_recovery_acceptance_20261116/screen-21/screen-21-criteria-v2-supplement.json) **141/7,710=1.828794% ≤2.5%**。判定仍 **OBSERVED_NOT_ACCEPTED / overall NOT_ESTABLISHED**；ADC独立窗0/81,815、USB/BLE invalid460/434保留、CAP8恢复核验。600秒、32配置USB＋BLE、50切换及本镜像SHORT/弱链路未跑；Wi-Fi隔离与手动重启补丁已完成源码/host验证，新目标仅已构建/启动，AP/重启/持续HIL仍待独立闭合，未宣称整体无P0/P1。 / Build06 boots successfully and all 73 host entries/build pass with 195 matching inputs. The closed **120.000119494-second** screen records **64.195756/64.233205/63.079023 FPS** capture/USB/BLE. USB has 7,707 internally consecutive frames; BLE has 7,569 with 141 missing observations: **1.828794% ≤2.5%**. Only that numeric budget passes; the verdict remains **OBSERVED_NOT_ACCEPTED / overall NOT_ESTABLISHED**. Separate ADC observations are 0/81,815, invalid points remain, and CAP8 recovery is verified. Long-run, complete configuration matrix, transition and same-image specialized acceptance remain unrun; Wi-Fi isolation/manual-restart source and host verification are complete, while the new target is only build/boot verified and its AP/restart/sustained-HIL gates remain open.

> **2026-11-21 保留build05筛查失败 / Preserved build05 screen failure:** `ble-snapshot-05` / ELF **`7f2d5ad9c73887f70d49c7f8eb45ca889c4c44fcb1fedd293299200d9b39b469`** 已[实际启动核验](../audit/artifacts/cdc_recovery_acceptance_20261116/ble-snapshot-05-boot-verification.json)，session `B73989CDD936FA54` / boot1。[73入口与构建](../audit/artifacts/cdc_recovery_acceptance_20261116/ble-snapshot-05-validated-identity.json)均通过、195生产输入一致；[screen20](../audit/artifacts/cdc_recovery_acceptance_20261116/screen-20/screen-20-verdict.json)完成 **120.000081240秒** VOLT1 USB＋BLE，Capture/USB/BLE **63.796225/63.871027/62.188982 FPS**。USB7,664连续，BLE7,461缺202，[同criteria补充](../audit/artifacts/cdc_recovery_acceptance_20261116/screen-20/screen-20-criteria-v2-supplement-02.json) **202/7,663=2.636043% > 2.5%：FAIL**。内部2,304 B发送工作区尚未证明性能收益；ADC独立窗0/80,344、USB/BLE无效点483/456保留，CAP8恢复核验。旧screen19/18/575c失败不覆盖；build04的SHORT/USB不读专项不替代新镜像整体或长期验收。 / Build05 is flashed and boot-verified, with all 73 host entries/build passing and 195 matching inputs. The closed **120.000081240-second** screen still **fails**: BLE receives 7,461 frames with 202 missing observations, **2.636043% > 2.5%**, while USB receives 7,664 consecutive frames. Capture/USB/BLE rates are **63.796225/63.871027/62.188982 FPS**. A 2,304-byte internal working copy has no demonstrated performance benefit. Separate ADC observations are 0/80,344 with invalid points retained and CAP8 recovery verified. Earlier failures remain; build04-specific SHORT/unread observations do not qualify this image's overall or long-run behavior.

> **2026-11-21 保留build04筛查失败 / Preserved build04 screen failure:** `ble-snapshot-04` / ELF **`a6b04a2a68ddcb5c8c39cc28020abbe5db430c1e140cbb3afb4ddb703fdb217a`** 已[启动核验](../audit/artifacts/cdc_recovery_acceptance_20261116/ble-snapshot-04-boot-verification.json)，session `B4828F8EF45001F1` / boot1。195生产输入一致，[73入口整轮与构建](../audit/artifacts/cdc_recovery_acceptance_20261116/ble-snapshot-04-validated-identity.json)均exit0；[screen19](../audit/artifacts/cdc_recovery_acceptance_20261116/screen-19/screen-19-verdict.json)闭合 **120.000139309秒** VOLT1 USB＋BLE，Capture/USB/BLE **64.392618/64.399646/62.340491 FPS**。USB7,728帧内部连续；BLE7,481帧缺250，[同criteria v2](../audit/artifacts/cdc_recovery_acceptance_20261116/screen-19/screen-19-criteria-v2-supplement.json) **250/(7481+250)=3.233734% > 2.5%：FAIL**。周期合并生效、保护queued/complete/staleNormal/deadlineMiss增量均0，但容量拒绝仍+228；不能从250中抵扣。独立ADC窗0/82,589，USB/BLE无效点502/471保留，CAP8恢复已查询核验。600秒/50切换/32配置整套/SHORT/弱链路仍未验收，不宣称整体无P0/P1。 / All 73 host entries and the build pass with 195 matching inputs, and startup is verified. The closed **120.000139309-second** screen nevertheless **fails** the normal BLE gap budget: **3.233734% > 2.5%**, at **64.392618/64.399646/62.340491 FPS** capture/USB/BLE. USB receives 7,728 internally consecutive frames; BLE receives 7,481 with 250 missing observations. Coalescing eliminates additional protected sends/overtaking in the observed query interval, while 228 local capacity rejections remain; they cannot be subtracted from PC gaps. A separate ADC window records 0/82,589 rejections, invalid points remain, and CAP8 recovery is verified. Longer and broader acceptance remains incomplete.

> **2026-11-21 保留旧镜像筛查失败 / Preserved prior-image screen failure:** `ble-snapshot-03` / ELF **`7ce5e9f8746aefe0f999b467bc08fc9470903a59141339829c6363a30db7e1c8`** 的[启动恢复](../audit/artifacts/cdc_recovery_acceptance_20261116/ble-snapshot-03-boot-verification.json)已实证；同session `0C1E0125F83FF11C` / boot1的[screen18](../audit/artifacts/cdc_recovery_acceptance_20261116/screen-18/screen-18-verdict.json)完成120.000089528秒VOLT1 USB＋BLE。Capture/USB/BLE **63.042560/63.017922/61.120460 FPS**；USB7562帧内部连续，BLE7334帧缺228，[criteria v2补充](../audit/artifacts/cdc_recovery_acceptance_20261116/screen-18/screen-18-criteria-v2-supplement.json) **228/(7334+228)=3.015075% > 2.5%：FAIL**。独立健康窗ADC **0/79,590**，USB/BLE无效点466/442保留，CAP8查询恢复。该旧镜像600秒、50切换、32配置整套、SHORT/弱链路验收仍NOT_RUN；不宣称整体无P0/P1。 / Startup is verified, but the completed 120-second screen fails the normal-link gap budget. Capture/USB/BLE rates are **63.042560/63.017922/61.120460 FPS**. USB receives 7,562 internally consecutive frames; BLE receives 7,334 with 228 missing observations: **3.015075% > 2.5%**. Separate ADC observations have 0/79,590 CRC rejections and retain 466/442 invalid USB/BLE points. CAP8 recovery is query-verified. Long-run, complete configuration-suite, transition, SHORT and weak-link acceptance remain unrun for this image; overall P0/P1 absence is not established.

> **2026-11-21 旧候选启动失败保留 / Preserved previous-candidate startup failure:** `ble-snapshot-02` / ELF **`575c5dc25217721cfb47f651556d0e28edd4b5fe666a022200b8737608ff2e40`** 已实际烧录；[只读设备证据](../audit/artifacts/cdc_recovery_acceptance_20261116/ble-snapshot-state-01.summary.json)报告 `READY=0,stage=scan_task_create_failed,err=0x101`（ESP_ERR_NO_MEM）。USB 控制仍可查询，SPI `phase=idle,attempted=0,samples=0`，因此 ADC CRC、采集性能和保护快照 HIL 均 **NOT_RUN**，初始化 `crc=0` 不是零错误率。`BOOT.heap=8295496` 不是 internal largest block。该失败已由上方新镜像独立复验启动恢复，原失败记录不改。 / The previously flashed candidate reports scan-task allocation failure while USB control remains responsive. SPI never starts; zero initialized counters are not a measured CRC rate. Aggregate 8.29 MB heap does not prove a contiguous internal stack allocation. The new image above independently verifies startup recovery; original failed evidence remains unchanged.

> **2026-11-21 历史完整采集窗口 / Historical completed capture window:** screen17 在 `8a1794c-dirty`、完整 ELF **`f1ec518dfba78453dcd7fe057bba29eb61a8f8898274a21e22125df26b933a04`** 上录制 **120.000057708 秒**；Capture/USB/BLE 为 **64.978526/65.035014/64.800778 FPS**，USB 7,805 帧内部连续、BLE 7,776 帧缺 29。按 criteria v2，**29/(7,776+29)=0.371557% ≤ 2.5%**，仅正常链路缺口数值预算通过；[补充判定](../audit/artifacts/cdc_recovery_acceptance_20261116/screen-17/screen-17-criteria-v2-supplement.json)不代表整体 PASS，原 `OBSERVED_NOT_ACCEPTED` 保留。独立健康窗 ADC CRC **0/81,537**；USB/BLE 无效点 373/370 保留，CAP8 查询恢复。普通 LOG 策略已有查询证据；新弱链路适应/保护快照仍实施中，本 ELF 的 600 秒/50 次切换/32 配置未运行。 / Screen17 records **120.000057708 seconds** on `8a1794c-dirty`, full ELF **`f1ec518dfba78453dcd7fe057bba29eb61a8f8898274a21e22125df26b933a04`**, at **64.978526/65.035014/64.800778 FPS** capture/USB/BLE. USB has 7,805 internally consecutive frames; BLE has 7,776 with 29 missing observations. Under criteria v2, **29/(7,776+29)=0.371557% ≤ 2.5%** meets only the normal-link numeric gap budget. The [supplement](../audit/artifacts/cdc_recovery_acceptance_20261116/screen-17/screen-17-criteria-v2-supplement.json) is not overall PASS; original `OBSERVED_NOT_ACCEPTED` remains. The separate health span has **0/81,537 ADC CRC rejections**; 373/370 USB/BLE invalid points remain, and CAP8 recovery is query-verified. Ordinary-LOG policy has query evidence; new weak-link adaptation/protected snapshots remain in progress. This ELF’s 600-second/50-transition/32-configuration runs are NOT_RUN.

<a id="wifi-ap08-observation"></a>

## AP08实录不改变交付含义 / AP08 observation preserves delivery semantics

[AP08](../audit/artifacts/cdc_recovery_acceptance_20261116/wifi-control-08/wifi-control-08.summary.json)在完整ELF`179f9183e2c31b35c906c9b811bb2ca1cf4d16dbca43ee3a1c6c4371c0f56a2d`、session`00B44EE26D0A4A0F`/boot1中完成AP/AP/OFF/AP/OFF五请求，全部applied；首次AP完成53.432ms。USB查询和[零命令BLE observer](../audit/artifacts/cdc_recovery_acceptance_20261116/wifi-bss-coexist-01/wifi-bss-ble-01.ble.summary.json)共存，后者无unexpected disconnect且正常stop_file关闭。此新镜像用SDK支持的external BSS及任务FDC校准表迁移释放内部余量；没有降低采集/缓冲容量或重定义科学wire。

The exact-image AP08 sequence applies all five requests, with first initialization completing in53.432ms. USB queries coexist with a zero-command BLE observer that closes normally without an unexpected disconnect. SDK-supported external BSS and the task-owned FDC calibration table create internal headroom without reducing acquisition/buffer capacity or redefining scientific wire data.

重复AP和OFF→AP复用已ready backend，不是重复deinit/init。末次OFF的`outputGate0,backend=ready,resources1,quarantined0,physicalOff=unknown`是预期软件gate语义，不能扩展成radio断电。MEMFAIL count0→0/incomplete0只覆盖该boot注册后记录范围，heap前93,356/47,092 B、后29,008/20,468 B为顺序free/largest。WTERM的queued仍只是本地发布；未凭通知计数认定逐条实收，Wi-Fi UDP真实PC实收、关联/吞吐和持续性能仍NOT_RUN。此前AP06的1×1,604 B内部DMA失败与AP07的hostap_attach panic分别保留，不能因AP08成功回填旧记录。

Repeated AP and OFF→AP reuse the ready backend rather than reinitialize the driver. Final OFF retains resources and unknown physical-off state as the software-gate contract requires. MEMFAIL0→0/incomplete0 is scoped to recorded callbacks in this boot; memory snapshots remain sequential. Queued still means local publication, notification counts do not prove individual terminal receipt, and actual Wi-Fi UDP receipt, association/throughput and sustained performance remain untested. Earlier AP06 allocation failure and AP07 panic are preserved independently.

<a id="memory-failure-v1"></a>

## MEMFAIL v1：boot范围的失败分配观察 / Boot-scoped failed-allocation observations

`heap-failure-02`新增精确只读`MEMFAIL?`，early/正常控制均可查询；不清计数、不重试分配、不改变Wi-Fi/采集配置。其[实际构建](../audit/artifacts/cdc_recovery_acceptance_20261116/heap-failure-02-validated-identity.json)与[ready02](../audit/artifacts/cdc_recovery_acceptance_20261116/heap-failure-ready-02.summary.json)已绑定完整ELF`90f10fdaa5a0b859995caba70b1f1c65d3c7f9e19e85b39c81366e699c6e2736`，session`EE880CC8223D06C2`/boot1/CAP8/READY1；旧镜像unsupported不是零失败。callback保留内部静态首/末记录，以nonwaiting try-lock访问；不分配、日志、遍历栈或读取functionName所指字符串。失败记录仅覆盖注册之后，不跨boot保留。

The exact read-only query is available in early and normal control on the boot-verified diagnostic image. It neither resets the counter, retries allocations nor changes configuration. Older unsupported replies are not zero failures. The callback stores bounded first/last records in internal static memory, uses a nonwaiting lock attempt and performs no allocation, logging, stack walk or API-string dereference. Coverage starts at registration and does not survive a reboot.

| 字段 / Fields | 解释 / Meaning |
| --- | --- |
| `v,scope,session,bootId,identityKnown,elfSha256` | `scope=boot`；使用同session/boot/full ELF比较，不拼接跨重启值。 / Compare only within the same full image and boot identity. |
| `attempted,regKnown,registered,regErr` | 尝试注册、注册结果是否已知、是否成功及原错误；不能把未注册解释为未失败。 / Registration status is independent of whether failures were observed. |
| `known,firstSize,firstCaps,firstFn,lastSize,lastCaps,lastFn` | known0的各记录为unknown；known1提供首末SDK callback参数。Fn是opaque API-string地址，必须按同ELF离线解码，**不是caller PC**。 / Known records preserve callback arguments; function pointers identify allocator API strings, not the caller. |
| `count,saturated,incomplete` | recorded callback计数；达到u32上限置saturated，锁忙漏记置incomplete。count0不证明注册前或未捕获范围无失败。 / Saturation and incomplete capture remain explicit. |
| `fn,countScope,coverage` | 固定`opaque_pointer,recorded,since_register`；不是全部尝试次数或完整heap trace。 / This is a bounded observation, not a complete allocation ledger. |

SDK`*_prefer`只在所有偏好都失败后调用hook，caps为**最后尝试**的capabilities。pinned v5.5.5的`heap_caps_calloc_prefer`报告size而不是n×size，不能未经调用参数核验放大为总请求字节；直接`heap_caps_calloc`则报告n×size。size也不包含alignment/heap-poison开销。MEM的internal8 free/largest/min是另一次顺序查询，不是失败瞬间的DMA统计；已知非零总free仍不能保证匹配caps的连续分配。详细[源码map](../audit/artifacts/cdc_recovery_acceptance_20261116/wifi-memory-source-map-01.md)保留精确SDK引用。

Preference allocators call the hook only on ultimate failure and report the last attempted capabilities. Pinned calloc_prefer reports size rather than n×size; direct calloc reports the product. Sizes exclude alignment/poisoning overhead. Separate internal8 memory snapshots are not instantaneous DMA statistics, and aggregate free memory does not guarantee a contiguous block with the required capabilities.

[AP06](../audit/artifacts/cdc_recovery_acceptance_20261116/wifi-control-06/wifi-control-06.summary.json)中count0→1、incomplete0，first=last1,604 B/caps0x80C/fn0x3C1506DC；同ELF解码为`heap_caps_malloc`的INTERNAL|DMA|8BIT请求，与`wDev_Rxbuf_Init`的1600+4分配路径强匹配，但不是runtime caller/backtrace证明。AP04/AP05无此记录，不回填。AP06仍返回NO_MEM并隔离，OFF只是output gate。后续[AP07](../audit/artifacts/cdc_recovery_acceptance_20261116/wifi-control-07/wifi-control-07.summary.json)在不同ELF2fed…发生USB EOF、无终态或post-MEMFAIL；[panic回读](../audit/artifacts/cdc_recovery_acceptance_20261116/dma-panic-01.summary.json)的origin与AP07匹配，PC位于hostap_attach，但boot2的MEMFAIL count0只属于新boot，不能证明旧boot无分配失败，也不能把AP06的allocator套用到AP07。

AP06 records one direct 1,604-byte internal DMA request failure, strongly matching the static RX code path without identifying the runtime caller. Earlier AP04/AP05 runs are not retroactively populated. AP06 still returns NO_MEM and OFF remains an output gate. Later AP07 on another ELF loses USB without a terminal or post-attempt MEMFAIL; its retained panic locates hostap_attach, while the new boot's zero counter cannot establish old-boot allocation results or import AP06's allocator attribution.

<a id="panic-diagnostics-v1"></a>

## PANIC v1：保留诊断，不是修复 / Retained diagnosis, not a fix

本轮新增`main/control/sensorarrayPanicDiagnostics.{c,h}`及`PANIC?`、`PANIC?1`、`PANIC?2`三个精确只读命令，early/正常控制均可查询；无触发panic、清除记录或改变配置的命令。当前诊断镜像没有新增PROTO/CAPS2能力位；旧镜像unsupported须保持unsupported，不能解释为known0。候选ELF `6a91b6aa606ada32346f9d86ab6ad85df40187bdb567846f643262345ca3a94e`的[79入口/构建](../audit/artifacts/cdc_recovery_acceptance_20261116/panic-diagnostic-01-validated-identity.json)及[独立源码、host、目标审查](../audit/artifacts/cdc_recovery_acceptance_20261116/panic-diagnostics-independent-review-01.json)通过。它只为下一次故障保存有限证据，不能补录此前没有捕获的panic；真实RTC保留及wifiOwner栈溢出已由本节末实录单独核实；具体调用触发点和修复效果仍待验证，不因加诊断而宣称已修复。

New panic-diagnostics APIs expose three exact read-only commands in early and normal control. They neither inject a panic, clear a record nor change configuration. This diagnostic image adds no PROTO/CAPS2 capability bit; an older unsupported reply is not known0. The linked79-entry build and independent source/host/target review pass for the stated ELF. This adds bounded evidence capture only and cannot reconstruct a panic that was never captured; Physical retention and the owner-stack overflow are separately observed below; the precise call trigger and a validated repair remain open.

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

本板[实际PANIC闭合记录](../audit/artifacts/cdc_recovery_acceptance_20261116/wifi-owner-stack-overflow-evidence-01.json)现确认record F9CE9DC1三页一致，current585FFDD061878C2E/boot2，origin7939E4B087EA883A/boot1/fullELF6a91…匹配AP02前身份；65字节hex完整解码为wifiOwner stack overflow。所有相关reply绑定实际TX后的原始完整行，26输入hash不变；没有补造page0末尾复读、host RTC CRC重算或backtrace。该次READY0/boot_sweep与CAP8配置分别保留。具体嵌套调用触发点、修复和最终性能仍未验收。

The [closed hardware record](../audit/artifacts/cdc_recovery_acceptance_20261116/wifi-owner-stack-overflow-evidence-01.json) now confirms three matching pages and a65-byte untruncated wifiOwner stack-overflow message, with the origin matched to AP02 preflight. All replies bind to complete original lines after their writes and26 input hashes remain unchanged. No trailing metadata reread, host RTC-CRC recomputation or backtrace is invented. READY0/boot_sweep remains distinct from CAP8 configuration; the nested trigger and repair remain unvalidated.

<a id="wifi-owner-contract-v1"></a>

## Wi-Fi owner v1 与异步控制 / Wi-Fi owner v1 and asynchronous control

**本轮新增接口；当前为源码/host验证，最终候选身份与实机验收另记。** [交接02](../audit/artifacts/cdc_recovery_acceptance_20261116/wifi-owner-contract-handoff-02.json)及[host02 manifest](../audit/artifacts/cdc_recovery_acceptance_20261116/wifi-owner-host-02/manifest.json)绑定实现和14个exit0入口（owner 60个实际C场景）。最后完成吞吐筛查仍是image06/screen21；不能用它证明以下新Wi-Fi行为。上位机改动仍交独立软件仓库任务，本仓库不修改该仓库。

**New interfaces in this round; source/host evidence is separate from the final image identity and hardware acceptance.** [Handoff02](../audit/artifacts/cdc_recovery_acceptance_20261116/wifi-owner-contract-handoff-02.json) and the [host02 manifest](../audit/artifacts/cdc_recovery_acceptance_20261116/wifi-owner-host-02/manifest.json) bind the implementation and14 passing entry points, including60 actual-C owner scenarios. The last completed throughput screen remains image06/screen21 and does not qualify these Wi-Fi changes. Host implementation belongs to the separate software-repository task.

先在 `PROTO?.caps` 查 `caps2_v1`，然后发送 `CAPS2?`，当前回复为 `CAPS2,v=1,page=1,caps=wifi_owner_v1,next=none`。这是能力分页，不是ready证明；旧固件没有该cap时不推断新异步语义。现有最长PROTO含换行510 B，Wi-Fi能力放在第二页接口中，不能截断原回复。CAPS2当前46 B含换行，后续页应按公开next处理。

First require `caps2_v1` in PROTO, then query CAPS2 for `wifi_owner_v1`. The current reply above advertises capability, not readiness. Do not infer the new asynchronous contract on older firmware without the capability. The existing longest PROTO reply is510 bytes including newline; pagination avoids truncation. Current CAPS2 is46 bytes including newline; follow any future explicit next-page indicator.

| 命令 / Command | 语义 / Meaning |
| --- | --- |
| `WIFI?` | `ACK,cmd=WIFI,...,ver=2`，一次owner snapshot；requested、outputGate、backend、resources及request分开。 / One owner snapshot with separate requested mode, output gate, backend, resources and request state. |
| `WIFI=AP` | accepted1不等于applied；未完成ok0/pending1。任务可能先完成，ACK可携带该确切已缓存终态。相同ready配置产生新id的幂等applied、不重复SDK调用。busy/quarantine/bootstrap失败为accepted0/id0和真实err。 / Acceptance is not application; completion may race ACK formatting. Same-ready requests are idempotently applied under a new ID; rejection has no accepted ID. |
| `WIFI=OFF` | 新id/epoch，立即应用软件输出gate；不stop/close/deinit、不删除任务、不清quarantine或持有中的SDK对象。scope为output_gate_only，physicalOff始终unknown。 / Immediate software-gate result under a new ID/epoch; not radio shutdown, cancellation or quarantine recovery. |
| `WIFIREQ?` | 当前最后请求，无副作用。 / Read the latest request without side effects. |
| `WIFIREQ?<id>` | 精确非零十进制u32；例如`WIFIREQ?17`，显式0无效。旧结果被淘汰时known0/outcomeunknown/reasonnot_cached，不代入另一个请求。 / Query an exact nonzero decimal u32 ID; an evicted result is unknown rather than a substituted request. |
| `WIFIGUARD?` | SDK调用/隔离的缓存证据，不进入SDK、不测radio/bus。 / Cached SDK-call/quarantine evidence, without querying the SDK or physical radio/bus. |
| `WIFIQ?D`, `WIFIQ?L`, `WIFIQ?C` | 三条精确只读命令，分别返回DATA/LOG/CTRL的boot域计数。 / Three exact read-only per-channel queries with boot-scoped counters. |

`WIFI?`字段：`v=AP|OFF,ok,ver=2,session,boot,bootKnown,requested,outputGate,backend,startAllowed,resources,initErr,quarantined,id,epoch,linkEpoch,outcome,pending,physicalOff=unknown,peerReceipt=unknown`。backend取`quarantined/ready/initializing/resource_failed/inactive`。initErr仅指惰性payload/timer/task资源创建结果，AP后端失败查WIFIREQ/GUARD。AP的ok1要求本次applied、gate开、backend ready且未隔离；OFF的ok1仅证明gate应用。初始默认OFF尚无请求id0/ok0可为合法未初始化状态，不是测得硬件故障。当前极值格式回归310 B。

WIFI version2 fields are listed above. Backend distinguishes quarantined, ready, initializing, resource failure and inactive. initErr covers lazy resource creation only; request/guard records describe backend AP errors. AP ok1 requires applied state, enabled gate, ready backend and no quarantine. OFF ok1 proves only gate application. Default inactive OFF with no request can legitimately be id0/ok0. The tested maximum reply is310 bytes.

`WIFIREQ`及`WTERM`字段：`v=1,session,boot,bootKnown,id,known,epoch,requested,outcome,err,cleanupErr,acceptedUs,completedUs,terminalAttempts,terminalQueued,terminalPending,scope,peerReceipt=unknown`。outcome为`none/pending/running/applied/failed/superseded/quarantined`；unknown精确查询另标unknown/not_cached。一个active加两个terminal缓存，terminal只commit一次；OFF可supersede活动AP，不能改写旧failed/quarantined/applied结果。pending/running的completedUs为unknown。AP applied要求对应epoch的必要SDK调用、socket和RX task实际创建成功，scope=backend_and_gate；不证明已有station或PC。WTERM走本地Serial优先＋可观察network lifecycle，最多3次、相隔200ms，可早于或晚于ACK；terminalQueued只证明本地接纳。精确查询优先于重发setter。当前极值338 B。

WIFIREQ/WTERM fields are listed above. One active request and two cached terminals are retained; terminal commitment is single-shot. OFF may supersede an active AP request without rewriting an already terminal result. Pending/running completion time is unknown. AP application requires the matching epoch's required SDK calls, sockets and RX-task creation, not a connected station or PC. WTERM has at most three local submission attempts200ms apart via priority Serial and observed network lifecycle, and can precede or follow ACK. Query the exact result rather than replay a setter. terminalQueued never acknowledges peer receipt. The tested maximum is338 bytes.

| WIFIGUARD字段 / Field group | 定义 / Definition |
| --- | --- |
| `v=1,session,boot,bootKnown` | boot身份，unknown不填0。 / Boot identity; unknown is not fabricated zero. |
| `quarantined,active,ownerBusy,phase,callId` | active指隔离owner的SDK调用，不是radio状态或RX等待；phase是最近执行阶段。 / Active describes the owner's SDK call, not radio state or RX waiting; phase names the execution stage. |
| `startUs,deadlineUs,ageUs,detectedUs,pollUs=100000` | 单调µs。无调用的start/deadline/age及未隔离的detected为unknown；100ms观察周期不是硬实时保证。 / Monotonic microseconds; absent call/quarantine times remain unknown. Poll period is not a hard-real-time guarantee. |
| `completionKnown,err,cleanupErr,sdkErr,errno,returned` | 首个事务/guard错误独立于cleanup；sdkErr/errno/returned来自真实SDK返回。隔离后completionKnown保留该调用，晚返回只补证据，不清隔离。 / First error and cleanup are separate; actual-return evidence is distinct from timeout. Late completion updates evidence without clearing quarantine. |
| `held,writing,timeouts,late,recovery=explicit_manual_restart_attempt` | held包含WRITING/PENDING/OWNED；未知owner保留。显式重启只是恢复尝试。 / Held includes WRITING/PENDING/OWNED; unknown ownership is retained. Explicit restart is a recovery attempt. |

phase覆盖`idle/netif_init/event_loop/netif_new/netif_attach/handlers/driver_init/event_register/set_mode/set_config/start/set_power/set_ps/set_protocol/set_bandwidth/socket/setsockopt/bind/send/station_query/close/event_unregister/stop/deinit/clear_handlers/netif_destroy/rx_task_create`；这是软件调用阶段，不是物理安全状态。当前WIFIGUARD极值483 B。

Phase values above identify software call stages, not physical safety. The tested maximum WIFIGUARD reply is483 bytes.

`WIFIQ`字段：`v=1,session,boot,channel,scope=boot,queued,rejected,sent,dropped,expired,stale,partial,driverAcceptedBytes,terminalNoSink,terminalEvicted,stackKnown,stackMinBytes,stackScope=owner_after_job,sentScope=full_sendto_return,peerReceipt=unknown`。此回复没有bootKnown字段。queued表示深拷贝入owner队列；rejected是准入失败；sent及旧Transport的wifiDataSent/wifiLogSent/wifiCtrlTx仅在实际完整sendto返回后增加，晚返回记一次。不把ESP_OK入队当sent，不把完整返回当PC收到。dropped含expired/stale/partial/SDK失败，子计数重叠不能相加；在途timeout尚不证明datagram丢失。driverAcceptedBytes为u64，其他计数u32模回绕；仅同boot、有界间隔计算delta。terminalNoSink/terminalEvicted是终态保留/提交边界。stackMinBytes是owner在自身工作真实返回后缓存的最小剩余栈字节，未采样stackKnown0/unknown；不是RX或全系统栈，也不扫描被阻塞的其他任务。当前极值408 B。

WIFIQ fields are listed above; this reply has no bootKnown field. Admission, actual full sendto completion and peer receipt are separate. Drop categories overlap, and an in-flight timeout alone cannot establish datagram loss. Byte totals are u64; other counters use u32 modular arithmetic only within a known boot and bounded observation interval. The cached stack minimum is remaining owner-stack bytes sampled after real work returns, not an RX/system-stack scan. Unknown remains explicit. The tested maximum is408 bytes.

`CPUOWNER?7=WIFI_OWNER`为新增role；原0–6不改，6仍是WIFI_CTRL。需要同时支持cpu_owner_v1和分页wifi_owner_v1才查询7；旧image06仅有0–6，不重解释历史表。原SCHED/SCHEDWHY的bt/tr/log/ser/bc/sc/wc七组字段保持原义，不追加第八组或移位。各Wi-Fi查询各自原子复制owner snapshot，多个命令不是一个原子集合；Host核对session/boot/id/epoch并在有变化时重查，不能拼接从未存在的状态。request/epoch/linkEpoch非零u32跳0回绕；只按精确ID和当前boot缓存解释，不跨boot排序。

New CPU role7 is WIFI_OWNER; role6 remains WIFI_CTRL and roles0–6 retain their identity. Query7 only when both cpu_owner_v1 and paginated wifi_owner_v1 are present. Historical image06 exposes only seven roles. Existing seven-field SCHED/SCHEDWHY semantics are unchanged. Individual snapshots are coherent, but multiple query replies are not an atomic set: compare identity/request epochs and retry if they change. Nonzero u32 request/epoch counters wrap past zero; use exact current-boot IDs rather than cross-boot ordering.

资源与期限：默认OFF不创建任务/timer/payload；真实boot身份和两个启动资源预留参与者完成后才开启动gate。owner永久7,424 B PSRAM\|8BIT（2×2304普通、1×2304 lifecycle、1×512 CTRL），无internal fallback；wifi-stack-01的owner/RX栈分别6144/4096 B，执行/初始化工作区为owner生命周期，metadata/锁/栈仍内部。copy/SDK/callback在锁外，WRITING/OWNED不可覆盖。AP执行5s、cleanup最多1s、总6s；send500ms；100ms观察只置隔离，不调用SDK/free/reset。Wi-Fi队列1s send-start TTL取原protected deadline较早值，不续期，**不保证优先级洪流或隔离时1s内回收pending**。Transport保护descriptor的1s逐dispatch到期清理是另一层。原始err不被cleanup覆盖；driver_init返回错误可能隐藏内部deinit失败，此时保留对象隔离；不重建部分存活资源，不自动HIGH→BLE_COMPAT重试，不把timeout当取消。

Resources and deadlines are local isolation policies. Default OFF allocates no owner task/timer/payload until boot identity and both startup reservations are established. Persistent PSRAM-only storage totals7,424 bytes, with 6144/4096-byte owner/RX stacks in wifi-stack-01, owner-lifetime execution/init workspace and internal metadata/locks/stacks. Copies, SDK calls and callbacks run outside the owner lock. AP execution/cleanup/total limits are5/1/6 seconds; send is500ms and observation100ms. The one-second Wi-Fi send-start TTL preserves any earlier protected deadline and forbids starting stale work; it does not promise reclaiming pending slots during priority starvation or quarantine. Transport expiry is a different layer. Unknown ownership and partial initialization remain retained; neither timeout nor an initialization error proves safe cancellation/deinitialization.

<a id="manual-restart-contract"></a>

### 显式RESTART/USBBOOT终止路径 / Explicit RESTART/USBBOOT terminal path

本轮新增`sensorarrayUsbOtgManualRestart(bool)`仅服务已受理manual命令。原RACK本地发布栅栏、RESTART?缓存、RAPP restarting意图及有限发送机会不改；它们不证明PC接收或reset发生。USB owner准备仍有250,000µs期限；成功通过已有linker wrapper进入IDF5.5.5/S3的esp_restart_noos，避免esp_restart先执行Wi-Fi shutdown handler而新卡住。generic/AUTO仍用原esp_restart路径。失败/晚USB确认仍走原emergency Core Reset/USJ恢复策略，未知DMA缓冲不释放；USBBOOT成功路径保留已验证ROM下载标记，但必须由新枚举/芯片ID实测确认。

New sensorarrayUsbOtgManualRestart serves only accepted explicit commands. Existing local RACK gating, cached restart intent and bounded RAPP publication opportunity remain, without proving receipt or reset. After the250,000µs USB-owner preparation fence, success enters the existing wrapped noos path in pinned IDF5.5.5/S3, bypassing potentially blocking shutdown handlers. Generic/AUTO still uses esp_restart. Failed/late preparation retains the emergency Core Reset/USJ path and unknown DMA ownership. A prepared ROM latch still needs actual enumeration/chip-ID verification.

这是当前SDK私有终止API，不是通用cache-off调用；RTC watchdog在后续cache/core/clock收尾之前设置，不承诺精确1s墙钟恢复，也不假定不同reset级别都保留RTC下载标志。跳过最终wall-time/RTC同步可能失去日历连续性；科学时间是boot域esp_timer，跨boot不能排序。[独立74 USB host场景/源码审查](../audit/artifacts/cdc_recovery_acceptance_20261116/manual-restart-independent-review-01.json)已通过，但最终ELF的wrapper目标和正常/USBBOOT/AP成功失败隔离下的实机恢复须另验；不添加自动重启循环或全片擦除，不宣称模拟SAFE。

This is a pinned private terminal API, not a general cache-off call. RTC watchdog setup precedes subsequent teardown, but neither exact one-second wall-clock recovery nor latch survival through every reset level is guaranteed. Skipped final RTC/wall-clock synchronization can lose calendar continuity; scientific time remains boot-scoped. [Independent source review and74 USB host scenarios](../audit/artifacts/cdc_recovery_acceptance_20261116/manual-restart-independent-review-01.json) pass. Final-target wrapper checks and physical application/ROM/AP recovery remain separate; there is no new automatic reset loop, whole-chip erase or analog-SAFE assertion.

### 本次返回失败后的手动重启观察 / Manual restart after this returned failure

[wifi-quarantine-restart-01](../audit/artifacts/cdc_recovery_acceptance_20261116/wifi-stack-control-evidence-01.json)在完整ELF ab39…、session E400DABECADCE252/boot1的`driver_init`已返回、active0/completionKnown1条件下只发送一次RESTART。1.403712秒后观察新OTG枚举；后续BUILD/CONFIG为同ELF、session5AAC0452906F854B/boot2/CAP8。首次post查询`reply_deadline_expired`保留，未采READY；不是对仍阻塞SDK的取消证明，也不代表AP/UDP可用。RACK/RAPP仍只提供本地状态，不改变上面的交付契约。

The closed run sends one RESTART after the failed SDK call is known to have returned. A new OTG epoch appears 1.403712 seconds later; subsequent same-image BUILD/CONFIG identifies a new CAP8 session/boot. Its first post-query deadline error remains and no READY query is collected. This is neither cancellation evidence for an in-flight blocked call nor AP/UDP availability; RACK/RAPP retain their local-publication meaning.

## NVS初始化失败保留校准 / Preserve calibration on NVS initialization failure

工作区修复：`sensorarrayCalibrationInit`直接返回`nvs_flash_init_partition("calib")`原始错误，NO_FREE_PAGES/NEW_VERSION_FOUND不再触发自动擦除。LOAD/QUERY/SAVE在初始化失败时保留错误和既有记录，不把未知内容解释为空白或自动改写为默认值。[真实源码host回归](../audit/artifacts/cdc_recovery_acceptance_20261116/calibration-preservation-host-01-command.json)以合成NVS返回验证三类错误×四API、零erase/write与原blob后续可重载；这不是物理NVS故障注入。上述ab39…镜像不含此修复；后续wifi-memory-01已编译并启动，亦移除Transport通用NVS初始化的自动擦除，NVS失败仍启动独立控制任务并保留BLE初始化错误。[独立源码/host/target审查](../audit/artifacts/cdc_recovery_acceptance_20261116/transport-psram-independent-review-01.json)通过；本轮没有物理NVS故障注入、CAL=SAVE或擦除，正常启动不替代持久化故障实测。

The workspace fix returns the original calibration-partition initialization error without automatically erasing NO_FREE_PAGES/NEW_VERSION_FOUND contents. LOAD/QUERY/SAVE preserve that error and existing bytes rather than treating unknown contents as empty or replacing them with defaults. Actual-source host tests use synthetic NVS errors across three errors/four APIs, verify zero erase/write and reload the unchanged blob. They do not qualify the older ab39… image. Later wifi-memory-01 includes this fix and removal of generic Transport auto-erasure: NVS errors retain independent control startup and the BLE initialization error. Independent source/host/target review passes, but there is no physical NVS fault injection, CAL=SAVE or erase; ordinary boot does not validate a persistence-failure scenario.

### Bluedroid host的PSRAM优先策略 / Bluedroid host PSRAM preference

bt-host-memory-01启用官方`CONFIG_BT_ALLOCATION_FROM_SPIRAM_FIRST=y`，只令Bluedroid动态host malloc/calloc优先外部RAM、保留内部fallback；不改变controller DMA、FreeRTOS栈/队列、全局malloc、MTU或传输容量。构建/测试与配置diff/目标反汇编的hash在[闭合证据](../audit/artifacts/cdc_recovery_acceptance_20261116/bt-host-memory-control-evidence-01.json)中对应同ELF。AP05仍NO_MEM，不能因host内存增加就报告Wi-Fi ready或降低错误语义。新run两条WTERM与原USB查询的immutable字段一致，发布计数仍是较早快照。采集器后续增加allocation-trace参数，旧run源码只剩recorded SHA且无归档；不借新源码解释旧run，亦不把新增诊断当作已经烧录。

The official option makes dynamic Bluedroid host allocation prefer external RAM while retaining internal fallback. It changes neither controller DMA, FreeRTOS stack/queue placement, global malloc, MTU nor transport capacity. Stored build/test/config/target hashes bind the exact image. AP05 still fails, so extra memory is not Wi-Fi readiness. Its two received WTERM records retain the same immutable USB-query fields and earlier publication counters. The collector was subsequently extended; the old source has only a recorded SHA and no archive. New code or diagnostics are not retroactively attributed to this run.

### WTERM实收与发布快照 / WTERM receipt and publication snapshots

[wifi-memory-01共存实录](../audit/artifacts/cdc_recovery_acceptance_20261116/wifi-memory-control-evidence-01.json)的两条WTERM通过L而非C接收，都是单个原始ATT中的完整ASCII行（285/282 B，未加G或科学帧CRC）。其session/boot/id/epoch/结果/时间与稍后USB无副作用查询一致；报文中terminalAttempts/terminalQueued/terminalPending=0/0/1是发布前快照，稍后查询为1/1/0，不是旧结果或重新应用。PC实收依据是原始通知及完整行，固件peerReceipt仍unknown，不将这两次观察升级为网络保证。

The two actual WTERM receipts use lifecycle L, not C, and are complete raw ATT ASCII lines without a G/scientific CRC. Their immutable request identity/result/times match later USB queries. Publication counters differ because the wire record captures the pre-submission snapshot; this is neither stale outcome nor reapplication. Original notification bytes prove these two PC receipts, while firmware peerReceipt remains unknown and no delivery guarantee follows.

## build06 typed metadata兼容边界 / Typed metadata compatibility

本轮新增内部FramePublicationMetadata/PublishFrameMetadata/ApplyFrameMetadata只传成功encoder所配的不可变配置，原wire/CRC/invalid/SHORT字节不变；旧CAP gen/rid属ROWS、V/R属MODE、MIXED行/profile域不统一数值，revision0保留旧扩展省略规则。raw/旧API仍解析；typed POD不作为任意输入完整性认证。build05的内部工作副本已撤回。新ELFc47a3710…的[screen21](../../README.md#screen21-typed-metadata-results)仅1.828794%正常缺口数值通过，仍OBSERVED_NOT_ACCEPTED；不能拼接旧build04/05计数或把BLESNAP保护age作全部完整帧周期。

The new internal typed-metadata path carries immutable configuration paired with a successful encoder result; wire/CRC/invalid/SHORT bytes stay unchanged. Legacy CAP remains ROWS-owned, V/R MODE-owned and MIXED row/profile domains distinct, including revision-zero omission. Raw/legacy APIs still parse; typed POD is not arbitrary-input integrity proof. Build05's working copy is removed. [Screen21](../../README.md#screen21-typed-metadata-results) on the new ELF meets only the1.828794% normal gap dimension, retaining OBSERVED_NOT_ACCEPTED. Do not splice older image counters or treat protected-only age as the all-full period.

## build05对照不改变契约 / Build05 comparison leaves the contract unchanged

build05的单BLE owner内部2,304 B工作区不改wire/CRC/identity、查询字段、source/NET/BLE槽数或due/TTL；原slot等真实send返回，SDK超时仍不等于取消。新session B73989CDD936FA54的[screen20](../../README.md#screen20-working-copy-results)仍FAIL：7461完整BLE帧缺202，202/7663=2.636043%>2.5%。`BLEFLOW.stage=pressure`也可保持`ratePm=1000`且intentionalSkip0，Host不能把pressure自动解释成已降输出或弱RF。build04的SHORT/unread数据保持其原镜像身份；不能把其PC序号/周期/恢复记录接到build05连续窗口。

Build05's 2,304-byte single-owner internal working copy changes no wire/CRC/identity, query fields, slot counts or due/TTL semantics. Original-slot ownership lasts until real send return; timeout still is not cancellation. New-session [screen20](../../README.md#screen20-working-copy-results) remains FAIL at 2.636043% missing complete BLE observations. `BLEFLOW.stage=pressure` may retain `ratePm=1000` and zero intentional skips, so hosts cannot equate pressure with rate reduction or weak RF. Build04 SHORT/unread observations retain their original image/session; their sequences, periodic evidence and recovery cannot be spliced into a continuous build05 window.

## SHORT实测分母 / SHORT observation denominator

build04 [CAP8 SHORT专项](../../README.md#short-snapshot-results)在120.000123756秒收到118个BLE完整快照，PC最大相邻完整间隔1.028126934秒；USB1,416完整帧连续、118公共原包一致。普通预览没有完整K语义，解析器保留的missing1287不能作为普通FULL丢包率套2.5%门限。`BLESNAP`本地+118仍非应用ACK；历史maxCompleteGapUs不等于精确捕获间隔。CAP8在历史TX恢复前已查询，TX=REL恢复由真实ACK及TX?核验；没有补造第二finished或恢复后CONFIG。整体性能和弱链路保证仍未因此通过。

The [CAP8 SHORT run](../../README.md#short-snapshot-results) receives 118 complete BLE snapshots over 120.000123756 seconds, with a maximum observed adjacent full interval of 1.028126934 seconds. USB receives 1,416 consecutive full frames and all 118 common original packets match. Ordinary previews lack full-K semantics: retained missing1287 cannot be treated as normal FULL transport loss against the 2.5% budget. Local BLESNAP +118 is not an application ACK, and historical maxCompleteGapUs is not the exact capture interval. CAP8 was queried before the historical TX restoration, which is independently verified through real ACK/TX? bytes; no second finished or later CONFIG is fabricated. Overall and weak-link acceptance remain incomplete.

## USB unread：缓存与接收语义 / Buffered USB and receipt semantics

build04的[120.000367295秒应用不读专项](../../README.md#usb-unread-results)在同一formal CDC0 FD上零read/write，BLE继续接收7,821完整帧、65.177392 FPS、25/7,846=0.318634%缺口；USBQ同时dDrop+5,586、expired+2,240、partial+1，控制drop/expiry增量0。USBQ是独立查询窗，状态mounted/dma不等于PC已收。Linux仍可缓冲，恢复后读到的旧帧保留原identity/timestamp，**不能当作hold时刻USB实收或用当前配置重新盖章**。USB hold FPS为unknown；CAP8恢复查询成功仅证明后续命令可完成，不证明hold中无人读取时控制回复必达。不是物理拔线/Windows枚举验收，也不覆盖screen19普通FULL失败。

Build04's [120.000367295-second unread-client stress window](../../README.md#usb-unread-results) performs no application reads/writes on the formal CDC0 FD while BLE continues receiving 7,821 whole frames at 65.177392 FPS, with 25/7,846 missing observations. USBQ separately records DATA drops/expiry/partial writes while control drops/expiry stay unchanged. Mounted/DMA state is not PC receipt. Linux may still buffer; older frames drained afterward retain original identity/timestamps and **cannot become hold-time USB receipt or acquire the current configuration's labels**. Hold-time USB FPS is unknown. Later CAP8 query success proves later completion, not guaranteed control delivery while unread. This is not physical-unplug/Windows-enumeration qualification and does not override screen19's normal FULL failure.

本专项收尾以[analysis02与独立复审](../audit/artifacts/cdc_recovery_acceptance_20261116/usb-unread-independent-review-02.json)为准：hold raw/offset/最终finished与cleanup绑定、每个query回复来自对应write完成后新读入字节；postprobe summary绑定完整原始raw/events及26条query，不能只信缓存的fields或来自hold前的读数。只读postprobe没有最终finished，明确false、不补造；初版反例/analysis01仍保留。离线10项测试通过与数值不变仅提高证据可信度，不扩大实机验收范围。

Use [analysis02 and its independent review](../audit/artifacts/cdc_recovery_acceptance_20261116/usb-unread-independent-review-02.json) for this closure: hold raw/offsets/final-finished/cleanup are bound, and each reply comes from bytes newly read after its corresponding completed write. The postprobe summary is bound to original raw/events and all 26 queries; cached fields or pre-hold observations cannot suffice. Its read-only collector has no final finished event, explicitly false rather than fabricated. Initial counterexamples/analysis01 remain. Ten passing offline tests and unchanged numbers strengthen evidence binding without extending hardware qualification.

## screen19：protected-only字段的实机反例 / Hardware example of protected-only fields

build04 / session `B4828F8EF45001F1` 的120.000139309秒VOLT1已闭合，**FAIL_BLE_THROUGHPUT**：USB7,728内部连续，BLE7,481缺250，250/7,731=**3.233734% > 2.5%**。BLE端点973–8703，USB976–8703，不能用USB数量代BLE分母。两路7,479公共原包字节相同；wire/ASCII CRC/identity错误0，仍保留502/471个invalid点。ADC独立119.264441785秒窗0/82,589 checked reads，与完整帧CRC和点有效性分离。

Build04/session `B4828F8EF45001F1` closes a 120.000139309-second VOLT1 window with **FAIL_BLE_THROUGHPUT**: USB has 7,728 internally consecutive frames; BLE has 7,481 plus 250 missing observations, **3.233734% > 2.5%**. BLE endpoints 973–8703 differ from USB 976–8703, so USB's count is not the BLE denominator. All 7,479 common original packets match, with zero wire/ASCII-CRC/identity errors and 502/471 invalid USB/BLE points preserved. ADC independently has 0/82,589 checked-read rejections over 119.264441785 seconds, separate from frame integrity and point validity.

`BLESNAP`在自己的约120.50–120.54秒查询窗queued/localComplete/staleNormal/deadlineMiss均+0，lastSeq609、protected age125,788,271µs、gapKnown0；PC却持续收到完整帧，相邻最大42.942640ms。这是**禁止用保护age当all-full周期**的实际例子；完整包含Xhh，间隔不覆盖捕获边缘或保证指定保护ticket。SNAPSRC周期+119、SNAPNET候选+124/derived+5/retry+3/obsolete+8，与BLE保护分类不要求一一对应。BLEFLOW attempts+7,738/capacityReject+228、intentionalSkip0、采样端点FULL1000‰；剩余容量压力不能从250中抵扣或认定RF丢包。

In its separate query bracket, BLESNAP adds zero queued/localComplete/staleNormal/deadlineMiss, retaining lastSeq 609, protected age 125,788,271 µs and unknown protected gap. The PC nevertheless continues receiving full frames, with a maximum observed adjacent interval of 42.942640 ms. This directly demonstrates why **protected age is not an all-full-period metric**. Complete frames include Xhh; intervals exclude capture edges and do not guarantee a particular protected ticket. Source periodic +119 and Transport candidates +124/derived +5/retry +3/obsolete +8 need not match BLE protected classification. BLEFLOW adds 7,738 attempts and 228 capacity rejections, zero intentional skips and sampled FULL 1000‰. Remaining pressure cannot be subtracted from 250 or labeled RF loss.

收尾原子CONFIG恢复CAP8 rev6/boundary8741、同BUILD/session；无PC ACK、first-config/SHORT/弱链路保证仍未验收。完整数值、独立CPU/内存/driver区间、命令退出码与原件归档见[README](../../README.md#screen19-coalescing-results)、[同criteria补充](../audit/artifacts/cdc_recovery_acceptance_20261116/screen-19/screen-19-criteria-v2-supplement.json)。保留旧screen18 FAIL，不用新host通过或局部周期修复覆盖吞吐失败。

Closing atomic CONFIG verifies CAP8 revision 6/boundary 8741 under the same BUILD/session. There is no PC ACK, and first-configuration, SHORT and weak-link guarantees remain unaccepted. [README](../../README.md#screen19-coalescing-results) and the [unchanged-criteria supplement](../audit/artifacts/cdc_recovery_acceptance_20261116/screen-19/screen-19-criteria-v2-supplement.json) retain rates, separate CPU/memory/driver windows, command exit codes and original-file archives. Earlier screen18 failure remains; host success and repaired cadence do not override failed throughput.

<a id="ble-periodic-coalescing-v1"></a>

## build04兼容边界：protected-only查询与内部full依据 / Protected-only queries and private full evidence

build04的wire、PROTO2、ble_flow_v1/ble_snapshot_v1字段定义不变。BLE本轮新增内部 `sensorarrayBleFrameFullCompletedLocked`：当前connection/stream、同session/boot/config、seq相对上次成功full前进且时间不倒退的typed完整包本地完成，才重置1秒周期；普通FULL可满足，SHORT预览/raw旧API/失败或部分发送/旧stream或config不能满足。first-config独立；上游PERIODIC只保护source/network候选通路，BLE可合并已有义务，将该候选按普通FULL/SHORT规则处理。已owned对象不因合并取消，合并本身不丢弃所提供的帧，暂停不复制旧数据。

Build04 retains wire bytes, PROTO2 and existing ble_flow_v1/ble_snapshot_v1 field definitions. New private full-completion evidence accepts only successful typed complete bundles in the current connection/stream and same session/boot/configuration, with advancement from the last successful full sequence and a nonregressing clock. Ordinary FULL qualifies; SHORT previews, legacy raw calls, failed/partial sends and stale streams/configurations do not. FIRST remains separate. Upstream PERIODIC protects a source/network candidate; BLE may coalesce that obligation and use ordinary FULL/SHORT handling. Accepted owners are not canceled, coalescing itself does not discard the offered frame, and paused acquisition replays no old data.

`BLESNAP`仍仅计BLE保护分类，`lastCompleteAgeUs/maxCompleteGapUs/known/gapKnown`并未扩展成all-full或PC周期。正常FULL可能仅first计保护，之后age增长也能有持续完整数据；Host必须以实际CRC/identity完整K接收时间判周期，并保留边缘未观测、invalid/Xhh和无ACK边界。`SNAPNET.bleAccepted`仍为Transport保护候选的BLE准入数，**不保证BLE最终保护分类**；不可要求与BLESNAP.queued一一对应。NET原候选fence仍可能显式淘汰旧普通帧。上述变化不授权重解释旧gen/rid或本地失败计数。

BLESNAP remains restricted to BLE's protected class: lastCompleteAgeUs/maxCompleteGapUs/known/gapKnown are not all-full or PC-period metrics. Normal FULL may protect only FIRST while subsequent complete data keeps flowing. Hosts must use actual CRC/identity-valid completing K receipt times, retaining edge uncertainty, invalid/Xhh and no-ACK limits. SNAPNET.bleAccepted counts BLE admission of Transport protected candidates, **not guaranteed final BLE protected classification**; no one-to-one BLESNAP.queued relation exists. The original NET candidate fence may still discard obsolete ordinary frames explicitly. No legacy generation/domain or failure-counter meaning changes.

旧真实源码的200候选/10秒错相host反例已[实际失败](../audit/artifacts/cdc_recovery_acceptance_20261116/ble-periodic-coalesce-before-01.json)，修后FULL1保护/0stale/200完整、SHORT10保护/9stale有[独立执行与审查](../audit/artifacts/cdc_recovery_acceptance_20261116/ble-periodic-coalesce-independent-review-01.json)；这不能倒扣screen18的228缺口。运行身份a6b0…ELF/sessionB4828F8EF45001F1已[READY1核验](../audit/artifacts/cdc_recovery_acceptance_20261116/ble-snapshot-04-boot-verification.json)，screen19已闭合且正常缺口预算仍FAIL。完整机制/哈希/测试见[README](../../README.md#ble-periodic-coalescing)。

The old actual-source phased 200-candidate/10-second [host counterexample fails](../audit/artifacts/cdc_recovery_acceptance_20261116/ble-periodic-coalesce-before-01.json). [Independent corrected-source execution/review](../audit/artifacts/cdc_recovery_acceptance_20261116/ble-periodic-coalesce-independent-review-01.json) records FULL 1 protected/0 stale/200 complete and SHORT 10 protected/9 stale. This cannot retroactively subtract screen18's 228 gaps. Running image a6b0…/session B4828F8EF45001F1 is [READY1-verified](../audit/artifacts/cdc_recovery_acceptance_20261116/ble-snapshot-04-boot-verification.json), with screen19 now closed and still failing the normal gap budget. See [README](../../README.md#ble-periodic-coalescing) for mechanics, hashes and test boundaries.

## screen18：诊断计数与PC实收分离 / Local diagnostic counters versus PC receipt

同 `7ce5…` ELF/session `0C1E0125F83FF11C` 的120.000089528秒VOLT1实测为 **FAIL_BLE_THROUGHPUT**：BLE7334完整可信帧、228个缺口，**228/(7334+228)=3.015075% > 2.5%**；USB7562帧内部连续，7,333公共原包字节相同。计数不因新增保护能力而改变分母，不从缺口中减本地stale/drop。PC相邻同身份完整K最大间隔42.185839ms只覆盖已观察帧间；完整仍包含442个BLE X12，不保证所有点有效、指定保护ticket送达或弱链路1Hz。完整性能/窗口/归档见[README](../../README.md#screen18-protected-results)和[criteria补充](../audit/artifacts/cdc_recovery_acceptance_20261116/screen-18/screen-18-criteria-v2-supplement.json)。

The same `7ce5…` image/session `0C1E0125F83FF11C` completes 120.000089528 seconds of VOLT1 and **fails BLE throughput**: 7,334 trusted complete BLE frames and 228 missing observations, **3.015075% > 2.5%**. USB has 7,562 internally consecutive frames; 7,333 common original packets match. Protection does not redefine the denominator or justify subtracting local stale/drop counters. The maximum 42.185839 ms between observed same-identity completing K receipts includes 442 invalid X12 points across the capture; it proves neither all-point validity, a particular protected ticket's receipt nor weak-link 1 Hz delivery. [README](../../README.md#screen18-protected-results) and the [supplement](../audit/artifacts/cdc_recovery_acceptance_20261116/screen-18/screen-18-criteria-v2-supplement.json) retain full windows/results/archive.

四个cap已协商查询都保留自己的Host区间；SNAPSRC周期+119、SNAPNET保护queued/bleAccepted+129且derived+10、BLESNAP queued/localComplete+239且staleNormal+118、BLEFLOW capacityReject+99，intentionalSkip/NETpolicySkip为0、两端FULL1000‰。这不是一套可相加或相减的计数；screen18闭合时的相位假设随后已用真实源码host复现并修复，仍不能归因全部缺帧。`BLESNAP.failed=0`也不能推算depth为0或PC无缺口。收尾CAP8 rev6/boundary8761可按原子CONFIG重建；首次切换/SHORT/弱链路保护验收仍待运行，不能由状态查询成功推定。

Each negotiated query retains its own host interval. Source periodic admissions increase 119; Transport protected queued/BLE admissions increase 129 with 10 derived; BLE protected queued/local completions increase 239 with 118 stale-normal; capacity rejections increase 99. Intentional/NET policy skips are zero, with sampled FULL 1000‰ endpoints. These counters are not additive or mutually subtractable; the initial phase hypothesis is subsequently reproduced and repaired using actual-source host timing, without attributing every missing frame to it. `BLESNAP.failed=0` proves neither zero queue depth nor loss-free PC receipt. Closing atomic CONFIG reconstructs CAP8 revision 6/boundary 8761. First-transition, SHORT and weak-link protection remain untested rather than inferred from query success.

<a id="protected-snapshot-v1"></a>

## 保护快照 v1 契约 / Protected-snapshot v1 contract

本轮新增 typed publication 与四个只读查询；**保持 PROTO2，显式新增 `ble_flow_v1`、`ble_snapshot_v1` 能力，不改变 C/V/R/M/MR/K 或 G envelope 字节和 CRC 定义**。先用 PROTO caps gate：`ble_flow_v1` 对应 `BLEFLOW?`；`ble_snapshot_v1` 对应 `BLESNAP?`、`SNAPSRC?`、`SNAPNET?`，回复版本均为 `v=1`。缺少对应cap或明确 UNKNOWN/unsupported → unavailable；已协商查询没有回复 → unknown，均不得补成零。没有新 setter。Host 应保留未知扩展字段并按版本解析，不以日志缺失推定采集停止。

This change adds typed publication and four read-only queries. **It retains PROTO2 and explicitly advertises `ble_flow_v1` and `ble_snapshot_v1`, changing neither C/V/R/M/MR/K nor G-envelope bytes or CRC definitions.** Gate queries using PROTO caps: `ble_flow_v1` covers `BLEFLOW?`; `ble_snapshot_v1` covers `BLESNAP?`, `SNAPSRC?` and `SNAPNET?`. Replies use `v=1`. Missing corresponding capability or explicit UNKNOWN/unsupported means unavailable; an unanswered negotiated query means unknown, never zero. There is no new setter. Hosts should retain unknown extension fields and parse by version without inferring stopped acquisition from missing logs.

`gen/stream` 是 BLE connection/effective-DATA-stream 域，与 mode/rows/profile generation 不同。取消有效订阅、重订阅或 FAST/SAFE 有效资格变化会隔离旧 stream；即使连接没断也不能把旧完成用于新义务。完整源帧在采集开始捕获不可变 session/boot/revision/mgen/mrid/rgen/rrid/pgen/prid/sequence；各层只携带此身份，不在发送时查询最新配置。CAP 的旧 gen/rid 仍来自 rows 域，V/R 来自 mode 域，M 的 rgen/pgen 保持原定义。跨 boot/session 不比较大小；同 boot 的有限 sequence 用半范围规则，generation 只在其域内验证。恢复配置用原子 `CONFIG?` 和原 applied boundary，不以 MACK 或某一个 gen 数值替代。

`gen/stream` belong to BLE connection/effective-DATA-stream domains, distinct from mode/rows/profile generations. Effective unsubscribe/resubscribe or FAST/SAFE eligibility changes isolate the previous stream even without disconnecting. A stale completion cannot satisfy a new obligation. A full source frame carries immutable session/boot/revision/mgen/mrid/rgen/rrid/pgen/prid/sequence captured before acquisition; later layers never query current configuration to relabel it. Legacy CAP gen/rid remains rows-owned; V/R remains mode-owned; M retains its rgen/pgen definitions. Never numerically compare identities across boot/session boundaries. Same-boot finite sequences use serial-number half-range rules; generations are checked only in their own domains. Reconstruct configuration with atomic `CONFIG?` and applied boundaries, not MACK or one generation number.

| 查询 / Query | 字段 / Fields | 解释 / Meaning |
|---|---|---|
| `BLEFLOW` | `scope=local_output,counters=boot,gen,stream,active,stage` | 本地输出与有效订阅；stage=inactive/full/pressure/limited，不是 RSSI 分类。 / Local output and subscription, not RSSI classification. |
| `BLEFLOW` | `ratePm,intentionalSkip,probeSelected` | 普通选择比例‰；后两项为 boot uint64 模计数。探测计入 ordinary，不能用 ratePm 推算精确帧数。 / Ordinary permille rate and wrapping boot counters; probes are included in ordinary selections. |
| `BLEFLOW` | `attempts,capacityReject,evidence,rejectPm,pressureWindows,cleanUs` | attempts/capacityReject 为 boot uint32；排除主动跳过，capacity 仅容量原因。其余为当前证据窗/gauge，`evidence=0` 时不能把 rejectPm 当当前有效观察。 / Attempts exclude intentional skips; capacity identifies capacity causes only. Other fields describe current evidence, not independent totals. |
| `BLEFLOW` | `localServiceMilliFps,probeUs,cleanTtlUs,peerLoss,rssi` | 完整 message 的本地完成率，1000=1 FPS；无证据时 unknown。probeUs=1000000、cleanTtlUs=3000000；peerLoss/rssi=unknown。 / Local whole-message rate, not peer receipt; absent evidence remains unknown. |
| `BLESNAP` | `scope=local_complete,counters=boot,gen,stream,queued,localComplete,failed` | 保护快照的本地阶段计数，uint32 boot 模回绕。failed 只含部分发送终态，不含所有 stale/cancel/替代/drop；三项不能相减得到队列深度。 / Protected local counters; failed is not exhaustive and their difference is not queue depth. |
| `BLESNAP` | `admitMiss,deadlineMiss,superseded,staleNormal` | 分别为保护准入缺失、周期 due 未完成、排队替代、被完整快照超越的普通旧帧；可能重叠，不加总成 PC 丢包。 / Admission misses, missed periodic obligations, replacement and overtaken normal frames; not additive peer loss. |
| `BLESNAP` | `known,lastSeq,lastRev,lastCompleteAgeUs,gapKnown,maxCompleteGapUs` | BLE保护分类完整本地完成的身份/年龄/最大间隔，时间单位µs；不是内部all-full或PC周期。未知输出unknown。 / Identity/age/gap of protected-class local completion only, not the private all-full anchor or PC period; unknown remains explicit. |
| `BLESNAP` | `firstPending,periodicPending,dueUs=1000000,pcReceipt=unknown` | 义务状态；1秒是到期调度，不保证本地/PC在1秒完成。 / Obligation status; one second is a due interval, not a completion deadline. |
| `SNAPSRC` | `scope=boot,queued,firstQueued,periodicQueued,candidateMiss,firstMiss,superseded,firstSuperseded` | source保护槽 uint32 计数；同帧可以同时 first/periodic。miss 后可保护后续帧，不声称原首帧已送达。`stage=source_queue`。 / Source reservation counters, with overlapping first/periodic flags and explicit missed candidates. |
| `SNAPNET` | `scope=boot,readout=sequential,queued,admissionMiss,firstMiss,derived,superseded,retry` | Transport保护队列；derived 表示本层补充保护义务，retry 是保留待重试次数，不是 PC 重传次数。 / Transport protection; derived adds a local obligation, retry counts retained retries, not peer retransmissions. |
| `SNAPNET` | `expired,bleExpired,wifiExpired` | expired 每个到期保护 network descriptor 一次；两 sink 子计数可重叠。BLE须为入队时有效目标且未admitted；Wi-Fi须当时enabled/ready且从未attempted。已尝试Wi-Fi不重复计数，BLE未启用不记假timeout。 / Per-descriptor TTL plus potentially overlapping captured-sink expiries; not additive total frame loss. |
| `SNAPNET` | `bleAccepted,bleFailed,policySkip,obsolete,stage=ble_admission,pcReceipt=unknown` | Transport保护候选获BLE准入/部分失败，最终BLE可归普通；policySkip/obsolete仍为各自本层含义。不是SDK完成或PC确认，bleFailed非所有失败总数。 / BLE admission of Transport protected candidates, possibly into ordinary handling; selected failures/skips/obsolete counts are local, not completion or receipt. |

source/Transport 查询本身没有完整 session 字段；应与 BUILD/CONFIG 身份、主机查询时间一起保存。SNAPNET 两份本地统计分开读取，四个查询也不是跨层原子快照。比较增量时核验同 boot/session、同连接/stream、同配置与窗口；查询期间身份变化则重取，不拼接不同实例。boot 总计按各字段 uint32/uint64 无符号模差处理，超过半范围或身份缺失视为不确定。scope/counters=boot 不表示连接重置时计数全部归零。

Source/Transport replies do not themselves carry complete session identity; retain BUILD/CONFIG identity and host query timestamps with them. SNAPNET reads two local snapshots sequentially, and the four queries are not one cross-layer atomic snapshot. Validate boot/session, connection/stream, configuration and observation windows before comparing deltas; retry if identities change. Use unsigned uint32/uint64 modular deltas within a verified boot, treating ambiguous range/identity as unknown. Boot scope does not promise all counters reset on reconnect.

正常 full 策略全部选择普通候选；2.5% 不转成主动丢弃。容量拒绝严格 >20% 的连续3个有效1–2秒窗后，每至少3秒降低50‰；累计10秒有效clean证据再提高50‰。无观察不当成功，clean TTL为3秒；受限时每秒至多一个真实帧探测。完整周期默认1秒；build04以同身份且seq前进的最近成功typed完整帧为依据，普通FULL可满足，逾期才保护下一份真实encoder包；不会回放旧数据。first/周期原包包括全部活动行和invalid/Xhh，SHORT普通CAP预览不能冒充full。源、Transport、BLE各保留2槽，实际最早候选无法保存时记miss；软件优先级不证明PC收件。

Normal full policy selects every ordinary candidate; the 2.5% allowance never becomes deliberate discard. Three valid 1–2-second windows strictly above 20% capacity rejection permit 50‰ reductions at intervals of at least three seconds. Ten accumulated clean seconds permit a 50‰ increase. Missing observation is not success; clean evidence has a three-second TTL, and limited output allows at most one real-frame probe per second. From build04, advancing typed-full local completion, including ordinary FULL, anchors periodic due one second later. An overdue obligation protects the next available real encoder packet, never a replay. First/periodic packets include every active row and invalid/Xhh values; an ordinary SHORT CAP preview is not full. Each layer reserves two slots and counts missed earliest candidates. Scheduling priority does not establish PC receipt.

BLE CTRL/关键生命周期优先，连续控制可使 firstPending 长时间等待；周期 deadlineMiss 为实际逾期计数，不能声称1Hz必达。Transport保护network descriptor另有从入队起1秒共享TTL：每次dispatch先有界回收已取得所有权的到期pending/retry，保留原FIFO存活项，不调SDK/lwIP。该规则同时处理尚未提交的BLE/Wi-Fi副本；USB源流独立。BLE接收保护原包与普通预览的同步typed API会深拷贝实际选中包，入队ESP_OK仅代表BLE本地接管。无PC应用ACK，不提供网络exactly-once或绝对送达；500ms SDK guard仅隔离未知完成对象，不能取消。现有Wi-Fi MSG_DONTWAIT内部可能等待SDK mailbox/sem，未证明取消或HIL恢复。

BLE CTRL/critical lifecycle has priority, so continuous control can leave firstPending waiting. Periodic deadlineMiss counts actual missed obligations rather than proving 1 Hz delivery. Separately, protected Transport network descriptors have a shared one-second enqueue TTL: each dispatch reclaims only clearly owned expired pending/retry entries, restores surviving FIFO entries, and calls neither SDK nor lwIP. This applies to unsubmitted BLE/Wi-Fi copies; USB source output is independent. BLE's typed API synchronously deep-copies the selected original-full or ordinary-preview packet. ESP_OK means local BLE admission only. There is no PC application ACK, network exactly-once or guaranteed delivery. The 500 ms SDK guard quarantines unknown completion without cancellation. Existing Wi-Fi MSG_DONTWAIT may still wait in SDK mailboxes/semaphores; cancellation and HIL recovery are unproven.

内存修复镜像build03的只读 `MEM?` 另由 `heap_v1` gate；575c…失败镜像不含此能力。`MEM,v=1,readout=sequential,unit=bytes,intCaps=internal8` 的 `intFree/intLargest/intMin` 是当前内部8-bit heap观察，`extFree/extLargest`是外部PSRAM观察；`scan=unknown/pending/created/failed,scanStack,beforeFree,beforeLargest,afterFree,afterLargest,err`保存scan-task创建前后记录，未知输出unknown。`sampleUs`为启动以来µs，heap遍历/格式化在snapshot锁外，多个读数不同时、before/after差值不等于该task独占分配。此低频查询不在采集热路径，不把PSRAM总量当DMA/内部栈资格。build03的[实际启动MEM值](../audit/artifacts/cdc_recovery_acceptance_20261116/ble-snapshot-03-boot-verification.json)已记录，不能倒填575c失败时刻。

Memory-correction image build03 separately gates read-only `MEM?` with `heap_v1`; failed image 575c… lacks it. `MEM,v=1,readout=sequential,unit=bytes,intCaps=internal8` reports current internal-8-bit `intFree/intLargest/intMin`, external-PSRAM `extFree/extLargest`, and saved scan-creation `scan=unknown/pending/created/failed,scanStack,beforeFree,beforeLargest,afterFree,afterLargest,err`. Unknown records remain unknown. `sampleUs` is microseconds since boot. Heap traversal/formatting stays outside the snapshot lock; reads are sequential, and before/after differences are not an exclusive task allocation measurement. The low-frequency query is outside acquisition's hot path and does not treat PSRAM totals as DMA/internal-stack eligibility. Build03 has [actual boot MEM observations](../audit/artifacts/cdc_recovery_acceptance_20261116/ble-snapshot-03-boot-verification.json); these cannot be backfilled into the failed boot.

wifi-memory-01在同一`MEM,v=1`中追加可选`poolReady,poolErr,poolInt,poolExt`：Transport永久CPU文本池是否成功绑定、原始初始化错误、内部pool对象/状态的sizeof字节、已绑定外部payload字节。它们不是全Transport heap/allocator开销、DMA资格或AP就绪证明；旧镜像缺字段时保持unknown。实录为1/0/1,157/16,135 B，394 B最大回复经host验证（含LF、加NUL为395/512）。PSRAM块只在初始化申请，无内部fallback/热路径申请/未知owner回收；现有7个固定payload槽和独立512 B控制保留不变。

Wifi-memory-01 adds optional poolReady/poolErr/poolInt/poolExt to MEM v1: successful permanent CPU text-pool binding, its original init error, internal pool/state sizeof bytes and bound external payload capacity. These are not total Transport heap/allocator cost, DMA eligibility or AP readiness; absent fields on older images remain unknown. The observed values are 1/0/1,157/16,135 bytes. The tested maximum reply is 394 bytes including LF, 395 with NUL in 512. Allocation is initialization-only without internal fallback or reclaiming unknown-owned storage; seven fixed payload slots and the separate 512-byte control reservation remain unchanged.

实现与新候选状态见 [README](../../README.md#ble-protected-output)；旧 screen17/600秒实录及FAIL仍按下面各自镜像解释，不能作为新能力的HIL验收。

See the [README](../../README.md#ble-protected-output) for implementation and candidate status. Existing screen17/600-second recordings and FAIL verdicts below remain scoped to their own images, not HIL qualification of these new capabilities.


## screen17 / BLELOG v1：已观察到的普通日志准入策略 / Observed ordinary-log admission policy

当前镜像完整 ELF **`f1ec518dfba78453dcd7fe057bba29eb61a8f8898274a21e22125df26b933a04`**，BUILD 仍 `8a1794c-dirty`，session `863626B103487FE5` / boot1。[screen17](../audit/artifacts/cdc_recovery_acceptance_20261116/screen-17/screen-17-verdict.json)的120.000057708秒 VOLT1 行快照为 rev4、mgen3/mrid4、rgen2/rrid3、pgen3/prid4。USB 7,805 连续帧、BLE 7,776 帧缺29，Capture/USB/BLE64.978526/65.035014/64.800778 FPS；[criteria v2 补充](../audit/artifacts/cdc_recovery_acceptance_20261116/screen-17/screen-17-criteria-v2-supplement.json) **29/7,805=0.371557%**，仅正常链路数值预算通过。原 OBSERVED_NOT_ACCEPTED 保留，表示全体验收未完成。ADC 0/81,537属独立119.120057965秒健康窗；USB/BLE373/370个无效点保留，不能将“完整”改为“所有点有效”。

The current full ELF is **`f1ec518dfba78453dcd7fe057bba29eb61a8f8898274a21e22125df26b933a04`**, still BUILD `8a1794c-dirty`, session `863626B103487FE5`, boot 1. [Screen17](../audit/artifacts/cdc_recovery_acceptance_20261116/screen-17/screen-17-verdict.json) records 120.000057708 seconds of VOLT 1 row with immutable rev4, mgen3/mrid4, rgen2/rrid3 and pgen3/prid4. USB receives 7,805 consecutive frames; BLE receives 7,776 with 29 missing, at 64.978526/65.035014/64.800778 FPS capture/USB/BLE. The [criteria v2 supplement](../audit/artifacts/cdc_recovery_acceptance_20261116/screen-17/screen-17-criteria-v2-supplement.json) computes **29/7,805=0.371557%**, meeting only the normal-link numeric budget. Original OBSERVED_NOT_ACCEPTED remains because overall acceptance is incomplete. ADC 0/81,537 belongs to a separate 119.120057965-second health span. USB/BLE retain 373/370 invalid points; complete does not mean all-point validity.

本轮新增 `BLELOG?` 是只读 v1 query-presence probe，不新增 PROTO capability 或 setter。有效 DATA 订阅期间，新普通 LOG 被准入策略拒绝；CTRL 与 priority 生命周期绕过该策略，已接受/在途 LOG 不取消。`dataActive` 不是 DATA 队列非空或实测采集帧率，暂停采集但保留订阅仍可抑制普通 LOG；LOG-only 需取消 DATA 订阅。收不到普通 LOG 不代表采集已停；权威状态继续用原子 CONFIG/终态查询。

New `BLELOG?` is a read-only v1 query-presence probe, with no new PROTO capability or setter. Effective DATA subscription rejects new ordinary LOG admissions. CTRL and priority lifecycle bypass this policy; already accepted/inflight LOG is not canceled. `dataActive` does not mean a nonempty DATA queue or measured acquisition rate. Paused acquisition with the subscription retained may still suppress ordinary LOG; LOG-only operation requires DATA unsubscribe. Missing ordinary LOG does not establish stopped acquisition; authoritative recovery still uses atomic CONFIG/terminal queries.

| 字段 / Fields | 域与边界 / Domain and limit |
|---|---|
| `v=1,scope=boot,gen` | 查询版本、计数 boot 域、连接 generation；gen 不是 mode/profile/rows generation。 / Query version, boot counter scope and connection generation; not a configuration generation. |
| `connected,mode,dataCccd,dataActive` | 真实连接与 FAST/SAFE 下有效订阅条件；不证明已有 DATA 实收。 / Connection and effective FAST/SAFE subscription, not DATA receipt. |
| `ordinaryAllowed,policy` | 普通 LOG 当前准入资格；policy 为 drop_ordinary_when_data，不代表已经发送或空中容量测量。 / Ordinary LOG eligibility, not transmission or measured radio capacity. |
| `policyDrop,dropL` | boot 内 uint32 回绕；policyDrop 是 dropL 子集，普通日志的主动拒绝，不是 DATA 帧损失；不可相加。 / Boot-local wrapping counters; policyDrop is an ordinary-LOG subset of dropL, not DATA loss or additive counts. |
| `priorityBypass=1,inflightCancel=0` | 关键 priority 不受普通策略拒绝，已接受对象保留；不是无线保证或取消 SDK。 / Critical bypass and retained accepted objects, not guaranteed radio delivery or SDK cancellation. |
| `pcReceipt=unknown` | SDK/本地成功不等同 Host 完整 CRC/identity 帧确认。 / SDK/local success is not confirmed complete trusted Host receipt. |

[前](../audit/artifacts/cdc_recovery_acceptance_20261116/ble-log-policy-prescreen17-01.summary.json)/[后](../audit/artifacts/cdc_recovery_acceptance_20261116/ble-log-policy-postscreen17-01.summary.json)查询同session、连接gen1，均dataActive=1/ordinaryAllowed=0；policyDrop/dropL4→358，增量354属于 **184.153099360秒独立查询窗**，不是120秒精确窗口，也不应与PC缺29相加。guard均quarantined=0/rejected=0；新只读工具 `--ble-log` 实际发出该查询。收尾CONFIG为CAP8、rev6/boundary10462，mgen4/mrid5、rgen3/rrid6、pgen4/prid5，不能将之前rev4数据重新盖章。

[Before](../audit/artifacts/cdc_recovery_acceptance_20261116/ble-log-policy-prescreen17-01.summary.json)/[after](../audit/artifacts/cdc_recovery_acceptance_20261116/ble-log-policy-postscreen17-01.summary.json) queries share the session and connection gen=1, both with dataActive=1/ordinaryAllowed=0. policyDrop/dropL change 4→358; delta 354 belongs to an **independent 184.153099360-second query span**, not exactly 120 seconds, and must not be added to 29 missing PC observations. Guard remains quarantined=0/rejected=0. The new read-only `--ble-log` tool option actually issues this query. Closing CONFIG is CAP8, rev6/boundary10462, mgen4/mrid5, rgen3/rrid6 and pgen4/prid5; prior rev4 data must not be restamped.

[18个关闭原流归档](../audit/artifacts/cdc_recovery_acceptance_20261116/screen-17/archive-index-01.json)保留原件。首次verdict工具失败为boot JSON schema不匹配，原嵌套记录和失败log保留，同观测adapter后exit 0；不是协议字段或设备故障。新ELF不继承旧8e61…的600秒/50次切换，32配置、弱链路/RSSI适应、配置首完整帧与默认1Hz保护快照的新策略、SDK阻塞恢复、精度仍待实施或验收。原criteria修订生成时尚无本次screen17；当前实测以本节为准。

The [archive of 18 closed original records](../audit/artifacts/cdc_recovery_acceptance_20261116/screen-17/archive-index-01.json) retains originals. The first verdict tool failure is a boot JSON schema mismatch; the nested record/failure log remain and a same-observation adapter succeeds. This is neither a protocol-field nor a device fault. The new ELF inherits no 600-second/50-transition coverage from older 8e61…. The 32-configuration matrix, weak-link/RSSI adaptation, new first-configuration-frame/default 1 Hz snapshot policy, SDK-stall recovery and accuracy remain pending implementation or acceptance. Screen17 did not exist when the earlier criteria supplement was generated; this section is the current observation.

## criteria v2：传输缺口预算不是科学 CRC 宽限 / Transport-gap budget is separate from scientific CRC

[验收 criteria v2](../audit/artifacts/cdc_recovery_acceptance_20261116/acceptance-criteria-20261121-v2.json)是本次用户需求修订，**不是 PROTO 版本或新 capability**。正常 BLE 完整帧缺口约 2.5% 可接受；弱链路约 10–20%，更弱降低常规输出，采集独立。新增优先目标为关键控制/生命周期、新配置应用后的首个完整帧、周期完整快照（当前实现默认 1 秒，用户未固定周期）；弱链路/RSSI 分类、输出适应与保护策略尚实施中，不预告未实现的命令或帧字段。完整快照保留活动行的 invalid/Xhh，不等同首次全部有效测量，也不等同 MAPP 或 PC 已确认收到。

[Acceptance criteria v2](../audit/artifacts/cdc_recovery_acceptance_20261116/acceptance-criteria-20261121-v2.json) records the updated user requirement; it is **not a PROTO version or capability**. About 2.5% missing complete BLE frames is acceptable normally and about 10–20% on weak links, with reduced routine output on still weaker links and independent acquisition. New priority targets are critical control/lifecycle, the first complete frame after applying a configuration and periodic complete snapshots. The current implementation default is one second; the user did not prescribe the interval. Weak-link/RSSI classification, adaptation and protection remain in progress; no unimplemented command or frame field is promised. Complete snapshots retain invalid/Xhh points for active rows and do not mean a first all-valid measurement, MAPP or confirmed PC receipt.

[旧实录补充判定](../audit/artifacts/cdc_recovery_acceptance_20261116/stability-02/stability-02-criteria-v2-supplement.json)为 **421/(38,047+421)=1.094416% ≤ 2.5%**，只满足新正常链路数值预算。原 FAIL JSON/原流不改；原 8e61… ELF 的 600 秒记录不属于新烧录的 `f1ec518dfba78453dcd7fe057bba29eb61a8f8898274a21e22125df26b933a04` ELF，该次 criteria 补充生成时，后者仅有[BUILD/READY 核验](../audit/artifacts/cdc_recovery_acceptance_20261116/ble-log-policy-01-boot-verification.json)，本补充无其闭合 HIL。不得跨 session/配置/镜像拼接分母，不把 BLE 首末范围 38,468 改成 USB 的 38,467 或 SDK accepted。

The [supplemental old-recording assessment](../audit/artifacts/cdc_recovery_acceptance_20261116/stability-02/stability-02-criteria-v2-supplement.json) computes **421/(38,047+421)=1.094416% ≤ 2.5%**, meeting only the revised normal-link numeric budget. Original FAIL JSON/raw streams remain unchanged. The old 8e61… ELF’s 600-second recording does not belong to newly flashed ELF `f1ec518dfba78453dcd7fe057bba29eb61a8f8898274a21e22125df26b933a04`, which, when that criteria supplement was generated, had [BUILD/READY verification](../audit/artifacts/cdc_recovery_acceptance_20261116/ble-log-policy-01-boot-verification.json) but no closed HIL in this supplement. Denominators must not combine sessions/configurations/images or replace BLE’s 38,468 endpoint observations with USB’s 38,467 frames or SDK acceptance counts.

Host 需分开显示：①主动输出降帧（采集继续，未来须有明确策略/计数）；②本地队列/guard/策略拒绝；③PC 缺少 CRC/identity 正确的完整帧观察；④ADC SPI CRC 检查读取失败。主动跳过采集 seq 仍可出现在端到端缺口中，除非有同窗、同域的发送资格定义和统计，不得静默扣除。本地拒绝和 PC 缺口可能描述同一事件，不能求和。传输预算不是允许接收坏帧 CRC、填零、平滑、改标签或放宽 ADC 当前启动资格 2.5%；无效点保持无效。

The host must distinguish: (1) intentional output decimation while acquisition continues, requiring an explicit policy/counter; (2) local queue/guard/policy rejection; (3) missing CRC-and-identity-valid complete PC observations; and (4) failed checked ADC SPI reads. Intentionally skipped capture sequences can still appear in end-to-end gaps. Do not silently subtract them without a same-window, same-domain eligibility definition and measurements. Local rejection and a PC gap can describe the same event and must not be added. The transport budget permits neither bad frame CRC acceptance, zero-filling, smoothing or relabeling nor relaxation of the separate existing ADC startup 2.5% qualification. Invalid points remain invalid.

整体状态仍未验收：32 配置 NOT_RUN，弱链路/RSSI 与默认 1 Hz 保护快照的新策略待实现/实测，SDK 阻塞恢复及绝对精度未完成。优先调度不是无线 exactly-once 或绝对送达；本次 criteria 修订不增加网络保证。下文保留旧标准判定作为历史证据，当前数值预算解释以本节和补充 JSON 为准。

Overall acceptance remains incomplete: the 32-configuration matrix is NOT_RUN; new weak-link/RSSI and default 1 Hz protected-snapshot policies await implementation/measurement; SDK-stall recovery and absolute accuracy remain unaccepted. Priority scheduling is not wireless exactly-once or guaranteed receipt. This criteria revision adds no network guarantee. The original assessments below remain historical evidence; this section and the supplemental JSON govern the revised numeric-budget interpretation.

## stability-02：保留原判定及身份边界 / Preserved original verdict and identity boundaries

[stability-02](../audit/artifacts/cdc_recovery_acceptance_20261116/stability-02/stability-02-verdict.json)实际镜像为 `8a1794c-dirty` / 完整 ELF **`8e61aacaee859cd5ebe1dff13dff1d9d2c722268bb564837c15b4741613eb4fc`**、session `391F6A5399682456` / boot1。VOLT 1 行录制 600.000114995 秒，配置快照 rev59、mgen48/mrid59、rgen4/rrid58、pgen56/prid59；本 VOLT 帧旧 gen48/rid59 属于 mode 域，不把 CAP 的 rows 域旧 gen/rid 改成同义。USB seq12675…51141 连续 38,467 帧，BLE 38,047 帧缺 421；38,045 个公共帧原始字节相同，完整帧 CRC/identity/wire-contract 错误均 0。Capture/USB/BLE 为 64.110459/64.111810/63.412571 FPS，**仍 FAIL_BLE_THROUGHPUT**。

[Stability-02](../audit/artifacts/cdc_recovery_acceptance_20261116/stability-02/stability-02-verdict.json) runs `8a1794c-dirty`, full ELF **`8e61aacaee859cd5ebe1dff13dff1d9d2c722268bb564837c15b4741613eb4fc`**, session `391F6A5399682456`, boot 1. Its 600.000114995-second VOLT 1-row recording uses immutable rev59, mgen48/mrid59, rgen4/rrid58 and pgen56/prid59. Legacy gen48/rid59 belongs to the mode domain in these VOLT frames; CAP’s legacy rows-domain fields are not reinterpreted. USB receives 38,467 consecutive frames across seq12675…51141; BLE receives 38,047 with 421 missing observations. All 38,045 common frames have identical original bytes, with zero complete-frame CRC/identity/wire-contract errors. Capture/USB/BLE are 64.110459/64.111810/63.412571 FPS; status **remains FAIL_BLE_THROUGHPUT**.

ADC CRC0/413,595 属于独立 598.945560056 秒健康窗，不是帧分母；engineFrameCount=8512…46912、modeEntryFrameCount=8149、healthInterval=64 为实录。USB/BLE 的 2,717/2,644 个 X12 仍无效，完整帧接收与所有点有效必须分开。BL50 自身 598.946417553 秒窗 dropD +343/dropL +40/dropC +0，USBQ 自身 600.427256331 秒窗无 drop/expiry/partial 增量，BLETX 自身 600.456470292 秒窗 SDK accepted79,882/errors0；这些不同层和窗口不能相加或用 SDK 接受证明 PC 收齐。

ADC CRC 0/413,595 belongs to a separate 598.945560056-second health span, not a frame-count denominator. Actual counters are engineFrameCount=8512…46912, modeEntryFrameCount=8149 and healthInterval=64. USB/BLE retain 2,717/2,644 invalid X12 points, so complete-frame receipt stays distinct from all-point validity. BL50’s own 598.946417553-second span reports dropD +343/dropL +40/dropC +0; USBQ’s 600.427256331-second span has no drop/expiry/partial increments; BLETX’s 600.456470292-second span has 79,882 SDK acceptances and zero errors. These layers/windows must not be added, and SDK acceptance does not prove complete PC receipt.

[收尾 CONFIG?/BUILD?/BLEGUARD? 查询](../audit/artifacts/cdc_recovery_acceptance_20261116/ble-data-priority-poststability02-01.summary.json)确认同镜像/session、CAP8 rev61/boundary51176、mgen49/mrid60、rgen5/rrid61、pgen57/prid60，各终态为 applied；Host 应采用这份一致快照恢复，不能将此前 rev59 数据换成 rev61 标签。guard 为 ready1/quarantined0/rejected0/active0/completionKnown1/restartRequired0，ageUs=17,902,898 是上次调用起点年龄，active0 时不是阻塞时长。PROTO2/capabilities/科学 CRC 和字段域未变化；正常查询不证明 SDK 隔离/恢复故障路径已验收。

The [closing CONFIG?/BUILD?/BLEGUARD? query](../audit/artifacts/cdc_recovery_acceptance_20261116/ble-data-priority-poststability02-01.summary.json) confirms the same image/session, CAP8 rev61/boundary51176, mgen49/mrid60, rgen5/rrid61 and pgen57/prid60, with applied terminal results. The host should recover from this consistent snapshot without relabeling earlier rev59 data as rev61. Guard reports ready1/quarantined0/rejected0/active0/completionKnown1/restartRequired0. ageUs=17,902,898 is age since the previous call started, not blocked duration when active0. PROTO2, capabilities, scientific CRC and field domains remain unchanged; normal queries do not qualify SDK-containment/recovery fault paths.

覆盖内未观测 fault/reset/WDT，CAP8 收尾已确认。[7 个关闭原流](../audit/artifacts/cdc_recovery_acceptance_20261116/stability-02/archive-index-01.json)逐字节/SHA 回验并保留原件，[归档命令](../audit/artifacts/cdc_recovery_acceptance_20261116/stability-02-archive-01-command.json) exit 0。该 ELF 的 600 秒录制与 50 次 USB 身份切换已完成，**吞吐未通过、32 配置仍 NOT_RUN、绝对精度未知**。本记录结束时尚未烧录的普通 BLE LOG 抑制候选不属于该契约的 HIL 证据，也不构成独立上位机 UI 验收。

No fault/reset/WDT is observed within coverage, and CAP8 cleanup is confirmed. [Seven closed original streams](../audit/artifacts/cdc_recovery_acceptance_20261116/stability-02/archive-index-01.json) pass byte/SHA verification with originals retained; the [archive command](../audit/artifacts/cdc_recovery_acceptance_20261116/stability-02-archive-01-command.json) exits 0. This ELF’s 600-second recording and 50 USB identity transitions are complete, but **throughput fails, the 32-configuration matrix is NOT_RUN and absolute accuracy is unknown**. The ordinary-BLE-LOG suppression candidate, unflashed when this recording ended, is not part of this contract’s HIL evidence or acceptance of the separate desktop UI.

## 50次实机切换的字段域证据 / Field-domain evidence from 50 hardware transitions

同8e61…ELF/session391F6A5399682456的[50次USB切换](../audit/artifacts/cdc_recovery_acceptance_20261116/mode-switches-01/mode-switches-01.summary.json)已完成50/50，COMPLETE_50_IDENTITY_VERIFIED，全程14.213275秒、最长步骤0.520350秒。目标CAP/VOLT/RES/MIXED均为8行；每次以接受id、终态、原子CONFIG及应用boundary后的首个匹配CRC帧核对，再查closing CONFIG，未把MACK等同首个有效测量。全部50次实际有终态；终态丢失/早于终态数据的验证属于独立合成测试，不宣称实机注入。

The [50 USB transitions](../audit/artifacts/cdc_recovery_acceptance_20261116/mode-switches-01/mode-switches-01.summary.json) on the same 8e61… ELF/session391F6A5399682456 complete 50/50, COMPLETE_50_IDENTITY_VERIFIED, over 14.213275 seconds with a maximum step of 0.520350 seconds. CAP/VOLT/RES/MIXED all use eight rows. Each step verifies accepted ID, terminal result, atomic CONFIG and the first matching CRC frame at/after the applied boundary, then checks closing CONFIG; MACK is not equated with a first valid measurement. All 50 live steps receive terminals. Lost-terminal and data-before-terminal checks are separate synthetic tests, not claimed hardware injections.

最后CAP帧seq11789的旧gen3/rid6来自ROWS，显式mgen46/mrid56及rev56才对应本次MODE/完整配置；即使旧gen数字较小仍合法。另有1条旧rev26排队帧seq11728保留原身份/字节并计stale，不能用最新CONFIG重新盖章。50个匹配帧CRC/identity/boundary通过，**不要求所有点valid或acquired非零**；fault0/reset[]、CAP8收尾查询通过，精度未知。BLE旁路有原notification实录，但此结果不是BLE端50次身份验收。[独立审查](../audit/artifacts/cdc_recovery_acceptance_20261116/mode-switches-independent-review-01.json)与[原流归档](../audit/artifacts/cdc_recovery_acceptance_20261116/mode-switches-01/archive-index-01.json)保留证据及限制。

Final CAP frame seq11789 legitimately carries legacy gen3/rid6 from ROWS, while explicit mgen46/mrid56 and rev56 identify MODE/full configuration. Its smaller legacy generation is valid. One old-revision26 queued frame, seq11728, retains original identity/bytes and is counted stale; it must not be restamped from latest CONFIG. All 50 matching frames pass CRC/identity/boundary checks, **without requiring every point valid or a nonzero acquired mask**. Fault0/reset[] and query-verified CAP8 cleanup are observed; accuracy remains unknown. BLE notifications are recorded alongside USB, but this is not BLE-side 50-transition identity acceptance. The [independent review](../audit/artifacts/cdc_recovery_acceptance_20261116/mode-switches-independent-review-01.json) and [raw-stream archive](../audit/artifacts/cdc_recovery_acceptance_20261116/mode-switches-01/archive-index-01.json) preserve the evidence and limits.

## screen16：DATA 优先与普通LOG可延后 / screen16: DATA priority and deferrable ordinary LOG

本版实际ELF为 **`8e61aacaee859cd5ebe1dff13dff1d9d2c722268bb564837c15b4741613eb4fc`**、BUILD仍 `8a1794c-dirty`、session=`391F6A5399682456`，以[完整启动身份](../audit/artifacts/cdc_recovery_acceptance_20261116/ble-data-priority-01-boot-verification.json)区分旧同名镜像。69个host入口/build通过、187输入相同，变更只在BLE descriptor选择，PROTO2与科学encoder/CRC/identity不变。

This version’s actual ELF is **`8e61aacaee859cd5ebe1dff13dff1d9d2c722268bb564837c15b4741613eb4fc`**, with the same BUILD text `8a1794c-dirty` and session `391F6A5399682456`. Use [full boot identity](../audit/artifacts/cdc_recovery_acceptance_20261116/ble-data-priority-01-boot-verification.json) to distinguish older identically named images. The 69-entry host suite/build pass with 187 identical inputs. Only BLE descriptor selection changes; PROTO2 and scientific encoder/CRC/identity semantics remain.

选择顺序为 **保留CTRL/生命周期 → 已排队DATA → 普通LOG**。priority生命周期即使沿用LOG channel也不等同普通诊断LOG；普通LOG无最低服务频率保证，持续DATA积压时可延后或满槽计数拒绝。Host不能将普通日志暂缺当作采集已停，也不能依赖普通LOG按固定时序确认MAPP；权威恢复仍用既有终态/原子CONFIG和按需只读查询。队列内已选记录不被中途抢占；新CTRL可能需等当前记录收尾，SDK准入/CONF仍不代表PC完整接收。持续CTRL也可能延后DATA，不能从应用选择顺序推断硬实时或无线优先保证。

Selection order is **reserved CTRL/lifecycle → queued DATA → ordinary LOG**. A priority lifecycle record remains distinct from ordinary diagnostic LOG even when it uses the LOG channel. Ordinary LOG has no minimum service-rate guarantee: continuous DATA backlog can delay it or cause a counted full-slot rejection. The host must not infer stopped acquisition from absent ordinary logs or depend on fixed ordinary-LOG timing to confirm MAPP. Authoritative recovery still uses existing terminal results/atomic CONFIG and on-demand read-only queries. A selected record is not preempted midway; new CTRL may wait for its cleanup, and SDK admission/CONF still does not prove complete PC receipt. Sustained CTRL can also delay DATA, so application selection order is not a hard-real-time or radio-priority guarantee.

[screen16实录](../audit/artifacts/cdc_recovery_acceptance_20261116/screen-16/screen-16-verdict.json)120.000331701秒为Capture/USB/BLE64.24/64.12/63.41 FPS、USB7,695内部连续帧、BLE7,609帧缺84、ADC0/82,428，仍FAIL_BLE_THROUGHPUT。[post16](../audit/artifacts/cdc_recovery_acceptance_20261116/ble-data-priority-post16-01.summary.json)查询同身份CAP8及BLEGUARD ready1/quarantined0/rejected0/active0/completionKnown1；正常查询不替代guard触发/恢复测试。该ELF后续600秒录制及50次USB身份切换见上节；32配置仍NOT_RUN；不继承旧镜像的完整验收。

The [screen16 recording](../audit/artifacts/cdc_recovery_acceptance_20261116/screen-16/screen-16-verdict.json) covers 120.000331701 seconds at 64.24/64.12/63.41 FPS capture/USB/BLE, with 7,695 internally consecutive USB frames, 7,609 BLE frames and 84 missing observations, plus ADC0/82,428. It remains FAIL_BLE_THROUGHPUT. The [post16 query](../audit/artifacts/cdc_recovery_acceptance_20261116/ble-data-priority-post16-01.summary.json) confirms the same identity, CAP8 and BLEGUARD ready1/quarantined0/rejected0/active0/completionKnown1; normal queries do not replace guard activation/recovery testing. This ELF’s later 600-second recording and 50 USB identity transitions are recorded above. Its 32-configuration matrix remains NOT_RUN, without inheriting old-image acceptance.

## screen15：已观察到的 BLEGUARD 支持 / screen15: observed BLEGUARD support

已烧录镜像完整ELF **`48f7352f2f0f637ee4c0290a09f1141e0a2d6e5a7ad12442910b5132c5250116`**、BUILD `8a1794c-dirty`、session `FD6F81721A66C665` 与[69入口/187输入验证](../audit/artifacts/cdc_recovery_acceptance_20261116/ble-guard-cadence-01-validated-identity.json)及[启动核验](../audit/artifacts/cdc_recovery_acceptance_20261116/ble-guard-cadence-01-boot-verification.json)对应。screen15正常运行后，[USB只读BLEGUARD?](../audit/artifacts/cdc_recovery_acceptance_20261116/ble-guard-cadence-post15-01.summary.json)实际返回v1/ready1/quarantined0/rejected0/active0/completionKnown1/restartRequired0，证明该镜像查询支持；它不是故障触发或恢复演练。ageUs=65,820,515表示上次调用开始后的年龄，active0时不得当作65秒SDK阻塞。

The flashed image’s full ELF **`48f7352f2f0f637ee4c0290a09f1141e0a2d6e5a7ad12442910b5132c5250116`**, BUILD `8a1794c-dirty` and session `FD6F81721A66C665` match the [69-entry/187-input validation](../audit/artifacts/cdc_recovery_acceptance_20261116/ble-guard-cadence-01-validated-identity.json) and [boot verification](../audit/artifacts/cdc_recovery_acceptance_20261116/ble-guard-cadence-01-boot-verification.json). After normal screen15 operation, a real [USB read-only BLEGUARD? query](../audit/artifacts/cdc_recovery_acceptance_20261116/ble-guard-cadence-post15-01.summary.json) returns v1/ready1/quarantined0/rejected0/active0/completionKnown1/restartRequired0, establishing query support in this image. It is not a fault-activation or recovery exercise. ageUs=65,820,515 is age since the last call started; with active0 it must not be called a 65-second SDK stall.

[screen15](../audit/artifacts/cdc_recovery_acceptance_20261116/screen-15/screen-15-verdict.json)120.000115322秒的Capture/USB/BLE为63.99/64.02/63.25 FPS，USB7,682连续帧、BLE7,589帧缺96，ADC0/82,246、CAP8恢复已查询；仍FAIL_BLE_THROUGHPUT，未新设允许差距门限。debug汇总capture内154条SF50均n50（全observer168条），科学DATA逐帧契约不变。该ELF600秒/32配置、实机SDK卡死注入与隔离后恢复均NOT_RUN，独立桌面UI未因此完成验收。

[Screen15](../audit/artifacts/cdc_recovery_acceptance_20261116/screen-15/screen-15-verdict.json) records 120.000115322 seconds at 63.99/64.02/63.25 FPS capture/USB/BLE, with 7,682 consecutive USB frames, 7,589 BLE frames and 96 missing observations, ADC0/82,246 and query-verified CAP8 recovery. Status remains FAIL_BLE_THROUGHPUT without inventing an allowable-gap threshold. The capture contains 154 debug SF50 summaries, all n50 (168 across the whole observer); scientific DATA remains per-frame. This ELF’s 600-second/32-configuration runs, hardware SDK-stall injection and recovery after quarantine are NOT_RUN. This observation does not complete acceptance of the separate desktop UI.

## BLEGUARD v1：新增只读能力探测 / BLEGUARD v1: new read-only support probe

本轮候选与screen14的BUILD文本均为 `8a1794c-dirty`，应以完整ELF区分：guard候选为 **`48f7352f2f0f637ee4c0290a09f1141e0a2d6e5a7ad12442910b5132c5250116`**。该候选[69入口/目标构建](../audit/artifacts/cdc_recovery_acceptance_20261116/ble-guard-cadence-01-validated-identity.json)通过、187生产输入相同；上节记录其正常运行及查询HIL，**SDK阻塞注入/恢复仍待核验**。下表是实现及host验证的接口契约，独立UI支持未验收。

This candidate and screen14 share the BUILD text `8a1794c-dirty`; distinguish them by full ELF. The guard candidate is **`48f7352f2f0f637ee4c0290a09f1141e0a2d6e5a7ad12442910b5132c5250116`**. Its [69-entry suite/target build](../audit/artifacts/cdc_recovery_acceptance_20261116/ble-guard-cadence-01-validated-identity.json) pass with 187 matching production inputs. Normal-operation/query HIL is recorded above; **SDK-stall injection/recovery remains pending**. This table specifies the implemented, host-tested interface; separate desktop UI support remains unaccepted.

通过独立只读 `BLEGUARD?` 查询、收到完整 `BLEGUARD,v=1,scope=send_observer,...` 回复后确认支持；不扩展现有PROTO capability列表或改科学帧版本。旧固件可返回不支持，超时/丢回复不能直接等同不支持；隔离时优先通过USB查询，BLE回复本身可能不可发送。只读snapshot不解除隔离、不取消SDK、不触发重启。

Probe support with read-only `BLEGUARD?` and a complete `BLEGUARD,v=1,scope=send_observer,...` response. The existing PROTO capability list and scientific-frame version are unchanged. Older firmware may report unsupported; timeout or a missing reply alone does not prove lack of support. Prefer USB queries when quarantined because the BLE reply itself may be unsendable. Reading the snapshot does not clear quarantine, cancel the SDK or trigger restart.

| 字段 / Fields | 归属、单位与含义 / Domain, unit and meaning |
|---|---|
| `v=1,scope=send_observer` | 本接口版本/范围，不是帧配置revision / Query version/scope, not frame configuration revision |
| `ready,initErr` | 独立observer初始化状态及原始ESP错误；ready不代表BLE/采集健康 / Observer initialization and raw ESP error; ready is not general BLE/acquisition health |
| `quarantined,reason` | sticky隔离；reason为none/sdk_deadline/observer_unavailable；重连不清除 / Sticky containment; reconnect does not clear it |
| `callKnown,callId,startUs` | callId为本boot内uint32回绕计数，配合startUs和session识别；callKnown=0时数值不是有效调用证据 / Boot-local wrapping ID, paired with startUs and session; no call evidence when callKnown=0 |
| `detectedUs,ageUs` | 本boot单调µs；detectedUs为锁存时点，ageUs持续表示距start时间，迟返后仍增长 / Boot-local monotonic microseconds; age is not completed-call duration or CPU time |
| `deadlineUs=500000,pollUs=100000` | active SDK调用观察阈值/ESP_TIMER_TASK周期；实际观察延迟依赖调度 / Active-call threshold and observer period; actual detection depends on scheduling |
| `active,completionKnown` | active为SDK调用尚未真实返回；completionKnown仅证实该已知调用返回，不是PC接收 / Active SDK call versus observed real return, not PC receipt |
| `restartRequired` | quarantine或observer初始化失败时要求显式恢复，不意味着已自动重启 / Explicit recovery needed, not an automatic restart event |
| `rejected` | uint32模回绕；真实走到BLE准入/发送路径且被guard拒绝的次数，不是所有丢帧 / Modulo-32-bit actual guard-rejected admissions/attempts, not total missing frames |
| `recovery=manual_restart,sdkCancelled=0,pcReceipt=unknown` | 已有USB显式RESTART恢复；不能解释为SDK取消或主机ACK / Existing explicit USB RESTART recovery, never SDK cancellation or host ACK |

quarantine后callId/startUs/detectedUs/completionKnown描述被冻结的故障调用，不因重连改成新连接调用。若迟返，实际SDK返回值/计数仍记录，completionKnown可变1，但quarantine保持；尚未返回时slot/缓冲/任务对象保持所有权。外层timeout与新guard都不能使SDK内部BTC队列的无限等待自动取消，不允许Host据超时宣称数据已撤回。idle、合法暂停或CONF等待不满足active SDK调用超时条件；observer自身任务调度可能延后，不能把500+100 ms当硬实时保证。

After quarantine, callId/startUs/detectedUs/completionKnown describe the frozen fault call rather than a new connection’s call. A late return still records the actual SDK outcome/counters and may set completionKnown=1, but quarantine remains. While return is unknown, the slot, buffer and task stay owned. Neither the outer timeout nor this guard cancels an indefinite BTC queue wait inside the SDK; the host must not infer withdrawal from timeout. Idle, legitimate pause and CONF waiting are not active-call timeouts. Observer scheduling can be delayed; 500+100 ms is not a hard-real-time bound.

`rejected`还有明确统计边界：`sensorarrayTransportQueue`发现没有启用的network sink时，既有路径可在调用BLE前ESP_OK早退，因此不会增加guard rejected，ESP_OK也不代表数据入了BLE队列或PC收到。host测试的 `transportNoNetworkSink` 是合成fixture统计，**不是固件遥测字段**。需要结合原DATA提交/丢弃计数和真实接收序号，不把各层计数相加为总损失。

`rejected` has an explicit counting boundary: when `sensorarrayTransportQueue` finds no enabled network sink, its existing path may return ESP_OK before calling BLE and therefore does not increment guard rejected. That ESP_OK proves neither BLE enqueue nor PC receipt. The host test’s `transportNoNetworkSink` is a synthetic fixture counter, **not firmware telemetry**. Combine interpretation with the existing DATA submission/drop counters and actual receive sequences without adding different layers into a total-loss count.

OTG默认debug汇总改为网络周期的归一化值（当前50帧、配置0则20帧），legacy Serial/JTAG仍15帧、ADSDBG=1不变；科学DATA仍逐帧，ADS健康回读仍64帧。Host不能把debug汇总频率降低当科学采集丢失；同输入network aggregate字节/状态回归相同，但实机USB计数会随实际输出量变化。[guard审查](../audit/artifacts/cdc_recovery_acceptance_20261116/ble-send-guard-independent-review-01.json)与[汇总审查](../audit/artifacts/cdc_recovery_acceptance_20261116/otg-debug-cadence-review-01.json)仅覆盖这些补丁，不构成全项目或HIL通过。

OTG’s default debug summary uses the normalized network period (currently 50 frames, or 20 for configured zero); legacy Serial/JTAG stays at 15 frames and ADSDBG=1 is unchanged. Scientific DATA remains per-frame and ADS health readback every 64 frames. Reduced debug-summary frequency is not scientific acquisition loss. Same-input network aggregate bytes/state pass equivalence tests, while live USB counters change with actual output. The [guard review](../audit/artifacts/cdc_recovery_acceptance_20261116/ble-send-guard-independent-review-01.json) and [summary review](../audit/artifacts/cdc_recovery_acceptance_20261116/otg-debug-cadence-review-01.json) cover only these patches, not whole-project or HIL acceptance.

## screen14 可变分片长度仍按原协议 / screen14 retains the variable-length fragment contract

本节保留screen14采集时的实现状态；后续guard候选的本地验证见上节，不能回填成screen14已包含guard。 / This section preserves implementation status at screen14 capture. The later guard candidate’s local validation above does not make guard part of the screen14 image.

打包候选实际BUILD=`8a1794c-dirty`、ELF=`efa5b2f980ae1739afe54cc56f90080464eb3622013e4081bc4c510f2cb8a8f4`、session=`2036380CE3845BDB`；dirty包含生产改动，精确185输入见[验证身份](../audit/artifacts/cdc_recovery_acceptance_20261116/ble-pack-01-validated-identity.json)。新增helper只优化满足ATT cap512 B的DATA两片几何，不能假定所有message都这样拆；原planner及G envelope字段含义、科学CRC/identity不变。Host必须按实际index/count/chunk/total重组，不能要求各片等长或固定244 B上限。原始实收显示7,502个完整message均为495 B+154–165 B，CRC32重组通过；这是ATT callback字节证据，**不是LL空中分包或peer DLE实测**。见[几何记录](../audit/artifacts/cdc_recovery_acceptance_20261116/screen-14/recorded-geometry-01.json)。

The packing candidate’s actual BUILD is `8a1794c-dirty`, ELF `efa5b2f980ae1739afe54cc56f90080464eb3622013e4081bc4c510f2cb8a8f4`, session `2036380CE3845BDB`. Dirty includes production changes; the [validated identity](../audit/artifacts/cdc_recovery_acceptance_20261116/ble-pack-01-validated-identity.json) records all 185 exact inputs. The new helper only optimizes fitting DATA two-fragment geometry with ATT cap512; not every message uses that split. The original planner, G-envelope field semantics and scientific CRC/identity remain unchanged. The host must reassemble by actual index/count/chunk/total, without requiring equal chunks or a fixed 244-byte ceiling. Original callbacks show all 7,502 complete messages at 495 bytes +154–165 bytes with valid reassembled CRC32. This is ATT callback-byte evidence, **not measured LL air segmentation or peer DLE**; see the [geometry record](../audit/artifacts/cdc_recovery_acceptance_20261116/screen-14/recorded-geometry-01.json).

[screen14](../audit/artifacts/cdc_recovery_acceptance_20261116/screen-14/screen-14-verdict.json)为120.000089831秒、Capture/USB/BLE66.16/66.17/62.52 FPS，USB7,940连续帧、BLE7,502帧缺437，ADC0/84,044；仍FAIL_BLE_THROUGHPUT，600秒/32配置NOT_RUN。SDK等待guard仍 **WORK_IN_PROGRESS/未烧录**，本候选不新增其capability、恢复终态或有限返回保证。与前述待修P1相关的接口或恢复行为，只能在后续实现/验证后另行公布。

[Screen14](../audit/artifacts/cdc_recovery_acceptance_20261116/screen-14/screen-14-verdict.json) covers 120.000089831 seconds at 66.16/66.17/62.52 FPS capture/USB/BLE, with 7,940 consecutive USB frames, 7,502 BLE frames and 437 missing observations, plus ADC0/84,044. It remains FAIL_BLE_THROUGHPUT; 600-second/32-configuration runs are NOT_RUN. The SDK-wait guard remains **WORK_IN_PROGRESS and unflashed**. This candidate adds no guard capability, recovery terminal or finite-return guarantee. Any interface or recovery behavior for the pending P1 requires separate implementation and validation evidence.

帧长随原有十进制seq/time字段自然变化：本次[长度复核](../audit/artifacts/cdc_recovery_acceptance_20261116/screen-14/recorded-size-confounder-01.json)为screen13的601–609 B（7位seq，众数608 B）与screen14的591–600 B（4位seq，众数599 B）。Host不得固定逻辑长度；不同boot/运行时间也是性能比较的混杂，不能将约1 FPS增益全部归因新的分片几何。

Frame length naturally varies with the existing decimal sequence/time fields. The [size check](../audit/artifacts/cdc_recovery_acceptance_20261116/screen-14/recorded-size-confounder-01.json) observes 601–609 bytes in screen13 (seven-digit sequence, mode 608 bytes), versus 591–600 bytes in screen14 (four-digit sequence, mode 599 bytes). The host must not hard-code logical message length. Different boots/uptimes also confound the performance comparison; the approximately 1 FPS gain cannot be wholly attributed to new fragment geometry.

## 2026-11-21 同版新会话与待修 SDK 等待 / Same image, new session and pending SDK wait fix

screen13 镜像仍为 `53005f0-dirty`，session 已变为 `720D1B47362F4E7E`；先以 BUILD/PROTO/CONFIG 重新建立身份，不能把旧 session 的 sequence/generation 大小用于本会话。120.000081842 秒实测 USB 7,898 连续帧、BLE 7,380 帧缺516，完整公共帧原字节一致；ADC CRC0/84,604，CAP8 已查询恢复，吞吐仍 FAIL。该记录不增加协议版本、capability 或变更任何帧字段含义，详见[当前判定](../audit/artifacts/cdc_recovery_acceptance_20261116/screen-13/screen-13-verdict.json)。旧600秒结果不属于这个镜像/会话。

Screen13 still runs `53005f0-dirty`, but the session is now `720D1B47362F4E7E`. Re-establish identity through BUILD/PROTO/CONFIG rather than comparing old-session sequences/generations with the new session. The 120.000081842-second recording contains 7,898 consecutive USB frames and 7,380 BLE frames with 516 missing observations; common complete frames have identical bytes. ADC CRC is 0/84,604, CAP8 recovery is query-verified, and throughput remains FAIL. This observation adds no protocol version, capability or field reinterpretation; see the [current verdict](../audit/artifacts/cdc_recovery_acceptance_20261116/screen-13/screen-13-verdict.json). The historical 600-second result does not belong to this image/session.

新增 P1 的修复尚未完成：SDK BTC 队列内部可能无限等待，当前 `sdkCallActive` 不构成独立进度监管，外层发送 timeout 不能证明 SDK 已返回或数据已取消。Host 必须继续区分本地接受、SDK返回、CONF和实际完整接收；响应丢失时利用既有原子 CONFIG/终态查询恢复，不能通过无界重发 setter 推断成功。后续固件修复若新增能力或诊断字段，应独立协商与记录；此处不预告未实现接口。源码限制及待实现范围见[修复报告](../audit/firmware_reliability_fix_report.md)。

The newly identified P1 is not yet fixed: SDK BTC admission may wait indefinitely, current `sdkCallActive` is not independent progress supervision, and an outer send timeout proves neither SDK return nor data cancellation. The host must continue distinguishing local acceptance, SDK return, CONF and complete receipt. Recover missing responses through existing atomic CONFIG/terminal queries rather than inferring success from unbounded setter retries. Any later firmware capability or diagnostic extension requires its own negotiation and evidence; no unimplemented interface is promised here. The [fix report](../audit/firmware_reliability_fix_report.md) records the source limit and pending scope.

## MTU517 兼容性补充 / MTU517 compatibility update

源码commit `53005f0042a7a7694338f92c9b86ab3276db668a` 仅把 `sensorarrayTransportInit` 的 preferredMtu请求247→517；PROTO2、capabilities、科学encoder、各域gen/rid、时间单位和CRC语义均不因此改变。实际BUILD=`53005f0-dirty`、ELF SHA256=`ca6a90ffe1c2b72344e4b1a54f44169c751b39217f4da0ee5f3bdbfabe323d90`，68个host入口/目标构建通过、185生产输入相同，见[构建身份](../audit/artifacts/cdc_recovery_acceptance_20261116/ble-mtu517-01-validated-identity.json)和[启动核验](../audit/artifacts/cdc_recovery_acceptance_20261116/ble-mtu517-01-boot-verification.json)。后续仓库HEAD不能替代运行镜像身份。

Source commit `53005f0042a7a7694338f92c9b86ab3276db668a` only raises preferredMtu247→517 in `sensorarrayTransportInit`. PROTO2, capabilities, scientific encoder, generation/request domains, time units and CRC semantics are unchanged. Actual BUILD is `53005f0-dirty`, ELF SHA256 `ca6a90ffe1c2b72344e4b1a54f44169c751b39217f4da0ee5f3bdbfabe323d90`;68 host entries and the target build pass with185 identical production inputs. See the [build identity](../audit/artifacts/cdc_recovery_acceptance_20261116/ble-mtu517-01-validated-identity.json) and [boot verification](../audit/artifacts/cdc_recovery_acceptance_20261116/ble-mtu517-01-boot-verification.json). Later repository HEAD is not the running-image identity.

Host 应继续同时接受直接完整message与G envelope分片，不能硬编码244 B notification上限或假设固定分片数。发送使用每条消息的连接generation/协商MTU快照，单包上限 `min(MTU−3,512)`，完整DATA message最大2304 B、CTRL最大512 B；较大的MTU可让原来分片的消息直接发送。每次重连先按MTU23初始化，再由实际MTU事件更新；旧generation的排队项不能套用新连接身份。按原messageId/index/count/chunk/total/CRC规则重组后，再检查完整科学帧的CRC和不可变配置身份；SDK成功或CONF回调均不是PC应用收到完整帧的确认。

The host must continue accepting both raw complete messages and G-envelope fragments, without a hard-coded244-byte notification ceiling or fixed fragment count. Each message uses a connection-generation/negotiated-MTU snapshot with packet cap `min(MTU−3,512)`, complete DATA limit2304 bytes and CTRL limit512 bytes. A larger MTU can make a previously fragmented message fit directly. Reconnect initializes MTU23 until an actual MTU event updates it; old-generation queued items cannot acquire the new connection’s identity. Reassemble using the existing messageId/index/count/chunk/total/CRC rules, then verify the complete scientific CRC and immutable configuration identity. SDK success or a CONF callback is not proof that the PC application received a complete frame.

[现场BLELINK](../audit/artifacts/cdc_recovery_acceptance_20261116/ble-mtu517-link-01.summary.json)为MTU517/PHY2/7.5ms/latency0/localDLE251、peerKnown0；preferredMtu、BLEBOOT以及Bleak未验证的默认23不作为协商结果。实际C发送路径和本仓库接收器已通过247/517、原始/分片、重连和旧generation回归；**独立上位机UI兼容性尚未运行**，软件仓库未修改。完整机制及结果见[README](../../README.md)；旧600秒结果仅属于a656d53，不能用于本版验收。

The [recorded BLELINK](../audit/artifacts/cdc_recovery_acceptance_20261116/ble-mtu517-link-01.summary.json) reports MTU517/PHY2/7.5 ms/latency0/local DLE251 with peerKnown0. preferredMtu, BLEBOOT and Bleak’s unverified default23 are not negotiated-MTU evidence. The actual C sending path and this repository’s receiver pass247/517, raw/fragmented, reconnect and old-generation regressions; **separate desktop UI compatibility remains NOT_RUN**, and the software repository is unchanged. [README](../../README.md) contains mechanics and results. The earlier600-second result belongs only to a656d53 and is not acceptance evidence for this version.

> 2026-11-16 历史扩展记录 / Historical extension record: `b58336e-dirty` 公告 `cpu_owner_v1`，当前诊断镜像开启 runtime profiling，正常默认配置仍关闭。完整 BUILD/READY 身份与 65 个测试入口见[本轮启动核验](../audit/artifacts/cdc_recovery_acceptance_20261116/cpu-owner-01-boot-verification.json)；screen07 ADC CRC 0/76,128、Capture/USB/BLE 59.90/59.81/32.39 FPS，BLE 吞吐仍未通过。能力存在不等于诊断启用，也不等于性能验收；下面旧阶段和旧提交作为历史保留。 / `b58336e-dirty` advertises `cpu_owner_v1`; runtime profiling is enabled in this diagnostic image, while normal defaults remain disabled. Full BUILD/READY identity and 65 test entries are linked in the [boot verification](../audit/artifacts/cdc_recovery_acceptance_20261116/cpu-owner-01-boot-verification.json). Screen07 records 0/76,128 ADC CRC rejections and 59.90/59.81/32.39 FPS capture/USB/BLE; BLE throughput remains unaccepted. Capability presence means neither enabled diagnostics nor performance acceptance. Older stages and commits below remain historical records.

> 2026-11-13 历史阶段烧录源码为 `0d8f60fd9c29b9ce12e64ed2e6534ee37db41704`，
> BUILD=`0d8f60f-dirty`（dirty 是文档/证据，构建源码 hash 独立保存），app SHA256
> `232ca3976f6b8ea38efd5e09de8279f35c9bfbbd0c605c8dd84e5169ddd1dbaa`。
> 本轮启动模块/扫描主实现 `ee8f972`，FDC ID 地址修正 `e339a1d`，
> 运行期 CRC 局部失效分类 `0d8f60f`。
> [构建](../audit/artifacts/boot_autotune_20261113/final-03-build-manifest.json)、
> [42 个测试入口](../audit/artifacts/boot_autotune_20261113/final-04-tests-manifest.json)、
> [app-only 烧录](../audit/artifacts/boot_autotune_20261113/final-03-flash-result.json)
> 均已实际执行。以下 ebabe7e 是历史版本；本轮实测与限制见
> [启动/恢复验收](../audit/frontend_startup_recovery_20261113.md)。

## 启动扫描与前端恢复扩展（本轮新增）

PROTO 保持 2，新增 `boot_spi_qual_v1/frontend_presence_v1` capabilities；现有数据
encoder、CRC、单位和各配置身份域不变。此扩展不要求 Host 把坏 ADC CRC 当成有效点。

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

本轮审计基线为 `8045e9e9ec9599533c52c15dfcb6002f79fd15f1`，首批固件修复为 `ba8346b7b643e54985699630f608aba3386810e9`。**上一版已烧录源码 commit：`ebabe7e6816c0347c6d1f5187d5f7f801a7ef98d`；BUILD 版本：`ebabe7e-dirty`；app SHA256：`2dfd717ec374c3d046c20316320b3e2199e3eeda22c4dd94d193336a1aad8ba2`。** 此处 dirty 为文档/证据改动，源码已提交；准确构建输入、逐文件 hash 和 diff hash 见 [最终构建清单](../audit/artifacts/reference_fixture/serial-lines-final-build-manifest.json)，烧录校验见 [最终烧录结果](../audit/artifacts/reference_fixture/serial-lines-final-flash-result.json)。本文保留该版记录；其对应结果见 [11月4/5日 HIL 记录](../audit/reference_fixture_hil_20261104.md)，当前版本以文首更新为准。

ADC SPI CRC 错误**尚未根治**，NOP 候选已撤回，不能将有限重试、诊断完善或 ASCII 帧 CRC 通过视为 ADC 链路无误。`release-switch50-identity-01` 的 [50 次控制/身份测试](../audit/artifacts/reference_fixture/release-switch50-identity-01.summary.json) 为 PASS，但测量状态是 `NOT_PRECISION_ACCEPTED`，没有写入校准；它不替代或改写此前严格“切换且参考点有效”测试的失败记录。

此协议交给 ErrorChen/SensorArray_Software 的独立任务实现；本轮未修改软件仓库。协议格式来自实际 `main/output/sensorarrayTextProtocol.c` encoder，完整修改及验证记录见 [固件可靠性报告](../audit/firmware_reliability_fix_report.md)。

2026-11-04 实机追加修复 commit **`a0460b8`** 保持 PROTO2 和测量帧字段语义，补充 ADC CRC 有限恢复与诊断。该镜像的构建/烧录/实录见 [夹具 HIL 记录](../audit/reference_fixture_hil_20261104.md)。

## 运行期 CRC 部分帧（0d8f60f）

内部 `boundedCrcFailure` 只许可纯采样 CRC 拒绝的部分帧进入原数据 encoder，不是新 wire 字段；旧帧 `expected/acquired/fresh/valid/error` 和 X02 语义保留。终态与 CONFIG 仍是配置权威，不要求每个点都 CRC 有效。若旧 Host 以全 acquired 门控整帧，它可能继续丢弃合法部分帧；必须交由软件任务调整，固件不会把缺失位改成已采集。

`ADSDISPOSITION` 是诊断事件，`crcRejected/acquired/expected` 为十六进制位图，`allCrcRun/limit=3` 是连续全 CRC 拒绝的 ADS engine 扫段计数，`scope=ads_sweep` 不等于主机混合帧数。`firstError` 保留首错；`abortError` 独立说明导致停止的后续错误或全坏预算耗尽。首错CRC后出现timeout、RESET、配置变化、owner/route故障、SPI/DMA未知时不允许继续。一次合法非CRC测量结论（包括OPEN）可清除此连续计数，CAP或模式切换本身不能清。

## 能力协商与兼容边界

连接后先查询 `PROTO?`、`BUILD?`、`CONFIG?`，保存原始回复。协议版本升为 **2**，新增 `config_identity_v1/config_query_v1/session64` capability。`dataMax` 为 **2304 bytes**，CTRL 保持 512 bytes。固件中的固定容量 slot、BLE 完整 message 和文本 encoder 容量一起调整；USB普通缓冲和日志CPU工作区/帧副本启动时优先PSRAM一次分配，没有在采样热路径动态分配大报文。

旧 `C/V/R/M/MR/K` 字段含义保留；新增字段是扩展。Host 不得将旧 `gen/rid` 统一解释为 MODE。只支持 1536-byte 重组缓冲或要求 K 行 crc 字段位于行尾的旧 Host 必须升级；不能宣称新版所有报文对旧解析器完全兼容。未协商到新能力时，旧帧只能按下表原域处理，不能从 CAP 的低 gen 推断模式回退或帧过期。

## 字段域与 immutable frame identity

| 报文/字段 | 所属域与来源 | 对应终态 |
| --- | --- | --- |
| C `gen/rid`，对应 K `gen/rid` | **ROWS**；`frame.configSnapshot.generation/requestId` | 行数 RAPP |
| V/R `gen/rid`，对应 K `gen/rid` | **MODE**；`frame.measurement.modeGeneration/modeRequestId` | MAPP；同质 ROWMODES 也应用 MODE |
| M `rgen/rrid`，对应 K | **ROWS**；`frame.configSnapshot` | 行数 RAPP |
| M `pgen/prid`，对应 K | **ROWMODES profile**；`frame.rowProfileGeneration/RequestId` | RMAPP；MODE 同步同质 profile |
| MR | 没有独立配置计数；继承本 M/K message 的身份，`s` 为实际 S 行，`m/unit/scale` 属于该行 | 同本 M |
| 全部新 C/V/R/M header 的 `mgen/mrid` | 显式 MODE 域 | MAPP/CONFIG |
| 全部新 C/V/R/M header 的 `rgen/rrid` | 显式 ROWS 域 | 行数 RAPP/CONFIG |
| 全部新 C/V/R/M header 的 `pgen/prid` | 显式 profile 域 | RMAPP/CONFIG |
| 新 header 与 K 的 `session/bootId/rev` | 启动会话、旧启动计数、完整配置 revision | CONFIG、带相同身份的终态 |
| ROWS? `generation/appliedId` | ROWS；pending/requestId 是待处理行请求 | RCMD/RAPP |
| MODE?/STATE? `gen/rid/seq` | MODE；seq 是其应用边界，不是最新成功接收帧 | MAPP |
| ROWMODES? `gen/rid` | profile 域 | RMAPP |
| MAPP `id/gen/seq` | MODE 请求、成功 generation、允许新配置采样的边界 | 终态 |
| RMAPP `id/gen/seq` | profile 请求、成功 generation、当前 profile 的边界 | 终态 |
| 行数 RAPP `id/gen/seq/old/new` | ROWS 请求与边界 | 终态 |
| RAILCFG 的 RAPP `avdd/avss/source` | rail 校准事务；不是 ROWS | 不能用来推进 ROWS generation |
| RECOVER/RESTART 的 RAPP `cmd/level/state` | 恢复生命周期；不是 ROWS/MODE 成功 | 查询 CONFIG 重建 |

布局同质性检查完整的 **8 项配置 profile**，不是仅检查活动行。`ROWMODES=CVRCVRCV` 加 `ROWS=1` 仍走 MIXED engine，输出 `M(profile=CNNNNNNN)`、一个 CAP `MR` 和 `K`；CONFIG 保留完整 `CVRCVRCV`。这里只有活动测点全为 CAP，不能按普通 C 帧或其旧 `gen/rid` 域解析。2026-11-13 的 120 秒实录确认了这一点；首次 benchmark 工具预期 C 帧属于工具误报，原始记录保留。

Layout homogeneity examines the complete **eight-entry configured profile**, not only active rows. `ROWMODES=CVRCVRCV` with `ROWS=1` still uses the MIXED engine and emits `M(profile=CNNNNNNN)`, one CAP `MR`, and `K`; CONFIG retains `CVRCVRCV`. Only the active measurements are all CAP. Do not parse this as an ordinary C frame or assign C's legacy identity domain. A 120-second capture on 2026-11-13 confirms this; the initial benchmark expectation of a C frame was a harness error, whose original evidence is retained.

采样 owner 在每次 engine 调用之前，从已选定的 mode、rows、profile 和 plan 捕获 `sensorarrayConfigIdentity_t`（本轮新增）。engine 对 frame 的清零不会使它丢失：调度器在返回后复制同一局部 snapshot，encoder 和 transport descriptor 仅解析/携带该 frame。发送阶段不得读取最新全局配置盖章。`rev` 在完整配置或安全状态改变时前进；正常逐帧不前进。profile 准备期间采样使用候选 identity，只有成功应用后发布相同 revision 的权威快照；失败帧无效，权威状态保留真实失败状态。

因此现场反例 `CAP gen=2/rid=1` 与此前 `VOLT gen=4/rid=3` **不属于同一域**。新 CAP 明确携带 `mgen/mrid`，Host 必须使用它进行 MODE 关联。

## CONFIG?：无副作用的原子查询

`CONFIG?`（本轮新增）在短临界区内复制一个已发布的完整对象；它不执行硬件操作，不拼接 MODE?/ROWS?/ROWMODES? 的独立读数。未发布时返回 `ERR,cmd=CONFIG,reason=not_published`。查询字段包括：

- `session`：16 位十六进制随机启动会话；`bootId`：保持旧 RTC breadcrumb 计数。
- `rev/boundary`：配置 revision 与开始允许该配置采样的 frame boundary。
- `state/mode/rows/profile`：采集 owner 发布的状态；mixed 的 mode 为 `MIXED`，profile 保持完整 8 个字符。M header 的非活动行仍用 N，不产生 MR。
- `mgen/mrid/rgen/rrid/pgen/prid`：各域最近**成功**应用的身份。
- `mtermId/mterm/merr`、`ptermId/pterm/perr`、`rtermId/rterm/rerr`：最近一次各域请求的终态缓存；outcome 为 none/applied/rejected。
- `err`：当前 mode 状态错误，与某个请求的 terminal error 分开。

失败 request ID 不覆盖成功 applied ID。`MODE?`、`STATE?`、`ROWS?`、`ROWMODES?` 仍供旧软件/诊断使用，它们组合起来不保证原子性；支持新能力的 Host 必须以 CONFIG 为重建依据。CONFIG 公开的是最后完整发布的配置，不声称队列里尚未应用的每条 accepted intent 已生效。

## 接受、应用、丢失与乱序恢复

MODE、ROWS、ROWMODES 在所有 transport 共用配置 ID allocator 和单个 outstanding 配置事务。忙时在接受前拒绝，Host 可查询并稍后重试；不静默替代已接受请求。其它有界诊断命令仍可排队。MACK、RCMD、RMACK 是 accepted，绝不是 Applied。终态的 `identityKnown=0/rev=0` 表示当时无法取得完整快照，仍保留真实 session；此时重试 CONFIG?，不能将零 revision 当成功配置。

ACK 本地提交成功后，配置命令才对 Core 1 可见；ACK 提交失败则释放 reservation。Core 1 在完整帧边界应用。终态存入专用有界保留槽，优先于普通诊断发送；最后结果另存权威快照。这里的本地提交和保留不能证明 PC 收到，不承诺 BLE/UDP exactly-once。物理断连期间仍可能没有任何有效接收端。

同一 CAP 再次请求仍有新的 request ID 和成功 generation；不能无声忽略。MAPP 表示硬件/配置已应用，`boundary` 指定允许新配置采样的边界，**不保证首个有效数据已传到 Host**。RMAPP 接受完整新扫帧或已验证配置下的有界 CRC 部分帧；它不保证所有点有效。局部 CRC 重试耗尽仍输出 X02，valid/fresh/acquired 保持真实缺失，Host 应按点处理位图，不能因为配置已 applied 就将缺失点补零或沿用旧值。ROWS RAPP 只确认行配置，SAFE 中也可完成，不表示矩阵已恢复运行。

Host 的恢复顺序：

1. 将 accepted 请求与当时 session 和请求域一起保存，保留旧显示数据的原身份。
2. 新帧早于终态时，可以暂存有界数量的新 tuple；用 CONFIG 验证 session、rev、各域身份与 boundary 后接纳。不要把数据早到当成终态丢失的唯一原因。
3. 终态丢失而 CAP 数据持续时，CONFIG 中匹配的 mrid/mtermId 及显式 CAP mgen 可结束 pending。C 的 legacy gen/rid 只验证 ROWS。
4. 旧终态迟到时按 session+请求域+ID+revision 关联；不得用它回退较新的权威配置。
5. 新旧两个 CAP 请求均可成功，必须按 mrid/mgen 区分，不能只比 mode 字符串。
6. 配置切换前排队的帧仍保留原身份。可以按应用边界明确丢弃并计数，不能给它们重贴新标签。

## 启动、计数器与时间

RECOVER 另绑定接受时的 session 及各域 gen/rid，旧配置的恢复请求返回 `RERR,reason=stale_config`；仅运行状态/rev 改变不使同配置恢复失效。单个恢复 pending 不被后来请求无声覆盖。

旧 bootId 在 RTC 无效/断电后可能重新从 1 开始，不能单独证明两个连接属于同一启动。新增 session 在 app_main 生成一次、重启重新生成，是随机 64-bit nonce；碰撞概率很低，**不是数学上绝对唯一**。跨 session 不比较 sequence、generation 或 revision 大小；恢复连接先查询 CONFIG 并重新建立基准。

32-bit generation/revision/request allocator 回绕跳过 0；0 表示未应用/未知。frame sequence 是无符号 32-bit 模运算，可回到 0。排序仅限同 session、同一域、距离小于 2^31 的 serial-number arithmetic；请求关联优先精确相等。无法证明半区间时重新查询。不同域绝不比较大小。

`ts`、`dur`、`tr`、`transitionUs`、采集/事务 `*Us` 使用微秒；`*Ms` 使用毫秒；seq/frameCount/ageFrames 是帧计数，不是墙钟秒。零 timestamp 的含义按报文说明，不推导成物理稳定。`4294967295` 的 age/lastGood 未知哨兵不能显示为实测巨大年龄。

## CRC、无效点与科学数据

完整 ASCII frame CRC32 使用 reflected polynomial `0xEDB88320`、初值 `0xFFFFFFFF`、final xor `0xFFFFFFFF`。覆盖从 C/V/R/M header 首字节到 K 之前最后一个换行的所有字节，包括新增 header identity。K 自身不在该 CRC 内；同时检查 K 的 seq、legacy 域、session/bootId/rev 与 header 一致。BLE G envelope 的总长及 CRC32 是额外的重组校验，不能代替 ADC SPI 校验。

CAP pf6 是 pF × 10^6，既有无效 sentinel 和 masks 保留。VOLT fixed value 是 µV（scale=-6），RES fixed value 是 mΩ（scale=-3）；MR 各行独立 unit/scale。`Xhh` 是明确无效原因，不能填 0 或用于 PGA/cache。expected 是计划掩码，acquired 是实际已完成采集的掩码；健康检查在任何点转换前失败时 acquired=0，不等于执行了 64 次转换失败。

TX=SHORT/RT 的 B20 是 `previewOnly=1` 摘要，保留来自原队列 payload 的 session/revision/各域身份；无效端点为 na。B20 没有完整科学 frame 的 ASCII CRC，不可替代 C/D/K，也不能仅凭预览声称全部点已采集。

## rail、电池、错误与 transport

透明内部 rail 监测是否可用必须看 capability/硬件隔离。当前板型无已证实隔离时报告 unavailable，外部 DMM 校准是受限 VOLT 的合法路径；不要在 Host 自动发起未授权 INTREF 激励。rail span、AVDD、AVSS 分开解释，source 与 estimated/valid/age 分开显示；span 实测不等于两条对地轨均独立实测。外部校准不能因 matrix 帧龄而自动变为内部实测。rail/battery 的 valid=0 保留未知和真实 last-good age，不沿用旧值冒充本次有效值。

错误码必须按实际 BUILD 的 ESP-IDF 解释；本次 IDF `ESP_ERR_INVALID_RESPONSE=0x108`，`ESP_ERR_TIMEOUT=0x107`，不得把所有 0x108 称为超时。ADSFAULT 的 originalError 与 cleanupError 分开，阶段指向首次比较/读取失败；SAFE_REQUESTED 或 GPIO 回读不代表模拟节点已验证安全。故障多行按 bootId + seq 关联，不能把截断行补为原始数据：

| 报文 | 本轮新增/明确的字段 |
| --- | --- |
| ADSFAULT | 首次 stage 及收尾前 mode/route；未知 bootId 明确 unknown |
| ADSFAULTRAIL | railSpanUv、avddUv、avssUv、source、valid、ref |
| ADSFAULTCTX | originalError、cleanupError、engineFrameCount、modeEntryFrameCount、healthInterval、healthReadbackDue、precisionFrame、attempted、acquired |
| ADSFAULTREG | reg、expected、actual、mask、cacheGen、physicalGen、configGen |
| ADSFAULTTXN | kind=rail/battery/ADSCHK、id、startUs、completeUs、firstValidFrameUs、originalError、cleanupError |
| ABAT | 旧 UINT_MAX sentinel 保留；新增 lastGoodKnown，unknown 不是实测年龄 |
| ARL | split=unknown/estimated/external_measured、aincom=estimated、uncertaintyUv=unknown、calUs |

上述 ADS 故障格式化行保持有界（<384 bytes）；这不是全部协议行的长度上限，USBSTREAM 控制回复使用下文的 512-byte 容量规则。ADC 数据 CRC8 在生产路径启用，但 ADC STATUS 不在该芯片数据 CRC 保护范围；软件仍需检查状态和配置 freshness。完整实现证据见 `docs/audit/artifacts/reliability_fix/ads_notes.md`。不能通过相同名字的 rail 字段猜 AVDD 或 span。

Serial 为 ESP32-S3 USB Serial/JTAG，不是 TinyUSB CDC。波特率设置不是 Native USB 的真实线速上限。USBSTREAM/DATA 限流不关闭命令 RX 或关键终态。USBSTREAM? 的 bufferMemory=0/1/2 分别为未分配/内部RAM/PSRAM，bufferBytes 是普通队列的总容量；txStackBytes/txStackMinBytes 是实际TX任务预算/水位。driverAcceptedBytes 只证明驱动接收，不证明 PC 已读；没有驱动枚举证据时 bus 状态为 unknown。COM8 不存在只能说明该次系统打开失败，Host 需自己查询 OS 枚举。

BLE notify 成功排队不等于接收确认；SAFE 的 Indicate/Notify fallback 必须按实际链路状态解释。Wi-Fi UDP 无端到端送达保证。Capture、output queue、各 sink 提交、驱动完成/丢弃分层统计，使用增量与测量窗口；不能把累积 dropD 当每一帧都丢。采集率与 BLE 容量允许不同，Host 应保留可计数的 sequence 缺口。

## 可重放证据与尚未运行项

`python3 tools/test_config_identity.py` 真正编译并运行 C 状态机和实际 encoder，生成 `tests/fixtures/config_identity/*.frame` 与 `.config`。这些是**合成输入，经真实 encoder/formatter 输出**，不是现场原始日志；CRC 从 encoder 产生并由 Python zlib 独立验证。覆盖 1/8 行 CAP/VOLT/RES/mixed、健康前转换失败、50 次模式切换、rows 域与 mode 域分离、失败 appliedId、恢复后 CAP、boot/session 和回绕。R 极值 1844 bytes、mixed 极值 2085 bytes 均小于 2304。

`python3 tools/test_rowmodes_ack_guard.py` 执行真实 mailbox/config/guaranteed lane 测试，含 MODE/ROWS/ROWMODES 仲裁、ACK 前不可见、ACK 失败取消、满队列回退及终态有界存储。`python3 tools/test_main_config_commit.py` 从当前 main 抽取实际 commit/terminal 分支，7 个硬件边界 stub 场景检查 snapshot/preAccept 失败不发 MAPP、成功 mode/profile 同 revision、mixed RMERR 发布最终故障状态。目标芯片 build 与全套实际测试结果由根审计报告记录。首批仅完成 host 回归时，缺少对应实机证据的项目记为 **NOT_RUN**；这是当时的范围，不是对当前所有 HIL 的结论。后续已执行的逐配置采样、控制切换及 BLE 接收按 [最新 HIL 记录](../audit/reference_fixture_hil_20261104.md) 分别判定 PASS/FAIL/未验收；仍未执行的 Windows 枚举、示波器或其它场景继续保留 NOT_RUN。不能由这些 host 测试推导物理准确度 PASS。

## 实机追加诊断：CRC 与健康边界

- `0x109` 是实际 ESP-IDF 的 `ESP_ERR_INVALID_CRC`；`MATRIX_CRC` 区别于路由、寄存器比较和 DRDY 故障。`ADSFAULT` 的 modeGen/profileGen/rowGen/rowReq 分别属于 MODE/profile/ROWS/ROWS；辅助事务 ID 不再借用 rowReq。只有同一 frame sequence 的 engine context 才可附着到当前 fault。
- `ADSSPI` 是可限频的诊断事件：crcFailures/crcRetries/crcRecoveredReads 属该帧，crcTotal/retryTotal 是本次设备启动内的驱动累计计数。读取时必须做同 boot 增量；日志缺失不等于无错误，恢复成功也保留累计数。`firstS/firstD` 指该帧第一个被拒绝 CRC 样本所在测点，firstError/terminalError/frameError 分别说明首次失败、最终重试失败和整帧结果。正常结束时 terminalError=0。
- `ADSSPIBYTES` 给出同 seq 的首个坏样本，bytes 最多 6 字节，按 STATUS（若启用）+ 4 字节数据 + CRC 排列，len 指实际有效长度；received 是线上 CRC，computed 是从 4 个数据字节计算的 CRC。STATUS 不受芯片 CRC 保护。firstUs 为设备单调 µs；physicalGen/drdyGen 是当时配置/转换域。
- 每 cell 的 CRC 重试共享现有 `CONFIG_SENSORARRAY_ADS_MATRIX_IO_RETRY_COUNT` 预算（当前 1），且受同一个 100 ms cell 总截止时间约束。每次重试等待新的 DRDY；坏样本不进入 PGA/cache。DMA 不能收回时仍隔离，不能把超时当取消。
- `ADSFAULTREG` 在 MATRIX_CRC 时 reg=255 表示没有寄存器可比较，expected/computed 与 actual/received 是 CRC 字节；cell 只从 ADSSPI 读取，不能把 reg 当 cell index。
- `ADSHEALTH` 是健康回读事务退出后的诊断，字段为 seq、engineFrameCount、modeEntryFrameCount、healthInterval、healthReadbackDue、precisionFrame、err。它用于核验实际累计 engine 边界，不能拿 host seq 或模式区间长度替代 engineFrameCount。
- 电池 admission 已明确因 rail 不存在/过期拒绝且未改状态或恢复已验证时，reason 保留 rail_invalid；不再泛化成 spi_error。真实 SPI/CRC、restore failure 仍单独报告。LastGood 时间不因拒绝测量而刷新。

这些都是可忽略的追加诊断标签，不是新的有效测量点；不要将 ADSSPI/ADSHEALTH 当 C/V/R/M 数据，也不从诊断事件推导 PC 已收齐测量帧。

## 2026-11-05 追加诊断契约：8d6d656

本节记录 **`8d6d656a6272af7f19e338b594e98d582762737e`** 引入的诊断，当前已烧录版本见文首；该历史版本继承 `a0460b8` 的 CRC/故障域修复、`8b0e651` 的电池入场诊断和 SPI 时序试验、`6a8f502` 的 CRC 末位覆盖计数。以下字段在当前版本继续保留，保持 PROTO2、测量帧 CRC 与字段域不变；诊断新增字段由 Host 按标签解析并容忍未知字段。该版本号或新增日志本身不表示所有 HIL 项通过，也不表示 SPI 电气问题已排除。前文 NOT_RUN 条目仍以各自的实际证据为准。

### 电池：入场状态与恢复结果分开

`BATREG/BATRAW/BATCTX/BATCMP` 以 **当前 session + seq + txn** 关联；这些行本身没有 session，Host 必须从同一连接的 BUILD/CONFIG 与启动事件建立上下文，断连后不得仅凭 txn 拼接。`txn` 属 BATTERY 辅助事务，不是配置 request ID。`BATCTX` 在事务返回错误或 continuation gate 失败时输出；`BATCMP` 仅在保存了具体比较差异时输出。没有某行可能是没有该证据或日志丢失，不能反推测量成功。

| 字段 | 当前生产语义 |
| --- | --- |
| BATREG `phase` | saved/configured/restored；故障时另有 shadow。新增 ID、INTERFACE；各寄存器为无 `0x` 前缀的十六进制。OCAL/FSCAL 按高字节在前显示，不改变其实际寄存器写入顺序。 |
| BATREG `known`；BATCTX `*Known` | 该组快照是否取得；shadowKnown 仅说明软件 shadow 完整。savedKnown/configuredKnown/restoredKnown 不等于“匹配预期”或“模拟稳定”；known=0 时的零字节是未填充内容，不是实际寄存器值。 |
| BATCTX `stage/cleanupStage` | 首个事务失败所在步骤与收尾步骤分别记录。snapshot_shadow_mismatch 是稳定物理读数与 shadow 不一致；snapshot_inconsistent 是驱动未取得一致读数；snapshot_read 是快照读取失败；reset_observed 是读取到了 POWER.RESET。不能把上述几种情况统称为芯片复位或执行过恢复。 |
| BATCTX `originalError/cleanupError` | 保留首个事务错误，收尾错误单列；如果仅收尾失败，两者可以相同。入场即发现未知状态、未执行恢复时 cleanupError=0，不再用虚构的恢复错误替代原始条件。 |
| BATCTX `modified/restoreAttempted` | modified=1 表示已经开始可能修改硬件的命令，失败命令也不能假定未生效；restoreAttempted=1 才表示执行过恢复流程。 |
| BATCTX `restore`；BATERR `restoreState` | unchanged：无需恢复且允许继续；verified：恢复路径成功；failed：执行恢复但失败；unknown：入场状态未知，未执行恢复。verified 是软件/寄存器恢复判定，不是模拟节点实测。 |
| BATCTX `snapshotRead` | 入场快照驱动诊断：0 无该次读取失败、1 读取/SPI 错误、2 有界多次读数不一致。必须结合 savedKnown/stage，未调用读取时的默认 0 不证明读成功。 |
| BATCMP `phase/reg/expected/actual/mask` | phase=admission 或 restore；每阶段保留首个差异。reg 是**芯片寄存器地址的十六进制**，其余三字段也是十六进制。shadow mismatch 时 expected 来自 shadow；读数不一致时 expected/actual 是驱动保留的两次读数；恢复比较时 expected 来自 saved。 |

比较只排除手册规定的保留位：POWER mask=13（包含 RESET）、INTERFACE=0F、REFMUX=3F，其余被比较的 ID/MODE/MUX/校准字节 mask=FF。RESET 单独观察时 mask=10；INTREF 是配置位，仍比较。日志中的稳定差异不证明 shadow 或芯片哪一方正确，当前行为是保留证据并隔离，不自动用读数覆盖预期。

`BATERR` 保留旧 `restore=ok/fail` continuation gate，新增 `restoreState/stage/cleanupError` 供精确解释。`restore=fail` 也可能是入场发现未知状态，不能仅凭旧字段称“恢复操作失败”。`action=isolate_ads` 表示不得继续使用 ADS 并交给有界恢复；仅允许继续的电池无效结果使用 `action=report_continue`。LastGood 的值、frame 与实际测量时间不因这些失败刷新。`BATRAW.timeUs` 的零槽位不代表采到了零值；结合样本数、错误和阶段解释，不能把未开始转换计作三次有效采样。

### Rail：专用采样配置与能力边界

`PROTO?/ROUTE?` 的 `railMonitor=unavailable` 指当前板型没有已验证的**透明隔离辅助监测能力**。它不表示 RES 中所有内部 rail 读取都被禁用：已选择 RES、参考本来就有意开启时，内部 monitor 可复用该状态。生产入口要求物理 POWER.INTREF 已开启，REF source 命令正确且 Q1 钳位命令/回读均释放；这仍不是模拟隔离实测。CAP/VOLT 不能为内部监测偷偷开启参考，受限 VOLT 继续使用合法外部实测 rail 校准。

`8d6d656` 为内部 supply monitor 显式设置 PGA enabled/gain 1、chop off、连续转换、SINC1、当前配置的 ADC data rate、INPMUX=CC、REFMUX=00、neutral OCAL/unity FSCAL，再读回验证；不再继承上一个矩阵点的 PGA bypass/gain/filter。当前 sdkconfig 的 DR=15 对应 MODE0/1/2=`00/00/0F`，Host 不应把这组常量当未来所有 BUILD 的固定值。事务结束恢复完整原配置；配置验证、采样成功和恢复成功是不同条件。

| 报文/字段 | 当前生产语义 |
| --- | --- |
| RAILPROFILE `before/configured` | 三个十六进制字节依次为 MODE0/MODE1/MODE2。configured 是物理读取值；known=0 时不可解释其零值。before 在入场/保存失败时也可能未取得。 |
| RAILPROFILE `mux/ref/known/verified/restored` | mux/ref 是配置快照的 INPMUX/REFMUX；known 为快照取得，verified 为专用 profile 比较通过，restored 为完整原配置恢复成功。restored=1 不排除先前 STOP 失败，仍须检查 cleanup。 |
| RAILPROFILE `stage/err/cleanup` | stage 为 ADMISSION/CONFIGURE/PROFILE_READ/PROFILE_COMPARE/START/SAMPLE/SAMPLE_STATUS/RESTORE_VERIFY/NONE。ADMISSION 也覆盖尚未填充诊断前的读取/保存错误，不能仅凭此判断隔离拒绝。err 是边界函数最终结果，可能还包含 rail 合法性拒绝；stage=NONE 单独不代表整个边界事务成功。cleanup 单列 STOP/恢复错误。 |
| RAILSAMPLE `n/raw/monitorUv/status` | n 是已尝试的 fresh-read 调用数，**包含失败的那次**，当前最多 5（4 次丢弃加最终 1 次）。raw/monitorUv/status 只记录最近一次尝试，不是全部原始样本，也不保证 CRC/状态通过。monitorUv 为内部约 `(AVDD−AVSS)/4` 通路的 µV，不能直接当 railSpanUv 或对地电压。 |
| RAILSAMPLE `reg/expected/actual/mask` | PROFILE_COMPARE 时给出首个差异；这里 reg 是**十进制**地址，与 BATCMP 的显示进制不同，后三字段为十六进制。255 表示已初始化但无寄存器差异；早期 ADMISSION 的默认零值也不是有效比较证据。 |

以上两行按当前 session + seq + RAIL txn 关联，不与相同数值的 BATTERY txn 合并。样本只有在整笔事务成功、profile/状态/恢复检查通过且后续 rail 合法性验证通过后才更新有效 rail；诊断原始值不能直接写入测量/校准。Span、来源或采样 profile 变化应划分分析区间，不能把跳变前后总均值拟合成一个固定 offset。完整事务起止仍看 ADSFAULTTXN/对应事务证据，不能从 RAILSAMPLE 或单独采样耗时推断全部等待时间。

### CRC 覆盖计数与有界日志

`ADSHEALTH` 追加 `crcValidEven/crcValidOdd/crcErrors/crcRetries`。前两者是 CRC8 **校验通过的 CRC 字节最低位**为 0/1 的累计次数，不是数据值奇偶、bit population parity、健康检查次数或有效测量点数；checksum/off 模式不计入它们。计数发生在数据解析层，随后仍可能被 STATUS、freshness、generation 或电气合法性检查拒绝，且包含共用 ADC1 驱动的辅助读取。crcErrors/crcRetries 同样为驱动累计值，而 ADSSPI 的 crcFailures/crcRetries 是对应帧值，不能直接混算。

这些 uint32 计数属于当前驱动 handle 生命周期，通常跨模式保留；重启或驱动重新初始化会归零，回绕按同一生命周期的无符号差值处理，无法确认连续性时重建基准。Host 应在同 session 的有界时间窗比较增量。只有 odd 增长不能覆盖“末位被强制为 1”的反例；even/odd 都增长只证明该窗观察到两类通过校验的 CRC，不能证明线路完全可靠。

BATCTX/BATCMP、RAILPROFILE/RAILSAMPLE、ADSHEALTH 均是普通诊断事件，没有独立科学帧 CRC，也没有终态送达保证。这些采集侧事件的格式化缓冲为 **384 bytes（含结尾 NUL）**，不适用于 USBSTREAM 查询回复或完整科学帧；当前上述固定格式按有界字段输出，事件队列零等待提交，背压记入 droppedEventLogs。通用格式化器仍有截断后补换行的兜底路径，因此“有换行”不证明字段完整，Host 必须拒绝缺必需字段的诊断记录，保留原字节，不能补成完整原始行。下文串口整行队列处理的是 VFS 片段与队列边界，不能补回源格式化器已丢失的内容。

对应的纯主机回归为 `tools/test_battery_snapshot_evidence.py`（实际入场/比较/日志函数）、`tools/test_battery_validity.py`（错误分类与 LastGood）、`tools/test_ads_rail_profile.py`（实际 rail 配置/保存/恢复函数配合合成寄存器 bank）、`tools/test_ads_driver_reliability.py`（驱动校验和边界）。这些测试的合成输入不冒充实录；目标构建、烧录、逐配置 HIL 结果及未通过项以根审计报告追加的证据为准。

### 后续边界补丁：rail CRC、MIXED、USB 输出绑定

本节对应 `8d6d656` 之后的完整帧边界与传输补丁，已包含在文首关联的 `ebabe7e` 镜像中；不能把下面新增字段归到未包含这些改动的旧镜像。旧字段和测量 CRC 覆盖范围保持不变。

- `RAILSAMPLE` 追加 `accepted/crcErrors/crcRetries/terminalError`。accepted 是通过 CRC、freshness、配置 generation 和 STATUS 检查的采样数；成功需 5 个合格转换，其中前 4 个丢弃。n 仍是外层 fresh-read 调用数，内部 CRC 重试不另外增加 n。crcErrors/crcRetries 是本笔 rail 事务的计数；整个 5 次转换共享 **1 次** CRC 重试预算，不能每个样本重置预算。坏帧不计入 accepted 或有效丢弃数，每次重试等待新 DRDY，沿用同一 **250 ms ADS owner 截止时间**。terminalError 只描述最后采样结果；其为 0 不排除 RAILPROFILE 的后续合法性或 cleanup 错误。
- `RAILCRC` 仅在本次 rail 事务发生 CRC 错误时输出，与 RAILPROFILE/RAILSAMPLE 按 session + seq + txn 关联。firstUs 是首个坏帧的单调 µs；bytes/len/received/computed/physicalGen/drdyGen 与 ADSSPIBYTES 的 ADC 响应语义一致，最多保留 6 字节，不是全部重试样本。即使后来恢复成功，仍保留这条首错证据；日志缺失不能清零错误计数。
- `MIXED_RAIL_BOUNDARY` 记录完整混合帧**首点之前**的预备事务。firstRes 是活动 profile 中用于建立 RES 激励的首个物理 S 行；restoreMode/restoreRow 是进入前要恢复的路由。originalError 与 cleanupError 分开，restored 仅表示原路由及 reference-ready 等待成功，不取代 cleanupError。startUs/endUs 包含选行、受控 RES 路由建立、内部 rail 事务、原路由/行恢复和参考等待；这是**外层完整预备耗时**，不能声称整体受内部 250 ms 单一截止时间限制，也不能把 monitor 采样耗时当全部暂停时间。有效 rail 无需预备或早期拒绝时可能没有该行。
- MIXED 只对活动 profile 中明确的 RES 行建立参考；先选中目标行再进入 RES。无 RES 且无真实隔离能力时，不为 CAP/VOLT 额外开启参考。预备失败时 CAP/ADS 子段均未开始，整帧 acquired=0；标准 ADSFAULTCTX 同时保留当前 seq 的原错与 cleanup，attempted=0/acquired=0。该上下文的 mode=MIXED、route=unknown 表示没有保存收尾前路由证据，不能拿恢复后的 SAFE/CAP 冒充首次故障路由。
- 内部新增 `sensorarrayMixedRowPrepareRailAtBoundary`、`sensorarrayAdsMatrixEngineRecordMixedBoundaryFault` 及 plan 的 `auxiliaryTransactionsAllowed`，都不是 Host 命令。普通 plan 明确允许完整帧边界事务，MIXED 子段明确禁止 fallback 刷新；当前预备调度比 rail 最大帧龄提前 1 帧，不延长合法帧龄，也不把失败前旧 rail 重记为新测量。battery/ADSCHK 继续在完整 RunOneFrame 返回后串行执行。新 mask/plan 标记不重解释 gen/rid 或科学数据单位。

`USBSTREAM?` 追加 `stdioBound/initError`。stdioBound=1 表示固件已经验证共享 stdout/stderr FILE 指向它自己的有界 USB 输出 VFS，协议和普通日志可由同一 TX owner 排队；它不表示 PC 已读、COM 已枚举或所有排队数据已送达。initError 是最近一次初始化返回的 ESP-IDF 错误码；默认 0 单独不能证明完成初始化，应与 stdioBound、driverInstalled 一起看。stdioBound=0 时队列拒绝提交，不能静默退回第二个直接 USB writer；此初始化门禁的拒绝发生在 sinkQueued/sinkDropped 更新之前，不能只看这两个计数判断初始化成功。命令 RX 仍由唯一 Serial reader 使用 USB Serial/JTAG 驱动读取，不依赖 stdout/stderr 绑定；enumerated 继续明确 unknown。

`3fb2a19` 曾试验完整 RDATA1 后追加 NOP（DIN=00h），实机 `final-res8-01` 仍出现末位 CRC 错误，候选已撤回。当前 RDATA1 使用原响应长度：4 字节数据，加可选 STATUS 和校验字节，最多 6 字节；所有 ADC CRC 位继续验证。该历史试验也未改变 CRC 位置、ADSSPIBYTES/RAILCRC 的 bytes/len 或 ASCII frame CRC；Host 不需要添加 ADC 字节或改变校验范围。CRC 的底层物理/控制器原因仍未确认，不能从一次软件尝试推导其已解决。

`tools/test_mixed_rail_boundary.py` 使用**合成硬件边界**运行完整生产 MIXED 引擎和真实 ADS fault formatter，验证首点前刷新、已知 rail 的无切换路径、无 RES 拒绝激励、失败 acquired=0、恢复错误独立保留，以及旧 seq 上下文不附着当前故障。它是 host 回归，不是实机采样记录；未覆盖的 HIL 项继续标 NOT_RUN。

### 完整串口记录边界（ebabe7e）

stdout/stderr 的VFS write可以是同一printf的多个片段。固件按两个流分别组到LF后才把整行交给唯一TX owner；它从现有normal pool暂借槽，不增加永久大缓冲。中途priority回复不会进入半行。槽不足、队列满或超出2304B时，整条诊断丢弃并持续丢到LF重同步，不发布前缀；两个未完成流最多各占一个槽，其余数据与priority仍可工作。legacy同步科学帧也直接按完整packet入队，保留真实queue错误，不能由stdio的本地消费返回值推定已提交。

`USBSTREAM?` 新增lineDropped（整行诊断丢弃次数）、lineOversize（其中超长行次数）、discardBytes（丢弃诊断字节）。它们不同于driver partialWrite/timeout；sinkDropped包括队列或整行丢弃，不能将这些计数简单相加成科学帧损失。VFS返回size只确认有界本地消费（包括已计数的丢弃），sinkQueued才是完整记录本地入队，driverAcceptedBytes仍不证明PC收到。无效的非换行packet和驱动部分写还使用有界delimiter恢复，下条控制回复保持行边界；断连不保证终态送达。

USBSTREAM输出受512B控制回复容量限制。当前实录计数下完整回复约414B，所有字段类型同时取极值可能超容量，此时明确返回ERR,reason=response_too_long，不输出截断的伪完整统计。不能把一次query失败或计数缺失当零；CONFIG?的原子配置身份独立于这一统计查询。

### 2026-11-13 待验收补丁：RECOVER 终态与独立 I/O 观测

本节描述 `0d8f60f` 失败基线之后、本轮新增的源代码接口；不把它追溯为旧镜像已有能力，也不把 host 回归等同于新镜像 HIL 通过。对应能力必须从 `PROTO?` 的 `recovery_query_v1/restart_query_v1/serial_sink_v1/serial_classes_v1/work_budget_v1/work_quantum_v2` 协商；修复提交、实际烧录 SHA 和新 HIL 结果由后续构建记录关联。旧基线的 VOLT1 IDLE0 watchdog 与 USB 截断证据继续保留。

**RECOVER 的有界请求与结果。** 新增内部模块 `sensorarrayRecoveryControl`，对所有 transport 使用一个请求槽及一个最后结果缓存。RECOVER 与 MODE/ROWS/ROWMODES 共用已有 GuaranteedText 终态保留槽；先保留终态空间再 ACK，只有 ACK 被本地 control sink 接受后，采集 owner 才能在完整帧边界取走请求。该 ACK 只是本地提交证据。并发或重复文本在槽忙时明确拒绝，闲时再次提交是新请求；不会将有副作用的 RECOVER 重发当查询或去重令牌。ACK 本地发布失败时不运行硬件，缓存 `rejected/ack_publish_failed` 并释放保留槽。

请求保存接受时完整不可变 identity。执行前逐一比较 known、session、bootId、revision，以及 mode/rows/profile 的 generation 和 requestId；任何变化均终结为 `rejected/stale_config`，不碰较新配置。未发布配置的早期启动允许 unknown→unknown 的重启请求，unknown→known 或 known→unknown 必须拒绝。RECOVER 的 ID 独立于 MODE/ROWS/ROWMODES 和 RESTART；uint32 加一、跳过 0，同 boot 内不按不同域 ID 的大小推断顺序。跨完整计数回绕或身份不明确时重新查询；跨 session 不比较 ID，缓存也不跨启动保存。

完成后先写最后结果，再将 `RAPP,cmd=RECOVER,...` 放入保留的优先终态槽。既有三类前置拒绝仍用 RERR：stale_config 保留 currentRev，wait_frontends 保留 route=UNKNOWN，fdc_restart_required 保留 restartRequired=1；新增公共结果字段与 RECOVERY? 的 rejected 语义一致。`state=applied` 表示恢复动作成功；`rejected` 表示拒绝或动作失败；`restarting` 只表示有界等待后准备请求 reset，不能证明 reset 已发生或终态已到达 Host。重启后的新 session/BOOT/CONFIG 才建立新身份。`RAPP` 的旧 `err` 仍是原始错误，新增 `cleanupError` 单独保留首个收尾错误；`physicalSafe=unknown` 始终明确没有模拟节点实测保证。`routeApplied/routeKnown/routeVerified` 只来自 route 的软件与寄存器快照，任何本次 cleanup 错误都使结果中的三个字段为 0。

新增无副作用 `RECOVERY?` 返回单次原子缓存副本，不运行探测、不增加请求 ID、不改变恢复状态。其字段为：

| 字段 | 归属与解释 |
| --- | --- |
| session / bootId | 当前启动身份；session 为 16 位十六进制随机标识，沿用前文碰撞及复位边界 |
| busy / phase | 是否有已接受请求；phase 为 none/pending/running，不代表硬件已安全 |
| pendingId / pendingLevel / pendingKnown / pendingRev | 当前请求的 RECOVER ID、0..2 级别和接受时配置；空闲时为 0 |
| lastId / lastLevel / last / reason | 本 boot 最后一个已接受请求的结果；尚无结果为 lastId=0、last=none、reason=none；不保留无限历史 |
| acceptedKnown / acceptedRev | 最后结果对应请求的接受时配置，不能当完成时配置 |
| resultKnown / rev / boundary | 完成时原子配置快照的 revision 与 appliedFrameBoundary；known=0 时数字 0 是未知占位，不是合法已应用边界 |
| originalError / cleanupError | ESP-IDF 首错及独立首个收尾错误；完整 uint32 十六进制，不把所有 0x108 叫 timeout |
| routeApplied / routeKnown / routeVerified / physicalSafe | 软件路由证据和 unknown 物理安全边界；参见上文 |
| terminalQueued | 结果是否曾进入固件终态保留槽；排空或丢失后仍保留该值，绝非 PC 接收确认 |

终态丢失后，Host 用 `RECOVERY?` 的 session + lastId 匹配原请求，再用 `CONFIG?` 验证结果 revision。查询之间 CONFIG revision 改变时应重读，不把 RECOVERY 与后来配置拼成从未存在的状态。缓存只有最后一个结果；更旧结果已被替换时须报告 outcome_unknown，不能猜成功。

`RECOVER=2` 自动重启预算耗尽时，主调度先记原错，强制 mode=NONE/state=DEGRADED、停止 ADS、请求 SAFE，并单独保留 SAFE/停止错误，再发布 CONFIG 和 rejected/restart_guard。真实下一帧入口因此拒绝采集，不再让 CAP 自动重建 route。已触及硬件的 level 0/1 恢复失败采用同样停止路径，并取消尚未结束的 ADS 自动恢复，避免其随后悄悄恢复旧配置；前置 busy/stale/不支持配置拒绝不破坏现有合法配置。用户后续显式 MODE 或 RESTART 仍可尝试恢复，原有自动重启预算不因失败重新补足。`BOOT.guard=recovery_safe` 是兼容历史名称，只表示预算 guard，不能推导 SAFE 已成功。

**SERIAL?。** 该独立查询调用一次 `sensorarraySerialIoGetStats`，返回其原子统计副本；不增大已有 USBSTREAM 回复。除 driverAcceptedBytes 为 uint64 外，以下字段均为 uint32；累计计数按模 2^32/2^64 回绕，在同 session 的有界窗口内计算增量。

| 字段 | 单位和计数层级 |
| --- | --- |
| normalSubmitted / prioritySubmitted | 完整记录被 USB 驱动接受的次数；normal 包括 DATA 和普通诊断，不等同科学帧数 |
| dataSubmitted / logSubmitted | `serial_classes_v1` 增补：分别为完整科学帧／诊断记录被驱动接纳的次数；两者之和保持 normalSubmitted 的原聚合语义 |
| dataRejected / logRejected | `serial_classes_v1` 增补：因相应槽位或队列容量不足而拒收的记录；VFS 按整行首次失败记一次。不是 ADC 无效点，也不包含 lineOversize |
| normalExpired / priorityExpired | 记录从入队起的有界生命周期耗尽次数，包含排队等待；不是设备采样丢点数 |
| priorityRejected | 优先槽或队列不能接纳记录的次数；初始化门禁前的拒绝不一定进入该计数 |
| driverShortWrite | 驱动返回正数且少于本次请求字节的次数；0 返回的等待重试不计正短写 |
| driverWriteError | 驱动返回负数或大于请求长度等无效结果的次数 |
| txYield | Serial TX worker 因工作预算实际请求一次 vTaskDelay(1) 的次数，不是 USB 包数 |
| maxQueueAgeUs | 驱动接受字节时观测的最大入队年龄，µs；达到 UINT32_MAX 后饱和，不包括从未提交便过期的记录 |
| driverTxBufferBytes | 驱动 TX ring 配置容量，字节；是容量而非已发送量 |
| driverAcceptedBytes | 驱动同步接收的字节数，包含恢复 delimiter；不证明总线已完成或 PC 已读 |
| partialWrite | 保持旧语义：记录未完整提交且已有前缀被驱动接受的事件；不能与 driverShortWrite 简单相加 |

正常/优先记录分别保留固定的 250 ms/1000 ms 入队绝对期限；等待、调度让出不会重新开始计时。当前 IDF BYTEBUF 驱动按整项接受或返回 0，TX ring 覆盖最大完整 2304 B 科学记录；这解决应用层逐 64 B 提交造成的半帧，但不承诺物理断连后的整帧交付。异常正短写仍有界处理和 delimiter 恢复；Host 继续校验完整科学帧、身份后缀和序号缺口。USBSTREAM=DEBUG/FULL 不关闭 command RX、RECOVERY? 或关键回复。

**SCHED?。** 新增 `sensorarrayFormatSchedulerReply` 从 WorkBudget 取一次观测副本。`registered/initError` 说明 CPU0 idle hook 的注册状态；`idleKnown` 说明是否实际看见 CPU0 idle callback，不能用 scheduler 主动计数冒充 idle。`tickHz` 为 RTOS tick/s。新增能力 `work_quantum_v2` 区分本地工作预算与真实 idle 保护：`quantumTicks=2`（100 Hz 时 20 ms）和 `maxUnits=32` 限制每个 worker 的本地工作批次；主动等待后只重启该本地账本，不更新 idle 时间或 generation。`budgetTicks=4/backstopTicks=4`（40 ms）仍表示真实 idle 年龄达到门限后请求让出，并非 SDK 调用耗时的绝对上限。旧字段 `graceUnits=0` 明确不再使用每 idle epoch 的两条记录额度。未知 idle 或 hook 注册失败仍用有限让出策略，不能伪造 idle 进度。旧 `work_budget_v1` 设备仍可能报告 budgetTicks=2/graceUnits=2；Host 按协商能力解释策略，保留原字段域。均不是采集 FPS 配额。`idleGen` 在真实 callback 中递增、跳过 0。`idleAgeTicks/maxIdleGapTicks` 分别为距离最近 callback 的 tick 数及已观测最大间隔（包括尚未闭合间隔）；未知时输出 unknown，不把 UINT32_MAX 当超长实测值。

`bt/tr/log/ser/bc/sc/wc` 每项都是 `checkpoints/yields/maxBurstUnits` 三元组，依次对应 BLE TX、transport、async log、Serial TX、BLE CTRL、Serial CTRL、Wi-Fi CTRL。checkpoint 的工作单元由对应 worker 循环定义，不是统一帧数；yield 只表示真实请求了一次有界 vTaskDelay(1)，不保证 idle 当时已运行。各计数独立取样、按 uint32 回绕；它们既不是原子配置快照，也不是 CPU 利用率。注册失败仍可有 bounded fallback，但 idleKnown 必须保持诚实；不得从一次 SCHED 回复证明长期没有 watchdog 问题。

本轮实际 host 回归 `tools/test_recovery_control.py` 编译执行 6 个生产模块和 19 个 main 函数/实际前置分支，硬件返回均为明确的合成边界，覆盖预算耗尽、SAFE 失败、原错/cleanup、FullRescue 失败、ACK 失败、终态丢失查询、10 类身份变化、3 transport 并发、重复请求、回绕、恢复后合法 MODE，以及 early/runtime 四个查询及两条 RESTART 命令的真实 dispatch。极值实际输出：RECOVERY 450 B、RAPP 330 B、带旧字段的 RERR 335 B（384 B 含 NUL 容量）、SERIAL（含 DATA/LOG 分类字段）435 B、七 worker SCHED（含 graceUnits/backstopTicks/quantumTicks）508 B（各查询 512 B 含 NUL 容量）；所有 snprintf 越界返回错误，不发布截断文本。新恢复缓存结构在 host ABI 下为 264 B 静态状态，目标 ABI 内存增量由实际构建记录确认。原 RTC 重启预算回归保留在 `tools/test_restart_budget.py`，旧 RECOVER 身份覆盖移入新测试并增加 revision fencing。上述通过不代表 HIL 或物理安全验收已完成。


**显式 RESTART 的独立请求槽（同轮补齐）。** 旧生产 `sensorarrayRestartPost/Take` 可复现：先后 RACK 接受 id=1/2 后只执行 id=2，或 id=3 的 ACK 发布失败后仍执行，甚至 Take 可早于任何 ACK 回调。这是软件控制缺陷，反例源函数和输出保存在 `docs/audit/artifacts/load_reliability_fix_20261113/restart-control-before-01.json/.txt`；该实录是生产函数配合**合成命令交错/失败**的 host 执行，不是设备重启证据。

本轮新增 `sensorarrayRestartControl`：所有 transport 共用独立单 pending 和同 boot 最后结果缓存，已接受 pending 不可覆盖，后续提交明确返回 `ERR,cmd=RESTART,reason=pending`。只有对应 RACK 本地 control sink 接受成功才开放 Take；失败缓存 rejected/ack_publish_failed、保留 originalError，释放 pending 以允许未来显式请求。它不占配置/RECOVER 保留槽；主任务的正常、初始化失败和采集任务创建失败三个执行点都使用同一 helper，且手动重启优先于前端门禁，不进入 I²C/SPI 锁。配置 revision 改变不会撤销用户的显式整机重启意图，acceptedRev 与完成时 rev/boundary 分开记录；这与 RECOVER 的 stale_config 拒绝语义不同。

执行前先缓存 restarting，再把完整、含原 `kind=manual` 的 RAPP 显式交给 SerialIo 的优先队列，并通过已有优先 Lifecycle 通路提交网络，给出有限发送机会后进入重启路径（此历史实现调用esp_restart；当前显式manual入口见[增量契约](#manual-restart-contract)，generic/AUTO不变）。发布失败保留 publishError/terminalQueued=0，不把本地控制队列拥塞变成永久无法重启；此时 ACK 已成功，不是 ACK 失败后偷偷执行。restarting 状态持续 busy 直到实际 reset，不能因迟到 ACK/重复 Take 再执行一次。既有自动恢复预算不因手动重启伪造健康而清零。

新增 `RESTART?` 和 `restart_query_v1`。回复字段为 session/bootId、busy/phase、pendingId/pendingKnown/pendingRev、lastId/last/reason、acceptedKnown/acceptedRev、resultKnown/rev/boundary、originalError/publishError/terminalQueued、serialQueued/serialError/netQueued/netError。revision 的 known 与取样域沿用 RECOVERY? 规则；originalError 是本地 ACK 发布失败，publishError 是重启意图终态的本地发布错误，二者不是硬件 reset 结果。serialQueued/serialError 与 netQueued/netError 分别记录 Serial 完整记录和网络 descriptor 的本地提交结果；publishError 保留首个非零发布错误，不能用另一 sink 成功掩盖它。terminalQueued 只表示至少一条本地通路接纳，不是 PC 接收或两路都成功。网络 sink 全关时，旧 PublishLifecycle 的 ESP_OK 是无动作；本轮新增 PublishLifecycleObserved 从真实 Queue 路径取得 outQueued=false，不能把它当网络排队成功。Serial-only 仍显式发布 RAPP。查询无副作用且原子复制独立缓存；它不能与 CONFIG? 拼装原子整体状态，应按 session 和 revision 检查一致性。

RESTART ID 是独立 uint32 计数器，跳过 0，不与 RECOVER/MODE 的 ID 比大小。真实重启产生新 session、清空缓存；Host 在重启后通过新 BOOT/CONFIG 身份确认设备回归，旧请求可能仍为 outcome_unknown。不得用重发 RESTART 模拟查询，不承诺跨 boot 网络 exactly-once。查询结果 restarting 是意图，既不证明 reset 已发生，也不证明终态已到达 Host。

新增回归执行真实模块、ACK callbacks、两个命令 handler 分支和主重启执行 helper，验证上面的三个旧反例、忙拒绝、迟到 ACK、优先发布失败后查询、配置变化/前端 quarantine/配置槽占用时独立逃生、3 transport 并发、有限计数回绕及新 boot 清缓存。极值实际输出 RESTART? 413 B（512 B 容量）、RAPP 191 B（384 B 容量）；新缓存 host ABI 为 80 B。SCHED? 的 wc 第七组三元组表示 Wi-Fi CTRL，与前六组同单位；七组最大输出为 437 B。所有 reset 与硬件边界在这些回归中是合成 mock，目标构建和实机结果仍需另行记录。


针对终态 sink 的追加回归直接执行生产 `sensorarrayTransportQueue`、`sensorarrayTransportPublishLifecycleObserved` 和 `sensorarrayTransportDrainGuaranteedText`：Serial-only 的 RESTART 确有完整优先 RAPP；USB 拒绝和网络队列拒绝各自保留错误；禁用网络的 ESP_OK 无动作保持 netQueued=0。RECOVER 的既有 GuaranteedText drain 原本就显式调用 SerialIoQueue，然后独立尝试 BLE/Wi-Fi，因此没有相同的 Serial 遗漏。RECOVERY.terminalQueued 仍只描述保留槽曾写入，与 RESTART.terminalQueued 的“至少一个本地输出队列接纳”不同；两个字段都不能证明 Host 接收。测试中注入 USB 拒绝仍保留 RECOVERY 结果，真实 sink 丢弃由原分层统计可查。

## USB-OTG formal data / 正式数据扩展（2026-11-14）

Implementation revision is pending the final validated commit in this phase. The wire protocol remains PROTO 2 ASCII with the existing real-encoder CRC and immutable session/config identity. This extension changes the USB interface, not the meaning of legacy frame `gen/rid`. / 本阶段修复 commit 待最终构建验证后记录；ASCII、CRC、各身份域不变。

`PROTO?` selects capabilities from the active backend. `usb_vendor_bulk_v1` replaces `serial_sink_v1/serial_classes_v1` on OTG. `usb_rom_handoff_v1` announces the explicit `USBBOOT` action. A one-boot USJ recovery backend reports its own capabilities; do not infer OTG from the firmware version alone. Query `USB?` to confirm the active backend.

| Interface | Class/subclass/protocol | Endpoints | Contract / 契约 |
| --- | --- | --- | --- |
| 0 DATA | FF/53/01 | OUT 01, IN 81 | Full scientific ASCII packets. DATA OUT is discarded. / 完整科学帧；OUT 不解释命令。 |
| 1 CTRL | FF/53/02 | OUT 02, IN 82 | Commands, replies and lifecycle/fault events. / 命令、回复、模式终态与 MFAULT。 |
| 2/3 CDC | CDC ACM | notify 83; OUT 04, IN 84 | Best-effort debug only; RX is discarded. / 可丢弃调试日志，不接收控制命令。 |

The prototype uses Espressif development VID/PID `303a:4041`, bcdDevice `0100`, serial equal to the uppercase six-byte factory MAC without separators (this board `80B54ECDD9AC`). This is not a uniquely allocated production PID. Exact role/interface/endpoint matching is required: old `303a:1001` contains a vendor JTAG interface and MUST NOT receive these ASCII commands. MS OS 2.0/BOS descriptors bind only the two vendor functions to WinUSB with stable distinct interface GUIDs; CDC uses the serial driver. Windows binding is NOT_RUN until actually tested there.

每个 IN 端点是独立字节流。主机必须分别缓存半行；不能先交织原始 chunks 再解析。一帧可以跨多个 USB 包／传输，一个传输也可以含多行。64 字节包、短包、ZLP 和读超时均不是帧边界。CONTROL 终态可能早于或晚于 DATA，仍按 CONFIG session/revision/boundary 契约接纳和恢复，不能靠跨端点到达先后推断配置。模式 action 结果未知时不得自动重放完整命令。

Each IN endpoint has an independent byte stream and partial-line buffer. Neither USB packets nor transfer boundaries delimit scientific frames. Preserve nonzero transferred bytes on timeout and write only the known unsent suffix. A disconnect ends the capture window; reconnect queries PROTO/CONFIG anew and never compares sequence/generation across sessions. Preserve DATA and CTRL raw streams separately; a merged synthetic stream is not original evidence. The capture tools label these `.rx.bin` and `.ctrl.rx.bin`; CDC uses its own `.debug.rx.bin` observer and is excluded from formal data receipt counts.

All three TX lanes reuse the existing bounded persistent application pool. A single CPU0 owner services TinyUSB and independent endpoint offsets/deadlines. CTRL is serviced first, DATA next, CDC last. Partial expiry requires a delimiter only on that endpoint. Bus reset/deconfiguration increments a connection epoch; old queued records are discarded with their original identity. The owner first proves the queued reset event was processed before admitting new-epoch commands. An ISR notification alone is insufficient.

USB-OTG 使用 S3 内部 Buffer DMA，不是 Serial/JTAG DMA，也不是 GDMA。最大 USB FS 包仍为 64 B；vendor DMA staging 为 512 B（每传输最多八包）。TinyUSB FIFO/端点缓冲在内部 DRAM、4 字节对齐并持续存活；PSRAM 仅存 CPU 层队列文本，写入 TinyUSB 会复制，不能称零拷贝。编译启用 DMA 不是实机证明；`USB?.dma` 读实际控制器 enable 位。

`USB?` reports stack initialization, mount/configuration, suspend, connection epoch/reset observations and RX counts/times. `USBQ?` separates queue rejection, whole-record submission, FIFO-accepted bytes and transfer callback bytes. TinyUSB 0.21's vendor callback drops the transfer result; therefore `callbackResult=unknown`, never successful-bus or PC-application receipt. `SERIAL?` on OTG explicitly identifies CDC debug. `USBSTREAM?` returns BULK/dataEvery=1; DEBUG/FULL setters reject as legacy-only. `ST` still controls wired versus wireless sink eligibility; control is independent of DATA selection.

`USBBOOT` uses the existing independent RESTART request ID and local ACK publication guard, with `target=rom_download`. It does not acknowledge PC receipt. At a main-loop boundary the USB owner stops service, disables its IRQ, clears incompatible ROM persistence flags, asserts and verifies the OTG hardware reset, and returns the PHY to USJ before the ROM-download restart. No TinyUSB object or DMA buffer is freed/reused. Ordinary controlled application restarts use the same fence. If the owner cannot acknowledge by the bounded deadline, the firmware uses an ESP32-S3 digital Core Reset and latches subsequent boots into USJ recovery until explicit USBBOOT prepares and reads back the armed ROM flag (repair `3a55543`); it does not claim local cancellation or enter ROM download on that failure. The panic restart call is wrapped so CPU-only reset cannot silently leave OTG DMA using memory reinitialized by the next boot. These terminal paths retain watchdog behavior; physical BOOT/EN remains a recovery route. Do not burn USB eFuses.

BLE 正式 DATA 的应用优先级为 7，CTRL 为 8；USB owner 和 transport 为 6。编码/output 仍为 7，保证采样结果被送入各 sink；普通日志不能占用 DATA admission slots。BLE 生命周期沿既有 FF30 wire 格式使用保留优先槽，不与普通 LOG 排队。`BLELINK?` 公布实际 PHY/connection completion、请求错误和 known 标志；interval 单位 1250 µs、supervision timeout 单位 10 ms。IDF DLE completion 没有 peer ID，故 `dlePeerKnown=0`，其数值只是未归属观测。`BLETX?` 记录本地服务及 SDK 调用时间（µs），不证明空口／PC 完成。IDF 单次 GATT API 内部仍有可阻塞的 BTC 入队；外层 retry deadline 不等于已取消该 SDK 调用，slot 不会提前释放。

The host software repository was not modified. Its separate integration task must add libusb/WinUSB device discovery and independent endpoint readers, preserve the existing session/config gate, support terminal-loss recovery by CONFIG, and keep CDC/BLE diagnostic loss separate from scientific data loss. Wi-Fi wire behavior is unchanged; no new Wi-Fi HIL result is claimed.

## 双 CDC 替代接口 / Dual-CDC alternative (2026-11-14)

修复提交 / Firmware commits: `3dc0c48e92f080cad334cce50614d07c3df6e38c` adds dual CDC; `4a183bca6a5a0566a834a727a58d5e25a80ad31b` adds bounded detach and rejects late cleanup ACK; `3a55543cb21fa3412ede26ee79c1cce7d8371804` retains the recovery latch and supplies the complete descriptor ELF hash. 当前已实际烧录及查询确认 / flashed and queried BUILD: `3a55543`, binary SHA256 `59d3f07d772730692ea4a68e9af6437779ddfd24849c78f8fb9a03fbb2e31fee`, ELF SHA256 `d4ac82a199bd2bc8bb730b331225ffdbb94acd2c84c410cd675636da706e92e5`. [软件恢复实录 / Software recovery evidence](../audit/cdc_recovery_acceptance_20261116.md).

BUILD 的 ELF 字段固定为完整 64 位十六进制；`4a183bc` 的实录只有 9 位，属于已修复的查询缺陷，不允许主机把前缀扩写为完整哈希或混同 binary SHA。本仓库捕获工具对精确、无关联身份的 `ERR,cmd=UNKNOWN,reason=unsupported` 仅在白名单只读查询中保存并继续等待原期限；明确命令错误、模式 setter、Applied 请求 ID 门控保持严格。 / BUILD returns 64 ELF hash hex digits. The observed nine-digit `4a183bc` field was a query defect: do not expand it or confuse it with the binary hash. Capture tools may retain an uncorrelated exact UNKNOWN during allowlisted read-only queries and continue within the existing deadline; explicit errors, setters and Applied request IDs remain strict.

Linux 设备节点可能先出现，再由 udev 应用用户组权限。主机应在同一连接期限内重新核对 serial/interface/devnum 并等待访问就绪；不得因短暂 EACCES 再发 RESTART、修改权限或要求管理员。 / A Linux tty can appear before udev applies group permissions. Within the same connection deadline, revalidate serial/interface/devnum and wait for access readiness; transient EACCES must not trigger another RESTART or an administrator requirement.

本节扩展上节 OTG 契约：用户要求当前调试不安装管理员权限规则，因此默认 `CONFIG_SENSORARRAY_USB_OTG_CDC=y`，仍使用原生 OTG DMA。设为 n 才构建上一节的 vendor backend；二者不能同时占用 S3 的 IN 端点。旧 vendor 记录和主机工具保留，不能把其数据伪称为新 CDC 数据。实现提交及 HIL 结果以阶段 manifest 为准，尚未声明完整验收通过。

This extends the preceding OTG contract. To continue without installing an administrator permission rule, `CONFIG_SENSORARRAY_USB_OTG_CDC=y` selects the default dual-CDC backend, still using native OTG DMA. Set it to n for the preceding vendor backend. Both cannot occupy S3 IN endpoints simultaneously. Historical vendor records and tools remain distinct; CDC acceptance is not inferred from them. Exact source/build/HIL identity is recorded in stage manifests; full acceptance is not yet claimed.

| 属性 / Property | 双 CDC / Dual CDC |
| --- | --- |
| Capability / backend | `usb_cdc_data_v1` / `USB,backend=otg_cdc_v1` |
| 开发 VID:PID / Development identifier | `303a:4042`, bcdDevice `0100`; not an allocated production product ID |
| 板卡身份 / Board identity | Factory MAC, uppercase 12 hex digits without separators |
| 正式 DATA+CTRL / Formal stream | CDC0, IAD interfaces 0/1; notification IN81, bulk OUT02/IN82; MPS64 |
| 调试 / Debug | CDC1, IAD interfaces 2/3; notification IN83, bulk OUT04/IN84; MPS64 |
| 发现 / Discovery | Verify VID/PID, serial, CDC interface 0 (`02/02/00`) and capability; never tty index alone |
| 权限 / Permissions | OS `cdc_acm`/CDC driver and existing port ACL; no libusb raw-device claim; not a guarantee of every account's access |
| 输出设置 / Output setting | `USBSTREAM,v=CDC,scope=otg_data,dataEvery=1,debug=cdc,control=shared_cdc`; legacy setters reject |
| 证据 / Evidence streams | One original formal `.rx.bin` containing DATA+CTRL; separate `.debug.rx.bin`; no invented CTRL raw stream |

CDC0 接收 ASCII 命令，正式 DATA 和优先 CTRL 共用同一 FIFO。发送方在记录边界先服务 CTRL。带有 `USBQ?.dQueueCap` 的固件会等 FIFO 有一整帧空间后再提交 DATA。PSRAM 正常时 DATA 可排队 100 帧，另有一个发送中的槽位，每帧自入队起有效 1,000,000 µs。小池降级时可排队 4 帧，每帧有效 250,000 µs。`dQueueCap` 是队列名额，`dLifetimeUs` 是当前 DATA 的有效期。旧固件没有这两个可选字段。主机按实际到达的 K、CRC 和序列确认接收。队列满或积压过久仍可能整帧缺失。连接重置、旧版短写过期或驱动异常也可能留下残片，接收方保留原始字节并记录缺口。

CDC0 accepts ASCII commands. DATA and priority CTRL share its FIFO, with CTRL served first at record boundaries. Firmware reporting `USBQ?.dQueueCap` waits for space for the entire DATA record before copying it into the FIFO. The PSRAM queue holds 100 waiting frames plus one in service. Each record has a 1,000,000 µs lifetime from enqueue. The small-pool fallback holds four waiting frames with a 250,000 µs lifetime. The optional `dQueueCap` and `dLifetimeUs` fields expose the active limits. Earlier builds omit both. Validate host receipt through the trailer, CRC and sequence. A full queue or an expired record can still produce a gap. Preserve fragments across reads and retain raw evidence when a bus reset, older firmware or a driver error leaves a partial record.

CDC1 有独立的调试 FIFO，输入有界丢弃。正式口可在 DTR 未设置时工作，调试口需要 DTR。USB reset 终止该连接 epoch 并清空 FIFO，主机重新查询 CONFIG。DTR 只表示主机线路设置。 / CDC1 has a separate debug FIFO and bounded input discard. Formal traffic can proceed with DTR clear. Debug output requires DTR. A USB reset ends the connection epoch and clears the FIFO. Query CONFIG after re-enumeration. DTR reports the host line setting.

两种 OTG backend 都保持同一 session/boot/revision/mgen/rgen/pgen、MODE/ROWS/ROWMODES 终态与 ASCII CRC 契约。共享 CDC 的字节顺序只说明这个端点上的发布顺序；它仍不代表 MAPP 后已有首个有效数据，也不消除 BLE 的异步到达。跨连接重读 `PROTO?/CONFIG?`，跨 boot 不比较 sequence/generation 大小。命令结果未知时用权威查询恢复，不自动重放有副作用动作。

Both OTG backends retain the same session/boot/revision/mgen/rgen/pgen, transition-terminal and ASCII CRC contracts. A shared CDC stream establishes only publication order on that endpoint; MAPP still does not prove first valid data, nor order BLE delivery. Query PROTO/CONFIG after reconnect, never compare counters across boots, and resolve unknown command outcomes through authoritative queries rather than automatic action replay.

`USBQ.dBytes/cBytes` 仍是应用 lane 被 FIFO 接受的字节；CDC callback 无法提供按 lane 的完成字节，因此 `dCallbackBytes/cCallbackBytes=unknown`，不是 0 次完成。`BUILD?` 新增 `version`（运行 app descriptor）与完整 64 个十六进制字符（256 bit）的 `elfSha256`；后者是 ELF hash，不是烧录 `.bin` 的 SHA256，二者必须分别记录。

`USBQ.dBytes/cBytes` remain FIFO-accepted application-lane byte counts. CDC callbacks cannot provide per-lane completed bytes, so callback byte fields are `unknown`, not zero completions. `BUILD?` adds running app-descriptor `version` and a complete 64-hex-digit `elfSha256`. The ELF hash is different from the flashed binary SHA256; record both.

主机仓库交接：支持标准串口 API 的正式 CDC0、单流半行缓存、独立可选 CDC1 日志；保留 vendor 可选接收器和原配置门控。不能把 debug 文本帧计入正式实收。Linux 当前普通用户已属 dialout，可直接访问；Windows 自动 CDC 驱动绑定有官方支持，但本板 Windows 枚举／吞吐仍 NOT_RUN。[Linux CDC ACM](https://docs.kernel.org/usb/acm.html)，[Microsoft CDC 驱动](https://learn.microsoft.com/en-us/windows-hardware/drivers/usbcon/usb-driver-installation-based-on-compatible-ids)。

Host handoff: use ordinary serial APIs for formal CDC0 with one partial-line buffer and an optional independent CDC1 debug reader. Retain the optional vendor receiver and existing configuration gate. Never count debug text as formal data receipt. This Linux account already has dialout access. Windows CDC driver support is documented, but this board's Windows enumeration and throughput remain NOT_RUN. No host-repository edits are included here.

下载交接的后续修复 / Subsequent ROM-handoff repair: the owner now retains the detached OTG PHY for at least 20 ms before switching to USJ. The existing 250 ms total deadline includes this engineering dwell. A cleanup ACK arriving after the deadline returns TIMEOUT and does not arm ROM; verified cleanup is retained. This does not prove the host observed a physical detach. RACK/RAPP and BLE disconnection are insufficient: confirm OS removal/addition and the ROM programming interface before attempting a write. The first real handoff on the earlier image failed this observation; the new dwell still requires HIL. / owner 新增至少 20 ms 的有界断开窗口；迟到 ACK 不再伪报按时成功。主机必须核对下载口实际枚举，不能把 RACK/RAPP 或 BLE 断连当作 ROM 已就绪。

## 有界 ADS 时序诊断 / Bounded ADS timing diagnostic (2026-11-16)

本轮新增 `ads_timing_probe_v1`；旧 `ADSCHK` / `ADSCHK=<n>` 的语义不变。`ADSCHK=TIMING,200` 是显式、单次诊断动作，固定运行四组各 200 次主读：0=CONT/gain1、1=PULSE/gain1、2=CONT/bypass、3=PULSE/bypass。输入 MUX 为 AA 内部同节点短路，MODE1=0、MODE2=0F/8F，数据率仍为 38,400 SPS。诊断不发布 C/V/R/M 科学数据，不进入 PGA/value cache，不更换 SPI 时钟、不改变 GPIO/POWER/参考路由，也不放宽科学 CRC 或有效性规则。执行完成不是链路资格或精度验收通过。

The new `ads_timing_probe_v1` capability adds the explicit single-action `ADSCHK=TIMING,200`; existing ADSCHK commands retain their semantics. Four groups each perform 200 primary reads: 0=CONT/gain1, 1=PULSE/gain1, 2=CONT/bypass, 3=PULSE/bypass. The input is AA (both inputs select the same internal node), MODE1=0, MODE2=0F/8F, at 38,400 SPS. This produces diagnostic records only, never scientific C/V/R/M samples or PGA/value-cache updates. It does not change SPI clock, GPIO, POWER, reference routing, or scientific CRC/validity requirements. Completion does not mean link qualification or accuracy acceptance.

存储修正：`bb230d0` 实机在尚未运行诊断前出现 `BOOT/READY stage=scan_task_create_failed,err=0x101`。该版三个结果槽各占 2,976 B 内部 BSS，共 8,928 B，挤占采集任务所需内部内存。后续实现把三个 CPU-only 槽一次性预分配到显式 `SPIRAM|8BIT`，无内部 fallback、无硬件事务内分配，也不释放仍可能被查询/worker 引用的槽；metadata/mux 留内部。`ADSTIMING?` 原子公开 `storageKnown/storageReady/storageBytes/storageError`。分配失败时正常启动及矩阵仍继续，仅动作返回 `ERR,cmd=ADSCHK,reason=diagnostic_unavailable,err=…`，不生成 accepted ID 或占用配置仲裁。能力存在不等于当前存储可用，主机发动作前必须查 storageReady。该资源修复尚需主任务重新构建和 HIL，旧失败实录保留。[失败实录](../audit/artifacts/cdc_recovery_acceptance_20261116/timing-startup-failure-01.json)，[存储回归](../audit/artifacts/cdc_recovery_acceptance_20261116/timing-storage-fix-host-01.json)。

Storage repair: before any diagnostic ran, `bb230d0` reported BOOT/READY `stage=scan_task_create_failed,err=0x101`. Its three 2,976-byte result slots consumed 8,928 bytes of internal BSS needed by acquisition. The follow-up allocates all three CPU-only slots once from explicit SPIRAM|8BIT, without internal fallback, allocation inside a hardware transaction, or freeing storage referenced by a query/worker. Metadata and mutex remain internal. ADSTIMING? atomically exposes storageKnown/storageReady/storageBytes/storageError. Allocation failure disables only the diagnostic; normal startup/acquisition continues, and the action returns diagnostic_unavailable without an accepted ID or configuration reservation. Advertised capability does not imply available storage: query storageReady before requesting the action. Target rebuild/HIL remain the main task's responsibility; the original failure evidence is retained in the linked records.

命令沿用 MODE/ROWS/ROWMODES 的全局请求 ID 与单槽仲裁，但不修改这些配置域的 generation。只有 ACK 已成功本地发布后，Core 1 才可在完整帧边界消费；接受时保存 session、boot 和全部配置域身份，执行前必须仍相等。只允许已知 CAP/VOLT 路由、前端 owner 全部静止、INTREF 已 OFF；core 再核对 START=0、VBIAS、CRC/STATUS、AVDD/AVSS 参考和物理/缓存寄存器。RES/MIXED、SAFE/未就绪、旧 revision 或未知 owner 明确拒绝，不为诊断开启参考。软件寄存器验证不代表模拟节点或物理安全实测。

The action shares the global configuration request-ID allocator and single outstanding-request arbiter, without changing mode/rows/profile generations. Core 1 can consume it only at a complete-frame boundary after successful local ACK publication. Admission captures session, boot and every configuration-domain identity; all must still match at execution. Only a known CAP/VOLT route, quiescent frontend owners and INTREF already OFF are admitted. The core additionally checks START=0, VBIAS, CRC/STATUS, AVDD/AVSS reference and physical/shadow registers. RES/MIXED, SAFE/not-ready, stale identity or unknown ownership is explicitly rejected; the diagnostic never enables a reference. Register verification is not a physical-safety or analog-node measurement.

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
| `ADSTIMING?REG,<id>,<0|1>` | `ADSTIMINGREG`：0=saved、1=restored；known 与 ID/POWER/INTERFACE/MODE0/1/2/INPMUX/REFMUX/OCAL/FSCAL / register evidence |
| `ADSTIMING?RECORD,<id>,<index>` | `ADSTIMINGRAW`：平面索引 0…records−1，最多 32 条；raw/status/CRC/fresh、时间、配置/DRDY generation 与有界原始字节 / bounded raw diagnostic records |

详情只接受当前最后结果的精确 ID；结果被后续完成请求取代后，旧 ID 查询明确失败。主机应在详情集合前后查询 ADSTIMING?/CONFIG?，核对 session、lastId 和结果 revision，避免拼接不同结果；不要因 ACK/终态丢失重发动作。元数据 `resultKnown` 表示终态时配置身份可知，`result` 表示诊断详情缓存可用；二者都不等于 restoreVerified。`terminalQueued` 只表示本地终态槽接受，不证明 USB/BLE/Wi-Fi 或 PC 收到。跨 boot 不比较 ID/sequence/generation；有限 ID 回绕跳过 0，仅以当前 session 和最后结果上下文解释。

Detail queries require the exact last-result ID; an overwritten result fails explicitly. Query ADSTIMING?/CONFIG? before and after a detail set and compare session, lastId and result revision. Do not replay the action when ACK or terminal receipt is unknown. Metadata resultKnown means final configuration identity is known; result means cached diagnostic details are available. Neither means restoreVerified. `terminalQueued` describes local terminal-slot acceptance, not USB/BLE/Wi-Fi delivery or PC receipt. Do not order IDs/sequences/generations across boots. Finite IDs wrap while skipping zero and are interpreted only within the current session and last-result context.

最多每组保留 8 条分类代表记录，不是 800 条全量原始捕获；recordsOmitted 明示容量不足。好样本只保留驱动实际返回的 raw，`wireKnown=0` 且 bytes/computed/received/xor=unknown；不得补造“好帧”原始 CRC。坏 CRC 保存驱动实际错误字节，raw/status 仍是不可信诊断值，不能进入科学曲线。所有详情行有界 512 B，终态槽 384 B；完整行放不下返回错误且缓冲为空，不把截断行称为完整结果。

At most eight category-representative records per group are retained, not an 800-read raw capture; recordsOmitted exposes capacity omissions. Good reads retain only the driver's returned raw value and report wireKnown=0 with unknown wire bytes/CRC fields. No good wire frame is reconstructed. Bad-CRC records preserve actual driver fault bytes, while their raw/status remain untrusted diagnostics and cannot enter scientific plots. Detail lines are bounded by 512 bytes and the terminal slot by 384 bytes; insufficient capacity returns an error with an empty buffer rather than a truncated result.

本轮还新增只读 `BLESVC?` / `SCHEDWHY?`，由 `ble_service_obs_v1` / `work_reason_v1` 协商。BLESVC 的 boot 范围 started/completed、累计 wallUs 和 idleChanged/Same/Unknown 是 SDK 调用经过时间与真实 idle generation 观测，不是 CPU 利用率、阻塞时间或 PC 完成。SCHEDWHY 的七组 bt/tr/log/ser/bc/sc/wc 按 quantum/backstop/units 输出触发计数；`overlap=1` 表示同一 checkpoint 可同时满足多项，不可相加当独立让出次数。旧 BLETX/SCHED 语义不变。

The new read-only BLESVC?/SCHEDWHY? queries negotiate via ble_service_obs_v1/work_reason_v1. BLESVC's boot-scoped started/completed counts, cumulative wallUs and idleChanged/Same/Unknown observe SDK elapsed time and actual idle generation, not CPU use, blocked time or PC completion. SCHEDWHY reports quantum/backstop/units trigger counts for bt/tr/log/ser/bc/sc/wc; overlap=1 means multiple reasons may hold at one checkpoint and must not be summed as distinct yields. Existing BLETX/SCHED semantics are unchanged.

主机合成回归已执行 / Executed synthetic host regression: [timing-control-host-02](../audit/artifacts/cdc_recovery_acceptance_20261116/timing-control-host-02-command.json), 13 groups plus 10 admission subcases; the production control/mailbox/ScanConfig/terminal code and main boundary/callback branches ran successfully. Four existing recovery/frontend/mailbox/config regressions also passed. These are not hardware timing or CRC root-cause evidence. Target build and HIL are recorded separately by the main task; this addition does not claim runtime CRC is solved or acceptance complete.

## TAIL 诊断种类扩展 / TAIL diagnostic-kind extension (2026-11-16)

本轮新增能力 `ads_tail_probe_v1` 与显式动作 `ADSCHK=TAIL,200`，沿用上一节的单槽请求仲裁、ACK 栅栏、完整帧边界、2 秒事务期限、500 ms 恢复预算和 PSRAM 三槽。原 `ADSCHK=TIMING,200` 仍比较 CONT/PULSE；TAIL 的四组全部 CONT，顺序为 0=STD/gain1、1=GPIOtail/gain1、2=STD/bypass、3=GPIOtail/bypass，每组 200 次主读，held 计数为 0。TAIL 仅用于诊断，不切换普通矩阵/boot 读取路径，不修改资格或科学有效性阈值，也不把 CRC 失败改为有效值。

The new ads_tail_probe_v1 capability adds the explicit ADSCHK=TAIL,200 action. It shares the preceding single request arbiter, local ACK guard, full-frame boundary, two-second transaction deadline, 500 ms restoration reserve and three PSRAM slots. TIMING retains its CONT/PULSE comparison. TAIL uses CONT throughout: 0=STD/gain1, 1=GPIOtail/gain1, 2=STD/bypass, 3=GPIOtail/bypass, with 200 primary reads per group and no held reads. This diagnostic does not select a normal matrix/boot read path, change qualification/scientific thresholds, or make rejected CRC data valid.

kind 随接受时的持久请求进入 mailbox 与结果缓存，不能在发送时重贴新标签。ACK 保留既有前缀 `ACK,cmd=ADSCHK,timing=1,id=…`，追加 `probe=TIMING|TAIL`；这里 timing=1 是这套诊断控制族的兼容标记。ADSTERM 和所有详情行同样有 probe。`ADSTIMING?` 分别给 `pendingProbe` 和 `lastProbe`，无对应请求时为 none。主机必须按 session/请求 ID/probe 一致性取回结果；一个 TAIL 请求不能因后到的 TIMING 请求被解释成另一组实验。二者相互忙拒绝，不覆盖已经接受的请求。

The accepted persistent request carries its kind through the mailbox and immutable result cache; transmission never relabels it. The existing ACK prefix ACK,cmd=ADSCHK,timing=1,id=… gains probe=TIMING|TAIL; timing=1 remains a compatibility marker for this diagnostic control family. ADSTERM and every detail line also identify probe. ADSTIMING? reports pendingProbe and lastProbe independently, using none when absent. Retrieve results only when session, request ID and probe agree. A later TIMING request cannot reinterpret a TAIL experiment. The two kinds share busy rejection and never overwrite an accepted request.

除既有 RESULT/GROUP/TIME/REG/RECORD 查询外，新增 `ADSTIMING?TAIL,<id>,<flatRecordIndex>`，返回 `ADSTIMINGTAIL`。平面索引与 RECORD 相同；只接受最后缓存的 TAIL 结果，TIMING/旧 ID/越界均返回有界错误。GROUP/RAW 的 `source=spi56` 表示标准路径；`source=spi55_gpio1` 表示 55 个硬件时钟位加一个真实 GPIO 采样位。GPIOtail 的 RAW 始终 `wireKnown=0,bytes=unknown`，不得把组装数据描述为连续 SPI wire 实录。收到的末位参与完整 CRC 校验，不允许猜位或为了 CRC 通过翻位。

In addition to RESULT/GROUP/TIME/REG/RECORD, ADSTIMING?TAIL,<id>,<flatRecordIndex> returns ADSTIMINGTAIL. Its flat index matches RECORD. Only the last cached TAIL result is accepted; TIMING, obsolete IDs and out-of-range indices fail explicitly. GROUP/RAW source=spi56 identifies the standard path; source=spi55_gpio1 identifies 55 hardware-clocked bits plus one actual GPIO sample. GPIOtail RAW always reports wireKnown=0,bytes=unknown: the assembled data is not a continuous SPI wire capture. The observed final bit participates in the full CRC check; it is neither guessed nor flipped to obtain a passing CRC.

TAIL 补充记录区分 `tailApplicable`、`spiBits`、部分 `prefix`、`gpioKnown/gpioBit`、`assembledKnown/assembledCrc`、attempted/spiDone/frameDone/routeRestored。STD 组 tailApplicable=0、spiBits=56，不能把默认 tail 字段当成实际执行；GPIOtail 组 spiBits=55，prefix 最后一字节的未采位由 `lsbKnown=0` 标明，另由 gpioKnown 判断独立末位是否观测到。startUs 为单调微秒时间，spiUs/riseUs/sampleUs/fallUs/restoreUs 为相对 startUs 的微秒偏移。`routeRestored` 只描述驱动 pinmux/软件收尾证据；`physicalVerified=0` 保留，没有示波器时序或模拟安全保证。

The supplemental record distinguishes tailApplicable, spiBits, partial prefix, gpioKnown/gpioBit, assembledKnown/assembledCrc and attempted/spiDone/frameDone/routeRestored. Standard groups have tailApplicable=0 and spiBits=56; default tail fields are not execution evidence. GPIOtail uses spiBits=55; lsbKnown=0 marks the unobserved bit in the partial SPI prefix, while gpioKnown separately identifies whether the GPIO bit was sampled. startUs is monotonic microseconds; spiUs/riseUs/sampleUs/fallUs/restoreUs are microsecond offsets from it. routeRestored describes driver pinmux/software cleanup evidence only. physicalVerified=0 remains explicit; no oscilloscope timing or analog-safety guarantee is inferred.

所有行仍有界：本轮真实 C 控制回归的 ADSTIMING 状态最大 483/512 B、ADSTERM 最大 285/384 B，三个 USB backend 的实际 PROTO snprintf 最大 421/512 B。新增 tail evidence 扩大的是原 PSRAM 分配的 sizeof，不新增内部 DRAM 大数组，不回退内部 heap。存储失败仍只禁用诊断。TAIL 实机结果与根因判断须由新的独立证据证明；本节只定义接口，不把历史 TIMING 的实录当作 TAIL 验收。

Lines remain bounded: the production C control regression measured at most 483/512 bytes for ADSTIMING state, 285/384 for ADSTERM and 421/512 for actual PROTO snprintf across all three USB backends. Tail evidence enlarges the existing PSRAM allocation, without large internal-DRAM arrays or internal-heap fallback. Storage failure still disables only the diagnostic. TAIL HIL results and root-cause conclusions require new independent evidence; historical TIMING measurements are not TAIL acceptance.

[TAIL 控制回归与源码 SHA / TAIL control regression and source hashes](../audit/artifacts/cdc_recovery_acceptance_20261116/tail-control-host-01-manifest.json)：20 组及 10 个准入子例通过，另四个既有 recovery/frontend/mailbox/config 入口 exit 0；硬件均为合成边界，未由本子任务构建或烧录。 / Twenty groups plus ten admission subcases passed, along with four existing recovery/frontend/mailbox/config entries. Hardware boundaries were synthetic; this subtask did not build or flash the target.

### TAIL IO 取证与失败结果恢复 / TAIL IO evidence and failed-result recovery

本轮新增 `ads_tail_io_v1` 能力和只读 `ADSTIMING?IO,<id>,<flatRecordIndex>`，返回 `ADSTIMINGIO`；与 RECORD/TAIL 共用缓存 ID、种类和有限索引。未公告该能力的 `bc99767` 镜像仍只用原 TAIL 查询，不能把未知查询的错误当作诊断动作失败。IO 响应不扩大原 RAW/TAIL 行，也不改变科学数据或旧 TIMING 行为。

The new `ads_tail_io_v1` capability announces the read-only `ADSTIMING?IO,<id>,<flatRecordIndex>` query and `ADSTIMINGIO` reply. It shares the cached ID, kind and bounded record index with RECORD/TAIL. The earlier `bc99767` image does not advertise this capability and retains its original TAIL queries; an unsupported query is not a failed diagnostic action. IO uses a separate line and does not change scientific data or existing TIMING behavior.

IO 字段包括 `stage`、`clockKnown/misoKnown`、`clkFunc/clkSig/clkIE/clkOE/clkPeriph/clkInv/clkOD`、对应 `miso*` 字段及 `clkLevelKnown/clkLevel`。配置和电平来自准入时的软件 GPIO 观测；`stage` 为第一个失败阶段或 complete。`OE=1,Periph=1` 表示输出使能交由外设控制，不能单凭 GPIO 配置位认定 pad 当时主动驱动。`physicalVerified=0`，这些字段不证明电气安全、采样边沿或模拟隔离。STD 记录的 stage 为 not_applicable，未观测字段由 known=0 标记。

IO reports `stage`, `clockKnown/misoKnown`, `clkFunc/clkSig/clkIE/clkOE/clkPeriph/clkInv/clkOD`, the corresponding `miso*` fields, and `clkLevelKnown/clkLevel`. Configuration and level describe software GPIO observations at admission; stage identifies the first failure or completion. `OE=1,Periph=1` delegates output enable to the peripheral and does not prove that the pad was actively driven. `physicalVerified=0` remains explicit: these fields verify neither electrical safety, sampling edges nor analog isolation. STD records use stage=not_applicable and known=0 for unobserved values.

失败终态的 `acceptedRev` 与 `rev` 可以不同：例如隔离 ADS 后，配置权威状态合法进入新的 DEGRADED revision。Host 必须保留接受时 revision、终态 revision 和 closing CONFIG，仍按同一 session、请求 ID 和 probe 读取全部缓存；不能因失败产生的新 revision 而丢弃错误证据，也不能将该失败标为成功。成功诊断仍要求配置身份一致、完整 schema、所有组实际完成、terminal/cleanup=0 且恢复已知已验证。CRC-only 的 first=0x109 保留，不能据此自动宣告整项诊断执行失败或科学有效。

A failed terminal may have different `acceptedRev` and `rev`: isolating the ADS can legitimately publish a new DEGRADED configuration revision. Preserve the accepted revision, terminal revision and closing CONFIG, and continue collecting the same session/request/probe cache. A failure-induced revision must neither discard fault evidence nor be called success. Successful diagnostic completion still requires consistent configuration identity, complete schemas, completed groups, zero terminal/cleanup errors and known, verified restoration. CRC-only first=0x109 remains visible; it alone establishes neither failed diagnostic execution nor scientific validity.

本仓库采集工具新增 `--result-id <id> --expected-session <16hex>`，同时要求完整 `--expected-elf <64hex>`。此路径仅查询，不发送 ADSCHK、模式 setter、重启或烧录；旧 ID/跨 session/不同 ELF 均拒绝。失败组缺字段或某详情查询失败会保留原始字节和错误并继续其他有限查询，未知记录数最多尝试 32 个索引，总期限仍为 90 秒。只读取证的 exit 0 表示取证完整，`status=DIAGNOSTIC_INCOMPLETE` 和原始失败终态继续保留，不表示硬件验收通过。新标签必须单独存档，不覆盖原失败实录。

The repository capture tool adds `--result-id <id> --expected-session <16hex>` together with the required full `--expected-elf <64hex>`. This path only queries: it sends no ADSCHK, mode setter, restart or flash operation, and rejects an obsolete ID, another session or a different ELF. Missing failed-group fields or an individual query failure retain raw bytes and errors while other bounded queries continue. An unknown record count permits at most 32 indices within the same 90-second overall deadline. Exit zero in read-only mode means evidence collection completed; `status=DIAGNOSTIC_INCOMPLETE` and the original failed terminal remain, without hardware acceptance. Use a new archive label and preserve the original failure capture.

[本轮主机回归与源码散列 / Host regression and source hashes](../audit/artifacts/cdc_recovery_acceptance_20261116/tail-io-control-host-01-manifest.json)：40 项 Python 合成回归和真实 C 控制代码 21 组加 10 个准入子例均 exit 0；PROTO 实测极值 436/512 B，IO 极值案例 324/512 B。这是主机协议/控制验证，不是新增 TAIL HIL 通过。 / Forty synthetic Python regressions and 21 production-C control groups plus ten admission subcases exited zero. The tested PROTO maximum was 436/512 bytes and the IO extreme case was 324/512 bytes. These are host protocol/control checks, not passing TAIL HIL evidence.

### ADC final-bit 候选的来源契约 / Read provenance for the ADC final-bit candidate

本轮新增 `adc_finalbit_v1` 公告只读来源查询与 CRC 拒收证据格式，不表示某个板型已通过运行期 CRC 或科学精度验收。普通 C/V/R/M/MR/K 帧、ASCII CRC、配置身份域、Xhh 和有效性掩码语义不变；来源扩展不允许猜测 CRC 末位、修补坏样本或提高科学有效性阈值。

The new `adc_finalbit_v1` capability announces read-only provenance queries and CRC-rejection evidence formats. It does not establish runtime CRC or scientific-accuracy acceptance for a board. C/V/R/M/MR/K frames, ASCII CRC, configuration identity domains, Xhh and validity masks keep their meanings. The extension permits neither guessing the final CRC bit, repairing rejected samples nor raising scientific-validity thresholds.

`SPISOURCE?` 返回独立的 `SPISOURCE,scope=boot_probe`：`phase/sourceKnown/policy/source/interfaceKnown/interface/hz/actualHz/qualified/restored/joined/timedOut/err/cleanup`，并明确 `runtime=unobserved,physicalVerified=0`。它只复制启动 worker 已在锁内发布的不可变结果，不在控制任务中读取驱动活字段。尚未完成或来源未知时输出 sourceKnown=0、source/policy=unknown；未确认 INTERFACE 则 interfaceKnown=0、interface=unknown。policy 为 spi_bytes 或 gpio_tail_required；source 为 spi_bytes、spi56、spi55_gpio1 或 unknown。后者区分常规字节式 SPI、标准 56 位 ADC1 读取以及 SPI55/GPIO1 分开观测。SPI? 原有 `scope=all_candidates` 和累计计数保持原义。SPISOURCE 不是当前运行期统计；Host 用前后 CONFIG 的 session 一致性确认查询属于同一 boot，不跨 boot 拼接结果。

`SPISOURCE?` returns a separate `SPISOURCE,scope=boot_probe` record containing `phase/sourceKnown/policy/source/interfaceKnown/interface/hz/actualHz/qualified/restored/joined/timedOut/err/cleanup`, with explicit `runtime=unobserved,physicalVerified=0`. It copies only the boot worker's immutable, mutex-published result; the control task never reads live driver fields. An unfinished or unknown source reports sourceKnown=0 and source/policy=unknown; an unconfirmed INTERFACE reports interfaceKnown=0 and interface=unknown. Policies are spi_bytes or gpio_tail_required. Sources are spi_bytes, spi56, spi55_gpio1 or unknown, distinguishing generic byte transfers, standard 56-bit ADC1 reads and separate SPI55/GPIO1 observations. Existing SPI? `scope=all_candidates` counters retain their meanings. SPISOURCE is not runtime telemetry; bracket it with matching CONFIG sessions to prevent joining results across boots.

`ADSSPIBYTES` 的标准路径保留原 bytes 字段，增加实际 source 和 wireKnown=1。split 路径必须 `source=spi55_gpio1,wireKnown=0,bytes=unknown`，另给 `assembledBytes/spiPrefix/gpioKnown/gpioBit/tailStage`。assembledBytes 是 STATUS、四个数据字节和收到的 CRC 共六字节；spiPrefix 是七字节的部分 SPI 存储，其中第 0 字节属于命令阶段 DOUT，不属于 ADC payload；最后一字节 bit0 是未由 SPI 收到的零占位，必须由真实 gpioBit 合成。只有 gpioKnown=1 才把该位当观测值；未知时 assembledBytes、spiPrefix、gpioBit 输出 unknown，不能把初始化 00 当实录。RAILCRC 沿用相同来源字段，并保留自己的 seq/txn 关联。完整 split CRC 拒收证据要求 tailStage=complete；其他阶段只是未完成的诊断，不能据此统计完整 ADC CRC 帧。

Standard `ADSSPIBYTES` retains its bytes field and adds the actual source and wireKnown=1. Split records must report `source=spi55_gpio1,wireKnown=0,bytes=unknown`, with separate `assembledBytes/spiPrefix/gpioKnown/gpioBit/tailStage`. assembledBytes contains six bytes: STATUS, four data bytes and the received CRC. spiPrefix contains seven bytes of partial SPI storage: byte zero belongs to command-phase DOUT and is not ADC payload; the final byte's bit zero is an unreceived zero placeholder that only the actual gpioBit supplies. Treat that bit as observed only when gpioKnown=1. Unknown observations produce unknown assembledBytes/spiPrefix/gpioBit, never initialized zero bytes presented as a capture. RAILCRC uses the same provenance fields and retains its seq/txn association. A complete split CRC-rejection record requires tailStage=complete; another stage is incomplete diagnostic evidence, not a complete ADC CRC frame.

CRC8 仍只保护四个 ADC 数据字节，STATUS 不在该 CRC 内。Host 应先验证来源与组装一致性，再独立复算 CRC；不得把 assembledBytes 当连续 SPI wire。错误文本仍有界且可限频，累计 CRC/retry 计数与 Xhh 不随文本限频而丢弃。新 PROTO 实际格式极值 452/512 B，SPISOURCE 281/512 B，ADSSPIBYTES 合成边界测试最大 294/384 B；这些是实际生产格式器的主机回归，不是新候选 HIL 结果。

CRC8 still covers only the four ADC data bytes, excluding STATUS. The host verifies provenance and assembly consistency before independently recomputing CRC; assembledBytes must never be labelled a continuous SPI wire capture. Error text remains bounded and may be rate limited, while cumulative CRC/retry counters and Xhh retain their semantics. Production formatter host tests measured 452/512 bytes for PROTO, 281/512 for SPISOURCE and at most 294/384 for the ADSSPIBYTES synthetic boundary cases. These are formatter regressions, not new-candidate HIL results.

[本轮实际命令、退出码与源码散列 / Executed commands, exit codes and source hashes](../audit/artifacts/cdc_recovery_acceptance_20261116/finalbit-protocol-host-01-manifest.json)；[实际 C 格式器产生的合成来源夹具 / Synthetic provenance fixture emitted by the production C formatter](../audit/artifacts/cdc_recovery_acceptance_20261116/finalbit-crc-format.synthetic.txt)。标准和 split 两行都经独立多项式 CRC 复算为 xor=01；该夹具不是硬件实录。 / Both standard and split records independently recompute to xor=01. The fixture is not a hardware capture.

修复实现提交 / Implementation commit: `b13321a2c166715fa15f98c1677626786bec65b1`. 本板首轮运行期120秒CRC0/79417，不能推导其他板型或无限运行可靠性；完整证据与剩余BLE阻塞见[阶段记录](../audit/cdc_recovery_acceptance_20261116.md)。后续诊断标准SPI组会增加全局CRC累计值；Host必须按同boot、事务事件及采集窗口取增量。 / The first120-second runtime screen records0/79417 CRC rejections, without proving other boards or indefinite reliability. Later standard-SPI diagnostic groups increment global CRC counters; hosts must scope deltas by boot, transaction events and capture window.

### BLE local CONF observation extension

`0c94fe3` 在已有`ble_service_obs_v1`的BLESVC v1中追加`confOk/confError/confMatched/confUnmatched`及`confMatch=waiter_predicate,confEpoch=unknown,pcReceipt=unknown`。四计数为boot累计uint32，重连不清零，按模2^32差分；SDK status分类与waiter分类相互独立。旧fe/stale字段和ACK语义不变。Host通过字段存在性兼容读取，缺字段为unknown。CONF可能早于SDK wrapper完成，不能要求瞬时计数相等。

`0c94fe3` appends these fields to existing BLESVC v1 under `ble_service_obs_v1`. Counters are boot totals, survive reconnect and wrap modulo2^32; SDK status and waiter matching classify the same events independently. Existing fe/stale and acknowledgement meanings remain. Discover optional fields by presence; absence is unknown. CONF can precede SDK wrapper completion, so instantaneous counters need not equal. No counter proves notification delivery to the peer or host application.

### CPU owner 只读运行计数契约 / Read-only owner runtime contract

以下是提交 `b58336eeaa136d0aca23e4c62ac11eb292d5a904` 的原0–6契约；当前新增role7须同时分页协商wifi_owner_v1，见[增量](#wifi-owner-contract-v1)，历史数据不重释。`PROTO?.caps` 中的 `cpu_owner_v1` 表示支持 `CPUOWNER?0` 至 `CPUOWNER?6`，不表示 profiling 必然开启。命令是精确的问号加单个角色数字，无 `=`；其他后缀返回有界 `ERR,cmd=CPUOWNER,reason=role_0_to_6`。early 与正常控制路径都可查询，不占配置请求 lane、不发送 ACK/Applied、不采硬件、不触发自采或重启。C/V/R/M/MR/K、原 gen/rid、CRC 与 MODE/ROWS/ROWMODES 终态语义均不变。

The following is the original role0–6 contract at commit `b58336eeaa136d0aca23e4c62ac11eb292d5a904`. Current role7 additionally requires paginated wifi_owner_v1, as defined [above](#wifi-owner-contract-v1); historical data retains its meaning. `cpu_owner_v1` in `PROTO?.caps` announces `CPUOWNER?0` through `CPUOWNER?6`, not necessarily enabled profiling. The exact grammar is a question mark followed by one role digit, without `=`; other suffixes receive the bounded `ERR,cmd=CPUOWNER,reason=role_0_to_6`. Queries work through early and normal control dispatch, reserve no configuration lane, emit no ACK/Applied, access no measurement hardware and trigger neither self-sampling nor restart. C/V/R/M/MR/K, legacy gen/rid, CRC and MODE/ROWS/ROWMODES terminal semantics are unchanged.

| role | 应用 owner / Application owner |
|---|---|
| 0 | BLE_TX |
| 1 | TRANSPORT |
| 2 | ASYNC_LOG |
| 3 | SERIAL_TX |
| 4 | BLE_CTRL |
| 5 | SERIAL_CTRL |
| 6 | WIFI_CTRL |

回复 tag 为 `CPUOWNER`、`v=1`；字段保持下列含义，数值未知使用字面量 `unknown`，不能补 0。 / Replies use tag `CPUOWNER`, `v=1`; fields have these meanings. Unknown numeric values use literal `unknown`, never a substituted zero.

| 字段 / Field | 编码与语义 / Encoding and meaning |
|---|---|
| `session` | 当前 boot 的 16 个十六进制字符；与 CONFIG 相等关联，不比较大小。 / Sixteen hex digits for the current boot, matched to CONFIG by equality, not magnitude. |
| `role` | 十进制 0–6；必须等于所查询角色。 / Decimal 0–6, matching the requested role. |
| `enabled` | 0/1，当前构建是否启用该诊断。 / Whether this build enables the diagnostic. |
| `known` | 0/1，是否曾完整发布本角色样本；不表示新鲜或可用于差分。 / Whether a complete role sample has ever been published, not whether it is fresh or usable. |
| `available` | 0/1，已知且没有 writer/counter/clock 错误；**可仍 stale=1**。 / Known with no writer/counter/clock fault; **may still be stale**. |
| `conflict` | 0/1，不同 taskNumber 或同时 sampler 的 writer 冲突，保持至 boot。 / A different observed task number or overlapping sampler; sticky until boot. |
| `stale` | 0/1，未知 age 或 `ageUs >= 3000000` 为 1。 / One for unknown age or age at least 3,000,000 µs. |
| `reason` | 枚举值 `disabled`, `not_observed`, `writer_conflict`, `counter_regressed`, `clock_regressed`, `ok`。 / These are the defined reason values. |
| `taskNumber` | uint32 十进制或 unknown；绑定角色首次观察的 task 身份，不能跨 session 比较。 / Decimal uint32 or unknown; binds the first observed writer and is not comparable across sessions. |
| `runUs` | uint64 十进制，available=1 才输出；否则 unknown。切出累计，非即时 CPU。 / Decimal uint64 only when available; otherwise unknown. Cumulative switch-out runtime, not instantaneous CPU. |
| `t0Us`, `t1Us` | uint64 单调微秒，自采内核调用前后时间；known=1 时保留原样。 / Monotonic U64 µs bracketing the owner’s kernel call, retained when known. |
| `observedAtUs` | 等于该样本 t1Us；不是查询收到时间。 / Equals that sample’s t1Us, not query receipt time. |
| `ageUs` | 查询时刻减 observedAtUs；时钟倒退或样本未知时 unknown。 / Query time minus observedAtUs; unknown for a clock regression or unknown sample. |
| `sampleMinUs`, `staleAfterUs` | 当前分别为 1000000、3000000；主机读取实际字段，不假定查询就产生新样本。 / Currently 1,000,000 and 3,000,000; read the advertised values and never assume a query creates a sample. |
| `scope`, `isr`, `sdk`, `blocked` | 固定 `switch_out`, `may_include`, `not_isolated`, `unknown`。 / Explicit accounting limitations, not estimates. |

`known=0` 时 task/runtime/所有时间为 unknown，stale=1。`enabled=0` 时 available=0、known=0、reason=disabled；capability 仍可公告。已知但过期的样本可 `available=1,stale=1,reason=ok` 并保留历史 runUs，不能据此计算当前 CPU。writer/counter 错误可保留已知 task/时间，runUs 必为 unknown。查询遇到时间倒退只使本次副本 unavailable/age unknown，不修改 owner 状态；后续真实 checkpoint 自己发现的错误则保持不可用。完整字段之间不允许用缺失字段的默认值推断通过。

With known=0, task/runtime/all times are unknown and stale=1. With enabled=0, available=0, known=0 and reason=disabled; the capability can still be advertised. A known old sample may report available=1, stale=1, reason=ok and retain historical runtime, without supporting current-CPU inference. Writer/counter faults may retain known task/time fields, but runUs is unknown. A query-time clock regression makes only that copied result unavailable with unknown age, without mutating the owner; a fault independently observed at a real checkpoint remains unavailable. Missing fields must not be replaced with defaults to pass validation.

配置 `SENSORARRAY_CPU_OWNER_DIAGNOSTICS` 默认 n，依赖双核 IDF FreeRTOS、非 SMP/UNICORE、runtime stats、U64 与 ESP_TIMER，不强选系统 choice。ESP-IDF 5.5.5 已核 port 路径直接把 64 位 esp_timer 值转换到 U64 runtime counter，无中间 32 位回绕；核验依据在[源代码与回归记录](../audit/artifacts/cdc_recovery_acceptance_20261116/cpu-owner-host-01-freeze-review.json)。当前诊断构建额外启用了 trace；正常 defaults 保持关闭。未达成该依赖的镜像不能通过仅添加 capability 宣称数据有效。

`SENSORARRAY_CPU_OWNER_DIAGNOSTICS` defaults to n and depends on dual-core IDF FreeRTOS, non-SMP/non-UNICORE, runtime statistics, U64 and ESP_TIMER, without forcing system choices. The inspected ESP-IDF 5.5.5 port directly casts the 64-bit esp_timer value to the U64 runtime counter, with no intermediate 32-bit wrap; the [source-linked review](../audit/artifacts/cdc_recovery_acceptance_20261116/cpu-owner-host-01-freeze-review.json) records that evidence. This diagnostic build also enables trace; normal defaults remain disabled. Advertising a capability alone cannot make data from an unsupported configuration valid.

每角色只在自身 CPU0 自然 checkpoint 自采、完成样本间隔至少 1 秒。内核读取 `vTaskGetInfo(NULL, …, pdFALSE, eReady)` 不持应用 mux、不开栈水位扫描、不发布 eReady 占位状态；内核双核 mux 保证 U64 复制，之后短锁发布值。查询不读取 live task handle，缓存不持有名称、栈或 handle 指针。新接口不增加任务、delay 或改原预算/优先级；配置 OFF 的 checkpoint 不增加 timer/kernel/mux 调用。profiling 与 ON 自采有实际耗时/内存开销，可影响原有时间条件，不能宣传完全无扰动。当前目标新增本地静态存储 456 B，不包括全局 trace/TCB 变化。

Each role self-samples only at its natural CPU0 checkpoint, with at least one second between completed samples. The kernel call `vTaskGetInfo(NULL, …, pdFALSE, eReady)` runs without the application mux, performs no stack-watermark scan and does not publish the placeholder eReady state. The dual-core kernel mux protects the U64 copy, followed by short-lock publication. Queries never read a live task handle; the cache retains no name, stack or handle pointer. The interface adds no tasks/delays or changes to existing budgets/priorities. With diagnostics OFF, checkpoints add no timer/kernel/mux calls. Profiling and ON-path observation still cost time/memory and can affect existing elapsed-time conditions; disturbance-free behavior is not claimed. Current target local static storage is 456 bytes, excluding global trace/TCB changes.

Host 差分须逐角色核同 session、同 taskNumber、known/available、无 conflict、未 stale、runUs 不倒退且采样 bracket 有序；时间和 runtime 不跨 boot 拼接，U64 倒退当错误，不擅自按 32 位补回绕。每角色使用自己的 `observedAtUs` 时间跨度，保留 t0/t1 不确定边界；多个查询不是原子跨任务 CPU 快照，比例不可直接求和。runUs 不含未切出的运行片段，可能含 ISR、SDK 内外所有本任务工作；没有 SDK/controller/IDLE task 样本，不可算全核 residual 或把 SDK wall−owner run 叫 blocked。BLESVC 的 cpu/blocked 继续 unknown，网络确认/PC 实收语义不变。

Hosts validate each delta independently: same session/task number, known/available, no conflict, fresh samples, nonregressing runtime and ordered sampling brackets. Times/runtime never span boots, and a U64 regression is an error, not an assumed 32-bit wrap. Each role uses its own observedAtUs span and retains t0/t1 uncertainty; separate queries are not an atomic cross-task CPU snapshot, so ratios must not be summed. Runtime excludes an open running slice and may include ISR and all task work inside/outside the SDK. Without SDK/controller/idle samples, neither full-core residuals nor SDK wall−owner run as blocked time are defined. BLESVC cpu/blocked remain unknown, and no network/PC receipt meaning changes.

真实 C 格式器回归：CPUOWNER 最大 372/512 B，PROTO 最大 465/512 B，短 buffer 显式失败。实际 65 入口回归通过，[screen07 离线分析](../audit/artifacts/cdc_recovery_acceptance_20261116/screen-07/screen-07-cpu-owner-deltas-01.json)仅五角色有新鲜可配对结果；BLE_CTRL 的历史零 runtime 和未观测 Wi-Fi 都为 unknown。screen07 的 ADC 0/76,128 和 USB/BLE 59.81/32.39 FPS 是本板、本窗口结果，不把能力公告或 CPU 观测当作 BLE/精度验收通过。上位机仓库改动仍交独立任务。

Production C formatter regressions measured at most 372/512 bytes for CPUOWNER and 465/512 for PROTO, with explicit short-buffer failure. All 65 test entries passed. The [screen07 offline analysis](../audit/artifacts/cdc_recovery_acceptance_20261116/screen-07/screen-07-cpu-owner-deltas-01.json) pairs only five fresh roles; BLE_CTRL's historical zero runtime and unobserved Wi-Fi remain unknown. Screen07's 0/76,128 ADC CRC result and 59.81/32.39 USB/BLE FPS apply only to this board/window. Capability presence and CPU observation are not BLE or precision acceptance. Host-repository implementation remains a separate task.


## Final-capture host authority and evidence gates / 最终采集身份与证据门禁

本轮新增 `sensorarray_capture_identity.py` 供最终采集入口复用。expected ELF/session/boot 必须来自单独记录的实际只读查询，不能从文件名或相近版本推断。`CONFIG → BUILD → CONFIG` 两端完整配置字段相等才允许设置；首个设置和 CAP8 清理分别有原始回复证据。看到 reset/不同 CONFIG boot 后立即撤销权限；跨 boot 不比较 sequence/generation 大小。明确只读 `WIFIQ?D/L/C`、`WIFIREQ?<u32>` 可在未授权设置时查询，`WIFI=AP` 等设置不因没有空格或含查询前缀而绕过门禁。这只是采集工具的镜像身份保护，不是设备物理 SAFE 声明。

The new shared capture identity helper requires expected ELF/session/boot from separate recorded read-only observations, never a filename or nearby version. Setters require equal complete CONFIG snapshots around BUILD; the first setter and CAP8 cleanup have separate original-reply evidence. Observed reset/changed CONFIG boot revokes authority; sequence/generation is never ordered across boots. Explicit read-only `WIFIQ?D/L/C` and `WIFIREQ?<u32>` remain usable without setter authority; `WIFI=AP` and malformed query-like setters cannot bypass it. This is a capture-tool image guard, not a physical SAFE assertion.

完整帧有效性仍以真实 encoder 字节、科学帧 CRC、使用 G 封装时的外层 CRC、不可变配置域和协议格式判断。最终 FULL 筛查另要求可信 host 首/末完整帧距窗口两侧各≤1秒、device timestamp跨度≥requested−2秒；合法但更慢或未知的情况进入 UNKNOWN人工审查。这个充分性规则与 v2 BLE端点缺口≤2.5%并列，不能更改该公式、扣减本地计数、把订阅连接时间当数据覆盖，或把1秒写成BLE送达期限。USB host时间为collector相对秒，BLE为同主机绝对monotonic秒，必须各对自己的采集边界，不直接比较两种坐标。

Complete-frame integrity uses original encoder bytes, the scientific-frame CRC, the outer CRC when a G envelope is used, immutable identity domains and the wire format. Final FULL screening also requires trustworthy first/last host completions within one second of each window edge and device timestamp span ≥requested−2 seconds; slower or unknown cases stop for manual review. This sufficiency rule accompanies the unchanged v2 BLE endpoint gap budget of2.5%. It cannot alter the formula, subtract local counters, substitute subscription coverage for data coverage, or turn one second into a delivery deadline. USB host times are collector-relative seconds and BLE times are absolute host-monotonic seconds; compare each with its own matching window coordinates.

[冻结与122项已执行回归](../audit/artifacts/cdc_recovery_acceptance_20261116/final-acceptance-runner-freeze-01.json) / [Frozen sources and122 executed regressions](../audit/artifacts/cdc_recovery_acceptance_20261116/final-acceptance-runner-freeze-01.json)。[实录离线回放](../audit/artifacts/cdc_recovery_acceptance_20261116/final-acceptance-gate-replay-02.json)仅保持原screen20 FAIL、screen21正常数值预算通过；不为旧记录追认新preflight，也不代表完成最终HIL。

独立[最终runner审查02](../audit/artifacts/cdc_recovery_acceptance_20261116/final-acceptance-runner-independent-review-02.json)再次实际运行全部122项，15个冻结源码hash前后相同；原两帧反例在新gate返回UNKNOWN。[独立实录回放03](../audit/artifacts/cdc_recovery_acceptance_20261116/final-acceptance-gate-independent-replay-03.json)重新解析screen20/21，仍为FAIL/仅数值PASS，原始输入不改。这个PASS仅授权继续下一次HIL尝试，不是整体验收。

Independent [runner review02](../audit/artifacts/cdc_recovery_acceptance_20261116/final-acceptance-runner-independent-review-02.json) reruns all122 tests with all15 frozen source hashes unchanged; the original two-frame counterexample now returns UNKNOWN. [Independent recording replay03](../audit/artifacts/cdc_recovery_acceptance_20261116/final-acceptance-gate-independent-replay-03.json) reparses screen20/21 and retains FAIL/numeric-only PASS without changing originals. This review permits the next HIL attempt, not overall acceptance.

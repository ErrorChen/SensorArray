> **2026-11-25 · 原始32配置矩阵 / Original 32-configuration matrix — 9 PASS, 1 FAIL, 22 NOT_STARTED.** 在同一镜像、session 和 boot 上，CAP 1–8行及 VOLT 1行通过。VOLT 2行的 USB 缺7个完整帧，BLE 收齐5,050帧，矩阵随后停止。CAP 2行首次预检中止，第二次正式窗口通过。 / CAP1–8 and VOLT1 passed. The run stopped when USB missed seven complete VOLT2 frames. CAP2 passed after an observer preflight error. [矩阵及夹具 / Matrix and fixture](#matrix-bss-01-02) · [全部32项 / All 32 cases](docs/audit/artifacts/cdc_recovery_acceptance_20261116/bss-matrix-combined-01.md)

> 同一 boot 的600秒 VOLT1 正常 FULL 筛查通过，50次切换均有 USB 验证与匹配 BLE 帧，SHORT 收到117个完整快照。Wi-Fi UDP 上位机实收仍未测，夹具精度与整体验收待验证。 / The 600-second VOLT1 screen passed, all 50 transitions had USB verification and matching BLE frames, and SHORT delivered 117 complete snapshots. Host Wi-Fi UDP receipt, fixture accuracy and overall acceptance remain open. [三个独立窗口 / Three separate recordings](#bss-current-recordings)

> **历史记录 / Historical records.** 下方旧 ELF 及旧阶段的 FAIL 和 NOT_RUN 按各自镜像和启动周期保留。当前矩阵只合并 `bss-matrix-01` 与 `bss-matrix-02` 的同身份记录。 / Earlier FAIL and NOT_RUN entries describe the image and boot named in each record. The matrix above combines only the two named runs on their verified common identity.

> **2026-11-25 AP08启动实录（boot1） / AP08 startup observation (boot1).** [wifi-bss-01构建](docs/audit/artifacts/cdc_recovery_acceptance_20261116/wifi-bss-01-validated-identity.json)与[READY核验](docs/audit/artifacts/cdc_recovery_acceptance_20261116/wifi-bss-ready-01.summary.json)绑定完整ELF`179f9183e2c31b35c906c9b811bb2ca1cf4d16dbca43ee3a1c6c4371c0f56a2d`、session`00B44EE26D0A4A0F`/boot1/CAP8/READY1；80入口/构建通过、203生产输入一致。SDK外部BSS及8,704 B任务校准表合计迁移29,116 B，保留既有采集速率与缓冲容量。[AP08](docs/audit/artifacts/cdc_recovery_acceptance_20261116/wifi-control-08/wifi-control-08.summary.json)的AP/AP/OFF/AP/OFF五请求全applied，首次AP53.432ms；最终同boot outputGate0/backend ready/quarantined0。internal free/largest前93,356/47,092 B、后29,008/20,468 B、min27,952 B；MEMFAIL0→0、incomplete0，无观察fault/reset，BLE observer正常关闭。**OFF只关输出gate，不证明radio断电；AP08当时未创建正式FPS或最终验收窗口。后续boot2的600秒、50切换及SHORT已闭合，见当前实测小节；UDP实收仍未测。** AP04/AP05的NO_MEM、AP06分配失败与AP07 panic历史保留。 / The exact image boots CAP8/READY1 and passes all 80 entries/build. Supported external BSS placement moves 29,116 bytes, including the task-owned calibration table, without reducing acquisition rate or buffer capacity. All five AP/AP/OFF/AP/OFF requests apply; the first AP completes in 53.432 ms while USB/BLE control coexists. The backend remains ready with the output gate closed, no observed allocation failure, fault or reset, and a clean BLE observer shutdown. OFF does not prove radio power-down. AP08 itself has no formal throughput or final-suite window. Later boot2 long-run, transition and SHORT recordings are closed below; UDP receipt remains untested and earlier failures remain preserved.

> **2026-11-25 保留AP07启动panic / Preserved AP07 startup panic.** [wifi-dma-headroom-01](docs/audit/artifacts/cdc_recovery_acceptance_20261116/wifi-dma-headroom-01-validated-identity.json)关闭SDK可选Wi-Fi IRAM speed两项后，map回收18,432 B共用SRAM；完整ELF`2fed45c22153b580aef27584107809d15cdabdd8afe6d6bd90cf6d5a282c07a7`、80入口/构建通过、203生产输入一致。[AP07](docs/audit/artifacts/cdc_recovery_acceptance_20261116/wifi-control-07/wifi-control-07.summary.json)前session`DE8FA9C044C728B7`/boot1，internal free/largest64,324/31,732 B；随后USB EOF/BLE断连，AP终态和OFF恢复未知。[USJ保留panic](docs/audit/artifacts/cdc_recovery_acceptance_20261116/dma-panic-01.summary.json)的record`C3FA0326`匹配该origin/同ELF；新session`DC4763F0436668C0`/boot2，PC`420C9C8D`位于`ieee80211_hostap_attach`，cause0x1C/vaddr0x2C。**AP07未取得post-MEMFAIL，不推定具体allocator；boot2的count0不能回填boot1。** 该run未证明AP/UDP可用；AP04/05/06旧证据保留，AP08结果单列于上方。 / The optional-code placement change releases 18,432 bytes in the map and passes all 80 entries/build. AP07 starts with more internal memory but loses USB and BLE. A retained panic matches its origin and exact ELF, locating ieee80211_hostap_attach in AP startup. No post-attempt MEMFAIL exists; the new boot's zero counter does not identify an allocator or fill the missing prior-boot observation. This run does not establish AP/UDP operation; the later AP08 result is recorded separately above.

> **2026-11-25 保留AP06分配失败观察 / Preserved AP06 allocation-failure observation.** [heap-failure-02构建](docs/audit/artifacts/cdc_recovery_acceptance_20261116/heap-failure-02-validated-identity.json)与[启动核验](docs/audit/artifacts/cdc_recovery_acceptance_20261116/heap-failure-ready-02.summary.json)绑定完整ELF `90f10fdaa5a0b859995caba70b1f1c65d3c7f9e19e85b39c81366e699c6e2736`、session `EE880CC8223D06C2`/boot1/CAP8/READY1，80入口/构建通过、203生产输入一致。[BLE共存AP06](docs/audit/artifacts/cdc_recovery_acceptance_20261116/wifi-control-06/wifi-control-06.summary.json)的`MEMFAIL?`由count0增至1、incomplete0：**1×1,604 B，caps0x80C（INTERNAL|DMA|8BIT）**；同ELF解码fn `0x3C1506DC`为`heap_caps_malloc`。[静态二进制路径](docs/audit/artifacts/cdc_recovery_acceptance_20261116/wifi-memory-source-map-01.md)强匹配Wi-Fi static RX初始化，**不是运行时caller/backtrace证明**。AP仍在34.050ms返回NO_MEM并隔离，cleanup0x103未知保留；internal free/largest前44,856/31,732 B、后28,480/9,716 B、min7,996 B。OFF gate同boot可查，无观察fault/reset；AP04/AP05旧值保留。**本轮未证明AP/UDP可用，无正式FPS窗口，600秒/32配置/50切换/SHORT仍NOT_RUN。** / The exact diagnostic image is boot-verified and all 80 entries/build pass with 203 matching inputs. During BLE coexistence, AP06 records one failed 1,604-byte internal DMA allocation. The decoded allocator API and static RX initialization path strongly match; the hook does not prove the runtime caller. AP still returns NO_MEM in 34.050 ms, with uncertain cleanup retained. Same-boot OFF queries remain available, with no observed fault/reset. AP04/AP05 history stays intact. AP/UDP availability, formal throughput and final acceptance are not established.

> **2026-11-21 保留AP05观察：Bluedroid优先PSRAM后AP仍NO_MEM / Preserved AP05 observation after Bluedroid PSRAM preference.** [bt-host-memory-01闭合证据](docs/audit/artifacts/cdc_recovery_acceptance_20261116/bt-host-memory-control-evidence-01.json)绑定完整ELF `2f74130825f1b4c173cee27bf569bc4385bbdd26a7049adee3efe6e8b77c39e9`、session `55B0EC5D078FCD53`/boot1/CAP8/READY1；80入口/构建通过、203生产输入一致。官方`CONFIG_BT_ALLOCATION_FROM_SPIRAM_FIRST`只改变Bluedroid动态host分配优先级并保留内部fallback，不迁controller DMA/任务栈/队列。BLE前internal free47,144 B，较前一镜像增加5,528 B；BLE共存AP05仍以NO_MEM0x101终结，accepted→terminal34.482ms、cleanup0x103未知保留。AP前free/largest45,716/31,732 B，后29,076/10,740 B、min8,592 B；OFF及旧失败缓存可查，无观察fault/reset。原BLE journal实际回放确认L通道收到id1 AP失败与id2 OFF两条WTERM，不以本地queued代替实收。旧采集器仅有recorded SHA、源码未独立归档，原始回复/事件另行核验。**无正式FPS窗口，最终600秒/32配置/50切换/SHORT仍NOT_RUN。** / The exact image boots CAP8/READY1 and passes all 80 entries/build with 203 matching inputs. Official Bluedroid host allocation now prefers PSRAM with internal fallback; controller DMA, task stacks and queues retain their placement. Pre-BLE internal free memory is 5,528 bytes higher, but AP05 still returns NO_MEM in 34.482 ms with uncertain cleanup preserved. Both exact WTERM results reach this PC on lifecycle L. The historical collector has a recorded SHA without a separate source archive; original replies/events are independently checked. No formal FPS or final acceptance is claimed.

> **2026-11-21 保留首次AP失败观察（当时原因未知） / Preserved first AP failure observation, before diagnosis.** 同`wifi-owner-02` ELF `9e1ee9f9812dcd21a1295e315f08c10bdf98ae55d309e67b1f84b97b3c825573`的[AP实录](docs/audit/artifacts/cdc_recovery_acceptance_20261116/wifi-control-01/wifi-control-01.summary.json)中，id1先accepted/pending，随后查询running；AP写入后约10ms内出现USB EOF，终态未收到，OFF恢复FAILED_OR_UNKNOWN（主机事件时间，不是精确panic时刻）。[后续只读证据](docs/audit/artifacts/cdc_recovery_acceptance_20261116/wifi-failure-usj-01.summary.json)确认同ELF `reset=panic`、boot2 / session `370C59E1DF8015F5`、CAP8/READY1，控制在`serial_jtag`恢复通路可查询。**根因仍待诊断；先修复，600秒/32配置/50切换/SHORT及手动重启验收均NOT_RUN。** 新boot的idle/零guard计数不能证明旧SDK调用已取消，旧session的最终测试参数已失效。 / Request1 was accepted and observed running, followed by USB EOF roughly10ms after the AP write, with no terminal and unverified OFF restoration. Subsequent read-only queries confirm a same-image panic reboot and accessible Serial/JTAG recovery in the new boot. The root cause is not established; repair precedes final testing. New-boot idle counters do not prove cancellation of the previous call, and old-session capture arguments are no longer valid.

> **2026-11-21 AP尝试前的构建/启动核验记录 / Preserved build and boot verification before the AP attempt:** `wifi-owner-02` / ELF **`9e1ee9f9812dcd21a1295e315f08c10bdf98ae55d309e67b1f84b97b3c825573`**、bin `55b25e40cc20de1b619f000080d5fe984752e1e693b2327e6e2dda4de87c2c9e`，[78入口与构建](docs/audit/artifacts/cdc_recovery_acceptance_20261116/wifi-owner-02-validated-identity.json)全部exit0、201生产输入一致，app1,628,976 B；[完整镜像归档](docs/audit/artifacts/cdc_recovery_acceptance_20261116/wifi-owner-02-image-archive.json)另存。[烧录后启动证据](docs/audit/artifacts/cdc_recovery_acceptance_20261116/wifi-owner-02-boot-verification.json)：session `0ACE6CD97F8649DC` / boot1，CAP8/READY1，SPI启动资格2MHz、2000样本CRC/status/reg0；这是启动扫描，不是持续稳定性。MEM当前internal free/largest/min为27,408/11,252/21,044 B，scan创建前largest25,588、创建后8,180 B，查询sequential。Wi-Fi仍默认OFF、resources0、backend inactive，AP与显式重启/ROM枚举尚未实测；新镜像600秒/32配置/50切换/SHORT尚未运行。重启后的session必须重新实录，不能复用本次身份。 / All78 host entries and the build pass with201 matching production inputs and a1,628,976-byte app. Archived images and post-flash observations verify CAP8/READY1 in the stated session. The2MHz/2000-sample zero-error startup sweep is not a sustained run. Internal-memory observations are sequential and retain the scan-allocation cost. Wi-Fi remains uninitialized/OFF; AP, explicit restart/ROM enumeration and the new image’s long-run/matrix/transition/SHORT tests remain unrun. A later restart requires freshly recorded identity.

> **2026-11-21 保留screen21吞吐筛查，仅通过正常BLE数值预算 / Preserved screen21 throughput screen, meeting only the normal BLE numeric budget:** `ble-snapshot-06` / ELF **`c47a3710a770506abd030d03790b58c137150c9c1cba86a596f07962114a3c4e`** 已[启动核验](docs/audit/artifacts/cdc_recovery_acceptance_20261116/ble-snapshot-06-boot-verification.json)，session `630DC17010A097FF` / boot1；195生产输入一致，[73入口与构建](docs/audit/artifacts/cdc_recovery_acceptance_20261116/ble-snapshot-06-validated-identity.json)均exit0。[screen21](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-21/screen-21-verdict.json)完成 **120.000119494秒** VOLT1 USB＋BLE：Capture/USB/BLE **64.195756/64.233205/63.079023 FPS**，USB7,707连续，BLE7,569缺141，[同criteria v2](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-21/screen-21-criteria-v2-supplement.json) **141/7,710=1.828794% ≤2.5%**。判定仍 **OBSERVED_NOT_ACCEPTED / overall NOT_ESTABLISHED**；ADC独立窗0/81,815、USB/BLE invalid460/434保留、CAP8恢复核验。600秒、32配置USB＋BLE、50切换及本镜像SHORT/弱链路未跑；Wi-Fi隔离与手动重启补丁已完成源码/host验证，新目标仅已构建/启动，AP/重启/持续HIL仍待独立闭合，未宣称整体无P0/P1。 / Build06 boots successfully and all 73 host entries/build pass with 195 matching inputs. The closed **120.000119494-second** screen records **64.195756/64.233205/63.079023 FPS** capture/USB/BLE. USB has 7,707 internally consecutive frames; BLE has 7,569 with 141 missing observations: **1.828794% ≤2.5%**. Only that numeric budget passes; the verdict remains **OBSERVED_NOT_ACCEPTED / overall NOT_ESTABLISHED**. Separate ADC observations are 0/81,815, invalid points remain, and CAP8 recovery is verified. Long-run, complete configuration matrix, transition and same-image specialized acceptance remain unrun; Wi-Fi isolation/manual-restart source and host verification are complete, while the new target is only build/boot verified and its AP/restart/sustained-HIL gates remain open.

> **2026-11-21 保留build05筛查失败 / Preserved build05 screen failure:** `ble-snapshot-05` / ELF **`7f2d5ad9c73887f70d49c7f8eb45ca889c4c44fcb1fedd293299200d9b39b469`** 已[实际启动核验](docs/audit/artifacts/cdc_recovery_acceptance_20261116/ble-snapshot-05-boot-verification.json)，session `B73989CDD936FA54` / boot1。[73入口与构建](docs/audit/artifacts/cdc_recovery_acceptance_20261116/ble-snapshot-05-validated-identity.json)均通过、195生产输入一致；[screen20](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-20/screen-20-verdict.json)完成 **120.000081240秒** VOLT1 USB＋BLE，Capture/USB/BLE **63.796225/63.871027/62.188982 FPS**。USB7,664连续，BLE7,461缺202，[同criteria补充](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-20/screen-20-criteria-v2-supplement-02.json) **202/7,663=2.636043% > 2.5%：FAIL**。内部2,304 B发送工作区尚未证明性能收益；ADC独立窗0/80,344、USB/BLE无效点483/456保留，CAP8恢复核验。旧screen19/18/575c失败不覆盖；build04的SHORT/USB不读专项不替代新镜像整体或长期验收。 / Build05 is flashed and boot-verified, with all 73 host entries/build passing and 195 matching inputs. The closed **120.000081240-second** screen still **fails**: BLE receives 7,461 frames with 202 missing observations, **2.636043% > 2.5%**, while USB receives 7,664 consecutive frames. Capture/USB/BLE rates are **63.796225/63.871027/62.188982 FPS**. A 2,304-byte internal working copy has no demonstrated performance benefit. Separate ADC observations are 0/80,344 with invalid points retained and CAP8 recovery verified. Earlier failures remain; build04-specific SHORT/unread observations do not qualify this image's overall or long-run behavior.

> **2026-11-21 保留build04筛查失败 / Preserved build04 screen failure:** `ble-snapshot-04` / ELF **`a6b04a2a68ddcb5c8c39cc28020abbe5db430c1e140cbb3afb4ddb703fdb217a`** 已[启动核验](docs/audit/artifacts/cdc_recovery_acceptance_20261116/ble-snapshot-04-boot-verification.json)，session `B4828F8EF45001F1` / boot1。195生产输入一致，[73入口整轮与构建](docs/audit/artifacts/cdc_recovery_acceptance_20261116/ble-snapshot-04-validated-identity.json)均exit0；[screen19](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-19/screen-19-verdict.json)闭合 **120.000139309秒** VOLT1 USB＋BLE，Capture/USB/BLE **64.392618/64.399646/62.340491 FPS**。USB7,728帧内部连续；BLE7,481帧缺250，[同criteria v2](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-19/screen-19-criteria-v2-supplement.json) **250/(7481+250)=3.233734% > 2.5%：FAIL**。周期合并生效、保护queued/complete/staleNormal/deadlineMiss增量均0，但容量拒绝仍+228；不能从250中抵扣。独立ADC窗0/82,589，USB/BLE无效点502/471保留，CAP8恢复已查询核验。600秒/50切换/32配置整套/SHORT/弱链路仍未验收，不宣称整体无P0/P1。 / All 73 host entries and the build pass with 195 matching inputs, and startup is verified. The closed **120.000139309-second** screen nevertheless **fails** the normal BLE gap budget: **3.233734% > 2.5%**, at **64.392618/64.399646/62.340491 FPS** capture/USB/BLE. USB receives 7,728 internally consecutive frames; BLE receives 7,481 with 250 missing observations. Coalescing eliminates additional protected sends/overtaking in the observed query interval, while 228 local capacity rejections remain; they cannot be subtracted from PC gaps. A separate ADC window records 0/82,589 rejections, invalid points remain, and CAP8 recovery is verified. Longer and broader acceptance remains incomplete.

> **2026-11-21 保留旧镜像筛查失败 / Preserved prior-image screen failure:** `ble-snapshot-03` / ELF **`7ce5e9f8746aefe0f999b467bc08fc9470903a59141339829c6363a30db7e1c8`** 的[启动恢复](docs/audit/artifacts/cdc_recovery_acceptance_20261116/ble-snapshot-03-boot-verification.json)已实证；同session `0C1E0125F83FF11C` / boot1的[screen18](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-18/screen-18-verdict.json)完成120.000089528秒VOLT1 USB＋BLE。Capture/USB/BLE **63.042560/63.017922/61.120460 FPS**；USB7562帧内部连续，BLE7334帧缺228，[criteria v2补充](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-18/screen-18-criteria-v2-supplement.json) **228/(7334+228)=3.015075% > 2.5%：FAIL**。独立健康窗ADC **0/79,590**，USB/BLE无效点466/442保留，CAP8查询恢复。该旧镜像600秒、50切换、32配置整套、SHORT/弱链路验收仍NOT_RUN；不宣称整体无P0/P1。 / Startup is verified, but the completed 120-second screen fails the normal-link gap budget. Capture/USB/BLE rates are **63.042560/63.017922/61.120460 FPS**. USB receives 7,562 internally consecutive frames; BLE receives 7,334 with 228 missing observations: **3.015075% > 2.5%**. Separate ADC observations have 0/79,590 CRC rejections and retain 466/442 invalid USB/BLE points. CAP8 recovery is query-verified. Long-run, complete configuration-suite, transition, SHORT and weak-link acceptance remain unrun for this image; overall P0/P1 absence is not established.

> **2026-11-21 旧候选启动失败保留 / Preserved previous-candidate startup failure:** `ble-snapshot-02` / ELF **`575c5dc25217721cfb47f651556d0e28edd4b5fe666a022200b8737608ff2e40`** 已实际烧录；[只读设备证据](docs/audit/artifacts/cdc_recovery_acceptance_20261116/ble-snapshot-state-01.summary.json)报告 `READY=0,stage=scan_task_create_failed,err=0x101`（ESP_ERR_NO_MEM）。USB 控制仍可查询，SPI `phase=idle,attempted=0,samples=0`，因此 ADC CRC、采集性能和保护快照 HIL 均 **NOT_RUN**，初始化 `crc=0` 不是零错误率。`BOOT.heap=8295496` 不是 internal largest block。该失败已由上方新镜像独立复验启动恢复，原失败记录不改。 / The previously flashed candidate reports scan-task allocation failure while USB control remains responsive. SPI never starts; zero initialized counters are not a measured CRC rate. Aggregate 8.29 MB heap does not prove a contiguous internal stack allocation. The new image above independently verifies startup recovery; original failed evidence remains unchanged.

> **2026-11-21 历史完整采集窗口 / Historical completed capture window:** screen17 在 `8a1794c-dirty`、完整 ELF **`f1ec518dfba78453dcd7fe057bba29eb61a8f8898274a21e22125df26b933a04`** 上录制 **120.000057708 秒**；Capture/USB/BLE 为 **64.978526/65.035014/64.800778 FPS**，USB 7,805 帧内部连续、BLE 7,776 帧缺 29。按 criteria v2，**29/(7,776+29)=0.371557% ≤ 2.5%**，仅正常链路缺口数值预算通过；[补充判定](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-17/screen-17-criteria-v2-supplement.json)不代表整体 PASS，原 `OBSERVED_NOT_ACCEPTED` 保留。独立健康窗 ADC CRC **0/81,537**；USB/BLE 无效点 373/370 保留，CAP8 查询恢复。普通 LOG 策略已有查询证据；新弱链路适应/保护快照仍实施中，本 ELF 的 600 秒/50 次切换/32 配置未运行。 / Screen17 records **120.000057708 seconds** on `8a1794c-dirty`, full ELF **`f1ec518dfba78453dcd7fe057bba29eb61a8f8898274a21e22125df26b933a04`**, at **64.978526/65.035014/64.800778 FPS** capture/USB/BLE. USB has 7,805 internally consecutive frames; BLE has 7,776 with 29 missing observations. Under criteria v2, **29/(7,776+29)=0.371557% ≤ 2.5%** meets only the normal-link numeric gap budget. The [supplement](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-17/screen-17-criteria-v2-supplement.json) is not overall PASS; original `OBSERVED_NOT_ACCEPTED` remains. The separate health span has **0/81,537 ADC CRC rejections**; 373/370 USB/BLE invalid points remain, and CAP8 recovery is query-verified. Ordinary-LOG policy has query evidence; new weak-link adaptation/protected snapshots remain in progress. This ELF’s 600-second/50-transition/32-configuration runs are NOT_RUN.

> 历史连续稳定性 / Historical continuous stability: `a656d53` 的独立 600.000075751 秒 USB+BLE 结果完整保留；其 ADC 0/428,131 和 CAP8 恢复仅属于旧镜像，不能填入 `53005f0-dirty` 的 600 秒验收。screen11 增加 CRC32 ROM/planner 等价优化和显式 SCLK 低电平 hold；screen12 仅进一步请求 MTU517，均不放宽科学 CRC/invalid 门限。 / The independent 600.000075751-second USB+BLE result on `a656d53` remains preserved. Its 0/428,131 ADC result and CAP8 recovery belong to the previous image and do not fill the new image’s 600-second acceptance. Screen11 adds equivalent ROM-CRC32/planner optimization and an explicit SCLK-low hold; screen12 additionally requests MTU517. None relaxes scientific CRC/invalid criteria.

Still in test

# SensorArray — ESP32-S3 固件 / ESP32-S3 Firmware

SensorArray 面向 8×8 测量矩阵，支持 **1–8 个活动 S 行、每行 8 个 D 通道**。每行可选择电容 `CAP/C`、电压 `VOLT/V` 或电阻 `RES/R`；`ROWMODES` 可将不同模式组合在同一帧。模式切换的最小单位是一行，不是固定四点。

SensorArray is firmware for an 8×8 measurement matrix with **1–8 active S rows and 8 D channels per row**. Each row can measure capacitance (`CAP/C`), voltage (`VOLT/V`), or resistance (`RES/R`); `ROWMODES` combines these modes in one frame. A row is the smallest mode-switching unit; acquisition is not restricted to a fixed four-point scan.

仓库 / Repository: [ErrorChen/SensorArray](https://github.com/ErrorChen/SensorArray). 上位机兼容改动属于独立的 `ErrorChen/SensorArray_Software` 任务。 / Host compatibility changes belong to the separate `ErrorChen/SensorArray_Software` task.

<a id="bss-current-recordings"></a>

## 2026-11-25 当前镜像实测 / Measurements on the current image

[源码身份](docs/audit/artifacts/cdc_recovery_acceptance_20261116/wifi-bss-01-source-identity.json)：已推送`f8d8424492c855a402838737ee8718e32f5135d6`，运行ELF为`179f9183e2c31b35c906c9b811bb2ca1cf4d16dbca43ee3a1c6c4371c0f56a2d`、BUILD version仍为`ad0f9ef-dirty`。以下三项均属于session`05DD194E2065ADB0`/boot2；AP08的boot1内存数值不填入本轮。 / The pushed source commit matches the archived production inputs. The binary predates that commit and retains its original BUILD version. All three recordings below share the stated new session/boot; AP08 memory samples belong to its earlier boot.

| 实测 / Recording | 结果与范围 / Result and scope |
| --- | --- |
| [VOLT1 USB＋BLE 600秒](docs/audit/artifacts/cdc_recovery_acceptance_20261116/bss-volt600-01/bss-volt600-01.gate.json)；[指标](docs/audit/artifacts/cdc_recovery_acceptance_20261116/bss-volt600-01/bss-volt600-01.metrics.json) | **600.000132044秒；PASS_CASE_SCREEN**。Capture/USB/BLE **61.279399/61.278351/60.975142 FPS**。USB 36,767帧连续；BLE 36,585帧、缺182，**182/(36,585+182)=0.495009% ≤2.5%**。 / USB is internally consecutive; BLE meets the normal FULL numeric gap budget for this case. |
| [50次模式切换](docs/audit/artifacts/cdc_recovery_acceptance_20261116/bss-final50-01/bss-final50-01.gate.json)；[原summary](docs/audit/artifacts/cdc_recovery_acceptance_20261116/bss-final50-01/bss-final50-01.summary.json) | **50 attempted/applied/verified**，observer均闭合；50步均有匹配BLE帧，每步首个USB验证帧也在BLE实收。完整BLE窗口106帧，其中104帧匹配50步身份；CAP8恢复已查询核验。 / All 50 transitions have USB and BLE evidence; the 106-frame observer window is distinct from the 104 step-matched frames. |
| [SHORT保护快照](docs/audit/artifacts/cdc_recovery_acceptance_20261116/bss-short-01/bss-short-01.gate.json) | **SHORT_V2_FULL_SNAPSHOTS_OBSERVED**；120.000141602秒CAP8窗口，BLE实收117个可信完整快照，116个相邻host完整接收间隔最大**1.035322708秒**。REL恢复与CAP8恢复均query verified。 / These are host receipt intervals, not an over-air timing or future delivery guarantee. |

600秒的独立ADSHEALTH窗为**0/390,581 ADC CRC拒绝**（598.474691秒内检查读数增量，含重试尝试，非帧分母）；USB/BLE无效点**1,536/1,499**保留，USB均为S1D2的X12。完整可信帧不等于每点有效。两通道共同身份/序号的36,583帧原始wire SHA一致、差异0；184个USB未匹配与2个BLE未匹配是跨窗口匹配统计，不能再加到182缺口上。50切换的BLE窗口保留**1,303无效点，含1,296 open**；SHORT本窗口USB/BLE无效点均0，但协议完整性的定义始终保留无效点，不用有效点筛掉帧。 / The separate ADC health span has zero CRC rejections over 390,581 checked reads; it is not a frame error rate. Invalid points remain visible in both the long run and transition recording. Common USB/BLE frames match their original wire bytes. Cross-stream unmatched counts are not additional independent losses, and complete protocol receipt does not require every point to be valid.

600秒CPU只取得五个owner自身采样跨度内的runtime比例：BLE_TX **17.7208%**、TRANSPORT **6.9737%**、ASYNC_LOG **11.5892%**、SERIAL_TX **10.6899%**、SERIAL_CTRL **0.8002%**。BLE_CTRL前样本过时、WIFI_CTRL前样本不可用；**整机CPU、SDK CPU与blocked time均UNKNOWN**，不能把这些不同时窗比例相加当总CPU。STK50观测总free heap **8,288,768 B**、固件生命周期min **8,281,524 B**，包含可用PSRAM，不是internal/DMA余量；最大连续块未测。本窗最小剩余栈log/transport/bleTx/bleCtrl/serialCtrl为**12,492/3,100/2,084/5,196/1,652 B**。 / These are task runtime ratios over separate observation spans, not whole-device utilization. Total heap includes PSRAM; the lifetime minimum is not a window-only minimum, and no contiguous internal/DMA block measurement is available for this recording.

三项正式录制均未观察到FAULT/RESET，600秒和50切换CAP8恢复均核验，SHORT还恢复原REL；这些结果不建立精度、零风险或整体PASS。600秒只通过正常FULL case筛查；50切换与SHORT不套用正常FULL的2.5%缺口预算。[详细审计](docs/audit/firmware_reliability_fix_report.md#bss-current-hil-20261125)。 / No FAULT/RESET is observed in these formal recordings, and the stated recovery queries succeed. The scoped evidence does not establish measurement precision or overall acceptance. Transition and SHORT sequence gaps use their own semantics; the normal FULL loss budget is not applied to them.

<a id="matrix-bss-01-02"></a>

### 原始32配置矩阵 / Original 32-configuration matrix

两段记录使用相同 ELF `179f9183e2c31b35c906c9b811bb2ca1cf4d16dbca43ee3a1c6c4371c0f56a2d`、session `05DD194E2065ADB0`、boot `2`。每个已运行配置有一个约120秒的正式 USB 与独立 BLE 观察窗口。下表列出10个闭合窗口。VOLT 3–8行、RES 1–8行、MIXED 1–8行共22项未启动。
Both runs share the ELF, session and boot above. Ten configurations reached a formal USB and independent BLE window. The remaining 22 configurations were never started in these two runs. [完整矩阵 / Full matrix](docs/audit/artifacts/cdc_recovery_acceptance_20261116/bss-matrix-combined-01.md) · [逐次尝试与 CPU、heap、stack / Attempts and resource samples](docs/audit/artifacts/cdc_recovery_acceptance_20261116/bss-matrix-combined-01.json)

| 模式 / Mode | 行 / Rows | 判定 / Result | 采集 / Capture FPS | USB FPS | BLE FPS | USB 完整 / 缺口 · complete / missing (%) | BLE 完整 / 缺口 · complete / missing (%) | ADC CRC 拒绝 / 检查 · rejected / checked | 无效点 / Invalid (USB / BLE) |
|---|---:|---|---:|---:|---:|---:|---:|---:|---:|
| CAP | 1 | PASS | 68.073 | 68.076 | 68.074 | 8,170 / 0 (0.0000%) | 8,169 / 0 (0.0000%) | — | 0 / 0 |
| CAP | 2 | PASS¹ | 40.298 | 40.298 | 40.295 | 4,836 / 0 (0.0000%) | 4,835 / 0 (0.0000%) | — | 0 / 0 |
| CAP | 3 | PASS | 28.137 | 28.135 | 28.136 | 3,376 / 0 (0.0000%) | 3,376 / 0 (0.0000%) | — | 0 / 0 |
| CAP | 4 | PASS | 21.882 | 21.882 | 21.882 | 2,626 / 0 (0.0000%) | 2,626 / 0 (0.0000%) | — | 0 / 0 |
| CAP | 5 | PASS | 17.893 | 17.894 | 17.893 | 2,147 / 0 (0.0000%) | 2,147 / 0 (0.0000%) | — | 0 / 0 |
| CAP | 6 | PASS | 15.139 | 15.139 | 15.139 | 1,816 / 0 (0.0000%) | 1,816 / 0 (0.0000%) | — | 0 / 0 |
| CAP | 7 | PASS | 13.128 | 13.130 | 13.129 | 1,576 / 0 (0.0000%) | 1,576 / 0 (0.0000%) | — | 0 / 0 |
| CAP | 8 | PASS | 11.577 | 11.578 | 11.578 | 1,390 / 0 (0.0000%) | 1,390 / 0 (0.0000%) | — | 0 / 0 |
| VOLT | 1 | PASS | 61.087 | 61.085 | 60.758 | 7,331 / 0 (0.0000%) | 7,290 / 43 (0.5864%) | 0 / 77,984 | 283 / 279 |
| VOLT | 2 | **FAIL** | 42.086 | 42.032 | 42.090 | 5,043 / 7 (0.1386%) | 5,050 / 0 (0.0000%) | 0 / 107,060 | 0 / 0 |

¹ CAP2 首次 BLE 观察器在就绪前关闭，未开始正式录制。第二段 CAP2 窗口通过。其余8个通过项各自首次即通过。USB 要求完整帧零缺口，正常 FULL 的 BLE 缺口预算为2.5%。分母为该接收端首末可信序号之间的实收与缺口之和。CAP 行的 ADC CRC 均无可用检查分母。VOLT 的 ADC 分母来自各自 ADSHEALTH 诊断窗。
CAP2 first stopped before formal capture when its BLE observer closed before readiness. The second CAP2 attempt passed. USB requires zero gaps, and normal FULL BLE allows up to 2.5% missing within its observed sequence endpoints. ADC counts use checked reads in a separate diagnostic interval. A dash means no checked-read denominator was observed. [CAP2首次记录 / First CAP2 attempt](docs/audit/artifacts/cdc_recovery_acceptance_20261116/bss-matrix-01/bss-matrix-01-cap2/bss-matrix-01-cap2.command.json) · [筛查明细 / Gate details](docs/audit/artifacts/cdc_recovery_acceptance_20261116/bss-matrix-combined-01.json)

**VOLT2 FAIL.** 两端首末序号均为 76,513–81,562。USB 缺 77,016 和 77,021–77,026，BLE 收齐区间内的5,050帧。两端共有的5,043帧原始字节 SHA-256 全部相同。CAP8 恢复查询成功，USB 缺口使本次配置筛查失败。
Both receivers observed the same endpoints. Seven sequences are missing on USB and present as complete frames on BLE. All 5,043 common frames have matching original wire bytes. The capture verifies a USB gap and the subsequent CAP8 recovery. The cause of the gap is unresolved by this window. [VOLT2原始判定 / Original VOLT2 gate](docs/audit/artifacts/cdc_recovery_acceptance_20261116/bss-matrix-02/bss-matrix-02-volt2/bss-matrix-02-volt2.gate.json)

#### 参考夹具 / Reference fixture

两个标称33 pF电容分别接在 S4D4、S5D5。表内 LC 均值包含背景电容，P05–P95 为实际样本分布。最后一列依次列出逐帧减去同排、同列、全部悬空单元中位数的三个差值均值。两个窗口均使用正式 USB 原始帧。
The nominal capacitors are 33 pF. Mean LC includes the background capacitance. P05–P95 gives the observed sample spread. The three estimates subtract a predeclared median background on each paired frame. [CAP8夹具报告 / Matrix CAP8 fixture](docs/audit/artifacts/cdc_recovery_acceptance_20261116/bss-matrix-02-cap8-fixture-01/cap8.report.md) · [SHORT下的USB夹具报告 / SHORT USB fixture](docs/audit/artifacts/cdc_recovery_acceptance_20261116/bss-short-01/bss-short-01.fixture-window.report.md)

| 窗口 / Window | 点 / Cell | 有效 / 无效 · valid / invalid | LC均值 / Mean LC (pF) | P05–P95 (pF) | 差值均值，同排 / 同列 / 全部悬空 · mean difference, row / column / all (pF) |
|---|---|---:|---:|---:|---:|
| CAP8 | S4D4 | 1,390 / 0 | 92.431562 | 92.423831–92.441396 | 32.494354 / 13.438154 / 32.493448 |
| CAP8 | S5D5 | 1,390 / 0 | 77.405972 | 77.397519–77.414335 | 17.468394 / 11.691997 / 17.467859 |
| SHORT | S4D4 | 1,409 / 0 | 92.421321 | 92.411281–92.431356 | 32.508467 / 13.434103 / 32.507510 |
| SHORT | S5D5 | 1,409 / 0 | 77.381926 | 77.372815–77.389611 | 17.468790 / 11.690489 / 17.468115 |

60个悬空单元各自的时间中位数在矩阵 CAP8 为 57.092659–80.003703 pF，在 SHORT 为 57.070793–79.986162 pF。两处电容均缺同一单元卸载基线，精度状态为 `BASELINE_REQUIRED`。S1D1、S8D8 的标称10 kΩ电阻在这两段矩阵中没有 RES 窗口，阻值未测。
Across 60 floating cells, the ranges above describe their individual time medians. Validating either capacitor requires an unloaded reading of that same cell. The two matrix runs contain no resistance measurement for either 10 kΩ reference. [CAP8字节核对 / CAP8 byte verification](docs/audit/artifacts/cdc_recovery_acceptance_20261116/bss-matrix-02-cap8-fixture-01/cap8.validation.json) · [SHORT字节核对 / SHORT byte verification](docs/audit/artifacts/cdc_recovery_acceptance_20261116/bss-short-01/bss-short-01.fixture-window.validation.json)

<a id="wifi-owner-v1"></a>

## Wi-Fi SDK 独立任务与手动重启 / Isolated Wi-Fi SDK execution and manual restart

**初版owner隔离的[接口交接02](docs/audit/artifacts/cdc_recovery_acceptance_20261116/wifi-owner-contract-handoff-02.json)、[14个host入口](docs/audit/artifacts/cdc_recovery_acceptance_20261116/wifi-owner-host-02/manifest.json)与[独立审查03](docs/audit/artifacts/cdc_recovery_acceptance_20261116/wifi-owner-independent-review-03.json)保留。** 随后的实机栈溢出已确认；`wifi-stack-01`将owner栈改为6,144 B，并把执行/初始化工作区保留到owner生命周期。[新镜像79入口/构建](docs/audit/artifacts/cdc_recovery_acceptance_20261116/wifi-stack-01-validated-identity.json)通过，但[AP03](docs/audit/artifacts/cdc_recovery_acceptance_20261116/wifi-stack-control-evidence-01.json)仍因NO_MEM失败。该镜像只有返回失败路径的stack/heap和一次隔离后手动重启证据。后续wifi-memory-01将Transport的16,135 B CPU文本池一次性迁入PSRAM、内部pool元数据不迁出，固定槽/所有权/CRC不变；AP04与BLE共存仍NO_MEM，两条WTERM已在PC实收。AP04当时没有成功AP；后续AP08已在新ELF179f9183…验证首次AP与USB/BLE共存控制（见开头），UDP实收及新Wi-Fi FPS/CPU仍未验收，image06/screen21不能代替。

**The original [handoff02](docs/audit/artifacts/cdc_recovery_acceptance_20261116/wifi-owner-contract-handoff-02.json), [14 host entries](docs/audit/artifacts/cdc_recovery_acceptance_20261116/wifi-owner-host-02/manifest.json) and [independent review03](docs/audit/artifacts/cdc_recovery_acceptance_20261116/wifi-owner-independent-review-03.json) remain.** A later hardware stack overflow is confirmed; wifi-stack-01 uses a 6,144-byte owner stack and owner-lifetime execution/init workspace. Its 79 entries/build pass, while AP03 still fails with NO_MEM. That image observes only the returned failure path and one subsequent manual restart. Later wifi-memory-01 moves the 16,135-byte Transport CPU text pool to one permanent PSRAM allocation, retaining internal metadata, fixed slots, ownership and CRC. AP04 still returns NO_MEM during BLE coexistence, while both WTERM records are received by the PC. AP04 did not establish successful AP. The later AP08 run above verifies first AP startup with USB/BLE control on ELF179f9183…, while UDP receipt and new Wi-Fi FPS/CPU acceptance remain unqualified; image06/screen21 is not substitute evidence.

此前 `sendto(..., MSG_DONTWAIT)` 仍可能在 lwIP mailbox/semaphore 等待，AP初始化/停止也可能停在SDK内。新 `sensorarrayWifiOwner.c`（本轮新增）将这些工作移到持久 `wifiOwner` 任务；Transport、BLE/USB控制只提交深拷贝请求或读取snapshot，不进入这些SDK调用。独立RX任务只负责现有UDP控制接收，不能通过删任务/关socket声称它已停止。应用Wi-Fi事件callback只记录station事件；SDK默认事件handler仍由SDK线程执行。双核采集、双FDC并行、ADS缓存/PGA及科学wire不改，正式数据不改标签或CRC。

The earlier `sendto(..., MSG_DONTWAIT)` can still wait inside lwIP mailbox/semaphore code, and AP initialization/stopping can block inside the SDK. New `sensorarrayWifiOwner.c` moves that work to a persistent `wifiOwner` task. Transport and BLE/USB control copy requests or read snapshots without entering those SDK calls. A separate persistent RX task receives UDP control; deleting it or closing its socket would not prove it had stopped. The application event callback only records station events; vendor default handlers still run in the SDK thread. Dual-core acquisition, parallel FDCs, ADS cache/PGA and scientific wire identity/CRC remain unchanged.

| 层/Layer | 当前机制 / Current mechanism |
| --- | --- |
| 默认 OFF / Default OFF | 只有静态metadata，不创建owner任务、timer或payload；实际boot身份和启动资源预留完成后才允许首次AP。 / Static metadata only; no owner task, timer or payload until the real boot identity and startup reservation gate permit AP. |
| 容量 / Storage | 一次性PSRAM\|8BIT **7,424 B**，无internal fallback：2×2,304普通DATA/LOG、1×2,304 lifecycle、1×512 CTRL。wifi-stack-01的owner/RX栈分别6,144/4,096 B，执行/初始化工作区为owner生命周期；当前Core0/priority5，metadata/锁/栈仍需内部内存，不是DMA。 / One permanent7,424-byte PSRAM allocation with no internal fallback; separate ordinary/lifecycle/CTRL slots. In wifi-stack-01, owner/RX stacks are 6,144/4,096 bytes with owner-lifetime execution/init workspace. Internal metadata/locks/stacks remain; this is not DMA. |
| Bluedroid host分配 / Bluedroid host allocation | bt-host-memory-01启用官方`CONFIG_BT_ALLOCATION_FROM_SPIRAM_FIRST=y`，动态host malloc/calloc优先PSRAM、失败可退内部；不是全局malloc或controller DMA/栈/队列迁移。BLE前internal free增加5,528 B仍不足以通过AP05；不改变MTU、队列容量或输出速率策略。 / Official host malloc/calloc preference retains internal fallback and existing controller/task/queue placement. The observed extra 5,528 internal bytes do not make AP05 succeed; MTU, queue capacities and rate policy are unchanged. |
| 所有权 / Ownership | `WRITING → PENDING → OWNED`；copy/SDK/callback在mux外。超时不释放/覆盖未知owner，晚返回才有释放证据；quarantine不因晚成功或OFF清除。 / Copy/SDK/callback work stays outside the mux. Timeout never reuses unknown-owned storage; actual return proves release eligibility, without clearing quarantine. |
| 期限 / Deadlines | AP执行5s＋最多1s cleanup，总6s；单次send最多500ms；100ms观察周期。另有1s send-start TTL，继承保护descriptor更早deadline，不续期；禁止过期后开始发送，**不承诺压力/隔离时pending槽1s内回收**。 / AP execution5s plus at most1s cleanup, total6s; send500ms; observer100ms. A separate1s send-start TTL inherits any earlier protected deadline. It prevents starting stale work, not guaranteed reclamation under priority pressure/quarantine. |
| OFF / Recovery | `WIFI=OFF`立即关闭软件DATA/LOG输出并推进epoch，不执行radio stop/close/deinit，不清quarantine。显式手动RESTART是尝试恢复路径，不能将OFF等同断电。 / OFF closes the software output gate and advances epochs without stopping/deinitializing the radio or clearing quarantine. Explicit manual restart is a recovery attempt, not proof that OFF powered hardware down. |

这些都是应用隔离策略，不是SDK取消API或硬实时完成保证。owner bootstrap的payload/timer/task分配失败会保留首错且不重复创建部分存活对象。`esp_wifi_init`返回错误也可能隐藏SDK内部deinit失败，相关netif/handler状态未知时保留并隔离；不自动从HIGH降到BLE_COMPAT重试。不在观察timer中执行SDK、日志、free或reset。

These are application isolation policies, not SDK cancellation APIs or hard-real-time completion guarantees. Partial payload/timer/task bootstrap failure remains sticky instead of recreating live objects. `esp_wifi_init` may return an error while hiding failed internal deinitialization; uncertain netif/handler ownership is retained and quarantined. No automatic HIGH-to-BLE_COMPAT retry is added. The observer timer performs no SDK call, logging, freeing or reset.

能力协商先看 `PROTO?.caps` 的 `caps2_v1`，再读 `CAPS2?` 的 `wifi_owner_v1`。新 `WIFI? ver=2` 分开requested、outputGate、backend、resource/initErr、quarantine和最新request；capability不代表AP已ready。`WIFI=AP`先给accepted/id/epoch，未完成时ok0/pending1；`WTERM`可以早于或晚于ACK，使用无副作用 `WIFIREQ?<非零u32 id>`查询确切终态，不通过重发设置查询结果。相同已ready配置获得新id的幂等applied，旧缓存结果不改写；一个活动请求加两个终态缓存，淘汰后known0/unknown。终态最多3次本地提交、间隔200ms；queued不是PC ACK。

Negotiate `caps2_v1` in PROTO, then `wifi_owner_v1` from `CAPS2?`. Version2 `WIFI?` separates requested mode, output gate, backend, resources/init error, quarantine and latest request; capability is not AP readiness. `WIFI=AP` returns acceptance/id/epoch while pending remains ok0/pending1. WTERM may precede or follow ACK; use side-effect-free `WIFIREQ?<nonzero-u32-id>` for that exact result instead of replaying a setter. Re-requesting the same ready configuration gets a new idempotently applied result without SDK work. One active request plus two cached terminals are retained; eviction yields known0/unknown. At most three local terminal submission attempts occur200ms apart; queued is not PC acknowledgment.

`WIFIGUARD?`保留original err、独立cleanupErr、真实sdkErr/errno/returned、phase/callId/期限/late/held与completionKnown；时间为单调µs，未知显式unknown。`WIFIQ?D/L/C`逐通道统计queued/rejected/sent/drop/expired/stale/partial和driverAcceptedBytes；sent及旧Transport Wi-Fi sent计数只在实际完整sendto返回后增加，晚返回照实记一次，仍不证明空中/PC接收。drop子类重叠，不能相加当独立丢包。新增 `CPUOWNER?7=WIFI_OWNER`；旧0–6及SCHED/SCHEDWHY字段不重释。owner仅在自身完成工作后缓存stackMinBytes，未采样unknown，不在查询中扫描阻塞任务。完整字段见[主机契约](docs/integration/firmware_host_contract.md#wifi-owner-contract-v1)。

`WIFIGUARD?` preserves the original error, separate cleanup error, actual SDK error/errno/return count, phase/call identity/deadline/late/held and completion knowledge. Times are monotonic microseconds; unavailable values remain unknown. `WIFIQ?D/L/C` exposes per-channel admission, completion and drop counters. Sent, including legacy Transport Wi-Fi sent counters, increments only after an actual full-length sendto return, including one recorded late return; it proves neither air transmission nor PC receipt. Drop subcategories overlap. New `CPUOWNER?7` identifies WIFI_OWNER; roles0–6 and existing SCHED/SCHEDWHY fields retain their meaning. Stack minima are cached by the owner only after completed work, not scanned by queries while it is blocked. See the [host contract](docs/integration/firmware_host_contract.md#wifi-owner-contract-v1) for all fields.

<a id="manual-restart-noos"></a>

### 显式手动重启的SDK依赖 / SDK dependency of explicit manual restart

新增 `sensorarrayUsbOtgManualRestart(bool)` 仅由已通过本地接受栅栏的 `RESTART/USBBOOT` 路径调用。保留原 **250,000µs** USB owner准备期限；成功后经已有 `__wrap_esp_restart_noos` 进入当前 **ESP32-S3 / IDF5.5.5** noos终止路径，跳过 `esp_restart()` 先调用的shutdown handler列表，避免新发生的 `esp_wifi_stop` 阻塞。AUTO/普通重启仍用原路径。USB准备失败/晚确认仍走原emergency分支，保留未知DMA缓冲所有权并进入既有USJ恢复策略，不伪造ROM已就绪、取消成功或模拟SAFE。

New `sensorarrayUsbOtgManualRestart(bool)` is called only for explicit RESTART/USBBOOT after the existing local acceptance fence. It retains the **250,000µs** USB-owner preparation deadline. On success it enters the current **ESP32-S3/IDF5.5.5** terminal noos path through the existing `__wrap_esp_restart_noos`, bypassing the shutdown-handler list that `esp_restart()` runs first, including a newly blocked `esp_wifi_stop`. AUTO/generic restarts retain their existing path. Failed/late USB preparation retains the emergency branch and unknown DMA-buffer ownership, with the existing USJ recovery policy; it does not invent ROM readiness, cancellation or analog SAFE.

这是明确绑定当前SDK私有API的终止逃生路径，不是通用FreeRTOS/cache-off API。SDK在后续cache/core/clock收尾前设置RTC watchdog，但RC频率/eFuse/reset层级意味着不能宣称精确1s物理恢复。跳过最终RTC/wall-time同步可能影响跨重启日历时间；科学帧仍使用boot域esp_timer，跨boot不比较seq/time。没有新增自动重启循环、删worker或取消SDK。独立[74个USB场景及源码审查](docs/audit/artifacts/cdc_recovery_acceptance_20261116/manual-restart-independent-review-01.json)是主机验证；必须在最终ELF核对成功分支确实指向wrapper，再分别实测普通RESTART、USBBOOT、实际AP成功/失败/隔离条件下的恢复与新BUILD/session/READY。按键、Windows枚举及真实阻塞SDK恢复不能由host stub证明。

This terminal escape deliberately depends on the pinned private SDK API; it is not a general FreeRTOS or cache-off API. The SDK arms an RTC watchdog before subsequent cache/core/clock teardown, but RC frequency, eFuse settings and reset levels preclude an exact one-second physical recovery claim. Skipping final RTC/wall-clock synchronization may affect calendar-time continuity; science frames use boot-scoped esp_timer time and do not compare sequence/time across boots. No automatic reset loop, worker deletion or SDK cancellation is added. [Independent review and74 USB host scenarios](docs/audit/artifacts/cdc_recovery_acceptance_20261116/manual-restart-independent-review-01.json) are host evidence. The final ELF must confirm the wrapper target, followed by separate RESTART/USBBOOT and safely available AP/failure/quarantine recovery observations with fresh BUILD/session/READY. Host stubs do not prove buttons, Windows enumeration or recovery from a live blocked SDK.

<a id="final-acceptance-runner-guards"></a>

## 最终验收采集的身份与停止条件 / Identity and stop conditions for final acceptance captures

本轮新增 [身份检查模块](tools/sensorarray_capture_identity.py) 与 [闭合数据筛查](docs/audit/artifacts/cdc_recovery_acceptance_20261116/scripts/qualify_closed_capture.py)，并接入 600 秒、50 次切换、SHORT 和 [32 配置入口](docs/audit/artifacts/cdc_recovery_acceptance_20261116/scripts/run_cdc_matrix_failfast.py)。这些入口要求从已保存的只读查询取得完整 **64 个十六进制字符的 ELF SHA256、16 个十六进制字符的 session、bootId**；在第一个设置命令之前实际执行 `CONFIG? → BUILD? → CONFIG?`，要求两端完整配置一致。CAP8 清理前重新核验同一镜像与 boot；初始身份未知时连清理设置也不发送。已观察到 reset 或不同 boot 的 CONFIG 会立即撤销设置权限。该软件身份依据不证明模拟隔离，也不能消除尚未观察到的重启竞态。

The new [identity module](tools/sensorarray_capture_identity.py) and [closed-capture gate](docs/audit/artifacts/cdc_recovery_acceptance_20261116/scripts/qualify_closed_capture.py) support the 600-second, 50-transition, SHORT and [32-configuration entry points](docs/audit/artifacts/cdc_recovery_acceptance_20261116/scripts/run_cdc_matrix_failfast.py). Each requires a recorded full **64-hex ELF SHA256, 16-hex session and bootId**. Before its first setter, the collector performs an actual `CONFIG? → BUILD? → CONFIG?` bracket and requires the complete configurations to agree. CAP8 cleanup repeats the image/boot check; unknown initial authority permits no cleanup setter. An observed reset or changed CONFIG boot revokes authority. This identifies software, not physical isolation, and cannot eliminate an unobserved restart race.

32 配置按 CAP/VOLT/RES/MIXED × 1–8 行依次运行；每项独立启动 BLE 与 CDC debug 观察者，连接和准备不计入完整 **120 秒**。正式采集结束、CAP8 查询核验、两个观察者退出并写完日志之后，才使用真实重组器与科学帧 parser 筛查这一项；任何 **FAIL 或 UNKNOWN 都停止，不自动重试**。每项保留独立原始字节、事件、观察者收尾、分析与判定。超时后的进程 kill 不被称为 CAP8 或物理安全；无法确认的退出仍阻止下一项。命令控制使用 CDC0，debug 使用同 USB serial 的 CDC2，一个端口只有一个本任务 reader。

The matrix runs CAP/VOLT/RES/MIXED × 1–8 rows in order. Each case starts independent BLE and CDC-debug observers; connection/preparation is outside its full **120-second** window. The gate runs the real reassembler and scientific parser only after formal capture, query-verified CAP8 cleanup, observer exit and completed journals. **FAIL and UNKNOWN both stop without automatic retry.** Raw bytes, events, process closure, analysis and the decision remain separate for every case. Killing a timed-out process does not establish CAP8 or physical safety; unknown termination blocks the next case. Formal control/data uses CDC0 and debug uses CDC2 with the same USB serial, with one reader per port in this task.

正常 FULL 的 v2 缺口公式仍是同 boot/配置、BLE 自身可信端点间的 `missing / (ordered_received + missing) ≤ 0.025`。另加独立的观测充分性要求：USB 与 BLE 的可信完整帧首/末分别距各自采集窗口边界不超过 **1 秒**，设备采样时间跨度至少为 `requested_seconds − 2`；采集周期超过 1 秒或时间未知时停止为 UNKNOWN，人工审查，不自动放宽阈值。这防止“订阅完整 120 秒但只收到两帧”或“两相邻旧帧延迟到窗口两端”被判通过。**1 秒是本轮连续 FULL 筛查边界，不是 BLE 交付保证。** SHORT 普通预览不使用 FULL 的缺口预算。跨窗 G 片段/帧保留独立诊断，只有明确完全位于窗口内的 CRC/身份/格式错误进入该窗口完整性判定；整个日志中的控制完整性、fault/reset 仍检查。完整帧仍保留 Xhh/无效点，不等于所有点有效或精度合格。

The normal-FULL v2 metric remains `missing / (ordered_received + missing) ≤ 0.025` within the BLE observation endpoints of one boot/configuration. A separate observation-sufficiency guard requires the first/last trustworthy USB and BLE complete frames to lie within **one second** of their respective capture edges, with device sample-time span at least `requested_seconds − 2`. A capture period above one second or unknown timing produces UNKNOWN for manual review; thresholds never expand automatically. This rejects a full subscription window containing only two frames, including adjacent old frames delayed to opposite edges. **The one-second check is a FULL-screen sufficiency rule, not a BLE delivery guarantee.** SHORT previews do not use the FULL gap budget. Cross-window fragments/frames retain separate diagnostics; clearly contained CRC/identity/format failures are checked within the window, while control integrity and fault/reset records are checked across the whole journal. Complete frames retain invalid/Xhh points and do not imply all points valid or precision acceptance.

已实际运行 [122 项定向主机/PTY回归与源码冻结记录](docs/audit/artifacts/cdc_recovery_acceptance_20261116/final-acceptance-runner-freeze-01.json)，均 exit0；这些是合成测试，不是新的板上数据。使用原始实录重新运行 [screen20/21 离线筛查](docs/audit/artifacts/cdc_recovery_acceptance_20261116/final-acceptance-gate-replay-02.json)：screen20 保持 FAIL，screen21 保持 PASS_NUMERIC_SCREEN；原 raw/summary 哈希不变，旧记录不补造新的采集前身份核验。最终 600 秒、32 配置和 50 切换仍待在最终同一镜像执行，整体结论仍 NOT_ESTABLISHED。

All [122 targeted host/PTY regressions](docs/audit/artifacts/cdc_recovery_acceptance_20261116/final-acceptance-runner-freeze-01.json) actually ran with exit0; these synthetic tests are not new board measurements. [Replaying the original screen20/21 recordings](docs/audit/artifacts/cdc_recovery_acceptance_20261116/final-acceptance-gate-replay-02.json) retains screen20 FAIL and screen21 PASS_NUMERIC_SCREEN. Original raw/summary hashes are unchanged, and historical runs do not acquire invented pre-mutation authority. The final 600-second, 32-configuration and 50-transition runs remain pending on the final common image; overall acceptance remains NOT_ESTABLISHED.

独立[最终runner审查02](docs/audit/artifacts/cdc_recovery_acceptance_20261116/final-acceptance-runner-independent-review-02.json)再次实际运行全部122项，15个冻结源码hash前后相同；原两帧反例在新gate返回UNKNOWN。[独立实录回放03](docs/audit/artifacts/cdc_recovery_acceptance_20261116/final-acceptance-gate-independent-replay-03.json)重新解析screen20/21，仍为FAIL/仅数值PASS，原始输入不改。这个PASS仅授权继续下一次HIL尝试，不是整体验收。

Independent [runner review02](docs/audit/artifacts/cdc_recovery_acceptance_20261116/final-acceptance-runner-independent-review-02.json) reruns all122 tests with all15 frozen source hashes unchanged; the original two-frame counterexample now returns UNKNOWN. [Independent recording replay03](docs/audit/artifacts/cdc_recovery_acceptance_20261116/final-acceptance-gate-independent-replay-03.json) reparses screen20/21 and retains FAIL/numeric-only PASS without changing originals. This review permits the next HIL attempt, not overall acceptance.

<a id="screen21-typed-metadata-results"></a>

## 2026-11-21 screen21：保留配置域的typed metadata与正常缺口预算 / Typed metadata and the normal gap budget

build06本轮新增`sensorarrayFramePublicationMetadata → sensorarrayTransportPublishFrameMetadata → sensorarrayTransportApplyFrameMetadata`，将成功真实encoder对应的不可变frame元数据同步复制到Transport私有slot，减少同一包头的重复数字解析。CAP的旧gen/rid仍取ROWS域；VOLT/RES仍取MODE；MIXED的行/配置字段保留原域，revision0省略扩展时也保持旧语义。raw/旧identity-only API仍走原parser；typed POD本身**不是任意输入的CRC或完整帧认证**，调用者必须保持成功encoder与同一个frame配对。wire、CRC、invalid token、SHORT、保护池和SDK owner生命周期不变。build05内部2,304 B工作副本未证实收益，已从build06撤回；不能把其内存占用继续计入新镜像。

Build06 adds `sensorarrayFramePublicationMetadata → sensorarrayTransportPublishFrameMetadata → sensorarrayTransportApplyFrameMetadata`, synchronously copying immutable metadata from the same successfully encoded frame into a private Transport slot, avoiding repeated numeric header parsing. Legacy CAP gen/rid remains ROWS-owned, VOLT/RES remains MODE-owned, and MIXED row/profile fields keep their domains. Revision-zero omission behavior is preserved. Raw and legacy identity-only APIs still use the existing parser. The typed POD **does not authenticate arbitrary bytes or prove CRC/full-frame validity**; callers must preserve the successful encoder/frame pairing. Wire bytes, CRC, invalid markers, SHORT, protected pools and SDK-owner lifetimes are unchanged. Build05's unproven 2,304-byte internal working copy is removed in build06.

[独立审查](docs/audit/artifacts/cdc_recovery_acceptance_20261116/typed-frame-metadata-independent-review-01.json)执行4个实际测试入口：96组真实encoder cases涵盖1–8行/C/V/R/M、域冲突与revision0/最大值，metadata/SHORT/原字节完全一致、16类无效输入拒绝，入队后修改producer不污染slot；旧parser66,896差分/20原wire、22 pipeline与恢复control回归通过。实际host路径计数从96次parser/1040次strtoull降为0/0；**这不是ESP32 CPU或吞吐收益测量**。本轮7个生产文件增量审查未发现新增P0/P1，不覆盖另行发现的Wi-Fi SDK owner隔离问题。

The [independent review](docs/audit/artifacts/cdc_recovery_acceptance_20261116/typed-frame-metadata-independent-review-01.json) executes four test entries. Ninety-six real-encoder cases cover 1–8 rows, C/V/R/M, distinct domains, revision zero and maximal values, preserving metadata, SHORT and original bytes; sixteen invalid inputs are rejected and producer mutation does not corrupt queued slots. The legacy parser's 66,896 differential cases/20 wire fixtures, 22 pipeline cases and recovery-control tests pass. Host call counts decrease from 96 parser/1,040 strtoull calls to zero: **these counts are not target CPU or throughput measurements**. The scoped seven-file review finds no new P0/P1; separate Wi-Fi SDK-owner isolation remains outside that conclusion.

[构建与73入口整轮](docs/audit/artifacts/cdc_recovery_acceptance_20261116/ble-snapshot-06-validated-identity.json)均exit0，195生产输入前后一致，版本8a1794c-dirty/IDF5.5.5，app **1,616,144 B**；bin SHA256 `70b95500569e56a1aff4af1490f4ce18e158b6473f58323ec68120a0446770ac`、完整ELF `c47a3710a770506abd030d03790b58c137150c9c1cba86a596f07962114a3c4e`，[bin/ELF/map归档](docs/audit/artifacts/cdc_recovery_acceptance_20261116/ble-snapshot-06-image-archive.json)回验。[实际启动](docs/audit/artifacts/cdc_recovery_acceptance_20261116/ble-snapshot-06-boot-verification.json)READY1/CAP8、session630DC17010A097FF/boot1；SPI2MHz/2000 reads、CRC/status/register0、I²C/SPI重叠693,889µs。MEM scan created/stack16,384 B，创建前free/largest40,168/30,708，后22,368/13,812 B，启动查询intFree/largest/min28,608/13,812/22,260 B；均为独立顺序观察。

The [build and all 73 host entries](docs/audit/artifacts/cdc_recovery_acceptance_20261116/ble-snapshot-06-validated-identity.json) pass with 195 matching production inputs. Version is 8a1794c-dirty/IDF5.5.5; app size is **1,616,144 bytes**, with the full hashes above and a verified [bin/ELF/map archive](docs/audit/artifacts/cdc_recovery_acceptance_20261116/ble-snapshot-06-image-archive.json). [Actual startup](docs/audit/artifacts/cdc_recovery_acceptance_20261116/ble-snapshot-06-boot-verification.json) verifies READY1/CAP8 in session630DC17010A097FF, boot1. SPI qualifies at 2MHz over 2,000 reads with no CRC/status/register rejection and 693,889µs I²C/SPI overlap. The 16,384-byte scan stack is created, with sequential memory observations above; they do not isolate allocation costs.

[screen21闭合判定](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-21/screen-21-verdict.json)为 **OBSERVED_NOT_ACCEPTED**，VOLT1 rev4、mgen3/mrid4、rgen2/rrid3、pgen3/prid4：

The [closed screen21 verdict](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-21/screen-21-verdict.json) is **OBSERVED_NOT_ACCEPTED**, scoped to VOLT1 revision4 and the configuration domains above:

| 维度 / Dimension | 实录 / Observation | 限制 / Limit |
|---|---|---|
| 完整时间 / Duration | **120.000119494 s** | formal单调起止；准备/收尾不计入。 / Formal endpoints exclude preparation/cleanup. |
| Capture / USB / BLE FPS | **64.195756 / 64.233205 / 63.079023** | 不同设备/PC端点分母，不能混用。 / Separate device/receipt denominators. |
| USB | **7,707**连续，seq1607–9313 | 同identity内部缺口0。 / Internal continuity only. |
| BLE | **7,569**完整，seq1604–9313，缺**141** | **141/7,710=1.828794%≤2.5%**；只通过正常缺口数值预算，不证明RF强弱。 / Numeric budget only, not RF classification. |
| 原包 / Original bytes | **7,567**公共原包SHA一致，CRC/identity/wire错误0 | 不重算假CRC或重新盖章。 / No invented CRC or relabeling. |
| 点有效性 / Point validity | USB/BLE invalid **460/434**，均S1D2 X12；not-fresh/unacquired0 | 完整帧仍含悬空口无效点，不是精度验收。 / Whole frames retain invalid measurements from floating inputs. |
| ADC SPI CRC | **0/81,815** checked reads | 独立 **119.595982425 s**健康窗，even41,470＋odd40,345；engine384→8064、entry0/interval64，末crcRetries unknown。 / Separate diagnostic denominator. |
| PC最大相邻完整间隔 / Maximum adjacent complete interval | BLE **38.049396 ms / 7,568 pairs**；USB **61.369954 ms / 7,706 pairs** | 同identity完成K之间；不含窗口边缘，不保证每个保护ticket。 / Excludes edge waits and ticket ACK guarantees. |
| 收尾 / Recovery | **CAP8_QUERY_VERIFIED**，rev6/boundary9348 | 同BUILD/session，BLE guard ready1/quarantine0/rejected0；未观察fault/reset/WDT不证明绝无。 / Same-image observed recovery only. |

[四层查询](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-21/screen-21-protection-01.json)各有约120.365–120.420秒括号：SRCperiodic/queued+119；NETqueued/bleAccepted+128、derived+9、retry+2、obsolete+3，admitMiss/expiry/failed/policySkip0；BLE保护queued/localComplete/staleNormal/deadlineMiss增量均0。BLESNAP lastSeq1241/age125,755,951µs仍只指保护分类，不代表全部完整帧沉默。FLOWattempts+7,723/capacityReject+135、intentionalSkip/probe0；before pressure/rejectPm112→after full/0，两端rate1000。**不能从141中扣135或3**，也不把2.5%当主动丢弃目标。

The [four independent query brackets](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-21/screen-21-protection-01.json) span roughly 120.365–120.420 seconds. Source periodic/queued increases119; NET candidates/admissions128, derived9, retries2 and obsolete3, with zero admission misses/expiry/failure/policy skips. BLE protected counters remain unchanged; lastSeq1241/age125,755,951µs is protected-only rather than an all-full silence interval. FLOW adds7,723 attempts and135 capacity rejections with no intentional skips/probes; sampled pressure/rejectPm112 becomes full/0 at rate1000 throughout. **Neither135 nor3 may be subtracted from141**, and2.5% is not deliberate loss scheduling.

USBQ独立120.456119291秒窗dDrop/cDrop/expired/cExpired/partial均0，dSubmit+7,737/cSubmit+115、dBytes4,630,634/cBytes27,140，callback bytes unknown。BLETX独立120.400184663秒窗sdkCalls/accepted各+15,182/errors0；BL50独立119.145540729秒窗dropD/L/C为129/306/0。它们不是PC receipt或互斥丢失层，不加总成141。

USBQ's own120.456119291-second bracket has zero new drops/expiry/partial events,7,737 DATA/115 CTRL submissions and4,630,634/27,140 submitted bytes, with callback bytes unknown. BLETX's separate120.400184663-second bracket adds15,182 calls/acceptances and zero errors; BL50's119.145540729-second interval gives DATA/LOG/CTRL drops129/306/0. These do not acknowledge PC receipt or form disjoint losses that sum to141.

各自CPU owner窗：BLE_TX **17.281721%**、Transport **6.437006%**、AsyncLog **10.849593%**、SerialTX **10.794650%**、SerialCTRL **0.782218%**；BLE/Wi-FiCTRL unknown，总CPU/SDK/blocked unknown，不相加。[screen20→21描述比较](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-21/screen20-screen21-comparison.json)中AsyncLog13.143679→10.849593%，BLE缺口2.636043→1.828794%，但BLE_TX17.213336→17.281721%，且同时撤回scratch、不同session/负载/RF；**不能将metadata优化宣称唯一根因或受控因果证明**。

Independent owner ratios are BLE_TX **17.281721%**, Transport **6.437006%**, AsyncLog **10.849593%**, SerialTX **10.794650%** and SerialCTRL **0.782218%**. BLE/Wi-Fi CTRL, total CPU, SDK CPU and blocked time remain unknown; ratios are not summed. The [descriptive comparison](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-21/screen20-screen21-comparison.json) shows lower AsyncLog ratio13.143679→10.849593% and BLE gap2.636043→1.828794%, while BLE_TX rises17.213336→17.281721%. Scratch is also withdrawn and sessions/load/RF differ. **This does not prove typed metadata was the sole cause or constitute a controlled causal experiment.**

STK50 observed aggregate free/lifetime-min为8,274,572/8,271,084 B；log/transport/bleTx/bleCtrl/serialCtrl最小剩余12,500/3,184/2,080/5,208/1,404 B。精确窗internal largest未观测；[后续MEM](docs/audit/artifacts/cdc_recovery_acceptance_20261116/ble-snapshot-postscreen21-01.summary.json)intFree/largest/min26,156/13,812/22,260 B、extFree/largest8,248,968/8,126,452 B仍独立顺序值，不将aggregate heap当internal contiguous。

Complete STK50 observations give aggregate free/lifetime-minimum8,274,572/8,271,084 bytes and remaining-stack minima12,500/3,184/2,080/5,208/1,404 bytes. Capture-window internal-largest-block behavior is unobserved. A [later sequential MEM query](docs/audit/artifacts/cdc_recovery_acceptance_20261116/ble-snapshot-postscreen21-01.summary.json) reports internal free/largest/min26,156/13,812/22,260 and external free/largest8,248,968/8,126,452 bytes. Aggregate heap is not contiguous internal capacity.

verdict、[同criteria v2补充](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-21/screen-21-criteria-v2-supplement.json)与归档命令均exit0；criteria SHA仍`e0ef35a43a75932b8878aaa5c7cc94a6b562a9e4e1f86e50055ccd164d95ba3c`。[7原件](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-21/archive-index-01.json)21,993,293→2,290,621 B经gzip/字节/SHA回验。screen20/19/18与启动FAIL原件不覆盖。该镜像600秒、32配置USB＋BLE各120秒、50切换、SHORT/弱链路仍NOT_RUN，Wi-Fi SDK隔离P1另行修复；最终测试还需首个mutation前ELF绑定和逐case窗口判定，再失败即停止。**总体未验收，暂不运行最终套件。**

Verdict, [unchanged-criteria supplement](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-21/screen-21-criteria-v2-supplement.json) and archive commands exit zero. [Seven originals](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-21/archive-index-01.json) compress21,993,293→2,290,621 bytes with verified gzip/byte/SHA roundtrips; prior failed records remain. Same-image600-second stability,32 configurations at120seconds each over USB+BLE,50 transitions,SHORT and weak-link tests remain NOT_RUN. Separate Wi-Fi SDK isolation and final-run pre-mutation image binding/per-case failure gating remain work items. **Overall acceptance is incomplete and the final suite has not started.**

<a id="screen20-working-copy-results"></a>

## 2026-11-21 screen20：内部发送工作区对照未达标 / Internal TX working-copy comparison remains below target

build05新增单个BLE TX owner内部 **2,304 B**工作区，从永久slot复制当前完整发送数据再分片；原slot继续持有直到真实发送返回，超时不代表取消或释放。队列槽数、CRC/ASCII/epoch验证、SDK deadline、采集率和输出策略不变；这是PSRAM访问成本的性能假设，不是已证明根因修复。[构建/73入口](docs/audit/artifacts/cdc_recovery_acceptance_20261116/ble-snapshot-05-validated-identity.json)全部exit0，195生产输入一致；app **1,617,920 B**，bin SHA256 `d85f932ce527b914d6b87c1a8cf5cb952133b684da76aeb1e970579ec9ba070f`，ELF SHA256 `7f2d5ad9c73887f70d49c7f8eb45ca889c4c44fcb1fedd293299200d9b39b469`，[完整bin/ELF/map归档](docs/audit/artifacts/cdc_recovery_acceptance_20261116/ble-snapshot-05-image-archive.json)回验。

Build05 adds one **2,304-byte internal working copy** owned by BLE TX, copying current payload from its permanent slot before fragmentation. The original slot remains owned until the real send returns; timeout proves neither cancellation nor safe release. Queue sizes, CRC/ASCII/epoch checks, SDK deadline, capture rate and output policy are unchanged. This tests a PSRAM-cost hypothesis without proving a root-cause repair. The [build and all 73 host entries](docs/audit/artifacts/cdc_recovery_acceptance_20261116/ble-snapshot-05-validated-identity.json) pass with 195 matching production inputs. App size is **1,617,920 bytes**; the complete bin/ELF hashes above and [archived bin/ELF/map](docs/audit/artifacts/cdc_recovery_acceptance_20261116/ble-snapshot-05-image-archive.json) identify this exact image.

[实际启动](docs/audit/artifacts/cdc_recovery_acceptance_20261116/ble-snapshot-05-boot-verification.json)为READY1/CAP8、session B73989CDD936FA54/boot1；SPI2MHz、2000读取CRC/status/reg0、I²C/SPI重叠701,047µs。内部BLE对象7,008 B包含工作区；MEM scan created/stack16,384 B，创建前free/largest37,864/25,588、后20,064/8,180 B，启动查询intFree/largest/min26,320/9,716/19,956 B。顺序读数不当独占分配差值，不以PSRAM总量替内部连续块。

[Actual boot](docs/audit/artifacts/cdc_recovery_acceptance_20261116/ble-snapshot-05-boot-verification.json) verifies READY1/CAP8 in session B73989CDD936FA54, boot 1. Startup SPI is 2 MHz with 2,000 reads and no CRC/status/register rejection; I²C/SPI overlap is 701,047 µs. The 7,008-byte internal BLE object includes the working copy. MEM records successful creation of the 16,384-byte scan stack: before free/largest 37,864/25,588 bytes, after 20,064/8,180, and startup-query internal free/largest/min 26,320/9,716/19,956. Sequential differences do not isolate one allocation, and PSRAM totals do not establish contiguous internal capacity.

三路collector及postprobe均闭合。[screen20判定](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-20/screen-20-verdict.json)仍为 **FAIL_BLE_THROUGHPUT / overall NOT_ESTABLISHED**，对应VOLT1 rev4（mgen3/mrid4、rgen2/rrid3、pgen3/prid4）：

All collectors and the postprobe close. The [screen20 verdict](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-20/screen-20-verdict.json) remains **FAIL_BLE_THROUGHPUT / overall NOT_ESTABLISHED**, scoped to VOLT1 revision 4 and the identities above:

| 维度 / Dimension | 实际结果 / Recorded result | 边界 / Scope |
|---|---|---|
| 完整时间 / Duration | **120.000081240 s** | formal summary的单调起止。 / Formal monotonic endpoints. |
| Capture / USB / BLE FPS | **63.796225 / 63.871027 / 62.188982** | 各自设备/Host时间分母，USB略高不表示多采样。 / Independent device/receipt denominators. |
| USB | **7,664**连续，seq1110–8773 | 同identity内部缺口0。 / Internally consecutive only. |
| BLE | **7,461**完整，seq1108–8770，缺**202** | **202/7,663=2.636043% >2.5%：FAIL**；不采用USB7,664作分母。 / BLE endpoints define the denominator. |
| 原包与有效性 / Bytes and validity | 7,459公共原包SHA相同；frame CRC/identity/wire错误0；USB/BLE invalid **483/456** | invalid保留，均为悬空S1D2 X12；unacquired/not-fresh0。 / Whole frames are not all-valid or precision acceptance. |
| ADC SPI CRC | **0/80,344** checked reads | 独立 **119.296122407 s**，even40,447＋odd39,897；engine384→8000，entry0/interval64，末crcRetries unknown。 / Separate health interval and denominator. |
| PC最大相邻完整间隔 / Maximum adjacent full interval | BLE **45.106313 ms / 7,460 pairs**；USB **59.996266 ms / 7,663 pairs** | 不含捕获边缘、不等于保护ticket ACK。 / Edges and protected-ticket acknowledgment excluded. |
| 收尾 / Recovery | **CAP8_QUERY_VERIFIED** rev6/boundary8806 | 同BUILD/session，BLEGUARD ready1/quarantined0/rejected0；无fault/reset/WDT文本仍非绝无事件证明。 / Observed closure is not universal recovery proof. |

[分层查询](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-20/screen-20-protection-01.json)约120.455–120.508秒：SRC周期+119；NET保护候选queued/bleAccepted+128、derived+9、retry+2、obsolete+10，expiry/bleExpired/wifiExpired/bleFailed/policySkip0；BLE保护queued/complete/staleNormal/deadlineMiss等增量均0。FLOWattempts+7,669/capacityReject+179、intentionalSkip/probe0；**before stage=pressure、rejectPm117，after stage=full、rejectPm46，两端ratePm1000**。pressure是当前本地证据窗状态，不意味着已经降输出；也不是RF弱信号分类。所有窗口单独保留，不从202中扣179或10。

[Separate query brackets](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-20/screen-20-protection-01.json) span roughly 120.455–120.508 seconds: source periodic +119; NET candidates/admissions +128, derived +9, retry +2 and obsolete +10, without expiry/failure/policy skips. BLE protected queued/completed/stale/deadline-miss counters do not increase. FLOW adds 7,669 attempts and 179 capacity rejections with zero intentional skips/probes. **The sampled state is pressure/rejectPm117 before and full/rejectPm46 afterward, both at ratePm1000.** Pressure is local evidence, not necessarily reduced output or weak-RF classification. Independent counters cannot be subtracted from 202 PC gaps.

USBQ独立120.547589182秒窗drop/expiry/partial增量0，dSubmit+7,697/cSubmit+115、dBytes4,605,826/cBytes27,107，callback bytes unknown。BLETX独立120.529826188秒窗sdkCalls+14,981/accepted+14,982/errors0；跨端点in-flight可使accepted多1，不代表虚构一次PC接收。BL50独立119.073934689秒窗dropD/L/C为171/304/0，不与202加总。

USBQ separately spans 120.547589182 seconds, with zero new drop/expiry/partial events, 7,697 data and 115 control submissions, and 4,605,826/27,107 submitted data/control bytes; callback bytes remain unknown. BLETX spans 120.529826188 seconds with 14,981 calls and 14,982 acceptances, zero errors. An in-flight call can cross an endpoint and explain one extra acceptance delta without implying PC receipt. BL50's separate 119.073934689-second dropD/L/C deltas are 171/304/0, not additive to 202.

[screen19→20描述比较](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-20/screen19-screen20-comparison.json)：BLE_TX自身CPU窗 **17.127876%→17.213336%**，没有改善证据；本地capacityReject228→179，但Capture64.392618→63.796225 FPS、BLE62.340491→62.188982 FPS也下降。不同session/RF/端点/负载不构成受控A/B，**更小丢弃数不能证明工作区有收益**。本轮其余owner CPU为Transport6.399688%、AsyncLog13.143679%、SerialTX10.374553%、SerialCTRL0.786524%，各分母独立不相加；总CPU/SDK/阻塞unknown。

The [descriptive screen19→20 comparison](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-20/screen19-screen20-comparison.json) shows BLE_TX's own CPU ratio **17.127876%→17.213336%**, without demonstrated improvement. Capacity rejections fall 228→179, but capture falls 64.392618→63.796225 FPS and BLE receipt 62.340491→62.188982 FPS. Distinct sessions, RF conditions, endpoints and loads are not a controlled A/B: **a smaller drop count does not establish a working-copy benefit**. Other independent owner ratios are Transport 6.399688%, AsyncLog 13.143679%, SerialTX 10.374553% and SerialCTRL 0.786524%; total/SDK CPU and blocked time remain unknown and ratios are not summed.

STK50 aggregate observed free/lifetime min为8,272,284/8,268,780 B；log/transport/bleTx/bleCtrl/serialCtrl最小剩余12,500/3,248/2,096/5,208/1,456 B。精确窗口internal largest未观测；[后续MEM](docs/audit/artifacts/cdc_recovery_acceptance_20261116/ble-snapshot-postscreen20-01.summary.json)intFree/largest/min23,868/8,180/19,956 B、extFree/largest8,248,968/8,126,452 B是独立顺序值。不能忽略增加内部工作区的内存代价。

Complete STK50 records give aggregate free/lifetime-minimum heap of 8,272,284/8,268,780 bytes and remaining-stack minima of 12,500/3,248/2,096/5,208/1,456 bytes. Exact capture-window internal-largest-block behavior is unobserved. A [later sequential MEM read](docs/audit/artifacts/cdc_recovery_acceptance_20261116/ble-snapshot-postscreen20-01.summary.json) reports internal free/largest/min 23,868/8,180/19,956 and external free/largest 8,248,968/8,126,452 bytes. The internal-memory cost of the working copy remains part of this comparison.

verdict命令exit0；criteria首轮沿用“两端full”假设而exit1，实录pressure→full均保持1000的情况修正后exit0；随后仅修正输出内的生成命令引用，新[criteria补充02](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-20/screen-20-criteria-v2-supplement-02.json)由第三命令exit0生成，数值判定不变，所有旧结果/失败/源码副本保留。同一criteria SHA `e0ef35a43a75932b8878aaa5c7cc94a6b562a9e4e1f86e50055ccd164d95ba3c` 未放宽。归档exit0，[7原件](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-20/archive-index-01.json)20,402,625→2,100,049 B经gzip/字节/SHA回验。build05的600秒、50切换、32配置整套、SHORT/USB不读/弱链路专项未运行，不能继承build04实测；主任务继续修复，不宣布最终验收完成。

Verdict generation exits zero. The first criteria run rejects an inherited assumption that both endpoints must say full; the real pressure→full transition remains at rate1000, so the corrected assessment exits zero. A subsequent provenance-only correction points the [current supplement02](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-20/screen-20-criteria-v2-supplement-02.json) at its actual third command; numeric assessment is unchanged and every earlier result/failure/source copy is retained. The criteria hash is unchanged. [Seven originals](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-20/archive-index-01.json) compress from 20,402,625 to 2,100,049 bytes with verified byte/gzip/SHA roundtrips. Build05's 600-second, 50-transition, complete 32-configuration, SHORT/unread/weak-link checks remain unrun; build04 observations are not inherited. Repair continues and final acceptance is not declared.

<a id="short-snapshot-results"></a>

## 2026-11-21 CAP8 SHORT专项：普通预览与完整周期快照 / Ordinary previews and periodic full snapshots

[short-snapshots-01评估](docs/audit/artifacts/cdc_recovery_acceptance_20261116/short-snapshots-01/short-snapshots-assessment-01.json)属于build04 `a6b0…` ELF、session B4828F8EF45001F1/boot1、CAP8 rev8（mgen5/mrid7、rgen4/rrid8、pgen5/prid7；旧CAP gen/rid仍为rows域4/8）。**120.000123756秒**内USB **1,416**完整可信帧、seq11989–13404内部连续，Capture/USB **11.795365/11.796284 FPS**；BLE **118**完整快照、seq11999–13403、**0.982995完整FPS**，PC完整K最大相邻间隔 **1.028126934秒/117对**。118个公共完整原包SHA相同，wire/ASCII CRC/identity错误0；USB90,624点与BLE7,552点均有效，但这不证明33pF绝对值、环境baseline或精度已校准。CAP模式没有本窗ADS健康读数，ADC CRC为NOT_OBSERVED，不填0错误率。

The [SHORT assessment](docs/audit/artifacts/cdc_recovery_acceptance_20261116/short-snapshots-01/short-snapshots-assessment-01.json) is scoped to build04's `a6b0…` ELF, session B4828F8EF45001F1, boot 1 and CAP8 revision 8. Legacy CAP gen/rid remains rows-owned, 4/8. Over **120.000123756 seconds**, USB receives **1,416 internally consecutive full frames** at **11.796284 FPS**, with inferred capture **11.795365 FPS**. BLE receives **118 full snapshots** at **0.982995 full FPS**; the maximum observed adjacent completing-K interval is **1.028126934 seconds over 117 pairs**. All 118 common original full packets match, with zero wire/ASCII-CRC/identity errors. All observed USB/BLE points are valid, but that does not establish absolute 33 pF accuracy, environmental baseline stability or calibration. ADC health is unobserved in this CAP window, not a measured zero CRC-error rate.

BLE有1,416个逻辑DATA message，普通CAP SHORT为预览，118个保护包保留全部8行与真实encoder CRC。解析器原样保留完整帧之间1,287个seq未观察数；**不能套用普通FULL的2.5%随机缺口预算**或把它们叫无线丢包，也不重写计数。1秒是到期后下一份真实完整帧的调度周期；1.028秒实测间隔不是严格1秒交付承诺，边缘等待、首次配置和弱信号另测。整体结论 **PERIODIC_FULL_SNAPSHOTS_OBSERVED_IN_SHORT_MODE / NOT_ESTABLISHED**，不把主VOLT1吞吐FAIL改为PASS。

BLE carries 1,416 logical DATA messages. Ordinary CAP SHORT messages are previews; 118 protected bundles retain all eight rows and the original encoder CRC. The parser preserves 1,287 unobserved sequence positions between full frames. **The normal FULL 2.5% gap budget does not apply to this preview stream**; those positions are not proven radio losses and no counter is rewritten. One second schedules the next real full candidate after due, rather than promising strict one-second delivery. Capture-edge waits, first-configuration delivery and weak signals require separate checks. The scoped result is **PERIODIC_FULL_SNAPSHOTS_OBSERVED_IN_SHORT_MODE / NOT_ESTABLISHED**, leaving the main VOLT1 throughput FAIL unchanged.

[四层保护查询](docs/audit/artifacts/cdc_recovery_acceptance_20261116/short-snapshots-01/short-snapshots-01-protection-03.json)各自约120.085–120.091秒：source周期/queued+118、NETqueued/bleAccepted+118、BLEqueued/localComplete+118，所有本窗admitMiss/failed/deadlineMiss/staleNormal/expiry增量0；FLOWattempts+1,416、capacityReject/intentionalSkip/probe0。118的数值相同不把本地计数变成PC ACK。末端BLESNAP.maxCompleteGapUs5,929,446为本地历史最大gap，不能覆盖/替代本次PC1.028126934秒间隔。

The [four independent protection-query brackets](docs/audit/artifacts/cdc_recovery_acceptance_20261116/short-snapshots-01/short-snapshots-01-protection-03.json) each span roughly 120.085–120.091 seconds. Source periodic/queued, NET queued/BLE admission and BLE protected queued/local completion each increase 118. Admission misses, failure, deadline misses, stale-normal and expiry deltas are zero; FLOW attempts increase 1,416 with no capacity rejections, intentional skips or probes. Equal local and PC counts do not create a PC ACK. Final BLESNAP maxCompleteGapUs 5,929,446 is a local historical maximum, distinct from this window's observed PC maximum of 1.028126934 seconds.

历史采集器v1在`TX=REL`恢复前记录base `finished`，尾部实际仍有TX=REL→ACK→TX?→ACK且没有第二个finished。原保护分析据此exit1，失败日志保留。[后续严格兼容分析](docs/audit/artifacts/cdc_recovery_acceptance_20261116/short-snapshots-protection-legacy-closure-02.json)绑定原v1源码SHA545d0ef0…，核验全部raw哈希/RX连续offset、恢复每个ACK来自对应command之后的新读入范围、故障/复位总数/首错/总deadline，输出`legacy_SHORT_restore_tail_verified`且明确`explicit_final_finished_event=false`。CAP8 rev10/boundary13408是在TX恢复前查询；恢复本身QUERY_VERIFIED，**没有补造恢复后的CONFIG或finished**。原summary/raw/events哈希前后一致。新版v2采集器只在全部恢复收尾后发最终finished；16组实际host回归包括v2成功、late MFAULT、第17次RST、restore ACK丢失，失败路径仍ERROR。兼容分析不是新烧录或重录。

Historical collector v1 emits a base `finished` before restoring TX. The real tail contains TX=REL → ACK → TX? → ACK, without a second finished event. Initial protection analysis therefore exits 1 and its log remains. The [strict compatibility analysis](docs/audit/artifacts/cdc_recovery_acceptance_20261116/short-snapshots-protection-legacy-closure-02.json) binds the exact archived v1 source, validates raw hashes/contiguous RX offsets, binds each restore ACK to bytes newly read after its command, and checks fault/reset totals, first error and total deadline. It reports `legacy_SHORT_restore_tail_verified` with `explicit_final_finished_event=false`. CAP8 revision 10/boundary 13408 was queried before TX restoration; restoration is QUERY_VERIFIED. **No post-restore CONFIG or finished event is invented.** Original summary/raw/event hashes are unchanged. Collector v2 emits its final finished only after restoration. Sixteen executed host regressions cover successful v2 restoration, late MFAULT, the 17th reset and missing restore ACK, preserving ERROR on failures. This compatibility analysis is neither a new flash nor a rerecording.

CPU各自owner窗为BLE_TX0.987951%、Transport2.389888%、AsyncLog2.264629%、SerialTX4.132637%、SerialCTRL0.662628%，不相加；总CPU/SDK/阻塞unknown。完整STK50 aggregate free/min8,274,764/8,270,568 B，log/transport/bleTx/bleCtrl/serialCtrl最小剩余12,500/2,896/2,080/5,208/1,452 B。无fault/reset文本，不证明所有事件绝无。评估首次命令因CPU identity含额外state/profile字段而严格字典比较失败；改为核对同九个配置域后第二命令exit0，原失败日志/源码副本保留。归档命令exit0：[5原件](docs/audit/artifacts/cdc_recovery_acceptance_20261116/short-snapshots-01/archive-index-01.json)4,421,846→562,828 B经gzip/字节/SHA回验。此前screen19补充的SHORT NOT_RUN为当时证据状态，本专项补充其覆盖范围，不改原件或整体判定。

Per-owner CPU brackets are BLE_TX 0.987951%, Transport 2.389888%, AsyncLog 2.264629%, SerialTX 4.132637% and SerialCTRL 0.662628%; they are not summed. Total CPU, SDK CPU and blocked time remain unknown. Complete STK50 records retain aggregate heap and stack minima above. No fault/reset text is observed, without claiming universal absence. The first assessment command rejects a whole-dictionary comparison because CPU identity includes extra state/profile fields; matching the same nine identity domains resolves it on the second execution, preserving the first log/source. Archiving exits zero: [five retained originals](docs/audit/artifacts/cdc_recovery_acceptance_20261116/short-snapshots-01/archive-index-01.json) compress from 4,421,846 to 562,828 bytes with gzip/byte/SHA verification. The earlier screen19 supplement's SHORT NOT_RUN described its then-available evidence; this separate run adds coverage without rewriting that artifact or overall verdict.

<a id="usb-unread-results"></a>

## 2026-11-21 USB应用不读专项：BLE继续，USB排队有界丢弃 / Unread USB client: BLE continues while USB queues discard data

[usb-unread-01闭合分析](docs/audit/artifacts/cdc_recovery_acceptance_20261116/usb-unread-01/usb-unread-analysis-02.json)对应同一build04 ELF `a6b04a2a68ddcb5c8c39cc28020abbe5db430c1e140cbb3afb4ddb703fdb217a`、session `B4828F8EF45001F1` / boot1，VOLT1 rev12/boundary14532（mgen7/mrid12、rgen6/rrid11、pgen7/prid12）。正式CDC0持有一个已打开FD，单调时间 **134342.703814096–134462.704181391秒**内 **120.000367295秒零应用read/write**，raw offset保持18,158；没有USBSTREAM/FPS限速、NVS写入、复位或拔线。CDC debug和BLE分别独立录制。系统可继续缓冲USB：这不是总线零流量、物理断线、无主机或Windows重新枚举测试；独占reader状态也不由设备日志证明。

The [closed unread-client analysis](docs/audit/artifacts/cdc_recovery_acceptance_20261116/usb-unread-01/usb-unread-analysis-02.json) belongs to the same build04 ELF/session, boot 1 and VOLT1 revision 12/boundary 14532 with the identities above. One formal CDC0 FD remains open, with **zero application reads/writes for 120.000367295 seconds**, over monotonic seconds **134342.703814096–134462.704181391**. Its raw offset stays 18,158. No USBSTREAM/FPS limit, NVS write, reset or unplug occurs; independent CDC-debug and BLE observers continue. Linux can still buffer USB traffic. This is not zero bus traffic, a physical disconnect, absent host or Windows re-enumeration test, and device logs do not prove reader exclusivity.

| 测量 / Measurement | 结果 / Result | 限制 / Limit |
|---|---|---|
| BLE完整帧 / Complete BLE frames | **7,821**可信帧，seq14536–22381，缺25 | **25/7,846=0.318634%**，该专项数值低于2.5%；不能覆盖screen19的FAIL。 / This stress-window ratio meets the numeric budget only; screen19 remains failed. |
| Capture / BLE PC实收 / Receipt | **65.391754 / 65.177392 FPS** | Capture由BLE可信seq进展/设备时间推算；不是独立ADC转换计数。 / Capture is inferred from trusted BLE sequence/device-time progression. |
| USB PC实收 / Receipt | **NOT_OBSERVED，FPS unknown** | 后续排空缓存不计入hold；不是测得0 FPS。 / Later drain is excluded; this is not a measured zero rate. |
| 完整间隔 / Complete-frame interval | BLE最大 **36.236946 ms**，7,820相邻对 | 同identity已观察K之间，不含窗口边缘，不保证每个保护ticket。 / Adjacent observed completing-K receipts only. |
| 原始协议 / Original protocol | 15,642通知→7,821完整D message；G CRC/ASCII CRC/identity/wire错误 **0** | 使用真实重组与科学帧解析器，原字节不改。 / Existing real reassembler/parser, without rewriting bytes. |
| 点有效性 / Point validity | **351个X12，均S1D2**；unacquired/not-fresh **0** | 完整仍包含无效点，未做绝对精度验收。 / Whole frames retain invalid points; precision is unaccepted. |
| ADC SPI CRC | **0/82,372** checked reads | 独立 **118.501447312 s**健康窗；even41,448＋odd40,924，不能用完整帧数作分母。 / Separate health-counter interval. |
| 收尾 / Recovery | **CAP8_QUERY_VERIFIED**，rev14/boundary22403 | 后续BUILD/session一致，BLE guard无隔离；没有测试hold期间主动控制命令的往返。 / Same-image recovery is verified; active command roundtrips during hold were not tested. |

ADC健康记录的engineFrameCount从8192到15936、**modeEntryFrameCount=8131**、healthInterval64，说明本次进入VOLT时engine计数已经累计，不把模式区间当从零开始。健康末字段crcRetries仍因缺少后续marker而unknown。独立debug的全部raw SHA/receipt offset经核对，观察覆盖hold；未见fault/reset/WDT文本，但缺日志不证明无事件。

ADS health records advance engineFrameCount from 8192 to 15936 with **modeEntryFrameCount=8131** and healthInterval 64: engine counting already existed before this VOLT interval. It is not assumed to start at zero. The terminal crcRetries field remains unknown without a following marker. Independent debug raw hashes and every receipt offset are verified with observer coverage across the hold. No fault/reset/WDT text is observed, which does not establish universal absence.

USBQ的独立查询括号约120.184655–120.201211秒：`dDrop +5586、expired +2240、partial +1`；`cDrop/cExpired +0`；`dSubmit +29/cSubmit +14、dBytes +18093/cBytes +3354`。这些计数包含hold前后查询的时间，不是hold PC实收；partial与expiry可能重叠，不加总成科学帧损失。USB两端mounted1/suspended0/dma1/epoch2，reset/mount/unmount增量0；这只描述所查询的驱动状态，不能证明无主机或PC收件。恢复读取后的查询完成证据保留，不能据此断言无人读时仍能向PC送达控制回复。

USBQ has its own approximately 120.184655–120.201211-second query bracket: **dDrop +5,586, expired +2,240 and partial +1**, zero new control drops/expiry, 29 data and 14 control submissions, and 18,093/3,354 submitted data/control bytes. These brackets include activity before and after the hold; they are not hold-time PC receipt. Partial and expiry can overlap and cannot be added as scientific-frame losses. USB endpoints report mounted=1, suspended=0, dma=1 and epoch=2, with no additional reset/mount/unmount. That is queried driver state, not proof of absent host or PC receipt. Later query completion does not prove control delivery while the client does not read.

BLEFLOW独立约120.198–120.240秒窗attempts+7,851/capacityReject+19、intentionalSkip/probe0、两端FULL1000‰；BLESNAP约120.209–120.246秒窗全部保护计数增量0，lastSeq14532/age120,260,860µs仍是protected-only。BL50另一个119.303113207秒窗dropD/L/C为19/312/0。它们不是25个PC缺口的可加减组成，更不等于无线丢包。与screen19相比BLE缺口下降只是不同负载/时间窗的描述，不能直接认定USB hot path是唯一根因或将主测试改判通过。

BLEFLOW's separate roughly 120.198–120.240-second bracket records 7,851 attempts, 19 capacity rejections, no intentional skips/probes and sampled FULL 1000‰. BLESNAP adds no protected counters over its own bracket; lastSeq 14532 and age 120,260,860 µs remain protected-only. BL50 separately observes dropD/L/C of 19/312/0 over 119.303113207 seconds. These are neither additive components of the 25 PC gaps nor radio-loss measurements. Better BLE receipt than screen19 is a descriptive difference in load/time, not proof that USB's hot path is the sole cause or grounds to reverse the main FAIL.

完整STK50的aggregate heap observed free/lifetime min为8,274,408/8,270,568 B；最小剩余栈log/transport/bleTx/bleCtrl/serialCtrl为12,500/2,896/2,080/5,208/1,452 B。hold期间CPU和内部最大连续块未测；[后续MEM](docs/audit/artifacts/cdc_recovery_acceptance_20261116/ble-snapshot-postunread-01.summary.json)intFree/largest/min26,016/13,812/21,744 B，extFree/largest8,248,968/8,126,452 B，属于独立顺序观察。

Complete STK50 records give aggregate free/lifetime-minimum heap of 8,274,408/8,270,568 bytes and minimum remaining log/transport/bleTx/bleCtrl/serialCtrl stacks of 12,500/2,896/2,080/5,208/1,452 bytes. Hold-time CPU and internal largest-block behavior are unmeasured. A [later sequential MEM read](docs/audit/artifacts/cdc_recovery_acceptance_20261116/ble-snapshot-postunread-01.summary.json) reports internal free/largest/min 26,016/13,812/21,744 and external free/largest 8,248,968/8,126,452 bytes, not hold-time worst cases.

新离线[analyze_usb_unread.py](docs/audit/artifacts/cdc_recovery_acceptance_20261116/scripts/analyze_usb_unread.py)仅把实录hold起止/CONFIG作为内存窗口传给既有`analyse_ble_journal → FragmentReassembler/TextProtocolParser/Group`；**没有伪造capture summary或USB科学帧**。初版5组测试通过，但[独立反例审查01](docs/audit/artifacts/cdc_recovery_acceptance_20261116/usb-unread-independent-review-01.json)发现其未严格绑定最终finished/cleanup、实际`tx_begin/tx_complete`事件，以及postprobe原始收件证据；原analysis01和源码副本保留，不能仅凭其summary作收尾保证。修复后的[10组实际回归](docs/audit/artifacts/cdc_recovery_acceptance_20261116/usb-unread-analysis-host-03-command.json)与[独立复审02](docs/audit/artifacts/cdc_recovery_acceptance_20261116/usb-unread-independent-review-02.json)均exit0，原负向反例现在全部拒绝；新analysis02重算数值完全不变。测试仍覆盖边缘分片、完成时刻、内外CRC、identity/invalid和未知计数，不用更宽容解析取得更好结果。

The new offline [analyzer](docs/audit/artifacts/cdc_recovery_acceptance_20261116/scripts/analyze_usb_unread.py) passes only recorded hold bounds and CONFIG into the existing BLE reassembler/scientific-parser window interface. **No capture summary or USB scientific frame is fabricated.** Five initial tests pass, but [independent counterexample review01](docs/audit/artifacts/cdc_recovery_acceptance_20261116/usb-unread-independent-review-01.json) identifies missing final-finished/cleanup binding, actual tx_begin/tx_complete recognition and original postprobe evidence binding. Analysis01 and its original source copies remain; its summary alone is insufficient for closure assurance. The corrected [ten executed regressions](docs/audit/artifacts/cdc_recovery_acceptance_20261116/usb-unread-analysis-host-03-command.json) and [independent review02](docs/audit/artifacts/cdc_recovery_acceptance_20261116/usb-unread-independent-review-02.json) exit zero and reject all original negative counterexamples. Analysis02 reproduces identical measured numbers. Edge fragments, completion time, inner/outer CRC, identity/invalid markers and unknown counters remain covered; no parser relaxation improves the result.

当前源码SHA `3709c2caee483e1745130521f817370a6e30775b43352873174e1bde782c872f`要求hold原始raw哈希、连续offset、唯一最终成功finished及cleanup无fault/reset；禁止hold中真实RX/TX事件。每条query回复必须由对应写完成后新读入的原字节支持。postprobe原始raw/events和summary逐项绑定26条回复、全部开始于hold之后；该只读采集器原本没有最终finished，明确`explicit_final_finished_event=false`，不补造。原输入哈希前后一致。独立复审首轮仅因内存结构与JSON表示差异退出，保留原脚本/日志；规范化后实际重跑通过，未改生产或实录。

Current source SHA `3709c2caee483e1745130521f817370a6e30775b43352873174e1bde782c872f` requires the original hold raw hash, contiguous offsets, one successful final finished event and fault/reset-free cleanup; actual RX/TX events during hold are forbidden. Each query reply must be supported by original bytes newly read after its corresponding completed write. Original postprobe raw/events and summary bind all 26 replies, with every query beginning after the hold. This read-only collector never emits a final finished event, so `explicit_final_finished_event=false` remains explicit and none is invented. Input hashes remain identical. The first independent-review harness run rejects a live-versus-JSON representation mismatch; its script/log remain, and normalization allows the actual rerun to pass without firmware or recording changes.

[archive02](docs/audit/artifacts/cdc_recovery_acceptance_20261116/usb-unread-01/archive-index-02.json)再次回验7原件12,171,808→1,229,815 B，保留archive01、两版分析/旧源码及反例；归档命令exit0。结论仍只是在该条件下采集/BLE继续及后续恢复可查询，`overall_acceptance=NOT_ESTABLISHED`；不把离线证据绑定修复变成新HIL、物理断线、Windows枚举、长期或整体性能通过。

[Archive02](docs/audit/artifacts/cdc_recovery_acceptance_20261116/usb-unread-01/archive-index-02.json) revalidates seven originals, 12,171,808→1,229,815 bytes, retaining archive01, both analyses, old sources and counterexamples. Archiving exits zero. The conclusion remains continued acquisition/BLE and queryable later recovery under this condition, with `overall_acceptance=NOT_ESTABLISHED`. Repaired offline evidence binding is not new HIL or physical-disconnect, Windows-enumeration, long-run or overall-performance acceptance.

<a id="screen19-coalescing-results"></a>

## 2026-11-21 screen19：周期合并已生效，容量拒绝仍阻塞验收 / Coalescing works; admission pressure still blocks acceptance

三路collector及收尾只读查询均已退出，原始录制不改。[本轮判定](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-19/screen-19-verdict.json)为 **FAIL_BLE_THROUGHPUT**，仅属于 `ble-snapshot-04` 的 `a6b0…` ELF、session `B4828F8EF45001F1` / boot1、VOLT1 rev4、mgen3/mrid4、rgen2/rrid3、pgen3/prid4。此前screen18和575c启动失败继续保留，不用本轮局部改善替换。所有数字从闭合[analysis](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-19/screen-19-analysis-01.json)、[CPU](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-19/screen-19-cpu-01.json)及[分层保护分析](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-19/screen-19-protection-01.json)取得。

All three collectors and the later read-only probe have exited. The [verdict](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-19/screen-19-verdict.json) is **FAIL_BLE_THROUGHPUT**, scoped to build04's `a6b0…` ELF, session `B4828F8EF45001F1`, boot 1 and VOLT1 revision 4 with the mode/rows/profile identities above. Earlier screen18 and 575c startup failures remain unchanged. The linked closed analysis, CPU and protection records supply the following observations.

| 维度 / Dimension | 实际观察 / Observation | 证据边界 / Scope |
|---|---|---|
| 完整捕获 / Capture window | **120.000139309 s** | formal summary的起止单调时钟；已完成时长，不代表整体通过。 / Completed duration is not acceptance. |
| Capture / USB / BLE | **64.392618 / 64.399646 / 62.340491 FPS** | Capture用设备时间与seq，实收用各自首末可信K接收时间；不同分母使USB略高不表示额外采样。 / Separate device/receipt denominators explain the small USB excess. |
| USB完整可信帧 / Trusted full frames | **7,728，seq976–8703，内部缺口0** | 仅该identity内连续，不跨boot比较。 / Consecutive within this identity only. |
| BLE完整可信帧 / Trusted full frames | **7,481，seq973–8703，缺250** | **250/7,731=3.233734% > 2.5%：FAIL**；分母不是USB7,728。 / The BLE endpoint denominator differs from USB's count. |
| 原包一致性 / Original bytes | **7,479**公共完整原包SHA相同；wire/ASCII CRC/identity错误0 | USB未配对249、BLE未配对2是不同端点/匹配域，不能另加到250。 / Unmatched records are not additional independent losses. |
| 科学有效性 / Point validity | USB **502**、BLE **471** 个X12，USB定位悬空S1D2；unacquired/not-fresh 0 | 完整帧保留invalid标记，不等于所有点有效或精度通过。 / Complete frames retain invalid points; absolute precision is untested. |
| ADC SPI CRC | **0 / 82,589** checked reads | 另一个 **119.264441785 s**健康窗；even41,421＋odd41,168，engine384→8064、healthInterval64。 / A separate read-counter interval, not scientific-frame count. |
| PC相邻完整帧最大间隔 / Maximum adjacent full-frame interval | BLE **42.942640 ms / 7,480 pairs**；USB **70.305342 ms / 7,727 pairs** | 同identity已观察完整K之间；不含capture边缘等待，不证明每个保护ticket、首配置或弱链路必达。 / Observed adjacent receipts exclude edge waits and do not prove protected-ticket delivery. |
| 收尾恢复 / Closing recovery | **CAP8_QUERY_VERIFIED**，rev6/boundary8741 | 后续只读BUILD仍a6b0…，BLEGUARD ready1/quarantined0/rejected0/completionKnown1；不等于所有故障路径已测。 / Query-verified state is not universal recovery coverage. |

没有观察到fault/reset/WDT文本；这不证明未观察范围内绝无这些事件。ADC健康窗来自完整CDC诊断，末尾无后续marker的`crcRetries`仍unknown，不能由`crcErrors=0`伪造重试数。未写校准、未删无效点，也未把悬空口解释成有效已知输入。

No fault/reset/WDT text is observed, which does not prove absence outside the recorded evidence. ADC statistics come from complete CDC diagnostics. The terminal `crcRetries` token lacks a following marker and remains unknown; zero CRC errors cannot fabricate a retry count. No calibration is written, invalid points are retained, and floating ports are not treated as known valid inputs.

四个查询各有独立Host括号区间，约120.50–120.54秒，**不与120.000139309秒捕获边界原子对齐**：

The four queries have independent host brackets of roughly 120.50–120.54 seconds, **not atomic alignment with the capture endpoints**:

| 层 / Layer | 同一查询前后增量 / Within-query deltas | 解释 / Meaning |
|---|---|---|
| SNAPSRC | queued/periodic **+119**；first/miss/superseded **0** | source仍保留真实周期候选。 / Real periodic source candidates remain. |
| SNAPNET | queued/bleAccepted **+124**，derived **+5**，retry **+3**，obsolete **+8**；expiry/bleExpired/wifiExpired/bleFailed/policySkip **0** | 接纳的是Transport保护候选，不保证最终BLE class；旧普通帧淘汰仍单独计数。 / Candidate admission does not require protected BLE classification. |
| BLESNAP | queued/localComplete/staleNormal/deadlineMiss/admitMiss/failed/superseded **全部+0** | FULL普通完整帧已满足内部周期依据；没有重复保护超越。 / Ordinary FULL satisfies the internal periodic evidence without new protected overtaking. |
| BLEFLOW | attempts **+7,738**，capacityReject **+228**，intentionalSkip/probe **0**；两端full、ratePm1000 | 本地容量拒绝仍存在；不是随机2.5%丢弃，也不是已测RF丢包。 / Local admission pressure remains, without intentional decimation or measured RF loss. |

这是周期合并机制在本次FULL运行中的实际证据，但**正常缺帧预算仍失败**。`BLESNAP`末端仍为lastSeq609、localComplete4、protected age125,788,271µs、gapKnown0；这些保护分类历史字段不代表125秒没有完整数据。真正PC完整帧最大相邻间隔为上表42.942640ms。capacityReject228、NETobsolete8、PC缺250的时间与含义不同，不能凑等式、抵扣或断言全部丢失来自某一层；RSSI/空中丢包仍unknown。首配置、SHORT/弱链路和连续CTRL压力下的保护验收仍需另外运行。

This is hardware evidence that coalescing operates during this FULL run, while **normal gap acceptance still fails**. Final BLESNAP retains lastSeq 609, localComplete 4, protected age 125,788,271 µs and gapKnown 0. These historical protected-class fields do not imply 125 seconds without full data: the PC's maximum observed adjacent complete interval is 42.942640 ms. Capacity rejection 228, NET obsolete 8 and PC missing 250 have different scopes and cannot be added, subtracted or assigned a single proven cause. RSSI/radio loss remain unknown. First-configuration, SHORT, weak-link and continuous-CTRL protection require separate acceptance.

BL50独立119.586313004秒窗的dropD/L/C为+224/+308/+0。USBQ独立120.500439367秒窗的DATA/CTRL drop、expiry、partial增量0，dSubmit7,761/cSubmit115、dBytes4,643,164/cBytes27,089；callback bytes仍unknown。BLETX独立120.507435010秒窗为sdkCalls15,015/accepted15,014/errors0；1次差异可为跨端点in-flight，不当PC丢包。BLESVC独立查询完成15,019次、wall55,761,859µs，平均3,712.754µs/call是墙钟占用，不是CPU或阻塞时间。这些数不与250相加。

BL50's separate 119.586313004-second span has dropD/L/C deltas of 224/308/0. USBQ's 120.500439367-second span reports zero new DATA/CTRL drops, expiry or partial writes, with 7,761 data and 115 control submissions; 4,643,164 data bytes and 27,089 control bytes. Callback-byte counts remain unknown. BLETX's 120.507435010-second span has 15,015 SDK calls, 15,014 acceptances and no errors; one in-flight call can straddle endpoints, so the one-count difference is not PC loss. BLESVC independently observes 15,019 completions over 55,761,859 µs total call wall time, averaging 3,712.754 µs per call. Wall time is neither CPU nor blocked time. These observations are not additive to 250.

[CPU自身观察窗](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-19/screen-19-cpu-01.json)比例为BLE_TX17.127876%、Transport6.489029%、AsyncLog13.232456%、SerialTX10.352611%、SerialCTRL0.807136%；BLE_CTRL/WIFI_CTRL、全系统CPU、SDK CPU和阻塞时间unknown，不能相加。STK50完整记录的aggregate heap observed free/lifetime minimum为8,273,700/8,270,568 B，栈最低剩余log/transport/bleTx/bleCtrl/serialCtrl为12,500/3,248/2,080/5,208/1,644 B。精确capture窗internal largest未观测；[后续MEM](docs/audit/artifacts/cdc_recovery_acceptance_20261116/ble-snapshot-postscreen19-01.summary.json)顺序读数intFree/largest/min为26,024/13,812/21,744 B、extFree/largest8,248,968/8,126,452 B，不能反填为捕获最坏值。

[Per-task CPU brackets](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-19/screen-19-cpu-01.json) give BLE_TX 17.127876%, Transport 6.489029%, AsyncLog 13.232456%, SerialTX 10.352611% and SerialCTRL 0.807136%. BLE_CTRL/WIFI_CTRL, whole-system CPU, SDK CPU and blocked time remain unknown; ratios cannot be summed. Complete STK50 records report aggregate free/lifetime-minimum heap of 8,273,700/8,270,568 bytes and minimum remaining stacks of 12,500/3,248/2,080/5,208/1,644 bytes for log/transport/bleTx/bleCtrl/serialCtrl. Exact capture-window internal-largest-block behavior is unobserved. A [later sequential MEM read](docs/audit/artifacts/cdc_recovery_acceptance_20261116/ble-snapshot-postscreen19-01.summary.json) gives internal free/largest/min 26,024/13,812/21,744 and external free/largest 8,248,968/8,126,452 bytes; these are not capture-window worst cases.

[screen18→19比较](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-19/screen18-screen19-comparison.json)仅描述不同ELF/session中的独立窗口，不跨boot减累计计数或声称RF条件相同。[verdict、criteria-v2补充](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-19/screen-19-criteria-v2-supplement.json)与归档三个实际命令均exit0；criteria源SHA保持 `e0ef35a43a75932b8878aaa5c7cc94a6b562a9e4e1f86e50055ccd164d95ba3c`，overall **NOT_ESTABLISHED**。[7份关闭raw/event/query文件](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-19/archive-index-01.json)24,392,868 B压缩到2,604,564 B，gzip解压/字节/SHA回验，原件全部保留。600秒、50次切换、32配置整套、SHORT/弱链路与绝对精度仍NOT_RUN/未验收；先修当前容量/调度问题再扩展测试。

The [screen18→19 comparison](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-19/screen18-screen19-comparison.json) is descriptive across distinct images/sessions, without subtracting cumulative counters across boots or asserting identical RF conditions. Verdict, [criteria supplement](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-19/screen-19-criteria-v2-supplement.json) and archive commands all exit zero. The criteria hash is unchanged and overall acceptance is **NOT_ESTABLISHED**. [Seven closed raw/event/query files](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-19/archive-index-01.json) compress from 24,392,868 to 2,604,564 bytes, with decompressed-byte/SHA verification and every original retained. The 600-second, 50-transition, complete 32-configuration, SHORT/weak-link and absolute-precision checks remain unrun or unaccepted. Repair current admission/scheduling behavior before extending the matrix.

<a id="ble-periodic-coalescing"></a>

## build04：完整帧完成合并周期义务 / Full-frame completion coalesces periodic obligations

screen18的119/129/239计数差异随后已用旧真实BLE源码＋合成RTOS/SDK时序[复现缺陷](docs/audit/artifacts/cdc_recovery_acceptance_20261116/ble-periodic-coalesce-before-01.json)：20FPS、10秒、200个FULL候选及错相上游周期请求，旧实现产生20个保护包、19个stale丢弃、181个完整本地完成；原失败exit 1及旧源码gzip保留。它证明此调度缺陷存在，不证明screen18的228个缺帧全部由该机制造成，原3.015075% FAIL不改。

The 119/129/239 counter difference is followed by an [actual-old-source reproduction](docs/audit/artifacts/cdc_recovery_acceptance_20261116/ble-periodic-coalesce-before-01.json) using synthetic RTOS/SDK timing. With 200 FULL candidates at 20 FPS for 10 seconds and phase-offset upstream periodic requests, the old implementation produces 20 protected packets, 19 stale drops and 181 whole local completions. Original exit 1 and compressed old source remain. This proves the scheduling defect, not attribution of every one of screen18's 228 missing observations; its 3.015075% FAIL remains unchanged.

本轮仅BLE新增内部“最近完整帧完成”依据 `sensorarrayBleFrameFullCompletedLocked`：必须真实typed完整原包、本地整包发送成功、当前connection/stream、同session/boot/config revision、sequence相对上次成功完整帧前进且完成时间不倒退，才将周期due更新为完成时刻＋1秒。**普通FULL也满足周期完整数据义务**；SHORT预览、raw旧接口、失败/部分发送、旧stream/config或重复/旧序号不能满足。比较对象是上次成功full的seq，不能用最新尚未完成候选的seq挡住排队中的正常进展。配置/有效stream变化清空该依据，first-config仍独立保留且优先。

Only BLE changes. New internal `sensorarrayBleFrameFullCompletedLocked` accepts a successful typed whole-full-packet local completion in the current connection/stream and same session/boot/configuration, with sequence advancement from the previous successful full packet and a nonregressing completion clock. It sets periodic due to completion+one second. **Ordinary FULL traffic also satisfies the periodic full-data obligation.** SHORT previews, legacy raw calls, failed/partial sends, stale streams/configurations and duplicate/old sequences do not. Comparison is against the last successful full sequence, not the newest offered-but-incomplete candidate. Configuration/stream changes clear this evidence; first-configuration protection remains independent and prioritized.

上游PERIODIC是source/Transport的保护候选标记，不再强制产生另一套BLE周期。已满足当前周期时，该候选按正常FULL/SHORT规则继续处理，合并本身不丢弃所提供的帧；无flag的真实typed帧在full已逾期时也能触发BLE补充保护。已接受的WRITING/PENDING/OWNED对象不因此取消或回收，仍保留原字节/identity并等待真实SDK完成。暂停/无帧时不复制旧样本；1秒仍是收到真实候选时检查的义务，不是PC交付保证。

An upstream PERIODIC flag reserves passage through source/Transport; it no longer forces an independent BLE cadence. If the obligation is already satisfied, the offered frame continues through ordinary FULL/SHORT handling rather than being discarded by coalescing. An unflagged real typed frame can trigger fallback protection after full completion becomes overdue. Accepted WRITING/PENDING/OWNED objects are not canceled or reclaimed: they retain original bytes/identity and actual SDK-return ownership. Paused/no-frame periods fabricate no old sample. The one-second obligation is checked on real input, not a PC delivery guarantee.

[修后独立回归](docs/audit/artifacts/cdc_recovery_acceptance_20261116/ble-periodic-coalesce-independent-review-01.json)在上游相位0.1/0.5/0.9秒及无上游候选的同类输入下：FULL为**1个first保护、0stale、200完整本地完成**；SHORT为**10个完整保护、9stale、10完整本地完成**，其余普通preview不计完整义务。ASan/UBSan的18组真实BLE测试、7组anchor边界、39个真实C通知接收roundtrip和22个pipeline场景通过；包括失订阅/旧config迟返、回绕/半范围、时钟倒退/饱和、failed/raw/preview排除、已pending owner保留。合成时序不是无线或HIL性能证明；SHORT仍允许显式计数的普通预览被保护包超越。

[Independent corrected-source checks](docs/audit/artifacts/cdc_recovery_acceptance_20261116/ble-periodic-coalesce-independent-review-01.json) repeat phases 0.1/0.5/0.9 seconds and missing upstream candidates: FULL yields **one first protected packet, zero stale drops and 200 whole local completions**; SHORT yields **ten protected full packets, nine stale drops and ten whole completions**. Ordinary previews do not satisfy full obligations. ASan/UBSan passes 18 actual BLE groups, seven anchor-boundary groups, 39 actual C notification receiver roundtrips and 22 pipeline scenarios, covering stale epochs/configurations, wrap/ambiguity, clock regression/saturation, failed/raw/preview exclusions and retained pending owners. This synthetic timing is not RF/HIL performance proof; SHORT may still incur counted preview overtaking.

**查询语义没有改成“全部完整帧”。** `BLESNAP.queued/localComplete/lastSeq/lastRev/lastCompleteAgeUs/maxCompleteGapUs`继续只描述BLE保护分类。在持续普通FULL成功时，可能只有最初first被保护，所以protected age增长、gap未知或很大并不等于缺少完整数据；Host用真实CRC/identity完整帧到达判断PC间隔，不能用保护age代替。新all-full依据仅内部使用，未新增wire字段或重解释现有字段。`SNAPNET.bleAccepted`是**Transport保护候选获BLE准入**；BLE最终可归普通，不等于BLESNAP queued。NET原候选fence及obsolete计数仍在，旧普通帧可能按原规则被明确丢弃，不能宣称本补丁清除了所有来源的缺帧。

**Queries retain protected-only meanings.** BLESNAP queued/localComplete/lastSeq/lastRev/lastCompleteAgeUs/maxCompleteGapUs still describe BLE's protected class. Healthy ordinary FULL streaming may need only the initial first protected packet: growing protected age or unknown/large protected gaps do not imply missing full data. Hosts measure PC intervals from actual complete CRC/identity-valid frames. The all-full anchor is private; no wire field is added or silently redefined. `SNAPNET.bleAccepted` means **a Transport protected candidate was admitted by BLE**, possibly into ordinary handling; it is not BLESNAP queued. Transport's existing candidate fence and obsolete counters remain, so this patch does not promise elimination of every upstream frame gap.

`ble-snapshot-04` [构建/73入口完整一轮](docs/audit/artifacts/cdc_recovery_acceptance_20261116/ble-snapshot-04-validated-identity.json)全部exit0，195输入构建/测试前后一致；app1,615,664 B，binary SHA256 `4903ce004daf970bac3e5b80e24bdffd452571cbd91f34cd1223357532afc484`，ELF SHA256 `a6b04a2a68ddcb5c8c39cc28020abbe5db430c1e140cbb3afb4ddb703fdb217a`，完整bin/ELF/map已[归档回验](docs/audit/artifacts/cdc_recovery_acceptance_20261116/ble-snapshot-04-image-archive.json)。已实际flash，[state03](docs/audit/artifacts/cdc_recovery_acceptance_20261116/ble-snapshot-state-03.summary.json)首次READY0/boot_sweep是正常启动未就绪，没有开始测试；[state04/启动闭环](docs/audit/artifacts/cdc_recovery_acceptance_20261116/ble-snapshot-04-boot-verification.json)同session `B4828F8EF45001F1` / boot1为READY1/CAP8。启动SPI2MHz、2000读取CRC0、SPI/I²C overlap694,027µs；MEM scan created、栈16,384 B，beforeFree/largest40,168/30,708，after 22,368/13,812，查询intFree/largest/min26,024/13,812/22,260 B，仍是顺序观察。screen19随后闭合，正常BLE缺口预算仍FAIL；详见本页screen19结果。

`ble-snapshot-04` passes its [complete 73-entry run and build](docs/audit/artifacts/cdc_recovery_acceptance_20261116/ble-snapshot-04-validated-identity.json), with 195 matching before/after production inputs. App size is 1,615,664 bytes; complete binary/ELF hashes appear above, with a verified [bin/ELF/map archive](docs/audit/artifacts/cdc_recovery_acceptance_20261116/ble-snapshot-04-image-archive.json). After actual flash, [state03](docs/audit/artifacts/cdc_recovery_acceptance_20261116/ble-snapshot-state-03.summary.json) is initially READY0/boot_sweep, a normal unfinished startup where no test begins. [State04/boot verification](docs/audit/artifacts/cdc_recovery_acceptance_20261116/ble-snapshot-04-boot-verification.json) confirms READY1/CAP8 in session `B4828F8EF45001F1`, boot1. Startup SPI is 2 MHz with 0/2000 CRC rejections and 694,027 µs I²C overlap. Sequential MEM records a created 16,384-byte scan stack, before free/largest 40,168/30,708, after 22,368/13,812 and query-time internal free/largest/min 26,024/13,812/22,260 bytes. Screen19 subsequently closes with another normal BLE gap-budget failure; see the screen19 results above.

<a id="screen18-protected-results"></a>

## 2026-11-21 screen18：保护机制首轮实测未通过 / First protected-output screen fails acceptance

本次完整[判定](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-18/screen-18-verdict.json)属于 `7ce5…` ELF、session `0C1E0125F83FF11C` / boot1，VOLT1配置rev4、mgen3/mrid4、rgen2/rrid3、pgen3/prid4、applied boundary804。三路collector及收尾查询已退出。状态 **FAIL_BLE_THROUGHPUT**，记录时长达标不等于验收通过；启动内存故障已经修复，吞吐仍需继续修复。下面所有数据保留原窗口和无效点。

The complete [verdict](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-18/screen-18-verdict.json) belongs to ELF `7ce5…`, session `0C1E0125F83FF11C`, boot1, VOLT1 revision4, mgen3/mrid4, rgen2/rrid3, pgen3/prid4 and applied boundary804. All three collectors and closing queries have exited. **FAIL_BLE_THROUGHPUT** remains the result: completing the requested duration is not acceptance. Startup memory recovery is verified, while throughput still needs correction. Each observation below retains its own window and invalid points.

| 项目 / Measurement | 实测与边界 / Observed result and scope |
|---|---|
| 采集时长 / Capture duration | 120.000089528秒 / seconds |
| Capture / USB完整实收 / BLE完整实收 | **63.042559664 / 63.017921719 / 61.120459804 FPS**。Capture按可信序号与设备µs时间跨度；接收率按首末完整K到达计算。 / Capture uses trusted sequence/device timestamps; receive rates use first-to-last completing-K receipt. |
| USB | 7,562完整可信帧，seq1165…8726内部连续；ASCII CRC/identity/wire错误0。 / Internally consecutive trusted frames, zero complete-frame integrity/identity/contract errors. |
| BLE | 7,334完整可信帧，seq1164…8725内缺228；**228/7,562=3.015075377%**，超过正常2.5%目标。分母来自BLE端点，不能替换成USB或SDK计数。 / Missing observations within BLE endpoints, exceeding the normal budget; do not substitute USB/SDK totals. |
| 原字节 / Original bytes | 7,333公共可信帧的原header/data/K字节SHA相同；USB缺BLE匹配229、BLE缺USB匹配1，含不同窗口边缘，不能另加到228。 / All common original packets match; unmatched cross-receiver edges are not additional independent loss counts. |
| 测点 / Point quality | USB466、BLE442个X12，归于悬空S1D2；不补零。acquired/fresh缺失0。完整帧CRC有效不等于所有点有效。 / Invalid X12 points remain, with no unacquired/not-fresh points; a complete trusted frame need not contain all-valid points. |
| ADC SPI CRC | **0/79,590 checked reads**，40,004 even＋39,586 odd，独立118.844514607秒ADSHEALTH窗；engine384…7872，entry0，interval64。 / Separate health receipts, not frame count or exact capture endpoints. |
| PC完整帧间隔 / PC complete-frame interval | BLE相邻可信完整帧最大**42.185839 ms**，7,333对；USB48.434697 ms。仅同身份已观察帧间，不含窗口边缘，不能证明配置首帧、指定保护ticket或弱链路1Hz必达。 / Observed adjacent full frames only; not a first-frame/ticket/weak-link delivery guarantee. |
| 恢复 / Recovery | 无fault/reset/WDT文本观察；[收尾只读查询](docs/audit/artifacts/cdc_recovery_acceptance_20261116/ble-snapshot-postscreen18-01.summary.json)同ELF/session，CAP8、rev6/boundary8761，guard ready1/quarantined0/rejected0。 / No such event observed within coverage; later query confirms CAP8 and unquarantined guard. |

[保护计数分析](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-18/screen-18-protection-01.json)验证采样端点同gen1/stream1、同不可变VOLT身份，**不是跨层原子查询**。每项本地snapshot时刻位于各自Host查询区间，因此窗口给出下限/上限：

[Protection-counter analysis](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-18/screen-18-protection-01.json) validates matching sampled gen1/stream1 and immutable VOLT identity, **not a cross-layer atomic query**. Each local snapshot falls within its own host request/reply bracket, producing the following interval bounds:

| 层 / Layer | 独立窗口秒 / Own window seconds | 实际增量 / Observed delta |
|---|---|---|
| SNAPSRC | 120.435041252–120.458481638 | queued/periodic +119，first/miss/superseded +0 |
| SNAPNET | 120.423265878–120.460826889 | queued/bleAccepted +129，derived +10，obsolete +5；retry/expired/bleExpired/wifiExpired/bleFailed/policySkip +0 |
| BLESNAP | 120.432523513–120.468759668 | queued/localComplete +239，staleNormal +118，deadlineMiss +1；failed/admitMiss/superseded +0 |
| BLEFLOW | 120.445937636–120.476209724 | attempts +7,574，capacityReject +99；intentionalSkip/probeSelected +0，两端stage=full、ratePm=1000 |

119→129→239的差异在screen18闭合时提示周期相位假设；后续真实源码host已复现重复触发并修复（见上文机制），仍不能认定全部228缺帧来自它，不能扣掉118或99。`ratePm=1000`与intentionalSkip0没有主动2.5%丢弃，但不保证队列不会拒绝。BLE本地complete239不能当239个PC已确认保护包；其lastSeq可能晚于捕获窗口，字段仍原样保留。末端maxCompleteGapUs1,009,309是本地同配置历史最大gap，既非PC42.185839ms也不是当前精确窗口最大。RSSI/空中丢包仍unknown，不能凭3.015%将本次自动改判为弱链路通过。

The 119→129→239 difference initially suggests independently phased periodic obligations. Later actual-source host execution reproduces and repairs duplicate triggering as described above; neither 118 stale-normal events nor 99 capacity rejections can be subtracted from 228 missing observations. Full rate and zero intentional skips do not eliminate admission rejection. The 239 local completions are not 239 acknowledged PC snapshots, and their latest sequence may lie beyond the capture window. Final maxCompleteGapUs 1,009,309 is a local same-configuration historical maximum, distinct from the PC 42.185839 ms observation and not an exact-window maximum. RSSI/radio loss remain unknown; the 3.015% ratio cannot reclassify this run as an accepted weak link.

独立BL50窗119.030682509秒为dropD +210/dropL +300/dropC +0；USBQ窗120.483358341秒dDrop/cDrop/expired/cExpired/partial均+0；BLETX窗120.489666451秒SDK calls/accepted/errors为+14,721/+14,720/+0，两个端点可以夹住未返回调用，不能把差1改成PC丢包。上述本地计数均不得与228相加。[CPU比较](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-18/screen17-screen18-comparison.json)保留各任务自身窗口：BLE_TX **17.050516%**、Transport **6.476017%**、AsyncLog **13.032250%**、SerialTX **10.753309%**、SerialCTRL **0.811827%**；全机/SDK CPU和blocked未知，不求和。screen17→18跨ELF/session且改动不止一项，是描述性比较，不是单因素因果实验。

BL50's separate 119.030682509-second span reports dropD +210/dropL +300/dropC +0. USBQ's 120.483358341-second span has no new data/control drop, expiry or partial write. BLETX's 120.489666451-second span has 14,721 calls, 14,720 acceptances and zero errors; an in-flight call may straddle query endpoints, so the one-count difference is not PC loss. None is additive to 228. [CPU comparison](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-18/screen17-screen18-comparison.json) retains separate task windows: BLE_TX 17.050516%, Transport 6.476017%, AsyncLog 13.032250%, SerialTX 10.753309%, SerialCTRL 0.811827%. Whole-system/SDK CPU and blocked time remain unknown; percentages are not summed. Different images/sessions and multiple changes make screen17→18 a descriptive comparison, not a causal experiment.

完整STK50的heap observed free/lifetime minimum为8,274,596/8,271,100 B，栈最低剩余log/transport/bleTx/bleCtrl/serialCtrl为12,500/3,248/2,080/5,208/1,680 B；精确capture窗最大内部连续块未采到。[后续MEM查询](docs/audit/artifacts/cdc_recovery_acceptance_20261116/ble-snapshot-postscreen18-01.summary.json)为intFree/intLargest/intMin26,180/13,812/22,276 B，extFree/extLargest8,248,968/8,126,452 B，scan仍created；它是独立后续顺序读数，不能反填为120秒最坏值。

Complete STK50 records report observed free/lifetime-minimum aggregate heap 8,274,596/8,271,100 bytes and minimum remaining log/transport/bleTx/bleCtrl/serialCtrl stacks 12,500/3,248/2,080/5,208/1,680 bytes. Internal largest-block behavior over the exact capture window is unobserved. A [later MEM query](docs/audit/artifacts/cdc_recovery_acceptance_20261116/ble-snapshot-postscreen18-01.summary.json) reports internal free/largest/min 26,180/13,812/22,276 and external free/largest 8,248,968/8,126,452 bytes, scan=created. Its sequential observations cannot be backfilled as 120-second worst cases.

[criteria补充](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-18/screen-18-criteria-v2-supplement.json)实际命令exit0，沿用同一criteria源SHA，判定 **EXCEEDS_REVISED_NORMAL_LINK_GAP_BUDGET / overall NOT_ESTABLISHED**。首次verdict命令仅因previousCPU文件名错误exit1；原失败保留，修正路径后第二命令exit0，不是设备重试或数据重录。7个关闭raw/event/query文件已[归档回验](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-18/archive-index-01.json)：20,106,304 B→2,081,561 B，原件保留，字节/gzip/SHA一致。该镜像的600秒、50次切换、32配置整套、SHORT/弱链路、SDK阻塞恢复、绝对精度仍未验收；继续先修当前问题，再运行后续矩阵。

The executed [criteria supplement](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-18/screen-18-criteria-v2-supplement.json) exits zero, retains the identical criteria source hash and returns **EXCEEDS_REVISED_NORMAL_LINK_GAP_BUDGET / overall NOT_ESTABLISHED**. The first verdict command fails only because the previousCPU filename is wrong; its original failure remains, and the corrected second command exits zero without device retry or rerecording. [Seven closed raw/event/query files](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-18/archive-index-01.json) compress 20,106,304 to 2,081,561 bytes with verified byte/gzip/SHA roundtrips and originals retained. This image's 600-second, 50-transition, full 32-configuration, SHORT/weak-link, SDK-stall recovery and absolute-precision acceptance remain incomplete. Correct the current failure before extending the matrix.

<a id="ble-protected-output"></a>

## 2026-11-21 保护快照与输出适应 / Protected snapshots and output adaptation

本节描述新候选源码；下文 screen17 和独立 600 秒记录继续只属于各自 ELF。`ble-snapshot-01` 构建通过（193 个生产输入一致，app 1,632,512 B，BUILD `8a1794c-dirty`），但独立审查发现 Transport 到期清理饥饿及 Wi-Fi 到期计数错误，故 **REVIEW_HOLD、未烧录**；后续修复候选的构建/全量测试及实机结论须单独记录，不能继承旧实测 PASS。build01 binary SHA256 `bbbd5bcb92c9aca5080697ea508536671dc28bdaa5e39d31518524f90144d342`，ELF SHA256 `db43fa60f9b408306d7f3c8d19ab4918c985eb312f5a4a5018e1a876adeaaf4f`。

This section describes the new candidate source. Screen17 and the independent 600-second recordings below remain evidence for their own ELF images. `ble-snapshot-01` builds successfully with 193 unchanged production inputs, a 1,632,512-byte app and BUILD `8a1794c-dirty`, but independent review finds Transport expiry starvation and incorrect Wi-Fi expiry accounting. It is therefore **REVIEW_HOLD and unflashed**. The corrected candidate requires its own build, full host suite and hardware results; earlier hardware PASS cannot be inherited. The complete build01 binary/ELF SHA256 values are recorded above.

`ble-snapshot-02` [构建与实际测试身份](docs/audit/artifacts/cdc_recovery_acceptance_20261116/ble-snapshot-02-validated-identity.json)对应 app 1,632,496 B、binary SHA256 `e287b819c8a8e6c7d77bd7aa100f8ddfcff1ee4b003f58760bd931b4af3030bc`、上述575c… ELF；193个生产输入匹配，72入口首次70通过、2个旧harness失败，修正harness后各入口最新执行均exit0，原失败保留。它随后实机 **HIL_STARTUP_FAIL_NO_MEM**，不能被构建和host通过覆盖。[内存审计](docs/audit/artifacts/cdc_recovery_acceptance_20261116/ble-snapshot-02-pipeline-memory-audit-01.json)核对目标map/DWARF：Transport内部静态池17,216 B，相对旧派生布局11,904 B增加5,312 B；AsyncLog六个完整帧槽42,960 B和workspace13,536 B已有永久PSRAM优先分配。该失败时刻的internalLargest未观测，尚不能区分碎片、总内部耗尽或TCB分配失败。后续方案只迁移BLE的18,432 B CPU payload，保留内部队列/metadata、所有保护槽和采集栈；Transport不改。

The [build02/test identity](docs/audit/artifacts/cdc_recovery_acceptance_20261116/ble-snapshot-02-validated-identity.json) records a 1,632,496-byte app, the binary hash above and the failed 575c… ELF. All 193 production inputs match. The initial 72-entry run passes 70 and fails two outdated harnesses; both corrected harnesses then execute successfully, with original failures retained. Hardware subsequently yields **HIL_STARTUP_FAIL_NO_MEM**, which build/host success does not override. The [memory audit](docs/audit/artifacts/cdc_recovery_acceptance_20261116/ble-snapshot-02-pipeline-memory-audit-01.json) records a target-map/DWARF Transport pool of 17,216 bytes, 5,312 above the derived old 11,904-byte layout. AsyncLog already permanently allocates its six full-frame slots (42,960 bytes) and workspace (13,536 bytes) with PSRAM preference. Internal-largest-block telemetry at the failure is missing, so fragmentation, total internal exhaustion and TCB failure are not distinguished. The next correction moves only 18,432 BLE CPU-payload bytes while preserving internal queue/metadata, all reserved slots and acquisition stacks; Transport stays unchanged.

内存修复候选 `ble-snapshot-03` 已[完成构建及73入口记录](docs/audit/artifacts/cdc_recovery_acceptance_20261116/ble-snapshot-03-validated-identity.json)：app 1,615,232 B，binary `726425b30120e2a3f72c2628a4963d79b216186a785f5b1d08809d62429fa30c`，ELF `7ce5e9f8746aefe0f999b467bc08fc9470903a59141339829c6363a30db7e1c8`，195个生产输入一致。首轮71通过、2个MEM依赖旧fixture编译失败，定向修正后各入口记录均通过，原失败不删。[独立审查](docs/audit/artifacts/cdc_recovery_acceptance_20261116/ble-payload-memory-independent-review-01.json)测得BLE内部静态对象23,080→4,688 B，释放18,392 B（18,432 B数据迁移扣除指针/对齐），未减槽/栈。PSRAM分配在一次性init claim之后、所有worker/SDK之前完成，失败明确NO_MEM且本boot不重建；无internal fallback、热分配或未知owner释放。新[完整镜像归档](docs/audit/artifacts/cdc_recovery_acceptance_20261116/ble-snapshot-03-image-archive.json)已gzip/SHA回验。身份记录生成后已[实际烧录及启动复验](docs/audit/artifacts/cdc_recovery_acceptance_20261116/ble-snapshot-03-boot-verification.json)：READY1/CAP8，SPI2MHz、0/2000 CRC、I²C/SPI overlap692,890µs；不是运行期CRC或精度证明。MEM的scan=created、stack=16384，创建前free/largest40,184/30,708 B、后22,384/13,812 B；查询时内部free/largest/min为28,632/13,812/22,276 B，外部free/largest为8,248,968/8,126,452 B。读数顺序采集，差值不归因独占task分配。screen18已闭合且BLE正常缺口预算FAIL，详见本页实测节。

Memory candidate `ble-snapshot-03` has [build and 73-entry records](docs/audit/artifacts/cdc_recovery_acceptance_20261116/ble-snapshot-03-validated-identity.json): a 1,615,232-byte app, the binary/ELF hashes above and 195 unchanged production inputs. Its initial run passes 71 and exposes two outdated MEM-dependency fixtures; targeted fixture repairs pass, retaining original failures. The [independent review](docs/audit/artifacts/cdc_recovery_acceptance_20261116/ble-payload-memory-independent-review-01.json) measures BLE's internal static object shrinking from 23,080 to 4,688 bytes, releasing 18,392 bytes after pointer/alignment overhead. Slot counts and stacks are unchanged. The one-time initialization claim precedes PSRAM allocation, which completes before workers/SDK initialization; allocation failure remains explicit NO_MEM for that boot. There is no internal fallback, hot allocation or release of an unknown owner. The [complete generated images](docs/audit/artifacts/cdc_recovery_acceptance_20261116/ble-snapshot-03-image-archive.json) are gzip/SHA-verified. Subsequent [actual flash/boot verification](docs/audit/artifacts/cdc_recovery_acceptance_20261116/ble-snapshot-03-boot-verification.json) confirms READY1/CAP8, 2 MHz SPI, 0/2000 startup CRC rejections and 692,890 µs I²C/SPI overlap, without establishing runtime CRC or precision. MEM records scan=created and stack=16384: before free/largest 40,184/30,708 bytes and after 22,384/13,812 bytes. At query time internal free/largest/min is 28,632/13,812/22,276 bytes and external free/largest 8,248,968/8,126,452 bytes. Sequential measurements do not isolate task allocation through subtraction. Screen18 is closed and fails the normal BLE gap budget; see the measured-results section.

**输出策略不是允许坏数据通过。** 正常 `BLEFLOW.stage=full` 的 `ratePm=1000` 不按 2.5% 主动丢帧；2.5% 与弱链路 10–20% 是端到端验收预算，不是随机丢弃目标。策略只观察本地 DATA 容量拒绝，不能据此声称 RSSI、无线丢包或 PC 接收率。有效证据窗至少 1 秒、最多 2 秒；连续 3 个有效窗的容量拒绝率严格大于 20% 后，普通 BLE 输出比例每步降低 50‰（5 个百分点），相邻降档至少 3 秒。10–20% 本地拒绝可显示 pressure，但不因此降档。采集任务、原始 ADC 值和科学有效性判据不随该策略改变。

**Output policy does not admit corrupt data.** In normal `BLEFLOW.stage=full`, `ratePm=1000` does not intentionally discard 2.5% of frames. The normal 2.5% and weak-link 10–20% allowances are end-to-end acceptance budgets, not random-drop targets. Adaptation observes local DATA capacity rejection, not measured RSSI, radio loss or PC receipt. A valid evidence window lasts 1–2 seconds. Three consecutive valid windows strictly above 20% capacity rejection reduce ordinary BLE selection by 50‰ (5 percentage points), with at least three seconds between reductions. Local rejection around 10–20% may show pressure without reducing the rate. Acquisition, raw ADC values and scientific-validity criteria remain independent.

恢复需要累计 10 秒**实际有效 clean 证据**：没有容量拒绝或其它失败，且有完整 message 的本地完成；每次提高 5 个百分点。空闲、无尝试、失订阅、未知观察或超过 2 秒的缺口不能当成成功。没有新证据只暂停累计，已有 clean 证据超过 3 秒 TTL 即失效。受限状态仍可从真实候选帧每秒选择至多一个恢复探测，不复制历史样本；探测计入普通选择数，因此实际普通输出不必严格等于比例值。

Recovery requires ten accumulated seconds of **valid clean evidence**: no capacity rejection or other failure, and at least one complete local message completion. Each recovery step adds five percentage points. Idle time, no attempts, lost subscription, unknown observations or gaps longer than two seconds are not success. Missing evidence pauses accumulation; clean evidence expires after a three-second TTL. While limited, at most one recovery probe per second may select an offered real frame. It never replays a historical sample. Probes count as ordinary selections, so observed output need not equal the proportional setting exactly.

完整快照在 source、Transport、BLE **各有 2 个独立保护槽**。source 的现行配置为 4 普通＋2 保护槽，USB 仍按 source FIFO 消费；Transport 为 4 普通＋1 lifecycle＋2 保护槽；BLE 为 4 普通 DATA＋2 LOG＋2 CTRL＋2 保护槽。保护数据始终使用真实 encoder 生成的完整原包及原 CRC，保留所有活动行、invalid/Xhh 和不可变 session/boot/revision/sequence/mode/rows/profile 身份。完整不等于所有点有效；不会填零、平滑或发送时改标签。普通 CAP 的 `TX=SHORT` 仍可发送 B20 预览，保护快照始终发送完整原包。

Each of source, Transport and BLE has **two dedicated protected slots**. The current source allocation is four ordinary plus two protected slots, and USB consumes source FIFO order. Transport has four ordinary, one lifecycle and two protected slots; BLE has four ordinary DATA, two LOG, two CTRL and two protected slots. Protection carries the complete original encoder packet and original CRC, including every active row, invalid/Xhh markers and immutable session/boot/revision/sequence/mode/rows/profile identity. Complete does not mean every point is valid. There is no zero-fill, smoothing or relabeling at send time. Ordinary CAP may retain the B20 `TX=SHORT` preview; protected snapshots always use the original full packet.

配置或有效 DATA stream 改变后，下一份可用真实完整帧承担独立first-config义务；build04起，周期以同身份、seq前进的最近成功typed完整帧完成＋1秒计时，普通FULL也满足，SHORT预览不满足；上游PERIODIC只作候选，不叠加BLE时钟。最早候选若因已持有槽或编码失败无法保留，会记录 miss，再保护后续可用帧，不冒称最早采集帧已发送。BLE在短锁中选择完整原包/普通预览并保留WRITING，锁外计算CRC/复制后重验身份发布，避免退订/重订时把 SHORT 包误升格为完整快照。CTRL/关键生命周期先于保护快照，保护快照先于普通 DATA/LOG；已交给 SDK 的对象不得因超时回收。新快照超越的旧普通帧显式计数，原 identity 不变。

After configuration or effective DATA-stream changes, the next available real complete frame carries an independent first-configuration obligation. From build04, periodic due is the most recent advancing same-identity typed-full local completion plus one second. Ordinary FULL qualifies; SHORT preview does not. Upstream PERIODIC is a candidate hint rather than another BLE clock. If an earliest candidate cannot be retained because a slot is owned or encoding fails, a miss is counted and a later available frame is protected; the original first capture is not claimed as sent. BLE selects bytes and reserves WRITING under a short lock, copies/CRCs outside it and revalidates identity before publication, preventing a SHORT preview from being promoted incorrectly during unsubscribe/resubscribe. CTRL/critical lifecycle precedes protected snapshots, which precede ordinary DATA/LOG. Timeout cannot reclaim an SDK-owned object. Older ordinary frames overtaken by protection are counted explicitly without changing their identities.

**两种“1 秒”不同。** BLE 的 1 秒是 due/miss 调度，不是交付或释放截止；连续 CTRL 可以让 firstPending 长时间保持 pending，周期未完成由 deadlineMiss 反映。Transport 的 1 秒是**保护 network descriptor 从入队开始的共享总 TTL**：每次调度做常数有界清理，最多处理两个 pending 槽及一个本任务 retry，且不调用 lwIP/SDK。到期 descriptor 记一次 expired；按入队捕获的目标分别记 bleExpired（BLE 尚未 admitted，NO_MEM 重试也属此类）和 wifiExpired（Wi-Fi 当时 enabled/ready，且从未 attempted）。Wi-Fi 已尝试成功或失败均不重复计数/发送，BLE 未启用不虚构 BLE timeout；USB 源帧独立处理。TTL 不取消正在执行的 SDK；现有 Wi-Fi `MSG_DONTWAIT` 也不证明 lwIP 内部 mailbox/semaphore 等待有界。

**The two one-second intervals differ.** BLE's interval is a due/miss schedule, not a delivery or reclamation deadline. Continuous CTRL can leave firstPending pending indefinitely; deadlineMiss exposes missed periodic obligations. Transport applies a **shared one-second total TTL to protected network descriptors from enqueue**. Each dispatch performs constant bounded cleanup of at most two pending slots and one task-owned retry, without calling lwIP or the SDK. Each expired descriptor increments expired once; captured targets separately increment bleExpired (not yet admitted, including NO_MEM retries) and wifiExpired (enabled/ready at enqueue and never attempted). A successful or failed Wi-Fi attempt is never counted or sent twice; disabled BLE does not invent a BLE timeout. USB source processing remains independent. TTL does not cancel an executing SDK call; existing Wi-Fi `MSG_DONTWAIT` also does not prove bounded waits inside lwIP mailboxes/semaphores.

新增只读查询保持 PROTO2、科学帧格式和 CRC：`BLEFLOW?` 由 `ble_flow_v1` 协商，`BLESNAP?`、`SNAPSRC?`、`SNAPNET?` 由 `ble_snapshot_v1` 协商，各自回复 `v=1`。主机必须先检查 PROTO caps：缺少对应能力或明确 UNKNOWN/unsupported 时记 unavailable，查询超时记 unknown，不能臆造全零；未知字段可保留/忽略但不能重解释旧字段。字段和计数域见[主机契约](docs/integration/firmware_host_contract.md#protected-snapshot-v1)。`BLESNAP.failed` 只覆盖部分发送终态，不含全部 stale/取消/替代/drop，禁止用 queued−localComplete−failed 推算队列深度。SDK 接受、完整本地完成与 PC 完整 CRC/identity 实收是不同层；没有应用层 ACK，优先保护不保证 exactly-once 或必达。BLE 500 ms SDK guard 的隔离同样不是取消或安全回收证明。

New read-only queries preserve PROTO2, scientific frame format and CRC. PROTO advertises `ble_flow_v1` for `BLEFLOW?` and `ble_snapshot_v1` for `BLESNAP?`, `SNAPSRC?` and `SNAPNET?`; each reply has `v=1`. Hosts must check the corresponding advertised capability first: absent capability or explicit UNKNOWN/unsupported means unavailable; timeout means unknown, never fabricated zero counters. Preserve or ignore unknown extension fields without reinterpreting old ones. See the [host contract](docs/integration/firmware_host_contract.md#protected-snapshot-v1) for field domains. `BLESNAP.failed` covers only some send terminal paths, excluding all stale/cancel/supersede/drop outcomes; queued−localComplete−failed is not queue depth. SDK acceptance, whole local completion and complete CRC/identity-valid PC receipt are distinct layers. With no application ACK, priority protection guarantees neither exactly-once nor delivery. The 500 ms BLE SDK guard likewise quarantines rather than proving cancellation or safe reclamation.

新机制已运行真实 source/Transport/encoder 的 host 回归：旧首帧淘汰反例、50 次配置替换、C/V/R/M 原 CRC、2304 字节、SHORT、WRITING/owned 生命周期、连续控制下到期及逐 sink 计数；RTOS/无线边界为合成模型，**不是 HIL 或帧率证明**。旧 screen17、600 秒失败和原 raw/FAIL 文件均保留；新候选 600 秒、50 次实机切换、32 配置 USB/BLE、弱信号和 PC 完整快照验收仍须分别执行。

Host regressions execute actual source/Transport/encoder code for the old first-frame-eviction counterexample, 50 configuration changes, original C/V/R/M CRCs, 2304-byte payloads, SHORT, WRITING/owned lifetimes, expiry under continuous control and per-sink accounting. RTOS/radio boundaries are synthetic: **these are not HIL or frame-rate evidence**. Screen17, historical 600-second failures and original raw/FAIL files remain. The new candidate still needs its own 600-second run, 50 hardware transitions, 32 USB/BLE configurations, weak-signal testing and complete-PC-snapshot acceptance.

## USB-OTG CDC 与 BLE 运转机制 / USB-OTG CDC and BLE operation

USB-OTG CDC 仍然是原生 USB Full Speed（12 Mbit/s 总线速率），不是 UART 转 USB。虚拟串口的波特率参数不限制总线速率；实际完整帧吞吐受主机读取、USB 调度、编码、队列和采集耗时共同影响，必须实测，不能用 12 Mbit/s 推算成已达成的有效吞吐。S3 OTG 内部 DMA 使用持续存活的内部 DRAM 缓冲；应用文本复制到 TinyUSB FIFO，不是 PSRAM 零拷贝。`USB?.dma` 查询实际控制器 enable 位。

USB-OTG CDC is native USB Full Speed (12 Mbit/s signaling), not a UART-to-USB link. Virtual baud settings do not set the USB wire rate. Complete-frame throughput still depends on host reads, bus scheduling, encoding, queues and acquisition, so signaling rate is not measured payload throughput. S3 internal OTG DMA uses persistent internal-DRAM buffers; application text is copied into TinyUSB FIFOs rather than DMA-read directly from PSRAM. `USB?.dma` reads the controller's actual enable bit.

双 CDC 用独立端点隔离正式口与调试口；正式口内 DATA 和 CTRL 在完整记录边界仲裁，CTRL 优先，已发送部分 DATA 不允许插入 CTRL 字节。短写、到期、断连和配置 epoch 均有独立状态；到期部分记录以换行重新建立边界并记丢弃，不补零或修改科学帧。调试口无人读取不能占住正式口。只有一个 TinyUSB owner，保留 DMA 停止确认、受控重启和 ROM 下载通路；不会因修改虚拟波特率或 DTR 自动进下载模式。

Separate CDC endpoints isolate the formal stream from debug output. DATA and CTRL share the formal byte stream only at complete-record boundaries: CTRL has admission priority, but cannot interrupt partially accepted DATA bytes. Short writes, expiry, disconnects and connection epochs have explicit state. An expired partial record gets a delimiter and a counted loss; scientific data is neither zero-filled nor relabeled. An unread debug port cannot retain the formal endpoint. One TinyUSB owner preserves the hardware DMA shutdown fence, controlled restart and ROM download path. Virtual baud or DTR changes do not trigger bootloader entry.

BLE DATA/CTRL 应用任务优先级为 7/8；`1e9fd19` 将正式 OTG USB owner 与 transport 从 6 提至 7，旧 Serial/JTAG owner 保持 6；DATA、LOG、控制终态使用有界容量，普通日志不能挤占保留终态槽。`BLELINK?` 报真实完成回调的 PHY/连接参数，`BLETX?` 报 SDK 调用时长和本地排队情况。SDK 接受、USB FIFO 接受、回调和主机完整 CRC 帧实收是不同统计层。CDC 回调没有分 lane 的字节数，所以 `USBQ` 对相应字段输出 `unknown`。IDF 单次 BLE GATT 调用内部的 BTC 队列仍可能阻塞；外层截止时间不证明内部调用已取消。

BLE DATA/CTRL application priorities are 7/8. In `1e9fd19`, the formal OTG owner and transport move from 6 to 7; the legacy Serial/JTAG owner stays at 6. DATA, LOG and terminal replies have bounded capacity; ordinary logs cannot consume reserved terminal slots. `BLELINK?` reports observed PHY/connection completion, while `BLETX?` reports SDK call timing and local queues. SDK acceptance, USB FIFO acceptance, callbacks and host CRC-valid complete-frame receipt are distinct layers. CDC callbacks provide no per-lane byte counts, so the corresponding `USBQ` values are `unknown`. The IDF GATT call can still block internally on its BTC queue; an outer deadline does not establish cancellation.

接口通过 VID/PID、板卡 MAC 序列号、接口编号和 `PROTO?` capability 共同识别，不能按 `/dev/ttyACM0` 枚举顺序猜测。新双 CDC 开发 PID 为 `303a:4042`，接口 0/1 正式、2/3 调试；旧 vendor 开发 PID `303a:4041` 不混用。当前 Linux 可利用已有 `cdc_acm` 驱动和 `dialout` 组访问；这不是对所有操作系统权限的保证。Windows 仍待实测。`BUILD?` 新增运行镜像 version/ELF SHA256，以免把工作区 HEAD 当作板上版本。

Discovery verifies VID/PID, the board's MAC-derived USB serial, interface number and `PROTO?` capability together, never tty enumeration order. The dual-CDC development PID is `303a:4042`, with interfaces 0/1 for formal traffic and 2/3 for debug; it is distinct from the optional vendor PID `303a:4041`. This Linux account can use its existing `cdc_acm` driver and `dialout` membership; this is not a universal OS permission guarantee. Windows remains untested. `BUILD?` adds running-image version and ELF SHA256 so a checkout HEAD is not mistaken for the flashed build.

本轮已完成及未完成项目以[阶段报告](docs/audit/usb_otg_ble_priority_20261114.md)和逐命令 manifest 为准。夹具仍为 S1D1/S8D8 10 kΩ、S4D4/S5D5 33 pF，其余悬空电容仅作为环境背景代理。AVDD/AVSS 的用户读数只用于注明来源的临时 RAM 校准；不执行 CAL=SAVE，不把背景扣除当作绝对精度验证。

Completed and pending checks are recorded in the [stage report](docs/audit/usb_otg_ble_priority_20261114.md) and command manifests. The fixture remains 10 kΩ at S1D1/S8D8 and 33 pF at S4D4/S5D5; floating-port capacitances are environmental-background proxies. User-supplied AVDD/AVSS readings support only explicitly sourced temporary RAM calibration. No CAL=SAVE is performed, and background subtraction is not absolute accuracy validation.

恢复闭环源码 / Recovery-loop source: `3a55543cb21fa3412ede26ee79c1cce7d8371804`; BUILD `3a55543`; app 1,591,088 B; binary SHA256 `59d3f07d772730692ea4a68e9af6437779ddfd24849c78f8fb9a03fbb2e31fee`; ELF SHA256 `d4ac82a199bd2bc8bb730b331225ffdbb94acd2c84c410cd675636da706e92e5`. [构建与 58 入口验证 / Build and 58-entry validation](docs/audit/artifacts/cdc_recovery_acceptance_20261116/recovery-candidate-01-validated-identity.json), [正式 CDC 恢复闭环 / Formal-CDC recovery loop](docs/audit/artifacts/cdc_recovery_acceptance_20261116/cdc-rom-01-verified-loop.json).

更新时由 USBBOOT 显式请求 ROM 下载，owner 在同一个 250 ms 总期限内停止 USB 事件、保持至少 20 ms detach 窗口、确认控制器 reset 后才允许 CPU restart；20 ms 是工程间隔，不是模拟节点或主机收到终态的证明。恢复模式的 RTC 标记不再在启动选择时清除，普通 RESTART 和重复 early panic 保留 USJ；显式 USBBOOT 完成准备并读回 ROM 启动标志后才解除。完整 BUILD 哈希直接读取应用描述符 32 字节，避免 IDF 日志缓存长度仅 9 位造成截断。主机只读查询会保留未关联的精确 UNKNOWN 错误并在原期限内等待真实回复，模式 setter/终态 ID 门控不变。

Updates explicitly request ROM download using USBBOOT. Within the same 250 ms transaction deadline, the owner stops USB events, holds an engineering detach window of at least 20 ms and verifies controller reset before CPU restart. That interval proves neither analog stability nor terminal receipt. The RTC recovery latch now survives backend selection, ordinary RESTART and repeated early panic; only an explicit, prepared USBBOOT with a read-back ROM flag releases it. BUILD reads all 32 descriptor hash bytes rather than the IDF's nine-character logging cache. Read-only host queries retain an uncorrelated exact UNKNOWN error while awaiting the real reply within the original deadline; mode setter and terminal-ID checks remain strict.

本机 esptool 4.12 的 RTS-only 退出下载未启动应用；明确清 DTR 也未恢复，根因仍未完全确认。新的 app-only 助手在核对镜像、分区、MAC 和写入哈希后，仅调用一次官方 S3 watchdog 软件复位，独立核验新枚举和 BUILD。该路径已实测，不等于任意硬件故障都能免物理操作恢复。旧失败与尚未运行的早崩溃注入均保留在阶段报告。

Here, esptool 4.12's RTS-only download exit did not start the application; explicitly clearing DTR also failed, so the full cause remains unresolved. After image, partition, MAC and flash-hash checks, the new app-only helper calls the official S3 watchdog software reset once, with independent enumeration and BUILD verification. This path has hardware evidence, but arbitrary hardware failures are not guaranteed to recover without physical access. Prior failures and unrun early-crash injection remain in the stage report.

## 2026-11-21 screen17：普通日志让出 BLE 容量 / Ordinary logs yield BLE capacity

当前完整 ELF 为 `f1ec518dfba78453dcd7fe057bba29eb61a8f8898274a21e22125df26b933a04`，BUILD 文本仍 `8a1794c-dirty`，session `863626B103487FE5` / boot 1；与下文 8e61… 的 600 秒记录不同。[69 个 host 入口/构建](docs/audit/artifacts/cdc_recovery_acceptance_20261116/ble-log-policy-01-validated-identity.json)通过、187 个生产输入匹配，[启动身份](docs/audit/artifacts/cdc_recovery_acceptance_20261116/ble-log-policy-01-boot-verification-02.json)核验。此版只对有效 DATA 订阅期间的新普通 BLE LOG 准入做计数拒绝，保留关键 CTRL/生命周期和已接受记录；采集、科学 CRC/invalid、队列容量及 SDK 未变。它不是尚在实施的 RSSI 适应或 1 Hz 保护快照策略。

The current full ELF is `f1ec518dfba78453dcd7fe057bba29eb61a8f8898274a21e22125df26b933a04`, with the same BUILD text `8a1794c-dirty`, session `863626B103487FE5`, boot 1. It differs from the 8e61… image’s 600-second run below. The [69-entry host suite/build](docs/audit/artifacts/cdc_recovery_acceptance_20261116/ble-log-policy-01-validated-identity.json) pass with 187 matching production inputs, and [boot identity](docs/audit/artifacts/cdc_recovery_acceptance_20261116/ble-log-policy-01-boot-verification-02.json) is verified. This version counts and rejects new ordinary BLE LOG admissions while an effective DATA subscription exists, preserving critical CTRL/lifecycle and already accepted records. Acquisition, scientific CRC/invalid handling, queue capacity and SDK are unchanged. This is separate from the RSSI adaptation and 1 Hz protected-snapshot policy still in progress.

| screen17 实测 / Observation | 结果 / Result |
|---|---:|
| VOLT 1 行时长 / VOLT 1-row duration | 120.000057708 s |
| Capture / USB / BLE | 64.978526 / 65.035014 / 64.800778 FPS |
| USB 完整可信帧 / Trusted complete USB frames | 7,805; seq 2633…10437, missing 0 |
| BLE 完整可信帧/内部缺失 / Trusted complete BLE frames/internal missing | 7,776 / 29 |
| criteria v2 缺口 / Gap ratio | 29/7,805 = 0.371557%; numeric budget met |
| ADC CRC 拒绝/检查读取 / Rejected/checked ADC reads | 0/81,537 |
| USB / BLE 无效点 / Invalid points | 373 / 370 |
| 收尾 / Cleanup | CAP8_QUERY_VERIFIED |

[原判定](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-17/screen-17-verdict.json)保留 **OBSERVED_NOT_ACCEPTED**，表示全体验收未完成；[criteria v2 补充](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-17/screen-17-criteria-v2-supplement.json)仅给正常链路数值预算 **MEETS_REVISED_NORMAL_LINK_GAP_BUDGET_ONLY**。7,775 个公共 USB/BLE 帧原字节一致，完整帧 ASCII CRC/identity/wire-contract 错误、unacquired/not-fresh 均 0；S1D2 X12 无效点不因传输预算而变有效。ADC 分母来自独立 119.120057965 秒健康窗（41,114 even + 40,423 odd），不是帧数；首末 engineFrameCount 384…8128、modeEntryFrameCount 0、healthInterval 64、err 0。与旧镜像的变化是观察结果，boot、序号/帧长和窗口不同，不能仅凭这次比较证明全部增益都来自日志策略。

The [original verdict](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-17/screen-17-verdict.json) remains **OBSERVED_NOT_ACCEPTED**, meaning overall acceptance is incomplete. The [criteria v2 supplement](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-17/screen-17-criteria-v2-supplement.json) yields only **MEETS_REVISED_NORMAL_LINK_GAP_BUDGET_ONLY**. All 7,775 common USB/BLE frames retain identical original bytes. Complete-frame ASCII CRC/identity/wire-contract errors and unacquired/not-fresh points are zero; S1D2 X12 points do not become valid because of a transport allowance. ADC checks span a separate 119.120057965 seconds (41,114 even + 40,423 odd), not the frame count. First/last engineFrameCount is 384…8128, with modeEntryFrameCount 0, healthInterval 64 and err 0. Differences from the older image are observations across different boots, sequence/frame lengths and windows, not proof that every gain is caused by log policy alone.

机制与边界：`sensorarrayBleDataActiveLocked` 用与 DATA 准入相同的连接/CCCD/FAST–SAFE 条件；`sensorarrayBleAllocateTxDescriptor` 仅对普通、非 priority 的 LOG 返回计数拒绝。DATA 队列瞬时为空或采集暂停但订阅仍在，不会开放普通 LOG；取消 DATA 订阅后可恢复 LOG-only。关键生命周期即使使用 LOG channel 仍走 priority bypass，已接受/在途 LOG 可完成，既不取消 SDK，也不回收仍在使用的对象。[独立审查](docs/audit/artifacts/cdc_recovery_acceptance_20261116/ble-log-policy-independent-review-01.json)覆盖有效订阅、关键绕行、计数一次/回绕和 owner 生命周期；这不是无线绝对送达证明。

Mechanism and limits: `sensorarrayBleDataActiveLocked` uses the same connection/CCCD/FAST–SAFE conditions as DATA admission. `sensorarrayBleAllocateTxDescriptor` rejects and counts only ordinary, non-priority LOG. A momentarily empty DATA queue or paused acquisition with the subscription retained does not reopen ordinary LOG; unsubscribing DATA restores LOG-only operation. Critical lifecycle records retain priority bypass even on the LOG channel. Already accepted/inflight LOG can finish; neither the SDK nor owned objects are canceled/reclaimed. The [independent review](docs/audit/artifacts/cdc_recovery_acceptance_20261116/ble-log-policy-independent-review-01.json) covers effective subscriptions, critical bypass, single-count/wrap and owner lifetime. It does not establish guaranteed wireless delivery.

新增只读 `BLELOG?` v1 与 `--ble-log` 采集选项已实际用于[前查询](docs/audit/artifacts/cdc_recovery_acceptance_20261116/ble-log-policy-prescreen17-01.summary.json)/[后查询](docs/audit/artifacts/cdc_recovery_acceptance_20261116/ble-log-policy-postscreen17-01.summary.json)。同 session、connection gen=1，均 connected1/dataActive=1/ordinaryAllowed=0；policyDrop/dropL 均 **4→358（+354）**，这是自身 **184.153099360 秒**查询窗的普通日志策略计数，不能写成精确 120 秒内 DATA 丢帧。BL50 自身 119.189627548 秒窗 dropD +22/dropL +310/dropC +0；USBQ 自身 120.407030202 秒窗 drop/expiry/partial 增量均 0。BLETX 自身 120.414763909 秒窗 calls +15,602/accepted +15,603/errors 0，边界上的在途调用可使两个增量差 1，不能改写为 PC 收到 15,603 帧。前后 guard 均 ready=1/quarantined=0/rejected=0/active=0/completionKnown=1。

New read-only `BLELOG?` v1 and the `--ble-log` capture option are exercised in the [before](docs/audit/artifacts/cdc_recovery_acceptance_20261116/ble-log-policy-prescreen17-01.summary.json)/[after](docs/audit/artifacts/cdc_recovery_acceptance_20261116/ble-log-policy-postscreen17-01.summary.json) queries. In the same session and connection gen=1, both report connected1/dataActive=1/ordinaryAllowed=0. policyDrop and dropL each change **4→358 (+354)** over their own **184.153099360-second** query span: ordinary-log policy counts, not DATA losses in exactly 120 seconds. BL50’s 119.189627548-second span reports dropD +22/dropL +310/dropC +0. USBQ’s 120.407030202-second span has zero drop/expiry/partial increments. BLETX’s 120.414763909-second span reports calls +15,602/accepted +15,603/errors 0; an in-flight boundary call can make the deltas differ by one, not prove 15,603 received PC frames. Before/after guard reports ready=1/quarantined=0/rejected=0/active=0/completionKnown=1.

CPUOWNER 独立窗口占比 BLE_TX/TRANSPORT/ASYNC_LOG/SERIAL_TX/SERIAL_CTRL 为 **15.2295/3.6957/12.5544/11.2530/0.8055%**；不同分母不求和，整机/SDK CPU 与 blocked 时间未知。STK50 observed free/lifetime minimum 为 8,301,288/8,297,784 B，最大连续块未知；最低剩余栈 log/transport/bleTx/bleCtrl/serialCtrl 为 12504/3272/2192/5208/1664 B。覆盖内未观测 MFAULT/reset/WDT；CAP8 rev=6/boundary=10462 的收尾查询确认。全部性能和独立窗口见[分析](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-17/screen-17-analysis-01.json)及[比较](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-17/screen16-screen17-comparison.json)。

CPUOWNER ratios over each role’s own span are **15.2295/3.6957/12.5544/11.2530/0.8055%** for BLE_TX/TRANSPORT/ASYNC_LOG/SERIAL_TX/SERIAL_CTRL. Their denominators differ, so they are not summed; whole-device/SDK CPU and blocked time remain unknown. STK50 observed free/lifetime-minimum heap is 8,301,288/8,297,784 bytes, largest contiguous block unknown. Minimum remaining log/transport/bleTx/bleCtrl/serialCtrl stacks are 12504/3272/2192/5208/1664 bytes. No MFAULT/reset/WDT is observed within coverage; closing queries confirm CAP8 rev=6/boundary=10462. See the [analysis](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-17/screen-17-analysis-01.json) and [comparison](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-17/screen16-screen17-comparison.json) for performance and independent windows.

首个离线 verdict 命令因新嵌套 boot JSON 与旧汇总器格式不兼容而 `KeyError: version`/exit1，属工具 schema 问题；保留[失败日志](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-17-verdict-command.txt)及原 boot 文件，基于同一原观测生成 `boot-verification-02` adapter 后[第二命令](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-17-verdict-02-command.json) exit 0，未新增硬件故障。18 个关闭原流/查询/ROM/flash 共 21,620,723 B→2,237,453 B，[归档](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-17/archive-index-01.json)字节/SHA 回验、原件保留，[实际命令](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-17-archive-01-command.json) exit 0。本 ELF 的 600 秒/50 次切换/32 配置、弱链路与新保护快照均未验收，不继承旧 8e61… 覆盖。

The first offline verdict command fails with `KeyError: version`/exit1 because the new nested boot JSON does not match the existing summarizer schema. The [failure log](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-17-verdict-command.txt) and original boot file remain. A `boot-verification-02` adapter from the same observations lets the [second command](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-17-verdict-02-command.json) exit 0; this is a tooling issue, not a new device fault. Eighteen closed streams/query/ROM/flash records compress from 21,620,723 to 2,237,453 bytes, passing [byte/SHA archive verification](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-17/archive-index-01.json) with originals retained and the [command](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-17-archive-01-command.json) exit 0. This ELF’s 600-second/50-transition/32-configuration runs, weak links and new protected snapshots remain unaccepted; it inherits no older 8e61… coverage.

## 2026-11-21 criteria v2：BLE 可接受缺口与保护数据 / Acceptable BLE gaps and protected data

用户将正常链路约 2.5% 完整帧缺口定义为可接受；弱链路允许约 10–20%，更弱时降低常规输出，采集独立运行。关键控制/生命周期、应用新配置后的首个完整帧、周期完整快照应优先。周期当前采用 **1 秒默认目标**，这是本轮实现默认值，不是用户指定的固定间隔。完整快照保留所有活动行及原始 invalid/Xhh，不能要求全点有效后才发送。弱链路/RSSI 分类、输出适应和新增保护策略尚在实施，不能在旧镜像上追认已实现。

The user now accepts about 2.5% missing complete frames on a normal link and about 10–20% on a weak link. A still weaker link should reduce routine output while acquisition continues independently. Critical control/lifecycle, the first complete frame after applying a new configuration and periodic complete snapshots take priority. The current implementation target uses a **one-second default period**; the user requested periodic snapshots without prescribing that interval. A complete snapshot retains all active rows and original invalid/Xhh points rather than waiting for every point to be valid. Weak-link/RSSI classification, adaptive output and the new protection policy remain in progress; they are not retroactively implemented in the old image.

[标准修订](docs/audit/artifacts/cdc_recovery_acceptance_20261116/acceptance-criteria-20261121-v2.json)与[补充判定](docs/audit/artifacts/cdc_recovery_acceptance_20261116/stability-02/stability-02-criteria-v2-supplement.json)重新解释同一份关闭实录，不修改任何原流、分析或旧 FAIL JSON。stability-02 在同身份 BLE 观察端点内收到 38,047 个有序可信完整帧、缺 421 个，因此分母是 38,468，缺口 **1.094416% ≤ 2.5%**。这不是用 USB 的 38,467 帧作分母，也没有减去主动降帧或把本地 dropD 再加进去。结论仅为 **MEETS_REVISED_NORMAL_LINK_GAP_BUDGET_ONLY**；未通过 RSSI 注入证明信号等级，未验收 32 配置、弱链路、保护快照、绝对精度或整个系统。

The [criteria revision](docs/audit/artifacts/cdc_recovery_acceptance_20261116/acceptance-criteria-20261121-v2.json) and [supplement](docs/audit/artifacts/cdc_recovery_acceptance_20261116/stability-02/stability-02-criteria-v2-supplement.json) reinterpret the same closed recording without changing any raw stream, analysis or original FAIL JSON. Within one identity’s BLE observation endpoints, 38,047 ordered trusted complete frames plus 421 missing observations yield a denominator of 38,468 and a gap of **1.094416% ≤ 2.5%**. The denominator is not USB’s 38,467 frames. No intentional decimation is subtracted, and local dropD is not added again. The result is only **MEETS_REVISED_NORMAL_LINK_GAP_BUDGET_ONLY**. It establishes no injected RSSI class and does not accept the 32-configuration matrix, weak links, protected snapshots, absolute accuracy or the whole system.

| 指标 / Metric | 必须保留的含义 / Required interpretation |
|---|---|
| 主动降低输出 / Intentional output decimation | 主动选取较少常规 DATA，采集不随之降速；单独计数，若按所有采集 seq 统计仍会形成缺口。不得没有同窗依据就从损失中扣除。 / Deliberately select fewer routine frames without slowing capture; report separately. These still appear as capture-sequence gaps unless a supported eligible-output denominator is defined. |
| 本地拒绝 / Local rejection | 队列满、guard 或策略在本地拒绝；不等于无线丢包，也不能与 PC 缺口相加为独立损失。 / Queue, guard or policy rejection is local, not measured air loss or an independent count to add to PC gaps. |
| PC 完整帧缺口 / PC complete-frame gap | 定义端点和身份内缺少 CRC/identity 正确的完整帧观察；不表示所有字节都未收到。 / Missing trusted complete-frame observations within defined endpoints/identity, not proof that every byte was absent. |
| ADC SPI CRC / ADC SPI CRC | 分母是检查读取，不是传输帧；现有启动资格 2.5% 独立保留，坏样本仍按既有有限重试/无效处理。 / Checked-read denominator, not transport frames. The separate existing 2.5% startup qualification remains; bad samples retain bounded retry/invalid treatment. |

新普通 LOG 策略的[启动身份](docs/audit/artifacts/cdc_recovery_acceptance_20261116/ble-log-policy-01-boot-verification.json)为 ELF `f1ec518dfba78453dcd7fe057bba29eb61a8f8898274a21e22125df26b933a04`，与这次被补充判定的 8e61… 不同；它不能继承旧镜像的 600 秒或 50 次切换覆盖。本补充生成时尚无新镜像闭合 HIL。实际[重判命令](docs/audit/artifacts/cdc_recovery_acceptance_20261116/stability-02-criteria-v2-supplement-01-command.json) exit 0，并再次校验 7 个原流/gzip 的字节和 SHA。下文保留当时标准下的历史结论；当前数值预算解释以本节为准，历史 FAIL 不被抹去。

The ordinary-LOG candidate’s [boot identity](docs/audit/artifacts/cdc_recovery_acceptance_20261116/ble-log-policy-01-boot-verification.json) is ELF `f1ec518dfba78453dcd7fe057bba29eb61a8f8898274a21e22125df26b933a04`, distinct from the reassessed 8e61… image; it does not inherit the old image’s 600-second or 50-transition coverage. No new-image closed HIL is supplied at this supplement’s creation. The [recorded reassessment command](docs/audit/artifacts/cdc_recovery_acceptance_20261116/stability-02-criteria-v2-supplement-01-command.json) exits 0 and rechecks all seven original/gzip byte and SHA pairs. The following history retains conclusions under the then-current criteria. This section governs the revised numeric-budget interpretation without erasing historical FAIL results.

## 2026-11-21 stability-02：保留原标准下的 10 分钟结果 / Preserved 10-minute result under the original criteria

本次 VOLT 1 行使用上述 8e61… ELF、session `391F6A5399682456`、boot 1，与 screen16 和 50 次身份切换为同一镜像/会话；没有使用后续普通 BLE LOG 抑制候选。三个 reader 和收尾查询均正常退出。600.000114995 秒完整录制的状态是 **CAPTURED_600_SECONDS_NOT_ACCEPTED / FAIL_BLE_THROUGHPUT**，不能写成“10 分钟全项通过”，也不能用旧镜像的结果补齐新候选。

This VOLT 1-row run uses the above 8e61… ELF, session `391F6A5399682456`, boot 1, matching screen16 and the 50 identity transitions. It does not include the later ordinary-BLE-LOG suppression candidate. All three readers and the closing query exit successfully. The full 600.000114995-second recording is **CAPTURED_600_SECONDS_NOT_ACCEPTED / FAIL_BLE_THROUGHPUT**, not a complete 10-minute acceptance pass; old-image results cannot qualify a later candidate.

| 实测项 / Observation | screen16 | stability-02 |
|---|---:|---:|
| 时长 / Duration (s) | 120.000331701 | 600.000114995 |
| Capture / USB / BLE (FPS) | 64.242834 / 64.122140 / 63.414893 | 64.110459 / 64.111810 / 63.412571 |
| USB 完整可信帧 / Complete trusted USB frames | 7,695 | 38,467 |
| USB 内部缺失 / Internal missing USB observations | 0 | 0 |
| BLE 完整可信帧 / Complete trusted BLE frames | 7,609 | 38,047 |
| BLE 内部缺失 / Internal missing BLE observations | 84 | 421 |
| ADC CRC 拒绝/检查读取 / Rejected/checked ADC reads | 0/82,428 | 0/413,595 |
| USB / BLE 无效点 / Invalid points | 606 / 586 | 2,717 / 2,644 |
| 判定 / Verdict | FAIL_BLE_THROUGHPUT | FAIL_BLE_THROUGHPUT |

USB seq 12675…51141 的连续性由同身份有序帧数与首末序号正向核对，非缺字段默认零；38,045 个公共 USB/BLE 帧原始字节完全相同。完整帧 ASCII CRC、identity、wire-contract 错误均为 0，unacquired/not-fresh 点均为 0，但悬空 S1D2 的 X12 仍明确无效，不能把完整接收等同每点可用或绝对精度通过。ADC 健康窗独立为 598.945560056 秒，检查读取分母为 207,444 valid-even + 206,151 valid-odd；不是 38,467 帧，也不把重试数末字段未验证内容填零。实际 engineFrameCount 8512…46912、modeEntryFrameCount 8149、healthInterval 64，首末健康记录均 err=0；未假定切换时累计计数归零。详见[判定](docs/audit/artifacts/cdc_recovery_acceptance_20261116/stability-02/stability-02-verdict.json)与[同镜像比较](docs/audit/artifacts/cdc_recovery_acceptance_20261116/stability-02/screen16-stability02-comparison.json)。

USB contiguity across seq 12675…51141 is positively checked against the ordered count within one identity, not inferred from an absent field. All 38,045 common USB/BLE frames have identical original bytes. Complete-frame ASCII CRC, identity and wire-contract errors are zero, as are unacquired/not-fresh points. Floating S1D2 X12 points remain explicitly invalid: complete receipt does not mean every point is usable or absolute accuracy passes. The separate 598.945560056-second ADC health span counts 207,444 valid-even + 206,151 valid-odd reads, not 38,467 frames; the unverified terminal retry field is not filled with zero. Actual engineFrameCount spans 8512…46912, modeEntryFrameCount is 8149 and healthInterval is 64, with err=0 in the first/last health records; mode entry is not assumed to reset the cumulative count. See the [verdict](docs/audit/artifacts/cdc_recovery_acceptance_20261116/stability-02/stability-02-verdict.json) and [same-image comparison](docs/audit/artifacts/cdc_recovery_acceptance_20261116/stability-02/screen16-stability02-comparison.json).

分层窗口继续分开：BL50 自身 598.946417553 秒窗 dropD +343、dropL +40、dropC +0；USBQ 自身 600.427256331 秒窗 dDrop/cDrop/expired/cExpired/partial 增量均 0，历史 expired=5124/partial=7 未增长。BLETX 自身 600.456470292 秒窗 SDK 调用/接受 79,882 次、错误 0，不能据此替代主机缺失 421 的事实，也不能把各层损失相加。CPUOWNER 各自观察窗占比为 BLE_TX 15.6668%、TRANSPORT 3.7010%、ASYNC_LOG 12.5184%、SERIAL_TX 10.6578%、SERIAL_CTRL 0.7566%；分母不同，不求和为整机 CPU，SDK CPU/blocked 时间未知。完整 STK50 的 observed free 为 8,301,040 B、firmware lifetime minimum 为 8,297,160 B，最大连续块未知；剩余栈最低值 log/transport/bleTx/bleCtrl/serialCtrl 为 12504/3288/2192/5208/1744 B。

Layer windows remain separate. BL50’s own 598.946417553-second span reports dropD +343, dropL +40 and dropC +0. USBQ’s 600.427256331-second span has zero dDrop/cDrop/expired/cExpired/partial deltas; historical expired=5124/partial=7 do not increase. BLETX’s 600.456470292-second span has 79,882 SDK calls/acceptances and zero errors. These do not erase the host’s 421 missing observations, and losses from different layers must not be added. CPUOWNER ratios over each role’s own span are BLE_TX 15.6668%, TRANSPORT 3.7010%, ASYNC_LOG 12.5184%, SERIAL_TX 10.6578% and SERIAL_CTRL 0.7566%. Different denominators prevent summing them into whole-device CPU; SDK CPU/blocked time is unknown. Complete STK50 records report 8,301,040 bytes observed free and a firmware lifetime minimum of 8,297,160 bytes; largest contiguous block is unknown. Minimum remaining log/transport/bleTx/bleCtrl/serialCtrl stacks are 12504/3288/2192/5208/1744 bytes.

覆盖内未观测 MFAULT/reset/WDT，收尾[只读查询](docs/audit/artifacts/cdc_recovery_acceptance_20261116/ble-data-priority-poststability02-01.summary.json)确认同 ELF/session、CAP8、rev61/boundary51176、BLEGUARD ready1/quarantined0/rejected0/active0/completionKnown1/restartRequired0；正常查询不替代阻塞注入/恢复。关闭后的 7 个原流共 99,070,221 B→10,871,981 B，[归档](docs/audit/artifacts/cdc_recovery_acceptance_20261116/stability-02/archive-index-01.json)逐字节/SHA 回验通过且保留原件，[实际命令](docs/audit/artifacts/cdc_recovery_acceptance_20261116/stability-02-archive-01-command.json) exit 0。该 ELF 的 600 秒录制及 50 次 USB 身份切换已完成，**32 配置仍 NOT_RUN**，精度/硬件限制和吞吐 FAIL 未解除。后续普通 BLE LOG 抑制候选在本记录关闭时尚未烧录，不能提前声称其 HIL 有效。

No MFAULT/reset/WDT is observed within coverage. The closing [read-only query](docs/audit/artifacts/cdc_recovery_acceptance_20261116/ble-data-priority-poststability02-01.summary.json) confirms the same ELF/session, CAP8, rev61/boundary51176 and BLEGUARD ready1/quarantined0/rejected0/active0/completionKnown1/restartRequired0; normal queries do not replace stall injection/recovery. [Seven closed original streams](docs/audit/artifacts/cdc_recovery_acceptance_20261116/stability-02/archive-index-01.json) compress from 99,070,221 to 10,871,981 bytes with byte/SHA roundtrip verification and originals retained; the [recorded command](docs/audit/artifacts/cdc_recovery_acceptance_20261116/stability-02-archive-01-command.json) exits 0. This ELF’s 600-second recording and 50 USB identity transitions are complete; **the 32-configuration matrix is NOT_RUN**, while throughput FAIL and accuracy/hardware limits remain. The later ordinary-BLE-LOG suppression candidate was unflashed when this recording closed; its HIL effectiveness is not established here.

## 2026-11-21 同镜像50次配置身份切换 / 50 configuration-identity transitions on the same image

[实际HIL记录](docs/audit/artifacts/cdc_recovery_acceptance_20261116/mode-switches-01/mode-switches-01.summary.json)在screen16同ELF `8e61aaca…13eb4fc`、同session `391F6A5399682456` 上完成 **50/50，COMPLETE_50_IDENTITY_VERIFIED**。全程14.213275秒，最长单步0.520350秒（第23步MIXED），最后查询恢复CAP8。全部8行，目标为CAP25次、VOLT9次、RES8次、MIXED8次（逐行CVRCVRCV），不是32种行数/模式组合的性能测试。步骤时间是命令/身份/首个匹配帧及收尾查询的观察时间，不是模拟稳定或单ADC转换时间。

The [live HIL record](docs/audit/artifacts/cdc_recovery_acceptance_20261116/mode-switches-01/mode-switches-01.summary.json) completes **50/50 transitions, COMPLETE_50_IDENTITY_VERIFIED**, on screen16’s same ELF `8e61aaca…13eb4fc` and session `391F6A5399682456`. Total elapsed time is 14.213275 seconds; the longest step is 0.520350 seconds (step 23, MIXED). Final CAP8 recovery is query-verified. All configurations use eight rows: CAP25, VOLT9, RES8 and MIXED8 with CVRCVRCV. This is not the 32-row-count/mode performance matrix. Step duration observes commands, identity, the first matching frame and closing query, not analog settling or one ADC conversion.

每次都收到对应ACK/终态，再以原子CONFIG和完整frame CRC/配置域/应用boundary核对，50条首个匹配帧全部通过；共解析51条CRC正确帧，额外1条旧revision排队帧保留原字节并计stale，没有换新标签。fault=0、reset观察为空，**不要求每个测点有效/acquired非零，也不证明精度**。BLE和debug采集同时保留，但本harness的50次判定来自正式USB；未将其称作BLE端50次验收。该次无reset、CAL=SAVE或NVS写入，也未新写RAM rail校准。

Every step receives its matching ACK/terminal result and verifies atomic CONFIG against the complete frame CRC, configuration domains and applied boundary. All 50 first matching frames pass. There are 51 parsed CRC-valid frames in total: one queued old-revision frame remains unchanged and is counted stale rather than relabeled. Fault count is zero and reset observations are empty. **The test does not require every point to be valid or an acquired mask to be nonzero, and does not establish accuracy.** BLE/debug recordings run alongside it, but this harness adjudicates the 50 transitions through formal USB; it is not a BLE-side 50-transition acceptance. No reset, CAL=SAVE, NVS write or new RAM rail calibration is performed in this run.

最后CAP帧seq11789的旧 `gen=3,rid=6` 属于ROWS，同时显式 `mgen=46,mrid=56,rev=56` 对应最新MODE请求；这条实录不能因为旧gen数字小就丢弃。[工具独立审查](docs/audit/artifacts/cdc_recovery_acceptance_20261116/mode-switches-independent-review-01.json)和10个合成回归支持域门控、旧帧保留、终态查询恢复及有界清理；实际50次都是ACK_TERMINAL_CONFIG，不能把合成丢终态/早到帧测试写成现场注入。[5个关闭原流归档](docs/audit/artifacts/cdc_recovery_acceptance_20261116/mode-switches-01/archive-index-01.json)为1,335,958 B→191,295 B，字节/SHA回验通过且原件保留。此限定身份结果不改变screen16吞吐FAIL；后续600秒录制见上节，32配置/精度未验收。

The final CAP frame, seq11789, has legacy `gen=3,rid=6` in the ROWS domain while explicit `mgen=46,mrid=56,rev=56` corresponds to the latest MODE request. Its smaller legacy generation is not a reason to drop it. The [independent tool review](docs/audit/artifacts/cdc_recovery_acceptance_20261116/mode-switches-independent-review-01.json) and ten synthetic regressions support domain gating, retained stale frames, terminal-query recovery and bounded cleanup. All 50 live steps use ACK_TERMINAL_CONFIG; synthetic lost-terminal/early-frame cases are not claimed as hardware injections. [Five closed original streams](docs/audit/artifacts/cdc_recovery_acceptance_20261116/mode-switches-01/archive-index-01.json) compress from 1,335,958 to 191,295 bytes, passing byte/SHA verification with originals retained. This scoped identity result does not change screen16’s throughput FAIL. The later 600-second recording is above; 32-configuration/accuracy acceptance remains incomplete.

## 2026-11-21 screen16：正式 DATA 优先于普通 LOG / screen16: formal DATA before ordinary LOG

本次仍显示 `8a1794c-dirty`，但完整ELF为 **`8e61aacaee859cd5ebe1dff13dff1d9d2c722268bb564837c15b4741613eb4fc`**、binary SHA256=`5f35bfc3e11aab5d6adcd748c7a09ab817ebf561417cf8a477c5ee665143e112`、session=`391F6A5399682456`，与screen15不同。[69入口/目标构建](docs/audit/artifacts/cdc_recovery_acceptance_20261116/ble-data-priority-01-validated-identity.json)通过且187生产输入一致；[烧录](docs/audit/artifacts/cdc_recovery_acceptance_20261116/ble-data-priority-01-flash-usj-result.json)和[完整BUILD/READY](docs/audit/artifacts/cdc_recovery_acceptance_20261116/ble-data-priority-01-boot-verification.json)核验已完成，启动仍2 MHz/2,000检查/CRC0。dirty包含生产修改，后续HEAD不能替代真实镜像身份。

The version still reads `8a1794c-dirty`, but the full ELF is **`8e61aacaee859cd5ebe1dff13dff1d9d2c722268bb564837c15b4741613eb4fc`**, binary SHA256 `5f35bfc3e11aab5d6adcd748c7a09ab817ebf561417cf8a477c5ee665143e112`, session `391F6A5399682456`, distinct from screen15. The [69-entry suite/target build](docs/audit/artifacts/cdc_recovery_acceptance_20261116/ble-data-priority-01-validated-identity.json) pass with 187 matching production inputs; [flashing](docs/audit/artifacts/cdc_recovery_acceptance_20261116/ble-data-priority-01-flash-usj-result.json) and [full BUILD/READY verification](docs/audit/artifacts/cdc_recovery_acceptance_20261116/ble-data-priority-01-boot-verification.json) are complete. Boot still qualifies 2 MHz with zero CRC rejections in 2,000 checks. Dirty includes production changes; later HEAD is not the running image’s identity.

本补丁只改变 `sensorarrayBleReceiveNext` 的记录选择：**保留槽内CTRL/生命周期 → 已排队DATA → 普通诊断LOG**，移除每4条DATA让一次普通LOG的强制穿插。已选中的完整记录保留到分片发送/失败收尾，不在中途抢占或换identity；新CTRL在下一次选择时优先。普通LOG在持续DATA积压时可一直延后，两个独立LOG槽仍有界，满槽拒绝计数；关键生命周期仍走保留优先队列，不当普通LOG丢弃。这是应用descriptor选择，不是修改FreeRTOS任务优先级或保证空中调度；持续CTRL流量也可能延后DATA，不能许诺固定最迟交付时间。

This patch changes only record selection in `sensorarrayBleReceiveNext`: **reserved CTRL/lifecycle → queued DATA → ordinary diagnostic LOG**, removing the forced ordinary-LOG turn after every four DATA records. A selected record retains ownership through fragment sending or failure cleanup; it is not preempted midway or relabeled, and new CTRL wins at the next selection. Ordinary LOG can remain delayed while DATA backlog persists. Its two independent slots stay bounded and count rejected admissions; critical lifecycle records retain their reserved priority queue rather than ordinary-LOG treatment. This is application descriptor selection, not a FreeRTOS priority change or radio scheduling guarantee. Sustained CTRL traffic can also delay DATA, so no fixed latest-delivery time is promised.

[独立审查](docs/audit/artifacts/cdc_recovery_acceptance_20261116/ble-data-priority-independent-review-01.json)核对只改选择逻辑；实际回归包含100次持续DATA选择、2个LOG槽保留/第3次拒绝计数、步骤20/60插入CTRL/priority lifecycle及DATA停止后全部槽释放。既有CRC/receiver/owner/SDK guard/work-budget回归保留。科学采集、两片打包、guard、OTG汇总50帧、health64帧、队列容量和任务优先级均未改。

The [independent review](docs/audit/artifacts/cdc_recovery_acceptance_20261116/ble-data-priority-independent-review-01.json) verifies that only selection logic changed. Executed regressions cover 100 sustained DATA selections, retention of two LOG slots with a counted third-admission rejection, CTRL/priority-lifecycle insertion at steps 20/60, and complete slot drainage after DATA stops. Existing CRC/receiver/ownership/SDK-guard/work-budget regressions remain. Scientific acquisition, two-fragment packing, guard, the OTG 50-frame summary, 64-frame health checks, queue capacities and task priorities are unchanged.

| 指标 / Metric | screen15 | screen16 |
|---|---:|---:|
| Duration | 120.000115322 s | 120.000331701 s |
| Capture / USB / BLE FPS | 63.994784 / 64.024883 / 63.248039 | 64.242834 / 64.122140 / 63.414893 |
| USB trusted / internal missing observations | 7,682 / 0 | 7,695 / 0 |
| BLE trusted / missing observations | 7,589 / 96 | 7,609 / 84 |
| ADC rejected / checked reads | 0 / 82,246 | 0 / 82,428 |
| USB / BLE X12 invalid points | 527 / 513 | 606 / 586 |
| ASYNC_LOG / BLE_TX own-window run ratio | 12.477512% / 15.680669% | 12.515174% / 15.659861% |
| Overall verdict | FAIL_BLE_THROUGHPUT | **FAIL_BLE_THROUGHPUT** |

USB连续性只指seq970…8664这7,695个有序可信帧；USB/BLE公共7,608帧原字节相同。ADC0/82,428来自独立118.803774800秒健康窗（41,328 even+41,100 odd），不是帧数分母。BL50自身119.338832787秒窗dropD+62、dropL+6、dropC+0，与主机缺84不是可相加的独立损失；普通日志的取舍已有实际计数。USBQ自身120.414812269秒窗dDrop/cDrop/expired/cExpired/partial增量均0。录制覆盖内未观测MFAULT/reset/WDT/ASCII CRC/identity错误，CAP8已查询恢复，但**BLE仍未达到采集吞吐**，有限差异不是零丢帧保证。详细窗口/CPU见[判定](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-16/screen-16-verdict.json)和[screen15→16比较](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-16/screen15-screen16-comparison.json)。

USB contiguity refers only to its 7,695 ordered trusted frames across seq970…8664; all 7,608 common USB/BLE frames retain identical bytes. ADC0/82,428 comes from a separate 118.803774800-second health span (41,328 valid-even +41,100 valid-odd), not frame count. BL50’s own 119.338832787-second span reports dropD+62, dropL+6 and dropC+0; these are not independent losses to add to the host’s 84 missing observations. Ordinary-log tradeoffs now have observed counters. USBQ’s separate 120.414812269-second span has zero dDrop/cDrop/expired/cExpired/partial deltas. No MFAULT/reset/WDT/ASCII CRC/identity error was observed within recording coverage, and CAP8 recovery is query-verified. **BLE still does not match acquisition throughput**; the bounded difference is not a zero-loss guarantee. See the [verdict](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-16/screen-16-verdict.json) and [screen15→16 comparison](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-16/screen15-screen16-comparison.json) for exact layer/CPU windows.

[post16只读查询](docs/audit/artifacts/cdc_recovery_acceptance_20261116/ble-data-priority-post16-01.summary.json)再次确认同ELF/session、CAP8，BLEGUARD为ready1/quarantined0/rejected0/active0/completionKnown1/restartRequired0；这是正常查询证据，不替代实机SDK阻塞/恢复注入。14个关闭原流/flash/state共23,818,105 B→2,765,315 B，[归档回验](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-16/archive-index-01.json)通过、原件保留。本ELF后续600秒录制和50次USB身份切换见上节；32配置仍NOT_RUN，旧镜像覆盖不回填，精度与硬件限制继续保留。

The [post16 read-only query](docs/audit/artifacts/cdc_recovery_acceptance_20261116/ble-data-priority-post16-01.summary.json) reconfirms the same ELF/session and CAP8, with BLEGUARD ready1/quarantined0/rejected0/active0/completionKnown1/restartRequired0. This is normal query evidence, not hardware SDK-stall/recovery injection. Fourteen closed original streams/flash/state records compress from 23,818,105 to 2,765,315 bytes and pass [archive verification](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-16/archive-index-01.json), with originals retained. This ELF’s later 600-second recording and 50 USB identity transitions are recorded above. Its 32-configuration matrix remains NOT_RUN, without transferring old-image coverage. Accuracy and hardware limits remain.

## 2026-11-21 screen15：guard 与汇总周期的同版实测 / screen15: hardware observation of guard and summary cadence

[app-only烧录](docs/audit/artifacts/cdc_recovery_acceptance_20261116/ble-guard-cadence-01-flash-usj-result.json)及[完整BUILD/READY核验](docs/audit/artifacts/cdc_recovery_acceptance_20261116/ble-guard-cadence-01-boot-verification.json)已执行；session=`FD6F81721A66C665`，ELF=`48f7352f2f0f637ee4c0290a09f1141e0a2d6e5a7ad12442910b5132c5250116`。它与screen14虽然都显示 `8a1794c-dirty`，但不能混用镜像或结果。启动仍为2 MHz、2,000检查CRC0，69个host入口/目标构建与187输入hash对应此ELF。下节机制已进入本镜像，普通运行和只读guard查询有了实机证据；**未做实机SDK永久阻塞注入或guard触发后的手动恢复验收**。

[App-only flashing](docs/audit/artifacts/cdc_recovery_acceptance_20261116/ble-guard-cadence-01-flash-usj-result.json) and [full BUILD/READY verification](docs/audit/artifacts/cdc_recovery_acceptance_20261116/ble-guard-cadence-01-boot-verification.json) are executed, with session `FD6F81721A66C665` and ELF `48f7352f2f0f637ee4c0290a09f1141e0a2d6e5a7ad12442910b5132c5250116`. It shares the text `8a1794c-dirty` with screen14, but the images and results are distinct. Boot still qualifies 2 MHz with zero CRC rejections in 2,000 checks; the 69-entry host suite/target build and 187 input hashes correspond to this ELF. The mechanisms below are now in this image, with hardware evidence for normal operation and read-only guard queries. **Hardware permanent-SDK-stall injection and manual recovery after guard activation remain untested.**

| 指标 / Metric | screen14 packing | screen15 guard + cadence |
|---|---:|---:|
| Duration | 120.000089831 s | 120.000115322 s |
| Capture / USB / BLE FPS | 66.160374 / 66.174144 / 62.522868 | 63.994784 / 64.024883 / 63.248039 |
| USB trusted / internal missing observations | 7,940 / 0 | 7,682 / 0 |
| BLE trusted / missing observations | 7,502 / 437 | 7,589 / 96 |
| ADC rejected / checked reads | 0 / 84,044 | 0 / 82,246 |
| USB / BLE X12 invalid points | 705 / 657 | 527 / 513 |
| ASYNC_LOG / BLE_TX own-window run ratio | 15.034534% / 14.346032% | 12.477512% / 15.680669% |
| Overall verdict | FAIL_BLE_THROUGHPUT | **FAIL_BLE_THROUGHPUT** |

USB seq1019…8700连续，公共7,587帧USB/BLE原字节相同；ADC为独立118.962395512秒健康窗的41,248 even+40,998 odd读数。BL50自身119.495944540秒窗dropD+78、dropL/C+0，与主机缺96不能相加；USBQ自身120.403522522秒窗dDrop/cDrop/expired/cExpired/partial增量均0。正式/debug覆盖内未观测MFAULT/reset/WDT/ASCII CRC/identity错误，CAP8已查询恢复。完整STK50 free=8,301,116 B、lifetime minimum=8,297,284 B，最大连续块unknown。其余角色/SDK窗口见[screen14→15分层比较](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-15/screen14-screen15-comparison.json)，不同分母不合计为全核CPU或blocked time。

USB seq1019…8700 is contiguous, and all 7,587 common USB/BLE frames retain identical bytes. ADC counts are 41,248 valid-even +40,998 valid-odd reads over a separate 118.962395512-second health span. BL50’s own 119.495944540-second span reports dropD+78 and dropL/C+0; it is not added to the host’s 96 missing observations. USBQ’s separate 120.403522522-second span has zero dDrop/cDrop/expired/cExpired/partial deltas. No MFAULT/reset/WDT/ASCII CRC/identity error was observed within formal/debug coverage, and CAP8 recovery is query-verified. Complete STK50 reports free heap 8,301,116 bytes and lifetime minimum 8,297,284 bytes; largest contiguous block is unknown. Other role/SDK windows are in the [screen14→15 layer comparison](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-15/screen14-screen15-comparison.json); independent denominators are not summed into whole-core CPU or blocked time.

[原debug字节/接收offset检查](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-15/debug-cadence-observation-01.json)实际执行：screen15整个observer内168条完整SF50均n=50，其中严格按正式capture接收时间窗选入 **154条**，也全n=50；screen14对应capture有476条n=15/30/45汇总。168不是capture计数，SF50频率也不是科学帧率。logger占用下降与合并汇总边界一致，但guard和cadence同时改变；capture较screen14下降 **3.273244%**，没有修改采集限速设置，下降原因不能由这一个对照单独归因。BLE更接近capture仍有实收缺口，不能据此自设放宽门槛或宣布吞吐通过。

The [executed original-debug-byte/receipt-offset check](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-15/debug-cadence-observation-01.json) finds 168 complete SF50 lines across the whole screen15 observer, all n=50. Exactly **154 lines** fall inside the formal capture receipt window, also all n=50; screen14 has 476 capture-window summaries with n=15/30/45. The 168 count is not capture-only, and SF50 frequency is not scientific frame rate. Reduced logger runtime is consistent with merged summary boundaries, but guard and cadence changed together. Capture is **3.273244% lower** than screen14 despite no acquisition-rate limit change; this single comparison does not isolate the cause. BLE is closer to capture but still has missing observations, so no relaxed acceptance threshold or throughput pass is invented.

采集关闭后的[独立只读查询](docs/audit/artifacts/cdc_recovery_acceptance_20261116/ble-guard-cadence-post15-01.summary.json)在同session记录 `ready=1,quarantined=0,rejected=0,active=0,completionKnown=1,restartRequired=0`；这仅说明该观察时点未锁存guard故障，不是触发/恢复测试。其ageUs=65,820,515为上次调用开始后的年龄，active=0时不能误判为65秒卡顿。14个关闭原流/flash/state记录20,954,730 B→2,397,156 B，[归档](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-15/archive-index-01.json)逐字节/SHA回验通过且原件保留。新ELF600秒/32配置NOT_RUN，绝对精度和实机阻塞恢复仍待验收。

An [independent read-only query after capture closure](docs/audit/artifacts/cdc_recovery_acceptance_20261116/ble-guard-cadence-post15-01.summary.json) records `ready=1,quarantined=0,rejected=0,active=0,completionKnown=1,restartRequired=0` in the same session. It shows no latched guard fault at that observation, not an activation/recovery test. Its ageUs=65,820,515 is age since the last call began; active=0 means it must not be interpreted as a 65-second stall. Fourteen closed original streams/flash/state records compress from 20,954,730 to 2,397,156 bytes and pass [byte/SHA archive verification](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-15/archive-index-01.json), with originals retained. This ELF’s 600-second/32-configuration runs are NOT_RUN; absolute accuracy and hardware stall recovery remain unaccepted.

## 2026-11-21 BLE SDK 等待监管与 OTG 调试汇总候选 / BLE SDK wait supervision and OTG debug-summary candidate

该候选已实际通过69个host入口及目标构建，187个生产输入逐hash一致；[验证身份](docs/audit/artifacts/cdc_recovery_acceptance_20261116/ble-guard-cadence-01-validated-identity.json)记录app 1,611,184 B、binary SHA256=`80687a88b442e6ea23dd6e76832db36259c1aac77632267b22ba6d2748eb1cc7`、ELF SHA256=`48f7352f2f0f637ee4c0290a09f1141e0a2d6e5a7ad12442910b5132c5250116`。版本文本仍是 **`8a1794c-dirty`，与screen14相同，但镜像不同**；必须按完整ELF区分。此处记录本地验证机制；上节screen15补充该ELF的独立正常运行HIL，实机SDK阻塞注入/恢复仍未验证。不迁移screen14的吞吐/零CRC或更早600秒数据，不声称全项目不存在P0/P1。

This candidate passes the executed 69-entry host suite and target build, with identical hashes for 187 production inputs. The [validated identity](docs/audit/artifacts/cdc_recovery_acceptance_20261116/ble-guard-cadence-01-validated-identity.json) records app size 1,611,184 bytes, binary SHA256 `80687a88b442e6ea23dd6e76832db36259c1aac77632267b22ba6d2748eb1cc7`, and ELF SHA256 `48f7352f2f0f637ee4c0290a09f1141e0a2d6e5a7ad12442910b5132c5250116`. The version text remains **`8a1794c-dirty`, the same as screen14, but this is a different image**; distinguish it by full ELF hash. This section records locally validated mechanisms; screen15 above adds separate normal-operation HIL for this ELF. Hardware SDK-stall injection/recovery remains unverified. Screen14 throughput/zero-CRC results and older 600-second evidence are not transferred, and no whole-project absence of P0/P1 defects is claimed.

本轮新增 `sensorarrayBleWatchdog.c` 使用 `ESP_TIMER_TASK` 每100,000 µs观察一次SDK调用的active标记；调用从原子准入标记起达到500,000 µs时，锁存quarantine及callId/startUs/detectedUs。SDK迟于期限返回也在清active前检查，避免恰好漏过定时器poll。锁存后拒绝新的BLE准入/发送，重连不会清除；恢复为已有USB控制通路上的显式 `RESTART`，不新增自动重启循环。调用仍未返回时，原slot/缓冲/任务保持所有权，不删除任务、不撤销SDK操作、不复用对象；之后真实返回可更新completionKnown，但不会解除quarantine。初始化claim/result独立于可清零状态，部分创建失败后同boot不重复创建timer/task/SDK对象。

New `sensorarrayBleWatchdog.c` uses `ESP_TIMER_TASK` to observe the active SDK-call marker every 100,000 µs. At 500,000 µs from atomic admission marking it latches quarantine plus callId/startUs/detectedUs. A late SDK return is checked before clearing active, so a return between timer polls cannot evade the deadline. Quarantine rejects further BLE admission/sends and survives reconnect. Recovery uses the existing explicit `RESTART` through USB control, with no new automatic reboot loop. Until the call actually returns, its slot/buffer/task remain owned: no task deletion, SDK cancellation or object reuse. A real later return updates completionKnown without clearing quarantine. Initialization claim/result live outside resettable state, preventing repeated creation of timer/task/SDK objects after partial startup failure in the same boot.

这是独立观察与隔离，**不是让SDK调用保证在500 ms内返回**。定时器依赖任务调度，100 ms是poll周期而非硬实时最大延迟；暂停、idle或等待CONF不等于active SDK调用超时。新增只读 `BLEGUARD?` 用有效 `BLEGUARD,v=1` 回复探测支持，不扩展PROTO列表；字段及兼容边界见[Host契约](docs/integration/firmware_host_contract.md)。`rejected`只统计真实走到BLE准入/发送的guard拒绝；`sensorarrayTransportQueue` 无可用网络sink时原有的ESP_OK早退不会增加它，测试输出 `transportNoNetworkSink` 只是fixture计数。既有driver/SDK/主机完整接收仍分层。

This is independent observation and containment, **not a guarantee that the SDK call returns within 500 ms**. The timer depends on task scheduling; its 100 ms period is not a hard maximum detection latency. Idle, pause and CONF waiting are not active SDK-call timeouts. New read-only `BLEGUARD?` uses a valid `BLEGUARD,v=1` reply as the support probe without extending the PROTO list; the [host contract](docs/integration/firmware_host_contract.md) defines fields and limits. `rejected` counts only guard rejections actually reached at BLE admission/send paths. The existing `sensorarrayTransportQueue` early ESP_OK return with no enabled network sink does not increment it, and test output `transportNoNetworkSink` is only a fixture counter. Driver, SDK and complete host receipt remain separate layers.

OTG默认debug汇总周期改为归一化网络汇总周期：当前config=50帧，config=0时用20帧；旧Serial/JTAG仍15帧，`ADSDBG=1`细节模式不变。它减少15/30/45帧与50帧边界重叠格式化，完整network aggregate和summary状态保留；正式科学DATA继续每帧，ADS寄存器健康回读仍64帧，采集/CRC/PGA/invalid策略未改。[本地差分验证](docs/audit/artifacts/cdc_recovery_acceptance_20261116/otg-debug-cadence-review-01.json)实际覆盖12个配置、32个场景、4,832帧，same-input网络汇总字节及状态一致；实机USB计数字段可因输出量改变而自然变化，不能要求它们静态相同。

The OTG default debug-summary cadence now follows the normalized network summary period: 50 frames in the current configuration, or 20 when configured as zero. Legacy Serial/JTAG stays at 15 frames, and detailed `ADSDBG=1` behavior is unchanged. This avoids overlapping formatting at frames 15/30/45 and the 50-frame boundary while preserving the complete network aggregate and summary state. Formal scientific DATA remains per-frame; ADS register health readback stays at 64 frames, with acquisition/CRC/PGA/invalid policies unchanged. Executed [local differential validation](docs/audit/artifacts/cdc_recovery_acceptance_20261116/otg-debug-cadence-review-01.json) covers 12 configurations, 32 scenarios and 4,832 frames with identical same-input network aggregate bytes/state. Live USB counter fields can naturally change as output volume changes; their values are not claimed to remain identical.

[独立guard审查](docs/audit/artifacts/cdc_recovery_acceptance_20261116/ble-send-guard-independent-review-01.json)核对暂停在SDK内部的owner、499999/500000 µs边界、迟返成功/失败、callId回绕、重连锁存和初始化失败；该补丁范围内未发现阻塞性P0/P1。[汇总审查](docs/audit/artifacts/cdc_recovery_acceptance_20261116/otg-debug-cadence-review-01.json)保留最初测试stub编译失败及修正履历。以上为host/源码证据；普通USB查询和性能已有上节screen15观察，实际SDK卡顿隔离及故障后的手动恢复仍需后续实机核验，不能用构建通过替代HIL。

The [independent guard review](docs/audit/artifacts/cdc_recovery_acceptance_20261116/ble-send-guard-independent-review-01.json) checks an owner suspended inside the SDK, the 499999/500000 µs boundary, late success/failure, callId wrap, reconnect latching and initialization failures; it identifies no blocking P0/P1 within these patches. The [summary review](docs/audit/artifacts/cdc_recovery_acceptance_20261116/otg-debug-cadence-review-01.json) preserves the initial test-stub compilation failure and its correction. These are host/source results. Normal USB queries and performance now have the screen15 observation above; real SDK-stall containment and manual recovery after a fault still require hardware verification. Build success is not HIL.

## 2026-11-21 screen14：保留两片数量的 DATA 打包 / screen14: DATA packing with the same two-fragment count

本节保留screen14采集时的实现状态；后续guard候选的本地验证见上节，不能回填成screen14已包含guard。 / This section preserves implementation status at screen14 capture. The later guard candidate’s local validation above does not make guard part of the screen14 image.

实际镜像 `8a1794c-dirty`、session=`2036380CE3845BDB`，ELF SHA256=`efa5b2f980ae1739afe54cc56f90080464eb3622013e4081bc4c510f2cb8a8f4`，binary SHA256=`af100132d57e732264ad1ac7b2632ce72a045f9399f09d548399124effbc1e4c`。这次 dirty **包含未提交的生产打包改动**，不能只解释为文档；[实际68入口/构建](docs/audit/artifacts/cdc_recovery_acceptance_20261116/ble-pack-01-validated-identity.json)记录185个相同生产输入，[启动身份](docs/audit/artifacts/cdc_recovery_acceptance_20261116/ble-pack-01-boot-verification.json)确认完整 BUILD/READY、2 MHz、2,000检查 CRC0。后续正在实现的 SDK-wait guard 不属于这个镜像，仍 **WORK_IN_PROGRESS/未烧录**。

The actual image is `8a1794c-dirty`, session `2036380CE3845BDB`, ELF SHA256 `efa5b2f980ae1739afe54cc56f90080464eb3622013e4081bc4c510f2cb8a8f4`, binary SHA256 `af100132d57e732264ad1ac7b2632ce72a045f9399f09d548399124effbc1e4c`. This dirty build **includes uncommitted production packing changes**, not only documentation. The [executed 68-entry suite/build](docs/audit/artifacts/cdc_recovery_acceptance_20261116/ble-pack-01-validated-identity.json) records 185 matching production inputs. [Boot verification](docs/audit/artifacts/cdc_recovery_acceptance_20261116/ble-pack-01-boot-verification.json) confirms full BUILD/READY and 2 MHz qualification with zero CRC rejections in 2,000 checks. The SDK-wait guard under development is not part of this image; it remains **WORK_IN_PROGRESS and unflashed**.

本轮新增 `sensorarrayBleTwoFragmentFirstChunk`：只在原 planner 已选择2片、DATA通道、单包ATT上限512 B且两片都能满足目标时，将首片限制为495 B、次片≤244 B；其它情况保留原均衡分片。原最小片数 planner、科学payload、G envelope字段/CRC、连接generation、owner生命周期、队列、优先级和工作预算均不变。[独立审查](docs/audit/artifacts/cdc_recovery_acceptance_20261116/ble-pack-independent-review-01.json)覆盖35,840几何用例、1,921实际发送、3取消/失败路径及36个实际C→receiver回环，ASan/UBSan通过；SDK无限等待监管P1仍未由此修复。

The newly added `sensorarrayBleTwoFragmentFirstChunk` applies only when the original planner selected two fragments, the channel is DATA, the ATT packet cap is 512 bytes, and both target bounds fit. It targets a 495-byte first value and a second value no larger than 244 bytes; other cases retain balanced splitting. The minimum-count planner, scientific payload, G-envelope fields/CRC, connection generation, ownership, queues, priorities and work budgets remain unchanged. The [independent review](docs/audit/artifacts/cdc_recovery_acceptance_20261116/ble-pack-independent-review-01.json) covers 35,840 geometry cases, 1,921 actual sends, three cancellation/failure paths and 36 actual-C-to-receiver roundtrips, with ASan/UBSan passing. This change does not fix the SDK infinite-wait supervision P1.

| 指标 / Metric | screen13 balanced | screen14 packing |
|---|---:|---:|
| Duration | 120.000081842 s | 120.000089831 s |
| Capture / USB / BLE FPS | 65.789422 / 65.818399 / 61.504531 | 66.160374 / 66.174144 / 62.522868 |
| USB trusted / internal missing observations | 7,898 / 0 | 7,940 / 0 |
| BLE trusted / missing observations | 7,380 / 516 | 7,502 / 437 |
| ADC rejected / checked reads | 0 / 84,604 | 0 / 84,044 |
| USB / BLE X12 invalid points | 600 / 554 | 705 / 657 |
| Overall verdict | FAIL_BLE_THROUGHPUT | **FAIL_BLE_THROUGHPUT** |

[实际执行的通知几何分析](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-14/recorded-geometry-01.json)直接检查原notification字节：screen14全部7,502个完整DATA message仍为2片，ATT value为 **495 B +154–165 B**，重组CRC32全部通过；screen13的7,380个完整message为约330–335 B的均衡两片。每个窗口另有1个不完整group单独报告，不补造缺失片。若假设有效LL payload为251 B，含L2CAP/ATT开销的算术模型为4→3 PDU；**对端有效DLE和实际空中分包仍unknown**，模型不能证明吞吐增益的独立因果。BLE实收有所提高，仍未跟上采集。

The [executed notification-geometry analysis](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-14/recorded-geometry-01.json) reads original notification bytes: all 7,502 complete screen14 DATA messages still use two fragments, with ATT values **495 bytes +154–165 bytes**, and every reassembled CRC32 passes. Screen13’s 7,380 complete messages use balanced values around 330–335 bytes. Each window also has one separately reported incomplete group; no missing fragment is invented. Assuming an effective 251-byte LL payload, the L2CAP/ATT arithmetic model changes four PDUs to three. **Effective peer DLE and actual over-air segmentation remain unknown**; that model does not establish an independent cause of the measured gain. BLE receipt improves but still trails capture.

另有实际对照混杂：[消息长度复核](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-14/recorded-size-confounder-01.json)确认 screen13 是较长运行时间的旧boot，7位seq的消息为601–609 B（众数608 B，5,093条）；screen14新boot的4位seq消息为591–600 B（众数599 B，5,135条）。逻辑消息本身缩短，因此约1 FPS的BLE增益不能全部归因打包。两组均保留实际payload，未改短时间戳/序号或协议字段来制造结果。

A measured comparison confounder also remains: the [message-size check](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-14/recorded-size-confounder-01.json) finds that screen13’s older boot and seven-digit sequences produce 601–609-byte messages (mode 608 bytes, 5,093 messages), while screen14’s new boot and four-digit sequences produce 591–600-byte messages (mode 599 bytes, 5,135 messages). The logical messages are shorter, so the approximately 1 FPS BLE increase cannot be attributed entirely to packing. Both retain actual payloads; timestamps, sequences and protocol fields were not shortened to manufacture the result.

USB seq1140…9079连续，共同7,501帧原字节一致；ADC0/84,044属于独立119.004731837秒健康窗口（42,678 even+41,366 odd）。BL50自身119.911471595秒窗dropD+398，不能与主机缺437相加。录制范围未观测MFAULT/reset/WDT/ASCII CRC/identity错误，CAP8已查询恢复；完整STK50 free=8,301,236 B、lifetime minimum=8,297,924 B，最大连续块unknown。分层和CPU见[判定](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-14/screen-14-verdict.json)及[screen13→14比较](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-14/screen13-screen14-comparison.json)。14个关闭原流/flash日志共23,177,227 B→2,658,949 B，[归档逐字节/SHA回验](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-14/archive-index-01.json)通过、原件保留。600秒/32配置仍NOT_RUN；不将有限改善、旧镜像结果或零CRC观察当完整验收。

USB seq1140…9079 is contiguous, and all 7,501 common frames retain identical bytes. ADC0/84,044 belongs to a separate 119.004731837-second health span (42,678 valid-even +41,366 valid-odd). BL50 reports dropD+398 over its own 119.911471595-second span; it is not added to the host’s 437 missing observations. No MFAULT/reset/WDT/ASCII CRC/identity error was observed within recording coverage, and CAP8 recovery is query-verified. Complete STK50 reports free heap 8,301,236 bytes and lifetime minimum 8,297,924 bytes; largest contiguous block remains unknown. See the [verdict](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-14/screen-14-verdict.json) and [screen13→14 comparison](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-14/screen13-screen14-comparison.json) for layers and CPU. Fourteen closed original streams/flash logs compress from 23,177,227 to 2,658,949 bytes and pass [byte/SHA archive verification](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-14/archive-index-01.json), with originals retained. The 600-second/32-configuration runs remain NOT_RUN; bounded improvement, old-image results and observed zero CRC do not constitute full acceptance.

## 2026-11-21 screen13：重启后的同版复测 / screen13: same-image observation after restart

本次先用只读查询核对完整 BUILD/ELF、READY 与新 boot session，再运行 USB+BLE+debug 的独立 120 秒窗口；没有把旧 session 的计数或序号接到新 session。镜像仍为 `53005f0-dirty`，ELF SHA256=`ca6a90ffe1c2b72344e4b1a54f44169c751b39217f4da0ee5f3bdbfabe323d90`，session=`720D1B47362F4E7E`。启动选中 2 MHz、2,000 次检查 CRC 0，见[本次身份核对](docs/audit/artifacts/cdc_recovery_acceptance_20261116/resume-nov21-boot-verification-01.json)；这不是新候选构建或重新烧录证明。

Read-only queries first verified full BUILD/ELF, READY and the new boot session, followed by a separate 120-second USB+BLE+debug recording. Counters and sequences were not joined across sessions. The image remains `53005f0-dirty`, ELF SHA256 `ca6a90ffe1c2b72344e4b1a54f44169c751b39217f4da0ee5f3bdbfabe323d90`, session `720D1B47362F4E7E`. Boot selected 2 MHz with zero CRC rejections in 2,000 checks; see the [new identity verification](docs/audit/artifacts/cdc_recovery_acceptance_20261116/resume-nov21-boot-verification-01.json). This is not evidence of a new candidate build or another flash operation.

| 指标 / Metric | screen12 | screen13 |
|---|---:|---:|
| Duration | 120.000092493 s | 120.000081842 s |
| Capture / USB / BLE FPS | 65.909847 / 65.922827 / 61.480926 | 65.789422 / 65.818399 / 61.504531 |
| USB trusted / internal missing observations | 7,911 / 0 | 7,898 / 0 |
| BLE trusted / missing observations | 7,377 / 537 | 7,380 / 516 |
| ADC rejected / checked reads | 0 / 84,791 | 0 / 84,604 |
| USB / BLE X12 invalid points | 682 / 616 | 600 / 554 |
| Overall verdict | FAIL_BLE_THROUGHPUT | **FAIL_BLE_THROUGHPUT** |

screen13 的 USB seq1511513…1519410 连续，USB/BLE 共同的 7,378 帧原字节一致；ASCII CRC、identity、wire-contract 错误均未观察到。ADC 分母是独立 119.633505836 秒健康窗口的 42,766 even + 41,838 odd 检查，不是帧数。BL50 自身 119.723002217 秒窗 dropD+455，不能与主机缺516相加；USBQ 自身 120.439432853 秒窗 dDrop/cDrop/expired/cExpired/partial 增量均0。正式/debug范围未观测 MFAULT/reset/WDT，CAP8 已查询恢复。最大连续 heap 块仍 unknown；完整 STK50 free=8,301,028 B、lifetime minimum=8,297,800 B。详见[判定](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-13/screen-13-verdict.json)和[分层/CPU比较](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-13/screen12-screen13-comparison.json)。这次重复结果仍显示 BLE 吞吐差距，不能据有限观察宣布通过；本版 600 秒、32 配置及绝对精度仍未验收。

Screen13’s USB sequence 1511513…1519410 is internally contiguous, and all 7,378 common USB/BLE frames have identical original bytes. No ASCII CRC, identity or wire-contract error was observed. The ADC denominator is 42,766 valid-even + 41,838 valid-odd checks over a separate 119.633505836-second health span, not frame count. BL50 reports dropD+455 over its own 119.723002217-second span; it is not added to the host’s 516 missing observations. USBQ’s separate 120.439432853-second span has zero dDrop/cDrop/expired/cExpired/partial deltas. No MFAULT/reset/WDT was observed within formal/debug coverage, and CAP8 recovery is query-verified. Largest contiguous heap block remains unknown; complete STK50 reports free heap 8,301,028 bytes and lifetime minimum 8,297,800 bytes. See the [verdict](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-13/screen-13-verdict.json) and [layer/CPU comparison](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-13/screen12-screen13-comparison.json). This repeated result still shows a BLE throughput shortfall; bounded observation is not acceptance. This image’s 600-second, 32-configuration and absolute-accuracy acceptance remain incomplete.

**追加发现 P1，待实现/验证：** `sensorarrayBleSendGattValue` 的外层期限无法约束 `esp_ble_gatts_send_indicate` 内部 BTC 队列的 `portMAX_DELAY`。若 SDK 调用不返回，同一个 BLE owner 无法继续 DATA/CTRL；已公开的 `sdkCallActive` 只提供观察，尚未形成独立监管与有限隔离/升级闭环。当前 screen13 没有复现卡死，因此不能把它当作已证实的吞吐根因；但也不能声称所有事务有界完成。修复需由调用者之外监督真实进度、保留仍被 SDK 使用的槽位，并采用已有预算约束恢复；超时不等于取消或可复用缓冲。既有 [SDK 源码证据](docs/audit/artifacts/cdc_recovery_acceptance_20261116/open-items-review-01.json) 保留，新的 P1 认定补充其旧窄范围审查，不篡改旧记录。

**Additional P1 finding, implementation/validation pending:** the outer deadline in `sensorarrayBleSendGattValue` cannot bound the BTC queue’s `portMAX_DELAY` inside `esp_ble_gatts_send_indicate`. If the SDK call never returns, the same BLE owner cannot continue DATA/CTRL. Published `sdkCallActive` is observation, not an independent finite containment/escalation mechanism. Screen13 did not reproduce a stall, so this is not a demonstrated cause of its throughput gap; it also prevents an unconditional bounded-transaction claim. The fix needs progress supervision outside the caller, retained ownership of slots still used by the SDK, and the existing budgeted recovery path. Timeout is not cancellation or permission to reuse buffers. Existing [SDK source evidence](docs/audit/artifacts/cdc_recovery_acceptance_20261116/open-items-review-01.json) remains intact; this P1 classification supplements the older narrow review rather than rewriting it.

7 个已关闭的 screen13/只读原始流共 22,237,822 B，压缩为 2,508,964 B，全部逐字节/SHA 回验且保留原件，见[归档索引](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-13/archive-index-01.json)与[实际执行命令](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-13-archive-01-command.json)。三个采集器和只读查询均已正常退出后才归档；未读取活动流、修改科学数据或做硬件操作。

Seven closed screen13/read-only recording streams total 22,237,822 bytes, compressed to 2,508,964 bytes with byte/SHA roundtrip verification and originals retained. See the [archive index](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-13/archive-index-01.json) and [executed command](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-13-archive-01-command.json). Archiving followed normal exits of all three collectors and the read-only probe; it did not read active streams, modify scientific data or access hardware.

## 2026-11-16 screen12：实际 MTU517 与剩余吞吐差距 / screen12: negotiated MTU517 and remaining throughput gap

烧录源码 `53005f0042a7a7694338f92c9b86ab3276db668a`，实际 BUILD=`53005f0-dirty`、session=`BB7AAF6430B1FAD3`；app 1,592,032 B，binary SHA256=`d53347bb7e28bbef833d19c9651ef492ccf79701fd2dad9e18471d47f1864bff`，ELF SHA256=`ca6a90ffe1c2b72344e4b1a54f44169c751b39217f4da0ee5f3bdbfabe323d90`。68/68 host 入口与实际目标构建通过，185 个生产/配置输入相同，详见[构建/测试身份](docs/audit/artifacts/cdc_recovery_acceptance_20261116/ble-mtu517-01-validated-identity.json)与[启动核验](docs/audit/artifacts/cdc_recovery_acceptance_20261116/ble-mtu517-01-boot-verification.json)。保留实际 `-dirty` 名称；构建期差异为并发文档修改，不能拿后续 HEAD 替换板上身份。

The flashed source is `53005f0042a7a7694338f92c9b86ab3276db668a`, actual BUILD `53005f0-dirty`, session `BB7AAF6430B1FAD3`: app 1,592,032 bytes, binary SHA256 `d53347bb7e28bbef833d19c9651ef492ccf79701fd2dad9e18471d47f1864bff`, ELF SHA256 `ca6a90ffe1c2b72344e4b1a54f44169c751b39217f4da0ee5f3bdbfabe323d90`. All 68 host entries and the target build pass with 185 identical production/configuration inputs; see the [build/test identity](docs/audit/artifacts/cdc_recovery_acceptance_20261116/ble-mtu517-01-validated-identity.json) and [boot verification](docs/audit/artifacts/cdc_recovery_acceptance_20261116/ble-mtu517-01-boot-verification.json). The real `-dirty` name is retained: concurrent documentation edits explain it, and later HEAD is not substituted for the running identity.

唯一生产改动是 `sensorarrayTransportInit` 请求 MTU247→517。`sensorarrayBleSendPayload` 仍以每条消息捕获的连接状态限制单包为 `min(协商MTU−3,512)`；消息装得下就直接发送，装不下仍按原 G envelope 分片，2304 B 消息上限、CRC、identity、队列、优先级和截止时间不变。启动请求值不是协商结果：[BLELINK 实录](docs/audit/artifacts/cdc_recovery_acceptance_20261116/ble-mtu517-link-01.summary.json) 确认 MTU517、双向 PHY2、interval=6×1.25 ms、latency0；本地 DLE251 已观察，但 `dlePeerKnown=0`，不能据此宣称空中分包或对端有效长度已测得。实际发送路径的 511/512/513/598/2304 B、重连 517→23→247、旧 generation 拒绝及原接收器重组回归均已执行，见[候选回归](docs/audit/artifacts/cdc_recovery_acceptance_20261116/ble-mtu517-candidate-01/scope-tests.json)；该预烧录记录的 NOT_RUN 项由后续独立 HIL 记录补充，不改写历史记录。

The only production change requests MTU517 instead of247 in `sensorarrayTransportInit`. `sensorarrayBleSendPayload` still caps each packet at `min(negotiated MTU−3,512)` using a per-message connection snapshot. A fitting message is sent directly; otherwise the existing G envelope fragments it. The 2304-byte message limit, CRC, identity, queues, priorities and deadlines are unchanged. A requested MTU is not negotiation evidence: the [BLELINK recording](docs/audit/artifacts/cdc_recovery_acceptance_20261116/ble-mtu517-link-01.summary.json) confirms MTU517, bidirectional PHY2, interval6×1.25 ms and latency0. Local DLE251 is observed, but `dlePeerKnown=0`; effective peer length and over-air segmentation remain unmeasured. Executed [candidate regressions](docs/audit/artifacts/cdc_recovery_acceptance_20261116/ble-mtu517-candidate-01/scope-tests.json) cover actual sending at511/512/513/598/2304 bytes, reconnect517→23→247, old-generation rejection and real-receiver reassembly. Later independent HIL evidence supplements the pre-flash record’s NOT_RUN items without rewriting that record.

| 指标 / Metric | screen11 MTU247 | screen12 MTU517 |
|---|---:|---:|
| Duration | 120.000070722 s | 120.000092493 s |
| Capture / USB / BLE FPS | 64.376134 / 64.384201 / 47.080922 | 65.909847 / 65.922827 / 61.480926 |
| USB trusted / internal missing observations | 7,724 / 0 | 7,911 / 0 |
| BLE trusted / missing observations | 5,650 / 2,077 | 7,377 / 537 |
| ADC rejected / checked reads | 0 / 82,911 | 0 / 84,791 |
| USB / BLE X12 invalid points | 670 / 434 | 682 / 616 |
| ASCII CRC / identity / wire-contract errors | all 0 | all 0 |
| Overall verdict | FAIL_BLE_THROUGHPUT | **FAIL_BLE_THROUGHPUT** |

USB 连续性由同 identity 的 7,911 有序可信帧、seq1963…9873 和空 gap 列表确认；USB/BLE 共同的 7,375 帧原字节一致。ADC 的 0/84,791 来自独立119.432042502秒健康窗口（43,199 even+41,592 odd），不是用帧数做分母；modeEntryFrameCount=0 属于本次观察，不覆盖旧600秒的非零累计计数案例。BL50 独立119.834741003秒窗 `dropD+479`、`dropL/C+0`，不能与主机缺537相加。USBQ 独立120.466072094秒窗 dDrop/cDrop/expired/cExpired/partial 增量均0。正式/debug覆盖内未观察到 MFAULT/reset/WDT，已查询恢复 CAP8；无效点、2.5% 启动资格门限与科学 CRC 拒绝规则保留。

USB contiguity follows from 7,911 ordered trusted frames under one identity, seq1963…9873 and an empty gap list; all 7,375 common USB/BLE frames retain identical original bytes. ADC0/84,791 comes from a separate119.432042502-second health span (43,199 valid-even+41,592 valid-odd), not a frame-count denominator. Its modeEntryFrameCount=0 does not replace the older600-second accumulated-counter case. BL50’s independent119.834741003-second span reports dropD+479 and dropL/C+0; these are not added to the host’s537 missing observations. USBQ’s separate120.466072094-second span has zero dDrop/cDrop/expired/cExpired/partial deltas. No MFAULT/reset/WDT was observed within formal/debug coverage; CAP8 recovery is query-verified. Invalid points, the2.5% boot-qualification threshold and scientific CRC rejection remain unchanged.

CPU owner 各自窗口比率为 BLE_TX14.140298%、TRANSPORT3.611761%、ASYNC_LOG14.983642%、SERIAL_TX10.239330%、SERIAL_CTRL0.762656%；BLE_CTRL/WIFI_CTRL unknown，不合计成全核CPU。BLESVC 已完成15,591次，wall=66,128,458 µs，约4,241.45 µs/次；started15,592且结束快照active=1说明窗口跨过一个正在执行的调用，不能称丢失调用；SDK wall、CPU、blocked 与 PC接收仍不同。完整STK50 observed free=8,301,420 B、lifetime minimum=8,297,776 B，最大连续块 unknown。见[判定](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-12/screen-12-verdict.json)和[逐层/CPU比较](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-12/screen11-screen12-layer-cpu-comparison.json)。这是一组有限对照，BLE仍比采集低，**新镜像600秒和32配置NOT_RUN，绝对精度未验收**；不把旧600秒迁移为本版通过。

Per-owner ratios over their own windows are BLE_TX14.140298%, TRANSPORT3.611761%, ASYNC_LOG14.983642%, SERIAL_TX10.239330% and SERIAL_CTRL0.762656%; BLE_CTRL/WIFI_CTRL remain unknown and are not summed into whole-core CPU. BLESVC completes15,591 calls with66,128,458 µs wall time, about4,241.45 µs/call. Started15,592 and active=1 at the ending snapshot explain one in-flight boundary crossing, not a lost call; SDK wall, CPU, blocked time and PC receipt remain distinct. Complete STK50 reports observed free heap8,301,420 bytes and lifetime minimum8,297,776 bytes; largest contiguous block is unknown. See the [verdict](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-12/screen-12-verdict.json) and [layer/CPU comparison](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-12/screen11-screen12-layer-cpu-comparison.json). This bounded comparison still leaves BLE below capture: **the new image’s600-second and32-configuration runs remain NOT_RUN, and absolute accuracy is unaccepted**. The older600-second result is not transferred to this version.

## 2026-11-16 screen11：CRC32/planner 优化与明确的低电平等待 / screen11: CRC32/planner optimization and explicit low hold

screen11 当时烧录源码为 `caa90829f34ffeda76dfe681e8e20bb29de4627c`，BUILD=`caa9082-dirty`；app 1,592,032 B，binary SHA256=`65c4d3af1ca56cb2ae60e4972ad40fc4beb27f069c5b878aaa467b3fd85f28d4`，完整 ELF SHA256=`b880b7ea8680ae34b68167d3d75b59b06e12b134afc00ef114e883d7dd8b9fcd`。`-dirty` 来自构建期间并发文档修改，不改写已烧镜像名称；后续 `849d96c` 只修测试提取器。最终 68/68 host 入口通过，build01 与 test02 的 185 个生产输入逐 SHA 相同；首轮 67 PASS/1 提取器编译 FAIL 原件保留。完整 BUILD/READY 与 session=`5AE787A210773D6D` 已独立核验，见[构建/测试身份](docs/audit/artifacts/cdc_recovery_acceptance_20261116/transport-cpu-01-validated-identity.json)与[启动核对](docs/audit/artifacts/cdc_recovery_acceptance_20261116/transport-cpu-01-boot-verification.json)。

The source flashed for screen11 was `caa90829f34ffeda76dfe681e8e20bb29de4627c`, BUILD `caa9082-dirty`: app 1,592,032 bytes, binary SHA256 `65c4d3af1ca56cb2ae60e4972ad40fc4beb27f069c5b878aaa467b3fd85f28d4`, full ELF SHA256 `b880b7ea8680ae34b68167d3d75b59b06e12b134afc00ef114e883d7dd8b9fcd`. Concurrent documentation edits explain the build's `-dirty` suffix; the historical image is not renamed. Follow-up `849d96c` changes only a test extractor. The final host run passes 68/68 entries, with identical SHA hashes for 185 production inputs in build01/test02. The first run's 67 PASS/one extractor-compilation FAIL is preserved. Full BUILD/READY and session `5AE787A210773D6D` are independently verified in the [build/test identity](docs/audit/artifacts/cdc_recovery_acceptance_20261116/transport-cpu-01-validated-identity.json) and [boot verification](docs/audit/artifacts/cdc_recovery_acceptance_20261116/transport-cpu-01-boot-verification.json).

`4edc99a` 将科学文本/BLE 包的逐位 CRC32 换为同一 IEEE 结果的 `esp_rom_crc32_le`，seed=0，继续保留 encoder self-check、BLE 入队 CRC 和出队复算；ADC 的 SPI CRC8 不变。分片 planner 只将试算 header 的重复 `snprintf` 换为精确十进制位数算术；真实发送仍由原 formatter 生成，分片选择/包字节/2304 B 上限/队列/截止时间不变。实际 host 差分、ASan/UBSan 与最终 ELF ROM call-site 核验通过；不是靠去掉校验提速。

`4edc99a` replaces bitwise CRC32 in scientific text/BLE packets with `esp_rom_crc32_le` producing the same IEEE result from seed zero. Encoder self-check, BLE enqueue CRC and dequeue recomputation remain; ADC SPI CRC8 is unchanged. Fragment planning replaces repeated trial-header `snprintf` calls with exact decimal-width arithmetic. The transmitted header still uses the original formatter, with unchanged fragment selection, bytes, 2304-byte limit, queues and deadlines. Executed host differential tests, ASan/UBSan and final-ELF ROM call-site checks pass; checks are retained rather than removed for speed.

`caa9082` 另补 ADS 末位接管前的明确低电平等待：只有精确 DMA 描述符确认回收、SPI SCLK 为低且 GPIO 接管后再次读低，才执行 `ceil(500/actual_kHz)` µs 等待（2 MHz 时 1 µs），随后执行既有上升沿/真实 MISO 采样/低电平与路由恢复。读低失败不产生上升沿或样本并隔离；DMA 完成未知不接管。它补上源码层面不能依靠调用开销的顺序保证，**不证明旧实机波形曾违反手册或接管已无毛刺**。首次错误、恢复错误、CRC/freshness/invalid 语义保留；已开始 framing 的收尾可能超过工作 deadline，不能宣称超时即硬件已停止。证据：[CRC/planner 等价审查](docs/audit/artifacts/cdc_recovery_acceptance_20261116/crc-planner-independent-review-01.json)、[low-hold 审查](docs/audit/artifacts/cdc_recovery_acceptance_20261116/adc-tail-low-hold-independent-review-01.json)、[实际 ELF](docs/audit/artifacts/cdc_recovery_acceptance_20261116/transport-cpu-01-elf-evidence.json)。

`caa9082` also adds an explicit low hold before the ADS final-bit edge. Only after the exact DMA descriptor is reclaimed, SPI SCLK reads low and GPIO handoff also reads low does it wait `ceil(500/actual_kHz)` microseconds—1 µs at 2 MHz—before the existing rising edge, real MISO sample, low edge and route restoration. A low-read failure produces no rise/sample and isolates the driver; unknown DMA completion never permits handoff. This closes a source-level ordering guarantee that must not depend on call overhead; **it does not prove the old hardware waveform violated the datasheet or that handoff is glitch-free**. First-error, cleanup-error, CRC/freshness/invalid semantics remain. Completing already-started framing can extend beyond the work deadline, so timeout does not establish stopped hardware. Evidence: [CRC/planner equivalence](docs/audit/artifacts/cdc_recovery_acceptance_20261116/crc-planner-independent-review-01.json), [low-hold review](docs/audit/artifacts/cdc_recovery_acceptance_20261116/adc-tail-low-hold-independent-review-01.json), [actual ELF](docs/audit/artifacts/cdc_recovery_acceptance_20261116/transport-cpu-01-elf-evidence.json).

| 指标 / Metric | screen10 `a656d53` | screen11 `caa9082-dirty` |
|---|---:|---:|
| Duration | 120.000122194 s | 120.000070722 s |
| Capture / USB / BLE FPS | 65.594513 / 65.616123 / 44.162295 | 64.376134 / 64.384201 / 47.080922 |
| USB trusted / internal missing observations | 7,873 / 0 | 7,724 / 0 |
| BLE trusted / missing observations | 5,298 / 2,572 | 5,650 / 2,077 |
| ADC rejected / checked reads | 0 / 84,943 | 0 / 82,911 |
| USB/BLE ASCII CRC / identity / wire-contract errors | all 0 | all 0 |
| USB S1D2 X12 / BLE X12 invalid points | 740 / 465 | 670 / 434 |
| Cleanup / overall verdict | CAP8_QUERY_VERIFIED / FAIL_BLE_THROUGHPUT | CAP8_QUERY_VERIFIED / **FAIL_BLE_THROUGHPUT** |

USB 连续性来自同 identity、7,724 有序可信帧、seq1571…9294 和空 gap 列表，不将缺键补零。ADC 为119.320022秒独立健康窗口内42323 even+40588 odd读数增量。BLE 实收比 screen10 增加6.608866%，但独立119.609541秒BL50窗仍 dropD+2044，不能称背压已解决。实际 BLELINK 为 MTU247、PHY2、7.5 ms；**未将 MTU517 候选实施或计入此结果**。未观测 MFAULT/reset/WDT 受录制覆盖限制，X12仍无效，绝对精度未验收。

USB contiguity follows from one identity, 7,724 ordered trusted frames, seq1571…9294 and an empty gap list, not a zero default for a missing key. ADC counts are 42323 valid-even plus40588 valid-odd reads over a separate 119.320022-second health span. BLE receipt rises 6.608866% over screen10, but its independent 119.609541-second BL50 span still reports dropD+2044; backpressure remains. Actual BLELINK reports MTU 247, PHY 2 and 7.5 ms; **an MTU517 candidate is neither implemented nor credited in this result**. No observed MFAULT/reset/WDT is limited by recording coverage; X12 remains invalid and absolute accuracy is unaccepted.

分层和各角色 CPU 比较见 [screen11 verdict](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-11/screen-11-verdict.json) 与 [screen10→11 对照](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-11/screen10-screen11-layer-cpu-comparison.json)：TRANSPORT 比率5.324313%→3.615579%，BLE_TX 16.297535%→15.454282%；独立 switch-out 窗口不可求和为全核 CPU，SDK wall−owner run 不是 blocked。16 个已关闭 boot/state/ROM/debug/screen11 原流及 flash stdout 共21,438,993 B→2,515,154 B gzip，逐字节/SHA回验且保留原件，见[归档索引](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-11/archive-index-01.json)。当前镜像600秒/32配置 NOT_RUN；下节600秒属于旧 `a656d53`。

See the [screen11 verdict](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-11/screen-11-verdict.json) and [screen10→11 comparison](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-11/screen10-screen11-layer-cpu-comparison.json) for independent layers/CPU roles: TRANSPORT ratio 5.324313%→3.615579%, BLE_TX 16.297535%→15.454282%. Independent switch-out windows are not summed into whole-core CPU, and SDK wall minus owner runtime is not blocked time. Sixteen closed boot/state/ROM/debug/screen11 recordings and flash stdout logs total 21,438,993 bytes→2,515,154 gzip bytes, with byte/SHA roundtrip checks and originals retained in the [archive index](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-11/archive-index-01.json). The current image's 600-second/32-configuration runs remain NOT_RUN; the next section's 600-second result belongs to the previous `a656d53` image.

## 2026-11-16 历史 `a656d53` 连续 600 秒稳定性 / Historical `a656d53` continuous 600-second stability

`stability-01` 是同一 `a656d53` / session `B528ED81F93297CF` 的单次连续采集，duration 与单调 start/end 一致，**不是把多个 120 秒窗口累加**。健康日志首末 `engineFrameCount=8640→47936`、`modeEntryFrameCount=8259`、interval=64，显示已有累计计数的模式区间；不预设 SetMode 把计数清零。0/428,131 是 599.484129 秒健康日志间已校验 even/odd/rejected 读数增量，含重试尝试，不是 39,328 个科学帧的计数。

`stability-01` is one continuous recording on the same `a656d53` / session `B528ED81F93297CF`, with matching duration and monotonic start/end, **not a sum of 120-second screens**. Health endpoints report `engineFrameCount=8640→47936`, `modeEntryFrameCount=8259`, interval 64, documenting a mode interval with prior accumulated counts rather than assuming SetMode resets them. The 0/428,131 result counts checked even/odd/rejected reads, including retry attempts, between health records 599.484129 seconds apart; it is not the scientific-frame count.

| 指标 / Metric | 连续稳定性 / Continuous stability |
|---|---:|
| Capture duration | 600.000075751 s |
| Capture / USB / BLE FPS | 65.543247 / 65.549657 / 44.207974 |
| USB trusted frames / internal missing observations | 39,328 / 0, seq 13901–53228 |
| BLE trusted frames / missing observations | 26,523 / 12,805 |
| ADC rejected / checked reads | 0 / 428,131 |
| USB/BLE ASCII CRC / identity / wire-contract errors | all 0 |
| Common frames with identical original bytes | 26,520 / 26,520 |
| USB S1D2 X12 / BLE X12 invalid points | 3,640 / 2,291 |
| Complete STK50 free / lifetime minimum | 8,301,048 / 8,297,776 B |
| MFAULT / RESET / WDT | Not observed within covered recordings |
| Cleanup / overall acceptance | CAP8_QUERY_VERIFIED / **FAIL_BLE_THROUGHPUT** |

USB 内部零缺口由单身份、有序帧数、首末序号差及空 gap 列表共同证明。BLE 收到的分片无校验/组装错误，但 BL50 自己的 599.542572 秒窗仍 `dropD+12583`，不能与主机缺少的 12805 序号相加。USBQ 在独立 600.432809 秒查询窗内 dDrop/cDrop/expired/partial 增量均 0；无效 X12 不填零、不平滑。主机同期存在开发工作，通知时间为 host callback/接收时间，不是空中时间或隔离主机负载的实验；各角色 CPU/SDK 墙钟仍用独立分母，不推导 blocked。

Zero internal USB loss is proven jointly by one identity, ordered count, sequence endpoints and an empty gap list. Received BLE fragments pass checksum/assembly checks, but BL50 still reports `dropD+12583` in its own 599.542572-second window; this cannot be added to the host's 12,805 missing observations. USBQ dDrop/cDrop/expired/partial deltas are zero in a separate 600.432809-second query window. X12 is neither zero-filled nor smoothed. Development work ran concurrently on the host; notification timestamps are host callback/receipt times, not over-air timing or an isolated-host-load experiment. CPU/SDK wall observations retain their own denominators and do not determine blocked time.

三个采集器关闭后，[后续只读查询](docs/audit/artifacts/cdc_recovery_acceptance_20261116/post-stability-unread-01.summary.json)仍返回同一完整 BUILD/session、CAP8/ready1、USB mounted1/epoch1。后来的累计 expired=10258、partial=7 与控制 cDrop=cExpired=0 是**另一个查询时点**，不是 600 秒内丢失；本任务 reader 关闭不等于物理拔线或 USB unmounted，其他 reader 是否存在未核验。

After the three collectors closed, a [later read-only query](docs/audit/artifacts/cdc_recovery_acceptance_20261116/post-stability-unread-01.summary.json) still returned the same full BUILD/session, CAP8/ready1 and USB mounted1/epoch1. Its cumulative expired=10258, partial=7 and control cDrop=cExpired=0 belong to **a later observation**, not loss inside the 600-second window. Closing this task's readers is neither physical unplugging nor USB unmounting; other readers were not ruled out.

[稳定性判定](docs/audit/artifacts/cdc_recovery_acceptance_20261116/stability-01/stability-01-verdict.json)、[完整分析](docs/audit/artifacts/cdc_recovery_acceptance_20261116/stability-01/stability-01-analysis-01.json)、[screen10→600 秒各层/CPU 对照](docs/audit/artifacts/cdc_recovery_acceptance_20261116/stability-01/screen10-stability01-layer-cpu-comparison.json)。5 个稳定性原始流共 97,229,846 B，gzip 为 11,121,002 B；2 个后续只读流另为 84,591→16,636 B，全部逐字节/SHA 回验且保留原件，见[稳定性归档](docs/audit/artifacts/cdc_recovery_acceptance_20261116/stability-01/archive-index-01.json)和[后续查询归档](docs/audit/artifacts/cdc_recovery_acceptance_20261116/stability-01/post-readers-closed-archive-index-01.json)。该连续窗口完成不等于 BLE/精度验收；32 配置仍 NOT_RUN。

See the [stability verdict](docs/audit/artifacts/cdc_recovery_acceptance_20261116/stability-01/stability-01-verdict.json), [full analysis](docs/audit/artifacts/cdc_recovery_acceptance_20261116/stability-01/stability-01-analysis-01.json) and [screen10→600-second layer/CPU comparison](docs/audit/artifacts/cdc_recovery_acceptance_20261116/stability-01/screen10-stability01-layer-cpu-comparison.json). Five stability originals total 97,229,846 bytes, compressed to 11,121,002 bytes; two later query streams separately total 84,591→16,636 bytes. All passed byte/SHA roundtrip verification with originals retained in the [stability archive](docs/audit/artifacts/cdc_recovery_acceptance_20261116/stability-01/archive-index-01.json) and [later-query archive](docs/audit/artifacts/cdc_recovery_acceptance_20261116/stability-01/post-readers-closed-archive-index-01.json). Completing this continuous window is not BLE/accuracy acceptance; the 32-configuration suite remains NOT_RUN.

## 2026-11-16 `-O2` 候选与安全检查 / `-O2` candidate and retained safety checks

ESP-IDF 5.5.5 的原 Kconfig 只在 debug/core-dump/GDB 条件下启用 `FREERTOS_TASK_FUNCTION_WRAPPER`。单纯改为 performance 会移除任务意外返回时的 log/abort 检查，所以[第一候选被拒绝且未烧录](docs/audit/artifacts/cdc_recovery_acceptance_20261116/compiler-perf-01-scope-rejected.json)。`a656d53` 在项目 `main/kconfigs/Kconfig.compat` 中仅增加 S3+PERF 的同一 SDK symbol 条件，main 编译 guard 在 wrapper 缺失时拒绝构建；没有修改 SDK、开启 GDB/core dump 或另写替代 wrapper。

ESP-IDF 5.5.5's original Kconfig enables `FREERTOS_TASK_FUNCTION_WRAPPER` only for debug/core-dump/GDB configurations. Simply selecting performance removes the log/abort check for an unexpectedly returning task, so [the first candidate was rejected and never flashed](docs/audit/artifacts/cdc_recovery_acceptance_20261116/compiler-perf-01-scope-rejected.json). `a656d53` adds only the S3+PERF condition to that same SDK symbol in project `main/kconfigs/Kconfig.compat`; a main compile guard rejects a missing wrapper. It neither edits the SDK, enables GDB/core dump nor substitutes a new hand-written wrapper.

18 个真实 Kconfig 边界、6 个 guard 编译用例通过，3,769 个全树 symbol 中该扩展只改变 wrapper。当前配置保留 assertions、strong stack check、heap poisoning、task watchdog 和 end-of-stack watchpoint；CPU 仍 160 MHz，CRC 门槛/协议/队列/限速不变。最终 ELF 已确认 `vPortTaskWrapper` 调用任务、返回后 log→abort，以及初始 task stack PC 实际引用 wrapper。**这证明链接/初始栈路径，不是实机任务返回注入通过**；该故障注入为 NOT_RUN。证据：[范围](docs/audit/artifacts/cdc_recovery_acceptance_20261116/compiler-perf-02-scope.json)、[独立审查](docs/audit/artifacts/cdc_recovery_acceptance_20261116/performance-wrapper-independent-review-01.json)、[ELF](docs/audit/artifacts/cdc_recovery_acceptance_20261116/compiler-perf-02-wrapper-elf-review.json)。

Eighteen real Kconfig boundary cases and six guard-compilation cases passed; the extension changes only the wrapper among 3,769 full-tree symbols. Assertions, strong stack checks, heap poisoning, task watchdog and end-of-stack watchpoint remain enabled. CPU stays at 160 MHz, with unchanged CRC criterion, protocol, queues and rate limits. Final ELF inspection confirms the wrapper calls the task, reaches log→abort on return, and is the actual initial task-stack PC. **This verifies linked code/stack initialization, not a hardware return-fault injection**, which is NOT_RUN. Evidence: [scope](docs/audit/artifacts/cdc_recovery_acceptance_20261116/compiler-perf-02-scope.json), [independent review](docs/audit/artifacts/cdc_recovery_acceptance_20261116/performance-wrapper-independent-review-01.json), [ELF](docs/audit/artifacts/cdc_recovery_acceptance_20261116/compiler-perf-02-wrapper-elf-review.json).

[67 入口与构建身份](docs/audit/artifacts/cdc_recovery_acceptance_20261116/compiler-perf-02-validated-identity.json)及[完整 BUILD/READY](docs/audit/artifacts/cdc_recovery_acceptance_20261116/compiler-perf-02-boot-verification.json)已完成。新 screen10 实测已闭合，旧 screen09 数字仅作历史对照。

The [67-entry/build identity](docs/audit/artifacts/cdc_recovery_acceptance_20261116/compiler-perf-02-validated-identity.json) and [full BUILD/READY verification](docs/audit/artifacts/cdc_recovery_acceptance_20261116/compiler-perf-02-boot-verification.json) are complete. Screen10 is now closed; screen09 numbers remain a historical comparison.

| 指标 / Metric | screen09 `-Og` | screen10 `-O2` |
|---|---:|---:|
| Capture / USB / BLE FPS | 57.787813 / 57.664438 / 39.133976 | 65.594513 / 65.616123 / 44.162295 |
| USB trusted frames / internal missing observations | 6,920 / 0 | 7,873 / 0, seq 1331–9203 |
| BLE trusted frames / missing observations | 4,695 / 2,223 | 5,298 / 2,572 |
| ADC rejected / checked reads | 0 / 74,721 | 0 / 84,943 |
| USB/BLE ASCII CRC / identity / wire-contract errors | all 0 | all 0 |
| USB S1D2 X12 / BLE X12 invalid points | 467 / 340 | 740 / 465 |
| ASYNC_LOG Δrun / own observation span | 13.494264% | 14.849499% |
| Overall acceptance | FAIL_BLE_THROUGHPUT | **FAIL_BLE_THROUGHPUT** |

120.000122 秒内 BLE 实收提高 12.849%，但采集也更快，BLE pool 的独立 119.677834 秒 BL50 窗仍 `dropD +2533`；不能只看 FPS 增长宣布背压消失。USB 连续性按单身份、有序数量、首末 seq 差及空 gap 列表核验，USBQ 在 120.404974 秒查询窗内 dDrop/cDrop/expired/partial 增量均 0。ADC 0/84943 属于 119.069615 秒健康窗口；浮空 S1D2 X12 如实保留，未观测到 WDT/MFAULT/reset不等于无限运行保证。结束后 CAP8 已查询确认，完整 600 秒/32 配置及绝对精度仍未验收。

Over 120.000122 seconds BLE receipt improves 12.849%, but capture is faster too; the independent 119.677834-second BL50 window still reports `dropD +2533`. Higher FPS therefore does not mean backpressure disappeared. USB contiguity is checked from one identity, ordered count, first/last sequence distance and an empty gap list; USBQ dDrop/cDrop/expired/partial increments are zero over its 120.404974-second query span. The ADC result covers 119.069615 seconds of health records. Floating S1D2 X12 remains invalid, and no observed WDT/MFAULT/reset is not an indefinite-run guarantee. CAP8 was query-verified afterward; full 600-second/32-configuration and absolute-accuracy acceptance remain pending.

[screen10 判定](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-10/screen-10-verdict.json)、[09→10 各层/CPU 对照](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-10/screen-09-10-layer-cpu-comparison-01.json)保留独立窗口；不跨 boot 相减累计量、不把 SDK wall−owner run 叫 blocked。13 个 compiler-perf/screen10 已关闭原始流共 21,033,025 B，压缩后 2,490,426 B，均解压逐字节及 SHA 回验通过，[索引](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-10/archive-index-01.json)保留所有原始文件，未覆盖旧证据。

The [screen10 verdict](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-10/screen-10-verdict.json) and [09→10 layer/CPU comparison](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-10/screen-09-10-layer-cpu-comparison-01.json) retain independent windows. Cumulative counters are not subtracted across boots, and SDK wall−owner run is not blocked time. Thirteen closed compiler-perf/screen10 recordings total 21,033,025 bytes, compressed to 2,490,426 bytes; every file passed decompression byte/SHA verification. The [index](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-10/archive-index-01.json) retains all originals without overwriting historical evidence.

## 2026-11-16 screen09（历史）：元数据单遍解析 / Historical screen09: single-pass metadata parsing

`dafaf9b` 只优化 `sensorarrayTransportParseFrameMeta`：输出 owner 在入队前，按一次 token 遍历从已复制的不可变帧字节提取字段，避免每个字段反复扫描整段 header。它保留各 gen/rid 域、重复字段/数值边界、首行规则和 CAP preview，既不从最新全局状态重新盖章，也不改 encoder、科学 CRC、队列容量、调度预算或任何原有 stdout 输出。[66,896 个完整 slot 差分](docs/audit/artifacts/cdc_recovery_acceptance_20261116/transport-meta-single-pass-host-01.json)与 ASan/UBSan 实际通过；20 个完整 encoder 帧保持原始字节及 CRC。LSan 因执行环境 ptrace 限制为 NOT_RUN，首次失败原样保留，不能写成泄漏检查通过。

`dafaf9b` optimizes only `sensorarrayTransportParseFrameMeta`. Before enqueue, the output owner extracts metadata from the copied immutable frame in one token traversal, replacing repeated whole-header scans per field. Generation/request domains, duplicate/numeric boundaries, first-line rules and CAP preview remain unchanged. It neither restamps from current globals nor changes the encoder, scientific CRC, queue capacity, scheduling budgets or existing stdout output. The [66,896 complete-slot differential cases](docs/audit/artifacts/cdc_recovery_acceptance_20261116/transport-meta-single-pass-host-01.json) and ASan/UBSan actually passed; 20 complete encoder frames retain their original bytes and CRC. LSan is NOT_RUN because of the environment's ptrace limitation; the initial failure is preserved, not represented as a passing leak check.

| 指标 / Metric | screen08 | screen09 |
|---|---:|---:|
| Capture / USB / BLE FPS | 59.184655 / 59.173785 / 33.118267 | 57.787813 / 57.664438 / 39.133976 |
| USB trusted frames; internal missing observations | 7,101; 0 | 6,920; 0, seq 1600–8519 |
| BLE trusted frames; missing observations | 3,974; 3,128 | 4,695; 2,223 |
| ADC rejected / checked reads | 0 / 76,841 | 0 / 74,721 |
| USB / BLE ASCII CRC and identity errors | all 0 | all 0 |
| USB S1D2 X12 invalid points | 525 | 467 |
| ASYNC_LOG Δrun / own observation span | 24.600562% | 13.494264% |
| 整体验收 / Overall acceptance | FAIL_BLE_THROUGHPUT | **FAIL_BLE_THROUGHPUT** |

此次 BLE 实收提高 18.16%，ASYNC_LOG 独立窗口比例下降 11.1063 个百分点，支持该热路径有实际开销；并不证明全部剩余瓶颈已定位，采集速率本次也下降。USB 内部连续由 ordered 数量、首末序号差及空 gap 列表共同核验；BLE 收到的分片无校验/组装错误，仍存在完整帧背压。ADC 0/74,721 属于 118.765452 秒健康日志窗口，不是永久零错保证。未观测到 WDT/MFAULT/reset，X12 保留无效；绝对精度仍未评估。

BLE receipt increased by 18.16% in this comparison, while ASYNC_LOG's independent-window ratio fell 11.1063 percentage points, supporting a material cost in this path. It does not identify every remaining bottleneck; capture rate also fell in this run. USB contiguity is verified jointly from ordered count, first/last sequence distance and an empty gap list. Received BLE fragments pass checksum/assembly checks, while complete-frame backpressure remains. The ADC result covers a 118.765452-second health-record window, not indefinitely error-free operation. No WDT/MFAULT/reset was observed, X12 remains invalid, and absolute accuracy is unevaluated.

[完整 BUILD/READY 身份](docs/audit/artifacts/cdc_recovery_acceptance_20261116/frame-meta-01-boot-verification.json)、[66 入口与构建](docs/audit/artifacts/cdc_recovery_acceptance_20261116/frame-meta-01-validated-identity.json)、[screen09 判定](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-09/screen-09-verdict.json)和[08→09 各层/CPU 对照](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-09/screen-08-09-layer-cpu-comparison-01.json)保留窗口和来源。结束后 CAP8 已查询确认；CPU 配置仍 ON、160 MHz、`-Og`，下一 `-O2` 候选尚未作为此结果测试。各 CPU 角色窗口不能合计，SDK wall−owner run 仍不是 blocked。

The [full BUILD/READY identity](docs/audit/artifacts/cdc_recovery_acceptance_20261116/frame-meta-01-boot-verification.json), [66-entry/build record](docs/audit/artifacts/cdc_recovery_acceptance_20261116/frame-meta-01-validated-identity.json), [screen09 verdict](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-09/screen-09-verdict.json) and [08→09 layer/CPU comparison](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-09/screen-08-09-layer-cpu-comparison-01.json) retain source/window details. CAP8 was query-verified after capture. Profiling remains ON at 160 MHz with `-Og`; the next `-O2` candidate has not been tested as part of this result. Per-role CPU windows cannot be summed, and SDK wall−owner run is not blocked time.

## 2026-11-16 screen08（历史）：分配生命周期修复与日志统计清理 / Historical screen08: allocation lifetime and unused log statistics

当前 `b9ebe2f-dirty` 包含 `c670897` 的 ADS SPI 部分缓冲分配 P1 修复：分配成功前指针仅保存在局部，避免失败路径释放后 Init/Deinit 再次释放；S3/非 S3 的真实生产 Init/Deinit 各 16 组合回归通过。`98b760c` 只移除不可达 PREC20 与已停用 legacy netStatus 消费的统计，活跃输出按字节对照不变。`b9ebe2f` 本身修正主机 BLE 清理测试对总 deadline 耗尽时第四个错误的断言，并新增确定性用例，**没有放宽生产 timeout**。原失败日志保留。[修改范围](docs/audit/artifacts/cdc_recovery_acceptance_20261116/output-trim-source-diff-01.json)、[66 入口与构建身份](docs/audit/artifacts/cdc_recovery_acceptance_20261116/output-trim-02-validated-identity.json)、[完整 BUILD/READY 核验](docs/audit/artifacts/cdc_recovery_acceptance_20261116/output-trim-02-boot-verification.json)。

Current `b9ebe2f-dirty` includes the `c670897` ADS SPI partial-buffer-allocation P1 fix: pointers remain local until allocation succeeds, avoiding a second free by Init/Deinit after partial failure. Sixteen production Init/Deinit combinations passed for each S3/non-S3 path. `98b760c` removes only statistics consumed by unreachable PREC20 and disabled legacy netStatus; active output remains byte-equivalent. `b9ebe2f` itself corrects the host BLE-cleanup test for the legitimate fourth error when its total deadline expires and adds a deterministic case; **production timeouts are unchanged**. Original failure logs remain. See the [change scope](docs/audit/artifacts/cdc_recovery_acceptance_20261116/output-trim-source-diff-01.json), [66-entry/build identity](docs/audit/artifacts/cdc_recovery_acceptance_20261116/output-trim-02-validated-identity.json) and [full BUILD/READY verification](docs/audit/artifacts/cdc_recovery_acceptance_20261116/output-trim-02-boot-verification.json).

| 指标 / Metric | screen07 历史 / Historical | screen08 当前 / Current |
|---|---:|---:|
| 完整采集窗 / Complete capture window | 120.000117 s | 120.000130 s |
| Capture / USB / BLE FPS | 59.900009 / 59.812416 / 32.390344 | 59.184655 / 59.173785 / 33.118267 |
| USB 可信帧；内部缺少序号 / Trusted frames; internal missing observations | 7,178; 1 | 7,101; **0**, seq 1259–8359 |
| BLE 可信帧；缺少序号 / Trusted frames; missing observations | 3,886; 3,293 | 3,974; 3,128 |
| ADC CRC 拒收 / Checked-read rejections | 0 / 76,128 | 0 / 76,841 |
| USB ASCII CRC / identity errors | 0 / 0 | 0 / 0 |
| BLE ASCII CRC / identity errors | 0 / 0 | 0 / 0 |
| USB S1D2 无效点 / Invalid points | 406 × X12 | 525 × X12 |
| ASYNC_LOG Δrun / 独立观测跨度 / Own observation span | 24.974627% | 24.600562% |
| 总体验收 / Overall acceptance | FAIL_BLE_THROUGHPUT | **FAIL_BLE_THROUGHPUT** |

screen08 的 USB 零缺口由有序可信帧数、首末序号差和空 gap 列表共同验证，不把缺失的 `missing_frames` 字段默认为零。各层独立窗口仍分开：USBQ 在 120.592613 秒查询窗内 `dDrop/cDrop/expired/partial` 增量均 0；BL50 在 119.650096 秒窗内 `dropD +3037`，不能与主机 3128 缺口相加。ADC 0/76841 覆盖 118.996649 秒健康日志跨度；完整原始流未观测到 WDT/MFAULT/reset，但不证明未记录事件绝不存在。结束后 CAP8 cleanup 已查询确认，绝对精度仍未评估。

Screen08's zero internal USB gap is established from ordered trusted-frame count, first/last sequence distance and an empty gap list, not by defaulting an absent `missing_frames` field to zero. Layer windows remain separate: USBQ increments for `dDrop/cDrop/expired/partial` are all zero over a 120.592613-second query span; BL50 reports `dropD +3037` over 119.650096 seconds, which cannot be added to the host's 3,128 missing observations. The ADC result covers a 118.996649-second health-record span. No WDT/MFAULT/reset was observed in the recorded streams, without ruling out unrecorded events. CAP8 cleanup was query-verified; absolute accuracy remains unevaluated.

ASYNC_LOG 比例仅降低约 0.374 个百分点，BLE 仍约 33 FPS，**不能宣称日志计算是主要原因或瓶颈已修复**。两次 boot 的累计值不相减，仅比较各自角色独立窗口内的差分与比例；stale/未观测仍 unknown，SDK wall−owner run 仍不能称 blocked。[新判定](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-08/screen-08-verdict.json)和[分层/CPU 对照](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-08/screen-07-08-layer-cpu-comparison-01.json)保留全部分母、原始身份和限制；历史 screen07 及更早失败不改写。

ASYNC_LOG's ratio fell only about 0.374 percentage points while BLE remains around 33 FPS. **This does not establish log computation as the primary cause or a repaired bottleneck.** Cumulative counters are never subtracted across boots; only independently computed per-role deltas/ratios are compared. Stale/unobserved values remain unknown, and SDK wall−owner run is still not blocked time. The [new verdict](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-08/screen-08-verdict.json) and [layer/CPU comparison](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-08/screen-07-08-layer-cpu-comparison-01.json) preserve denominators, identities and limits. Screen07 and earlier failures remain unchanged historical evidence.

## 2026-11-16 CPU 诊断首轮 screen07（历史） / First CPU diagnostic screen07 (historical)

[完整 BUILD 与启动身份核验 / Full BUILD and boot identity](docs/audit/artifacts/cdc_recovery_acceptance_20261116/cpu-owner-01-boot-verification.json)将 app-only 写入、完整 64 个十六进制字符的 ELF SHA256、`READY=1` 和同 session 的 CAP8 状态连接起来；ELF SHA256 与 app binary SHA256 分别保存，不能互换。此镜像使用 ESP-IDF 5.5.5，启动选中 2 MHz、`gpio_tail_required`。CPU profiling 在该诊断构建中开启，正常 `sdkconfig.defaults` 仍关闭；[65 入口结果](docs/audit/artifacts/cdc_recovery_acceptance_20261116/cpu-owner-02-tests-manifest.json)保留前一轮单个 harness 缺声明的失败记录，没有把它改写成固件失败或删除。

The [boot verification](docs/audit/artifacts/cdc_recovery_acceptance_20261116/cpu-owner-01-boot-verification.json) links the app-only write, full 64-hex-digit ELF SHA256, `READY=1` and same-session CAP8 state. ELF and app-binary hashes are recorded separately. This ESP-IDF 5.5.5 image selected 2 MHz with `gpio_tail_required`. CPU profiling is enabled for this diagnostic build; normal defaults remain disabled. The [65-entry results](docs/audit/artifacts/cdc_recovery_acceptance_20261116/cpu-owner-02-tests-manifest.json) preserve the earlier missing-declaration harness failure without relabeling it as a firmware failure or deleting it.

| 指标 / Metric | screen07 实测 / Observation |
|---|---:|
| 完整采集窗 / Complete capture window | 120.000117 s |
| Capture / USB / BLE | 59.900009 / 59.812416 / 32.390344 FPS |
| USB 可信帧 / Trusted frames; missing sequence observations | 7,178; 1 |
| BLE 可信帧 / Trusted frames; missing sequence observations | 3,886; 3,293 |
| ASCII CRC / identity / wire-contract errors | USB 0 / 0 / 0; BLE 0 / 0 / 0 |
| 两端共同帧原字节一致 / Common frames with identical original bytes | 3,884 / 3,884 |
| ADC CRC 拒收 / Rejected checked ADC reads | 0 / 76,128; diagnostic span 118.796879 s |
| 科学无效点 / Invalid scientific points | USB 406 × X12; BLE 223 × X12; retained as invalid |
| BLE pool `dropD` / `dropL` / `dropC` 增量 / Delta | +3,177 / 0 / 0; separate 119.823636 s BL50 receipt window |
| 完整 STK50 内存观测 / Memory from complete STK50 records | free 8,294,036 B; boot-lifetime minimum 8,290,104 B |
| WDT / MFAULT / reset | 未观测到；日志缺失仍是证据边界 / Not observed; limited to available recordings |
| 整体验收 / Overall acceptance | **FAIL_BLE_THROUGHPUT**; absolute accuracy NOT_EVALUATED |

ADC 分母是健康日志首末的已校验读数增量，含重试尝试，不是帧数，也不是精确采集起止。USB/BLE 缺口各按本端首末可信帧统计，不能相减或与 BL50 丢弃直接相加。收到的 BLE 分片没有校验或组装错误，仍不表示完整科学帧没有丢弃。原始记录、窗口和计数见 [screen07 判定](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-07/screen-07-verdict.json)；旧 screen05/06 的 0/79,417、0/76,069 及所有失败基线继续保留。

The ADC denominator is the checked-read delta between complete health records, including retry attempts, not frame count or exact capture endpoints. USB/BLE gaps use each receiver's own trusted sequence range and cannot be subtracted or added directly to BL50 drops. Received BLE fragments passed checksum and assembly checks, which does not imply that no complete scientific frames were discarded. The [screen07 verdict](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-07/screen-07-verdict.json) retains the raw-evidence links, windows and counters. Prior screen05/06 results of 0/79,417 and 0/76,069, and all failed baselines, remain historical evidence.

历史b58336e/image06的 `cpu_owner_v1` 提供只读 `CPUOWNER?0` 至 `CPUOWNER?6`；当前新增role7另受分页wifi_owner_v1协商，见[新机制](#wifi-owner-v1)，历史计数不移位。七个应用 owner 仅在自身自然 checkpoint 中、最多每秒自采一次 `vTaskGetInfo(NULL, …, pdFALSE, eReady)`，不扫描栈、不保存外部 task handle、不增加 delay 或改变预算/优先级。查询只读短锁保护的快照，带 session、taskNumber、采样前后时间、累计 runUs 和 age；角色多 writer 冲突会保持 unavailable 到下一 boot。配置关闭、从未采到或过期都不能解释为零 CPU。实际新增本地静态存储 456 B，不包括 FreeRTOS trace/runtime 的全局开销。

The historical b58336e/image06 `cpu_owner_v1` exposes read-only `CPUOWNER?0` through `CPUOWNER?6`; current role7 additionally requires paginated wifi_owner_v1, as [defined above](#wifi-owner-v1), without shifting historical counters. The seven application owners self-sample `vTaskGetInfo(NULL, …, pdFALSE, eReady)` at their natural checkpoints at most once per second, without stack scans, retained external task handles, new delays or changes to budgets/priorities. Queries copy a briefly locked snapshot containing session, task number, the sampling bracket, cumulative runtime and age. A conflicting writer makes that role unavailable until boot. Disabled, unseen or stale observations are not zero CPU. The new local static storage is 456 B, excluding global FreeRTOS trace/runtime overhead.

| Role | 本角色独立跨度 / Own observation span (s) | switch-out Δrun (s) | Δrun / own span |
|---|---:|---:|---:|
| 0 BLE_TX | 120.095170 | 17.903690 | 14.9079% |
| 1 TRANSPORT | 119.846299 | 9.070485 | 7.5684% |
| 2 ASYNC_LOG | 120.182388 | 30.015103 | 24.9746% |
| 3 SERIAL_TX | 120.609289 | 11.569529 | 9.5926% |
| 4 BLE_CTRL | unknown: stale | unknown | unknown |
| 5 SERIAL_CTRL | 120.285279 | 0.853506 | 0.7096% |
| 6 WIFI_CTRL | unknown: not_observed | unknown | unknown |

这些比例不是同步 CPU 利用率表，不能合计；runUs 仅在切出时累计，未含正在运行的片段，可能含 ISR 及 SDK 以外工作。独立 BLESVC 查询窗记录 12,991 次完成调用、SDK 墙钟增量 62.018883 s（平均 4.773988 ms）；**不能用 SDK 墙钟减 BLE_TX runUs 得到 blocked**。SDK/IDLE 任务、总 CPU、就绪等待与阻塞归因仍 unknown；开启 profiling 的两镜像对比也不是受控开销实验。[可复跑分析与各角色窗口](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-07/screen-07-cpu-owner-deltas-01.json)及[完整查询契约](docs/integration/firmware_host_contract.md#cpu-owner-只读运行计数契约--read-only-owner-runtime-contract)保留这些限制。

These ratios are not a simultaneous CPU-utilization table and must not be summed. Runtime is accumulated only at switch-out; it excludes the open running slice and may include ISR and non-SDK work. A separate BLESVC query window records 12,991 completed calls and 62.018883 s of SDK wall time, averaging 4.773988 ms. **SDK wall minus BLE_TX runtime is not blocked time.** SDK/idle tasks, total CPU utilization and ready-versus-blocked attribution remain unknown. Comparing images with and without profiling is not a controlled overhead experiment. The [reproducible per-role analysis](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-07/screen-07-cpu-owner-deltas-01.json) and [query contract](docs/integration/firmware_host_contract.md#cpu-owner-只读运行计数契约--read-only-owner-runtime-contract) preserve these limits.

## 2026-11-16 首轮双通路筛查 / First dual-link screen

[完整 120 秒实录与判定 / Full 120-second evidence and verdict](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-01/screen-01-verdict.json)：**FAIL**。运行源码 `3a55543`，VOLT 1 行，USB CDC0 和 BLE 同时记录，独立 CDC1 用于 ADC 诊断；相同会话/配置身份。启动 SPI 为请求 550 kHz、驱动观测 551 kHz，不能拿前一个 boot 的 850 kHz 解释本窗口。

The complete VOLT1 screen **failed** on `3a55543`. Formal CDC0 and BLE were recorded simultaneously, with separate CDC1 diagnostics and matching session/configuration identity. This boot selected 550 kHz (driver-observed 551 kHz); the preceding boot's 850 kHz is not this run's clock.

| 指标 / Metric | 本轮实测 / Observation |
|---|---:|
| 科学采集窗口 / Scientific capture window | 120.0001 s |
| Capture | 64.563554 FPS |
| USB 完整可信帧 / USB complete trusted frames | 62.532747 FPS; 7,502 frames; 239 missing sequence observations |
| BLE 完整可信帧 / BLE complete trusted frames | 43.370896 FPS; 5,204 frames; 2,537 missing sequence observations |
| ASCII CRC / configuration identity errors | USB 0 / 0; BLE 0 / 0 |
| ADC SPI CRC 拒绝 / ADC SPI CRC rejection | 4,378 / 102,828 = **4.257595%**, limit 2.5% |
| CRC 诊断跨度 / CRC diagnostic span | 118.992248 s inside capture; first/last complete ADSHEALTH receipts |
| MFAULT / reset | Not observed in the recorded window; absence is limited to observed logs |
| 绝对精度 / Absolute accuracy | NOT_ACCEPTED; approximate DMM RAM rails and floating-port background remain nonprecision references |

ADC 分母是已校验有效 even/odd 读数与 CRC 拒绝的增量之和，包含有限重试，不能再次加 retry 或用帧数代替。两通路缺口是各自的可信序号观察缺口，不能相减为独立层丢失。扫描汇总 `scope=all_candidates` 包含被淘汰档位，不等于选中档错误率。先定位运行期错误及有界输出队列背压，再重做筛查；600 秒与32配置测试不以本次失败替代。

The ADC denominator sums valid-even, valid-odd and rejected checked reads, including bounded retries; retries must not be added again and frame count is not the denominator. Each transport's missing count refers to its own trusted sequence observations, not disjoint losses. Startup `scope=all_candidates` includes rejected rates and is not the selected-rate error ratio. Runtime CRC and bounded-output backpressure must be addressed before repeating the screen and starting the 600-second/32-configuration tests.

## 2026-11-16 第二轮筛查与 CRC 定位 / Second screen and CRC investigation

第二轮已烧录源码 / Second-screen source: `1e9fd19799afa13bbf4584d986f5a797c57ec74d`; app SHA256 `06d15e24fa2f60961789645cb1b46b46b4f06de4b53445861315444a695c3f1c`; ELF SHA256 `c7bdc7a11bf87aec5a99a720cf88b796484c9cb2f129669da3827360dcaf82dc`. 58 个 host 入口和目标构建通过，烧录后完整 BUILD 已核对。 / All 58 host entries and the target build passed; the full running BUILD hash was verified after flashing.

[第二轮完整 120 秒 / Second full 120-second screen](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-02/screen-02-verdict.json)仍为 **FAIL_BLE_THROUGHPUT**，不是整体验收通过。 / It remains **FAIL_BLE_THROUGHPUT**, not full acceptance.

| 指标 / Metric | screen-02 |
|---|---:|
| 科学采集窗口 / Scientific capture window | 120.000091 s |
| Capture | 68.047855 FPS |
| USB 完整可信帧 / USB complete trusted frames | 68.028989 FPS; 8,164 frames; 1 missing sequence observation |
| BLE 完整可信帧 / BLE complete trusted frames | 35.347479 FPS; 4,241 frames; 3,923 missing sequence observations |
| ADC SPI CRC 拒收 / ADC SPI CRC rejection | 2,152 / 107,124 = 2.008887% |
| 本 boot SPI 请求 / 驱动观测 / Requested / observed SPI clock | 750,000 / 754,000 Hz |
| ASCII CRC / configuration identity errors | USB 0 / 0; BLE 0 / 0 |
| MFAULT / reset | Not observed in the recorded window |

正式 USB 缺口大幅减少，但 BLE pool 的 dropD 增量为 3,690，上一轮为 0；原 transport 层排队拒绝减少，压力转移到 BLE 服务环节。各计数器查询跨度与完整帧采集窗口不同，不把它们直接相加为实收缺口。不能靠扩大队列消除持续服务率差额。两轮启动选速不同，因此不能把 CRC 下降归因于任务优先级修改。详见[分层对照](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-02/ble-service-review-01.json)。

Formal USB loss fell substantially, but BLE pool dropD increased by 3,690 versus zero in screen-01. Admission pressure moved from the transport queue into BLE service. Counter-query spans differ from the complete-frame capture window, so their increments are not added to infer receiver gaps. A larger queue cannot remove a sustained service-rate deficit. Different boot-selected SPI clocks also prevent attribution of the lower CRC ratio to task priority alone.

用户随后提出可考虑 5% 错误率，同时要求先查明根因。本轮将 5% 作为待评估的调试容忍度，尚未改变启动选速的 2.5% 门限；原先按 2.5% 判定的 FAIL 保留。任何容忍度都不允许 CRC 错误值进入科学帧、缓存或 PGA 决策。

The user subsequently suggested considering 5% while prioritizing root-cause repair. It is a prospective debugging tolerance, not an implemented change to the 2.5% startup qualification. Historical failures retain their original criteria. No tolerance permits CRC-rejected values into scientific frames, caches or PGA decisions.

[249 条完整坏包的独立复算](docs/audit/artifacts/cdc_recovery_acceptance_20261116/ads-runtime-root-cause-01.json)全部只差 CRC 最低位，其中 248 条为 0→1、1 条为 1→0。记录是限量诊断子集，不代表所有失败包。该方向性支持调查末位采样/DOUT-DRDY 交接，但不能单凭它认定硬件或驱动根因。正在实现保持 SPI 时钟不变的连续/单次转换、PGA1/bypass 对照，以及单次保持数据的重复诊断读取；这些读取不是科学样本，恢复状态必须独立验证。2 秒总期限包含 500 ms 收尾预留；未完成组必须标 incomplete。

Independent recomputation of 249 complete bad packets found only CRC bit 0 differing: 248 transitions from 0 to 1 and one from 1 to 0. These are bounded diagnostic records, not every failure. This directional pattern motivates investigation of final-bit sampling and DOUT/DRDY handoff without establishing either a hardware or driver cause. A constant-clock continuous/pulse and PGA1/bypass comparison, including held-result diagnostic rereads, is being implemented. These reads are not scientific samples; restored state requires independent verification. The two-second total deadline reserves 500 ms for cleanup, and unfinished groups must remain incomplete.

第二轮在 754 kHz 驱动观测时钟下的[独立复算](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-02/crc-tail-review-01.json)又取得 156 条完整坏包：全部 `computed XOR received = 01`，其中145条0→1、11条1→0，复算不一致与截断均为0。两轮合计405条均出现这一 syndrome；这是限量错误样本的共同特征，不能据此证明数据域一定无误或推定全部失败的物理原因。

A [second independent calculation](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-02/crc-tail-review-01.json), at the driver-observed754kHz clock, finds156 further complete bad packets: all have `computed XOR received = 01`, with145 zero-to-one and11 one-to-zero differences, no recomputation disagreement and no truncated records. All405 records across the two screens share this syndrome. This bounded error sample does not establish an uncorrupted data field or the physical cause of every failure.

## 诊断首版启动失败与内存修正 / Initial diagnostic boot failure and memory correction

`bb230d0` 实际app-only烧录、软件启动、完整BUILD ELF核对成功，但正常采集未启动：BOOT/READY明确 `stage=scan_task_create_failed,err=0x101,ready=0,seq=0`。SPI扫描尚未开始，ADSCHK=TIMING未发送。原boot observer的CONFIG超时ERROR和独立只读定位记录均保留，[失败证据](docs/audit/artifacts/cdc_recovery_acceptance_20261116/timing-startup-failure-01.json)。

The `bb230d0` app-only write, software startup and full BUILD verification succeeded, but acquisition did not start. BOOT/READY report `stage=scan_task_create_failed,err=0x101,ready=0,seq=0`. SPI qualification had not started and no timing action was sent. The original CONFIG timeout and independent read-only investigation remain preserved.

链接map显示新增3个2976B诊断结果槽进入内部BSS，共8928B；采集任务需要16384B连续内部栈。BOOT的约8.3MB总heap包含PSRAM，不代表存在所需连续内部块。正在将诊断结果槽改为一次、显式PSRAM分配，无内部fallback，分配失败只使诊断unavailable，不能阻止矩阵启动；原采集栈及watchdog不变。后续同板重测才能闭合此资源修正的因果验证。

The link map places three2976-byte diagnostic result slots in internal BSS, totaling8928bytes. The acquisition task needs a contiguous16384-byte internal stack. BOOT's approximately8.3MB total heap includes PSRAM and does not establish that internal capacity. The correction uses one explicit PSRAM allocation with no internal fallback; allocation failure must disable only the optional diagnostic, while retaining the acquisition stack and watchdog. A same-board rerun is required to close the causal comparison.

## 已完成的内存修正、时序对照与第三轮筛查 / Memory repair, timing comparison and third screen

当前已烧录并核对完整BUILD的是 `8b1ce9f8a6612498e8338f822ad63ec71e8444c6`（BUILD `8b1ce9f-dirty`），app1606272B，bin SHA256 `ead6850d358e5831e3ef4b84ba260504ba1ba7e8abdeaa9b3440201b7f5ef63b`，ELF SHA256 `1e501ba784cdf95cd9664d71ebafc218e44e5ba44e2156908af4554da33cef5f`。dirty为文档/测试工具；183个生产输入hash与60个已通过host入口清单相符。诊断固定槽改为显式PSRAM分配后，同板正常进入CAP8，BOOT ready1/err0/seq148，SPI选速完成。早期失败镜像也通过一次USBBOOT进入ROM，未按键或拔插；旧失败不改写。

The currently flashed and fully BUILD-verified image is `8b1ce9f` (reported as `8b1ce9f-dirty`), with the binary and ELF hashes above. Dirty entries are documentation/test tooling;183 production inputs match the60 passing host entries. After moving diagnostic storage to explicit PSRAM, the same board starts normal CAP8 acquisition (BOOT ready1/err0/seq148) and completes SPI qualification. Even the failed early-startup image entered ROM through one USBBOOT, without a button or cable operation. Its original failure is retained.

在同一boot和750000Hz请求/754000Hz驱动观测时钟下，11个独立TIMING请求均完整结束，事务总时长603371–612709µs，恢复readback通过且配置身份一致。每次四组200次，共8800个主读、4400个held诊断重读。输入为AINCOM对自身的AA短接，位于CAP8完整帧边界，**不是矩阵VOLT运行期错误率或绝对精度测试**。

Eleven independent TIMING requests completed on one boot at requested750000/observed754000Hz. Whole transactions took603371–612709µs, with verified register restoration and unchanged configuration identity. Four groups of200 reads per request produce8800 primary and4400 held diagnostic reads. The AA input shorts AINCOM to itself at CAP8 frame boundaries; **this is not the runtime VOLT matrix error rate or an absolute accuracy test**.

| 组 / Group | 主读 / Primary reads | CRC拒收 / CRC rejected | held重读拒收 / Held reread rejected |
|---|---:|---:|---:|
| CONT / PGA1 | 2200 | 5 | N/A |
| PULSE / PGA1 | 2200 | 0 | 0 / 2200 |
| CONT / bypass | 2200 | 4 | N/A |
| PULSE / bypass | 2200 | 0 | 0 / 2200 |

全部9个拒收仍为CRC最低位0→1 syndrome；坏包独立复算一致，没有修位、填零或进入科学数据。该结果支持转换状态/读数时序参与错误，但固定组序、每次START及held读取节奏也不同，不能只归因RUNMODE。新GPIO末位候选仍在开发，尚未烧录/运行：只在确认专用SPI owner和DMA回收后，用真实GPIO采最后一位，明确分开SPI55bit与GPIO1bit证据；不根据computed CRC猜位，不直接改科学路径。见[完整对照](docs/audit/artifacts/cdc_recovery_acceptance_20261116/timing-comparison-01.json)。

All nine rejections have the same least-significant-CRC-bit zero-to-one syndrome and independently verified CRC calculation. No bit is repaired or admitted into science. The result supports a role for conversion/read timing, but fixed order and START/held-read cadence prevent attribution to RUNMODE alone. A separate GPIO-tail diagnostic candidate is under development and has not been flashed or run. It requires dedicated SPI ownership and reclaimed DMA, physically samples the last bit, and records SPI55-bit and GPIO1-bit evidence separately. It must never infer the bit from computed CRC or directly change scientific acquisition.

[第三轮120秒筛查](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-03/screen-03-verdict.json)仍为**FAIL_BLE_THROUGHPUT**。Capture66.360471FPS，USB66.331356FPS/7963帧/2个序号缺口，BLE35.680964FPS/4283帧/3682个缺口；ADC CRC2433/104722=2.323294%，诊断跨度119.564089s。两路ASCII CRC/identity错误0，窗口未观察MFAULT/reset。TIMING命令不在该采集窗口内，不把其held计数混入本错误率。临时RAM轨值为用户ADS供电脚读数的+3.350/−1.795V代表值，未CAL=SAVE；不是精度验收。

The third full120-second screen still **fails BLE throughput**: capture66.360471FPS; USB66.331356FPS/7963frames/2missing observations; BLE35.680964FPS/4283frames/3682missing observations; ADC CRC2433/104722=2.323294% over119.564089seconds of diagnostic receipts. Neither receiver reports ASCII CRC or identity errors, and no MFAULT/reset was observed in the window. No TIMING command ran within it, so held rereads do not enter this ratio. The user's ADS-pin DMM range supplies only nominal+3.350/−1.795V volatile RAM rails, without CAL=SAVE or precision acceptance.

三轮实录中共取得549条完整CRC坏包，523条最低位0→1、26条1→0，全部syndrome为01；独立多项式复算一致。此为限量坏包样本，并非全部尝试或物理根因的充分证明。[联合证据](docs/audit/artifacts/cdc_recovery_acceptance_20261116/crc-three-screens-01.json)。

Across three real captures, all549 complete rejected packets share syndrome01:523 have a zero-to-one least-bit difference and26 the reverse. Independent polynomial division agrees. These rate-limited bad-packet samples do not by themselves prove the corruption location or physical cause. [Combined evidence](docs/audit/artifacts/cdc_recovery_acceptance_20261116/crc-three-screens-01.json).

## 4bf31db 末位对照结果 / Final-bit comparison results

当前已烧录并核对完整BUILD：`4bf31dbc576cf5061e1694102f82ad2111ef5392`，BUILD `4bf31db-dirty`，app1612752B，bin SHA256 `366994b942f643eb1c688291efbe22694cfa7f2d1336a1d2c07f9cfd6c10b08a`，ELF SHA256 `a2557c8f07f1e1735313315dd65bd71f4e2b79b58bbf284338888fa9e2220d5c`。61个host入口全部exit0，实际ESP-IDF构建通过，183个生产输入hash一致。[验证清单](docs/audit/artifacts/cdc_recovery_acceptance_20261116/tail-io-01-validated-identity.json)。软件从前一版DEGRADED进入ROM、app-only烧录和退出下载全部成功；新boot session `18407FDC95220620` 正常CAP8，未CAL=SAVE，未物理按键。

The currently flashed image is `4bf31db-dirty`, with the full commit, binary and ELF identities above. All61 host entries and the actual ESP-IDF build pass;183 production input hashes match. Software ROM entry from the previous DEGRADED state, app-only programming and software boot all succeed. The new session `18407FDC95220620` runs CAP8 without CAL=SAVE or a physical button operation.

11次独立TAIL请求（编号为日志tail-probe-02至12）均完整恢复，配置身份不变，使用同boot750000Hz请求/754000Hz驱动观测时钟：

| CONT诊断组 / Continuous diagnostic group | 校验读数 / Checked reads | CRC拒收 / CRC rejected |
|---|---:|---:|
| 标准SPI、PGA1 / Standard SPI, gain1 | 2200 | 7 |
| SPI55+GPIO1、PGA1 / Physical final-bit sample, gain1 | 2200 | 0 |
| 标准SPI、bypass / Standard SPI, bypass | 2200 | 8 |
| SPI55+GPIO1、bypass / Physical final-bit sample, bypass | 2200 | 0 |

保存的44条GPIO分段原始观察包含0和1两种末位，主机独立多项式复算全部一致；这44条不是全部4400次逐条原始波形。新IO查询实测MISO `OE=1/peripheralOE=1/signal67`、SCLK signal66，门禁修复后通过。55位SPI的未接收LSB始终明确unknown，最终位来自真实GPIO读取，不根据计算CRC填补。不把成功的GPIO/寄存器回读称为模拟隔离或无毛刺验证。[原始结果及来源边界](docs/audit/artifacts/cdc_recovery_acceptance_20261116/tail-comparison-01.json)。

All11 independent TAIL requests complete restoration with unchanged configuration identity on the same boot and clock. Standard SPI rejects15/4400 reads; physical final-bit sampling rejects0/4400. The44 saved split observations contain both0 and1 tail bits and independently recompute correctly; they are not individual waveform records of all4400 reads. Actual IO queries show MISO OE1/peripheralOE1/signal67 and SCLK signal66. The unreceived SPI prefix bit remains explicitly unknown; only the real GPIO sample supplies it. Digital readback does not prove analogue isolation or glitch-free signalling.

这个固定组序的短接输入实验改变了最后一个时钟的暂停和采样时刻，支持末位时序假设，但没有示波器证据可唯一归因于某个硅片缺陷。**正式矩阵/电池/rail读取尚未采用新方案，不能宣称运行期CRC已修复。** 启动门槛仍2.5%。SPI根因修复候选正在评审，600秒和32配置验收仍NOT_RUN。

The fixed-order shorted-input experiment changes both the final-clock pause and sample timing. It supports a last-bit timing mechanism without uniquely proving a silicon/pad defect. **Normal matrix, battery and rail reads still use their existing path; runtime CRC is not yet repaired.** Startup qualification remains2.5%. A production repair is under review; the600-second and32-configuration acceptance runs remain NOT_RUN.

BLE TX8候选在原调度压力模型中未通过数据生产进度断言，未烧录，已撤回到TX7/CTRL8；失败原记录和候选patch保留。该模型Core0生产者对应AsyncLog，并非Core1采集；BLE永远ready假设不含真实四槽依赖，不能把模型计数当实机FPS。候选未通过的事实不被改写，也不放宽控制时限来制造PASS。[撤回记录](docs/audit/artifacts/cdc_recovery_acceptance_20261116/ble-tx8-retraction-coordination-01.json)。

The TX8 BLE candidate fails a producer-progress assertion in the existing scheduling stress model and is withdrawn before flashing; TX7/CTRL8 remains. Failure logs and the candidate patch are retained. Its Core0 producer represents AsyncLog, not Core1 capture, and its perpetually-ready BLE assumption omits the real four-slot dependency; model counts are not hardware FPS. The failure is not relabelled and control deadlines are not relaxed.

## bc99767 实机失败保留 / Retained hardware failures

第四轮完整120.000145秒VOLT1筛查为 **FAIL_BLE_THROUGHPUT_AND_ADC_CRC**：capture65.900556FPS、USB65.811374FPS/7897帧/15个缺口、BLE31.815184FPS/3816帧/4092个缺口。实际BLE interval6×1.25ms=7.5ms、MTU247、2M PHY协商成功，但吞吐仍不足。ADC诊断窗119.390852s内5168/106486=4.853220%拒收，门槛仍2.5%；两路ASCII CRC/identity错误0，无MFAULT/reset观察。启动SPI为1.111MHz，不能把跨boot CRC或吞吐变化单独归因于连接间隔。新日志366条完整坏包均syndrome01，独立复算一致。[完整结果](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-04/screen-04-verdict.json)、[坏包证据](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-04/crc-tail-review-01.json)。

The fourth full120.000145-second VOLT1 screen **fails BLE throughput and ADC CRC**: capture65.900556FPS; USB65.811374FPS/7897frames/15missing observations; BLE31.815184FPS/3816frames/4092missing observations. The actual7.5ms interval, MTU247 and2M PHY are negotiated, but throughput remains insufficient. ADC rejects5168/106486=4.853220% over119.390852seconds of diagnostic receipts against the unchanged2.5% criterion. Both receivers have zero ASCII CRC/identity errors, with no MFAULT/reset observed. This boot's1.111MHz SPI clock and changed diagnostic load confound direct causal attribution to the BLE interval. All366 complete new bad-packet records share syndrome01 and agree with independent CRC calculation.

随后首次TAIL请求id7 **失败**，总事务123802µs：标准组200/200有效；GPIO组第1次读在提交55位SPI前被拒绝，后两组NOT_RUN。first/terminal=0x103、cleanup=0、restoreKnown/Verified=1；主状态转DEGRADED/软件SAFE、revision6→7，physicalSafe仍unknown。旧runner将这次合法失败rev变化当身份冲突提前停止收证；已用同ELF/session/id的纯只读查询取回完整缓存，未重发动作。源码核对确认门禁不兼容IDF的MISO INPUT_OUTPUT/peripheral OE设置，但旧记录没有保存具体IO gate，故不假称已观测实际GPIO寄存器。修正和IO上下文扩展进行中。[原始失败与边界](docs/audit/artifacts/cdc_recovery_acceptance_20261116/tail-prequeue-failure-01.json)。

The subsequent first TAIL request, id7, **fails** after123802µs. The standard group completes200 valid reads; the first GPIO-group read is rejected before SPI55 submission, and the other groups are NOT_RUN. First/terminal error is0x103, cleanup0 and register restoration verified. The main state enters DEGRADED/software SAFE at revision6→7; physical safety remains unknown. The old runner mistakenly rejected this valid failure revision transition before collecting detail. A same-ELF/session/id read-only query retrieved the complete cache without repeating the action. Source review identifies a gate incompatible with IDF's MISO INPUT_OUTPUT/peripheral-OE setup, but the old record contains no individual IO gate/config snapshot. The admission repair and explicit IO evidence are in progress.

## 末位诊断候选 bc99767 / Final-bit diagnostic candidate bc99767

本轮新增 `ADSCHK=TAIL,200`：固定四组CONT×标准SPI/末位GPIO×PGA1/bypass对照；仍用原启动选定时钟。独立owner门禁、持久DMA描述符精确回收后才接管SCLK、真实GPIO位、完整恢复和隔离失败路径均有host回归。结果固定槽仍只在启动时分配PSRAM；实机查询为14328B。新BLE补丁 `df68a46` 明确请求7.5ms，但实际间隔须以GAP回调/查询及实收为准。两者均不放宽CRC门槛，也不把坏CRC填成有效点。

The new `ADSCHK=TAIL,200` runs four continuous-conversion groups comparing standard SPI versus a physically sampled final GPIO bit at gain1/bypass, using the existing selected clock. Host regression covers exclusive ownership, exact persistent-DMA reclamation before SCLK handoff, real GPIO observations, full restoration and quarantine failures. Fixed result storage is allocated only once in PSRAM; the board reports14328B. BLE patch `df68a46` explicitly requests7.5ms, whose actual negotiation and throughput still require hardware confirmation. Neither change loosens CRC qualification or admits rejected data.

生产提交 `bc99767`，BUILD `bc99767-dirty`；app1610960B、bin SHA256 `47a9ff30ffd60232d12a39ebf19def30e72a19d81a56c3504a0ba43089aa945e`、ELF SHA256 `2cd47eb02b556120fc9459a0cc2ec9c394ca95ac5313a7c4284b4267800cf93b`。61个host入口初跑60通过，presence旧提取器编译失败；修正后单项21场景通过，失败原录保留。实际IDF构建成功，生产输入与测试逐hash一致。app-only烧录及独立BUILD核对成功，READY1、CAP8，boot session `4C618C351450A839`；启动选中1100000Hz请求/1111000Hz驱动观测值，不把它与此前754000Hz跨boot直接当同条件结果。[构建和回归](docs/audit/artifacts/cdc_recovery_acceptance_20261116/tail-01-validated-identity.json)、[烧录](docs/audit/artifacts/cdc_recovery_acceptance_20261116/tail-01-flash-usj-result.json)、[独立启动核对](docs/audit/artifacts/cdc_recovery_acceptance_20261116/tail-01-boot.summary.json)、[READY](docs/audit/artifacts/cdc_recovery_acceptance_20261116/tail-ready-01.summary.json)。

Production commit `bc99767` reports BUILD `bc99767-dirty`; sizes and full binary/ELF hashes are above. Of61 host entries,60 initially passed; an old presence-source extraction boundary failed to compile. Its repaired21-case rerun passes, with the original failure retained. The actual IDF build passes and its production input hashes match the test run. App-only flashing and an independent full BUILD query succeed; READY1/CAP8 uses session `4C618C351450A839`. Startup selects requested1100000/driver-observed1111000Hz, so cross-boot comparison against754000Hz is not a controlled same-clock experiment. TAIL HIL and new BLE throughput results are pending at this entry.

## 历史 Serial/JTAG 修复与失败基线 / Historical Serial/JTAG repairs and failed baselines

以下“当前”表述保留当时阶段语境，均不是本轮 OTG CDC 的版本或验收结论。 / The following historical references to “current” describe their original stages, not the present OTG CDC image or acceptance outcome.

**当前修复源码为 `d314888`；51 个 host 测试入口与目标构建通过，但 screen-04 仍为负载验收 FAIL，完整长测尚未开始。** 已完成 app-only 烧录和 VOLT1 同步 USB+BLE 120 s 筛查：Capture 69.013312 FPS，USB 42.820965 FPS、BLE 43.300331 FPS，分别有 3,143 / 3,082 个未观察到的序号。该窗口未观察到 watchdog、MFAULT、reset、完整帧 ASCII CRC 或 identity 错误；这不能消除 DATA 排队拒绝、吞吐不足或测量有效性问题，也不是精度 PASS。

**Current repair source is `d314888`; all 51 host test entries and the target build passed, but screen-04 still failed load acceptance and the full long suite has not started.** App-only flashing and a 120 s simultaneous VOLT1 USB+BLE screen completed: capture 69.013312 FPS, USB 42.820965 FPS and BLE 43.300331 FPS, with 3,143 / 3,082 missing sequence observations respectively. No watchdog, MFAULT, reset, complete-frame ASCII CRC or identity error was observed in this window. This does not remove DATA admission rejection, insufficient throughput or measurement-validity issues, and is not an accuracy pass.

当前源码 / current source: `d31488848844667dfa88502bf1b894d7bfe571a2`；app `d314888-dirty`，1,562,224 B，SHA256 `e63e383eec05460e613e1cf81664c1e5c0ef0ce7a7aa68592b8905ff756e3fc7`。repair-08 的 51/51 host 入口 exit 0，manifest 为提交前 `4b665ac+dirty`；其 166 个构建输入与 51 个测试输入 hash 均与 repair-09 已提交构建一致。 / All 51 repair-08 host entries exited 0 under pre-commit `4b665ac+dirty`; its 166 build-input and 51 test-input hashes match the repair-09 committed-source build. 证据 / Evidence: [repair-08 tests](docs/audit/artifacts/load_reliability_fix_20261113/repair-08-tests-manifest.json)、[repair-09 build](docs/audit/artifacts/load_reliability_fix_20261113/repair-09-build-manifest.json)、[repair-09 app-only flash](docs/audit/artifacts/load_reliability_fix_20261113/repair-09-flash-result.json)。

| 已完成筛查 / Completed screen | Capture FPS | USB FPS；可信完整帧 / trusted complete frames | BLE FPS；可信完整帧 / trusted complete frames | USB / BLE 缺失序号 / missing sequences | 原判定 / Recorded outcome |
| --- | ---: | ---: | ---: | ---: | --- |
| screen-03：`4b665ac`，120 s | 70.663301 | 61.422020；7,371 | 39.968146；4,796 | 1,120 / 3,687 | FAIL |
| screen-04：`d314888`，120 s | 69.013312 | 42.820965；5,138 | 43.300331；5,195 | 3,143 / 3,082 | FAIL |

Capture 按有序可信帧的设备 sequence/timestamp 推算；实收 FPS 按各端首末可信完整帧接收跨度计算，缺失序号亦限定在各自范围，不是 gap 事件数，不能将两端直接相减。screen-04 的 USB 有效/无效点为 37,833/3,271，BLE 为 38,321/3,239；两端各有 11 点 fresh/acquired=0，未填零或伪装有效。来源：[screen-03 判定](docs/audit/artifacts/load_reliability_fix_20261113/iteration-03-evaluation.json)、[screen-04 独立分析](docs/audit/artifacts/load_reliability_fix_20261113/screen-analysis-05.json)、[分层诊断](docs/audit/artifacts/load_reliability_fix_20261113/screen-diagnostics-04.json)、[判定](docs/audit/artifacts/load_reliability_fix_20261113/iteration-04-evaluation.json)与[无损归档](docs/audit/artifacts/load_reliability_fix_20261113/evidence-index-screen-04.json)。

Capture is inferred from ordered trusted-frame device sequence/timestamp progression. Receive FPS uses each receiver's first-to-last trusted complete-frame receipt span; missing sequences also use each receiver's own range. They are not gap-event counts and cannot be directly subtracted between receivers. Screen-04 contains 37,833 valid / 3,271 invalid USB points and 38,321 / 3,239 BLE points. Each receiver has 11 points with fresh/acquired=0; none is zero-filled or relabeled valid. The linked decisions, independent analysis, layered diagnostics and lossless archive retain the evidence.

当前待修的调度服务问题是 USB TX 任务优先级 4 低于同核 BLE/transport 的 5；减少 checkpoint 等待后 DATA 容量拒绝仍增加。下一步是优先级 6 候选的最小模型验证与新镜像筛查，**不是 d314888 已实施或已验收的修复**。用户接受 Serial 输出 FPS 低于 capture：后续判据是差额有分层计数、控制通路不饥饿、等待有界且无已确认阻塞/P0/P1，而非强迫 USB/BLE FPS 都等于 capture。此口径不倒改历史 FAIL，也不放宽 ADC CRC 的 2.5% 要求。

The remaining scheduling-service issue is USB TX priority 4 below same-core BLE/transport priority 5; DATA capacity rejection increased despite fewer checkpoint waits. Next comes minimal-model validation of a priority-6 candidate and a new-image screen; **this is not an implemented or accepted d314888 repair**. The user accepts Serial output FPS below capture. Subsequent acceptance requires layered accounting of the difference, a control path that is not starved, bounded waits, and no confirmed blocker/P0/P1; it does not require USB and BLE FPS to equal capture. This criterion does not retrospectively change historical FAIL results or relax the ADC CRC 2.5% requirement.

历史增量仍保留：repair-06 候选 50/50 host 入口与构建通过，repair-07 完成 `4b665ac` 构建/烧录后得到上表 screen-03 FAIL。 / Historical increments remain: the repair-06 candidate passed 50/50 host entries and its build; repair-07 built/flashed `4b665ac`, followed by screen-03 FAIL above. [repair-06 tests](docs/audit/artifacts/load_reliability_fix_20261113/repair-06-tests-manifest.json)、[repair-06 build](docs/audit/artifacts/load_reliability_fix_20261113/repair-06-build-manifest.json)、[repair-07 build](docs/audit/artifacts/load_reliability_fix_20261113/repair-07-build-manifest.json)、[repair-07 flash](docs/audit/artifacts/load_reliability_fix_20261113/repair-07-flash-result.json)。

以下镜像表对应已完成历史筛查的 `b44c37b`，不是尚未完成验收的新镜像。 / The image table below identifies the completed historical `b44c37b` screening, not a newly accepted image.

| 上次实机筛查镜像 / Previous screened image | 关联记录 / Association |
| --- | --- |
| 源码 / Source | `b44c37b51067e376e73aa44535ea47d175846b67` |
| app | `b44c37b-dirty`，1,561,568 bytes |
| app SHA256 | `d29289eeb6f93de96adfb59f7cf8a2e2142ecc9e46baf510355ce76a01ddb531` |
| 构建与烧录 / Build and flash | [repair-05 build manifest](docs/audit/artifacts/load_reliability_fix_20261113/repair-05-build-manifest.json)、[repair-05 flash result](docs/audit/artifacts/load_reliability_fix_20261113/repair-05-flash-result.json)。仅写 app；未写 NVS、校准、分区表或 bootloader。 / App only; NVS, calibration, partition table, and bootloader were not written. |

前一修复版 **`fd0190a` 已通过 ESP-IDF 构建及 50 个 host 测试入口，但未通过性能验收**：其 VOLT1 USB+BLE 120 秒筛查没有 watchdog/帧 CRC/身份错误，却暴露了调度造成的吞吐回退。`b44c37b` 随后加入仅由真实 idle 更新的两 work-unit 宽限；不能把 fd0190a 的首次筛查当作本版结果。两次启动选中的 SPI 都是 requested 750 kHz / IDF actual 754 kHz；这项相同条件不等于新性能已验证。机制及历史证据见 [负载修复报告](docs/audit/load_reliability_fix_20261113.md) 和 [首次调度诊断](docs/audit/artifacts/load_reliability_fix_20261113/work-budget-screen-01-diagnosis.json)。

The preceding repair **`fd0190a` passed the ESP-IDF build and 50 host test entries but failed performance acceptance**: its 120-second VOLT1 USB+BLE screening had no watchdog/frame CRC/identity error, yet exposed a scheduling-related throughput regression. `b44c37b` subsequently added a two-work-unit grace renewed only by actual idle progress; the first fd0190a screening is not a result for this version. Both startups selected requested 750 kHz / IDF actual 754 kHz SPI; matching this condition does not establish new performance. See the [load repair report](docs/audit/load_reliability_fix_20261113.md) and [first scheduling diagnosis](docs/audit/artifacts/load_reliability_fix_20261113/work-budget-screen-01-diagnosis.json).

`b44c37b` 的第二次同步 VOLT1 USB+BLE 120 s 窗口已完成分析：Capture 70.6880 FPS、USB 67.3570 FPS（8,083 可信完整帧、397 个缺失序号）、BLE 36.1023 FPS（4,332 可信完整帧、4,144 个缺失序号），两端 ASCII CRC/identity 错误均为 0；吞吐与背压仍需修复。随后同镜像 USB-only 的 collector 在 120.000351 s 记录 10,913 个完整 CRC 通过帧、窗口均值 90.9414 FPS、identity 错误 0；它与双输出不是相同负载，也不是精度验收。详见[第二次独立分析](docs/audit/artifacts/load_reliability_fix_20261113/screen-analysis-02.json)与 [USB-only 原 summary](docs/audit/artifacts/load_reliability_fix_20261113/repair-screen-02-usb-only.summary.json)。这两次均早于 DATA/LOG 分离与 BLE 批次修复，不能用来宣称新版本已通过。

Analysis of the second `b44c37b` simultaneous VOLT1 USB+BLE 120 s window is complete: capture 70.6880 FPS, USB 67.3570 FPS (8,083 trusted complete frames and 397 missing sequence observations), and BLE 36.1023 FPS (4,332 trusted complete frames and 4,144 missing sequence observations), with zero ASCII CRC/identity errors on both receivers. Throughput and backpressure still require repair. The later same-image USB-only collector recorded 10,913 complete CRC-valid frames over 120.000351 s, averaging 90.9414 FPS over the window, with zero identity errors. This differs from simultaneous-output load and is not accuracy acceptance. See the [second independent analysis](docs/audit/artifacts/load_reliability_fix_20261113/screen-analysis-02.json) and [original USB-only summary](docs/audit/artifacts/load_reliability_fix_20261113/repair-screen-02-usb-only.summary.json). Both precede DATA/LOG isolation and BLE batching and cannot establish acceptance of the new version.

### 0d8f60f 历史失败基线（原记录保留） / Historical 0d8f60f failed baseline (original record retained)

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

`PORT` 替换为核实身份后的实际端口，如 `COMx` 或 `/dev/ttyACM*`；上面的标准烧录命令要求设备已处于下载状态。当前应用使用 USB-OTG 双 CDC `303a:4042`，下载器使用 USB Serial/JTAG `303a:1001`，同一数据线在两个状态下重新枚举；不能把应用正式口直接当作 esptool ROM 端口。每接口只允许一个 reader，采集前关闭该接口的 monitor。本机标准 RTS-only 烧录退出未能启动应用，已验证的 USBBOOT、app-only 写入和单次软件退出流程见[恢复记录](docs/audit/cdc_recovery_acceptance_20261116.md)，不要靠自动重复重启掩盖失败。两种 USB 后端均为原生 Full Speed，tty baud 不决定 12 Mbit/s 总线速率，也不代表 PC 实收吞吐。

Replace `PORT` with an identity-verified device such as `COMx` or `/dev/ttyACM*`; the standard flash command above requires download mode. The current application uses USB-OTG dual CDC `303a:4042`, while the downloader uses USB Serial/JTAG `303a:1001`. The same cable re-enumerates between those states; the formal application port is not an esptool ROM port. Use one reader per interface and close its monitor before capture. Here, standard RTS-only exit did not start the application. The validated USBBOOT, app-only write and single software-exit procedure is documented in the [recovery record](docs/audit/cdc_recovery_acceptance_20261116.md); do not hide failure with automatic reset loops. Both backends use native Full Speed USB. The tty baud does not determine its12Mbit/s signaling rate or establish PC receive throughput.

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
SERIAL?
SCHED?
RECOVERY?
RESTART?
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

`RECOVER` 默认 full/level 1，也支持 `RECOVER=0|1|2`；level 2 请求受预算约束的重启，`RESTART` 是独立的显式重启请求。两者都须先成功提交本地接受回复才允许执行；接受回复不证明 PC 已收到。丢失终态时使用无副作用的 `RECOVERY?` / `RESTART?`，不要为查询结果而重发动作。真实重启后需按新 session 重新查询，RAM RAILCFG 需重新提供；显式重启后的 boot 变化不是未解释的崩溃。

`RECOVER` defaults to full/level 1 and accepts `RECOVER=0|1|2`; level 2 requests a budgeted restart, while `RESTART` is an independent explicit restart request. Both require successful local acceptance-reply submission before execution; that reply does not prove PC receipt. If a terminal result is lost, use the side-effect-free `RECOVERY?` / `RESTART?` instead of resending the action to discover its result. After an actual restart, query the new session and reapply RAM RAILCFG. A boot change after an explicit restart is not an unexplained crash.

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

### CPU0 工作预算与真实 idle / CPU0 work budgets and actual idle progress

七个应用 worker 共用工作预算观测：BLE TX、transport、异步日志、Serial TX，以及 BLE/Serial/Wi-Fi CTRL。它们在不持硬件、队列或 stdio 锁的工作单元边界检查 CPU0 idle hook 的真实 generation。`SCHED?` 的 `bt/tr/log/ser/bc/sc/wc` 依次对应这七个 worker，每组为 `checkpoints/yields/maxBurstUnits`；idle generation、年龄和最大间隔是实际 idle 进度观测，**不是 CPU 使用率，也不是替采集任务喂 watchdog**。未知时输出 `idleKnown=0` / `unknown`。

Seven application workers share the work-budget observation: BLE TX, transport, async logging, Serial TX, and BLE/Serial/Wi-Fi CTRL. At work-unit boundaries outside hardware, queue, and stdio locks, they check the actual generation from the CPU0 idle hook. `SCHED?` maps `bt/tr/log/ser/bc/sc/wc` to these workers, each as `checkpoints/yields/maxBurstUnits`. Idle generation, age, and maximum gap observe actual idle progress; **they are not CPU utilization and do not feed the watchdog on behalf of acquisition**. Unavailable observations report `idleKnown=0` / `unknown`.

`d314888` 的 `work_quantum_v2` 将每个 worker 的本地时间片与真实 idle 失联保护分开：当前 100 Hz 下，本地工作跨度达到 **2 ticks（约 20 ms）**或累计 **32 work units** 即在检查点请求一次 `vTaskDelay(1)`；真实 CPU0 idle 年龄达到 **4 ticks（约 40 ms）**则独立触发 backstop。本地时间片在让出后重新开始，不把自己的 delay 计作真实 idle；仅 idle hook 更新共享 generation。新真实 idle 也可重置本地 burst。旧两 work-unit 宽限已取消，兼容字段 `graceUnits=0`；这里的“2”是 ticks，不是允许两条记录。

In `d314888`, `work_quantum_v2` separates each worker's local time slice from protection against absent actual idle progress. At the current 100 Hz, **2 ticks (about 20 ms)** of local work or **32 work units** requests one `vTaskDelay(1)` at a checkpoint. An actual CPU0 idle age of **4 ticks (about 40 ms)** independently triggers the backstop. A local slice restarts after yielding without treating that delay as actual idle; only the idle hook updates the shared generation. Newly observed actual idle can also reset the local burst. The former two-work-unit grace is removed, with legacy `graceUnits=0`; “2” now means ticks, not two permitted records.

每次检查最多请求 1 tick 延时，不循环等 idle，不重启原事务 deadline，也不永久逐记录限速或限制 Core1 FPS。它针对旧全局 idle 年龄与低优先级服务相位耦合的过度让出；screen-04 说明仍不能仅靠该修复保证输出吞吐。单次 SDK 调用、格式化单元和 SDK 自有任务没有因此获得硬执行时间上限。新机制的[独立 10 场景审阅](docs/audit/artifacts/load_reliability_fix_20261113/independent-work-budget-quantum-review.json)为 host 证据；原[两 unit 宽限模型](docs/audit/artifacts/load_reliability_fix_20261113/independent-work-budget-grace-model.json)作为已替代设计保留。

Each check requests at most one tick of delay, without repeatedly waiting for idle, renewing transaction deadlines, permanently throttling each record or capping Core1 FPS. This addresses excessive yielding caused by coupling global idle age to lower-priority service timing; screen-04 shows it does not by itself establish output throughput. Individual SDK calls, formatting units and SDK-owned tasks do not acquire a hard execution-time bound. The [independent ten-scenario review](docs/audit/artifacts/load_reliability_fix_20261113/independent-work-budget-quantum-review.json) is host evidence; the [two-unit grace model](docs/audit/artifacts/load_reliability_fix_20261113/independent-work-budget-grace-model.json) remains as superseded-design history.

共享观测量采用对齐的片内 DRAM 32-bit 原子访问，并核对 Xtensa 目标实现；idle hook 不打印、不分配、不取应用锁。初始化失败和内存放置异常有明确状态与有界回退，不能把“注册成功”伪装成“idle 已运行”。实现与回归见 [WorkBudget](components/sensorarrayWorkBudget/sensorarrayWorkBudget.c)、[目标审阅](docs/audit/artifacts/load_reliability_fix_20261113/independent-work-budget-target-fix-review.json)。

Shared observations use aligned 32-bit atomics in internal DRAM, with the Xtensa target implementation checked. The idle hook does not print, allocate, or take application locks. Initialization failure and invalid memory placement have explicit status and bounded fallback; successful registration is not presented as proof that idle has run. See [WorkBudget](components/sensorarrayWorkBudget/sensorarrayWorkBudget.c), the [target review](docs/audit/artifacts/load_reliability_fix_20261113/independent-work-budget-target-fix-review.json).

`4b665ac` 保留 BLE TX 每个 descriptor 的检查点；同一大消息的分片之间，在已发最多 4 片或自上次检查经过至少 1 tick、且仍有下一片时，再检查共享预算。普通快速三片消息不再把每个分片重复计为独立预算工作单元。每一片仍检查连接 generation，暂停仍保留原 descriptor/slot；单次 SDK 调用时间仍无硬上限。**`SCHED.bt` 的工作单元现在是 descriptor/分片批次，不是每个 BLE 分片**，跨版本的 checkpoint 数不能直接当同一吞吐单位比较。

`4b665ac` retains the BLE TX checkpoint at every descriptor. Within a larger message, another shared-budget check occurs between fragments after at most four transmitted fragments or at least one elapsed tick, when another fragment remains. Ordinary fast three-fragment messages no longer count every fragment as a separate budget work unit. Each fragment still checks connection generation, and a pause retains the original descriptor/slot; a single SDK call still has no hard execution-time bound. **The `SCHED.bt` work unit is now a descriptor/fragment batch, not each BLE fragment**; checkpoint counts across versions are not directly comparable throughput units.

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

`d314888` 另修复 PGA 不稳定分支的 cache 训练错误：三笔新鲜、CRC 通过的样本即使 spread 超限，中位数重复仍可能增加 stableStreak，导致下一组尚不稳定的数据只读一笔便快速接纳。新增九行在该分支清零 stableStreak、饱和递增 unstableStreak，保留诊断历史；新的稳定全样本序列才重新允许快速路径。生产函数的旧版反例与修复后 10 场景已实跑，CRC 拒绝不污染有效 cache/PGA，未提高 spread 阈值。此前 screen-02 的 X12 均在 bypass，不能把此 PGA 缺陷追认为那些实录的根因。见 [PGA 稳定性修复与原失败](docs/audit/artifacts/load_reliability_fix_20261113/ads-stability-fix-review.json)和 [repair-08 实跑](docs/audit/artifacts/load_reliability_fix_20261113/repair-08-test_ads_cell_stability.txt)。

`d314888` also fixes cache training in the unstable PGA branch: three fresh CRC-valid samples could exceed the spread limit while their repeated median increased stableStreak, allowing the next still-unstable set to be accepted after one read. Nine added lines reset stableStreak and saturating-increment unstableStreak in that branch while retaining diagnostic history; a new stable full-sample streak must qualify the fast path again. The actual production-function counterexample and ten repaired scenarios were executed. CRC rejection does not train valid caches/PGA, and the spread threshold is unchanged. All screen-02 X12 observations used bypass, so this PGA defect is not retrospectively claimed as their cause. The linked review retains the original failure and repaired test output.

### CRC 部分帧与全局故障 / CRC partial frames and global faults

ADC 数据 CRC8 与 ASCII 测量帧 CRC32 各自验证不同层，不能互相替代；ADC STATUS 不受该数据 CRC8 保护。当前每 cell 共用一次 I/O retry，受100 ms cell 总deadline约束，坏样本不更新PGA/有效cache。originalError 与 cleanupError 分开，`ADSFAULT*` 保存清理前上下文。

ADC data CRC8 and ASCII measurement-frame CRC32 protect different layers and cannot replace one another; ADC STATUS is outside the data CRC8 coverage. Each cell currently shares one I/O retry within a 100 ms cell deadline. Bad samples do not update PGA decisions or valid caches. originalError and cleanupError remain separate, and ADSFAULT diagnostics preserve pre-cleanup context.

`0d8f60f` 的 `boundedCrcFailure` 只允许**扫段已完成、软件所检查的路由/owner/寄存器状态仍一致、缺失仅由采样 CRC 拒绝引起**的部分帧输出；该许可不是模拟隔离或供电完整性的实测证明。点仍为 X02，valid/fresh/acquired 不伪造；首个可信转换前失败可为fresh=0，已有新转换后失败可为fresh=1，但均不是有效数值。后续timeout、RESET、配置变化、reference/owner/route错误或DMA未回收会否决许可，不能用首个CRC掩盖后错。

The `boundedCrcFailure` disposition in 0d8f60f permits a partial frame only when **the sweep completed, software-checked route/owner/register state remains coherent, and missing conversions are due solely to sample CRC rejection**. This permission is not measured proof of analogue isolation or supply integrity. Points remain X02 with truthful valid/fresh/acquired bits. Failure before a trusted conversion may have fresh=0, while failure after a new conversion may retain fresh=1; neither is a valid value. A later timeout, RESET, configuration change, reference/owner/route error, or unreclaimed DMA revokes permission. An initial CRC error must not hide a later fault.

连续三次活动点全因CRC拒绝的 **ADS engine扫段**停止继续发布；这不是三个主机 MIXED 帧。CAP/SetMode 不清预算，完成非CRC的ADS测量结论（含合法OPEN）才可清除。`ADSDISPOSITION` 公开 scope=ads_sweep、拒绝位图、真实 acquired/expected、allCrcRun 和 firstError/abortError。 / Three consecutive **ADS engine sweeps** whose active points are all CRC-rejected stop continuation; this is not three host MIXED frames. CAP/SetMode does not clear the budget; a completed non-CRC ADS conclusion, including legitimate OPEN, can clear it. ADSDISPOSITION exposes sweep scope, rejection masks, actual acquired/expected masks, allCrcRun, and firstError/abortError.

运行期 CRC 风险仍未闭环。历史 `b44c37b` USB-only screen-02 的周期 ADSHEALTH 首末接收跨度 119.612244 s 内，CRC 通过计数增加 140,360、拒绝增加 5,402，拒绝率为 **5,402 / (140,360 + 5,402) = 3.706041%**。分母是已校验 ADC1 数据响应，包含 retry/辅助读取；不能再加 retry，也不是科学点无效率或所有 ADC 产生转换的比例。对应双输出窗口为 1.646187%，不能互相替代。启动每个 PGA/bypass confirm 各 800 笔、各 0 CRC 拒绝，仅证明当时固定 MUX/负载的有限资格，不代表运行期已满足 2.5%。

Runtime CRC risk remains open. In the historical `b44c37b` USB-only screen-02, periodic ADSHEALTH endpoints span 119.612244 s and increase CRC-valid responses by 140,360 and rejections by 5,402: **5,402 / (140,360 + 5,402) = 3.706041%** rejected. The denominator counts checked ADC1 responses, including retry/helper reads; retries must not be added again. This is neither the invalid-scientific-point fraction nor the fraction of all ADC-generated conversions. The corresponding dual-output window is 1.646187%, a different condition. Boot confirmation passed 800 reads in each PGA/bypass profile with zero CRC rejection, establishing only finite qualification at that fixed MUX/load, not runtime compliance with 2.5%.

只读核验确认启动与矩阵使用同一命令式 RDATA/DMA 长度和 CRC 算法；手册的 holding/shift 双缓冲允许 command read 跨后续转换，不能仅凭 754 kHz 读取慢于 38.4 kSPS 就认定覆盖。有限原始诊断中，收到 CRC 与按 DATA 重算的 CRC 仅 bit0 不同（xor=01，重算为 0、收到为 1）；这不能单独证明故障发生在最后一个时钟或排除 DATA 多位差错。限频样本不能代表全部错误，电气/采样根因仍 **unknown**；未证明 CS 配置违规，也未通过旧 duty-cycle A/B 根治。当前没有运行期滚动 CRC 比率门禁或自动重新资格。详见[运行期 CRC 只读证据](docs/audit/artifacts/load_reliability_fix_20261113/adc-runtime-crc-readonly-review-01.json)。

Read-only verification found the same command-mode RDATA/DMA length and CRC algorithm in startup and matrix paths. Datasheet holding/shift buffers permit command reads across later conversions; reading at 754 kHz slower than the 38.4 kSPS conversion interval alone does not establish overwrite. In the bounded raw diagnostics, received CRC differs from CRC recomputed over DATA only at bit0 (xor=01, recomputed 0 and received 1). This alone does not prove failure on the final clock or exclude multiple DATA-bit errors. Rate-limited examples do not represent every rejection; the electrical/sampling cause remains **unknown**; no CS timing violation is proven, and earlier duty-cycle A/B testing did not resolve it. There is currently no runtime rolling-CRC-rate gate or automatic requalification. See the [runtime CRC read-only evidence](docs/audit/artifacts/load_reliability_fix_20261113/adc-runtime-crc-readonly-review-01.json).

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

### RECOVER/RESTART 的有界终态 / Bounded RECOVER/RESTART terminal results

`RECOVER` 有一个待执行槽和最后结果缓存，接受时捕获同 boot/session 的完整配置身份，包括 revision 和 mode/rows/profile 各域；执行前不匹配则以 `stale_config` 淘汰旧请求，不触碰新配置。它与配置命令共用终态保留仲裁；先预留空间并成功提交本地 ACK，再允许执行。忙时明确拒绝，不覆盖已接受请求；ACK 发布失败也记录拒绝结果，不悄悄执行。终态先缓存再进入保留槽，由优先路径投递给 Serial 和已启用网络；`RECOVERY?` 只读快照，可在终态丢失后重建结果。这里 `terminalQueued` 只证明终态进入本地保留槽，不证明每个 sink 已接纳或 PC 已收到。

`RECOVER` has one pending slot and a last-result cache. Acceptance captures the complete configuration identity within the boot/session, including revision and the mode/rows/profile domains. A mismatch before execution rejects the old request as `stale_config` without touching the newer configuration. Recovery shares terminal reservation arbitration with configuration commands: reserve space and successfully submit the local ACK before allowing execution. Busy requests are rejected without overwriting an accepted request; ACK publication failure is cached as rejection and does not silently execute. Results are cached before entering the reserved slot for priority delivery to Serial and enabled networks. `RECOVERY?` reads the snapshot without side effects so a lost terminal can be reconstructed. Here, `terminalQueued` proves only local reserved-slot admission, not acceptance by every sink or PC receipt.

恢复实际触碰硬件后失败，或 `RECOVER=2` 的重启预算耗尽，会保留 `originalError` 与首个 `cleanupError`，停止矩阵并发布 `NONE/DEGRADED` 的 CONFIG 权威状态；SAFE 动作失败不会标成 applied/known/verified 成功，也不会继续下一帧。软件/寄存器验证不证明模拟节点物理安全，`physicalSafe=unknown`。未触碰硬件的预检查拒绝不破坏原有合法配置；之后合法的 MODE 或显式 RESTART 仍可使用。

If recovery fails after touching hardware, or `RECOVER=2` exhausts its restart budget, it preserves `originalError` and the first `cleanupError`, stops matrix acquisition, and publishes authoritative CONFIG state `NONE/DEGRADED`. Failed SAFE actions are not reported as successfully applied/known/verified, and acquisition does not continue into the next frame. Software/register verification does not establish physical analogue safety: `physicalSafe=unknown`. Precondition rejection before hardware access leaves a valid existing configuration intact; a later valid MODE request or explicit RESTART remains available.

`RESTART` 有独立待执行槽与同 boot 最后结果，作为 quarantine/降级时的显式逃生路径，不因普通配置仲裁忙而永久拒绝。它同样不覆盖 pending、必须通过本地 ACK 栅栏；请求接受与实际执行的 revision 分别记录，整体重启不因期间配置 revision 改变而被误判为旧恢复请求。执行前缓存 restarting，分别提交 Serial priority 和网络 lifecycle，并给出有界发送机会；网络全关的 no-op 不能算已发送。`RESTART?` 可读各 sink 的接纳/错误，不把 `terminalQueued` 解释为 PC 确认。真实重启产生新 session，RAM 结果缓存和请求计数重新开始；两类请求 ID 各自成域，不能跨命令或跨 boot 比大小，也不提供跨重启网络 exactly-once。

`RESTART` has an independent pending slot and same-boot last result as an explicit escape from quarantine/degraded states; ordinary configuration arbitration cannot permanently block it. It also preserves pending requests and requires the local ACK fence. Acceptance and execution revisions are recorded separately; a whole-device restart is not incorrectly rejected as stale recovery merely because configuration changed in between. Before resetting, it caches `restarting`, explicitly submits to Serial priority and network lifecycle paths, and allows a bounded transmission opportunity. A no-op with all network sinks disabled does not count as a send. `RESTART?` exposes per-sink admission/errors, and `terminalQueued` is not PC acknowledgment. A real restart creates a new session and resets RAM result caches and request counters. Each command has its own request-ID domain, so IDs are not ordered across commands or boots, and no cross-restart network exactly-once guarantee is offered.

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

PROTO2保留原域/单位，扩展配置身份/查询、session64、启动SPI资格和frontend presence，并明确协商 `recovery_query_v1`、`restart_query_v1`、`serial_sink_v1`、`serial_classes_v1`、`work_budget_v1`、`work_quantum_v2`。先查询 PROTO?/BUILD?/CONFIG?。完整 DATA/LOG message 容量为 2304 bytes、CTRL reply 为 512 bytes，旧 1536-byte Host 缓冲须升级。容量拒绝、驱动 timeout、接收缺口与无效点独立计数，不相加成一个“丢包率”。

PROTO2 retains existing domains/units and extends configuration identity/query, session64, boot SPI qualification, and frontend presence, with explicit `recovery_query_v1`, `restart_query_v1`, `serial_sink_v1`, `serial_classes_v1`, `work_budget_v1`, and `work_quantum_v2` capabilities. Query PROTO?/BUILD?/CONFIG? first. Complete DATA/LOG message capacity is 2304 bytes and CTRL reply capacity is 512 bytes; old 1536-byte host buffers need upgrading. Capacity rejection, driver timeout, receive gaps, and invalid points are separate counts, not one combined loss rate.

| 层 / Layer | 可证明的事实与限制 / Evidence and limits |
| --- | --- |
| Capture | 设备完成扫帧及时间，不证明transport提交。 / Device sweep completion/timing, not transport submission. |
| Emitted/frame bus | 编码/本地队列接纳；原identity保留。 / Encoding/local admission; original identity retained. |
| USB | 唯一RX reader/TX owner；stdout/stderr分别组到LF后整行排队。拥塞/超长整行丢弃，lineDropped/lineOversize/discardBytes与partialWrite/timeout分开。 / One RX reader/TX owner; stdout/stderr separately assemble to LF before whole-line queuing. Congested/oversize lines are dropped; lineDropped/lineOversize/discardBytes differ from partialWrite/timeout. |
| USBSTREAM? | sinkQueued是本地提交，driverAcceptedBytes是驱动接收；stdioBound/driverInstalled/SOF不证明PC接收或COM枚举，enumerated=unknown。超过512B明确response_too_long，不截断伪统计。 / sinkQueued is local admission, driverAcceptedBytes is driver acceptance; stdioBound/driverInstalled/SOF do not prove PC receipt or COM enumeration, which remains unknown. Over-512B replies report response_too_long instead of truncated statistics. |
| SERIAL? | 独立有界 sink 统计：normalSubmitted/prioritySubmitted 是整条记录已提交驱动，normal 包括日志；normalExpired/priorityExpired/priorityRejected、driverShortWrite/driverWriteError、txYield、maxQueueAgeUs、driverTxBufferBytes 与 driverAcceptedBytes/partialWrite 分层保留。都不是 PC 实收计数。 / Separate bounded sink statistics: normalSubmitted/prioritySubmitted count complete records submitted to the driver; normal includes logs. Expiry/rejection, short-write/error, yield, queue age, ring size, accepted bytes, and partial writes remain distinct. None counts PC receipt. |
| SCHED? | 独立 512 B 缓冲；当前极值字符串 508 B，另留 NUL。含真实 idle、quantumTicks=2、budgetTicks/backstopTicks=4、maxUnits=32、graceUnits=0 与七 worker 三元组；计数独立采样，不是 CONFIG 原子状态或 CPU 利用率。 / A separate 512 B buffer holds the current 508 B extreme string plus NUL: actual idle, quantumTicks=2, budgetTicks/backstopTicks=4, maxUnits=32, graceUnits=0 and seven worker triples. Independently sampled counters are not CONFIG atomic state or CPU utilization. |
| BLE | 有界队列、分片、独立DATA/LOG/CTRL计数；Notify排队≠PC接收，Indicate确认≠应用处理。B20预览保留原identity，不替完整科学帧。 / Bounded queues, fragments, and separate DATA/LOG/CTRL counts; queued Notify is not PC receipt, nor Indicate acknowledgment application processing. Identity-preserving B20 previews do not replace scientific frames. |
| Wi-Fi UDP | 有界按通道提交，不保证送达/顺序/exactly-once。 / Bounded per-channel submission without guaranteed delivery/order/exactly-once. |
| Host接收 / Host receive | 原字节、完整帧CRC/identity、序号缺口与接收时间；与同窗口设备计数比较。 / Raw bytes, complete-frame CRC/identity, sequence gaps, receive times; compare with device counts from the same window. |

`4b665ac` 将 USB **DATA 与 LOG 各保留 5 个 payload slots、4 项 queue**，CTRL 独立 4 个较小 slots/queue entries；stdout/stderr 的未完成行只占 LOG，不再占 DATA。普通 payload 总容量 23,040 B，启动时优先 PSRAM、失败才尝试片内内存；比前版增加 11,520 B，无热路径 payload 分配。完整记录边界先选 CTRL，再选 DATA；LOG 等待时最多完成 4 条 DATA 就服务一条 LOG。已发生正短写的当前记录先完成或到期，不能为抢占而交织字节。该公平性是 DATA/LOG 间的有界轮转，不是持续 CTRL 洪流下的绝对带宽保证。

`4b665ac` gives USB **DATA and LOG five payload slots and four queue entries each**, with four independent smaller CTRL slots/queue entries. Incomplete stdout/stderr lines occupy LOG only, leaving DATA capacity available. Normal payload capacity totals 23,040 B, allocated once at startup with PSRAM preferred and internal memory as fallback, an increase of 11,520 B over the preceding version; there is no hot-path payload allocation. Whole-record boundaries select CTRL first, then DATA; a waiting LOG is served after at most four completed DATA records. A record with an accepted short-write prefix finishes or expires before another record can interleave bytes. This is bounded DATA/LOG rotation, not an absolute bandwidth guarantee under continuous CTRL flooding.

`serial_classes_v1` 为 `SERIAL?` 新增 `dataSubmitted/logSubmitted/dataRejected/logRejected`。submitted 只计驱动已接纳整条记录；rejected 只计相应 slots/queue 容量拒绝，不含非法长度调用或超长诊断行。旧 `normalSubmitted` 仍包括 DATA+LOG，`normalExpired` 仍为两类合计，不静默改变旧字段语义。新旧计数都不是 Host 实收。

`serial_classes_v1` adds `dataSubmitted/logSubmitted/dataRejected/logRejected` to `SERIAL?`. Submitted counts whole-record driver admission only; rejected counts the respective slot/queue capacity rejection, excluding invalid-size calls and oversized diagnostic lines. Legacy `normalSubmitted` still includes DATA+LOG, and `normalExpired` remains their combined expiry count; existing fields are not silently reinterpreted. Neither old nor new counters measure host receipt.

最终容量方案的生产回归与独立审阅见 [serial-admission-host-02](docs/audit/artifacts/load_reliability_fix_20261113/serial-admission-host-02.json) 和 [final admission/batch review](docs/audit/artifacts/load_reliability_fix_20261113/independent-load-review-final-admission-batch.json)。旧两 LOG slots 候选已被替代，不代表当前实现；这些 host 证据不代替新版本 HIL。 / See these records for production regressions and independent review of the final capacity design. The earlier two-LOG-slot candidate was superseded and does not describe the current implementation; host evidence does not substitute for new-version HIL.

原生 USB Serial/JTAG 为 Full-Speed **12 Mbit/s**，本板 sysfs 枚举已有[实录](docs/audit/artifacts/load_reliability_fix_20261113/usb-enumerated-speed-01.json)；它不是 UART 桥，波特率设置不等于线上吞吐上限。对当前 ESP32-S3 / IDF v5.5.5 路径，**没有已支持的 Serial/JTAG DMA/GDMA 开关**：CPU 通过中断从软件 ring 向最多 64 B 的端点 FIFO 搬运。独立 USB-OTG 控制器具有内部 DMA 硬件，但不是当前 Serial/JTAG 的配置选项；更换控制器还需验证实际驱动/栈是否启用 DMA、PHY 所有权、枚举身份和调试共存，尚未实施。依据：[DMA 能力核验](docs/audit/artifacts/load_reliability_fix_20261113/usb-serial-jtag-dma-capability-review-01.json)与 [ESP32-S3 TRM](datasheets/esp32-s3_technical_reference_manual_en.pdf) §3.4.2、§33.2/33.3.3、§32.3.5。

Native USB Serial/JTAG is Full-Speed **12 Mbit/s**, as [recorded by this board's sysfs enumeration](docs/audit/artifacts/load_reliability_fix_20261113/usb-enumerated-speed-01.json). It is not a UART bridge, and baud-rate settings do not define its wire-throughput limit. The current ESP32-S3 / IDF v5.5.5 path has **no supported Serial/JTAG DMA/GDMA switch**: interrupt-driven CPU copies move bytes from the software ring into endpoint FIFOs of at most 64 B. The separate USB-OTG controller has internal DMA hardware, not a configuration option for current Serial/JTAG. Migration would require proof that the actual driver/stack enables DMA and validation of PHY ownership, enumeration identity and debug coexistence; none is implemented. See the [DMA capability review](docs/audit/artifacts/load_reliability_fix_20261113/usb-serial-jtag-dma-capability-review-01.json) and [ESP32-S3 TRM](datasheets/esp32-s3_technical_reference_manual_en.pdf) §3.4.2, §33.2/33.3.3 and §32.3.5.

当前 Serial TX owner 将完整记录交给 **2304-byte 驱动 TX ring**；已核对的 IDF v5.5.5 BYTEBUF 路径整条接纳或返回 0，避免旧版按 64 bytes 应用分块后在 deadline 丢尾。2304 bytes 是软件 ring/记录容量，不是 USB 端点物理包长。普通记录从本地入队开始有 **250 ms** 绝对期限，优先控制/生命周期记录为 **1000 ms**；排队、驱动等待与调度让出共用此期限，不在重试时续期。每次驱动等待最多 1 tick，临近期限用零等待。过期记录继续有界清理，不依赖未来生产者再发通知才回收槽位。

The current Serial TX owner submits complete records to a **2304-byte driver TX ring**. The verified IDF v5.5.5 BYTEBUF path admits an entire record or returns zero, avoiding the old application-level 64-byte chunking that could lose a tail at deadline. The 2304 bytes describe software ring/record capacity, not a physical USB endpoint packet length. Normal records have a **250 ms** absolute lifetime from local enqueue, and priority control/lifecycle records have **1000 ms**. Queue residence, driver waits, and scheduling yields share that deadline; retries do not renew it. Each driver wait is at most 1 tick and becomes nonblocking near expiry. Expired records continue through bounded cleanup without depending on a future producer notification to reclaim slots.

整行/整帧本地入队及整条 driver 接纳仍**不保证线上原子交付或 PC 确认**。防御性短写路径保留游标，不把部分写当成功，也不在一条已开始的记录中插入另一条；达到绝对期限后计数并回收。已经被驱动接收的前缀无法撤回，必要时补 LF 只恢复下一条的边界。Host 仍须保存并拒绝截断帧，不能补 CRC/identity 或将缺 trailer 直接解释成配置域变化。下文 0d8f60f 旧截断实录完整保留，不能据新实现提前宣告 HIL 已无截断。

Whole-record local queuing and whole-record driver admission still **do not guarantee atomic wire delivery or PC acknowledgment**. Defensive short-write handling retains its cursor, never counts a partial write as success, and does not interleave another record inside one already started; absolute expiry is counted and the slot reclaimed. An accepted prefix cannot be withdrawn, and an LF when needed only restores the next record boundary. The host must still preserve and reject truncated frames without inventing CRC/identity fields or interpreting a missing trailer as direct proof of a configuration-domain change. The historical 0d8f60f truncation captures below remain intact; the new implementation alone does not establish truncation-free HIL results.

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


## 正式 ADC 末位读取修复 / Scientific ADC final-bit repair

修复提交为 `b13321a2c166715fa15f98c1677626786bec65b1`，运行BUILD `b13321a-dirty`，ELF SHA256 `e0e6d5878e9385fd883cffb782126a2a6e6b6cf00ccc471abd6a015a151f68ef`。64个host入口、实际ESP-IDF v5.5.5构建及183个生产输入hash闭合；app-only写入后独立BUILD/READY核对成功。首次reconfigure重排sdkconfig的验证记录保留，随后以解析后的固定配置重新验证，没有使用变化中的构建清单烧录。

The repair is commit `b13321a2c166715fa15f98c1677626786bec65b1`, running as BUILD `b13321a-dirty` with ELF SHA256 `e0e6d5878e9385fd883cffb782126a2a6e6b6cf00ccc471abd6a015a151f68ef`. All64 host entries and the actual ESP-IDFv5.5.5 build pass with matching183 production inputs. Independent BUILD/READY checks follow the app-only write. The initial reconfigure changed sdkconfig, so that evidence is retained and validation was repeated against the resolved configuration before flashing.

本板显式启用 `CONFIG_SENSORARRAY_ADS_GPIO_TAIL_READ_ENABLE`。ADC1标准包包括命令、STATUS、四个数据字节和CRC共56位；新路径先由SPI/DMA接收55位，在确认描述符回收后实际驱动最后一个时钟并读取MISO，再完成下降沿和恢复SPI路由。CRC末位来自GPIO实测，不来自计算出的CRC；完整CRC、STATUS、新鲜度、配置generation与有限重试仍按原规则判断。超时且DMA归属未知时保持隔离，不复用描述符。既有真实ADS1262身份、独占SPI3、STATUS+CRC8、INTERFACE=06的物理回读是准入条件；配置写入、复位和失败恢复撤销证明。缺条件即明确失败，不偷偷回退到旧路径。

This board explicitly enables `CONFIG_SENSORARRAY_ADS_GPIO_TAIL_READ_ENABLE`. The56-bit ADC1 transaction comprises command, STATUS, four data bytes and CRC. SPI/DMA receives55 bits; after exact descriptor reclamation, the driver physically clocks and samples the final MISO bit, completes the falling edge and restores SPI routing. The bit comes from GPIO, never from the computed CRC. Full CRC, STATUS, freshness, configuration generation and bounded-retry rules remain intact. Unknown DMA ownership stays quarantined without descriptor reuse. Admission requires an actually identified ADS1262, dedicated SPI3 and physical INTERFACE=06 verification for STATUS+CRC8 with serial timeout disabled. Writes, reset and failed restoration invalidate that proof; missing prerequisites fail explicitly without fallback.

启动选速、电池/rail辅助读取及正式ADC1读取共用该策略，保留细档扫描和2.5%门限。诊断STD组显式强制标准SPI56，从而保持真实对照；`SPISOURCE?`只查询不可变启动结果，不能冒充实时统计。新错误文本明确区分连续SPI字节和SPI55/GPIO1组装证据。模拟建立、参考安全、自动PGA、shadow/cache、逐行模式和科学有效性规则保持。

Startup qualification, battery/rail ADC1 helpers and scientific reads use the same policy while retaining the fine clock ladder and2.5% criterion. Diagnostic STD groups explicitly use SPI56 so the control remains genuine. `SPISOURCE?` reports immutable boot evidence, not live telemetry. Error text distinguishes continuous SPI bytes from SPI55/GPIO1 assembly. Analog settling, reference safety, auto-PGA, shadow/cache, per-row modes and scientific-validity rules remain.

| 实际观测 / Observation | 结果 / Result |
|---|---|
| 启动选速 / Boot qualification | 2MHz requested/observed; one candidate;2000 checked;0 CRC |
| 正式VOLT1窗口 / Scientific VOLT1 window | 120.000100s; ADC0/79417 CRC over118.658930s complete-health bracket |
| Capture / USB | 61.464674 /61.508761FPS; USB7381 trusted frames;0 internal missing sequence observations |
| BLE | 32.867940FPS;3944 trusted frames;3433 missing sequence observations |
| Frame integrity / identity | USB0/0 and BLE0/0 errors |
| Validity | USB480 X12 points at floatingS1D2 remain invalid;0 CRC-invalid or unacquired points observed |
| MFAULT / reset | Not observed in the closed capture window |
| 同boot2MHz反向对照 / Same-boot2MHz diagnostic control | SPI56 gain1:28/200; bypass:35/200 CRC; GPIOtail:0/200 in each group |
| Restore after diagnostic | known=1,verified=1,isolate=0; first=0x109 retained; terminal=0,cleanup=0 |
| 整体验收 / Overall acceptance | FAIL_BLE_THROUGHPUT; absolute precision NOT_ACCEPTED;600s/32 configurations NOT_RUN |

同频切回标准路径即恢复错误，而实际采样末位又消除错误，支持问题位于末位读取/时序路径。它尚未区分芯片采样延迟、DOUT末尾状态变化、GPIO矩阵或板级波形；没有示波器证据，`physicalVerified=0`。55+1同时改变最后位的暂停和采样时刻，不能声称已唯一证明某个硅缺陷。采集帧率比上一轮下降约6.7%，时钟、输入变化和GPIO开销混杂，不能把全部差异直接归因某一项；没有永久限速或放宽阈值。后续TAIL诊断的63次标准路径拒收会进入累计驱动计数，发生在已闭合正式窗口之后，不能误报为该窗口科学读取回归。

Errors return when the same-clock diagnostic switches back to standard reads and disappear with physical final-bit sampling. This localizes the defect to the final-bit read/timing path without uniquely identifying silicon sampling delay, end-of-packet DOUT behavior, GPIO routing or board waveforms. No oscilloscope evidence exists: `physicalVerified=0`. The55+1 operation changes both pause and sampling time. Capture FPS is approximately6.7% below the preceding run; clock, input trajectories and GPIO overhead are confounded, so the entire difference cannot be assigned to one factor. No permanent rate limit or relaxed threshold was introduced. The63 later diagnostic standard-path rejections enter cumulative driver counters after the closed scientific window and must not be reported as runtime failures within it.

证据 / Evidence: [build/test identity](docs/audit/artifacts/cdc_recovery_acceptance_20261116/finalbit-science-02-validated-identity.json), [normal boot](docs/audit/artifacts/cdc_recovery_acceptance_20261116/finalbit-science-02-boot-verification.json), [120-second verdict](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-05/screen-05-verdict.json), [same-clock control](docs/audit/artifacts/cdc_recovery_acceptance_20261116/finalbit-science-tail-comparison-01.json), [precise source/test diff](docs/audit/artifacts/cdc_recovery_acceptance_20261116/finalbit-science-increment-01.patch), [independent source audit](docs/audit/artifacts/cdc_recovery_acceptance_20261116/finalbit-independent-source-review-01.json). 所有实录保留，压缩归档逐文件校验；合成回归单独标识。 / Original captures remain intact, compressed archives are verified per file, and synthetic regressions are labelled separately.


## BLE SDK 异步返回观测 / BLE SDK callback observations

`0c94fe3bb58b25b0994ed2b718a8f5c33d456f4d` 只在既有BLE锁内为每个CONF事件增加状态与等待者匹配两类计数：`confOk/confError/confMatched/confUnmatched`。它不改优先级、队列、工作预算、SDK或科学采集路径。BLESVC保留v1旧字段，新增字段为boot累计、uint32回绕；同一个snapshot锁内复制，回复极值371B，含NUL372B。matched只表示原等待者谓词命中，connection epoch来源未知；unmatched不能当FAST身份、丢包或PC ACK。

Commit `0c94fe3bb58b25b0994ed2b718a8f5c33d456f4d` adds status and waiter-match counters for each local SDK CONF event under the existing BLE lock: `confOk/confError/confMatched/confUnmatched`. It changes no priority, queue, work budget, SDK or scientific path. BLESVC retains its v1 fields. New counters are boot totals with uint32 wrap, copied in the same snapshot lock; the extreme reply uses371 bytes,372 including NUL. Matched means only that the existing waiter predicate matches. Originating connection epoch remains unknown, and unmatched establishes neither FAST identity, loss nor PC acknowledgement.

实际64个host入口、目标构建、app-only写入及完整BUILD/READY均通过。ELF SHA256 `3077bfbfb35d7f90e27e15673ab90f90c35243db97ec0259776ecd84db019f97`，本boot仍选2MHz并通过2000次检查、CRC0。独立120秒screen-06结果如下；原screen-05和失败筛查均保留。

All64 host entries, the target build, app-only write and full BUILD/READY checks pass. ELF SHA256 is `3077bfbfb35d7f90e27e15673ab90f90c35243db97ec0259776ecd84db019f97`. Boot again selects2MHz with2000 checked reads and zero CRC rejection. The separate120-second screen-06 results follow; screen-05 and all earlier failures remain intact.

| 指标 / Metric | screen-06 |
|---|---:|
| Capture / USB / BLE FPS | 60.016896 /60.029869 /32.613674 |
| USB可信帧 / trusted frames; missing sequence observations | 7203;1 |
| BLE可信帧 / trusted frames; missing sequence observations | 3913;3290 |
| ADC CRC | 0/76069 within complete-health bracket |
| USB无效点 / invalid points | 376 X12 at S1D2; no observed CRC-invalid/unacquired point |
| ASCII CRC / configuration identity errors | USB0/0; BLE0/0 |
| CONF query-bracket delta | ok13076;error0;matched0;unmatched13076 |
| SDK wrapper query-bracket delta | started13076;completed13075;after active1 |
| 总体验收 / Overall acceptance | FAIL_BLE_THROUGHPUT; no absolute precision acceptance |

CONF回调可以在发送API返回前发生，因此末尾active=1时CONF比wrapper完成计数多1并非必然错误。回调查询窗与主机完整帧窗口不完全重合，不能把计数相减成空中丢包。该窗口未观察到SDK异步错误，不能证明PC收到；BLE输出仍仅采集的一部分。下一步采用独立可关闭的任务runtime诊断配置，区分owner自身调度时间与SDK调用墙钟；当前BLESVC的cpu/blocked仍为unknown，不从差值编造阻塞时间。

A CONF callback may arrive before the sending API returns. With after.active=1, one more CONF than completed wrapper call is not necessarily an error. Query and complete-frame receipt windows differ and cannot be subtracted to infer over-air loss. No SDK asynchronous error was observed in this bracket, but that does not prove PC receipt; BLE output still trails capture. The next independent, disableable runtime diagnostic will distinguish owner scheduled time from SDK-call wall time. Existing BLESVC cpu/blocked fields remain unknown; their difference is not invented blocked time.

证据 / Evidence: [build/test identity](docs/audit/artifacts/cdc_recovery_acceptance_20261116/ble-conf-01-validated-identity.json), [boot verification](docs/audit/artifacts/cdc_recovery_acceptance_20261116/ble-conf-01-boot-verification.json), [screen-06 verdict](docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-06/screen-06-verdict.json). 本轮未做600秒或32配置的通过声明，也未修改原CRC2.5%门限。 / No600-second or32-configuration pass is claimed, and the original2.5% CRC criterion remains unchanged.

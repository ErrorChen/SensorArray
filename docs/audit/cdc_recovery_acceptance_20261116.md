# 2026-11-16 软件恢复与双 CDC 验收 / Software recovery and dual-CDC acceptance

> **2026-11-16 最新实机阶段 / Latest hardware stage:** `caa9082-dirty`（160 MHz、`-O2`、CPU owner 诊断 ON）的 screen11 已闭合：120.000070722 秒，ADC CRC **0/82,911**，Capture/USB/BLE **64.38/64.38/47.08 FPS**；USB 7,724 帧内部连续，BLE 5,650 帧缺 2,077 个序号观察，**整体仍 FAIL_BLE_THROUGHPUT**。68 个 host 入口最终全部通过、实际 build 与 test 的 185 个生产输入匹配；首轮提取器失败保留。CAP8 已查询恢复，670 个 S1D2 X12 保留无效，绝对精度未验收。当前镜像 600 秒/32 配置仍 NOT_RUN。 / Screen11 on `caa9082-dirty` (160 MHz, `-O2`, CPU-owner diagnostics ON) is closed: 120.000070722 seconds, **0/82,911 ADC CRC rejections**, and **64.38/64.38/47.08 FPS capture/USB/BLE**. USB receives 7,724 internally consecutive frames; BLE receives 5,650 with 2,077 missing observations. **Overall status remains FAIL_BLE_THROUGHPUT**. All 68 host entries pass in the final run with 185 production inputs matching the actual build; the first extractor failure is retained. CAP8 recovery is query-verified, 670 S1D2 X12 points remain invalid, and absolute accuracy is unaccepted. This image’s 600-second/32-configuration runs remain NOT_RUN.

> 历史连续稳定性 / Historical continuous stability: `a656d53` 的独立 600.000075751 秒 USB+BLE 结果完整保留；其 ADC 0/428,131 和 CAP8 恢复仅属于旧镜像，不能填入 `caa9082-dirty` 的 600 秒验收。新镜像才包含 CRC32 ROM/planner 等价优化和显式 SCLK 低电平 hold；两者都不放宽科学 CRC/invalid 门限。 / The independent 600.000075751-second USB+BLE result on `a656d53` remains preserved. Its 0/428,131 ADC result and CAP8 recovery belong to the previous image and do not fill the new image’s 600-second acceptance. `caa9082-dirty` adds the equivalent ROM-CRC32/planner optimization and explicit SCLK-low hold; neither relaxes scientific CRC/invalid criteria.

本记录接续 [11 月 14 日阶段](usb_otg_ble_priority_20261114.md)。用户已重启板卡，并授权继续烧录、调试；没有要求安装软件或使用管理员权限。历史失败记录保留，下面的下载成功不代表完整性能/精度验收通过。

This continues the [November 14 stage](usb_otg_ble_priority_20261114.md). The user rebooted the board and authorized continued flashing/debugging. No software installation or administrator account is required. Historical failures remain intact; successful recovery is not full performance or accuracy acceptance.

证据目录 / Evidence: [cdc_recovery_acceptance_20261116](artifacts/cdc_recovery_acceptance_20261116/). All command records retain argv, checkout HEAD, tracked-diff hash, exit status and logs. Hardware tools run as ordinary UID 1000; escaping the agent sandbox is not `sudo`.

## 已执行的恢复步骤 / Executed recovery steps

| 操作 / Operation | 实际结果 / Observed result |
|---|---|
| 重启后枚举 / Enumeration after user reboot | `303a:1001`, serial `80:B5:4E:CD:D9:AC`, tty accessible through existing `dialout` membership |
| `flash-01` app-only write | Source `4a183bca6a5a0566a834a727a58d5e25a80ad31b`; 1,590,656 bytes at `0x10000`; SHA256 `283e70782280789ed9c79f29e2263179deb23e9be57f6e35e8f5fb8595fda223`; esptool verified flash hash; no bootloader/partition/NVS/calibration write |
| esptool `after=hard_reset` | 未启动应用 / App not observed; subsequent no-reset sync reported the same MAC and an already-running stub |
| 明确 DTR/RTS 退出序列 / Explicit DTR/RTS exit | `usj-exit-01` completed without ioctl error, but no application enumeration observed; not a successful recovery |
| 软件 WDT 复位 / Software watchdog reset | `usj-watchdog-exit-01`: same MAC verified, FORCE_DOWNLOAD bit cleared with mask 1 and read back as 0, one installed esptool S3 `watchdog_reset()`; new `303a:4042`, devnum 15, formal IF0 and debug IF2 appeared |
| 应用身份 / Application identity | CDC and BLE returned `version=4a183bc`; ELF hash field contained only `9a16d7673`, a 9-character prefix; full running-hash attestation failed and was not waived |
| BLE 状态查询 / BLE state queries | `boot-05-readonly`: PROTO2, dual-CDC capability, all three frontends present, SPI qualification completed, selected requested/observed clock 750000/754000 Hz, BLE 2M PHY and MTU247 |
| BLE → USBBOOT | Exactly one action in `ble-rom-01`; kernel monitor recorded old `303a:4042` device 15 removed at monotonic 309074.718360 and new `303a:1001` device 16 added at 309075.628112 |
| ROM 同步 / Downloader sync | `ble-rom-01-identify`: no-reset esptool synchronized, confirmed ESP32-S3 rev0.2, MAC `80:b5:4e:cd:d9:ac`, 16 MB flash and embedded 8 MB PSRAM; no flash write |

`ble-rom-01-udev` 的退出码 124 是预设的 40 秒只读监视窗口结束，不是烧录超时。原始 remove/add 事件是重新枚举证据；RACK/RAPP 本身仅是终态发布证据。串口 RTS 失败的电气/主机根因尚未确定：默认 DTR 状态只能作为候选解释，明确清 DTR 的实测也未恢复。

The udev monitor's exit 124 means its planned 40-second observation window ended, not a flash timeout. Actual remove/add events establish re-enumeration; RACK/RAPP alone do not. The electrical/host cause of the RTS failure is unresolved: default DTR state is only a hypothesis, since the explicit clear-DTR attempt also failed.

esptool 的普通 ROM 连接可能按 SDK 流程暂时关闭下载器 watchdog；这是下载环境行为，不是删除应用安全 watchdog。退出助手不写 Flash/eFuse，真正的应用是否启动必须由后续枚举和 BUILD 核验判断。不得只凭成功返回就自动反复重启。

Normal esptool ROM connection may disable downloader watchdogs as part of its SDK procedure. This does not remove application watchdog protection. The exit helper writes no flash/eFuse; subsequent enumeration and BUILD queries establish whether the application booted. A returned reset call does not authorize an unbounded retry loop.

## 本轮复现与修改 / Reproductions and changes

### 恢复标记 / Recovery latch

旧 `sensorarrayUsbOtgEnabled()` 选择一次 USJ 后即清 RTC marker；若恢复启动再次 early panic，下个启动会重选 OTG。已用基线真实 C 函数复现失败。修复后普通 RESTART 和再次 panic 保留恢复锁存，只有显式 USBBOOT 完成原有有界收尾、设置并读回 ROM 启动标志后才解除。本 boot 的后端选择不热切换。

Previously, `sensorarrayUsbOtgEnabled()` consumed the RTC marker when selecting USJ. Another early panic could return the following boot to OTG. The baseline production C fails the added regression. The repair retains the latch through ordinary RESTART and repeated panic; only explicit USBBOOT releases it after bounded cleanup and verified ROM-flag arming. The current boot's backend remains fixed.

66 个双后端 owner 场景、SerialIo 和控制回复回归已执行通过。真实早崩溃故障注入尚未执行；RTC 保留不保证跨断电，也不保证任意硬件故障可软件恢复。

All 66 owner scenarios across both backends, SerialIo and control-reply regressions passed. Physical early-crash injection remains untested. RTC retention across power removal and recovery from arbitrary hardware faults are not guaranteed.

### 完整 BUILD 哈希 / Full BUILD hash

`boot-04` 原始帧确认只有 9 位 hash。实际 IDF 5.5.5 的 `esp_app_get_elf_sha256()` 从 `CONFIG_APP_RETRIEVE_LEN_ELF_SHA=9` 的缓存复制，传入 65 字节缓冲仍会截断。新增 `sensorarrayFormatBuildReply()` 直接读取应用描述符中的 32 字节 ELF SHA，早期和正常查询统一输出 64 位十六进制；不修改全局日志长度配置。真实 C 格式化、缓冲边界及两处 dispatch 已回归通过。

The raw `boot-04` reply contains only nine hash characters. In the installed IDF 5.5.5, `esp_app_get_elf_sha256()` copies a cache limited by `CONFIG_APP_RETRIEVE_LEN_ELF_SHA=9`, even with a 65-byte destination. The new shared formatter reads all 32 descriptor bytes for both early and normal BUILD queries. Actual C formatter, buffer-boundary and dispatch tests passed; the global logging setting is unchanged.

### 首次查询的旧错误 / Stale error at the first query

`boot-03` 将 UNKNOWN 归给 BUILD，但独立 debug 原始记录中的入站内容是 `READY,bootId=1,b`，并不是 `BUILD?`。随后只读查询实际收到了 BUILD。PTY 对照复现了默认 ECHO、切 raw、前导 LF 终结旧半行的机制；它不能替代现场 USB OUT 抓包证明。

`boot-03` attributed UNKNOWN to BUILD, but the independent debug log identifies the received command as `READY,bootId=1,b`, not `BUILD?`. A subsequent read-only query received BUILD. A PTY control reproduces the default-ECHO/raw-transition/leading-delimiter mechanism; it is not a capture of the original USB OUT traffic.

## 新镜像恢复闭环 / Repaired-image recovery loop

源码 `3a55543cb21fa3412ede26ee79c1cce7d8371804` 的目标构建和 58 个 host 测试入口均实际通过，179 个生产输入 hash 一致。app 1,591,088 B，binary SHA256 `59d3f07d772730692ea4a68e9af6437779ddfd24849c78f8fb9a03fbb2e31fee`，完整 ELF SHA256 `d4ac82a199bd2bc8bb730b331225ffdbb94acd2c84c410cd675636da706e92e5` 已由设备 BUILD 核对。正式 CDC USBBOOT 一次动作后 1.002248 秒观察新下载接口，随后核 MAC、app-only 重刷和再次软件启动通过；[闭环证据](artifacts/cdc_recovery_acceptance_20261116/cdc-rom-01-verified-loop.json)。未再按 BOOT、未用 sudo、未修改 NVS/校准。

The target build and all 58 host test entries passed with matching hashes for 179 production inputs. The running BUILD matched the complete ELF hash above. One formal-CDC USBBOOT action produced a new downloader interface after 1.002248 seconds, followed by same-MAC sync, app-only reflash and verified software startup. No further BOOT-button operation, sudo, NVS or calibration write was used.

普通 RESTART 的第一次观察脚本在新 tty 权限尚未可用时收到 EACCES；原 ERROR 保留，后续独立只读查询确认同一 BUILD 且 session 从 `50D0CC36C32764D1` 变为 `2D7D5CE074215BB2`，没有重发动作。工具现对新 epoch 的 ENOENT/EACCES 在原60秒期限内重新核身份并等待，24个合成场景通过；修后该工具的普通重启实机重测仍待执行。

The first ordinary-RESTART observer encountered EACCES opening the new tty. Its ERROR remains unchanged; a separate read-only query confirmed the same BUILD and a changed session, without another action. The observer now retries only new-epoch ENOENT/EACCES within the original 60-second deadline, revalidating identity each time. All 24 synthetic cases pass; a new hardware run of that revised observer remains pending.

## 120 秒负载筛查失败 / 120-second load screen failed

`screen-01` 完整120秒，Capture64.563554FPS、USB62.532747FPS/239序号缺口、BLE43.370896FPS/2537序号缺口。ADC CRC 4378/102828=4.257595%，超过用户2.5%要求；本boot选择550kHz、驱动观测551kHz。ASCII CRC及身份错误均0，记录窗口未观察MFAULT/reset。完整指标、分母和边界见[原始分析](artifacts/cdc_recovery_acceptance_20261116/screen-01/screen-01-analysis-01.json)、[失败判定](artifacts/cdc_recovery_acceptance_20261116/screen-01/screen-01-verdict.json)及README双语表格。

The full 120-second screen failed both the ADC CRC requirement and transport throughput expectations: capture64.563554FPS, USB62.532747FPS/239missing observations, BLE43.370896FPS/2537missing observations, and 4.257595% ADC rejection versus the 2.5% limit. This boot selected550kHz, observed551kHz. No ASCII CRC or identity errors, MFAULT or reset were observed in the recorded window. This is not a precision pass.

## 第二轮筛查与当前定位 / Second screen and current investigation

源码 `1e9fd19` 仅将正式 OTG owner/transport 优先级从 6 改为 7，保留旧 USJ 的 6，以及所有队列容量、work budget、时限、ADC 语义。58/58 host 入口、目标构建和烧录完整 BUILD 对照通过；[镜像关联](artifacts/cdc_recovery_acceptance_20261116/priority-candidate-01-validated-identity.json)。完整 `screen-02` 为 **FAIL_BLE_THROUGHPUT**：Capture 68.047855 FPS、USB 68.028989 FPS/1 缺口、BLE 35.347479 FPS/3923 缺口；ADC CRC 2152/107124=2.008887%，本 boot 请求/观测 SPI 为750000/754000 Hz。无 ASCII CRC/identity 错误，记录窗口未观察 MFAULT/reset。

Source `1e9fd19` changes only formal OTG owner/transport priority from 6 to 7, retaining legacy USJ at 6 and all queue capacities, work budgets, deadlines and ADC semantics. All 58 host entries, the target build and full running BUILD verification passed. The full second screen still fails BLE throughput: capture68.047855FPS, USB68.028989FPS/1gap, BLE35.347479FPS/3923gaps; ADC CRC2152/107124=2.008887% at requested/observed750000/754000Hz. No ASCII CRC, identity errors, MFAULT or reset were observed in the recorded window.

BLE 排队压力从 transport 移至 BLE pool（dropD 增量3690）；SDK 单次最大20.757ms是墙钟，不证明 CPU 忙等或阻塞因果。现有 WorkBudget 已在真实 idle generation 更新后重启本任务 quantum，不能重复“修复”已存在逻辑。新增低开销 yield 原因与 SDK 前后 idle 观测用于下一轮定位，尚不改变调度规则。完整原文、SHA、不同计数窗口边界见[服务对照](artifacts/cdc_recovery_acceptance_20261116/screen-02/ble-service-review-01.json)。

BLE pressure moved from transport admission to the BLE pool (dropD increment3690). The20.757ms maximum SDK call is wall time, not proof of CPU busy-wait or causal blocking. WorkBudget already renews each task's quantum after genuine idle-generation progress; that existing logic must not be “fixed” again. New low-overhead yield-reason and SDK idle observations are intended to locate the remaining deficit without changing scheduling rules.

CRC 分析优先于放宽门限。用户提出的5%暂作可评估调试容忍度，当前启动策略仍为2.5%，旧FAIL不倒改。完整坏包独立CRC复算显示249条均差最低位（248次0→1、1次1→0），是调查末位采样/转换状态的线索而非因果证明。新的有界连续/单次转换对照在独立模块中实施，保持当前SPI频率与科学采样语义；实机对照尚未运行，不能预先填写结论。

CRC root-cause work takes precedence over changing the limit. The proposed5% is a prospective debugging tolerance; startup still uses2.5% and historical failures remain unchanged. Independent recomputation finds all249 complete diagnostic bad packets differ at the least significant CRC bit (248 zero-to-one, one one-to-zero). This is a clue, not a proven cause. The bounded continuous/pulse comparison is being implemented in a separate module with unchanged SPI clock and scientific acquisition semantics; hardware results remain pending.

第二轮在 754 kHz 驱动观测时钟下的[独立复算](artifacts/cdc_recovery_acceptance_20261116/screen-02/crc-tail-review-01.json)又取得 156 条完整坏包：全部 `computed XOR received = 01`，其中145条0→1、11条1→0，复算不一致与截断均为0。两轮合计405条均出现这一 syndrome；这是限量错误样本的共同特征，不能据此证明数据域一定无误或推定全部失败的物理原因。

A [second independent calculation](artifacts/cdc_recovery_acceptance_20261116/screen-02/crc-tail-review-01.json), at the driver-observed754kHz clock, finds156 further complete bad packets: all have `computed XOR received = 01`, with145 zero-to-one and11 one-to-zero differences, no recomputation disagreement and no truncated records. All405 records across the two screens share this syndrome. This bounded error sample does not establish an uncorrupted data field or the physical cause of every failure.

## 诊断首版启动失败与内存修正 / Initial diagnostic boot failure and memory correction

`bb230d0` 实际app-only烧录、软件启动、完整BUILD ELF核对成功，但正常采集未启动：BOOT/READY明确 `stage=scan_task_create_failed,err=0x101,ready=0,seq=0`。SPI扫描尚未开始，ADSCHK=TIMING未发送。原boot observer的CONFIG超时ERROR和独立只读定位记录均保留，[失败证据](artifacts/cdc_recovery_acceptance_20261116/timing-startup-failure-01.json)。

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

全部9个拒收仍为CRC最低位0→1 syndrome；坏包独立复算一致，没有修位、填零或进入科学数据。该结果支持转换状态/读数时序参与错误，但固定组序、每次START及held读取节奏也不同，不能只归因RUNMODE。新GPIO末位候选仍在开发，尚未烧录/运行：只在确认专用SPI owner和DMA回收后，用真实GPIO采最后一位，明确分开SPI55bit与GPIO1bit证据；不根据computed CRC猜位，不直接改科学路径。见[完整对照](artifacts/cdc_recovery_acceptance_20261116/timing-comparison-01.json)。

All nine rejections have the same least-significant-CRC-bit zero-to-one syndrome and independently verified CRC calculation. No bit is repaired or admitted into science. The result supports a role for conversion/read timing, but fixed order and START/held-read cadence prevent attribution to RUNMODE alone. A separate GPIO-tail diagnostic candidate is under development and has not been flashed or run. It requires dedicated SPI ownership and reclaimed DMA, physically samples the last bit, and records SPI55-bit and GPIO1-bit evidence separately. It must never infer the bit from computed CRC or directly change scientific acquisition.

[第三轮120秒筛查](artifacts/cdc_recovery_acceptance_20261116/screen-03/screen-03-verdict.json)仍为**FAIL_BLE_THROUGHPUT**。Capture66.360471FPS，USB66.331356FPS/7963帧/2个序号缺口，BLE35.680964FPS/4283帧/3682个缺口；ADC CRC2433/104722=2.323294%，诊断跨度119.564089s。两路ASCII CRC/identity错误0，窗口未观察MFAULT/reset。TIMING命令不在该采集窗口内，不把其held计数混入本错误率。临时RAM轨值为用户ADS供电脚读数的+3.350/−1.795V代表值，未CAL=SAVE；不是精度验收。

The third full120-second screen still **fails BLE throughput**: capture66.360471FPS; USB66.331356FPS/7963frames/2missing observations; BLE35.680964FPS/4283frames/3682missing observations; ADC CRC2433/104722=2.323294% over119.564089seconds of diagnostic receipts. Neither receiver reports ASCII CRC or identity errors, and no MFAULT/reset was observed in the window. No TIMING command ran within it, so held rereads do not enter this ratio. The user's ADS-pin DMM range supplies only nominal+3.350/−1.795V volatile RAM rails, without CAL=SAVE or precision acceptance.

三轮实录中共取得549条完整CRC坏包，523条最低位0→1、26条1→0，全部syndrome为01；独立多项式复算一致。此为限量坏包样本，并非全部尝试或物理根因的充分证明。[联合证据](artifacts/cdc_recovery_acceptance_20261116/crc-three-screens-01.json)。

Across three real captures, all549 complete rejected packets share syndrome01:523 have a zero-to-one least-bit difference and26 the reverse. Independent polynomial division agrees. These rate-limited bad-packet samples do not by themselves prove the corruption location or physical cause. [Combined evidence](artifacts/cdc_recovery_acceptance_20261116/crc-three-screens-01.json).

## 4bf31db 末位对照结果 / Final-bit comparison results

当前已烧录并核对完整BUILD：`4bf31dbc576cf5061e1694102f82ad2111ef5392`，BUILD `4bf31db-dirty`，app1612752B，bin SHA256 `366994b942f643eb1c688291efbe22694cfa7f2d1336a1d2c07f9cfd6c10b08a`，ELF SHA256 `a2557c8f07f1e1735313315dd65bd71f4e2b79b58bbf284338888fa9e2220d5c`。61个host入口全部exit0，实际ESP-IDF构建通过，183个生产输入hash一致。[验证清单](artifacts/cdc_recovery_acceptance_20261116/tail-io-01-validated-identity.json)。软件从前一版DEGRADED进入ROM、app-only烧录和退出下载全部成功；新boot session `18407FDC95220620` 正常CAP8，未CAL=SAVE，未物理按键。

The currently flashed image is `4bf31db-dirty`, with the full commit, binary and ELF identities above. All61 host entries and the actual ESP-IDF build pass;183 production input hashes match. Software ROM entry from the previous DEGRADED state, app-only programming and software boot all succeed. The new session `18407FDC95220620` runs CAP8 without CAL=SAVE or a physical button operation.

11次独立TAIL请求（编号为日志tail-probe-02至12）均完整恢复，配置身份不变，使用同boot750000Hz请求/754000Hz驱动观测时钟：

| CONT诊断组 / Continuous diagnostic group | 校验读数 / Checked reads | CRC拒收 / CRC rejected |
|---|---:|---:|
| 标准SPI、PGA1 / Standard SPI, gain1 | 2200 | 7 |
| SPI55+GPIO1、PGA1 / Physical final-bit sample, gain1 | 2200 | 0 |
| 标准SPI、bypass / Standard SPI, bypass | 2200 | 8 |
| SPI55+GPIO1、bypass / Physical final-bit sample, bypass | 2200 | 0 |

保存的44条GPIO分段原始观察包含0和1两种末位，主机独立多项式复算全部一致；这44条不是全部4400次逐条原始波形。新IO查询实测MISO `OE=1/peripheralOE=1/signal67`、SCLK signal66，门禁修复后通过。55位SPI的未接收LSB始终明确unknown，最终位来自真实GPIO读取，不根据计算CRC填补。不把成功的GPIO/寄存器回读称为模拟隔离或无毛刺验证。[原始结果及来源边界](artifacts/cdc_recovery_acceptance_20261116/tail-comparison-01.json)。

All11 independent TAIL requests complete restoration with unchanged configuration identity on the same boot and clock. Standard SPI rejects15/4400 reads; physical final-bit sampling rejects0/4400. The44 saved split observations contain both0 and1 tail bits and independently recompute correctly; they are not individual waveform records of all4400 reads. Actual IO queries show MISO OE1/peripheralOE1/signal67 and SCLK signal66. The unreceived SPI prefix bit remains explicitly unknown; only the real GPIO sample supplies it. Digital readback does not prove analogue isolation or glitch-free signalling.

这个固定组序的短接输入实验改变了最后一个时钟的暂停和采样时刻，支持末位时序假设，但没有示波器证据可唯一归因于某个硅片缺陷。**正式矩阵/电池/rail读取尚未采用新方案，不能宣称运行期CRC已修复。** 启动门槛仍2.5%。SPI根因修复候选正在评审，600秒和32配置验收仍NOT_RUN。

The fixed-order shorted-input experiment changes both the final-clock pause and sample timing. It supports a last-bit timing mechanism without uniquely proving a silicon/pad defect. **Normal matrix, battery and rail reads still use their existing path; runtime CRC is not yet repaired.** Startup qualification remains2.5%. A production repair is under review; the600-second and32-configuration acceptance runs remain NOT_RUN.

BLE TX8候选在原调度压力模型中未通过数据生产进度断言，未烧录，已撤回到TX7/CTRL8；失败原记录和候选patch保留。该模型Core0生产者对应AsyncLog，并非Core1采集；BLE永远ready假设不含真实四槽依赖，不能把模型计数当实机FPS。候选未通过的事实不被改写，也不放宽控制时限来制造PASS。[撤回记录](artifacts/cdc_recovery_acceptance_20261116/ble-tx8-retraction-coordination-01.json)。

The TX8 BLE candidate fails a producer-progress assertion in the existing scheduling stress model and is withdrawn before flashing; TX7/CTRL8 remains. Failure logs and the candidate patch are retained. Its Core0 producer represents AsyncLog, not Core1 capture, and its perpetually-ready BLE assumption omits the real four-slot dependency; model counts are not hardware FPS. The failure is not relabelled and control deadlines are not relaxed.

## bc99767 实机失败保留 / Retained hardware failures

第四轮完整120.000145秒VOLT1筛查为 **FAIL_BLE_THROUGHPUT_AND_ADC_CRC**：capture65.900556FPS、USB65.811374FPS/7897帧/15个缺口、BLE31.815184FPS/3816帧/4092个缺口。实际BLE interval6×1.25ms=7.5ms、MTU247、2M PHY协商成功，但吞吐仍不足。ADC诊断窗119.390852s内5168/106486=4.853220%拒收，门槛仍2.5%；两路ASCII CRC/identity错误0，无MFAULT/reset观察。启动SPI为1.111MHz，不能把跨boot CRC或吞吐变化单独归因于连接间隔。新日志366条完整坏包均syndrome01，独立复算一致。[完整结果](artifacts/cdc_recovery_acceptance_20261116/screen-04/screen-04-verdict.json)、[坏包证据](artifacts/cdc_recovery_acceptance_20261116/screen-04/crc-tail-review-01.json)。

The fourth full120.000145-second VOLT1 screen **fails BLE throughput and ADC CRC**: capture65.900556FPS; USB65.811374FPS/7897frames/15missing observations; BLE31.815184FPS/3816frames/4092missing observations. The actual7.5ms interval, MTU247 and2M PHY are negotiated, but throughput remains insufficient. ADC rejects5168/106486=4.853220% over119.390852seconds of diagnostic receipts against the unchanged2.5% criterion. Both receivers have zero ASCII CRC/identity errors, with no MFAULT/reset observed. This boot's1.111MHz SPI clock and changed diagnostic load confound direct causal attribution to the BLE interval. All366 complete new bad-packet records share syndrome01 and agree with independent CRC calculation.

随后首次TAIL请求id7 **失败**，总事务123802µs：标准组200/200有效；GPIO组第1次读在提交55位SPI前被拒绝，后两组NOT_RUN。first/terminal=0x103、cleanup=0、restoreKnown/Verified=1；主状态转DEGRADED/软件SAFE、revision6→7，physicalSafe仍unknown。旧runner将这次合法失败rev变化当身份冲突提前停止收证；已用同ELF/session/id的纯只读查询取回完整缓存，未重发动作。源码核对确认门禁不兼容IDF的MISO INPUT_OUTPUT/peripheral OE设置，但旧记录没有保存具体IO gate，故不假称已观测实际GPIO寄存器。修正和IO上下文扩展进行中。[原始失败与边界](artifacts/cdc_recovery_acceptance_20261116/tail-prequeue-failure-01.json)。

The subsequent first TAIL request, id7, **fails** after123802µs. The standard group completes200 valid reads; the first GPIO-group read is rejected before SPI55 submission, and the other groups are NOT_RUN. First/terminal error is0x103, cleanup0 and register restoration verified. The main state enters DEGRADED/software SAFE at revision6→7; physical safety remains unknown. The old runner mistakenly rejected this valid failure revision transition before collecting detail. A same-ELF/session/id read-only query retrieved the complete cache without repeating the action. Source review identifies a gate incompatible with IDF's MISO INPUT_OUTPUT/peripheral-OE setup, but the old record contains no individual IO gate/config snapshot. The admission repair and explicit IO evidence are in progress.

## 末位诊断候选 bc99767 / Final-bit diagnostic candidate bc99767

本轮新增 `ADSCHK=TAIL,200`：固定四组CONT×标准SPI/末位GPIO×PGA1/bypass对照；仍用原启动选定时钟。独立owner门禁、持久DMA描述符精确回收后才接管SCLK、真实GPIO位、完整恢复和隔离失败路径均有host回归。结果固定槽仍只在启动时分配PSRAM；实机查询为14328B。新BLE补丁 `df68a46` 明确请求7.5ms，但实际间隔须以GAP回调/查询及实收为准。两者均不放宽CRC门槛，也不把坏CRC填成有效点。

The new `ADSCHK=TAIL,200` runs four continuous-conversion groups comparing standard SPI versus a physically sampled final GPIO bit at gain1/bypass, using the existing selected clock. Host regression covers exclusive ownership, exact persistent-DMA reclamation before SCLK handoff, real GPIO observations, full restoration and quarantine failures. Fixed result storage is allocated only once in PSRAM; the board reports14328B. BLE patch `df68a46` explicitly requests7.5ms, whose actual negotiation and throughput still require hardware confirmation. Neither change loosens CRC qualification or admits rejected data.

生产提交 `bc99767`，BUILD `bc99767-dirty`；app1610960B、bin SHA256 `47a9ff30ffd60232d12a39ebf19def30e72a19d81a56c3504a0ba43089aa945e`、ELF SHA256 `2cd47eb02b556120fc9459a0cc2ec9c394ca95ac5313a7c4284b4267800cf93b`。61个host入口初跑60通过，presence旧提取器编译失败；修正后单项21场景通过，失败原录保留。实际IDF构建成功，生产输入与测试逐hash一致。app-only烧录及独立BUILD核对成功，READY1、CAP8，boot session `4C618C351450A839`；启动选中1100000Hz请求/1111000Hz驱动观测值，不把它与此前754000Hz跨boot直接当同条件结果。[构建和回归](artifacts/cdc_recovery_acceptance_20261116/tail-01-validated-identity.json)、[烧录](artifacts/cdc_recovery_acceptance_20261116/tail-01-flash-usj-result.json)、[独立启动核对](artifacts/cdc_recovery_acceptance_20261116/tail-01-boot.summary.json)、[READY](artifacts/cdc_recovery_acceptance_20261116/tail-ready-01.summary.json)。

Production commit `bc99767` reports BUILD `bc99767-dirty`; sizes and full binary/ELF hashes are above. Of61 host entries,60 initially passed; an old presence-source extraction boundary failed to compile. Its repaired21-case rerun passes, with the original failure retained. The actual IDF build passes and its production input hashes match the test run. App-only flashing and an independent full BUILD query succeed; READY1/CAP8 uses session `4C618C351450A839`. Startup selects requested1100000/driver-observed1111000Hz, so cross-boot comparison against754000Hz is not a controlled same-clock experiment. TAIL HIL and new BLE throughput results are pending at this entry.

## `bc99767` 时点待完成（历史） / Pending at `bc99767` (historical)

以下保留当时未完成状态；后续 600 秒单次连续观察见文末 `stability-01`，32 配置仍未运行。 / The status below is retained at its original point in time. The later single continuous 600-second observation is documented under `stability-01` at the end; the 32-configuration suite is still unrun.

先定位修复本次 CRC/输出背压，再重做筛查；600秒稳定性和32配置×120秒USB+BLE仍须实际执行。未运行项目不得由旧版本结果补齐。夹具、临时DMM rail校准和非精度验收边界沿用README；不执行CAL=SAVE。

Diagnose and repair the observed CRC/backpressure failures before repeating the screen. The600-second stability and32 configurations×120-second USB+BLE runs remain pending; older-image results cannot fill them. Fixture and temporary DMM rail limits apply, without CAL=SAVE.


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

证据 / Evidence: [build/test identity](artifacts/cdc_recovery_acceptance_20261116/finalbit-science-02-validated-identity.json), [normal boot](artifacts/cdc_recovery_acceptance_20261116/finalbit-science-02-boot-verification.json), [120-second verdict](artifacts/cdc_recovery_acceptance_20261116/screen-05/screen-05-verdict.json), [same-clock control](artifacts/cdc_recovery_acceptance_20261116/finalbit-science-tail-comparison-01.json), [precise source/test diff](artifacts/cdc_recovery_acceptance_20261116/finalbit-science-increment-01.patch), [independent source audit](artifacts/cdc_recovery_acceptance_20261116/finalbit-independent-source-review-01.json). 所有实录保留，压缩归档逐文件校验；合成回归单独标识。 / Original captures remain intact, compressed archives are verified per file, and synthetic regressions are labelled separately.


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

证据 / Evidence: [build/test identity](artifacts/cdc_recovery_acceptance_20261116/ble-conf-01-validated-identity.json), [boot verification](artifacts/cdc_recovery_acceptance_20261116/ble-conf-01-boot-verification.json), [screen-06 verdict](artifacts/cdc_recovery_acceptance_20261116/screen-06/screen-06-verdict.json). 本轮未做600秒或32配置的通过声明，也未修改原CRC2.5%门限。 / No600-second or32-configuration pass is claimed, and the original2.5% CRC criterion remains unchanged.

## CPU owner 诊断镜像与 screen07 / CPU-owner diagnostic image and screen07

本节闭合上一节提出的 owner 自采诊断，**不是 BLE 吞吐修复完成声明**。源码提交 `b58336eeaa136d0aca23e4c62ac11eb292d5a904`，运行 version=`b58336e-dirty`；app 1,621,296 B，binary SHA256=`14cb051e3351e61028e87d44d0903870fc7624384bfd5ee5000242af839097b5`，运行描述符完整 ELF SHA256=`6986f8b5859c63caeac1566ae16dd36e65be7669d44a4cddd2595f35df9f8a6d`。`dirty` 的构建差异 SHA 与 185 个生产输入逐项散列已保存，不以当前工作区内容替代烧录身份。后续已确认的 ADS DMA 缓冲部分分配错误修复不在该镜像内，须独立构建/验收。

This section closes the owner self-observation proposed above; **it does not declare the BLE throughput repair complete**. Source commit is `b58336eeaa136d0aca23e4c62ac11eb292d5a904`, running version `b58336e-dirty`. The app is 1,621,296 bytes, with binary SHA256 `14cb051e3351e61028e87d44d0903870fc7624384bfd5ee5000242af839097b5` and full running-descriptor ELF SHA256 `6986f8b5859c63caeac1566ae16dd36e65be7669d44a4cddd2595f35df9f8a6d`. The build-diff hash and all 185 production-input hashes are retained; the present checkout is not substituted for flashed identity. The subsequently confirmed ADS partial-DMA-buffer-allocation repair is not in this image and requires separate build/acceptance evidence.

[构建与测试身份](artifacts/cdc_recovery_acceptance_20261116/cpu-owner-01-validated-identity.json)对应 ESP-IDF 5.5.5 实际构建 exit 0 和 [65/65 host 入口 exit 0](artifacts/cdc_recovery_acceptance_20261116/cpu-owner-02-tests-manifest.json)，构建/测试生产输入一致。第一次完整测试中 recovery harness 未链接新增 CPU API 的失败仍在 [原 manifest](artifacts/cdc_recovery_acceptance_20261116/cpu-owner-01-tests-manifest.json)；后续联入真实默认 OFF 模块并执行 early query，未用宽松假桩掩盖失败。[烧录记录](artifacts/cdc_recovery_acceptance_20261116/cpu-owner-01-flash-usj-result.json)原有 BOOT_UNVERIFIED 状态保留；新增 [启动核验](artifacts/cdc_recovery_acceptance_20261116/cpu-owner-01-boot-verification.json)用后续 raw SHA、完整 BUILD、READY=1 和两次一致 CONFIG 补齐，session=`71055F1EF49B5A00`、bootId=1、CAP8。最早 BOOT=boot_sweep/ready0 是启动过程观测，不冒称当时已就绪。

The [build/test identity](artifacts/cdc_recovery_acceptance_20261116/cpu-owner-01-validated-identity.json) ties an actual ESP-IDF 5.5.5 build exit zero to [65/65 host entries exiting zero](artifacts/cdc_recovery_acceptance_20261116/cpu-owner-02-tests-manifest.json), with identical production inputs. The first full-suite recovery-harness failure, caused by missing linkage for the new CPU API, remains in its [original manifest](artifacts/cdc_recovery_acceptance_20261116/cpu-owner-01-tests-manifest.json). The harness was then linked to the real default-OFF module and exercised early queries, not replaced with permissive stubs. The [flash record](artifacts/cdc_recovery_acceptance_20261116/cpu-owner-01-flash-usj-result.json) retains its historical BOOT_UNVERIFIED status. A new [boot verification](artifacts/cdc_recovery_acceptance_20261116/cpu-owner-01-boot-verification.json) closes that boundary using later raw hashes, full BUILD, READY=1 and two matching CONFIG replies: session `71055F1EF49B5A00`, bootId 1, CAP8. The initial boot_sweep/ready0 observation is not relabeled ready.

该诊断镜像显式开启 `SENSORARRAY_CPU_OWNER_DIAGNOSTICS`、FreeRTOS runtime stats、U64 counter、ESP_TIMER 和 trace；正常 defaults 仍关闭。实际 IDF 5.5.5 Xtensa 路径将 `esp_timer_get_time()` 直接转换为 U64，无中间 32 位截断；限制为双核、非 Amazon SMP、非 UNICORE。每角色仅在原 checkpoint 自采、最多每秒一次；`vTaskGetInfo(NULL, …, pdFALSE, eReady)` 在应用锁外调用，内核 U64 复制受其双核 mux 保护，随后短锁发布快照。没有全任务/栈扫描、外部 task handle 生命周期或新 delay；首个 taskNumber 绑定后观测到第二 writer 即永久 conflict 至 boot，原工作预算不受该错误分支改变。本地新增静态存储为 456 B，trace/TCB 全局开销另计；开启 profiling 仍有真实开销，不能称零成本。[已解析配置](artifacts/cdc_recovery_acceptance_20261116/cpu-owner-resolved-config-01.json)与构建 sdkconfig SHA 相符。

This diagnostic image explicitly enables `SENSORARRAY_CPU_OWNER_DIAGNOSTICS`, FreeRTOS runtime statistics, the U64 counter, ESP_TIMER and trace; normal defaults remain disabled. The actual IDF 5.5.5 Xtensa path casts `esp_timer_get_time()` directly to U64 without an intermediate 32-bit truncation. It is restricted to dual-core, non-Amazon-SMP, non-UNICORE builds. Each role self-samples only at its existing checkpoint, at most once per second. `vTaskGetInfo(NULL, …, pdFALSE, eReady)` executes outside the application lock; the kernel's dual-core mux protects its U64 copy, followed by a short snapshot-publication lock. There is no all-task/stack scan, retained external task handle or new delay. A different observed writer after the first task-number binding makes the role conflicted until boot without changing the worker's budget logic. Local added static storage is 456 bytes; global trace/TCB overhead is separate. Profiling has real overhead and is not claimed free. The [resolved configuration](artifacts/cdc_recovery_acceptance_20261116/cpu-owner-resolved-config-01.json) matches the build's sdkconfig hash.

### 完整双通路窗口 / Complete dual-link window

120.000117 秒 VOLT1 同时记录正式 CDC0、BLE 和独立 CDC1 诊断，配置 rev=4、相同 session/boot；结束后查询确认 CAP8。启动 source 为 `gpio_tail_required` / `spi55_gpio1`，本 boot 实际 SPI 2 MHz。单纯完整采满窗口不表示通过。

VOLT1 ran for 120.000117 seconds with simultaneous formal CDC0, BLE and independent CDC1 diagnostics, configuration revision 4 in one session/boot. CAP8 was query-verified afterward. The boot source was `gpio_tail_required` / `spi55_gpio1` at an observed 2 MHz SPI clock. Filling the requested window alone does not mean acceptance.

| 指标 / Metric | screen07 |
|---|---:|
| Capture / USB / BLE | 59.900009 / 59.812416 / 32.390344 FPS |
| USB 可信完整帧 / Trusted complete frames | 7,178; missing sequence observations 1 |
| BLE 可信完整帧 / Trusted complete frames | 3,886; missing sequence observations 3,293 |
| ASCII CRC / identity / wire-contract errors | USB 0 / 0 / 0; BLE 0 / 0 / 0 |
| 共同 identity/seq 帧 / Common frames | 3,884; all original wire-byte SHA256 equal |
| 收到的 BLE 分片 / Received BLE fragments | DATA 3,887 assembled messages; fragment CRC/assembly errors 0 |
| ADC CRC | 0 rejected / 76,128 checked reads; observed ratio 0%, criterion ≤2.5% |
| ADC 计数跨度 / ADC counter span | 118.796879 s, first/last complete ADSHEALTH receipts inside capture |
| 无效点 / Invalid points | USB 406 X12; BLE 223 X12; not zero-filled or smoothed |
| fresh/acquired 不成立的点 / Not-fresh / unacquired points | USB 0 / 0; BLE 0 / 0 |
| BL50 `dropD` / `dropL` / `dropC` | +3,177 / 0 / 0 over its separate 119.823636 s receipt span |
| 完整 STK50 观测 / Complete STK50 observations | free 8,294,036 B; firmware lifetime minimum 8,290,104 B |
| 最小剩余任务栈 / Minimum remaining task stack | log 13,064 B; transport 3,180 B; BLE_TX 1,932 B; BLE_CTRL 5,236 B; SERIAL_CTRL 1,896 B |
| WDT / MFAULT / reset | Not observed in formal/debug recordings; absence remains bounded by evidence |
| 总体 / Overall | **FAIL_BLE_THROUGHPUT**; absolute accuracy NOT_EVALUATED |

ADC 分母为 valid-even Δ38,345 + valid-odd Δ37,783 + rejected Δ0，含重试的实际已校验读数，不是帧数；末尾未受保护的 `crcRetries` 文本仍 unknown。两个主机的 sequence range、BL50 receipt 窗口和查询窗口不同，不相加/相减推断空中丢包。完整 CDC observer 覆盖本采集窗，但没有 WDT 文本不能证明任何未记录事件绝不存在。一个 warmup 中截断的 STK50 按完整 schema 拒绝，未用数值阈值排除低堆；PERF 末尾 heapMin 不代替可信 STK50。最大连续 heap 块、总 CPU 与绝对精度仍 unknown/未评估。

The ADC denominator is valid-even Δ38,345 + valid-odd Δ37,783 + rejected Δ0: checked reads, including retry attempts, not frames. The unprotected terminal `crcRetries` token remains unknown. The two host sequence ranges, BL50 receipt span and query windows differ; their counts cannot be added/subtracted into over-air losses. The CDC observer covers the capture, but absent WDT text does not rule out unrecorded events. A truncated warmup STK50 was rejected by schema completeness, not by filtering small heap values; the terminal PERF heapMin does not replace trusted STK50. Maximum contiguous heap, total CPU and absolute accuracy remain unknown/not evaluated.

证据 / Evidence: [原 summary](artifacts/cdc_recovery_acceptance_20261116/screen-07/screen-07-volt1.summary.json), [独立科学/诊断分析](artifacts/cdc_recovery_acceptance_20261116/screen-07/screen-07-analysis-01.json), [判定](artifacts/cdc_recovery_acceptance_20261116/screen-07/screen-07-verdict.json), [逐字节归档核验](artifacts/cdc_recovery_acceptance_20261116/screen-07/archive-index-01.json)。本镜像 600 秒及 32 配置均 NOT_RUN；不把旧阶段结果移植成新镜像通过。 / This image's 600-second and 32-configuration acceptance are NOT_RUN; older-stage results are not transplanted as a pass.

### 各角色独立差分 / Independent per-role deltas

只在完整前后 CONFIG 同身份、CPUOWNER 同 session/taskNumber、已知且未过期、计数不倒退、采样时间有序时取差分。下表每行分母是该角色自己的 `observedAtUs` 跨度，采样调用的 t0/t1 边界也原样保存；它们不等于统一 120 秒采集窗。

Deltas require complete matching CONFIG brackets, identical CPUOWNER session/task number, known fresh samples, nonregressing counters and ordered sample times. Each row's denominator is its own `observedAtUs` span; original t0/t1 sampling brackets are retained. These are not one common 120-second capture window.

| Role / taskNumber | ΔrunUs | Own spanUs | Δrun / own span |
|---|---:|---:|---:|
| 0 BLE_TX / 9 | 17,903,690 | 120,095,170 | 14.907918% |
| 1 TRANSPORT / 15 | 9,070,485 | 119,846,299 | 7.568431% |
| 2 ASYNC_LOG / 17 | 30,015,103 | 120,182,388 | 24.974627% |
| 3 SERIAL_TX / 8 | 11,569,529 | 120,609,289 | 9.592569% |
| 4 BLE_CTRL / 10 | unknown | unknown | stale historical sample |
| 5 SERIAL_CTRL / 16 | 853,506 | 120,285,279 | 0.709568% |
| 6 WIFI_CTRL / unknown | unknown | unknown | not_observed |

BLE_CTRL 缓存中的 runUs=0 来自启动旧样本，前后 age 超过 305/425 秒，不能写成当前 CPU=0；Wi-Fi 未观测也不能填零。runUs 只在切出时累计，正在运行片段尚未计入，可能含 ISR 和 SDK 外工作。各行不可合计为双核 CPU 利用率，未覆盖 SDK/controller/IDLE task 的残差也不能归因。

BLE_CTRL's cached runUs=0 is an old startup sample, aged over 305/425 seconds at the two queries, not zero current CPU. An unobserved Wi-Fi role is also unknown. Runtime accumulates only at switch-out, excludes the open running slice and may include ISR and non-SDK work. The rows cannot be summed into dual-core CPU utilization, and unobserved SDK/controller/idle-task residuals cannot be attributed.

独立 BLESVC 查询窗约 120.538556 秒：completed Δ12,991，wallUs Δ62,018,883，平均 4,773.988 µs/完成调用；CONF ok Δ12,991、error Δ0，仍不是 PC 收到证明。BLE_TX 自身累计运行明显小于附近 SDK 墙钟，但窗口与工作范围不同；**不计算 wall−run=blocked**，就绪等待、阻塞、controller 工作和中断占比都尚 unknown。此候选仅增加观测，未修复 BLE 服务率；screen06/07 也不是严格控制所有变量的 profiling 开销 A/B。

The independent BLESVC query window spans approximately 120.538556 seconds: completed Δ12,991, wallUs Δ62,018,883, averaging 4,773.988 µs per completed call; CONF ok Δ12,991 and error Δ0 still do not prove PC receipt. BLE_TX's own cumulative runtime is substantially below the nearby SDK wall observation, but their windows and work scopes differ. **No wall−run=blocked calculation is made.** Ready waiting, blocking, controller work and ISR shares remain unknown. This candidate adds observations rather than repairing BLE service rate; screen06/07 are not a fully controlled profiling-overhead A/B.

[CPU 差分 JSON](artifacts/cdc_recovery_acceptance_20261116/screen-07/screen-07-cpu-owner-deltas-01.json)校验原 raw SHA、完整行、event phase、CONFIG identity 与各字段，不拼补截断尾行。新增离线分析器的 7 个合成回归实际通过，覆盖独立窗口、超过 32 位计数、stale/unknown、跨 task/session、倒退/重叠、缺 schema/event、raw SHA 不符和截断；这些是另外 7 个测试用例，不把 65 个入口改称 72 个入口。[命令与 SHA](artifacts/cdc_recovery_acceptance_20261116/screen-07-cpu-analysis-01-manifest.json)。

The [CPU delta JSON](artifacts/cdc_recovery_acceptance_20261116/screen-07/screen-07-cpu-owner-deltas-01.json) verifies the original raw hash, complete lines, event phases, CONFIG identity and fields without repairing truncated tails. Seven new synthetic offline regression cases actually passed, covering independent windows, counters beyond 32 bits, stale/unknown values, task/session changes, regressions/overlaps, missing schemas/events, raw-hash mismatch and truncation. These are seven additional test cases, not a claim of 72 test entries. See the [commands and hashes](artifacts/cdc_recovery_acceptance_20261116/screen-07-cpu-analysis-01-manifest.json).

离线复跑使用新输出名，既有输出不会被覆盖；不打开设备。 / Offline replay requires a new output name, preserves existing outputs and opens no device:

```sh
python3 docs/audit/artifacts/cdc_recovery_acceptance_20261116/scripts/test_analyze_cpu_owner.py
python3 docs/audit/artifacts/cdc_recovery_acceptance_20261116/scripts/analyze_cpu_owner.py \
  --summary docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-07/screen-07-volt1.summary.json \
  --output /tmp/screen07-cpu-replay-new.json
```

## screen08：P1 分配修复、日志清理与独立对照 / screen08: allocation P1 repair, log cleanup and independent comparison

当前完整提交 `b9ebe2f7b8b49819347608392e70ff5966f2ad87`、BUILD=`b9ebe2f-dirty`；app 1,620,752 B，binary SHA256=`b4ae27a3fc5da400febf6db587f0d828f9dd6666e617ec26a071b4988217b3d3`，ELF SHA256=`529b440aae12e5cecf2abed938558cc392009e97f929970ea6191ae5bc9b2b8c`。完整 BUILD、BOOT、READY 与 CAP8 已在[启动核验](artifacts/cdc_recovery_acceptance_20261116/output-trim-02-boot-verification.json)闭合；session=`FD091D519F388B90`、bootId=1，启动 SPI 请求/实测均 2 MHz、`gpio_tail_required`。CPU owner/runtime/trace 诊断仍 ON，正常默认配置仍 OFF；不把两个不同 session 的 taskNumber 或累计值大小相互比较。

The current full commit is `b9ebe2f7b8b49819347608392e70ff5966f2ad87`, BUILD `b9ebe2f-dirty`. The app is 1,620,752 bytes, binary SHA256 `b4ae27a3fc5da400febf6db587f0d828f9dd6666e617ec26a071b4988217b3d3`, ELF SHA256 `529b440aae12e5cecf2abed938558cc392009e97f929970ea6191ae5bc9b2b8c`. The [boot verification](artifacts/cdc_recovery_acceptance_20261116/output-trim-02-boot-verification.json) closes full BUILD, BOOT, READY and CAP8 identity: session `FD091D519F388B90`, bootId 1, requested/observed SPI 2 MHz with `gpio_tail_required`. CPU owner/runtime/trace diagnostics remain ON while normal defaults remain OFF. Task numbers and cumulative magnitudes are not compared between sessions.

`c670897` 修复已复现的 `ads126xAdcAllocSpiBuffers` 部分分配错误：tx/rx 先以局部指针分配，所有分配成功后才发布到 device；失败仍返回首个 NO_MEM，避免失败时释放后 Init/Deinit 再次释放。既有 in-flight/DMA 所有权规则保持；[真实生产 Init/Deinit 回归](artifacts/cdc_recovery_acceptance_20261116/ads-alloc-regression-01-command.txt)覆盖 S3/非 S3 各 16 组合，共 32，duplicate-free/leak 均 0。这是合成分配失败注入，不称实机 OOM 测试。

`c670897` repairs the reproduced partial-allocation defect in `ads126xAdcAllocSpiBuffers`: tx/rx allocations stay in local pointers until all succeed, then are published to the device. Failure retains NO_MEM and avoids a second free by Init/Deinit. Existing in-flight/DMA ownership rules remain. The [production Init/Deinit regression](artifacts/cdc_recovery_acceptance_20261116/ads-alloc-regression-01-command.txt) covers 16 S3 and 16 non-S3 combinations, 32 total, with zero duplicate frees/leaks. Allocation-failure injection is synthetic, not a claimed hardware OOM test.

`98b760c` 只移除不可达 PREC20 与停用 legacy netStatus 消费的逐点统计；活跃 summary 状态及输出按字节保持一致。第一轮 output-trim-01 的 BLE 清理测试断言只接受 3 个错误，但总期限耗尽时真实逻辑正确记录了第四个 `cleanup deadline` 错误；[原失败日志](artifacts/cdc_recovery_acceptance_20261116/output-trim-01-test_capture_benchmark_ble.txt)保留。`b9ebe2f` 只改该测试对精确 deadline 分支的要求并新增确定性用例，生产 collector 的总期限不变。[最终 66 个入口](artifacts/cdc_recovery_acceptance_20261116/output-trim-02-tests-manifest.json)与[实际构建](artifacts/cdc_recovery_acceptance_20261116/output-trim-02-build-manifest.json)均 exit 0，185 个生产输入完全一致；这不是通过延长 timeout 得到的结果。

`98b760c` removes only per-point statistics consumed by unreachable PREC20 and disabled legacy netStatus; active summary state/output remain byte-equivalent. In output-trim-01, a BLE-cleanup test expected only three errors, but the real logic correctly recorded a fourth cleanup-deadline error when its total budget expired. The [original failure log](artifacts/cdc_recovery_acceptance_20261116/output-trim-01-test_capture_benchmark_ble.txt) remains. `b9ebe2f` changes only that test's exact-deadline expectation and adds a deterministic case; the production collector's total deadline is unchanged. All [66 final test entries](artifacts/cdc_recovery_acceptance_20261116/output-trim-02-tests-manifest.json) and the [actual target build](artifacts/cdc_recovery_acceptance_20261116/output-trim-02-build-manifest.json) exited zero, with identical 185 production inputs. Passing was not obtained by extending a timeout.

没有改动 2.5% CRC 门限、160 MHz CPU、`-Og`/断言、BLE/CTRL 优先级、WorkBudget、科学有效性或 wire 协议。[源码变化范围](artifacts/cdc_recovery_acceptance_20261116/output-trim-source-diff-01.json)单独保存，避免把这两项修复说成 BLE 调度修复。

The 2.5% CRC criterion, 160 MHz CPU, `-Og`/assertions, BLE/CTRL priorities, WorkBudget, scientific validity and wire protocol remain unchanged. The [source change scope](artifacts/cdc_recovery_acceptance_20261116/output-trim-source-diff-01.json) is recorded separately; these two repairs are not a BLE scheduling repair.

### 科学接收与各层计数 / Scientific receipt and layer counters

screen08 是 120.000130 秒 VOLT1 正式 CDC0+BLE，同时独立 CDC1 观测诊断；同 session/rev4。科学帧结束后已查询确认恢复 CAP8。 / Screen08 records 120.000130 seconds of VOLT1 through formal CDC0 and BLE, with independent CDC1 diagnostics, in one session/revision 4. CAP8 recovery was query-verified afterward.

| 指标 / Metric | screen07 | screen08 |
|---|---:|---:|
| Capture FPS | 59.900009 | 59.184655 |
| USB FPS / trusted complete frames | 59.812416 / 7,178 | 59.173785 / 7,101 |
| BLE FPS / trusted complete frames | 32.390344 / 3,886 | 33.118267 / 3,974 |
| USB internal missing sequence observations | 1 | 0: seq 1259–8359, ordered 7,101 |
| BLE missing sequence observations | 3,293 | 3,128 |
| USB / BLE ASCII CRC errors | 0 / 0 | 0 / 0 |
| USB / BLE identity / wire-contract errors | 0 / 0 / 0 / 0 | 0 / 0 / 0 / 0 |
| 两端共同帧原字节相同 / Common original frames equal | 3,884 / 3,884 | 3,971 / 3,971 |
| ADC rejected / checked reads | 0 / 76,128 | 0 / 76,841 |
| ADC health receipt span | 118.796879 s | 118.996649 s |
| USB S1D2 X12 invalid points | 406 | 525 |
| BLE X12 invalid points | 223 | 303 |
| USB/BLE not-fresh or unacquired points | 0 | 0 |
| 完整 STK50 free / lifetime minimum / B | 8,294,036 / 8,290,104 | 8,295,596 / 8,292,112 |
| 总体验收 / Overall | FAIL_BLE_THROUGHPUT | **FAIL_BLE_THROUGHPUT** |

screen08 analyzer 未输出 `missing_frames` 键，不据此补零。新闭环核验只有一个 identity group、ordered trusted=7,101、last−first+1=7,101、progress=7,100 且 gap_examples=[]，由此证明首末可信帧范围内连续；范围外仍不承诺无丢失。BLE 收到的 3,975 个 DATA 分片消息组装没有 gap/CRC/重复/乱序错误，但完整可信科学帧仅 3,974；两者窗口/尾部边界不同，不能宣称整个 BLE 路径无背压。

The analyzer omits `missing_frames` for screen08; this is not defaulted to zero. The new closure checks one identity group, 7,101 ordered trusted frames, last−first+1=7,101, progress=7,100 and gap_examples=[], establishing contiguity inside the trusted first/last range only. BLE's 3,975 received DATA-message assemblies have no fragment gap/CRC/duplicate/order error, but there are 3,974 trusted scientific frames; window/tail boundaries differ. Neither proves an absence of BLE backpressure.

| 层级独立窗口 / Independent layer window | screen07 within-window delta | screen08 within-window delta |
|---|---|---|
| USBQ receipt span | 120.483491 s | 120.592613 s |
| USBQ `dSubmit/cSubmit` | 7,205 / 111 | 7,140 / 111 |
| USBQ `dDrop/cDrop/expired/cExpired/partial` | 1 / 0 / 0 / 0 / 0 | 0 / 0 / 0 / 0 / 0 |
| USBQ `dBytes/cBytes` | 4,313,661 / 26,249 B | 4,273,137 / 26,187 B |
| BL50 receipt span | 119.823636 s | 119.650096 s |
| BL50 `sentD/dropD/dropL/dropC` | 3,882 / 3,177 / 0 / 0 | 3,963 / 3,037 / 0 / 0 |
| BLETX query span | 120.518957 s | 120.602186 s |
| BLETX `sdkCalls/sdkAccepted/sdkErrors` | 12,986 / 12,986 / 0 | 13,251 / 13,251 / 0 |
| BLESVC completed / wall delta / mean wall | 12,991 / 62.018883 s / 4.773988 ms | 13,248 / 61.923999 s / 4.674215 ms |
| BLESVC `confOk/confError`; after.active | 12,991 / 0; 1 | 13,249 / 0; 1 |

USBQ 字节只表示本地 FIFO 接受；CDC 分 lane 回调字节和 PC receipt 仍 unknown。screen08 旧累计 expired=1037、partial=3 在查询窗内未增长，不能把旧累计误报成实时故障。BL50 pool 丢弃、BLETX SDK 接受、BLESVC 回调和主机缺口属于不同层/窗口，不加减为互斥损失；CONF 先于 wrapper 完成且 after.active=1 可以产生一条差额。USB epoch/mount/reset 计数无增量；完整正式/调试记录未观测到 MFAULT/reset/WDT，但无日志不构成无限安全保证。ADC 分母 38,910 even +37,931 odd +0 rejected，仍含重试尝试，不是科学帧数，末尾 crcRetries 文本仍 unknown。

USBQ byte counts indicate local FIFO acceptance only; per-lane CDC callback bytes and PC receipt remain unknown. Screen08's old totals expired=1037 and partial=3 did not grow in the query window and are not current faults. BL50 pool drops, BLETX SDK acceptance, BLESVC callbacks and host gaps belong to different layers/windows and cannot be combined into disjoint losses. CONF may precede wrapper completion, so after.active=1 can explain a one-event difference. USB epoch/mount/reset counts did not increase. Formal/debug recordings show no MFAULT/reset/WDT, without an unlimited safety guarantee from absent text. The ADC denominator is 38,910 valid-even +37,931 valid-odd +0 rejected checked reads, including retries, not scientific frames; the terminal crcRetries token remains unknown.

### CPU owner 对照与结论边界 / CPU-owner comparison and limits

每行先在各自 boot 的本角色样本间求差，再比较比率；绝不把跨 boot 累计 runtime 相减。 / Each row is first differenced within its own boot and role sample bracket, then its ratio is compared. Cumulative runtime is never subtracted across boots.

| Role | screen08 ΔrunUs / own spanUs | screen07 ratio | screen08 ratio | Difference / percentage points |
|---|---:|---:|---:|---:|
| BLE_TX | 18,189,732 / 120,992,137 | 14.907918% | 15.033813% | +0.125895 |
| TRANSPORT | 9,125,833 / 120,906,284 | 7.568431% | 7.547857% | −0.020575 |
| ASYNC_LOG | 29,526,020 / 120,021,728 | 24.974627% | 24.600562% | −0.374065 |
| SERIAL_TX | 11,760,714 / 120,679,896 | 9.592569% | 9.745380% | +0.152811 |
| BLE_CTRL | unknown: stale historical | unknown | unknown | unknown |
| SERIAL_CTRL | 947,358 / 120,534,705 | 0.709568% | 0.785963% | +0.076395 |
| WIFI_CTRL | unknown: not_observed | unknown | unknown | unknown |

这些不是同步 CPU 百分比分账。ASYNC_LOG 约 24.60% 对比 24.97% 的降低不足以解释或解决 BLE 仅 33 FPS；本次不能宣布主要原因已修复，也不能用小差异作单因素因果证明。SDK/IDLE 任务与 ISR 占比仍未单独观测；`wall−run` 不等于 blocked。科学无效 X12 继续保留，不能以临时参考/环境背景把绝对电压精度宣告通过。当前 600 秒/32 配置仍 NOT_RUN，下一修复与复测另建标签。

These are not simultaneous CPU-utilization shares. ASYNC_LOG at roughly 24.60% versus 24.97% neither explains nor resolves BLE's approximately 33 FPS. The primary cause is not declared fixed, and a small difference is not a controlled single-factor causal result. SDK/idle tasks and ISR shares remain separately unobserved; wall−run is not blocked time. Scientific X12 invalids remain, and temporary references/environmental background do not establish absolute voltage accuracy. Current 600-second/32-configuration acceptance remains NOT_RUN; later repairs/retests require new labels.

新增 [通用离线判定脚本](artifacts/cdc_recovery_acceptance_20261116/scripts/summarize_diagnostic_screen.py)不硬编码 screen 编号，显式接收 reviewer verdict，逐项核 summary/raw/events/CPU/boot 来源 SHA，输出新文件而不覆盖旧证据。6 个合成回归实际通过，包括缺零键的充分条件、证据不足 unknown、同身份 uint32 回绕、显式缺口保留、不同窗口/boot CPU 比率与未知角色。它们是新增离线用例，不改变 66 个 host 入口数。[执行与散列 manifest](artifacts/cdc_recovery_acceptance_20261116/screen-08-closure-01-manifest.json)。

The new [generic offline closure script](artifacts/cdc_recovery_acceptance_20261116/scripts/summarize_diagnostic_screen.py) has no hard-coded screen number, takes an explicit reviewer verdict, verifies summary/raw/event/CPU/boot evidence hashes and creates new outputs without overwriting recordings. Six synthetic regressions actually passed, covering sufficient proof for an omitted zero key, insufficient-evidence unknowns, same-identity uint32 wrap, explicit gaps, independent cross-run ratios and unknown roles. These are additional offline cases, not an increase in the 66 host-entry count. See the [execution/hash manifest](artifacts/cdc_recovery_acceptance_20261116/screen-08-closure-01-manifest.json).

证据 / Evidence: [screen08 summary](artifacts/cdc_recovery_acceptance_20261116/screen-08/screen-08-volt1.summary.json), [科学/诊断分析 / scientific/diagnostic analysis](artifacts/cdc_recovery_acceptance_20261116/screen-08/screen-08-analysis-01.json), [CPU 单次差分 / per-run CPU deltas](artifacts/cdc_recovery_acceptance_20261116/screen-08/screen-08-cpu-owner-deltas-01.json), [screen08 verdict](artifacts/cdc_recovery_acceptance_20261116/screen-08/screen-08-verdict.json), [screen07–08 分层/CPU 对照 / layer/CPU comparison](artifacts/cdc_recovery_acceptance_20261116/screen-08/screen-07-08-layer-cpu-comparison-01.json)。所有原始流只读，没有修改生产、重新构建或访问硬件。 / All raw streams were read-only; this closure changed no production source, rebuilt nothing and accessed no hardware.

## screen09：元数据单遍解析 / screen09: single-pass frame metadata parsing

已烧录提交 `dafaf9bc841e756b7d78aa54391703a952b819e5`，BUILD=`dafaf9b`，app 1,620,976 B，binary SHA256=`ae0975e0d4ea203f4914a491be876d9bee435ac56ed7f1524f5a5d939ccb8a50`，完整 ELF SHA256=`922e1c880317d4ace18eb2ec2b368ad7d3b9a1ac1c548f7e34a63182ffeb2c10`。session=`48E77595DEBD273B`、bootId=1；[完整 BUILD/BOOT/READY 核验](artifacts/cdc_recovery_acceptance_20261116/frame-meta-01-boot-verification.json)确认 CAP8 可运行，启动选择 `gpio_tail_required`、SPI 请求/实测 2 MHz。160 MHz CPU、`-Og`、断言、CPU owner/runtime/trace ON 与上轮相同；正常诊断 defaults 仍 OFF。[66 个 host 入口及实际构建](artifacts/cdc_recovery_acceptance_20261116/frame-meta-01-validated-identity.json)通过，185 个生产输入逐项一致，构建差异为空。后续准备的 `-O2` 候选不属于本镜像或本次 HIL。

The flashed commit is `dafaf9bc841e756b7d78aa54391703a952b819e5`, BUILD `dafaf9b`, app size 1,620,976 bytes, binary SHA256 `ae0975e0d4ea203f4914a491be876d9bee435ac56ed7f1524f5a5d939ccb8a50`, full ELF SHA256 `922e1c880317d4ace18eb2ec2b368ad7d3b9a1ac1c548f7e34a63182ffeb2c10`. Session is `48E77595DEBD273B`, bootId 1. [Full BUILD/BOOT/READY verification](artifacts/cdc_recovery_acceptance_20261116/frame-meta-01-boot-verification.json) establishes running CAP8, with `gpio_tail_required` and requested/observed SPI 2 MHz. CPU 160 MHz, `-Og`, assertions and CPU owner/runtime/trace ON remain unchanged; normal diagnostic defaults remain OFF. All [66 host entries and the target build](artifacts/cdc_recovery_acceptance_20261116/frame-meta-01-validated-identity.json) passed with identical 185 production inputs and an empty build-source diff. The subsequently prepared `-O2` candidate is not this image or this HIL result.

### 实施与语义回归 / Implementation and semantic regression

改动限于 `core/transport/sensorarrayTransportFrameMeta.c` 的 `sensorarrayTransportParseFrameMeta` 和私有 key 表。实际调用链是 AsyncLog/output owner → `sensorarrayNetTextPublish` → transport 拷贝 payload 到独占 slot → 同步提取元数据 → descriptor 入队；原实现对最多 14 个字段反复扫描 header，这部分时间属于发布者而不是 BLE worker。新实现首行定位一次、comma token 遍历一次，以 seen/valid mask 和局部数值保留“第一个非空字段优先”的既有语义，再一次发布已验证元数据。

The change is confined to `sensorarrayTransportParseFrameMeta` and its private key table in `core/transport/sensorarrayTransportFrameMeta.c`. The actual path is AsyncLog/output owner → `sensorarrayNetTextPublish` → transport copies the payload into an exclusive slot → synchronous metadata extraction → descriptor enqueue. The former parser rescanned the header for up to 14 fields, so this cost belongs to the publisher, not the BLE worker. The new parser locates the first-line boundary once, traverses comma tokens once, retains the existing first-nonempty-field rule using seen/valid masks and local values, then publishes validated metadata together.

保持旧 C=rows、V/R=mode、M legacy aliases=rgen/rrid 的域，不改任何显式 mode/rows/profile 身份；只解析已复制字节，不读取最新全局状态。保留 required/optional 数值错误、重复字段、CR/LF/NUL、strtoull 边界与失败时旧 slot 字段行为；`hasFrameMeta` 仍不等于完整科学帧或 CRC 有效。CAP preview 与旧 helper 保持原字节。没有修改 encoder/wire/CRC、科学 freshness、PGA、队列容量、任务优先级、WorkBudget，也没有关闭原 stdout 日志。固定 14×uint64 局部值（112 B）和 mask 无新增 heap/任务；新目标栈水位见下表，不以主机编译大小替代实测。

Legacy domains remain C=rows, V/R=mode and M aliases=rgen/rrid, with all explicit mode/rows/profile identities separate. The parser reads copied bytes, never current globals. Required/optional numeric errors, duplicates, CR/LF/NUL, strtoull boundaries and stale-slot behavior on failure remain; `hasFrameMeta` still does not prove a complete scientific frame or valid CRC. CAP preview and the old helper retain their bytes. Encoder/wire/CRC, scientific freshness, PGA, queue capacity, priorities and WorkBudget are unchanged, and existing stdout logs remain enabled. The fixed 14×uint64 local array (112 bytes) and masks add no heap or task; target stack observations are below rather than inferred from host object sizes.

[生产差分 manifest](artifacts/cdc_recovery_acceptance_20261116/transport-meta-single-pass-host-01.json)与[本镜像测试 stdout](artifacts/cdc_recovery_acceptance_20261116/frame-meta-01-test_transport_frame_metadata.txt)：90 个固定边界、50,000 个确定种子、20 个完整帧、16,786 个逐字节前缀截断，共 **66,896** 次整 slot 比较（51,663 接受/15,233 拒绝）通过。20 帧中 15 个由真实 encoder 合成、5 个来自既有设备 encoder 实录；原字节/CRC 经 zlib 与真实 parser 双重验证，未手造 CRC。相同语料 ASan/UBSan 实际通过；LeakSanitizer 在执行环境 ptrace 下失败，**LSan NOT_RUN**，首次失败日志保留，不称内存泄漏检查通过。主机 x86 `-O2` 微基准不是 ESP32 结果，不能与本板 `-Og` 混用。

The [production differential manifest](artifacts/cdc_recovery_acceptance_20261116/transport-meta-single-pass-host-01.json) and [this image's test stdout](artifacts/cdc_recovery_acceptance_20261116/frame-meta-01-test_transport_frame_metadata.txt) record **66,896** full-slot comparisons: 90 fixed boundaries, 50,000 seeded cases, 20 complete frames and 16,786 byte-prefix truncations, with 51,663 accepted and 15,233 rejected. Of the 20 frames, 15 are synthetic outputs of the real encoder and five are existing device-encoder recordings. Original bytes/CRC were checked using both zlib and the actual parser, without fabricated CRCs. The same corpus actually passed ASan/UBSan. LeakSanitizer failed under the execution environment's ptrace restriction, so **LSan is NOT_RUN**, with the initial log preserved, not a passing leak check. Host x86 `-O2` microbenchmarks are not ESP32 results and must not be mixed with this board's `-Og` build.

### screen09 结果和各层窗口 / screen09 results and layer windows

完整 120.000062 秒 VOLT1，正式 CDC0/BLE 同时采集，独立 CDC1 诊断；同 session/rev4，结束后 CAP8 cleanup 查询通过。 / VOLT1 completed 120.000062 seconds of simultaneous formal CDC0/BLE capture with independent CDC1 diagnostics in one session/revision 4; CAP8 cleanup was query-verified afterward.

| 指标 / Metric | screen08 | screen09 |
|---|---:|---:|
| Capture / USB / BLE FPS | 59.184655 / 59.173785 / 33.118267 | 57.787813 / 57.664438 / 39.133976 |
| USB trusted complete frames / internal missing observations | 7,101 / 0 | 6,920 / 0 |
| BLE trusted complete frames / missing observations | 3,974 / 3,128 | 4,695 / 2,223 |
| USB / BLE ASCII CRC, identity and wire-contract errors | all 0 | all 0 |
| 共同 identity/seq 原字节一致 / Common original frames equal | 3,971 / 3,971 | 4,691 / 4,691 |
| ADC rejected / checked reads | 0 / 76,841 | 0 / 74,721 |
| Complete ADSHEALTH receipt span | 118.996649 s | 118.765452 s |
| USB S1D2 X12 / BLE X12 | 525 / 303 | 467 / 340 |
| USB/BLE not-fresh or unacquired points | 0 | 0 |
| Complete STK50 free / lifetime minimum | 8,295,596 / 8,292,112 B | 8,295,596 / 8,292,112 B |
| Minimum remaining log / transport / BLE_TX stack | 13,064 / 3,152 / 1,932 B | 13,064 / 3,184 / 1,932 B |
| Overall acceptance | FAIL_BLE_THROUGHPUT | **FAIL_BLE_THROUGHPUT** |

USB 内部零缺口由单 identity、有序可信 6,920 帧、seq1600…8519、progress6919 和 gap_examples=[] 共同证明；不存在 `missing_frames` 键并非证明。BLE 收到的 DATA 4,695 和 LOG 278 个完整分片消息无 gap/CRC/重复/乱序错误，但完整科学序号仍缺 2,223；片段完整不等于没有上游整帧丢弃。ADC 分母是健康日志已校验 even/odd/rejected 增量，不是帧数；2.5% 门限不变。浮空输入 S1D2 的 X12 不作零填补或平滑，也不解释为校准精度通过。

Zero internal USB loss is established by one identity, 6,920 ordered trusted frames, seq1600…8519, progress6919 and gap_examples=[]; an absent `missing_frames` key alone is not proof. BLE received 4,695 DATA and 278 LOG message assemblies without fragment gap/CRC/duplicate/order errors, while 2,223 complete scientific sequence observations are still missing. Intact fragments do not eliminate upstream whole-frame drops. The ADC denominator is the complete-health valid-even/valid-odd/rejected delta, not frames; the 2.5% criterion is unchanged. X12 on floating S1D2 remains invalid, without zero fill, smoothing or a claim of calibration accuracy.

| 层级 / Layer | screen08 within-window delta | screen09 within-window delta |
|---|---|---|
| USBQ receipt span | 120.592613 s | 120.402249 s |
| USBQ `dSubmit/cSubmit` | 7,140 / 111 | 6,943 / 111 |
| USBQ `dDrop/cDrop/expired/cExpired/partial` | all 0 | all 0 |
| USBQ `dBytes/cBytes` | 4,273,137 / 26,187 B | 4,155,302 / 26,184 B |
| BL50 receipt span | 119.650096 s | 119.641568 s |
| BL50 `sentD/dropD/dropL/dropC` | 3,963 / 3,037 / 0 / 0 | 4,683 / 2,201 / 0 / 0 |
| BLETX query span | 120.602186 s | 120.401641 s |
| BLETX `sdkCalls/sdkAccepted/sdkErrors` | 13,251 / 13,251 / 0 | 15,379 / 15,378 / 0 |
| BLESVC completed / wall delta / mean wall | 13,248 / 61.923999 s / 4.674215 ms | 15,383 / 62.979474 s / 4.094096 ms |
| BLESVC `confOk/confError` | 13,249 / 0 | 15,382 / 0 |

USB FIFO 接受和 SDK 接受不是 PC 收到。各行的 query/receipt 窗口不同；活跃调用及 CONF 与 wrapper 的异步时序可形成 1 条边界差，不能把它们与主机缺口相加。WDT/MFAULT/reset 在完整正式/调试窗口内未观测到；缺失日志不能证明未记录事件不存在。最大连续 heap、SDK CPU、blocked 和绝对精度仍 unknown/未评估。

USB FIFO and SDK acceptance do not establish PC receipt. Query/receipt windows differ, and active calls plus asynchronous CONF/wrapper timing can produce one-event boundary differences; these are not added to host gaps. No WDT/MFAULT/reset was observed in the covered formal/debug window, without ruling out unrecorded events. Maximum contiguous heap, SDK CPU, blocked time and absolute accuracy remain unknown/not evaluated.

### CPU 变化与剩余阻塞 / CPU changes and remaining blocker

| Role | screen09 ΔrunUs / own spanUs | screen08 ratio | screen09 ratio |
|---|---:|---:|---:|
| BLE_TX | 21,167,994 / 119,927,415 | 15.033813% | 17.650671% |
| TRANSPORT | 8,935,494 / 120,069,062 | 7.547857% | 7.441962% |
| ASYNC_LOG | 16,217,602 / 120,181,452 | 24.600562% | 13.494264% |
| SERIAL_TX | 12,332,742 / 120,513,737 | 9.745380% | 10.233474% |
| BLE_CTRL | unknown: stale historical sample | unknown | unknown |
| SERIAL_CTRL | 948,055 / 120,135,925 | 0.785963% | 0.789152% |
| WIFI_CTRL | unknown: not_observed | unknown | unknown |

本对照 ASYNC_LOG 比率下降 11.106299 个百分点，BLE 实收提高 **18.164321%**，支持单遍解析消除了发布路径的一部分实际开销；但 BLE39.13FPS 仍低于 capture57.79FPS，采集本次下降且各运行的输入/调度时序并不完全相同，不将一次对照当作所有瓶颈的因果分解。各角色本窗口 switch-out 差分独立，不能跨 boot 相减、不能求和成全核 CPU、不能用 SDK 墙钟减 owner runtime 推 blocked。仍需下一候选和有界复测；完整 600 秒/32 配置继续 NOT_RUN。下一 `-O2` 候选尚未烧录到本次结果，不提前填通过数。

In this comparison, ASYNC_LOG's ratio fell 11.106299 percentage points and BLE receipt rose **18.164321%**, supporting removal of a material publisher-path cost. BLE at 39.13 FPS still trails capture at 57.79 FPS. Capture fell in this run, and input/scheduling timing is not identical between runs, so one comparison is not a causal decomposition of all bottlenecks. Each role's switch-out delta uses its own window: no cross-boot subtraction, aggregate core-CPU sum or SDK-wall-minus-owner-runtime blocked estimate is valid. Further candidates and bounded retests remain necessary; the complete 600-second/32-configuration suite is still NOT_RUN. The next `-O2` candidate has not been flashed as part of these results, and no future passing counts are filled in.

证据 / Evidence: [原 summary](artifacts/cdc_recovery_acceptance_20261116/screen-09/screen-09-volt1.summary.json), [独立科学/诊断分析](artifacts/cdc_recovery_acceptance_20261116/screen-09/screen-09-analysis-01.json), [CPU delta](artifacts/cdc_recovery_acceptance_20261116/screen-09/screen-09-cpu-owner-deltas-01.json), [判定](artifacts/cdc_recovery_acceptance_20261116/screen-09/screen-09-verdict.json), [08→09 分层/CPU 对照](artifacts/cdc_recovery_acceptance_20261116/screen-09/screen-08-09-layer-cpu-comparison-01.json), [实际闭环命令及 SHA](artifacts/cdc_recovery_acceptance_20261116/screen-09-closure-01-manifest.json)。本轮复用未修改的通用离线脚本，实际 exit0；原 stdout/raw/events 与历史失败均保留，文档闭合未修改生产、测试或采集工具，也未执行 build/HIL。 / The unchanged generic offline script actually exited zero. Original stdout/raw/events and historical failures remain; this documentation closure modified no production, test or capture tool and performed no build/HIL.

## screen10：保留检查的 `-O2` 与实机对照 / screen10: `-O2` with retained checks and hardware comparison

已烧录提交 `a656d53f2efc447507f5247a116a22aed35dcc04`，BUILD=`a656d53`；app 1,591,888 B，binary SHA256=`dc6c74800d88760a8ba85274b6954317f6a95d019bf2d4883bea7cdf406a3dd6`，完整 ELF SHA256=`63ab97c31a1c8c7ca92e631661e42ad7e6357e0475a38f2cc5270959973c36f4`。session=`B528ED81F93297CF`、bootId=1；[完整 BUILD/BOOT/READY](artifacts/cdc_recovery_acceptance_20261116/compiler-perf-02-boot-verification.json)已核验。CPU 160 MHz、ADC `gpio_tail_required`/SPI 2 MHz、CPU owner/runtime/trace ON 保持；正常默认诊断 OFF，项目优化默认改为 PERF `-O2`。所有 [67 个 host 入口和实际构建](artifacts/cdc_recovery_acceptance_20261116/compiler-perf-02-validated-identity.json)通过，185 个生产输入在 build/test 前后逐 SHA 一致。

The flashed commit is `a656d53f2efc447507f5247a116a22aed35dcc04`, BUILD `a656d53`. The app is 1,591,888 bytes, binary SHA256 `dc6c74800d88760a8ba85274b6954317f6a95d019bf2d4883bea7cdf406a3dd6`, full ELF SHA256 `63ab97c31a1c8c7ca92e631661e42ad7e6357e0475a38f2cc5270959973c36f4`. Session `B528ED81F93297CF`, bootId 1 and [full BUILD/BOOT/READY](artifacts/cdc_recovery_acceptance_20261116/compiler-perf-02-boot-verification.json) are verified. CPU remains 160 MHz, ADC uses `gpio_tail_required` at SPI 2 MHz, and CPU owner/runtime/trace remain ON. Normal diagnostic defaults stay OFF; the project's compiler default is now PERF `-O2`. All [67 host entries and the target build](artifacts/cdc_recovery_acceptance_20261116/compiler-perf-02-validated-identity.json) passed, with identical SHA hashes for 185 production inputs before/after build and tests.

### 未烧录的拒绝候选与 wrapper 修正 / Rejected unflashed candidate and wrapper correction

`compiler-perf-01` 虽能构建，但 ESP-IDF 5.5.5 的 `FREERTOS_TASK_FUNCTION_WRAPPER` 依赖 DEBUG 或 core-dump/GDB；直接切换 PERF 会隐式令其消失。该 wrapper 在任务函数错误返回时打印任务名并 abort，初始任务栈 PC 也指向它；移除它违反保留安全检查的要求。因此该候选 [REJECTED_DO_NOT_FLASH](artifacts/cdc_recovery_acceptance_20261116/compiler-perf-01-scope-rejected.json)，**没有执行烧录**，其构建/测试与配置失败证据不改写。

Although compiler-perf-01 built, ESP-IDF 5.5.5 makes `FREERTOS_TASK_FUNCTION_WRAPPER` depend on DEBUG or core-dump/GDB. Selecting PERF implicitly removes it. The wrapper logs the task name and aborts if a task function mistakenly returns; initial task-stack PC points to it. Removing it violates the requirement to retain safety checks. That candidate was therefore [REJECTED_DO_NOT_FLASH](artifacts/cdc_recovery_acceptance_20261116/compiler-perf-01-scope-rejected.json), **never flashed**, with its build/test/configuration-rejection evidence preserved.

最终实现是在项目 `main/kconfigs/Kconfig.compat` 对同一 SDK symbol 添加有条件定义，仅增加 `IDF_TARGET_ESP32S3 && COMPILER_OPTIMIZATION_PERF` 的依赖分支及 default y；实际 Kconfig parser 按 OR 合并多定义依赖。没有修改 SDK 文件、用应用 `#define` 假装开启 SDK 分支、替换 abort、开启 GDB/core dump 或关闭 watchdog。main 的新增编译 guard 在 S3+PERF 且 wrapper 缺失时直接拒绝构建。其他目标/优化模式保持原 SDK 条件。

The final implementation adds a conditional definition of the same SDK symbol in project `main/kconfigs/Kconfig.compat`, extending only the `IDF_TARGET_ESP32S3 && COMPILER_OPTIMIZATION_PERF` dependency branch with default y. The actual Kconfig parser OR-combines dependencies from multiple definitions. There is no SDK edit, application-level macro pretending to enable SDK code, replacement abort, enabled GDB/core dump or disabled watchdog. A new main compile guard rejects S3+PERF if the wrapper is absent. Other targets/optimization modes keep the original SDK conditions.

[独立真实 parser 回归](artifacts/cdc_recovery_acceptance_20261116/performance-wrapper-independent-review-01.json)覆盖 18 个配置边界、6 个 guard 编译用例；3,769 个全树有效 symbol 中扩展只使 wrapper n→y。生成配置与 [范围对照](artifacts/cdc_recovery_acceptance_20261116/compiler-perf-02-scope.json)确认 assertions、strong stack checking、light heap poisoning、task watchdog、end-of-stack watchpoint 保留，CPU/科学门槛/协议/队列/WorkBudget 未变。多定义提示保留为受测的 Kconfig 扩展，不静默忽略任意同名冲突。

The [independent real-parser regression](artifacts/cdc_recovery_acceptance_20261116/performance-wrapper-independent-review-01.json) covers 18 configuration boundaries and six compile-guard cases. Among 3,769 effective full-tree symbols, the extension changes only the wrapper from n to y. Generated configuration and the [scope comparison](artifacts/cdc_recovery_acceptance_20261116/compiler-perf-02-scope.json) retain assertions, strong stack checking, light heap poisoning, task watchdog and end-of-stack watchpoint, without changing CPU frequency, scientific criteria, protocol, queues or WorkBudget. The multiple-definition notice is a tested Kconfig extension, not a blanket dismissal of arbitrary naming conflicts.

最终 ELF 的 [反汇编核验](artifacts/cdc_recovery_acceptance_20261116/compiler-perf-02-wrapper-elf-review.json)看到 `vPortTaskWrapper` 的 task call，返回后的 `pcTaskGetName → log → abort`，以及 `pxPortInitialiseStack` 将真实 wrapper 地址写入初始 frame PC。这是最终链接代码证据，而不只是头文件标志。**实机任务返回故障注入 NOT_RUN**；保留 wrapper 不等于已验证所有 panic/restart 情形。将来 SDK 若改变同一 flag 的实现，编译 guard 不足以发现，必须重新检查最终 ELF。

The final ELF's [disassembly review](artifacts/cdc_recovery_acceptance_20261116/compiler-perf-02-wrapper-elf-review.json) shows the task call in `vPortTaskWrapper`, the return path through `pcTaskGetName → log → abort`, and `pxPortInitialiseStack` writing the actual wrapper address into initial frame PC. This is final linked-code evidence, not just a header flag. **Hardware task-return fault injection is NOT_RUN**; retaining the wrapper does not validate every panic/restart scenario. If a future SDK changes what the flag enables, the compile guard alone is insufficient and the final ELF must be rechecked.

### 完整 screen10 结果 / Complete screen10 results

VOLT1 正式 CDC0+BLE、独立 CDC1 诊断采集 **120.000122194 s**，同 session/rev4；结束后 `CAP8_QUERY_VERIFIED`。screen10 已关闭全部 observer 后分析，不从活动 raw 推断最终结果。 / VOLT1 formal CDC0/BLE and independent CDC1 diagnostics ran for **120.000122194 s**, in one session/revision 4, ending with `CAP8_QUERY_VERIFIED`. Analysis used closed observers, not provisional active streams.

| 指标 / Metric | screen09 `-Og` | screen10 `-O2` |
|---|---:|---:|
| Capture / USB / BLE FPS | 57.787813 / 57.664438 / 39.133976 | 65.594513 / 65.616123 / 44.162295 |
| USB trusted frames / internal missing observations | 6,920 / 0 | 7,873 / 0 |
| BLE trusted frames / missing observations | 4,695 / 2,223 | 5,298 / 2,572 |
| USB/BLE ASCII CRC / identity / wire-contract errors | all 0 | all 0 |
| 共同帧原字节一致 / Common original frames equal | 4,691 / 4,691 | 5,295 / 5,295 |
| ADC rejected / checked reads | 0 / 74,721 | 0 / 84,943 |
| Complete ADSHEALTH receipt span | 118.765452 s | 119.069615 s |
| USB S1D2 X12 / BLE X12 | 467 / 340 | 740 / 465 |
| USB/BLE not-fresh or unacquired points | 0 | 0 |
| Complete STK50 free / lifetime minimum | 8,295,596 / 8,292,112 B | 8,301,316 / 8,297,776 B |
| Minimum remaining log / transport / BLE_TX stack | 13,064 / 3,184 / 1,932 B | 12,504 / 3,280 / 2,108 B |
| Minimum remaining BLE_CTRL / SERIAL_CTRL stack | 5,236 / 1,952 B | 5,204 / 1,792 B |
| 总体 / Overall | FAIL_BLE_THROUGHPUT | **FAIL_BLE_THROUGHPUT** |

USB verdict 已核单 identity、ordered trusted=7,873、first1331/last9203、progress7872、gap_examples=[]，故首末可信范围内无缺口；不是因为 `missing_frames` 键缺失就填 0。Capture 使用设备时间/sequence progression，USB 使用主机首末可信 K 收到时间，分母不同可有小幅高低差，不表示帧数超发。BLE 5,298 个 DATA/315 个 LOG 消息的收到分片无 CRC/缺片/乱序错误，仍不能消除上游整帧丢弃。ADC 分母 43,277 even +41,666 odd +0 rejected，含 retry attempts，不是帧数；未受保护的末尾 crcRetries 字段仍 unknown。

The USB verdict verifies one identity, 7,873 ordered trusted frames, first1331/last9203, progress7872 and gap_examples=[], establishing no gap inside the trusted endpoint range. It does not default an omitted `missing_frames` key to zero. Capture uses device time/sequence progression; USB uses host receipt times of the first/last trusted K. Different denominators permit a small rate difference without excess frames. Received fragments for 5,298 BLE DATA and 315 LOG messages have no checksum/missing/order error, while upstream whole-frame loss remains. The ADC denominator is 43,277 valid-even +41,666 valid-odd +0 rejected checked reads, including retries, not frames; the unprotected final crcRetries token stays unknown.

浮空 S1D2 的 X12 全部保留无效，不用平滑或零填补取得通过。正式/独立 debug 窗口未观测到 MFAULT/reset/WDT；这一否定结论受观测覆盖限制，不声称未记录事件绝不存在。最大连续 heap、全核 CPU 和绝对精度仍 unknown/未评估；当前镜像 600 秒/32 配置未运行。

All X12 readings at floating S1D2 remain invalid; no smoothing or zero fill is used to obtain a pass. No MFAULT/reset/WDT was observed in the formal/independent-debug window, subject to recording coverage, not proof that no unrecorded event existed. Maximum contiguous heap, whole-core CPU and absolute accuracy remain unknown/not evaluated; this image has not run the 600-second/32-configuration suite.

### 各层与 CPU 独立窗口 / Independent layer and CPU windows

| Layer | screen09 within-window delta | screen10 within-window delta |
|---|---|---|
| USBQ receipt span | 120.402249 s | 120.404974 s |
| USBQ `dSubmit/cSubmit` | 6,943 / 111 | 7,898 / 111 |
| USBQ `dDrop/cDrop/expired/cExpired/partial` | all 0 | all 0 |
| USBQ `dBytes/cBytes` | 4,155,302 / 26,184 B | 4,724,829 / 26,189 B |
| BL50 receipt span | 119.641568 s | 119.677834 s |
| BL50 `sentD/dropD/dropL/dropC` | 4,683 / 2,201 / 0 / 0 | 5,284 / 2,533 / 0 / 0 |
| BLETX query span | 120.401641 s | 120.387778 s |
| BLETX `sdkCalls/sdkAccepted/sdkErrors` | 15,379 / 15,378 / 0 | 17,361 / 17,361 / 0 |
| BLESVC completed / wall delta / mean wall | 15,383 / 62.979474 s / 4.094096 ms | 17,360 / 66.127493 s / 3.809187 ms |
| BLESVC `confOk/confError`; before/after active | 15,382 / 0; 1 / 1 | 17,360 / 0; 0 / 0 |

| Role | screen10 ΔrunUs / own spanUs | screen09 ratio | screen10 ratio |
|---|---:|---:|---:|
| BLE_TX | 19,719,285 / 120,995,504 | 17.650671% | 16.297535% |
| TRANSPORT | 6,430,590 / 120,777,834 | 7.441962% | 5.324313% |
| ASYNC_LOG | 17,839,172 / 120,133,160 | 13.494264% | 14.849499% |
| SERIAL_TX | 12,055,945 / 119,550,795 | 10.233474% | 10.084370% |
| BLE_CTRL | unknown: stale historical | unknown | unknown |
| SERIAL_CTRL | 961,475 / 120,400,085 | 0.789152% | 0.798567% |
| WIFI_CTRL | unknown: not_observed | unknown | unknown |

BLE 本次比 screen09 增加 **12.848988%**，采集亦增加，pool 丢弃仍有增量，故结果仍 FAIL_BLE_THROUGHPUT。ASYNC_LOG 比率虽升到 14.85%，本轮处理帧数更多，不能直接说 `-O2` 使该任务变慢；各角色独立 switch-out 分母也不能合计。USB/SDK 本地接受不等于 PC 实收，query/BL50/科学窗口不同不得相加成分层损失。SDK wall−owner run 仍不是 blocked，未观测 SDK/controller/IDLE 的残差不能归因。该一次有限窗口的收益不替代长期或完整模式验收。

BLE improves **12.848988%** over screen09, but capture also increases and pool drops continue, so the verdict remains FAIL_BLE_THROUGHPUT. ASYNC_LOG's ratio rises to 14.85% while handling more frames; this alone does not mean `-O2` slowed that task. Independent per-role switch-out denominators also cannot be summed. USB/SDK local acceptance is not PC receipt, and query/BL50/scientific windows must not be added into disjoint layer losses. SDK wall−owner run is still not blocked time; unobserved SDK/controller/idle residuals cannot be attributed. One bounded window's gain is not long-duration or complete-mode acceptance.

证据 / Evidence: [screen10 summary](artifacts/cdc_recovery_acceptance_20261116/screen-10/screen-10-volt1.summary.json), [科学与诊断分析](artifacts/cdc_recovery_acceptance_20261116/screen-10/screen-10-analysis-01.json), [CPU delta](artifacts/cdc_recovery_acceptance_20261116/screen-10/screen-10-cpu-owner-deltas-01.json), [verdict](artifacts/cdc_recovery_acceptance_20261116/screen-10/screen-10-verdict.json), [09→10 对照](artifacts/cdc_recovery_acceptance_20261116/screen-10/screen-09-10-layer-cpu-comparison-01.json)。[独立文档核验](artifacts/cdc_recovery_acceptance_20261116/compiler-perf-screen10-doc-review-01.json)重新确认所有 verdict/comparison 输入 SHA 与 67 测试/构建 185 输入，无活动流推断。

### 原始证据归档 / Original evidence archive

本轮对 compiler-perf 的已关闭 BOOT/状态/ROM/debug 流及 screen10 正式 DATA+CTRL、独立 debug、BLE notification 原始流共 **13 个文件**执行确定性 gzip（mtime=0）。原始共 **21,033,025 B**，gzip 共 **2,490,426 B**；每个 gzip 解压后逐字节相等、SHA256 相等，归档前后原文件 SHA 不变，**全部原始文件保留**。没有覆盖任何旧索引或失败记录。索引同时关联未烧录拒绝候选、最终 scope/ELF/build/test/boot 与各分析文件的散列。

This round archives **13 closed files**: compiler-perf BOOT/state/ROM/debug recordings and screen10 formal DATA+CTRL, independent debug and BLE-notification originals. Deterministic gzip uses mtime=0. Original size is **21,033,025 bytes**, gzip size **2,490,426 bytes**. Every decompressed file matches the original byte-for-byte and by SHA256; original hashes remain unchanged, and **all original files are retained**. No old index or failure is overwritten. The index also links hashes for the unflashed rejected candidate, final scope/ELF/build/test/boot and analysis files.

[压缩与回验索引](artifacts/cdc_recovery_acceptance_20261116/screen-10/archive-index-01.json)、[实际 argv/exit0](artifacts/cdc_recovery_acceptance_20261116/compiler-perf-screen10-archive-01-command.json)、[归档 stdout](artifacts/cdc_recovery_acceptance_20261116/compiler-perf-screen10-archive-01.txt)。归档工具仅处理显式选择的已关闭文件，不打开硬件、不删除/覆盖原件；本轮文档/归档子任务未修改生产、采集工具或测试，也未执行 build/flash/HIL。 / The archive tool processes only explicitly selected closed files, opens no hardware and deletes/overwrites no originals. This documentation/archive subtask changed no production, capture tool or test and performed no build/flash/HIL.

## stability-01：同镜像连续 600 秒 / stability-01: continuous 600 seconds on the same image

本次为已烧录 `a656d53` 的一次独立、连续 VOLT1 正式 CDC0+BLE 采集，另有独立 CDC1 诊断 observer。单调时间 `329986.129214936→330586.129290687`，实际 **600.000075751 秒**；不是把 screen04…10 累加，也不是后续 CPU/CRC32/planner 优化或显式 SCLK 低电平 hold 补丁的结果。仍为 160 MHz、`-O2`、CPU owner/runtime/trace ON、ADC GPIO-tail-required/2 MHz。完整 BUILD 的 ELF SHA 为 `63ab97c31a1c8c7ca92e631661e42ad7e6357e0475a38f2cc5270959973c36f4`，session=`B528ED81F93297CF`、bootId=1。采集配置 rev9（mgen6/mrid9、rgen4/rrid8、pgen6/prid9），不与先前 screen10 的 rev4 混用。

This is one independent, continuous VOLT1 formal-CDC0/BLE recording on the already flashed `a656d53`, with a separate CDC1 diagnostic observer. Monotonic time runs from `329986.129214936` to `330586.129290687`, totaling **600.000075751 seconds**. It is not a sum of screen04…10 and does not include subsequent CPU/CRC32/planner optimizations or an explicit SCLK-low-hold patch. The image remains at 160 MHz, `-O2`, CPU owner/runtime/trace ON, GPIO-tail-required ADC reading at 2 MHz. Full BUILD ELF SHA is `63ab97c31a1c8c7ca92e631661e42ad7e6357e0475a38f2cc5270959973c36f4`, session `B528ED81F93297CF`, bootId 1. Capture revision is 9 (mgen6/mrid9, rgen4/rrid8, pgen6/prid9), distinct from screen10 revision 4.

已关闭三路采集并核对原始输入 SHA 后才生成 [verdict](artifacts/cdc_recovery_acceptance_20261116/stability-01/stability-01-verdict.json)。`continuous_600_seconds.status=CAPTURED_600_SECONDS_NOT_ACCEPTED` 明确只针对本次单窗口；总体仍 **FAIL_BLE_THROUGHPUT**。离线汇总器现在检查有限 start/end/duration、requested≥600、实际满足请求、状态 CAPTURED 和同一窗口，而不是固定写 NOT_RUN；[7 个真实执行的离线回归](artifacts/cdc_recovery_acceptance_20261116/stability-closure-tests-01-command.json)通过，含不足时长/非有限或缺失时间/错误状态等反例。这 7 个用例不改变已验证固件的 67 个 host 入口计数，也不是新的固件构建或 HIL。

The three observers were closed and original input hashes checked before generating the [verdict](artifacts/cdc_recovery_acceptance_20261116/stability-01/stability-01-verdict.json). `continuous_600_seconds.status=CAPTURED_600_SECONDS_NOT_ACCEPTED` applies only to this single recording; overall status remains **FAIL_BLE_THROUGHPUT**. The offline summarizer now checks finite start/end/duration, a request of at least 600 seconds, the requested duration actually met, CAPTURED status and one consistent window instead of hard-coding NOT_RUN. All [seven executed offline regression cases](artifacts/cdc_recovery_acceptance_20261116/stability-closure-tests-01-command.json) passed, including short/nonfinite/missing-time and error-state counterexamples. They do not change the image's 67-entry host count and are not a new firmware build or hardware run.

### 科学帧与健康边界 / Scientific frames and health boundaries

| 指标 / Metric | 单次连续结果 / Single continuous result |
|---|---:|
| Duration | 600.000075751 s |
| Capture / USB / BLE FPS | 65.543247 / 65.549657 / 44.207974 |
| USB trusted frames / internal missing observations | 39,328 / 0 |
| BLE trusted frames / missing observations | 26,523 / 12,805 |
| USB/BLE ASCII CRC / identity / wire-contract errors | all 0 |
| Common original frames equal | 26,520 / 26,520 |
| ADC rejected / checked reads | 0 / 428,131 |
| Complete ADSHEALTH receipt span | 599.484129 s |
| USB S1D2 X12 / BLE X12 invalid points | 3,640 / 2,291 |
| USB/BLE not-fresh or unacquired points | 0 |
| Complete STK50 free / lifetime minimum | 8,301,048 / 8,297,776 B |
| Minimum remaining log / transport / BLE_TX stack | 12,504 / 3,280 / 2,108 B |
| Minimum remaining BLE_CTRL / SERIAL_CTRL stack | 5,204 / 1,712 B |
| Cleanup / overall verdict | CAP8_QUERY_VERIFIED / **FAIL_BLE_THROUGHPUT** |

USB 的连续性证据为：同一 identity、ordered trusted=39328、first13901/last53228、progress39327、gap_examples=[]；不是将缺少的 `missing_frames` 键当 0。BLE 收到的 26,523 DATA 和 1,574 LOG 组装均没有分片 CRC/缺片/重复/乱序错误，仍有上游完整 DATA 丢弃。共同帧原 wire bytes 全相同；USB-only/BLE-only 的边界差异不直接解释为另一侧丢失。

USB contiguity is established by one identity, ordered trusted=39328, first13901/last53228, progress39327 and gap_examples=[]; an absent `missing_frames` key is not defaulted to zero. Received BLE assemblies—26,523 DATA and 1,574 LOG—have no fragment CRC/missing/duplicate/order errors, while upstream whole-DATA drops remain. Every common frame has identical original wire bytes. USB-only/BLE-only boundary differences are not automatically classified as loss on the other side.

| ADSHEALTH 边界 / Boundary | 首条完整记录 / First complete record | 末条完整记录 / Last complete record |
|---|---:|---:|
| Host receipt monotonic (s) | 329986.571616645 | 330586.055745967 |
| seq / engineFrameCount | 13929 / 8640 | 53225 / 47936 |
| modeEntryFrameCount | 8259 | 8259 |
| healthInterval / healthReadbackDue / precisionFrame | 64 / 1 / 1 | 64 / 1 / 1 |
| err | 0 | 0 |
| crcValidEven / crcValidOdd / crcErrors | 49579 / 47800 / 0 | 267038 / 258472 / 0 |

分母为 **217459 even +210672 odd +0 rejected=428131** 个受校验读数，包含重试尝试，不是帧数。健康区间跨越已有累计计数的 mode，不能假定 mode 切换将 engineFrameCount 清零，也不能把健康日志首末当作科学窗口的精确端点。末尾无完整性保护的 crcRetries token 仍 unknown；没有 ADSSPI 错误行不等于重试计数已证明为零。正式和独立 debug 观察范围内未观测 MFAULT/reset/WDT；缺少日志不证明任意未记录事件都不存在。

The denominator is **217459 valid-even +210672 valid-odd +0 rejected=428131** checked reads, including retry attempts rather than frames. The health span crosses a mode with prior accumulated counts; it neither assumes mode changes zero engineFrameCount nor treats health endpoints as exact scientific-window boundaries. The final crcRetries token lacks an integrity delimiter and remains unknown; no ADSSPI error line does not establish a zero retry count. No MFAULT/reset/WDT was observed in the formal and independent-debug coverage, which does not prove the absence of any unrecorded event.

所有浮空 S1D2 的 X12 保留无效，没有平滑或补零。最大连续 heap、全核 CPU 和绝对精度仍 unknown/未评估；当前镜像 32 配置×120 秒仍 NOT_RUN。内存最小值来自完整 STK50，不能混用启动 BOOT 的历史 heap 快照。

Every floating-S1D2 X12 remains invalid without smoothing or zero filling. Maximum contiguous heap, whole-core CPU and absolute accuracy remain unknown/not evaluated; the current image's 32 configurations×120 seconds remain NOT_RUN. Memory minima come from complete STK50 records, not a BOOT record's separately scoped historical heap snapshot.

### 各层与 CPU 独立窗口 / Independent layer and CPU windows

| Layer | 本层观察窗口 / Own observation span | 本层增量 / Own delta |
|---|---:|---|
| USB | 600.426124 s | resets/mounts/unmounts/rxDrop/rxOver: 0; rxBytes: 4,219 |
| USBQ | 600.432809 s | dSubmit/cSubmit: 39,355 / 487; dDrop/cDrop/expired/cExpired/partial: all 0 |
| USBQ accepted bytes | same USBQ span | dBytes/cBytes: 23,678,340 / 112,608 B; per-lane callback bytes unknown |
| BL50 | 599.542572 s | sentD/dropD/dropL/dropC: 26,502 / 12,583 / 0 / 0; ctrlRetry/ctrlExhaust: 0 |
| BLETX | 600.447359 s | sdkCalls/accepted/errors: 86,714 / 86,715 / 0 |
| BLESVC | 600.434275 s | started/completed: 86,712 / 86,713; wall: 329.772301 s; mean 3.803032 ms/completed |
| BLESVC confirmations | same BLESVC span | confOk/confError: 86,712 / 0; before/after active: 1 / 0 |

USBQ 本地 FIFO 接受、BLE SDK 完成和主机 CRC 完整帧不是相同事件。BLETX/BLESVC 的一次边界在途调用说明 completed/accepted 可比 calls/started 多 1，不表示收到额外科学帧。BL50 是周期日志首末 receipt 窗口，不是主动查询的科学边界；dropD+12583 不能与主机 missing12805 相加为互斥损失。主机同期有开发工作，BLE timestamp 是 host callback/接收时间，不是空中抓包或隔离主机负载实验。

USBQ local FIFO acceptance, BLE SDK completion and host CRC-valid complete-frame receipt are different events. One call in flight at the BLETX/BLESVC boundary explains why completed/accepted can exceed calls/started by one; it does not create an extra received scientific frame. BL50 uses first/last periodic-log receipt times, not queried scientific boundaries; dropD+12583 cannot be added to host missing12805 as disjoint losses. Development work ran concurrently on the host, and BLE timestamps are host callback/receipt times rather than over-air captures or an isolated-host-load experiment.

| Role | stability-01 ΔrunUs / own spanUs | screen10 ratio | stability-01 ratio |
|---|---:|---:|---:|
| BLE_TX | 98,076,373 / 600,267,916 | 16.297535% | 16.338766% |
| TRANSPORT | 31,886,240 / 600,159,072 | 5.324313% | 5.312965% |
| ASYNC_LOG | 89,312,732 / 600,411,082 | 14.849499% | 14.875264% |
| SERIAL_TX | 59,904,941 / 599,954,851 | 10.084370% | 9.984908% |
| BLE_CTRL | unknown: stale historical | unknown | unknown |
| SERIAL_CTRL | 4,725,750 / 600,441,021 | 0.798567% | 0.787046% |
| WIFI_CTRL | unknown: not_observed | unknown | unknown |

各角色仅比较同 session/task 的新鲜 switch-out 累计值；角色分母独立，不合计为全核占用，不用 SDK wall−owner runtime 推 blocked。`runUs` 可能含 ISR，SDK/controller/IDLE 没有独立测量。screen10 与本次同镜像但不同时长/配置 revision，以上是描述性对照，不是新优化的因果 A/B。

Each role pairs fresh switch-out accumulators only within the same session/task. Independent denominators are not summed into whole-core utilization, and SDK wall minus owner runtime does not determine blocked time. `runUs` may include ISR execution; SDK/controller/idle work is not separately measured. Screen10 and this run use the same image but different duration/configuration revisions, so the table is descriptive rather than a causal A/B test of a new optimization.

### 关闭本任务 reader 后的只读观察 / Read-only observation after this task's readers closed

三路 observer 关闭后，`post-stability-unread-01` 在 **3.057783 秒**内完成只读查询；完整 BUILD/session 不变，CONFIG rev11、CAP8/CCCCCCCC，BOOT seq55208/ready1，USB mounted1/epoch1。USBQ 此时的 **累计** expired=10258、partial=7、cDrop=0、cExpired=0 是后续时点证据，不能覆盖本次 600 秒 USBQ 窗内 expired/partial 增量均 0 的结果，也不直接归因成相同时间段的科学丢帧。

After all three observers closed, `post-stability-unread-01` completed read-only queries in **3.057783 seconds**. Full BUILD/session are unchanged; CONFIG is revision11, CAP8/CCCCCCCC, BOOT seq55208/ready1 and USB mounted1/epoch1. USBQ's **cumulative** expired=10258, partial=7, cDrop=0 and cExpired=0 describe a later observation. They neither replace the zero expired/partial deltas inside the 600-second USBQ window nor directly quantify scientific loss over that same interval.

USB 物理仍连接，本任务停止读取不等于拔线、unmounted 或系统没有任何 reader；其他 reader 未核验。该工具未发 USBBOOT/reset、未校准或改配置。它证明此后控制仍可查询，不证明所有无人读/断线恢复情形；没有独立 post debug，不能从此次查询补造 ADC/WDT 证据。见 [summary](artifacts/cdc_recovery_acceptance_20261116/post-stability-unread-01.summary.json) 与 [命令](artifacts/cdc_recovery_acceptance_20261116/post-stability-unread-01-command.json)。

USB remained physically connected. Stopping this task's readers is neither unplugging, unmounting nor proof that no other system reader existed; other readers were unverified. The tool issued no USBBOOT/reset and changed no calibration or configuration. It demonstrates subsequent control queryability, not every unread/disconnect-recovery scenario. Without a separate post-capture debug observer, the query cannot supply new ADC/WDT evidence. See the [summary](artifacts/cdc_recovery_acceptance_20261116/post-stability-unread-01.summary.json) and [command](artifacts/cdc_recovery_acceptance_20261116/post-stability-unread-01-command.json).

### 归档与复核 / Archive and verification

5 个已关闭稳定性原始流共 **97,229,846 B→11,121,002 B gzip**；后续只读 raw/events 两文件另为 **84,591 B→16,636 B**。每个文件均解压逐字节和 SHA256 回验，归档前后原件 SHA 不变，全部原件保留。独立索引不覆盖 screen10 或历史失败；该文档/归档步骤未执行生产修改、构建、烧录或硬件操作。

Five closed stability originals total **97,229,846 bytes→11,121,002 gzip bytes**; the later read-only raw/events separately total **84,591→16,636 bytes**. Every decompressed file is checked byte-for-byte and by SHA256, originals retain their pre-archive hashes, and all originals remain. Separate indexes preserve screen10 and historical failures. This documentation/archive step performs no production edit, build, flash or hardware action.

证据 / Evidence: [原 capture summary](artifacts/cdc_recovery_acceptance_20261116/stability-01/stability-01-volt1-stability-volt1.summary.json), [完整科学/诊断分析](artifacts/cdc_recovery_acceptance_20261116/stability-01/stability-01-analysis-01.json), [CPU delta](artifacts/cdc_recovery_acceptance_20261116/stability-01/stability-01-cpu-owner-deltas-01.json), [screen10→stability 对照](artifacts/cdc_recovery_acceptance_20261116/stability-01/screen10-stability01-layer-cpu-comparison.json), [稳定性归档索引](artifacts/cdc_recovery_acceptance_20261116/stability-01/archive-index-01.json), [归档命令/exit0](artifacts/cdc_recovery_acceptance_20261116/stability-01-archive-01-command.json), [后续只读归档](artifacts/cdc_recovery_acceptance_20261116/stability-01/post-readers-closed-archive-index-01.json), [后续归档命令/exit0](artifacts/cdc_recovery_acceptance_20261116/post-stability-unread-archive-01-command.json)。

## screen11：CRC32/planner 等价优化与 ADS 低电平建立 / screen11: equivalent CRC32/planner optimization and ADS low hold

当前板上源码为 `caa90829f34ffeda76dfe681e8e20bb29de4627c`，运行 BUILD=`caa9082-dirty`，app **1,592,032 B**，binary SHA256=`65c4d3af1ca56cb2ae60e4972ad40fc4beb27f069c5b878aaa467b3fd85f28d4`，完整 ELF SHA256=`b880b7ea8680ae34b68167d3d75b59b06e12b134afc00ef114e883d7dd8b9fcd`。完整 BUILD/READY 与 session=`5AE787A210773D6D`、bootId=1、CAP8 已[独立核对](artifacts/cdc_recovery_acceptance_20261116/transport-cpu-01-boot-verification.json)。CPU160 MHz、`-O2`、CPU owner/runtime/trace ON、ADS GPIO-tail-required/2 MHz；sdkconfig 自 compiler-perf-02 未变，保留 task wrapper/assert/栈/heap/watchdog 检查，正常 defaults 的 CPU 诊断仍 OFF。

The currently flashed source is `caa90829f34ffeda76dfe681e8e20bb29de4627c`, running BUILD `caa9082-dirty`: app **1,592,032 bytes**, binary SHA256 `65c4d3af1ca56cb2ae60e4972ad40fc4beb27f069c5b878aaa467b3fd85f28d4`, full ELF SHA256 `b880b7ea8680ae34b68167d3d75b59b06e12b134afc00ef114e883d7dd8b9fcd`. Full BUILD/READY, session `5AE787A210773D6D`, bootId 1 and CAP8 were [independently verified](artifacts/cdc_recovery_acceptance_20261116/transport-cpu-01-boot-verification.json). CPU remains 160 MHz, `-O2`, CPU owner/runtime/trace ON, GPIO-tail-required ADS reading at 2 MHz. sdkconfig is unchanged since compiler-perf-02, retaining task-wrapper/assertion/stack/heap/watchdog checks; CPU diagnostics remain OFF in normal defaults.

构建时并发文档修改使版本带 `-dirty`，这不代表使用其他未识别生产补丁，也不重命名实录镜像。首次全套 **67 PASS/1 FAIL**：`test_work_budget` 的旧 BLE 分片函数提取器缺少新增 helper/constants，编译失败；生产 target build 与其余入口均通过。测试专用 `849d96c` 修复提取范围、保留原断言，随后完整 **68/68** 通过。build01 与 test02 的 **185 个生产输入逐 SHA 相同**，不以测试 commit 代替烧录 source。证据：[最终身份](artifacts/cdc_recovery_acceptance_20261116/transport-cpu-01-validated-identity.json)、[首轮失败](artifacts/cdc_recovery_acceptance_20261116/transport-cpu-01-tests-manifest.json)、[最终68入口](artifacts/cdc_recovery_acceptance_20261116/transport-cpu-02-tests-manifest.json)、[实际构建](artifacts/cdc_recovery_acceptance_20261116/transport-cpu-01-build-manifest.json)。

Concurrent documentation edits caused the build's `-dirty` suffix; this does not identify an additional unknown production patch or rename the recorded image. The first complete run had **67 PASS/one FAIL**: the old `test_work_budget` BLE-fragment extractor omitted new helper/constants and failed compilation, while the production target build and other entries passed. Test-only `849d96c` corrects extraction while retaining assertions, followed by a complete **68/68** passing run. All **185 production-input SHA hashes match** between build01/test02; the test commit is not substituted for the flashed source. Evidence: [final identity](artifacts/cdc_recovery_acceptance_20261116/transport-cpu-01-validated-identity.json), [preserved first failure](artifacts/cdc_recovery_acceptance_20261116/transport-cpu-01-tests-manifest.json), [final 68 entries](artifacts/cdc_recovery_acceptance_20261116/transport-cpu-02-tests-manifest.json), [actual build](artifacts/cdc_recovery_acceptance_20261116/transport-cpu-01-build-manifest.json).

### 两个小补丁与保留的检查 / Two small patches and retained checks

`4edc99a` 只优化 `sensorarrayTextCrc32`、`sensorarrayBleCrc32` 和分片容量试算：S3 用 `esp_rom_crc32_le` 从 seed0 获得与原逐位 IEEE CRC32 相同的结果，累计调用不额外取反。host 保留独立逐位参考；科学 ASCII CRC、encoder self-check、BLE 入队前 CRC、出队复算和损坏拒绝均在，ADC SPI CRC8 不变。容量试算改为固定字符数加五个 uint32 十进制位数，真实发送仍用原 `snprintf`；分片搜索/base/remainder/tinyTail、2304 B 公共上限、owner/队列/截止时间/WorkBudget/MTU/PHY/连接请求不变。只对不可能经过公共入队的超 uint32 私有输入显式拒绝，不静默截断。

`4edc99a` optimizes only `sensorarrayTextCrc32`, `sensorarrayBleCrc32` and trial fragment capacity. On S3, `esp_rom_crc32_le` from seed zero gives the same IEEE CRC32 result as the original bitwise routine, with no extra complement between cumulative calls. The host retains an independent bitwise reference. Scientific ASCII CRC, encoder self-check, BLE CRC before enqueue, dequeue recomputation and corruption rejection all remain; ADC SPI CRC8 is unchanged. Capacity calculation becomes fixed character counts plus five uint32 decimal widths, while the actual transmitted header still uses the original `snprintf`. Fragment search/base/remainder/tinyTail, the public 2304-byte limit, owners/queues/deadlines/WorkBudget/MTU/PHY/connection requests are unchanged. Only private inputs above uint32—which cannot pass public enqueue—are explicitly rejected instead of silently narrowed.

[生产函数差分](artifacts/cdc_recovery_acceptance_20261116/ble-crc-planner-equivalence-01/review.json)实际覆盖 11,525 CRC、18,522 header、7,071 planner、3,087 wire 比较和 15 个真实 encoder golden frames。[独立 SDK 参考审查](artifacts/cdc_recovery_acceptance_20261116/crc-planner-independent-review-01.json)另覆盖 3,587 CRC、38,954 header、22,155 planner、175,816 wire/header 检查；20 个既有输入包含合成 encoder/实录/故意截断，不冒充20个新实机有效帧。ASan/UBSan通过，LSan NOT_RUN；host不能证明 S3 ROM 执行或吞吐。父任务随后核对[最终 ELF](artifacts/cdc_recovery_acceptance_20261116/transport-cpu-01-elf-evidence.json)中 encoder 与 BLE 入队/出队实际 call 到 `esp_rom_crc32_le` 的 `0x40001c98`，保留 task wrapper；objdump 同地址别名不是把 CRC 调到 BLE ISR 的证据。

Executed [production-function differential tests](artifacts/cdc_recovery_acceptance_20261116/ble-crc-planner-equivalence-01/review.json) cover 11,525 CRC, 18,522 header, 7,071 planner and 3,087 wire comparisons plus 15 actual-encoder golden frames. The [independent SDK-reference review](artifacts/cdc_recovery_acceptance_20261116/crc-planner-independent-review-01.json) separately covers 3,587 CRC, 38,954 header, 22,155 planner and 175,816 wire/header checks. Its 20 existing inputs include synthetic encoder fixtures, recordings and intentional truncations, not 20 newly acquired valid hardware frames. ASan/UBSan pass; LSan is NOT_RUN. Host execution does not prove S3 ROM execution or throughput. The parent then checked the [final ELF](artifacts/cdc_recovery_acceptance_20261116/transport-cpu-01-elf-evidence.json): encoder and BLE enqueue/dequeue call sites reach `esp_rom_crc32_le` at `0x40001c98`, with the task wrapper retained. An objdump alias at that same address is not evidence of calling a BLE ISR instead of CRC.

`caa9082` 补 ADS SPI55→GPIO1 接管前的低电平建立：精确 DMA descriptor 回收后先确认 SPI SCLK 低，接到 GPIO 后再读低，等待 `ceil(500/actual_kHz)` µs，然后既有高电平/真实采样/低电平/路由恢复。2 MHz、754/500 kHz为1 µs，250 kHz为2 µs，150/125 kHz为4 µs。频率读回失败只以1 µs完成已开始 framing，仍返回原 clock 错误而非科学样本。接管后读低失败不升沿/不采样，保留 CLOCK_LOW 首阶段并隔离；DMA完成未知不接管、不释放或复用仍在driver的缓冲。

`caa9082` adds low-level establishment before the ADS SPI55→GPIO1 handoff edge. After exact DMA-descriptor reclamation, it checks SPI SCLK low, routes to GPIO, checks low again, waits `ceil(500/actual_kHz)` µs, then performs the existing high edge/real sample/low edge/route restoration. The wait is 1 µs at 2 MHz and 754/500 kHz, 2 µs at 250 kHz, and 4 µs at 150/125 kHz. If frequency readback fails, a 1 µs fallback completes already-started framing, then returns the original clock error rather than a scientific sample. A post-handoff low mismatch generates no rise/sample, preserves the first CLOCK_LOW stage and isolates the driver. Unknown DMA completion never permits handoff, release or reuse of buffers still owned by the driver.

真实旧源码反例在零附带耗时的合成硬件模型中失败于 `lowRead < rise`，修后六频点顺序与失败分支、全部256 CRC-byte物理位组合通过；独立160,022个算术/UBSan检查及组合失败也通过。它证明缺少的**源码顺序/最短等待保证**已补齐，不能反推旧波形违反40 ns、`-O2`造成CRC错误或新接管物理无毛刺。已知迟到descriptor仍须完成已开始的单边沿对并恢复再返回TIMEOUT，收尾可能超过工作deadline；没有虚构DMA取消。CRC全位、STATUS/freshness/cache/invalid路径保持。见[修复证据](artifacts/cdc_recovery_acceptance_20261116/adc-tail-low-hold-fix-01.json)及[独立审查](artifacts/cdc_recovery_acceptance_20261116/adc-tail-low-hold-independent-review-01.json)。

The actual old source fails `lowRead < rise` in a synthetic hardware model with zero incidental call time. The repair passes six-frequency ordering/failure cases and all 256 CRC-byte physical-bit patterns; independent 160,022 arithmetic/UBSan cases and combined failures also pass. This establishes the missing **source-level ordering/minimum-wait guarantee**, not that an old waveform violated 40 ns, `-O2` caused CRC errors, or the new physical handoff is glitch-free. A known late descriptor still requires completing the already-started edge pair and restoring routing before TIMEOUT; cleanup may extend beyond the work deadline, with no fictitious DMA cancellation. Full CRC, STATUS/freshness/cache/invalid paths remain. See the [repair evidence](artifacts/cdc_recovery_acceptance_20261116/adc-tail-low-hold-fix-01.json) and [independent review](artifacts/cdc_recovery_acceptance_20261116/adc-tail-low-hold-independent-review-01.json).

### screen11 科学结果 / screen11 scientific results

正式 CDC0+BLE 与独立 CDC1 observer 完整录制 **120.000070722 秒**，同session/rev4；结束后 **CAP8_QUERY_VERIFIED**。所有 raw 关闭后分析。当前 BLELINK 实测 MTU247、TX/RX PHY2、interval6×1.25ms=7.5ms、DLE251；DLE peerKnown=0保留未知。MTU517仍为研究候选，没有在本镜像实施或计入此结果。

Formal CDC0/BLE and a separate CDC1 observer recorded a complete **120.000070722 seconds**, within one session/revision 4, followed by **CAP8_QUERY_VERIFIED**. Analysis uses closed raw files. Actual BLELINK reports MTU 247, TX/RX PHY 2, interval 6×1.25 ms=7.5 ms and DLE 251; DLE peerKnown=0 remains unknown. MTU 517 is still a research candidate, neither implemented in this image nor credited here.

| 指标 / Metric | screen10 `a656d53` | screen11 `caa9082-dirty` |
|---|---:|---:|
| Capture / USB / BLE FPS | 65.594513 / 65.616123 / 44.162295 | 64.376134 / 64.384201 / 47.080922 |
| USB trusted / internal missing observations | 7,873 / 0 | 7,724 / 0 |
| BLE trusted / missing observations | 5,298 / 2,572 | 5,650 / 2,077 |
| Common frames with identical original bytes | 5,295 / 5,295 | 5,648 / 5,648 |
| USB/BLE ASCII CRC / identity / wire-contract errors | all 0 | all 0 |
| ADC rejected / checked reads | 0 / 84,943 | 0 / 82,911 |
| Complete ADSHEALTH receipt span | 119.069615 s | 119.320022 s |
| USB S1D2 X12 / BLE X12 invalid points | 740 / 465 | 670 / 434 |
| USB/BLE not-fresh or unacquired points | 0 | 0 |
| Complete STK50 free / lifetime minimum | 8,301,316 / 8,297,776 B | 8,301,264 / 8,297,776 B |
| Minimum remaining log / transport / BLE_TX stack | 12,504 / 3,280 / 2,108 B | 12,504 / 3,280 / 2,188 B |
| Minimum remaining BLE_CTRL / SERIAL_CTRL stack | 5,204 / 1,792 B | 5,204 / 1,824 B |
| Overall | FAIL_BLE_THROUGHPUT | **FAIL_BLE_THROUGHPUT** |

USB 证据为同一identity、7,724有序可信帧、seq1571…9294、progress7723、gap=[]，不是缺少 `missing_frames` 键就填0。BLE收到5,650 DATA/308 LOG无分片CRC/缺片/乱序错误，仍有完整帧上游丢弃。ADC health首末engine 384→8064、modeEntry0、interval64、err0，delta42323 even+40588 odd=82911；本次modeEntry确为0不改变此前稳定性modeEntry8259的事实。健康首末不是精确科学端点，分母含retry attempts而非帧数，末尾未保护crcRetries仍unknown。X12全保留无效；未观测MFAULT/reset/WDT受录制覆盖限制，不替代绝对精度或无限时运行保证。

USB evidence is one identity, 7,724 ordered trusted frames, seq1571…9294, progress7723 and an empty gap list—not zero inferred from an absent `missing_frames` key. Received BLE assemblies contain 5,650 DATA/308 LOG with no fragment CRC/missing/order errors, while upstream whole-frame drops remain. ADS health endpoints report engine 384→8064, modeEntry 0, interval 64 and err 0; deltas 42323 valid-even+40588 valid-odd=82911. This run's actual modeEntry 0 does not change the previous stability run's modeEntry 8259. Health endpoints are not exact science boundaries, checked reads include retry attempts rather than frames, and the unprotected terminal crcRetries token remains unknown. Every X12 remains invalid. No observed MFAULT/reset/WDT is limited by recording coverage, not an absolute-accuracy or indefinite-run guarantee.

### 独立窗口的吞吐与 CPU / Throughput and CPU in independent windows

| Layer | screen11 own observation span | Within-window delta |
|---|---:|---|
| USB | 120.363549 s | reset/mount/unmount/rxDrop/rxOver:0; rxBytes:976 |
| USBQ | 120.355108 s | dSubmit/cSubmit:7,748/111; dDrop/cDrop/expired/cExpired/partial:all0 |
| USBQ accepted bytes | same USBQ span | dBytes/cBytes:4,635,602/26,198 B; callback lane bytes unknown |
| BL50 | 119.609541 s | sentD/dropD/dropL/dropC:5,629/2,044/0/0; ctrlRetry/ctrlExhaust:0 |
| BLETX | 120.352648 s | sdkCalls/accepted/errors:18,382/18,382/0 |
| BLESVC | 120.353767 s | completed18,383; wall70.100014 s; mean3.813307 ms/completed |
| BLESVC confirmations | same BLESVC span | confOk/confError:18,383/0; active before/after:1/1 |

| Role | screen11 ΔrunUs / own spanUs | screen10 ratio | screen11 ratio |
|---|---:|---:|---:|
| BLE_TX | 18,521,451 / 119,846,726 | 16.297535% | 15.454282% |
| TRANSPORT | 4,335,046 / 119,899,071 | 5.324313% | 3.615579% |
| ASYNC_LOG | 17,650,849 / 120,142,729 | 14.849499% | 14.691567% |
| SERIAL_TX | 12,282,096 / 119,565,363 | 10.084370% | 10.272286% |
| BLE_CTRL | unknown: stale historical | unknown | unknown |
| SERIAL_CTRL | 909,206 / 120,387,871 | 0.798567% | 0.755231% |
| WIFI_CTRL | unknown: not_observed | unknown | unknown |

BLE实收比screen10增加 **6.608866%**，TRANSPORT/BLE_TX自身switch-out比率降低；但64.38FPS采集仍高于47.08FPS BLE，BL50 dropD仍增2044，因此整体继续FAIL_BLE_THROUGHPUT。两个补丁一起进入镜像且两次调度/采样时序不相同，不能单独归因每一毫秒或FPS变化。SDK墙钟均值约3.81ms仍不是SDK CPU或blocked。CPU各角色分母独立，不跨boot相减、不求和成全核CPU；BL50/USBQ/主机缺口窗口不能相加为互斥损失。BLE时间为host callback而非空中测量。

BLE receipt improves **6.608866%** over screen10 while TRANSPORT/BLE_TX own switch-out ratios fall. Capture at 64.38 FPS still exceeds BLE 47.08 FPS, however, and BL50 dropD rises 2,044, so overall status remains FAIL_BLE_THROUGHPUT. Two patches enter the same image and scheduling/sampling timing differs between runs, so each millisecond/FPS change cannot be assigned to one patch. The SDK mean wall time of about 3.81 ms is neither SDK CPU nor blocked time. Per-role CPU denominators remain independent: no cross-boot subtraction or summed whole-core CPU, and no addition of BL50/USBQ/host gap windows as disjoint losses. BLE timing is host callback time rather than over-air measurement.

旧 `a656d53` 独立连续600秒及0/428131已完整保留，但新 `caa9082-dirty` 的600秒/32配置仍 **NOT_RUN**，不得用历史结果补齐。绝对精度、最大连续heap块仍未评估；本次已恢复CAP8。证据：[summary](artifacts/cdc_recovery_acceptance_20261116/screen-11/screen-11-volt1.summary.json)、[analysis](artifacts/cdc_recovery_acceptance_20261116/screen-11/screen-11-analysis-01.json)、[CPU delta](artifacts/cdc_recovery_acceptance_20261116/screen-11/screen-11-cpu-owner-deltas-01.json)、[verdict](artifacts/cdc_recovery_acceptance_20261116/screen-11/screen-11-verdict.json)、[screen10→11对照](artifacts/cdc_recovery_acceptance_20261116/screen-11/screen10-screen11-layer-cpu-comparison.json)。

The previous `a656d53` single continuous 600-second/0-of428131 result remains preserved, but the new `caa9082-dirty` image's 600-second/32-configuration runs are **NOT_RUN**, never filled by history. Absolute accuracy and maximum contiguous heap remain unassessed; this run ends in query-verified CAP8. Evidence: [summary](artifacts/cdc_recovery_acceptance_20261116/screen-11/screen-11-volt1.summary.json), [analysis](artifacts/cdc_recovery_acceptance_20261116/screen-11/screen-11-analysis-01.json), [CPU delta](artifacts/cdc_recovery_acceptance_20261116/screen-11/screen-11-cpu-owner-deltas-01.json), [verdict](artifacts/cdc_recovery_acceptance_20261116/screen-11/screen-11-verdict.json), [screen10→11 comparison](artifacts/cdc_recovery_acceptance_20261116/screen-11/screen10-screen11-layer-cpu-comparison.json).

### 原始流归档 / Original-recording archive

本轮已关闭 boot/state/ROM/debug/screen11的13个raw/events/notification流及3份实际flash identity/app/watchdog stdout，共 **16文件、21,438,993 B→2,515,154 B gzip**。ROM枚举、MAC/app写入与后续完整BUILD核验为不同阶段，未把RACK/RAPP或ROM枚举当烧录成功。每份gzip解压逐字节/SHA匹配，原件不删改；metadata索引保留首次测试FAIL和最终build/test/ELF/boot证据。见[归档索引](artifacts/cdc_recovery_acceptance_20261116/screen-11/archive-index-01.json)、[实际argv/exit0](artifacts/cdc_recovery_acceptance_20261116/transport-cpu-screen11-archive-01-command.json)、[stdout](artifacts/cdc_recovery_acceptance_20261116/transport-cpu-screen11-archive-01.txt)。文档/归档子任务未操作硬件、构建、git或生产代码。

This round archives 13 closed boot/state/ROM/debug/screen11 raw/events/notification streams and three actual flash identity/app/watchdog stdout logs: **16 files, 21,438,993 bytes→2,515,154 gzip bytes**. ROM enumeration, MAC/app write and subsequent full BUILD verification remain distinct stages; RACK/RAPP or ROM enumeration is not flash success. Each gzip decompresses byte-for-byte and SHA-identically, with originals retained unchanged. The metadata index preserves the first test FAIL and final build/test/ELF/boot evidence. See the [index](artifacts/cdc_recovery_acceptance_20261116/screen-11/archive-index-01.json), [actual argv/exit0](artifacts/cdc_recovery_acceptance_20261116/transport-cpu-screen11-archive-01-command.json) and [stdout](artifacts/cdc_recovery_acceptance_20261116/transport-cpu-screen11-archive-01.txt). The documentation/archive subtask performs no hardware, build, git or production-code operation.

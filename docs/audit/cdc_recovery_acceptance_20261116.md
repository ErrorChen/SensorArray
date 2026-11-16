# 2026-11-16 软件恢复与双 CDC 验收 / Software recovery and dual-CDC acceptance

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

## 待完成 / Pending

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

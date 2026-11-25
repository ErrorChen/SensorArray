# 迭代记录 / Development history

本页按阶段汇总固件演进、失败、修复与验收。末尾逐项列出提交，并保留各次审计的原始章节索引。实测数字只适用于所列镜像、板卡、会话与观察窗口。日常使用查阅[项目手册](../../README.md)，复现实验查阅[测试方法](../validation.md)。

This is the chronological record of changes and results. The commit ledger and report indexes retain the original evidence and unsuccessful attempts. Measurements are scoped to their image, board, session and observation interval. The project manual and test-method catalogue describe current operation and reproducible procedures.

[最终 32 工况和 10 分钟结果 / Final matrix and stability](#final-window) · [FW 审计编号 / FW register](#fw-register) · [历次性能对照 / Earlier comparisons](#comparison-tables) · [原始报告 / Original reports](#original-reports) · [提交索引 / Commits](#commit-ledger)

## 演进 / Milestones

| 时期 / Period | 改动与验证 / Change and evidence |
|---|---|
| 2025-12 – 2026-03 | 建立 ESP32-S3 工程、FDC2214/ADS126x、TMUX、板级资源与 wire/USB 实验协议。以 S1D1 和 SEL 路由排查负轨偏移。 / Board bring-up added drivers, board resources and binary protocol experiments. Single-cell measurements isolated routing and negative-rail issues. |
| 2026-04 – 05 | I²C 时钟扫描、S5D5 激励电流与去毛刺组合、10/40 MHz 配置、重锁与有界逐步超时。单点验证扩展为矩阵与双器件逐行 autoscan。 / FDC investigations swept bus rate, drive, deglitch and clock settings. Single-cell tests expanded to matrix sweeps and per-row dual-device autoscan. |
| 2026-06 | 分离 INTB 唤醒、STATUS、DRDY、unread 与实际 read4，实测时序启发 watchdog、epoch 丢弃、独立修复与高速 profile。 / FDC work separated notification, hardware readiness and valid readout, with per-worker epochs, deadlines and rescue. |
| 2026-07 – 08 | 演进为双核采集、逐行混合、ASCII 帧与多通路输出。串口与 BLE 的 smoke、长跑、订阅交叉、重连、内存、校准持久化和电池测试分别记录。 / The firmware gained multicore acquisition, per-row mixed frames and transport isolation. Serial and BLE campaigns tracked subscriptions, reconnection, persistence, memory and battery behavior. |
| 2026-11-04 – 05 | 以 `8045e9e` 为比较基线。VOLT 跳变、MFAULT、ROW/MODE 域错判、COM8 打开失败为独立现场观察。修复前置健康检查首错、SAFE/模拟证据边界、周期辅助事务、模式终态和 DMA/worker 生命周期；与初始现场因果关系仍有未证实部分。 / Reliability audit separated register-health errors, cleanup, configuration domains, analog limits and host enumeration. The cause of each historical field event could not be reconstructed from the terminal log alone. |
| 2026-11-13 – 14 | 修复 S8D8 接触后双电阻恢复有限值。启动并行扫 I²C/SPI、三前端失联低速探测及重新选速。运行时 ADS CRC 单点拒收；观察到各 SPI 速率仍有错误。USB 从 Serial/JTAG 转向 OTG 数据接口并保留下载恢复。 / Fixture repair restored the second resistor. Startup and power-return qualification, bounded cell rejection, USB ownership and download recovery were tested separately. |
| 2026-11-16 – 21 | 逐次 screen 检查 ADS 末位 CRC、DMA 与真实 GPIO 末位采样、BLE 分片、MTU、优先级、CPU 和保护快照。期间的失败和同镜像复测见下方原始分项。 / Successive screens isolated ADC final-bit errors and measured BLE throughput and snapshot behavior. Earlier failures stayed associated with their original image and criteria. |
| 2026-11-25 | SoftAP 内存修复、USB 短暂停读复现与整帧准入修复 `f9a82f7`。重新执行 32 个完整窗口、600 秒、50 次切换、SHORT 快照及 USB 长停读。 / SoftAP allocation and USB whole-record admission fixes were checked against independent host receipts. Full results and outstanding limitations follow. |
| 文档统一 / Documentation consolidation | 将模块接口、历史、测试方法分开；测试脚本和固定输入迁到维护目录，历史实录按不可变提交查阅。 / Module manuals, history and test procedures now have separate homes. Reusable runners and regression inputs are retained independently of generated captures. |

**文档整理与离线复核 / Documentation consolidation.** 35 份项目 README 改为双语。移除历史测试产物前，620 个本地原始文件与版本库中对应 gzip 的解压内容逐一 SHA-256 匹配。保留板卡备份、真实 encoder 金样、实录切片及可移植的验收程序。80 个顶层 host 回归入口和 14 个 HIL 工具回归入口均在主机执行成功。此次文档整理没有产生新的板上性能或精度结果。

All 35 project READMEs are bilingual. The cleanup verified 620 local raw files against the decompressed archives in the preceding revision and separated reusable runners from run output. The 80 top-level host test entries and 14 acceptance-tool test entries passed offline. The hardware numbers below retain their original image and observation windows.

需要复核已清理的输入时，manifest 中的 `archive_git_object` 是清理前的不可变提交和路径。例如，下式仅向标准输出交付原始字节，可直接用 SHA-256 与 manifest 比对。 / Retrieve archived bytes without switching the working tree. Compare the uncompressed digest to the slice manifest.

```sh
git show dc5a5ad:docs/audit/artifacts/reference_fixture/repaired-cap8-01.rx.bin.gz | gzip -dc | sha256sum
```

<a id="final-window"></a>

## 最终镜像的实机记录 / Hardware results for the final image


## 32 配置实测 / 32-configuration matrix

**32/32 项通过正常 FULL 配置筛查。** 同一块板在完整 ELF `892f586bdb131e77614c1ca72723c9247159f9d3a7bf5b49917004383f252677`、session `0119031D7DEF4971`、boot `1` 上完成 CAP、VOLT、RES、MIXED 各 1–8 行。USB 实收 85,284 个完整帧，各窗口内部缺口合计 0。BLE 实收 85,228 个完整帧。最高缺口在 VOLT1，55 / (7,347 + 55) = **0.743042%**，低于 2.5% 门限。其他 31 项 BLE 内部缺口均为 0。

**All 32 normal FULL case screens passed** on this ELF, session and boot. Each receiver uses its own first and last complete sequence in each window. USB received 85,284 complete frames with zero internal gaps. BLE received 85,228. VOLT1 accounted for all 55 missing BLE sequences and met the 2.5% criterion. All 32 cleanup queries confirmed CAP8. [原始矩阵记录 / Run index](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/artifacts/cdc_recovery_acceptance_20261116/record-matrix-01/record-matrix-01.failfast.json) · [源码及构建身份 / Source and image identity](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/artifacts/cdc_recovery_acceptance_20261116/usb-record-01-source-identity.json)

采集 FPS 按可信帧的序号进度和设备时戳跨度计算。USB、BLE FPS 分别按主机收到完整帧的时间计算。MIXED 使用 `CVRCVRCV`，每行含 8 个 D 通道。CAP8、RES8 的主机窗口各请求 122 秒，其余 30 项各请求 120 秒。 / Capture FPS uses sequence progression over device time. USB and BLE FPS use complete-frame receipt at each host receiver. MIXED uses the named row profile. CAP8 and RES8 each requested 122 host seconds, and the other 30 requested 120. [逐项筛查 / Case screens](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/artifacts/cdc_recovery_acceptance_20261116/record-matrix-final.md)

| 模式 / Mode | 活动行 / Rows | 采集 / Capture FPS | USB 实收 / Host FPS | BLE 实收 / Host FPS | USB 完整帧 / Frames | BLE 完整帧 / Frames | BLE 缺帧 / Missing |
|---|---:|---:|---:|---:|---:|---:|---:|
| CAP | 1 | 68.206 | 68.207 | 68.211 | 8,185 | 8,185 | 0 |
| CAP | 2 | 40.243 | 40.243 | 40.243 | 4,829 | 4,829 | 0 |
| CAP | 3 | 28.100 | 28.101 | 28.102 | 3,372 | 3,373 | 0 |
| CAP | 4 | 21.867 | 21.868 | 21.869 | 2,624 | 2,624 | 0 |
| CAP | 5 | 17.888 | 17.888 | 17.888 | 2,146 | 2,146 | 0 |
| CAP | 6 | 15.137 | 15.138 | 15.138 | 1,817 | 1,817 | 0 |
| CAP | 7 | 13.121 | 13.122 | 13.122 | 1,574 | 1,574 | 0 |
| CAP | 8 | 11.575 | 11.577 | 11.576 | 1,413 | 1,413 | 0 |
| VOLT | 1 | 61.673 | 61.681 | 61.230 | 7,402 | 7,347 | 55 |
| VOLT | 2 | 43.094 | 43.098 | 43.094 | 5,171 | 5,171 | 0 |
| VOLT | 3 | 33.713 | 33.715 | 33.715 | 4,046 | 4,045 | 0 |
| VOLT | 4 | 27.695 | 27.695 | 27.694 | 3,322 | 3,322 | 0 |
| VOLT | 5 | 22.203 | 22.206 | 22.207 | 2,665 | 2,664 | 0 |
| VOLT | 6 | 19.165 | 19.178 | 19.177 | 2,300 | 2,300 | 0 |
| VOLT | 7 | 16.794 | 16.797 | 16.796 | 2,016 | 2,016 | 0 |
| VOLT | 8 | 14.793 | 14.793 | 14.794 | 1,775 | 1,775 | 0 |
| RES | 1 | 46.054 | 46.055 | 46.056 | 5,526 | 5,527 | 0 |
| RES | 2 | 28.908 | 28.909 | 28.911 | 3,469 | 3,469 | 0 |
| RES | 3 | 20.832 | 20.835 | 20.834 | 2,500 | 2,500 | 0 |
| RES | 4 | 16.253 | 16.252 | 16.252 | 1,950 | 1,950 | 0 |
| RES | 5 | 13.265 | 13.272 | 13.273 | 1,592 | 1,592 | 0 |
| RES | 6 | 11.140 | 11.147 | 11.147 | 1,337 | 1,337 | 0 |
| RES | 7 | 9.586 | 9.588 | 9.588 | 1,150 | 1,150 | 0 |
| RES | 8 | 8.243 | 8.252 | 8.252 | 1,007 | 1,007 | 0 |
| MIXED | 1 | 43.497 | 43.499 | 43.500 | 5,220 | 5,220 | 0 |
| MIXED | 2 | 20.902 | 20.905 | 20.905 | 2,509 | 2,509 | 0 |
| MIXED | 3 | 7.610 | 7.611 | 7.611 | 914 | 913 | 0 |
| MIXED | 4 | 7.066 | 7.068 | 7.067 | 848 | 848 | 0 |
| MIXED | 5 | 6.221 | 6.222 | 6.222 | 747 | 747 | 0 |
| MIXED | 6 | 5.494 | 5.496 | 5.496 | 660 | 660 | 0 |
| MIXED | 7 | 5.208 | 5.210 | 5.210 | 625 | 625 | 0 |
| MIXED | 8 | 4.774 | 4.775 | 4.775 | 573 | 573 | 0 |

<details>
<summary>无效点、ADC CRC、性能与恢复时间 / Invalid points, ADC CRC, performance and recovery timing</summary>

| 模式 / Mode | USB 无效点 / Invalid points | BLE 无效点 / Invalid points | 有 ADC 分母的窗口 / Windows with checked reads | ADC CRC 拒绝 / 检查 · rejected / checked |
|---|---:|---:|---:|---:|
| CAP | 0 | 0 | 0 / 8 | — |
| VOLT | 1,516 | 1,508 | 8 / 8 | 0 / 1,156,521 |
| RES | 459,278 | 459,285 | 8 / 8 | 0 / 2,100,836 |
| MIXED | 49,800 | 49,792 | 7 / 8 | 0 / 931,681 |

两端实收完整帧仍保留无效点，USB / BLE 分别合计 510,594 / 510,585 个。RES 和 MIXED 表中的无效点均标为 OPEN。ADC CRC 在 23 个独立 ADSHEALTH 诊断窗内为 **0 / 4,189,038**，分母是已检查读取次数，包含重试。CAP1–8 和仅激活第一条 C 行的 MIXED1 缺少检查分母。

Complete frames retain invalid points. All invalid RES and MIXED points in these windows were flagged OPEN. Across 23 bounded ADSHEALTH intervals, the ADC rejected zero of 4,189,038 checked reads, including retry attempts. CAP1–8 and MIXED1 have no checked-read denominator. [每项完整帧、缺口、ADC 与无效点 / Full case evidence](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/artifacts/cdc_recovery_acceptance_20261116/record-matrix-final.json)

32 项从首个恢复命令到最终同 boot CAP8 CONFIG 回读的耗时范围为 **56.167–265.354 ms**。从采集结束计入结束查询，耗时范围为 **195.834–660.560 ms**。性能表列出固件 emit、USB DATA 提交、BLE 本地 DATA 与 SDK 接受、各角色 CPU、heap 和剩余栈。USB 物理总线完成次数、BLE 空口完成次数及本轮缺少 MEM? 样本的最大连续内存块均为 UNKNOWN。

All 32 configurations recovered to CAP8. The measured range was **56.167–265.354 ms** from the first restore command and **195.834–660.560 ms** from the end of capture, including post-capture queries. The performance report gives firmware emit rates, USB and BLE submission counts, owner CPU, heap and stack observations, and the time bounds for each. Physical USB and BLE completion counts and the largest contiguous heap blocks remain unknown. [性能与恢复表 / Performance and recovery](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/artifacts/cdc_recovery_acceptance_20261116/record-matrix-final-performance.md) · [各层时间范围及原始观测 / Layer timings and evidence](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/artifacts/cdc_recovery_acceptance_20261116/record-matrix-final-performance.json)

</details>

### 十分钟稳定性、切换与关键帧 / Stability, transitions and protected snapshots

| 同一镜像、同一 boot / Same image and boot | 实机结果 / Measured result |
|---|---|
| VOLT1，600.000072 s | 采集 / capture 61.7392 FPS。USB 37,046 帧、内部缺口 0。BLE 36,652 帧、缺 395/37,047（1.0662%）。ADC CRC 0/394,895 次受检读取，独立计数跨度 599.1331 s。USB/BLE 各自保留 1,767/1,718 个无效点。 / All invalid point codes were retained. [Gate](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/artifacts/cdc_recovery_acceptance_20261116/record-volt600-01/record-volt600-01.gate.json) · [Metrics](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/artifacts/cdc_recovery_acceptance_20261116/record-volt600-01/record-volt600-01.metrics.json) |
| 50 次模式切换 / Mode transitions | USB 验证 50/50 个 applied、对应配置和数据。每步的首个已验证 USB 帧在 BLE 也有收件。最慢一步从发起到闭合配置回读 342.5 ms。 / Both receivers observed every transition. The maximum step-to-closing-query interval was 342.5 ms. [逐步实录 / Per-step results](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/artifacts/cdc_recovery_acceptance_20261116/record-final50-01/record-final50-01.gate.json) |
| SHORT 完整快照 / Protected full snapshots | CAP8 120.000279 s 中，BLE 收到 117 个真实完整快照，116 个相邻收件间隔最长 1.037481 s。结尾回读确认恢复 REL 和 CAP8。 / The receiver observed 117 complete snapshots. One second is the firmware scheduling target. The measured maximum host receipt interval was 1.037481 s. [快照窗口 / Snapshot window](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/artifacts/cdc_recovery_acceptance_20261116/record-short-01/record-short-01.gate.json) |

本版的矩阵和十分钟缺口统计只覆盖预先定义的正式窗口。原始文件也保留了打开接收器、切换测量配置时的旧帧。例如十分钟测试的准备阶段缺 146 个旧 CAP 序号。健康日志中 579 个预期周期槽实际收到 578 个，缺 `engineFrameCount=103488`。弱射频环境、三颗前端同时断电复电、Windows 枚举和同步模拟波形仍未实测。BLE 的降帧依据是本地排队压力，空口丢包率和 RSSI 未测。SPI 启动选速保留 2.5% CRC 容限；每个科学读数仍需通过其自身校验。

Matrix and ten-minute sequence statistics cover the defined measurement windows. The original recordings also include configuration preparation. For example, the ten-minute recording contains a 146-sequence gap in older CAP traffic before its window. We observed 578 of 579 expected health log slots. The missing slot was engine frame 103488. Weak RF, simultaneous power cycling of the three front ends, Windows enumeration and simultaneous analog waveforms remain untested on this build. BLE pacing uses local queue pressure. Peer packet loss and RSSI remain unknown. SPI startup qualification keeps its 2.5% CRC threshold and each scientific read is checked. [健康周期与边界 / Health and window coverage](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/artifacts/cdc_recovery_acceptance_20261116/record-volt600-01/record-volt600-01.review.json)

### 正式 USB 长时间停读与 Wi-Fi 共存 / Unread USB and Wi-Fi coexistence

正式 CDC0 文件保持打开 120.000483 秒，应用读写次数为 0，起止原始文件偏移均为 28,092。主机在此期间的 USB FPS 未观测。前后 USBQ 查询之间 DATA 本地提交 65 条，普通记录过期 7,493 条，DATA 拒绝和部分写增量均为 0。BLE 在停读窗口内收到 7,562 个完整帧，端点间缺 11/7,573（0.1453%），其中保留 192 个无效点。ADC 诊断独立覆盖 119.657 秒，CRC 拒绝 0/79,896。恢复后同次启动确认 CAP8。

During the 120-second application read pause, the USB file descriptor stayed open. The host made zero reads and writes. Its receive rate was unobserved. The bounded queue expired old records while BLE continued receiving full frames. The Linux kernel could still accept USB traffic. The post-test state probe verified CAP8. [停读原始分析 / Unread test](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/artifacts/cdc_recovery_acceptance_20261116/record-unread-01/record-unread-01.analysis.json)

同一镜像上 USB 发起 AP、AP、OFF、AP、OFF 五次请求，BLE 确实收到五条对应 WTERM，首次 AP 从接受到完成为 59.238 ms。内部可用内存从 93,108 B 到 28,744 B，最大连续块从 47,092 B 到 20,468 B，历史低水位 27,860 B，分配失败计数无增量。最终查询为 CAP8、Wi-Fi 输出 OFF。 / All five Wi-Fi requests reached applied terminals and were independently received over BLE. OFF is the queried output gate. UDP peer throughput and physical radio-off were unmeasured. [终态、堆与结束状态 / Verified terminals, heap and final state](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/artifacts/cdc_recovery_acceptance_20261116/record-ap-01/record-ap-ble-terminals-check.json)

## 正式 USB 短暂停读回归 / Formal USB reader-pause regression

ESP32-S3 实机在 VOLT 两行、USB 与 BLE 同时接收的条件下，分别测了旧固件及修复提交 `f9a82f7`。两次都在 120 秒窗口内暂停主机 CDC0 应用读取约 650 ms，BLE 观察器继续运行。下表中的缺帧按各自通路的完整帧序号计算，设备计数取本次窗口增量。

The same board ran VOLT on two rows with formal CDC0 and BLE receivers. Both 120-second windows included a 650 ms pause in the host application's CDC0 reads. The BLE observer kept recording. Sequence gaps below use complete frames at each receiver. The device columns show changes across each run.

| 版本 / Image | USB 实收 / 缺口 · received / missing | BLE 实收 / 缺口 · received / missing | USB DATA 拒绝 / rejected | 普通过期 / normal expired | 部分写 / partial |
|---|---:|---:|---:|---:|---:|
| 旧 / Previous | 5,033 / 12 | 5,045 / 0 | 7 | 5 | 1 |
| 修复 / Fixed | 5,225 / 0 | 5,225 / 0 | 0 | 0 | 0 |

固件现在等 TinyUSB FIFO 有整帧空间才提交正式 DATA。PSRAM 能容纳 100 个待发记录和一个正在处理的记录，入队后最多保留 1 秒。CTRL 在记录边界优先，调试日志用另一 CDC 接口。长时间无人读取仍会产生有计数的缺口。两次窗口都查询确认恢复 CAP8。旧镜像的自然停顿和人为暂停有独立原始记录。

The USB owner waits until the FIFO can accept the full science record. The PSRAM queue holds 100 waiting records plus one in service, with a one-second lifetime from enqueue. CTRL gets priority at record boundaries. Debug uses a separate CDC interface. A long reader outage can still cause counted drops. Both runs ended with queried CAP8 recovery. The original interruption and the controlled pauses have separate recordings. [自然停顿 / Original interruption](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/artifacts/cdc_recovery_acceptance_20261116/bss-volt2-usb-stall-analysis-01/evidence.md) · [旧镜像 / Before](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/artifacts/cdc_recovery_acceptance_20261116/bss-pause-volt2-01/bss-pause-volt2-01-volt2/bss-pause-volt2-01-volt2.gate.json) · [修复后 / After](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/artifacts/cdc_recovery_acceptance_20261116/record-pause-volt2-01/record-pause-volt2-01-volt2/record-pause-volt2-01-volt2.gate.json) · [主机契约 / Host contract](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/integration/firmware_host_contract.md)

### VOLT8 逐帧电压与夹具 / Frame-level voltage and fixture checks

8 行 VOLT 使用 ADS 供电脚附近万用表读数的 RAM 标称值，AVDD 为 3.350 V，AVSS 为 −1.795 V。序号 62819–64593 的 1,775 帧连续。对同一单元相邻两帧都有效的 111,792 个点对逐一相减，最大绝对变化为 14.716 mV，位于悬空 S8D1。1,156 个 X12 无效点也都在该单元。这个 120 秒窗口没有观察到原现场多通道共同的 18–19 mV 跃升。101 个序号在设备时钟上相隔 6.640–6.985 秒。健康记录的独立计数窗有 185,295 次受检读取，ADC CRC 拒绝为零。记录的 120 笔电池事务均成功恢复。

The VOLT8 window used the RAM-only rail values derived from the meter readings near the ADS supply pins. Across 1,775 consecutive frames, 111,792 pairs of consecutive valid point readings showed a maximum change of 14.716 mV at floating S8D1. All 1,156 X12 values were retained at that cell. The earlier common 18–19 mV step was absent in this window. A span of 101 frames took 6.640–6.985 seconds. The separate register-health span contained 185,295 CRC-checked reads with zero rejections. All 120 recorded battery transactions restored successfully. Per-cell raw ADC codes and simultaneous analog waveforms were unavailable. [逐帧原值与时序 / Frame-level review](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/artifacts/cdc_recovery_acceptance_20261116/record-volt8-frame-review-01/findings.md)

CAP8 的 122 秒主机窗口包含 121.983293 秒设备数据，USB/BLE 各 1,413 帧。S4D4、S5D5 的平均总电容是 92.536、77.425 pF。逐帧减去 60 个悬空点的中位数后分别为 32.573、17.462 pF。选用同列悬空点时分别为 13.445、11.716 pF。这些差异是归零估计对背景选择的敏感度。 / The 122-second CAP8 recording spans 121.983293 seconds on the device. Both receivers captured 1,413 frames. Mean total capacitance was 92.536 and 77.425 pF. Subtracting the median of the 60 floating cells per frame gave 32.573 and 17.462 pF. Column-local subtraction gave 13.445 and 11.716 pF. Absolute capacitor accuracy remains unknown without unloaded measurements of the same two cells. [实测与各归零口径 / Fixture measurements](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/artifacts/cdc_recovery_acceptance_20261116/record-fixture-cap8-01/cap8.report.md)

RES8 在 122.041209 秒设备时间内采到 1,007 帧。S1D1 和 S8D8 每帧都有有限阻值，其余 60 个悬空点均报 OPEN。以下数值均为 Ω。

| 夹具 / Fixture | 原始均值 / Raw mean | 原始范围 / Raw range | 相对 10 kΩ 标称 / Deviation from nominal |
|---|---:|---:|---:|
| S1D1 | 10,164.972 | 10,159.550–10,171.083 | +1.6497% |
| S8D8 | 10,166.390 | 10,160.526–10,171.652 | +1.6639% |

只用 S1D1 的原始均值计算比例 0.983770502386，固定后应用于 S8D8，得到均值 10,001.394 Ω，P05–P95 为 9,998.841–10,003.941 Ω。两点共用电源，但板上参考电阻和开关路径不同。该结果量化夹具在 10 kΩ 附近的一致性。设备与原始文件保留未经归一化的阻值。

Both resistors had 1,007 finite readings. All 60 floating cells reported OPEN. Using S1D1 alone to set the scale factor gave a held-out S8D8 mean of 10,001.394 Ω (+0.01394% from nominal). The standards share the board rails and use different on-board divider resistors and switch paths. Absolute resistance accuracy and calibration at other resistances remain undetermined. The calculation ran offline and the firmware retained its original calibration. [原始夹具 / Raw fixture](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/artifacts/cdc_recovery_acceptance_20261116/record-fixture-res8-01/res8.review.md) · [归一化复算 / Held-out calculation](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/artifacts/cdc_recovery_acceptance_20261116/record-10k-reference-check-01/report.md)

> **2026-11-25 · 原始32配置矩阵 / Original 32-configuration matrix — 9 PASS, 1 FAIL, 22 NOT_STARTED.** 在同一镜像、session 和 boot 上，CAP 1–8行及 VOLT 1行通过。VOLT 2行的 USB 缺7个完整帧，BLE 收齐5,050帧，矩阵随后停止。CAP 2行首次预检中止，第二次正式窗口通过。 / CAP1–8 and VOLT1 passed. The run stopped when USB missed seven complete VOLT2 frames. CAP2 passed after an observer preflight error. [矩阵及夹具 / Matrix and fixture](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/README.md#matrix-bss-01-02) · [全部32项 / All 32 cases](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/artifacts/cdc_recovery_acceptance_20261116/bss-matrix-combined-01.md)

> 同一 boot 的600秒 VOLT1 正常 FULL 筛查通过，50次切换均有 USB 验证与匹配 BLE 帧，SHORT 收到117个完整快照。Wi-Fi UDP 上位机实收仍未测，夹具精度与整体验收待验证。 / The 600-second VOLT1 screen passed, all 50 transitions had USB verification and matching BLE frames, and SHORT delivered 117 complete snapshots. Host Wi-Fi UDP receipt, fixture accuracy and overall acceptance remain open. [三个独立窗口 / Three separate recordings](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/README.md#bss-current-recordings)

> **历史记录 / Historical records.** 下方旧 ELF 及旧阶段的 FAIL 和 NOT_RUN 按各自镜像和启动周期保留。当前矩阵只合并 `bss-matrix-01` 与 `bss-matrix-02` 的同身份记录。 / Earlier FAIL and NOT_RUN entries describe the image and boot named in each record. The matrix above combines only the two named runs on their verified common identity.

> **2026-11-25 AP08启动实录（boot1） / AP08 startup observation (boot1).** [wifi-bss-01构建](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/artifacts/cdc_recovery_acceptance_20261116/wifi-bss-01-validated-identity.json)与[READY核验](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/artifacts/cdc_recovery_acceptance_20261116/wifi-bss-ready-01.summary.json)绑定完整ELF`179f9183e2c31b35c906c9b811bb2ca1cf4d16dbca43ee3a1c6c4371c0f56a2d`、session`00B44EE26D0A4A0F`/boot1/CAP8/READY1；80入口/构建通过、203生产输入一致。SDK外部BSS及8,704 B任务校准表合计迁移29,116 B，保留既有采集速率与缓冲容量。[AP08](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/artifacts/cdc_recovery_acceptance_20261116/wifi-control-08/wifi-control-08.summary.json)的AP/AP/OFF/AP/OFF五请求全applied，首次AP53.432ms；最终同boot outputGate0/backend ready/quarantined0。internal free/largest前93,356/47,092 B、后29,008/20,468 B、min27,952 B；MEMFAIL0→0、incomplete0，无观察fault/reset，BLE observer正常关闭。**OFF只关输出gate，不证明radio断电；AP08当时未创建正式FPS或最终验收窗口。后续boot2的600秒、50切换及SHORT已闭合，见当前实测小节；UDP实收仍未测。** AP04/AP05的NO_MEM、AP06分配失败与AP07 panic历史保留。 / The exact image boots CAP8/READY1 and passes all 80 entries/build. Supported external BSS placement moves 29,116 bytes, including the task-owned calibration table, without reducing acquisition rate or buffer capacity. All five AP/AP/OFF/AP/OFF requests apply; the first AP completes in 53.432 ms while USB/BLE control coexists. The backend remains ready with the output gate closed, no observed allocation failure, fault or reset, and a clean BLE observer shutdown. OFF does not prove radio power-down. AP08 itself has no formal throughput or final-suite window. Later boot2 long-run, transition and SHORT recordings are closed below; UDP receipt remains untested and earlier failures remain preserved.

> **2026-11-25 保留AP07启动panic / Preserved AP07 startup panic.** [wifi-dma-headroom-01](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/artifacts/cdc_recovery_acceptance_20261116/wifi-dma-headroom-01-validated-identity.json)关闭SDK可选Wi-Fi IRAM speed两项后，map回收18,432 B共用SRAM；完整ELF`2fed45c22153b580aef27584107809d15cdabdd8afe6d6bd90cf6d5a282c07a7`、80入口/构建通过、203生产输入一致。[AP07](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/artifacts/cdc_recovery_acceptance_20261116/wifi-control-07/wifi-control-07.summary.json)前session`DE8FA9C044C728B7`/boot1，internal free/largest64,324/31,732 B；随后USB EOF/BLE断连，AP终态和OFF恢复未知。[USJ保留panic](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/artifacts/cdc_recovery_acceptance_20261116/dma-panic-01.summary.json)的record`C3FA0326`匹配该origin/同ELF；新session`DC4763F0436668C0`/boot2，PC`420C9C8D`位于`ieee80211_hostap_attach`，cause0x1C/vaddr0x2C。**AP07未取得post-MEMFAIL，不推定具体allocator；boot2的count0不能回填boot1。** 该run未证明AP/UDP可用；AP04/05/06旧证据保留，AP08结果单列于上方。 / The optional-code placement change releases 18,432 bytes in the map and passes all 80 entries/build. AP07 starts with more internal memory but loses USB and BLE. A retained panic matches its origin and exact ELF, locating ieee80211_hostap_attach in AP startup. No post-attempt MEMFAIL exists; the new boot's zero counter does not identify an allocator or fill the missing prior-boot observation. This run does not establish AP/UDP operation; the later AP08 result is recorded separately above.

> **2026-11-25 保留AP06分配失败观察 / Preserved AP06 allocation-failure observation.** [heap-failure-02构建](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/artifacts/cdc_recovery_acceptance_20261116/heap-failure-02-validated-identity.json)与[启动核验](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/artifacts/cdc_recovery_acceptance_20261116/heap-failure-ready-02.summary.json)绑定完整ELF `90f10fdaa5a0b859995caba70b1f1c65d3c7f9e19e85b39c81366e699c6e2736`、session `EE880CC8223D06C2`/boot1/CAP8/READY1，80入口/构建通过、203生产输入一致。[BLE共存AP06](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/artifacts/cdc_recovery_acceptance_20261116/wifi-control-06/wifi-control-06.summary.json)的`MEMFAIL?`由count0增至1、incomplete0：**1×1,604 B，caps0x80C（INTERNAL|DMA|8BIT）**；同ELF解码fn `0x3C1506DC`为`heap_caps_malloc`。[静态二进制路径](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/artifacts/cdc_recovery_acceptance_20261116/wifi-memory-source-map-01.md)强匹配Wi-Fi static RX初始化，**不是运行时caller/backtrace证明**。AP仍在34.050ms返回NO_MEM并隔离，cleanup0x103未知保留；internal free/largest前44,856/31,732 B、后28,480/9,716 B、min7,996 B。OFF gate同boot可查，无观察fault/reset；AP04/AP05旧值保留。**本轮未证明AP/UDP可用，无正式FPS窗口，600秒/32配置/50切换/SHORT仍NOT_RUN。** / The exact diagnostic image is boot-verified and all 80 entries/build pass with 203 matching inputs. During BLE coexistence, AP06 records one failed 1,604-byte internal DMA allocation. The decoded allocator API and static RX initialization path strongly match; the hook does not prove the runtime caller. AP still returns NO_MEM in 34.050 ms, with uncertain cleanup retained. Same-boot OFF queries remain available, with no observed fault/reset. AP04/AP05 history stays intact. AP/UDP availability, formal throughput and final acceptance are not established.

> **2026-11-21 保留AP05观察：Bluedroid优先PSRAM后AP仍NO_MEM / Preserved AP05 observation after Bluedroid PSRAM preference.** [bt-host-memory-01闭合证据](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/artifacts/cdc_recovery_acceptance_20261116/bt-host-memory-control-evidence-01.json)绑定完整ELF `2f74130825f1b4c173cee27bf569bc4385bbdd26a7049adee3efe6e8b77c39e9`、session `55B0EC5D078FCD53`/boot1/CAP8/READY1；80入口/构建通过、203生产输入一致。官方`CONFIG_BT_ALLOCATION_FROM_SPIRAM_FIRST`只改变Bluedroid动态host分配优先级并保留内部fallback，不迁controller DMA/任务栈/队列。BLE前internal free47,144 B，较前一镜像增加5,528 B；BLE共存AP05仍以NO_MEM0x101终结，accepted→terminal34.482ms、cleanup0x103未知保留。AP前free/largest45,716/31,732 B，后29,076/10,740 B、min8,592 B；OFF及旧失败缓存可查，无观察fault/reset。原BLE journal实际回放确认L通道收到id1 AP失败与id2 OFF两条WTERM，不以本地queued代替实收。旧采集器仅有recorded SHA、源码未独立归档，原始回复/事件另行核验。**无正式FPS窗口，最终600秒/32配置/50切换/SHORT仍NOT_RUN。** / The exact image boots CAP8/READY1 and passes all 80 entries/build with 203 matching inputs. Official Bluedroid host allocation now prefers PSRAM with internal fallback; controller DMA, task stacks and queues retain their placement. Pre-BLE internal free memory is 5,528 bytes higher, but AP05 still returns NO_MEM in 34.482 ms with uncertain cleanup preserved. Both exact WTERM results reach this PC on lifecycle L. The historical collector has a recorded SHA without a separate source archive; original replies/events are independently checked. No formal FPS or final acceptance is claimed.

> **2026-11-21 保留首次AP失败观察（当时原因未知） / Preserved first AP failure observation, before diagnosis.** 同`wifi-owner-02` ELF `9e1ee9f9812dcd21a1295e315f08c10bdf98ae55d309e67b1f84b97b3c825573`的[AP实录](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/artifacts/cdc_recovery_acceptance_20261116/wifi-control-01/wifi-control-01.summary.json)中，id1先accepted/pending，随后查询running；AP写入后约10ms内出现USB EOF，终态未收到，OFF恢复FAILED_OR_UNKNOWN（主机事件时间，不是精确panic时刻）。[后续只读证据](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/artifacts/cdc_recovery_acceptance_20261116/wifi-failure-usj-01.summary.json)确认同ELF `reset=panic`、boot2 / session `370C59E1DF8015F5`、CAP8/READY1，控制在`serial_jtag`恢复通路可查询。**根因仍待诊断；先修复，600秒/32配置/50切换/SHORT及手动重启验收均NOT_RUN。** 新boot的idle/零guard计数不能证明旧SDK调用已取消，旧session的最终测试参数已失效。 / Request1 was accepted and observed running, followed by USB EOF roughly10ms after the AP write, with no terminal and unverified OFF restoration. Subsequent read-only queries confirm a same-image panic reboot and accessible Serial/JTAG recovery in the new boot. The root cause is not established; repair precedes final testing. New-boot idle counters do not prove cancellation of the previous call, and old-session capture arguments are no longer valid.

> **2026-11-21 AP尝试前的构建/启动核验记录 / Preserved build and boot verification before the AP attempt:** `wifi-owner-02` / ELF **`9e1ee9f9812dcd21a1295e315f08c10bdf98ae55d309e67b1f84b97b3c825573`**、bin `55b25e40cc20de1b619f000080d5fe984752e1e693b2327e6e2dda4de87c2c9e`，[78入口与构建](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/artifacts/cdc_recovery_acceptance_20261116/wifi-owner-02-validated-identity.json)全部exit0、201生产输入一致，app1,628,976 B；[完整镜像归档](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/artifacts/cdc_recovery_acceptance_20261116/wifi-owner-02-image-archive.json)另存。[烧录后启动证据](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/artifacts/cdc_recovery_acceptance_20261116/wifi-owner-02-boot-verification.json)：session `0ACE6CD97F8649DC` / boot1，CAP8/READY1，SPI启动资格2MHz、2000样本CRC/status/reg0；这是启动扫描，不是持续稳定性。MEM当前internal free/largest/min为27,408/11,252/21,044 B，scan创建前largest25,588、创建后8,180 B，查询sequential。Wi-Fi仍默认OFF、resources0、backend inactive，AP与显式重启/ROM枚举尚未实测；新镜像600秒/32配置/50切换/SHORT尚未运行。重启后的session必须重新实录，不能复用本次身份。 / All78 host entries and the build pass with201 matching production inputs and a1,628,976-byte app. Archived images and post-flash observations verify CAP8/READY1 in the stated session. The2MHz/2000-sample zero-error startup sweep is not a sustained run. Internal-memory observations are sequential and retain the scan-allocation cost. Wi-Fi remains uninitialized/OFF; AP, explicit restart/ROM enumeration and the new image’s long-run/matrix/transition/SHORT tests remain unrun. A later restart requires freshly recorded identity.

> **2026-11-21 保留screen21吞吐筛查，仅通过正常BLE数值预算 / Preserved screen21 throughput screen, meeting only the normal BLE numeric budget:** `ble-snapshot-06` / ELF **`c47a3710a770506abd030d03790b58c137150c9c1cba86a596f07962114a3c4e`** 已[启动核验](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/artifacts/cdc_recovery_acceptance_20261116/ble-snapshot-06-boot-verification.json)，session `630DC17010A097FF` / boot1；195生产输入一致，[73入口与构建](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/artifacts/cdc_recovery_acceptance_20261116/ble-snapshot-06-validated-identity.json)均exit0。[screen21](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-21/screen-21-verdict.json)完成 **120.000119494秒** VOLT1 USB＋BLE：Capture/USB/BLE **64.195756/64.233205/63.079023 FPS**，USB7,707连续，BLE7,569缺141，[同criteria v2](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-21/screen-21-criteria-v2-supplement.json) **141/7,710=1.828794% ≤2.5%**。判定仍 **OBSERVED_NOT_ACCEPTED / overall NOT_ESTABLISHED**；ADC独立窗0/81,815、USB/BLE invalid460/434保留、CAP8恢复核验。600秒、32配置USB＋BLE、50切换及本镜像SHORT/弱链路未跑；Wi-Fi隔离与手动重启补丁已完成源码/host验证，新目标仅已构建/启动，AP/重启/持续HIL仍待独立闭合，未宣称整体无P0/P1。 / Build06 boots successfully and all 73 host entries/build pass with 195 matching inputs. The closed **120.000119494-second** screen records **64.195756/64.233205/63.079023 FPS** capture/USB/BLE. USB has 7,707 internally consecutive frames; BLE has 7,569 with 141 missing observations: **1.828794% ≤2.5%**. Only that numeric budget passes; the verdict remains **OBSERVED_NOT_ACCEPTED / overall NOT_ESTABLISHED**. Separate ADC observations are 0/81,815, invalid points remain, and CAP8 recovery is verified. Long-run, complete configuration matrix, transition and same-image specialized acceptance remain unrun; Wi-Fi isolation/manual-restart source and host verification are complete, while the new target is only build/boot verified and its AP/restart/sustained-HIL gates remain open.

> **2026-11-21 保留build05筛查失败 / Preserved build05 screen failure:** `ble-snapshot-05` / ELF **`7f2d5ad9c73887f70d49c7f8eb45ca889c4c44fcb1fedd293299200d9b39b469`** 已[实际启动核验](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/artifacts/cdc_recovery_acceptance_20261116/ble-snapshot-05-boot-verification.json)，session `B73989CDD936FA54` / boot1。[73入口与构建](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/artifacts/cdc_recovery_acceptance_20261116/ble-snapshot-05-validated-identity.json)均通过、195生产输入一致；[screen20](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-20/screen-20-verdict.json)完成 **120.000081240秒** VOLT1 USB＋BLE，Capture/USB/BLE **63.796225/63.871027/62.188982 FPS**。USB7,664连续，BLE7,461缺202，[同criteria补充](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-20/screen-20-criteria-v2-supplement-02.json) **202/7,663=2.636043% > 2.5%：FAIL**。内部2,304 B发送工作区尚未证明性能收益；ADC独立窗0/80,344、USB/BLE无效点483/456保留，CAP8恢复核验。旧screen19/18/575c失败不覆盖；build04的SHORT/USB不读专项不替代新镜像整体或长期验收。 / Build05 is flashed and boot-verified, with all 73 host entries/build passing and 195 matching inputs. The closed **120.000081240-second** screen still **fails**: BLE receives 7,461 frames with 202 missing observations, **2.636043% > 2.5%**, while USB receives 7,664 consecutive frames. Capture/USB/BLE rates are **63.796225/63.871027/62.188982 FPS**. A 2,304-byte internal working copy has no demonstrated performance benefit. Separate ADC observations are 0/80,344 with invalid points retained and CAP8 recovery verified. Earlier failures remain; build04-specific SHORT/unread observations do not qualify this image's overall or long-run behavior.

> **2026-11-21 保留build04筛查失败 / Preserved build04 screen failure:** `ble-snapshot-04` / ELF **`a6b04a2a68ddcb5c8c39cc28020abbe5db430c1e140cbb3afb4ddb703fdb217a`** 已[启动核验](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/artifacts/cdc_recovery_acceptance_20261116/ble-snapshot-04-boot-verification.json)，session `B4828F8EF45001F1` / boot1。195生产输入一致，[73入口整轮与构建](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/artifacts/cdc_recovery_acceptance_20261116/ble-snapshot-04-validated-identity.json)均exit0；[screen19](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-19/screen-19-verdict.json)闭合 **120.000139309秒** VOLT1 USB＋BLE，Capture/USB/BLE **64.392618/64.399646/62.340491 FPS**。USB7,728帧内部连续；BLE7,481帧缺250，[同criteria v2](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-19/screen-19-criteria-v2-supplement.json) **250/(7481+250)=3.233734% > 2.5%：FAIL**。周期合并生效、保护queued/complete/staleNormal/deadlineMiss增量均0，但容量拒绝仍+228；不能从250中抵扣。独立ADC窗0/82,589，USB/BLE无效点502/471保留，CAP8恢复已查询核验。600秒/50切换/32配置整套/SHORT/弱链路仍未验收，不宣称整体无P0/P1。 / All 73 host entries and the build pass with 195 matching inputs, and startup is verified. The closed **120.000139309-second** screen nevertheless **fails** the normal BLE gap budget: **3.233734% > 2.5%**, at **64.392618/64.399646/62.340491 FPS** capture/USB/BLE. USB receives 7,728 internally consecutive frames; BLE receives 7,481 with 250 missing observations. Coalescing eliminates additional protected sends/overtaking in the observed query interval, while 228 local capacity rejections remain; they cannot be subtracted from PC gaps. A separate ADC window records 0/82,589 rejections, invalid points remain, and CAP8 recovery is verified. Longer and broader acceptance remains incomplete.

> **2026-11-21 保留旧镜像筛查失败 / Preserved prior-image screen failure:** `ble-snapshot-03` / ELF **`7ce5e9f8746aefe0f999b467bc08fc9470903a59141339829c6363a30db7e1c8`** 的[启动恢复](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/artifacts/cdc_recovery_acceptance_20261116/ble-snapshot-03-boot-verification.json)已实证；同session `0C1E0125F83FF11C` / boot1的[screen18](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-18/screen-18-verdict.json)完成120.000089528秒VOLT1 USB＋BLE。Capture/USB/BLE **63.042560/63.017922/61.120460 FPS**；USB7562帧内部连续，BLE7334帧缺228，[criteria v2补充](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-18/screen-18-criteria-v2-supplement.json) **228/(7334+228)=3.015075% > 2.5%：FAIL**。独立健康窗ADC **0/79,590**，USB/BLE无效点466/442保留，CAP8查询恢复。该旧镜像600秒、50切换、32配置整套、SHORT/弱链路验收仍NOT_RUN；不宣称整体无P0/P1。 / Startup is verified, but the completed 120-second screen fails the normal-link gap budget. Capture/USB/BLE rates are **63.042560/63.017922/61.120460 FPS**. USB receives 7,562 internally consecutive frames; BLE receives 7,334 with 228 missing observations: **3.015075% > 2.5%**. Separate ADC observations have 0/79,590 CRC rejections and retain 466/442 invalid USB/BLE points. CAP8 recovery is query-verified. Long-run, complete configuration-suite, transition, SHORT and weak-link acceptance remain unrun for this image; overall P0/P1 absence is not established.

> **2026-11-21 旧候选启动失败保留 / Preserved previous-candidate startup failure:** `ble-snapshot-02` / ELF **`575c5dc25217721cfb47f651556d0e28edd4b5fe666a022200b8737608ff2e40`** 已实际烧录；[只读设备证据](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/artifacts/cdc_recovery_acceptance_20261116/ble-snapshot-state-01.summary.json)报告 `READY=0,stage=scan_task_create_failed,err=0x101`（ESP_ERR_NO_MEM）。USB 控制仍可查询，SPI `phase=idle,attempted=0,samples=0`，因此 ADC CRC、采集性能和保护快照 HIL 均 **NOT_RUN**，初始化 `crc=0` 不是零错误率。`BOOT.heap=8295496` 不是 internal largest block。该失败已由上方新镜像独立复验启动恢复，原失败记录不改。 / The previously flashed candidate reports scan-task allocation failure while USB control remains responsive. SPI never starts; zero initialized counters are not a measured CRC rate. Aggregate 8.29 MB heap does not prove a contiguous internal stack allocation. The new image above independently verifies startup recovery; original failed evidence remains unchanged.

> **2026-11-21 历史完整采集窗口 / Historical completed capture window:** screen17 在 `8a1794c-dirty`、完整 ELF **`f1ec518dfba78453dcd7fe057bba29eb61a8f8898274a21e22125df26b933a04`** 上录制 **120.000057708 秒**；Capture/USB/BLE 为 **64.978526/65.035014/64.800778 FPS**，USB 7,805 帧内部连续、BLE 7,776 帧缺 29。按 criteria v2，**29/(7,776+29)=0.371557% ≤ 2.5%**，仅正常链路缺口数值预算通过；[补充判定](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-17/screen-17-criteria-v2-supplement.json)不代表整体 PASS，原 `OBSERVED_NOT_ACCEPTED` 保留。独立健康窗 ADC CRC **0/81,537**；USB/BLE 无效点 373/370 保留，CAP8 查询恢复。普通 LOG 策略已有查询证据；新弱链路适应/保护快照仍实施中，本 ELF 的 600 秒/50 次切换/32 配置未运行。 / Screen17 records **120.000057708 seconds** on `8a1794c-dirty`, full ELF **`f1ec518dfba78453dcd7fe057bba29eb61a8f8898274a21e22125df26b933a04`**, at **64.978526/65.035014/64.800778 FPS** capture/USB/BLE. USB has 7,805 internally consecutive frames; BLE has 7,776 with 29 missing observations. Under criteria v2, **29/(7,776+29)=0.371557% ≤ 2.5%** meets only the normal-link numeric gap budget. The [supplement](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/artifacts/cdc_recovery_acceptance_20261116/screen-17/screen-17-criteria-v2-supplement.json) is not overall PASS; original `OBSERVED_NOT_ACCEPTED` remains. The separate health span has **0/81,537 ADC CRC rejections**; 373/370 USB/BLE invalid points remain, and CAP8 recovery is query-verified. Ordinary-LOG policy has query evidence; new weak-link adaptation/protected snapshots remain in progress. This ELF’s 600-second/50-transition/32-configuration runs are NOT_RUN.

> 历史连续稳定性 / Historical continuous stability: `a656d53` 的独立 600.000075751 秒 USB+BLE 结果完整保留；其 ADC 0/428,131 和 CAP8 恢复仅属于旧镜像，不能填入 `53005f0-dirty` 的 600 秒验收。screen11 增加 CRC32 ROM/planner 等价优化和显式 SCLK 低电平 hold；screen12 仅进一步请求 MTU517，均不放宽科学 CRC/invalid 门限。 / The independent 600.000075751-second USB+BLE result on `a656d53` remains preserved. Its 0/428,131 ADC result and CAP8 recovery belong to the previous image and do not fill the new image’s 600-second acceptance. Screen11 adds equivalent ROM-CRC32/planner optimization and an explicit SCLK-low hold; screen12 additionally requests MTU517. None relaxes scientific CRC/invalid criteria.

Still in test


## 现场线索与未闭合条件 / Field evidence and outstanding coverage

E1–E3：早期八行 VOLT 中 rail 事件相隔 101 帧，随后报告 MATRIX_ROUTE、0x108 和零次转换尝试；两个模式区间按包含首帧计为 512 与 128 帧。这提示按 ADS engine 计数核对周期回读，不能从 mode 的 seq 推算 engine 从零起步。E4 的 unstable、rail_invalid 和未知 lastGood 分开记。E5 中 CAP 的旧 gen/rid 属于行数域。E6 只观测到一次 COM8 打开失败。E7 的采集 FPS、BLE FPS 与累计 drop 各有自己的分母。

The early rail-step correlation did not establish analog causation. Zero conversion attempts preceded the recorded terminal failure. The two observed interval lengths motivated testing the actual ADS engine counter across mode changes. Battery validity, mode authority, host port visibility and interval transport losses were investigated independently. Weak-signal thresholds, independent physical waveforms, absolute capacitor accuracy and Windows enumeration require their own observations.


<a id="fw-register"></a>

## FW 审计编号 / Reliability register

下列逐项保留原审计的证据、修复和限制。FW03 的原始定义未给出；FW04–09 依报告按主题归组。 / The original audit grouped FW04–09 by subsystem and left FW03 without an assigned meaning. Outcomes were scoped to that audit.

| ID | 证据与实现 / Evidence and implementation | 测试 / Checks | 残余风险 / Residual limits |
|---|---|---|---|
| FW01 | 同 notification 两次 pdTRUE 清零表达 JOB/RUN；超时路径持有协调栈。持久 slot + 显式阶段 + 单调 deadline + CANCEL/QUARANTINED；旧 DONE 不满足新 epoch；未释放不删 task、不复用、不串行回退 | lifecycle 与完整生产 worker/coordinator pthread shim；56 事务，50正常及 JOB/ACK/RUN/DONE/cancel/bus阻塞；ASan/UBSan PASS | 同步 I²C 无强制取消；硬件阻塞上限与锁持有时间 HIL 未测。隔离保留所有对象至释放或受控重启 |
| FW02 | 同步 SPI transmit 和 DMA completion 生命周期不能靠一次 timeout 结束。改用 handle 持久 descriptor/buffer，lock/queue/result/DRDY/retry 消耗统一 deadline；超时 quarantine | 实际生产 driver stub：mutex/queue/complete总预算、跨调用预算、迟到completion、不复用/reset/free；PASS | 无局部 DMA 取消 API，不宣称已终止。250ms helper 的清理也受原期限约束，耗尽则 restore未知并隔离 |
| FW03 | 原始编号定义未提供 | NOT_ASSESSED（编号映射） | 不杜撰编号含义；本次全部具名入口的覆盖见其余行 |
| FW04–FW09 / SAFE | 先错后 safe=true、INTREF未知时钳位。SAFE requested/applied/known/verified 分开，只有必要动作和回读成功才 safe；Q1钳位前确认INTREF OFF；quarantine阻止重新配置 | 实际 RouteController + BoardMap mock，10场景 PASS | safe 是软件路由验证，不是模拟安全；analogIsolationVerified=false，ROUTE? physicalSafe=unknown |
| FW04–FW09 / 前端 | FDC worker未释放仍可能重配；sleep/shutdown与cache混淆。增加quiescence门禁；SD强关清ready/row shadow；已有物理SD的ADS隔离路径保留 | FDC runtime、FDCISO mailbox、route 回归 PASS | 不能证明非响应器件模拟隔离；未知依赖保守停机，恢复CAP需明确restart |
| FW04–FW09 / rail | SAFE_RAIL_MONITOR 实际连接REF、没有持续断开；span被当独立轨；外部校准被帧龄淘汰 | capability=false时零硬件动作；范围/来源/估算/外部时间策略回归 PASS | VOLT需合法外部校准；本板R38–45加载不能靠offset修正；实装与不确定度未知 |
| FW04–FW09 / 主调度 | 辅助事务在行间、旧FDC救援可跨模式、恢复请求无配置身份 | helper统一frame boundary；全扫仅homogeneous CAP，配置变化丢弃旧全扫/cell请求；RECOVER绑定session和域身份，旧请求RERR；生产RecoveryPost/Take回归 PASS | 真实混合行切换的电气建立未测；诊断不会自动证明安全 |
| FW04–FW09 / 内存与并发 | worker栈逃逸、双worker共写tracker、发送时取全局metadata、版本号被误当互斥 | 持久对象；独立tracker；Core1单writer；authority/mailbox短临界区；slot容量按真实encoder极值；sanitize/metadata/pool PASS | Xtensa实际heap/连续块/栈水位NOT_RUN；seqlock仍依赖单writer约束 |
| FW04–FW09 / watchdog与预算 | READY或30s窗口会重新发放自动重启预算，慢故障可循环 | 最大3自动restart跨软件启动保留；只由真实healthy采集进度重新授予；READY/SAFE/错误不清预算；缺帧不计连续健康；生产函数回归 PASS | 保留现有idle TWDT；未添加能覆盖任意“让出CPU但无进度”缺陷的独立硬件验证监控，不能把idle TWDT或wdt=unknown说成采集健康证明 |
| FW10 | 低DRDY电平不等于新配置转换；SPI数据CRC未全路径启用 | config/physical generation与DRDY关联；清通知不清硬件新鲜度；CRC8同步STATUS/长度/boot/calibration/restore；160个单bit损坏与旧DRDY注入 PASS | 芯片CRC不覆盖STATUS；仍需状态/复位检查。PGA共模约束保留，未将E1先验归因PGA |

<a id="comparison-tables"></a>

## 分阶段的原始对照表 / Measurements by iteration

每个展开项是当时一次实验或对照，保留其判据、分母与缺测标记。不同镜像和会话的数字不能相加。 / Expand an experiment for its original metrics. Each table uses the source image, window and acceptance criteria linked with it. They do not form a single dataset.

<details>
<summary>2026-11-25 当前镜像实测 / Measurements on the current image</summary>

[原始上下文 / Original context](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/README.md#L168)

| 实测 / Recording | 结果与范围 / Result and scope |
| --- | --- |
| [VOLT1 USB＋BLE 600秒](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/artifacts/cdc_recovery_acceptance_20261116/bss-volt600-01/bss-volt600-01.gate.json)；[指标](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/artifacts/cdc_recovery_acceptance_20261116/bss-volt600-01/bss-volt600-01.metrics.json) | **600.000132044秒；PASS_CASE_SCREEN**。Capture/USB/BLE **61.279399/61.278351/60.975142 FPS**。USB 36,767帧连续；BLE 36,585帧、缺182，**182/(36,585+182)=0.495009% ≤2.5%**。 / USB is internally consecutive; BLE meets the normal FULL numeric gap budget for this case. |
| [50次模式切换](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/artifacts/cdc_recovery_acceptance_20261116/bss-final50-01/bss-final50-01.gate.json)；[原summary](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/artifacts/cdc_recovery_acceptance_20261116/bss-final50-01/bss-final50-01.summary.json) | **50 attempted/applied/verified**，observer均闭合；50步均有匹配BLE帧，每步首个USB验证帧也在BLE实收。完整BLE窗口106帧，其中104帧匹配50步身份；CAP8恢复已查询核验。 / All 50 transitions have USB and BLE evidence; the 106-frame observer window is distinct from the 104 step-matched frames. |
| [SHORT保护快照](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/artifacts/cdc_recovery_acceptance_20261116/bss-short-01/bss-short-01.gate.json) | **SHORT_V2_FULL_SNAPSHOTS_OBSERVED**；120.000141602秒CAP8窗口，BLE实收117个可信完整快照，116个相邻host完整接收间隔最大**1.035322708秒**。REL恢复与CAP8恢复均query verified。 / These are host receipt intervals, not an over-air timing or future delivery guarantee. |

</details>


<details>
<summary>原始32配置矩阵 / Original 32-configuration matrix</summary>

[原始上下文 / Original context](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/README.md#L187)

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

</details>


<details>
<summary>参考夹具 / Reference fixture</summary>

[原始上下文 / Original context](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/README.md#L211)

| 窗口 / Window | 点 / Cell | 有效 / 无效 · valid / invalid | LC均值 / Mean LC (pF) | P05–P95 (pF) | 差值均值，同排 / 同列 / 全部悬空 · mean difference, row / column / all (pF) |
|---|---|---:|---:|---:|---:|
| CAP8 | S4D4 | 1,390 / 0 | 92.431562 | 92.423831–92.441396 | 32.494354 / 13.438154 / 32.493448 |
| CAP8 | S5D5 | 1,390 / 0 | 77.405972 | 77.397519–77.414335 | 17.468394 / 11.691997 / 17.467859 |
| SHORT | S4D4 | 1,409 / 0 | 92.421321 | 92.411281–92.431356 | 32.508467 / 13.434103 / 32.507510 |
| SHORT | S5D5 | 1,409 / 0 | 77.381926 | 77.372815–77.389611 | 17.468790 / 11.690489 / 17.468115 |

</details>


<details>
<summary>2026-11-21 screen21：保留配置域的typed metadata与正常缺口预算 / Typed metadata and the normal gap budget</summary>

[原始上下文 / Original context](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/README.md#L310)

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

</details>


<details>
<summary>2026-11-21 screen20：内部发送工作区对照未达标 / Internal TX working-copy comparison remains below target</summary>

[原始上下文 / Original context](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/README.md#L358)

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

</details>


<details>
<summary>2026-11-21 USB应用不读专项：BLE继续，USB排队有界丢弃 / Unread USB client: BLE continues while USB queues discard data</summary>

[原始上下文 / Original context](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/README.md#L421)

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

</details>


<details>
<summary>2026-11-21 screen19：周期合并已生效，容量拒绝仍阻塞验收 / Coalescing works; admission pressure still blocks acceptance</summary>

[原始上下文 / Original context](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/README.md#L468)

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

</details>


<details>
<summary>2026-11-21 screen19：周期合并已生效，容量拒绝仍阻塞验收 / Coalescing works; admission pressure still blocks acceptance</summary>

[原始上下文 / Original context](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/README.md#L488)

| 层 / Layer | 同一查询前后增量 / Within-query deltas | 解释 / Meaning |
|---|---|---|
| SNAPSRC | queued/periodic **+119**；first/miss/superseded **0** | source仍保留真实周期候选。 / Real periodic source candidates remain. |
| SNAPNET | queued/bleAccepted **+124**，derived **+5**，retry **+3**，obsolete **+8**；expiry/bleExpired/wifiExpired/bleFailed/policySkip **0** | 接纳的是Transport保护候选，不保证最终BLE class；旧普通帧淘汰仍单独计数。 / Candidate admission does not require protected BLE classification. |
| BLESNAP | queued/localComplete/staleNormal/deadlineMiss/admitMiss/failed/superseded **全部+0** | FULL普通完整帧已满足内部周期依据；没有重复保护超越。 / Ordinary FULL satisfies the internal periodic evidence without new protected overtaking. |
| BLEFLOW | attempts **+7,738**，capacityReject **+228**，intentionalSkip/probe **0**；两端full、ratePm1000 | 本地容量拒绝仍存在；不是随机2.5%丢弃，也不是已测RF丢包。 / Local admission pressure remains, without intentional decimation or measured RF loss. |

</details>


<details>
<summary>2026-11-21 screen18：保护机制首轮实测未通过 / First protected-output screen fails acceptance</summary>

[原始上下文 / Original context](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/README.md#L547)

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
| 恢复 / Recovery | 无fault/reset/WDT文本观察；[收尾只读查询](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/artifacts/cdc_recovery_acceptance_20261116/ble-snapshot-postscreen18-01.summary.json)同ELF/session，CAP8、rev6/boundary8761，guard ready1/quarantined0/rejected0。 / No such event observed within coverage; later query confirms CAP8 and unquarantined guard. |

</details>


<details>
<summary>2026-11-21 screen18：保护机制首轮实测未通过 / First protected-output screen fails acceptance</summary>

[原始上下文 / Original context](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/README.md#L563)

| 层 / Layer | 独立窗口秒 / Own window seconds | 实际增量 / Observed delta |
|---|---|---|
| SNAPSRC | 120.435041252–120.458481638 | queued/periodic +119，first/miss/superseded +0 |
| SNAPNET | 120.423265878–120.460826889 | queued/bleAccepted +129，derived +10，obsolete +5；retry/expired/bleExpired/wifiExpired/bleFailed/policySkip +0 |
| BLESNAP | 120.432523513–120.468759668 | queued/localComplete +239，staleNormal +118，deadlineMiss +1；failed/admitMiss/superseded +0 |
| BLEFLOW | 120.445937636–120.476209724 | attempts +7,574，capacityReject +99；intentionalSkip/probeSelected +0，两端stage=full、ratePm=1000 |

</details>


<details>
<summary>2026-11-21 screen17：普通日志让出 BLE 容量 / Ordinary logs yield BLE capacity</summary>

[原始上下文 / Original context](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/README.md#L668)

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

</details>


<details>
<summary>2026-11-21 criteria v2：BLE 可接受缺口与保护数据 / Acceptable BLE gaps and protected data</summary>

[原始上下文 / Original context](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/README.md#L709)

| 指标 / Metric | 必须保留的含义 / Required interpretation |
|---|---|
| 主动降低输出 / Intentional output decimation | 主动选取较少常规 DATA，采集不随之降速；单独计数，若按所有采集 seq 统计仍会形成缺口。不得没有同窗依据就从损失中扣除。 / Deliberately select fewer routine frames without slowing capture; report separately. These still appear as capture-sequence gaps unless a supported eligible-output denominator is defined. |
| 本地拒绝 / Local rejection | 队列满、guard 或策略在本地拒绝；不等于无线丢包，也不能与 PC 缺口相加为独立损失。 / Queue, guard or policy rejection is local, not measured air loss or an independent count to add to PC gaps. |
| PC 完整帧缺口 / PC complete-frame gap | 定义端点和身份内缺少 CRC/identity 正确的完整帧观察；不表示所有字节都未收到。 / Missing trusted complete-frame observations within defined endpoints/identity, not proof that every byte was absent. |
| ADC SPI CRC / ADC SPI CRC | 分母是检查读取，不是传输帧；现有启动资格 2.5% 独立保留，坏样本仍按既有有限重试/无效处理。 / Checked-read denominator, not transport frames. The separate existing 2.5% startup qualification remains; bad samples retain bounded retry/invalid treatment. |

</details>


<details>
<summary>2026-11-21 stability-02：保留原标准下的 10 分钟结果 / Preserved 10-minute result under the original criteria</summary>

[原始上下文 / Original context](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/README.md#L726)

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

</details>


<details>
<summary>2026-11-21 screen16：正式 DATA 优先于普通 LOG / screen16: formal DATA before ordinary LOG</summary>

[原始上下文 / Original context](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/README.md#L778)

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

</details>


<details>
<summary>2026-11-21 screen15：guard 与汇总周期的同版实测 / screen15: hardware observation of guard and summary cadence</summary>

[原始上下文 / Original context](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/README.md#L803)

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

</details>


<details>
<summary>2026-11-21 screen14：保留两片数量的 DATA 打包 / screen14: DATA packing with the same two-fragment count</summary>

[原始上下文 / Original context](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/README.md#L860)

| 指标 / Metric | screen13 balanced | screen14 packing |
|---|---:|---:|
| Duration | 120.000081842 s | 120.000089831 s |
| Capture / USB / BLE FPS | 65.789422 / 65.818399 / 61.504531 | 66.160374 / 66.174144 / 62.522868 |
| USB trusted / internal missing observations | 7,898 / 0 | 7,940 / 0 |
| BLE trusted / missing observations | 7,380 / 516 | 7,502 / 437 |
| ADC rejected / checked reads | 0 / 84,604 | 0 / 84,044 |
| USB / BLE X12 invalid points | 600 / 554 | 705 / 657 |
| Overall verdict | FAIL_BLE_THROUGHPUT | **FAIL_BLE_THROUGHPUT** |

</details>


<details>
<summary>2026-11-21 screen13：重启后的同版复测 / screen13: same-image observation after restart</summary>

[原始上下文 / Original context](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/README.md#L888)

| 指标 / Metric | screen12 | screen13 |
|---|---:|---:|
| Duration | 120.000092493 s | 120.000081842 s |
| Capture / USB / BLE FPS | 65.909847 / 65.922827 / 61.480926 | 65.789422 / 65.818399 / 61.504531 |
| USB trusted / internal missing observations | 7,911 / 0 | 7,898 / 0 |
| BLE trusted / missing observations | 7,377 / 537 | 7,380 / 516 |
| ADC rejected / checked reads | 0 / 84,791 | 0 / 84,604 |
| USB / BLE X12 invalid points | 682 / 616 | 600 / 554 |
| Overall verdict | FAIL_BLE_THROUGHPUT | **FAIL_BLE_THROUGHPUT** |

</details>


<details>
<summary>2026-11-16 screen12：实际 MTU517 与剩余吞吐差距 / screen12: negotiated MTU517 and remaining throughput gap</summary>

[原始上下文 / Original context](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/README.md#L920)

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

</details>


<details>
<summary>2026-11-16 screen11：CRC32/planner 优化与明确的低电平等待 / screen11: CRC32/planner optimization and explicit low hold</summary>

[原始上下文 / Original context](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/README.md#L953)

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

</details>


<details>
<summary>2026-11-16 历史 `a656d53` 连续 600 秒稳定性 / Historical `a656d53` continuous 600-second stability</summary>

[原始上下文 / Original context](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/README.md#L978)

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

</details>


<details>
<summary>2026-11-16 `-O2` 候选与安全检查 / `-O2` candidate and retained safety checks</summary>

[原始上下文 / Original context](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/README.md#L1018)

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

</details>


<details>
<summary>2026-11-16 screen09（历史）：元数据单遍解析 / Historical screen09: single-pass metadata parsing</summary>

[原始上下文 / Original context](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/README.md#L1043)

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

</details>


<details>
<summary>2026-11-16 screen08（历史）：分配生命周期修复与日志统计清理 / Historical screen08: allocation lifetime and unused log statistics</summary>

[原始上下文 / Original context](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/README.md#L1068)

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

</details>


<details>
<summary>2026-11-16 CPU 诊断首轮 screen07（历史） / First CPU diagnostic screen07 (historical)</summary>

[原始上下文 / Original context](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/README.md#L1095)

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

</details>


<details>
<summary>2026-11-16 CPU 诊断首轮 screen07（历史） / First CPU diagnostic screen07 (historical)</summary>

[原始上下文 / Original context](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/README.md#L1118)

| Role | 本角色独立跨度 / Own observation span (s) | switch-out Δrun (s) | Δrun / own span |
|---|---:|---:|---:|
| 0 BLE_TX | 120.095170 | 17.903690 | 14.9079% |
| 1 TRANSPORT | 119.846299 | 9.070485 | 7.5684% |
| 2 ASYNC_LOG | 120.182388 | 30.015103 | 24.9746% |
| 3 SERIAL_TX | 120.609289 | 11.569529 | 9.5926% |
| 4 BLE_CTRL | unknown: stale | unknown | unknown |
| 5 SERIAL_CTRL | 120.285279 | 0.853506 | 0.7096% |
| 6 WIFI_CTRL | unknown: not_observed | unknown | unknown |

</details>


<details>
<summary>2026-11-16 首轮双通路筛查 / First dual-link screen</summary>

[原始上下文 / Original context](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/README.md#L1138)

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

</details>


<details>
<summary>2026-11-16 第二轮筛查与 CRC 定位 / Second screen and CRC investigation</summary>

[原始上下文 / Original context](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/README.md#L1160)

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

</details>


<details>
<summary>历史 Serial/JTAG 修复与失败基线 / Historical Serial/JTAG repairs and failed baselines</summary>

[原始上下文 / Original context](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/README.md#L1283)

| 已完成筛查 / Completed screen | Capture FPS | USB FPS；可信完整帧 / trusted complete frames | BLE FPS；可信完整帧 / trusted complete frames | USB / BLE 缺失序号 / missing sequences | 原判定 / Recorded outcome |
| --- | ---: | ---: | ---: | ---: | --- |
| screen-03：`4b665ac`，120 s | 70.663301 | 61.422020；7,371 | 39.968146；4,796 | 1,120 / 3,687 | FAIL |
| screen-04：`d314888`，120 s | 69.013312 | 42.820965；5,138 | 43.300331；5,195 | 3,143 / 3,082 | FAIL |

</details>


<details>
<summary>0d8f60f 历史失败基线（原记录保留） / Historical 0d8f60f failed baseline (original record retained)</summary>

[原始上下文 / Original context](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/README.md#L1319)

| 项目 / Item | 已核实内容 / Verified information |
| --- | --- |
| 烧录基线源码 / Flashed baseline source | `0d8f60fd9c29b9ce12e64ed2e6534ee37db41704`；本文档起点 HEAD / documentation starting HEAD: `334fb2a081d70731985a6c2d5c2b6f02548ec56e` |
| 已烧录 app / Flashed app | 编译版本 / compiled version `0d8f60f-dirty`，1,552,752 bytes；dirty 为文档/证据，源码输入 hash 独立保存。 / Dirty entries were documentation/evidence; source-input hashes were saved separately. |
| app SHA256 | `232ca3976f6b8ea38efd5e09de8279f35c9bfbbd0c605c8dd84e5169ddd1dbaa` |
| 实际环境 / Tested environment | ESP-IDF **v5.5.5**；ESP32-S3 rev0.2，Flash 16 MB，PSRAM 8 MB；实机 ADS1262，ID `03`。 / Actual device: ESP32-S3 rev0.2, 16 MB flash, 8 MB PSRAM, ADS1262 with ID `03`. |
| 构建与 host 回归 / Build and host regression | 实际IDF构建通过；最终47个host测试入口全部exit 0，另保留早期42入口清单。不是47项硬件验收。 / The actual IDF build passed; all 47 final host test entry points exited 0, with the earlier 42-entry manifest retained. These are not 47 hardware acceptance tests. |
| 当前结论 / Current conclusion | 启动选速、配置身份及有界局部CRC失效已有证据；**双输出32配置仅31项采满120s；VOLT1在15.065s触发CPU0 IDLE0任务看门狗，压力FAIL。运行中仍有ADC SPI CRC拒绝，不能由启动≤2.5%资格推导长期错误率已验收；USB尾截断仍存在，绝对精度未验收。** / Evidence covers startup clock selection, configuration identity, and bounded local CRC rejection. **Only 31 of 32 dual-output configurations completed 120 s; VOLT1 triggered the CPU0 IDLE0 task watchdog after 15.065 s and failed the stress test. Runtime ADC SPI CRC rejections remain; the ≤2.5% startup qualification does not establish an accepted long-term error rate. USB tail truncation remains, and absolute accuracy is not accepted.** |

</details>


<details>
<summary>逐行 C/V/R、参考与辅助事务 / Per-row C/V/R, reference, and auxiliary transactions</summary>

[原始上下文 / Original context](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/README.md#L1506)

| 模式 / Mode | 测量与输出 / Measurement and output | 参考与激励命令 / Reference and excitation commands |
| --- | --- | --- |
| CAP / C | 双FDC，LC总电容，pF；`C/D/K`。 / Dual FDC, total LC capacitance in pF; C/D/K. | INTREF OFF，不主动施加REFOUT。 / INTREF OFF, no intentional REFOUT excitation. |
| VOLT / V | ADS ADC1节点电压，µV；`V/D/P/K`。 / ADS ADC1 node voltage in µV; V/D/P/K. | INTREF OFF，AVDD/AVSS reference，需合法rail模型。 / INTREF OFF, AVDD/AVSS reference, valid rail model required. |
| RES / R | ADS ADC1、参考电阻与REFOUT；线上整数mΩ；`R/D/P/K`。 / ADS ADC1, reference resistor and REFOUT; integer mΩ on the wire; R/D/P/K. | 有意开启INTREF/REFOUT；参考相对AVSS，不是默认对地2.5V。 / Intentional INTREF/REFOUT; reference is relative to AVSS, not automatically 2.5V to ground. |
| ROWMODES / MIXED | 各物理S行有自己的模式/单位；完整8项profile异构时使用`M/MR/K`，即使活动行恰好只有一种模式。只有完整8项同质profile才走单模式分支。 / Each physical S row retains its mode/units. A heterogeneous full eight-entry profile uses M/MR/K even if all active rows happen to share one mode. Only a homogeneous full eight-entry profile uses the single-mode branch. | 帧内按CAP、VOLT、RES分组，保留S行位置和分组时间/偏斜，不是全点同时采样。 / Groups run in CAP, VOLT, RES order, retaining S-row positions and group timing/skew; points are not simultaneous. |

</details>


<details>
<summary>启动并行选速 / Parallel startup qualification</summary>

[原始上下文 / Original context](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/README.md#L1531)

| 参数 / Parameter | 当前默认 / Current default |
| --- | --- |
| SPI候选 / Candidates | 配置上限以内：2MHz→1MHz每100kHz，950kHz→150kHz每50kHz，最后125kHz。 / Within the configured ceiling: 2MHz→1MHz in 100kHz steps, 950kHz→150kHz in 50kHz steps, then 125kHz. |
| 初筛/确认 / Screen/confirm | 每档400/1600次，各平分PGA gain1与bypass。 / 400/1600 attempts per candidate, each split equally between PGA gain1 and bypass. |
| CRC资格 / CRC qualification | 每profile/阶段各自≤2.5%：200中5次、800中20次，不能用阶段平均隐藏失败profile。 / Each profile/phase independently permits ≤2.5%: 5/200 or 20/800; averaging cannot hide a failing profile. |
| 覆盖/拒绝 / Coverage/rejection | 合格CRC字节末位0/1需足够覆盖；坏CRC不计valid/奇偶/raw范围。寄存器、RESET、STATUS、饱和、事务错误独立拒绝。 / Accepted CRC LSB0/1 need sufficient coverage; rejected CRCs do not count as valid/parity/raw-range samples. Register, RESET, STATUS, saturation, and transaction errors independently reject. |
| 总预算 / Total budget | SPI全候选共享30s单调期限，含调度/初筛/确认/恢复，预留500ms cleanup；原I²C初始化总时长另计。 / All SPI candidates share a 30s monotonic deadline including scheduling/screening/confirmation/restore, reserving 500ms for cleanup; total I²C startup duration is separate. |

</details>


<details>
<summary>已完成的0d8f60f窗口 / Completed 0d8f60f runs</summary>

[原始上下文 / Original context](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/README.md#L1659)

| 实录 / Capture | 时长、完整帧、速率 / Duration, complete frames, rate | 结果与限制 / Result and limit |
| --- | --- | --- |
| boot-03 | SPI requested/actual 900/909kHz；SPI/I²C overlap 804,751µs | screen CRC 2/200、4/200；confirm 10/800、11/800，各≤2.5%；不是精度PASS。 / Each pass meets 2.5%, not accuracy acceptance. |
| volt1-02 | 139.650s；13,042 V帧；capture≈93.57、received≈93.39FPS | MFAULT/协议CRC/identity/结构错误0，缺25帧；661个X02/631帧，8296个X12。 / Zero MFAULT/protocol CRC/identity/structure errors, 25 gaps; 661 X02 points in 631 frames, 8296 X12 points. |
| volt8-02 | 139.890s；2534 V帧；≈18.12FPS | 协议CRC/identity/缺口/MFAULT为0；760个X02/402帧，3199个X12。 / Zero protocol CRC/identity/gaps/MFAULT; 760 X02 points in 402 frames, 3199 X12 points. |
| res8-02 | 139.662s；1391 R帧；≈9.96FPS | 协议CRC/identity/缺口/MFAULT为0；S1D1/S8D8有51/2个X02，有效均值10164.95/10166.69Ω，绝对精度未验收。 / Zero protocol CRC/identity/gaps/MFAULT; S1D1/S8D8 have 51/2 X02 points, valid means 10164.95/10166.69Ω; absolute accuracy not accepted. |
| cap8-02 | 139.849s；1590 C帧；≈11.37FPS | 全点有效；两电容点σ=0.01568/0.01262pF、峰峰=0.16134/0.13653pF，未空载归零/裸33pF校准。 / All points valid; two capacitor points have those σ/peak-to-peak values; no unloaded zero/bare-33pF calibration. |
| mixed8-03 | 139.699s；777 M帧；≈5.56FPS；RCCCCCCR | 协议CRC/identity/缺口/MFAULT为0；S1D1和两电容点全有效，S8D8有3个X02；旧profile不同于新CVRCVRCV。 / Zero protocol CRC/identity/gaps/MFAULT; S1D1 and capacitor points valid, S8D8 has 3 X02; this older profile differs from new CVRCVRCV. |
| switch50-02 | 50次CAP/RES，每次3帧 / 50 CAP/RES changes, 3 frames each | CONTROL_PASS_MEASUREMENT_UNCALIBRATED；身份通过，不替长期稳定性/精度。 / Identity passes, not long-run stability or accuracy acceptance. |

</details>


<details>
<summary>0d8f60f：600秒与32配置失败基线 / 0d8f60f: 600-second run and failed 32-configuration baseline</summary>

[原始上下文 / Original context](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/README.md#L1681)

| 测试 / Test | 目标窗口 / Target window | 完成结果 / Completed result |
| --- | --- | --- |
| [stability-volt1-01](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/artifacts/full_benchmark_20261113/stability-usb-independent-review.json) | VOLT/1 row，600s，USB-only | 实际窗口600.000s（离线分析600.000063s），55,773完整USB帧；达到目标窗口，但有数据缺口与无效点。 / Actual window 600.000 s (offline analysis: 600.000063 s), 55,773 complete USB frames; required duration met with receive gaps and invalid points. |

</details>


<details>
<summary>0d8f60f：600秒与32配置失败基线 / 0d8f60f: 600-second run and failed 32-configuration baseline</summary>

[原始上下文 / Original context](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/README.md#L1685)

| 稳定性指标 / Stability metric | 实际结果及口径 / Observed result and scope |
| --- | --- |
| 身份与范围 / Identity and range | session=`0DCB1029EC3CCA0D`，boot=7，rev=71，seq100309–156162；未跨boot合并。 / Same session, boot 7 and rev 71, sequence 100309–156162; no cross-boot aggregation. |
| 速率 / Rates | device capture **93.09094 FPS**；USB实收 **92.95469 FPS**。前者按序号进展/设备时间差，后者按完整帧数减1/首末K接收时间差。 / Device capture **93.09094 FPS**, USB receive **92.95469 FPS**; sequence progression/device-time span versus (complete frames−1)/first-to-last K receipt span. |
| 链路与运行 / Link and runtime | ASCII CRC=0、identity错误=0、MFAULT=0、reset=0；**缺81帧**，不能称零丢失。 / Zero ASCII CRC/identity errors, MFAULT, or reset; **81 missing frames**, not lossless reception. |
| 测点质量 / Point quality | 共446,184点；有效407,974，无效38,210，其中X02=2,754、X12=35,456；17点fresh/acquired=0真实保留，未补零或改成已采集。 / 446,184 points: 407,974 valid and 38,210 invalid, comprising 2,754 X02 and 35,456 X12; 17 points retain fresh/acquired=0, without zero-filling or false acquisition claims. |
| 内存 / Memory | 起止PERF非尾字段free heap同为8,343,552 bytes；v2完整STK50确认生命周期minimum=8,336,952 bytes，PERF尾字段heapMin保留unknown。不是CPU或最大连续块测量。 / Nonterminal free-heap fields in endpoint PERF queries both report 8,343,552 bytes; v2 complete STK50 confirms a lifetime minimum of 8,336,952 bytes, while PERF’s terminal heapMin remains unknown. This does not measure CPU or the largest contiguous block. |
| 验收边界 / Acceptance limit | 完成600s持续运行观察；ADC CRC仍有拒绝，精度状态NOT_EVALUATED。不能将ASCII CRC=0解释为ADC SPI零错。 / The 600 s runtime observation completed; ADC CRC rejections remain and precision is NOT_EVALUATED. Zero ASCII CRC errors do not mean zero ADC SPI errors. |

</details>


<details>
<summary>32配置吞吐与完整性 / Throughput and integrity for all 32 configurations</summary>

[原始上下文 / Original context](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/README.md#L1709)

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

</details>


<details>
<summary>提交、背压与ADC计数窗口 / Submission, backpressure, and ADC counter windows</summary>

[原始上下文 / Original context](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/README.md#L1814)

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

</details>


<details>
<summary>内存统计与分析器更正 / Memory statistics and analyzer correction</summary>

[原始上下文 / Original context](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/README.md#L1860)

| 指标 / Metric | 实测与限制 / Observation and limit |
| --- | --- |
| 600s USB-only heap | 完整STK50采集窗最小free=8,343,552 B，生命周期minimum=8,336,952 B；起止PERF的非尾字段free相符。 / Complete capture-window STK50 reports minimum observed free heap 8,343,552 B and lifetime minimum 8,336,952 B; the nonterminal free-heap fields in endpoint PERF queries agree. |
| 31完整窗口 heap / 31 complete windows | 每窗最小observed free范围8,340,140–8,341,148 B；生命周期minimum最终8,336,464 B。采集窗接受4,284条完整STK50、拒绝189条截断布局。 / Per-window minimum observed free heap ranges from 8,340,140 to 8,341,148 B; the final lifetime minimum is 8,336,464 B. Capture windows contain 4,284 accepted complete STK50 records and 189 rejected incomplete layouts. |
| 栈余量 / Remaining stack | 完整STK50观测到的生命周期最低余量：log12,792、transport3,204、bleTx1,948、bleCtrl5,236、serialCtrl1,808 B；USB独立查询txStackMinBytes=3,092 B。不是每个任务/瞬时CPU采样。 / Observed lifetime low-water remaining bytes from complete STK50: log12,792, transport3,204, bleTx1,948, bleCtrl5,236, serialCtrl1,808; separate USB queries report txStackMinBytes=3,092 B. These do not measure every task or instantaneous CPU load. |
| 未测 / Not measured | CPU利用率、最大连续heap块、完整raw ADC样本与示波器模拟节点仍NOT_RUN/不可由现有帧重建。没有据此宣称无内存泄漏或物理安全。 / CPU utilization, largest contiguous heap block, complete raw ADC samples, and oscilloscope analogue nodes remain NOT_RUN or unreconstructable from these frames. These results do not establish absence of memory leaks or physical safety. |

</details>


<details>
<summary>失败、重测与工具边界 / Failures, retries, and harness limits</summary>

[原始上下文 / Original context](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/README.md#L1871)

| 窗口 / Window | 结果与证据含义 / Result and meaning |
| --- | --- |
| `preflight-01` | 工具将TX回复错期待为`TX`标签，实际是`ACK,cmd=TX`，导致准备超时；修正后只读查询通过。原ERROR保留，不归为固件失败。 / The harness expected a TX tag instead of the actual `ACK,cmd=TX`, causing a preparation timeout. Corrected read-only queries passed. The original ERROR remains and is not a firmware failure. |
| `suite-01-volt1` / `suite-01-volt2` | 准备回复超时，正式采集0s；当时仍持续收到原始字节。没有把0s当120s，也没有因端口仍有数据而忽略控制回复丢失。 / Preparation replies timed out; the formal capture was 0 s while raw bytes continued. These are not 120 s captures, and continuing data does not excuse lost control replies. |
| `retry-01-volt2` / `suite-01-volt3` / `suite-01-res1` | 采满120s，但分别有10/2/9个K尾身份不完整帧被拒收；ASCII CRC可仍通过，因为K身份尾部不属于其测量payload CRC覆盖。原始截断、缺失字段和collector ERROR均保留；不是可用科学帧，也不能直接证明设备配置域改变。 / Completed 120 s, but 10/2/9 frames respectively were rejected for incomplete K-trailer identity. ASCII CRC can still pass because the identity suffix is outside the measurement-payload CRC. Original truncation, missing fields, and collector ERROR remain; these are not usable scientific frames or direct proof of a device configuration change. |
| `suite-01-mixed1` | 120s完成；旧工具误期待C帧，独立复核真实M/MR/K通过，属于保留原ERROR的harness假阳性。 / Completed 120 s. The older harness wrongly expected C; independent validation of actual M/MR/K passed. This is a harness false positive whose original ERROR is retained. |
| `retry-01-volt1` | **真实压力FAIL**：正式窗口15.065057s，原生task WDT报告CPU0的IDLE0未及时喂狗；当时任务快照CPU0=`btController`、CPU1=`sensorarrayScan`。该快照不能证明连续霸核或根因。PANIC未开启，未观察到MFAULT/reset；随后CAP8恢复查询通过，停止压力复测。 / **Real stress FAIL**: the formal window lasted 15.065057 s; native task WDT reported CPU0 IDLE0 missing its feed. The task snapshot showed CPU0=`btController` and CPU1=`sensorarrayScan`; this alone does not prove continuous CPU monopolization or root cause. PANIC was disabled and no MFAULT/reset was observed. CAP8 recovery queries then passed and stress retesting stopped. |

</details>


<details>
<summary>正式 ADC 末位读取修复 / Scientific ADC final-bit repair</summary>

[原始上下文 / Original context](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/README.md#L1951)

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

</details>


<details>
<summary>BLE SDK 异步返回观测 / BLE SDK callback observations</summary>

[原始上下文 / Original context](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/README.md#L1981)

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

</details>


<details>
<summary>当前阶段：d314888，screen-04 仍 FAIL / Current stage: d314888, screen-04 still FAIL</summary>

[原始上下文 / Original context](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/load_reliability_fix_20261113.md#L14)

| 已完成筛查 / Completed screen | Capture FPS | USB FPS；可信完整帧 / trusted complete frames | BLE FPS；可信完整帧 / trusted complete frames | USB / BLE 未观察到序号 / missing sequences | 原判定 / Recorded outcome |
| --- | ---: | ---: | ---: | ---: | --- |
| screen-03，`4b665ac`，120 s | 70.663301 | 61.422020；7,371 | 39.968146；4,796 | 1,120 / 3,687 | FAIL |
| screen-04，`d314888`，120 s | 69.013312 | 42.820965；5,138 | 43.300331；5,195 | 3,143 / 3,082 | FAIL |

</details>


<details>
<summary>fd0190a 历史筛查与当时状态 / Historical fd0190a screening and status at that stage</summary>

[原始上下文 / Original context](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/load_reliability_fix_20261113.md#L65)

| 版本/证据项 / Version or evidence | 当前记录 / Current record |
| --- | --- |
| 原审计比较基线 / Original comparison baseline | `8045e9e9ec9599533c52c15dfcb6002f79fd15f1`，不是本轮板上二进制身份 / not this round's device image identity |
| 失败实机基线源码 / Failed HIL baseline source | `0d8f60fd9c29b9ce12e64ed2e6534ee37db41704` |
| 本轮 host 修复记录 HEAD / HEAD in repair records | `1f82b73a1019930f419492e3e742a96bd43f374e` + dirty；逐文件 hash、dirty hash、命令/退出码在各 manifest / per-file and dirty hashes, commands and exits are in each manifest |
| 工具链 / Toolchain | ESP-IDF v5.5.5，ESP32-S3；实际首次构建命令见下方 / actual first build command below |
| 修复 commit / Repair commit | `fd0190a3a175938e92e702a111ae9d35982d678f` |
| 新 app SHA / New app SHA | `4107906941556b24a9cfb6c15686943f937a7a548b47a3c43bd2eb30cbddb1ad`；1,561,408 B；build version `fd0190a-dirty`，见 repair-03 build manifest / see repair-03 build manifest |
| 新镜像烧录/实机 / New image flash/HIL | app-only 烧录/启动观察由主任务进行；HIL 待验证，结果另追加 / main task owns app-only flash/startup observation; HIL validation pending, results to be appended |

</details>


<details>
<summary>已确认缺陷与修改 / Confirmed defects and changes</summary>

[原始上下文 / Original context](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/load_reliability_fix_20261113.md#L91)

| 问题 / Issue | 证据、函数和根因 / Evidence, function and cause | 修改与验证 / Change and validation |
| --- | --- | --- |
| USB 半记录与回复过早放弃 / Partial USB records and premature reply expiry | `sensorarraySerialTxTask` 原按 64 B、zero-wait 分段；100 Hz 下两次失败等待就耗尽 20 ms。真实旧源码在模拟主机 30 ms 才腾出空间时留下 64 B CONFIG 前缀。 / The old owner used 64 B zero-wait chunks; two failed waits consumed 20 ms at 100 Hz. Actual old source left a 64 B CONFIG prefix when the simulated host freed space at 30 ms. | 完整记录驱动入队，普通/优先原始入队期限 250/1000 ms；实际 IDF BYTEBUF 与应用生产代码回归。 / Whole-record driver admission with original enqueue deadlines of 250/1000 ms; executed IDF BYTEBUF and application regressions. [证据 / Evidence](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/artifacts/load_reliability_fix_20261113/serial-io-host-02.json) |
| 过期队列无新通知便永久滞留 / Expired queue backlog stranded without a new notification | 初版 retained item 过期后可能 sleep，旧通知已清空，队列仍有 3 个优先槽。独立反例 exit 1。 / The first repair could sleep after a retained item expired, with the old notifications consumed and three priority slots still queued; independent reproduction exited 1. | `drainProgress` 同时包括取队列和过期释放；sleep 前再次查看队列，不依赖未来 producer。 / Count both dequeue and expiry release as drain progress; inspect queues again before sleeping without requiring a future producer. [失败 / Failure](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/artifacts/load_reliability_fix_20261113/serial-io-expiry-failure-01.json)、[修复 / Fix](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/artifacts/load_reliability_fix_20261113/independent-usb-expired-backlog-fixed.json) |
| CPU0 自有任务持续 ready / Continuously ready application tasks on CPU0 | 多个 drain、快速成功/失败、BLE 分片和控制洪流路径没有共享的有界让出；Wi-Fi 连续 UDP、零包、立即 socket error 同样可持续 ready。 / Several drain, fast success/failure, BLE fragment and control-flood paths lacked a shared bounded yield; continuous UDP, zero-length receives and immediate socket errors could do the same. | 七 worker 共享真实 idle 进度观测，在安全边界用工作预算请求一次 tick 阻塞，原帧/slot/身份继续保留。 / Seven workers observe real idle progress and request one tick of blocking at safe work-budget boundaries while retaining frame/slot identity. [Host 证据 / Host evidence](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/artifacts/load_reliability_fix_20261113/work-budget-host-03.json)；目标适配、独立反汇编与完整构建已完成；screen-04 未见 WDT，但负载仍 FAIL，完整长测待验证 / target adaptation, independent disassembly and complete build are done; screen-04 observed no WDT but still failed load acceptance, with the full long suite pending. |
| RECOVER 结果不可查询，预算耗尽仍可能采下一帧 / Unqueryable RECOVER result and capture after exhausted restart budget | 原终态无最后结果缓存；budget 分支忽略 EnterSafe 返回，打印 safe，却保留 CAP active，下一帧可能重新建路由。 / No last-result cache; the budget branch ignored EnterSafe, printed safe and retained active CAP, allowing the next frame to rebuild its route. | ACK 栅栏、保留槽、同配置身份校验、RECOVERY?；故障后 NONE/DEGRADED，原错/cleanup 分开，真实下一帧入口拒绝。 / ACK gate, reserved terminal slot, configuration fencing and RECOVERY?; NONE/DEGRADED after failure, separate original/cleanup errors, and rejection by the actual next-frame entry. [证据 / Evidence](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/artifacts/load_reliability_fix_20261113/recovery-control-host-02.json) |
| RESTART pending 覆盖和 ACK 失败仍执行 / RESTART overwrite and execution despite failed ACK | 真实旧 Post/Take/失败回调复现：接受 id1/id2 后仅取 id2；ACK id3 失败仍取走；id4 可在 ACK 前取走。 / Actual old Post/Take/failure callback reproduced id1 being replaced by id2, id3 remaining executable after ACK failure, and id4 being taken before ACK. | 独立请求/结果槽，busy 拒绝、不覆盖，ACK 本地成功后执行；RESTART? 无副作用查询。 / Independent request/result slot, busy rejection without overwrite, execution after local ACK admission, and side-effect-free RESTART?. [原反例 / Counterexample](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/artifacts/load_reliability_fix_20261113/restart-control-before-01.json) |
| 初版 RESTART 漏 Serial / Initial RESTART repair omitted Serial | PublishLifecycle 只处理 BLE/Wi-Fi；网络全关时 ESP_OK 是 no-op，不能当终态入队。 / PublishLifecycle handles BLE/Wi-Fi only; with both disabled, ESP_OK is a no-op rather than terminal admission. | 显式 Serial priority + 网络 Observed API，记录每路错误；16 组合独立生产路径复核。 / Explicit Serial priority submission plus an observed network API, retaining each sink error; 16 combinations independently reviewed. [失败 / Failure](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/artifacts/load_reliability_fix_20261113/independent-restart-serial-missing-review.json)、[修复 / Fix](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/artifacts/load_reliability_fix_20261113/independent-restart-routing-review.json) |

</details>


<details>
<summary>实际执行与保留失败 / Executed validation and retained failures</summary>

[原始上下文 / Original context](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/load_reliability_fix_20261113.md#L194)

| 验证 / Validation | 结果与证据 / Result and evidence |
| --- | --- |
| USB 旧源码/过期 backlog 反例及修复 / Old USB and expiry-backlog reproduction and repair | 预期失败与修复 PASS 都保留 / expected failures and repaired passes retained: [serial-io-host-02.json](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/artifacts/load_reliability_fix_20261113/serial-io-host-02.json) |
| 七 worker、真实 drain/分片/UDP 函数 / Seven workers and actual drain/fragment/UDP functions | Host PASS，25 场景，含缺 idle、注册失败、错误 DRAM/对齐、跨核年龄、回绕、无限补队列与控制洪流 / 25 host scenarios pass, including absent idle, registration failure, invalid DRAM/alignment, cross-core age, wrap, replenished queues and control floods: [work-budget-host-03.json](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/artifacts/load_reliability_fix_20261113/work-budget-host-03.json) |
| RECOVER/RESTART/查询/真实 sink 路由 / Recovery, restart, queries and actual sink routing | 6 个生产模块、19 个 main 函数/实际前置分支、真实 Queue/Observed/GuaranteedDrain；6 个相关测试入口 exit 0 / six production modules, 19 main functions/actual prefixes, real Queue/Observed/GuaranteedDrain; six related test entries exit 0: [recovery-control-host-02.json](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/artifacts/load_reliability_fix_20261113/recovery-control-host-02.json) |
| 全部 host 入口首次集合 / First complete host batch | **50 个入口：49 exit 0、1 exit 1**。失败为 frontend-wait harness 缺四个新 query formatter 声明，不掩盖为生产 PASS / **50 entries: 49 exit 0, one exit 1**; the frontend-wait harness lacked four new query formatter declarations: [原清单 / Original manifest](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/artifacts/load_reliability_fix_20261113/repair-01-tests-manifest.json) |
| frontend-wait harness 适配 / Harness adaptation | 保留旧断言，补四个边界 stub；实际 held/null-context query dispatch 各自验证且无硬件副作用，重跑 exit 0 / old assertions retained; four boundary stubs added, actual held/null-context dispatch verified without hardware effects; rerun exit 0: [修复记录 / Repair record](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/artifacts/load_reliability_fix_20261113/frontend-wait-harness-fix-01.json) |
| 完整目标构建第一次 / First complete target build | **FAIL exit 2**；`source /home/csc/esp/esp-idf-v5.5.5/export.sh && idf.py reconfigure build`，FreeRTOS include 与 lock-free 断言错误 / FreeRTOS inclusion and lock-free assertion errors: [manifest](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/artifacts/load_reliability_fix_20261113/repair-01-build-manifest.json)、[原始日志 / Raw log](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/artifacts/load_reliability_fix_20261113/repair-01-idf.txt) |
| 全部 host 第二次集合（历史） / Second complete host batch (history) | **50 个入口：49 exit 0、1 exit 1**；SerialIo host shim 未提供新增 esp_attr.h，随后补齐并重跑。不是目标构建失败 / **50 entries: 49 exit 0, one exit 1**; the SerialIo host shim lacked the new esp_attr.h and was then adapted; this was not a target-build failure: [manifest](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/artifacts/load_reliability_fix_20261113/repair-02-tests-manifest.json)、[失败日志 / Failure log](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/artifacts/load_reliability_fix_20261113/repair-02-test_serial_io.txt) |
| 目标适配后未提交构建（历史） / Adapted pre-commit target build (history) | **PASS exit 0**，version `1f82b73-dirty`；保留该记录，不冒充最终 commit 镜像 / record retained, not mislabeled as the final committed image: [repair-02 build](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/artifacts/load_reliability_fix_20261113/repair-02-build-manifest.json) |
| 全部 host 最终集合 / Final complete host batch | **50/50 exit 0**；运行时 HEAD 为提交前的 1f82b73+dirty，但 166 个构建输入及 50 个测试输入 hash 均与 fd0190a 构建完全相同 / execution HEAD was pre-commit 1f82b73+dirty, but all 166 build-input and 50 test-input hashes match the fd0190a build: [repair-03 tests](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/artifacts/load_reliability_fix_20261113/repair-03-tests-manifest.json) |
| 最终已提交目标构建 / Final committed-source target build | **PASS exit 0**，`fd0190a-dirty`，app 1,561,408 B；源码输入与 repair-02 build/repair-03 tests 相同，app SHA 随构建版本等元数据改变而不同 / production inputs match repair-02 build and repair-03 tests; the app SHA differs with build-version metadata: [repair-03 build](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/artifacts/load_reliability_fix_20261113/repair-03-build-manifest.json)、[日志 / Log](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/artifacts/load_reliability_fix_20261113/repair-03-idf.txt) |
| 新采集工具 PTY 回归 / New capture-tool PTY regression | 8 项 PASS、无硬件；不是 HIL / eight passes, no hardware; not HIL: [记录 / Record](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/artifacts/load_reliability_fix_20261113/load-capture-pty-01.json) |
| 新镜像 USB+BLE、无主机/恢复、Wi-Fi 负载 / New-image USB+BLE, absent host/recovery and Wi-Fi load | **待验证；启动观察不等于压力 HIL 通过 / pending validation; startup observation is not a load-HIL pass** |

</details>


<details>
<summary>首次修复镜像的实际结果</summary>

[原始上下文 / Original context](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/reference_fixture_hil_20261104.md#L18)

| 采集标签 | 实际窗口 / 完整帧 | CRC / 身份 / 序号缺口 | 结果 |
|---|---:|---|---|
| `repaired-cap8-01` | 139.840 s / 1608 | 0 / 0 / 0 | CAP 数据链通过此窗口；33 pF 精度 BASELINE_REQUIRED |
| `repaired-res8-01` | 1.111 s / 13 | ASCII 0 / 0 / 0 | FAIL：ADC SPI CRC `0x109`，seq4231，未达到 120 s |
| `repaired-cap1-recovery-01` | 139.832 s / 10149 | 0 / 0 / 0 | RES 故障后命令可用，CAP 恢复成功，约 72.58 FPS |

</details>


<details>
<summary>最终实机结果及限制</summary>

[原始上下文 / Original context](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/reference_fixture_hil_20261104.md#L111)

| ebabe7e 场景 | 实际结果 | 结论边界 |
|---|---|---|
| CAP8 `release-cap8-01` | 139.867 s / 1615帧，CRC32/identity/gap=0 | 数据链筛查通过；33pF BASELINE_REQUIRED |
| MIXED8 `release-mixed8-01` | 139.632 s / 807帧，CRC32/identity/gap=0，无MFAULT | 夹具评估FAIL：S8D8全部OPEN；未标精度PASS |
| 严格切换 `release-switch50-01` | 完成28次，第29次S1D1 X02 | FAIL，终态/身份本身正确 |
| 分项切换 `release-switch50-identity-01` | 50次控制身份PASS，200连续帧；25RES/25CAP | S1D1有效、S8D8仍OPEN；未判精度PASS，SPI仍有恢复的CRC错误 |
| CAP1+BLE并行 | Serial149.784 s；BLE140.001 s | BLE接收及两通路一致性已实测，USB有可计数数据缺口 |
| RES8 `release-res8-01` | 2.075 s / 26帧，seq18301 MATRIX_CRC/0x109 | FAIL，未达到2分钟；不能宣称ADS CRC根治 |
| 最终CAP恢复 | 22.332 s / 257帧，无CRC32/identity/gap/MFAULT | 仅恢复短窗，设备留在CAP8/DEBUG |

</details>


<details>
<summary>实际采集与首次故障</summary>

[原始上下文 / Original context](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/reference_fixture_recheck_20261113.md#L19)

| 标签（均以20261113-开头） | 实际记录 | 结果 |
|---|---|---|
| contact-fixed-cap8-01 | 139.835 s，1614完整C，seq8656990–8658603 | CRC32/identity/gap=0，全部点fresh/acquired有效；电容精度仍未建立 |
| contact-fixed-res8-internal-01 | 45.686 s，540完整R，seq8659026–8659565 | seq8659566 MATRIX_CRC/0x109；engine3288、entry2746、healthDue0，cleanup0 |
| res8-external-01 | 54.846 s，658完整R，seq8659723–8660380 | seq8660381 MATRIX_CRC/0x109；engine3948、entry3288、healthDue0，cleanup0；无内部rail刷新 |
| volt8-external-01 | 首个VOLT采样seq8660383失败，无完整V帧 | 已确认INTREF0/REFMUX24/source external；healthDue1且同seq健康回读err0，随后数据CRC失败，非健康回读失败 |
| cap8-external-recovery-01 | 139.816 s，1601完整C，seq8660385–8661985 | CRC32/identity/gap=0，无MFAULT；新电池样本有效 |
| res8-external-batteryoff-01 | 139.662 s，1873完整R，seq8663070–8664942 | 无MFAULT但仍有CRC和1个X02；严格evaluator为FAIL，不是可靠性PASS |
| final-cap-recovery-01 | 26.104 s，299完整C，seq8665164–8665462 | CRC32/identity/gap=0；已恢复CAP8，仅恢复验证，不称两分钟筛查PASS |

</details>


<details>
<summary>电阻、rail与CRC的区分</summary>

[原始上下文 / Original context](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/reference_fixture_recheck_20261113.md#L33)

| 配置 | S1D1有效值均值 | S8D8有效值均值 | 无效参考点 |
|---|---:|---:|---|
| 内部rail、周期电池开 | 9989.169 Ω（540点） | 9991.152 Ω（538点） | S8D8有2个X02；两点OPEN均0 |
| 外部rail、周期电池开 | 10167.366 Ω（657点） | 10169.230 Ω（656点） | S1D1有1个、S8D8有2个X02；两点OPEN均0 |
| 外部rail、周期电池关 | 10167.325 Ω（1873点） | 10169.305 Ω（1872点） | S8D8有1个X02；两点OPEN均0 |

</details>


<details>
<summary>用户授权的悬空口近似背景</summary>

[原始上下文 / Original context](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/reference_fixture_recheck_20261113.md#L55)

| 测点 | 原始总C均值 | 同行背景相减 | 同列背景相减 | 全60点背景相减 |
|---|---:|---:|---:|---:|
| S4D4 | 92.490262 pF | 32.409237 pF | 13.511588 pF | 32.408135 pF |
| S5D5 | 77.530541 pF | 17.449767 pF | 11.773416 pF | 17.448414 pF |

</details>


<details>
<summary>中间版本 e339a1d 的实际结果（保留失败，不作为最终验收）</summary>

[原始上下文 / Original context](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/frontend_startup_recovery_20261113.md#L40)

| 实测 | 结果与边界 |
|---|---|
| `boot-01`，ee8f972 | SPI 选中请求 750 kHz / 驱动实际 754 kHz；与 I²C 重叠 869793 µs。FDC presence 地址错误，不能算完整启动诊断 PASS；一条选中档初筛日志被穿插截断，不能补全 |
| `boot-02`，e339a1d | 选中请求 900 kHz / 实际 909 kHz，13 个候选；选中档 screen PGA/bypass CRC=1/200、0/200，confirm=14/800、18/800，各自 ≤2.5%。SPI Core0 与 I²C Core1 重叠 837621 µs。累计 133/4256 包含被淘汰档，不能当选中档错误率 |
| `boot-02` 前端证据 | 两颗 FDC 均两轮读出 5449/3055，I²C 345 kHz；查询观察到 known/present=7，ADS 探测125 kHz、恢复909 kHz、cleanup=0。仅末次快照，不证明此前每轮均成功。SPI worker 栈6144 bytes，最低余量2992 bytes |
| `cap8-01` | 139.810 s /1601 个完整帧，ASCII CRC、身份、序列缺口均0，约11.45 FPS。S4D4/S5D5 标准差0.00482/0.00546 pF、峰峰0.03513/0.03737 pF。绝对值含夹具/环境寄生，不与裸33 pF直接比较；没有执行归零或持久校准 |
| `rail-01` | 在 CAP/INTREF OFF 确认后使用用户 ADS 供电脚 DMM 的代表值，临时 RAM RAILCFG=3350000,-1795000 µV、span=5145000；source=external、uncertainty=unknown。用户读数范围不能变成更精确校准；没有 CAL=SAVE |
| `res8-01` | 139.714 s /1390 帧、约9.96 FPS，无 MFAULT；S1D1 1389有效+1个X02，S8D8 1388有效+2个X02，均无OPEN。有效均值10167.04/10168.72 Ω，不能以未知轨不确定度宣称10k精度通过。真实health128/256/384/512回读err=0，模式计数累计保留 |
| `volt8-01` | **FAIL**：46.882 s /924 完整帧后，seq7331 MATRIX_CRC/0x109；终帧2次CRC、1次重试、0次恢复。ASCII CRC和identity正确不代表ADC有效 |
| `volt1-01` | **FAIL**：5.446 s /532 完整帧后再次固件故障；约98 FPS短窗口不能代替两分钟稳定性验收 |
| `volt1-baseline-tail-01` | 旧版再现：34.191 s /3342完整帧后seq19617 MFAULT/0x109；同帧health engine=8064、modeEntry=4720、err=0。延迟2秒保留诊断尾部后仍未获全部context行，不能补成完整原始故障 |
| `mixed8-01` | 前置阶段RMERR/0x103，0秒采集；从DEGRADED直接请求混合行遭拒，不能报告混合运行失败时长 |
| `recovery-cap-01` | 显式MODE=CAP及ROWS=8恢复，CONFIG原子确认；FRONTENDS hold=0、known/present=7，ADS临时125 kHz后恢复909 kHz。电池有效且restore=ok。控制能恢复不代表CRC根因消失 |
| `mixed8-02` | CAP恢复后重试，139.648 s /786完整M帧，约5.63 FPS；CRC、身份、序列缺口和MFAULT均0。S1D1有3个X02，S8D8全有效；两电容点无失效，但绝对基线未校准 |
| `switch50-01` | 50次CAP/RES交替均完成，每次实收3帧并对CONFIG和header/trailer身份，CONTROL_PASS；所有25次S1D1快照有效。不把此短控制测试当精度或两分钟稳定性验收 |

</details>


<details>
<summary>当前烧录版 0d8f60f 的实测记录</summary>

[原始上下文 / Original context](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/frontend_startup_recovery_20261113.md#L72)

| 实测 | 结果与性能边界 |
|---|---|
| `boot-03` | session=0DCB1029EC3CCA0D、boot7；选中900 kHz/实际909 kHz，screen CRC=2/200、4/200，confirm=10/800、11/800，各自≤2.5%。SPI/I²C时间重叠804751 µs，worker栈最低余量3000/6144 bytes；三前端末次查询均present，125 kHz ADS探测后恢复909 kHz。选中档四段原始记录完整，其它个别日志截断不补齐 |
| `rail-02` | 新boot后再次核验CAP/INTREF OFF，再将相同用户代表值写入RAM；非持久校准，来源与不确定度不变 |
| `volt1-02` | 139.650 s、实收13042个完整V帧，协议CRC/identity/结构错误与MFAULT均0；共同序号范围缺25帧。按设备帧时间/序号推算capture约93.57 FPS、实收约93.39 FPS。旧版短失败窗口约97.79 FPS，不是同长度/同错误分布的严格性能A/B；没有设置永久采样限速 |
| `volt8-02` | 139.890 s /2534完整V帧，约18.12 FPS；协议CRC、身份、结构、序号缺口、MFAULT均0；存在被明确拒绝的ADC样本，不是全部点精度PASS |
| `res8-02` | 139.662 s /1391完整R帧，约9.96 FPS，协议CRC/身份/序号缺口/MFAULT均0。S1D1有51个X02、S8D8有2个X02，均无OPEN；有效均值10164.95/10166.69 Ω，标准差1.80/2.12 Ω。不能把模式不中断当作数据质量或10k精度通过 |
| `cap8-02` | 139.849 s /1590完整C帧，约11.37 FPS，全部点有效，协议CRC/身份/序号缺口/MFAULT均0。两电容点标准差0.01568/0.01262 pF、峰峰0.16134/0.13653 pF；仅报告相对稳定性，没有归零或绝对33pF校准 |
| `mixed8-03` | 139.699 s /777完整M帧，约5.56 FPS；协议CRC/身份/序号缺口/MFAULT均0。S1D1全部有效，S8D8有3个X02；两电容点全部有效。保持逐行模式，不改成固定四点 |
| `switch50-02` | 50次CAP/RES控制及每次3个实收帧身份核验全部通过，25次S1D1快照有效；CONTROL_PASS_MEASUREMENT_UNCALIBRATED，不能替代精度验收 |
| `final-state-01` | CAP/8行/USB DEBUG，session=0DCB1029EC3CCA0D、boot7、rev68、boundary27899；FRONTENDS known/present=7、hold/quarantine=0，ADS恢复909 kHz。电池valid=1、reason/restore=ok，validRun973/invalidRun7；heap最小8336952 bytes。未执行CAL=SAVE、NVS写入或push |

</details>


<details>
<summary>USB-OTG 与 BLE 优先级修复 / USB-OTG and BLE priority repair</summary>

[原始上下文 / Original context](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/usb_otg_ble_priority_20261114.md#L13)

| Evidence / 证据 | Function / 函数 | Change / 修改 | Verification / 验证 |
| --- | --- | --- | --- |
| USJ has no DMA; S3 OTG has internal DMA | new `sensorarrayUsbOtgOwnerInit`, descriptors | pinned TinyUSB 0.21.0~1; 2 vendor + CDC; internal aligned DMA buffers | target builds; HIL pending |
| Shared wire/log capacity could stall formal traffic | `sensorarraySerialOtgTxTask` (new) | separate endpoint state, CTRL/DATA/debug scheduling; same persistent payload pool | real C partial/no-host/epoch regressions |
| Reset interrupt precedes reset event processing | `sensorarrayUsbOtgOwnerPoll`, `ConsumeCommands` | processed-epoch fence, queue-empty proof, per-chunk checks | real C event-17 and reset-during-read regressions plus negative mutation |
| ESP_FAIL collided with pending sentinel; retry could falsely succeed | `sensorarraySerialIoInit` | independent done/result, repeat init checks actual result | real C failure/timeout/retry cases |
| CPU-only restart does not prove OTG DMA stopped | new `PrepareRestart`, `OwnerQuiesce`, emergency wrapper | final owner hardware-reset fence; no uninstall/free; bounded Core Reset escalation | host cases; final ELF/ROM handoff HIL pending |
| BLE DATA admission drops without fragment CRC errors | BLE task/queue/negotiation code | split DATA/LOG descriptors, retained CTRL/lifecycle capacity, actual negotiation reports | real C scheduling/reconnect/generation tests; HIL pending |
| CDC-only MFAULT would evade formal clients | new `sensorarrayTransportPublishRuntimeFault` | bounded priority USB and BLE lifecycle publication, optional CDC mirror | real publisher regression; HIL pending |
| New endpoint provenance must not falsify original streams | `UsbBulkPort`, `Collector`, benchmark tools | separate DATA/CTRL raw files and partial lines, exact interface matching | fake backend + actual encoder fixtures; Linux HIL pending |

</details>


<details>
<summary>已执行的恢复步骤 / Executed recovery steps</summary>

[原始上下文 / Original context](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/cdc_recovery_acceptance_20261116.md#L15)

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

</details>


<details>
<summary>完整双通路窗口 / Complete dual-link window</summary>

[原始上下文 / Original context](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/cdc_recovery_acceptance_20261116.md#L270)

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

</details>


<details>
<summary>各角色独立差分 / Independent per-role deltas</summary>

[原始上下文 / Original context](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/cdc_recovery_acceptance_20261116.md#L300)

| Role / taskNumber | ΔrunUs | Own spanUs | Δrun / own span |
|---|---:|---:|---:|
| 0 BLE_TX / 9 | 17,903,690 | 120,095,170 | 14.907918% |
| 1 TRANSPORT / 15 | 9,070,485 | 119,846,299 | 7.568431% |
| 2 ASYNC_LOG / 17 | 30,015,103 | 120,182,388 | 24.974627% |
| 3 SERIAL_TX / 8 | 11,569,529 | 120,609,289 | 9.592569% |
| 4 BLE_CTRL / 10 | unknown | unknown | stale historical sample |
| 5 SERIAL_CTRL / 16 | 853,506 | 120,285,279 | 0.709568% |
| 6 WIFI_CTRL / unknown | unknown | unknown | not_observed |

</details>


<details>
<summary>科学接收与各层计数 / Scientific receipt and layer counters</summary>

[原始上下文 / Original context](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/cdc_recovery_acceptance_20261116.md#L353)

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

</details>


<details>
<summary>科学接收与各层计数 / Scientific receipt and layer counters</summary>

[原始上下文 / Original context](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/cdc_recovery_acceptance_20261116.md#L375)

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

</details>


<details>
<summary>CPU owner 对照与结论边界 / CPU-owner comparison and limits</summary>

[原始上下文 / Original context](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/cdc_recovery_acceptance_20261116.md#L396)

| Role | screen08 ΔrunUs / own spanUs | screen07 ratio | screen08 ratio | Difference / percentage points |
|---|---:|---:|---:|---:|
| BLE_TX | 18,189,732 / 120,992,137 | 14.907918% | 15.033813% | +0.125895 |
| TRANSPORT | 9,125,833 / 120,906,284 | 7.568431% | 7.547857% | −0.020575 |
| ASYNC_LOG | 29,526,020 / 120,021,728 | 24.974627% | 24.600562% | −0.374065 |
| SERIAL_TX | 11,760,714 / 120,679,896 | 9.592569% | 9.745380% | +0.152811 |
| BLE_CTRL | unknown: stale historical | unknown | unknown | unknown |
| SERIAL_CTRL | 947,358 / 120,534,705 | 0.709568% | 0.785963% | +0.076395 |
| WIFI_CTRL | unknown: not_observed | unknown | unknown | unknown |

</details>


<details>
<summary>screen09 结果和各层窗口 / screen09 results and layer windows</summary>

[原始上下文 / Original context](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/cdc_recovery_acceptance_20261116.md#L440)

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

</details>


<details>
<summary>screen09 结果和各层窗口 / screen09 results and layer windows</summary>

[原始上下文 / Original context](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/cdc_recovery_acceptance_20261116.md#L459)

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

</details>


<details>
<summary>CPU 变化与剩余阻塞 / CPU changes and remaining blocker</summary>

[原始上下文 / Original context](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/cdc_recovery_acceptance_20261116.md#L478)

| Role | screen09 ΔrunUs / own spanUs | screen08 ratio | screen09 ratio |
|---|---:|---:|---:|
| BLE_TX | 21,167,994 / 119,927,415 | 15.033813% | 17.650671% |
| TRANSPORT | 8,935,494 / 120,069,062 | 7.547857% | 7.441962% |
| ASYNC_LOG | 16,217,602 / 120,181,452 | 24.600562% | 13.494264% |
| SERIAL_TX | 12,332,742 / 120,513,737 | 9.745380% | 10.233474% |
| BLE_CTRL | unknown: stale historical sample | unknown | unknown |
| SERIAL_CTRL | 948,055 / 120,135,925 | 0.785963% | 0.789152% |
| WIFI_CTRL | unknown: not_observed | unknown | unknown |

</details>


<details>
<summary>完整 screen10 结果 / Complete screen10 results</summary>

[原始上下文 / Original context](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/cdc_recovery_acceptance_20261116.md#L522)

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

</details>


<details>
<summary>各层与 CPU 独立窗口 / Independent layer and CPU windows</summary>

[原始上下文 / Original context](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/cdc_recovery_acceptance_20261116.md#L548)

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

</details>


<details>
<summary>各层与 CPU 独立窗口 / Independent layer and CPU windows</summary>

[原始上下文 / Original context](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/cdc_recovery_acceptance_20261116.md#L561)

| Role | screen10 ΔrunUs / own spanUs | screen09 ratio | screen10 ratio |
|---|---:|---:|---:|
| BLE_TX | 19,719,285 / 120,995,504 | 17.650671% | 16.297535% |
| TRANSPORT | 6,430,590 / 120,777,834 | 7.441962% | 5.324313% |
| ASYNC_LOG | 17,839,172 / 120,133,160 | 13.494264% | 14.849499% |
| SERIAL_TX | 12,055,945 / 119,550,795 | 10.233474% | 10.084370% |
| BLE_CTRL | unknown: stale historical | unknown | unknown |
| SERIAL_CTRL | 961,475 / 120,400,085 | 0.789152% | 0.798567% |
| WIFI_CTRL | unknown: not_observed | unknown | unknown |

</details>


<details>
<summary>科学帧与健康边界 / Scientific frames and health boundaries</summary>

[原始上下文 / Original context](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/cdc_recovery_acceptance_20261116.md#L597)

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

</details>


<details>
<summary>科学帧与健康边界 / Scientific frames and health boundaries</summary>

[原始上下文 / Original context](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/cdc_recovery_acceptance_20261116.md#L618)

| ADSHEALTH 边界 / Boundary | 首条完整记录 / First complete record | 末条完整记录 / Last complete record |
|---|---:|---:|
| Host receipt monotonic (s) | 329986.571616645 | 330586.055745967 |
| seq / engineFrameCount | 13929 / 8640 | 53225 / 47936 |
| modeEntryFrameCount | 8259 | 8259 |
| healthInterval / healthReadbackDue / precisionFrame | 64 / 1 / 1 | 64 / 1 / 1 |
| err | 0 | 0 |
| crcValidEven / crcValidOdd / crcErrors | 49579 / 47800 / 0 | 267038 / 258472 / 0 |

</details>


<details>
<summary>各层与 CPU 独立窗口 / Independent layer and CPU windows</summary>

[原始上下文 / Original context](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/cdc_recovery_acceptance_20261116.md#L637)

| Layer | 本层观察窗口 / Own observation span | 本层增量 / Own delta |
|---|---:|---|
| USB | 600.426124 s | resets/mounts/unmounts/rxDrop/rxOver: 0; rxBytes: 4,219 |
| USBQ | 600.432809 s | dSubmit/cSubmit: 39,355 / 487; dDrop/cDrop/expired/cExpired/partial: all 0 |
| USBQ accepted bytes | same USBQ span | dBytes/cBytes: 23,678,340 / 112,608 B; per-lane callback bytes unknown |
| BL50 | 599.542572 s | sentD/dropD/dropL/dropC: 26,502 / 12,583 / 0 / 0; ctrlRetry/ctrlExhaust: 0 |
| BLETX | 600.447359 s | sdkCalls/accepted/errors: 86,714 / 86,715 / 0 |
| BLESVC | 600.434275 s | started/completed: 86,712 / 86,713; wall: 329.772301 s; mean 3.803032 ms/completed |
| BLESVC confirmations | same BLESVC span | confOk/confError: 86,712 / 0; before/after active: 1 / 0 |

</details>


<details>
<summary>各层与 CPU 独立窗口 / Independent layer and CPU windows</summary>

[原始上下文 / Original context](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/cdc_recovery_acceptance_20261116.md#L651)

| Role | stability-01 ΔrunUs / own spanUs | screen10 ratio | stability-01 ratio |
|---|---:|---:|---:|
| BLE_TX | 98,076,373 / 600,267,916 | 16.297535% | 16.338766% |
| TRANSPORT | 31,886,240 / 600,159,072 | 5.324313% | 5.312965% |
| ASYNC_LOG | 89,312,732 / 600,411,082 | 14.849499% | 14.875264% |
| SERIAL_TX | 59,904,941 / 599,954,851 | 10.084370% | 9.984908% |
| BLE_CTRL | unknown: stale historical | unknown | unknown |
| SERIAL_CTRL | 4,725,750 / 600,441,021 | 0.798567% | 0.787046% |
| WIFI_CTRL | unknown: not_observed | unknown | unknown |

</details>


<details>
<summary>screen11 科学结果 / screen11 scientific results</summary>

[原始上下文 / Original context](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/cdc_recovery_acceptance_20261116.md#L717)

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

</details>


<details>
<summary>独立窗口的吞吐与 CPU / Throughput and CPU in independent windows</summary>

[原始上下文 / Original context](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/cdc_recovery_acceptance_20261116.md#L739)

| Layer | screen11 own observation span | Within-window delta |
|---|---:|---|
| USB | 120.363549 s | reset/mount/unmount/rxDrop/rxOver:0; rxBytes:976 |
| USBQ | 120.355108 s | dSubmit/cSubmit:7,748/111; dDrop/cDrop/expired/cExpired/partial:all0 |
| USBQ accepted bytes | same USBQ span | dBytes/cBytes:4,635,602/26,198 B; callback lane bytes unknown |
| BL50 | 119.609541 s | sentD/dropD/dropL/dropC:5,629/2,044/0/0; ctrlRetry/ctrlExhaust:0 |
| BLETX | 120.352648 s | sdkCalls/accepted/errors:18,382/18,382/0 |
| BLESVC | 120.353767 s | completed18,383; wall70.100014 s; mean3.813307 ms/completed |
| BLESVC confirmations | same BLESVC span | confOk/confError:18,383/0; active before/after:1/1 |

</details>


<details>
<summary>独立窗口的吞吐与 CPU / Throughput and CPU in independent windows</summary>

[原始上下文 / Original context](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/cdc_recovery_acceptance_20261116.md#L749)

| Role | screen11 ΔrunUs / own spanUs | screen10 ratio | screen11 ratio |
|---|---:|---:|---:|
| BLE_TX | 18,521,451 / 119,846,726 | 16.297535% | 15.454282% |
| TRANSPORT | 4,335,046 / 119,899,071 | 5.324313% | 3.615579% |
| ASYNC_LOG | 17,650,849 / 120,142,729 | 14.849499% | 14.691567% |
| SERIAL_TX | 12,282,096 / 119,565,363 | 10.084370% | 10.272286% |
| BLE_CTRL | unknown: stale historical | unknown | unknown |
| SERIAL_CTRL | 909,206 / 120,387,871 | 0.798567% | 0.755231% |
| WIFI_CTRL | unknown: not_observed | unknown | unknown |

</details>


<details>
<summary>Serial `final_serial_full`: PASS</summary>

[原始上下文 / Original context](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/validation.md#L348)

| 指标 | 实测值 |
| --- | ---: |
| 完整 RES frames | 2000 |
| Duration | 1193.687 s |
| Complete-frame rate | 1.675 fps |
| S1D1 sanity | 10050.623 Ω |
| S8D8 sanity | 10046.781 Ω |
| Post-ready wire CRC / malformed / non-ASCII | 0 / 0 / 0 |
| Unexpected reset/panic | 0 |

</details>


<details>
<summary>BLE targeted `final_ble_longrun_reconnect_wwr`: PASS</summary>

[原始上下文 / Original context](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/validation.md#L381)

| 指标 | 实测值 |
| --- | ---: |
| 完整 RES DATA frames | 2000 |
| Duration | 108.219 s |
| Complete-frame rate | 18.481 fps |
| S1D1 sanity | 10050.655 Ω |
| S8D8 sanity | 10046.588 Ω |
| Reconnect cycles | 30 |
| BLE wire CRC / malformed / non-ASCII | 0 / 0 / 0 |
| Serial sidecar CRC / malformed / non-ASCII | 0 / 0 / 0 |
| Unexpected reset/panic | 0 |

</details>


<details>
<summary>BLE complete `final_ble_full_pass5`：PASS</summary>

[原始上下文 / Original context](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/validation.md#L427)

| Phase | 实测结果 |
| --- | --- |
| Subscription matrix | NONE 60.015 s；FF11 only；FF20 only 1000 DATA；FF30 only 4 LOG；FF11+20；FF11+30；FF20+30；FF11+20+30 全部 PASS |
| TX modes | FAST/Notify → SAFE/Indicate（Linux/BlueZ 可为 Notify fallback）→ FAST/Notify，完成 DATA/LOG/CTRL，completed-fragment CRC error=0 |
| Mode stress | 20 cycles、80 transitions、542 complete frames（CAP 265 / RES 142 / VOLT 135），stale=0、CRC/malformed/non-ASCII/regression=0 |
| Subscribe stress | 100 cycles、200 subscription windows、200 unsubscribe quiet checks、grace notification=0 |
| RES long-run | 2000 required frames，107.391 s，18.624 complete fps；S1D1=10050.302 Ω、S8D8=10046.572 Ω |
| Reconnect | 30/30 cycles，1041 complete DATA frames；completed CRC error=0、stale=0、CTRL drop=0 |

</details>

<a id="original-reports"></a>

## 原始审计分项 / Report-by-report provenance

以下按原报告分项追溯未通过的候选、对照和修复。每个链接均固定在清理前提交，带行号的链接直达该次记录。下方提交表涵盖更早的迭代。 / Each entry points to the original section at the pre-cleanup revision, including unsuccessful trials and intermediate results. Earlier iterations are included in the commit ledger.


<details>
<summary>docs/audit/cdc_recovery_acceptance_20261116.md</summary>

- [2026-11-16 软件恢复与双 CDC 验收 / Software recovery and dual-CDC acceptance](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/cdc_recovery_acceptance_20261116.md#L1)
- [已执行的恢复步骤 / Executed recovery steps](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/cdc_recovery_acceptance_20261116.md#L13)
- [本轮复现与修改 / Reproductions and changes](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/cdc_recovery_acceptance_20261116.md#L35)
- [恢复标记 / Recovery latch](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/cdc_recovery_acceptance_20261116.md#L37)
- [完整 BUILD 哈希 / Full BUILD hash](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/cdc_recovery_acceptance_20261116.md#L47)
- [首次查询的旧错误 / Stale error at the first query](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/cdc_recovery_acceptance_20261116.md#L53)
- [新镜像恢复闭环 / Repaired-image recovery loop](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/cdc_recovery_acceptance_20261116.md#L59)
- [120 秒负载筛查失败 / 120-second load screen failed](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/cdc_recovery_acceptance_20261116.md#L69)
- [第二轮筛查与当前定位 / Second screen and current investigation](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/cdc_recovery_acceptance_20261116.md#L75)
- [诊断首版启动失败与内存修正 / Initial diagnostic boot failure and memory correction](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/cdc_recovery_acceptance_20261116.md#L93)
- [已完成的内存修正、时序对照与第三轮筛查 / Memory repair, timing comparison and third screen](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/cdc_recovery_acceptance_20261116.md#L103)
- [4bf31db 末位对照结果 / Final-bit comparison results](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/cdc_recovery_acceptance_20261116.md#L132)
- [bc99767 实机失败保留 / Retained hardware failures](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/cdc_recovery_acceptance_20261116.md#L159)
- [末位诊断候选 bc99767 / Final-bit diagnostic candidate bc99767](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/cdc_recovery_acceptance_20261116.md#L169)
- [`bc99767` 时点待完成（历史） / Pending at `bc99767` (historical)](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/cdc_recovery_acceptance_20261116.md#L179)
- [正式 ADC 末位读取修复 / Scientific ADC final-bit repair](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/cdc_recovery_acceptance_20261116.md#L188)
- [BLE SDK 异步返回观测 / BLE SDK callback observations](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/cdc_recovery_acceptance_20261116.md#L222)
- [CPU owner 诊断镜像与 screen07 / CPU-owner diagnostic image and screen07](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/cdc_recovery_acceptance_20261116.md#L250)
- [完整双通路窗口 / Complete dual-link window](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/cdc_recovery_acceptance_20261116.md#L264)
- [各角色独立差分 / Independent per-role deltas](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/cdc_recovery_acceptance_20261116.md#L294)
- [screen08：P1 分配修复、日志清理与独立对照 / screen08: allocation P1 repair, log cleanup and independent comparison](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/cdc_recovery_acceptance_20261116.md#L331)
- [科学接收与各层计数 / Scientific receipt and layer counters](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/cdc_recovery_acceptance_20261116.md#L349)
- [CPU owner 对照与结论边界 / CPU-owner comparison and limits](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/cdc_recovery_acceptance_20261116.md#L392)
- [screen09：元数据单遍解析 / screen09: single-pass frame metadata parsing](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/cdc_recovery_acceptance_20261116.md#L416)
- [实施与语义回归 / Implementation and semantic regression](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/cdc_recovery_acceptance_20261116.md#L422)
- [screen09 结果和各层窗口 / screen09 results and layer windows](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/cdc_recovery_acceptance_20261116.md#L436)
- [CPU 变化与剩余阻塞 / CPU changes and remaining blocker](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/cdc_recovery_acceptance_20261116.md#L476)
- [screen10：保留检查的 `-O2` 与实机对照 / screen10: `-O2` with retained checks and hardware comparison](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/cdc_recovery_acceptance_20261116.md#L494)
- [未烧录的拒绝候选与 wrapper 修正 / Rejected unflashed candidate and wrapper correction](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/cdc_recovery_acceptance_20261116.md#L500)
- [完整 screen10 结果 / Complete screen10 results](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/cdc_recovery_acceptance_20261116.md#L518)
- [各层与 CPU 独立窗口 / Independent layer and CPU windows](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/cdc_recovery_acceptance_20261116.md#L546)
- [原始证据归档 / Original evidence archive](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/cdc_recovery_acceptance_20261116.md#L577)
- [stability-01：同镜像连续 600 秒 / stability-01: continuous 600 seconds on the same image](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/cdc_recovery_acceptance_20261116.md#L585)
- [科学帧与健康边界 / Scientific frames and health boundaries](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/cdc_recovery_acceptance_20261116.md#L595)
- [关闭本任务 reader 后的只读观察 / Read-only observation after this task's readers closed](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/cdc_recovery_acceptance_20261116.md#L665)
- [归档与复核 / Archive and verification](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/cdc_recovery_acceptance_20261116.md#L675)
- [screen11：CRC32/planner 等价优化与 ADS 低电平建立 / screen11: equivalent CRC32/planner optimization and ADS low hold](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/cdc_recovery_acceptance_20261116.md#L683)
- [两个小补丁与保留的检查 / Two small patches and retained checks](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/cdc_recovery_acceptance_20261116.md#L693)
- [screen11 科学结果 / screen11 scientific results](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/cdc_recovery_acceptance_20261116.md#L711)
- [独立窗口的吞吐与 CPU / Throughput and CPU in independent windows](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/cdc_recovery_acceptance_20261116.md#L737)
- [原始流归档 / Original-recording archive](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/cdc_recovery_acceptance_20261116.md#L767)

</details>


<details>
<summary>docs/audit/firmware_reliability_fix_report.md</summary>

- [SensorArray 固件可靠性修复报告](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/firmware_reliability_fix_report.md#L1)
- [32 工况正式 USB 与 BLE 验收 / 32-configuration USB and BLE screen](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/firmware_reliability_fix_report.md#L3)
- [USB CDC0 在短暂停读后的数据完整性 / Formal USB delivery after a short reader pause](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/firmware_reliability_fix_report.md#L13)
- [BSS-CURRENT-HIL-20261125 — 600秒、50切换及SHORT闭合 / Closed long run, transitions and SHORT](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/firmware_reliability_fix_report.md#L60)
- [600秒VOLT1正常FULL / 600-second VOLT1 normal FULL](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/firmware_reliability_fix_report.md#L66)
- [50次模式切换 / 50 mode transitions](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/firmware_reliability_fix_report.md#L99)
- [SHORT保护FULL快照 / Protected FULL snapshots in SHORT](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/firmware_reliability_fix_report.md#L109)
- [原始矩阵闭合记录 / Original matrix windows](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/firmware_reliability_fix_report.md#L121)
- [WIFI-BSS-HIL-20261125 — AP08首次启动与USB/BLE共存控制通过 / AP08 startup and coexistence control observed](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/firmware_reliability_fix_report.md#L149)
- [WIFI-DMA-HEADROOM-HIL-20261125 — 内存增加后在AP启动发生panic / More internal memory, then an AP-start panic](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/firmware_reliability_fix_report.md#L163)
- [HEAP-FAILURE-HIL-20261125 — AP06记录内部DMA分配失败 / AP06 records an internal DMA allocation failure](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/firmware_reliability_fix_report.md#L173)
- [BT-HOST-MEMORY-HIL-20261121 — host优先PSRAM的内存收益未解除AP阻塞 / Host PSRAM preference does not clear AP blockage](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/firmware_reliability_fix_report.md#L193)
- [WIFI-MEMORY-HIL-20261121 — PSRAM迁移后仍失败，BLE终态实收 / AP still fails after PSRAM migration; BLE terminals received](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/firmware_reliability_fix_report.md#L207)
- [WIFI-STACK-HIL-20261121 — 返回失败路径可查询，AP仍受内存阻塞 / Queryable returned failure; AP memory blocker remains](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/firmware_reliability_fix_report.md#L217)
- [NVS-PRESERVE-20261121 — 初始化错误不能授权擦除 / Initialization failure does not authorize erasure](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/firmware_reliability_fix_report.md#L227)
- [PANIC-DIAG-20261121 — 为AP panic保留有限证据 / Bounded evidence for the AP panic](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/firmware_reliability_fix_report.md#L233)
- [PANIC-HIL-20261121：已确认owner栈溢出 / Owner-stack overflow confirmed](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/firmware_reliability_fix_report.md#L247)
- [WIFI-OWNER-20261121 — SDK阻塞隔离与可查询终态 / SDK isolation and queryable terminals](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/firmware_reliability_fix_report.md#L257)
- [MANUAL-RESTART-20261121 — 显式重启绕过阻塞shutdown handler / Explicit restart bypasses blocking shutdown handlers](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/firmware_reliability_fix_report.md#L278)
- [BLE-TYPED-SCREEN21-20261121 — 数值预算通过但整体验收未闭合 / Numeric budget met, overall acceptance incomplete](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/firmware_reliability_fix_report.md#L292)
- [BLE-SCRATCH-SCREEN20-20261121 — 内部工作区性能假设未获验收 / Internal working-copy hypothesis remains unaccepted](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/firmware_reliability_fix_report.md#L306)
- [BLE-SHORT-20261121 — SHORT预览中的完整周期实收与历史闭合 / Complete snapshots during SHORT preview and historical closure](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/firmware_reliability_fix_report.md#L320)
- [USB-UNREAD-20261121 — 正式CDC应用停止读写的独立专项 / Independent unread-client stress observation](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/firmware_reliability_fix_report.md#L330)
- [BLE-SNAP-SCREEN19-20261121 — 周期合并后实机吞吐仍失败 / Hardware throughput still fails after cadence coalescing](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/firmware_reliability_fix_report.md#L344)
- [BLE-SNAP-COALESCE-20261121：修复重复周期保护 / Coalescing duplicate periodic protection](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/firmware_reliability_fix_report.md#L358)
- [2026-11-21 screen18：启动已恢复，正常BLE缺口预算失败 / Startup recovered, normal BLE gap budget fails](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/firmware_reliability_fix_report.md#L376)
- [BLE-SNAP-20261121：保护快照、调度与到期闭环 / Protected snapshots, scheduling and expiry](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/firmware_reliability_fix_report.md#L398)
- [BLE-SNAP-MEM-20261121：build02实机启动失败 / Actual build02 startup failure](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/firmware_reliability_fix_report.md#L429)
- [2026-11-21 screen17：LOG 策略的实测与限定通过 / Measured LOG policy and scoped acceptance](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/firmware_reliability_fix_report.md#L451)
- [2026-11-21 criteria v2 补充判定 / Supplemental assessment under criteria v2](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/firmware_reliability_fix_report.md#L469)
- [2026-11-21 stability-02：原标准下的 600 秒判定记录 / Preserved 600-second verdict under the original criteria](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/firmware_reliability_fix_report.md#L485)
- [2026-11-21 同ELF的50次USB身份切换 / 50 USB identity transitions on the same ELF](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/firmware_reliability_fix_report.md#L503)
- [2026-11-21 screen16：CTRL/生命周期、DATA、普通LOG的严格选择顺序 / screen16: ordered CTRL/lifecycle, DATA and ordinary LOG selection](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/firmware_reliability_fix_report.md#L513)
- [2026-11-21 screen15：guard/汇总候选实测闭环 / screen15: guard/summary hardware closure](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/firmware_reliability_fix_report.md#L527)
- [2026-11-21 SDK guard/OTG 汇总候选：本地验证 / SDK guard/OTG summary candidate: local validation](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/firmware_reliability_fix_report.md#L541)
- [2026-11-21 screen14：DATA 两片打包候选 / screen14: two-fragment DATA packing candidate](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/firmware_reliability_fix_report.md#L558)
- [2026-11-21 screen13 与待修 P1 / screen13 and pending P1](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/firmware_reliability_fix_report.md#L578)
- [2026-11-16 screen12：MTU517 实测补充 / screen12: MTU517 hardware evidence](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/firmware_reliability_fix_report.md#L592)
- [2026-11-16 screen11：CRC32/planner 与 ADS 低电平等待 / screen11: CRC32/planner and ADS low hold](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/firmware_reliability_fix_report.md#L618)
- [2026-11-16 历史 `a656d53` 单次连续 600 秒观察 / Historical `a656d53` single continuous 600-second observation](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/firmware_reliability_fix_report.md#L645)
- [2026-11-16 保留检查的 `-O2` 与 screen10 / `-O2` with retained checks and screen10](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/firmware_reliability_fix_report.md#L674)
- [2026-11-16 元数据解析性能增量 / Metadata-parsing performance increment](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/firmware_reliability_fix_report.md#L704)
- [2026-11-16 已复现的诊断内存回归与修复 / Reproduced diagnostic-memory regression](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/firmware_reliability_fix_report.md#L724)
- [2026-11-16 CRC 根因诊断增量 / CRC diagnostic increment](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/firmware_reliability_fix_report.md#L732)
- [上一版已烧录版本、结果与剩余阻塞（2026-11-05，历史）](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/firmware_reliability_fix_report.md#L759)
- [首次离线审计记录（ba8346b，历史）](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/firmware_reliability_fix_report.md#L783)
- [环境、材料和证据边界](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/firmware_reliability_fix_report.md#L787)
- [已核实 symbol map 与首错定位](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/firmware_reliability_fix_report.md#L804)
- [编号对应、修改与验证](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/firmware_reliability_fix_report.md#L824)
- [模拟连接、安全降级及辅助事务](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/firmware_reliability_fix_report.md#L845)
- [锁顺序与所有权](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/firmware_reliability_fix_report.md#L853)
- [验证、性能和未运行项目](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/firmware_reliability_fix_report.md#L870)
- [交付与剩余验收](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/firmware_reliability_fix_report.md#L892)
- [FW02/FW10 增量：末位采样 / Final-bit sampling increment](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/firmware_reliability_fix_report.md#L900)
- [FW02 增量：初始化部分分配失败 / Partial allocation failure during initialization](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/firmware_reliability_fix_report.md#L935)
- [ACCEPTANCE-RUNNER-20261121：闭合采集证据 / Closed capture evidence](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/firmware_reliability_fix_report.md#L955)

</details>


<details>
<summary>docs/audit/frontend_startup_recovery_20261113.md</summary>

- [前端启动选速率与掉电恢复（2026-11-13）](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/frontend_startup_recovery_20261113.md#L1)
- [代码职责](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/frontend_startup_recovery_20261113.md#L5)
- [SPI 资格标准](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/frontend_startup_recovery_20261113.md#L12)
- [失联与恢复边界](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/frontend_startup_recovery_20261113.md#L22)
- [验证记录](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/frontend_startup_recovery_20261113.md#L30)
- [中间版本 e339a1d 的实际结果（保留失败，不作为最终验收）](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/frontend_startup_recovery_20261113.md#L34)
- [运行期 CRC 分类追加修复（0d8f60f）](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/frontend_startup_recovery_20261113.md#L58)
- [当前烧录版 0d8f60f 的实测记录](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/frontend_startup_recovery_20261113.md#L68)
- [交付与未运行项目](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/frontend_startup_recovery_20261113.md#L94)

</details>


<details>
<summary>docs/audit/load_reliability_fix_20261113.md</summary>

- [SensorArray 负载可靠性修复 / Load reliability repair — 2026-11-13](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/load_reliability_fix_20261113.md#L4)
- [当前阶段：d314888，screen-04 仍 FAIL / Current stage: d314888, screen-04 still FAIL](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/load_reliability_fix_20261113.md#L6)
- [已完成的 b44c37b 筛查（历史） / Completed b44c37b screening (history)](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/load_reliability_fix_20261113.md#L39)
- [fd0190a 历史筛查与当时状态 / Historical fd0190a screening and status at that stage](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/load_reliability_fix_20261113.md#L47)
- [失败基线与因果边界 / Failed baseline and limits of causal evidence](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/load_reliability_fix_20261113.md#L79)
- [已确认缺陷与修改 / Confirmed defects and changes](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/load_reliability_fix_20261113.md#L89)
- [USB 记录与期限 / USB records and deadlines](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/load_reliability_fix_20261113.md#L100)
- [七任务工作预算与真实 idle / Seven-worker work budget and real idle](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/load_reliability_fix_20261113.md#L122)
- [PGA 稳定性与运行期 CRC / PGA stability and runtime CRC](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/load_reliability_fix_20261113.md#L154)
- [恢复、重启与配置身份 / Recovery, restart and configuration identity](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/load_reliability_fix_20261113.md#L168)
- [实际执行与保留失败 / Executed validation and retained failures](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/load_reliability_fix_20261113.md#L190)
- [待完成与科学边界 / Remaining work and scientific limits](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/load_reliability_fix_20261113.md#L213)
- [4b665ac screening result / 4b665ac 筛查结果](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/load_reliability_fix_20261113.md#L223)

</details>


<details>
<summary>docs/audit/reference_fixture_hil_20261104.md</summary>

- [2026-11-04 参考夹具实机验收记录](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/reference_fixture_hil_20261104.md#L1)
- [设备与烧录证据](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/reference_fixture_hil_20261104.md#L5)
- [首次修复镜像的实际结果](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/reference_fixture_hil_20261104.md#L14)
- [HIL 暴露的追加缺陷](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/reference_fixture_hil_20261104.md#L30)
- [追加修复与实际离线回归](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/reference_fixture_hil_20261104.md#L39)
- [a0460b8 实机复测：恢复生效但仍有故障](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/reference_fixture_hil_20261104.md#L46)
- [同一实录中的 rail 平台跃迁（尚不可校准）](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/reference_fixture_hil_20261104.md#L59)
- [8d6d656：rail 专用配置与寄存器 guard 的实机结果](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/reference_fixture_hil_20261104.md#L65)
- [再次实测反例驱动的边界与输出修复](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/reference_fixture_hil_20261104.md#L83)
- [ebabe7e 最终软件版本与后续实机验收](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/reference_fixture_hil_20261104.md#L97)
- [最终实机结果及限制](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/reference_fixture_hil_20261104.md#L109)

</details>


<details>
<summary>docs/audit/reference_fixture_recheck_20261113.md</summary>

- [2026-11-13：接触修复、外部轨输入与悬空口近似背景复测](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/reference_fixture_recheck_20261113.md#L1)
- [新输入与适用边界](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/reference_fixture_recheck_20261113.md#L7)
- [实际采集与首次故障](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/reference_fixture_recheck_20261113.md#L15)
- [电阻、rail与CRC的区分](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/reference_fixture_recheck_20261113.md#L31)
- [用户授权的悬空口近似背景](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/reference_fixture_recheck_20261113.md#L49)
- [验证、输出与最终状态](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/reference_fixture_recheck_20261113.md#L62)

</details>


<details>
<summary>docs/audit/usb_otg_ble_priority_20261114.md</summary>

- [USB-OTG 与 BLE 优先级修复 / USB-OTG and BLE priority repair](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/usb_otg_ble_priority_20261114.md#L1)
- [实机迁移阻塞：下载切换未完成 / HIL migration blocker: ROM handoff not confirmed](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/usb_otg_ble_priority_20261114.md#L32)

</details>


<details>
<summary>README.md</summary>

- [32 配置实测 / 32-configuration matrix](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/README.md#L3)
- [十分钟稳定性、切换与关键帧 / Stability, transitions and protected snapshots](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/README.md#L66)
- [正式 USB 长时间停读与 Wi-Fi 共存 / Unread USB and Wi-Fi coexistence](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/README.md#L78)
- [正式 USB 短暂停读回归 / Formal USB reader-pause regression](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/README.md#L86)
- [VOLT8 逐帧电压与夹具 / Frame-level voltage and fixture checks](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/README.md#L101)
- [SensorArray — ESP32-S3 固件 / ESP32-S3 Firmware](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/README.md#L154)
- [2026-11-25 当前镜像实测 / Measurements on the current image](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/README.md#L164)
- [原始32配置矩阵 / Original 32-configuration matrix](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/README.md#L182)
- [Wi-Fi SDK 独立任务与手动重启 / Isolated Wi-Fi SDK execution and manual restart](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/README.md#L223)
- [显式手动重启的SDK依赖 / SDK dependency of explicit manual restart](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/README.md#L256)
- [最终验收采集的身份与停止条件 / Identity and stop conditions for final acceptance captures](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/README.md#L268)
- [2026-11-21 screen21：保留配置域的typed metadata与正常缺口预算 / Typed metadata and the normal gap budget](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/README.md#L292)
- [2026-11-21 screen20：内部发送工作区对照未达标 / Internal TX working-copy comparison remains below target](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/README.md#L344)
- [2026-11-21 CAP8 SHORT专项：普通预览与完整周期快照 / Ordinary previews and periodic full snapshots](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/README.md#L391)
- [2026-11-21 USB应用不读专项：BLE继续，USB排队有界丢弃 / Unread USB client: BLE continues while USB queues discard data](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/README.md#L415)
- [2026-11-21 screen19：周期合并已生效，容量拒绝仍阻塞验收 / Coalescing works; admission pressure still blocks acceptance](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/README.md#L462)
- [build04：完整帧完成合并周期义务 / Full-frame completion coalesces periodic obligations](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/README.md#L513)
- [2026-11-21 screen18：保护机制首轮实测未通过 / First protected-output screen fails acceptance](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/README.md#L541)
- [2026-11-21 保护快照与输出适应 / Protected snapshots and output adaptation](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/README.md#L588)
- [USB-OTG CDC 与 BLE 运转机制 / USB-OTG CDC and BLE operation](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/README.md#L630)
- [2026-11-21 screen17：普通日志让出 BLE 容量 / Ordinary logs yield BLE capacity](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/README.md#L662)
- [2026-11-21 criteria v2：BLE 可接受缺口与保护数据 / Acceptable BLE gaps and protected data](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/README.md#L699)
- [2026-11-21 stability-02：保留原标准下的 10 分钟结果 / Preserved 10-minute result under the original criteria](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/README.md#L720)
- [2026-11-21 同镜像50次配置身份切换 / 50 configuration-identity transitions on the same image](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/README.md#L750)
- [2026-11-21 screen16：正式 DATA 优先于普通 LOG / screen16: formal DATA before ordinary LOG](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/README.md#L764)
- [2026-11-21 screen15：guard 与汇总周期的同版实测 / screen15: hardware observation of guard and summary cadence](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/README.md#L797)
- [2026-11-21 BLE SDK 等待监管与 OTG 调试汇总候选 / BLE SDK wait supervision and OTG debug-summary candidate](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/README.md#L826)
- [2026-11-21 screen14：保留两片数量的 DATA 打包 / screen14: DATA packing with the same two-fragment count](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/README.md#L848)
- [2026-11-21 screen13：重启后的同版复测 / screen13: same-image observation after restart](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/README.md#L882)
- [2026-11-16 screen12：实际 MTU517 与剩余吞吐差距 / screen12: negotiated MTU517 and remaining throughput gap](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/README.md#L910)
- [2026-11-16 screen11：CRC32/planner 优化与明确的低电平等待 / screen11: CRC32/planner optimization and explicit low hold](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/README.md#L939)
- [2026-11-16 历史 `a656d53` 连续 600 秒稳定性 / Historical `a656d53` continuous 600-second stability](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/README.md#L972)
- [2026-11-16 `-O2` 候选与安全检查 / `-O2` candidate and retained safety checks](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/README.md#L1004)
- [2026-11-16 screen09（历史）：元数据单遍解析 / Historical screen09: single-pass metadata parsing](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/README.md#L1037)
- [2026-11-16 screen08（历史）：分配生命周期修复与日志统计清理 / Historical screen08: allocation lifetime and unused log statistics](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/README.md#L1062)
- [2026-11-16 CPU 诊断首轮 screen07（历史） / First CPU diagnostic screen07 (historical)](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/README.md#L1089)
- [2026-11-16 首轮双通路筛查 / First dual-link screen](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/README.md#L1132)
- [2026-11-16 第二轮筛查与 CRC 定位 / Second screen and CRC investigation](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/README.md#L1154)
- [诊断首版启动失败与内存修正 / Initial diagnostic boot failure and memory correction](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/README.md#L1187)
- [已完成的内存修正、时序对照与第三轮筛查 / Memory repair, timing comparison and third screen](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/README.md#L1197)
- [4bf31db 末位对照结果 / Final-bit comparison results](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/README.md#L1226)
- [bc99767 实机失败保留 / Retained hardware failures](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/README.md#L1253)
- [末位诊断候选 bc99767 / Final-bit diagnostic candidate bc99767](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/README.md#L1263)
- [历史 Serial/JTAG 修复与失败基线 / Historical Serial/JTAG repairs and failed baselines](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/README.md#L1273)
- [0d8f60f 历史失败基线（原记录保留） / Historical 0d8f60f failed baseline (original record retained)](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/README.md#L1315)
- [快速开始 / Quick start](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/README.md#L1332)
- [环境、构建与连接 / Environment, build, and connection](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/README.md#L1334)
- [Serial 查询与配置 / Serial queries and configuration](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/README.md#L1353)
- [BLE 与 Wi-Fi / BLE and Wi-Fi](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/README.md#L1398)
- [固件如何运转 / How the firmware operates](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/README.md#L1411)
- [双核与双 FDC / Dual cores and dual FDCs](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/README.md#L1413)
- [CPU0 工作预算与真实 idle / CPU0 work budgets and actual idle progress](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/README.md#L1445)
- [ADS shadow、cache、PGA 与新鲜度 / ADS shadow, caches, PGA, and freshness](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/README.md#L1467)
- [CRC 部分帧与全局故障 / CRC partial frames and global faults](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/README.md#L1484)
- [逐行 C/V/R、参考与辅助事务 / Per-row C/V/R, reference, and auxiliary transactions](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/README.md#L1504)
- [启动并行选速 / Parallel startup qualification](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/README.md#L1525)
- [全前端失联与恢复 / Frontend loss and recovery](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/README.md#L1543)
- [原子配置身份与终态 / Atomic configuration identity and terminal results](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/README.md#L1557)
- [RECOVER/RESTART 的有界终态 / Bounded RECOVER/RESTART terminal results](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/README.md#L1577)
- [rail、电池、校准与无效点 / Rails, battery, calibration, and invalid points](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/README.md#L1591)
- [传输与分层背压 / Transport and layered backpressure](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/README.md#L1611)
- [基线实测结果 / Baseline test results](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/README.md#L1653)
- [已完成的0d8f60f窗口 / Completed 0d8f60f runs](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/README.md#L1655)
- [0d8f60f：600秒与32配置失败基线 / 0d8f60f: 600-second run and failed 32-configuration baseline](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/README.md#L1675)
- [测试方法与判据 / Method and interpretation](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/README.md#L1893)
- [文档与源码导航 / Documentation and source map](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/README.md#L1904)
- [协作与许可证 / Contribution and license](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/README.md#L1926)
- [正式 ADC 末位读取修复 / Scientific ADC final-bit repair](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/README.md#L1937)
- [BLE SDK 异步返回观测 / BLE SDK callback observations](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/README.md#L1971)

</details>


<details>
<summary>docs/validation.md</summary>

- [构建、自动测试与 HIL 验证](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/validation.md#L1)
- [验证原则](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/validation.md#L6)
- [状态模板与证据边界](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/validation.md#L22)
- [Clean build](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/validation.md#L42)
- [Host 与协议测试](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/validation.md#L62)
- [HIL 依赖](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/validation.md#L91)
- [Transport 验收顺序（Serial -> BLE）](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/validation.md#L111)
- [同一已刷写 image、串口 /dev/ttyACM0、BLE adapter 已就绪](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/validation.md#L128)
- [Serial HIL](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/validation.md#L146)
- [BLE HIL](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/validation.md#L196)
- [Subscription matrix](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/validation.md#L214)
- [FAST / SAFE](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/validation.md#L236)
- [Stress](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/validation.md#L252)
- [nRF Connect 最小手工验收](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/validation.md#L260)
- [Stack、heap 与 slot](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/validation.md#L281)
- [性能回归](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/validation.md#L318)
- [历史实测结果（不作为当前 HEAD 证据）](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/validation.md#L322)
- [2026-08-13 `/dev/ttyACM0` 重试（历史）](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/validation.md#L328)
- [Serial `final_serial_full`: PASS](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/validation.md#L344)
- [BLE targeted `final_ble_longrun_reconnect_wwr`: PASS](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/validation.md#L377)
- [精度证据边界](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/validation.md#L417)
- [BLE complete `final_ble_full_pass5`：PASS](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/validation.md#L423)
- [发布结果格式](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/validation.md#L468)

</details>


<details>
<summary>docs/integration/firmware_host_contract.md</summary>

- [固件与上位机契约：配置身份与可靠性修复](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/integration/firmware_host_contract.md#L1)
- [AP08实录不改变交付含义 / AP08 observation preserves delivery semantics](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/integration/firmware_host_contract.md#L31)
- [MEMFAIL v1：boot范围的失败分配观察 / Boot-scoped failed-allocation observations](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/integration/firmware_host_contract.md#L43)
- [PANIC v1：保留诊断，不是修复 / Retained diagnosis, not a fix](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/integration/firmware_host_contract.md#L67)
- [Wi-Fi owner v1 与异步控制 / Wi-Fi owner v1 and asynchronous control](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/integration/firmware_host_contract.md#L96)
- [显式RESTART/USBBOOT终止路径 / Explicit RESTART/USBBOOT terminal path](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/integration/firmware_host_contract.md#L150)
- [本次返回失败后的手动重启观察 / Manual restart after this returned failure](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/integration/firmware_host_contract.md#L160)
- [NVS初始化失败保留校准 / Preserve calibration on NVS initialization failure](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/integration/firmware_host_contract.md#L166)
- [Bluedroid host的PSRAM优先策略 / Bluedroid host PSRAM preference](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/integration/firmware_host_contract.md#L172)
- [WTERM实收与发布快照 / WTERM receipt and publication snapshots](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/integration/firmware_host_contract.md#L178)
- [build06 typed metadata兼容边界 / Typed metadata compatibility](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/integration/firmware_host_contract.md#L184)
- [build05对照不改变契约 / Build05 comparison leaves the contract unchanged](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/integration/firmware_host_contract.md#L190)
- [SHORT实测分母 / SHORT observation denominator](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/integration/firmware_host_contract.md#L196)
- [USB unread：缓存与接收语义 / Buffered USB and receipt semantics](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/integration/firmware_host_contract.md#L202)
- [screen19：protected-only字段的实机反例 / Hardware example of protected-only fields](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/integration/firmware_host_contract.md#L212)
- [build04兼容边界：protected-only查询与内部full依据 / Protected-only queries and private full evidence](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/integration/firmware_host_contract.md#L228)
- [screen18：诊断计数与PC实收分离 / Local diagnostic counters versus PC receipt](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/integration/firmware_host_contract.md#L242)
- [保护快照 v1 契约 / Protected-snapshot v1 contract](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/integration/firmware_host_contract.md#L254)
- [screen17 / BLELOG v1：已观察到的普通日志准入策略 / Observed ordinary-log admission policy](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/integration/firmware_host_contract.md#L304)
- [criteria v2：传输缺口预算不是科学 CRC 宽限 / Transport-gap budget is separate from scientific CRC](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/integration/firmware_host_contract.md#L331)
- [stability-02：保留原判定及身份边界 / Preserved original verdict and identity boundaries](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/integration/firmware_host_contract.md#L349)
- [50次实机切换的字段域证据 / Field-domain evidence from 50 hardware transitions](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/integration/firmware_host_contract.md#L367)
- [screen16：DATA 优先与普通LOG可延后 / screen16: DATA priority and deferrable ordinary LOG](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/integration/firmware_host_contract.md#L377)
- [screen15：已观察到的 BLEGUARD 支持 / screen15: observed BLEGUARD support](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/integration/firmware_host_contract.md#L391)
- [BLEGUARD v1：新增只读能力探测 / BLEGUARD v1: new read-only support probe](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/integration/firmware_host_contract.md#L401)
- [screen14 可变分片长度仍按原协议 / screen14 retains the variable-length fragment contract](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/integration/firmware_host_contract.md#L436)
- [2026-11-21 同版新会话与待修 SDK 等待 / Same image, new session and pending SDK wait fix](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/integration/firmware_host_contract.md#L452)
- [MTU517 兼容性补充 / MTU517 compatibility update](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/integration/firmware_host_contract.md#L462)
- [启动扫描与前端恢复扩展（本轮新增）](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/integration/firmware_host_contract.md#L489)
- [运行期 CRC 部分帧（0d8f60f）](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/integration/firmware_host_contract.md#L530)
- [能力协商与兼容边界](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/integration/firmware_host_contract.md#L536)
- [字段域与 immutable frame identity](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/integration/firmware_host_contract.md#L542)
- [CONFIG?：无副作用的原子查询](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/integration/firmware_host_contract.md#L572)
- [接受、应用、丢失与乱序恢复](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/integration/firmware_host_contract.md#L585)
- [启动、计数器与时间](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/integration/firmware_host_contract.md#L602)
- [CRC、无效点与科学数据](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/integration/firmware_host_contract.md#L612)
- [rail、电池、错误与 transport](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/integration/firmware_host_contract.md#L620)
- [可重放证据与尚未运行项](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/integration/firmware_host_contract.md#L642)
- [实机追加诊断：CRC 与健康边界](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/integration/firmware_host_contract.md#L648)
- [2026-11-05 追加诊断契约：8d6d656](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/integration/firmware_host_contract.md#L660)
- [电池：入场状态与恢复结果分开](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/integration/firmware_host_contract.md#L664)
- [Rail：专用采样配置与能力边界](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/integration/firmware_host_contract.md#L683)
- [CRC 覆盖计数与有界日志](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/integration/firmware_host_contract.md#L699)
- [后续边界补丁：rail CRC、MIXED、USB 输出绑定](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/integration/firmware_host_contract.md#L709)
- [完整串口记录边界（ebabe7e）](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/integration/firmware_host_contract.md#L725)
- [2026-11-13 待验收补丁：RECOVER 终态与独立 I/O 观测](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/integration/firmware_host_contract.md#L733)
- [USB-OTG formal data / 正式数据扩展（2026-11-14）](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/integration/firmware_host_contract.md#L802)
- [双 CDC 替代接口 / Dual-CDC alternative (2026-11-14)](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/integration/firmware_host_contract.md#L832)
- [有界 ADS 时序诊断 / Bounded ADS timing diagnostic (2026-11-16)](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/integration/firmware_host_contract.md#L878)
- [TAIL 诊断种类扩展 / TAIL diagnostic-kind extension (2026-11-16)](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/integration/firmware_host_contract.md#L923)
- [TAIL IO 取证与失败结果恢复 / TAIL IO evidence and failed-result recovery](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/integration/firmware_host_contract.md#L947)
- [ADC final-bit 候选的来源契约 / Read provenance for the ADC final-bit candidate](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/integration/firmware_host_contract.md#L967)
- [BLE local CONF observation extension](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/integration/firmware_host_contract.md#L989)
- [CPU owner 只读运行计数契约 / Read-only owner runtime contract](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/integration/firmware_host_contract.md#L995)
- [Final-capture host authority and evidence gates / 最终采集身份与证据门禁](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/integration/firmware_host_contract.md#L1052)

</details>


<details>
<summary>docs/hardware/reliability_revision_notes.md</summary>

- [Reliability hardware revision notes](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/hardware/reliability_revision_notes.md#L1)
- [Connectivity and firmware limits](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/hardware/reliability_revision_notes.md#L15)
- [Revision register and proposed BOM](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/hardware/reliability_revision_notes.md#L41)
- [Manuals reviewed](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/hardware/reliability_revision_notes.md#L56)
- [Observations from the 2026-11-04/05 passive-fixture run](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/hardware/reliability_revision_notes.md#L62)
- [2026-11-13 fixture and startup-scan update](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/hardware/reliability_revision_notes.md#L72)
- [2026-11-14 USB interface migration / USB 接口迁移](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/hardware/reliability_revision_notes.md#L93)
- [2026-11-16 软件下载恢复实测 / Software download recovery evidence](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/hardware/reliability_revision_notes.md#L100)
- [2026-11-16 ADC读取路径实测边界 / ADC read-path evidence boundary](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/hardware/reliability_revision_notes.md#L106)

</details>


<a id="commit-ledger"></a>

## 提交索引 / Commit ledger

按 Git 拓扑的逆序遍历列出至本次整理前的全部可达提交（旧至新），保留原始提交标题及合并节点；较长原始标题可点提交全文。 / All commits reachable from the archival revision, including merges, are listed oldest first in reverse topological order. Open a commit for the full original message and file diff.

| 作者日期 / Author date | 提交 / Commit | 原题 / Original subject |
|---|---|---|
| 2025-12-22 | [f6fbc0aa](https://github.com/ErrorChen/SensorArray/commit/f6fbc0aa2657a1fac9f0b7019a91ef645b1415c0) | NewProject |
| 2025-12-22 | [a4d678e4](https://github.com/ErrorChen/SensorArray/commit/a4d678e4ac33b318b883d4f47234cde2278c3b0d) | Initial commit |
| 2025-12-22 | [e7cf4497](https://github.com/ErrorChen/SensorArray/commit/e7cf4497bb660bc31b3d361e69b5a5abc2198ba4) | Resolve conflicts |
| 2025-12-22 | [e681842e](https://github.com/ErrorChen/SensorArray/commit/e681842ea80b36727131d7d031222b707756c430) | Test |
| 2025-12-22 | [31c63cc3](https://github.com/ErrorChen/SensorArray/commit/31c63cc3c0978eb77831a6fb9c6b1e94027c0679) | Finish conflicts |
| 2025-12-22 | [ab14ff53](https://github.com/ErrorChen/SensorArray/commit/ab14ff53434260e35ab737a2b51f75b745f4c396) | Final? |
| 2025-12-22 | [9fc02060](https://github.com/ErrorChen/SensorArray/commit/9fc0206030adc7a055f87544fa16be9979bbba7f) | gitset |
| 2025-12-22 | [39cbb2b1](https://github.com/ErrorChen/SensorArray/commit/39cbb2b11aa1830229c708a8ef65e6237b09ab02) | Ignore build artifacts |
| 2025-12-22 | [952bba4f](https://github.com/ErrorChen/SensorArray/commit/952bba4f2c1fa0393f9a1fc59e9b5d43934cbc43) | Structure |
| 2025-12-22 | [81fc8833](https://github.com/ErrorChen/SensorArray/commit/81fc8833f22574d57686b3a863af1b9dbfa79313) | Delete .vscode directory |
| 2025-12-22 | [613431ca](https://github.com/ErrorChen/SensorArray/commit/613431ca0695259ea4799441daa425200784fef4) | Merge branches 'main' and 'main' of https://github.com/ErrorChen/SensorArray |
| 2025-12-22 | [80a635ea](https://github.com/ErrorChen/SensorArray/commit/80a635ea0143f67329db3dc286bc71371e297ceb) | FDC2214 |
| 2025-12-24 | [0d2f4029](https://github.com/ErrorChen/SensorArray/commit/0d2f402995e0997858de384d8dc45869c14ee391) | Tmux |
| 2025-12-25 | [dc68a535](https://github.com/ErrorChen/SensorArray/commit/dc68a53512dba0a88370dc63773cbdf8e693cdae) | add some datasheets |
| 2025-12-25 | [10d36519](https://github.com/ErrorChen/SensorArray/commit/10d36519752d57ea2835ee6b034dd4fa0fcdf130) | ads126x |
| 2025-12-25 | [d742237f](https://github.com/ErrorChen/SensorArray/commit/d742237f0e5cc9d0fcf510562b1bb3a44ebb23fd) | debug |
| 2025-12-25 | [e1b4d728](https://github.com/ErrorChen/SensorArray/commit/e1b4d72859192731881d4476baab946e955d44f2) | updated |
| 2025-12-25 | [0db827ed](https://github.com/ErrorChen/SensorArray/commit/0db827ed1ebc288b368275e2ffb45d670956166a) | Structure change |
| 2025-12-25 | [9a447b69](https://github.com/ErrorChen/SensorArray/commit/9a447b693e5326c382ddb69b3bd7104a309fab22) | boardSupport and some updates |
| 2025-12-25 | [9be61f29](https://github.com/ErrorChen/SensorArray/commit/9be61f2932ff4a79e77efe91f77db9a6603a3b9e) | Matrix Control |
| 2025-12-25 | [f075561d](https://github.com/ErrorChen/SensorArray/commit/f075561d7962fe14411adcda5e6d282c98d355b2) | addReadme |
| 2025-12-25 | [6ec9f910](https://github.com/ErrorChen/SensorArray/commit/6ec9f9103ac9a3bedd737421a93957a1a085f4d7) | examples |
| 2025-12-25 | [b6b28327](https://github.com/ErrorChen/SensorArray/commit/b6b28327465c764356236f511d8227cc94d555a6) | protocolWire |
| 2025-12-26 | [411c8434](https://github.com/ErrorChen/SensorArray/commit/411c8434c6d5ab47b3d74a4e76c6ea91c687cb83) | USB |
| 2025-12-26 | [0225f11c](https://github.com/ErrorChen/SensorArray/commit/0225f11cb78cfcaef5eb54fbfad1f6dcb86798e5) | update |
| 2025-12-26 | [04cdd965](https://github.com/ErrorChen/SensorArray/commit/04cdd96562e182c20562294b542bab0975fdc6c0) | main.c |
| 2025-12-26 | [ef2b5452](https://github.com/ErrorChen/SensorArray/commit/ef2b5452cd69a7c8f765abaca2de8bc91572095c) | debugging |
| 2025-12-26 | [3e83d449](https://github.com/ErrorChen/SensorArray/commit/3e83d449741682daaf2987a16b9c98b667c924cb) | Only S3 |
| 2026-01-20 | [cc9bfb7a](https://github.com/ErrorChen/SensorArray/commit/cc9bfb7a38234d20fb1aaa14c0b594f93297cd81) | add check fuction |
| 2026-03-09 | [be450780](https://github.com/ErrorChen/SensorArray/commit/be4507807c3a421868bc33b7fdf654d2c9ea2e8b) | Debug |
| 2026-03-09 | [a540cae9](https://github.com/ErrorChen/SensorArray/commit/a540cae953638494ffdceee1bb31e996fdbe753e) | upgrade |
| 2026-03-09 | [6324da17](https://github.com/ErrorChen/SensorArray/commit/6324da1759e7aca7aadfe5162d8fb06b24c0dec1) | debug |
| 2026-03-09 | [8a49a4df](https://github.com/ErrorChen/SensorArray/commit/8a49a4dfa03f32f61b2d8d2f825d03d0d9a8e4a6) | Remap |
| 2026-03-10 | [39b61beb](https://github.com/ErrorChen/SensorArray/commit/39b61beb84fe5ba75b9fe887f00caca720b6c8f3) | In order to solve S-D -1.8v |
| 2026-03-10 | [5555f4c5](https://github.com/ErrorChen/SensorArray/commit/5555f4c50657d3610f4b4956041fdb5298428aee) | change |
| 2026-03-10 | [e2d1d8eb](https://github.com/ErrorChen/SensorArray/commit/e2d1d8eb935bb3bc3831c0bbd5b1e93390db70f5) | NewVer |
| 2026-03-10 | [1bdba9d9](https://github.com/ErrorChen/SensorArray/commit/1bdba9d9b876557e0615d96e4b0fd79f24506a56) | git |
| 2026-03-10 | [60cd9a2e](https://github.com/ErrorChen/SensorArray/commit/60cd9a2ee55ae9b2676c9d33393301f439579397) | OnlyS1D1 |
| 2026-03-10 | [a93744a8](https://github.com/ErrorChen/SensorArray/commit/a93744a8eb2d9ae7821aaa512f075c3f9bab8db7) | update |
| 2026-03-11 | [41ecb5c3](https://github.com/ErrorChen/SensorArray/commit/41ecb5c3fa0a32d67aea8396b9a7b1376e657fed) | NewVersion |
| 2026-03-23 | [ab469188](https://github.com/ErrorChen/SensorArray/commit/ab4691880a4b75836cba861c161c7656662d042b) | debugFor1134 |
| 2026-03-24 | [74f9eec7](https://github.com/ErrorChen/SensorArray/commit/74f9eec7bb609ff6af4a5cd120393e26e329b35c) | debugADS1263 |
| 2026-03-24 | [027c5567](https://github.com/ErrorChen/SensorArray/commit/027c556794a0826567d4e39ab3b496d9919d0134) | selA |
| 2026-03-24 | [f5bbee53](https://github.com/ErrorChen/SensorArray/commit/f5bbee536ec37788fd42cc4007dd3458e5e6cc29) | selA |
| 2026-03-24 | [9c206b7b](https://github.com/ErrorChen/SensorArray/commit/9c206b7b516ff570ace392fd3b69b4305d56010a) | selA |
| 2026-03-25 | [1b65098b](https://github.com/ErrorChen/SensorArray/commit/1b65098bcc6ce1a90ae2f3ff8c613f449406ed5b) | GoIntoFDC2214 |
| 2026-03-26 | [166cb820](https://github.com/ErrorChen/SensorArray/commit/166cb820fafbdb951087726c7607a89fab6e6580) | ffdc2214 |
| 2026-04-01 | [180ff958](https://github.com/ErrorChen/SensorArray/commit/180ff958e751792be022487ddc0430f3aa46ce57) | CallFDC2214 |
| 2026-04-01 | [d99c9b6e](https://github.com/ErrorChen/SensorArray/commit/d99c9b6ecd17b1b9557cf4474227b0927c62fc1b) | i2c 400kHz |
| 2026-04-01 | [e82c99d8](https://github.com/ErrorChen/SensorArray/commit/e82c99d87ca2313828953c8952c29e7f13a89f6e) | 300kHz |
| 2026-04-01 | [687e0ab1](https://github.com/ErrorChen/SensorArray/commit/687e0ab10f1454d0c901ebc820c056a92bcf0520) | 350kHz |
| 2026-04-01 | [57a80ff1](https://github.com/ErrorChen/SensorArray/commit/57a80ff111667dbf0b0cc57ca4502bc2204351bb) | 375kHz |
| 2026-04-01 | [f4cadf4c](https://github.com/ErrorChen/SensorArray/commit/f4cadf4ce813e5c3665779197a8442bbca69b3a3) | 362.5kHz |
| 2026-04-01 | [10399f60](https://github.com/ErrorChen/SensorArray/commit/10399f60c3289b9bd4649a7b0e11674dceff3c07) | 350kHz is the highest frequency |
| 2026-04-01 | [9d2f6507](https://github.com/ErrorChen/SensorArray/commit/9d2f65075bcc4bc93e40f964ae0a784ccdacee11) | Common use 325kHz |
| 2026-04-01 | [3f1502f9](https://github.com/ErrorChen/SensorArray/commit/3f1502f9b2bb500b73c8f97d5290b679928b32ec) | TestFDC2214 |
| 2026-04-01 | [ba5d7ba0](https://github.com/ErrorChen/SensorArray/commit/ba5d7ba0bdf5d194a49845a134d47ffa0f2ae71d) | Debug |
| 2026-04-01 | [5e070b97](https://github.com/ErrorChen/SensorArray/commit/5e070b97b1b564272266bb39440f10b25b51883a) | BigDebug |
| 2026-04-01 | [25930c52](https://github.com/ErrorChen/SensorArray/commit/25930c52d3f2c8d6d1dc5569e207b2f8ea336a62) | i2c_frequency to 325kHz |
| 2026-04-19 | [035cce29](https://github.com/ErrorChen/SensorArray/commit/035cce2956aefd1a719f5d84f710d48872526714) | Add S5D5 FDC2214 drive-strength sweep debug logging |
| 2026-04-20 | [2a7aca14](https://github.com/ErrorChen/SensorArray/commit/2a7aca14d478b599b9d0b0c70436fae9ca067881) | debug |
| 2026-04-20 | [bb2b9c00](https://github.com/ErrorChen/SensorArray/commit/bb2b9c00706c3c022d9e4916174d2398525dc47b) | de |
| 2026-04-20 | [153bba63](https://github.com/ErrorChen/SensorArray/commit/153bba63c22607f9ec90c6c80a751c6a9367b08e) | debug |
| 2026-04-28 | [d842692b](https://github.com/ErrorChen/SensorArray/commit/d842692b03ad1d3d2d4794795ee986af691510ee) | FDC2214CanRunThisVer |
| 2026-05-21 | [07607c4f](https://github.com/ErrorChen/SensorArray/commit/07607c4ff3e5c343a39f6835cbc3008cad8d111a) | comeToFDC |
| 2026-05-21 | [e8213884](https://github.com/ErrorChen/SensorArray/commit/e82138843aaab89fd0635b3c5a1cb3395ec5f178) | 修复freq |
| 2026-05-21 | [fd192d65](https://github.com/ErrorChen/SensorArray/commit/fd192d65e29251795c1a2eec76edcc688666b624) | ShutdownRefInFDC |
| 2026-05-23 | [1c891518](https://github.com/ErrorChen/SensorArray/commit/1c891518a0c174171a269a499609069e80688d92) | S5D5 sweep 扩展为 DEGLITCH x HIGH_CURRENT_DRV x DRIVE_CURRENT_CH0 三维扫描 |
| 2026-05-25 | [4dfa5281](https://github.com/ErrorChen/SensorArray/commit/4dfa52814b9054ea161b6f4f42a2d1f46f1ff50d) | 主要改动在 sensorarrayDebugSelftest.c (line 1713) |
| 2026-05-25 | [09d86075](https://github.com/ErrorChen/SensorArray/commit/09d86075623ea83fe359ef2ad447379f8cffebae) | Revert "主要改动在 sensorarrayDebugSelftest.c (line 1713)" |
| 2026-05-25 | [b0d150c2](https://github.com/ErrorChen/SensorArray/commit/b0d150c2bf03fbfd4225ce9b83857856e0818b41) | 已完成。主要改动在 main/sensorarrayDebugSelftest.c，main/sensorarrayConfig.h，main/Kconfig.projbuild；main/sensorarrayDebugFdcSelbS5d5.c 保持 thin legacy wrapper，未加入复杂逻辑。 |
| 2026-05-25 | [a8649a77](https://github.com/ErrorChen/SensorArray/commit/a8649a779fc2c933feafde58fe81c7e9a3de7018) | 针对性修复 |
| 2026-05-25 | [56487d64](https://github.com/ErrorChen/SensorArray/commit/56487d6400aba90391df8f5653c290bf5d8930cd) | 默认启动路径现在不再依赖 boot baseline，也不再跑 full sweep：sweep_enter -&gt; direct 10MHz lock -&gt; locked sample loop |
| 2026-05-26 | [8ceb2a5b](https://github.com/ErrorChen/SensorArray/commit/8ceb2a5bfce0d151cf08de87ad860990dbfba354) | targeted fix |
| 2026-05-26 | [240af244](https://github.com/ErrorChen/SensorArray/commit/240af2440e1c9f3bd5a13dfe7f44c7b8fd0eddda) | 默认开启 normal sweep、sweep trace、per-step timeout、total timeout、I2C timeout recovery、minimal log。 |
| 2026-05-27 | [1dc10f49](https://github.com/ErrorChen/SensorArray/commit/1dc10f49b0ad0014575cfb4a341f2ea9621405bf) | 架构调整 |
| 2026-05-27 | [2505547b](https://github.com/ErrorChen/SensorArray/commit/2505547b11f48554abf1bfe38db5d51bb4b47825) | 新增 sensorarrayS5d5FeedTaskWdtIfSubscribed() 修改 sensorarrayS5d5ServiceScheduler() |
| 2026-05-27 | [f78f87d4](https://github.com/ErrorChen/SensorArray/commit/f78f87d4fb692c6a6b3688440c431e3ce3f8ae2b) | FDC40MHz |
| 2026-05-27 | [72e176f4](https://github.com/ErrorChen/SensorArray/commit/72e176f4ca2720e408696d23aaeb1e62050c2aa2) | relock 逻辑 |
| 2026-05-28 | [fc5fd7bf](https://github.com/ErrorChen/SensorArray/commit/fc5fd7bffc3a5d82daf83e75c5138d72d85eb0a2) | FromS5D5ToMatrix |
| 2026-05-28 | [bb378227](https://github.com/ErrorChen/SensorArray/commit/bb37822758870f740e08cfda5de6df1de3516e78) | FDC sweep |
| 2026-05-28 | [a05c56f1](https://github.com/ErrorChen/SensorArray/commit/a05c56f1ca719f9540197f1e17a0756a63babfb9) | matrix 读取改为逐 cell 策略 |
| 2026-05-28 | [f8fa7509](https://github.com/ErrorChen/SensorArray/commit/f8fa7509b8e52bc499e16f4522102254062943cc) | sensorarrayAppRun() |
| 2026-05-28 | [1bb3e511](https://github.com/ErrorChen/SensorArray/commit/1bb3e511f4a4fd00c9ed1b322b202cc60c7ce534) | 增加 APP_STACK high-water 日志，并把 main task stack 提到 16384 |
| 2026-05-29 | [5d15854c](https://github.com/ErrorChen/SensorArray/commit/5d15854ca2938839efd9409db01d9425dc8161e7) | 改成按行读取，两颗 FDC2214 各自 CH0-CH3 autoscan |
| 2026-06-01 | [4fd31814](https://github.com/ErrorChen/SensorArray/commit/4fd3181479b5d0295d1cc2e8bd7ce7aa8b762049) | 普通 channel-level 改为 cell-level |
| 2026-06-02 | [441673a1](https://github.com/ErrorChen/SensorArray/commit/441673a13993efcfa7e9027dbab1e2226be08888) | S 行只 route 一次，然后 primary/secondary FDC 都进入 CH0-CH3 autoscan 并读取 4 通道数据。 |
| 2026-06-02 | [fe2c2e35](https://github.com/ErrorChen/SensorArray/commit/fe2c2e3541e3ef32b3e781c241e3de005cf31699) | 核心改动集中在 FDC runtime 架构 |
| 2026-06-02 | [6de9e33a](https://github.com/ErrorChen/SensorArray/commit/6de9e33adad8d5d73fe813c91ea0eace3da542e4) | FDC matrix 默认目标改为 20 fps / 50 ms |
| 2026-06-02 | [36f68c2d](https://github.com/ErrorChen/SensorArray/commit/36f68c2d6882c0b0d824e02046ccf4c419ef4674) | 重构修改，主要落在 core/measure/sensorarrayMeasure.c |
| 2026-06-03 | [a01e9114](https://github.com/ErrorChen/SensorArray/commit/a01e91149f2a28180ffdf01c61cd761b1c55097e) | 新增 worker core 解析 |
| 2026-06-03 | [d2a76f48](https://github.com/ErrorChen/SensorArray/commit/d2a76f480c76287086560c22a0ae8a0638ba8276) | 修改文件： sensorarrayMeasure.c sensorarrayTypes.h sensorarrayApp.c sensorarrayFdcSweep.c |
| 2026-06-03 | [ed31be84](https://github.com/ErrorChen/SensorArray/commit/ed31be84b9659143b0f28b39098abbf84ffeefc4) | 在 sensorarrayMeasure.c (line 2691) 将 formal worker ready 改为 STATUS polling-only，只有 unreadMask == 0xF 才进入正常 read4。 |
| 2026-06-03 | [e71c3229](https://github.com/ErrorChen/SensorArray/commit/e71c322985919c3dbe844306e4dfbb33ff9f94d8) | main/main.c 已改成真正顶层调度器，不再调用 sensorarrayAppRun()；生命周期现在在 main.c (line 528) / app_main (line 537) 能直接看到。 |
| 2026-06-03 | [c1aead40](https://github.com/ErrorChen/SensorArray/commit/c1aead406b1b8b042b1fd1567d827b122f4d4a31) | boardSupport.c: validates I2C ports before all ESP-IDF I2C calls, adds BOARD_I2C_CFG, BOARD_I2C_INIT, BOARD_I2C_READY, reject/recover logs, and primary-only secondary dow… |
| 2026-06-04 | [b20b9424](https://github.com/ErrorChen/SensorArray/commit/b20b94246c603924bf40d507da0ccb2dc747e9a3) | 当前 panic 不是 FDC2214 NACK、不是 sweep、不是正式矩阵读取、也不是普通 I2C timeout。它发生在 I2C0 legacy i2c_driver_install() 申请 interrupt 时，调用链进入 esp_intr_alloc/find_desc_for_source。因为这是 driver 内部… |
| 2026-06-04 | [5c22f780](https://github.com/ErrorChen/SensorArray/commit/5c22f780d53991d4a8b220b4d6accaf6eb5d1770) | 迁移 |
| 2026-06-04 | [4aa79118](https://github.com/ErrorChen/SensorArray/commit/4aa7911869612abc9edd23785950a4305315a6af) | readme |
| 2026-06-04 | [27451d22](https://github.com/ErrorChen/SensorArray/commit/27451d22905c2289f82b2d6ed689cebf4563b2ea) | readme Rewrite |
| 2026-06-04 | [a08d2fb1](https://github.com/ErrorChen/SensorArray/commit/a08d2fb106250f1d487b0d147cb5edccf13e5364) | Implemented the FDC fixes |
| 2026-06-04 | [fd6b30d9](https://github.com/ErrorChen/SensorArray/commit/fd6b30d995591f677ecc281ea4ec915e2994638f) | ready wait 不再同时计入 sleepExitToIntbUs；新增 worker trace、late/stale discard、duplicate read 诊断；parallel timeout 后只 repair 失败 device，不再整行重跑已成功 device。 |
| 2026-06-05 | [c3fa723d](https://github.com/ErrorChen/SensorArray/commit/c3fa723d61133b82a94bf2a038f1361c8496754c) | sensorarrayFdcWaitDeviceReady() 原来把 unreadMask == 0xF 当成 full ready，没有要求 DataReady/DRDY=1；随后 freshMask = outReady-&gt;unreadMask 传给 read4，requestedFresh 被置真，导致 status=0x000F… |
| 2026-06-09 | [47b71e27](https://github.com/ErrorChen/SensorArray/commit/47b71e27cd0ed58ca9f904a66fc8d7b777d57b6c) | INTB 输出默认启用 |
| 2026-06-09 | [004ebda1](https://github.com/ErrorChen/SensorArray/commit/004ebda19467ec6f117acc68fa5b31e84de3708e) | 修正 FDC ready 语义：INTB 只作为 wake hint，最终只以 DRDY=1 && required unread full 允许读 DATA。 unread=0xF, DRDY=0 改为 WAIT_DRDY 中间态，不再走 UNSAFE_U/hard invalid。 新增 sensorarrayFdcRead4IsDa… |
| 2026-06-09 | [50e1c356](https://github.com/ErrorChen/SensorArray/commit/50e1c356a09a47b0a3c2ff1ddcc88d75225ea986) | FDC ready 改为默认 INTB_STRICT_LEVEL，严格先等 INTB，再读一次 STATUS ack/verify；正式路径不会走 src=FB，除非显式切到 legacy fallback。见 sensorarrayFdcRowEpoch.inc (line 843)。 增加 after-INTB recheck src… |
| 2026-06-10 | [c24d5a6f](https://github.com/ErrorChen/SensorArray/commit/c24d5a6f0bd916b383e9754e33d51151075fd732) | sensorarrayFdcRowEpoch.inc (line 1010)：重做 ready wait 预算，加入 actualIntbWaitUs = max(INTB_WAIT_TIMEOUT, estimatedRoundUs + ROW_WAIT_SAFETY)，按 row-device hard deadline clamp；… |
| 2026-06-10 | [98bd9765](https://github.com/ErrorChen/SensorArray/commit/98bd97657de057932e226c411567ab328e5e4585) | 4000 us 现在只是 formal fast target。默认关闭时只用于 PFU/P5 诊断，不再把 13544 强制压成 4000，也不再改 expected.rCount。 |
| 2026-06-10 | [ee06de43](https://github.com/ErrorChen/SensorArray/commit/ee06de43635b644966882215c970b7f8eb9e64e9) | 重做 FDC row-device ready 分类：区分 OK、recovered、transient unread/no-DRDY、stale unread/no-DRDY、hard timeout、worker epoch mismatch。 unread=0xF,DRDY=0 不再直接变成 hard timeout/rescue：… |
| 2026-06-10 | [bfe6f1e4](https://github.com/ErrorChen/SensorArray/commit/bfe6f1e4879eaf0646305e8fd722c08229282d31) | 删除 finalPoll 参数、sensorarrayMeasureFdcCaptureFinalReadyPoll、guard final repoll、final retry、statusReadsFinalPoll/finalPollAttempted/finalRetry* 以及 D4/SR 的 fp/fr/fSt/... 日志字… |
| 2026-06-10 | [998e6ab5](https://github.com/ErrorChen/SensorArray/commit/998e6ab55182fa106ee862e1f12144564c29f5e5) | sensorarrayFdcRead4.inc (line 45)：STATUS 显示 DRDY=1 且 unreadMask 满足读取需求时，覆盖 INTB miss/timeout/diagOnly，不再跳过 DATA read。 sensorarrayFdcRowEpoch.inc (line 1089)：RWD_DIAG_STAT… |
| 2026-06-11 | [769dc2af](https://github.com/ErrorChen/SensorArray/commit/769dc2afede13060302d4500da183291b7b6b076) | 在 sensorarrayFdcRowEpoch.inc 增加单次 STATUS pre-check：ready 时走 src=SP / ready_before_wait，不计 originalIntbMiss/originalTimeout。 INTB timeout 后改为单次 STATUS 分类：STATUS ready 走 sr… |
| 2026-06-11 | [69f3d42d](https://github.com/ErrorChen/SensorArray/commit/69f3d42dfd2b3bbb8e80953e89b23088776d5195) | sensorarrayFdcRowEpoch.inc：删除 pre-wait STATUS read；guard 前及 guard 后到 hard deadline 仅等待 INTB level/notification；加入 STI/STT/STH/STM/RWT/RWS。 • sensorarrayFdcRead4.inc：仅接受明确… |
| 2026-06-11 | [cb39c20d](https://github.com/ErrorChen/SensorArray/commit/cb39c20d5041f6c420d727869c4a650a81a5f51e) | • INTB-first 正常路径稳定运行，等待期间无 STATUS poll。 • RP、WAIT_DRDY、before_estimated_round_transient：均为 0。 • 无 worker timeout、late-good、并行降级。 • Q5 持续：nr=0, softInvalid=0, hardInvalid… |
| 2026-06-11 | [48da5349](https://github.com/ErrorChen/SensorArray/commit/48da5349ff55e4d20b419498601c6606908868b9) | 重写 row-level 并行路径：parent 直接读取 primary，worker 同时读取 secondary，并增加 ready/start barrier、generation/epoch 校验和 WP 日志。 |
| 2026-06-11 | [89ca3462](https://github.com/ErrorChen/SensorArray/commit/89ca34620e0226d82238255c0bea4499e947e6bb) | 新增异步日志任务：sensorarrayAsyncLog.c (line 426)，主循环只发布 frame snapshot，不再同步格式化 64 个 pF 文本；LOG20 汇报采样/输出分离、队列深度和 drop 计数。 FDC2214 正式读取改为 ordered DATA_CHx -&gt; DATA_LSB_CHx：fdc2214C… |
| 2026-06-11 | [399e47cf](https://github.com/ErrorChen/SensorArray/commit/399e47cfcd9fb9c29c7df5940bd61d4a5ba5a3b3) | 心改动是把 FDC row 并行路径从“coordinator 读 primary + worker 读 secondary”改成真正对称的双 worker barrier 模型：sensorarrayFdcRowEpoch.inc (line 3946)。coordinator 现在只做 row select、settle、cache/… |
| 2026-06-12 | [cf63c186](https://github.com/ErrorChen/SensorArray/commit/cf63c186b8c80ab982a9791d673667aa696166fd) | 20 FPS 未达到，稳态为 12.36 FPS。原因是单行耗时约 9674 us，其中 ready 等待 4773 us、数据读取 2317 us；单总线每帧实测 I2C 时间约 24.6 ms，明显高于理论 9.36 ms。瓶颈是 FDC ready 和 I2C driver/wrapper 开销，不是线程偏斜或通信错误 |
| 2026-06-12 | [dcdb0c81](https://github.com/ErrorChen/SensorArray/commit/dcdb0c81df3c1b896f379e678b7954a5ea6e653c) | fresh frame/epoch/mask 约束与正确 frameId 递增。 INTB direct DATA + STATUS fallback。 burst runtime probe、ordered-read 优化、I2C 负载扫频。 row cache compare/diff-only apply。 异步 Cap 输出、SA… |
| 2026-06-12 | [ca7b5991](https://github.com/ErrorChen/SensorArray/commit/ca7b5991f847700f278dfa5856e3f597c886c2f0) | fdc2214Cap.c、fdc2214Cap.h ◦ 实现 ordered sequence read：seqPair4/seqQuad2/seqAll1。 ◦ 启动期 fixed-register + frozen DATA exact-match probe。 ◦ sequence 错误自动回退 ordered8。 • boardS… |
| 2026-06-12 | [8b00baf2](https://github.com/ErrorChen/SensorArray/commit/8b00baf2c380d944f413ee51f6cd5a451f3a1d6b) | 双核架构 • Core 1：acquisition scheduler、TMUX、FDC workers、FDC I2C、ADS SPI/START/DRDY、原始帧提交。 • Core 0：统计、precisionGuard、日志格式化、USB 输出、Wi-Fi/BLE、状态发送。 • 使用预分配槽位和 SPSC/drop-old 队列… |
| 2026-06-12 | [4ca65acd](https://github.com/ErrorChen/SensorArray/commit/4ca65acdd08e448ed5f449f85bf6f1d28cd2c078) | ignore log |
| 2026-06-15 | [a17e12ad](https://github.com/ErrorChen/SensorArray/commit/a17e12adca2ad7f105afef2868bad372725788c6) | Core1 采集与 Core0 输出解耦，USB/WiFi/BLE 独立队列。 统一 C/D/K ASCII 协议、CRC32、事件环和命令邮箱。 修复 ADS1262/1263 识别、校准、ADC2 支持及电池计算。寄存器实现依据 TI ADS1263 数据手册。 增加串口、UDP、BLE 接收和协议验证工具。 更新配置与 [READM… |
| 2026-06-15 | [4d90fa1e](https://github.com/ErrorChen/SensorArray/commit/4d90fa1ead203acce3c2c413c35e4460d197ff07) | 异构传输程序 |
| 2026-06-16 | [da2efd97](https://github.com/ErrorChen/SensorArray/commit/da2efd9741ecb1d8a30906db699ebcf15f26e806) | 主要完成项： • Kconfig 已拆分到 main/kconfigs/，main/Kconfig.projbuild 只保留 source 入口；旧 compat 隐藏保留，业务旧配置直读检查通过。 • 新增 main/sensorarrayConfigDerived.h，关键运行策略改用 SENSORARRAY_CFG_*。 • FD… |
| 2026-06-16 | [67a11ee6](https://github.com/ErrorChen/SensorArray/commit/67a11ee6b38cbecee32dd38487522de3b38d72d9) | [core/measure/ads/sensorarrayAdsGap.c (line 131)](c:/ESP32/SensorArray/core/measure/ads/sensorarrayAdsGap.c:131)：重做 ADS rail/battery 采样链路，rail 使用 ADS126x analog-supply mo… |
| 2026-06-17 | [3820637e](https://github.com/ErrorChen/SensorArray/commit/3820637e0262334f293d548e3daafd441fb853f7) | half |
| 2026-06-18 | [b41c5256](https://github.com/ErrorChen/SensorArray/commit/b41c5256fbb5b23a0f0d98ed651db2f6ced3a0d6) | BLE 改成固定 slot + 小 descriptor 队列，避免 BTC task 栈上/队列里搬 1544B 对象；串口命令做严格 ASCII 规范化，并修正 legacy 命令先被误报 CMDERR 的顺序；GPIO ISR service 统一由 boardSupport 拥有；FDC 默认恢复 fast_runtime / R… |
| 2026-08-05 | [191db117](https://github.com/ErrorChen/SensorArray/commit/191db11789a86c5d46a2004bf238ee9d1571dd5c) | Add resistence array and Voltage array |
| 2026-08-06 | [b5c9b5c9](https://github.com/ErrorChen/SensorArray/commit/b5c9b5c9e07d9e4b53fa9dc5d99326ad686b67d8) | You've hit your usage limit. |
| 2026-08-06 | [4e24cbcf](https://github.com/ErrorChen/SensorArray/commit/4e24cbcfab1155dd5995f07f506a3d77e04cb348) | 四层 cache：新增 register shadow、按 mode/cell 隔离的 PGA/bypass profile、带 25 mV 阈值和 5 mV 滞回的 rail fingerprint、value/noise cache。cache 定义 • 完整扫描仍是 row=1..activeRows、dLine=1..8，没有四点… |
| 2026-08-07 | [22b7709e](https://github.com/ErrorChen/SensorArray/commit/22b7709e70ba0fdf59e6e947624139146cc4bac0) | Harden transport and BLE stability |
| 2026-08-13 | [a4889e44](https://github.com/ErrorChen/SensorArray/commit/a4889e44bdac207a708e7c5bc2a0da546f480f41) | logger 任务与 USB sink 改为两阶段启动，避免 Core0/Core1 状态竞态。 logger 使用 PSRAM，启动不再因堆碎片失败。 电池断开、AIN8 出现 range_error/unstable/PWM 时保留 last-known-good，不触发硬故障。 VOLT 模式电源轨过期前自动安全刷新。 串口输出抽样… |
| 2026-08-13 | [b59209cd](https://github.com/ErrorChen/SensorArray/commit/b59209cdea479cabb099d80d92919a379173aef8) | 电池异常现在仅发布普通异步 BATERR，零等待、不进入高优先级 lifecycle 队列，不触发 SAFE，也不影响矩阵/控制流程。 ADS 电池转换稳定时间从 500 µs 缩短为 50 µs；DRDY 超时保持独立的 600 µs。 现场 BATD 已确认：settleUs=50、discardCount=1、restore=ok、… |
| 2026-08-13 | [331c4458](https://github.com/ErrorChen/SensorArray/commit/331c44589318db9ba642cf3ab33bb08ca3dd8a34) | 已完成，实机验收 PASS。 BLE 60 秒：1440 帧，60.004 秒，23.998 fps 矩阵、FAST/SAFE 切换、订阅压力、2 次重连全部通过 resets=0、panics=0、无异常故障 正常电池电压约 4108–4110 mV，BATERR=0、MFAULT=0 最终串口 smoke：SERIAL_HIL_PAS… |
| 2026-08-15 | [8cfd16e8](https://github.com/ErrorChen/SensorArray/commit/8cfd16e893712b11653eaf200342be98d17663f2) | fix: harden mixed measurement and recovery lifecycle |
| 2026-08-17 | [8045e9e9](https://github.com/ErrorChen/SensorArray/commit/8045e9e9ec9599533c52c15dfcb6002f79fd15f1) | FDCswich |
| 2026-11-04 | [ba8346b7](https://github.com/ErrorChen/SensorArray/commit/ba8346b7b643e54985699630f608aba3386810e9) | fix: bound frontend recovery and preserve authoritative frame identity |
| 2026-11-04 | [1ff7b0ab](https://github.com/ErrorChen/SensorArray/commit/1ff7b0ab6e915a828a463386dc5d3497c4f3ad3c) | docs: publish reliability audit, host contract and verified fixtures |
| 2026-11-04 | [a0460b80](https://github.com/ErrorChen/SensorArray/commit/a0460b80f146b70e24e69ded8e6668969f250926) | fix: recover bounded ADC CRC faults and retain fault domains |
| 2026-11-04 | [8b0e6512](https://github.com/ErrorChen/SensorArray/commit/8b0e65124669d378f2613d147acf985fb4dda58b) | fix: expose battery admission faults and adjust S3 ADS sampling duty |
| 2026-11-05 | [6a8f5029](https://github.com/ErrorChen/SensorArray/commit/6a8f5029feadd455a66f5dedb226d6e554073079) | test: expose valid ADC CRC parity coverage for hardware trials |
| 2026-11-05 | [8d6d656a](https://github.com/ErrorChen/SensorArray/commit/8d6d656a6272af7f19e338b594e98d582762737e) | fix: configure the ADS supply monitor and guard register reads |
| 2026-11-05 | [3fb2a19a](https://github.com/ErrorChen/SensorArray/commit/3fb2a19a19b054ca9621ef089d961806a4740478) | fix: serialize USB output and complete rail transactions before mixed frames |
| 2026-11-05 | [bd491682](https://github.com/ErrorChen/SensorArray/commit/bd491682a8b2027a09831b62147cd92ee782ce7f) | revert: remove RDATA NOP experiment after hardware CRC recurrence |
| 2026-11-05 | [ebabe7e6](https://github.com/ErrorChen/SensorArray/commit/ebabe7e6816c0347c6d1f5187d5f7f801a7ef98d) | fix: queue complete serial log lines and preserve control boundaries |
| 2026-11-05 | [a73df5e9](https://github.com/ErrorChen/SensorArray/commit/a73df5e9c8aaafb81be810326ef9ed19a3dec682) | docs: record fixture HIL evidence and unresolved ADC CRC acceptance |
| 2026-11-13 | [89564145](https://github.com/ErrorChen/SensorArray/commit/89564145ef3690d0afc042fc8214bbf72b402df7) | test: record repaired fixture and external rail contrast measurements |
| 2026-11-13 | [7e1d2c6d](https://github.com/ErrorChen/SensorArray/commit/7e1d2c6dd9c85883ce9018fbc89a5d2402410607) | docs: confirm rail measurements at ADS supply pins |
| 2026-11-13 | [ee8f972f](https://github.com/ErrorChen/SensorArray/commit/ee8f972f3198cd6879b3184f4d528f68dc6c29d3) | Qualify frontend bus clocks and restart after communication recovery |
| 2026-11-13 | [e339a1d5](https://github.com/ErrorChen/SensorArray/commit/e339a1d5b5e293d39649da887fed2b90bf9a575d) | Correct FDC presence register addresses and expose readback evidence |
| 2026-11-13 | [0d8f60fd](https://github.com/ErrorChen/SensorArray/commit/0d8f60fd9c29b9ce12e64ed2e6534ee37db41704) | Keep bounded ADC CRC rejection distinct from frontend faults |
| 2026-11-13 | [334fb2a0](https://github.com/ErrorChen/SensorArray/commit/334fb2a081d70731985a6c2d5c2b6f02548ec56e) | Record frontend qualification and CRC partial-frame HIL evidence |
| 2026-11-13 | [1f82b73a](https://github.com/ErrorChen/SensorArray/commit/1f82b73a1019930f419492e3e742a96bd43f374e) | Record USB and BLE benchmark failure baseline |
| 2026-11-13 | [fd0190a3](https://github.com/ErrorChen/SensorArray/commit/fd0190a3a175938e92e702a111ae9d35982d678f) | Fix bounded USB output, CPU0 work budgets and recovery control |
| 2026-11-13 | [b44c37b5](https://github.com/ErrorChen/SensorArray/commit/b44c37b51067e376e73aa44535ea47d175846b67) | Avoid per-record pacing after real CPU0 idle progress |
| 2026-11-14 | [4b665acf](https://github.com/ErrorChen/SensorArray/commit/4b665acf355adb4b145114abf5ecaa4325090be5) | fix: isolate USB data admission and batch BLE work checkpoints |
| 2026-11-14 | [d3148884](https://github.com/ErrorChen/SensorArray/commit/d31488848844667dfa88502bf1b894d7bfe571a2) | fix: separate worker quanta from idle progress and invalidate unstable PGA history |
| 2026-11-14 | [be09762e](https://github.com/ErrorChen/SensorArray/commit/be09762eadc6ffd908ec0375d1e41c354a9f7c23) | fix: service USB output ahead of continuously ready network senders |
| 2026-11-14 | [16f41fae](https://github.com/ErrorChen/SensorArray/commit/16f41faee878edfe7579378fab64079fc70cf8ed) | docs: preserve Serial JTAG repair evidence before OTG migration |
| 2026-11-14 | [7d46db08](https://github.com/ErrorChen/SensorArray/commit/7d46db080fce80c721384ad3b7d4f04555eb18f4) | feat: add DMA USB OTG data and control with prioritized BLE service |
| 2026-11-14 | [3dc0c48e](https://github.com/ErrorChen/SensorArray/commit/3dc0c48e92f080cad334cce50614d07c3df6e38c) | Add native OTG CDC data interface with bounded record arbitration |
| 2026-11-14 | [4a183bca](https://github.com/ErrorChen/SensorArray/commit/4a183bca6a5a0566a834a727a58d5e25a80ad31b) | Bound OTG ROM handoff and reject late cleanup acknowledgements |
| 2026-11-14 | [ae651337](https://github.com/ErrorChen/SensorArray/commit/ae6513373df95cc5660ef295235365afe63f74cb) | Record CDC handoff validation and preserve programming session |
| 2026-11-14 | [a485c475](https://github.com/ErrorChen/SensorArray/commit/a485c47533c0e6db864325ce7da6658ff3c19a40) | Record ordinary-user recovery access limits |
| 2026-11-16 | [3a55543c](https://github.com/ErrorChen/SensorArray/commit/3a55543cb21fa3412ede26ee79c1cce7d8371804) | Latch USB recovery and report complete firmware identity |
| 2026-11-16 | [0ff56888](https://github.com/ErrorChen/SensorArray/commit/0ff5688819a7cafe1716a55b630d7a8c5b25a584) | Record software USB recovery and failed dual-link acceptance screen |
| 2026-11-16 | [1e9fd197](https://github.com/ErrorChen/SensorArray/commit/1e9fd19799afa13bbf4584d986f5a797c57ec74d) | Align formal transport consumers with producer priority |
| 2026-11-16 | [bb230d03](https://github.com/ErrorChen/SensorArray/commit/bb230d03f438e724436881a7f61af35122726403) | Add bounded ADC conversion timing probe and BLE service observations |
| 2026-11-16 | [8b1ce9f8](https://github.com/ErrorChen/SensorArray/commit/8b1ce9f8a6612498e8338f822ad63ec71e8444c6) | Keep optional ADC diagnostics out of acquisition internal RAM |
| 2026-11-16 | [df68a468](https://github.com/ErrorChen/SensorArray/commit/df68a468a5394cf9e1f39574692cdcb9b3f5b5a4) | fix(ble): request negotiated 7.5 ms interval explicitly |
| 2026-11-16 | [bc997679](https://github.com/ErrorChen/SensorArray/commit/bc997679bcdc1db2df272914180bde50f16ff703) | feat(ads): add bounded physical final-bit timing diagnostic |
| 2026-11-16 | [4bf31dbc](https://github.com/ErrorChen/SensorArray/commit/4bf31dbc576cf5061e1694102f82ad2111ef5392) | fix(ads): admit peripheral-owned MISO and retain diagnostic IO gates |
| 2026-11-16 | [b13321a2](https://github.com/ErrorChen/SensorArray/commit/b13321a2c166715fa15f98c1677626786bec65b1) | fix: sample ADS final CRC bit under a verified read policy |
| 2026-11-16 | [0c94fe3b](https://github.com/ErrorChen/SensorArray/commit/0c94fe3bb58b25b0994ed2b718a8f5c33d456f4d) | diag: count BLE SDK confirmations without assuming peer receipt |
| 2026-11-16 | [167e0eae](https://github.com/ErrorChen/SensorArray/commit/167e0eaedcbed6216acd78d7be770c8f0be7a058) | docs: preserve final-bit repair experiments and BLE load evidence |
| 2026-11-16 | [b58336ee](https://github.com/ErrorChen/SensorArray/commit/b58336eeaa136d0aca23e4c62ac11eb292d5a904) | diag: publish bounded owner runtime snapshots for load analysis |
| 2026-11-16 | [1e918e67](https://github.com/ErrorChen/SensorArray/commit/1e918e677fd11af8ec110a7e4d0b544f869fdcda) | test: close CPU owner capture and recovery regressions |
| 2026-11-16 | [c6708970](https://github.com/ErrorChen/SensorArray/commit/c670897050e99e6d01c95b2da7ba55a57fe8e52b) | fix(ads): retain partial SPI allocation ownership locally |
| 2026-11-16 | [98b760c7](https://github.com/ErrorChen/SensorArray/commit/98b760c7a213e5b707d0580e326260cd482c10ff) | perf(output): remove unused legacy precision accumulation |
| 2026-11-16 | [b9ebe2f7](https://github.com/ErrorChen/SensorArray/commit/b9ebe2f7b8b49819347608392e70ff5966f2ad87) | test(ble): distinguish shared cleanup deadline exhaustion |
| 2026-11-16 | [7a3780a6](https://github.com/ErrorChen/SensorArray/commit/7a3780a6520659888dd38f8ea6d50228f78d41fd) | docs: preserve CPU profiling and CRC validation through screen 08 |
| 2026-11-16 | [dafaf9bc](https://github.com/ErrorChen/SensorArray/commit/dafaf9bc841e756b7d78aa54391703a952b819e5) | perf(transport): extract immutable frame metadata in one pass |
| 2026-11-16 | [0b371b64](https://github.com/ErrorChen/SensorArray/commit/0b371b6431557c4233b537211f5396fe4575f26f) | build: use supported O2 optimization with assertions enabled |
| 2026-11-16 | [54de309d](https://github.com/ErrorChen/SensorArray/commit/54de309d4fa710840c56baf4668a09e103bd7ac4) | docs: record metadata throughput gain and remaining BLE limit |
| 2026-11-16 | [a656d53f](https://github.com/ErrorChen/SensorArray/commit/a656d53f2efc447507f5247a116a22aed35dcc04) | build: preserve S3 task return checks under O2 |
| 2026-11-16 | [4ed6a47c](https://github.com/ErrorChen/SensorArray/commit/4ed6a47cb13d22b5b5577c0c8a1096ba954ccbcc) | docs: close O2 screen and retain continuous stability evidence rules |
| 2026-11-16 | [4edc99ae](https://github.com/ErrorChen/SensorArray/commit/4edc99ae73262e5febcf6f7cca12ffdfb87021d6) | perf(transport): use ROM CRC32 and exact fragment length arithmetic |
| 2026-11-16 | [caa90829](https://github.com/ErrorChen/SensorArray/commit/caa90829f34ffeda76dfe681e8e20bb29de4627c) | fix(ads): hold SCLK low explicitly before the sampled final bit |
| 2026-11-16 | [849d96cb](https://github.com/ErrorChen/SensorArray/commit/849d96cb03ba2089e353563f9c1a750c719434cb) | test: extract complete BLE planner dependencies for work budget checks |
| 2026-11-16 | [66012407](https://github.com/ErrorChen/SensorArray/commit/66012407de6a65b776b03dfd7fafba2861fac8e7) | docs: record continuous ten minute CRC validation and remaining BLE loss |
| 2026-11-16 | [53005f00](https://github.com/ErrorChen/SensorArray/commit/53005f0042a7a7694338f92c9b86ab3276db668a) | perf(ble): negotiate up to MTU 517 within existing packet limits |
| 2026-11-16 | [87d11082](https://github.com/ErrorChen/SensorArray/commit/87d110827e0183da4c1667b39427c50d9d7077d1) | docs: close ROM CRC optimization screen and preserve MTU experiment baseline |
| 2026-11-18 | [84bb978b](https://github.com/ErrorChen/SensorArray/commit/84bb978b9bacd1243520ea3990f91ed9f5f039d4) | Debug for ADS126x Crc Error |
| 2026-11-18 | [8a1794ce](https://github.com/ErrorChen/SensorArray/commit/8a1794ce3be95ff0007698c4e4e667dcf25fbe19) | test |
| 2026-11-23 | [ad0f9efb](https://github.com/ErrorChen/SensorArray/commit/ad0f9efbd62f0565a01966cbc85da54677fa374d) | BLE |
| 2026-11-25 | [f8d84244](https://github.com/ErrorChen/SensorArray/commit/f8d8424492c855a402838737ee8718e32f5135d6) | Free internal SRAM for SoftAP startup during BLE acquisition |
| 2026-11-25 | [9022cf3d](https://github.com/ErrorChen/SensorArray/commit/9022cf3d5663bb0ed0cf54c3fa8a324b4a8aa97a) | Record the SoftAP failure, memory repair and USB/BLE coexistence test |
| 2026-11-25 | [835313eb](https://github.com/ErrorChen/SensorArray/commit/835313eb5007aef1c1ab0c58cebd6f57d60e3c71) | Verify the 600-second run, 50 transitions and periodic full BLE snapshots |
| 2026-11-25 | [da68b407](https://github.com/ErrorChen/SensorArray/commit/da68b407833011fa14834a1620c773f76e3567be) | Reproduce formal USB loss during a 650 ms reader pause and preserve matrix evidence |
| 2026-11-25 | [f9a82f7a](https://github.com/ErrorChen/SensorArray/commit/f9a82f7a93e55f86064361579c8f721b8035d1de) | Buffer short formal CDC stalls and admit whole science records to TinyUSB |
| 2026-11-25 | [8cb5ea62](https://github.com/ErrorChen/SensorArray/commit/8cb5ea62d8a632ac693e25e9f971497fa2f74c37) | Publish USB fix provenance, host contract and original fixture measurements |
| 2026-11-25 | [b61707a8](https://github.com/ErrorChen/SensorArray/commit/b61707a8855284b789a891a3a9450a3713b448eb) | Verify whole-record USB delivery during the reproduced reader stall |
| 2026-11-25 | [efcb3585](https://github.com/ErrorChen/SensorArray/commit/efcb35857b75a1cf366a4b7cac2a70bd47a6426c) | Bind reader-pause comparisons and fixture windows to their raw recordings |
| 2026-11-25 | [11e0cb04](https://github.com/ErrorChen/SensorArray/commit/11e0cb043bde956a33ffed5f1cd3356545308f06) | Record all CAP and VOLT matrix windows and frame-level voltage evidence |
| 2026-11-25 | [5876b93d](https://github.com/ErrorChen/SensorArray/commit/5876b93d961a071d7e3370a6b1716d62010b09cf) | Measure both repaired 10k fixture connections and verify the held-out channel |
| 2026-11-25 | [398af885](https://github.com/ErrorChen/SensorArray/commit/398af885d71cdea7ee7c6047dda8f2b72e81ceac) | Verify 32 same-boot USB and BLE frame-rate configurations |
| 2026-11-25 | [93b9b690](https://github.com/ErrorChen/SensorArray/commit/93b9b690e666dc9aa1dc852db76e123156cfd9e5) | Verify ten-minute stability, transition receipts and periodic complete snapshots on the repaired image |
| 2026-11-25 | [dc5a5adb](https://github.com/ErrorChen/SensorArray/commit/dc5a5adb4bd62677221d0a2ceb0acf15886165c7) | Verify bounded unread USB, independent BLE service and SoftAP operation |

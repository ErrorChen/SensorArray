# 测试方法总表 / Test methods

本页收录从单点板级调试到正式 USB/BLE 整机验收的方法。按目标选入口，并区分 host 回归、实录回放、实机 HIL 和外部仪器。结果与失败时间线在[迭代记录](history/README.md)。 / Select the method by the question it can answer. Host tests, recordings, live hardware and electrical instruments provide different evidence. See the history for measured outcomes.

## 1. 构建、配置与无板回归 / Build, configuration and host regression

从项目根目录、ESP-IDF 5.5.5 的环境执行。选择一个新的构建目录，可保留正在调试的 ELF。主机 C 回归需要 C 编译器；硬件工具的 Python 依赖见 requirements-hil.txt。 / Run at the repository root with the S3 environment active. A new build directory preserves any ELF being used to decode a device. Use a native C compiler for the stand-alone production-code regressions.

```bash
idf.py --version
idf.py -B /tmp/sensorarray-validation-build -D IDF_TARGET=esp32s3 build
idf.py -B /tmp/sensorarray-validation-build size
python3 tools/check_command_docs.py
python3 tools/check_no_legacy_config_usage.py
python3 tools/audit_config_surface.py
for test in tools/test_*.py tools/hil/test_*.py; do
    CC="$(command -v cc)" python3 "$test" || exit 1
done
```

`tools/test_*.py` 中既有直接运行的断言，也有编译并调用生产 C 的入口。保留各脚本的退出码，不能以单独运行 pytest 的收集数代替。/ Run each script as a program. Some compile and exercise the production C with controlled clocks, buses or queues. Retain the failing entry and exit status.

| 问题 / Question | 重现方法与判据 / Method and interpretation |
|---|---|
| 配置/协议 / Configuration and wire | 运行 test_config_identity、test_text_protocol、test_transport_frame_metadata。检查 encoder 原始字节、header/K 的身份、CRC、1/8 行、极端长度以及丢终态/迟到数据。 / Replay the real encoder, malformed/truncated records and request orderings. Check identities within one session and one domain. |
| FDC worker | 运行 test_fdc_worker_lifecycle、test_fdc_worker_runtime、test_frontend_wait_integration。延迟 JOB/ACK/RUN/DONE，模拟无返回 worker，核对 epoch、隔离及释放。 / Inject delays, stale completion and bus stalls, then assert bounded return and safe slot ownership. |
| ADS CRC/健康 / ADC health | 运行 test_ads_matrix_diagnostics、test_ads_fault_fixtures、test_ads_driver_reliability、test_ads_crc_frame_flow 和 test_adc_read_provenance。按实际 engineFrameCount 检查 128/256/384/512 及非零入场计数，分别注入比较、读回、restore 失败。 / Inspect original error, stage/mask and whether a conversion started. Count bad ADC reads over checked operations separately from incomplete output frames. |
| 模式/恢复 / Mode and recovery | test_main_config_commit、test_rowmodes_frame_metadata、test_recovery_control、test_restart_budget；注入跨 boot 和迟来的旧 epoch。 / Assert the first applicable boundary, identity snapshots and bounded retry/restart budget. |
| 传输/内存 / Transport and memory | test_usb_cdc_capture、test_serial_io、test_protected_frame_pipeline、test_ble_watchdog、test_wifi_owner、test_memory_diagnostics。使消费者延迟、拒收和晚返回，检查终态优先、对象所有权及计数。 / Exercise late consumers, partial writes, starvation, DMA ownership and cached terminals. |

## 2. FDC、ADS 与电源恢复 / Frontend qualification

| 方法 / Method | 施加条件、观测与限度 / Setup, observations and limits |
|---|---|
| S5D5 单点及矩阵 sweep | 对历史板级版本，逐项记录 FDC 时钟、DEGLITCH、HIGH_CURRENT_DRV、DRIVE_CURRENT_CH0、RCOUNT、锁定及 raw28；先单点再逐行。 / Sweep one input and then both four-channel devices. Retain valid/fresh masks, reference clock and settling conditions. |
| I²C/SPI 扫速与三器件失联 | 记录启动前后 SPI?、两 FDC 的身份和实际时钟，逐档比较样本分母、CRC、读回失败。授权条件下断/复前端电，确认低速探测、全部缓存失效及启动重选。 / Compare achieved clock and error counts per candidate. On a controlled frontend power cycle observe loss, low-rate polling, new qualification and first fresh frame. |
| ADS 末位及校验 | 同一板、同一配置比较短接差分输入下的标准 SPI、55 位 SPI+1 位 GPIO、连续/脉冲和重复读取。保存 status、原始字节、独立计算 CRC 与样本代数。 / Run the capability-advertised ADS timing diagnostic and read cached pages by request ID. Separate physical last-bit sampling from reconstructed bytes and repeated latch reads. Use a scope for electrical timing. |
| Rail 与 battery 四组合 | 外部合规双轨下先测 battery 开/关；仅在硬件确有隔离时才测内部 rail 开的两组，否则记 NOT_RUN。每组先 120 s，再覆盖实际 engine 故障边界。 / Cross rail refresh and battery enable independently. Preserve raw ADC when available, frames, wall time, precision/health intervals, restoration and first valid sample. Distinguish instability from a missing rail reference. |
| 电池转换与滞后 | 检查 AIN8、AINCOM 和实测双轨，记录三个原始样本、时间、spread、restore、lastGood 与重试。 / Validate divider conversion against simultaneous readings at VBAT, AIN8 and AINCOM. Use a controlled passive setup. A good restore and unavailable battery have separate outcomes. |
| 硬件限值 | 对 HW01–HW06 核对实装 BOM、EN、负轨启动、跨域反灌、R54、USB 眼图及 VOLT 对负轨加载。 / Scope rail/EN and reference transients, and measure VOLT with known source impedance. Record bandwidth, probe reference, temperature and error budget. [硬件测点 / Limits and test points](hardware/reliability_revision_notes.md) |

## 3. 被动夹具 / Passive reference fixture

S1D1、S8D8 各 10 kΩ；S4D4、S5D5 各 33 pF；其余 60 点悬空。ROWS=8 才覆盖四个参考件。RES 要求 fresh/acquired 和明确 OPEN（X0D），CAP 输出总 LC 电容。使用同点拆除电容的空载记录验证差分；若按现场条件借用其它悬空通道，用同行/同列/全悬空三种估算并标明代理基线。两个同标称件无法独立拟合增益和偏移。 / Use RES8, CAP8 and RCCCCCCR to exercise both standards and all open inputs. Compare both resistor paths separately, or fit one and report the held-out other. A floating-channel estimate is a practical background proxy. Without same-point unloaded data, traceable standards and acceptance limits, absolute capacitance accuracy remains unverified. VOLT has no voltage standard on this fixture.

```bash
python3 tools/reference_fixture.py --input "$CAPTURE" --provenance recorded --output "$NEW_REPORT"
python3 tools/estimate_floating_cap_background.py --input "$CAPTURE" --output "$NEW_PROXY_REPORT"
python3 tools/hil/analyse_closed_fixture_window.py --help
```

精度验收时在 reference_fixture 中显式给 `--baseline-input` 同点文件，容差来自所用标准件和实验预算。不能以其它端口代替这一参数。报告读数、离散度、缺失原因和覆盖区间。 / Supply a genuine same-point unloaded recording to the strict baseline option. Keep proxy estimates separate from strict results and retain invalid reason counts.

## 4. 32 种配置与稳定性 / 32 configurations, stability and recovery

使用两路 CDC 和 BLE 的独立观察器。先由实际只读查询取得完整 ELF、session、boot ID，再确定 USB serial 和 BLE 地址。以下为 Bash，变量来自当前板，输出目录须是新运行目录。工具支持 `--print-plan` 只检查计划。 / Obtain current identity from the target, use unique output paths, and close both observers before qualifying each window. Supply measured signed rails together with their provenance if the board has no verified external rail configuration.

```bash
identity=(--expected-elf "$ELF" --expected-session "$SESSION" --expected-boot-id "$BOOT"
          --usb-serial "$USB_SERIAL" --ble-address "$BLE_ADDRESS")
python3 tools/hil/run_cdc_matrix_failfast.py "${identity[@]}" --fixture-margin \
  --label matrix --output-directory "$RUN/matrix" --stop-file "$RUN/STOP"
python3 tools/hil/run_final_observed_case.py final600 "${identity[@]}" --label stable --output-directory "$RUN/stable"
python3 tools/hil/run_final_observed_case.py final50 "${identity[@]}" --label switches --output-directory "$RUN/switches"
python3 tools/hil/run_final_observed_case.py short_v2 "${identity[@]}" --label short --output-directory "$RUN/short"
```

| 工况 / Case | 窗口、动作与验收 / Window, stimulus and gates |
|---|---|
| CAP/VOLT/RES/MIXED × 1–8 | 每个配置 120 s，夹具余量模式 CAP8/RES8 为 122 s；MIXED 配置 CVRCVRCV，一活动行只采 C。起止独立检查板上时间及两主机观察器完整帧覆盖。 / Use 32 independent windows. Compare capture, emitted and receiver FPS, gaps, masks/OPEN, ADC checked reads, owner CPU, heap, stack and confirmed return to CAP8. |
| VOLT1 / 600 s | 核对完整连续窗口、无非预期 boot/故障、USB/BLE 完整帧和诊断分母。 / Run ten uninterrupted minutes. Split cell/ADC errors from radio gaps and observer failures. |
| 50 次切换 / 50 transitions | 检查 MODE/ROWS/ROWMODES 接受、相应终态、CONFIG 和两端首个完整目标帧；覆盖重复 CAP、ROWS-only、MIXED 与 CAP 恢复。 / Check terminal and first valid data independently. Exercise missing, late and stale terminals by host replay as well. |
| SHORT / protected full | 以原始 B20 与完整 C/D/K 分开计数，独立验证配置边界首帧与周期帧、最大间隔和拥塞后的恢复。 / Endpoint full records satisfy the snapshot obligation. Use actual monotonic receipt time for maximum spacing. |
| 丢包与弱信号 / Link budgets | 正常 FULL 预算以 missing/(received+missing) 计算，门槛 2.5%。弱链路分开测量 RSSI/条件、10%–20% 缺口及降普通帧率后的关键回复和周期全量。 / A normal-link run cannot qualify an attenuated link. Reassemble and CRC-check before counting; compare independent receiver endpoints within the same session. |

## 5. USB、蓝牙、Wi-Fi 与恢复 / Interface-specific methods

- **USB**：短停读用 `--start-case volt2 --end-case volt2 --pause-formal-reader` 注入 650 ms；长停读用 `capture_usb_unread.py`，BLE 独立观察且不发命令。分别记录设备准入、过期、驱动接受、两端科学 CRC 及恢复首个完整帧。用 VID/PID/interface/serial 确认调试/正式/ROM 枚举与重连。 / Inject a short reader stall, then a separate no-reader interval. Verify whole records, bounded memory and independent BLE progress. Observe ROM and application re-enumeration at the OS.
- **BLE**：测试 FF11/FF20/FF30 全部八种订阅组合、FAST/SAFE、两种 CCCD 发送语义、实际 MTU 分片、30 次重连和 100 次订阅切换。nRF Connect 可人工写 FF10、检查 ACK、关闭/恢复 FF30。 / Exercise subscription gating and control under load, including notify-only fallback, indication, fragment sizes and session changes. Count complete messages after reassembly.
- **Wi-Fi**：在同镜像/boot 上依次 AP/AP/OFF/AP/OFF，逐次匹配接受、WTERM、WIFIREQ、UDP 实收及 BLE/USB 共存。同步采集 MEM、MEMFAIL、PANIC、各 owner 的 CPU 与栈；另测 SDK 长阻塞和内存不足的隔离。 / Distinguish output gating, actual SDK return, physical radio and peer datagram receipt. Report unobserved stages independently.
- **早期 Serial/JTAG**：`validate_transports.py --port PORT --ble --serial-duration 120 --ble-duration 60` 重现先串口后 BLE 的契约；历史长跑为 serial 1200 s、BLE 600 s。另用 `sensorarray_hil.py serial --profile smoke --port PORT --output-directory DIR`。 / Use that staged runner with a matching serial firmware and a reset-observer sidecar. The formal dual-CDC method above uses simultaneous independent endpoints.
- **恢复与持久化 / Recovery and persistence**：匹配原始配置和授权后测试 RECOVER、RESTART、USBBOOT，记录原会话意图、实际枚举和新会话 READY；断电后的电源恢复另测。校准 save/load 的验证需独立授权和已备份设置。 / Separate query/terminal evidence from an actual boot, retained state and a verified restored configuration. Preserve the original calibration.

## 6. 保存与判读 / Evidence and replay

每次保存源码 HEAD、dirty diff/hash、IDF、ELF、板卡 ID、BUILD/PROTO/CONFIG、时钟和运行参数。采集文件使用新的输出来保留原始字节，不补齐截断帧。host 收到的完整帧以第一/末帧和设备时间跨度验证覆盖。实测、真实 encoder 合成、故障注入及未运行项须标明。提交回归只保留最小输入和来源清单，本地采集输出使用被忽略的 `validation_artifacts/`。 / Record raw bytes, clocks, image identity, exact command, return status, coverage and gaps. Archive independent lab evidence before removing temporary output. Label FAIL, UNKNOWN and NOT_RUN explicitly. Keep reproducible goldens, captured slices and provenance in version control.

[主机测试和固定夹具 / Host tests and fixtures](../tests/README.md) · [运行工具 / Tool reference](../tools/README.md) · [正式验收入口 / Acceptance runners](../tools/hil/README.md)

## 7. 早期实验检索 / Earlier experimental procedures

以下为针对特定固件或设备的一次性实验。使用这些历史步骤时，先核对当时的接线、协议和构建身份。固定链接指向原脚本，避免把当时的板号、串口或烧录参数用于另一块板；当前固件的可移植入口在上文。 / These experiments depended on the hardware, packet formats and configuration present at the time. The pinned source describes the exact method and original inputs. Use the maintained collectors for a new board.

| 方法族 / Method family | 原始入口与观察量 / Original procedure and observable |
|---|---|
| FDC 时钟、驱动强度与 DRDY / FDC sweeps | 对照 S5D5 单点、RCOUNT、DEGLITCH/drive 的锁定、幅度、等待时间；由单行扩展为双 worker 和 8 行。 / Reproduce single-cell and row-epoch sweeps with the corresponding firmware revision. [逐次提交 / Commit ledger](history/README.md) |
| UART 接收与电池 / Serial and battery | smoke、持续采集、断开/恢复电池、工频/浮动、只读诊断、重连与 NVS 持久化。 / Run serial/HIL profiles, capture bytes separately from decoded values, and correlate the recorded battery evidence with restore results. [早期方法 / Original profiles](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/validation.md) |
| 旧镜像基准与顺序 / Benchmark baselines | 区分单通路、BLE 旁路观察、失败重试与全矩阵独立窗口。 / Keep each retry as a distinct run with its own identity and duration. [全矩阵基准 / Original matrix baseline](https://github.com/ErrorChen/SensorArray/tree/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/artifacts/full_benchmark_20261113) |
| 仪器背景、RC 建立与寄存器 / Analog and rail | 两条对地电源轨、样本时间、有效来源、首个恢复帧；保留热切换与四组合的不可执行条件。 / Match physical rail measurements, timing, recovery and fixture contact to the recorded window. [夹具对照 / Fixture campaign](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/reference_fixture_recheck_20261113.md) |
| USB vendor bulk、CDC 与下载恢复 / USB variants | 分别测试 vendor/CDC 的端点选择、OS 访问权、原应用备份、ROM 交接、下载后恢复应用、重新枚举和无读者过期。 / Check actual OS identity and transport role at every handoff. Keep ROM recovery tests separate from firmware frame delivery. [接口实验 / Interface experiments](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/usb_otg_ble_priority_20261114.md) |
| I²C 独立于 SPI 的启动验证 / Bus qualification | 分别报告两个 FDC 的 I²C 候选和 ADS 的 SPI 候选、实速和错误分母，并观察失电后重新出现。 / Measure actual divided clock and reject rates per frontend. [启动和恢复 / Startup qualification](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/frontend_startup_recovery_20261113.md) |
| 性能归因 / Performance attribution | 对照旧/新 metadata parser、ROM CRC、优化级别、MTU/分片规划、普通 LOG/保护队列；询问 CPUOWNER、heap、栈和独立观察器。 / Change one candidate at a time, use the same fixture, preserve failed comparisons, and report queue and host counters over their own observation windows. [性能实验 / Load methods](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/load_reliability_fix_20261113.md) |
| 压力、优先级与反例 / Negative controls | 回放过期帧、旧 DONE、短预览替代全帧、SDK 未返回、两帧假覆盖、采集器断开与查询间 revision 变化。 / Test event order, frame coverage, query consistency and actual observer closure before interpreting any threshold. [CDC/BLE 验收实验 / Acceptance methods](https://github.com/ErrorChen/SensorArray/blob/dc5a5adb4bd62677221d0a2ceb0acf15886165c7/docs/audit/cdc_recovery_acceptance_20261116.md) |

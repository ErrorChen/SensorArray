# USB / BLE 采集与封存验收 / USB and BLE capture and closed-case validation

正式 CDC 读写在同一接口完成，另一进程只读调试 CDC，BLE 进程独立订阅 C、D、L。协调器先确认观察器就绪，再打开正式窗口，等全部子进程有界退出后才核对原始证据。为每次尝试使用新标签。 / Formal CDC carries data and commands. One process reads debug CDC and another subscribes to BLE C, D and L. The coordinator waits for observers before the formal window and checks original receipts after bounded process closure. Use a fresh label per attempt. [通用工具 / Host tools](../README.md) · [测试方法 / Test methods](../../docs/validation.md) · [项目 / Project](../../README.md)

## 生成计划与执行 / Planning and execution

这些标识仅用于演示 `--print-plan`。实测前从当前板的 BUILD? 和 CONFIG? 获取完整 ELF SHA-256、session、bootId，确认 USB 序列号及 BLE 地址，再去掉该选项。 / These example identifiers illustrate `--print-plan`. Before a real capture, substitute the board’s current BUILD? ELF SHA-256, CONFIG? session/bootId, USB serial and BLE address, then omit the flag.

~~~bash
common=(--usb-serial 001122334455 --ble-address 00:11:22:33:44:55
        --expected-elf 0000000000000000000000000000000000000000000000000000000000000000
        --expected-session 0000000000000000 --expected-boot-id 1)
python3 tools/hil/run_cdc_matrix_failfast.py "${common[@]}" --label matrix \
  --output-directory validation_artifacts/matrix --stop-file validation_artifacts/matrix.stop \
  --fixture-margin --print-plan
for case in final600 final50 short_v2
do
  python3 tools/hil/run_final_observed_case.py "$case" "${common[@]}" \
    --label "$case" --output-directory validation_artifacts --print-plan
done
~~~

| 工况 / Case | 采集与证据 / Acquisition and evidence |
| --- | --- |
| 32 工况矩阵 / Matrix | CAP、VOLT、RES、MIXED 各 1–8 行，每项 120 秒，`--fixture-margin` 将 CAP8/RES8 延长至 122 秒。混合 profile 为 CVRCVRCV。 / All four modes, 1–8 active rows, 120 s each. Fixture margin requests 122 s for CAP8/RES8. MIXED1 uses one active row of the CVRCVRCV profile. |
| `final600` | 1 行 VOLT 连续 600 秒，分别检查 USB 和 BLE 完整窗口。 / VOLT1 for 600 continuous seconds with independent USB and BLE window checks. |
| `final50` | 50 次有界切换，逐次核对配置终态、应用边界之后的完整帧以及 CAP8 恢复。 / 50 bounded transitions with terminals, post-boundary full frames and verified CAP8 recovery. |
| `short_v2` | CAP8 采集 120 秒，临时设置 BLE SHORT，核验预览期间的周期完整快照及 TX 恢复。 / CAP8 for 120 s under temporary BLE SHORT, verifying periodic full snapshots and TX restoration. |

临时 rail 校准要求三个参数成组传入。`--rail-avdd-uv` 为正值，`--rail-avss-uv` 为负值，`--rail-source-note` 记录当前板测点及来源。省略时采集器检查已有外部校准。输出目录中的 raw、events、BLE journal、summary 和 gate 用于回放。 / Supply positive AVDD, negative AVSS and the current board’s measurement provenance together for volatile calibration. Omitting them checks the existing external calibration. Replay uses raw bytes, events, BLE journal, summaries and gate under the new output directory. [BLE 判据 / BLE criteria](ble_acceptance_v2.json)

Linux 的 `/proc/<pid>/stat` 提供进程起始时间以检查观察器身份，sysfs 提供两路 CDC 角色。 / Linux process start times identify observers, and sysfs identifies the two CDC roles.

### 选择工况及判定输入 / Case and qualification options

| 选项 / Option | 作用 / Meaning |
| --- | --- |
| `--expected-elf`、`--expected-session`、`--expected-boot-id` | 填入当前板 64 个十六进制字符的 ELF SHA-256、16 个十六进制字符的 session 和十进制 boot ID，供前后查询及离线重放进行匹配。 / Bind preparation/cleanup queries and offline replay to the current board's 64-hex ELF SHA-256, 16-hex session and decimal boot ID. |
| `--usb-serial`、`--ble-address`、`--ble-adapter` | USB 使用无冒号的 12 位十六进制序列号；BLE 使用六组冒号分隔的十六进制字节，Linux 适配器缺省 hci0。 / USB expects 12 hex digits with no colons; BLE uses six colon-separated octets. The Linux adapter defaults to hci0. |
| `--output-directory`、`--label`、`--stop-file` | 每次使用新目录或标签。矩阵运行中可创建 stop-file 请求收尾，正式工况的计划也给出 operator_stop。 / Use a fresh case label. Creating the matrix stop file requests bounded cleanup; per-case plans also report an operator_stop path. |
| `--start-case`、`--end-case`、`--fixture-margin` | 按 cap1 至 mixed8 的顺序选首尾（包含两端）。margin 仅延长 CAP8、RES8 到 122 秒。 / Select an inclusive case range from cap1 to mixed8. Margin changes only CAP8 and RES8 to 122 seconds. |
| `--pause-formal-reader` | 单独运行 VOLT2 时插入一次 650 ms 应用层读暂停。 / Injects one 650 ms application read pause when running VOLT2 alone. |
| `--print-plan` | 生成无硬件的计划供核对；未开始任何正式窗口。 / Prints the plan without opening hardware or starting a formal capture window. |
| `--summary`、`--ble-summary`、`--debug-summary`、`--output` | 单工况离线判定读入这三个已关闭的摘要，按记录路径回查原始证据，写入新 JSON。 / The offline per-case gate takes the three closed summaries, resolves their original evidence and writes a fresh JSON verdict. |
| `--result-id`、`--probe` | ADS 时序工具可读取已有探测结果；按 ID 回读同时要求 expected-session。probe 在 TIMING、TAIL 中选择。 / The ADS timing tool can retrieve an existing result; a result ID also requires the expected session. Choose TIMING or TAIL for the probe. |

## 模块索引 / Module index

[analyse_closed_fixture_window](#analyse_closed_fixture_window) · [analyze_ble_protection](#analyze_ble_protection) · [analyze_bulk_stage](#analyze_bulk_stage) · [analyze_cpu_owner](#analyze_cpu_owner) · [analyze_usb_unread](#analyze_usb_unread) · [archive_cdc_debug](#archive_cdc_debug) · [capture_ads_timing](#capture_ads_timing) · [capture_bulk_benchmark](#capture_bulk_benchmark) · [capture_cdc_benchmark](#capture_cdc_benchmark) · [capture_cdc_short_pause](#capture_cdc_short_pause) · [capture_load_benchmark](#capture_load_benchmark) · [capture_mode_switches](#capture_mode_switches) · [capture_short_snapshots](#capture_short_snapshots) · [capture_usb_unread](#capture_usb_unread) · [check_software_recovery](#check_software_recovery) · [check_wifi_owner_control](#check_wifi_owner_control) · [closed_matrix_performance](#closed_matrix_performance) · [fixture_rail](#fixture_rail) · [observe_bulk_boot](#observe_bulk_boot) · [observe_cdc_debug](#observe_cdc_debug) · [qualify_closed_capture](#qualify_closed_capture) · [run_cdc_matrix_failfast](#run_cdc_matrix_failfast) · [run_final_observed_case](#run_final_observed_case) · [summarize_closed_matrix](#summarize_closed_matrix) · [summarize_diagnostic_screen](#summarize_diagnostic_screen)

下表沿用源码签名。数据类列出字段、类型及缺省值，实例方法通过对象调用，`self` 由 Python 绑定。标为“内部”的函数只在所属函数内使用。 / Signatures follow the source. Dataclass constructors list field types and defaults. Call instance methods through their object, which binds `self`. Entries marked *local* are used inside their enclosing function.

<a id="analyse_closed_fixture_window"></a>

### analyse_closed_fixture_window

仅从已关闭、哈希核验的字节与接收时间重建捕获窗口，分别输出传输、夹具和悬空背景评估。 / Reconstructs a capture window from closed, hash-checked bytes and receipt times, then writes separate transport, fixture and floating-background results. [源码 / Source](analyse_closed_fixture_window.py)

命令 / CLI　`python3 tools/hil/analyse_closed_fixture_window.py --help`。参数 / Arguments　`--directory`、`--label`、`--output-directory`、`--output-prefix`。

| 函数、类型与形参 / Callable, type and parameters | 中文用途 / English behavior |
| --- | --- |
| <code>sha(data)</code> | 计算文件或字节的摘要，为结果绑定实际输入。 / Computes a digest for file or byte evidence so results can be traced to the actual input. |
| <code>encoded(value)</code> | 按稳定格式序列化 JSON 为 UTF-8 字节。 / Serializes JSON to reproducible UTF-8 bytes. |
| <code>existing(directory, label, suffix)</code> | 根据原始位置或同目录的无损压缩文件解析实际证据路径。 / Resolves the actual evidence path, including a colocated losslessly compressed file when supported. |
| <code>read_source(path)</code> | 读取指定输入并按格式保留原始字节、行或哈希。 / Reads the specified input, preserving its original bytes, lines or digest as required by the format. |
| <code>select_frames(raw, events, window)</code> | 按主机收到帧头与帧尾的时间从同一窗口选择整帧。 / Selects whole frames whose header and trailer receipt times fall within one window. |
| <code>select_frames.receipt(offset)</code>（内部 / local） | 定位原始数据中指定位置的主机接收时间。 / Locates the host receipt time for a position in the original stream. |
| <code>metrics_report(capture, fixture_report, proxy, expected_config)</code> | 将经过核验的字段和各层指标组成报告。 / Assembles a report from verified fields and layer-specific metrics. |
| <code>make_markdown(label, source, window, transport, metrics, paths)</code> | 将已分析的工况、夹具与来源写为 Markdown 报告。 / Renders the analyzed case, fixture and source evidence as Markdown. |
| <code>make_markdown.num(value)</code>（内部 / local） | 把数值或缺失情况格式化为报告文本。 / Formats a numeric reading or missing value as report text. |
| <code>main()</code> | 解析命令行，执行本模块的操作并按结果退出。 / Parses CLI arguments, runs the module operation and reports the resulting exit status. |

<a id="analyze_ble_protection"></a>

### analyze_ble_protection

用 CONFIG、BLEFLOW、BLESNAP 和原始请求区间验证 BLE 关键输出、周期快照和降频计数。 / Checks protected BLE output, periodic snapshots and decimation accounting against configuration, diagnostic queries and raw reply intervals. [源码 / Source](analyze_ble_protection.py)

命令 / CLI　`python3 tools/hil/analyze_ble_protection.py --help`。参数 / Arguments　`--summary`、`--output`。

| 函数、类型与形参 / Callable, type and parameters | 中文用途 / English behavior |
| --- | --- |
| <code>parse_line(line: str, tag: str) → dict[str, str]</code> | 解析逗号分隔的标签与键值字段，保留 wire 表示。 / Parses the tag and comma-delimited key/value fields while preserving their wire representation. |
| <code>monotonic(value: object) → float</code> | 校验主机单调时钟字段有效且有限。 / Validates that a host monotonic timestamp is finite and usable. |
| <code>bounded_delta(before: str, after: str, bits: int, maximum: int) → dict</code> | 在规定的计数域和边界上计算增量，处理可识别的回绕和未知情况。 / Computes a counter delta under the specified domain and bounds, accounting for supported wraps and unknowns. |
| <code>config_brackets(summary: dict, raw: dict, replies: set) → dict</code> | 核对 CONFIG 前后界、查询原始行和采样 epoch，提取端点计数。 / Checks CONFIG brackets, query source lines and sampling epochs to obtain endpoint counters. |
| <code>query_record(summary: dict, phase: str, command: str, raw: dict, replies: set) → dict</code> | 核对 CONFIG 前后界、查询原始行和采样 epoch，提取端点计数。 / Checks CONFIG brackets, query source lines and sampling epochs to obtain endpoint counters. |
| <code>sampled_epoch(queries: dict, summary: dict) → dict</code> | 核对 CONFIG 前后界、查询原始行和采样 epoch，提取端点计数。 / Checks CONFIG brackets, query source lines and sampling epochs to obtain endpoint counters. |
| <code>endpoint_view(command: str, record: dict) → dict</code> | 核对 CONFIG 前后界、查询原始行和采样 epoch，提取端点计数。 / Checks CONFIG brackets, query source lines and sampling epochs to obtain endpoint counters. |
| <code>analyze_document(summary: dict, raw: dict, replies: set) → dict</code> | 从提供的记录／文件核对本模块要求的来源和时间边界并生成分析结果。 / Analyzes the supplied record/file with the source and time-boundary checks required by this module. |
| <code>evidence_path(text: str, summary_path: Path) → Path</code> | 根据原始位置或同目录的无损压缩文件解析实际证据路径。 / Resolves the actual evidence path, including a colocated losslessly compressed file when supported. |
| <code>file_sha(path: Path) → str</code> | 计算文件或字节的摘要，为结果绑定实际输入。 / Computes a digest for file or byte evidence so results can be traced to the actual input. |
| <code>read_raw(path: Path, expected_sha: str) → tuple[set, dict]</code> | 读取指定输入并按格式保留原始字节、行或哈希。 / Reads the specified input, preserving its original bytes, lines or digest as required by the format. |
| <code>integer(value: object) → int</code> | 读取并检查数值字段，保留该调用的范围及未知值规则。 / Reads a numeric field subject to the caller's range and unknown-value rules. |
| <code>raw_reply_between(info: dict, line: str, begin: int, end: int) → bool</code> | 确认匹配回复位于本次发送与记录结束之间的原始字节内。 / Verifies the reply occurs in original bytes between this query write and its closing observation. |
| <code>validate_legacy_short_closure(summary: dict, events: list, raw: dict, inputs: dict) → dict</code> | 按固定的 v1 采集器摘要和原始事件验证 SHORT 恢复及最终故障观察。 / Uses the frozen v1 collector digest and original events to verify SHORT restore and final fault observations. |
| <code>analyze_file(summary_path: Path) → dict</code> | 从提供的记录／文件核对本模块要求的来源和时间边界并生成分析结果。 / Analyzes the supplied record/file with the source and time-boundary checks required by this module. |
| <code>main(argv=None) → int</code> | 解析命令行，执行本模块的操作并按结果退出。 / Parses CLI arguments, runs the module operation and reports the resulting exit status. |

<a id="analyze_bulk_stage"></a>

### analyze_bulk_stage

向共享基准分析器传递实际参数，按正式 USB 与调试流输出结果。 / Delegates CLI arguments to the shared benchmark analyzer for formal USB and diagnostic streams. [源码 / Source](analyze_bulk_stage.py)

命令 / CLI　`python3 tools/hil/analyze_bulk_stage.py --help`。参数 / Arguments　`--directory`、`--output`、`--ble-journal`、`--debug-summary`、`--selection-output`、`--replace`。


<a id="analyze_cpu_owner"></a>

### analyze_cpu_owner

核对 CPUOWNER 各角色的 epoch、有效性、时钟域和采样区间，按同一身份输出 CPU 归属。 / Validates CPUOWNER roles, epochs, clocks and sampling intervals before reporting CPU attribution by identity. [源码 / Source](analyze_cpu_owner.py)

命令 / CLI　`python3 tools/hil/analyze_cpu_owner.py --help`。参数 / Arguments　`--summary`、`--output`。

| 函数、类型与形参 / Callable, type and parameters | 中文用途 / English behavior |
| --- | --- |
| <code>sha(path: Path) → str</code> | 计算文件或字节的摘要，为结果绑定实际输入。 / Computes a digest for file or byte evidence so results can be traced to the actual input. |
| <code>uint(value: object, bits: int=64) → int</code> | 读取并检查数值字段，保留该调用的范围及未知值规则。 / Reads a numeric field subject to the caller's range and unknown-value rules. |
| <code>parse_line(line: str, tag: str=&#x27;CPUOWNER&#x27;) → dict[str, str]</code> | 解析逗号分隔的标签与键值字段，保留 wire 表示。 / Parses the tag and comma-delimited key/value fields while preserving their wire representation. |
| <code>validate(fields: dict, role: int, session: str) → dict</code> | 校验角色字段或重放查询轨迹，并识别捕获中的故障及重启证据。 / Validates role fields or replays query traces, detecting fault and reset evidence in the capture. |
| <code>config_identity(snapshot: dict) → tuple[str, ...]</code> | 提取或比较同一 boot 内的配置身份字段。 / Extracts or compares configuration identity fields within one boot. |
| <code>analyze_document(summary: dict, raw_lines: set[str], event_replies: set[tuple]) → dict</code> | 从提供的记录／文件核对本模块要求的来源和时间边界并生成分析结果。 / Analyzes the supplied record/file with the source and time-boundary checks required by this module. |
| <code>evidence_path(text: str, summary: Path) → Path</code> | 根据原始位置或同目录的无损压缩文件解析实际证据路径。 / Resolves the actual evidence path, including a colocated losslessly compressed file when supported. |
| <code>analyze_file(summary_path: Path) → dict</code> | 从提供的记录／文件核对本模块要求的来源和时间边界并生成分析结果。 / Analyzes the supplied record/file with the source and time-boundary checks required by this module. |
| <code>main() → int</code> | 解析命令行，执行本模块的操作并按结果退出。 / Parses CLI arguments, runs the module operation and reports the resulting exit status. |

<a id="analyze_usb_unread"></a>

### analyze_usb_unread

核验主机停止读取期间的事件、BLE 数据和恢复后的只读快照，对无观测计数保留 unknown。 / Verifies unread-host intervals, BLE data and post-recovery queries, preserving unknown counters where evidence is absent. [源码 / Source](analyze_usb_unread.py)

命令 / CLI　`python3 tools/hil/analyze_usb_unread.py --help`。参数 / Arguments　`--command-record`。

| 函数、类型与形参 / Callable, type and parameters | 中文用途 / English behavior |
| --- | --- |
| <code>sha(path)</code> | 计算文件或字节的摘要，为结果绑定实际输入。 / Computes a digest for file or byte evidence so results can be traced to the actual input. |
| <code>reference(path)</code> | 计算文件或字节的摘要，为结果绑定实际输入。 / Computes a digest for file or byte evidence so results can be traced to the actual input. |
| <code>load(path)</code> | 独占写入新记录或读取已有 JSON 证据。 / Creates a new exclusive record or reads existing JSON evidence. |
| <code>resolve(path, summary)</code> | 根据原始位置或同目录的无损压缩文件解析实际证据路径。 / Resolves the actual evidence path, including a colocated losslessly compressed file when supported. |
| <code>read_bytes(path)</code> | 读取指定输入并按格式保留原始字节、行或哈希。 / Reads the specified input, preserving its original bytes, lines or digest as required by the format. |
| <code>finite(value)</code> | 检查有限数值，避免将缺失或非有限速率用于统计。 / Checks finite numbers so missing or non-finite rates stay out of statistics. |
| <code>verify_query_trace(events, raw)</code> | 校验角色字段或重放查询轨迹，并识别捕获中的故障及重启证据。 / Validates role fields or replays query traces, detecting fault and reset evidence in the capture. |
| <code>reject_fault_reset_text(raw)</code> | 校验角色字段或重放查询轨迹，并识别捕获中的故障及重启证据。 / Validates role fields or replays query traces, detecting fault and reset evidence in the capture. |
| <code>validate_hold(summary, events, raw)</code> | 核对无读窗口或随后只读探测的原始证据，或分析窗口内的 BLE 完整帧。 / Verifies original evidence for an unread window or later read-only probe, or analyzes complete BLE frames in that window. |
| <code>validate_post_probe(post, events, raw, hold_end, config, build)</code> | 核对无读窗口或随后只读探测的原始证据，或分析窗口内的 BLE 完整帧。 / Verifies original evidence for an unread window or later read-only probe, or analyzes complete BLE frames in that window. |
| <code>NoUsbComparison</code> | 标记 USB 主机无读窗口内跨通路对比不可观测。 / Marks cross-transport comparison unavailable during a host-unread USB window. |
| <code>NoUsbComparison.compare(self, *_args)</code> | 将接收帧与另一通路或要求的身份作逐帧比对。 / Compares a received frame against the other transport or expected identity. |
| <code>NoUsbComparison.result(self, _label)</code> | 返回适合序列化的字段、样本统计或比对结果。 / Returns serializable fields, sample statistics or comparison results. |
| <code>analyse_ble_window(journal, start, end, config, label)</code> | 核对无读窗口或随后只读探测的原始证据，或分析窗口内的 BLE 完整帧。 / Verifies original evidence for an unread window or later read-only probe, or analyzes complete BLE frames in that window. |
| <code>counter_delta(before, after)</code> | 在规定的计数域和边界上计算增量，处理可识别的回绕和未知情况。 / Computes a counter delta under the specified domain and bounds, accounting for supported wraps and unknowns. |
| <code>query_windows(summary)</code> | 按同一身份的起止观测计算分层计数窗口。 / Derives layer-counter windows from bracketing observations with the same identity. |
| <code>analyze(summary_path, ble_path, debug_path, post_path, identity_path, command_records=())</code> | 消费输入证据并返回本模块定义的逐项分析结果。 / Consumes input evidence and returns the module's per-item analysis. |
| <code>main(argv=None)</code> | 解析命令行，执行本模块的操作并按结果退出。 / Parses CLI arguments, runs the module operation and reports the resulting exit status. |

<a id="archive_cdc_debug"></a>

### archive_cdc_debug

将已关闭的 CDC 调试流压缩并回读比对，保留原文件和校验清单。 / Compresses closed CDC debug streams, verifies decompressed bytes and retains originals plus a digest manifest. [源码 / Source](archive_cdc_debug.py)

命令 / CLI　`python3 tools/hil/archive_cdc_debug.py --help`。参数 / Arguments　`--debug-summary`、`--output`。

| 函数、类型与形参 / Callable, type and parameters | 中文用途 / English behavior |
| --- | --- |
| <code>archive(summary_path, output)</code> | 压缩已关闭的调试文件，逐字节验证解压结果并保存摘要。 / Compresses closed debug files, verifies decompressed bytes and saves their digests. |
| <code>main(argv=None)</code> | 解析命令行，执行本模块的操作并按结果退出。 / Parses CLI arguments, runs the module operation and reports the resulting exit status. |

<a id="capture_ads_timing"></a>

### capture_ads_timing

抓取有界 ADS 时序检查，核对配置、分组、寄存器、SPI 原字节、读尾及 CRC。 / Captures bounded ADS timing diagnostics, checking configuration, groups, registers, raw SPI bytes, transfer tails and CRCs. [源码 / Source](capture_ads_timing.py)

命令 / CLI　`python3 tools/hil/capture_ads_timing.py --help`。参数 / Arguments　`--output-directory`、`--label`、`--usb-serial`、`--expected-elf`、`--probe`、`--result-id`、`--expected-session`。

| 函数、类型与形参 / Callable, type and parameters | 中文用途 / English behavior |
| --- | --- |
| <code>require(fields, keys, context)</code> | 条件失败时以具体原因中止当前证据检查。 / Stops the current evidence check with its specific reason when the condition fails. |
| <code>unsigned(fields, key, bits=32)</code> | 读取并检查数值字段，保留该调用的范围及未知值规则。 / Reads a numeric field subject to the caller's range and unknown-value rules. |
| <code>flag(fields, key)</code> | 解析标志、驱动错误码或核对诊断记录的类型。 / Decodes a flag or driver error code, or checks the diagnostic record kind. |
| <code>error_code(fields, key)</code> | 解析标志、驱动错误码或核对诊断记录的类型。 / Decodes a flag or driver error code, or checks the diagnostic record kind. |
| <code>kind_matches(fields, expected, key=&#x27;probe&#x27;)</code> | 解析标志、驱动错误码或核对诊断记录的类型。 / Decodes a flag or driver error code, or checks the diagnostic record kind. |
| <code>validate_state(state, config, probe_kind, ident=None, allow_failed_revision=False)</code> | 核验 ADS 探测之前条件以及状态、分组、寄存器或记录字段。 / Validates ADS probe prerequisites and the state, groups, registers or record fields. |
| <code>validate_preconditions(pre, probe_kind)</code> | 核验 ADS 探测之前条件以及状态、分组、寄存器或记录字段。 / Validates ADS probe prerequisites and the state, groups, registers or record fields. |
| <code>validate_summary(probe, spi)</code> | 核验 ADS 探测之前条件以及状态、分组、寄存器或记录字段。 / Validates ADS probe prerequisites and the state, groups, registers or record fields. |
| <code>validate_group(stats, timing, group, probe_kind)</code> | 核验 ADS 探测之前条件以及状态、分组、寄存器或记录字段。 / Validates ADS probe prerequisites and the state, groups, registers or record fields. |
| <code>validate_registers(fields, index)</code> | 核验 ADS 探测之前条件以及状态、分组、寄存器或记录字段。 / Validates ADS probe prerequisites and the state, groups, registers or record fields. |
| <code>validate_record(fields, expected_group, stats, timing, probe_kind, seen)</code> | 核验 ADS 探测之前条件以及状态、分组、寄存器或记录字段。 / Validates ADS probe prerequisites and the state, groups, registers or record fields. |
| <code>crc_by_division(data)</code> | 逐位作多项式除法，独立重算采样字节的 CRC。 / Computes CRC by bitwise polynomial division over the supplied sample bytes. |
| <code>verify_wire_record(fields)</code> | 核查 SPI 字节、尾部时钟或 GPIO 观测与芯片返回的状态和校验值。 / Checks SPI bytes, transfer tails or GPIO observations against chip status and check bytes. |
| <code>verify_tail_record(fields, tail)</code> | 核查 SPI 字节、尾部时钟或 GPIO 观测与芯片返回的状态和校验值。 / Checks SPI bytes, transfer tails or GPIO observations against chip status and check bytes. |
| <code>same_identity(before, after)</code> | 提取或比较同一 boot 内的配置身份字段。 / Extracts or compares configuration identity fields within one boot. |
| <code>verify_io_record(fields, io)</code> | 核查 SPI 字节、尾部时钟或 GPIO 观测与芯片返回的状态和校验值。 / Checks SPI bytes, transfer tails or GPIO observations against chip status and check bytes. |
| <code>read_cached_result(options, port_factory=UsbCdcPort)</code> | 读取本次探测缓存的有界明细并核对来源。 / Reads bounded records from this probe’s result cache and verifies their identity. |
| <code>read_cached_result.query(command, tag)</code>（内部 / local） | 发送查询并匹配其回复，遵守本采集器的截止时间和重试规则。 / Sends a query and matches its reply under the collector’s deadline and retry rules. |
| <code>read_cached_result.detail(part, tag, index=None)</code>（内部 / local） | 查询并核验本轮 ADS 时序探测的具体结果。 / Queries and checks detailed results of the current ADS timing probe. |
| <code>run(options, port_factory=UsbCdcPort)</code> | 在本对象定义的输入、时限与恢复约束下执行本节所述流程。 / Executes the operation described in this section under the object's input, deadline and recovery constraints. |
| <code>run.assess(scope, function, *args)</code>（内部 / local） | 查询并核验本轮 ADS 时序探测的具体结果。 / Queries and checks detailed results of the current ADS timing probe. |
| <code>run.detail(kind, tag, index=None)</code>（内部 / local） | 查询并核验本轮 ADS 时序探测的具体结果。 / Queries and checks detailed results of the current ADS timing probe. |
| <code>main()</code> | 解析命令行，执行本模块的操作并按结果退出。 / Parses CLI arguments, runs the module operation and reports the resulting exit status. |

<a id="capture_bulk_benchmark"></a>

### capture_bulk_benchmark

在 vendor bulk 传输上复用基准采集器，增加有界 CPUOWNER、调度和链路诊断。 / Extends the vendor bulk benchmark collector with bounded CPU-owner, scheduler and link diagnostics. [源码 / Source](capture_bulk_benchmark.py)

命令 / CLI　`python3 tools/hil/capture_bulk_benchmark.py --help`。参数 / Arguments　`--output-directory`、`--label`、`--mode`、`--rows`、`--profile`、`--suite`、`--stability`、`--capture-seconds`、`--rail-avdd-uv`、`--rail-avss-uv`、`--rail-source-note`、`--expected-elf`、`--expected-session`、`--expected-boot-id`、`--transport`、`--usb-vid`、`--usb-pid`、`--usb-serial`。

| 函数、类型与形参 / Callable, type and parameters | 中文用途 / English behavior |
| --- | --- |
| <code>unsigned(value, command, field, bits=32)</code> | 读取并检查数值字段，保留该调用的范围及未知值规则。 / Reads a numeric field subject to the caller's range and unknown-value rules. |
| <code>validate_frame_output(command, fields)</code> | 核对完整帧输出或 CPUOWNER 的角色、身份、计数和查询证据。 / Checks complete-frame output or CPUOWNER roles, identity, counters and raw query evidence. |
| <code>validate_frame_output.reject(field)</code>（内部 / local） | 将字段校验失败及其原因上报给调用方。 / Reports the failed field check and reason to the caller. |
| <code>validate_cpu_owner(command, fields)</code> | 核对完整帧输出或 CPUOWNER 的角色、身份、计数和查询证据。 / Checks complete-frame output or CPUOWNER roles, identity, counters and raw query evidence. |
| <code>validate_cpu_owner.reject(field)</code>（内部 / local） | 将字段校验失败及其原因上报给调用方。 / Reports the failed field check and reason to the caller. |
| <code>cpu_owner_evidence(fields)</code> | 核对完整帧输出或 CPUOWNER 的角色、身份、计数和查询证据。 / Checks complete-frame output or CPUOWNER roles, identity, counters and raw query evidence. |
| <code>validate_reply(command, fields, *, backend=&#x27;otg_bulk_v1&#x27;)</code> | 核对查询或回复中的必需字段、范围及能力语义。 / Checks required reply fields, ranges and capability semantics. |
| <code>scheduler_health(fields)</code> | 由 SCHED 计数判定进度、deadline 或故障诊断。 / Derives progress, deadline or fault indicators from SCHED counters. |
| <code>BulkBenchmarkCollector</code> | 实现本节的采集阶段、数据／命令观察和有界状态验证。 / Implements the module's capture phases, data/command observation and bounded state checks. |
| <code>BulkBenchmarkCollector.validate_fields(cls, command, fields)</code> | 核对查询或回复中的必需字段、范围及能力语义。 / Checks required reply fields, ranges and capability semantics. |
| <code>BulkBenchmarkCollector.__init__(self, *args, **kwargs)</code> | 使用所列参数构造本对象，初始化其计数、状态或 I/O 句柄。 / Constructs the object from the listed arguments and initializes its counters, state or I/O handles. |
| <code>BulkBenchmarkCollector.event(self, kind, **fields)</code> | 附加带时间和来源的事件或诊断输出。 / Appends timestamped event or diagnostic output with its source. |
| <code>BulkBenchmarkCollector.observe(self, line)</code> | 处理当前观测，保存本对象要求的字段、身份和时间。 / Consumes an observation and records the fields, identity and timing required by this object. |
| <code>BulkBenchmarkCollector.query_cpu_owner(self, command, phase_deadline)</code> | 核对完整帧输出或 CPUOWNER 的角色、身份、计数和查询证据。 / Checks complete-frame output or CPUOWNER roles, identity, counters and raw query evidence. |
| <code>BulkBenchmarkCollector.query(self, command, tag, phase_deadline)</code> | 发送查询并匹配其回复，遵守本采集器的截止时间和重试规则。 / Sends a query and matches its reply under the collector’s deadline and retry rules. |
| <code>BulkBenchmarkCollector.diagnostic_window(self, name, deadline, *, full=False)</code> | 在有界窗口内采集前后状态及诊断查询。 / Captures state and diagnostic queries within a bounded observation window. |
| <code>BulkBenchmarkCollector.diagnostic_window.window_queries()</code>（内部 / local） | 按该工况诊断能力列出应执行的一组查询。 / Builds the set of queries supported for the case's diagnostic window. |
| <code>BulkBenchmarkCollector.snapshot_window(self, name, deadline)</code> | 在有界窗口内采集前后状态及诊断查询。 / Captures state and diagnostic queries within a bounded observation window. |
| <code>BulkBenchmarkCollector.pump(self, deadline)</code> | 在截止时间内读取、解析新数据并更新采集状态。 / Reads and parses new input within the deadline and updates capture state. |
| <code>BulkBenchmarkCollector.run(self)</code> | 在本对象定义的输入、时限与恢复约束下执行本节所述流程。 / Executes the operation described in this section under the object's input, deadline and recovery constraints. |
| <code>bulk_collector_selected(collector_class=BulkBenchmarkCollector)</code> | 在上下文内临时选择采集器实现，离开时恢复原配置。 / Selects a collector implementation for the context and restores the prior selection on exit. |
| <code>bulk_collector_selected.require_formal_bulk(options)</code>（内部 / local） | 核验命令行选择了正式 bulk 接口及相容的选项。 / Requires a formal bulk selection and compatible command-line options. |
| <code>run_benchmark(options, **kwargs)</code> | 打开所选的采集器，在有界阶段内取证并关闭原始文件。 / Opens the selected collector, captures within bounded phases and closes the original evidence files. |
| <code>main(argv=None)</code> | 解析命令行，执行本模块的操作并按结果退出。 / Parses CLI arguments, runs the module operation and reports the resulting exit status. |

<a id="capture_cdc_benchmark"></a>

### capture_cdc_benchmark

为正式 USB CDC 数据接口选用共享基准采集器，校验 CTRL／DATA 共用 CDC 的元数据。 / Selects the formal CDC implementation of the shared collector and checks metadata for DATA and CTRL on the same CDC interface. [源码 / Source](capture_cdc_benchmark.py)

命令 / CLI　`python3 tools/hil/capture_cdc_benchmark.py --help`。参数 / Arguments　`--output-directory`、`--label`、`--mode`、`--rows`、`--profile`、`--suite`、`--stability`、`--capture-seconds`、`--rail-avdd-uv`、`--rail-avss-uv`、`--rail-source-note`、`--expected-elf`、`--expected-session`、`--expected-boot-id`、`--transport`、`--usb-vid`、`--usb-pid`、`--usb-serial`。

| 函数、类型与形参 / Callable, type and parameters | 中文用途 / English behavior |
| --- | --- |
| <code>CdcBenchmarkCollector</code> | 实现本节的采集阶段、数据／命令观察和有界状态验证。 / Implements the module's capture phases, data/command observation and bounded state checks. |
| <code>CdcBenchmarkCollector.validate_fields(cls, command, fields)</code> | 对于 USBSTREAM? 要求 v=CDC、scope=otg_data、dataEvery=1、debug=cdc、control=shared_cdc。其它命令沿用基础采集器的字段校验。 / Checks that USBSTREAM? reports v=CDC, scope=otg_data, dataEvery=1, debug=cdc and control=shared_cdc. Delegates other replies to the base collector. |
| <code>run_benchmark(options, **kwargs)</code> | 打开所选的采集器，在有界阶段内取证并关闭原始文件。 / Opens the selected collector, captures within bounded phases and closes the original evidence files. |
| <code>main(argv=None)</code> | 解析命令行，执行本模块的操作并按结果退出。 / Parses CLI arguments, runs the module operation and reports the resulting exit status. |

<a id="capture_cdc_short_pause"></a>

### capture_cdc_short_pause

在指定 VOLT2 采集窗口的主机读取路径插入一次 650 ms 暂停，记录暂停前后单调时间。 / Injects one 650 ms application-level read pause in a VOLT2 window and records monotonic times on both sides. [源码 / Source](capture_cdc_short_pause.py)

命令 / CLI　`python3 tools/hil/capture_cdc_short_pause.py --help`。参数 / Arguments　`--output-directory`、`--label`、`--mode`、`--rows`、`--profile`、`--suite`、`--stability`、`--capture-seconds`、`--rail-avdd-uv`、`--rail-avss-uv`、`--rail-source-note`、`--expected-elf`、`--expected-session`、`--expected-boot-id`、`--transport`、`--usb-vid`、`--usb-pid`、`--usb-serial`。

| 函数、类型与形参 / Callable, type and parameters | 中文用途 / English behavior |
| --- | --- |
| <code>PausedReader</code> | 实现本节的采集阶段、数据／命令观察和有界状态验证。 / Implements the module's capture phases, data/command observation and bounded state checks. |
| <code>PausedReader.pump(self, deadline)</code> | 在截止时间内读取、解析新数据并更新采集状态。 / Reads and parses new input within the deadline and updates capture state. |
| <code>PausedReader.run(self)</code> | 在本对象定义的输入、时限与恢复约束下执行本节所述流程。 / Executes the operation described in this section under the object's input, deadline and recovery constraints. |

<a id="capture_load_benchmark"></a>

### capture_load_benchmark

在原始基准窗口增加 SERIAL/SCHED 时间和计数，读取负载诊断与控制回复。 / Adds SERIAL/SCHED timing and counter queries inside the benchmark windows and records the load diagnostics and replies. [源码 / Source](capture_load_benchmark.py)

命令 / CLI　`python3 tools/hil/capture_load_benchmark.py --help`。参数 / Arguments　`--output-directory`、`--label`、`--mode`、`--rows`、`--profile`、`--suite`、`--stability`、`--capture-seconds`、`--rail-avdd-uv`、`--rail-avss-uv`、`--rail-source-note`、`--expected-elf`、`--expected-session`、`--expected-boot-id`、`--transport`、`--usb-vid`、`--usb-pid`、`--usb-serial`。

| 函数、类型与形参 / Callable, type and parameters | 中文用途 / English behavior |
| --- | --- |
| <code>unsigned(value, command, field, bits=32)</code> | 读取并检查数值字段，保留该调用的范围及未知值规则。 / Reads a numeric field subject to the caller's range and unknown-value rules. |
| <code>validate_reply(command, fields)</code> | 核对查询或回复中的必需字段、范围及能力语义。 / Checks required reply fields, ranges and capability semantics. |
| <code>scheduler_health(fields)</code> | 由 SCHED 计数判定进度、deadline 或故障诊断。 / Derives progress, deadline or fault indicators from SCHED counters. |
| <code>LoadBenchmarkCollector</code> | 实现本节的采集阶段、数据／命令观察和有界状态验证。 / Implements the module's capture phases, data/command observation and bounded state checks. |
| <code>LoadBenchmarkCollector.__init__(self, *args, **kwargs)</code> | 使用所列参数构造本对象，初始化其计数、状态或 I/O 句柄。 / Constructs the object from the listed arguments and initializes its counters, state or I/O handles. |
| <code>LoadBenchmarkCollector.query(self, command, tag, phase_deadline)</code> | 发送查询并匹配其回复，遵守本采集器的截止时间和重试规则。 / Sends a query and matches its reply under the collector’s deadline and retry rules. |
| <code>LoadBenchmarkCollector.snapshot_window(self, name, deadline)</code> | 在有界窗口内采集前后状态及诊断查询。 / Captures state and diagnostic queries within a bounded observation window. |
| <code>LoadBenchmarkCollector.run(self)</code> | 在本对象定义的输入、时限与恢复约束下执行本节所述流程。 / Executes the operation described in this section under the object's input, deadline and recovery constraints. |
| <code>load_collector_selected()</code> | 在上下文内临时选择采集器实现，离开时恢复原配置。 / Selects a collector implementation for the context and restores the prior selection on exit. |
| <code>run_benchmark(options, **kwargs)</code> | 打开所选的采集器，在有界阶段内取证并关闭原始文件。 / Opens the selected collector, captures within bounded phases and closes the original evidence files. |
| <code>main(argv=None)</code> | 解析命令行，执行本模块的操作并按结果退出。 / Parses CLI arguments, runs the module operation and reports the resulting exit status. |

<a id="capture_mode_switches"></a>

### capture_mode_switches

执行 50 次切换，为每次保存权威状态、终态、首个同配置的完整数据；末尾恢复 CAP8。 / Executes 50 transitions, collecting authoritative state, terminal events and the first matching complete frame for each, then restores CAP8. [源码 / Source](capture_mode_switches.py)

命令 / CLI　`python3 tools/hil/capture_mode_switches.py --help`。参数 / Arguments　`--output-directory`、`--label`、`--usb-serial`、`--total-seconds`、`--expected-elf`、`--expected-session`、`--expected-boot-id`、`--rail-avdd-uv`、`--rail-avss-uv`、`--rail-source-note`。

| 函数、类型与形参 / Callable, type and parameters | 中文用途 / English behavior |
| --- | --- |
| <code>Options(output_directory: Path, label: str, usb_serial: str, total_seconds: float = 300.0, prepare_seconds: float = 30.0, cleanup_seconds: float = 10.0, command_timeout: float = 2.0, frame_timeout: float = 3.0, rail_avdd_uv: int &#124; None = None, rail_avss_uv: int &#124; None = None, rail_source_note: str &#124; None = None, expected_elf_sha256: str &#124; None = None, expected_session: str &#124; None = None, expected_boot_id: int &#124; None = None)</code> | 存放一次调用的工况、接口、期限或 profile；字段在下表列出。 / Stores case, interface, deadline or profile options for one invocation. Fields are listed below. |
| <code>unsigned(value, field)</code> | 读取并检查数值字段，保留该调用的范围及未知值规则。 / Reads a numeric field subject to the caller's range and unknown-value rules. |
| <code>forward_or_equal(value, reference)</code> | 比较同一 boot 的 uint32 序号，差值小于半个计数周期时视为向前。 / Compares two uint32 numbers from one boot and treats a difference below half a cycle as forward. |
| <code>require_config(config, mode=None)</code> | 核对 CONFIG 的模式、活动行、profile、运行状态及错误码。 / Checks CONFIG mode, active rows, profile, running state and error code. |
| <code>ModeSwitchCollector</code> | 实现本节的采集阶段、数据／命令观察和有界状态验证。 / Implements the module's capture phases, data/command observation and bounded state checks. |
| <code>ModeSwitchCollector.__init__(self, settings, configuration, port, raw, events, *, limits, start)</code> | 使用所列参数构造本对象，初始化其计数、状态或 I/O 句柄。 / Constructs the object from the listed arguments and initializes its counters, state or I/O handles. |
| <code>ModeSwitchCollector.check_fault(self, baseline=0)</code> | 检测新的故障、重启或回调异常，及时终止有问题的阶段。 / Checks for new faults, resets or callback exceptions and stops the affected phase. |
| <code>ModeSwitchCollector.pump(self, deadline)</code> | 在截止时间内读取、解析新数据并更新采集状态。 / Reads and parses new input within the deadline and updates capture state. |
| <code>ModeSwitchCollector.observe(self, line)</code> | 处理当前观测，保存本对象要求的字段、身份和时间。 / Consumes an observation and records the fields, identity and timing required by this object. |
| <code>ModeSwitchCollector.prepare(self)</code> | 准备采集条件或在预留的恢复时间内核对安全终态。 / Prepares capture conditions or checks the terminal recovery state within the reserved cleanup interval. |
| <code>ModeSwitchCollector.verify_new_frame(self, mode)</code> | 核查应用边界后首个完整帧，或依次执行 50 次切换。 / Checks the first complete frame after application or executes the 50-transition plan. |
| <code>ModeSwitchCollector.run_plan(self)</code> | 核查应用边界后首个完整帧，或依次执行 50 次切换。 / Checks the first complete frame after application or executes the 50-transition plan. |
| <code>ModeSwitchCollector.cleanup(self)</code> | 准备采集条件或在预留的恢复时间内核对安全终态。 / Prepares capture conditions or checks the terminal recovery state within the reserved cleanup interval. |
| <code>validate_options(options)</code> | 检查选项组合、身份和期限，错误时阻止开始采集。 / Checks option combinations, identities and deadlines before starting capture. |
| <code>run_capture(options, *, port_factory=open_capture_port)</code> | 打开所选的采集器，在有界阶段内取证并关闭原始文件。 / Opens the selected collector, captures within bounded phases and closes the original evidence files. |
| <code>main(argv=None)</code> | 解析命令行，执行本模块的操作并按结果退出。 / Parses CLI arguments, runs the module operation and reports the resulting exit status. |

<a id="capture_short_snapshots"></a>

### capture_short_snapshots

采集 CAP8，在一个有界事务内设置并恢复 BLE TX 短预览模式，记录受保护的完整快照。 / Captures CAP8 with BLE SHORT preview in one bounded transaction. Records protected full snapshots and verifies TX restoration. [源码 / Source](capture_short_snapshots.py)

命令 / CLI　`python3 tools/hil/capture_short_snapshots.py --help`。参数 / Arguments　`--output-directory`、`--label`、`--mode`、`--rows`、`--profile`、`--suite`、`--stability`、`--capture-seconds`、`--rail-avdd-uv`、`--rail-avss-uv`、`--rail-source-note`、`--expected-elf`、`--expected-session`、`--expected-boot-id`、`--transport`、`--usb-vid`、`--usb-pid`、`--usb-serial`。

| 函数、类型与形参 / Callable, type and parameters | 中文用途 / English behavior |
| --- | --- |
| <code>ShortSnapshotCollector</code> | 实现本节的采集阶段、数据／命令观察和有界状态验证。 / Implements the module's capture phases, data/command observation and bounded state checks. |
| <code>ShortSnapshotCollector.validate_fields(cls, command, fields)</code> | 核对查询或回复中的必需字段、范围及能力语义。 / Checks required reply fields, ranges and capability semantics. |
| <code>ShortSnapshotCollector.__init__(self, *args, **kwargs)</code> | 使用所列参数构造本对象，初始化其计数、状态或 I/O 句柄。 / Constructs the object from the listed arguments and initializes its counters, state or I/O handles. |
| <code>ShortSnapshotCollector.event(self, kind, **fields)</code> | 附加带时间和来源的事件或诊断输出。 / Appends timestamped event or diagnostic output with its source. |
| <code>ShortSnapshotCollector.set_tx_once(self, mode, phase_deadline)</code> | 仅提交一次 TX 变更并检查对应查询结果。 / Submits the TX change once and checks the matching query result. |
| <code>ShortSnapshotCollector.snapshot_window(self, name, deadline)</code> | 在有界窗口内采集前后状态及诊断查询。 / Captures state and diagnostic queries within a bounded observation window. |
| <code>ShortSnapshotCollector.run(self)</code> | 在本对象定义的输入、时限与恢复约束下执行本节所述流程。 / Executes the operation described in this section under the object's input, deadline and recovery constraints. |
| <code>main(argv=None)</code> | 解析命令行，执行本模块的操作并按结果退出。 / Parses CLI arguments, runs the module operation and reports the resulting exit status. |

<a id="capture_usb_unread"></a>

### capture_usb_unread

保持正式 CDC 打开但停止应用层读写 120 秒；前后查询硬件并校验 CAP8 恢复。 / Keeps formal CDC open without application reads or writes for 120 seconds, queries the device on both sides and checks CAP8 recovery. [源码 / Source](capture_usb_unread.py)

命令 / CLI　`python3 tools/hil/capture_usb_unread.py --help`。参数 / Arguments　`--output-directory`、`--label`、`--usb-serial`、`--stop-file`、`--expected-elf-sha256`、`--rail-avdd-uv`、`--rail-avss-uv`、`--rail-source-note`。

| 函数、类型与形参 / Callable, type and parameters | 中文用途 / English behavior |
| --- | --- |
| <code>Options(output_directory: Path, label: str, usb_serial: str, hold_seconds: float = 120.0, total_seconds: float = 200.0, prepare_seconds: float = 35.0, after_seconds: float = 15.0, cleanup_seconds: float = 20.0, command_timeout: float = 2.0, stop_file: Path &#124; None = None, rail_avdd_uv: int &#124; None = None, rail_avss_uv: int &#124; None = None, rail_source_note: str &#124; None = None, expected_elf_sha256: str &#124; None = None)</code> | 存放一次调用的工况、接口、期限或 profile；字段在下表列出。 / Stores case, interface, deadline or profile options for one invocation. Fields are listed below. |
| <code>Cancellation</code> | 记录停止及中断原因，供有界观察流程查询。 / Stores stop/interrupt reasons for bounded observation. |
| <code>Cancellation.__init__(self)</code> | 使用所列参数构造本对象，初始化其计数、状态或 I/O 句柄。 / Constructs the object from the listed arguments and initializes its counters, state or I/O handles. |
| <code>Cancellation.signal(self, number, _frame=None)</code> | 安装临时信号处理、记录取消请求或在安全点检测停止条件。 / Installs temporary signal handling, records cancellation or checks stop conditions at a safe point. |
| <code>Cancellation.check(self, stop_file)</code> | 安装临时信号处理、记录取消请求或在安全点检测停止条件。 / Installs temporary signal handling, records cancellation or checks stop conditions at a safe point. |
| <code>signal_scope(cancel)</code> | 安装临时信号处理、记录取消请求或在安全点检测停止条件。 / Installs temporary signal handling, records cancellation or checks stop conditions at a safe point. |
| <code>AuditedPort</code> | 提供有界端口读写、接口元数据和关闭操作。 / Provides bounded port I/O, interface metadata and closure. |
| <code>AuditedPort.__init__(self, port)</code> | 使用所列参数构造本对象，初始化其计数、状态或 I/O 句柄。 / Constructs the object from the listed arguments and initializes its counters, state or I/O handles. |
| <code>AuditedPort.metadata(self)</code> | 返回端口、设备身份、端点或通路能力信息。 / Returns port, device identity, endpoint or path capability metadata. |
| <code>AuditedPort.read_some(self, size, deadline)</code> | 在指定期限内尝试收发一段字节，返回实际进展。 / Attempts a bounded read or write and returns actual progress. |
| <code>AuditedPort.write_some(self, data, deadline)</code> | 在指定期限内尝试收发一段字节，返回实际进展。 / Attempts a bounded read or write and returns actual progress. |
| <code>AuditedPort.close(self)</code> | 关闭本对象拥有的端口、文件、线程或比对存储。 / Closes ports, files, threads or comparison storage owned by the object. |
| <code>u32(value, key)</code> | 读取并检查数值字段，保留该调用的范围及未知值规则。 / Reads a numeric field subject to the caller's range and unknown-value rules. |
| <code>require_target(config, mode, rows)</code> | 核对 CONFIG 的模式、活动行、profile、运行状态及错误码。 / Checks CONFIG mode, active rows, profile, running state and error code. |
| <code>UnreadCollector</code> | 实现本节的采集阶段、数据／命令观察和有界状态验证。 / Implements the module's capture phases, data/command observation and bounded state checks. |
| <code>UnreadCollector.validate_fields(cls, command, fields)</code> | 核对查询或回复中的必需字段、范围及能力语义。 / Checks required reply fields, ranges and capability semantics. |
| <code>UnreadCollector.__init__(self, *args, limits, start, cancel, **kwargs)</code> | 使用所列参数构造本对象，初始化其计数、状态或 I/O 句柄。 / Constructs the object from the listed arguments and initializes its counters, state or I/O handles. |
| <code>UnreadCollector.event(self, kind, **fields)</code> | 附加带时间和来源的事件或诊断输出。 / Appends timestamped event or diagnostic output with its source. |
| <code>UnreadCollector.observe(self, line)</code> | 处理当前观测，保存本对象要求的字段、身份和时间。 / Consumes an observation and records the fields, identity and timing required by this object. |
| <code>UnreadCollector.check_fault(self, baseline=0)</code> | 检测新的故障、重启或回调异常，及时终止有问题的阶段。 / Checks for new faults, resets or callback exceptions and stops the affected phase. |
| <code>UnreadCollector.pump(self, deadline)</code> | 在截止时间内读取、解析新数据并更新采集状态。 / Reads and parses new input within the deadline and updates capture state. |
| <code>UnreadCollector.query(self, command, tag, deadline)</code> | 发送查询并匹配其回复，遵守本采集器的截止时间和重试规则。 / Sends a query and matches its reply under the collector’s deadline and retry rules. |
| <code>UnreadCollector.verify_boot(self, config)</code> | 验证观测到的 boot、镜像或设备就绪条件。 / Checks the observed boot, image or readiness condition. |
| <code>UnreadCollector.verify_build(self, fields)</code> | 验证观测到的 boot、镜像或设备就绪条件。 / Checks the observed boot, image or readiness condition. |
| <code>UnreadCollector.verify_ready(self, ready, config)</code> | 验证观测到的 boot、镜像或设备就绪条件。 / Checks the observed boot, image or readiness condition. |
| <code>UnreadCollector.prepare(self)</code> | 准备采集条件或在预留的恢复时间内核对安全终态。 / Prepares capture conditions or checks the terminal recovery state within the reserved cleanup interval. |
| <code>UnreadCollector.snapshot(self, name, deadline)</code> | 在有界窗口内采集前后状态及诊断查询。 / Captures state and diagnostic queries within a bounded observation window. |
| <code>UnreadCollector.hold_unread(self, *, sleeper=time.sleep)</code> | 在 after 与 cleanup 预留预算足够时保持文件描述符打开，按单调时钟等待并轮询停止标志，记录窗口前后应用层 read/write 次数、原始位置及实际时长。 / Keeps the descriptor open only if the full hold fits before reserved after/cleanup budgets. Polls cancellation against monotonic time and records application read/write counts, raw position and elapsed time across the hold. |
| <code>UnreadCollector.cleanup(self)</code> | 准备采集条件或在预留的恢复时间内核对安全终态。 / Prepares capture conditions or checks the terminal recovery state within the reserved cleanup interval. |
| <code>validate_options(o)</code> | 检查选项组合、身份和期限，错误时阻止开始采集。 / Checks option combinations, identities and deadlines before starting capture. |
| <code>run_capture(options, *, port_factory=open_capture_port, sleeper=time.sleep, cancel=None)</code> | 打开所选的采集器，在有界阶段内取证并关闭原始文件。 / Opens the selected collector, captures within bounded phases and closes the original evidence files. |
| <code>main(argv=None)</code> | 解析命令行，执行本模块的操作并按结果退出。 / Parses CLI arguments, runs the module operation and reports the resulting exit status. |

<a id="check_software_recovery"></a>

### check_software_recovery

观察应用和 ROM 的 USB 枚举；只有显式请求时发送重启或下载命令，记录断开、重现和身份。 / Observes USB application/ROM enumeration. Sends restart or download commands only when explicitly requested and records removal, return and identity. [源码 / Source](check_software_recovery.py)

命令 / CLI　`python3 tools/hil/check_software_recovery.py --help`。参数 / Arguments　`--output-directory`、`--label`、`--usb-serial`、`--duration`、`--request-rom`、`--request-restart`。

| 函数、类型与形参 / Callable, type and parameters | 中文用途 / English behavior |
| --- | --- |
| <code>normalize_serial(value)</code> | 将 UUID、USB 序列号或 TX 模式规范为该后端匹配所用的形式。 / Normalizes a UUID, serial number or TX mode into the form its backend compares. |
| <code>usb_inventory(usb_root=Path(&#x27;/sys/bus/usb/devices&#x27;), tty_root=Path(&#x27;/sys/class/tty&#x27;))</code> | 枚举 Linux USB sysfs 节点或获取 tty 对应的设备身份。 / Enumerates Linux USB sysfs nodes or reads the device identity for a tty. |
| <code>epoch_key(device)</code> | 根据应用／ROM 身份、端口角色及出现世代识别同一枚举。 / Identifies the same enumeration epoch from app/ROM identity, port role and appearance. |
| <code>formal_role(device)</code> | 根据应用／ROM 身份、端口角色及出现世代识别同一枚举。 / Identifies the same enumeration epoch from app/ROM identity, port role and appearance. |
| <code>same_formal_epoch(expected, devices)</code> | 根据应用／ROM 身份、端口角色及出现世代识别同一枚举。 / Identifies the same enumeration epoch from app/ROM identity, port role and appearance. |
| <code>tty_node_metadata(paths)</code> | 枚举 Linux USB sysfs 节点或获取 tty 对应的设备身份。 / Enumerates Linux USB sysfs nodes or reads the device identity for a tty. |
| <code>PendingEnumerationNode</code> | 记录仍待核验的 USB 枚举设备节点。 / Holds a USB node whose enumeration identity is awaiting verification. |
| <code>PendingEnumerationNode.__init__(self, error)</code> | 使用所列参数构造本对象，初始化其计数、状态或 I/O 句柄。 / Constructs the object from the listed arguments and initializes its counters, state or I/O handles. |
| <code>collector_clock(clock)</code> | 定位采集器的单调时钟及时间对齐依据。 / Identifies the collector's monotonic clock and alignment evidence. |
| <code>RecordedLines</code> | 在追加协议行时执行观测回调或记录枚举世代。 / Invokes an observation callback or records the enumeration epoch as protocol lines are appended. |
| <code>RecordedLines.__init__(self, observer)</code> | 使用所列参数构造本对象，初始化其计数、状态或 I/O 句柄。 / Constructs the object from the listed arguments and initializes its counters, state or I/O handles. |
| <code>RecordedLines.append(self, line)</code> | 把当前记录或完整帧纳入本对象维护的样本／计数集合。 / Adds the current observation or complete frame to the object's sample or counter set. |
| <code>EpochCollector</code> | 实现本节的采集阶段、数据／命令观察和有界状态验证。 / Implements the module's capture phases, data/command observation and bounded state checks. |
| <code>EpochCollector.__init__(self, *args, writer, clock, epoch, **kwargs)</code> | 使用所列参数构造本对象，初始化其计数、状态或 I/O 句柄。 / Constructs the object from the listed arguments and initializes its counters, state or I/O handles. |
| <code>EpochCollector.event(self, kind, **fields)</code> | 附加带时间和来源的事件或诊断输出。 / Appends timestamped event or diagnostic output with its source. |
| <code>EpochCollector.progress(self)</code> | 记录当前采集相位与进度。 / Records the current capture phase and progress. |
| <code>EpochCollector.observe_line(self, line)</code> | 将收到的诊断行归入当前 USB 枚举 epoch。 / Associates a received diagnostic line with the current USB enumeration epoch. |
| <code>Options(output_directory: Path, label: str, usb_serial: str, action: str &#124; None = None, duration: float = 60.0, query_timeout: float = 2.0, poll_seconds: float = 0.1)</code> | 存放一次调用的工况、接口、期限或 profile；字段在下表列出。 / Stores case, interface, deadline or profile options for one invocation. Fields are listed below. |
| <code>validate_snapshot(replies, action=None)</code> | 核对软件重启／下载前的只读状态，验证允许执行请求的条件。 / Checks the read-only snapshot before restart/download and validates prerequisites for a request. |
| <code>observe(options, *, inventory=usb_inventory, port_factory=UsbCdcPort, clock=time.monotonic, sleep=time.sleep, sysfs_exists=lambda path: Path(path).exists(), node_metadata=tty_node_metadata)</code> | 处理当前观测，保存本对象要求的字段、身份和时间。 / Consumes an observation and records the fields, identity and timing required by this object. |
| <code>observe.event(kind, **fields)</code>（内部 / local） | 附加带时间和来源的事件或诊断输出。 / Appends timestamped event or diagnostic output with its source. |
| <code>observe.close_epoch(reason)</code>（内部 / local） | 关闭旧枚举记录或为新出现的设备打开一次 epoch 观察。 / Closes the previous enumeration record or starts an epoch observation for a newly appeared device. |
| <code>observe.open_epoch(device, *, allow_node_wait=False)</code>（内部 / local） | 关闭旧枚举记录或为新出现的设备打开一次 epoch 观察。 / Closes the previous enumeration record or starts an epoch observation for a newly appeared device. |
| <code>observe.snapshot(action=None)</code>（内部 / local） | 在有界窗口内采集前后状态及诊断查询。 / Captures state and diagnostic queries within a bounded observation window. |
| <code>main(argv=None)</code> | 解析命令行，执行本模块的操作并按结果退出。 / Parses CLI arguments, runs the module operation and reports the resulting exit status. |

<a id="check_wifi_owner_control"></a>

### check_wifi_owner_control

核验镜像及 CONFIG 身份后交替执行 AP/OFF，检查每个请求的终态、epoch 和资源分配跟踪。 / Verifies image and configuration identity, exercises AP/OFF requests, and checks request terminals, epochs and optional allocation tracing. [源码 / Source](check_wifi_owner_control.py)

命令 / CLI　`python3 tools/hil/check_wifi_owner_control.py --help`。参数 / Arguments　`--label`、`--usb-serial`、`--output-directory`、`--expected-elf`、`--expected-session`、`--expected-boot`、`--allocation-trace`。

| 函数、类型与形参 / Callable, type and parameters | 中文用途 / English behavior |
| --- | --- |
| <code>main()</code> | 解析命令行，执行本模块的操作并按结果退出。 / Parses CLI arguments, runs the module operation and reports the resulting exit status. |
| <code>main.same_wifi_boot(fields)</code>（内部 / local） | 核查 Wi-Fi 控制 boot、执行带请求 ID 的切换，或采集资源分配轨迹。 / Checks the Wi-Fi control boot, performs a request-ID-tracked switch or captures allocation trace evidence. |
| <code>main.query(command, tag, end)</code>（内部 / local） | 发送查询并匹配其回复，遵守本采集器的截止时间和重试规则。 / Sends a query and matches its reply under the collector’s deadline and retry rules. |
| <code>main.request(mode, end)</code>（内部 / local） | 核查 Wi-Fi 控制 boot、执行带请求 ID 的切换，或采集资源分配轨迹。 / Checks the Wi-Fi control boot, performs a request-ID-tracked switch or captures allocation trace evidence. |
| <code>main.allocation_trace(end)</code>（内部 / local） | 核查 Wi-Fi 控制 boot、执行带请求 ID 的切换，或采集资源分配轨迹。 / Checks the Wi-Fi control boot, performs a request-ID-tracked switch or captures allocation trace evidence. |

<a id="closed_matrix_performance"></a>

### closed_matrix_performance

从已封存工况中提取 CPU、堆、栈、传输、延迟和速率，维护每项指标的来源和 unknown。 / Extracts CPU, heap, stack, transport, timing and rates from closed cases with per-metric provenance and unknown states. [源码 / Source](closed_matrix_performance.py)

库入口 / Library　`import closed_matrix_performance`。

| 函数、类型与形参 / Callable, type and parameters | 中文用途 / English behavior |
| --- | --- |
| <code>require(ok, reason)</code> | 条件失败时以具体原因中止当前证据检查。 / Stops the current evidence check with its specific reason when the condition fails. |
| <code>bound_bytes(reference, directory)</code> | 验证封存路径和摘要，再返回原始或无损解压的字节。 / Verifies the closed path and digest before returning original or losslessly decompressed bytes. |
| <code>unknown(reason)</code> | 保留指标未知的原因。 / Preserves why a metric is unavailable. |
| <code>debug_observations(raw, events, capture, summary)</code> | 从有时钟与身份的日志提取调试观测、查询时延、堆或分层速率，组合性能表。 / Extracts debug observations, timing, heap or layer rates from clocked and identified evidence and assembles the performance table. |
| <code>collector_clock(events, summary)</code> | 定位采集器的单调时钟及时间对齐依据。 / Identifies the collector's monotonic clock and alignment evidence. |
| <code>query_timing_and_heap(events, raw, summary, expected, clock)</code> | 从有时钟与身份的日志提取调试观测、查询时延、堆或分层速率，组合性能表。 / Extracts debug observations, timing, heap or layer rates from clocked and identified evidence and assembles the performance table. |
| <code>counter_rates(window, names)</code> | 从同一计数域的前后差值和时间计算每秒速率。 / Derives per-second rates from counter deltas and elapsed time in the same domain. |
| <code>performance(attempt)</code> | 从有时钟与身份的日志提取调试观测、查询时延、堆或分层速率，组合性能表。 / Extracts debug observations, timing, heap or layer rates from clocked and identified evidence and assembles the performance table. |
| <code>performance_report(report)</code> | 将经过核验的字段和各层指标组成报告。 / Assembles a report from verified fields and layer-specific metrics. |
| <code>fmt(value, places=3)</code> | 将数值、接收情况或逐工况结果格式化进汇总表。 / Formats a numeric value, receipt result or case rows for the summary table. |
| <code>performance_table(report)</code> | 从有时钟与身份的日志提取调试观测、查询时延、堆或分层速率，组合性能表。 / Extracts debug observations, timing, heap or layer rates from clocked and identified evidence and assembles the performance table. |

<a id="fixture_rail"></a>

### fixture_rail

统一解析实测 AVDD、AVSS 和来源三个参数，确保仅在调用方提供有效组合时传递临时校准。 / Parses measured AVDD, AVSS and origin together, forwarding a volatile calibration only for a complete valid set. [源码 / Source](fixture_rail.py)

库入口 / Library　`import fixture_rail`。

| 函数、类型与形参 / Callable, type and parameters | 中文用途 / English behavior |
| --- | --- |
| <code>add_rail_arguments(parser)</code> | 向解析器登记该模块支持的命令行选项。 / Registers the module's command-line options with the supplied parser. |
| <code>rail_arguments(options)</code> | 校验 AVDD/AVSS 的极性和来源，将完整输入转换成子进程参数。 / Validates rail polarity and provenance, converting a complete set into collector CLI arguments. |

<a id="observe_bulk_boot"></a>

### observe_bulk_boot

等待符合 VID/PID、序列号和角色的 USB 接口，执行有界只读启动探测。 / Waits for the USB device identity and role, then issues bounded read-only boot queries. [源码 / Source](observe_bulk_boot.py)

命令 / CLI　`python3 tools/hil/observe_bulk_boot.py --help`。参数 / Arguments　`--output-directory`、`--label`、`--usb-serial`、`--transport`、`--usb-vid`、`--usb-pid`、`--duration`、`--observe-seconds`、`--command-timeout`。

| 函数、类型与形参 / Callable, type and parameters | 中文用途 / English behavior |
| --- | --- |
| <code>validate_boot(fields)</code> | 验证 BOOT 回复或在枚举时限内等待目标 USB 设备。 / Validates a BOOT reply or waits for the target USB device within the enumeration deadline. |
| <code>Options(output_directory: Path, label: str, usb_serial: str, duration: float = 90.0, observe_seconds: float = 30.0, command_timeout: float = 3.0, usb_vid: int = USB_VID, usb_pid: int &#124; None = None, transport: str = &#x27;usb_bulk&#x27;)</code> | 存放一次调用的工况、接口、期限或 profile；字段在下表列出。 / Stores case, interface, deadline or profile options for one invocation. Fields are listed below. |
| <code>wait_for_bulk(options, deadline, attempts, *, factory=UsbBulkPort, clock=time.monotonic, sleep=time.sleep)</code> | 验证 BOOT 回复或在枚举时限内等待目标 USB 设备。 / Validates a BOOT reply or waits for the target USB device within the enumeration deadline. |
| <code>observe(options, *, port_factory=None)</code> | 处理当前观测，保存本对象要求的字段、身份和时间。 / Consumes an observation and records the fields, identity and timing required by this object. |
| <code>main(argv=None)</code> | 解析命令行，执行本模块的操作并按结果退出。 / Parses CLI arguments, runs the module operation and reports the resulting exit status. |

<a id="observe_cdc_debug"></a>

### observe_cdc_debug

只打开调试 CDC 接口并保存原始字节、接收时间和退出状态。 / Opens only the CDC debug interface and records original bytes, receipt times and closure state. [源码 / Source](observe_cdc_debug.py)

命令 / CLI　`python3 tools/hil/observe_cdc_debug.py --help`。参数 / Arguments　`--output-directory`、`--label`、`--usb-serial`、`--usb-pid`、`--duration`、`--enumerate-seconds`、`--stop-file`。

| 函数、类型与形参 / Callable, type and parameters | 中文用途 / English behavior |
| --- | --- |
| <code>find_debug_tty(serial, *, pid=16449, sys_tty=Path(&#x27;/sys/class/tty&#x27;), dev_root=Path(&#x27;/dev&#x27;))</code> | 委托共享 sysfs 选择器，以所给序列号、PID 和接口 2 寻找调试 CDC，将缺设备或不唯一的枚举原因传递给观察器。 / Calls the shared sysfs selector with the supplied serial, PID and interface 2 for debug CDC, passing missing or ambiguous enumeration reasons to the observer. |
| <code>Options(output_directory: Path, label: str, usb_serial: str, duration: float = 9000, enumerate_seconds: float = 60, stop_file: Path &#124; None = None, usb_pid: int = 16449)</code> | 存放一次调用的工况、接口、期限或 profile；字段在下表列出。 / Stores case, interface, deadline or profile options for one invocation. Fields are listed below. |
| <code>observe(options, *, metadata_finder=None, port_factory=PosixPort)</code> | 处理当前观测，保存本对象要求的字段、身份和时间。 / Consumes an observation and records the fields, identity and timing required by this object. |
| <code>observe.event(kind, **fields)</code>（内部 / local） | 附加带时间和来源的事件或诊断输出。 / Appends timestamped event or diagnostic output with its source. |
| <code>main(argv=None)</code> | 解析命令行，执行本模块的操作并按结果退出。 / Parses CLI arguments, runs the module operation and reports the resulting exit status. |

<a id="qualify_closed_capture"></a>

### qualify_closed_capture

对已关闭的 USB、BLE、调试三路证据校验哈希、来源、采样时长、科学完整性及普通 BLE 完整帧缺口。 / Checks hashes, identity, duration, scientific integrity and normal-link BLE complete-frame gaps across three closed USB/BLE/debug recordings. [源码 / Source](qualify_closed_capture.py)

命令 / CLI　`python3 tools/hil/qualify_closed_capture.py --help`。参数 / Arguments　`--expected-elf`、`--expected-session`、`--expected-boot-id`、`--summary`、`--ble-summary`、`--debug-summary`、`--output`。

| 函数、类型与形参 / Callable, type and parameters | 中文用途 / English behavior |
| --- | --- |
| <code>require(condition, reason)</code> | 条件失败时以具体原因中止当前证据检查。 / Stops the current evidence check with its specific reason when the condition fails. |
| <code>events_file(path)</code> | 读取并定位采集器事件记录和时间元数据。 / Locates or reads collector event records and their timing metadata. |
| <code>verify_formal(summary_path, expected, minimum_seconds)</code> | 重算正式 CDC 原始字节的长度和 SHA-256，核验采集起止、CAP8 恢复、无新重启／故障及准备和清理两组 CONFIG?/BUILD?/CONFIG? 的原始请求轨迹。返回已封存摘要和可定位的证据引用。 / Rechecks raw formal CDC byte count and SHA-256, window bounds, CAP8 restoration, fault/reset evidence and the original prepare/cleanup CONFIG?/BUILD?/CONFIG? traces. Returns the closed summary and reproducible evidence references. |
| <code>verify_debug(summary_path, start, end, usb_serial)</code> | 检查调试采集已终止且无错误，重算原始字节的长度和哈希，核对唯一 opened 事件的接口角色与序列号，并确认其时间覆盖传入的 start 到 end。 / Checks clean closure, raw length and hash, the interface/serial from a unique opened event, and debug coverage from start through end. |
| <code>assess_science(report, label, expected)</code> | 按两路完整有效帧、采集跨度和有序序列统计判定数值完整性。 / Assesses integrity from ordered complete observations, window coverage and sequence gaps on both receivers. |
| <code>qualify_closed_case(summary_path, ble_summary_path, debug_summary_path, expected, minimum_seconds=120)</code> | 核实正常 FULL 工况中三路采集的原始引用、共同 boot、有效窗口和完整帧缺口，返回判定与来源。 / Verifies original references, shared boot, window coverage and complete-frame gaps across three collectors for one normal FULL case and returns verdict plus provenance. |
| <code>main(argv=None)</code> | 解析命令行，执行本模块的操作并按结果退出。 / Parses CLI arguments, runs the module operation and reports the resulting exit status. |

<a id="run_cdc_matrix_failfast"></a>

### run_cdc_matrix_failfast

分工况启动正式 USB、BLE 与独立调试观察器，按 CAP/VOLT/RES/MIXED 和 1–8 行运行；失败或证据不足即停止。 / Starts formal USB plus separate BLE and debug observers per case. Exercises CAP/VOLT/RES/MIXED over rows 1–8 and stops on failure or insufficient evidence. [源码 / Source](run_cdc_matrix_failfast.py)

命令 / CLI　`python3 tools/hil/run_cdc_matrix_failfast.py --help`。参数 / Arguments　`--output-directory`、`--label`、`--print-plan`、`--expected-elf`、`--expected-session`、`--expected-boot-id`、`--ble-address`、`--ble-adapter`、`--stop-file`、`--usb-serial`、`--start-case`、`--end-case`、`--pause-formal-reader`、`--fixture-margin`、`--rail-avdd-uv`、`--rail-avss-uv`、`--rail-source-note`。

| 函数、类型与形参 / Callable, type and parameters | 中文用途 / English behavior |
| --- | --- |
| <code>process_identity(pid)</code> | 用 Linux PID 和进程起始时间验证观察器未被替换，记录退出或未就绪的原因。 / Uses the Linux PID and process start time to detect replacement, exit or missing observer readiness. |
| <code>observer_problem(paths, identities=())</code> | 用 Linux PID 和进程起始时间验证观察器未被替换，记录退出或未就绪的原因。 / Uses the Linux PID and process start time to detect replacement, exit or missing observer readiness. |
| <code>validate_case(summary, expected_elf, previous_boot, requested_seconds=120)</code> | 核对采集时长、CAP8 恢复和设备镜像身份。 / Checks capture duration, CAP8 recovery and device image identity. |
| <code>run_child(command, log_path, observer_paths, stop_path, timeout=190, grace=12, observer_identities=(), reap_timeout=2)</code> | 启动正式采集，将 stdout/stderr 记录到新日志，监视观察器、操作者停止文件和总时限。中止时先发 SIGINT 并预留清理时间，必要时终止进程并有界等待回收，返回退出码、原因及是否确认退出。 / Starts the formal collector with a fresh combined log and monitors observer liveness, stop file and total timeout. On interruption sends SIGINT, allows cleanup, then kills and reaps within a deadline if necessary. Returns exit code, stop reason and whether termination was confirmed. |
| <code>close_observers(owners, stop_path)</code> | 创建观察器停止标志，逐个等待退出，必要时升级 SIGINT 与 kill；关闭各自日志，返回每进程的 PID、退出码、是否回收、错误及日志散列。 / Creates the stop marker and waits for each observer, escalating to SIGINT and kill when needed. Closes logs and reports PID, exit code, known termination, cleanup errors and log hash per process. |
| <code>start_observers(directory, label, args)</code> | 在每个工况开始前启动相互独立的 BLE 和 CDC 调试进程。 / Starts independent BLE and CDC debug processes before the case. |
| <code>main(argv=None)</code> | 解析命令行，执行本模块的操作并按结果退出。 / Parses CLI arguments, runs the module operation and reports the resulting exit status. |

<a id="run_final_observed_case"></a>

### run_final_observed_case

从当前设备身份生成 VOLT1 600 秒、50 次切换或 SHORT 的命令，等待两路观察器就绪并校验已关闭证据。 / Builds the VOLT1 600-second, 50-transition or SHORT command from the current identity, waits for both observers and verifies their closed evidence. [源码 / Source](run_final_observed_case.py)

命令 / CLI　`python3 tools/hil/run_final_observed_case.py --help`。参数 / Arguments　`case`、`--label`、`--output-directory`、`--ble-address`、`--ble-adapter`、`--usb-serial`、`--expected-elf`、`--expected-session`、`--expected-boot-id`、`--print-plan`、`--rail-avdd-uv`、`--rail-avss-uv`、`--rail-source-note`。

| 函数、类型与形参 / Callable, type and parameters | 中文用途 / English behavior |
| --- | --- |
| <code>save(path, record)</code> | 独占写入新记录或读取已有 JSON 证据。 / Creates a new exclusive record or reads existing JSON evidence. |
| <code>plan_case(args, expected)</code> | 按当前身份、工况和输出目录构造正式采集及两个观察器的命令。 / Builds the formal and both observer commands from the current identity, case and output directory. |
| <code>launch_observer(kind, command, directory, label)</code> | 有界启动、等待或回收子进程，保留观察器就绪和退出证据。 / Starts, awaits or reaps child processes within deadlines, retaining readiness and closure evidence. |
| <code>await_ready(plan, owners, summaries, address, serial)</code> | 最多等待 45 秒，核对调试 opened 的 USB 序列号与接口 2、BLE 地址及 C/D/L 三个订阅。遇停止标志、观察器退出、故障或提前关闭时不启动正式采集。 / Waits up to 45 seconds for the debug opened event on the expected serial/interface 2 and BLE readiness for the expected address and all C/D/L channels. Stop markers, observer exits, faults or early closure prevent formal capture. |
| <code>verify_journal(journal, start, end)</code> | 检查 BLE 订阅覆盖正式窗口、关闭状态及关键通道分片计数。 / Checks BLE subscription coverage, journal closure and critical-channel fragment counts. |
| <code>SwitchReceipts</code> | 按 CRC 与身份对应两种通路的完整帧。 / Correlates complete frames across two transports by CRC and identity. |
| <code>SwitchReceipts.__init__(self, steps)</code> | 使用所列参数构造本对象，初始化其计数、状态或 I/O 句柄。 / Constructs the object from the listed arguments and initializes its counters, state or I/O handles. |
| <code>SwitchReceipts.compare(self, _label, header, frame, _digest)</code> | 将接收帧与另一通路或要求的身份作逐帧比对。 / Compares a received frame against the other transport or expected identity. |
| <code>SwitchReceipts.result(self, _label)</code> | 返回适合序列化的字段、样本统计或比对结果。 / Returns serializable fields, sample statistics or comparison results. |
| <code>qualify_switches(summary_path, summary, journal_path, expected, start, end)</code> | 逐切换匹配应用终态、首帧与 BLE 被观察到的完整数据。 / Correlates each switch terminal and first-frame evidence with observed complete BLE data. |
| <code>qualify_short(summary_path, journal_path, debug_path, expected, start, end)</code> | 核对原 TX 设置、SHORT 采集和 TX 恢复的事件顺序及回复，再将 BLE 完整帧与 USB 身份和周期快照计数对应。返回已观测到的完整快照判定。 / Verifies the original TX setting and ordered SHORT/restore events and replies, then correlates complete BLE frames with USB identity and periodic snapshot counters. Returns the observed full-snapshot verdict. |
| <code>qualify(plan, expected, start, end)</code> | 按工况核对已关闭 BLE 通知、50 次终态或 SHORT 下的完整快照。 / Verifies the closed BLE journal, transition terminals or complete SHORT snapshots for the specified case. |
| <code>run(plan, expected)</code> | 在本对象定义的输入、时限与恢复约束下执行本节所述流程。 / Executes the operation described in this section under the object's input, deadline and recovery constraints. |
| <code>main(argv=None)</code> | 解析命令行，执行本模块的操作并按结果退出。 / Parses CLI arguments, runs the module operation and reports the resulting exit status. |
| <code>main.stop(_sig, _frame)</code>（内部 / local） | 请求停止当前接收循环或旁路任务。 / Requests termination of the active receiver loop or sidecar. |

<a id="summarize_closed_matrix"></a>

### summarize_closed_matrix

按 case 核对命令、门槛和摘要的路径与哈希，仅将相容身份和观测整合入帧率表。 / Verifies paths and digests for commands, gate and summary per case and combines compatible identities and observations into the FPS table. [源码 / Source](summarize_closed_matrix.py)

命令 / CLI　`python3 tools/hil/summarize_closed_matrix.py --help`。参数 / Arguments　`--directory`、`--label`、`--json`、`--markdown`、`--performance`。

| 函数、类型与形参 / Callable, type and parameters | 中文用途 / English behavior |
| --- | --- |
| <code>require(condition, reason)</code> | 条件失败时以具体原因中止当前证据检查。 / Stops the current evidence check with its specific reason when the condition fails. |
| <code>local_file(path, directory)</code> | 根据目录、标签和数据类型构造或解析输出文件路径。 / Builds or resolves output file paths by directory, label and record type. |
| <code>read_record(path)</code> | 独占写入新记录或读取已有 JSON 证据。 / Creates a new exclusive record or reads existing JSON evidence. |
| <code>argument(argv, flag)</code> | 从记录的命令中取出某个选项的值。 / Extracts an option value from the recorded command. |
| <code>sink_metrics(fps, counts, missing)</code> | 抽取可写入汇总表的字段，保留缺失标记。 / Projects observations into report columns while preserving missing values. |
| <code>closed_metrics(directory, formal_path, summary, gate, summary_digest)</code> | 抽取可写入汇总表的字段，保留缺失标记。 / Projects observations into report columns while preserving missing values. |
| <code>read_case(matrix, prefix, mode, rows)</code> | 验证并读取单工况文件，或合并同一工况的可信输入。 / Validates and reads case files or combines trusted inputs for one case. |
| <code>combine_case(rows)</code> | 验证并读取单工况文件，或合并同一工况的可信输入。 / Validates and reads case files or combines trusted inputs for one case. |
| <code>display(value, places=3)</code> | 将数值、接收情况或逐工况结果格式化进汇总表。 / Formats a numeric value, receipt result or case rows for the summary table. |
| <code>reception(sink)</code> | 将数值、接收情况或逐工况结果格式化进汇总表。 / Formats a numeric value, receipt result or case rows for the summary table. |
| <code>table(report)</code> | 将数值、接收情况或逐工况结果格式化进汇总表。 / Formats a numeric value, receipt result or case rows for the summary table. |
| <code>main()</code> | 解析命令行，执行本模块的操作并按结果退出。 / Parses CLI arguments, runs the module operation and reports the resulting exit status. |

<a id="summarize_diagnostic_screen"></a>

### summarize_diagnostic_screen

比较两份诊断运行，在各自采集窗口内汇总采集、传输、故障、堆和观察缺口。 / Compares two diagnostic runs using each run's own capture window, reporting throughput, faults, heap and missing observations. [源码 / Source](summarize_diagnostic_screen.py)

命令 / CLI　`python3 tools/hil/summarize_diagnostic_screen.py --help`。参数 / Arguments　`--verdict`。

| 函数、类型与形参 / Callable, type and parameters | 中文用途 / English behavior |
| --- | --- |
| <code>sha(path: Path) → str</code> | 计算文件或字节的摘要，为结果绑定实际输入。 / Computes a digest for file or byte evidence so results can be traced to the actual input. |
| <code>read(path: Path) → dict</code> | 读取指定输入并按格式保留原始字节、行或哈希。 / Reads the specified input, preserving its original bytes, lines or digest as required by the format. |
| <code>stability_observation(summary: dict) → dict</code> | 抽取单一稳定性窗口中实际观测到的指标。 / Extracts metrics actually observed during one stability window. |
| <code>missing_observations(counts: dict, groups: list) → dict</code> | 提取稳定性／缺失观测，或载入并比较两次运行的实测指标。 / Extracts stability/missing observations or loads and compares observed metrics across runs. |
| <code>layer_window(value: dict) → dict</code> | 按同一身份的起止观测计算分层计数窗口。 / Derives layer-counter windows from bracketing observations with the same identity. |
| <code>capture_summary(analysis: dict, summary: dict, cpu: dict) → dict</code> | 按配置或片段键汇总帧、科学状态、传输计数及未知数据。 / Summarizes frames, scientific status, transport counters and unknowns by configuration or segment key. |
| <code>compare_runs(previous: dict, current: dict) → dict</code> | 提取稳定性／缺失观测，或载入并比较两次运行的实测指标。 / Extracts stability/missing observations or loads and compares observed metrics across runs. |
| <code>compare_runs.own(r)</code>（内部 / local） | 从当前记录取得带来源的局部观测。 / Extracts a local observation with its source from the current record. |
| <code>load_run(analysis_path: Path, summary_path: Path, cpu_path: Path) → tuple[dict, dict]</code> | 提取稳定性／缺失观测，或载入并比较两次运行的实测指标。 / Extracts stability/missing observations or loads and compares observed metrics across runs. |
| <code>main() → int</code> | 解析命令行，执行本模块的操作并按结果退出。 / Parses CLI arguments, runs the module operation and reports the resulting exit status. |

## 参数与数据边界 / Parameter conventions

字节偏移从捕获文件起点开始，`_us` 或 `Us` 表示微秒。主机 BLE 接收时间使用注明的单调时钟。分片、完整帧、传输提交和采集各自有独立分母。 / Byte offsets start at the beginning of the capture. The suffix `_us` or `Us` denotes microseconds. BLE receipt times use the named monotonic clock. Fragments, complete frames, submissions and device captures have separate denominators.

| 参数名 / Parameter names | 含义 / Meaning |
| --- | --- |
| `analysis_path`, `ble_summary_path`, `cpu_path`, `debug_path`, `debug_summary_path`, `dev_root`, `formal_path`, `identity_path`, `log_path`, `path`, `paths`, `post_path`, `stop_path`, `summary_path`, `sys_tty`, `sysfs_exists`, `tty_root`, `usb_root` | 相应的输入文件、输出位置、仓库／SDK 根或 sysfs 设备目录。 / Matching input/output location, repository/SDK root or sysfs device directory. |
| `analysis`, `build`, `capture`, `events`, `journal`, `metrics`, `record`, `report`, `snapshot`, `state`, `summary`, `timing`, `window` | 已捕获的字节流、快照、前后界、观察窗口／元数据、输入分析或结果对象。 / Captured stream, snapshot, brackets, observation window/metadata, analysis or result. |
| `context`, `index`, `info`, `inventory`, `key`, `node_metadata`, `offset`, `part`, `reference`, `source`, `tail` | 目标条目、索引、字节边界、设备清单或原始证据引用。 / Target item, index, byte boundary, device inventory or original evidence reference. |
| `cpu`, `gate`, `groups`, `identities`, `inputs`, `matrix`, `observer_identities`, `owners`, `plan`, `role`, `summaries` | 一组 CPU 角色、闭合判据、身份、观察器进程、运行计划或分工况摘要。 / CPU role, gate, identities, observer owners, run plan or a group of per-case summaries. |
| `config`, `group`, `ident`, `mode`, `probe`, `probe_kind`, `rows`, `scope`, `sink`, `transport` | 工况、行数、逐行模式或采集／控制／诊断类别；按调用范围选择。 / Case, row count, per-row profile or capture/control/diagnostic class for this operation. |
| `field`, `fields`, `keys`, `label`, `name`, `prefix`, `suffix`, `tag`, `text` | 待解析或输出的键、标签、字段名、前后缀或说明文本。 / Key, label, field name, prefix, suffix or explanatory text for parsing/output. |
| `collector_class`, `device`, `devices`, `metadata_finder`, `observer`, `port`, `port_factory`, `writer` | 调用方提供的端口、收发对象、硬件适配器或可替换的测试实现。 / Caller-provided I/O object, adapter or replaceable test implementation. |
| `_args`, `args`, `argv`, `kwargs`, `o`, `options`, `settings` | 调用参数、解析后的命令行选项或调用方提供的关键字参数。 / Invocation arguments, parsed CLI options or caller-supplied keyword arguments. |
| `_frame`, `_sig`, `backend`, `cls`, `factory`, `function`, `self` | 回调提供的发送端、信号、栈帧、函数或类／实例接收者。 / Callback sender, signal, stack frame, function or class/instance receiver. |
| `after`, `baseline`, `before`, `current`, `previous`, `previous_boot` | 用于前后对照、恢复或同一次配置检查的两端状态。 / Bracketing state for a delta, recovery or a single configuration check. |
| `bits`, `counts`, `fps`, `missing`, `places`, `stats` | 比率、误差、位宽、速率或计数。percent 表示百分比，fraction 表示 0–1 的比例。 / Ratio, error, bit width, rate or count; percent is a percentage and fraction uses 0–1. |
| `condition`, `error`, `ok`, `ready`, `reason` | 等待条件、故障原因、回退值或就绪／排队状态。 / Wait condition, fault reason, fallback value or readiness/queued state. |
| `data`, `frame`, `header`, `raw`, `value` | 帧头、行／尾、未改写载荷或原始测点值。 / Header, rows/trailer, unmodified bytes or original cell observations. |
| `expected_config`, `expected_elf`, `expected_group`, `expected_sha`, `summary_digest` | 预期镜像摘要、配置身份、后端或字段约束，用于校验实际观测。 / Expected image digest, config identity, backend or field contract checked against observations. |
| `action`, `command`, `command_records`, `queries` | 协议命令、查询集、事务请求号或单次命令等待预算。 / Wire command, query set, transaction request ID or per-command timeout. |
| `address`, `pid`, `serial`, `usb_serial` | 通路选择、蓝牙通道、端口、进程号或设备标识，按所在函数解释。 / Transport/channel selection, port, process ID or device identity as indicated by the function. |
| `attempts`, `maximum`, `size`, `steps` | 样本／帧／字节数量、允许误差、队列上限或栈／堆限值，由变量名标明计量对象。 / Count, tolerance, queue bound or stack/heap limit; the parameter name specifies the unit being counted. |
| `begin`, `end`, `hold_end`, `start` | 观测边界或接收时刻，带 ns 后缀的参数使用纳秒。 / Observation bounds or receipt times. The ns suffix denotes nanoseconds. |
| `event_replies`, `line`, `raw_lines`, `replies` | 收到的协议行、应答或终态原文。 / Received protocol lines, acknowledgments or terminal text. |
| `allow_failed_revision`, `allow_node_wait`, `full` | 选择校验强度、操作范围、重放或测试来源的标志／谓词。 / Flag or predicate controlling validation, scope, replacement or synthetic provenance. |
| `attempt`, `number`, `r` | 当前尝试号、数值或正在处理的记录。 / Current attempt, numeric value or record being processed. |
| `ble_path`, `observer_paths`, `stop_file` | 已封存的 BLE 证据、观察器摘要或操作者停止标志的路径。 / Path to BLE evidence, observer summaries or the operator stop marker. |
| `clock`, `sleep`, `sleeper` | 提供单调计时或等待的函数，可用于无硬件的确定性测试。 / Clock or wait function, injectable for deterministic host testing. |
| `configuration`, `flag`, `phase_deadline` | 设备控制参数、TX 输出策略、应恢复的设置或输入建立时间；us 表示微秒。 / Device controls, TX policy, settings to restore or input settling duration; us denotes microseconds. |
| `directory`, `journal_path`, `output` | 目标文件、日志或目录；相对路径由调用点声明的根目录解释。 / Output file, journal or directory. Relative paths refer to the root specified by the caller. |
| `grace`, `reap_timeout`, `timeout` | 允许等待的时间上限，以秒为单位，若接口注明其它单位则遵循该接口。 / Maximum wait, in seconds unless the interface specifies a different unit. |
| `post`, `pre`, `seen` | 探测前后的观测或已处理的条目集合。 / Pre/post-probe observations or the set of already processed items. |
| `_digest`, `_label` | 回调保留的摘要或标签；下划线表示该实现可能无需读取它。 / Callback digest or label; an underscore denotes a potentially unused argument. |
| `cancel`, `parser` | 解析器、校验函数、回调、比对存储或异步停止信号。 / Parser, validator, callback, comparison store or asynchronous stop signal. |
| `epoch`, `session` | boot/session、配置代或帧序号，从同一身份内进行比较。 / Boot/session identity, configuration generation or frame sequence compared within its identity domain. |
| `fixture_report`, `proxy` | 夹具参考位置、已知元件、公差、背景供体或实录／合成来源。 / Fixture coordinates, known part, tolerance, background donor or recorded/synthetic provenance. |
| `io`, `spi` | USB bulk、I/O 或 SPI 原始事务记录。 / Original USB bulk, I/O or SPI transaction evidence. |
| `kind`, `phase` | 测试或采集阶段及相应约束。 / Test/capture phase and its applicable constraints. |
| `minimum_seconds`, `requested_seconds` | 采样、驻留、静默、预热、查询或收尾区间，单位为秒。 / Capture, dwell, quiet, warmup, query or cleanup interval in seconds. |
| `deadline` | 主机单调时钟的绝对截止时刻；嵌套操作共享上层预算。 / Absolute host-monotonic deadline shared with enclosing operations. |
| `expected` | 候选、被选择或要求匹配的观测／配置集合。 / Candidate, selected or required observations/configurations. |
| `limits` | 该次采集的配置及运行预算。 / Per-capture configuration and runtime budgets. |

## 回归测试入口 / Regression entry points

在仓库根目录运行。测试使用合成 I/O 检查有界退出、镜像绑定、终态／首帧、快照和证据闭合。最后一项默认回收临时证据；传入 `--output` 可显式保留新报告和同目录的证据。 / Run at the repository root. Synthetic I/O checks bounded closure, identity binding, terminals/first frames, snapshots and evidence closure. The last command cleans its temporary evidence by default. Pass `--output` to retain a new report and its sibling evidence directory.

~~~sh
python3 -m unittest discover -s tools/hil
python3 tools/hil/test_check_wifi_owner_control.py
python3 tools/hil/test_software_recovery.py
~~~

**传输、控制及输出 / Transports, control and output**　[test_analyze_ble_protection](test_analyze_ble_protection.py) · [test_bulk_stage](test_bulk_stage.py) · [test_capture_short_snapshots](test_capture_short_snapshots.py) · [test_capture_usb_unread](test_capture_usb_unread.py) · [test_check_wifi_owner_control](test_check_wifi_owner_control.py) · [test_software_recovery](test_software_recovery.py)

**采集器、诊断及资源 / Collectors, diagnostics and resources**　[test_analyze_cpu_owner](test_analyze_cpu_owner.py) · [test_capture_mode_switches](test_capture_mode_switches.py) · [test_closed_matrix_performance](test_closed_matrix_performance.py) · [test_qualify_closed_capture](test_qualify_closed_capture.py) · [test_run_cdc_matrix_failfast](test_run_cdc_matrix_failfast.py) · [test_summarize_closed_matrix](test_summarize_closed_matrix.py) · [test_summarize_diagnostic_screen](test_summarize_diagnostic_screen.py)

**ADS、rail、电池 / ADS, rails and battery**　[test_capture_ads_timing](test_capture_ads_timing.py)

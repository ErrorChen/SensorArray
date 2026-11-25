# 主机工具与 Python API / Host tools and Python API

从仓库根目录运行本页命令。Python 3.10 及以上支持工具使用的类型语法。BLE 实机接收使用 Bleak，部分串口接收器需要 pyserial。Linux sysfs 可以用 USB 序列号和接口角色区分正式 CDC 数据接口与 debug 接口；vendor bulk 后端使用 PyUSB。设备采集、传输提交和主机完整收帧分别计数。 / Run from the repository root. Python 3.10+ supports the type annotations. Live BLE uses Bleak, some serial tools use pyserial, and vendor bulk uses PyUSB. Linux sysfs exposes serial and interface role for formal and debug CDC. Acquisition, submissions and host complete frames are measured separately. [HIL 依赖 / HIL dependencies](requirements-hil.txt) · [硬件验收 / HIL collectors](hil/README.md) · [测试方法 / Test methods](../docs/validation.md) · [项目 / Project](../README.md)

## 调用示例 / Examples

~~~sh
python3 tools/reference_fixture.py --input tests/fixtures/reference_capture/cap8_total.recorded.txt --provenance recorded
python3 tools/sensorarray_hil.py serial --help
python3 tools/sensorarray_hil.py ble --help
python3 tools/validate_transports.py --help
~~~

下面的 `feed_line` 按原始顺序接收已解码的一行，保留协议的 CRC、身份和无效标志。 / To reuse the parser from Python, pass one decoded line at a time in original order, retaining CRC, identity and invalid flags.

~~~sh
PYTHONPATH=tools python3 - <<'PYTHON'
from pathlib import Path
from text_protocol import TextProtocolParser

parser = TextProtocolParser()
sample = Path('tests/fixtures/reference_capture/cap8_total.recorded.txt')
for line in sample.read_text(encoding='ascii').splitlines():
    parser.feed_line(line)
print(parser.summary("FIXTURE"))
PYTHON
~~~

将新采集写入 `validation_artifacts/<run>/` 并使用新标签。原始 RX、事件、散列和最终 summary 需要保持对应关系。固定的最小重放输入在 tests/fixtures 中。 / Write new captures under `validation_artifacts/<run>/` with a fresh label. Preserve the relationship among raw RX, events, hashes and final summaries. Minimal versioned replay inputs live under tests/fixtures. [测试夹具 / Fixtures](../tests/README.md)

### 其它工具文件 / Other tool files

| 文件 / File | 作用与调用 / Purpose and invocation |
| --- | --- |
| [requirements-hil.txt](requirements-hil.txt) | 用 `python3 -m pip install -r tools/requirements-hil.txt` 将 Bleak、pyserial 安装到所选主机环境。 / Installs host Bleak/pyserial into the chosen Python environment. |
| [setup_sensorarray_usb_access.sh](setup_sensorarray_usb_access.sh) | `bash tools/setup_sensorarray_usb_access.sh --dry-run` 展示临时 Linux udev 规则；实际执行针对脚本指定序列号的 303a:4041 vendor bulk 设备，需要 root。 / Dry-run prints the temporary udev rule. Applying it requires root and targets the specified 303a:4041 bulk device and serial. |
| [SensorArray_CEE525_profile.xml](SensorArray_CEE525_profile.xml) | Windows WLAN 手动连接配置，SSID 固定为 SensorArray_CEE525。 / Windows manual WLAN profile for this specific SSID. |

### 常用选项及单位 / Principal CLI options and units

| 选项 / Option | 使用场景与含义 / Usage and meaning |
| --- | --- |
| `--port`、`--transport` | `posix` 按给定串口路径打开，`usb_cdc` 选择正式双 CDC 数据接口，`usb_bulk` 选择 vendor bulk。 / POSIX opens the specified path, usb_cdc selects formal dual-CDC data, and usb_bulk selects vendor bulk. |
| `--usb-vid`、`--usb-pid`、`--usb-serial` | 通过 USB 标识和 12 个无冒号十六进制数字匹配设备。正式 CDC 的 PID 为 0x4042。 / Select the USB device by product identity and a 12-hex serial with no colons. Formal CDC uses PID 0x4042. |
| `--mode`、`--rows`、`--profile` | 基准模式为 CAP、VOLT、RES、MIXED；活动行数 1–8。profile 是八个 C/V/R 字符，与活动行数分别设置。 / Benchmark mode is CAP, VOLT, RES or MIXED. Set 1–8 active rows independently from the full eight-character C/V/R profile. |
| `--capture-seconds`、`--stability`、`--suite` | 单工况以秒设定；stability 对给定 mode/rows 使用 600 秒窗口（默认 VOLT1），suite 展开四模式 × 八种行数。 / Single-case duration is in seconds. Stability selects a 600-second window for the chosen mode and row count (default VOLT1); suite expands four modes over eight row counts. |
| `--expected-elf`、`--expected-session`、`--expected-boot-id` | 对正式写操作传入当前完整 ELF SHA-256、16 个十六进制字符的 session、十进制 boot ID。 / Bind formal configuration changes to the board’s 64-hex ELF SHA-256, 16-hex session and decimal boot ID. |
| `--rail-avdd-uv`、`--rail-avss-uv`、`--rail-source-note` | 当前板 AVDD、AVSS 对 GND 的带符号微伏读数和测点／读数出处。 / Signed rail measurements in microvolts relative to board ground and the location/source for that measurement. |
| `--output-directory`、`--label`、`--directory`、`--output` | 采集使用新 run 路径与标签；离线分析用 directory 读取已有工况并按工具选项写出报告。 / Collectors use a fresh run path and label. Offline analyzers read existing cases from directory and write the selected report. |
| `--ble-journal`、`--debug-summary` | 指定已关闭的 BLE 通知日志或独立 CDC debug 摘要，以便核对 PC 实收和覆盖区间。 / Supply a closed BLE notification journal or independent CDC debug summary to verify host receipt and window coverage. |

## 模块索引 / Module index

[analyze_adc_read_provenance](#analyze_adc_read_provenance) · [analyze_firmware_benchmark](#analyze_firmware_benchmark) · [audit_config_surface](#audit_config_surface) · [capture_benchmark_ble](#capture_benchmark_ble) · [capture_reference_fixture](#capture_reference_fixture) · [check_command_docs](#check_command_docs) · [check_no_legacy_config_usage](#check_no_legacy_config_usage) · [estimate_floating_cap_background](#estimate_floating_cap_background) · [plot_firmware_benchmark](#plot_firmware_benchmark) · [receive_ble_text](#receive_ble_text) · [receive_serial_text](#receive_serial_text) · [receive_wifi_text](#receive_wifi_text) · [reference_fixture](#reference_fixture) · [run_firmware_benchmark](#run_firmware_benchmark) · [run_firmware_benchmark_retry](#run_firmware_benchmark_retry) · [sensorarray_bulk_diagnostics](#sensorarray_bulk_diagnostics) · [sensorarray_capture_identity](#sensorarray_capture_identity) · [sensorarray_hil](#sensorarray_hil) · [sensorarray_usb_bulk](#sensorarray_usb_bulk) · [sensorarray_usb_cdc](#sensorarray_usb_cdc) · [summarize_reference_capture](#summarize_reference_capture) · [text_protocol](#text_protocol) · [validate_battery_formula](#validate_battery_formula) · [validate_measurement_modes](#validate_measurement_modes) · [validate_transports](#validate_transports)

下表沿用源码签名。数据类列出字段、类型及缺省值，实例方法通过对象调用，`self` 由 Python 绑定。标为“内部”的函数只在所属函数内使用。 / Signatures follow the source. Dataclass constructors list field types and defaults. Call instance methods through their object, which binds `self`. Entries marked *local* are used inside their enclosing function.

<a id="analyze_adc_read_provenance"></a>

### analyze_adc_read_provenance

对 ADSREAD… 寄存器／SPI 明细独立重算 CRC，返回逐次读数的证据。 / Recomputes the ADC read CRC against captured wire bytes and groups evidence for individual checked reads. [源码 / Source](analyze_adc_read_provenance.py)

命令 / CLI　`python3 tools/analyze_adc_read_provenance.py --help`。参数 / Arguments　`--directory`、`--phase`、`--output`、`--evidence-kind`。

| 函数、类型与形参 / Callable, type and parameters | 中文用途 / English behavior |
| --- | --- |
| <code>crc_division(data)</code> | 逐位作多项式除法，独立重算采样字节的 CRC。 / Computes CRC by bitwise polynomial division over the supplied sample bytes. |
| <code>hex_bytes(fields, key, length)</code> | 将十六进制寄存器／链路字段解码为字节。 / Decodes hexadecimal register or wire fields into bytes. |
| <code>decode_record(line)</code> | 解码单个带原始 SPI 信息的 ADC 诊断记录。 / Decodes one ADC diagnostic record with its original SPI evidence. |
| <code>analyze(directory, phase)</code> | 消费输入证据并返回本模块定义的逐项分析结果。 / Consumes input evidence and returns the module's per-item analysis. |
| <code>main()</code> | 解析命令行，执行本模块的操作并按结果退出。 / Parses CLI arguments, runs the module operation and reports the resulting exit status. |

<a id="analyze_firmware_benchmark"></a>

### analyze_firmware_benchmark

读取已关闭的 summary、原始字节、事件和可选 BLE 日志；按 boot、配置及时间窗口计算完整帧、序号缺口、设备／主机速率和各层计数。 / Reads closed summaries, bytes, event timelines and optional BLE journals. Groups complete frames, sequence gaps, rates and layer counters by boot, configuration and time window. [源码 / Source](analyze_firmware_benchmark.py)

命令 / CLI　`python3 tools/analyze_firmware_benchmark.py --help`。参数 / Arguments　`--directory`、`--output`、`--ble-journal`、`--debug-summary`、`--selection-output`、`--replace`。

| 函数、类型与形参 / Callable, type and parameters | 中文用途 / English behavior |
| --- | --- |
| <code>integer(value)</code> | 读取并检查数值字段，保留该调用的范围及未知值规则。 / Reads a numeric field subject to the caller's range and unknown-value rules. |
| <code>memory_telemetry_record_error(tag, line)</code> | 检查堆、栈及内存能力字段，返回记录不可用于分析的原因。 / Checks heap, stack and capability fields and returns the reason a telemetry record is unusable. |
| <code>file_hash(path)</code> | 计算文件或字节的摘要，为结果绑定实际输入。 / Computes a digest for file or byte evidence so results can be traced to the actual input. |
| <code>event_lines(path)</code> | 读取并定位采集器事件记录和时间元数据。 / Locates or reads collector event records and their timing metadata. |
| <code>event_metadata(path)</code> | 读取并定位采集器事件记录和时间元数据。 / Locates or reads collector event records and their timing metadata. |
| <code>RxCursor</code> | 按接收事件和字节偏移为原始行定位时间。 / Maps offsets in original input to host receive events and times. |
| <code>RxCursor.__init__(self, path, window)</code> | 使用所列参数构造本对象，初始化其计数、状态或 I/O 句柄。 / Constructs the object from the listed arguments and initializes its counters, state or I/O handles. |
| <code>RxCursor.at(self, offset)</code> | 以位置或接收时间在已索引的时间线上查找对应记录。 / Looks up an observation in an indexed receive timeline by offset or time. |
| <code>location(item, window)</code> | 将字节区间映射到源文件及偏移。 / Maps a byte range to its source file and offset. |
| <code>Snapshots</code> | 保留诊断计数快照并计算同域差值。 / Stores diagnostic counter snapshots and computes same-domain deltas. |
| <code>Snapshots.__init__(self)</code> | 使用所列参数构造本对象，初始化其计数、状态或 I/O 句柄。 / Constructs the object from the listed arguments and initializes its counters, state or I/O handles. |
| <code>Snapshots.add(self, item, window)</code> | 把当前记录或完整帧纳入本对象维护的样本／计数集合。 / Adds the current observation or complete frame to the object's sample or counter set. |
| <code>Snapshots.compact(self)</code> | 抽取可写入汇总表的字段，保留缺失标记。 / Projects observations into report columns while preserving missing values. |
| <code>Snapshots.delta(self, keys, continuity, window_label=&#x27;query_snapshots_not_exact_capture_boundaries&#x27;)</code> | 在规定的计数域和边界上计算增量，处理可识别的回绕和未知情况。 / Computes a counter delta under the specified domain and bounds, accounting for supported wraps and unknowns. |
| <code>expected_wire_contract(declared, config)</code> | 从工况确定预期 wire 类型与行数，或列出实际帧偏离这些条件的字段。 / Derives the expected wire type and row contract, or reports fields in an observed frame that depart from it. |
| <code>frame_wire_contract_errors(frame, header, contract)</code> | 从工况确定预期 wire 类型与行数，或列出实际帧偏离这些条件的字段。 / Derives the expected wire type and row contract, or reports fields in an observed frame that depart from it. |
| <code>Group</code> | 汇总同配置的完整帧、收帧时间和有效样本。 / Aggregates complete frames, receive times and valid samples for one configuration. |
| <code>Group.__init__(self, header, frame, expected_identity=None, expected_contract=None)</code> | 使用所列参数构造本对象，初始化其计数、状态或 I/O 句柄。 / Constructs the object from the listed arguments and initializes its counters, state or I/O handles. |
| <code>Group.add(self, frame, header, trailer, raw_rows, host, wire_bytes, payload_bytes, raw_trailer=None)</code> | 把当前记录或完整帧纳入本对象维护的样本／计数集合。 / Adds the current observation or complete frame to the object's sample or counter set. |
| <code>Group.observe_ordered_host_receipt(self, host)</code> | 记录或汇总同一配置内主机有序收到完整帧的时间间隔。 / Records or summarizes host receipt intervals for ordered complete frames within one configuration. |
| <code>Group.host_complete_frame_intervals(self)</code> | 记录或汇总同一配置内主机有序收到完整帧的时间间隔。 / Records or summarizes host receipt intervals for ordered complete frames within one configuration. |
| <code>Group.output(self)</code> | 返回适合序列化的字段、样本统计或比对结果。 / Returns serializable fields, sample statistics or comparison results. |
| <code>requested_configuration(summary, label)</code> | 对命令行要求展开或辨识具体的模式、行数和 profile。 / Expands or identifies requested measurement modes, row counts and profiles. |
| <code>analyse_capture(directory, label, summary, fingerprints=None, debug_summaries=())</code> | 解析输入帧并按配置身份汇总采集窗口。 / Parses the input stream and groups capture observations by configuration identity. |
| <code>fingerprint_key(header, frame)</code> | 为完整帧构建跨传输比对所需的身份键。 / Builds the identity key used to correlate complete frames across transports. |
| <code>Fingerprints</code> | 按 CRC 与身份对应两种通路的完整帧。 / Correlates complete frames across two transports by CRC and identity. |
| <code>Fingerprints.__init__(self, path)</code> | 使用所列参数构造本对象，初始化其计数、状态或 I/O 句柄。 / Constructs the object from the listed arguments and initializes its counters, state or I/O handles. |
| <code>Fingerprints.add(self, label, header, frame, digest)</code> | 把当前记录或完整帧纳入本对象维护的样本／计数集合。 / Adds the current observation or complete frame to the object's sample or counter set. |
| <code>Fingerprints.compare(self, label, header, frame, digest)</code> | 将接收帧与另一通路或要求的身份作逐帧比对。 / Compares a received frame against the other transport or expected identity. |
| <code>Fingerprints.result(self, label)</code> | 返回适合序列化的字段、样本统计或比对结果。 / Returns serializable fields, sample statistics or comparison results. |
| <code>Fingerprints.close(self)</code> | 关闭本对象拥有的端口、文件、线程或比对存储。 / Closes ports, files, threads or comparison storage owned by the object. |
| <code>BleWindow</code> | 从给定订阅区间收集并结束 BLE 帧观测。 / Accumulates and finalizes BLE frame observations in a subscription window. |
| <code>BleWindow.__init__(self, capture, start, end)</code> | 使用所列参数构造本对象，初始化其计数、状态或 I/O 句柄。 / Constructs the object from the listed arguments and initializes its counters, state or I/O handles. |
| <code>BleWindow.add_frame(self, frame, pending, trailer, received_ns, fingerprints)</code> | 把当前记录或完整帧纳入本对象维护的样本／计数集合。 / Adds the current observation or complete frame to the object's sample or counter set. |
| <code>BleWindow.finish(self, ready_intervals, fingerprints, source_ok)</code> | 完成当前窗口，计算覆盖范围、完整帧与缺口。 / Finalizes the window and calculates coverage, complete frames and gaps. |
| <code>ble_prefix_lines(path, evidence)</code> | 从 BLE 数据中取得保留的前缀行，用于片段和载荷分类。 / Extracts retained BLE prefix lines for classifying fragments and payload. |
| <code>analyse_ble_journal(path, captures, fingerprints)</code> | 按通知时间、分片和订阅区间重建 BLE 完整帧与缺口。 / Reconstructs complete BLE frames and gaps using notification times, fragments and subscription coverage. |
| <code>analyse_ble_journal.at(ns)</code>（内部 / local） | 以位置或接收时间在已索引的时间线上查找对应记录。 / Looks up an observation in an indexed receive timeline by offset or time. |
| <code>analyse_ble_journal.retire_fragments(reason)</code>（内部 / local） | 结束过期的 BLE 重组槽，并记录不完整分片。 / Retires stale BLE assemblies and records incomplete fragments. |
| <code>choose_capture(captures)</code> | 按采集时间／身份为观测选择关联的工况。 / Selects the matching captured case for an observation using time or identity. |
| <code>choose_capture.order(capture)</code>（内部 / local） | 提供稳定的显示／处理排序键。 / Provides a stable sort key for ordered output or processing. |
| <code>compact_row(mode, rows, captures)</code> | 抽取可写入汇总表的字段，保留缺失标记。 / Projects observations into report columns while preserving missing values. |
| <code>_analyse_directory(directory, ble_journal=None, fingerprints=None, debug_summaries=())</code> | 对目录中的已关闭工况匹配 USB、BLE 和诊断证据。 / Matches USB, BLE and diagnostic evidence for closed cases in a directory. |
| <code>analyse_directory(directory, ble_journal=None, debug_summaries=None)</code> | 对目录中的已关闭工况匹配 USB、BLE 和诊断证据。 / Matches USB, BLE and diagnostic evidence for closed cases in a directory. |
| <code>write_report(path, result, replace=False)</code> | 将经过核验的字段和各层指标组成报告。 / Assembles a report from verified fields and layer-specific metrics. |
| <code>main(argv=None)</code> | 解析命令行，执行本模块的操作并按结果退出。 / Parses CLI arguments, runs the module operation and reports the resulting exit status. |

<a id="audit_config_surface"></a>

### audit_config_surface

解析工程与 SDK 的 Kconfig、defaults 和源码，找出未定义选项、使用默认值和本地 fallback 的位置。 / Cross-references project/SDK Kconfig, defaults and source uses to locate undeclared options and local fallbacks. [源码 / Source](audit_config_surface.py)

运行 / Run　`python3 tools/audit_config_surface.py`。

| 函数、类型与形参 / Callable, type and parameters | 中文用途 / English behavior |
| --- | --- |
| <code>is_generated_build_path(path: Path) → bool</code> | 判别生成路径或枚举待审计的 Kconfig／代码文件。 / Identifies generated paths or enumerates Kconfig/source files to audit. |
| <code>iter_kconfig_files() → list[Path]</code> | 判别生成路径或枚举待审计的 Kconfig／代码文件。 / Identifies generated paths or enumerates Kconfig/source files to audit. |
| <code>rel(path: Path) → str</code> | 相对于给定根路径格式化文件名。 / Formats a path relative to the supplied root. |
| <code>iter_source_files() → list[Path]</code> | 判别生成路径或枚举待审计的 Kconfig／代码文件。 / Identifies generated paths or enumerates Kconfig/source files to audit. |
| <code>load_text(path: Path) → str</code> | 读取指定输入并按格式保留原始字节、行或哈希。 / Reads the specified input, preserving its original bytes, lines or digest as required by the format. |
| <code>source_code(path: Path) → str</code> | 从源码文本去除非代码部分以供配置宏匹配。 / Extracts searchable source text for configuration macro matching. |
| <code>find_sdk_root() → Path &#124; None</code> | 定位 SDK，或从相应输入收集选项定义、默认值、使用处及 fallback。 / Locates the SDK or collects option definitions, defaults, uses or fallbacks from the corresponding inputs. |
| <code>collect_sdk_defined(sdk_root: Path &#124; None) → set[str]</code> | 定位 SDK，或从相应输入收集选项定义、默认值、使用处及 fallback。 / Locates the SDK or collects option definitions, defaults, uses or fallbacks from the corresponding inputs. |
| <code>collect_defined() → dict[str, list[str]]</code> | 定位 SDK，或从相应输入收集选项定义、默认值、使用处及 fallback。 / Locates the SDK or collects option definitions, defaults, uses or fallbacks from the corresponding inputs. |
| <code>collect_defaults() → set[str]</code> | 定位 SDK，或从相应输入收集选项定义、默认值、使用处及 fallback。 / Locates the SDK or collects option definitions, defaults, uses or fallbacks from the corresponding inputs. |
| <code>collect_used() → dict[str, list[str]]</code> | 定位 SDK，或从相应输入收集选项定义、默认值、使用处及 fallback。 / Locates the SDK or collects option definitions, defaults, uses or fallbacks from the corresponding inputs. |
| <code>collect_fallbacks() → dict[str, list[str]]</code> | 定位 SDK，或从相应输入收集选项定义、默认值、使用处及 fallback。 / Locates the SDK or collects option definitions, defaults, uses or fallbacks from the corresponding inputs. |
| <code>print_list(title: str, items: list[str], limit: int=80) → None</code> | 格式化并输出审计命中列表。 / Formats and prints the set of audit findings. |
| <code>main() → int</code> | 解析命令行，执行本模块的操作并按结果退出。 / Parses CLI arguments, runs the module operation and reports the resulting exit status. |

<a id="capture_benchmark_ble"></a>

### capture_benchmark_ble

在 Linux BLE 上按地址订阅 C、D、L，保存原始通知、单调时钟和订阅区间。另一个进程拥有 USB 控制。 / Subscribes to C, D and L on a selected BLE peripheral. Persists original notifications, monotonic times and subscription intervals while another process owns USB control. [源码 / Source](capture_benchmark_ble.py)

命令 / CLI　`python3 tools/capture_benchmark_ble.py --help`。参数 / Arguments　`--address`、`--name`、`--adapter`、`--output-directory`、`--label`、`--duration`、`--stop-file`。

| 函数、类型与形参 / Callable, type and parameters | 中文用途 / English behavior |
| --- | --- |
| <code>Options(address: str, output_directory: Path, label: str, name: str &#124; None = None, adapter: str = &#x27;hci0&#x27;, duration: float = 4500.0, stop_file: Path &#124; None = None, scan_seconds: float = 10.0, connect_seconds: float = 15.0, operation_seconds: float = 5.0, cleanup_seconds: float = 10.0, progress_seconds: float = 20.0)</code> | 存放一次调用的工况、接口、期限或 profile；字段在下表列出。 / Stores case, interface, deadline or profile options for one invocation. Fields are listed below. |
| <code>Journal</code> | 记录 BLE 通知、订阅就绪和退出事件。 / Records BLE notifications, subscription readiness and closure events. |
| <code>Journal.__init__(self, stream)</code> | 使用所列参数构造本对象，初始化其计数、状态或 I/O 句柄。 / Constructs the object from the listed arguments and initializes its counters, state or I/O handles. |
| <code>Journal.emit(self, kind, *, monotonic_ns=None, **fields)</code> | 附加带时间和来源的事件或诊断输出。 / Appends timestamped event or diagnostic output with its source. |
| <code>Journal.flush(self)</code> | 将缓冲的日志或原始字节推送到对应输出流。 / Flushes buffered journal or original byte data to its output stream. |
| <code>Recorder</code> | 记录 BLE 通知、订阅就绪和退出事件。 / Records BLE notifications, subscription readiness and closure events. |
| <code>Recorder.__init__(self, options, journal, ready_path)</code> | 使用所列参数构造本对象，初始化其计数、状态或 I/O 句柄。 / Constructs the object from the listed arguments and initializes its counters, state or I/O handles. |
| <code>Recorder.request_stop(self, reason)</code> | 请求停止当前接收循环或旁路任务。 / Requests termination of the active receiver loop or sidecar. |
| <code>Recorder.callback(self, channel)</code> | 处理订阅回调送来的通知字节，交给对应通道的接收器。 / Handles received notification bytes and dispatches them to their channel receiver. |
| <code>Recorder.callback.received(_sender, data)</code>（内部 / local） | 闭包回调，接收本次订阅的字节并保留通道／时间信息。 / Receives subscribed bytes in a closure that retains channel/timing context. |
| <code>Recorder.on_disconnect(self, _client)</code> | 记录断连事件并唤醒等待中的接收流程。 / Records a disconnect and wakes the waiting receiver flow. |
| <code>Recorder.bounded(self, awaitable, seconds, *, cleanup=False)</code> | 在给定截止时间内等待 BLE 操作。 / Awaits a BLE operation within its assigned deadline. |
| <code>Recorder.run(self, scanner, client_class)</code> | 在本对象定义的输入、时限与恢复约束下执行本节所述流程。 / Executes the operation described in this section under the object's input, deadline and recovery constraints. |
| <code>capture(options, *, backend=None)</code> | 打开所选的采集器，在有界阶段内取证并关闭原始文件。 / Opens the selected collector, captures within bounded phases and closes the original evidence files. |
| <code>main(argv=None)</code> | 解析命令行，执行本模块的操作并按结果退出。 / Parses CLI arguments, runs the module operation and reports the resulting exit status. |

<a id="capture_reference_fixture"></a>

### capture_reference_fixture

按选定的串口、USB Bulk 或 CDC 通路设置夹具工况，逐字节记录测量帧和请求事件，按截止时间查询、恢复和关闭。 / Configures a fixture capture over serial, USB bulk or CDC. Records original bytes and command events, queries and restores the device within bounded phases. [源码 / Source](capture_reference_fixture.py)

命令 / CLI　`python3 tools/capture_reference_fixture.py --help`。参数 / Arguments　`--port`、`--transport`、`--usb-vid`、`--usb-pid`、`--usb-serial`、`--duration`、`--output-directory`、`--label`、`--mode`、`--rowmodes`、`--rows`、`--command-timeout`、`--progress-seconds`。

| 函数、类型与形参 / Callable, type and parameters | 中文用途 / English behavior |
| --- | --- |
| <code>CaptureError</code> | 用于调用方区分采集／身份／传输失败或环境跳过的异常类型。 / Exception type for distinguishing capture/identity/transport failure or an unavailable test environment. |
| <code>ReceivedLine</code> | 保存收到的行／帧、故障原因或时间元数据。 / Holds received line/frame data, a classified fault or timing metadata. |
| <code>ReceivedLine.__new__(cls, text, channel)</code> | 创建附带接收元数据的行对象。 / Creates a line object with receive metadata attached. |
| <code>PosixPort</code> | 提供有界端口读写、接口元数据和关闭操作。 / Provides bounded port I/O, interface metadata and closure. |
| <code>PosixPort.__init__(self, path: str)</code> | 使用所列参数构造本对象，初始化其计数、状态或 I/O 句柄。 / Constructs the object from the listed arguments and initializes its counters, state or I/O handles. |
| <code>PosixPort.metadata(self)</code> | 返回端口、设备身份、端点或通路能力信息。 / Returns port, device identity, endpoint or path capability metadata. |
| <code>PosixPort.read_some(self, max_bytes, deadline)</code> | 在指定期限内尝试收发一段字节，返回实际进展。 / Attempts a bounded read or write and returns actual progress. |
| <code>PosixPort.write_some(self, data, deadline)</code> | 在指定期限内尝试收发一段字节，返回实际进展。 / Attempts a bounded read or write and returns actual progress. |
| <code>PosixPort.close(self)</code> | 关闭本对象拥有的端口、文件、线程或比对存储。 / Closes ports, files, threads or comparison storage owned by the object. |
| <code>CaptureOptions(port: str, duration: float, output_directory: Path, label: str, mode: str &#124; None = None, rows: int &#124; None = None, command_timeout: float = 10.0, progress_seconds: float = 20.0, rowmodes: str &#124; None = None, transport: str = &#x27;posix&#x27;, usb_vid: int = 12346, usb_pid: int &#124; None = None, usb_serial: str &#124; None = None)</code> | 存放一次调用的工况、接口、期限或 profile；字段在下表列出。 / Stores case, interface, deadline or profile options for one invocation. Fields are listed below. |
| <code>add_transport_arguments(parser)</code> | 向解析器登记该模块支持的命令行选项。 / Registers the module's command-line options with the supplied parser. |
| <code>transport_usb_pid(options)</code> | 为所选 USB 后端解析或选择产品 ID。 / Resolves or selects the product ID for the chosen USB backend. |
| <code>validate_transport_options(options)</code> | 检查选项组合、身份和期限，错误时阻止开始采集。 / Checks option combinations, identities and deadlines before starting capture. |
| <code>open_capture_port(options)</code> | 根据配置打开串口、USB CDC 或 USB bulk 数据端口。 / Opens the serial, USB CDC or USB bulk data port selected in the options. |
| <code>Collector</code> | 实现本节的采集阶段、数据／命令观察和有界状态验证。 / Implements the module's capture phases, data/command observation and bounded state checks. |
| <code>Collector.__init__(self, options: CaptureOptions, port, raw, events, *, ctrl_raw=None)</code> | 使用所列参数构造本对象，初始化其计数、状态或 I/O 句柄。 / Constructs the object from the listed arguments and initializes its counters, state or I/O handles. |
| <code>Collector.event(self, kind, **fields)</code> | 附加带时间和来源的事件或诊断输出。 / Appends timestamped event or diagnostic output with its source. |
| <code>Collector.progress(self)</code> | 记录当前采集相位与进度。 / Records the current capture phase and progress. |
| <code>Collector.pump(self, deadline)</code> | 在截止时间内读取、解析新数据并更新采集状态。 / Reads and parses new input within the deadline and updates capture state. |
| <code>Collector.send(self, command: str, deadline)</code> | 按通道封装并发送一次控制命令。 / Formats and submits one control command on the selected channel. |
| <code>Collector.validate_protocol(self, fields)</code> | 正式 OTG 会话要求 PROTO version=2 与 config_query_v1，同时要求与所选后端相符的 usb_cdc_data_v1 或 usb_vendor_bulk_v1。无匹配时在配置前抛出 CaptureError。 / Requires version 2, config_query_v1 and either usb_cdc_data_v1 or usb_vendor_bulk_v1 for the chosen formal OTG backend. Raises CaptureError before configuration when requirements are missing. |
| <code>Collector.transport_result(self)</code> | 汇总当前传输后端观察到的结果及错误。 / Reports observed outcomes and errors from the active transport. |
| <code>Collector.command_deadline(self, phase_deadline)</code> | 为一个命令计算受阶段总预算约束的截止时间。 / Derives a command deadline bounded by the remaining phase budget. |
| <code>Collector.next_line(self, deadline)</code> | 等待接收队列中满足条件的下一行或观测。 / Waits for the next line or matching observation from the receive queue. |
| <code>Collector.wait(self, predicate, deadline, command)</code> | 等待接收队列中满足条件的下一行或观测。 / Waits for the next line or matching observation from the receive queue. |
| <code>Collector.query(self, command, tag, phase_deadline)</code> | 发送查询并匹配其回复，遵守本采集器的截止时间和重试规则。 / Sends a query and matches its reply under the collector’s deadline and retry rules. |
| <code>Collector.set_usb(self, mode, phase_deadline)</code> | 设置采集要求的 USB 输出策略并核对回复。 / Sets the requested USB stream policy and checks the reply. |
| <code>Collector.transition(self, kind, value)</code> | 为 MODE、ROWS 或 ROWMODES 写入一次目标值，分别匹配带请求 ID 的 ACK 与终态，再查询对应状态确认 active 等字段。拒绝、终态超时或查询与目标不符均为失败。 / Issues the MODE, ROWS or ROWMODES setter once, correlates its request ID with the acknowledgment and terminal, then queries the matching state. Rejection, a missing terminal and disagreement with the active target each fail the transition. |
| <code>Collector.run(self)</code> | 在本对象定义的输入、时限与恢复约束下执行本节所述流程。 / Executes the operation described in this section under the object's input, deadline and recovery constraints. |
| <code>capture(options: CaptureOptions) → dict</code> | 打开所选的采集器，在有界阶段内取证并关闭原始文件。 / Opens the selected collector, captures within bounded phases and closes the original evidence files. |
| <code>main()</code> | 解析命令行，执行本模块的操作并按结果退出。 / Parses CLI arguments, runs the module operation and reports the resulting exit status. |

<a id="check_command_docs"></a>

### check_command_docs

从实际 C 格式串和命令分支抽取诊断字段，检查协议文档覆盖情况。 / Extracts command literals and diagnostic fields from C and checks the protocol documentation. [源码 / Source](check_command_docs.py)

命令 / CLI　`python3 tools/check_command_docs.py --help`。参数 / Arguments　`--list`。

| 函数、类型与形参 / Callable, type and parameters | 中文用途 / English behavior |
| --- | --- |
| <code>decode_c_literal(value: str) → str</code> | 解码 C 字符串或提取命令字段，找出协议表缺少的列。 / Decodes C string literals or extracts command fields and identifies entries missing from protocol tables. |
| <code>add_matches(found: Dict[str, List[str]], source: Path, patterns: Iterable[re.Pattern]) → None</code> | 解码 C 字符串或提取命令字段，找出协议表缺少的列。 / Decodes C string literals or extracts command fields and identifies entries missing from protocol tables. |
| <code>extract_literals() → Dict[str, List[str]]</code> | 解码 C 字符串或提取命令字段，找出协议表缺少的列。 / Decodes C string literals or extracts command fields and identifies entries missing from protocol tables. |
| <code>missing_columns(document: str) → List[str]</code> | 解码 C 字符串或提取命令字段，找出协议表缺少的列。 / Decodes C string literals or extracts command fields and identifies entries missing from protocol tables. |
| <code>main() → int</code> | 解析命令行，执行本模块的操作并按结果退出。 / Parses CLI arguments, runs the module operation and reports the resulting exit status. |

<a id="check_no_legacy_config_usage"></a>

### check_no_legacy_config_usage

搜索生产代码中的弃用配置宏，排除生成文件，返回命中位置和退出状态。 / Scans project sources for retired configuration macros, excluding generated files, and reports their locations via CLI status. [源码 / Source](check_no_legacy_config_usage.py)

运行 / Run　`python3 tools/check_no_legacy_config_usage.py`。

| 函数、类型与形参 / Callable, type and parameters | 中文用途 / English behavior |
| --- | --- |
| <code>is_generated_build_path(relative_path: Path) → bool</code> | 判别生成路径或枚举待审计的 Kconfig／代码文件。 / Identifies generated paths or enumerates Kconfig/source files to audit. |
| <code>iter_files() → list[Path]</code> | 判别生成路径或枚举待审计的 Kconfig／代码文件。 / Identifies generated paths or enumerates Kconfig/source files to audit. |
| <code>main() → int</code> | 解析命令行，执行本模块的操作并按结果退出。 / Parses CLI arguments, runs the module operation and reports the resulting exit status. |

<a id="estimate_floating_cap_background"></a>

### estimate_floating_cap_background

在同一身份窗口内，从其它悬空单元估算 CAP 背景及分布；记录有效样本与供体范围。 / Estimates CAP background distributions from other floating cells in one identity window and reports contributing samples and donors. [源码 / Source](estimate_floating_cap_background.py)

命令 / CLI　`python3 tools/estimate_floating_cap_background.py --help`。参数 / Arguments　`--input`、`--output`。

| 函数、类型与形参 / Callable, type and parameters | 中文用途 / English behavior |
| --- | --- |
| <code>donor_sets(target: str) → dict[str, tuple[str, ...]]</code> | 按目标行列选择可用的悬空参考点集合。 / Selects sets of floating donor cells for a target coordinate. |
| <code>distribution(values: list[float]) → dict</code> | 汇总有效数值的样本数、分位数及分布。 / Summarizes valid sample counts, quantiles and distribution. |
| <code>distribution.percentile(fraction)</code>（内部 / local） | 在已排序的样本分布中求指定分位数。 / Computes the requested quantile from the sorted sample distribution. |
| <code>_memory_diagnostic(line: str) → bool</code> | 提取内存／栈报警，为背景估值记录运行条件。 / Extracts memory/stack diagnostics to qualify the background estimate. |
| <code>_complete_frames(lines: Iterable[str])</code> | 筛选完整帧或其中可用的 CAP 单元及坐标。 / Selects complete frames or their usable CAP cells and coordinates. |
| <code>_cap_cells(frame, header, raw_rows) → dict</code> | 筛选完整帧或其中可用的 CAP 单元及坐标。 / Selects complete frames or their usable CAP cells and coordinates. |
| <code>MethodSamples(donors: tuple[str, ...], background: list[float] = field(default_factory=list), target: list[float] = field(default_factory=list), difference: list[float] = field(default_factory=list), coverage: list[float] = field(default_factory=list), rejections: Counter = field(default_factory=Counter), missing_donors: dict[str, Counter] = field(default_factory=dict))</code> | 保存按方法、坐标或工况汇总的输入或结果。 / Holds inputs or results grouped by method, cell coordinate or test stage. |
| <code>MethodSamples.output(self, frames: int) → dict</code> | 返回适合序列化的字段、样本统计或比对结果。 / Returns serializable fields, sample statistics or comparison results. |
| <code>TargetSamples(label: str, total: list[float] = field(default_factory=list), rejections: Counter = field(default_factory=Counter), methods: dict[str, MethodSamples] = field(init=False), ranges: list[float] = field(default_factory=list), pairs: dict[str, list[float]] = field(default_factory=dict))</code> | 保存按方法、坐标或工况汇总的输入或结果。 / Holds inputs or results grouped by method, cell coordinate or test stage. |
| <code>TargetSamples.__post_init__(self)</code> | 检查或初始化构造参数所派生的字段。 / Checks or initializes fields derived from the constructor arguments. |
| <code>TargetSamples.output(self, frames: int) → dict</code> | 返回适合序列化的字段、样本统计或比对结果。 / Returns serializable fields, sample statistics or comparison results. |
| <code>_estimate_target(target: TargetSamples, cells: dict, frame_rejections: list[str]) → dict</code> | 由悬空供体分布估算目标点背景并记录不确定性。 / Estimates the target background from floating donors and records uncertainty. |
| <code>estimate_capture(lines: Iterable[str], source: dict &#124; None=None) → dict</code> | 由悬空供体分布估算目标点背景并记录不确定性。 / Estimates the target background from floating donors and records uncertainty. |
| <code>read_input(path: Path) → tuple[list[str], dict]</code> | 读取指定输入并按格式保留原始字节、行或哈希。 / Reads the specified input, preserving its original bytes, lines or digest as required by the format. |
| <code>main(argv=None) → int</code> | 解析命令行，执行本模块的操作并按结果退出。 / Parses CLI arguments, runs the module operation and reports the resulting exit status. |

<a id="plot_firmware_benchmark"></a>

### plot_firmware_benchmark

将已有基准汇总中的设备和主机 FPS 画为图，缺失数值保持缺失。 / Plots device and host FPS from an existing benchmark summary, preserving missing values. [源码 / Source](plot_firmware_benchmark.py)

命令 / CLI　`python3 tools/plot_firmware_benchmark.py --help`。参数 / Arguments　`--input`、`--output-directory`、`--stem`、`--label-prefix`、`--selection`、`--synthetic`。

| 函数、类型与形参 / Callable, type and parameters | 中文用途 / English behavior |
| --- | --- |
| <code>finite_rate(value)</code> | 检查有限数值，避免将缺失或非有限速率用于统计。 / Checks finite numbers so missing or non-finite rates stay out of statistics. |
| <code>extract_rates(report, label_prefix=None, selection=None)</code> | 提取分工况速率或绘制设备、USB 和 BLE 帧率图。 / Extracts per-case rates or renders the device, USB and BLE FPS plot. |
| <code>render(report_path, output_directory=None, stem=&#x27;firmware-benchmark-fps&#x27;, label_prefix=None, synthetic=False, selection_path=None)</code> | 提取分工况速率或绘制设备、USB 和 BLE 帧率图。 / Extracts per-case rates or renders the device, USB and BLE FPS plot. |
| <code>main(argv=None)</code> | 解析命令行，执行本模块的操作并按结果退出。 / Parses CLI arguments, runs the module operation and reports the resulting exit status. |

<a id="receive_ble_text"></a>

### receive_ble_text

连接指定 BLE 外设，订阅选择的通道，检查分片序号和 CRC，重组并解析文本，也支持命令和旁路串口。 / Connects to a BLE peripheral, subscribes to selected channels, reassembles and checks fragments, parses text, and supports command writes and a serial sidecar. [源码 / Source](receive_ble_text.py)

命令 / CLI　`python3 tools/receive_ble_text.py --help`。参数 / Arguments　`--name-prefix`、`--rows`、`--tx`、`--duration`、`--scan-seconds`、`--tail`、`--show-fragments`、`--interactive`、`--command`、`--save-log`、`--safe`、`--serial-port`、`--serial-baud`。

| 函数、类型与形参 / Callable, type and parameters | 中文用途 / English behavior |
| --- | --- |
| <code>first_non_ascii(payload: bytes) → tuple[int, int] &#124; None</code> | 查找非 ASCII 字节位置或解码一行，并保留错误证据。 / Locates a non-ASCII byte or decodes a line while retaining evidence of encoding errors. |
| <code>parse_fragment_identity(fallback: str, payload: bytes) → tuple[str, int, int]</code> | 从分片头读出通道、消息标识和分片序号。 / Reads channel, message identity and fragment position from the fragment header. |
| <code>SerialSidecar</code> | 管理指定接口的数据接收、解码、控制或验收。 / Manages receipt, decoding, control or validation on the designated interface. |
| <code>SerialSidecar.__init__(self, port: str &#124; None, baud: int, log_file: TextIO &#124; None) → None</code> | 使用所列参数构造本对象，初始化其计数、状态或 I/O 句柄。 / Constructs the object from the listed arguments and initializes its counters, state or I/O handles. |
| <code>SerialSidecar.start(self) → None</code> | 打开设备或启动该对象的后台接收任务。 / Opens the device or starts the object’s background receiver task. |
| <code>SerialSidecar.stop(self) → None</code> | 请求停止当前接收循环或旁路任务。 / Requests termination of the active receiver loop or sidecar. |
| <code>SerialSidecar._write(self, line: str) → None</code> | 提交指定字节或文字，按对象的约定保存接收／写入进度。 / Submits the given bytes or text and records progress according to the object's stream contract. |
| <code>SerialSidecar._run(self) → None</code> | 运行后台接收循环，将原始数据投递给解析器。 / Runs the background receive loop and forwards original data to the parser. |
| <code>BleTextReceiver</code> | 管理指定接口的数据接收、解码、控制或验收。 / Manages receipt, decoding, control or validation on the designated interface. |
| <code>BleTextReceiver.__init__(self, args: argparse.Namespace, log_file: TextIO &#124; None) → None</code> | 使用所列参数构造本对象，初始化其计数、状态或 I/O 句柄。 / Constructs the object from the listed arguments and initializes its counters, state or I/O handles. |
| <code>BleTextReceiver._write(self, line: str) → None</code> | 提交指定字节或文字，按对象的约定保存接收／写入进度。 / Submits the given bytes or text and records progress according to the object's stream contract. |
| <code>BleTextReceiver._on_cap_frame(self, frame) → None</code> | 处理解析器回调送来的完整测量帧。 / Consumes a completed measurement frame delivered by the parser callback. |
| <code>BleTextReceiver._on_measurement_frame(self, frame) → None</code> | 处理解析器回调送来的完整测量帧。 / Consumes a completed measurement frame delivered by the parser callback. |
| <code>BleTextReceiver.process_message(self, channel: str, payload: bytes) → None</code> | 将重组后的消息分发给文本、状态或测量解析器。 / Dispatches a reassembled message to text, status or measurement handling. |
| <code>BleTextReceiver.callback(self, fallback: str)</code> | 处理订阅回调送来的通知字节，交给对应通道的接收器。 / Handles received notification bytes and dispatches them to their channel receiver. |
| <code>BleTextReceiver.callback.receive(_sender: object, payload: bytearray) → None</code>（内部 / local） | 闭包回调，接收本次订阅的字节并保留通道／时间信息。 / Receives subscribed bytes in a closure that retains channel/timing context. |
| <code>BleTextReceiver.on_disconnect(self, client) → None</code> | 记录断连事件并唤醒等待中的接收流程。 / Records a disconnect and wakes the waiting receiver flow. |
| <code>BleTextReceiver.stats_line(self) → str</code> | 汇总链路的字节、完整帧、丢弃和解析计数。 / Reports link bytes, complete frames, drops and parser counters. |
| <code>BleTextReceiver.classify_error(self, error: Exception, sidecar: SerialSidecar) → str</code> | 把收到的异常或日志行归入具体故障类别。 / Classifies a received exception or log line into its specific fault category. |
| <code>write_command(client, receiver: BleTextReceiver, command: str) → None</code> | 按通道封装并发送一次控制命令。 / Formats and submits one control command on the selected channel. |
| <code>interactive_loop(client, receiver: BleTextReceiver, stop_event: asyncio.Event) → None</code> | 运行交互命令输入循环或周期输出链路统计。 / Runs interactive command input or periodically reports link statistics. |
| <code>stats_loop(receiver: BleTextReceiver, stop_event: asyncio.Event) → None</code> | 运行交互命令输入循环或周期输出链路统计。 / Runs interactive command input or periodically reports link statistics. |
| <code>run(args: argparse.Namespace) → int</code> | 在本对象定义的输入、时限与恢复约束下执行本节所述流程。 / Executes the operation described in this section under the object's input, deadline and recovery constraints. |
| <code>parse_args() → argparse.Namespace</code> | 解析命令行参数或选择的阶段。 / Parses CLI arguments or the selected phases. |
| <code>main() → int</code> | 解析命令行，执行本模块的操作并按结果退出。 / Parses CLI arguments, runs the module operation and reports the resulting exit status. |

<a id="receive_serial_text"></a>

### receive_serial_text

按指定端口接收 ASCII 流，执行显式命令并用同一解析器统计帧、错误和速率。 / Receives ASCII on a selected port, sends explicit commands and counts frames, errors and rates with the shared parser. [源码 / Source](receive_serial_text.py)

命令 / CLI　`python3 tools/receive_serial_text.py --help`。参数 / Arguments　`--port`、`--baud`、`--duration`、`--startup-wait`、`--set-rows`、`--rows`、`--stream`、`--tx`、`--fpscap`、`--outcap`、`--adsgap`、`--command`、`--rowmodes`、`--require-rowmodes`、`--show-cap`、`--show-log`。

| 函数、类型与形参 / Callable, type and parameters | 中文用途 / English behavior |
| --- | --- |
| <code>parse_rowmodes(value: str) → list[str]</code> | 校验并解析模式或逐行 profile 的字符串表示。 / Validates and parses a mode list or per-row profile string. |
| <code>decode_ascii_line(raw: bytes, protocol: TextProtocolParser) → str &#124; None</code> | 查找非 ASCII 字节位置或解码一行，并保留错误证据。 / Locates a non-ASCII byte or decodes a line while retaining evidence of encoding errors. |
| <code>write_control(connection: object, command: bytes) → bool</code> | 按通道封装并发送一次控制命令。 / Formats and submits one control command on the selected channel. |
| <code>main() → int</code> | 解析命令行，执行本模块的操作并按结果退出。 / Parses CLI arguments, runs the module operation and reports the resulting exit status. |

<a id="receive_wifi_text"></a>

### receive_wifi_text

绑定 UDP 地址／端口，在选定期间接收并解析文本数据报。 / Binds a UDP address and port and decodes received text datagrams for a bounded interval. [源码 / Source](receive_wifi_text.py)

命令 / CLI　`python3 tools/receive_wifi_text.py --help`。参数 / Arguments　`--device`、`--host`、`--ip`、`--data-port`、`--log-port`、`--ctrl-port`、`--duration`、`--set-rows`、`--rows`、`--stream`、`--tx`、`--command`、`--show-cap`、`--show-log`。

| 函数、类型与形参 / Callable, type and parameters | 中文用途 / English behavior |
| --- | --- |
| <code>first_non_ascii(payload: bytes) → tuple[int, int] &#124; None</code> | 查找非 ASCII 字节位置或解码一行，并保留错误证据。 / Locates a non-ASCII byte or decodes a line while retaining evidence of encoding errors. |
| <code>receiver(port: int) → socket.socket</code> | 运行有界 UDP 接收器并解析到达的数据报。 / Runs the bounded UDP receiver and decodes arriving datagrams. |
| <code>main() → int</code> | 解析命令行，执行本模块的操作并按结果退出。 / Parses CLI arguments, runs the module operation and reports the resulting exit status. |
| <code>main.process_message(ch: str, payload: bytes) → None</code>（内部 / local） | 将重组后的消息分发给文本、状态或测量解析器。 / Dispatches a reassembled message to text, status or measurement handling. |

<a id="reference_fixture"></a>

### reference_fixture

读取 10 kΩ／33 pF 夹具定义和完整 wire frame，分别评估电阻、电容、悬空状态和科学无效值。 / Loads the 10 kΩ / 33 pF layout and full wire frames. Evaluates resistors, capacitors, floating cells and scientific invalid values separately. [源码 / Source](reference_fixture.py)

命令 / CLI　`python3 tools/reference_fixture.py --help`。参数 / Arguments　`--fixture`、`--input`、`--baseline-input`、`--provenance`、`--output`。

| 函数、类型与形参 / Callable, type and parameters | 中文用途 / English behavior |
| --- | --- |
| <code>load_fixture(path: Path) → dict</code> | 读取或描述夹具参考值、坐标和评估基准。 / Loads or describes fixture values, coordinates and evaluation criteria. |
| <code>describe_fixture(fixture: dict) → dict</code> | 读取或描述夹具参考值、坐标和评估基准。 / Loads or describes fixture values, coordinates and evaluation criteria. |
| <code>summary(values: list[float]) → dict</code> | 接收分片或返回本解析器的分通道统计和摘要。 / Consumes fragments or returns the parser’s per-channel statistics and summary. |
| <code>CellSamples(observed: int = 0, invalid: int = 0, stale: int = 0, unacquired: int = 0, masks_unknown: int = 0, rejected_crc: int = 0, rejected_identity: int = 0, inconsistent: int = 0, unit_errors: int = 0, open_samples: int = 0, unexpected_res_values: int = 0, reasons: Counter = field(default_factory=Counter), values: list[float] = field(default_factory=list))</code> | 保存按方法、坐标或工况汇总的输入或结果。 / Holds inputs or results grouped by method, cell coordinate or test stage. |
| <code>CellSamples.output(self) → dict</code> | 返回适合序列化的字段、样本统计或比对结果。 / Returns serializable fields, sample statistics or comparison results. |
| <code>CellSamples.acquisition_errors(self) → int</code> | 对未采集、无效和硬件错误单元计数。 / Counts unavailable acquisitions, invalid cells and hardware errors. |
| <code>CaptureGroup(identity: dict, identity_status: str, frame_type: str, rows: int, profile: str, frames: int = 0, duplicate_or_old_frames: int = 0, sequence_gaps: int = 0, missing_sequences: int = 0, timestamp_regressions: int = 0, first_sequence: int &#124; None = None, last_sequence: int &#124; None = None, first_timestamp_us: int &#124; None = None, last_timestamp_us: int &#124; None = None, cells: dict[tuple[str, str], CellSamples] = field(default_factory=dict))</code> | 汇总同配置的完整帧、收帧时间和有效样本。 / Aggregates complete frames, receive times and valid samples for one configuration. |
| <code>CaptureGroup.cell(self, label: str, mode: str) → CellSamples</code> | 由行列获得单元索引、标签或样本对象。 / Maps row/column coordinates to an index, label or cell sample object. |
| <code>CaptureGroup.add_frame(self, frame) → bool</code> | 把当前记录或完整帧纳入本对象维护的样本／计数集合。 / Adds the current observation or complete frame to the object's sample or counter set. |
| <code>CaptureGroup.duration(self) → float</code> | 返回该身份窗口内有观测的时长。 / Returns the observed duration within this identity window. |
| <code>Capture(groups: list[CaptureGroup], counters: dict, truncated_frames: int = 0, malformed_frames: int = 0, identity_errors: int = 0, non_ascii_lines: int = 0, source: dict = field(default_factory=dict), fault_events: list[str] = field(default_factory=list), fault_event_count: int = 0, memory_diagnostic_lines: int = 0)</code> | 保存按方法、坐标或工况汇总的输入或结果。 / Holds inputs or results grouped by method, cell coordinate or test stage. |
| <code>_identity(header: dict) → tuple[dict, str]</code> | 提取或比较同一 boot 内的配置身份字段。 / Extracts or compares configuration identity fields within one boot. |
| <code>_trailer_matches(header: dict, trailer: dict) → bool</code> | 核对尾行身份与帧头及完整掩码的一致性。 / Checks trailer identity and completeness against the frame header and masks. |
| <code>_observations(frame, header: dict, raw_rows: dict)</code> | 提取帧中的夹具点观测与有效性。 / Extracts fixture cell observations and validity from frames. |
| <code>analyse_capture(lines: Iterable[str], source: dict &#124; None=None) → Capture</code> | 解析输入帧并按配置身份汇总采集窗口。 / Parses the input stream and groups capture observations by configuration identity. |
| <code>read_capture(path: Path) → Capture</code> | 解析输入帧并按配置身份汇总采集窗口。 / Parses the input stream and groups capture observations by configuration identity. |
| <code>_point_status(point: CellSamples &#124; None, identity_status: str, floating=False) → str</code> | 按夹具坐标、原始有效性和参考统计给出测点判定。 / Classifies fixture observations using their coordinates, original validity and reference statistics. |
| <code>_reference_result(label: str, ref: dict, group: CaptureGroup, baseline: Capture &#124; None) → dict</code> | 按夹具坐标、原始有效性和参考统计给出测点判定。 / Classifies fixture observations using their coordinates, original validity and reference statistics. |
| <code>evaluate_fixture(fixture: dict, capture: Capture &#124; None=None, baseline: Capture &#124; None=None, provenance: str=&#x27;unknown&#x27;) → dict</code> | 按夹具坐标、原始有效性和参考统计给出测点判定。 / Classifies fixture observations using their coordinates, original validity and reference statistics. |
| <code>main() → int</code> | 解析命令行，执行本模块的操作并按结果退出。 / Parses CLI arguments, runs the module operation and reports the resulting exit status. |

<a id="run_firmware_benchmark"></a>

### run_firmware_benchmark

执行单工况、32 工况或 VOLT1 稳定性采集；采样窗口、诊断查询和恢复均有时限，原始数据保留在输出目录。 / Executes individual cases, a 32-case suite or a VOLT1 stability run with bounded windows, diagnostic queries and recovery. Writes raw evidence per case. [源码 / Source](run_firmware_benchmark.py)

命令 / CLI　`python3 tools/run_firmware_benchmark.py --help`。参数 / Arguments　`--output-directory`、`--label`、`--mode`、`--rows`、`--profile`、`--suite`、`--stability`、`--capture-seconds`、`--rail-avdd-uv`、`--rail-avss-uv`、`--rail-source-note`、`--expected-elf`、`--expected-session`、`--expected-boot-id`、`--transport`、`--usb-vid`、`--usb-pid`、`--usb-serial`。

| 函数、类型与形参 / Callable, type and parameters | 中文用途 / English behavior |
| --- | --- |
| <code>BenchmarkOptions(port: str, output_directory: Path, label: str, mode: str = &#x27;VOLT&#x27;, rows: int = 1, profile: str = &#x27;CVRCVRCV&#x27;, suite: bool = False, stability: bool = False, capture_seconds: float = 120.0, stability_seconds: float = 600.0, warmup_seconds: float = 5.0, prepare_seconds: float = 45.0, cleanup_seconds: float = 10.0, command_timeout: float = 4.0, progress_seconds: float = 20.0, rail_avdd_uv: int &#124; None = None, rail_avss_uv: int &#124; None = None, rail_source_note: str &#124; None = None, transport: str = &#x27;posix&#x27;, usb_vid: int = 12346, usb_pid: int &#124; None = None, usb_serial: str &#124; None = None, expected_elf_sha256: str &#124; None = None, expected_session: str &#124; None = None, expected_boot_id: int &#124; None = None)</code> | 存放一次调用的工况、接口、期限或 profile；字段在下表列出。 / Stores case, interface, deadline or profile options for one invocation. Fields are listed below. |
| <code>Configuration(label: str, mode: str, rows: int, profile: str, seconds: float, kind: str = &#x27;configuration&#x27;)</code> | 存放一次调用的工况、接口、期限或 profile；字段在下表列出。 / Stores case, interface, deadline or profile options for one invocation. Fields are listed below. |
| <code>configurations(options)</code> | 对命令行要求展开或辨识具体的模式、行数和 profile。 / Expands or identifies requested measurement modes, row counts and profiles. |
| <code>expected_wire_mode(configuration)</code> | 从工况确定预期 wire 类型与行数，或列出实际帧偏离这些条件的字段。 / Derives the expected wire type and row contract, or reports fields in an observed frame that depart from it. |
| <code>record_paths(directory, label, *, bulk=False)</code> | 根据目录、标签和数据类型构造或解析输出文件路径。 / Builds or resolves output file paths by directory, label and record type. |
| <code>HashingRaw</code> | 在写入时累计原始字节数与 SHA-256。 / Tracks byte count and SHA-256 as original bytes are written. |
| <code>HashingRaw.__init__(self, stream)</code> | 使用所列参数构造本对象，初始化其计数、状态或 I/O 句柄。 / Constructs the object from the listed arguments and initializes its counters, state or I/O handles. |
| <code>HashingRaw.write(self, data)</code> | 提交指定字节或文字，按对象的约定保存接收／写入进度。 / Submits the given bytes or text and records progress according to the object's stream contract. |
| <code>HashingRaw.flush(self)</code> | 将缓冲的日志或原始字节推送到对应输出流。 / Flushes buffered journal or original byte data to its output stream. |
| <code>ObservedLines</code> | 在追加协议行时执行观测回调或记录枚举世代。 / Invokes an observation callback or records the enumeration epoch as protocol lines are appended. |
| <code>ObservedLines.__init__(self, observe)</code> | 使用所列参数构造本对象，初始化其计数、状态或 I/O 句柄。 / Constructs the object from the listed arguments and initializes its counters, state or I/O handles. |
| <code>ObservedLines.append(self, line)</code> | 把当前记录或完整帧纳入本对象维护的样本／计数集合。 / Adds the current observation or complete frame to the object's sample or counter set. |
| <code>BenchmarkCollector</code> | 实现本节的采集阶段、数据／命令观察和有界状态验证。 / Implements the module's capture phases, data/command observation and bounded state checks. |
| <code>BenchmarkCollector.__init__(self, settings, configuration, port, raw, events, *, carry=b&#x27;&#x27;, original_usb=None, ctrl_raw=None, ctrl_carry=b&#x27;&#x27;)</code> | 使用所列参数构造本对象，初始化其计数、状态或 I/O 句柄。 / Constructs the object from the listed arguments and initializes its counters, state or I/O handles. |
| <code>BenchmarkCollector.event(self, kind, **fields)</code> | 附加带时间和来源的事件或诊断输出。 / Appends timestamped event or diagnostic output with its source. |
| <code>BenchmarkCollector.send(self, command, deadline)</code> | 按通道封装并发送一次控制命令。 / Formats and submits one control command on the selected channel. |
| <code>BenchmarkCollector.verify_mutation_authority(self, deadline, *, cleanup=False)</code> | 对于已绑定的正式采集，先撤销旧授权，再按 CONFIG?、BUILD?、CONFIG? 顺序核对 ELF、session、boot 和配置边界。准备和清理分别留痕；清理允许设备尚未处于运行态。 / For a bound capture, revokes earlier authority and queries CONFIG?, BUILD?, CONFIG? in order to verify ELF, session, boot and configuration. Records separate preparation and cleanup evidence. Cleanup permits a non-running state. |
| <code>BenchmarkCollector.image_authority_result(self)</code> | 核实或返回当前镜像与 boot 的写操作权限。 / Verifies or reports mutation authority for the current image and boot. |
| <code>BenchmarkCollector.progress(self)</code> | 记录当前采集相位与进度。 / Records the current capture phase and progress. |
| <code>BenchmarkCollector.observe(self, line)</code> | 处理当前观测，保存本对象要求的字段、身份和时间。 / Consumes an observation and records the fields, identity and timing required by this object. |
| <code>BenchmarkCollector.check_fault(self, baseline=0)</code> | 检测新的故障、重启或回调异常，及时终止有问题的阶段。 / Checks for new faults, resets or callback exceptions and stops the affected phase. |
| <code>BenchmarkCollector.query(self, command, tag, phase_deadline)</code> | 发送查询并匹配其回复，遵守本采集器的截止时间和重试规则。 / Sends a query and matches its reply under the collector’s deadline and retry rules. |
| <code>BenchmarkCollector.transition(self, kind, value, phase_deadline=None)</code> | 发起配置切换，跟踪请求及应用终态。 / Requests a configuration change and tracks its request and terminal state. |
| <code>BenchmarkCollector.verify_cap(self, deadline)</code> | 查询并确认恢复后的 CAP 配置。 / Queries and verifies the recovered CAP configuration. |
| <code>BenchmarkCollector.rail_valid(rail)</code> | 验证外部 rail 校准；需要时使用调用方给出的有来源的临时值。 / Checks external rail calibration and, when needed, uses the caller’s sourced volatile measurements. |
| <code>BenchmarkCollector.ensure_rail(self)</code> | 先读 RAIL?。已有外部校准时核对提供的 AVDD/AVSS；需要临时校准时先确认 CAP，然后仅提交一次 RAILCFG，按请求 ID 收齐应答与 applied，复查 RAIL? 并留下来源。 / Queries RAIL? first. Checks any supplied AVDD/AVSS against a valid existing external calibration. When volatile calibration is needed, confirms CAP, submits RAILCFG once, matches its accepted/applied ID and rereads RAIL? with provenance. |
| <code>BenchmarkCollector.snapshot_window(self, name, deadline)</code> | 在有界窗口内采集前后状态及诊断查询。 / Captures state and diagnostic queries within a bounded observation window. |
| <code>BenchmarkCollector.interval(self, seconds, phase, deadline, *, fault_check=True)</code> | 在指定时间内继续读取并维护采集器状态。 / Continues reading and servicing the collector for the given interval. |
| <code>BenchmarkCollector.run(self)</code> | 在本对象定义的输入、时限与恢复约束下执行本节所述流程。 / Executes the operation described in this section under the object's input, deadline and recovery constraints. |
| <code>run_benchmark(options, *, port_factory=None)</code> | 打开所选的采集器，在有界阶段内取证并关闭原始文件。 / Opens the selected collector, captures within bounded phases and closes the original evidence files. |
| <code>main(argv=None, *, require_image_identity=False)</code> | 解析命令行，执行本模块的操作并按结果退出。 / Parses CLI arguments, runs the module operation and reports the resulting exit status. |

<a id="run_firmware_benchmark_retry"></a>

### run_firmware_benchmark_retry

复用基本采集器，校验只读回复，最多三次尝试共享原阶段截止时间；每个写命令只提交一次。 / Extends the base collector with validated read-only replies and up to three attempts under the original phase deadline. Submits each setter once. [源码 / Source](run_firmware_benchmark_retry.py)

命令 / CLI　`python3 tools/run_firmware_benchmark_retry.py --help`。参数 / Arguments　`--output-directory`、`--label`、`--mode`、`--rows`、`--profile`、`--suite`、`--stability`、`--capture-seconds`、`--rail-avdd-uv`、`--rail-avss-uv`、`--rail-source-note`、`--expected-elf`、`--expected-session`、`--expected-boot-id`、`--transport`、`--usb-vid`、`--usb-pid`、`--usb-serial`。

| 函数、类型与形参 / Callable, type and parameters | 中文用途 / English behavior |
| --- | --- |
| <code>InvalidQueryReply</code> | 用于调用方区分采集／身份／传输失败或环境跳过的异常类型。 / Exception type for distinguishing capture/identity/transport failure or an unavailable test environment. |
| <code>validate_reply(command, fields)</code> | 核对查询或回复中的必需字段、范围及能力语义。 / Checks required reply fields, ranges and capability semantics. |
| <code>RetryBenchmarkCollector</code> | 实现本节的采集阶段、数据／命令观察和有界状态验证。 / Implements the module's capture phases, data/command observation and bounded state checks. |
| <code>RetryBenchmarkCollector.__init__(self, *args, **kwargs)</code> | 使用所列参数构造本对象，初始化其计数、状态或 I/O 句柄。 / Constructs the object from the listed arguments and initializes its counters, state or I/O handles. |
| <code>RetryBenchmarkCollector.query(self, command, tag, phase_deadline)</code> | 发送查询并匹配其回复，遵守本采集器的截止时间和重试规则。 / Sends a query and matches its reply under the collector’s deadline and retry rules. |
| <code>RetryBenchmarkCollector.run(self)</code> | 在本对象定义的输入、时限与恢复约束下执行本节所述流程。 / Executes the operation described in this section under the object's input, deadline and recovery constraints. |
| <code>retry_collector_selected()</code> | 在上下文内临时选择采集器实现，离开时恢复原配置。 / Selects a collector implementation for the context and restores the prior selection on exit. |
| <code>run_benchmark(options, **kwargs)</code> | 打开所选的采集器，在有界阶段内取证并关闭原始文件。 / Opens the selected collector, captures within bounded phases and closes the original evidence files. |
| <code>main(argv=None)</code> | 解析命令行，执行本模块的操作并按结果退出。 / Parses CLI arguments, runs the module operation and reports the resulting exit status. |

<a id="sensorarray_bulk_diagnostics"></a>

### sensorarray_bulk_diagnostics

校验正式 USB、CTRL、debug 输入的元数据和字节流，统计独立诊断源及缺失字段。 / Checks stream metadata and bytes for formal USB, CTRL and debug, then reports each diagnostic source and unavailable fields. [源码 / Source](sensorarray_bulk_diagnostics.py)

库入口 / Library　`import sensorarray_bulk_diagnostics`。

| 函数、类型与形参 / Callable, type and parameters | 中文用途 / English behavior |
| --- | --- |
| <code>number(value)</code> | 读取并检查数值字段，保留该调用的范围及未知值规则。 / Reads a numeric field subject to the caller's range and unknown-value rules. |
| <code>finite(value)</code> | 检查有限数值，避免将缺失或非有限速率用于统计。 / Checks finite numbers so missing or non-finite rates stay out of statistics. |
| <code>existing(path)</code> | 根据原始位置或同目录的无损压缩文件解析实际证据路径。 / Resolves the actual evidence path, including a colocated losslessly compressed file when supported. |
| <code>open_raw(path)</code> | 读取指定输入并按格式保留原始字节、行或哈希。 / Reads the specified input, preserving its original bytes, lines or digest as required by the format. |
| <code>record_error(tag, line, fields, memory_validator, expected_backend=&#x27;otg_bulk_v1&#x27;)</code> | 返回诊断记录的字段错误以供计数或排除。 / Returns a diagnostic field error for counting or exclusion. |
| <code>Bucket</code> | 按诊断标签聚合样本和错误。 / Groups diagnostic samples and errors by tag. |
| <code>Bucket.__init__(self)</code> | 使用所列参数构造本对象，初始化其计数、状态或 I/O 句柄。 / Constructs the object from the listed arguments and initializes its counters, state or I/O handles. |
| <code>Bucket.add(self, item)</code> | 把当前记录或完整帧纳入本对象维护的样本／计数集合。 / Adds the current observation or complete frame to the object's sample or counter set. |
| <code>Bucket.report(self, keys=(), continuity=False, only_capture=False)</code> | 返回适合序列化的字段、样本统计或比对结果。 / Returns serializable fields, sample statistics or comparison results. |
| <code>read_stream(raw_path, events_path, expected_sha, *, event_type, origin, bounds, source, memory_validator, expected_backend=&#x27;otg_bulk_v1&#x27;)</code> | 读取指定输入并按格式保留原始字节、行或哈希。 / Reads the specified input, preserving its original bytes, lines or digest as required by the format. |
| <code>public_stream(stream)</code> | 将内部接收流状态转为可写入报告的记录。 / Converts internal receive-stream state into a reportable record. |
| <code>analyse_bulk_diagnostics(directory, label, summary, data_boots, window, debug_summaries, memory_validator, perf_counters)</code> | 按接收流、设备身份和时间窗口关联 USB 调试证据。 / Correlates USB diagnostic evidence by receive stream, device identity and time window. |

<a id="sensorarray_capture_identity"></a>

### sensorarray_capture_identity

在变更模式前将 BUILD 和 CONFIG 的 ELF、session、boot 与调用方提供的身份绑定；拒绝过期的变更权限。 / Binds BUILD and CONFIG evidence to the supplied ELF, session and boot before any configuration mutation. Revokes stale authority. [源码 / Source](sensorarray_capture_identity.py)

库入口 / Library　`import sensorarray_capture_identity`。

| 函数、类型与形参 / Callable, type and parameters | 中文用途 / English behavior |
| --- | --- |
| <code>CaptureIdentityError</code> | 用于调用方区分采集／身份／传输失败或环境跳过的异常类型。 / Exception type for distinguishing capture/identity/transport failure or an unavailable test environment. |
| <code>u32(value, name)</code> | 读取并检查数值字段，保留该调用的范围及未知值规则。 / Reads a numeric field subject to the caller's range and unknown-value rules. |
| <code>ExpectedCaptureIdentity(elf_sha256: str, session: str, boot_id: int)</code> | 保存预期 ELF/session/boot 或将其与实际设备身份绑定后发放变更许可。 / Holds the expected ELF/session/boot or authorizes mutations after binding it to the observed device. |
| <code>ExpectedCaptureIdentity.__post_init__(self)</code> | 检查或初始化构造参数所派生的字段。 / Checks or initializes fields derived from the constructor arguments. |
| <code>ExpectedCaptureIdentity.output(self)</code> | 返回适合序列化的字段、样本统计或比对结果。 / Returns serializable fields, sample statistics or comparison results. |
| <code>expected_from_options(elf_sha256=None, session=None, boot_id=None)</code> | 从命令行选项构建预期的 ELF、session 和 boot 身份。 / Builds the expected ELF, session and boot identity from CLI options. |
| <code>readonly_command(command)</code> | 判定命令是否属于允许的只读查询集合。 / Checks whether a command belongs to the allowed read-only query set. |
| <code>validate_config(expected, config, *, require_running)</code> | 检查 BUILD 与前后 CONFIG 的 boot、revision 和身份一致性。 / Checks boot, revision and identity consistency across BUILD and bracketing CONFIG replies. |
| <code>verify_identity_evidence(expected, build, opening, closing, *, require_running=True)</code> | 检查 BUILD 与前后 CONFIG 的 boot、revision 和身份一致性。 / Checks boot, revision and identity consistency across BUILD and bracketing CONFIG replies. |
| <code>CaptureImageAuthority</code> | 保存预期 ELF/session/boot 或将其与实际设备身份绑定后发放变更许可。 / Holds the expected ELF/session/boot or authorizes mutations after binding it to the observed device. |
| <code>CaptureImageAuthority.__init__(self, expected)</code> | 使用所列参数构造本对象，初始化其计数、状态或 I/O 句柄。 / Constructs the object from the listed arguments and initializes its counters, state or I/O handles. |
| <code>CaptureImageAuthority.revoke(self)</code> | 撤销之前取得的变更权限。 / Revokes previously established mutation authority. |
| <code>CaptureImageAuthority.verify(self, build, opening, closing, *, require_running=True)</code> | 检查 BUILD 与前后 CONFIG 的 boot、revision 和身份一致性。 / Checks boot, revision and identity consistency across BUILD and bracketing CONFIG replies. |
| <code>CaptureImageAuthority.require_command(self, command)</code> | 核实或返回当前镜像与 boot 的写操作权限。 / Verifies or reports mutation authority for the current image and boot. |

<a id="sensorarray_hil"></a>

### sensorarray_hil

串口和 BLE 的可组合 HIL 入口，覆盖订阅、控制、切换、长跑、遥测和重连；保留每阶段记录与失败原因。 / Provides serial and BLE HIL phases for subscriptions, control, mode changes, sustained runs, telemetry and reconnection with per-phase evidence and failure reasons. [源码 / Source](sensorarray_hil.py)

命令 / CLI　`python3 tools/sensorarray_hil.py --help`。参数 / Arguments　`--profile`、`--output-directory`、`--startup-timeout`、`--command-timeout`、`--mode-timeout`、`--frame-timeout`、`--query-interval`、`--minimum-log-stack-bytes`、`--minimum-other-stack-bytes`、`--heap-leak-tolerance-bytes`、`--rail-avdd-uv`、`--rail-avss-uv`、`--known-resistor`、`--port`、`--baud`、`--query`、`--modes`、`--mode-cycles`、`--frames-per-mode`、`--long-run-mode`、`--long-run-frames`、`--long-run-seconds`、`--long-run-maximum-seconds`、`--serial-port`、`--serial-baud`、`--address`、`--name-prefix`、`--scan-seconds`、`--phases`、`--subscription-cases`。

| 函数、类型与形参 / Callable, type and parameters | 中文用途 / English behavior |
| --- | --- |
| <code>normalizeUuid(value: str) → str</code> | 将 UUID、USB 序列号或 TX 模式规范为该后端匹配所用的形式。 / Normalizes a UUID, serial number or TX mode into the form its backend compares. |
| <code>HilFailure</code> | 用于调用方区分采集／身份／传输失败或环境跳过的异常类型。 / Exception type for distinguishing capture/identity/transport failure or an unavailable test environment. |
| <code>HilSkipped</code> | 用于调用方区分采集／身份／传输失败或环境跳过的异常类型。 / Exception type for distinguishing capture/identity/transport failure or an unavailable test environment. |
| <code>serialMissingEmissions(previousSequence: int, currentSequence: int) → int</code> | 从串口观察数据估算缺少的发帧数。 / Derives missing serial emissions from the available observations. |
| <code>SerialCandidate(device: str, description: str, hardwareId: str)</code> | 表示一个夹具参考电阻或一个枚举得到的串口候选。 / Represents one fixture resistor or one enumerated serial-port candidate. |
| <code>FaultEvent(kind: str, source: str, line: str, observedAt: float)</code> | 保存收到的行／帧、故障原因或时间元数据。 / Holds received line/frame data, a classified fault or timing metadata. |
| <code>LineRecord(observedAt: float, channel: str, line: str)</code> | 保存收到的行／帧、故障原因或时间元数据。 / Holds received line/frame data, a classified fault or timing metadata. |
| <code>FrameRecord(observedAt: float, channel: str, frame: Any)</code> | 保存收到的行／帧、故障原因或时间元数据。 / Holds received line/frame data, a classified fault or timing metadata. |
| <code>KnownResistance(row: int, drive: int, minimumOhms: float, maximumOhms: float)</code> | 表示一个夹具参考电阻或一个枚举得到的串口候选。 / Represents one fixture resistor or one enumerated serial-port candidate. |
| <code>KnownResistance.label(self) → str</code> | 由行列获得单元索引、标签或样本对象。 / Maps row/column coordinates to an index, label or cell sample object. |
| <code>KnownResistance.index(self) → int</code> | 由行列获得单元索引、标签或样本对象。 / Maps row/column coordinates to an index, label or cell sample object. |
| <code>FaultDetector</code> | 有限保留新的故障事件，并记录 arm 边界。 / Keeps bounded evidence of new faults and records the arming boundary. |
| <code>FaultDetector.__init__(self) → None</code> | 使用所列参数构造本对象，初始化其计数、状态或 I/O 句柄。 / Constructs the object from the listed arguments and initializes its counters, state or I/O handles. |
| <code>FaultDetector.classify(line: str) → Optional[str]</code> | 把收到的异常或日志行归入具体故障类别。 / Classifies a received exception or log line into its specific fault category. |
| <code>FaultDetector.arm(self, clearExisting: bool=True) → None</code> | 标记故障监控开始，区分通过边界前后的事件。 / Arms fault monitoring and marks its observation boundary. |
| <code>FaultDetector.observe(self, line: str, source: str) → Optional[FaultEvent]</code> | 处理当前观测，保存本对象要求的字段、身份和时间。 / Consumes an observation and records the fields, identity and timing required by this object. |
| <code>FaultDetector.events(self) → List[FaultEvent]</code> | 列出记录到的故障事件或明确忽略的早期事件。 / Returns observed fault events or the explicitly ignored early events. |
| <code>FaultDetector.ignoredEvents(self) → List[FaultEvent]</code> | 列出记录到的故障事件或明确忽略的早期事件。 / Returns observed fault events or the explicitly ignored early events. |
| <code>FaultDetector.assertHealthy(self) → None</code> | 检测新的故障、重启或回调异常，及时终止有问题的阶段。 / Checks for new faults, resets or callback exceptions and stops the affected phase. |
| <code>HilOutput</code> | 管理一次 HIL 的控制台、wire 及结果文件。 / Manages console, wire and output files for a HIL run. |
| <code>HilOutput.__init__(self, logPath: Optional[Path]) → None</code> | 使用所列参数构造本对象，初始化其计数、状态或 I/O 句柄。 / Constructs the object from the listed arguments and initializes its counters, state or I/O handles. |
| <code>HilOutput.emit(self, line: str) → None</code> | 附加带时间和来源的事件或诊断输出。 / Appends timestamped event or diagnostic output with its source. |
| <code>HilOutput.wire(self, source: str, line: str) → None</code> | 按通道保存接收到的 wire 文本。 / Persists received wire text with its channel. |
| <code>HilOutput.close(self) → None</code> | 关闭本对象拥有的端口、文件、线程或比对存储。 / Closes ports, files, threads or comparison storage owned by the object. |
| <code>enumerateSerialPorts() → List[SerialCandidate]</code> | 枚举、筛选或打印主机上的串口候选与设备元数据。 / Enumerates, selects or prints serial-port candidates and their device metadata. |
| <code>selectSerialPort(explicitPort: Optional[str], environmentPort: Optional[str], candidates: Sequence[SerialCandidate]) → str</code> | 枚举、筛选或打印主机上的串口候选与设备元数据。 / Enumerates, selects or prints serial-port candidates and their device metadata. |
| <code>printSerialCandidates(output: HilOutput, candidates: Sequence[SerialCandidate]) → None</code> | 枚举、筛选或打印主机上的串口候选与设备元数据。 / Enumerates, selects or prints serial-port candidates and their device metadata. |
| <code>_safeField(value: str) → str</code> | 读取字段并为缺失值保留安全的显示形式。 / Extracts a field with a safe representation for missing values. |
| <code>parseKnownResistance(value: str) → KnownResistance</code> | 解析、提供或核查已知电阻的行列位置及预期阻值。 / Parses, supplies or checks known resistor coordinates and expected resistance values. |
| <code>defaultKnownResistances() → List[KnownResistance]</code> | 解析、提供或核查已知电阻的行列位置及预期阻值。 / Parses, supplies or checks known resistor coordinates and expected resistance values. |
| <code>parseModes(value: str) → List[str]</code> | 校验并解析模式或逐行 profile 的字符串表示。 / Validates and parses a mode list or per-row profile string. |
| <code>parseSubscriptionCase(value: str) → Set[str]</code> | 解析或编码 C／D／L 通道订阅组合。 / Parses or formats sets of C/D/L channel subscriptions. |
| <code>parseSubscriptionCases(value: str) → List[Set[str]]</code> | 解析或编码 C／D／L 通道订阅组合。 / Parses or formats sets of C/D/L channel subscriptions. |
| <code>formatSubscriptionCase(channels: Iterable[str]) → str</code> | 解析或编码 C／D／L 通道订阅组合。 / Parses or formats sets of C/D/L channel subscriptions. |
| <code>frameMode(frame: Any) → str</code> | 识别帧模式、就绪及应用边界，依目标模式核对收到的帧。 / Identifies frame mode, readiness and apply boundary to check a received frame against the target mode. |
| <code>isRuntimeReadyFrame(frame: Any) → bool</code> | 识别帧模式、就绪及应用边界，依目标模式核对收到的帧。 / Identifies frame mode, readiness and apply boundary to check a received frame against the target mode. |
| <code>isPreApplyFrame(frame: Any, appliedSequence: Optional[int]) → bool</code> | 识别帧模式、就绪及应用边界，依目标模式核对收到的帧。 / Identifies frame mode, readiness and apply boundary to check a received frame against the target mode. |
| <code>expectedCommandPrefix(command: str) → str</code> | 由请求文本得出匹配回复的前缀。 / Derives the reply prefix to match from the request text. |
| <code>isTransientSnapshotBusy(command: str, line: str) → bool</code> | 识别可在总期限内重试的快照忙回复。 / Recognizes transient busy replies that permit a snapshot retry within the deadline. |
| <code>validateModeFrame(frame: Any, mode: str, requestId: int, generation: Optional[int], appliedSequence: Optional[int]) → None</code> | 识别帧模式、就绪及应用边界，依目标模式核对收到的帧。 / Identifies frame mode, readiness and apply boundary to check a received frame against the target mode. |
| <code>checkKnownResistances(frames: Sequence[Any], ranges: Sequence[KnownResistance]) → Dict[str, float]</code> | 解析、提供或核查已知电阻的行列位置及预期阻值。 / Parses, supplies or checks known resistor coordinates and expected resistance values. |
| <code>_slashIntegers(value: str, expectedCount: int) → List[int]</code> | 解析斜杠分隔的遥测计数组。 / Decodes a slash-delimited group of integer telemetry counters. |
| <code>validateStackTelemetry(records: Sequence[LineRecord], minimumLogBytes: int, minimumOtherBytes: int, heapLeakToleranceBytes: int, required: bool) → Dict[str, Any]</code> | 检查栈余量或固件 BLE 计数器的数据类型和约束。 / Checks stack headroom or firmware BLE counter types and bounds. |
| <code>validateBleFirmwareCounters(records: Sequence[LineRecord], required: bool, allowedMessageDrops: int=0) → Dict[str, Any]</code> | 检查栈余量或固件 BLE 计数器的数据类型和约束。 / Checks stack headroom or firmware BLE counter types and bounds. |
| <code>telemetryWindowRecords(monitor: &#x27;SerialMonitor&#x27;, startIndex: int) → List[LineRecord]</code> | 按相位与时间筛出窗口内遥测。 / Filters telemetry records to the specified phase and time window. |
| <code>SerialMonitor</code> | 管理指定接口的数据接收、解码、控制或验收。 / Manages receipt, decoding, control or validation on the designated interface. |
| <code>SerialMonitor.__init__(self, connection: Any, port: str, output: HilOutput, detector: FaultDetector) → None</code> | 使用所列参数构造本对象，初始化其计数、状态或 I/O 句柄。 / Constructs the object from the listed arguments and initializes its counters, state or I/O handles. |
| <code>SerialMonitor.open(cls, port: str, baud: int, output: HilOutput, detector: FaultDetector) → &#x27;SerialMonitor&#x27;</code> | 打开设备或启动该对象的后台接收任务。 / Opens the device or starts the object’s background receiver task. |
| <code>SerialMonitor.start(self) → None</code> | 打开设备或启动该对象的后台接收任务。 / Opens the device or starts the object’s background receiver task. |
| <code>SerialMonitor.close(self) → None</code> | 关闭本对象拥有的端口、文件、线程或比对存储。 / Closes ports, files, threads or comparison storage owned by the object. |
| <code>SerialMonitor._readerLoop(self) → None</code> | 运行后台接收循环，将原始数据投递给解析器。 / Runs the background receive loop and forwards original data to the parser. |
| <code>SerialMonitor.send(self, command: str) → None</code> | 按通道封装并发送一次控制命令。 / Formats and submits one control command on the selected channel. |
| <code>SerialMonitor.lineCount(self) → int</code> | 返回接收器在当前观测期间累计的行数或完整帧数。 / Returns the number of lines or complete frames observed in the active interval. |
| <code>SerialMonitor.frameCount(self) → int</code> | 返回接收器在当前观测期间累计的行数或完整帧数。 / Returns the number of lines or complete frames observed in the active interval. |
| <code>SerialMonitor.waitLine(self, predicate: Callable[[str], bool], startIndex: int, timeout: float, description: str) → Tuple[LineRecord, int]</code> | 等待接收队列中满足条件的下一行或观测。 / Waits for the next line or matching observation from the receive queue. |
| <code>SerialMonitor.waitFrame(self, predicate: Callable[[Any], bool], startIndex: int, timeout: float, description: str) → Tuple[FrameRecord, int]</code> | 等待符合模式、身份或序号谓词的完整帧。 / Waits for a complete frame satisfying mode, identity or sequence predicates. |
| <code>waitSerialCommand(monitor: SerialMonitor, command: str, timeout: float) → str</code> | 发送并等待目标通路的匹配回复，根据阶段判断设备是否就绪。 / Issues and waits for a matching reply on the indicated transport and evaluates readiness for the phase. |
| <code>waitSerialCommand.matches(line: str) → bool</code>（内部 / local） | 检查当前记录是否满足外层等待的条件。 / Tests whether the current record satisfies the enclosing wait predicate. |
| <code>waitSerialCommandReady(monitor: SerialMonitor, command: str, timeout: float) → str</code> | 发送并等待目标通路的匹配回复，根据阶段判断设备是否就绪。 / Issues and waits for a matching reply on the indicated transport and evaluates readiness for the phase. |
| <code>setSerialRailCalibration(monitor: SerialMonitor, avddUv: int, avssUv: int, timeout: float) → Dict[str, int]</code> | 在相应控制通路提交并核查给定的外部 rail 校准。 / Submits and verifies supplied external rail calibration on the indicated control transport. |
| <code>switchSerialMode(monitor: SerialMonitor, mode: str, frameTarget: int, timeout: float) → Tuple[List[Any], Dict[str, int]]</code> | 通过指定传输请求切换模式并等待与请求匹配的终态。 / Requests a mode change on the indicated transport and waits for its matching terminal. |
| <code>collectSerialLongRun(monitor: SerialMonitor, mode: str, initialEvidence: Dict[str, int], frameTarget: int, secondsTarget: float, maximumSeconds: float, queryInterval: float, frameTimeout: float, knownRanges: Sequence[KnownResistance]) → Dict[str, Any]</code> | 在长时间采集中累计完整帧、缺口、故障与遥测。 / Accumulates complete frames, gaps, faults and telemetry during a sustained acquisition. |
| <code>firstNonAscii(payload: bytes) → Optional[Tuple[int, int]]</code> | 查找非 ASCII 字节位置或解码一行，并保留错误证据。 / Locates a non-ASCII byte or decodes a line while retaining evidence of encoding errors. |
| <code>BleObserver</code> | 管理指定接口的数据接收、解码、控制或验收。 / Manages receipt, decoding, control or validation on the designated interface. |
| <code>BleObserver.__init__(self, output: HilOutput, detector: FaultDetector) → None</code> | 使用所列参数构造本对象，初始化其计数、状态或 I/O 句柄。 / Constructs the object from the listed arguments and initializes its counters, state or I/O handles. |
| <code>BleObserver.resetWindow(self) → None</code> | 重置 BLE 观测窗口，过期清理或统计正在重组的分片。 / Resets a BLE observation window, expires stale fragment assemblies or counts pending ones. |
| <code>BleObserver.callback(self, fallbackChannel: str) → Callable[[Any, bytearray], None]</code> | 处理订阅回调送来的通知字节，交给对应通道的接收器。 / Handles received notification bytes and dispatches them to their channel receiver. |
| <code>BleObserver.callback.receive(_sender: Any, payload: bytearray) → None</code>（内部 / local） | 闭包回调，接收本次订阅的字节并保留通道／时间信息。 / Receives subscribed bytes in a closure that retains channel/timing context. |
| <code>BleObserver.feedNotification(self, fallbackChannel: str, payload: bytes) → None</code> | 处理订阅回调送来的通知字节，交给对应通道的接收器。 / Handles received notification bytes and dispatches them to their channel receiver. |
| <code>BleObserver._processMessage(self, channel: str, message: bytes) → None</code> | 将重组后的消息分发给文本、状态或测量解析器。 / Dispatches a reassembled message to text, status or measurement handling. |
| <code>BleObserver.lineCount(self, channel: str) → int</code> | 返回接收器在当前观测期间累计的行数或完整帧数。 / Returns the number of lines or complete frames observed in the active interval. |
| <code>BleObserver.frameCount(self) → int</code> | 返回接收器在当前观测期间累计的行数或完整帧数。 / Returns the number of lines or complete frames observed in the active interval. |
| <code>BleObserver.waitLine(self, channel: str, predicate: Callable[[str], bool], startIndex: int, timeout: float, description: str) → Tuple[LineRecord, int]</code> | 等待接收队列中满足条件的下一行或观测。 / Waits for the next line or matching observation from the receive queue. |
| <code>BleObserver.waitFrame(self, predicate: Callable[[Any], bool], startIndex: int, timeout: float, description: str) → Tuple[FrameRecord, int]</code> | 等待符合模式、身份或序号谓词的完整帧。 / Waits for a complete frame satisfying mode, identity or sequence predicates. |
| <code>BleObserver.expireStaleFragments(self, maximumAge: float) → None</code> | 重置 BLE 观测窗口，过期清理或统计正在重组的分片。 / Resets a BLE observation window, expires stale fragment assemblies or counts pending ones. |
| <code>BleObserver.pendingFragments(self) → int</code> | 重置 BLE 观测窗口，过期清理或统计正在重组的分片。 / Resets a BLE observation window, expires stale fragment assemblies or counts pending ones. |
| <code>BleObserver.raiseCallbackError(self) → None</code> | 检测新的故障、重启或回调异常，及时终止有问题的阶段。 / Checks for new faults, resets or callback exceptions and stops the affected phase. |
| <code>BleObserver.wireStats(self) → Dict[str, Any]</code> | 汇总链路的字节、完整帧、丢弃和解析计数。 / Reports link bytes, complete frames, drops and parser counters. |
| <code>validateBleWindow(observer: BleObserver, strictSequence: bool=True, maximumStaleAssemblies: int=0) → Dict[str, Any]</code> | 校验 BLE 窗口计数或给出高负载下不完整分片的允许预算。 / Checks BLE window counters or derives the incomplete-assembly allowance for a high-load window. |
| <code>fastHighLoadIncompleteAssemblyBudget(completedFrames: int) → int</code> | 校验 BLE 窗口计数或给出高负载下不完整分片的允许预算。 / Checks BLE window counters or derives the incomplete-assembly allowance for a high-load window. |
| <code>requiredGattProperties() → Dict[str, Set[str]]</code> | 确定或核验当前通道要求的 GATT 特征属性。 / Determines or validates GATT properties required by the selected channels. |
| <code>validateGatt(client: Any) → Dict[str, Any]</code> | 确定或核验当前通道要求的 GATT 特征属性。 / Determines or validates GATT properties required by the selected channels. |
| <code>startSubscriptions(client: Any, observer: BleObserver, channels: Set[str], forceIndicate: bool=False) → None</code> | 启停通知、复位观测窗口，或验证退订通道的静默。 / Starts/stops notification subscriptions, resets the observation window or verifies quiescence after unsubscribe. |
| <code>stopSubscriptions(client: Any, channels: Set[str]) → None</code> | 启停通知、复位观测窗口，或验证退订通道的静默。 / Starts/stops notification subscriptions, resets the observation window or verifies quiescence after unsubscribe. |
| <code>resetBleWindowAfterUnsubscribe(observer: BleObserver) → None</code> | 启停通知、复位观测窗口，或验证退订通道的静默。 / Starts/stops notification subscriptions, resets the observation window or verifies quiescence after unsubscribe. |
| <code>assertBleChannelQuietAfterUnsubscribe(client: Any, observer: BleObserver, channel: str, detector: FaultDetector, settleSeconds: float=BLE_UNSUBSCRIBE_SETTLE_SECONDS, quietSeconds: float=BLE_UNSUBSCRIBE_QUIET_SECONDS) → Dict[str, int]</code> | 启停通知、复位观测窗口，或验证退订通道的静默。 / Starts/stops notification subscriptions, resets the observation window or verifies quiescence after unsubscribe. |
| <code>waitBleCommand(client: Any, observer: BleObserver, command: str, timeout: float, writeWithResponse: bool=False) → str</code> | 发送并等待目标通路的匹配回复，根据阶段判断设备是否就绪。 / Issues and waits for a matching reply on the indicated transport and evaluates readiness for the phase. |
| <code>waitBleCommand.matches(line: str) → bool</code>（内部 / local） | 检查当前记录是否满足外层等待的条件。 / Tests whether the current record satisfies the enclosing wait predicate. |
| <code>setBleRailCalibration(client: Any, observer: BleObserver, avddUv: int, avssUv: int, timeout: float, sidecar: Optional[SerialMonitor]=None) → Dict[str, int]</code> | 在相应控制通路提交并核查给定的外部 rail 校准。 / Submits and verifies supplied external rail calibration on the indicated control transport. |
| <code>switchBleMode(client: Any, observer: BleObserver, mode: str, frameTarget: int, timeout: float, logSubscribed: bool, sidecar: Optional[SerialMonitor]=None) → Tuple[List[Any], Dict[str, int]]</code> | 通过指定传输请求切换模式并等待与请求匹配的终态。 / Requests a mode change on the indicated transport and waits for its matching terminal. |
| <code>waitForBleTraffic(client: Any, observer: BleObserver, dataFrames: int, logMessages: int, minimumSeconds: float, timeout: float, detector: FaultDetector, requiredLogTags: Optional[Set[str]]=None) → None</code> | 等待已订阅通道出现预期的完整观测。 / Waits for expected complete observations on subscribed BLE channels. |
| <code>requiredLogTagsForSubscription(profileName: str, channels: Set[str]) → Set[str]</code> | 确定订阅组合要求的日志标签或运行该组合的验收。 / Determines required log tags for a subscription case or executes that case. |
| <code>runSubscriptionCase(client: Any, observer: BleObserver, sidecar: SerialMonitor, channels: Set[str], queryCommands: Sequence[str], dataFrames: int, logMessages: int, minimumSeconds: float, timeout: float, detector: FaultDetector, minimumLogStackBytes: int, minimumOtherStackBytes: int, heapLeakToleranceBytes: int, requireFirmwareTelemetry: bool, allowedMessageDrops: int=3, requiredLogTags: Optional[Set[str]]=None) → Dict[str, Any]</code> | 确定订阅组合要求的日志标签或运行该组合的验收。 / Determines required log tags for a subscription case or executes that case. |
| <code>setSerialRows(monitor: SerialMonitor, rows: int, timeout: float) → Dict[str, str]</code> | 在指定控制通路请求行数并核对实际应用。 / Requests a row count on the selected control path and verifies application. |
| <code>serialRowCountsForProfile(profileName: str) → Tuple[int, ...]</code> | 由配置 profile 生成行数、逐行模式或对应的测试顺序。 / Derives row counts, row modes or their test sequence from the selected profile. |
| <code>serialRowmodesForProfile(profileName: str) → Tuple[str, ...]</code> | 由配置 profile 生成行数、逐行模式或对应的测试顺序。 / Derives row counts, row modes or their test sequence from the selected profile. |
| <code>bleRowmodesForProfile(profileName: str) → Tuple[str, ...]</code> | 由配置 profile 生成行数、逐行模式或对应的测试顺序。 / Derives row counts, row modes or their test sequence from the selected profile. |
| <code>buildSerialRowmodePlan(profileName: str) → List[Dict[str, Any]]</code> | 由配置 profile 生成行数、逐行模式或对应的测试顺序。 / Derives row counts, row modes or their test sequence from the selected profile. |
| <code>validateRowmodesTerminal(ackLine: str, terminalLine: str, profile: str) → Dict[str, int]</code> | 核对 ROWMODES 终态或混合帧的活动行和配置身份。 / Verifies a ROWMODES terminal or the active rows and identity of a mixed frame. |
| <code>validateMixedProfileFrame(frame: Any, profile: str) → None</code> | 核对 ROWMODES 终态或混合帧的活动行和配置身份。 / Verifies a ROWMODES terminal or the active rows and identity of a mixed frame. |
| <code>setSerialRowmodes(monitor: SerialMonitor, profile: str, timeout: float) → Dict[str, int]</code> | 在所选通道应用逐行 C/V/R，并验证行数、终态及后续帧。 / Applies per-row C/V/R on the selected path and checks rows, terminal and subsequent frames. |
| <code>runSerialRowmodesAndRows(monitor: SerialMonitor, args: argparse.Namespace) → List[Dict[str, Any]]</code> | 在所选通道应用逐行 C/V/R，并验证行数、终态及后续帧。 / Applies per-row C/V/R on the selected path and checks rows, terminal and subsequent frames. |
| <code>setBleRows(client: Any, observer: BleObserver, rows: int, timeout: float, sidecar: Optional[SerialMonitor]=None) → Dict[str, str]</code> | 在指定控制通路请求行数并核对实际应用。 / Requests a row count on the selected control path and verifies application. |
| <code>setBleRowmodes(client: Any, observer: BleObserver, profile: str, timeout: float) → Dict[str, int]</code> | 在所选通道应用逐行 C/V/R，并验证行数、终态及后续帧。 / Applies per-row C/V/R on the selected path and checks rows, terminal and subsequent frames. |
| <code>writeSummary(outputDirectory: Optional[Path], name: str, summary: Dict[str, Any]) → None</code> | 保存会话摘要及通过、失败和未知项目。 / Saves the session summary and its pass, fail and unknown items. |
| <code>resolveOutput(args: argparse.Namespace, name: str) → Tuple[HilOutput, Optional[Path]]</code> | 根据目录、标签和数据类型构造或解析输出文件路径。 / Builds or resolves output file paths by directory, label and record type. |
| <code>applySerialProfile(args: argparse.Namespace) → None</code> | 将选定 HIL profile 展开为具体通路参数和测试阶段。 / Expands a selected HIL profile into transport settings and test phases. |
| <code>runSerialHil(args: argparse.Namespace) → int</code> | 选择或运行该传输的诊断、负载、订阅与恢复阶段。 / Selects or executes the transport’s diagnostic, load, subscription and recovery phases. |
| <code>BleProfile(connectedIdleSeconds: float, caseDataFrames: int, ff20OnlyFrames: int, ff20OnlySeconds: float, caseLogMessages: int, caseTimeout: float, txModeFrames: int, modeCycles: int, framesPerMode: int, subscribeCycles: int, reconnectCycles: int, longRunFrames: int, longRunSeconds: float, longRunMaximumSeconds: float)</code> | 存放一次调用的工况、接口、期限或 profile；字段在下表列出。 / Stores case, interface, deadline or profile options for one invocation. Fields are listed below. |
| <code>buildBleProfile(args: argparse.Namespace) → BleProfile</code> | 将选定 HIL profile 展开为具体通路参数和测试阶段。 / Expands a selected HIL profile into transport settings and test phases. |
| <code>discoverBleDevice(args: argparse.Namespace, output: HilOutput, BleakScanner: Any) → Any</code> | 根据 BLE 设备地址／名称发现外设或创建指定适配器的客户端。 / Discovers a BLE peripheral by address/name or creates a client for the selected adapter. |
| <code>createBleClient(BleakClient: Any, device: Any, disconnectedCallback: Optional[Callable[[Any], None]]=None) → Any</code> | 根据 BLE 设备地址／名称发现外设或创建指定适配器的客户端。 / Discovers a BLE peripheral by address/name or creates a client for the selected adapter. |
| <code>runTxModePhase(client: Any, observer: BleObserver, profile: BleProfile, detector: FaultDetector, timeout: float) → Dict[str, Any]</code> | 执行对应的 TX 模式、模式切换、订阅、命令一致性或重连场景。 / Runs the indicated TX mode, switching, subscription, command-parity or reconnect scenario. |
| <code>runModeStressPhase(client: Any, observer: BleObserver, profile: BleProfile, detector: FaultDetector, timeout: float, knownRanges: Sequence[KnownResistance], sidecar: SerialMonitor) → Dict[str, Any]</code> | 执行对应的 TX 模式、模式切换、订阅、命令一致性或重连场景。 / Runs the indicated TX mode, switching, subscription, command-parity or reconnect scenario. |
| <code>runSubscribeStressPhase(client: Any, observer: BleObserver, profile: BleProfile, detector: FaultDetector, sidecar: SerialMonitor, minimumLogStackBytes: int, minimumOtherStackBytes: int, heapLeakToleranceBytes: int, requireTelemetry: bool) → Dict[str, Any]</code> | 执行对应的 TX 模式、模式切换、订阅、命令一致性或重连场景。 / Runs the indicated TX mode, switching, subscription, command-parity or reconnect scenario. |
| <code>runCommandParity(client: Any, observer: BleObserver, sidecar: SerialMonitor, timeout: float) → Dict[str, str]</code> | 执行对应的 TX 模式、模式切换、订阅、命令一致性或重连场景。 / Runs the indicated TX mode, switching, subscription, command-parity or reconnect scenario. |
| <code>runBleRowmodesPhase(client: Any, observer: BleObserver, detector: FaultDetector, profileName: str, commandTimeout: float, modeTimeout: float) → Dict[str, Any]</code> | 在所选通道应用逐行 C/V/R，并验证行数、终态及后续帧。 / Applies per-row C/V/R on the selected path and checks rows, terminal and subsequent frames. |
| <code>runBleLongRunPhase(client: Any, observer: BleObserver, sidecar: SerialMonitor, profile: BleProfile, detector: FaultDetector, commandTimeout: float, frameTimeout: float, queryInterval: float, knownRanges: Sequence[KnownResistance], minimumLogStackBytes: int, minimumOtherStackBytes: int, heapLeakToleranceBytes: int, requireTelemetry: bool) → Dict[str, Any]</code> | 在长时间采集中累计完整帧、缺口、故障与遥测。 / Accumulates complete frames, gaps, faults and telemetry during a sustained acquisition. |
| <code>runReconnectStress(device: Any, BleakClient: Any, output: HilOutput, detector: FaultDetector, profile: BleProfile, commandTimeout: float, restoreStream: str, restoreTx: str, restoreBtx: str, sidecar: SerialMonitor, minimumLogStackBytes: int, minimumOtherStackBytes: int, heapLeakToleranceBytes: int, requireTelemetry: bool) → Dict[str, Any]</code> | 执行对应的 TX 模式、模式切换、订阅、命令一致性或重连场景。 / Runs the indicated TX mode, switching, subscription, command-parity or reconnect scenario. |
| <code>parsePhases(value: str) → Set[str]</code> | 解析命令行参数或选择的阶段。 / Parses CLI arguments or the selected phases. |
| <code>resolveBlePhases(profile: str, phases: Set[str]) → Set[str]</code> | 选择或运行该传输的诊断、负载、订阅与恢复阶段。 / Selects or executes the transport’s diagnostic, load, subscription and recovery phases. |
| <code>runBleHil(args: argparse.Namespace) → int</code> | 选择或运行该传输的诊断、负载、订阅与恢复阶段。 / Selects or executes the transport’s diagnostic, load, subscription and recovery phases. |
| <code>addCommonArguments(parser: argparse.ArgumentParser) → None</code> | 向解析器登记该模块支持的命令行选项。 / Registers the module's command-line options with the supplied parser. |
| <code>buildArgumentParser() → argparse.ArgumentParser</code> | 创建含子命令、选项和缺省值的参数解析器。 / Constructs the argument parser with supported subcommands, options and defaults. |
| <code>validateArguments(parser: argparse.ArgumentParser, args: argparse.Namespace) → None</code> | 检查选项组合、身份和期限，错误时阻止开始采集。 / Checks option combinations, identities and deadlines before starting capture. |
| <code>main(argv: Optional[Sequence[str]]=None) → int</code> | 解析命令行，执行本模块的操作并按结果退出。 / Parses CLI arguments, runs the module operation and reports the resulting exit status. |

<a id="sensorarray_usb_bulk"></a>

### sensorarray_usb_bulk

使用 PyUSB、VID/PID/序列号和接口端点访问 vendor bulk 设备；分开报告 DATA、CTRL 和主机接收。 / Opens a vendor bulk device by VID/PID, serial and interface endpoints using PyUSB. Distinguishes DATA, CTRL and host receipt. [源码 / Source](sensorarray_usb_bulk.py)

库入口 / Library　`import sensorarray_usb_bulk`。

| 函数、类型与形参 / Callable, type and parameters | 中文用途 / English behavior |
| --- | --- |
| <code>UsbBulkError</code> | 用于调用方区分采集／身份／传输失败或环境跳过的异常类型。 / Exception type for distinguishing capture/identity/transport failure or an unavailable test environment. |
| <code>UsbBulkPort</code> | 提供有界端口读写、接口元数据和关闭操作。 / Provides bounded port I/O, interface metadata and closure. |
| <code>UsbBulkPort.__init__(self, serial: str, *, vid=USB_VID, pid=USB_PID, usb_core=None, usb_util=None, clock=time.monotonic)</code> | 使用所列参数构造本对象，初始化其计数、状态或 I/O 句柄。 / Constructs the object from the listed arguments and initializes its counters, state or I/O handles. |
| <code>UsbBulkPort.metadata(self)</code> | 返回端口、设备身份、端点或通路能力信息。 / Returns port, device identity, endpoint or path capability metadata. |
| <code>UsbBulkPort._timeout_ms(self, deadline, maximum=10)</code> | 换算剩余超时、识别驱动超时或检查设备仍处于打开状态。 / Converts remaining timeout, recognizes driver timeouts or checks that the device is still open. |
| <code>UsbBulkPort._is_timeout(error)</code> | 换算剩余超时、识别驱动超时或检查设备仍处于打开状态。 / Converts remaining timeout, recognizes driver timeouts or checks that the device is still open. |
| <code>UsbBulkPort._check_open(self)</code> | 换算剩余超时、识别驱动超时或检查设备仍处于打开状态。 / Converts remaining timeout, recognizes driver timeouts or checks that the device is still open. |
| <code>UsbBulkPort.read_some(self, max_bytes, deadline)</code> | 在指定期限内尝试收发一段字节，返回实际进展。 / Attempts a bounded read or write and returns actual progress. |
| <code>UsbBulkPort.write_some(self, data, deadline)</code> | 在指定期限内尝试收发一段字节，返回实际进展。 / Attempts a bounded read or write and returns actual progress. |
| <code>UsbBulkPort.close(self)</code> | 关闭本对象拥有的端口、文件、线程或比对存储。 / Closes ports, files, threads or comparison storage owned by the object. |

<a id="sensorarray_usb_cdc"></a>

### sensorarray_usb_cdc

从 Linux sysfs 识别正式 CDC 数据角色并打开匹配的 tty，返回部分读写和设备身份元数据。 / Resolves the formal CDC data role to a Linux tty using sysfs and exposes partial reads/writes and identity metadata. [源码 / Source](sensorarray_usb_cdc.py)

库入口 / Library　`import sensorarray_usb_cdc`。

| 函数、类型与形参 / Callable, type and parameters | 中文用途 / English behavior |
| --- | --- |
| <code>UsbCdcError</code> | 用于调用方区分采集／身份／传输失败或环境跳过的异常类型。 / Exception type for distinguishing capture/identity/transport failure or an unavailable test environment. |
| <code>find_cdc_tty(serial, *, vid=USB_VID, pid=USB_PID, interface_number=0, sys_tty=Path(&#x27;/sys/class/tty&#x27;), dev_root=Path(&#x27;/dev&#x27;))</code> | 从 sys_tty 的 ttyACM 节点沿父目录找 USB 设备与接口，要求 12 位十六进制序列号、VID/PID 和 CDC 描述符一致。正式角色为 303a:4042 接口 0，调试角色为接口 2。仅唯一匹配返回带 tty 路径和 sysfs 身份的字典。 / Walks ttyACM ancestors under sys_tty, matching the 12-hex serial, VID/PID and CDC interface descriptor. Formal data is 303a:4042 interface 0, debug is interface 2. Returns tty path and sysfs metadata only on a unique match. |
| <code>find_cdc_tty.read(base, field)</code>（内部 / local） | 读取指定输入并按格式保留原始字节、行或哈希。 / Reads the specified input, preserving its original bytes, lines or digest as required by the format. |
| <code>UsbCdcPort</code> | 提供有界端口读写、接口元数据和关闭操作。 / Provides bounded port I/O, interface metadata and closure. |
| <code>UsbCdcPort.__init__(self, serial, *, vid=USB_VID, pid=USB_PID, metadata_finder=find_cdc_tty, port_factory=None)</code> | 使用所列参数构造本对象，初始化其计数、状态或 I/O 句柄。 / Constructs the object from the listed arguments and initializes its counters, state or I/O handles. |
| <code>UsbCdcPort.metadata(self)</code> | 返回端口、设备身份、端点或通路能力信息。 / Returns port, device identity, endpoint or path capability metadata. |
| <code>UsbCdcPort.read_some(self, max_bytes, deadline)</code> | 在指定期限内尝试收发一段字节，返回实际进展。 / Attempts a bounded read or write and returns actual progress. |
| <code>UsbCdcPort.write_some(self, data, deadline)</code> | 在指定期限内尝试收发一段字节，返回实际进展。 / Attempts a bounded read or write and returns actual progress. |
| <code>UsbCdcPort.close(self)</code> | 关闭本对象拥有的端口、文件、线程或比对存储。 / Closes ports, files, threads or comparison storage owned by the object. |

<a id="summarize_reference_capture"></a>

### summarize_reference_capture

将原始 RX 与事件字节偏移对齐，按同一 boot 和配置分段，汇总夹具点及诊断计数。 / Aligns receive bytes with event offsets and summarizes fixture values and counters within each boot/configuration segment. [源码 / Source](summarize_reference_capture.py)

命令 / CLI　`python3 tools/summarize_reference_capture.py --help`。参数 / Arguments　`--directory`、`--capture`、`--fixture`、`--output`。

| 函数、类型与形参 / Callable, type and parameters | 中文用途 / English behavior |
| --- | --- |
| <code>integer(value)</code> | 读取并检查数值字段，保留该调用的范围及未知值规则。 / Reads a numeric field subject to the caller's range and unknown-value rules. |
| <code>file_evidence(path: Path)</code> | 计算文件或字节的摘要，为结果绑定实际输入。 / Computes a digest for file or byte evidence so results can be traced to the actual input. |
| <code>read_raw(directory: Path, label: str)</code> | 读取指定输入并按格式保留原始字节、行或哈希。 / Reads the specified input, preserving its original bytes, lines or digest as required by the format. |
| <code>RxTimeline</code> | 按接收事件和字节偏移为原始行定位时间。 / Maps offsets in original input to host receive events and times. |
| <code>RxTimeline.__init__(self, events)</code> | 使用所列参数构造本对象，初始化其计数、状态或 I/O 句柄。 / Constructs the object from the listed arguments and initializes its counters, state or I/O handles. |
| <code>RxTimeline.at(self, offset)</code> | 以位置或接收时间在已索引的时间线上查找对应记录。 / Looks up an observation in an indexed receive timeline by offset or time. |
| <code>FrameRecord(lines: list[str], header: dict, trailer: dict, raw_rows: dict, frame: object, line_start: int, line_end: int, byte_start: int, byte_end: int)</code> | 保存收到的行／帧、故障原因或时间元数据。 / Holds received line/frame data, a classified fault or timing metadata. |
| <code>parse_records(payload: bytes, timeline: RxTimeline)</code> | 将捕获字节与接收时间线解码为有来源的观察记录。 / Parses capture bytes and the receive timeline into sourced observations. |
| <code>counter_window(records, keys, same_boot)</code> | 按同一身份的起止观测计算分层计数窗口。 / Derives layer-counter windows from bracketing observations with the same identity. |
| <code>compact_group(group)</code> | 按配置或片段键汇总帧、科学状态、传输计数及未知数据。 / Summarizes frames, scientific status, transport counters and unknowns by configuration or segment key. |
| <code>segment_key(record)</code> | 按配置或片段键汇总帧、科学状态、传输计数及未知数据。 / Summarizes frames, scientific status, transport counters and unknowns by configuration or segment key. |
| <code>s1d1_observation(record, timeline, order_ok)</code> | 提取帧中的夹具点观测与有效性。 / Extracts fixture cell observations and validity from frames. |
| <code>summarize_segment(records, index, fixture, timeline)</code> | 按配置或片段键汇总帧、科学状态、传输计数及未知数据。 / Summarizes frames, scientific status, transport counters and unknowns by configuration or segment key. |
| <code>summarize_capture(directory: Path, label: str, fixture: dict)</code> | 按配置或片段键汇总帧、科学状态、传输计数及未知数据。 / Summarizes frames, scientific status, transport counters and unknowns by configuration or segment key. |
| <code>make_report(directory: Path, labels: list[str], fixture_path: Path)</code> | 将经过核验的字段和各层指标组成报告。 / Assembles a report from verified fields and layer-specific metrics. |
| <code>main()</code> | 解析命令行，执行本模块的操作并按结果退出。 / Parses CLI arguments, runs the module operation and reports the resulting exit status. |

<a id="text_protocol"></a>

### text_protocol

解析 ASCII 行、CAP/VOLT/RES/混合帧以及 BLE 分片，检查行掩码、CRC、序列和协议无效值。 / Parses ASCII lines, CAP/VOLT/RES/mixed frames and BLE fragments, validating masks, CRCs, sequence and explicit invalid values. [源码 / Source](text_protocol.py)

库入口 / Library　`import text_protocol`。

| 函数、类型与形参 / Callable, type and parameters | 中文用途 / English behavior |
| --- | --- |
| <code>parse_fields(line: str) → dict[str, str]</code> | 从第一项标签之后提取首个等号分隔的各键和值。 / Splits comma-delimited fields after the tag at the first equals sign in each field. |
| <code>CapFrame(sequence: int, timestamp_us: int, rows: int, cells: int, values_fixed: list[int], crc_ok: bool, generation: int, request_id: int, row_fresh_mask: int, primary_fresh_mask: int, secondary_fresh_mask: int, expected_mask: int = 0, acquired_mask: int = 0)</code> | 保存完整 CAP、VOLT/RES 或逐行混合数据及其配置与有效性。 / Holds CAP, VOLT/RES or mixed-row data together with configuration and validity. |
| <code>CapFrame.values_pf(self) → list[Optional[float]]</code> | 将 CAP 的整数编码换算为 pF。 / Converts integer-encoded CAP samples to pF. |
| <code>MeasurementFrame(sequence: int, timestamp_us: int, rows: int, cells: int, mode: str, unit: str, scale: int, values_fixed: list[Optional[int]], error_reasons: list[int], pga_gains: list[int], valid_mask: int, fresh_mask: int, error_mask: int, reference: str, rail_valid: bool, generation: int, request_id: int, rail_age_frames: int, frame_duration_us: int, transition_duration_us: int, gain_change_count: int, overrange_count: int, autorange_attempt_count: int, autorange_fallback_count: int, io_retry_count: int, drdy_timeout_count: int, stale_count: int, spi_error_count: int, crc_ok: bool, expected_mask: int = 0, acquired_mask: int = 0)</code> | 保存完整 CAP、VOLT/RES 或逐行混合数据及其配置与有效性。 / Holds CAP, VOLT/RES or mixed-row data together with configuration and validity. |
| <code>MeasurementFrame.values_si(self) → list[Optional[float]]</code> | 按帧 scale 恢复有效电压／电阻的 SI 数值，无效标志对应缺失样本。 / Applies the frame scale to valid voltage/resistance samples and preserves explicit invalid values. |
| <code>MixedRow(row: int, mode: str, unit: str, scale: int, values_fixed: list[Optional[int]], valid_mask: int, fresh_mask: int, error_mask: int, expected_mask: int = 0, acquired_mask: int = 0)</code> | 保存完整 CAP、VOLT/RES 或逐行混合数据及其配置与有效性。 / Holds CAP, VOLT/RES or mixed-row data together with configuration and validity. |
| <code>MixedFrame(sequence: int, timestamp_us: int, rows: int, cells: int, profile: str, generation: int, request_id: int, profile_generation: int, profile_request_id: int, row_frames: dict[int, MixedRow], crc_ok: bool, expected_mask: int = 0, acquired_mask: int = 0)</code> | 保存完整 CAP、VOLT/RES 或逐行混合数据及其配置与有效性。 / Holds CAP, VOLT/RES or mixed-row data together with configuration and validity. |
| <code>ProtocolCounters(lines: int = 0, cap_frames: int = 0, voltage_frames: int = 0, resistance_frames: int = 0, mixed_frames: int = 0, crc_errors: int = 0, malformed: int = 0, sequence_gaps: int = 0, missing_frames: int = 0, sequence_regressions: int = 0, non_ascii_chunks: int = 0, summary_lines: int = 0, rmack: int = 0, rmapp: int = 0, rmerr: int = 0, lifecycle_duplicate_terminal: int = 0, lifecycle_terminal_without_ack: int = 0)</code> | 分级累计协议错误、分片和完整消息。 / Counts protocol errors, fragments and complete messages at their respective layers. |
| <code>FragmentStats(ok: int = 0, missing: int = 0, gap: int = 0, crc_fail: int = 0, dropped: int = 0, tiny: int = 0, duplicate: int = 0, out_of_order: int = 0)</code> | 分级累计协议错误、分片和完整消息。 / Counts protocol errors, fragments and complete messages at their respective layers. |
| <code>FragmentReassembler</code> | 将分片或文本行重组为有校验、有身份的逻辑消息或完整帧。 / Reassembles fragments or text lines into checked, identified messages or full frames. |
| <code>FragmentReassembler.__init__(self) → None</code> | 使用所列参数构造本对象，初始化其计数、状态或 I/O 句柄。 / Constructs the object from the listed arguments and initializes its counters, state or I/O handles. |
| <code>FragmentReassembler.feed(self, fallback_channel: str, payload: bytes) → list[tuple[str, bytes]]</code> | 普通消息直接配上 fallback_channel；分片按通道和消息 ID 暂存。核对分片长度、数量、总长度（有声明时）和重组 CRC32；不完整或损坏时返回空列表，重复和乱序分别计数。 / Returns an unfragmented payload with fallback_channel. Buffers fragments by channel/message ID, checks part lengths/count, declared total size when present and assembled CRC32. Returns an empty list for incomplete or bad input and counts duplicates and reordering separately. |
| <code>FragmentReassembler._stat(self, fallback_channel: str) → FragmentStats</code> | 接收分片或返回本解析器的分通道统计和摘要。 / Consumes fragments or returns the parser’s per-channel statistics and summary. |
| <code>FragmentReassembler.summary(self, ch: str) → str</code> | 接收分片或返回本解析器的分通道统计和摘要。 / Consumes fragments or returns the parser’s per-channel statistics and summary. |
| <code>TextProtocolParser</code> | 将分片或文本行重组为有校验、有身份的逻辑消息或完整帧。 / Reassembles fragments or text lines into checked, identified messages or full frames. |
| <code>TextProtocolParser.__init__(self, on_cap_frame: Optional[Callable[[CapFrame], None]]=None, on_measurement_frame: Optional[Callable[[MeasurementFrame], None]]=None) → None</code> | 使用所列参数构造本对象，初始化其计数、状态或 I/O 句柄。 / Constructs the object from the listed arguments and initializes its counters, state or I/O handles. |
| <code>TextProtocolParser.note_non_ascii(self) → None</code> | 记录非 ASCII 输入或检查活动帧／行的掩码。 / Records non-ASCII input or validates masks for active frames and rows. |
| <code>TextProtocolParser._active_mask(cells: int) → int</code> | 记录非 ASCII 输入或检查活动帧／行的掩码。 / Records non-ASCII input or validates masks for active frames and rows. |
| <code>TextProtocolParser._parse_frame_masks(cls, fields: dict[str, str], cells: int) → Optional[tuple[int, int]]</code> | 记录非 ASCII 输入或检查活动帧／行的掩码。 / Records non-ASCII input or validates masks for active frames and rows. |
| <code>TextProtocolParser._parse_row_masks(cls, fields: dict[str, str]) → Optional[tuple[int, int]]</code> | 记录非 ASCII 输入或检查活动帧／行的掩码。 / Records non-ASCII input or validates masks for active frames and rows. |
| <code>TextProtocolParser.feed_line(self, line: str) → Optional[CapFrame &#124; MeasurementFrame &#124; MixedFrame]</code> | 剥离行尾并识别标签，分发数据头、D/P/MR 块、K 尾和生命周期行。完整时返回相应 CapFrame、MeasurementFrame 或 MixedFrame；其它行或未完成帧返回 None。 / Strips the line ending and dispatches tagged headers, D/P/MR chunks, K trailers and lifecycle text. Returns a CapFrame, MeasurementFrame or MixedFrame when assembly completes, otherwise None. |
| <code>TextProtocolParser._start_mixed_frame(self, line: str) → None</code> | 处理混合帧 M 头、MR 行或 K 尾，验证各行模式与完整性。 / Handles the M header, MR row or K trailer of a mixed frame and validates row modes and completeness. |
| <code>TextProtocolParser._add_mixed_row(self, line: str) → None</code> | 处理混合帧 M 头、MR 行或 K 尾，验证各行模式与完整性。 / Handles the M header, MR row or K trailer of a mixed frame and validates row modes and completeness. |
| <code>TextProtocolParser._finish_mixed_frame(self, line: str) → Optional[MixedFrame]</code> | 处理混合帧 M 头、MR 行或 K 尾，验证各行模式与完整性。 / Handles the M header, MR row or K trailer of a mixed frame and validates row modes and completeness. |
| <code>TextProtocolParser._observe_rowmodes_event(self, tag: str, line: str) → None</code> | 追踪逐行配置的请求与终态，返回顺序或身份错误。 / Tracks per-row requests and terminals and reports ordering or identity errors. |
| <code>TextProtocolParser.rowmode_lifecycle_errors(self) → dict[str, int]</code> | 追踪逐行配置的请求与终态，返回顺序或身份错误。 / Tracks per-row requests and terminals and reports ordering or identity errors. |
| <code>TextProtocolParser._start_cap_frame(self, line: str) → None</code> | 开始、填充或完成 CAP 帧并核对预期有效载荷。 / Starts, appends to or completes a CAP frame and checks its expected payload. |
| <code>TextProtocolParser._add_cap_chunk(self, line: str, chunk_index: int) → None</code> | 开始、填充或完成 CAP 帧并核对预期有效载荷。 / Starts, appends to or completes a CAP frame and checks its expected payload. |
| <code>TextProtocolParser._finish_cap_frame(self, line: str) → Optional[CapFrame]</code> | 开始、填充或完成 CAP 帧并核对预期有效载荷。 / Starts, appends to or completes a CAP frame and checks its expected payload. |
| <code>TextProtocolParser._start_measurement_frame(self, line: str, tag: str) → None</code> | 处理 VOLT/RES 头、数值、PGA 数据或尾部，按原单位和有效性提交完整帧。 / Handles VOLT/RES headers, data, PGA chunks or trailers and emits complete frames with their units and validity intact. |
| <code>TextProtocolParser._add_measurement_chunk(self, line: str, chunk_index: int) → None</code> | 处理 VOLT/RES 头、数值、PGA 数据或尾部，按原单位和有效性提交完整帧。 / Handles VOLT/RES headers, data, PGA chunks or trailers and emits complete frames with their units and validity intact. |
| <code>TextProtocolParser._add_pga_chunk(self, line: str, chunk_index: int) → None</code> | 处理 VOLT/RES 头、数值、PGA 数据或尾部，按原单位和有效性提交完整帧。 / Handles VOLT/RES headers, data, PGA chunks or trailers and emits complete frames with their units and validity intact. |
| <code>TextProtocolParser._finish_measurement_frame(self, line: str) → Optional[MeasurementFrame]</code> | 处理 VOLT/RES 头、数值、PGA 数据或尾部，按原单位和有效性提交完整帧。 / Handles VOLT/RES headers, data, PGA chunks or trailers and emits complete frames with their units and validity intact. |
| <code>TextProtocolParser._record_sequence(self, sequence: int) → None</code> | 记录本解析器首次和最近的主机单调收帧时间。相邻序号前跳时累加缺帧数，相同或回退时计为回退。此层没有 boot 参数，跨 boot 分组应使用配置身份分析。 / Tracks first/last monotonic host receipt. A forward jump increments the missing-frame count; an equal or decreasing sequence increments regressions. This parser has no boot argument; per-boot analysis groups by configuration identity. |
| <code>TextProtocolParser.cap_fps(self) → float</code> | 用 CAP 帧数减一除以当前解析器首尾主机收帧时差；时间范围不足时返回 0。 / Divides CAP frames minus one by elapsed host time between this parser’s first and last frame observations, returning zero if the interval is unavailable. |
| <code>TextProtocolParser.frame_fps(self) → float</code> | 在主机首尾收帧时差内，根据已解析完整帧总数计算观察速率。 / Calculates an observed rate from the completed-frame count and elapsed host time between first and last receipts. |
| <code>TextProtocolParser.summary(self, prefix: str) → str</code> | 接收分片或返回本解析器的分通道统计和摘要。 / Consumes fragments or returns the parser’s per-channel statistics and summary. |
| <code>format_cap_preview(frame: CapFrame) → str</code> | 将帧内电容或电压／电阻值格式化为简短的人读预览。 / Formats capacitance or voltage/resistance cells into a short human-readable preview. |
| <code>format_measurement_preview(frame: MeasurementFrame) → str</code> | 将帧内电容或电压／电阻值格式化为简短的人读预览。 / Formats capacitance or voltage/resistance cells into a short human-readable preview. |

<a id="validate_battery_formula"></a>

### validate_battery_formula

用分压输入、AINCOM 偏置及给定电源轨计算电池电压，提供独立的换算检查。 / Computes battery voltage using the divider input, AINCOM bias and supplied rails for an independent conversion check. [源码 / Source](validate_battery_formula.py)

运行 / Run　`python3 tools/validate_battery_formula.py`。

| 函数、类型与形参 / Callable, type and parameters | 中文用途 / English behavior |
| --- | --- |
| <code>calculate(ain8_diff_uv: int, zero_uv: int, aincom_uv: int &#124; None, numerator: int, denominator: int) → tuple[int, str]</code> | 通过电源轨估算 AINCOM 或将 ADC 读数换算成电池电压。 / Estimates AINCOM from the rails or converts an ADC reading to battery voltage. |
| <code>aincom_from_rail(rail_uv: int, avdd_uv: int, avss_magnitude_uv: int) → int</code> | 通过电源轨估算 AINCOM 或将 ADC 读数换算成电池电压。 / Estimates AINCOM from the rails or converts an ADC reading to battery voltage. |
| <code>main() → int</code> | 解析命令行，执行本模块的操作并按结果退出。 / Parses CLI arguments, runs the module operation and reports the resulting exit status. |

<a id="validate_measurement_modes"></a>

### validate_measurement_modes

通过串口逐模式测量，核查电阻位置、ADC 故障、PGA、rail、电池及 RES 建立时间。 / Runs serial measurements by mode and checks known resistor coordinates, ADC faults, PGA, rails, battery and RES settling. [源码 / Source](validate_measurement_modes.py)

命令 / CLI　`python3 tools/validate_measurement_modes.py --help`。参数 / Arguments　`--port`、`--baud`、`--duration`、`--startup-timeout`、`--rows`、`--modes`、`--cycles`、`--known-resistor`、`--known-capacitor`、`--probe-cell`、`--rail-avdd-uv`、`--rail-avss-uv`、`--resistor-tolerance-percent`、`--res-settle-values`、`--res-settle-frames`、`--skip-res-settle-sweep`、`--performance-frames`、`--battery-dwell-seconds`、`--no-reset`、`--output-directory`。

| 函数、类型与形参 / Callable, type and parameters | 中文用途 / English behavior |
| --- | --- |
| <code>parse_coordinate(value: str) → tuple[int, int]</code> | 校验 S/D 或行列格式，并转换到矩阵中的对应坐标。 / Checks a row/column coordinate and maps it to the selected matrix cell. |
| <code>parse_known_resistor(value: str) → tuple[tuple[int, int], float]</code> | 解析、提供或核查已知电阻的行列位置及预期阻值。 / Parses, supplies or checks known resistor coordinates and expected resistance values. |
| <code>parse_fields(line: str) → dict[str, str]</code> | 从第一项标签之后提取首个等号分隔的各键和值。 / Splits comma-delimited fields after the tag at the first equals sign in each field. |
| <code>StageResult(mode: str, rows: int, frames: int = 0, observed_frames: int = 0, crc_errors: int = 0, sequence_first: int = 0, sequence_last: int = 0, elapsed_seconds: float = 0.0, fps: float = 0.0, acquisition_fps: float = 0.0, valid_cells_min: int = 64, error_cells_max: int = 0, freshness_failures: int = 0, frame_duration_us_max: int = 0, gain_changes_max: int = 0, autorange_attempts_max: int = 0, autorange_fallbacks_max: int = 0, io_retries_total: int = 0, overrange_total: int = 0, drdy_timeouts_total: int = 0, stale_total: int = 0, spi_errors_total: int = 0)</code> | 保存按方法、坐标或工况汇总的输入或结果。 / Holds inputs or results grouped by method, cell coordinate or test stage. |
| <code>ValidationResult(port: str, baud: int, started_at: str, firmware_ads_boot: str = &#x27;&#x27;, self_test: str = &#x27;&#x27;, protocol_self_test: str = &#x27;&#x27;, fdc_boot: str = &#x27;&#x27;, rail_status: dict[str, str] = field(default_factory=dict), ads_status: dict[str, str] = field(default_factory=dict), ads_check: dict[str, str] = field(default_factory=dict), ads_check_stats: dict[str, str] = field(default_factory=dict), ads_cache_telemetry: list[dict[str, str]] = field(default_factory=list), ads_timing_telemetry: list[dict[str, str]] = field(default_factory=list), battery_summary_telemetry: list[dict[str, str]] = field(default_factory=list), battery_status: dict[str, str] = field(default_factory=dict), res_settle_sweep: list[dict[str, object]] = field(default_factory=list), battery_dwell: list[dict[str, object]] = field(default_factory=list), performance: list[dict[str, object]] = field(default_factory=list), mode_status: dict[str, dict[str, str]] = field(default_factory=dict), stages: list[StageResult] = field(default_factory=list), known_resistors: dict[str, dict[str, object]] = field(default_factory=dict), known_capacitors: dict[str, dict[str, object]] = field(default_factory=dict), probed_cells: dict[str, dict[str, str]] = field(default_factory=dict), cycles_completed: int = 0, ads_frame_events: list[dict[str, str]] = field(default_factory=list), ads_error_summary: dict[str, int] = field(default_factory=dict), resets: int = 0, watchdogs: int = 0, fatal_lines: list[str] = field(default_factory=list), failures: list[str] = field(default_factory=list))</code> | 保存按方法、坐标或工况汇总的输入或结果。 / Holds inputs or results grouped by method, cell coordinate or test stage. |
| <code>summarise_ads_frame_events(events: list[dict[str, str]]) → dict[str, int]</code> | 汇总 ADSFRAME 中的有效性、尝试和故障字段。 / Summarizes validity, attempts and fault fields from ADSFRAME events. |
| <code>summarise_ads_frame_events.number(name: str) → int</code>（内部 / local） | 读取并检查数值字段，保留该调用的范围及未知值规则。 / Reads a numeric field subject to the caller's range and unknown-value rules. |
| <code>SerialValidator</code> | 管理指定接口的数据接收、解码、控制或验收。 / Manages receipt, decoding, control or validation on the designated interface. |
| <code>SerialValidator.__init__(self, connection: object, raw_log: object, result: ValidationResult) → None</code> | 使用所列参数构造本对象，初始化其计数、状态或 I/O 句柄。 / Constructs the object from the listed arguments and initializes its counters, state or I/O handles. |
| <code>SerialValidator.write_command(self, command: str) → None</code> | 按通道封装并发送一次控制命令。 / Formats and submits one control command on the selected channel. |
| <code>SerialValidator.read_once(self) → tuple[str, Optional[Frame]]</code> | 读取串口一次或将读到的行提交给验证器。 / Reads once from serial or submits a received line to the validator. |
| <code>SerialValidator._observe_line(self, line: str) → None</code> | 读取串口一次或将读到的行提交给验证器。 / Reads once from serial or submits a received line to the validator. |
| <code>SerialValidator.wait_line(self, predicate: Callable[[str], bool], timeout: float, description: str) → str</code> | 等待接收队列中满足条件的下一行或观测。 / Waits for the next line or matching observation from the receive queue. |
| <code>SerialValidator.wait_frame(self, predicate: Callable[[Frame], bool], timeout: float, description: str) → Frame</code> | 等待符合模式、身份或序号谓词的完整帧。 / Waits for a complete frame satisfying mode, identity or sequence predicates. |
| <code>SerialValidator.switch_mode(self, mode: str, timeout: float) → tuple[int, int]</code> | 发起配置切换，跟踪请求及应用终态。 / Requests a configuration change and tracks its request and terminal state. |
| <code>SerialValidator.switch_mode.matches(item: Frame) → bool</code>（内部 / local） | 检查当前记录是否满足外层等待的条件。 / Tests whether the current record satisfies the enclosing wait predicate. |
| <code>SerialValidator.query_mode(self, mode: str, timeout: float) → dict[str, str]</code> | 查询模式、电池或指定单元并返回匹配回复。 / Queries mode, battery or the specified cell and returns the matching reply. |
| <code>SerialValidator.set_rail_calibration(self, avdd_uv: int, avss_uv: int, timeout: float) → dict[str, str]</code> | 在相应控制通路提交并核查给定的外部 rail 校准。 / Submits and verifies supplied external rail calibration on the indicated control transport. |
| <code>SerialValidator.set_rows(self, rows: int, timeout: float) → None</code> | 在指定控制通路请求行数并核对实际应用。 / Requests a row count on the selected control path and verifies application. |
| <code>SerialValidator.run_active_ads_check(self, sample_count: int, timeout: float) → None</code> | 触发有界 ADS／电池检查，或在驻留窗口内观察电池状态。 / Requests a bounded ADS/battery check or observes battery status during a dwell window. |
| <code>SerialValidator.query_battery(self, timeout: float) → dict[str, str]</code> | 查询模式、电池或指定单元并返回匹配回复。 / Queries mode, battery or the specified cell and returns the matching reply. |
| <code>SerialValidator.run_battery_now(self, timeout: float) → dict[str, str]</code> | 触发有界 ADS／电池检查，或在驻留窗口内观察电池状态。 / Requests a bounded ADS/battery check or observes battery status during a dwell window. |
| <code>SerialValidator._field_int(fields: dict[str, str], name: str) → int</code> | 读取并检查数值字段，保留该调用的范围及未知值规则。 / Reads a numeric field subject to the caller's range and unknown-value rules. |
| <code>SerialValidator._mode_frame_is_clean(frame: Optional[Frame], mode: str, rows: int) → bool</code> | 检查模式帧的科学有效性或运行有界性能验收。 / Checks scientific validity of a mode frame or executes the bounded performance screen. |
| <code>SerialValidator.run_performance_acceptance(self, mode: str, frame_count: int, timeout: float) → None</code> | 检查模式帧的科学有效性或运行有界性能验收。 / Checks scientific validity of a mode frame or executes the bounded performance screen. |
| <code>SerialValidator.run_battery_dwell(self, mode: str, duration_seconds: float, timeout: float) → None</code> | 触发有界 ADS／电池检查，或在驻留窗口内观察电池状态。 / Requests a bounded ADS/battery check or observes battery status during a dwell window. |
| <code>SerialValidator.set_res_settle(self, settle_us: int, timeout: float) → None</code> | 设置或扫描 RES 建立时间，或在指定工况收集一组样本。 / Sets or sweeps RES settling time, or collects a sample set for the specified stage. |
| <code>SerialValidator.run_res_settle_sweep(self, settle_values: list[int], frame_count: int, known_resistors: dict[tuple[int, int], float], tolerance_percent: float, timeout: float) → None</code> | 设置或扫描 RES 建立时间，或在指定工况收集一组样本。 / Sets or sweeps RES settling time, or collects a sample set for the specified stage. |
| <code>SerialValidator.sample_stage(self, mode: str, rows: int, timeout: float, frame_count: int=4) → StageResult</code> | 设置或扫描 RES 建立时间，或在指定工况收集一组样本。 / Sets or sweeps RES settling time, or collects a sample set for the specified stage. |
| <code>SerialValidator.query_cell(self, coordinate: tuple[int, int], timeout: float) → dict[str, str]</code> | 查询模式、电池或指定单元并返回匹配回复。 / Queries mode, battery or the specified cell and returns the matching reply. |
| <code>main() → int</code> | 解析命令行，执行本模块的操作并按结果退出。 / Parses CLI arguments, runs the module operation and reports the resulting exit status. |

<a id="validate_transports"></a>

### validate_transports

将串口、BLE、Wi-Fi 接收器编排成带时限的阶段，保存原始输出并检查接收器的末次计数。 / Runs serial, BLE and Wi-Fi receivers as bounded stages. Saves receiver output and checks final counters. [源码 / Source](validate_transports.py)

命令 / CLI　`python3 tools/validate_transports.py --help`。参数 / Arguments　`--port`、`--rows`、`--duration`、`--serial-duration`、`--ble-duration`、`--serial`、`--ble`、`--wifi`、`--require-rowmodes`、`--stream`、`--tx`、`--wifi-host`、`--wifi-ip`。

| 函数、类型与形参 / Callable, type and parameters | 中文用途 / English behavior |
| --- | --- |
| <code>build_parser() → argparse.ArgumentParser</code> | 创建含子命令、选项和缺省值的参数解析器。 / Constructs the argument parser with supported subcommands, options and defaults. |
| <code>should_run_serial(args: argparse.Namespace) → bool</code> | 选择测试阶段，并给各通路分配与核查时长／截止时间。 / Selects validation stages and allocates or checks durations/deadlines for each transport. |
| <code>stage_plan(args: argparse.Namespace) → list[str]</code> | 选择测试阶段，并给各通路分配与核查时长／截止时间。 / Selects validation stages and allocates or checks durations/deadlines for each transport. |
| <code>resolve_durations(args: argparse.Namespace) → dict[str, float]</code> | 选择测试阶段，并给各通路分配与核查时长／截止时间。 / Selects validation stages and allocates or checks durations/deadlines for each transport. |
| <code>validate_durations(durations: dict[str, float], serial_requested: bool, ble_requested: bool, wifi_requested: bool) → str &#124; None</code> | 选择测试阶段，并给各通路分配与核查时长／截止时间。 / Selects validation stages and allocates or checks durations/deadlines for each transport. |
| <code>normalize_tx_mode(tx: str) → str</code> | 将 UUID、USB 序列号或 TX 模式规范为该后端匹配所用的形式。 / Normalizes a UUID, serial number or TX mode into the form its backend compares. |
| <code>ble_rows_arg(rows: str) → str &#124; None</code> | 将目标行数转换为 BLE 接收器参数。 / Converts the requested row count to the BLE receiver option. |
| <code>stage_timeout(duration_s: float) → float</code> | 选择测试阶段，并给各通路分配与核查时长／截止时间。 / Selects validation stages and allocates or checks durations/deadlines for each transport. |
| <code>serial_stage_command(root: Path, python: str, args: argparse.Namespace, serial_duration: float) → list[str]</code> | 构造相应接收器的子进程参数。 / Builds the subprocess arguments for the selected transport receiver. |
| <code>ble_stage_command(root: Path, python: str, args: argparse.Namespace, ble_duration: float) → list[str]</code> | 构造相应接收器的子进程参数。 / Builds the subprocess arguments for the selected transport receiver. |
| <code>wifi_stage_command(root: Path, python: str, args: argparse.Namespace, wifi_duration: float) → list[str] &#124; None</code> | 构造相应接收器的子进程参数。 / Builds the subprocess arguments for the selected transport receiver. |
| <code>parse_kv_line(line: str) → dict[str, str]</code> | 解析逗号分隔的标签与键值字段，保留 wire 表示。 / Parses the tag and comma-delimited key/value fields while preserving their wire representation. |
| <code>last_summary_fields(output: str, marker: str) → dict[str, str]</code> | 提取末次摘要并核对该接收器的完成状态、帧和计数。 / Extracts the last summary and checks receiver completion, frames and counters. |
| <code>_int_field(fields: dict[str, str], name: str) → int &#124; None</code> | 读取并检查数值字段，保留该调用的范围及未知值规则。 / Reads a numeric field subject to the caller's range and unknown-value rules. |
| <code>validate_serial_output(output: str, require_rowmodes: bool=False) → str &#124; None</code> | 提取末次摘要并核对该接收器的完成状态、帧和计数。 / Extracts the last summary and checks receiver completion, frames and counters. |
| <code>validate_ble_output(output: str, port: str, require_rowmodes: bool=False) → str &#124; None</code> | 提取末次摘要并核对该接收器的完成状态、帧和计数。 / Extracts the last summary and checks receiver completion, frames and counters. |
| <code>decode_output(value: bytes &#124; str &#124; None) → str</code> | 解码、回显子进程输出或在截止前执行一个测试阶段。 / Decodes or echoes child output, or executes a validation stage within its deadline. |
| <code>echo_captured(stdout: str &#124; None, stderr: str &#124; None) → str</code> | 解码、回显子进程输出或在截止前执行一个测试阶段。 / Decodes or echoes child output, or executes a validation stage within its deadline. |
| <code>run_stage(name: str, command: list[str], timeout_s: float, duration_s: float, port: str) → tuple[int, str]</code> | 解码、回显子进程输出或在截止前执行一个测试阶段。 / Decodes or echoes child output, or executes a validation stage within its deadline. |
| <code>main(argv: list[str] &#124; None=None) → int</code> | 解析命令行，执行本模块的操作并按结果退出。 / Parses CLI arguments, runs the module operation and reports the resulting exit status. |

## 参数与数据边界 / Parameter conventions

字节偏移从捕获文件起点开始，`_us` 或 `Us` 表示微秒。主机 BLE 接收时间使用注明的单调时钟。分片、完整帧、传输提交和采集各自有独立分母。 / Byte offsets start at the beginning of the capture. The suffix `_us` or `Us` denotes microseconds. BLE receipt times use the named monotonic clock. Fragments, complete frames, submissions and device captures have separate denominators.

| 参数名 / Parameter names | 含义 / Meaning |
| --- | --- |
| `ble_journal`, `build`, `capture`, `captures`, `closing`, `debug_summaries`, `event`, `events`, `evidence`, `journal`, `opening`, `ready_intervals`, `record`, `records`, `report`, `result`, `stream`, `summary`, `window` | 已捕获的字节流、快照、前后界、观察窗口／元数据、输入分析或结果对象。 / Captured stream, snapshot, brackets, observation window/metadata, analysis or result. |
| `cells`, `chunk_index`, `ctrl_raw`, `data`, `frame`, `frames`, `header`, `original_usb`, `payload`, `payload_bytes`, `point`, `raw`, `raw_log`, `trailer`, `value`, `values`, `wire_bytes` | 帧头、行／尾、未改写载荷或原始测点值。 / Header, rows/trailer, unmodified bytes or original cell observations. |
| `completedFrames`, `dataFrames`, `expectedCount`, `frameTarget`, `frame_count`, `length`, `limit`, `logMessages`, `max_bytes`, `maximum`, `maximumAge`, `maximumStaleAssemblies`, `minimumLogBytes`, `minimumLogStackBytes`, `minimumOtherBytes`, `minimumOtherStackBytes`, `requiredLogTags` | 样本／帧／字节数量、允许误差、队列上限或栈／堆限值，由变量名标明计量对象。 / Count, tolerance, queue bound or stack/heap limit; the parameter name specifies the unit being counted. |
| `BleakClient`, `BleakScanner`, `_client`, `client`, `detector`, `device`, `metadata_finder`, `monitor`, `observer`, `port`, `port_factory`, `receiver`, `sidecar`, `usb_core`, `usb_util` | 调用方提供的端口、收发对象、硬件适配器或可替换的测试实现。 / Caller-provided I/O object, adapter or replaceable test implementation. |
| `clearExisting`, `forceIndicate`, `logSubscribed`, `observe`, `only_capture`, `predicate`, `replace`, `requireFirmwareTelemetry`, `requireTelemetry`, `require_image_identity`, `require_rowmodes`, `require_running`, `source_ok`, `strictSequence`, `synthetic` | 选择校验强度、操作范围、重放或测试来源的标志／谓词。 / Flag or predicate controlling validation, scope, replacement or synthetic provenance. |
| `description`, `event_type`, `field`, `fields`, `keys`, `label`, `label_prefix`, `labels`, `name`, `prefix`, `stem`, `tag`, `text`, `title` | 待解析或输出的键、标签、字段名、前后缀或说明文本。 / Key, label, field name, prefix, suffix or explanatory text for parsing/output. |
| `ble_requested`, `ch`, `channel`, `channels`, `fallbackChannel`, `fallback_channel`, `host`, `interface_number`, `pid`, `serial`, `serial_requested`, `vid`, `wifi_requested` | 通路选择、蓝牙通道、端口、进程号或设备标识，按所在函数解释。 / Transport/channel selection, port, process ID or device identity as indicated by the function. |
| `dev_root`, `events_path`, `fixture_path`, `logPath`, `outputDirectory`, `output_directory`, `path`, `ready_path`, `root`, `sdk_root`, `selection_path`, `sys_tty` | 相应的输入文件、输出位置、仓库／SDK 根或 sysfs 设备目录。 / Matching input/output location, repository/SDK root or sysfs device directory. |
| `ble_duration`, `duration_seconds`, `durations`, `minimumSeconds`, `queryInterval`, `quietSeconds`, `seconds`, `secondsTarget`, `serial_duration`, `settleSeconds`, `wifi_duration` | 采样、驻留、静默、预热、查询或收尾区间，单位为秒。 / Capture, dwell, quiet, warmup, query or cleanup interval in seconds. |
| `ain8_diff_uv`, `aincom_uv`, `avddUv`, `avdd_uv`, `avssUv`, `avss_magnitude_uv`, `avss_uv`, `rail`, `rail_uv`, `zero_uv` | AIN 输入、电源轨或零点量，带 uv/Uv 后缀的数值为微伏；AVSS 保留符号。 / AIN input, rail or zero quantity in microvolts for uv/Uv suffixes; signed AVSS is preserved. |
| `configuration`, `generation`, `phase_deadline`, `requestId`, `restoreBtx`, `restoreStream`, `restoreTx`, `settle_us`, `settle_values`, `tx` | 设备控制参数、TX 输出策略、应恢复的设置或输入建立时间；us 表示微秒。 / Device controls, TX policy, settings to restore or input settling duration; us denotes microseconds. |
| `bounds`, `index`, `item`, `items`, `key`, `offset`, `source`, `startIndex`, `target` | 目标条目、索引、字节边界、设备清单或原始证据引用。 / Target item, index, byte boundary, device inventory or original evidence reference. |
| `coordinate`, `fixture`, `floating`, `knownRanges`, `known_resistors`, `provenance`, `ranges`, `ref` | 夹具参考位置、已知元件、公差、背景供体或实录／合成来源。 / Fixture coordinates, known part, tolerance, background donor or recorded/synthetic provenance. |
| `disconnectedCallback`, `fault_check`, `fingerprints`, `memory_validator`, `on_cap_frame`, `on_measurement_frame`, `parser`, `stop_event` | 解析器、校验函数、回调、比对存储或异步停止信号。 / Parser, validator, callback, comparison store or asynchronous stop signal. |
| `_frame`, `_sender`, `backend`, `client_class`, `cls`, `scanner`, `self` | 回调提供的发送端、信号、栈帧、函数或类／实例接收者。 / Callback sender, signal, stack frame, function or class/instance receiver. |
| `appliedSequence`, `boot_id`, `currentSequence`, `data_boots`, `previousSequence`, `sequence`, `session` | boot/session、配置代或帧序号，从同一身份内进行比较。 / Boot/session identity, configuration generation or frame sequence compared within its identity domain. |
| `config`, `group`, `identity_status`, `mode`, `profile`, `raw_rows`, `rows` | 工况、行数、逐行模式或采集／控制／诊断类别；按调用范围选择。 / Case, row count, per-row profile or capture/control/diagnostic class for this operation. |
| `denominator`, `fraction`, `heapLeakToleranceBytes`, `numerator`, `perf_counters`, `sample_count`, `tolerance_percent` | 比率、误差、位宽、速率或计数。percent 表示百分比，fraction 表示 0–1 的比例。 / Ratio, error, bit width, rate or count; percent is a percentage and fraction uses 0–1. |
| `directory`, `document`, `log_file`, `output`, `raw_path`, `relative_path`, `report_path` | 目标文件、日志或目录；相对路径由调用点声明的根目录解释。 / Output file, journal or directory. Relative paths refer to the root specified by the caller. |
| `commandTimeout`, `frameTimeout`, `maximumSeconds`, `modeTimeout`, `timeout`, `timeout_s` | 允许等待的时间上限，以秒为单位，若接口注明其它单位则遵循该接口。 / Maximum wait, in seconds unless the interface specifies a different unit. |
| `connection`, `continuity`, `contract`, `expected_contract`, `protocol`, `same_boot` | 协议契约、连通状态或连续性检查条件。 / Protocol contract, connection state or continuity check. |
| `duration_s`, `fallback`, `kind`, `phase`, `phases`, `profileName` | 测试或采集阶段及相应约束。 / Test/capture phase and its applicable constraints. |
| `ackLine`, `line`, `lines`, `raw_trailer`, `terminalLine` | 收到的协议行、应答或终态原文。 / Received protocol lines, acknowledgments or terminal text. |
| `allowedMessageDrops`, `error`, `marker`, `pending`, `reason` | 等待条件、故障原因、回退值或就绪／排队状态。 / Wait condition, fault reason, fallback value or readiness/queued state. |
| `args`, `argv`, `kwargs`, `options`, `settings` | 调用参数、解析后的命令行选项或调用方提供的关键字参数。 / Invocation arguments, parsed CLI options or caller-supplied keyword arguments. |
| `candidates`, `expected`, `required`, `selection`, `window_label` | 候选、被选择或要求匹配的观测／配置集合。 / Candidate, selected or required observations/configurations. |
| `digest`, `elf_sha256`, `expected_backend`, `expected_identity`, `expected_sha` | 预期镜像摘要、配置身份、后端或字段约束，用于校验实际观测。 / Expected image digest, config identity, backend or field contract checked against observations. |
| `end`, `monotonic_ns`, `ns`, `received_ns`, `start` | 观测边界或接收时刻，带 ns 后缀的参数使用纳秒。 / Observation bounds or receipt times. The ns suffix denotes nanoseconds. |
| `baseline`, `cleanup`, `initialEvidence`, `order_ok` | 用于前后对照、恢复或同一次配置检查的两端状态。 / Bracketing state for a delta, recovery or a single configuration check. |
| `found`, `match`, `patterns` | 源码审计中的匹配模式、匹配对象及已收集项。 / Patterns, match object or accumulated findings in source auditing. |
| `python`, `stderr`, `stdout` | 主机 Python 解释器及子进程标准输出／错误流。 / Host Python executable and captured child stdout/stderr. |
| `base`, `origin` | 源数据起点、基准目录或单调时间原点。 / Source origin, base directory or monotonic-time origin. |
| `carry`, `ctrl_carry` | 前一次读取末尾留下的未完整 DATA／CTRL 字节。 / Partial DATA/CTRL bytes carried over from a previous read. |
| `command`, `queryCommands` | 协议命令、查询集、事务请求号或单次命令等待预算。 / Wire command, query set, transaction request ID or per-command timeout. |
| `environmentPort`, `explicitPort` | 来自环境变量或调用方的串口路径。 / Serial-port path from the environment or explicit selection. |
| `awaitable` | 在限时内等待的异步调用。 / Awaitable completed within the supplied deadline. |
| `baud` | 串口波特率设定。 / Serial-port baud rate setting. |
| `bulk` | USB bulk、I/O 或 SPI 原始事务记录。 / Original USB bulk, I/O or SPI transaction evidence. |
| `clock` | 提供单调计时或等待的函数，可用于无硬件的确定性测试。 / Clock or wait function, injectable for deterministic host testing. |
| `deadline` | 主机单调时钟的绝对截止时刻；嵌套操作共享上层预算。 / Absolute host-monotonic deadline shared with enclosing operations. |
| `declared` | 消息、掩码或载荷声明的边界。 / Declared message, mask or payload boundary. |
| `frame_rejections` | 解析器按原因累计的拒帧数。 / Counts of frame rejections by parser reason. |
| `message` | 已组装的链路消息或诊断文字。 / Assembled link message or diagnostic text. |
| `timeline` | 字节或帧对应的主机接收时间索引。 / Host receipt timeline indexed by bytes or frames. |
| `writeWithResponse` | BLE GATT 写操作是否要求协议栈返回响应。 / Whether a BLE GATT write requests an ATT response. |

## 回归测试入口 / Regression entry points

逐一以脚本方式运行 `test_*.py`。部分脚本直接编译并执行引用生产 C 源的 host harness，单靠 unittest discovery 会漏掉这些入口。`CC` 选择主机 C 编译器，IDF 相关项还需要可用 SDK。 / Run every test file as a script. Several compile and execute host harnesses against production C. Unittest discovery alone misses those entry points. Select a host C compiler via CC and make the SDK available for IDF-specific checks.

~~~sh
for test in tools/test_*.py
do
  CC=cc python3 "$test" || exit 1
done
~~~

[test_config_identity.py](test_config_identity.py) 默认在临时目录运行生产 encoder，并逐字节比对版本控制中的合成金样。仅显式传入 `--update` 时替换金样。 / Runs the production encoder in a temporary directory and compares synthetic goldens byte for byte. Only an explicit `--update` replaces the versioned goldens.

**ADS、rail、电池 / ADS, rails and battery**　[test_adc_read_provenance](test_adc_read_provenance.py) · [test_ads_cell_stability](test_ads_cell_stability.py) · [test_ads_crc_disposition](test_ads_crc_disposition.py) · [test_ads_crc_frame_flow](test_ads_crc_frame_flow.py) · [test_ads_driver_reliability](test_ads_driver_reliability.py) · [test_ads_fault_fixtures](test_ads_fault_fixtures.py) · [test_ads_matrix_diagnostics](test_ads_matrix_diagnostics.py) · [test_ads_presence](test_ads_presence.py) · [test_ads_rail_error_chain](test_ads_rail_error_chain.py) · [test_ads_rail_profile](test_ads_rail_profile.py) · [test_ads_scientific_read](test_ads_scientific_read.py) · [test_ads_spi_probe](test_ads_spi_probe.py) · [test_ads_tail_diagnostic](test_ads_tail_diagnostic.py) · [test_ads_timing_control](test_ads_timing_control.py) · [test_ads_timing_probe](test_ads_timing_probe.py) · [test_battery_snapshot_evidence](test_battery_snapshot_evidence.py) · [test_battery_validity](test_battery_validity.py) · [test_mixed_rail_boundary](test_mixed_rail_boundary.py)

**采集器、诊断及资源 / Collectors, diagnostics and resources**　[test_analyze_firmware_benchmark](test_analyze_firmware_benchmark.py) · [test_async_log_storage](test_async_log_storage.py) · [test_async_log_summary](test_async_log_summary.py) · [test_cpu_owner](test_cpu_owner.py) · [test_firmware_benchmark](test_firmware_benchmark.py) · [test_firmware_benchmark_retry](test_firmware_benchmark_retry.py) · [test_load_benchmark_capture](test_load_benchmark_capture.py) · [test_main_fault_context](test_main_fault_context.py) · [test_memory_diagnostics](test_memory_diagnostics.py) · [test_panic_diagnostics](test_panic_diagnostics.py) · [test_performance_task_wrapper](test_performance_task_wrapper.py) · [test_restart_budget](test_restart_budget.py) · [test_sensorarray_hil](test_sensorarray_hil.py) · [test_text_protocol](test_text_protocol.py) · [test_work_budget](test_work_budget.py)

**测量、身份与夹具 / Measurement, identity and fixture**　[test_audit_config_surface](test_audit_config_surface.py) · [test_build_identity_reply](test_build_identity_reply.py) · [test_calibration_persistence](test_calibration_persistence.py) · [test_capture_identity](test_capture_identity.py) · [test_capture_reference_fixture](test_capture_reference_fixture.py) · [test_config_identity](test_config_identity.py) · [test_estimate_floating_cap_background](test_estimate_floating_cap_background.py) · [test_frame_semantics](test_frame_semantics.py) · [test_main_config_commit](test_main_config_commit.py) · [test_measurement_logic](test_measurement_logic.py) · [test_reference_fixture](test_reference_fixture.py) · [test_route_safety](test_route_safety.py) · [test_rowmodes_ack_guard](test_rowmodes_ack_guard.py) · [test_rowmodes_frame_metadata](test_rowmodes_frame_metadata.py) · [test_summarize_reference_capture](test_summarize_reference_capture.py)

**传输、控制及输出 / Transports, control and output**　[test_ble_crc_planner](test_ble_crc_planner.py) · [test_ble_output_schedule](test_ble_output_schedule.py) · [test_ble_pipeline](test_ble_pipeline.py) · [test_ble_priority](test_ble_priority.py) · [test_ble_watchdog](test_ble_watchdog.py) · [test_bulk_diagnostics](test_bulk_diagnostics.py) · [test_capture_benchmark_ble](test_capture_benchmark_ble.py) · [test_ctrl_reply_guard](test_ctrl_reply_guard.py) · [test_firmware_benchmark_mixed_wire](test_firmware_benchmark_mixed_wire.py) · [test_link_control](test_link_control.py) · [test_protected_frame_pipeline](test_protected_frame_pipeline.py) · [test_receive_ble_text](test_receive_ble_text.py) · [test_recovery_control](test_recovery_control.py) · [test_serial_io](test_serial_io.py) · [test_snapshot_diagnostics](test_snapshot_diagnostics.py) · [test_transport_frame_metadata](test_transport_frame_metadata.py) · [test_transport_pool](test_transport_pool.py) · [test_transport_storage](test_transport_storage.py) · [test_usb_bulk_capture](test_usb_bulk_capture.py) · [test_usb_cdc_capture](test_usb_cdc_capture.py) · [test_usb_otg_owner](test_usb_otg_owner.py) · [test_validate_transports](test_validate_transports.py) · [test_wifi_command](test_wifi_command.py) · [test_wifi_owner](test_wifi_owner.py) · [test_wifi_startup_gate](test_wifi_startup_gate.py) · [test_wifi_transport_isolation](test_wifi_transport_isolation.py)

**FDC 与硬件时序 / FDC and hardware timing**　[test_boot_spi_parallel](test_boot_spi_parallel.py) · [test_fdc_iso_mailbox](test_fdc_iso_mailbox.py) · [test_fdc_worker_lifecycle](test_fdc_worker_lifecycle.py) · [test_fdc_worker_runtime](test_fdc_worker_runtime.py) · [test_frontend_presence](test_frontend_presence.py) · [test_frontend_wait_integration](test_frontend_wait_integration.py)

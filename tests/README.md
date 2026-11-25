# 主机测试与夹具 / Host tests and fixtures

host/ 是用于链接生产 C 模块的测试源及受控 ESP-IDF/FreeRTOS 替身。入口在 tools/test_*.py，部分脚本编译并执行 C，另一些直接执行 Python 解析与调度用例。由 wrapper 选择编译宏、包含路径和 stub；从根目录调用，例如 `CC="$(command -v cc)" python3 tools/test_fdc_worker_runtime.py`。退出非零时保留最早断言。 / Use the matching Python wrapper to compile production modules alongside hardware, clock and queue substitutes. Run wrappers as programs to include their assertions. HIL collectors require real interfaces.

| 输入 / Input | 用途、参数及调用 / Use and invocation |
|---|---|
| [ADS 故障 / Fault contexts](fixtures/ads_fault/README.md) | 阶段、原始错误、cleanup、attempted/acquired；用 formatter 复现。 / Replay synthetic pre- and post-cleanup context through the production formatter. |
| [配置帧 / Configuration and wire](fixtures/config_identity/README.md) | 实际 encoder 的合成 CAP/VOLT/RES/MIXED 及原子 CONFIG。 / Verify bytes, CRC, masks, domain authority and late events. |
| [实录 / Recorded frames](fixtures/reference_capture/README.md) | 真实只读字节、来源 SHA 与切片偏移。 / Decode unmodified fixture recordings. |
| [分片基准 / Planner baseline](fixtures/ble_crc_planner/README.md) | 参考实现和实际 golden，同时对比消息大小、MTU、CRC。 / Differential planner and CRC suite. |
| [帧元数据 / Frame metadata](fixtures/transport_frame_meta/README.md) | 固定源码作为旧解析器，对同一 payload 比较。 / Preserve baseline behavior for queued metadata. |
| [串口片段 / Serial fragments](fixtures/serial_io/) | `python3 tools/test_serial_io.py`。用完整及交错片段检测并发写入；保留损坏字节。 / Exercise atomic record writes with intact and interleaved data. |
| [日志源码 / Logger baseline](fixtures/async_log/) | `python3 tools/test_async_log_summary.py`。校验固定源码 SHA，与当前输出比较。 / Check canonical summary formatting against the archived production implementation. |
| [被动夹具 / Passive fixture](fixtures/reference_10k_33pf.json) | 校准角色、位置、标称值及未知容差输入。 / Input map for reference_fixture.py and read-only analyzers. |
| [任务入口 / Task wrapper cases](fixtures/performance_task_wrapper_cases.json) | `python3 tools/test_performance_task_wrapper.py`。检查 task 返回防护配置。 / Matrix of build configurations for task-return checks. |

[SHORT v1 默认判据的只读摘录 / SHORT oracle](fixtures/hil/short_v1/source/capture_short_snapshots.py.txt) 供分母/覆盖回归调用。它是固定输入；新采集使用工具目录的维护版本。 / The replay suite uses this immutable excerpt to compare coverage and denominator rules. New captures use the maintained runner.

[完整测试方法 / Methods](../docs/validation.md) · [测试工具 / Wrappers and APIs](../tools/README.md) · [C 测试源与替身 / C sources and stubs](host/)

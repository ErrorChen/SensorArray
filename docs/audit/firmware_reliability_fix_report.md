# SensorArray 固件可靠性修复报告

> **2026-11-16 最新实机阶段 / Latest hardware stage:** 当前板 `a656d53`（160 MHz、`-O2`、CPU owner 诊断 ON），67 个 host 入口、目标构建和完整 BUILD/READY 已核验。compiler-perf-01 移除 task-return wrapper，已拒绝且未烧录；最终版保留真实 SDK wrapper、assert、栈/堆及 watchdog 检查。screen10 完整 120 秒 ADC CRC 0/84,943，Capture/USB/BLE 65.59/65.62/44.16 FPS；USB 7,873 帧内部连续，BLE 5,298 帧缺 2,572 个序号观察，仍 **FAIL_BLE_THROUGHPUT**。2.5% CRC 错误率门槛不变，截至 screen10 闭合时 600 秒/32 配置未运行；原 FW01–FW10 和失败历史保留。 / The board runs `a656d53` (160 MHz, `-O2`, CPU-owner diagnostics ON), with all 67 host entries, the target build and full BUILD/READY verified. Compiler-perf-01 removed the task-return wrapper and was rejected without flashing; the final image retains the actual SDK wrapper, assertions, stack/heap checks and watchdog. The complete 120-second screen10 records 0/84,943 ADC CRC rejections and 65.59/65.62/44.16 FPS capture/USB/BLE: 7,873 internally consecutive USB frames and 5,298 BLE frames with 2,572 missing observations. Status remains **FAIL_BLE_THROUGHPUT**. The 2.5% CRC-rate criterion is unchanged and the 600-second/32-configuration suite was unrun at screen10 closure; original FW01–FW10 numbers and failures remain. [完整双语记录 / Full bilingual record](cdc_recovery_acceptance_20261116.md).

> 后续进度 / Subsequent work: 同一 `a656d53` 的独立 `stability-01` USB+BLE 600 秒连续 CRC/健康诊断已开始，结果待闭合；它不是新优化候选，也不覆盖 screen10 的 BLE 不足。活动文件未被本次文档/归档读取或压缩。 / An independent 600-second USB+BLE CRC/health run, `stability-01`, has started on the same `a656d53`; results await closure. It is not a new optimization candidate and does not erase screen10's BLE deficit. This documentation/archive task has neither read nor compressed its active files.

## 2026-11-16 保留检查的 `-O2` 与 screen10 / `-O2` with retained checks and screen10

首个 `compiler-perf-01` 候选因配置切换隐式取消 `FREERTOS_TASK_FUNCTION_WRAPPER` 被拒绝，**未烧录**；此前可构建和 host 测试通过不能覆盖这个范围失败。ESP-IDF 5.5.5 原 wrapper 依赖 DEBUG/core-dump/GDB，负责任务错误返回时 log/abort 与初始 task frame PC。`a656d53` 在 `main/kconfigs/Kconfig.compat` 仅扩展同一 SDK symbol 的 S3+PERF 条件，main guard 在缺少 wrapper 时拒绝构建；没有修改 SDK、替代 abort、开启调试器或关闭 watchdog。

The initial compiler-perf-01 candidate was rejected because the optimization switch implicitly removed `FREERTOS_TASK_FUNCTION_WRAPPER`; **it was never flashed**. A successful build and host tests do not override that scope failure. ESP-IDF 5.5.5 gates the wrapper on DEBUG/core-dump/GDB; it provides log/abort on erroneous task return and the initial task-frame PC. `a656d53` extends only the same SDK symbol's S3+PERF condition in `main/kconfigs/Kconfig.compat`, with a main compile guard rejecting a missing wrapper. It does not edit the SDK, replace abort, enable a debugger or disable the watchdog.

| 证据 / Evidence | 实施与验证 / Implementation and verification | 边界 / Limit |
|---|---|---|
| [拒绝候选](artifacts/cdc_recovery_acceptance_20261116/compiler-perf-01-scope-rejected.json) | REJECTED_DO_NOT_FLASH; hardware_flash_attempted=false | 失败记录保留，不归入烧录履历 / Preserved rejection, not a flashed build |
| [真实 Kconfig/guard 审查](artifacts/cdc_recovery_acceptance_20261116/performance-wrapper-independent-review-01.json) | 18 个边界、6 个编译用例，3,769 symbol 中只改 wrapper / Only wrapper changes in full-tree comparison | 不泛化到未来 SDK / Recheck future SDKs |
| [最终配置范围](artifacts/cdc_recovery_acceptance_20261116/compiler-perf-02-scope.json) | `-O2`，assert/strong stack/heap poison/task WDT/watchpoint 保留；160 MHz / Checks retained at 160 MHz | 未修改协议、队列、CRC 门槛或工作预算 / Unchanged protocol, queues, CRC criterion and budgets |
| [最终 ELF](artifacts/cdc_recovery_acceptance_20261116/compiler-perf-02-wrapper-elf-review.json) | task call→返回 log→abort；初始 stack PC 引用真实 wrapper / Linked wrapper and initial PC verified | 实机 return-fault 注入 NOT_RUN / Hardware return-fault injection unrun |
| [67 测试与构建身份](artifacts/cdc_recovery_acceptance_20261116/compiler-perf-02-validated-identity.json) | 67/67 exit0、实际目标构建 exit0、185 输入一致 / Matching production inputs | 编译/host 不是 HIL 或精度通过 / Not HIL/accuracy acceptance |

已烧录完整提交 `a656d53f2efc447507f5247a116a22aed35dcc04`，ELF SHA256=`63ab97c31a1c8c7ca92e631661e42ad7e6357e0475a38f2cc5270959973c36f4`，app binary SHA256=`dc6c74800d88760a8ba85274b6954317f6a95d019bf2d4883bea7cdf406a3dd6`，1,591,888 B。运行 [BUILD/BOOT/READY](artifacts/cdc_recovery_acceptance_20261116/compiler-perf-02-boot-verification.json) 核验 session=`B528ED81F93297CF`、bootId=1、CAP8；CPU owner/runtime/trace ON、ADC GPIO tail required/2 MHz，正常 defaults 诊断 OFF。链接保留 wrapper 不证明所有 panic/restart 路径已实机故障注入。

The flashed full commit is `a656d53f2efc447507f5247a116a22aed35dcc04`, ELF SHA256 `63ab97c31a1c8c7ca92e631661e42ad7e6357e0475a38f2cc5270959973c36f4`, app binary SHA256 `dc6c74800d88760a8ba85274b6954317f6a95d019bf2d4883bea7cdf406a3dd6`, size 1,591,888 bytes. Running [BUILD/BOOT/READY](artifacts/cdc_recovery_acceptance_20261116/compiler-perf-02-boot-verification.json) verifies session `B528ED81F93297CF`, bootId 1 and CAP8. CPU owner/runtime/trace are ON; ADC uses GPIO-tail-required at 2 MHz, while normal diagnostic defaults remain OFF. A linked wrapper is not hardware fault-injection evidence for every panic/restart path.

screen10 的 120.000122 秒窗口实测 Capture/USB/BLE **65.594513/65.616123/44.162295 FPS**、ADC **0/84,943**（119.069615 秒独立健康跨度）。USB 7,873 有序可信帧覆盖 seq1331…9203、progress7872、gap=[]，证明内部连续，未用缺少 `missing_frames` 键补零。BLE 5,298 帧缺 2,572 个序号观察，独立 BL50 窗 `dropD+2533`；收到分片完整也不能称没有整帧背压。较 screen09 BLE 提高 **12.848988%**，但采集亦提高、BLE 仍明显落后，结果保持 **FAIL_BLE_THROUGHPUT**。

Screen10's 120.000122-second window measures **65.594513/65.616123/44.162295 FPS capture/USB/BLE**, with **0/84,943 ADC CRC rejections** in a separate 119.069615-second health span. USB's 7,873 ordered trusted frames cover seq1331…9203, progress7872 and gap=[], proving internal contiguity rather than defaulting an absent missing_frames key to zero. BLE receives 5,298 frames with 2,572 missing observations; its independent BL50 span reports dropD+2533. Intact received fragments do not remove whole-frame backpressure. BLE improves **12.848988%** over screen09, while capture rises too and BLE still trails substantially. The verdict remains **FAIL_BLE_THROUGHPUT**.

USB S1D2 740 个 X12、BLE 465 个 X12 继续无效；未观测 MFAULT/WDT/reset，CAP8 cleanup 已查询确认。原始计数窗口和 [09→10 CPU/分层对照](artifacts/cdc_recovery_acceptance_20261116/screen-10/screen-09-10-layer-cpu-comparison-01.json)保留未知项，不跨 boot 相减、不把角色比例合计为全核 CPU、不用 SDK wall−owner run 推 blocked。完整 [screen10 verdict](artifacts/cdc_recovery_acceptance_20261116/screen-10/screen-10-verdict.json)不是 600 秒/32 配置或绝对精度验收。

USB S1D2's 740 X12 points and BLE's 465 X12 points remain invalid. No MFAULT/WDT/reset was observed and CAP8 cleanup was query-verified. Original windows and the [09→10 CPU/layer comparison](artifacts/cdc_recovery_acceptance_20261116/screen-10/screen-09-10-layer-cpu-comparison-01.json) retain unknowns: no cross-boot cumulative subtraction, summed whole-core CPU ratio or SDK-wall-minus-owner-run blocked estimate. The complete [screen10 verdict](artifacts/cdc_recovery_acceptance_20261116/screen-10/screen-10-verdict.json) is not 600-second/32-configuration or absolute-accuracy acceptance.

compiler-perf 与 screen10 共 13 个已关闭原始流完成 [gzip 归档及逐字节/SHA 回验](artifacts/cdc_recovery_acceptance_20261116/screen-10/archive-index-01.json)，21,033,025 B→2,490,426 B，全部原件保留；活动的后续稳定性流不在该索引内。完整中英文配置、回归与性能表见[阶段报告](cdc_recovery_acceptance_20261116.md)。原 FW01–FW10 编号和所有先前失败记录保持。

Thirteen closed compiler-perf/screen10 streams passed [gzip byte/SHA roundtrip verification](artifacts/cdc_recovery_acceptance_20261116/screen-10/archive-index-01.json), from 21,033,025 to 2,490,426 bytes, with all originals retained. Subsequent active stability recordings are excluded. Full bilingual configuration/regression/performance tables are in the [stage report](cdc_recovery_acceptance_20261116.md). Original FW01–FW10 numbers and every earlier failure remain.

## 2026-11-16 元数据解析性能增量 / Metadata-parsing performance increment

`dafaf9bc841e756b7d78aa54391703a952b819e5` 仅将 `sensorarrayTransportParseFrameMeta` 从每字段重复扫描改成首行一次 token 遍历，使用固定局部值和 seen/valid mask。该同步调用位于 AsyncLog/output publisher 入队路径；它不属于 BLE worker 的重复发送解析。仍只读取独占 slot 中的不可变 payload，不重读全局配置，不合并 C/V/R/M 的 legacy gen/rid 域，不改 encoder/CRC、preview、队列容量、优先级或 stdout 日志。继承上一镜像的 ADS 分配生命周期 P1 修复；该性能修改不冒充新的安全缺陷修复。

`dafaf9bc841e756b7d78aa54391703a952b819e5` changes only `sensorarrayTransportParseFrameMeta`, replacing per-field rescans with one first-line token traversal, fixed local values and seen/valid masks. This synchronous call runs in the AsyncLog/output publisher enqueue path, not repeated BLE-worker parsing. It still reads only the exclusive slot's immutable payload, never current globals, and preserves C/V/R/M legacy gen/rid domains, encoder/CRC, preview, queue capacity, priorities and stdout logging. The preceding ADS allocation-lifetime P1 repair remains included; this optimization is not labeled a new safety-defect repair.

| 证据—函数 / Evidence and function | 修改 / Change | 实际验证与限制 / Validation and limits |
|---|---|---|
| 旧 parser 最多 14 次 header 字段扫描；`sensorarrayTransportParseFrameMeta` / Up to 14 repeated field scans | 单次 token 遍历，first-nonempty、数值错误和身份域语义保持 / One pass, preserving duplicate, numeric and identity semantics | 66,896 个整 slot 差分、20 帧原 bytes/CRC、ASan/UBSan 通过；LSan 环境受限 NOT_RUN / Differential and address/UB checks pass; leak check unrun |
| screen08 ASYNC_LOG 独立窗口比例 24.600562% / Own-window ratio | 不关日志、不提高超时、不限科学 FPS / No disabled logs, relaxed timeout or science-FPS cap | screen09 13.494264%；不同角色窗口不可合计，未隔离全部 SDK/ISR 时间 / Not a summed CPU table or full SDK/ISR attribution |
| BLE 33.118267 FPS，低于 capture / Receipt below capture | 发布者元数据解析开销减少 / Reduced publisher metadata work | screen09 39.133976 FPS，+18.164321%，仍缺 2,223 个科学序号观察、整体 FAIL / Improved but still unaccepted |

[生产回归与原 stdout](artifacts/cdc_recovery_acceptance_20261116/transport-meta-single-pass-host-01.json)保留首次 LeakSanitizer/ptrace 环境失败；关闭 leak 检查后 ASan/UBSan 的同一 66,896 语料通过，不能称 LSan 通过。完整 66 个 host 入口与目标构建使用相同 185 个生产输入；[运行身份](artifacts/cdc_recovery_acceptance_20261116/frame-meta-01-boot-verification.json)的完整 ELF SHA256 为 `922e1c880317d4ace18eb2ec2b368ad7d3b9a1ac1c548f7e34a63182ffeb2c10`，session=`48E77595DEBD273B`，与 app binary SHA 区分。CPU profiling ON、160 MHz、`-Og` 和 2.5% CRC 错误率验收门槛均未变。后续 `-O2` 候选未计入此处。

The [production regressions and original stdout](artifacts/cdc_recovery_acceptance_20261116/transport-meta-single-pass-host-01.json) retain the initial LeakSanitizer/ptrace environment failure. With leak detection disabled, address/undefined-behavior checks passed the same 66,896-case corpus; this is not an LSan pass. All 66 host entries and the target build share 185 production inputs. The [running identity](artifacts/cdc_recovery_acceptance_20261116/frame-meta-01-boot-verification.json) verifies full ELF SHA256 `922e1c880317d4ace18eb2ec2b368ad7d3b9a1ac1c548f7e34a63182ffeb2c10`, session `48E77595DEBD273B`, separately from the app binary hash. CPU profiling ON, 160 MHz, `-Og` and the 2.5% CRC-rate acceptance criterion remain unchanged. A later `-O2` candidate is not included here.

[screen09 判定](artifacts/cdc_recovery_acceptance_20261116/screen-09/screen-09-verdict.json)记录 ADC 0/74,721（118.765452 秒独立健康窗口）、USB 6,920 有序帧 seq1600…8519 内部连续、BLE 4,695 帧/2,223 缺口、USB S1D2 467 个 X12 保留无效；未观测 MFAULT/WDT/reset，CAP8 cleanup 查询完成。零缺口由首末序号、有序数量及空 gap 列表证明，非缺少键时补零。USBQ、BL50、BLETX、BLESVC 与 CPU 的独立分母保存在 [08→09 对照](artifacts/cdc_recovery_acceptance_20261116/screen-09/screen-08-09-layer-cpu-comparison-01.json)。科学绝对精度与完整 600 秒/32 配置仍未验收，不把 SDK wall−owner runtime 称为 blocked，也不把本次收益称为全部剩余根因已解决。

The [screen09 verdict](artifacts/cdc_recovery_acceptance_20261116/screen-09/screen-09-verdict.json) records 0/74,721 ADC CRC rejections in a separate 118.765452-second health window, 6,920 internally consecutive USB frames at seq1600…8519, 4,695 BLE frames with 2,223 missing observations, and 467 retained X12 invalids at USB S1D2. No MFAULT/WDT/reset was observed, and CAP8 cleanup was query-verified. Zero internal USB loss is proven by sequence endpoints, ordered count and an empty gap list, not defaulted from a missing key. Independent USBQ, BL50, BLETX, BLESVC and CPU denominators remain in the [08→09 comparison](artifacts/cdc_recovery_acceptance_20261116/screen-09/screen-08-09-layer-cpu-comparison-01.json). Absolute scientific accuracy and complete 600-second/32-configuration acceptance remain pending. SDK wall−owner runtime is not blocked time, and the measured improvement does not resolve every remaining cause.

## 2026-11-16 已复现的诊断内存回归与修复 / Reproduced diagnostic-memory regression

`bb230d0` 虽通过编译和host测试，同板启动却在 `scan_task_create_failed/0x101` 停止，seq=0，SPI扫描尚未启动。新诊断模块的三个内部静态结果槽消耗约9KB，破坏了采集任务16KB内部连续栈的分配余量；总heap含PSRAM，不能证明内部可分配。`8b1ce9f` 将可选诊断槽改为启动时一次性PSRAM分配，无内部fallback；失败只令该诊断不可用。内部占用减少8908B，未缩减采集栈或watchdog。60个host入口和目标构建再次通过，同板已完整BUILD核对并正常CAP8采集；失败镜像和退出下载记录均保留。[失败记录](artifacts/cdc_recovery_acceptance_20261116/timing-startup-failure-01.json)、[修复验证](artifacts/cdc_recovery_acceptance_20261116/timing-storage-01-validated-identity.json)。

Although `bb230d0` compiled and passed host tests, the board stopped at `scan_task_create_failed/0x101`, before SPI qualification or any acquisition. Three new static diagnostic result slots consumed approximately9KB of internal memory, removing allocation headroom for the16KB acquisition stack; total heap included PSRAM and did not establish internal contiguous availability. `8b1ce9f` allocates these optional slots once in PSRAM with no internal fallback. Allocation failure disables only the diagnostic. It saves8908 internal bytes without reducing the acquisition stack or watchdog. All60 host entries and the target build pass again; the same board boots and acquires CAP8 with a verified full BUILD hash. The failed image's evidence remains intact.

Eleven TIMING requests on this image completed with verified restore and unchanged configuration identity: continuous groups rejected9/4400 reads, pulse groups0/4400 and held diagnostic rereads0/4400. This is a frame-boundary shorted-input diagnostic, not the runtime matrix error rate or a causal proof. 三轮120秒筛查累计549条完整坏包均为最低位syndrome01，但正式VOLT运行仍有CRC拒收；第三轮capture66.36/USB66.33/BLE35.68FPS，整体仍FAIL。详情与边界见[本轮双语记录](cdc_recovery_acceptance_20261116.md)。

## 2026-11-16 CRC 根因诊断增量 / CRC diagnostic increment

`bb230d03f438e724436881a7f61af35122726403` 已通过实际 ESP-IDF 构建和60个host测试入口，183个生产输入hash与测试清单逐项一致。它新增诊断能力和BLE服务观测，不是“CRC已修复”或“性能已通过”的声明；原FW01–FW10编号不变。

The `bb230d0` increment passes the actual ESP-IDF build and all60 host test entries, with183 production input hashes matching. It adds diagnostic capability and BLE service observations, not a claim of repaired CRC or accepted performance. Original FW01–FW10 numbering remains unchanged.

| 证据—函数 / Evidence and functions | 修改 / Change | 实际验证与残余风险 / Validation and remaining limits |
|---|---|---|
| 两轮闭合日志405条完整坏包均CRC syndrome01；`sensorarrayAdsTimingProbeRun`（本轮新增） | 独立模块固定CONT/PULSE×PGA1/bypass、200次/组，pulse held reread另计；不改SPI时钟、启动2.5%门限或科学路径 / Separate fixed comparison; held rereads counted separately, unchanged clock/qualification/science | 25个真实driver+module+Gap场景通过；不凭syndrome断言硬件根因 / 25 production-path scenarios pass; syndrome alone does not establish cause |
| pulse STOP不能取消进行中转换；`sensorarrayAdsGapRunTimingProbe`（本轮新增） | 单owner与2s绝对期限含500ms恢复余量；MODE0连续模式verified→STOP→完整快照/driver/cache恢复；首错/terminal/cleanup分开 / One owner and absolute deadline, verified full restore | 未确认DMA回收禁止复用/恢复；下一帧helperGeneration使profile/value cache失效 / Unknown DMA ownership remains quarantined; next-frame cache invalidation verified by call-chain review |
| 多transport接受/终态可能先后到达；`sensorarrayAdsTimingControl.[ch]`（本轮新增）、main边界接入 | 复用单槽全局配置仲裁、ACK栅栏、10s排队期限、不可变last-result和按id查询 / Shared arbitration, ACK fence, bounded queue wait and immutable result queries | 13组＋10准入子例及4既有入口通过，未假称终态绝对送达 / Actual C tests pass; local terminal retention is not guaranteed PC receipt |
| screen02 BLE35.35FPS低于capture68.05；WorkBudget/SendGattValue/LinkDiagnostics | 新BLESVC?/SCHEDWHY?仅观察SDK墙钟/真实idle与可重叠yield条件，不改调度策略 / Observation only, unchanged scheduling | 10组旧/新调度模型输出一致；尚须新HIL确定服务差额根因 / Model outputs match; throughput root cause still requires hardware evidence |

详细命令、退出码、source SHA、完整坏包与后续HIL见[当前阶段](cdc_recovery_acceptance_20261116.md)。 / Exact commands, exit codes, source hashes, packet evidence and subsequent HIL are in the current-stage report.

> 2026-11-14 的烧录受阻记录为历史，见 [原始阶段](usb_otg_ble_priority_20261114.md)；失败与原始数据未被覆盖。 / The earlier programming blockage and original evidence remain preserved.

> **2026-11-13 当前负载修复（待验收）**：USB/CPU0 工作预算及 RECOVER/RESTART 的已确认 P0/P1 修复已提交为 `fd0190a`；最终已提交目标构建 PASS，50 个 host 入口全部 exit 0，输入 hash 与构建逐项相符。WorkBudget 的 Xtensa/include 适配已完成，repair-01/02 失败历史保留。app-only 烧录/启动观察由主任务执行，新镜像 HIL 待验证。完整中英文机制、证据和剩余边界见 [负载可靠性修复](load_reliability_fix_20261113.md)。下文历史版本/实测不被改写。
>
> **Current load repair, 2026-11-13 — acceptance pending:** the confirmed USB/CPU0-budget and RECOVER/RESTART P0/P1 fixes are committed as `fd0190a`. The final committed-source target build passes and all 50 host entries exit 0, with input hashes matching the build. Xtensa/include adaptation is complete; repair-01/02 failures remain preserved. The main task owns app-only flash/startup observation, and new-image HIL validation is pending. See the [complete bilingual report](load_reliability_fix_20261113.md); historical results below remain unchanged.

> 2026-11-13 启动选速率/恢复更新：已实现独立前端启动模块、SPI 与 I²C 并行资格扫描、细档位及各阶段 2.5% CRC 门限、三前端失联的低速探测和受控重启。已实际构建、执行 42 个 host 测试入口、app-only 烧录 `0d8f60f`。旧版 `e339a1d` 的 VOLT 失败实录促成运行期 CRC 分类修复；新版 VOLT1/VOLT8/RES8/CAP8/MIXED8 各约140秒均无MFAULT，50次控制身份切换通过。VOLT1仍有25个实收帧缺口，RES参考点仍有X02，不能宣称无背压或精度通过。**SPI 电气 CRC 错误尚未根治，不能把选速成功当作完整验收通过。** 后续修订及实测见 [启动/恢复验收](frontend_startup_recovery_20261113.md)，其版本/结果优先于下文历史记录。

> 2026-11-13 先前夹具复测（历史）：[接触修复与外部轨对照](reference_fixture_recheck_20261113.md)。S8D8松动修复后两只电阻均不再OPEN；用户提供了轨电压范围，已临时输入RAM。外部rail且周期电池关闭时仍观察到CRC错误，因此完整验收仍未通过。本轮未改/重烧生产固件；新增悬空口近似背景工具按用户授权保留三种方法，不冒充同点精度校准。以下2026-11-05状态为此前记录。

> 上一版交付状态（2026-11-05）：已按用户授权烧录并完成下述实机测试，**完整可靠性与精度验收未通过**。后续发现、失败记录与原始数据见 [参考夹具 HIL 记录](reference_fixture_hil_20261104.md)。本节之后的首次离线审计保留历史状态，其“未烧录 / NOT_RUN / 24 个测试”不能覆盖本节的后续结果。

## 上一版已烧录版本、结果与剩余阻塞（2026-11-05，历史）

最终交付前在已提交的 `ebabe7e` 上再次实际运行全部34个测试入口，全部退出0，见 [已提交源码回归清单](artifacts/reference_fixture/final-committed-test-results.json)。此前 `serial-lines-final` 清单记录提交前的 `bd49168 + dirty`，仍原样保留；此次重跑消除版本对应歧义，不改变已有HIL失败结论。

生产源码 commit 为 `ebabe7e6816c0347c6d1f5187d5f7f801a7ef98d`，BUILD 为 `ebabe7e-dirty`；dirty 是文档/证据，153 个构建源码文件与最终工作树逐 SHA 校验一致。app SHA256 为 `2dfd717ec374c3d046c20316320b3e2199e3eeda22c4dd94d193336a1aad8ba2`，1534288 bytes。实际 ESP-IDF v5.5.5 构建、app-only 烧录和全部 **34 个 `tools/test_*.py` 入口通过**，见 [build](artifacts/reference_fixture/serial-lines-final-build-manifest.json)、[flash](artifacts/reference_fixture/serial-lines-final-flash-result.json)、[测试退出码](artifacts/reference_fixture/serial-lines-final-test-results.json)、[源码复核](artifacts/reference_fixture/final-source-verification.json)。文档提交不改变板上 BUILD。

实机为 ESP32-S3 rev0.2 / Flash16MB / PSRAM8MB、ADS1262 ID03；连接是 USB Serial/JTAG `/dev/ttyACM0`，并完成 Linux BLE 实收。初始板上为 IDF v5.5.1 / PROTO1，不能等同于原审计基线。最终设备 PROTO2，停留 **CAP / 8 行 / USB DEBUG**，CONFIG session `26888A60879294CC` / boot10 / rev95。没有写 NVS/calib、执行 CAL=SAVE、push 或发布。

| 后续实测发现 | 函数与实际修改 | 验证和限制 |
|---|---|---|
| rail 继承末点 PGA bypass，RES 数值随刷新跳变约1100 Ω | `sensorarrayAdsReadRailMonitorUv` 使用手册要求的 PGA1/chop OFF 专用配置，物理核验后采样、完整恢复后发布 | `8d6d656` RES8 139.683 s 未再见该大台阶；最终 MIXED8 139.632 s /807帧无 MFAULT。独立轨电压未知，不能当作 VOLT 因果闭环或精度 PASS |
| RREG 健康回读及辅助 CRC 错误丢失原因 | 合法尾寄存器 guard；`sensorarrayAdsMatrixGetRail` 保留 esp_err_t，原错误/cleanup 独立；rail 整事务共用一次新鲜 CRC 重试 | 健康边界128/256/384/512及累计超过2048有实录；最终 RES 故障明确为非健康回读的 MATRIX_CRC/0x109 |
| MIXED 辅助事务发生于已有 CAP 点之后 | 本轮新增 `sensorarrayMixedRowPrepareRailAtBoundary`，在任意采样前完成 rail，选择真实 RES 行；子段禁止辅助事务 | 8次完整预备约120 ms、100帧间隔；恢复成功且第一错误完整保存。不把局部monitorUs当整个事务耗时 |
| USB VFS 路径超限导致双输出者，修复后仍有 printf 碎片穿插 | 短路径、stdio绑定核验与ready门禁；`sensorarraySerialIo` stdout/stderr分别组完整行后入队，整行丢弃计数；科学帧直接完整packet入队 | 两个历史生产反例均复现后回归；最终50次控制身份和CONFIG核验通过。CAP1并行测试USB仍有78个数据包缺口，与逐层计数相符，不能称无背压 |
| RDATA CRC 末位错误持续 | 保存CRC原字节、有限重试；duty64与RDATA+NOP实机候选均失败，均已撤回，保留所有校验位 | 最终RES8仅26帧/2.075 s后MFAULT，**FAIL**。没有示波器证据，不能断言硬件原因或已根治 |

最终 CAP8 139.867 s /1615帧全部点有效、ASCII CRC/identity/gap为0；33pF两点发布的是总电容，缺各自卸载基线，仍为 BASELINE_REQUIRED。最终 MIXED8 的 S1D1均值9988.021 Ω，S8D8全部OPEN；用户尚未确认后者接线，夹具评估FAIL。严格50切换测试在第29次参考点X02停止，失败原样保留；另一次分项测试50次控制身份通过，不代表精度通过。

BLE140 s实收9131个完整C帧，校验、缺片和序号缺口为0；与Serial共有9049帧、3511654字节完全一致，Serial缺少的78个共同范围数据帧都在BLE中。稳态CAP1 capture约65.37 FPS、BLE提交约65.4 FPS，不冒充用户VOLT1的97.99 FPS。DIRAM链接占用286999/341760，运行heap/stack实测见HIL报告；CPU、最大连续块、Windows重枚举、Wi-Fi端到端、多transport同时变更配置、危险硬件故障注入仍NOT_RUN。

用户暂无独立AVDD/AVSS万用表读数，因此不以内部估算填写外部RAILCFG，不执行无物理隔离前提的rail-on组合。最终CAP电池仍报告rail_invalid，lastGood真实时间保留。还需SPI时序取证、S8D8接线核验、33pF同点基线及误差限，才能继续完成可靠性和精度验收。

全部最终实录、lossless gzip和SHA见 [证据索引](artifacts/reference_fixture/release-evidence-index-final.json)；分析器实际重跑 [退出码](artifacts/reference_fixture/final-analysis-command-results.json) 中，退出0但REVIEW_REQUIRED不等于PASS。精确 [文件清单](artifacts/reference_fixture/firmware-final-changed-files.txt) 和 [函数级diff](artifacts/reference_fixture/firmware-final-function-diff.patch) 对应 baseline→`ebabe7e`。实录黄金帧见 [reference_capture](../../tests/fixtures/reference_capture/)，生成式协议反例仍单独标明 synthetic。

## 首次离线审计记录（ba8346b，历史）

本轮在 `ErrorChen/SensorArray` 工作树修改、编译并执行回归。审计基线与开始时 HEAD 均为 `8045e9e9ec9599533c52c15dfcb6002f79fd15f1`，开始时 clean。固件/测试修复 commit 为 `ba8346b7b643e54985699630f608aba3386810e9`；报告/契约/故障回放由随后的本地文档提交交付。未 push、未发布、未烧录；不得把基线或此修复 commit 当成用户已烧录版本。最终构建输入、二进制 hash、dirty diff hash、命令和退出码见 [验证清单](artifacts/reliability_fix/final-manifest.json)。只有源码/host 回归已验证；E1 的现场因果及物理性能验收仍未完成。

## 环境、材料和证据边界

- 已读取 README、架构/协议/模式/命令/验证文档、实际 main/、Kconfig、sdkconfig/defaults、`datasheets/circuit.pdf` 和本地 ADS126x/ESP32-S3 手册。仓库及父目录未找到 AGENTS.md。
- 实际工具链 ESP-IDF **v5.5.5**（README 的 5.5.1 不是本次构建版本）；target ESP32-S3；配置 Flash 16 MB、DIO、80 MHz，octal PSRAM 80 MHz、type AUTO。图纸模块标 N16R8；**实际芯片 ID、Flash/PSRAM 容量、设备 BUILD/PROTO 均 NOT_RUN**。
- sdkconfig：`CONFIG_SENSORARRAY_ADS_REGISTER_HEALTH_INTERVAL=64`、rail max age=100 frames、FreeRTOS 100 Hz、USB Serial/JTAG console；PM 未启用；5 s 的现有双核 idle TWDT 保留，未关闭 watchdog。实际编译/分区参数见 build 日志；没有调用 flash。
- 没有可访问串口设备、Windows、DMM、示波器或负载夹具。指定四个“粘贴的文本”文件和截图未在仓库找到。E1–E7 只能引用用户给出的摘述，不能补成完整原始帧。本轮所有新夹具注明 synthetic，未覆盖科学数据。
- 上位机仓库未修改，且本环境没有其可读 checkout；主机交接按真实固件 encoder 制定，不假定现有 Host 已支持。

| 现场事实 | 能支持的判断 | 本轮源码/回归结论与限制 |
|---|---|---|
| E1 seq 1809/1910/2011/2112/2213，101 帧间隔，共模 18–19 mV | rail 刷新是强关联假设，非完整因果 | age >100 的触发条件与 101 帧相容；删除本板无法隔离的透明 CAP/VOLT INTREF 激励。没有逐帧实采对照，不能宣称测得尖峰消失或固定五秒周期 |
| E2/E3 512/128 个模式区间帧后 0x108；attempts=0 | 优先检查周期健康回读；不能把 host seq 当 engine count | SetMode 不重置累计 engine frameCount；实际 interval=64。原健康路径把不同错误折叠成 INVALID_RESPONSE，已保留首错并加上下文；0x108 是 INVALID_RESPONSE，0x107 才是 TIMEOUT |
| E4 unstable / rail_invalid，lastGood=UINT_MAX | 电池有效性、rail 依赖、恢复可信度是不同条件 | 分别诊断原始三样本/配置/恢复；阈值未放宽；unknown 显式可辨；外部 rails 可解除 CAP 的校准依赖，内部不可强开参考 |
| E5 CAP 持续而主机 FRAME_DROP、gen=2/rid=1 | 模式权威与数据身份不一致，不限于 MAPP 丢包 | 证实 C 的 legacy gen/rid 属 ROWS，V/R 属 MODE；新增显式域和原子 CONFIG?，保留旧含义 |
| E6 COM8 打开 FileNotFoundError | 仅该次端口名不可打开 | 软件改为单 USB owner 和有界 TX，不能据此排除电气/OS/ROM 下载模式问题 |
| E7 capture≈97.99、BLE≈65.3、dropD 累计24 | 采集和输出容量不同；需要计数增量 | 保留异步采集/有界输出，增加 USB 分层观察，未实测新 FPS；旧累计287不是逐帧实时丢包证明 |

## 已核实 symbol map 与首错定位

原请求中的主要函数在当前 HEAD 仍同名。实际入口及映射如下；本轮新增接口在源码注释及对应说明标识：

| 链/职责 | 实际文件与函数 |
|---|---|
| ADS 健康首错 | `core/measure/ads/sensorarrayAdsMatrix.c`: EngineReadFrame → RefreshRegisterShadow / NoteRegisterSnapshot；`sensorarrayAdsCache.c`: 健康比较策略 |
| rail 与模式调度 | `main/main.c`: RefreshVoltageRailIfNeeded、ApplyMeasurementMode、RunOneFrame、AcquisitionTask；BoardMapGetSafeRailMonitorProfile → RouteControllerEnterSafeRailMonitor → AdsGapRefreshRailAtBoundary |
| MFAULT | `main/main.c`: RunOneFrame 与采集主循环错误分支；EmitAdsFault 在 SAFE 前的 engine context 中取根因；不把最终 NONE/SAFE 当原模式 |
| 电池/ADSCHK/校准 | `sensorarrayAdsGap.c`: ReadBatteryTransaction、RestoreBatteryCoreState、RunBatteryAtBoundary、RunActiveCheck；新增 RunPendingAtBoundary 和 BindBootIdentity；旧 GapTryRun 不再做行间硬件事务 |
| FDC 双 worker | `sensorarrayFdcRowEpoch.c`: MeasureFdcWorkerTask / ReadFdcMatrixRowParallelEpoch；新增 WorkerLifecycle、WorkersQuiescent/Quarantined |
| identity | `sensorarrayScanConfig.c`、MeasurementMode/RowModeProfile、main RunOneFrame；新增 `sensorarrayConfigIdentity.[ch]` |
| 真实 encoder | `main/output/sensorarrayTextProtocol.c`；FrameBuilder 初始化后恢复采样前 identity；transport 新 ParseFrameMeta 只读 payload |
| Serial/log sink | `sensorarrayTransport.c`: SerialControlTask / Init / ProcessDescriptor / DrainGuaranteedText；新增 `sensorarraySerialIo.[ch]`；UsbSink、AsyncLog 的 VFS/协议输出进入唯一 TX owner |
| 有界重启 | `main/main.c`: 实际名称 `sensorarrayBootPrepareRestart`、新增 MarkStableProgress；不是假设的 BreadcrumbPrepareRestart |

健康诊断保存 `originalError`，SAFE/restore 的 `cleanupError` 独立；阶段包括 REGISTER_HEALTH_READ、POWER_COMPARE、REFMUX_COMPARE、MODE_COMPARE、RESET_OBSERVED、ROW_GPIO_COMPARE、OWNER_ACQUIRE、RESTORE_VERIFY。POWER 的 RESET 语义单列，比较 mask 排除保留位；SPI 失败、双读不一致、稳定但不符、缓存落后和复位各有条件，不忽略全部 mismatch。context 含 engineFrameCount、modeEntryFrameCount、interval/due、precisionFrame、owner、cache/physical/config generation、expected/actual/mask、辅助事务 ID/起止/恢复后首个有效帧时间。

ADSFAULT 分成有界多行，每行 <384 bytes。railSpanUv/avddUv/avssUv/source 不再混名；未知 boot 明确 unknown，main 绑定真实 breadcrumb。attempted/acquired=false 与 frame mask 表示未开始转换，不能把初始化的64个无效点报告成64次 ADC 转换失败。详细字段及错误码见 [主机契约](../integration/firmware_host_contract.md)。

## 编号对应、修改与验证

原始 FW01–FW10 完整审计编号表未随材料提供。本报告保留用户明确的 FW01、FW02、FW10 对应，不擅自给未定义的 FW03 或 FW04–FW09 分配历史标题。FW04–FW09 以下按本次 prompt 的公共问题分项列出，不覆盖旧编号。

| 编号/本轮项 | 根因和修改 | 实际验证 | 残余风险 |
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
| 新增 MFAULT | 健康分支折叠原错误；SAFE覆盖原mode；rail字段混用 | 1…512累计计数、interval64/128、模式从65累计进入、128/256/384/512、各比较分支、fault formatter极值 PASS | 无现场原SPI波形，不能认定E2/E3根因必为周期健康检查 |
| 新增 辅助恢复/电池 | reference重新启动未统一等待；helper驱动/缓存不一致；失效电池与恢复失败混在一起 | 实际OFF→ON启动50ms reference gate；不每点重等；helper保存/恢复/验证/shadow/generation；原三样本、退避≤30s，阈值不改；PASS | 50ms来自1µF手册条件，不能证明任意实装输入建立；电池噪声与AINCOM实际偏置HIL待测 |
| 新增 配置身份 | C.gen/rid为ROWS，V/R为MODE；发送/初始化可丢身份；终态不完整 | PROTO2显式m/r/p域+session/rev/boundary，CONFIG?原子；共用请求仲裁、ACK后可见、保留终态、失败不写成功ID；真实encoder与5反例回放/50状态切换 PASS | Host必须适配2304-byte重组及新能力；不承诺网络exactly-once |
| 新增 USB | fgets/VFS/多个输出者混杂；stdio成功非PC送达；无主机可阻塞其它transport | 单Serial/JTAG reader与TX owner，20ms packet预算、优先队列、非阻塞VFS、分层统计、截断后分隔符恢复；实际模块模拟 PASS | 无Windows/USB枚举/信号完整性证据；SOF不是COM存在或PC已读 |

## 模拟连接、安全降级及辅助事务

完整连接真值表、HW01–HW06 的拟议 BOM/电气限值/漏电寄生影响和测量步骤见 [硬件整改记录](../hardware/reliability_revision_notes.md)。重点是 U5 EN 由 R25 上拉，D接REF，Q1只是并联对地钳位；GPIO回读不能测得断路。启动未知INTREF时先保持Q1释放，不能同时启用内部参考和钳位。RES明确允许激励；本板不能悄悄借同一通道完成透明CAP/VOLT rail测量。

当前限制是关闭**无隔离的透明内部监测能力**，不是永久禁用VOLT/RES。CAP、1–8行、逐行ROWMODES、双FDC并行、ADS cache/shadow/自动PGA、Serial/BLE/Wi-Fi、校准/baseline/offset和电池调度保留。VOLT使用 `RAILCFG` 提供的独立AVDD/AVSS实测校准；CAP电池缺rail时报告unavailable并有界重探，合法外部校准可解除依赖。内部rail只有在已明确激励的RES状态或未来真实隔离板型中才可执行。不存在安全隔离的rail-on对照不得在接科学输入的设备上运行。

完整辅助事务由同一ADS owner覆盖保存→执行→恢复→验证→reference/input/filter/freshness门控→允许新帧。reference OFF→ON才重新等待；helper generation变化会使profile/value cache失效并触发保守转换路径。恢复前转换不进入PGA/cache；恢复失败不report_continue。`monitorUs` 只是局部阶段，TXN start/complete与firstValidFrameUs说明完整事务和恢复后数据间隔。`railSpanUv` 实测不等于AVDD/AVSS独立实测；AINCOM为估算偏置，不把AIN9叫GND。没有提高spread阈值、用零替无效点或平滑删除尖峰。

## 锁顺序与所有权

```text
Core0 Serial/BLE/Wi-Fi control
  -> mailbox/config admission（短临界区，统一请求ID）
  -> reserved ACK/terminal slot；硬件命令只排队
Core1 acquisition / frame boundary（mode/profile/route唯一writer）
  -> ADS logical owner -> 全事务deadline -> SPI mutex -> 持久IDF descriptor
  -> 两个FDC持久slot -> 各自I²C锁 -> release/DONE或QUARANTINED
  -> ConfigAuthority短临界区（只复制，不调硬件/网络/printf）
  -> bounded async frame/event queues
Core0 transport/output -> bounded per-sink queues
  -> sole serialTx -> USB driver（20ms包预算）
```

回调从config锁内取出后在锁外执行；recovery读取authority在recovery mux外。版本号不是多writer互斥；mode/profile/route writer保持Core1，初始化在启动前。FDC通知仅唤醒，不授予job结果所有权。SPI超时不归还描述符。日志在SPI锁释放后入有界event ring；丢日志可计数，不把本地排队叫PC确认。

## 验证、性能和未运行项目

实际命令、退出码和逐项输出位于 [test-results.json](artifacts/reliability_fix/test-results.json)、[build日志](artifacts/reliability_fix/final-build.txt)、[baseline](artifacts/reliability_fix/baseline.json)。先编译clean基线并运行既有Python测试（148项通过），再修改；最终对全部24个 `tools/test_*.py` 逐入口实际执行（均退出0）；另4个检查脚本与git diff --check均通过，包括会编译生产C的脚本。`test_sensorarray_hil.py` 是HIL工具的单元测试，**不是实机HIL**。

新测试重点：生产ADS driver、健康比较和battery policy、真实encoder/config formatter、production FDC worker/coordinator + ASan/UBSan、SerialIo、RouteController、重启预算/恢复身份、主调度配置提交。CRC黄金帧不手工造CRC；[config_identity夹具](../../tests/fixtures/config_identity/)由真实encoder生成并由zlib独立复核；[ADS故障夹具](../../tests/fixtures/ads_fault/)由真实formatter生成，诊断文本不冒充带CRC测量帧。

迭代期间曾发现StartAdc1缺reference gate、未消费JOB即quarantine未release等失败，已修后重跑；部分既有脚本默认找clang失败，明确指定 `CC=/usr/bin/cc` 后真实编译执行。最终失败项以test-results为准，不把环境缺工具记成HIL通过。补充配置表审计首次返回1：把诊断字符串/测试日志当宏，且未识别SDK的SPIRAM定义；原日志保留为audit_config_surface-before.txt。检查器现读取实际IDF Kconfig并区分代码和字符串/注释，真实未定义宏仍由回归证明会失败。unused/compat列表仍保留，退出0不表示所有配置已清理。

| 场景 | Host/模型 | 实机状态 |
|---|---|---|
| 128/256/384/512健康边界、切模式前累计 | PASS | NOT_RUN |
| CAP/VOLT/RES/mixed，1/8行，50模式状态切换 | encoder/状态机PASS | 2分钟每配置及失败帧数覆盖NOT_RUN |
| rail关+电池开、合法外部校准 | 策略/恢复mock PASS | NOT_RUN |
| rail开+电池关；两者开 | capability拒绝/既有RES策略验证 | 本板透明监测不安全，NOT_RUN；须真实隔离后才准测 |
| 两者关、合法外部校准 | 策略/encoder PASS | NOT_RUN |
| 终态丢失/迟于帧/旧终态迟到/重复CAP/ROWS域反例 | synthetic replay PASS | 多transport并发及链路实际丢包NOT_RUN |
| SPI timeout/迟到DMA；FDC旧epoch/取消/阻塞 | production source mocks/sanitizers PASS | 电气注入NOT_RUN |
| USB无人读取/部分写/恢复 | production SerialIo模拟PASS | Windows重枚举/插拔NOT_RUN |
| BLE高负载、采集与输出背压 | bounded pool/metadata PASS | FPS、接收完成和增量drop NOT_RUN |

另外在 `/tmp/sensorarray-clean-baseline-reliability` 从git archive重建了同一clean基线（未切换或覆盖当前工作树），确认基线DIRAM 313511/341760 bytes、二进制0x16ec80。中途修复版本DIRAM达到340407（99.6%），因此把USB普通文本缓冲、日志格式化workspace和帧副本池改为启动时一次性PSRAM优先分配，失败可内部fallback或明确报NO_MEM；锁、状态、优先控制队列、worker与DMA仍在内部。没有热路径分配。不能把链接余量当实际free heap；真实heap还包含启动回收区并受BLE/Wi-Fi/任务分配影响。最终DIRAM为286791/341760 bytes（83.92%，链接余量54969），比基线少26720 bytes；固件bin为0x174bd0（1526736 bytes），比基线增加24400 bytes（约1.62%）。CPU-only三个池优先PSRAM共53664 bytes；若fallback内部则不享有该运行内存收益。完整size见manifest/size日志。没有基线或修复后硬件 Capture、Emitted、driver完成、CPU利用率、最小heap/最大连续块、task栈水位、恢复时间数据，全部NOT_RUN，不提供虚假性能对照。保留采样性能路径，不永久限到5FPS。USB旧 `usbSent/usbBytes` 明确 `usbStage=queued`，USBSTREAM?用bufferMemory（0未知/未分配、1内部、2PSRAM）及bufferBytes披露普通缓冲实际位置，另外提供driverAcceptedBytes、partialWrite、timeout、rxBytes、lastRx/Tx和TX栈水位；driverDrained/SOF均不证明PC收到。需要同一session测窗口差分，不能直接比较累计drop。

## 交付与剩余验收

- [主机契约](../integration/firmware_host_contract.md)：字段域、PROTO2、CRC、原子CONFIG查询、终态、session/回绕、rail/battery与三transport语义。软件仓库独立任务应按此适配。
- [硬件整改记录](../hardware/reliability_revision_notes.md)：HARDWARE_CHANGE_REQUIRED，未画假已实施PCB补丁。
- [文件清单](artifacts/reliability_fix/changed-files.txt)和[函数diff](artifacts/reliability_fix/function-diff.txt)记录准确修改位置；各子系统进一步证据见 `ads_notes.md`、`fdc_usb_notes.md`、`config_identity_notes.md`。

仍需具备安全接线及外部校准的实机执行两分钟筛查、覆盖原失败帧数、保存raw ADC/physical voltage/逐帧rail及事件，测恢复首帧与模拟节点；需要Windows确认USB枚举。当前交付证明本地编译、软件状态/期限/对象生命周期与协议回归通过，**没有证明现场尖峰、电池物理噪声、USB电气问题或全部性能指标已经消失**。硬件阻塞和未提供原日志均保留，不以暂时有图作为完成标准。

## FW02/FW10 增量：末位采样 / Final-bit sampling increment


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


## FW02 增量：初始化部分分配失败 / Partial allocation failure during initialization

独立审查复现了既有P1：`ads126xAdcAllocSpiBuffers` 的DMA分配失败后，普通8BIT fallback仅一块成功时，函数释放该块却把已释放指针留在handle中；`ads126xAdcInit` 的错误收尾随后调用 `ads126xAdcDeinit` 再释放一次。合成反例真实执行旧生产C，退出23，`duplicateFreeCalls=1,queued=0`；这不是已观察到的板上CRC根因，也不是末位修复引入的错误。

An independent synthetic reproduction confirms an existing P1 in `ads126xAdcAllocSpiBuffers`: after DMA allocation fails, a partially successful8BIT fallback is freed without clearing the published handle pointer. Initialization then calls `ads126xAdcDeinit`, freeing that pointer again. The old production C exits23 with one duplicate free and zero queued SPI transactions. This is neither an established cause of the board's CRC failures nor a regression introduced by final-bit sampling.

修复使用局部tx/rx持有未完成分配，只有完整缓冲对成功后才发布到空handle。失败释放局部资源并保留原始 `ESP_ERR_NO_MEM`；不改在途DMA的隔离、回收或Deinit规则。固定旧harness的独立反例在修后退出0；真实Init/Deinit新增16种分配组合（7成功、9 NO_MEM），两个目标配置共32例通过，无重复释放/泄漏，重复Deinit也不再释放资源。该增量当前尚待本轮完整目标构建和烧录验证，不以host测试冒充实机验证。

The repair retains incomplete allocations in local tx/rx variables and publishes only a complete pair into the empty handle. Failure frees local storage and preserves `ESP_ERR_NO_MEM`; in-flight DMA quarantine, reclamation and deinitialization rules are unchanged. The independent reproduction using a fixed historical harness now exits0. Real Init/Deinit tests cover16 allocation combinations—7 successes and9 allocation failures—in each of two target configurations:32 cases with no duplicate free or leak, including repeated deinitialization. Target build and flashing of this increment are still pending at this entry.

证据 / Evidence: [独立旧/新反例](artifacts/cdc_recovery_acceptance_20261116/finalbit-independent-allocation-negative-02.json), [修后反例命令](artifacts/cdc_recovery_acceptance_20261116/ads-alloc-failure-fixed-01-command.json), [32种目标/分配组合回归](artifacts/cdc_recovery_acceptance_20261116/ads-alloc-regression-01-command.json).


后续实机闭合 / Subsequent target validation: `c670897` 的分配修复与 `98b760c` 的死统计清理已随 BUILD `b9ebe2f-dirty` 构建、烧录。ELF SHA256 `529b440aae12e5cecf2abed938558cc392009e97f929970ea6191ae5bc9b2b8c`；185生产输入与66个通过的host入口清单相同。早期BOOT查询ready0被保留，后续独立查询证明READY1/CAP8，session `FD091D519F388B90`。screen08完整120秒ADC0/76841、无MFAULT/reset观察；BLE仍33.12FPS。AsyncLog自身采样窗口占比24.60%对前轮24.97%，不能据此宣称找到主要CPU瓶颈。原66入口首轮的BLE测试失败亦保留：三个10ms取消超时共用35ms清理预算，宿主调度消耗余量后生产工具正确输出第四个 `session_deadline_expired`；测试现严格验证该终态，并新增确定性总期限耗尽案例，没有放宽生产截止时间。

The allocation repair `c670897` and dead-statistics cleanup `98b760c` are now built and flashed as `b9ebe2f-dirty`, ELF SHA256 above, with185 matching production inputs and66 passing host entries. The early BOOT ready0 observation is preserved; later independent queries verify READY1/CAP8 and the stated session. The full screen08 window observes0/76841 ADC CRC rejections and no MFAULT/reset; BLE remains33.12FPS. AsyncLog's own observation-window share is24.60% versus24.97% previously, which does not establish the main CPU bottleneck. The earlier test failure is retained: three10ms cleanup cancellations share35ms, and host scheduling can consume the remaining margin, correctly producing an explicit fourth session-deadline error. The regression now checks that exact terminal outcome and a deterministic exhausted-budget case; production deadlines were not increased.

[构建/测试身份](artifacts/cdc_recovery_acceptance_20261116/output-trim-02-validated-identity.json) · [启动核验](artifacts/cdc_recovery_acceptance_20261116/output-trim-02-boot-verification.json) · [精确生产函数diff](artifacts/cdc_recovery_acceptance_20261116/output-trim-source-diff-01.json) · [实录解析](artifacts/cdc_recovery_acceptance_20261116/screen-08/screen-08-analysis-01.json) · [CPU差分](artifacts/cdc_recovery_acceptance_20261116/screen-08/screen-08-cpu-owner-deltas-01.json).

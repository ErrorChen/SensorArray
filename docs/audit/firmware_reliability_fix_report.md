# SensorArray 固件可靠性修复报告

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

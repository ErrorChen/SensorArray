# 固件与上位机契约：配置身份与可靠性修复

> 2026-11-13 当前烧录源码为 `0d8f60fd9c29b9ce12e64ed2e6534ee37db41704`，
> BUILD=`0d8f60f-dirty`（dirty 是文档/证据，构建源码 hash 独立保存），app SHA256
> `232ca3976f6b8ea38efd5e09de8279f35c9bfbbd0c605c8dd84e5169ddd1dbaa`。
> 本轮启动模块/扫描主实现 `ee8f972`，FDC ID 地址修正 `e339a1d`，
> 运行期 CRC 局部失效分类 `0d8f60f`。
> [构建](../audit/artifacts/boot_autotune_20261113/final-03-build-manifest.json)、
> [42 个测试入口](../audit/artifacts/boot_autotune_20261113/final-04-tests-manifest.json)、
> [app-only 烧录](../audit/artifacts/boot_autotune_20261113/final-03-flash-result.json)
> 均已实际执行。以下 ebabe7e 是历史版本；本轮实测与限制见
> [启动/恢复验收](../audit/frontend_startup_recovery_20261113.md)。

## 启动扫描与前端恢复扩展（本轮新增）

PROTO 保持 2，新增 `boot_spi_qual_v1/frontend_presence_v1` capabilities；现有数据
encoder、CRC、单位和各配置身份域不变。此扩展不要求 Host 把坏 ADC CRC 当成有效点。

- `SPI?`：在 READY 前也可读取同一个已发布启动结果。成功需要
  `done=1,joined=1,timedOut=0,qualified=1,restored=1,err=0`，不能单看 qualified。
  `hz` 是请求频率，`actualHz` 为 IDF 观测频率（kHz 分辨率换算为 Hz），不是实测
  SCLK 波形；`overlapUs` 是 SPI 工作区间和 I²C 初始化区间的交集。`attempted/samples/crc`
  的 `scope=all_candidates` 包含被淘汰档位，不能据此计算选中档的错误率。
- `SPI_SWEEP`：每档分别记录 screen/confirm、pga1/bypass。默认每个 profile
  200/800 次 wire attempts，最多 5/20 次 CRC 拒绝；坏样本不计 valid、奇偶覆盖或
  rawMin/rawMax。2.5% 是有限启动样本的资格门限，不是长期错误概率保证。
- `FRONTENDS?`：原子探测快照，不触发总线操作。`known/present` 是三位掩码：
  bit0=primary FDC，bit1=secondary FDC，bit2=ADS。`unknown/busy/inhibited` 不是
  物理失联证据；`sdInhibited=1` 也可能是 SD 未知，不能解释为实测高电平。
  `aHz/aRestoreHz` 区分低速探测时钟和恢复后的时钟；原始错误与各 cleanup 分开。
  ADS 每轮以 125 kHz 探测后恢复选定频率；FDC 正常时使用现有频率，SUSPECT/WAIT
  才临时降到不高于 100 kHz。`lowSpeed` 描述此等待期策略，不表示正常轮的 ADS 没有临时降速。
- 首次同轮三芯片 unavailable、已初始化 ADS 的 RESET，或探测恢复失败会进入
  SUSPECT 并停止采集。3 轮连续全失联确认 WAIT；两轮清理成功、ID 稳定且全 present
  后请求受预算约束的 ESP 重启。启动未完成也可进入相同等待，不伪造“三芯片断电”证据。
  正常/SUSPECT 每 1 秒探测，WAIT 每 2 秒探测；每轮有独立有界事务，等待期间可长期
  保持合法 idle，不能由健康任务伪造采集进度。FDC SD 主动关闭不当作全失联。
- hold 期间 MODE/ROWS/ROWMODES 和测量辅助操作被拒绝，已接受的请求有拒绝终态；
  控制接收、查询和显式 RESTART 仍有效。恢复采用整机重启重新调用独立 Startup 模块，
  不复用旧 worker/DMA/DRDY/cache。自动重启预算耗尽后保持可查询等待，需要显式处置。

SPI?/FRONTENDS? 是诊断快照，不是新的帧身份权威；接纳数据仍以 CONFIG 的
session/revision/域计数为准。重启后先重新查询 PROTO/BUILD/CONFIG，再恢复上位机配置。
临时 RAILCFG 随 RAM 丢失，不能自动把旧电源周期的轨校准套到新会话。等待的路由为
UNKNOWN，`physicalSafe=unknown`；GPIO 回读和 ID 恢复均不证明模拟隔离或供电合规。

本轮审计基线为 `8045e9e9ec9599533c52c15dfcb6002f79fd15f1`，首批固件修复为 `ba8346b7b643e54985699630f608aba3386810e9`。**上一版已烧录源码 commit：`ebabe7e6816c0347c6d1f5187d5f7f801a7ef98d`；BUILD 版本：`ebabe7e-dirty`；app SHA256：`2dfd717ec374c3d046c20316320b3e2199e3eeda22c4dd94d193336a1aad8ba2`。** 此处 dirty 为文档/证据改动，源码已提交；准确构建输入、逐文件 hash 和 diff hash 见 [最终构建清单](../audit/artifacts/reference_fixture/serial-lines-final-build-manifest.json)，烧录校验见 [最终烧录结果](../audit/artifacts/reference_fixture/serial-lines-final-flash-result.json)。本文保留该版记录；其对应结果见 [11月4/5日 HIL 记录](../audit/reference_fixture_hil_20261104.md)，当前版本以文首更新为准。

ADC SPI CRC 错误**尚未根治**，NOP 候选已撤回，不能将有限重试、诊断完善或 ASCII 帧 CRC 通过视为 ADC 链路无误。`release-switch50-identity-01` 的 [50 次控制/身份测试](../audit/artifacts/reference_fixture/release-switch50-identity-01.summary.json) 为 PASS，但测量状态是 `NOT_PRECISION_ACCEPTED`，没有写入校准；它不替代或改写此前严格“切换且参考点有效”测试的失败记录。

此协议交给 ErrorChen/SensorArray_Software 的独立任务实现；本轮未修改软件仓库。协议格式来自实际 `main/output/sensorarrayTextProtocol.c` encoder，完整修改及验证记录见 [固件可靠性报告](../audit/firmware_reliability_fix_report.md)。

2026-11-04 实机追加修复 commit **`a0460b8`** 保持 PROTO2 和测量帧字段语义，补充 ADC CRC 有限恢复与诊断。该镜像的构建/烧录/实录见 [夹具 HIL 记录](../audit/reference_fixture_hil_20261104.md)。

## 运行期 CRC 部分帧（0d8f60f）

内部 `boundedCrcFailure` 只许可纯采样 CRC 拒绝的部分帧进入原数据 encoder，不是新 wire 字段；旧帧 `expected/acquired/fresh/valid/error` 和 X02 语义保留。终态与 CONFIG 仍是配置权威，不要求每个点都 CRC 有效。若旧 Host 以全 acquired 门控整帧，它可能继续丢弃合法部分帧；必须交由软件任务调整，固件不会把缺失位改成已采集。

`ADSDISPOSITION` 是诊断事件，`crcRejected/acquired/expected` 为十六进制位图，`allCrcRun/limit=3` 是连续全 CRC 拒绝的 ADS engine 扫段计数，`scope=ads_sweep` 不等于主机混合帧数。`firstError` 保留首错；`abortError` 独立说明导致停止的后续错误或全坏预算耗尽。首错CRC后出现timeout、RESET、配置变化、owner/route故障、SPI/DMA未知时不允许继续。一次合法非CRC测量结论（包括OPEN）可清除此连续计数，CAP或模式切换本身不能清。

## 能力协商与兼容边界

连接后先查询 `PROTO?`、`BUILD?`、`CONFIG?`，保存原始回复。协议版本升为 **2**，新增 `config_identity_v1/config_query_v1/session64` capability。`dataMax` 为 **2304 bytes**，CTRL 保持 512 bytes。固件中的固定容量 slot、BLE 完整 message 和文本 encoder 容量一起调整；USB普通缓冲和日志CPU工作区/帧副本启动时优先PSRAM一次分配，没有在采样热路径动态分配大报文。

旧 `C/V/R/M/MR/K` 字段含义保留；新增字段是扩展。Host 不得将旧 `gen/rid` 统一解释为 MODE。只支持 1536-byte 重组缓冲或要求 K 行 crc 字段位于行尾的旧 Host 必须升级；不能宣称新版所有报文对旧解析器完全兼容。未协商到新能力时，旧帧只能按下表原域处理，不能从 CAP 的低 gen 推断模式回退或帧过期。

## 字段域与 immutable frame identity

| 报文/字段 | 所属域与来源 | 对应终态 |
| --- | --- | --- |
| C `gen/rid`，对应 K `gen/rid` | **ROWS**；`frame.configSnapshot.generation/requestId` | 行数 RAPP |
| V/R `gen/rid`，对应 K `gen/rid` | **MODE**；`frame.measurement.modeGeneration/modeRequestId` | MAPP；同质 ROWMODES 也应用 MODE |
| M `rgen/rrid`，对应 K | **ROWS**；`frame.configSnapshot` | 行数 RAPP |
| M `pgen/prid`，对应 K | **ROWMODES profile**；`frame.rowProfileGeneration/RequestId` | RMAPP；MODE 同步同质 profile |
| MR | 没有独立配置计数；继承本 M/K message 的身份，`s` 为实际 S 行，`m/unit/scale` 属于该行 | 同本 M |
| 全部新 C/V/R/M header 的 `mgen/mrid` | 显式 MODE 域 | MAPP/CONFIG |
| 全部新 C/V/R/M header 的 `rgen/rrid` | 显式 ROWS 域 | 行数 RAPP/CONFIG |
| 全部新 C/V/R/M header 的 `pgen/prid` | 显式 profile 域 | RMAPP/CONFIG |
| 新 header 与 K 的 `session/bootId/rev` | 启动会话、旧启动计数、完整配置 revision | CONFIG、带相同身份的终态 |
| ROWS? `generation/appliedId` | ROWS；pending/requestId 是待处理行请求 | RCMD/RAPP |
| MODE?/STATE? `gen/rid/seq` | MODE；seq 是其应用边界，不是最新成功接收帧 | MAPP |
| ROWMODES? `gen/rid` | profile 域 | RMAPP |
| MAPP `id/gen/seq` | MODE 请求、成功 generation、允许新配置采样的边界 | 终态 |
| RMAPP `id/gen/seq` | profile 请求、成功 generation、当前 profile 的边界 | 终态 |
| 行数 RAPP `id/gen/seq/old/new` | ROWS 请求与边界 | 终态 |
| RAILCFG 的 RAPP `avdd/avss/source` | rail 校准事务；不是 ROWS | 不能用来推进 ROWS generation |
| RECOVER/RESTART 的 RAPP `cmd/level/state` | 恢复生命周期；不是 ROWS/MODE 成功 | 查询 CONFIG 重建 |

布局同质性检查完整的 **8 项配置 profile**，不是仅检查活动行。`ROWMODES=CVRCVRCV` 加 `ROWS=1` 仍走 MIXED engine，输出 `M(profile=CNNNNNNN)`、一个 CAP `MR` 和 `K`；CONFIG 保留完整 `CVRCVRCV`。这里只有活动测点全为 CAP，不能按普通 C 帧或其旧 `gen/rid` 域解析。2026-11-13 的 120 秒实录确认了这一点；首次 benchmark 工具预期 C 帧属于工具误报，原始记录保留。

Layout homogeneity examines the complete **eight-entry configured profile**, not only active rows. `ROWMODES=CVRCVRCV` with `ROWS=1` still uses the MIXED engine and emits `M(profile=CNNNNNNN)`, one CAP `MR`, and `K`; CONFIG retains `CVRCVRCV`. Only the active measurements are all CAP. Do not parse this as an ordinary C frame or assign C's legacy identity domain. A 120-second capture on 2026-11-13 confirms this; the initial benchmark expectation of a C frame was a harness error, whose original evidence is retained.

采样 owner 在每次 engine 调用之前，从已选定的 mode、rows、profile 和 plan 捕获 `sensorarrayConfigIdentity_t`（本轮新增）。engine 对 frame 的清零不会使它丢失：调度器在返回后复制同一局部 snapshot，encoder 和 transport descriptor 仅解析/携带该 frame。发送阶段不得读取最新全局配置盖章。`rev` 在完整配置或安全状态改变时前进；正常逐帧不前进。profile 准备期间采样使用候选 identity，只有成功应用后发布相同 revision 的权威快照；失败帧无效，权威状态保留真实失败状态。

因此现场反例 `CAP gen=2/rid=1` 与此前 `VOLT gen=4/rid=3` **不属于同一域**。新 CAP 明确携带 `mgen/mrid`，Host 必须使用它进行 MODE 关联。

## CONFIG?：无副作用的原子查询

`CONFIG?`（本轮新增）在短临界区内复制一个已发布的完整对象；它不执行硬件操作，不拼接 MODE?/ROWS?/ROWMODES? 的独立读数。未发布时返回 `ERR,cmd=CONFIG,reason=not_published`。查询字段包括：

- `session`：16 位十六进制随机启动会话；`bootId`：保持旧 RTC breadcrumb 计数。
- `rev/boundary`：配置 revision 与开始允许该配置采样的 frame boundary。
- `state/mode/rows/profile`：采集 owner 发布的状态；mixed 的 mode 为 `MIXED`，profile 保持完整 8 个字符。M header 的非活动行仍用 N，不产生 MR。
- `mgen/mrid/rgen/rrid/pgen/prid`：各域最近**成功**应用的身份。
- `mtermId/mterm/merr`、`ptermId/pterm/perr`、`rtermId/rterm/rerr`：最近一次各域请求的终态缓存；outcome 为 none/applied/rejected。
- `err`：当前 mode 状态错误，与某个请求的 terminal error 分开。

失败 request ID 不覆盖成功 applied ID。`MODE?`、`STATE?`、`ROWS?`、`ROWMODES?` 仍供旧软件/诊断使用，它们组合起来不保证原子性；支持新能力的 Host 必须以 CONFIG 为重建依据。CONFIG 公开的是最后完整发布的配置，不声称队列里尚未应用的每条 accepted intent 已生效。

## 接受、应用、丢失与乱序恢复

MODE、ROWS、ROWMODES 在所有 transport 共用配置 ID allocator 和单个 outstanding 配置事务。忙时在接受前拒绝，Host 可查询并稍后重试；不静默替代已接受请求。其它有界诊断命令仍可排队。MACK、RCMD、RMACK 是 accepted，绝不是 Applied。终态的 `identityKnown=0/rev=0` 表示当时无法取得完整快照，仍保留真实 session；此时重试 CONFIG?，不能将零 revision 当成功配置。

ACK 本地提交成功后，配置命令才对 Core 1 可见；ACK 提交失败则释放 reservation。Core 1 在完整帧边界应用。终态存入专用有界保留槽，优先于普通诊断发送；最后结果另存权威快照。这里的本地提交和保留不能证明 PC 收到，不承诺 BLE/UDP exactly-once。物理断连期间仍可能没有任何有效接收端。

同一 CAP 再次请求仍有新的 request ID 和成功 generation；不能无声忽略。MAPP 表示硬件/配置已应用，`boundary` 指定允许新配置采样的边界，**不保证首个有效数据已传到 Host**。RMAPP 接受完整新扫帧或已验证配置下的有界 CRC 部分帧；它不保证所有点有效。局部 CRC 重试耗尽仍输出 X02，valid/fresh/acquired 保持真实缺失，Host 应按点处理位图，不能因为配置已 applied 就将缺失点补零或沿用旧值。ROWS RAPP 只确认行配置，SAFE 中也可完成，不表示矩阵已恢复运行。

Host 的恢复顺序：

1. 将 accepted 请求与当时 session 和请求域一起保存，保留旧显示数据的原身份。
2. 新帧早于终态时，可以暂存有界数量的新 tuple；用 CONFIG 验证 session、rev、各域身份与 boundary 后接纳。不要把数据早到当成终态丢失的唯一原因。
3. 终态丢失而 CAP 数据持续时，CONFIG 中匹配的 mrid/mtermId 及显式 CAP mgen 可结束 pending。C 的 legacy gen/rid 只验证 ROWS。
4. 旧终态迟到时按 session+请求域+ID+revision 关联；不得用它回退较新的权威配置。
5. 新旧两个 CAP 请求均可成功，必须按 mrid/mgen 区分，不能只比 mode 字符串。
6. 配置切换前排队的帧仍保留原身份。可以按应用边界明确丢弃并计数，不能给它们重贴新标签。

## 启动、计数器与时间

RECOVER 另绑定接受时的 session 及各域 gen/rid，旧配置的恢复请求返回 `RERR,reason=stale_config`；仅运行状态/rev 改变不使同配置恢复失效。单个恢复 pending 不被后来请求无声覆盖。

旧 bootId 在 RTC 无效/断电后可能重新从 1 开始，不能单独证明两个连接属于同一启动。新增 session 在 app_main 生成一次、重启重新生成，是随机 64-bit nonce；碰撞概率很低，**不是数学上绝对唯一**。跨 session 不比较 sequence、generation 或 revision 大小；恢复连接先查询 CONFIG 并重新建立基准。

32-bit generation/revision/request allocator 回绕跳过 0；0 表示未应用/未知。frame sequence 是无符号 32-bit 模运算，可回到 0。排序仅限同 session、同一域、距离小于 2^31 的 serial-number arithmetic；请求关联优先精确相等。无法证明半区间时重新查询。不同域绝不比较大小。

`ts`、`dur`、`tr`、`transitionUs`、采集/事务 `*Us` 使用微秒；`*Ms` 使用毫秒；seq/frameCount/ageFrames 是帧计数，不是墙钟秒。零 timestamp 的含义按报文说明，不推导成物理稳定。`4294967295` 的 age/lastGood 未知哨兵不能显示为实测巨大年龄。

## CRC、无效点与科学数据

完整 ASCII frame CRC32 使用 reflected polynomial `0xEDB88320`、初值 `0xFFFFFFFF`、final xor `0xFFFFFFFF`。覆盖从 C/V/R/M header 首字节到 K 之前最后一个换行的所有字节，包括新增 header identity。K 自身不在该 CRC 内；同时检查 K 的 seq、legacy 域、session/bootId/rev 与 header 一致。BLE G envelope 的总长及 CRC32 是额外的重组校验，不能代替 ADC SPI 校验。

CAP pf6 是 pF × 10^6，既有无效 sentinel 和 masks 保留。VOLT fixed value 是 µV（scale=-6），RES fixed value 是 mΩ（scale=-3）；MR 各行独立 unit/scale。`Xhh` 是明确无效原因，不能填 0 或用于 PGA/cache。expected 是计划掩码，acquired 是实际已完成采集的掩码；健康检查在任何点转换前失败时 acquired=0，不等于执行了 64 次转换失败。

TX=SHORT/RT 的 B20 是 `previewOnly=1` 摘要，保留来自原队列 payload 的 session/revision/各域身份；无效端点为 na。B20 没有完整科学 frame 的 ASCII CRC，不可替代 C/D/K，也不能仅凭预览声称全部点已采集。

## rail、电池、错误与 transport

透明内部 rail 监测是否可用必须看 capability/硬件隔离。当前板型无已证实隔离时报告 unavailable，外部 DMM 校准是受限 VOLT 的合法路径；不要在 Host 自动发起未授权 INTREF 激励。rail span、AVDD、AVSS 分开解释，source 与 estimated/valid/age 分开显示；span 实测不等于两条对地轨均独立实测。外部校准不能因 matrix 帧龄而自动变为内部实测。rail/battery 的 valid=0 保留未知和真实 last-good age，不沿用旧值冒充本次有效值。

错误码必须按实际 BUILD 的 ESP-IDF 解释；本次 IDF `ESP_ERR_INVALID_RESPONSE=0x108`，`ESP_ERR_TIMEOUT=0x107`，不得把所有 0x108 称为超时。ADSFAULT 的 originalError 与 cleanupError 分开，阶段指向首次比较/读取失败；SAFE_REQUESTED 或 GPIO 回读不代表模拟节点已验证安全。故障多行按 bootId + seq 关联，不能把截断行补为原始数据：

| 报文 | 本轮新增/明确的字段 |
| --- | --- |
| ADSFAULT | 首次 stage 及收尾前 mode/route；未知 bootId 明确 unknown |
| ADSFAULTRAIL | railSpanUv、avddUv、avssUv、source、valid、ref |
| ADSFAULTCTX | originalError、cleanupError、engineFrameCount、modeEntryFrameCount、healthInterval、healthReadbackDue、precisionFrame、attempted、acquired |
| ADSFAULTREG | reg、expected、actual、mask、cacheGen、physicalGen、configGen |
| ADSFAULTTXN | kind=rail/battery/ADSCHK、id、startUs、completeUs、firstValidFrameUs、originalError、cleanupError |
| ABAT | 旧 UINT_MAX sentinel 保留；新增 lastGoodKnown，unknown 不是实测年龄 |
| ARL | split=unknown/estimated/external_measured、aincom=estimated、uncertaintyUv=unknown、calUs |

上述 ADS 故障格式化行保持有界（<384 bytes）；这不是全部协议行的长度上限，USBSTREAM 控制回复使用下文的 512-byte 容量规则。ADC 数据 CRC8 在生产路径启用，但 ADC STATUS 不在该芯片数据 CRC 保护范围；软件仍需检查状态和配置 freshness。完整实现证据见 `docs/audit/artifacts/reliability_fix/ads_notes.md`。不能通过相同名字的 rail 字段猜 AVDD 或 span。

Serial 为 ESP32-S3 USB Serial/JTAG，不是 TinyUSB CDC。波特率设置不是 Native USB 的真实线速上限。USBSTREAM/DATA 限流不关闭命令 RX 或关键终态。USBSTREAM? 的 bufferMemory=0/1/2 分别为未分配/内部RAM/PSRAM，bufferBytes 是普通队列的总容量；txStackBytes/txStackMinBytes 是实际TX任务预算/水位。driverAcceptedBytes 只证明驱动接收，不证明 PC 已读；没有驱动枚举证据时 bus 状态为 unknown。COM8 不存在只能说明该次系统打开失败，Host 需自己查询 OS 枚举。

BLE notify 成功排队不等于接收确认；SAFE 的 Indicate/Notify fallback 必须按实际链路状态解释。Wi-Fi UDP 无端到端送达保证。Capture、output queue、各 sink 提交、驱动完成/丢弃分层统计，使用增量与测量窗口；不能把累积 dropD 当每一帧都丢。采集率与 BLE 容量允许不同，Host 应保留可计数的 sequence 缺口。

## 可重放证据与尚未运行项

`python3 tools/test_config_identity.py` 真正编译并运行 C 状态机和实际 encoder，生成 `tests/fixtures/config_identity/*.frame` 与 `.config`。这些是**合成输入，经真实 encoder/formatter 输出**，不是现场原始日志；CRC 从 encoder 产生并由 Python zlib 独立验证。覆盖 1/8 行 CAP/VOLT/RES/mixed、健康前转换失败、50 次模式切换、rows 域与 mode 域分离、失败 appliedId、恢复后 CAP、boot/session 和回绕。R 极值 1844 bytes、mixed 极值 2085 bytes 均小于 2304。

`python3 tools/test_rowmodes_ack_guard.py` 执行真实 mailbox/config/guaranteed lane 测试，含 MODE/ROWS/ROWMODES 仲裁、ACK 前不可见、ACK 失败取消、满队列回退及终态有界存储。`python3 tools/test_main_config_commit.py` 从当前 main 抽取实际 commit/terminal 分支，7 个硬件边界 stub 场景检查 snapshot/preAccept 失败不发 MAPP、成功 mode/profile 同 revision、mixed RMERR 发布最终故障状态。目标芯片 build 与全套实际测试结果由根审计报告记录。首批仅完成 host 回归时，缺少对应实机证据的项目记为 **NOT_RUN**；这是当时的范围，不是对当前所有 HIL 的结论。后续已执行的逐配置采样、控制切换及 BLE 接收按 [最新 HIL 记录](../audit/reference_fixture_hil_20261104.md) 分别判定 PASS/FAIL/未验收；仍未执行的 Windows 枚举、示波器或其它场景继续保留 NOT_RUN。不能由这些 host 测试推导物理准确度 PASS。

## 实机追加诊断：CRC 与健康边界

- `0x109` 是实际 ESP-IDF 的 `ESP_ERR_INVALID_CRC`；`MATRIX_CRC` 区别于路由、寄存器比较和 DRDY 故障。`ADSFAULT` 的 modeGen/profileGen/rowGen/rowReq 分别属于 MODE/profile/ROWS/ROWS；辅助事务 ID 不再借用 rowReq。只有同一 frame sequence 的 engine context 才可附着到当前 fault。
- `ADSSPI` 是可限频的诊断事件：crcFailures/crcRetries/crcRecoveredReads 属该帧，crcTotal/retryTotal 是本次设备启动内的驱动累计计数。读取时必须做同 boot 增量；日志缺失不等于无错误，恢复成功也保留累计数。`firstS/firstD` 指该帧第一个被拒绝 CRC 样本所在测点，firstError/terminalError/frameError 分别说明首次失败、最终重试失败和整帧结果。正常结束时 terminalError=0。
- `ADSSPIBYTES` 给出同 seq 的首个坏样本，bytes 最多 6 字节，按 STATUS（若启用）+ 4 字节数据 + CRC 排列，len 指实际有效长度；received 是线上 CRC，computed 是从 4 个数据字节计算的 CRC。STATUS 不受芯片 CRC 保护。firstUs 为设备单调 µs；physicalGen/drdyGen 是当时配置/转换域。
- 每 cell 的 CRC 重试共享现有 `CONFIG_SENSORARRAY_ADS_MATRIX_IO_RETRY_COUNT` 预算（当前 1），且受同一个 100 ms cell 总截止时间约束。每次重试等待新的 DRDY；坏样本不进入 PGA/cache。DMA 不能收回时仍隔离，不能把超时当取消。
- `ADSFAULTREG` 在 MATRIX_CRC 时 reg=255 表示没有寄存器可比较，expected/computed 与 actual/received 是 CRC 字节；cell 只从 ADSSPI 读取，不能把 reg 当 cell index。
- `ADSHEALTH` 是健康回读事务退出后的诊断，字段为 seq、engineFrameCount、modeEntryFrameCount、healthInterval、healthReadbackDue、precisionFrame、err。它用于核验实际累计 engine 边界，不能拿 host seq 或模式区间长度替代 engineFrameCount。
- 电池 admission 已明确因 rail 不存在/过期拒绝且未改状态或恢复已验证时，reason 保留 rail_invalid；不再泛化成 spi_error。真实 SPI/CRC、restore failure 仍单独报告。LastGood 时间不因拒绝测量而刷新。

这些都是可忽略的追加诊断标签，不是新的有效测量点；不要将 ADSSPI/ADSHEALTH 当 C/V/R/M 数据，也不从诊断事件推导 PC 已收齐测量帧。

## 2026-11-05 追加诊断契约：8d6d656

本节记录 **`8d6d656a6272af7f19e338b594e98d582762737e`** 引入的诊断，当前已烧录版本见文首；该历史版本继承 `a0460b8` 的 CRC/故障域修复、`8b0e651` 的电池入场诊断和 SPI 时序试验、`6a8f502` 的 CRC 末位覆盖计数。以下字段在当前版本继续保留，保持 PROTO2、测量帧 CRC 与字段域不变；诊断新增字段由 Host 按标签解析并容忍未知字段。该版本号或新增日志本身不表示所有 HIL 项通过，也不表示 SPI 电气问题已排除。前文 NOT_RUN 条目仍以各自的实际证据为准。

### 电池：入场状态与恢复结果分开

`BATREG/BATRAW/BATCTX/BATCMP` 以 **当前 session + seq + txn** 关联；这些行本身没有 session，Host 必须从同一连接的 BUILD/CONFIG 与启动事件建立上下文，断连后不得仅凭 txn 拼接。`txn` 属 BATTERY 辅助事务，不是配置 request ID。`BATCTX` 在事务返回错误或 continuation gate 失败时输出；`BATCMP` 仅在保存了具体比较差异时输出。没有某行可能是没有该证据或日志丢失，不能反推测量成功。

| 字段 | 当前生产语义 |
| --- | --- |
| BATREG `phase` | saved/configured/restored；故障时另有 shadow。新增 ID、INTERFACE；各寄存器为无 `0x` 前缀的十六进制。OCAL/FSCAL 按高字节在前显示，不改变其实际寄存器写入顺序。 |
| BATREG `known`；BATCTX `*Known` | 该组快照是否取得；shadowKnown 仅说明软件 shadow 完整。savedKnown/configuredKnown/restoredKnown 不等于“匹配预期”或“模拟稳定”；known=0 时的零字节是未填充内容，不是实际寄存器值。 |
| BATCTX `stage/cleanupStage` | 首个事务失败所在步骤与收尾步骤分别记录。snapshot_shadow_mismatch 是稳定物理读数与 shadow 不一致；snapshot_inconsistent 是驱动未取得一致读数；snapshot_read 是快照读取失败；reset_observed 是读取到了 POWER.RESET。不能把上述几种情况统称为芯片复位或执行过恢复。 |
| BATCTX `originalError/cleanupError` | 保留首个事务错误，收尾错误单列；如果仅收尾失败，两者可以相同。入场即发现未知状态、未执行恢复时 cleanupError=0，不再用虚构的恢复错误替代原始条件。 |
| BATCTX `modified/restoreAttempted` | modified=1 表示已经开始可能修改硬件的命令，失败命令也不能假定未生效；restoreAttempted=1 才表示执行过恢复流程。 |
| BATCTX `restore`；BATERR `restoreState` | unchanged：无需恢复且允许继续；verified：恢复路径成功；failed：执行恢复但失败；unknown：入场状态未知，未执行恢复。verified 是软件/寄存器恢复判定，不是模拟节点实测。 |
| BATCTX `snapshotRead` | 入场快照驱动诊断：0 无该次读取失败、1 读取/SPI 错误、2 有界多次读数不一致。必须结合 savedKnown/stage，未调用读取时的默认 0 不证明读成功。 |
| BATCMP `phase/reg/expected/actual/mask` | phase=admission 或 restore；每阶段保留首个差异。reg 是**芯片寄存器地址的十六进制**，其余三字段也是十六进制。shadow mismatch 时 expected 来自 shadow；读数不一致时 expected/actual 是驱动保留的两次读数；恢复比较时 expected 来自 saved。 |

比较只排除手册规定的保留位：POWER mask=13（包含 RESET）、INTERFACE=0F、REFMUX=3F，其余被比较的 ID/MODE/MUX/校准字节 mask=FF。RESET 单独观察时 mask=10；INTREF 是配置位，仍比较。日志中的稳定差异不证明 shadow 或芯片哪一方正确，当前行为是保留证据并隔离，不自动用读数覆盖预期。

`BATERR` 保留旧 `restore=ok/fail` continuation gate，新增 `restoreState/stage/cleanupError` 供精确解释。`restore=fail` 也可能是入场发现未知状态，不能仅凭旧字段称“恢复操作失败”。`action=isolate_ads` 表示不得继续使用 ADS 并交给有界恢复；仅允许继续的电池无效结果使用 `action=report_continue`。LastGood 的值、frame 与实际测量时间不因这些失败刷新。`BATRAW.timeUs` 的零槽位不代表采到了零值；结合样本数、错误和阶段解释，不能把未开始转换计作三次有效采样。

### Rail：专用采样配置与能力边界

`PROTO?/ROUTE?` 的 `railMonitor=unavailable` 指当前板型没有已验证的**透明隔离辅助监测能力**。它不表示 RES 中所有内部 rail 读取都被禁用：已选择 RES、参考本来就有意开启时，内部 monitor 可复用该状态。生产入口要求物理 POWER.INTREF 已开启，REF source 命令正确且 Q1 钳位命令/回读均释放；这仍不是模拟隔离实测。CAP/VOLT 不能为内部监测偷偷开启参考，受限 VOLT 继续使用合法外部实测 rail 校准。

`8d6d656` 为内部 supply monitor 显式设置 PGA enabled/gain 1、chop off、连续转换、SINC1、当前配置的 ADC data rate、INPMUX=CC、REFMUX=00、neutral OCAL/unity FSCAL，再读回验证；不再继承上一个矩阵点的 PGA bypass/gain/filter。当前 sdkconfig 的 DR=15 对应 MODE0/1/2=`00/00/0F`，Host 不应把这组常量当未来所有 BUILD 的固定值。事务结束恢复完整原配置；配置验证、采样成功和恢复成功是不同条件。

| 报文/字段 | 当前生产语义 |
| --- | --- |
| RAILPROFILE `before/configured` | 三个十六进制字节依次为 MODE0/MODE1/MODE2。configured 是物理读取值；known=0 时不可解释其零值。before 在入场/保存失败时也可能未取得。 |
| RAILPROFILE `mux/ref/known/verified/restored` | mux/ref 是配置快照的 INPMUX/REFMUX；known 为快照取得，verified 为专用 profile 比较通过，restored 为完整原配置恢复成功。restored=1 不排除先前 STOP 失败，仍须检查 cleanup。 |
| RAILPROFILE `stage/err/cleanup` | stage 为 ADMISSION/CONFIGURE/PROFILE_READ/PROFILE_COMPARE/START/SAMPLE/SAMPLE_STATUS/RESTORE_VERIFY/NONE。ADMISSION 也覆盖尚未填充诊断前的读取/保存错误，不能仅凭此判断隔离拒绝。err 是边界函数最终结果，可能还包含 rail 合法性拒绝；stage=NONE 单独不代表整个边界事务成功。cleanup 单列 STOP/恢复错误。 |
| RAILSAMPLE `n/raw/monitorUv/status` | n 是已尝试的 fresh-read 调用数，**包含失败的那次**，当前最多 5（4 次丢弃加最终 1 次）。raw/monitorUv/status 只记录最近一次尝试，不是全部原始样本，也不保证 CRC/状态通过。monitorUv 为内部约 `(AVDD−AVSS)/4` 通路的 µV，不能直接当 railSpanUv 或对地电压。 |
| RAILSAMPLE `reg/expected/actual/mask` | PROFILE_COMPARE 时给出首个差异；这里 reg 是**十进制**地址，与 BATCMP 的显示进制不同，后三字段为十六进制。255 表示已初始化但无寄存器差异；早期 ADMISSION 的默认零值也不是有效比较证据。 |

以上两行按当前 session + seq + RAIL txn 关联，不与相同数值的 BATTERY txn 合并。样本只有在整笔事务成功、profile/状态/恢复检查通过且后续 rail 合法性验证通过后才更新有效 rail；诊断原始值不能直接写入测量/校准。Span、来源或采样 profile 变化应划分分析区间，不能把跳变前后总均值拟合成一个固定 offset。完整事务起止仍看 ADSFAULTTXN/对应事务证据，不能从 RAILSAMPLE 或单独采样耗时推断全部等待时间。

### CRC 覆盖计数与有界日志

`ADSHEALTH` 追加 `crcValidEven/crcValidOdd/crcErrors/crcRetries`。前两者是 CRC8 **校验通过的 CRC 字节最低位**为 0/1 的累计次数，不是数据值奇偶、bit population parity、健康检查次数或有效测量点数；checksum/off 模式不计入它们。计数发生在数据解析层，随后仍可能被 STATUS、freshness、generation 或电气合法性检查拒绝，且包含共用 ADC1 驱动的辅助读取。crcErrors/crcRetries 同样为驱动累计值，而 ADSSPI 的 crcFailures/crcRetries 是对应帧值，不能直接混算。

这些 uint32 计数属于当前驱动 handle 生命周期，通常跨模式保留；重启或驱动重新初始化会归零，回绕按同一生命周期的无符号差值处理，无法确认连续性时重建基准。Host 应在同 session 的有界时间窗比较增量。只有 odd 增长不能覆盖“末位被强制为 1”的反例；even/odd 都增长只证明该窗观察到两类通过校验的 CRC，不能证明线路完全可靠。

BATCTX/BATCMP、RAILPROFILE/RAILSAMPLE、ADSHEALTH 均是普通诊断事件，没有独立科学帧 CRC，也没有终态送达保证。这些采集侧事件的格式化缓冲为 **384 bytes（含结尾 NUL）**，不适用于 USBSTREAM 查询回复或完整科学帧；当前上述固定格式按有界字段输出，事件队列零等待提交，背压记入 droppedEventLogs。通用格式化器仍有截断后补换行的兜底路径，因此“有换行”不证明字段完整，Host 必须拒绝缺必需字段的诊断记录，保留原字节，不能补成完整原始行。下文串口整行队列处理的是 VFS 片段与队列边界，不能补回源格式化器已丢失的内容。

对应的纯主机回归为 `tools/test_battery_snapshot_evidence.py`（实际入场/比较/日志函数）、`tools/test_battery_validity.py`（错误分类与 LastGood）、`tools/test_ads_rail_profile.py`（实际 rail 配置/保存/恢复函数配合合成寄存器 bank）、`tools/test_ads_driver_reliability.py`（驱动校验和边界）。这些测试的合成输入不冒充实录；目标构建、烧录、逐配置 HIL 结果及未通过项以根审计报告追加的证据为准。

### 后续边界补丁：rail CRC、MIXED、USB 输出绑定

本节对应 `8d6d656` 之后的完整帧边界与传输补丁，已包含在文首关联的 `ebabe7e` 镜像中；不能把下面新增字段归到未包含这些改动的旧镜像。旧字段和测量 CRC 覆盖范围保持不变。

- `RAILSAMPLE` 追加 `accepted/crcErrors/crcRetries/terminalError`。accepted 是通过 CRC、freshness、配置 generation 和 STATUS 检查的采样数；成功需 5 个合格转换，其中前 4 个丢弃。n 仍是外层 fresh-read 调用数，内部 CRC 重试不另外增加 n。crcErrors/crcRetries 是本笔 rail 事务的计数；整个 5 次转换共享 **1 次** CRC 重试预算，不能每个样本重置预算。坏帧不计入 accepted 或有效丢弃数，每次重试等待新 DRDY，沿用同一 **250 ms ADS owner 截止时间**。terminalError 只描述最后采样结果；其为 0 不排除 RAILPROFILE 的后续合法性或 cleanup 错误。
- `RAILCRC` 仅在本次 rail 事务发生 CRC 错误时输出，与 RAILPROFILE/RAILSAMPLE 按 session + seq + txn 关联。firstUs 是首个坏帧的单调 µs；bytes/len/received/computed/physicalGen/drdyGen 与 ADSSPIBYTES 的 ADC 响应语义一致，最多保留 6 字节，不是全部重试样本。即使后来恢复成功，仍保留这条首错证据；日志缺失不能清零错误计数。
- `MIXED_RAIL_BOUNDARY` 记录完整混合帧**首点之前**的预备事务。firstRes 是活动 profile 中用于建立 RES 激励的首个物理 S 行；restoreMode/restoreRow 是进入前要恢复的路由。originalError 与 cleanupError 分开，restored 仅表示原路由及 reference-ready 等待成功，不取代 cleanupError。startUs/endUs 包含选行、受控 RES 路由建立、内部 rail 事务、原路由/行恢复和参考等待；这是**外层完整预备耗时**，不能声称整体受内部 250 ms 单一截止时间限制，也不能把 monitor 采样耗时当全部暂停时间。有效 rail 无需预备或早期拒绝时可能没有该行。
- MIXED 只对活动 profile 中明确的 RES 行建立参考；先选中目标行再进入 RES。无 RES 且无真实隔离能力时，不为 CAP/VOLT 额外开启参考。预备失败时 CAP/ADS 子段均未开始，整帧 acquired=0；标准 ADSFAULTCTX 同时保留当前 seq 的原错与 cleanup，attempted=0/acquired=0。该上下文的 mode=MIXED、route=unknown 表示没有保存收尾前路由证据，不能拿恢复后的 SAFE/CAP 冒充首次故障路由。
- 内部新增 `sensorarrayMixedRowPrepareRailAtBoundary`、`sensorarrayAdsMatrixEngineRecordMixedBoundaryFault` 及 plan 的 `auxiliaryTransactionsAllowed`，都不是 Host 命令。普通 plan 明确允许完整帧边界事务，MIXED 子段明确禁止 fallback 刷新；当前预备调度比 rail 最大帧龄提前 1 帧，不延长合法帧龄，也不把失败前旧 rail 重记为新测量。battery/ADSCHK 继续在完整 RunOneFrame 返回后串行执行。新 mask/plan 标记不重解释 gen/rid 或科学数据单位。

`USBSTREAM?` 追加 `stdioBound/initError`。stdioBound=1 表示固件已经验证共享 stdout/stderr FILE 指向它自己的有界 USB 输出 VFS，协议和普通日志可由同一 TX owner 排队；它不表示 PC 已读、COM 已枚举或所有排队数据已送达。initError 是最近一次初始化返回的 ESP-IDF 错误码；默认 0 单独不能证明完成初始化，应与 stdioBound、driverInstalled 一起看。stdioBound=0 时队列拒绝提交，不能静默退回第二个直接 USB writer；此初始化门禁的拒绝发生在 sinkQueued/sinkDropped 更新之前，不能只看这两个计数判断初始化成功。命令 RX 仍由唯一 Serial reader 使用 USB Serial/JTAG 驱动读取，不依赖 stdout/stderr 绑定；enumerated 继续明确 unknown。

`3fb2a19` 曾试验完整 RDATA1 后追加 NOP（DIN=00h），实机 `final-res8-01` 仍出现末位 CRC 错误，候选已撤回。当前 RDATA1 使用原响应长度：4 字节数据，加可选 STATUS 和校验字节，最多 6 字节；所有 ADC CRC 位继续验证。该历史试验也未改变 CRC 位置、ADSSPIBYTES/RAILCRC 的 bytes/len 或 ASCII frame CRC；Host 不需要添加 ADC 字节或改变校验范围。CRC 的底层物理/控制器原因仍未确认，不能从一次软件尝试推导其已解决。

`tools/test_mixed_rail_boundary.py` 使用**合成硬件边界**运行完整生产 MIXED 引擎和真实 ADS fault formatter，验证首点前刷新、已知 rail 的无切换路径、无 RES 拒绝激励、失败 acquired=0、恢复错误独立保留，以及旧 seq 上下文不附着当前故障。它是 host 回归，不是实机采样记录；未覆盖的 HIL 项继续标 NOT_RUN。

### 完整串口记录边界（ebabe7e）

stdout/stderr 的VFS write可以是同一printf的多个片段。固件按两个流分别组到LF后才把整行交给唯一TX owner；它从现有normal pool暂借槽，不增加永久大缓冲。中途priority回复不会进入半行。槽不足、队列满或超出2304B时，整条诊断丢弃并持续丢到LF重同步，不发布前缀；两个未完成流最多各占一个槽，其余数据与priority仍可工作。legacy同步科学帧也直接按完整packet入队，保留真实queue错误，不能由stdio的本地消费返回值推定已提交。

`USBSTREAM?` 新增lineDropped（整行诊断丢弃次数）、lineOversize（其中超长行次数）、discardBytes（丢弃诊断字节）。它们不同于driver partialWrite/timeout；sinkDropped包括队列或整行丢弃，不能将这些计数简单相加成科学帧损失。VFS返回size只确认有界本地消费（包括已计数的丢弃），sinkQueued才是完整记录本地入队，driverAcceptedBytes仍不证明PC收到。无效的非换行packet和驱动部分写还使用有界delimiter恢复，下条控制回复保持行边界；断连不保证终态送达。

USBSTREAM输出受512B控制回复容量限制。当前实录计数下完整回复约414B，所有字段类型同时取极值可能超容量，此时明确返回ERR,reason=response_too_long，不输出截断的伪完整统计。不能把一次query失败或计数缺失当零；CONFIG?的原子配置身份独立于这一统计查询。

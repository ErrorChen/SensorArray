# 固件与上位机契约：配置身份与可靠性修复

本轮修复基于 `8045e9e9ec9599533c52c15dfcb6002f79fd15f1`。**固件修复 commit：`ba8346b7b643e54985699630f608aba3386810e9`**；后续本地文档提交附本契约、报告与故障回放。本文件描述该修复及配套夹具，准确构建输入/dirty hash 见 `docs/audit/firmware_reliability_fix_report.md` 的验证记录。此协议交给 ErrorChen/SensorArray_Software 的独立任务实现；本轮未修改软件仓库。协议格式是实际 `main/output/sensorarrayTextProtocol.c` encoder 的输出。

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

同一 CAP 再次请求仍有新的 request ID 和成功 generation；不能无声忽略。MAPP 表示硬件/配置已应用，`boundary` 指定允许新配置采样的边界，**不保证首个有效数据已传到 Host**。RMAPP 的当前实现还要求该 profile 产生 fresh frame，但 fresh 不等于全部点电气有效。ROWS RAPP 只确认行配置，SAFE 中也可完成，不表示矩阵已恢复运行。

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

各故障行保持有界（<384 bytes）。ADC 数据 CRC8 在生产路径启用，但 ADC STATUS 不在该芯片数据 CRC 保护范围；软件仍需检查状态和配置 freshness。完整实现证据见 `docs/audit/artifacts/reliability_fix/ads_notes.md`。不能通过相同名字的 rail 字段猜 AVDD 或 span。

Serial 为 ESP32-S3 USB Serial/JTAG，不是 TinyUSB CDC。波特率设置不是 Native USB 的真实线速上限。USBSTREAM/DATA 限流不关闭命令 RX 或关键终态。USBSTREAM? 的 bufferMemory=0/1/2 分别为未分配/内部RAM/PSRAM，bufferBytes 是普通队列的总容量；txStackBytes/txStackMinBytes 是实际TX任务预算/水位。driverAcceptedBytes 只证明驱动接收，不证明 PC 已读；没有驱动枚举证据时 bus 状态为 unknown。COM8 不存在只能说明该次系统打开失败，Host 需自己查询 OS 枚举。

BLE notify 成功排队不等于接收确认；SAFE 的 Indicate/Notify fallback 必须按实际链路状态解释。Wi-Fi UDP 无端到端送达保证。Capture、output queue、各 sink 提交、驱动完成/丢弃分层统计，使用增量与测量窗口；不能把累积 dropD 当每一帧都丢。采集率与 BLE 容量允许不同，Host 应保留可计数的 sequence 缺口。

## 可重放证据与尚未运行项

`python3 tools/test_config_identity.py` 真正编译并运行 C 状态机和实际 encoder，生成 `tests/fixtures/config_identity/*.frame` 与 `.config`。这些是**合成输入，经真实 encoder/formatter 输出**，不是现场原始日志；CRC 从 encoder 产生并由 Python zlib 独立验证。覆盖 1/8 行 CAP/VOLT/RES/mixed、健康前转换失败、50 次模式切换、rows 域与 mode 域分离、失败 appliedId、恢复后 CAP、boot/session 和回绕。R 极值 1844 bytes、mixed 极值 2085 bytes 均小于 2304。

`python3 tools/test_rowmodes_ack_guard.py` 执行真实 mailbox/config/guaranteed lane 测试，含 MODE/ROWS/ROWMODES 仲裁、ACK 前不可见、ACK 失败取消、满队列回退及终态有界存储。`python3 tools/test_main_config_commit.py` 从当前 main 抽取实际 commit/terminal 分支，7 个硬件边界 stub 场景检查 snapshot/preAccept 失败不发 MAPP、成功 mode/profile 同 revision、mixed RMERR 发布最终故障状态。目标芯片 build 与全套实际测试结果由根审计报告记录。没有本轮实机、Windows 枚举、示波器或 BLE 接收证据的 2 分钟/HIL/重插拔/吞吐/物理准确度项全部是 **NOT_RUN**，不能由这些 host 测试推导 PASS。

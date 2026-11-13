# 前端启动选速率与掉电恢复（2026-11-13）

本轮继续基于 `7e1d2c6dd9c85883ce9018fbc89a5d2402410607`，不改上位机仓库。用户新增要求是启动时 SPI 与 I²C 并行扫描、细化 SPI 档位、扫描容许 2.5% CRC 错误，以及三颗前端失联后低频探测、恢复后重新启动并扫描。先前电阻夹具/CRC 失败实录仍保留在 [11月13日复测](reference_fixture_recheck_20261113.md)，不能由本次新增启动测试覆盖。

## 代码职责

- `core/board/sensorarrayFrontendStartup.[ch]`（本轮新增）：承接从 `main/main.c` 移出的原 FDC ID/负载扫描、选频、初始化和 SPI 并行任务。当前双 FDC 的初始化仍由采集核依次执行，SPI 在另一个核并行；正常逐行双 FDC 并行采集保留。
- `core/board/sensorarrayAdsSpiProbe.[ch]`（本轮新增）：只做独占的 ADS 链路资格测试和完整配置恢复，不生成科学测量帧。启动和前端恢复后的新 boot 共用此实现。
- `core/board/sensorarrayFrontendPresence.[ch]`（本轮新增）：低频三芯片通信证据及等待/恢复状态机。查询只复制已发布状态，不在命令回调中访问总线。
- ADS 驱动的统一持久 SPI 描述符应用频率覆盖；完成未知仍 quarantine，不能改频、释放或复用 DMA 对象。`sensorarrayRouteControllerEnterFrontendWait`（本轮新增）只执行等待期 GPIO 动作并将路由标为 UNKNOWN。

## SPI 资格标准

候选从配置上限向下：2 MHz 至 1 MHz 每档 100 kHz，950 kHz 至 150 kHz 每档 50 kHz，最后 125 kHz。请求频率经过 ESP32-S3 时钟分频量化；日志分别保留 requestedHz 与 IDF 返回的 actualHz，后者不是示波器测量值。实际完成事务后核验频率，避免把驱动拒绝覆盖后继续使用的旧频率误认为新档位。

默认每个候选初筛 400 次、独立确认 1600 次，均平分为 PGA gain1 和 bypass。每个 profile/阶段各自允许最多 2.5% CRC 错误：200 次中的 5 次、800 次中的 20 次；超过后才按 CRC 条件拒绝该档。坏 CRC 只计入分母和错误数，不计有效样本、CRC 奇偶覆盖、原始值范围或测量缓存。该门限仅用于启动选速率，生产测量的 CRC 校验与无效点语义不变。寄存器不符、芯片复位、STATUS 异常、饱和或事务完成未知仍是独立失败条件。

所有档位共用默认 30 秒单调期限，含任务调度、初筛、确认和恢复，预留 500 ms 收尾。期限用尽就报告失败，不跳过未测试档位，也不把未验证的最低速度作为成功结果。这个 SPI 期限不包含原有 I²C 启动流程的总耗时。日志中的累计 CRC 包含被淘汰候选；判断选中档位须使用对应 profile/phase 的结果。

测试把 AINCOM 同时接 ADC 两个差分输入，保持 INTREF OFF、REFMUX=AVDD/AVSS，包含偏置建立等待和新 DRDY 判据。不对外部输入作高低阻负载试探，不把短接输入的链路稳定性当成 RES/VOLT 精度或长期无错保证。测试后恢复真实寄存器及 driver shadow；所有后续矩阵/route/gap 初始化必须等待资格结果和恢复结果完成。

## 失联与恢复边界

两颗 FDC 的 Manufacturer/Device ID 与 ADS 的稳定 ID/核心寄存器响应是通信依据；`ready=false`、CRC 错误或 FDC 被主动 SD shutdown 都不能单独证明三颗芯片失联。探测不读取会清中断的 FDC STATUS。事务忙或完成未知时不得另起探测覆盖原 owner。

恢复采用受现有预算约束的 ESP 重启，重新调用同一个启动模块，而不是在旧 worker、DRDY ISR、DMA 和配置缓存上原地重建。新 session/boot 的帧计数和配置身份必须重新查询；RAM 中的临时 RAILCFG 会丢失，不能静默沿用为新电源周期的校准。没有写 NVS 或执行 CAL=SAVE。

当前板 U5 没有已证实的 REF 隔离能力。ADS 重新上电默认 INTREF ON；等待期间释放 Q1 是为了避免主动钳住返回的参考，禁用可控 TMUX1134 也不等于隔离 U5。失联识别存在延迟，软件不能消除上电间隙的所有电气风险，路由只可报告 UNKNOWN/physicalSafe=unknown。实机测试仍限用户的被动 10 kΩ/33 pF 夹具；真实供电切换及边沿需分别记录，不把 host 模拟当作物理断电验证。

## 验证记录

构建、全部 host 回归、烧录、启动实录和运行期采集结果记录于 [本轮证据目录](artifacts/boot_autotune_20261113/)。每次命令的退出码、HEAD、dirty diff hash、包括新文件的逐源码 hash 独立保存。编译通过不等于 HIL 通过，有限启动 CRC 比例也不等于长期错误概率上限。

### 中间版本 e339a1d 的实际结果（保留失败，不作为最终验收）

代码 `ee8f972` 引入功能，`e339a1d` 修正第一次 HIL 发现的 FDC ID 地址错误。ID 寄存器为手册的 0x7E/0x7F，旧探测错误使用 0xFE/0xFF；生产驱动原有初始化地址正确。修正后的独立回归在旧代码上确实失败，见 `fdc-address-negative-regression.json`。没有把读错地址解释为芯片掉电。

实际 IDF v5.5.5 构建和 40 个测试入口均退出 0，源码输入前后 hash 一致；见 `final-02-build-manifest.json`、`final-02-tests-manifest.json`。app-only 烧录地址 0x10000，未擦除 NVS/校准区，esptool 校验 hash；镜像为 1551520 bytes、SHA256 `3f6e0f3bddbc190ae22a96b3472f32850d13d8241855565856fd91baac758833`。真实设备 ESP32-S3 rev0.2 / Flash16MB / PSRAM8MB、ADS1262 ID03；原始记录都在本轮证据目录。

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

USB 的 driverAcceptedBytes 是驱动接收，不是 PC 收到字节证明；启动日志已有 sinkDropped/partialWrite，CAP查询窗口 sinkDropped 增量也非零，不能宣称无背压。科学数据以实收完整帧的 CRC、identity、序列独立核验。CPU占用、最大连续heap块、Windows重新枚举、三颗芯片真实物理断电/复电、供电边沿及SPI波形仍 NOT_RUN；没有用模拟测试冒充这些实测。

### 运行期 CRC 分类追加修复（0d8f60f）

实录失败促使复核 `ReadAdaptive → FinishFrame → main`：首个可信转换前 CRC 重试耗尽使 fresh=0；后续转换 CRC 耗尽可能保留第一次可信转换的 fresh=1。旧调度把 `freshFrame=false` 一律解释为路由/参考失效，导致同类 CRC 因发生阶段不同而有时只报 X02、有时触发 MFAULT。没有把两个现场失败臆断为某个具体 ADC 子读阶段；已确认 VOLT 不进入 RES 的 HighZ 确认分支。

`0d8f60f` 新增明确的 `boundedCrcFailure` 内部状态，只允许完整扫段且硬件状态可信的纯采样 CRC 拒绝继续发布；坏样本的 valid/fresh/acquired 不会被补成真，仍为 X02。`ReadFreshAfter/MeasureCell` 分别保存本点 terminalError，后续 timeout、配置 generation 变化、RESET/reference alarm、owner/route 或 DMA 未回收均独立取消许可，不被之前的首个 CRC 错误掩盖。连续三次活动点全部被 CRC 拒绝的 **ADS engine 扫段** 停止，不把它误称为三个主机混合帧；CAP/SetMode 不清计数，完成非 CRC 的 ADS 测量结论（包括合法 OPEN）才清除。

`ADSDISPOSITION` 公开 scope=ads_sweep、拒绝 mask、真实 acquired/expected、allCrcRun/limit、firstError/abortError。MIXED 后续分组失败会否决前段的许可；模式/profile 终态表示配置已应用，不再要求每个点都通过 CRC。异步和 legacy 输出门允许这类部分帧，协议位图、旧 gen/rid 域和 ASCII CRC 算法不变。

实际生产函数提取测试19场景已通过，包含CRC后timeout/config/quarantine、首CRC后另一点RESET/SPI/owner/row错误、1/8/稀疏行全坏预算、跨SetMode、全OPEN、health故障零转换。驱动/模拟边界是 **synthetic**，不是实机故障注入。中间编译通过；旧测试有一处精确源码needle因新判定失配，修正后仍保留原context先于SAFE的动态断言。最终整套构建、测试和HIL结果另列，不能从此测试推断SPI电气错误已消除。

### 当前烧录版 0d8f60f 的实测记录

当前 BUILD=`0d8f60f-dirty`，镜像 1552752 bytes，SHA256 `232ca3976f6b8ea38efd5e09de8279f35c9bfbbd0c605c8dd84e5169ddd1dbaa`。实际 IDF 构建、41入口初验及加入动态流转测试后的全部 **42入口** 均通过；见 `final-03-build-manifest.json`、`final-03-tests-manifest.json`、`final-04-tests-manifest.json`。构建源码输入前后相同；dirty为文档/证据，新增集成测试不改变镜像。`final-03-flash-result.json` 记录已授权app-only烧录/hash校验。动态流转的11采集场景执行完整生产RunOneFrame/MixedRow、真实mode/profile/identity/encoder，并独立验证3个synthetic报文CRC与位图；不能当作实机故障注入。

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

`volt1-02` 真实帧seq2693的S1D2=X02、expected=FF而acquired/fresh/valid=FD；紧邻seq2694全部为FF，session/revision不变。两段均来自真实encoder且保留线上CRC字节，见 [实录golden manifest](../../tests/fixtures/reference_capture/crc_partial_volt1_20261113.manifest.json)。该反例证明局部无效不再必然将模式送入SAFE，不能证明后续所有CRC都可以忽略。

此VOLT1观察窗口的累计CRC增量4986、retry增量4299，范围是首末收到的ADSSPI快照，包含其间ADC操作，不能除以帧数冒充wire错误率。USB查询窗口sinkDropped增量55（含诊断/准备/清理），partialWrite和timeout增量0；它不能独自精确解释25个科学帧缺口。CRC物理来源、BLE/Wi-Fi本版高负载及三芯片真实断电恢复仍未完成实测。


独立逐帧复核见 `final-voltage-review.json`：VOLT1有661个X02点/631帧，VOLT8有760个X02点/402帧；二者各有2个X02点的acquired/fresh确为0而帧仍正常发布。所观察的ADSDISPOSITION均boundedCrc=1、abortError=0、allCrcRun=0；三次全坏隔离只经过host故障注入，不能称HIL已覆盖。另有X12（UNSTABLE，十六进制0x12=枚举18）8296/3199点，不能算作CRC；本被动悬空夹具并非已知稳定电压源，未放宽spread阈值。

可见健康回读仍每64 ADS engine帧发生；VOLT1绝对128/256/384/512均err=0，继续至13056；VOLT8 modeEntry=18048、首检查18049，累计至20544，入模式后128/256/384/512均err=0。这直接覆盖模式切换前已有累计计数的情况。可见的CRC字节诊断仍全部呈received XOR computed=01，只能定位错误形态，不能没有波形就断言物理原因。


### 交付与未运行项目

当前生产变更的精确文件及函数上下文差异见 [文件清单](artifacts/boot_autotune_20261113/final-production-changed-files.txt) 和 [函数diff](artifacts/boot_autotune_20261113/final-production-function-diff.patch)，比较范围7e1d2c6→0d8f60f；原8045e9e→先前版本的差异仍保留在旧审计报告中，没有覆盖编号。全部原始实录保留，并有逐字节解压验证的lossless gzip及SHA，见 [证据索引](artifacts/boot_autotune_20261113/evidence-index.json)。[最终命令记录](artifacts/boot_autotune_20261113/hil-command-results.json) 区分执行成功与测量验收结论，失败基线未删改。

本轮通过的是可重复启动扫描、现有总线恢复状态机host测试、CRC局部失效分类、模式身份与上述有限窗口运行；不是完整电气/精度验收。真实三芯片断电/复电（且MCU保持供电）、故障注入下的实际DMA/I²C锁阻塞、示波器时序/电源边沿、Windows重新枚举、此版本BLE/Wi-Fi高负载与多transport同时控制、CPU占用及最大连续heap块均NOT_RUN。三芯片物理开关覆盖范围未获确认，不用软件模拟补成HIL。没有危险输入/电池故障注入，也没有PCB或BOM实装修改。

# 2026-11-04 参考夹具实机验收记录

本记录补充 `firmware_reliability_fix_report.md` 的离线阶段，不把该阶段的 NOT_RUN 改写成当时已经实测。用户指定 S1D1/S8D8 为 10 kΩ，S4D4/S5D5 为 33 pF，其余 60 点悬空，并明确授权“烧录修复固件后验收”。校准定义与限制见 [夹具说明](../validation/reference_fixture_10k_33pf.md)。只操作 SensorArray；未修改上位机、未 push。

## 设备与烧录证据

- 宿主 `/dev/ttyACM0`，Espressif USB Serial/JTAG `303A:1001`，MAC `80:B5:4E:CD:D9:AC`。这是原生 USB 虚拟串口，不是已经证明存在的独立 UART 转接器。普通沙箱没有设备节点，宿主授权访问后可用。
- esptool 实测 ESP32-S3 QFN56 rev v0.2、16 MB Flash、内置 8 MB AP_3v3 PSRAM、40 MHz crystal。配置 DIO/80 MHz Flash、octal/80 MHz PSRAM。
- 原设备 BUILD 为 IDF v5.5.1 / PROTO1，ADS1262 ID `03`、ADC2 不可用；新设备查询确认为 IDF v5.5.5 / PROTO2，`adcCrc=crc8`、`railMonitor=unavailable`。源码基线不冒充原设备二进制。
- 首次 pyserial 探测实际观察到 `USB_UART_CHIP_RESET`；随后 POSIX 采集不操作 DTR/RTS，也不声称能证明驱动 open 绝无复位。每次只有一个 reader，保存原始字节与单调时间事件。
- 原 Flash `0..0x31FFFF` 已备份至本地忽略目录（含 NVS，不纳入 git），SHA256 `e25f56491f33db37654074f7088a9bea778c803913ee54c13ea08e244ea316ec`。原分区表与工程一致。首次仅写 bootloader 和 app，未写分区表、NVS 或 calib，未整片擦除或 CAL=SAVE。
- 首次修复镜像对应 `ba8346b7b643e54985699630f608aba3386810e9`，SHA256 `e8c9bbdb4c515ce70573581ead256b406ea762fdb1761fea6ca9bab930e991cf`。命令、退出码、设备识别与 hash 校验见 [flash-result](artifacts/reference_fixture/20261104T122029Z-flash-result.json)。

## 首次修复镜像的实际结果

原始文件名均以 `docs/audit/artifacts/reference_fixture/` 为前缀；`.rx.bin` 是未改写 RX，`.events.jsonl` 含命令、回复、字节偏移及阶段时间；`.summary.json` 是采集结果，`.analysis.json` 是独立 CRC/身份/数值检查。原始文件不因失败而删除。

| 采集标签 | 实际窗口 / 完整帧 | CRC / 身份 / 序号缺口 | 结果 |
|---|---:|---|---|
| `repaired-cap8-01` | 139.840 s / 1608 | 0 / 0 / 0 | CAP 数据链通过此窗口；33 pF 精度 BASELINE_REQUIRED |
| `repaired-res8-01` | 1.111 s / 13 | ASCII 0 / 0 / 0 | FAIL：ADC SPI CRC `0x109`，seq4231，未达到 120 s |
| `repaired-cap1-recovery-01` | 139.832 s / 10149 | 0 / 0 / 0 | RES 故障后命令可用，CAP 恢复成功，约 72.58 FPS |

CAP8 的两个电容路径全程 fresh/acquired，无无效点：S4D4 总电容均值 92.235983 pF、σ 0.004709 pF、范围 92.220881–92.250931 pF；S5D5 均值 77.441666 pF、σ 0.002845 pF、范围 77.432758–77.451686 pF。这些是 LC 总电容，不能直接与 33 pF 相减作为误差；缺各自空载基线、元件实测值和验收限。没有生成或写入校准系数。

RES 故障前 S1D1 中位数 9981.441 Ω（9978.056–9983.773 Ω），相对标称约 −0.186%。这不是高精度 PASS。S8D8 连续 13/13 帧 X0D/OPEN，均 fresh/acquired；其余 60 悬空点也均 OPEN。故障后 `CELL?=S8D8` 的 raw 为 −2147483648、nodeUv=−1740069、pga=0，证实它不是 Host 裁剪或未采集。代码映射 S8D8=index63、row address111、ADS AIN7/AINCOM；未发现需要交换坐标的软件证据。用户回复“尚未确认，稍后检查”，故 S8D8 接线/物理路径为待核实，不靠 offset 修正。

USB 的累计 drop/timeout 在连接前已经增加。三个采集窗口中，起止 USBSTREAM 查询的 `sinkDropped`、`timeout`、`partialWrite` 增量均为 0；CAP8 `driverAcceptedBytes` 增量 1551354，CAP1 增量 4575186。这里另有 PC 原始 RX 与完整帧证据；驱动接受字节本身仍不等于 PC 收到。无 Windows 重枚举、BLE/Wi-Fi 高负载或示波器证据。

## HIL 暴露的追加缺陷

1. **CRC 恢复分类遗漏**：SPI CRC8 已启用，但矩阵有限重试只包含 TIMEOUT/INVALID_RESPONSE，没有 INVALID_CRC。seq4231 的 fresh mask `FFFFFFFFFFFFFFFE` 和 `spi=1` 表明 S1D1 失败，其余 63 点已实际转换；不能称为 64 次 ADC 失败。此前大量转换通过，不能据此认定 CRC 多项式错误或电气原因。
2. **故障上下文仍有漏网分支**：该采样错误没有保留 engine context，main 在 RecordRuntimeFault/SetMode(NONE)/SAFE 后才填日志，产生 MATRIX_ROUTE/NONE/SAFE；故障 `rowGen/rowReq` 又错误使用 profile 域。需保存首次样本校验信息及清理前配置域。
3. **OPEN 诊断 median 早退**：`sensorarrayAdsMathConfirmOpenSet` 的 HIGH_Z 早退可留下初始化的 0，导致诊断 nodeUv 不是实测 median。修复不得改变 OPEN 阈值或把 S8D8 强制判为 10 kΩ。

追加补丁、构建与烧录后的复测结果在本记录后续章节追加；在得到这些证据前，本夹具总体结论为 **未通过精度验收**。


## 追加修复与实际离线回归

固件追加 commit `a0460b8`：每 cell 共用现有一次 CRC 重试预算和 100 ms 总 deadline；拒绝旧 DRDY/跨配置转换，持久 DMA 超时仍隔离。保存首次坏 CRC 的线上字节、计算结果、S/D、generation 和单调时间，恢复事件限频但累计数保留。首个 cell 错误不会被后续 row 错误覆盖；隔离后不再继续扫描其它行。新增 ADSHEALTH 记录真实 engine 计数。main 在收尾前捕获同帧身份，故障 ROWS 字段不再借用 profile 或辅助请求域；旧 engine context 不可附着到新 seq。OPEN 诊断返回实际 median；电池 rail_invalid 的明确早退不再被误报 SPI。

27 个 `tools/test_*.py` 入口实际全通过，另新增 battery validity 10 场景及最终 fault fixture/main context 复核通过，见 [测试结果](artifacts/reference_fixture/crc-followup-test-results.json)、[最终定向回归](artifacts/reference_fixture/crc-final-targeted-tests.json)。CRC 注入包含坏→好、连续坏、CRC→DRDY 超时、跨配置、新数据位缺失、DRDY 回绕、共享预算耗尽、CRC→DMA 超时隔离；main 生产函数覆盖各域不同 generation/requestId、SAFE 前上下文及旧 seq 拒绝。矩阵层两层扫描退出的最后守卫是代码审查覆盖，不冒充硬件故障注入。编译/输入 SHA 与镜像见 [最终构建清单](artifacts/reference_fixture/crc-followup-build-manifest.json)。


## a0460b8 实机复测：恢复生效但仍有故障

`crc-fixed-res8-01` 实际采集 19.278 s、237 个完整 RES 帧（seq177–413），ASCII CRC/identity/序号缺口均 0，随后电池异常进入三次有界恢复并 DEGRADED；本次仍 FAIL，没有达到 120 s。新鲜 CRC 重试已实际恢复至少六次，ADSHEALTH 在 engineFrameCount=64/128 时 err=0。原始数据继续保留，不能把只通过这两个边界写成 512 通过。

五条已发出的 ADSSPIBYTES 都为 received XOR computed=1：99/98、27/26、5F/5E、B7/B6、79/78（前者为收到值）。但 OPEN 常用负满量程 raw=0x80000000 的正确 CRC 是31，本来末位为1，所以其它 OPEN 点通过 CRC 不能排除末位采样问题，也不能从“S1D1独占报错”推定 PGA 根因。

seq413 在进入电池转换前即退出，BATREG saved/configured/restored均unknown，原错误0x108；初版把这一未知入场状态说成restore_failed，且打印report_continue，均不足以定位实际条件。追加 BATCTX/BATCMP 保存实际物理快照、shadow和首次比较差异，独立 RESET 语义，只屏蔽手册保留位；未知/恢复失败明确 isolate_ads。这个变化不自动同步不符缓存、不放宽检查。

SPI 时序对照来自实际观测和手册约束。ADS CS=-1，不能通过设置CS hold制造不存在的片选控制。Espressif 仓库有其它 ADC 的相关 CPHA1末位问题报告，但不构成本板因果证明：[issue 17908](https://github.com/espressif/esp-idf/issues/17908)。当前候选仅在 ESP32-S3、请求2 MHz时把duty改为64，mode1、CRC、频率及重试预算不变；实际IDF divider计算为高125 ns/低375 ns，满足ADS1262高低≥40 ns、DOUT上升沿后≤60 ns有效的要求。未使用S3不支持的sample_point，也未超频。此处没有示波器信号完整性证明。

候选源码 `8b0e651`，随后 `6a8f502` 加上只有CRC校验成功才增长的 crcValidEven/crcValidOdd 计数，确保A/B包含末位0的有效非饱和数据。第一次 timing-candidate-build 仅为编译检查，期间补充了计数器，因此它未烧录、不作为最终输入对应证明；最终以 timing-final-build-manifest 为准。


### 同一实录中的 rail 平台跃迁（尚不可校准）

`crc-fixed-res8-01` 的237帧不能合并成一个10619.110 Ω校准均值：seq177–276的100帧均值9978.924 Ω、railSpan5167748 µV；seq277–377的101帧均值11086.031 Ω、span5036352 µV；seq378–413均值11087.434 Ω、span5036192 µV。276→277单帧跃迁+1101.586 Ω，S1D1全程gain=1、Rref=10000不变；估算AINCOM同时下降19323 µV。277→378是101帧、8.054866秒，本录没有完整RAIL_REFRESH事件，不能冒充已抓到该事件原文。

在当前零offset公式下重建的差分电压三段仅约1.6 µV差；保持首段差分量，只替换railSpan，预测后两段电阻与实录均值差0.028/0.008 Ω。**这是从发布值和公式进行的数学重建，不是实录raw ADC**。证据支持优先审查rail采样/快照换算，仍缺独立DMM确认哪组rail真实；不能写offset去掉平台跃迁，也不能据此宣称VOLT因果闭环已经完成。

## 8d6d656：rail 专用配置与寄存器 guard 的实机结果

手册 ADS1262 §9.3.5 要求内部供电监测使用 PGA enabled/gain 1、chop OFF。原 helper 继承末点 MODE2=8F（bypass），而首次 RES route 是 PGA1，违反同一测量的配置前提。`8d6d656a6272af7f19e338b594e98d582762737e` 增加专用监测配置、物理全配置核验及事务结束后的 RAILPROFILE/RAILSAMPLE；全恢复成功才发布结果。没有平滑、改变阈值或永久限速。RREG 增读一个合法尾寄存器后丢弃，保护请求字节免受响应末位故障；请求位仍严格校验。duty64 候选实机失败，恢复128，数据CRC所有位保留。细节与实际27场景回归见 [rail/guard证据](artifacts/reference_fixture/ads_rail_profile_guard/notes.md)。

最终 app-only 烧录镜像 SHA256 `0414c6b4fdbdaa5f964280e608cae15011c9bf22b547d8026b761f2680f5e34f`，1531360 bytes；版本 `8d6d656-dirty`，构建清单包含源码hash/dirty hash，不把文档dirty隐藏。见 [build](artifacts/reference_fixture/rail-guard-final-build-manifest.json)、[flash](artifacts/reference_fixture/rail-guard-final-flash-result.json)。32个 `tools/test_*.py` 入口全部退出0，见 [执行清单](artifacts/reference_fixture/last-build-test-results.json)。这包括HIL工具单测，不把它称作实机HIL。

`rail-fixed-res8-01` 连续139.683 s，1636完整帧，seq611–2246，ASCII CRC/identity/序号缺口均0，无MFAULT或复位；实际数据跨度139.491 s，约11.72 FPS。17个rail段未复现约1100 Ω台阶：S1D1有效1628帧均值9986.593 Ω、范围9973.602–9995.105 Ω，段均值9980.013–9990.703 Ω，刷新边界最大单帧变化11.357 Ω。仍有8帧X02，最后可见累计93次ADC CRC错误、75次重试；54条完整坏CRC记录均xor1。不能用ASCII CRC全通过掩盖ADC错误，也不能把本次RES结论直接当E1 VOLT因果证明。

S8D8仍1636/1636 OPEN，用户尚未确认接线；60个悬空点符合OPEN语义。独立evaluator退出1/FAIL，绝非精度PASS。health128/256/384/512全部有完整err0记录，完整记录延伸至1600。1472字段与I2C50交织，1088/1408没有完整记录，不能补成原文。USB窗口sinkDropped/partialWrite/timeout增量均0，driverAcceptedBytes增加1704170，原始PC RX为1832490bytes；累计83/1/83来自窗口之前。DEBUG恢复查询验证。

`rail-fixed-mixed8-01` 的RCCCCCCR在31.141 s失败：seq4145 rail事务36的样本CRC为0x109，配置与恢复verified；上层GetRail bool接口丢失err，MFAULT/ADSFAULTCTX最终写成0x103。ADSFAULTTXN仍保存original0x109。attempted/acquired均0，不能说64次转换失败。故障后CONFIG可查询DEGRADED，BAT显示lastGood随时间正常老化，没有复位或假report_continue。

`rail-fixed-switch50-01` 成功核验3次切换后，在第4次CAP的CONFIG回复发现日志穿插：原始行含 `...,boundary=4160,stPR,d=s,...`。终态与查询尚未完整送达，不可把缺字段当权威状态矛盾，也不通过Host忽略坏行来掩盖。该次50切换验收FAIL，原始记录保留。后续修复和复测单独追加。

`rail-fixed-cap1-recovery-01` 在上述混合故障与查询交织之后成功恢复CAP单行，采集139.810 s，10239完整帧；解析CRC/identity/缺口/重置均0（约73.2 FPS）。这里只覆盖S1行，S4D4/S5D5为NOT_OBSERVED；不能把高FPS或恢复成功写成33 pF校准通过。原始记录与独立analysis均保留。

CAP1补充性能：稳态SF50窗口cfps72.43–73.50（median73.28），序列速率73.1945 FPS；USB sinkQueued+11478、driverAcceptedBytes+4694515、rxBytes+77，sinkDropped/partialWrite/timeout均+0。heap initial/current/lifetimeMin=8378600/8350952/8337836 bytes，不能当内部heap分项；剩余栈低水位log13328、transport3996、BLEtx3300、BLEctrl5228、serialCtrl2124 bytes，USB真实查询为3092/4096 bytes。STK的usb=0/0不是实测栈耗尽。CPU、最大连续块、内部heap分项NOT_RUN。204个H50的e/sat/st/rc及FB50 failed/recovered均0；5次电池rail_invalid保持modified=0/restore=unchanged，没有升级矩阵故障。

## 再次实测反例驱动的边界与输出修复

混合事务补丁保留rail最大年龄100，不把101的旧快照伪装成合法。rail整个250 ms事务共用一次新鲜CRC重试，坏转换不计有效采样、不更新cache；重试耗尽保留原错误，cleanup独立。Matrix GetRail由bool改为esp_err_t，原始SPI/CRC错误不再折叠成0x103。Mixed子段禁止自行执行辅助事务；预备阶段在任何点采样前选择本profile实际RES行，再进入有意激励、完整监测、恢复与稳定。缺RES/真实隔离时不强开参考；电池与ADSCHK仍只在完整帧边界串行。

USB交织找到可直接复现的初始化缺陷：自定义VFS路径 `/dev/sensorarray` 为16 bytes，而当前ESP-IDF `ESP_VFS_PATH_MAX=15`，注册返回INVALID_ARG。此前TX task已经存在，Queue只看task，因此协议走新TX、stdout仍走旧console。原始CONFIG恰在64B边界插入CRLF诊断，sink丢弃/timeout增量0也会发生；不能靠忽略坏行判通过。补丁缩短路径、增加编译期限制与ready门禁，并公开stdio绑定/初始化结果。

另有依据充分但仍需实测的SPI候选：ADS手册印刷页86 §9.5/§9.5.1允许CS保持低时连续完整命令，NOP=00h；页70 §9.4.7.2规定RDATA完成后恢复命令响应。ADC1读取现在是完整RDATA1响应后紧接一个NOP，共64 clocks（STATUS+CRC启用时），只解析原6 bytes响应。NOP接收字节无定义，直接丢弃；它不是增加RDATA长度，也不修补CRC位。该变化把CRC移离ESP事务末尾，但不改变ADC自身RDATA结束边界；不能从RREG guard成功推导它必然有效。driver双target回归实际通过，覆盖STATUS/CRC全部6种长度、合法even CRC、host事务尾bit故障、实际payload损坏仍拒绝、legacy入口及超时持久DMA。效果以随后的硬件记录为准。

统一固件提交 `3fb2a19a19b054ca9621ef089d961806a4740478`，34个host测试入口实际全部退出0（[清单](artifacts/reference_fixture/boundary-nop-final-test-results.json)）。前一次candidate sweep混合测试在公共接口尚未落地时失败，原退出1日志保留；接口完成后重跑全套通过。实际IDF构建/烧录均退出0，app SHA256 `10c624ee405bed75bc4f24c5bb0b0faf2f8d3097dd5b20cf7f700328fe2923d7`，1533584bytes，版本 `3fb2a19-dirty`（文档/证据dirty，源码已提交）。见 [build manifest](artifacts/reference_fixture/boundary-nop-final-build-manifest.json)、[app-only flash](artifacts/reference_fixture/boundary-nop-final-flash-result.json)。原NVS/calib与分区未写。

`final-res8-01` 是3fb2a19的NOP候选实录，**不是最终通过记录**。201个完整R帧（seq2–202）后seq203 rail事务3第5个有效采样遭两次CRC失败（accepted=4、crcErrors=2、crcRetries=1、terminalError=0x109），完整恢复成功、cleanup0，MFAULT准确保留0x109与RAIL_MONITOR，attempted/acquired0。实际采集16.865 s，未达到120 s；RDATA+NOP未消除末位xor1错误，因此后续撤回NOP，保留其它修复及其历史证据。

此次stdioBound1/initError0证实VFS注册已修，但末尾 `ADSFRAME,...err=0x109USBSTREAM,v=DEBUG,...` 暴露第二层问题：printf经VFS分块入队，优先控制回复可插在尚未换行的诊断中，且普通碎片可能因满队列丢失。collector因未识别完整ACK而报告restore超时；随后的只读查询明确DEBUG已经应用、CONFIG仍可查询DEGRADED、无复位。不能称控制器死亡。本录仅有一份完整USB计数快照；与后续同session只读查询相比，跨文件端点sinkDropped增加3，而partialWrite/timeout没有增加。端点之间包含未采集空窗，不能把这个差值冒充纯采样窗口统计，也不能把拼接误判为驱动部分写。需要按完整逻辑行入队，整行丢弃与有界计数。

## ebabe7e 最终软件版本与后续实机验收

交付前对已提交 `ebabe7e` 的全部34个 `tools/test_*.py` 再跑一次，全部退出0：[final-committed-test-results](artifacts/reference_fixture/final-committed-test-results.json)。旧serial-lines测试是在提交前dirty工作树执行，新清单明确对应已提交HEAD；两份记录均保留，不能用host回归覆盖以下实机FAIL。

NOP候选按实机反证在 `bd49168` 撤回，未回退其它修复。随后 `ebabe7e6816c0347c6d1f5187d5f7f801a7ef98d` 完成stdout/stderr独立有界组行，整行LF后入队，拥塞/超长整行丢弃并计数；借用既有normal槽，没有新增大PSRAM缓冲。legacy科学帧绕过组行，保持整packet与真实排队结果。两个历史USB反例均用对应未改生产版本复现；新增完整行并发/压力/恢复与legacy整帧回归通过。34个测试入口全部退出0，见 [最终测试](artifacts/reference_fixture/serial-lines-final-test-results.json)。

ESP-IDF实际构建成功并仅烧录app，SHA256 `2dfd717ec374c3d046c20316320b3e2199e3eeda22c4dd94d193336a1aad8ba2`，1534288bytes，`ebabe7e-dirty`；源码已提交，dirty仅文档/证据，输入逐文件hash和diff hash见 [最终build](artifacts/reference_fixture/serial-lines-final-build-manifest.json)，烧录校验见 [flash](artifacts/reference_fixture/serial-lines-final-flash-result.json)。没有NVS/calib写入。下列release-*是这份镜像的独立新实录，标签不自动表示验收PASS。

`release-switch50-01` 的严格“切换+参考点有效”测试在完成28次后，于第29次RES的S1D1发现X02而停止：seq2215两次CRC失败、一次重试，CELL valid0/fresh1/error=spi，原始data/CRC保留。第29次的MAPP、CONFIG和三帧身份本身一致，没有MFAULT或串口回复拼接；因此不能把电气有效性失败误判成模式身份失败，也不能把28次说成50次通过。后续将控制/身份结果与测量有效性分别记录，但不删除或改写本次FAIL。

`release-cap8-01`：139.867 s，1615完整C帧seq300–1914，ASCII CRC/identity/缺口/截断均0，全部103360点fresh/acquired且有效，按设备时间跨度计算11.55234 FPS（主机闭合采集窗口为11.54666 FPS）。S4D4总电容均值92.125982824 pF（92.095913–92.149608），S5D5均值77.356788988 pF（77.335520–77.374912）；均为BASELINE_REQUIRED，不以标称33pF判精度PASS。USB完整起止差：sinkDropped/lineDropped各+65，discardBytes+2255，partialWrite/timeout/lineOversize均+0；说明有已计数诊断背压，科学帧未缺失，不能称全链路无drop。driverAcceptedBytes+1575046，PC实收1577924 bytes，DEBUG恢复确认。栈剩余最低log13064、transport3196、USB3092、BLEtx3296、BLEctrl5240、serialCtrl2284 bytes；heap initial/current/lifetimeMin=8378640/8350992/8337776 bytes，CPU/最大连续块仍NOT_RUN。

### 最终实机结果及限制

| ebabe7e 场景 | 实际结果 | 结论边界 |
|---|---|---|
| CAP8 `release-cap8-01` | 139.867 s / 1615帧，CRC32/identity/gap=0 | 数据链筛查通过；33pF BASELINE_REQUIRED |
| MIXED8 `release-mixed8-01` | 139.632 s / 807帧，CRC32/identity/gap=0，无MFAULT | 夹具评估FAIL：S8D8全部OPEN；未标精度PASS |
| 严格切换 `release-switch50-01` | 完成28次，第29次S1D1 X02 | FAIL，终态/身份本身正确 |
| 分项切换 `release-switch50-identity-01` | 50次控制身份PASS，200连续帧；25RES/25CAP | S1D1有效、S8D8仍OPEN；未判精度PASS，SPI仍有恢复的CRC错误 |
| CAP1+BLE并行 | Serial149.784 s；BLE140.001 s | BLE接收及两通路一致性已实测，USB有可计数数据缺口 |
| RES8 `release-res8-01` | 2.075 s / 26帧，seq18301 MATRIX_CRC/0x109 | FAIL，未达到2分钟；不能宣称ADS CRC根治 |
| 最终CAP恢复 | 22.332 s / 257帧，无CRC32/identity/gap/MFAULT | 仅恢复短窗，设备留在CAP8/DEBUG |

MIXED的S1D1全部807帧有效，均值9988.020786 Ω、范围9979.221–10000.112 Ω；S8D8全部807帧OPEN。S4D4/S5D5总电容均值92.019612608/77.251309171 pF，均缺同点卸载基线。8次整帧预备在seq3517、3617…4217，每100帧、实际17.289930–17.299241 s；完整预备119.757–120.933 ms，不能只拿局部monitorUs称事务4ms。原错/cleanup均0、restored1。807条本次health全部err0，engine1331→2137、entry1330→2136，实际覆盖1408/1536/1664/1792/1920/2048；SetMode没有重置累计计数，混合切换令本段每帧healthDue=1。SPI错误累计28→40仍存在，不能将寄存器健康通过等同于ADC数据零错误。USB仅诊断行丢弃+30/928bytes，科学帧无缺口。

50次分项切换约21.710 s，所有MACK/MAPP、CONFIG与每次至少3帧身份一致；运行内200连续完整帧（100CAP/100RES）无ASCII CRC或序号错误。S1D1该次100个RES帧有效，但health检查点SPI crcErrors51→61、retry42→52，限频事件显示重试恢复；S8D8仍全部OPEN。退出码只代表控制身份断言，summary明确measurement_status=NOT_PRECISION_ACCEPTED。先前严格FAIL未删除，不靠重试到成功宣称测量稳定。

BLE由Linux hci0连接实测地址80:B5:4E:CD:D9:AE/name CscArray_CDD9AD，纯订阅FF11/20/30，不发应用控制命令、不另开Serial。原始notification、重组消息及科学数据均保存。140秒实际收到9131完整C帧（其中9128个1行帧），信封/帧CRC、缺片、身份/gap均0。固件MTU=247；Bleak显示23已标为后端未验证值。两个通路共有9049帧、3511654字节C/D/K完全一致。共同1行范围seq5505–14630，BLE连续，Serial缺少的78帧全部在BLE实录中存在；OT50的USB包丢弃合计78与缺口吻合。USB全段sinkDropped+455=日志lineDropped377+数据包78，discardBytes12834，driver partialWrite/timeout无增量。不能称新日志完全无背压。BLE检查点dropD/L/C增量0，不推导绝对送达；结束sentD包含cleanup边界，不能用9134−9131虚构丢3帧。独立重放/字节对照见 [BLE/Serial审计](artifacts/reference_fixture/release-ble-serial-parallel-01.audit.json)。

BLE连接稳态capture中位65.37 FPS、emitted65.36、BLE提交65.4；主机闭合窗口实收65.221 FPS。这是CAP1，不是用户E7的VOLT1，不宣称复现或维持97.99 FPS。采集限速配置仍关闭，最终MODE查询captureCap=0；未通过永久5 FPS掩盖问题。Serial事件只有相对时间，无法与BLE绝对monotonic起点精确对齐，因此跨通路主机延迟NOT_RUN，重叠按同session/seq及设备时间核对。

最后一次RES失败不是周期健康回读：seq18301 engine2746、modeEntry2718、healthDue0、precision0；S1D1两次CRC失败（receivedDB/computedDA）耗尽一次重试，MATRIX_CRC/0x109，原RES/RES/MATRIX身份与cleanup0均保存。其余63点OPEN已采，spi=1，不是64次SPI失败。控制查询持续可用。随后CAP请求92、ROWS请求93正常应用，257帧恢复无复位；最终CONFIG为session26888A60879294CC/boot10/rev95，CAP/rows8/CCCCCCCC，mgen/mrid86/92、rgen/rrid8/93、pgen/prid87/92，err0。USBDEBUG已确认，CAL仍source0/schema0/valid0，未写校准。

最终链接DIRAM 286999/341760 bytes（83.98%，余54761），app1534288bytes；这是链接占用，不是运行内部heap。CPU利用率、最大连续空闲块、Windows重枚举、示波器信号完整性、Wi-Fi端到端与多transport同时变更配置的HIL仍NOT_RUN；相应host模型测试不冒充实机。VOLT绝对精度/安全外部校准及rail四组合未跑：用户明确没有独立DMM轨读数，本板透明内部隔离不可用，不能输入内部估算作外部校准。

**完整可靠性与精度验收仍未通过。** 已修复的软件路径有构建/回归/上述实机证据；ADC CRC底层原因仍需MISO/SCLK/CS时序取证，不能靠改低位、关CRC或无限重试。S8D8接线仍待用户核实，33pF同点空载基线、元件实测值及误差限未提供。没有自动push、发布、整片擦除或CAL=SAVE。

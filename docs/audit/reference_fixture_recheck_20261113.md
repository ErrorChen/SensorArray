# 2026-11-13：接触修复、外部轨输入与悬空口近似背景复测

**S8D8 接触问题已由实测确认改善；ADC SPI CRC 尚未解决，完整可靠性/精度验收仍未通过。** 本轮没有改动或烧录生产固件，没有关闭 CRC/watchdog、增大重试预算、调 offset 或写 NVS。唯一设备校准变更是 RAM 中的 `RAILCFG=3350000,-1795000`。最后恢复 CAP8、USB DEBUG、电池周期1000 ms；外部轨输入保留至重启或显式替换。

起始仓库 HEAD `a73df5e9c8aaafb81be810326ef9ed19a3dec682`，clean，与已推送 origin/main 一致。前次烧录生产源码为 `ebabe7e`，app SHA256 `2dfd717ec374c3d046c20316320b3e2199e3eeda22c4dd94d193336a1aad8ba2`。本次 BUILD? 报 IDF v5.5.5/ESP32-S3/PROTO2；它未直接提供 app SHA。设备仍是此前已记录的 session `26888A60879294CC`、boot10，初始 CAP 累计约865万帧；二进制身份通过前次烧录记录和同启动会话关联，本轮没有读回 Flash 重新计算 SHA。访问口 `/dev/ttyACM0`，USB Serial/JTAG，ADS? 仍为1262/ADC2不可用。

## 新输入与适用边界

用户确认 S8D8 松动并已修复。S1D1/S8D8仍为标称10 kΩ、S4D4/S5D5为标称33 pF，其余60点悬空。元件实测值、容差和系统验收限尚未提供。

独立万用表读数：AVSS对GND约−1.795 V、上下0.005 V；AVDD对GND为3.33–3.37 V。取AVDD范围中点3.350 V与AVSS代表值−1.795 V输入，跨度5.145 V。独立组合端点为5.120–5.170 V；这是给定读数范围，不是统计置信区间。用户认为工频噪声可能由测量引入、真实波动小于10 mV，此判断尚无独立波形验证。仪表型号、校准状态、实际探测位置尚不确定；不能直接排除R54等供电路径压降。[输入原文含义与来源记录](artifacts/reference_fixture/20261113-user-inputs.json)

`RAILCFG` 在CAP/INTREF OFF时入场，RACK/RAPP id99、应用边界seq8659568，RAIL?确认`src=external,rail=5145000,avdd=3350000,avss=-1795000,split=external_measured`。setter只改内存，未启动透明内部参考。ARL的source是固件来源分类，不代表这些代表值具有可溯源精度；uncertainty仍unknown。[应用记录](artifacts/reference_fixture/20261113-apply-external-rails-01.summary.json)

## 实际采集与首次故障

以下标签均在 `docs/audit/artifacts/reference_fixture/`，原始`.rx.bin`保留，tracked `.rx.bin.gz`解压逐字节相同。每次配置总预算150 s，准备/恢复占用其中一部分；发生MFAULT停止该次采集，没有自动循环重试。

| 标签（均以20261113-开头） | 实际记录 | 结果 |
|---|---|---|
| contact-fixed-cap8-01 | 139.835 s，1614完整C，seq8656990–8658603 | CRC32/identity/gap=0，全部点fresh/acquired有效；电容精度仍未建立 |
| contact-fixed-res8-internal-01 | 45.686 s，540完整R，seq8659026–8659565 | seq8659566 MATRIX_CRC/0x109；engine3288、entry2746、healthDue0，cleanup0 |
| res8-external-01 | 54.846 s，658完整R，seq8659723–8660380 | seq8660381 MATRIX_CRC/0x109；engine3948、entry3288、healthDue0，cleanup0；无内部rail刷新 |
| volt8-external-01 | 首个VOLT采样seq8660383失败，无完整V帧 | 已确认INTREF0/REFMUX24/source external；healthDue1且同seq健康回读err0，随后数据CRC失败，非健康回读失败 |
| cap8-external-recovery-01 | 139.816 s，1601完整C，seq8660385–8661985 | CRC32/identity/gap=0，无MFAULT；新电池样本有效 |
| res8-external-batteryoff-01 | 139.662 s，1873完整R，seq8663070–8664942 | 无MFAULT但仍有CRC和1个X02；严格evaluator为FAIL，不是可靠性PASS |
| final-cap-recovery-01 | 26.104 s，299完整C，seq8665164–8665462 | CRC32/identity/gap=0；已恢复CAP8，仅恢复验证，不称两分钟筛查PASS |

VOLT summary中的0.501 s是collector在准备阶段已观察MFAULT后进入capture阶段的等待时间，不能当成0.5 s有效VOLT采集。首帧诊断有35次CRC失败、28次重试、27次恢复、7个终端SPI失败点；没有完整V/K科学帧，不能用该段评估VOLT噪声或周期跃升。[完整原始事件](artifacts/reference_fixture/20261113-volt8-external-01.events.jsonl)

## 电阻、rail与CRC的区分

| 配置 | S1D1有效值均值 | S8D8有效值均值 | 无效参考点 |
|---|---:|---:|---|
| 内部rail、周期电池开 | 9989.169 Ω（540点） | 9991.152 Ω（538点） | S8D8有2个X02；两点OPEN均0 |
| 外部rail、周期电池开 | 10167.366 Ω（657点） | 10169.230 Ω（656点） | S1D1有1个、S8D8有2个X02；两点OPEN均0 |
| 外部rail、周期电池关 | 10167.325 Ω（1873点） | 10169.305 Ω（1872点） | S8D8有1个X02；两点OPEN均0 |

S8D8此前连续OPEN已消失，支持接触修复有效；仍存在的X02是ADC校验问题，不能归为OPEN。内部rail这次6个事务按101帧间隔刷新，5个完整邻帧边界的最大电阻跳变为S1 6.330 Ω、S8 7.528 Ω，未见旧约1100 Ω台阶；这不是原VOLT跃升因果验收。

当前零offset和AINCOM中点估算下，生产公式可化为`R = Rref × (2.5 V / (d + span/2) − 1)`。保持同一差分模型量d，仅把内部约5.167 V跨度换成外部5.145 V，就预测S1/S8上移177.772/177.806 Ω，与实录上移178.197/178.078 Ω接近。实际调用生产C数学复核2391个有效值，原值重现无不符。d来自发布值的模型反演，**不是实录raw ADC**；本轮没有同期有效CELL原始码查询。[公式复核及源码SHA](artifacts/reference_fixture/20261113-external-rail-res-model.ads.json)

固定同一模型d，用户跨度端点给出S1约9965.643–10372.391 Ω、S8约9967.540–10374.366 Ω。该敏感度范围说明代表轨输入尚不足以支持高精度判定，不是测得的电阻波动或不确定度置信区间。没有根据10 kΩ反校rail或拟合offset。

BAT OFF对照完整帧窗内，没有电池事务或rail刷新；112个电池快照的run/valid/invalid/lastGood不变，reason=disabled。仍有67条限频ADSSPIBYTES（另1条属于准备帧），均为reported received XOR computed=1；相邻诊断计数边界增加96次CRC错误、90次重试。计数上端位于末完整R之后一帧的健康回读，不冒充逐帧精确归属。关闭辅助事务仍有坏CRC，故它们不是错误发生的必要条件；单次没有升级MFAULT不证明CRC已修复。

周期电池已按原值1000 ms恢复并查询确认。最终BAT有新的有效4098 mV样本、lastGoodFrame8666319，晚于外部rail应用边界；restoreFail仍0。此前CAP rail_invalid依赖得到改善，但4098 mV仍是使用代表轨输入的固件估算，并无独立电池DMM精度验收。

## 用户授权的悬空口近似背景

新增只读工具 `tools/estimate_floating_cap_background.py`。每帧排除四个接元件点，固定使用同行7点、同列7点或全60悬空点的median，与**同一帧**目标总C相减。任一固定donor不完整/无效/不新鲜时，该方法该帧不产生估值，不用0填补、不缩小总体；不同session/revision分组，负差和尖峰保留。

第一段新CAP8结果如下。均为跨点近似差值，非同点卸载测量，也没有选择最接近标称33 pF的方法。

| 测点 | 原始总C均值 | 同行背景相减 | 同列背景相减 | 全60点背景相减 |
|---|---:|---:|---:|---:|
| S4D4 | 92.490262 pF | 32.409237 pF | 13.511588 pF | 32.408135 pF |
| S5D5 | 77.530541 pF | 17.449767 pF | 11.773416 pF | 17.448414 pF |

60个悬空点的时间median范围57.197345–80.071068 pF，列结构明显；三方法均值跨度分别18.897649、5.676351 pF。同行/全局与同列结论差异远大于短期帧间抖动。工具输出`CROSS_CELL_PROXY / APPROXIMATE_DIAGNOSTIC`，正式精度验收仍未建立；不能由跨点背景强制产生33 pF或写校准。[逐帧近似结果](artifacts/reference_fixture/20261113-contact-fixed-cap8-01.proxy-cap.json)

## 验证、输出与最终状态

新增工具12项测试实际通过，另外现有fixture、capture、真实text protocol测试也各实际通过；没有因文档/只读分析工具变化重烧固件。[命令、退出码与源文件SHA](artifacts/reference_fixture/20261113-proxy-tool-validation.json)

11次串口查询/配置/采集的实际调用与观察到的退出码见 [硬件命令清单](artifacts/reference_fixture/20261113-hil-command-results.json)；设备线上命令、回复和相对时间在对应events文件。生产文件未改、测试源码SHA和本地链接检查见 [最终核验](artifacts/reference_fixture/20261113-final-verification.json)。

7次科学数据采样均实际重跑严格evaluator：3个CAP记录为REVIEW_REQUIRED/exit0，2个电池开RES、1个电池关RES及VOLT为FAIL/exit1。脚本完成或退出0不表示精度PASS。11份新原始接收记录共7,157,316 bytes均已无损归档；没有覆盖旧记录、补全截断行或改CRC。[重放命令与归档SHA](artifacts/reference_fixture/20261113-replay-and-archives.json)

实录由独立审计再次核对，6份逐场景audit与SHA见 [独立审计索引](artifacts/reference_fixture/20261113-independent-audit-index.json)。按(N−1)/设备时间跨度计算，外部CAP为11.4552 FPS、外部RES电池开为12.0142 FPS、电池关为13.4113 FPS；不可与不同时间口径或不同模式的旧高FPS直接比较。测试结束已恢复周期电池，不以永久关闭功能宣称修复。

CAP首段USB诊断行丢弃+63，外部CAP段+56；这两段科学帧均无缺口，driver timeout/partial无增量。BAT OFF完整对照的USB drop/timeout/partial/discard增量均0。串口无人读取的间隔仍增加累计timeout，不能将这些间隔差值混进采样窗口，也不能声称全程无背压。CPU/最大连续块、Windows枚举、示波器SPI波形、同点卸载和精密电压标准均NOT_RUN；BLE/Wi-Fi本轮未复测。

最终CONFIG为同session/boot10、rev114，CAP/rows8/profile CCCCCCCC，mgen/mrid95/111、rgen/rrid15/112、pgen/prid96/111。USB DEBUG，BATPERIOD=1000 ms，INTREF0/REFMUX24，CAL仍source0/schema0/valid0。外部RAIL保留5.145 V，calUs没有重写刷新。没有自动push或发布。[最终只读状态](artifacts/reference_fixture/20261113-final-state-01.summary.json)

下一步优先取得SPI SCLK/MISO/CS时序证据；在可以测量时确认轨电压探测位置，并在同一RES窗口同步保留CELL原始码和RAIL快照。当前可以继续用跨点近似做变化观察，但不能把上述模型差异当成33 pF精密校准已完成。

# 10 kΩ / 33 pF 参考夹具验收

用户指定以下实装夹具。它是显式选择的验收配置，不写入固件默认值，也不把测量结果强制改成标称值。

| 测点 | 元件 | 标称值 | 验收用途 |
|---|---|---:|---|
| S1D1、S8D8 | 高精度电阻 | 各 10000 Ω | RES，两个不同路径 |
| S4D4、S5D5 | 电容 | 各 33 pF | CAP，同点空载差分 |
| 其余 60 点 | 悬空 | 无有限参考值 | RES 明确 OPEN；CAP/VOLT 原始诊断 |

配置在 [`reference_10k_33pf.json`](../../tests/fixtures/reference_10k_33pf.json)。元件实测值、参考容差及系统验收误差限未知，分别保留 null。不能沿用旧工具的 25% 电阻容差或 5–20 kΩ 粗筛范围宣称高精度通过。

## 数值含义

CAP 生产路径 `sensorarrayMeasureComputeFdcFrameCapTotalPf()` / `sensorarrayMeasureFdcComputeCapacitancePf()` 发布 LC 总电容，包括板上固定电容与寄生。验证 33 pF 需要 S4D4、S5D5 **各自移除标准电容后的空载记录**，保持其它接线、量程和测量条件一致。不能减去其它悬空点均值；`CAL=ZERO` 处理 ADS 残差，不是这项空载测量。没有基线时保留 total pF，结果 BASELINE_REQUIRED。

RES 的 60 个未装点预期为 fresh、acquired 的 `X0D`（OPEN=13）。SPI/DRDY/rail/路由失败、STALE、SHORT 或其它无效值都不是开路通过。两只电容在 RES 下的建立/稳态表现另记。CAP 悬空不要求 0 pF；VOLT 悬空不要求 0 V。此夹具没有电压标准，VOLT 绝对校准为 NOT_APPLICABLE。

ROWS=1 只覆盖 S1D1；ROWS=4/5 依次包含 S4D4/S5D5；ROWS=8 才覆盖全部四点。混合行采用 `ROWMODES=RCCCCCCR`，S1/S8 行为 RES，S4/S5 行为 CAP，保留逐行八列采集。未采到的点为 NOT_OBSERVED。

两只同为 10 kΩ 的标准不能同时唯一拟合增益和 offset；两只同为 33 pF 亦然。四个点不外推成其它 60 点的校准。工具只评估证据，不自动拟合或 CAL=SAVE。

## 记录与回放

原始 RX 字节、带单调时间的收发事件和分析 JSON 分开保存，不补齐截断行。预检与实录回放：

```sh
python3 tools/reference_fixture.py --output /tmp/reference-preflight.json
python3 tools/reference_fixture.py \
  --input /path/to/capture.rx.bin --provenance recorded \
  --output /tmp/reference-report.json
```

具备同点空载记录时加 `--baseline-input /path/to/unloaded.rx.bin`，表示操作者明确声明基线满足同点及相同物理条件；软件不能证明电容已移除或温度/连接一致。不能任意挑选多个配置的基线拼接。

分析使用真实 ASCII parser，检查 CRC、fresh/acquired、无效原因及 header/trailer identity，按 session/revision/域/行数/profile 分组。旧协议身份明确 unverified；跨启动不比较计数大小。均值、median、stddev、range、标称偏差和坏样本计数保留。

退出 0 只表示没有检测到明确失败。NEEDS_LIMITS、BASELINE_REQUIRED、NOT_OBSERVED、SCREENING_INCOMPLETE、NOT_RUN 都不是 PASS。设置明确 `acceptance_abs` 后才判定误差限；参考元件不确定度还需纳入最终精度结论。输出以 exclusive-create 新建，拒绝覆盖旧数据。

## 实机步骤与边界

保存 BUILD/PROTO/CONFIG（若支持）、MODE/ROWS/ROWMODES、CAL/RAIL、USBSTREAM。实际板上版本独立于源码 HEAD。CAP、RES 和混合行分别先取得至少 120 s 采集窗口，再核对实际 ADS engine 计数覆盖原失败边界。总程序时间不等于有效采集时间。

MODE/ROWS/ROWMODES 一次一个请求，等待匹配终态并查询权威状态。MACK 不是已应用。记录参考点和全部悬空点，不删除尖峰，不用均值掩盖首次故障。结束时只恢复已查询确认的原 USBSTREAM 配置，恢复未确认则报告 UNKNOWN/ERROR。

本轮先在宿主 sysfs 找到 `/dev/ttyACM0`；普通沙箱隐藏设备节点，不能据此称整机 USB 消失。授权宿主访问后确认 Espressif `303A:1001`、MAC `80:B5:4E:CD:D9:AC`。初始固件 BUILD 为 v5.5.1/PROTO1；首次 pyserial open 记录 USB_UART_CHIP_RESET，故后续采用不显式操作 DTR/RTS 的 POSIX 采集，但不保证底层驱动首次 open 绝不会复位。

用户随后明确授权烧录。已备份原 Flash `0x000000..0x31FFFF` 到忽略目录 `validation_artifacts/`，确认分区表相同，首次仅写 bootloader 与 factory app，后续仅写 app，未写 NVS/calib 或执行 CAL=SAVE。首次修复镜像 SHA256 为 `e8c9bbdb4c515ce70573581ead256b406ea762fdb1761fea6ca9bab930e991cf`。烧录日志及后续实测证据在 `docs/audit/artifacts/reference_fixture/`。

最终已烧录版本为 `ebabe7e-dirty`，app SHA256 `2dfd717ec374c3d046c20316320b3e2199e3eeda22c4dd94d193336a1aad8ba2`。**完整验收未通过**：CAP8和MIXED8均完成超过120秒数据筛查，但S8D8持续OPEN且接线尚未确认；两只33pF缺同点卸载基线；最终RES8仍因ADC CRC在2.075秒停止。50次控制身份通过仅覆盖协议。用户没有独立轨电压读数，不能执行外部校准或据内部估算宣称VOLT精度合格。设备已恢复CAP8/USB DEBUG，校准未写入。

实际采集结果、已观察故障及追加修复证据见 [2026-11-04 HIL 记录](../audit/reference_fixture_hil_20261104.md)。

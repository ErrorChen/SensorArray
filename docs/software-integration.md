# Software integration / 上位机兼容说明

## 中文

上位机最小修改为：

1. 增加 CAP/VOLT/RES 选择，向既有控制通道发送统一 `MODE=<name>`。
2. 将 `MACK` 视为已排队，不立即切换显示；收到匹配 requestId 的 `MAPP` 后再提交 UI 状态。
3. 继续解析 CAP 的既有 C/D/K；为 `V`/`R` header 和 `P` chunk 增加分支。
4. 依据显式 `mode/unit/scale` 显示 pF、V 或 Ω，禁止从标签或数值范围猜单位。
5. 对 `Xhh`、valid/fresh/error masks 和 rail/reference validity 显示无效原因，不把它变成 0。
6. CRC 覆盖 header、D 和 P，排除 K；动态 `rows` 决定 cell 数。
7. 可选使用 `CELL?=SxDy` 展示最后完整 ADS 帧的诊断详情。
8. 在请求 VOLT 前，通过既有控制通道发送经实测且为微伏单位的
   `RAILCFG=<AVDD_UV>,<negative_AVSS_UV>`；把 `RAPP` 当作帧边界应用确认。
9. 将 `P` 值 `00` 显示为 PGA bypass，并记录 V/R header 的 `ir` recovered-retry
   telemetry；不要把它们解释成缺失字段。
10. 把 `ADSCHK` 的 accepted ACK 与随后相同 requestId 的 `ADSCHK/ADSCHKSTAT`
    关联；`ADS? chip=unknown,valid=0` 必须显示为未确认，不能回退成 ADS1262。
11. 解析 `BAT?` 的 `ABAT`、压缩 `AB50` 和 `BATPERIOD/BATNOW` ACK；用 `ageMs`
    和 `fresh/valid/reason/restore` 展示，不从电压推算 SOC。
12. 将 `ADS50/ADST50/SF50/OT50` 分开：capture、emitted 与 per-sink FPS 是不同
    指标，`OUTCAP` 不能被 UI 解释为采集频率设置。

无需改变 BLE service/characteristic、UDP port、Serial framing、transport parser
入口或 sink backpressure 策略。所有 sink 收到 Core 1 已格式化的同一 packet。
`tools/text_protocol.py` 是参考 parser，`tools/test_text_protocol.py` 提供 CAP/VOLT/
RES、CRC、invalid、PGA、ADSCHK、ABAT/AB50 cache telemetry 和 worst-size golden
tests。Serial/BLE/Wi-Fi receiver 都可重复使用 `--command ADSCHK=100`、
`--command BATNOW` 或 `--command BATPERIOD=1000`，不需要新增 transport endpoint。

兼容限制：旧软件仍可读取默认 CAP，因为 CAP wire bytes 保持原样；旧软件无法
理解 V/R，应忽略未知 header 而不是把它当电容。模式切换后的第一个数据帧必须
匹配 `MAPP` generation/requestId，旧 generation 数据应丢弃。

## Australian English

The host needs one mode selector, MACK/MAPP transaction handling, V/R header
and PGA-chunk parsing, and explicit unit/scale/error rendering. Existing BLE
characteristics, UDP port, Serial framing, shared command parser, and CAP bytes
do not change. Before VOLT, submit a measured microvolt AVDD/negative-AVSS pair
through `RAILCFG` and wait for its applied event. Treat acceptance as queued and commit UI state only on the
matching applied event. Render PGA `00` as bypass, retain the `ir` recovered-
retry telemetry, and discard old-generation packets after a transition.

For battery reliability dashboards, retain cumulative `validRun` and
`invalidRun` rather than displaying only the latest `valid` bit. Parse
`retry=<last>/<total>`, `unstable`, `timeout`, `spreadRaw`, and
`spreadMaxRaw` from both ABAT/AB50. A retry means firmware waited for a later
DRDY generation; it is not stale-value reuse. Treat any increase in
`invalidRun`, `unstable`, `timeout`, or `restoreFail` as an event that remains
visible even if a later sample is valid.

# 故障定位 / Troubleshooting

| 症状 / Symptom | 核对与解释 / Evidence and interpretation |
|---|---|
| MFAULT，整帧无效 / Entire frame invalid | 按 session/seq 关联 ADSFAULTCTX/REG/TXN，先看 attempted/acquired、首错、cleanup、healthReadbackDue 和实际 engineFrameCount。0x108 在此 IDF 为 INVALID_RESPONSE；0x107 为 TIMEOUT。 / Inspect the first failing stage and counts before cleanup. A failure before conversion leaves no acquired cells. |
| VOLT 台阶 / Common voltage step | 对齐逐点原始值、配置、rail 来源/事务、第一有效帧和墙钟。用有效外部双轨对照 battery 开/关。 / Align per-frame voltage against rail, battery, precision and health events. Preserve invalid/settling records. |
| 某电阻 OPEN / Reference reads OPEN | 查 X0D、fresh/acquired、行列及夹具接触；独立查看 SPI/DRDY 故障。电容使用同点空载或标明背景代理。 / Inspect contact, physical position and fresh masks. Retain proxy-baseline uncertainty. |
| 电池 unavailable / Battery unavailable | 分开查 rail_invalid、三样本 spread、restore 和 lastGoodKnown。恢复未知时查看隔离状态。 / Separate reference validity, sample stability and restore evidence. Use the actual last-good timestamp. |
| DATA 已切换而 UI pending / Data before terminal | 查询 CONFIG；按会话/rev 和显式 m/r/p 域关联。CAP legacy gen/rid 是 ROWS。 / Rebuild authority from CONFIG and check the immutable frame identity. |
| PC 找不到 COM / Port missing | 在主机重新枚举，查看 VID/PID、序列号及接口号，区分应用/恢复/ROM。 / Check OS enumeration, cable/power and operating mode. Missing one port name identifies a failed open. |
| BLE 缺帧或 CTRL 超时 / BLE gaps or control timeout | 查实际 MTU/CCCD、保护快照、拥塞、返回码与主机序列；连接后重新订阅 FF11/20/30。 / Check negotiated MTU, subscriptions and endpoint gap deltas. Query terminal records for outstanding requests. |
| panic、启动失败 / Panic or allocation failure | 保存 BUILD、boot、PANIC/MEMFAIL、heap 各 capability、stackMinBytes 及时间窗。用该 ELF 解析回溯。 / Preserve the exact ELF, retained registers and allocation observations before restarting or changing conditions. |
| UDP 无输出 / No UDP | 查 SoftAP owner 及 peer、输出 gate 和 WTERM。WIFI=STA 等返回 sta_nyi。 / Check AP admission, receiver address, queue and actual SDK return separately. |

[测试方法 / Reproduction methods](validation.md) · [命令 / Queries](command-reference.md)

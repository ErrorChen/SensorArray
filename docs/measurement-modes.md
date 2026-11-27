# 测量与路由 / Measurement and routing

| 路由 / Route | 物理路径与数据 / Physical path and output |
|---|---|
| CAP | 钳位前关闭并确认 ADS INTREF。两颗 FDC 各读四列，并行产出 LC 总电容，单位 pF。 / After disabling the internal reference, clamp the selected row and read four channels per FDC. The published quantity is total LC capacitance. |
| VOLT | 行钳位到 GND；ADC1 取各 D 输入、AVDD/AVSS 参考，输出有符号 µV。非隔离板（`railMonitor=unavailable`）使用外部 `RAILCFG` 或沿用最近一次有效 `RES` 内部 monitor 轨（均不再 100 帧过期）；若冷启动尚无任何有效轨则以 `0x106` / `RAIL_UNAVAILABLE` 拒绝。Unknown or out-of-range inputs carry explicit invalid reasons. / Clamp the row and measure the eight inputs relative to the configured reference and rail split. On `railMonitor=unavailable` boards, VOLT and passive C+V mixed profiles retain external `RAILCFG` or the last valid `RES` monitor rail without 100-frame expiry; cold boot with no valid rail rejects with `0x106` (`ESP_ERR_NOT_SUPPORTED`). |
| RES | 松开钳位后使用内部参考及 REFOUT 激励，配合 Rref 求电阻，整数 mΩ。REFOUT 以 AVSS 为基准。 / Release the clamp, establish the internal reference, then compute resistance against the reference resistor. Open and short have explicit statuses. |
| MIXED | `ROWMODES=CVVRRVVC` 指定八个物理行的 profile；`ROWS=1..8` 选择活动行前缀。每个活动行仍是八列。 / Each active physical row carries a single mode and eight columns. Reassemble M/MR/K by physical row number. |
| SAFE / FRONTEND_WAIT | 停止普通采样并独立记录数字控制与模拟可知性。U5 EN 上拉，选中行仍与 REF 相连。 / Ordinary acquisition pauses. The control and register observations have limited analog observability because U5 remains connected. |

Core 1 在帧边界停止旧前端、按安全顺序改参考和路由、读回、等待建立，然后才允许新的有效点。INTREF 的真实 OFF→ON 边沿启动参考稳定期限。辅助 rail/battery/ADSCHK 保存和恢复硬件与 shadow，restore 失败时隔离 ADS。 / At the frame boundary the owner quiesces the old frontend, sequences reference and routing, verifies the configuration and waits for settling. Auxiliary transactions save and restore hardware and cache state. A failed restore quarantines the frontend.

CAP 的 STATUS 只按 INTB/必要时的验证路径读取；读取四通道前须满足 DRDY 和 unread 条件。ADS 通过转换代数、CRC8、状态和输入范围管理 cache 与自动 PGA；`Xhh` 带原因。 / FDC freshness combines interrupt, status and data-read evidence. ADS cache and gain updates require fresh, verified conversions. A partial frame retains both usable cells and reason-coded failures.

[硬件连通表 / Connectivity](hardware/reliability_revision_notes.md) · [帧格式 / Frame format](output-format.md) · [主机契约 / Host contract](integration/firmware_host_contract.md)

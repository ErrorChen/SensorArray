# VOLT8 / E1 有限窗口复核 — Limited recorded-window review

ELF `892f586bdb131e77614c1ca72723c9247159f9d3a7bf5b49917004383f252677`，session `0119031D7DEF4971` / boot 1；原始 USB SHA256 `f98eae6169a07d7f69f7d3ca6327ce8465c2035790b358a52a075299f9e9343b`。

本窗 1775 个可信完整 V 帧，seq 62819–64593；CRC、配置身份及原始 token 独立复算通过。固定规则是同配置相邻序号、两端 valid/fresh/acquired/expected 且无 error，同向变化至少18mV的点达到可比点一半（至少2点）。111,792个可比点对中，有任意单点变化达到18mV的相邻帧对为0；多点候选0，全64点候选0。保留 1156 个 invalid token：`{'S8D1': {'X12': 1156}}`。不能跨 invalid 插值或宣称整板所有时刻稳定。

每帧 rail/电源值分布：`[{'ref': 'AVDD_AVSS', 'rail': '1', 'avdd': '3350000', 'avss': '-1795000', 'frames': 1775}]`；与 ARL external 值匹配 1775帧。相邻 age 差分布 `{'1': 1774}`，rail值变化0对，年龄回退0对。帧未单独编码 external 来源；ARL 的 raw=0/mon=0 不代表测得零，也不等于连续物理电源监测。

设备相邻帧间隔 51.562–161.332ms，中位 57.305ms；每101序号的设备时间为 6.640449–6.985284s，中位 6.848812s（1674个重叠完整区间）。不能称固定5秒周期或逐点采样间隔。

可见 28 条 ADSHEALTH，声明interval `{'64': 28}`、err `{'0x0': 28}`，独立健康窗 CRC 拒绝 0/185,295；120笔 BATRAW，真实三样本完整性True，原始/cleanup错误 `{'0x0': 120}` / `{'0x0': 120}`，INTREF `{'0': 360}`。ADS50为35条聚合（n分布`{'50': 35}`），不能恢复完整逐帧precision状态；只有对应ADSHEALTH明确给出precisionFrame。没有矩阵各点逐次raw ADC或同步模拟波形。

局部样例 / Local examples:

- S8D1 seq 63353→63354: -1793719→-1779003µV，Δ+14716µV；PGA字段0→0；其余点最大|Δ| 672µV，全点中位Δ -2.0µV。
- S8D1 seq 63357→63358: -1779225→-1793726µV，Δ-14501µV；PGA字段0→0；其余点最大|Δ| 606µV，全点中位Δ 57.5µV。

局部区间的可见序号诊断：`[{'seq': '63353', 'tag': 'ADSHEALTH', 'txn': None, 'err': '0x0', 'cleanupError': None, 'precisionFrame': '1'}, {'seq': '63357', 'tag': 'BATRAW', 'txn': '2136', 'err': '0x0', 'cleanupError': '0x0', 'precisionFrame': None}]`。这是序号邻近关联，不能证明因果；被动10k/33pF夹具其余60点未占用。不能宣布E1唯一根因、绝对精度或所有工况已解决。

The closed window contains 1775 CRC/identity-valid V frames. Independent checks preserve every signed integer and Xhh token. Across 111,792 comparable consecutive point pairs, 0 frame pairs contain a change reaching18mV; the half-of-available-points screen flags0, and the separate all64 screen flags0. The 1156 invalid tokens remain unchanged. Comparable-point counts per adjacent pair are `{63: 1744, 64: 30}`; missing/invalid intervals are never interpolated.

Header rail-value changes: 0; age decreases: 0; external-query value matches: 1775. This is calibration provenance, not continuous measured supply verification. Frame-to-frame device timing is51.562–161.332ms;101 sequence increments span6.640449–6.985284s, not a fixed five-second period. Per-cell sample timing is unavailable.

The signed local examples above preserve exact before/after values and the maximum change among all other points. The nearby explicit-sequence diagnostics are retained in local-context.json. These are temporal associations, not causal proof. The28 health snapshots show0/185,295 checked-response CRC rejections over their own bracket. Battery raw triples and aggregate precision logs exist; matrix-cell raw ADC codes and simultaneous analogue waveforms do not. The passive fixture has no known stable voltage stimulus, and the original E1 complete raw recording is absent. This window does not establish absolute precision, a unique physical cause, or universal repair.

复算 / Reproduce: run the command in `manifest.json`, then `validate_voltage_frame_review.py <output> --write-result` and `summarize_voltage_review_context.py <output>`. Each refuses overwriting its result. `context-manifest.json` binds this supplement to the original review and validation hashes.

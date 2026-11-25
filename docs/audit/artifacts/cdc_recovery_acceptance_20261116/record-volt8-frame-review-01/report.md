# VOLT8 原始帧复核 / Recorded VOLT frame review

状态 / Status: **OBSERVED_NOT_PRECISION_OR_CAUSAL_ACCEPTANCE**。仅为本次被动夹具的有限观察；绝对电压精度、原 E1 因果和全场景修复均未验收。 / This finite passive-fixture observation does not establish absolute voltage accuracy, the original E1 cause, or all-condition repair.

正式 USB 原始 SHA-256 / Formal raw SHA-256: `f98eae6169a07d7f69f7d3ca6327ce8465c2035790b358a52a075299f9e9343b`。
1775 trusted / 1775 selected complete candidates; seq 62819–64593; identity `{'elfSha256': '892f586bdb131e77614c1ca72723c9247159f9d3a7bf5b49917004383f252677', 'session': '0119031D7DEF4971', 'bootId': '1'}`.
invalid 点 / points: 1156; 非连续或不可信相邻对 / rejected adjacent pairs: 0.

固定观察规则 / Fixed descriptive rule: 同配置、seq+1、两帧 CRC/身份/结构可信；仅比较两端 valid/fresh/expected/acquired 且无 error 的点。至少两个点同向变化 ≥18,000 µV，且占可比点至少一半，记为候选。另列完整64点同向变化，不能把半数或少数点称为全板跃升。 / Require consecutive trustworthy frames and usable points at both endpoints; at least two same-direction changes ≥18 mV covering at least half the comparable points flag a candidate. An all-64-point condition is reported separately.

可比相邻对 / Eligible adjacent pairs: 1774; 多点候选 / candidates: **0**; 全64点同向 ≥18 mV / all-64 candidates: **0**; 至少一离群变化 ≥18 mV / any-point pairs: 0.
相邻设备帧间隔 min/median/max / adjacent device interval (µs): 51562 / 57305.0 / 161332。
101 序号间隔 min/median/max / 101-sequence device interval (µs): 6640449 / 6848812.0 / 6985284，n=1674；主机收件时间另存，不称固定5秒周期。 / Host receipt intervals are separately retained; no fixed five-second period is inferred.

外部 rail / External rail: 每帧保留 ref/rail/avdd/avss/age；帧本身不含 src=external，来源只能与真实 ARL 查询及其原始字节关联。 / Every header retains its actual rail fields. The frame has no external-source field; external provenance comes from recorded ARL evidence, not invented per-frame metadata.
ARL 外部快照数 / external ARL records: 1; 可信帧值匹配 / header-value matches: 1775/1775; rail 值变化对 / changed pairs: 0; age 回退对 / age decreases: 0.

可关联诊断 / Observable diagnostics: 仅把真实 seq 相同的 BATRAW/BATREG/ADSHEALTH 记录列为序号关联。BATRAW 是电池原始 ADC 三样本，不是64个矩阵点的 raw ADC。ADS50 是50帧聚合，缺逐点/逐帧标识时不能反推 precision 或 raw。诊断 CDC 是 best effort，缺记录表示未观察到。 / Exact-sequence associations do not prove causality. Battery raw triples are not matrix-cell raw ADC codes; aggregate ADS50 cannot identify every frame's precision state. Missing best-effort debug records remain unobserved.
本窗诊断标签 / in-window diagnostic tags: `{'BATREG': 360, 'BATRAW': 120, 'ADSHEALTH': 28, 'ADS50': 35, 'ADST50': 35, 'AB50': 35}`。
ADSHEALTH 计数差 / counter deltas: `{'crcValidEven': 91193, 'crcValidOdd': 94102, 'crcErrors': 0, 'crcRetries': 0, 'engineFrameCount': 1728}`；其分母为独立健康观察窗内已校验 ADC 响应，不能当帧数。 / These counters cover checked ADC responses in a separate health bracket, not the frame denominator.
BATRAW 记录 / records: 120; 原始/收尾错误 / original/cleanup errors: `{'0x0': 120}` / `{'0x0': 120}`。
可见 RAIL_REFRESH / observed records: 0；未观察到不等于证明从未发生。 / Absence in best-effort logs is not proof that an event never occurred.

夹具 / Fixture: S1D1/S8D8 为10k电阻，S4D4/S5D5为33pF电容，其余60点按现有夹具定义悬空；本窗没有外部已知稳定电压源或同时模拟波形，不能将悬空点当0V标准或将共同变化唯一归因供电。 / The fixture has two nominal resistors, two capacitors and 60 unpopulated cells; no controlled voltage source or simultaneous analogue waveform establishes voltage accuracy or unique cause.

候选及最大变化样例 / Candidate and largest-change examples:

- 无满足上述多点规则的候选。 / No multi-point candidate meets the stated rule.
- S8D1, seq 63353→63354: -1793719→-1779003 µV; Δ=14716 µV; PGA 0→0.
- S8D1, seq 63357→63358: -1779225→-1793726 µV; Δ=-14501 µV; PGA 0→0.
- S1D2, seq 64154→64155: -1791500→-1793857 µV; Δ=-2357 µV; PGA 0→0.
- S1D2, seq 64124→64125: -1791498→-1793832 µV; Δ=-2334 µV; PGA 0→0.
- S1D2, seq 64128→64129: -1793755→-1791477 µV; Δ=2278 µV; PGA 0→0.
- S1D2, seq 64486→64487: -1791514→-1793761 µV; Δ=-2247 µV; PGA 0→0.
- S1D2, seq 63206→63207: -1793689→-1791458 µV; Δ=2231 µV; PGA 0→0.
- S1D2, seq 64239→64240: -1793889→-1791665 µV; Δ=2224 µV; PGA 0→0.

复算 / Reproduction: `manifest.json` 保存源文件/工具 hash、闭合校验、每帧原始字节范围和命令；`frames.jsonl` 保存所有候选原值及 invalid，`adjacent-pairs.jsonl` 保存所有连续对的有符号差，`lag101.jsonl` 保存所有101序号间隔，`diagnostics.jsonl` 保存真实诊断原文/偏移。`selected.rx.bin` 仅拼接原始完整帧范围，未重编码或修补。 / Source and tool hashes, closure checks and exact byte ranges are retained. All raw numeric values and invalids remain available; selected bytes are unchanged.


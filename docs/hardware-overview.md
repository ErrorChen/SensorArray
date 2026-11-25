# 板卡与信号 / Board and signals

S1–S8 是行，D1–D8 是列。线序为 S1D1…S1D8、S2D1…S8D8。主 FDC（默认 0x2B）CH0–3 对 D1–4，从 FDC（0x2A）CH0–3 对 D5–8。ADS1262 使用 ADC1，运行时识别器件。 / Row-major wire order follows the physical S and D labels. Two four-channel FDCs cover the lower and upper column groups. ADC capabilities depend on the detected device ID.

U5 公共端接 REF、EN 由 R25 上拉；Q1 对地钳位。U6/U7 切换列的 FDC/ADS 路径。ADS 的 REFOUT 相对 AVSS。RES 的参考电阻和 VOLT 对负轨的加载均影响换算。电池分压送 AIN8，AINCOM 有偏置，AIN9 接 AINCOM。 / U5 connects the selected row to REF. Q1 clamps REF. The D-line switches select frontends. Account for the negative rail and resistor loading. Derive VBAT from AIN8−AINCOM plus AINCOM relative to ground, then apply the physical divider ratio.

[电路图 / Schematic](../datasheets/circuit.pdf) · [ADS126x 手册 / Manual](../datasheets/ads1263.pdf) · [FDC 手册 / Manual](../datasheets/fdc2212.pdf) · [板级映射 / Board map](../core/board/README.md) · [HW01–HW06](hardware/reliability_revision_notes.md)

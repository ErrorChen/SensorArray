# Recorded fixture window: bss-matrix-02-cap8

Derived only from finalized formal USB raw bytes. No device commands, baseline substitution, calibration writes, or precision PASS.

Source raw SHA256: `a7249c244264b83f2571d7996057bf62e186993ef20fe2478eb890309642a93f`.
Host capture window: 5.295784917–125.295933173 s (relative receipt clock).
Transport replay status: `RECORDED`; fixture report: `REVIEW_REQUIRED`.

All intervals below describe observed samples. P05–P95 is not a confidence interval. CAP values are LC total capacitance; a same-cell unloaded baseline is absent.

## C / 8 rows / rev 90

Session `05DD194E2065ADB0`, boot `2`; sequences 67031–68420; 1390 frames.
Device first-to-last timestamp span: 119.977220 s; strict fixture screening: `SCREENING_INCOMPLETE`.

| Point | Unit | Valid n | Invalid n | Mean | Min–max | P05–P95 | Accuracy status |
|---|---|---:|---:|---:|---|---|---|
| S4D4 | pF | 1390 | 0 | 92.431562 | 92.413794–92.448923 | 92.423831–92.441396 | BASELINE_REQUIRED |
| S5D5 | pF | 1390 | 0 | 77.405972 | 77.389611–77.422744 | 77.397519–77.414335 | BASELINE_REQUIRED |

Floating RES cells meeting OPEN checks: 0/0 observed.
Floating CAP per-cell time medians (60 cells): 57.092659–80.003703 pF. Per-cell distributions remain in metrics JSON.

## Cross-cell CAP background proxy

All three fixed donor populations are reported. Missing/invalid/stale/unacquired donors exclude that frame/method; no shrinking, zero fill, smoothing, or selection by closeness to 33 pF.

| Point | Method | Paired n | Background mean (pF) | Difference mean (pF) | Difference min–max (pF) | Difference P05–P95 (pF) |
|---|---|---:|---:|---:|---|---|
| S4D4 | same_row_median | 1390 | 59.937209 | 32.494354 | 32.475599–32.513936 | 32.486389–32.502978 |
| S4D4 | same_column_median | 1390 | 78.993409 | 13.438154 | 13.418669–13.454715 | 13.427620–13.448096 |
| S4D4 | all_floating_median | 1390 | 59.938114 | 32.493448 | 32.469412–32.510666 | 32.485991–32.501514 |
| S5D5 | same_row_median | 1390 | 59.937579 | 17.468394 | 17.450519–17.481925 | 17.460077–17.476423 |
| S5D5 | same_column_median | 1390 | 65.713976 | 11.691997 | 11.676363–11.705982 | 11.684143–11.699427 |
| S5D5 | all_floating_median | 1390 | 59.938114 | 17.467859 | 17.442286–17.481980 | 17.459608–17.475375 |

## Reproduction and provenance

The selection manifest records source file hashes, tool hashes, invocation, every retained source/output byte range and its SHA256, excluded frames and truncated candidates. Only whole original ranges are concatenated; wire text and CRC are never rebuilt.

A frame's first header byte and final K byte must both arrive inside the explicit capture_start/capture_end window. This is at least as strict as the benchmark parser's complete-header/final-K rule; their counts are compared in the manifest.

The transport JSON audits diagnostics and malformed/incomplete material from the full original raw stream, so science-only slicing cannot erase a recorded fault. CAP proxy output is an approximate diagnostic, not a strict same-cell baseline.

- [raw](/home/csc/esp/workspace/SensorArray/docs/audit/artifacts/cdc_recovery_acceptance_20261116/bss-matrix-02-cap8-fixture-01/cap8.rx.bin)
- [selection](/home/csc/esp/workspace/SensorArray/docs/audit/artifacts/cdc_recovery_acceptance_20261116/bss-matrix-02-cap8-fixture-01/cap8.selection.json)
- [transport](/home/csc/esp/workspace/SensorArray/docs/audit/artifacts/cdc_recovery_acceptance_20261116/bss-matrix-02-cap8-fixture-01/cap8.transport.json)
- [fixture](/home/csc/esp/workspace/SensorArray/docs/audit/artifacts/cdc_recovery_acceptance_20261116/bss-matrix-02-cap8-fixture-01/cap8.fixture.json)
- [proxy](/home/csc/esp/workspace/SensorArray/docs/audit/artifacts/cdc_recovery_acceptance_20261116/bss-matrix-02-cap8-fixture-01/cap8.cap-proxy.json)
- [metrics](/home/csc/esp/workspace/SensorArray/docs/audit/artifacts/cdc_recovery_acceptance_20261116/bss-matrix-02-cap8-fixture-01/cap8.metrics.json)
- [report](/home/csc/esp/workspace/SensorArray/docs/audit/artifacts/cdc_recovery_acceptance_20261116/bss-matrix-02-cap8-fixture-01/cap8.report.md)

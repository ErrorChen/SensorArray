# Recorded fixture window: bss-short-01

Derived only from finalized formal USB raw bytes. No device commands, baseline substitution, calibration writes, or precision PASS.

Source raw SHA256: `b79551673d354b89c93ec1280091c14ec850598975a50dc7ee4be572e1b5f7d0`.
Host capture window: 5.247274737–125.247441621 s (relative receipt clock).
Transport replay status: `RECORDED`; fixture report: `REVIEW_REQUIRED`.

All intervals below describe observed samples. P05–P95 is not a confidence interval. CAP values are LC total capacitance; a same-cell unloaded baseline is absent.

## C / 8 rows / rev 58

Session `05DD194E2065ADB0`, boot `2`; sequences 38302–39710; 1409 frames.
Device first-to-last timestamp span: 119.941683 s; strict fixture screening: `SCREENING_INCOMPLETE`.

| Point | Unit | Valid n | Invalid n | Mean | Min–max | P05–P95 | Accuracy status |
|---|---|---:|---:|---:|---|---|---|
| S4D4 | pF | 1409 | 0 | 92.421321 | 92.401246–92.438882 | 92.411281–92.431356 | BASELINE_REQUIRED |
| S5D5 | pF | 1409 | 0 | 77.381926 | 77.364402–77.395420 | 77.372815–77.389611 | BASELINE_REQUIRED |

Floating RES cells meeting OPEN checks: 0/0 observed.
Floating CAP per-cell time medians (60 cells): 57.070793–79.986162 pF. Per-cell distributions remain in metrics JSON.

## Cross-cell CAP background proxy

All three fixed donor populations are reported. Missing/invalid/stale/unacquired donors exclude that frame/method; no shrinking, zero fill, smoothing, or selection by closeness to 33 pF.

| Point | Method | Paired n | Background mean (pF) | Difference mean (pF) | Difference min–max (pF) | Difference P05–P95 (pF) |
|---|---|---:|---:|---:|---|---|
| S4D4 | same_row_median | 1409 | 59.912853 | 32.508467 | 32.488296–32.526610 | 32.497440–32.519084 |
| S4D4 | same_column_median | 1409 | 78.987217 | 13.434103 | 13.415653–13.451843 | 13.425012–13.443245 |
| S4D4 | all_floating_median | 1409 | 59.913811 | 32.507510 | 32.487482–32.525856 | 32.496704–32.517515 |
| S5D5 | same_row_median | 1409 | 59.913136 | 17.468790 | 17.448940–17.484386 | 17.459920–17.476929 |
| S5D5 | same_column_median | 1409 | 65.691437 | 11.690489 | 11.673255–11.705411 | 11.682607–11.698151 |
| S5D5 | all_floating_median | 1409 | 59.913811 | 17.468115 | 17.448940–17.484836 | 17.459450–17.476560 |

## Reproduction and provenance

The selection manifest records source file hashes, tool hashes, invocation, every retained source/output byte range and its SHA256, excluded frames and truncated candidates. Only whole original ranges are concatenated; wire text and CRC are never rebuilt.

A frame's first header byte and final K byte must both arrive inside the explicit capture_start/capture_end window. This is at least as strict as the benchmark parser's complete-header/final-K rule; their counts are compared in the manifest.

The transport JSON audits diagnostics and malformed/incomplete material from the full original raw stream, so science-only slicing cannot erase a recorded fault. CAP proxy output is an approximate diagnostic, not a strict same-cell baseline.

- [raw](/home/csc/esp/workspace/SensorArray/docs/audit/artifacts/cdc_recovery_acceptance_20261116/bss-short-01/bss-short-01.fixture-window.rx.bin)
- [selection](/home/csc/esp/workspace/SensorArray/docs/audit/artifacts/cdc_recovery_acceptance_20261116/bss-short-01/bss-short-01.fixture-window.selection.json)
- [transport](/home/csc/esp/workspace/SensorArray/docs/audit/artifacts/cdc_recovery_acceptance_20261116/bss-short-01/bss-short-01.fixture-window.transport.json)
- [fixture](/home/csc/esp/workspace/SensorArray/docs/audit/artifacts/cdc_recovery_acceptance_20261116/bss-short-01/bss-short-01.fixture-window.fixture.json)
- [proxy](/home/csc/esp/workspace/SensorArray/docs/audit/artifacts/cdc_recovery_acceptance_20261116/bss-short-01/bss-short-01.fixture-window.cap-proxy.json)
- [metrics](/home/csc/esp/workspace/SensorArray/docs/audit/artifacts/cdc_recovery_acceptance_20261116/bss-short-01/bss-short-01.fixture-window.metrics.json)
- [report](/home/csc/esp/workspace/SensorArray/docs/audit/artifacts/cdc_recovery_acceptance_20261116/bss-short-01/bss-short-01.fixture-window.report.md)

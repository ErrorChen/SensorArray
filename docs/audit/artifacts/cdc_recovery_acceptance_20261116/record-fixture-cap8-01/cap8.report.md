# Recorded fixture window: record-matrix-01-cap8

Derived only from finalized formal USB raw bytes. No device commands, baseline substitution, calibration writes, or precision PASS.

Source raw SHA256: `d64f37b61ce5e6e2c1e15f728a41b1599761beb50e78a8c5970fbd0c337e99ce`.
Host capture window: 5.300236456–127.300472064 s (relative receipt clock).
Transport replay status: `RECORDED`; fixture report: `REVIEW_REQUIRED`.

All intervals below describe observed samples. P05–P95 is not a confidence interval. CAP values are LC total capacitance; a same-cell unloaded baseline is absent.

## C / 8 rows / rev 36

Session `0119031D7DEF4971`, boot `1`; sequences 32624–34036; 1413 frames.
Device first-to-last timestamp span: 121.983293 s; strict fixture screening: `WINDOW_SPAN_MET`.

| Point | Unit | Valid n | Invalid n | Mean | Min–max | P05–P95 | Accuracy status |
|---|---|---:|---:|---:|---|---|---|
| S4D4 | pF | 1413 | 0 | 92.535958 | 92.514252–92.555735 | 92.526808–92.544389 | BASELINE_REQUIRED |
| S5D5 | pF | 1413 | 0 | 77.425353 | 77.405926–77.441171 | 77.416434–77.433245 | BASELINE_REQUIRED |

Floating RES cells meeting OPEN checks: 0/0 observed.
Floating CAP per-cell time medians (60 cells): 57.112987–80.092987 pF. Per-cell distributions remain in metrics JSON.

## Cross-cell CAP background proxy

All three fixed donor populations are reported. Missing/invalid/stale/unacquired donors exclude that frame/method; no shrinking, zero fill, smoothing, or selection by closeness to 33 pF.

| Point | Method | Paired n | Background mean (pF) | Difference mean (pF) | Difference min–max (pF) | Difference P05–P95 (pF) |
|---|---|---:|---:|---:|---|---|
| S4D4 | same_row_median | 1413 | 59.962184 | 32.573774 | 32.552437–32.591417 | 32.564994–32.582589 |
| S4D4 | same_column_median | 1413 | 79.091206 | 13.444752 | 13.421260–13.464543 | 13.435801–13.453688 |
| S4D4 | all_floating_median | 1413 | 59.962965 | 32.572993 | 32.552437–32.591417 | 32.564191–32.581701 |
| S5D5 | same_row_median | 1413 | 59.962520 | 17.462833 | 17.444869–17.475230 | 17.455846–17.470070 |
| S5D5 | same_column_median | 1413 | 65.709537 | 11.715816 | 11.699486–11.729894 | 11.708720–11.722618 |
| S5D5 | all_floating_median | 1413 | 59.962965 | 17.462388 | 17.446097–17.474954 | 17.454693–17.469823 |

## Reproduction and provenance

The selection manifest records source file hashes, tool hashes, invocation, every retained source/output byte range and its SHA256, excluded frames and truncated candidates. Only whole original ranges are concatenated; wire text and CRC are never rebuilt.

A frame's first header byte and final K byte must both arrive inside the explicit capture_start/capture_end window. This is at least as strict as the benchmark parser's complete-header/final-K rule; their counts are compared in the manifest.

The transport JSON audits diagnostics and malformed/incomplete material from the full original raw stream, so science-only slicing cannot erase a recorded fault. CAP proxy output is an approximate diagnostic, not a strict same-cell baseline.

- [raw](/home/csc/esp/workspace/SensorArray/docs/audit/artifacts/cdc_recovery_acceptance_20261116/record-fixture-cap8-01/cap8.rx.bin)
- [selection](/home/csc/esp/workspace/SensorArray/docs/audit/artifacts/cdc_recovery_acceptance_20261116/record-fixture-cap8-01/cap8.selection.json)
- [transport](/home/csc/esp/workspace/SensorArray/docs/audit/artifacts/cdc_recovery_acceptance_20261116/record-fixture-cap8-01/cap8.transport.json)
- [fixture](/home/csc/esp/workspace/SensorArray/docs/audit/artifacts/cdc_recovery_acceptance_20261116/record-fixture-cap8-01/cap8.fixture.json)
- [proxy](/home/csc/esp/workspace/SensorArray/docs/audit/artifacts/cdc_recovery_acceptance_20261116/record-fixture-cap8-01/cap8.cap-proxy.json)
- [metrics](/home/csc/esp/workspace/SensorArray/docs/audit/artifacts/cdc_recovery_acceptance_20261116/record-fixture-cap8-01/cap8.metrics.json)
- [report](/home/csc/esp/workspace/SensorArray/docs/audit/artifacts/cdc_recovery_acceptance_20261116/record-fixture-cap8-01/cap8.report.md)

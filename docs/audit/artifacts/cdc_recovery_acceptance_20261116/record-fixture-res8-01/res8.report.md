# Recorded fixture window: record-matrix-01-res8

Derived only from finalized formal USB raw bytes. No device commands, baseline substitution, calibration writes, or precision PASS.

Source raw SHA256: `fe3c5bc360284ecc5b0658afbd61068f15ea7790e99135d942644d3263e0ca4b`.
Host capture window: 5.453932947–127.454102160 s (relative receipt clock).
Transport replay status: `RECORDED`; fixture report: `REVIEW_REQUIRED`.

All intervals below describe observed samples. P05–P95 is not a confidence interval. CAP values are LC total capacitance; a same-cell unloaded baseline is absent.

## R / 8 rows / rev 116

Session `0119031D7DEF4971`, boot `1`; sequences 83401–84407; 1007 frames.
Device first-to-last timestamp span: 122.041209 s; strict fixture screening: `WINDOW_SPAN_MET`.

| Point | Unit | Valid n | Invalid n | Mean | Min–max | P05–P95 | Accuracy status |
|---|---|---:|---:|---:|---|---|---|
| S1D1 | ohm | 1007 | 0 | 10164.972395 | 10159.550000–10171.083000 | 10162.038000–10167.986200 | NEEDS_LIMITS |
| S8D8 | ohm | 1007 | 0 | 10166.389779 | 10160.526000–10171.652000 | 10163.794000–10168.978200 | NEEDS_LIMITS |

Floating RES cells meeting OPEN checks: 60/60 observed.

## Reproduction and provenance

The selection manifest records source file hashes, tool hashes, invocation, every retained source/output byte range and its SHA256, excluded frames and truncated candidates. Only whole original ranges are concatenated; wire text and CRC are never rebuilt.

A frame's first header byte and final K byte must both arrive inside the explicit capture_start/capture_end window. This is at least as strict as the benchmark parser's complete-header/final-K rule; their counts are compared in the manifest.

The transport JSON audits diagnostics and malformed/incomplete material from the full original raw stream, so science-only slicing cannot erase a recorded fault. CAP proxy output is an approximate diagnostic, not a strict same-cell baseline.

- [raw](/home/csc/esp/workspace/SensorArray/docs/audit/artifacts/cdc_recovery_acceptance_20261116/record-fixture-res8-01/res8.rx.bin)
- [selection](/home/csc/esp/workspace/SensorArray/docs/audit/artifacts/cdc_recovery_acceptance_20261116/record-fixture-res8-01/res8.selection.json)
- [transport](/home/csc/esp/workspace/SensorArray/docs/audit/artifacts/cdc_recovery_acceptance_20261116/record-fixture-res8-01/res8.transport.json)
- [fixture](/home/csc/esp/workspace/SensorArray/docs/audit/artifacts/cdc_recovery_acceptance_20261116/record-fixture-res8-01/res8.fixture.json)
- [proxy](/home/csc/esp/workspace/SensorArray/docs/audit/artifacts/cdc_recovery_acceptance_20261116/record-fixture-res8-01/res8.cap-proxy.json)
- [metrics](/home/csc/esp/workspace/SensorArray/docs/audit/artifacts/cdc_recovery_acceptance_20261116/record-fixture-res8-01/res8.metrics.json)
- [report](/home/csc/esp/workspace/SensorArray/docs/audit/artifacts/cdc_recovery_acceptance_20261116/record-fixture-res8-01/res8.report.md)

# Fixture review: CAP8

Formal USB window: **1413 frames**, sequences **32624–34036**, device span **121.983293 s**, strict requirement **>=120 s met**. Formal sequence gaps, CRC errors and identity errors are zero. The recorded gate reports an identical complete USB/BLE frame set; invalid point semantics are preserved.

The complete raw file is broader than this measurement window. Its parser reports:

- Missing complete-frame sequence observations **32518–32551** (34), between original byte ranges ending and starting at **4530 / 4530**. Both neighboring complete frames were received during `prepare`, before the formal start at **5.300236456 s**. They are retained in the selection manifest as excluded boundary material. This proves their location, not their cause.

Do not describe the whole file as gap-free. See the full byte offsets, headers and receipt events in [boundary check](cap8.boundary-check.json).

CAP values are total LC capacitance. Both capacitor references remain `BASELINE_REQUIRED`: the three fixed cross-cell background proxies are descriptive and do not replace the same cell's unloaded baseline. The report preserves all row, column and all-floating proxy results; no method is selected because it is closer to 33 pF.

[Measurement report](cap8.report.md) · [Metrics](cap8.metrics.json) · [Closure and byte verification](cap8.validation.json)

No hardware actions, calibration writes or absolute precision acceptance were performed.

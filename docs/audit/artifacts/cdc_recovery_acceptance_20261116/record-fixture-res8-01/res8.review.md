# Fixture review: RES8

Formal USB window: **1007 frames**, sequences **83401–84407**, device span **122.041209 s**, strict requirement **>=120 s met**. Formal sequence gaps, CRC errors and identity errors are zero. The recorded gate reports an identical complete USB/BLE frame set; invalid point semantics are preserved.

The complete raw file is broader than this measurement window. Its parser reports:

- Missing complete-frame sequence observations **83315–83344** (30), between original byte ranges ending and starting at **4565 / 4565**. Both neighboring complete frames were received during `prepare`, before the formal start at **5.453932947 s**. They are retained in the selection manifest as excluded boundary material. This proves their location, not their cause.

Do not describe the whole file as gap-free. See the full byte offsets, headers and receipt events in [boundary check](res8.boundary-check.json).

| Point | Valid / invalid n | Mean (ohm) | Observed min–max (ohm) | Mean minus nominal 10000 ohm | Nominal deviation |
|---|---:|---:|---|---:|---:|
| S1D1 | 1007 / 0 | 10164.972395 | 10159.550000–10171.083000 | +164.972395 ohm | +1.649724% |
| S8D8 | 1007 / 0 | 10166.389779 | 10160.526000–10171.652000 | +166.389779 ohm | +1.663898% |

All **60/60** designated floating cells meet fresh acquired OPEN checks. The full 64-point stream retains **62434** invalid-numeric **X0D OPEN** observations (62434 OPEN; 62 per frame), including the two capacitor positions. The two resistor positions have no invalid samples. Complete receipt does not mean every point has a numeric value.

These are observed deviations from a nominal value, not errors against a traceable measured resistor standard. Measured reference values, resistor/reference tolerances and acceptance limits were not supplied; both results remain `NEEDS_LIMITS`. Min/max and P05–P95 describe sample spread, not confidence intervals or accuracy bounds.

[Measurement report](res8.report.md) · [Metrics](res8.metrics.json) · [Closure and byte verification](res8.validation.json)

No hardware actions, calibration writes or absolute precision acceptance were performed.

# Recorded passive reference-fixture frames

These are byte-identical slices of the device's production encoder output. They are **recorded**, not synthetic and not regenerated CRCs. `manifest.json` gives source hashes, offsets and slice hashes. The actual parser was executed and verified all three frame CRCs. These standalone slices do not prove transport completeness or a 120-second screening window.

- `cap8_total`: LC total capacitance, not a calibrated 33 pF component result.
- `res8_first`: S1D1 near 10 kΩ; S8D8 OPEN is an unresolved fixture/path finding, not acceptance PASS.
- `res8_crc_rejected`: an ADC CRC rejection is represented as S1D1 X02 with no valid numeric value; the ASCII transport CRC itself is valid. Several conversions preceded the rejection, so fresh/acquired masks alone do not make X02 usable data.

The source captures and analysis are preserved under `docs/audit/artifacts/reference_fixture/` (tracked gzip archives decompress exactly to the ignored raw `.rx.bin` originals). Session/boot/generations belong to each captured run; never combine their numeric sequence order across boots.

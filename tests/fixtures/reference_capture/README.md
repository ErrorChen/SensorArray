# Recorded passive reference-fixture frames

These are byte-identical slices of the device's production encoder output. They are **recorded**, not synthetic and not regenerated CRCs. `manifest.json` gives source hashes, offsets and slice hashes. The actual parser was executed and verified all three frame CRCs. These standalone slices do not prove transport completeness or a 120-second screening window.

- `cap8_total`: LC total capacitance, not a calibrated 33 pF component result.
- `res8_first`: S1D1 near 10 kΩ; S8D8 OPEN is an unresolved fixture/path finding, not acceptance PASS.
- `res8_crc_rejected`: an ADC CRC rejection is represented as S1D1 X02 with no valid numeric value; the ASCII transport CRC itself is valid. Several conversions preceded the rejection, so fresh/acquired masks alone do not make X02 usable data.

The source captures and analysis are preserved under `docs/audit/artifacts/reference_fixture/` (tracked gzip archives decompress exactly to the ignored raw `.rx.bin` originals). Session/boot/generations belong to each captured run; never combine their numeric sequence order across boots.

2026-11-13 adds `crc_partial_volt1_20261113.partial.recorded.txt` and the
immediately following `.next.recorded.txt`, extracted byte-for-byte from the
`0d8f60f-dirty` VOLT1 run. The first preserves X02 at S1D2 and acquired/fresh/valid
mask FD; the next is FF. Their unchanged wire CRCs pass the real parser.
`crc_partial_volt1_20261113.manifest.json` records original SHA, byte offsets,
configuration identity and the same-sequence ADSDISPOSITION association.
This is recorded partial-data recovery evidence, not a synthesized CRC fixture
or proof that the ADC electrical link has stopped producing errors.

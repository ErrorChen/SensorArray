# Rail monitor profile and bounded RREG guard follow-up

This follow-up is against `6a8f5029feadd455a66f5dedb226d6e554073079` plus the source hashes in `host_test_manifest.json`. The parent task owns ESP-IDF builds, serial access and hardware captures. These host fixtures are synthetic and do not replace the preserved `crc-fixed-res8-01.rx.bin` or `timing-res8-01.rx.bin` recordings.

## Confirmed rail configuration defect

The local [ADS1262/ADS1263 manual](../../../../../datasheets/ads1263.pdf), section 9.3.5, requires the supply monitor's PGA to be enabled at gain 1 with chop disabled. The previous `sensorarrayAdsReadRailMonitorUv` changed reference, calibration and input MUX but inherited MODE0/1/2 from the last matrix cell. In the hardware recording that state was frequently MODE2=8F (PGA bypass). The initial RES route used PGA enabled/gain 1, so initial and periodic monitor samples did not use the same hardware profile.

The captured transition from railSpanUv=5167748 to 5036352 coincided with the measured S1D1 resistance plateau changing from about 9979 to 11086 ohms. The parent capture analysis found almost unchanged inferred differential voltage across those intervals. This supports a conversion-scale effect from the rail snapshot. The prerequisite violation is confirmed in source and the datasheet; neither an unchanged physical supply voltage nor the entire observed step's analog cause is proven without an independent measurement.

`sensorarrayAdsReadRailMonitorUv` now configures continuous conversion, chop off, SINC1, PGA enabled/gain 1 and the configured data rate (DR=15 in this build), while retaining neutral ADC calibration and internal-reference monitor MUX CC. New `sensorarrayAdsVerifyRailMonitorProfile` reads a stable physical register snapshot and compares the documented configuration bits, including POWER/RESET, INTERFACE, MODE0/1/2, INPMUX, all calibration bytes and REFMUX. A mismatch prevents START. The comparison excludes documented reserved bits, not real configuration differences.

Each fresh monitor sample must pass the existing SPI CRC and freshness checks and have no RESET, reference or PGA alarm. The helper does not publish the candidate raw/voltage until full state restoration succeeds. It preserves the original sampling/configuration error separately from cleanup error, retains restore-cache synchronization and latches isolation on uncertain cleanup. The redundant unchecked SPI read after an already verified restore was removed. Successful restore evidence comes from `ads126xAdcRestoreAdc1RegisterSnapshot`, which already reads back the complete saved profile.

`RAILPROFILE` and `RAILSAMPLE` are bounded diagnostic records emitted only after the transaction ends and the ADS owner is released. They expose the inherited and configured MODE0/1/2, profile validity, raw sample/status, restore result and first comparison. `monitorUv` is the internal divided span, not an independent AVDD or AVSS measurement. Existing reference-ready, input settling and discarded conversion behavior remains; no new reference power cycle, smoothing, gain threshold or sampling-rate limit was introduced. The board's unavailable analog-isolation capability still rejects CAP/VOLT monitoring that would enable REFOUT.

## RREG guard and unsuccessful duty experiment

The 25% high duty experiment did not resolve the error. The parent `timing-res8-01` capture ended after 109.251 seconds with REFMUX expected 00 / actual 01 / mask 3F at ADS engineFrameCount=1280, following 40 CRC errors. `ads126xAdcSpiDutyCycleForClock` now returns the normal duty128 for all targets and frequencies. The old capture and candidate source history remain evidence, not a claimed repair.

Datasheet section 9.5.6 defines the RREG byte count as the number of real consecutive registers returned; section 9.6 identifies addresses 00h through 10h as common to both ADS1262 and ADS1263, with TDACP at 10h. `ads126xAdcReadRegisters` now requests one additional legal register only when the requested range ends at or before REFMUX (0Fh). The extra byte is discarded. The caller receives exactly its requested bytes, and those bytes no longer occupy the last bit of that RREG response. No requested bit is repaired, masked or guessed. Reads ending beyond 0Fh keep their previous length; ADC2 or unsupported addresses are not added.

The host fault model forces the final returned RREG bit to 1. Under that model, POWER and REFMUX remain correct with the guard; corruptions in requested bytes still fail verified configuration or stable-snapshot comparison. This is a test of the mechanism, not proof of the board's electrical cause. The production transaction still uses the existing persistent DMA buffers and descriptor, shared deadline and timeout quarantine. The local copy is filled only after the exact completed descriptor returns. At 2 MHz the added byte adds 4 microseconds of clocking per protected register read; full system performance requires the parent HIL capture.

RDATA1 remains its exact command plus STATUS/data/CRC payload. Its CRC, valid-even/odd counters, fresh-generation checks and single per-cell retry budget are unchanged. The RREG guard cannot fix the ADC CRC byte's final bit; an electrical or controller timing problem remains unresolved until measured. No unsupported ESP32-S3 sampling point, above-spec clock, CS hold assumption with CS=-1, or disabled CRC was introduced.

## Executed tests

All commands, exit codes, source hashes and logs are in `host_test_manifest.json`.

| Executed production path | Result |
| --- | --- |
| Driver reliability, both ESP32-S3 and other-target compile variants | PASS |
| RREG final-bit injection: POWER, REFMUX, full snapshot, true requested-byte corruption, two/three snapshot reads, bounds, DMA timeout/quarantine | PASS |
| Rail chain, 27 synthetic cases: inherited PGA1/bypass/gain32+chop, nine profile mismatch registers, SPI/config errors, five alarm bits, original/cleanup errors, cache restoration, safety admission | PASS |
| Existing battery admission and snapshot evidence, nine cases | PASS |
| Actual fault formatter golden fixtures, eleven fixtures | PASS |
| Actual matrix diagnostic formatter and rate/fatal behavior | PASS |
| Measurement math and logic suite | PASS |
| `git diff --check` | PASS |

The rail harness executes the actual save, profile verification, monitor and restore functions with a synthetic register bank. The independent driver harness executes the actual SPI/CRC/DRDY driver with a persistent-DMA mock. These provide complementary software coverage; no host fixture claims real rail stability, oscilloscope timing, Windows USB behavior or end-to-end transport delivery.

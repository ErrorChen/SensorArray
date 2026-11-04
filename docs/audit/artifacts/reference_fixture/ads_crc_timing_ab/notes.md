# SPI timing A/B candidate: CRC final bit

Baseline: `a0460b8`, raw capture `../crc-fixed-res8-01.rx.bin` (preserved). The five emitted rejected ADC1 wire frames independently recompute as:

| Sequence | Wire bytes after RDATA1 | Computed CRC | Received CRC | XOR |
|---|---|---|---|---|
| 207 | 60BBC758E599 | 98 | 99 | 01 |
| 239 | 60BBC8E15327 | 26 | 27 | 01 |
| 277 | 60BBC50AEF5F | 5E | 5F | 01 |
| 335 | 60BBC491C8B7 | B6 | B7 | 01 |
| 399 | 60BBC457D779 | 78 | 79 | 01 |

All five are S1D1 and all were recovered by the bounded fresh-conversion retry. Cumulative error counters exceed the number of text records because recovered diagnostics are rate limited. The later BATTERY fault is a separate admission/restore investigation; these CRC records are not evidence of its exact cause.

The remaining OPEN points cannot disprove a final-bit fault: raw `80000000` correctly produces CRC `31`; `7FFFFFFF` produces `EF`. Both already have bit zero set. “Only S1D1 fails” therefore does not establish PGA causality.

## What the source and manuals establish

- Actual configuration: ESP32-S3, SPI3, SCLK GPIO47/MOSI21/MISO14 through the GPIO matrix, mode 1, 2 MHz, 56 clocks per status+ADC1+CRC RDATA1 transfer. `CONFIG_BOARD_ADS126X_CS_GPIO=-1`; there is no controlled CS output. Changing post-transaction CS hold cannot fix that wiring. IDF 5.5.5 also converts a zero hold request to one clock when CS is present.
- Local `datasheets/ads1263.pdf`, tables 7.6/7.7: SCLK high/low minimum 40 ns, DOUT valid within 60 ns after SCLK rising, last falling edge to CS rising minimum 40 ns. Section 9.4.7.2 permits holding-register reads by command without synchronizing the start to DRDY. No local ADS1262 specification found here proves the exact post-final-bit DOUT transition responsible for these captures.
- Local IDF 5.5.5 S3 HAL has empty `spi_ll_set_miso_delay` and RX timing setter implementations; standard sample phase is unsupported. An `input_delay_ns` or PHASE_1 assignment would not be a supported fix to this current full-duplex path.
- Espressif [issue 17908](https://github.com/espressif/esp-idf/issues/17908) describes a matching CPHA1 final-bit-to-1 signature when an ADC changes its output to DRDY before the ESP samples it. It concerns ADS1235/AD7190, so it supports this hypothesis rather than establishing ADS1262 causality. Its above-spec clock workarounds are not used here.

## Candidate and executed host checks

`ads126xAdcSpiDutyCycleForClock` (new interface) returns duty position 64 only for ESP32-S3 at a request of exactly 2 MHz. Every other target or requested clock retains 128. The actual initialization `sensorarrayInitSpi` in `core/board/sensorarrayBringup.c` and optional `ads126xAdcHelperCreateSpiDevice` both use it. Each records requested clock/mode/duty/CS/candidate/result as `ADS_SPI_TIMING`. No SPI clock, CRC, retry budget, ADC sampling rate or measurement threshold changed.

`idf_spi_divider_check.c` executes the current IDF divider calculation without touching hardware. Its successful log shows 2 MHz, 125 ns high and 375 ns low for 40/80/160 MHz source clocks, within the listed ADC limits. The production driver test compiles for S3 and non-S3, checks the exact 2 MHz boundary and higher-clock fallback, and reruns CRC/DRDY/DMA tests. Both target variants pass.

ADSHEALTH adds boot-domain cumulative `crcValidEven` and `crcValidOdd`: these increase **only after CRC equality succeeds**, according to the computed CRC's least significant bit. `crcErrors` counts rejected checks separately and `crcRetries` counts fresh-read retry attempts. The executed test verifies that a rejected frame, including repeated rejection, does not increment either valid counter. Even-valid counts provide direct coverage of the zero final bit that saturated OPEN data lack. Counters are diagnostic and do not replace fresh/configuration/status validation.

The change remains a **timing candidate** until a separate, preserved HIL capture compares rejected-CRC increments and successful even-CRC coverage against the baseline. No oscilloscope measurement or physical cause is claimed by these host checks. Root owns the build, flash, serial capture and final A/B conclusion; this agent did not open the device.


## Recorded A/B outcome (root, 2026-11-05 Sydney)

`timing-res8-01` ended after 109.251 seconds. The 25% high candidate still reported ADC CRC final-bit mismatches and then a register health failure at engineFrameCount=1280: REFMUX expected00/actual01/mask3F. Valid-even checks were1165, valid-odd385840, CRC errors40 and retries27 at that boundary; these counts include earlier boot/auxiliary reads and are not all per-matrix conversions. This candidate did **not** fix the observed problem. Production returns to duty128; the historical candidate and capture remain as evidence. No physical timing mechanism is claimed proven.

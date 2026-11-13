# ADS SPI startup qualification and presence probing

Implementation/test base: `7e1d2c6`, with the current shared working-tree changes.
Exact command exit codes, source hashes and dirty diff hash are in
`20261113-ads-spi-worker-tests.json`; the corresponding unabridged synthetic host
output is in `20261113-ads-spi-worker-tests.log`. This worker did not access
serial, flash firmware, or perform HIL.

## New implementation

- `ads126xAdcRequestSpiClockHz` records a bounded, mutex-protected frequency
  request. The existing persistent SPI descriptor applies `override_freq_hz`
  to every register/command/data transfer. There is no device removal or DMA
  cancellation. Quarantine/pending descriptors prohibit another request.
- `ads126xAdcGetSpiClockHz` queries IDF's calculated frequency after a completed
  transaction. `ads126xAdcPredictDefaultSpiClockHz` uses the same default clock
  source and IDF divider calculator; comparison uses truncated kHz, as does
  IDF's getter. This is calculated clock evidence, not an instrument reading.
  IDF 5.5.5 can reject an override with only a warning, so a successful SPI
  queue/completion alone cannot qualify a new rate.
- `sensorarrayInitSpi` uses a 125-kHz bootstrap (capped by the configured upper
  limit) when auto-tuning is enabled. Bootstrap success is not qualification.
- `sensorarrayAdsSpiProbeRun` is a separate synchronous module for the persistent
  boot job managed by the main task. It owns ADS before matrix/route/Gap startup;
  the existing FDC initialization can run concurrently. It uses the caller's
  absolute deadline, reserving 500 ms for cleanup, and yields every 32 attempts.
- The 29 candidates are 2 MHz through 1 MHz in 100-kHz steps, 950 kHz through
  150 kHz in 50-kHz steps, then 125 kHz. Candidates above the configured ceiling
  are not attempted. There is no untested minimum-speed fallback.
- Each candidate is screened with 200 fresh attempts in gain-one and 200 in
  bypass. Its independent confirmation uses 800 fresh attempts per profile.
  Each profile permits at most 25 per mille rejected CRC frames (5/200 and
  20/800). A CRC-rejected read consumes one attempt but contributes no valid
  sample, parity, raw extrema, measurement, or cache update. Runtime CRC checks
  and runtime retry budgets are unchanged. Hard status/register/stale/reset/
  timeout errors fail the current candidate; exceeding its maximum possible
  CRC allowance also fails early.
- Each profile separately requires at least 32 CRC-valid even and 32 CRC-valid
  odd final bits, non-saturated data and varying raw codes. Both profiles and
  the confirmation must pass. Invalid configured coverage requirements are
  rejected before hardware mutation.
- The test profile is ADC1, INTREF off, VBIAS on, AVDD/AVSS reference, MUX AA
  (both inputs select AINCOM), gain-one or bypass, continuous/chop off/SINC1 at
  38400 SPS. No rail value is needed because only raw codes and wire validity
  are checked. No calibration/rail/value cache is populated. This is not a
  precision or actual matrix-input qualification.
- Register verification uses masks and the existing bounded snapshot reader
  at profile entry, during sampling and after sampling. Cleanup restores the
  complete saved ADC1 register state and driver shadow, vref bookkeeping,
  stopped state and DRDY freshness. The selected clock remains only when it
  passed qualification; cleanup verifies its actual divider. `qualified` is
  false whenever restoration is unverified; original and cleanup errors are
  separate.
- The driver retains its existing runtime majority-of-three snapshot recovery,
  but counts every first/second read disagreement in `snapshotMismatchCount`.
  Startup rejects any such delta in the initial saved snapshot, candidate
  verification or final restore. CRC's allowed error rate does not permit
  unstable register reads.
- `sensorarrayBringupProbeAdsPresence` performs read-only, low-speed ID/POWER/
  INTERFACE presence checks. A live handle is reused even when `adsReady` was
  cleared. If cold initialization destroyed that handle, a separate permanent
  DMA slot is used. Two stable reads, a legal nonzero/non-FF device ID, any prior
  trusted ID match, legal interface bits and calculated 125-kHz clock are
  required. This does not establish CRC workload stability or analogue safety.
- Presence checks restore the entry clock after both present and absent
  responses, using a completed read-only transaction and actual-clock check.
  `actualHz` describes the low-speed check and `restoredHz` the restored clock.
  Restoration failure makes presence unknown; `cleanupError` preserves its
  separate cause. Pending DMA is never reused, freed or claimed canceled.
- Repeated presence polling cannot adopt a failed restore's low clock as a new
  successful baseline. Runtime handles retain their requested clock's predicted
  divider; cold slots retain their first entry-clock restore target until
  restoration succeeds. Atomic `sensorarrayBringupAdsPresenceBusy` and
  `sensorarrayBringupAdsPresenceQuarantined` expose the actual cold-slot owner
  and DMA lifetime to the main recovery coordinator.

## Manual and IDF evidence

Local `datasheets/ads1263.pdf`: serial timing table 7.6 permits CS held low,
requires SCLK period at least 125 ns and high/low pulses at least 40 ns.
INPMUX table 9-41 has no dedicated continuously sampled internal-short opcode:
AA selects AINCOM twice; FF is float and BB is the temperature monitor.
REFMUX table 9-46 permits positive AVDD / negative AVSS (`0x24`) independently
of INTREF. VBIAS section 9.3.12 gives 22-ms settling at a 10-uF load; startup
uses a bounded 25-ms input-bias guard plus fresh-generation/status checks.
This guard does not claim instrument-verified analogue settling or accuracy.

Local ESP-IDF 5.5.5 `spi_master.h` exposes `override_freq_hz` and
`spi_device_get_actual_freq`; `spi_master.c` applies the override during
transaction setup and only warns if timing calculation rejects it. The getter
returns `real_freq / 1000`. The public divider calculator is deprecated but
still implemented; its warning waiver is local to the prediction call.
The project uses default APB clock source and 50% duty on ESP32-S3.

## Executed host evidence

- Driver regression: PASS for S3 and non-S3 host variants, including pending
  versus applied frequency, ignored override, quantization, deadline and DMA
  quarantine checks.
- Complete production startup probe plus production driver: 20 cases PASS,
  including normal selection, each bad rate, confirmation failure, odd-only
  CRC coverage, saturation, status/reset, physical register mismatch,
  restore mismatch, ignored override, DMA completion unknown, DRDY timeout,
  bypass-only failure, total deadline, exact/excess 2.5% boundaries and
  majority-recovered snapshot instability in all three phases. A separately
  compiled impossible configured parity/error allowance case also PASS.
- Production presence implementation plus production driver: 21 cases PASS,
  covering live and destroyed-handle paths, all-zero/all-FF/stable/unstable
  reads, persistent DMA, pending completion, actual-clock rejection, busy
  ownership, expired deadline, normal and absent clock restoration, frequency
  quantization, cleanup timeout and repeated rejected-restoration recovery.
- Existing rail profile regression: 33 cases PASS. `git diff --check`: PASS.

## Boundaries retained

The worst hypothetical complete traversal of all 29 candidates uses about
6.344 seconds of RDATA wire time plus 17.98 seconds of explicit one-tick yields
at the current 100-Hz RTOS tick, before register operations and other scheduling.
The 30-second default total budget is finite; exhaustion is an error, not
permission to skip rates or accept an incomplete candidate.

Finite boot samples cannot prove zero runtime CRC errors. Actual RES/VOLT
inputs, common mode, analogue loading and power disturbances remain HIL
requirements. Presence reads do not enable INTREF, reset ADC, START conversion,
or alter matrix GPIO. The board still lacks independent isolation of REFOUT
from the selected S row: ADS power-on defaults can enable INTREF before firmware
acts. Q1 clamp readback does not prove safe power restoration. This hardware
limitation must remain explicit in automatic-recovery policy.

Lock/lifetime order: boot/presence owner -> driver transaction deadline ->
driver SPI mutex -> persistent IDF descriptor completion -> release mutex.
Logging occurs after the SPI mutex is released. Presence's atomic busy flag
only serializes its callers; the main coordinator must still establish frontend
quiescence. The startup task is retained after completion to avoid a dangling
ISR-cached task pointer, as implemented by the main-task owner.

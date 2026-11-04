# ADS reliability implementation notes

Source baseline: `8045e9e9ec9599533c52c15dfcb6002f79fd15f1`. Worktree initially clean. These notes describe uncommitted changes; the root report supplies final dirty hash and build result. No board flash, instrument, Windows enumeration or HIL was performed by this worker.

## Confirmed evidence and symbols

- `sensorarrayAdsMatrixEngineReadFrame`: before this patch every non-OK `sensorarrayAdsMatrixRefreshRegisterShadow` result was replaced by `ESP_ERR_INVALID_RESPONSE`. This erased SPI/time-out/snapshot errors before main emitted MATRIX_ROUTE.
- `sensorarrayAdsMatrixEngineSetMode`: does **not** reset cumulative `frameCount`. Actual checked-in sdkconfig interval is **64**, not 128. E2/E3 alone therefore do not establish which health call failed. New `modeEntryFrameCount` records the cumulative count at mode entry.
- `sensorarrayAdsMatrixNoteRegisterSnapshot`: previously compared complete POWER/INTERFACE/REFMUX bytes through the register cache. The new production `sensorarrayAdsRegisterHealthCompare` checks RESET independently, then POWER mask 0x03, REFMUX mask 0x3f, MODE0/1 and cache, preserving the first mismatch. INTERFACE mask is 0x0f. Other documented config bits remain checked. Cache mismatch is evidence of disagreement, not proof of which side is stale; firmware isolates instead of silently repairing it.
- `ads126xAdcReadAdc1RegisterSnapshot`: RREG has no CRC. Two bounded equal masked reads, or a third resolving a disagreement, remain required. SPI failure and three inconsistent snapshots now have separate diagnostics. POWER.RESET stays in duplicate-read comparisons and is never ignored as a reserved bit.
- Actual IDF 5.5.5 `esp_err.h`: 0x108 INVALID_RESPONSE, 0x107 TIMEOUT, 0x109 INVALID_CRC. Host stub was corrected to these values.

## Implemented changes

1. Before cleanup, matrix fault context preserves mode, route, owner, original and cleanup errors, actual engine/mode-entry counts, health interval/due, precision flag, expected/actual/mask, cache/physical/config generations, attempted/acquired state, and previous rail/battery/ADSCHK transaction evidence. New API: `sensorarrayAdsMatrixEngineCopyFaultContext`.
2. `sensorarrayAdsFaultFormat/Emit` produce bounded complete ADSFAULT / ADSFAULTRAIL / ADSFAULTCTX / ADSFAULTREG / ADSFAULTTXN records. Every part includes boot/sequence where required; each is below the 384-byte acquisition-event limit. Rate limiting uses the captured first-failure stage. railSpanUv, avddUv, avssUv and source are separate. Unknown boot is literal `unknown`; main binds real breadcrumb identity for auxiliary events using new `sensorarrayAdsGapBindBootIdentity`.
3. SPI sync and DMA paths share one queued transfer using handle-owned descriptor and buffers. Mutex/queue/get-result consume the same 100-ms per-operation deadline. Completion timeout quarantines the handle. No reuse, reset, init or free is permitted while driver ownership remains unresolved. Deinit may reclaim only after `spi_device_get_trans_result` returns the exact descriptor. No claim of local DMA cancellation is made.
4. New owner deadline APIs cover full transactions: matrix cell 100 ms including retries; health snapshot 100 ms; rail/battery/zero 250 ms including save/restore; ADSCHK 1.5 s including all samples/cleanup. Each nested SPI/DRDY operation uses the remaining outer budget. An expired cleanup budget leads to explicit restoration failure/isolation. Scheduler tick rounding can overrun a blocking wait by less than one tick; no new budget is opened.
5. CRC8 is selected during ADS boot calibration (root also changed bringup/route). Existing conversion framing is length-checked exactly. Datasheet section 9.4.7.3.4 defines polynomial 0x107 (8-bit shift implementation 0x07), initial zero and the four ADC1 data bytes. STATUS is **not** CRC-protected; status/reset/reference checks remain separate. Bad CRC never publishes raw code to PGA/cache.
6. Actual POWER INTREF OFF→ON starts `referenceReadyAtUs`; `StartAdc1` and snapshot restore await it. C42 is 1 uF between REFOUT and AVSS on circuit sheet 3. Datasheet 11.3 specifies a minimum 50 ms for this capacitor, while section 9.3.8.1 warns larger capacitance increases startup. Repeated ON or per-cell mux changes do not restart that wait. This is reference timing, not physical node verification; real hardware/precision still needs instrument validation.
7. DRDY freshness requires a generation newer than both the caller start and last physical configuration marker. A held-low DRDY is insufficient. Configuration changes while waiting invalidate the sample. Clearing notifications does not claim hardware freshness.
8. Internal rail monitoring cannot turn INTREF on where board capability lacks isolation. It can diagnose an existing intentionally enabled RES reference, or a genuinely isolated future board capability. CAP/VOLT use explicit external calibration. Root owns route/main capability changes. Rail helper now owns the shared ADS owner, restores driver and register shadow, tracks physical generation, and matrix invalidates profile/value caches after auxiliary generation changes.
9. Battery rejects absent/stale monitor rails before changing hardware, while explicit external calibration does not age out. Battery records saved/configured/restored physical snapshots (POWER, MODE0/1/2, INPMUX, REFMUX, offset/full-scale calibration, VBIAS, INTREF, DRDY). Three bounded raw/status/timestamp samples and complete transaction duration are emitted. Cached saved state is compared against physical state before it can be used for restore. Original error survives restore failure, with cleanupError separate. Battery invalidity with verified restoration remains unavailable data, not a matrix fault. Repeated same reason backs off to at most 30 s (or the operator's longer period), and BATNOW bypasses backoff for a deliberate probe. Threshold unchanged.
10. All auxiliary hardware work runs at complete frame boundaries. Legacy `sensorarrayAdsGapTryRun` is a no-op compatibility entry. New `sensorarrayAdsGapRunPendingAtBoundary` handles zero/rail requests; battery and ADSCHK have dedicated boundary calls. Zero now has owner/save/restore and restore-failure latch. Boundary diagnostics use ADC1 on both chips so the full saved ADC1 snapshot covers every changed register. ADC2 configuration remains untouched; existing ADS1263 running state is preserved when applicable.
11. External rail calibration rejects analog span outside 4.75–5.25 V or AVSS below -2.6 V relative to DGND (datasheet 7.3), preserves exact AVDD/AVSS, and has no frame-age expiry. Internal span-derived AVDD/AVSS is marked `estimated`; AINCOM midpoint remains estimated even with independently supplied rail values. ARL exposes uncertainty as unknown. No value is clamped into validity. ABAT retains numeric UINT32_MAX for old parsers and adds `lastGoodKnown`; unknown is not a measured age.

## Lock and lifetime order

```text
Core 1 acquisition/frame boundary (single hardware writer)
  -> register-cache ADS owner (MATRIX / BATTERY / RAIL / ZERO / ADSCHK)
  -> owner transaction deadline
  -> driver SPI mutex (finite remaining deadline)
  -> IDF queued persistent transaction
  -> exact descriptor completion OR quarantine
  -> release SPI mutex
  -> restore/verify + shadow synchronisation
  -> release deadline and ADS owner
```

Logging happens after the SPI mutex is released. Route/controller operations are sequenced by the same Core 1 writer; root added route owner arbitration. No worker stack contains an outstanding SPI descriptor/buffer. The owner field is not a multi-writer lock: Core 0 command handlers must continue to queue work instead of calling hardware functions. Driver quarantine reads in main are from the same Core 1 writer domain.

## Actually run tests

- `CC=/usr/bin/gcc python3 tools/test_measurement_logic.py`: exit 0, `MEASUREMENT_LOGIC_TESTS,passed=1`; log `ads_measurement_logic.log`. Added direct production-logic health checks for every count through 512 at intervals 64/128, entry after cumulative 65, 128/256/384/512, zero/wrap, RESET/POWER/REFMUX/MODE0/MODE1/MODE2/cache/calibration faults, reserved-bit masks and earliest condition. Bounded battery backoff/explicit re-probe, original/cleanup split, pre-SAFE mode, all maximal uint32/uint64 fields/long stage strings below per-record 384 bytes, multiple first-failure stage rate limits, and boot unknown are exercised.
- `python3 tools/test_ads_driver_reliability.py`: exit 0, `ADS_DRIVER_RELIABILITY_TESTS,passed=1,source=production_driver,fixture=synthetic`; log `ads_driver_reliability.log`. Compiles and executes the actual driver source under fake IDF/SPI/time functions. Four groups test deadlines/lifetimes, CRC/length, reference/DRDY freshness, and duplicate-read snapshots. CRC tests use independent long division and 160 one-bit corruption cases. Mutex+queue+completion share 100 ms; two calls share a 5-ms owner budget; pending descriptor survives timeout, rejects reuse/reset/init/deinit, and can be freed only after simulated driver return. GPIO low and config-change cases are explicitly rejected.
- `python3 tools/test_ads_fault_fixtures.py --update` followed by `python3 tools/test_ads_fault_fixtures.py`: exit 0, 10 synthetic replay fixtures generated by the actual C formatter; exact outputs and README are under `tests/fixtures/ads_fault`. These diagnostic records have no measurement CRC and are not field captures.
- A failing intermediate test detected missing StartAdc1 reference gate. The gate was corrected and both suites were rerun to PASS. Initial host invocation without CC failed because this environment has gcc but no clang; specifying `/usr/bin/gcc` resolved it.

## Residual / NOT_RUN

- E1 18–19 mV common step and E2/E3 physical failure have not been reproduced on attached hardware. The code removes unsafe transparent CAP/VOLT monitor and stale helper-state paths; this is not a measured proof of E1's cause.
- Two-minute four-combination rail/battery tests, 1/8-row CAP/VOLT/RES/mixed, fault-frame-count HIL, actual chip ID, analog reference settling, battery noise/source/disconnection and CPU/heap/stack/FPS comparisons are NOT_RUN. The new code does not increase spread threshold or smooth samples.
- R38–R45 loading, REF/Q1/U5 connectivity, actual installed capacitor/revision and independent AVDD/AVSS calibration uncertainty remain physical constraints. A 50-ms timer does not certify an arbitrary external analog node has settled.
- Hardware DMA cancellation is unavailable; a quarantined frontend requires the root bounded recovery/restart policy. Host mocks establish software lifetime invariants, not electrical bus recovery.
- FaultEmit uses one 1537-byte local buffer; Format writes directly into caller storage to avoid a second nested copy. Task stack high-water must be checked on hardware; the root build is not stack/HIL validation. Additional diagnostic records use the bounded event ring and may be dropped under pressure; no exactly-once delivery claim.

# ADS tail diagnostic implementation evidence

HEAD before root commit: `8b1ce9f8a6612498e8338f822ad63ec71e8444c6`. All commands and source hashes are recorded beside the host logs. No target build, serial or physical operation was performed by this agent.

New interfaces this round: `sensorarrayAdsTimingProbeKind_t`, `sensorarrayAdsGapRunTimingProbeKind`, `sensorarrayAdsTimingProbeKindName`, `sensorarrayAdsTimingProbeFormatTailRecord`, `ads126xAdcTailDiagnosticAvailable`, `ads126xAdcReadAdc1FreshTailDiagnostic`. Existing TIMING wrapper remains.

TAIL groups are STD gain1, GPIO-tail gain1, STD bypass, GPIO-tail bypass, all CONTINUOUS. Each attempts200 primary reads; no held repeat. POWER/REF/clock remain saved values, INTREF stays off, MUX=AA. Timeout-off is verified before diagnostic mutation. All ordinary ADC reads and boot qualification retain their original behavior.

The private diagnostic transfer holds the ordinary ADS mutex from admission through55-bit queue/get-exact-result, physical one-clock GPIO sample, and restored output selector. The permanent DMA buffers are at least8bytes. No SDK acquire_bus, remove-device or cancellation is used. The board capability is created only when this boot newly initializes the bus and adds its single ADS device; already-initialized bus, helper-enabled config, non-tied-lowCS and active/quarantined presence owner reject it.

Failure after an unknown DMA completion never takes SCLK. Unknown GPIO route/framing quarantines ADS and blocks normal register restore. If the55-bit descriptor is known complete after the workdeadline, the final clock is completed before returningTIMEOUT; no valid sample is returned. Generic wrapper then uses its reserved cleanup budget.

GPIO latch is preloaded low while SPI still owns SCLK; only output routing changes. High wait1us precedes GPIO MISO read; low wait1us precedes level/route verification. No IRQ disabling, RTOS delay, allocation, log or network operation is added inside the transfer. ISR/preemption may lengthen the pause; elapsed timestamps are software observations. GPIO level and routing verification do not prove absence of glitches.

RAW source=spi55_gpio1 always has wireKnown=0,bytes=unknown. The separate TAIL record contains7byte partial prefix (only55bits known, bit0 forcedzero as unknown), actual gpioBit, assembledCrc and software timing. The parser checks the unchanged4data-byte CRC. CRC computation is diagnostic evidence and never determines the GPIO bit. STATUS remains separately checked and is notCRC-protected.

Normal group TAIL detail explicitly says tailApplicable=0,source=spi56,spiBits=56,gpioKnown=0,assembledKnown=0. Counters alone never claim receipt at the PC.

Actual tests: driver board/prefix256patterns +3 badbit/data cases +11 failure cases +known completion deadline +board owner scenarios; complete module25TIMING+8TAIL; formatter worst423/436bytes below512; ordinary driver S3/non-S3; rail33; matrix CRC disposition19. All exits0 with source_changed_during_run=false. These are synthetic wire/RTOS tests, notHIL.

The IDF v5.5.5 test_master1–9bit tx/rx includes DMA AUTO and MSB high-part expectations, supporting remainder7. The S3 TRM generic non-byte statement differs; preserve this distinction and obtain board prefix/CRC evidence before claiming55bit layout validated here.

Files and current frozen SHA256:

- `components/ads126xAdc/ads126xAdc.c`: `682408c952fb1f26996854098314bdfbede7d7bc860a7b1a1fc688e89e66822f`
- `components/ads126xAdc/include/ads126xAdc.h`: `f4a1a198cd1af72baf4641e114912141128ee5db46ce8d11f1a9659601c7c3b4`
- `core/board/sensorarrayBringup.c`: `b93a6881ba6e37492c493172d011500a94ca97399e316c2c3b7a93fe62fa621e`
- `core/measure/ads/sensorarrayAdsTimingProbe.c`: `49be9e4ba13bafacd12ad6ad7ef1f4be0d07674cce8bf0a035cca484b89ecdb4`
- `core/measure/ads/sensorarrayAdsTimingProbe.h`: `362fefd333162fbed0ea608e4e854b51fd2efc533140038351d0017869a281ee`
- `core/measure/ads/sensorarrayAdsGap.c`: `1522b6a97cd81494b47e8cd2fe155ef48e1b3b1ee528c285f8e2d947082fa7b5`
- `core/measure/ads/sensorarrayAdsGap.h`: `709fff489653b66943ab52498335c848a3ceea1f941b004210c6dc63540a036b`

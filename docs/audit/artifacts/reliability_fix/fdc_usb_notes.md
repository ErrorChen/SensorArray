# FDC worker / USB implementation evidence

This file records the firmware patch against `8045e9e9ec9599533c52c15dfcb6002f79fd15f1`. The input worktree was clean. Test inputs below are **synthetic fault injection**, not recordings from the user's device. No device was flashed and no Windows enumeration experiment was performed.

## FW01: bounded FDC worker ownership

Confirmed in the baseline: `sensorarrayMeasureFdcWorkerTask()` consumed the same notification counter twice with `ulTaskNotifyTake(pdTRUE, portMAX_DELAY)` to express JOB then RUN. `sensorarrayMeasureReadFdcMatrixRowParallelEpoch()` passed stack-local job/result/trace and caller frame-local sample/config/timing/tracker pointers into those tasks, then used unbounded completion waits on error paths. Coalesced wakeups could strand the worker at RUN and the coordinator at DONE.

Implemented in `core/measure/fdc/sensorarrayFdcWorkerLifecycle.h` (new this round), `sensorarrayFdcInternal.h`, `sensorarrayFdcRowEpoch.c`, and guarded callers in `sensorarrayMeasure.c`:

- The two workers and two physical buses remain parallel. The existing INTB/necessary STATUS/DRDY/read4 path remains the hardware acquisition path.
- Per-worker persistent storage owns job, result, trace, samples, runtime channel configuration, timing and a separate read tracker. The sole external job pointer is `state`, pointing to the application's permanent context. No caller stack object is retained.
- Release/acquire atomic phase transitions implement IDLE → POSTED → ACKED → RUNNING → DONE; cancellation uses CANCEL_REQUESTED → CANCEL_ACKED → RECLAIMED. Notifications and event bits only wake tasks. Lost/coalesced/stale notifications cannot grant result ownership or satisfy a new epoch.
- An absolute monotonic deadline includes posting, ACK, route/profile preparation and RUN. A separate, fixed reclaim deadline does not extend on notifications. Tick conversion rounds up. A task idle with no accepted job may wait indefinitely.
- Timeouts do **not** terminate an I²C call. A worker not released by the reclaim deadline stays QUARANTINED with every object retained. A late release allows physical stop/restart handling but does not permit reuse of the quarantined slot. The coordinator never copies an in-flight result and does not fall back to serial I²C while a worker is quarantined.
- `sensorarrayMeasureFdcWorkersQuiescent()` and `sensorarrayMeasureFdcWorkersQuarantined()` are new interfaces. Caller route/recovery guards prevent hardware reconfiguration while ownership is outstanding. Worker cleanup does not delete tasks that may own I²C locks.
- Row output is merged only after both workers release ownership. Read trackers no longer have two workers updating one shared tracker.
- The row-gap ADS helper call was removed; auxiliary ADS transactions belong at frame boundaries.
- The independent boot/rescue `sensorarrayMeasurePrepareFdcMatrixPath()` checks worker quiescence and physically reads ADS POWER/INTREF=0 before asserting the Q1 clamp. This is evidence about a register and GPIO command, not an analog voltage measurement.

The synchronous lower-level I²C APIs still have their existing finite mutex/bus timeouts. A call already in progress cannot be forcibly cancelled at the logical deadline. In particular, coordinator profile writes can return after that deadline; their elapsed time is included and no RUN is then granted. Exact HIL upper bounds require measurement of those driver calls. This patch does not claim a DMA/I²C cancellation API exists.

Actual tests, all exit 0:

```text
python3 tools/test_fdc_worker_lifecycle.py
FDC_WORKER_LIFECYCLE_PASS,scenarios=job_unconsumed;ack_delay;run_delay;done_lost;cancel_race;blocked_bus;late_done

python3 tools/test_fdc_worker_runtime.py
FDC_WORKER_RUNTIME_PASS,actualFunctions=worker+coordinator,iterations=56,asan=1,ubsan=1,scenarios=normal50;job_unconsumed;ack_delay;run_delay;done_lost;cancel_before_run;blocked_bus;mode_guard;stale_done
```

The runtime test compiles the **complete actual production worker and coordinator functions extracted verbatim** into a pthread FreeRTOS shim. Hardware calls are stubbed, with delayed JOB consumption, sleep ACK, RUN wakeup and one blocked bus. It asserts a finite return, preservation of caller stack sentinels after return, persistent pointer targets, refusal of a new epoch while quarantined, delayed release and 50 normal parallel epochs. ASan stack-use-after-return and UBSan are enabled. LeakSanitizer is disabled because this execution environment runs under ptrace; this is not a leak test. The shim revealed and fixed the case where an idle worker first observes QUARANTINED before it ever consumes POSTED: it must still publish release of its unused slot. These 50 epochs are not a substitute for 50 HIL mode transitions.

## USB / transport

Baseline configuration is ESP32-S3 native USB Serial/JTAG (`CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG=y`, TinyUSB CDC console disabled, power management disabled). Baseline had no explicit USB driver installation: `stdin/stdout` used the console VFS; Serial control used `fgets(stdin)`. USB DATA used a separate `fwrite`/`fflush` task, but guaranteed lifecycle and Serial CTRL replies also called `fwrite`/`fflush` from transport/control tasks, so those paths could wait on USB. A successful stdio write did not prove driver acceptance, much less PC receipt.

New `core/transport/sensorarraySerialIo.[ch]` (this round):

- Install native Serial/JTAG driver once; do not initialize TinyUSB or reconfigure GPIO19/20. One `serialCtrl` task reads that driver; one `serialTx` task owns physical TX. `stdin` is no longer a second consumer.
- Register a nonblocking VFS for stdout/stderr. Normal printf/log writes enqueue to the same physical owner. CTRL and guaranteed lifecycle use a separate priority queue. `USBSTREAM` and DATA eligibility do not disable command RX or priority TX.
- No acquisition/BLE/Wi-Fi/control task waits for USB transfer completion. Queue admission is nonblocking. USB writes are zero-wait chunks (64 bytes) within one 20 ms packet deadline. Partial acceptance and timeout are counted; an incomplete packet gets a delimiter before the next packet, so a truncated message cannot silently become a complete scientific frame.
- The next queue drain clears stdio error flags left by VFS EAGAIN, permitting later output attempts. VFS rejects full queues rather than returning a fabricated accepted length.
- Priority is checked before each DATA packet. A packet already being written can take up to its bounded deadline; no exactly-once or remote-delivery guarantee is claimed.
- The old `main/output/sensorarrayUsbSink.c` queue, two scratch packets and second USB task were removed. Its API still implements DATA profile and submission statistics. SerialIo has five 2304-byte normal buffers allocated once (PSRAM preferred, internal fallback), four 512-byte internal priority buffers, nine small queue indices/lengths and one 4096-byte task. Producer payload copying is outside the pool spinlock and does not allocate a large stack packet. The old standalone USB task reports configured stack 0; SerialIo reports its actual configured/minimum remaining bytes.
- USB installation failure is reported after optional network transports are initialized, so it does not prevent BLE/Wi-Fi initialization.

Statistics are local observations:

| Field | Meaning |
| --- | --- |
| `sinkQueued` | Number of locally accepted stdout/DATA/priority packets |
| `priorityQueued` | Subset accepted in the priority queue |
| `sinkDropped` | Queue rejection or packet remainder dropped at TX deadline |
| `driverAcceptedBytes` | Bytes accepted by `usb_serial_jtag_write_bytes`, including resynchronizing delimiters |
| `driverDrained` | Observed transitions with `usb_serial_jtag_wait_tx_done(0)==ESP_OK`; not PC application acknowledgement |
| `partialWrite` | Packets with a nonzero accepted prefix and a dropped remainder |
| `timeout` | Packets not completely accepted before the packet deadline |
| `rxBytes`, `lastRxUs` | Bytes read by the sole driver reader, and monotonic timestamp |
| `lastTxUs` | Monotonic timestamp of the last nonzero driver acceptance |
| `driverInstalled` | Driver installation observation |
| `sofPresent` | Driver observed host SOF packets; not COM enumeration, port-open state or PC byte receipt |
| `taskStackConfiguredBytes`, `taskStackMinBytes` | Actual serialTx task stack budget and minimum remaining bytes |

Legacy USB sink `sentPackets/sentBytes` are **sink submission counts**. The new query/report labels must say queued, not completed. BLE `bleDataSent` remains BLE backend enqueue acceptance; Wi-Fi send is UDP socket acceptance. Drops must be compared as deltas in the same boot; neither these counters nor Serial FPS prove PC delivery. COM8 FileNotFoundError only establishes that COM8 was absent for that open attempt. Hardware/power/cable/bootloader-vs-application enumeration remain separate NOT_RUN investigations.

Actual test, exit 0:

```text
python3 tools/test_serial_io.py
SERIAL_IO_PASS,initOnce=1,priority=1,noHostBoundUs=20000,rxIndependent=1,partialRecovery=1,vfsRejectAndResume=1
```

The test compiles the actual SerialIo module and supplies synthetic USB/queue/VFS callbacks. It injects zero and partial driver acceptance, queue flooding, reappearance after a stall, and checks priority delivery order and independent RX. It does not emulate the electrical USB controller or Windows.

## Transport frame metadata and capacity

`sensorarrayTransportFillFrameMeta()` now delegates to the new pure `sensorarrayTransportParseFrameMeta()` implementation. It parses comma-delimited fields from the header only; `gen=` cannot match inside `mgen=` and a trailer cannot overwrite header metadata. Payload-owned metadata includes session, bootId, revision, mode/rows/profile generations and request IDs. `TX=SHORT/RT` B20 previews also carry that same immutable identity and `previewOnly=1`; missing preview values are `na`, not fabricated zero. The preview remains non-scientific and has no standalone ASCII measurement CRC; full frames retain their encoder CRC. It never reads a global current configuration when sending an old queued frame.

Legacy fields are retained: C `gen/rid` are ROWS; V/R `gen/rid` are MODE; M `rgen/rrid` are ROWS and `pgen/prid` are profile. Data capacity was raised to 2304 bytes in the transport pool and BLE message pool to accommodate the explicit domain extension and real worst-case encoder output. CTRL remains 512 bytes. Guaranteed terminal storage is 384 bytes. This is an explicit protocol capability change, not a silent reinterpretation.

Actual tests, exit 0:

```text
python3 tools/test_transport_frame_metadata.py
TRANSPORT_FRAME_METADATA_PASS,domains=rows;mode;profile;session,exact_fields=1,immutable=1
CC=/usr/bin/cc python3 tools/test_transport_pool.py
TRANSPORT_POOL_TESTS,passed=1,slots=5,descriptorBytes=8,slotBytes=2376
CC=/usr/bin/cc python3 tools/test_ctrl_reply_guard.py
CTRL_REPLY_GUARD,passed=1
CC=/usr/bin/cc python3 tools/test_rowmodes_ack_guard.py
ROWMODES_ACK_GUARD,passed=1
```

The initial unqualified existing transport test failed to discover a compiler because it searches clang/gcc but this environment exposes `/usr/bin/cc`. Re-running with the explicit `CC` above executed the test successfully. No failure was hidden. Build and whole-project testing are recorded by the main report, separate from these focused tests. Hardware throughput, minimum heap/contiguous block, real stack high-water, dual-bus electrical faults, Windows USB unplug/re-enumeration, BLE high-load and real mode switching are **NOT_RUN** here.

## Restart budget and recovery request regression

`tools/test_restart_budget.py` extracts seven actual production functions from `main/main.c`, the complete READY block and the healthy-frame call expression. Its RTC/monotonic clocks, event sink and authority copies are explicit host shims. The source name is `sensorarrayBootPrepareRestart`, corresponding to the prompt's `sensorarrayBootBreadcrumbPrepareRestart`. No main source is changed by this test.

Executed with exit 0:

```text
python3 tools/test_restart_budget.py
RECOVERY_IDENTITY_PASS,productionFunctions=post+take,singlePending=1,copyOutsideMux=1,revisionOnlyAllowed=1,staleSessionAndDomains=7,knownTransitions=1,unknownBootRecovery=1,idWrap=1
RESTART_BUDGET_PASS,productionFunctions=7,readyBlock=actual,frameCaller=actual,maxAuto=3,slowBootWindowReset=0,stableHealthyUs=120000000,minHealthyFrames=128,maxGapUs=30000000,stallAndSparseDoNotClear=1,failBreaksStreak=1,clockRollback=1,powerOnReset=1
```

Assertions cover three permitted automatic restarts across 60-second startups, refusal of a fourth after an additional hour, READY preserving the budget, 119 seconds of healthy frames being insufficient, the 120-second boundary clearing the chain only after at least 128 healthy frames, a single frame after a 120-second stall not clearing it, 256 sparse frames with 31-second gaps not clearing it, 127 frames over 126 seconds not clearing it, and actual caller failures/non-fresh/no-valid-data breaking the interval, READY breaking an existing interval, monotonic rollback, and power-on reset. Recovery request tests assert one pending request without overwrite, authority copied outside the request spinlock, revision/state-only changes allowed, session and each of six domain generation/request fields changing rejected with `stale_config`, known/unknown transitions rejected, unknown-to-unknown boot recovery allowed, and wrap skipping reserved request ID zero.

The production stable-progress function now also records `lastHealthyUs` and the number of healthy frames. Restart budget is reset only after 120 seconds, at least 128 healthy frames, and no gap greater than 30 seconds between healthy observations. A failure, invalid frame, READY event, monotonic rollback or longer gap starts qualification again. This rule only determines eligibility to replenish the restart budget: it does not declare a slow mode faulty, arm a watchdog in PAUSED/SAFE, or limit capture. A mode slower than this qualification requirement may retain its consumed budget indefinitely until real progress or explicit power-on. Hardware progress/watchdog behavior still requires HIL.

Final memory correction: the actual baseline sdkconfig USB queue length is 2. Its removed storage was 3088 bytes plus two 1544-byte scratch packets. SerialIo normal storage is now a one-time 11520-byte PSRAM-preferred allocation; `normalBufferMemory` 0/1/2 means unavailable/internal/PSRAM. The main USBSTREAM query uses `bufferMemory/bufferBytes`. Priority, mutex and queue state remain internal. Allocation-failure/retry/internal fallback tests passed; no hot-path allocation. See final-size.txt and async_log_storage_notes.md for the subsequent logger pool migration.

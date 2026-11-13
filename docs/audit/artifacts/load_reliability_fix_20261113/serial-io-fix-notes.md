USB record admission and retained priority replies
=================================================

The reproduced baseline is commit `1f82b73a1019930f419492e3e742a96bd43f374e`.
Its owner wrote 64-byte chunks with zero driver wait, slept one tick after a
failed attempt, and discarded the remaining packet at 20 ms. At the configured
100 Hz tick rate, two unsuccessful retries consume that entire lifetime. This
applied equally to scientific frames and priority CONFIG/control replies.

`python3 tools/test_serial_io.py --tx-baseline 1f82b73a1019930f419492e3e742a96bd43f374e`
compiled that actual old source and reproduced a 64-byte CONFIG prefix followed
by expiry, although the simulated host made space at 30 ms. The modified source
admitted the same complete CONFIG record at 30 ms. The complete command,
exit code 0, output, HEAD, dirty state and before/after source hashes are in
`serial-io-host-01.json` and `serial-io-host-01.txt`.

Changed production functions
----------------------------

| File / function | Change |
| --- | --- |
| `core/transport/sensorarraySerialIo.c`, `sensorarraySerialPublishSlot` | Capture enqueue time and absolute expiry before publishing the slot. Normal records expire after 250 ms, priority after 1,000 ms. Failed priority admission is counted separately. |
| Same file, `sensorarraySerialTxTask` | Retain one candidate from each lane; prefer priority while a normal record has not begun. Submit the complete remaining record to the driver. Preserve positive short-write cursor and slot ownership until completion or expiry. Recheck original deadlines after waits. |
| Same file, `sensorarraySerialTxCheckpoint` **(new this round)** | Invoke the shared CPU0 work-budget checkpoint outside all locks. It returns after at most one tick; it never waits indefinitely for idle progress. |
| Same file, `sensorarraySerialIoInit` | Use a 2,304-byte driver TX byte ring so the largest complete record fits. Preserve existing allocations and retry behavior. Refuse to adopt an unknown preinstalled driver, whose capacity the public API cannot report; do not uninstall it. Initialize the shared work budget without taking command service away when its hook falls back. |
| Same file, `sensorarraySerialIoGetStats` | Expose driver buffer size and the new completion/expiry/error/yield counters. |
| `core/transport/sensorarraySerialIo.h` | Add ten `uint32_t` diagnostic fields, retaining existing field meanings. |

The TX task no longer acquires stdout/stderr FILE locks after draining queues.
The only transient VFS EAGAIN occurs before binding becomes ready, and the
existing initialization retry still clears those FILE errors. Once ready,
diagnostic VFS writes consume or count whole-line drops. Protocol callers keep
their actual queue-admission result.

The driver ring increases from 1,024 to 2,304 bytes: **1,280 additional bytes**
through IDF's existing ring allocation. This build uses `SPIRAM_USE_CAPS_ALLOC`,
not general PSRAM malloc; budget that increase in internal heap. Enqueue and
deadline arrays add 144 static bytes, counters add 40 bytes, and driver ownership
adds one bool plus compiler alignment. The existing 11,520-byte CPU-only payload
allocation remains preferentially in PSRAM. No payload allocation occurs in a
producer or TX hot path. The owner task stack remains 4,096 bytes.

Driver evidence and deadline semantics
-------------------------------------

The installed ESP-IDF v5.5.5 source was inspected and compiled into a host test:

- `esp_driver_usb_serial_jtag/src/usb_serial_jtag.c`,
  `usb_serial_jtag_write_bytes`: copies an entire requested item into its ring
  and returns its size, or returns zero. The mutex and ring can each wait for
  the supplied tick count.
- `esp_ringbuf/ringbuf.c`: BYTEBUF has `xMaxItemSize == xSize` with no item-header
  overhead. Its actual fit/copy/free-space functions admit 2,304 bytes into an
  empty ring, including wrapped empty pointers, and reject an item without
  copying any prefix when free capacity is insufficient.

The owner requests at most one tick for each driver call, and requests zero
when fewer than two ticks remain in its original enqueue deadline. A driver
positive short return is handled defensively although this IDF driver is
all-or-zero. If a partial record eventually expires, the next record remains
behind a newline barrier. Invalid driver returns are not counted as successful
records. Slot memory is persistent, and this synchronous copying driver retains
no caller buffer pointer after returning.

The shared work budget replaces the first draft's separate USB busy-time
heuristic. It observes actual CPU0 idle progress and bounds a work burst to
32 units or an idle gap of two ticks before requesting one tick of blocking.
The 32-unit guard represents up to 73,728 accepted bytes at maximum packet size;
it does not set any acquisition or transport FPS limit. Actual driver blocking
is not treated as CPU work merely because wall time elapsed.

These are cooperative RTOS deadlines. A higher-priority task that prevents the
owner from being scheduled can delay cleanup; no software deadline promises
otherwise. On resumption, expired records are reclaimed before another write.

New diagnostic meanings
-----------------------

| Field | Meaning |
| --- | --- |
| `normalSubmitted`, `prioritySubmitted` | Complete records copied into the driver; normal includes DATA and diagnostic lines. Not PC receipt. |
| `normalExpired`, `priorityExpired` | Admitted application records discarded at their enqueue deadline. |
| `priorityRejected` | Priority record could not enter the local bounded slot/queue. |
| `driverShortWrite` | Positive driver return smaller than the requested remaining record. |
| `driverWriteError` | Negative or larger-than-requested driver return; completion unknown. |
| `txYield` | Shared checkpoint requested a real one-tick RTOS block. Driver waits are separate. |
| `maxQueueAgeUs` | Largest enqueue-to-accepted-byte age observed, including send waiting. |
| `driverTxBufferBytes` | Capacity configured by this owner, zero if ownership is unknown. |

Existing `partialWrite` counts abandoned application records with a known
accepted prefix. `driverAcceptedBytes` includes accepted delimiter bytes.
`driverDrained` is the driver's idle observation, not a host application receipt.
The parallel command task adds `SERIAL?`; it does not enlarge the existing
near-capacity `USBSTREAM?` reply.

Executed host coverage and remaining validation
-----------------------------------------------

The production SerialIo and shared WorkBudget modules run together in the host
shim at **100 Hz**. Coverage includes slow host admission, no host, independent
RX during TX wait, priority/data arbitration, queue residence expiry, a deadline
with less than two ticks remaining, positive short-write resume, partial cancel
and newline recovery across an idle/wake boundary, invalid driver returns,
priority queue exhaustion, 40 continuously replenished maximum-size records,
missing idle progress, and an immediately-zero-returning broken driver. All
eight initialization/allocation paths pass, as do the prior two-stream,
concurrent stdio, oversized-line, VFS registration/retry and real-encoder-frame
regressions. No hot allocation or leaked slot is permitted by those checks.

`tests/host/test_usb_driver_record.c` **(new this round)** executes the actual IDF
write and BYTEBUF functions; RTOS wait/ISR scheduling is simulated.
`tests/host/test_serial_io.c` and `tools/test_serial_io.py` execute the current
application owner and the explicit old revision reproduction.

No serial, BLE, flash or target operation was performed by this subtask.
The root task must still build and run simultaneous USB/BLE HIL. Driver
admission cannot guarantee that a disconnected or non-reading PC received a
record. Bounded expiry and admission rejection remain observable outcomes;
authoritative configuration/terminal queries reconstruct state. Emergency ROM
watchdog/panic output can still interrupt the wire outside the application
queue; such a run remains a fault, not a successful scientific capture.

Independent review follow-up: expiry must drain all queued records
----------------------------------------------------------------

The initial host coverage tested one retained item per lane. Independent review
identified a P1 gap when four priority records were queued before the first
notification take: after the retained record expired, the local `fetched` flag
was false, so the owner could sleep even though three queue slots remained.
The initial `ulTaskNotifyTake(pdTRUE, ...)` had already consumed every wake
count; no future producer was guaranteed to wake it.

The actual pre-fix production source reproduced this with four normal plus four
priority records and no subsequent notifications: at 1,000,000 us, normal expiry
count was four but priority expiry count was one, with three queued/owned slots
remaining. `serial-io-expiry-failure-01.json` records the failing command, exit 1,
source hashes and output. The exact pre-fix source is preserved in
`serial-io-expiry-before.c.txt`, verified against that pre-edit hash.

The minimal fix treats both fetching and expiry/release as `drainProgress`.
After releasing the last retained candidate, the task inspects the queues again
before it can enter a blocking notification wait. It does not require another
producer, reset a deadline, or fabricate a notification.

`serial-io-host-02.json` supersedes host-01 for final validation. It records an
explicit saved-source failure reproduction and the corrected production pass.
New cases fill each actual queue capacity separately and together, require all
slots to be released, check exact expiry/drop counts, and prove that an expired
partial priority packet advances to a younger item in the same queue with a
newline boundary and no future producer notification. The complete existing
driver, VFS, work-budget and allocation regressions also pass.

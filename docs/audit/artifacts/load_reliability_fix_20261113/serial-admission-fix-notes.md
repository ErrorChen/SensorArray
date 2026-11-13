# USB DATA / diagnostic admission repair

Baseline firmware: `b44c37b51067e376e73aa44535ea47d175846b67`.
The completed dual-link and USB-only observations belong to that flashed image;
the following source changes were not running during either observation.

`serial-admission-diagnosis-01.json` records source-summary hashes, actual query
windows, raw offsets, and representative gaps. The dual-link window received
8083 complete frame trailers in 120.000110 s, with 397 missing sequence numbers.
One frame started 67 bytes before capture_start, so the strict header-and-trailer
count wholly inside that byte window is 8082. The raw capture was not altered.

Across the respective diagnostic queries, sinkDropped increased 659 and
lineDropped increased 149, while normalExpired, partialWrite, driverShortWrite,
and driverWriteError did not increase. UsbSink usbDrop increased 511, but this
counter includes scientific packets and both diagnostic summary packets. Its
window differs from the SERIAL/USBSTREAM query windows; it must not be labelled
511 lost scientific frames. Of 380 sequence-gap events, 263 had a compact
summary between the surrounding complete frames; 243 had an ADS summary.
That is supporting correlation, not a reconstruction of the scheduler at every
rejected frame. Capture RX volume was about 48.05 kB/s.

The actual old code allows AsyncLog (priority 7) to fill one four-record FIFO
with DATA and diagnostic records before serialTx (priority 4) runs. Two stdio
line assemblers also borrowed the same five payload slots. The new host
baseline case executes that original source, fills four diagnostic records,
and observes the next scientific packet rejected with no TX expiry or short
write. The same image's USB-only control received 10913 frames in 120.000351 s
with usbDrop delta zero; its 13 sink drops were all diagnostic line drops.

Changed functions:

- `sensorarraySerialClaimSlot`, `sensorarraySerialPublishSlot`,
  `sensorarraySerialQueueRecord` (new), `sensorarraySerialIoQueueData` (new),
  `sensorarraySerialIoQueue`, `sensorarraySerialWrite` and
  `sensorarraySerialLineDrop`: separate DATA, priority and diagnostic admission.
- `sensorarraySerialTxTask`: retains a slot through every positive short write
  in any lane, prioritizes control at record boundaries, and serves a waiting
  LOG after at most four DATA completions when control is not pending.
  Every lane retains its original enqueue deadline and expiry reclamation.
- `sensorarraySerialIoInit` / `GetStats`: one startup payload allocation grows
  from 11520 to 16128 bytes, preferentially PSRAM. DATA remains five slots/four
  queue entries; priority remains four slots/four entries; LOG has two slots/two
  entries. Additional internal storage is one queue, two slot metadata entries,
  and four uint32 counters; no extra task or hot-path allocation is added.
- `sensorarrayUsbSinkPublishClass` and `PublishDiagnostic` (new): explicit
  classification while preserving existing UsbSink aggregate counters.
  Only the two compact/ADS summary calls in AsyncLog change to Diagnostic.
  The regular UsbSink frame path and legacy `sensorarrayFrameOutputPrint`
  use the new DATA API.

`normalSubmitted` remains DATA plus LOG. New `dataSubmitted` / `logSubmitted`
count complete records admitted by the driver, not PC receipt. New
`dataRejected` / `logRejected` count capacity refusal; VFS counts a rejected
logical line once, and oversize lines remain separate lineOversize events.
The task priority, 250 ms normal lifetime, 1000 ms priority lifetime, native
driver ring size, acquisition rate, and watchdog policy are unchanged.

Validation: `serial-admission-host-01.json` / `.txt` preserve the actual command,
exit code 0, and stable before/after source hashes. The command includes both
the exact b44c37b admission baseline and the older 1f82b73a TX truncation
baseline. It runs the installed ESP-IDF BYTEBUF/write code plus current
production SerialIo, legacy output and UsbSink functions under synthetic host
scheduling. Ten initialization variants include PSRAM/internal allocation,
allocation/queue/task/stdio retries and foreign-driver refusal. New assertions
cover diagnostic floods, two unfinished stdio streams, DATA/CTRL independent
capacity, forty continuously replenished maximum-sized DATA records with LOG
fairness, LOG short-write ownership against both other lanes, and all seven
nonempty lane combinations expiring without any future producer notification.
Existing protocol CRC flow and production work-budget/drain tests also passed.

Remaining limits: physical USB delivery and revised dual-link throughput require
HIL. Two LOG slots are deliberately bounded; a burst of more than two separate
diagnostic records before service can still lose whole records, including
best-effort fault detail, with rejection/line counters. Critical lifecycle and
control records retain independent priority capacity. Fair LOG service is
conditional on driver progress and no continuously pending priority flood;
positive prefixes are never interrupted to satisfy scheduling preference.

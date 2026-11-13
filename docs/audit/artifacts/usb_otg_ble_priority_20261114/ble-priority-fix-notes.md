# BLE data service and negotiation observation

Baseline: `be09762eadc6ffd908ec0375d1e41c354a9f7c23`; the working tree also contains concurrent USB-OTG and documentation changes. This note covers only the BLE component and its host regressions. No hardware operation, target build, or commit was performed by this worker.

## Confirmed evidence

The completed `../load_reliability_fix_20261113/repair-screen-05.summary.json` and `screen-analysis-06.json` cover the same 120-second USB/BLE window. Capture was approximately 69.29 FPS and BLE reception 41.53 FPS. Between the first and last in-window BL50/STK50 reports, `dropD` and the BLE slot-allocation-failure counter each increased by 3,279. Successfully received fragments had zero recorded CRC/reassembly failures. The first observed local loss was therefore admission to the four DATA payload slots. This does **not** establish that RF/controller service was unconstrained: absent congestion/error callbacks do not measure SDK queue waiting or negotiated link capacity.

The existing DATA and LOG payload pools were independent, but their descriptors shared a six-item FIFO. A LOG descriptor already in that FIFO delayed new DATA. The old TX task checked CTRL then waited up to 20 ms only on the shared DATA/LOG queue, so new CTRL could not wake that wait. Existing logs reported PHY only from the initial read; PHY-update, connection-parameter, and packet-length completion events were not retained.

The local IDF 5.5.5 Kconfig options `BT_CTRL_BLE_LLCP_CONN_UPDATE` and `BT_CTRL_BLE_LLCP_PHY_UPDATE` select disconnection behavior after Instant Passed (0x28); their unset values do not disable PHY/interval update requests.

## Production changes

| File / functions | Change |
| --- | --- |
| `components/sensorarrayBle/sensorarrayBle.c`: `sensorarrayBleReceiveNext` (new), `sensorarrayBleTxTask`, `sensorarrayBleQueueTxDescriptor`, `sensorarrayBleDrainTxQueues`, `sensorarrayBleInit` | Split descriptor service into existing-capacity DATA(4)/LOG(2)/CTRL(2) queues. CTRL first, then at most four DATA records before a waiting LOG. A static binary work semaphore wakes the owner from any queue; SAFE confirmation retains its separate task notification. Whole messages retain their slot through all fragments. |
| `sensorarrayBleAllocateTxDescriptor`, `sensorarrayBleNotifyInternal`, `sensorarrayBleNotifyLifecycleForGeneration` (new) | Lifecycle text uses the existing reserved ≤512-byte CTRL pool/priority queue while retaining the FF30 channel and its envelope/CRC. Ordinary LOG remains separate. Queue rejection is still returned; local acceptance is not peer delivery. |
| `sensorarrayBleRequestLinkParameters`, `sensorarrayBleResetLinkObservationLocked`, `sensorarrayBleObserveLinkEvent` (new), GAP/GATT callbacks | Record request errors and actual completion statuses. Request 2M preference with recorded 1M fallback, 7.5–15 ms interval/zero latency, 4-second supervision, and 251-byte data length. Successful requests do not set “known.” PHY and interval values update only from successful matching-address completion events. Disconnect resets connection observations. |
| `sensorarrayBleGetLinkSnapshot` (new), `sensorarrayBleSendGattValue`, `sensorarrayBleSendDescriptor` | Atomic read-only negotiation/TX snapshot; SDK call active/start/completion/count/error/max duration; queue maximum age by channel; DATA/LOG/priority dequeue counters; actual task priorities. Application retry elapsed time is checked as well as attempt count. |
| `components/sensorarrayBle/include/sensorarrayBle.h` | New lifecycle API and explicitly documented link/TX snapshot. |
| `components/sensorarrayBle/CMakeLists.txt` | Explicit `esp_timer` dependency for monotonic timestamps. |

Default component priorities are BLE TX 7 and control RX 8; root owns the matching Kconfig/sdkconfig values and other task-priority changes. Local SDK BTC/BTU/controller task priorities are higher. The existing WorkBudget checkpoints remain in place; no fixed FPS cap or watchdog change was introduced.

Payload capacity remains 14,848 bytes: DATA 4×2304, LOG 2×2304, CTRL 2×512. The eight descriptors grow from 24 to 32 bytes (+64 bytes total). New metadata also needs one queue control block, one static semaphore, the link snapshot, and pointers; target map/heap measurements remain for root's build/HIL. The host shim's `sizeof(state)` is not a target-memory measurement.

## Observation and deadline boundaries

`connIntervalUnits` is in 1.25 ms units; supervision timeout is in 10 ms units; all `*Us` fields use microseconds. Counter increments are unsigned 32-bit; maximum durations saturate at UINT32_MAX and should be reported as start/end maxima, not subtracted.

Completion status 65535 means none observed. `dataLengthObserved` means an IDF packet-length callback was received. IDF 5.5.5 omits peer address and connection ID from that event; `dataLengthConnectionKnown` remains false, including after reconnect. The raw TX/RX length and status are diagnostic observations, not proof of this connection's effective DLE. Preferred-PHY-command completion is similarly not proof of an actual PHY update. Address-bearing GAP events cannot distinguish a delayed event from an earlier connection to the same peer; current-generation attribution is limited by the SDK event shape.

The public IDF call chain `esp_ble_gatts_send_indicate → btc_transfer_context → btc_task_post → osi_thread_post` uses `OSI_THREAD_MAX_TIMEOUT`. `osi_thead_work_queue_put` maps it to `xQueueSend(..., portMAX_DELAY)` on the BTC queue (default capacity 100). Therefore a single SDK admission call is not bounded by the application's retry window. The implementation keeps the persistent slot owned and exposes active/start/completion diagnostics; it does not pretend that elapsed time cancels SDK work, delete the owner, or reuse its buffer. This remains a dependency risk requiring HIL observation and a separately reviewed SDK/recovery solution if triggered. The regression explicitly simulates a 300 ms SDK call, observes the overrun, and prevents a further application retry after its budget expires.

Lifecycle and CTRL share two reserved slots, so simultaneous priority traffic can still reject admission. The caller's terminal cache and authoritative query remain necessary. Ordinary diagnostic LOG can be rejected; its weighted service prevents starvation under continuous DATA, not lossless delivery.

## Executed validation

`ble-priority-validation-01.json` and `.txt` record exact commands, exit codes, output, elapsed time, source SHA256 values, and HEAD. All six recorded commands exited 0:

- `python3 tools/test_ble_priority.py`: 39 production function bodies, six scenario groups, ASan/UBSan. Cases cover full pools, DATA-before-LOG, weighted LOG service, lifecycle priority, idle CTRL wakeup, whole-message fragments, queued CTRL during fragments, disconnect/old generation, failed admission cleanup, corrupt payload rejection, bounded application retry, SDK-overrun observation, confirmation/work-signal separation, request failures, matching/stale completion events, reconnect unknowns, and static queue initialization failures.
- `python3 tools/test_work_budget.py`: 20 scheduler-policy scenarios, 15 real drain scenarios, and 11 real BLE fragmentation scenarios. Only BLE/Serial API shims changed; the production WorkBudget component did not.
- `python3 tools/test_receive_ble_text.py`: three receiver tests.
- `python3 tools/test_capture_benchmark_ble.py`: eight BLE collector tests.
- `CC=/usr/bin/clang-18 python3 tools/test_transport_pool.py`: production transport pool/policy regression.
- Scoped `git diff --check`.

LeakSanitizer cannot operate under this sandbox's ptrace and is disabled in the new static-storage harness; AddressSanitizer and UndefinedBehaviorSanitizer execute. An initial unconfigured compiler lookup and that LeakSanitizer environment failure were corrected before the recorded passing run. Target build, RF throughput, PC receipt, IRQ placement, and physical USB-OTG behavior are **NOT_RUN** by this worker. Passing host tests do not establish that BLE now matches capture FPS.

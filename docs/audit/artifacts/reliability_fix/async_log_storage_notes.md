# Async logger CPU storage migration

This change only migrates CPU-only logger formatting and frame-copy storage from internal static BSS to two permanent heap allocations. It does not change frame-slot ownership, queue depth, protocol priority, slot-state bytes, ring indices, mutexes or DMA buffers. The root build/size report supplies the actual linked memory comparison.

Evidence:

- `sensorarrayFrame_t` aliases `sensorarrayFdcMatrixFrame_t` in `main/sensorarrayTypes.h`. The snapshot consists of numeric fields, fixed arrays and embedded value snapshots; it carries no SPI/I2C DMA transaction, device handle, ISR-owned object or live external pointer.
- `sensorarrayAsyncLogFrameSlot_t` holds that frame, a fixed text packet, duration and timestamp.
- `sensorarrayAsyncLogPublishFrameSnapshot` reserves internal slot state before copying into the frame/text slot outside the critical section, then publishes the internal pending-slot index.
- `sensorarrayAsyncLogPopFrame` transfers internal slot ownership to consumer state 3 before copying out of the slot outside the critical section, and releases the internal slot state afterwards.
- The formatter workspace is only used by `sensorarrayLogTask`; it is not handed to DMA or an ISR.
- `sensorarrayAsyncLogInit` has one caller in main startup. It returns before touching storage when already running; a non-null preexisting task handle also prevents resetting in-use storage.

Implementation:

- New production helper `sensorarrayAsyncLogStorageEnsure` in `main/output/sensorarrayAsyncLogStorage.h` allocates with `heap_caps_calloc(1, bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT)` when PSRAM is configured, then bounded internal-heap fallback using `MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT`.
- Both successful allocations live for the lifetime of the firmware. An allocation failure leaves that pool empty and returns `ESP_ERR_NO_MEM` from Init. If the second pool or task creation fails, already allocated pools are retained and reused on a later Init retry; no successful allocation is replaced.
- The large static workspace and frame-slot array are pointers backed by these two pools. Pool byte count and memset size use `sizeof(*s_frameSlots) * SENSORARRAY_ASYNC_LOG_FRAME_SLOT_COUNT`, never sizeof(pointer).
- No allocation occurs in a frame publication, formatting or consumer loop. Existing internal synchronization and state arrays are unchanged.
- `APP_LOG_INIT` reports `workspaceBytes`, `workspaceMemory=psram|internal`, `frameSlotsBytes`, and `frameSlotsMemory=psram|internal` based on the allocator branch that actually succeeded. The formatter task asserts both pointers are non-null before use.

Actually run:

`python3 tools/test_async_log_storage.py` — exit 0.

`ASYNC_LOG_STORAGE_TESTS,passed=1,psram=1,fallback=1,allfail=1,retry=1,noHotAlloc=1,secondPoolRetry=1`

The test compiles and executes the actual production helper with a fake capability allocator. It verifies PSRAM selection, internal fallback, both-fail empty state, later allocation retry, internal-only configuration, zero initialization, size mismatch rejection, preservation of an already allocated pool when the second fails, and 10,000 repeated two-pool ensures without allocations or data clearing. Log: `async_log_storage.log`.

The allocated byte-count examples in the host allocator test are 13,536 and 28,608; actual target sizes are obtained by sizeof in production and reported by the linked build/runtime log. No target byte-count is hardcoded into the allocator.

Residual: PSRAM copy/formatting latency, runtime free heap/largest block, and actual task stack high-water require HIL and are NOT_RUN here. No fixed FPS limit or sampling delay was added. Internal fallback is a functional bounded allocation path, not a guarantee of sufficient remaining heap for every other subsystem.

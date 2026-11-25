# Wi-Fi memory source map — pinned ESP-IDF v5.5.5

Scope: read-only source/ELF investigation on 2026-11-25. No production source,
SDK, configuration, queue capacity, acquisition rate, build or hardware change
was made by this investigation. No buffer-count change is recommended here.
The AP06 observation and exact-ELF static allocation path are recorded below;
the hook does not provide a runtime caller/backtrace.

SDK root: `/home/csc/esp/esp-idf-v5.5.5`.
Application root: `/home/csc/esp/workspace/SensorArray`.
Paths beginning `components/` below are relative to the SDK root unless marked
application. Line numbers refer to the inspected files, whose key hashes follow.

## Current configuration and application input

| Setting | Current evidence |
| --- | --- |
| PSRAM enabled, octal 80 MHz, capability allocator only | Application `sdkconfig:2028`, `:2034`, `:2042`, `:2051`; `SPIRAM_USE_MALLOC` off at `:2052`; defaults in `sdkconfig.defaults.esp32s3:94-98`. |
| Wi-Fi/LwIP SPIRAM preference off | Application `sdkconfig:2054`. Ordinary libc allocation consequently stays internal by default. |
| Bluedroid host SPIRAM preference on | Application `sdkconfig:1363`, `sdkconfig.defaults:15`; SDK `components/bt/common/osi/include/osi/allocator.h:39-40` uses SPIRAM then internal preference for host malloc/calloc. This is not controller/stack relocation. |
| Wi-Fi buffer inputs | Application `sdkconfig:2203-2217`: static RX 8, dynamic RX 16, dynamic TX type 1/count 16, static RX management/count 5, RX BA window 6; management short count 12 at `:2222`. |
| Application overrides | Application `components/sensorarrayWifi/sensorarrayWifi.c:81-101` starts from `WIFI_INIT_CONFIG_DEFAULT`, then HIGH explicitly sets RX 8/16, dynamic TX 16, RX BA 6, management short 12. BLE_COMPAT is a separate 6/8/8/4/8 policy, not an automatic failure retry. Configuration changes alone do not replace these explicit fields. |
| Other placement options off | Application `sdkconfig:2055-2056` BSS/noinit external placement off; NVS cache SPIRAM off at `:2859`. |
| Stacks and failure behavior | Application `sdkconfig:2393` permits explicit external static task creation but does not move dynamic task allocations; heap poisoning light on `:2421`, abort-on-allocation-failure off `:2428`. |

`WIFI_INIT_CONFIG_DEFAULT()` wires `g_wifi_osi_funcs` and configuration fields
in SDK `components/esp_wifi/include/esp_wifi.h:316-342`.

## Allocation paths and hard placement requirements

| API/path | Exact implementation and implication |
| --- | --- |
| Public init | `components/esp_wifi/src/wifi_init.c:348` calls binary `esp_wifi_init_internal(config)` at `:445`, then supplicant init at `:481`. On error it calls private deinit at `:518` but returns the original init error at `:523`; reclamation cannot be inferred from NO_MEM alone. Application guards this at `components/sensorarrayWifi/sensorarrayWifi.c:206-214`. |
| Wi-Fi CPU allocation family | `components/esp_wifi/esp32s3/esp_adapter.c:71-108`: `wifi_malloc`, `wifi_realloc`, `wifi_calloc`, `wifi_zalloc_wrapper`. With `SPIRAM_TRY_ALLOCATE_WIFI_LWIP`, use `heap_caps_*_prefer` with DEFAULT\|SPIRAM then DEFAULT\|INTERNAL. Currently use libc `malloc/realloc/calloc`. OSI hooks are `:726-729`. |
| General OSI malloc | Same adapter `:679`: `._malloc = malloc`. Wi-Fi preference does not rewrite this hook. |
| Internal/DMA family | Adapter `:412-424`: realloc/calloc/zalloc require `8BIT\|DMA\|INTERNAL`. Malloc hook `:722` calls `esp_coex_common_malloc_internal_wrapper`, implemented in `components/esp_coex/esp32s3/esp_coex_adapter.c:127-129` with the same caps. Neither PSRAM preference nor generic malloc settings remove these requirements. |
| General OSI queue | Adapter `:258-273`, hook `:659`: `queue_create_wrapper` already allocates StaticQueue_t plus storage SPIRAM-first unconditionally, then internal fallback. This is distinct from the Wi-Fi-specific queue API. |
| Wi-Fi-specific queue | Adapter `:112-149`, hook `:730`: metadata explicitly INTERNAL\|8BIT. With USE_MALLOC, storage/control object explicitly INTERNAL\|8BIT; otherwise `xQueueCreate` obtains internal FreeRTOS memory. |
| Tasks and FreeRTOS objects | Adapter task hooks `:330-337` call ordinary `xTaskCreate[ PinnedToCore ]`. `components/freertos/heap_idf.c:14-20`, `:43-55` require INTERNAL\|8BIT for dynamic task stacks, TCBs and queues because they must remain accessible with caches disabled. |
| LwIP CPU allocation | `components/lwip/port/include/lwipopts.h:1708-1714`: same Wi-Fi/LwIP preference flag changes `mem_clib_malloc/calloc` to SPIRAM-first, internal fallback; otherwise libc. It does not move the TCP/IP task stack. |
| Static RX, dynamic TX, static TX | Pinned SDK `docs/en/api-guides/wifi.rst:4188-4215` classifies all three as DMA buffers. Static RX and static TX are created at Wi-Fi init; dynamic TX on transmission. Dynamic RX is copied from static RX and passed upward (`:4172-4174`, `:4195-4199`). Do not treat all Wi-Fi buffers as movable CPU-only storage. |
| Libc | `components/newlib/src/heap.c:22-29`, `:38-50`: malloc and calloc route through `heap_caps_malloc_default`; realloc through `heap_caps_realloc_default`. With external libc allocation disabled, `components/heap/heap_caps.c:107-110`, `:140-143` request DEFAULT\|INTERNAL. A malloc hook failure therefore does not distinguish libc malloc from calloc. |

## MEMFAIL interpretation

Capabilities are defined in `components/heap/include/esp_heap_caps.h:29-41`:

| Caps | Meaning | What is established |
| --- | --- | --- |
| `0x804` | INTERNAL\|8BIT | Explicit internal byte-addressable request, including ordinary FreeRTOS allocation. Not sufficient to identify a particular task or queue. |
| `0x80c` | INTERNAL\|DMA\|8BIT | Explicit internal DMA request; consistent with the Wi-Fi internal allocator family. Cannot be satisfied by relocating that allocation into PSRAM. |
| `0x1800` | INTERNAL\|DEFAULT | Current libc path, or the final fallback of a preference allocator. The allocator API name distinguishes those cases. |
| `0x1400` / `0x404` | SPIRAM\|DEFAULT / SPIRAM\|8BIT | Explicit external-capability request. An external failure alone does not establish internal exhaustion. |
| `0x1000` | DEFAULT | Can match eligible internal or external heap; not equivalent to an explicit internal requirement. |

The callback receives the allocator's `__func__`, not the Wi-Fi caller or a
return address (`components/heap/heap_caps.c:82-88`, `:241-259`). For a
`*_prefer` API it fires only after all preferences fail and receives the **last**
attempted caps (`:172-188`, `:196-212`, `:220-235`). Thus a preference failure
with `0x1800` can mean both PSRAM and then internal failed, unlike plain
`heap_caps_malloc` with the same caps. The pinned `heap_caps_calloc_prefer`
reports **size**, not `n * size`, at `:235`; direct `heap_caps_calloc` reports
`n * size` at `:259`. Sizes exclude heap alignment/poisoning overhead.

Application `main/control/sensorarrayMemoryDiagnostics.c:62-85` stores only
first/last/count and opaque API-string pointer. Its bounded try-lock can miss
callbacks, reported through `incomplete`. A pre/post count change establishes
recorded failures since registration, not a caller stack or every attempted
fallback. Existing pre/post MEM snapshots are sequential observations, not
memory statistics taken at the exact failing allocation.

Offline API-string addresses were decoded from archived ELF SHA-256
`90f10fdaa5a0b859995caba70b1f1c65d3c7f9e19e85b39c81366e699c6e2736`, with the
archive hash checked against `heap-failure-02-image-archive.json`. See
`heap-failure-02-allocator-api-strings.json`: plain malloc string
`0x3c1506dc`; calloc `0x3c15064c`; realloc `0x3c150660`; malloc_prefer
`0x3c15068c`; calloc_prefer `0x3c150674`. These addresses apply only to that
exact ELF and are string locations in `.flash.rodata`, not code addresses.

## SDK-supported placement options, not proposed production changes

1. `SPIRAM_TRY_ALLOCATE_WIFI_LWIP` is supported with current CAPS_ALLOC mode
   (`components/esp_psram/Kconfig.spiram.common:109-115`). It changes the CPU
   allocation hooks above while retaining explicit internal/DMA requests.
   It also disallows dynamic TX when PSRAM is mandatory
   (`components/esp_wifi/Kconfig:78-82`) and enables the static TX/cache model
   (`:90-116`). Static TX default is 16 buffers, approximately 1.6 KB each,
   allocated at init; cache default is 32. Enabling this flag alone therefore
   has a material internal-init-memory consequence. No count adjustment is
   justified or proposed without the actual failure evidence.
2. `NVS_ALLOCATE_CACHE_IN_SPIRAM` targets NVS page-cache and key-hash CPU
   structures and works with CAPS_ALLOC (`components/nvs_flash/Kconfig:39-47`).
   Its allocator uses SPIRAM-first/internal fallback in
   `components/nvs_flash/src/nvs_memory_management.hpp:57-71`. It does not
   alter Wi-Fi DMA or FreeRTOS stacks; savings depend on actual NVS cache use.
3. Explicit application CPU-buffer `heap_caps_malloc(..., SPIRAM|8BIT)` is
   supported in current mode (`components/esp_psram/Kconfig.spiram.common:73-83`).
   Every candidate still needs an ownership/cache-disabled-access audit.
4. `SPIRAM_USE_MALLOC` plus ALWAYSINTERNAL/RESERVE_INTERNAL is a broader
   global allocation-policy option (`components/esp_psram/Kconfig.spiram.common:84-86`,
   `:97-107`, `:117-135`). It preserves explicit internal capability requests
   and dynamic FreeRTOS placement, but changes libc behavior across the app.
   Reserve is not new RAM and need not be contiguous. This is not the current
   configuration or a narrow Wi-Fi-only relocation.
5. External BSS placement is supported (`:137-146`) but automatically affects
   Wi-Fi/LwIP and **SDK-selected BT component library BSS**, including
   `components/bt/controller/linker_esp_ble_controller.lf:39-46` and
   `components/bt/linker_common.lf:9-13`. The selected archive is `libbt.a`,
   including Bluedroid host objects; the linker file living under `controller/`
   does not establish that all controller memory is moved. Audit the selected
   sections separately from explicit internal/DMA requests and task stacks.

## AP06 result and smallest supported candidate

`wifi-control-06/wifi-control-06.summary.json` binds the new exact ELF above,
session `EE880CC8223D06C2`, boot 1, and unchanged CAP8 configuration. Recorded
allocation failures increased from zero to one with `incomplete=0`: first/last
size **1604**, caps **0x80c**, API-string pointer **0x3c1506dc**. This is a direct
`heap_caps_malloc` request for **INTERNAL|DMA|8BIT**, not a preference failure.
Internal free/largest changed from 44,856/31,732 to 28,480/9,716 bytes; observed
minimum was 7,996. The request completed in 34.050 ms with driver-init NO_MEM,
unknown cleanup retained as `0x103` and quarantine. These heap snapshots do not
measure the largest DMA block at the failure instant.

The archived ELF contains a matching static RX allocation path:

- `wdev_data_init` assigns `pTxRx` to `TxRxCxt`; the ELF data at
  `TxRxCxt + 0x3b8` (`0x3fcad050`) is `0x640`, or 1600.
- `wDev_Rxbuf_Init` (`0x420e8d4c`) reads that size at `0x420e8da9`, adds four
  bytes at `0x420e8dae`, and invokes OSI offset `0x158` at `0x420e8dce`.
  CSI is disabled in the actual configuration; its larger-buffer branch is
  not selected. The allocation is repeated for the static RX buffer count.
- `g_wifi_osi_funcs + 0x158` (`0x3fcac72c`) contains `0x40377418`, the exact
  `esp_coex_common_malloc_internal_wrapper`. That wrapper loads `0x80c` and
  calls `heap_caps_malloc` at `0x40377421`.
- The null-result branch references the ELF string `malloc buffer fail`;
  short initialization reports the expected/actual RX count, calls RX deinit,
  and returns `0x101` at `0x420e8eba`.

Disassembly and decoded data are in `wifi-memory-allocator-path-01/`; the
allocation branch is also disassembled from its exact branch target because
linear Xtensa disassembly can decode alignment padding as instructions.
This is a strong binary match for static RX initialization, **not** direct
runtime caller proof from MEMFAIL.

The smallest supported candidate with substantial memory potential is to
disable `ESP_WIFI_IRAM_OPT` and `ESP_WIFI_RX_IRAM_OPT` in repo configuration,
then verify the actual map and runtime memory. This retains buffer counts,
DMA placement, controller placement, task stacks and acquisition settings.
It trades Wi-Fi throughput and instruction-cache pressure for internal RAM:

- `components/esp_wifi/Kconfig:274-281` and `:292-299` explicitly describe
  optional speed optimizations and their performance tradeoffs; no hard
  ISR/cache safety dependency is attached to these flags.
- `components/esp_wifi/linker.lf:74-81` maps the optional Wi-Fi sections to
  flash by default; `:111-124` and `:135-148` opt them into IRAM. Mandatory
  `.iram1` code is outside these optional section switches. SLP_IRAM_OPT is
  already disabled in the inspected configuration.
- `components/heap/port/esp32s3/memory_layout.c:45-65` gives the dual-use
  D/IRAM region INTERNAL|DMA|8BIT capabilities. `:124-128` reserves the DRAM
  alias of the actual `_iram_start.._iram_end` extent. A smaller linked IRAM
  extent can therefore return DMA-capable memory to the heap. The old/new
  map and boot measurement must establish the actual number of freed bytes.

AP success and coexistence throughput still need actual validation. If more
internal memory is required, the narrow official NVS-cache SPIRAM option
above avoids the Wi-Fi TX-pool policy change; its actual savings are unknown
until measured. Global malloc or Wi-Fi/LwIP preference is broader and should
not be conflated with this optional-code-placement candidate.

The baseline AP05 evidence remains: ELF `2f74130825f1b4c173cee27bf569bc4385bbdd26a7049adee3efe6e8b77c39e9`,
session `55B0EC5D078FCD53`, boot 1, AP terminal NO_MEM, cleanup `0x103`,
quarantined. Internal free/largest before 45,716/31,732 bytes, after
29,076/10,740, observed minimum 8,592. Source:
`wifi-control-05/wifi-control-05.summary.json`. This does not establish the
failed request's size/caps or a complete memory-allocation ledger.

## Subsequent hardware observations

The earlier candidate analysis above is preserved as source reasoning, not a
claim that optional IRAM relocation alone repaired AP. `wifi-control-07` on
ELF `2fed45c22153b580aef27584107809d15cdabdd8afe6d6bd90cf6d5a282c07a7`
released 18,432 bytes in the map but panicked in AP startup; its post-attempt
MEMFAIL is unavailable. `wifi-control-08` on ELF
`179f9183e2c31b35c906c9b811bb2ca1cf4d16dbca43ee3a1c6c4371c0f56a2d` then used
SDK external BSS plus the task-owned FDC calibration table (29,116 bytes total,
including 8,704 calibration bytes). Its five AP/AP/OFF/AP/OFF requests applied
in the same boot, MEMFAIL stayed zero with incomplete zero, and the BLE observer
closed cleanly. This is AP startup/coexistence control evidence; actual host UDP
receipt and sustained throughput remain untested. The exact run limits and
historical results are in `../../firmware_reliability_fix_report.md`.

## Source SHA-256 at inspection

| File | SHA-256 |
| --- | --- |
| Application sdkconfig | `f1ba1ca0ff492eac452219161d985f4677e21684a86be09ea3468a7bd2da6c3a` |
| Application sdkconfig.defaults | `ace8bf23cfea213bb3d553da25ea100b6150adf7737ab5d050f71407af3e48b0` |
| Application sdkconfig.defaults.esp32s3 | `f6beaae38b5badb5c7a4216af76b1db82a631ea95555e19e3f0694dd40c5ad16` |
| components/esp_wifi/esp32s3/esp_adapter.c | `0bd67f85a88f0c1bb0d6a0412a48c767de7e2eab76c1ee8a05db56920c512d6d` |
| components/esp_wifi/src/wifi_init.c | `aee4898a95c946766e9e0ceae37c3f7a5e358665a19768c284f6dd0a648a5887` |
| components/esp_wifi/Kconfig | `d3fa69f3b42d007d81b35f4cc51674bed4b787deade22dcb0f79128ef1617717` |
| components/esp_psram/Kconfig.spiram.common | `0ea5bf089440c8cf32e2702fdfc690af1866f94f859339188dd6b450a3ce9812` |
| components/heap/heap_caps.c | `a1ddc96cda0df2758552c611e409bde6358b8b18145470f928a33cb59628f2a1` |
| components/heap/include/esp_heap_caps.h | `5fd9bf5e089788bc91b81474047379c96826c768607f0f7ad4b8e0930a7de3ec` |
| components/esp_coex/esp32s3/esp_coex_adapter.c | `7fb34f00fb22a158b73252f4be55800b429623153a050998b5e3a1fb015b5c1d` |
| components/lwip/port/include/lwipopts.h | `2c9aacaa8e152d5270fabf0b76eefd2801d051c847b8ef875d12a17259d2f4ac` |
| components/freertos/heap_idf.c | `6201b70a245f721c3021656d62fbbbf45c3e4d5d843365df007d0da9ac99db2e` |
| components/newlib/src/heap.c | `6071b8c2b4fe82fb840d149783d362dcaf37685215c233fb69e0f82eef18c1b2` |

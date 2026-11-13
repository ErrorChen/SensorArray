# USB-OTG 与 BLE 优先级修复 / USB-OTG and BLE priority repair

状态 / Status: implementation and regression in progress; not final HIL acceptance. Historical USJ evidence is committed at `16f41fa`, with running firmware baseline `be09762` read back and SHA-256 verified. The app-only rollback image matches `32d66c6bf3bd562b09422965228045c3399ed5b114a910a9528f66d1677e9241`; no NVS/calibration/partition/bootloader write occurred during backup.

用户明确选择 USB-OTG 正式数据接口，BLE 正式数据优先，CDC 仅调试。本阶段不能沿用旧 USJ 的 120 s／中止 186.137 s 结果作为新接口通过。/ The user explicitly selected a new OTG formal interface and BLE data priority; previous USJ measurements do not validate it.

| Evidence / 证据 | Function / 函数 | Change / 修改 | Verification / 验证 |
| --- | --- | --- | --- |
| USJ has no DMA; S3 OTG has internal DMA | new `sensorarrayUsbOtgOwnerInit`, descriptors | pinned TinyUSB 0.21.0~1; 2 vendor + CDC; internal aligned DMA buffers | target builds; HIL pending |
| Shared wire/log capacity could stall formal traffic | `sensorarraySerialOtgTxTask` (new) | separate endpoint state, CTRL/DATA/debug scheduling; same persistent payload pool | real C partial/no-host/epoch regressions |
| Reset interrupt precedes reset event processing | `sensorarrayUsbOtgOwnerPoll`, `ConsumeCommands` | processed-epoch fence, queue-empty proof, per-chunk checks | real C event-17 and reset-during-read regressions plus negative mutation |
| ESP_FAIL collided with pending sentinel; retry could falsely succeed | `sensorarraySerialIoInit` | independent done/result, repeat init checks actual result | real C failure/timeout/retry cases |
| CPU-only restart does not prove OTG DMA stopped | new `PrepareRestart`, `OwnerQuiesce`, emergency wrapper | final owner hardware-reset fence; no uninstall/free; bounded Core Reset escalation | host cases; final ELF/ROM handoff HIL pending |
| BLE DATA admission drops without fragment CRC errors | BLE task/queue/negotiation code | split DATA/LOG descriptors, retained CTRL/lifecycle capacity, actual negotiation reports | real C scheduling/reconnect/generation tests; HIL pending |
| CDC-only MFAULT would evade formal clients | new `sensorarrayTransportPublishRuntimeFault` | bounded priority USB and BLE lifecycle publication, optional CDC mirror | real publisher regression; HIL pending |
| New endpoint provenance must not falsify original streams | `UsbBulkPort`, `Collector`, benchmark tools | separate DATA/CTRL raw files and partial lines, exact interface matching | fake backend + actual encoder fixtures; Linux HIL pending |

The existing ADC/FDC fixes, caches, adaptive PGA, per-row modes and science semantics remain intact. Original audit numbering FW01–FW10 is not replaced. Protocol and electrical limitations are in [host contract](../integration/firmware_host_contract.md) and [hardware notes](../hardware/reliability_revision_notes.md).

所有回归中的合成事件均明确标 synthetic；真实板卡原始日志不覆盖。Windows、示波器、物理拔插和电气精度未执行时保留 NOT_RUN。/ Synthetic event tests are labeled; original captures are preserved. Windows, scope, physical unplug and electrical precision remain NOT_RUN unless actually exercised.

# VOLT2 USB incomplete delivery: offline evidence, 2026-11-25

本证据只分析已关闭的`bss-matrix-02-volt2`录制，不更改原始记录、既有FAIL判定、生产代码或验收门槛。结果见[analysis.json](analysis.json)，可重复检查见[analyze.py](analyze.py)。

This bundle analyzes the closed recording only. It does not change the original FAIL verdict or establish the initial cause of the backpressure. The replay performs no device, network or git operation.

## Identity and result

- Full ELF: `179f9183e2c31b35c906c9b811bb2ca1cf4d16dbca43ee3a1c6c4371c0f56a2d`.
- Session `05DD194E2065ADB0`, boot `2`, rev `100`, VOLT2; mgen/mrid `69/100`, rgen/rrid `24/99`, pgen/prid `77/100`.
- Capture monotonic window: `176489.879441053..176609.879547619`; formal event coordinates: `5.393958289991133..125.39407450897852` seconds.
- [Original gate](../bss-matrix-02/bss-matrix-02-volt2/bss-matrix-02-volt2.gate.json): `FAIL`, `USB_complete_sequence_gap`. USB has **5,043** trusted complete frames; BLE has **5,050**, with **zero** missing inside the same endpoints `76513..81562`.
- All **5,043** common frames have identical original wire bytes. The seven complete frames absent from USB are **77016, 77021, 77022, 77023, 77024, 77025, 77026**. BLE reassembly and the recorded parser independently verify their CRC and complete configuration identity.

结论：缺口是真实的USB完整交付缺失。它不是capture边缘筛选或解析器把完整原帧漏算。原始字节与本地计数强匹配“一个已开始的DATA记录超时后终止，加六次DATA接纳拒绝”；计数不含逐sequence trace，因此仍需保留这个因果归属限制。

The USB recording is genuinely incomplete. A partial record followed by six absent records strongly matches the local expiry/admission-rejection signature. The counters do not identify individual sequences, so this is corroborated attribution, not a per-sequence execution trace.

## Exact raw/BLE alignment

Offsets below are zero-based positions in the original formal `.rx.bin`. BLE line numbers are one-based journal lines, unchanged by gzip archival. Each listed BLE frame is 713 bytes with valid frame CRC and matching identity.

| Sequence | USB raw observation | BLE completing journal line | BLE CRC | BLE completing host monotonic |
| --- | --- | ---: | --- | ---: |
| 77016 | Offset **534190**, **331 B** until next header at **534521**; exact first **330 B** of BLE frame plus LF, with original final **383 B** missing | 1453 | `83478C50` | 176502.110087088 |
| 77021 | No `seq=77021,` token anywhere in formal raw | 1463 | `FCC0E2AC` | 176502.110242065 |
| 77022 | No `seq=77022,` token anywhere in formal raw | 1465 | `A9461A76` | 176502.110289072 |
| 77023 | No `seq=77023,` token anywhere in formal raw | 1467 | `F4051A6B` | 176502.110322714 |
| 77024 | No `seq=77024,` token anywhere in formal raw | 1469 | `F215137A` | 176502.110345877 |
| 77025 | No `seq=77025,` token anywhere in formal raw | 1471 | `ED1FA1C9` | 176502.110368840 |
| 77026 | No `seq=77026,` token anywhere in formal raw | 1473 | `7F418247` | 176502.110392018 |

77016的USB残头在`,ir=0,to=0,st=0\n`结束；没有其D/P/K。77017随即开始且完整、与BLE完全相同。77020的完整K之后，raw offset537374直接开始77027，之间没有CTRL文本、DATA正文或六个缺失序号的header/trailer。完整帧缺口分别是`77015→77017`（1）与`77020→77027`（6）。本次实际parser在新header处放弃未完成旧frame；分析器独立记录`truncated_frames_capture=1`，因此CRC error0并不等于没有截断。

The partial 77016 header has no data/PGA/trailer. Its following 77017 frame is intact. The six later sequences are entirely absent by their original header/trailer identifiers. No priority text interleaves at either discontinuity. The parser correctly abandons the unfinished frame when the next header starts; zero complete-frame CRC errors do not mean zero truncations.

## Local counters and matching-image code

The nearest surrounding `USBQ?` receipts are at formal times **15.43490106597892** and **25.517540853994433** seconds:

| Counter | Before | After | Delta |
| --- | ---: | ---: | ---: |
| dDrop (`dataRejected`) | 0 | 6 | **+6** |
| expired (`normalExpired`) | 3789 | 3790 | **+1** |
| partial (`partialWrite`) | 24 | 25 | **+1** |
| cDrop (`priorityRejected`) | 0 | 0 | 0 |
| cExpired (`priorityExpired`) | 0 | 0 | 0 |

All sampled USB connection state remains epoch2/resets2/mounts1/unmounts0/mounted1/suspended0. No observed reset/detach explains the gap. The debug stream independently contains `SF50,seq=77001-77050,...,drop=0/0/6` at byte49211 and ADC health for sequence77024 at byte49015; the latter further confirms acquisition progressed through the missing USB interval.

`normalExpired` covers DATA and LOG, and `partialWrite` aggregates lanes. Neither counter directly names sequence77016. Their single increments, the exact original prefix followed by LF, the single capture truncation and unchanged connection state make the ordinary expiry path a strong match. The six DATA rejections correspond numerically to six wholly absent records, but the current telemetry does not timestamp or identify each rejection.

Static references use the fixed [source snapshot](source-snapshot/core/transport/sensorarraySerialIo.c), obtained with explicitly authorized read-only `git show f8d8424492c855a402838737ee8718e32f5135d6:<path>`. Each whole file matches both `inputs_before` and `inputs_after` in the [original image manifest](../wifi-bss-01-build-manifest.json); the replay excludes concurrent production edits.

- [sensorarrayUsbControl.c](source-snapshot/main/control/sensorarrayUsbControl.c):52–55 establishes the exact USBQ field-to-counter mapping used above.
- `sensorarraySerialIo.c:25–45,130–155`: DATA has five slots/four queue entries; normal records retain the original **250,000µs enqueue deadline**, priority records 1,000,000µs. Failed slot/queue admission increments `dataRejected` for DATA.
- `sensorarraySerialIo.c:486–503`: a normal pending record expires at that original deadline even after a prefix has been written. The code increments normal expiry and partial counters, releases the slot, and requests a delimiter when the prefix was written without an epoch change.
- `sensorarraySerialIo.c:504–547`: the shared formal stream prevents another lane from interleaving a started record; after expiry the next successful write emits LF before the next record. This exactly matches the observed extra LF after 330 original bytes.
- [sensorarrayUsbOtg.c](source-snapshot/components/sensorarrayUsbOtg/sensorarrayUsbOtg.c):298–337 checks that FIFO availability is nonzero and returns the number actually copied by `tud_cdc_n_write`. It does not require enough available FIFO space for the entire record.
- [tusb_config.h](source-snapshot/components/sensorarrayUsbOtg/include/tusb_config.h):21–22 defines the 4,096-byte CDC TX FIFO and 512-byte transfer buffer; the FIFO is non-overwritable. This static configuration is evidence context, not a proposal to enlarge it.

The code path and counters support bounded USB backpressure/admission behavior. No evidence supports scientific-parser loss, frame-byte reconstruction, priority-record interleaving, or changing capture rate to explain these seven sequences.

## Initial stall: observed, not assigned a root cause

Formal RX event lines **3414→3415** have a **0.622053006-second** receipt gap (`t=17.002156459988328→17.624209465982858`). Their raw offsets remain contiguous at524208. Debug RX lines **320→321** pause **0.658895849 seconds**, then resume at host monotonic176502.109676673. BLE notification lines **1439→1440** pause **0.424112111 seconds**, then resume at176502.109839848. USB formal/debug and BLE receipt activity resumes in the same small host-time neighborhood, with several BLE frames subsequently delivered in a burst.

这支持共同host交付停顿/排队的迹象，但不能仅由callback时刻判定是主机调度、TTY读取、USB控制器/主机栈、还是设备服务延迟。BLE最终完整无缺与USB连接计数不变不能单独排除这些路径。正式文件全部**24,801**个RX事件连续覆盖**3,809,266 B**，原始SHA与collector summary一致；这排除了已记录数据的offset洞或文件写入截断，但不能证明字节到达主机程序之前的链路从未停顿。

The synchronized receipt pause is a clue to queued host delivery, not proof of its initiating cause. Existing callback timestamps cannot distinguish host scheduling, TTY/USB-stack backlog and device service delay. The file-accounting check excludes holes in the recorded byte stream; it does not prove continuous upstream service.

## Reproduce without hardware

From any working directory, run:

```sh
python3 /home/csc/esp/workspace/SensorArray/docs/audit/artifacts/cdc_recovery_acceptance_20261116/bss-volt2-usb-stall-analysis-01/analyze.py
```

The script uses archived, SHA-matched parser/production source and original recordings, accepting `.gz` counterparts when raw files have been removed after verified archival. It asserts file accounting, all seven exact sequences, 5,043 common identical frames, 5,050 consecutive BLE frames, original-prefix/LF behavior and the single USBQ delta window, then writes JSON to stdout. The saved [analysis.json](analysis.json) is that output. No gate, source, threshold, original byte or hardware state is changed. The original gate remains **FAIL** and overall acceptance remains **NOT_ESTABLISHED**.

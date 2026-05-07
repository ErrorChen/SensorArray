# main / Application Layer

`main/main.c` remains the firmware entry point. `app_main()` selects one of:

- `PIEZO_READ`: default ADS126x voltage scan, `SW=HIGH`, software source `GND`
  / zero path, REFOUT off, VBIAS off, `REFMUX=AVDD/AVSS`.
- `RESISTANCE_READ`: ADS126x voltage scan with software source `REF`, internal
  REF on, VBIAS on, and REF/MID path preserved.
- `DEBUG`: bring-up and FDC2214 debug dispatcher.

`PIEZO_READ` and `RESISTANCE_READ` do not initialize or poll FDC2214 during the
normal ADS126x voltage scan path.

## Startup Sequence

The voltage read modes share one initialization path:

1. Print `APPMODE` and `BUILD_CONFIG`.
2. Initialize board support, TMUX, and ADS126x.
3. Apply the selected ADS reference policy.
4. Print `DBGADSREFPOLICY`.
5. Apply selected TMUX route policy.
6. Print `DBGROUTEPOLICY` and `DBGTMUXPOLICY`.
7. Enter either FAST/MAX stream tasks or the SAFE legacy CSV loop.

Default FAST/BINARY startup should include:

```text
RESET_REASON,reason=...,heapFree=...,heapMinFree=...
APPMODE,active=PIEZO_READ,...
BUILD_CONFIG,appMode=PIEZO_READ,profile=FAST,output=BINARY,csvEvery=0,dedicatedTasks=1,queueDepth=16,usbNonblocking=1,commit=<hash>
DBGADSREFPOLICY,mode=PIEZO_READ,stage=piezo_no_ref_settled,...
DBGROUTEPOLICY,mode=PIEZO_READ,sw=GND,...
DBGTMUXPOLICY,mode=PIEZO_READ,stage=voltage_scan_init,...
ADS_FAST_CONFIG,dr=15,...
VOLTSCAN_CONFIG,mode=PIEZO_READ,dr=15,format=binary,queueDepth=16,...
STREAM_MEM,streamFrameSize=...,compactFrameSize=312,queueDepth=16,queueBytes=...,commStack=16384,scanStack=12288,heapFree=...,heapMinFree=...
STREAM_INIT,format=binary,magic=0x31434153,magicBytes=SAC1,version=1,frameType=0x1261,frameSize=312,csvEvery=0,queueDepth=16,statusEvery=1000,textStatus=0
```

## FAST/MAX Stream Architecture

FAST/MAX does not use the legacy `sensorarrayRunVoltageReadMode()` CSV hot loop.
It starts `sensorarrayVoltageStreamStart()` and then idles in `main.c`.

- `sensorarrayVoltageScanTask`: owns TMUX and ADS126x SPI, scans 64 points, and
  sends frames to the queue. It does not call `printf`, `fprintf`, `fwrite`, or
  `ESP_LOGI` in the hot path.
- `sensorarray_out`: owns USB Serial/JTAG stdout. It prints startup config,
  writes compact binary frames, optional sampled CSV, `STAT`, and `RATE_EVENT`.
  In pure FAST/BINARY, `STAT` and `RATE_EVENT` are disabled after `STREAM_INIT`
  unless `CONFIG_SENSORARRAY_BINARY_TEXT_STATUS=y`.
- Queue sends are nonblocking in FAST/MAX. Queue full events update `qFull`,
  `drop`, and frame metadata.

FAST/BINARY output behavior:

```text
CONFIG_SENSORARRAY_OUTPUT_FORMAT_BINARY=y
CONFIG_SENSORARRAY_VOLTAGE_SCAN_CSV_EVERY_N=0
```

Each published frame writes one 312-byte compact binary frame. It does not write
`MATV_HEADER`, `MATV`, `MATV_RAW`, `MATV_GAIN`, periodic `STAT`, or
`RATE_EVENT` in the default pure-binary profile. Ordinary serial monitors will
show binary data after `STREAM_INIT`; host tools must resync by `SAC1`, frame
size 312, and CRC32.

For manual text diagnostics, enable `CONFIG_SENSORARRAY_BINARY_TEXT_STATUS=y`,
select `CONFIG_SENSORARRAY_OUTPUT_FORMAT_BOTH`, or use SAFE/CSV.

## SAFE Legacy CSV

SAFE keeps the compatibility path:

```text
MATV_HEADER,seq,timestamp_us,duration_us,unit,S1D1,...,S8D8
MATV,<seq>,<timestamp_us>,<duration_us>,uV,<64 int32 microvolts>
```

CSV is printed only when:

```text
CONFIG_SENSORARRAY_OUTPUT_FORMAT_CSV=y
```

or:

```text
CONFIG_SENSORARRAY_OUTPUT_FORMAT_BOTH=y
```

and `CONFIG_SENSORARRAY_VOLTAGE_SCAN_CSV_EVERY_N > 0`.

To switch back for debug:

```text
SensorArray application mode -> PIEZO_READ or RESISTANCE_READ
Voltage scan throughput profile -> SAFE
SensorArray output format -> CSV
CSV every N frames -> 1
```

Then run:

```bash
idf.py fullclean
idf.py reconfigure
idf.py build
idf.py flash monitor
```

## Compact Binary Frame

`sensorarrayVoltageCompactFrame_t` is fixed at 312 bytes and has a build-time
static assert:

```text
magic      = 0x31434153, bytes "SAC1"
version    = 1
frameType  = 0x1261
microvolts = S1D1, S1D2, ..., S8D8
crc32      = IEEE CRC32 over all previous bytes
```

The point order matches the old `MATV` CSV exactly.
Passive drops and active decimation are distinct saturated 16-bit fields in the
compact frame; full cumulative counts are also printed in `STAT`.
If GUI `crc_errors`, `resyncs`, or `parse_errors` climb continuously, first
check whether mixed text/binary output was enabled.

## STAT Fields

`STAT` now exposes the likely bottleneck directly:

```text
scanFps     ADS/TMUX scan task frame rate
outFps      successful compact binary frame output rate
scanAvgUs   average 64-point scan duration
usbAvgUs    average bounded binary stdout write time
usbMaxUs    maximum bounded binary stdout write time
qDepth      configured queue depth
qUsed       queue fill at STAT time
qFull       queue-full events
drop        passive queue/output drops
decimated   active output skips from rate control
shortWrite  partial nonblocking stdout writes
writeFail   zero-byte/nonrecoverable stdout write failures
outputDiv   active output decimation divisor
outStackMinWords  minimum observed output task stack high-water mark
scanStackMinWords minimum observed scan task stack high-water mark
heapFree / heapMinFree  current and minimum free 8-bit heap
```

Interpretation:

- Low `scanFps`: ADS/TMUX/settle/DRDY/SPI is limiting.
- `scanFps` normal but `outFps` low: USB stdout or host parser is limiting.
- Rising `qUsed`, `qFull`, or `drop`: output task is behind scan task.
- Rising `decimated` or `outputDiv > 1`: auto rate control is protecting output.
- Rising `shortWrite` or `writeFail`: nonblocking USB Serial/JTAG stdout is slow
  or the host is not reading fast enough.

## Stack And REF Diagnostics

Task names are intentionally short so FreeRTOS panic output is unambiguous:

```text
sensorarray_out   output/USB stdout task
sensorarray_scan  TMUX/ADS scan task
```

For `sensorarray_out stack overflow`, confirm `STREAM_MEM` reports
`commStack=16384`, the output path is not mixing periodic text into FAST_BINARY,
and text debug `STAT` still shows healthy `outStackMinWords`. For
`sensorarray_scan stack overflow`, inspect scan-frame and ADS hot-path stack use.

Periodic REF pulses during a reboot loop should be treated first as a secondary
effect of MCU reset and ADS/route reinitialization. If resets stop but REF still
toggles, debug the REF/SW/ADS reference policy separately.

Recent stream/output status codes:

```text
0x3006 STREAM_INTERNAL_ERROR
0x3007 OUT_STACK_LOW
0x3008 OUT_STACK_CRITICAL
0x3009 BINARY_TEXT_SUPPRESSED
0x300A BINARY_WRITE_PARTIAL
```

## Boundary

This firmware still uses ESP32-S3 USB Serial/JTAG stdout. USB Vendor Bulk
Transfer is a future transport option, not part of this fix. Collect
FAST/BINARY `scanFps`, `outFps`, queue, stdout-write, and host parser data
before deciding whether Vendor Bulk is necessary.

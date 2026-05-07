# SensorArray Firmware

This repository builds the ESP32-S3 embedded firmware for the SensorArray
hardware. The default branch is `main`; the default application mode is now the
high-speed ADS126x voltage scan path.

## Default Mode

Default firmware configuration:

```text
CONFIG_SENSORARRAY_APP_MODE_PIEZO_READ=y
CONFIG_SENSORARRAY_VOLTAGE_SCAN_PROFILE_FAST=y
CONFIG_SENSORARRAY_OUTPUT_FORMAT_BINARY=y
CONFIG_SENSORARRAY_VOLTAGE_SCAN_CSV_EVERY_N=0
CONFIG_SENSORARRAY_SCAN_USE_DEDICATED_TASKS=y
CONFIG_SENSORARRAY_STREAM_QUEUE_DEPTH=16
CONFIG_SENSORARRAY_USB_STDOUT_NONBLOCKING=y
CONFIG_SENSORARRAY_AUTO_RATE_CONTROL=y
CONFIG_SENSORARRAY_AUTO_RATE_CONTROL_OUTPUT_FIRST=y
```

`PIEZO_READ` scans `S1-S8 x D1-D8` through ADS126x with `SW=HIGH`, software
source `GND` / zero path, REFOUT off, VBIAS off, and `REFMUX=AVDD/AVSS`.
`RESISTANCE_READ` keeps the REF/MID path with internal REF and VBIAS enabled.
Debug and FDC2214 bring-up modes are still available through `DEBUG`.

The firmware still uses ESP32-S3 USB Serial/JTAG stdout. It does not switch to
USB Vendor Bulk Transfer.

## Build / Flash / Monitor

When changing profiles or output format, force Kconfig regeneration before
flashing:

```bash
idf.py set-target esp32s3
idf.py fullclean
idf.py reconfigure
idf.py build
idf.py flash monitor
```

## Expected Startup

Default FAST/BINARY firmware should print a short ASCII startup section before
the compact binary stream:

```text
RESET_REASON,reason=...,heapFree=...,heapMinFree=...
APPMODE,active=PIEZO_READ,...
BUILD_CONFIG,appMode=PIEZO_READ,profile=FAST,output=BINARY,csvEvery=0,dedicatedTasks=1,queueDepth=16,usbNonblocking=1,commit=<hash>
DBGADSREFPOLICY,mode=PIEZO_READ,...
DBGROUTEPOLICY,mode=PIEZO_READ,...
DBGTMUXPOLICY,mode=PIEZO_READ,...
STREAM_MEM,streamFrameSize=...,compactFrameSize=312,queueDepth=16,queueBytes=...,commStack=16384,scanStack=12288,heapFree=...,heapMinFree=...
VOLTSCAN_CONFIG,mode=PIEZO_READ,dr=15,format=binary,queueDepth=16,...
STREAM_INIT,format=binary,magic=0x31434153,magicBytes=SAC1,version=1,frameType=0x1261,frameSize=312,csvEvery=0,queueDepth=16,statusEvery=1000,textStatus=0
```

After `STREAM_INIT`, FAST/BINARY defaults to pure 312-byte `SAC1` compact
binary frames. A plain serial monitor may show binary noise; that is normal.
The host parser must resync on `magic=SAC1`, `frameSize=312`, and CRC32.

FAST/BINARY does not mix periodic ASCII `STAT` or `RATE_EVENT` lines into the
binary stream unless `CONFIG_SENSORARRAY_BINARY_TEXT_STATUS=y` is explicitly
enabled. For manual text diagnostics, use that option, select
`CONFIG_SENSORARRAY_OUTPUT_FORMAT_BOTH`, or switch to the SAFE/CSV profile.

In FAST/BINARY hot path you should not see continuous:

```text
MATV_HEADER
MATV,
MATV_RAW
MATV_GAIN
```

## Compact Binary Frame

FAST/MAX default output is `sensorarrayVoltageCompactFrame_t`:

```text
magic      = 0x31434153, bytes "SAC1"
version    = 1
frameType  = 0x1261
frameSize  = 312 bytes
crc32      = IEEE CRC32 over all bytes before crc32
```

Point order is unchanged from legacy CSV:

```text
microvolts[0]  = S1D1
microvolts[1]  = S1D2
...
microvolts[63] = S8D8
```

`validMask` uses the same index: bit `s * 8 + d`, where `s` and `d` are
zero-based row/column indices. Invalid points are reported with bit `0`; the
host should display those points as NaN.

The compact frame keeps passive drops and active decimation as separate
saturated 16-bit counters. Full cumulative values are available in `STAT`.
If GUI `crc_errors`, `resyncs`, or `parse_errors` climb continuously, first
check whether mixed text/binary output was enabled.

## Switching Back To Legacy CSV

Use this only for debug compatibility:

```bash
idf.py menuconfig
```

Select:

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

## If MATV Still Appears

Check `sdkconfig` and startup logs:

```text
CONFIG_SENSORARRAY_VOLTAGE_SCAN_PROFILE_SAFE=y
CONFIG_SENSORARRAY_OUTPUT_FORMAT_CSV=y
CONFIG_SENSORARRAY_VOLTAGE_SCAN_CSV_EVERY_N=1
```

The first decisive line is:

```text
BUILD_CONFIG,appMode=PIEZO_READ,profile=FAST,output=BINARY,csvEvery=0,...
```

If it does not show `profile=FAST,output=BINARY,csvEvery=0`, rebuild after
`idf.py fullclean` and confirm the new image was flashed. Also check the
`commit=` field in `BUILD_CONFIG`.

## Bottleneck Diagnosis

`STAT` separates scan, queue, USB stdout, and host-parser problems:

```text
STAT,seq=1000,fps=31,scanFps=31,outFps=31,scanAvgUs=31442,scanMaxUs=...,usbAvgUs=...,usbMaxUs=...,qDepth=16,qUsed=0,qFull=0,drop=0,decimated=0,shortWrite=0,writeFail=0,outputDiv=1,outStackMinWords=...,scanStackMinWords=...,heapFree=...,heapMinFree=...
```

Use the fields this way:

- `scanFps` near 31 and `outFps` near 31: firmware output path is keeping up.
- `scanFps` near 31 but `outFps` low: USB stdout or host reader is limiting.
- `qUsed`, `qFull`, or `drop` rising: output task cannot drain frames quickly enough.
- `decimated` rising or `outputDiv > 1`: rate control is intentionally skipping output frames.
- `shortWrite` or `writeFail` rising: nonblocking stdout could not accept full 312-byte frames.
- `scanAvgUs` high: ADS/TMUX scan timing, settle time, DRDY, or SPI is the bottleneck.

## Stack Overflow Checks

The FAST/MAX tasks are named `sensorarray_out` and `sensorarray_scan`.
If a panic reports `sensorarray_out stack overflow`, first confirm:

- `CONFIG_SENSORARRAY_COMM_TASK_STACK >= 16384`
- `CONFIG_SENSORARRAY_SCAN_TASK_STACK >= 12288`
- `STREAM_MEM` reports the expected stack sizes and `compactFrameSize=312`
- pure FAST/BINARY has `textStatus=0` and no periodic `STAT` after `STREAM_INIT`
- `outStackMinWords`, `scanStackMinWords`, `heapFree`, and `heapMinFree` in text
  debug mode still have margin

If a panic reports `sensorarray_scan stack overflow`, inspect scan-frame and ADS
hot-path changes first.

Periodic REF activity during this failure should be treated as a likely
secondary effect of MCU reboot. Once `sensorarray_out` no longer overflows and
the board stops resetting, re-check REF on the scope. If REF still toggles
without resets, debug the REF/SW/ADS reference policy separately.

Recent stream/output status codes:

```text
0x3006 STREAM_INTERNAL_ERROR
0x3007 OUT_STACK_LOW
0x3008 OUT_STACK_CRITICAL
0x3009 BINARY_TEXT_SUPPRESSED
0x300A BINARY_WRITE_PARTIAL
```

Future USB Vendor Bulk Transfer may still be worth testing, but only after
recording FAST/BINARY `scanFps`, `outFps`, queue usage, write failures, and host
parser FPS with this stdout path.

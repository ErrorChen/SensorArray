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
APPMODE,active=PIEZO_READ,...
BUILD_CONFIG,appMode=PIEZO_READ,profile=FAST,output=BINARY,csvEvery=0,dedicatedTasks=1,queueDepth=16,usbNonblocking=1,commit=<hash>
DBGADSREFPOLICY,mode=PIEZO_READ,...
DBGROUTEPOLICY,mode=PIEZO_READ,...
DBGTMUXPOLICY,mode=PIEZO_READ,...
VOLTSCAN_CONFIG,mode=PIEZO_READ,dr=15,format=binary,queueDepth=16,...
STREAM_INIT,format=binary,magic=0x31434153,magicBytes=SAC1,version=1,frameType=0x1261,frameSize=312,csvEvery=0,...
```

After scanning starts, a plain serial monitor may show binary noise. That is
normal. The host parser should resync on `SAC1`.

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
STAT,seq=100,fps=31,scanFps=31,outFps=31,scanAvgUs=31442,scanMaxUs=...,usbAvgUs=...,usbMaxUs=...,qDepth=16,qUsed=0,qFull=0,drop=0,decimated=0,shortWrite=0,writeFail=0,outputDiv=1,...
```

Use the fields this way:

- `scanFps` near 31 and `outFps` near 31: firmware output path is keeping up.
- `scanFps` near 31 but `outFps` low: USB stdout or host reader is limiting.
- `qUsed`, `qFull`, or `drop` rising: output task cannot drain frames quickly enough.
- `decimated` rising or `outputDiv > 1`: rate control is intentionally skipping output frames.
- `shortWrite` or `writeFail` rising: nonblocking stdout could not accept full 312-byte frames.
- `scanAvgUs` high: ADS/TMUX scan timing, settle time, DRDY, or SPI is the bottleneck.

Future USB Vendor Bulk Transfer may still be worth testing, but only after
recording FAST/BINARY `scanFps`, `outFps`, queue usage, write failures, and host
parser FPS with this stdout path.

# SensorArray Firmware (ESP-IDF, ESP32-S3)

SensorArray firmware targets ESP32-S3 and provides a modular stack for sensor
matrix scanning, mux control, ADC sampling, capacitance sensing, and transport
framing. The repository is organized as reusable ESP-IDF components so the
drivers can be integrated into multiple app_main variants.

## Sub-README Jump Table

| Area | Link | Notes |
| --- | --- | --- |
| ADS126x ADC driver | [components/ads126xAdc/README.md](components/ads126xAdc/README.md) | ADS1262/ADS1263 SPI driver and conversion helpers |
| FDC2214 capacitance | [components/fdc2214Cap/README.md](components/fdc2214Cap/README.md) | I2C driver, autoscan, 28-bit data |
| TMUX switch control | [components/tmuxSwitch/README.md](components/tmuxSwitch/README.md) | TMUX1108/TMUX1134 GPIO control and safe row switching |
| Matrix scan engine | [core/matrixEngine/README.md](core/matrixEngine/README.md) | Row/col selection + ADC/FDC2214 coordination |
| Wire protocol | [transport/protocolWire/README.md](transport/protocolWire/README.md) | Fixed 400-byte frame format |
| USB framing | [transport/protocolUsb/README.md](transport/protocolUsb/README.md) | Sync/length/CRC wrapper for USB-CDC |

## Table of Contents
- [Overview](#overview)
- [Current Status and Entry Point](#current-status-and-entry-point)
- [Repository Layout](#repository-layout)
- [Build and Flash](#build-and-flash)
- [Configuration (menuconfig)](#configuration-menuconfig)
- [Data Flow and Protocols](#data-flow-and-protocols)
- [Drivers and Components](#drivers-and-components)
- [Core Modules](#core-modules)
- [Transport Modules](#transport-modules)
- [Example App](#example-app)
- [Datasheets](#datasheets)

## Overview
The firmware is designed to scan a sensor matrix by selecting rows and column
groups using TMUX1108/TMUX1134, then measuring either voltage/resistance via
ADS126x or capacitance via FDC2214. Results are packed into a fixed binary
frame for streaming over BLE or wired links. The codebase is componentized so
the sensor stack can be reused by future app_main implementations.

## Current Status and Entry Point
- `example/example_ble_server_throughput.c` provides the active `app_main()`.
- `main/main.c` is a placeholder and intentionally does not define `app_main()`.
- Driver components are implemented; several higher-level modules are still
  placeholders or partially integrated.

## Repository Layout
- `components/`: Hardware drivers (TMUX1108/TMUX1134, ADS126x, FDC2214).
- `core/`: Board abstractions, power control, and the matrix scan engine.
- `transport/`: Protocol framing and transport-specific helpers.
- `example/`: BLE throughput example used as the current entry point.
- `main/`: Placeholder for a future application entry.
- `datasheets/`: Vendor datasheets and reference documents.

## Build and Flash
Typical ESP-IDF flow:
- `idf.py set-target esp32s3`
- `idf.py menuconfig`
- `idf.py build`
- `idf.py flash monitor`

## Configuration (menuconfig)
Key Kconfig groups and what they control:
- SensorArray Project: enable wired/BLE, select ADS126x variant, task core
  pinning, task stacks/priorities, SPI DMA, default matrix sizing.
- Board Support (Pins/Buses): I2C1/I2C2 ports and pins, SPI host and pins,
  ADS126x CS/DRDY/RESET GPIOs.
- Power Control: main/analog enable GPIO, charger status, and power-good pins.
- TMUX Switch: TMUX1108 A0/A1/A2/SW pins, safe row switching, TMUX1134 SEL/EN
  default behavior.
- ADS126x ADC: log level, SPI clock rate, ADC2 enable, helper to create SPI
  device handle.
- FDC2214Cap: enable/disable, log level, I2C address, channel count.
- Matrix Engine: rows/cols, frame period, oversample, ringbuffer depth, task
  cores/stacks/priorities.
- Wire Protocol: CRC16 enable and frame version.
- Wired Transport: UART enable/port/pins/baud, USB-CDC enable.
- BLE Transport: enable, MTU target, notify queue length.

## Data Flow and Protocols
Typical pipeline for matrix reads:
1) `matrixEngine` selects a row (TMUX1108) and a column group (TMUX1134).
2) Measurements are captured via ADS126x (voltage/resistance) or FDC2214
   (capacitance).
3) Samples are packed into a `protocolWire` frame (fixed 400 bytes).
4) For USB-CDC, `protocolUsb` adds sync + length + CRC to each frame.
5) Transport modules stream over BLE or wired links (in progress).

Protocol details:
- `protocolWire` uses a fixed layout: header fields, 64 offsets, 64 tagged data
  words, and an optional CRC16 (CCITT-FALSE).
- `protocolUsb` wraps payloads with a sync word, payload length, and CRC16 to
  recover framing on USB-CDC byte streams.

## Drivers and Components
TMUX switch (`components/tmuxSwitch`):
- Supports TMUX1108 (8:1 mux) and TMUX1134 (4x SPDT) via GPIO-only control.
- Optional safe row switching sequence to avoid transient connections.
- Maps row 0..7 to TMUX1108 S1..S8; SW selects REF vs GND with configurable
  polarity.

ADS126x ADC (`components/ads126xAdc`):
- Supports ADS1262 (ADC1) and ADS1263 (ADC1 + ADC2).
- SPI mode 1, CS controlled by SPI driver; DOUT/DRDY is shared.
- CRC and checksum handling are supported; ADC1 raw code to microvolts helper
  is provided.

FDC2214 capacitance (`components/fdc2214Cap`):
- 4-channel LC resonant capacitance-to-digital converter.
- Supports single-channel continuous conversions and autoscan sequences.
- Returns 28-bit samples with error flags; enforces MUX_CONFIG fixed bits.

## Core Modules
- `core/boardSupport`: board pin/bus mapping and init (placeholder).
- `core/powerCtrl`: power enable/PG/IRQ GPIO abstraction (placeholder).
- `core/matrixEngine`: coordinates row/col selection and measurement, supports
  rectangular region read/write with row-major output.

## Transport Modules
- `transport/protocolWire`: fixed-size binary frame for streaming.
- `transport/protocolUsb`: USB framing helper with sync/len/CRC16.
- `transport/transportWire`: wired streaming module (placeholder).
- `transport/bleTransport`: BLE streaming module (placeholder).

## Example App
The BLE throughput example is currently the only `app_main()`:
- `example/example_ble_server_throughput.c`

## Datasheets
Vendor references are kept in `datasheets/` for ADS126x, FDC2214, TMUX, and
other related parts.

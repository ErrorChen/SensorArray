# SensorArray Firmware (ESP-IDF, ESP32-S3)

## Overview
SensorArray firmware targets ESP32-S3 with a modular component layout for mux control, ADC sampling, and capacitance sensing. The current app_main entry point is still the BLE throughput example while the sensor array stack is built out.

## Status
- app_main is defined in `example/example_ble_server_throughput.c`.
- `main/main.c` and most components do not define app_main().
- Implemented drivers live in `components/ads126xAdc`, `components/fdc2214Cap`, and `components/tmuxSwitch`.

## Build
- `idf.py set-target esp32s3`
- `idf.py menuconfig`
- `idf.py build flash monitor`

## Configuration (menuconfig)
- SensorArray Project: enable wired/BLE, ADS126x variant, task core pinning, task stacks/priorities, SPI DMA, matrix defaults
- Board Support (Pins/Buses): I2C1/I2C2 ports/pins, SPI host/pins, ADS126x CS/DRDY/RESET pins
- Power Control: main/analog enable GPIO, charger status, power-good
- TMUX Switch: TMUX1108 A0/A1/A2/SW + safe row switching, TMUX1134 SEL/EN defaults
- ADS126x ADC: log level, SPI clock, ADC2 enable, helper to create SPI device
- FDC2214Cap: enable, log level, I2C address, channel count
- Matrix Engine: rows/cols, frame period, oversample, ringbuffer, task cores/stacks/priorities
- Wire Protocol: CRC16 enable, frame version
- Wired Transport: UART enable/port/pins/baud, USB-CDC enable
- BLE Transport: enable, MTU target, notify queue length

## Components
- tmuxSwitch: TMUX1108/TMUX1134 GPIO control and safe row switching
- ads126xAdc: ADS1262/ADS1263 SPI driver (ADC2 optional)
- fdc2214Cap: FDC2214 capacitance sensing over I2C

## Core
- boardSupport: board pin/bus mapping and init (placeholder)
- powerCtrl: power enable/pg/irq GPIO abstraction (placeholder)
- matrixEngine: scan scheduler, ringbuffer, multicore tasking (placeholder)

## Transport
- protocolWire: binary framing + CRC (placeholder)
- protocolUsb: USB sync/length/CRC framing helper
- transportWire: wired streaming (UART/USB-CDC) (placeholder)
- bleTransport: BLE streaming module (placeholder)

## Component docs
- `components/tmuxSwitch/README.md`
- `components/ads126xAdc/README.md`
- `components/fdc2214Cap/README.md`

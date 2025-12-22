# SensorArray Firmware (ESP-IDF, ESP32-S3)

## Overview
This project keeps the BLE throughput example as the current app_main entry while a modular sensor array architecture is built out in components.

## Build
- idf.py set-target esp32s3
- idf.py menuconfig
- idf.py build flash monitor

## Configuration (menuconfig)
- SensorArray Project: transport enable, ADS126x variant, task pinning, SPI DMA, matrix defaults
- Board Support (Pins/Buses): I2C1/I2C2 + SPI + ADS126x pins
- TMUX Control Pins: TMUX1108/TMUX1134 select lines
- Wired Transport: UART/USB-CDC options
- BLE Transport: MTU and notify queue

## Architecture (components)
- boardSupport: SPI/I2C init and pin mappings
- powerCtrl: power enable/pg/irq GPIO abstraction
- tmuxSwitch: TMUX1108/TMUX1134 selection control
- ads126xAdc: ADS1262/ADS1263 ADC driver (ADC2 optional)
- fdc2212Cap: FDC2212 capacitance sensing (I2C)
- matrixEngine: scan scheduler, ringbuffer, multicore tasking
- protocolWire: binary framing + CRC
- transportWire: wired streaming (UART/USB-CDC)
- bleTransport: future BLE streaming module

## Notes
- BLE throughput example remains the app_main entry for now.
- New modules are placeholders and do not define app_main().

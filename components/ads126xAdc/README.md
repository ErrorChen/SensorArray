# ads126xAdc (ADS1262/ADS1263 SPI driver)

## Overview
- Supports ADS1262 (ADC1 only) and ADS1263 (ADC1 + ADC2).
- Uses ESP-IDF SPI master driver only.
- ADC2 APIs return ESP_ERR_NOT_SUPPORTED on ADS1262 or when disabled in Kconfig.

## Hardware connections
- SPI: CS, SCLK, MOSI (DIN), MISO (DOUT/DRDY).
- DRDY: connect DOUT/DRDY to a GPIO so the driver can wait for DRDY=0 before reads.
- RESET/PWDN: optional; tie to GPIO for hardware reset or set GPIO_NUM_NC to disable.
- Internal reference: if INTREF is enabled, place a 1 uF (up to 10 uF) capacitor between REFOUT and AVSS.

## SPI configuration notes
- SPI mode 1 (CPOL=0, CPHA=1).
- Keep CS under SPI driver control; ADS126x resets the serial interface when CS goes high.
- If CS is tied low on the board, set `CONFIG_BOARD_ADS126X_CS_GPIO=-1` so the SPI driver does not drive CS.
- Do not sample DRDY during SPI clocking because DOUT/DRDY changes state during reads.

## Command and register summary (used by this driver)
- Commands: NOP, RESET, START1/STOP1, START2/STOP2, RDATA1/RDATA2, RREG/WREG, SYOCAL1, SYGCAL1, SFOCAL1.
- Registers: ID (0x00), POWER (0x01), INTERFACE (0x02), MODE2 (0x05), INPMUX (0x06), REFMUX (0x0F).

## API list
- ads126xAdcInit: create mutex, reset the device, read ID, and configure POWER/INTERFACE/MODE2.
- ads126xAdcDeinit: release driver resources (does not delete the SPI device).
- ads126xAdcHardwareReset: toggle RESET/PWDN if resetGpio is configured.
- ads126xAdcSendCommand: send a single-byte command.
- ads126xAdcReadRegisters / ads126xAdcWriteRegisters: RREG/WREG access with burst support.
- ads126xAdcGetIdRaw: read ID register (0x00).
- ads126xAdcConfigure: configure INTREF, STATUS byte, CRC/CHK, gain, and data rate.
- ads126xAdcSetRefMux: write REFMUX (0x0F).
- ads126xAdcSetInputMux: write INPMUX (0x06) with muxp/muxn nibbles.
- ads126xAdcStartAdc1 / ads126xAdcStopAdc1: control ADC1 conversion.
- ads126xAdcWaitDrdy: wait for DRDY to go low (or delay if DRDY not wired).
- ads126xAdcReadAdc1Raw: read one ADC1 sample and validate CRC/CHK if enabled.
- ads126xAdcRawToMicrovolts: convert ADC1 signed code to microvolts.
- ads126xAdcSelfOffsetCal: send SFOCAL1.
- ads126xAdcSelfGainCal: maps to SYGCAL1 (requires external full-scale input).
- ads126xAdcSystemOffsetCal: send SYOCAL1.
- ads126xAdcSystemGainCal: send SYGCAL1.
- ads126xAdcSelfCal: runs offset then gain (gain still requires full-scale input).
- ads126xAdcStartAdc2 / ads126xAdcStopAdc2 / ads126xAdcReadAdc2Raw: ADC2 access on ADS1263 only.

## Raw code to voltage conversion
ADC1 data are 32-bit two's complement. The datasheet ideal code table implies:

V = (code / 2^31) * (VREF / GAIN)

The driver uses vrefMicrovolts and pgaGain from the handle:

microvolts = rawCode * vrefMicrovolts / (pgaGain * 2^31)

This assumes a bipolar measurement range of +/- (VREF / GAIN).

## CRC and checksum
- Checksum mode: checksum = (sum(data bytes) + 0x9B) & 0xFF.
- CRC mode: CRC-8-ATM polynomial X^8 + X^2 + X + 1 (0x07), MSB first.
- CRC/CHK is computed over data bytes only (status byte is not included).

## Minimal example (Option A: external SPI device handle)
```c
#include "driver/spi_master.h"
#include "ads126xAdc.h"

static void init_ads126x(void)
{
    spi_bus_config_t busCfg = {
        .mosi_io_num = CONFIG_BOARD_SPI_MOSI_GPIO,
        .miso_io_num = CONFIG_BOARD_SPI_MISO_GPIO,
        .sclk_io_num = CONFIG_BOARD_SPI_SCLK_GPIO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = CONFIG_SENSORARRAY_SPI_MAX_TRANSFER_BYTES,
    };
    spi_device_interface_config_t devCfg = {
        .clock_speed_hz = CONFIG_ADS126X_SPI_CLOCK_HZ,
        .mode = 1,
        .spics_io_num = CONFIG_BOARD_ADS126X_CS_GPIO,
        .queue_size = 1,
    };

    spi_bus_initialize(CONFIG_BOARD_SPI_HOST, &busCfg,
                       CONFIG_SENSORARRAY_SPI_USE_DMA ? SPI_DMA_CH_AUTO : 0);

    spi_device_handle_t spiDev = NULL;
    spi_bus_add_device(CONFIG_BOARD_SPI_HOST, &devCfg, &spiDev);

    ads126xAdcConfig_t cfg = {
        .spiDevice = spiDev,
        .drdyGpio = CONFIG_BOARD_ADS126X_DRDY_GPIO,
        .resetGpio = CONFIG_BOARD_ADS126X_RESET_GPIO,
        .forcedType = ADS126X_DEVICE_AUTO,
        .crcMode = ADS126X_CRC_OFF,
        .enableStatusByte = false,
        .enableInternalRef = true,
        .vrefMicrovolts = 2500000,
        .pgaGain = 1,
        .dataRateDr = 4,
    };

    ads126xAdcHandle_t adc = {0};
    ads126xAdcInit(&adc, &cfg);

    ads126xAdcSetInputMux(&adc, 0x00, 0x01); /* AIN0/AIN1 */
    ads126xAdcStartAdc1(&adc);

    ads126xAdcWaitDrdy(&adc, 1000);
    int32_t raw = 0;
    uint8_t status = 0;
    ads126xAdcReadAdc1Raw(&adc, &raw, &status);

    int32_t uv = ads126xAdcRawToMicrovolts(&adc, raw);
    (void)uv;
}
```

## Minimal example (Option B: helper to create SPI device)
Enable CONFIG_ADS126X_HELPER_CREATE_SPI first.
```c
spi_device_handle_t spiDev = NULL;
ads126xAdcHelperCreateSpiDevice(&spiDev);
```
Note: the helper assumes the ADS126x owns the SPI bus. Do not use it on a shared bus.

## Common pitfalls
- DRDY/DOUT is shared: wait for DRDY low before sending RDATA1/RDATA2.
- If DRDY is not wired, ads126xAdcReadAdc1Raw/ReadAdc2Raw fall back to a delay based on drdyTimeoutMs.
- When INTREF is enabled, allow time for REFOUT to settle (50 ms typical with 1 uF).
- INPMUX defaults to AIN0/AIN1; always set the correct input pair.
- ADC2 APIs return ESP_ERR_NOT_SUPPORTED on ADS1262 hardware.

## SensorArray debug integration
`main/main.c` now includes dedicated ADS debug modes that use this driver to:
- Dump key registers (`ID/POWER/INTERFACE/MODE2/INPMUX/REFMUX`) before/after config.
- Force INPMUX (`muxp/muxn`) and raw REFMUX values.
- Run explicit read sequence variants (`STOP1 -> INPMUX -> delay -> START1 -> discard -> read`).
- Emit machine-readable logs with raw code, converted microvolts, mux pair, refmux, discard count,
  and DRDY-timeout information.

The normal route read path also exposes configurable sequencing policy in `menuconfig`:
- Optional `STOP1` before MUX change.
- Optional settle delay after MUX change.
- `START1` per-read or only when needed.
- Base discard count.
- Retry count on timeout/error.

# FDC2214Cap Component

## What it is
This component provides a handle-based driver for the TI FDC2214, a 4-channel LC resonant capacitance-to-digital converter. It supports single-channel continuous conversion and autoscan sequences, and returns 28-bit raw samples with error flags.

## Register highlights (for maintenance)
- 0x00..0x07: DATA_CH0..CH3 (MSB + status) and DATA_LSB_CHx
  - MSB bits: [13]=ERR_WD, [12]=ERR_AW, [11:0]=DATA[27:16]
  - LSB bits: [15:0]=DATA[15:0]
  - DATA_LSB must be read immediately after DATA_MSB to ensure a matching sample.
- 0x08..0x0B: RCOUNT_CH0..CH3
- 0x0C..0x0F: OFFSET_CH0..CH3
- 0x10..0x13: SETTLECOUNT_CH0..CH3
- 0x14..0x17: CLOCK_DIVIDERS_CH0..CH3 (FREF_DIVIDER must be non-zero)
- 0x18: STATUS
- 0x19: STATUS_CONFIG
- 0x1A: CONFIG (ACTIVE_CHAN, SLEEP_MODE)
- 0x1B: MUX_CONFIG (AUTOSCAN_EN, RR_SEQUENCE, DEGLITCH; bits12:3 fixed to 0x41)
- 0x1C: RESET_DEV (write bit15=1 to reset; read returns 0)
- 0x1E..0x21: DRIVE_CURRENT_CH0..CH3
- 0x7E: MANUFACTURER_ID (expected 0x5449)
- 0x7F: DEVICE_ID (expected 0x3055 for FDC2214)

## Quick start
1) Provide I2C callbacks (example uses ESP-IDF I2C master APIs):

```c
#include "driver/i2c.h"

typedef struct {
    i2c_port_t port;
} Fdc2214I2cCtx_t;

static esp_err_t Fdc2214WriteRead(void* userCtx, uint8_t addr7,
                                 const uint8_t* tx, size_t txLen,
                                 uint8_t* rx, size_t rxLen)
{
    Fdc2214I2cCtx_t* ctx = (Fdc2214I2cCtx_t*)userCtx;
    return i2c_master_write_read_device(ctx->port, addr7, tx, txLen, rx, rxLen, pdMS_TO_TICKS(100));
}

static esp_err_t Fdc2214Write(void* userCtx, uint8_t addr7,
                             const uint8_t* tx, size_t txLen)
{
    Fdc2214I2cCtx_t* ctx = (Fdc2214I2cCtx_t*)userCtx;
    return i2c_master_write_to_device(ctx->port, addr7, tx, txLen, pdMS_TO_TICKS(100));
}
```

2) Create device, optional reset, and check IDs:

```c
Fdc2214CapDevice_t* dev = NULL;
Fdc2214I2cCtx_t i2cCtx = { .port = I2C_NUM_0 };
Fdc2214CapBusConfig_t bus = {
    .I2cAddress7 = 0x2A,
    .UserCtx = &i2cCtx,
    .WriteRead = Fdc2214WriteRead,
    .Write = Fdc2214Write,
    .IntGpio = -1,
};

Fdc2214CapCreate(&bus, &dev);
Fdc2214CapReset(dev);
uint16_t mid = 0, did = 0;
Fdc2214CapReadId(dev, &mid, &did);
```

3) Configure channels and select mode:

```c
Fdc2214CapChannelConfig_t chCfg = {
    .Rcount = 0xFFFF,
    .SettleCount = 0x0400,
    .Offset = 0,
    .ClockDividers = 0x0401, // example: FIN_SEL + FREF_DIVIDER
    .DriveCurrent = 0x7C00,
};

Fdc2214CapConfigureChannel(dev, FDC2214_CH0, &chCfg);
Fdc2214CapSetSingleChannelMode(dev, FDC2214_CH0);
// or: Fdc2214CapSetAutoScanMode(dev, 2, FDC2214_DEGLITCH_10MHZ);
```

4) Read samples:

```c
Fdc2214CapSample_t sample = {0};
Fdc2214CapReadSample(dev, FDC2214_CH0, &sample);
```

## API overview
- Fdc2214CapCreate: Create a handle with I2C callbacks and address.
- Fdc2214CapDestroy: Free the handle and mutex.
- Fdc2214CapReset: Write RESET_DEV bit15=1.
- Fdc2214CapReadId: Read manufacturer and device IDs.
- Fdc2214CapConfigureChannel: Configure RCOUNT, SETTLECOUNT, OFFSET, CLOCK_DIVIDERS, DRIVE_CURRENT for a channel.
- Fdc2214CapSetSingleChannelMode: Single-channel continuous conversions via CONFIG.ACTIVE_CHAN.
- Fdc2214CapSetAutoScanMode: Autoscan with RR_SEQUENCE and deglitch filter.
- Fdc2214CapReadSample: Read a 28-bit conversion with ERR_WD/ERR_AW flags (flags clear on read).
- Fdc2214CapReadRawRegisters: Read a raw 16-bit register value.

## Common pitfalls
- DATA_LSB must be read immediately after DATA_MSB; this driver reads 4 bytes in one transaction.
- MUX_CONFIG bits12:3 must be fixed to 0x41; the driver enforces this on every write.
- CLOCK_DIVIDERS FREF_DIVIDER cannot be 0; the driver rejects it.
- Expected IDs: MANUFACTURER_ID=0x5449, DEVICE_ID=0x3055.

## SensorArray test app
`main/main.c` includes a bring-up test that initializes FDC2214, validates IDs,
reads a small set of samples across enabled channels, and logs raw results before idling.

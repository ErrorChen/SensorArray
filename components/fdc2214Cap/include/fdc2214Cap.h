#ifndef FDC2214CAP_H
#define FDC2214CAP_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct Fdc2214CapDevice Fdc2214CapDevice_t;

typedef enum {
    FDC2214_CH0 = 0,
    FDC2214_CH1 = 1,
    FDC2214_CH2 = 2,
    FDC2214_CH3 = 3,
} Fdc2214CapChannel_t;

typedef esp_err_t (*Fdc2214I2cWriteReadFn)(void* userCtx,
                                          uint8_t addr7,
                                          const uint8_t* tx,
                                          size_t txLen,
                                          uint8_t* rx,
                                          size_t rxLen);

typedef esp_err_t (*Fdc2214I2cWriteFn)(void* userCtx,
                                      uint8_t addr7,
                                      const uint8_t* tx,
                                      size_t txLen);

typedef struct {
    uint8_t I2cAddress7;
    void* UserCtx;
    Fdc2214I2cWriteReadFn WriteRead;
    Fdc2214I2cWriteFn Write;
    int IntGpio;
} Fdc2214CapBusConfig_t;

typedef enum {
    FDC2214_DEGLITCH_1MHZ = 0b001,
    FDC2214_DEGLITCH_3P3MHZ = 0b100,
    FDC2214_DEGLITCH_10MHZ = 0b101,
    FDC2214_DEGLITCH_33MHZ = 0b111,
} Fdc2214CapDeglitch_t;

typedef struct {
    uint16_t Rcount;
    uint16_t SettleCount;
    uint16_t Offset;
    uint16_t ClockDividers;
    uint16_t DriveCurrent;
} Fdc2214CapChannelConfig_t;

typedef struct {
    uint32_t Raw28;
    bool ErrWatchdog;
    bool ErrAmplitude;
} Fdc2214CapSample_t;

// Create a device handle; the I2C callbacks are used for all transactions.
esp_err_t Fdc2214CapCreate(const Fdc2214CapBusConfig_t* busConfig, Fdc2214CapDevice_t** outDev);
// Destroy the device handle and release the mutex.
esp_err_t Fdc2214CapDestroy(Fdc2214CapDevice_t* dev);

// Write RESET_DEV with bit15 set to trigger device reset.
esp_err_t Fdc2214CapReset(Fdc2214CapDevice_t* dev);
// Read MANUFACTURER_ID and DEVICE_ID registers.
esp_err_t Fdc2214CapReadId(Fdc2214CapDevice_t* dev, uint16_t* manufacturerId, uint16_t* deviceId);

// Configure one channel (RCOUNT, SETTLECOUNT, OFFSET, CLOCK_DIVIDERS, DRIVE_CURRENT).
esp_err_t Fdc2214CapConfigureChannel(Fdc2214CapDevice_t* dev,
                                     Fdc2214CapChannel_t ch,
                                     const Fdc2214CapChannelConfig_t* cfg);

// Single channel continuous conversion; CONFIG.ACTIVE_CHAN selects channel.
esp_err_t Fdc2214CapSetSingleChannelMode(Fdc2214CapDevice_t* dev, Fdc2214CapChannel_t activeCh);
// Autoscan conversion over CH0..CHn; rrSequence maps 0->CH0-CH1, 1->CH0-CH2, 2->CH0-CH3.
esp_err_t Fdc2214CapSetAutoScanMode(Fdc2214CapDevice_t* dev, uint8_t rrSequence, Fdc2214CapDeglitch_t deglitch);

// Read one 28-bit sample; ErrWatchdog/ErrAmplitude come from MSB bits and clear on read.
esp_err_t Fdc2214CapReadSample(Fdc2214CapDevice_t* dev, Fdc2214CapChannel_t ch, Fdc2214CapSample_t* outSample);

// Read a raw 16-bit register value.
esp_err_t Fdc2214CapReadRawRegisters(Fdc2214CapDevice_t* dev, uint8_t reg, uint16_t* outValue);

#ifdef __cplusplus
}
#endif

#endif // FDC2214CAP_H

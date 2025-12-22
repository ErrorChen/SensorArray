#include "fdc2214Cap.h"

#include <stdlib.h>
#include <string.h>

#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#ifndef CONFIG_FDC2214CAP_LOG_LEVEL
#define CONFIG_FDC2214CAP_LOG_LEVEL 3
#endif
#ifndef LOG_LOCAL_LEVEL
#define LOG_LOCAL_LEVEL CONFIG_FDC2214CAP_LOG_LEVEL
#endif
#include "esp_log.h"

static const char* TAG = "fdc2214Cap";

// Register map notes (FDC2214):
// 0x00..0x07: DATA_CH0..CH3 (MSB + status) and DATA_LSB_CHx
//   MSB bits: [13]=ERR_WD, [12]=ERR_AW, [11:0]=DATA[27:16]
//   LSB bits: [15:0]=DATA[15:0]
//   DATA_LSB must be read immediately after DATA_MSB to ensure one conversion pair.
// 0x08..0x0B: RCOUNT_CH0..CH3
// 0x0C..0x0F: OFFSET_CH0..CH3
// 0x10..0x13: SETTLECOUNT_CH0..CH3
// 0x14..0x17: CLOCK_DIVIDERS_CH0..CH3
// 0x18: STATUS
// 0x19: STATUS_CONFIG
// 0x1A: CONFIG (ACTIVE_CHAN, SLEEP_MODE)
// 0x1B: MUX_CONFIG (AUTOSCAN_EN, RR_SEQUENCE, DEGLITCH; bits12:3 fixed to 0x41)
// 0x1C: RESET_DEV (write bit15=1 to reset, read returns 0)
// 0x1E..0x21: DRIVE_CURRENT_CH0..CH3
// 0x7E: MANUFACTURER_ID (0x5449)
// 0x7F: DEVICE_ID (0x3055 for FDC2214)

#define FDC2214_REG_DATA_MSB_BASE 0x00
#define FDC2214_REG_DATA_LSB_BASE 0x01
#define FDC2214_REG_RCOUNT_BASE 0x08
#define FDC2214_REG_OFFSET_BASE 0x0C
#define FDC2214_REG_SETTLECOUNT_BASE 0x10
#define FDC2214_REG_CLOCK_DIVIDERS_BASE 0x14
#define FDC2214_REG_STATUS 0x18
#define FDC2214_REG_STATUS_CONFIG 0x19
#define FDC2214_REG_CONFIG 0x1A
#define FDC2214_REG_MUX_CONFIG 0x1B
#define FDC2214_REG_RESET_DEV 0x1C
#define FDC2214_REG_DRIVE_CURRENT_BASE 0x1E
#define FDC2214_REG_MANUFACTURER_ID 0x7E
#define FDC2214_REG_DEVICE_ID 0x7F

#define FDC2214_DATA_MSB_MASK 0x0FFF
#define FDC2214_DATA_ERR_WD_MASK (1U << 13)
#define FDC2214_DATA_ERR_AW_MASK (1U << 12)

#define FDC2214_CONFIG_ACTIVE_CHAN_SHIFT 14
#define FDC2214_CONFIG_ACTIVE_CHAN_MASK (0x3U << FDC2214_CONFIG_ACTIVE_CHAN_SHIFT)

#define FDC2214_MUX_AUTOSCAN_BIT (1U << 15)
#define FDC2214_MUX_RR_SEQUENCE_SHIFT 13
#define FDC2214_MUX_RR_SEQUENCE_MASK (0x3U << FDC2214_MUX_RR_SEQUENCE_SHIFT)
#define FDC2214_MUX_FIXED_BITS 0x41
#define FDC2214_MUX_FIXED_MASK (FDC2214_MUX_FIXED_BITS << 3)
#define FDC2214_MUX_DEGLITCH_MASK 0x7U

#define FDC2214_CLOCK_FREF_DIVIDER_MASK 0x03FF
#define FDC2214_CLOCK_RESERVED_MASK 0xCC00

#define FDC2214_RESET_DEV_BIT (1U << 15)

// Drive current register uses a 5-bit field; reserved bits are forced to 0.
#define FDC2214_DRIVE_CURRENT_MASK 0x07C0

typedef struct Fdc2214CapDevice {
    Fdc2214CapBusConfig_t bus;
    SemaphoreHandle_t mutex;
} Fdc2214CapDevice_t;

static bool Fdc2214IsValidChannel(Fdc2214CapChannel_t ch)
{
    return (ch >= FDC2214_CH0) && (ch <= FDC2214_CH3);
}

static uint8_t Fdc2214RegForChannelStep1(uint8_t base, Fdc2214CapChannel_t ch)
{
    return (uint8_t)(base + (uint8_t)ch);
}

static uint8_t Fdc2214RegForChannelStep2(uint8_t base, Fdc2214CapChannel_t ch)
{
    return (uint8_t)(base + (uint8_t)ch * 2U);
}

static esp_err_t Fdc2214CapLock(Fdc2214CapDevice_t* dev)
{
    if (!dev || !dev->mutex) {
        return ESP_ERR_INVALID_STATE;
    }
    if (xSemaphoreTake(dev->mutex, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

static void Fdc2214CapUnlock(Fdc2214CapDevice_t* dev)
{
    if (dev && dev->mutex) {
        xSemaphoreGive(dev->mutex);
    }
}

static esp_err_t Fdc2214CapWriteBytes(Fdc2214CapDevice_t* dev, const uint8_t* tx, size_t txLen)
{
    if (!dev || !tx || txLen == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!dev->bus.Write) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = Fdc2214CapLock(dev);
    if (err != ESP_OK) {
        return err;
    }
    err = dev->bus.Write(dev->bus.UserCtx, dev->bus.I2cAddress7, tx, txLen);
    Fdc2214CapUnlock(dev);

    return err;
}

static esp_err_t Fdc2214CapWriteReadBytes(Fdc2214CapDevice_t* dev,
                                          const uint8_t* tx,
                                          size_t txLen,
                                          uint8_t* rx,
                                          size_t rxLen)
{
    if (!dev || !tx || txLen == 0 || !rx || rxLen == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!dev->bus.WriteRead) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = Fdc2214CapLock(dev);
    if (err != ESP_OK) {
        return err;
    }
    err = dev->bus.WriteRead(dev->bus.UserCtx, dev->bus.I2cAddress7, tx, txLen, rx, rxLen);
    Fdc2214CapUnlock(dev);

    return err;
}

static esp_err_t Fdc2214CapWriteReg16(Fdc2214CapDevice_t* dev, uint8_t reg, uint16_t value)
{
    uint8_t tx[3] = { reg, (uint8_t)(value >> 8), (uint8_t)(value & 0xFF) };
    esp_err_t err = Fdc2214CapWriteBytes(dev, tx, sizeof(tx));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Write reg 0x%02X failed: %d", reg, err);
        return err;
    }
    ESP_LOGD(TAG, "Write reg 0x%02X = 0x%04X", reg, value);
    return ESP_OK;
}

static esp_err_t Fdc2214CapReadReg16(Fdc2214CapDevice_t* dev, uint8_t reg, uint16_t* outValue)
{
    if (!outValue) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t tx = reg;
    uint8_t rx[2] = {0};
    esp_err_t err = Fdc2214CapWriteReadBytes(dev, &tx, sizeof(tx), rx, sizeof(rx));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Read reg 0x%02X failed: %d", reg, err);
        return err;
    }

    *outValue = ((uint16_t)rx[0] << 8) | rx[1];
    ESP_LOGD(TAG, "Read reg 0x%02X = 0x%04X", reg, *outValue);
    return ESP_OK;
}

static esp_err_t Fdc2214CapReadBytes(Fdc2214CapDevice_t* dev, uint8_t reg, uint8_t* rx, size_t rxLen)
{
    uint8_t tx = reg;
    esp_err_t err = Fdc2214CapWriteReadBytes(dev, &tx, sizeof(tx), rx, rxLen);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Read bytes from reg 0x%02X failed: %d", reg, err);
        return err;
    }
    return ESP_OK;
}

esp_err_t Fdc2214CapCreate(const Fdc2214CapBusConfig_t* busConfig, Fdc2214CapDevice_t** outDev)
{
    if (!busConfig || !outDev) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!busConfig->WriteRead || !busConfig->Write) {
        return ESP_ERR_INVALID_ARG;
    }

    Fdc2214CapDevice_t* dev = (Fdc2214CapDevice_t*)calloc(1, sizeof(Fdc2214CapDevice_t));
    if (!dev) {
        return ESP_ERR_NO_MEM;
    }

    dev->bus = *busConfig;
    dev->mutex = xSemaphoreCreateMutex();
    if (!dev->mutex) {
        free(dev);
        return ESP_ERR_NO_MEM;
    }

    *outDev = dev;
    ESP_LOGI(TAG, "Created device at I2C address 0x%02X", dev->bus.I2cAddress7);
    return ESP_OK;
}

esp_err_t Fdc2214CapDestroy(Fdc2214CapDevice_t* dev)
{
    if (!dev) {
        return ESP_ERR_INVALID_ARG;
    }
    if (dev->mutex) {
        vSemaphoreDelete(dev->mutex);
        dev->mutex = NULL;
    }
    free(dev);
    return ESP_OK;
}

esp_err_t Fdc2214CapReset(Fdc2214CapDevice_t* dev)
{
    if (!dev) {
        return ESP_ERR_INVALID_ARG;
    }
    ESP_LOGI(TAG, "Reset device");
    return Fdc2214CapWriteReg16(dev, FDC2214_REG_RESET_DEV, FDC2214_RESET_DEV_BIT);
}

esp_err_t Fdc2214CapReadId(Fdc2214CapDevice_t* dev, uint16_t* manufacturerId, uint16_t* deviceId)
{
    if (!dev || !manufacturerId || !deviceId) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = Fdc2214CapReadReg16(dev, FDC2214_REG_MANUFACTURER_ID, manufacturerId);
    if (err != ESP_OK) {
        return err;
    }
    err = Fdc2214CapReadReg16(dev, FDC2214_REG_DEVICE_ID, deviceId);
    if (err != ESP_OK) {
        return err;
    }

    ESP_LOGI(TAG, "IDs: manufacturer=0x%04X device=0x%04X", *manufacturerId, *deviceId);
    return ESP_OK;
}

esp_err_t Fdc2214CapConfigureChannel(Fdc2214CapDevice_t* dev,
                                    Fdc2214CapChannel_t ch,
                                    const Fdc2214CapChannelConfig_t* cfg)
{
    if (!dev || !cfg) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!Fdc2214IsValidChannel(ch)) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = Fdc2214CapWriteReg16(dev, Fdc2214RegForChannelStep1(FDC2214_REG_RCOUNT_BASE, ch), cfg->Rcount);
    if (err != ESP_OK) {
        return err;
    }
    err = Fdc2214CapWriteReg16(dev, Fdc2214RegForChannelStep1(FDC2214_REG_SETTLECOUNT_BASE, ch), cfg->SettleCount);
    if (err != ESP_OK) {
        return err;
    }
    err = Fdc2214CapWriteReg16(dev, Fdc2214RegForChannelStep1(FDC2214_REG_OFFSET_BASE, ch), cfg->Offset);
    if (err != ESP_OK) {
        return err;
    }

    uint16_t clockDividers = cfg->ClockDividers;
    if ((clockDividers & FDC2214_CLOCK_FREF_DIVIDER_MASK) == 0) {
        ESP_LOGE(TAG, "Clock divider FREF_DIVIDER must be non-zero");
        return ESP_ERR_INVALID_ARG;
    }
    clockDividers &= (uint16_t)(~FDC2214_CLOCK_RESERVED_MASK);
    err = Fdc2214CapWriteReg16(dev, Fdc2214RegForChannelStep1(FDC2214_REG_CLOCK_DIVIDERS_BASE, ch), clockDividers);
    if (err != ESP_OK) {
        return err;
    }

    uint16_t driveCurrent = (uint16_t)(cfg->DriveCurrent & FDC2214_DRIVE_CURRENT_MASK);
    if (driveCurrent != cfg->DriveCurrent) {
        ESP_LOGW(TAG, "Drive current masked to 0x%04X to clear reserved bits", driveCurrent);
    }
    err = Fdc2214CapWriteReg16(dev, Fdc2214RegForChannelStep1(FDC2214_REG_DRIVE_CURRENT_BASE, ch), driveCurrent);
    if (err != ESP_OK) {
        return err;
    }

    ESP_LOGI(TAG, "Configured CH%d", (int)ch);
    return ESP_OK;
}

esp_err_t Fdc2214CapSetSingleChannelMode(Fdc2214CapDevice_t* dev, Fdc2214CapChannel_t activeCh)
{
    if (!dev) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!Fdc2214IsValidChannel(activeCh)) {
        return ESP_ERR_INVALID_ARG;
    }

    uint16_t configReg = 0;
    esp_err_t err = Fdc2214CapReadReg16(dev, FDC2214_REG_CONFIG, &configReg);
    if (err != ESP_OK) {
        return err;
    }
    configReg &= (uint16_t)(~FDC2214_CONFIG_ACTIVE_CHAN_MASK);
    configReg |= (uint16_t)((uint16_t)activeCh << FDC2214_CONFIG_ACTIVE_CHAN_SHIFT);
    err = Fdc2214CapWriteReg16(dev, FDC2214_REG_CONFIG, configReg);
    if (err != ESP_OK) {
        return err;
    }

    uint16_t muxReg = 0;
    err = Fdc2214CapReadReg16(dev, FDC2214_REG_MUX_CONFIG, &muxReg);
    if (err != ESP_OK) {
        return err;
    }
    uint16_t deglitch = (uint16_t)(muxReg & FDC2214_MUX_DEGLITCH_MASK);
    uint16_t rrSequence = (uint16_t)((muxReg & FDC2214_MUX_RR_SEQUENCE_MASK) >> FDC2214_MUX_RR_SEQUENCE_SHIFT);

    uint16_t newMux = (uint16_t)(rrSequence << FDC2214_MUX_RR_SEQUENCE_SHIFT);
    newMux |= FDC2214_MUX_FIXED_MASK;
    newMux |= deglitch;
    err = Fdc2214CapWriteReg16(dev, FDC2214_REG_MUX_CONFIG, newMux);
    if (err != ESP_OK) {
        return err;
    }

    ESP_LOGI(TAG, "Single-channel mode on CH%d", (int)activeCh);
    return ESP_OK;
}

esp_err_t Fdc2214CapSetAutoScanMode(Fdc2214CapDevice_t* dev, uint8_t rrSequence, Fdc2214CapDeglitch_t deglitch)
{
    if (!dev) {
        return ESP_ERR_INVALID_ARG;
    }
    if (rrSequence > 2) {
        return ESP_ERR_INVALID_ARG;
    }

    uint16_t muxReg = FDC2214_MUX_AUTOSCAN_BIT;
    muxReg |= (uint16_t)((uint16_t)rrSequence << FDC2214_MUX_RR_SEQUENCE_SHIFT);
    muxReg |= FDC2214_MUX_FIXED_MASK;
    muxReg |= (uint16_t)(deglitch & FDC2214_MUX_DEGLITCH_MASK);

    esp_err_t err = Fdc2214CapWriteReg16(dev, FDC2214_REG_MUX_CONFIG, muxReg);
    if (err != ESP_OK) {
        return err;
    }

    ESP_LOGI(TAG, "Autoscan mode set, rrSequence=%u", (unsigned)rrSequence);
    return ESP_OK;
}

esp_err_t Fdc2214CapReadSample(Fdc2214CapDevice_t* dev, Fdc2214CapChannel_t ch, Fdc2214CapSample_t* outSample)
{
    if (!dev || !outSample) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!Fdc2214IsValidChannel(ch)) {
        return ESP_ERR_INVALID_ARG;
    }

    // Read MSB then LSB in one I2C transaction to keep a consistent sample.
    uint8_t rx[4] = {0};
    uint8_t reg = Fdc2214RegForChannelStep2(FDC2214_REG_DATA_MSB_BASE, ch);
    esp_err_t err = Fdc2214CapReadBytes(dev, reg, rx, sizeof(rx));
    if (err != ESP_OK) {
        return err;
    }

    uint16_t msb = ((uint16_t)rx[0] << 8) | rx[1];
    uint16_t lsb = ((uint16_t)rx[2] << 8) | rx[3];

    outSample->Raw28 = ((uint32_t)(msb & FDC2214_DATA_MSB_MASK) << 16) | lsb;
    outSample->ErrWatchdog = (msb & FDC2214_DATA_ERR_WD_MASK) != 0;
    outSample->ErrAmplitude = (msb & FDC2214_DATA_ERR_AW_MASK) != 0;

    // ERR_WD / ERR_AW bits are cleared by reading the MSB register.
    return ESP_OK;
}

esp_err_t Fdc2214CapReadRawRegisters(Fdc2214CapDevice_t* dev, uint8_t reg, uint16_t* outValue)
{
    if (!dev || !outValue) {
        return ESP_ERR_INVALID_ARG;
    }
    return Fdc2214CapReadReg16(dev, reg, outValue);
}

#include "fdc2214Cap.h"

#include <stdlib.h>
#include <string.h>

#include "sdkconfig.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#ifndef CONFIG_FDC2214CAP_LOG_LEVEL
#define CONFIG_FDC2214CAP_LOG_LEVEL 3
#endif
#ifndef CONFIG_FDC2214CAP_MUTEX_TIMEOUT_MS
#define CONFIG_FDC2214CAP_MUTEX_TIMEOUT_MS 200
#endif
#ifndef LOG_LOCAL_LEVEL
#define LOG_LOCAL_LEVEL CONFIG_FDC2214CAP_LOG_LEVEL
#endif
#include "esp_log.h"

static const char* TAG = "fdc2214Cap";

// Register map notes (FDC2214):
// 0x00..0x07: DATA_CH0..CH3 (MSB + status) and DATA_LSB_CHx
// 0x08..0x0B: RCOUNT_CH0..CH3
// 0x0C..0x0F: OFFSET_CH0..CH3
// 0x10..0x13: SETTLECOUNT_CH0..CH3
// 0x14..0x17: CLOCK_DIVIDERS_CH0..CH3
// 0x18: STATUS
// 0x19: STATUS_CONFIG (ERROR_CONFIG)
// 0x1A: CONFIG
// 0x1B: MUX_CONFIG
// 0x1C: RESET_DEV (write bit15=1 to reset)
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

#define FDC2214_STATUS_ERR_CHAN_SHIFT 14
#define FDC2214_STATUS_ERR_CHAN_MASK (0x3U << FDC2214_STATUS_ERR_CHAN_SHIFT)
#define FDC2214_STATUS_ERR_WD_MASK (1U << 11)
#define FDC2214_STATUS_ERR_AHW_MASK (1U << 10)
#define FDC2214_STATUS_ERR_ALW_MASK (1U << 9)
#define FDC2214_STATUS_DRDY_MASK (1U << 6)
#define FDC2214_STATUS_UNREAD_CH0_MASK (1U << 3)
#define FDC2214_STATUS_UNREAD_CH1_MASK (1U << 2)
#define FDC2214_STATUS_UNREAD_CH2_MASK (1U << 1)
#define FDC2214_STATUS_UNREAD_CH3_MASK (1U << 0)

#define FDC2214_CONFIG_ACTIVE_CHAN_SHIFT 14
#define FDC2214_CONFIG_ACTIVE_CHAN_MASK (0x3U << FDC2214_CONFIG_ACTIVE_CHAN_SHIFT)
#define FDC2214_CONFIG_SLEEP_MODE_EN_MASK (1U << 13)
#define FDC2214_CONFIG_RESERVED_BIT12_MASK (1U << 12)
#define FDC2214_CONFIG_SENSOR_ACTIVATE_SEL_MASK (1U << 11)
#define FDC2214_CONFIG_RESERVED_BIT10_MASK (1U << 10)
#define FDC2214_CONFIG_REF_CLK_SRC_MASK (1U << 9)
#define FDC2214_CONFIG_RESERVED_BIT8_MASK (1U << 8)
#define FDC2214_CONFIG_INTB_DIS_MASK (1U << 7)
#define FDC2214_CONFIG_HIGH_CURRENT_DRV_MASK (1U << 6)
#define FDC2214_CONFIG_RESERVED_LOW_MASK 0x003F
#define FDC2214_CONFIG_RESERVED_LOW_REQUIRED 0x0001

#define FDC2214_CONFIG_REQUIRED_SET_MASK \
    (FDC2214_CONFIG_RESERVED_BIT12_MASK | FDC2214_CONFIG_RESERVED_BIT10_MASK | FDC2214_CONFIG_RESERVED_LOW_REQUIRED)

#define FDC2214_MUX_AUTOSCAN_BIT (1U << 15)
#define FDC2214_MUX_RR_SEQUENCE_SHIFT 13
#define FDC2214_MUX_RR_SEQUENCE_MASK (0x3U << FDC2214_MUX_RR_SEQUENCE_SHIFT)
#define FDC2214_MUX_FIXED_BITS 0x41
#define FDC2214_MUX_FIXED_MASK (FDC2214_MUX_FIXED_BITS << 3)
#define FDC2214_MUX_DEGLITCH_MASK 0x7U

#define FDC2214_CLOCK_RESERVED_MASK 0xCC00
#define FDC2214_CLOCK_FIN_SEL_SHIFT 12
#define FDC2214_CLOCK_FIN_SEL_MASK (0x3U << FDC2214_CLOCK_FIN_SEL_SHIFT)
#define FDC2214_CLOCK_FREF_DIVIDER_MASK 0x03FF

#define FDC2214_STATUS_CONFIG_ALLOWED_MASK 0x3821

#define FDC2214_RESET_DEV_BIT (1U << 15)
#define FDC2214_RAW_SCALE_2P28 268435456.0

// Drive current register uses CHx_IDRIVE [15:11]; reserved bits must stay clear.
#define FDC2214_DRIVE_CURRENT_MASK FDC2214_CAP_DRIVE_CURRENT_MASK

// Datasheet timing (FDC221x): sleep-to-active wake-up time typ/min requirement.
#define FDC2214_SLEEP_WAKEUP_US 1000U
#ifdef CONFIG_SENSORARRAY_DEBUG_CAP_FDC_SECONDARY_POST_RESET_DELAY_MS
#define FDC2214_RESET_SETTLE_US ((uint32_t)CONFIG_SENSORARRAY_DEBUG_CAP_FDC_SECONDARY_POST_RESET_DELAY_MS * 1000U)
#else
#define FDC2214_RESET_SETTLE_US 5000U
#endif

typedef struct Fdc2214CapDevice {
    Fdc2214CapBusConfig_t bus;
    SemaphoreHandle_t mutex;
    Fdc2214CapTxnDiag_t txnDiag;
} Fdc2214CapDevice_t;

static bool Fdc2214IsValidChannel(Fdc2214CapChannel_t ch)
{
    return (ch >= FDC2214_CH0) && (ch <= FDC2214_CH3);
}

static bool Fdc2214IsValidDeglitch(Fdc2214CapDeglitch_t deglitch)
{
    switch (deglitch) {
    case FDC2214_DEGLITCH_1MHZ:
    case FDC2214_DEGLITCH_3P3MHZ:
    case FDC2214_DEGLITCH_10MHZ:
    case FDC2214_DEGLITCH_33MHZ:
        return true;
    default:
        return false;
    }
}

static bool Fdc2214IsValidRefClock(Fdc2214CapRefClockSource_t refClockSource)
{
    return (refClockSource == FDC2214_REF_CLOCK_INTERNAL) || (refClockSource == FDC2214_REF_CLOCK_EXTERNAL);
}

static uint8_t Fdc2214RegForChannelStep1(uint8_t base, Fdc2214CapChannel_t ch)
{
    return (uint8_t)(base + (uint8_t)ch);
}

static uint8_t Fdc2214RegForChannelStep2(uint8_t base, Fdc2214CapChannel_t ch)
{
    return (uint8_t)(base + (uint8_t)ch * 2U);
}

static void Fdc2214CapDelayUs(uint32_t delayUs)
{
    if (delayUs > 0U) {
        esp_rom_delay_us(delayUs);
    }
}

static uint16_t Fdc2214CapApplyConfigReservedBits(uint16_t configValue)
{
    // CONFIG requires fixed reserved values:
    // bit12=1, bit10=1, bit8=0, bits[5:0]=0b000001.
    configValue |= FDC2214_CONFIG_REQUIRED_SET_MASK;
    configValue &= (uint16_t)~FDC2214_CONFIG_RESERVED_BIT8_MASK;
    configValue &= (uint16_t)~(FDC2214_CONFIG_RESERVED_LOW_MASK & (uint16_t)~FDC2214_CONFIG_RESERVED_LOW_REQUIRED);
    return configValue;
}

static bool Fdc2214CapConfigReservedBitsValid(uint16_t configValue)
{
    if ((configValue & FDC2214_CONFIG_RESERVED_BIT12_MASK) == 0U) {
        return false;
    }
    if ((configValue & FDC2214_CONFIG_RESERVED_BIT10_MASK) == 0U) {
        return false;
    }
    if ((configValue & FDC2214_CONFIG_RESERVED_BIT8_MASK) != 0U) {
        return false;
    }
    return (configValue & FDC2214_CONFIG_RESERVED_LOW_MASK) == FDC2214_CONFIG_RESERVED_LOW_REQUIRED;
}

static Fdc2214CapChannel_t Fdc2214CapActiveChannelFromConfig(uint16_t configValue)
{
    uint8_t active = (uint8_t)((configValue & FDC2214_CONFIG_ACTIVE_CHAN_MASK) >> FDC2214_CONFIG_ACTIVE_CHAN_SHIFT);
    if (active > (uint8_t)FDC2214_CH3) {
        return FDC2214_CH0;
    }
    return (Fdc2214CapChannel_t)active;
}

static Fdc2214CapRefClockSource_t Fdc2214CapRefClockFromConfig(uint16_t configValue)
{
    return ((configValue & FDC2214_CONFIG_REF_CLK_SRC_MASK) != 0U) ? FDC2214_REF_CLOCK_EXTERNAL
                                                                    : FDC2214_REF_CLOCK_INTERNAL;
}

static Fdc2214CapConfigOptions_t Fdc2214CapConfigOptionsFromRaw(uint16_t configValue)
{
    return (Fdc2214CapConfigOptions_t){
        .ActiveChannel = Fdc2214CapActiveChannelFromConfig(configValue),
        .SleepModeEnabled = (configValue & FDC2214_CONFIG_SLEEP_MODE_EN_MASK) != 0U,
        .SensorActivateSelLowPower = (configValue & FDC2214_CONFIG_SENSOR_ACTIVATE_SEL_MASK) != 0U,
        .RefClockSource = Fdc2214CapRefClockFromConfig(configValue),
        .IntbDisabled = (configValue & FDC2214_CONFIG_INTB_DIS_MASK) != 0U,
        .HighCurrentDrive = (configValue & FDC2214_CONFIG_HIGH_CURRENT_DRV_MASK) != 0U,
    };
}

static void Fdc2214CapDecodeStatusRaw(uint16_t raw, Fdc2214CapStatus_t* outStatus)
{
    if (!outStatus) {
        return;
    }

    *outStatus = (Fdc2214CapStatus_t){
        .Raw = raw,
        .ErrorChannel = (uint8_t)((raw & FDC2214_STATUS_ERR_CHAN_MASK) >> FDC2214_STATUS_ERR_CHAN_SHIFT),
        .ErrWatchdog = (raw & FDC2214_STATUS_ERR_WD_MASK) != 0U,
        .ErrAmplitudeHigh = (raw & FDC2214_STATUS_ERR_AHW_MASK) != 0U,
        .ErrAmplitudeLow = (raw & FDC2214_STATUS_ERR_ALW_MASK) != 0U,
        .DataReady = (raw & FDC2214_STATUS_DRDY_MASK) != 0U,
        .UnreadConversion =
            {
                (raw & FDC2214_STATUS_UNREAD_CH0_MASK) != 0U,
                (raw & FDC2214_STATUS_UNREAD_CH1_MASK) != 0U,
                (raw & FDC2214_STATUS_UNREAD_CH2_MASK) != 0U,
                (raw & FDC2214_STATUS_UNREAD_CH3_MASK) != 0U,
            },
    };
}

static esp_err_t Fdc2214CapBuildMuxValue(bool autoScan,
                                         uint8_t rrSequence,
                                         Fdc2214CapDeglitch_t deglitch,
                                         uint16_t* outMuxValue)
{
    if (!outMuxValue) {
        return ESP_ERR_INVALID_ARG;
    }
    if (rrSequence > 2U) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!Fdc2214IsValidDeglitch(deglitch)) {
        return ESP_ERR_INVALID_ARG;
    }

    uint16_t muxValue = 0U;
    if (autoScan) {
        muxValue |= FDC2214_MUX_AUTOSCAN_BIT;
    }
    muxValue |= (uint16_t)((uint16_t)rrSequence << FDC2214_MUX_RR_SEQUENCE_SHIFT);
    muxValue |= FDC2214_MUX_FIXED_MASK;
    muxValue |= (uint16_t)deglitch;
    *outMuxValue = muxValue;
    return ESP_OK;
}

static esp_err_t Fdc2214CapNormalizeClockDividers(uint16_t rawClockDividers, uint16_t* outClockDividers)
{
    if (!outClockDividers) {
        return ESP_ERR_INVALID_ARG;
    }

    Fdc2214CapClockDividerInfo_t info = {0};
    esp_err_t err = Fdc2214CapDecodeClockDividers(rawClockDividers, &info);
    if (err != ESP_OK) {
        return err;
    }

    *outClockDividers = info.NormalizedClockDividers;
    return ESP_OK;
}

esp_err_t Fdc2214CapDecodeClockDividers(uint16_t rawClockDividers, Fdc2214CapClockDividerInfo_t* outInfo)
{
    if (!outInfo) {
        return ESP_ERR_INVALID_ARG;
    }

    uint16_t normalizedClockDividers = (uint16_t)(rawClockDividers & (uint16_t)~FDC2214_CLOCK_RESERVED_MASK);
    uint16_t finSel = (uint16_t)((normalizedClockDividers & FDC2214_CLOCK_FIN_SEL_MASK) >> FDC2214_CLOCK_FIN_SEL_SHIFT);
    uint16_t frefDivider = (uint16_t)(normalizedClockDividers & FDC2214_CLOCK_FREF_DIVIDER_MASK);

    if (finSel == 0U) {
        ESP_LOGE(TAG, "CLOCK_DIVIDERS invalid: CHx_FIN_SEL must be explicit/non-zero (raw=0x%04X)", rawClockDividers);
        return ESP_ERR_INVALID_ARG;
    }
    if (frefDivider == 0U) {
        ESP_LOGE(TAG, "CLOCK_DIVIDERS invalid: CHx_FREF_DIVIDER must be non-zero (raw=0x%04X)", rawClockDividers);
        return ESP_ERR_INVALID_ARG;
    }

    /*
     * Use the explicit CHx_FIN_SEL numeric value as the divider factor.
     * Keeping this decode centralized avoids formula drift between layers.
     */
    outInfo->RawClockDividers = rawClockDividers;
    outInfo->NormalizedClockDividers = normalizedClockDividers;
    outInfo->FinSel = (uint8_t)finSel;
    outInfo->FinDivider = (uint8_t)finSel;
    outInfo->FrefDivider = frefDivider;
    return ESP_OK;
}

bool Fdc2214CapComputeSensorFrequencyHz(uint32_t raw28,
                                        uint32_t fClkHz,
                                        const Fdc2214CapClockDividerInfo_t* clockDividerInfo,
                                        Fdc2214CapFrequencyInfo_t* outFrequencyInfo)
{
    if (!clockDividerInfo || !outFrequencyInfo) {
        return false;
    }
    if (raw28 == 0U || fClkHz == 0U || clockDividerInfo->FrefDivider == 0U || clockDividerInfo->FinDivider == 0U) {
        return false;
    }

    double fclkHz = (double)fClkHz;
    double frefHz = fclkHz / (double)clockDividerInfo->FrefDivider;
    if (frefHz <= 0.0) {
        return false;
    }

    double finHz = ((double)raw28 * frefHz) / FDC2214_RAW_SCALE_2P28;
    double fsensorHz = finHz * (double)clockDividerInfo->FinDivider;
    if (finHz <= 0.0 || fsensorHz <= 0.0) {
        return false;
    }

    *outFrequencyInfo = (Fdc2214CapFrequencyInfo_t){
        .FclkHz = fclkHz,
        .FrefHz = frefHz,
        .FinHz = finHz,
        .FsensorHz = fsensorHz,
    };
    return true;
}

uint16_t Fdc2214CapMaskDriveCurrentRaw(uint16_t rawDriveCurrent)
{
    return (uint16_t)(rawDriveCurrent & FDC2214_DRIVE_CURRENT_MASK);
}

bool Fdc2214CapDriveStepToRaw(uint8_t step, uint16_t* outRawDriveCurrent)
{
    if (!outRawDriveCurrent || step > FDC2214_CAP_DRIVE_STEP_MAX) {
        return false;
    }
    *outRawDriveCurrent = (uint16_t)(((uint16_t)(step & 0x1Fu)) << 11u);
    return true;
}

bool Fdc2214CapDriveRawToStep(uint16_t rawDriveCurrent, uint8_t* outStep, uint16_t* outMaskedRawDriveCurrent)
{
    uint16_t masked = Fdc2214CapMaskDriveCurrentRaw(rawDriveCurrent);
    if (outMaskedRawDriveCurrent) {
        *outMaskedRawDriveCurrent = masked;
    }

    uint8_t step = (uint8_t)((masked >> 11u) & 0x1Fu);
    if (outStep) {
        *outStep = step;
    }
    return true;
}

static uint16_t Fdc2214CapNormalizeDriveCurrent(uint16_t rawDriveCurrent)
{
    return Fdc2214CapMaskDriveCurrentRaw(rawDriveCurrent);
}

static uint16_t Fdc2214CapNormalizeStatusConfig(uint16_t statusConfig)
{
    return (uint16_t)(statusConfig & FDC2214_STATUS_CONFIG_ALLOWED_MASK);
}

static void Fdc2214CapTxnMark(Fdc2214CapDevice_t* dev,
                              Fdc2214CapTxnOpKind_t opKind,
                              uint8_t reg,
                              size_t txLen,
                              size_t rxLen,
                              esp_err_t err,
                              uint8_t failedReg)
{
    if (!dev) {
        return;
    }
    dev->txnDiag.LastOpKind = opKind;
    dev->txnDiag.LastReg = reg;
    dev->txnDiag.LastTxLen = txLen;
    dev->txnDiag.LastRxLen = rxLen;
    dev->txnDiag.LastErr = err;
    dev->txnDiag.FailedReg = failedReg;
    if (err == ESP_OK) {
        dev->txnDiag.ConsecutiveFailCount = 0u;
        dev->txnDiag.LastSuccessTick = (uint32_t)xTaskGetTickCount();
    } else {
        dev->txnDiag.ConsecutiveFailCount++;
    }
}

static esp_err_t Fdc2214CapLock(Fdc2214CapDevice_t* dev)
{
    if (!dev || !dev->mutex) {
        return ESP_ERR_INVALID_STATE;
    }
    TickType_t timeoutTicks = pdMS_TO_TICKS((uint32_t)CONFIG_FDC2214CAP_MUTEX_TIMEOUT_MS);
    if (timeoutTicks == 0) {
        timeoutTicks = 1;
    }
    if (xSemaphoreTake(dev->mutex, timeoutTicks) != pdTRUE) {
        ESP_LOGE(TAG, "Mutex timeout after %u ms", (unsigned)CONFIG_FDC2214CAP_MUTEX_TIMEOUT_MS);
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
    if (!dev || !tx || txLen == 0U) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!dev->bus.Write) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = Fdc2214CapLock(dev);
    if (err != ESP_OK) {
        Fdc2214CapTxnMark(dev, FDC2214_TXN_OP_WRITE, tx[0], txLen, 0u, err, tx[0]);
        return err;
    }
    err = dev->bus.Write(dev->bus.UserCtx, dev->bus.I2cAddress7, tx, txLen);
    Fdc2214CapUnlock(dev);
    Fdc2214CapTxnMark(dev, FDC2214_TXN_OP_WRITE, tx[0], txLen, 0u, err, tx[0]);

    return err;
}

static esp_err_t Fdc2214CapWriteReadBytes(Fdc2214CapDevice_t* dev,
                                          const uint8_t* tx,
                                          size_t txLen,
                                          uint8_t* rx,
                                          size_t rxLen)
{
    if (!dev || !tx || txLen == 0U || !rx || rxLen == 0U) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!dev->bus.WriteRead) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = Fdc2214CapLock(dev);
    if (err != ESP_OK) {
        Fdc2214CapTxnMark(dev, FDC2214_TXN_OP_WRITE_READ, tx[0], txLen, rxLen, err, tx[0]);
        return err;
    }
    err = dev->bus.WriteRead(dev->bus.UserCtx, dev->bus.I2cAddress7, tx, txLen, rx, rxLen);
    Fdc2214CapUnlock(dev);
    Fdc2214CapTxnMark(dev, FDC2214_TXN_OP_WRITE_READ, tx[0], txLen, rxLen, err, tx[0]);

    return err;
}

static esp_err_t Fdc2214CapWriteReg16(Fdc2214CapDevice_t* dev, uint8_t reg, uint16_t value)
{
    uint8_t tx[3] = { reg, (uint8_t)(value >> 8), (uint8_t)(value & 0xFF) };
    esp_err_t err = Fdc2214CapWriteBytes(dev, tx, sizeof(tx));
    Fdc2214CapTxnMark(dev, FDC2214_TXN_OP_REG_WRITE, reg, sizeof(tx), 0u, err, reg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Write reg 0x%02X failed: %d", reg, err);
        return err;
    }
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
    Fdc2214CapTxnMark(dev, FDC2214_TXN_OP_REG_READ, reg, sizeof(tx), sizeof(rx), err, reg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Read reg 0x%02X failed: %d", reg, err);
        return err;
    }

    *outValue = (uint16_t)(((uint16_t)rx[0] << 8) | rx[1]);
    return ESP_OK;
}

static esp_err_t Fdc2214CapWriteReg16VerifyWithMask(Fdc2214CapDevice_t* dev,
                                                     uint8_t reg,
                                                     uint16_t expectedValue,
                                                     uint16_t compareMask,
                                                     bool mismatchIsWarning,
                                                     const char* regName,
                                                     bool* outMaskedMatch,
                                                     uint16_t* outReadbackValue)
{
    esp_err_t err = Fdc2214CapWriteReg16(dev, reg, expectedValue);
    if (err != ESP_OK) {
        return err;
    }

    uint16_t readbackValue = 0U;
    err = Fdc2214CapReadReg16(dev, reg, &readbackValue);
    if (err != ESP_OK) {
        return err;
    }

    uint16_t expectedMasked = (uint16_t)(expectedValue & compareMask);
    uint16_t readbackMasked = (uint16_t)(readbackValue & compareMask);
    bool maskedMatch = (expectedMasked == readbackMasked);

    if (outMaskedMatch) {
        *outMaskedMatch = maskedMatch;
    }
    if (outReadbackValue) {
        *outReadbackValue = readbackValue;
    }

    if (!maskedMatch) {
        if (mismatchIsWarning) {
            ESP_LOGW(TAG,
                     "%s readback masked mismatch reg 0x%02X: wrote 0x%04X read 0x%04X mask 0x%04X",
                     regName ? regName : "REG",
                     reg,
                     expectedValue,
                     readbackValue,
                     compareMask);
            return ESP_OK;
        }
        ESP_LOGE(TAG,
                 "%s readback mismatch reg 0x%02X: wrote 0x%04X read 0x%04X mask 0x%04X",
                 regName ? regName : "REG",
                 reg,
                 expectedValue,
                 readbackValue,
                 compareMask);
        return ESP_ERR_INVALID_RESPONSE;
    }
    return ESP_OK;
}

static esp_err_t Fdc2214CapWriteReg16Verify(Fdc2214CapDevice_t* dev, uint8_t reg, uint16_t expectedValue)
{
    return Fdc2214CapWriteReg16VerifyWithMask(dev,
                                              reg,
                                              expectedValue,
                                              0xFFFFu,
                                              false,
                                              "REG",
                                              NULL,
                                              NULL);
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
    dev->txnDiag = (Fdc2214CapTxnDiag_t){
        .LastOpKind = FDC2214_TXN_OP_NONE,
        .LastReg = 0u,
        .LastTxLen = 0u,
        .LastRxLen = 0u,
        .LastErr = ESP_OK,
        .ConsecutiveFailCount = 0u,
        .LastSuccessTick = 0u,
        .FailedReg = 0xFFu,
    };

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
    esp_err_t err = Fdc2214CapWriteReg16(dev, FDC2214_REG_RESET_DEV, FDC2214_RESET_DEV_BIT);
    if (err != ESP_OK) {
        return err;
    }
    // Keep post-reset access conservative; the next register transaction should not race device reset.
    Fdc2214CapDelayUs(FDC2214_RESET_SETTLE_US);
    return ESP_OK;
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

uint16_t Fdc2214CapBuildConfig(const Fdc2214CapConfigOptions_t* options)
{
    const Fdc2214CapConfigOptions_t defaults = {
        .ActiveChannel = FDC2214_CH0,
        .SleepModeEnabled = true,
        .SensorActivateSelLowPower = true,
        .RefClockSource = FDC2214_REF_CLOCK_INTERNAL,
        .IntbDisabled = false,
        .HighCurrentDrive = false,
    };
    Fdc2214CapConfigOptions_t cfg = options ? *options : defaults;

    if (!Fdc2214IsValidChannel(cfg.ActiveChannel)) {
        cfg.ActiveChannel = FDC2214_CH0;
    }
    if (!Fdc2214IsValidRefClock(cfg.RefClockSource)) {
        cfg.RefClockSource = FDC2214_REF_CLOCK_INTERNAL;
    }

    uint16_t config = 0U;
    config |= (uint16_t)((uint16_t)cfg.ActiveChannel << FDC2214_CONFIG_ACTIVE_CHAN_SHIFT);
    if (cfg.SleepModeEnabled) {
        config |= FDC2214_CONFIG_SLEEP_MODE_EN_MASK;
    }
    if (cfg.SensorActivateSelLowPower) {
        config |= FDC2214_CONFIG_SENSOR_ACTIVATE_SEL_MASK;
    }
    if (cfg.RefClockSource == FDC2214_REF_CLOCK_EXTERNAL) {
        config |= FDC2214_CONFIG_REF_CLK_SRC_MASK;
    }
    if (cfg.IntbDisabled) {
        config |= FDC2214_CONFIG_INTB_DIS_MASK;
    }
    if (cfg.HighCurrentDrive) {
        config |= FDC2214_CONFIG_HIGH_CURRENT_DRV_MASK;
    }

    return Fdc2214CapApplyConfigReservedBits(config);
}

esp_err_t Fdc2214CapEnterSleep(Fdc2214CapDevice_t* dev, uint16_t configWithoutSleep)
{
    if (!dev) {
        return ESP_ERR_INVALID_ARG;
    }

    uint16_t sleepConfig = Fdc2214CapApplyConfigReservedBits((uint16_t)(configWithoutSleep | FDC2214_CONFIG_SLEEP_MODE_EN_MASK));
    return Fdc2214CapWriteReg16Verify(dev, FDC2214_REG_CONFIG, sleepConfig);
}

esp_err_t Fdc2214CapExitSleep(Fdc2214CapDevice_t* dev, uint16_t configWithoutSleep)
{
    if (!dev) {
        return ESP_ERR_INVALID_ARG;
    }

    uint16_t activeConfig = Fdc2214CapApplyConfigReservedBits((uint16_t)(configWithoutSleep & ~FDC2214_CONFIG_SLEEP_MODE_EN_MASK));
    esp_err_t err = Fdc2214CapWriteReg16Verify(dev, FDC2214_REG_CONFIG, activeConfig);
    if (err != ESP_OK) {
        return err;
    }

    // Datasheet requires a short sleep-to-active wake-up interval before trusting conversions.
    Fdc2214CapDelayUs(FDC2214_SLEEP_WAKEUP_US);
    return ESP_OK;
}

esp_err_t Fdc2214CapReadStatus(Fdc2214CapDevice_t* dev, Fdc2214CapStatus_t* outStatus)
{
    if (!dev || !outStatus) {
        return ESP_ERR_INVALID_ARG;
    }

    uint16_t rawStatus = 0U;
    esp_err_t err = Fdc2214CapReadReg16(dev, FDC2214_REG_STATUS, &rawStatus);
    if (err != ESP_OK) {
        return err;
    }

    Fdc2214CapDecodeStatusRaw(rawStatus, outStatus);
    return ESP_OK;
}

void Fdc2214CapDecodeStatusHealth(const Fdc2214CapStatus_t* status, Fdc2214CapStatusHealth_t* outHealth)
{
    if (!outHealth) {
        return;
    }

    *outHealth = (Fdc2214CapStatusHealth_t){0};
    if (!status) {
        return;
    }

    bool anyUnread = false;
    for (size_t i = 0u; i < 4u; ++i) {
        anyUnread = anyUnread || status->UnreadConversion[i];
    }

    outHealth->WatchdogFault = status->ErrWatchdog;
    outHealth->AmplitudeFault = status->ErrAmplitudeHigh || status->ErrAmplitudeLow;
    outHealth->AnyFault = outHealth->WatchdogFault || outHealth->AmplitudeFault;
    outHealth->DataReady = status->DataReady;
    outHealth->AnyUnreadConversion = anyUnread;
}

esp_err_t Fdc2214CapReadStatusDecoded(Fdc2214CapDevice_t* dev,
                                      Fdc2214CapStatus_t* outStatus,
                                      Fdc2214CapStatusHealth_t* outHealth)
{
    if (!dev || !outStatus || !outHealth) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = Fdc2214CapReadStatus(dev, outStatus);
    if (err != ESP_OK) {
        return err;
    }

    Fdc2214CapDecodeStatusHealth(outStatus, outHealth);
    return ESP_OK;
}

esp_err_t Fdc2214CapReadCoreRegs(Fdc2214CapDevice_t* dev, Fdc2214CapCoreRegs_t* outRegs)
{
    if (!dev || !outRegs) {
        return ESP_ERR_INVALID_ARG;
    }

    Fdc2214CapCoreRegs_t regs = {0};
    esp_err_t err = Fdc2214CapReadReg16(dev, FDC2214_REG_STATUS, &regs.Status);
    if (err != ESP_OK) {
        return err;
    }
    err = Fdc2214CapReadReg16(dev, FDC2214_REG_STATUS_CONFIG, &regs.StatusConfig);
    if (err != ESP_OK) {
        return err;
    }
    err = Fdc2214CapReadReg16(dev, FDC2214_REG_CONFIG, &regs.Config);
    if (err != ESP_OK) {
        return err;
    }
    err = Fdc2214CapReadReg16(dev, FDC2214_REG_MUX_CONFIG, &regs.MuxConfig);
    if (err != ESP_OK) {
        return err;
    }

    *outRegs = regs;
    return ESP_OK;
}

esp_err_t Fdc2214CapReadDebugSnapshot(Fdc2214CapDevice_t* dev,
                                      Fdc2214CapChannel_t dataChannel,
                                      Fdc2214CapDebugSnapshot_t* outSnapshot)
{
    if (!dev || !outSnapshot) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!Fdc2214IsValidChannel(dataChannel)) {
        return ESP_ERR_INVALID_ARG;
    }

    Fdc2214CapDebugSnapshot_t snapshot = {
        .DataChannel = dataChannel,
        .ActiveChannel = FDC2214_CH0,
        .FailedReg = 0xFFu,
        .Partial = false,
    };

    uint8_t dataReg = Fdc2214RegForChannelStep2(FDC2214_REG_DATA_MSB_BASE, dataChannel);
    const uint8_t snapshotRegs[] = {
        FDC2214_REG_STATUS,
        FDC2214_REG_STATUS_CONFIG,
        FDC2214_REG_CONFIG,
        FDC2214_REG_MUX_CONFIG,
        Fdc2214RegForChannelStep1(FDC2214_REG_RCOUNT_BASE, FDC2214_CH0),
        Fdc2214RegForChannelStep1(FDC2214_REG_SETTLECOUNT_BASE, FDC2214_CH0),
        Fdc2214RegForChannelStep1(FDC2214_REG_CLOCK_DIVIDERS_BASE, FDC2214_CH0),
        Fdc2214RegForChannelStep1(FDC2214_REG_DRIVE_CURRENT_BASE, FDC2214_CH0),
        dataReg,
        (uint8_t)(dataReg + 1u),
    };
    uint16_t* snapshotFields[] = {
        &snapshot.Status,
        &snapshot.StatusConfig,
        &snapshot.Config,
        &snapshot.MuxConfig,
        &snapshot.RcountCh0,
        &snapshot.SettleCountCh0,
        &snapshot.ClockDividersCh0,
        &snapshot.DriveCurrentCh0,
        &snapshot.DataMsb,
        &snapshot.DataLsb,
    };

    esp_err_t err = ESP_OK;
    for (size_t i = 0u; i < (sizeof(snapshotRegs) / sizeof(snapshotRegs[0])); ++i) {
        err = Fdc2214CapReadReg16(dev, snapshotRegs[i], snapshotFields[i]);
        if (err != ESP_OK) {
            snapshot.Partial = true;
            snapshot.FailedReg = snapshotRegs[i];
            *outSnapshot = snapshot;
            Fdc2214CapTxnMark(dev, FDC2214_TXN_OP_SNAPSHOT, snapshotRegs[i], 1u, 2u, err, snapshotRegs[i]);
            return err;
        }
    }

    Fdc2214CapStatus_t statusDecoded = {0};
    Fdc2214CapDecodeStatusRaw(snapshot.Status, &statusDecoded);

    snapshot.DataRaw28 = ((uint32_t)(snapshot.DataMsb & FDC2214_DATA_MSB_MASK) << 16) | snapshot.DataLsb;
    snapshot.DataErrWatchdog = ((snapshot.DataMsb & FDC2214_DATA_ERR_WD_MASK) != 0U) || statusDecoded.ErrWatchdog;
    snapshot.DataErrAmplitude = ((snapshot.DataMsb & FDC2214_DATA_ERR_AW_MASK) != 0U) ||
                                statusDecoded.ErrAmplitudeHigh ||
                                statusDecoded.ErrAmplitudeLow;
    snapshot.ErrorChannel = statusDecoded.ErrorChannel;
    snapshot.StatusErrWatchdog = statusDecoded.ErrWatchdog;
    snapshot.StatusErrAmplitudeHigh = statusDecoded.ErrAmplitudeHigh;
    snapshot.StatusErrAmplitudeLow = statusDecoded.ErrAmplitudeLow;
    snapshot.DataReady = statusDecoded.DataReady;
    for (size_t i = 0U; i < 4U; ++i) {
        snapshot.UnreadConversion[i] = statusDecoded.UnreadConversion[i];
    }

    snapshot.ActiveChannel = Fdc2214CapActiveChannelFromConfig(snapshot.Config);
    snapshot.SleepModeEnabled = (snapshot.Config & FDC2214_CONFIG_SLEEP_MODE_EN_MASK) != 0U;
    snapshot.AutoScanEnabled = (snapshot.MuxConfig & FDC2214_MUX_AUTOSCAN_BIT) != 0U;
    snapshot.Converting = (!snapshot.SleepModeEnabled) &&
                          (snapshot.AutoScanEnabled || (snapshot.ActiveChannel == dataChannel));
    snapshot.FailedReg = 0xFFu;
    snapshot.Partial = false;

    *outSnapshot = snapshot;
    Fdc2214CapTxnMark(dev, FDC2214_TXN_OP_SNAPSHOT, (uint8_t)(dataReg + 1u), 1u, 2u, ESP_OK, 0xFFu);
    return ESP_OK;
}

esp_err_t Fdc2214CapGetLastTransactionDiag(Fdc2214CapDevice_t* dev, Fdc2214CapTxnDiag_t* outDiag)
{
    if (!dev || !outDiag) {
        return ESP_ERR_INVALID_ARG;
    }
    *outDiag = dev->txnDiag;
    return ESP_OK;
}

esp_err_t Fdc2214CapConfigureChannelWithResult(Fdc2214CapDevice_t* dev,
                                               Fdc2214CapChannel_t ch,
                                               const Fdc2214CapChannelConfig_t* cfg,
                                               Fdc2214CapChannelConfigResult_t* outResult,
                                               uint16_t* outDriveCurrentReadback)
{
    if (outResult) {
        *outResult = FDC2214_CHANNEL_CONFIG_RESULT_OK;
    }
    if (outDriveCurrentReadback) {
        *outDriveCurrentReadback = 0U;
    }

    if (!dev || !cfg) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!Fdc2214IsValidChannel(ch)) {
        return ESP_ERR_INVALID_ARG;
    }
    if (cfg->Rcount < 0x0100U) {
        ESP_LOGE(TAG, "RCOUNT must be >= 0x0100 (got 0x%04X)", cfg->Rcount);
        return ESP_ERR_INVALID_ARG;
    }
    if (cfg->SettleCount == 0U) {
        ESP_LOGE(TAG, "SETTLECOUNT must be non-zero");
        return ESP_ERR_INVALID_ARG;
    }

    uint16_t clockDividers = 0U;
    esp_err_t err = Fdc2214CapNormalizeClockDividers(cfg->ClockDividers, &clockDividers);
    if (err != ESP_OK) {
        return err;
    }

    uint16_t driveCurrent = Fdc2214CapNormalizeDriveCurrent(cfg->DriveCurrent);
    uint8_t driveStep = 0U;
    (void)Fdc2214CapDriveRawToStep(driveCurrent, &driveStep, NULL);
    if (driveCurrent != cfg->DriveCurrent) {
        ESP_LOGW(TAG,
                 "Drive current sanitized: reqRaw=0x%04X maskedRaw=0x%04X driveStep=%u",
                 cfg->DriveCurrent,
                 driveCurrent,
                 (unsigned)driveStep);
    }

    err = Fdc2214CapWriteReg16Verify(dev, Fdc2214RegForChannelStep1(FDC2214_REG_RCOUNT_BASE, ch), cfg->Rcount);
    if (err != ESP_OK) {
        return err;
    }
    err = Fdc2214CapWriteReg16Verify(dev, Fdc2214RegForChannelStep1(FDC2214_REG_SETTLECOUNT_BASE, ch), cfg->SettleCount);
    if (err != ESP_OK) {
        return err;
    }
    err = Fdc2214CapWriteReg16Verify(dev, Fdc2214RegForChannelStep1(FDC2214_REG_OFFSET_BASE, ch), cfg->Offset);
    if (err != ESP_OK) {
        return err;
    }
    err = Fdc2214CapWriteReg16Verify(dev, Fdc2214RegForChannelStep1(FDC2214_REG_CLOCK_DIVIDERS_BASE, ch), clockDividers);
    if (err != ESP_OK) {
        return err;
    }
    /*
     * DRIVE_CURRENT uses CHx_IDRIVE [15:11] only; lower bits are reserved.
     * Comparing all 16 bits can report false mismatches even when the effective
     * IDRIVE setting is correct. In bring-up/debug flows, an IDRIVE mismatch is
     * downgraded to warning so we can keep collecting status/raw data instead of
     * terminating visibility too early.
     */
    bool driveMaskedMatch = true;
    uint16_t driveReadback = 0U;
    err = Fdc2214CapWriteReg16VerifyWithMask(dev,
                                             Fdc2214RegForChannelStep1(FDC2214_REG_DRIVE_CURRENT_BASE, ch),
                                             driveCurrent,
                                             FDC2214_DRIVE_CURRENT_MASK,
                                             true,
                                             "DRIVE_CURRENT",
                                             &driveMaskedMatch,
                                             &driveReadback);
    if (err != ESP_OK) {
        return err;
    }
    if (outDriveCurrentReadback) {
        *outDriveCurrentReadback = driveReadback;
    }
    if (!driveMaskedMatch && outResult) {
        *outResult = FDC2214_CHANNEL_CONFIG_RESULT_WARN_DRIVE_CURRENT_MISMATCH;
    }

    ESP_LOGI(TAG,
             "Configured CH%d rcount=0x%04X settle=0x%04X offset=0x%04X clock=0x%04X "
             "driveStep=%u driveRawReq=0x%04X driveRawMasked=0x%04X",
             (int)ch,
             cfg->Rcount,
             cfg->SettleCount,
             cfg->Offset,
             clockDividers,
             (unsigned)driveStep,
             cfg->DriveCurrent,
             driveCurrent);
    return ESP_OK;
}

esp_err_t Fdc2214CapConfigureChannel(Fdc2214CapDevice_t* dev,
                                     Fdc2214CapChannel_t ch,
                                     const Fdc2214CapChannelConfig_t* cfg)
{
    return Fdc2214CapConfigureChannelWithResult(dev, ch, cfg, NULL, NULL);
}

esp_err_t Fdc2214CapReadbackVerifyChannelConfigWithResult(Fdc2214CapDevice_t* dev,
                                                          Fdc2214CapChannel_t ch,
                                                          const Fdc2214CapChannelConfig_t* expectedCfg,
                                                          Fdc2214CapChannelVerifyResult_t* outResult,
                                                          uint16_t* outDriveCurrentReadback)
{
    if (outResult) {
        *outResult = FDC2214_CHANNEL_VERIFY_RESULT_OK;
    }
    if (outDriveCurrentReadback) {
        *outDriveCurrentReadback = 0U;
    }

    if (!dev || !expectedCfg) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!Fdc2214IsValidChannel(ch)) {
        return ESP_ERR_INVALID_ARG;
    }

    uint16_t expectedClockDividers = 0U;
    esp_err_t err = Fdc2214CapNormalizeClockDividers(expectedCfg->ClockDividers, &expectedClockDividers);
    if (err != ESP_OK) {
        return err;
    }
    uint16_t expectedDriveCurrent = Fdc2214CapNormalizeDriveCurrent(expectedCfg->DriveCurrent);

    struct VerifyItem {
        uint8_t reg;
        uint16_t expected;
        const char* name;
    };
    const struct VerifyItem verifyItems[] = {
        { Fdc2214RegForChannelStep1(FDC2214_REG_RCOUNT_BASE, ch), expectedCfg->Rcount, "RCOUNT" },
        { Fdc2214RegForChannelStep1(FDC2214_REG_SETTLECOUNT_BASE, ch), expectedCfg->SettleCount, "SETTLECOUNT" },
        { Fdc2214RegForChannelStep1(FDC2214_REG_OFFSET_BASE, ch), expectedCfg->Offset, "OFFSET" },
        { Fdc2214RegForChannelStep1(FDC2214_REG_CLOCK_DIVIDERS_BASE, ch), expectedClockDividers, "CLOCK_DIVIDERS" },
    };

    for (size_t i = 0U; i < (sizeof(verifyItems) / sizeof(verifyItems[0])); ++i) {
        uint16_t readback = 0U;
        err = Fdc2214CapReadReg16(dev, verifyItems[i].reg, &readback);
        if (err != ESP_OK) {
            return err;
        }
        if (readback != verifyItems[i].expected) {
            ESP_LOGE(TAG,
                     "CH%d %s mismatch reg 0x%02X expected 0x%04X got 0x%04X",
                     (int)ch,
                     verifyItems[i].name,
                     verifyItems[i].reg,
                     verifyItems[i].expected,
                     readback);
            return ESP_ERR_INVALID_RESPONSE;
        }
    }

    uint16_t driveReadback = 0U;
    err = Fdc2214CapReadReg16(dev, Fdc2214RegForChannelStep1(FDC2214_REG_DRIVE_CURRENT_BASE, ch), &driveReadback);
    if (err != ESP_OK) {
        return err;
    }
    if (outDriveCurrentReadback) {
        *outDriveCurrentReadback = driveReadback;
    }

    uint16_t expectedDriveMasked = (uint16_t)(expectedDriveCurrent & FDC2214_DRIVE_CURRENT_MASK);
    uint16_t readDriveMasked = (uint16_t)(driveReadback & FDC2214_DRIVE_CURRENT_MASK);
    if (readDriveMasked != expectedDriveMasked) {
        uint8_t expectedStep = 0U;
        uint8_t readStep = 0U;
        (void)Fdc2214CapDriveRawToStep(expectedDriveMasked, &expectedStep, NULL);
        (void)Fdc2214CapDriveRawToStep(readDriveMasked, &readStep, NULL);
        ESP_LOGW(TAG,
                 "CH%d DRIVE_CURRENT IDRIVE mismatch reg 0x%02X expected(masked)=0x%04X(step=%u) "
                 "got(masked)=0x%04X(step=%u) raw=0x%04X",
                 (int)ch,
                 Fdc2214RegForChannelStep1(FDC2214_REG_DRIVE_CURRENT_BASE, ch),
                 expectedDriveMasked,
                 (unsigned)expectedStep,
                 readDriveMasked,
                 (unsigned)readStep,
                 driveReadback);
        if (outResult) {
            *outResult = FDC2214_CHANNEL_VERIFY_RESULT_WARN_DRIVE_CURRENT_MISMATCH;
        }
    }

    return ESP_OK;
}

esp_err_t Fdc2214CapReadbackVerifyChannelConfig(Fdc2214CapDevice_t* dev,
                                                Fdc2214CapChannel_t ch,
                                                const Fdc2214CapChannelConfig_t* expectedCfg)
{
    return Fdc2214CapReadbackVerifyChannelConfigWithResult(dev, ch, expectedCfg, NULL, NULL);
}

esp_err_t Fdc2214CapSetStatusConfig(Fdc2214CapDevice_t* dev, uint16_t statusConfig)
{
    if (!dev) {
        return ESP_ERR_INVALID_ARG;
    }

    uint16_t normalized = Fdc2214CapNormalizeStatusConfig(statusConfig);
    if (normalized != statusConfig) {
        ESP_LOGW(TAG, "STATUS_CONFIG masked from 0x%04X to 0x%04X", statusConfig, normalized);
    }
    return Fdc2214CapWriteReg16Verify(dev, FDC2214_REG_STATUS_CONFIG, normalized);
}

esp_err_t Fdc2214CapSetMuxConfig(Fdc2214CapDevice_t* dev,
                                 bool autoScan,
                                 uint8_t rrSequence,
                                 Fdc2214CapDeglitch_t deglitch)
{
    if (!dev) {
        return ESP_ERR_INVALID_ARG;
    }

    uint16_t muxValue = 0U;
    esp_err_t err = Fdc2214CapBuildMuxValue(autoScan, rrSequence, deglitch, &muxValue);
    if (err != ESP_OK) {
        return err;
    }
    return Fdc2214CapWriteReg16Verify(dev, FDC2214_REG_MUX_CONFIG, muxValue);
}

esp_err_t Fdc2214CapSetSingleChannelMode(Fdc2214CapDevice_t* dev, Fdc2214CapChannel_t activeCh)
{
    if (!dev) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!Fdc2214IsValidChannel(activeCh)) {
        return ESP_ERR_INVALID_ARG;
    }

    uint16_t muxReg = 0U;
    esp_err_t err = Fdc2214CapReadReg16(dev, FDC2214_REG_MUX_CONFIG, &muxReg);
    if (err != ESP_OK) {
        return err;
    }

    Fdc2214CapDeglitch_t deglitch = (Fdc2214CapDeglitch_t)(muxReg & FDC2214_MUX_DEGLITCH_MASK);
    if (!Fdc2214IsValidDeglitch(deglitch)) {
        deglitch = FDC2214_DEGLITCH_10MHZ;
    }
    err = Fdc2214CapSetMuxConfig(dev, false, 0U, deglitch);
    if (err != ESP_OK) {
        return err;
    }

    uint16_t configReg = 0U;
    err = Fdc2214CapReadReg16(dev, FDC2214_REG_CONFIG, &configReg);
    if (err != ESP_OK) {
        return err;
    }

    Fdc2214CapConfigOptions_t options = Fdc2214CapConfigOptionsFromRaw(configReg);
    options.ActiveChannel = activeCh;
    options.SleepModeEnabled = false;
    uint16_t newConfig = Fdc2214CapBuildConfig(&options);
    err = Fdc2214CapWriteReg16Verify(dev, FDC2214_REG_CONFIG, newConfig);
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
    if (rrSequence > 2U) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!Fdc2214IsValidDeglitch(deglitch)) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = Fdc2214CapSetMuxConfig(dev, true, rrSequence, deglitch);
    if (err != ESP_OK) {
        return err;
    }

    uint16_t configReg = 0U;
    err = Fdc2214CapReadReg16(dev, FDC2214_REG_CONFIG, &configReg);
    if (err != ESP_OK) {
        return err;
    }

    // Keep ref-clock/intb/high-current policy but force active conversion state.
    Fdc2214CapConfigOptions_t options = Fdc2214CapConfigOptionsFromRaw(configReg);
    options.ActiveChannel = FDC2214_CH0;
    options.SleepModeEnabled = false;
    uint16_t newConfig = Fdc2214CapBuildConfig(&options);
    err = Fdc2214CapWriteReg16Verify(dev, FDC2214_REG_CONFIG, newConfig);
    if (err != ESP_OK) {
        return err;
    }

    ESP_LOGI(TAG, "Autoscan mode set, rrSequence=%u", (unsigned)rrSequence);
    return ESP_OK;
}

static esp_err_t Fdc2214CapReadSampleWithValidityMode(Fdc2214CapDevice_t* dev,
                                                       Fdc2214CapChannel_t ch,
                                                       bool relaxedValidity,
                                                       Fdc2214CapSample_t* outSample)
{
    if (!dev || !outSample) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!Fdc2214IsValidChannel(ch)) {
        return ESP_ERR_INVALID_ARG;
    }

    *outSample = (Fdc2214CapSample_t){
        .ActiveChannel = FDC2214_CH0,
        .RefClockSource = FDC2214_REF_CLOCK_INTERNAL,
        .SampleStatus = FDC2214_SAMPLE_STATUS_CONFIG_UNKNOWN,
    };

    Fdc2214CapDebugSnapshot_t snapshot = {0};
    esp_err_t err = Fdc2214CapReadDebugSnapshot(dev, ch, &snapshot);
    if (err != ESP_OK) {
        return err;
    }

    bool unreadConversion = snapshot.UnreadConversion[(uint8_t)ch];
    bool configKnown = Fdc2214CapConfigReservedBitsValid(snapshot.Config) &&
                       ((snapshot.MuxConfig & FDC2214_MUX_FIXED_MASK) == FDC2214_MUX_FIXED_MASK);

    outSample->Raw28 = snapshot.DataRaw28;
    outSample->ErrWatchdog = snapshot.DataErrWatchdog;
    outSample->ErrAmplitude = snapshot.DataErrAmplitude;
    outSample->StatusRaw = snapshot.Status;
    outSample->ConfigRaw = snapshot.Config;
    outSample->MuxRaw = snapshot.MuxConfig;
    outSample->SleepModeEnabled = snapshot.SleepModeEnabled;
    outSample->AutoScanEnabled = snapshot.AutoScanEnabled;
    outSample->Converting = snapshot.Converting;
    outSample->UnreadConversionPresent = unreadConversion;
    outSample->DataReady = snapshot.DataReady;
    outSample->ActiveChannel = snapshot.ActiveChannel;
    outSample->RefClockSource = Fdc2214CapRefClockFromConfig(snapshot.Config);

    Fdc2214CapSampleStatus_t semanticStatus = FDC2214_SAMPLE_STATUS_CONFIG_UNKNOWN;
    if (!configKnown) {
        semanticStatus = FDC2214_SAMPLE_STATUS_CONFIG_UNKNOWN;
    } else if (snapshot.SleepModeEnabled) {
        semanticStatus = FDC2214_SAMPLE_STATUS_STILL_SLEEPING;
    } else if (!snapshot.Converting) {
        semanticStatus = FDC2214_SAMPLE_STATUS_I2C_READ_OK_BUT_NOT_CONVERTING;
    } else if (!unreadConversion) {
        semanticStatus = FDC2214_SAMPLE_STATUS_NO_UNREAD_CONVERSION;
    } else if (outSample->ErrWatchdog) {
        semanticStatus = FDC2214_SAMPLE_STATUS_WATCHDOG_FAULT;
    } else if (outSample->ErrAmplitude) {
        semanticStatus = FDC2214_SAMPLE_STATUS_AMPLITUDE_FAULT;
    } else if (snapshot.DataRaw28 == 0U) {
        // An all-zero payload can be stale data from a non-converting/sleeping path.
        // Do not auto-promote transport success to semantic sample validity.
        semanticStatus = FDC2214_SAMPLE_STATUS_ZERO_RAW_INVALID;
    } else {
        semanticStatus = FDC2214_SAMPLE_STATUS_SAMPLE_VALID;
    }

    outSample->SampleStatus = semanticStatus;
    if (relaxedValidity) {
        /*
         * Relaxed bring-up validity keeps payload visibility when transport is healthy
         * even if unread/amplitude/watchdog flags are present.
         */
        outSample->SampleValid = configKnown && snapshot.Converting && (snapshot.DataRaw28 != 0U);
    } else {
        outSample->SampleValid = (semanticStatus == FDC2214_SAMPLE_STATUS_SAMPLE_VALID);
    }
    return ESP_OK;
}

esp_err_t Fdc2214CapReadSample(Fdc2214CapDevice_t* dev, Fdc2214CapChannel_t ch, Fdc2214CapSample_t* outSample)
{
    return Fdc2214CapReadSampleWithValidityMode(dev, ch, false, outSample);
}

esp_err_t Fdc2214CapReadSampleRelaxed(Fdc2214CapDevice_t* dev,
                                      Fdc2214CapChannel_t ch,
                                      Fdc2214CapSample_t* outSample)
{
    return Fdc2214CapReadSampleWithValidityMode(dev, ch, true, outSample);
}

esp_err_t Fdc2214CapReadRawRegisters(Fdc2214CapDevice_t* dev, uint8_t reg, uint16_t* outValue)
{
    if (!dev || !outValue) {
        return ESP_ERR_INVALID_ARG;
    }
    return Fdc2214CapReadReg16(dev, reg, outValue);
}

esp_err_t Fdc2214CapWriteRawRegisters(Fdc2214CapDevice_t* dev, uint8_t reg, uint16_t value)
{
    if (!dev) {
        return ESP_ERR_INVALID_ARG;
    }
    if (reg >= FDC2214_REG_DRIVE_CURRENT_BASE && reg <= (uint8_t)(FDC2214_REG_DRIVE_CURRENT_BASE + 3u)) {
        uint16_t masked = Fdc2214CapMaskDriveCurrentRaw(value);
        if (masked != value) {
            uint8_t step = 0U;
            (void)Fdc2214CapDriveRawToStep(masked, &step, NULL);
            ESP_LOGW(TAG,
                     "Raw DRIVE_CURRENT write sanitized reg=0x%02X reqRaw=0x%04X maskedRaw=0x%04X step=%u",
                     reg,
                     value,
                     masked,
                     (unsigned)step);
        }
        value = masked;
    }
    return Fdc2214CapWriteReg16(dev, reg, value);
}

const char* Fdc2214CapSampleStatusName(Fdc2214CapSampleStatus_t status)
{
    switch (status) {
    case FDC2214_SAMPLE_STATUS_SAMPLE_VALID:
        return "sample_valid";
    case FDC2214_SAMPLE_STATUS_STILL_SLEEPING:
        return "still_sleeping";
    case FDC2214_SAMPLE_STATUS_I2C_READ_OK_BUT_NOT_CONVERTING:
        return "i2c_read_ok_but_not_converting";
    case FDC2214_SAMPLE_STATUS_NO_UNREAD_CONVERSION:
        return "no_unread_conversion";
    case FDC2214_SAMPLE_STATUS_ZERO_RAW_INVALID:
        return "zero_raw_invalid";
    case FDC2214_SAMPLE_STATUS_WATCHDOG_FAULT:
        return "watchdog_fault";
    case FDC2214_SAMPLE_STATUS_AMPLITUDE_FAULT:
        return "amplitude_fault";
    case FDC2214_SAMPLE_STATUS_CONFIG_UNKNOWN:
    default:
        return "config_unknown";
    }
}

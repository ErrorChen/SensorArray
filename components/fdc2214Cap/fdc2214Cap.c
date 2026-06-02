#include "fdc2214Cap.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "sdkconfig.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#ifndef CONFIG_FDC2214CAP_LOG_LEVEL
#define CONFIG_FDC2214CAP_LOG_LEVEL 3
#endif
#ifndef CONFIG_FDC2214CAP_MUTEX_TIMEOUT_MS
#define CONFIG_FDC2214CAP_MUTEX_TIMEOUT_MS 200
#endif
#ifndef CONFIG_FDC2214CAP_LOW_LEVEL_I2C_TRACE
#define CONFIG_FDC2214CAP_LOW_LEVEL_I2C_TRACE 0
#endif
#ifndef CONFIG_FDC2214CAP_RAW_I2C_TRACE
#define CONFIG_FDC2214CAP_RAW_I2C_TRACE 0
#endif
#ifndef CONFIG_SENSORARRAY_FDC_I2C_TRACE_RING_SIZE
#define CONFIG_SENSORARRAY_FDC_I2C_TRACE_RING_SIZE 128
#endif
#ifndef LOG_LOCAL_LEVEL
#define LOG_LOCAL_LEVEL CONFIG_FDC2214CAP_LOG_LEVEL
#endif
#include "esp_log.h"

static const char* TAG = "fdc2214Cap";

#if CONFIG_FDC2214CAP_LOW_LEVEL_I2C_TRACE
#define FDCLOW_TRACE(...) printf(__VA_ARGS__)
#else
#define FDCLOW_TRACE(...) do { } while (0)
#endif

#if CONFIG_FDC2214CAP_RAW_I2C_TRACE
#define FDC_RAW_TRACE(...) printf(__VA_ARGS__)
#else
#define FDC_RAW_TRACE(...) do { } while (0)
#endif

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

// Drive current register uses CHx_IDRIVE [15:11]; reserved bits must stay clear.
#define FDC2214_DRIVE_CURRENT_MASK 0xF800

// Keep the same saturation guard as the sweep layer for strict driver samples.
#define FDC2214_RAW28_SATURATED_THRESHOLD 0x0FFFFF00U

// Datasheet timing (FDC221x): sleep-to-active wake-up time typ/min requirement.
#define FDC2214_SLEEP_WAKEUP_US 50U

typedef struct Fdc2214CapDevice {
    Fdc2214CapBusConfig_t bus;
    SemaphoreHandle_t mutex;
    Fdc2214CapI2cStats_t i2cStats;
} Fdc2214CapDevice_t;

typedef struct {
    uint32_t sequence;
    int64_t timestampUs;
    const char *op;
    uint8_t addr7;
    uint16_t txLen;
    uint16_t rxLen;
    uint64_t elapsedUs;
    esp_err_t err;
} Fdc2214CapI2cTraceRecord_t;

#if CONFIG_SENSORARRAY_FDC_I2C_TRACE_RING_SIZE > 0
static Fdc2214CapI2cTraceRecord_t s_i2cTraceRing[CONFIG_SENSORARRAY_FDC_I2C_TRACE_RING_SIZE];
#endif
static bool s_i2cTraceEnabled = false;
static uint32_t s_i2cTraceWriteIndex = 0u;
static uint32_t s_i2cTraceSequence = 0u;

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

static void Fdc2214CapTraceRecord(const char *op,
                                  uint8_t addr7,
                                  size_t txLen,
                                  size_t rxLen,
                                  uint64_t elapsedUs,
                                  esp_err_t err)
{
#if CONFIG_SENSORARRAY_FDC_I2C_TRACE_RING_SIZE > 0
    if (!s_i2cTraceEnabled) {
        return;
    }
    uint32_t slot = s_i2cTraceWriteIndex % (uint32_t)CONFIG_SENSORARRAY_FDC_I2C_TRACE_RING_SIZE;
    s_i2cTraceRing[slot] = (Fdc2214CapI2cTraceRecord_t){
        .sequence = s_i2cTraceSequence++,
        .timestampUs = esp_timer_get_time(),
        .op = op ? op : "unknown",
        .addr7 = addr7,
        .txLen = (uint16_t)((txLen > UINT16_MAX) ? UINT16_MAX : txLen),
        .rxLen = (uint16_t)((rxLen > UINT16_MAX) ? UINT16_MAX : rxLen),
        .elapsedUs = elapsedUs,
        .err = err,
    };
    s_i2cTraceWriteIndex++;
#else
    (void)op;
    (void)addr7;
    (void)txLen;
    (void)rxLen;
    (void)elapsedUs;
    (void)err;
#endif
}

static void Fdc2214CapAccumulateI2cStats(Fdc2214CapDevice_t *dev,
                                         bool readTransaction,
                                         size_t txLen,
                                         size_t rxLen,
                                         uint64_t elapsedUs,
                                         esp_err_t err)
{
    if (!dev) {
        return;
    }

    if (readTransaction) {
        dev->i2cStats.readCount++;
        dev->i2cStats.readBytes += (uint32_t)((rxLen > UINT32_MAX) ? UINT32_MAX : rxLen);
    } else {
        dev->i2cStats.writeCount++;
    }
    dev->i2cStats.writeBytes += (uint32_t)((txLen > UINT32_MAX) ? UINT32_MAX : txLen);
    dev->i2cStats.totalUs += elapsedUs;

    if (err == ESP_ERR_TIMEOUT) {
        dev->i2cStats.timeoutCount++;
    } else if (err == ESP_FAIL) {
        dev->i2cStats.nackCount++;
    } else if (err != ESP_OK) {
        dev->i2cStats.retryCount++;
    }
}

static uint64_t Fdc2214CapElapsedUs(int64_t startUs)
{
    int64_t elapsedUs = esp_timer_get_time() - startUs;
    return (elapsedUs > 0) ? (uint64_t)elapsedUs : 0u;
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

uint16_t Fdc2214CapUnreadMaskForChannel(Fdc2214CapChannel_t ch)
{
    switch (ch) {
    case FDC2214_CH0:
        return FDC2214CAP_STATUS_CH0_UNREAD_MASK;
    case FDC2214_CH1:
        return FDC2214CAP_STATUS_CH1_UNREAD_MASK;
    case FDC2214_CH2:
        return FDC2214CAP_STATUS_CH2_UNREAD_MASK;
    case FDC2214_CH3:
        return FDC2214CAP_STATUS_CH3_UNREAD_MASK;
    default:
        return 0u;
    }
}

#if CONFIG_SENSORARRAY_FDC_VERBOSE_SCAN_LOG
static const char *Fdc2214CapRrSequenceName(uint8_t rrSequence)
{
    switch (rrSequence) {
    case FDC2214_RR_SEQUENCE_CH0_CH1:
        return "ch0_ch1";
    case FDC2214_RR_SEQUENCE_CH0_CH1_CH2:
        return "ch0_ch1_ch2";
    case FDC2214_RR_SEQUENCE_CH0_CH1_CH2_CH3:
        return "ch0_ch1_ch2_ch3";
    default:
        return "invalid";
    }
}
#endif

esp_err_t Fdc2214CapDecodeStatusRaw(uint16_t statusRaw, Fdc2214CapStatus_t* outStatus)
{
    if (!outStatus) {
        return ESP_ERR_INVALID_ARG;
    }

    *outStatus = (Fdc2214CapStatus_t){
        .Raw = statusRaw,
        .ErrorChannel = (uint8_t)((statusRaw & FDC2214CAP_STATUS_ERR_CHAN_MASK) >>
                                  FDC2214CAP_STATUS_ERR_CHAN_SHIFT),
        .ErrWatchdog = (statusRaw & FDC2214CAP_STATUS_ERR_WD_MASK) != 0U,
        .ErrAmplitudeHigh = (statusRaw & FDC2214CAP_STATUS_ERR_AHW_MASK) != 0U,
        .ErrAmplitudeLow = (statusRaw & FDC2214CAP_STATUS_ERR_ALW_MASK) != 0U,
        .DataReady = (statusRaw & FDC2214CAP_STATUS_DRDY_MASK) != 0U,
        .UnreadConversion =
            {
                (statusRaw & FDC2214CAP_STATUS_CH0_UNREAD_MASK) != 0U,
                (statusRaw & FDC2214CAP_STATUS_CH1_UNREAD_MASK) != 0U,
                (statusRaw & FDC2214CAP_STATUS_CH2_UNREAD_MASK) != 0U,
                (statusRaw & FDC2214CAP_STATUS_CH3_UNREAD_MASK) != 0U,
            },
    };
    return ESP_OK;
}

bool Fdc2214CapStatusHasAmplitudeFault(const Fdc2214CapStatus_t *status)
{
    return status && (status->ErrAmplitudeHigh || status->ErrAmplitudeLow);
}

bool Fdc2214CapStatusHasWatchdogFault(const Fdc2214CapStatus_t *status)
{
    return status && status->ErrWatchdog;
}

bool Fdc2214CapStatusHasUnreadForChannel(const Fdc2214CapStatus_t *status,
                                         Fdc2214CapChannel_t ch)
{
    if (!status || !Fdc2214IsValidChannel(ch)) {
        return false;
    }
    return status->UnreadConversion[(uint8_t)ch];
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

    uint16_t clockDividers = (uint16_t)(rawClockDividers & (uint16_t)~FDC2214_CLOCK_RESERVED_MASK);
    uint16_t finSel = (uint16_t)((clockDividers & FDC2214_CLOCK_FIN_SEL_MASK) >> FDC2214_CLOCK_FIN_SEL_SHIFT);
    uint16_t frefDiv = (uint16_t)(clockDividers & FDC2214_CLOCK_FREF_DIVIDER_MASK);

    if (finSel == 0U) {
        ESP_LOGE(TAG, "CLOCK_DIVIDERS invalid: CHx_FIN_SEL must be explicit/non-zero (raw=0x%04X)", rawClockDividers);
        return ESP_ERR_INVALID_ARG;
    }
    if (frefDiv == 0U) {
        ESP_LOGE(TAG, "CLOCK_DIVIDERS invalid: CHx_FREF_DIVIDER must be non-zero (raw=0x%04X)", rawClockDividers);
        return ESP_ERR_INVALID_ARG;
    }

    *outClockDividers = clockDividers;
    return ESP_OK;
}

static uint16_t Fdc2214CapNormalizeDriveCurrent(uint16_t rawDriveCurrent)
{
    return (uint16_t)(rawDriveCurrent & FDC2214_DRIVE_CURRENT_MASK);
}

static uint16_t Fdc2214CapNormalizeStatusConfig(uint16_t statusConfig)
{
    return (uint16_t)(statusConfig & FDC2214_STATUS_CONFIG_ALLOWED_MASK);
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

    FDCLOW_TRACE("FDCLOW,op=write_begin,addr=0x%02X,txLen=%u\n",
                 dev->bus.I2cAddress7,
                 (unsigned)txLen);
    esp_err_t err = Fdc2214CapLock(dev);
    if (err != ESP_OK) {
        FDCLOW_TRACE("FDCLOW,op=write_done,addr=0x%02X,err=%ld\n",
                     dev->bus.I2cAddress7,
                     (long)err);
        return err;
    }
    int64_t startUs = esp_timer_get_time();
    err = dev->bus.Write(dev->bus.UserCtx, dev->bus.I2cAddress7, tx, txLen);
    uint64_t elapsedUs = Fdc2214CapElapsedUs(startUs);
    Fdc2214CapUnlock(dev);
    Fdc2214CapAccumulateI2cStats(dev, false, txLen, 0u, elapsedUs, err);
    Fdc2214CapTraceRecord("write", dev->bus.I2cAddress7, txLen, 0u, elapsedUs, err);

    FDCLOW_TRACE("FDCLOW,op=write_done,addr=0x%02X,err=%ld\n",
                 dev->bus.I2cAddress7,
                 (long)err);
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

    FDCLOW_TRACE("FDCLOW,op=write_read_begin,addr=0x%02X,txLen=%u,rxLen=%u\n",
                 dev->bus.I2cAddress7,
                 (unsigned)txLen,
                 (unsigned)rxLen);
    esp_err_t err = Fdc2214CapLock(dev);
    if (err != ESP_OK) {
        FDCLOW_TRACE("FDCLOW,op=write_read_done,addr=0x%02X,err=%ld\n",
                     dev->bus.I2cAddress7,
                     (long)err);
        return err;
    }
    int64_t startUs = esp_timer_get_time();
    err = dev->bus.WriteRead(dev->bus.UserCtx, dev->bus.I2cAddress7, tx, txLen, rx, rxLen);
    uint64_t elapsedUs = Fdc2214CapElapsedUs(startUs);
    Fdc2214CapUnlock(dev);
    Fdc2214CapAccumulateI2cStats(dev, true, txLen, rxLen, elapsedUs, err);
    Fdc2214CapTraceRecord("write_read", dev->bus.I2cAddress7, txLen, rxLen, elapsedUs, err);

    FDCLOW_TRACE("FDCLOW,op=write_read_done,addr=0x%02X,err=%ld\n",
                 dev->bus.I2cAddress7,
                 (long)err);
    return err;
}

static esp_err_t Fdc2214CapWriteReg16(Fdc2214CapDevice_t* dev, uint8_t reg, uint16_t value)
{
    FDCLOW_TRACE("FDCLOW,op=write_reg_begin,reg=0x%02X,value=0x%04X\n", reg, value);
    uint8_t tx[3] = { reg, (uint8_t)(value >> 8), (uint8_t)(value & 0xFF) };
    esp_err_t err = Fdc2214CapWriteBytes(dev, tx, sizeof(tx));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Write reg 0x%02X failed: %d", reg, err);
        FDCLOW_TRACE("FDCLOW,op=write_reg_done,reg=0x%02X,err=%ld\n", reg, (long)err);
        return err;
    }
    FDCLOW_TRACE("FDCLOW,op=write_reg_done,reg=0x%02X,err=%ld\n", reg, (long)ESP_OK);
    return ESP_OK;
}

static esp_err_t Fdc2214CapReadReg16(Fdc2214CapDevice_t* dev, uint8_t reg, uint16_t* outValue)
{
    if (!outValue) {
        return ESP_ERR_INVALID_ARG;
    }

    FDCLOW_TRACE("FDCLOW,op=read_reg_begin,reg=0x%02X\n", reg);
    uint8_t tx = reg;
    uint8_t rx[2] = {0};
    esp_err_t err = Fdc2214CapWriteReadBytes(dev, &tx, sizeof(tx), rx, sizeof(rx));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Read reg 0x%02X failed: %d", reg, err);
        FDCLOW_TRACE("FDCLOW,op=read_reg_done,reg=0x%02X,err=%ld,value=0x%04X\n",
                     reg,
                     (long)err,
                     0u);
        return err;
    }

    *outValue = (uint16_t)(((uint16_t)rx[0] << 8) | rx[1]);
    FDCLOW_TRACE("FDCLOW,op=read_reg_done,reg=0x%02X,err=%ld,value=0x%04X\n",
                 reg,
                 (long)ESP_OK,
                 *outValue);
    return ESP_OK;
}

static esp_err_t Fdc2214CapReadReg16Verify(Fdc2214CapDevice_t* dev, uint8_t reg, uint16_t* outValue)
{
    if (dev) {
        dev->i2cStats.verifyReadCount++;
    }
    return Fdc2214CapReadReg16(dev, reg, outValue);
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
    FDCLOW_TRACE("FDCLOW,op=write_reg_verify_begin,reg=0x%02X,value=0x%04X,mask=0x%04X\n",
                 reg,
                 expectedValue,
                 compareMask);
    esp_err_t err = Fdc2214CapWriteReg16(dev, reg, expectedValue);
    if (err != ESP_OK) {
        FDCLOW_TRACE("FDCLOW,op=write_reg_verify_done,reg=0x%02X,err=%ld,readback=0x%04X\n",
                     reg,
                     (long)err,
                     0u);
        return err;
    }

    uint16_t readbackValue = 0U;
    err = Fdc2214CapReadReg16Verify(dev, reg, &readbackValue);
    if (err != ESP_OK) {
        FDCLOW_TRACE("FDCLOW,op=write_reg_verify_done,reg=0x%02X,err=%ld,readback=0x%04X\n",
                     reg,
                     (long)err,
                     readbackValue);
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
            FDCLOW_TRACE("FDCLOW,op=write_reg_verify_done,reg=0x%02X,err=%ld,readback=0x%04X,status=warning_mismatch\n",
                         reg,
                         (long)ESP_OK,
                         readbackValue);
            return ESP_OK;
        }
        ESP_LOGE(TAG,
                 "%s readback mismatch reg 0x%02X: wrote 0x%04X read 0x%04X mask 0x%04X",
                 regName ? regName : "REG",
                 reg,
                 expectedValue,
                 readbackValue,
                 compareMask);
        FDCLOW_TRACE("FDCLOW,op=write_reg_verify_done,reg=0x%02X,err=%ld,readback=0x%04X,status=mismatch\n",
                     reg,
                     (long)ESP_ERR_INVALID_RESPONSE,
                     readbackValue);
        return ESP_ERR_INVALID_RESPONSE;
    }
    FDCLOW_TRACE("FDCLOW,op=write_reg_verify_done,reg=0x%02X,err=%ld,readback=0x%04X,status=match\n",
                 reg,
                 (long)ESP_OK,
                 readbackValue);
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

void Fdc2214CapResetI2cStats(Fdc2214CapDevice_t* dev)
{
    if (dev) {
        dev->i2cStats = (Fdc2214CapI2cStats_t){0};
    }
}

void Fdc2214CapGetI2cStats(Fdc2214CapDevice_t* dev, Fdc2214CapI2cStats_t* outStats)
{
    if (!outStats) {
        return;
    }
    *outStats = dev ? dev->i2cStats : (Fdc2214CapI2cStats_t){0};
}

void Fdc2214CapI2cTraceSetEnabled(bool enabled)
{
    s_i2cTraceEnabled = enabled;
}

bool Fdc2214CapI2cTraceIsEnabled(void)
{
    return s_i2cTraceEnabled;
}

void Fdc2214CapI2cTraceClear(void)
{
#if CONFIG_SENSORARRAY_FDC_I2C_TRACE_RING_SIZE > 0
    memset(s_i2cTraceRing, 0, sizeof(s_i2cTraceRing));
#endif
    s_i2cTraceWriteIndex = 0u;
    s_i2cTraceSequence = 0u;
}

void Fdc2214CapI2cTraceDump(void)
{
#if CONFIG_SENSORARRAY_FDC_I2C_TRACE_RING_SIZE > 0
    uint32_t ringSize = (uint32_t)CONFIG_SENSORARRAY_FDC_I2C_TRACE_RING_SIZE;
    uint32_t count = (s_i2cTraceWriteIndex < ringSize) ? s_i2cTraceWriteIndex : ringSize;
    uint32_t first = (s_i2cTraceWriteIndex >= count) ? (s_i2cTraceWriteIndex - count) : 0u;
    for (uint32_t i = 0u; i < count; ++i) {
        const Fdc2214CapI2cTraceRecord_t *rec = &s_i2cTraceRing[(first + i) % ringSize];
        printf("I2C_TRACE,seq=%lu,timestampUs=%lld,op=%s,addr=0x%02X,txLen=%u,rxLen=%u,elapsedUs=%llu,err=0x%lx\n",
               (unsigned long)rec->sequence,
               (long long)rec->timestampUs,
               rec->op ? rec->op : "unknown",
               rec->addr7,
               (unsigned)rec->txLen,
               (unsigned)rec->rxLen,
               (unsigned long long)rec->elapsedUs,
               (unsigned long)rec->err);
    }
    printf("I2C_TRACE_DUMP,count=%lu,ringSize=%lu,enabled=%u\n",
           (unsigned long)count,
           (unsigned long)ringSize,
           s_i2cTraceEnabled ? 1u : 0u);
#else
    printf("I2C_TRACE_DUMP,count=0,ringSize=0,enabled=%u\n", s_i2cTraceEnabled ? 1u : 0u);
#endif
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

    return Fdc2214CapDecodeStatusRaw(rawStatus, outStatus);
}

esp_err_t Fdc2214CapClearStatus(Fdc2214CapDevice_t* dev)
{
    if (!dev) {
        return ESP_ERR_INVALID_ARG;
    }

    uint16_t throwaway = 0U;
    esp_err_t firstErr = Fdc2214CapReadReg16(dev, FDC2214_REG_STATUS, &throwaway);
    for (uint8_t ch = 0U; ch < 4U; ++ch) {
        uint8_t dataReg = Fdc2214RegForChannelStep2(FDC2214_REG_DATA_MSB_BASE, (Fdc2214CapChannel_t)ch);
        uint16_t msb = 0U;
        uint16_t lsb = 0U;
        esp_err_t err = Fdc2214CapReadReg16(dev, dataReg, &msb);
        if (err != ESP_OK && firstErr == ESP_OK) {
            firstErr = err;
        }
        err = Fdc2214CapReadReg16(dev, (uint8_t)(dataReg + 1U), &lsb);
        if (err != ESP_OK && firstErr == ESP_OK) {
            firstErr = err;
        }
    }
    return firstErr;
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

esp_err_t Fdc2214CapReadClockDividers(Fdc2214CapDevice_t* dev,
                                      Fdc2214CapChannel_t ch,
                                      uint16_t* outClockDividers)
{
    if (!dev || !outClockDividers) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!Fdc2214IsValidChannel(ch)) {
        return ESP_ERR_INVALID_ARG;
    }
    return Fdc2214CapReadReg16(dev, Fdc2214RegForChannelStep1(FDC2214_REG_CLOCK_DIVIDERS_BASE, ch), outClockDividers);
}

esp_err_t Fdc2214CapWriteDriveCurrent(Fdc2214CapDevice_t* dev,
                                      Fdc2214CapChannel_t ch,
                                      uint16_t driveCurrent)
{
    if (!dev) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!Fdc2214IsValidChannel(ch)) {
        return ESP_ERR_INVALID_ARG;
    }

    bool maskedMatch = true;
    uint16_t readback = 0U;
    uint16_t normalized = Fdc2214CapNormalizeDriveCurrent(driveCurrent);
    esp_err_t err = Fdc2214CapWriteReg16VerifyWithMask(dev,
                                                       Fdc2214RegForChannelStep1(FDC2214_REG_DRIVE_CURRENT_BASE, ch),
                                                       normalized,
                                                       FDC2214_DRIVE_CURRENT_MASK,
                                                       true,
                                                       "DRIVE_CURRENT",
                                                       &maskedMatch,
                                                       &readback);
    if (err != ESP_OK) {
        return err;
    }
    return maskedMatch ? ESP_OK : ESP_ERR_INVALID_RESPONSE;
}

esp_err_t Fdc2214CapReadDriveCurrent(Fdc2214CapDevice_t* dev,
                                     Fdc2214CapChannel_t ch,
                                     uint16_t* outDriveCurrent)
{
    if (!dev || !outDriveCurrent) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!Fdc2214IsValidChannel(ch)) {
        return ESP_ERR_INVALID_ARG;
    }

    uint16_t raw = 0U;
    esp_err_t err = Fdc2214CapReadReg16(dev, Fdc2214RegForChannelStep1(FDC2214_REG_DRIVE_CURRENT_BASE, ch), &raw);
    if (err != ESP_OK) {
        return err;
    }
    *outDriveCurrent = Fdc2214CapNormalizeDriveCurrent(raw);
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
    };

    FDCLOW_TRACE("FDCLOW,stage=read_status_begin\n");
    esp_err_t err = Fdc2214CapReadReg16(dev, FDC2214_REG_STATUS, &snapshot.Status);
    FDCLOW_TRACE("FDCLOW,stage=read_status_done,err=%ld,status=0x%04X\n",
                 (long)err,
                 snapshot.Status);
    if (err != ESP_OK) {
        return err;
    }
    FDCLOW_TRACE("FDCLOW,stage=read_status_config_begin\n");
    err = Fdc2214CapReadReg16(dev, FDC2214_REG_STATUS_CONFIG, &snapshot.StatusConfig);
    FDCLOW_TRACE("FDCLOW,stage=read_status_config_done,err=%ld,statusConfig=0x%04X\n",
                 (long)err,
                 snapshot.StatusConfig);
    if (err != ESP_OK) {
        return err;
    }
    FDCLOW_TRACE("FDCLOW,stage=read_config_begin\n");
    err = Fdc2214CapReadReg16(dev, FDC2214_REG_CONFIG, &snapshot.Config);
    FDCLOW_TRACE("FDCLOW,stage=read_config_done,err=%ld,config=0x%04X\n",
                 (long)err,
                 snapshot.Config);
    if (err != ESP_OK) {
        return err;
    }
    FDCLOW_TRACE("FDCLOW,stage=read_mux_begin\n");
    err = Fdc2214CapReadReg16(dev, FDC2214_REG_MUX_CONFIG, &snapshot.MuxConfig);
    FDCLOW_TRACE("FDCLOW,stage=read_mux_done,err=%ld,mux=0x%04X\n",
                 (long)err,
                 snapshot.MuxConfig);
    if (err != ESP_OK) {
        return err;
    }
    err = Fdc2214CapReadReg16(dev, Fdc2214RegForChannelStep1(FDC2214_REG_RCOUNT_BASE, FDC2214_CH0), &snapshot.RcountCh0);
    if (err != ESP_OK) {
        return err;
    }
    err = Fdc2214CapReadReg16(dev,
                              Fdc2214RegForChannelStep1(FDC2214_REG_SETTLECOUNT_BASE, FDC2214_CH0),
                              &snapshot.SettleCountCh0);
    if (err != ESP_OK) {
        return err;
    }
    FDCLOW_TRACE("FDCLOW,stage=read_clock_begin\n");
    err = Fdc2214CapReadReg16(dev,
                              Fdc2214RegForChannelStep1(FDC2214_REG_CLOCK_DIVIDERS_BASE, FDC2214_CH0),
                              &snapshot.ClockDividersCh0);
    FDCLOW_TRACE("FDCLOW,stage=read_clock_done,err=%ld,clock=0x%04X\n",
                 (long)err,
                 snapshot.ClockDividersCh0);
    if (err != ESP_OK) {
        return err;
    }
    err = Fdc2214CapReadReg16(dev,
                              Fdc2214RegForChannelStep1(FDC2214_REG_DRIVE_CURRENT_BASE, FDC2214_CH0),
                              &snapshot.DriveCurrentCh0);
    if (err != ESP_OK) {
        return err;
    }

    uint8_t dataReg = Fdc2214RegForChannelStep2(FDC2214_REG_DATA_MSB_BASE, dataChannel);
    FDCLOW_TRACE("FDCLOW,stage=read_data_begin,channel=%u,reg=0x%02X\n",
                 (unsigned)dataChannel,
                 dataReg);
    err = Fdc2214CapReadReg16(dev, dataReg, &snapshot.DataMsb);
    if (err != ESP_OK) {
        FDCLOW_TRACE("FDCLOW,stage=read_data_done,err=%ld,raw=%lu,msb=0x%04X,lsb=0x%04X\n",
                     (long)err,
                     0ul,
                     snapshot.DataMsb,
                     snapshot.DataLsb);
        return err;
    }
    err = Fdc2214CapReadReg16(dev, (uint8_t)(dataReg + 1u), &snapshot.DataLsb);
    if (err != ESP_OK) {
        FDCLOW_TRACE("FDCLOW,stage=read_data_done,err=%ld,raw=%lu,msb=0x%04X,lsb=0x%04X\n",
                     (long)err,
                     0ul,
                     snapshot.DataMsb,
                     snapshot.DataLsb);
        return err;
    }

    Fdc2214CapStatus_t statusDecoded = {0};
    Fdc2214CapDecodeStatusRaw(snapshot.Status, &statusDecoded);

    snapshot.DataRaw28 = ((uint32_t)(snapshot.DataMsb & FDC2214_DATA_MSB_MASK) << 16) | snapshot.DataLsb;
    FDCLOW_TRACE("FDCLOW,stage=read_data_done,err=%ld,raw=%lu,msb=0x%04X,lsb=0x%04X\n",
                 (long)ESP_OK,
                 (unsigned long)snapshot.DataRaw28,
                 snapshot.DataMsb,
                 snapshot.DataLsb);
    snapshot.DataErrWatchdog = (snapshot.DataMsb & FDC2214_DATA_ERR_WD_MASK) != 0U;
    snapshot.DataErrAmplitude = (snapshot.DataMsb & FDC2214_DATA_ERR_AW_MASK) != 0U;
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

    *outSnapshot = snapshot;
    return ESP_OK;
}

esp_err_t Fdc2214CapConfigureChannelWriteOnly(Fdc2214CapDevice_t* dev,
                                              Fdc2214CapChannel_t ch,
                                              const Fdc2214CapChannelConfig_t* cfg)
{
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
    if (driveCurrent != cfg->DriveCurrent) {
        ESP_LOGW(TAG, "Drive current masked to 0x%04X to clear reserved bits", driveCurrent);
    }

    err = Fdc2214CapWriteReg16(dev, Fdc2214RegForChannelStep1(FDC2214_REG_RCOUNT_BASE, ch), cfg->Rcount);
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
    err = Fdc2214CapWriteReg16(dev, Fdc2214RegForChannelStep1(FDC2214_REG_CLOCK_DIVIDERS_BASE, ch), clockDividers);
    if (err != ESP_OK) {
        return err;
    }
    return Fdc2214CapWriteReg16(dev,
                                Fdc2214RegForChannelStep1(FDC2214_REG_DRIVE_CURRENT_BASE, ch),
                                driveCurrent);
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
    if (driveCurrent != cfg->DriveCurrent) {
        ESP_LOGW(TAG, "Drive current masked to 0x%04X to clear reserved bits", driveCurrent);
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

#if CONFIG_SENSORARRAY_FDC_VERBOSE_SCAN_LOG
    ESP_LOGI(TAG,
             "Configured CH%d rcount=0x%04X settle=0x%04X offset=0x%04X clock=0x%04X drive=0x%04X",
             (int)ch,
             cfg->Rcount,
             cfg->SettleCount,
             cfg->Offset,
             clockDividers,
             driveCurrent);
#endif
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
        err = Fdc2214CapReadReg16Verify(dev, verifyItems[i].reg, &readback);
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
    err = Fdc2214CapReadReg16Verify(dev, Fdc2214RegForChannelStep1(FDC2214_REG_DRIVE_CURRENT_BASE, ch), &driveReadback);
    if (err != ESP_OK) {
        return err;
    }
    if (outDriveCurrentReadback) {
        *outDriveCurrentReadback = driveReadback;
    }

    uint16_t expectedDriveMasked = (uint16_t)(expectedDriveCurrent & FDC2214_DRIVE_CURRENT_MASK);
    uint16_t readDriveMasked = (uint16_t)(driveReadback & FDC2214_DRIVE_CURRENT_MASK);
    if (readDriveMasked != expectedDriveMasked) {
        ESP_LOGW(TAG,
                 "CH%d DRIVE_CURRENT IDRIVE mismatch reg 0x%02X expected(masked)=0x%04X got(masked)=0x%04X raw=0x%04X",
                 (int)ch,
                 Fdc2214RegForChannelStep1(FDC2214_REG_DRIVE_CURRENT_BASE, ch),
                 expectedDriveMasked,
                 readDriveMasked,
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

esp_err_t Fdc2214CapSetAutoScanModeWriteOnly(Fdc2214CapDevice_t* dev,
                                             uint8_t rrSequence,
                                             Fdc2214CapDeglitch_t deglitch,
                                             uint16_t configTemplate,
                                             uint16_t* outConfig,
                                             uint16_t* outMuxConfig)
{
    if (!dev) {
        return ESP_ERR_INVALID_ARG;
    }
    if (rrSequence > FDC2214_RR_SEQUENCE_CH0_CH1_CH2_CH3) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!Fdc2214IsValidDeglitch(deglitch)) {
        return ESP_ERR_INVALID_ARG;
    }

    Fdc2214CapConfigOptions_t options =
        Fdc2214CapConfigOptionsFromRaw(Fdc2214CapApplyConfigReservedBits(configTemplate));
    options.ActiveChannel = FDC2214_CH0;
    options.SleepModeEnabled = false;
    options.HighCurrentDrive = false;
    uint16_t newConfig = Fdc2214CapBuildConfig(&options);

    uint16_t muxValue = 0U;
    esp_err_t err = Fdc2214CapBuildMuxValue(true, rrSequence, deglitch, &muxValue);
    if (err != ESP_OK) {
        return err;
    }

    err = Fdc2214CapWriteReg16(dev, FDC2214_REG_CONFIG, newConfig);
    if (err != ESP_OK) {
        return err;
    }
    err = Fdc2214CapWriteReg16(dev, FDC2214_REG_MUX_CONFIG, muxValue);
    if (err != ESP_OK) {
        return err;
    }

    if (outConfig) {
        *outConfig = newConfig;
    }
    if (outMuxConfig) {
        *outMuxConfig = muxValue;
    }
    return ESP_OK;
}

esp_err_t Fdc2214CapSetAutoScanMode(Fdc2214CapDevice_t* dev, uint8_t rrSequence, Fdc2214CapDeglitch_t deglitch)
{
    if (!dev) {
        return ESP_ERR_INVALID_ARG;
    }
    if (rrSequence > FDC2214_RR_SEQUENCE_CH0_CH1_CH2_CH3) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!Fdc2214IsValidDeglitch(deglitch)) {
        return ESP_ERR_INVALID_ARG;
    }

    uint16_t configReg = 0U;
    esp_err_t err = Fdc2214CapReadReg16(dev, FDC2214_REG_CONFIG, &configReg);
    if (err != ESP_OK) {
        return err;
    }

    // Keep ref-clock/INTB policy but force active conversion state. ACTIVE_CHAN
    // is ignored by the runtime matrix path while AUTOSCAN_EN is asserted.
    Fdc2214CapConfigOptions_t options = Fdc2214CapConfigOptionsFromRaw(configReg);
    options.ActiveChannel = FDC2214_CH0;
    options.SleepModeEnabled = false;
    options.HighCurrentDrive = false;
    uint16_t newConfig = Fdc2214CapBuildConfig(&options);
    err = Fdc2214CapWriteReg16Verify(dev, FDC2214_REG_CONFIG, newConfig);
    if (err != ESP_OK) {
        return err;
    }

    err = Fdc2214CapSetMuxConfig(dev, true, rrSequence, deglitch);
    if (err != ESP_OK) {
        return err;
    }

    Fdc2214CapCoreRegs_t regs = {0};
    err = Fdc2214CapReadCoreRegs(dev, &regs);
    if (err != ESP_OK) {
        return err;
    }

    uint8_t muxRr = (uint8_t)((regs.MuxConfig & FDC2214_MUX_RR_SEQUENCE_MASK) >>
                              FDC2214_MUX_RR_SEQUENCE_SHIFT);
    uint8_t muxDeglitch = (uint8_t)(regs.MuxConfig & FDC2214_MUX_DEGLITCH_MASK);
    bool autoscan = (regs.MuxConfig & FDC2214_MUX_AUTOSCAN_BIT) != 0u;
    bool highCurrent = (regs.Config & FDC2214_CONFIG_HIGH_CURRENT_DRV_MASK) != 0u;
#if CONFIG_SENSORARRAY_FDC_VERBOSE_SCAN_LOG
    printf("FDC_AUTOSCAN_CONFIG,device=addr0x%02X,mux=0x%04X,config=0x%04X,autoscan=%u,rr=%u,rrName=%s,deglitch=0x%X,highCurrent=%u\n",
           dev->bus.I2cAddress7,
           regs.MuxConfig,
           regs.Config,
           autoscan ? 1u : 0u,
           (unsigned)muxRr,
           Fdc2214CapRrSequenceName(muxRr),
           (unsigned)muxDeglitch,
           highCurrent ? 1u : 0u);
#endif

    if (!autoscan || muxRr != rrSequence || muxDeglitch != (uint8_t)deglitch || highCurrent) {
        return ESP_ERR_INVALID_RESPONSE;
    }

#if CONFIG_SENSORARRAY_FDC_VERBOSE_SCAN_LOG
    ESP_LOGI(TAG, "Autoscan mode set, rrSequence=%u", (unsigned)rrSequence);
#endif
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

    Fdc2214CapStatus_t statusDecoded = {0};
    (void)Fdc2214CapDecodeStatusRaw(snapshot.Status, &statusDecoded);

    bool unreadConversion = Fdc2214CapStatusHasUnreadForChannel(&statusDecoded, ch);
    bool dataReady = statusDecoded.DataReady || snapshot.DataReady;
    bool readable = unreadConversion || dataReady;
    bool statusWatchdog = Fdc2214CapStatusHasWatchdogFault(&statusDecoded);
    bool statusAmplitude = Fdc2214CapStatusHasAmplitudeFault(&statusDecoded);
    bool dataWatchdog = snapshot.DataErrWatchdog;
    bool dataAmplitude = snapshot.DataErrAmplitude;
    bool watchdogFault = dataWatchdog || statusWatchdog;
    bool amplitudeFault = dataAmplitude || statusAmplitude;
    bool rawNonZero = snapshot.DataRaw28 != 0U;
    bool saturated = snapshot.DataRaw28 >= FDC2214_RAW28_SATURATED_THRESHOLD;
    bool configKnown = Fdc2214CapConfigReservedBitsValid(snapshot.Config) &&
                       ((snapshot.MuxConfig & FDC2214_MUX_FIXED_MASK) == FDC2214_MUX_FIXED_MASK);

    outSample->Raw28 = snapshot.DataRaw28;
    outSample->ErrWatchdog = dataWatchdog;
    outSample->ErrAmplitude = dataAmplitude;
    outSample->StatusRaw = snapshot.Status;
    outSample->ConfigRaw = snapshot.Config;
    outSample->MuxRaw = snapshot.MuxConfig;
    outSample->SleepModeEnabled = snapshot.SleepModeEnabled;
    outSample->AutoScanEnabled = snapshot.AutoScanEnabled;
    outSample->Converting = snapshot.Converting;
    outSample->UnreadConversionPresent = unreadConversion;
    outSample->DataReady = dataReady;
    outSample->ActiveChannel = snapshot.ActiveChannel;
    outSample->RefClockSource = Fdc2214CapRefClockFromConfig(snapshot.Config);

    Fdc2214CapSampleStatus_t semanticStatus = FDC2214_SAMPLE_STATUS_CONFIG_UNKNOWN;
    if (!configKnown) {
        semanticStatus = FDC2214_SAMPLE_STATUS_CONFIG_UNKNOWN;
    } else if (snapshot.SleepModeEnabled) {
        semanticStatus = FDC2214_SAMPLE_STATUS_STILL_SLEEPING;
    } else if (!snapshot.Converting) {
        semanticStatus = FDC2214_SAMPLE_STATUS_I2C_READ_OK_BUT_NOT_CONVERTING;
    } else if (watchdogFault) {
        semanticStatus = FDC2214_SAMPLE_STATUS_WATCHDOG_FAULT;
    } else if (amplitudeFault) {
        semanticStatus = FDC2214_SAMPLE_STATUS_AMPLITUDE_FAULT;
    } else if (!rawNonZero) {
        // An all-zero payload can be stale data from a non-converting/sleeping path.
        // Do not auto-promote transport success to semantic sample validity.
        semanticStatus = FDC2214_SAMPLE_STATUS_ZERO_RAW_INVALID;
    } else if (!readable) {
        semanticStatus = FDC2214_SAMPLE_STATUS_NO_UNREAD_CONVERSION;
    } else {
        semanticStatus = FDC2214_SAMPLE_STATUS_SAMPLE_VALID;
    }

    outSample->SampleStatus = semanticStatus;
    if (relaxedValidity) {
        outSample->SampleValid = snapshot.Converting &&
                                 rawNonZero &&
                                 !watchdogFault &&
                                 readable;
    } else {
        outSample->SampleValid = configKnown &&
                                 snapshot.Converting &&
                                 rawNonZero &&
                                 !saturated &&
                                 !watchdogFault &&
                                 readable &&
                                 (semanticStatus == FDC2214_SAMPLE_STATUS_SAMPLE_VALID ||
                                  semanticStatus == FDC2214_SAMPLE_STATUS_AMPLITUDE_FAULT);
    }

    if (snapshot.Status == FDC2214CAP_STATUS_CH0_UNREAD_MASK && ch == FDC2214_CH0 &&
        (!outSample->UnreadConversionPresent ||
         outSample->ErrAmplitude ||
         statusAmplitude ||
         outSample->SampleStatus == FDC2214_SAMPLE_STATUS_AMPLITUDE_FAULT)) {
        printf("FDC_STATUS_REGRESSION,status=0x%04X,ch=%u,unread=%u,errAmplitude=%u,statusAmplitude=%u,sampleStatus=%d\n",
               snapshot.Status,
               (unsigned)ch,
               outSample->UnreadConversionPresent ? 1u : 0u,
               outSample->ErrAmplitude ? 1u : 0u,
               statusAmplitude ? 1u : 0u,
               (int)outSample->SampleStatus);
    }
    return ESP_OK;
}

esp_err_t Fdc2214CapReadSample(Fdc2214CapDevice_t* dev, Fdc2214CapChannel_t ch, Fdc2214CapSample_t* outSample)
{
    FDCLOW_TRACE("FDCLOW,op=read_sample_begin,channel=%u,relaxed=0\n", (unsigned)ch);
    esp_err_t err = Fdc2214CapReadSampleWithValidityMode(dev, ch, false, outSample);
    FDCLOW_TRACE("FDCLOW,op=read_sample_done,channel=%u,err=%ld,raw=%lu,status=%u\n",
                 (unsigned)ch,
                 (long)err,
                 outSample ? (unsigned long)outSample->Raw28 : 0ul,
                 outSample ? (unsigned)outSample->SampleStatus : 0u);
    return err;
}

esp_err_t Fdc2214CapReadSampleRelaxed(Fdc2214CapDevice_t* dev,
                                      Fdc2214CapChannel_t ch,
                                      Fdc2214CapSample_t* outSample)
{
    FDCLOW_TRACE("FDCLOW,op=read_sample_begin,channel=%u,relaxed=1\n", (unsigned)ch);
    esp_err_t err = Fdc2214CapReadSampleWithValidityMode(dev, ch, true, outSample);
    FDCLOW_TRACE("FDCLOW,op=read_sample_done,channel=%u,err=%ld,raw=%lu,status=%u\n",
                 (unsigned)ch,
                 (long)err,
                 outSample ? (unsigned long)outSample->Raw28 : 0ul,
                 outSample ? (unsigned)outSample->SampleStatus : 0u);
    return err;
}

esp_err_t Fdc2214CapReadChannelRawWithStatus(Fdc2214CapDevice_t* dev,
                                             Fdc2214CapChannel_t ch,
                                             Fdc2214CapSample_t* outSample)
{
    return Fdc2214CapReadSampleWithValidityMode(dev, ch, false, outSample);
}

esp_err_t Fdc2214CapReadChannelsDataRegsFast(Fdc2214CapDevice_t* dev,
                                             uint8_t channelMask,
                                             Fdc2214CapFastChannelSample_t* outSamples,
                                             size_t outSampleCount)
{
    if (!dev || !outSamples) {
        return ESP_ERR_INVALID_ARG;
    }
    if ((channelMask & 0xF0u) != 0u) {
        return ESP_ERR_INVALID_ARG;
    }
    if (outSampleCount < 4u) {
        return ESP_ERR_INVALID_SIZE;
    }

    for (size_t i = 0u; i < 4u; ++i) {
        outSamples[i] = (Fdc2214CapFastChannelSample_t){0};
    }

    uint16_t statusRaw = 0u;
    esp_err_t firstErr = Fdc2214CapReadReg16(dev, FDC2214_REG_STATUS, &statusRaw);
    Fdc2214CapStatus_t status = {0};
    if (firstErr == ESP_OK) {
        Fdc2214CapDecodeStatusRaw(statusRaw, &status);
    }

    for (uint8_t ch = 0u; ch < 4u; ++ch) {
        Fdc2214CapFastChannelSample_t* sample = &outSamples[ch];
        sample->statusRaw = statusRaw;
        sample->dataReady = (firstErr == ESP_OK) && status.DataReady;
        sample->unreadConversion = (firstErr == ESP_OK) && status.UnreadConversion[ch];
        sample->sampleStatus = FDC2214_SAMPLE_STATUS_CONFIG_UNKNOWN;
        if ((channelMask & (uint8_t)(1u << ch)) == 0u) {
            continue;
        }
        if (firstErr != ESP_OK) {
            sample->errorMask |= FDC2214CAP_FAST_ERROR_I2C;
            continue;
        }

        uint8_t dataReg = Fdc2214RegForChannelStep2(FDC2214_REG_DATA_MSB_BASE, (Fdc2214CapChannel_t)ch);
        esp_err_t err = Fdc2214CapReadReg16(dev, dataReg, &sample->dataMsb);
        if (err == ESP_OK) {
            err = Fdc2214CapReadReg16(dev, (uint8_t)(dataReg + 1u), &sample->dataLsb);
        }
        if (err != ESP_OK) {
            sample->errorMask |= FDC2214CAP_FAST_ERROR_I2C;
            if (firstErr == ESP_OK) {
                firstErr = err;
            }
            continue;
        }

        sample->raw28 = ((uint32_t)(sample->dataMsb & FDC2214_DATA_MSB_MASK) << 16) |
                        (uint32_t)sample->dataLsb;
        sample->errWatchdog = (sample->dataMsb & FDC2214_DATA_ERR_WD_MASK) != 0u;
        sample->errAmplitude = (sample->dataMsb & FDC2214_DATA_ERR_AW_MASK) != 0u;

        bool statusFaultForChannel = status.ErrorChannel == ch &&
                                     (status.ErrWatchdog ||
                                      status.ErrAmplitudeHigh ||
                                      status.ErrAmplitudeLow);
        if (sample->errWatchdog || (status.ErrorChannel == ch && status.ErrWatchdog)) {
            sample->errorMask |= FDC2214CAP_FAST_ERROR_WATCHDOG;
        }
        if (sample->errAmplitude ||
            (status.ErrorChannel == ch && (status.ErrAmplitudeHigh || status.ErrAmplitudeLow))) {
            sample->errorMask |= FDC2214CAP_FAST_ERROR_AMPLITUDE;
        }
        if (statusFaultForChannel) {
            sample->errorMask |= FDC2214CAP_FAST_ERROR_STATUS_FAULT;
        }
        if (!sample->unreadConversion) {
            sample->errorMask |= FDC2214CAP_FAST_ERROR_NO_UNREAD;
        }
        if (sample->raw28 == 0u) {
            sample->errorMask |= FDC2214CAP_FAST_ERROR_ZERO_RAW;
        }

        if ((sample->errorMask & FDC2214CAP_FAST_ERROR_I2C) != 0u) {
            sample->sampleStatus = FDC2214_SAMPLE_STATUS_CONFIG_UNKNOWN;
        } else if ((sample->errorMask & FDC2214CAP_FAST_ERROR_WATCHDOG) != 0u) {
            sample->sampleStatus = FDC2214_SAMPLE_STATUS_WATCHDOG_FAULT;
        } else if (sample->raw28 == 0u) {
            sample->sampleStatus = FDC2214_SAMPLE_STATUS_ZERO_RAW_INVALID;
        } else {
            sample->sampleStatus = FDC2214_SAMPLE_STATUS_SAMPLE_VALID;
        }

        /*
         * UnreadConversion and amplitude are diagnostics in the fast matrix path.
         * A readable non-zero payload with no watchdog fault remains usable, and
         * the caller can still inspect errorMask for stale/unread/amplitude suspicion.
         */
        sample->valid = (sample->raw28 != 0u) &&
                        ((sample->errorMask & (FDC2214CAP_FAST_ERROR_I2C |
                                               FDC2214CAP_FAST_ERROR_WATCHDOG)) == 0u);
    }

    return firstErr;
}

esp_err_t Fdc2214CapReadAutoscan4RawFast(Fdc2214CapDevice_t* dev,
                                         Fdc2214CapFastChannelSample_t outSamples[4])
{
    return Fdc2214CapReadChannelsDataRegsFast(dev, 0x0Fu, outSamples, 4u);
}

esp_err_t Fdc2214CapReadChannelsRaw(Fdc2214CapDevice_t* dev,
                                    uint8_t channelMask,
                                    Fdc2214CapChannelSample_t* outSamples,
                                    size_t outSampleCount)
{
    if (!dev || !outSamples) {
        return ESP_ERR_INVALID_ARG;
    }
    if ((channelMask & 0xF0u) != 0u) {
        return ESP_ERR_INVALID_ARG;
    }
    if (outSampleCount < 4u) {
        return ESP_ERR_INVALID_SIZE;
    }

    for (size_t i = 0u; i < 4u; ++i) {
        outSamples[i] = (Fdc2214CapChannelSample_t){0};
    }

    Fdc2214CapFastChannelSample_t fastSamples[4] = {0};
    esp_err_t firstErr = Fdc2214CapReadChannelsDataRegsFast(dev, channelMask, fastSamples, 4u);
    for (uint8_t ch = 0u; ch < 4u; ++ch) {
        if ((channelMask & (uint8_t)(1u << ch)) == 0u) {
            continue;
        }
        outSamples[ch].raw28 = fastSamples[ch].raw28;
        outSamples[ch].status = fastSamples[ch].statusRaw;
        outSamples[ch].valid = fastSamples[ch].valid;
    }

    return firstErr;
}

esp_err_t Fdc2214CapReadRawRegisters(Fdc2214CapDevice_t* dev, uint8_t reg, uint16_t* outValue)
{
    if (!dev || !outValue) {
        return ESP_ERR_INVALID_ARG;
    }
    *outValue = 0xFFFFu;
    FDCLOW_TRACE("FDCLOW,op=read_raw_begin,reg=0x%02X\n", reg);
    esp_err_t err = Fdc2214CapReadReg16(dev, reg, outValue);
    FDCLOW_TRACE("FDCLOW,op=read_raw_done,reg=0x%02X,err=%ld,value=0x%04X\n",
                 reg,
                 (long)err,
                 outValue ? *outValue : 0u);
    FDC_RAW_TRACE("FDC_RAW,op=read,reg=0x%02X,value=0x%04X,err=%ld\n",
                  reg,
                  *outValue,
                  (long)err);
    return err;
}

esp_err_t Fdc2214CapWriteRawRegisters(Fdc2214CapDevice_t* dev, uint8_t reg, uint16_t value)
{
    if (!dev) {
        return ESP_ERR_INVALID_ARG;
    }
    FDCLOW_TRACE("FDCLOW,op=write_raw_begin,reg=0x%02X,value=0x%04X\n", reg, value);
    esp_err_t err = Fdc2214CapWriteReg16(dev, reg, value);
    FDCLOW_TRACE("FDCLOW,op=write_raw_done,reg=0x%02X,err=%ld\n", reg, (long)err);
    FDC_RAW_TRACE("FDC_RAW,op=write,reg=0x%02X,value=0x%04X,err=%ld\n",
                  reg,
                  value,
                  (long)err);
    return err;
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

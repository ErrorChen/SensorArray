#include "sensorarrayFdcSweep.h"

#include <limits.h>
#include <stdio.h>
#include <string.h>

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "sensorarrayConfig.h"
#include "sensorarrayBoardMap.h"
#include "sensorarrayMeasure.h"
#include "tmuxSwitch.h"

#define SENSORARRAY_FDC_SWEEP_DRIVE_CURRENT_MASK 0xF800u
#define SENSORARRAY_FDC_SWEEP_CONFIG_ACTIVE_CHAN_SHIFT 14u
#define SENSORARRAY_FDC_SWEEP_CONFIG_ACTIVE_CHAN_MASK 0xC000u
#define SENSORARRAY_FDC_SWEEP_CONFIG_SLEEP_MODE_EN_MASK 0x2000u
#define SENSORARRAY_FDC_SWEEP_CONFIG_HIGH_CURRENT_DRV_MASK 0x0040u
#define SENSORARRAY_FDC_SWEEP_MUX_CONFIG_AUTOSCAN_MASK 0x8000u
#define SENSORARRAY_FDC_SWEEP_MUX_CONFIG_DEGLITCH_MASK 0x0007u
#define SENSORARRAY_FDC_SWEEP_REG_STATUS 0x18u
#define SENSORARRAY_FDC_SWEEP_REG_STATUS_CONFIG 0x19u
#define SENSORARRAY_FDC_SWEEP_REG_CONFIG 0x1Au
#define SENSORARRAY_FDC_SWEEP_REG_MUX_CONFIG 0x1Bu
#define SENSORARRAY_FDC_SWEEP_REG_RCOUNT_BASE 0x08u
#define SENSORARRAY_FDC_SWEEP_REG_SETTLECOUNT_BASE 0x10u
#define SENSORARRAY_FDC_SWEEP_REG_CLOCK_DIVIDERS_BASE 0x14u
#define SENSORARRAY_FDC_SWEEP_REG_DRIVE_CURRENT_BASE 0x1Eu
#define SENSORARRAY_FDC_SWEEP_REG_MANUFACTURER_ID 0x7Eu
#define SENSORARRAY_FDC_SWEEP_REG_DEVICE_ID 0x7Fu
#define SENSORARRAY_FDC_SWEEP_STATUS_ERR_WD_MASK FDC2214CAP_STATUS_ERR_WD_MASK
#define SENSORARRAY_FDC_SWEEP_STATUS_ERR_AHW_MASK FDC2214CAP_STATUS_ERR_AHW_MASK
#define SENSORARRAY_FDC_SWEEP_STATUS_ERR_ALW_MASK FDC2214CAP_STATUS_ERR_ALW_MASK
#define SENSORARRAY_FDC_SWEEP_STATUS_DRDY_MASK 0x0040u
#define SENSORARRAY_FDC_SWEEP_RAW28_MAX 0x0FFFFFFFu
#define SENSORARRAY_FDC_SWEEP_RAW28_SATURATED_THRESHOLD 0x0FFFFF00u
#define SENSORARRAY_FDC_SWEEP_FAST_PROBE_SAMPLES 2u
#define SENSORARRAY_FDC_SWEEP_CONFIRM_SAMPLES 4u
#define SENSORARRAY_FDC_SWEEP_MAX_SAMPLES_PER_CANDIDATE \
    (SENSORARRAY_FDC_SWEEP_FAST_PROBE_SAMPLES + SENSORARRAY_FDC_SWEEP_CONFIRM_SAMPLES)
#define SENSORARRAY_FDC_SWEEP_CACHE_FREQ_MARGIN_MIN_HZ 150000.0
#define SENSORARRAY_FDC_SWEEP_CACHE_FREQ_MARGIN_RATIO 0.08
#define SENSORARRAY_FDC_SWEEP_DEFAULT_DEGLITCH_REQ FDC2214_DEGLITCH_3P3MHZ
#define SENSORARRAY_FDC_SWEEP_DEFAULT_DEGLITCH_BW_HZ 3300000u
#define SENSORARRAY_FDC_SWEEP_DEFAULT_DRIVE_CURRENT_REQ 0x4000u
#define SENSORARRAY_FDC_SWEEP_STABLE_RELATIVE_SPREAD 0.005
#define SENSORARRAY_FDC_SWEEP_FREQ_JUMP_RATIO 0.15
#define SENSORARRAY_FDC_SWEEP_FREQ_JUMP_ABS_HZ 500000.0
#define SENSORARRAY_FDC_SWEEP_DIRECT_MIN_VALID_SAMPLES 4u
#define SENSORARRAY_FDC_SWEEP_FAST_DRIVE_STEP_SMALL 0x0800u
#define SENSORARRAY_FDC_SWEEP_FAST_DRIVE_STEP_LARGE 0x1000u
#define SENSORARRAY_FDC_SWEEP_DISCARD_AFTER_LOCK 2u
#define SENSORARRAY_FDC_SWEEP_AUTOSCAN_RR_SEQUENCE FDC2214_RR_SEQUENCE_CH0_CH1_CH2_CH3
#define SENSORARRAY_FDC_SWEEP_DEVICE_CHANNELS 4u
#define SENSORARRAY_FDC_SWEEP_ROW_CELLS 8u
#define SENSORARRAY_FDC_SWEEP_AUTOSCAN_READY_MASK 0x0Fu

/*
 * The validated debug-cell table started at 0x7800. The matrix has been observed
 * at about 6 Vpp, so lower IDRIVE candidates are prepended while keeping the
 * fast-probe/confirm selection strategy intact.
 */
static const uint16_t SENSORARRAY_FDC_DRIVE_CURRENT_SWEEP_TABLE[] = {
    0x7800u,
    0x9000u,
    0xA000u,
    0xB000u,
    0xC000u,
    0xD000u,
    0xE000u,
    0xF000u,
    0xF800u,
    0x4000u,
    0x5000u,
    0x6000u,
    0x7000u,
    0x8000u,
};

static const sensorarrayFdcDeglitchCandidate_t SENSORARRAY_FDC_DEGLITCH_SWEEP_TABLE[] = {
    {0x1u, 1000000u, "1MHz"},
    {0x4u, 3300000u, "3.3MHz"},
    {0x5u, 10000000u, "10MHz"},
    {0x7u, 33000000u, "33MHz"},
};

typedef struct {
    uint32_t raw28[SENSORARRAY_FDC_SWEEP_DEVICE_CHANNELS];
    double freqHz[SENSORARRAY_FDC_SWEEP_DEVICE_CHANNELS];
    uint16_t statusRaw;
    uint16_t rCount[SENSORARRAY_FDC_SWEEP_DEVICE_CHANNELS];
    uint16_t settleCount[SENSORARRAY_FDC_SWEEP_DEVICE_CHANNELS];
    uint16_t clockDividers[SENSORARRAY_FDC_SWEEP_DEVICE_CHANNELS];
    uint16_t driveCurrent[SENSORARRAY_FDC_SWEEP_DEVICE_CHANNELS];
    uint32_t effectiveFclkHz;
    uint8_t validMask;
    uint8_t warnMask;
    uint8_t errorMask;
    esp_err_t err;
} sensorarrayFdcSweepDeviceRead4_t;

static sensorarrayFdcCellCalibration_t gFdcCalibrationTable[SENSORARRAY_MATRIX_ROWS][SENSORARRAY_MATRIX_COLS];
static bool gFdcCalibrationTableInitialized = false;
static volatile bool gFdcForceFullSweepAllPending = false;
static volatile bool gFdcForceFullSweepPending = false;
static volatile uint8_t gFdcForceFullSweepS = 1u;
static volatile uint8_t gFdcForceFullSweepD = 1u;
static volatile uint32_t gFdcSweepRequestEpoch = 0u;
static volatile bool gFdcRescueInProgress = false;
static uint32_t gFdcAllInvalidReportCount = 0u;

typedef struct {
    esp_err_t i2cErr;
    bool i2cOk;
    bool sampleValid;
    bool rawNonZero;
    bool statusOk;
    bool sleep;
    bool converting;
    bool watchdogFault;
    bool amplitudeFault;
    bool saturated;
    bool dataReadyOk;
    bool unreadOk;
    bool readableOk;
    uint32_t raw28;
    uint16_t status;
    const char *invalidReason;
} sensorarrayFdcSampleEval_t;

static bool sensorarrayFdcSweepCandidateHasStableSamples(const sensorarrayFdcSweepCandidateResult_t *candidate);

static const char *sensorarrayFdcSweepDevName(sensorarrayFdcDeviceId_t devId)
{
    return (devId == SENSORARRAY_FDC_DEV_SECONDARY) ? "secondary" : "primary";
}

static const char *sensorarrayFdcSweepDevFullName(sensorarrayFdcDeviceId_t devId)
{
    return (devId == SENSORARRAY_FDC_DEV_SECONDARY) ? "secondary_fdc2214" : "primary_fdc2214";
}

static uint8_t sensorarrayFdcSweepDLineForDeviceChannel(sensorarrayFdcDeviceId_t devId,
                                                        Fdc2214CapChannel_t channel)
{
    uint8_t base = (devId == SENSORARRAY_FDC_DEV_SECONDARY) ? 5u : 1u;
    return (uint8_t)(base + (uint8_t)channel);
}

static uint8_t sensorarrayFdcSweepRegForChannel(uint8_t base, Fdc2214CapChannel_t channel)
{
    return (uint8_t)(base + (uint8_t)channel);
}

static double sensorarrayFdcSweepAbsDouble(double value)
{
    return (value < 0.0) ? -value : value;
}

static uint32_t sensorarrayFdcSweepClampQualitySamples(uint32_t requested)
{
    if (requested < SENSORARRAY_FDC_SWEEP_DIRECT_MIN_VALID_SAMPLES) {
        return SENSORARRAY_FDC_SWEEP_DIRECT_MIN_VALID_SAMPLES;
    }
    if (requested > SENSORARRAY_FDC_SWEEP_MAX_SAMPLES_PER_CANDIDATE) {
        return SENSORARRAY_FDC_SWEEP_MAX_SAMPLES_PER_CANDIDATE;
    }
    return requested;
}

static uint16_t sensorarrayFdcSweepClampDrive(uint16_t drive)
{
    uint16_t norm = (uint16_t)(drive & SENSORARRAY_FDC_SWEEP_DRIVE_CURRENT_MASK);
    if (norm == 0u) {
        return SENSORARRAY_FDC_SWEEP_DEFAULT_DRIVE_CURRENT_REQ;
    }
    return norm;
}

static uint16_t sensorarrayFdcSweepAddDriveOffset(uint16_t center, int32_t offset)
{
    int32_t value = (int32_t)(center & SENSORARRAY_FDC_SWEEP_DRIVE_CURRENT_MASK) + offset;
    if (value < (int32_t)SENSORARRAY_FDC_SWEEP_DEFAULT_DRIVE_CURRENT_REQ) {
        value = (int32_t)SENSORARRAY_FDC_SWEEP_DEFAULT_DRIVE_CURRENT_REQ;
    }
    if (value > 0xF800) {
        value = 0xF800;
    }
    return (uint16_t)value;
}

static bool sensorarrayFdcSweepCapPfFromFreq(double freqHz, double *outCapPf)
{
    if (!outCapPf) {
        return false;
    }
    *outCapPf = 0.0;
    return sensorarrayMeasureFdcTryCapacitancePf(freqHz,
                                                 (uint32_t)CONFIG_SENSORARRAY_FDC_TANK_INDUCTOR_UH,
                                                 outCapPf);
}

static sensorarrayFdcCellCalibration_t *sensorarrayFdcSweepCell(uint8_t sIndex, uint8_t dIndex)
{
    if (!sensorarrayMatrixIndexIsValid(sIndex, dIndex)) {
        return NULL;
    }
    return &gFdcCalibrationTable[sIndex - 1u][dIndex - 1u];
}

sensorarrayFdcCellCalibration_t *sensorarrayFdcSweepGetCellCalibration(uint8_t sIndex,
                                                                       uint8_t dIndex)
{
    return sensorarrayFdcSweepCell(sIndex, dIndex);
}

esp_err_t sensorarrayFdcSweepRequestForceFullSweepAll(void)
{
    gFdcForceFullSweepAllPending = true;
    uint32_t epoch = ++gFdcSweepRequestEpoch;
    printf("FDC_SWEEP_REQUEST,scope=all,reason=manual_or_auto,status=queued,epoch=%lu\n",
           (unsigned long)epoch);
    return ESP_OK;
}

bool sensorarrayFdcSweepConsumeForceFullSweepAll(void)
{
    if (!gFdcForceFullSweepAllPending) {
        return false;
    }
    gFdcForceFullSweepAllPending = false;
    return true;
}

esp_err_t sensorarrayFdcSweepRequestForceFullSweepCell(uint8_t sIndex, uint8_t dIndex)
{
    if (!sensorarrayMatrixIndexIsValid(sIndex, dIndex)) {
        printf("FDC_SWEEP_REQUEST,scope=cell,status=failed,reason=bad_cell,s=%u,d=%u\n",
               (unsigned)sIndex,
               (unsigned)dIndex);
        return ESP_ERR_INVALID_ARG;
    }
    gFdcForceFullSweepS = sIndex;
    gFdcForceFullSweepD = dIndex;
    gFdcForceFullSweepPending = true;
    uint32_t epoch = ++gFdcSweepRequestEpoch;
    printf("FDC_SWEEP_REQUEST,scope=cell,s=%u,d=%u,reason=manual_or_auto,status=queued,epoch=%lu\n",
           (unsigned)sIndex,
           (unsigned)dIndex,
           (unsigned long)epoch);
    return ESP_OK;
}

static bool sensorarrayFdcSweepConsumeForceFullSweep(uint8_t sIndex, uint8_t dIndex)
{
    if (!gFdcForceFullSweepPending ||
        gFdcForceFullSweepS != sIndex ||
        gFdcForceFullSweepD != dIndex) {
        return false;
    }
    gFdcForceFullSweepPending = false;
    return true;
}

static void sensorarrayFdcSweepInitCalibrationCell(uint8_t sIndex, uint8_t dIndex)
{
    sensorarrayFdcCellCalibration_t *cal = sensorarrayFdcSweepCell(sIndex, dIndex);
    const sensorarrayFdcDLineMap_t *map = sensorarrayBoardMapFindFdcByDLine(dIndex);
    if (!cal || !map) {
        return;
    }

    *cal = (sensorarrayFdcCellCalibration_t){
        .sIndex = sIndex,
        .dIndex = dIndex,
        .fdcDevice = (uint8_t)map->devId,
        .fdcChannel = (uint8_t)map->channel,
        .bootDriveCurrent = SENSORARRAY_FDC_SWEEP_DEFAULT_DRIVE_CURRENT_REQ,
        .bootDeglitch = FDC2214_DEGLITCH_10MHZ,
        .lastGoodDriveCurrent = SENSORARRAY_FDC_SWEEP_DEFAULT_DRIVE_CURRENT_REQ,
        .lastGoodDeglitch = FDC2214_DEGLITCH_10MHZ,
        .lastFailReason = "not_measured",
    };
}

static void sensorarrayFdcSweepInitCalibrationTable(void)
{
    for (uint8_t s = 1u; s <= SENSORARRAY_MATRIX_ROWS; ++s) {
        for (uint8_t d = 1u; d <= SENSORARRAY_MATRIX_COLS; ++d) {
            sensorarrayFdcSweepInitCalibrationCell(s, d);
        }
    }
    gFdcCalibrationTableInitialized = true;
}

static void sensorarrayFdcSweepEnsureCalibrationTable(void)
{
    if (!gFdcCalibrationTableInitialized) {
        sensorarrayFdcSweepInitCalibrationTable();
    }
}

const char *sensorarrayFdcSweepReasonName(sensorarrayFdcSweepReason_t reason)
{
    switch (reason) {
    case SENSORARRAY_FDC_SWEEP_REASON_BOOT:
        return "boot";
    case SENSORARRAY_FDC_SWEEP_REASON_QUICK:
        return "quick";
    case SENSORARRAY_FDC_SWEEP_REASON_AMPLITUDE_FAULT:
        return "amplitude_fault";
    case SENSORARRAY_FDC_SWEEP_REASON_WATCHDOG_FAULT:
        return "watchdog_fault";
    case SENSORARRAY_FDC_SWEEP_REASON_INVALID_STREAK:
        return "invalid_streak";
    case SENSORARRAY_FDC_SWEEP_REASON_RAW_ZERO:
        return "raw_zero";
    case SENSORARRAY_FDC_SWEEP_REASON_SATURATED:
        return "saturated";
    case SENSORARRAY_FDC_SWEEP_REASON_FREQUENCY_MARGIN:
        return "frequency_margin";
    case SENSORARRAY_FDC_SWEEP_REASON_DEGLITCH_MARGIN:
        return "deglitch_margin";
    default:
        return "unknown";
    }
}

const char *sensorarrayFdcSweepFaultReasonName(sensorarrayFdcFaultReason_t reason)
{
    switch (reason) {
    case SENSORARRAY_FDC_FAULT_NONE:
        return "none";
    case SENSORARRAY_FDC_FAULT_ZERO_RAW:
        return "zero_raw";
    case SENSORARRAY_FDC_FAULT_NO_OSCILLATION:
        return "no_oscillation";
    case SENSORARRAY_FDC_FAULT_STATUS_SLEEP:
        return "status_sleep";
    case SENSORARRAY_FDC_FAULT_NOT_CONVERTING:
        return "not_converting";
    case SENSORARRAY_FDC_FAULT_WATCHDOG:
        return "watchdog";
    case SENSORARRAY_FDC_FAULT_AMPLITUDE:
        return "amplitude";
    case SENSORARRAY_FDC_FAULT_TIMEOUT:
        return "timeout";
    case SENSORARRAY_FDC_FAULT_ALL_INVALID_FRAME:
        return "all_invalid_frame";
    default:
        return "unknown";
    }
}

bool sensorarrayFdcSweepIsRescueInProgress(void)
{
    return gFdcRescueInProgress;
}

void sensorarrayFdcSweepReportAllInvalidFrame(uint64_t validMask,
                                              uint64_t errorMask,
                                              uint32_t zeroRawCount)
{
    gFdcAllInvalidReportCount++;
    const char *reason =
        (zeroRawCount >= SENSORARRAY_MATRIX_CELL_COUNT) ?
        "persistent_all_rows_hardware_zero" :
        "normal_path_invalid_after_boot_ok";
    printf("FDC_SWEEP_REQUEST,scope=all,reason=%s,status=reported_no_queue,epoch=%lu,validMask=0x%016llX,errorMask=0x%016llX,zeroRaw=%lu,reports=%lu\n",
           reason,
           (unsigned long)gFdcSweepRequestEpoch,
           (unsigned long long)validMask,
           (unsigned long long)errorMask,
           (unsigned long)zeroRawCount,
           (unsigned long)gFdcAllInvalidReportCount);
}

static uint32_t sensorarrayFdcSweepNowMs(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000LL);
}

static uint32_t sensorarrayFdcSweepStepTimeoutMs(void)
{
    uint32_t timeoutMs = (uint32_t)CONFIG_SENSORARRAY_FDC_SWEEP_STEP_TIMEOUT_MS;
    return (timeoutMs == 0u) ? 1u : timeoutMs;
}

static uint32_t sensorarrayFdcSweepTotalTimeoutMs(void)
{
    uint32_t timeoutMs = (uint32_t)CONFIG_SENSORARRAY_FDC_SWEEP_TOTAL_TIMEOUT_MS;
    return (timeoutMs == 0u) ? 1u : timeoutMs;
}

static uint32_t sensorarrayFdcSweepRemainingMsUntil(int64_t deadlineUs)
{
    int64_t remainingUs = deadlineUs - esp_timer_get_time();
    if (remainingUs <= 0) {
        return 0u;
    }
    return (uint32_t)((remainingUs + 999LL) / 1000LL);
}

static TickType_t sensorarrayFdcSweepMsToTicksAtLeastOne(uint32_t delayMs)
{
    TickType_t ticks = pdMS_TO_TICKS(delayMs);
    return (ticks == 0u) ? 1u : ticks;
}

static void sensorarrayFdcSweepDelayMs(uint32_t delayMs)
{
    uint32_t remainingMs = delayMs;
    while (remainingMs > 0u) {
        uint32_t chunkMs = (remainingMs > 10u) ? 10u : remainingMs;
        vTaskDelay(sensorarrayFdcSweepMsToTicksAtLeastOne(chunkMs));
        remainingMs -= chunkMs;
    }
}

static esp_err_t sensorarrayFdcSweepDumpDeviceRegs(sensorarrayFdcDeviceState_t *fdc,
                                                   const char *stage,
                                                   const char *reason)
{
    if (!fdc || !fdc->handle) {
        return ESP_ERR_INVALID_STATE;
    }

    uint16_t mfg = 0u;
    uint16_t dev = 0u;
    uint16_t status = 0u;
    uint16_t statusConfig = 0u;
    uint16_t config = 0u;
    uint16_t mux = 0u;
    esp_err_t firstErr = Fdc2214CapReadRawRegisters(fdc->handle, SENSORARRAY_FDC_SWEEP_REG_MANUFACTURER_ID, &mfg);
    esp_err_t err = Fdc2214CapReadRawRegisters(fdc->handle, SENSORARRAY_FDC_SWEEP_REG_DEVICE_ID, &dev);
    if (err != ESP_OK && firstErr == ESP_OK) {
        firstErr = err;
    }
    err = Fdc2214CapReadRawRegisters(fdc->handle, SENSORARRAY_FDC_SWEEP_REG_STATUS, &status);
    if (err != ESP_OK && firstErr == ESP_OK) {
        firstErr = err;
    }
    err = Fdc2214CapReadRawRegisters(fdc->handle, SENSORARRAY_FDC_SWEEP_REG_STATUS_CONFIG, &statusConfig);
    if (err != ESP_OK && firstErr == ESP_OK) {
        firstErr = err;
    }
    err = Fdc2214CapReadRawRegisters(fdc->handle, SENSORARRAY_FDC_SWEEP_REG_CONFIG, &config);
    if (err != ESP_OK && firstErr == ESP_OK) {
        firstErr = err;
    }
    err = Fdc2214CapReadRawRegisters(fdc->handle, SENSORARRAY_FDC_SWEEP_REG_MUX_CONFIG, &mux);
    if (err != ESP_OK && firstErr == ESP_OK) {
        firstErr = err;
    }

    printf("FDC_REGS,stage=%s,reason=%s,device=%s,addr=0x%02X,mfg=0x%04X,dev=0x%04X,status=0x%04X,statusConfig=0x%04X,config=0x%04X,mux=0x%04X,err=0x%lx\n",
           stage ? stage : SENSORARRAY_NA,
           reason ? reason : SENSORARRAY_NA,
           fdc->label ? fdc->label : SENSORARRAY_NA,
           fdc->i2cAddr,
           mfg,
           dev,
           status,
           statusConfig,
           config,
           mux,
           (unsigned long)firstErr);

    if (firstErr == ESP_OK) {
        fdc->manufacturerId = mfg;
        fdc->deviceId = dev;
        fdc->statusConfigReg = statusConfig;
        fdc->configReg = config;
        fdc->muxConfigReg = mux;
    }

    for (uint8_t ch = 0u; ch < 4u; ++ch) {
        uint16_t rCount = 0u;
        uint16_t settle = 0u;
        uint16_t clockDiv = 0u;
        uint16_t drive = 0u;
        esp_err_t chErr = Fdc2214CapReadRawRegisters(fdc->handle,
                                                     sensorarrayFdcSweepRegForChannel(SENSORARRAY_FDC_SWEEP_REG_RCOUNT_BASE,
                                                                                      (Fdc2214CapChannel_t)ch),
                                                     &rCount);
        if (chErr == ESP_OK) {
            chErr = Fdc2214CapReadRawRegisters(fdc->handle,
                                               sensorarrayFdcSweepRegForChannel(SENSORARRAY_FDC_SWEEP_REG_SETTLECOUNT_BASE,
                                                                                (Fdc2214CapChannel_t)ch),
                                               &settle);
        }
        if (chErr == ESP_OK) {
            chErr = Fdc2214CapReadRawRegisters(fdc->handle,
                                               sensorarrayFdcSweepRegForChannel(SENSORARRAY_FDC_SWEEP_REG_CLOCK_DIVIDERS_BASE,
                                                                                (Fdc2214CapChannel_t)ch),
                                               &clockDiv);
        }
        if (chErr == ESP_OK) {
            chErr = Fdc2214CapReadRawRegisters(fdc->handle,
                                               sensorarrayFdcSweepRegForChannel(SENSORARRAY_FDC_SWEEP_REG_DRIVE_CURRENT_BASE,
                                                                                (Fdc2214CapChannel_t)ch),
                                               &drive);
        }
        if (chErr != ESP_OK && firstErr == ESP_OK) {
            firstErr = chErr;
        }
        printf("FDC_REGS_CH,stage=%s,device=%s,ch=%u,rCount=0x%04X,settle=0x%04X,clockDiv=0x%04X,drive=0x%04X,err=0x%lx\n",
               stage ? stage : SENSORARRAY_NA,
               fdc->label ? fdc->label : SENSORARRAY_NA,
               (unsigned)ch,
               rCount,
               settle,
               clockDiv,
               drive,
               (unsigned long)chErr);
    }
    return firstErr;
}

esp_err_t sensorarrayFdcSweepDumpAllDeviceRegs(sensorarrayState_t *state,
                                               const char *stage,
                                               const char *reason)
{
    if (!state) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t firstErr = ESP_OK;
    esp_err_t err = sensorarrayFdcSweepDumpDeviceRegs(&state->fdcPrimary, stage, reason);
    if (err != ESP_OK && firstErr == ESP_OK) {
        firstErr = err;
    }
    err = sensorarrayFdcSweepDumpDeviceRegs(&state->fdcSecondary, stage, reason);
    if (err != ESP_OK && firstErr == ESP_OK) {
        firstErr = err;
    }
    return firstErr;
}

static esp_err_t sensorarrayFdcSweepLogBootDeviceRegs(sensorarrayFdcDeviceState_t *fdc,
                                                      const char *deviceName)
{
    if (!fdc || !fdc->handle) {
        printf("FDC_BOOT,stage=device_regs,device=%s,err=0x%lx,reason=not_ready\n",
               deviceName ? deviceName : SENSORARRAY_NA,
               (unsigned long)ESP_ERR_INVALID_STATE);
        return ESP_ERR_INVALID_STATE;
    }

    uint16_t mfg = 0u;
    uint16_t dev = 0u;
    Fdc2214CapCoreRegs_t regs = {0};
    esp_err_t err = Fdc2214CapReadRawRegisters(fdc->handle, SENSORARRAY_FDC_SWEEP_REG_MANUFACTURER_ID, &mfg);
    if (err == ESP_OK) {
        err = Fdc2214CapReadRawRegisters(fdc->handle, SENSORARRAY_FDC_SWEEP_REG_DEVICE_ID, &dev);
    }
    if (err == ESP_OK) {
        err = Fdc2214CapReadCoreRegs(fdc->handle, &regs);
    }
    printf("FDC_BOOT,stage=device_regs,device=%s,addr=0x%02X,mfg=0x%04X,dev=0x%04X,status=0x%04X,config=0x%04X,mux=0x%04X,err=0x%lx\n",
           deviceName ? deviceName : SENSORARRAY_NA,
           fdc->i2cAddr,
           mfg,
           dev,
           regs.Status,
           regs.Config,
           regs.MuxConfig,
           (unsigned long)err);
    return err;
}

static const sensorarrayFdcDeglitchCandidate_t *sensorarrayFdcSweepFindDeglitchCandidate(uint8_t deglitchCode)
{
    uint8_t code = (uint8_t)(deglitchCode & SENSORARRAY_FDC_SWEEP_MUX_CONFIG_DEGLITCH_MASK);
    for (size_t i = 0u; i < (sizeof(SENSORARRAY_FDC_DEGLITCH_SWEEP_TABLE) /
                             sizeof(SENSORARRAY_FDC_DEGLITCH_SWEEP_TABLE[0])); ++i) {
        if (SENSORARRAY_FDC_DEGLITCH_SWEEP_TABLE[i].deglitchCode == code) {
            return &SENSORARRAY_FDC_DEGLITCH_SWEEP_TABLE[i];
        }
    }
    return NULL;
}

static const char *sensorarrayFdcSweepDeglitchNameFromCode(uint8_t deglitchCode)
{
    const sensorarrayFdcDeglitchCandidate_t *candidate =
        sensorarrayFdcSweepFindDeglitchCandidate(deglitchCode);
    return candidate ? candidate->deglitchName : SENSORARRAY_NA;
}

static Fdc2214CapDeglitch_t sensorarrayFdcSweepDeglitchEnum(uint8_t deglitchCode)
{
    const sensorarrayFdcDeglitchCandidate_t *candidate =
        sensorarrayFdcSweepFindDeglitchCandidate(deglitchCode);
    return candidate ? (Fdc2214CapDeglitch_t)candidate->deglitchCode : FDC2214_DEGLITCH_10MHZ;
}

static void sensorarrayFdcSweepAddUniqueDrive(uint16_t *drives,
                                              uint32_t *count,
                                              uint32_t capacity,
                                              uint16_t drive)
{
    if (!drives || !count || *count >= capacity) {
        return;
    }
    uint16_t norm = (uint16_t)(drive & SENSORARRAY_FDC_SWEEP_DRIVE_CURRENT_MASK);
    for (uint32_t i = 0u; i < *count; ++i) {
        if (drives[i] == norm) {
            return;
        }
    }
    drives[*count] = norm;
    (*count)++;
}

static uint32_t __attribute__((unused)) sensorarrayFdcSweepBuildDriveOrder(const sensorarrayFdcSweepProfile_t *profile,
                                                                          uint16_t *drives,
                                                                          uint32_t capacity)
{
    uint32_t count = 0u;
    if (profile && profile->valid) {
        sensorarrayFdcSweepAddUniqueDrive(drives, &count, capacity, profile->selectedDriveCurrent);
    }
    for (size_t i = 0u; i < (sizeof(SENSORARRAY_FDC_DRIVE_CURRENT_SWEEP_TABLE) /
                             sizeof(SENSORARRAY_FDC_DRIVE_CURRENT_SWEEP_TABLE[0])); ++i) {
        sensorarrayFdcSweepAddUniqueDrive(drives, &count, capacity, SENSORARRAY_FDC_DRIVE_CURRENT_SWEEP_TABLE[i]);
    }
    return count;
}

static void sensorarrayFdcSweepAddUniqueDeglitch(const sensorarrayFdcDeglitchCandidate_t **order,
                                                 uint32_t *count,
                                                 uint32_t capacity,
                                                 const sensorarrayFdcDeglitchCandidate_t *candidate)
{
    if (!order || !count || !candidate || *count >= capacity) {
        return;
    }
    for (uint32_t i = 0u; i < *count; ++i) {
        if (order[i] && order[i]->deglitchCode == candidate->deglitchCode) {
            return;
        }
    }
    order[*count] = candidate;
    (*count)++;
}

static uint32_t sensorarrayFdcSweepBuildDeglitchOrder(double predictedFreqHz,
                                                      const sensorarrayFdcDeglitchCandidate_t **order,
                                                      uint32_t capacity)
{
    uint32_t count = 0u;
    const sensorarrayFdcDeglitchCandidate_t *d1 =
        sensorarrayFdcSweepFindDeglitchCandidate(FDC2214_DEGLITCH_1MHZ);
    const sensorarrayFdcDeglitchCandidate_t *d3 =
        sensorarrayFdcSweepFindDeglitchCandidate(FDC2214_DEGLITCH_3P3MHZ);
    const sensorarrayFdcDeglitchCandidate_t *d10 =
        sensorarrayFdcSweepFindDeglitchCandidate(FDC2214_DEGLITCH_10MHZ);
    const sensorarrayFdcDeglitchCandidate_t *d33 =
        sensorarrayFdcSweepFindDeglitchCandidate(FDC2214_DEGLITCH_33MHZ);

    sensorarrayFdcSweepAddUniqueDeglitch(order, &count, capacity, d3);
    sensorarrayFdcSweepAddUniqueDeglitch(order, &count, capacity, d10);
    if (predictedFreqHz > 0.0) {
        if (predictedFreqHz >= 9000000.0 || count == 1u) {
            sensorarrayFdcSweepAddUniqueDeglitch(order, &count, capacity, d33);
        }
        if (predictedFreqHz < 900000.0) {
            sensorarrayFdcSweepAddUniqueDeglitch(order, &count, capacity, d1);
        }
    } else {
        sensorarrayFdcSweepAddUniqueDeglitch(order, &count, capacity, d33);
        sensorarrayFdcSweepAddUniqueDeglitch(order, &count, capacity, d1);
    }
    return count;
}

static esp_err_t sensorarrayFdcSweepReadKeyRegs(Fdc2214CapDevice_t *dev,
                                                Fdc2214CapChannel_t channel,
                                                sensorarrayFdcSweepCandidateResult_t *candidate)
{
    if (!dev || !candidate) {
        return ESP_ERR_INVALID_ARG;
    }

    uint16_t driveCurrent = 0u;
    esp_err_t err = Fdc2214CapReadRawRegisters(dev, SENSORARRAY_FDC_SWEEP_REG_STATUS, &candidate->statusReg);
    if (err == ESP_OK) {
        err = Fdc2214CapReadRawRegisters(dev, SENSORARRAY_FDC_SWEEP_REG_CONFIG, &candidate->configReg);
    }
    if (err == ESP_OK) {
        err = Fdc2214CapReadRawRegisters(dev, SENSORARRAY_FDC_SWEEP_REG_MUX_CONFIG, &candidate->muxConfigReg);
    }
    if (err == ESP_OK) {
        err = Fdc2214CapReadRawRegisters(dev,
                                         sensorarrayFdcSweepRegForChannel(SENSORARRAY_FDC_SWEEP_REG_RCOUNT_BASE,
                                                                          channel),
                                         &candidate->rCountReg);
    }
    if (err == ESP_OK) {
        err = Fdc2214CapReadRawRegisters(dev,
                                         sensorarrayFdcSweepRegForChannel(SENSORARRAY_FDC_SWEEP_REG_SETTLECOUNT_BASE,
                                                                          channel),
                                         &candidate->settleCountReg);
    }
    if (err == ESP_OK) {
        err = Fdc2214CapReadClockDividers(dev, channel, &candidate->clockDividersReg);
    }
    if (err == ESP_OK) {
        err = Fdc2214CapReadDriveCurrent(dev, channel, &driveCurrent);
    }
    if (err != ESP_OK) {
        return err;
    }

    candidate->driveCurrentReadback = (uint16_t)(driveCurrent & SENSORARRAY_FDC_SWEEP_DRIVE_CURRENT_MASK);
    candidate->highCurrentReadback =
        (candidate->configReg & SENSORARRAY_FDC_SWEEP_CONFIG_HIGH_CURRENT_DRV_MASK) != 0u;
    candidate->activeChannelReadback =
        (uint8_t)((candidate->configReg & SENSORARRAY_FDC_SWEEP_CONFIG_ACTIVE_CHAN_MASK) >>
                  SENSORARRAY_FDC_SWEEP_CONFIG_ACTIVE_CHAN_SHIFT);

    const char *clockStatus = NULL;
    bool clockOk = sensorarrayMeasureFdcDecodeClockDividers(candidate->clockDividersReg,
                                                            &candidate->finSelCode,
                                                            &candidate->finFactor,
                                                            &candidate->frefDivider,
                                                            &clockStatus);
    candidate->effectiveFclkHz = sensorarrayMeasureFdcEffectiveFclkHz();
    candidate->effectiveFrefHz = (clockOk && candidate->frefDivider != 0u)
                                     ? ((double)candidate->effectiveFclkHz / (double)candidate->frefDivider)
                                     : 0.0;
    if (clockOk && candidate->raw28Mean > 0u && candidate->frequencyHz <= 0.0) {
        sensorarrayFdcFrequencyDiag_t freqDiag = {0};
        if (sensorarrayMeasureFdcComputeFrequencyDiag(candidate->raw28Mean,
                                                      candidate->clockDividersReg,
                                                      &freqDiag) &&
            freqDiag.valid) {
            candidate->frequencyHz = freqDiag.freqHzCorrected;
        }
    }
    if (candidate->frequencyHz > 0.0) {
        double marginHz = candidate->frequencyHz * SENSORARRAY_FDC_SWEEP_CACHE_FREQ_MARGIN_RATIO;
        if (marginHz < SENSORARRAY_FDC_SWEEP_CACHE_FREQ_MARGIN_MIN_HZ) {
            marginHz = SENSORARRAY_FDC_SWEEP_CACHE_FREQ_MARGIN_MIN_HZ;
        }
        candidate->frequencyMarginHz = marginHz;
        if (candidate->deglitchBandwidthHz > 0u) {
            candidate->deglitchMarginRatio =
                (double)candidate->deglitchBandwidthHz / candidate->frequencyHz;
            candidate->deglitchBandwidthOk =
                candidate->frequencyHz < ((double)candidate->deglitchBandwidthHz * 0.90);
        }
    }
    (void)clockStatus;
    return ESP_OK;
}

static bool sensorarrayFdcSweepCandidateIsWorking(const sensorarrayFdcSweepCandidateResult_t *candidate)
{
    return sensorarrayFdcSweepCandidateHasStableSamples(candidate);
}

static bool sensorarrayFdcSweepCandidateIsFallbackUsable(const sensorarrayFdcSweepCandidateResult_t *candidate)
{
    if (!candidate || !candidate->valid) {
        return false;
    }
    return candidate->validSampleCount >= SENSORARRAY_FDC_SWEEP_DIRECT_MIN_VALID_SAMPLES &&
           candidate->timeoutCount == 0u &&
           candidate->watchdogCount == 0u &&
           candidate->zeroRawCount == 0u &&
           candidate->saturatedCount == 0u &&
           candidate->raw28Mean != 0u &&
           candidate->frequencyHz > 0.0 &&
           candidate->deglitchBandwidthOk;
}

static bool sensorarrayFdcSweepCandidateHasRawFrequency(const sensorarrayFdcSweepCandidateResult_t *candidate)
{
    return candidate &&
           candidate->valid &&
           candidate->raw28Mean != 0u &&
           candidate->frequencyHz > 0.0;
}

static uint8_t sensorarrayFdcSweepCandidatePriority(const sensorarrayFdcSweepCandidateResult_t *candidate)
{
    if (sensorarrayFdcSweepCandidateIsWorking(candidate)) {
        return 3u;
    }
    if (sensorarrayFdcSweepCandidateIsFallbackUsable(candidate)) {
        return 2u;
    }
    if (sensorarrayFdcSweepCandidateHasRawFrequency(candidate)) {
        return 1u;
    }
    return 0u;
}

static int32_t sensorarrayFdcSweepCandidateScore(const sensorarrayFdcSweepCandidateResult_t *candidate)
{
    if (!candidate) {
        return INT_MIN;
    }

    if (candidate->raw28Mean == 0u || candidate->frequencyHz <= 0.0) {
        return INT_MIN / 2;
    }

    bool stable = sensorarrayFdcSweepCandidateHasStableSamples(candidate);
    int32_t score = 0;
    score += stable ? 10000 : 0;
    score += (int32_t)(candidate->validSampleCount * 40u);
    score -= (int32_t)(candidate->invalidSampleCount * 400u);
    score -= (int32_t)(candidate->timeoutCount * 1000u);
    score -= (int32_t)(candidate->watchdogCount * 1000u);
    score -= (int32_t)(candidate->amplitudeFaultCount * 300u);
    score -= (int32_t)(candidate->zeroRawCount * 500u);
    score -= (int32_t)(candidate->saturatedCount * 700u);
    if (candidate->raw28Mean > 0u) {
        uint32_t span = candidate->raw28Max - candidate->raw28Min;
        uint32_t spanPermille = (uint32_t)(((uint64_t)span * 1000ull) / candidate->raw28Mean);
        if (spanPermille <= 5u) {
            score += 35;
        } else if (spanPermille <= 20u) {
            score += 20;
        } else if (spanPermille <= 80u) {
            score += 5;
        } else {
            uint32_t penalty = spanPermille / 4u;
            score -= (int32_t)((penalty > 120u) ? 120u : penalty);
        }
    }
    if (candidate->deglitchMarginRatio > 0.0) {
        if (candidate->deglitchMarginRatio < 1.15) {
            score -= 120;
        } else if (candidate->deglitchMarginRatio < 1.50) {
            score -= 40;
        } else if (candidate->deglitchMarginRatio > 12.0) {
            score -= 15;
        }
    } else {
        score -= 200;
    }
    if (candidate->deglitchReq == SENSORARRAY_FDC_SWEEP_DEFAULT_DEGLITCH_REQ) {
        score += 15;
    }
    if (candidate->driveCurrentNorm <= 0x7000u) {
        score += 30;
    } else if (candidate->driveCurrentNorm <= 0xB800u) {
        score += 10;
    }
    if (candidate->highCurrentReq) {
        score -= 25;
    }
    return score;
}

static bool sensorarrayFdcSweepCandidateBetter(const sensorarrayFdcSweepCandidateResult_t *candidate,
                                               const sensorarrayFdcSweepCandidateResult_t *best)
{
    if (!candidate || !candidate->valid) {
        return false;
    }
    if (!best || !best->valid) {
        return true;
    }
    uint8_t candidatePriority = sensorarrayFdcSweepCandidatePriority(candidate);
    uint8_t bestPriority = sensorarrayFdcSweepCandidatePriority(best);
    if (candidatePriority != bestPriority) {
        return candidatePriority > bestPriority;
    }
    int32_t candidateScore = sensorarrayFdcSweepCandidateScore(candidate);
    int32_t bestScore = sensorarrayFdcSweepCandidateScore(best);
    if (candidateScore != bestScore) {
        return candidateScore > bestScore;
    }
    if (candidate->amplitudeFaultCount != best->amplitudeFaultCount) {
        return candidate->amplitudeFaultCount < best->amplitudeFaultCount;
    }
    if (candidate->deglitchBandwidthHz != best->deglitchBandwidthHz) {
        return candidate->deglitchBandwidthHz < best->deglitchBandwidthHz;
    }
    return candidate->driveCurrentNorm < best->driveCurrentNorm;
}

static bool sensorarrayFdcSweepFallbackBetter(const sensorarrayFdcSweepCandidateResult_t *candidate,
                                              const sensorarrayFdcSweepCandidateResult_t *best)
{
    uint8_t candidatePriority = sensorarrayFdcSweepCandidatePriority(candidate);
    if (candidatePriority == 0u || candidatePriority == 3u) {
        return false;
    }
    uint8_t bestPriority = sensorarrayFdcSweepCandidatePriority(best);
    if (bestPriority == 0u || bestPriority == 3u) {
        return true;
    }
    if (candidatePriority != bestPriority) {
        return candidatePriority > bestPriority;
    }
    if (candidate->amplitudeFaultCount != best->amplitudeFaultCount) {
        return candidate->amplitudeFaultCount < best->amplitudeFaultCount;
    }
    if (candidate->invalidSampleCount != best->invalidSampleCount) {
        return candidate->invalidSampleCount < best->invalidSampleCount;
    }
    if (candidate->driveCurrentNorm != best->driveCurrentNorm) {
        return candidate->driveCurrentNorm < best->driveCurrentNorm;
    }
    return sensorarrayFdcSweepCandidateScore(candidate) > sensorarrayFdcSweepCandidateScore(best);
}

static sensorarrayFdcSampleEval_t sensorarrayFdcSweepEvaluateSample(const sensorarrayFdcReadDiag_t *diag)
{
    sensorarrayFdcSampleEval_t eval = {
        .i2cErr = diag ? diag->err : ESP_ERR_INVALID_ARG,
        .i2cOk = diag && diag->err == ESP_OK && diag->i2cOk,
        .invalidReason = "i2c_read_failed",
    };
    if (!diag) {
        return eval;
    }

    eval.raw28 = diag->sample.Raw28;
    eval.status = diag->sample.StatusRaw;
    eval.rawNonZero = diag->sample.Raw28 != 0u;
    eval.sleep = diag->sample.SleepModeEnabled;
    eval.converting = diag->sample.Converting;
    bool statusWatchdog = Fdc2214CapStatusHasWatchdogFault(&diag->status);
    bool statusAmplitude = Fdc2214CapStatusHasAmplitudeFault(&diag->status);
    if ((eval.status & 0x000Fu) != 0u &&
        (eval.status & FDC2214CAP_STATUS_AMP_MASK) == 0u) {
        statusAmplitude = false;
    }
    eval.watchdogFault = diag->sample.ErrWatchdog || statusWatchdog;
    eval.amplitudeFault = diag->sample.ErrAmplitude || statusAmplitude;
    eval.saturated = diag->sample.Raw28 >= SENSORARRAY_FDC_SWEEP_RAW28_SATURATED_THRESHOLD;
    eval.dataReadyOk = diag->sample.DataReady || diag->status.DataReady;
    eval.unreadOk = diag->unreadConversionPresent || diag->sample.UnreadConversionPresent;
    eval.readableOk = eval.unreadOk || eval.dataReadyOk;
    eval.statusOk = eval.i2cOk &&
                    !eval.sleep &&
                    eval.converting &&
                    !eval.watchdogFault;
    eval.sampleValid = eval.statusOk &&
                       eval.rawNonZero &&
                       eval.readableOk &&
                       !eval.saturated;

    if (!eval.i2cOk) {
        eval.invalidReason = "i2c_read_failed";
    } else if (eval.sleep) {
        eval.invalidReason = "status_sleep";
    } else if (!eval.converting) {
        eval.invalidReason = "status_not_converting";
    } else if (eval.watchdogFault) {
        eval.invalidReason = "status_watchdog";
    } else if (!eval.rawNonZero) {
        eval.invalidReason = "zero_raw_no_oscillation";
    } else if (eval.saturated) {
        eval.invalidReason = "saturated";
    } else if (!eval.readableOk) {
        eval.invalidReason = "no_unread_conversion";
    } else if (!eval.sampleValid) {
        eval.invalidReason = sensorarrayMeasureFdcSampleStatusName(diag->statusCode);
    } else {
        eval.invalidReason = "valid";
    }
    return eval;
}

static esp_err_t sensorarrayFdcSweepReadOneSampleBounded(Fdc2214CapDevice_t *dev,
                                                         const char *deviceName,
                                                         Fdc2214CapChannel_t channel,
                                                         uint32_t timeoutMs,
                                                         sensorarrayFdcReadDiag_t *outDiag,
                                                         sensorarrayFdcSampleEval_t *outEval)
{
    if (!dev || !outDiag) {
        return ESP_ERR_INVALID_ARG;
    }

    uint32_t intervalMs = (uint32_t)CONFIG_SENSORARRAY_FDC_SWEEP_SAMPLE_POLL_MS;
    if (intervalMs == 0u) {
        intervalMs = 1u;
    }
    if (timeoutMs == 0u) {
        timeoutMs = intervalMs;
    }

    int64_t deadlineUs = esp_timer_get_time() + ((int64_t)timeoutMs * 1000LL);
    sensorarrayFdcReadDiag_t lastDiag = {0};
    uint32_t polls = 0u;
    while (esp_timer_get_time() < deadlineUs) {
        sensorarrayFdcReadDiag_t diag = {0};
        esp_err_t err = sensorarrayMeasureReadFdcSampleDiagRelaxed(dev,
                                                                   channel,
                                                                   false,
                                                                   true,
                                                                   true,
                                                                   &diag);
        polls++;
        lastDiag = diag;
        if (err != ESP_OK) {
            *outDiag = diag;
            outDiag->err = err;
            if (outEval) {
                *outEval = sensorarrayFdcSweepEvaluateSample(outDiag);
            }
            return err;
        }

        sensorarrayFdcSampleEval_t eval = sensorarrayFdcSweepEvaluateSample(&diag);
        bool terminal = eval.sleep ||
                        !eval.converting ||
                        eval.watchdogFault ||
                        (eval.amplitudeFault && eval.rawNonZero);
        if (eval.unreadOk ||
            eval.dataReadyOk ||
            terminal) {
            *outDiag = diag;
            if (outEval) {
                *outEval = eval;
            }
            if (eval.amplitudeFault && eval.sampleValid) {
                printf("FDC_SAMPLE,stage=warning,reason=amplitude_warning,device=%s,ch=%u,"
                       "status=0x%04X,raw28=%lu,"
                       "i2cOk=%u,converting=%u,dataReady=%u,"
                       "unread=%u,errWd=%u,errAhw=%u,errAlw=%u,"
                       "watchdog=%u,amplitude=%u,saturated=%u,"
                       "sampleStatus=%d\n",
                       deviceName ? deviceName : SENSORARRAY_NA,
                       (unsigned)channel,
                       eval.status,
                       (unsigned long)eval.raw28,
                       eval.i2cOk ? 1u : 0u,
                       eval.converting ? 1u : 0u,
                       eval.dataReadyOk ? 1u : 0u,
                       eval.unreadOk ? 1u : 0u,
                       diag.status.ErrWatchdog ? 1u : 0u,
                       diag.status.ErrAmplitudeHigh ? 1u : 0u,
                       diag.status.ErrAmplitudeLow ? 1u : 0u,
                       eval.watchdogFault ? 1u : 0u,
                       eval.amplitudeFault ? 1u : 0u,
                       eval.saturated ? 1u : 0u,
                       (int)diag.sample.SampleStatus);
            }
            if (!eval.sampleValid) {
                printf("FDC_SAMPLE,stage=invalid,reason=%s,device=%s,ch=%u,"
                       "status=0x%04X,raw28=%lu,"
                       "i2cOk=%u,converting=%u,dataReady=%u,"
                       "unread=%u,errWd=%u,errAhw=%u,errAlw=%u,"
                       "watchdog=%u,amplitude=%u,saturated=%u,"
                       "sampleStatus=%d\n",
                       eval.invalidReason,
                       deviceName ? deviceName : SENSORARRAY_NA,
                       (unsigned)channel,
                       eval.status,
                       (unsigned long)eval.raw28,
                       eval.i2cOk ? 1u : 0u,
                       eval.converting ? 1u : 0u,
                       eval.dataReadyOk ? 1u : 0u,
                       eval.unreadOk ? 1u : 0u,
                       diag.status.ErrWatchdog ? 1u : 0u,
                       diag.status.ErrAmplitudeHigh ? 1u : 0u,
                       diag.status.ErrAmplitudeLow ? 1u : 0u,
                       eval.watchdogFault ? 1u : 0u,
                       eval.amplitudeFault ? 1u : 0u,
                       eval.saturated ? 1u : 0u,
                       (int)diag.sample.SampleStatus);
            } else if (eval.status == FDC2214CAP_STATUS_CH0_UNREAD_MASK &&
                       channel == FDC2214_CH0) {
                static bool s_loggedCh0UnreadOnly = false;
                if (!s_loggedCh0UnreadOnly) {
                    s_loggedCh0UnreadOnly = true;
                    printf("FDC_SAMPLE,stage=valid,reason=unread_conversion,device=%s,ch=%u,"
                           "status=0x%04X,raw28=%lu,"
                           "i2cOk=%u,converting=%u,dataReady=%u,"
                           "unread=%u,errWd=%u,errAhw=%u,errAlw=%u,"
                           "watchdog=%u,amplitude=%u,saturated=%u,"
                           "sampleStatus=%d\n",
                           deviceName ? deviceName : SENSORARRAY_NA,
                           (unsigned)channel,
                           eval.status,
                           (unsigned long)eval.raw28,
                           eval.i2cOk ? 1u : 0u,
                           eval.converting ? 1u : 0u,
                           eval.dataReadyOk ? 1u : 0u,
                           eval.unreadOk ? 1u : 0u,
                           diag.status.ErrWatchdog ? 1u : 0u,
                           diag.status.ErrAmplitudeHigh ? 1u : 0u,
                           diag.status.ErrAmplitudeLow ? 1u : 0u,
                           eval.watchdogFault ? 1u : 0u,
                           eval.amplitudeFault ? 1u : 0u,
                           eval.saturated ? 1u : 0u,
                           (int)diag.sample.SampleStatus);
                }
            }
            return ESP_OK;
        }

        uint32_t remainingMs = sensorarrayFdcSweepRemainingMsUntil(deadlineUs);
        if (remainingMs == 0u) {
            break;
        }
        sensorarrayFdcSweepDelayMs((intervalMs < remainingMs) ? intervalMs : remainingMs);
    }

    *outDiag = lastDiag;
    outDiag->err = ESP_ERR_TIMEOUT;
    outDiag->i2cOk = false;
    if (outEval) {
        *outEval = sensorarrayFdcSweepEvaluateSample(outDiag);
        outEval->i2cErr = ESP_ERR_TIMEOUT;
        outEval->i2cOk = false;
        outEval->sampleValid = false;
        outEval->invalidReason = "timeout";
    }
    printf("FDC_SAMPLE,stage=timeout,device=%s,ch=%u,timeoutMs=%lu,polls=%lu,status=0x%04X,unread=%u,dataReady=%u,err=0x%lx\n",
           deviceName ? deviceName : SENSORARRAY_NA,
           (unsigned)channel,
           (unsigned long)timeoutMs,
           (unsigned long)polls,
           lastDiag.sample.StatusRaw,
           lastDiag.sample.UnreadConversionPresent ? 1u : 0u,
           lastDiag.sample.DataReady ? 1u : 0u,
           (unsigned long)ESP_ERR_TIMEOUT);
    return ESP_ERR_TIMEOUT;
}

static void sensorarrayFdcSweepFinalizeRawStats(sensorarrayFdcSweepCandidateResult_t *candidate,
                                                uint64_t rawSum)
{
    if (!candidate) {
        return;
    }
    if (candidate->validSampleCount > 0u) {
        candidate->raw28Mean = (uint32_t)(rawSum / candidate->validSampleCount);
    } else {
        candidate->raw28Min = 0u;
        candidate->raw28Max = 0u;
        candidate->raw28Mean = 0u;
    }
    candidate->saturated = candidate->saturatedCount != 0u ||
                           candidate->raw28Max >= SENSORARRAY_FDC_SWEEP_RAW28_SATURATED_THRESHOLD;
}

static void sensorarrayFdcSweepSortFrequencies(double *values, uint32_t count)
{
    if (!values) {
        return;
    }
    for (uint32_t i = 1u; i < count; ++i) {
        double value = values[i];
        uint32_t j = i;
        while (j > 0u && values[j - 1u] > value) {
            values[j] = values[j - 1u];
            j--;
        }
        values[j] = value;
    }
}

static void sensorarrayFdcSweepFinalizeFrequencyStats(sensorarrayFdcSweepCandidateResult_t *candidate,
                                                      double *freqSamples,
                                                      uint32_t freqCount)
{
    if (!candidate) {
        return;
    }

    if (freqSamples && freqCount > 0u) {
        sensorarrayFdcSweepSortFrequencies(freqSamples, freqCount);
        double minHz = freqSamples[0];
        double maxHz = freqSamples[freqCount - 1u];
        if ((freqCount & 1u) == 0u) {
            candidate->medianFreqHz =
                (freqSamples[(freqCount / 2u) - 1u] + freqSamples[freqCount / 2u]) / 2.0;
        } else {
            candidate->medianFreqHz = freqSamples[freqCount / 2u];
        }
        candidate->freqSpreadHz = maxHz - minHz;
        candidate->relativeSpread =
            (candidate->medianFreqHz > 0.0) ? (candidate->freqSpreadHz / candidate->medianFreqHz) : 1.0;
        candidate->frequencyHz = candidate->medianFreqHz;
    } else if (candidate->frequencyHz > 0.0) {
        candidate->medianFreqHz = candidate->frequencyHz;
        candidate->freqSpreadHz = 0.0;
        candidate->relativeSpread = 0.0;
    } else {
        candidate->medianFreqHz = 0.0;
        candidate->freqSpreadHz = 0.0;
        candidate->relativeSpread = 1.0;
    }

    if (candidate->frequencyHz > 0.0) {
        double marginHz = candidate->frequencyHz * SENSORARRAY_FDC_SWEEP_CACHE_FREQ_MARGIN_RATIO;
        if (marginHz < SENSORARRAY_FDC_SWEEP_CACHE_FREQ_MARGIN_MIN_HZ) {
            marginHz = SENSORARRAY_FDC_SWEEP_CACHE_FREQ_MARGIN_MIN_HZ;
        }
        candidate->frequencyMarginHz = marginHz;
        if (candidate->deglitchBandwidthHz > 0u) {
            candidate->deglitchMarginRatio =
                (double)candidate->deglitchBandwidthHz / candidate->frequencyHz;
            candidate->deglitchBandwidthOk =
                candidate->frequencyHz < ((double)candidate->deglitchBandwidthHz * 0.90);
        }
    }
}

static bool sensorarrayFdcSweepCandidateHasStableSamples(const sensorarrayFdcSweepCandidateResult_t *candidate)
{
    if (!candidate || !candidate->valid) {
        return false;
    }
    if (candidate->validSampleCount < SENSORARRAY_FDC_SWEEP_DIRECT_MIN_VALID_SAMPLES) {
        return false;
    }
    if (candidate->invalidSampleCount != 0u ||
        candidate->timeoutCount != 0u ||
        candidate->watchdogCount != 0u ||
        candidate->zeroRawCount != 0u ||
        candidate->saturatedCount != 0u) {
        return false;
    }
    if (!candidate->deglitchBandwidthOk || candidate->frequencyHz <= 0.0) {
        return false;
    }
    if (candidate->relativeSpread > SENSORARRAY_FDC_SWEEP_STABLE_RELATIVE_SPREAD) {
        return false;
    }
    if (candidate->raw28Mean > 0u) {
        uint32_t span = candidate->raw28Max - candidate->raw28Min;
        if (((uint64_t)span * 1000ull) > ((uint64_t)candidate->raw28Mean * 5ull)) {
            return false;
        }
    }
    return true;
}

esp_err_t sensorarrayFdcSweepApplyDirectSafeLock(sensorarrayState_t *state,
                                                 sensorarrayFdcDeviceId_t devId,
                                                 Fdc2214CapChannel_t channel,
                                                 uint8_t deglitchCode,
                                                 uint16_t driveCurrent,
                                                 bool highCurrent,
                                                 const char *reason)
{
    sensorarrayFdcDeviceState_t *fdcState = sensorarrayMeasureGetFdcState(state, devId);
    if (!fdcState || !fdcState->ready || !fdcState->handle) {
        return ESP_ERR_INVALID_STATE;
    }

    uint16_t driveNorm = (uint16_t)(driveCurrent & SENSORARRAY_FDC_SWEEP_DRIVE_CURRENT_MASK);
    Fdc2214CapDeglitch_t deglitch = sensorarrayFdcSweepDeglitchEnum(deglitchCode);
    const char *devName = sensorarrayFdcSweepDevName(devId);
    const char *devFullName = sensorarrayFdcSweepDevFullName(devId);
    const char *source = reason ? reason : SENSORARRAY_NA;
    bool highCurrentApplied = highCurrent;

    printf("FDC_LOCK,stage=apply_begin,device=%s,ch=%u,driveReq=0x%04X,deglitchReq=0x%X,highCurrentReq=%u,reason=%s\n",
           devFullName,
           (unsigned)channel,
           driveNorm,
           (unsigned)deglitchCode,
           highCurrent ? 1u : 0u,
           source);
    printf("FDC_SWEEP,stage=direct_lock_begin,dev=%s,ch=%u,reason=%s,deglitch=0x%X,drive=0x%04X\n",
           devName,
           (unsigned)channel,
           source,
           (unsigned)deglitch,
           driveNorm);

    esp_err_t err = Fdc2214CapSetMuxConfig(fdcState->handle, false, 0u, deglitch);
    uint16_t muxReadback = 0u;
    if (err == ESP_OK) {
        err = Fdc2214CapReadRawRegisters(fdcState->handle, SENSORARRAY_FDC_SWEEP_REG_MUX_CONFIG, &muxReadback);
    }
    if (err != ESP_OK ||
        ((muxReadback & SENSORARRAY_FDC_SWEEP_MUX_CONFIG_DEGLITCH_MASK) !=
         (deglitchCode & SENSORARRAY_FDC_SWEEP_MUX_CONFIG_DEGLITCH_MASK)) ||
        ((muxReadback & SENSORARRAY_FDC_SWEEP_MUX_CONFIG_AUTOSCAN_MASK) != 0u)) {
        printf("FDC_SWEEP,stage=direct_lock_done,dev=%s,ch=%u,reason=%s,err=%ld,muxConfig=0x%04X,status=mux_readback_failed\n",
               devName,
               (unsigned)channel,
               source,
               (long)((err != ESP_OK) ? err : ESP_ERR_INVALID_RESPONSE),
               muxReadback);
        return (err != ESP_OK) ? err : ESP_ERR_INVALID_RESPONSE;
    }

    Fdc2214CapConfigOptions_t config = {
        .ActiveChannel = channel,
        .SleepModeEnabled = false,
        .SensorActivateSelLowPower = false,
        .RefClockSource = fdcState->refClockKnown ? fdcState->refClockSource :
#if SENSORARRAY_FDC_REF_CLOCK_USE_EXTERNAL
                                                   FDC2214_REF_CLOCK_EXTERNAL,
#else
                                                   FDC2214_REF_CLOCK_INTERNAL,
#endif
        .IntbDisabled = true,
        .HighCurrentDrive = highCurrentApplied,
    };
    uint16_t configReq = Fdc2214CapBuildConfig(&config);
    err = Fdc2214CapExitSleep(fdcState->handle, configReq);
    uint16_t configReadback = 0u;
    if (err == ESP_OK) {
        err = Fdc2214CapReadRawRegisters(fdcState->handle, SENSORARRAY_FDC_SWEEP_REG_CONFIG, &configReadback);
    }
    uint8_t activeReadback =
        (uint8_t)((configReadback & SENSORARRAY_FDC_SWEEP_CONFIG_ACTIVE_CHAN_MASK) >>
                  SENSORARRAY_FDC_SWEEP_CONFIG_ACTIVE_CHAN_SHIFT);
    bool sleepReadback = (configReadback & SENSORARRAY_FDC_SWEEP_CONFIG_SLEEP_MODE_EN_MASK) != 0u;
    bool highCurrentReadback = (configReadback & SENSORARRAY_FDC_SWEEP_CONFIG_HIGH_CURRENT_DRV_MASK) != 0u;
    if (err != ESP_OK ||
        activeReadback != (uint8_t)channel ||
        sleepReadback ||
        highCurrentReadback != highCurrentApplied) {
        printf("FDC_SWEEP,stage=direct_lock_done,dev=%s,ch=%u,reason=%s,err=%ld,config=0x%04X,active=%u,sleep=%u,highCurrent=%u,status=config_readback_failed\n",
               devName,
               (unsigned)channel,
               source,
               (long)((err != ESP_OK) ? err : ESP_ERR_INVALID_RESPONSE),
               configReadback,
               (unsigned)activeReadback,
               sleepReadback ? 1u : 0u,
               highCurrentReadback ? 1u : 0u);
        return (err != ESP_OK) ? err : ESP_ERR_INVALID_RESPONSE;
    }

    err = Fdc2214CapWriteDriveCurrent(fdcState->handle, channel, driveNorm);
    uint16_t driveReadback = 0u;
    if (err == ESP_OK) {
        err = Fdc2214CapReadDriveCurrent(fdcState->handle, channel, &driveReadback);
    }
    if (err != ESP_OK || driveReadback != driveNorm) {
        printf("FDC_SWEEP,stage=direct_lock_done,dev=%s,ch=%u,reason=%s,err=%ld,driveReadback=0x%04X,status=drive_readback_failed\n",
               devName,
               (unsigned)channel,
               source,
               (long)((err != ESP_OK) ? err : ESP_ERR_INVALID_RESPONSE),
               driveReadback);
        return (err != ESP_OK) ? err : ESP_ERR_INVALID_RESPONSE;
    }

    uint16_t rCountReadback = 0u;
    uint16_t settleReadback = 0u;
    uint16_t clockReadback = 0u;
    uint16_t statusReadback = 0u;
    esp_err_t regsErr = Fdc2214CapReadRawRegisters(fdcState->handle,
                                                   sensorarrayFdcSweepRegForChannel(SENSORARRAY_FDC_SWEEP_REG_RCOUNT_BASE,
                                                                                    channel),
                                                   &rCountReadback);
    if (regsErr == ESP_OK) {
        regsErr = Fdc2214CapReadRawRegisters(fdcState->handle,
                                             sensorarrayFdcSweepRegForChannel(SENSORARRAY_FDC_SWEEP_REG_SETTLECOUNT_BASE,
                                                                              channel),
                                             &settleReadback);
    }
    if (regsErr == ESP_OK) {
        regsErr = Fdc2214CapReadRawRegisters(fdcState->handle,
                                             sensorarrayFdcSweepRegForChannel(SENSORARRAY_FDC_SWEEP_REG_CLOCK_DIVIDERS_BASE,
                                                                              channel),
                                             &clockReadback);
    }
    if (regsErr == ESP_OK) {
        regsErr = Fdc2214CapReadRawRegisters(fdcState->handle,
                                             SENSORARRAY_FDC_SWEEP_REG_STATUS,
                                             &statusReadback);
    }
    printf("FDC_LOCK,stage=reg_readback,device=%s,ch=%u,rCount=0x%04X,settle=0x%04X,clockDiv=0x%04X,drive=0x%04X,config=0x%04X,mux=0x%04X,err=0x%lx\n",
           devFullName,
           (unsigned)channel,
           rCountReadback,
           settleReadback,
           clockReadback,
           driveReadback,
           configReadback,
           muxReadback,
           (unsigned long)regsErr);
    if (regsErr != ESP_OK) {
        return regsErr;
    }

    sensorarrayFdcSweepDelayMs((uint32_t)CONFIG_SENSORARRAY_FDC_SWEEP_SETTLE_MS);
    err = Fdc2214CapReadRawRegisters(fdcState->handle, SENSORARRAY_FDC_SWEEP_REG_STATUS, &statusReadback);
    bool statusWatchdog = (statusReadback & SENSORARRAY_FDC_SWEEP_STATUS_ERR_WD_MASK) != 0u;
    bool statusAmplitude = (statusReadback & (SENSORARRAY_FDC_SWEEP_STATUS_ERR_AHW_MASK |
                                              SENSORARRAY_FDC_SWEEP_STATUS_ERR_ALW_MASK)) != 0u;
    bool statusConverting = !sleepReadback;
    printf("FDC_LOCK,stage=apply_done,device=%s,ch=%u,status=0x%04X,sleep=%u,converting=%u,watchdog=%u,amplitude=%u,err=0x%lx\n",
           devFullName,
           (unsigned)channel,
           statusReadback,
           sleepReadback ? 1u : 0u,
           statusConverting ? 1u : 0u,
           statusWatchdog ? 1u : 0u,
           statusAmplitude ? 1u : 0u,
           (unsigned long)err);
    if (err != ESP_OK) {
        return err;
    }

    err = Fdc2214CapClearStatus(fdcState->handle);
    printf("FDC_SWEEP,stage=direct_lock_done,dev=%s,ch=%u,reason=%s,err=%ld,muxConfig=0x%04X,config=0x%04X,driveReadback=0x%04X,status=%s\n",
           devName,
           (unsigned)channel,
           source,
           (long)err,
           muxReadback,
           configReadback,
           driveReadback,
           (err == ESP_OK) ? "applied" : "clear_status_failed");
    return err;
}

static esp_err_t sensorarrayFdcSweepCaptureCandidate(sensorarrayState_t *state,
                                                     sensorarrayFdcDeviceId_t devId,
                                                     Fdc2214CapChannel_t channel,
                                                     const sensorarrayFdcDeglitchCandidate_t *deglitch,
                                                     uint16_t driveCurrent,
                                                     bool highCurrent,
                                                     uint8_t sIndex,
                                                     uint8_t dIndex,
                                                     const char *stage,
                                                     uint32_t candidateIndex,
                                                     uint32_t candidateCount,
                                                     bool discardFirst,
                                                     uint32_t sampleTarget,
                                                     sensorarrayFdcSweepCandidateResult_t *outCandidate)
{
    sensorarrayFdcDeviceState_t *fdcState = sensorarrayMeasureGetFdcState(state, devId);
    if (!fdcState || !fdcState->ready || !fdcState->handle || !deglitch || !outCandidate) {
        return ESP_ERR_INVALID_ARG;
    }
    sensorarrayFdcCellTarget_t target = {0};
    if (!sensorarrayMeasureMakeFdcCellTarget(state, sIndex, dIndex, &target) ||
        target.devId != devId ||
        target.fdcChannel != (uint8_t)channel) {
        printf("FDC_SWEEP,stage=candidate_reject,reason=bad_cell_target,s=%u,d=%u,device=%s,ch=%u\n",
               (unsigned)sIndex,
               (unsigned)dIndex,
               sensorarrayFdcSweepDevFullName(devId),
               (unsigned)channel);
        return ESP_ERR_INVALID_ARG;
    }

    *outCandidate = (sensorarrayFdcSweepCandidateResult_t){
        .valid = true,
        .deglitchReq = deglitch->deglitchCode,
        .deglitchName = deglitch->deglitchName,
        .deglitchBandwidthHz = deglitch->deglitchBandwidthHz,
        .driveCurrentReq = driveCurrent,
        .driveCurrentNorm = (uint16_t)(driveCurrent & SENSORARRAY_FDC_SWEEP_DRIVE_CURRENT_MASK),
        .highCurrentReq = highCurrent,
        .raw28Min = SENSORARRAY_FDC_SWEEP_RAW28_MAX,
    };

    const char *devName = sensorarrayFdcSweepDevName(devId);
    const char *devFullName = sensorarrayFdcSweepDevFullName(devId);
    const char *stageName = stage ? stage : "sweep";
    sampleTarget = sensorarrayFdcSweepClampQualitySamples(sampleTarget);
    uint32_t stepTimeoutMs = sensorarrayFdcSweepStepTimeoutMs();
    int64_t candidateDeadlineUs = esp_timer_get_time() + ((int64_t)stepTimeoutMs * 1000LL);

    printf("FDC_SWEEP,stage=candidate_begin,s=%u,d=%u,index=%u,device=%s,ch=%u,drive=0x%04X,deglitch=0x%X,highCurrent=%u,candidateIndex=%lu,count=%lu,reason=%s\n",
           (unsigned)sIndex,
           (unsigned)dIndex,
           (unsigned)target.matrixIndex,
           devFullName,
           (unsigned)channel,
           outCandidate->driveCurrentNorm,
           (unsigned)deglitch->deglitchCode,
           highCurrent ? 1u : 0u,
           (unsigned long)candidateIndex,
           (unsigned long)candidateCount,
           stageName);
    printf("FDC_SWEEP,stage=%s_candidate_begin,dev=%s,ch=%u,index=%lu,count=%lu,deglitch=0x%X,deglitchName=%s,bandwidthHz=%lu,drive=0x%04X,highCurrent=%u,fastProbe=%u,confirm=%u,timeoutMs=%lu\n",
           stageName,
           devName,
           (unsigned)channel,
           (unsigned long)candidateIndex,
           (unsigned long)candidateCount,
           (unsigned)deglitch->deglitchCode,
           deglitch->deglitchName,
           (unsigned long)deglitch->deglitchBandwidthHz,
           outCandidate->driveCurrentNorm,
           highCurrent ? 1u : 0u,
           (unsigned)SENSORARRAY_FDC_SWEEP_FAST_PROBE_SAMPLES,
           (unsigned)sampleTarget,
           (unsigned long)stepTimeoutMs);

    esp_err_t firstErr = sensorarrayMeasureApplyFdcCellRoute(state, &target, stageName);
    if (firstErr != ESP_OK) {
        outCandidate->invalidSampleCount = 1u;
        printf("FDC_SWEEP,stage=candidate_reject,reason=route_failed,s=%u,d=%u,index=%u,device=%s,ch=%u,err=0x%lx\n",
               (unsigned)target.sColumn,
               (unsigned)target.dLine,
               (unsigned)target.matrixIndex,
               devFullName,
               (unsigned)channel,
               (unsigned long)firstErr);
        return firstErr;
    }

    firstErr = sensorarrayFdcSweepApplyDirectSafeLock(state,
                                                      devId,
                                                      channel,
                                                      deglitch->deglitchCode,
                                                      driveCurrent,
                                                      highCurrent,
                                                      stageName);
    if (firstErr != ESP_OK) {
        outCandidate->invalidSampleCount = 1u;
        printf("FDC_SWEEP,stage=candidate_reject,reason=lock_failed,s=%u,d=%u,index=%u,device=%s,ch=%u,err=0x%lx\n",
               (unsigned)target.sColumn,
               (unsigned)target.dLine,
               (unsigned)target.matrixIndex,
               devFullName,
               (unsigned)channel,
               (unsigned long)firstErr);
        printf("FDC_SWEEP,stage=%s_candidate_summary,dev=%s,ch=%u,index=%lu,err=%ld,stable=0,status=apply_failed\n",
               stageName,
               devName,
               (unsigned)channel,
               (unsigned long)candidateIndex,
               (long)firstErr);
        return firstErr;
    }

    esp_err_t discardErr = sensorarrayMeasureFdcDiscardStaleSamples(state,
                                                                    &target,
                                                                    SENSORARRAY_FDC_SWEEP_DISCARD_AFTER_LOCK,
                                                                    "candidate_lock");
    if (discardErr != ESP_OK) {
        outCandidate->invalidSampleCount = 1u;
        printf("FDC_SWEEP,stage=candidate_reject,reason=discard_failed,s=%u,d=%u,index=%u,device=%s,ch=%u,err=0x%lx\n",
               (unsigned)target.sColumn,
               (unsigned)target.dLine,
               (unsigned)target.matrixIndex,
               devFullName,
               (unsigned)channel,
               (unsigned long)discardErr);
        return discardErr;
    }

    uint32_t settleMs = (uint32_t)CONFIG_SENSORARRAY_FDC_SWEEP_SETTLE_MS;
    uint32_t remainingMs = sensorarrayFdcSweepRemainingMsUntil(candidateDeadlineUs);
    if (settleMs >= remainingMs) {
        outCandidate->timeoutCount++;
        return ESP_ERR_TIMEOUT;
    }
    sensorarrayFdcSweepDelayMs(settleMs);

    uint16_t sampleClockDividers = 0u;
    firstErr = Fdc2214CapReadClockDividers(fdcState->handle, channel, &sampleClockDividers);
    if (firstErr != ESP_OK) {
        outCandidate->invalidSampleCount++;
    }

    bool failFast = firstErr != ESP_OK;
    const char *failReason = failFast ? "clock_read_error" : "none";
    uint64_t rawSum = 0u;
    double freqSamples[SENSORARRAY_FDC_SWEEP_MAX_SAMPLES_PER_CANDIDATE] = {0};
    uint32_t freqCount = 0u;

    if (!failFast && discardFirst) {
        uint32_t sampleTimeoutMs = (uint32_t)CONFIG_SENSORARRAY_FDC_SWEEP_SAMPLE_TIMEOUT_MS;
        remainingMs = sensorarrayFdcSweepRemainingMsUntil(candidateDeadlineUs);
        if (sampleTimeoutMs == 0u || sampleTimeoutMs > remainingMs) {
            sampleTimeoutMs = remainingMs;
        }
        sensorarrayFdcReadDiag_t discardDiag = {0};
        sensorarrayFdcSampleEval_t discardEval = {0};
        if (sampleTimeoutMs == 0u ||
            sensorarrayFdcSweepReadOneSampleBounded(fdcState->handle,
                                                    devFullName,
                                                    channel,
                                                    sampleTimeoutMs,
                                                    &discardDiag,
                                                    &discardEval) != ESP_OK) {
            outCandidate->timeoutCount++;
            firstErr = ESP_ERR_TIMEOUT;
            failFast = true;
            failReason = "discard_timeout";
        }
    }

    for (uint32_t sampleIndex = 0u;
         !failFast && sampleIndex < sampleTarget;
         ++sampleIndex) {
        remainingMs = sensorarrayFdcSweepRemainingMsUntil(candidateDeadlineUs);
        if (remainingMs == 0u) {
            outCandidate->timeoutCount++;
            firstErr = ESP_ERR_TIMEOUT;
            failFast = true;
            failReason = "candidate_timeout";
            break;
        }

        uint32_t sampleTimeoutMs = (uint32_t)CONFIG_SENSORARRAY_FDC_SWEEP_SAMPLE_TIMEOUT_MS;
        if (sampleTimeoutMs == 0u || sampleTimeoutMs > remainingMs) {
            sampleTimeoutMs = remainingMs;
        }

        sensorarrayFdcReadDiag_t diag = {0};
        sensorarrayFdcSampleEval_t eval = {0};
        esp_err_t readErr = sensorarrayFdcSweepReadOneSampleBounded(fdcState->handle,
                                                                    devFullName,
                                                                    channel,
                                                                    sampleTimeoutMs,
                                                                    &diag,
                                                                    &eval);
        if (readErr != ESP_OK) {
            firstErr = readErr;
            if (readErr == ESP_ERR_TIMEOUT) {
                outCandidate->timeoutCount++;
            }
            outCandidate->invalidSampleCount++;
            failFast = true;
            failReason = (readErr == ESP_ERR_TIMEOUT) ? "sample_timeout" : "i2c_error";
            break;
        }

        bool saturated = eval.saturated;
        bool sampleValid = eval.sampleValid;
        outCandidate->raw28Last = diag.sample.Raw28;
        if (!eval.rawNonZero) {
            outCandidate->zeroRawCount++;
            printf("FDC_SWEEP,stage=candidate_sample_invalid,reason=zero_raw,device=%s,ch=%u,status=0x%04X,drive=0x%04X,deglitch=0x%X\n",
                   devFullName,
                   (unsigned)channel,
                   eval.status,
                   outCandidate->driveCurrentNorm,
                   (unsigned)outCandidate->deglitchReq);
        }
        if (saturated) {
            outCandidate->saturatedCount++;
        }
        if (eval.watchdogFault) {
            outCandidate->watchdogCount++;
        }
        if (eval.amplitudeFault) {
            outCandidate->amplitudeFaultCount++;
        }
        if (sampleValid) {
            outCandidate->validSampleCount++;
            if (diag.sample.Raw28 < outCandidate->raw28Min) {
                outCandidate->raw28Min = diag.sample.Raw28;
            }
            if (diag.sample.Raw28 > outCandidate->raw28Max) {
                outCandidate->raw28Max = diag.sample.Raw28;
            }
            rawSum += diag.sample.Raw28;
            if (sampleClockDividers != 0u &&
                freqCount < SENSORARRAY_FDC_SWEEP_MAX_SAMPLES_PER_CANDIDATE) {
                sensorarrayFdcFrequencyDiag_t sampleFreqDiag = {0};
                if (sensorarrayMeasureFdcComputeFrequencyDiag(diag.sample.Raw28,
                                                              sampleClockDividers,
                                                              &sampleFreqDiag) &&
                    sampleFreqDiag.valid) {
                    freqSamples[freqCount++] = sampleFreqDiag.freqHzCorrected;
                }
            }
        } else {
            outCandidate->invalidSampleCount++;
            failReason = eval.invalidReason;
        }

        if (sampleIndex + 1u >= SENSORARRAY_FDC_SWEEP_FAST_PROBE_SAMPLES) {
            if (outCandidate->timeoutCount != 0u ||
                outCandidate->watchdogCount != 0u ||
                outCandidate->saturatedCount != 0u ||
                outCandidate->zeroRawCount >= SENSORARRAY_FDC_SWEEP_FAST_PROBE_SAMPLES ||
                outCandidate->validSampleCount == 0u) {
                failFast = true;
                failReason = "fast_probe_reject";
            }
        }

        if (!failFast && sampleClockDividers != 0u && diag.sample.Raw28 != 0u) {
            sensorarrayFdcFrequencyDiag_t freqDiag = {0};
            if (sensorarrayMeasureFdcComputeFrequencyDiag(diag.sample.Raw28, sampleClockDividers, &freqDiag) &&
                freqDiag.valid &&
                deglitch->deglitchBandwidthHz > 0u &&
                freqDiag.freqHzCorrected >= ((double)deglitch->deglitchBandwidthHz * 0.90)) {
                failFast = true;
                failReason = "deglitch_bandwidth_low";
            }
        }
    }

    sensorarrayFdcSweepFinalizeRawStats(outCandidate, rawSum);
    esp_err_t regsErr = sensorarrayFdcSweepReadKeyRegs(fdcState->handle, channel, outCandidate);
    if (regsErr != ESP_OK && firstErr == ESP_OK) {
        firstErr = regsErr;
    }
    sensorarrayFdcSweepFinalizeFrequencyStats(outCandidate, freqSamples, freqCount);
    outCandidate->stable = sensorarrayFdcSweepCandidateHasStableSamples(outCandidate);

    printf("FDC_SWEEP,stage=%s_candidate_summary,dev=%s,ch=%u,index=%lu,err=%ld,stable=%u,failReason=%s,drive=0x%04X,deglitch=0x%X,freqHz=%.3f,medianFreqHz=%.3f,freqSpreadHz=%.3f,relativeSpread=%.6f,rawMin=%lu,rawMax=%lu,rawMean=%lu,valid=%lu,invalid=%lu,wd=%lu,amp=%lu,saturated=%lu,zero=%lu,timeout=%lu,score=%ld,status=%s\n",
           stageName,
           devName,
           (unsigned)channel,
           (unsigned long)candidateIndex,
           (long)firstErr,
           outCandidate->stable ? 1u : 0u,
           failReason,
           outCandidate->driveCurrentNorm,
           (unsigned)outCandidate->deglitchReq,
           outCandidate->frequencyHz,
           outCandidate->medianFreqHz,
           outCandidate->freqSpreadHz,
           outCandidate->relativeSpread,
           (unsigned long)outCandidate->raw28Min,
           (unsigned long)outCandidate->raw28Max,
           (unsigned long)outCandidate->raw28Mean,
           (unsigned long)outCandidate->validSampleCount,
           (unsigned long)outCandidate->invalidSampleCount,
           (unsigned long)outCandidate->watchdogCount,
           (unsigned long)outCandidate->amplitudeFaultCount,
           (unsigned long)outCandidate->saturatedCount,
           (unsigned long)outCandidate->zeroRawCount,
           (unsigned long)outCandidate->timeoutCount,
           (long)sensorarrayFdcSweepCandidateScore(outCandidate),
           outCandidate->stable ? "working" :
           (sensorarrayFdcSweepCandidateIsFallbackUsable(outCandidate) ? "fallback_usable" : "not_working"));

    bool accepted = sensorarrayFdcSweepCandidateIsWorking(outCandidate);
    const char *resultReason = accepted ? "valid" :
                               (outCandidate->zeroRawCount != 0u) ? "zero_raw_no_oscillation" :
                               (outCandidate->timeoutCount != 0u) ? "timeout" :
                               (outCandidate->watchdogCount != 0u) ? "status_watchdog" :
                               (outCandidate->saturatedCount != 0u) ? "saturated" :
                               (outCandidate->amplitudeFaultCount != 0u) ? "amplitude_warning" :
                               failReason;
    printf("FDC_SWEEP,stage=candidate_result,result=%s,reason=%s,device=%s,ch=%u,rawMean=%lu,freqMeanHz=%.3f,spread=%.6f,drive=0x%04X,deglitch=0x%X,highCurrent=%u,validSamples=%lu,zeroRaw=%lu,timeout=%lu,statusFault=%lu\n",
           accepted ? "accept" : "reject",
           resultReason ? resultReason : SENSORARRAY_NA,
           devFullName,
           (unsigned)channel,
           (unsigned long)outCandidate->raw28Mean,
           outCandidate->frequencyHz,
           outCandidate->relativeSpread,
           outCandidate->driveCurrentNorm,
           (unsigned)outCandidate->deglitchReq,
           outCandidate->highCurrentReq ? 1u : 0u,
           (unsigned long)outCandidate->validSampleCount,
           (unsigned long)outCandidate->zeroRawCount,
           (unsigned long)outCandidate->timeoutCount,
           (unsigned long)outCandidate->watchdogCount);

    return firstErr;
}

esp_err_t sensorarrayFdcSweepApplyResult(sensorarrayState_t *state,
                                         const sensorarrayFdcSweepResult_t *result)
{
    if (!state || !result || !result->valid || result->channel > FDC2214_CH3) {
        return ESP_ERR_INVALID_ARG;
    }
    sensorarrayFdcDeviceState_t *fdcState = sensorarrayMeasureGetFdcState(state, result->devId);
    if (!fdcState || !fdcState->ready || !fdcState->handle) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = Fdc2214CapWriteDriveCurrent(fdcState->handle,
                                                result->channel,
                                                result->selectedDriveCurrent);
    if (err != ESP_OK) {
        return err;
    }

    sensorarrayFdcSweepProfile_t *profile = &fdcState->sweepProfile[(uint8_t)result->channel];
    *profile = (sensorarrayFdcSweepProfile_t){
        .valid = true,
        .selectedDriveCurrent =
            (uint16_t)(result->selectedDriveCurrent & SENSORARRAY_FDC_SWEEP_DRIVE_CURRENT_MASK),
        .selectedHighCurrent = result->selectedHighCurrent,
        .selectedDeglitchCode = result->selectedDeglitchCode,
        .selectedDeglitchBandwidthHz = result->selectedDeglitchBandwidthHz,
        .selectedClockDividers = result->selectedClockDividers,
        .lastRaw28 = result->selectedRaw28,
        .lastFrequencyHz = result->selectedFrequencyHz,
        .lastValidTimestampUs = (uint64_t)esp_timer_get_time(),
        .quickSweepReason = SENSORARRAY_NA,
    };
    return ESP_OK;
}

esp_err_t sensorarrayFdcSweepRunChannel(sensorarrayState_t *state,
                                        sensorarrayFdcDeviceId_t devId,
                                        Fdc2214CapChannel_t channel,
                                        const char *reason,
                                        sensorarrayFdcSweepResult_t *outResult)
{
    if (!state || !outResult || channel > FDC2214_CH3) {
        return ESP_ERR_INVALID_ARG;
    }
    *outResult = (sensorarrayFdcSweepResult_t){
        .devId = devId,
        .channel = channel,
        .lastErr = ESP_ERR_NOT_SUPPORTED,
        .reason = reason ? reason : SENSORARRAY_NA,
    };
    printf("FDC_SWEEP,stage=channel_reject,scope=legacy_channel_rejected,device=%s,ch=%u,reason=cell_target_required,requestReason=%s\n",
           sensorarrayFdcSweepDevName(devId),
           (unsigned)channel,
           outResult->reason);
    return ESP_ERR_NOT_SUPPORTED;
#if 0
    sensorarrayFdcDeviceState_t *fdcState = sensorarrayMeasureGetFdcState(state, devId);
    if (!fdcState || !fdcState->ready || !fdcState->handle) {
        return ESP_ERR_INVALID_STATE;
    }

    const char *stage = (reason && strcmp(reason, "boot") == 0) ? "boot" : "quick";
    const char *devName = sensorarrayFdcSweepDevName(devId);
    uint32_t startMs = sensorarrayFdcSweepNowMs();
    uint32_t totalTimeoutMs = sensorarrayFdcSweepTotalTimeoutMs();
    double predictedFreqHz = fdcState->sweepProfile[(uint8_t)channel].lastFrequencyHz;
    const sensorarrayFdcDeglitchCandidate_t *deglitchOrder[4] = {0};
    uint16_t driveOrder[20] = {0};
    uint32_t deglitchCount = sensorarrayFdcSweepBuildDeglitchOrder(predictedFreqHz,
                                                                   deglitchOrder,
                                                                   (uint32_t)(sizeof(deglitchOrder) /
                                                                              sizeof(deglitchOrder[0])));
    uint32_t driveCount = sensorarrayFdcSweepBuildDriveOrder(&fdcState->sweepProfile[(uint8_t)channel],
                                                             driveOrder,
                                                             (uint32_t)(sizeof(driveOrder) /
                                                                        sizeof(driveOrder[0])));
    uint32_t highCurrentPasses = (strcmp(stage, "boot") == 0) ? 2u : 1u;
    uint32_t candidateCount = deglitchCount * driveCount * highCurrentPasses;

    *outResult = (sensorarrayFdcSweepResult_t){
        .devId = devId,
        .channel = channel,
        .lastErr = ESP_FAIL,
        .reason = reason ? reason : SENSORARRAY_NA,
        .candidateCount = candidateCount,
    };

    printf("FDC_SWEEP,stage=%s,reason=%s,dev=%s,ch=%u,event=start,candidates=%lu,predictedFreqHz=%.3f,timeoutMs=%lu\n",
           stage,
           outResult->reason,
           devName,
           (unsigned)channel,
           (unsigned long)candidateCount,
           predictedFreqHz,
           (unsigned long)totalTimeoutMs);

    sensorarrayFdcSweepCandidateResult_t best = {0};
    sensorarrayFdcSweepCandidateResult_t fallback = {0};
    bool haveBest = false;
    bool haveFallback = false;
    esp_err_t lastErr = ESP_OK;
    uint32_t tried = 0u;

    for (uint32_t hp = 0u; hp < highCurrentPasses; ++hp) {
        bool highCurrent = (hp != 0u);
        for (uint32_t d = 0u; d < deglitchCount; ++d) {
            for (uint32_t i = 0u; i < driveCount; ++i) {
                if ((uint32_t)(sensorarrayFdcSweepNowMs() - startMs) >= totalTimeoutMs) {
                    lastErr = ESP_ERR_TIMEOUT;
                    goto sweep_done;
                }
                sensorarrayFdcSweepCandidateResult_t candidate = {0};
                esp_err_t candidateErr = sensorarrayFdcSweepCaptureCandidate(state,
                                                                             devId,
                                                                             channel,
                                                                             deglitchOrder[d],
                                                                             driveOrder[i],
                                                                             highCurrent,
                                                                             0u,
                                                                             sensorarrayFdcSweepDLineForDeviceChannel(devId,
                                                                                                                       channel),
                                                                             stage,
                                                                             tried,
                                                                             candidateCount,
                                                                             false,
                                                                             SENSORARRAY_FDC_SWEEP_MAX_SAMPLES_PER_CANDIDATE,
                                                                             &candidate);
                if (candidateErr != ESP_OK) {
                    lastErr = candidateErr;
                }
                tried++;
                if (sensorarrayFdcSweepCandidateIsWorking(&candidate)) {
                    if (!haveBest || sensorarrayFdcSweepCandidateBetter(&candidate, &best)) {
                        best = candidate;
                        haveBest = true;
                    }
                } else if (sensorarrayFdcSweepFallbackBetter(&candidate, &fallback)) {
                    fallback = candidate;
                    haveFallback = true;
                }
                if (haveBest && !haveFallback && candidate.driveCurrentNorm <= 0x7000u && !highCurrent) {
                    /* Low-current stable candidates are preferred for the observed overdrive case. */
                    goto sweep_done;
                }
            }
        }
    }

sweep_done:
    outResult->elapsedMs = sensorarrayFdcSweepNowMs() - startMs;
    const sensorarrayFdcSweepCandidateResult_t *selected = NULL;
    if (haveBest) {
        selected = &best;
    }

    if (!selected) {
        outResult->lastErr = (lastErr != ESP_OK) ? lastErr : ESP_ERR_INVALID_RESPONSE;
        printf("FDC_SWEEP,stage=%s,reason=%s,dev=%s,ch=%u,event=result,result=failed,err=%ld,tried=%lu,elapsedMs=%lu\n",
               stage,
               outResult->reason,
               devName,
               (unsigned)channel,
               (long)outResult->lastErr,
               (unsigned long)tried,
               (unsigned long)outResult->elapsedMs);
        return outResult->lastErr;
    }

    sensorarrayFdcSweepCandidateResult_t selectedCopy = *selected;
    selectedCopy.selected = true;
    esp_err_t applyErr = sensorarrayFdcSweepApplyDirectSafeLock(state,
                                                                devId,
                                                                channel,
                                                                selectedCopy.deglitchReq,
                                                                selectedCopy.driveCurrentNorm,
                                                                selectedCopy.highCurrentReq,
                                                                stage);
    if (applyErr != ESP_OK) {
        outResult->lastErr = applyErr;
        return applyErr;
    }

    outResult->valid = true;
    outResult->selectedDeglitchCode = selectedCopy.deglitchReq;
    outResult->selectedDeglitchBandwidthHz = selectedCopy.deglitchBandwidthHz;
    outResult->selectedDeglitchName = selectedCopy.deglitchName;
    outResult->selectedDriveCurrent = selectedCopy.driveCurrentNorm;
    outResult->selectedHighCurrent = selectedCopy.highCurrentReq;
    outResult->selectedConfig = selectedCopy.configReg;
    outResult->selectedMuxConfig = selectedCopy.muxConfigReg;
    outResult->selectedClockDividers = selectedCopy.clockDividersReg;
    outResult->selectedRaw28 = selectedCopy.raw28Last ? selectedCopy.raw28Last : selectedCopy.raw28Mean;
    outResult->selectedFrequencyHz = selectedCopy.frequencyHz;
    outResult->lastErr = ESP_OK;

    esp_err_t cacheErr = sensorarrayFdcSweepApplyResult(state, outResult);
    if (cacheErr != ESP_OK) {
        outResult->lastErr = cacheErr;
        return cacheErr;
    }

    printf("FDC_SWEEP,stage=%s,reason=%s,dev=%s,ch=%u,event=result,bestDrive=0x%04X,deglitch=0x%X,deglitchName=%s,freqHz=%.3f,raw28=%lu,fallback=%u,result=%s,err=%ld,tried=%lu,elapsedMs=%lu\n",
           stage,
           outResult->reason,
           devName,
           (unsigned)channel,
           outResult->selectedDriveCurrent,
           (unsigned)outResult->selectedDeglitchCode,
           outResult->selectedDeglitchName ? outResult->selectedDeglitchName : SENSORARRAY_NA,
           outResult->selectedFrequencyHz,
           (unsigned long)outResult->selectedRaw28,
           0u,
           "ok",
           (long)outResult->lastErr,
           (unsigned long)tried,
           (unsigned long)outResult->elapsedMs);

    return ESP_OK;
#endif
}

static uint32_t sensorarrayFdcSweepDeglitchBandwidthForProfile(const sensorarrayFdcSweepProfile_t *profile)
{
    if (!profile || !profile->valid || profile->selectedDeglitchBandwidthHz == 0u) {
        return SENSORARRAY_FDC_SWEEP_DEFAULT_DEGLITCH_BW_HZ;
    }
    return profile->selectedDeglitchBandwidthHz;
}

esp_err_t sensorarrayFdcSweepRestoreAutoscan(sensorarrayState_t *state,
                                             sensorarrayFdcDeviceId_t devId,
                                             const char *reason)
{
    sensorarrayFdcDeviceState_t *fdcState = sensorarrayMeasureGetFdcState(state, devId);
    if (!fdcState || !fdcState->ready || !fdcState->handle) {
        return ESP_ERR_INVALID_STATE;
    }

    const char *devName = sensorarrayFdcSweepDevFullName(devId);
    printf("FDC_LOCK,stage=autoscan_restore_begin,device=%s,reason=%s\n",
           devName,
           reason ? reason : SENSORARRAY_NA);

    uint32_t selectedBw = SENSORARRAY_FDC_SWEEP_DEFAULT_DEGLITCH_BW_HZ;
    uint8_t selectedCode = SENSORARRAY_FDC_SWEEP_DEFAULT_DEGLITCH_REQ;
    for (uint8_t ch = 0u; ch < 4u; ++ch) {
        uint32_t bw = sensorarrayFdcSweepDeglitchBandwidthForProfile(&fdcState->sweepProfile[ch]);
        if (bw > selectedBw) {
            selectedBw = bw;
            selectedCode = fdcState->sweepProfile[ch].selectedDeglitchCode;
        }
    }

    Fdc2214CapDeglitch_t deglitch = sensorarrayFdcSweepDeglitchEnum(selectedCode);
    esp_err_t err = Fdc2214CapSetAutoScanMode(fdcState->handle, 2u, deglitch);
    if (err == ESP_OK) {
        err = Fdc2214CapClearStatus(fdcState->handle);
    }
    Fdc2214CapCoreRegs_t regs = {0};
    if (err == ESP_OK) {
        err = Fdc2214CapReadCoreRegs(fdcState->handle, &regs);
        if (err == ESP_OK) {
            fdcState->statusConfigReg = regs.StatusConfig;
            fdcState->configReg = regs.Config;
            fdcState->muxConfigReg = regs.MuxConfig;
        }
    }

    uint16_t statusReg = regs.Status;
    uint16_t configReg = regs.Config;
    uint16_t muxReg = regs.MuxConfig;
    if (err != ESP_OK) {
        (void)sensorarrayFdcSweepDumpDeviceRegs(fdcState, "autoscan_restore_failed", reason);
    }

    printf("FDC_LOCK,stage=autoscan_restore_done,device=%s,config=0x%04X,mux=0x%04X,status=0x%04X,err=0x%lx\n",
           devName,
           configReg,
           muxReg,
           statusReg,
           (unsigned long)err);
    printf("FDC_SWEEP,stage=restore_autoscan,reason=%s,dev=%s,deglitch=0x%X,deglitchName=%s,bandwidthHz=%lu,err=%ld,status=%s\n",
           reason ? reason : SENSORARRAY_NA,
           sensorarrayFdcSweepDevName(devId),
           (unsigned)selectedCode,
           sensorarrayFdcSweepDeglitchNameFromCode(selectedCode),
           (unsigned long)selectedBw,
           (long)err,
           (err == ESP_OK) ? "autoscan_ch0_ch3" : "restore_failed");
    return err;
}

esp_err_t sensorarrayFdcSweepRunDevice(sensorarrayState_t *state,
                                       sensorarrayFdcDeviceId_t devId,
                                       uint8_t channelMask,
                                       const char *reason)
{
    if (!state || (channelMask & 0xF0u) != 0u) {
        return ESP_ERR_INVALID_ARG;
    }
    sensorarrayFdcDeviceState_t *fdcState = sensorarrayMeasureGetFdcState(state, devId);
    const char *devName = sensorarrayFdcSweepDevFullName(devId);
    printf("FDC_SWEEP,stage=device_begin,device=%s,addr=0x%02X,channels=0x%X,reason=%s\n",
           devName,
           fdcState ? fdcState->i2cAddr : 0u,
           (unsigned)channelMask,
           reason ? reason : SENSORARRAY_NA);
    if (!fdcState || !fdcState->ready || !fdcState->handle) {
        printf("FDC_SWEEP,stage=device_skip,reason=not_ready,device=%s\n", devName);
        return ESP_ERR_INVALID_STATE;
    }
    esp_err_t firstErr = ESP_OK;
    uint32_t validChannels = 0u;
    uint32_t failedChannels = 0u;
    for (uint8_t ch = 0u; ch < 4u; ++ch) {
        if ((channelMask & (uint8_t)(1u << ch)) == 0u) {
            continue;
        }
        sensorarrayFdcSweepResult_t result = {0};
        printf("FDC_SWEEP,stage=channel_begin,device=%s,ch=%u,d=%u\n",
               devName,
               (unsigned)ch,
               (unsigned)sensorarrayFdcSweepDLineForDeviceChannel(devId, (Fdc2214CapChannel_t)ch));
        esp_err_t err = sensorarrayFdcSweepRunChannel(state,
                                                      devId,
                                                      (Fdc2214CapChannel_t)ch,
                                                      reason,
                                                      &result);
        if (err == ESP_OK && result.valid && result.selectedRaw28 != 0u) {
            validChannels++;
        } else {
            failedChannels++;
            printf("FDC_SWEEP,stage=channel_failed,reason=no_oscillation,device=%s,ch=%u,err=0x%lx\n",
                   devName,
                   (unsigned)ch,
                   (unsigned long)err);
        }
        if (err != ESP_OK && firstErr == ESP_OK) {
            firstErr = err;
        }
    }

    esp_err_t restoreErr = sensorarrayFdcSweepRestoreAutoscan(state, devId, reason);
    if (restoreErr != ESP_OK && firstErr == ESP_OK) {
        firstErr = restoreErr;
    }
    printf("FDC_SWEEP,stage=device_done,device=%s,validChannels=%lu,failedChannels=%lu,zeroRaw=%lu,noOscillation=%lu,err=0x%lx\n",
           devName,
           (unsigned long)validChannels,
           (unsigned long)failedChannels,
           (unsigned long)failedChannels,
           (unsigned long)failedChannels,
           (unsigned long)((validChannels == 0u && firstErr == ESP_OK) ? ESP_ERR_INVALID_STATE : firstErr));
    if (validChannels == 0u) {
        return (firstErr != ESP_OK) ? firstErr : ESP_ERR_INVALID_STATE;
    }
    return firstErr;
}

typedef enum {
    SENSORARRAY_FDC_CELL_SWEEP_BOOT_FULL = 0,
    SENSORARRAY_FDC_CELL_SWEEP_FAST,
    SENSORARRAY_FDC_CELL_SWEEP_FULL_RESCUE,
} sensorarrayFdcCellSweepMode_t;

static sensorarrayFdcCacheSource_t sensorarrayFdcSweepCacheSourceForMode(sensorarrayFdcCellSweepMode_t mode)
{
    switch (mode) {
    case SENSORARRAY_FDC_CELL_SWEEP_BOOT_FULL:
        return SENSORARRAY_FDC_CACHE_SOURCE_BOOT_FULL;
    case SENSORARRAY_FDC_CELL_SWEEP_FAST:
        return SENSORARRAY_FDC_CACHE_SOURCE_FAST_RESCUE;
    case SENSORARRAY_FDC_CELL_SWEEP_FULL_RESCUE:
        return SENSORARRAY_FDC_CACHE_SOURCE_MANUAL_FULL;
    default:
        return SENSORARRAY_FDC_CACHE_SOURCE_NONE;
    }
}

static const char *sensorarrayFdcSweepCacheSourceName(sensorarrayFdcCacheSource_t source)
{
    switch (source) {
    case SENSORARRAY_FDC_CACHE_SOURCE_BOOT_FULL:
        return "boot_full";
    case SENSORARRAY_FDC_CACHE_SOURCE_MANUAL_FULL:
        return "manual_full";
    case SENSORARRAY_FDC_CACHE_SOURCE_FAST_RESCUE:
        return "fast_rescue";
    case SENSORARRAY_FDC_CACHE_SOURCE_LAST_GOOD:
        return "last_good";
    case SENSORARRAY_FDC_CACHE_SOURCE_NONE:
    default:
        return "none";
    }
}

static void sensorarrayFdcSweepIncrementFailCount(uint8_t *count)
{
    if (count && *count < 0xFFu) {
        (*count)++;
    }
}

static void sensorarrayFdcSweepIncrementFailCount16(uint16_t *count)
{
    if (count && *count < UINT16_MAX) {
        (*count)++;
    }
}

static bool sensorarrayFdcSweepCooldownElapsed(uint32_t nowMs, uint32_t lastMs, uint32_t cooldownMs)
{
    return cooldownMs == 0u || lastMs == 0u || (uint32_t)(nowMs - lastMs) >= cooldownMs;
}

static esp_err_t sensorarrayFdcSweepPrepareCellPath(sensorarrayState_t *state,
                                                    const sensorarrayFdcCellCalibration_t *cal,
                                                    const char *reason)
{
    if (!state || !cal) {
        return ESP_ERR_INVALID_ARG;
    }

    sensorarrayFdcCellTarget_t target = {0};
    if (!sensorarrayMeasureMakeFdcCellTarget(state, cal->sIndex, cal->dIndex, &target)) {
        return ESP_ERR_INVALID_ARG;
    }
    return sensorarrayMeasureApplyFdcCellRoute(state, &target, reason);
}

static void sensorarrayFdcSweepUpdateCalibrationFromCandidate(sensorarrayFdcCellCalibration_t *cal,
                                                              const sensorarrayFdcSweepCandidateResult_t *candidate,
                                                              bool bootSweep,
                                                              bool lastGood)
{
    if (!cal || !candidate) {
        return;
    }

    double capPf = 0.0;
    bool capValid = sensorarrayFdcSweepCapPfFromFreq(candidate->frequencyHz, &capPf);
    uint32_t raw28 = candidate->raw28Last ? candidate->raw28Last : candidate->raw28Mean;
    int score = (int)sensorarrayFdcSweepCandidateScore(candidate);

    if (bootSweep) {
        cal->hasBootSweep = true;
        cal->bootDeglitch = sensorarrayFdcSweepDeglitchEnum(candidate->deglitchReq);
        cal->bootDriveCurrent = candidate->driveCurrentNorm;
        cal->bootRaw28 = raw28;
        cal->bootFreqHz = candidate->frequencyHz;
        cal->bootCapPf = capValid ? capPf : 0.0;
        cal->bootQualityScore = score;
    }

    if (lastGood) {
        cal->hasLastGood = true;
        cal->lockValid = true;
        cal->lastGoodDeglitch = sensorarrayFdcSweepDeglitchEnum(candidate->deglitchReq);
        cal->lastGoodDriveCurrent = candidate->driveCurrentNorm;
        cal->lastGoodHighCurrent = candidate->highCurrentReq;
        cal->lastGoodRaw28 = raw28;
        cal->lastGoodFreqHz = candidate->frequencyHz;
        cal->lastGoodTimestampUs = (uint64_t)esp_timer_get_time();
        cal->lastGoodCapPf = capValid ? capPf : 0.0;
        cal->lastGoodQualityScore = score;
        cal->consecutiveFailCount = 0u;
        cal->consecutiveNoUnreadCount = 0u;
        cal->consecutiveStatusFaultCount = 0u;
        cal->consecutiveZeroRawCount = 0u;
        cal->directFailCount = 0u;
        cal->fastSweepFailCount = 0u;
        cal->fullSweepFailCount = 0u;
        cal->lastFailReason = "valid";
    }
}

static void sensorarrayFdcSweepUpdateDeviceProfileFromCandidate(sensorarrayState_t *state,
                                                                sensorarrayFdcDeviceId_t devId,
                                                                Fdc2214CapChannel_t channel,
                                                                const sensorarrayFdcSweepCandidateResult_t *candidate)
{
    sensorarrayFdcDeviceState_t *fdcState = sensorarrayMeasureGetFdcState(state, devId);
    if (!fdcState || channel > FDC2214_CH3 || !candidate) {
        return;
    }

    uint32_t raw28 = candidate->raw28Last ? candidate->raw28Last : candidate->raw28Mean;
    sensorarrayFdcSweepProfile_t *profile = &fdcState->sweepProfile[(uint8_t)channel];
    *profile = (sensorarrayFdcSweepProfile_t){
        .valid = true,
        .selectedDriveCurrent =
            (uint16_t)(candidate->driveCurrentNorm & SENSORARRAY_FDC_SWEEP_DRIVE_CURRENT_MASK),
        .selectedHighCurrent = candidate->highCurrentReq,
        .selectedDeglitchCode = candidate->deglitchReq,
        .selectedDeglitchBandwidthHz = candidate->deglitchBandwidthHz,
        .selectedClockDividers = candidate->clockDividersReg,
        .lastRaw28 = raw28,
        .lastFrequencyHz = candidate->frequencyHz,
        .lastValidTimestampUs = (uint64_t)esp_timer_get_time(),
        .quickSweepReason = SENSORARRAY_NA,
    };
}

static void sensorarrayFdcSweepUpdateStateCacheFromCandidate(sensorarrayState_t *state,
                                                             const sensorarrayFdcCellCalibration_t *cal,
                                                             const sensorarrayFdcSweepCandidateResult_t *candidate,
                                                             sensorarrayFdcCacheSource_t source)
{
    if (!state || !cal || !candidate) {
        return;
    }

    sensorarrayFdcCellTarget_t target = {0};
    if (!sensorarrayMeasureMakeFdcCellTarget(state, cal->sIndex, cal->dIndex, &target)) {
        return;
    }
    sensorarrayFdcCellConfigCache_t *cache = sensorarrayMeasureGetFdcCellCache(state, &target);
    if (!cache) {
        return;
    }

    uint32_t raw28 = candidate->raw28Last ? candidate->raw28Last : candidate->raw28Mean;
    int64_t nowUs = esp_timer_get_time();
    uint32_t generation = cache->generation + 1u;
    if (generation == 0u) {
        generation = 1u;
    }

    cache->valid = true;
    cache->source = source;
    cache->rCount = candidate->rCountReg;
    cache->settleCount = candidate->settleCountReg;
    cache->clockDiv = candidate->clockDividersReg;
    cache->driveCurrent = candidate->driveCurrentNorm;
    cache->deglitchCode = candidate->deglitchReq;
    cache->highCurrentObserved = candidate->highCurrentReq || candidate->highCurrentReadback;
    cache->effectiveFclkHz = candidate->effectiveFclkHz ? candidate->effectiveFclkHz :
        sensorarrayMeasureFdcEffectiveFclkHz();
    cache->lastRaw28 = raw28;
    cache->lastFreqHz = candidate->frequencyHz;
    cache->qualityScore = candidate->validSampleCount * 100u +
        (candidate->stable ? 50u : 0u) +
        (candidate->deglitchBandwidthOk ? 25u : 0u);
    cache->generation = generation;
    cache->storedTimestampUs = nowUs;
    cache->lastGoodTimestampUs = nowUs;
    cache->consecutiveAmplitudeWarnings = 0u;
    cache->consecutiveErrors = 0u;
    cache->consecutiveNoUnread = 0u;
    cache->consecutiveZeroRaw = 0u;
    cache->consecutiveWatchdogFaults = 0u;
    cache->consecutiveI2cErrors = 0u;
    cache->reapplyPending = false;
    cache->rescuePending = false;
    cache->lastWarningReason[0] = '\0';
    cache->lastRescueReason[0] = '\0';
    cache->fastRescueFailCount = 0u;

    if (target.devId <= SENSORARRAY_FDC_DEV_SECONDARY) {
        sensorarrayFdcAppliedRowConfig_t *applied = &state->fdcAppliedRow[(uint8_t)target.devId];
        if (applied->valid &&
            applied->row == target.sColumn &&
            applied->deviceId == (uint8_t)target.devId) {
            applied->dirty = true;
        }
    }

    printf("FDC_CACHE,stage=store,source=%s,s=%u,d=%u,device=%s,ch=%u,valid=1,drive=0x%04X,rCount=0x%04X,settle=0x%04X,clockDiv=0x%04X,deglitch=0x%X,freqHz=%.3f,raw28=%lu,quality=%lu,generation=%lu\n",
           sensorarrayFdcSweepCacheSourceName(source),
           (unsigned)cal->sIndex,
           (unsigned)cal->dIndex,
           sensorarrayFdcSweepDevName((sensorarrayFdcDeviceId_t)cal->fdcDevice),
           (unsigned)cal->fdcChannel,
           cache->driveCurrent,
           cache->rCount,
           cache->settleCount,
           cache->clockDiv,
           (unsigned)cache->deglitchCode,
           cache->lastFreqHz,
           (unsigned long)cache->lastRaw28,
           (unsigned long)cache->qualityScore,
           (unsigned long)cache->generation);
}

static void sensorarrayFdcSweepLogCapResult(const sensorarrayFdcCellCalibration_t *cal,
                                            const sensorarrayFdcSweepCandidateResult_t *candidate)
{
    if (!cal || !candidate) {
        return;
    }

    double capPf = 0.0;
    bool capValid = sensorarrayFdcSweepCapPfFromFreq(candidate->frequencyHz, &capPf);
    uint32_t raw28 = candidate->raw28Last ? candidate->raw28Last : candidate->raw28Mean;
    printf("FDC_CAP_RESULT,s=%u,d=%u,dev=%s,ch=%u,freqHz=%.3f,capPf=%.6f,capValid=%u,raw28=%lu,drive=0x%04X,deglitch=0x%X\n",
           (unsigned)cal->sIndex,
           (unsigned)cal->dIndex,
           sensorarrayFdcSweepDevName((sensorarrayFdcDeviceId_t)cal->fdcDevice),
           (unsigned)cal->fdcChannel,
           candidate->frequencyHz,
           capValid ? capPf : 0.0,
           capValid ? 1u : 0u,
           (unsigned long)raw28,
           candidate->driveCurrentNorm,
           (unsigned)candidate->deglitchReq);
}

static bool sensorarrayFdcSweepCandidateHugeJump(const sensorarrayFdcCellCalibration_t *cal,
                                                 const sensorarrayFdcSweepCandidateResult_t *candidate)
{
    if (!cal || !candidate || !cal->hasLastGood ||
        cal->lastGoodFreqHz <= 0.0 || candidate->frequencyHz <= 0.0) {
        return false;
    }

    double deltaHz = sensorarrayFdcSweepAbsDouble(candidate->frequencyHz - cal->lastGoodFreqHz);
    double ratio = deltaHz / cal->lastGoodFreqHz;
    return deltaHz > SENSORARRAY_FDC_SWEEP_FREQ_JUMP_ABS_HZ ||
           ratio > SENSORARRAY_FDC_SWEEP_FREQ_JUMP_RATIO;
}

static uint32_t sensorarrayFdcSweepBuildFastDriveOrderForCell(const sensorarrayFdcCellCalibration_t *cal,
                                                              uint16_t *drives,
                                                              uint32_t capacity,
                                                              uint16_t *outCenter)
{
    uint16_t center = SENSORARRAY_FDC_SWEEP_DEFAULT_DRIVE_CURRENT_REQ;
    if (cal && cal->hasLastGood) {
        center = cal->lastGoodDriveCurrent;
    } else if (cal && cal->hasBootSweep) {
        center = cal->bootDriveCurrent;
    }
    center = sensorarrayFdcSweepClampDrive(center);
    if (outCenter) {
        *outCenter = center;
    }

    uint32_t count = 0u;
    sensorarrayFdcSweepAddUniqueDrive(drives, &count, capacity, center);
    sensorarrayFdcSweepAddUniqueDrive(drives,
                                      &count,
                                      capacity,
                                      sensorarrayFdcSweepAddDriveOffset(center,
                                                                        -(int32_t)SENSORARRAY_FDC_SWEEP_FAST_DRIVE_STEP_SMALL));
    sensorarrayFdcSweepAddUniqueDrive(drives,
                                      &count,
                                      capacity,
                                      sensorarrayFdcSweepAddDriveOffset(center,
                                                                        (int32_t)SENSORARRAY_FDC_SWEEP_FAST_DRIVE_STEP_SMALL));
    sensorarrayFdcSweepAddUniqueDrive(drives,
                                      &count,
                                      capacity,
                                      sensorarrayFdcSweepAddDriveOffset(center,
                                                                        -(int32_t)SENSORARRAY_FDC_SWEEP_FAST_DRIVE_STEP_LARGE));
    sensorarrayFdcSweepAddUniqueDrive(drives,
                                      &count,
                                      capacity,
                                      sensorarrayFdcSweepAddDriveOffset(center,
                                                                        (int32_t)SENSORARRAY_FDC_SWEEP_FAST_DRIVE_STEP_LARGE));
    return count;
}

static uint32_t sensorarrayFdcSweepBuildFullDriveOrderForCell(const sensorarrayFdcCellCalibration_t *cal,
                                                              uint16_t *drives,
                                                              uint32_t capacity)
{
    uint32_t count = 0u;
    if (cal && cal->hasLastGood) {
        sensorarrayFdcSweepAddUniqueDrive(drives, &count, capacity, cal->lastGoodDriveCurrent);
    }
    if (cal && cal->hasBootSweep) {
        sensorarrayFdcSweepAddUniqueDrive(drives, &count, capacity, cal->bootDriveCurrent);
    }
    for (size_t i = 0u; i < (sizeof(SENSORARRAY_FDC_DRIVE_CURRENT_SWEEP_TABLE) /
                             sizeof(SENSORARRAY_FDC_DRIVE_CURRENT_SWEEP_TABLE[0])); ++i) {
        sensorarrayFdcSweepAddUniqueDrive(drives, &count, capacity, SENSORARRAY_FDC_DRIVE_CURRENT_SWEEP_TABLE[i]);
    }
    return count;
}

static uint32_t sensorarrayFdcSweepBuildFastDeglitchOrderForCell(const sensorarrayFdcCellCalibration_t *cal,
                                                                 const sensorarrayFdcDeglitchCandidate_t **order,
                                                                 uint32_t capacity,
                                                                 uint8_t *outCenter)
{
    uint8_t center = SENSORARRAY_FDC_SWEEP_DEFAULT_DEGLITCH_REQ;
    if (cal && cal->hasLastGood) {
        center = (uint8_t)cal->lastGoodDeglitch;
    } else if (cal && cal->hasBootSweep) {
        center = (uint8_t)cal->bootDeglitch;
    }
    if (outCenter) {
        *outCenter = center;
    }

    uint32_t count = 0u;
    sensorarrayFdcSweepAddUniqueDeglitch(order,
                                         &count,
                                         capacity,
                                         sensorarrayFdcSweepFindDeglitchCandidate(center));
    if (cal && !cal->hasLastGood && !cal->hasBootSweep) {
        sensorarrayFdcSweepAddUniqueDeglitch(order,
                                             &count,
                                             capacity,
                                             sensorarrayFdcSweepFindDeglitchCandidate(FDC2214_DEGLITCH_10MHZ));
    }
    return count;
}

static uint32_t sensorarrayFdcSweepBuildFullDeglitchOrderForCell(const sensorarrayFdcCellCalibration_t *cal,
                                                                 const sensorarrayFdcDeglitchCandidate_t **order,
                                                                 uint32_t capacity)
{
    double predictedFreqHz = 0.0;
    if (cal && cal->hasLastGood) {
        predictedFreqHz = cal->lastGoodFreqHz;
    } else if (cal && cal->hasBootSweep) {
        predictedFreqHz = cal->bootFreqHz;
    }
    return sensorarrayFdcSweepBuildDeglitchOrder(predictedFreqHz, order, capacity);
}

static const char *sensorarrayFdcSweepModeStage(sensorarrayFdcCellSweepMode_t mode)
{
    switch (mode) {
    case SENSORARRAY_FDC_CELL_SWEEP_BOOT_FULL:
        return "boot_full";
    case SENSORARRAY_FDC_CELL_SWEEP_FAST:
        return "fast";
    case SENSORARRAY_FDC_CELL_SWEEP_FULL_RESCUE:
        return "full_rescue";
    default:
        return "cell";
    }
}

static esp_err_t sensorarrayFdcSweepRunCellSweep(sensorarrayState_t *state,
                                                 sensorarrayFdcCellCalibration_t *cal,
                                                 sensorarrayFdcCellSweepMode_t mode,
                                                 const char *reason,
                                                 sensorarrayFdcSweepCandidateResult_t *outBest,
                                                 bool *outAccepted)
{
    if (outBest) {
        *outBest = (sensorarrayFdcSweepCandidateResult_t){0};
    }
    if (outAccepted) {
        *outAccepted = false;
    }
    if (!state || !cal) {
        return ESP_ERR_INVALID_ARG;
    }

    sensorarrayFdcDeviceId_t devId = (sensorarrayFdcDeviceId_t)cal->fdcDevice;
    Fdc2214CapChannel_t channel = (Fdc2214CapChannel_t)cal->fdcChannel;
    sensorarrayFdcDeviceState_t *fdcState = sensorarrayMeasureGetFdcState(state, devId);
    if (!fdcState || !fdcState->ready || !fdcState->handle || channel > FDC2214_CH3) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t pathErr = sensorarrayFdcSweepPrepareCellPath(state, cal, reason);
    if (pathErr != ESP_OK) {
        return pathErr;
    }

    const char *stage = sensorarrayFdcSweepModeStage(mode);
    const sensorarrayFdcDeglitchCandidate_t *deglitchOrder[4] = {0};
    uint16_t driveOrder[20] = {0};
    uint16_t centerDrive = 0u;
    uint8_t centerDeglitch = 0u;
    uint32_t deglitchCount = 0u;
    uint32_t driveCount = 0u;

    if (mode == SENSORARRAY_FDC_CELL_SWEEP_FAST) {
        driveCount = sensorarrayFdcSweepBuildFastDriveOrderForCell(cal,
                                                                   driveOrder,
                                                                   (uint32_t)(sizeof(driveOrder) /
                                                                              sizeof(driveOrder[0])),
                                                                   &centerDrive);
        deglitchCount = sensorarrayFdcSweepBuildFastDeglitchOrderForCell(cal,
                                                                         deglitchOrder,
                                                                         (uint32_t)(sizeof(deglitchOrder) /
                                                                                    sizeof(deglitchOrder[0])),
                                                                         &centerDeglitch);
        printf("FDC_FAST_SWEEP_START,s=%u,d=%u,centerDrive=0x%04X,centerDeglitch=0x%X\n",
               (unsigned)cal->sIndex,
               (unsigned)cal->dIndex,
               centerDrive,
               (unsigned)centerDeglitch);
    } else {
        driveCount = sensorarrayFdcSweepBuildFullDriveOrderForCell(cal,
                                                                   driveOrder,
                                                                   (uint32_t)(sizeof(driveOrder) /
                                                                              sizeof(driveOrder[0])));
        deglitchCount = sensorarrayFdcSweepBuildFullDeglitchOrderForCell(cal,
                                                                         deglitchOrder,
                                                                         (uint32_t)(sizeof(deglitchOrder) /
                                                                                    sizeof(deglitchOrder[0])));
        if (mode == SENSORARRAY_FDC_CELL_SWEEP_FULL_RESCUE) {
            printf("FDC_FULL_SWEEP_RESCUE_START,s=%u,d=%u,reason=%s\n",
                   (unsigned)cal->sIndex,
                   (unsigned)cal->dIndex,
                   reason ? reason : SENSORARRAY_NA);
        }
    }

    uint32_t highCurrentPasses = (mode == SENSORARRAY_FDC_CELL_SWEEP_FAST) ? 1u : 2u;
    uint32_t candidateCount = deglitchCount * driveCount * highCurrentPasses;
    uint32_t startMs = sensorarrayFdcSweepNowMs();
    uint32_t totalTimeoutMs = sensorarrayFdcSweepTotalTimeoutMs();
    sensorarrayFdcSweepCandidateResult_t best = {0};
    sensorarrayFdcSweepCandidateResult_t diagnostic = {0};
    bool haveBest = false;
    bool haveDiagnostic = false;
    esp_err_t lastErr = ESP_OK;
    uint32_t tried = 0u;

    for (uint32_t hp = 0u; hp < highCurrentPasses; ++hp) {
        bool highCurrent = (hp != 0u);
        for (uint32_t d = 0u; d < deglitchCount; ++d) {
            for (uint32_t i = 0u; i < driveCount; ++i) {
                if ((uint32_t)(sensorarrayFdcSweepNowMs() - startMs) >= totalTimeoutMs) {
                    lastErr = ESP_ERR_TIMEOUT;
                    goto cell_sweep_done;
                }
                sensorarrayFdcSweepCandidateResult_t candidate = {0};
                esp_err_t candidateErr = sensorarrayFdcSweepCaptureCandidate(state,
                                                                             devId,
                                                                             channel,
                                                                             deglitchOrder[d],
                                                                             driveOrder[i],
                                                                             highCurrent,
                                                                             cal->sIndex,
                                                                             cal->dIndex,
                                                                             stage,
                                                                             tried,
                                                                             candidateCount,
                                                                             false,
                                                                             SENSORARRAY_FDC_SWEEP_MAX_SAMPLES_PER_CANDIDATE,
                                                                             &candidate);
                if (candidateErr != ESP_OK) {
                    lastErr = candidateErr;
                }
                tried++;

                if (sensorarrayFdcSweepCandidateIsWorking(&candidate)) {
                    if (!haveBest || sensorarrayFdcSweepCandidateBetter(&candidate, &best)) {
                        best = candidate;
                        haveBest = true;
                    }
                    if (mode == SENSORARRAY_FDC_CELL_SWEEP_FAST) {
                        printf("FDC_SWEEP,stage=quick_accept_and_stop,s=%u,d=%u,index=%u,device=%s,ch=%u,drive=0x%04X,deglitch=0x%X,freqHz=%.3f,rawMean=%lu,relativeSpread=%.6f\n",
                               (unsigned)cal->sIndex,
                               (unsigned)cal->dIndex,
                               (unsigned)sensorarrayMatrixIndex(cal->sIndex, cal->dIndex),
                               sensorarrayFdcSweepDevName(devId),
                               (unsigned)channel,
                               candidate.driveCurrentNorm,
                               (unsigned)candidate.deglitchReq,
                               candidate.frequencyHz,
                               (unsigned long)candidate.raw28Mean,
                               candidate.relativeSpread);
                        goto cell_sweep_done;
                    }
                } else if (sensorarrayFdcSweepFallbackBetter(&candidate, &diagnostic)) {
                    diagnostic = candidate;
                    haveDiagnostic = true;
                }
            }
        }
    }

cell_sweep_done:
    cal->lastSweepMs = sensorarrayFdcSweepNowMs();
    if (mode == SENSORARRAY_FDC_CELL_SWEEP_FAST) {
        cal->lastFastSweepTimestampUs = (uint64_t)esp_timer_get_time();
    } else if (mode == SENSORARRAY_FDC_CELL_SWEEP_FULL_RESCUE) {
        cal->lastFullSweepTimestampUs = (uint64_t)esp_timer_get_time();
    }

    bool useFallback = !haveBest &&
                       haveDiagnostic &&
                       sensorarrayFdcSweepCandidateIsFallbackUsable(&diagnostic);
    sensorarrayFdcSweepCandidateResult_t selected = haveBest ? best : diagnostic;
    const char *selectedStage = haveBest ? stage : "fallback";
    if (haveBest || useFallback) {
        printf("FDC_SWEEP,stage=restore_best_candidate,s=%u,d=%u,index=%u,device=%s,ch=%u,drive=0x%04X,deglitch=0x%X,result=%s\n",
               (unsigned)cal->sIndex,
               (unsigned)cal->dIndex,
               (unsigned)sensorarrayMatrixIndex(cal->sIndex, cal->dIndex),
               sensorarrayFdcSweepDevName(devId),
               (unsigned)channel,
               selected.driveCurrentNorm,
               (unsigned)selected.deglitchReq,
               haveBest ? "accepted" : "fallback");
        esp_err_t applyErr = sensorarrayFdcSweepApplyDirectSafeLock(state,
                                                                    devId,
                                                                    channel,
                                                                    selected.deglitchReq,
                                                                    selected.driveCurrentNorm,
                                                                    selected.highCurrentReq,
                                                                    selectedStage);
        if (applyErr != ESP_OK) {
            return applyErr;
        }
        sensorarrayFdcSweepUpdateCalibrationFromCandidate(cal,
                                                          &selected,
                                                          mode == SENSORARRAY_FDC_CELL_SWEEP_BOOT_FULL,
                                                          true);
        sensorarrayFdcSweepUpdateDeviceProfileFromCandidate(state, devId, channel, &selected);
        sensorarrayFdcSweepUpdateStateCacheFromCandidate(state,
                                                         cal,
                                                         &selected,
                                                         sensorarrayFdcSweepCacheSourceForMode(mode));
        if (outBest) {
            *outBest = selected;
        }
        if (outAccepted) {
            *outAccepted = true;
        }

        if (mode == SENSORARRAY_FDC_CELL_SWEEP_BOOT_FULL) {
            double capPf = 0.0;
            bool capValid = sensorarrayFdcSweepCapPfFromFreq(selected.frequencyHz, &capPf);
            printf("FDC_BOOT_FULL_SWEEP_RESULT,s=%u,d=%u,dev=%s,ch=%u,result=%s,bestFreqHz=%.3f,bestDrive=0x%04X,bestDeglitch=0x%X,capPf=%.6f,capValid=%u\n",
                   (unsigned)cal->sIndex,
                   (unsigned)cal->dIndex,
                   sensorarrayFdcSweepDevName(devId),
                   (unsigned)channel,
                   haveBest ? "ok" : "fallback",
                   selected.frequencyHz,
                   selected.driveCurrentNorm,
                   (unsigned)selected.deglitchReq,
                   capValid ? capPf : 0.0,
                   capValid ? 1u : 0u);
        } else if (mode == SENSORARRAY_FDC_CELL_SWEEP_FAST) {
            printf("FDC_FAST_SWEEP_RESULT,s=%u,d=%u,result=%s,bestFreqHz=%.3f,action=update_last_good\n",
                   (unsigned)cal->sIndex,
                   (unsigned)cal->dIndex,
                   haveBest ? "ok" : "fallback",
                   selected.frequencyHz);
        } else {
            printf("FDC_FULL_SWEEP_RESCUE_RESULT,s=%u,d=%u,result=%s,bestFreqHz=%.3f,action=update_last_good\n",
                   (unsigned)cal->sIndex,
                   (unsigned)cal->dIndex,
                   haveBest ? "ok" : "fallback",
                   selected.frequencyHz);
        }
        return ESP_OK;
    }

    const sensorarrayFdcSweepCandidateResult_t *diag = haveDiagnostic ? &diagnostic : NULL;
    if (diag && mode == SENSORARRAY_FDC_CELL_SWEEP_BOOT_FULL) {
        sensorarrayFdcSweepUpdateCalibrationFromCandidate(cal, diag, true, false);
    }

    if (mode == SENSORARRAY_FDC_CELL_SWEEP_BOOT_FULL) {
        sensorarrayFdcSweepIncrementFailCount16(&cal->consecutiveFailCount);
        cal->lastFailReason = diag ? ((diag->zeroRawCount != 0u) ? "zero_raw_no_oscillation" :
                                      (diag->timeoutCount != 0u) ? "timeout" :
                                      (diag->watchdogCount != 0u) ? "status_watchdog" :
                                      (sensorarrayFdcSweepCandidateHasRawFrequency(diag) ? "unstable_sample" : "boot_sweep_failed"))
                                  : "boot_sweep_failed";
        printf("FDC_BOOT_FULL_SWEEP_RESULT,s=%u,d=%u,dev=%s,ch=%u,result=%s,bestFreqHz=%.3f,bestDrive=0x%04X,bestDeglitch=0x%X,capPf=0.000000,capValid=0\n",
               (unsigned)cal->sIndex,
               (unsigned)cal->dIndex,
               sensorarrayFdcSweepDevName(devId),
               (unsigned)channel,
               "fail",
               diag ? diag->frequencyHz : 0.0,
               diag ? diag->driveCurrentNorm : 0u,
               diag ? (unsigned)diag->deglitchReq : 0u);
    } else if (mode == SENSORARRAY_FDC_CELL_SWEEP_FAST) {
        sensorarrayFdcSweepIncrementFailCount(&cal->fastSweepFailCount);
        sensorarrayFdcSweepIncrementFailCount16(&cal->consecutiveFailCount);
        cal->lastFailReason = diag ? ((diag->zeroRawCount != 0u) ? "zero_raw_no_oscillation" :
                                      (diag->timeoutCount != 0u) ? "timeout" :
                                      (diag->watchdogCount != 0u) ? "status_watchdog" :
                                      (sensorarrayFdcSweepCandidateHasRawFrequency(diag) ? "unstable_sample" : "fast_sweep_failed"))
                                  : "fast_sweep_failed";
        printf("FDC_FAST_SWEEP_RESULT,s=%u,d=%u,result=%s,bestFreqHz=%.3f,action=no_cap_result\n",
               (unsigned)cal->sIndex,
               (unsigned)cal->dIndex,
               "fail",
               diag ? diag->frequencyHz : 0.0);
    } else {
        sensorarrayFdcSweepIncrementFailCount(&cal->fullSweepFailCount);
        sensorarrayFdcSweepIncrementFailCount16(&cal->consecutiveFailCount);
        cal->lastFailReason = diag ? ((diag->zeroRawCount != 0u) ? "zero_raw_no_oscillation" :
                                      (diag->timeoutCount != 0u) ? "timeout" :
                                      (diag->watchdogCount != 0u) ? "status_watchdog" :
                                      (sensorarrayFdcSweepCandidateHasRawFrequency(diag) ? "unstable_sample" : "full_sweep_failed"))
                                  : "full_sweep_failed";
        printf("FDC_FULL_SWEEP_RESCUE_RESULT,s=%u,d=%u,result=%s,bestFreqHz=%.3f,action=no_cap_result\n",
               (unsigned)cal->sIndex,
               (unsigned)cal->dIndex,
               "no_lock",
               diag ? diag->frequencyHz : 0.0);
    }

    if (outBest && diag) {
        *outBest = *diag;
    }
    return (lastErr != ESP_OK) ? lastErr : ESP_ERR_INVALID_RESPONSE;
}

static esp_err_t sensorarrayFdcSweepRunDirectReadForCell(sensorarrayState_t *state,
                                                         sensorarrayFdcCellCalibration_t *cal,
                                                         sensorarrayFdcSweepCandidateResult_t *outCandidate,
                                                         bool *outAccepted)
{
    if (outCandidate) {
        *outCandidate = (sensorarrayFdcSweepCandidateResult_t){0};
    }
    if (outAccepted) {
        *outAccepted = false;
    }
    if (!state || !cal) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!cal->hasLastGood) {
        return ESP_ERR_INVALID_STATE;
    }

    sensorarrayFdcDeviceId_t devId = (sensorarrayFdcDeviceId_t)cal->fdcDevice;
    Fdc2214CapChannel_t channel = (Fdc2214CapChannel_t)cal->fdcChannel;
    esp_err_t err = sensorarrayFdcSweepPrepareCellPath(state, cal, "direct_read");
    if (err != ESP_OK) {
        return err;
    }

    sensorarrayFdcSweepCandidateResult_t candidate = {0};
    err = sensorarrayFdcSweepCaptureCandidate(state,
                                              devId,
                                              channel,
                                              sensorarrayFdcSweepFindDeglitchCandidate((uint8_t)cal->lastGoodDeglitch),
                                              cal->lastGoodDriveCurrent,
                                              cal->lastGoodHighCurrent,
                                              cal->sIndex,
                                              cal->dIndex,
                                              "direct_read",
                                              0u,
                                              1u,
                                              true,
                                              (uint32_t)CONFIG_SENSORARRAY_FDC_DIRECT_QUALITY_SAMPLES,
                                              &candidate);
    if (outCandidate) {
        *outCandidate = candidate;
    }

    bool accepted = sensorarrayFdcSweepCandidateIsWorking(&candidate) ||
                    sensorarrayFdcSweepCandidateIsFallbackUsable(&candidate);
    printf("FDC_DIRECT_READ,s=%u,d=%u,dev=%s,ch=%u,source=last_good,drive=0x%04X,deglitch=0x%X,freqHz=%.3f,stable=%u\n",
           (unsigned)cal->sIndex,
           (unsigned)cal->dIndex,
           sensorarrayFdcSweepDevName(devId),
           (unsigned)channel,
           cal->lastGoodDriveCurrent,
           (unsigned)cal->lastGoodDeglitch,
           candidate.frequencyHz,
           candidate.stable ? 1u : 0u);

    if (accepted) {
        sensorarrayFdcSweepUpdateCalibrationFromCandidate(cal, &candidate, false, true);
        sensorarrayFdcSweepUpdateStateCacheFromCandidate(state,
                                                         cal,
                                                         &candidate,
                                                         SENSORARRAY_FDC_CACHE_SOURCE_LAST_GOOD);
        sensorarrayFdcSweepLogCapResult(cal, &candidate);
        if (outAccepted) {
            *outAccepted = true;
        }
        return ESP_OK;
    }

    sensorarrayFdcSweepIncrementFailCount(&cal->directFailCount);
    sensorarrayFdcSweepIncrementFailCount16(&cal->consecutiveFailCount);
    if (!candidate.raw28Mean && !candidate.raw28Last) {
        sensorarrayFdcSweepIncrementFailCount16(&cal->consecutiveZeroRawCount);
    }
    if (candidate.watchdogCount != 0u) {
        sensorarrayFdcSweepIncrementFailCount16(&cal->consecutiveStatusFaultCount);
    }
    if (candidate.invalidSampleCount != 0u && candidate.validSampleCount == 0u) {
        sensorarrayFdcSweepIncrementFailCount16(&cal->consecutiveNoUnreadCount);
    }
    cal->lastFailReason = (candidate.zeroRawCount != 0u) ? "zero_raw_no_oscillation" :
                          (candidate.timeoutCount != 0u) ? "timeout" :
                          (candidate.watchdogCount != 0u) ? "status_watchdog" :
                          (candidate.raw28Mean != 0u || candidate.raw28Last != 0u) ? "unstable_sample" :
                          "direct_read_failed";
    return (err != ESP_OK) ? err : ESP_ERR_INVALID_RESPONSE;
}

esp_err_t sensorarrayFdcSweepMeasureCell(sensorarrayState_t *state,
                                         uint8_t sIndex,
                                         uint8_t dIndex,
                                         uint32_t *outRaw28,
                                         bool *outValid)
{
    if (outRaw28) {
        *outRaw28 = 0u;
    }
    if (outValid) {
        *outValid = false;
    }
    if (!state || !outRaw28 || !outValid || !sensorarrayMatrixIndexIsValid(sIndex, dIndex)) {
        return ESP_ERR_INVALID_ARG;
    }

    printf("FDC_RUNTIME_WARN,reason=unexpected_cell_sweep,s=%u,d=%u\n",
           (unsigned)sIndex,
           (unsigned)dIndex);

    sensorarrayFdcSweepEnsureCalibrationTable();
    sensorarrayFdcCellCalibration_t *cal = sensorarrayFdcSweepCell(sIndex, dIndex);
    if (!cal) {
        return ESP_ERR_INVALID_ARG;
    }

    if (sensorarrayFdcSweepConsumeForceFullSweep(sIndex, dIndex)) {
        sensorarrayFdcSweepCandidateResult_t forceCandidate = {0};
        bool forceAccepted = false;
        esp_err_t forceErr = sensorarrayFdcSweepRunCellSweep(state,
                                                             cal,
                                                             SENSORARRAY_FDC_CELL_SWEEP_FULL_RESCUE,
                                                             "force_full_sweep",
                                                             &forceCandidate,
                                                             &forceAccepted);
        if (forceAccepted) {
            sensorarrayFdcSweepLogCapResult(cal, &forceCandidate);
            *outRaw28 = forceCandidate.raw28Last ? forceCandidate.raw28Last : forceCandidate.raw28Mean;
            *outValid = true;
            return ESP_OK;
        }
        return forceErr;
    }

    sensorarrayFdcSweepCandidateResult_t directCandidate = {0};
    bool directAccepted = false;
    bool hugeJump = false;
    esp_err_t firstErr = ESP_OK;

    if (cal->hasLastGood) {
        esp_err_t directErr = sensorarrayFdcSweepRunDirectReadForCell(state,
                                                                      cal,
                                                                      &directCandidate,
                                                                      &directAccepted);
        if (directAccepted) {
            *outRaw28 = directCandidate.raw28Last ? directCandidate.raw28Last : directCandidate.raw28Mean;
            *outValid = true;
            return ESP_OK;
        }
        if (directErr != ESP_OK && firstErr == ESP_OK) {
            firstErr = directErr;
        }
        hugeJump = sensorarrayFdcSweepCandidateHugeJump(cal, &directCandidate);
    }

    uint32_t nowMs = sensorarrayFdcSweepNowMs();
    uint16_t fastThreshold = (uint16_t)CONFIG_SENSORARRAY_FDC_CELL_FAST_SWEEP_FAIL_THRESHOLD;
    if (fastThreshold == 0u) {
        fastThreshold = 1u;
    }
    bool fastDue = !cal->hasLastGood ||
                   cal->consecutiveFailCount >= fastThreshold ||
                   cal->directFailCount >= (uint8_t)CONFIG_SENSORARRAY_FDC_DIRECT_FAIL_THRESHOLD;
    if (fastDue &&
        sensorarrayFdcSweepCooldownElapsed(nowMs,
                                           cal->lastSweepMs,
                                           (uint32_t)CONFIG_SENSORARRAY_FDC_FAST_SWEEP_COOLDOWN_MS)) {
        sensorarrayFdcSweepCandidateResult_t fastCandidate = {0};
        bool fastAccepted = false;
        esp_err_t fastErr = sensorarrayFdcSweepRunCellSweep(state,
                                                            cal,
                                                            SENSORARRAY_FDC_CELL_SWEEP_FAST,
                                                            cal->hasLastGood ? "direct_read_failed" : "fast_no_last_good",
                                                            &fastCandidate,
                                                            &fastAccepted);
        if (fastAccepted) {
            sensorarrayFdcSweepLogCapResult(cal, &fastCandidate);
            *outRaw28 = fastCandidate.raw28Last ? fastCandidate.raw28Last : fastCandidate.raw28Mean;
            *outValid = true;
            return ESP_OK;
        }
        if (fastErr != ESP_OK && firstErr == ESP_OK) {
            firstErr = fastErr;
        }
    } else if (!fastDue) {
        cal->lastFailReason = hugeJump ? "frequency_jump" : "direct_read_failed";
    } else {
        cal->lastFailReason = "fast_sweep_cooldown";
    }

    printf("FDC_NO_VALID_CAP_RESULT,s=%u,d=%u,dev=%s,ch=%u,directFail=%u,fastFail=%u,fullFail=%u,hasBootSweep=%u,hasLastGood=%u\n",
           (unsigned)cal->sIndex,
           (unsigned)cal->dIndex,
           sensorarrayFdcSweepDevName((sensorarrayFdcDeviceId_t)cal->fdcDevice),
           (unsigned)cal->fdcChannel,
           (unsigned)cal->directFailCount,
           (unsigned)cal->fastSweepFailCount,
           (unsigned)cal->fullSweepFailCount,
           cal->hasBootSweep ? 1u : 0u,
           cal->hasLastGood ? 1u : 0u);
    return (firstErr != ESP_OK) ? firstErr : ESP_ERR_INVALID_RESPONSE;
}

static uint8_t sensorarrayFdcSweepUnreadMaskFromStatus(const Fdc2214CapStatus_t *status)
{
    if (!status) {
        return 0u;
    }

    uint8_t mask = 0u;
    for (uint8_t ch = 0u; ch < SENSORARRAY_FDC_SWEEP_DEVICE_CHANNELS; ++ch) {
        if (status->UnreadConversion[ch]) {
            mask |= (uint8_t)(1u << ch);
        }
    }
    return mask;
}

#if CONFIG_SENSORARRAY_FDC_VERBOSE_SCAN_LOG
static const char *sensorarrayFdcSweepRrName(uint8_t rrSequence)
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

static sensorarrayFdcDeviceState_t *sensorarrayFdcSweepDeviceState(sensorarrayState_t *state,
                                                                   sensorarrayFdcDeviceId_t devId)
{
    return sensorarrayMeasureGetFdcState(state, devId);
}

esp_err_t sensorarrayFdcSweepRouteRowForCap(sensorarrayState_t *state,
                                            uint8_t sIndex,
                                            const char *reason)
{
    if (!state || !sensorarrayMatrixIndexIsValid(sIndex, 1u)) {
        return ESP_ERR_INVALID_ARG;
    }

    const char *source = reason ? reason : SENSORARRAY_NA;
    esp_err_t err = sensorarrayMeasurePrepareFdcMatrixPath(state, source);
    if (err == ESP_OK) {
        err = tmuxSwitchSelectRow((uint8_t)(sIndex - 1u));
    }
    if (err == ESP_OK) {
        sensorarrayFdcSweepDelayMs(((uint32_t)CONFIG_SENSORARRAY_FDC_MATRIX_SETTLE_US + 999u) / 1000u);
    }

    printf("FDC_ROW_ROUTE,s=%u,stage=done,reason=%s,err=0x%lx\n",
           (unsigned)sIndex,
           source,
           (unsigned long)err);
    return err;
}

static esp_err_t sensorarrayFdcSweepApplyCandidateToDevice(sensorarrayState_t *state,
                                                           sensorarrayFdcDeviceId_t devId,
                                                           const sensorarrayFdcDeglitchCandidate_t *deglitch,
                                                           const uint16_t driveCurrent[4],
                                                           bool highCurrent,
                                                           const char *reason)
{
    sensorarrayFdcDeviceState_t *fdcState = sensorarrayFdcSweepDeviceState(state, devId);
    if (!fdcState || !fdcState->ready || !fdcState->handle || !deglitch || !driveCurrent) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t firstErr = ESP_OK;
    for (uint8_t ch = 0u; ch < SENSORARRAY_FDC_SWEEP_DEVICE_CHANNELS; ++ch) {
        esp_err_t err = Fdc2214CapWriteDriveCurrent(fdcState->handle,
                                                    (Fdc2214CapChannel_t)ch,
                                                    sensorarrayFdcSweepClampDrive(driveCurrent[ch]));
        if (err != ESP_OK && firstErr == ESP_OK) {
            firstErr = err;
        }
    }
    if (firstErr != ESP_OK) {
        return firstErr;
    }

    Fdc2214CapDeglitch_t deglitchCode = sensorarrayFdcSweepDeglitchEnum(deglitch->deglitchCode);
    esp_err_t err = Fdc2214CapSetMuxConfig(fdcState->handle,
                                           true,
                                           SENSORARRAY_FDC_SWEEP_AUTOSCAN_RR_SEQUENCE,
                                           deglitchCode);
    if (err != ESP_OK) {
        return err;
    }

    Fdc2214CapConfigOptions_t config = {
        .ActiveChannel = FDC2214_CH0,
        .SleepModeEnabled = false,
        .SensorActivateSelLowPower = false,
        .RefClockSource = fdcState->refClockKnown ? fdcState->refClockSource :
#if SENSORARRAY_FDC_REF_CLOCK_USE_EXTERNAL
                                                   FDC2214_REF_CLOCK_EXTERNAL,
#else
                                                   FDC2214_REF_CLOCK_INTERNAL,
#endif
        .IntbDisabled = true,
        .HighCurrentDrive = highCurrent,
    };
    uint16_t configReq = Fdc2214CapBuildConfig(&config);
    err = Fdc2214CapExitSleep(fdcState->handle, configReq);
    if (err != ESP_OK) {
        return err;
    }

    err = Fdc2214CapClearStatus(fdcState->handle);
    if (err != ESP_OK) {
        return err;
    }

    Fdc2214CapCoreRegs_t regs = {0};
    err = Fdc2214CapReadCoreRegs(fdcState->handle, &regs);
    if (err != ESP_OK) {
        return err;
    }

    fdcState->statusConfigReg = regs.StatusConfig;
    fdcState->configReg = regs.Config;
    fdcState->muxConfigReg = regs.MuxConfig;

    uint8_t rr = (uint8_t)((regs.MuxConfig & 0x6000u) >> 13u);
    uint8_t muxDeglitch = (uint8_t)(regs.MuxConfig & SENSORARRAY_FDC_SWEEP_MUX_CONFIG_DEGLITCH_MASK);
    bool autoscan = (regs.MuxConfig & SENSORARRAY_FDC_SWEEP_MUX_CONFIG_AUTOSCAN_MASK) != 0u;
    bool sleep = (regs.Config & SENSORARRAY_FDC_SWEEP_CONFIG_SLEEP_MODE_EN_MASK) != 0u;
    bool highCurrentReadback = (regs.Config & SENSORARRAY_FDC_SWEEP_CONFIG_HIGH_CURRENT_DRV_MASK) != 0u;

#if CONFIG_SENSORARRAY_FDC_VERBOSE_SCAN_LOG
    printf("FDC_AUTOSCAN_CONFIG,device=addr0x%02X,mux=0x%04X,config=0x%04X,autoscan=%u,rr=%u,rrName=%s,deglitch=0x%X,deglitchName=%s,highCurrent=%u,reason=%s\n",
           fdcState->i2cAddr,
           regs.MuxConfig,
           regs.Config,
           autoscan ? 1u : 0u,
           (unsigned)rr,
           sensorarrayFdcSweepRrName(rr),
           (unsigned)muxDeglitch,
           deglitch->deglitchName ? deglitch->deglitchName : SENSORARRAY_NA,
           highCurrentReadback ? 1u : 0u,
           reason ? reason : SENSORARRAY_NA);
#endif

    if (!autoscan ||
        rr != SENSORARRAY_FDC_SWEEP_AUTOSCAN_RR_SEQUENCE ||
        muxDeglitch != (uint8_t)deglitchCode ||
        sleep ||
        highCurrentReadback != highCurrent) {
        return ESP_ERR_INVALID_RESPONSE;
    }
    return ESP_OK;
}

static esp_err_t sensorarrayFdcSweepStartOrRestoreAutoscan(sensorarrayState_t *state,
                                                           sensorarrayFdcDeviceId_t devId,
                                                           const char *reason)
{
    return sensorarrayFdcSweepRestoreAutoscan(state, devId, reason);
}

static esp_err_t sensorarrayFdcSweepWaitDeviceAutoscan(sensorarrayFdcDeviceState_t *fdcState,
                                                       sensorarrayFdcDeviceId_t devId,
                                                       uint8_t sIndex,
                                                       uint32_t timeoutMs,
                                                       uint16_t *outStatus)
{
    if (!fdcState || !fdcState->handle) {
        return ESP_ERR_INVALID_STATE;
    }
    if (timeoutMs == 0u) {
        timeoutMs = 1u;
    }

    int64_t deadlineUs = esp_timer_get_time() + ((int64_t)timeoutMs * 1000LL);
    Fdc2214CapStatus_t status = {0};
    esp_err_t lastErr = ESP_ERR_TIMEOUT;
    while (esp_timer_get_time() <= deadlineUs) {
        esp_err_t err = Fdc2214CapReadStatus(fdcState->handle, &status);
        if (err == ESP_OK) {
            uint8_t unreadMask = sensorarrayFdcSweepUnreadMaskFromStatus(&status);
            if (status.DataReady || unreadMask == SENSORARRAY_FDC_SWEEP_AUTOSCAN_READY_MASK) {
                if (outStatus) {
                    *outStatus = status.Raw;
                }
                return ESP_OK;
            }
        }
        lastErr = err;
        sensorarrayFdcSweepDelayMs(1u);
    }

    if (outStatus) {
        *outStatus = status.Raw;
    }
    printf("FDC_FRAME_READY,row=%u,device=%s,status=0x%04X,unreadMask=0x%X,drdy=%u,err=0x%lx\n",
           (unsigned)sIndex,
           sensorarrayFdcSweepDevName(devId),
           status.Raw,
           (unsigned)sensorarrayFdcSweepUnreadMaskFromStatus(&status),
           status.DataReady ? 1u : 0u,
           (unsigned long)((lastErr == ESP_OK) ? ESP_ERR_TIMEOUT : lastErr));
    return (lastErr == ESP_OK) ? ESP_ERR_TIMEOUT : lastErr;
}

static esp_err_t sensorarrayFdcSweepReadDeviceChannels(sensorarrayFdcDeviceState_t *fdcState,
                                                       sensorarrayFdcDeviceId_t devId,
                                                       uint8_t sIndex,
                                                       sensorarrayFdcSweepDeviceRead4_t *outRead)
{
    if (!fdcState || !fdcState->handle || !outRead) {
        return ESP_ERR_INVALID_ARG;
    }

    *outRead = (sensorarrayFdcSweepDeviceRead4_t){
        .effectiveFclkHz = (fdcState->refClockKnown && fdcState->refClockHz != 0u) ?
                           fdcState->refClockHz :
                           sensorarrayMeasureFdcEffectiveFclkHz(),
        .err = ESP_OK,
    };

    Fdc2214CapFastChannelSample_t samples[SENSORARRAY_FDC_SWEEP_DEVICE_CHANNELS] = {0};
    esp_err_t firstErr = Fdc2214CapReadAutoscan4RawFast(fdcState->handle, samples);

    for (uint8_t ch = 0u; ch < SENSORARRAY_FDC_SWEEP_DEVICE_CHANNELS; ++ch) {
        const Fdc2214CapFastChannelSample_t *sample = &samples[ch];
        outRead->raw28[ch] = sample->raw28;
        outRead->statusRaw = sample->statusRaw;

        esp_err_t regErr = Fdc2214CapReadRawRegisters(fdcState->handle,
                                                      sensorarrayFdcSweepRegForChannel(SENSORARRAY_FDC_SWEEP_REG_RCOUNT_BASE,
                                                                                       (Fdc2214CapChannel_t)ch),
                                                      &outRead->rCount[ch]);
        if (regErr == ESP_OK) {
            regErr = Fdc2214CapReadRawRegisters(fdcState->handle,
                                                sensorarrayFdcSweepRegForChannel(SENSORARRAY_FDC_SWEEP_REG_SETTLECOUNT_BASE,
                                                                                 (Fdc2214CapChannel_t)ch),
                                                &outRead->settleCount[ch]);
        }
        if (regErr == ESP_OK) {
            regErr = Fdc2214CapReadClockDividers(fdcState->handle,
                                                 (Fdc2214CapChannel_t)ch,
                                                 &outRead->clockDividers[ch]);
        }
        if (regErr == ESP_OK) {
            regErr = Fdc2214CapReadDriveCurrent(fdcState->handle,
                                                (Fdc2214CapChannel_t)ch,
                                                &outRead->driveCurrent[ch]);
        }
        if (regErr != ESP_OK && firstErr == ESP_OK) {
            firstErr = regErr;
        }

        bool i2cError = (sample->errorMask & FDC2214CAP_FAST_ERROR_I2C) != 0u || regErr != ESP_OK;
        bool watchdog = (sample->errorMask & FDC2214CAP_FAST_ERROR_WATCHDOG) != 0u;
        bool amplitude = (sample->errorMask & FDC2214CAP_FAST_ERROR_AMPLITUDE) != 0u ||
                         sample->errAmplitude;
        bool noUnread = (sample->errorMask & FDC2214CAP_FAST_ERROR_NO_UNREAD) != 0u;
        bool rawZero = sample->raw28 == 0u;
        bool saturated = sample->raw28 >= SENSORARRAY_FDC_SWEEP_RAW28_SATURATED_THRESHOLD;
        bool readable = sample->unreadConversion || sample->dataReady || !noUnread;

        if (!i2cError && sample->raw28 != 0u && outRead->clockDividers[ch] != 0u) {
            outRead->freqHz[ch] =
                sensorarrayMeasureFdcRaw28ToFreqHz(sample->raw28,
                                                   outRead->effectiveFclkHz,
                                                   outRead->clockDividers[ch]);
        }

        bool freqOk = outRead->freqHz[ch] > 0.0;
        bool valid = !i2cError &&
                     readable &&
                     !watchdog &&
                     !rawZero &&
                     !saturated &&
                     freqOk;
        if (valid) {
            outRead->validMask |= (uint8_t)(1u << ch);
        }
        if (amplitude || noUnread) {
            outRead->warnMask |= (uint8_t)(1u << ch);
        }
        if (i2cError || watchdog || rawZero || saturated || !freqOk) {
            outRead->errorMask |= (uint8_t)(1u << ch);
        }
    }

    outRead->err = firstErr;
    printf("FDC_DEVICE_READ4,dev=%s,s=%u,raw28=[%lu,%lu,%lu,%lu],freqHz=[%.3f,%.3f,%.3f,%.3f],validMask=0x%X,warnMask=0x%X,errorMask=0x%X,status=0x%04X,err=0x%lx\n",
           sensorarrayFdcSweepDevName(devId),
           (unsigned)sIndex,
           (unsigned long)outRead->raw28[0],
           (unsigned long)outRead->raw28[1],
           (unsigned long)outRead->raw28[2],
           (unsigned long)outRead->raw28[3],
           outRead->freqHz[0],
           outRead->freqHz[1],
           outRead->freqHz[2],
           outRead->freqHz[3],
           (unsigned)outRead->validMask,
           (unsigned)outRead->warnMask,
           (unsigned)outRead->errorMask,
           outRead->statusRaw,
           (unsigned long)firstErr);
    return firstErr;
}

static void sensorarrayFdcSweepBuildCandidateFromRead(const sensorarrayFdcSweepDeviceRead4_t *read,
                                                      sensorarrayFdcDeviceId_t devId,
                                                      uint8_t channel,
                                                      const sensorarrayFdcDeglitchCandidate_t *deglitch,
                                                      bool highCurrent,
                                                      sensorarrayFdcSweepCandidateResult_t *outCandidate)
{
    if (!read || !deglitch || !outCandidate || channel >= SENSORARRAY_FDC_SWEEP_DEVICE_CHANNELS) {
        return;
    }

    uint8_t bit = (uint8_t)(1u << channel);
    bool valid = (read->validMask & bit) != 0u;
    bool warning = (read->warnMask & bit) != 0u;
    uint32_t raw28 = read->raw28[channel];
    double freqHz = read->freqHz[channel];

    *outCandidate = (sensorarrayFdcSweepCandidateResult_t){
        .valid = true,
        .deglitchReq = deglitch->deglitchCode,
        .deglitchName = deglitch->deglitchName,
        .deglitchBandwidthHz = deglitch->deglitchBandwidthHz,
        .highCurrentReq = highCurrent,
        .highCurrentReadback = highCurrent,
        .driveCurrentReq = read->driveCurrent[channel],
        .driveCurrentNorm = sensorarrayFdcSweepClampDrive(read->driveCurrent[channel]),
        .driveCurrentReadback = sensorarrayFdcSweepClampDrive(read->driveCurrent[channel]),
        .statusReg = read->statusRaw,
        .rCountReg = read->rCount[channel],
        .settleCountReg = read->settleCount[channel],
        .clockDividersReg = read->clockDividers[channel],
        .raw28Min = raw28,
        .raw28Max = raw28,
        .raw28Mean = raw28,
        .raw28Last = raw28,
        .validSampleCount = valid ? 1u : 0u,
        .invalidSampleCount = valid ? 0u : 1u,
        .saturatedCount = raw28 >= SENSORARRAY_FDC_SWEEP_RAW28_SATURATED_THRESHOLD ? 1u : 0u,
        .amplitudeFaultCount = warning ? 1u : 0u,
        .zeroRawCount = raw28 == 0u ? 1u : 0u,
        .timeoutCount = read->err == ESP_ERR_TIMEOUT ? 1u : 0u,
        .medianFreqHz = freqHz,
        .freqSpreadHz = 0.0,
        .relativeSpread = (freqHz > 0.0) ? 0.0 : 1.0,
        .frequencyHz = freqHz,
        .stable = valid,
        .saturated = raw28 >= SENSORARRAY_FDC_SWEEP_RAW28_SATURATED_THRESHOLD,
    };

    const char *clockStatus = NULL;
    bool clockOk = sensorarrayMeasureFdcDecodeClockDividers(outCandidate->clockDividersReg,
                                                            &outCandidate->finSelCode,
                                                            &outCandidate->finFactor,
                                                            &outCandidate->frefDivider,
                                                            &clockStatus);
    outCandidate->effectiveFclkHz = read->effectiveFclkHz;
    outCandidate->effectiveFrefHz = (clockOk && outCandidate->frefDivider != 0u) ?
        ((double)outCandidate->effectiveFclkHz / (double)outCandidate->frefDivider) :
        0.0;
    if (freqHz > 0.0) {
        double marginHz = freqHz * SENSORARRAY_FDC_SWEEP_CACHE_FREQ_MARGIN_RATIO;
        if (marginHz < SENSORARRAY_FDC_SWEEP_CACHE_FREQ_MARGIN_MIN_HZ) {
            marginHz = SENSORARRAY_FDC_SWEEP_CACHE_FREQ_MARGIN_MIN_HZ;
        }
        outCandidate->frequencyMarginHz = marginHz;
        outCandidate->deglitchMarginRatio =
            deglitch->deglitchBandwidthHz ? ((double)deglitch->deglitchBandwidthHz / freqHz) : 0.0;
        outCandidate->deglitchBandwidthOk =
            deglitch->deglitchBandwidthHz == 0u ||
            freqHz < ((double)deglitch->deglitchBandwidthHz * 0.90);
    }
    (void)devId;
    (void)clockStatus;
}

static bool sensorarrayFdcSweepRowCandidateUsable(const sensorarrayFdcSweepCandidateResult_t *candidate)
{
    if (!candidate || !candidate->valid) {
        return false;
    }
    return candidate->raw28Mean != 0u &&
           candidate->frequencyHz > 0.0 &&
           candidate->invalidSampleCount == 0u &&
           candidate->timeoutCount == 0u &&
           candidate->watchdogCount == 0u &&
           candidate->zeroRawCount == 0u &&
           candidate->saturatedCount == 0u &&
           candidate->deglitchBandwidthOk;
}

static bool sensorarrayFdcSweepRowCandidateBetter(const sensorarrayFdcSweepCandidateResult_t *candidate,
                                                  const sensorarrayFdcSweepCandidateResult_t *best)
{
    bool candidateUsable = sensorarrayFdcSweepRowCandidateUsable(candidate);
    bool bestUsable = sensorarrayFdcSweepRowCandidateUsable(best);
    if (candidateUsable != bestUsable) {
        return candidateUsable;
    }
    if (!candidateUsable) {
        return sensorarrayFdcSweepCandidateHasRawFrequency(candidate) &&
               !sensorarrayFdcSweepCandidateHasRawFrequency(best);
    }
    int32_t candidateScore = sensorarrayFdcSweepCandidateScore(candidate);
    int32_t bestScore = sensorarrayFdcSweepCandidateScore(best);
    if (candidateScore != bestScore) {
        return candidateScore > bestScore;
    }
    if (candidate->amplitudeFaultCount != best->amplitudeFaultCount) {
        return candidate->amplitudeFaultCount < best->amplitudeFaultCount;
    }
    if (candidate->deglitchBandwidthHz != best->deglitchBandwidthHz) {
        return candidate->deglitchBandwidthHz < best->deglitchBandwidthHz;
    }
    return candidate->driveCurrentNorm < best->driveCurrentNorm;
}

static void sensorarrayFdcSweepBuildRowDriveCandidate(uint8_t sIndex,
                                                      uint16_t drive,
                                                      sensorarrayFdcDeviceId_t devId,
                                                      uint16_t outDrive[4])
{
    for (uint8_t ch = 0u; ch < SENSORARRAY_FDC_SWEEP_DEVICE_CHANNELS; ++ch) {
        uint8_t dIndex = sensorarrayFdcSweepDLineForDeviceChannel(devId, (Fdc2214CapChannel_t)ch);
        const sensorarrayFdcCellCalibration_t *cal = sensorarrayFdcSweepCell(sIndex, dIndex);
        uint16_t selected = drive;
        if (selected == 0u && cal && cal->hasLastGood) {
            selected = cal->lastGoodDriveCurrent;
        } else if (selected == 0u && cal && cal->hasBootSweep) {
            selected = cal->bootDriveCurrent;
        }
        outDrive[ch] = sensorarrayFdcSweepClampDrive(selected);
    }
}

static esp_err_t sensorarrayFdcSweepRow(sensorarrayState_t *state,
                                        uint8_t sIndex,
                                        sensorarrayFdcCellSweepMode_t mode,
                                        const char *reason,
                                        uint32_t *outOkCount,
                                        uint32_t *outFailCount)
{
    if (outOkCount) {
        *outOkCount = 0u;
    }
    if (outFailCount) {
        *outFailCount = 0u;
    }
    if (!state || !sensorarrayMatrixIndexIsValid(sIndex, 1u)) {
        return ESP_ERR_INVALID_ARG;
    }

    sensorarrayFdcSweepEnsureCalibrationTable();
    const char *stage = sensorarrayFdcSweepModeStage(mode);
    const char *source = reason ? reason : stage;
    esp_err_t routeErr = sensorarrayFdcSweepRouteRowForCap(state, sIndex, source);
    if (routeErr != ESP_OK) {
        return routeErr;
    }

    const sensorarrayFdcDeglitchCandidate_t *deglitchOrder[4] = {0};
    uint16_t driveOrder[20] = {0};
    uint32_t deglitchCount = 0u;
    uint32_t driveCount = 0u;
    if (mode == SENSORARRAY_FDC_CELL_SWEEP_FAST) {
        sensorarrayFdcCellCalibration_t *firstCal = sensorarrayFdcSweepCell(sIndex, 1u);
        uint16_t centerDrive = 0u;
        uint8_t centerDeglitch = 0u;
        driveCount = sensorarrayFdcSweepBuildFastDriveOrderForCell(firstCal,
                                                                   driveOrder,
                                                                   (uint32_t)(sizeof(driveOrder) / sizeof(driveOrder[0])),
                                                                   &centerDrive);
        deglitchCount = sensorarrayFdcSweepBuildFastDeglitchOrderForCell(firstCal,
                                                                         deglitchOrder,
                                                                         (uint32_t)(sizeof(deglitchOrder) / sizeof(deglitchOrder[0])),
                                                                         &centerDeglitch);
        (void)centerDrive;
        (void)centerDeglitch;
    } else {
        sensorarrayFdcCellCalibration_t *firstCal = sensorarrayFdcSweepCell(sIndex, 1u);
        driveCount = sensorarrayFdcSweepBuildFullDriveOrderForCell(firstCal,
                                                                   driveOrder,
                                                                   (uint32_t)(sizeof(driveOrder) / sizeof(driveOrder[0])));
        deglitchCount = sensorarrayFdcSweepBuildFullDeglitchOrderForCell(firstCal,
                                                                         deglitchOrder,
                                                                         (uint32_t)(sizeof(deglitchOrder) / sizeof(deglitchOrder[0])));
    }

    if (deglitchCount == 0u || driveCount == 0u) {
        return ESP_ERR_INVALID_STATE;
    }

    uint32_t highCurrentPasses = (mode == SENSORARRAY_FDC_CELL_SWEEP_FAST) ? 1u : 2u;
    uint32_t candidateCount = deglitchCount * driveCount * highCurrentPasses;
    uint32_t candidateIndex = 0u;
    uint32_t startMs = sensorarrayFdcSweepNowMs();
    uint32_t totalTimeoutMs = sensorarrayFdcSweepTotalTimeoutMs();
    sensorarrayFdcSweepCandidateResult_t best[SENSORARRAY_FDC_SWEEP_ROW_CELLS] = {0};
    bool haveBest[SENSORARRAY_FDC_SWEEP_ROW_CELLS] = {0};
    esp_err_t firstErr = ESP_OK;

    printf("FDC_ROW_SWEEP_BEGIN,s=%u,mode=%s,reason=%s,candidates=%lu\n",
           (unsigned)sIndex,
           stage,
           source,
           (unsigned long)candidateCount);

    for (uint32_t hp = 0u; hp < highCurrentPasses; ++hp) {
        bool highCurrent = (hp != 0u);
        for (uint32_t d = 0u; d < deglitchCount; ++d) {
            const sensorarrayFdcDeglitchCandidate_t *deglitch = deglitchOrder[d];
            if (!deglitch) {
                continue;
            }
            for (uint32_t driveIdx = 0u; driveIdx < driveCount; ++driveIdx) {
                if ((uint32_t)(sensorarrayFdcSweepNowMs() - startMs) >= totalTimeoutMs) {
                    firstErr = ESP_ERR_TIMEOUT;
                    goto row_sweep_done;
                }

                uint16_t primaryDrive[4] = {0};
                uint16_t secondaryDrive[4] = {0};
                sensorarrayFdcSweepBuildRowDriveCandidate(sIndex,
                                                          driveOrder[driveIdx],
                                                          SENSORARRAY_FDC_DEV_PRIMARY,
                                                          primaryDrive);
                sensorarrayFdcSweepBuildRowDriveCandidate(sIndex,
                                                          driveOrder[driveIdx],
                                                          SENSORARRAY_FDC_DEV_SECONDARY,
                                                          secondaryDrive);

                esp_err_t primaryErr =
                    sensorarrayFdcSweepApplyCandidateToDevice(state,
                                                              SENSORARRAY_FDC_DEV_PRIMARY,
                                                              deglitch,
                                                              primaryDrive,
                                                              highCurrent,
                                                              stage);
                esp_err_t secondaryErr =
                    sensorarrayFdcSweepApplyCandidateToDevice(state,
                                                              SENSORARRAY_FDC_DEV_SECONDARY,
                                                              deglitch,
                                                              secondaryDrive,
                                                              highCurrent,
                                                              stage);
                if (primaryErr != ESP_OK && firstErr == ESP_OK) {
                    firstErr = primaryErr;
                }
                if (secondaryErr != ESP_OK && firstErr == ESP_OK) {
                    firstErr = secondaryErr;
                }

                sensorarrayFdcSweepDelayMs((uint32_t)CONFIG_SENSORARRAY_FDC_SWEEP_SETTLE_MS);
                sensorarrayFdcSweepDeviceRead4_t primaryRead = {0};
                sensorarrayFdcSweepDeviceRead4_t secondaryRead = {0};

                if (primaryErr == ESP_OK) {
                    uint16_t status = 0u;
                    esp_err_t waitErr =
                        sensorarrayFdcSweepWaitDeviceAutoscan(&state->fdcPrimary,
                                                              SENSORARRAY_FDC_DEV_PRIMARY,
                                                              sIndex,
                                                              (uint32_t)CONFIG_SENSORARRAY_FDC_SWEEP_SAMPLE_TIMEOUT_MS,
                                                              &status);
                    if (waitErr != ESP_OK && firstErr == ESP_OK) {
                        firstErr = waitErr;
                    }
                    (void)status;
                    esp_err_t readErr =
                        sensorarrayFdcSweepReadDeviceChannels(&state->fdcPrimary,
                                                              SENSORARRAY_FDC_DEV_PRIMARY,
                                                              sIndex,
                                                              &primaryRead);
                    if (readErr != ESP_OK && firstErr == ESP_OK) {
                        firstErr = readErr;
                    }
                } else {
                    primaryRead.err = primaryErr;
                    primaryRead.errorMask = 0x0Fu;
                }

                if (secondaryErr == ESP_OK) {
                    uint16_t status = 0u;
                    esp_err_t waitErr =
                        sensorarrayFdcSweepWaitDeviceAutoscan(&state->fdcSecondary,
                                                              SENSORARRAY_FDC_DEV_SECONDARY,
                                                              sIndex,
                                                              (uint32_t)CONFIG_SENSORARRAY_FDC_SWEEP_SAMPLE_TIMEOUT_MS,
                                                              &status);
                    if (waitErr != ESP_OK && firstErr == ESP_OK) {
                        firstErr = waitErr;
                    }
                    (void)status;
                    esp_err_t readErr =
                        sensorarrayFdcSweepReadDeviceChannels(&state->fdcSecondary,
                                                              SENSORARRAY_FDC_DEV_SECONDARY,
                                                              sIndex,
                                                              &secondaryRead);
                    if (readErr != ESP_OK && firstErr == ESP_OK) {
                        firstErr = readErr;
                    }
                } else {
                    secondaryRead.err = secondaryErr;
                    secondaryRead.errorMask = 0x0Fu;
                }

                printf("FDC_ROW_SWEEP_CANDIDATE,s=%u,dev=primary,deglitchName=%s,drive=0x%04X,highCurrent=%u,usableMask=0x%X,warnMask=0x%X,errorMask=0x%X,candidate=%lu/%lu\n",
                       (unsigned)sIndex,
                       deglitch->deglitchName ? deglitch->deglitchName : SENSORARRAY_NA,
                       primaryDrive[0],
                       highCurrent ? 1u : 0u,
                       (unsigned)primaryRead.validMask,
                       (unsigned)primaryRead.warnMask,
                       (unsigned)primaryRead.errorMask,
                       (unsigned long)(candidateIndex + 1u),
                       (unsigned long)candidateCount);
                printf("FDC_ROW_SWEEP_CANDIDATE,s=%u,dev=secondary,deglitchName=%s,drive=0x%04X,highCurrent=%u,usableMask=0x%X,warnMask=0x%X,errorMask=0x%X,candidate=%lu/%lu\n",
                       (unsigned)sIndex,
                       deglitch->deglitchName ? deglitch->deglitchName : SENSORARRAY_NA,
                       secondaryDrive[0],
                       highCurrent ? 1u : 0u,
                       (unsigned)secondaryRead.validMask,
                       (unsigned)secondaryRead.warnMask,
                       (unsigned)secondaryRead.errorMask,
                       (unsigned long)(candidateIndex + 1u),
                       (unsigned long)candidateCount);

                const sensorarrayFdcSweepDeviceRead4_t *reads[2] = {&primaryRead, &secondaryRead};
                for (uint8_t dev = 0u; dev < 2u; ++dev) {
                    for (uint8_t ch = 0u; ch < SENSORARRAY_FDC_SWEEP_DEVICE_CHANNELS; ++ch) {
                        uint8_t rowCell = (uint8_t)(ch + (dev * SENSORARRAY_FDC_SWEEP_DEVICE_CHANNELS));
                        sensorarrayFdcSweepCandidateResult_t candidate = {0};
                        sensorarrayFdcSweepBuildCandidateFromRead(reads[dev],
                                                                  (sensorarrayFdcDeviceId_t)dev,
                                                                  ch,
                                                                  deglitch,
                                                                  highCurrent,
                                                                  &candidate);
                        if (!haveBest[rowCell] ||
                            sensorarrayFdcSweepRowCandidateBetter(&candidate, &best[rowCell])) {
                            best[rowCell] = candidate;
                            haveBest[rowCell] = true;
                        }
                    }
                }

                candidateIndex++;
                if (mode == SENSORARRAY_FDC_CELL_SWEEP_FAST) {
                    bool allHaveUsable = true;
                    for (uint8_t i = 0u; i < SENSORARRAY_FDC_SWEEP_ROW_CELLS; ++i) {
                        if (!sensorarrayFdcSweepRowCandidateUsable(&best[i])) {
                            allHaveUsable = false;
                            break;
                        }
                    }
                    if (allHaveUsable) {
                        goto row_sweep_done;
                    }
                }
                taskYIELD();
            }
        }
    }

row_sweep_done:
    ;
    uint32_t okCount = 0u;
    uint32_t failCount = 0u;
    uint8_t validMask = 0u;
    uint8_t warnMask = 0u;
    uint8_t errorMask = 0u;
    double freqHz[SENSORARRAY_FDC_SWEEP_ROW_CELLS] = {0};
    uint16_t bestDrive[SENSORARRAY_FDC_SWEEP_ROW_CELLS] = {0};
    uint8_t bestDeglitch[SENSORARRAY_FDC_SWEEP_ROW_CELLS] = {0};

    for (uint8_t dIndex = 1u; dIndex <= SENSORARRAY_MATRIX_COLS; ++dIndex) {
        uint8_t rowCell = (uint8_t)(dIndex - 1u);
        sensorarrayFdcCellCalibration_t *cal = sensorarrayFdcSweepCell(sIndex, dIndex);
        sensorarrayFdcSweepCandidateResult_t *candidate = &best[rowCell];
        bool usable = haveBest[rowCell] && sensorarrayFdcSweepRowCandidateUsable(candidate);
        if (candidate->amplitudeFaultCount != 0u) {
            warnMask |= (uint8_t)(1u << rowCell);
        }
        if (usable) {
            okCount++;
            validMask |= (uint8_t)(1u << rowCell);
            freqHz[rowCell] = candidate->frequencyHz;
            bestDrive[rowCell] = candidate->driveCurrentNorm;
            bestDeglitch[rowCell] = candidate->deglitchReq;
            if (cal) {
                sensorarrayFdcSweepUpdateCalibrationFromCandidate(cal,
                                                                  candidate,
                                                                  mode == SENSORARRAY_FDC_CELL_SWEEP_BOOT_FULL,
                                                                  true);
                sensorarrayFdcSweepUpdateDeviceProfileFromCandidate(state,
                                                                    (sensorarrayFdcDeviceId_t)cal->fdcDevice,
                                                                    (Fdc2214CapChannel_t)cal->fdcChannel,
                                                                    candidate);
                sensorarrayFdcSweepUpdateStateCacheFromCandidate(state,
                                                                 cal,
                                                                 candidate,
                                                                 sensorarrayFdcSweepCacheSourceForMode(mode));
            }
        } else {
            failCount++;
            errorMask |= (uint8_t)(1u << rowCell);
            bestDrive[rowCell] = candidate->driveCurrentNorm;
            bestDeglitch[rowCell] = candidate->deglitchReq;
            if (cal) {
                sensorarrayFdcSweepIncrementFailCount16(&cal->consecutiveFailCount);
                if (mode == SENSORARRAY_FDC_CELL_SWEEP_FAST) {
                    sensorarrayFdcSweepIncrementFailCount(&cal->fastSweepFailCount);
                    cal->lastFailReason = "fast_row_sweep_failed";
                } else if (mode == SENSORARRAY_FDC_CELL_SWEEP_FULL_RESCUE) {
                    sensorarrayFdcSweepIncrementFailCount(&cal->fullSweepFailCount);
                    cal->lastFailReason = "full_row_sweep_failed";
                } else {
                    cal->lastFailReason = "boot_row_sweep_failed";
                    if (haveBest[rowCell]) {
                        sensorarrayFdcSweepUpdateCalibrationFromCandidate(cal, candidate, true, false);
                    }
                }
            }
        }
    }

    printf("FDC_ROW_SWEEP_RESULT,s=%u,freqHz=[%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f],bestDeglitch=[0x%X,0x%X,0x%X,0x%X,0x%X,0x%X,0x%X,0x%X],bestDrive=[0x%04X,0x%04X,0x%04X,0x%04X,0x%04X,0x%04X,0x%04X,0x%04X],validMask=0x%02X,warnMask=0x%02X,errorMask=0x%02X,mode=%s,reason=%s\n",
           (unsigned)sIndex,
           freqHz[0], freqHz[1], freqHz[2], freqHz[3],
           freqHz[4], freqHz[5], freqHz[6], freqHz[7],
           (unsigned)bestDeglitch[0], (unsigned)bestDeglitch[1],
           (unsigned)bestDeglitch[2], (unsigned)bestDeglitch[3],
           (unsigned)bestDeglitch[4], (unsigned)bestDeglitch[5],
           (unsigned)bestDeglitch[6], (unsigned)bestDeglitch[7],
           bestDrive[0], bestDrive[1], bestDrive[2], bestDrive[3],
           bestDrive[4], bestDrive[5], bestDrive[6], bestDrive[7],
           (unsigned)validMask,
           (unsigned)warnMask,
           (unsigned)errorMask,
           stage,
           source);

    if (outOkCount) {
        *outOkCount = okCount;
    }
    if (outFailCount) {
        *outFailCount = failCount;
    }

    esp_err_t restorePrimary = sensorarrayFdcSweepStartOrRestoreAutoscan(state,
                                                                         SENSORARRAY_FDC_DEV_PRIMARY,
                                                                         source);
    esp_err_t restoreSecondary = sensorarrayFdcSweepStartOrRestoreAutoscan(state,
                                                                           SENSORARRAY_FDC_DEV_SECONDARY,
                                                                           source);
    if (restorePrimary != ESP_OK && firstErr == ESP_OK) {
        firstErr = restorePrimary;
    }
    if (restoreSecondary != ESP_OK && firstErr == ESP_OK) {
        firstErr = restoreSecondary;
    }

    if (okCount != 0u) {
        return ESP_OK;
    }
    return (firstErr != ESP_OK) ? firstErr : ESP_ERR_INVALID_RESPONSE;
}

esp_err_t sensorarrayFdcSweepRunFastRescueRow(sensorarrayState_t *state,
                                              uint8_t sIndex,
                                              const char *reason)
{
    return sensorarrayFdcSweepRow(state,
                                  sIndex,
                                  SENSORARRAY_FDC_CELL_SWEEP_FAST,
                                  reason ? reason : "row_fast_rescue",
                                  NULL,
                                  NULL);
}

esp_err_t sensorarrayFdcSweepRunFullRescueRow(sensorarrayState_t *state,
                                              uint8_t sIndex,
                                              const char *reason)
{
    return sensorarrayFdcSweepRow(state,
                                  sIndex,
                                  SENSORARRAY_FDC_CELL_SWEEP_FULL_RESCUE,
                                  reason ? reason : "row_full_rescue",
                                  NULL,
                                  NULL);
}

esp_err_t sensorarrayFdcSweepForceFullSweepCell(sensorarrayState_t *state,
                                                uint8_t sIndex,
                                                uint8_t dIndex)
{
    return sensorarrayFdcSweepRunFullRescueCell(state, sIndex, dIndex, "force_full_sweep");
}

esp_err_t sensorarrayFdcSweepRunCellSweepTarget(sensorarrayState_t *state,
                                                const sensorarrayFdcCellTarget_t *target,
                                                const char *reason,
                                                bool allowFullSweep)
{
    if (!state || !target || !sensorarrayMatrixIndexIsValid(target->sColumn, target->dLine)) {
        return ESP_ERR_INVALID_ARG;
    }

    printf("FDC_RESCUE,stage=begin,scope=row,s=%u,triggerD=%u,index=%u,device=%s,ch=%u,mode=fast,reason=%s\n",
           (unsigned)target->sColumn,
           (unsigned)target->dLine,
           (unsigned)target->matrixIndex,
           sensorarrayFdcSweepDevName(target->devId),
           (unsigned)target->fdcChannel,
           reason ? reason : SENSORARRAY_NA);

    uint32_t rowOk = 0u;
    uint32_t rowFail = 0u;
    esp_err_t err = sensorarrayFdcSweepRow(state,
                                           target->sColumn,
                                           SENSORARRAY_FDC_CELL_SWEEP_FAST,
                                           reason ? reason : "row_fast_rescue",
                                           &rowOk,
                                           &rowFail);
    printf("FDC_RESCUE,stage=end,scope=row,s=%u,triggerD=%u,mode=fast,okCount=%lu,failCount=%lu,err=0x%lx\n",
           (unsigned)target->sColumn,
           (unsigned)target->dLine,
           (unsigned long)rowOk,
           (unsigned long)rowFail,
           (unsigned long)err);
    if (err == ESP_OK) {
        return ESP_OK;
    }
    if (!allowFullSweep) {
        return err;
    }

    printf("FDC_RESCUE,stage=begin,scope=row,s=%u,triggerD=%u,index=%u,device=%s,ch=%u,mode=full,reason=fast_failed\n",
           (unsigned)target->sColumn,
           (unsigned)target->dLine,
           (unsigned)target->matrixIndex,
           sensorarrayFdcSweepDevName(target->devId),
           (unsigned)target->fdcChannel);
    err = sensorarrayFdcSweepRow(state,
                                 target->sColumn,
                                 SENSORARRAY_FDC_CELL_SWEEP_FULL_RESCUE,
                                 reason ? reason : "row_full_rescue",
                                 &rowOk,
                                 &rowFail);
    printf("FDC_RESCUE,stage=end,scope=row,s=%u,triggerD=%u,mode=full,okCount=%lu,failCount=%lu,err=0x%lx\n",
           (unsigned)target->sColumn,
           (unsigned)target->dLine,
           (unsigned long)rowOk,
           (unsigned long)rowFail,
           (unsigned long)err);
    return err;
}

esp_err_t sensorarrayFdcSweepRunCellFastRescue(sensorarrayState_t *state,
                                               const sensorarrayFdcCellTarget_t *target,
                                               const char *reason)
{
    return sensorarrayFdcSweepRunCellSweepTarget(state, target, reason, false);
}

esp_err_t sensorarrayFdcSweepRunFullRescueCell(sensorarrayState_t *state,
                                               uint8_t sIndex,
                                               uint8_t dIndex,
                                               const char *reason)
{
    if (!state) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!sensorarrayMatrixIndexIsValid(sIndex, dIndex)) {
        printf("FDC_SWEEP,stage=force_full_reject,reason=bad_cell,s=%u,d=%u\n",
               (unsigned)sIndex,
               (unsigned)dIndex);
        return ESP_ERR_INVALID_ARG;
    }
    sensorarrayFdcCellTarget_t target = {0};
    if (!sensorarrayMeasureMakeFdcCellTarget(state, sIndex, dIndex, &target)) {
        return ESP_ERR_INVALID_ARG;
    }

    printf("FDC_SWEEP,stage=force_full_begin,scope=row,s=%u,triggerD=%u,device=%s,ch=%u,reason=%s\n",
           (unsigned)sIndex,
           (unsigned)dIndex,
           sensorarrayFdcSweepDevFullName(target.devId),
           (unsigned)target.fdcChannel,
           reason ? reason : SENSORARRAY_NA);

    uint32_t rowOk = 0u;
    uint32_t rowFail = 0u;
    esp_err_t err = sensorarrayFdcSweepRow(state,
                                           sIndex,
                                           SENSORARRAY_FDC_CELL_SWEEP_FULL_RESCUE,
                                           reason ? reason : "force_full_sweep",
                                           &rowOk,
                                           &rowFail);
    printf("FDC_SWEEP,stage=force_full_done,scope=row,s=%u,triggerD=%u,okCount=%lu,failCount=%lu,err=0x%lx\n",
           (unsigned)sIndex,
           (unsigned)dIndex,
           (unsigned long)rowOk,
           (unsigned long)rowFail,
           (unsigned long)err);
    return err;
}

esp_err_t sensorarrayFdcSweepRunFullRescueAll(sensorarrayState_t *state, const char *reason)
{
    if (!state) {
        return ESP_ERR_INVALID_ARG;
    }
    if (gFdcRescueInProgress) {
        printf("FDC_RESCUE,stage=skip,reason=rescue_already_in_progress,requestReason=%s\n",
               reason ? reason : SENSORARRAY_NA);
        return ESP_ERR_INVALID_STATE;
    }

    gFdcRescueInProgress = true;
    uint32_t epoch = ++gFdcSweepRequestEpoch;
    const char *source = reason ? reason : SENSORARRAY_NA;
    printf("FDC_FULL_MATRIX_RESCUE_START,reason=%s\n", source);
#if CONFIG_SENSORARRAY_FDC_VERBOSE_REG_DUMP
    (void)sensorarrayFdcSweepDumpAllDeviceRegs(state, "rescue_begin", source);
#endif

    esp_err_t firstErr = ESP_OK;
    uint32_t successCount = 0u;
    uint32_t failedCount = 0u;

    for (uint8_t s = 1u; s <= SENSORARRAY_MATRIX_ROWS; ++s) {
        uint32_t rowOk = 0u;
        uint32_t rowFail = 0u;
        esp_err_t err = sensorarrayFdcSweepRow(state,
                                               s,
                                               SENSORARRAY_FDC_CELL_SWEEP_FULL_RESCUE,
                                               source,
                                               &rowOk,
                                               &rowFail);
        successCount += rowOk;
        failedCount += rowFail;
        if (err != ESP_OK && firstErr == ESP_OK) {
            firstErr = err;
        }
        printf("FDC_FULL_MATRIX_RESCUE_ROW,s=%u,result=%s,okCount=%lu,failCount=%lu,err=0x%lx\n",
               (unsigned)s,
               (rowOk != 0u) ? "ok" : "fail",
               (unsigned long)rowOk,
               (unsigned long)rowFail,
               (unsigned long)err);
        taskYIELD();
    }

    if (successCount == 0u) {
#if CONFIG_SENSORARRAY_FDC_VERBOSE_REG_DUMP
        (void)sensorarrayFdcSweepDumpAllDeviceRegs(state, "rescue_failed", source);
#endif
        firstErr = ESP_ERR_INVALID_RESPONSE;
    }

    gFdcRescueInProgress = false;
    (void)epoch;
    printf("FDC_FULL_MATRIX_RESCUE_DONE,okCount=%lu,failCount=%lu,err=0x%lx\n",
           (unsigned long)successCount,
           (unsigned long)failedCount,
           (unsigned long)((successCount == 0u) ? firstErr : ESP_OK));
    return (successCount == 0u) ? ESP_ERR_INVALID_RESPONSE : ESP_OK;
}

esp_err_t sensorarrayFdcSweepRunBoot(sensorarrayState_t *state)
{
    if (!state) {
        return ESP_ERR_INVALID_ARG;
    }

    sensorarrayFdcSweepInitCalibrationTable();

    printf("FDC_BOOT,stage=begin\n");
    esp_err_t pathErr = sensorarrayMeasurePrepareFdcMatrixPath(state, "boot_sweep");
    printf("FDC_BOOT,stage=path_prepare,err=0x%lx\n", (unsigned long)pathErr);
    if (pathErr != ESP_OK) {
        return pathErr;
    }

#if CONFIG_SENSORARRAY_FDC_VERBOSE_REG_DUMP
    (void)sensorarrayFdcSweepDumpAllDeviceRegs(state, "boot_begin", "boot");
#endif
    (void)sensorarrayFdcSweepLogBootDeviceRegs(&state->fdcPrimary, "primary_fdc2214");
    (void)sensorarrayFdcSweepLogBootDeviceRegs(&state->fdcSecondary, "secondary_fdc2214");

    esp_err_t firstErr = ESP_OK;
    uint32_t okCount = 0u;
    uint32_t failCount = 0u;

    printf("FDC_BOOT_MATRIX_SWEEP_START,targetCount=%lu\n",
           (unsigned long)SENSORARRAY_MATRIX_CELL_COUNT);
    for (uint8_t s = 1u; s <= SENSORARRAY_MATRIX_ROWS; ++s) {
        uint32_t rowOk = 0u;
        uint32_t rowFail = 0u;
        esp_err_t err = sensorarrayFdcSweepRow(state,
                                               s,
                                               SENSORARRAY_FDC_CELL_SWEEP_BOOT_FULL,
                                               "boot_matrix",
                                               &rowOk,
                                               &rowFail);
        okCount += rowOk;
        failCount += rowFail;
        if (err != ESP_OK && err != ESP_ERR_INVALID_RESPONSE && firstErr == ESP_OK) {
            firstErr = err;
        }
        printf("FDC_BOOT_ROW_RESULT,s=%u,result=%s,okCount=%lu,failCount=%lu,err=0x%lx\n",
               (unsigned)s,
               (rowOk != 0u) ? "ok" : "degraded",
               (unsigned long)rowOk,
               (unsigned long)rowFail,
               (unsigned long)err);
        taskYIELD();
    }
    printf("FDC_BOOT_MATRIX_SWEEP_DONE,okCount=%lu,failCount=%lu\n",
           (unsigned long)okCount,
           (unsigned long)failCount);

    esp_err_t restorePrimary = sensorarrayFdcSweepRestoreAutoscan(state, SENSORARRAY_FDC_DEV_PRIMARY, "boot");
    esp_err_t restoreSecondary = sensorarrayFdcSweepRestoreAutoscan(state, SENSORARRAY_FDC_DEV_SECONDARY, "boot");
    if (restorePrimary != ESP_OK && firstErr == ESP_OK) {
        firstErr = restorePrimary;
    }
    if (restoreSecondary != ESP_OK && firstErr == ESP_OK) {
        firstErr = restoreSecondary;
    }

    if (okCount == 0u) {
#if CONFIG_SENSORARRAY_FDC_VERBOSE_REG_DUMP
        (void)sensorarrayFdcSweepDumpAllDeviceRegs(state, "boot_failed", "no_valid_oscillation");
#endif
        printf("FDC_BOOT,stage=warning,reason=no_valid_oscillation,action=enter_normal_loop_degraded\n");
        if (restorePrimary != ESP_OK && restoreSecondary != ESP_OK) {
            printf("FDC_BOOT,stage=failed,reason=both_fdc_autoscan_restore_failed\n");
            printf("FDC_SWEEP,stage=boot,event=done,err=0x%lx,result=failed\n",
                   (unsigned long)((firstErr != ESP_OK) ? firstErr : ESP_ERR_INVALID_RESPONSE));
            return (firstErr != ESP_OK) ? firstErr : ESP_ERR_INVALID_RESPONSE;
        }
        printf("FDC_SWEEP,stage=boot,event=done,err=0x0,result=degraded,validCount=0\n");
        return ESP_OK;
    }

    if (restorePrimary != ESP_OK || restoreSecondary != ESP_OK) {
        printf("FDC_BOOT,stage=warning,reason=autoscan_restore_failed,primaryErr=0x%lx,secondaryErr=0x%lx\n",
               (unsigned long)restorePrimary,
               (unsigned long)restoreSecondary);
    }

    printf("FDC_SWEEP,stage=boot,event=done,err=0x0,result=ok,validCount=%lu\n",
           (unsigned long)okCount);
    return ESP_OK;
}

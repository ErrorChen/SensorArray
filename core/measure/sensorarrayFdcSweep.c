#include "sensorarrayFdcSweep.h"

#include <limits.h>
#include <stdio.h>
#include <string.h>

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "sensorarrayConfig.h"
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
#define SENSORARRAY_FDC_SWEEP_REG_CONFIG 0x1Au
#define SENSORARRAY_FDC_SWEEP_REG_MUX_CONFIG 0x1Bu
#define SENSORARRAY_FDC_SWEEP_RAW28_MAX 0x0FFFFFFFu
#define SENSORARRAY_FDC_SWEEP_RAW28_SATURATED_THRESHOLD 0x0FFFFF00u
#define SENSORARRAY_FDC_SWEEP_FAST_PROBE_SAMPLES 2u
#define SENSORARRAY_FDC_SWEEP_CONFIRM_SAMPLES 4u
#define SENSORARRAY_FDC_SWEEP_MAX_SAMPLES_PER_CANDIDATE \
    (SENSORARRAY_FDC_SWEEP_FAST_PROBE_SAMPLES + SENSORARRAY_FDC_SWEEP_CONFIRM_SAMPLES)
#define SENSORARRAY_FDC_SWEEP_CACHE_FREQ_MARGIN_MIN_HZ 150000.0
#define SENSORARRAY_FDC_SWEEP_CACHE_FREQ_MARGIN_RATIO 0.08
#define SENSORARRAY_FDC_SWEEP_DEFAULT_DEGLITCH_REQ 0x5u
#define SENSORARRAY_FDC_SWEEP_DEFAULT_DEGLITCH_BW_HZ 10000000u
#define SENSORARRAY_FDC_SWEEP_DEFAULT_DRIVE_CURRENT_REQ 0x4000u

/*
 * The validated S5D5 debug table started at 0x7800. The matrix has been observed
 * at about 6 Vpp, so lower IDRIVE candidates are prepended while keeping the
 * S5D5 fast-probe/confirm/fallback selection strategy intact.
 */
static const uint16_t SENSORARRAY_FDC_DRIVE_CURRENT_SWEEP_TABLE[] = {
    0x4000u,
    0x5000u,
    0x6000u,
    0x7000u,
    0x7800u,
    0x8000u,
    0x8800u,
    0x9000u,
    0x9800u,
    0xA000u,
    0xA800u,
    0xB000u,
    0xB800u,
    0xC000u,
    0xD000u,
    0xE000u,
    0xF800u,
};

static const sensorarrayFdcDeglitchCandidate_t SENSORARRAY_FDC_DEGLITCH_SWEEP_TABLE[] = {
    {0x1u, 1000000u, "1MHz"},
    {0x4u, 3300000u, "3p3MHz"},
    {0x5u, 10000000u, "10MHz"},
    {0x7u, 33000000u, "33MHz"},
};

static const char *sensorarrayFdcSweepDevName(sensorarrayFdcDeviceId_t devId)
{
    return (devId == SENSORARRAY_FDC_DEV_SECONDARY) ? "secondary" : "primary";
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

static uint32_t sensorarrayFdcSweepBuildDriveOrder(const sensorarrayFdcSweepProfile_t *profile,
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

    sensorarrayFdcSweepAddUniqueDeglitch(order, &count, capacity, d10);
    if (predictedFreqHz > 0.0) {
        if (predictedFreqHz <= 3000000.0) {
            sensorarrayFdcSweepAddUniqueDeglitch(order, &count, capacity, d3);
        }
        if (predictedFreqHz >= 9000000.0 || count == 1u) {
            sensorarrayFdcSweepAddUniqueDeglitch(order, &count, capacity, d33);
        }
        if (predictedFreqHz < 900000.0) {
            sensorarrayFdcSweepAddUniqueDeglitch(order, &count, capacity, d1);
        }
    } else {
        sensorarrayFdcSweepAddUniqueDeglitch(order, &count, capacity, d3);
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
    if (clockOk && candidate->raw28Mean > 0u) {
        sensorarrayFdcFrequencyDiag_t freqDiag = {0};
        if (sensorarrayMeasureFdcComputeFrequencyDiag(candidate->raw28Mean,
                                                      candidate->clockDividersReg,
                                                      &freqDiag) &&
            freqDiag.valid) {
            candidate->frequencyHz = freqDiag.freqHzCorrected;
            double marginHz = candidate->frequencyHz * SENSORARRAY_FDC_SWEEP_CACHE_FREQ_MARGIN_RATIO;
            if (marginHz < SENSORARRAY_FDC_SWEEP_CACHE_FREQ_MARGIN_MIN_HZ) {
                marginHz = SENSORARRAY_FDC_SWEEP_CACHE_FREQ_MARGIN_MIN_HZ;
            }
            candidate->frequencyMarginHz = marginHz;
            if (candidate->deglitchBandwidthHz > 0u && candidate->frequencyHz > 0.0) {
                candidate->deglitchMarginRatio =
                    (double)candidate->deglitchBandwidthHz / candidate->frequencyHz;
                candidate->deglitchBandwidthOk =
                    candidate->frequencyHz < ((double)candidate->deglitchBandwidthHz * 0.90);
            }
        }
    }
    (void)clockStatus;
    return ESP_OK;
}

static bool sensorarrayFdcSweepCandidateIsWorking(const sensorarrayFdcSweepCandidateResult_t *candidate)
{
    if (!candidate || !candidate->valid) {
        return false;
    }
    if (candidate->validSampleCount < SENSORARRAY_FDC_SWEEP_CONFIRM_SAMPLES) {
        return false;
    }
    if (candidate->invalidSampleCount != 0u ||
        candidate->timeoutCount != 0u ||
        candidate->watchdogCount != 0u ||
        candidate->amplitudeFaultCount != 0u ||
        candidate->zeroRawCount != 0u ||
        candidate->saturatedCount != 0u) {
        return false;
    }
    if (!candidate->deglitchBandwidthOk || candidate->frequencyHz <= 0.0) {
        return false;
    }
    if (candidate->raw28Mean > 0u) {
        uint32_t span = candidate->raw28Max - candidate->raw28Min;
        if (((uint64_t)span * 1000ull) > ((uint64_t)candidate->raw28Mean * 80ull)) {
            return false;
        }
    }
    return true;
}

static bool sensorarrayFdcSweepCandidateIsFallbackUsable(const sensorarrayFdcSweepCandidateResult_t *candidate)
{
    if (!candidate || !candidate->valid) {
        return false;
    }
    return candidate->validSampleCount != 0u &&
           candidate->timeoutCount == 0u &&
           candidate->watchdogCount == 0u &&
           candidate->zeroRawCount == 0u &&
           candidate->saturatedCount == 0u &&
           candidate->raw28Mean != 0u &&
           candidate->frequencyHz > 0.0 &&
           candidate->deglitchBandwidthOk;
}

static int32_t sensorarrayFdcSweepCandidateScore(const sensorarrayFdcSweepCandidateResult_t *candidate)
{
    if (!candidate) {
        return INT_MIN;
    }

    int32_t score = sensorarrayFdcSweepCandidateIsWorking(candidate) ? 300 : 0;
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
    bool candidateWorking = sensorarrayFdcSweepCandidateIsWorking(candidate);
    bool bestWorking = sensorarrayFdcSweepCandidateIsWorking(best);
    if (candidateWorking != bestWorking) {
        return candidateWorking;
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
    if (!sensorarrayFdcSweepCandidateIsFallbackUsable(candidate)) {
        return false;
    }
    if (!sensorarrayFdcSweepCandidateIsFallbackUsable(best)) {
        return true;
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

static esp_err_t sensorarrayFdcSweepReadOneSampleBounded(Fdc2214CapDevice_t *dev,
                                                         Fdc2214CapChannel_t channel,
                                                         uint32_t timeoutMs,
                                                         sensorarrayFdcReadDiag_t *outDiag)
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
            return err;
        }

        bool terminal = diag.sample.SleepModeEnabled ||
                        !diag.sample.Converting ||
                        diag.sample.ErrWatchdog ||
                        diag.sample.ErrAmplitude;
        if (diag.sample.UnreadConversionPresent ||
            diag.sample.DataReady ||
            terminal ||
            diag.provisionalReadable) {
            *outDiag = diag;
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
    printf("FDC_SWEEP,stage=wait_sample,timeoutMs=%lu,polls=%lu,statusReg=0x%04X,unread=%u,dataReady=%u,result=timeout,err=%ld\n",
           (unsigned long)timeoutMs,
           (unsigned long)polls,
           lastDiag.sample.StatusRaw,
           lastDiag.sample.UnreadConversionPresent ? 1u : 0u,
           lastDiag.sample.DataReady ? 1u : 0u,
           (long)ESP_ERR_TIMEOUT);
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
    const char *source = reason ? reason : SENSORARRAY_NA;
    bool highCurrentApplied = false;
    if (highCurrent) {
        printf("FDC_SWEEP,stage=direct_lock,dev=%s,ch=%u,reason=%s,highCurrentReq=1,highCurrentApplied=0,status=matrix_autoscan_disables_high_current\n",
               devName,
               (unsigned)channel,
               source);
    }

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
                                                     const char *stage,
                                                     uint32_t candidateIndex,
                                                     uint32_t candidateCount,
                                                     sensorarrayFdcSweepCandidateResult_t *outCandidate)
{
    sensorarrayFdcDeviceState_t *fdcState = sensorarrayMeasureGetFdcState(state, devId);
    if (!fdcState || !fdcState->ready || !fdcState->handle || !deglitch || !outCandidate) {
        return ESP_ERR_INVALID_ARG;
    }

    *outCandidate = (sensorarrayFdcSweepCandidateResult_t){
        .valid = true,
        .deglitchReq = deglitch->deglitchCode,
        .deglitchName = deglitch->deglitchName,
        .deglitchBandwidthHz = deglitch->deglitchBandwidthHz,
        .driveCurrentReq = driveCurrent,
        .driveCurrentNorm = (uint16_t)(driveCurrent & SENSORARRAY_FDC_SWEEP_DRIVE_CURRENT_MASK),
        .raw28Min = SENSORARRAY_FDC_SWEEP_RAW28_MAX,
    };

    const char *devName = sensorarrayFdcSweepDevName(devId);
    const char *stageName = stage ? stage : "sweep";
    uint32_t stepTimeoutMs = sensorarrayFdcSweepStepTimeoutMs();
    int64_t candidateDeadlineUs = esp_timer_get_time() + ((int64_t)stepTimeoutMs * 1000LL);

    printf("FDC_SWEEP,stage=%s_candidate_begin,dev=%s,ch=%u,index=%lu,count=%lu,deglitch=0x%X,deglitchName=%s,bandwidthHz=%lu,drive=0x%04X,fastProbe=%u,confirm=%u,timeoutMs=%lu\n",
           stageName,
           devName,
           (unsigned)channel,
           (unsigned long)candidateIndex,
           (unsigned long)candidateCount,
           (unsigned)deglitch->deglitchCode,
           deglitch->deglitchName,
           (unsigned long)deglitch->deglitchBandwidthHz,
           outCandidate->driveCurrentNorm,
           (unsigned)SENSORARRAY_FDC_SWEEP_FAST_PROBE_SAMPLES,
           (unsigned)SENSORARRAY_FDC_SWEEP_CONFIRM_SAMPLES,
           (unsigned long)stepTimeoutMs);

    esp_err_t firstErr = sensorarrayFdcSweepApplyDirectSafeLock(state,
                                                               devId,
                                                               channel,
                                                               deglitch->deglitchCode,
                                                               driveCurrent,
                                                               false,
                                                               stageName);
    if (firstErr != ESP_OK) {
        outCandidate->invalidSampleCount = 1u;
        printf("FDC_SWEEP,stage=%s_candidate_summary,dev=%s,ch=%u,index=%lu,err=%ld,stable=0,status=apply_failed\n",
               stageName,
               devName,
               (unsigned)channel,
               (unsigned long)candidateIndex,
               (long)firstErr);
        return firstErr;
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
    for (uint32_t sampleIndex = 0u;
         !failFast && sampleIndex < SENSORARRAY_FDC_SWEEP_MAX_SAMPLES_PER_CANDIDATE;
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
        esp_err_t readErr = sensorarrayFdcSweepReadOneSampleBounded(fdcState->handle,
                                                                    channel,
                                                                    sampleTimeoutMs,
                                                                    &diag);
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

        bool saturated = diag.sample.Raw28 >= SENSORARRAY_FDC_SWEEP_RAW28_SATURATED_THRESHOLD;
        bool sampleValid = diag.provisionalReadable &&
                           diag.sample.UnreadConversionPresent &&
                           diag.sample.Raw28 != 0u &&
                           !saturated &&
                           !diag.sample.ErrWatchdog &&
                           !diag.sample.ErrAmplitude;
        outCandidate->raw28Last = diag.sample.Raw28;
        if (diag.sample.Raw28 == 0u) {
            outCandidate->zeroRawCount++;
        }
        if (saturated) {
            outCandidate->saturatedCount++;
        }
        if (diag.sample.ErrWatchdog) {
            outCandidate->watchdogCount++;
        }
        if (diag.sample.ErrAmplitude) {
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
        } else {
            outCandidate->invalidSampleCount++;
        }

        if (sampleIndex + 1u >= SENSORARRAY_FDC_SWEEP_FAST_PROBE_SAMPLES) {
            if (outCandidate->timeoutCount != 0u ||
                outCandidate->watchdogCount != 0u ||
                outCandidate->saturatedCount != 0u ||
                outCandidate->zeroRawCount >= SENSORARRAY_FDC_SWEEP_FAST_PROBE_SAMPLES ||
                outCandidate->amplitudeFaultCount >= SENSORARRAY_FDC_SWEEP_FAST_PROBE_SAMPLES ||
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
    outCandidate->stable = sensorarrayFdcSweepCandidateIsWorking(outCandidate);

    printf("FDC_SWEEP,stage=%s_candidate_summary,dev=%s,ch=%u,index=%lu,err=%ld,stable=%u,failReason=%s,drive=0x%04X,deglitch=0x%X,freqHz=%.3f,rawMin=%lu,rawMax=%lu,rawMean=%lu,valid=%lu,invalid=%lu,wd=%lu,amp=%lu,saturated=%lu,zero=%lu,timeout=%lu,score=%ld,status=%s\n",
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
    uint32_t candidateCount = deglitchCount * driveCount;

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
                                                                         stage,
                                                                         tried,
                                                                         candidateCount,
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
            if (haveBest && !haveFallback && candidate.driveCurrentNorm <= 0x7000u) {
                /* Low-current stable candidates are preferred for the observed overdrive case. */
                goto sweep_done;
            }
        }
    }

sweep_done:
    outResult->elapsedMs = sensorarrayFdcSweepNowMs() - startMs;
    const sensorarrayFdcSweepCandidateResult_t *selected = NULL;
    bool fallbackSelected = false;
    if (haveBest) {
        selected = &best;
    } else if (haveFallback) {
        selected = &fallback;
        fallbackSelected = true;
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
                                                                false,
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
    outResult->selectedHighCurrent = false;
    outResult->selectedConfig = selectedCopy.configReg;
    outResult->selectedMuxConfig = selectedCopy.muxConfigReg;
    outResult->selectedClockDividers = selectedCopy.clockDividersReg;
    outResult->selectedRaw28 = selectedCopy.raw28Last ? selectedCopy.raw28Last : selectedCopy.raw28Mean;
    outResult->selectedFrequencyHz = selectedCopy.frequencyHz;
    outResult->lastErr = fallbackSelected ? ESP_ERR_INVALID_RESPONSE : ESP_OK;

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
           fallbackSelected ? 1u : 0u,
           fallbackSelected ? "fallback" : "ok",
           (long)outResult->lastErr,
           (unsigned long)tried,
           (unsigned long)outResult->elapsedMs);

    return ESP_OK;
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
    if (err == ESP_OK && Fdc2214CapReadCoreRegs(fdcState->handle, &regs) == ESP_OK) {
        fdcState->statusConfigReg = regs.StatusConfig;
        fdcState->configReg = regs.Config;
        fdcState->muxConfigReg = regs.MuxConfig;
    }

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
    esp_err_t firstErr = ESP_OK;
    for (uint8_t ch = 0u; ch < 4u; ++ch) {
        if ((channelMask & (uint8_t)(1u << ch)) == 0u) {
            continue;
        }
        sensorarrayFdcSweepResult_t result = {0};
        esp_err_t err = sensorarrayFdcSweepRunChannel(state,
                                                      devId,
                                                      (Fdc2214CapChannel_t)ch,
                                                      reason,
                                                      &result);
        if (err != ESP_OK && firstErr == ESP_OK) {
            firstErr = err;
        }
    }

    esp_err_t restoreErr = sensorarrayFdcSweepRestoreAutoscan(state, devId, reason);
    if (restoreErr != ESP_OK && firstErr == ESP_OK) {
        firstErr = restoreErr;
    }
    return firstErr;
}

esp_err_t sensorarrayFdcSweepRunBoot(sensorarrayState_t *state)
{
    if (!state) {
        return ESP_ERR_INVALID_ARG;
    }

    printf("FDC_SWEEP,stage=boot,event=start\n");
    esp_err_t pathErr = sensorarrayMeasureEnsureFdcMatrixPath(state, "fdc_boot_sweep");
    if (pathErr == ESP_OK) {
        pathErr = tmuxSwitchSelectRow(0u);
    }
    if (pathErr != ESP_OK) {
        printf("FDC_SWEEP,stage=boot,event=path_failed,err=%ld\n", (long)pathErr);
        return pathErr;
    }

    esp_err_t firstErr = sensorarrayFdcSweepRunDevice(state,
                                                      SENSORARRAY_FDC_DEV_PRIMARY,
                                                      0x0Fu,
                                                      "boot");
    esp_err_t secondaryErr = sensorarrayFdcSweepRunDevice(state,
                                                          SENSORARRAY_FDC_DEV_SECONDARY,
                                                          0x0Fu,
                                                          "boot");
    if (secondaryErr != ESP_OK && firstErr == ESP_OK) {
        firstErr = secondaryErr;
    }

    printf("FDC_SWEEP,stage=boot,event=done,err=%ld,result=%s\n",
           (long)firstErr,
           (firstErr == ESP_OK) ? "ok" : "warning_or_failed");
    return firstErr;
}

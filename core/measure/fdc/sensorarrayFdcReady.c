#include "sensorarrayFdcInternal.h"

// Split from core/measure/sensorarrayMeasure.c to keep FDC matrix internals isolated.

bool sensorarrayMeasureFdcDeviceReadyForIo(const sensorarrayFdcDeviceState_t *fdcState)
{
    return fdcState && fdcState->ready && fdcState->handle && fdcState->i2cCtx;
}

void sensorarrayMeasureLogFdcSecondaryUnavailableOnce(void)
{
    if (s_fdcSecondaryUnavailableLogged) {
        return;
    }
    s_fdcSecondaryUnavailableLogged = true;
    printf("FDC_BUS_WARN,secondaryReady=0,action=fill_d5_d8_minus_one\n");
}

esp_err_t sensorarrayMeasureFdcSetSleepMode(sensorarrayFdcDeviceState_t *fdcState,
                                                   bool enable,
                                                   sensorarrayFdcDeviceTiming_t *timing)
{
    if (!fdcState || !fdcState->handle) {
        return ESP_ERR_INVALID_ARG;
    }

    uint16_t baseConfig = sensorarrayMeasureFdcConfigBaseWithoutSleep(fdcState);
    int64_t startUs = esp_timer_get_time();
    esp_err_t err = enable ?
        Fdc2214CapEnterSleepWriteOnly(fdcState->handle, baseConfig) :
        Fdc2214CapExitSleepWriteOnly(fdcState->handle, baseConfig);
    uint64_t elapsedUs = sensorarrayMeasureElapsedUs(startUs);
    if (timing) {
        if (enable) {
            timing->sleepEnterUs += elapsedUs;
        } else {
            timing->sleepExitUs += elapsedUs;
        }
        timing->diffConfigWrites++;
    }
    if (err == ESP_OK) {
        uint16_t config = enable ?
            (uint16_t)(baseConfig | SENSORARRAY_FDC_CONFIG_SLEEP_MODE_EN_MASK) :
            (uint16_t)(baseConfig & (uint16_t)~SENSORARRAY_FDC_CONFIG_SLEEP_MODE_EN_MASK);
        fdcState->configReg = config;
        fdcState->configVerified = true;
    } else {
        fdcState->configVerified = false;
    }
    return err;
}

const char *sensorarrayMeasureFdcRrSequenceName(uint8_t rrSequence)
{
    switch (rrSequence) {
    case FDC2214_RR_SEQUENCE_CH0_CH1:
        return "CH0_CH1";
    case FDC2214_RR_SEQUENCE_CH0_CH1_CH2:
        return "CH0_CH1_CH2";
    case FDC2214_RR_SEQUENCE_CH0_CH1_CH2_CH3:
        return "CH0_CH1_CH2_CH3";
    default:
        return "invalid";
    }
}

esp_err_t sensorarrayMeasureFdcVerifySleepExit(sensorarrayState_t *state,
                                                      uint8_t row,
                                                      uint32_t epochId,
                                                      sensorarrayFdcDeviceId_t devId,
                                                      sensorarrayFdcDeviceTiming_t *timing)
{
    if (!state || devId > SENSORARRAY_FDC_DEV_SECONDARY) {
        return ESP_ERR_INVALID_ARG;
    }

    sensorarrayFdcDeviceState_t *fdcState = sensorarrayMeasureGetFdcState(state, devId);
    if (!fdcState || !fdcState->handle) {
        return ESP_ERR_INVALID_STATE;
    }

    uint16_t config = 0u;
    uint16_t mux = 0u;
    uint16_t statusConfig = 0u;
    esp_err_t firstErr = ESP_OK;

    int64_t verifyStartUs = esp_timer_get_time();
    esp_err_t err = Fdc2214CapReadRawRegisters(fdcState->handle,
                                               SENSORARRAY_FDC_REG_CONFIG,
                                               &config);
    sensorarrayMeasureRecordFirstErr(err, &firstErr);
    err = Fdc2214CapReadRawRegisters(fdcState->handle,
                                     SENSORARRAY_FDC_REG_MUX_CONFIG,
                                     &mux);
    sensorarrayMeasureRecordFirstErr(err, &firstErr);
    err = Fdc2214CapReadRawRegisters(fdcState->handle,
                                     SENSORARRAY_FDC_REG_STATUS_CONFIG,
                                     &statusConfig);
    sensorarrayMeasureRecordFirstErr(err, &firstErr);
    if (timing) {
        timing->verifyUs += sensorarrayMeasureElapsedUs(verifyStartUs);
    }

    bool sleepBit = (config & SENSORARRAY_FDC_CONFIG_SLEEP_MODE_EN_MASK) != 0u;
    bool autoscan = (mux & SENSORARRAY_FDC_MUX_AUTOSCAN_EN_MASK) != 0u;
    uint8_t rrSequence = (uint8_t)((mux & SENSORARRAY_FDC_MUX_RR_SEQUENCE_MASK) >>
                                   SENSORARRAY_FDC_MUX_RR_SEQUENCE_SHIFT);
    bool statusConfigOk = statusConfig == SENSORARRAY_FDC_STATUS_CONFIG_DEFAULT;
    bool verifyOk = firstErr == ESP_OK &&
                    !sleepBit &&
                    autoscan &&
                    rrSequence == SENSORARRAY_FDC_AUTOSCAN_RR_SEQUENCE &&
                    statusConfigOk;
    const char *diagnostic = verifyOk ? "ok" :
        (firstErr != ESP_OK) ? "i2c_error" :
        sleepBit ? "still_sleep" :
        (!autoscan || rrSequence != SENSORARRAY_FDC_AUTOSCAN_RR_SEQUENCE) ? "autoscan_config_lost" :
        "status_config_mismatch";

    if (!verifyOk && firstErr == ESP_OK) {
        const sensorarrayFdcAppliedRowConfig_t *applied = &state->fdcAppliedRow[(uint8_t)devId];
        esp_err_t rewriteErr = Fdc2214CapWriteRawRegisters(fdcState->handle,
                                                           SENSORARRAY_FDC_REG_STATUS_CONFIG,
                                                           applied->valid ? applied->statusConfig :
                                                           SENSORARRAY_FDC_STATUS_CONFIG_DEFAULT);
        sensorarrayMeasureRecordFirstErr(rewriteErr, &firstErr);
        rewriteErr = Fdc2214CapWriteRawRegisters(fdcState->handle,
                                                 SENSORARRAY_FDC_REG_MUX_CONFIG,
                                                 applied->valid ? applied->muxConfig :
                                                 sensorarrayMeasureFdcBuildMuxConfig(sensorarrayMeasureFdcSafeDefaultDeglitch()));
        sensorarrayMeasureRecordFirstErr(rewriteErr, &firstErr);
        rewriteErr = Fdc2214CapExitSleep(fdcState->handle,
                                         applied->valid ? applied->configBaseWithoutSleepBit :
                                         sensorarrayMeasureFdcConfigBaseWithoutSleep(fdcState));
        sensorarrayMeasureRecordFirstErr(rewriteErr, &firstErr);
        state->fdcAppliedRow[(uint8_t)devId].dirty = true;

        uint16_t configAfter = config;
        uint16_t muxAfter = mux;
        uint16_t statusConfigAfter = statusConfig;
        (void)Fdc2214CapReadRawRegisters(fdcState->handle,
                                         SENSORARRAY_FDC_REG_CONFIG,
                                         &configAfter);
        (void)Fdc2214CapReadRawRegisters(fdcState->handle,
                                         SENSORARRAY_FDC_REG_MUX_CONFIG,
                                         &muxAfter);
        (void)Fdc2214CapReadRawRegisters(fdcState->handle,
                                         SENSORARRAY_FDC_REG_STATUS_CONFIG,
                                         &statusConfigAfter);
        config = configAfter;
        mux = muxAfter;
        statusConfig = statusConfigAfter;
        sleepBit = (config & SENSORARRAY_FDC_CONFIG_SLEEP_MODE_EN_MASK) != 0u;
        autoscan = (mux & SENSORARRAY_FDC_MUX_AUTOSCAN_EN_MASK) != 0u;
        rrSequence = (uint8_t)((mux & SENSORARRAY_FDC_MUX_RR_SEQUENCE_MASK) >>
                               SENSORARRAY_FDC_MUX_RR_SEQUENCE_SHIFT);
        diagnostic = (firstErr == ESP_OK && !sleepBit && autoscan &&
                      rrSequence == SENSORARRAY_FDC_AUTOSCAN_RR_SEQUENCE) ?
            "rewritten_ok" :
            diagnostic;
    }

    if (firstErr == ESP_OK) {
        fdcState->configReg = config;
        fdcState->muxConfigReg = mux;
        fdcState->statusConfigReg = statusConfig;
        fdcState->configVerified = true;
    } else {
        fdcState->configVerified = false;
    }

    if (!verifyOk || CONFIG_SENSORARRAY_FDC_VERBOSE_SCAN_LOG) {
        printf("FDC_SLEEP_EXIT_VERIFY,row=%u,epoch=%lu,dev=%s,config=0x%04X,sleepBit=%u,mux=0x%04X,autoscan=%u,rrSequence=%s,statusConfig=0x%04X,statusRead=0,err=0x%lx,diagnostic=%s\n",
               (unsigned)row,
               (unsigned long)epochId,
               sensorarrayMeasureFdcDeviceName(devId),
               config,
               sleepBit ? 1u : 0u,
               mux,
               autoscan ? 1u : 0u,
               sensorarrayMeasureFdcRrSequenceName(rrSequence),
               statusConfig,
               (unsigned long)firstErr,
               diagnostic);
        printf("FDC_EPOCH,stage=after_exit_sleep,row=%u,dev=%s,cfgBefore=0x%04X,cfgAfter=0x%04X,sleepBefore=%u,sleepAfter=%u,statusRead=0,intbLevel=%d,mux=0x%04X,errorConfig=0x%04X\n",
               (unsigned)row,
               sensorarrayMeasureFdcDeviceName(devId),
               config,
               config,
               sleepBit ? 1u : 0u,
               sleepBit ? 1u : 0u,
               (int)((sensorarrayMeasureFdcWorkerContext(devId) &&
                      sensorarrayMeasureFdcWorkerContext(devId)->intbReady) ?
                         gpio_get_level((gpio_num_t)sensorarrayMeasureFdcWorkerContext(devId)->intbGpio) :
                         -1),
               mux,
               statusConfig);
    }

    return firstErr;
}

__attribute__((unused)) const char *sensorarrayMeasureFdcReadyKindName(sensorarrayFdcReadyKind_t kind)
{
    switch (kind) {
    case SENSORARRAY_FDC_READY_EDGE_WAKE:
        return "edge_wake";
    case SENSORARRAY_FDC_READY_STATUS_READY_BEFORE_WAIT:
        return "ready_before_wait";
    case SENSORARRAY_FDC_READY_POLL_FULL:
        return "data_ready_full";
    case SENSORARRAY_FDC_READY_POLL_RECOVERED_AFTER_UNREAD_BEFORE_DRDY:
        return "recovered_after_wait_drdy";
    case SENSORARRAY_FDC_READY_AFTER_INTB_RECHECK_FULL:
        return "after_intb_recheck_full";
    case SENSORARRAY_FDC_READY_INTB_TIMEOUT:
        return "intb_timeout";
    case SENSORARRAY_FDC_READY_DRDY_NOT_CLOSED_AFTER_INTB:
        return "drdy_not_closed_after_intb";
    case SENSORARRAY_FDC_READY_STATUS_INCONSISTENT:
        return "status_inconsistent";
    case SENSORARRAY_FDC_READY_POLL_PARTIAL:
        return "poll_partial";
    case SENSORARRAY_FDC_READY_TIMEOUT_PARTIAL:
        return "timeout_partial";
    case SENSORARRAY_FDC_READY_TIMEOUT_NONE:
        return "timeout_none";
    case SENSORARRAY_FDC_READY_I2C_ERROR:
        return "i2c_error";
    case SENSORARRAY_FDC_READY_UNREAD_FULL_NO_DRDY_TRANSIENT:
        return "unread_full_no_drdy_transient";
    case SENSORARRAY_FDC_READY_STALE_UNREAD_NO_DRDY:
        return "stale_unread_no_drdy";
    case SENSORARRAY_FDC_READY_HARD_TIMEOUT:
        return "hard_timeout";
    case SENSORARRAY_FDC_READY_STATUS_FALLBACK_AFTER_INTB_MISS:
        return "status_fallback_after_intb_miss";
    case SENSORARRAY_FDC_READY_STATUS_READY_AFTER_TIMEOUT:
        return "status_ready_after_timeout";
    case SENSORARRAY_FDC_READY_LEVEL_ACTIVE_FALLBACK:
        return "level_active_fallback";
    case SENSORARRAY_FDC_READY_WAIT_BUDGET_TOO_SHORT_STATUS_FALLBACK:
        return "wait_budget_too_short_status_fallback";
    case SENSORARRAY_FDC_READY_EPOCH_MISMATCH_OR_STALE_WORKER_RESULT:
        return "epoch_mismatch_or_stale_worker_result";
    case SENSORARRAY_FDC_READY_NONE:
    default:
        return "none";
    }
}

bool sensorarrayMeasureFdcShouldLogNormalFrame(uint32_t frameSeq)
{
    if (CONFIG_SENSORARRAY_FDC_VERBOSE_SCAN_LOG) {
        return true;
    }
    uint32_t period = (uint32_t)CONFIG_SENSORARRAY_FDC_SAMPLE_DEVICE_LOG_EVERY_N_FRAMES;
    return period != 0u && frameSeq != 0u && (frameSeq % period) == 0u;
}

bool sensorarrayMeasureFdcShouldLogNormalPollSuccess(uint32_t frameSeq)
{
    return CONFIG_SENSORARRAY_FDC_LOG_NORMAL_POLL_SUCCESS ||
           sensorarrayMeasureFdcShouldLogNormalFrame(frameSeq);
}

bool sensorarrayMeasureFdcRead4IsFullSuccess(const sensorarrayFdcDeviceRead4Result_t *read4)
{
    return sensorarrayFdcRead4IsDataCompleteGood(read4,
                                                 SENSORARRAY_FDC_AUTOSCAN_READY_UNREAD_MASK);
}

bool sensorarrayMeasureFdcWorkerResultIsGood(const sensorarrayFdcWorkerResult_t *result,
                                                    uint8_t expectedRow,
                                                    uint32_t expectedEpoch,
                                                    sensorarrayFdcDeviceId_t expectedDev)
{
    return result &&
           result->err == ESP_OK &&
           result->row == expectedRow &&
           result->epochId == expectedEpoch &&
           result->devId == expectedDev &&
           sensorarrayMeasureFdcRead4IsFullSuccess(&result->read4);
}

void sensorarrayMeasureFdcFrameTrackerNoteRead(sensorarrayFdcFrameReadTracker_t *tracker,
                                                      uint8_t row,
                                                      uint32_t epochId,
                                                      sensorarrayFdcDeviceId_t devId,
                                                      const sensorarrayFdcDeviceRead4Result_t *read4,
                                                      const char *reason)
{
    if (!tracker || row == 0u || row > SENSORARRAY_MATRIX_ROWS ||
        devId > SENSORARRAY_FDC_DEV_SECONDARY) {
        return;
    }

    bool *okRead = &tracker->okRead[row - 1u][(uint8_t)devId];
    if (*okRead) {
        tracker->duplicateReadCount++;
        printf("FDC_DUPLICATE_READ_ERROR,seq=%lu,row=%u,epoch=%lu,device=%s,previousOk=1,currentReason=%s,validMask=0x%X,err=0x%lx\n",
               (unsigned long)tracker->frameSeq,
               (unsigned)row,
               (unsigned long)epochId,
               sensorarrayMeasureFdcDeviceName(devId),
               reason ? reason : "unknown",
               read4 ? (unsigned)read4->validMask4 : 0u,
               read4 ? (unsigned long)read4->readErr : (unsigned long)ESP_ERR_INVALID_ARG);
    }
    if (sensorarrayMeasureFdcRead4IsFullSuccess(read4)) {
        *okRead = true;
    }
}

uint32_t sensorarrayMeasureClampU32(uint32_t value, uint32_t minValue, uint32_t maxValue)
{
    if (value < minValue) {
        return minValue;
    }
    if (value > maxValue) {
        return maxValue;
    }
    return value;
}

uint32_t sensorarrayMeasureFdcCeilDivU64(uint64_t numerator, uint64_t denominator)
{
    if (denominator == 0u) {
        return 0u;
    }
    return (uint32_t)((numerator + denominator - 1u) / denominator);
}

uint32_t sensorarrayMeasureFdcEstimateAutoscanReadyTimeoutUsWithSnapshot(
    const sensorarrayFdcRuntimeChannelConfig_t configs[4],
    uint8_t requiredUnreadMask,
    uint32_t *outEstimatedRoundUs,
    sensorarrayFdcProfileSnapshot_t *snapshot)
{
    uint64_t estimatedRoundUs = 0u;
    sensorarrayFdcProfileSnapshot_t localSnapshot = {0};

    for (uint8_t ch = 0u; ch < 4u; ++ch) {
        uint16_t rCount = (configs && configs[ch].rCount != 0u) ?
            configs[ch].rCount :
            SENSORARRAY_FDC_RCOUNT;
        uint16_t settleCount = (configs && configs[ch].settleCount != 0u) ?
            configs[ch].settleCount :
            SENSORARRAY_FDC_SETTLECOUNT;
        uint16_t clockDiv = (configs && configs[ch].clockDividers != 0u) ?
            configs[ch].clockDividers :
            SENSORARRAY_FDC_CLOCK_DIVIDERS;
        uint32_t effectiveFclkHz = (configs && configs[ch].effectiveFclkHz != 0u) ?
            configs[ch].effectiveFclkHz :
            sensorarrayMeasureFdcEffectiveFclkHz();
        if ((requiredUnreadMask & (uint8_t)(1u << ch)) == 0u) {
            localSnapshot.rCount[ch] = rCount;
            localSnapshot.settleCount[ch] = settleCount;
            localSnapshot.clockDividers[ch] = clockDiv;
            localSnapshot.driveCurrent[ch] = configs ? configs[ch].driveCurrent : SENSORARRAY_FDC_DRIVE_CURRENT;
            localSnapshot.deglitchCode[ch] = configs ? configs[ch].deglitchCode : sensorarrayMeasureFdcSafeDefaultDeglitch();
            localSnapshot.effectiveFclkHz[ch] = effectiveFclkHz;
            continue;
        }

        double frefDividerDouble = sensorarrayMeasureFdcFrefDividerFromClockDiv(clockDiv);
        uint32_t frefDivider = (frefDividerDouble > 0.0) ? (uint32_t)frefDividerDouble : 1u;
        uint32_t frefHz = (frefDivider != 0u) ? (effectiveFclkHz / frefDivider) : effectiveFclkHz;
        if (frefHz == 0u) {
            frefHz = SENSORARRAY_FDC_REF_CLOCK_HZ;
        }

        uint32_t settleUs = sensorarrayMeasureFdcCeilDivU64((uint64_t)settleCount * 16u * 1000000u,
                                                            frefHz);
        uint32_t convUs = sensorarrayMeasureFdcCeilDivU64((uint64_t)rCount * 16u * 1000000u,
                                                          frefHz);
        uint32_t switchUs =
            sensorarrayMeasureFdcCeilDivU64((692u * (uint64_t)frefHz) + 5000000000ull,
                                            (uint64_t)frefHz * 1000000000ull);
        if (switchUs < 2u) {
            switchUs = 2u;
        }
        localSnapshot.rCount[ch] = rCount;
        localSnapshot.settleCount[ch] = settleCount;
        localSnapshot.clockDividers[ch] = clockDiv;
        localSnapshot.driveCurrent[ch] = configs ? configs[ch].driveCurrent : SENSORARRAY_FDC_DRIVE_CURRENT;
        localSnapshot.deglitchCode[ch] = configs ? configs[ch].deglitchCode : sensorarrayMeasureFdcSafeDefaultDeglitch();
        localSnapshot.effectiveFclkHz[ch] = effectiveFclkHz;
        localSnapshot.chSettleUs[ch] = settleUs;
        localSnapshot.chConvertUs[ch] = convUs;
        localSnapshot.chSwitchUs[ch] = switchUs;
        localSnapshot.chTotalUs[ch] = settleUs + convUs + switchUs;
        estimatedRoundUs += (uint64_t)settleUs + convUs + switchUs;
    }

    if (estimatedRoundUs > UINT32_MAX) {
        estimatedRoundUs = UINT32_MAX;
    }
    if (outEstimatedRoundUs) {
        *outEstimatedRoundUs = (uint32_t)estimatedRoundUs;
    }
    localSnapshot.valid = true;
    localSnapshot.sourceIsShadow = true;
    localSnapshot.autoscanRoundUs = (uint32_t)estimatedRoundUs;

    uint64_t timeoutUs = (estimatedRoundUs * 2u) + 3000u;
    if (timeoutUs > UINT32_MAX) {
        timeoutUs = UINT32_MAX;
    }
    uint32_t clampedTimeoutUs = sensorarrayMeasureClampU32((uint32_t)timeoutUs, 25000u, 80000u);
    localSnapshot.expectedTimeoutUs = clampedTimeoutUs;
    if (snapshot) {
        *snapshot = localSnapshot;
    }
    return clampedTimeoutUs;
}

uint32_t sensorarrayMeasureFdcEstimateAutoscanReadyTimeoutUsFromConfigs(
    const sensorarrayFdcRuntimeChannelConfig_t configs[4],
    uint8_t requiredUnreadMask,
    uint32_t *outEstimatedRoundUs)
{
    return sensorarrayMeasureFdcEstimateAutoscanReadyTimeoutUsWithSnapshot(configs,
                                                                          requiredUnreadMask,
                                                                          outEstimatedRoundUs,
                                                                          NULL);
}

uint32_t sensorarrayMeasureFdcEstimateAppliedRowReadyTimeoutUs(
    const sensorarrayState_t *state,
    sensorarrayFdcDeviceId_t devId,
    uint8_t requiredUnreadMask,
    uint32_t *outEstimatedRoundUs)
{
    sensorarrayFdcRuntimeChannelConfig_t configs[4] = {0};
    if (state && devId <= SENSORARRAY_FDC_DEV_SECONDARY) {
        const sensorarrayFdcAppliedRowConfig_t *applied = &state->fdcAppliedRow[(uint8_t)devId];
        const sensorarrayFdcDeviceState_t *fdcState =
            (devId == SENSORARRAY_FDC_DEV_SECONDARY) ? &state->fdcSecondary : &state->fdcPrimary;
        uint32_t effectiveFclkHz =
            (fdcState->refClockKnown && fdcState->refClockHz != 0u) ?
            fdcState->refClockHz :
            sensorarrayMeasureFdcEffectiveFclkHz();
        for (uint8_t ch = 0u; ch < 4u; ++ch) {
            configs[ch] = (sensorarrayFdcRuntimeChannelConfig_t){
                .valid = applied->valid,
                .rCount = applied->rCount[ch],
                .settleCount = applied->settleCount[ch],
                .clockDividers = applied->clockDiv[ch],
                .driveCurrent = applied->driveCurrent[ch],
                .deglitchCode = applied->selectedDeglitch,
                .effectiveFclkHz = effectiveFclkHz,
            };
        }
    }
    return sensorarrayMeasureFdcEstimateAutoscanReadyTimeoutUsFromConfigs(configs,
                                                                         requiredUnreadMask,
                                                                         outEstimatedRoundUs);
}

const char *sensorarrayMeasureFdcReadyDiagnosticName(const sensorarrayFdcReadyState_t *ready,
                                                           uint8_t requiredUnreadMask)
{
    if (!ready) {
        return "invalid_ready_state";
    }
    if (ready->pollCount == 0u) {
        return "bug_ready_poll_not_entered";
    }
    if (ready->i2cError || ready->kind == SENSORARRAY_FDC_READY_I2C_ERROR) {
        return "i2c_error";
    }
    if (ready->readyForDataRead) {
        return "data_ready_full_unread";
    }
    if (ready->unreadWithoutDataReady) {
        return "wait_drdy_after_unread_full";
    }
    if (ready->dataReady && (ready->unreadMask & requiredUnreadMask) != 0u) {
        return "drdy_partial_unread";
    }
    if (ready->dataReady) {
        return "drdy_without_unread";
    }
    if ((ready->unreadMask & requiredUnreadMask) != 0u) {
        return "partial_unread_without_drdy";
    }
    return "data_not_ready";
}

void sensorarrayMeasureFdcUpdateReadyTiming(const sensorarrayFdcReadyState_t *ready,
                                                   sensorarrayFdcDeviceTiming_t *timing,
                                                   bool fallbackAttempted)
{
    if (!ready || !timing) {
        return;
    }

    timing->readyPollCount += ready->pollCount;
    timing->unreadWithoutDrdyCount += ready->unreadWithoutDrdyCount;
    if (ready->intbBeforeStatus == 0) {
        timing->alreadyLowCount++;
    }
    if (ready->readyResult == FDC_READY_RECOVERED_AFTER_RETRY) {
        timing->recoveredAfterRetryCount++;
    } else if (ready->readyResult == FDC_READY_UNREAD_FULL_NO_DRDY_TRANSIENT) {
        timing->transientUnreadNoDrdyCount++;
    } else if (ready->readyResult == FDC_READY_HARD_TIMEOUT ||
               ready->readyResult == FDC_READY_HARD_TIMEOUT_NO_DRDY) {
        timing->hardReadyTimeoutCount++;
    }
    timing->statusReadsBeforeIntbCount += ready->statusReadsBeforeIntb;
    timing->statusReadsPrecheckCount += ready->statusReadsPrecheck;
    timing->statusReadsAfterIntbCount += ready->statusReadsAfterIntb +
                                         ready->statusReadsAfterIntbRecheck;
    timing->statusReadsInFallbackCount += ready->statusReadsInFallback;
    timing->statusReadSuppressedBeforeIntbCount += ready->statusReadsSuppressedBeforeIntb;
    timing->noStatusPollWaitCount += ready->statusReadsBeforeIntb == 0u ? 1u : 0u;
    timing->statusAfterIntbCount += ready->statusReadsAfterIntb +
                                    ready->statusReadsAfterIntbRecheck;
    timing->statusAfterTimeoutCount += ready->statusReadsInFallback;
    timing->hardTimeoutStatusDiagCount += ready->statusReadsWatchdogDiag;
    timing->intbActiveStatusMismatchCount +=
        ready->readyResult == FDC_READY_INTB_ACTIVE_STATUS_MISMATCH ? 1u : 0u;
    timing->suppressedRpCount += ready->statusReadsSuppressedBeforeIntb;
    timing->internalWaitStateLeakCount +=
        ready->readyResult == FDC_READY_INTERNAL_STATE_ERROR ? 1u : 0u;
    timing->intbWaitOnlyUs += ready->intbWaitUs;
    timing->statusVerifyAfterIntbUs += ready->statusVerifyUs;
    timing->pollFallbackUs += ready->pollFallbackUs;
    if (ready->i2cError || ready->kind == SENSORARRAY_FDC_READY_I2C_ERROR) {
        timing->statusReadErrCount++;
    }
    if (ready->unreadWithoutDataReady) {
        timing->unsafeUnreadNoDrdyCount++;
    }
    if (ready->dataReady && !ready->unreadFull) {
        timing->drdyPartialUnreadCount++;
    }
    if (ready->readyForDataRead) {
        timing->drdyFullUnreadReadyCount++;
    }
    bool preStatusReady =
        ready->preStatusReady ||
        ready->kind == SENSORARRAY_FDC_READY_STATUS_READY_BEFORE_WAIT;
    bool intbReady =
        ready->kind == SENSORARRAY_FDC_READY_EDGE_WAKE ||
        ready->kind == SENSORARRAY_FDC_READY_AFTER_INTB_RECHECK_FULL ||
        ready->kind == SENSORARRAY_FDC_READY_LEVEL_ACTIVE_FALLBACK;
    bool lateStatusReady =
        ready->lateStatusReady ||
        ready->kind == SENSORARRAY_FDC_READY_STATUS_READY_AFTER_TIMEOUT;
    bool legacyStatusFallbackReady =
        ready->kind == SENSORARRAY_FDC_READY_STATUS_FALLBACK_AFTER_INTB_MISS ||
        ready->kind == SENSORARRAY_FDC_READY_WAIT_BUDGET_TOO_SHORT_STATUS_FALLBACK;
    if (preStatusReady && ready->readyForDataRead) {
        timing->preStatusReadyCount++;
        timing->readyFullCount++;
    } else if (intbReady && ready->readyForDataRead) {
        timing->intbReadyCount++;
        timing->readyFullCount++;
        timing->intbFreshDrdyCount++;
    } else if (lateStatusReady && ready->readyForDataRead) {
        timing->lateStatusReadyCount++;
        timing->readyFullCount++;
    } else if ((ready->kind == SENSORARRAY_FDC_READY_POLL_FULL ||
                legacyStatusFallbackReady) &&
               ready->readyForDataRead) {
        timing->readyFullCount++;
    } else if (ready->kind == SENSORARRAY_FDC_READY_POLL_RECOVERED_AFTER_UNREAD_BEFORE_DRDY &&
               ready->readyForDataRead) {
        timing->readyFullCount++;
    } else if (ready->kind == SENSORARRAY_FDC_READY_POLL_PARTIAL ||
               ready->kind == SENSORARRAY_FDC_READY_TIMEOUT_PARTIAL) {
        timing->readyPartialCount++;
    } else if (!sensorarrayMeasureFdcReadyResultIsSoftInvalid(ready) &&
               (ready->kind == SENSORARRAY_FDC_READY_INTB_TIMEOUT ||
                ready->kind == SENSORARRAY_FDC_READY_DRDY_NOT_CLOSED_AFTER_INTB ||
                ready->kind == SENSORARRAY_FDC_READY_STATUS_INCONSISTENT ||
                ready->kind == SENSORARRAY_FDC_READY_TIMEOUT_NONE ||
                ready->kind == SENSORARRAY_FDC_READY_HARD_TIMEOUT ||
                ready->kind == SENSORARRAY_FDC_READY_I2C_ERROR)) {
        timing->readyNoneCount++;
    }
    if (ready->timeout && !sensorarrayMeasureFdcReadyResultIsSoftInvalid(ready)) {
        timing->intbTimeoutCount++;
    }
    if (ready->trueTimeoutNotReady ||
        (ready->timeout && !ready->readyForDataRead &&
         !sensorarrayMeasureFdcReadyResultIsSoftInvalid(ready))) {
        timing->trueTimeoutCount++;
    }
    if (fallbackAttempted) {
        timing->fallbackAttemptCount++;
        if (ready->readyForDataRead) {
            timing->fallbackSuccessCount++;
            if (ready->unreadMask != SENSORARRAY_FDC_AUTOSCAN_READY_UNREAD_MASK) {
                timing->fallbackPartialCount++;
            }
        } else {
            timing->fallbackFailCount++;
        }
    }
    if (ready->statusFallbackUsed && !fallbackAttempted) {
        timing->fallbackSuccessCount++;
    }
    if (ready->waitUs > timing->maxWaitReadyUs) {
        timing->maxWaitReadyUs = ready->waitUs;
    }
}

const char *sensorarrayMeasureFdcDeviceToken(sensorarrayFdcDeviceId_t devId)
{
    return devId == SENSORARRAY_FDC_DEV_SECONDARY ? "s" : "p";
}

sensorarrayFdcReadyPolicy_t sensorarrayMeasureFdcConfiguredReadyPolicy(void)
{
#if SENSORARRAY_CFG_FDC_READY_POLL_ONLY
    return SENSORARRAY_FDC_READY_POLICY_POLL_ONLY;
#elif SENSORARRAY_CFG_FDC_STATUS_CONFIRM_ENABLED
    return SENSORARRAY_FDC_READY_POLICY_INTB_THEN_STATUS;
#elif SENSORARRAY_CFG_FDC_READY_STRICT_ENABLED
    return SENSORARRAY_FDC_READY_POLICY_INTB_STRICT_LEVEL;
#else
    return SENSORARRAY_FDC_READY_POLICY_INTB_WITH_POLL_FALLBACK;
#endif
}

sensorarrayFdcReadyPolicy_t sensorarrayMeasureFdcReadyPolicyForDevice(
    sensorarrayFdcDeviceId_t devId,
    const sensorarrayFdcWorkerContext_t *ctx)
{
    sensorarrayFdcReadyPolicy_t policy = sensorarrayMeasureFdcConfiguredReadyPolicy();
    if (policy == SENSORARRAY_FDC_READY_POLICY_POLL_ONLY) {
        return policy;
    }
    if (!ctx || !ctx->intbReady || devId > SENSORARRAY_FDC_DEV_SECONDARY ||
        !s_fdcIntbRuntimeUsable[(uint8_t)devId]) {
        if (policy == SENSORARRAY_FDC_READY_POLICY_INTB_STRICT_LEVEL) {
            return policy;
        }
        return SENSORARRAY_FDC_READY_POLICY_POLL_ONLY;
    }
    return policy;
}

const char *sensorarrayMeasureFdcReadyPolicyName(sensorarrayFdcReadyPolicy_t policy)
{
    switch (policy) {
    case SENSORARRAY_FDC_READY_POLICY_INTB_STRICT_LEVEL:
        return "INTB_STRICT_LEVEL";
    case SENSORARRAY_FDC_READY_POLICY_INTB_THEN_STATUS:
        return "INTB_THEN_STATUS";
    case SENSORARRAY_FDC_READY_POLICY_INTB_WITH_POLL_FALLBACK:
        return "INTB_WITH_POLL_FALLBACK";
    case SENSORARRAY_FDC_READY_POLICY_POLL_ONLY:
    default:
        return "POLL_ONLY";
    }
}

const char *sensorarrayMeasureFdcWatchdogReasonName(sensorarrayFdcWatchdogReason_t reason)
{
    switch (reason) {
    case SENSORARRAY_FDC_WATCHDOG_INTB_TIMEOUT:
        return "intb_wait_miss";
    case SENSORARRAY_FDC_WATCHDOG_DRDY_NOT_CLOSED_AFTER_INTB:
        return "drdy_not_closed_after_intb";
    case SENSORARRAY_FDC_WATCHDOG_STATUS_INCONSISTENT:
        return "status_inconsistent_after_intb";
    case SENSORARRAY_FDC_WATCHDOG_READ4_I2C_ERROR:
        return "read4_i2c_error";
    case SENSORARRAY_FDC_WATCHDOG_ZERO_AFTER_DRDY:
        return "zero_after_drdy";
    case SENSORARRAY_FDC_WATCHDOG_RAW_ALL_ZERO:
        return "raw_all_zero";
    case SENSORARRAY_FDC_WATCHDOG_AMPLITUDE_WARNING:
        return "amplitude_warning";
    case SENSORARRAY_FDC_WATCHDOG_SENSOR_WATCHDOG_FAULT:
        return "watchdog_fault";
    case SENSORARRAY_FDC_WATCHDOG_SATURATED:
        return "saturated";
    case SENSORARRAY_FDC_WATCHDOG_PROFILE_TOO_SLOW:
        return "profile_too_slow";
    default:
        return "unknown";
    }
}

uint32_t sensorarrayMeasureFdcRowDeviceWatchdogHardTimeoutUs(void)
{
    uint32_t overrideUs = (uint32_t)CONFIG_SENSORARRAY_FDC_ROW_DEVICE_WATCHDOG_OVERRIDE_US;
    if (overrideUs != 0u) {
        return overrideUs;
    }

    uint32_t rowCount = SENSORARRAY_MATRIX_ROWS;
    if (rowCount == 0u) {
        rowCount = 1u;
    }
    uint32_t singleRowBudgetUs = SENSORARRAY_FDC_TARGET_FRAME_US / rowCount;
    if (singleRowBudgetUs == 0u) {
        singleRowBudgetUs = 1u;
    }
    uint32_t multiplier = (uint32_t)CONFIG_SENSORARRAY_FDC_ROW_DEVICE_WATCHDOG_MULTIPLIER;
    if (multiplier == 0u) {
        multiplier = 1u;
    }
    uint64_t hardTimeoutUs = (uint64_t)singleRowBudgetUs * multiplier;
    return hardTimeoutUs > UINT32_MAX ? UINT32_MAX : (uint32_t)hardTimeoutUs;
}

esp_err_t sensorarrayFdcHandleRowDeviceWatchdog(
    sensorarrayState_t *state,
    sensorarrayFdcDeviceId_t devId,
    uint8_t row,
    uint32_t epochId,
    sensorarrayFdcWatchdogReason_t reason,
    sensorarrayFdcWorkerResult_t *result,
    sensorarrayFdcDeviceTiming_t *timing)
{
    (void)result;
    uint32_t singleRowBudgetUs =
        SENSORARRAY_MATRIX_ROWS ? (SENSORARRAY_FDC_TARGET_FRAME_US / SENSORARRAY_MATRIX_ROWS) : 0u;
    uint32_t hardTimeoutUs = sensorarrayMeasureFdcRowDeviceWatchdogHardTimeoutUs();
    if (timing) {
        timing->fallbackAttemptCount++;
        if (reason == SENSORARRAY_FDC_WATCHDOG_INTB_TIMEOUT) {
            timing->intbTimeoutCount++;
        }
    }
    uint8_t dBase = devId == SENSORARRAY_FDC_DEV_SECONDARY ? 5u : 1u;
    bool requestRescue =
        reason != SENSORARRAY_FDC_WATCHDOG_PROFILE_TOO_SLOW ||
        CONFIG_SENSORARRAY_FDC_PROFILE_TOO_SLOW_RESCUE_ENABLE;
    for (uint8_t ch = 0u; state && ch < 4u; ++ch) {
        if (!requestRescue) {
            continue;
        }
        uint8_t matrixIndex = (uint8_t)sensorarrayMatrixIndex(row, (uint8_t)(dBase + ch));
        (void)sensorarrayMeasureRequestFdcCellRescue(state,
                                                     matrixIndex,
                                                     sensorarrayMeasureFdcWatchdogReasonName(reason));
    }
    uint8_t rowSlot = (row > 0u && row <= SENSORARRAY_MATRIX_ROWS) ? (uint8_t)(row - 1u) : 0u;
    uint8_t devSlot = (uint8_t)devId;
    printf("RWD,d=%s,r=%u,e=%lu,why=%s,rowBudget=%lu,mul=%lu,hard=%lu,override=%lu,retryMax=%lu,retryActual=%lu,classification=hard,rescueAction=%s,consecutiveSoft=%u,consecutiveStale=%u,consecutiveHard=%u\n",
           sensorarrayMeasureFdcDeviceToken(devId),
           (unsigned)row,
           (unsigned long)epochId,
           sensorarrayMeasureFdcWatchdogReasonName(reason),
           (unsigned long)singleRowBudgetUs,
           (unsigned long)CONFIG_SENSORARRAY_FDC_ROW_DEVICE_WATCHDOG_MULTIPLIER,
           (unsigned long)hardTimeoutUs,
           (unsigned long)CONFIG_SENSORARRAY_FDC_ROW_DEVICE_WATCHDOG_OVERRIDE_US,
           (unsigned long)CONFIG_SENSORARRAY_FDC_ROW_DEVICE_RETRY_MAX,
           timing ? (unsigned long)timing->fallbackAttemptCount : 0ul,
           requestRescue ? "request_cell_rescue" : "profile_too_slow_diag_only",
           (unsigned)s_fdcSoftReadyMissConsecutive[devSlot][rowSlot],
           (unsigned)s_fdcStaleUnreadConsecutive[devSlot][rowSlot],
           (unsigned)s_fdcHardReadyTimeoutConsecutive[devSlot][rowSlot]);
    return ESP_OK;
}

TickType_t sensorarrayMeasureFdcTicksForUs(uint32_t waitUs)
{
    uint32_t waitMs = (waitUs + 999u) / 1000u;
    if (waitMs == 0u) {
        waitMs = 1u;
    }
    TickType_t ticks = pdMS_TO_TICKS(waitMs);
    return ticks == 0 ? 1 : ticks;
}

uint32_t sensorarrayMeasureFdcRemainingDeadlineUs(uint64_t deadlineUs)
{
    if (deadlineUs == 0u) {
        return UINT32_MAX;
    }
    int64_t nowUs = esp_timer_get_time();
    if ((uint64_t)nowUs >= deadlineUs) {
        return 0u;
    }
    uint64_t remainingUs = deadlineUs - (uint64_t)nowUs;
    return remainingUs > UINT32_MAX ? UINT32_MAX : (uint32_t)remainingUs;
}

const char *sensorarrayMeasureFdcEstimateKindName(uint8_t requiredUnreadMask)
{
    requiredUnreadMask &= 0x0Fu;
    if (requiredUnreadMask == SENSORARRAY_FDC_AUTOSCAN_READY_UNREAD_MASK) {
        return "autoscan_4ch_round";
    }
    if (requiredUnreadMask != 0u && (requiredUnreadMask & (uint8_t)(requiredUnreadMask - 1u)) == 0u) {
        return "single_channel";
    }
    return "unknown";
}

uint32_t sensorarrayMeasureFdcAddSaturateU32(uint32_t a, uint32_t b)
{
    uint64_t value = (uint64_t)a + b;
    return value > UINT32_MAX ? UINT32_MAX : (uint32_t)value;
}

uint32_t sensorarrayMeasureFdcComputeActualIntbWaitUs(uint32_t estimatedRoundUs,
                                                             uint64_t rowDeviceDeadlineUs,
                                                             sensorarrayFdcReadyState_t *ready)
{
    uint32_t readyGuardUs = (uint32_t)CONFIG_SENSORARRAY_FDC_READY_GUARD_US;
    uint32_t rowSafetyUs = (uint32_t)CONFIG_SENSORARRAY_FDC_ROW_WAIT_SAFETY_US;
    uint32_t waitMarginUs = rowSafetyUs > readyGuardUs ? rowSafetyUs : readyGuardUs;
    uint32_t estimatedWaitUs =
        sensorarrayMeasureFdcAddSaturateU32(estimatedRoundUs, waitMarginUs);
    uint32_t defaultWaitUs = (uint32_t)CONFIG_SENSORARRAY_FDC_INTB_WAIT_TIMEOUT_US;
    uint32_t actualWaitUs = defaultWaitUs;
    uint32_t plannedWaitUs = actualWaitUs;
    const char *waitSource = "default_intb";
    if (estimatedWaitUs > actualWaitUs) {
        actualWaitUs = estimatedWaitUs;
        plannedWaitUs = estimatedWaitUs;
        waitSource = "estimated_round";
    }

    uint32_t hardRemainUs = sensorarrayMeasureFdcRemainingDeadlineUs(rowDeviceDeadlineUs);
    if (ready) {
        ready->hardDeadlineRemainingBeforeWaitUs = hardRemainUs;
        ready->hardDeadlineRemainingUs = hardRemainUs;
        ready->rowDeviceHardUs = sensorarrayMeasureFdcRowDeviceWatchdogHardTimeoutUs();
        ready->guardMarginUs = readyGuardUs;
        ready->guardDeadlineUs = sensorarrayMeasureFdcReadyGuardDeadlineUs(estimatedRoundUs);
        ready->plannedIntbWaitUs = plannedWaitUs;
        ready->waitBudgetTooShort =
            hardRemainUs != UINT32_MAX && hardRemainUs < ready->guardDeadlineUs;
    }
    if (hardRemainUs != UINT32_MAX && actualWaitUs > hardRemainUs) {
        actualWaitUs = hardRemainUs;
        waitSource = "clamped_hard_deadline";
        if (ready) {
            ready->waitClampedByHardDeadline = true;
        }
    }
    if (actualWaitUs == 0u) {
        actualWaitUs = 1u;
    }
    if (ready) {
        ready->actualIntbWaitUs = actualWaitUs;
        ready->waitSource = waitSource;
        if (actualWaitUs < ready->guardDeadlineUs) {
            ready->waitBudgetTooShort = true;
        }
    }
    return actualWaitUs;
}

void sensorarrayMeasureFdcDrainCurrentTaskNotify(void)
{
    (void)xTaskNotifyStateClear(xTaskGetCurrentTaskHandle());
    while (ulTaskNotifyTake(pdTRUE, 0) != 0u) {
    }
}

void sensorarrayMeasureFdcArmCurrentTaskForIntb(sensorarrayFdcWorkerContext_t *ctx,
                                                       uint32_t epochId)
{
    if (!ctx || !ctx->intbReady) {
        return;
    }
    sensorarrayMeasureFdcDrainCurrentTaskNotify();
    ctx->currentEpoch = epochId;
    ctx->lastEpochSeen = 0u;
    ctx->lastEdgeUs = 0;
    ctx->waitTask = xTaskGetCurrentTaskHandle();
}

const char *sensorarrayMeasureFdcReadyPollToken(const sensorarrayFdcReadyState_t *ready,
                                                       uint8_t requiredUnreadMask)
{
    if (!ready) {
        return "BAD";
    }
    if (ready->i2cError || ready->kind == SENSORARRAY_FDC_READY_I2C_ERROR) {
        return "I2C";
    }
    if (ready->readyForDataRead) {
        return "FULL";
    }
    if (ready->unreadWithoutDataReady) {
        return "NO_DRDY";
    }
    if (ready->dataReady && (ready->unreadMask & requiredUnreadMask) != requiredUnreadMask) {
        return "WAIT_UNREAD";
    }
    if (ready->dataReady) {
        return "DRDY_NO_U";
    }
    if ((ready->unreadMask & requiredUnreadMask) != 0u) {
        return "PARTIAL_WAIT";
    }
    return "NOT_READY";
}

void sensorarrayMeasureFdcLogReadyCounts(sensorarrayFdcDeviceId_t devId,
                                                uint8_t row,
                                                uint32_t epochId,
                                                const sensorarrayFdcReadyState_t *ready)
{
    if (!ready) {
        return;
    }
    if (ready->readyResult == FDC_READY_OK_INTB_DRDY_UNREAD_FULL &&
        ready->err == ESP_OK &&
        !ready->timeout &&
        !ready->partial &&
        !ready->i2cError) {
        return;
    }
    uint32_t statusReadsTotal = ready->statusReadsBeforeIntb +
                                ready->statusReadsAfterIntb +
                                ready->statusReadsAfterIntbRecheck +
                                ready->statusReadsInFallback +
                                ready->statusReadsWatchdogDiag +
                                ready->statusReadsPollDiag;
    printf("SR,d=%s,r=%u,e=%lu,pre=%lu,bi=%lu,ai=%lu,ar=%lu,fb=%lu,wd=%lu,pd=%lu,sr=%lu,supp=%lu,ack=%lu,ib0=%d,ib1=%d\n",
           sensorarrayMeasureFdcDeviceToken(devId),
           (unsigned)row,
           (unsigned long)epochId,
           (unsigned long)ready->statusReadsPrecheck,
           (unsigned long)ready->statusReadsBeforeIntb,
           (unsigned long)ready->statusReadsAfterIntb,
           (unsigned long)ready->statusReadsAfterIntbRecheck,
           (unsigned long)ready->statusReadsInFallback,
           (unsigned long)ready->statusReadsWatchdogDiag,
           (unsigned long)ready->statusReadsPollDiag,
           (unsigned long)statusReadsTotal,
           (unsigned long)ready->statusReadsSuppressedBeforeIntb,
           (unsigned long)ready->statusAckCount,
           ready->intbBeforeStatus,
           ready->intbAfterStatus);
}

esp_err_t sensorarrayMeasureFdcReadStatusAndAckIntbForReady(sensorarrayFdcDeviceState_t *fdcState,
                                                                   sensorarrayFdcDeviceId_t devId,
                                                                   uint8_t row,
                                                                   uint32_t epochId,
                                                                   uint8_t requiredUnreadMask,
                                                                   bool afterIntb,
                                                                   bool afterIntbRecheck,
                                                                   bool fallback,
                                                                   bool hardTimeout,
                                                                   bool preWaitCheck,
                                                                   bool logNormalSuccess,
                                                                   sensorarrayFdcReadyState_t *ready,
                                                                   sensorarrayFdcDeviceTiming_t *timing,
                                                                   bool *bestPartialSeen,
                                                                   sensorarrayFdcReadyDecoded_t *bestPartial)
{
    Fdc2214CapStatus_t status = {0};
    sensorarrayFdcWorkerContext_t *ctx = sensorarrayMeasureFdcWorkerContext(devId);
    int intbBeforeStatus = (ctx && ctx->intbReady) ? gpio_get_level((gpio_num_t)ctx->intbGpio) : -1;
    int64_t statusStartUs = esp_timer_get_time();
    esp_err_t err = Fdc2214CapReadStatus(fdcState->handle, &status);
    uint64_t statusUs = sensorarrayMeasureElapsedUs(statusStartUs);
    int intbAfterStatus = (ctx && ctx->intbReady) ? gpio_get_level((gpio_num_t)ctx->intbGpio) : -1;
    if (timing) {
        timing->statusReadUs += statusUs;
        if (preWaitCheck) {
            timing->statusPrecheckUs += statusUs;
        }
    }
    ready->pollCount++;
    ready->intbBeforeStatus = intbBeforeStatus;
    ready->intbAfterStatus = intbAfterStatus;
    ready->err = err;
    if ((afterIntb || afterIntbRecheck) && intbBeforeStatus == 0) {
        ready->statusAckCount++;
    }
    if (preWaitCheck) {
        ready->statusReadsPrecheck++;
        ready->statusReadsBeforeIntb++;
        ready->statusPrecheckUs += (uint32_t)statusUs;
    } else if (hardTimeout) {
        ready->statusReadsWatchdogDiag++;
    } else if (fallback) {
        ready->statusReadsInFallback++;
    } else if (!afterIntb && !afterIntbRecheck) {
        ready->statusReadsPollDiag++;
        ready->statusReadsBeforeIntb++;
    } else if (afterIntbRecheck) {
        ready->statusReadsAfterIntbRecheck++;
        ready->statusVerifyUs += (uint32_t)statusUs;
    } else if (afterIntb) {
        ready->statusReadsAfterIntb++;
        ready->statusVerifyUs += (uint32_t)statusUs;
    }
    if (err != ESP_OK) {
        ready->kind = SENSORARRAY_FDC_READY_I2C_ERROR;
        ready->readyResult = FDC_READY_I2C_ERROR;
        ready->i2cError = true;
        ready->diagnostic = "i2c_status_error";
        const char *statusTag = (afterIntb || afterIntbRecheck) ? "STI" :
            hardTimeout ? "STH" :
            fallback ? "STT" :
            preWaitCheck ? "STE" :
            "STD";
        printf("%s,d=%s,r=%u,e=%lu,i=%lu,st=%04X,u=%X,dr=%u,req=%X,k=I2C,ib0=%d,ib1=%d,err=0x%lx\n",
               statusTag,
               sensorarrayMeasureFdcDeviceToken(devId),
               (unsigned)row,
               (unsigned long)epochId,
               (unsigned long)ready->pollCount,
               ready->statusRaw,
               (unsigned)ready->unreadMask,
               (unsigned)ready->drdy,
               (unsigned)requiredUnreadMask,
               intbBeforeStatus,
               intbAfterStatus,
               (unsigned long)err);
        return err;
    }

    sensorarrayFdcReadyDecoded_t decoded =
        sensorarrayMeasureFdcBuildReadyState(&status, requiredUnreadMask);
    sensorarrayMeasureFdcApplyReadyDecoded(ready, &decoded);
    ready->diagnostic = sensorarrayMeasureFdcReadyDiagnosticName(ready, requiredUnreadMask);
    if (ready->unreadWithoutDataReady) {
        ready->unreadWithoutDrdyCount++;
    }
    if (ready->unreadMask != 0u || ready->dataReady) {
        if (bestPartialSeen) {
            *bestPartialSeen = true;
        }
        if (bestPartial) {
            *bestPartial = decoded;
        }
    }
    const char *statusTag = (afterIntb || afterIntbRecheck) ? "STI" :
        hardTimeout ? "STH" :
        fallback ? "STT" :
        preWaitCheck ? "STE" :
        "STD";
    bool normalIntbConfirmation =
        (afterIntb || afterIntbRecheck) && ready->readyForDataRead;
    if (!normalIntbConfirmation || logNormalSuccess) {
        printf("%s,d=%s,r=%u,e=%lu,i=%lu,st=%04X,u=%X,dr=%u,req=%X,k=%s,ib0=%d,ib1=%d\n",
               statusTag,
               sensorarrayMeasureFdcDeviceToken(devId),
               (unsigned)row,
               (unsigned long)epochId,
               (unsigned long)ready->pollCount,
               ready->statusRaw,
               (unsigned)ready->unreadMask,
               (unsigned)ready->drdy,
               (unsigned)requiredUnreadMask,
               sensorarrayMeasureFdcReadyPollToken(ready, requiredUnreadMask),
               intbBeforeStatus,
               intbAfterStatus);
    }
    return ESP_OK;
}

__attribute__((unused)) esp_err_t sensorarrayMeasureFdcReadStatusForWatchdogDiagOnly(
    sensorarrayFdcDeviceState_t *fdcState,
    sensorarrayFdcDeviceId_t devId,
    uint8_t row,
    uint32_t epochId,
    const char *reason,
    sensorarrayFdcReadyState_t *ready,
    sensorarrayFdcDeviceTiming_t *timing)
{
    if (!fdcState || !fdcState->handle || !ready || devId > SENSORARRAY_FDC_DEV_SECONDARY) {
        return ESP_ERR_INVALID_ARG;
    }

    Fdc2214CapStatus_t status = {0};
    sensorarrayFdcWorkerContext_t *ctx = sensorarrayMeasureFdcWorkerContext(devId);
    int intbBeforeStatus = (ctx && ctx->intbReady) ? gpio_get_level((gpio_num_t)ctx->intbGpio) : -1;
    int64_t statusStartUs = esp_timer_get_time();
    esp_err_t err = Fdc2214CapReadStatus(fdcState->handle, &status);
    uint64_t statusUs = sensorarrayMeasureElapsedUs(statusStartUs);
    int intbAfterStatus = (ctx && ctx->intbReady) ? gpio_get_level((gpio_num_t)ctx->intbGpio) : -1;
    if (timing) {
        timing->statusReadUs += statusUs;
    }
    ready->pollCount++;
    ready->statusReadsWatchdogDiag++;
    ready->intbBeforeStatus = intbBeforeStatus;
    ready->intbAfterStatus = intbAfterStatus;
    if (intbBeforeStatus == 0) {
        ready->statusAckCount++;
    }

    uint8_t unreadMask = (err == ESP_OK) ? sensorarrayMeasureFdcUnreadMaskFromStatus(&status) : 0u;
    bool originalIntbMiss = ready->intbMiss || ready->originalIntbMiss;
    bool originalTimeout = ready->timeout || ready->originalTimeout || ready->err == ESP_ERR_TIMEOUT;
    bool originalWatchdogOnly = ready->watchdogOnly || ready->diagOnly || ready->originalWatchdogOnly;
    esp_err_t originalErr = ready->err;
    bool diagDrdy = err == ESP_OK && status.DataReady;
    bool diagReadable = err == ESP_OK &&
                        diagDrdy &&
                        ((unreadMask & ready->requiredUnreadMask) == ready->requiredUnreadMask);
    if (devId <= SENSORARRAY_FDC_DEV_SECONDARY) {
        s_fdcLastDiagStatusRaw[(uint8_t)devId] = (err == ESP_OK) ? status.Raw : 0u;
        s_fdcLastDiagUnreadMask[(uint8_t)devId] = unreadMask & 0x0Fu;
        s_fdcLastDiagDrdy[(uint8_t)devId] = diagDrdy;
        s_fdcLastDiagStatusOk[(uint8_t)devId] = err == ESP_OK;
        s_fdcLastDiagWasReady[(uint8_t)devId] = diagReadable;
        s_fdcLastDiagEpoch[(uint8_t)devId] = epochId;
        s_fdcLastDiagRow[(uint8_t)devId] = row;
    }
    if (err == ESP_OK) {
        sensorarrayFdcReadyDecoded_t decoded =
            sensorarrayMeasureFdcBuildReadyState(&status, ready->requiredUnreadMask);
        sensorarrayMeasureFdcApplyReadyDecoded(ready, &decoded);
        ready->diagnostic = sensorarrayMeasureFdcReadyDiagnosticName(ready,
                                                                     ready->requiredUnreadMask);
    }
    ready->originalIntbMiss = originalIntbMiss;
    ready->originalWaitMiss = originalIntbMiss;
    ready->originalTimeout = originalTimeout;
    ready->originalWatchdogOnly = originalWatchdogOnly;
    ready->originalErr = originalErr;
    bool recoveredByLevelLow =
        intbBeforeStatus == 0 ||
        ready->levelActiveAfterArm ||
        ready->levelActiveAtWaitReturn;
    if (diagReadable) {
        ready->ready = true;
        ready->readyForDataRead = true;
        ready->readyResult = FDC_READY_OK_DRDY_UNREAD_FULL;
        ready->kind = recoveredByLevelLow ?
            SENSORARRAY_FDC_READY_LEVEL_ACTIVE_FALLBACK :
            (ready->waitBudgetTooShort ?
             SENSORARRAY_FDC_READY_WAIT_BUDGET_TOO_SHORT_STATUS_FALLBACK :
             SENSORARRAY_FDC_READY_STATUS_FALLBACK_AFTER_INTB_MISS);
        ready->err = ESP_OK;
        ready->timeout = false;
        ready->partial = false;
        ready->i2cError = false;
        ready->intbMiss = false;
        ready->watchdogOnly = false;
        ready->diagOnly = false;
        ready->statusFallbackUsed = true;
        ready->acceptedByStatusFallback = true;
        ready->recoveredByStatusReady = true;
        ready->recoveredByLevelLow = recoveredByLevelLow;
        ready->diagnostic = recoveredByLevelLow ?
            "level_low_status_ready_after_wait_return" :
            (ready->waitBudgetTooShort ?
             "wait_budget_too_short_status_ready" :
             "status_ready_after_intb_wait_miss");
    } else if (err != ESP_OK) {
        ready->err = err;
        ready->i2cError = true;
        ready->kind = SENSORARRAY_FDC_READY_I2C_ERROR;
        ready->readyResult = FDC_READY_HARD_TIMEOUT;
        ready->diagnostic = "watchdog_diag_status_i2c_error";
    }

    printf("RWD_DIAG_STATUS,d=%s,r=%u,e=%lu,st=%04X,u=%X,dr=%u,diagOnly=%u,accepted=%u,acceptedByStatusFallback=%u,reason=%s,ib0=%d,ib1=%d,err=0x%lx,statusFallbackUsed=%u,originalIntbMiss=%u,originalTimeout=%u,recoveredByStatusReady=%u,recoveredByLevelLow=%u,reqMask=%X,diagUnreadMask=%X,diagDrdy=%u\n",
           sensorarrayMeasureFdcDeviceToken(devId),
           (unsigned)row,
           (unsigned long)epochId,
           (err == ESP_OK) ? status.Raw : 0u,
           (unsigned)unreadMask,
           (err == ESP_OK && status.DataReady) ? 1u : 0u,
           ready->diagOnly ? 1u : 0u,
           diagReadable ? 1u : 0u,
           ready->acceptedByStatusFallback ? 1u : 0u,
           diagReadable ? ready->diagnostic : (reason ? reason : "watchdog_diag_only"),
           intbBeforeStatus,
           intbAfterStatus,
           (unsigned long)err,
           ready->statusFallbackUsed ? 1u : 0u,
           ready->originalIntbMiss ? 1u : 0u,
           ready->originalTimeout ? 1u : 0u,
           ready->recoveredByStatusReady ? 1u : 0u,
           ready->recoveredByLevelLow ? 1u : 0u,
           (unsigned)ready->requiredUnreadMask,
           (unsigned)unreadMask,
           diagDrdy ? 1u : 0u);
    return ready->err;
}

bool sensorarrayMeasureFdcReadyIsUnreadFullNoDrdy(const sensorarrayFdcReadyState_t *ready,
                                                         uint8_t requiredUnreadMask)
{
    if (!ready) {
        return false;
    }
    requiredUnreadMask &= 0x0Fu;
    if (requiredUnreadMask == 0u) {
        requiredUnreadMask = SENSORARRAY_FDC_AUTOSCAN_READY_UNREAD_MASK;
    }
    return !ready->readyForDataRead &&
           ready->unreadWithoutDataReady &&
           (ready->unreadMask & requiredUnreadMask) == requiredUnreadMask &&
           !ready->dataReady;
}

bool sensorarrayMeasureFdcClassifyUnreadFullNoDrdySoft(sensorarrayFdcDeviceId_t devId,
                                                             uint8_t row,
                                                             uint32_t epochId,
                                                             const char *source,
                                                             sensorarrayFdcReadyState_t *ready)
{
    if (!ready ||
        !sensorarrayMeasureFdcReadyIsUnreadFullNoDrdy(ready, ready->requiredUnreadMask)) {
        return false;
    }

    ready->guardMarginUs = (uint32_t)CONFIG_SENSORARRAY_FDC_READY_GUARD_US;
    ready->guardDeadlineUs = sensorarrayMeasureFdcReadyGuardDeadlineUs(ready->estimatedRoundUs);
    bool beforeGuard = ready->waitUs < ready->guardDeadlineUs;
    if (beforeGuard) {
        ready->ready = false;
        ready->readyForDataRead = false;
        ready->kind = SENSORARRAY_FDC_READY_NONE;
        ready->readyResult = FDC_READY_INTERNAL_STATE_ERROR;
        ready->err = ESP_ERR_INVALID_STATE;
        ready->diagnostic = "internal_wait_state_leak_before_guard";
        printf("RWS,d=%s,r=%u,e=%lu,result=internal_state_error,elapsed=%lu,guardDeadline=%lu,action=do_not_emit_read4,source=%s\n",
               sensorarrayMeasureFdcDeviceToken(devId),
               (unsigned)row,
               (unsigned long)epochId,
               (unsigned long)ready->waitUs,
               (unsigned long)ready->guardDeadlineUs,
               source ? source : "unknown");
        return true;
    }
    ready->ready = false;
    ready->readyForDataRead = false;
    ready->timeout = false;
    ready->partial = false;
    ready->err = ESP_OK;
    ready->kind = SENSORARRAY_FDC_READY_STALE_UNREAD_NO_DRDY;
    ready->readyResult = FDC_READY_STALE_UNREAD_NO_DRDY;
    ready->diagnostic = "after_estimated_round_no_drdy";

    printf("%s,row=%u,dev=%s,epoch=%lu,st=%04X,u=%X,dr=%u,elapsed=%lu,est=%lu,guard=%lu,hardRemain=%lu,iw=%lu,pc=%lu,intb=%d,decision=%s,source=%s\n",
           "FDC_READY_STALE_UNREAD",
           (unsigned)row,
           sensorarrayMeasureFdcDeviceName(devId),
           (unsigned long)epochId,
           ready->statusRaw,
           (unsigned)ready->unreadMask,
           (unsigned)ready->drdy,
           (unsigned long)ready->waitUs,
           (unsigned long)ready->estimatedRoundUs,
           (unsigned long)ready->guardMarginUs,
           (unsigned long)ready->hardDeadlineRemainingUs,
           (unsigned long)ready->actualIntbWaitUs,
           (unsigned long)ready->pollCount,
           ready->finalIntbLevel,
           "drain_discard",
           source ? source : "unknown");
    return true;
}

esp_err_t sensorarrayFdcWaitDeviceReady(sensorarrayState_t *state,
                                               sensorarrayFdcDeviceId_t devId,
                                               uint8_t row,
                                               uint32_t epochId,
                                               uint32_t frameSeq,
                                               uint8_t requiredUnreadMask,
                                               uint32_t timeoutUs,
                                               uint32_t estimatedRoundUs,
                                               uint64_t rowDeviceDeadlineUs,
                                               bool allowDirectData,
                                               sensorarrayFdcReadyState_t *ready,
                                               sensorarrayFdcDeviceTiming_t *timing)
{
    if (!state || !ready || devId > SENSORARRAY_FDC_DEV_SECONDARY) {
        return ESP_ERR_INVALID_ARG;
    }
    sensorarrayFdcDeviceState_t *fdcState = sensorarrayMeasureGetFdcState(state, devId);
    if (!fdcState || !fdcState->handle) {
        return ESP_ERR_INVALID_STATE;
    }
    if (timeoutUs == 0u) {
        timeoutUs = 1u;
    }
    requiredUnreadMask &= 0x0Fu;
    if (requiredUnreadMask == 0u) {
        requiredUnreadMask = SENSORARRAY_FDC_AUTOSCAN_READY_UNREAD_MASK;
    }

    *ready = (sensorarrayFdcReadyState_t){
        .kind = SENSORARRAY_FDC_READY_NONE,
        .err = ESP_ERR_TIMEOUT,
        .initialIntbLevel = -1,
        .finalIntbLevel = -1,
        .intbBeforeStatus = -1,
        .intbAfterStatus = -1,
        .diagnostic = "not_started",
        .waitReturnReason = "not_started",
        .requiredUnreadMask = requiredUnreadMask,
        .estimatedRoundUs = estimatedRoundUs,
        .rowDeviceHardUs = sensorarrayMeasureFdcRowDeviceWatchdogHardTimeoutUs(),
        .hardDeadlineRemainingUs = sensorarrayMeasureFdcRemainingDeadlineUs(rowDeviceDeadlineUs),
        .hardDeadlineRemainingBeforeWaitUs = sensorarrayMeasureFdcRemainingDeadlineUs(rowDeviceDeadlineUs),
        .waitSource = "not_computed",
        .estKind = sensorarrayMeasureFdcEstimateKindName(requiredUnreadMask),
        .rawLevelAfterArm = -1,
        .rawLevelAtWaitReturn = -1,
        .activeLowConfigured = true,
        .originalErr = ESP_ERR_TIMEOUT,
    };
    uint32_t actualIntbWaitUs =
        sensorarrayMeasureFdcComputeActualIntbWaitUs(estimatedRoundUs,
                                                     rowDeviceDeadlineUs,
                                                     ready);

    sensorarrayFdcWorkerContext_t *ctx = sensorarrayMeasureFdcWorkerContext(devId);
    sensorarrayFdcReadyPolicy_t policy = sensorarrayMeasureFdcReadyPolicyForDevice(devId, ctx);
    bool useIntb = policy != SENSORARRAY_FDC_READY_POLICY_POLL_ONLY;
    bool fallbackAllowed = SENSORARRAY_CFG_FDC_POLL_FALLBACK_ENABLED &&
                           (policy == SENSORARRAY_FDC_READY_POLICY_INTB_WITH_POLL_FALLBACK ||
                            policy == SENSORARRAY_FDC_READY_POLICY_POLL_ONLY);
    bool fallbackAttempted = false;
    bool bestPartialSeen = false;
    sensorarrayFdcReadyDecoded_t bestPartial = {0};
    uint32_t edgeStart = 0u;
    int64_t startUs = esp_timer_get_time();
    if (timing) {
        timing->readyBeginUs = (uint64_t)startUs;
    }

    if (ctx && ctx->intbReady) {
        if (useIntb &&
            (ctx->waitTask != xTaskGetCurrentTaskHandle() ||
             ctx->currentEpoch != epochId)) {
            sensorarrayMeasureFdcArmCurrentTaskForIntb(ctx, epochId);
        }
        edgeStart = ctx->edgeCount;
        ready->rawLevelAfterArm = gpio_get_level((gpio_num_t)ctx->intbGpio);
        ready->levelActiveAfterArm = ready->rawLevelAfterArm == 0;
        ready->initialIntbLevel = ready->rawLevelAfterArm;
    }
    if (useIntb &&
        (!ctx || !ctx->intbReady || devId > SENSORARRAY_FDC_DEV_SECONDARY ||
         !s_fdcIntbRuntimeUsable[(uint8_t)devId])) {
        ready->kind = SENSORARRAY_FDC_READY_NONE;
        ready->readyResult = FDC_READY_INTERNAL_STATE_ERROR;
        ready->err = ESP_ERR_INVALID_STATE;
        ready->diagnostic = "intb_strict_level_unavailable";
        ready->waitReturnReason = "internal_state_error";
        ready->waitUs = (uint32_t)sensorarrayMeasureElapsedUs(startUs);
        sensorarrayMeasureFdcUpdateReadyTiming(ready, timing, false);
        printf("RWS,d=%s,r=%u,e=%lu,result=internal_state_error,reason=intb_strict_level_unavailable\n",
               sensorarrayMeasureFdcDeviceToken(devId),
               (unsigned)row,
               (unsigned long)epochId);
        return ready->err;
    }

    bool logReady = CONFIG_SENSORARRAY_FDC_LOG_READY_EVERY_ROW ||
                    sensorarrayMeasureFdcShouldLogNormalFrame(frameSeq);
    if (logReady) {
        printf("RB,d=%s,r=%u,e=%lu,to=%lu,iwPlan=%lu,iwActual=%lu,est=%lu,hard=%lu,hardRemainBeforeWait=%lu,waitClampedByHardDeadline=%u,waitBudgetTooShort=%u,waitSource=%s,estKind=%s,req=%X,rawLevelAfterArm=%d,levelActiveAfterArm=%u,edge=%lu,mode=%s,intb=%u\n",
               sensorarrayMeasureFdcDeviceToken(devId),
               (unsigned)row,
               (unsigned long)epochId,
               (unsigned long)timeoutUs,
               (unsigned long)ready->plannedIntbWaitUs,
               (unsigned long)actualIntbWaitUs,
               (unsigned long)estimatedRoundUs,
               (unsigned long)ready->rowDeviceHardUs,
               (unsigned long)ready->hardDeadlineRemainingBeforeWaitUs,
               ready->waitClampedByHardDeadline ? 1u : 0u,
               ready->waitBudgetTooShort ? 1u : 0u,
               ready->waitSource ? ready->waitSource : "unknown",
               ready->estKind ? ready->estKind : "unknown",
               (unsigned)requiredUnreadMask,
               ready->rawLevelAfterArm,
               ready->levelActiveAfterArm ? 1u : 0u,
               (unsigned long)edgeStart,
               sensorarrayMeasureFdcReadyPolicyName(policy),
               useIntb ? 1u : 0u);
    }

#define SENSORARRAY_FDC_RR_TIMING_FMT ",iwPlan=%lu,iwActual=%lu,est=%lu,hardRemain=%lu,hardRemainBeforeWait=%lu,elapsed=%lu,waitClampedByHardDeadline=%u,waitBudgetTooShort=%u,returnedBeforeEstimatedRound=%u,waitReturnReason=%s,notifyValue=%lu,rawLevelAfterArm=%d,rawLevelAtWaitReturn=%d,activeLowConfigured=%u,levelActiveAfterArm=%u,levelActiveAtWaitReturn=%u"
#define SENSORARRAY_FDC_RR_TIMING_ARGS \
    (unsigned long)ready->plannedIntbWaitUs, \
    (unsigned long)ready->actualIntbWaitUs, \
    (unsigned long)ready->estimatedRoundUs, \
    (unsigned long)sensorarrayMeasureFdcRemainingDeadlineUs(rowDeviceDeadlineUs), \
    (unsigned long)ready->hardDeadlineRemainingBeforeWaitUs, \
    (unsigned long)ready->waitUs, \
    ready->waitClampedByHardDeadline ? 1u : 0u, \
    ready->waitBudgetTooShort ? 1u : 0u, \
    ready->returnedBeforeEstimatedRound ? 1u : 0u, \
    ready->waitReturnReason ? ready->waitReturnReason : "unknown", \
    (unsigned long)ready->notifyValue, \
    ready->rawLevelAfterArm, \
    ready->rawLevelAtWaitReturn, \
    ready->activeLowConfigured ? 1u : 0u, \
    ready->levelActiveAfterArm ? 1u : 0u, \
    ready->levelActiveAtWaitReturn ? 1u : 0u

    if (useIntb && ctx && ctx->intbReady) {
        bool intbSeen = ready->initialIntbLevel == 0;
        if (!intbSeen) {
            if (!SENSORARRAY_CFG_FDC_STATUS_CONFIRM_ENABLED) {
                ready->statusReadsSuppressedBeforeIntb++;
            }
            int64_t intbWaitStartUs = esp_timer_get_time();
            uint32_t notifyCount = 0u;
            bool strictLevel = policy == SENSORARRAY_FDC_READY_POLICY_INTB_STRICT_LEVEL;
            for (;;) {
                uint32_t elapsedUs = (uint32_t)sensorarrayMeasureElapsedUs(intbWaitStartUs);
                uint32_t remainingUs =
                    elapsedUs < actualIntbWaitUs ? (actualIntbWaitUs - elapsedUs) : 0u;
                if (remainingUs == 0u) {
                    break;
                }
                notifyCount += ulTaskNotifyTake(pdTRUE,
                                                sensorarrayMeasureFdcTicksForUs(remainingUs));
                ready->finalIntbLevel = gpio_get_level((gpio_num_t)ctx->intbGpio);
                if (ready->finalIntbLevel == 0 || (!strictLevel && notifyCount != 0u)) {
                    break;
                }
            }
            ready->notifyValue = notifyCount;
            ready->intbWaitUs = (uint32_t)sensorarrayMeasureElapsedUs(intbWaitStartUs);
            ready->finalIntbLevel = gpio_get_level((gpio_num_t)ctx->intbGpio);
            ready->rawLevelAtWaitReturn = ready->finalIntbLevel;
            ready->levelActiveAtWaitReturn = ready->rawLevelAtWaitReturn == 0;
            ready->edgeDelta = ctx->edgeCount >= edgeStart ? (ctx->edgeCount - edgeStart) : 0u;
            ready->hadEdge = (notifyCount != 0u || ready->edgeDelta != 0u) ? 1u : 0u;
            ready->returnedBeforeEstimatedRound =
                estimatedRoundUs != 0u && ready->intbWaitUs < estimatedRoundUs;
            ready->waitReturnReason =
                ready->levelActiveAtWaitReturn ? "intb_level_active" :
                (!strictLevel && notifyCount != 0u) ? "intb_notify" :
                ready->waitBudgetTooShort ? "wait_budget_too_short" :
                ready->waitClampedByHardDeadline ? "hard_deadline_clamp" :
                "timeout";
            intbSeen = ready->finalIntbLevel == 0 ||
                       (!strictLevel && ready->hadEdge);
        } else {
            ready->finalIntbLevel = ready->initialIntbLevel;
            ready->rawLevelAtWaitReturn = ready->initialIntbLevel;
            ready->levelActiveAtWaitReturn = ready->rawLevelAtWaitReturn == 0;
            ready->hadEdge = 1u;
            ready->waitReturnReason = ready->levelActiveAfterArm ?
                "intb_level_active" :
                "intb_notify";
        }
        if (intbSeen) {
            if (timing) {
                timing->intbSeenUs = (uint64_t)esp_timer_get_time();
            }
            bool newEpochEdge = ready->edgeDelta != 0u || ready->notifyValue != 0u;
            if (CONFIG_SENSORARRAY_FDC_INTB_DIRECT_DATA_ENABLE &&
                allowDirectData &&
                newEpochEdge &&
                !ready->levelActiveAfterArm) {
                ready->ready = true;
                ready->dataReady = true;
                ready->unreadFull = true;
                ready->readyForDataRead = true;
                ready->unreadWithoutDataReady = false;
                ready->kind = SENSORARRAY_FDC_READY_EDGE_WAKE;
                ready->readyResult = FDC_READY_OK_INTB_DRDY_UNREAD_FULL;
                ready->statusRaw = 0u;
                ready->unreadMask = requiredUnreadMask;
                ready->drdy = 1u;
                ready->waitUs = (uint32_t)sensorarrayMeasureElapsedUs(startUs);
                ready->diagnostic = "intb_direct_data_candidate";
                ready->timeout = false;
                ready->partial = false;
                ready->err = ESP_OK;
                ready->directDataCandidate = true;
                ready->waitReturnReason = "new_epoch_intb_edge";
                if (timing) {
                    timing->drdyUs = (uint64_t)esp_timer_get_time();
                    timing->directDataReadCount++;
                    timing->statusSavedReadCount++;
                }
                if (ctx) {
                    ctx->freshDrdyCount++;
                    ctx->waitTask = NULL;
                }
                sensorarrayMeasureFdcUpdateReadyTiming(ready, timing, false);
                return ESP_OK;
            }
            int64_t verifyStartUs = esp_timer_get_time();
            if (timing) {
                timing->statusVerifyStartUs = (uint64_t)verifyStartUs;
            }
            esp_err_t statusErr =
                sensorarrayMeasureFdcReadStatusAndAckIntbForReady(fdcState,
                                                                   devId,
                                                                   row,
                                                                   epochId,
                                                                   requiredUnreadMask,
                                                                   true,
                                                                   false,
                                                                   false,
                                                                   false,
                                                                   false,
                                                                   logReady,
                                                                   ready,
                                                                   timing,
                                                                   &bestPartialSeen,
                                                                   &bestPartial);
            if (statusErr != ESP_OK) {
                ready->waitUs = (uint32_t)sensorarrayMeasureElapsedUs(startUs);
                sensorarrayMeasureFdcUpdateReadyTiming(ready, timing, false);
                if (ctx) {
                    ctx->waitTask = NULL;
                }
                sensorarrayMeasureFdcLogReadyCounts(devId, row, epochId, ready);
                return statusErr;
            }
            if (ready->readyForDataRead) {
                ready->ready = true;
                ready->kind = SENSORARRAY_FDC_READY_EDGE_WAKE;
                ready->readyResult = FDC_READY_OK_INTB_DRDY_UNREAD_FULL;
                ready->waitUs = (uint32_t)sensorarrayMeasureElapsedUs(startUs);
                ready->diagnostic = "data_ready_full_unread";
                ready->timeout = false;
                ready->partial = false;
                ready->err = ESP_OK;
                if (timing) {
                    timing->drdyUs = (uint64_t)esp_timer_get_time();
                }
                if (ctx) {
                    ctx->freshDrdyCount++;
                    ctx->waitTask = NULL;
                }
                sensorarrayMeasureFdcUpdateReadyTiming(ready, timing, false);
                if (logReady) {
                    printf("RR,d=%s,r=%u,e=%lu,src=%s,st=%04X,u=%X,dr=%u,wu=%lu,pc=%lu,k=FULL,ack=%lu,ib0=%d,ib1=%d,err=0"
                           SENSORARRAY_FDC_RR_TIMING_FMT "\n",
                           sensorarrayMeasureFdcDeviceToken(devId),
                           (unsigned)row,
                           (unsigned long)epochId,
                           ready->initialIntbLevel == 0 ? "IL" : "IE",
                           ready->statusRaw,
                           (unsigned)ready->unreadMask,
                           (unsigned)ready->drdy,
                           (unsigned long)ready->waitUs,
                           (unsigned long)ready->pollCount,
                           (unsigned long)ready->statusAckCount,
                           ready->intbBeforeStatus,
                           ready->intbAfterStatus,
                           SENSORARRAY_FDC_RR_TIMING_ARGS);
                }
                sensorarrayMeasureFdcLogReadyCounts(devId, row, epochId, ready);
                return ESP_OK;
            }
            if (CONFIG_SENSORARRAY_FDC_INTB_STATUS_CONFIRM_RETRY != 0) {
                uint32_t maxRechecks = 1u;
                uint32_t intervalUs =
                    (uint32_t)CONFIG_SENSORARRAY_FDC_INTB_STATUS_CONFIRM_RETRY_US;
                uint32_t deadlineUs =
                    (uint32_t)CONFIG_SENSORARRAY_FDC_AFTER_INTB_RECHECK_DEADLINE_US;
                int64_t recheckStartUs = esp_timer_get_time();
                for (uint32_t poll = 0u; poll < maxRechecks; ++poll) {
                    if (deadlineUs != 0u &&
                        sensorarrayMeasureElapsedUs(recheckStartUs) >= deadlineUs) {
                        break;
                    }
                    if (intervalUs > 0u) {
                        esp_rom_delay_us(intervalUs);
                    }
                    statusErr =
                        sensorarrayMeasureFdcReadStatusAndAckIntbForReady(fdcState,
                                                                           devId,
                                                                           row,
                                                                           epochId,
                                                                           requiredUnreadMask,
                                                                           false,
                                                                           true,
                                                                           false,
                                                                           false,
                                                                           false,
                                                                           logReady,
                                                                           ready,
                                                                           timing,
                                                                           &bestPartialSeen,
                                                                           &bestPartial);
                    if (statusErr != ESP_OK) {
                        ready->waitUs = (uint32_t)sensorarrayMeasureElapsedUs(startUs);
                        sensorarrayMeasureFdcUpdateReadyTiming(ready, timing, false);
                        if (ctx) {
                            ctx->waitTask = NULL;
                        }
                        sensorarrayMeasureFdcLogReadyCounts(devId, row, epochId, ready);
                        return statusErr;
                    }
                    if (ready->readyForDataRead) {
                        ready->ready = true;
                        ready->kind = SENSORARRAY_FDC_READY_AFTER_INTB_RECHECK_FULL;
                        ready->readyResult = FDC_READY_OK_INTB_DRDY_UNREAD_FULL;
                        ready->waitUs = (uint32_t)sensorarrayMeasureElapsedUs(startUs);
                        ready->diagnostic = "after_intb_recheck_full";
                        ready->timeout = false;
                        ready->partial = false;
                        ready->err = ESP_OK;
                        if (timing) {
                            timing->drdyUs = (uint64_t)esp_timer_get_time();
                        }
                        if (ctx) {
                            ctx->freshDrdyCount++;
                            ctx->waitTask = NULL;
                        }
                        sensorarrayMeasureFdcUpdateReadyTiming(ready, timing, false);
                        if (logReady) {
                            printf("RR,d=%s,r=%u,e=%lu,src=AR,st=%04X,u=%X,dr=%u,wu=%lu,pc=%lu,k=AFTER_INTB_RECHECK_FULL,ack=%lu,ib0=%d,ib1=%d,err=0"
                                   SENSORARRAY_FDC_RR_TIMING_FMT "\n",
                                   sensorarrayMeasureFdcDeviceToken(devId),
                                   (unsigned)row,
                                   (unsigned long)epochId,
                                   ready->statusRaw,
                                   (unsigned)ready->unreadMask,
                                   (unsigned)ready->drdy,
                                   (unsigned long)ready->waitUs,
                                   (unsigned long)ready->pollCount,
                                   (unsigned long)ready->statusAckCount,
                                   ready->intbBeforeStatus,
                                   ready->intbAfterStatus,
                                   SENSORARRAY_FDC_RR_TIMING_ARGS);
                        }
                        sensorarrayMeasureFdcLogReadyCounts(devId, row, epochId, ready);
                        return ESP_OK;
                    }
                }
                if (policy == SENSORARRAY_FDC_READY_POLICY_INTB_STRICT_LEVEL ||
                    policy == SENSORARRAY_FDC_READY_POLICY_INTB_THEN_STATUS) {
                    ready->kind = SENSORARRAY_FDC_READY_DRDY_NOT_CLOSED_AFTER_INTB;
                    ready->readyResult = FDC_READY_INTB_ACTIVE_STATUS_MISMATCH;
                    ready->waitUs = (uint32_t)sensorarrayMeasureElapsedUs(startUs);
                    ready->timeout = false;
                    ready->err = ESP_ERR_INVALID_RESPONSE;
                    ready->diagnostic = "intb_active_status_mismatch";
                    if (ctx) {
                        ctx->waitTask = NULL;
                    }
                    sensorarrayMeasureFdcUpdateReadyTiming(ready, timing, false);
                    printf("STM,d=%s,r=%u,e=%lu,st=%04X,u=%X,dr=%u,wu=%lu,pc=%lu,k=INTB_ACTIVE_STATUS_MISMATCH,ack=%lu,ib0=%d,ib1=%d,err=0x%lx"
                           SENSORARRAY_FDC_RR_TIMING_FMT "\n",
                           sensorarrayMeasureFdcDeviceToken(devId),
                           (unsigned)row,
                           (unsigned long)epochId,
                           ready->statusRaw,
                           (unsigned)ready->unreadMask,
                           (unsigned)ready->drdy,
                           (unsigned long)ready->waitUs,
                           (unsigned long)ready->pollCount,
                           (unsigned long)ready->statusAckCount,
                           ready->intbBeforeStatus,
                           ready->intbAfterStatus,
                           (unsigned long)ready->err,
                           SENSORARRAY_FDC_RR_TIMING_ARGS);
                    sensorarrayMeasureFdcLogReadyCounts(devId, row, epochId, ready);
                    return ready->err;
                }
            } else if (policy == SENSORARRAY_FDC_READY_POLICY_INTB_STRICT_LEVEL ||
                       policy == SENSORARRAY_FDC_READY_POLICY_INTB_THEN_STATUS) {
                ready->kind = SENSORARRAY_FDC_READY_STATUS_INCONSISTENT;
                ready->readyResult = FDC_READY_INTB_ACTIVE_STATUS_MISMATCH;
                ready->waitUs = (uint32_t)sensorarrayMeasureElapsedUs(startUs);
                ready->timeout = false;
                ready->err = ESP_ERR_INVALID_RESPONSE;
                ready->diagnostic = "intb_active_status_mismatch";
                if (ctx) {
                    ctx->waitTask = NULL;
                }
                sensorarrayMeasureFdcUpdateReadyTiming(ready, timing, false);
                printf("STM,d=%s,r=%u,e=%lu,src=%s,st=%04X,u=%X,dr=%u,wu=%lu,pc=%lu,k=INTB_ACTIVE_STATUS_MISMATCH,ack=%lu,ib0=%d,ib1=%d,err=0x%lx"
                       SENSORARRAY_FDC_RR_TIMING_FMT "\n",
                       sensorarrayMeasureFdcDeviceToken(devId),
                       (unsigned)row,
                       (unsigned long)epochId,
                       ready->initialIntbLevel == 0 ? "IL" : "IE",
                       ready->statusRaw,
                       (unsigned)ready->unreadMask,
                       (unsigned)ready->drdy,
                       (unsigned long)ready->waitUs,
                       (unsigned long)ready->pollCount,
                       (unsigned long)ready->statusAckCount,
                       ready->intbBeforeStatus,
                       ready->intbAfterStatus,
                       (unsigned long)ready->err,
                       SENSORARRAY_FDC_RR_TIMING_ARGS);
                sensorarrayMeasureFdcLogReadyCounts(devId, row, epochId, ready);
                return ready->err;
            }
        } else {
            ready->diagnostic = (policy == SENSORARRAY_FDC_READY_POLICY_INTB_WITH_POLL_FALLBACK) ?
                "intb_hint_timeout_poll_fallback" :
                "intb_wait_miss";
            if (ctx) {
                ctx->timeoutCount++;
            }
            ready->waitUs = (uint32_t)sensorarrayMeasureElapsedUs(startUs);
            ready->hardDeadlineRemainingUs =
                sensorarrayMeasureFdcRemainingDeadlineUs(rowDeviceDeadlineUs);
            if (policy == SENSORARRAY_FDC_READY_POLICY_INTB_STRICT_LEVEL ||
                policy == SENSORARRAY_FDC_READY_POLICY_INTB_THEN_STATUS) {
                uint32_t edgeAfter = ctx ? ctx->edgeCount : edgeStart;
                int levelAfterWait = ready->finalIntbLevel;
                uint32_t epochSeen = ctx ? ctx->lastEpochSeen : 0u;
                uint32_t currentEpoch = ctx ? ctx->currentEpoch : epochId;
                int64_t lastEdgeUs = ctx ? ctx->lastEdgeUs : 0;
                bool waitTaskSet = ctx && ctx->waitTask != NULL;
                bool guardReached = ready->waitUs >= ready->guardDeadlineUs;
                bool hardStatusAlreadyRead = !guardReached;
                bool readGuardStatus =
                    guardReached && CONFIG_SENSORARRAY_FDC_STATUS_AFTER_TIMEOUT_FALLBACK;
                esp_err_t statusErr = ESP_OK;
                if (readGuardStatus || hardStatusAlreadyRead) {
                    statusErr =
                        sensorarrayMeasureFdcReadStatusAndAckIntbForReady(fdcState,
                                                                           devId,
                                                                           row,
                                                                           epochId,
                                                                           requiredUnreadMask,
                                                                           false,
                                                                           false,
                                                                           readGuardStatus,
                                                                           hardStatusAlreadyRead,
                                                                           false,
                                                                           logReady,
                                                                           ready,
                                                                           timing,
                                                                           &bestPartialSeen,
                                                                           &bestPartial);
                }
                if (statusErr != ESP_OK) {
                    ready->waitReturnReason = "true_timeout_not_ready";
                    ready->waitUs = (uint32_t)sensorarrayMeasureElapsedUs(startUs);
                    sensorarrayMeasureFdcUpdateReadyTiming(ready, timing, false);
                    if (ctx) {
                        ctx->waitTask = NULL;
                    }
                    sensorarrayMeasureFdcLogReadyCounts(devId, row, epochId, ready);
                    return statusErr;
                }
                if (ready->readyForDataRead) {
                    ready->ready = true;
                    ready->kind = SENSORARRAY_FDC_READY_STATUS_READY_AFTER_TIMEOUT;
                    ready->readyResult = FDC_READY_OK_STATUS_READY_AFTER_TIMEOUT;
                    ready->waitReturnReason = "status_ready_after_timeout";
                    ready->diagnostic = "status_ready_after_timeout";
                    ready->err = ESP_OK;
                    ready->timeout = false;
                    ready->partial = false;
                    ready->i2cError = false;
                    ready->intbMiss = false;
                    ready->watchdogOnly = false;
                    ready->diagOnly = false;
                    ready->statusFallbackUsed = true;
                    ready->acceptedByStatusFallback = true;
                    ready->recoveredByStatusReady = true;
                    ready->lateStatusReady = true;
                    ready->originalIntbMiss = true;
                    ready->originalWaitMiss = true;
                    ready->originalTimeout = true;
                    ready->originalErr = ESP_ERR_TIMEOUT;
                    if (timing) {
                        timing->drdyUs = (uint64_t)esp_timer_get_time();
                    }
                    if (ctx) {
                        ctx->waitTask = NULL;
                    }
                    sensorarrayMeasureFdcUpdateReadyTiming(ready, timing, false);
                    printf("RR,d=%s,r=%u,e=%lu,src=STT,st=%04X,u=%X,dr=%u,wu=%lu,pc=%lu,k=STATUS_READY_AFTER_TIMEOUT,ack=%lu,ib0=%d,ib1=%d,err=0,statusFallbackUsed=%u,originalIntbMiss=%u,originalTimeout=%u"
                           SENSORARRAY_FDC_RR_TIMING_FMT "\n",
                           sensorarrayMeasureFdcDeviceToken(devId),
                           (unsigned)row,
                           (unsigned long)epochId,
                           ready->statusRaw,
                           (unsigned)ready->unreadMask,
                           (unsigned)ready->drdy,
                           (unsigned long)ready->waitUs,
                           (unsigned long)ready->pollCount,
                           (unsigned long)ready->statusAckCount,
                           ready->intbBeforeStatus,
                           ready->intbAfterStatus,
                           ready->statusFallbackUsed ? 1u : 0u,
                           ready->originalIntbMiss ? 1u : 0u,
                           ready->originalTimeout ? 1u : 0u,
                           SENSORARRAY_FDC_RR_TIMING_ARGS);
                    sensorarrayMeasureFdcLogReadyCounts(devId, row, epochId, ready);
                    return ESP_OK;
                }
                ready->kind = SENSORARRAY_FDC_READY_INTB_TIMEOUT;
                ready->readyResult = FDC_READY_NOT_READY_AFTER_GUARD;
                ready->diagnostic = "not_ready_after_guard_waiting_hard_deadline";
                ready->waitReturnReason = "wait_until_hard_deadline";
                printf("RWT,d=%s,r=%u,e=%lu,stage=after_guard,st=%04X,u=%X,dr=%u,elapsed=%lu,hardRemain=%lu,action=wait_intb_only_until_hard_deadline\n",
                       sensorarrayMeasureFdcDeviceToken(devId),
                       (unsigned)row,
                       (unsigned long)epochId,
                       ready->statusRaw,
                       (unsigned)ready->unreadMask,
                       (unsigned)ready->drdy,
                       (unsigned long)ready->waitUs,
                       (unsigned long)ready->hardDeadlineRemainingUs);

                uint32_t hardRemainUs =
                    sensorarrayMeasureFdcRemainingDeadlineUs(rowDeviceDeadlineUs);
                if (hardRemainUs > 0u) {
                    int64_t hardWaitStartUs = esp_timer_get_time();
                    uint32_t notifyCount = 0u;
                    for (;;) {
                        uint32_t remainingUs =
                            sensorarrayMeasureFdcRemainingDeadlineUs(rowDeviceDeadlineUs);
                        if (remainingUs == 0u) {
                            break;
                        }
                        notifyCount += ulTaskNotifyTake(
                            pdTRUE,
                            sensorarrayMeasureFdcTicksForUs(remainingUs));
                        ready->finalIntbLevel =
                            gpio_get_level((gpio_num_t)ctx->intbGpio);
                        if (ready->finalIntbLevel == 0) {
                            break;
                        }
                    }
                    ready->notifyValue += notifyCount;
                    ready->intbWaitUs +=
                        (uint32_t)sensorarrayMeasureElapsedUs(hardWaitStartUs);
                    ready->finalIntbLevel = gpio_get_level((gpio_num_t)ctx->intbGpio);
                    ready->rawLevelAtWaitReturn = ready->finalIntbLevel;
                    ready->levelActiveAtWaitReturn = ready->rawLevelAtWaitReturn == 0;
                    ready->edgeDelta =
                        ctx->edgeCount >= edgeStart ? (ctx->edgeCount - edgeStart) : 0u;
                    ready->hadEdge =
                        (notifyCount != 0u || ready->edgeDelta != 0u) ? 1u : 0u;
                }
                if (ready->levelActiveAtWaitReturn) {
                    statusErr =
                        sensorarrayMeasureFdcReadStatusAndAckIntbForReady(fdcState,
                                                                           devId,
                                                                           row,
                                                                           epochId,
                                                                           requiredUnreadMask,
                                                                           true,
                                                                           false,
                                                                           false,
                                                                           false,
                                                                           false,
                                                                           logReady,
                                                                           ready,
                                                                           timing,
                                                                           &bestPartialSeen,
                                                                           &bestPartial);
                    if (statusErr == ESP_OK && !ready->readyForDataRead &&
                        CONFIG_SENSORARRAY_FDC_INTB_STATUS_CONFIRM_RETRY != 0) {
                        uint32_t retryUs =
                            (uint32_t)CONFIG_SENSORARRAY_FDC_INTB_STATUS_CONFIRM_RETRY_US;
                        if (retryUs != 0u) {
                            esp_rom_delay_us(retryUs);
                        }
                        statusErr =
                            sensorarrayMeasureFdcReadStatusAndAckIntbForReady(fdcState,
                                                                               devId,
                                                                               row,
                                                                               epochId,
                                                                               requiredUnreadMask,
                                                                               false,
                                                                               true,
                                                                               false,
                                                                               false,
                                                                               false,
                                                                               logReady,
                                                                               ready,
                                                                               timing,
                                                                               &bestPartialSeen,
                                                                               &bestPartial);
                    }
                    ready->waitUs = (uint32_t)sensorarrayMeasureElapsedUs(startUs);
                    if (statusErr != ESP_OK) {
                        if (ctx) {
                            ctx->waitTask = NULL;
                        }
                        sensorarrayMeasureFdcUpdateReadyTiming(ready, timing, false);
                        sensorarrayMeasureFdcLogReadyCounts(devId, row, epochId, ready);
                        return statusErr;
                    }
                    if (ready->readyForDataRead) {
                        ready->ready = true;
                        ready->kind = SENSORARRAY_FDC_READY_EDGE_WAKE;
                        ready->readyResult = FDC_READY_OK_INTB_DRDY_UNREAD_FULL;
                        ready->diagnostic = "data_ready_full_unread_after_guard";
                        ready->waitReturnReason = "intb_level_active_after_guard";
                        ready->timeout = false;
                        ready->partial = false;
                        ready->err = ESP_OK;
                        if (timing) {
                            timing->drdyUs = (uint64_t)esp_timer_get_time();
                        }
                        if (ctx) {
                            ctx->freshDrdyCount++;
                            ctx->waitTask = NULL;
                        }
                        sensorarrayMeasureFdcUpdateReadyTiming(ready, timing, false);
                        if (logReady) {
                            printf("RR,d=%s,r=%u,e=%lu,src=IE,st=%04X,u=%X,dr=%u,wu=%lu,pc=%lu,k=FULL_AFTER_GUARD,waitReturnReason=%s,rawLevelAtWaitReturn=%d,activeLowConfigured=1,levelActiveAtWaitReturn=%u\n",
                                   sensorarrayMeasureFdcDeviceToken(devId),
                                   (unsigned)row,
                                   (unsigned long)epochId,
                                   ready->statusRaw,
                                   (unsigned)ready->unreadMask,
                                   (unsigned)ready->drdy,
                                   (unsigned long)ready->waitUs,
                                   (unsigned long)ready->pollCount,
                                   ready->waitReturnReason,
                                   ready->rawLevelAtWaitReturn,
                                   ready->levelActiveAtWaitReturn ? 1u : 0u);
                        }
                        sensorarrayMeasureFdcLogReadyCounts(devId, row, epochId, ready);
                        return ESP_OK;
                    }

                    ready->kind = SENSORARRAY_FDC_READY_STATUS_INCONSISTENT;
                    ready->readyResult = FDC_READY_INTB_ACTIVE_STATUS_MISMATCH;
                    ready->diagnostic = "intb_active_status_mismatch";
                    ready->err = ESP_ERR_INVALID_RESPONSE;
                    ready->timeout = false;
                    if (ctx) {
                        ctx->waitTask = NULL;
                    }
                    sensorarrayMeasureFdcUpdateReadyTiming(ready, timing, false);
                    printf("STM,d=%s,r=%u,e=%lu,st=%04X,u=%X,dr=%u,pc=%lu,k=INTB_ACTIVE_STATUS_MISMATCH_AFTER_GUARD,err=0x%lx\n",
                           sensorarrayMeasureFdcDeviceToken(devId),
                           (unsigned)row,
                           (unsigned long)epochId,
                           ready->statusRaw,
                           (unsigned)ready->unreadMask,
                           (unsigned)ready->drdy,
                           (unsigned long)ready->pollCount,
                           (unsigned long)ready->err);
                    sensorarrayMeasureFdcLogReadyCounts(devId, row, epochId, ready);
                    return ready->err;
                }

                if (!hardStatusAlreadyRead) {
                    statusErr =
                        sensorarrayMeasureFdcReadStatusAndAckIntbForReady(fdcState,
                                                                           devId,
                                                                           row,
                                                                           epochId,
                                                                           requiredUnreadMask,
                                                                           false,
                                                                           false,
                                                                           false,
                                                                           true,
                                                                           false,
                                                                           logReady,
                                                                           ready,
                                                                           timing,
                                                                           &bestPartialSeen,
                                                                           &bestPartial);
                    if (statusErr != ESP_OK) {
                        ready->waitUs = (uint32_t)sensorarrayMeasureElapsedUs(startUs);
                        if (ctx) {
                            ctx->waitTask = NULL;
                        }
                        sensorarrayMeasureFdcUpdateReadyTiming(ready, timing, false);
                        sensorarrayMeasureFdcLogReadyCounts(devId, row, epochId, ready);
                        return statusErr;
                    }
                }
                if (ready->readyForDataRead) {
                    ready->ready = true;
                    ready->kind = SENSORARRAY_FDC_READY_STATUS_READY_AFTER_TIMEOUT;
                    ready->readyResult = FDC_READY_OK_STATUS_READY_AFTER_TIMEOUT;
                    ready->waitUs = (uint32_t)sensorarrayMeasureElapsedUs(startUs);
                    ready->waitReturnReason = "status_ready_at_hard_timeout";
                    ready->diagnostic = "status_ready_after_timeout";
                    ready->err = ESP_OK;
                    ready->timeout = false;
                    ready->partial = false;
                    ready->statusFallbackUsed = true;
                    ready->acceptedByStatusFallback = true;
                    ready->recoveredByStatusReady = true;
                    ready->lateStatusReady = true;
                    ready->originalIntbMiss = true;
                    ready->originalWaitMiss = true;
                    ready->originalTimeout = true;
                    ready->originalErr = ESP_ERR_TIMEOUT;
                    if (timing) {
                        timing->drdyUs = (uint64_t)esp_timer_get_time();
                    }
                    if (ctx) {
                        ctx->waitTask = NULL;
                    }
                    sensorarrayMeasureFdcUpdateReadyTiming(ready, timing, false);
                    printf("RR,d=%s,r=%u,e=%lu,src=STT,st=%04X,u=%X,dr=%u,wu=%lu,pc=%lu,k=STATUS_READY_AT_HARD_TIMEOUT,statusFallbackUsed=1\n",
                           sensorarrayMeasureFdcDeviceToken(devId),
                           (unsigned)row,
                           (unsigned long)epochId,
                           ready->statusRaw,
                           (unsigned)ready->unreadMask,
                           (unsigned)ready->drdy,
                           (unsigned long)ready->waitUs,
                           (unsigned long)ready->pollCount);
                    sensorarrayMeasureFdcLogReadyCounts(devId, row, epochId, ready);
                    return ESP_OK;
                }
                ready->kind = SENSORARRAY_FDC_READY_INTB_TIMEOUT;
                ready->readyResult = FDC_READY_HARD_TIMEOUT_NO_DRDY;
                ready->ready = false;
                ready->readyForDataRead = false;
                ready->timeout = true;
                ready->err = ESP_ERR_TIMEOUT;
                ready->intbMiss = true;
                ready->watchdogOnly = true;
                ready->diagOnly = true;
                ready->trueTimeoutNotReady = true;
                ready->originalIntbMiss = true;
                ready->originalWaitMiss = true;
                ready->originalTimeout = true;
                ready->originalErr = ESP_ERR_TIMEOUT;
                ready->waitUs = (uint32_t)sensorarrayMeasureElapsedUs(startUs);
                ready->waitReturnReason = "hard_timeout_no_drdy";
                ready->diagnostic = "hard_timeout_no_drdy";
                printf("IM,d=%s,r=%u,e=%lu,stage=wait_miss,armLevel=%d,levelBeforeWait=%d,levelAfterWait=%d,edgeBefore=%lu,edgeAfter=%lu,lastEdgeUs=%lld,waitTaskSet=%u,epochSeen=%lu,currentEpoch=%lu,action=true_timeout_not_ready,iwPlan=%lu,iwActual=%lu,est=%lu,hardRemainBeforeWait=%lu,waitBudgetTooShort=%u,waitClampedByHardDeadline=%u,returnedBeforeEstimatedRound=%u,waitReturnReason=%s,notifyValue=%lu,rawLevelAfterArm=%d,rawLevelAtWaitReturn=%d,activeLowConfigured=%u,levelActiveAfterArm=%u,levelActiveAtWaitReturn=%u\n",
                       sensorarrayMeasureFdcDeviceToken(devId),
                       (unsigned)row,
                       (unsigned long)epochId,
                       ready->initialIntbLevel,
                       ready->initialIntbLevel,
                       levelAfterWait,
                       (unsigned long)edgeStart,
                       (unsigned long)edgeAfter,
                       (long long)lastEdgeUs,
                       waitTaskSet ? 1u : 0u,
                       (unsigned long)epochSeen,
                       (unsigned long)currentEpoch,
                       (unsigned long)ready->plannedIntbWaitUs,
                       (unsigned long)ready->actualIntbWaitUs,
                       (unsigned long)ready->estimatedRoundUs,
                       (unsigned long)ready->hardDeadlineRemainingBeforeWaitUs,
                       ready->waitBudgetTooShort ? 1u : 0u,
                       ready->waitClampedByHardDeadline ? 1u : 0u,
                       ready->returnedBeforeEstimatedRound ? 1u : 0u,
                       ready->waitReturnReason ? ready->waitReturnReason : "unknown",
                       (unsigned long)ready->notifyValue,
                       ready->rawLevelAfterArm,
                       ready->rawLevelAtWaitReturn,
                       ready->activeLowConfigured ? 1u : 0u,
                       ready->levelActiveAfterArm ? 1u : 0u,
                       ready->levelActiveAtWaitReturn ? 1u : 0u);
                if (ctx) {
                    ctx->waitTask = NULL;
                }
                sensorarrayMeasureFdcUpdateReadyTiming(ready, timing, false);
                printf("RR,d=%s,r=%u,e=%lu,src=IT,st=%04X,u=%X,dr=%u,wu=%lu,pc=%lu,k=true_timeout_not_ready,ack=%lu,ib0=%d,ib1=%d,err=0x%lx,statusFallbackUsed=%u,originalIntbMiss=%u,originalTimeout=%u,recoveredByLevelLow=%u"
                       SENSORARRAY_FDC_RR_TIMING_FMT "\n",
                       sensorarrayMeasureFdcDeviceToken(devId),
                       (unsigned)row,
                       (unsigned long)epochId,
                       ready->statusRaw,
                       (unsigned)ready->unreadMask,
                       (unsigned)ready->drdy,
                       (unsigned long)ready->waitUs,
                       (unsigned long)ready->pollCount,
                       (unsigned long)ready->statusAckCount,
                       ready->intbBeforeStatus,
                       ready->intbAfterStatus,
                       (unsigned long)ready->err,
                       ready->statusFallbackUsed ? 1u : 0u,
                       ready->originalIntbMiss ? 1u : 0u,
                       ready->originalTimeout ? 1u : 0u,
                       ready->recoveredByLevelLow ? 1u : 0u,
                       SENSORARRAY_FDC_RR_TIMING_ARGS);
                sensorarrayMeasureFdcLogReadyCounts(devId, row, epochId, ready);
                return ready->err;
            }
        }
    }

    if (fallbackAllowed) {
        fallbackAttempted = useIntb;
        int64_t fallbackStartUs = esp_timer_get_time();
        uint32_t maxPolls = (uint32_t)CONFIG_SENSORARRAY_FDC_READY_MAX_POLLS_AFTER_UNREAD_BEFORE_DRDY;
        if (maxPolls == 0u) {
            maxPolls = (uint32_t)CONFIG_SENSORARRAY_FDC_POLL_FALLBACK_MAX_POLLS;
        }
        uint32_t pollIntervalUs = (uint32_t)CONFIG_SENSORARRAY_FDC_READY_POLL_INTERVAL_US;
        for (uint32_t poll = 0u; poll < maxPolls; ++poll) {
            if (poll != 0u && pollIntervalUs > 0u) {
                esp_rom_delay_us(pollIntervalUs);
            }
            esp_err_t statusErr =
                sensorarrayMeasureFdcReadStatusAndAckIntbForReady(fdcState,
                                                                   devId,
                                                                   row,
                                                                   epochId,
                                                                   requiredUnreadMask,
                                                                   false,
                                                                   false,
                                                                   useIntb,
                                                                   false,
                                                                   false,
                                                                   logReady,
                                                                   ready,
                                                                   timing,
                                                                   &bestPartialSeen,
                                                                   &bestPartial);
            if (statusErr != ESP_OK) {
                ready->waitUs = (uint32_t)sensorarrayMeasureElapsedUs(startUs);
                ready->pollFallbackUs += (uint32_t)sensorarrayMeasureElapsedUs(fallbackStartUs);
                sensorarrayMeasureFdcUpdateReadyTiming(ready, timing, fallbackAttempted);
                if (ctx) {
                    ctx->waitTask = NULL;
                }
                sensorarrayMeasureFdcLogReadyCounts(devId, row, epochId, ready);
                return statusErr;
            }
            if (ready->readyForDataRead) {
                bool recoveredAfterWaitDrdy = ready->unreadWithoutDrdyCount != 0u;
                ready->ready = true;
                ready->kind = recoveredAfterWaitDrdy ?
                    SENSORARRAY_FDC_READY_POLL_RECOVERED_AFTER_UNREAD_BEFORE_DRDY :
                    SENSORARRAY_FDC_READY_POLL_FULL;
                ready->readyResult = recoveredAfterWaitDrdy ?
                    FDC_READY_RECOVERED_AFTER_RETRY :
                    FDC_READY_OK_DRDY_UNREAD_FULL;
                ready->waitUs = (uint32_t)sensorarrayMeasureElapsedUs(startUs);
                ready->pollFallbackUs += (uint32_t)sensorarrayMeasureElapsedUs(fallbackStartUs);
                ready->diagnostic = recoveredAfterWaitDrdy ?
                    "recovered_after_wait_drdy" :
                    (useIntb ? "poll_fallback_ready" : "data_ready_full_unread");
                ready->timeout = false;
                ready->partial = false;
                ready->err = ESP_OK;
                if (timing) {
                    timing->drdyUs = (uint64_t)esp_timer_get_time();
                }
                if (ctx) {
                    if (useIntb) {
                        ctx->fallbackPollCount++;
                    }
                    ctx->freshDrdyCount++;
                    ctx->waitTask = NULL;
                }
                sensorarrayMeasureFdcUpdateReadyTiming(ready, timing, fallbackAttempted);
                printf("RR,d=%s,r=%u,e=%lu,src=%s,st=%04X,u=%X,dr=%u,wu=%lu,pc=%lu,k=%s,ack=%lu,ib0=%d,ib1=%d,err=0"
                       SENSORARRAY_FDC_RR_TIMING_FMT "\n",
                       sensorarrayMeasureFdcDeviceToken(devId),
                       (unsigned)row,
                       (unsigned long)epochId,
                       useIntb ? "FB" : "PD",
                       ready->statusRaw,
                       (unsigned)ready->unreadMask,
                       (unsigned)ready->drdy,
                       (unsigned long)ready->waitUs,
                       (unsigned long)ready->pollCount,
                       recoveredAfterWaitDrdy ? "RECOVERED_NO_DRDY" :
                       (useIntb ? "FB_FULL" : "FULL"),
                       (unsigned long)ready->statusAckCount,
                       ready->intbBeforeStatus,
                       ready->intbAfterStatus,
                       SENSORARRAY_FDC_RR_TIMING_ARGS);
                sensorarrayMeasureFdcLogReadyCounts(devId, row, epochId, ready);
                return ESP_OK;
            }
        }
        ready->pollFallbackUs += (uint32_t)sensorarrayMeasureElapsedUs(fallbackStartUs);
    }

    if (ctx) {
        ctx->waitTask = NULL;
        ready->finalIntbLevel = ctx->intbReady ? gpio_get_level((gpio_num_t)ctx->intbGpio) : -1;
        ready->rawLevelAtWaitReturn = ready->finalIntbLevel;
        ready->levelActiveAtWaitReturn = ready->rawLevelAtWaitReturn == 0;
        ready->edgeDelta = ctx->edgeCount >= edgeStart ? (ctx->edgeCount - edgeStart) : 0u;
        ready->hadEdge = ready->edgeDelta > 0u ? 1u : 0u;
    }
    ready->waitUs = (uint32_t)sensorarrayMeasureElapsedUs(startUs);
    ready->hardDeadlineRemainingUs = sensorarrayMeasureFdcRemainingDeadlineUs(rowDeviceDeadlineUs);
    ready->returnedBeforeEstimatedRound =
        ready->estimatedRoundUs != 0u && ready->waitUs < ready->estimatedRoundUs;
    if (!ready->waitReturnReason || strcmp(ready->waitReturnReason, "not_started") == 0) {
        ready->waitReturnReason =
            ready->levelActiveAtWaitReturn ? "intb_level_active" :
            ready->waitBudgetTooShort ? "wait_budget_too_short" :
            ready->waitClampedByHardDeadline ? "hard_deadline_clamp" :
            "timeout";
    }
    if (sensorarrayMeasureFdcReadyIsUnreadFullNoDrdy(ready, requiredUnreadMask)) {
        if (sensorarrayMeasureFdcClassifyUnreadFullNoDrdySoft(devId,
                                                              row,
                                                              epochId,
                                                              "fallback_timeout",
                                                              ready)) {
            sensorarrayMeasureFdcUpdateReadyTiming(ready, timing, fallbackAttempted);
            sensorarrayMeasureFdcLogReadyCounts(devId, row, epochId, ready);
            return ESP_OK;
        }
    }
    ready->timeoutCount++;
    ready->timeout = true;
    if (bestPartialSeen) {
        ready->partial = true;
        ready->kind = SENSORARRAY_FDC_READY_TIMEOUT_PARTIAL;
        ready->readyResult = FDC_READY_HARD_TIMEOUT;
        sensorarrayMeasureFdcApplyReadyDecoded(ready, &bestPartial);
    } else {
        ready->kind = SENSORARRAY_FDC_READY_TIMEOUT_NONE;
        ready->readyResult = FDC_READY_HARD_TIMEOUT;
    }
    ready->ready = false;
    ready->readyForDataRead = false;
    ready->err = ESP_ERR_TIMEOUT;
    ready->readyResult = FDC_READY_HARD_TIMEOUT;
    ready->trueTimeoutNotReady = true;
    ready->waitReturnReason = "true_timeout_not_ready";
    ready->diagnostic = ready->unreadWithoutDataReady ?
        "timeout_wait_drdy" :
        (useIntb ?
         (fallbackAttempted ? "poll_fallback_timeout" : "intb_timeout") :
         "poll_fallback_timeout");
    sensorarrayMeasureFdcUpdateReadyTiming(ready, timing, fallbackAttempted);
    printf("RR,d=%s,r=%u,e=%lu,src=%s,st=%04X,u=%X,dr=%u,wu=%lu,pc=%lu,k=%s,ack=%lu,ib0=%d,ib1=%d,err=0x%lx"
           SENSORARRAY_FDC_RR_TIMING_FMT "\n",
           sensorarrayMeasureFdcDeviceToken(devId),
           (unsigned)row,
           (unsigned long)epochId,
           useIntb ? (fallbackAttempted ? "FB" : "IT") : "PD",
           ready->statusRaw,
           (unsigned)ready->unreadMask,
           (unsigned)ready->drdy,
           (unsigned long)ready->waitUs,
           (unsigned long)ready->pollCount,
           ready->diagnostic ? ready->diagnostic : "timeout",
           (unsigned long)ready->statusAckCount,
           ready->intbBeforeStatus,
           ready->intbAfterStatus,
           (unsigned long)ESP_ERR_TIMEOUT,
           SENSORARRAY_FDC_RR_TIMING_ARGS);
    sensorarrayMeasureFdcLogReadyCounts(devId, row, epochId, ready);
#undef SENSORARRAY_FDC_RR_TIMING_FMT
#undef SENSORARRAY_FDC_RR_TIMING_ARGS
    return ESP_ERR_TIMEOUT;
}

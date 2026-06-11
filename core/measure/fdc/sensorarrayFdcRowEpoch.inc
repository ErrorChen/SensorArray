// Split from core/measure/sensorarrayMeasure.c to keep FDC matrix internals isolated.
static bool sensorarrayMeasureFdcDeviceReadyForIo(const sensorarrayFdcDeviceState_t *fdcState)
{
    return fdcState && fdcState->ready && fdcState->handle && fdcState->i2cCtx;
}

static void sensorarrayMeasureLogFdcSecondaryUnavailableOnce(void)
{
    if (s_fdcSecondaryUnavailableLogged) {
        return;
    }
    s_fdcSecondaryUnavailableLogged = true;
    printf("FDC_BUS_WARN,secondaryReady=0,action=fill_d5_d8_minus_one\n");
}

static esp_err_t sensorarrayMeasureFdcSetSleepMode(sensorarrayFdcDeviceState_t *fdcState,
                                                   bool enable,
                                                   sensorarrayFdcDeviceTiming_t *timing)
{
    if (!fdcState || !fdcState->handle) {
        return ESP_ERR_INVALID_ARG;
    }

    uint16_t baseConfig = sensorarrayMeasureFdcConfigBaseWithoutSleep(fdcState);
    int64_t startUs = esp_timer_get_time();
    esp_err_t err = enable ?
        Fdc2214CapEnterSleep(fdcState->handle, baseConfig) :
        Fdc2214CapExitSleep(fdcState->handle, baseConfig);
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

static const char *sensorarrayMeasureFdcRrSequenceName(uint8_t rrSequence)
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

static esp_err_t sensorarrayMeasureFdcVerifySleepExit(sensorarrayState_t *state,
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

static __attribute__((unused)) const char *sensorarrayMeasureFdcReadyKindName(sensorarrayFdcReadyKind_t kind)
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

static bool sensorarrayMeasureFdcShouldLogNormalFrame(uint32_t frameSeq)
{
    if (CONFIG_SENSORARRAY_FDC_VERBOSE_SCAN_LOG) {
        return true;
    }
    uint32_t period = (uint32_t)CONFIG_SENSORARRAY_FDC_SAMPLE_DEVICE_LOG_EVERY_N_FRAMES;
    return period != 0u && frameSeq != 0u && (frameSeq % period) == 0u;
}

static bool sensorarrayMeasureFdcShouldLogNormalPollSuccess(uint32_t frameSeq)
{
    return CONFIG_SENSORARRAY_FDC_LOG_NORMAL_POLL_SUCCESS ||
           sensorarrayMeasureFdcShouldLogNormalFrame(frameSeq);
}

static bool sensorarrayMeasureFdcRead4IsFullSuccess(const sensorarrayFdcDeviceRead4Result_t *read4)
{
    return sensorarrayFdcRead4IsDataCompleteGood(read4,
                                                 SENSORARRAY_FDC_AUTOSCAN_READY_UNREAD_MASK);
}

static bool sensorarrayMeasureFdcWorkerResultIsGood(const sensorarrayFdcWorkerResult_t *result,
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

static void sensorarrayMeasureFdcFrameTrackerNoteRead(sensorarrayFdcFrameReadTracker_t *tracker,
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

static uint32_t sensorarrayMeasureClampU32(uint32_t value, uint32_t minValue, uint32_t maxValue)
{
    if (value < minValue) {
        return minValue;
    }
    if (value > maxValue) {
        return maxValue;
    }
    return value;
}

static uint32_t sensorarrayMeasureFdcCeilDivU64(uint64_t numerator, uint64_t denominator)
{
    if (denominator == 0u) {
        return 0u;
    }
    return (uint32_t)((numerator + denominator - 1u) / denominator);
}

static uint32_t sensorarrayMeasureFdcEstimateAutoscanReadyTimeoutUsWithSnapshot(
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

static uint32_t sensorarrayMeasureFdcEstimateAutoscanReadyTimeoutUsFromConfigs(
    const sensorarrayFdcRuntimeChannelConfig_t configs[4],
    uint8_t requiredUnreadMask,
    uint32_t *outEstimatedRoundUs)
{
    return sensorarrayMeasureFdcEstimateAutoscanReadyTimeoutUsWithSnapshot(configs,
                                                                          requiredUnreadMask,
                                                                          outEstimatedRoundUs,
                                                                          NULL);
}

static uint32_t sensorarrayMeasureFdcEstimateAppliedRowReadyTimeoutUs(
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

static const char *sensorarrayMeasureFdcReadyDiagnosticName(const sensorarrayFdcReadyState_t *ready,
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

static void sensorarrayMeasureFdcUpdateReadyTiming(const sensorarrayFdcReadyState_t *ready,
                                                   sensorarrayFdcDeviceTiming_t *timing,
                                                   bool fallbackAttempted)
{
    if (!ready || !timing) {
        return;
    }

    timing->readyPollCount += ready->pollCount;
    timing->unreadWithoutDrdyCount += ready->unreadWithoutDrdyCount;
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

static const char *sensorarrayMeasureFdcDeviceToken(sensorarrayFdcDeviceId_t devId)
{
    return devId == SENSORARRAY_FDC_DEV_SECONDARY ? "s" : "p";
}

static sensorarrayFdcReadyPolicy_t sensorarrayMeasureFdcConfiguredReadyPolicy(void)
{
#if CONFIG_SENSORARRAY_FDC_READY_POLICY_POLL_ONLY
    return SENSORARRAY_FDC_READY_POLICY_POLL_ONLY;
#elif CONFIG_SENSORARRAY_FDC_READY_POLICY_INTB_THEN_STATUS
    return SENSORARRAY_FDC_READY_POLICY_INTB_THEN_STATUS;
#elif CONFIG_SENSORARRAY_FDC_READY_POLICY_INTB_STRICT_LEVEL
    return SENSORARRAY_FDC_READY_POLICY_INTB_STRICT_LEVEL;
#else
    return SENSORARRAY_FDC_READY_POLICY_INTB_WITH_POLL_FALLBACK;
#endif
}

static sensorarrayFdcReadyPolicy_t sensorarrayMeasureFdcReadyPolicyForDevice(
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

static const char *sensorarrayMeasureFdcReadyPolicyName(sensorarrayFdcReadyPolicy_t policy)
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

static const char *sensorarrayMeasureFdcWatchdogReasonName(sensorarrayFdcWatchdogReason_t reason)
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

static uint32_t sensorarrayMeasureFdcRowDeviceWatchdogHardTimeoutUs(void)
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

static esp_err_t sensorarrayFdcHandleRowDeviceWatchdog(
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

static TickType_t sensorarrayMeasureFdcTicksForUs(uint32_t waitUs)
{
    uint32_t waitMs = (waitUs + 999u) / 1000u;
    if (waitMs == 0u) {
        waitMs = 1u;
    }
    TickType_t ticks = pdMS_TO_TICKS(waitMs);
    return ticks == 0 ? 1 : ticks;
}

static uint32_t sensorarrayMeasureFdcRemainingDeadlineUs(uint64_t deadlineUs)
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

static BaseType_t sensorarrayMeasureFdcTakeUntilDeadline(SemaphoreHandle_t sem,
                                                         uint64_t deadlineUs,
                                                         uint32_t fallbackWaitUs)
{
    if (!sem) {
        return pdFALSE;
    }
    uint32_t waitUs = sensorarrayMeasureFdcRemainingDeadlineUs(deadlineUs);
    if (waitUs == UINT32_MAX) {
        waitUs = fallbackWaitUs;
    }
    if (waitUs == 0u) {
        return xSemaphoreTake(sem, 0);
    }
    return xSemaphoreTake(sem, sensorarrayMeasureFdcTicksForUs(waitUs));
}

static const char *sensorarrayMeasureFdcEstimateKindName(uint8_t requiredUnreadMask)
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

static uint32_t sensorarrayMeasureFdcAddSaturateU32(uint32_t a, uint32_t b)
{
    uint64_t value = (uint64_t)a + b;
    return value > UINT32_MAX ? UINT32_MAX : (uint32_t)value;
}

static uint32_t sensorarrayMeasureFdcComputeActualIntbWaitUs(uint32_t estimatedRoundUs,
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

static void sensorarrayMeasureFdcDrainCurrentTaskNotify(void)
{
    (void)xTaskNotifyStateClear(xTaskGetCurrentTaskHandle());
    while (ulTaskNotifyTake(pdTRUE, 0) != 0u) {
    }
}

static void sensorarrayMeasureFdcArmCurrentTaskForIntb(sensorarrayFdcWorkerContext_t *ctx,
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

static const char *sensorarrayMeasureFdcReadyPollToken(const sensorarrayFdcReadyState_t *ready,
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

static void sensorarrayMeasureFdcLogReadyCounts(sensorarrayFdcDeviceId_t devId,
                                                uint8_t row,
                                                uint32_t epochId,
                                                const sensorarrayFdcReadyState_t *ready)
{
    if (!ready) {
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

static esp_err_t sensorarrayMeasureFdcReadStatusAndAckIntbForReady(sensorarrayFdcDeviceState_t *fdcState,
                                                                   sensorarrayFdcDeviceId_t devId,
                                                                   uint8_t row,
                                                                   uint32_t epochId,
                                                                   uint8_t requiredUnreadMask,
                                                                   bool afterIntb,
                                                                   bool afterIntbRecheck,
                                                                   bool fallback,
                                                                   bool hardTimeout,
                                                                   bool preWaitCheck,
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
    return ESP_OK;
}

static __attribute__((unused)) esp_err_t sensorarrayMeasureFdcReadStatusForWatchdogDiagOnly(
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

static bool sensorarrayMeasureFdcReadyIsUnreadFullNoDrdy(const sensorarrayFdcReadyState_t *ready,
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

static bool sensorarrayMeasureFdcClassifyUnreadFullNoDrdySoft(sensorarrayFdcDeviceId_t devId,
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

static esp_err_t sensorarrayFdcWaitDeviceReady(sensorarrayState_t *state,
                                               sensorarrayFdcDeviceId_t devId,
                                               uint8_t row,
                                               uint32_t epochId,
                                               uint32_t frameSeq,
                                               uint8_t requiredUnreadMask,
                                               uint32_t timeoutUs,
                                               uint32_t estimatedRoundUs,
                                               uint64_t rowDeviceDeadlineUs,
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
    bool fallbackAllowed =
        !CONFIG_SENSORARRAY_FDC_DISABLE_READY_STATUS_POLL &&
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
            if (CONFIG_SENSORARRAY_FDC_SUPPRESS_STATUS_READ_BEFORE_INTB) {
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

static esp_err_t sensorarrayMeasureFdcDeviceLevelResync(sensorarrayState_t *state,
                                                        uint8_t row,
                                                        uint32_t epochId,
                                                        uint32_t frameSeq,
                                                        sensorarrayFdcDeviceId_t devId,
                                                        const sensorarrayFdcReadyState_t *readyBefore,
                                                        sensorarrayFdcReadyState_t *readyAfter,
                                                        sensorarrayFdcDeviceTiming_t *timing)
{
    if (!state || !readyAfter || devId > SENSORARRAY_FDC_DEV_SECONDARY) {
        return ESP_ERR_INVALID_ARG;
    }
    sensorarrayFdcDeviceState_t *fdcState = sensorarrayMeasureGetFdcState(state, devId);
    if (!fdcState || !fdcState->handle) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t firstErr = ESP_OK;
    uint16_t idMfg = 0u;
    uint16_t idDev = 0u;
    uint16_t configBefore = 0u;
    uint16_t muxBefore = 0u;
    uint16_t errorConfigBefore = 0u;
    uint16_t statusBefore = readyBefore ? readyBefore->statusRaw : 0u;

    esp_err_t err = Fdc2214CapReadId(fdcState->handle, &idMfg, &idDev);
    sensorarrayMeasureRecordFirstErr(err, &firstErr);
    err = Fdc2214CapReadRawRegisters(fdcState->handle,
                                     SENSORARRAY_FDC_REG_CONFIG,
                                     &configBefore);
    sensorarrayMeasureRecordFirstErr(err, &firstErr);
    err = Fdc2214CapReadRawRegisters(fdcState->handle,
                                     SENSORARRAY_FDC_REG_MUX_CONFIG,
                                     &muxBefore);
    sensorarrayMeasureRecordFirstErr(err, &firstErr);
    err = Fdc2214CapReadRawRegisters(fdcState->handle,
                                     SENSORARRAY_FDC_REG_STATUS_CONFIG,
                                     &errorConfigBefore);
    sensorarrayMeasureRecordFirstErr(err, &firstErr);

    if (firstErr == ESP_OK) {
        err = sensorarrayMeasureFdcSetSleepMode(fdcState, true, timing);
        sensorarrayMeasureRecordFirstErr(err, &firstErr);
    }
    if (firstErr == ESP_OK) {
        err = sensorarrayMeasureApplyFdcCachedRowConfig(state,
                                                        row,
                                                        devId,
                                                        "device_resync_no_unread",
                                                        true,
                                                        timing);
        sensorarrayMeasureRecordFirstErr(err, &firstErr);
    }
    if (firstErr == ESP_OK) {
    err = sensorarrayMeasureFdcSetSleepMode(fdcState, false, timing);
    sensorarrayMeasureRecordFirstErr(err, &firstErr);
    if (firstErr == ESP_OK) {
        err = sensorarrayMeasureFdcVerifySleepExit(state, row, epochId, devId, timing);
        sensorarrayMeasureRecordFirstErr(err, &firstErr);
    }
    }
    if (firstErr == ESP_OK) {
        err = Fdc2214CapClearStatus(fdcState->handle);
        sensorarrayMeasureRecordFirstErr(err, &firstErr);
    }
    if (firstErr == ESP_OK) {
        int64_t waitStartUs = esp_timer_get_time();
        uint64_t rowDeviceDeadlineUs =
            (uint64_t)waitStartUs + sensorarrayMeasureFdcRowDeviceWatchdogHardTimeoutUs();
        uint32_t estimatedRoundUs = 0u;
        uint32_t timeoutUs = sensorarrayMeasureFdcEstimateAppliedRowReadyTimeoutUs(
            state,
            devId,
            SENSORARRAY_FDC_AUTOSCAN_READY_UNREAD_MASK,
            &estimatedRoundUs);
        err = sensorarrayFdcWaitDeviceReady(state,
                                            devId,
                                            row,
                                            epochId,
                                            frameSeq,
                                            SENSORARRAY_FDC_AUTOSCAN_READY_UNREAD_MASK,
                                            timeoutUs,
                                            estimatedRoundUs,
                                            rowDeviceDeadlineUs,
                                            readyAfter,
                                            timing);
        uint64_t waitUs = sensorarrayMeasureElapsedUs(waitStartUs);
        if (timing) {
            timing->waitReadyUs += waitUs;
        }
        sensorarrayMeasureRecordFirstErr(err, &firstErr);
    }

    bool idOk = idMfg == SENSORARRAY_FDC_EXPECTED_MANUFACTURER_ID &&
                idDev == SENSORARRAY_FDC_EXPECTED_DEVICE_ID;
    bool ok = firstErr == ESP_OK && readyAfter->readyForDataRead;
    printf("FDC_DEVICE_RESYNC,dev=%s,reason=ready_epoch_fault,row=%u,epoch=%lu,statusBefore=0x%04X,configBefore=0x%04X,muxBefore=0x%04X,errorConfigBefore=0x%04X,idMfg=0x%04X,idDev=0x%04X,idOk=%u,statusAfter=0x%04X,unreadAfter=0x%X,drdyAfter=%u,readyForDataRead=%u,ok=%u,err=0x%lx,errName=%s\n",
           sensorarrayMeasureFdcDeviceName(devId),
           (unsigned)row,
           (unsigned long)epochId,
           statusBefore,
           configBefore,
           muxBefore,
           errorConfigBefore,
           idMfg,
           idDev,
           idOk ? 1u : 0u,
           readyAfter->statusRaw,
           (unsigned)(readyAfter->unreadMask & 0x0Fu),
           readyAfter->dataReady ? 1u : 0u,
           readyAfter->readyForDataRead ? 1u : 0u,
           ok ? 1u : 0u,
           (unsigned long)firstErr,
           sensorarrayMeasureEspErrName(firstErr));
    return firstErr;
}

static esp_err_t sensorarrayMeasureFdcPrepareDeviceWhileSleeping(sensorarrayState_t *state,
                                                                 uint8_t row,
                                                                 uint32_t epochId,
                                                                 uint32_t frameSeq,
                                                                 sensorarrayFdcDeviceId_t devId,
                                                                 sensorarrayFdcRuntimeChannelConfig_t outConfigs[4],
                                                                 sensorarrayFdcDeviceTiming_t *timing)
{
    if (!state || !outConfigs || devId > SENSORARRAY_FDC_DEV_SECONDARY) {
        return ESP_ERR_INVALID_ARG;
    }
    bool logNormal = sensorarrayMeasureFdcShouldLogNormalFrame(frameSeq);
    if (logNormal) {
        printf("FDC_ROW_EPOCH,stage=apply_cache,row=%u,epoch=%lu,dev=%s,force=0,reason=matrix_row_epoch_sleep\n",
               (unsigned)row,
               (unsigned long)epochId,
               sensorarrayMeasureFdcDeviceName(devId));
    }
    if (timing) {
        timing->applyStartUs = (uint64_t)esp_timer_get_time();
    }
    esp_err_t err = sensorarrayMeasureApplyFdcCachedRowConfig(state,
                                                              row,
                                                              devId,
                                                              "matrix_row_epoch_sleep",
                                                              false,
                                                              timing);
    if (err == ESP_OK) {
        sensorarrayMeasureRuntimeConfigsFromApplied(state, devId, outConfigs);
        const sensorarrayFdcAppliedRowConfig_t *applied =
            &state->fdcAppliedRow[(uint8_t)devId];
        bool profileTooSlow =
            applied->profileSnapshot.autoscanRoundUs >
            (uint32_t)CONFIG_SENSORARRAY_FDC_PROFILE_TOO_SLOW_WARN_US;
        if (logNormal || profileTooSlow) {
            printf("FDC_EPOCH,stage=after_cache_apply,row=%u,dev=%s,cfg=0x%04X,mux=0x%04X,statusConfig=0x%04X,roundUs=%lu,profileTooSlow=%u\n",
                   (unsigned)row,
                   sensorarrayMeasureFdcDeviceName(devId),
                   applied->configBaseWithoutSleepBit,
                   applied->muxConfig,
                   applied->statusConfig,
                   (unsigned long)applied->profileSnapshot.autoscanRoundUs,
                   profileTooSlow ? 1u : 0u);
        }
    }
    if (timing) {
        timing->applyDoneUs = (uint64_t)esp_timer_get_time();
    }
    return err;
}

static esp_err_t sensorarrayMeasureFdcDrainStaleUnread(sensorarrayFdcDeviceState_t *fdcState,
                                                       sensorarrayFdcDeviceId_t devId,
                                                       uint8_t row,
                                                       uint32_t epochId,
                                                       const sensorarrayFdcReadyState_t *ready,
                                                       sensorarrayFdcDeviceTiming_t *timing)
{
    if (!fdcState || !fdcState->handle || !ready) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!CONFIG_SENSORARRAY_FDC_STALE_UNREAD_DRAIN_ENABLE &&
        !CONFIG_SENSORARRAY_FDC_DIAG_READ_UNREAD_FULL_WITHOUT_DRDY) {
        printf("FDC_RESCUE_SUPPRESSED,row=%u,dev=%s,epoch=%lu,reason=stale_unread_no_drdy,policy=drain_disabled,rescueAction=none\n",
               (unsigned)row,
               sensorarrayMeasureFdcDeviceName(devId),
               (unsigned long)epochId);
        return ESP_OK;
    }

    Fdc2214CapFastChannelSample_t discard[4] = {0};
    int64_t drainStartUs = esp_timer_get_time();
    esp_err_t err = Fdc2214CapReadChannelsDataRegsOnlyFast(
        fdcState->handle,
        SENSORARRAY_FDC_AUTOSCAN_READY_UNREAD_MASK,
        discard,
        4u);
    uint32_t drainUs = (uint32_t)sensorarrayMeasureElapsedUs(drainStartUs);

    uint8_t rawNonZeroMask = 0u;
    uint8_t msbErrMask = 0u;
    for (uint8_t ch = 0u; ch < 4u; ++ch) {
        uint8_t bit = (uint8_t)(1u << ch);
        if (discard[ch].raw28 != 0u) {
            rawNonZeroMask |= bit;
        }
        if (discard[ch].errWatchdog || discard[ch].errAmplitude ||
            (discard[ch].errorMask & (FDC2214CAP_FAST_ERROR_WATCHDOG |
                                      FDC2214CAP_FAST_ERROR_AMPLITUDE |
                                      FDC2214CAP_FAST_ERROR_STATUS_FAULT)) != 0u) {
            msbErrMask |= bit;
        }
    }

    if (timing) {
        timing->staleDrainUs += drainUs;
        timing->dataReadUs += drainUs;
        timing->readRawUs += drainUs;
        timing->drainCount++;
        if (err == ESP_OK) {
            timing->staleUnreadDrainCount++;
        } else {
            timing->hardReadyTimeoutCount++;
        }
        if (drainUs > timing->maxI2cReadUs) {
            timing->maxI2cReadUs = drainUs;
        }
    }

    printf("FDC_STALE_UNREAD_DRAIN,row=%u,dev=%s,epoch=%lu,statusBefore=%04X,unreadBefore=%X,drdyBefore=%u,drainMask=%X,readErr=0x%lx,rawNonZeroMask=%X,msbErrMask=%X,drainUs=%lu,statusAfterOptional=skip,controlledStatusSideEffect=1\n",
           (unsigned)row,
           sensorarrayMeasureFdcDeviceName(devId),
           (unsigned long)epochId,
           ready->statusRaw,
           (unsigned)ready->unreadMask,
           ready->dataReady ? 1u : 0u,
           SENSORARRAY_FDC_AUTOSCAN_READY_UNREAD_MASK,
           (unsigned long)err,
           (unsigned)rawNonZeroMask,
           (unsigned)msbErrMask,
           (unsigned long)drainUs);

    if (CONFIG_SENSORARRAY_FDC_DIAG_READ_UNREAD_FULL_WITHOUT_DRDY) {
        if (rawNonZeroMask == SENSORARRAY_FDC_AUTOSCAN_READY_UNREAD_MASK && msbErrMask == 0u) {
            if (timing) {
                timing->diagUnreadLikelyFreshCount++;
            }
        } else if (timing) {
            timing->diagUnreadLikelyStaleCount++;
        }
        printf("FDC_UNREAD_ONLY_DIAG,row=%u,dev=%s,epoch=%lu,status=%04X,unread=%X,drdy=%u,elapsed=%lu,est=%lu,raw=[%lu,%lu,%lu,%lu],rawNonZeroMask=%X,msbErrMask=%X,comparePolicy=discard_only\n",
               (unsigned)row,
               sensorarrayMeasureFdcDeviceName(devId),
               (unsigned long)epochId,
               ready->statusRaw,
               (unsigned)ready->unreadMask,
               ready->dataReady ? 1u : 0u,
               (unsigned long)ready->waitUs,
               (unsigned long)ready->estimatedRoundUs,
               (unsigned long)discard[0].raw28,
               (unsigned long)discard[1].raw28,
               (unsigned long)discard[2].raw28,
               (unsigned long)discard[3].raw28,
               (unsigned)rawNonZeroMask,
               (unsigned)msbErrMask);
    }

    return err;
}

static esp_err_t sensorarrayMeasureFdcRunDeviceEpochAfterSleep(sensorarrayState_t *state,
                                                               uint8_t row,
                                                               uint32_t epochId,
                                                               uint32_t frameSeq,
                                                               sensorarrayFdcDeviceId_t devId,
                                                               sensorarrayFdcAutoscanSamples_t *outSamples,
                                                               sensorarrayFdcRuntimeChannelConfig_t outConfigs[4],
                                                               sensorarrayFdcReadyState_t *outReady,
                                                               sensorarrayFdcDeviceRead4Result_t *outRead4,
                                                               sensorarrayFdcDeviceTiming_t *timing,
                                                               sensorarrayFdcFrameReadTracker_t *readTracker,
                                                               const char *readReason,
                                                               uint64_t rowDeviceDeadlineOverrideUs)
{
    if (!state || !outSamples || !outConfigs || !outReady || !outRead4 ||
        devId > SENSORARRAY_FDC_DEV_SECONDARY) {
        return ESP_ERR_INVALID_ARG;
    }
    sensorarrayFdcDeviceState_t *fdcState = sensorarrayMeasureGetFdcState(state, devId);
    if (!sensorarrayMeasureFdcDeviceReadyForIo(fdcState)) {
        return ESP_ERR_INVALID_STATE;
    }
    if (timing) {
        timing->row = row;
        timing->deviceId = devId;
    }

    int64_t jobStartUs = esp_timer_get_time();
    uint64_t rowDeviceDeadlineUs =
        rowDeviceDeadlineOverrideUs != 0u ?
        rowDeviceDeadlineOverrideUs :
        (uint64_t)jobStartUs + sensorarrayMeasureFdcRowDeviceWatchdogHardTimeoutUs();
    bool logNormal = sensorarrayMeasureFdcShouldLogNormalFrame(frameSeq);
    sensorarrayFdcWorkerContext_t *workerCtx = sensorarrayMeasureFdcWorkerContext(devId);
    bool preparedBeforeRelease = workerCtx &&
                                 workerCtx->rowConfigPrepared &&
                                 workerCtx->preparedEpoch == epochId;
    esp_err_t err = preparedBeforeRelease ?
        (esp_err_t)workerCtx->preparedErr :
        sensorarrayMeasureFdcPrepareDeviceWhileSleeping(state,
                                                        row,
                                                        epochId,
                                                        frameSeq,
                                                        devId,
                                                        outConfigs,
                                                        timing);
    if (err == ESP_ERR_NOT_FOUND) {
        *outReady = (sensorarrayFdcReadyState_t){
            .kind = SENSORARRAY_FDC_READY_NONE,
            .err = ESP_ERR_NOT_FOUND,
            .diagnostic = "cache_missing",
        };
        sensorarrayMeasureMarkFdcNoFreshSamples(outSamples, outReady, false);
        sensorarrayMeasureBuildFdcRead4Result(row,
                                              epochId,
                                              devId,
                                              outSamples,
                                              outConfigs,
                                              outReady,
                                              ESP_ERR_NOT_FOUND,
                                              0u,
                                              true,
                                              outRead4);
        sensorarrayMeasureFdcFrameTrackerNoteRead(readTracker,
                                                  row,
                                                  epochId,
                                                  devId,
                                                  outRead4,
                                                  readReason);
        if (timing) {
            timing->deviceUs = sensorarrayMeasureElapsedUs(jobStartUs);
            timing->deviceFullInvalidCount++;
        }
        printf("RS,r=%u,e=%lu,d=%s,vm=0,em=F,cm=F,tm=0,action=skip,why=cache_missing\n",
               (unsigned)row,
               (unsigned long)epochId,
               sensorarrayMeasureFdcDeviceName(devId));
        return ESP_OK;
    }
    if (preparedBeforeRelease && err == ESP_OK) {
        sensorarrayMeasureRuntimeConfigsFromApplied(state, devId, outConfigs);
    }
    if (err != ESP_OK) {
        return err;
    }

    int64_t exitStartUs = esp_timer_get_time();
    if (timing) {
        timing->sleepExitStartUs = (uint64_t)exitStartUs;
    }
    if (logNormal) {
        printf("FDC_ROW_EPOCH,stage=sleep_exit,row=%u,epoch=%lu,dev=%s\n",
               (unsigned)row,
               (unsigned long)epochId,
               sensorarrayMeasureFdcDeviceName(devId));
    }
    sensorarrayMeasureFdcArmCurrentTaskForIntb(sensorarrayMeasureFdcWorkerContext(devId),
                                               epochId);
    err = sensorarrayMeasureFdcSetSleepMode(fdcState, false, timing);
    uint64_t exitUs = sensorarrayMeasureElapsedUs(exitStartUs);
    if (timing) {
        timing->sleepExitDoneUs = (uint64_t)esp_timer_get_time();
    }
    if (err != ESP_OK) {
        return err;
    }
    err = sensorarrayMeasureFdcVerifySleepExit(state, row, epochId, devId, timing);
    if (err != ESP_OK) {
        return err;
    }

    int64_t waitStartUs = esp_timer_get_time();
    uint32_t estimatedRoundUs = 0u;
    uint32_t timeoutUs = sensorarrayMeasureFdcEstimateAutoscanReadyTimeoutUsFromConfigs(
        outConfigs,
        SENSORARRAY_FDC_AUTOSCAN_READY_UNREAD_MASK,
        &estimatedRoundUs);
    err = sensorarrayFdcWaitDeviceReady(state,
                                        devId,
                                        row,
                                        epochId,
                                        frameSeq,
                                        SENSORARRAY_FDC_AUTOSCAN_READY_UNREAD_MASK,
                                        timeoutUs,
                                        estimatedRoundUs,
                                        rowDeviceDeadlineUs,
                                        outReady,
                                        timing);
    uint64_t waitUs = sensorarrayMeasureElapsedUs(waitStartUs);
    if (timing) {
        timing->waitReadyUs += waitUs;
    }
    if (outReady->readyResult == FDC_READY_UNREAD_FULL_NO_DRDY_TRANSIENT ||
        outReady->readyResult == FDC_READY_INTERNAL_STATE_ERROR) {
        outReady->readyResult = FDC_READY_INTERNAL_STATE_ERROR;
        outReady->err = ESP_ERR_INVALID_STATE;
        outReady->diagnostic = "internal_wait_state_leak";
        printf("RWS,d=%s,r=%u,e=%lu,result=internal_state_error,action=abort_before_read4\n",
               sensorarrayMeasureFdcDeviceToken(devId),
               (unsigned)row,
               (unsigned long)epochId);
        return ESP_ERR_INVALID_STATE;
    }
    bool normalDataReadAllowed =
        sensorarrayMeasureFdcReadyAllowsNormalDataRead(outReady,
                                                       SENSORARRAY_FDC_AUTOSCAN_READY_UNREAD_MASK);
    uint8_t freshMask = normalDataReadAllowed ? (outReady->unreadMask & 0x0Fu) : 0u;
    if (err == ESP_OK && normalDataReadAllowed) {
        s_fdcNoUnreadConsecutive[(uint8_t)devId] = 0u;
        if (row > 0u && row <= SENSORARRAY_MATRIX_ROWS) {
            uint8_t rowSlot = (uint8_t)(row - 1u);
            uint8_t devSlot = (uint8_t)devId;
            if (s_fdcStaleUnreadConsecutive[devSlot][rowSlot] != 0u) {
                if (timing) {
                    timing->diagUnreadLikelyStaleCount++;
                }
                printf("FDC_STALE_UNREAD_RECOVERY,row=%u,dev=%s,epoch=%lu,previousStale=%u,status=%04X,unread=%X,drdy=%u\n",
                       (unsigned)row,
                       sensorarrayMeasureFdcDeviceName(devId),
                       (unsigned long)epochId,
                       (unsigned)s_fdcStaleUnreadConsecutive[devSlot][rowSlot],
                       outReady->statusRaw,
                       (unsigned)outReady->unreadMask,
                       outReady->dataReady ? 1u : 0u);
            }
            s_fdcSoftReadyMissConsecutive[devSlot][rowSlot] = 0u;
            s_fdcStaleUnreadConsecutive[devSlot][rowSlot] = 0u;
            s_fdcHardReadyTimeoutConsecutive[devSlot][rowSlot] = 0u;
        }
    } else {
        bool noUnreadStatus = outReady->statusRaw == 0u && outReady->unreadMask == 0u &&
                              !outReady->dataReady && !outReady->i2cError;
        if (noUnreadStatus) {
            s_fdcNoUnreadConsecutive[(uint8_t)devId]++;
        } else {
            s_fdcNoUnreadConsecutive[(uint8_t)devId] = 0u;
        }
        bool recentDiagReady =
            devId <= SENSORARRAY_FDC_DEV_SECONDARY &&
            s_fdcLastDiagStatusOk[(uint8_t)devId] &&
            s_fdcLastDiagDrdy[(uint8_t)devId] &&
            s_fdcLastDiagWasReady[(uint8_t)devId] &&
            ((s_fdcLastDiagUnreadMask[(uint8_t)devId] &
              SENSORARRAY_FDC_AUTOSCAN_READY_UNREAD_MASK) ==
             SENSORARRAY_FDC_AUTOSCAN_READY_UNREAD_MASK);
        if (devId == SENSORARRAY_FDC_DEV_SECONDARY &&
            noUnreadStatus &&
            recentDiagReady) {
            printf("FDC_DEVICE_RESYNC_SUPPRESS,reason=diag_status_ready,diagStatus=0x%04X,diagUnread=0x%X,diagDrdy=%u,row=%u,dev=%s,epoch=%lu,noUnreadConsecutive=%lu\n",
                   s_fdcLastDiagStatusRaw[(uint8_t)devId],
                   (unsigned)s_fdcLastDiagUnreadMask[(uint8_t)devId],
                   s_fdcLastDiagDrdy[(uint8_t)devId] ? 1u : 0u,
                   (unsigned)s_fdcLastDiagRow[(uint8_t)devId],
                   sensorarrayMeasureFdcDeviceName(devId),
                   (unsigned long)s_fdcLastDiagEpoch[(uint8_t)devId],
                   (unsigned long)s_fdcNoUnreadConsecutive[(uint8_t)devId]);
            s_fdcNoUnreadConsecutive[(uint8_t)devId] = 0u;
        } else if (devId == SENSORARRAY_FDC_DEV_SECONDARY &&
            noUnreadStatus &&
            s_fdcNoUnreadConsecutive[(uint8_t)devId] >= SENSORARRAY_FDC_NO_UNREAD_RESYNC_THRESHOLD) {
            sensorarrayFdcReadyState_t resyncReady = {0};
            esp_err_t resyncErr = sensorarrayMeasureFdcDeviceLevelResync(state,
                                                                         row,
                                                                         epochId,
                                                                         frameSeq,
                                                                         devId,
                                                                         outReady,
                                                                         &resyncReady,
                                                                         timing);
            s_fdcNoUnreadConsecutive[(uint8_t)devId] = 0u;
            if (resyncErr == ESP_OK &&
                sensorarrayMeasureFdcReadyAllowsNormalDataRead(
                    &resyncReady,
                    SENSORARRAY_FDC_AUTOSCAN_READY_UNREAD_MASK)) {
                *outReady = resyncReady;
                err = ESP_OK;
                normalDataReadAllowed = true;
                freshMask = outReady->unreadMask & 0x0Fu;
            }
        }
    }
    if (err != ESP_OK || !normalDataReadAllowed ||
        freshMask != SENSORARRAY_FDC_AUTOSCAN_READY_UNREAD_MASK) {
        uint8_t rowSlot = (row > 0u && row <= SENSORARRAY_MATRIX_ROWS) ? (uint8_t)(row - 1u) : 0u;
        uint8_t devSlot = (uint8_t)devId;
        bool softReadyMiss = sensorarrayMeasureFdcReadyResultIsSoftInvalid(outReady);
        bool staleUnread = outReady->readyResult == FDC_READY_STALE_UNREAD_NO_DRDY;
        esp_err_t drainErr = ESP_OK;
        const char *classification = softReadyMiss ? (staleUnread ? "stale" : "transient") : "hard";
        const char *rescueAction = "none";

        if (softReadyMiss) {
            if (s_fdcSoftReadyMissConsecutive[devSlot][rowSlot] < UINT8_MAX) {
                s_fdcSoftReadyMissConsecutive[devSlot][rowSlot]++;
            }
            if (staleUnread) {
                if (s_fdcStaleUnreadConsecutive[devSlot][rowSlot] < UINT8_MAX) {
                    s_fdcStaleUnreadConsecutive[devSlot][rowSlot]++;
                }
                drainErr = sensorarrayMeasureFdcDrainStaleUnread(fdcState,
                                                                 devId,
                                                                 row,
                                                                 epochId,
                                                                 outReady,
                                                                 timing);
                rescueAction = (drainErr == ESP_OK) ? "drain_only" : "request_cell_rescue";
                if (drainErr != ESP_OK) {
                    outReady->readyResult = FDC_READY_HARD_TIMEOUT;
                    outReady->kind = SENSORARRAY_FDC_READY_HARD_TIMEOUT;
                    outReady->timeout = true;
                    outReady->err = drainErr;
                    classification = "hard";
                    if (s_fdcHardReadyTimeoutConsecutive[devSlot][rowSlot] < UINT8_MAX) {
                        s_fdcHardReadyTimeoutConsecutive[devSlot][rowSlot]++;
                    }
                    (void)sensorarrayFdcHandleRowDeviceWatchdog(state,
                                                                devId,
                                                                row,
                                                                epochId,
                                                                SENSORARRAY_FDC_WATCHDOG_READ4_I2C_ERROR,
                                                                NULL,
                                                                timing);
                } else {
                    uint32_t staleThreshold =
                        (uint32_t)CONFIG_SENSORARRAY_FDC_STALE_UNREAD_HARD_THRESHOLD;
                    if (staleThreshold == 0u) {
                        staleThreshold = 1u;
                    }
                    if (s_fdcStaleUnreadConsecutive[devSlot][rowSlot] >= staleThreshold) {
                        if (s_fdcHardReadyTimeoutConsecutive[devSlot][rowSlot] < UINT8_MAX) {
                            s_fdcHardReadyTimeoutConsecutive[devSlot][rowSlot]++;
                        }
                        if (timing) {
                            timing->hardReadyTimeoutCount++;
                        }
                        if (CONFIG_SENSORARRAY_FDC_UNREAD_NO_DRDY_RESCUE_ENABLE) {
                            rescueAction = "request_cell_rescue";
                            (void)sensorarrayFdcHandleRowDeviceWatchdog(state,
                                                                        devId,
                                                                        row,
                                                                        epochId,
                                                                        SENSORARRAY_FDC_WATCHDOG_INTB_TIMEOUT,
                                                                        NULL,
                                                                        timing);
                        } else {
                            printf("FDC_RESCUE_SUPPRESSED,row=%u,dev=%s,epoch=%lu,reason=stale_unread_no_drdy,policy=unread_no_drdy_rescue_disabled,consecutiveStale=%u,threshold=%lu\n",
                                   (unsigned)row,
                                   sensorarrayMeasureFdcDeviceName(devId),
                                   (unsigned long)epochId,
                                   (unsigned)s_fdcStaleUnreadConsecutive[devSlot][rowSlot],
                                   (unsigned long)staleThreshold);
                        }
                    }
                }
            }
            printf("RWD,d=%s,r=%u,e=%lu,why=%s,rowBudget=%lu,mul=%lu,hard=%lu,override=%lu,retryMax=%lu,retryActual=%lu,classification=%s,rescueAction=%s,consecutiveSoft=%u,consecutiveStale=%u,consecutiveHard=%u\n",
                   sensorarrayMeasureFdcDeviceToken(devId),
                   (unsigned)row,
                   (unsigned long)epochId,
                   outReady->diagnostic ? outReady->diagnostic : "soft_ready_miss",
                   (unsigned long)(SENSORARRAY_MATRIX_ROWS ?
                       (SENSORARRAY_FDC_TARGET_FRAME_US / SENSORARRAY_MATRIX_ROWS) : 0u),
                   (unsigned long)CONFIG_SENSORARRAY_FDC_ROW_DEVICE_WATCHDOG_MULTIPLIER,
                   (unsigned long)sensorarrayMeasureFdcRowDeviceWatchdogHardTimeoutUs(),
                   (unsigned long)CONFIG_SENSORARRAY_FDC_ROW_DEVICE_WATCHDOG_OVERRIDE_US,
                   (unsigned long)CONFIG_SENSORARRAY_FDC_ROW_DEVICE_RETRY_MAX,
                   timing ? (unsigned long)timing->fallbackAttemptCount : 0ul,
                   classification,
                   rescueAction,
                   (unsigned)s_fdcSoftReadyMissConsecutive[devSlot][rowSlot],
                   (unsigned)s_fdcStaleUnreadConsecutive[devSlot][rowSlot],
                   (unsigned)s_fdcHardReadyTimeoutConsecutive[devSlot][rowSlot]);
        } else {
            sensorarrayFdcWatchdogReason_t watchdogReason =
                (outReady->kind == SENSORARRAY_FDC_READY_INTB_TIMEOUT) ?
                SENSORARRAY_FDC_WATCHDOG_INTB_TIMEOUT :
                (outReady->kind == SENSORARRAY_FDC_READY_DRDY_NOT_CLOSED_AFTER_INTB) ?
                SENSORARRAY_FDC_WATCHDOG_DRDY_NOT_CLOSED_AFTER_INTB :
                (outReady->kind == SENSORARRAY_FDC_READY_STATUS_INCONSISTENT) ?
                SENSORARRAY_FDC_WATCHDOG_STATUS_INCONSISTENT :
                SENSORARRAY_FDC_WATCHDOG_INTB_TIMEOUT;
            if (outReady->readyResult == FDC_READY_RESULT_NONE ||
                outReady->readyResult == FDC_READY_NOT_READY_AFTER_GUARD) {
                outReady->readyResult = FDC_READY_HARD_TIMEOUT_NO_DRDY;
            }
            if (s_fdcHardReadyTimeoutConsecutive[devSlot][rowSlot] < UINT8_MAX) {
                s_fdcHardReadyTimeoutConsecutive[devSlot][rowSlot]++;
            }
            (void)sensorarrayFdcHandleRowDeviceWatchdog(state,
                                                        devId,
                                                        row,
                                                        epochId,
                                                        watchdogReason,
                                                        NULL,
                                                        timing);
        }
        sensorarrayMeasureMarkFdcNoFreshSamples(outSamples, outReady, outReady->i2cError);
        if (softReadyMiss) {
            outSamples->staleUnreadDrain = staleUnread && drainErr == ESP_OK;
        }
        sensorarrayMeasureBuildFdcRead4Result(row,
                                              epochId,
                                              devId,
                                              outSamples,
                                              outConfigs,
                                              outReady,
                                              softReadyMiss && drainErr == ESP_OK ?
                                              ESP_OK :
                                              ((err == ESP_OK) ? ESP_ERR_TIMEOUT : err),
                                              0u,
                                              true,
                                              outRead4);
        sensorarrayMeasureFdcFrameTrackerNoteRead(readTracker,
                                                  row,
                                                  epochId,
                                                  devId,
                                                  outRead4,
                                                  readReason);
        if (timing) {
            timing->deviceUs = sensorarrayMeasureElapsedUs(jobStartUs);
            timing->deviceFullInvalidCount++;
        }
        printf("FB,stage=%s,r=%u,d=%s,st=%04X,u=%X,dr=%u,ready=%u,err=0x%lx\n",
               softReadyMiss ? "stale_unread_drain" :
               outReady->unreadWithoutDataReady ? "wait_drdy_after_unread_full" :
               (outReady->timeout ? "ready_timeout" : "data_not_ready"),
               (unsigned)row,
               sensorarrayMeasureFdcDeviceName(devId),
               outReady->statusRaw,
               (unsigned)outReady->unreadMask,
               outReady->dataReady ? 1u : 0u,
               outReady->readyForDataRead ? 1u : 0u,
               (unsigned long)(softReadyMiss && drainErr == ESP_OK ?
                               ESP_OK :
                               ((err == ESP_OK) ? ESP_ERR_TIMEOUT : err)));
        return ESP_OK;
    }

    int64_t readStartUs = esp_timer_get_time();
    if (timing) {
        timing->readStartUs = (uint64_t)readStartUs;
    }
    err = sensorarrayMeasureReadFdcAutoscan4chMasked(fdcState, freshMask, outReady, outSamples);
    uint64_t readUs = sensorarrayMeasureElapsedUs(readStartUs);
    if (timing) {
        timing->readDoneUs = (uint64_t)esp_timer_get_time();
    }
    if (timing) {
        timing->readRawUs += readUs;
        timing->dataReadUs += readUs;
        if (readUs > timing->maxI2cReadUs) {
            timing->maxI2cReadUs = readUs;
        }
        (void)exitUs;
    }
    sensorarrayMeasureBuildFdcRead4Result(row,
                                          epochId,
                                          devId,
                                          outSamples,
                                          outConfigs,
                                          outReady,
                                          err,
                                          (uint32_t)readUs,
                                          logNormal,
                                          outRead4);
    if (err == ESP_OK &&
        outRead4->validMask4 == 0u &&
        outRead4->zeroAfterDrdyMask4 != 0u) {
        if (timing) {
            timing->fallbackAttemptCount++;
        }
        (void)sensorarrayFdcHandleRowDeviceWatchdog(state,
                                                    devId,
                                                    row,
                                                    epochId,
                                                    SENSORARRAY_FDC_WATCHDOG_ZERO_AFTER_DRDY,
                                                    NULL,
                                                    timing);
        printf("FB,stage=zero_after_drdy,r=%u,d=%s,st=%04X,u=%X,dr=%u,zd=%X,action=retry_once\n",
               (unsigned)row,
               sensorarrayMeasureFdcDeviceName(devId),
               outRead4->status,
               (unsigned)outRead4->unreadMask4,
               (unsigned)outRead4->drdy,
               (unsigned)outRead4->zeroAfterDrdyMask4);

        sensorarrayFdcReadyState_t retryReady = {0};
        sensorarrayFdcAutoscanSamples_t retrySamples = {0};
        sensorarrayFdcDeviceRead4Result_t retryRead4 = {0};
        int64_t retryWaitStartUs = esp_timer_get_time();
        esp_err_t retryErr = sensorarrayFdcWaitDeviceReady(state,
                                                           devId,
                                                           row,
                                                           epochId,
                                                           frameSeq,
                                                           SENSORARRAY_FDC_AUTOSCAN_READY_UNREAD_MASK,
                                                           timeoutUs,
                                                           estimatedRoundUs,
                                                           rowDeviceDeadlineUs,
                                                           &retryReady,
                                                           timing);
        uint64_t retryWaitUs = sensorarrayMeasureElapsedUs(retryWaitStartUs);
        if (timing) {
            timing->waitReadyUs += retryWaitUs;
        }
        if (retryErr == ESP_OK &&
            sensorarrayMeasureFdcReadyAllowsNormalDataRead(
                &retryReady,
                SENSORARRAY_FDC_AUTOSCAN_READY_UNREAD_MASK)) {
            uint8_t retryFreshMask = retryReady.unreadMask & 0x0Fu;
            int64_t retryReadStartUs = esp_timer_get_time();
            if (timing) {
                timing->readStartUs = (uint64_t)retryReadStartUs;
            }
            retryErr = sensorarrayMeasureReadFdcAutoscan4chMasked(fdcState,
                                                                   retryFreshMask,
                                                                   &retryReady,
                                                                   &retrySamples);
            uint64_t retryReadUs = sensorarrayMeasureElapsedUs(retryReadStartUs);
            if (timing) {
                timing->readDoneUs = (uint64_t)esp_timer_get_time();
            }
            if (timing) {
                timing->readRawUs += retryReadUs;
                timing->dataReadUs += retryReadUs;
                if (retryReadUs > timing->maxI2cReadUs) {
                    timing->maxI2cReadUs = retryReadUs;
                }
            }
            sensorarrayMeasureBuildFdcRead4Result(row,
                                                  epochId,
                                                  devId,
                                                  &retrySamples,
                                                  outConfigs,
                                                  &retryReady,
                                                  retryErr,
                                                  (uint32_t)retryReadUs,
                                                  true,
                                                  &retryRead4);
            if (retryErr == ESP_OK && retryRead4.validMask4 != 0u) {
                *outSamples = retrySamples;
                *outReady = retryReady;
                *outRead4 = retryRead4;
                err = retryErr;
                if (timing) {
                    timing->fallbackSuccessCount++;
                    if (retryRead4.validMask4 != SENSORARRAY_FDC_AUTOSCAN_READY_UNREAD_MASK) {
                        timing->fallbackPartialCount++;
                    }
                }
                printf("FB,stage=success_after_retry,r=%u,d=%s,vm=%X,u=%X,st=%04X\n",
                       (unsigned)row,
                       sensorarrayMeasureFdcDeviceName(devId),
                       (unsigned)retryRead4.validMask4,
                       (unsigned)retryRead4.unreadMask4,
                       retryRead4.status);
            } else {
                if (timing) {
                    timing->fallbackFailCount++;
                }
                printf("FB,stage=zero_after_drdy_retry_failed,r=%u,d=%s,vm=%X,zd=%X,err=0x%lx\n",
                       (unsigned)row,
                       sensorarrayMeasureFdcDeviceName(devId),
                       (unsigned)retryRead4.validMask4,
                       (unsigned)retryRead4.zeroAfterDrdyMask4,
                       (unsigned long)retryErr);
            }
        } else {
            if (timing) {
                timing->fallbackFailCount++;
            }
            printf("FB,stage=retry_after_zero_after_drdy_failed,r=%u,d=%s,st=%04X,u=%X,dr=%u,ready=%u,err=0x%lx\n",
                   (unsigned)row,
                   sensorarrayMeasureFdcDeviceName(devId),
                   retryReady.statusRaw,
                   (unsigned)retryReady.unreadMask,
                   retryReady.dataReady ? 1u : 0u,
                   retryReady.readyForDataRead ? 1u : 0u,
                   (unsigned long)retryErr);
        }
    }
    if (timing) {
        timing->deviceUs = sensorarrayMeasureElapsedUs(jobStartUs);
    }
    sensorarrayMeasureFdcFrameTrackerNoteRead(readTracker,
                                              row,
                                              epochId,
                                              devId,
                                              outRead4,
                                              readReason);
    if (timing && outRead4->validMask4 == 0u) {
        timing->deviceFullInvalidCount++;
    }
    const bool dataCompleteGood =
        sensorarrayFdcRead4IsDataCompleteGood(outRead4,
                                              SENSORARRAY_FDC_AUTOSCAN_READY_UNREAD_MASK);
    const bool isFullPollSuccess =
        dataCompleteGood &&
        !outRead4->timeout &&
        !outRead4->partial &&
        (outRead4->readyKind == SENSORARRAY_FDC_READY_EDGE_WAKE ||
         outRead4->readyKind == SENSORARRAY_FDC_READY_AFTER_INTB_RECHECK_FULL ||
         outRead4->readyKind == SENSORARRAY_FDC_READY_POLL_FULL);
    bool fallbackRelevant =
        !isFullPollSuccess &&
        (dataCompleteGood ||
         outRead4->timeout ||
         outRead4->partial ||
         outRead4->readyKind == SENSORARRAY_FDC_READY_TIMEOUT_PARTIAL ||
         outRead4->readErr != ESP_OK ||
         outRead4->validMask4 != SENSORARRAY_FDC_AUTOSCAN_READY_UNREAD_MASK);
    if (outRead4->readErr != ESP_OK || outRead4->i2cError) {
        (void)sensorarrayFdcHandleRowDeviceWatchdog(state,
                                                    devId,
                                                    row,
                                                    epochId,
                                                    SENSORARRAY_FDC_WATCHDOG_READ4_I2C_ERROR,
                                                    NULL,
                                                    timing);
    } else if (outRead4->rawAllZero && outRead4->drdy) {
        (void)sensorarrayFdcHandleRowDeviceWatchdog(state,
                                                    devId,
                                                    row,
                                                    epochId,
                                                    SENSORARRAY_FDC_WATCHDOG_RAW_ALL_ZERO,
                                                    NULL,
                                                    timing);
    }
    if (fallbackRelevant && outRead4->validMask4 != 0u) {
        printf("FB,stage=%s,r=%u,d=%s,vm=%X,u=%X,st=%04X\n",
               dataCompleteGood ? "recovered_full" :
               (outRead4->partial ? "poll_partial" : "recovered_partial"),
               (unsigned)row,
               sensorarrayMeasureFdcDeviceName(devId),
               (unsigned)outRead4->validMask4,
               (unsigned)outRead4->unreadMask4,
               outRead4->status);
    } else if (isFullPollSuccess &&
               sensorarrayMeasureFdcShouldLogNormalPollSuccess(frameSeq)) {
        printf("PS,r=%u,d=%s,vm=%X,u=%X,st=%04X,wu=%lu,ru=%lu\n",
               (unsigned)row,
               sensorarrayMeasureFdcDeviceName(devId),
               (unsigned)outRead4->validMask4,
               (unsigned)outRead4->unreadMask4,
               outRead4->status,
               (unsigned long)outRead4->waitUs,
               (unsigned long)outRead4->readUs);
    } else if (outRead4->validMask4 == 0u) {
        printf("FB,stage=%s,r=%u,d=%s,st=%04X,u=%X,dr=%u,nr=%X,z0=%X,zd=%X,err=0x%lx\n",
               outRead4->zeroBeforeReadyMask4 != 0u ? "zero_raw_without_drdy" :
               (outRead4->zeroAfterDrdyMask4 != 0u ? "zero_raw_after_drdy" : "poll_failed"),
               (unsigned)row,
               sensorarrayMeasureFdcDeviceName(devId),
               outRead4->status,
               (unsigned)outRead4->unreadMask4,
               (unsigned)outRead4->drdy,
               (unsigned)outRead4->notReadyMask4,
               (unsigned)outRead4->zeroBeforeReadyMask4,
               (unsigned)outRead4->zeroAfterDrdyMask4,
               (unsigned long)outRead4->readErr);
    }
    return ESP_OK;
}

static void sensorarrayMeasureFdcWorkerTask(void *arg)
{
    sensorarrayFdcWorkerContext_t *ctx = (sensorarrayFdcWorkerContext_t *)arg;
    if (!ctx) {
        vTaskDelete(NULL);
        return;
    }

    for (;;) {
        sensorarrayFdcWorkerJob_t job = {0};
        if (xQueueReceive(ctx->queue, &job, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        if (!job.state || !job.outSamples || !job.outConfigs || !job.result) {
            if (job.result) {
                job.result->err = ESP_ERR_INVALID_ARG;
            }
            xSemaphoreGive(ctx->done);
            continue;
        }

        *job.result = (sensorarrayFdcWorkerResult_t){
            .err = ESP_ERR_TIMEOUT,
            .row = job.row,
            .devId = ctx->devId,
            .epochId = job.epochId,
        };
        if (job.trace) {
            job.trace->frameSeq = job.frameSeq;
            job.trace->row = job.row;
            job.trace->epochId = job.epochId;
            job.trace->devId = ctx->devId;
            job.trace->jobQueued = true;
        }
        sensorarrayFdcDeviceState_t *fdcState = sensorarrayMeasureGetFdcState(job.state, ctx->devId);
        esp_err_t err = ESP_ERR_INVALID_STATE;
        if (fdcState && fdcState->ready && fdcState->handle) {
            if (sensorarrayMeasureFdcShouldLogNormalFrame(job.frameSeq)) {
                printf("FDC_ROW_EPOCH,stage=sleep_enter,row=%u,epoch=%lu,dev=%s\n",
                       (unsigned)job.row,
                       (unsigned long)job.epochId,
                       sensorarrayMeasureFdcDeviceName(ctx->devId));
            }
            err = sensorarrayMeasureFdcSetSleepMode(fdcState, true, job.timing);
        }
        job.result->err = err;
        if (job.trace) {
            job.trace->err = err;
        }
        xSemaphoreGive(ctx->sleepAck);

        uint32_t startWaitMs = (uint32_t)CONFIG_SENSORARRAY_FDC_WORKER_SYNC_TIMEOUT_MS;
        if (startWaitMs < 100u) {
            startWaitMs = 100u;
        }
        TickType_t waitTicks = pdMS_TO_TICKS(startWaitMs);
        if (waitTicks == 0) {
            waitTicks = 1;
        }
        int64_t startWaitStartUs = esp_timer_get_time();
        if (err == ESP_OK && xSemaphoreTake(ctx->start, waitTicks) == pdTRUE) {
            if (job.trace) {
                job.trace->startWaitUs = (uint32_t)sensorarrayMeasureElapsedUs(startWaitStartUs);
                job.trace->runStarted = true;
            }
            int64_t runStartUs = esp_timer_get_time();
            err = sensorarrayMeasureFdcRunDeviceEpochAfterSleep(job.state,
                                                                job.row,
                                                                job.epochId,
                                                                job.frameSeq,
                                                                ctx->devId,
                                                                job.outSamples,
                                                                job.outConfigs,
                                                                &job.result->ready,
                                                                &job.result->read4,
                                                                job.timing,
                                                                job.readTracker,
                                                                "worker",
                                                                job.rowDeviceDeadlineUs);
            job.result->err = err;
            if (job.trace) {
                job.trace->workerRunUs = (uint32_t)sensorarrayMeasureElapsedUs(runStartUs);
                job.trace->runCompleted = true;
                job.trace->err = err;
            }
        } else if (err == ESP_OK) {
            job.result->err = ESP_ERR_TIMEOUT;
            if (job.trace) {
                job.trace->startWaitUs = (uint32_t)sensorarrayMeasureElapsedUs(startWaitStartUs);
                job.trace->err = ESP_ERR_TIMEOUT;
            }
        }
        xSemaphoreGive(ctx->done);
    }
}

static esp_err_t sensorarrayMeasureEnsureFdcWorkers(void)
{
    if (s_fdcWorkersInitAttempted) {
        return s_fdcWorkersAvailable ? ESP_OK : ESP_ERR_NOT_SUPPORTED;
    }
    s_fdcWorkersInitAttempted = true;

    esp_err_t firstErr = ESP_OK;
    for (uint8_t i = 0u; i < 2u; ++i) {
        sensorarrayFdcWorkerContext_t *ctx = &s_fdcWorkers[i];
        ctx->queue = xQueueCreateStatic(SENSORARRAY_FDC_WORKER_QUEUE_DEPTH,
                                        sizeof(sensorarrayFdcWorkerJob_t),
                                        ctx->queueBuffer,
                                        &ctx->queueStorage);
        ctx->sleepAck = xSemaphoreCreateBinaryStatic(&ctx->sleepAckStorage);
        ctx->start = xSemaphoreCreateBinaryStatic(&ctx->startStorage);
        ctx->done = xSemaphoreCreateBinaryStatic(&ctx->doneStorage);
        if (!ctx->queue || !ctx->sleepAck || !ctx->start || !ctx->done) {
            firstErr = ESP_FAIL;
            printf("FDC_WORKER_CREATE_FAIL,dev=%s,err=0x%lx,status=fallback_serial,reason=resource_init_failed\n",
                   sensorarrayMeasureFdcDeviceName(ctx->devId),
                   (unsigned long)firstErr);
            break;
        }
        (void)sensorarrayMeasureEnsureFdcIntb(ctx);

        char taskName[24] = {0};
        snprintf(taskName, sizeof(taskName), "fdc_%s_worker",
                 ctx->devId == SENSORARRAY_FDC_DEV_SECONDARY ? "secondary" : "primary");
        BaseType_t resolvedCore = 0;
        bool pinned = true;
        const char *coreReason = "unknown";
        if (!sensorarrayMeasureResolveWorkerCore(taskName,
                                                 CONFIG_SENSORARRAY_FDC_WORKER_TASK_CORE,
                                                 1,
                                                 &resolvedCore,
                                                 &pinned,
                                                 &coreReason)) {
            firstErr = ESP_FAIL;
            printf("FDC_WORKER_CREATE_FAIL,dev=%s,err=0x%lx,status=fallback_serial,reason=core_resolve_failed\n",
                   sensorarrayMeasureFdcDeviceName(ctx->devId),
                   (unsigned long)firstErr);
            break;
        }
        if (strcmp(coreReason, "configured") != 0) {
            printf("FDC_WORKER_CORE_FIX,dev=%s,configuredCore=%d,resolvedCore=%d,pinned=%u,portNumProcessors=%d,reason=%s\n",
                   sensorarrayMeasureFdcDeviceName(ctx->devId),
                   CONFIG_SENSORARRAY_FDC_WORKER_TASK_CORE,
                   (int)resolvedCore,
                   pinned ? 1u : 0u,
                   (int)portNUM_PROCESSORS,
                   coreReason);
        }
        printf("FDC_WORKER_CREATE_BEGIN,dev=%s,configuredCore=%d,resolvedCore=%d,pinned=%u,stackWords=%lu,prio=%d,portNumProcessors=%d\n",
               sensorarrayMeasureFdcDeviceName(ctx->devId),
               CONFIG_SENSORARRAY_FDC_WORKER_TASK_CORE,
               (int)resolvedCore,
               pinned ? 1u : 0u,
               (unsigned long)SENSORARRAY_FDC_WORKER_STACK_WORDS,
               CONFIG_SENSORARRAY_FDC_WORKER_TASK_PRIO,
               (int)portNUM_PROCESSORS);
        if (pinned) {
            ctx->task = xTaskCreateStaticPinnedToCore(sensorarrayMeasureFdcWorkerTask,
                                                      taskName,
                                                      SENSORARRAY_FDC_WORKER_STACK_WORDS,
                                                      ctx,
                                                      CONFIG_SENSORARRAY_FDC_WORKER_TASK_PRIO,
                                                      ctx->stack,
                                                      &ctx->taskStorage,
                                                      resolvedCore);
        } else {
            ctx->task = xTaskCreateStatic(sensorarrayMeasureFdcWorkerTask,
                                          taskName,
                                          SENSORARRAY_FDC_WORKER_STACK_WORDS,
                                          ctx,
                                          CONFIG_SENSORARRAY_FDC_WORKER_TASK_PRIO,
                                          ctx->stack,
                                          &ctx->taskStorage);
        }
        if (!ctx->task) {
            firstErr = ESP_FAIL;
            printf("FDC_WORKER_CREATE_FAIL,dev=%s,err=0x%lx,status=fallback_serial,reason=task_create_failed\n",
                   sensorarrayMeasureFdcDeviceName(ctx->devId),
                   (unsigned long)firstErr);
            break;
        }
        printf("FDC_WORKER_CREATE_DONE,dev=%s,handle=%p,err=0x0,status=ok\n",
               sensorarrayMeasureFdcDeviceName(ctx->devId),
               (void *)ctx->task);
        ctx->initialized = true;
    }

    s_fdcWorkersAvailable = firstErr == ESP_OK;
    if (!s_fdcWorkersAvailable) {
        sensorarrayMeasureCleanupFdcWorkers();
        printf("FDC_WORKER,stage=init,status=fallback_serial,err=0x%lx\n",
               (unsigned long)firstErr);
        return firstErr;
    }
    printf("FDC_WORKER,stage=init,status=parallel_ready,primaryBus=0,secondaryBus=1,readyMode=%s,intbHint=%s,configIntbDisabled=%u\n",
           SENSORARRAY_FDC_READY_MODE_NAME,
           SENSORARRAY_FDC_INTB_HINT_NAME,
           SENSORARRAY_FDC_INTB_OUTPUT_ENABLE ? 0u : 1u);
    return ESP_OK;
}

static void sensorarrayMeasureAccumulateRowEpochTiming(sensorarrayFdcTimingSummary_t *summary,
                                                       const sensorarrayFdcRowTiming_t *rowTiming,
                                                       const sensorarrayFdcDeviceTiming_t *primaryTiming,
                                                       const sensorarrayFdcDeviceTiming_t *secondaryTiming)
{
    if (!summary || !rowTiming || !primaryTiming || !secondaryTiming) {
        return;
    }

    summary->sleepBeforeRowSwitchUs += rowTiming->sleepBeforeRowSwitchUs;
    summary->rowSwitchWhileSleepingUs += rowTiming->rowSwitchWhileSleepingUs;
    summary->rowSettleUs += rowTiming->rowSettleUs;
    summary->diffApplyWhileSleepingUs += primaryTiming->applyUs + secondaryTiming->applyUs;
    summary->sleepTotalUs += primaryTiming->sleepEnterUs + secondaryTiming->sleepEnterUs +
                             primaryTiming->sleepExitUs + secondaryTiming->sleepExitUs;
    summary->sleepExitToIntbUs += primaryTiming->sleepExitToIntbUs + secondaryTiming->sleepExitToIntbUs;
    summary->statusReadUs += primaryTiming->statusReadUs + secondaryTiming->statusReadUs;
    summary->statusPrecheckUs += primaryTiming->statusPrecheckUs + secondaryTiming->statusPrecheckUs;
    summary->dataReadUs += primaryTiming->dataReadUs + secondaryTiming->dataReadUs;
    summary->intbWaitOnlyUs += primaryTiming->intbWaitOnlyUs + secondaryTiming->intbWaitOnlyUs;
    summary->statusVerifyAfterIntbUs += primaryTiming->statusVerifyAfterIntbUs +
                                        secondaryTiming->statusVerifyAfterIntbUs;
    summary->pollFallbackUs += primaryTiming->pollFallbackUs + secondaryTiming->pollFallbackUs;
    summary->staleDrainUs += primaryTiming->staleDrainUs + secondaryTiming->staleDrainUs;
    summary->primaryJobUs += rowTiming->primaryJobUs ? rowTiming->primaryJobUs : primaryTiming->deviceUs;
    summary->secondaryJobUs += rowTiming->secondaryJobUs ? rowTiming->secondaryJobUs : secondaryTiming->deviceUs;
    summary->dualBusWaitUs += rowTiming->dualBusWaitUs;
    summary->dualBusSkewUs += rowTiming->dualBusSkewUs;
    summary->workerQueueSendUs += rowTiming->workerQueueSendUs;
    summary->workerSleepAckWaitUs += rowTiming->workerSleepAckWaitUs;
    summary->workerStartGiveUs += rowTiming->workerStartGiveUs;
    summary->workerDoneWaitUs += rowTiming->workerDoneWaitUs;
    summary->workerLateDoneUs += rowTiming->workerLateDoneUs;
    summary->workerIdleAfterTimeoutUs += rowTiming->workerIdleAfterTimeoutUs;
    summary->serialFallbackUs += rowTiming->serialFallbackUs;
    summary->repairPrimaryUs += rowTiming->repairPrimaryUs;
    summary->repairSecondaryUs += rowTiming->repairSecondaryUs;

    summary->cacheApplyDiffWriteCount += primaryTiming->cacheDiffWriteCount +
                                         secondaryTiming->cacheDiffWriteCount;
    summary->cacheApplyFullWriteCount += primaryTiming->cacheFullWriteCount +
                                         secondaryTiming->cacheFullWriteCount;
    summary->cacheApplyNoDiffCount += primaryTiming->cacheNoDiffCount +
                                      secondaryTiming->cacheNoDiffCount;
    summary->diffRcountWrites += primaryTiming->diffRcountWrites + secondaryTiming->diffRcountWrites;
    summary->diffSettleWrites += primaryTiming->diffSettleWrites + secondaryTiming->diffSettleWrites;
    summary->diffClockDivWrites += primaryTiming->diffClockDivWrites + secondaryTiming->diffClockDivWrites;
    summary->diffDriveWrites += primaryTiming->diffDriveWrites + secondaryTiming->diffDriveWrites;
    summary->diffMuxWrites += primaryTiming->diffMuxWrites + secondaryTiming->diffMuxWrites;
    summary->diffStatusConfigWrites += primaryTiming->diffStatusConfigWrites +
                                       secondaryTiming->diffStatusConfigWrites;
    summary->diffConfigWrites += primaryTiming->diffConfigWrites + secondaryTiming->diffConfigWrites;
    summary->appliedFingerprintChanges += primaryTiming->appliedFingerprintChanged +
                                          secondaryTiming->appliedFingerprintChanged;

    summary->intbEdgeCountPrimary += primaryTiming->intbEdgeCount;
    summary->intbEdgeCountSecondary += secondaryTiming->intbEdgeCount;
    summary->intbFalseEdgeCount += primaryTiming->intbFalseEdgeCount +
                                   secondaryTiming->intbFalseEdgeCount;
    summary->intbTimeoutCount += primaryTiming->intbTimeoutCount + secondaryTiming->intbTimeoutCount;
    summary->intbFallbackPollCount += primaryTiming->intbFallbackPollCount +
                                      secondaryTiming->intbFallbackPollCount;
    summary->intbFreshDrdyCount += primaryTiming->intbFreshDrdyCount +
                                   secondaryTiming->intbFreshDrdyCount;
    summary->preStatusReadyCount += primaryTiming->preStatusReadyCount +
                                    secondaryTiming->preStatusReadyCount;
    summary->intbReadyCount += primaryTiming->intbReadyCount + secondaryTiming->intbReadyCount;
    summary->lateStatusReadyCount += primaryTiming->lateStatusReadyCount +
                                     secondaryTiming->lateStatusReadyCount;
    summary->trueTimeoutCount += primaryTiming->trueTimeoutCount + secondaryTiming->trueTimeoutCount;
    summary->readyFullCount += primaryTiming->readyFullCount + secondaryTiming->readyFullCount;
    summary->recoveredAfterRetryCount += primaryTiming->recoveredAfterRetryCount +
                                         secondaryTiming->recoveredAfterRetryCount;
    summary->readyPartialCount += primaryTiming->readyPartialCount + secondaryTiming->readyPartialCount;
    summary->readyNoneCount += primaryTiming->readyNoneCount + secondaryTiming->readyNoneCount;
    summary->transientUnreadNoDrdyCount += primaryTiming->transientUnreadNoDrdyCount +
                                           secondaryTiming->transientUnreadNoDrdyCount;
    summary->staleUnreadDrainCount += primaryTiming->staleUnreadDrainCount +
                                      secondaryTiming->staleUnreadDrainCount;
    summary->hardReadyTimeoutCount += primaryTiming->hardReadyTimeoutCount +
                                      secondaryTiming->hardReadyTimeoutCount;
    summary->statusReadsBeforeIntbCount += primaryTiming->statusReadsBeforeIntbCount +
                                           secondaryTiming->statusReadsBeforeIntbCount;
    summary->statusReadsPrecheckCount += primaryTiming->statusReadsPrecheckCount +
                                         secondaryTiming->statusReadsPrecheckCount;
    summary->statusReadsAfterIntbCount += primaryTiming->statusReadsAfterIntbCount +
                                          secondaryTiming->statusReadsAfterIntbCount;
    summary->statusReadsInFallbackCount += primaryTiming->statusReadsInFallbackCount +
                                           secondaryTiming->statusReadsInFallbackCount;
    summary->statusReadSuppressedBeforeIntbCount += primaryTiming->statusReadSuppressedBeforeIntbCount +
                                                    secondaryTiming->statusReadSuppressedBeforeIntbCount;
    summary->noStatusPollWaitCount += primaryTiming->noStatusPollWaitCount +
                                      secondaryTiming->noStatusPollWaitCount;
    summary->statusAfterIntbCount += primaryTiming->statusAfterIntbCount +
                                     secondaryTiming->statusAfterIntbCount;
    summary->statusAfterTimeoutCount += primaryTiming->statusAfterTimeoutCount +
                                        secondaryTiming->statusAfterTimeoutCount;
    summary->hardTimeoutStatusDiagCount += primaryTiming->hardTimeoutStatusDiagCount +
                                           secondaryTiming->hardTimeoutStatusDiagCount;
    summary->intbActiveStatusMismatchCount += primaryTiming->intbActiveStatusMismatchCount +
                                              secondaryTiming->intbActiveStatusMismatchCount;
    summary->suppressedRpCount += primaryTiming->suppressedRpCount +
                                  secondaryTiming->suppressedRpCount;
    summary->internalWaitStateLeakCount += primaryTiming->internalWaitStateLeakCount +
                                           secondaryTiming->internalWaitStateLeakCount;
    summary->unsafeUnreadNoDrdyCount += primaryTiming->unsafeUnreadNoDrdyCount +
                                        secondaryTiming->unsafeUnreadNoDrdyCount;
    summary->drdyPartialUnreadCount += primaryTiming->drdyPartialUnreadCount +
                                       secondaryTiming->drdyPartialUnreadCount;
    summary->drdyFullUnreadReadyCount += primaryTiming->drdyFullUnreadReadyCount +
                                         secondaryTiming->drdyFullUnreadReadyCount;
    summary->statusReadErrCount += primaryTiming->statusReadErrCount +
                                   secondaryTiming->statusReadErrCount;
    summary->unreadWithoutDrdyCount += primaryTiming->unreadWithoutDrdyCount +
                                       secondaryTiming->unreadWithoutDrdyCount;
    summary->softInvalidCount += primaryTiming->softInvalidCount + secondaryTiming->softInvalidCount;
    summary->hardInvalidCount += primaryTiming->hardInvalidCount + secondaryTiming->hardInvalidCount;
    summary->drainCount += primaryTiming->drainCount + secondaryTiming->drainCount;
    summary->diagUnreadLikelyFreshCount += primaryTiming->diagUnreadLikelyFreshCount +
                                           secondaryTiming->diagUnreadLikelyFreshCount;
    summary->diagUnreadLikelyStaleCount += primaryTiming->diagUnreadLikelyStaleCount +
                                           secondaryTiming->diagUnreadLikelyStaleCount;
    summary->fallbackAttemptCount += primaryTiming->fallbackAttemptCount + secondaryTiming->fallbackAttemptCount;
    summary->fallbackSuccessCount += primaryTiming->fallbackSuccessCount + secondaryTiming->fallbackSuccessCount;
    summary->fallbackPartialCount += primaryTiming->fallbackPartialCount + secondaryTiming->fallbackPartialCount;
    summary->fallbackFailCount += primaryTiming->fallbackFailCount + secondaryTiming->fallbackFailCount;
    summary->deviceFullInvalidCount += primaryTiming->deviceFullInvalidCount +
                                       secondaryTiming->deviceFullInvalidCount;
    summary->workerTimeoutCount += rowTiming->workerTimeoutCount;
    summary->workerLateDoneCount += rowTiming->workerLateDoneCount;
    summary->workerLateGoodAcceptedCount += rowTiming->workerLateGoodAcceptedCount;
    summary->staleResultDiscardedCount += rowTiming->staleResultDiscardedCount;
    summary->duplicateReadCount += rowTiming->duplicateReadCount;
    summary->deferredRepairRequestCount += rowTiming->deferredRepairRequestCount;
    summary->inlineRepairSuppressedCount += rowTiming->inlineRepairSuppressedCount;
    summary->waitReadyUsPrimaryTotal += primaryTiming->waitReadyUs;
    summary->waitReadyUsSecondaryTotal += secondaryTiming->waitReadyUs;
    summary->read4UsPrimaryTotal += primaryTiming->readRawUs;
    summary->read4UsSecondaryTotal += secondaryTiming->readRawUs;
    if (primaryTiming->maxWaitReadyUs > summary->maxWaitReadyUs) {
        summary->maxWaitReadyUs = primaryTiming->maxWaitReadyUs;
    }
    if (secondaryTiming->maxWaitReadyUs > summary->maxWaitReadyUs) {
        summary->maxWaitReadyUs = secondaryTiming->maxWaitReadyUs;
    }
    if (primaryTiming->maxI2cReadUs > summary->maxI2cReadUs) {
        summary->maxI2cReadUs = primaryTiming->maxI2cReadUs;
    }
    if (secondaryTiming->maxI2cReadUs > summary->maxI2cReadUs) {
        summary->maxI2cReadUs = secondaryTiming->maxI2cReadUs;
    }
}

static esp_err_t sensorarrayMeasureSelectFdcRowWhileSleeping(sensorarrayState_t *state,
                                                             uint8_t row,
                                                             uint32_t epochId,
                                                             uint32_t frameSeq,
                                                             sensorarrayFdcRowTiming_t *rowTiming)
{
    (void)state;
    if (sensorarrayMeasureFdcShouldLogNormalFrame(frameSeq)) {
        printf("FDC_ROW_EPOCH,stage=row_switch,row=%u,epoch=%lu\n",
               (unsigned)row,
               (unsigned long)epochId);
    }
    int64_t rowSelectStartUs = esp_timer_get_time();
    esp_err_t err = tmuxSwitchSelectRow((uint8_t)(row - 1u));
    uint64_t rowSelectUs = sensorarrayMeasureElapsedUs(rowSelectStartUs);
    if (rowTiming) {
        rowTiming->rowSelectUs = rowSelectUs;
        rowTiming->rowSwitchWhileSleepingUs += rowSelectUs;
    }

    int64_t settleStartUs = esp_timer_get_time();
    sensorarrayMeasureDelayUs((uint32_t)CONFIG_SENSORARRAY_FDC_ROW_SWITCH_SETTLE_US);
    uint64_t settleUs = sensorarrayMeasureElapsedUs(settleStartUs);
    if (rowTiming) {
        rowTiming->analogSettleUs = settleUs;
        rowTiming->rowSettleUs += settleUs;
    }
    if (sensorarrayMeasureFdcShouldLogNormalFrame(frameSeq)) {
        printf("FDC_EPOCH,stage=after_row_switch,row=%u,dev=both,settleUs=%lu\n",
               (unsigned)row,
               (unsigned long)settleUs);
    }
#if CONFIG_SENSORARRAY_FDC_VERBOSE_SCAN_LOG
    tmuxSwitchControlState_t ctrl = {0};
    sensorarrayMeasureReadFdcPathControl(&ctrl);
    printf("FDC_ROW,stage=select_while_sleep,row=%u,selaCmd=%d,selaReadback=%d,err=0x%lx\n",
           (unsigned)row,
           ctrl.cmdSelaLevel,
           ctrl.obsSelaLevel,
           (unsigned long)err);
#endif
    return err;
}

static esp_err_t sensorarrayMeasureReadFdcMatrixRowSerialEpoch(sensorarrayState_t *state,
                                                               uint8_t row,
                                                               uint32_t epochId,
                                                               uint32_t frameSeq,
                                                               sensorarrayFdcAutoscanSamples_t *primarySamples,
                                                               sensorarrayFdcAutoscanSamples_t *secondarySamples,
                                                               sensorarrayFdcRuntimeChannelConfig_t runtimeConfigs[2][4],
                                                               sensorarrayFdcDeviceTiming_t *primaryTiming,
                                                               sensorarrayFdcDeviceTiming_t *secondaryTiming,
                                                               sensorarrayFdcRowTiming_t *rowTiming,
                                                               sensorarrayFdcFrameReadTracker_t *readTracker)
{
    if (!state || !primarySamples || !secondarySamples || !runtimeConfigs ||
        !primaryTiming || !secondaryTiming || !rowTiming) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t firstErr = ESP_OK;
    bool primaryAvailable = sensorarrayMeasureFdcDeviceReadyForIo(&state->fdcPrimary);
    bool secondaryAvailable = sensorarrayMeasureFdcDeviceReadyForIo(&state->fdcSecondary);
    if (!primaryAvailable) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!secondaryAvailable) {
        sensorarrayMeasureLogFdcSecondaryUnavailableOnce();
        sensorarrayMeasureMarkFdcNoFreshSamples(secondarySamples, NULL, false);
        secondaryTiming->row = row;
        secondaryTiming->deviceId = SENSORARRAY_FDC_DEV_SECONDARY;
        secondaryTiming->deviceFullInvalidCount++;
    }

    int64_t sleepStartUs = esp_timer_get_time();
    bool logNormal = sensorarrayMeasureFdcShouldLogNormalFrame(frameSeq);
    if (logNormal) {
        printf("FDC_ROW_EPOCH,stage=sleep_enter,row=%u,epoch=%lu,dev=primary\n",
               (unsigned)row,
               (unsigned long)epochId);
    }
    esp_err_t primaryErr = sensorarrayMeasureFdcSetSleepMode(&state->fdcPrimary, true, primaryTiming);
    esp_err_t secondaryErr = ESP_ERR_NOT_SUPPORTED;
    if (secondaryAvailable) {
        if (logNormal) {
            printf("FDC_ROW_EPOCH,stage=sleep_enter,row=%u,epoch=%lu,dev=secondary\n",
                   (unsigned)row,
                   (unsigned long)epochId);
        }
        secondaryErr = sensorarrayMeasureFdcSetSleepMode(&state->fdcSecondary, true, secondaryTiming);
    }
    rowTiming->sleepBeforeRowSwitchUs = sensorarrayMeasureElapsedUs(sleepStartUs);
    if (primaryErr != ESP_OK && firstErr == ESP_OK) {
        firstErr = primaryErr;
    }
    if (secondaryAvailable && secondaryErr != ESP_OK && firstErr == ESP_OK) {
        firstErr = secondaryErr;
    }

    sensorarrayMeasureFdcPrepareIntbEpoch(sensorarrayMeasureFdcWorkerContext(SENSORARRAY_FDC_DEV_PRIMARY),
                                          epochId);
    if (secondaryAvailable) {
        sensorarrayMeasureFdcPrepareIntbEpoch(sensorarrayMeasureFdcWorkerContext(SENSORARRAY_FDC_DEV_SECONDARY),
                                              epochId);
    }

    esp_err_t rowErr = sensorarrayMeasureSelectFdcRowWhileSleeping(state, row, epochId, frameSeq, rowTiming);
    if (rowErr != ESP_OK && firstErr == ESP_OK) {
        firstErr = rowErr;
    }

    sensorarrayFdcReadyState_t primaryReady = {0};
    sensorarrayFdcReadyState_t secondaryReady = {0};
    sensorarrayFdcDeviceRead4Result_t primaryRead4 = {0};
    sensorarrayFdcDeviceRead4Result_t secondaryRead4 = {0};
    if (primaryErr == ESP_OK) {
        primaryErr = sensorarrayMeasureFdcRunDeviceEpochAfterSleep(state,
                                                                   row,
                                                                   epochId,
                                                                   frameSeq,
                                                                   SENSORARRAY_FDC_DEV_PRIMARY,
                                                                   primarySamples,
                                                                   runtimeConfigs[SENSORARRAY_FDC_DEV_PRIMARY],
                                                                   &primaryReady,
                                                                   &primaryRead4,
                                                                   primaryTiming,
                                                                   readTracker,
                                                                   "serial",
                                                                   0u);
        if (primaryErr != ESP_OK && firstErr == ESP_OK) {
            firstErr = primaryErr;
        }
    }
    if (secondaryAvailable && secondaryErr == ESP_OK) {
        secondaryErr = sensorarrayMeasureFdcRunDeviceEpochAfterSleep(state,
                                                                     row,
                                                                     epochId,
                                                                     frameSeq,
                                                                     SENSORARRAY_FDC_DEV_SECONDARY,
                                                                     secondarySamples,
                                                                     runtimeConfigs[SENSORARRAY_FDC_DEV_SECONDARY],
                                                                     &secondaryReady,
                                                                     &secondaryRead4,
                                                                     secondaryTiming,
                                                                     readTracker,
                                                                     "serial",
                                                                     0u);
        if (secondaryErr != ESP_OK && firstErr == ESP_OK) {
            firstErr = secondaryErr;
        }
    }

    rowTiming->waitReadyUs = primaryTiming->waitReadyUs + secondaryTiming->waitReadyUs;
    rowTiming->readUs = primaryTiming->readRawUs + secondaryTiming->readRawUs;
    rowTiming->primaryTotalUs = primaryTiming->deviceUs;
    rowTiming->secondaryTotalUs = secondaryTiming->deviceUs;
    rowTiming->dualBusSkewUs =
        (primaryTiming->deviceUs > secondaryTiming->deviceUs) ?
        (primaryTiming->deviceUs - secondaryTiming->deviceUs) :
        (secondaryTiming->deviceUs - primaryTiming->deviceUs);
    (void)primaryReady;
    (void)secondaryReady;
    return firstErr;
}

static void sensorarrayMeasureFdcLogWorkerTimeout(const sensorarrayFdcWorkerTrace_t *trace,
                                                  const char *timeoutStage,
                                                  const sensorarrayFdcWorkerResult_t *result)
{
    if (!trace) {
        return;
    }
    const sensorarrayFdcReadyState_t *ready = result ? &result->ready : NULL;
    const sensorarrayFdcDeviceRead4Result_t *read4 = result ? &result->read4 : NULL;
    const char *timeoutRoot =
        (ready && ready->pollCount != 0u && !ready->readyForDataRead) ? "ready_state_wait" :
        (!trace->doneReceived ? "done_notify" :
         (read4 && read4->readUs != 0u && read4->readErr != ESP_OK) ? "i2c_read" :
         "worker_sync");
    printf("FDC_WORKER_TIMEOUT,seq=%lu,row=%u,epoch=%lu,device=%s,timeoutStage=%s,timeoutRoot=%s,queueSendUs=%lu,sleepAckWaitUs=%lu,startWaitUs=%lu,doneWaitUs=%lu,workerRunUs=%lu,waitWorkerIdleAfterTimeoutUs=%lu,workerDeadlineUs=%llu,rowHardDeadlineUs=%lu,lateDone=%u,staleResultDiscarded=%u,readyState=%s,readState=%s,lastStatus=%04X,lastUnread=%X,lastDrdy=%u,lastDecision=%s,err=0x%lx\n",
           (unsigned long)trace->frameSeq,
           (unsigned)trace->row,
           (unsigned long)trace->epochId,
           sensorarrayMeasureFdcDeviceName(trace->devId),
           timeoutStage ? timeoutStage : "unknown",
           timeoutRoot,
           (unsigned long)trace->queueSendUs,
           (unsigned long)trace->sleepAckWaitUs,
           (unsigned long)trace->startWaitUs,
           (unsigned long)trace->doneWaitUs,
           (unsigned long)trace->workerRunUs,
           (unsigned long)trace->waitWorkerIdleAfterTimeoutUs,
           (unsigned long long)trace->workerDeadlineUs,
           (unsigned long)trace->rowHardDeadlineUs,
           trace->lateDone ? 1u : 0u,
           trace->staleResultDiscarded ? 1u : 0u,
           ready ? sensorarrayMeasureFdcReadyKindName(ready->kind) : "none",
           read4 ? sensorarrayMeasureFdcRead4DiagnosticName(read4) : "none",
           ready ? ready->statusRaw : 0u,
           ready ? (unsigned)(ready->unreadMask & 0x0Fu) : 0u,
           ready && ready->dataReady ? 1u : 0u,
           ready && ready->diagnostic ? ready->diagnostic : "none",
           (unsigned long)trace->err);
}

static void sensorarrayMeasureFdcLogLateDone(const sensorarrayFdcWorkerTrace_t *trace)
{
    if (!trace) {
        return;
    }
    printf("FDC_WORKER_LATE_DONE,seq=%lu,row=%u,epoch=%lu,device=%s,queueSendUs=%lu,sleepAckWaitUs=%lu,startWaitUs=%lu,doneWaitUs=%lu,workerRunUs=%lu,waitWorkerIdleAfterTimeoutUs=%lu,workerDeadlineUs=%llu,rowHardDeadlineUs=%lu,staleResultDiscarded=%u,err=0x%lx\n",
           (unsigned long)trace->frameSeq,
           (unsigned)trace->row,
           (unsigned long)trace->epochId,
           sensorarrayMeasureFdcDeviceName(trace->devId),
           (unsigned long)trace->queueSendUs,
           (unsigned long)trace->sleepAckWaitUs,
           (unsigned long)trace->startWaitUs,
           (unsigned long)trace->doneWaitUs,
           (unsigned long)trace->workerRunUs,
           (unsigned long)trace->waitWorkerIdleAfterTimeoutUs,
           (unsigned long long)trace->workerDeadlineUs,
           (unsigned long)trace->rowHardDeadlineUs,
           trace->staleResultDiscarded ? 1u : 0u,
           (unsigned long)trace->err);
}

static void sensorarrayMeasureFdcDiscardWorkerResult(sensorarrayFdcWorkerResult_t *result,
                                                     sensorarrayFdcAutoscanSamples_t *samples,
                                                     sensorarrayFdcWorkerTrace_t *trace,
                                                     const char *reason)
{
    if (!result) {
        return;
    }
    sensorarrayMeasureMarkFdcNoFreshSamples(samples, &result->ready, result->read4.i2cError);
    result->err = ESP_ERR_TIMEOUT;
    result->read4.staleRejected = true;
    result->read4.validMask4 = 0u;
    result->read4.freshMask4 = 0u;
    result->read4.errorMask4 = 0x0Fu;
    if (trace) {
        trace->staleResultDiscarded =
            (reason && (strcmp(reason, "stale_result") == 0 ||
                        strcmp(reason, "late_epoch_mismatch") == 0));
        trace->err = ESP_ERR_TIMEOUT;
    }
}

static void sensorarrayMeasureFdcDeferDeviceRepairAfterParallel(sensorarrayState_t *state,
                                                                uint8_t row,
                                                                uint32_t epochId,
                                                                uint32_t frameSeq,
                                                                sensorarrayFdcDeviceId_t devId,
                                                                sensorarrayFdcAutoscanSamples_t *samples,
                                                                const sensorarrayFdcWorkerResult_t *result,
                                                                sensorarrayFdcRowTiming_t *rowTiming,
                                                                const char *reason)
{
    if (!state || !samples || devId > SENSORARRAY_FDC_DEV_SECONDARY) {
        return;
    }
    if (result &&
        sensorarrayFdcRead4IsDataCompleteGood(&result->read4,
                                              SENSORARRAY_FDC_AUTOSCAN_READY_UNREAD_MASK)) {
        printf("FDC_RESULT_MERGE_BUG,dev=%s,row=%u,epoch=%lu,validMask=0x%X,freshMask=0x%X,unread=0x%X,drdy=%u,timeout=%u,partial=%u,err=0x%lx,reason=%s,action=keep_recovered_samples\n",
               sensorarrayMeasureFdcDeviceName(devId),
               (unsigned)row,
               (unsigned long)epochId,
               (unsigned)(result->read4.validMask4 & 0x0Fu),
               (unsigned)(result->read4.freshMask4 & 0x0Fu),
               (unsigned)(result->read4.unreadMask4 & 0x0Fu),
               (unsigned)result->read4.drdy,
               result->read4.timeout ? 1u : 0u,
               result->read4.partial ? 1u : 0u,
               (unsigned long)result->read4.readErr,
               reason ? reason : "parallel_device_result");
        return;
    }

    sensorarrayMeasureMarkFdcNoFreshSamples(samples,
                                            result ? &result->ready : NULL,
                                            result ? result->read4.i2cError : false);
    bool softInvalid = result && sensorarrayMeasureFdcReadyResultIsSoftInvalid(&result->ready);
    uint8_t dBase = devId == SENSORARRAY_FDC_DEV_SECONDARY ? 5u : 1u;
    if (!softInvalid) {
        for (uint8_t ch = 0u; ch < 4u; ++ch) {
            uint8_t matrixIndex = (uint8_t)sensorarrayMatrixIndex(row, (uint8_t)(dBase + ch));
            (void)sensorarrayMeasureRequestFdcCellRescue(state,
                                                         matrixIndex,
                                                         reason ? reason : "parallel_deferred_repair");
        }
    }
    if (rowTiming) {
        rowTiming->inlineRepairSuppressedCount++;
        if (!softInvalid) {
            rowTiming->deferredRepairRequestCount++;
        }
    }
    printf("FDC_DEFERRED_REPAIR,seq=%lu,row=%u,epoch=%lu,dev=%s,reason=%s,status=0x%04X,unread=0x%X,drdy=%u,err=0x%lx,classification=%s,action=%s\n",
           (unsigned long)frameSeq,
           (unsigned)row,
           (unsigned long)epochId,
           sensorarrayMeasureFdcDeviceName(devId),
           reason ? reason : "parallel_device_result",
           result ? result->ready.statusRaw : 0u,
           result ? (unsigned)(result->ready.unreadMask & 0x0Fu) : 0u,
           result ? (unsigned)result->ready.drdy : 0u,
           result ? (unsigned long)result->err : (unsigned long)ESP_ERR_INVALID_RESPONSE,
           softInvalid ? sensorarrayMeasureFdcReadyResultName(result->ready.readyResult) : "hard",
           softInvalid ? "mark_soft_invalid_no_rescue" : "mark_invalid_defer_rescue");
}

static uint32_t sensorarrayMeasureFdcAbsDeltaUs(uint64_t a, uint64_t b)
{
    if (a == 0u || b == 0u) {
        return 0u;
    }
    uint64_t delta = (a > b) ? (a - b) : (b - a);
    return delta > UINT32_MAX ? UINT32_MAX : (uint32_t)delta;
}

static uint64_t sensorarrayMeasureFdcSpanUs(uint64_t startA,
                                            uint64_t doneA,
                                            uint64_t startB,
                                            uint64_t doneB)
{
    if (startA == 0u || doneA == 0u || startB == 0u || doneB == 0u) {
        return 0u;
    }
    uint64_t start = startA < startB ? startA : startB;
    uint64_t done = doneA > doneB ? doneA : doneB;
    return done >= start ? (done - start) : 0u;
}

static void sensorarrayMeasureFdcFinalizeParallelTiming(uint8_t row,
                                                        uint32_t epochId,
                                                        const sensorarrayFdcDeviceTiming_t *primaryTiming,
                                                        const sensorarrayFdcDeviceTiming_t *secondaryTiming,
                                                        sensorarrayFdcRowTiming_t *rowTiming,
                                                        esp_err_t primaryErr,
                                                        esp_err_t secondaryErr)
{
    if (!primaryTiming || !secondaryTiming || !rowTiming) {
        return;
    }

    rowTiming->primarySleepExitStartUs = primaryTiming->sleepExitStartUs;
    rowTiming->primarySleepExitDoneUs = primaryTiming->sleepExitDoneUs;
    rowTiming->secondarySleepExitStartUs = secondaryTiming->sleepExitStartUs;
    rowTiming->secondarySleepExitDoneUs = secondaryTiming->sleepExitDoneUs;
    rowTiming->primaryReadyBeginUs = primaryTiming->readyBeginUs;
    rowTiming->secondaryReadyBeginUs = secondaryTiming->readyBeginUs;
    rowTiming->primaryIntbSeenUs = primaryTiming->intbSeenUs;
    rowTiming->secondaryIntbSeenUs = secondaryTiming->intbSeenUs;
    rowTiming->primaryStatusVerifyStartUs = primaryTiming->statusVerifyStartUs;
    rowTiming->secondaryStatusVerifyStartUs = secondaryTiming->statusVerifyStartUs;
    rowTiming->primaryDrdyUs = primaryTiming->drdyUs;
    rowTiming->secondaryDrdyUs = secondaryTiming->drdyUs;
    rowTiming->primaryReadStartUs = primaryTiming->readStartUs;
    rowTiming->secondaryReadStartUs = secondaryTiming->readStartUs;
    rowTiming->primaryReadDoneUs = primaryTiming->readDoneUs;
    rowTiming->secondaryReadDoneUs = secondaryTiming->readDoneUs;

    rowTiming->sleepExitStartDeltaUs =
        sensorarrayMeasureFdcAbsDeltaUs(primaryTiming->sleepExitStartUs,
                                        secondaryTiming->sleepExitStartUs);
    rowTiming->readyBeginDeltaUs =
        sensorarrayMeasureFdcAbsDeltaUs(primaryTiming->readyBeginUs,
                                        secondaryTiming->readyBeginUs);
    rowTiming->intbSeenDeltaUs =
        sensorarrayMeasureFdcAbsDeltaUs(primaryTiming->intbSeenUs,
                                        secondaryTiming->intbSeenUs);
    rowTiming->statusVerifyStartDeltaUs =
        sensorarrayMeasureFdcAbsDeltaUs(primaryTiming->statusVerifyStartUs,
                                        secondaryTiming->statusVerifyStartUs);
    rowTiming->drdyDeltaUs =
        sensorarrayMeasureFdcAbsDeltaUs(primaryTiming->drdyUs,
                                        secondaryTiming->drdyUs);
    rowTiming->readStartDeltaUs =
        sensorarrayMeasureFdcAbsDeltaUs(primaryTiming->readStartUs,
                                        secondaryTiming->readStartUs);

    uint64_t primaryJobUs =
        (primaryTiming->sleepExitStartUs != 0u && primaryTiming->readDoneUs != 0u &&
         primaryTiming->readDoneUs >= primaryTiming->sleepExitStartUs) ?
        (primaryTiming->readDoneUs - primaryTiming->sleepExitStartUs) :
        primaryTiming->deviceUs;
    uint64_t secondaryJobUs =
        (secondaryTiming->sleepExitStartUs != 0u && secondaryTiming->readDoneUs != 0u &&
         secondaryTiming->readDoneUs >= secondaryTiming->sleepExitStartUs) ?
        (secondaryTiming->readDoneUs - secondaryTiming->sleepExitStartUs) :
        secondaryTiming->deviceUs;
    rowTiming->primaryJobUs = primaryJobUs;
    rowTiming->secondaryJobUs = secondaryJobUs;
    rowTiming->parallelSerialEquivalentUs = primaryJobUs + secondaryJobUs;
    rowTiming->parallelSpanUs =
        sensorarrayMeasureFdcSpanUs(primaryTiming->sleepExitStartUs,
                                    primaryTiming->readDoneUs,
                                    secondaryTiming->sleepExitStartUs,
                                    secondaryTiming->readDoneUs);
    if (rowTiming->parallelSpanUs == 0u) {
        rowTiming->parallelSpanUs = rowTiming->dualBusWaitUs;
    }
    uint64_t minJobUs = primaryJobUs < secondaryJobUs ? primaryJobUs : secondaryJobUs;
    if (minJobUs == 0u) {
        minJobUs = 1u;
    }
    rowTiming->parallelEfficiencyPct =
        (rowTiming->parallelSerialEquivalentUs > rowTiming->parallelSpanUs) ?
        (uint32_t)(((rowTiming->parallelSerialEquivalentUs - rowTiming->parallelSpanUs) * 100ull) /
                   minJobUs) :
        0u;

    if (CONFIG_SENSORARRAY_FDC_LOG_ROW_PARALLEL_TIMING) {
        printf("RE,r=%u,e=%lu,pj=%llu,sj=%llu,span=%llu,ser=%llu,pe=%lu,sdx=%lu,rdx=%lu,idx=%lu,tdx=%lu,ddx=%lu,rx=%lu,pe0=0x%lx,se0=0x%lx\n",
               (unsigned)row,
               (unsigned long)epochId,
               (unsigned long long)primaryJobUs,
               (unsigned long long)secondaryJobUs,
               (unsigned long long)rowTiming->parallelSpanUs,
               (unsigned long long)rowTiming->parallelSerialEquivalentUs,
               (unsigned long)rowTiming->parallelEfficiencyPct,
               (unsigned long)rowTiming->sleepExitStartDeltaUs,
               (unsigned long)rowTiming->readyBeginDeltaUs,
               (unsigned long)rowTiming->intbSeenDeltaUs,
               (unsigned long)rowTiming->statusVerifyStartDeltaUs,
               (unsigned long)rowTiming->drdyDeltaUs,
               (unsigned long)rowTiming->readStartDeltaUs,
               (unsigned long)primaryErr,
               (unsigned long)secondaryErr);
    }
}

static esp_err_t sensorarrayMeasureReadFdcMatrixRowParallelEpoch(sensorarrayState_t *state,
                                                                 uint8_t row,
                                                                 uint32_t epochId,
                                                                 uint32_t frameSeq,
                                                                 sensorarrayFdcAutoscanSamples_t *primarySamples,
                                                                 sensorarrayFdcAutoscanSamples_t *secondarySamples,
                                                                 sensorarrayFdcRuntimeChannelConfig_t runtimeConfigs[2][4],
                                                                 sensorarrayFdcDeviceTiming_t *primaryTiming,
                                                                 sensorarrayFdcDeviceTiming_t *secondaryTiming,
                                                                 sensorarrayFdcRowTiming_t *rowTiming,
                                                                 sensorarrayFdcFrameReadTracker_t *readTracker,
                                                                 bool *outWorkerTimeout)
{
    if (!state || !primarySamples || !secondarySamples || !runtimeConfigs ||
        !primaryTiming || !secondaryTiming || !rowTiming) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t err = sensorarrayMeasureEnsureFdcWorkers();
    if (err != ESP_OK) {
        return err;
    }

    sensorarrayFdcWorkerContext_t *primaryCtx = sensorarrayMeasureFdcWorkerContext(SENSORARRAY_FDC_DEV_PRIMARY);
    sensorarrayFdcWorkerContext_t *secondaryCtx = sensorarrayMeasureFdcWorkerContext(SENSORARRAY_FDC_DEV_SECONDARY);
    if (!primaryCtx || !secondaryCtx || !primaryCtx->initialized || !secondaryCtx->initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    sensorarrayMeasureDrainSemaphore(primaryCtx->sleepAck);
    sensorarrayMeasureDrainSemaphore(primaryCtx->start);
    sensorarrayMeasureDrainSemaphore(primaryCtx->done);
    sensorarrayMeasureDrainSemaphore(secondaryCtx->sleepAck);
    sensorarrayMeasureDrainSemaphore(secondaryCtx->start);
    sensorarrayMeasureDrainSemaphore(secondaryCtx->done);
    primaryCtx->rowConfigPrepared = false;
    primaryCtx->preparedEpoch = 0u;
    primaryCtx->preparedErr = (int)ESP_ERR_INVALID_STATE;
    secondaryCtx->rowConfigPrepared = false;
    secondaryCtx->preparedEpoch = 0u;
    secondaryCtx->preparedErr = (int)ESP_ERR_INVALID_STATE;

    if (outWorkerTimeout) {
        *outWorkerTimeout = false;
    }

    sensorarrayFdcWorkerResult_t primaryResult = {0};
    sensorarrayFdcWorkerResult_t secondaryResult = {0};
    uint32_t rowHardDeadlineUs = sensorarrayMeasureFdcRowDeviceWatchdogHardTimeoutUs();
    uint64_t workerDeadlineUs = (uint64_t)esp_timer_get_time() + rowHardDeadlineUs;
    sensorarrayFdcWorkerTrace_t primaryTrace = {
        .frameSeq = frameSeq,
        .row = row,
        .epochId = epochId,
        .devId = SENSORARRAY_FDC_DEV_PRIMARY,
        .workerDeadlineUs = workerDeadlineUs,
        .rowHardDeadlineUs = rowHardDeadlineUs,
        .err = ESP_ERR_TIMEOUT,
    };
    sensorarrayFdcWorkerTrace_t secondaryTrace = {
        .frameSeq = frameSeq,
        .row = row,
        .epochId = epochId,
        .devId = SENSORARRAY_FDC_DEV_SECONDARY,
        .workerDeadlineUs = workerDeadlineUs,
        .rowHardDeadlineUs = rowHardDeadlineUs,
        .err = ESP_ERR_TIMEOUT,
    };
    sensorarrayFdcWorkerJob_t primaryJob = {
        .state = state,
        .frameSeq = frameSeq,
        .row = row,
        .epochId = epochId,
        .outSamples = primarySamples,
        .outConfigs = runtimeConfigs[SENSORARRAY_FDC_DEV_PRIMARY],
        .timing = primaryTiming,
        .result = &primaryResult,
        .trace = &primaryTrace,
        .readTracker = readTracker,
        .rowDeviceDeadlineUs = workerDeadlineUs,
    };
    sensorarrayFdcWorkerJob_t secondaryJob = {
        .state = state,
        .frameSeq = frameSeq,
        .row = row,
        .epochId = epochId,
        .outSamples = secondarySamples,
        .outConfigs = runtimeConfigs[SENSORARRAY_FDC_DEV_SECONDARY],
        .timing = secondaryTiming,
        .result = &secondaryResult,
        .trace = &secondaryTrace,
        .readTracker = readTracker,
        .rowDeviceDeadlineUs = workerDeadlineUs,
    };

    TickType_t syncTicks = pdMS_TO_TICKS((uint32_t)CONFIG_SENSORARRAY_FDC_WORKER_SYNC_TIMEOUT_MS);
    if (syncTicks == 0) {
        syncTicks = 1;
    }
    int64_t sleepStartUs = esp_timer_get_time();
    int64_t queueStartUs = esp_timer_get_time();
    BaseType_t primaryQueued = xQueueSend(primaryCtx->queue, &primaryJob, syncTicks);
    primaryTrace.queueSendUs = (uint32_t)sensorarrayMeasureElapsedUs(queueStartUs);
    queueStartUs = esp_timer_get_time();
    BaseType_t secondaryQueued = xQueueSend(secondaryCtx->queue, &secondaryJob, syncTicks);
    secondaryTrace.queueSendUs = (uint32_t)sensorarrayMeasureElapsedUs(queueStartUs);
    rowTiming->workerQueueSendUs = primaryTrace.queueSendUs + secondaryTrace.queueSendUs;
    if (primaryQueued != pdTRUE || secondaryQueued != pdTRUE) {
        if (outWorkerTimeout) {
            *outWorkerTimeout = true;
        }
        if (primaryQueued == pdTRUE) {
            (void)xSemaphoreTake(primaryCtx->sleepAck, syncTicks);
            (void)xSemaphoreTake(primaryCtx->done, portMAX_DELAY);
        } else {
            primaryTrace.err = ESP_ERR_TIMEOUT;
            sensorarrayMeasureFdcLogWorkerTimeout(&primaryTrace, "queue_send", NULL);
        }
        if (secondaryQueued == pdTRUE) {
            (void)xSemaphoreTake(secondaryCtx->sleepAck, syncTicks);
            (void)xSemaphoreTake(secondaryCtx->done, portMAX_DELAY);
        } else {
            secondaryTrace.err = ESP_ERR_TIMEOUT;
            sensorarrayMeasureFdcLogWorkerTimeout(&secondaryTrace, "queue_send", NULL);
        }
        printf("FDC_PARALLEL_FALLBACK,reason=queue_failed,row=%u,epoch=%lu,primaryQueued=%u,secondaryQueued=%u,workerDeadlineUs=%llu,rowHardDeadlineUs=%lu,workerTimedOut=1,err=0x%lx\n",
               (unsigned)row,
               (unsigned long)epochId,
               primaryQueued == pdTRUE ? 1u : 0u,
               secondaryQueued == pdTRUE ? 1u : 0u,
               (unsigned long long)workerDeadlineUs,
               (unsigned long)rowHardDeadlineUs,
               (unsigned long)ESP_ERR_TIMEOUT);
        return ESP_ERR_TIMEOUT;
    }

    int64_t sleepAckStartUs = esp_timer_get_time();
    BaseType_t primarySleep =
        sensorarrayMeasureFdcTakeUntilDeadline(primaryCtx->sleepAck,
                                               workerDeadlineUs,
                                               (uint32_t)CONFIG_SENSORARRAY_FDC_WORKER_SYNC_TIMEOUT_MS * 1000u);
    primaryTrace.sleepAckWaitUs = (uint32_t)sensorarrayMeasureElapsedUs(sleepAckStartUs);
    primaryTrace.sleepAckReceived = primarySleep == pdTRUE;
    sleepAckStartUs = esp_timer_get_time();
    BaseType_t secondarySleep =
        sensorarrayMeasureFdcTakeUntilDeadline(secondaryCtx->sleepAck,
                                               workerDeadlineUs,
                                               (uint32_t)CONFIG_SENSORARRAY_FDC_WORKER_SYNC_TIMEOUT_MS * 1000u);
    secondaryTrace.sleepAckWaitUs = (uint32_t)sensorarrayMeasureElapsedUs(sleepAckStartUs);
    secondaryTrace.sleepAckReceived = secondarySleep == pdTRUE;
    rowTiming->workerSleepAckWaitUs =
        primaryTrace.sleepAckWaitUs + secondaryTrace.sleepAckWaitUs;
    rowTiming->sleepBeforeRowSwitchUs = sensorarrayMeasureElapsedUs(sleepStartUs);
    esp_err_t firstErr = ESP_OK;
    if (primarySleep != pdTRUE) {
        primaryResult.err = ESP_ERR_TIMEOUT;
        firstErr = ESP_ERR_TIMEOUT;
        if (outWorkerTimeout) {
            *outWorkerTimeout = true;
        }
    } else if (primaryResult.err != ESP_OK) {
        firstErr = primaryResult.err;
    }
    if (secondarySleep != pdTRUE) {
        secondaryResult.err = ESP_ERR_TIMEOUT;
        if (outWorkerTimeout) {
            *outWorkerTimeout = true;
        }
        if (firstErr == ESP_OK) {
            firstErr = ESP_ERR_TIMEOUT;
        }
    } else if (secondaryResult.err != ESP_OK && firstErr == ESP_OK) {
        firstErr = secondaryResult.err;
    }
    if (firstErr != ESP_OK) {
        if (primaryQueued == pdTRUE) {
            (void)xSemaphoreTake(primaryCtx->done, portMAX_DELAY);
        }
        if (secondaryQueued == pdTRUE) {
            (void)xSemaphoreTake(secondaryCtx->done, portMAX_DELAY);
        }
        if (primarySleep != pdTRUE || primaryResult.err == ESP_ERR_TIMEOUT) {
            primaryTrace.err = ESP_ERR_TIMEOUT;
            sensorarrayMeasureFdcLogWorkerTimeout(&primaryTrace, "sleep_ack", NULL);
        }
        if (secondarySleep != pdTRUE || secondaryResult.err == ESP_ERR_TIMEOUT) {
            secondaryTrace.err = ESP_ERR_TIMEOUT;
            sensorarrayMeasureFdcLogWorkerTimeout(&secondaryTrace, "sleep_ack", NULL);
        }
        return firstErr;
    }

    sensorarrayMeasureFdcPrepareIntbEpoch(primaryCtx, epochId);
    sensorarrayMeasureFdcPrepareIntbEpoch(secondaryCtx, epochId);

    esp_err_t rowErr = sensorarrayMeasureSelectFdcRowWhileSleeping(state, row, epochId, frameSeq, rowTiming);
    if (rowErr != ESP_OK && firstErr == ESP_OK) {
        firstErr = rowErr;
    }
    if (rowErr != ESP_OK) {
        (void)xSemaphoreTake(primaryCtx->done, portMAX_DELAY);
        (void)xSemaphoreTake(secondaryCtx->done, portMAX_DELAY);
        return rowErr;
    }

    esp_err_t primaryPrepareErr =
        sensorarrayMeasureFdcPrepareDeviceWhileSleeping(state,
                                                        row,
                                                        epochId,
                                                        frameSeq,
                                                        SENSORARRAY_FDC_DEV_PRIMARY,
                                                        runtimeConfigs[SENSORARRAY_FDC_DEV_PRIMARY],
                                                        primaryTiming);
    primaryCtx->preparedEpoch = epochId;
    primaryCtx->preparedErr = (int)primaryPrepareErr;
    primaryCtx->rowConfigPrepared = true;
    esp_err_t secondaryPrepareErr =
        sensorarrayMeasureFdcPrepareDeviceWhileSleeping(state,
                                                        row,
                                                        epochId,
                                                        frameSeq,
                                                        SENSORARRAY_FDC_DEV_SECONDARY,
                                                        runtimeConfigs[SENSORARRAY_FDC_DEV_SECONDARY],
                                                        secondaryTiming);
    secondaryCtx->preparedEpoch = epochId;
    secondaryCtx->preparedErr = (int)secondaryPrepareErr;
    secondaryCtx->rowConfigPrepared = true;
    if (primaryPrepareErr != ESP_OK && primaryPrepareErr != ESP_ERR_NOT_FOUND &&
        firstErr == ESP_OK) {
        firstErr = primaryPrepareErr;
    }
    if (secondaryPrepareErr != ESP_OK && secondaryPrepareErr != ESP_ERR_NOT_FOUND &&
        firstErr == ESP_OK) {
        firstErr = secondaryPrepareErr;
    }
    if (sensorarrayMeasureFdcShouldLogNormalFrame(frameSeq)) {
        printf("FDC_ROW_EPOCH,stage=workers_armed,row=%u,epoch=%lu,mask=0x3\n",
               (unsigned)row,
               (unsigned long)epochId);
    }

    int64_t dualStartUs = esp_timer_get_time();
    int64_t startGiveStartUs = esp_timer_get_time();
    rowTiming->workerReleaseUs = (uint64_t)startGiveStartUs;
    if (sensorarrayMeasureFdcShouldLogNormalFrame(frameSeq)) {
        printf("FDC_ROW_EPOCH,stage=workers_release,row=%u,epoch=%lu\n",
               (unsigned)row,
               (unsigned long)epochId);
    }
    xSemaphoreGive(primaryCtx->start);
    primaryTrace.startGiven = true;
    xSemaphoreGive(secondaryCtx->start);
    secondaryTrace.startGiven = true;
    rowTiming->workerStartGiveUs = sensorarrayMeasureElapsedUs(startGiveStartUs);
    int64_t doneOverallStartUs = startGiveStartUs;
    int64_t doneStartUs = doneOverallStartUs;
    BaseType_t primaryDone =
        sensorarrayMeasureFdcTakeUntilDeadline(primaryCtx->done,
                                               workerDeadlineUs,
                                               rowHardDeadlineUs);
    primaryTrace.doneWaitUs = (uint32_t)sensorarrayMeasureElapsedUs(doneStartUs);
    primaryTrace.doneReceived = primaryDone == pdTRUE;
    doneStartUs = esp_timer_get_time();
    BaseType_t secondaryDone =
        sensorarrayMeasureFdcTakeUntilDeadline(secondaryCtx->done,
                                               workerDeadlineUs,
                                               0u);
    secondaryTrace.doneWaitUs = (uint32_t)sensorarrayMeasureElapsedUs(doneStartUs);
    secondaryTrace.doneReceived = secondaryDone == pdTRUE;
    rowTiming->workerDoneWaitUs = sensorarrayMeasureElapsedUs(doneOverallStartUs);
    rowTiming->parallelJoinWaitUs = rowTiming->workerDoneWaitUs;
    rowTiming->dualBusWaitUs = sensorarrayMeasureElapsedUs(dualStartUs);
    bool primaryLate = false;
    bool secondaryLate = false;
    if (primaryDone != pdTRUE) {
        primaryTiming->timeoutCount++;
        rowTiming->workerTimeoutCount++;
        if (outWorkerTimeout) {
            *outWorkerTimeout = true;
        }
        int64_t idleStartUs = esp_timer_get_time();
        primaryDone = xSemaphoreTake(primaryCtx->done, portMAX_DELAY);
        primaryTrace.waitWorkerIdleAfterTimeoutUs =
            (uint32_t)sensorarrayMeasureElapsedUs(idleStartUs);
        rowTiming->workerIdleAfterTimeoutUs += primaryTrace.waitWorkerIdleAfterTimeoutUs;
        primaryTrace.doneReceived = primaryDone == pdTRUE;
        primaryTrace.lateDone = primaryDone == pdTRUE;
        primaryLate = primaryDone == pdTRUE;
        bool primaryLateGood =
            primaryLate &&
            sensorarrayMeasureFdcWorkerResultIsGood(&primaryResult,
                                                    row,
                                                    epochId,
                                                    SENSORARRAY_FDC_DEV_PRIMARY);
        bool primaryLateMismatch =
            primaryLate &&
            (primaryResult.epochId != epochId ||
             primaryResult.row != row ||
             primaryResult.devId != SENSORARRAY_FDC_DEV_PRIMARY);
        if (primaryLateGood) {
            rowTiming->workerLateGoodAcceptedCount++;
            printf("FDC_WORKER_LATE_GOOD_ACCEPTED,seq=%lu,row=%u,epoch=%lu,device=primary,waitWorkerIdleAfterTimeoutUs=%lu,validMask=0x%X,status=%04X,unread=%X,drdy=%u\n",
                   (unsigned long)frameSeq,
                   (unsigned)row,
                   (unsigned long)epochId,
                   (unsigned long)primaryTrace.waitWorkerIdleAfterTimeoutUs,
                   (unsigned)primaryResult.read4.validMask4,
                   primaryResult.ready.statusRaw,
                   (unsigned)primaryResult.ready.unreadMask,
                   primaryResult.ready.dataReady ? 1u : 0u);
        } else {
            primaryResult.err = ESP_ERR_TIMEOUT;
            sensorarrayMeasureFdcDiscardWorkerResult(&primaryResult,
                                                     primarySamples,
                                                     &primaryTrace,
                                                     primaryLateMismatch ?
                                                     "late_epoch_mismatch" :
                                                     "late_done");
            if (primaryLateMismatch) {
                rowTiming->staleResultDiscardedCount++;
                printf("FDC_WORKER_EPOCH_MISMATCH_DISCARD,seq=%lu,row=%u,epoch=%lu,device=primary,resultRow=%u,resultEpoch=%lu,resultDev=%s,late=1\n",
                       (unsigned long)frameSeq,
                       (unsigned)row,
                       (unsigned long)epochId,
                       (unsigned)primaryResult.row,
                       (unsigned long)primaryResult.epochId,
                       sensorarrayMeasureFdcDeviceName(primaryResult.devId));
            }
        }
        sensorarrayMeasureFdcLogWorkerTimeout(&primaryTrace, "done_wait", &primaryResult);
        if (primaryLate) {
            rowTiming->workerLateDoneCount++;
            rowTiming->workerLateDoneUs += primaryTrace.waitWorkerIdleAfterTimeoutUs;
            sensorarrayMeasureFdcLogLateDone(&primaryTrace);
        }
        if (!primaryLateGood && firstErr == ESP_OK) {
            firstErr = ESP_ERR_TIMEOUT;
        }
    }
    if (secondaryDone != pdTRUE) {
        secondaryTiming->timeoutCount++;
        rowTiming->workerTimeoutCount++;
        if (outWorkerTimeout) {
            *outWorkerTimeout = true;
        }
        int64_t idleStartUs = esp_timer_get_time();
        secondaryDone = xSemaphoreTake(secondaryCtx->done, portMAX_DELAY);
        secondaryTrace.waitWorkerIdleAfterTimeoutUs =
            (uint32_t)sensorarrayMeasureElapsedUs(idleStartUs);
        rowTiming->workerIdleAfterTimeoutUs += secondaryTrace.waitWorkerIdleAfterTimeoutUs;
        secondaryTrace.doneReceived = secondaryDone == pdTRUE;
        secondaryTrace.lateDone = secondaryDone == pdTRUE;
        secondaryLate = secondaryDone == pdTRUE;
        bool secondaryLateGood =
            secondaryLate &&
            sensorarrayMeasureFdcWorkerResultIsGood(&secondaryResult,
                                                    row,
                                                    epochId,
                                                    SENSORARRAY_FDC_DEV_SECONDARY);
        bool secondaryLateMismatch =
            secondaryLate &&
            (secondaryResult.epochId != epochId ||
             secondaryResult.row != row ||
             secondaryResult.devId != SENSORARRAY_FDC_DEV_SECONDARY);
        if (secondaryLateGood) {
            rowTiming->workerLateGoodAcceptedCount++;
            printf("FDC_WORKER_LATE_GOOD_ACCEPTED,seq=%lu,row=%u,epoch=%lu,device=secondary,waitWorkerIdleAfterTimeoutUs=%lu,validMask=0x%X,status=%04X,unread=%X,drdy=%u\n",
                   (unsigned long)frameSeq,
                   (unsigned)row,
                   (unsigned long)epochId,
                   (unsigned long)secondaryTrace.waitWorkerIdleAfterTimeoutUs,
                   (unsigned)secondaryResult.read4.validMask4,
                   secondaryResult.ready.statusRaw,
                   (unsigned)secondaryResult.ready.unreadMask,
                   secondaryResult.ready.dataReady ? 1u : 0u);
        } else {
            secondaryResult.err = ESP_ERR_TIMEOUT;
            sensorarrayMeasureFdcDiscardWorkerResult(&secondaryResult,
                                                     secondarySamples,
                                                     &secondaryTrace,
                                                     secondaryLateMismatch ?
                                                     "late_epoch_mismatch" :
                                                     "late_done");
            if (secondaryLateMismatch) {
                rowTiming->staleResultDiscardedCount++;
                printf("FDC_WORKER_EPOCH_MISMATCH_DISCARD,seq=%lu,row=%u,epoch=%lu,device=secondary,resultRow=%u,resultEpoch=%lu,resultDev=%s,late=1\n",
                       (unsigned long)frameSeq,
                       (unsigned)row,
                       (unsigned long)epochId,
                       (unsigned)secondaryResult.row,
                       (unsigned long)secondaryResult.epochId,
                       sensorarrayMeasureFdcDeviceName(secondaryResult.devId));
            }
        }
        sensorarrayMeasureFdcLogWorkerTimeout(&secondaryTrace, "done_wait", &secondaryResult);
        if (secondaryLate) {
            rowTiming->workerLateDoneCount++;
            rowTiming->workerLateDoneUs += secondaryTrace.waitWorkerIdleAfterTimeoutUs;
            sensorarrayMeasureFdcLogLateDone(&secondaryTrace);
        }
        if (!secondaryLateGood && firstErr == ESP_OK) {
            firstErr = ESP_ERR_TIMEOUT;
        }
    }
    bool primaryStale = false;
    bool secondaryStale = false;
    if (!primaryLate &&
        primaryDone == pdTRUE &&
        (primaryResult.epochId != epochId ||
         primaryResult.row != row ||
         primaryResult.devId != SENSORARRAY_FDC_DEV_PRIMARY)) {
        primaryStale = true;
        rowTiming->staleResultDiscardedCount++;
        printf("FDC_WORKER_EPOCH_MISMATCH_DISCARD,seq=%lu,row=%u,epoch=%lu,device=primary,resultRow=%u,resultEpoch=%lu,resultDev=%s,late=0\n",
               (unsigned long)frameSeq,
               (unsigned)row,
               (unsigned long)epochId,
               (unsigned)primaryResult.row,
               (unsigned long)primaryResult.epochId,
               sensorarrayMeasureFdcDeviceName(primaryResult.devId));
        sensorarrayMeasureFdcDiscardWorkerResult(&primaryResult,
                                                 primarySamples,
                                                 &primaryTrace,
                                                 "stale_result");
    }
    if (!secondaryLate &&
        secondaryDone == pdTRUE &&
        (secondaryResult.epochId != epochId ||
         secondaryResult.row != row ||
         secondaryResult.devId != SENSORARRAY_FDC_DEV_SECONDARY)) {
        secondaryStale = true;
        rowTiming->staleResultDiscardedCount++;
        printf("FDC_WORKER_EPOCH_MISMATCH_DISCARD,seq=%lu,row=%u,epoch=%lu,device=secondary,resultRow=%u,resultEpoch=%lu,resultDev=%s,late=0\n",
               (unsigned long)frameSeq,
               (unsigned)row,
               (unsigned long)epochId,
               (unsigned)secondaryResult.row,
               (unsigned long)secondaryResult.epochId,
               sensorarrayMeasureFdcDeviceName(secondaryResult.devId));
        sensorarrayMeasureFdcDiscardWorkerResult(&secondaryResult,
                                                 secondarySamples,
                                                 &secondaryTrace,
                                                 "stale_result");
    }

    bool primaryGood =
                       !primaryStale &&
                       sensorarrayMeasureFdcWorkerResultIsGood(&primaryResult,
                                                               row,
                                                               epochId,
                                                               SENSORARRAY_FDC_DEV_PRIMARY);
    bool secondaryGood =
                         !secondaryStale &&
                         sensorarrayMeasureFdcWorkerResultIsGood(&secondaryResult,
                                                                 row,
                                                                 epochId,
                                                                 SENSORARRAY_FDC_DEV_SECONDARY);
    if (!primaryGood || !secondaryGood) {
        bool workerTimeout = outWorkerTimeout && *outWorkerTimeout;
        printf("FDC_PARALLEL_FALLBACK,reason=%s,row=%u,epoch=%lu,primaryGood=%u,secondaryGood=%u,primaryErr=0x%lx,secondaryErr=0x%lx,primaryLate=%u,secondaryLate=%u,primaryStale=%u,secondaryStale=%u,primaryBus=%d,secondaryBus=%d,sameBus=0,workerDeadlineUs=%llu,workerJoinUs=%llu,parentWaitUs=%llu,workerTimedOut=%u,workerLateDoneUs=%llu,staleDiscarded=%lu,rowHardDeadlineUs=%lu,workerDoneWaitUs=%llu,workerIdleAfterTimeoutUs=%llu\n",
               workerTimeout ? "timeout" : "device_result",
               (unsigned)row,
               (unsigned long)epochId,
               primaryGood ? 1u : 0u,
               secondaryGood ? 1u : 0u,
               (unsigned long)primaryResult.err,
               (unsigned long)secondaryResult.err,
               primaryLate ? 1u : 0u,
               secondaryLate ? 1u : 0u,
               primaryStale ? 1u : 0u,
               secondaryStale ? 1u : 0u,
               0,
               1,
               (unsigned long long)workerDeadlineUs,
               (unsigned long long)rowTiming->parallelJoinWaitUs,
               (unsigned long long)rowTiming->workerDoneWaitUs,
               workerTimeout ? 1u : 0u,
               (unsigned long long)rowTiming->workerLateDoneUs,
               (unsigned long)rowTiming->staleResultDiscardedCount,
               (unsigned long)rowHardDeadlineUs,
               (unsigned long long)rowTiming->workerDoneWaitUs,
               (unsigned long long)rowTiming->workerIdleAfterTimeoutUs);
    }

    if (!primaryGood) {
        sensorarrayMeasureFdcDeferDeviceRepairAfterParallel(state,
                                                            row,
                                                            epochId,
                                                            frameSeq,
                                                            SENSORARRAY_FDC_DEV_PRIMARY,
                                                            primarySamples,
                                                            &primaryResult,
                                                            rowTiming,
                                                            "primary_parallel_invalid");
    }
    if (!secondaryGood) {
        sensorarrayMeasureFdcDeferDeviceRepairAfterParallel(state,
                                                            row,
                                                            epochId,
                                                            frameSeq,
                                                            SENSORARRAY_FDC_DEV_SECONDARY,
                                                            secondarySamples,
                                                            &secondaryResult,
                                                            rowTiming,
                                                            "secondary_parallel_invalid");
    }

    rowTiming->waitReadyUs = primaryTiming->waitReadyUs + secondaryTiming->waitReadyUs;
    rowTiming->readUs = primaryTiming->readRawUs + secondaryTiming->readRawUs;
    rowTiming->primaryTotalUs = primaryTiming->deviceUs;
    rowTiming->secondaryTotalUs = secondaryTiming->deviceUs;
    rowTiming->dualBusSkewUs =
        (primaryTiming->deviceUs > secondaryTiming->deviceUs) ?
        (primaryTiming->deviceUs - secondaryTiming->deviceUs) :
        (secondaryTiming->deviceUs - primaryTiming->deviceUs);
    sensorarrayMeasureFdcFinalizeParallelTiming(row,
                                                epochId,
                                                primaryTiming,
                                                secondaryTiming,
                                                rowTiming,
                                                primaryResult.err,
                                                secondaryResult.err);
    (void)primaryGood;
    (void)secondaryGood;
    return ESP_OK;
}

static esp_err_t __attribute__((unused)) sensorarrayMeasureReadFdcRuntimeChannelConfigs(sensorarrayState_t *state,
                                                                                        sensorarrayFdcRuntimeChannelConfig_t configs[2][4])
{
    if (!state || !configs) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(configs, 0, sizeof(sensorarrayFdcRuntimeChannelConfig_t) * 2u * 4u);
    esp_err_t firstErr = ESP_OK;
    for (uint8_t dev = 0u; dev < 2u; ++dev) {
        sensorarrayFdcDeviceState_t *fdcState =
            sensorarrayMeasureGetFdcState(state, (sensorarrayFdcDeviceId_t)dev);
        if (!fdcState || !fdcState->ready || !fdcState->handle) {
            if (firstErr == ESP_OK) {
                firstErr = ESP_ERR_INVALID_STATE;
            }
            continue;
        }

        Fdc2214CapCoreRegs_t coreRegs = {0};
        esp_err_t coreErr = Fdc2214CapReadCoreRegs(fdcState->handle, &coreRegs);
        uint8_t deglitchCode = (coreErr == ESP_OK) ?
            (uint8_t)(coreRegs.MuxConfig & SENSORARRAY_FDC_MUX_DEGLITCH_MASK) :
            (uint8_t)(fdcState->muxConfigReg & SENSORARRAY_FDC_MUX_DEGLITCH_MASK);
        uint32_t effectiveFclkHz =
            (fdcState->refClockKnown && fdcState->refClockHz != 0u) ?
            fdcState->refClockHz :
            sensorarrayMeasureFdcEffectiveFclkHz();

        if (coreErr != ESP_OK && firstErr == ESP_OK) {
            firstErr = coreErr;
        }

        for (uint8_t ch = 0u; ch < 4u; ++ch) {
            sensorarrayFdcRuntimeChannelConfig_t *cfg = &configs[dev][ch];
            cfg->deglitchCode = deglitchCode;
            cfg->effectiveFclkHz = effectiveFclkHz;

            esp_err_t err = Fdc2214CapReadRawRegisters(fdcState->handle,
                                                       sensorarrayMeasureFdcRegForChannel(SENSORARRAY_FDC_REG_RCOUNT_BASE,
                                                                                          (Fdc2214CapChannel_t)ch),
                                                       &cfg->rCount);
            if (err == ESP_OK) {
                err = Fdc2214CapReadRawRegisters(fdcState->handle,
                                                 sensorarrayMeasureFdcRegForChannel(SENSORARRAY_FDC_REG_SETTLECOUNT_BASE,
                                                                                    (Fdc2214CapChannel_t)ch),
                                                 &cfg->settleCount);
            }
            if (err == ESP_OK) {
                err = Fdc2214CapReadRawRegisters(fdcState->handle,
                                                 sensorarrayMeasureFdcRegForChannel(SENSORARRAY_FDC_REG_CLOCK_DIVIDERS_BASE,
                                                                                    (Fdc2214CapChannel_t)ch),
                                                 &cfg->clockDividers);
            }
            if (err == ESP_OK) {
                err = Fdc2214CapReadRawRegisters(fdcState->handle,
                                                 sensorarrayMeasureFdcRegForChannel(SENSORARRAY_FDC_REG_DRIVE_CURRENT_BASE,
                                                                                    (Fdc2214CapChannel_t)ch),
                                                 &cfg->driveCurrent);
            }
            cfg->valid = err == ESP_OK && cfg->clockDividers != 0u && cfg->effectiveFclkHz != 0u;
            if (err != ESP_OK && firstErr == ESP_OK) {
                firstErr = err;
            }
        }
    }
    return firstErr;
}

static esp_err_t __attribute__((unused)) sensorarrayMeasureDiscardFdcAutoscanRow(sensorarrayState_t *state,
                                                                                uint8_t sIndex,
                                                                                sensorarrayFdcDeviceId_t devId,
                                                                                uint8_t discardCount,
                                                                                const char *reason)
{
    if (!state || !sensorarrayMatrixIndexIsValid(sIndex, 1u)) {
        return ESP_ERR_INVALID_ARG;
    }
    if (discardCount == 0u) {
        return ESP_OK;
    }
    sensorarrayFdcDeviceState_t *fdcState = sensorarrayMeasureGetFdcState(state, devId);
    if (!fdcState || !fdcState->ready || !fdcState->handle) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t firstErr = ESP_OK;
#if CONFIG_SENSORARRAY_FDC_VERBOSE_SCAN_LOG
    printf("FDC_DISCARD,scope=device,row=%u,device=%s,count=%u,reason=%s\n",
           (unsigned)sIndex,
           sensorarrayMeasureFdcDeviceName(devId),
           (unsigned)discardCount,
           reason ? reason : SENSORARRAY_NA);
#endif

    for (uint8_t i = 0u; i < discardCount; ++i) {
        uint16_t status = 0u;
        esp_err_t err = sensorarrayMeasureWaitFdcAutoscanFrameReady(fdcState,
                                                                    sIndex,
                                                                    (uint32_t)SENSORARRAY_FDC_AUTOSCAN_READY_TIMEOUT_MS,
                                                                    &status);
        if (err == ESP_OK) {
            sensorarrayFdcAutoscanSamples_t discard = {0};
            err = sensorarrayMeasureReadFdcAutoscan4ch(fdcState, &discard);
        }
        if (err != ESP_OK && firstErr == ESP_OK) {
            firstErr = err;
        }
    }
    return firstErr;
}

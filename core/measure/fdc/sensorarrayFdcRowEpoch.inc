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
    Fdc2214CapStatus_t status = {0};
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
    err = Fdc2214CapReadStatus(fdcState->handle, &status);
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
        Fdc2214CapStatus_t statusAfter = status;
        (void)Fdc2214CapReadRawRegisters(fdcState->handle,
                                         SENSORARRAY_FDC_REG_CONFIG,
                                         &configAfter);
        (void)Fdc2214CapReadRawRegisters(fdcState->handle,
                                         SENSORARRAY_FDC_REG_MUX_CONFIG,
                                         &muxAfter);
        (void)Fdc2214CapReadRawRegisters(fdcState->handle,
                                         SENSORARRAY_FDC_REG_STATUS_CONFIG,
                                         &statusConfigAfter);
        (void)Fdc2214CapReadStatus(fdcState->handle, &statusAfter);
        config = configAfter;
        mux = muxAfter;
        statusConfig = statusConfigAfter;
        status = statusAfter;
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
        printf("FDC_SLEEP_EXIT_VERIFY,row=%u,epoch=%lu,dev=%s,config=0x%04X,sleepBit=%u,mux=0x%04X,autoscan=%u,rrSequence=%s,statusConfig=0x%04X,status=0x%04X,unread=0x%X,err=0x%lx,diagnostic=%s\n",
               (unsigned)row,
               (unsigned long)epochId,
               sensorarrayMeasureFdcDeviceName(devId),
               config,
               sleepBit ? 1u : 0u,
               mux,
               autoscan ? 1u : 0u,
               sensorarrayMeasureFdcRrSequenceName(rrSequence),
               statusConfig,
               status.Raw,
               (unsigned)sensorarrayMeasureFdcUnreadMaskFromStatus(&status),
               (unsigned long)firstErr,
               diagnostic);
    }

    return firstErr;
}

static __attribute__((unused)) const char *sensorarrayMeasureFdcReadyKindName(sensorarrayFdcReadyKind_t kind)
{
    switch (kind) {
    case SENSORARRAY_FDC_READY_EDGE_WAKE:
        return "edge_wake";
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
    timing->statusReadsBeforeIntbCount += ready->statusReadsBeforeIntb;
    timing->statusReadsAfterIntbCount += ready->statusReadsAfterIntb +
                                         ready->statusReadsAfterIntbRecheck;
    timing->statusReadsInFallbackCount += ready->statusReadsInFallback;
    timing->statusReadSuppressedBeforeIntbCount += ready->statusReadsSuppressedBeforeIntb;
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
    if ((ready->kind == SENSORARRAY_FDC_READY_EDGE_WAKE ||
         ready->kind == SENSORARRAY_FDC_READY_POLL_FULL ||
         ready->kind == SENSORARRAY_FDC_READY_AFTER_INTB_RECHECK_FULL) &&
        ready->readyForDataRead) {
        timing->readyFullCount++;
        timing->intbFreshDrdyCount++;
    } else if (ready->kind == SENSORARRAY_FDC_READY_POLL_RECOVERED_AFTER_UNREAD_BEFORE_DRDY &&
               ready->readyForDataRead) {
        timing->readyFullCount++;
    } else if (ready->kind == SENSORARRAY_FDC_READY_POLL_PARTIAL ||
               ready->kind == SENSORARRAY_FDC_READY_TIMEOUT_PARTIAL) {
        timing->readyPartialCount++;
    } else if (ready->kind == SENSORARRAY_FDC_READY_INTB_TIMEOUT ||
               ready->kind == SENSORARRAY_FDC_READY_DRDY_NOT_CLOSED_AFTER_INTB ||
               ready->kind == SENSORARRAY_FDC_READY_STATUS_INCONSISTENT ||
               ready->kind == SENSORARRAY_FDC_READY_TIMEOUT_NONE ||
               ready->kind == SENSORARRAY_FDC_READY_I2C_ERROR) {
        timing->readyNoneCount++;
    }
    if (ready->timeout) {
        timing->intbTimeoutCount++;
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
        return "intb_timeout";
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
    for (uint8_t ch = 0u; state && ch < 4u; ++ch) {
        uint8_t matrixIndex = (uint8_t)sensorarrayMatrixIndex(row, (uint8_t)(dBase + ch));
        (void)sensorarrayMeasureRequestFdcCellRescue(state,
                                                     matrixIndex,
                                                     sensorarrayMeasureFdcWatchdogReasonName(reason));
    }
    printf("RWD,d=%s,r=%u,e=%lu,why=%s,rowBudget=%lu,mul=%lu,hard=%lu,override=%lu,retryMax=%lu\n",
           sensorarrayMeasureFdcDeviceToken(devId),
           (unsigned)row,
           (unsigned long)epochId,
           sensorarrayMeasureFdcWatchdogReasonName(reason),
           (unsigned long)singleRowBudgetUs,
           (unsigned long)CONFIG_SENSORARRAY_FDC_ROW_DEVICE_WATCHDOG_MULTIPLIER,
           (unsigned long)hardTimeoutUs,
           (unsigned long)CONFIG_SENSORARRAY_FDC_ROW_DEVICE_WATCHDOG_OVERRIDE_US,
           (unsigned long)CONFIG_SENSORARRAY_FDC_ROW_DEVICE_RETRY_MAX);
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
        return "WAIT_DRDY";
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
    printf("SR,d=%s,r=%u,e=%lu,b=%lu,a=%lu,ar=%lu,fb=%lu,pd=%lu,supp=%lu,ack=%lu,ib0=%d,ib1=%d\n",
           sensorarrayMeasureFdcDeviceToken(devId),
           (unsigned)row,
           (unsigned long)epochId,
           (unsigned long)ready->statusReadsBeforeIntb,
           (unsigned long)ready->statusReadsAfterIntb,
           (unsigned long)ready->statusReadsAfterIntbRecheck,
           (unsigned long)ready->statusReadsInFallback,
           (unsigned long)ready->statusReadsPollDiag,
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
    }
    ready->pollCount++;
    ready->statusAckCount++;
    ready->intbBeforeStatus = intbBeforeStatus;
    ready->intbAfterStatus = intbAfterStatus;
    ready->err = err;
    if (fallback) {
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
        ready->i2cError = true;
        ready->diagnostic = "i2c_status_error";
        printf("RP,d=%s,r=%u,e=%lu,i=%lu,st=%04X,u=%X,dr=%u,req=%X,k=I2C,ack=1,ib0=%d,ib1=%d,err=0x%lx\n",
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
    if (!ready->readyForDataRead) {
        printf("RP,d=%s,r=%u,e=%lu,i=%lu,st=%04X,u=%X,dr=%u,req=%X,k=%s,ack=1,ib0=%d,ib1=%d\n",
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

static esp_err_t sensorarrayFdcWaitDeviceReady(sensorarrayState_t *state,
                                               sensorarrayFdcDeviceId_t devId,
                                               uint8_t row,
                                               uint32_t epochId,
                                               uint32_t frameSeq,
                                               uint8_t requiredUnreadMask,
                                               uint32_t timeoutUs,
                                               uint32_t estimatedRoundUs,
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
        .requiredUnreadMask = requiredUnreadMask,
        .estimatedRoundUs = estimatedRoundUs,
    };

    sensorarrayFdcWorkerContext_t *ctx = sensorarrayMeasureFdcWorkerContext(devId);
    sensorarrayFdcReadyPolicy_t policy = sensorarrayMeasureFdcReadyPolicyForDevice(devId, ctx);
    bool useIntb = policy != SENSORARRAY_FDC_READY_POLICY_POLL_ONLY;
    bool fallbackAllowed =
        policy == SENSORARRAY_FDC_READY_POLICY_INTB_WITH_POLL_FALLBACK ||
        policy == SENSORARRAY_FDC_READY_POLICY_POLL_ONLY;
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
        ready->initialIntbLevel = gpio_get_level((gpio_num_t)ctx->intbGpio);
    }

    bool logReady = CONFIG_SENSORARRAY_FDC_LOG_READY_EVERY_ROW ||
                    sensorarrayMeasureFdcShouldLogNormalFrame(frameSeq);
    if (logReady) {
        printf("RB,d=%s,r=%u,e=%lu,to=%lu,est=%lu,req=%X,lvl=%d,edge=%lu,mode=%s,intb=%u\n",
               sensorarrayMeasureFdcDeviceToken(devId),
               (unsigned)row,
               (unsigned long)epochId,
               (unsigned long)timeoutUs,
               (unsigned long)estimatedRoundUs,
               (unsigned)requiredUnreadMask,
               ready->initialIntbLevel,
               (unsigned long)edgeStart,
               sensorarrayMeasureFdcReadyPolicyName(policy),
               useIntb ? 1u : 0u);
    }

    if (useIntb && ctx && ctx->intbReady) {
        bool intbSeen = ready->initialIntbLevel == 0;
        uint32_t waitBudgetUs = estimatedRoundUs + (uint32_t)CONFIG_SENSORARRAY_FDC_READY_GUARD_US;
        if (waitBudgetUs == 0u || waitBudgetUs > timeoutUs) {
            waitBudgetUs = timeoutUs;
        }
        if (!intbSeen) {
            if (CONFIG_SENSORARRAY_FDC_SUPPRESS_STATUS_READ_BEFORE_INTB) {
                ready->statusReadsSuppressedBeforeIntb++;
            }
            int64_t intbWaitStartUs = esp_timer_get_time();
            uint32_t notifyCount = ulTaskNotifyTake(pdTRUE,
                                                    sensorarrayMeasureFdcTicksForUs(waitBudgetUs));
            ready->intbWaitUs = (uint32_t)sensorarrayMeasureElapsedUs(intbWaitStartUs);
            ready->finalIntbLevel = gpio_get_level((gpio_num_t)ctx->intbGpio);
            ready->edgeDelta = ctx->edgeCount >= edgeStart ? (ctx->edgeCount - edgeStart) : 0u;
            ready->hadEdge = (notifyCount != 0u || ready->edgeDelta != 0u) ? 1u : 0u;
            intbSeen = ready->hadEdge || ready->finalIntbLevel == 0;
        } else {
            ready->finalIntbLevel = ready->initialIntbLevel;
            ready->hadEdge = 1u;
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
                printf("RR,d=%s,r=%u,e=%lu,src=%s,st=%04X,u=%X,dr=%u,wu=%lu,pc=%lu,k=FULL,ack=%lu,ib0=%d,ib1=%d,err=0\n",
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
                       ready->intbAfterStatus);
                sensorarrayMeasureFdcLogReadyCounts(devId, row, epochId, ready);
                return ESP_OK;
            }
            if (ready->unreadWithoutDataReady &&
                CONFIG_SENSORARRAY_FDC_AFTER_INTB_RECHECK_ENABLE) {
                uint32_t maxRechecks = (uint32_t)CONFIG_SENSORARRAY_FDC_AFTER_INTB_RECHECK_MAX;
                uint32_t intervalUs =
                    (uint32_t)CONFIG_SENSORARRAY_FDC_AFTER_INTB_RECHECK_INTERVAL_US;
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
                        printf("RR,d=%s,r=%u,e=%lu,src=AR,st=%04X,u=%X,dr=%u,wu=%lu,pc=%lu,k=AFTER_INTB_RECHECK_FULL,ack=%lu,ib0=%d,ib1=%d,err=0\n",
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
                               ready->intbAfterStatus);
                        sensorarrayMeasureFdcLogReadyCounts(devId, row, epochId, ready);
                        return ESP_OK;
                    }
                }
                if (policy == SENSORARRAY_FDC_READY_POLICY_INTB_STRICT_LEVEL ||
                    policy == SENSORARRAY_FDC_READY_POLICY_INTB_THEN_STATUS) {
                    ready->kind = SENSORARRAY_FDC_READY_DRDY_NOT_CLOSED_AFTER_INTB;
                    ready->waitUs = (uint32_t)sensorarrayMeasureElapsedUs(startUs);
                    ready->timeout = true;
                    ready->err = ESP_ERR_TIMEOUT;
                    ready->diagnostic = "drdy_not_closed_after_intb";
                    if (ctx) {
                        ctx->waitTask = NULL;
                    }
                    sensorarrayMeasureFdcUpdateReadyTiming(ready, timing, false);
                    printf("RR,d=%s,r=%u,e=%lu,src=AR,st=%04X,u=%X,dr=%u,wu=%lu,pc=%lu,k=DRDY_NOT_CLOSED_AFTER_INTB,ack=%lu,ib0=%d,ib1=%d,err=0x%lx\n",
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
                           (unsigned long)ESP_ERR_TIMEOUT);
                    sensorarrayMeasureFdcLogReadyCounts(devId, row, epochId, ready);
                    return ESP_ERR_TIMEOUT;
                }
            } else if (policy == SENSORARRAY_FDC_READY_POLICY_INTB_STRICT_LEVEL ||
                       policy == SENSORARRAY_FDC_READY_POLICY_INTB_THEN_STATUS) {
                ready->kind = SENSORARRAY_FDC_READY_STATUS_INCONSISTENT;
                ready->waitUs = (uint32_t)sensorarrayMeasureElapsedUs(startUs);
                ready->timeout = true;
                ready->err = ESP_ERR_TIMEOUT;
                ready->diagnostic = "status_inconsistent_after_intb";
                if (ctx) {
                    ctx->waitTask = NULL;
                }
                sensorarrayMeasureFdcUpdateReadyTiming(ready, timing, false);
                printf("RR,d=%s,r=%u,e=%lu,src=%s,st=%04X,u=%X,dr=%u,wu=%lu,pc=%lu,k=STATUS_INCONSISTENT,ack=%lu,ib0=%d,ib1=%d,err=0x%lx\n",
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
                       (unsigned long)ESP_ERR_TIMEOUT);
                sensorarrayMeasureFdcLogReadyCounts(devId, row, epochId, ready);
                return ESP_ERR_TIMEOUT;
            }
        } else {
            ready->diagnostic = (policy == SENSORARRAY_FDC_READY_POLICY_INTB_WITH_POLL_FALLBACK) ?
                "intb_hint_timeout_poll_fallback" :
                "intb_timeout";
            if (ctx) {
                ctx->timeoutCount++;
            }
            if (policy == SENSORARRAY_FDC_READY_POLICY_INTB_STRICT_LEVEL ||
                policy == SENSORARRAY_FDC_READY_POLICY_INTB_THEN_STATUS) {
                ready->kind = SENSORARRAY_FDC_READY_INTB_TIMEOUT;
                ready->waitUs = (uint32_t)sensorarrayMeasureElapsedUs(startUs);
                ready->timeout = true;
                ready->err = ESP_ERR_TIMEOUT;
                if (ctx) {
                    ctx->waitTask = NULL;
                }
                sensorarrayMeasureFdcUpdateReadyTiming(ready, timing, false);
                printf("RR,d=%s,r=%u,e=%lu,src=IT,st=%04X,u=%X,dr=%u,wu=%lu,pc=%lu,k=INTB_TIMEOUT,ack=%lu,ib0=%d,ib1=%d,err=0x%lx\n",
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
                       (unsigned long)ESP_ERR_TIMEOUT);
                sensorarrayMeasureFdcLogReadyCounts(devId, row, epochId, ready);
                return ESP_ERR_TIMEOUT;
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
                printf("RR,d=%s,r=%u,e=%lu,src=%s,st=%04X,u=%X,dr=%u,wu=%lu,pc=%lu,k=%s,ack=%lu,ib0=%d,ib1=%d,err=0\n",
                       sensorarrayMeasureFdcDeviceToken(devId),
                       (unsigned)row,
                       (unsigned long)epochId,
                       useIntb ? "FB" : "PD",
                       ready->statusRaw,
                       (unsigned)ready->unreadMask,
                       (unsigned)ready->drdy,
                       (unsigned long)ready->waitUs,
                       (unsigned long)ready->pollCount,
                       recoveredAfterWaitDrdy ? "RECOVERED_WAIT_DRDY" :
                       (useIntb ? "FB_FULL" : "FULL"),
                       (unsigned long)ready->statusAckCount,
                       ready->intbBeforeStatus,
                       ready->intbAfterStatus);
                sensorarrayMeasureFdcLogReadyCounts(devId, row, epochId, ready);
                return ESP_OK;
            }
        }
        ready->pollFallbackUs += (uint32_t)sensorarrayMeasureElapsedUs(fallbackStartUs);
    }

    if (ctx) {
        ctx->waitTask = NULL;
        ready->finalIntbLevel = ctx->intbReady ? gpio_get_level((gpio_num_t)ctx->intbGpio) : -1;
        ready->edgeDelta = ctx->edgeCount >= edgeStart ? (ctx->edgeCount - edgeStart) : 0u;
        ready->hadEdge = ready->edgeDelta > 0u ? 1u : 0u;
    }
    ready->waitUs = (uint32_t)sensorarrayMeasureElapsedUs(startUs);
    ready->timeoutCount++;
    ready->timeout = true;
    if (bestPartialSeen) {
        ready->partial = true;
        ready->kind = SENSORARRAY_FDC_READY_TIMEOUT_PARTIAL;
        sensorarrayMeasureFdcApplyReadyDecoded(ready, &bestPartial);
    } else {
        ready->kind = SENSORARRAY_FDC_READY_TIMEOUT_NONE;
    }
    ready->ready = false;
    ready->readyForDataRead = false;
    ready->err = ESP_ERR_TIMEOUT;
    ready->diagnostic = ready->unreadWithoutDataReady ?
        "timeout_wait_drdy" :
        (useIntb ?
         (fallbackAttempted ? "poll_fallback_timeout" : "intb_timeout") :
         "poll_fallback_timeout");
    sensorarrayMeasureFdcUpdateReadyTiming(ready, timing, fallbackAttempted);
    printf("RR,d=%s,r=%u,e=%lu,src=%s,st=%04X,u=%X,dr=%u,wu=%lu,pc=%lu,k=%s,ack=%lu,ib0=%d,ib1=%d,err=0x%lx\n",
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
           (unsigned long)ESP_ERR_TIMEOUT);
    sensorarrayMeasureFdcLogReadyCounts(devId, row, epochId, ready);
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
    }
    if (timing) {
        timing->applyDoneUs = (uint64_t)esp_timer_get_time();
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
                                                               const char *readReason)
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
                                        outReady,
                                        timing);
    uint64_t waitUs = sensorarrayMeasureElapsedUs(waitStartUs);
    if (timing) {
        timing->waitReadyUs += waitUs;
    }
    uint8_t freshMask = outReady->readyForDataRead ? (outReady->unreadMask & 0x0Fu) : 0u;
    if (err == ESP_OK && outReady->readyForDataRead) {
        s_fdcNoUnreadConsecutive[(uint8_t)devId] = 0u;
    } else {
        bool noUnreadStatus = outReady->statusRaw == 0u && outReady->unreadMask == 0u &&
                              !outReady->dataReady && !outReady->i2cError;
        if (noUnreadStatus) {
            s_fdcNoUnreadConsecutive[(uint8_t)devId]++;
        } else {
            s_fdcNoUnreadConsecutive[(uint8_t)devId] = 0u;
        }
        if (devId == SENSORARRAY_FDC_DEV_SECONDARY &&
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
            if (resyncErr == ESP_OK && resyncReady.readyForDataRead) {
                *outReady = resyncReady;
                err = ESP_OK;
                freshMask = outReady->unreadMask & 0x0Fu;
            }
        }
    }
    if (err != ESP_OK || !outReady->readyForDataRead ||
        freshMask != SENSORARRAY_FDC_AUTOSCAN_READY_UNREAD_MASK) {
        sensorarrayFdcWatchdogReason_t watchdogReason =
            (outReady->kind == SENSORARRAY_FDC_READY_INTB_TIMEOUT) ?
            SENSORARRAY_FDC_WATCHDOG_INTB_TIMEOUT :
            (outReady->kind == SENSORARRAY_FDC_READY_DRDY_NOT_CLOSED_AFTER_INTB) ?
            SENSORARRAY_FDC_WATCHDOG_DRDY_NOT_CLOSED_AFTER_INTB :
            (outReady->kind == SENSORARRAY_FDC_READY_STATUS_INCONSISTENT) ?
            SENSORARRAY_FDC_WATCHDOG_STATUS_INCONSISTENT :
            SENSORARRAY_FDC_WATCHDOG_INTB_TIMEOUT;
        (void)sensorarrayFdcHandleRowDeviceWatchdog(state,
                                                    devId,
                                                    row,
                                                    epochId,
                                                    watchdogReason,
                                                    NULL,
                                                    timing);
        sensorarrayMeasureMarkFdcNoFreshSamples(outSamples, outReady, outReady->i2cError);
        sensorarrayMeasureBuildFdcRead4Result(row,
                                              epochId,
                                              devId,
                                              outSamples,
                                              outConfigs,
                                              outReady,
                                              (err == ESP_OK) ? ESP_ERR_TIMEOUT : err,
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
               outReady->unreadWithoutDataReady ? "wait_drdy_after_unread_full" :
               (outReady->timeout ? "ready_timeout" : "data_not_ready"),
               (unsigned)row,
               sensorarrayMeasureFdcDeviceName(devId),
               outReady->statusRaw,
               (unsigned)outReady->unreadMask,
               outReady->dataReady ? 1u : 0u,
               outReady->readyForDataRead ? 1u : 0u,
               (unsigned long)((err == ESP_OK) ? ESP_ERR_TIMEOUT : err));
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
                                                           &retryReady,
                                                           timing);
        uint64_t retryWaitUs = sensorarrayMeasureElapsedUs(retryWaitStartUs);
        if (timing) {
            timing->waitReadyUs += retryWaitUs;
        }
        if (retryErr == ESP_OK && retryReady.readyForDataRead) {
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
                                                                "worker");
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
    summary->dataReadUs += primaryTiming->dataReadUs + secondaryTiming->dataReadUs;
    summary->intbWaitOnlyUs += primaryTiming->intbWaitOnlyUs + secondaryTiming->intbWaitOnlyUs;
    summary->statusVerifyAfterIntbUs += primaryTiming->statusVerifyAfterIntbUs +
                                        secondaryTiming->statusVerifyAfterIntbUs;
    summary->pollFallbackUs += primaryTiming->pollFallbackUs + secondaryTiming->pollFallbackUs;
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
    summary->readyFullCount += primaryTiming->readyFullCount + secondaryTiming->readyFullCount;
    summary->readyPartialCount += primaryTiming->readyPartialCount + secondaryTiming->readyPartialCount;
    summary->readyNoneCount += primaryTiming->readyNoneCount + secondaryTiming->readyNoneCount;
    summary->statusReadsBeforeIntbCount += primaryTiming->statusReadsBeforeIntbCount +
                                           secondaryTiming->statusReadsBeforeIntbCount;
    summary->statusReadsAfterIntbCount += primaryTiming->statusReadsAfterIntbCount +
                                          secondaryTiming->statusReadsAfterIntbCount;
    summary->statusReadsInFallbackCount += primaryTiming->statusReadsInFallbackCount +
                                           secondaryTiming->statusReadsInFallbackCount;
    summary->statusReadSuppressedBeforeIntbCount += primaryTiming->statusReadSuppressedBeforeIntbCount +
                                                    secondaryTiming->statusReadSuppressedBeforeIntbCount;
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
    summary->fallbackAttemptCount += primaryTiming->fallbackAttemptCount + secondaryTiming->fallbackAttemptCount;
    summary->fallbackSuccessCount += primaryTiming->fallbackSuccessCount + secondaryTiming->fallbackSuccessCount;
    summary->fallbackPartialCount += primaryTiming->fallbackPartialCount + secondaryTiming->fallbackPartialCount;
    summary->fallbackFailCount += primaryTiming->fallbackFailCount + secondaryTiming->fallbackFailCount;
    summary->deviceFullInvalidCount += primaryTiming->deviceFullInvalidCount +
                                       secondaryTiming->deviceFullInvalidCount;
    summary->workerTimeoutCount += rowTiming->workerTimeoutCount;
    summary->workerLateDoneCount += rowTiming->workerLateDoneCount;
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
                                                                   "serial");
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
                                                                     "serial");
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
                                                  const char *timeoutStage)
{
    if (!trace) {
        return;
    }
    printf("FDC_WORKER_TIMEOUT,seq=%lu,row=%u,epoch=%lu,device=%s,timeoutStage=%s,queueSendUs=%lu,sleepAckWaitUs=%lu,startWaitUs=%lu,doneWaitUs=%lu,workerRunUs=%lu,waitWorkerIdleAfterTimeoutUs=%lu,lateDone=%u,staleResultDiscarded=%u,err=0x%lx\n",
           (unsigned long)trace->frameSeq,
           (unsigned)trace->row,
           (unsigned long)trace->epochId,
           sensorarrayMeasureFdcDeviceName(trace->devId),
           timeoutStage ? timeoutStage : "unknown",
           (unsigned long)trace->queueSendUs,
           (unsigned long)trace->sleepAckWaitUs,
           (unsigned long)trace->startWaitUs,
           (unsigned long)trace->doneWaitUs,
           (unsigned long)trace->workerRunUs,
           (unsigned long)trace->waitWorkerIdleAfterTimeoutUs,
           trace->lateDone ? 1u : 0u,
           trace->staleResultDiscarded ? 1u : 0u,
           (unsigned long)trace->err);
}

static void sensorarrayMeasureFdcLogLateDone(const sensorarrayFdcWorkerTrace_t *trace)
{
    if (!trace) {
        return;
    }
    printf("FDC_WORKER_LATE_DONE,seq=%lu,row=%u,epoch=%lu,device=%s,queueSendUs=%lu,sleepAckWaitUs=%lu,startWaitUs=%lu,doneWaitUs=%lu,workerRunUs=%lu,waitWorkerIdleAfterTimeoutUs=%lu,staleResultDiscarded=%u,err=0x%lx\n",
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
        trace->staleResultDiscarded = true;
        trace->err = ESP_ERR_TIMEOUT;
    }
    (void)reason;
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
    uint8_t dBase = devId == SENSORARRAY_FDC_DEV_SECONDARY ? 5u : 1u;
    for (uint8_t ch = 0u; ch < 4u; ++ch) {
        uint8_t matrixIndex = (uint8_t)sensorarrayMatrixIndex(row, (uint8_t)(dBase + ch));
        (void)sensorarrayMeasureRequestFdcCellRescue(state, matrixIndex, reason ? reason : "parallel_deferred_repair");
    }
    if (rowTiming) {
        rowTiming->inlineRepairSuppressedCount++;
        rowTiming->deferredRepairRequestCount++;
    }
    printf("FDC_DEFERRED_REPAIR,seq=%lu,row=%u,epoch=%lu,dev=%s,reason=%s,status=0x%04X,unread=0x%X,drdy=%u,err=0x%lx,action=mark_invalid_defer_rescue\n",
           (unsigned long)frameSeq,
           (unsigned)row,
           (unsigned long)epochId,
           sensorarrayMeasureFdcDeviceName(devId),
           reason ? reason : "parallel_device_result",
           result ? result->ready.statusRaw : 0u,
           result ? (unsigned)(result->ready.unreadMask & 0x0Fu) : 0u,
           result ? (unsigned)result->ready.drdy : 0u,
           result ? (unsigned long)result->err : (unsigned long)ESP_ERR_INVALID_RESPONSE);
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
    sensorarrayFdcWorkerTrace_t primaryTrace = {
        .frameSeq = frameSeq,
        .row = row,
        .epochId = epochId,
        .devId = SENSORARRAY_FDC_DEV_PRIMARY,
        .err = ESP_ERR_TIMEOUT,
    };
    sensorarrayFdcWorkerTrace_t secondaryTrace = {
        .frameSeq = frameSeq,
        .row = row,
        .epochId = epochId,
        .devId = SENSORARRAY_FDC_DEV_SECONDARY,
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
            sensorarrayMeasureFdcLogWorkerTimeout(&primaryTrace, "queue_send");
        }
        if (secondaryQueued == pdTRUE) {
            (void)xSemaphoreTake(secondaryCtx->sleepAck, syncTicks);
            (void)xSemaphoreTake(secondaryCtx->done, portMAX_DELAY);
        } else {
            secondaryTrace.err = ESP_ERR_TIMEOUT;
            sensorarrayMeasureFdcLogWorkerTimeout(&secondaryTrace, "queue_send");
        }
        printf("FDC_PARALLEL_FALLBACK,reason=queue_failed,row=%u,epoch=%lu,primaryQueued=%u,secondaryQueued=%u,err=0x%lx\n",
               (unsigned)row,
               (unsigned long)epochId,
               primaryQueued == pdTRUE ? 1u : 0u,
               secondaryQueued == pdTRUE ? 1u : 0u,
               (unsigned long)ESP_ERR_TIMEOUT);
        return ESP_ERR_TIMEOUT;
    }

    int64_t sleepAckStartUs = esp_timer_get_time();
    BaseType_t primarySleep = xSemaphoreTake(primaryCtx->sleepAck, syncTicks);
    primaryTrace.sleepAckWaitUs = (uint32_t)sensorarrayMeasureElapsedUs(sleepAckStartUs);
    primaryTrace.sleepAckReceived = primarySleep == pdTRUE;
    sleepAckStartUs = esp_timer_get_time();
    BaseType_t secondarySleep = xSemaphoreTake(secondaryCtx->sleepAck, syncTicks);
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
            sensorarrayMeasureFdcLogWorkerTimeout(&primaryTrace, "sleep_ack");
        }
        if (secondarySleep != pdTRUE || secondaryResult.err == ESP_ERR_TIMEOUT) {
            secondaryTrace.err = ESP_ERR_TIMEOUT;
            sensorarrayMeasureFdcLogWorkerTimeout(&secondaryTrace, "sleep_ack");
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
    TickType_t doneTicks =
        pdMS_TO_TICKS((uint32_t)((CONFIG_SENSORARRAY_FDC_INTB_WAIT_TIMEOUT_US / 1000u) + 250u));
    if (doneTicks == 0) {
        doneTicks = pdMS_TO_TICKS(250u);
    }
    int64_t doneOverallStartUs = startGiveStartUs;
    int64_t doneStartUs = doneOverallStartUs;
    BaseType_t primaryDone = xSemaphoreTake(primaryCtx->done, doneTicks);
    primaryTrace.doneWaitUs = (uint32_t)sensorarrayMeasureElapsedUs(doneStartUs);
    primaryTrace.doneReceived = primaryDone == pdTRUE;
    doneStartUs = esp_timer_get_time();
    BaseType_t secondaryDone = xSemaphoreTake(secondaryCtx->done, doneTicks);
    secondaryTrace.doneWaitUs = (uint32_t)sensorarrayMeasureElapsedUs(doneStartUs);
    secondaryTrace.doneReceived = secondaryDone == pdTRUE;
    rowTiming->workerDoneWaitUs = sensorarrayMeasureElapsedUs(doneOverallStartUs);
    rowTiming->parallelJoinWaitUs = rowTiming->workerDoneWaitUs;
    rowTiming->dualBusWaitUs = sensorarrayMeasureElapsedUs(dualStartUs);
    bool primaryLate = false;
    bool secondaryLate = false;
    if (primaryDone != pdTRUE) {
        primaryResult.err = ESP_ERR_TIMEOUT;
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
        sensorarrayMeasureFdcDiscardWorkerResult(&primaryResult,
                                                 primarySamples,
                                                 &primaryTrace,
                                                 "late_done");
        rowTiming->staleResultDiscardedCount++;
        sensorarrayMeasureFdcLogWorkerTimeout(&primaryTrace, "done_wait");
        if (primaryLate) {
            rowTiming->workerLateDoneCount++;
            rowTiming->workerLateDoneUs += primaryTrace.waitWorkerIdleAfterTimeoutUs;
            sensorarrayMeasureFdcLogLateDone(&primaryTrace);
        }
        if (firstErr == ESP_OK) {
            firstErr = ESP_ERR_TIMEOUT;
        }
    }
    if (secondaryDone != pdTRUE) {
        secondaryResult.err = ESP_ERR_TIMEOUT;
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
        sensorarrayMeasureFdcDiscardWorkerResult(&secondaryResult,
                                                 secondarySamples,
                                                 &secondaryTrace,
                                                 "late_done");
        rowTiming->staleResultDiscardedCount++;
        sensorarrayMeasureFdcLogWorkerTimeout(&secondaryTrace, "done_wait");
        if (secondaryLate) {
            rowTiming->workerLateDoneCount++;
            rowTiming->workerLateDoneUs += secondaryTrace.waitWorkerIdleAfterTimeoutUs;
            sensorarrayMeasureFdcLogLateDone(&secondaryTrace);
        }
        if (firstErr == ESP_OK) {
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
        printf("FDC_STALE_RESULT_DISCARD,seq=%lu,row=%u,epoch=%lu,device=primary,resultRow=%u,resultEpoch=%lu,resultDev=%s\n",
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
        printf("FDC_STALE_RESULT_DISCARD,seq=%lu,row=%u,epoch=%lu,device=secondary,resultRow=%u,resultEpoch=%lu,resultDev=%s\n",
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

    bool primaryGood = !primaryLate &&
                       !primaryStale &&
                       sensorarrayMeasureFdcWorkerResultIsGood(&primaryResult,
                                                               row,
                                                               epochId,
                                                               SENSORARRAY_FDC_DEV_PRIMARY);
    bool secondaryGood = !secondaryLate &&
                         !secondaryStale &&
                         sensorarrayMeasureFdcWorkerResultIsGood(&secondaryResult,
                                                                 row,
                                                                 epochId,
                                                                 SENSORARRAY_FDC_DEV_SECONDARY);
    if (!primaryGood || !secondaryGood) {
        bool workerTimeout = outWorkerTimeout && *outWorkerTimeout;
        printf("FDC_PARALLEL_FALLBACK,reason=%s,row=%u,epoch=%lu,primaryGood=%u,secondaryGood=%u,primaryErr=0x%lx,secondaryErr=0x%lx,primaryLate=%u,secondaryLate=%u,primaryStale=%u,secondaryStale=%u,primaryBus=%d,secondaryBus=%d,sameBus=0,workerDoneWaitUs=%llu,workerIdleAfterTimeoutUs=%llu\n",
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

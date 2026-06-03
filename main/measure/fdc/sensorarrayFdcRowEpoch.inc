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

    return firstErr;
}

static const char *sensorarrayMeasureFdcReadyKindName(sensorarrayFdcReadyKind_t kind)
{
    switch (kind) {
    case SENSORARRAY_FDC_READY_EDGE_WAKE:
        return "edge_wake";
    case SENSORARRAY_FDC_READY_POLL_FULL:
        return "poll_full";
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

static uint32_t sensorarrayMeasureFdcEstimateAutoscanReadyTimeoutUsFromConfigs(
    const sensorarrayFdcRuntimeChannelConfig_t configs[4],
    uint8_t requiredUnreadMask,
    uint32_t *outEstimatedRoundUs)
{
    uint64_t estimatedRoundUs = 0u;

    for (uint8_t ch = 0u; ch < 4u; ++ch) {
        if ((requiredUnreadMask & (uint8_t)(1u << ch)) == 0u) {
            continue;
        }

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
        estimatedRoundUs += (uint64_t)settleUs + convUs + switchUs;
    }

    if (estimatedRoundUs > UINT32_MAX) {
        estimatedRoundUs = UINT32_MAX;
    }
    if (outEstimatedRoundUs) {
        *outEstimatedRoundUs = (uint32_t)estimatedRoundUs;
    }

    uint64_t timeoutUs = (estimatedRoundUs * 2u) + 3000u;
    if (timeoutUs > UINT32_MAX) {
        timeoutUs = UINT32_MAX;
    }
    return sensorarrayMeasureClampU32((uint32_t)timeoutUs, 25000u, 80000u);
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
    if ((ready->unreadMask & requiredUnreadMask) == requiredUnreadMask) {
        return "full_unread";
    }
    if ((ready->unreadMask & requiredUnreadMask) != 0u) {
        return "partial_unread";
    }
    return "no_unread_after_poll";
}

static void sensorarrayMeasureFdcUpdateReadyTiming(const sensorarrayFdcReadyState_t *ready,
                                                   sensorarrayFdcDeviceTiming_t *timing,
                                                   bool fallbackAttempted)
{
    if (!ready || !timing) {
        return;
    }

    timing->readyPollCount += ready->pollCount;
    if (ready->kind == SENSORARRAY_FDC_READY_EDGE_WAKE ||
        ready->kind == SENSORARRAY_FDC_READY_POLL_FULL) {
        timing->readyFullCount++;
        timing->intbFreshDrdyCount++;
    } else if (ready->kind == SENSORARRAY_FDC_READY_POLL_PARTIAL ||
               ready->kind == SENSORARRAY_FDC_READY_TIMEOUT_PARTIAL) {
        timing->readyPartialCount++;
    } else if (ready->kind == SENSORARRAY_FDC_READY_TIMEOUT_NONE ||
               ready->kind == SENSORARRAY_FDC_READY_I2C_ERROR) {
        timing->readyNoneCount++;
    }
    if (ready->timeout) {
        timing->intbTimeoutCount++;
    }
    if (fallbackAttempted) {
        timing->fallbackAttemptCount++;
        if (ready->unreadMask != 0u) {
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

static esp_err_t sensorarrayFdcWaitDeviceReady(sensorarrayState_t *state,
                                               sensorarrayFdcDeviceId_t devId,
                                               uint8_t row,
                                               uint32_t epochId,
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
        .diagnostic = "not_started",
        .requiredUnreadMask = requiredUnreadMask,
        .estimatedRoundUs = estimatedRoundUs,
    };
    sensorarrayFdcWorkerContext_t *ctx = sensorarrayMeasureFdcWorkerContext(devId);
    uint32_t edgeStart = ctx ? ctx->edgeCount : 0u;
    bool bestPartialSeen = false;
    uint16_t bestPartialStatus = 0u;
    uint8_t bestPartialUnread = 0u;
    uint8_t bestPartialDrdy = 0u;
    if (ctx && ctx->intbReady) {
        ctx->currentEpoch = epochId;
        ready->initialIntbLevel = gpio_get_level((gpio_num_t)ctx->intbGpio);
        ctx->waitTask = NULL;
    }
    printf("FDC_READY,row=%u,epoch=%lu,device=%s,stage=begin,timeoutUs=%lu,estimatedRoundUs=%lu,requiredUnread=0x%X,startLevel=%d,startEdge=%lu,readyMode=%s,intbHint=%s\n",
           (unsigned)row,
           (unsigned long)epochId,
           sensorarrayMeasureFdcDeviceName(devId),
           (unsigned long)timeoutUs,
           (unsigned long)estimatedRoundUs,
           (unsigned)requiredUnreadMask,
           ready->initialIntbLevel,
           (unsigned long)edgeStart,
           SENSORARRAY_FDC_READY_MODE_NAME,
           SENSORARRAY_FDC_INTB_HINT_NAME);

    int64_t startUs = esp_timer_get_time();
    int64_t deadlineUs = startUs + (int64_t)timeoutUs;
    for (;;) {
        Fdc2214CapStatus_t status = {0};
        int64_t statusStartUs = esp_timer_get_time();
        esp_err_t err = Fdc2214CapReadStatus(fdcState->handle, &status);
        if (timing) {
            timing->statusReadUs += sensorarrayMeasureElapsedUs(statusStartUs);
        }
        ready->pollCount++;
        ready->err = err;
        if (err != ESP_OK) {
            ready->kind = SENSORARRAY_FDC_READY_I2C_ERROR;
            ready->i2cError = true;
            ready->waitUs = (uint32_t)sensorarrayMeasureElapsedUs(startUs);
            ready->diagnostic = sensorarrayMeasureFdcReadyDiagnosticName(ready, requiredUnreadMask);
            sensorarrayMeasureFdcUpdateReadyTiming(ready, timing, false);
            if (ctx) {
                ctx->waitTask = NULL;
            }
            printf("FDC_READY,row=%u,epoch=%lu,device=%s,stage=i2c_error,status=0x%04X,unread=0x%X,drdy=%u,err=0x%lx,errName=%s,pollCount=%lu,waitUs=%lu,timeoutUs=%lu,estimatedRoundUs=%lu,readyMode=%s,intbHint=%s,result=i2c_error,diagnostic=%s\n",
                   (unsigned)row,
                   (unsigned long)epochId,
                   sensorarrayMeasureFdcDeviceName(devId),
                   ready->statusRaw,
                   (unsigned)ready->unreadMask,
                   (unsigned)ready->drdy,
                   (unsigned long)err,
                   sensorarrayMeasureEspErrName(err),
                   (unsigned long)ready->pollCount,
                   (unsigned long)ready->waitUs,
                   (unsigned long)timeoutUs,
                   (unsigned long)estimatedRoundUs,
                   SENSORARRAY_FDC_READY_MODE_NAME,
                   SENSORARRAY_FDC_INTB_HINT_NAME,
                   ready->diagnostic);
            return err;
        }
        ready->statusRaw = status.Raw;
        ready->dataReady = status.DataReady;
        ready->drdy = status.DataReady ? 1u : 0u;
        ready->unreadMask = sensorarrayMeasureFdcUnreadMaskFromStatus(&status);
        if (ready->unreadMask != 0u) {
            bestPartialSeen = true;
            bestPartialStatus = status.Raw;
            bestPartialUnread = ready->unreadMask;
            bestPartialDrdy = ready->drdy;
        }
        if ((ready->unreadMask & requiredUnreadMask) == requiredUnreadMask) {
            ready->ready = true;
            ready->kind = SENSORARRAY_FDC_READY_POLL_FULL;
            ready->waitUs = (uint32_t)sensorarrayMeasureElapsedUs(startUs);
            ready->diagnostic = sensorarrayMeasureFdcReadyDiagnosticName(ready, requiredUnreadMask);
            if (ctx) {
                uint32_t staleEdges = edgeStart <= ctx->edgeCount ? (ctx->edgeCount - edgeStart) : 0u;
                ready->edgeDelta = staleEdges;
                ready->hadEdge = staleEdges > 0u ? 1u : 0u;
                if (staleEdges > 0u) {
                    ctx->staleBeforeClearCount += staleEdges;
                }
                ctx->freshDrdyCount++;
                ctx->waitTask = NULL;
            }
            sensorarrayMeasureFdcUpdateReadyTiming(ready, timing, false);
#if CONFIG_SENSORARRAY_FDC_INTB_DEBUG_LOG
            printf("FDC_INTB_READY,row=%u,epoch=%lu,device=%s,status=0x%04X,unread=0x%X,drdy=%u,edge=%u\n",
                   (unsigned)row,
                   (unsigned long)epochId,
                   sensorarrayMeasureFdcDeviceName(devId),
                   ready->statusRaw,
                   (unsigned)ready->unreadMask,
                   ready->dataReady ? 1u : 0u,
                   ready->hadEdge ? 1u : 0u);
#endif
            printf("FDC_READY,row=%u,epoch=%lu,device=%s,stage=full,status=0x%04X,unread=0x%X,drdy=%u,lastStatus=0x%04X,lastUnread=0x%X,waitUs=%lu,timeoutUs=%lu,estimatedRoundUs=%lu,pollCount=%lu,readyMode=%s,intbHint=%s,kind=%s,edgeDelta=%lu,result=full,diagnostic=%s\n",
                   (unsigned)row,
                   (unsigned long)epochId,
                   sensorarrayMeasureFdcDeviceName(devId),
                   ready->statusRaw,
                   (unsigned)ready->unreadMask,
                   (unsigned)ready->drdy,
                   ready->statusRaw,
                   (unsigned)ready->unreadMask,
                   (unsigned long)ready->waitUs,
                   (unsigned long)timeoutUs,
                   (unsigned long)estimatedRoundUs,
                   (unsigned long)ready->pollCount,
                   SENSORARRAY_FDC_READY_MODE_NAME,
                   SENSORARRAY_FDC_INTB_HINT_NAME,
                   sensorarrayMeasureFdcReadyKindName(ready->kind),
                   (unsigned long)ready->edgeDelta,
                   ready->diagnostic);
            return ESP_OK;
        }
        if (esp_timer_get_time() >= deadlineUs) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(1u));
    }

    if (ctx) {
        ctx->timeoutCount++;
        ctx->waitTask = NULL;
        ready->finalIntbLevel = ctx->intbReady ? gpio_get_level((gpio_num_t)ctx->intbGpio) : -1;
        ready->edgeDelta = ctx->edgeCount >= edgeStart ? (ctx->edgeCount - edgeStart) : 0u;
        ready->hadEdge = ready->edgeDelta > 0u ? 1u : 0u;
    }
    ready->waitUs = (uint32_t)sensorarrayMeasureElapsedUs(startUs);
    ready->timeoutCount++;
    ready->timeout = true;
    if (bestPartialSeen) {
        ready->ready = false;
        ready->partial = true;
        ready->kind = SENSORARRAY_FDC_READY_TIMEOUT_PARTIAL;
        ready->statusRaw = bestPartialStatus;
        ready->unreadMask = bestPartialUnread;
        ready->drdy = bestPartialDrdy;
        ready->dataReady = bestPartialDrdy != 0u;
        ready->err = ESP_ERR_TIMEOUT;
        ready->diagnostic = sensorarrayMeasureFdcReadyDiagnosticName(ready, requiredUnreadMask);
        sensorarrayMeasureFdcUpdateReadyTiming(ready, timing, false);
        printf("FDC_READY,row=%u,epoch=%lu,device=%s,stage=partial_timeout,status=0x%04X,unread=0x%X,drdy=%u,lastStatus=0x%04X,lastUnread=0x%X,waitUs=%lu,timeoutUs=%lu,estimatedRoundUs=%lu,pollCount=%lu,readyMode=%s,intbHint=%s,kind=partial_timeout,result=timeout,diagnostic=%s\n",
               (unsigned)row,
               (unsigned long)epochId,
               sensorarrayMeasureFdcDeviceName(devId),
               ready->statusRaw,
               (unsigned)ready->unreadMask,
               ready->dataReady ? 1u : 0u,
               ready->statusRaw,
               (unsigned)ready->unreadMask,
               (unsigned long)ready->waitUs,
               (unsigned long)timeoutUs,
               (unsigned long)estimatedRoundUs,
               (unsigned long)ready->pollCount,
               SENSORARRAY_FDC_READY_MODE_NAME,
               SENSORARRAY_FDC_INTB_HINT_NAME,
               ready->diagnostic);
        printf("FDC_INTB_TIMEOUT,row=%u,epoch=%lu,device=%s,status=0x%04X,unread=0x%X,drdy=%u,timeoutUs=%lu,pollCount=%lu,readyMode=%s,intbHint=%s,kind=partial_timeout\n",
               (unsigned)row,
               (unsigned long)epochId,
               sensorarrayMeasureFdcDeviceName(devId),
               ready->statusRaw,
               (unsigned)ready->unreadMask,
               ready->dataReady ? 1u : 0u,
               (unsigned long)timeoutUs,
               (unsigned long)ready->pollCount,
               SENSORARRAY_FDC_READY_MODE_NAME,
               SENSORARRAY_FDC_INTB_HINT_NAME);
        return ESP_ERR_TIMEOUT;
    }

    ready->kind = SENSORARRAY_FDC_READY_TIMEOUT_NONE;
    ready->err = ESP_ERR_TIMEOUT;
    ready->diagnostic = sensorarrayMeasureFdcReadyDiagnosticName(ready, requiredUnreadMask);
    sensorarrayMeasureFdcUpdateReadyTiming(ready, timing, false);
    if (ready->pollCount == 0u) {
        printf("BUG_READY_POLL_NOT_ENTERED,row=%u,epoch=%lu,device=%s,timeoutUs=%lu\n",
               (unsigned)row,
               (unsigned long)epochId,
               sensorarrayMeasureFdcDeviceName(devId),
               (unsigned long)timeoutUs);
    }
    printf("FDC_READY,row=%u,epoch=%lu,device=%s,stage=timeout_none,status=0x%04X,unread=0x%X,drdy=%u,lastStatus=0x%04X,lastUnread=0x%X,waitUs=%lu,timeoutUs=%lu,estimatedRoundUs=%lu,pollCount=%lu,readyMode=%s,intbHint=%s,kind=timeout_none,result=timeout,diagnostic=%s\n",
           (unsigned)row,
           (unsigned long)epochId,
           sensorarrayMeasureFdcDeviceName(devId),
           ready->statusRaw,
           (unsigned)ready->unreadMask,
           ready->dataReady ? 1u : 0u,
           ready->statusRaw,
           (unsigned)ready->unreadMask,
           (unsigned long)ready->waitUs,
           (unsigned long)timeoutUs,
           (unsigned long)estimatedRoundUs,
           (unsigned long)ready->pollCount,
           SENSORARRAY_FDC_READY_MODE_NAME,
           SENSORARRAY_FDC_INTB_HINT_NAME,
           ready->diagnostic);
    printf("FDC_INTB_TIMEOUT,row=%u,epoch=%lu,device=%s,status=0x%04X,unread=0x%X,drdy=%u,timeoutUs=%lu,pollCount=%lu,readyMode=%s,intbHint=%s,kind=timeout_none\n",
           (unsigned)row,
           (unsigned long)epochId,
           sensorarrayMeasureFdcDeviceName(devId),
           ready->statusRaw,
           (unsigned)ready->unreadMask,
           ready->dataReady ? 1u : 0u,
           (unsigned long)timeoutUs,
           (unsigned long)ready->pollCount,
           SENSORARRAY_FDC_READY_MODE_NAME,
           SENSORARRAY_FDC_INTB_HINT_NAME);
    return ESP_ERR_TIMEOUT;
}

static esp_err_t sensorarrayMeasureFdcDeviceLevelResync(sensorarrayState_t *state,
                                                        uint8_t row,
                                                        uint32_t epochId,
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
                                            SENSORARRAY_FDC_AUTOSCAN_READY_UNREAD_MASK,
                                            timeoutUs,
                                            estimatedRoundUs,
                                            readyAfter,
                                            timing);
        uint64_t waitUs = sensorarrayMeasureElapsedUs(waitStartUs);
        if (timing) {
            timing->waitReadyUs += waitUs;
            timing->sleepExitToIntbUs += waitUs;
        }
        sensorarrayMeasureRecordFirstErr(err, &firstErr);
    }

    bool idOk = idMfg == SENSORARRAY_FDC_EXPECTED_MANUFACTURER_ID &&
                idDev == SENSORARRAY_FDC_EXPECTED_DEVICE_ID;
    bool ok = firstErr == ESP_OK &&
              (readyAfter->unreadMask & 0x0Fu) == SENSORARRAY_FDC_AUTOSCAN_READY_UNREAD_MASK;
    printf("FDC_DEVICE_RESYNC,dev=%s,reason=no_unread,row=%u,epoch=%lu,statusBefore=0x%04X,configBefore=0x%04X,muxBefore=0x%04X,errorConfigBefore=0x%04X,idMfg=0x%04X,idDev=0x%04X,idOk=%u,statusAfter=0x%04X,unreadAfter=0x%X,ok=%u,err=0x%lx,errName=%s\n",
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
           ok ? 1u : 0u,
           (unsigned long)firstErr,
           sensorarrayMeasureEspErrName(firstErr));
    return firstErr;
}

static esp_err_t sensorarrayMeasureFdcRunDeviceEpochAfterSleep(sensorarrayState_t *state,
                                                               uint8_t row,
                                                               uint32_t epochId,
                                                               sensorarrayFdcDeviceId_t devId,
                                                               sensorarrayFdcAutoscanSamples_t *outSamples,
                                                               sensorarrayFdcRuntimeChannelConfig_t outConfigs[4],
                                                               sensorarrayFdcReadyState_t *outReady,
                                                               sensorarrayFdcDeviceRead4Result_t *outRead4,
                                                               sensorarrayFdcDeviceTiming_t *timing)
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
    int64_t applyStartUs = esp_timer_get_time();
    printf("FDC_ROW_EPOCH,stage=apply_cache,row=%u,epoch=%lu,dev=%s,force=1\n",
           (unsigned)row,
           (unsigned long)epochId,
           sensorarrayMeasureFdcDeviceName(devId));
    esp_err_t err = sensorarrayMeasureApplyFdcCachedRowConfig(state,
                                                              row,
                                                              devId,
                                                              "matrix_row_epoch_sleep",
                                                              true,
                                                              timing);
    uint64_t applyUs = sensorarrayMeasureElapsedUs(applyStartUs);
    if (timing) {
        timing->applyUs += 0u;
    }
    sensorarrayMeasureRuntimeConfigsFromApplied(state, devId, outConfigs);
    if (err != ESP_OK) {
        return err;
    }

    int64_t exitStartUs = esp_timer_get_time();
    printf("FDC_ROW_EPOCH,stage=sleep_exit,row=%u,epoch=%lu,dev=%s\n",
           (unsigned)row,
           (unsigned long)epochId,
           sensorarrayMeasureFdcDeviceName(devId));
    err = sensorarrayMeasureFdcSetSleepMode(fdcState, false, timing);
    uint64_t exitUs = sensorarrayMeasureElapsedUs(exitStartUs);
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
                                        SENSORARRAY_FDC_AUTOSCAN_READY_UNREAD_MASK,
                                        timeoutUs,
                                        estimatedRoundUs,
                                        outReady,
                                        timing);
    uint64_t waitUs = sensorarrayMeasureElapsedUs(waitStartUs);
    if (timing) {
        timing->waitReadyUs += waitUs;
        timing->sleepExitToIntbUs += waitUs;
    }
    uint8_t freshMask = outReady->unreadMask & 0x0Fu;
    if (err == ESP_OK && freshMask == SENSORARRAY_FDC_AUTOSCAN_READY_UNREAD_MASK) {
        s_fdcNoUnreadConsecutive[(uint8_t)devId] = 0u;
    } else {
        bool noUnreadStatus = outReady->statusRaw == 0u && freshMask == 0u && !outReady->i2cError;
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
                                                                         devId,
                                                                         outReady,
                                                                         &resyncReady,
                                                                         timing);
            s_fdcNoUnreadConsecutive[(uint8_t)devId] = 0u;
            if (resyncErr == ESP_OK &&
                (resyncReady.unreadMask & 0x0Fu) == SENSORARRAY_FDC_AUTOSCAN_READY_UNREAD_MASK) {
                *outReady = resyncReady;
                err = ESP_OK;
                freshMask = outReady->unreadMask & 0x0Fu;
            }
        }
    }
    if (err != ESP_OK || freshMask != SENSORARRAY_FDC_AUTOSCAN_READY_UNREAD_MASK) {
        sensorarrayMeasureMarkFdcNoFreshSamples(outSamples, outReady, outReady->i2cError);
        sensorarrayMeasureBuildFdcRead4Result(row,
                                              epochId,
                                              devId,
                                              outSamples,
                                              outConfigs,
                                              outReady,
                                              (err == ESP_OK) ? ESP_ERR_TIMEOUT : err,
                                              0u,
                                              outRead4);
        if (timing) {
            timing->deviceUs = sensorarrayMeasureElapsedUs(jobStartUs);
            timing->deviceFullInvalidCount++;
        }
        printf("FDC_FALLBACK,stage=poll_failed,row=%u,device=%s,status=0x%04X,unread=0x%X,err=0x%lx\n",
               (unsigned)row,
               sensorarrayMeasureFdcDeviceName(devId),
               outReady->statusRaw,
               (unsigned)outReady->unreadMask,
               (unsigned long)((err == ESP_OK) ? ESP_ERR_TIMEOUT : err));
        return ESP_OK;
    }

    int64_t readStartUs = esp_timer_get_time();
    err = sensorarrayMeasureReadFdcAutoscan4chMasked(fdcState, freshMask, outSamples);
    uint64_t readUs = sensorarrayMeasureElapsedUs(readStartUs);
    if (timing) {
        timing->readRawUs += readUs;
        timing->dataReadUs += readUs;
        if (readUs > timing->maxI2cReadUs) {
            timing->maxI2cReadUs = readUs;
        }
        timing->deviceUs = sensorarrayMeasureElapsedUs(jobStartUs);
        (void)applyUs;
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
                                          outRead4);
    if (timing && outRead4->validMask4 == 0u) {
        timing->deviceFullInvalidCount++;
    }
    bool fallbackRelevant = outRead4->timeout ||
                            outRead4->partial ||
                            outRead4->readyKind == SENSORARRAY_FDC_READY_POLL_FULL ||
                            outRead4->readyKind == SENSORARRAY_FDC_READY_TIMEOUT_PARTIAL;
    if (fallbackRelevant && outRead4->validMask4 != 0u) {
        printf("FDC_FALLBACK,stage=%s,row=%u,device=%s,validMask=0x%X,unread=0x%X,status=0x%04X\n",
               outRead4->partial ? "poll_partial" : "poll_success",
               (unsigned)row,
               sensorarrayMeasureFdcDeviceName(devId),
               (unsigned)outRead4->validMask4,
               (unsigned)outRead4->unreadMask4,
               outRead4->status);
    } else if (outRead4->validMask4 == 0u) {
        printf("FDC_FALLBACK,stage=poll_failed,row=%u,device=%s,status=0x%04X,unread=0x%X,err=0x%lx\n",
               (unsigned)row,
               sensorarrayMeasureFdcDeviceName(devId),
               outRead4->status,
               (unsigned)outRead4->unreadMask4,
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
        sensorarrayFdcDeviceState_t *fdcState = sensorarrayMeasureGetFdcState(job.state, ctx->devId);
        esp_err_t err = ESP_ERR_INVALID_STATE;
        if (fdcState && fdcState->ready && fdcState->handle) {
            printf("FDC_ROW_EPOCH,stage=sleep_enter,row=%u,epoch=%lu,dev=%s\n",
                   (unsigned)job.row,
                   (unsigned long)job.epochId,
                   sensorarrayMeasureFdcDeviceName(ctx->devId));
            err = sensorarrayMeasureFdcSetSleepMode(fdcState, true, job.timing);
        }
        job.result->err = err;
        xSemaphoreGive(ctx->sleepAck);

        TickType_t waitTicks = pdMS_TO_TICKS((uint32_t)CONFIG_SENSORARRAY_FDC_WORKER_SYNC_TIMEOUT_MS);
        if (waitTicks == 0) {
            waitTicks = 1;
        }
        if (err == ESP_OK && xSemaphoreTake(ctx->start, waitTicks) == pdTRUE) {
            err = sensorarrayMeasureFdcRunDeviceEpochAfterSleep(job.state,
                                                                job.row,
                                                                job.epochId,
                                                                ctx->devId,
                                                                job.outSamples,
                                                                job.outConfigs,
                                                                &job.result->ready,
                                                                &job.result->read4,
                                                                job.timing);
            job.result->err = err;
        } else if (err == ESP_OK) {
            job.result->err = ESP_ERR_TIMEOUT;
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
    printf("FDC_WORKER,stage=init,status=parallel_ready,primaryBus=0,secondaryBus=1,readyMode=%s,intbHint=%s,configIntbDisabled=1\n",
           SENSORARRAY_FDC_READY_MODE_NAME,
           SENSORARRAY_FDC_INTB_HINT_NAME);
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
    summary->primaryJobUs += primaryTiming->deviceUs;
    summary->secondaryJobUs += secondaryTiming->deviceUs;
    summary->dualBusWaitUs += rowTiming->dualBusWaitUs;
    summary->dualBusSkewUs += rowTiming->dualBusSkewUs;

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
    summary->fallbackAttemptCount += primaryTiming->fallbackAttemptCount + secondaryTiming->fallbackAttemptCount;
    summary->fallbackSuccessCount += primaryTiming->fallbackSuccessCount + secondaryTiming->fallbackSuccessCount;
    summary->fallbackPartialCount += primaryTiming->fallbackPartialCount + secondaryTiming->fallbackPartialCount;
    summary->fallbackFailCount += primaryTiming->fallbackFailCount + secondaryTiming->fallbackFailCount;
    summary->deviceFullInvalidCount += primaryTiming->deviceFullInvalidCount +
                                       secondaryTiming->deviceFullInvalidCount;
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
                                                             sensorarrayFdcRowTiming_t *rowTiming)
{
    (void)state;
    printf("FDC_ROW_EPOCH,stage=row_switch,row=%u,epoch=%lu\n",
           (unsigned)row,
           (unsigned long)epochId);
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
                                                               sensorarrayFdcAutoscanSamples_t *primarySamples,
                                                               sensorarrayFdcAutoscanSamples_t *secondarySamples,
                                                               sensorarrayFdcRuntimeChannelConfig_t runtimeConfigs[2][4],
                                                               sensorarrayFdcDeviceTiming_t *primaryTiming,
                                                               sensorarrayFdcDeviceTiming_t *secondaryTiming,
                                                               sensorarrayFdcRowTiming_t *rowTiming)
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
    printf("FDC_ROW_EPOCH,stage=sleep_enter,row=%u,epoch=%lu,dev=primary\n",
           (unsigned)row,
           (unsigned long)epochId);
    esp_err_t primaryErr = sensorarrayMeasureFdcSetSleepMode(&state->fdcPrimary, true, primaryTiming);
    esp_err_t secondaryErr = ESP_ERR_NOT_SUPPORTED;
    if (secondaryAvailable) {
        printf("FDC_ROW_EPOCH,stage=sleep_enter,row=%u,epoch=%lu,dev=secondary\n",
               (unsigned)row,
               (unsigned long)epochId);
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

    esp_err_t rowErr = sensorarrayMeasureSelectFdcRowWhileSleeping(state, row, epochId, rowTiming);
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
                                                                   SENSORARRAY_FDC_DEV_PRIMARY,
                                                                   primarySamples,
                                                                   runtimeConfigs[SENSORARRAY_FDC_DEV_PRIMARY],
                                                                   &primaryReady,
                                                                   &primaryRead4,
                                                                   primaryTiming);
        if (primaryErr != ESP_OK && firstErr == ESP_OK) {
            firstErr = primaryErr;
        }
    }
    if (secondaryAvailable && secondaryErr == ESP_OK) {
        secondaryErr = sensorarrayMeasureFdcRunDeviceEpochAfterSleep(state,
                                                                     row,
                                                                     epochId,
                                                                     SENSORARRAY_FDC_DEV_SECONDARY,
                                                                     secondarySamples,
                                                                     runtimeConfigs[SENSORARRAY_FDC_DEV_SECONDARY],
                                                                     &secondaryReady,
                                                                     &secondaryRead4,
                                                                     secondaryTiming);
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

static esp_err_t sensorarrayMeasureReadFdcMatrixRowParallelEpoch(sensorarrayState_t *state,
                                                                 uint8_t row,
                                                                 uint32_t epochId,
                                                                 sensorarrayFdcAutoscanSamples_t *primarySamples,
                                                                 sensorarrayFdcAutoscanSamples_t *secondarySamples,
                                                                 sensorarrayFdcRuntimeChannelConfig_t runtimeConfigs[2][4],
                                                                 sensorarrayFdcDeviceTiming_t *primaryTiming,
                                                                 sensorarrayFdcDeviceTiming_t *secondaryTiming,
                                                                 sensorarrayFdcRowTiming_t *rowTiming)
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

    sensorarrayFdcWorkerResult_t primaryResult = {0};
    sensorarrayFdcWorkerResult_t secondaryResult = {0};
    sensorarrayFdcWorkerJob_t primaryJob = {
        .state = state,
        .row = row,
        .epochId = epochId,
        .outSamples = primarySamples,
        .outConfigs = runtimeConfigs[SENSORARRAY_FDC_DEV_PRIMARY],
        .timing = primaryTiming,
        .result = &primaryResult,
    };
    sensorarrayFdcWorkerJob_t secondaryJob = {
        .state = state,
        .row = row,
        .epochId = epochId,
        .outSamples = secondarySamples,
        .outConfigs = runtimeConfigs[SENSORARRAY_FDC_DEV_SECONDARY],
        .timing = secondaryTiming,
        .result = &secondaryResult,
    };

    TickType_t syncTicks = pdMS_TO_TICKS((uint32_t)CONFIG_SENSORARRAY_FDC_WORKER_SYNC_TIMEOUT_MS);
    if (syncTicks == 0) {
        syncTicks = 1;
    }
    int64_t sleepStartUs = esp_timer_get_time();
    BaseType_t primaryQueued = xQueueSend(primaryCtx->queue, &primaryJob, syncTicks);
    BaseType_t secondaryQueued = xQueueSend(secondaryCtx->queue, &secondaryJob, syncTicks);
    if (primaryQueued != pdTRUE || secondaryQueued != pdTRUE) {
        if (primaryQueued == pdTRUE) {
            (void)xSemaphoreTake(primaryCtx->sleepAck, syncTicks);
            xSemaphoreGive(primaryCtx->start);
            (void)xSemaphoreTake(primaryCtx->done, syncTicks);
        }
        if (secondaryQueued == pdTRUE) {
            (void)xSemaphoreTake(secondaryCtx->sleepAck, syncTicks);
            xSemaphoreGive(secondaryCtx->start);
            (void)xSemaphoreTake(secondaryCtx->done, syncTicks);
        }
        printf("FDC_PARALLEL_FALLBACK,reason=queue_failed,row=%u,epoch=%lu,primaryQueued=%u,secondaryQueued=%u,err=0x%lx\n",
               (unsigned)row,
               (unsigned long)epochId,
               primaryQueued == pdTRUE ? 1u : 0u,
               secondaryQueued == pdTRUE ? 1u : 0u,
               (unsigned long)ESP_ERR_TIMEOUT);
        return ESP_ERR_TIMEOUT;
    }

    BaseType_t primarySleep = xSemaphoreTake(primaryCtx->sleepAck, syncTicks);
    BaseType_t secondarySleep = xSemaphoreTake(secondaryCtx->sleepAck, syncTicks);
    rowTiming->sleepBeforeRowSwitchUs = sensorarrayMeasureElapsedUs(sleepStartUs);
    esp_err_t firstErr = ESP_OK;
    if (primarySleep != pdTRUE) {
        primaryResult.err = ESP_ERR_TIMEOUT;
        firstErr = ESP_ERR_TIMEOUT;
    } else if (primaryResult.err != ESP_OK) {
        firstErr = primaryResult.err;
    }
    if (secondarySleep != pdTRUE) {
        secondaryResult.err = ESP_ERR_TIMEOUT;
        if (firstErr == ESP_OK) {
            firstErr = ESP_ERR_TIMEOUT;
        }
    } else if (secondaryResult.err != ESP_OK && firstErr == ESP_OK) {
        firstErr = secondaryResult.err;
    }

    sensorarrayMeasureFdcPrepareIntbEpoch(primaryCtx, epochId);
    sensorarrayMeasureFdcPrepareIntbEpoch(secondaryCtx, epochId);

    esp_err_t rowErr = sensorarrayMeasureSelectFdcRowWhileSleeping(state, row, epochId, rowTiming);
    if (rowErr != ESP_OK && firstErr == ESP_OK) {
        firstErr = rowErr;
    }

    int64_t dualStartUs = esp_timer_get_time();
    xSemaphoreGive(primaryCtx->start);
    xSemaphoreGive(secondaryCtx->start);
    TickType_t doneTicks =
        pdMS_TO_TICKS((uint32_t)((CONFIG_SENSORARRAY_FDC_INTB_WAIT_TIMEOUT_US / 1000u) + 250u));
    if (doneTicks == 0) {
        doneTicks = pdMS_TO_TICKS(250u);
    }
    BaseType_t primaryDone = xSemaphoreTake(primaryCtx->done, doneTicks);
    BaseType_t secondaryDone = xSemaphoreTake(secondaryCtx->done, doneTicks);
    rowTiming->dualBusWaitUs = sensorarrayMeasureElapsedUs(dualStartUs);
    if (primaryDone != pdTRUE) {
        primaryResult.err = ESP_ERR_TIMEOUT;
        primaryTiming->timeoutCount++;
        printf("FDC_WORKER,row=%u,epoch=%lu,device=primary,status=timeout\n",
               (unsigned)row,
               (unsigned long)epochId);
        if (firstErr == ESP_OK) {
            firstErr = ESP_ERR_TIMEOUT;
        }
    }
    if (secondaryDone != pdTRUE) {
        secondaryResult.err = ESP_ERR_TIMEOUT;
        secondaryTiming->timeoutCount++;
        printf("FDC_WORKER,row=%u,epoch=%lu,device=secondary,status=timeout\n",
               (unsigned)row,
               (unsigned long)epochId);
        if (firstErr == ESP_OK) {
            firstErr = ESP_ERR_TIMEOUT;
        }
    }
    if (primaryDone == pdTRUE &&
        (primaryResult.epochId != epochId ||
         primaryResult.row != row ||
         primaryResult.devId != SENSORARRAY_FDC_DEV_PRIMARY)) {
        printf("FDC_WORKER_RESULT,stage=stale_reject,row=%u,epoch=%lu,device=primary,resultRow=%u,resultEpoch=%lu,resultDev=%s\n",
               (unsigned)row,
               (unsigned long)epochId,
               (unsigned)primaryResult.row,
               (unsigned long)primaryResult.epochId,
               sensorarrayMeasureFdcDeviceName(primaryResult.devId));
        sensorarrayMeasureMarkFdcNoFreshSamples(primarySamples, &primaryResult.ready, false);
        primaryResult.read4.staleRejected = true;
        primaryResult.read4.validMask4 = 0u;
        primaryResult.read4.errorMask4 = 0x0Fu;
    }
    if (secondaryDone == pdTRUE &&
        (secondaryResult.epochId != epochId ||
         secondaryResult.row != row ||
         secondaryResult.devId != SENSORARRAY_FDC_DEV_SECONDARY)) {
        printf("FDC_WORKER_RESULT,stage=stale_reject,row=%u,epoch=%lu,device=secondary,resultRow=%u,resultEpoch=%lu,resultDev=%s\n",
               (unsigned)row,
               (unsigned long)epochId,
               (unsigned)secondaryResult.row,
               (unsigned long)secondaryResult.epochId,
               sensorarrayMeasureFdcDeviceName(secondaryResult.devId));
        sensorarrayMeasureMarkFdcNoFreshSamples(secondarySamples, &secondaryResult.ready, false);
        secondaryResult.read4.staleRejected = true;
        secondaryResult.read4.validMask4 = 0u;
        secondaryResult.read4.errorMask4 = 0x0Fu;
    }
    if (primaryResult.err != ESP_OK && firstErr == ESP_OK) {
        firstErr = primaryResult.err;
    }
    if (secondaryResult.err != ESP_OK && firstErr == ESP_OK) {
        firstErr = secondaryResult.err;
    }

    rowTiming->waitReadyUs = primaryTiming->waitReadyUs + secondaryTiming->waitReadyUs;
    rowTiming->readUs = primaryTiming->readRawUs + secondaryTiming->readRawUs;
    rowTiming->primaryTotalUs = primaryTiming->deviceUs;
    rowTiming->secondaryTotalUs = secondaryTiming->deviceUs;
    rowTiming->primaryJobUs = primaryTiming->deviceUs;
    rowTiming->secondaryJobUs = secondaryTiming->deviceUs;
    rowTiming->dualBusSkewUs =
        (primaryTiming->deviceUs > secondaryTiming->deviceUs) ?
        (primaryTiming->deviceUs - secondaryTiming->deviceUs) :
        (secondaryTiming->deviceUs - primaryTiming->deviceUs);
    return firstErr;
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

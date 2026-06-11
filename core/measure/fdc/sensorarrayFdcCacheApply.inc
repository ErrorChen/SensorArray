// Split from core/measure/sensorarrayMeasure.c to keep FDC matrix internals isolated.
static void sensorarrayMeasureRuntimeConfigsFromApplied(sensorarrayState_t *state,
                                                        sensorarrayFdcDeviceId_t devId,
                                                        sensorarrayFdcRuntimeChannelConfig_t configs[4])
{
    if (!state || !configs || devId > SENSORARRAY_FDC_DEV_SECONDARY) {
        return;
    }

    sensorarrayFdcDeviceState_t *fdcState = sensorarrayMeasureGetFdcState(state, devId);
    uint32_t effectiveFclkHz =
        (fdcState && fdcState->refClockKnown && fdcState->refClockHz != 0u) ?
        fdcState->refClockHz :
        sensorarrayMeasureFdcEffectiveFclkHz();
    const sensorarrayFdcAppliedRowConfig_t *applied = &state->fdcAppliedRow[(uint8_t)devId];
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

static uint32_t sensorarrayMeasureFdcTheoreticalFrameFpsX100(uint32_t roundUs)
{
    return roundUs ? (uint32_t)(100000000ull / ((uint64_t)roundUs * SENSORARRAY_MATRIX_ROWS)) : 0u;
}

static void sensorarrayMeasureLogFdcPfuDiag(sensorarrayFdcDeviceId_t devId,
                                            uint8_t row,
                                            const char *action,
                                            uint32_t roundUs,
                                            uint32_t targetRoundUs,
                                            uint32_t rowBudgetUs,
                                            const uint16_t rCount[4],
                                            const uint16_t settleCount[4])
{
    uint32_t theoreticalMaxFpsX100 = sensorarrayMeasureFdcTheoreticalFrameFpsX100(roundUs);
    printf("PFU,d=%s,r=%u,profile=%s,why=profile_too_slow,action=%s,round=%lu,target=%lu,rowBudget=%lu,theoreticalMaxFps=%lu.%02lu,rCount=[0x%04X,0x%04X,0x%04X,0x%04X],settle=[0x%04X,0x%04X,0x%04X,0x%04X]\n",
           sensorarrayMeasureFdcDeviceToken(devId),
           (unsigned)row,
           SENSORARRAY_FDC_PROFILE_NAME,
           action ? action : "profile_too_slow_diag_only",
           (unsigned long)roundUs,
           (unsigned long)targetRoundUs,
           (unsigned long)rowBudgetUs,
           (unsigned long)(theoreticalMaxFpsX100 / 100u),
           (unsigned long)(theoreticalMaxFpsX100 % 100u),
           rCount[0],
           rCount[1],
           rCount[2],
           rCount[3],
           settleCount[0],
           settleCount[1],
           settleCount[2],
           settleCount[3]);
}

static void sensorarrayMeasureLogFdcPfuFastProfile(sensorarrayFdcDeviceId_t devId,
                                                   uint8_t row,
                                                   const char *action,
                                                   uint32_t oldRoundUs,
                                                   uint32_t newRoundUs,
                                                   uint32_t targetRoundUs,
                                                   uint32_t rowBudgetUs,
                                                   const uint16_t oldRcount[4],
                                                   const uint16_t newRcount[4],
                                                   const uint16_t oldSettle[4],
                                                   const uint16_t newSettle[4],
                                                   const char *decisionReason,
                                                   uint32_t diffWriteCount)
{
    uint32_t oldTheoreticalMaxFpsX100 = sensorarrayMeasureFdcTheoreticalFrameFpsX100(oldRoundUs);
    uint32_t newTheoreticalMaxFpsX100 = sensorarrayMeasureFdcTheoreticalFrameFpsX100(newRoundUs);
    printf("PFU,d=%s,r=%u,profile=%s,why=profile_too_slow,action=%s,oldRound=%lu,newRound=%lu,target=%lu,rowBudget=%lu,chosenRound=%lu,oldTheoreticalMaxFps=%lu.%02lu,newTheoreticalMaxFps=%lu.%02lu,oldRcount=[0x%04X,0x%04X,0x%04X,0x%04X],newRcount=[0x%04X,0x%04X,0x%04X,0x%04X],oldSettle=[0x%04X,0x%04X,0x%04X,0x%04X],newSettle=[0x%04X,0x%04X,0x%04X,0x%04X],diffWrites=%lu,decisionReason=%s\n",
           sensorarrayMeasureFdcDeviceToken(devId),
           (unsigned)row,
           SENSORARRAY_FDC_PROFILE_NAME,
           action ? action : SENSORARRAY_NA,
           (unsigned long)oldRoundUs,
           (unsigned long)newRoundUs,
           (unsigned long)targetRoundUs,
           (unsigned long)rowBudgetUs,
           (unsigned long)newRoundUs,
           (unsigned long)(oldTheoreticalMaxFpsX100 / 100u),
           (unsigned long)(oldTheoreticalMaxFpsX100 % 100u),
           (unsigned long)(newTheoreticalMaxFpsX100 / 100u),
           (unsigned long)(newTheoreticalMaxFpsX100 % 100u),
           oldRcount[0],
           oldRcount[1],
           oldRcount[2],
           oldRcount[3],
           newRcount[0],
           newRcount[1],
           newRcount[2],
           newRcount[3],
           oldSettle[0],
           oldSettle[1],
           oldSettle[2],
           oldSettle[3],
           newSettle[0],
           newSettle[1],
           newSettle[2],
           newSettle[3],
           (unsigned long)diffWriteCount,
           decisionReason ? decisionReason : SENSORARRAY_NA);
}

static void sensorarrayMeasureLogFdcPfuApplyResult(sensorarrayFdcDeviceId_t devId,
                                                   uint8_t row,
                                                   const char *why,
                                                   const char *action,
                                                   uint32_t oldRoundUs,
                                                   uint32_t newRoundUs,
                                                   uint32_t targetRoundUs,
                                                   uint32_t rowBudgetUs,
                                                   uint16_t oldDriveCurrent0,
                                                   uint16_t newDriveCurrent0,
                                                   bool profileTooSlow,
                                                   uint32_t diffWriteCount)
{
    uint32_t newTheoreticalMaxFpsX100 = sensorarrayMeasureFdcTheoreticalFrameFpsX100(newRoundUs);
    printf("PFU,d=%s,r=%u,profile=%s,why=%s,action=%s,oldRound=%lu,newRound=%lu,target=%lu,rowBudget=%lu,theoreticalMaxFps=%lu.%02lu,oldDc=0x%04X,newDc=0x%04X,profileTooSlow=%u,diffWrites=%lu\n",
           sensorarrayMeasureFdcDeviceToken(devId),
           (unsigned)row,
           SENSORARRAY_FDC_PROFILE_NAME,
           why ? why : SENSORARRAY_NA,
           action ? action : SENSORARRAY_NA,
           (unsigned long)oldRoundUs,
           (unsigned long)newRoundUs,
           (unsigned long)targetRoundUs,
           (unsigned long)rowBudgetUs,
           (unsigned long)(newTheoreticalMaxFpsX100 / 100u),
           (unsigned long)(newTheoreticalMaxFpsX100 % 100u),
           oldDriveCurrent0,
           newDriveCurrent0,
           profileTooSlow ? 1u : 0u,
           (unsigned long)diffWriteCount);
}

static bool sensorarrayMeasurePromoteFdcExpectedProfileToCellCache(
    sensorarrayState_t *state,
    uint8_t row,
    sensorarrayFdcDeviceId_t devId,
    sensorarrayFdcAppliedRowConfig_t *expected,
    uint32_t effectiveFclkHz)
{
    if (!state || !expected || row < 1u || row > SENSORARRAY_MATRIX_ROWS ||
        devId > SENSORARRAY_FDC_DEV_SECONDARY) {
        return false;
    }

    expected->fingerprint = sensorarrayMeasureFdcConfigFingerprint(expected);

    bool promoted = false;
    for (uint8_t ch = 0u; ch < 4u; ++ch) {
        sensorarrayFdcCellConfigCache_t *cache =
            sensorarrayMeasureFdcRowDeviceCache(state, row, devId, ch, NULL);
        if (!cache || !cache->valid) {
            continue;
        }

        bool changed =
            cache->rCount != expected->rCount[ch] ||
            cache->settleCount != expected->settleCount[ch] ||
            cache->clockDiv != expected->clockDiv[ch] ||
            cache->driveCurrent != expected->driveCurrent[ch] ||
            cache->deglitchCode != expected->selectedDeglitch ||
            cache->effectiveFclkHz != effectiveFclkHz ||
            cache->lastAppliedFingerprint != expected->fingerprint;

        cache->rCount = expected->rCount[ch];
        cache->settleCount = expected->settleCount[ch];
        cache->clockDiv = expected->clockDiv[ch];
        cache->driveCurrent = expected->driveCurrent[ch];
        cache->deglitchCode = expected->selectedDeglitch;
        cache->effectiveFclkHz = effectiveFclkHz;
        cache->lastAppliedFingerprint = expected->fingerprint;
        promoted = promoted || changed;
    }

    return promoted;
}

static esp_err_t sensorarrayMeasureApplyFdcCachedRowConfig(sensorarrayState_t *state,
                                                           uint8_t row,
                                                           sensorarrayFdcDeviceId_t devId,
                                                           const char *reason,
                                                           bool forceWrite,
                                                           sensorarrayFdcDeviceTiming_t *timing)
{
    if (!state || row < 1u || row > SENSORARRAY_MATRIX_ROWS ||
        devId > SENSORARRAY_FDC_DEV_SECONDARY) {
        return ESP_ERR_INVALID_ARG;
    }

    sensorarrayFdcDeviceState_t *fdcState = sensorarrayMeasureGetFdcState(state, devId);
    if (!fdcState || !fdcState->ready || !fdcState->handle) {
        return ESP_ERR_INVALID_STATE;
    }
    int64_t applyStartUs = esp_timer_get_time();
    if (timing) {
        timing->row = row;
        timing->deviceId = devId;
    }

    int64_t buildStartUs = esp_timer_get_time();
    sensorarrayFdcAppliedRowConfig_t expected = {
        .valid = true,
        .autoscanConfigured = true,
        .row = row,
        .deviceId = (uint8_t)devId,
        .selectedDeglitch = sensorarrayMeasureFdcSafeDefaultDeglitch(),
    };
    uint8_t missingMask = 0u;
    bool reapplyRequested = false;

    for (uint8_t ch = 0u; ch < 4u; ++ch) {
        uint8_t dIndex = 0u;
        sensorarrayFdcCellConfigCache_t *cache =
            sensorarrayMeasureFdcRowDeviceCache(state, row, devId, ch, &dIndex);
        bool useCache = cache && cache->valid;
        if (!useCache) {
            missingMask |= (uint8_t)(1u << ch);
        }
        if (cache && cache->reapplyPending) {
            reapplyRequested = true;
        }

        expected.rCount[ch] = (useCache && cache->rCount != 0u) ?
            cache->rCount :
            SENSORARRAY_FDC_RCOUNT;
        expected.settleCount[ch] = (useCache && cache->settleCount != 0u) ?
            cache->settleCount :
            SENSORARRAY_FDC_SETTLECOUNT;
        expected.clockDiv[ch] = (useCache && cache->clockDiv != 0u) ?
            cache->clockDiv :
            SENSORARRAY_FDC_CLOCK_DIVIDERS;
        expected.driveCurrent[ch] = (useCache && cache->driveCurrent != 0u) ?
            cache->driveCurrent :
            SENSORARRAY_FDC_DRIVE_CURRENT;
        expected.cacheGeneration[ch] = useCache ? cache->generation : 0u;
        (void)dIndex;
    }

    expected.selectedDeglitch =
        sensorarrayFdcMergeDeglitchForRowDevice(state,
                                                row,
                                                devId,
                                                sensorarrayMeasureFdcSafeDefaultDeglitch());
    expected.muxConfig = sensorarrayMeasureFdcBuildMuxConfig(expected.selectedDeglitch);
    expected.statusConfig = SENSORARRAY_FDC_STATUS_CONFIG_DEFAULT;
    expected.configBaseWithoutSleepBit = sensorarrayMeasureFdcConfigBaseWithoutSleep(fdcState);
    sensorarrayFdcRuntimeChannelConfig_t snapshotConfigs[4] = {0};
    uint32_t effectiveFclkHz =
        (fdcState && fdcState->refClockKnown && fdcState->refClockHz != 0u) ?
        fdcState->refClockHz :
        sensorarrayMeasureFdcEffectiveFclkHz();
    for (uint8_t ch = 0u; ch < 4u; ++ch) {
        snapshotConfigs[ch] = (sensorarrayFdcRuntimeChannelConfig_t){
            .valid = true,
            .rCount = expected.rCount[ch],
            .settleCount = expected.settleCount[ch],
            .clockDividers = expected.clockDiv[ch],
            .driveCurrent = expected.driveCurrent[ch],
            .deglitchCode = expected.selectedDeglitch,
            .effectiveFclkHz = effectiveFclkHz,
        };
    }
    uint32_t expectedRoundUs = 0u;
    (void)sensorarrayMeasureFdcEstimateAutoscanReadyTimeoutUsWithSnapshot(
        snapshotConfigs,
        SENSORARRAY_FDC_AUTOSCAN_READY_UNREAD_MASK,
        &expectedRoundUs,
        &expected.profileSnapshot);
    sensorarrayFdcProfileSnapshot_t bootStableProfile = expected.profileSnapshot;
    uint32_t initialRoundUs = expected.profileSnapshot.autoscanRoundUs;
    uint16_t oldRcount[4] = {
        expected.rCount[0],
        expected.rCount[1],
        expected.rCount[2],
        expected.rCount[3],
    };
    uint16_t oldSettle[4] = {
        expected.settleCount[0],
        expected.settleCount[1],
        expected.settleCount[2],
        expected.settleCount[3],
    };
    uint32_t targetRoundUs = (uint32_t)CONFIG_SENSORARRAY_FDC_FORMAL_FAST_TARGET_ROUND_US;
    if (targetRoundUs == 0u || targetRoundUs > SENSORARRAY_FDC_TARGET_ROW_US) {
        targetRoundUs = SENSORARRAY_FDC_TARGET_ROW_US;
    }
    const uint32_t rowBudgetUs = SENSORARRAY_FDC_TARGET_ROW_US;
    bool initialProfileTooSlow =
        initialRoundUs > (uint32_t)CONFIG_SENSORARRAY_FDC_PROFILE_TOO_SLOW_WARN_US ||
        initialRoundUs > targetRoundUs;
    bool fastProfileApplied = false;
    bool fastProfilePossible = true;
    bool formalFastProfileEnabled = CONFIG_SENSORARRAY_FDC_FORMAL_FAST_PROFILE_ENABLE != 0;
    const char *fastProfileDecisionReason = "profile_within_budget";
    if (initialProfileTooSlow && formalFastProfileEnabled) {
        const uint16_t minRuntimeRcount = 0x0100u;
        for (uint8_t attempt = 0u;
             attempt < 4u && expected.profileSnapshot.autoscanRoundUs > targetRoundUs;
             ++attempt) {
            uint32_t currentRoundUs = expected.profileSnapshot.autoscanRoundUs;
            bool changedThisAttempt = false;
            if (currentRoundUs == 0u) {
                break;
            }
            for (uint8_t ch = 0u; ch < 4u; ++ch) {
                uint64_t scaled =
                    ((uint64_t)expected.rCount[ch] * (uint64_t)targetRoundUs) /
                    (uint64_t)currentRoundUs;
                if (scaled < minRuntimeRcount) {
                    scaled = minRuntimeRcount;
                }
                if (scaled > UINT16_MAX) {
                    scaled = UINT16_MAX;
                }
                uint16_t nextRcount = (uint16_t)scaled;
                if (nextRcount != 0u && nextRcount < expected.rCount[ch]) {
                    expected.rCount[ch] = nextRcount;
                    snapshotConfigs[ch].rCount = nextRcount;
                    changedThisAttempt = true;
                    fastProfileApplied = true;
                }
            }
            if (!changedThisAttempt) {
                fastProfilePossible = false;
                break;
            }
            (void)sensorarrayMeasureFdcEstimateAutoscanReadyTimeoutUsWithSnapshot(
                snapshotConfigs,
                SENSORARRAY_FDC_AUTOSCAN_READY_UNREAD_MASK,
                &expectedRoundUs,
                &expected.profileSnapshot);
        }
        fastProfilePossible = fastProfilePossible &&
                              expected.profileSnapshot.autoscanRoundUs <= targetRoundUs;
        fastProfileDecisionReason = fastProfilePossible ?
            "round_within_target_after_rcount_reduce" :
            (fastProfileApplied ? "current_profile_cannot_meet_target_fps" :
             "no_rCount_reduction_possible");
    } else if (initialProfileTooSlow) {
        fastProfileDecisionReason = "formal_fast_profile_disabled";
    }
    expected.profileSnapshot.muxConfig = expected.muxConfig;
    expected.profileSnapshot.statusConfig = expected.statusConfig;
    expected.profileSnapshot.config = expected.configBaseWithoutSleepBit;
    expected.bootStableProfile = bootStableProfile;
    expected.formalFastProfile = expected.profileSnapshot;
    expected.fingerprint = sensorarrayMeasureFdcConfigFingerprint(&expected);
    if (timing) {
        timing->applyBuildConfigUs += sensorarrayMeasureElapsedUs(buildStartUs);
    }

    if (missingMask != 0u && !CONFIG_SENSORARRAY_FDC_ALLOW_SAFE_DEFAULT_FORMAL_READ) {
        printf("FDC_CACHE_MISS,row=%u,device=%s,missingMask=0x%X,action=mark_invalid_request_fast_sweep,reason=cache_missing\n",
               (unsigned)row,
               sensorarrayMeasureFdcDeviceName(devId),
               (unsigned)missingMask);
        for (uint8_t ch = 0u; ch < 4u; ++ch) {
            if ((missingMask & (uint8_t)(1u << ch)) == 0u) {
                continue;
            }
            uint8_t dIndex = (uint8_t)(1u + ch + ((devId == SENSORARRAY_FDC_DEV_SECONDARY) ? 4u : 0u));
            uint8_t matrixIndex = (uint8_t)sensorarrayMatrixIndex(row, dIndex);
            (void)sensorarrayMeasureRequestFdcCellRescue(state, matrixIndex, "cache_missing");
        }
        if (timing) {
            timing->applyUs += sensorarrayMeasureElapsedUs(applyStartUs);
        }
        return ESP_ERR_NOT_FOUND;
    }

    if (missingMask != 0u) {
        printf("FDC_CACHE_MISS,row=%u,device=%s,missingMask=0x%X,action=use_safe_default,reason=debug_safe_default_enabled\n",
               (unsigned)row,
               sensorarrayMeasureFdcDeviceName(devId),
               (unsigned)missingMask);
    }

    sensorarrayFdcAppliedRowConfig_t *applied = &state->fdcAppliedRow[(uint8_t)devId];
    if (!forceWrite &&
        !reapplyRequested &&
        sensorarrayMeasureAppliedRowConfigMatches(applied, &expected)) {
        bool fastProfileCachePromoted = false;
        if (formalFastProfileEnabled && fastProfileApplied) {
            fastProfileCachePromoted =
                sensorarrayMeasurePromoteFdcExpectedProfileToCellCache(state,
                                                                       row,
                                                                       devId,
                                                                       &expected,
                                                                       effectiveFclkHz);
        }
        if (initialProfileTooSlow) {
            if (!formalFastProfileEnabled) {
                sensorarrayMeasureLogFdcPfuDiag(devId,
                                                row,
                                                "disabled_diag_only",
                                                initialRoundUs,
                                                targetRoundUs,
                                                rowBudgetUs,
                                                oldRcount,
                                                oldSettle);
            } else if (fastProfileApplied) {
                sensorarrayMeasureLogFdcPfuFastProfile(
                    devId,
                    row,
                    fastProfileCachePromoted ? "promote_fast_profile_cache_only" :
                    "fast_profile_already_applied_no_write",
                    initialRoundUs,
                    expected.profileSnapshot.autoscanRoundUs,
                    targetRoundUs,
                    rowBudgetUs,
                    oldRcount,
                    expected.rCount,
                    oldSettle,
                    expected.settleCount,
                    fastProfileDecisionReason,
                    0u);
            } else {
                sensorarrayMeasureLogFdcPfuApplyResult(devId,
                                                       row,
                                                       "profile_too_slow",
                                                       "no_write_already_applied",
                                                       applied->profileSnapshot.autoscanRoundUs,
                                                       expected.profileSnapshot.autoscanRoundUs,
                                                       targetRoundUs,
                                                       rowBudgetUs,
                                                       applied->driveCurrent[0],
                                                       expected.driveCurrent[0],
                                                       true,
                                                       0u);
            }
        }
#if CONFIG_SENSORARRAY_FDC_VERBOSE_SCAN_LOG
        printf("FDC_CACHE,stage=hit,row=%u,device=%s,action=no_write\n",
               (unsigned)row,
               sensorarrayMeasureFdcDeviceName(devId));
#endif
        if (timing) {
            timing->applyUs += sensorarrayMeasureElapsedUs(applyStartUs);
            timing->cacheNoDiffCount++;
        }
        return ESP_OK;
    }

    esp_err_t firstErr = ESP_OK;
    uint32_t diffWriteCount = 0u;
    uint32_t diffRcountWrites = 0u;
    uint32_t diffSettleWrites = 0u;
    uint32_t diffClockWrites = 0u;
    uint32_t diffDriveWrites = 0u;
    uint32_t diffMuxWrites = 0u;
    uint32_t diffStatusWrites = 0u;
    uint32_t diffConfigWrites = 0u;
    const bool fullWrite = !applied->valid;

#define SENSORARRAY_FDC_WRITE_DIFF(reg_, desired_, appliedValue_, counter_) \
    do { \
        uint16_t desiredValue__ = (uint16_t)(desired_); \
        bool needWrite__ = forceWrite || reapplyRequested || !applied->valid || \
                            ((uint16_t)(appliedValue_) != desiredValue__); \
        if (needWrite__) { \
            int64_t writeStartUs__ = esp_timer_get_time(); \
            esp_err_t writeErr__ = Fdc2214CapWriteRawRegisters(fdcState->handle, (uint8_t)(reg_), desiredValue__); \
            uint64_t writeUs__ = sensorarrayMeasureElapsedUs(writeStartUs__); \
            if (timing) { \
                timing->channelConfigWriteUs += writeUs__; \
            } \
            if (writeErr__ != ESP_OK && firstErr == ESP_OK) { \
                firstErr = writeErr__; \
            } \
            if (writeErr__ == ESP_OK) { \
                diffWriteCount++; \
                (counter_)++; \
            } \
        } \
    } while (0)

    for (uint8_t ch = 0u; ch < 4u; ++ch) {
        SENSORARRAY_FDC_WRITE_DIFF(sensorarrayMeasureFdcRegForChannel(SENSORARRAY_FDC_REG_RCOUNT_BASE,
                                                                      (Fdc2214CapChannel_t)ch),
                                   expected.rCount[ch],
                                   applied->rCount[ch],
                                   diffRcountWrites);
        SENSORARRAY_FDC_WRITE_DIFF(sensorarrayMeasureFdcRegForChannel(SENSORARRAY_FDC_REG_SETTLECOUNT_BASE,
                                                                      (Fdc2214CapChannel_t)ch),
                                   expected.settleCount[ch],
                                   applied->settleCount[ch],
                                   diffSettleWrites);
        SENSORARRAY_FDC_WRITE_DIFF(sensorarrayMeasureFdcRegForChannel(SENSORARRAY_FDC_REG_CLOCK_DIVIDERS_BASE,
                                                                      (Fdc2214CapChannel_t)ch),
                                   expected.clockDiv[ch],
                                   applied->clockDiv[ch],
                                   diffClockWrites);
        SENSORARRAY_FDC_WRITE_DIFF(sensorarrayMeasureFdcRegForChannel(SENSORARRAY_FDC_REG_DRIVE_CURRENT_BASE,
                                                                      (Fdc2214CapChannel_t)ch),
                                   (uint16_t)(expected.driveCurrent[ch] & SENSORARRAY_FDC_DRIVE_CURRENT_MASK),
                                   applied->driveCurrent[ch],
                                   diffDriveWrites);
    }

    int64_t globalWriteStartUs = esp_timer_get_time();
    SENSORARRAY_FDC_WRITE_DIFF(SENSORARRAY_FDC_REG_STATUS_CONFIG,
                               expected.statusConfig,
                               applied->statusConfig,
                               diffStatusWrites);
    SENSORARRAY_FDC_WRITE_DIFF(SENSORARRAY_FDC_REG_MUX_CONFIG,
                               expected.muxConfig,
                               applied->muxConfig,
                               diffMuxWrites);
    SENSORARRAY_FDC_WRITE_DIFF(SENSORARRAY_FDC_REG_CONFIG,
                               (uint16_t)(expected.configBaseWithoutSleepBit |
                                          SENSORARRAY_FDC_CONFIG_SLEEP_MODE_EN_MASK),
                               (uint16_t)(applied->configBaseWithoutSleepBit |
                                          SENSORARRAY_FDC_CONFIG_SLEEP_MODE_EN_MASK),
                               diffConfigWrites);
    if (timing) {
        timing->globalConfigWriteUs += sensorarrayMeasureElapsedUs(globalWriteStartUs);
    }
#undef SENSORARRAY_FDC_WRITE_DIFF

    if (firstErr == ESP_OK) {
        fdcState->statusConfigReg = expected.statusConfig;
        fdcState->muxConfigReg = expected.muxConfig;
        fdcState->configReg = (uint16_t)(expected.configBaseWithoutSleepBit |
                                         SENSORARRAY_FDC_CONFIG_SLEEP_MODE_EN_MASK);
        fdcState->configVerified = true;
    }

    if (firstErr != ESP_OK) {
        applied->dirty = true;
        printf("FDC_CACHE,stage=apply_row_failed,row=%u,device=%s,reason=%s,err=0x%lx\n",
               (unsigned)row,
               sensorarrayMeasureFdcDeviceName(devId),
               reason ? reason : SENSORARRAY_NA,
               (unsigned long)firstErr);
        if (timing) {
            timing->applyUs += sensorarrayMeasureElapsedUs(applyStartUs);
        }
        return firstErr;
    }

    uint32_t previousFingerprint = applied->fingerprint;
    uint32_t previousRoundUs = applied->profileSnapshot.valid ?
        applied->profileSnapshot.autoscanRoundUs :
        0u;
    uint16_t previousDriveCurrent0 = applied->driveCurrent[0];
    uint32_t nextApplyCount = applied->applyCount + 1u;
    expected.applyCount = nextApplyCount;
    expected.lastAppliedTimestampUs = esp_timer_get_time();
    expected.dirty = false;
    bool fastProfileCachePromoted = false;
    if (formalFastProfileEnabled && fastProfileApplied) {
        fastProfileCachePromoted =
            sensorarrayMeasurePromoteFdcExpectedProfileToCellCache(state,
                                                                   row,
                                                                   devId,
                                                                   &expected,
                                                                   effectiveFclkHz);
    }
    *applied = expected;
    s_fdcProfileSnapshotByRow[row - 1u][(uint8_t)devId] = applied->profileSnapshot;
    if (timing) {
        timing->cacheDiffWriteCount += diffWriteCount;
        timing->cacheFullWriteCount += fullWrite ? 1u : 0u;
        timing->cacheNoDiffCount += (diffWriteCount == 0u) ? 1u : 0u;
        timing->diffRcountWrites += diffRcountWrites;
        timing->diffSettleWrites += diffSettleWrites;
        timing->diffClockDivWrites += diffClockWrites;
        timing->diffDriveWrites += diffDriveWrites;
        timing->diffMuxWrites += diffMuxWrites;
        timing->diffStatusConfigWrites += diffStatusWrites;
        timing->diffConfigWrites += diffConfigWrites;
        timing->appliedFingerprintChanged += (previousFingerprint != expected.fingerprint) ? 1u : 0u;
    }

    for (uint8_t ch = 0u; ch < 4u; ++ch) {
        sensorarrayFdcCellConfigCache_t *cache =
            sensorarrayMeasureFdcRowDeviceCache(state, row, devId, ch, NULL);
        if (cache) {
            cache->lastAppliedTimestampUs = applied->lastAppliedTimestampUs;
            cache->lastAppliedFingerprint = applied->fingerprint;
            cache->reapplyPending = false;
        }
    }

    bool profileTooSlow =
        applied->profileSnapshot.autoscanRoundUs >
        (uint32_t)CONFIG_SENSORARRAY_FDC_PROFILE_TOO_SLOW_WARN_US;
    if (initialProfileTooSlow && !formalFastProfileEnabled) {
        sensorarrayMeasureLogFdcPfuDiag(devId,
                                        row,
                                        "disabled_diag_only",
                                        initialRoundUs,
                                        targetRoundUs,
                                        rowBudgetUs,
                                        oldRcount,
                                        oldSettle);
    }
    if (diffWriteCount != 0u ||
        previousRoundUs != applied->profileSnapshot.autoscanRoundUs ||
        profileTooSlow) {
        if (formalFastProfileEnabled && fastProfileApplied) {
            sensorarrayMeasureLogFdcPfuFastProfile(
                devId,
                row,
                fastProfileCachePromoted ? "promote_fast_profile_and_write" :
                "apply_fast_profile_and_write",
                initialRoundUs,
                applied->profileSnapshot.autoscanRoundUs,
                targetRoundUs,
                rowBudgetUs,
                oldRcount,
                applied->rCount,
                oldSettle,
                applied->settleCount,
                fastProfileDecisionReason,
                diffWriteCount);
        } else {
            sensorarrayMeasureLogFdcPfuApplyResult(
                devId,
                row,
                profileTooSlow ? "profile_too_slow" :
                (previousRoundUs == 0u ? "shadow_profile_init" : "cache_profile_update"),
                diffWriteCount != 0u ? "apply_cached_profile_write" :
                "no_write_already_applied",
                previousRoundUs,
                applied->profileSnapshot.autoscanRoundUs,
                targetRoundUs,
                rowBudgetUs,
                previousRoundUs ? previousDriveCurrent0 : 0u,
                applied->driveCurrent[0],
                profileTooSlow,
                diffWriteCount);
        }
    }
    if (profileTooSlow && CONFIG_SENSORARRAY_FDC_PROFILE_TOO_SLOW_RESCUE_ENABLE) {
        for (uint8_t ch = 0u; ch < 4u; ++ch) {
            uint8_t dIndex = (uint8_t)(1u + ch + ((devId == SENSORARRAY_FDC_DEV_SECONDARY) ? 4u : 0u));
            uint8_t matrixIndex = (uint8_t)sensorarrayMatrixIndex(row, dIndex);
            (void)sensorarrayMeasureRequestFdcCellRescue(state, matrixIndex, "profile_too_slow");
        }
    }

#if CONFIG_SENSORARRAY_LOG_CACHE_APPLY_VERBOSE
    const bool printApplyLog = true;
#else
    const bool printApplyLog = diffWriteCount != 0u;
#endif
    if (printApplyLog) {
    printf("FDC_CACHE_APPLY,row=%u,dev=%s,force=%u,reason=%s,changed=%lu,driveCurrent=[0x%04X,0x%04X,0x%04X,0x%04X],rCount=[0x%04X,0x%04X,0x%04X,0x%04X],settle=[0x%04X,0x%04X,0x%04X,0x%04X],clockDiv=[0x%04X,0x%04X,0x%04X,0x%04X],deglitch=0x%X,muxConfig=0x%04X,statusConfig=0x%04X,configBaseWithoutSleep=0x%04X\n",
           (unsigned)row,
           sensorarrayMeasureFdcDeviceName(devId),
           forceWrite || reapplyRequested ? 1u : 0u,
           reason ? reason : SENSORARRAY_NA,
           (unsigned long)diffWriteCount,
           applied->driveCurrent[0],
           applied->driveCurrent[1],
           applied->driveCurrent[2],
           applied->driveCurrent[3],
           applied->rCount[0],
           applied->rCount[1],
           applied->rCount[2],
           applied->rCount[3],
           applied->settleCount[0],
           applied->settleCount[1],
           applied->settleCount[2],
           applied->settleCount[3],
           applied->clockDiv[0],
           applied->clockDiv[1],
           applied->clockDiv[2],
           applied->clockDiv[3],
           (unsigned)applied->selectedDeglitch,
           applied->muxConfig,
           applied->statusConfig,
           applied->configBaseWithoutSleepBit);
    }
    if (timing) {
        timing->applyUs += sensorarrayMeasureElapsedUs(applyStartUs);
    }
    return ESP_OK;
}

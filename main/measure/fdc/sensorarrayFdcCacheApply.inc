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
    expected.fingerprint = sensorarrayMeasureFdcConfigFingerprint(&expected);
    if (timing) {
        timing->applyBuildConfigUs += sensorarrayMeasureElapsedUs(buildStartUs);
    }

    if (missingMask != 0u) {
        printf("FDC_CACHE,stage=miss,row=%u,device=%s,missingMask=0x%X,action=use_safe_default\n",
               (unsigned)row,
               sensorarrayMeasureFdcDeviceName(devId),
               (unsigned)missingMask);
    }

    sensorarrayFdcAppliedRowConfig_t *applied = &state->fdcAppliedRow[(uint8_t)devId];
    if (!forceWrite &&
        !reapplyRequested &&
        sensorarrayMeasureAppliedRowConfigMatches(applied, &expected)) {
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
    uint32_t nextApplyCount = applied->applyCount + 1u;
    expected.applyCount = nextApplyCount;
    expected.lastAppliedTimestampUs = esp_timer_get_time();
    expected.dirty = false;
    *applied = expected;
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

#if CONFIG_SENSORARRAY_FDC_CACHE_APPLY_VERBOSE_LOG
    const bool printApplyLog = true;
#else
    const bool printApplyLog = diffWriteCount != 0u;
#endif
    if (printApplyLog) {
    printf("FDC_CACHE,stage=apply_row,row=%u,device=%s,force=%u,reason=%s,changed=%lu,driveCurrent=[0x%04X,0x%04X,0x%04X,0x%04X],rCount=[0x%04X,0x%04X,0x%04X,0x%04X],settle=[0x%04X,0x%04X,0x%04X,0x%04X],clockDiv=[0x%04X,0x%04X,0x%04X,0x%04X],deglitch=0x%X,muxConfig=0x%04X,statusConfig=0x%04X,configBaseWithoutSleep=0x%04X\n",
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

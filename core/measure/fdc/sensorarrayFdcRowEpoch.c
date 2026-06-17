#include "sensorarrayFdcInternal.h"

esp_err_t sensorarrayMeasureFdcDeviceLevelResync(sensorarrayState_t *state,
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
                                            false,
                                            readyAfter,
                                            timing);
        uint64_t waitUs = sensorarrayMeasureElapsedUs(waitStartUs);
        if (timing) {
            timing->waitReadyUs += waitUs;
            if (waitUs > timing->maxWaitReadyUs) {
                timing->maxWaitReadyUs = waitUs;
            }
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

esp_err_t sensorarrayMeasureFdcPrepareDeviceWhileSleeping(sensorarrayState_t *state,
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
            printf("FE,r=%u,d=%s,cfg=%04X,mux=%04X,sc=%04X,ru=%lu,slow=%u\n",
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

esp_err_t sensorarrayMeasureFdcDrainStaleUnread(sensorarrayFdcDeviceState_t *fdcState,
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

esp_err_t sensorarrayMeasureFdcRunDeviceEpochAfterSleep(sensorarrayState_t *state,
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
    uint64_t rowDeviceDeadlineUs = rowDeviceDeadlineOverrideUs;
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
    if (rowDeviceDeadlineUs == 0u) {
        rowDeviceDeadlineUs =
            (uint64_t)esp_timer_get_time() +
            sensorarrayMeasureFdcRowDeviceWatchdogHardTimeoutUs();
    }
#if CONFIG_SENSORARRAY_FDC_VERIFY_MODE_FULL
    err = sensorarrayMeasureFdcVerifySleepExit(state, row, epochId, devId, timing);
    if (err != ESP_OK) {
        return err;
    }
#endif

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
                                        true,
                                        outReady,
                                        timing);
    uint64_t waitUs = sensorarrayMeasureElapsedUs(waitStartUs);
    if (timing) {
        timing->waitReadyUs += waitUs;
        if (waitUs > timing->maxWaitReadyUs) {
            timing->maxWaitReadyUs = waitUs;
        }
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
    sensorarrayMeasureDebugReadWindow(devId, true);
    err = sensorarrayMeasureReadFdcAutoscan4chMasked(fdcState, freshMask, outReady, outSamples);
    sensorarrayMeasureDebugReadWindow(devId, false);
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
    if (outReady->directDataCandidate &&
        !sensorarrayFdcRead4IsDataCompleteGood(
            outRead4,
            SENSORARRAY_FDC_AUTOSCAN_READY_UNREAD_MASK)) {
        uint32_t fallbackReasonMask = 0u;
        if (outRead4->readErr != ESP_OK || outRead4->i2cError) {
            fallbackReasonMask |= SENSORARRAY_FDC_DIRECT_FALLBACK_I2C;
        }
        if (outRead4->validMask4 != SENSORARRAY_FDC_AUTOSCAN_READY_UNREAD_MASK) {
            fallbackReasonMask |= SENSORARRAY_FDC_DIRECT_FALLBACK_VALID_MASK;
        }
        if (outRead4->errorMask4 != 0u) {
            fallbackReasonMask |= SENSORARRAY_FDC_DIRECT_FALLBACK_ERROR_MASK;
        }
        if (outRead4->rawAllZero || outRead4->zeroAfterDrdyMask4 != 0u) {
            fallbackReasonMask |= SENSORARRAY_FDC_DIRECT_FALLBACK_RAW;
        }
        if (timing) {
            timing->directDataFallbackCount++;
            timing->directDataFallbackReasonMask |= fallbackReasonMask;
            timing->fallbackAttemptCount++;
            if (timing->statusSavedReadCount > 0u) {
                timing->statusSavedReadCount--;
            }
        }

        sensorarrayFdcReadyState_t fallbackReady = {0};
        sensorarrayFdcAutoscanSamples_t fallbackSamples = {0};
        sensorarrayFdcDeviceRead4Result_t fallbackRead4 = {0};
        int64_t fallbackWaitStartUs = esp_timer_get_time();
        uint64_t fallbackDeadlineUs = rowDeviceDeadlineUs;
        esp_err_t fallbackErr = sensorarrayFdcWaitDeviceReady(
            state,
            devId,
            row,
            epochId,
            frameSeq,
            SENSORARRAY_FDC_AUTOSCAN_READY_UNREAD_MASK,
            timeoutUs,
            estimatedRoundUs,
            fallbackDeadlineUs,
            false,
            &fallbackReady,
            timing);
        uint64_t fallbackWaitUs = sensorarrayMeasureElapsedUs(fallbackWaitStartUs);
        if (timing) {
            timing->waitReadyUs += fallbackWaitUs;
            timing->fallbackSecondWaitUs += fallbackWaitUs;
            timing->fallbackSecondWaitCount++;
            if (fallbackWaitUs > timing->fallbackSecondWaitMaxUs) {
                timing->fallbackSecondWaitMaxUs = fallbackWaitUs;
            }
            if (fallbackWaitUs > timing->maxWaitReadyUs) {
                timing->maxWaitReadyUs = fallbackWaitUs;
            }
        }
        if (fallbackErr == ESP_OK &&
            sensorarrayMeasureFdcReadyAllowsNormalDataRead(
                &fallbackReady,
                SENSORARRAY_FDC_AUTOSCAN_READY_UNREAD_MASK)) {
            fallbackReady.statusFallbackUsed = true;
            uint8_t fallbackFreshMask = fallbackReady.unreadMask & 0x0Fu;
            int64_t fallbackReadStartUs = esp_timer_get_time();
            sensorarrayMeasureDebugReadWindow(devId, true);
            fallbackErr = sensorarrayMeasureReadFdcAutoscan4chMasked(fdcState,
                                                                      fallbackFreshMask,
                                                                      &fallbackReady,
                                                                      &fallbackSamples);
            sensorarrayMeasureDebugReadWindow(devId, false);
            uint64_t fallbackReadUs = sensorarrayMeasureElapsedUs(fallbackReadStartUs);
            if (timing) {
                timing->readRawUs += fallbackReadUs;
                timing->dataReadUs += fallbackReadUs;
                if (fallbackReadUs > timing->maxI2cReadUs) {
                    timing->maxI2cReadUs = fallbackReadUs;
                }
            }
            sensorarrayMeasureBuildFdcRead4Result(row,
                                                  epochId,
                                                  devId,
                                                  &fallbackSamples,
                                                  outConfigs,
                                                  &fallbackReady,
                                                  fallbackErr,
                                                  (uint32_t)fallbackReadUs,
                                                  true,
                                                  &fallbackRead4);
        }
        if (fallbackErr == ESP_OK &&
            sensorarrayFdcRead4IsDataCompleteGood(
                &fallbackRead4,
                SENSORARRAY_FDC_AUTOSCAN_READY_UNREAD_MASK)) {
            *outReady = fallbackReady;
            *outSamples = fallbackSamples;
            *outRead4 = fallbackRead4;
            err = ESP_OK;
            if (timing) {
                timing->fallbackSuccessCount++;
            }
        } else if (timing) {
            timing->fallbackFailCount++;
        }
    }
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
                                                           false,
                                                           &retryReady,
                                                           timing);
        uint64_t retryWaitUs = sensorarrayMeasureElapsedUs(retryWaitStartUs);
        if (timing) {
            timing->waitReadyUs += retryWaitUs;
            timing->fallbackSecondWaitUs += retryWaitUs;
            timing->fallbackSecondWaitCount++;
            if (retryWaitUs > timing->fallbackSecondWaitMaxUs) {
                timing->fallbackSecondWaitMaxUs = retryWaitUs;
            }
            if (retryWaitUs > timing->maxWaitReadyUs) {
                timing->maxWaitReadyUs = retryWaitUs;
            }
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
            sensorarrayMeasureDebugReadWindow(devId, true);
            retryErr = sensorarrayMeasureReadFdcAutoscan4chMasked(fdcState,
                                                                   retryFreshMask,
                                                                   &retryReady,
                                                                   &retrySamples);
            sensorarrayMeasureDebugReadWindow(devId, false);
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

#define SENSORARRAY_FDC_WORKER_ACK_PRIMARY_BIT BIT0
#define SENSORARRAY_FDC_WORKER_ACK_SECONDARY_BIT BIT1
#define SENSORARRAY_FDC_WORKER_DONE_PRIMARY_BIT BIT2
#define SENSORARRAY_FDC_WORKER_DONE_SECONDARY_BIT BIT3
#define SENSORARRAY_FDC_WORKER_ACK_ALL_BITS \
    (SENSORARRAY_FDC_WORKER_ACK_PRIMARY_BIT | SENSORARRAY_FDC_WORKER_ACK_SECONDARY_BIT)
#define SENSORARRAY_FDC_WORKER_DONE_ALL_BITS \
    (SENSORARRAY_FDC_WORKER_DONE_PRIMARY_BIT | SENSORARRAY_FDC_WORKER_DONE_SECONDARY_BIT)

EventBits_t sensorarrayMeasureFdcWorkerAckBit(sensorarrayFdcDeviceId_t devId)
{
    return devId == SENSORARRAY_FDC_DEV_SECONDARY ?
        SENSORARRAY_FDC_WORKER_ACK_SECONDARY_BIT : SENSORARRAY_FDC_WORKER_ACK_PRIMARY_BIT;
}

EventBits_t sensorarrayMeasureFdcWorkerDoneBit(sensorarrayFdcDeviceId_t devId)
{
    return devId == SENSORARRAY_FDC_DEV_SECONDARY ?
        SENSORARRAY_FDC_WORKER_DONE_SECONDARY_BIT : SENSORARRAY_FDC_WORKER_DONE_PRIMARY_BIT;
}

void sensorarrayMeasureFdcWorkerTask(void *arg)
{
    sensorarrayFdcWorkerContext_t *ctx = (sensorarrayFdcWorkerContext_t *)arg;
    if (!ctx) {
        vTaskDelete(NULL);
        return;
    }
    printf("TASKCORE,name=fdc_%s,core=%d,expected=%d\n",
           ctx->devId == SENSORARRAY_FDC_DEV_SECONDARY ? "secondary" : "primary",
           (int)xPortGetCoreID(),
           ctx->devId == SENSORARRAY_FDC_DEV_SECONDARY ?
               CONFIG_SENSORARRAY_FDC_SECONDARY_WORKER_TASK_CORE :
               CONFIG_SENSORARRAY_FDC_PRIMARY_WORKER_TASK_CORE);

    for (;;) {
        if (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) == 0u) {
            continue;
        }
        sensorarrayFdcWorkerJob_t *job = (sensorarrayFdcWorkerJob_t *)ctx->job;
        if (!job || !job->state || !job->outSamples || !job->outConfigs || !job->result) {
            if (job && job->result) {
                job->result->err = ESP_ERR_INVALID_ARG;
            }
            xEventGroupSetBits(s_fdcWorkerEvents, sensorarrayMeasureFdcWorkerDoneBit(ctx->devId));
            continue;
        }

        *job->result = (sensorarrayFdcWorkerResult_t){
            .err = ESP_ERR_TIMEOUT,
            .frameSeq = job->frameSeq,
            .row = job->row,
            .devId = ctx->devId,
            .epochId = job->epochId,
            .generation = job->generation,
        };
        if (job->trace) {
            job->trace->frameSeq = job->frameSeq;
            job->trace->row = job->row;
            job->trace->epochId = job->epochId;
            job->trace->devId = ctx->devId;
            job->trace->jobQueued = true;
            job->trace->generation = job->generation;
        }
        sensorarrayFdcDeviceState_t *fdcState = sensorarrayMeasureGetFdcState(job->state, ctx->devId);
        esp_err_t err = ESP_ERR_INVALID_STATE;
        if (fdcState && fdcState->ready && fdcState->handle) {
            if (sensorarrayMeasureFdcShouldLogNormalFrame(job->frameSeq)) {
                printf("FDC_ROW_EPOCH,stage=sleep_enter,row=%u,epoch=%lu,dev=%s\n",
                       (unsigned)job->row,
                       (unsigned long)job->epochId,
                       sensorarrayMeasureFdcDeviceName(ctx->devId));
            }
            err = sensorarrayMeasureFdcSetSleepMode(fdcState, true, job->timing);
        }
        job->result->err = err;
        if (job->trace) {
            job->trace->err = err;
            job->trace->readyAckUs = (uint64_t)esp_timer_get_time();
        }
        xEventGroupSetBits(s_fdcWorkerEvents, sensorarrayMeasureFdcWorkerAckBit(ctx->devId));

        int64_t startWaitStartUs = esp_timer_get_time();
        uint32_t startNotify = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (err == ESP_OK && startNotify != 0u) {
            if (job->trace) {
                job->trace->startWaitUs = (uint32_t)sensorarrayMeasureElapsedUs(startWaitStartUs);
                job->trace->runStarted = true;
            }
            int64_t runStartUs = esp_timer_get_time();
            uint32_t rowDeviceBudgetUs = job->rowDeviceBudgetUs != 0u ?
                job->rowDeviceBudgetUs : sensorarrayMeasureFdcRowDeviceWatchdogHardTimeoutUs();
            uint64_t rowDeviceDeadlineUs = (uint64_t)runStartUs + rowDeviceBudgetUs;
            if (job->trace) {
                job->trace->workerStartUs = (uint64_t)runStartUs;
                job->trace->workerDeadlineUs = rowDeviceDeadlineUs;
                job->trace->rowHardDeadlineUs = rowDeviceBudgetUs;
            }
            err = sensorarrayMeasureFdcRunDeviceEpochAfterSleep(job->state,
                                                                job->row,
                                                                job->epochId,
                                                                job->frameSeq,
                                                                ctx->devId,
                                                                job->outSamples,
                                                                job->outConfigs,
                                                                &job->result->ready,
                                                                &job->result->read4,
                                                                job->timing,
                                                                job->readTracker,
                                                                "worker",
                                                                rowDeviceDeadlineUs);
            job->result->err = err;
            if (job->trace) {
                job->trace->workerRunUs = (uint32_t)sensorarrayMeasureElapsedUs(runStartUs);
                job->trace->workerEndUs = (uint64_t)esp_timer_get_time();
                job->trace->runCompleted = true;
                job->trace->err = err;
            }
        } else {
            job->result->err = err == ESP_OK ? ESP_ERR_TIMEOUT : err;
            if (job->trace) {
                job->trace->startWaitUs = (uint32_t)sensorarrayMeasureElapsedUs(startWaitStartUs);
                job->trace->err = job->result->err;
            }
        }
        while (ulTaskNotifyTake(pdTRUE, 0) != 0u) {
        }
        xEventGroupSetBits(s_fdcWorkerEvents, sensorarrayMeasureFdcWorkerDoneBit(ctx->devId));
    }
}

esp_err_t sensorarrayMeasureEnsureFdcWorkers(void)
{
    if (s_fdcWorkersInitAttempted) {
        return s_fdcWorkersAvailable ? ESP_OK : ESP_ERR_NOT_SUPPORTED;
    }
    s_fdcWorkersInitAttempted = true;
    s_fdcWorkerEvents = xEventGroupCreateStatic(&s_fdcWorkerEventStorage);
    if (!s_fdcWorkerEvents) {
        return ESP_ERR_NO_MEM;
    }

    esp_err_t firstErr = ESP_OK;
    for (uint8_t i = 0u; i < 2u; ++i) {
        sensorarrayFdcWorkerContext_t *ctx = &s_fdcWorkers[i];
        (void)sensorarrayMeasureEnsureFdcIntb(ctx);

        char taskName[24] = {0};
        snprintf(taskName, sizeof(taskName), "fdc_%s_worker",
                 ctx->devId == SENSORARRAY_FDC_DEV_SECONDARY ? "secondary" : "primary");
        int configuredWorkerCore =
            (ctx->devId == SENSORARRAY_FDC_DEV_SECONDARY) ?
            CONFIG_SENSORARRAY_FDC_SECONDARY_WORKER_TASK_CORE :
            CONFIG_SENSORARRAY_FDC_PRIMARY_WORKER_TASK_CORE;
        BaseType_t resolvedCore = 0;
        bool pinned = true;
        const char *coreReason = "unknown";
        if (!sensorarrayMeasureResolveWorkerCore(taskName,
                                                 configuredWorkerCore,
                                                 ctx->devId == SENSORARRAY_FDC_DEV_SECONDARY ? 0 : 1,
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
                   configuredWorkerCore,
                   (int)resolvedCore,
                   pinned ? 1u : 0u,
                   (int)portNUM_PROCESSORS,
                   coreReason);
        }
        printf("FDC_WORKER_CREATE_BEGIN,dev=%s,configuredCore=%d,resolvedCore=%d,pinned=%u,stackWords=%lu,prio=%d,portNumProcessors=%d\n",
               sensorarrayMeasureFdcDeviceName(ctx->devId),
               configuredWorkerCore,
               (int)resolvedCore,
               pinned ? 1u : 0u,
               (unsigned long)SENSORARRAY_FDC_WORKER_STACK_WORDS,
               CONFIG_SENSORARRAY_FDC_WORKER_TASK_PRIO,
               (int)portNUM_PROCESSORS);
        (void)pinned;
        ctx->task = xTaskCreateStaticPinnedToCore(sensorarrayMeasureFdcWorkerTask,
                                                  taskName,
                                                  SENSORARRAY_FDC_WORKER_STACK_WORDS,
                                                  ctx,
                                                  CONFIG_SENSORARRAY_FDC_WORKER_TASK_PRIO,
                                                  ctx->stack,
                                                  &ctx->taskStorage,
                                                  resolvedCore);
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

static uint64_t sensorarrayMeasureFdcSummarySpanUs(uint64_t startA,
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

static uint64_t sensorarrayMeasureFdcSummaryOverlapUs(uint64_t serialUs,
                                                      uint64_t spanUs)
{
    return (serialUs > spanUs) ? (serialUs - spanUs) : 0u;
}

void sensorarrayMeasureAccumulateRowEpochTiming(sensorarrayFdcTimingSummary_t *summary,
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
    summary->workerPreReleaseUs += rowTiming->workerPreReleaseUs;
    summary->workerWaitPrimaryUs += rowTiming->workerWaitPrimaryUs;
    summary->workerWaitSecondaryUs += rowTiming->workerWaitSecondaryUs;
    summary->workerJoinUs += rowTiming->workerJoinUs;
    summary->frameMaskUpdateUs += rowTiming->frameMaskUpdateUs;
    summary->frameBookkeepingUs += rowTiming->frameBookkeepingUs;
    summary->workerLateDoneUs += rowTiming->workerLateDoneUs;
    summary->workerIdleAfterTimeoutUs += rowTiming->workerIdleAfterTimeoutUs;
    summary->primaryWorkerRunUs += rowTiming->primaryWorkerRunUs;
    summary->secondaryWorkerRunUs += rowTiming->secondaryWorkerRunUs;
    if (rowTiming->primaryWorkerRunUs > summary->primaryWorkerRunMaxUs) {
        summary->primaryWorkerRunMaxUs = rowTiming->primaryWorkerRunUs;
    }
    if (rowTiming->secondaryWorkerRunUs > summary->secondaryWorkerRunMaxUs) {
        summary->secondaryWorkerRunMaxUs = rowTiming->secondaryWorkerRunUs;
    }
    summary->workerStartSkewUs += rowTiming->workerStartSkewUs;
    summary->workerDoneSkewUs += rowTiming->workerDoneSkewUs;
    if (rowTiming->workerStartSkewUs > summary->workerStartSkewMaxUs) {
        summary->workerStartSkewMaxUs = rowTiming->workerStartSkewUs;
    }
    if (rowTiming->workerDoneSkewUs > summary->workerDoneSkewMaxUs) {
        summary->workerDoneSkewMaxUs = rowTiming->workerDoneSkewUs;
    }
    summary->primaryFirstI2cStartUs += rowTiming->primaryFirstI2cStartUs;
    summary->secondaryFirstI2cStartUs += rowTiming->secondaryFirstI2cStartUs;
    summary->primaryMinusSecondaryStartUs += rowTiming->primaryMinusSecondaryStartUs;
    summary->primaryMinusSecondaryDoneUs += rowTiming->primaryMinusSecondaryDoneUs;
    uint64_t waitSpanUs = sensorarrayMeasureFdcSummarySpanUs(primaryTiming->readyBeginUs,
                                                             primaryTiming->drdyUs,
                                                             secondaryTiming->readyBeginUs,
                                                             secondaryTiming->drdyUs);
    uint64_t readSpanUs = sensorarrayMeasureFdcSummarySpanUs(primaryTiming->readStartUs,
                                                             primaryTiming->readDoneUs,
                                                             secondaryTiming->readStartUs,
                                                             secondaryTiming->readDoneUs);
    summary->waitSpanUs += waitSpanUs;
    summary->readSpanUs += readSpanUs;
    summary->waitOverlapUs += sensorarrayMeasureFdcSummaryOverlapUs(
        primaryTiming->waitReadyUs + secondaryTiming->waitReadyUs,
        waitSpanUs);
    summary->readOverlapUs += sensorarrayMeasureFdcSummaryOverlapUs(
        primaryTiming->readRawUs + secondaryTiming->readRawUs,
        readSpanUs);
    summary->readStartDeltaUsTotal += rowTiming->readStartDeltaUs;
    if (rowTiming->primaryWorkerRunUs != 0u || rowTiming->secondaryWorkerRunUs != 0u) {
        if (rowTiming->wpNormal) {
            summary->wpOkRowCount++;
        } else {
            summary->wpAnomRowCount++;
        }
    }
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
    summary->statusReadCountPrimary += primaryTiming->statusReadsBeforeIntbCount +
                                       primaryTiming->statusReadsPrecheckCount +
                                       primaryTiming->statusReadsAfterIntbCount +
                                       primaryTiming->statusReadsInFallbackCount +
                                       primaryTiming->statusAfterTimeoutCount;
    summary->statusReadCountSecondary += secondaryTiming->statusReadsBeforeIntbCount +
                                         secondaryTiming->statusReadsPrecheckCount +
                                         secondaryTiming->statusReadsAfterIntbCount +
                                         secondaryTiming->statusReadsInFallbackCount +
                                         secondaryTiming->statusAfterTimeoutCount;
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
    summary->fallbackSecondWaitUs += primaryTiming->fallbackSecondWaitUs +
                                     secondaryTiming->fallbackSecondWaitUs;
    summary->fallbackSecondWaitCount += primaryTiming->fallbackSecondWaitCount +
                                        secondaryTiming->fallbackSecondWaitCount;
    if (primaryTiming->fallbackSecondWaitMaxUs > summary->fallbackSecondWaitMaxUs) {
        summary->fallbackSecondWaitMaxUs = primaryTiming->fallbackSecondWaitMaxUs;
    }
    if (secondaryTiming->fallbackSecondWaitMaxUs > summary->fallbackSecondWaitMaxUs) {
        summary->fallbackSecondWaitMaxUs = secondaryTiming->fallbackSecondWaitMaxUs;
    }
    summary->directDataReadCount +=
        primaryTiming->directDataReadCount + secondaryTiming->directDataReadCount;
    summary->directDataFallbackCount +=
        primaryTiming->directDataFallbackCount + secondaryTiming->directDataFallbackCount;
    summary->directDataFallbackReasonMask |=
        primaryTiming->directDataFallbackReasonMask |
        secondaryTiming->directDataFallbackReasonMask;
    summary->statusSavedReadCount +=
        primaryTiming->statusSavedReadCount + secondaryTiming->statusSavedReadCount;
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
    summary->statusReadUsPrimaryTotal += primaryTiming->statusReadUs;
    summary->statusReadUsSecondaryTotal += secondaryTiming->statusReadUs;
    summary->dataReadUsPrimaryTotal += primaryTiming->dataReadUs;
    summary->dataReadUsSecondaryTotal += secondaryTiming->dataReadUs;
    summary->alreadyLowPrimaryCount += primaryTiming->alreadyLowCount;
    summary->alreadyLowSecondaryCount += secondaryTiming->alreadyLowCount;
    if (primaryTiming->maxWaitReadyUs > summary->maxWaitReadyUs) {
        summary->maxWaitReadyUs = primaryTiming->maxWaitReadyUs;
    }
    if (secondaryTiming->maxWaitReadyUs > summary->maxWaitReadyUs) {
        summary->maxWaitReadyUs = secondaryTiming->maxWaitReadyUs;
    }
    if (primaryTiming->maxWaitReadyUs > summary->maxWaitReadyUsPrimary) {
        summary->maxWaitReadyUsPrimary = primaryTiming->maxWaitReadyUs;
    }
    if (secondaryTiming->maxWaitReadyUs > summary->maxWaitReadyUsSecondary) {
        summary->maxWaitReadyUsSecondary = secondaryTiming->maxWaitReadyUs;
    }
    if (primaryTiming->maxI2cReadUs > summary->maxI2cReadUs) {
        summary->maxI2cReadUs = primaryTiming->maxI2cReadUs;
    }
    if (secondaryTiming->maxI2cReadUs > summary->maxI2cReadUs) {
        summary->maxI2cReadUs = secondaryTiming->maxI2cReadUs;
    }
}

esp_err_t sensorarrayMeasureSelectFdcRowWhileSleeping(sensorarrayState_t *state,
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
        rowTiming->routeSetTimestampUs = (uint64_t)esp_timer_get_time();
    }
    sensorarrayMeasureDebugPulse(CONFIG_SENSORARRAY_FDC_ROW_STROBE_GPIO);

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

esp_err_t sensorarrayMeasureReadFdcMatrixRowSerialEpoch(sensorarrayState_t *state,
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

void sensorarrayMeasureFdcLogWorkerTimeout(const sensorarrayFdcWorkerTrace_t *trace,
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

void __attribute__((unused)) sensorarrayMeasureFdcLogLateDone(const sensorarrayFdcWorkerTrace_t *trace)
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

void sensorarrayMeasureFdcDiscardWorkerResult(sensorarrayFdcWorkerResult_t *result,
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

void sensorarrayMeasureFdcDeferDeviceRepairAfterParallel(sensorarrayState_t *state,
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
    if (sensorarrayMeasureFdcShouldLogNormalFrame(frameSeq)) {
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
}

uint32_t sensorarrayMeasureFdcAbsDeltaUs(uint64_t a, uint64_t b)
{
    if (a == 0u || b == 0u) {
        return 0u;
    }
    uint64_t delta = (a > b) ? (a - b) : (b - a);
    return delta > UINT32_MAX ? UINT32_MAX : (uint32_t)delta;
}

uint64_t sensorarrayMeasureFdcSpanUs(uint64_t startA,
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

void sensorarrayMeasureFdcFinalizeParallelTiming(uint8_t row,
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

esp_err_t sensorarrayMeasureReadFdcMatrixRowParallelEpoch(sensorarrayState_t *state,
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

    (void)xEventGroupClearBits(s_fdcWorkerEvents,
                               SENSORARRAY_FDC_WORKER_ACK_ALL_BITS |
                               SENSORARRAY_FDC_WORKER_DONE_ALL_BITS);
    primaryCtx->job = NULL;
    secondaryCtx->job = NULL;
    primaryCtx->rowConfigPrepared = false;
    primaryCtx->preparedEpoch = 0u;
    primaryCtx->preparedErr = (int)ESP_ERR_INVALID_STATE;
    secondaryCtx->rowConfigPrepared = false;
    secondaryCtx->preparedEpoch = 0u;
    secondaryCtx->preparedErr = (int)ESP_ERR_INVALID_STATE;

    if (outWorkerTimeout) {
        *outWorkerTimeout = false;
    }

    const uint64_t rowStartUs = (uint64_t)esp_timer_get_time();
    uint32_t rowHardDeadlineUs = sensorarrayMeasureFdcRowDeviceWatchdogHardTimeoutUs();
    if (CONFIG_SENSORARRAY_FDC_INTB_DIRECT_DATA_ENABLE && rowHardDeadlineUs <= UINT32_MAX / 2u) {
        rowHardDeadlineUs *= 2u;
    }
    uint64_t workerDeadlineUs = rowStartUs + rowHardDeadlineUs;
    uint32_t generation =
        (primaryCtx->generation > secondaryCtx->generation) ?
        primaryCtx->generation : secondaryCtx->generation;
    generation++;
    if (generation == 0u) {
        generation = 1u;
    }
    primaryCtx->generation = generation;
    secondaryCtx->generation = generation;

    sensorarrayFdcWorkerResult_t primaryResult = {
        .err = ESP_ERR_TIMEOUT,
        .frameSeq = frameSeq,
        .row = row,
        .devId = SENSORARRAY_FDC_DEV_PRIMARY,
        .epochId = epochId,
        .generation = generation,
    };
    sensorarrayFdcWorkerResult_t secondaryResult = {
        .err = ESP_ERR_TIMEOUT,
        .frameSeq = frameSeq,
        .row = row,
        .devId = SENSORARRAY_FDC_DEV_SECONDARY,
        .epochId = epochId,
        .generation = generation,
    };
    sensorarrayFdcWorkerTrace_t primaryTrace = {
        .frameSeq = frameSeq,
        .row = row,
        .epochId = epochId,
        .devId = SENSORARRAY_FDC_DEV_PRIMARY,
        .workerDeadlineUs = workerDeadlineUs,
        .rowHardDeadlineUs = rowHardDeadlineUs,
        .generation = generation,
        .err = ESP_ERR_TIMEOUT,
    };
    sensorarrayFdcWorkerTrace_t secondaryTrace = {
        .frameSeq = frameSeq,
        .row = row,
        .epochId = epochId,
        .devId = SENSORARRAY_FDC_DEV_SECONDARY,
        .workerDeadlineUs = workerDeadlineUs,
        .rowHardDeadlineUs = rowHardDeadlineUs,
        .generation = generation,
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
        .rowDeviceBudgetUs = rowHardDeadlineUs,
        .generation = generation,
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
        .rowDeviceBudgetUs = rowHardDeadlineUs,
        .generation = generation,
    };

    TickType_t syncTicks = pdMS_TO_TICKS((uint32_t)CONFIG_SENSORARRAY_FDC_WORKER_SYNC_TIMEOUT_MS);
    if (syncTicks == 0) {
        syncTicks = 1;
    }

    int64_t sleepStartUs = esp_timer_get_time();
    int64_t queueStartUs = esp_timer_get_time();
    primaryCtx->job = &primaryJob;
    secondaryCtx->job = &secondaryJob;
    BaseType_t primaryQueued = xTaskNotifyGive(primaryCtx->task);
    BaseType_t secondaryQueued = xTaskNotifyGive(secondaryCtx->task);
    primaryTrace.queueSendUs = (uint32_t)sensorarrayMeasureElapsedUs(queueStartUs);
    secondaryTrace.queueSendUs = 0u;
    rowTiming->workerQueueSendUs = primaryTrace.queueSendUs + secondaryTrace.queueSendUs;
    if (primaryQueued != pdTRUE || secondaryQueued != pdTRUE) {
        if (outWorkerTimeout) {
            *outWorkerTimeout = true;
        }
        if (primaryQueued != pdTRUE) {
            sensorarrayMeasureFdcLogWorkerTimeout(&primaryTrace, "queue_send", NULL);
        }
        if (secondaryQueued != pdTRUE) {
            sensorarrayMeasureFdcLogWorkerTimeout(&secondaryTrace, "queue_send", NULL);
        }
        if (primaryQueued == pdTRUE) {
            (void)xTaskNotifyGive(primaryCtx->task);
        }
        if (secondaryQueued == pdTRUE) {
            (void)xTaskNotifyGive(secondaryCtx->task);
        }
        (void)xEventGroupWaitBits(s_fdcWorkerEvents,
                                  SENSORARRAY_FDC_WORKER_DONE_ALL_BITS,
                                  pdTRUE,
                                  pdTRUE,
                                  portMAX_DELAY);
        printf("WP,r=%u,e=%lu,q=%lu,ack=0,sg=0,ps=0,pe=0,ws=0,we=0,jd=0,ov=0,pr=0,wr=0,mode=fallback,reason=queue_fail,pq=%u,sq=%u\n",
               (unsigned)row,
               (unsigned long)epochId,
               (unsigned long)rowTiming->workerQueueSendUs,
               primaryQueued == pdTRUE ? 1u : 0u,
               secondaryQueued == pdTRUE ? 1u : 0u);
        return ESP_ERR_TIMEOUT;
    }

    int64_t sleepAckStartUs = esp_timer_get_time();
    EventBits_t ackBits = xEventGroupWaitBits(s_fdcWorkerEvents,
                                              SENSORARRAY_FDC_WORKER_ACK_ALL_BITS,
                                              pdTRUE,
                                              pdTRUE,
                                              syncTicks);
    uint32_t ackWaitUs = (uint32_t)sensorarrayMeasureElapsedUs(sleepAckStartUs);
    BaseType_t primarySleep =
        (ackBits & SENSORARRAY_FDC_WORKER_ACK_PRIMARY_BIT) != 0u ? pdTRUE : pdFALSE;
    BaseType_t secondarySleep =
        (ackBits & SENSORARRAY_FDC_WORKER_ACK_SECONDARY_BIT) != 0u ? pdTRUE : pdFALSE;
    primaryTrace.sleepAckWaitUs = ackWaitUs;
    secondaryTrace.sleepAckWaitUs = ackWaitUs;
    primaryTrace.sleepAckReceived = primarySleep == pdTRUE;
    secondaryTrace.sleepAckReceived = secondarySleep == pdTRUE;
    rowTiming->workerSleepAckWaitUs = ackWaitUs;
    rowTiming->sleepBeforeRowSwitchUs = sensorarrayMeasureElapsedUs(sleepStartUs);
    if (primarySleep != pdTRUE || secondarySleep != pdTRUE ||
        primaryResult.err != ESP_OK || secondaryResult.err != ESP_OK) {
        if (outWorkerTimeout) {
            *outWorkerTimeout = true;
        }
        (void)xTaskNotifyGive(primaryCtx->task);
        (void)xTaskNotifyGive(secondaryCtx->task);
        (void)xEventGroupWaitBits(s_fdcWorkerEvents,
                                  SENSORARRAY_FDC_WORKER_DONE_ALL_BITS,
                                  pdTRUE,
                                  pdTRUE,
                                  portMAX_DELAY);
        if (primarySleep != pdTRUE || primaryResult.err != ESP_OK) {
            sensorarrayMeasureFdcLogWorkerTimeout(&primaryTrace, "ready_ack", &primaryResult);
        }
        if (secondarySleep != pdTRUE || secondaryResult.err != ESP_OK) {
            sensorarrayMeasureFdcLogWorkerTimeout(&secondaryTrace, "ready_ack", &secondaryResult);
        }
        printf("WP,r=%u,e=%lu,q=%lu,ack=%llu,sg=0,ps=0,pe=0,ws=0,we=0,jd=0,ov=0,pr=0,wr=0,mode=fallback,reason=ack_timeout,pa=%u,sa=%u,pe0=0x%lx,se0=0x%lx\n",
               (unsigned)row,
               (unsigned long)epochId,
               (unsigned long)rowTiming->workerQueueSendUs,
               (unsigned long long)(rowTiming->sleepBeforeRowSwitchUs),
               primarySleep == pdTRUE ? 1u : 0u,
               secondarySleep == pdTRUE ? 1u : 0u,
               (unsigned long)primaryResult.err,
               (unsigned long)secondaryResult.err);
        return (primaryResult.err != ESP_OK) ? primaryResult.err :
            ((secondaryResult.err != ESP_OK) ? secondaryResult.err : ESP_ERR_TIMEOUT);
    }

    sensorarrayMeasureFdcPrepareIntbEpoch(primaryCtx, epochId);
    sensorarrayMeasureFdcPrepareIntbEpoch(secondaryCtx, epochId);

    esp_err_t rowErr = sensorarrayMeasureSelectFdcRowWhileSleeping(state, row, epochId, frameSeq, rowTiming);
    if (rowErr != ESP_OK) {
        (void)xTaskNotifyGive(primaryCtx->task);
        (void)xTaskNotifyGive(secondaryCtx->task);
        (void)xEventGroupWaitBits(s_fdcWorkerEvents,
                                  SENSORARRAY_FDC_WORKER_DONE_ALL_BITS,
                                  pdTRUE,
                                  pdTRUE,
                                  portMAX_DELAY);
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
    if ((primaryPrepareErr != ESP_OK && primaryPrepareErr != ESP_ERR_NOT_FOUND) ||
        (secondaryPrepareErr != ESP_OK && secondaryPrepareErr != ESP_ERR_NOT_FOUND)) {
        (void)xTaskNotifyGive(primaryCtx->task);
        (void)xTaskNotifyGive(secondaryCtx->task);
        (void)xEventGroupWaitBits(s_fdcWorkerEvents,
                                  SENSORARRAY_FDC_WORKER_DONE_ALL_BITS,
                                  pdTRUE,
                                  pdTRUE,
                                  portMAX_DELAY);
        return primaryPrepareErr != ESP_OK ? primaryPrepareErr : secondaryPrepareErr;
    }

    int64_t startGiveStartUs = esp_timer_get_time();
    rowTiming->workerReleaseUs = (uint64_t)startGiveStartUs;
    rowTiming->workerPreReleaseUs =
        rowTiming->workerReleaseUs >= rowStartUs ?
        rowTiming->workerReleaseUs - rowStartUs : 0u;
    /*
     * Queueing, worker sleep entry, row routing, and first-use profile writes
     * happen before the read workers are released. They are coordinator work
     * and must remain visible in row wall timing, but they must not consume the
     * worker's ready/read watchdog budget. Rebase the join deadline at the
     * barrier release; each worker independently uses the same duration from
     * its actual run start.
     */
    workerDeadlineUs = rowTiming->workerReleaseUs + rowHardDeadlineUs;
    primaryTrace.workerDeadlineUs = workerDeadlineUs;
    secondaryTrace.workerDeadlineUs = workerDeadlineUs;
    (void)xTaskNotifyGive(primaryCtx->task);
    primaryTrace.startGiven = true;
    (void)xTaskNotifyGive(secondaryCtx->task);
    secondaryTrace.startGiven = true;
    rowTiming->workerStartGiveUs = sensorarrayMeasureElapsedUs(startGiveStartUs);

    uint32_t primaryExpectedReadyUs =
        state->fdcAppliedRow[SENSORARRAY_FDC_DEV_PRIMARY].profileSnapshot.autoscanRoundUs;
    uint32_t secondaryExpectedReadyUs =
        state->fdcAppliedRow[SENSORARRAY_FDC_DEV_SECONDARY].profileSnapshot.autoscanRoundUs;
    uint32_t expectedReadyUs = primaryExpectedReadyUs > secondaryExpectedReadyUs ?
        primaryExpectedReadyUs : secondaryExpectedReadyUs;
    if (expectedReadyUs != 0u) {
        sensorarrayAdsGapTryRun(state,
                                rowTiming->workerReleaseUs + expectedReadyUs,
                                frameSeq,
                                row);
    }

    int64_t joinStartUs = esp_timer_get_time();
    uint32_t joinBudgetUs = workerDeadlineUs > (uint64_t)joinStartUs ?
        (uint32_t)(workerDeadlineUs - (uint64_t)joinStartUs) : 0u;
    EventBits_t doneBits = xEventGroupWaitBits(
        s_fdcWorkerEvents,
        SENSORARRAY_FDC_WORKER_DONE_ALL_BITS,
        pdFALSE,
        pdTRUE,
        sensorarrayMeasureFdcTicksForUs(joinBudgetUs));
    rowTiming->workerDoneWaitUs = sensorarrayMeasureElapsedUs(joinStartUs);
    rowTiming->parallelJoinWaitUs = rowTiming->workerDoneWaitUs;
    BaseType_t primaryDone =
        (doneBits & SENSORARRAY_FDC_WORKER_DONE_PRIMARY_BIT) != 0u ? pdTRUE : pdFALSE;
    BaseType_t secondaryDone =
        (doneBits & SENSORARRAY_FDC_WORKER_DONE_SECONDARY_BIT) != 0u ? pdTRUE : pdFALSE;
    primaryTrace.doneWaitUs = (uint32_t)rowTiming->workerDoneWaitUs;
    secondaryTrace.doneWaitUs = (uint32_t)rowTiming->workerDoneWaitUs;
    primaryTrace.doneReceived = primaryDone == pdTRUE;
    secondaryTrace.doneReceived = secondaryDone == pdTRUE;
    rowTiming->workerWaitPrimaryUs = primaryTrace.doneWaitUs;
    rowTiming->workerWaitSecondaryUs = secondaryTrace.doneWaitUs;
    int64_t workerJoinStartUs = esp_timer_get_time();
    bool primaryLate = primaryDone != pdTRUE;
    bool secondaryLate = secondaryDone != pdTRUE;
    if (primaryLate || secondaryLate) {
        int64_t idleStartUs = esp_timer_get_time();
        doneBits = xEventGroupWaitBits(s_fdcWorkerEvents,
                                       SENSORARRAY_FDC_WORKER_DONE_ALL_BITS,
                                       pdFALSE,
                                       pdTRUE,
                                       portMAX_DELAY);
        uint32_t lateWaitUs = (uint32_t)sensorarrayMeasureElapsedUs(idleStartUs);
        rowTiming->workerIdleAfterTimeoutUs += lateWaitUs;
        rowTiming->workerLateDoneUs += lateWaitUs;
        if (primaryLate) {
            primaryTrace.waitWorkerIdleAfterTimeoutUs = lateWaitUs;
            primaryTrace.doneReceived =
                (doneBits & SENSORARRAY_FDC_WORKER_DONE_PRIMARY_BIT) != 0u;
            primaryTrace.lateDone = primaryTrace.doneReceived;
            rowTiming->workerLateDoneCount++;
            rowTiming->workerTimeoutCount++;
            sensorarrayMeasureFdcLogWorkerTimeout(&primaryTrace, "worker_deadline", &primaryResult);
        }
        if (secondaryLate) {
            secondaryTrace.waitWorkerIdleAfterTimeoutUs = lateWaitUs;
            secondaryTrace.doneReceived =
                (doneBits & SENSORARRAY_FDC_WORKER_DONE_SECONDARY_BIT) != 0u;
            secondaryTrace.lateDone = secondaryTrace.doneReceived;
            rowTiming->workerLateDoneCount++;
            rowTiming->workerTimeoutCount++;
            sensorarrayMeasureFdcLogWorkerTimeout(&secondaryTrace, "worker_deadline", &secondaryResult);
        }
        if (outWorkerTimeout) {
            *outWorkerTimeout = true;
        }
    }
    (void)xEventGroupClearBits(s_fdcWorkerEvents, SENSORARRAY_FDC_WORKER_DONE_ALL_BITS);
    primaryCtx->job = NULL;
    secondaryCtx->job = NULL;

    bool primaryStale =
        primaryResult.frameSeq != frameSeq ||
        primaryResult.row != row ||
        primaryResult.epochId != epochId ||
        primaryResult.devId != SENSORARRAY_FDC_DEV_PRIMARY ||
        primaryResult.generation != generation;
    if (primaryStale) {
        rowTiming->staleResultDiscardedCount++;
        printf("FDC_WORKER_EPOCH_MISMATCH_DISCARD,seq=%lu,row=%u,epoch=%lu,device=primary,resultSeq=%lu,resultRow=%u,resultEpoch=%lu,resultDev=%s,g=%lu,resultG=%lu\n",
               (unsigned long)frameSeq,
               (unsigned)row,
               (unsigned long)epochId,
               (unsigned long)primaryResult.frameSeq,
               (unsigned)primaryResult.row,
               (unsigned long)primaryResult.epochId,
               sensorarrayMeasureFdcDeviceName(primaryResult.devId),
               (unsigned long)generation,
               (unsigned long)primaryResult.generation);
        sensorarrayMeasureFdcDiscardWorkerResult(&primaryResult,
                                                 primarySamples,
                                                 &primaryTrace,
                                                 "stale_result");
    }
    bool secondaryStale =
        secondaryResult.frameSeq != frameSeq ||
        secondaryResult.row != row ||
        secondaryResult.epochId != epochId ||
        secondaryResult.devId != SENSORARRAY_FDC_DEV_SECONDARY ||
        secondaryResult.generation != generation;
    if (secondaryStale) {
        rowTiming->staleResultDiscardedCount++;
        printf("FDC_WORKER_EPOCH_MISMATCH_DISCARD,seq=%lu,row=%u,epoch=%lu,device=secondary,resultSeq=%lu,resultRow=%u,resultEpoch=%lu,resultDev=%s,g=%lu,resultG=%lu\n",
               (unsigned long)frameSeq,
               (unsigned)row,
               (unsigned long)epochId,
               (unsigned long)secondaryResult.frameSeq,
               (unsigned)secondaryResult.row,
               (unsigned long)secondaryResult.epochId,
               sensorarrayMeasureFdcDeviceName(secondaryResult.devId),
               (unsigned long)generation,
               (unsigned long)secondaryResult.generation);
        sensorarrayMeasureFdcDiscardWorkerResult(&secondaryResult,
                                                 secondarySamples,
                                                 &secondaryTrace,
                                                 "stale_result");
    }

    bool primaryGood =
        !primaryStale && !primaryLate &&
        sensorarrayMeasureFdcWorkerResultIsGood(&primaryResult,
                                                row,
                                                epochId,
                                                SENSORARRAY_FDC_DEV_PRIMARY);
    bool secondaryGood =
        !secondaryStale && !secondaryLate &&
        sensorarrayMeasureFdcWorkerResultIsGood(&secondaryResult,
                                                row,
                                                epochId,
                                                SENSORARRAY_FDC_DEV_SECONDARY);
    if ((!primaryGood || !secondaryGood) &&
        sensorarrayMeasureFdcShouldLogNormalFrame(frameSeq)) {
        const char *fallbackReason = (primaryLate || secondaryLate) ? "worker_deadline" :
            ((primaryStale || secondaryStale) ? "stale_result" : "device_result");
        printf("FDC_PARALLEL_FALLBACK,reason=%s,row=%u,epoch=%lu,primaryGood=%u,secondaryGood=%u,primaryErr=0x%lx,secondaryErr=0x%lx,primaryLate=%u,secondaryLate=%u,primaryStale=%u,secondaryStale=%u,g=%lu,workerJoinUs=%llu\n",
               fallbackReason,
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
               (unsigned long)generation,
               (unsigned long long)rowTiming->workerDoneWaitUs);
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
    uint64_t primaryWorkerStartUs = primaryTrace.workerStartUs;
    uint64_t primaryWorkerEndUs = primaryTrace.workerEndUs;
    uint64_t secondaryWorkerStartUs = secondaryTrace.workerStartUs;
    uint64_t secondaryWorkerEndUs = secondaryTrace.workerEndUs;
    uint64_t overlapStartUs = primaryWorkerStartUs > secondaryWorkerStartUs ?
        primaryWorkerStartUs : secondaryWorkerStartUs;
    uint64_t overlapEndUs = primaryWorkerEndUs < secondaryWorkerEndUs ?
        primaryWorkerEndUs : secondaryWorkerEndUs;
    uint64_t overlapUs = (primaryWorkerStartUs != 0u && secondaryWorkerStartUs != 0u &&
                          overlapEndUs > overlapStartUs) ?
        (overlapEndUs - overlapStartUs) : 0u;
    uint64_t primaryRunUs = primaryWorkerEndUs > primaryWorkerStartUs ?
        (primaryWorkerEndUs - primaryWorkerStartUs) : 0u;
    uint64_t secondaryRunUs = secondaryWorkerEndUs > secondaryWorkerStartUs ?
        (secondaryWorkerEndUs - secondaryWorkerStartUs) : 0u;
    uint64_t latestWorkerEndUs = primaryWorkerEndUs > secondaryWorkerEndUs ?
        primaryWorkerEndUs : secondaryWorkerEndUs;
    rowTiming->dualBusWaitUs =
        (latestWorkerEndUs > (uint64_t)startGiveStartUs) ?
        (latestWorkerEndUs - (uint64_t)startGiveStartUs) : 0u;
    uint64_t startSkewUs = sensorarrayMeasureFdcAbsDeltaUs(primaryWorkerStartUs,
                                                           secondaryWorkerStartUs);
    uint64_t doneSkewUs = sensorarrayMeasureFdcAbsDeltaUs(primaryWorkerEndUs,
                                                          secondaryWorkerEndUs);
    rowTiming->primaryWorkerRunUs = primaryRunUs;
    rowTiming->secondaryWorkerRunUs = secondaryRunUs;
    rowTiming->workerStartSkewUs = startSkewUs;
    rowTiming->workerDoneSkewUs = doneSkewUs;
    rowTiming->primaryFirstI2cStartUs =
        (primaryTiming->sleepExitStartUs >= rowTiming->workerReleaseUs) ?
        (primaryTiming->sleepExitStartUs - rowTiming->workerReleaseUs) : 0u;
    rowTiming->secondaryFirstI2cStartUs =
        (secondaryTiming->sleepExitStartUs >= rowTiming->workerReleaseUs) ?
        (secondaryTiming->sleepExitStartUs - rowTiming->workerReleaseUs) : 0u;
    rowTiming->primaryMinusSecondaryStartUs =
        (primaryWorkerStartUs != 0u && secondaryWorkerStartUs != 0u) ?
        (int32_t)((int64_t)primaryWorkerStartUs - (int64_t)secondaryWorkerStartUs) : 0;
    rowTiming->primaryMinusSecondaryDoneUs =
        (primaryWorkerEndUs != 0u && secondaryWorkerEndUs != 0u) ?
        (int32_t)((int64_t)primaryWorkerEndUs - (int64_t)secondaryWorkerEndUs) : 0;

    const char *wpReason = "ok";
    if (primaryLate || secondaryLate) {
        wpReason = "worker_deadline";
    } else if (primaryStale || secondaryStale) {
        wpReason = "stale_done_or_join_bug";
    } else if (startSkewUs > 1000u) {
        wpReason = "worker_late_start";
    } else if (rowTiming->workerDoneWaitUs > 1000u &&
               primaryWorkerEndUs != 0u &&
               secondaryWorkerEndUs != 0u &&
               latestWorkerEndUs <= (uint64_t)joinStartUs) {
        wpReason = "stale_done_or_join_bug";
    } else if (primaryRunUs != 0u && secondaryRunUs != 0u &&
               overlapUs < ((primaryRunUs < secondaryRunUs ? primaryRunUs : secondaryRunUs) / 4u)) {
        wpReason = "i2c_lock_serialized";
    }
    rowTiming->wpNormal = strcmp(wpReason, "ok") == 0;
    if (CONFIG_SENSORARRAY_FDC_VERBOSE_SCAN_LOG ||
        strcmp(wpReason, "ok") != 0) {
        printf("WP,r=%u,e=%lu,q=%lu,ack=%llu,sg=%llu,ps=%llu,pe=%llu,ws=%llu,we=%llu,jd=%llu,ov=%llu,pr=%llu,wr=%llu,mode=par,reason=%s\n",
               (unsigned)row,
               (unsigned long)epochId,
               (unsigned long)rowTiming->workerQueueSendUs,
               (unsigned long long)rowTiming->sleepBeforeRowSwitchUs,
               (unsigned long long)((uint64_t)startGiveStartUs - rowStartUs),
               (unsigned long long)(primaryWorkerStartUs >= rowStartUs ? primaryWorkerStartUs - rowStartUs : 0u),
               (unsigned long long)(primaryWorkerEndUs >= rowStartUs ? primaryWorkerEndUs - rowStartUs : 0u),
               (unsigned long long)(secondaryWorkerStartUs >= rowStartUs ? secondaryWorkerStartUs - rowStartUs : 0u),
               (unsigned long long)(secondaryWorkerEndUs >= rowStartUs ? secondaryWorkerEndUs - rowStartUs : 0u),
               (unsigned long long)rowTiming->workerDoneWaitUs,
               (unsigned long long)overlapUs,
               (unsigned long long)primaryRunUs,
               (unsigned long long)secondaryRunUs,
               wpReason);
    }
    rowTiming->workerJoinUs = sensorarrayMeasureElapsedUs(workerJoinStartUs);
    sensorarrayMeasureFdcFinalizeParallelTiming(row,
                                                epochId,
                                                primaryTiming,
                                                secondaryTiming,
                                                rowTiming,
                                                primaryResult.err,
                                                secondaryResult.err);
    return ESP_OK;
}

esp_err_t __attribute__((unused)) sensorarrayMeasureReadFdcRuntimeChannelConfigs(sensorarrayState_t *state,
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

esp_err_t __attribute__((unused)) sensorarrayMeasureDiscardFdcAutoscanRow(sensorarrayState_t *state,
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

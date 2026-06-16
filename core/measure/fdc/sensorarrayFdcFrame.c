#include "sensorarrayFdcInternal.h"

// Split from core/measure/sensorarrayMeasure.c to keep measurement domains isolated.
void sensorarrayMeasureMarkFdcMatrixCellEx(sensorarrayFdcMatrixFrame_t *frame,
                                                  uint8_t sIndex,
                                                  uint8_t dIndex,
                                                  uint32_t raw28,
                                                  double freqHz,
                                                  const sensorarrayFdcRuntimeChannelConfig_t *config,
                                                  bool fresh,
                                                  bool valid,
                                                  bool warning,
                                                  bool error,
                                                  bool notReady,
                                                  bool zeroBeforeReady,
                                                  bool zeroAfterDrdy,
                                                  bool i2cError);

void sensorarrayMeasureFillFdcMatrixRow(sensorarrayFdcMatrixFrame_t *frame,
                                               uint8_t sIndex,
                                               const sensorarrayFdcAutoscanSamples_t *primary,
                                               const sensorarrayFdcAutoscanSamples_t *secondary,
                                               const sensorarrayFdcRuntimeChannelConfig_t configs[2][4],
                                               uint8_t *outValidMask8,
                                               uint8_t *outWarnMask8,
                                               uint8_t *outErrorMask8)
{
    uint8_t validMask8 = 0u;
    uint8_t warnMask8 = 0u;
    uint8_t errorMask8 = 0u;
    const sensorarrayFdcAutoscanSamples_t *samplesByHalf[2] = {primary, secondary};

    for (uint8_t half = 0u; half < 2u; ++half) {
        const sensorarrayFdcAutoscanSamples_t *samples = samplesByHalf[half];
        for (uint8_t ch = 0u; ch < 4u; ++ch) {
            uint8_t dIndex = (uint8_t)(1u + ch + (half * 4u));
            bool fresh = samples && samples->fresh[ch];
            bool valid = samples && samples->valid[ch];
            bool warning = samples && samples->amplitudeWarning[ch];
            bool notReady = samples && samples->notReady[ch];
            bool zeroBeforeReady = samples && samples->zeroBeforeReady[ch];
            bool zeroAfterDrdy = samples && samples->zeroAfterDrdy[ch];
            bool i2cError = samples && samples->i2cError[ch];
            bool softInvalid = samples && samples->softInvalid[ch];
            bool hardInvalid = samples && samples->hardInvalid[ch];
            const sensorarrayFdcRuntimeChannelConfig_t *config = configs ? &configs[half][ch] : NULL;
            bool configOk = config && config->valid;
            bool severeFault = hardInvalid ||
                               !samples ||
                               (!fresh && !softInvalid) ||
                               (!valid && !softInvalid) ||
                               !configOk ||
                               notReady ||
                               zeroBeforeReady ||
                               zeroAfterDrdy ||
                               (samples && (samples->i2cError[ch] ||
                                            samples->watchdogFault[ch] ||
                                            samples->saturated[ch] ||
                                            (fresh && samples->raw28[ch] == 0u)));
            bool error = severeFault && !softInvalid;
            uint32_t raw28 = samples ? samples->raw28[ch] : 0u;
            double freqHz = (valid && configOk) ?
                sensorarrayMeasureFdcRaw28ToFreqHz(raw28, config->effectiveFclkHz, config->clockDividers) :
                0.0;
            bool frameValid = valid && freqHz > 0.0;
            sensorarrayMeasureMarkFdcMatrixCellEx(frame,
                                                  sIndex,
                                                  dIndex,
                                                  raw28,
                                                  freqHz,
                                                  config,
                                                  fresh,
                                                  frameValid,
                                                  warning,
                                                  error,
                                                  notReady,
                                                  zeroBeforeReady,
                                                  zeroAfterDrdy,
                                                  i2cError);
            if (frameValid) {
                validMask8 |= (uint8_t)(1u << (dIndex - 1u));
            }
            if (warning) {
                warnMask8 |= (uint8_t)(1u << (dIndex - 1u));
            }
            if (error) {
                errorMask8 |= (uint8_t)(1u << (dIndex - 1u));
            }
            if (samples && samples->unreadWithoutDrdy && frame &&
                frame->unreadWithoutDrdyCount < UINT8_MAX) {
                frame->unreadWithoutDrdyCount++;
            }
            if (softInvalid && frame && frame->softInvalidCount < UINT8_MAX) {
                frame->softInvalidCount++;
            }
            if ((hardInvalid || error) && frame && frame->hardInvalidCount < UINT8_MAX) {
                frame->hardInvalidCount++;
            }
            if (samples && samples->staleUnreadDrain && frame &&
                frame->staleUnreadDrainCount < UINT8_MAX) {
                frame->staleUnreadDrainCount++;
            }
            if (samples && samples->readyStatusReadable && !samples->dataReadAttempted && frame &&
                frame->diagReadyButRejectedCount < UINT8_MAX) {
                frame->diagReadyButRejectedCount++;
            }
            if (samples && samples->originalIntbMiss && samples->readyStatusReadable && frame &&
                frame->intbMissButStatusReadyCount < UINT8_MAX) {
                frame->intbMissButStatusReadyCount++;
            }
            if (samples && samples->statusFallbackUsed && frame &&
                frame->statusFallbackAcceptedCount < UINT8_MAX) {
                frame->statusFallbackAcceptedCount++;
            }
            if (samples && samples->waitBudgetTooShort && frame &&
                frame->waitBudgetTooShortCount < UINT8_MAX) {
                frame->waitBudgetTooShortCount++;
            }
            if (samples && samples->levelLowButEdgeMiss && frame &&
                frame->levelLowButEdgeMissCount < UINT8_MAX) {
                frame->levelLowButEdgeMissCount++;
            }
            if (samples && samples->readyStatusReadable && !samples->dataReadAttempted && frame &&
                frame->actualDataReadSkippedDespiteStatusReadyCount < UINT8_MAX) {
                frame->actualDataReadSkippedDespiteStatusReadyCount++;
            }
        }
    }

    if (outValidMask8) {
        *outValidMask8 = validMask8;
    }
    if (outWarnMask8) {
        *outWarnMask8 = warnMask8;
    }
    if (outErrorMask8) {
        *outErrorMask8 = errorMask8;
    }
}

void sensorarrayMeasureAccumulateFdcHealth(sensorarrayFdcFrameHealth_t *health,
                                                  uint8_t sIndex,
                                                  const sensorarrayFdcAutoscanSamples_t *primary,
                                                  const sensorarrayFdcAutoscanSamples_t *secondary,
                                                  const sensorarrayFdcRuntimeChannelConfig_t configs[2][4],
                                                  const sensorarrayFdcMatrixFrame_t *frame)
{
    if (!health || !sensorarrayMatrixIndexIsValid(sIndex, 1u)) {
        return;
    }

    const sensorarrayFdcAutoscanSamples_t *samplesByHalf[2] = {primary, secondary};
    for (uint8_t half = 0u; half < 2u; ++half) {
        const sensorarrayFdcAutoscanSamples_t *samples = samplesByHalf[half];
        for (uint8_t ch = 0u; ch < 4u; ++ch) {
            uint8_t dIndex = (uint8_t)(1u + ch + (half * 4u));
            size_t matrixIndex = sensorarrayMatrixIndex(sIndex, dIndex);
            uint8_t s0 = (uint8_t)(sIndex - 1u);
            uint8_t d0 = (uint8_t)(dIndex - 1u);
            bool valid = samples && samples->valid[ch] && frame &&
                         ((frame->validMask & (1ULL << matrixIndex)) != 0u);
            bool fresh = samples && samples->fresh[ch];
            if (valid) {
                health->validSeen[s0][d0] = true;
                health->lastRaw28[s0][d0] = samples->raw28[ch];
                health->lastFreqHz[s0][d0] = frame ? frame->freqHz[matrixIndex] : 0.0;
            } else {
                health->invalidSeen[s0][d0] = true;
            }
            health->amplitudeWarningSeen[s0][d0] =
                health->amplitudeWarningSeen[s0][d0] || (samples && samples->amplitudeWarning[ch]);
            health->freshAmplitudeWarningSeen[s0][d0] =
                health->freshAmplitudeWarningSeen[s0][d0] ||
                (samples && samples->freshAmplitudeWarning[ch]);
            health->staleAmplitudeWarningSeen[s0][d0] =
                health->staleAmplitudeWarningSeen[s0][d0] ||
                (samples && samples->staleAmplitudeWarning[ch]);
            health->transientAmplitudeWarningSeen[s0][d0] =
                health->transientAmplitudeWarningSeen[s0][d0] ||
                (samples && samples->transientAmplitudeWarning[ch]);
            health->watchdogSeen[s0][d0] =
                health->watchdogSeen[s0][d0] || (samples && samples->watchdogFault[ch]);
            health->saturatedSeen[s0][d0] =
                health->saturatedSeen[s0][d0] || (samples && samples->saturated[ch]);
            health->zeroRawSeen[s0][d0] =
                health->zeroRawSeen[s0][d0] || (samples && samples->zeroAfterDrdy[ch]);
            health->notReadySeen[s0][d0] =
                health->notReadySeen[s0][d0] || (samples && samples->notReady[ch]);
            health->zeroBeforeReadySeen[s0][d0] =
                health->zeroBeforeReadySeen[s0][d0] || (samples && samples->zeroBeforeReady[ch]);
            health->zeroAfterDrdySeen[s0][d0] =
                health->zeroAfterDrdySeen[s0][d0] || (samples && samples->zeroAfterDrdy[ch]);
            health->softInvalidSeen[s0][d0] =
                health->softInvalidSeen[s0][d0] || (samples && samples->softInvalid[ch]);
            health->hardInvalidSeen[s0][d0] =
                health->hardInvalidSeen[s0][d0] || (samples && samples->hardInvalid[ch]);
            health->staleUnreadDrainSeen[s0][d0] =
                health->staleUnreadDrainSeen[s0][d0] ||
                (samples && samples->staleUnreadDrain);
            health->placeholderZeroSeen[s0][d0] =
                health->placeholderZeroSeen[s0][d0] || (!fresh && (!samples || samples->raw28[ch] == 0u));
            health->i2cErrorSeen[s0][d0] =
                health->i2cErrorSeen[s0][d0] || (samples && samples->i2cError[ch]);
            if (configs) {
                const sensorarrayFdcRuntimeChannelConfig_t *config = &configs[half][ch];
                health->clockDividers[s0][d0] = config->clockDividers;
                health->driveCurrent[s0][d0] = config->driveCurrent;
                health->deglitchCode[s0][d0] = config->deglitchCode;
                health->effectiveFclkHz[s0][d0] = config->effectiveFclkHz;
            }
        }
    }
}

uint32_t sensorarrayMeasureUpdateFdcRuntimeProfiles(sensorarrayState_t *state,
                                                   const sensorarrayFdcFrameHealth_t *health,
                                                   uint8_t activeRows)
{
    if (!state || !health) {
        return 0u;
    }
    if (activeRows < 1u || activeRows > SENSORARRAY_MATRIX_ROWS) {
        activeRows = SENSORARRAY_MATRIX_ROWS;
    }

    uint32_t threshold = (uint32_t)CONFIG_SENSORARRAY_FDC_RESCUE_HARD_ERROR_THRESHOLD;
    if (threshold == 0u) {
        threshold = 1u;
    }

    uint32_t amplitudeThreshold = (uint32_t)CONFIG_SENSORARRAY_FDC_WARNING_REAPPLY_THRESHOLD;
    if (amplitudeThreshold == 0u) {
        amplitudeThreshold = 1u;
    }

    int64_t nowUs = esp_timer_get_time();
    uint32_t touched = 0u;
    for (uint8_t s = 1u; s <= activeRows; ++s) {
        for (uint8_t d = 1u; d <= SENSORARRAY_MATRIX_COLS; ++d) {
            touched++;
            uint8_t s0 = (uint8_t)(s - 1u);
            uint8_t d0 = (uint8_t)(d - 1u);
            uint8_t matrixIndex = (uint8_t)sensorarrayMatrixIndex(s, d);
            sensorarrayFdcCellTarget_t target = {0};
            if (!sensorarrayMeasureMakeFdcCellTarget(state, s, d, &target)) {
                continue;
            }

            sensorarrayFdcCellConfigCache_t *cache = sensorarrayMeasureGetFdcCellCache(state, &target);
            sensorarrayFdcDeviceState_t *fdcState = sensorarrayMeasureGetFdcState(state, target.devId);
            if (!cache || !fdcState || target.fdcChannel > (uint8_t)FDC2214_CH3) {
                continue;
            }

            sensorarrayFdcSweepProfile_t *profile = &fdcState->sweepProfile[target.fdcChannel];
            bool valid = health->validSeen[s0][d0];
            bool invalid = health->invalidSeen[s0][d0];
            bool amplitude = health->amplitudeWarningSeen[s0][d0];
            bool freshAmplitude = health->freshAmplitudeWarningSeen[s0][d0];
            bool staleAmplitude = health->staleAmplitudeWarningSeen[s0][d0];
            bool transientAmplitude = health->transientAmplitudeWarningSeen[s0][d0];
            bool runtimeNotReady = health->notReadySeen[s0][d0] ||
                                   health->zeroBeforeReadySeen[s0][d0];
            bool severeError = !health->softInvalidSeen[s0][d0] &&
                               (!runtimeNotReady || health->hardInvalidSeen[s0][d0]) &&
                               (health->hardInvalidSeen[s0][d0] ||
                                invalid ||
                                health->i2cErrorSeen[s0][d0] ||
                                health->watchdogSeen[s0][d0] ||
                                health->saturatedSeen[s0][d0] ||
                                health->zeroAfterDrdySeen[s0][d0]);

            if (valid) {
                bool wasCacheValid = cache->valid;
                if (!wasCacheValid) {
                    uint32_t generation = cache->generation + 1u;
                    cache->source = SENSORARRAY_FDC_CACHE_SOURCE_LAST_GOOD;
                    cache->rCount = health->clockDividers[s0][d0] ? SENSORARRAY_FDC_RCOUNT : cache->rCount;
                    cache->settleCount = SENSORARRAY_FDC_SETTLECOUNT;
                    cache->clockDiv = health->clockDividers[s0][d0];
                    cache->driveCurrent = health->driveCurrent[s0][d0];
                    cache->deglitchCode = health->deglitchCode[s0][d0];
                    cache->effectiveFclkHz = health->effectiveFclkHz[s0][d0];
                    cache->generation = (generation == 0u) ? 1u : generation;
                    cache->storedTimestampUs = nowUs;
                    sensorarrayMeasureMarkFdcAppliedCellDirty(state, &target);
                }
                cache->valid = true;
                cache->lastRaw28 = health->lastRaw28[s0][d0];
                cache->lastFreqHz = health->lastFreqHz[s0][d0];
                cache->lastGoodTimestampUs = nowUs;
                cache->consecutiveErrors = 0u;
                cache->consecutiveNoUnread = 0u;
                cache->consecutiveZeroRaw = 0u;
                cache->consecutiveWatchdogFaults = 0u;
                cache->consecutiveI2cErrors = 0u;

                profile->valid = true;
                profile->lastRaw28 = health->lastRaw28[s0][d0];
                profile->lastFrequencyHz = health->lastFreqHz[s0][d0];
                profile->lastValidTimestampUs = (uint64_t)nowUs;
                profile->consecutiveInvalid = 0u;
                profile->consecutiveWatchdogFault = 0u;
                profile->consecutiveSaturated = 0u;
                profile->consecutiveZeroRaw = 0u;

                sensorarrayFdcCellCalibration_t *cal = sensorarrayFdcSweepGetCellCalibration(s, d);
                if (cal) {
                    cal->hasLastGood = true;
                    cal->lockValid = true;
                    cal->lastGoodDriveCurrent = cache->driveCurrent ? cache->driveCurrent : SENSORARRAY_FDC_DRIVE_CURRENT;
                    cal->lastGoodDeglitch = cache->deglitchCode ?
                        (Fdc2214CapDeglitch_t)cache->deglitchCode :
                        FDC2214_DEGLITCH_10MHZ;
                    cal->lastGoodHighCurrent = false;
                    cal->lastGoodRaw28 = cache->lastRaw28;
                    cal->lastGoodFreqHz = cache->lastFreqHz;
                    cal->lastGoodTimestampUs = (uint64_t)nowUs;
                    cal->consecutiveFailCount = 0u;
                    cal->consecutiveNoUnreadCount = 0u;
                    cal->consecutiveStatusFaultCount = 0u;
                    cal->consecutiveZeroRawCount = 0u;
                    cal->directFailCount = 0u;
                    cal->lastFailReason = "valid";
                }
            } else if (severeError) {
                if (cache->consecutiveErrors < UINT16_MAX) {
                    cache->consecutiveErrors++;
                }
                if (invalid &&
                    !health->i2cErrorSeen[s0][d0] &&
                    !health->watchdogSeen[s0][d0] &&
                    !health->zeroRawSeen[s0][d0] &&
                    !health->saturatedSeen[s0][d0] &&
                    cache->consecutiveNoUnread < UINT16_MAX) {
                    cache->consecutiveNoUnread++;
                }
                if (health->zeroRawSeen[s0][d0] && cache->consecutiveZeroRaw < UINT16_MAX) {
                    cache->consecutiveZeroRaw++;
                }
                if (health->watchdogSeen[s0][d0] && cache->consecutiveWatchdogFaults < UINT16_MAX) {
                    cache->consecutiveWatchdogFaults++;
                }
                if (health->i2cErrorSeen[s0][d0] && cache->consecutiveI2cErrors < UINT16_MAX) {
                    cache->consecutiveI2cErrors++;
                }
                if (profile->consecutiveInvalid < UINT32_MAX) {
                    profile->consecutiveInvalid++;
                }
                if (health->watchdogSeen[s0][d0] && profile->consecutiveWatchdogFault < UINT32_MAX) {
                    profile->consecutiveWatchdogFault++;
                }
                if (health->saturatedSeen[s0][d0] && profile->consecutiveSaturated < UINT32_MAX) {
                    profile->consecutiveSaturated++;
                }
                if (health->zeroRawSeen[s0][d0] && profile->consecutiveZeroRaw < UINT32_MAX) {
                    profile->consecutiveZeroRaw++;
                }
            }

            if (freshAmplitude) {
                if (cache->consecutiveAmplitudeWarnings < UINT16_MAX) {
                    cache->consecutiveAmplitudeWarnings++;
                }
                cache->lastWarningTimestampUs = nowUs;
                snprintf(cache->lastWarningReason, sizeof(cache->lastWarningReason), "%s", "fresh_amplitude_warning");
                if (profile->consecutiveAmplitudeFault < UINT32_MAX) {
                    profile->consecutiveAmplitudeFault++;
                }
                printf("FDC_WARN,scope=cell,s=%u,d=%u,index=%u,device=%s,ch=%u,class=fresh,reason=amplitude_warning,consecutive=%u,fingerprint=%lu\n",
                       (unsigned)s,
                       (unsigned)d,
                       (unsigned)matrixIndex,
                       sensorarrayMeasureFdcDeviceName(target.devId),
                       (unsigned)target.fdcChannel,
                       (unsigned)cache->consecutiveAmplitudeWarnings,
                       (unsigned long)cache->lastAppliedFingerprint);
            } else {
                cache->consecutiveAmplitudeWarnings = 0u;
                profile->consecutiveAmplitudeFault = 0u;
            }

            if (staleAmplitude) {
                if (cache->staleAmplitudeWarnings < UINT16_MAX) {
                    cache->staleAmplitudeWarnings++;
                }
                printf("FDC_WARN,scope=cell,s=%u,d=%u,index=%u,device=%s,ch=%u,class=stale,reason=amplitude_warning,action=suppress_rescue\n",
                       (unsigned)s,
                       (unsigned)d,
                       (unsigned)matrixIndex,
                       sensorarrayMeasureFdcDeviceName(target.devId),
                       (unsigned)target.fdcChannel);
            }
            if (transientAmplitude) {
                if (cache->transientAmplitudeWarnings < UINT16_MAX) {
                    cache->transientAmplitudeWarnings++;
                }
                printf("FDC_WARN,scope=cell,s=%u,d=%u,index=%u,device=%s,ch=%u,class=transient,reason=amplitude_warning,action=suppress_rescue\n",
                       (unsigned)s,
                       (unsigned)d,
                       (unsigned)matrixIndex,
                       sensorarrayMeasureFdcDeviceName(target.devId),
                       (unsigned)target.fdcChannel);
            }

            if (freshAmplitude &&
                cache->valid &&
                CONFIG_SENSORARRAY_FDC_REAPPLY_CACHE_ON_WARNING &&
                cache->consecutiveAmplitudeWarnings >= amplitudeThreshold) {
                uint32_t cooldownFrames = (uint32_t)CONFIG_SENSORARRAY_FDC_WARNING_REAPPLY_COOLDOWN_FRAMES;
                bool cooldownElapsed =
                    cooldownFrames == 0u ||
                    s_fdcMatrixSequence >= cache->lastReapplyFrame + cooldownFrames;
                bool fingerprintAlreadyReapplied =
                    CONFIG_SENSORARRAY_FDC_WARNING_REAPPLY_ONCE_PER_FINGERPRINT &&
                    cache->lastReapplyFingerprint == cache->lastAppliedFingerprint;
                if (!fingerprintAlreadyReapplied && cooldownElapsed && !cache->reapplyPending) {
                    cache->reapplyPending = true;
                    cache->lastReapplyFingerprint = cache->lastAppliedFingerprint;
                    cache->lastReapplyFrame = s_fdcMatrixSequence;
                    printf("FDC_RESCUE_DECISION,scope=cell,s=%u,d=%u,index=%u,device=%s,ch=%u,class=fresh,action=sanity_reapply_once,reason=amplitude_warning,consecutive=%u,fingerprint=%lu\n",
                           (unsigned)s,
                           (unsigned)d,
                           (unsigned)matrixIndex,
                           sensorarrayMeasureFdcDeviceName(target.devId),
                           (unsigned)target.fdcChannel,
                           (unsigned)cache->consecutiveAmplitudeWarnings,
                           (unsigned long)cache->lastAppliedFingerprint);
                } else {
                    printf("FDC_RESCUE_DECISION,scope=cell,s=%u,d=%u,index=%u,device=%s,ch=%u,class=fresh,action=suppress_reapply,reason=amplitude_warning,consecutive=%u,fingerprint=%lu,policy=%s\n",
                           (unsigned)s,
                           (unsigned)d,
                           (unsigned)matrixIndex,
                           sensorarrayMeasureFdcDeviceName(target.devId),
                           (unsigned)target.fdcChannel,
                           (unsigned)cache->consecutiveAmplitudeWarnings,
                           (unsigned long)cache->lastAppliedFingerprint,
                           fingerprintAlreadyReapplied ? "fingerprint_already_reapplied" : "cooldown");
                }
            } else if (amplitude && !freshAmplitude) {
                printf("FDC_RESCUE_DECISION,scope=cell,s=%u,d=%u,index=%u,device=%s,ch=%u,class=%s,action=suppress_rescue,reason=amplitude_warning,policy=non_fresh_epoch\n",
                       (unsigned)s,
                       (unsigned)d,
                       (unsigned)matrixIndex,
                       sensorarrayMeasureFdcDeviceName(target.devId),
                       (unsigned)target.fdcChannel,
                       staleAmplitude ? "stale" : "transient");
            }

            if (freshAmplitude &&
                cache->valid &&
                cache->consecutiveAmplitudeWarnings >=
                    (uint16_t)CONFIG_SENSORARRAY_FDC_AMPLITUDE_FAST_SWEEP_THRESHOLD) {
                int64_t fastCooldownUs =
                    (int64_t)CONFIG_SENSORARRAY_FDC_WARNING_FAST_SWEEP_COOLDOWN_MS * 1000LL;
                bool fastCooldownElapsed =
                    cache->lastFastSweepRequestUs == 0u ||
                    fastCooldownUs == 0 ||
                    (nowUs - (int64_t)cache->lastFastSweepRequestUs) >= fastCooldownUs;
                if (fastCooldownElapsed) {
                    cache->lastFastSweepRequestUs = (uint64_t)nowUs;
                    printf("FDC_RESCUE_DECISION,scope=cell,s=%u,d=%u,index=%u,device=%s,ch=%u,class=fresh,action=fast_sweep,reason=persistent_fresh_amplitude_warning_after_cache_apply,consecutive=%u,fingerprint=%lu\n",
                           (unsigned)s,
                           (unsigned)d,
                           (unsigned)matrixIndex,
                           sensorarrayMeasureFdcDeviceName(target.devId),
                           (unsigned)target.fdcChannel,
                           (unsigned)cache->consecutiveAmplitudeWarnings,
                           (unsigned long)cache->lastAppliedFingerprint);
                    (void)sensorarrayMeasureRequestFdcCellRescue(state,
                                                                 matrixIndex,
                                                                 "persistent_fresh_amplitude_warning_after_cache_apply");
                } else {
                    printf("FDC_RESCUE_DECISION,scope=cell,s=%u,d=%u,index=%u,device=%s,ch=%u,class=fresh,action=suppress_fast_sweep,reason=amplitude_warning,policy=cooldown,consecutive=%u\n",
                           (unsigned)s,
                           (unsigned)d,
                           (unsigned)matrixIndex,
                           sensorarrayMeasureFdcDeviceName(target.devId),
                           (unsigned)target.fdcChannel,
                           (unsigned)cache->consecutiveAmplitudeWarnings);
                }
            }

            const char *rescueReason =
                (severeError && cache->consecutiveI2cErrors >= threshold) ? "i2c_error_consecutive" :
                (severeError && cache->consecutiveWatchdogFaults >= threshold) ? "watchdog_fault_consecutive" :
                (severeError && cache->consecutiveZeroRaw >= threshold) ? "zero_raw_consecutive" :
                (severeError && cache->consecutiveNoUnread >= threshold) ? "no_unread_consecutive" :
                (severeError && cache->consecutiveErrors >= threshold) ? "invalid_streak" :
                NULL;
            if (rescueReason) {
                (void)sensorarrayMeasureRequestFdcCellRescue(state, matrixIndex, rescueReason);
            }
        }
    }
    return touched;
}

uint32_t sensorarrayMeasureCountFdcFrameWarnings(const sensorarrayFdcFrameHealth_t *health,
                                                sensorarrayFdcTimingSummary_t *timing,
                                                uint8_t activeRows)
{
    if (!health || !timing) {
        return 0u;
    }
    if (activeRows < 1u || activeRows > SENSORARRAY_MATRIX_ROWS) {
        activeRows = SENSORARRAY_MATRIX_ROWS;
    }
    uint32_t touched = 0u;
    for (uint8_t s = 0u; s < activeRows; ++s) {
        for (uint8_t d = 0u; d < SENSORARRAY_MATRIX_COLS; ++d) {
            touched++;
            timing->freshAmplitudeWarningCount += health->freshAmplitudeWarningSeen[s][d] ? 1u : 0u;
            timing->staleAmplitudeWarningCount += health->staleAmplitudeWarningSeen[s][d] ? 1u : 0u;
            timing->transientAmplitudeWarningCount += health->transientAmplitudeWarningSeen[s][d] ? 1u : 0u;
            timing->dataNotReadyCount += health->notReadySeen[s][d] ? 1u : 0u;
            timing->softInvalidCount += health->softInvalidSeen[s][d] ? 1u : 0u;
            timing->hardInvalidCount += health->hardInvalidSeen[s][d] ? 1u : 0u;
            timing->zeroBeforeReadyCount += health->zeroBeforeReadySeen[s][d] ? 1u : 0u;
            timing->zeroAfterDrdyCount += health->zeroAfterDrdySeen[s][d] ? 1u : 0u;
        }
    }
    return touched;
}

void sensorarrayMeasureInitFdcMatrixFrame(sensorarrayFdcMatrixFrame_t *frame)
{
    memset(frame, 0, sizeof(*frame));
    frame->timestampUs = (uint64_t)esp_timer_get_time();
    /*
     * Keep the physical sweep sequence independent from the host-visible fresh
     * frame sequence.  Internal retry/cooldown state uses s_fdcMatrixSequence
     * and therefore must advance even when a sweep is rejected as stale.
     */
    frame->physicalSweepId = ++s_fdcMatrixSequence;
    frame->sequence = s_fdcFreshFrameSequence;
    frame->frameStartUs = frame->timestampUs;
    frame->activeRows = SENSORARRAY_MATRIX_ROWS;
    frame->stale = true;
    for (size_t i = 0u; i < SENSORARRAY_MATRIX_CELL_COUNT; ++i) {
        frame->freqHz[i] = SENSORARRAY_FDC_INVALID_FREQ_SENTINEL_HZ;
        frame->capTotalPf[i] = SENSORARRAY_FDC_INVALID_CAP_SENTINEL_PF;
    }
}

bool sensorarrayFdcMatrixFrameRawAllZero(const sensorarrayFdcMatrixFrame_t *frame)
{
    if (!frame) {
        return true;
    }
    uint8_t cells = (uint8_t)(frame->activeRows * SENSORARRAY_MATRIX_COLS);
    return frame->freshCount == cells &&
           frame->hardwareZeroRawCount == cells &&
           frame->notReadyCount == 0u &&
           frame->zeroBeforeReadyCount == 0u;
}

uint64_t sensorarrayMeasureComputeFdcFrameCapTotalPf(sensorarrayFdcMatrixFrame_t *frame)
{
    if (!frame) {
        return 0u;
    }

    int64_t startUs = esp_timer_get_time();
    frame->capValidMask = 0u;
    const double inductorUh = (double)CONFIG_SENSORARRAY_FDC_TANK_INDUCTOR_NH / 1000.0;
    size_t activeCells = (size_t)frame->activeRows * SENSORARRAY_MATRIX_COLS;
    for (size_t i = 0u; i < activeCells; ++i) {
        uint64_t bit = 1ULL << i;
        frame->capTotalPf[i] = SENSORARRAY_FDC_INVALID_CAP_SENTINEL_PF;
        if ((frame->validMask & (1ULL << i)) == 0u) {
            frame->errorMask |= bit;
            continue;
        }
        double capPf = 0.0;
        if (sensorarrayMeasureFdcComputeCapacitancePf(frame->freqHz[i], inductorUh, &capPf)) {
            frame->capTotalPf[i] = capPf;
            frame->capValidMask |= bit;
        } else if (frame->freqHz[i] > 0.0 && frame->raw28[i] != 0u) {
            frame->errorMask |= bit;
            uint8_t sColumn = 0u;
            uint8_t dLine = 0u;
            if (sensorarrayMeasureDecodeMatrixIndex((uint8_t)i, &sColumn, &dLine)) {
                printf("MATRIXFDC_DIAG,reason=cap_calc_zero_with_nonzero_freq,row=%u,d=%u,freqHz=%.3f,raw28=%lu,inductorNh=%lu\n",
                       (unsigned)sColumn,
                       (unsigned)dLine,
                       frame->freqHz[i],
                       (unsigned long)frame->raw28[i],
                       (unsigned long)CONFIG_SENSORARRAY_FDC_TANK_INDUCTOR_NH);
            }
        }
    }
    return sensorarrayMeasureElapsedUs(startUs);
}

void sensorarrayMeasureMarkFdcMatrixCellEx(sensorarrayFdcMatrixFrame_t *frame,
                                                  uint8_t sIndex,
                                                  uint8_t dIndex,
                                                  uint32_t raw28,
                                                  double freqHz,
                                                  const sensorarrayFdcRuntimeChannelConfig_t *config,
                                                  bool fresh,
                                                  bool valid,
                                                  bool warning,
                                                  bool error,
                                                  bool notReady,
                                                  bool zeroBeforeReady,
                                                  bool zeroAfterDrdy,
                                                  bool i2cError)
{
    if (!frame || !sensorarrayMatrixIndexIsValid(sIndex, dIndex)) {
        return;
    }

    size_t index = sensorarrayMatrixIndex(sIndex, dIndex);
    uint64_t bit = 1ULL << index;
    frame->raw28[index] = raw28;
    frame->freqHz[index] = valid ? freqHz : SENSORARRAY_FDC_INVALID_FREQ_SENTINEL_HZ;
    if (config) {
        frame->clockDividers[index] = config->clockDividers;
        frame->driveCurrent[index] = config->driveCurrent;
        frame->deglitchCode[index] = config->deglitchCode;
        frame->effectiveFclkHz[index] = config->effectiveFclkHz;
    }
    if (valid) {
        frame->validMask |= bit;
        if (frame->validCount < UINT8_MAX) {
            frame->validCount++;
        }
    } else {
        frame->validMask &= ~bit;
    }
    if (fresh) {
        frame->freshMask |= bit;
        if (frame->freshCount < UINT8_MAX) {
            frame->freshCount++;
        }
        if (raw28 == 0u && frame->hardwareZeroRawCount < UINT8_MAX) {
            frame->hardwareZeroRawCount++;
        }
    } else {
        frame->freshMask &= ~bit;
        if (raw28 == 0u && frame->placeholderZeroCount < UINT8_MAX) {
            frame->placeholderZeroCount++;
        }
    }
    if (notReady && frame->notReadyCount < UINT8_MAX) {
        frame->notReadyCount++;
    }
    if (zeroBeforeReady && frame->zeroBeforeReadyCount < UINT8_MAX) {
        frame->zeroBeforeReadyCount++;
    }
    if (zeroAfterDrdy && frame->zeroAfterDrdyCount < UINT8_MAX) {
        frame->zeroAfterDrdyCount++;
    }
    if (i2cError && frame->i2cErrorCount < UINT8_MAX) {
        frame->i2cErrorCount++;
    }
    if (zeroBeforeReady && frame->unreadWithoutDrdyCount < UINT8_MAX) {
        frame->unreadWithoutDrdyCount++;
    }
    if (warning) {
        frame->warnMask |= bit;
    } else {
        frame->warnMask &= ~bit;
    }
    if (error) {
        frame->errorMask |= bit;
        if (frame->firstBadRow == 0u) {
            frame->firstBadRow = sIndex;
            frame->firstBadDevice = (dIndex > 4u) ?
                (uint8_t)SENSORARRAY_FDC_DEV_SECONDARY :
                (uint8_t)SENSORARRAY_FDC_DEV_PRIMARY;
        }
    } else {
        frame->errorMask &= ~bit;
    }
}

esp_err_t sensorarrayMeasureCheckFdcMatrixReady(sensorarrayState_t *state)
{
    if (!state) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!state->boardReady || !state->tmuxReady || !state->adsReady) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!sensorarrayMeasureFdcDeviceReadyForIo(&state->fdcPrimary)) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!sensorarrayMeasureFdcDeviceReadyForIo(&state->fdcSecondary)) {
        sensorarrayMeasureLogFdcSecondaryUnavailableOnce();
    }
    return ESP_OK;
}

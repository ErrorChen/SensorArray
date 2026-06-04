// Split from core/measure/sensorarrayMeasure.c to keep measurement domains isolated.
static void sensorarrayMeasureMarkFdcMatrixCellEx(sensorarrayFdcMatrixFrame_t *frame,
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

static void sensorarrayMeasureFillFdcMatrixRow(sensorarrayFdcMatrixFrame_t *frame,
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
            const sensorarrayFdcRuntimeChannelConfig_t *config = configs ? &configs[half][ch] : NULL;
            bool configOk = config && config->valid;
            bool severeFault = !fresh ||
                               !valid ||
                               !configOk ||
                               notReady ||
                               zeroBeforeReady ||
                               zeroAfterDrdy ||
                               (samples && (samples->i2cError[ch] ||
                                            samples->watchdogFault[ch] ||
                                            samples->saturated[ch] ||
                                            (fresh && samples->raw28[ch] == 0u)));
            bool error = severeFault;
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

static void sensorarrayMeasureAccumulateFdcHealth(sensorarrayFdcFrameHealth_t *health,
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

static void sensorarrayMeasureUpdateFdcRuntimeProfiles(sensorarrayState_t *state,
                                                       const sensorarrayFdcFrameHealth_t *health)
{
    if (!state || !health) {
        return;
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
    for (uint8_t s = 1u; s <= SENSORARRAY_MATRIX_ROWS; ++s) {
        for (uint8_t d = 1u; d <= SENSORARRAY_MATRIX_COLS; ++d) {
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
            bool severeError = !runtimeNotReady &&
                               (invalid ||
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
}

static void sensorarrayMeasureCountFdcFrameWarnings(const sensorarrayFdcFrameHealth_t *health,
                                                    sensorarrayFdcTimingSummary_t *timing)
{
    if (!health || !timing) {
        return;
    }
    for (uint8_t s = 0u; s < SENSORARRAY_MATRIX_ROWS; ++s) {
        for (uint8_t d = 0u; d < SENSORARRAY_MATRIX_COLS; ++d) {
            timing->freshAmplitudeWarningCount += health->freshAmplitudeWarningSeen[s][d] ? 1u : 0u;
            timing->staleAmplitudeWarningCount += health->staleAmplitudeWarningSeen[s][d] ? 1u : 0u;
            timing->transientAmplitudeWarningCount += health->transientAmplitudeWarningSeen[s][d] ? 1u : 0u;
            timing->dataNotReadyCount += health->notReadySeen[s][d] ? 1u : 0u;
            timing->zeroBeforeReadyCount += health->zeroBeforeReadySeen[s][d] ? 1u : 0u;
            timing->zeroAfterDrdyCount += health->zeroAfterDrdySeen[s][d] ? 1u : 0u;
        }
    }
}

static void sensorarrayMeasureInitFdcMatrixFrame(sensorarrayFdcMatrixFrame_t *frame)
{
    memset(frame, 0, sizeof(*frame));
    frame->timestampUs = (uint64_t)esp_timer_get_time();
    frame->sequence = s_fdcMatrixSequence++;
    for (size_t i = 0u; i < SENSORARRAY_MATRIX_CELL_COUNT; ++i) {
        frame->freqHz[i] = SENSORARRAY_FDC_INVALID_FREQ_SENTINEL_HZ;
        frame->capTotalPf[i] = SENSORARRAY_FDC_INVALID_CAP_SENTINEL_PF;
    }
}

static bool sensorarrayFdcMatrixFrameRawAllZero(const sensorarrayFdcMatrixFrame_t *frame)
{
    if (!frame) {
        return true;
    }
    return frame->freshCount == SENSORARRAY_MATRIX_CELL_COUNT &&
           frame->hardwareZeroRawCount == SENSORARRAY_MATRIX_CELL_COUNT &&
           frame->notReadyCount == 0u &&
           frame->zeroBeforeReadyCount == 0u;
}

static uint64_t sensorarrayMeasureComputeFdcFrameCapTotalPf(sensorarrayFdcMatrixFrame_t *frame)
{
    if (!frame) {
        return 0u;
    }

    int64_t startUs = esp_timer_get_time();
    frame->capValidMask = 0u;
    const double inductorUh = (double)CONFIG_SENSORARRAY_FDC_TANK_INDUCTOR_NH / 1000.0;
    for (size_t i = 0u; i < SENSORARRAY_MATRIX_CELL_COUNT; ++i) {
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

static void sensorarrayMeasureMarkFdcMatrixCellEx(sensorarrayFdcMatrixFrame_t *frame,
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

static esp_err_t sensorarrayMeasureCheckFdcMatrixReady(sensorarrayState_t *state)
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

static uint64_t sensorarrayMeasureElapsedUs(int64_t startUs)
{
    int64_t elapsedUs = esp_timer_get_time() - startUs;
    return (elapsedUs > 0) ? (uint64_t)elapsedUs : 0u;
}

static uint64_t sensorarrayMeasureEstimateI2cBits(uint32_t writeCount,
                                                  uint32_t readCount,
                                                  uint32_t writeBytes,
                                                  uint32_t readBytes)
{
    /*
     * Approximate START/address/ACK/STOP cost. This is intentionally an estimate
     * for profiling deltas, not a replacement for logic-analyzer timing.
     */
    uint64_t dataBytes = (uint64_t)writeBytes + (uint64_t)readBytes;
    uint64_t dataBitsWithAck = dataBytes * 9ull;
    uint64_t addressBitsWithAck = ((uint64_t)writeCount + ((uint64_t)readCount * 2ull)) * 9ull;
    uint64_t framingBits = ((uint64_t)writeCount + (uint64_t)readCount) * 2ull;
    return dataBitsWithAck + addressBitsWithAck + framingBits;
}

static void sensorarrayMeasureMergeFdcI2cStats(const Fdc2214CapI2cStats_t *primaryStats,
                                               const Fdc2214CapI2cStats_t *secondaryStats,
                                               const BoardSupportI2cBusInfo_t *primaryBus,
                                               const BoardSupportI2cBusInfo_t *secondaryBus,
                                               sensorarrayFdcTimingSummary_t *summary)
{
    if (!summary) {
        return;
    }

    Fdc2214CapI2cStats_t primary = primaryStats ? *primaryStats : (Fdc2214CapI2cStats_t){0};
    Fdc2214CapI2cStats_t secondary = secondaryStats ? *secondaryStats : (Fdc2214CapI2cStats_t){0};

    summary->i2cWriteCount = primary.writeCount + secondary.writeCount;
    summary->i2cReadCount = primary.readCount + secondary.readCount;
    summary->i2cVerifyReadCount = primary.verifyReadCount + secondary.verifyReadCount;
    summary->i2cRetryCount = primary.retryCount + secondary.retryCount;
    summary->i2cNackCount = primary.nackCount + secondary.nackCount;
    summary->i2cTimeoutCount = primary.timeoutCount + secondary.timeoutCount;
    summary->i2cRecoveryCount = primary.recoveryCount + secondary.recoveryCount;

    summary->i2cBus0WriteCount = primary.writeCount;
    summary->i2cBus0ReadCount = primary.readCount;
    summary->i2cBus0WriteBytes = primary.writeBytes;
    summary->i2cBus0ReadBytes = primary.readBytes;
    summary->i2cBus0TotalUs = primary.totalUs;
    summary->i2cBus0RetryCount = primary.retryCount;
    summary->i2cBus0NackCount = primary.nackCount;
    summary->i2cBus0TimeoutCount = primary.timeoutCount;

    summary->i2cBus1WriteCount = secondary.writeCount;
    summary->i2cBus1ReadCount = secondary.readCount;
    summary->i2cBus1WriteBytes = secondary.writeBytes;
    summary->i2cBus1ReadBytes = secondary.readBytes;
    summary->i2cBus1TotalUs = secondary.totalUs;
    summary->i2cBus1RetryCount = secondary.retryCount;
    summary->i2cBus1NackCount = secondary.nackCount;
    summary->i2cBus1TimeoutCount = secondary.timeoutCount;

    uint32_t primaryFreq = (primaryBus && primaryBus->FrequencyHz != 0u) ?
        primaryBus->FrequencyHz :
        (uint32_t)CONFIG_BOARD_I2C_FREQ_HZ;
    uint32_t secondaryFreq = (secondaryBus && secondaryBus->FrequencyHz != 0u) ?
        secondaryBus->FrequencyHz :
        primaryFreq;
    summary->i2cFreqHz = (primaryFreq == secondaryFreq) ? primaryFreq : 0u;

    uint64_t primaryBits = sensorarrayMeasureEstimateI2cBits(primary.writeCount,
                                                             primary.readCount,
                                                             primary.writeBytes,
                                                             primary.readBytes);
    uint64_t secondaryBits = sensorarrayMeasureEstimateI2cBits(secondary.writeCount,
                                                               secondary.readCount,
                                                               secondary.writeBytes,
                                                               secondary.readBytes);
    summary->i2cEstimatedBits = primaryBits + secondaryBits;
    uint64_t primaryEstimatedUs = primaryFreq ? ((primaryBits * 1000000ull) / primaryFreq) : 0ull;
    uint64_t secondaryEstimatedUs = secondaryFreq ? ((secondaryBits * 1000000ull) / secondaryFreq) : 0ull;
    if (CONFIG_SENSORARRAY_FDC_PARALLEL_DUAL_BUS_READ && s_fdcWorkersAvailable) {
        summary->i2cEstimatedBusUs = (primaryEstimatedUs > secondaryEstimatedUs) ?
            primaryEstimatedUs :
            secondaryEstimatedUs;
        summary->i2cMeasuredUs = (primary.totalUs > secondary.totalUs) ?
            primary.totalUs :
            secondary.totalUs;
    } else {
        summary->i2cEstimatedBusUs = primaryEstimatedUs + secondaryEstimatedUs;
        summary->i2cMeasuredUs = primary.totalUs + secondary.totalUs;
    }
    summary->i2cOverheadUs = (int64_t)summary->i2cMeasuredUs - (int64_t)summary->i2cEstimatedBusUs;
}

static void sensorarrayMeasurePrintFdcTimingSummary(const sensorarrayFdcTimingSummary_t *summary,
                                                    uint32_t sequence)
{
    if (!summary) {
        return;
    }

    uint64_t targetFrameUs = SENSORARRAY_FDC_TARGET_FRAME_US;
    uint64_t overrunUs = (summary->frameUs > targetFrameUs) ? (summary->frameUs - targetFrameUs) : 0u;
    uint64_t fpsX100 = summary->frameUs ? (100000000ull / summary->frameUs) : 0ull;
    uint64_t budgetUsePct = targetFrameUs ? ((summary->frameUs * 100ull) / targetFrameUs) : 0ull;

    printf("SCAN_TIMING_FRAME,seq=%lu,targetFps=%lu,targetFrameUs=%llu,frameUs=%llu,fps=%llu.%02llu,budgetUsePct=%llu,overrun=%u,overrunUs=%llu,rowAvgUs=%llu,rowMaxUs=%llu,rowMinUs=%llu,slowRow=%u,pathEnsureUs=%llu,cacheApplyUs=%llu,applyBuildConfigUs=%llu,applyChannelConfigWriteUs=%llu,applyGlobalConfigWriteUs=%llu,applyVerifyUs=%llu,applyDelayUs=%llu,applyReadyWaitUs=%llu,applyMutexWaitUs=%llu,applyLogUs=%llu,discardUs=%llu,waitReadyUs=%llu,readUs=%llu,emitUs=%llu,capComputeUs=%llu,sweepUs=%llu,runtimeSweepCount=%lu,intbTimeoutCount=%lu,readyFullCount=%lu,readyPartialCount=%lu,readyNoneCount=%lu,fallbackAttemptCount=%lu,fallbackSuccessCount=%lu,fallbackPartialCount=%lu,fallbackFailCount=%lu,rowFullInvalidCount=%lu,deviceFullInvalidCount=%lu,avgWaitReadyUsPrimary=%llu,avgWaitReadyUsSecondary=%llu,maxWaitReadyUs=%llu,avgRead4UsPrimary=%llu,avgRead4UsSecondary=%llu,maxI2cReadUs=%llu,sweepRequestCount=%lu,sweepActuallyQueuedCount=%lu,i2cWriteCount=%lu,i2cReadCount=%lu,i2cVerifyReadCount=%lu,i2cRetryCount=%lu,i2cNackCount=%lu,i2cTimeoutCount=%lu,i2cRecoveryCount=%lu,i2cFreqHz=%lu,i2cEstimatedBits=%llu,i2cEstimatedBusUs=%llu,i2cMeasuredUs=%llu,i2cOverheadUs=%lld,i2cBus0WriteCount=%lu,i2cBus0ReadCount=%lu,i2cBus0WriteBytes=%lu,i2cBus0ReadBytes=%lu,i2cBus0TotalUs=%llu,i2cBus0RetryCount=%lu,i2cBus0NackCount=%lu,i2cBus0TimeoutCount=%lu,i2cBus1WriteCount=%lu,i2cBus1ReadCount=%lu,i2cBus1WriteBytes=%lu,i2cBus1ReadBytes=%lu,i2cBus1TotalUs=%llu,i2cBus1RetryCount=%lu,i2cBus1NackCount=%lu,i2cBus1TimeoutCount=%lu\n",
           (unsigned long)sequence,
           (unsigned long)CONFIG_SENSORARRAY_FDC_MATRIX_TARGET_FPS,
           (unsigned long long)targetFrameUs,
           (unsigned long long)summary->frameUs,
           (unsigned long long)(fpsX100 / 100ull),
           (unsigned long long)(fpsX100 % 100ull),
           (unsigned long long)budgetUsePct,
           overrunUs ? 1u : 0u,
           (unsigned long long)overrunUs,
           (unsigned long long)summary->rowAvgUs,
           (unsigned long long)summary->rowMaxUs,
           (unsigned long long)summary->rowMinUs,
           (unsigned)summary->slowRow,
           (unsigned long long)summary->pathEnsureUs,
           (unsigned long long)summary->cacheApplyUs,
           (unsigned long long)summary->applyBuildConfigUs,
           (unsigned long long)summary->applyChannelConfigWriteUs,
           (unsigned long long)summary->applyGlobalConfigWriteUs,
           (unsigned long long)summary->applyVerifyUs,
           (unsigned long long)summary->applyDelayUs,
           (unsigned long long)summary->applyReadyWaitUs,
           (unsigned long long)summary->applyMutexWaitUs,
           (unsigned long long)summary->applyLogUs,
           (unsigned long long)summary->discardUs,
           (unsigned long long)summary->waitReadyUs,
           (unsigned long long)summary->readUs,
           (unsigned long long)summary->emitUs,
           (unsigned long long)summary->capComputeUs,
           (unsigned long long)summary->sweepUs,
           (unsigned long)summary->runtimeSweepCount,
           (unsigned long)summary->intbTimeoutCount,
           (unsigned long)summary->readyFullCount,
           (unsigned long)summary->readyPartialCount,
           (unsigned long)summary->readyNoneCount,
           (unsigned long)summary->fallbackAttemptCount,
           (unsigned long)summary->fallbackSuccessCount,
           (unsigned long)summary->fallbackPartialCount,
           (unsigned long)summary->fallbackFailCount,
           (unsigned long)summary->rowFullInvalidCount,
           (unsigned long)summary->deviceFullInvalidCount,
           (unsigned long long)sensorarrayMeasureAvgU64(summary->waitReadyUsPrimaryTotal, SENSORARRAY_MATRIX_ROWS),
           (unsigned long long)sensorarrayMeasureAvgU64(summary->waitReadyUsSecondaryTotal, SENSORARRAY_MATRIX_ROWS),
           (unsigned long long)summary->maxWaitReadyUs,
           (unsigned long long)sensorarrayMeasureAvgU64(summary->read4UsPrimaryTotal, SENSORARRAY_MATRIX_ROWS),
           (unsigned long long)sensorarrayMeasureAvgU64(summary->read4UsSecondaryTotal, SENSORARRAY_MATRIX_ROWS),
           (unsigned long long)summary->maxI2cReadUs,
           (unsigned long)summary->sweepRequestCount,
           (unsigned long)summary->sweepActuallyQueuedCount,
           (unsigned long)summary->i2cWriteCount,
           (unsigned long)summary->i2cReadCount,
           (unsigned long)summary->i2cVerifyReadCount,
           (unsigned long)summary->i2cRetryCount,
           (unsigned long)summary->i2cNackCount,
           (unsigned long)summary->i2cTimeoutCount,
           (unsigned long)summary->i2cRecoveryCount,
           (unsigned long)summary->i2cFreqHz,
           (unsigned long long)summary->i2cEstimatedBits,
           (unsigned long long)summary->i2cEstimatedBusUs,
           (unsigned long long)summary->i2cMeasuredUs,
           (long long)summary->i2cOverheadUs,
           (unsigned long)summary->i2cBus0WriteCount,
           (unsigned long)summary->i2cBus0ReadCount,
           (unsigned long)summary->i2cBus0WriteBytes,
           (unsigned long)summary->i2cBus0ReadBytes,
           (unsigned long long)summary->i2cBus0TotalUs,
           (unsigned long)summary->i2cBus0RetryCount,
           (unsigned long)summary->i2cBus0NackCount,
           (unsigned long)summary->i2cBus0TimeoutCount,
           (unsigned long)summary->i2cBus1WriteCount,
           (unsigned long)summary->i2cBus1ReadCount,
           (unsigned long)summary->i2cBus1WriteBytes,
           (unsigned long)summary->i2cBus1ReadBytes,
           (unsigned long long)summary->i2cBus1TotalUs,
           (unsigned long)summary->i2cBus1RetryCount,
           (unsigned long)summary->i2cBus1NackCount,
           (unsigned long)summary->i2cBus1TimeoutCount);
    printf("SCAN_TIMING_WORKER,seq=%lu,workerQueueSendUs=%llu,workerSleepAckWaitUs=%llu,workerStartGiveUs=%llu,workerDoneWaitUs=%llu,workerLateDoneUs=%llu,workerIdleAfterTimeoutUs=%llu,serialFallbackUs=%llu,repairPrimaryUs=%llu,repairSecondaryUs=%llu,workerTimeoutCount=%lu,workerLateDoneCount=%lu,staleResultDiscardedCount=%lu,duplicateReadCount=%lu,outputFormatUs=%llu,outputPrintfUs=%llu,measureLockHeldUs=%llu\n",
           (unsigned long)sequence,
           (unsigned long long)summary->workerQueueSendUs,
           (unsigned long long)summary->workerSleepAckWaitUs,
           (unsigned long long)summary->workerStartGiveUs,
           (unsigned long long)summary->workerDoneWaitUs,
           (unsigned long long)summary->workerLateDoneUs,
           (unsigned long long)summary->workerIdleAfterTimeoutUs,
           (unsigned long long)summary->serialFallbackUs,
           (unsigned long long)summary->repairPrimaryUs,
           (unsigned long long)summary->repairSecondaryUs,
           (unsigned long)summary->workerTimeoutCount,
           (unsigned long)summary->workerLateDoneCount,
           (unsigned long)summary->staleResultDiscardedCount,
           (unsigned long)summary->duplicateReadCount,
           (unsigned long long)summary->outputFormatUs,
           (unsigned long long)summary->outputPrintfUs,
           (unsigned long long)summary->measureLockHeldUs);
}

static uint64_t sensorarrayMeasureAvgU64(uint64_t total, uint32_t count)
{
    return count ? (total / count) : 0u;
}

static void sensorarrayMeasurePrintFdcTimingAggregate(const sensorarrayFdcTimingAggregate_t *agg)
{
    if (!agg || agg->frames == 0u) {
        return;
    }

    const sensorarrayFdcTimingSummary_t *t = &agg->totals;
    uint64_t avgFrameUs = sensorarrayMeasureAvgU64(agg->frameUsTotal, agg->frames);
    uint64_t avgFpsX100 = avgFrameUs ? (100000000ull / avgFrameUs) : 0ull;
    uint64_t budgetUsePctAvg = SENSORARRAY_FDC_TARGET_FRAME_US ?
        ((avgFrameUs * 100ull) / SENSORARRAY_FDC_TARGET_FRAME_US) :
        0ull;
    uint32_t rowCount = agg->frames * SENSORARRAY_MATRIX_ROWS;

#if CONFIG_SENSORARRAY_LOG_I2C_STATS_EVERY_N_FRAMES > 0
    if (agg->frames >= CONFIG_SENSORARRAY_LOG_I2C_STATS_EVERY_N_FRAMES) {
        uint32_t bus0Ops = t->i2cBus0WriteCount + t->i2cBus0ReadCount;
        uint32_t bus1Ops = t->i2cBus1WriteCount + t->i2cBus1ReadCount;
        printf("BOARD_I2C_STATS,seqStart=%lu,seqEnd=%lu,periodFrames=%lu,port=0,writeCount=%lu,readCount=%lu,writeBytes=%lu,readBytes=%lu,avgUs=%llu,totalUs=%llu,retryCount=%lu,nackCount=%lu,timeoutCount=%lu,recoveryCountGlobal=%lu,maxReadUsGlobal=%llu\n",
               (unsigned long)agg->seqStart,
               (unsigned long)agg->seqEnd,
               (unsigned long)agg->frames,
               (unsigned long)t->i2cBus0WriteCount,
               (unsigned long)t->i2cBus0ReadCount,
               (unsigned long)t->i2cBus0WriteBytes,
               (unsigned long)t->i2cBus0ReadBytes,
               (unsigned long long)sensorarrayMeasureAvgU64(t->i2cBus0TotalUs, bus0Ops),
               (unsigned long long)t->i2cBus0TotalUs,
               (unsigned long)t->i2cBus0RetryCount,
               (unsigned long)t->i2cBus0NackCount,
               (unsigned long)t->i2cBus0TimeoutCount,
               (unsigned long)t->i2cRecoveryCount,
               (unsigned long long)t->maxI2cReadUs);
        printf("BOARD_I2C_STATS,seqStart=%lu,seqEnd=%lu,periodFrames=%lu,port=1,writeCount=%lu,readCount=%lu,writeBytes=%lu,readBytes=%lu,avgUs=%llu,totalUs=%llu,retryCount=%lu,nackCount=%lu,timeoutCount=%lu,recoveryCountGlobal=%lu,maxReadUsGlobal=%llu\n",
               (unsigned long)agg->seqStart,
               (unsigned long)agg->seqEnd,
               (unsigned long)agg->frames,
               (unsigned long)t->i2cBus1WriteCount,
               (unsigned long)t->i2cBus1ReadCount,
               (unsigned long)t->i2cBus1WriteBytes,
               (unsigned long)t->i2cBus1ReadBytes,
               (unsigned long long)sensorarrayMeasureAvgU64(t->i2cBus1TotalUs, bus1Ops),
               (unsigned long long)t->i2cBus1TotalUs,
               (unsigned long)t->i2cBus1RetryCount,
               (unsigned long)t->i2cBus1NackCount,
               (unsigned long)t->i2cBus1TimeoutCount,
               (unsigned long)t->i2cRecoveryCount,
               (unsigned long long)t->maxI2cReadUs);
    }
#endif

#if CONFIG_SENSORARRAY_LOG_TIMING_STATS_EVERY_N_FRAMES > 0
    if (agg->frames >= CONFIG_SENSORARRAY_LOG_TIMING_STATS_EVERY_N_FRAMES) {
        printf("FDC_TIMING_STATS,seqStart=%lu,seqEnd=%lu,periodFrames=%lu,targetFps=%lu,avgFrameUs=%llu,minFrameUs=%llu,maxFrameUs=%llu,avgFps=%llu.%02llu,budgetUsePctAvg=%llu,overrunCount=%lu,rowAvgUs=%llu,rowMaxUs=%llu,slowRow=%u,cacheApplyAvgUs=%llu,waitReadyPrimaryAvgUs=%llu,waitReadySecondaryAvgUs=%llu,read4PrimaryAvgUs=%llu,read4SecondaryAvgUs=%llu,sweepAvgUs=%llu,runtimeSweepCount=%lu,intbTimeoutCount=%lu\n",
               (unsigned long)agg->seqStart,
               (unsigned long)agg->seqEnd,
               (unsigned long)agg->frames,
               (unsigned long)CONFIG_SENSORARRAY_FDC_MATRIX_TARGET_FPS,
               (unsigned long long)avgFrameUs,
               (unsigned long long)agg->frameUsMin,
               (unsigned long long)agg->frameUsMax,
               (unsigned long long)(avgFpsX100 / 100ull),
               (unsigned long long)(avgFpsX100 % 100ull),
               (unsigned long long)budgetUsePctAvg,
               (unsigned long)agg->overrunCount,
               (unsigned long long)sensorarrayMeasureAvgU64(agg->rowUsTotal, rowCount),
               (unsigned long long)agg->rowUsMax,
               (unsigned)agg->slowRow,
               (unsigned long long)sensorarrayMeasureAvgU64(t->cacheApplyUs, agg->frames),
               (unsigned long long)sensorarrayMeasureAvgU64(t->waitReadyUsPrimaryTotal, rowCount),
               (unsigned long long)sensorarrayMeasureAvgU64(t->waitReadyUsSecondaryTotal, rowCount),
               (unsigned long long)sensorarrayMeasureAvgU64(t->read4UsPrimaryTotal, rowCount),
               (unsigned long long)sensorarrayMeasureAvgU64(t->read4UsSecondaryTotal, rowCount),
               (unsigned long long)sensorarrayMeasureAvgU64(t->sweepUs, t->runtimeSweepCount),
               (unsigned long)t->runtimeSweepCount,
               (unsigned long)t->intbTimeoutCount);
        printf("FDC_CACHE_STATS,seqStart=%lu,seqEnd=%lu,periodFrames=%lu,applyAvgUs=%llu,skippedCount=%lu,dirtyDeviceCount=%lu,diffWriteCount=%lu,fullWriteCount=%lu,noDiffCount=%lu,diffRcountWrites=%lu,diffSettleWrites=%lu,diffClockDivWrites=%lu,diffDriveWrites=%lu,diffMuxWrites=%lu,diffStatusConfigWrites=%lu,diffConfigWrites=%lu,appliedFingerprintChanges=%lu\n",
               (unsigned long)agg->seqStart,
               (unsigned long)agg->seqEnd,
               (unsigned long)agg->frames,
               (unsigned long long)sensorarrayMeasureAvgU64(t->cacheApplyUs, agg->frames),
               (unsigned long)t->cacheApplySkippedCount,
               (unsigned long)t->cacheApplyDirtyDeviceCount,
               (unsigned long)t->cacheApplyDiffWriteCount,
               (unsigned long)t->cacheApplyFullWriteCount,
               (unsigned long)t->cacheApplyNoDiffCount,
               (unsigned long)t->diffRcountWrites,
               (unsigned long)t->diffSettleWrites,
               (unsigned long)t->diffClockDivWrites,
               (unsigned long)t->diffDriveWrites,
               (unsigned long)t->diffMuxWrites,
               (unsigned long)t->diffStatusConfigWrites,
               (unsigned long)t->diffConfigWrites,
               (unsigned long)t->appliedFingerprintChanges);
        printf("FDC_WORKER_STATS,seqStart=%lu,seqEnd=%lu,periodFrames=%lu,parallel=%u,primaryJobAvgUs=%llu,secondaryJobAvgUs=%llu,dualBusWaitAvgUs=%llu,dualBusSkewAvgUs=%llu,intbFreshDrdyCount=%lu,intbFalseEdgeCount=%lu,intbTimeoutCount=%lu,readyFullCount=%lu,readyPartialCount=%lu,readyNoneCount=%lu,fallbackAttemptCount=%lu,fallbackSuccessCount=%lu,fallbackPartialCount=%lu,fallbackFailCount=%lu\n",
               (unsigned long)agg->seqStart,
               (unsigned long)agg->seqEnd,
               (unsigned long)agg->frames,
               (CONFIG_SENSORARRAY_FDC_PARALLEL_DUAL_BUS_READ && s_fdcWorkersAvailable) ? 1u : 0u,
               (unsigned long long)sensorarrayMeasureAvgU64(t->primaryJobUs, rowCount),
               (unsigned long long)sensorarrayMeasureAvgU64(t->secondaryJobUs, rowCount),
               (unsigned long long)sensorarrayMeasureAvgU64(t->dualBusWaitUs, rowCount),
               (unsigned long long)sensorarrayMeasureAvgU64(t->dualBusSkewUs, rowCount),
               (unsigned long)t->intbFreshDrdyCount,
               (unsigned long)t->intbFalseEdgeCount,
               (unsigned long)t->intbTimeoutCount,
               (unsigned long)t->readyFullCount,
               (unsigned long)t->readyPartialCount,
               (unsigned long)t->readyNoneCount,
               (unsigned long)t->fallbackAttemptCount,
               (unsigned long)t->fallbackSuccessCount,
               (unsigned long)t->fallbackPartialCount,
               (unsigned long)t->fallbackFailCount);
        printf("FDC_WORKER_SYNC_STATS,seqStart=%lu,seqEnd=%lu,periodFrames=%lu,queueSendAvgUs=%llu,sleepAckWaitAvgUs=%llu,startGiveAvgUs=%llu,doneWaitAvgUs=%llu,lateDoneUs=%llu,idleAfterTimeoutUs=%llu,serialFallbackAvgUs=%llu,repairPrimaryAvgUs=%llu,repairSecondaryAvgUs=%llu,workerTimeoutCount=%lu,workerLateDoneCount=%lu,staleResultDiscardedCount=%lu,duplicateReadCount=%lu,outputFormatAvgUs=%llu,outputPrintfAvgUs=%llu,measureLockHeldAvgUs=%llu\n",
               (unsigned long)agg->seqStart,
               (unsigned long)agg->seqEnd,
               (unsigned long)agg->frames,
               (unsigned long long)sensorarrayMeasureAvgU64(t->workerQueueSendUs, rowCount),
               (unsigned long long)sensorarrayMeasureAvgU64(t->workerSleepAckWaitUs, rowCount),
               (unsigned long long)sensorarrayMeasureAvgU64(t->workerStartGiveUs, rowCount),
               (unsigned long long)sensorarrayMeasureAvgU64(t->workerDoneWaitUs, rowCount),
               (unsigned long long)t->workerLateDoneUs,
               (unsigned long long)t->workerIdleAfterTimeoutUs,
               (unsigned long long)sensorarrayMeasureAvgU64(t->serialFallbackUs, rowCount),
               (unsigned long long)sensorarrayMeasureAvgU64(t->repairPrimaryUs, rowCount),
               (unsigned long long)sensorarrayMeasureAvgU64(t->repairSecondaryUs, rowCount),
               (unsigned long)t->workerTimeoutCount,
               (unsigned long)t->workerLateDoneCount,
               (unsigned long)t->staleResultDiscardedCount,
               (unsigned long)t->duplicateReadCount,
               (unsigned long long)sensorarrayMeasureAvgU64(t->outputFormatUs, agg->frames),
               (unsigned long long)sensorarrayMeasureAvgU64(t->outputPrintfUs, agg->frames),
               (unsigned long long)sensorarrayMeasureAvgU64(t->measureLockHeldUs, agg->frames));
        printf("FDC_VALIDITY_STATS,seqStart=%lu,seqEnd=%lu,periodFrames=%lu,rowFullInvalidCount=%lu,deviceFullInvalidCount=%lu,degradedCellCount=%lu,unreadWithoutDrdyCount=%lu,dataNotReadyCount=%lu,zeroBeforeReadyCount=%lu,zeroAfterDrdyCount=%lu,freshWarnCount=%lu,staleWarnCount=%lu,transientWarnCount=%lu,fastSweepRequestedCount=%lu,sweepRequestCount=%lu,sweepActuallyQueuedCount=%lu\n",
               (unsigned long)agg->seqStart,
               (unsigned long)agg->seqEnd,
               (unsigned long)agg->frames,
               (unsigned long)t->rowFullInvalidCount,
               (unsigned long)t->deviceFullInvalidCount,
               (unsigned long)t->degradedCellCount,
               (unsigned long)t->unreadWithoutDrdyCount,
               (unsigned long)t->dataNotReadyCount,
               (unsigned long)t->zeroBeforeReadyCount,
               (unsigned long)t->zeroAfterDrdyCount,
               (unsigned long)t->freshAmplitudeWarningCount,
               (unsigned long)t->staleAmplitudeWarningCount,
               (unsigned long)t->transientAmplitudeWarningCount,
               (unsigned long)t->warningFastSweepRequestedCount,
               (unsigned long)t->sweepRequestCount,
               (unsigned long)t->sweepActuallyQueuedCount);
    }
#endif

    printf("SCAN_TIMING_10,seqStart=%lu,seqEnd=%lu,frames=%lu,targetFps=%lu,avgFrameUs=%llu,minFrameUs=%llu,maxFrameUs=%llu,avgFps=%llu.%02llu,budgetUsePctAvg=%llu,overrunCount=%lu,rowAvgUs=%llu,rowMaxUs=%llu,slowRow=%u,sleepBeforeRowSwitchAvgUs=%llu,rowSwitchWhileSleepingAvgUs=%llu,rowSettleAvgUs=%llu,diffApplyWhileSleepingAvgUs=%llu,sleepTotalAvgUs=%llu,sleepExitToIntbAvgUs=%llu,statusReadAvgUs=%llu,dataReadAvgUs=%llu,primaryJobAvgUs=%llu,secondaryJobAvgUs=%llu,dualBusWaitAvgUs=%llu,dualBusSkewAvgUs=%llu,cacheApplyAvgUs=%llu,cacheDiffWriteAvg=%llu,i2cWriteAvg=%llu,i2cReadAvg=%llu,i2cMeasuredAvgUs=%llu,bus0AvgUs=%llu,bus1AvgUs=%llu,freshWarnCount=%lu,staleWarnCount=%lu,transientWarnCount=%lu,fastSweepCount=%lu,intbFreshDrdyCount=%lu,intbTimeoutCount=%lu,intbFalseEdgeCount=%lu,readyFullCount=%lu,readyPartialCount=%lu,readyNoneCount=%lu,unreadWithoutDrdyCount=%lu,dataNotReadyCount=%lu,zeroBeforeReadyCount=%lu,zeroAfterDrdyCount=%lu,fallbackAttemptCount=%lu,fallbackSuccessCount=%lu,fallbackPartialCount=%lu,fallbackFailCount=%lu,rowFullInvalidCount=%lu,deviceFullInvalidCount=%lu,avgWaitReadyUsPrimary=%llu,avgWaitReadyUsSecondary=%llu,maxWaitReadyUs=%llu,avgRead4UsPrimary=%llu,avgRead4UsSecondary=%llu,maxI2cReadUs=%llu,sweepRequestCount=%lu,sweepActuallyQueuedCount=%lu,outputMode=%s,rowRestartMethod=sleep,parallel=%u,readyMode=%s,intbHint=%s,configIntbDisabled=1\n",
           (unsigned long)agg->seqStart,
           (unsigned long)agg->seqEnd,
           (unsigned long)agg->frames,
           (unsigned long)CONFIG_SENSORARRAY_FDC_MATRIX_TARGET_FPS,
           (unsigned long long)avgFrameUs,
           (unsigned long long)agg->frameUsMin,
           (unsigned long long)agg->frameUsMax,
           (unsigned long long)(avgFpsX100 / 100ull),
           (unsigned long long)(avgFpsX100 % 100ull),
           (unsigned long long)budgetUsePctAvg,
           (unsigned long)agg->overrunCount,
           (unsigned long long)sensorarrayMeasureAvgU64(agg->rowUsTotal, rowCount),
           (unsigned long long)agg->rowUsMax,
           (unsigned)agg->slowRow,
           (unsigned long long)sensorarrayMeasureAvgU64(t->sleepBeforeRowSwitchUs, rowCount),
           (unsigned long long)sensorarrayMeasureAvgU64(t->rowSwitchWhileSleepingUs, rowCount),
           (unsigned long long)sensorarrayMeasureAvgU64(t->rowSettleUs, rowCount),
           (unsigned long long)sensorarrayMeasureAvgU64(t->diffApplyWhileSleepingUs, rowCount),
           (unsigned long long)sensorarrayMeasureAvgU64(t->sleepTotalUs, rowCount),
           (unsigned long long)sensorarrayMeasureAvgU64(t->sleepExitToIntbUs, rowCount),
           (unsigned long long)sensorarrayMeasureAvgU64(t->statusReadUs, rowCount),
           (unsigned long long)sensorarrayMeasureAvgU64(t->dataReadUs, rowCount),
           (unsigned long long)sensorarrayMeasureAvgU64(t->primaryJobUs, rowCount),
           (unsigned long long)sensorarrayMeasureAvgU64(t->secondaryJobUs, rowCount),
           (unsigned long long)sensorarrayMeasureAvgU64(t->dualBusWaitUs, rowCount),
           (unsigned long long)sensorarrayMeasureAvgU64(t->dualBusSkewUs, rowCount),
           (unsigned long long)sensorarrayMeasureAvgU64(t->cacheApplyUs, agg->frames),
           (unsigned long long)sensorarrayMeasureAvgU64(t->cacheApplyDiffWriteCount, agg->frames),
           (unsigned long long)sensorarrayMeasureAvgU64(t->i2cWriteCount, agg->frames),
           (unsigned long long)sensorarrayMeasureAvgU64(t->i2cReadCount, agg->frames),
           (unsigned long long)sensorarrayMeasureAvgU64(t->i2cMeasuredUs, agg->frames),
           (unsigned long long)sensorarrayMeasureAvgU64(t->i2cBus0TotalUs, agg->frames),
           (unsigned long long)sensorarrayMeasureAvgU64(t->i2cBus1TotalUs, agg->frames),
           (unsigned long)t->freshAmplitudeWarningCount,
           (unsigned long)t->staleAmplitudeWarningCount,
           (unsigned long)t->transientAmplitudeWarningCount,
           (unsigned long)t->warningFastSweepRequestedCount,
           (unsigned long)t->intbFreshDrdyCount,
           (unsigned long)t->intbTimeoutCount,
           (unsigned long)t->intbFalseEdgeCount,
           (unsigned long)t->readyFullCount,
           (unsigned long)t->readyPartialCount,
           (unsigned long)t->readyNoneCount,
           (unsigned long)t->unreadWithoutDrdyCount,
           (unsigned long)t->dataNotReadyCount,
           (unsigned long)t->zeroBeforeReadyCount,
           (unsigned long)t->zeroAfterDrdyCount,
           (unsigned long)t->fallbackAttemptCount,
           (unsigned long)t->fallbackSuccessCount,
           (unsigned long)t->fallbackPartialCount,
           (unsigned long)t->fallbackFailCount,
           (unsigned long)t->rowFullInvalidCount,
           (unsigned long)t->deviceFullInvalidCount,
           (unsigned long long)sensorarrayMeasureAvgU64(t->waitReadyUsPrimaryTotal, rowCount),
           (unsigned long long)sensorarrayMeasureAvgU64(t->waitReadyUsSecondaryTotal, rowCount),
           (unsigned long long)t->maxWaitReadyUs,
           (unsigned long long)sensorarrayMeasureAvgU64(t->read4UsPrimaryTotal, rowCount),
           (unsigned long long)sensorarrayMeasureAvgU64(t->read4UsSecondaryTotal, rowCount),
           (unsigned long long)t->maxI2cReadUs,
           (unsigned long)t->sweepRequestCount,
           (unsigned long)t->sweepActuallyQueuedCount,
#if CONFIG_SENSORARRAY_FDC_TEXT_OUTPUT_FREQ_HZ
           "freqHz",
#elif CONFIG_SENSORARRAY_FDC_TEXT_OUTPUT_BOTH_SEPARATE
           "bothSeparate",
#elif CONFIG_SENSORARRAY_FDC_TEXT_OUTPUT_BOTH_INLINE_DEBUG
           "bothInlineDebug",
#else
           "capTotalPf",
#endif
           (CONFIG_SENSORARRAY_FDC_PARALLEL_DUAL_BUS_READ && s_fdcWorkersAvailable) ? 1u : 0u,
           SENSORARRAY_FDC_READY_MODE_NAME,
           SENSORARRAY_FDC_INTB_HINT_NAME);
    printf("SCAN_TIMING_WORKER_10,seqStart=%lu,seqEnd=%lu,frames=%lu,workerQueueSendAvgUs=%llu,workerSleepAckWaitAvgUs=%llu,workerStartGiveAvgUs=%llu,workerDoneWaitAvgUs=%llu,workerLateDoneUs=%llu,workerIdleAfterTimeoutUs=%llu,serialFallbackAvgUs=%llu,repairPrimaryAvgUs=%llu,repairSecondaryAvgUs=%llu,workerTimeoutCount=%lu,workerLateDoneCount=%lu,staleResultDiscardedCount=%lu,duplicateReadCount=%lu,outputFormatAvgUs=%llu,outputPrintfAvgUs=%llu,measureLockHeldAvgUs=%llu\n",
           (unsigned long)agg->seqStart,
           (unsigned long)agg->seqEnd,
           (unsigned long)agg->frames,
           (unsigned long long)sensorarrayMeasureAvgU64(t->workerQueueSendUs, rowCount),
           (unsigned long long)sensorarrayMeasureAvgU64(t->workerSleepAckWaitUs, rowCount),
           (unsigned long long)sensorarrayMeasureAvgU64(t->workerStartGiveUs, rowCount),
           (unsigned long long)sensorarrayMeasureAvgU64(t->workerDoneWaitUs, rowCount),
           (unsigned long long)t->workerLateDoneUs,
           (unsigned long long)t->workerIdleAfterTimeoutUs,
           (unsigned long long)sensorarrayMeasureAvgU64(t->serialFallbackUs, rowCount),
           (unsigned long long)sensorarrayMeasureAvgU64(t->repairPrimaryUs, rowCount),
           (unsigned long long)sensorarrayMeasureAvgU64(t->repairSecondaryUs, rowCount),
           (unsigned long)t->workerTimeoutCount,
           (unsigned long)t->workerLateDoneCount,
           (unsigned long)t->staleResultDiscardedCount,
           (unsigned long)t->duplicateReadCount,
           (unsigned long long)sensorarrayMeasureAvgU64(t->outputFormatUs, agg->frames),
           (unsigned long long)sensorarrayMeasureAvgU64(t->outputPrintfUs, agg->frames),
           (unsigned long long)sensorarrayMeasureAvgU64(t->measureLockHeldUs, agg->frames));
}

static void sensorarrayMeasureUpdateFdcTimingAggregate(const sensorarrayFdcTimingSummary_t *summary,
                                                       uint32_t sequence)
{
    if (!summary || !CONFIG_SENSORARRAY_FDC_TIMING_SUMMARY_AGGREGATE) {
        return;
    }

    uint32_t period = CONFIG_SENSORARRAY_FDC_TIMING_SUMMARY_PERIOD_FRAMES;
    if (period == 0u) {
        return;
    }

    if (!s_fdcTimingAggregate.active || s_fdcTimingAggregate.frames == 0u) {
        s_fdcTimingAggregate = (sensorarrayFdcTimingAggregate_t){
            .active = true,
            .seqStart = sequence,
            .frameUsMin = UINT64_MAX,
        };
    }

    sensorarrayFdcTimingAggregate_t *agg = &s_fdcTimingAggregate;
    agg->frames++;
    agg->seqEnd = sequence;
    agg->frameUsTotal += summary->frameUs;
    if (summary->frameUs < agg->frameUsMin) {
        agg->frameUsMin = summary->frameUs;
    }
    if (summary->frameUs > agg->frameUsMax) {
        agg->frameUsMax = summary->frameUs;
    }
    agg->rowUsTotal += summary->rowAvgUs * SENSORARRAY_MATRIX_ROWS;
    if (summary->rowMaxUs > agg->rowUsMax) {
        agg->rowUsMax = summary->rowMaxUs;
        agg->slowRow = summary->slowRow;
    }
    if (summary->frameUs > SENSORARRAY_FDC_TARGET_FRAME_US) {
        agg->overrunCount++;
    }

#define SENSORARRAY_FDC_AGG_ADD(field_) agg->totals.field_ += summary->field_
    SENSORARRAY_FDC_AGG_ADD(cacheApplyUs);
    SENSORARRAY_FDC_AGG_ADD(sleepBeforeRowSwitchUs);
    SENSORARRAY_FDC_AGG_ADD(rowSwitchWhileSleepingUs);
    SENSORARRAY_FDC_AGG_ADD(rowSettleUs);
    SENSORARRAY_FDC_AGG_ADD(diffApplyWhileSleepingUs);
    SENSORARRAY_FDC_AGG_ADD(sleepTotalUs);
    SENSORARRAY_FDC_AGG_ADD(sleepExitToIntbUs);
    SENSORARRAY_FDC_AGG_ADD(statusReadUs);
    SENSORARRAY_FDC_AGG_ADD(dataReadUs);
    SENSORARRAY_FDC_AGG_ADD(primaryJobUs);
    SENSORARRAY_FDC_AGG_ADD(secondaryJobUs);
    SENSORARRAY_FDC_AGG_ADD(dualBusWaitUs);
    SENSORARRAY_FDC_AGG_ADD(dualBusSkewUs);
    SENSORARRAY_FDC_AGG_ADD(workerQueueSendUs);
    SENSORARRAY_FDC_AGG_ADD(workerSleepAckWaitUs);
    SENSORARRAY_FDC_AGG_ADD(workerStartGiveUs);
    SENSORARRAY_FDC_AGG_ADD(workerDoneWaitUs);
    SENSORARRAY_FDC_AGG_ADD(workerLateDoneUs);
    SENSORARRAY_FDC_AGG_ADD(workerIdleAfterTimeoutUs);
    SENSORARRAY_FDC_AGG_ADD(serialFallbackUs);
    SENSORARRAY_FDC_AGG_ADD(repairPrimaryUs);
    SENSORARRAY_FDC_AGG_ADD(repairSecondaryUs);
    SENSORARRAY_FDC_AGG_ADD(outputFormatUs);
    SENSORARRAY_FDC_AGG_ADD(outputPrintfUs);
    SENSORARRAY_FDC_AGG_ADD(measureLockHeldUs);
    SENSORARRAY_FDC_AGG_ADD(cacheApplySkippedCount);
    SENSORARRAY_FDC_AGG_ADD(cacheApplyDirtyDeviceCount);
    SENSORARRAY_FDC_AGG_ADD(cacheApplyDiffWriteCount);
    SENSORARRAY_FDC_AGG_ADD(cacheApplyFullWriteCount);
    SENSORARRAY_FDC_AGG_ADD(cacheApplyNoDiffCount);
    SENSORARRAY_FDC_AGG_ADD(diffRcountWrites);
    SENSORARRAY_FDC_AGG_ADD(diffSettleWrites);
    SENSORARRAY_FDC_AGG_ADD(diffClockDivWrites);
    SENSORARRAY_FDC_AGG_ADD(diffDriveWrites);
    SENSORARRAY_FDC_AGG_ADD(diffMuxWrites);
    SENSORARRAY_FDC_AGG_ADD(diffStatusConfigWrites);
    SENSORARRAY_FDC_AGG_ADD(diffConfigWrites);
    SENSORARRAY_FDC_AGG_ADD(appliedFingerprintChanges);
    SENSORARRAY_FDC_AGG_ADD(i2cWriteCount);
    SENSORARRAY_FDC_AGG_ADD(i2cReadCount);
    SENSORARRAY_FDC_AGG_ADD(i2cVerifyReadCount);
    SENSORARRAY_FDC_AGG_ADD(i2cRetryCount);
    SENSORARRAY_FDC_AGG_ADD(i2cNackCount);
    SENSORARRAY_FDC_AGG_ADD(i2cTimeoutCount);
    SENSORARRAY_FDC_AGG_ADD(i2cRecoveryCount);
    SENSORARRAY_FDC_AGG_ADD(i2cMeasuredUs);
    SENSORARRAY_FDC_AGG_ADD(i2cBus0WriteCount);
    SENSORARRAY_FDC_AGG_ADD(i2cBus0ReadCount);
    SENSORARRAY_FDC_AGG_ADD(i2cBus0WriteBytes);
    SENSORARRAY_FDC_AGG_ADD(i2cBus0ReadBytes);
    SENSORARRAY_FDC_AGG_ADD(i2cBus0TotalUs);
    SENSORARRAY_FDC_AGG_ADD(i2cBus0RetryCount);
    SENSORARRAY_FDC_AGG_ADD(i2cBus0NackCount);
    SENSORARRAY_FDC_AGG_ADD(i2cBus0TimeoutCount);
    SENSORARRAY_FDC_AGG_ADD(i2cBus1WriteCount);
    SENSORARRAY_FDC_AGG_ADD(i2cBus1ReadCount);
    SENSORARRAY_FDC_AGG_ADD(i2cBus1WriteBytes);
    SENSORARRAY_FDC_AGG_ADD(i2cBus1ReadBytes);
    SENSORARRAY_FDC_AGG_ADD(i2cBus1TotalUs);
    SENSORARRAY_FDC_AGG_ADD(i2cBus1RetryCount);
    SENSORARRAY_FDC_AGG_ADD(i2cBus1NackCount);
    SENSORARRAY_FDC_AGG_ADD(i2cBus1TimeoutCount);
    SENSORARRAY_FDC_AGG_ADD(freshAmplitudeWarningCount);
    SENSORARRAY_FDC_AGG_ADD(staleAmplitudeWarningCount);
    SENSORARRAY_FDC_AGG_ADD(transientAmplitudeWarningCount);
    SENSORARRAY_FDC_AGG_ADD(warningReapplySuppressedCount);
    SENSORARRAY_FDC_AGG_ADD(warningFastSweepRequestedCount);
    SENSORARRAY_FDC_AGG_ADD(warningFastSweepSuppressedCooldownCount);
    SENSORARRAY_FDC_AGG_ADD(degradedCellCount);
    SENSORARRAY_FDC_AGG_ADD(intbFreshDrdyCount);
    SENSORARRAY_FDC_AGG_ADD(intbTimeoutCount);
    SENSORARRAY_FDC_AGG_ADD(intbFalseEdgeCount);
    SENSORARRAY_FDC_AGG_ADD(readyFullCount);
    SENSORARRAY_FDC_AGG_ADD(readyPartialCount);
    SENSORARRAY_FDC_AGG_ADD(readyNoneCount);
    SENSORARRAY_FDC_AGG_ADD(unreadWithoutDrdyCount);
    SENSORARRAY_FDC_AGG_ADD(dataNotReadyCount);
    SENSORARRAY_FDC_AGG_ADD(zeroBeforeReadyCount);
    SENSORARRAY_FDC_AGG_ADD(zeroAfterDrdyCount);
    SENSORARRAY_FDC_AGG_ADD(fallbackAttemptCount);
    SENSORARRAY_FDC_AGG_ADD(fallbackSuccessCount);
    SENSORARRAY_FDC_AGG_ADD(fallbackPartialCount);
    SENSORARRAY_FDC_AGG_ADD(fallbackFailCount);
    SENSORARRAY_FDC_AGG_ADD(rowFullInvalidCount);
    SENSORARRAY_FDC_AGG_ADD(deviceFullInvalidCount);
    SENSORARRAY_FDC_AGG_ADD(workerTimeoutCount);
    SENSORARRAY_FDC_AGG_ADD(workerLateDoneCount);
    SENSORARRAY_FDC_AGG_ADD(staleResultDiscardedCount);
    SENSORARRAY_FDC_AGG_ADD(duplicateReadCount);
    SENSORARRAY_FDC_AGG_ADD(waitReadyUsPrimaryTotal);
    SENSORARRAY_FDC_AGG_ADD(waitReadyUsSecondaryTotal);
    SENSORARRAY_FDC_AGG_ADD(read4UsPrimaryTotal);
    SENSORARRAY_FDC_AGG_ADD(read4UsSecondaryTotal);
    SENSORARRAY_FDC_AGG_ADD(sweepUs);
    SENSORARRAY_FDC_AGG_ADD(runtimeSweepCount);
    SENSORARRAY_FDC_AGG_ADD(sweepRequestCount);
    SENSORARRAY_FDC_AGG_ADD(sweepActuallyQueuedCount);
#undef SENSORARRAY_FDC_AGG_ADD
    if (summary->maxWaitReadyUs > agg->totals.maxWaitReadyUs) {
        agg->totals.maxWaitReadyUs = summary->maxWaitReadyUs;
    }
    if (summary->maxI2cReadUs > agg->totals.maxI2cReadUs) {
        agg->totals.maxI2cReadUs = summary->maxI2cReadUs;
    }

    if (agg->frames >= period) {
        sensorarrayMeasurePrintFdcTimingAggregate(agg);
        *agg = (sensorarrayFdcTimingAggregate_t){0};
    }
}

static void sensorarrayMeasurePrintFdcBottleneck(const sensorarrayFdcTimingSummary_t *summary,
                                                 uint32_t sequence)
{
    if (!summary || summary->frameUs <= SENSORARRAY_FDC_TARGET_FRAME_US) {
        return;
    }

    struct BottleneckItem {
        const char *name;
        uint64_t us;
    } items[] = {
        {"cacheApplyUs", summary->cacheApplyUs},
        {"i2cMeasuredUs", summary->i2cMeasuredUs},
        {"readyPollUs", summary->waitReadyUs},
        {"read4I2cUs", summary->readUs},
        {"rowMaxUs", summary->rowMaxUs},
        {"diffApplyWhileSleepingUs", summary->diffApplyWhileSleepingUs},
        {"workerDoneWaitUs", summary->workerDoneWaitUs},
        {"workerSleepAckWaitUs", summary->workerSleepAckWaitUs},
        {"workerIdleAfterTimeoutUs", summary->workerIdleAfterTimeoutUs},
        {"serialFallbackUs", summary->serialFallbackUs},
        {"repairPrimaryUs", summary->repairPrimaryUs},
        {"repairSecondaryUs", summary->repairSecondaryUs},
        {"outputPrintfUs", summary->outputPrintfUs},
        {"measureLockHeldUs", summary->measureLockHeldUs},
    };

    for (size_t i = 0u; i < sizeof(items) / sizeof(items[0]); ++i) {
        for (size_t j = i + 1u; j < sizeof(items) / sizeof(items[0]); ++j) {
            if (items[j].us > items[i].us) {
                struct BottleneckItem tmp = items[i];
                items[i] = items[j];
                items[j] = tmp;
            }
        }
    }

    const char *hint =
        (summary->cacheApplyDiffWriteCount > 64u) ? "excessive_diff_writes" :
        (summary->workerTimeoutCount > 0u) ? "worker_sync_timeout" :
        (summary->serialFallbackUs > 0u) ? "serial_fallback" :
        (summary->intbTimeoutCount > 0u) ? "intb_timeout_or_polling_fallback" :
        (summary->i2cMeasuredUs > SENSORARRAY_FDC_TARGET_FRAME_US) ? "i2c_transaction_budget" :
        "row_epoch_timing";
    printf("SCAN_BOTTLENECK,seq=%lu,top1=%s:%llu,top2=%s:%llu,top3=%s:%llu,hint=%s\n",
           (unsigned long)sequence,
           items[0].name,
           (unsigned long long)items[0].us,
           items[1].name,
           (unsigned long long)items[1].us,
           items[2].name,
           (unsigned long long)items[2].us,
           hint);
}

static void sensorarrayMeasurePrintFdcRowTiming(uint32_t sequence,
                                                const sensorarrayFdcRowTiming_t *rowTiming)
{
    if (!rowTiming) {
        return;
    }

    uint64_t rowOverrunUs = (rowTiming->rowUs > SENSORARRAY_FDC_TARGET_ROW_US) ?
        (rowTiming->rowUs - SENSORARRAY_FDC_TARGET_ROW_US) :
        0u;
    uint64_t serialEquivalentUs = rowTiming->primaryTotalUs + rowTiming->secondaryTotalUs;
    uint64_t parallelActualUs = (rowTiming->primaryTotalUs > rowTiming->secondaryTotalUs) ?
        rowTiming->primaryTotalUs :
        rowTiming->secondaryTotalUs;
    parallelActualUs += rowTiming->parallelJoinWaitUs;
    uint64_t parallelEfficiencyPct = parallelActualUs ?
        ((serialEquivalentUs * 100ull) / parallelActualUs) :
        0u;

    printf("SCAN_ROW_TIMING,seq=%lu,row=%u,targetRowUs=%lu,rowUs=%llu,rowOverrun=%u,rowSelectUs=%llu,analogSettleUs=%llu,sleepBeforeRowSwitchUs=%llu,rowSwitchWhileSleepingUs=%llu,rowSettleUs=%llu,primaryTotalUs=%llu,secondaryTotalUs=%llu,parallelJoinWaitUs=%llu,dualBusWaitUs=%llu,dualBusSkewUs=%llu,discardUs=%llu,waitReadyUs=%llu,readUs=%llu,rowValidMask=0x%02X,rowWarnMask=0x%02X,rowErrorMask=0x%02X,parallelEnabled=%u,serialEquivalentUs=%llu,parallelActualUs=%llu,parallelEfficiencyPct=%llu\n",
           (unsigned long)sequence,
           (unsigned)rowTiming->row,
           (unsigned long)SENSORARRAY_FDC_TARGET_ROW_US,
           (unsigned long long)rowTiming->rowUs,
           rowOverrunUs ? 1u : 0u,
           (unsigned long long)rowTiming->rowSelectUs,
           (unsigned long long)rowTiming->analogSettleUs,
           (unsigned long long)rowTiming->sleepBeforeRowSwitchUs,
           (unsigned long long)rowTiming->rowSwitchWhileSleepingUs,
           (unsigned long long)rowTiming->rowSettleUs,
           (unsigned long long)rowTiming->primaryTotalUs,
           (unsigned long long)rowTiming->secondaryTotalUs,
           (unsigned long long)rowTiming->parallelJoinWaitUs,
           (unsigned long long)rowTiming->dualBusWaitUs,
           (unsigned long long)rowTiming->dualBusSkewUs,
           (unsigned long long)rowTiming->discardUs,
           (unsigned long long)rowTiming->waitReadyUs,
           (unsigned long long)rowTiming->readUs,
           (unsigned)rowTiming->rowValidMask,
           (unsigned)rowTiming->rowWarnMask,
           (unsigned)rowTiming->rowErrorMask,
           (CONFIG_SENSORARRAY_FDC_PARALLEL_DUAL_BUS_READ && s_fdcWorkersAvailable) ? 1u : 0u,
           (unsigned long long)serialEquivalentUs,
           (unsigned long long)parallelActualUs,
           (unsigned long long)parallelEfficiencyPct);
}

static void sensorarrayMeasurePrintFdcDeviceTiming(uint32_t sequence,
                                                   const sensorarrayFdcDeviceTiming_t *deviceTiming)
{
    if (!deviceTiming) {
        return;
    }

    printf("SCAN_DEVICE_TIMING,seq=%lu,row=%u,device=%s,deviceUs=%llu,sleepEnterUs=%llu,applyUs=%llu,sleepExitUs=%llu,sleepExitToIntbUs=%llu,statusReadUs=%llu,dataReadUs=%llu,applyBuildConfigUs=%llu,channelConfigWriteUs=%llu,globalConfigWriteUs=%llu,verifyUs=%llu,discardUs=%llu,waitReadyUs=%llu,readRawUs=%llu,cacheDiffWriteCount=%lu,cacheNoDiffCount=%lu,intbEdges=%lu,intbFalseEdges=%lu,intbTimeouts=%lu,readyPollCount=%lu,readyFullCount=%lu,readyPartialCount=%lu,readyNoneCount=%lu,fallbackAttemptCount=%lu,fallbackSuccessCount=%lu,fallbackPartialCount=%lu,fallbackFailCount=%lu,deviceFullInvalidCount=%lu,maxWaitReadyUs=%llu,maxI2cReadUs=%llu,regWriteCount=%lu,regReadCount=%lu,verifyReadCount=%lu,retryCount=%lu,nackCount=%lu,timeoutCount=%lu\n",
           (unsigned long)sequence,
           (unsigned)deviceTiming->row,
           sensorarrayMeasureFdcDeviceName(deviceTiming->deviceId),
           (unsigned long long)deviceTiming->deviceUs,
           (unsigned long long)deviceTiming->sleepEnterUs,
           (unsigned long long)deviceTiming->applyUs,
           (unsigned long long)deviceTiming->sleepExitUs,
           (unsigned long long)deviceTiming->sleepExitToIntbUs,
           (unsigned long long)deviceTiming->statusReadUs,
           (unsigned long long)deviceTiming->dataReadUs,
           (unsigned long long)deviceTiming->applyBuildConfigUs,
           (unsigned long long)deviceTiming->channelConfigWriteUs,
           (unsigned long long)deviceTiming->globalConfigWriteUs,
           (unsigned long long)deviceTiming->verifyUs,
           (unsigned long long)deviceTiming->discardUs,
           (unsigned long long)deviceTiming->waitReadyUs,
           (unsigned long long)deviceTiming->readRawUs,
           (unsigned long)deviceTiming->cacheDiffWriteCount,
           (unsigned long)deviceTiming->cacheNoDiffCount,
           (unsigned long)deviceTiming->intbEdgeCount,
           (unsigned long)deviceTiming->intbFalseEdgeCount,
           (unsigned long)deviceTiming->intbTimeoutCount,
           (unsigned long)deviceTiming->readyPollCount,
           (unsigned long)deviceTiming->readyFullCount,
           (unsigned long)deviceTiming->readyPartialCount,
           (unsigned long)deviceTiming->readyNoneCount,
           (unsigned long)deviceTiming->fallbackAttemptCount,
           (unsigned long)deviceTiming->fallbackSuccessCount,
           (unsigned long)deviceTiming->fallbackPartialCount,
           (unsigned long)deviceTiming->fallbackFailCount,
           (unsigned long)deviceTiming->deviceFullInvalidCount,
           (unsigned long long)deviceTiming->maxWaitReadyUs,
           (unsigned long long)deviceTiming->maxI2cReadUs,
           (unsigned long)deviceTiming->regWriteCount,
           (unsigned long)deviceTiming->regReadCount,
           (unsigned long)deviceTiming->verifyReadCount,
           (unsigned long)deviceTiming->retryCount,
           (unsigned long)deviceTiming->nackCount,
           (unsigned long)deviceTiming->timeoutCount);
}


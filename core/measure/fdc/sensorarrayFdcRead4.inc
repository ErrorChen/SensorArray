// Split from core/measure/sensorarrayMeasure.c to keep FDC matrix internals isolated.
static esp_err_t sensorarrayMeasureReadFdcAutoscan4chMasked(sensorarrayFdcDeviceState_t *fdcState,
                                                            uint8_t freshMask4,
                                                            const sensorarrayFdcReadyState_t *ready,
                                                            sensorarrayFdcAutoscanSamples_t *outSamples)
{
    if (!fdcState || !fdcState->handle || !outSamples) {
        return ESP_ERR_INVALID_ARG;
    }

    *outSamples = (sensorarrayFdcAutoscanSamples_t){0};
    freshMask4 &= 0x0Fu;
    bool readyForDataRead = ready && ready->readyForDataRead;
    if (ready && !readyForDataRead) {
        outSamples->statusRaw = ready->statusRaw;
        outSamples->unreadMask = ready->unreadMask & 0x0Fu;
        outSamples->dataReady = ready->dataReady;
        outSamples->unreadWithoutDrdy = ready->unreadWithoutDataReady;
        outSamples->timeout = ready->timeout;
        outSamples->partial = ready->partial;
        outSamples->errorMask = 0x0Fu;
        for (uint8_t ch = 0u; ch < 4u; ++ch) {
            uint8_t bit = (uint8_t)(1u << ch);
            outSamples->notReady[ch] = true;
            outSamples->notReadyMask |= bit;
            if (ready->unreadWithoutDataReady) {
                outSamples->zeroBeforeReady[ch] = true;
                outSamples->zeroBeforeReadyMask |= bit;
            }
        }
        return ESP_OK;
    }
    outSamples->dataReady = ready && ready->dataReady;
    outSamples->unreadWithoutDrdy = ready && ready->unreadWithoutDataReady;
    outSamples->freshMask = readyForDataRead ? freshMask4 : 0u;
    Fdc2214CapFastChannelSample_t fastSamples[4] = {0};
    esp_err_t firstErr = Fdc2214CapReadChannelsDataRegsFast(fdcState->handle,
                                                            freshMask4,
                                                            fastSamples,
                                                            4u);
    for (uint8_t ch = 0u; ch < 4u; ++ch) {
        const Fdc2214CapFastChannelSample_t *sample = &fastSamples[ch];
        uint8_t bit = (uint8_t)(1u << ch);
        bool requestedFresh = readyForDataRead && (freshMask4 & bit) != 0u;
        outSamples->fresh[ch] = requestedFresh;
        outSamples->raw28[ch] = sample->raw28;
        outSamples->statusRaw = sample->statusRaw;
        if (requestedFresh || sample->unreadConversion) {
            outSamples->unreadMask |= (uint8_t)(1u << ch);
        }
        outSamples->amplitudeWarning[ch] =
            sample->errAmplitude ||
            ((sample->errorMask & FDC2214CAP_FAST_ERROR_AMPLITUDE) != 0u);
        bool currentReady = requestedFresh || sample->dataReady || sample->unreadConversion;
        outSamples->freshAmplitudeWarning[ch] = outSamples->amplitudeWarning[ch] &&
                                                currentReady &&
                                                firstErr == ESP_OK;
        outSamples->staleAmplitudeWarning[ch] = outSamples->amplitudeWarning[ch] &&
                                                !currentReady &&
                                                firstErr == ESP_OK;
        outSamples->transientAmplitudeWarning[ch] = outSamples->amplitudeWarning[ch] &&
                                                    firstErr != ESP_OK;
        outSamples->watchdogFault[ch] =
            sample->errWatchdog ||
            ((sample->errorMask & FDC2214CAP_FAST_ERROR_WATCHDOG) != 0u);
        outSamples->saturated[ch] = sample->raw28 >= SENSORARRAY_FDC_RAW28_SATURATED_THRESHOLD;
        outSamples->i2cError[ch] = (sample->errorMask & FDC2214CAP_FAST_ERROR_I2C) != 0u;
        outSamples->notReady[ch] = !readyForDataRead && !outSamples->i2cError[ch];
        outSamples->zeroBeforeReady[ch] = sample->raw28 == 0u &&
                                          !readyForDataRead &&
                                          !outSamples->i2cError[ch];
        outSamples->zeroAfterDrdy[ch] = sample->raw28 == 0u &&
                                        readyForDataRead &&
                                        !outSamples->i2cError[ch];
        if (outSamples->notReady[ch]) {
            outSamples->notReadyMask |= bit;
        }
        if (outSamples->zeroBeforeReady[ch]) {
            outSamples->zeroBeforeReadyMask |= bit;
        }
        if (outSamples->zeroAfterDrdy[ch]) {
            outSamples->zeroAfterDrdyMask |= bit;
        }

        bool i2cOk = !outSamples->i2cError[ch];
        bool readable = requestedFresh;
        outSamples->valid[ch] = i2cOk &&
                                readable &&
                                sample->raw28 != 0u &&
                                !outSamples->watchdogFault[ch] &&
                                !outSamples->saturated[ch];
        if (outSamples->valid[ch]) {
            outSamples->validMask |= (uint8_t)(1u << ch);
        }
        if (outSamples->amplitudeWarning[ch]) {
            outSamples->warnMask |= (uint8_t)(1u << ch);
        }
        if (!outSamples->valid[ch]) {
            outSamples->errorMask |= (uint8_t)(1u << ch);
        }
    }
    outSamples->partial = freshMask4 != 0u && freshMask4 != SENSORARRAY_FDC_AUTOSCAN_READY_UNREAD_MASK;
    outSamples->i2cTransactionError = firstErr != ESP_OK;
    return firstErr;
}

static esp_err_t sensorarrayMeasureReadFdcAutoscan4ch(sensorarrayFdcDeviceState_t *fdcState,
                                                      sensorarrayFdcAutoscanSamples_t *outSamples)
{
    return sensorarrayMeasureReadFdcAutoscan4chMasked(fdcState,
                                                     SENSORARRAY_FDC_AUTOSCAN_READY_UNREAD_MASK,
                                                     NULL,
                                                     outSamples);
}

static void sensorarrayMeasureMarkFdcNoFreshSamples(sensorarrayFdcAutoscanSamples_t *outSamples,
                                                    const sensorarrayFdcReadyState_t *ready,
                                                    bool i2cError)
{
    if (!outSamples) {
        return;
    }

    *outSamples = (sensorarrayFdcAutoscanSamples_t){0};
    if (ready) {
        outSamples->statusRaw = ready->statusRaw;
        outSamples->unreadMask = ready->unreadMask & 0x0Fu;
        outSamples->timeout = ready->timeout;
        outSamples->partial = ready->partial;
        outSamples->dataReady = ready->dataReady;
        outSamples->unreadWithoutDrdy = ready->unreadWithoutDataReady;
    }
    outSamples->i2cTransactionError = i2cError;
    outSamples->errorMask = 0x0Fu;
    bool runtimeNotReady = ready &&
                           !i2cError &&
                           ready->err != ESP_ERR_NOT_FOUND &&
                           !ready->readyForDataRead &&
                           (ready->pollCount != 0u ||
                            ready->timeout ||
                            ready->unreadWithoutDataReady);
    for (uint8_t ch = 0u; ch < 4u; ++ch) {
        uint8_t bit = (uint8_t)(1u << ch);
        outSamples->i2cError[ch] = i2cError;
        outSamples->notReady[ch] = runtimeNotReady;
        outSamples->zeroBeforeReady[ch] = outSamples->notReady[ch] &&
                                          ready &&
                                          ready->unreadWithoutDataReady;
        if (outSamples->notReady[ch]) {
            outSamples->notReadyMask |= bit;
        }
        if (outSamples->zeroBeforeReady[ch]) {
            outSamples->zeroBeforeReadyMask |= bit;
        }
    }
}

static const char *sensorarrayMeasureFdcRead4DiagnosticName(const sensorarrayFdcDeviceRead4Result_t *read4)
{
    if (!read4) {
        return "invalid_read4_state";
    }
    if (read4->readUs == 0u &&
        read4->pollCount == 0u &&
        read4->unreadMask4 != SENSORARRAY_FDC_AUTOSCAN_READY_UNREAD_MASK) {
        return "bug_ready_poll_not_entered";
    }
    if (read4->i2cError || read4->readyKind == SENSORARRAY_FDC_READY_I2C_ERROR) {
        return "i2c_error";
    }
    if (read4->zeroBeforeReadyMask4 != 0u) {
        return "zero_before_ready";
    }
    if (read4->unreadWithoutDrdy) {
        return "unsafe_unread_no_drdy";
    }
    if (read4->notReadyMask4 != 0u) {
        return "data_not_ready";
    }
    if (read4->zeroAfterDrdyMask4 != 0u) {
        return "zero_after_drdy";
    }
    if (read4->readUs == 0u &&
        read4->unreadMask4 != SENSORARRAY_FDC_AUTOSCAN_READY_UNREAD_MASK) {
        return (read4->unreadMask4 != 0u) ? "partial_unread" : "no_unread_after_poll";
    }
    if (read4->unreadMask4 != SENSORARRAY_FDC_AUTOSCAN_READY_UNREAD_MASK) {
        return (read4->unreadMask4 != 0u) ? "partial_unread" : "no_unread_after_poll";
    }
    return "normal";
}

static void sensorarrayMeasureBuildFdcRead4Result(uint8_t row,
                                                  uint32_t epochId,
                                                  sensorarrayFdcDeviceId_t devId,
                                                  const sensorarrayFdcAutoscanSamples_t *samples,
                                                  const sensorarrayFdcRuntimeChannelConfig_t configs[4],
                                                  const sensorarrayFdcReadyState_t *ready,
                                                  esp_err_t readErr,
                                                  uint32_t readUs,
                                                  bool logNormal,
                                                  sensorarrayFdcDeviceRead4Result_t *outRead4)
{
    if (!outRead4) {
        return;
    }

    *outRead4 = (sensorarrayFdcDeviceRead4Result_t){
        .readErr = readErr,
        .i2cErr = (samples && samples->i2cTransactionError) ? readErr : ESP_OK,
        .readyKind = ready ? ready->kind : SENSORARRAY_FDC_READY_NONE,
        .status = ready ? ready->statusRaw : (samples ? samples->statusRaw : 0u),
        .errorStatus = ready ? ready->errorStatus : 0u,
        .unreadMask4 = ready ? (ready->unreadMask & 0x0Fu) : (samples ? samples->unreadMask : 0u),
        .drdy = ready ? ready->drdy : 0u,
        .timeout = ready ? ready->timeout : false,
        .partial = ready ? ready->partial : false,
        .i2cError = samples ? samples->i2cTransactionError : false,
        .notReadyMask4 = samples ? samples->notReadyMask : 0u,
        .zeroBeforeReadyMask4 = samples ? samples->zeroBeforeReadyMask : 0u,
        .zeroAfterDrdyMask4 = samples ? samples->zeroAfterDrdyMask : 0u,
        .unreadWithoutDrdy = ready ? ready->unreadWithoutDataReady : false,
        .waitUs = ready ? ready->waitUs : 0u,
        .readUs = readUs,
        .pollCount = ready ? ready->pollCount : 0u,
        .edgeDelta = ready ? ready->edgeDelta : 0u,
    };

    const double inductorUh = (double)CONFIG_SENSORARRAY_FDC_TANK_INDUCTOR_NH / 1000.0;
    for (uint8_t ch = 0u; ch < 4u; ++ch) {
        bool fresh = samples && samples->fresh[ch];
        bool configOk = configs && configs[ch].valid;
        uint32_t raw28 = samples ? samples->raw28[ch] : 0u;
        double freqHz = (fresh && configOk) ?
            sensorarrayMeasureFdcRaw28ToFreqHz(raw28, configs[ch].effectiveFclkHz, configs[ch].clockDividers) :
            0.0;
        bool valid = samples && samples->valid[ch] && configOk && freqHz > 0.0;
        bool warning = samples && samples->amplitudeWarning[ch];
        bool error = !valid;

        outRead4->raw28[ch] = raw28;
        outRead4->freqHz[ch] = valid ? freqHz : SENSORARRAY_FDC_INVALID_FREQ_SENTINEL_HZ;
        outRead4->capTotalPf[ch] = SENSORARRAY_FDC_INVALID_CAP_SENTINEL_PF;
        if (valid) {
            double capPf = 0.0;
            if (sensorarrayMeasureFdcComputeCapacitancePf(freqHz, inductorUh, &capPf)) {
                outRead4->capTotalPf[ch] = capPf;
                outRead4->validMask4 |= (uint8_t)(1u << ch);
            } else {
                valid = false;
                error = true;
                outRead4->freqHz[ch] = SENSORARRAY_FDC_INVALID_FREQ_SENTINEL_HZ;
            }
        }
        if (fresh) {
            outRead4->freshMask4 |= (uint8_t)(1u << ch);
        }
        if (warning) {
            outRead4->warnMask4 |= (uint8_t)(1u << ch);
        }
        if (error) {
            outRead4->errorMask4 |= (uint8_t)(1u << ch);
        }
    }
    outRead4->rawAllZero =
        outRead4->raw28[0] == 0u &&
        outRead4->raw28[1] == 0u &&
        outRead4->raw28[2] == 0u &&
        outRead4->raw28[3] == 0u;

    if (outRead4->timeout && outRead4->freshMask4 != SENSORARRAY_FDC_AUTOSCAN_READY_UNREAD_MASK) {
        outRead4->partial = outRead4->freshMask4 != 0u;
        outRead4->readErr = ESP_ERR_TIMEOUT;
    }

    bool diagnostic = outRead4->readUs == 0u &&
                      (outRead4->notReadyMask4 != 0u ||
                       outRead4->unreadMask4 != SENSORARRAY_FDC_AUTOSCAN_READY_UNREAD_MASK ||
                       !ready ||
                       !ready->readyForDataRead);
    const char *diagnosticName = sensorarrayMeasureFdcRead4DiagnosticName(outRead4);
    bool fullSuccess = outRead4->readErr == ESP_OK &&
                       !outRead4->timeout &&
                       !outRead4->partial &&
                       (outRead4->readyKind == SENSORARRAY_FDC_READY_EDGE_WAKE ||
                        outRead4->readyKind == SENSORARRAY_FDC_READY_POLL_FULL) &&
                       outRead4->validMask4 == SENSORARRAY_FDC_AUTOSCAN_READY_UNREAD_MASK;
    if (fullSuccess && !logNormal) {
        return;
    }
    printf("FDC_DEVICE_READ4,dev=%s,row=%u,epoch=%lu,readMode=%s,diagnostic=%s,rawSource=%s,raw28=[%lu,%lu,%lu,%lu],freqHz=[%.3f,%.3f,%.3f,%.3f],freshMask=0x%X,validMask=0x%X,warnMask=0x%X,errorMask=0x%X,notReadyMask=0x%X,zeroBeforeReadyMask=0x%X,zeroAfterDrdyMask=0x%X,rawAllZero=%u,status=0x%04X,unread=0x%X,drdy=%u,partial=%u,timeout=%u,edgeDelta=%lu,waitUs=%lu,readUs=%lu,pollCount=%lu,readyMode=%s,err=0x%lx,errName=%s\n",
           sensorarrayMeasureFdcDeviceName(devId),
           (unsigned)row,
           (unsigned long)epochId,
           diagnostic ? "skipped_not_ready" : "normal",
           diagnosticName,
           diagnostic ? "not_read" : "device_data",
           (unsigned long)outRead4->raw28[0],
           (unsigned long)outRead4->raw28[1],
           (unsigned long)outRead4->raw28[2],
           (unsigned long)outRead4->raw28[3],
           outRead4->freqHz[0],
           outRead4->freqHz[1],
           outRead4->freqHz[2],
           outRead4->freqHz[3],
           (unsigned)outRead4->freshMask4,
           (unsigned)outRead4->validMask4,
           (unsigned)outRead4->warnMask4,
           (unsigned)outRead4->errorMask4,
           (unsigned)outRead4->notReadyMask4,
           (unsigned)outRead4->zeroBeforeReadyMask4,
           (unsigned)outRead4->zeroAfterDrdyMask4,
           outRead4->rawAllZero ? 1u : 0u,
           outRead4->status,
           (unsigned)outRead4->unreadMask4,
           (unsigned)outRead4->drdy,
           outRead4->partial ? 1u : 0u,
           outRead4->timeout ? 1u : 0u,
           (unsigned long)outRead4->edgeDelta,
           (unsigned long)outRead4->waitUs,
           (unsigned long)outRead4->readUs,
           (unsigned long)outRead4->pollCount,
           SENSORARRAY_FDC_READY_MODE_NAME,
           (unsigned long)outRead4->readErr,
           sensorarrayMeasureEspErrName(outRead4->readErr));
    if (outRead4->rawAllZero || diagnostic || outRead4->zeroBeforeReadyMask4 != 0u ||
        outRead4->zeroAfterDrdyMask4 != 0u) {
        const char *decision =
            (outRead4->zeroBeforeReadyMask4 != 0u || outRead4->unreadWithoutDrdy) ? "rewait" :
            (outRead4->zeroAfterDrdyMask4 != 0u) ? "error" :
            (outRead4->validMask4 != 0u) ? "accepted" :
            "discard";
        printf("FDC_READ_CONSISTENCY,dev=%s,row=%u,epoch=%lu,status=0x%04X,drdy=%u,unread=0x%X,rawAllZero=%u,validMask=0x%X,notReadyMask=0x%X,zeroBeforeReadyMask=0x%X,zeroAfterDrdyMask=0x%X,decision=%s\n",
               sensorarrayMeasureFdcDeviceName(devId),
               (unsigned)row,
               (unsigned long)epochId,
               outRead4->status,
               (unsigned)outRead4->drdy,
               (unsigned)outRead4->unreadMask4,
               outRead4->rawAllZero ? 1u : 0u,
               (unsigned)outRead4->validMask4,
               (unsigned)outRead4->notReadyMask4,
               (unsigned)outRead4->zeroBeforeReadyMask4,
               (unsigned)outRead4->zeroAfterDrdyMask4,
               decision);
    }
}

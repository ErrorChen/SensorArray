#include "sensorarrayAdsMatrix.h"

#include <limits.h>
#include <string.h>

#include "esp_rom_sys.h"
#include "esp_timer.h"

#include "sensorarrayAdsGap.h"
#include "sensorarrayAdsMath.h"
#include "sensorarrayBoardMap.h"
#include "sensorarrayConfig.h"
#include "sensorarrayFrameBuilder.h"

typedef struct {
    int32_t raw;
    int32_t differentialUv;
    uint8_t status;
    uint32_t generationStart;
    uint32_t generationEnd;
} sensorarrayAdsMatrixSample_t;

typedef struct {
    int32_t raw;
    int32_t differentialUv;
    int32_t nodeUv;
    uint8_t status;
    uint8_t gain;
    sensorarrayCellError_t error;
    uint8_t gainChanges;
    uint8_t attempts;
    uint8_t ioRetries;
    bool fresh;
    bool stable;
    bool pgaBypassed;
    bool fullSample;
    uint32_t spreadRaw;
} sensorarrayAdsMatrixCellResult_t;

static const sensorarrayAdsAutoRangeConfig_t s_sensorarrayAdsAutoRangeConfig = {
    .increaseBelowPermille = CONFIG_SENSORARRAY_ADS_AUTORANGE_INCREASE_PERMILLE,
    .decreaseAbovePermille = CONFIG_SENSORARRAY_ADS_AUTORANGE_DECREASE_PERMILLE,
    .saturationPermille = CONFIG_SENSORARRAY_ADS_AUTORANGE_SATURATION_PERMILLE,
    .maximumAttempts = CONFIG_SENSORARRAY_ADS_AUTORANGE_MAX_ATTEMPTS,
    .pgaRailMarginUv = CONFIG_SENSORARRAY_ADS_AUTORANGE_RAIL_MARGIN_UV,
};

static const sensorarrayAdsVoltageLimits_t s_sensorarrayAdsVoltageLimits = {
    .minimumUv = CONFIG_SENSORARRAY_ADS_MATRIX_VOLT_MIN_UV,
    .maximumUv = CONFIG_SENSORARRAY_ADS_MATRIX_VOLT_MAX_UV,
};

static const sensorarrayAdsResistanceConfig_t s_sensorarrayAdsResistanceLimits = {
    .referenceResistorOhms = 1u,
    .pathOffsetMilliohms = 0,
    .minimumOhms = CONFIG_SENSORARRAY_ADS_MATRIX_RES_MIN_OHMS,
    .maximumOhms = CONFIG_SENSORARRAY_ADS_MATRIX_RES_MAX_OHMS,
    .shortThresholdOhms = CONFIG_SENSORARRAY_ADS_MATRIX_RES_SHORT_OHMS,
    .openDenominatorUv = CONFIG_SENSORARRAY_ADS_MATRIX_RES_OPEN_DENOM_UV,
};

bool sensorarrayAdsMatrixCalibrationValid(
    const sensorarrayAdsMatrixCalibration_t *calibration)
{
    if (!calibration ||
        calibration->version != SENSORARRAY_ADS_MATRIX_CALIBRATION_VERSION ||
        calibration->referenceResistorOhms == 0u ||
        calibration->referenceResistorOhms > 100000000u ||
        calibration->matrixReferenceSpanUv < 900000u ||
        calibration->matrixReferenceSpanUv > 5000000u ||
        calibration->globalPathOffsetMilliohms < -2000000000LL ||
        calibration->globalPathOffsetMilliohms > 2000000000LL) {
        return false;
    }
    for (size_t index = 0u; index < 2u; ++index) {
        if (calibration->bankOffsetMilliohms[index] < -1000000000 ||
            calibration->bankOffsetMilliohms[index] > 1000000000) {
            return false;
        }
    }
    for (size_t index = 0u; index < SENSORARRAY_MEASUREMENT_MAX_CELLS; ++index) {
        if ((calibration->cellOffsetValidMask & (UINT64_C(1) << index)) != 0u &&
            (calibration->cellOffsetMilliohms[index] < -1000000000 ||
             calibration->cellOffsetMilliohms[index] > 1000000000)) {
            return false;
        }
    }
    return true;
}

static uint64_t sensorarrayAdsMatrixActiveMask(uint8_t rows)
{
    uint8_t cells = (uint8_t)(rows * SENSORARRAY_MATRIX_COLS);
    return cells >= 64u ? UINT64_MAX : ((UINT64_C(1) << cells) - 1u);
}

static void sensorarrayAdsMatrixSetCellError(sensorarrayFrame_t *frame,
                                             size_t index,
                                             sensorarrayCellError_t error,
                                             bool fresh)
{
    uint64_t bit = UINT64_C(1) << index;
    frame->measurement.valuesFixed[index] = SENSORARRAY_MEASUREMENT_INVALID_FIXED;
    frame->measurement.errorReason[index] = (uint8_t)error;
    frame->measurement.errorMask |= bit;
    frame->errorMask |= bit;
    if (fresh) {
        frame->measurement.freshMask |= bit;
        frame->freshMask |= bit;
        if (frame->freshCount < UINT8_MAX) {
            frame->freshCount++;
        }
    }
    if (error == SENSORARRAY_CELL_ERROR_OPEN) {
        frame->measurement.openMask |= bit;
    } else if (error == SENSORARRAY_CELL_ERROR_SHORT) {
        frame->measurement.shortMask |= bit;
    } else if (error == SENSORARRAY_CELL_ERROR_UNSTABLE) {
        frame->measurement.unstableMask |= bit;
    } else if (error == SENSORARRAY_CELL_ERROR_SATURATED ||
               error == SENSORARRAY_CELL_ERROR_PGA_ABSOLUTE ||
               error == SENSORARRAY_CELL_ERROR_PGA_DIFFERENTIAL) {
        frame->measurement.saturatedMask |= bit;
    }
}

static void sensorarrayAdsMatrixSetCellValid(sensorarrayFrame_t *frame,
                                             size_t index,
                                             int64_t valueFixed,
                                             const sensorarrayAdsMatrixCellResult_t *result)
{
    uint64_t bit = UINT64_C(1) << index;
    frame->measurement.valuesFixed[index] = valueFixed;
    frame->measurement.rawCode[index] = result->raw;
    frame->measurement.nodeUv[index] = result->nodeUv;
    frame->measurement.pgaGain[index] = result->gain;
    frame->measurement.errorReason[index] = SENSORARRAY_CELL_ERROR_NONE;
    frame->measurement.validMask |= bit;
    frame->measurement.freshMask |= bit;
    frame->validMask |= bit;
    frame->freshMask |= bit;
    frame->errorMask &= ~bit;
    if (frame->validCount < UINT8_MAX) {
        frame->validCount++;
    }
    if (frame->freshCount < UINT8_MAX) {
        frame->freshCount++;
    }
}

static esp_err_t sensorarrayAdsMatrixReadFreshAfter(
    ads126xAdcHandle_t *ads,
    uint32_t startGeneration,
    sensorarrayMeasurementPayload_t *telemetry,
    sensorarrayAdsMatrixSample_t *outSample)
{
    if (!ads || !telemetry || !outSample) {
        return ESP_ERR_INVALID_ARG;
    }
    memset(outSample, 0, sizeof(*outSample));
    outSample->generationStart = startGeneration;
    int64_t waitStartUs = esp_timer_get_time();
    esp_err_t err = ads126xAdcWaitDrdyGenerationUs(
        ads,
        outSample->generationStart,
        (uint32_t)CONFIG_SENSORARRAY_ADS_DRDY_TIMEOUT_US,
        &outSample->generationEnd);
    int64_t waitEndUs = esp_timer_get_time();
    if (waitEndUs > waitStartUs) {
        telemetry->drdyWaitUs += (uint64_t)(waitEndUs - waitStartUs);
    }
    if (err != ESP_OK) {
        return err;
    }
    uint32_t readUs = 0u;
    err = ads126xAdcReadAdc1RawDmaReady(ads,
                                        &outSample->raw,
                                        &outSample->status,
                                        &readUs);
    telemetry->sampleReadUs += readUs;
    if (err != ESP_OK) {
        return err;
    }
    if (outSample->generationEnd == outSample->generationStart ||
        !ads126xAdcStatusByteHasAdc1NewData(ads, outSample->status)) {
        return ESP_ERR_INVALID_RESPONSE;
    }
    telemetry->rawConversionCount++;
    outSample->differentialUv = ads126xAdcRawToMicrovolts(ads, outSample->raw);
    return ESP_OK;
}

static bool sensorarrayAdsMatrixNoteRegisterSnapshot(
    sensorarrayAdsRegisterCache_t *cache,
    const ads126xAdc1RegisterSnapshot_t *snapshot)
{
    if (!cache || !snapshot) {
        return false;
    }
    const uint8_t values[SENSORARRAY_ADS_REGISTER_COUNT] = {
        [SENSORARRAY_ADS_REGISTER_POWER] = snapshot->power,
        [SENSORARRAY_ADS_REGISTER_INTERFACE] = snapshot->interface,
        [SENSORARRAY_ADS_REGISTER_MODE0] = snapshot->mode0,
        [SENSORARRAY_ADS_REGISTER_MODE1] = snapshot->mode1,
        [SENSORARRAY_ADS_REGISTER_MODE2] = snapshot->mode2,
        [SENSORARRAY_ADS_REGISTER_INPMUX] = snapshot->inpmux,
        [SENSORARRAY_ADS_REGISTER_REFMUX] = snapshot->refmux,
        [SENSORARRAY_ADS_REGISTER_OFCAL0] = snapshot->offsetCal[0],
        [SENSORARRAY_ADS_REGISTER_OFCAL1] = snapshot->offsetCal[1],
        [SENSORARRAY_ADS_REGISTER_OFCAL2] = snapshot->offsetCal[2],
        [SENSORARRAY_ADS_REGISTER_FSCAL0] = snapshot->fullScaleCal[0],
        [SENSORARRAY_ADS_REGISTER_FSCAL1] = snapshot->fullScaleCal[1],
        [SENSORARRAY_ADS_REGISTER_FSCAL2] = snapshot->fullScaleCal[2],
    };
    bool match = true;
    for (uint8_t index = 0u; index < SENSORARRAY_ADS_REGISTER_COUNT; ++index) {
        if (!sensorarrayAdsRegisterCacheNoteReadback(
                cache, (sensorarrayAdsRegisterId_t)index, values[index])) {
            match = false;
        }
    }
    return match;
}

static esp_err_t sensorarrayAdsMatrixRefreshRegisterShadow(
    sensorarrayAdsMatrixEngine_t *engine,
    sensorarrayMeasurementPayload_t *telemetry)
{
    if (!engine || !engine->state || !telemetry) {
        return ESP_ERR_INVALID_ARG;
    }
    ads126xAdc1RegisterSnapshot_t snapshot = {0};
    int64_t readStartUs = esp_timer_get_time();
    esp_err_t err = ads126xAdcReadAdc1RegisterSnapshot(&engine->state->ads,
                                                       &snapshot);
    int64_t readEndUs = esp_timer_get_time();
    if (readEndUs > readStartUs) {
        telemetry->registerReadbackUs += (uint64_t)(readEndUs - readStartUs);
    }
    if (err != ESP_OK) {
        sensorarrayAdsRegisterCacheInvalidate(&engine->registerCache);
        return err;
    }
    sensorarrayRouteSnapshot_t route = {0};
    if (!engine->routeController ||
        !sensorarrayRouteControllerCopySnapshot(engine->routeController, &route)) {
        sensorarrayAdsRegisterCacheInvalidate(&engine->registerCache);
        return ESP_ERR_INVALID_STATE;
    }
    bool expectedIntref = route.profile.intRef == SENSORARRAY_ADS_INTREF_ON;
    bool expectedVbias = route.profile.vbias == SENSORARRAY_ADS_VBIAS_ON;
    bool routeRegistersMatch =
        (((snapshot.power & ADS126X_POWER_INTREF) != 0u) == expectedIntref) &&
        (((snapshot.power & ADS126X_POWER_VBIAS) != 0u) == expectedVbias) &&
        snapshot.refmux == route.profile.adsRefMux;
    if (!routeRegistersMatch) {
        /* A block read is already duplicate-verified by the driver. One more
         * verified snapshot distinguishes a recoverable DOUT read upset from
         * persistent hardware state before declaring the route unsafe. */
        ads126xAdc1RegisterSnapshot_t retry = {0};
        readStartUs = esp_timer_get_time();
        err = ads126xAdcReadAdc1RegisterSnapshot(&engine->state->ads, &retry);
        readEndUs = esp_timer_get_time();
        if (readEndUs > readStartUs) {
            telemetry->registerReadbackUs += (uint64_t)(readEndUs - readStartUs);
        }
        bool retryMatches = err == ESP_OK &&
            ((((retry.power & ADS126X_POWER_INTREF) != 0u) == expectedIntref)) &&
            ((((retry.power & ADS126X_POWER_VBIAS) != 0u) == expectedVbias)) &&
            retry.refmux == route.profile.adsRefMux;
        if (!retryMatches) {
            sensorarrayAdsRegisterCacheInvalidate(&engine->registerCache);
            sensorarrayAdsProfileCacheInvalidate(&engine->profileCache);
            sensorarrayAdsValueCacheInvalidate(&engine->valueCache);
            return err == ESP_OK ? ESP_ERR_INVALID_RESPONSE : err;
        }
        snapshot = retry;
    }
    bool expectedMode = snapshot.mode0 == (uint8_t)CONFIG_SENSORARRAY_ADS_MODE0_VALUE &&
                        snapshot.mode1 == (uint8_t)CONFIG_SENSORARRAY_ADS_MODE1_VALUE;
    if (!expectedMode) {
        int64_t writeStartUs = esp_timer_get_time();
        err = ads126xAdcConfigureAdc1Mode(&engine->state->ads,
                                          (uint8_t)CONFIG_SENSORARRAY_ADS_MODE0_VALUE,
                                          (uint8_t)CONFIG_SENSORARRAY_ADS_MODE1_VALUE);
        int64_t writeEndUs = esp_timer_get_time();
        if (writeEndUs > writeStartUs) {
            telemetry->registerWriteUs += (uint64_t)(writeEndUs - writeStartUs);
        }
        if (err != ESP_OK) {
            sensorarrayAdsRegisterCacheInvalidate(&engine->registerCache);
            return err;
        }
        sensorarrayAdsRegisterCacheNoteWrite(&engine->registerCache,
                                              SENSORARRAY_ADS_REGISTER_MODE0,
                                              (uint8_t)CONFIG_SENSORARRAY_ADS_MODE0_VALUE,
                                              true);
        sensorarrayAdsRegisterCacheNoteWrite(&engine->registerCache,
                                              SENSORARRAY_ADS_REGISTER_MODE1,
                                              (uint8_t)CONFIG_SENSORARRAY_ADS_MODE1_VALUE,
                                              true);
        readStartUs = esp_timer_get_time();
        err = ads126xAdcReadAdc1RegisterSnapshot(&engine->state->ads, &snapshot);
        readEndUs = esp_timer_get_time();
        if (readEndUs > readStartUs) {
            telemetry->registerReadbackUs += (uint64_t)(readEndUs - readStartUs);
        }
        if (err != ESP_OK) {
            sensorarrayAdsRegisterCacheInvalidate(&engine->registerCache);
            return err;
        }
    }
    bool snapshotMatches = sensorarrayAdsMatrixNoteRegisterSnapshot(
        &engine->registerCache, &snapshot);
    bool resetSeen = (snapshot.power & ADS126X_POWER_RESET) != 0u;
    if (!snapshotMatches || resetSeen ||
        snapshot.mode0 != (uint8_t)CONFIG_SENSORARRAY_ADS_MODE0_VALUE ||
        snapshot.mode1 != (uint8_t)CONFIG_SENSORARRAY_ADS_MODE1_VALUE) {
        sensorarrayAdsRegisterCacheInvalidate(&engine->registerCache);
        sensorarrayAdsProfileCacheInvalidate(&engine->profileCache);
        sensorarrayAdsValueCacheInvalidate(&engine->valueCache);
        return ESP_ERR_INVALID_RESPONSE;
    }
    engine->registerCache.vrefValid = engine->state->ads.vrefMicrovolts != 0u;
    engine->registerCache.vrefUv = (int32_t)engine->state->ads.vrefMicrovolts;
    engine->registerCache.pgaModeValid = true;
    engine->registerCache.inputMode = ads126xAdcMode2PgaBypassed(snapshot.mode2) ?
        SENSORARRAY_ADS_INPUT_BYPASS : SENSORARRAY_ADS_INPUT_PGA;
    uint8_t gain = 1u;
    if (!ads126xAdcMode2PgaBypassed(snapshot.mode2) &&
        !ads126xAdcMode2DecodePgaGain(snapshot.mode2, &gain)) {
        sensorarrayAdsRegisterCacheInvalidate(&engine->registerCache);
        return ESP_ERR_INVALID_RESPONSE;
    }
    engine->registerCache.gain = gain;
    engine->registerCache.adc1RunningValid = true;
    engine->registerCache.adc1Running = ads126xAdcIsAdc1Running(&engine->state->ads);
    return ESP_OK;
}

static esp_err_t sensorarrayAdsMatrixApplyInputProfile(
    sensorarrayAdsMatrixEngine_t *engine,
    uint8_t muxp,
    uint8_t muxn,
    sensorarrayAdsInputMode_t inputMode,
    uint8_t gain,
    sensorarrayMeasurementPayload_t *telemetry,
    uint32_t *outStartGeneration)
{
    if (!engine || !engine->state || !telemetry || !outStartGeneration) {
        return ESP_ERR_INVALID_ARG;
    }
    ads126xAdcHandle_t *ads = &engine->state->ads;
    uint8_t mode2 = 0u;
    if (!ads126xAdcBuildMode2(inputMode == SENSORARRAY_ADS_INPUT_BYPASS,
                              gain,
                              (uint8_t)CONFIG_SENSORARRAY_ADS_DATA_RATE,
                              &mode2)) {
        return ESP_ERR_INVALID_ARG;
    }
    if (sensorarrayAdsRegisterCacheNeedsWrite(&engine->registerCache,
                                              SENSORARRAY_ADS_REGISTER_MODE2,
                                              mode2)) {
        int64_t startUs = esp_timer_get_time();
        esp_err_t err = ads126xAdcSetMode2Fast(ads, mode2);
        int64_t endUs = esp_timer_get_time();
        if (endUs > startUs) {
            telemetry->registerWriteUs += (uint64_t)(endUs - startUs);
        }
        if (err != ESP_OK) {
            sensorarrayAdsRegisterCacheInvalidate(&engine->registerCache);
            return err;
        }
        sensorarrayAdsRegisterCacheNoteWrite(&engine->registerCache,
                                              SENSORARRAY_ADS_REGISTER_MODE2,
                                              mode2,
                                              false);
        engine->registerCache.pgaModeValid = true;
        engine->registerCache.inputMode = inputMode;
        engine->registerCache.gain = gain;
    }
    uint8_t inpmux = (uint8_t)(((muxp & 0x0Fu) << 4) | (muxn & 0x0Fu));
    if (sensorarrayAdsRegisterCacheNeedsWrite(&engine->registerCache,
                                              SENSORARRAY_ADS_REGISTER_INPMUX,
                                              inpmux)) {
        int64_t startUs = esp_timer_get_time();
        esp_err_t err = ads126xAdcSetInputMuxFast(ads, muxp, muxn);
        int64_t endUs = esp_timer_get_time();
        if (endUs > startUs) {
            uint64_t elapsedUs = (uint64_t)(endUs - startUs);
            telemetry->muxWriteUs += elapsedUs;
            telemetry->registerWriteUs += elapsedUs;
        }
        if (err != ESP_OK) {
            sensorarrayAdsRegisterCacheInvalidate(&engine->registerCache);
            return err;
        }
        sensorarrayAdsRegisterCacheNoteWrite(&engine->registerCache,
                                              SENSORARRAY_ADS_REGISTER_INPMUX,
                                              inpmux,
                                              false);
    }
    esp_err_t err = ESP_OK;
    if (!ads126xAdcIsAdc1Running(ads)) {
        ads126xAdcClearDrdyNotifications(ads);
        err = ads126xAdcStartAdc1(ads);
        if (err == ESP_OK) {
            engine->state->adsAdc1Running = true;
            engine->registerCache.adc1RunningValid = true;
            engine->registerCache.adc1Running = true;
        }
    }
    if (err == ESP_OK && CONFIG_SENSORARRAY_ADS_MATRIX_MUX_SETTLE_US > 0) {
        esp_rom_delay_us(CONFIG_SENSORARRAY_ADS_MATRIX_MUX_SETTLE_US);
    }
    /* Capture the generation only after every MODE2/INPMUX write and the
     * optional analogue guard. A DRDY edge that occurred while the old route
     * was being replaced must never satisfy the new cell's freshness test. */
    *outStartGeneration = ads126xAdcGetDrdyGeneration(ads);
    return err;
}

static uint32_t sensorarrayAdsMatrixRawSpread(const int32_t values[3])
{
    int32_t minimum = values[0];
    int32_t maximum = values[0];
    for (uint8_t index = 1u; index < 3u; ++index) {
        if (values[index] < minimum) {
            minimum = values[index];
        }
        if (values[index] > maximum) {
            maximum = values[index];
        }
    }
    int64_t spread = (int64_t)maximum - minimum;
    return spread > UINT32_MAX ? UINT32_MAX : (uint32_t)spread;
}

static esp_err_t sensorarrayAdsMatrixReadAdaptive(
    sensorarrayAdsMatrixEngine_t *engine,
    uint32_t firstGeneration,
    sensorarrayAdsCellValueCache_t *valueCache,
    bool profileValid,
    bool pgaBypassed,
    bool precisionFrame,
    bool transitionSensitive,
    sensorarrayMeasurementPayload_t *telemetry,
    sensorarrayAdsMatrixCellResult_t *outResult)
{
    if (!engine || !telemetry || !outResult) {
        return ESP_ERR_INVALID_ARG;
    }
    sensorarrayAdsMatrixSample_t samples[3] = {0};
    esp_err_t err = sensorarrayAdsMatrixReadFreshAfter(&engine->state->ads,
                                                        firstGeneration,
                                                        telemetry,
                                                        &samples[0]);
    if (err != ESP_OK) {
        return err;
    }
    outResult->raw = samples[0].raw;
    outResult->differentialUv = samples[0].differentialUv;
    outResult->status = samples[0].status;
    outResult->fresh = true;
    outResult->stable = false;
    outResult->fullSample = false;
    uint8_t statusErrorMask = (uint8_t)(ADS126X_STATUS_REFERENCE_ALARM |
        ADS126X_STATUS_RESET_OCCURRED |
        (pgaBypassed ? 0u : ADS126X_STATUS_PGA_ALARM_MASK));
    bool statusClean = (samples[0].status & statusErrorMask) == 0u;
    if (sensorarrayAdsValueCacheShouldFastAccept(
            valueCache,
            samples[0].raw,
            (uint32_t)CONFIG_SENSORARRAY_ADS_ADAPTIVE_MIN_RAW_THRESHOLD,
            (uint32_t)CONFIG_SENSORARRAY_ADS_ADAPTIVE_NOISE_MULTIPLIER,
            (uint32_t)CONFIG_SENSORARRAY_ADS_ADAPTIVE_MIN_STABLE_STREAK,
            profileValid,
            statusClean,
            true,
            precisionFrame,
            transitionSensitive)) {
        outResult->raw = samples[0].raw;
        outResult->differentialUv = samples[0].differentialUv;
        outResult->status = samples[0].status;
        outResult->fresh = true;
        outResult->stable = true;
        outResult->fullSample = false;
        return ESP_OK;
    }
    for (uint8_t index = 1u; index < 3u; ++index) {
        uint32_t generation = ads126xAdcGetDrdyGeneration(&engine->state->ads);
        err = sensorarrayAdsMatrixReadFreshAfter(&engine->state->ads,
                                                  generation,
                                                  telemetry,
                                                  &samples[index]);
        if (err != ESP_OK) {
            /* The first conversion remains genuinely fresh even if a stricter
             * second or third sample fails. Preserve that fact while the cell
             * itself is reported invalid. */
            return err;
        }
    }
    int64_t aggregationStartUs = esp_timer_get_time();
    int32_t rawValues[3] = {samples[0].raw, samples[1].raw, samples[2].raw};
    int32_t uvValues[3] = {samples[0].differentialUv,
                           samples[1].differentialUv,
                           samples[2].differentialUv};
    int32_t medianRaw = 0;
    int32_t medianUv = 0;
    (void)sensorarrayAdsMathSamplesStable(rawValues, 3u, UINT32_MAX, &medianRaw);
    bool stable = sensorarrayAdsMathSamplesStable(
        uvValues, 3u, (uint32_t)CONFIG_SENSORARRAY_ADS_MATRIX_MAX_SPREAD_UV, &medianUv);
    outResult->raw = medianRaw;
    outResult->differentialUv = medianUv;
    outResult->status = (uint8_t)(samples[0].status | samples[1].status |
                                  samples[2].status);
    outResult->fresh = true;
    outResult->stable = stable;
    outResult->fullSample = true;
    outResult->spreadRaw = sensorarrayAdsMatrixRawSpread(rawValues);
    int64_t aggregationEndUs = esp_timer_get_time();
    if (aggregationEndUs > aggregationStartUs) {
        telemetry->aggregationUs += (uint64_t)(aggregationEndUs - aggregationStartUs);
    }
    return ESP_OK;
}

static sensorarrayCellError_t sensorarrayAdsMatrixIoError(esp_err_t err)
{
    if (err == ESP_ERR_TIMEOUT) {
        return SENSORARRAY_CELL_ERROR_DRDY_TIMEOUT;
    }
    if (err == ESP_ERR_INVALID_RESPONSE) {
        return SENSORARRAY_CELL_ERROR_STALE;
    }
    return SENSORARRAY_CELL_ERROR_SPI;
}

static esp_err_t sensorarrayAdsMatrixMeasureCell(sensorarrayAdsMatrixEngine_t *engine,
                                                 const sensorarrayAdsRailSplit_t *rail,
                                                 uint8_t row,
                                                 uint8_t dLine,
                                                 bool precisionFrame,
                                                 bool transitionSensitive,
                                                 sensorarrayMeasurementPayload_t *telemetry,
                                                 sensorarrayAdsMatrixCellResult_t *outResult)
{
    if (!engine || !rail || !rail->valid || !telemetry || !outResult) {
        return ESP_ERR_INVALID_ARG;
    }
    memset(outResult, 0, sizeof(*outResult));
    outResult->error = SENSORARRAY_CELL_ERROR_AUTORANGE;
    size_t cellIndex = sensorarrayMatrixIndex(row, dLine);
    uint8_t muxp = 0u;
    uint8_t muxn = 0u;
    if (!sensorarrayBoardMapAdsMuxForDLine(dLine, &muxp, &muxn)) {
        outResult->error = SENSORARRAY_CELL_ERROR_ROUTE;
        return ESP_ERR_NOT_FOUND;
    }

    sensorarrayAdsCellProfile_t profile = {0};
    bool profileHit = sensorarrayAdsProfileCacheGet(&engine->profileCache,
                                                     engine->mode,
                                                     (uint8_t)cellIndex,
                                                     &profile);
    sensorarrayAdsInputMode_t inputMode = profileHit ? profile.inputMode :
        SENSORARRAY_ADS_INPUT_PGA;
    uint8_t gain = profileHit ? profile.gain : 1u;
    if (profileHit) {
        telemetry->profileHitCount++;
        if (inputMode == SENSORARRAY_ADS_INPUT_BYPASS) {
            telemetry->bypassHitCount++;
        } else {
            telemetry->gainHitCount++;
        }
    } else {
        telemetry->profileMissCount++;
    }
    sensorarrayAdsCellValueCache_t *valueCache = sensorarrayAdsValueCacheGet(
        &engine->valueCache, engine->mode, (uint8_t)cellIndex);
    uint8_t ioRetries = 0u;
    for (uint8_t attempt = 0u;
         attempt < (uint8_t)CONFIG_SENSORARRAY_ADS_AUTORANGE_MAX_ATTEMPTS;
         ++attempt) {
        outResult->attempts = (uint8_t)(attempt + 1u);
        sensorarrayAdsMatrixCellResult_t sample = {0};
        esp_err_t err = ESP_FAIL;
        for (;;) {
            uint32_t firstGeneration = 0u;
            err = sensorarrayAdsMatrixApplyInputProfile(engine,
                                                         muxp,
                                                         muxn,
                                                         inputMode,
                                                         gain,
                                                         telemetry,
                                                         &firstGeneration);
            if (err == ESP_OK) {
                err = sensorarrayAdsMatrixReadAdaptive(
                    engine,
                    firstGeneration,
                    valueCache,
                    profileHit && attempt == 0u,
                    inputMode == SENSORARRAY_ADS_INPUT_BYPASS,
                    precisionFrame,
                    transitionSensitive || attempt != 0u,
                    telemetry,
                    &sample);
            }
            bool retryable = err == ESP_ERR_TIMEOUT || err == ESP_ERR_INVALID_RESPONSE;
            if (err == ESP_OK || !retryable ||
                ioRetries >= (uint8_t)CONFIG_SENSORARRAY_ADS_MATRIX_IO_RETRY_COUNT) {
                break;
            }
            ioRetries++;
        }
        if (err != ESP_OK) {
            if (sample.fresh) {
                sample.gain = inputMode == SENSORARRAY_ADS_INPUT_BYPASS ? 0u : gain;
                sample.pgaBypassed = inputMode == SENSORARRAY_ADS_INPUT_BYPASS;
                sample.attempts = outResult->attempts;
                sample.gainChanges = outResult->gainChanges;
                sample.ioRetries = ioRetries;
                *outResult = sample;
            }
            outResult->ioRetries = ioRetries;
            outResult->error = sensorarrayAdsMatrixIoError(err);
            sensorarrayAdsProfileCacheNoteFailure(
                &engine->profileCache,
                engine->mode,
                (uint8_t)cellIndex,
                false,
                (uint8_t)CONFIG_SENSORARRAY_ADS_PROFILE_FAILURE_INVALIDATE_COUNT);
            return err;
        }
        sample.gain = inputMode == SENSORARRAY_ADS_INPUT_BYPASS ? 0u : gain;
        sample.pgaBypassed = inputMode == SENSORARRAY_ADS_INPUT_BYPASS;
        sample.attempts = outResult->attempts;
        sample.gainChanges = outResult->gainChanges;
        sample.ioRetries = ioRetries;
        *outResult = sample;

        if ((sample.status & ADS126X_STATUS_RESET_OCCURRED) != 0u) {
            /* A reset invalidates register, reference, calibration and PGA
             * assumptions even when the conversion carries a data-ready bit. */
            outResult->error = SENSORARRAY_CELL_ERROR_READBACK;
            sensorarrayAdsMatrixEngineInvalidateCaches(engine);
            return ESP_ERR_INVALID_RESPONSE;
        }

        int64_t nodeUv64 = (int64_t)sample.differentialUv + rail->aincomUv;
        if (nodeUv64 < INT32_MIN || nodeUv64 > INT32_MAX) {
            outResult->error = SENSORARRAY_CELL_ERROR_OVERFLOW;
            return ESP_ERR_INVALID_SIZE;
        }
        outResult->nodeUv = (int32_t)nodeUv64;
        uint64_t magnitude = sample.raw == INT32_MIN ?
            (uint64_t)INT32_MAX + 1u :
            (uint64_t)(sample.raw < 0 ? -sample.raw : sample.raw);
        uint64_t saturationLimit =
            ((uint64_t)INT32_MAX * CONFIG_SENSORARRAY_ADS_AUTORANGE_SATURATION_PERMILLE) /
            1000u;
        if (inputMode == SENSORARRAY_ADS_INPUT_BYPASS) {
            int64_t marginUv = CONFIG_SENSORARRAY_ADS_BYPASS_INPUT_MARGIN_UV;
            int64_t lowerUv = (int64_t)rail->avssUv - marginUv;
            int64_t upperUv = (int64_t)rail->avddUv + marginUv;
            bool inputsSafe = nodeUv64 >= lowerUv && nodeUv64 <= upperUv &&
                              (int64_t)rail->aincomUv >= lowerUv &&
                              (int64_t)rail->aincomUv <= upperUv;
            if ((sample.status & ADS126X_STATUS_REFERENCE_ALARM) != 0u) {
                outResult->error = SENSORARRAY_CELL_ERROR_REFERENCE_ALARM;
                sensorarrayAdsProfileCacheNoteFailure(
                    &engine->profileCache, engine->mode, (uint8_t)cellIndex, false,
                    (uint8_t)CONFIG_SENSORARRAY_ADS_PROFILE_FAILURE_INVALIDATE_COUNT);
                return ESP_ERR_INVALID_RESPONSE;
            }
            if (!inputsSafe || magnitude >= saturationLimit) {
                /* In the confirmed resistance divider, an open/high-Z cell
                 * drives Vnode towards AVSS.  That can exceed the negative
                 * ADC full-scale limit even with the PGA bypassed.  Preserve
                 * the invalid sentinel, but report the physically useful
                 * OPEN classification instead of a generic saturation. */
                if (engine->mode == SENSORARRAY_MEASUREMENT_MODE_RESISTANCE &&
                    sample.raw == INT32_MIN) {
                    outResult->error = SENSORARRAY_CELL_ERROR_OPEN;
                    (void)sensorarrayAdsProfileCacheStore(
                        &engine->profileCache, engine->mode, (uint8_t)cellIndex,
                        SENSORARRAY_ADS_INPUT_BYPASS, 1u, engine->frameCount);
                    sensorarrayAdsValueCacheObserve(valueCache,
                                                     sample.raw,
                                                     outResult->nodeUv,
                                                     SENSORARRAY_MEASUREMENT_INVALID_FIXED,
                                                     sample.spreadRaw,
                                                     engine->frameCount,
                                                     sample.fullSample);
                } else {
                    outResult->error = inputsSafe ? SENSORARRAY_CELL_ERROR_SATURATED :
                                                    SENSORARRAY_CELL_ERROR_RANGE;
                    sensorarrayAdsProfileCacheNoteFailure(
                        &engine->profileCache, engine->mode, (uint8_t)cellIndex, true,
                        (uint8_t)CONFIG_SENSORARRAY_ADS_PROFILE_FAILURE_INVALIDATE_COUNT);
                }
                return ESP_ERR_INVALID_RESPONSE;
            }
            if (!outResult->stable) {
                sensorarrayAdsValueCacheObserve(valueCache,
                                                 sample.raw,
                                                 outResult->nodeUv,
                                                 SENSORARRAY_MEASUREMENT_INVALID_FIXED,
                                                 sample.spreadRaw,
                                                 engine->frameCount,
                                                 sample.fullSample);
                if (valueCache) {
                    valueCache->stableStreak = 0u;
                    if (valueCache->unstableStreak < UINT32_MAX) {
                        valueCache->unstableStreak++;
                    }
                }
                sensorarrayAdsProfileCacheNoteFailure(
                    &engine->profileCache, engine->mode, (uint8_t)cellIndex, false,
                    (uint8_t)CONFIG_SENSORARRAY_ADS_PROFILE_FAILURE_INVALIDATE_COUNT);
                outResult->error = SENSORARRAY_CELL_ERROR_UNSTABLE;
                return ESP_ERR_INVALID_RESPONSE;
            }
            outResult->error = SENSORARRAY_CELL_ERROR_NONE;
            if (!profileHit || profile.inputMode != SENSORARRAY_ADS_INPUT_BYPASS) {
                (void)sensorarrayAdsProfileCacheStore(
                    &engine->profileCache, engine->mode, (uint8_t)cellIndex,
                    SENSORARRAY_ADS_INPUT_BYPASS, 1u, engine->frameCount);
            } else {
                sensorarrayAdsProfileCacheNoteSuccess(
                    &engine->profileCache, engine->mode, (uint8_t)cellIndex,
                    engine->frameCount);
            }
            sensorarrayAdsValueCacheObserve(valueCache,
                                             sample.raw,
                                             outResult->nodeUv,
                                             SENSORARRAY_MEASUREMENT_INVALID_FIXED,
                                             sample.spreadRaw,
                                             engine->frameCount,
                                             sample.fullSample);
            return ESP_OK;
        }
        sensorarrayAdsAutoRangeInput_t rangeInput = {
            .rawCode = sample.raw,
            .positiveInputUv = outResult->nodeUv,
            .negativeInputUv = rail->aincomUv,
            .avddUv = rail->avddUv,
            .avssUv = rail->avssUv,
            .currentGain = gain,
            .attempt = attempt,
            .referenceAlarm = (sample.status & ADS126X_STATUS_REFERENCE_ALARM) != 0u,
            .pgaLowAlarm = (sample.status & ADS126X_STATUS_PGA_LOW_ALARM) != 0u,
            .pgaHighAlarm = (sample.status & ADS126X_STATUS_PGA_HIGH_ALARM) != 0u,
            .pgaDifferentialAlarm =
                (sample.status & ADS126X_STATUS_PGA_DIFFERENTIAL_ALARM) != 0u,
            .allowIncrease = true,
        };
        bool statusAlarm = (sample.status & (ADS126X_STATUS_REFERENCE_ALARM |
                                             ADS126X_STATUS_PGA_ALARM_MASK)) != 0u;
        bool commonModeSafe = sensorarrayAdsAutoRangeCommonModeSafe(
            outResult->nodeUv,
            rail->aincomUv,
            rail->avddUv,
            rail->avssUv,
            gain,
            s_sensorarrayAdsAutoRangeConfig.pgaRailMarginUv);
        bool shouldAutorange = !profileHit || precisionFrame || statusAlarm ||
                               !commonModeSafe || magnitude >= saturationLimit;
        sensorarrayAdsAutoRangeDecision_t decision = {
            .action = SENSORARRAY_ADS_AUTORANGE_KEEP,
            .error = SENSORARRAY_CELL_ERROR_NONE,
            .nextGain = gain,
            .commonModeSafe = commonModeSafe,
        };
        if (shouldAutorange) {
            int64_t autorangeStartUs = esp_timer_get_time();
            decision = sensorarrayAdsAutoRangeDecide(&s_sensorarrayAdsAutoRangeConfig,
                                                      &rangeInput);
            int64_t autorangeEndUs = esp_timer_get_time();
            if (autorangeEndUs > autorangeStartUs) {
                telemetry->autorangeUs += (uint64_t)(autorangeEndUs - autorangeStartUs);
            }
        }
        if (magnitude >= saturationLimit && decision.action == SENSORARRAY_ADS_AUTORANGE_KEEP) {
            decision.action = SENSORARRAY_ADS_AUTORANGE_FAIL;
            decision.error = SENSORARRAY_CELL_ERROR_SATURATED;
        }
        if (decision.action == SENSORARRAY_ADS_AUTORANGE_INCREASE ||
            decision.action == SENSORARRAY_ADS_AUTORANGE_DECREASE) {
            gain = decision.nextGain;
            inputMode = SENSORARRAY_ADS_INPUT_PGA;
            profileHit = false;
            outResult->gainChanges++;
            continue;
        }
        if (decision.action == SENSORARRAY_ADS_AUTORANGE_FAIL && gain == 1u &&
            (decision.error == SENSORARRAY_CELL_ERROR_PGA_ABSOLUTE ||
             decision.error == SENSORARRAY_CELL_ERROR_PGA_DIFFERENTIAL ||
             decision.error == SENSORARRAY_CELL_ERROR_COMMON_MODE)) {
            /* PGA bypass is the bounded lowest-risk fallback for inputs near
             * AVDD/AVSS. MODE2 readback, a new settle/discard cycle and fresh
             * status are required on the next attempt. Telemetry gain 0 means
             * bypass, not an unsupported programmable gain. */
            inputMode = SENSORARRAY_ADS_INPUT_BYPASS;
            gain = 1u;
            profileHit = false;
            outResult->gainChanges++;
            continue;
        }
        if (decision.action == SENSORARRAY_ADS_AUTORANGE_FAIL) {
            outResult->error = decision.error;
            sensorarrayAdsProfileCacheNoteFailure(
                &engine->profileCache, engine->mode, (uint8_t)cellIndex, true,
                (uint8_t)CONFIG_SENSORARRAY_ADS_PROFILE_FAILURE_INVALIDATE_COUNT);
            return ESP_ERR_INVALID_RESPONSE;
        }
        if (!outResult->stable) {
            sensorarrayAdsValueCacheObserve(valueCache,
                                             sample.raw,
                                             outResult->nodeUv,
                                             SENSORARRAY_MEASUREMENT_INVALID_FIXED,
                                             sample.spreadRaw,
                                             engine->frameCount,
                                             sample.fullSample);
            sensorarrayAdsProfileCacheNoteFailure(
                &engine->profileCache, engine->mode, (uint8_t)cellIndex, false,
                (uint8_t)CONFIG_SENSORARRAY_ADS_PROFILE_FAILURE_INVALIDATE_COUNT);
            outResult->error = SENSORARRAY_CELL_ERROR_UNSTABLE;
            return ESP_ERR_INVALID_RESPONSE;
        }
        outResult->error = SENSORARRAY_CELL_ERROR_NONE;
        sensorarrayAdsCellProfile_t currentProfile = {0};
        if (!sensorarrayAdsProfileCacheGet(&engine->profileCache,
                                           engine->mode,
                                           (uint8_t)cellIndex,
                                           &currentProfile) ||
            currentProfile.inputMode != SENSORARRAY_ADS_INPUT_PGA ||
            currentProfile.gain != gain) {
            (void)sensorarrayAdsProfileCacheStore(
                &engine->profileCache, engine->mode, (uint8_t)cellIndex,
                SENSORARRAY_ADS_INPUT_PGA, gain, engine->frameCount);
        } else {
            sensorarrayAdsProfileCacheNoteSuccess(
                &engine->profileCache, engine->mode, (uint8_t)cellIndex,
                engine->frameCount);
        }
        sensorarrayAdsValueCacheObserve(valueCache,
                                         sample.raw,
                                         outResult->nodeUv,
                                         SENSORARRAY_MEASUREMENT_INVALID_FIXED,
                                         sample.spreadRaw,
                                         engine->frameCount,
                                         sample.fullSample);
        return ESP_OK;
    }
    outResult->error = SENSORARRAY_CELL_ERROR_AUTORANGE;
    return ESP_ERR_TIMEOUT;
}

static bool sensorarrayAdsMatrixGetRail(sensorarrayAdsMatrixEngine_t *engine,
                                        sensorarrayAdsRailSplit_t *outRail,
                                        sensorarrayAdsGapSnapshot_t *outGap)
{
    uint32_t sequence = engine->frameSequenceHint ?
        engine->frameSequenceHint : (engine->frameCount + 1u);
    sensorarrayAdsGapSnapshot_t gap = {0};
    sensorarrayAdsGapCopySnapshot(&gap, sequence);
    bool valid = sensorarrayAdsGapCopyRailSplit(
        sequence,
        (uint32_t)CONFIG_SENSORARRAY_ADS_MATRIX_RAIL_MAX_AGE_FRAMES,
        outRail);
    if (!valid) {
        if (sensorarrayAdsGapRefreshRailAtBoundary(engine->state, sequence) != ESP_OK) {
            if (outGap) {
                *outGap = gap;
            }
            return false;
        }
        sensorarrayAdsGapCopySnapshot(&gap, sequence);
        valid = sensorarrayAdsGapCopyRailSplit(
            sequence,
            (uint32_t)CONFIG_SENSORARRAY_ADS_MATRIX_RAIL_MAX_AGE_FRAMES,
            outRail);
    }
    if (outGap) {
        *outGap = gap;
    }
    return valid;
}

static void sensorarrayAdsMatrixFinishFrame(sensorarrayFrame_t *frame,
                                            uint8_t rows,
                                            int64_t startUs,
                                            bool coherent)
{
    uint64_t activeMask = sensorarrayAdsMatrixActiveMask(rows);
    frame->measurement.errorMask &= activeMask;
    frame->errorMask &= activeMask;
    frame->stale = (frame->measurement.freshMask & activeMask) != activeMask;
    /* A cell may be explicitly invalid while the frame is still coherent.
     * Route/rail failures are different: the analogue ownership contract was
     * lost, so no partially assembled frame may be published. */
    frame->freshFrame = coherent && !frame->stale;
    frame->frameEndUs = (uint64_t)esp_timer_get_time();
    frame->physicalSweepUs = frame->frameEndUs - frame->frameStartUs;
    frame->measurement.frameDurationUs = frame->physicalSweepUs;
    frame->rowFreshMask = 0u;
    for (uint8_t row = 0u; row < rows; ++row) {
        uint64_t rowMask = UINT64_C(0xFF) << (row * SENSORARRAY_MATRIX_COLS);
        if ((frame->measurement.freshMask & rowMask) == rowMask) {
            frame->rowFreshMask |= (uint8_t)(1u << row);
        }
    }
    frame->primaryFreshMask = frame->rowFreshMask;
    frame->secondaryFreshMask = frame->rowFreshMask;
    if (startUs > 0 && (int64_t)frame->frameEndUs > startUs) {
        frame->measurement.frameDurationUs = (uint64_t)((int64_t)frame->frameEndUs - startUs);
    }
}

esp_err_t sensorarrayAdsMatrixEngineInit(sensorarrayAdsMatrixEngine_t *engine,
                                         sensorarrayState_t *state)
{
    if (!engine || !state) {
        return ESP_ERR_INVALID_ARG;
    }
    *engine = (sensorarrayAdsMatrixEngine_t){
        .state = state,
        .mode = SENSORARRAY_MEASUREMENT_MODE_NONE,
        .calibration = {
            .version = SENSORARRAY_ADS_MATRIX_CALIBRATION_VERSION,
            .referenceResistorOhms = CONFIG_SENSORARRAY_ADS_MATRIX_RREF_OHMS,
            .matrixReferenceSpanUv = CONFIG_SENSORARRAY_ADS_MATRIX_INTERNAL_REF_UV,
            .globalPathOffsetMilliohms =
                CONFIG_SENSORARRAY_ADS_MATRIX_PATH_OFFSET_MOHM,
        },
    };
    sensorarrayAdsProfileCacheInit(&engine->profileCache);
    sensorarrayAdsValueCacheInit(&engine->valueCache);
    sensorarrayAdsRegisterCacheInit(&engine->registerCache);
    sensorarrayAdsRailFingerprintInit(&engine->railFingerprint);
    engine->calibrationGeneration = 1u;
    engine->transitionSensitiveFrames = 1u;
    return ESP_OK;
}

esp_err_t sensorarrayAdsMatrixEngineSetCalibration(
    sensorarrayAdsMatrixEngine_t *engine,
    const sensorarrayAdsMatrixCalibration_t *calibration)
{
    /* Core 1 ownership is required. Calibration cannot change underneath an
     * active frame; callers first transition to SAFE. Persistence is left to
     * a future versioned/CRC-backed store rather than an ad-hoc NVS blob. */
    if (!engine || !sensorarrayAdsMatrixCalibrationValid(calibration)) {
        return ESP_ERR_INVALID_ARG;
    }
    if (engine->mode != SENSORARRAY_MEASUREMENT_MODE_NONE) {
        return ESP_ERR_INVALID_STATE;
    }
    engine->calibration = *calibration;
    engine->calibrationGeneration++;
    sensorarrayAdsProfileCacheInvalidate(&engine->profileCache);
    sensorarrayAdsValueCacheInvalidate(&engine->valueCache);
    sensorarrayAdsRailFingerprintInit(&engine->railFingerprint);
    return ESP_OK;
}

bool sensorarrayAdsMatrixEngineGetCalibration(
    const sensorarrayAdsMatrixEngine_t *engine,
    sensorarrayAdsMatrixCalibration_t *outCalibration)
{
    if (!engine || !outCalibration) {
        return false;
    }
    *outCalibration = engine->calibration;
    return sensorarrayAdsMatrixCalibrationValid(outCalibration);
}

void sensorarrayAdsMatrixEngineBindRouteController(
    sensorarrayAdsMatrixEngine_t *engine,
    sensorarrayRouteController_t *routeController)
{
    if (engine) {
        engine->routeController = routeController;
    }
}

esp_err_t sensorarrayAdsMatrixEngineSetMode(sensorarrayAdsMatrixEngine_t *engine,
                                           sensorarrayMeasurementMode_t mode)
{
    if (!engine || (mode != SENSORARRAY_MEASUREMENT_MODE_VOLTAGE &&
                    mode != SENSORARRAY_MEASUREMENT_MODE_RESISTANCE &&
                    mode != SENSORARRAY_MEASUREMENT_MODE_NONE)) {
        return ESP_ERR_INVALID_ARG;
    }
    if (engine->mode != mode) {
        engine->mode = mode;
        sensorarrayAdsProfileCacheInvalidate(&engine->profileCache);
        sensorarrayAdsValueCacheInvalidate(&engine->valueCache);
        sensorarrayAdsRegisterCacheInvalidate(&engine->registerCache);
        sensorarrayAdsRailFingerprintInit(&engine->railFingerprint);
        engine->transitionSensitiveFrames = 1u;
    }
    return ESP_OK;
}

void sensorarrayAdsMatrixEngineSetFrameSequenceHint(sensorarrayAdsMatrixEngine_t *engine,
                                                    uint32_t sequence)
{
    if (engine) {
        engine->frameSequenceHint = sequence;
    }
}

void sensorarrayAdsMatrixEngineInvalidateGainCache(sensorarrayAdsMatrixEngine_t *engine)
{
    if (engine) {
        sensorarrayAdsProfileCacheInvalidate(&engine->profileCache);
        sensorarrayAdsValueCacheInvalidate(&engine->valueCache);
        sensorarrayAdsRailFingerprintInit(&engine->railFingerprint);
    }
}

void sensorarrayAdsMatrixEngineInvalidateCaches(sensorarrayAdsMatrixEngine_t *engine)
{
    if (!engine) {
        return;
    }
    sensorarrayAdsProfileCacheInvalidate(&engine->profileCache);
    sensorarrayAdsValueCacheInvalidate(&engine->valueCache);
    sensorarrayAdsRegisterCacheInvalidate(&engine->registerCache);
    sensorarrayAdsRailFingerprintInit(&engine->railFingerprint);
    engine->transitionSensitiveFrames = 1u;
}

sensorarrayAdsRegisterCache_t *sensorarrayAdsMatrixEngineRegisterCache(
    sensorarrayAdsMatrixEngine_t *engine)
{
    return engine ? &engine->registerCache : NULL;
}

esp_err_t sensorarrayAdsMatrixEngineReadFrame(sensorarrayAdsMatrixEngine_t *engine,
                                              const sensorarrayScanPlan_t *plan,
                                              sensorarrayFrame_t *frame)
{
    if (!engine || !engine->state || !engine->routeController || !plan || !frame ||
        (engine->mode != SENSORARRAY_MEASUREMENT_MODE_VOLTAGE &&
         engine->mode != SENSORARRAY_MEASUREMENT_MODE_RESISTANCE)) {
        return ESP_ERR_INVALID_STATE;
    }
    sensorarrayRouteSnapshot_t route;
    if (!sensorarrayRouteControllerCopySnapshot(engine->routeController, &route) ||
        route.safe || route.mode != engine->mode || !route.gpioReadbackValid ||
        !route.adsReadbackValid) {
        sensorarrayFrameBuilderInitInvalid(frame);
        return ESP_ERR_INVALID_STATE;
    }

    sensorarrayFrameBuilderInitInvalid(frame);
    int64_t frameStartUs = esp_timer_get_time();
    frame->timestampUs = (uint64_t)frameStartUs;
    frame->frameStartUs = frame->timestampUs;
    frame->sequence = engine->frameSequenceHint;
    frame->physicalSweepId = ++engine->frameCount;
    frame->configSnapshot = plan->configSnapshot;
    frame->activeRows = plan->rowCount >= 1u && plan->rowCount <= 8u ?
        plan->rowCount : 8u;
    frame->measurement.mode = engine->mode;
    frame->measurement.unit = engine->mode == SENSORARRAY_MEASUREMENT_MODE_VOLTAGE ?
        SENSORARRAY_MEASUREMENT_UNIT_VOLT : SENSORARRAY_MEASUREMENT_UNIT_OHM;
    frame->measurement.decimalScale = engine->mode == SENSORARRAY_MEASUREMENT_MODE_VOLTAGE ?
        -6 : -3;
    frame->measurement.referenceSource = route.profile.adsReferenceSource;
    frame->measurement.referenceResistorOhms =
        engine->mode == SENSORARRAY_MEASUREMENT_MODE_RESISTANCE ?
            engine->calibration.referenceResistorOhms : 0u;
    frame->measurement.transitionDurationUs = route.transitionDurationUs;
    uint32_t registerHitStart = engine->registerCache.cacheHitCount;
    uint32_t registerWriteStart = engine->registerCache.writeCount;
    uint32_t registerReadbackStart = engine->registerCache.readbackCount;
    uint32_t profileInvalidationStart = engine->profileCache.invalidationCount;
    uint32_t railHitStart = engine->railFingerprint.hitCount;
    uint32_t railMissStart = engine->railFingerprint.missCount;
    uint32_t railInvalidationStart = engine->railFingerprint.invalidationCount;
    frame->measurement.precisionFrame =
        CONFIG_SENSORARRAY_ADS_PRECISION_FRAME_INTERVAL > 0 &&
        (engine->frameCount % CONFIG_SENSORARRAY_ADS_PRECISION_FRAME_INTERVAL) == 0u;

    sensorarrayAdsRailSplit_t rail = {0};
    if (!sensorarrayAdsMatrixGetRail(engine, &rail, &frame->adsGap)) {
        sensorarrayAdsProfileCacheInvalidate(&engine->profileCache);
        sensorarrayAdsValueCacheInvalidate(&engine->valueCache);
        sensorarrayAdsRailFingerprintInit(&engine->railFingerprint);
        for (size_t index = 0u;
             index < (size_t)frame->activeRows * SENSORARRAY_MATRIX_COLS;
             ++index) {
            sensorarrayAdsMatrixSetCellError(frame,
                                             index,
                                             SENSORARRAY_CELL_ERROR_RAIL_INVALID,
                                             false);
        }
        (void)sensorarrayRouteControllerEnterSafe(engine->routeController,
                                                  "rail_invalid");
        sensorarrayAdsMatrixFinishFrame(frame, frame->activeRows, frameStartUs, false);
        return ESP_ERR_INVALID_STATE;
    }
    /* Monitor noise changes the measured rail by small amounts every sample.
     * Use a thresholded fingerprint; only external calibration uses its exact
     * AVDD/AVSS split as an identity field. */
    int32_t fingerprintAvdd = frame->adsGap.railSource ==
        SENSORARRAY_ADS_RAIL_SOURCE_EXTERNAL_CALIBRATION ?
        rail.avddUv : CONFIG_SENSORARRAY_ADS_AVDD_TO_GND_UV;
    int32_t fingerprintAvss = frame->adsGap.railSource ==
        SENSORARRAY_ADS_RAIL_SOURCE_EXTERNAL_CALIBRATION ?
        rail.avssUv : -CONFIG_SENSORARRAY_ADS_AVSS_TO_GND_UV;
    sensorarrayAdsRailInvalidationReason_t railReason =
        SENSORARRAY_ADS_RAIL_INVALIDATION_NONE;
    bool railInvalidated = sensorarrayAdsRailFingerprintUpdate(
        &engine->railFingerprint,
        frame->adsGap.railValid,
        frame->adsGap.railStatus != SENSORARRAY_ADS_RAIL_STATUS_BAD,
        frame->adsGap.railUv,
        fingerprintAvdd,
        fingerprintAvss,
        frame->adsGap.railSource,
        (uint8_t)route.profile.adsReferenceSource,
        engine->calibrationGeneration,
        (uint32_t)CONFIG_SENSORARRAY_ADS_PROFILE_RAIL_INVALIDATE_UV,
        (uint32_t)CONFIG_SENSORARRAY_ADS_PROFILE_RAIL_HYSTERESIS_UV,
        false,
        &railReason);
    if (railInvalidated) {
        sensorarrayAdsProfileCacheInvalidate(&engine->profileCache);
        sensorarrayAdsValueCacheInvalidate(&engine->valueCache);
    }
    frame->measurement.railInvalidationReason = (uint8_t)railReason;
    sensorarrayRouteControllerUpdateRailSnapshot(engine->routeController, &rail);
    frame->measurement.avddUv = rail.avddUv;
    frame->measurement.avssUv = rail.avssUv;
    frame->measurement.railAgeFrames = rail.ageFrames;
    frame->measurement.railValid = rail.valid;
    frame->measurement.referenceValid = true;
    frame->measurement.matrixReferenceUv =
        engine->mode == SENSORARRAY_MEASUREMENT_MODE_RESISTANCE ?
        rail.avssUv + (int32_t)engine->calibration.matrixReferenceSpanUv : 0;

    if (!sensorarrayAdsRegisterCacheAcquire(&engine->registerCache,
                                             SENSORARRAY_ADS_OWNER_MATRIX)) {
        (void)sensorarrayRouteControllerEnterSafe(engine->routeController,
                                                  "ads_owner_collision");
        sensorarrayAdsMatrixFinishFrame(frame, frame->activeRows, frameStartUs, false);
        return ESP_ERR_INVALID_STATE;
    }
    bool healthReadbackDue =
        !engine->registerCache.registers[SENSORARRAY_ADS_REGISTER_MODE0].valid ||
        (engine->frameCount % CONFIG_SENSORARRAY_ADS_REGISTER_HEALTH_INTERVAL) == 0u;
    if (healthReadbackDue &&
        sensorarrayAdsMatrixRefreshRegisterShadow(engine, &frame->measurement) != ESP_OK) {
        (void)sensorarrayAdsRegisterCacheRelease(&engine->registerCache,
                                                  SENSORARRAY_ADS_OWNER_MATRIX);
        for (size_t index = 0u;
             index < (size_t)frame->activeRows * SENSORARRAY_MATRIX_COLS;
             ++index) {
            sensorarrayAdsMatrixSetCellError(frame,
                                             index,
                                             SENSORARRAY_CELL_ERROR_READBACK,
                                             false);
        }
        (void)sensorarrayRouteControllerEnterSafe(engine->routeController,
                                                  "ads_register_health");
        sensorarrayAdsMatrixFinishFrame(frame, frame->activeRows, frameStartUs, false);
        return ESP_ERR_INVALID_RESPONSE;
    }

    esp_err_t firstErr = ESP_OK;
    bool coherent = true;
    bool transitionSensitive = engine->transitionSensitiveFrames != 0u || railInvalidated;
    for (uint8_t row = 1u; row <= frame->activeRows; ++row) {
        int64_t rowStartUs = esp_timer_get_time();
        esp_err_t rowErr = sensorarrayRouteControllerSelectRow(engine->routeController, row);
        int64_t rowEndUs = esp_timer_get_time();
        if (rowEndUs > rowStartUs) {
            frame->measurement.rowRouteUs += (uint64_t)(rowEndUs - rowStartUs);
        }
        if (rowErr != ESP_OK) {
            for (uint8_t dLine = 1u; dLine <= SENSORARRAY_MATRIX_COLS; ++dLine) {
                sensorarrayAdsMatrixSetCellError(frame,
                                                 sensorarrayMatrixIndex(row, dLine),
                                                 SENSORARRAY_CELL_ERROR_ROUTE,
                                                 false);
            }
            firstErr = rowErr;
            coherent = false;
            break;
        }
        if (!engine->state->adsAdc1Running) {
            engine->registerCache.adc1RunningValid = true;
            engine->registerCache.adc1Running = false;
        }
        for (uint8_t dLine = 1u; dLine <= SENSORARRAY_MATRIX_COLS; ++dLine) {
            size_t index = sensorarrayMatrixIndex(row, dLine);
            sensorarrayAdsMatrixCellResult_t result = {0};
            esp_err_t err = sensorarrayAdsMatrixMeasureCell(engine,
                                                             &rail,
                                                             row,
                                                             dLine,
                                                             frame->measurement.precisionFrame,
                                                             transitionSensitive,
                                                             &frame->measurement,
                                                             &result);
            frame->measurement.rawCode[index] = result.raw;
            frame->measurement.nodeUv[index] = result.nodeUv;
            frame->measurement.pgaGain[index] = result.gain;
            frame->measurement.gainChangeCount += result.gainChanges;
            frame->measurement.autorangeAttemptCount += result.attempts;
            frame->measurement.ioRetryCount += result.ioRetries;
            if (result.fresh) {
                if (result.fullSample) {
                    frame->measurement.tripleSampleCellCount++;
                } else {
                    frame->measurement.singleSampleCellCount++;
                }
            }
            if (!result.pgaBypassed &&
                (result.status & ADS126X_STATUS_PGA_ALARM_MASK) != 0u) {
                frame->measurement.overrangeCount++;
            }
            uint8_t statusErrorMask = (uint8_t)(
                ADS126X_STATUS_REFERENCE_ALARM | ADS126X_STATUS_RESET_OCCURRED |
                (result.pgaBypassed ? 0u : ADS126X_STATUS_PGA_ALARM_MASK));
            if ((result.status & statusErrorMask) != 0u) {
                frame->measurement.statusErrorCount++;
            }
            if (result.pgaBypassed) {
                frame->measurement.autorangeFallbackCount++;
            }
            if (err != ESP_OK) {
                sensorarrayCellError_t error = result.error;
                if (error == SENSORARRAY_CELL_ERROR_NONE) {
                    error = sensorarrayAdsMatrixIoError(err);
                }
                if (error == SENSORARRAY_CELL_ERROR_DRDY_TIMEOUT) {
                    frame->measurement.drdyTimeoutCount++;
                } else if (error == SENSORARRAY_CELL_ERROR_STALE) {
                    frame->measurement.staleCount++;
                } else if (error == SENSORARRAY_CELL_ERROR_SPI) {
                    frame->measurement.spiErrorCount++;
                }
                sensorarrayAdsMatrixSetCellError(frame, index, error, result.fresh);
                if (firstErr == ESP_OK) {
                    firstErr = err;
                }
                continue;
            }

            if (engine->mode == SENSORARRAY_MEASUREMENT_MODE_VOLTAGE) {
                int32_t nodeUv = 0;
                sensorarrayCellError_t voltageError = SENSORARRAY_CELL_ERROR_NONE;
                if (!sensorarrayAdsMathVoltageFromDifferential(result.differentialUv,
                                                                &rail,
                                                                &s_sensorarrayAdsVoltageLimits,
                                                                &nodeUv,
                                                                &voltageError)) {
                    sensorarrayAdsMatrixSetCellError(frame, index, voltageError, true);
                    if (firstErr == ESP_OK) {
                        firstErr = ESP_ERR_INVALID_RESPONSE;
                    }
                    continue;
                }
                result.nodeUv = nodeUv;
                sensorarrayAdsCellValueCache_t *valueCache = sensorarrayAdsValueCacheGet(
                    &engine->valueCache, engine->mode, (uint8_t)index);
                if (valueCache) {
                    valueCache->lastNodeUv = nodeUv;
                    valueCache->lastValueFixed = nodeUv;
                }
                sensorarrayAdsMatrixSetCellValid(frame, index, nodeUv, &result);
            } else {
                sensorarrayAdsResistanceConfig_t resistanceConfig =
                    s_sensorarrayAdsResistanceLimits;
                resistanceConfig.referenceResistorOhms =
                    engine->calibration.referenceResistorOhms;
                int64_t pathOffset = engine->calibration.globalPathOffsetMilliohms +
                    engine->calibration.bankOffsetMilliohms[dLine > 4u ? 1u : 0u];
                if ((engine->calibration.cellOffsetValidMask &
                     (UINT64_C(1) << index)) != 0u) {
                    pathOffset += engine->calibration.cellOffsetMilliohms[index];
                }
                resistanceConfig.pathOffsetMilliohms = pathOffset;
                sensorarrayAdsResistanceResult_t resistance =
                    sensorarrayAdsMathResistanceDivider(
                        frame->measurement.matrixReferenceUv,
                        result.nodeUv,
                        rail.avssUv,
                        &resistanceConfig);
                if (!resistance.valid) {
                    sensorarrayAdsMatrixSetCellError(frame,
                                                     index,
                                                     resistance.error,
                                                     true);
                    if (firstErr == ESP_OK) {
                        firstErr = ESP_ERR_INVALID_RESPONSE;
                    }
                    continue;
                }
                sensorarrayAdsCellValueCache_t *valueCache = sensorarrayAdsValueCacheGet(
                    &engine->valueCache, engine->mode, (uint8_t)index);
                if (valueCache) {
                    valueCache->lastValueFixed = resistance.resistanceMilliohms;
                }
                sensorarrayAdsMatrixSetCellValid(frame,
                                                 index,
                                                 resistance.resistanceMilliohms,
                                                 &result);
            }
        }
    }
    (void)sensorarrayAdsRegisterCacheRelease(&engine->registerCache,
                                              SENSORARRAY_ADS_OWNER_MATRIX);
    if (engine->transitionSensitiveFrames > 0u) {
        engine->transitionSensitiveFrames--;
    }
    if (!coherent) {
        frame->measurement.validMask = 0u;
        frame->validCount = 0u;
        for (size_t index = 0u;
             index < (size_t)frame->activeRows * SENSORARRAY_MATRIX_COLS;
             ++index) {
            frame->measurement.valuesFixed[index] =
                SENSORARRAY_MEASUREMENT_INVALID_FIXED;
        }
    }
    frame->measurement.freshCellCount = frame->freshCount;
    frame->measurement.registerCacheHitCount =
        engine->registerCache.cacheHitCount - registerHitStart;
    frame->measurement.registerWriteCount =
        engine->registerCache.writeCount - registerWriteStart;
    frame->measurement.registerReadbackCount =
        engine->registerCache.readbackCount - registerReadbackStart;
    frame->measurement.profileInvalidationCount =
        engine->profileCache.invalidationCount - profileInvalidationStart;
    frame->measurement.railFingerprintHitCount =
        engine->railFingerprint.hitCount - railHitStart;
    frame->measurement.railFingerprintMissCount =
        engine->railFingerprint.missCount - railMissStart;
    frame->measurement.railInvalidationCount =
        engine->railFingerprint.invalidationCount - railInvalidationStart;
    sensorarrayAdsMatrixFinishFrame(frame, frame->activeRows, frameStartUs, coherent);
    return firstErr;
}

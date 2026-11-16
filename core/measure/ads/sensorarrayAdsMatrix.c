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
#include "sensorarrayAcqEvent.h"

/* Bounded high-Z confirmation: exactly three fresh conversions per decision.
 * The sample's confirmationConversions telemetry reports this bound. Never
 * grow the loop without re-validating frame timing and DRDY handling. */
#define SENSORARRAY_ADS_MATRIX_OPEN_CONFIRM_SAMPLE_COUNT 3u
#define SENSORARRAY_ADS_MATRIX_OPEN_CONFIRM_MARGIN_FACTOR 3u
#define SENSORARRAY_ADS_MATRIX_ALL_CRC_REJECTED_LIMIT 3u

typedef struct {
    int32_t raw;
    int32_t differentialUv;
    uint8_t status;
    uint32_t generationStart;
    uint32_t generationEnd;
    uint32_t configurationGeneration;
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
    uint32_t configurationGeneration;
    uint8_t confirmationConversions;
    sensorarrayAdsOpenSemantic_t openSemantic;
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
    .openConfirmMarginUv = CONFIG_SENSORARRAY_ADS_MATRIX_RES_OPEN_DENOM_UV *
        SENSORARRAY_ADS_MATRIX_OPEN_CONFIRM_MARGIN_FACTOR,
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

static uint32_t sensorarrayAdsMatrixConfigurationGeneration(
    const sensorarrayAdsMatrixEngine_t *engine)
{
    if (!engine) {
        return 0u;
    }
    uint32_t generation = engine->calibrationGeneration +
                          engine->profileCache.generation +
                          engine->registerCache.generation +
                          engine->valueCache.generation;
    return generation == 0u ? 1u : generation;
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
    sensorarrayAdsMatrixEngine_t *engine,
    uint32_t startGeneration,
    uint32_t configurationGeneration,
    sensorarrayMeasurementPayload_t *telemetry,
    sensorarrayAdsMatrixSample_t *outSample)
{
    if (!engine || !engine->state || !telemetry || !outSample) {
        return ESP_ERR_INVALID_ARG;
    }
    ads126xAdcHandle_t *ads = &engine->state->ads;
    memset(outSample, 0, sizeof(*outSample));
    outSample->generationStart = startGeneration;
    outSample->configurationGeneration = configurationGeneration;
    uint32_t physicalGeneration = ads->configurationGeneration;
    ads126xAdcFreshReadReport_t report = {0};
    esp_err_t err = ads126xAdcReadAdc1FreshChecked(ads, startGeneration,
        (uint32_t)CONFIG_SENSORARRAY_ADS_DRDY_TIMEOUT_US,
        &engine->cellCrcRetryBudget, &outSample->raw, &outSample->status, &report);
    engine->cellReadTerminalError = report.terminalError;
    if (err != ESP_OK && (physicalGeneration != ads->configurationGeneration ||
        configurationGeneration != sensorarrayAdsMatrixConfigurationGeneration(engine))) {
        engine->cellReadTerminalError = ESP_ERR_INVALID_STATE;
    }
    /* firstError may remain CRC after a later timeout/configuration failure.
     * Only a completed, rejected wire sample permits local X02 continuation. */
    engine->cellCrcFailureOnly = err == ESP_ERR_INVALID_CRC &&
        engine->cellReadTerminalError == ESP_ERR_INVALID_CRC &&
        physicalGeneration == ads->configurationGeneration &&
        configurationGeneration == sensorarrayAdsMatrixConfigurationGeneration(engine) &&
        !ads126xAdcIsQuarantined(ads);
    outSample->generationEnd = report.generationEnd;
    telemetry->drdyWaitUs += report.waitUs;
    telemetry->sampleReadUs += report.readUs;
    telemetry->ioRetryCount += report.crcRetries;
    if (report.crcErrors != 0u) {
        if (engine->frameCrcErrors == 0u) {
            engine->frameFirstCrc = report.firstCrcFault;
            engine->frameFirstCrcCell = engine->currentCell;
        }
        engine->frameCrcErrors += report.crcErrors;
        engine->frameCrcRetries += report.crcRetries;
        engine->frameCrcRecovered += err == ESP_OK ? 1u : 0u;
        if (err != ESP_OK && engine->frameCrcTerminalError == ESP_OK) {
            engine->frameCrcTerminalError = report.terminalError;
        }
        if (err == ESP_ERR_INVALID_CRC) {
            engine->cellFailedCrc = report.firstCrcFault;
        }
    }
    if (err != ESP_OK) { return err; }
    telemetry->rawConversionCount++;
    outSample->differentialUv = ads126xAdcRawToMicrovolts(ads, outSample->raw);
    return ESP_OK;
}

static void sensorarrayAdsMatrixCaptureFault(sensorarrayAdsMatrixEngine_t *engine,
    sensorarrayAdsFaultStage_t stage, esp_err_t originalError,
    uint8_t reg, uint8_t expected, uint8_t actual, uint8_t mask)
{
    if (!engine || engine->faultContext.valid) {
        return; /* Preserve the first condition before any cleanup. */
    }
    sensorarrayAdsMatrixFaultContext_t *fault = &engine->faultContext;
    fault->valid = true;
    fault->stage = stage;
    fault->originalError = originalError;
    fault->mode = sensorarrayMeasurementModeName(engine->mode);
    fault->route = fault->mode;
    fault->owner = sensorarrayAdsOwnerName(engine->registerCache.owner);
    fault->sequence = engine->frameSequenceHint;
    fault->engineFrameCount = engine->frameCount;
    fault->modeEntryFrameCount = engine->modeEntryFrameCount;
    fault->healthInterval = CONFIG_SENSORARRAY_ADS_REGISTER_HEALTH_INTERVAL;
    fault->cacheGeneration = engine->registerCache.generation;
    fault->physicalGeneration = engine->state->ads.configurationGeneration;
    fault->configGeneration = sensorarrayAdsMatrixConfigurationGeneration(engine);
    fault->registerId = reg;
    fault->expected = expected;
    fault->actual = actual;
    fault->compareMask = mask;
    sensorarrayAdsGapCopyTransactionEvidence(&fault->rail, &fault->battery, &fault->check);
}

void sensorarrayAdsMatrixEngineRecordMixedBoundaryFault(
    sensorarrayAdsMatrixEngine_t *engine, uint32_t sequence,
    esp_err_t originalError, esp_err_t cleanupError)
{
    if (!engine || !engine->state) { return; }
    if (engine->faultContext.valid && engine->faultContext.sequence == sequence) {
        if (engine->faultContext.cleanupError == ESP_OK) {
            engine->faultContext.cleanupError = cleanupError;
        }
        return; /* The first condition for this sequence remains authoritative. */
    }
    engine->faultContext = (sensorarrayAdsMatrixFaultContext_t){0};
    engine->frameSequenceHint = sequence;
    sensorarrayAdsMatrixCaptureFault(engine,
        SENSORARRAY_ADS_FAULT_STAGE_RAIL_MONITOR, originalError,
        UINT8_MAX, 0u, 0u, 0u);
    /* No child engine has started. Its old mode/route are not evidence of
     * this boundary's physical state, especially after failed restoration. */
    engine->faultContext.mode = "MIXED";
    engine->faultContext.route = "unknown";
    engine->faultContext.owner = "RAIL";
    engine->faultContext.cleanupError = cleanupError;
}

bool sensorarrayAdsMatrixEngineCopyFaultContext(
    const sensorarrayAdsMatrixEngine_t *engine,
    sensorarrayAdsMatrixFaultContext_t *outContext)
{
    if (!engine || !outContext || !engine->faultContext.valid) {
        return false;
    }
    *outContext = engine->faultContext;
    return true;
}

static bool sensorarrayAdsMatrixNoteRegisterSnapshot(
    sensorarrayAdsMatrixEngine_t *engine,
    const ads126xAdc1RegisterSnapshot_t *snapshot)
{
    sensorarrayAdsRegisterCache_t *cache = engine ? &engine->registerCache : NULL;
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
        uint8_t expected = cache->registers[index].value;
        if (!sensorarrayAdsRegisterCacheNoteReadback(
                cache, (sensorarrayAdsRegisterId_t)index, values[index])) {
            sensorarrayAdsMatrixCaptureFault(engine,
                SENSORARRAY_ADS_FAULT_STAGE_CACHE_COMPARE, ESP_ERR_INVALID_RESPONSE,
                index, expected, values[index],
                sensorarrayAdsRegisterCompareMask((sensorarrayAdsRegisterId_t)index));
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
    ads126xAdcHandle_t *ads = &engine->state->ads;
    ads126xAdc1RegisterSnapshot_t snapshot = {0};
    int64_t readStartUs = esp_timer_get_time();
    esp_err_t err = ads126xAdcReadAdc1RegisterSnapshot(ads, &snapshot);
    int64_t readEndUs = esp_timer_get_time();
    if (readEndUs > readStartUs) {
        telemetry->registerReadbackUs += (uint64_t)(readEndUs - readStartUs);
    }
    if (err != ESP_OK) {
        sensorarrayAdsMatrixCaptureFault(engine,
            ads->snapshotReadFailure == 2u ?
                SENSORARRAY_ADS_FAULT_STAGE_REGISTER_READ_UNSTABLE :
                SENSORARRAY_ADS_FAULT_STAGE_REGISTER_HEALTH_READ,
            err,
            ads->snapshotReadFailure == 2u ? ads->snapshotMismatchRegister : 0u,
            ads->snapshotReadFailure == 2u ? ads->snapshotExpected : 0u,
            ads->snapshotReadFailure == 2u ? ads->snapshotActual : 0u,
            ads->snapshotReadFailure == 2u ? 0xFFu : 0u);
        goto failed;
    }
    sensorarrayRouteSnapshot_t route = {0};
    if (!engine->routeController ||
        !sensorarrayRouteControllerCopySnapshot(engine->routeController, &route)) {
        err = ESP_ERR_INVALID_STATE;
        sensorarrayAdsMatrixCaptureFault(engine,
            SENSORARRAY_ADS_FAULT_STAGE_MATRIX_ROUTE, err, 0u, 0u, 0u, 0u);
        goto failed;
    }
    uint8_t expectedPower =
        (route.profile.intRef == SENSORARRAY_ADS_INTREF_ON ? ADS126X_POWER_INTREF : 0u) |
        (route.profile.vbias == SENSORARRAY_ADS_VBIAS_ON ? ADS126X_POWER_VBIAS : 0u);
    const uint8_t values[SENSORARRAY_ADS_REGISTER_COUNT] = {
        snapshot.power, snapshot.interface, snapshot.mode0, snapshot.mode1,
        snapshot.mode2, snapshot.inpmux, snapshot.refmux,
        snapshot.offsetCal[0], snapshot.offsetCal[1], snapshot.offsetCal[2],
        snapshot.fullScaleCal[0], snapshot.fullScaleCal[1], snapshot.fullScaleCal[2]};
    sensorarrayAdsHealthComparison_t comparison = sensorarrayAdsRegisterHealthCompare(
        &engine->registerCache, values, expectedPower, route.profile.adsRefMux,
        (uint8_t)CONFIG_SENSORARRAY_ADS_MODE0_VALUE, (uint8_t)CONFIG_SENSORARRAY_ADS_MODE1_VALUE);
    if (comparison.reason != SENSORARRAY_ADS_HEALTH_OK) {
        const sensorarrayAdsFaultStage_t stages[] = {
            SENSORARRAY_ADS_FAULT_STAGE_UNKNOWN,
            SENSORARRAY_ADS_FAULT_STAGE_RESET_OBSERVED,
            SENSORARRAY_ADS_FAULT_STAGE_POWER_COMPARE,
            SENSORARRAY_ADS_FAULT_STAGE_REFMUX_COMPARE,
            SENSORARRAY_ADS_FAULT_STAGE_MODE_COMPARE,
            SENSORARRAY_ADS_FAULT_STAGE_CACHE_COMPARE};
        sensorarrayAdsMatrixCaptureFault(engine, stages[comparison.reason],
            ESP_ERR_INVALID_RESPONSE, (uint8_t)comparison.reg,
            comparison.expected, comparison.actual, comparison.mask);
        err = ESP_ERR_INVALID_RESPONSE;
        goto failed;
    }
    if (!sensorarrayAdsMatrixNoteRegisterSnapshot(engine, &snapshot)) {
        err = ESP_ERR_INVALID_RESPONSE;
        goto failed;
    }
    engine->registerCache.vrefValid = ads->vrefMicrovolts != 0u;
    engine->registerCache.vrefUv = (int32_t)ads->vrefMicrovolts;
    engine->registerCache.pgaModeValid = true;
    engine->registerCache.inputMode = ads126xAdcMode2PgaBypassed(snapshot.mode2) ?
        SENSORARRAY_ADS_INPUT_BYPASS : SENSORARRAY_ADS_INPUT_PGA;
    uint8_t gain = 1u;
    if (!ads126xAdcMode2PgaBypassed(snapshot.mode2) &&
        !ads126xAdcMode2DecodePgaGain(snapshot.mode2, &gain)) {
        sensorarrayAdsMatrixCaptureFault(engine,
            SENSORARRAY_ADS_FAULT_STAGE_MODE_COMPARE, ESP_ERR_INVALID_RESPONSE,
            SENSORARRAY_ADS_REGISTER_MODE2, 0u, snapshot.mode2, 0x70u);
        err = ESP_ERR_INVALID_RESPONSE;
        goto failed;
    }
    engine->registerCache.gain = gain;
    engine->registerCache.adc1RunningValid = true;
    engine->registerCache.adc1Running = ads126xAdcIsAdc1Running(ads);
    return ESP_OK;
failed:
    sensorarrayAdsRegisterCacheInvalidate(&engine->registerCache);
    sensorarrayAdsProfileCacheInvalidate(&engine->profileCache);
    sensorarrayAdsValueCacheInvalidate(&engine->valueCache);
    return err;
}

static esp_err_t sensorarrayAdsMatrixApplyInputProfile(
    sensorarrayAdsMatrixEngine_t *engine,
    uint8_t muxp,
    uint8_t muxn,
    sensorarrayAdsInputMode_t inputMode,
    uint8_t gain,
    sensorarrayMeasurementPayload_t *telemetry,
    bool forceRoute,
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
    if (forceRoute || sensorarrayAdsRegisterCacheNeedsWrite(
                          &engine->registerCache,
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
    uint32_t configurationGeneration =
        sensorarrayAdsMatrixConfigurationGeneration(engine);
    sensorarrayAdsMatrixSample_t samples[3] = {0};
    esp_err_t err = sensorarrayAdsMatrixReadFreshAfter(engine,
                                                        firstGeneration,
                                                        configurationGeneration,
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
    outResult->configurationGeneration = configurationGeneration;
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
        if (!sensorarrayAdsMathConfigurationGenerationCurrent(
                configurationGeneration,
                sensorarrayAdsMatrixConfigurationGeneration(engine))) {
            outResult->error = SENSORARRAY_CELL_ERROR_STALE;
            return ESP_ERR_INVALID_RESPONSE;
        }
        return ESP_OK;
    }
    for (uint8_t index = 1u; index < 3u; ++index) {
        uint32_t generation = ads126xAdcGetDrdyGeneration(&engine->state->ads);
        err = sensorarrayAdsMatrixReadFreshAfter(engine,
                                                  generation,
                                                  configurationGeneration,
                                                  telemetry,
                                                  &samples[index]);
        if (err != ESP_OK) {
            /* The first conversion remains genuinely fresh even if a stricter
             * second or third sample fails. Preserve that fact while the cell
             * itself is reported invalid. */
            return err;
        }
    }
    if (!sensorarrayAdsMathConfigurationGenerationCurrent(
            configurationGeneration,
            sensorarrayAdsMatrixConfigurationGeneration(engine))) {
        outResult->error = SENSORARRAY_CELL_ERROR_STALE;
        return ESP_ERR_INVALID_RESPONSE;
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

static esp_err_t sensorarrayAdsMatrixConfirmHighZ(
    sensorarrayAdsMatrixEngine_t *engine,
    const sensorarrayAdsRailSplit_t *rail,
    uint8_t muxp,
    uint8_t muxn,
    sensorarrayAdsInputMode_t inputMode,
    uint8_t gain,
    sensorarrayMeasurementPayload_t *telemetry,
    sensorarrayAdsMatrixCellResult_t *sample)
{
    if (!engine || !engine->state || !rail || !rail->valid || !telemetry ||
        !sample) {
        return ESP_ERR_INVALID_ARG;
    }
    if (sample->error != SENSORARRAY_CELL_ERROR_NONE) {
        /* The triggering sample already carries a classified error. A clean
         * confirmation must not overwrite it with a fresh-sample decision. */
        return ESP_ERR_INVALID_RESPONSE;
    }

    /* Re-apply the cell route and discard every conversion from the old
     * epoch. The forced INPMUX write creates a fresh DRDY boundary even when
     * the register shadow already holds the same mux value. */
    uint32_t firstGeneration = 0u;
    esp_err_t err = sensorarrayAdsMatrixApplyInputProfile(engine,
                                                          muxp,
                                                          muxn,
                                                          inputMode,
                                                          gain,
                                                          telemetry,
                                                          true,
                                                          &firstGeneration);
    if (err != ESP_OK) {
        sample->fresh = false;
        sample->error = sensorarrayAdsMatrixIoError(err);
        return err;
    }

    uint32_t configurationGeneration =
        sensorarrayAdsMatrixConfigurationGeneration(engine);
    sensorarrayAdsMatrixSample_t confirmSamples[
        SENSORARRAY_ADS_MATRIX_OPEN_CONFIRM_SAMPLE_COUNT] = {0};
    uint8_t confirmStatuses[SENSORARRAY_ADS_MATRIX_OPEN_CONFIRM_SAMPLE_COUNT] = {0};
    for (uint8_t index = 0u;
         index < SENSORARRAY_ADS_MATRIX_OPEN_CONFIRM_SAMPLE_COUNT;
         ++index) {
        err = sensorarrayAdsMatrixReadFreshAfter(engine,
                                                 firstGeneration,
                                                 configurationGeneration,
                                                 telemetry,
                                                 &confirmSamples[index]);
        if (err != ESP_OK) {
            sample->fresh = false;
            sample->error = sensorarrayAdsMatrixIoError(err);
            return err;
        }
        confirmStatuses[index] = confirmSamples[index].status;
        firstGeneration = confirmSamples[index].generationEnd;
    }
    if (!sensorarrayAdsMathConfigurationGenerationCurrent(
            configurationGeneration,
            sensorarrayAdsMatrixConfigurationGeneration(engine))) {
        sample->fresh = false;
        sample->error = SENSORARRAY_CELL_ERROR_STALE;
        return ESP_ERR_INVALID_RESPONSE;
    }

    int32_t rawValues[SENSORARRAY_ADS_MATRIX_OPEN_CONFIRM_SAMPLE_COUNT];
    int32_t uvValues[SENSORARRAY_ADS_MATRIX_OPEN_CONFIRM_SAMPLE_COUNT];
    int32_t nodeUvValues[SENSORARRAY_ADS_MATRIX_OPEN_CONFIRM_SAMPLE_COUNT];
    for (uint8_t index = 0u;
         index < SENSORARRAY_ADS_MATRIX_OPEN_CONFIRM_SAMPLE_COUNT;
         ++index) {
        int64_t nodeUv64 =
            (int64_t)confirmSamples[index].differentialUv + rail->aincomUv;
        nodeUvValues[index] = nodeUv64 < INT32_MIN ? INT32_MIN :
            (nodeUv64 > INT32_MAX ? INT32_MAX : (int32_t)nodeUv64);
        rawValues[index] = confirmSamples[index].raw;
        uvValues[index] = confirmSamples[index].differentialUv;
    }

    /* Retain every alarm observed in the triggering sample: a clean set of
     * confirmation conversions must not mask RESET/REFERENCE/PGA bits. */
    uint8_t combinedStatus = sensorarrayAdsMathCombineStatusBytes(
        sample->status,
        confirmStatuses,
        SENSORARRAY_ADS_MATRIX_OPEN_CONFIRM_SAMPLE_COUNT);
    sample->status = combinedStatus;
    sample->fresh = true;
    if ((combinedStatus & ADS126X_STATUS_RESET_OCCURRED) != 0u) {
        sample->error = SENSORARRAY_CELL_ERROR_READBACK;
        sensorarrayAdsMatrixEngineInvalidateCaches(engine);
        return ESP_ERR_INVALID_RESPONSE;
    }
    if ((combinedStatus & ADS126X_STATUS_REFERENCE_ALARM) != 0u) {
        sample->error = SENSORARRAY_CELL_ERROR_REFERENCE_ALARM;
        return ESP_ERR_INVALID_RESPONSE;
    }
    if (inputMode == SENSORARRAY_ADS_INPUT_PGA) {
        if ((combinedStatus & ADS126X_STATUS_PGA_DIFFERENTIAL_ALARM) != 0u) {
            sample->error = SENSORARRAY_CELL_ERROR_PGA_DIFFERENTIAL;
            return ESP_ERR_INVALID_RESPONSE;
        }
        if ((combinedStatus & (ADS126X_STATUS_PGA_LOW_ALARM |
                               ADS126X_STATUS_PGA_HIGH_ALARM)) != 0u) {
            sample->error = SENSORARRAY_CELL_ERROR_PGA_ABSOLUTE;
            return ESP_ERR_INVALID_RESPONSE;
        }
    }

    int32_t medianRaw = 0;
    int32_t medianUv = 0;
    int32_t medianNodeUv = 0;
    (void)sensorarrayAdsMathSamplesStable(rawValues,
                                          SENSORARRAY_ADS_MATRIX_OPEN_CONFIRM_SAMPLE_COUNT,
                                          UINT32_MAX,
                                          &medianRaw);
    (void)sensorarrayAdsMathSamplesStable(uvValues,
                                          SENSORARRAY_ADS_MATRIX_OPEN_CONFIRM_SAMPLE_COUNT,
                                          UINT32_MAX,
                                          &medianUv);
    sensorarrayAdsOpenConfirm_t decision = sensorarrayAdsMathConfirmOpenSet(
        nodeUvValues,
        SENSORARRAY_ADS_MATRIX_OPEN_CONFIRM_SAMPLE_COUNT,
        rail->avssUv,
        &s_sensorarrayAdsResistanceLimits,
        (uint32_t)CONFIG_SENSORARRAY_ADS_MATRIX_MAX_SPREAD_UV,
        &medianNodeUv);

    sample->raw = medianRaw;
    sample->differentialUv = medianUv;
    sample->nodeUv = medianNodeUv;
    sample->status = combinedStatus;
    sample->fresh = true;
    sample->fullSample = true;
    sample->spreadRaw = sensorarrayAdsMatrixRawSpread(rawValues);
    sample->configurationGeneration = configurationGeneration;
    sample->confirmationConversions =
        SENSORARRAY_ADS_MATRIX_OPEN_CONFIRM_SAMPLE_COUNT;
    if (decision == SENSORARRAY_ADS_OPEN_CONFIRM_HIGH_Z) {
        sample->openSemantic = SENSORARRAY_ADS_OPEN_SEMANTIC_HIGH_Z_CONFIRMED;
        sample->stable = false;
        sample->error = SENSORARRAY_CELL_ERROR_OPEN;
        return ESP_OK;
    }
    sample->openSemantic = SENSORARRAY_ADS_OPEN_SEMANTIC_NONE;
    sample->stable = decision == SENSORARRAY_ADS_OPEN_CONFIRM_NONE;
    sample->error = sample->stable ? SENSORARRAY_CELL_ERROR_NONE :
                                     SENSORARRAY_CELL_ERROR_UNSTABLE;
    return sample->stable ? ESP_OK : ESP_ERR_INVALID_RESPONSE;
}

static esp_err_t sensorarrayAdsMatrixMeasureCellImpl(sensorarrayAdsMatrixEngine_t *engine,
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
                                                         false,
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
            /* CRC has its own fresh-conversion retry sharing this cell's
             * deadline and CRC budget. Never restart that exhausted budget. */
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
        sensorarrayAdsHighZCandidateInput_t highZInput = {
            .nodeUv = outResult->nodeUv,
            .avssUv = rail->avssUv,
            .rawCode = sample.raw,
            .magnitude = magnitude,
            .saturationLimit = saturationLimit,
            .openDenominatorUv =
                s_sensorarrayAdsResistanceLimits.openDenominatorUv,
            .openConfirmMarginUv =
                s_sensorarrayAdsResistanceLimits.openConfirmMarginUv,
        };
        uint64_t cellBit = UINT64_C(1) << cellIndex;
        bool highZCandidate =
            engine->mode == SENSORARRAY_MEASUREMENT_MODE_RESISTANCE &&
            sensorarrayAdsMathHighZOpenCandidate(&highZInput);
        bool highZLatched = (engine->highZOpenMask & cellBit) != 0u;
        bool shouldConfirmHighZ = highZCandidate || highZLatched;
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
            if (shouldConfirmHighZ) {
                esp_err_t confirmErr = sensorarrayAdsMatrixConfirmHighZ(
                    engine, rail, muxp, muxn, inputMode, gain, telemetry, &sample);
                if (confirmErr != ESP_OK) {
                    *outResult = sample;
                    outResult->error = sample.error != SENSORARRAY_CELL_ERROR_NONE ?
                        sample.error : sensorarrayAdsMatrixIoError(confirmErr);
                    if (outResult->error == SENSORARRAY_CELL_ERROR_UNSTABLE &&
                        sample.fresh) {
                        sensorarrayAdsValueCacheObserve(
                            valueCache, sample.raw, sample.nodeUv,
                            SENSORARRAY_MEASUREMENT_INVALID_FIXED,
                            sample.spreadRaw, engine->frameCount, true);
                        if (valueCache) {
                            valueCache->stableStreak = 0u;
                            if (valueCache->unstableStreak < UINT32_MAX) {
                                valueCache->unstableStreak++;
                            }
                        }
                    }
                    sensorarrayAdsProfileCacheNoteFailure(
                        &engine->profileCache, engine->mode, (uint8_t)cellIndex, false,
                        (uint8_t)CONFIG_SENSORARRAY_ADS_PROFILE_FAILURE_INVALIDATE_COUNT);
                    return confirmErr;
                }
                *outResult = sample;
                engine->highZOpenMask = sensorarrayAdsMathHighZLatchUpdate(
                    engine->highZOpenMask, cellIndex, sample.openSemantic);
                if (sample.openSemantic ==
                    SENSORARRAY_ADS_OPEN_SEMANTIC_HIGH_Z_CONFIRMED) {
                    (void)sensorarrayAdsProfileCacheStore(
                        &engine->profileCache, engine->mode, (uint8_t)cellIndex,
                        SENSORARRAY_ADS_INPUT_BYPASS, 1u, engine->frameCount);
                    sensorarrayAdsValueCacheObserve(
                        valueCache, sample.raw, sample.nodeUv,
                        SENSORARRAY_MEASUREMENT_INVALID_FIXED,
                        sample.spreadRaw, engine->frameCount, true);
                    if (valueCache) {
                        valueCache->stableStreak = 0u;
                        if (valueCache->unstableStreak < UINT32_MAX) {
                            valueCache->unstableStreak++;
                        }
                    }
                    return ESP_OK;
                }
                nodeUv64 = sample.nodeUv;
                magnitude = sample.raw == INT32_MIN ?
                    (uint64_t)INT32_MAX + 1u :
                    (uint64_t)(sample.raw < 0 ? -sample.raw : sample.raw);
                inputsSafe = nodeUv64 >= lowerUv && nodeUv64 <= upperUv &&
                             (int64_t)rail->aincomUv >= lowerUv &&
                             (int64_t)rail->aincomUv <= upperUv;
                if ((sample.status & ADS126X_STATUS_REFERENCE_ALARM) != 0u) {
                    outResult->error = SENSORARRAY_CELL_ERROR_REFERENCE_ALARM;
                    sensorarrayAdsProfileCacheNoteFailure(
                        &engine->profileCache, engine->mode, (uint8_t)cellIndex, false,
                        (uint8_t)CONFIG_SENSORARRAY_ADS_PROFILE_FAILURE_INVALIDATE_COUNT);
                    return ESP_ERR_INVALID_RESPONSE;
                }
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
            if (shouldConfirmHighZ) {
                esp_err_t confirmErr = sensorarrayAdsMatrixConfirmHighZ(
                    engine, rail, muxp, muxn, inputMode, gain, telemetry, &sample);
                if (confirmErr != ESP_OK) {
                    *outResult = sample;
                    outResult->error = sample.error != SENSORARRAY_CELL_ERROR_NONE ?
                        sample.error : sensorarrayAdsMatrixIoError(confirmErr);
                    if (outResult->error == SENSORARRAY_CELL_ERROR_UNSTABLE &&
                        sample.fresh) {
                        sensorarrayAdsValueCacheObserve(
                            valueCache, sample.raw, sample.nodeUv,
                            SENSORARRAY_MEASUREMENT_INVALID_FIXED,
                            sample.spreadRaw, engine->frameCount, true);
                        if (valueCache) {
                            valueCache->stableStreak = 0u;
                            if (valueCache->unstableStreak < UINT32_MAX) {
                                valueCache->unstableStreak++;
                            }
                        }
                    }
                    sensorarrayAdsProfileCacheNoteFailure(
                        &engine->profileCache, engine->mode, (uint8_t)cellIndex, true,
                        (uint8_t)CONFIG_SENSORARRAY_ADS_PROFILE_FAILURE_INVALIDATE_COUNT);
                    return confirmErr;
                }
                *outResult = sample;
                engine->highZOpenMask = sensorarrayAdsMathHighZLatchUpdate(
                    engine->highZOpenMask, cellIndex, sample.openSemantic);
                if (sample.openSemantic ==
                    SENSORARRAY_ADS_OPEN_SEMANTIC_HIGH_Z_CONFIRMED) {
                    (void)sensorarrayAdsProfileCacheStore(
                        &engine->profileCache, engine->mode, (uint8_t)cellIndex,
                        SENSORARRAY_ADS_INPUT_BYPASS, 1u, engine->frameCount);
                    sensorarrayAdsValueCacheObserve(
                        valueCache, sample.raw, sample.nodeUv,
                        SENSORARRAY_MEASUREMENT_INVALID_FIXED,
                        sample.spreadRaw, engine->frameCount, true);
                    if (valueCache) {
                        valueCache->stableStreak = 0u;
                        if (valueCache->unstableStreak < UINT32_MAX) {
                            valueCache->unstableStreak++;
                        }
                    }
                    return ESP_OK;
                }
                magnitude = sample.raw == INT32_MIN ?
                    (uint64_t)INT32_MAX + 1u :
                    (uint64_t)(sample.raw < 0 ? -sample.raw : sample.raw);
                if (magnitude >= saturationLimit) {
                    outResult->error = SENSORARRAY_CELL_ERROR_SATURATED;
                    sensorarrayAdsProfileCacheNoteFailure(
                        &engine->profileCache, engine->mode, (uint8_t)cellIndex, true,
                        (uint8_t)CONFIG_SENSORARRAY_ADS_PROFILE_FAILURE_INVALIDATE_COUNT);
                    return ESP_ERR_INVALID_RESPONSE;
                }
            } else {
                outResult->error = decision.error;
                sensorarrayAdsProfileCacheNoteFailure(
                    &engine->profileCache, engine->mode, (uint8_t)cellIndex, true,
                    (uint8_t)CONFIG_SENSORARRAY_ADS_PROFILE_FAILURE_INVALIDATE_COUNT);
                return ESP_ERR_INVALID_RESPONSE;
            }
        }
        if (decision.action == SENSORARRAY_ADS_AUTORANGE_KEEP &&
            shouldConfirmHighZ) {
            esp_err_t confirmErr = sensorarrayAdsMatrixConfirmHighZ(
                engine, rail, muxp, muxn, inputMode, gain, telemetry, &sample);
            if (confirmErr != ESP_OK) {
                *outResult = sample;
                outResult->error = sample.error != SENSORARRAY_CELL_ERROR_NONE ?
                    sample.error : sensorarrayAdsMatrixIoError(confirmErr);
                if (outResult->error == SENSORARRAY_CELL_ERROR_UNSTABLE &&
                    sample.fresh) {
                    sensorarrayAdsValueCacheObserve(
                        valueCache, sample.raw, sample.nodeUv,
                        SENSORARRAY_MEASUREMENT_INVALID_FIXED,
                        sample.spreadRaw, engine->frameCount, true);
                    if (valueCache) {
                        valueCache->stableStreak = 0u;
                        if (valueCache->unstableStreak < UINT32_MAX) {
                            valueCache->unstableStreak++;
                        }
                    }
                }
                sensorarrayAdsProfileCacheNoteFailure(
                    &engine->profileCache, engine->mode, (uint8_t)cellIndex, false,
                    (uint8_t)CONFIG_SENSORARRAY_ADS_PROFILE_FAILURE_INVALIDATE_COUNT);
                return confirmErr;
            }
            *outResult = sample;
            engine->highZOpenMask = sensorarrayAdsMathHighZLatchUpdate(
                engine->highZOpenMask, cellIndex, sample.openSemantic);
            if (sample.openSemantic ==
                SENSORARRAY_ADS_OPEN_SEMANTIC_HIGH_Z_CONFIRMED) {
                (void)sensorarrayAdsProfileCacheStore(
                    &engine->profileCache, engine->mode, (uint8_t)cellIndex,
                    SENSORARRAY_ADS_INPUT_BYPASS, 1u, engine->frameCount);
                sensorarrayAdsValueCacheObserve(
                    valueCache, sample.raw, sample.nodeUv,
                    SENSORARRAY_MEASUREMENT_INVALID_FIXED,
                    sample.spreadRaw, engine->frameCount, true);
                if (valueCache) {
                    valueCache->stableStreak = 0u;
                    if (valueCache->unstableStreak < UINT32_MAX) {
                        valueCache->unstableStreak++;
                    }
                }
                return ESP_OK;
            }
            magnitude = sample.raw == INT32_MIN ?
                (uint64_t)INT32_MAX + 1u :
                (uint64_t)(sample.raw < 0 ? -sample.raw : sample.raw);
            if (magnitude >= saturationLimit) {
                outResult->error = SENSORARRAY_CELL_ERROR_SATURATED;
                sensorarrayAdsProfileCacheNoteFailure(
                    &engine->profileCache, engine->mode, (uint8_t)cellIndex, true,
                    (uint8_t)CONFIG_SENSORARRAY_ADS_PROFILE_FAILURE_INVALIDATE_COUNT);
                return ESP_ERR_INVALID_RESPONSE;
            }
        }
        if (!outResult->stable) {
            sensorarrayAdsValueCacheObserve(valueCache,
                                             sample.raw,
                                             outResult->nodeUv,
                                             SENSORARRAY_MEASUREMENT_INVALID_FIXED,
                                             sample.spreadRaw,
                                             engine->frameCount,
                                             sample.fullSample);
            /* A repeatable median does not make the three conversions stable.
             * Match bypass: retain diagnostic history, but require a new
             * stable streak before permitting the single-sample fast path. */
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

/* 本轮新增: one cell budget includes every range retry, DRDY and SPI call. */
static esp_err_t sensorarrayAdsMatrixMeasureCell(sensorarrayAdsMatrixEngine_t *engine,
    const sensorarrayAdsRailSplit_t *rail, uint8_t row, uint8_t dLine,
    bool precisionFrame, bool transitionSensitive,
    sensorarrayMeasurementPayload_t *telemetry, sensorarrayAdsMatrixCellResult_t *outResult)
{
    if (!engine || !engine->state) { return ESP_ERR_INVALID_ARG; }
    engine->cellCrcRetryBudget = CONFIG_SENSORARRAY_ADS_MATRIX_IO_RETRY_COUNT;
    engine->cellReadTerminalError = ESP_OK;
    engine->cellCrcFailureOnly = false;
    engine->cellHardFailure = false;
    engine->currentCell = (uint8_t)sensorarrayMatrixIndex(row, dLine);
    memset(&engine->cellFailedCrc, 0, sizeof(engine->cellFailedCrc));
    esp_err_t err = ads126xAdcBeginTransaction(&engine->state->ads, 100000u);
    if (err != ESP_OK) {
        engine->cellHardFailure = true;
        engine->cellReadTerminalError = err;
        sensorarrayAdsMatrixCaptureFault(engine, SENSORARRAY_ADS_FAULT_STAGE_OWNER_ACQUIRE,
            err, 0u, 0u, 0u, 0u);
        return err;
    }
    err = sensorarrayAdsMatrixMeasureCellImpl(engine, rail, row, dLine,
        precisionFrame, transitionSensitive, telemetry, outResult);
    ads126xAdcEndTransaction(&engine->state->ads);
    /* Electrical invalidity (OPEN/range/unstable/PGA) is a cell result.
     * Hardware, ownership, freshness, RESET and reference failures are not.
     * This classification is independent of the frame's preserved firstErr. */
    bool statusUnknown = (outResult->status &
        (ADS126X_STATUS_RESET_OCCURRED | ADS126X_STATUS_REFERENCE_ALARM)) != 0u;
    bool localCrc = err == ESP_ERR_INVALID_CRC && engine->cellCrcFailureOnly &&
        !statusUnknown && !ads126xAdcIsQuarantined(&engine->state->ads);
    engine->cellCrcFailureOnly = localCrc;
    engine->cellHardFailure = ads126xAdcIsQuarantined(&engine->state->ads) ||
        statusUnknown || (err != ESP_OK && !localCrc &&
        (!outResult->fresh || outResult->error == SENSORARRAY_CELL_ERROR_SPI ||
         outResult->error == SENSORARRAY_CELL_ERROR_DRDY_TIMEOUT ||
         outResult->error == SENSORARRAY_CELL_ERROR_STALE ||
         outResult->error == SENSORARRAY_CELL_ERROR_READBACK ||
         outResult->error == SENSORARRAY_CELL_ERROR_ROUTE ||
         outResult->error == SENSORARRAY_CELL_ERROR_RAIL_INVALID ||
         outResult->error == SENSORARRAY_CELL_ERROR_REFERENCE_INVALID ||
         outResult->error == SENSORARRAY_CELL_ERROR_REFERENCE_ALARM));
    if (err == ESP_OK && engine->cellHardFailure) { err = ESP_ERR_INVALID_RESPONSE; }
    if (err != ESP_OK && outResult->error != SENSORARRAY_CELL_ERROR_OPEN) {
        sensorarrayAdsFaultStage_t stage = SENSORARRAY_ADS_FAULT_STAGE_MATRIX_READ;
        if (err == ESP_ERR_INVALID_CRC) {
            stage = SENSORARRAY_ADS_FAULT_STAGE_MATRIX_CRC;
        } else if ((outResult->status & ADS126X_STATUS_RESET_OCCURRED) != 0u) {
            stage = SENSORARRAY_ADS_FAULT_STAGE_RESET_OBSERVED;
        } else if (outResult->error == SENSORARRAY_CELL_ERROR_DRDY_TIMEOUT) {
            stage = SENSORARRAY_ADS_FAULT_STAGE_MATRIX_DRDY;
        } else if (outResult->error == SENSORARRAY_CELL_ERROR_READBACK ||
                   outResult->error == SENSORARRAY_CELL_ERROR_STALE) {
            stage = SENSORARRAY_ADS_FAULT_STAGE_MATRIX_READBACK;
        }
        /* Sample/CRC evidence is not an ADS register address. S/D live in
         * ADSSPI; 255 explicitly means register not applicable here. */
        sensorarrayAdsMatrixCaptureFault(engine, stage, err, UINT8_MAX,
            err == ESP_ERR_INVALID_CRC ? engine->cellFailedCrc.computed : 0u,
            err == ESP_ERR_INVALID_CRC ? engine->cellFailedCrc.received : outResult->status,
            err == ESP_ERR_INVALID_CRC ? 0xFFu : 0u);
    }
    return err;
}

/* Queue diagnostic copies only after the cell transaction has ended. Rate
 * limiting affects text, not cumulative integrity/retry counters or Xhh. */
static void sensorarrayAdsMatrixLogCrcEvidence(sensorarrayAdsMatrixEngine_t *engine,
    uint32_t sequence, esp_err_t frameError)
{
    if (engine->frameCrcErrors == 0u) { return; }
    uint64_t nowUs = (uint64_t)esp_timer_get_time();
    if (engine->frameCrcTerminalError == ESP_OK && engine->lastCrcEmitUs != 0u &&
        nowUs - engine->lastCrcEmitUs < 1000000u) {
        return;
    }
    engine->lastCrcEmitUs = nowUs;
    const ads126xAdcCrcFault_t *crc = &engine->frameFirstCrc;
    /* 本轮新增：the parser payload is not necessarily continuous SPI wire.
     * A GPIO-tail CRC rejection retains its two physical observations. */
    bool split = crc->readSource == ADS126X_READ_SOURCE_SPI55_GPIO1;
    bool payloadKnown = crc->length > 0u && crc->length <= sizeof(crc->bytes);
    bool gpioKnown = split && crc->gpioTailKnown && crc->gpioTailBit <= 1u;
    bool wireKnown = !split && crc->wireKnown && payloadKnown &&
        (crc->readSource == ADS126X_READ_SOURCE_SPI56 || crc->readSource == ADS126X_READ_SOURCE_SPI_BYTES);
    char payload[13] = "unknown", prefix[15] = "unknown", gpioBit[8] = "unknown";
    if (payloadKnown && (wireKnown || gpioKnown)) {
        for (unsigned i = 0u; i < crc->length; ++i) {
            (void)snprintf(payload + i * 2u, sizeof(payload) - i * 2u, "%02X", crc->bytes[i]);
        }
    }
    if (gpioKnown) {
        for (unsigned i = 0u; i < sizeof(crc->spiPrefix); ++i) {
            (void)snprintf(prefix + i * 2u, sizeof(prefix) - i * 2u, "%02X", crc->spiPrefix[i]);
        }
        (void)snprintf(gpioBit, sizeof(gpioBit), "%u", crc->gpioTailBit);
    }
    sensorarrayAcqEventPrintf(
        "ADSSPI,seq=%lu,mode=%s,crcFailures=%lu,crcRetries=%lu,crcRecoveredReads=%lu,crcTotal=%lu,retryTotal=%lu,firstS=%u,firstD=%u,firstError=0x%x,terminalError=0x%lx,frameError=0x%lx\n",
        (unsigned long)sequence, sensorarrayMeasurementModeName(engine->mode),
        (unsigned long)engine->frameCrcErrors, (unsigned long)engine->frameCrcRetries,
        (unsigned long)engine->frameCrcRecovered,
        (unsigned long)engine->state->ads.adc1CrcErrorCount,
        (unsigned long)engine->state->ads.adc1CrcRetryCount,
        engine->frameFirstCrcCell / SENSORARRAY_MATRIX_COLS + 1u,
        engine->frameFirstCrcCell % SENSORARRAY_MATRIX_COLS + 1u,
        ESP_ERR_INVALID_CRC, (unsigned long)(uint32_t)engine->frameCrcTerminalError,
        (unsigned long)(uint32_t)frameError);
    if (split) {
        sensorarrayAcqEventPrintf(
            "ADSSPIBYTES,seq=%lu,firstUs=%llu,bytes=unknown,len=%u,received=%02X,computed=%02X,crcMode=%u,statusEnabled=%u,physicalGen=%lu,drdyGen=%lu,source=spi55_gpio1,wireKnown=0,assembledBytes=%s,spiPrefix=%s,gpioKnown=%u,gpioBit=%s,tailStage=%s\n",
            (unsigned long)sequence, (unsigned long long)crc->timestampUs,
            crc->length, crc->received, crc->computed, crc->crcMode, crc->statusEnabled ? 1u : 0u,
            (unsigned long)crc->configurationGeneration, (unsigned long)crc->drdyGeneration,
            payload, prefix, gpioKnown ? 1u : 0u, gpioBit, ads126xAdcTailStageName(crc->tailStage));
    } else {
        sensorarrayAcqEventPrintf(
            "ADSSPIBYTES,seq=%lu,firstUs=%llu,bytes=%s,len=%u,received=%02X,computed=%02X,crcMode=%u,statusEnabled=%u,physicalGen=%lu,drdyGen=%lu,source=%s,wireKnown=%u\n",
            (unsigned long)sequence, (unsigned long long)crc->timestampUs, payload,
            crc->length, crc->received, crc->computed, crc->crcMode, crc->statusEnabled ? 1u : 0u,
            (unsigned long)crc->configurationGeneration, (unsigned long)crc->drdyGeneration,
            ads126xAdcReadSourceName(crc->readSource), wireKnown ? 1u : 0u);
    }
}

static esp_err_t sensorarrayAdsMatrixGetRail(sensorarrayAdsMatrixEngine_t *engine,
                                        sensorarrayAdsRailSplit_t *outRail,
                                        sensorarrayAdsGapSnapshot_t *outGap,
                                        esp_err_t *outCleanupError,
                                        bool allowRefresh)
{
    *outCleanupError = ESP_OK;
    uint32_t sequence = engine->frameSequenceHint ?
        engine->frameSequenceHint : (engine->frameCount + 1u);
    sensorarrayAdsGapSnapshot_t gap = {0};
    sensorarrayAdsGapCopySnapshot(&gap, sequence);
    bool valid = sensorarrayAdsGapCopyRailSplit(
        sequence,
        (uint32_t)CONFIG_SENSORARRAY_ADS_MATRIX_RAIL_MAX_AGE_FRAMES,
        outRail);
    if (!valid) {
        /* A mixed segment is not a complete-frame boundary. Its caller
         * prepares rail before any group runs. VOLT must use a legal rail
         * calibration here; never release its clamp inside a live frame. */
        if (!allowRefresh || engine->mode == SENSORARRAY_MEASUREMENT_MODE_VOLTAGE) {
            if (outGap) {
                *outGap = gap;
            }
            return ESP_ERR_INVALID_STATE;
        }
        sensorarrayAdsTransactionEvidence_t before = {0}, after = {0};
        sensorarrayAdsGapCopyTransactionEvidence(&before, NULL, NULL);
        esp_err_t refreshErr = sensorarrayAdsGapRefreshRailAtBoundary(engine->state, sequence);
        sensorarrayAdsGapCopyTransactionEvidence(&after, NULL, NULL);
        if (after.id != before.id) {
            *outCleanupError = after.cleanupError;
        }
        if (refreshErr != ESP_OK) {
            if (outGap) {
                *outGap = gap;
            }
            return refreshErr;
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
    return valid ? ESP_OK : ESP_ERR_INVALID_STATE;
}

static void sensorarrayAdsMatrixFinishFrame(sensorarrayFrame_t *frame,
                                            uint8_t rows,
                                            int64_t startUs,
                                            bool coherent)
{
    uint8_t rowMask = frame->configSnapshot.rowMask;
    if (rowMask == 0u) {
        rowMask = rows >= SENSORARRAY_MATRIX_ROWS ? 0xFFu :
            (uint8_t)((1u << rows) - 1u);
    }
    uint64_t activeMask = sensorarrayFrameBuilderActiveCellMask(rowMask);
    frame->expectedMask = activeMask;
    frame->acquiredMask = frame->measurement.freshMask & activeMask;
    frame->measurement.validMask &= activeMask;
    frame->measurement.freshMask &= activeMask;
    frame->validMask &= activeMask;
    frame->freshMask &= activeMask;
    frame->measurement.errorMask &= activeMask;
    frame->errorMask &= activeMask;
    /* Cells that never completed a conversion/read this sweep must not keep
     * payload from an earlier sweep. Preserve a specific acquisition-failure
     * reason when one was recorded; otherwise classify the cell as STALE. */
    uint64_t unacquiredMask = activeMask & ~frame->acquiredMask;
    for (size_t index = 0u; index < SENSORARRAY_MATRIX_CELL_COUNT; ++index) {
        uint64_t bit = UINT64_C(1) << index;
        if ((unacquiredMask & bit) == 0u) {
            continue;
        }
        frame->measurement.valuesFixed[index] = SENSORARRAY_MEASUREMENT_INVALID_FIXED;
        frame->measurement.validMask &= ~bit;
        frame->measurement.freshMask &= ~bit;
        frame->validMask &= ~bit;
        frame->freshMask &= ~bit;
        frame->measurement.errorMask |= bit;
        frame->errorMask |= bit;
        if (frame->measurement.errorReason[index] ==
                SENSORARRAY_CELL_ERROR_NONE ||
            frame->measurement.errorReason[index] ==
                SENSORARRAY_CELL_ERROR_UNSUPPORTED) {
            frame->measurement.errorReason[index] = SENSORARRAY_CELL_ERROR_STALE;
        }
    }
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
        if ((rowMask & (uint8_t)(1u << row)) == 0u) {
            continue;
        }
        uint64_t cellMask = UINT64_C(0xFF) << (row * SENSORARRAY_MATRIX_COLS);
        if ((frame->measurement.freshMask & cellMask) == cellMask) {
            frame->rowFreshMask |= (uint8_t)(1u << row);
        }
    }
    frame->primaryFreshMask = frame->rowFreshMask;
    frame->secondaryFreshMask = frame->rowFreshMask;
    if (startUs > 0 && (int64_t)frame->frameEndUs > startUs) {
        frame->measurement.frameDurationUs = (uint64_t)((int64_t)frame->frameEndUs - startUs);
    }
    if (startUs > 0) {
        if (frame->measurement.mode == SENSORARRAY_MEASUREMENT_MODE_VOLTAGE) {
            frame->voltStartUs = (uint64_t)startUs;
            frame->voltEndUs = frame->frameEndUs;
        } else if (frame->measurement.mode ==
                   SENSORARRAY_MEASUREMENT_MODE_RESISTANCE) {
            frame->resStartUs = (uint64_t)startUs;
            frame->resEndUs = frame->frameEndUs;
        }
    }
    frame->maxSkewUs = sensorarrayFrameBuilderMaxGroupSkewUs(
        frame->capStartUs, frame->capEndUs,
        frame->voltStartUs, frame->voltEndUs,
        frame->resStartUs, frame->resEndUs);
}

/* 本轮新增: sample integrity is independent of acquisition freshness. This
 * budget counts ADS engine sweeps, including sparse MIXED segments. CAP and
 * mode changes do not reset it; a completed non-CRC ADS cell does. */
static bool sensorarrayAdsMatrixCompleteCrcDisposition(
    sensorarrayAdsMatrixEngine_t *engine, sensorarrayFrame_t *frame,
    uint64_t crcRejectedMask, bool coherent)
{
    frame->boundedCrcFailure = false;
    uint8_t rowMask = frame->configSnapshot.rowMask;
    if (rowMask == 0u) {
        rowMask = frame->activeRows >= SENSORARRAY_MATRIX_ROWS ? 0xFFu :
            (uint8_t)((1u << frame->activeRows) - 1u);
    }
    uint64_t expected = sensorarrayFrameBuilderActiveCellMask(rowMask);
    if (!coherent || expected == 0u) { return false; }
    crcRejectedMask &= expected;
    if (crcRejectedMask == expected) {
        if (engine->allCrcRejectedSweeps < UINT8_MAX) {
            engine->allCrcRejectedSweeps++;
        }
        if (engine->allCrcRejectedSweeps >= SENSORARRAY_ADS_MATRIX_ALL_CRC_REJECTED_LIMIT) {
            return false;
        }
    } else if ((frame->measurement.freshMask & expected & ~crcRejectedMask) != 0u) {
        engine->allCrcRejectedSweeps = 0u;
    }
    uint64_t missing = expected & ~frame->measurement.freshMask;
    if ((missing & ~crcRejectedMask) != 0u) { return false; }
    frame->boundedCrcFailure = crcRejectedMask != 0u;
    return true;
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
    engine->highZOpenMask = 0u;
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
        engine->modeEntryFrameCount = engine->frameCount;
        engine->highZOpenMask = 0u;
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
        engine->highZOpenMask = 0u;
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
    engine->highZOpenMask = 0u;
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
    memset(&engine->faultContext, 0, sizeof(engine->faultContext));
    engine->frameCrcErrors = 0u;
    engine->frameCrcRetries = 0u;
    engine->frameCrcRecovered = 0u;
    engine->frameCrcTerminalError = ESP_OK;
    uint32_t helperGeneration = sensorarrayAdsGapTransactionGeneration();
    if (engine->helperGeneration != helperGeneration) {
        sensorarrayAdsProfileCacheInvalidate(&engine->profileCache);
        sensorarrayAdsValueCacheInvalidate(&engine->valueCache);
        engine->transitionSensitiveFrames = 1u;
        engine->helperGeneration = helperGeneration;
    }
    sensorarrayRouteSnapshot_t route;
    if (!sensorarrayRouteControllerCopySnapshot(engine->routeController, &route) ||
        route.safe || route.mode != engine->mode || !route.gpioReadbackValid ||
        !route.adsReadbackValid) {
        sensorarrayAdsMatrixCaptureFault(engine,
            SENSORARRAY_ADS_FAULT_STAGE_MATRIX_ROUTE, ESP_ERR_INVALID_STATE, 0u, 0u, 0u, 0u);
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

    engine->faultContext.precisionFrame = frame->measurement.precisionFrame;
    sensorarrayAdsRailSplit_t rail = {0};
    esp_err_t railCleanupError = ESP_OK;
    esp_err_t railError = sensorarrayAdsMatrixGetRail(engine, &rail, &frame->adsGap,
        &railCleanupError, plan->auxiliaryTransactionsAllowed);
    if (railError != ESP_OK) {
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
        sensorarrayAdsMatrixCaptureFault(engine,
            SENSORARRAY_ADS_FAULT_STAGE_RAIL_MONITOR, railError, UINT8_MAX, 0u, 0u, 0u);
        esp_err_t safeError = sensorarrayRouteControllerEnterSafe(
            engine->routeController, "rail_invalid");
        engine->faultContext.cleanupError = railCleanupError != ESP_OK ?
            railCleanupError : safeError;
        sensorarrayAdsMatrixFinishFrame(frame, frame->activeRows, frameStartUs, false);
        return railError;
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
        sensorarrayAdsMatrixCaptureFault(engine,
            SENSORARRAY_ADS_FAULT_STAGE_OWNER_ACQUIRE, ESP_ERR_INVALID_STATE, 0u,
            SENSORARRAY_ADS_OWNER_NONE, engine->registerCache.owner, 0xFFu);
        engine->faultContext.cleanupError = sensorarrayRouteControllerEnterSafe(
            engine->routeController, "ads_owner_collision");
        sensorarrayAdsMatrixFinishFrame(frame, frame->activeRows, frameStartUs, false);
        return ESP_ERR_INVALID_STATE;
    }
    bool healthReadbackDue = sensorarrayAdsRegisterHealthDue(engine->frameCount,
        CONFIG_SENSORARRAY_ADS_REGISTER_HEALTH_INTERVAL,
        engine->registerCache.registers[SENSORARRAY_ADS_REGISTER_MODE0].valid);
    engine->faultContext.healthReadbackDue = healthReadbackDue;
    esp_err_t healthError = ESP_OK;
    if (healthReadbackDue) {
        healthError = ads126xAdcBeginTransaction(&engine->state->ads, 100000u);
        if (healthError == ESP_OK) {
            healthError = sensorarrayAdsMatrixRefreshRegisterShadow(engine, &frame->measurement);
            ads126xAdcEndTransaction(&engine->state->ads);
        } else {
            sensorarrayAdsMatrixCaptureFault(engine, SENSORARRAY_ADS_FAULT_STAGE_OWNER_ACQUIRE,
                healthError, 0u, 0u, 0u, 0u);
        }
        sensorarrayAcqEventPrintf(
            "ADSHEALTH,seq=%lu,engineFrameCount=%lu,modeEntryFrameCount=%lu,healthInterval=%u,healthReadbackDue=1,precisionFrame=%u,err=0x%lx,crcValidEven=%lu,crcValidOdd=%lu,crcErrors=%lu,crcRetries=%lu\n",
            (unsigned long)frame->sequence, (unsigned long)engine->frameCount,
            (unsigned long)engine->modeEntryFrameCount,
            CONFIG_SENSORARRAY_ADS_REGISTER_HEALTH_INTERVAL,
            frame->measurement.precisionFrame ? 1u : 0u,
            (unsigned long)(uint32_t)healthError,
            (unsigned long)engine->state->ads.adc1CrcValidEvenCount,
            (unsigned long)engine->state->ads.adc1CrcValidOddCount,
            (unsigned long)engine->state->ads.adc1CrcErrorCount,
            (unsigned long)engine->state->ads.adc1CrcRetryCount);
    }
    if (healthError != ESP_OK) {
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
        engine->faultContext.cleanupError = sensorarrayRouteControllerEnterSafe(
            engine->routeController, "ads_register_health");
        sensorarrayAdsMatrixFinishFrame(frame, frame->activeRows, frameStartUs, false);
        return healthError;
    }

    esp_err_t firstErr = ESP_OK;
    esp_err_t abortError = ESP_OK;
    uint64_t crcRejectedMask = 0u;
    bool coherent = true;
    bool transitionSensitive = engine->transitionSensitiveFrames != 0u || railInvalidated;
    for (uint8_t row = 1u; row <= frame->activeRows; ++row) {
        if ((plan->configSnapshot.rowMask & (uint8_t)(1u << (row - 1u))) == 0u) {
            continue;
        }
        int64_t rowStartUs = esp_timer_get_time();
        esp_err_t rowErr = sensorarrayRouteControllerSelectRow(engine->routeController, row);
        int64_t rowEndUs = esp_timer_get_time();
        if (rowEndUs > rowStartUs) {
            frame->measurement.rowRouteUs += (uint64_t)(rowEndUs - rowStartUs);
        }
        if (rowErr != ESP_OK) {
            sensorarrayRouteSnapshot_t failedRoute = {0};
            sensorarrayRouteControllerCopySnapshot(engine->routeController, &failedRoute);
            const char *stage = failedRoute.failureStage ? failedRoute.failureStage : "unknown";
            sensorarrayAdsFaultStage_t faultStage = SENSORARRAY_ADS_FAULT_STAGE_MATRIX_ROUTE;
            if (strcmp(stage, "gpio_mismatch") == 0 || strcmp(stage, "gpio_readback") == 0) {
                faultStage = SENSORARRAY_ADS_FAULT_STAGE_ROW_GPIO_COMPARE;
            } else if (strcmp(stage, "power_mismatch") == 0) {
                faultStage = SENSORARRAY_ADS_FAULT_STAGE_POWER_COMPARE;
            } else if (strcmp(stage, "refmux_mismatch") == 0) {
                faultStage = SENSORARRAY_ADS_FAULT_STAGE_REFMUX_COMPARE;
            } else if (strcmp(stage, "ads_readback") == 0 || strcmp(stage, "ads_retry_read") == 0) {
                faultStage = SENSORARRAY_ADS_FAULT_STAGE_REGISTER_HEALTH_READ;
            }
            sensorarrayAdsMatrixCaptureFault(engine, faultStage, rowErr,
                faultStage == SENSORARRAY_ADS_FAULT_STAGE_REFMUX_COMPARE ?
                    SENSORARRAY_ADS_REGISTER_REFMUX : row,
                failedRoute.failureExpected, failedRoute.failureActual, failedRoute.failureMask);
            engine->faultContext.cleanupError = failedRoute.cleanupError;
            for (uint8_t dLine = 1u; dLine <= SENSORARRAY_MATRIX_COLS; ++dLine) {
                sensorarrayAdsMatrixSetCellError(frame,
                                                 sensorarrayMatrixIndex(row, dLine),
                                                 SENSORARRAY_CELL_ERROR_ROUTE,
                                                 false);
            }
            if (firstErr == ESP_OK) { firstErr = rowErr; }
            abortError = rowErr;
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
            engine->faultContext.attempted = true;
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
                if (engine->cellCrcFailureOnly) {
                    crcRejectedMask |= UINT64_C(1) << index;
                }
                if (error != SENSORARRAY_CELL_ERROR_OPEN && firstErr == ESP_OK) {
                    firstErr = err;
                }
                if (engine->cellHardFailure) {
                    /* A later hard failure must abort even when firstErr and
                     * the first diagnostic context already preserve CRC. */
                    abortError = engine->cellReadTerminalError != ESP_OK ?
                        engine->cellReadTerminalError : err;
                    coherent = false;
                    break;
                }
                continue;
            }

            if (!sensorarrayAdsMathConfigurationGenerationCurrent(
                    result.configurationGeneration,
                    sensorarrayAdsMatrixConfigurationGeneration(engine))) {
                sensorarrayAdsMatrixSetCellError(frame,
                                                 index,
                                                 SENSORARRAY_CELL_ERROR_STALE,
                                                 false);
                if (firstErr == ESP_OK) {
                    firstErr = ESP_ERR_INVALID_RESPONSE;
                }
                abortError = ESP_ERR_INVALID_RESPONSE;
                coherent = false;
                break;
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
                resistance.configurationGeneration =
                    result.configurationGeneration;
                if (result.openSemantic ==
                    SENSORARRAY_ADS_OPEN_SEMANTIC_HIGH_Z_CONFIRMED) {
                    /* Consume the bounded confirmation on the resistance
                     * decision itself. The latch path has already retained
                     * the cell; a transient finite median must still publish
                     * OPEN with the same wire error code as a raw open. */
                    resistance.openSemantic =
                        SENSORARRAY_ADS_OPEN_SEMANTIC_HIGH_Z_CONFIRMED;
                    resistance.open = true;
                    resistance.error = SENSORARRAY_CELL_ERROR_OPEN;
                    resistance.valid = false;
                    sensorarrayAdsMatrixSetCellError(frame,
                                                     index,
                                                     SENSORARRAY_CELL_ERROR_OPEN,
                                                     true);
                    continue;
                }
                if (!resistance.valid) {
                    sensorarrayAdsMatrixSetCellError(frame,
                                                     index,
                                                     resistance.error,
                                                     true);
                    if (resistance.error != SENSORARRAY_CELL_ERROR_OPEN &&
                        firstErr == ESP_OK) {
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
        if (!coherent) { break; }
    }
    (void)sensorarrayAdsRegisterCacheRelease(&engine->registerCache,
                                              SENSORARRAY_ADS_OWNER_MATRIX);
    if (engine->transitionSensitiveFrames > 0u) {
        engine->transitionSensitiveFrames--;
    }
    coherent = sensorarrayAdsMatrixCompleteCrcDisposition(
        engine, frame, crcRejectedMask, coherent);
    if (!coherent && abortError == ESP_OK) {
        abortError = crcRejectedMask != 0u ? ESP_ERR_INVALID_CRC : ESP_ERR_INVALID_RESPONSE;
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
    engine->faultContext.acquired = frame->measurement.freshMask != 0u;
    sensorarrayAdsMatrixFinishFrame(frame, frame->activeRows, frameStartUs, coherent);
    if (crcRejectedMask != 0u || abortError != ESP_OK) {
        printf("ADSDISPOSITION,seq=%lu,boundedCrc=%u,crcRejected=%016llX,acquired=%016llX,expected=%016llX,allCrcRun=%u,limit=%u,firstError=0x%x,abortError=0x%x,scope=ads_sweep\n",
            (unsigned long)frame->sequence, frame->boundedCrcFailure ? 1u : 0u,
            (unsigned long long)crcRejectedMask, (unsigned long long)frame->acquiredMask,
            (unsigned long long)frame->expectedMask, engine->allCrcRejectedSweeps,
            SENSORARRAY_ADS_MATRIX_ALL_CRC_REJECTED_LIMIT, firstErr, abortError);
    }
    sensorarrayAdsMatrixLogCrcEvidence(engine, frame->sequence, firstErr);
    if (frame->measurement.validMask != 0u) {
        sensorarrayAdsGapNoteFirstValidFrame(frame->frameEndUs);
    }
    return firstErr;
}

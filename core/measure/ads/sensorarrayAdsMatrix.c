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

static esp_err_t sensorarrayAdsMatrixReadFresh(ads126xAdcHandle_t *ads,
                                               sensorarrayAdsMatrixSample_t *outSample)
{
    if (!ads || !outSample) {
        return ESP_ERR_INVALID_ARG;
    }
    memset(outSample, 0, sizeof(*outSample));
    outSample->generationStart = ads126xAdcGetDrdyGeneration(ads);
    esp_err_t err = ads126xAdcWaitDrdyGenerationUs(
        ads,
        outSample->generationStart,
        (uint32_t)CONFIG_SENSORARRAY_ADS_DRDY_TIMEOUT_US,
        &outSample->generationEnd);
    if (err != ESP_OK) {
        return err;
    }
    err = ads126xAdcReadAdc1RawDmaReady(ads,
                                        &outSample->raw,
                                        &outSample->status,
                                        NULL);
    if (err != ESP_OK) {
        return err;
    }
    if (outSample->generationEnd == outSample->generationStart ||
        !ads126xAdcStatusByteHasAdc1NewData(ads, outSample->status)) {
        return ESP_ERR_INVALID_RESPONSE;
    }
    outSample->differentialUv = ads126xAdcRawToMicrovolts(ads, outSample->raw);
    return ESP_OK;
}

static esp_err_t sensorarrayAdsMatrixPrepareConversion(sensorarrayAdsMatrixEngine_t *engine,
                                                       uint8_t muxp,
                                                       uint8_t muxn,
                                                       uint8_t gain,
                                                       bool pgaBypassed)
{
    ads126xAdcHandle_t *ads = &engine->state->ads;
    esp_err_t err = ads126xAdcStopAdc1(ads);
    engine->state->adsAdc1Running = false;
    if (err == ESP_OK) {
        err = pgaBypassed ? ads126xAdcSetPgaBypass(ads, true) :
                           ads126xAdcSetPgaGain(ads, gain);
    }
    if (err == ESP_OK) {
        err = ads126xAdcSetInputMuxVerified(ads, muxp, muxn);
    }
    if (err == ESP_OK) {
        ads126xAdcClearDrdyNotifications(ads);
        err = ads126xAdcStartAdc1(ads);
    }
    if (err == ESP_OK) {
        engine->state->adsAdc1Running = true;
        esp_rom_delay_us(CONFIG_SENSORARRAY_ADS_MATRIX_MUX_SETTLE_US);
        for (uint8_t discard = 0u;
             discard < (uint8_t)CONFIG_SENSORARRAY_ADS_MATRIX_DISCARD_COUNT;
             ++discard) {
            sensorarrayAdsMatrixSample_t ignored;
            err = sensorarrayAdsMatrixReadFresh(ads, &ignored);
            if (err != ESP_OK) {
                break;
            }
        }
    }
    return err;
}

static esp_err_t sensorarrayAdsMatrixReadAggregate(sensorarrayAdsMatrixEngine_t *engine,
                                                   sensorarrayAdsMatrixCellResult_t *outResult)
{
    uint8_t sampleCount = (uint8_t)CONFIG_SENSORARRAY_ADS_MATRIX_OVERSAMPLE;
    if (sampleCount < 1u) {
        sampleCount = 1u;
    } else if (sampleCount > 9u) {
        sampleCount = 9u;
    }
    sensorarrayAdsMatrixSample_t samples[9] = {0};
    int32_t differentialUv[9] = {0};
    int32_t rawCode[9] = {0};
    uint8_t status = 0u;
    esp_err_t err = ESP_OK;
    for (uint8_t index = 0u; index < sampleCount; ++index) {
        err = sensorarrayAdsMatrixReadFresh(&engine->state->ads, &samples[index]);
        if (err != ESP_OK) {
            return err;
        }
        differentialUv[index] = samples[index].differentialUv;
        rawCode[index] = samples[index].raw;
        status |= samples[index].status;
    }
    int32_t medianUv = 0;
    int32_t medianRaw = 0;
    bool stable = sensorarrayAdsMathSamplesStable(
        differentialUv,
        sampleCount,
        (uint32_t)CONFIG_SENSORARRAY_ADS_MATRIX_MAX_SPREAD_UV,
        &medianUv);
    (void)sensorarrayAdsMathSamplesStable(rawCode, sampleCount, UINT32_MAX, &medianRaw);
    outResult->raw = medianRaw;
    outResult->differentialUv = medianUv;
    outResult->status = status;
    outResult->fresh = true;
    outResult->stable = stable;
    return ESP_OK;
}

static esp_err_t sensorarrayAdsMatrixAcquireWithBoundedRetry(
    sensorarrayAdsMatrixEngine_t *engine,
    uint8_t muxp,
    uint8_t muxn,
    uint8_t gain,
    bool pgaBypassed,
    uint8_t *ioRetries,
    sensorarrayAdsMatrixCellResult_t *outSample)
{
    if (!engine || !ioRetries || !outSample) {
        return ESP_ERR_INVALID_ARG;
    }
    for (;;) {
        esp_err_t err = sensorarrayAdsMatrixPrepareConversion(
            engine, muxp, muxn, gain, pgaBypassed);
        if (err == ESP_OK) {
            err = sensorarrayAdsMatrixReadAggregate(engine, outSample);
        }
        bool retryable = err == ESP_ERR_TIMEOUT || err == ESP_ERR_INVALID_RESPONSE;
        if (err == ESP_OK || !retryable ||
            *ioRetries >= (uint8_t)CONFIG_SENSORARRAY_ADS_MATRIX_IO_RETRY_COUNT) {
            return err;
        }
        (*ioRetries)++;
    }
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
                                                 sensorarrayAdsMatrixCellResult_t *outResult)
{
    if (!engine || !rail || !rail->valid || !outResult) {
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

    uint8_t gain = 1u;
    bool pgaBypassed = false;
    uint8_t ioRetries = 0u;
    (void)sensorarrayAdsGainCacheGet(&engine->gainCache,
                                     engine->mode,
                                     (uint8_t)cellIndex,
                                     &gain);
    for (uint8_t attempt = 0u;
         attempt < (uint8_t)CONFIG_SENSORARRAY_ADS_AUTORANGE_MAX_ATTEMPTS;
         ++attempt) {
        outResult->attempts = (uint8_t)(attempt + 1u);
        sensorarrayAdsMatrixCellResult_t sample = {0};
        esp_err_t err = sensorarrayAdsMatrixAcquireWithBoundedRetry(
            engine,
            muxp,
            muxn,
            gain,
            pgaBypassed,
            &ioRetries,
            &sample);
        if (err != ESP_OK) {
            outResult->ioRetries = ioRetries;
            outResult->error = sensorarrayAdsMatrixIoError(err);
            return err;
        }
        sample.gain = pgaBypassed ? 0u : gain;
        sample.pgaBypassed = pgaBypassed;
        sample.attempts = outResult->attempts;
        sample.gainChanges = outResult->gainChanges;
        sample.ioRetries = ioRetries;
        *outResult = sample;

        if ((sample.status & ADS126X_STATUS_RESET_OCCURRED) != 0u) {
            /* A reset invalidates register, reference, calibration and PGA
             * assumptions even when the conversion carries a data-ready bit. */
            outResult->error = SENSORARRAY_CELL_ERROR_READBACK;
            sensorarrayAdsGainCacheInvalidate(&engine->gainCache);
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
        if (pgaBypassed) {
            int64_t marginUv = CONFIG_SENSORARRAY_ADS_BYPASS_INPUT_MARGIN_UV;
            int64_t lowerUv = (int64_t)rail->avssUv - marginUv;
            int64_t upperUv = (int64_t)rail->avddUv + marginUv;
            bool inputsSafe = nodeUv64 >= lowerUv && nodeUv64 <= upperUv &&
                              (int64_t)rail->aincomUv >= lowerUv &&
                              (int64_t)rail->aincomUv <= upperUv;
            if ((sample.status & ADS126X_STATUS_REFERENCE_ALARM) != 0u) {
                outResult->error = SENSORARRAY_CELL_ERROR_REFERENCE_ALARM;
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
                } else {
                    outResult->error = inputsSafe ? SENSORARRAY_CELL_ERROR_SATURATED :
                                                    SENSORARRAY_CELL_ERROR_RANGE;
                }
                return ESP_ERR_INVALID_RESPONSE;
            }
            if (!outResult->stable) {
                outResult->error = SENSORARRAY_CELL_ERROR_UNSTABLE;
                return ESP_ERR_INVALID_RESPONSE;
            }
            outResult->error = SENSORARRAY_CELL_ERROR_NONE;
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
        sensorarrayAdsAutoRangeDecision_t decision = sensorarrayAdsAutoRangeDecide(
            &s_sensorarrayAdsAutoRangeConfig,
            &rangeInput);
        if (magnitude >= saturationLimit && decision.action == SENSORARRAY_ADS_AUTORANGE_KEEP) {
            decision.action = SENSORARRAY_ADS_AUTORANGE_FAIL;
            decision.error = SENSORARRAY_CELL_ERROR_SATURATED;
        }
        if (decision.action == SENSORARRAY_ADS_AUTORANGE_INCREASE ||
            decision.action == SENSORARRAY_ADS_AUTORANGE_DECREASE) {
            gain = decision.nextGain;
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
            pgaBypassed = true;
            outResult->gainChanges++;
            continue;
        }
        if (decision.action == SENSORARRAY_ADS_AUTORANGE_FAIL) {
            outResult->error = decision.error;
            sensorarrayAdsGainCacheNoteOverrange(&engine->gainCache,
                                                 engine->mode,
                                                 (uint8_t)cellIndex);
            return ESP_ERR_INVALID_RESPONSE;
        }
        if (!outResult->stable) {
            outResult->error = SENSORARRAY_CELL_ERROR_UNSTABLE;
            return ESP_ERR_INVALID_RESPONSE;
        }
        outResult->error = SENSORARRAY_CELL_ERROR_NONE;
        sensorarrayAdsGainCacheStore(&engine->gainCache,
                                     engine->mode,
                                     (uint8_t)cellIndex,
                                     gain);
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
    frame->freshFrame = coherent;
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
    sensorarrayAdsGainCacheInit(&engine->gainCache);
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
    sensorarrayAdsGainCacheInvalidate(&engine->gainCache);
    engine->railFingerprintValid = false;
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
        sensorarrayAdsGainCacheInvalidate(&engine->gainCache);
        engine->railFingerprintValid = false;
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
        sensorarrayAdsGainCacheInvalidate(&engine->gainCache);
    }
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

    sensorarrayAdsRailSplit_t rail = {0};
    if (!sensorarrayAdsMatrixGetRail(engine, &rail, &frame->adsGap)) {
        sensorarrayAdsGainCacheInvalidate(&engine->gainCache);
        engine->railFingerprintValid = false;
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
    /* PGA choices are only valid for the rail/reference conditions under
     * which their common-mode and output-swing checks were made. Treat a new
     * measured rail value as a new fingerprint; this favours safety over
     * retaining a stale per-cell shortcut when the analogue supply changes. */
    if (!engine->railFingerprintValid || engine->lastRailUv != frame->adsGap.railUv) {
        sensorarrayAdsGainCacheInvalidate(&engine->gainCache);
        engine->lastRailUv = frame->adsGap.railUv;
        engine->railFingerprintValid = true;
    }
    sensorarrayRouteControllerUpdateRailSnapshot(engine->routeController, &rail);
    frame->measurement.avddUv = rail.avddUv;
    frame->measurement.avssUv = rail.avssUv;
    frame->measurement.railAgeFrames = rail.ageFrames;
    frame->measurement.railValid = rail.valid;
    frame->measurement.referenceValid = true;
    frame->measurement.matrixReferenceUv =
        engine->mode == SENSORARRAY_MEASUREMENT_MODE_RESISTANCE ?
        rail.avssUv + (int32_t)engine->calibration.matrixReferenceSpanUv : 0;

    esp_err_t firstErr = ESP_OK;
    bool coherent = true;
    for (uint8_t row = 1u; row <= frame->activeRows; ++row) {
        esp_err_t rowErr = sensorarrayRouteControllerSelectRow(engine->routeController, row);
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
        for (uint8_t dLine = 1u; dLine <= SENSORARRAY_MATRIX_COLS; ++dLine) {
            size_t index = sensorarrayMatrixIndex(row, dLine);
            sensorarrayAdsMatrixCellResult_t result = {0};
            esp_err_t err = sensorarrayAdsMatrixMeasureCell(engine,
                                                             &rail,
                                                             row,
                                                             dLine,
                                                             &result);
            frame->measurement.rawCode[index] = result.raw;
            frame->measurement.nodeUv[index] = result.nodeUv;
            frame->measurement.pgaGain[index] = result.gain;
            frame->measurement.gainChangeCount += result.gainChanges;
            frame->measurement.autorangeAttemptCount += result.attempts;
            frame->measurement.ioRetryCount += result.ioRetries;
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
                sensorarrayAdsMatrixSetCellValid(frame,
                                                 index,
                                                 resistance.resistanceMilliohms,
                                                 &result);
            }
        }
    }
    (void)ads126xAdcStopAdc1(&engine->state->ads);
    engine->state->adsAdc1Running = false;
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
    sensorarrayAdsMatrixFinishFrame(frame, frame->activeRows, frameStartUs, coherent);
    return firstErr;
}

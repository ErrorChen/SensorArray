#include "sensorarrayVoltageScan.h"

#include <string.h>

#include "esp_rom_sys.h"
#include "esp_timer.h"

#include "sensorarrayBoardMap.h"
#include "sensorarrayConfig.h"
#include "tmuxSwitch.h"

#ifndef CONFIG_SENSORARRAY_VOLTAGE_SCAN_DISCARD_FIRST
#define CONFIG_SENSORARRAY_VOLTAGE_SCAN_DISCARD_FIRST 0
#endif

static uint32_t s_voltageFrameSequence = 0u;
static uint32_t s_fastMuxSettleUs = CONFIG_SENSORARRAY_VOLTAGE_SCAN_MUX_SETTLE_US;
static bool s_fastVerifiedInpmuxForced = false;

static void sensorarrayVoltageScanDelayUs(uint32_t delayUs)
{
    if (delayUs > 0u) {
        esp_rom_delay_us(delayUs);
    }
}

static bool sensorarrayVoltageScanGainValid(uint8_t gain)
{
    return gain == ADS126X_GAIN_1 ||
           gain == ADS126X_GAIN_2 ||
           gain == ADS126X_GAIN_4 ||
           gain == ADS126X_GAIN_8 ||
           gain == ADS126X_GAIN_16 ||
           gain == ADS126X_GAIN_32;
}

void sensorarrayVoltageScanSetFastRuntimeOptions(uint32_t muxSettleUs, bool verifiedInpmuxForced)
{
    s_fastMuxSettleUs = muxSettleUs;
    s_fastVerifiedInpmuxForced = verifiedInpmuxForced;
}

static void sensorarrayVoltageScanPerfAdd(uint64_t *totalUs, uint32_t *maxUs, uint32_t sampleUs)
{
    if (totalUs) {
        *totalUs += sampleUs;
    }
    if (maxUs && sampleUs > *maxUs) {
        *maxUs = sampleUs;
    }
}

static sensorarrayStatusCode_t sensorarrayVoltageScanStatusFromAdsErr(esp_err_t err)
{
    switch (err) {
    case ESP_ERR_TIMEOUT:
        return SENSORARRAY_STATUS_ADS_DRDY_TIMEOUT;
    case ESP_ERR_INVALID_CRC:
        return SENSORARRAY_STATUS_ADS_CRC_FAIL;
    case ESP_ERR_INVALID_RESPONSE:
        return SENSORARRAY_STATUS_ADS_STATUS_BYTE_BAD;
    default:
        return SENSORARRAY_STATUS_ADS_SPI_FAIL;
    }
}

static uint32_t sensorarrayVoltageScanFlagsFromStatus(sensorarrayStatusCode_t code)
{
    switch (code) {
    case SENSORARRAY_STATUS_ADS_DRDY_TIMEOUT:
        return SENSORARRAY_FRAME_FLAG_ADS_ERROR | SENSORARRAY_FRAME_FLAG_DRDY_TIMEOUT;
    case SENSORARRAY_STATUS_ADS_SPI_FAIL:
    case SENSORARRAY_STATUS_ADS_INPMUX_WRITE_FAIL:
    case SENSORARRAY_STATUS_ADS_DIRECT_READ_FAIL:
        return SENSORARRAY_FRAME_FLAG_ADS_ERROR | SENSORARRAY_FRAME_FLAG_SPI_ERROR;
    case SENSORARRAY_STATUS_TMUX_ROUTE_FAIL:
    case SENSORARRAY_STATUS_TMUX_SOURCE_FAIL:
        return SENSORARRAY_FRAME_FLAG_TMUX_ERROR;
    case SENSORARRAY_STATUS_ADS_REF_POLICY_MISMATCH:
        return SENSORARRAY_FRAME_FLAG_REF_POLICY_ERROR | SENSORARRAY_FRAME_FLAG_FATAL;
    default:
        return SENSORARRAY_FRAME_FLAG_ADS_ERROR;
    }
}

static void sensorarrayVoltageScanRecordFrameStatus(sensorarrayVoltageFrame_t *frame,
                                                    sensorarrayStatusCounters_t *status,
                                                    sensorarrayStatusCode_t code)
{
    if (!frame || code == SENSORARRAY_STATUS_OK) {
        return;
    }
    uint32_t flags = sensorarrayVoltageScanFlagsFromStatus(code);
    if (frame->firstStatusCode == SENSORARRAY_STATUS_OK) {
        frame->firstStatusCode = (uint32_t)code;
    }
    frame->lastStatusCode = (uint32_t)code;
    frame->statusFlags |= flags;
    if (status) {
        sensorarrayStatusRecord(status, code, flags);
    }
}

esp_err_t sensorarrayVoltageScanApplyRouteFastWithSource(uint8_t sColumn,
                                                         uint8_t dLine,
                                                         tmux1108Source_t swSource,
                                                         uint32_t rowSettleUs,
                                                         uint32_t pathSettleUs)
{
    if (sColumn < 1u || sColumn > SENSORARRAY_VOLTAGE_SCAN_ROWS ||
        dLine < 1u || dLine > SENSORARRAY_VOLTAGE_SCAN_COLS) {
        return ESP_ERR_INVALID_ARG;
    }
    if (swSource != TMUX1108_SOURCE_GND && swSource != TMUX1108_SOURCE_REF) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = tmuxSwitchSelectRow((uint8_t)(sColumn - 1u));
    if (err != ESP_OK) {
        return err;
    }
    sensorarrayVoltageScanDelayUs(rowSettleUs);

    int selaLevel = 0;
    if (!sensorarrayBoardMapSelaRouteToGpioLevel(SENSORARRAY_SELA_ROUTE_ADS1263, &selaLevel)) {
        return ESP_ERR_INVALID_STATE;
    }

    err = tmux1134SelectSelALevel(selaLevel != 0);
    if (err != ESP_OK) {
        return err;
    }

    err = tmux1134SelectSelBLevel(false);
    if (err != ESP_OK) {
        return err;
    }

    err = tmuxSwitchSet1108Source(swSource);
    if (err != ESP_OK) {
        return err;
    }

    err = tmux1134SetEnLogicalState(true);
    if (err != ESP_OK) {
        return err;
    }

    sensorarrayVoltageScanDelayUs(pathSettleUs);
    return ESP_OK;
}

esp_err_t sensorarrayVoltageScanApplyRouteFast(uint8_t sColumn,
                                               uint8_t dLine,
                                               uint32_t rowSettleUs,
                                               uint32_t pathSettleUs)
{
    return sensorarrayVoltageScanApplyRouteFastWithSource(sColumn,
                                                          dLine,
                                                          TMUX1108_SOURCE_REF,
                                                          rowSettleUs,
                                                          pathSettleUs);
}

esp_err_t sensorarrayVoltageScanOneFrameFastAds(ads126xAdcHandle_t *ads,
                                                tmux1108Source_t swSource,
                                                sensorarrayVoltageFrame_t *outFrame,
                                                sensorarrayStatusCounters_t *status,
                                                sensorarrayPerfCounters_t *perf)
{
    if (!ads || !outFrame) {
        return ESP_ERR_INVALID_ARG;
    }
    if (swSource != TMUX1108_SOURCE_GND && swSource != TMUX1108_SOURCE_REF) {
        return ESP_ERR_INVALID_ARG;
    }

    if (status) {
        sensorarrayStatusResetFrame(status);
    }

    sensorarrayVoltageFrame_t frame = {
        .sequence = s_voltageFrameSequence++,
        .timestampUs = (uint64_t)esp_timer_get_time(),
        .validMask = 0u,
        .adsDr = ads->dataRateDr,
    };

    bool originalFastMux = ads->fastOptions.fastInpmuxWrite;
    if (s_fastVerifiedInpmuxForced) {
        ads->fastOptions.fastInpmuxWrite = false;
    }

    esp_err_t frameErr = ESP_OK;
    esp_err_t err = ads126xAdcBeginFastFrame(ads);
    if (err != ESP_OK) {
        sensorarrayVoltageScanRecordFrameStatus(&frame, status, SENSORARRAY_STATUS_SPI_BUS_ACQUIRE_FAIL);
        frameErr = err;
        goto finish;
    }

    for (uint8_t s = 1u; s <= SENSORARRAY_VOLTAGE_SCAN_ROWS; ++s) {
        int64_t routeStartUs = esp_timer_get_time();
        esp_err_t routeErr = sensorarrayVoltageScanApplyRouteFastWithSource(
            s,
            1u,
            swSource,
            (uint32_t)CONFIG_SENSORARRAY_VOLTAGE_SCAN_ROW_SETTLE_US,
            (uint32_t)CONFIG_SENSORARRAY_VOLTAGE_SCAN_PATH_SETTLE_US);
        uint32_t routeElapsedUs = (uint32_t)(esp_timer_get_time() - routeStartUs);
        if (perf) {
            sensorarrayVoltageScanPerfAdd(&perf->routeTotalUs, &perf->routeMaxUs, routeElapsedUs);
        }

        if (routeErr != ESP_OK) {
            frameErr = routeErr;
            sensorarrayVoltageScanRecordFrameStatus(&frame, status, SENSORARRAY_STATUS_TMUX_ROUTE_FAIL);
            for (uint8_t d = 1u; d <= SENSORARRAY_VOLTAGE_SCAN_COLS; ++d) {
                uint8_t row = (uint8_t)(s - 1u);
                uint8_t col = (uint8_t)(d - 1u);
                frame.err[row][col] = routeErr;
                frame.gain[row][col] = ads->pgaGain;
                frame.pointStatus[row][col] = SENSORARRAY_STATUS_TMUX_ROUTE_FAIL;
            }
            continue;
        }

        for (uint8_t d = 1u; d <= SENSORARRAY_VOLTAGE_SCAN_COLS; ++d) {
            uint8_t row = (uint8_t)(s - 1u);
            uint8_t col = (uint8_t)(d - 1u);
            uint8_t muxp = 0u;
            uint8_t muxn = 0u;
            if (!sensorarrayBoardMapAdsMuxForDLine(d, &muxp, &muxn)) {
                frame.err[row][col] = ESP_ERR_INVALID_ARG;
                frame.pointStatus[row][col] = SENSORARRAY_STATUS_INTERNAL_ASSERT_FAIL;
                frameErr = ESP_ERR_INVALID_ARG;
                sensorarrayVoltageScanRecordFrameStatus(&frame, status, SENSORARRAY_STATUS_INTERNAL_ASSERT_FAIL);
                continue;
            }

            ads126xAdcVoltageReadConfig_t readCfg = {
                .muxp = muxp,
                .muxn = muxn,
                .stopBeforeMuxChange = false,
                .startAfterMuxChange = false,
                .settleAfterMuxUs = s_fastMuxSettleUs,
                .discardFirst = (CONFIG_SENSORARRAY_VOLTAGE_SCAN_DISCARD_FIRST != 0),
                .oversampleCount = (uint8_t)CONFIG_SENSORARRAY_VOLTAGE_SCAN_OVERSAMPLE,
                .drdyTimeoutUs = ads->fastOptions.drdyTimeoutUs,
            };

            ads126xAdcVoltageSample_t sample = {0};
            ads126xAdcFastPerfCounters_t adsPerf = {0};
            bool directFailedBefore = ads->fastDirectReadFailed;
            esp_err_t readErr = ads126xAdcReadVoltageMicrovoltsFast2(ads, &readCfg, &sample, &adsPerf);
            bool directFailedAfter = ads->fastDirectReadFailed;

            frame.microvolts[row][col] = (readErr == ESP_OK) ? sample.microvolts : 0;
            frame.raw[row][col] = sample.rawCode;
            frame.gain[row][col] = sample.gain;
            frame.err[row][col] = readErr;
            frame.clipped[row][col] = sample.clippedOrNearFullScale;
            if (readErr == ESP_OK) {
                frame.validMask |= (1ULL << ((uint32_t)row * SENSORARRAY_VOLTAGE_SCAN_COLS + (uint32_t)col));
                frame.pointStatus[row][col] = SENSORARRAY_STATUS_OK;
                if (sample.clippedOrNearFullScale) {
                    frame.statusFlags |= SENSORARRAY_FRAME_FLAG_CLIPPED;
                }
            } else {
                sensorarrayStatusCode_t code = sensorarrayVoltageScanStatusFromAdsErr(readErr);
                if (code == SENSORARRAY_STATUS_ADS_SPI_FAIL && adsPerf.inpmuxWriteCount > 0u &&
                    adsPerf.adcReadTotalUs == 0u) {
                    code = SENSORARRAY_STATUS_ADS_INPMUX_WRITE_FAIL;
                }
                frame.pointStatus[row][col] = (uint32_t)code;
                sensorarrayVoltageScanRecordFrameStatus(&frame, status, code);
                frameErr = readErr;
            }

            if (!directFailedBefore && directFailedAfter) {
                sensorarrayVoltageScanRecordFrameStatus(&frame, status, SENSORARRAY_STATUS_ADS_DIRECT_READ_FAIL);
            }

            if (perf) {
                perf->pointsScanned++;
                perf->spiTransactionCount += adsPerf.spiTransferCount;
                perf->inpmuxWriteTotalUs += adsPerf.inpmuxWriteTotalUs;
                if (adsPerf.inpmuxWriteMaxUs > perf->inpmuxWriteMaxUs) {
                    perf->inpmuxWriteMaxUs = adsPerf.inpmuxWriteMaxUs;
                }
                perf->drdyWaitTotalUs += adsPerf.drdyWaitTotalUs;
                if (adsPerf.drdyWaitMaxUs > perf->drdyWaitMaxUs) {
                    perf->drdyWaitMaxUs = adsPerf.drdyWaitMaxUs;
                }
                perf->adcReadTotalUs += adsPerf.adcReadTotalUs;
                if (adsPerf.adcReadMaxUs > perf->adcReadMaxUs) {
                    perf->adcReadMaxUs = adsPerf.adcReadMaxUs;
                }
                perf->spiTransferTotalUs += adsPerf.spiTransferTotalUs;
                if (adsPerf.spiTransferMaxUs > perf->spiTransferMaxUs) {
                    perf->spiTransferMaxUs = adsPerf.spiTransferMaxUs;
                }
            }
        }
    }

    err = ads126xAdcEndFastFrame(ads);
    if (err != ESP_OK) {
        sensorarrayVoltageScanRecordFrameStatus(&frame, status, SENSORARRAY_STATUS_SPI_BUS_RELEASE_FAIL);
        if (frameErr == ESP_OK) {
            frameErr = err;
        }
    }

finish:
    ads->fastOptions.fastInpmuxWrite = originalFastMux;
    frame.scanDurationUs = (uint32_t)((uint64_t)esp_timer_get_time() - frame.timestampUs);
    frame.droppedFrames = status ? status->droppedFrameCount : 0u;
    frame.outputDecimatedFrames = status ? status->outputDecimatedFrameCount : 0u;
    if (status) {
        status->frameStatusFlags = frame.statusFlags;
    }
    if (perf) {
        perf->framesScanned++;
        sensorarrayVoltageScanPerfAdd(&perf->scanTotalUs, &perf->scanMaxUs, frame.scanDurationUs);
    }
    memcpy(outFrame, &frame, sizeof(frame));
    return frameErr;
}

esp_err_t sensorarrayVoltageScanOneFrameWithSource(ads126xAdcHandle_t *ads,
                                                   const uint8_t gainTable[SENSORARRAY_VOLTAGE_SCAN_ROWS]
                                                                          [SENSORARRAY_VOLTAGE_SCAN_COLS],
                                                   tmux1108Source_t swSource,
                                                   sensorarrayVoltageFrame_t *outFrame)
{
    if (!ads || !outFrame) {
        return ESP_ERR_INVALID_ARG;
    }
    if (swSource != TMUX1108_SOURCE_GND && swSource != TMUX1108_SOURCE_REF) {
        return ESP_ERR_INVALID_ARG;
    }

    sensorarrayVoltageFrame_t frame = {
        .sequence = s_voltageFrameSequence++,
        .timestampUs = (uint64_t)esp_timer_get_time(),
    };

    esp_err_t frameErr = ESP_OK;
    for (uint8_t s = 1u; s <= SENSORARRAY_VOLTAGE_SCAN_ROWS; ++s) {
        esp_err_t routeErr = sensorarrayVoltageScanApplyRouteFastWithSource(
            s,
            1u,
            swSource,
            (uint32_t)CONFIG_SENSORARRAY_VOLTAGE_SCAN_ROW_SETTLE_US,
            (uint32_t)CONFIG_SENSORARRAY_VOLTAGE_SCAN_PATH_SETTLE_US);
        if (routeErr != ESP_OK) {
            frameErr = routeErr;
            for (uint8_t d = 1u; d <= SENSORARRAY_VOLTAGE_SCAN_COLS; ++d) {
                frame.err[s - 1u][d - 1u] = routeErr;
                frame.gain[s - 1u][d - 1u] = ads->pgaGain;
            }
            continue;
        }

        for (uint8_t d = 1u; d <= SENSORARRAY_VOLTAGE_SCAN_COLS; ++d) {
            uint8_t row = (uint8_t)(s - 1u);
            uint8_t col = (uint8_t)(d - 1u);
            uint8_t muxp = 0u;
            uint8_t muxn = 0u;
            if (!sensorarrayBoardMapAdsMuxForDLine(d, &muxp, &muxn)) {
                frame.err[row][col] = ESP_ERR_INVALID_ARG;
                frameErr = ESP_ERR_INVALID_ARG;
                continue;
            }

            ads126xAdcVoltageReadConfig_t readCfg = {
                .muxp = muxp,
                .muxn = muxn,
                .stopBeforeMuxChange = false,
                .startAfterMuxChange = false,
                .settleAfterMuxUs = (uint32_t)CONFIG_SENSORARRAY_VOLTAGE_SCAN_MUX_SETTLE_US,
                .discardFirst = (CONFIG_SENSORARRAY_VOLTAGE_SCAN_DISCARD_FIRST != 0),
                .oversampleCount = (uint8_t)CONFIG_SENSORARRAY_VOLTAGE_SCAN_OVERSAMPLE,
                .drdyTimeoutUs = (uint32_t)CONFIG_SENSORARRAY_VOLTAGE_SCAN_DRDY_TIMEOUT_US,
            };

#if CONFIG_SENSORARRAY_VOLTAGE_SCAN_AUTO_GAIN_PER_POINT
            ads126xAdcAutoGainConfig_t gainCfg = {
                .minGain = ADS126X_GAIN_1,
                .maxGain = ADS126X_GAIN_32,
                .initialGain = (uint8_t)CONFIG_SENSORARRAY_VOLTAGE_SCAN_DEFAULT_GAIN,
                .headroomPercent = (uint8_t)CONFIG_SENSORARRAY_VOLTAGE_SCAN_AUTO_GAIN_HEADROOM_PERCENT,
                .maxIterations = 4u,
                .settleAfterGainChangeUs = (uint32_t)CONFIG_SENSORARRAY_VOLTAGE_SCAN_MUX_SETTLE_US,
                .keepConfiguredOnSuccess = true,
            };
            ads126xAdcAutoGainResult_t gainResult = {0};
            esp_err_t autoGainErr = ads126xAdcSelectAutoGainForVoltage(ads, &readCfg, &gainCfg, &gainResult);
            if (autoGainErr != ESP_OK) {
                frame.err[row][col] = autoGainErr;
                frame.gain[row][col] = ads->pgaGain;
                frameErr = autoGainErr;
                continue;
            }
#else
            uint8_t targetGain = ads->pgaGain;
            if (gainTable && sensorarrayVoltageScanGainValid(gainTable[row][col])) {
                targetGain = gainTable[row][col];
            }
            if (targetGain != ads->pgaGain) {
                esp_err_t gainErr = ads126xAdcConfigureVoltageMode(ads,
                                                                   targetGain,
                                                                   ads->dataRateDr,
                                                                   ads->enableStatusByte,
                                                                   ads->crcMode != ADS126X_CRC_OFF);
                if (gainErr != ESP_OK) {
                    frame.err[row][col] = gainErr;
                    frame.gain[row][col] = targetGain;
                    frameErr = gainErr;
                    continue;
                }
            }
#endif
            ads126xAdcVoltageSample_t sample = {0};
            esp_err_t readErr = ads126xAdcReadVoltageMicrovoltsFast(ads, &readCfg, &sample);
            frame.microvolts[row][col] = sample.microvolts;
            frame.raw[row][col] = sample.rawCode;
            frame.gain[row][col] = sample.gain;
            frame.err[row][col] = readErr;
            frame.clipped[row][col] = sample.clippedOrNearFullScale;
            if (readErr != ESP_OK) {
                frameErr = readErr;
            }
        }
    }

    uint64_t frameEndUs = (uint64_t)esp_timer_get_time();
    frame.scanDurationUs = (uint32_t)(frameEndUs - frame.timestampUs);
    memcpy(outFrame, &frame, sizeof(frame));
    return frameErr;
}

esp_err_t sensorarrayVoltageScanOneFrame(ads126xAdcHandle_t *ads,
                                         const uint8_t gainTable[SENSORARRAY_VOLTAGE_SCAN_ROWS]
                                                                [SENSORARRAY_VOLTAGE_SCAN_COLS],
                                         sensorarrayVoltageFrame_t *outFrame)
{
    return sensorarrayVoltageScanOneFrameWithSource(ads,
                                                    gainTable,
                                                    TMUX1108_SOURCE_REF,
                                                    outFrame);
}

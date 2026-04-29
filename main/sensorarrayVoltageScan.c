#include "sensorarrayVoltageScan.h"

#include <stdio.h>
#include <string.h>

#include "esp_rom_sys.h"
#include "esp_timer.h"

#include "sensorarrayBoardMap.h"
#include "sensorarrayConfig.h"
#include "tmuxSwitch.h"

static uint32_t s_voltageFrameSequence = 0u;

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

static const char *sensorarrayVoltageScanSourceName(tmux1108Source_t source)
{
    return (source == TMUX1108_SOURCE_REF) ? "REF" : "GND";
}

static const char *sensorarrayVoltageScanPathName(tmux1108Source_t source)
{
    return (source == TMUX1108_SOURCE_REF) ? "RESISTIVE" : "PIEZO_VOLTAGE";
}

static const char *sensorarrayVoltageScanRouteTag(tmux1108Source_t source)
{
    return (source == TMUX1108_SOURCE_REF) ? "DBGRES_ROUTE" : "DBGPIEZO_ROUTE";
}

static const char *sensorarrayVoltageScanAssertStage(tmux1108Source_t source)
{
    return (source == TMUX1108_SOURCE_REF) ? "resistance_voltage_scan" : "piezo_voltage_scan";
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

    /*
     * Confirmed SensorArray board polarity:
     *   SW LOW  -> REF
     *   SW HIGH -> GND
     *
     * Piezo voltage scan requires GND at the selected TMUX1108 source,
     * so this mode must command TMUX1108_SOURCE_GND and observe SW HIGH.
     */
    err = tmuxSwitchAssert1108Source(swSource, sensorarrayVoltageScanAssertStage(swSource));
    if (err != ESP_OK) {
        return err;
    }

    err = tmux1134SetEnLogicalState(true);
    if (err != ESP_OK) {
        return err;
    }

    sensorarrayVoltageScanDelayUs(pathSettleUs);

    tmuxSwitchControlState_t ctrl = {0};
    bool haveCtrl = (tmuxSwitchGetControlState(&ctrl) == ESP_OK);
    int expectedSwLevel = tmuxSwitch1108SourceToSwLevel(swSource);
    printf("%s,timestamp_us=%lld,s=S%u,d=D%u,path=%s,adc=ADS126x,swSource=%s,expectedSwLevel=%d,"
           "cmdSwLevel=%d,obsSwLevel=%d,cmdA0=%d,cmdA1=%d,cmdA2=%d,obsA0=%d,obsA1=%d,obsA2=%d,"
           "cmdSELA=%d,cmdSELB=%d,obsSELA=%d,obsSELB=%d,status=route_locked\n",
           sensorarrayVoltageScanRouteTag(swSource),
           (long long)esp_timer_get_time(),
           (unsigned)sColumn,
           (unsigned)dLine,
           sensorarrayVoltageScanPathName(swSource),
           sensorarrayVoltageScanSourceName(swSource),
           expectedSwLevel,
           haveCtrl ? ctrl.cmdSwLevel : -1,
           haveCtrl ? ctrl.obsSwLevel : -1,
           haveCtrl ? ctrl.cmdA0Level : -1,
           haveCtrl ? ctrl.cmdA1Level : -1,
           haveCtrl ? ctrl.cmdA2Level : -1,
           haveCtrl ? ctrl.obsA0Level : -1,
           haveCtrl ? ctrl.obsA1Level : -1,
           haveCtrl ? ctrl.obsA2Level : -1,
           haveCtrl ? ctrl.cmdSelaLevel : -1,
           haveCtrl ? ctrl.cmdSelbLevel : -1,
           haveCtrl ? ctrl.obsSelaLevel : -1,
           haveCtrl ? ctrl.obsSelbLevel : -1);
    return ESP_OK;
}

esp_err_t sensorarrayVoltageScanApplyRouteFast(uint8_t sColumn,
                                               uint8_t dLine,
                                               uint32_t rowSettleUs,
                                               uint32_t pathSettleUs)
{
    return sensorarrayVoltageScanApplyRouteFastWithSource(sColumn,
                                                          dLine,
                                                          TMUX1108_SOURCE_GND,
                                                          rowSettleUs,
                                                          pathSettleUs);
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
                                                    TMUX1108_SOURCE_GND,
                                                    outFrame);
}

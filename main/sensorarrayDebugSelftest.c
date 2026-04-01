#include "sensorarrayDebugSelftest.h"

#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "sensorarrayConfig.h"
#include "sensorarrayDebug.h"
#include "sensorarrayLog.h"
#include "sensorarrayMeasure.h"

void sensorarrayDebugRunAdsSelftestModeImpl(sensorarrayState_t *state)
{
    if (!state || !state->adsReady) {
        sensorarrayLogStartup("ads_selftest", ESP_ERR_INVALID_STATE, "skip_ads_unavailable", 0);
        sensorarrayDebugIdleForever("ads_unavailable");
        return;
    }

    uint8_t muxp = (uint8_t)(CONFIG_SENSORARRAY_DEBUG_ADS_FORCE_MUXP & 0x0Fu);
    uint8_t muxn = (uint8_t)(CONFIG_SENSORARRAY_DEBUG_ADS_FORCE_MUXN & 0x0Fu);
    uint8_t refmux = (uint8_t)(CONFIG_SENSORARRAY_DEBUG_ADS_FORCE_REFMUX & 0xFFu);
    uint8_t retryCount = (uint8_t)CONFIG_SENSORARRAY_DEBUG_ADS_READ_RETRY_COUNT;
    uint8_t discardCount = (uint8_t)CONFIG_SENSORARRAY_DEBUG_ADS_DISCARD_COUNT;
    uint8_t sampleCount = (uint8_t)CONFIG_SENSORARRAY_DEBUG_ADS_SAMPLE_COUNT;
    bool drdyTimedOut = false;

    sensorarrayMeasureDumpAdsKeyRegisters(state, "pre_cfg");

    esp_err_t err = ads126xAdcConfigure(&state->ads, true, false, ADS126X_CRC_OFF, 1, 0);
    sensorarrayLogStartup("ads_selftest_cfg", err, (err == ESP_OK) ? "ok" : "configure_error", 0);
    if (err != ESP_OK) {
        sensorarrayDebugIdleForever("ads_cfg_fail");
        return;
    }

    sensorarrayMeasureDumpAdsKeyRegisters(state, "post_cfg");

    if (CONFIG_SENSORARRAY_DEBUG_ADS_STOP1_BEFORE_MUX && state->adsAdc1Running) {
        err = ads126xAdcStopAdc1(&state->ads);
        sensorarrayLogStartup("ads_selftest_stop1", err, (err == ESP_OK) ? "ok" : "stop_error", 0);
        if (err != ESP_OK) {
            sensorarrayDebugIdleForever("ads_stop_fail");
            return;
        }
        state->adsAdc1Running = false;
    }

    err = ads126xAdcSetRefMux(&state->ads, refmux);
    if (err == ESP_OK) {
        state->adsRefMux = refmux;
        state->adsRefMuxValid = true;
        sensorarrayLogDbgExtraSetRefMux(refmux);
    }
    sensorarrayLogStartup("ads_selftest_refmux", err, (err == ESP_OK) ? "ok" : "refmux_error", refmux);
    if (err != ESP_OK) {
        sensorarrayDebugIdleForever("ads_refmux_fail");
        return;
    }

    err = ads126xAdcSetInputMux(&state->ads, muxp, muxn);
    sensorarrayLogStartup("ads_selftest_inpmux",
                          err,
                          (err == ESP_OK) ? "ok" : "inpmux_error",
                          ((int32_t)muxp << 8) | (int32_t)muxn);
    if (err != ESP_OK) {
        sensorarrayDebugIdleForever("ads_inpmux_fail");
        return;
    }

    if (CONFIG_SENSORARRAY_DEBUG_ADS_SETTLE_AFTER_MUX_MS > 0) {
        vTaskDelay(pdMS_TO_TICKS((uint32_t)CONFIG_SENSORARRAY_DEBUG_ADS_SETTLE_AFTER_MUX_MS));
    }

    if (CONFIG_SENSORARRAY_DEBUG_ADS_START1_BEFORE_READ || !state->adsAdc1Running) {
        err = ads126xAdcStartAdc1(&state->ads);
        sensorarrayLogStartup("ads_selftest_start1", err, (err == ESP_OK) ? "ok" : "start_error", 0);
        if (err != ESP_OK) {
            sensorarrayDebugIdleForever("ads_start_fail");
            return;
        }
        state->adsAdc1Running = true;
    }

    for (uint8_t i = 0; i < discardCount; ++i) {
        int32_t throwaway = 0;
        err = sensorarrayMeasureReadAdsRawWithRetry(state, &throwaway, retryCount, &drdyTimedOut, NULL);
        printf("DBGADSSELF,stage=discard,index=%u,raw=%ld,muxp=%u,muxn=%u,refmux=0x%02X,discardCount=%u,"
               "discardFirst=%u,drdyTimeout=%u,err=%ld,status=%s\n",
               (unsigned)i,
               (long)throwaway,
               (unsigned)muxp,
               (unsigned)muxn,
               refmux,
               (unsigned)discardCount,
               (discardCount > 0u) ? 1u : 0u,
               drdyTimedOut ? 1u : 0u,
               (long)err,
               (err == ESP_OK) ? "discard_ok" : "discard_error");
        if (err != ESP_OK) {
            sensorarrayDebugIdleForever("ads_discard_fail");
            return;
        }
    }

    for (uint8_t i = 0; i < sampleCount; ++i) {
        int32_t raw = 0;
        drdyTimedOut = false;
        err = sensorarrayMeasureReadAdsRawWithRetry(state, &raw, retryCount, &drdyTimedOut, NULL);
        int32_t uv = (err == ESP_OK) ? ads126xAdcRawToMicrovolts(&state->ads, raw) : 0;
        printf("DBGADSSELF,stage=sample,index=%u,raw=%ld,uv=%ld,muxp=%u,muxn=%u,refmux=0x%02X,discardCount=%u,"
               "discardFirst=%u,drdyTimeout=%u,err=%ld,status=%s\n",
               (unsigned)i,
               (long)raw,
               (long)uv,
               (unsigned)muxp,
               (unsigned)muxn,
               refmux,
               (unsigned)discardCount,
               (discardCount > 0u) ? 1u : 0u,
               drdyTimedOut ? 1u : 0u,
               (long)err,
               (err == ESP_OK) ? "sample_ok" : "sample_error");
    }

    sensorarrayMeasureDumpAdsKeyRegisters(state, "post_read");
    sensorarrayDebugIdleForever("ads_selftest_done");
}

void sensorarrayDebugRunFdcSelftestModeImpl(sensorarrayState_t *state)
{
    if (!state) {
        sensorarrayDebugIdleForever("fdc_state_null");
        return;
    }

    uint8_t dLine = (uint8_t)CONFIG_SENSORARRAY_DEBUG_FDC_D_LINE;
    const sensorarrayFdcDLineMap_t *fdcMap = NULL;
    sensorarrayFdcDeviceState_t *fdcState = sensorarrayMeasureGetFdcStateForDLine(state, dLine, &fdcMap);

    if (!fdcMap || !fdcState || !fdcState->ready || !fdcState->handle) {
        sensorarrayLogStartup("fdc_selftest", ESP_ERR_INVALID_STATE, "fdc_not_ready", (int32_t)dLine);
        sensorarrayDebugIdleForever("fdc_unavailable");
        return;
    }

    bool discardFirst = (CONFIG_SENSORARRAY_DEBUG_FDC_DISCARD_FIRST != 0);
    uint8_t sampleCount = (uint8_t)CONFIG_SENSORARRAY_DEBUG_FDC_SAMPLE_COUNT;
    if (sampleCount == 0u) {
        sampleCount = 1u;
    }

    uint32_t validCount = 0u;
    uint32_t i2cErrCount = 0u;
    uint32_t configUnknownCount = 0u;
    uint32_t stillSleepingCount = 0u;
    uint32_t notConvertingCount = 0u;
    uint32_t noUnreadCount = 0u;
    uint32_t zeroRawCount = 0u;
    uint32_t watchdogCount = 0u;
    uint32_t amplitudeCount = 0u;

    for (uint8_t i = 0; i < sampleCount; ++i) {
        bool doDiscard = discardFirst && (i == 0u);
        sensorarrayFdcReadDiag_t diag = {0};
        esp_err_t err = sensorarrayMeasureReadFdcSampleDiag(fdcState->handle,
                                                            fdcMap->channel,
                                                            doDiscard,
                                                            fdcState->haveIds,
                                                            fdcState->configVerified,
                                                            &diag);
        const char *statusName = sensorarrayMeasureFdcSampleStatusName(diag.statusCode);
        const char *refClockName = (diag.sample.RefClockSource == FDC2214_REF_CLOCK_EXTERNAL)
                                       ? "external_clkin"
                                       : "internal_oscillator";

        switch (diag.statusCode) {
        case SENSORARRAY_FDC_SAMPLE_STATUS_SAMPLE_VALID:
            validCount++;
            break;
        case SENSORARRAY_FDC_SAMPLE_STATUS_I2C_READ_ERROR:
            i2cErrCount++;
            break;
        case SENSORARRAY_FDC_SAMPLE_STATUS_CONFIG_UNKNOWN:
            configUnknownCount++;
            break;
        case SENSORARRAY_FDC_SAMPLE_STATUS_STILL_SLEEPING:
            stillSleepingCount++;
            break;
        case SENSORARRAY_FDC_SAMPLE_STATUS_I2C_READ_OK_BUT_NOT_CONVERTING:
            notConvertingCount++;
            break;
        case SENSORARRAY_FDC_SAMPLE_STATUS_NO_UNREAD_CONVERSION:
            noUnreadCount++;
            break;
        case SENSORARRAY_FDC_SAMPLE_STATUS_ZERO_RAW_INVALID:
            zeroRawCount++;
            break;
        case SENSORARRAY_FDC_SAMPLE_STATUS_WATCHDOG_FAULT:
            watchdogCount++;
            break;
        case SENSORARRAY_FDC_SAMPLE_STATUS_AMPLITUDE_FAULT:
            amplitudeCount++;
            break;
        default:
            configUnknownCount++;
            break;
        }

        printf("DBGFDCSELF,index=%u,dLine=%u,fdcDev=%s,channel=%u,discardFirst=%u,i2cOk=%u,idOk=%u,configOk=%u,"
               "converting=%u,unread=%u,sampleValid=%u,raw=%lu,wd=%u,amp=%u,statusReg=0x%04X,configReg=0x%04X,"
               "muxReg=0x%04X,refClock=%s,err=%ld,status=%s\n",
               (unsigned)i,
               (unsigned)dLine,
               fdcState->label ? fdcState->label : SENSORARRAY_NA,
               (unsigned)fdcMap->channel,
               doDiscard ? 1u : 0u,
               diag.i2cOk ? 1u : 0u,
               diag.idOk ? 1u : 0u,
               diag.configOk ? 1u : 0u,
               diag.converting ? 1u : 0u,
               diag.unreadConversionPresent ? 1u : 0u,
               diag.sampleValid ? 1u : 0u,
               (unsigned long)diag.sample.Raw28,
               diag.sample.ErrWatchdog ? 1u : 0u,
               diag.sample.ErrAmplitude ? 1u : 0u,
               diag.sample.StatusRaw,
               diag.sample.ConfigRaw,
               diag.sample.MuxRaw,
               refClockName,
               (long)err,
               statusName);
    }

    uint32_t invalidCount = i2cErrCount + configUnknownCount + stillSleepingCount + notConvertingCount + noUnreadCount +
                            zeroRawCount + watchdogCount + amplitudeCount;
    const char *rootCause = "sample_valid";
    if (i2cErrCount > 0u) {
        rootCause = "i2c_read_error";
    } else if (configUnknownCount > 0u) {
        rootCause = "config_unknown";
    } else if (stillSleepingCount > 0u) {
        rootCause = "device_still_sleeping";
    } else if (notConvertingCount > 0u) {
        rootCause = "not_converting";
    } else if (noUnreadCount > 0u) {
        rootCause = "no_unread_conversions";
    } else if (zeroRawCount > 0u) {
        rootCause = "raw_stuck_zero";
    } else if (watchdogCount > 0u) {
        rootCause = "watchdog_fault";
    } else if (amplitudeCount > 0u) {
        rootCause = "amplitude_fault";
    }

    printf("DBGFDCSELF_SUMMARY,dLine=%u,samples=%u,valid=%lu,invalid=%lu,i2cErr=%lu,configUnknown=%lu,stillSleeping=%lu,"
           "notConverting=%lu,noUnread=%lu,zeroRaw=%lu,watchdog=%lu,amplitude=%lu,rootCause=%s\n",
           (unsigned)dLine,
           (unsigned)sampleCount,
           (unsigned long)validCount,
           (unsigned long)invalidCount,
           (unsigned long)i2cErrCount,
           (unsigned long)configUnknownCount,
           (unsigned long)stillSleepingCount,
           (unsigned long)notConvertingCount,
           (unsigned long)noUnreadCount,
           (unsigned long)zeroRawCount,
           (unsigned long)watchdogCount,
           (unsigned long)amplitudeCount,
           rootCause);

    sensorarrayDebugIdleForever("fdc_selftest_done");
}

#include "sensorarrayAdsGap.h"

#include <limits.h>
#include <string.h>

#include "driver/gpio.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "sensorarrayConfig.h"

typedef enum {
    SENSORARRAY_ADS_DIRECT_AIN9_OFFSET = 0,
    SENSORARRAY_ADS_DIRECT_AIN8_BATTERY = 1,
} sensorarrayAdsDirectJob_t;

static sensorarrayAdsGapSnapshot_t s_snapshot;
static uint32_t s_slackSamples;
static uint64_t s_slackTotalUs;
static uint32_t s_lastSampleFrame;
static sensorarrayAdsDirectJob_t s_nextJob;

static uint32_t sensorarrayAdsGapRemainingUs(uint64_t deadlineUs)
{
    int64_t nowUs = esp_timer_get_time();
    if (deadlineUs <= (uint64_t)nowUs) {
        return 0u;
    }
    uint64_t remaining = deadlineUs - (uint64_t)nowUs;
    return remaining > UINT32_MAX ? UINT32_MAX : (uint32_t)remaining;
}

static void sensorarrayAdsGapRecordSlack(uint32_t slackUs)
{
    s_slackTotalUs += slackUs;
    s_slackSamples++;
    s_snapshot.avgSlackUs = s_slackSamples ?
        (uint32_t)(s_slackTotalUs / s_slackSamples) : 0u;
    if (s_snapshot.minSlackUs == 0u || slackUs < s_snapshot.minSlackUs) {
        s_snapshot.minSlackUs = slackUs;
    }
}

static void sensorarrayAdsGapRecordOverrun(void)
{
    s_snapshot.overrunCount++;
    if (s_snapshot.guardUs < 5000u) {
        s_snapshot.guardUs += 100u;
    }
    if (s_snapshot.overrunCount >= 3u) {
        s_snapshot.fallbackToBoundary = true;
    }
}

esp_err_t sensorarrayAdsGapInit(sensorarrayState_t *state)
{
    memset(&s_snapshot, 0, sizeof(s_snapshot));
    s_snapshot.guardUs = (uint32_t)CONFIG_SENSORARRAY_ADS_GAP_GUARD_US;
    s_snapshot.rateCode = (uint8_t)CONFIG_SENSORARRAY_ADS_DATA_RATE;
    s_snapshot.minSlackUs = UINT32_MAX;
    s_slackSamples = 0u;
    s_slackTotalUs = 0u;
    s_lastSampleFrame = 0u;
    s_nextJob = SENSORARRAY_ADS_DIRECT_AIN9_OFFSET;

    if (!state || !state->adsReady) {
        return ESP_ERR_NOT_SUPPORTED;
    }
    if (xPortGetCoreID() != CONFIG_SENSORARRAY_ADS_WORKER_CORE) {
        return ESP_ERR_INVALID_STATE;
    }

    gpio_config_t startConfig = {
        .pin_bit_mask = 1ULL << (uint32_t)CONFIG_SENSORARRAY_ADS_START_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&startConfig);
    if (err != ESP_OK) {
        return err;
    }
    gpio_set_level((gpio_num_t)CONFIG_SENSORARRAY_ADS_START_GPIO, 0);

    err = ads126xAdcConfigure(&state->ads,
                              false,
                              false,
                              ADS126X_CRC_OFF,
                              1u,
                              (uint8_t)CONFIG_SENSORARRAY_ADS_DATA_RATE);
    if (err == ESP_OK) {
        err = ads126xAdcSetVbiasEnabled(&state->ads, false);
    }
    if (err == ESP_OK) {
        err = ads126xAdcSetRefMux(&state->ads, ADS126X_REFMUX_AVDD_AVSS);
    }
    if (err == ESP_OK) {
        err = ads126xAdcEnableDrdyNotification(&state->ads);
    }
    if (err != ESP_OK) {
        return err;
    }

    s_snapshot.initialized = true;
    s_snapshot.dmaCapable = state->ads.spiDmaCapable;
    s_snapshot.id = state->ads.idRegRaw;
    return ESP_OK;
}

void sensorarrayAdsGapTryRun(sensorarrayState_t *state,
                             uint64_t expectedFdcReadyUs,
                             uint32_t frameSequence,
                             uint8_t row)
{
    (void)row;
    s_snapshot.windows++;
    if (!CONFIG_SENSORARRAY_ADS_GAP_FILL_ENABLE || !state || !state->adsReady ||
        !s_snapshot.initialized || s_snapshot.fallbackToBoundary) {
        s_snapshot.jobsSkip++;
        return;
    }
    const uint32_t remainingBeforeUs = sensorarrayAdsGapRemainingUs(expectedFdcReadyUs);
    const uint32_t admissionUs =
        (uint32_t)CONFIG_SENSORARRAY_ADS_JOB_WORST_CASE_US + s_snapshot.guardUs;
    if (remainingBeforeUs <= admissionUs) {
        s_snapshot.jobsSkip++;
        return;
    }

    uint8_t muxp = s_nextJob == SENSORARRAY_ADS_DIRECT_AIN8_BATTERY ?
        SENSORARRAY_ADS_MUX_AIN8 : SENSORARRAY_ADS_MUX_AIN9;
    gpio_set_level((gpio_num_t)CONFIG_SENSORARRAY_ADS_START_GPIO, 0);
    esp_err_t err = ads126xAdcSetInputMuxFast(&state->ads, muxp, SENSORARRAY_ADS_MUX_AINCOM);
    if (err == ESP_OK) {
        gpio_set_level((gpio_num_t)CONFIG_SENSORARRAY_ADS_START_GPIO, 1);
        int32_t raw = 0;
        uint32_t dmaReadUs = 0u;
        err = ads126xAdcReadAdc1RawDma(&state->ads,
                                       (uint32_t)CONFIG_SENSORARRAY_ADS_DRDY_TIMEOUT_US,
                                       &raw,
                                       NULL,
                                       &dmaReadUs);
        gpio_set_level((gpio_num_t)CONFIG_SENSORARRAY_ADS_START_GPIO, 0);
        s_snapshot.dmaReadCount++;
        s_snapshot.dmaReadUs += dmaReadUs;
        if (err == ESP_OK) {
            int32_t uv = ads126xAdcRawToMicrovolts(&state->ads, raw);
            if (s_nextJob == SENSORARRAY_ADS_DIRECT_AIN8_BATTERY) {
                s_snapshot.ain8Raw = raw;
                s_snapshot.ain8RawUv = uv;
#if CONFIG_SENSORARRAY_ADS_AIN8_BATTERY_DIVIDER_NUM > 0
                s_snapshot.batteryMv =
                    (int32_t)(((int64_t)uv *
                               CONFIG_SENSORARRAY_ADS_AIN8_BATTERY_DIVIDER_NUM) /
                              ((int64_t)CONFIG_SENSORARRAY_ADS_AIN8_BATTERY_DIVIDER_DEN * 1000LL));
                s_snapshot.batteryValid = true;
#else
                s_snapshot.batteryMv = 0;
                s_snapshot.batteryValid = false;
#endif
            } else {
                s_snapshot.ain9OffsetRaw = raw;
                s_snapshot.ain9OffsetUv = uv;
            }
            s_snapshot.timestampUs = (uint64_t)esp_timer_get_time();
            s_lastSampleFrame = frameSequence;
            s_snapshot.jobsRun++;
            s_nextJob = s_nextJob == SENSORARRAY_ADS_DIRECT_AIN9_OFFSET ?
                SENSORARRAY_ADS_DIRECT_AIN8_BATTERY : SENSORARRAY_ADS_DIRECT_AIN9_OFFSET;
        }
    }
    if (err == ESP_ERR_TIMEOUT) {
        s_snapshot.drdyTimeoutCount++;
    } else if (err != ESP_OK) {
        s_snapshot.spiErrorCount++;
    }
    if (err != ESP_OK) {
        s_snapshot.jobsSkip++;
    }

    uint32_t slackUs = sensorarrayAdsGapRemainingUs(expectedFdcReadyUs);
    sensorarrayAdsGapRecordSlack(slackUs);
    if (slackUs < s_snapshot.guardUs) {
        sensorarrayAdsGapRecordOverrun();
    }
}

void sensorarrayAdsGapCopySnapshot(sensorarrayAdsGapSnapshot_t *outSnapshot,
                                   uint32_t frameSequence)
{
    if (!outSnapshot) {
        return;
    }
    *outSnapshot = s_snapshot;
    outSnapshot->sampleAgeFrames =
        (s_lastSampleFrame != 0u && frameSequence >= s_lastSampleFrame) ?
        (frameSequence - s_lastSampleFrame) : UINT32_MAX;
    if (outSnapshot->minSlackUs == UINT32_MAX) {
        outSnapshot->minSlackUs = 0u;
    }
}

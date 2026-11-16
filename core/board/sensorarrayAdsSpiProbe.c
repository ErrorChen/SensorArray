#include "sensorarrayAdsSpiProbe.h"

#include <limits.h>
#include <stdio.h>
#include <string.h>

#include "sdkconfig.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "freertos/task.h"

#ifndef CONFIG_SENSORARRAY_ADS_SPI_AUTOTUNE_PROBE_SAMPLES
#define CONFIG_SENSORARRAY_ADS_SPI_AUTOTUNE_PROBE_SAMPLES 400
#endif
#ifndef CONFIG_SENSORARRAY_ADS_SPI_AUTOTUNE_CONFIRM_SAMPLES
#define CONFIG_SENSORARRAY_ADS_SPI_AUTOTUNE_CONFIRM_SAMPLES 1600
#endif
#ifndef CONFIG_SENSORARRAY_ADS_SPI_AUTOTUNE_MIN_PARITY_SAMPLES
#define CONFIG_SENSORARRAY_ADS_SPI_AUTOTUNE_MIN_PARITY_SAMPLES 32
#endif
#ifndef CONFIG_SENSORARRAY_ADS_SPI_AUTOTUNE_MAX_CRC_PERMILLE
#define CONFIG_SENSORARRAY_ADS_SPI_AUTOTUNE_MAX_CRC_PERMILLE 25
#endif

#define ADS_SPI_PROBE_CLEANUP_US 500000u
#define ADS_SPI_PROBE_DRDY_TIMEOUT_US 20000u
#define ADS_SPI_PROBE_VBIAS_WAIT_MS 25u
#define ADS_SPI_PROBE_MUX_AINCOM 0x0Au

typedef struct {
    uint32_t attempted;
    uint32_t samples;
    uint32_t even;
    uint32_t odd;
    uint32_t crcErrors;
    uint32_t statusErrors;
    uint32_t registerErrors;
    uint32_t timeouts;
    int32_t rawMin;
    int32_t rawMax;
    uint64_t startUs;
    uint64_t endUs;
    esp_err_t error;
    const char *reason;
} sensorarrayAdsSpiProbePass_t;

static bool sensorarrayAdsSpiProbeRegistersMatch(
    const ads126xAdc1RegisterSnapshot_t *expected,
    const ads126xAdc1RegisterSnapshot_t *actual)
{
    return actual->id == expected->id &&
        (actual->power & ADS126X_POWER_RESET) == 0u &&
        ((actual->power ^ expected->power) & ADS126X_POWER_CONFIG_MASK) == 0u &&
        ((actual->interface ^ expected->interface) & ADS126X_INTERFACE_CONFIG_MASK) == 0u &&
        actual->mode0 == expected->mode0 && actual->mode1 == expected->mode1 &&
        actual->mode2 == expected->mode2 && actual->inpmux == expected->inpmux &&
        ((actual->refmux ^ expected->refmux) & ADS126X_REFMUX_CONFIG_MASK) == 0u &&
        memcmp(actual->offsetCal, expected->offsetCal, sizeof(actual->offsetCal)) == 0 &&
        memcmp(actual->fullScaleCal, expected->fullScaleCal, sizeof(actual->fullScaleCal)) == 0;
}

static esp_err_t sensorarrayAdsSpiProbeVerify(ads126xAdcHandle_t *ads,
    const ads126xAdc1RegisterSnapshot_t *expected)
{
    ads126xAdc1RegisterSnapshot_t actual = {0};
    uint32_t mismatches = ads->snapshotMismatchCount;
    esp_err_t err = ads126xAdcReadAdc1RegisterSnapshot(ads, &actual);
    return err != ESP_OK ? err :
        (mismatches == ads->snapshotMismatchCount &&
         sensorarrayAdsSpiProbeRegistersMatch(expected, &actual) ?
            ESP_OK : ESP_ERR_INVALID_RESPONSE);
}

static void sensorarrayAdsSpiProbeAccumulate(sensorarrayAdsSpiProbeResult_t *result,
    const sensorarrayAdsSpiProbePass_t *pass)
{
    result->validSamples += pass->samples;
    result->attemptedSamples += pass->attempted;
    result->crcValidEven += pass->even;
    result->crcValidOdd += pass->odd;
    result->crcErrors += pass->crcErrors;
    result->statusErrors += pass->statusErrors;
    result->registerErrors += pass->registerErrors;
    result->drdyTimeouts += pass->timeouts;
}

static esp_err_t sensorarrayAdsSpiProbePass(ads126xAdcHandle_t *ads,
    ads126xAdc1RegisterSnapshot_t *expected, uint32_t requestedHz,
    bool bypass, bool confirmation, uint32_t sampleTarget,
    uint64_t workDeadlineUs, sensorarrayAdsSpiProbeResult_t *result)
{
    sensorarrayAdsSpiProbePass_t pass = {
        .rawMin = INT32_MAX, .rawMax = INT32_MIN,
        .startUs = (uint64_t)esp_timer_get_time(),
        .reason = "register",
    };
    uint32_t actualHz = 0u, expectedHz = 0u;
    uint32_t maxCrcErrors = (uint32_t)(((uint64_t)sampleTarget *
        CONFIG_SENSORARRAY_ADS_SPI_AUTOTUNE_MAX_CRC_PERMILLE) / 1000u);
    esp_err_t err = ads126xAdcStopAdc1(ads);
    expected->mode2 = (uint8_t)((bypass ? 0x80u : 0u) | ADS126X_ADC1_DR_38400_SPS);
    if (err == ESP_OK) { err = ads126xAdcSetMode2Verified(ads, expected->mode2); }
    if (err == ESP_OK) { err = sensorarrayAdsSpiProbeVerify(ads, expected); }
    if (err == ESP_OK) {
        /* IDF applies override_freq_hz only in transaction setup and can log
         * a rejected clock while returning ESP_OK. This read follows actual
         * completed SPI transactions; all candidates are exact kHz values. */
        pass.reason = "actual_clock";
        err = ads126xAdcPredictDefaultSpiClockHz(requestedHz, &expectedHz);
        if (err == ESP_OK) { err = ads126xAdcGetSpiClockHz(ads, NULL, &actualHz); }
        if (err == ESP_OK && actualHz != expectedHz) { err = ESP_ERR_INVALID_RESPONSE; }
    }
    if (err != ESP_OK) { pass.registerErrors++; }
    if (err == ESP_OK) { pass.reason = "start"; err = ads126xAdcStartAdc1(ads); }
    for (uint32_t i = 0u; err == ESP_OK && i < sampleTarget; ++i) {
        if ((uint64_t)esp_timer_get_time() >= workDeadlineUs) {
            err = ESP_ERR_TIMEOUT;
            pass.reason = "deadline";
            pass.timeouts++;
            break;
        }
        uint32_t beforeEven = ads->adc1CrcValidEvenCount;
        uint32_t beforeOdd = ads->adc1CrcValidOddCount;
        uint32_t generation = ads126xAdcGetDrdyGeneration(ads);
        uint8_t retryBudget = 0u; /* Qualification never conceals a rejected wire frame. */
        int32_t raw = 0;
        uint8_t status = 0u;
        ads126xAdcFreshReadReport_t read = {0};
        err = ads126xAdcReadAdc1FreshChecked(ads, generation,
            ADS_SPI_PROBE_DRDY_TIMEOUT_US, &retryBudget, &raw, &status, &read);
        pass.attempted++;
        pass.crcErrors += read.crcErrors;
        if (err != ESP_OK) {
            pass.reason = err == ESP_ERR_INVALID_CRC ? "crc_rate" :
                (err == ESP_ERR_TIMEOUT ? "drdy_or_spi_timeout" :
                 err == ESP_ERR_INVALID_RESPONSE ? "freshness" : "spi_or_state");
        }
        /* User's boot qualification tolerance is a rate of rejected wire
         * frames, never permission to use one as scientific data. Count each
         * rejected fresh read once; no retry, parity, value or raw extrema. */
        if (err == ESP_ERR_INVALID_CRC && pass.crcErrors <= maxCrcErrors &&
            !ads126xAdcIsQuarantined(ads)) {
            err = ESP_OK;
            if ((pass.attempted % 64u) == 0u) {
                err = sensorarrayAdsSpiProbeVerify(ads, expected);
                if (err != ESP_OK) { pass.registerErrors++; pass.reason = "register"; break; }
            }
            if ((pass.attempted % 32u) == 0u) { vTaskDelay(1u); }
            continue;
        }
        if (err == ESP_ERR_TIMEOUT) { pass.timeouts++; }
        if (err == ESP_OK && ((status & (ADS126X_STATUS_RESET_OCCURRED |
                ADS126X_STATUS_REFERENCE_ALARM | ADS126X_STATUS_PGA_ALARM_MASK)) != 0u ||
                raw == INT32_MIN || raw == INT32_MAX)) {
            pass.statusErrors++;
            pass.reason = raw == INT32_MIN || raw == INT32_MAX ? "saturated" : "status";
            err = ESP_ERR_INVALID_RESPONSE;
        }
        if (err != ESP_OK) { break; }
        uint32_t even = ads->adc1CrcValidEvenCount - beforeEven;
        uint32_t odd = ads->adc1CrcValidOddCount - beforeOdd;
        if (even + odd != 1u) { pass.reason = "crc_counter"; err = ESP_ERR_INVALID_RESPONSE; break; }
        pass.even += even;
        pass.odd += odd;
        pass.samples++;
        if (raw < pass.rawMin) { pass.rawMin = raw; }
        if (raw > pass.rawMax) { pass.rawMax = raw; }
        if ((pass.attempted % 64u) == 0u) {
            err = sensorarrayAdsSpiProbeVerify(ads, expected);
            if (err != ESP_OK) { pass.registerErrors++; pass.reason = "register"; }
        }
        /* A 10-ms target tick bounds CPU monopolisation at high DRDY rates;
         * missed conversions are not sampled or counted as accepted data. */
        if (err == ESP_OK && (pass.attempted % 32u) == 0u) { vTaskDelay(1u); }
    }
    if (err == ESP_OK) {
        err = sensorarrayAdsSpiProbeVerify(ads, expected);
        if (err != ESP_OK) { pass.registerErrors++; pass.reason = "register"; }
    }
    if (err == ESP_OK &&
        (pass.attempted != sampleTarget || pass.samples + pass.crcErrors != sampleTarget ||
         pass.crcErrors > maxCrcErrors ||
         pass.even < CONFIG_SENSORARRAY_ADS_SPI_AUTOTUNE_MIN_PARITY_SAMPLES ||
         pass.odd < CONFIG_SENSORARRAY_ADS_SPI_AUTOTUNE_MIN_PARITY_SAMPLES ||
         pass.rawMin == pass.rawMax)) {
        err = ESP_ERR_INVALID_RESPONSE;
        pass.reason = pass.rawMin == pass.rawMax ? "constant_raw" : "parity_or_count";
    }
    if (err == ESP_OK) { pass.reason = "ok"; }
    pass.error = err;
    pass.endUs = (uint64_t)esp_timer_get_time();
    sensorarrayAdsSpiProbeAccumulate(result, &pass);
    printf("SPI_SWEEP,owner=BOOT,phase=%s,profile=%s,requestedHz=%lu,actualHz=%lu,actualSource=idf_khz,required=%lu,attempted=%lu,valid=%lu,even=%lu,odd=%lu,crc=%lu,maxCrc=%lu,permille=%u,status=%lu,register=%lu,timeout=%lu,reason=%s,err=0x%x\n",
        confirmation ? "confirm" : "screen", bypass ? "bypass" : "pga1",
        (unsigned long)requestedHz, (unsigned long)actualHz,
        (unsigned long)sampleTarget, (unsigned long)pass.attempted, (unsigned long)pass.samples,
        (unsigned long)pass.even, (unsigned long)pass.odd,
        (unsigned long)pass.crcErrors, (unsigned long)maxCrcErrors,
        (unsigned)CONFIG_SENSORARRAY_ADS_SPI_AUTOTUNE_MAX_CRC_PERMILLE,
        (unsigned long)pass.statusErrors,
        (unsigned long)pass.registerErrors, (unsigned long)pass.timeouts, pass.reason, err);
    printf("SPI_SWEEP_TIME,owner=BOOT,requestedHz=%lu,phase=%s,profile=%s,startUs=%llu,endUs=%llu,rawMin=%ld,rawMax=%ld,scope=raw_link_only\n",
        (unsigned long)requestedHz, confirmation ? "confirm" : "screen",
        bypass ? "bypass" : "pga1", (unsigned long long)pass.startUs,
        (unsigned long long)pass.endUs, (long)pass.rawMin, (long)pass.rawMax);
    return err;
}

esp_err_t sensorarrayAdsSpiProbeRun(sensorarrayState_t *state,
    uint64_t absoluteDeadlineUs, sensorarrayAdsSpiProbeResult_t *outResult)
{
    if (!outResult) { return ESP_ERR_INVALID_ARG; }
    memset(outResult, 0, sizeof(*outResult));
    outResult->startUs = (uint64_t)esp_timer_get_time();
    if (!state || !state->adsReady || ads126xAdcIsQuarantined(&state->ads) ||
        ads126xAdcIsAdc1Running(&state->ads) || state->ads.enableInternalRef) {
        outResult->error = ESP_ERR_INVALID_STATE;
        outResult->endUs = (uint64_t)esp_timer_get_time();
        return outResult->error;
    }
    if (absoluteDeadlineUs <= outResult->startUs + ADS_SPI_PROBE_CLEANUP_US) {
        outResult->error = ESP_ERR_TIMEOUT;
        outResult->endUs = (uint64_t)esp_timer_get_time();
        return outResult->error;
    }
    const uint32_t minimumPassCount =
        (CONFIG_SENSORARRAY_ADS_SPI_AUTOTUNE_PROBE_SAMPLES <
            CONFIG_SENSORARRAY_ADS_SPI_AUTOTUNE_CONFIRM_SAMPLES ?
            CONFIG_SENSORARRAY_ADS_SPI_AUTOTUNE_PROBE_SAMPLES :
            CONFIG_SENSORARRAY_ADS_SPI_AUTOTUNE_CONFIRM_SAMPLES) / 2u;
    if (CONFIG_SENSORARRAY_ADS_SPI_AUTOTUNE_MAX_CRC_PERMILLE > 25u ||
        minimumPassCount - (minimumPassCount *
            CONFIG_SENSORARRAY_ADS_SPI_AUTOTUNE_MAX_CRC_PERMILLE) / 1000u <
            2u * CONFIG_SENSORARRAY_ADS_SPI_AUTOTUNE_MIN_PARITY_SAMPLES) {
        outResult->error = ESP_ERR_INVALID_ARG;
        outResult->endUs = (uint64_t)esp_timer_get_time();
        printf("SPI_SELECTED,owner=BOOT,qualified=0,restored=0,reason=config_coverage_insufficient,err=0x%x\n",
            outResult->error);
        return outResult->error;
    }
    ads126xAdcHandle_t *ads = &state->ads;
    outResult->readPolicy = (uint8_t)ads->adc1ReadPolicy;
    outResult->readSource = ads->adc1ReadPolicy == ADS126X_ADC1_READ_GPIO_TAIL_REQUIRED ?
        ADS126X_READ_SOURCE_SPI55_GPIO1 : ADS126X_READ_SOURCE_SPI56;
    uint64_t workDeadlineUs = absoluteDeadlineUs - ADS_SPI_PROBE_CLEANUP_US;
    uint32_t savedClockHz = 0u;
    uint32_t savedVref = ads->vrefMicrovolts;
    ads126xAdc1RegisterSnapshot_t saved = {0};
    bool savedKnown = false;
    bool mutated = false;
    esp_err_t err = ads126xAdcBeginTransactionUntil(ads, workDeadlineUs);
    bool ownsTransaction = err == ESP_OK;
    if (err == ESP_OK) { err = ads126xAdcGetSpiClockHz(ads, NULL, &savedClockHz); }
    if (err == ESP_OK) {
        uint32_t mismatches = ads->snapshotMismatchCount;
        err = ads126xAdcReadAdc1RegisterSnapshot(ads, &saved);
        if (err == ESP_OK && mismatches != ads->snapshotMismatchCount) {
            err = ESP_ERR_INVALID_RESPONSE;
        }
    }
    if (err == ESP_OK && ((saved.power & (ADS126X_POWER_INTREF | ADS126X_POWER_RESET)) != 0u ||
            saved.id != ads->idRegRaw)) {
        err = ESP_ERR_INVALID_RESPONSE;
    }
    if (err == ESP_OK) { savedKnown = true; }
    if (err == ESP_OK) { err = ads126xAdcEnableDrdyNotification(ads); }
    if (err == ESP_OK) {
        gpio_config_t start = {
            .pin_bit_mask = 1ULL << CONFIG_SENSORARRAY_ADS_START_GPIO,
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        err = gpio_config(&start);
        if (err == ESP_OK) {
            err = gpio_set_level((gpio_num_t)CONFIG_SENSORARRAY_ADS_START_GPIO, 0);
        }
    }
    if (err == ESP_OK) {
        mutated = true;
        err = ads126xAdcStopAdc1(ads);
    }
    if (err == ESP_OK) { err = ads126xAdcConfigure(ads, false, true, ADS126X_CRC_CRC8,
                                                1u, ADS126X_ADC1_DR_38400_SPS); }
    if (err == ESP_OK) { err = ads126xAdcSetVbiasEnabled(ads, true); }
    if (err == ESP_OK) { err = ads126xAdcSetRefMux(ads, ADS126X_REFMUX_AVDD_AVSS); }
    if (err == ESP_OK) { err = ads126xAdcSetInputMuxVerified(ads,
        ADS_SPI_PROBE_MUX_AINCOM, ADS_SPI_PROBE_MUX_AINCOM); }
    ads126xAdc1RegisterSnapshot_t expected = saved;
    expected.power = (uint8_t)((saved.power & ~ADS126X_POWER_INTREF) | ADS126X_POWER_VBIAS);
    expected.interface = (uint8_t)((saved.interface & ~0x07u) | 0x06u);
    expected.mode0 = ADS126X_MODE0_CONTINUOUS_CHOP_OFF_DELAY_0;
    expected.mode1 = ADS126X_MODE1_FILTER_SINC1;
    expected.mode2 = ADS126X_ADC1_DR_38400_SPS;
    expected.inpmux = 0xAAu;
    expected.refmux = ADS126X_REFMUX_AVDD_AVSS;
    if (err == ESP_OK) { err = sensorarrayAdsSpiProbeVerify(ads, &expected); }
    if (err == ESP_OK) {
        /* Datasheet 9.3.12 table 9-7: VBIAS settling 22 ms at 10 uF.
         * This is an input-bias guard, not INTREF settling or precision
         * qualification. Every counted sample still needs a new DRDY and
         * valid status; no voltage value or rail estimate is produced. */
        if ((uint64_t)esp_timer_get_time() + ADS_SPI_PROBE_VBIAS_WAIT_MS * 1000u >= workDeadlineUs) {
            err = ESP_ERR_TIMEOUT;
        } else {
            vTaskDelay((ADS_SPI_PROBE_VBIAS_WAIT_MS * configTICK_RATE_HZ + 999u) / 1000u);
        }
    }
    static const uint32_t candidates[] = {
        2000000u, 1900000u, 1800000u, 1700000u, 1600000u, 1500000u,
        1400000u, 1300000u, 1200000u, 1100000u, 1000000u,
        950000u, 900000u, 850000u, 800000u, 750000u, 700000u,
        650000u, 600000u, 550000u, 500000u, 450000u, 400000u,
        350000u, 300000u, 250000u, 200000u, 150000u, 125000u,
    };
    esp_err_t firstCandidateError = ESP_OK;
    if (err == ESP_OK) {
        err = ESP_ERR_INVALID_RESPONSE; /* No untested fallback may become qualified. */
        for (size_t i = 0u; i < sizeof(candidates) / sizeof(candidates[0]); ++i) {
            if (candidates[i] > CONFIG_ADS126X_SPI_CLOCK_HZ) { continue; }
            if ((uint64_t)esp_timer_get_time() >= workDeadlineUs || ads126xAdcIsQuarantined(ads)) { break; }
            outResult->candidatesTested++;
            esp_err_t candidateError = ads126xAdcRequestSpiClockHz(ads, candidates[i]);
            for (unsigned confirm = 0u; candidateError == ESP_OK && confirm < 2u; ++confirm) {
                uint32_t count = confirm ? CONFIG_SENSORARRAY_ADS_SPI_AUTOTUNE_CONFIRM_SAMPLES :
                                          CONFIG_SENSORARRAY_ADS_SPI_AUTOTUNE_PROBE_SAMPLES;
                for (unsigned bypass = 0u; candidateError == ESP_OK && bypass < 2u; ++bypass) {
                    candidateError = sensorarrayAdsSpiProbePass(ads, &expected, candidates[i],
                        bypass != 0u, confirm != 0u, count / 2u + (bypass ? count % 2u : 0u),
                        workDeadlineUs, outResult);
                }
            }
            if (candidateError == ESP_OK) {
                outResult->selectedHz = candidates[i];
                err = ESP_OK;
                break;
            }
            if (firstCandidateError == ESP_OK) { firstCandidateError = candidateError; }
            err = firstCandidateError;
            /* Quarantine is terminal: no further clock request, descriptor
             * recycling, device removal, or ADC restore may be attempted. */
            if (ads126xAdcIsQuarantined(ads)) { break; }
        }
    }
    if (ownsTransaction) { ads126xAdcEndTransaction(ads); }
    esp_err_t cleanup = ESP_OK;
    if (ads126xAdcIsQuarantined(ads)) {
        cleanup = ESP_ERR_INVALID_STATE;
    } else if (savedKnown && mutated) {
        if ((uint64_t)esp_timer_get_time() >= absoluteDeadlineUs) {
            cleanup = ESP_ERR_TIMEOUT;
        } else {
            cleanup = ads126xAdcBeginTransactionUntil(ads, absoluteDeadlineUs);
            if (cleanup == ESP_OK) {
                /* A failed sweep restores the known bootstrap frequency;
                 * a passing sweep retains its separately confirmed clock. */
                cleanup = ads126xAdcRequestSpiClockHz(ads,
                    outResult->selectedHz ? outResult->selectedHz : savedClockHz);
                if (cleanup == ESP_OK) {
                    uint32_t mismatches = ads->snapshotMismatchCount;
                    cleanup = ads126xAdcRestoreAdc1RegisterSnapshot(ads, &saved, savedVref, false);
                    if (cleanup == ESP_OK && mismatches != ads->snapshotMismatchCount) {
                        cleanup = ESP_ERR_INVALID_RESPONSE;
                    }
                }
                uint32_t actualHz = 0u;
                if (cleanup == ESP_OK) { cleanup = ads126xAdcGetSpiClockHz(ads, NULL, &actualHz); }
                uint32_t expectedHz = savedClockHz;
                if (cleanup == ESP_OK && outResult->selectedHz) {
                    cleanup = ads126xAdcPredictDefaultSpiClockHz(outResult->selectedHz, &expectedHz);
                }
                if (cleanup == ESP_OK && actualHz != expectedHz) {
                    cleanup = ESP_ERR_INVALID_RESPONSE;
                }
                if (cleanup == ESP_OK) { outResult->actualHz = actualHz; }
                ads126xAdcEndTransaction(ads);
            }
        }
    }
    outResult->restored = savedKnown && cleanup == ESP_OK;
    if (cleanup != ESP_OK) { ads->interfaceKnown = false; }
    outResult->error = err;
    outResult->cleanupError = cleanup;
    outResult->qualified = err == ESP_OK && outResult->restored && outResult->selectedHz != 0u;
    outResult->readSourceKnown = outResult->validSamples != 0u;
    outResult->interfaceKnown = ads->interfaceKnown;
    outResult->interfaceReg = ads->interfaceReg;
    outResult->endUs = (uint64_t)esp_timer_get_time();
    printf("SPI_SELECTED,owner=BOOT,qualified=%u,restored=%u,selectedHz=%lu,actualHz=%lu,actualSource=idf_khz,candidates=%lu,err=0x%x,cleanup=0x%x,firstCandidateError=0x%x,startUs=%llu,endUs=%llu,scope=boot_link_samples\n",
        outResult->qualified ? 1u : 0u, outResult->restored ? 1u : 0u,
        (unsigned long)outResult->selectedHz, (unsigned long)outResult->actualHz,
        (unsigned long)outResult->candidatesTested, err, cleanup, firstCandidateError,
        (unsigned long long)outResult->startUs, (unsigned long long)outResult->endUs);
    printf("SPI_READ_SOURCE,owner=BOOT,policy=%u,known=%u,source=%s,interfaceKnown=%u,interface=%02X,scope=boot_link_samples\n",
        outResult->readPolicy, outResult->readSourceKnown ? 1u : 0u,
        ads126xAdcReadSourceName(outResult->readSource), outResult->interfaceKnown ? 1u : 0u,
        outResult->interfaceReg);
    return err != ESP_OK ? err : cleanup;
}

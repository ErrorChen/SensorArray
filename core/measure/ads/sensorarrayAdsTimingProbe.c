#include "sensorarrayAdsTimingProbe.h"

#include <limits.h>
#include <stdio.h>
#include <string.h>

#include "sdkconfig.h"
#include "esp_timer.h"
#include "freertos/task.h"

#ifndef CONFIG_SENSORARRAY_ADS_DRDY_TIMEOUT_US
#define CONFIG_SENSORARRAY_ADS_DRDY_TIMEOUT_US 600u
#endif

const char *sensorarrayAdsTimingProbeKindName(sensorarrayAdsTimingProbeKind_t kind)
{
    return kind == SENSORARRAY_ADS_TIMING_PROBE_TIMING ? "TIMING" :
        kind == SENSORARRAY_ADS_TIMING_PROBE_TAIL ? "TAIL" : "unknown";
}

static uint32_t sensorarrayAdsTimingU32(uint64_t value)
{
    return value > UINT32_MAX ? UINT32_MAX : (uint32_t)value;
}

static bool sensorarrayAdsTimingRegistersMatch(
    const ads126xAdc1RegisterSnapshot_t *a,
    const ads126xAdc1RegisterSnapshot_t *b)
{
    return a->id == b->id && (b->power & ADS126X_POWER_RESET) == 0u &&
        ((a->power ^ b->power) & ADS126X_POWER_CONFIG_MASK) == 0u &&
        ((a->interface ^ b->interface) & ADS126X_INTERFACE_CONFIG_MASK) == 0u &&
        a->mode0 == b->mode0 && a->mode1 == b->mode1 && a->mode2 == b->mode2 &&
        a->inpmux == b->inpmux &&
        ((a->refmux ^ b->refmux) & ADS126X_REFMUX_CONFIG_MASK) == 0u &&
        memcmp(a->offsetCal, b->offsetCal, sizeof(a->offsetCal)) == 0 &&
        memcmp(a->fullScaleCal, b->fullScaleCal, sizeof(a->fullScaleCal)) == 0;
}

static esp_err_t sensorarrayAdsTimingVerify(ads126xAdcHandle_t *ads,
    const ads126xAdc1RegisterSnapshot_t *expected)
{
    ads126xAdc1RegisterSnapshot_t actual = {0};
    uint32_t mismatches = ads->snapshotMismatchCount;
    esp_err_t err = ads126xAdcReadAdc1RegisterSnapshot(ads, &actual);
    if (err == ESP_OK && (mismatches != ads->snapshotMismatchCount ||
        !sensorarrayAdsTimingRegistersMatch(expected, &actual))) {
        err = ESP_ERR_INVALID_RESPONSE;
    }
    return err;
}

static void sensorarrayAdsTimingError(sensorarrayAdsTimingProbeResult_t *r,
    sensorarrayAdsTimingGroup_t *g, esp_err_t error, const char *reason,
    bool isolate)
{
    if (r->firstError == ESP_OK) { r->firstError = error; }
    r->terminalError = g->terminalError = error;
    r->reason = g->reason = reason;
    r->requiresIsolation |= isolate;
}

static void sensorarrayAdsTimingRecord(sensorarrayAdsTimingProbeResult_t *r,
    unsigned group, const sensorarrayAdsTimingRecord_t *record,
    uint32_t *categories)
{
    unsigned kind;
    if (record->error == ESP_ERR_INVALID_CRC) {
        uint8_t delta = record->computed ^ record->received;
        kind = delta == 1u ? ((record->received & 1u) ? 2u : 3u) : 4u;
    } else if (record->error == ESP_OK) {
        kind = record->crcParity;
    } else {
        kind = 5u;
    }
    if (record->held) { kind += 6u; }
    uint32_t bit = 1u << kind;
    if ((*categories & bit) != 0u) { return; }
    *categories |= bit;
    sensorarrayAdsTimingGroup_t *g = &r->groups[group];
    if (g->recordCount < SENSORARRAY_ADS_TIMING_RECORDS_PER_GROUP) {
        r->records[group][g->recordCount++] = *record;
    } else {
        g->recordsOmitted++;
    }
}

static void sensorarrayAdsTimingNoteCrc(sensorarrayAdsTimingReadStats_t *stats,
    sensorarrayAdsTimingRecord_t *record, const ads126xAdcCrcFault_t *fault)
{
    stats->crc++;
    record->computed = fault->computed;
    record->received = fault->received;
    record->crcParity = fault->computed & 1u;
    record->length = fault->length <= sizeof(record->bytes) ?
        fault->length : sizeof(record->bytes);
    memcpy(record->bytes, fault->bytes, record->length);
    if (record->length == 6u) {
        record->status = record->bytes[0]; /* Untrusted wire evidence only. */
        uint32_t code = ((uint32_t)record->bytes[1] << 24u) |
            ((uint32_t)record->bytes[2] << 16u) |
            ((uint32_t)record->bytes[3] << 8u) | record->bytes[4];
        record->raw = (int32_t)code;
    }
    uint8_t delta = fault->computed ^ fault->received;
    if (delta == 1u && (fault->received & 1u)) { stats->xorUp++; }
    else if (delta == 1u) { stats->xorDown++; }
    else { stats->xorOther++; }
}

static bool sensorarrayAdsTimingBadStatus(uint8_t status, int32_t raw)
{
    return (status & (ADS126X_STATUS_RESET_OCCURRED |
        ADS126X_STATUS_REFERENCE_ALARM | ADS126X_STATUS_PGA_ALARM_MASK)) != 0u ||
        raw == INT32_MIN || raw == INT32_MAX;
}

static esp_err_t sensorarrayAdsTimingRead(ads126xAdcHandle_t *ads,
    sensorarrayAdsTimingProbeResult_t *r, unsigned group, uint32_t attempt,
    bool held, uint32_t floor, uint32_t before, uint32_t startCallUs,
    bool priorRawValid, int32_t priorRaw, uint32_t *categories,
    uint32_t *outGeneration, bool *outRawValid, int32_t *outRaw)
{
    sensorarrayAdsTimingGroup_t *g = &r->groups[group];
    sensorarrayAdsTimingReadStats_t *stats = held ? &g->held : &g->primary;
    sensorarrayAdsTimingRecord_t rec = {
        .attempt = attempt, .held = held,
        .configurationGeneration = ads->configurationGeneration,
        .generationBefore = before, .generationFence = floor,
        .startCallUs = startCallUs,
    };
    uint32_t evenBefore = ads->adc1CrcValidEvenCount;
    uint32_t oddBefore = ads->adc1CrcValidOddCount;
    uint32_t badBefore = ads->adc1CrcErrorCount;
    ads126xAdcFreshReadReport_t read = {0};
    uint8_t retryBudget = 0u;
    int32_t raw = 0;
    uint8_t status = 0u;
    uint32_t readUs = 0u;
    stats->attempted++;
    esp_err_t err = g->gpioTail ?
        ads126xAdcReadAdc1FreshTailDiagnostic(ads, floor,
            CONFIG_SENSORARRAY_ADS_DRDY_TIMEOUT_US, &raw, &status, &read, &rec.tail) : held ?
        ads126xAdcReadAdc1RawSpiDiagnostic(ads, &raw, &status, &readUs) :
        ads126xAdcReadAdc1FreshSpiDiagnostic(ads, floor,
            CONFIG_SENSORARRAY_ADS_DRDY_TIMEOUT_US, &retryBudget,
            &raw, &status, &read);
    rec.observedUs = (uint64_t)esp_timer_get_time();
    rec.waitUs = sensorarrayAdsTimingU32(read.waitUs);
    rec.readUs = held ? readUs : sensorarrayAdsTimingU32(read.readUs);
    rec.generationAfter = ads126xAdcGetDrdyGeneration(ads);
    rec.error = err;
    if (err != ESP_OK && r->firstError == ESP_OK) { r->firstError = err; }
    if (err == ESP_ERR_INVALID_CRC) {
        sensorarrayAdsTimingNoteCrc(stats, &rec,
            held ? &ads->lastAdc1CrcFault : &read.firstCrcFault);
        if (r->firstError == ESP_OK) { r->firstError = err; }
    }
    if (g->gpioTail && rec.tail.crcKnown) {
        /* These bytes were physically observed over55SPI+1GPIO clocks. They
         * must not be labelled a complete continuous SPI wire capture. */
        rec.received = rec.tail.assembledCrc;
        rec.computed = rec.tail.computedCrc;
    }
    uint32_t even = ads->adc1CrcValidEvenCount - evenBefore;
    uint32_t odd = ads->adc1CrcValidOddCount - oddBefore;
    stats->even += even;
    stats->odd += odd;
    if (err == ESP_OK) {
        rec.raw = raw; rec.status = status; rec.rawValid = true;
        rec.crcValid = true;
        /* Only parity is known here; do not invent received good wire bytes. */
        rec.crcParity = odd ? 1u : 0u;
    }
    bool generationOk = ads->configurationGeneration == rec.configurationGeneration;
    if (held) {
        generationOk = generationOk && rec.generationAfter == floor;
    } else if (g->pulse) {
        generationOk = generationOk &&
            (uint32_t)(read.generationEnd - floor) == 1u &&
            rec.generationAfter == read.generationEnd;
    }
    if (ads126xAdcIsQuarantined(ads)) {
        sensorarrayAdsTimingError(r, g, err != ESP_OK ? err : ESP_ERR_INVALID_STATE,
            g->gpioTail && rec.tail.stage ? ads126xAdcTailStageName(rec.tail.stage) : "spi_quarantined", true);
    } else if (g->gpioTail && rec.tail.spiComplete &&
        (rec.tail.requestedHz != r->requestedHz || rec.tail.actualHz != r->actualHz)) {
        sensorarrayAdsTimingError(r, g, ESP_ERR_INVALID_RESPONSE, "tail_clock", true);
    } else if (err != ESP_OK && err != ESP_ERR_INVALID_CRC) {
        if (err == ESP_ERR_TIMEOUT) { stats->timeout++; }
        else { stats->freshness++; }
        sensorarrayAdsTimingError(r, g,
            !held && read.terminalError != ESP_OK ? read.terminalError : err,
            g->gpioTail && rec.tail.stage ? ads126xAdcTailStageName(rec.tail.stage) :
                err == ESP_ERR_TIMEOUT ? "drdy_or_spi_timeout" : "read", true);
    } else if (!generationOk) {
        stats->freshness++;
        sensorarrayAdsTimingError(r, g, ESP_ERR_INVALID_STATE, "generation", true);
    } else if (err == ESP_ERR_INVALID_CRC &&
        ads->adc1CrcErrorCount - badBefore == 1u &&
        (held || read.terminalError == ESP_ERR_INVALID_CRC)) {
        /* A rejected wire frame is counted, never retried or made valid. */
    } else if (err != ESP_OK) {
        if (err == ESP_ERR_TIMEOUT) { stats->timeout++; }
        else { stats->freshness++; }
        sensorarrayAdsTimingError(r, g,
            !held && read.terminalError != ESP_OK ? read.terminalError : err,
            err == ESP_ERR_TIMEOUT ? "drdy_or_spi_timeout" : "read", true);
    } else if (even + odd != 1u) {
        sensorarrayAdsTimingError(r, g, ESP_ERR_INVALID_RESPONSE, "crc_counter", true);
    } else if (sensorarrayAdsTimingBadStatus(status, raw)) {
        stats->status++;
        sensorarrayAdsTimingError(r, g, ESP_ERR_INVALID_RESPONSE, "status", true);
    } else if (held && (ads126xAdcStatusByteHasAdc1NewData(ads, status) ||
        (priorRawValid && raw != priorRaw))) {
        stats->freshness++;
        sensorarrayAdsTimingError(r, g, ESP_ERR_INVALID_RESPONSE, "hold_state_changed", true);
    } else {
        stats->valid++;
        rec.fresh = !held;
    }
    if (rec.error == ESP_OK && g->terminalError != ESP_OK) {
        rec.error = g->terminalError;
    }
    sensorarrayAdsTimingRecord(r, group, &rec, categories);
    *outGeneration = rec.generationAfter;
    *outRawValid = rec.rawValid && rec.fresh;
    *outRaw = raw;
    return g->terminalError;
}

esp_err_t sensorarrayAdsTimingProbeRun(ads126xAdcHandle_t *ads,
    uint64_t workDeadlineUs, sensorarrayAdsTimingProbeResult_t *r)
{
    if (!ads || !r || ads->transactionDeadlineUs == 0u ||
        workDeadlineUs > ads->transactionDeadlineUs) { return ESP_ERR_INVALID_ARG; }
    ads126xAdc1RegisterSnapshot_t expected = r->saved;
    if ((expected.power & (ADS126X_POWER_RESET | ADS126X_POWER_INTREF)) != 0u ||
        (expected.power & ADS126X_POWER_VBIAS) == 0u ||
        (expected.interface & 7u) != 6u || expected.refmux != ADS126X_REFMUX_AVDD_AVSS) {
        return ESP_ERR_INVALID_STATE;
    }
    if (r->kind != SENSORARRAY_ADS_TIMING_PROBE_TIMING && r->kind != SENSORARRAY_ADS_TIMING_PROBE_TAIL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (r->kind == SENSORARRAY_ADS_TIMING_PROBE_TAIL &&
        ((expected.interface & 0x08u) != 0u || !ads126xAdcTailDiagnosticAvailable(ads))) {
        return ESP_ERR_NOT_SUPPORTED;
    }
    for (unsigned i = 0u; i < SENSORARRAY_ADS_TIMING_GROUPS; ++i) {
        r->groups[i].pulse = r->kind == SENSORARRAY_ADS_TIMING_PROBE_TIMING && (i & 1u) != 0u;
        r->groups[i].gpioTail = r->kind == SENSORARRAY_ADS_TIMING_PROBE_TAIL && (i & 1u) != 0u;
        r->groups[i].bypass = i >= 2u;
    }
    for (unsigned i = 0u; i < SENSORARRAY_ADS_TIMING_GROUPS; ++i) {
        sensorarrayAdsTimingGroup_t *g = &r->groups[i];
        g->startedUs = (uint64_t)esp_timer_get_time();
        g->reason = "setup";
        esp_err_t err = g->startedUs >= workDeadlineUs ? ESP_ERR_TIMEOUT :
            ads126xAdcStopAdc1(ads);
        expected.mode0 = g->pulse ? 0x40u : 0x00u;
        expected.mode1 = 0u;
        expected.mode2 = g->bypass ? 0x8Fu : 0x0Fu;
        expected.inpmux = 0xAAu;
        if (err == ESP_OK) { err = ads126xAdcConfigureAdc1Mode(ads, expected.mode0, 0u); }
        if (err == ESP_OK) { err = ads126xAdcSetMode2Verified(ads, expected.mode2); }
        if (err == ESP_OK) { err = ads126xAdcSetInputMuxVerified(ads, 0x0Au, 0x0Au); }
        if (err == ESP_OK) { err = sensorarrayAdsTimingVerify(ads, &expected); }
        if (err != ESP_OK) {
            g->registerErrors++;
            sensorarrayAdsTimingError(r, g, err, "profile", true);
        }
        if (err == ESP_OK && !g->pulse) { err = ads126xAdcStartAdc1(ads); }
        if (err != ESP_OK && g->terminalError == ESP_OK) {
            sensorarrayAdsTimingError(r, g, err, "start", true);
        }
        g->configurationGeneration = ads->configurationGeneration;
        uint32_t categories = 0u;
        for (uint32_t attempt = 1u; g->terminalError == ESP_OK &&
             attempt <= SENSORARRAY_ADS_TIMING_SAMPLES; ++attempt) {
            if ((uint64_t)esp_timer_get_time() >= workDeadlineUs) {
                sensorarrayAdsTimingError(r, g, ESP_ERR_TIMEOUT, "deadline", false);
                break;
            }
            uint32_t before = ads126xAdcGetDrdyGeneration(ads);
            uint32_t floor = before, startCallUs = 0u;
            if (g->pulse) {
                uint64_t start = (uint64_t)esp_timer_get_time();
                err = ads126xAdcStartAdc1(ads);
                startCallUs = sensorarrayAdsTimingU32((uint64_t)esp_timer_get_time() - start);
                floor = ads->configuredDrdyGeneration;
                if (err != ESP_OK) {
                    sensorarrayAdsTimingError(r, g, err, "start", true);
                    break;
                }
                if (floor != before) {
                    g->startFenceMissed++;
                    sensorarrayAdsTimingError(r, g, ESP_ERR_INVALID_RESPONSE,
                        "start_fence_missed", false);
                    break;
                }
            }
            uint32_t generation = floor;
            bool primaryRawValid = false;
            int32_t primaryRaw = 0;
            uint32_t crcBefore = ads->adc1CrcErrorCount;
            err = sensorarrayAdsTimingRead(ads, r, i, attempt, false, floor, before,
                startCallUs, false, 0, &categories, &generation,
                &primaryRawValid, &primaryRaw);
            if (err == ESP_OK && g->pulse) {
                bool ignoredValid = false;
                int32_t ignoredRaw = 0;
                err = sensorarrayAdsTimingRead(ads, r, i, attempt, true, generation,
                    generation, 0u, primaryRawValid, primaryRaw, &categories,
                    &generation, &ignoredValid, &ignoredRaw);
            }
            if (err == ESP_OK && (ads->adc1CrcErrorCount != crcBefore || attempt % 64u == 0u)) {
                err = sensorarrayAdsTimingVerify(ads, &expected);
                if (err != ESP_OK) {
                    g->registerErrors++;
                    sensorarrayAdsTimingError(r, g, err, "register", true);
                }
            }
            if (err == ESP_OK && attempt % 32u == 0u) {
                uint64_t now = (uint64_t)esp_timer_get_time();
                uint64_t tickUs = (1000000u + configTICK_RATE_HZ - 1u) / configTICK_RATE_HZ;
                if (now + tickUs >= workDeadlineUs) {
                    sensorarrayAdsTimingError(r, g, ESP_ERR_TIMEOUT, "deadline", false);
                } else { vTaskDelay(1u); }
            }
        }
        if (g->terminalError == ESP_OK) {
            err = sensorarrayAdsTimingVerify(ads, &expected);
            uint32_t requestedHz = 0u, actualHz = 0u;
            if (err == ESP_OK) { err = ads126xAdcGetSpiClockHz(ads, &requestedHz, &actualHz); }
            if (err == ESP_OK && (requestedHz != r->requestedHz || actualHz != r->actualHz)) {
                err = ESP_ERR_INVALID_RESPONSE;
            }
            if (err != ESP_OK) {
                g->registerErrors++;
                sensorarrayAdsTimingError(r, g, err, "final_profile_or_clock", true);
            }
        }
        g->completed = g->terminalError == ESP_OK &&
            g->primary.attempted == SENSORARRAY_ADS_TIMING_SAMPLES &&
            g->primary.valid + g->primary.crc == SENSORARRAY_ADS_TIMING_SAMPLES &&
            (!g->pulse || g->held.attempted == SENSORARRAY_ADS_TIMING_SAMPLES);
        if (g->completed) {
            g->reason = g->primary.even < 32u || g->primary.odd < 32u ?
                "parity_insufficient" : g->primary.crc > 5u ? "crc_above_25permille" : "complete";
        }
        g->completedUs = (uint64_t)esp_timer_get_time();
        if (!g->completed) { break; }
    }
    r->completed = true;
    for (unsigned i = 0u; i < SENSORARRAY_ADS_TIMING_GROUPS; ++i) {
        r->completed &= r->groups[i].completed;
    }
    if (r->completed) { r->reason = "complete"; }
    r->workCompletedUs = (uint64_t)esp_timer_get_time();
    return r->firstError;
}

static size_t sensorarrayAdsTimingFormatted(char *buffer, size_t size, int count)
{
    if (count < 0 || (size_t)count >= size) {
        if (buffer && size) { buffer[0] = '\0'; }
        return 0u;
    }
    return (size_t)count;
}

size_t sensorarrayAdsTimingProbeFormatSummary(
    const sensorarrayAdsTimingProbeResult_t *r, char *b, size_t n)
{
    if (!r || !b || !n) { return 0u; }
    return sensorarrayAdsTimingFormatted(b, n, snprintf(b, n,
        "ADSTIMING,diagnostic=1,probe=%s,science=0,id=%lu,seq=%lu,groups=4,n=200,done=%u,mutated=%u,restoreKnown=%u,restoreVerified=%u,isolate=%u,hz=%lu,actualHz=%lu,first=0x%lx,terminal=0x%lx,cleanup=0x%lx,startUs=%llu,endUs=%llu,gen=%lu/%lu,crcGood=%lu/%lu,crcBad=%lu/%lu,records=%lu,reason=%s\n",
        sensorarrayAdsTimingProbeKindName(r->kind), (unsigned long)r->requestId, (unsigned long)r->frameSequence,
        r->completed, r->mutated, r->restoreKnown, r->restoreVerified, r->requiresIsolation,
        (unsigned long)r->requestedHz, (unsigned long)r->actualHz,
        (unsigned long)r->firstError, (unsigned long)r->terminalError,
        (unsigned long)r->cleanupError, (unsigned long long)r->startedUs,
        (unsigned long long)r->completedUs, (unsigned long)r->generationBefore,
        (unsigned long)r->generationAfter, (unsigned long)r->crcGoodBefore,
        (unsigned long)r->crcGoodAfter, (unsigned long)r->crcBadBefore,
        (unsigned long)r->crcBadAfter, (unsigned long)sensorarrayAdsTimingProbeRecordCount(r),
        r->reason ? r->reason : "unknown"));
}

size_t sensorarrayAdsTimingProbeFormatGroup(
    const sensorarrayAdsTimingProbeResult_t *r, uint32_t group, char *b, size_t n)
{
    if (!r || group >= SENSORARRAY_ADS_TIMING_GROUPS || !b || !n) { return 0u; }
    const sensorarrayAdsTimingGroup_t *g = &r->groups[group];
    const sensorarrayAdsTimingReadStats_t *p = &g->primary, *h = &g->held;
    return sensorarrayAdsTimingFormatted(b, n, snprintf(b, n,
        "ADSTIMINGGROUP,diagnostic=1,probe=%s,id=%lu,g=%lu,source=%s,mode=%s,pga=%s,n=200,a=%lu,v=%lu,crc=%lu,even=%lu,odd=%lu,up=%lu,down=%lu,other=%lu,status=%lu,freshErr=%lu,timeout=%lu,fenceMiss=%lu,regErr=%lu,heldA=%lu,heldV=%lu,heldCrc=%lu,heldUp=%lu,heldDown=%lu,heldOther=%lu,done=%u,err=0x%lx,reason=%s\n",
        sensorarrayAdsTimingProbeKindName(r->kind), (unsigned long)r->requestId, (unsigned long)group,
        g->gpioTail ? "spi55_gpio1" : "spi56", g->pulse ? "PULSE" : "CONT", g->bypass ? "bypass" : "gain1",
        (unsigned long)p->attempted, (unsigned long)p->valid, (unsigned long)p->crc,
        (unsigned long)p->even, (unsigned long)p->odd, (unsigned long)p->xorUp,
        (unsigned long)p->xorDown, (unsigned long)p->xorOther, (unsigned long)p->status,
        (unsigned long)p->freshness, (unsigned long)p->timeout,
        (unsigned long)g->startFenceMissed, (unsigned long)g->registerErrors,
        (unsigned long)h->attempted, (unsigned long)h->valid, (unsigned long)h->crc,
        (unsigned long)h->xorUp, (unsigned long)h->xorDown, (unsigned long)h->xorOther,
        g->completed, (unsigned long)g->terminalError, g->reason ? g->reason : "not_run"));
}

size_t sensorarrayAdsTimingProbeFormatGroupTiming(
    const sensorarrayAdsTimingProbeResult_t *r, uint32_t group, char *b, size_t n)
{
    if (!r || group >= SENSORARRAY_ADS_TIMING_GROUPS || !b || !n) { return 0u; }
    const sensorarrayAdsTimingGroup_t *g = &r->groups[group];
    return sensorarrayAdsTimingFormatted(b, n, snprintf(b, n,
        "ADSTIMINGTIME,diagnostic=1,probe=%s,id=%lu,g=%lu,startUs=%llu,endUs=%llu,gen=%lu,records=%lu,omitted=%lu,heldEven=%lu,heldOdd=%lu,heldStatus=%lu,heldFreshErr=%lu,heldTimeout=%lu\n",
        sensorarrayAdsTimingProbeKindName(r->kind), (unsigned long)r->requestId, (unsigned long)group,
        (unsigned long long)g->startedUs, (unsigned long long)g->completedUs,
        (unsigned long)g->configurationGeneration, (unsigned long)g->recordCount,
        (unsigned long)g->recordsOmitted, (unsigned long)g->held.even,
        (unsigned long)g->held.odd, (unsigned long)g->held.status,
        (unsigned long)g->held.freshness, (unsigned long)g->held.timeout));
}

size_t sensorarrayAdsTimingProbeFormatRegisters(
    const sensorarrayAdsTimingProbeResult_t *r, bool restored, char *b, size_t n)
{
    if (!r || !b || !n) { return 0u; }
    const ads126xAdc1RegisterSnapshot_t *s = restored ? &r->restored : &r->saved;
    return sensorarrayAdsTimingFormatted(b, n, snprintf(b, n,
        "ADSTIMINGREG,diagnostic=1,probe=%s,id=%lu,phase=%s,known=%u,ID=%02X,POWER=%02X,INTERFACE=%02X,MODE0=%02X,MODE1=%02X,MODE2=%02X,INPMUX=%02X,REFMUX=%02X,OCAL=%02X%02X%02X,FSCAL=%02X%02X%02X\n",
        sensorarrayAdsTimingProbeKindName(r->kind), (unsigned long)r->requestId, restored ? "restored" : "saved",
        restored ? r->restoreVerified : r->savedKnown,
        s->id, s->power, s->interface, s->mode0, s->mode1, s->mode2, s->inpmux,
        s->refmux, s->offsetCal[0], s->offsetCal[1], s->offsetCal[2],
        s->fullScaleCal[0], s->fullScaleCal[1], s->fullScaleCal[2]));
}

uint32_t sensorarrayAdsTimingProbeRecordCount(const sensorarrayAdsTimingProbeResult_t *r)
{
    if (!r) { return 0u; }
    uint32_t count = 0u;
    for (unsigned i = 0u; i < SENSORARRAY_ADS_TIMING_GROUPS; ++i) {
        count += r->groups[i].recordCount;
    }
    return count;
}

size_t sensorarrayAdsTimingProbeFormatRecord(const sensorarrayAdsTimingProbeResult_t *r,
    uint32_t index, char *b, size_t n)
{
    if (!r || !b || !n) { return 0u; }
    uint32_t group = 0u;
    while (group < SENSORARRAY_ADS_TIMING_GROUPS && index >= r->groups[group].recordCount) {
        index -= r->groups[group++].recordCount;
    }
    if (group >= SENSORARRAY_ADS_TIMING_GROUPS) { b[0] = '\0'; return 0u; }
    const sensorarrayAdsTimingRecord_t *v = &r->records[group][index];
    bool gpioTail = r->groups[group].gpioTail;
    bool continuousWire = v->length != 0u && !gpioTail;
    char bytes[13] = {0};
    char computed[8] = "unknown", received[8] = "unknown", delta[8] = "unknown";
    for (unsigned i = 0u; continuousWire && i < v->length && i < sizeof(v->bytes); ++i) {
        (void)snprintf(bytes + i * 2u, sizeof(bytes) - i * 2u, "%02X", v->bytes[i]);
    }
    if (v->length || (gpioTail && v->tail.crcKnown)) {
        (void)snprintf(computed, sizeof(computed), "%02X", v->computed);
        (void)snprintf(received, sizeof(received), "%02X", v->received);
        (void)snprintf(delta, sizeof(delta), "%02X", v->computed ^ v->received);
    }
    return sensorarrayAdsTimingFormatted(b, n, snprintf(b, n,
        "ADSTIMINGRAW,diagnostic=1,probe=%s,id=%lu,g=%lu,source=%s,attempt=%lu,kind=%s,raw=%ld,rawValid=%u,fresh=%u,status=%02X,crcValid=%u,parity=%u,wireKnown=%u,bytes=%s,computed=%s,received=%s,xor=%s,err=0x%lx,observedUs=%llu,startCallUs=%lu,waitUs=%lu,readUs=%lu,cfg=%lu,drdy=%lu/%lu/%lu\n",
        sensorarrayAdsTimingProbeKindName(r->kind), (unsigned long)r->requestId, (unsigned long)group, gpioTail ? "spi55_gpio1" : "spi56", (unsigned long)v->attempt,
        v->held ? "held_repeat" : "primary", (long)v->raw, v->rawValid, v->fresh,
        v->status, v->crcValid, v->crcParity, continuousWire, bytes[0] ? bytes : "unknown",
        computed, received, delta, (unsigned long)v->error,
        (unsigned long long)v->observedUs, (unsigned long)v->startCallUs,
        (unsigned long)v->waitUs, (unsigned long)v->readUs,
        (unsigned long)v->configurationGeneration, (unsigned long)v->generationBefore,
        (unsigned long)v->generationFence, (unsigned long)v->generationAfter));
}

size_t sensorarrayAdsTimingProbeFormatTailRecord(const sensorarrayAdsTimingProbeResult_t *r,
    uint32_t index, char *b, size_t n)
{
    if (!r || !b || !n || r->kind != SENSORARRAY_ADS_TIMING_PROBE_TAIL) { return 0u; }
    uint32_t group = 0u;
    while (group < SENSORARRAY_ADS_TIMING_GROUPS && index >= r->groups[group].recordCount) {
        index -= r->groups[group++].recordCount;
    }
    if (group >= SENSORARRAY_ADS_TIMING_GROUPS) { b[0] = '\0'; return 0u; }
    const sensorarrayAdsTimingRecord_t *v = &r->records[group][index];
    const ads126xAdcTailEvidence_t *t = &v->tail;
    char prefix[15] = "unknown";
    if (t->spiComplete) {
        for (unsigned i = 0u; i < sizeof(t->spi55); ++i) {
            (void)snprintf(prefix + i * 2u, sizeof(prefix) - i * 2u, "%02X", t->spi55[i]);
        }
    }
    return sensorarrayAdsTimingFormatted(b, n, snprintf(b, n,
        "ADSTIMINGTAIL,diagnostic=1,probe=TAIL,id=%lu,g=%lu,attempt=%lu,source=%s,tailApplicable=%u,spiBits=%u,lsbKnown=0,prefix=%s,gpioKnown=%u,gpioBit=%u,assembledKnown=%u,assembledCrc=%02X,attempted=%u,spiDone=%u,frameDone=%u,routeRestored=%u,physicalVerified=0,hz=%lu,actualHz=%lu,startUs=%llu,spiUs=%lu,riseUs=%lu,sampleUs=%lu,fallUs=%lu,restoreUs=%lu\n",
        (unsigned long)r->requestId, (unsigned long)group, (unsigned long)v->attempt,
        r->groups[group].gpioTail ? "spi55_gpio1" : "spi56",
        r->groups[group].gpioTail, r->groups[group].gpioTail ? 55u : 56u,
        prefix, t->gpioSampled, t->gpioBit, t->frameComplete, t->assembledCrc,
        t->attempted, t->spiComplete, t->frameComplete, t->routeRestored,
        (unsigned long)t->requestedHz, (unsigned long)t->actualHz,
        (unsigned long long)t->startedUs, (unsigned long)t->spiDoneUs,
        (unsigned long)t->riseUs, (unsigned long)t->sampleUs,
        (unsigned long)t->fallUs, (unsigned long)t->restoredUs));
}

size_t sensorarrayAdsTimingProbeFormatTailIoRecord(const sensorarrayAdsTimingProbeResult_t *r,
    uint32_t index, char *b, size_t n)
{
    if (!r || !b || !n || r->kind != SENSORARRAY_ADS_TIMING_PROBE_TAIL) { return 0u; }
    uint32_t group = 0u;
    while (group < SENSORARRAY_ADS_TIMING_GROUPS && index >= r->groups[group].recordCount) {
        index -= r->groups[group++].recordCount;
    }
    if (group >= SENSORARRAY_ADS_TIMING_GROUPS) { b[0] = '\0'; return 0u; }
    const sensorarrayAdsTimingRecord_t *v = &r->records[group][index];
    const ads126xAdcTailEvidence_t *t = &v->tail;
    const uint32_t c = t->clockIo, m = t->misoIo;
    return sensorarrayAdsTimingFormatted(b, n, snprintf(b, n,
        "ADSTIMINGIO,diagnostic=1,probe=TAIL,id=%lu,g=%lu,attempt=%lu,source=%s,stage=%s,clockKnown=%u,misoKnown=%u,clkFunc=%u,clkSig=%u,clkIE=%u,clkOE=%u,clkPeriph=%u,clkInv=%u,clkOD=%u,misoFunc=%u,misoSig=%u,misoIE=%u,misoOE=%u,misoPeriph=%u,misoInv=%u,misoOD=%u,clkLevelKnown=%u,clkLevel=%u,physicalVerified=0\n",
        (unsigned long)r->requestId, (unsigned long)group, (unsigned long)v->attempt,
        r->groups[group].gpioTail ? "spi55_gpio1" : "spi56",
        r->groups[group].gpioTail ? ads126xAdcTailStageName(t->stage) : "not_applicable",
        t->clockIoKnown, t->misoIoKnown,
        (unsigned)((c >> 5u) & 7u), (unsigned)((c >> 8u) & 0x1FFu),
        (unsigned)(c & 1u), (unsigned)((c >> 1u) & 1u), (unsigned)((c >> 2u) & 1u),
        (unsigned)((c >> 3u) & 1u), (unsigned)((c >> 4u) & 1u),
        (unsigned)((m >> 5u) & 7u), (unsigned)((m >> 8u) & 0x1FFu),
        (unsigned)(m & 1u), (unsigned)((m >> 1u) & 1u), (unsigned)((m >> 2u) & 1u),
        (unsigned)((m >> 3u) & 1u), (unsigned)((m >> 4u) & 1u),
        t->clockIoKnown && t->misoIoKnown, t->clockLevel));
}

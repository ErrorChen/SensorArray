/* Synthetic wire/RTOS only: actual complete ADS driver, timing module and Gap
 * wrapper execute below. This is not ADC settling, a wire capture or HIL. */
#define main adsDriverStandaloneTests
#include "driver_bus.h"
#undef main
#include "core/measure/ads/sensorarrayAdsTimingProbe.c"
#include "core/measure/ads/sensorarrayAdsCache.c"
#include "sensorarrayAdsFault.h"

typedef struct {
    ads126xAdcHandle_t ads;
    bool adsReady;
    bool adsAdc1Running;
} sensorarrayState_t;
/* PRODUCTION_GAP_TYPES */
static sensorarrayAdsTransactionEvidence_t s_checkTransaction;
static uint32_t s_transactionGeneration;
static bool s_batteryRestoreFailureLatched;
/* PRODUCTION_GAP_FUNCTIONS */

enum timingCase {
    GOOD, CRC_BOTH_DIRECTIONS, ALL_CRC, PULSE_FENCE_MISSED, EDGE_AFTER_FENCE,
    PULSE_TIMEOUT, CRC_THEN_TIMEOUT, CRC_CONFIG_CHANGE, RESET_STATUS,
    DMA_UNKNOWN, RESTORE_MODE_ERROR, CRC_RESTORE_ERROR, RESTORE_CAL_ERROR,
    BASELINE_UNSTABLE, PROFILE_MISMATCH, OWNER_BUSY, INTREF_ACTIVE,
    START_HIGH, BUDGET_EXHAUSTED, GENERATION_WRAP, HELD_NEW_DATA,
    HELD_CHANGED_RAW, HELD_BAD_CRC, CLOCK_CHANGE, BASELINE_SHADOW,
};
static enum timingCase selected;
static bool pulsePending, continuousRunning, dataNew, restoreStarted, afterFenceArmed;
static uint32_t startConfiguration, sampleSerial, primaryReads, heldReads;
static uint32_t allWrites, powerOnWrites, pulseStarts, pulseStopsIgnored, exitModeWrites;
static uint32_t snapshotCalls;
static unsigned quarantineQueueCalls;
static int64_t pulseDue;
static int32_t holdingRaw;

static void newConversion(void)
{
    ++notifyHandle->drdyGeneration;
    ++sampleSerial;
    holdingRaw = (int32_t)(12345u + sampleSerial * 17u);
    uint8_t data[4];
    for (;;) {
        uint32_t raw = (uint32_t)holdingRaw;
        data[0] = raw >> 24u; data[1] = raw >> 16u;
        data[2] = raw >> 8u; data[3] = raw;
        if ((ads126xAdcCrc8(data, sizeof(data)) & 1u) == (sampleSerial & 1u)) { break; }
        holdingRaw++;
    }
    dataNew = true;
    pulsePending = false;
}

int64_t esp_timer_get_time(void)
{
    if (afterFenceArmed && notifyHandle &&
        notifyHandle->configurationGeneration != startConfiguration) {
        afterFenceArmed = false;
        nowUs = pulseDue;
        newConversion();
    }
    return nowUs;
}

int gpio_get_level(gpio_num_t pin)
{
    return selected == START_HIGH && pin == CONFIG_SENSORARRAY_ADS_START_GPIO ? 1 : 0;
}

uint32_t ulTaskNotifyTake(BaseType_t clear, TickType_t ticks)
{
    (void)clear;
    if (!ticks) { return 0u; }
    if ((selected == PULSE_TIMEOUT || selected == CRC_THEN_TIMEOUT) &&
        (registers[3] & 0x40u)) {
        nowUs += (int64_t)ticks * (1000000u / configTICK_RATE_HZ);
        return 0u;
    }
    if ((registers[3] & 0x40u) ? pulsePending : continuousRunning) {
        if (pulsePending && nowUs < pulseDue) { nowUs = pulseDue; }
        else { nowUs += 27u; }
        newConversion();
        return 1u;
    }
    nowUs += (int64_t)ticks * (1000000u / configTICK_RATE_HZ);
    return 0u;
}

static void timingQueueHook(const spi_transaction_t *t)
{
    completionDelayUs = (uint32_t)((t->length * 1000000u + actualSpiHz - 1u) / actualSpiHz);
    const uint8_t *tx = t->tx_buffer;
    if (selected == DMA_UNKNOWN && tx[0] == ADS126X_CMD_RDATA1) {
        completionBlocked = true;
        quarantineQueueCalls = queueCalls;
    }
}

static bool timingTransactionHook(const uint8_t *tx, uint8_t *rx, size_t bytes)
{
    if ((tx[0] & 0xE0u) == ADS126X_CMD_RREG && (tx[0] & 0x1Fu) == 0u) {
        snapshotCalls++;
        if (selected == BASELINE_UNSTABLE && snapshotCalls == 1u) {
            memcpy(rx + 2u, registers, bytes - 2u); rx[5] ^= 1u; return true;
        }
    }
    if ((tx[0] & 0xE0u) == ADS126X_CMD_WREG) {
        uint8_t reg = tx[0] & 0x1Fu;
        allWrites++;
        if (reg == 1u && (tx[2] & ADS126X_POWER_INTREF)) { powerOnWrites++; }
        if (reg == 3u && tx[2] == 0u && primaryReads >= 800u) { restoreStarted = true; }
        if (reg == 3u && (registers[3] & 0x40u) && tx[2] == 0u) {
            exitModeWrites++;
            pulsePending = false; continuousRunning = true;
        }
        memcpy(registers + reg, tx + 2u, tx[1] + 1u);
        if (reg == 3u && tx[2] == 0x40u && selected == GENERATION_WRAP) {
            notifyHandle->drdyGeneration = UINT32_MAX;
        }
        if (reg == 3u && restoreStarted &&
            (selected == RESTORE_MODE_ERROR || selected == CRC_RESTORE_ERROR)) {
            registers[3] ^= 1u;
        }
        if (selected == RESTORE_CAL_ERROR && reg == 7u) { registers[7] ^= 1u; }
        if (selected == PROFILE_MISMATCH && reg == 6u) { registers[6] ^= 1u; }
        return true;
    }
    if (tx[0] == ADS126X_CMD_START1) {
        dataNew = false;
        if (registers[3] & 0x40u) {
            pulseStarts++; pulsePending = true; pulseDue = nowUs + 207u;
            continuousRunning = false;
            if (selected == PULSE_FENCE_MISSED && pulseStarts == 1u) {
                nowUs = pulseDue; newConversion();
            }
            if (selected == EDGE_AFTER_FENCE && pulseStarts == 1u) {
                startConfiguration = notifyHandle->configurationGeneration;
                afterFenceArmed = true;
            }
        } else { continuousRunning = true; pulsePending = false; }
        return true;
    }
    if (tx[0] == ADS126X_CMD_STOP1) {
        if ((registers[3] & 0x40u) && pulsePending) { pulseStopsIgnored++; }
        else { continuousRunning = false; }
        return true;
    }
    if (tx[0] != ADS126X_CMD_RDATA1) { return false; }
    assert(bytes == 7u);
    bool held = !dataNew;
    if (held) { heldReads++; } else { primaryReads++; }
    int32_t raw = holdingRaw;
    if (selected == HELD_CHANGED_RAW && held) { raw++; }
    uint32_t word = (uint32_t)raw;
    rx[1] = dataNew ? ADS126X_STATUS_ADC1_NEW_DATA : 0u;
    if (selected == RESET_STATUS) { rx[1] |= ADS126X_STATUS_RESET_OCCURRED; }
    if (selected == HELD_NEW_DATA && held) { rx[1] |= ADS126X_STATUS_ADC1_NEW_DATA; }
    rx[2] = word >> 24u; rx[3] = word >> 16u; rx[4] = word >> 8u; rx[5] = word;
    rx[6] = ads126xAdcCrc8(rx + 2u, 4u);
    bool corrupt = selected == ALL_CRC ||
        (selected == CRC_BOTH_DIRECTIONS && primaryReads <= 2u && !held) ||
        ((selected == CRC_THEN_TIMEOUT || selected == CRC_CONFIG_CHANGE ||
          selected == CRC_RESTORE_ERROR) && primaryReads == 1u && !held) ||
        (selected == HELD_BAD_CRC && held);
    if (corrupt) { rx[6] ^= 1u; }
    if (selected == CRC_CONFIG_CHANGE && corrupt) { notifyHandle->configurationGeneration++; }
    if (selected == CLOCK_CHANGE && primaryReads == 199u) { ignoreClockOverride = true; actualSpiHz = 500000u; }
    if (selected == BUDGET_EXHAUSTED && primaryReads == 100u) { nowUs += 1490000u; }
    dataNew = false;
    return true;
}

static sensorarrayState_t timingState(enum timingCase test)
{
    sensorarrayState_t state = {.ads = handle(), .adsReady = true, .adsAdc1Running = true};
    selected = test;
    pulsePending = dataNew = restoreStarted = afterFenceArmed = false;
    continuousRunning = true;
    sampleSerial = primaryReads = heldReads = allWrites = powerOnWrites = 0u;
    pulseStarts = pulseStopsIgnored = exitModeWrites = snapshotCalls = 0u;
    memset(&s_checkTransaction, 0, sizeof(s_checkTransaction));
    s_transactionGeneration = 0u; s_batteryRestoreFailureLatched = false;
    state.ads.idRegRaw = registers[0] = 3u;
    state.ads.deviceType = ADS126X_DEVICE_ADS1262;
    state.ads.requestedSpiClockHz = 550000u;
    state.ads.adc1Running = true; state.ads.mode0 = state.ads.mode1 = 0u;
    state.ads.pgaBypassed = true; state.ads.dataRateDr = 15u;
    registers[6] = 0x7Au; registers[12] = 0x40u;
    if (test == INTREF_ACTIVE) { registers[1] |= 1u; state.ads.enableInternalRef = true; }
    transactionHook = timingTransactionHook; queueHook = timingQueueHook;
    return state;
}

static void runCase(enum timingCase test)
{
    sensorarrayState_t state = timingState(test);
    notifyHandle = &state.ads;
    sensorarrayAdsRegisterCache_t cache = {0};
    ads126xAdc1RegisterSnapshot_t saved;
    ads126xAdcDecodeAdc1RegisterSnapshot(registers, &saved);
    sensorarrayAdsPopulateRestoredRegisterCache(&cache, &saved, state.ads.vrefMicrovolts, true);
    if (test == OWNER_BUSY) { cache.owner = SENSORARRAY_ADS_OWNER_MATRIX; }
    if (test == BASELINE_SHADOW) { cache.registers[SENSORARRAY_ADS_REGISTER_MODE2].value ^= 0x80u; }
    sensorarrayAdsTimingProbeResult_t result;
    uint64_t start = nowUs;
    esp_err_t err = sensorarrayAdsGapRunTimingProbe(&state, &cache, 42u, 123u, &result);
    assert(powerOnWrites == 0u);
    assert(state.ads.transactionDeadlineUs == 0u);
    assert(state.ads.drdyWaitTask == NULL);
    assert(cache.owner == (test == OWNER_BUSY ? SENSORARRAY_ADS_OWNER_MATRIX : SENSORARRAY_ADS_OWNER_NONE));
    assert(result.completedUs - start <= SENSORARRAY_ADS_TIMING_BUDGET_US + 10000u);
    assert(sensorarrayAdsTimingProbeRecordCount(&result) <= 32u);
    char line[512];
    assert(sensorarrayAdsTimingProbeFormatSummary(&result, line, sizeof(line)) > 0u);
    assert(strstr(line, "diagnostic=1,science=0"));
    for (unsigned i = 0u; i < 4u; ++i) {
        assert(sensorarrayAdsTimingProbeFormatGroup(&result, i, line, sizeof(line)) > 0u);
        assert(sensorarrayAdsTimingProbeFormatGroupTiming(&result, i, line, sizeof(line)) > 0u);
    }
    assert(sensorarrayAdsTimingProbeFormatRegisters(&result, false, line, sizeof(line)) > 0u);
    assert(sensorarrayAdsTimingProbeFormatRegisters(&result, true, line, sizeof(line)) > 0u);
    for (unsigned i = 0u; i < sensorarrayAdsTimingProbeRecordCount(&result); ++i) {
        assert(sensorarrayAdsTimingProbeFormatRecord(&result, i, line, sizeof(line)) > 0u);
    }
    bool complete = test == GOOD || test == CRC_BOTH_DIRECTIONS || test == ALL_CRC ||
        test == EDGE_AFTER_FENCE || test == RESTORE_MODE_ERROR || test == CRC_RESTORE_ERROR ||
        test == RESTORE_CAL_ERROR || test == GENERATION_WRAP || test == HELD_BAD_CRC;
    assert(result.completed == complete);
    bool restoreFailed = test == DMA_UNKNOWN || test == RESTORE_MODE_ERROR ||
        test == CRC_RESTORE_ERROR || test == RESTORE_CAL_ERROR || test == CLOCK_CHANGE;
    if (restoreFailed) {
        assert(!result.restoreVerified && result.cleanupError != ESP_OK && result.requiresIsolation);
        assert(s_batteryRestoreFailureLatched);
    } else if (result.savedKnown && (result.mutated || !result.requiresIsolation)) {
        assert(result.restoreVerified && result.cleanupError == ESP_OK);
    }
    if (test == GOOD || test == EDGE_AFTER_FENCE || test == GENERATION_WRAP) {
        assert(err == ESP_OK && result.completed && result.restoreVerified);
        assert(primaryReads == 800u && heldReads == 400u && pulseStarts == 400u);
        assert(!result.requiresIsolation && !pulsePending);
        for (unsigned i = 0u; i < 4u; ++i) {
            assert(result.groups[i].primary.valid == 200u);
            assert(result.groups[i].primary.even >= 32u && result.groups[i].primary.odd >= 32u);
        }
    }
    if (test == CRC_BOTH_DIRECTIONS) {
        assert(err == ESP_ERR_INVALID_CRC && result.completed && !result.requiresIsolation);
        assert(result.groups[0].primary.xorUp == 1u && result.groups[0].primary.xorDown == 1u);
        assert(result.groups[0].primary.valid == 198u);
    }
    if (test == ALL_CRC) {
        assert(result.groups[0].primary.crc == 200u && result.groups[0].primary.valid == 0u);
        assert(result.groups[1].held.crc == 200u && !result.requiresIsolation);
    }
    if (test == PULSE_FENCE_MISSED) {
        assert(result.groups[1].startFenceMissed == 1u && result.groups[1].primary.attempted == 0u);
        assert(!result.requiresIsolation && result.restoreVerified);
    }
    if (test == PULSE_TIMEOUT || test == CRC_THEN_TIMEOUT) {
        assert(result.terminalError == ESP_ERR_TIMEOUT && exitModeWrites != 0u);
        assert(pulseStopsIgnored == 0u && !pulsePending);
    }
    if (test == CRC_THEN_TIMEOUT || test == CRC_CONFIG_CHANGE || test == CRC_RESTORE_ERROR) {
        assert(result.firstError == ESP_ERR_INVALID_CRC && result.requiresIsolation);
    }
    if (test == DMA_UNKNOWN) {
        assert(state.ads.spiQuarantined && pending == &state.ads.dmaTransaction);
        assert(queueCalls == quarantineQueueCalls); /* No restore or descriptor reuse after unknown DMA. */
        assert(state.ads.spiInFlight && !result.restoreKnown);
        pending = NULL; state.ads.spiInFlight = false; /* synthetic completion only, before freeing test memory */
    }
    if (test == OWNER_BUSY || test == INTREF_ACTIVE || test == START_HIGH ||
        test == BASELINE_UNSTABLE || test == BASELINE_SHADOW) {
        assert(!result.mutated && allWrites == 0u);
    }
    if (test == BUDGET_EXHAUSTED) { assert(!result.completed && result.restoreVerified); }
    if (test == HELD_BAD_CRC) {
        assert(result.groups[1].primary.valid == 200u && result.groups[1].primary.crc == 0u);
        assert(result.groups[1].held.crc == 200u && result.firstError == ESP_ERR_INVALID_CRC);
    }
    if (result.mutated && result.restoreVerified) {
        ads126xAdc1RegisterSnapshot_t restored;
        ads126xAdcDecodeAdc1RegisterSnapshot(registers, &restored);
        assert(sensorarrayAdsBatterySnapshotsMatch(&saved, &restored, NULL));
        assert(state.ads.mode0 == saved.mode0 && state.ads.mode1 == saved.mode1);
        if (!result.requiresIsolation) {
            assert(cache.vrefValid && cache.registers[SENSORARRAY_ADS_REGISTER_INPMUX].value == saved.inpmux);
        } else { assert(!cache.vrefValid); }
    }
    printf("PASS timing case=%u complete=%u first=0x%x terminal=0x%x cleanup=0x%x us=%llu\n",
        test, result.completed, result.firstError, result.terminalError, result.cleanupError,
        (unsigned long long)(result.completedUs - start));
    free(state.ads.spiTxBuf); free(state.ads.spiRxBuf);
}

static void testFormatterBounds(void)
{
    sensorarrayAdsTimingProbeResult_t r = {0};
    r.requestId = r.frameSequence = r.requestedHz = r.actualHz = UINT32_MAX;
    r.generationBefore = r.generationAfter = r.crcGoodBefore = r.crcGoodAfter = UINT32_MAX;
    r.crcBadBefore = r.crcBadAfter = UINT32_MAX;
    r.startedUs = r.completedUs = UINT64_MAX;
    r.firstError = r.terminalError = r.cleanupError = INT_MAX;
    r.reason = "requires_passive_profile";
    char line[512], shortLine[16];
    size_t maximum = sensorarrayAdsTimingProbeFormatSummary(&r, line, sizeof(line));
    assert(maximum > 0u);
    assert(sensorarrayAdsTimingProbeFormatSummary(&r, shortLine, sizeof(shortLine)) == 0u);
    assert(shortLine[0] == '\0');
    for (unsigned i = 0u; i < 4u; ++i) {
        sensorarrayAdsTimingGroup_t *g = &r.groups[i];
        g->reason = "crc_above_25permille"; g->pulse = g->bypass = true;
        g->startedUs = g->completedUs = UINT64_MAX;
        g->configurationGeneration = UINT32_MAX;
        g->recordCount = 8u; g->recordsOmitted = 4u;
        g->primary = g->held = (sensorarrayAdsTimingReadStats_t){
            .attempted=200u,.valid=200u,.crc=200u,.even=200u,.odd=200u,
            .xorUp=200u,.xorDown=200u,.xorOther=200u,.status=200u,.freshness=200u,.timeout=200u};
        size_t length = sensorarrayAdsTimingProbeFormatGroup(&r, i, line, sizeof(line));
        assert(length > 0u); if (length > maximum) { maximum = length; }
        assert(sensorarrayAdsTimingProbeFormatGroupTiming(&r, i, line, sizeof(line)) > 0u);
        for (unsigned j = 0u; j < 8u; ++j) {
            r.records[i][j] = (sensorarrayAdsTimingRecord_t){
                .attempt=200u,.configurationGeneration=UINT32_MAX,.generationBefore=UINT32_MAX,
                .generationFence=UINT32_MAX,.generationAfter=UINT32_MAX,.observedUs=UINT64_MAX,
                .startCallUs=UINT32_MAX,.waitUs=UINT32_MAX,.readUs=UINT32_MAX,
                .raw=INT32_MIN,.error=INT_MAX,.status=0xFFu,.computed=0xFFu,.received=0xFEu,
                .crcParity=1u,.length=6u,.held=true,.bytes={0xFF,0x80,0,0,0,0xFE}};
        }
    }
    for (unsigned i = 0u; i < 32u; ++i) {
        size_t length = sensorarrayAdsTimingProbeFormatRecord(&r, i, line, sizeof(line));
        assert(length > 0u); if (length > maximum) { maximum = length; }
    }
    assert(sensorarrayAdsTimingProbeFormatRecord(&r, 32u, line, sizeof(line)) == 0u && !line[0]);
    printf("PASS timing formatter maximum=%zu capacity=512 records=%lu\n", maximum,
        (unsigned long)sensorarrayAdsTimingProbeRecordCount(&r));
}

int main(void)
{
    setbuf(stdout, NULL);
    for (unsigned test = GOOD; test <= BASELINE_SHADOW; ++test) { runCase((enum timingCase)test); }
    testFormatterBounds();
    puts("ADS timing probe production driver/module/Gap: 25 scenarios + formatter PASS (synthetic, no HIL)");
    return 0;
}

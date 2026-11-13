/* Synthetic SPI/DRDY bus executes the complete production probe + driver.
 * No raw measurement, physical settling, or HIL claims are made here. */
#define main adsDriverStandaloneTests
#include "test_ads_driver_reliability.c"
#undef main
#include "../../core/board/sensorarrayAdsSpiProbe.c"

enum probeScenario {
    PROBE_GOOD, PROBE_HIGH_CRC, PROBE_ALL_CRC, PROBE_CONFIRM_CRC,
    PROBE_ODD_ONLY, PROBE_SATURATED, PROBE_STATUS,
    PROBE_RESTORE_ERROR, PROBE_IGNORED_CLOCK, PROBE_DMA_UNKNOWN,
    PROBE_DRDY_TIMEOUT, PROBE_REGISTER_ERROR, PROBE_BYPASS_CRC,
    PROBE_BUDGET, PROBE_CRC_BOUNDARY, PROBE_CRC_SCREEN_OVER,
    PROBE_CRC_CONFIRM_OVER,
    PROBE_BASELINE_UNSTABLE, PROBE_REGISTER_UNSTABLE, PROBE_RESTORE_UNSTABLE,
};
static enum probeScenario scenario;
static uint32_t samplesAtClock, lastClock, sampleSerial, powerOnWrites;
static bool restoreSeen;
static uint32_t samplesInPass, passAtClock;
static unsigned fullSnapshotReads;
static bool restoreSnapshotCorrupted;

static void probeQueueHook(const spi_transaction_t *trans)
{
    const uint8_t *tx = trans->tx_buffer;
    if ((tx[0] & 0xE0u) == 0x40u && (tx[0] & 0x1Fu) == 1u &&
        (tx[2] & ADS126X_POWER_INTREF) != 0u) { powerOnWrites++; }
    if (scenario == PROBE_DMA_UNKNOWN && tx[0] == ADS126X_CMD_RDATA1) {
        completionBlocked = true;
    }
}

static bool probeTransactionHook(const uint8_t *tx, uint8_t *rx, size_t bytes)
{
    if (tx[0] == 0x20u && tx[1] == 16u) {
        fullSnapshotReads++;
        bool corruptSnapshot =
            (scenario == PROBE_BASELINE_UNSTABLE && fullSnapshotReads == 1u) ||
            (scenario == PROBE_REGISTER_UNSTABLE && actualSpiHz != 125000u &&
                !restoreSeen && (fullSnapshotReads % 3u) == 2u) ||
            (scenario == PROBE_RESTORE_UNSTABLE && restoreSeen && !restoreSnapshotCorrupted);
        if (corruptSnapshot) {
            assert(bytes == 19u);
            memcpy(rx + 2u, registers, 17u);
            rx[2u + 3u] ^= 1u;
            if (restoreSeen) { restoreSnapshotCorrupted = true; }
            return true;
        }
    }
    if (tx[0] == ADS126X_CMD_START1) {
        if (lastClock != actualSpiHz) { lastClock = actualSpiHz; samplesAtClock = passAtClock = 0u; }
        samplesInPass = 0u;
        passAtClock++;
    }
    if ((tx[0] & 0xE0u) == 0x40u && (tx[0] & 0x1Fu) == 2u && tx[1] == 4u) {
        restoreSeen = true;
        if (scenario == PROBE_RESTORE_ERROR) {
            /* Genuine restored physical INTERFACE mismatch, not a fake return. */
            memcpy(registers + 2u, tx + 2u, 5u);
            registers[2] ^= 4u;
            return true;
        }
    }
    if (tx[0] != ADS126X_CMD_RDATA1) { return false; }
    assert(bytes == 7u);
    if (lastClock != actualSpiHz) { lastClock = actualSpiHz; samplesAtClock = 0u; }
    samplesAtClock++;
    samplesInPass++;
    sampleSerial++;
    int32_t raw = (int32_t)(sampleSerial * 17u + 1234u);
    if (scenario == PROBE_SATURATED) { raw = INT32_MIN; }
    uint8_t data[4];
    uint8_t crc;
    for (;;) {
        data[0] = (uint8_t)((uint32_t)raw >> 24);
        data[1] = (uint8_t)((uint32_t)raw >> 16);
        data[2] = (uint8_t)((uint32_t)raw >> 8);
        data[3] = (uint8_t)raw;
        crc = ads126xAdcCrc8(data, 4u);
        uint8_t parity = scenario == PROBE_ODD_ONLY ? 1u : sampleSerial & 1u;
        if (scenario == PROBE_SATURATED || (crc & 1u) == parity) { break; }
        raw++;
    }
    rx[1] = ADS126X_STATUS_ADC1_NEW_DATA;
    if (scenario == PROBE_STATUS) { rx[1] |= ADS126X_STATUS_RESET_OCCURRED; }
    memcpy(rx + 2u, data, 4u);
    rx[6] = crc;
    bool corrupt = scenario == PROBE_ALL_CRC ||
        (scenario == PROBE_HIGH_CRC && actualSpiHz > 500000u) ||
        (scenario == PROBE_CONFIRM_CRC && actualSpiHz == 2000000u &&
            samplesAtClock > CONFIG_SENSORARRAY_ADS_SPI_AUTOTUNE_PROBE_SAMPLES) ||
        (scenario == PROBE_BYPASS_CRC && (registers[5] & 0x80u) != 0u);
    uint32_t threshold = passAtClock <= 2u ? 5u : 20u;
    if (scenario == PROBE_CRC_BOUNDARY && samplesInPass <= threshold) { corrupt = true; }
    if (scenario == PROBE_CRC_SCREEN_OVER && actualSpiHz == 2000000u && samplesInPass <= 6u) { corrupt = true; }
    if (scenario == PROBE_CRC_CONFIRM_OVER && actualSpiHz == 2000000u && passAtClock > 2u &&
        samplesInPass <= 21u) { corrupt = true; }
    if (corrupt) { rx[6] ^= 1u; }
    if (scenario == PROBE_REGISTER_ERROR && samplesAtClock == 63u) { registers[15] ^= 1u; }
    if (scenario == PROBE_BUDGET) { nowUs += 5000u; }
    return true;
}

static sensorarrayState_t probeState(enum probeScenario selected)
{
    sensorarrayState_t state = {0};
    state.ads = handle(); state.adsReady = true;
    actualSpiHz = 125000u;
    state.ads.idRegRaw = registers[0] = 3u;
    state.ads.deviceType = ADS126X_DEVICE_ADS1262;
    registers[1] = 0u; registers[2] = 2u; registers[5] = 15u;
    registers[6] = 1u; registers[15] = 0u;
    registers[12] = 0x40u;
    state.ads.enableStatusByte = false;
    scenario = selected;
    samplesAtClock = lastClock = sampleSerial = powerOnWrites = samplesInPass = passAtClock = 0u;
    restoreSeen = false;
    fullSnapshotReads = 0u;
    restoreSnapshotCorrupted = false;
    transactionHook = probeTransactionHook;
    queueHook = probeQueueHook;
    ignoreClockOverride = selected == PROBE_IGNORED_CLOCK;
    notifyEdgesRemaining = selected == PROBE_DRDY_TIMEOUT ? 0u : UINT32_MAX;
    return state;
}

static void probeRunCase(enum probeScenario selected, uint32_t selectedHz, bool restored)
{
    sensorarrayState_t state = probeState(selected);
    notifyHandle = &state.ads;
    uint8_t saved[16]; memcpy(saved, registers, sizeof(saved));
    uint32_t savedVref = state.ads.vrefMicrovolts;
    uint64_t start = (uint64_t)nowUs;
    uint64_t deadline = start + (selected == PROBE_BUDGET ? 850000u : 30000000u);
    sensorarrayAdsSpiProbeResult_t result;
    esp_err_t err = sensorarrayAdsSpiProbeRun(&state, deadline, &result);
    assert(result.selectedHz == selectedHz);
    assert(result.qualified == (selectedHz != 0u && restored));
    assert(result.restored == restored);
    assert((err == ESP_OK) == result.qualified);
    assert(powerOnWrites == 0u && state.ads.enableInternalRef == false);
    assert(result.endUs <= deadline + 1000u);
    assert(state.ads.transactionDeadlineUs == 0u);
    assert(state.ads.drdyWaitTask == NULL);
    if (result.qualified) {
        uint32_t expectedHz = 0u;
        assert(ads126xAdcPredictDefaultSpiClockHz(selectedHz, &expectedHz) == ESP_OK);
        assert(result.actualHz == expectedHz);
        assert(result.attemptedSamples >= CONFIG_SENSORARRAY_ADS_SPI_AUTOTUNE_PROBE_SAMPLES +
            CONFIG_SENSORARRAY_ADS_SPI_AUTOTUNE_CONFIRM_SAMPLES);
        assert(result.validSamples + result.crcErrors == result.attemptedSamples);
        assert(result.crcValidEven >= 4u * CONFIG_SENSORARRAY_ADS_SPI_AUTOTUNE_MIN_PARITY_SAMPLES);
        assert(result.crcValidOdd >= 4u * CONFIG_SENSORARRAY_ADS_SPI_AUTOTUNE_MIN_PARITY_SAMPLES);
    }
    if (restored) {
        assert(memcmp(saved, registers, sizeof(saved)) == 0);
        assert(state.ads.vrefMicrovolts == savedVref && !state.ads.adc1Running);
        assert(state.ads.enableStatusByte == false && state.ads.crcMode == ADS126X_CRC_CRC8);
    }
    if (selected == PROBE_DMA_UNKNOWN) {
        assert(result.candidatesTested == 1u && ads126xAdcIsQuarantined(&state.ads));
        assert(result.error == ESP_ERR_TIMEOUT && result.cleanupError == ESP_ERR_INVALID_STATE);
        unsigned queued = queueCalls;
        uint8_t originalTx[64]; memcpy(originalTx, state.ads.spiTxBuf, sizeof(originalTx));
        assert(ads126xAdcRequestSpiClockHz(&state.ads, 125000u) == ESP_ERR_INVALID_STATE);
        assert(queueCalls == queued && memcmp(originalTx, state.ads.spiTxBuf, sizeof(originalTx)) == 0);
        assert(ads126xAdcDeinit(&state.ads) == ESP_ERR_INVALID_STATE);
        completionBlocked = false;
    }
    if (selected == PROBE_RESTORE_ERROR || selected == PROBE_RESTORE_UNSTABLE) {
        assert(result.error == ESP_OK && result.cleanupError == ESP_ERR_INVALID_RESPONSE && restoreSeen);
    }
    if (selected == PROBE_IGNORED_CLOCK) { assert(result.candidatesTested == 29u); }
    if (selected == PROBE_ALL_CRC) { assert(result.crcErrors == 29u * 6u); }
    if (selected == PROBE_HIGH_CRC) { assert(result.crcErrors == 20u * 6u && result.candidatesTested == 21u); }
    if (selected == PROBE_CRC_BOUNDARY) {
        assert(result.crcErrors == 50u && result.attemptedSamples == 2000u && result.validSamples == 1950u);
        assert(result.candidatesTested == 1u);
    }
    assert(ads126xAdcDeinit(&state.ads) == ESP_OK);
}

int main(void)
{
#ifdef PROBE_TEST_CONFIG_INSUFFICIENT
    (void)probeRunCase;
    sensorarrayState_t state = probeState(PROBE_GOOD);
    notifyHandle = &state.ads;
    sensorarrayAdsSpiProbeResult_t result;
    assert(sensorarrayAdsSpiProbeRun(&state, nowUs + 30000000u, &result) == ESP_ERR_INVALID_ARG);
    assert(!result.qualified && queueCalls == 0u && !powerOnWrites);
    assert(ads126xAdcDeinit(&state.ads) == ESP_OK);
    puts("ADS_SPI_PROBE_CONFIG_TESTS,passed=1,reason=impossible_crc_and_parity_coverage");
#else
    probeRunCase(PROBE_GOOD, 2000000u, true);
    probeRunCase(PROBE_HIGH_CRC, 500000u, true);
    probeRunCase(PROBE_ALL_CRC, 0u, true);
    probeRunCase(PROBE_CONFIRM_CRC, 1900000u, true);
    probeRunCase(PROBE_ODD_ONLY, 0u, true);
    probeRunCase(PROBE_SATURATED, 0u, true);
    probeRunCase(PROBE_STATUS, 0u, true);
    probeRunCase(PROBE_RESTORE_ERROR, 2000000u, false);
    probeRunCase(PROBE_IGNORED_CLOCK, 125000u, true);
    probeRunCase(PROBE_DMA_UNKNOWN, 0u, false);
    probeRunCase(PROBE_DRDY_TIMEOUT, 0u, true);
    probeRunCase(PROBE_REGISTER_ERROR, 0u, true);
    probeRunCase(PROBE_BYPASS_CRC, 0u, true);
    probeRunCase(PROBE_BUDGET, 0u, true);
    probeRunCase(PROBE_CRC_BOUNDARY, 2000000u, true);
    probeRunCase(PROBE_CRC_SCREEN_OVER, 1900000u, true);
    probeRunCase(PROBE_CRC_CONFIRM_OVER, 1900000u, true);
    probeRunCase(PROBE_BASELINE_UNSTABLE, 0u, false);
    probeRunCase(PROBE_REGISTER_UNSTABLE, 125000u, true);
    probeRunCase(PROBE_RESTORE_UNSTABLE, 2000000u, false);
    puts("ADS_SPI_PROBE_TESTS,passed=20,source=complete_production_probe_and_driver,fixture=synthetic");
#endif
    return 0;
}

/* Production presence implementation and ADC driver on a synthetic SPI bus. */
#define main adsDriverStandaloneTests
#include "test_ads_driver_reliability.c"
#undef main
#include "sensorarray_ads_presence_impl.inc"

static unsigned presenceReads;
static bool unstablePresence;
static unsigned blockPresenceRead;
static bool rejectPresenceRestore;

static void presenceQueueHook(const spi_transaction_t *trans)
{
    const uint8_t *tx = trans->tx_buffer;
    assert(tx[0] == 0x20u && tx[1] == 3u); /* No ADC write/reset/START/INTREF. */
    assert((trans->override_freq_hz == 125000u || trans->override_freq_hz == 2000000u ||
            trans->override_freq_hz == 1900000u || trans->override_freq_hz == 1904000u) &&
        trans->length == 48u);
    presenceReads++;
    assert(sensorarrayBringupAdsPresenceBusy());
    assert(!sensorarrayBringupAdsPresenceQuarantined());
    if (presenceReads == blockPresenceRead) { completionBlocked = true; }
}

static bool presenceTransactionHook(const uint8_t *tx, uint8_t *rx, size_t bytes)
{
    (void)tx;
    assert(bytes == 6u);
    if (rejectPresenceRestore && presenceReads == 2u) { ignoreClockOverride = true; }
    if (!unstablePresence) { return false; }
    memcpy(rx + 2u, registers, 4u);
    if (presenceReads == 2u) { rx[2] ^= 1u; }
    return true;
}

static sensorarrayState_t presenceState(bool live)
{
    ads126xAdcHandle_t ads = handle();
    if (!live) { assert(ads126xAdcDeinit(&ads) == ESP_OK); }
    sensorarrayState_t state = {.ads = ads, .spiDevice = (void *)1, .adsReady = false};
    if (live) { state.ads.idRegRaw = 3u; }
    registers[0] = 3u;
    registers[1] = 0x11u; /* Returned POWER may truthfully show reset/INTREF. */
    registers[2] = 0x05u;
    presenceReads = 0u; unstablePresence = false; blockPresenceRead = 0u;
    rejectPresenceRestore = false;
    queueHook = presenceQueueHook;
    transactionHook = presenceTransactionHook;
    return state;
}

static void presenceFinish(sensorarrayState_t *state)
{
    completionBlocked = wrongDescriptor = false;
    if (state->ads.mutex) { assert(ads126xAdcDeinit(&state->ads) == ESP_OK); }
    else if (pending) {
        spi_transaction_t *done = NULL;
        assert(spi_device_get_trans_result((void *)1, &done, 100u) == ESP_OK);
        assert(done == &s_adsPresenceIo.transaction);
    }
    /* Simulates a new boot after DMA has demonstrably returned. Production
     * intentionally never resets this quarantined slot in the same boot. */
    free(s_adsPresenceIo.tx); free(s_adsPresenceIo.rx);
    memset(&s_adsPresenceIo, 0, sizeof(s_adsPresenceIo));
}

static void testPresenceMode(bool live)
{
    sensorarrayState_t state = presenceState(live);
    sensorarrayAdsPresenceResult_t result;
    assert(sensorarrayBringupProbeAdsPresence(&state, nowUs + 50000u, &result) == ESP_OK);
    assert(result.present && result.known && !result.busy && !result.quarantined);
    assert(result.id == 3u && result.power == 0x11u && result.interface == 5u);
    assert(result.actualHz == 125000u && result.restoredHz == 2000000u &&
        result.cleanupError == ESP_OK && actualSpiHz == 2000000u && presenceReads == 3u && !state.adsReady);
    assert(!sensorarrayBringupAdsPresenceBusy() && !sensorarrayBringupAdsPresenceQuarantined());
    uint8_t *persistentTx = s_adsPresenceIo.tx;
    assert(sensorarrayBringupProbeAdsPresence(&state, nowUs + 50000u, &result) == ESP_OK);
    assert(s_adsPresenceIo.tx == persistentTx && presenceReads == 6u && actualSpiHz == 2000000u);
    presenceFinish(&state);

    state = presenceState(live);
    memset(registers, 0, sizeof(registers));
    assert(sensorarrayBringupProbeAdsPresence(&state, nowUs + 50000u, &result) == ESP_ERR_NOT_FOUND);
    assert(result.known && !result.present);
    assert(result.cleanupError == ESP_OK && actualSpiHz == 2000000u);
    presenceFinish(&state);

    state = presenceState(live);
    memset(registers, 0xFF, sizeof(registers));
    assert(sensorarrayBringupProbeAdsPresence(&state, nowUs + 50000u, &result) == ESP_ERR_NOT_FOUND);
    assert(result.known && !result.present);
    assert(result.cleanupError == ESP_OK && actualSpiHz == 2000000u);
    presenceFinish(&state);

    state = presenceState(live); unstablePresence = true;
    assert(sensorarrayBringupProbeAdsPresence(&state, nowUs + 50000u, &result) == ESP_ERR_INVALID_RESPONSE);
    assert(!result.known && !result.present && presenceReads == 3u && actualSpiHz == 2000000u);
    presenceFinish(&state);

    state = presenceState(live); blockPresenceRead = 2u;
    uint64_t started = (uint64_t)nowUs;
    assert(sensorarrayBringupProbeAdsPresence(&state, started + 50000u, &result) == ESP_ERR_TIMEOUT);
    assert(!result.known && !result.present && result.quarantined && presenceReads == 2u);
    if (!live) {
        assert(sensorarrayBringupAdsPresenceBusy() && sensorarrayBringupAdsPresenceQuarantined());
    }
    assert((uint64_t)nowUs <= started + 51000u && pending != NULL);
    spi_transaction_t *descriptor = pending;
    uint8_t oldTx[6]; memcpy(oldTx, pending->tx_buffer, sizeof(oldTx));
    unsigned queued = queueCalls;
    assert(sensorarrayBringupProbeAdsPresence(&state, nowUs + 50000u, &result) == ESP_ERR_INVALID_STATE);
    assert(result.quarantined && queueCalls == queued && pending == descriptor);
    assert(memcmp(oldTx, pending->tx_buffer, sizeof(oldTx)) == 0);
    presenceFinish(&state);

    state = presenceState(live); ignoreClockOverride = true;
    assert(sensorarrayBringupProbeAdsPresence(&state, nowUs + 50000u, &result) == ESP_ERR_INVALID_RESPONSE);
    assert(!result.present && !result.known && result.actualHz == 2000000u);
    presenceFinish(&state);

    state = presenceState(live);
    actualSpiHz = 1904761u;
    if (live) { state.ads.requestedSpiClockHz = 1900000u; }
    assert(sensorarrayBringupProbeAdsPresence(&state, nowUs + 50000u, &result) == ESP_OK);
    assert(result.present && result.restoredHz == 1904000u);
    if (live) { assert(state.ads.requestedSpiClockHz == 1900000u && actualSpiHz == 1904761u); }
    presenceFinish(&state);

    state = presenceState(live); blockPresenceRead = 3u;
    assert(sensorarrayBringupProbeAdsPresence(&state, nowUs + 50000u, &result) == ESP_ERR_TIMEOUT);
    assert(result.error == ESP_OK && result.cleanupError == ESP_ERR_TIMEOUT &&
        result.quarantined && !result.known && !result.present && presenceReads == 3u);
    presenceFinish(&state);

    state = presenceState(live); rejectPresenceRestore = true;
    if (live) { state.ads.requestedSpiClockHz = 2000000u; }
    assert(sensorarrayBringupProbeAdsPresence(&state, nowUs + 50000u, &result) == ESP_ERR_INVALID_RESPONSE);
    assert(result.cleanupError == ESP_ERR_INVALID_RESPONSE && !result.known && actualSpiHz == 125000u);
    assert(sensorarrayBringupProbeAdsPresence(&state, nowUs + 50000u, &result) == ESP_ERR_INVALID_RESPONSE);
    assert(result.cleanupError == ESP_ERR_INVALID_RESPONSE && !result.known && !result.present && actualSpiHz == 125000u);
    /* A later genuinely applied restore repairs the original target rather
     * than blessing the failed 125-kHz entry value as a replacement target. */
    ignoreClockOverride = rejectPresenceRestore = false;
    assert(sensorarrayBringupProbeAdsPresence(&state, nowUs + 50000u, &result) == ESP_OK);
    assert(result.present && result.known && result.restoredHz == 2000000u && actualSpiHz == 2000000u);
    presenceFinish(&state);
}

int main(void)
{
    testPresenceMode(true);
    testPresenceMode(false);
    sensorarrayState_t state = presenceState(true);
    sensorarrayAdsPresenceResult_t result;
    state.ads.transactionDeadlineUs = nowUs + 10000u;
    assert(sensorarrayBringupProbeAdsPresence(&state, nowUs + 50000u, &result) == ESP_ERR_INVALID_STATE);
    assert(result.busy && presenceReads == 0u && state.ads.transactionDeadlineUs != 0u);
    ads126xAdcEndTransaction(&state.ads);
    s_adsPresenceIo.busy = true;
    assert(sensorarrayBringupProbeAdsPresence(&state, nowUs + 50000u, &result) == ESP_ERR_INVALID_STATE);
    assert(result.busy && presenceReads == 0u);
    s_adsPresenceIo.busy = false;
    assert(sensorarrayBringupProbeAdsPresence(&state, nowUs, &result) == ESP_ERR_TIMEOUT);
    assert(presenceReads == 0u);
    presenceFinish(&state);
    puts("ADS_PRESENCE_TESTS,passed=21,source=production_presence_and_driver,fixture=synthetic");
    return 0;
}

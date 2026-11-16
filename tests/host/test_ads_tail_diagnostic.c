/* Real driver/functions; fake hardware does not validate physical routing. */
#define main ordinaryDriverTests
#include "test_ads_driver_reliability.c"
#undef main

static bool presenceBusy, presenceQuarantined;
static esp_err_t busInitError, busAddError;
static spi_device_handle_t s_adsDedicatedSpiDevice;
bool sensorarrayBringupAdsPresenceBusy(void) { return presenceBusy; }
bool sensorarrayBringupAdsPresenceQuarantined(void) { return presenceQuarantined; }
esp_err_t spi_bus_initialize(int host, const spi_bus_config_t *config, int dma)
{
    assert(host == 2 && config->sclk_io_num == 47 && dma == SPI_DMA_CH_AUTO);
    return busInitError;
}
esp_err_t spi_bus_add_device(int host, const spi_device_interface_config_t *config,
    spi_device_handle_t *out)
{
    assert(host == 2 && config->mode == 1 && config->spics_io_num == -1 && config->queue_size == 1);
    if (busAddError == ESP_OK) { *out = (void *)1; }
    return busAddError;
}
/* PRODUCTION_BOARD_OWNERSHIP */

static void prepareFrame(ads126xAdcHandle_t *h, uint8_t value)
{
    notifyHandle = h;
    notifyEdge = true;
    dataFrameLimit = 1u;
    dataFrames[0][0] = ADS126X_STATUS_ADC1_NEW_DATA;
    dataFrames[0][1] = 0x12u; dataFrames[0][2] = 0x34u;
    dataFrames[0][3] = 0x56u; dataFrames[0][4] = value;
    dataFrames[0][5] = ads126xAdcCrc8(dataFrames[0] + 1u, 4u);
    assert(ads126xAdcBeginTransaction(h, 100000u) == ESP_OK);
}

static void testAllCrcBytes(void)
{
    bool seen[256] = {false};
    for (unsigned i = 0u; i < 256u; ++i) {
        ads126xAdcHandle_t h = handle(); prepareFrame(&h, (uint8_t)i);
        uint8_t crc = dataFrames[0][5]; seen[crc] = true;
        int32_t raw = -1; uint8_t status = 0u;
        ads126xAdcFreshReadReport_t report;
        ads126xAdcTailEvidence_t tail;
        esp_err_t error = ads126xAdcReadAdc1FreshTailDiagnostic(&h, 0u, 600u,
            &raw, &status, &report, &tail);
        if (i == 0u) { printf("REAL_IDF_MISO_INPUT_OUTPUT,ie=1,oe=1,peripheralOE=1,sig=33,err=0x%x,attempted=%u,spiDone=%u\n",
            error, tail.attempted, tail.spiComplete); fflush(stdout); }
        assert(error == ESP_OK);
        assert(raw == (int32_t)(0x12345600u | i) && status == ADS126X_STATUS_ADC1_NEW_DATA);
        assert(h.dmaTransaction.length == 55u && h.dmaTransaction.rxlength == 55u);
        assert(tail.spiComplete && tail.frameComplete && tail.gpioSampled && tail.routeRestored);
        assert(tail.spi55[6] == (crc & 0xFEu) && tail.gpioBit == (crc & 1u));
        assert(tail.assembledCrc == crc && tailEdges == 2u && tailRouteChanges == 2u);
        assert(tailClockConfig.sig_out == 32 && tailSclkLevel == 0 && mutexDepth == 0u);
        assert(tail.sampleUs >= tail.riseUs + 1u && tail.restoredUs >= tail.fallUs + 1u);
        assert(pending == NULL && !ads126xAdcIsQuarantined(&h));
        free(h.spiTxBuf); free(h.spiRxBuf);
    }
    for (unsigned i = 0u; i < 256u; ++i) { assert(seen[i]); }
    puts("PASS tail256 CRC byte patterns:55bit prefix + physical0/1, no computed-bit repair");
}

static void testMisoOutputGates(void)
{
    for (unsigned kind = 0u; kind < 8u; ++kind) {
        ads126xAdcHandle_t h = handle(); prepareFrame(&h, 8u);
        unsigned before = queueCalls;
        uint8_t expectedStage = ADS126X_TAIL_STAGE_NONE;
        switch (kind) {
        case 0u: tailMisoConfig.oe_ctrl_by_periph = false; expectedStage=ADS126X_TAIL_STAGE_MISO_OWNER; break;
        case 1u: tailMisoConfig.sig_out = 256u; expectedStage=ADS126X_TAIL_STAGE_MISO_SIGNAL; break;
        case 2u: tailMisoConfig.oe_inv = true; expectedStage=ADS126X_TAIL_STAGE_MISO_FLAGS; break;
        case 3u: tailMisoConfig.od = true; expectedStage=ADS126X_TAIL_STAGE_MISO_FLAGS; break;
        case 4u: tailMisoConfig.ie = false; expectedStage=ADS126X_TAIL_STAGE_MISO_INPUT; break;
        case 5u: tailMisoConfig.fun_sel = 0u; expectedStage=ADS126X_TAIL_STAGE_MISO_MUX; break;
        case 6u: tailClockConfig.oe_ctrl_by_periph = false; expectedStage=ADS126X_TAIL_STAGE_CLOCK_FLAGS; break;
        case 7u: tailClockConfig.ie = false; expectedStage=ADS126X_TAIL_STAGE_CLOCK_INPUT; break;
        }
        int32_t raw = 0; uint8_t status = 0u;
        ads126xAdcFreshReadReport_t report; ads126xAdcTailEvidence_t tail;
        assert(ads126xAdcReadAdc1FreshTailDiagnostic(&h, 0u, 600u,
            &raw, &status, &report, &tail) == ESP_ERR_INVALID_STATE);
        assert(tail.clockIoKnown && tail.misoIoKnown && tail.stage == expectedStage);
        assert(queueCalls == before && !tail.attempted && tailRouteChanges == 0u && tailEdges == 0u);
        assert(!ads126xAdcIsQuarantined(&h));
        free(h.spiTxBuf); free(h.spiRxBuf);
    }
    puts("PASS8 precise MISO/clock gates reject GPIO-driven/wrong-route/inverted/OD/disabled-input state beforequeue");
}

static void testBadPhysicalBitAndData(void)
{
    for (unsigned kind = 0u; kind < 3u; ++kind) {
        ads126xAdcHandle_t h = handle(); prepareFrame(&h, (uint8_t)kind);
        if (kind < 2u) { tailPhysicalBitOverride = !(dataFrames[0][5] & 1u); }
        else { dataFrames[0][2] ^= 0x10u; }
        int32_t raw = 123; uint8_t status = 45u;
        ads126xAdcFreshReadReport_t report;
        ads126xAdcTailEvidence_t tail;
        assert(ads126xAdcReadAdc1FreshTailDiagnostic(&h, 0u, 600u,
            &raw, &status, &report, &tail) == ESP_ERR_INVALID_CRC);
        assert(raw == 123 && status == 45u && report.crcErrors == 1u && report.crcRetries == 0u);
        assert(h.adc1CrcErrorCount == 1u && h.adc1CrcValidEvenCount + h.adc1CrcValidOddCount == 0u);
        assert(tail.frameComplete && tail.routeRestored && !ads126xAdcIsQuarantined(&h));
        if (kind < 2u) { assert(tail.gpioBit != (dataFrames[0][5] & 1u)); }
        free(h.spiTxBuf); free(h.spiRxBuf);
    }
    puts("PASS bad physical tail bit / real data corruption remains CRC invalid");
}

static void testFailures(void)
{
    for (unsigned kind = 0u; kind < 12u; ++kind) {
        ads126xAdcHandle_t h = handle(); prepareFrame(&h, 7u);
        unsigned queues = queueCalls;
        if (kind == 0u) { tailOwnerAvailable = false; }
        if (kind == 1u) { h.tailBus.exclusiveOwner = NULL; }
        if (kind == 2u) { h.tailBus.host = 1; }
        if (kind == 3u) { tailClockConfig.sig_out = 31; }
        if (kind == 4u) { queueError = ESP_FAIL; }
        if (kind == 5u) { completionBlocked = true; }
        if (kind == 6u) { wrongDescriptor = true; }
        if (kind == 7u) { tailHighFailure = true; }
        if (kind == 8u) { tailRestoreFailure = true; }
        if (kind == 9u) { mutexAvailable = false; }
        if (kind == 10u) { h.transactionDeadlineUs = nowUs; }
        if (kind == 11u) { tailLowReadbackHigh = true; }
        int32_t raw = 123; uint8_t status = 45u;
        ads126xAdcFreshReadReport_t report;
        ads126xAdcTailEvidence_t tail;
        assert(ads126xAdcReadAdc1FreshTailDiagnostic(&h, 0u, 600u,
            &raw, &status, &report, &tail) != ESP_OK);
        assert(raw == 123 && status == 45u && mutexDepth == 0u);
        if (kind <= 3u || kind == 9u || kind == 10u) { assert(queueCalls == queues); }
        if (kind <= 6u || kind == 9u || kind == 10u) { assert(tailEdges == 0u && tailRouteChanges == 0u); }
        bool quarantine = (kind >= 5u && kind <= 8u) || kind == 11u;
        if (kind == 11u) {
            assert(tail.stage == ADS126X_TAIL_STAGE_GPIO_LOW && !tail.frameComplete &&
                tail.spiComplete && tail.gpioSampled && tailRouteChanges == 2u);
        }
        assert(ads126xAdcIsQuarantined(&h) == quarantine);
        if (quarantine) {
            unsigned before = queueCalls;
            uint8_t reg = 0u;
            assert(ads126xAdcReadRegisters(&h, 1u, &reg, 1u) == ESP_ERR_INVALID_STATE);
            assert(queueCalls == before); /* no register restore or DMA reuse */
        }
        pending = NULL; /* synthetic completion before test frees its own memory */
        free(h.spiTxBuf); free(h.spiRxBuf);
    }
    puts("PASS12 tail admission/queue/DMA/route/mutex/deadline failure paths; low-readback first stage retained");
}

static void testCompletedAfterWorkDeadline(void)
{
    ads126xAdcHandle_t h = handle(); prepareFrame(&h, 8u);
    h.transactionDeadlineUs = nowUs + 50u;
    completionDelayUs = 100u;
    ads126xAdcTailEvidence_t tail; uint8_t frame[6];
    assert(ads126xAdcSpiTailDiagnostic(&h, frame, &tail) == ESP_ERR_TIMEOUT);
    assert(tail.spiComplete && tail.frameComplete && tail.routeRestored && tailEdges == 2u);
    assert(!ads126xAdcIsQuarantined(&h) && pending == NULL);
    assert(tail.restoredUs == 102u && mutexDepth == 0u);
    free(h.spiTxBuf); free(h.spiRxBuf);
    puts("PASS known55 completion after work deadline completes framing before timeout; no valid sample");
}

static void testBoardOwnership(void)
{
    spi_device_handle_t device = NULL;
    presenceBusy = presenceQuarantined = false;
    busInitError = busAddError = ESP_OK;
    assert(sensorarrayInitSpi(&device) == ESP_OK && sensorarrayAdsDedicatedSpiOwner(device));
    presenceBusy = true; assert(!sensorarrayAdsDedicatedSpiOwner(device));
    presenceBusy = false; presenceQuarantined = true; assert(!sensorarrayAdsDedicatedSpiOwner(device));
    presenceQuarantined = false;
    busInitError = ESP_ERR_INVALID_STATE;
    assert(sensorarrayInitSpi(&device) == ESP_OK && !sensorarrayAdsDedicatedSpiOwner(device));
    busInitError = ESP_OK; busAddError = ESP_FAIL;
    assert(sensorarrayInitSpi(&device) == ESP_FAIL && !sensorarrayAdsDedicatedSpiOwner(device));
    busInitError = ESP_FAIL;
    assert(sensorarrayInitSpi(&device) == ESP_FAIL && !sensorarrayAdsDedicatedSpiOwner(device));
    puts("PASS actual board init ownership: fresh only, already-initialized/failure/presence rejected");
}

int main(void)
{
    testAllCrcBytes();
    testMisoOutputGates();
    testBadPhysicalBitAndData();
    testFailures();
    testCompletedAfterWorkDeadline();
    testBoardOwnership();
    puts("ADS55+GPIO1 production driver/board: synthetic tests PASS; physical glitch-free NOT_RUN");
    return 0;
}

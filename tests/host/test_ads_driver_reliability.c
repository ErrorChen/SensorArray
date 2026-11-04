/* Synthetic fault injection against the actual production driver. No HIL claims. */
#include <assert.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "../../components/ads126xAdc/ads126xAdc.c"

static int64_t nowUs;
static uint32_t lockDelayUs, queueDelayUs, completionDelayUs;
static bool mutexAvailable = true, completionBlocked, wrongDescriptor;
static unsigned queueCalls, freeCalls, snapshotReads;
static uint8_t registers[32], snapshots[3][32];
static bool useSnapshots;
static esp_err_t queueError;
static spi_transaction_t *pending;
static int64_t completeAtUs;
static ads126xAdcHandle_t *notifyHandle;
static bool notifyEdge, notifyConfigChange;
static unsigned notifyEdgesRemaining, notifyEdgeCount, configChangeOnEdge;
static uint8_t dataFrames[4][6];
static unsigned dataFrameCount, dataFrameLimit, blockCompletionAfterRead;
static uint32_t dataFrameGeneration[4];
static bool forceFinalRegisterBit;
static uint8_t corruptReadRegister = UINT8_MAX;
static unsigned lastRegisterStart, lastRegisterCount;

int64_t esp_timer_get_time(void) { return nowUs; }
void *heap_caps_malloc(size_t size, int flags) { (void)flags; return malloc(size); }
void heap_caps_free(void *ptr) { ++freeCalls; free(ptr); }
bool esp_ptr_dma_capable(const void *ptr) { return ptr != NULL; }
SemaphoreHandle_t xSemaphoreCreateMutex(void) { return (void *)1; }
void vSemaphoreDelete(SemaphoreHandle_t handle) { (void)handle; }
BaseType_t xSemaphoreTake(SemaphoreHandle_t handle, TickType_t ticks)
{
    (void)handle;
    uint64_t budget = (uint64_t)ticks * 1000u;
    if (!mutexAvailable || lockDelayUs > budget) { nowUs += budget; return pdFALSE; }
    nowUs += lockDelayUs; return pdTRUE;
}
BaseType_t xSemaphoreGive(SemaphoreHandle_t handle) { (void)handle; return pdTRUE; }
TaskHandle_t xTaskGetCurrentTaskHandle(void) { return (void *)1; }
BaseType_t xTaskNotifyStateClear(TaskHandle_t task) { (void)task; return pdTRUE; }
uint32_t ulTaskNotifyTake(BaseType_t clear, TickType_t ticks)
{
    (void)clear;
    if (!ticks) { return 0u; }
    if ((notifyEdge || notifyEdgesRemaining != 0u) && notifyHandle) {
        nowUs += 100u;
        ++notifyHandle->drdyGeneration;
        ++notifyEdgeCount;
        if (notifyConfigChange || configChangeOnEdge == notifyEdgeCount) {
            ++notifyHandle->configurationGeneration;
        }
        if (notifyEdgesRemaining != 0u) { --notifyEdgesRemaining; }
        notifyEdge = false;
        return 1u;
    }
    nowUs += (int64_t)ticks * 1000; return 0u;
}
void vTaskDelay(TickType_t ticks) { nowUs += (int64_t)ticks * 1000; }
TickType_t xTaskGetTickCount(void) { return (TickType_t)(nowUs / 1000); }
void vTaskNotifyGiveFromISR(TaskHandle_t task, BaseType_t *woken) { (void)task; *woken = pdFALSE; }
esp_err_t gpio_set_direction(gpio_num_t pin, int mode) { (void)pin; (void)mode; return ESP_OK; }
esp_err_t gpio_set_level(gpio_num_t pin, int level) { (void)pin; (void)level; return ESP_OK; }
int gpio_get_level(gpio_num_t pin) { (void)pin; return 0; }
esp_err_t gpio_config(const gpio_config_t *cfg) { (void)cfg; return ESP_OK; }
esp_err_t gpio_isr_handler_add(gpio_num_t pin, void (*fn)(void *), void *arg)
{ (void)pin; (void)fn; (void)arg; return ESP_OK; }
esp_err_t gpio_isr_handler_remove(gpio_num_t pin) { (void)pin; return ESP_OK; }
esp_err_t sensorarrayBoardEnsureGpioIsrService(void) { return ESP_OK; }
void sensorarrayBoardNoteGpioIsrHandlerAdd(int kind) { (void)kind; }

esp_err_t spi_device_queue_trans(spi_device_handle_t device, spi_transaction_t *trans, TickType_t ticks)
{
    (void)device; ++queueCalls;
    assert(ticks != portMAX_DELAY);
    if (queueError != ESP_OK) { return queueError; }
    uint64_t budget = (uint64_t)ticks * 1000u;
    if (queueDelayUs > budget) { nowUs += budget; return ESP_ERR_TIMEOUT; }
    nowUs += queueDelayUs;
    assert(pending == NULL);
    pending = trans;
    completeAtUs = nowUs + completionDelayUs;
    return ESP_OK;
}

esp_err_t spi_device_get_trans_result(spi_device_handle_t device, spi_transaction_t **out, TickType_t ticks)
{
    (void)device;
    assert(ticks != portMAX_DELAY);
    uint64_t budget = (uint64_t)ticks * 1000u;
    if (!pending || completionBlocked || completeAtUs - nowUs > (int64_t)budget) {
        nowUs += budget; return ESP_ERR_TIMEOUT;
    }
    if (completeAtUs > nowUs) { nowUs = completeAtUs; }
    if (wrongDescriptor) {
        static spi_transaction_t wrong;
        *out = &wrong; return ESP_OK;
    }
    const uint8_t *tx = pending->tx_buffer;
    uint8_t *rx = pending->rx_buffer;
    size_t bytes = pending->length / 8u;
    memset(rx, 0, bytes);
    if ((tx[0] & 0xE0u) == 0x20u) {
        uint8_t reg = tx[0] & 0x1Fu;
        size_t len = tx[1] + 1u;
        assert(bytes == len + 2u);
        lastRegisterStart = reg; lastRegisterCount = len;
        const uint8_t *values = registers;
        if (useSnapshots && reg == 0u && len == 17u) {
            values = snapshots[snapshotReads < 3u ? snapshotReads : 2u];
            ++snapshotReads;
        }
        memcpy(rx + 2, values + reg, len);
        if (forceFinalRegisterBit) { rx[bytes - 1u] |= 1u; }
        if (corruptReadRegister >= reg && corruptReadRegister < reg + len) {
            rx[2u + corruptReadRegister - reg] ^= 1u;
        }
    } else if ((tx[0] & 0xE0u) == 0x40u) {
        uint8_t reg = tx[0] & 0x1Fu;
        memcpy(registers + reg, tx + 2, tx[1] + 1u);
    } else if (tx[0] == ADS126X_CMD_RDATA1) {
        assert(bytes == 7u && dataFrameCount < dataFrameLimit);
        memcpy(rx + 1u, dataFrames[dataFrameCount], 6u);
        dataFrameGeneration[dataFrameCount] = notifyHandle->drdyGeneration;
        ++dataFrameCount;
        if (blockCompletionAfterRead == dataFrameCount) { completionBlocked = true; }
    }
    *out = pending; pending = NULL;
    return ESP_OK;
}

static ads126xAdcHandle_t handle(void)
{
    nowUs = 1000000;
    lockDelayUs = queueDelayUs = completionDelayUs = 0u;
    mutexAvailable = true; completionBlocked = wrongDescriptor = useSnapshots = false;
    queueError = ESP_OK; pending = NULL; snapshotReads = 0u;
    notifyHandle = NULL; notifyEdge = notifyConfigChange = false;
    notifyEdgesRemaining = notifyEdgeCount = configChangeOnEdge = 0u;
    dataFrameCount = dataFrameLimit = blockCompletionAfterRead = 0u;
    forceFinalRegisterBit = false; corruptReadRegister = UINT8_MAX;
    lastRegisterStart = lastRegisterCount = 0u;
    memset(dataFrames, 0, sizeof(dataFrames));
    memset(registers, 0, sizeof(registers));
    registers[1] = 2u; registers[2] = 6u; registers[5] = 0x8Fu; registers[15] = 0x24u;
    ads126xAdcHandle_t result = {0};
    result.spiDevice = (void *)1; result.mutex = (void *)1;
    result.spiTxBuf = malloc(64u); result.spiRxBuf = malloc(64u);
    result.spiBufSize = 64u; result.spiDmaCapable = true;
    result.drdyGpio = 1; result.resetGpio = 2; result.drdyNotificationReady = true;
    result.enableStatusByte = true; result.crcMode = ADS126X_CRC_CRC8;
    result.pgaGain = 1u; result.vrefMicrovolts = 5200000u;
    return result;
}

static void testSpiDeadlineAndLifetime(void)
{
    ads126xAdcHandle_t ads = handle();
    uint8_t value = 0u;
    assert(ads126xAdcReadPowerRegister(&ads, &value) == ESP_OK && value == 2u);
    assert(!ads.spiInFlight);
    lockDelayUs = 90000u; queueDelayUs = 8000u; completionDelayUs = 3000u;
    int64_t start = nowUs;
    assert(ads126xAdcReadPowerRegister(&ads, &value) == ESP_ERR_TIMEOUT);
    assert(nowUs - start == 100000u);
    assert(pending == &ads.dmaTransaction && ads.spiInFlight && ads.spiQuarantined);
    uint8_t prior[64]; memcpy(prior, ads.spiTxBuf, sizeof(prior));
    unsigned count = queueCalls;
    assert(ads126xAdcWritePowerRegister(&ads, 1u) == ESP_ERR_INVALID_STATE);
    assert(queueCalls == count && memcmp(prior, ads.spiTxBuf, sizeof(prior)) == 0);
    completionBlocked = true;
    unsigned freed = freeCalls;
    assert(ads126xAdcDeinit(&ads) == ESP_ERR_INVALID_STATE && freeCalls == freed);
    assert(ads126xAdcHardwareReset(&ads) == ESP_ERR_INVALID_STATE);
    ads126xAdcConfig_t cfg = {.spiDevice = (void *)1};
    assert(ads126xAdcInit(&ads, &cfg) == ESP_ERR_INVALID_STATE);
    completionBlocked = false; nowUs += 1000000;
    assert(ads126xAdcDeinit(&ads) == ESP_OK && freeCalls == freed + 2u);

    ads = handle(); queueDelayUs = 200000u;
    assert(ads126xAdcReadPowerRegister(&ads, &value) == ESP_ERR_TIMEOUT);
    assert(!ads.spiInFlight && !ads.spiQuarantined && pending == NULL);
    queueDelayUs = 0u;
    assert(ads126xAdcReadPowerRegister(&ads, &value) == ESP_OK);
    assert(ads126xAdcDeinit(&ads) == ESP_OK);

    ads = handle();
    assert(ads126xAdcBeginTransaction(&ads, 5000u) == ESP_OK);
    queueDelayUs = 2000u; completionDelayUs = 2000u;
    assert(ads126xAdcReadPowerRegister(&ads, &value) == ESP_OK);
    assert(ads126xAdcReadPowerRegister(&ads, &value) == ESP_ERR_TIMEOUT);
    assert(nowUs == 1005000); /* second call did not restart owner budget */
    ads126xAdcEndTransaction(&ads);
    assert(ads126xAdcDeinit(&ads) == ESP_OK);
}

static void testSpiTimingSelection(void)
{
    assert(ads126xAdcSpiDutyCycleForClock(2000000u) == 128u);
    const uint32_t otherClocks[] = {0u, 500000u, 1000000u, 1999999u,
        2000001u, 4000000u, 8000000u, 20000000u};
    for (size_t i = 0u; i < sizeof(otherClocks) / sizeof(otherClocks[0]); ++i) {
        assert(ads126xAdcSpiDutyCycleForClock(otherClocks[i]) == 128u);
    }
}

static void testRegisterReadGuard(void)
{
    ads126xAdcHandle_t ads = handle();
    forceFinalRegisterBit = true;
    uint8_t bytes[18]; memset(bytes, 0xA5, sizeof(bytes));
    assert(ads126xAdcReadRegisters(&ads, ADS126X_REG_POWER, bytes, 1u) == ESP_OK);
    assert(bytes[0] == 2u && bytes[1] == 0xA5u); /* guard isn't returned */
    assert(lastRegisterStart == 1u && lastRegisterCount == 2u);
    registers[ADS126X_REG_REFMUX] = 0u;
    assert(ads126xAdcReadRegisters(&ads, ADS126X_REG_REFMUX, bytes, 1u) == ESP_OK);
    assert(bytes[0] == 0u && bytes[1] == 0xA5u && lastRegisterCount == 2u);
    assert(ads126xAdcReadRegisters(&ads, ADS126X_REG_ID, bytes, 16u) == ESP_OK);
    assert(bytes[15] == 0u && bytes[16] == 0xA5u && lastRegisterCount == 17u);
    ads126xAdc1RegisterSnapshot_t snapshot = {0};
    for (unsigned i = 0u; i < 3u; ++i) { memcpy(snapshots[i], registers, sizeof(registers)); }
    useSnapshots = true;
    assert(ads126xAdcReadAdc1RegisterSnapshot(&ads, &snapshot) == ESP_OK);
    assert(snapshot.power == 2u && snapshot.refmux == 0u && snapshotReads == 2u);

    /* The guard does not mask a real mismatch in any requested byte. */
    useSnapshots = false; corruptReadRegister = ADS126X_REG_MODE2;
    assert(ads126xAdcSetMode2Verified(&ads, 0x0Fu) == ESP_ERR_INVALID_RESPONSE);
    corruptReadRegister = UINT8_MAX;
    useSnapshots = true; snapshotReads = 0u;
    snapshots[0][15] = 0u; snapshots[1][15] = 1u; snapshots[2][15] = 2u;
    assert(ads126xAdcReadAdc1RegisterSnapshot(&ads, &snapshot) == ESP_ERR_INVALID_RESPONSE);
    assert(snapshotReads == 3u && ads.snapshotMismatchRegister == 15u);

    /* No extension into ADC2-only/unsupported address ranges. */
    useSnapshots = false;
    assert(ads126xAdcReadRegisters(&ads, 0x10u, bytes, 1u) == ESP_OK);
    assert(lastRegisterStart == 0x10u && lastRegisterCount == 1u);
    assert(bytes[0] == 1u); /* deliberately injected final bit, no invented repair */
    assert(ads126xAdcReadRegisters(&ads, ADS126X_REG_ADC2CFG, bytes, 2u) == ESP_OK);
    assert(lastRegisterStart == ADS126X_REG_ADC2CFG && lastRegisterCount == 2u);
    assert(ads126xAdcReadRegisters(&ads, 0x0Fu, bytes, 2u) == ESP_OK);
    assert(lastRegisterCount == 2u);
    unsigned queued = queueCalls;
    assert(ads126xAdcReadRegisters(&ads, 0u, bytes, 0u) == ESP_ERR_INVALID_ARG);
    assert(ads126xAdcReadRegisters(&ads, 0u, bytes, 33u) == ESP_ERR_INVALID_ARG);
    assert(queueCalls == queued);
    assert(ads126xAdcDeinit(&ads) == ESP_OK);

    /* Guard bytes still use persistent DMA storage, even on timeout. */
    ads = handle(); completionBlocked = true; bytes[0] = 0xA5u;
    assert(ads126xAdcReadRegisters(&ads, 1u, bytes, 1u) == ESP_ERR_TIMEOUT);
    assert(bytes[0] == 0xA5u && ads.spiQuarantined && pending == &ads.dmaTransaction);
    assert(pending->rx_buffer == ads.spiRxBuf && pending->length == 32u);
    uint8_t prior[64]; memcpy(prior, ads.spiTxBuf, sizeof(prior)); queued = queueCalls;
    assert(ads126xAdcReadRegisters(&ads, 1u, bytes, 1u) == ESP_ERR_INVALID_STATE);
    assert(queueCalls == queued && memcmp(prior, ads.spiTxBuf, sizeof(prior)) == 0);
    completionBlocked = false;
    assert(ads126xAdcDeinit(&ads) == ESP_OK);
}

static uint8_t crcPolynomialDivision(uint32_t value)
{
    uint64_t dividend = (uint64_t)value << 8u;
    for (int bit = 39; bit >= 8; --bit) {
        if (dividend & (UINT64_C(1) << bit)) {
            dividend ^= UINT64_C(0x107) << (bit - 8);
        }
    }
    return (uint8_t)dividend;
}

static void testCrcAndLength(void)
{
    ads126xAdcHandle_t ads = handle();
    const uint32_t words[] = {0u, 1u, UINT32_MAX, 0x12345678u, 0x80000000u};
    for (size_t i = 0u; i < sizeof(words) / sizeof(words[0]); ++i) {
        uint32_t word = words[i];
        uint8_t frame[] = {0x40u, word >> 24u, word >> 16u, word >> 8u, word, crcPolynomialDivision(word)};
        int32_t raw = 42; uint8_t status = 0u;
        assert(ads126xAdcParseAdc1Frame(&ads, frame, 6u, &raw, &status) == ESP_OK);
        assert((uint32_t)raw == word && status == 0x40u);
        for (unsigned bit = 0u; bit < 32u; ++bit) {
            frame[1u + bit / 8u] ^= 1u << (bit % 8u);
            raw = 42;
            assert(ads126xAdcParseAdc1Frame(&ads, frame, 6u, &raw, &status) == ESP_ERR_INVALID_CRC);
            assert(raw == 42); /* bad data never escapes into range/cache */
            frame[1u + bit / 8u] ^= 1u << (bit % 8u);
        }
        assert(ads126xAdcParseAdc1Frame(&ads, frame, 5u, &raw, &status) == ESP_ERR_INVALID_SIZE);
        frame[0] ^= ADS126X_STATUS_RESET_OCCURRED;
        assert(ads126xAdcParseAdc1Frame(&ads, frame, 6u, &raw, &status) == ESP_OK);
        assert((status & ADS126X_STATUS_RESET_OCCURRED) != 0u); /* STATUS isn't in CRC */
    }
    assert(ads126xAdcDeinit(&ads) == ESP_OK);
}

static void testReferenceAndFreshness(void)
{
    ads126xAdcHandle_t ads = handle();
    assert(ads126xAdcSetInternalReference(&ads, true) == ESP_OK);
    uint64_t readyAt = ads.referenceReadyAtUs;
    assert(readyAt == (uint64_t)nowUs + 50000u);
    nowUs += 1000;
    assert(ads126xAdcSetInternalReference(&ads, true) == ESP_OK);
    assert(ads.referenceReadyAtUs == readyAt);
    assert(ads126xAdcStartAdc1(&ads) == ESP_OK && (uint64_t)nowUs >= readyAt);
    int64_t stable = nowUs;
    assert(ads126xAdcStartAdc1(&ads) == ESP_OK && nowUs == stable);
    assert(ads126xAdcSetInternalReference(&ads, false) == ESP_OK);
    assert(ads126xAdcSetInternalReference(&ads, true) == ESP_OK);
    assert(ads126xAdcBeginTransaction(&ads, 1000u) == ESP_OK);
    assert(ads126xAdcStartAdc1(&ads) == ESP_ERR_TIMEOUT);
    ads126xAdcEndTransaction(&ads);

    uint32_t generation = ads.drdyGeneration;
    /* GPIO held low is insufficient, no new generation. */
    assert(ads126xAdcWaitDrdyGenerationUs(&ads, generation, 600u, NULL) == ESP_ERR_TIMEOUT);
    notifyHandle = &ads; notifyEdge = true;
    assert(ads126xAdcWaitDrdyGenerationUs(&ads, generation, 600u, NULL) == ESP_OK);
    generation = ads.drdyGeneration;
    notifyEdge = true; notifyConfigChange = true;
    assert(ads126xAdcWaitDrdyGenerationUs(&ads, generation, 600u, NULL) == ESP_ERR_INVALID_STATE);
    assert(ads126xAdcDeinit(&ads) == ESP_OK);
}

static void setDataFrame(unsigned index, uint32_t raw, uint8_t status, bool corrupt)
{
    assert(index < 4u);
    uint8_t *frame = dataFrames[index];
    frame[0] = status; frame[1] = raw >> 24u; frame[2] = raw >> 16u;
    frame[3] = raw >> 8u; frame[4] = raw;
    frame[5] = crcPolynomialDivision(raw) ^ (corrupt ? 1u : 0u);
    if (dataFrameLimit <= index) { dataFrameLimit = index + 1u; }
}

static void testFreshCrcRetry(void)
{
    ads126xAdcHandle_t ads = handle();
    notifyHandle = &ads; notifyEdgesRemaining = 2u;
    setDataFrame(0u, 0x12345678u, 0x40u, true);
    setDataFrame(1u, 0x23456789u, 0x40u, false);
    uint8_t budget = 1u, status = 0x55u;
    int32_t raw = 42;
    ads126xAdcFreshReadReport_t report;
    assert(ads126xAdcReadAdc1FreshChecked(&ads, 0u, 600u, &budget,
        &raw, &status, &report) == ESP_ERR_INVALID_STATE); /* owner required */
    assert(dataFrameCount == 0u && raw == 42 && status == 0x55u);
    assert(ads126xAdcBeginTransaction(&ads, 100000u) == ESP_OK);
    assert(ads126xAdcReadAdc1FreshChecked(&ads, 0u, 600u, &budget,
        &raw, &status, &report) == ESP_OK);
    assert((uint32_t)raw == 0x23456789u && status == 0x40u && budget == 0u);
    assert(report.firstError == ESP_ERR_INVALID_CRC && report.terminalError == ESP_OK);
    assert(report.crcErrors == 1u && report.crcRetries == 1u);
    assert(ads.adc1CrcErrorCount == 1u && ads.adc1CrcRetryCount == 1u);
    assert(ads.adc1CrcValidEvenCount + ads.adc1CrcValidOddCount == 1u);
    assert(ads.adc1CrcValidEvenCount != 0u); /* bad first CRC was not counted valid */
    assert(memcmp(report.firstCrcFault.bytes, dataFrames[0], 6u) == 0);
    assert(report.firstCrcFault.received != report.firstCrcFault.computed);
    assert(dataFrameCount == 2u && dataFrameGeneration[1] != dataFrameGeneration[0]);
    /* Another adaptive/high-Z read shares the exhausted cell budget. */
    setDataFrame(2u, 0x3456789Au, 0x40u, true);
    notifyEdgesRemaining = 1u; raw = 42; status = 0x55u;
    assert(ads126xAdcReadAdc1FreshChecked(&ads, ads.drdyGeneration, 600u,
        &budget, &raw, &status, &report) == ESP_ERR_INVALID_CRC);
    assert(dataFrameCount == 3u && raw == 42 && status == 0x55u && report.crcRetries == 0u);
    assert(ads.adc1CrcValidEvenCount + ads.adc1CrcValidOddCount == 1u);
    ads126xAdcEndTransaction(&ads);
    assert(ads126xAdcDeinit(&ads) == ESP_OK);

    ads = handle(); notifyHandle = &ads; notifyEdgesRemaining = 2u;
    setDataFrame(0u, 0x12345678u, 0x40u, true);
    setDataFrame(1u, 0x23456789u, 0x40u, true);
    budget = 1u; raw = 42; status = 0x55u;
    assert(ads126xAdcBeginTransaction(&ads, 100000u) == ESP_OK);
    assert(ads126xAdcReadAdc1FreshChecked(&ads, 0u, 600u, &budget,
        &raw, &status, &report) == ESP_ERR_INVALID_CRC);
    assert(report.crcErrors == 2u && report.crcRetries == 1u && dataFrameCount == 2u);
    assert(ads.adc1CrcValidEvenCount == 0u && ads.adc1CrcValidOddCount == 0u);
    assert(memcmp(report.firstCrcFault.bytes, dataFrames[0], 6u) == 0);
    assert(raw == 42 && status == 0x55u && !ads126xAdcIsQuarantined(&ads));
    ads126xAdcEndTransaction(&ads);
    assert(ads126xAdcDeinit(&ads) == ESP_OK);

    /* No new edge after CRC: retain CRC as first cause, expose retry timeout. */
    ads = handle(); notifyHandle = &ads; notifyEdgesRemaining = 1u;
    setDataFrame(0u, 0x12345678u, 0x40u, true);
    budget = 1u; raw = 42; status = 0x55u;
    assert(ads126xAdcBeginTransaction(&ads, 1200u) == ESP_OK);
    int64_t started = nowUs;
    assert(ads126xAdcReadAdc1FreshChecked(&ads, 0u, 100000u, &budget,
        &raw, &status, &report) == ESP_ERR_INVALID_CRC);
    assert(report.firstError == ESP_ERR_INVALID_CRC && report.terminalError == ESP_ERR_TIMEOUT);
    assert(nowUs - started <= 1200u + 1000u && dataFrameCount == 1u); /* one RTOS tick */
    assert(raw == 42 && status == 0x55u);
    ads126xAdcEndTransaction(&ads);
    assert(ads126xAdcDeinit(&ads) == ESP_OK);

    /* A retry SPI completion timeout retains DMA ownership and quarantines. */
    ads = handle(); notifyHandle = &ads; notifyEdgesRemaining = 2u;
    setDataFrame(0u, 0x12345678u, 0x40u, true);
    setDataFrame(1u, 0x23456789u, 0x40u, false);
    blockCompletionAfterRead = 1u; budget = 1u; raw = 42; status = 0x55u;
    assert(ads126xAdcBeginTransaction(&ads, 2500u) == ESP_OK);
    assert(ads126xAdcReadAdc1FreshChecked(&ads, 0u, 600u, &budget,
        &raw, &status, &report) == ESP_ERR_INVALID_CRC);
    assert(report.terminalError == ESP_ERR_TIMEOUT && ads126xAdcIsQuarantined(&ads));
    assert(pending == &ads.dmaTransaction && raw == 42 && status == 0x55u);
    unsigned queued = queueCalls;
    assert(ads126xAdcReadAdc1RawDmaReady(&ads, &raw, &status, NULL) == ESP_ERR_INVALID_STATE);
    assert(queueCalls == queued);
    ads126xAdcEndTransaction(&ads);
    assert(ads126xAdcDeinit(&ads) == ESP_ERR_INVALID_STATE);
    completionBlocked = false;
    assert(ads126xAdcDeinit(&ads) == ESP_OK);

    ads = handle(); notifyHandle = &ads; notifyEdgesRemaining = 2u;
    configChangeOnEdge = 2u;
    setDataFrame(0u, 0x12345678u, 0x40u, true);
    budget = 1u; raw = 42; status = 0x55u;
    assert(ads126xAdcBeginTransaction(&ads, 100000u) == ESP_OK);
    assert(ads126xAdcReadAdc1FreshChecked(&ads, 0u, 600u, &budget,
        &raw, &status, &report) == ESP_ERR_INVALID_CRC);
    assert(report.terminalError == ESP_ERR_INVALID_STATE && dataFrameCount == 1u);
    assert(raw == 42 && status == 0x55u);
    ads126xAdcEndTransaction(&ads);
    assert(ads126xAdcDeinit(&ads) == ESP_OK);

    ads = handle(); notifyHandle = &ads; notifyEdgesRemaining = 1u;
    setDataFrame(0u, 0x12345678u, 0u, false);
    budget = 1u; raw = 42; status = 0x55u;
    assert(ads126xAdcBeginTransaction(&ads, 100000u) == ESP_OK);
    assert(ads126xAdcReadAdc1FreshChecked(&ads, 0u, 600u, &budget,
        &raw, &status, &report) == ESP_ERR_INVALID_RESPONSE);
    assert(budget == 1u && raw == 42 && status == 0x55u); /* no NEW_DATA */
    ads126xAdcEndTransaction(&ads);
    assert(ads126xAdcDeinit(&ads) == ESP_OK);

    ads = handle(); notifyHandle = &ads; notifyEdgesRemaining = 1u;
    ads.drdyGeneration = ads.configuredDrdyGeneration = UINT32_MAX;
    setDataFrame(0u, 0x12345678u, 0x40u, false);
    budget = 1u;
    assert(ads126xAdcBeginTransaction(&ads, 100000u) == ESP_OK);
    assert(ads126xAdcReadAdc1FreshChecked(&ads, UINT32_MAX, 600u, &budget,
        &raw, &status, &report) == ESP_OK);
    assert(report.generationEnd == 0u && (uint32_t)raw == 0x12345678u);
    ads126xAdcEndTransaction(&ads);
    assert(ads126xAdcDeinit(&ads) == ESP_OK);
}

static void testSnapshots(void)
{
    ads126xAdcHandle_t ads = handle();
    for (unsigned i = 0u; i < 3u; ++i) { memcpy(snapshots[i], registers, sizeof(registers)); }
    snapshots[0][1] |= 0xE0u; snapshots[0][2] |= 0xF0u; snapshots[0][15] |= 0xC0u;
    useSnapshots = true;
    ads126xAdc1RegisterSnapshot_t snapshot = {0};
    assert(ads126xAdcReadAdc1RegisterSnapshot(&ads, &snapshot) == ESP_OK);
    assert(snapshot.power == 2u && snapshot.interface == 6u && snapshot.refmux == 0x24u);
    snapshotReads = 0u;
    snapshots[0][3] = 1u; snapshots[1][3] = 2u; snapshots[2][3] = 3u;
    assert(ads126xAdcReadAdc1RegisterSnapshot(&ads, &snapshot) == ESP_ERR_INVALID_RESPONSE);
    assert(ads.snapshotReadFailure == 2u && ads.snapshotMismatchRegister == 3u);
    queueError = ESP_FAIL;
    assert(ads126xAdcReadAdc1RegisterSnapshot(&ads, &snapshot) == ESP_FAIL);
    assert(ads.snapshotReadFailure == 1u);
    queueError = ESP_OK; useSnapshots = false;
    registers[1] |= ADS126X_POWER_RESET;
    assert(ads126xAdcReadAdc1RegisterSnapshot(&ads, &snapshot) == ESP_OK);
    assert(snapshot.power & ADS126X_POWER_RESET);
    assert(ads126xAdcRestoreAdc1RegisterSnapshot(&ads, &snapshot, 5200000u, false) == ESP_ERR_INVALID_STATE);
    assert(ads126xAdcDeinit(&ads) == ESP_OK);
}

int main(void)
{
    testSpiTimingSelection();
    testRegisterReadGuard();
    testSpiDeadlineAndLifetime();
    testCrcAndLength();
    testReferenceAndFreshness();
    testFreshCrcRetry();
    testSnapshots();
    puts("ADS_DRIVER_RELIABILITY_TESTS,passed=1,source=production_driver,fixture=synthetic");
    return 0;
}

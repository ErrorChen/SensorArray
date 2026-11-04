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
    if (notifyEdge && notifyHandle) {
        nowUs += 100u;
        ++notifyHandle->drdyGeneration;
        if (notifyConfigChange) { ++notifyHandle->configurationGeneration; }
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
        const uint8_t *values = registers;
        if (useSnapshots && reg == 0u && len == 16u) {
            values = snapshots[snapshotReads < 3u ? snapshotReads : 2u];
            ++snapshotReads;
        }
        memcpy(rx + 2, values + reg, len);
    } else if ((tx[0] & 0xE0u) == 0x40u) {
        uint8_t reg = tx[0] & 0x1Fu;
        memcpy(registers + reg, tx + 2, tx[1] + 1u);
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
    testSpiDeadlineAndLifetime();
    testCrcAndLength();
    testReferenceAndFreshness();
    testSnapshots();
    puts("ADS_DRIVER_RELIABILITY_TESTS,passed=1,source=production_driver,fixture=synthetic");
    return 0;
}

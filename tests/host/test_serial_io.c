#include <assert.h>
#include <setjmp.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "esp_heap_caps.h"
#include "common.h"
static int64_t nowUs;
static unsigned allocationCalls, allocationsToFail;
static bool psramUnavailable;
static int installCount, clearCount, writeBudget = -1, scenario, readCount;
static bool installed;
static void (*txTask)(void *);
static bool taskEntered;
static jmp_buf taskExit;
static char output[20000];
static size_t outputLength;
static esp_vfs_t registeredVfs;

void *heap_caps_calloc(size_t count, size_t size, uint32_t caps)
{
    allocationCalls++;
    if (allocationsToFail) { allocationsToFail--; return NULL; }
    if (psramUnavailable && (caps & MALLOC_CAP_SPIRAM)) { return NULL; }
    assert(caps == (MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT) ||
           caps == (MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
    return calloc(count, size);
}

QueueHandle_t xQueueCreateStatic(unsigned count, unsigned size, uint8_t *bytes, StaticQueue_t *queue)
{
    assert(size == 1); *queue = (StaticQueue_t){.length=count, .data=bytes}; return queue;
}
int xQueueSend(QueueHandle_t q, const void *item, TickType_t ticks)
{
    assert(ticks == 0); if (q->count == q->length) { return 0; }
    q->data[q->write++ % q->length] = *(const uint8_t *)item; q->count++; return 1;
}
int xQueueReceive(QueueHandle_t q, void *item, TickType_t ticks)
{
    assert(ticks == 0); if (!q->count) { return 0; }
    *(uint8_t *)item = q->data[q->read++ % q->length]; q->count--; return 1;
}
int xTaskCreatePinnedToCore(void (*fn)(void *), const char *name, unsigned stack, void *arg, unsigned prio, TaskHandle_t *task, int core)
{
    txTask = fn; *task = (void *)1; return pdPASS;
}
int xTaskNotifyGive(TaskHandle_t task) { return 1; }
uint32_t ulTaskNotifyTake(int clear, TickType_t ticks)
{
    if (taskEntered) { longjmp(taskExit, 1); }
    taskEntered = true; return 1;
}
void vTaskDelay(TickType_t ticks) { nowUs += ticks * 1000; }
unsigned uxTaskGetStackHighWaterMark(TaskHandle_t task) { return 2048; }
int64_t esp_timer_get_time(void) { return nowUs; }
esp_err_t usb_serial_jtag_driver_install(usb_serial_jtag_driver_config_t *config)
{
    installed = true; installCount++; return ESP_OK;
}
bool usb_serial_jtag_is_driver_installed(void) { return installed; }
bool usb_serial_jtag_is_connected(void) { return scenario != 1; }
int usb_serial_jtag_write_bytes(const void *data, size_t length, TickType_t ticks)
{
    assert(ticks == 0 && length <= 64);
    if (scenario == 2 && nowUs >= 20000) { writeBudget = -1; }
    if (writeBudget == 0) { return 0; }
    if (writeBudget > 0 && length > (size_t)writeBudget) { length = (size_t)writeBudget; }
    memcpy(output + outputLength, data, length); outputLength += length;
    if (writeBudget > 0) { writeBudget -= (int)length; }
    return (int)length;
}
int usb_serial_jtag_read_bytes(void *data, uint32_t length, TickType_t ticks)
{
    assert(ticks > 0); memcpy(data, "MODE?\n", 6); readCount++; return 6;
}
esp_err_t usb_serial_jtag_wait_tx_done(TickType_t ticks) { assert(ticks == 0); return ESP_OK; }
esp_err_t esp_vfs_register(const char *path, const esp_vfs_t *vfs, void *context) { registeredVfs = *vfs; return ESP_OK; }
FILE *test_freopen(const char *path, const char *mode, FILE *stream) { return stream; }
int test_setvbuf(FILE *stream, char *buffer, int mode, size_t size) { return 0; }
void test_clearerr(FILE *stream) { clearCount++; }

#include "../../core/transport/sensorarraySerialIo.c"

static void drain(void)
{
    taskEntered = false;
    if (setjmp(taskExit) == 0) { txTask(NULL); }
}
int main(int argc, char **argv)
{
    const char *allocation = argc > 1 ? argv[1] : "psram";
    psramUnavailable = strcmp(allocation, "internal") == 0;
    if (strcmp(allocation, "failure_retry") == 0) {
        allocationsToFail = 2;
        assert(sensorarraySerialIoInit() == ESP_ERR_NO_MEM);
        assert(installCount == 0 && txTask == NULL && s_dataBytes == NULL);
    }
    assert(sensorarraySerialIoInit() == ESP_OK);
    unsigned allocationsAfterInit = allocationCalls;
    assert(sensorarraySerialIoInit() == ESP_OK && installCount == 1);
    assert(sensorarraySerialIoQueue("DATA\n", 5, false) == ESP_OK);
    assert(sensorarraySerialIoQueue("MAPP\n", 5, true) == ESP_OK);
    drain();
    assert(outputLength == 10 && memcmp(output, "MAPP\nDATA\n", 10) == 0);
    /* DATA floods cannot use reserved priority slots. */
    for (unsigned i = 0; i < 4; ++i) { assert(sensorarraySerialIoQueue("D\n", 2, false) == ESP_OK); }
    assert(sensorarraySerialIoQueue("DROP\n", 5, false) == ESP_ERR_TIMEOUT);
    assert(sensorarraySerialIoQueue("CTRL\n", 5, true) == ESP_OK);
    drain();
    assert(clearCount >= 4);
    /* No host: bounded TX + a still functional independent command RX. */
    scenario = 1; writeBudget = 0; nowUs = 0;
    assert(sensorarraySerialIoQueue("DATA\n", 5, false) == ESP_OK);
    drain(); assert(nowUs == 20000);
    char rx[64]; assert(sensorarraySerialIoRead(rx, sizeof(rx)) == 6 && readCount == 1);
    sensorarraySerialIoStats_t stats;
    sensorarraySerialIoGetStats(&stats);
    assert(stats.normalBufferBytes == 5u * SENSORARRAY_TRANSPORT_TEXT_MAX);
    assert(stats.normalBufferMemory == (psramUnavailable ? SENSORARRAY_SERIAL_MEMORY_INTERNAL : SENSORARRAY_SERIAL_MEMORY_PSRAM));
    assert(stats.timeout == 1 && stats.rxBytes == 6 && !stats.sofPresent);
    /* Real partial driver return: retain prefix count, discard remainder,
     * delimit next packet, then resume once host reads again. */
    scenario = 2; writeBudget = 80; nowUs = 0; outputLength = 0;
    char packet[160]; memset(packet, 'A', sizeof(packet));
    assert(sensorarraySerialIoQueue(packet, sizeof(packet), false) == ESP_OK);
    assert(sensorarraySerialIoQueue("AFTER\n", 6, false) == ESP_OK);
    drain();
    assert(outputLength == 87 && memcmp(output + 80, "\nAFTER\n", 7) == 0);
    sensorarraySerialIoGetStats(&stats);
    assert(stats.partialWrite == 1 && stats.timeout == 2 && stats.driverAcceptedBytes > 80);
    /* The actual registered VFS reports full queue rejection, never fake success. */
    scenario = 0; writeBudget = -1;
    for (unsigned i = 0; i < 4; ++i) { assert(registeredVfs.write(0, "LOG\n", 4) == 4); }
    assert(registeredVfs.write(0, "LOG\n", 4) == -1 && errno == EAGAIN);
    drain(); assert(registeredVfs.write(0, "RECOVERED\n", 10) == 10); drain();
    assert(allocationCalls == allocationsAfterInit); /* no hot path allocations */
    printf("SERIAL_IO_PASS,allocation=%s,initOnce=1,noHotAlloc=1,priority=1,noHostBoundUs=20000,rxIndependent=1,partialRecovery=1,vfsRejectAndResume=1\n", allocation);
    return 0;
}

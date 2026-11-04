#include "sensorarraySerialIo.h"

#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include "driver/usb_serial_jtag.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "esp_vfs.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#ifndef CONFIG_SENSORARRAY_OUTPUT_TASK_CORE
#define CONFIG_SENSORARRAY_OUTPUT_TASK_CORE 0
#endif
#include "sensorarrayTransport.h"

/* One physical TX owner. stdout/log, DATA and priority replies all enter bounded
 * queues; no network or acquisition task calls a USB write/flush. */
#define SERIAL_IO_QUEUE_COUNT 4u
#define SERIAL_IO_DEADLINE_US 20000u
#define SERIAL_IO_CHUNK 64u
/* DATA has five slots (four queued + consumer); priority has four smaller
 * slots. No producer allocates a full payload on its task stack. */
static char (*s_dataBytes)[SENSORARRAY_TRANSPORT_TEXT_MAX];
static uint8_t s_dataMemory; /* 0 unavailable, 1 internal, 2 PSRAM */
static char s_priorityBytes[4][SENSORARRAY_TRANSPORT_CTRL_TEXT_MAX];
static uint16_t s_lengths[9];
static char *sensorarraySerialBytes(unsigned index)
{
    return index < 5u ? s_dataBytes[index] : s_priorityBytes[index - 5u];
}
static StaticQueue_t s_queueStorage[2];
static uint8_t s_queueBytes[2][SERIAL_IO_QUEUE_COUNT * sizeof(uint8_t)];
static bool s_inUse[9];
static portMUX_TYPE s_poolMux = portMUX_INITIALIZER_UNLOCKED;
static QueueHandle_t s_queues[2];
static TaskHandle_t s_task;

static portMUX_TYPE s_statsMux = portMUX_INITIALIZER_UNLOCKED;
static sensorarraySerialIoStats_t s_stats;
static bool s_vfsReady;
static bool s_vfsRegistered;

esp_err_t sensorarraySerialIoQueue(const char *data, size_t length, bool priority)
{
    if (!data || !length || length > (priority ? SENSORARRAY_TRANSPORT_CTRL_TEXT_MAX : SENSORARRAY_TRANSPORT_TEXT_MAX)) {
        return ESP_ERR_INVALID_SIZE;
    }
    if (!s_task || !s_queues[priority ? 1 : 0]) {
        return ESP_ERR_INVALID_STATE;
    }
    uint8_t index = UINT8_MAX;
    portENTER_CRITICAL(&s_poolMux);
    for (unsigned i = priority ? 5u : 0u; i < (priority ? 9u : 5u); ++i) {
        if (!s_inUse[i]) { s_inUse[i] = true; index = (uint8_t)i; break; }
    }
    portEXIT_CRITICAL(&s_poolMux);
    bool queued = false;
    if (index != UINT8_MAX) {
        s_lengths[index] = (uint16_t)length;
        memcpy(sensorarraySerialBytes(index), data, length);
        queued = xQueueSend(s_queues[priority ? 1 : 0], &index, 0) == pdTRUE;
        if (!queued) {
            portENTER_CRITICAL(&s_poolMux);
            s_inUse[index] = false;
            portEXIT_CRITICAL(&s_poolMux);
        }
    }
    portENTER_CRITICAL(&s_statsMux);
    if (queued) {
        s_stats.sinkQueued++;
        s_stats.priorityQueued += priority ? 1u : 0u;
    } else {
        s_stats.sinkDropped++;
    }
    portEXIT_CRITICAL(&s_statsMux);
    if (queued) {
        (void)xTaskNotifyGive(s_task);
    }
    return queued ? ESP_OK : ESP_ERR_TIMEOUT;
}

static int sensorarraySerialOpen(const char *path, int flags, int mode)
{
    (void)path; (void)flags; (void)mode;
    return 0;
}
static int sensorarraySerialClose(int fd) { (void)fd; return 0; }
static int sensorarraySerialFstat(int fd, struct stat *st)
{
    (void)fd;
    memset(st, 0, sizeof(*st));
    st->st_mode = S_IFCHR;
    return 0;
}
static ssize_t sensorarraySerialWrite(int fd, const void *data, size_t size)
{
    (void)fd;
    if (size == 0u) {
        return 0;
    }
    esp_err_t err = sensorarraySerialIoQueue(data, size, false);
    if (err != ESP_OK) {
        errno = err == ESP_ERR_INVALID_SIZE ? EMSGSIZE : EAGAIN;
        return -1;
    }
    return (ssize_t)size;
}

static void sensorarraySerialTxTask(void *arg)
{
    (void)arg;
    bool needsDelimiter = false;
    uint64_t lastDrainBytes = 0;
    for (;;) {
        (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        uint8_t index;
        while (xQueueReceive(s_queues[1], &index, 0) == pdTRUE ||
               xQueueReceive(s_queues[0], &index, 0) == pdTRUE) {
            const char *packetData = sensorarraySerialBytes(index);
            size_t packetLength = s_lengths[index];
            size_t offset = 0;
            int64_t deadline = esp_timer_get_time() + SERIAL_IO_DEADLINE_US;
            uint64_t accepted = 0;
            while (esp_timer_get_time() < deadline) {
                if (needsDelimiter) {
                    if (usb_serial_jtag_write_bytes("\n", 1u, 0) == 1) {
                        accepted++;
                        needsDelimiter = false;
                    }
                } else if (offset < packetLength) {
                    size_t chunk = packetLength - offset;
                    if (chunk > SERIAL_IO_CHUNK) { chunk = SERIAL_IO_CHUNK; }
                    int written = usb_serial_jtag_write_bytes(packetData + offset, chunk, 0);
                    if (written > 0) {
                        offset += (size_t)written;
                        accepted += (uint32_t)written;
                        continue;
                    }
                } else {
                    break;
                }
                vTaskDelay(1);
            }
            int64_t nowUs = esp_timer_get_time();
            portENTER_CRITICAL(&s_statsMux);
            s_stats.driverAcceptedBytes += accepted;
            if (accepted) { s_stats.lastTxUs = nowUs; }
            if (offset != packetLength) {
                s_stats.timeout++;
                s_stats.sinkDropped++;
                s_stats.partialWrite += offset != 0u ? 1u : 0u;
                needsDelimiter |= offset != 0u;
            }
            uint64_t acceptedTotal = s_stats.driverAcceptedBytes;
            portEXIT_CRITICAL(&s_statsMux);
            portENTER_CRITICAL(&s_poolMux);
            s_inUse[index] = false;
            portEXIT_CRITICAL(&s_poolMux);
            if (acceptedTotal != lastDrainBytes && usb_serial_jtag_wait_tx_done(0) == ESP_OK) {
                lastDrainBytes = acceptedTotal;
                portENTER_CRITICAL(&s_statsMux);
                s_stats.driverDrained++;
                portEXIT_CRITICAL(&s_statsMux);
            }
        }
        /* Newlib retains FILE error flags after EAGAIN; a later queue drain
         * permits retries. These only acquire short stdio locks, never USB. */
        clearerr(stdout);
        clearerr(stderr);
    }
}

esp_err_t sensorarraySerialIoInit(void)
{
    if (s_task && s_vfsReady) { return ESP_OK; }
    if (!s_dataBytes) {
        /* CPU-only payload: the driver copies each 64-byte chunk into its own
         * internal ring buffer. Allocate once, outside every producer path. */
        s_dataBytes = heap_caps_calloc(5u, sizeof(*s_dataBytes),
                                      MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (s_dataBytes) {
            s_dataMemory = SENSORARRAY_SERIAL_MEMORY_PSRAM;
        } else {
            s_dataBytes = heap_caps_calloc(5u, sizeof(*s_dataBytes),
                                          MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
            if (s_dataBytes) { s_dataMemory = SENSORARRAY_SERIAL_MEMORY_INTERNAL; }
        }
        if (!s_dataBytes) { return ESP_ERR_NO_MEM; }
    }
    if (!usb_serial_jtag_is_driver_installed()) {
        usb_serial_jtag_driver_config_t config = {.tx_buffer_size = 1024, .rx_buffer_size = 512};
        esp_err_t err = usb_serial_jtag_driver_install(&config);
        if (err != ESP_OK) { return err; }
    }
    if (!s_task) {
        for (unsigned i = 0; i < 2u; ++i) {
            s_queues[i] = xQueueCreateStatic(SERIAL_IO_QUEUE_COUNT, sizeof(uint8_t),
                                            s_queueBytes[i], &s_queueStorage[i]);
            if (!s_queues[i]) { return ESP_ERR_NO_MEM; }
        }
        if (xTaskCreatePinnedToCore(sensorarraySerialTxTask, "serialTx", 4096, NULL, 4, &s_task,
                                   CONFIG_SENSORARRAY_OUTPUT_TASK_CORE) != pdPASS) {
            return ESP_ERR_NO_MEM;
        }
    }
    if (!s_vfsReady) {
        const esp_vfs_t vfs = {.flags = ESP_VFS_FLAG_DEFAULT, .open = sensorarraySerialOpen,
            .write = sensorarraySerialWrite, .close = sensorarraySerialClose, .fstat = sensorarraySerialFstat};
        if (!s_vfsRegistered) {
            esp_err_t err = esp_vfs_register("/dev/sensorarray", &vfs, NULL);
            if (err != ESP_OK) { return err; }
            s_vfsRegistered = true;
        }
        if (!freopen("/dev/sensorarray", "w", stdout) || !freopen("/dev/sensorarray", "w", stderr)) {
            return ESP_FAIL;
        }
        setvbuf(stdout, NULL, _IONBF, 0);
        setvbuf(stderr, NULL, _IONBF, 0);
        s_vfsReady = true;
    }
    return ESP_OK;
}

int sensorarraySerialIoRead(void *buffer, size_t length)
{
    if (!usb_serial_jtag_is_driver_installed()) { return 0; }
    TickType_t ticks = pdMS_TO_TICKS(20u);
    int count = usb_serial_jtag_read_bytes(buffer, length, ticks ? ticks : 1);
    if (count > 0) {
        int64_t nowUs = esp_timer_get_time();
        portENTER_CRITICAL(&s_statsMux);
        s_stats.rxBytes += (uint32_t)count;
        s_stats.lastRxUs = nowUs;
        portEXIT_CRITICAL(&s_statsMux);
    }
    return count;
}

void sensorarraySerialIoNoteOversize(void)
{
    portENTER_CRITICAL(&s_statsMux);
    s_stats.rxOversize++;
    portEXIT_CRITICAL(&s_statsMux);
}

void sensorarraySerialIoGetStats(sensorarraySerialIoStats_t *outStats)
{
    if (!outStats) { return; }
    portENTER_CRITICAL(&s_statsMux);
    *outStats = s_stats;
    portEXIT_CRITICAL(&s_statsMux);
    outStats->normalBufferBytes = s_dataBytes ? 5u * sizeof(*s_dataBytes) : 0u;
    outStats->normalBufferMemory = s_dataMemory;
    outStats->taskStackConfiguredBytes = 4096u;
    outStats->taskStackMinBytes = s_task ? (uint32_t)uxTaskGetStackHighWaterMark(s_task) * sizeof(StackType_t) : 0u;
    outStats->driverInstalled = usb_serial_jtag_is_driver_installed();
    outStats->sofPresent = usb_serial_jtag_is_connected();
}

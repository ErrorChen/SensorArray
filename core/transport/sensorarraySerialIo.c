#include "sensorarraySerialIo.h"

#include <errno.h>
#include <stdatomic.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>
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
#define SERIAL_IO_VFS_PATH "/dev/saout"
#define SERIAL_IO_STDOUT_PATH SERIAL_IO_VFS_PATH "/out"
#define SERIAL_IO_STDERR_PATH SERIAL_IO_VFS_PATH "/err"
#define SERIAL_IO_VFS_DEVICE 0x5341u
_Static_assert(sizeof(SERIAL_IO_VFS_PATH) - 1u <= ESP_VFS_PATH_MAX,
               "Serial output VFS path exceeds ESP-IDF's registration limit");
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
static atomic_bool s_vfsReady;
static bool s_vfsRegistered;
static bool s_stdoutBound;
static bool s_stderrBound;

/* Each stream borrows at most one existing normal slot while a line is
 * incomplete. Two unfinished streams therefore still leave three slots for
 * DATA/complete logs. ESP-IDF shares each FILE across tasks; its FILE lock
 * serializes writes/close within that stream, while stdout/stderr stay distinct.
 * No formatter holds a queue/pool lock across a USB operation. */
typedef struct {
    size_t length;
    uint8_t index;
    bool discarding;
} sensorarraySerialLine_t;
static sensorarraySerialLine_t s_lines[2] = {
    {.index = UINT8_MAX}, {.index = UINT8_MAX},
};

static uint8_t sensorarraySerialClaimSlot(bool priority)
{
    uint8_t index = UINT8_MAX;
    portENTER_CRITICAL(&s_poolMux);
    for (unsigned i = priority ? 5u : 0u; i < (priority ? 9u : 5u); ++i) {
        if (!s_inUse[i]) { s_inUse[i] = true; index = (uint8_t)i; break; }
    }
    portEXIT_CRITICAL(&s_poolMux);
    return index;
}

static void sensorarraySerialReleaseSlot(uint8_t index)
{
    portENTER_CRITICAL(&s_poolMux);
    s_inUse[index] = false;
    portEXIT_CRITICAL(&s_poolMux);
}

static esp_err_t sensorarraySerialPublishSlot(uint8_t index, size_t length, bool priority)
{
    bool queued = false;
    if (index != UINT8_MAX) {
        s_lengths[index] = (uint16_t)length;
        queued = xQueueSend(s_queues[priority ? 1 : 0], &index, 0) == pdTRUE;
        if (!queued) {
            sensorarraySerialReleaseSlot(index);
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

esp_err_t sensorarraySerialIoQueue(const char *data, size_t length, bool priority)
{
    if (!data || !length || length > (priority ? SENSORARRAY_TRANSPORT_CTRL_TEXT_MAX : SENSORARRAY_TRANSPORT_TEXT_MAX)) {
        return ESP_ERR_INVALID_SIZE;
    }
    /* A running driver/task alone is not a single-writer guarantee. Until
     * both shared stdio streams have moved off the native console VFS, keep
     * protocol/DATA TX closed. Registration failure leaves the early console
     * intact; command RX does not depend on this output gate. */
    if (!atomic_load_explicit(&s_vfsReady, memory_order_acquire) ||
        !s_task || !s_queues[priority ? 1 : 0]) {
        return ESP_ERR_INVALID_STATE;
    }
    uint8_t index = sensorarraySerialClaimSlot(priority);
    if (index != UINT8_MAX) { memcpy(sensorarraySerialBytes(index), data, length); }
    return sensorarraySerialPublishSlot(index, length, priority);
}

static void sensorarraySerialLineDrop(size_t bytes, bool oversize, bool sinkCounted)
{
    portENTER_CRITICAL(&s_statsMux);
    s_stats.lineDropped++;
    s_stats.lineOversize += oversize ? 1u : 0u;
    s_stats.discardBytes += bytes;
    s_stats.sinkDropped += sinkCounted ? 0u : 1u;
    portEXIT_CRITICAL(&s_statsMux);
}

static int sensorarraySerialOpen(const char *path, int flags, int mode)
{
    (void)flags; (void)mode;
    if (strcmp(path, "/out") == 0) { return 0; }
    if (strcmp(path, "/err") == 0) { return 1; }
    errno = ENOENT;
    return -1;
}
static int sensorarraySerialClose(int fd)
{
    if (fd < 0 || fd >= 2) { errno = EBADF; return -1; }
    sensorarraySerialLine_t *line = &s_lines[fd];
    if (line->index != UINT8_MAX) {
        sensorarraySerialReleaseSlot(line->index);
        sensorarraySerialLineDrop(line->length, false, false);
    }
    *line = (sensorarraySerialLine_t){.index = UINT8_MAX};
    return 0;
}
static int sensorarraySerialFstat(int fd, struct stat *st)
{
    (void)fd;
    memset(st, 0, sizeof(*st));
    st->st_mode = S_IFCHR;
    st->st_dev = SERIAL_IO_VFS_DEVICE;
    return 0;
}

static bool sensorarraySerialStreamBound(FILE *stream)
{
    struct stat st;
    return stream && fstat(fileno(stream), &st) == 0 &&
        S_ISCHR(st.st_mode) && st.st_dev == SERIAL_IO_VFS_DEVICE;
}

static esp_err_t sensorarraySerialInitResult(esp_err_t err)
{
    portENTER_CRITICAL(&s_statsMux);
    s_stats.initError = err;
    portEXIT_CRITICAL(&s_statsMux);
    return err;
}
static ssize_t sensorarraySerialWrite(int fd, const void *data, size_t size)
{
    if (fd < 0 || fd >= 2) { errno = EBADF; return -1; }
    if (size == 0u) { return 0; }
    if (!data) { errno = EINVAL; return -1; }
    if (!atomic_load_explicit(&s_vfsReady, memory_order_acquire)) {
        errno = EAGAIN;
        return -1;
    }
    sensorarraySerialLine_t *line = &s_lines[fd];
    const char *bytes = data;
    size_t offset = 0u;
    while (offset < size) {
        const char *newline = memchr(bytes + offset, '\n', size - offset);
        size_t part = newline ? (size_t)(newline - bytes - offset) + 1u : size - offset;
        if (line->discarding) {
            portENTER_CRITICAL(&s_statsMux);
            s_stats.discardBytes += part;
            portEXIT_CRITICAL(&s_statsMux);
            line->discarding = newline == NULL;
        } else {
            if (line->index == UINT8_MAX) { line->index = sensorarraySerialClaimSlot(false); }
            if (line->index == UINT8_MAX) {
                sensorarraySerialLineDrop(part, false, false);
                line->discarding = newline == NULL;
            } else if (part > SENSORARRAY_TRANSPORT_TEXT_MAX - line->length) {
                sensorarraySerialReleaseSlot(line->index);
                sensorarraySerialLineDrop(line->length + part, true, false);
                *line = (sensorarraySerialLine_t){.index = UINT8_MAX, .discarding = newline == NULL};
            } else {
                memcpy(sensorarraySerialBytes(line->index) + line->length, bytes + offset, part);
                line->length += part;
                if (newline) {
                    uint8_t index = line->index;
                    size_t length = line->length;
                    *line = (sensorarraySerialLine_t){.index = UINT8_MAX};
                    if (sensorarraySerialPublishSlot(index, length, false) != ESP_OK) {
                        sensorarraySerialLineDrop(length, false, true);
                    }
                }
            }
        }
        offset += part;
    }
    /* This acknowledges bounded local consumption, including counted whole
     * line discard. It is not queue admission or USB delivery. Returning an
     * error after consuming a fragment lets stdio retry a suffix as a new line.
     * Direct protocol/DATA callers still receive actual queue admission errors. */
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
            } else {
                /* Defence for a direct caller that omitted LF. VFS fragments
                 * never arrive here; they are assembled into whole lines. */
                needsDelimiter = packetData[packetLength - 1u] != '\n';
            }
            uint64_t acceptedTotal = s_stats.driverAcceptedBytes;
            portEXIT_CRITICAL(&s_statsMux);
            sensorarraySerialReleaseSlot(index);
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
    if (atomic_load_explicit(&s_vfsReady, memory_order_acquire)) { return ESP_OK; }
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
        if (!s_dataBytes) { return sensorarraySerialInitResult(ESP_ERR_NO_MEM); }
    }
    if (!usb_serial_jtag_is_driver_installed()) {
        usb_serial_jtag_driver_config_t config = {.tx_buffer_size = 1024, .rx_buffer_size = 512};
        esp_err_t err = usb_serial_jtag_driver_install(&config);
        if (err != ESP_OK) { return sensorarraySerialInitResult(err); }
    }
    if (!s_task) {
        for (unsigned i = 0; i < 2u; ++i) {
            s_queues[i] = xQueueCreateStatic(SERIAL_IO_QUEUE_COUNT, sizeof(uint8_t),
                                            s_queueBytes[i], &s_queueStorage[i]);
            if (!s_queues[i]) { return sensorarraySerialInitResult(ESP_ERR_NO_MEM); }
        }
        if (xTaskCreatePinnedToCore(sensorarraySerialTxTask, "serialTx", 4096, NULL, 4, &s_task,
                                   CONFIG_SENSORARRAY_OUTPUT_TASK_CORE) != pdPASS) {
            return sensorarraySerialInitResult(ESP_ERR_NO_MEM);
        }
    }
    if (!atomic_load_explicit(&s_vfsReady, memory_order_acquire)) {
        const esp_vfs_t vfs = {.flags = ESP_VFS_FLAG_DEFAULT, .open = sensorarraySerialOpen,
            .write = sensorarraySerialWrite, .close = sensorarraySerialClose, .fstat = sensorarraySerialFstat};
        if (!s_vfsRegistered) {
            esp_err_t err = esp_vfs_register(SERIAL_IO_VFS_PATH, &vfs, NULL);
            if (err != ESP_OK) { return sensorarraySerialInitResult(err); }
            s_vfsRegistered = true;
        }
        /* ESP-IDF's esp_reent_init gives every task the same global FILE
         * objects. freopen changes those objects in place; do not replace only
         * this task's stdout pointer. Verify the resulting VFS fd, too. */
        if (!s_stdoutBound) {
            if (!freopen(SERIAL_IO_STDOUT_PATH, "w", stdout) ||
                !sensorarraySerialStreamBound(stdout)) {
                return sensorarraySerialInitResult(ESP_FAIL);
            }
            setvbuf(stdout, NULL, _IONBF, 0);
            s_stdoutBound = true;
        }
        if (!s_stderrBound) {
            if (!freopen(SERIAL_IO_STDERR_PATH, "w", stderr) ||
                !sensorarraySerialStreamBound(stderr)) {
                return sensorarraySerialInitResult(ESP_FAIL);
            }
            setvbuf(stderr, NULL, _IONBF, 0);
            s_stderrBound = true;
        }
        /* A stream bound on a previous attempt may have observed EAGAIN while
         * the other stream was unavailable. Clear that local FILE error now. */
        clearerr(stdout);
        clearerr(stderr);
        atomic_store_explicit(&s_vfsReady, true, memory_order_release);
    }
    return sensorarraySerialInitResult(ESP_OK);
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
    outStats->stdioBound = atomic_load_explicit(&s_vfsReady, memory_order_acquire);
}

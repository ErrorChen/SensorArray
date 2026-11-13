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
#include "sensorarrayWorkBudget.h"

/* One physical TX owner. stdout/log, DATA and priority replies all enter bounded
 * queues; no network or acquisition task calls a USB write/flush. */
#define SERIAL_IO_QUEUE_COUNT 4u
/* IDF 5.5.5 BYTEBUF has xMaxItemSize == xSize; write_bytes copies the entire
 * item or returns zero. Give it room for the largest complete protocol packet,
 * instead of releasing a 64-byte prefix when a later chunk times out. */
#define SERIAL_IO_DRIVER_TX_BYTES SENSORARRAY_TRANSPORT_TEXT_MAX
#define SERIAL_IO_NORMAL_LIFETIME_US 250000u
#define SERIAL_IO_PRIORITY_LIFETIME_US 1000000u
#define SERIAL_IO_TICK_US ((uint64_t)portTICK_PERIOD_MS * 1000u)
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
static int64_t s_enqueuedUs[9];
static int64_t s_deadlineUs[9];
static bool s_driverOwned;
_Static_assert(SERIAL_IO_DRIVER_TX_BYTES >= SENSORARRAY_TRANSPORT_TEXT_MAX,
               "USB byte ring must admit one complete DATA record");
_Static_assert(SERIAL_IO_DRIVER_TX_BYTES >= SENSORARRAY_TRANSPORT_CTRL_TEXT_MAX,
               "USB byte ring must admit one complete priority record");
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
        s_enqueuedUs[index] = esp_timer_get_time();
        s_deadlineUs[index] = s_enqueuedUs[index] +
            (priority ? SERIAL_IO_PRIORITY_LIFETIME_US : SERIAL_IO_NORMAL_LIFETIME_US);
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
        s_stats.priorityRejected += priority ? 1u : 0u;
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

static void sensorarraySerialTxCheckpoint(void)
{
    /* Shared CPU0 idle evidence, at most one tick. A real driver wait is not
     * misclassified as CPU work merely because wall time elapsed. */
    if (sensorarrayWorkBudgetCheckpoint(SENSORARRAY_WORK_BUDGET_SERIAL_TX)) {
        portENTER_CRITICAL(&s_statsMux);
        s_stats.txYield++;
        portEXIT_CRITICAL(&s_statsMux);
    }
}

static void sensorarraySerialTxTask(void *arg)
{
    (void)arg;
    bool needsDelimiter = false;
    uint64_t lastDrainBytes = 0;
    /* Slots remain owned until the complete record is copied or its bounded
     * lifetime expires. An unstarted normal record can wait while priority
     * runs; an unexpected positive short write must finish before any other
     * record. The driver copies synchronously, so it retains no slot pointer. */
    uint8_t pending[2] = {UINT8_MAX, UINT8_MAX};
    size_t offset[2] = {0u, 0u};
    for (;;) {
        (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        for (;;) {
            int64_t nowUs = esp_timer_get_time();
            bool drainProgress = false;
            for (unsigned lane = 0; lane < 2u; ++lane) {
                if (pending[lane] == UINT8_MAX &&
                    xQueueReceive(s_queues[lane], &pending[lane], 0) == pdTRUE) {
                    offset[lane] = 0u;
                    drainProgress = true;
                }
                if (pending[lane] != UINT8_MAX && nowUs >= s_deadlineUs[pending[lane]]) {
                    portENTER_CRITICAL(&s_statsMux);
                    s_stats.timeout++;
                    s_stats.sinkDropped++;
                    s_stats.partialWrite += offset[lane] != 0u ? 1u : 0u;
                    if (lane) { s_stats.priorityExpired++; }
                    else { s_stats.normalExpired++; }
                    portEXIT_CRITICAL(&s_statsMux);
                    needsDelimiter |= offset[lane] != 0u;
                    sensorarraySerialReleaseSlot(pending[lane]);
                    pending[lane] = UINT8_MAX;
                    offset[lane] = 0u;
                    drainProgress = true;
                }
            }
            if (pending[0] == UINT8_MAX && pending[1] == UINT8_MAX) {
                if (drainProgress) {
                    sensorarraySerialTxCheckpoint();
                    /* The initial pdTRUE take consumed all wake counts. A
                     * retained record expiring does not prove its queue is
                     * empty; inspect it again without requiring a producer. */
                    continue;
                }
                break;
            }
            unsigned lane = offset[0] != 0u ? 0u : (pending[1] != UINT8_MAX ? 1u : 0u);
            uint8_t index = pending[lane];
            const char *packetData = sensorarraySerialBytes(index);
            size_t packetLength = s_lengths[index];
            nowUs = esp_timer_get_time();
            if (nowUs >= s_deadlineUs[index]) { continue; }
            uint64_t remainingUs = (uint64_t)(s_deadlineUs[index] - nowUs);
            /* write_bytes can wait once for tx_mux and once for ring space.
             * Budget both waits from the original enqueue deadline. */
            TickType_t waitTicks = remainingUs >= 2u * SERIAL_IO_TICK_US ? 1u : 0u;
            size_t requested = needsDelimiter ? 1u : packetLength - offset[lane];
            const char *source = needsDelimiter ? "\n" : packetData + offset[lane];
            int written = usb_serial_jtag_write_bytes(source, requested, waitTicks);
            int64_t completedUs = esp_timer_get_time();
            if (written < 0 || (size_t)written > requested) {
                /* An invalid driver return gives no evidence of completion.
                 * Keep the next record behind a delimiter, never count success. */
                portENTER_CRITICAL(&s_statsMux);
                s_stats.driverWriteError++;
                s_stats.sinkDropped++;
                s_stats.partialWrite += offset[lane] != 0u ? 1u : 0u;
                portEXIT_CRITICAL(&s_statsMux);
                needsDelimiter = true;
                sensorarraySerialReleaseSlot(index);
                pending[lane] = UINT8_MAX;
                offset[lane] = 0u;
                sensorarraySerialTxCheckpoint();
                continue;
            }
            if (written == 0) {
                /* Bounded driver wait blocks only this owner. If a broken
                 * driver returns zero immediately, the shared unit budget
                 * still forces a real RTOS block; never spin indefinitely. */
                sensorarraySerialTxCheckpoint();
                continue;
            }
            uint64_t ageUs = (uint64_t)(completedUs - s_enqueuedUs[index]);
            portENTER_CRITICAL(&s_statsMux);
            s_stats.driverAcceptedBytes += (uint32_t)written;
            s_stats.lastTxUs = completedUs;
            s_stats.driverShortWrite += (size_t)written < requested ? 1u : 0u;
            if (ageUs > s_stats.maxQueueAgeUs) {
                s_stats.maxQueueAgeUs = ageUs > UINT32_MAX ? UINT32_MAX : (uint32_t)ageUs;
            }
            uint64_t acceptedTotal = s_stats.driverAcceptedBytes;
            portEXIT_CRITICAL(&s_statsMux);
            if (needsDelimiter) {
                needsDelimiter = false;
            } else {
                offset[lane] += (size_t)written;
                if (offset[lane] == packetLength) {
                    needsDelimiter = packetData[packetLength - 1u] != '\n';
                    portENTER_CRITICAL(&s_statsMux);
                    if (lane) { s_stats.prioritySubmitted++; }
                    else { s_stats.normalSubmitted++; }
                    portEXIT_CRITICAL(&s_statsMux);
                    sensorarraySerialReleaseSlot(index);
                    pending[lane] = UINT8_MAX;
                    offset[lane] = 0u;
                }
            }
            if (acceptedTotal != lastDrainBytes && usb_serial_jtag_wait_tx_done(0) == ESP_OK) {
                lastDrainBytes = acceptedTotal;
                portENTER_CRITICAL(&s_statsMux);
                s_stats.driverDrained++;
                portEXIT_CRITICAL(&s_statsMux);
            }
            sensorarraySerialTxCheckpoint();
        }
        /* Init clears the only transient EAGAIN (stdio not yet bound). Once
         * ready, VFS consumes complete/discarded lines without setting ferror.
         * The TX owner must not wait for a producer's FILE formatting lock. */
    }
}

esp_err_t sensorarraySerialIoInit(void)
{
    if (atomic_load_explicit(&s_vfsReady, memory_order_acquire)) { return ESP_OK; }
    if (!s_dataBytes) {
        /* CPU-only payload: the driver copies each complete record into its own
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
        usb_serial_jtag_driver_config_t config = {.tx_buffer_size = SERIAL_IO_DRIVER_TX_BYTES, .rx_buffer_size = 512};
        esp_err_t err = usb_serial_jtag_driver_install(&config);
        if (err != ESP_OK) { return sensorarraySerialInitResult(err); }
        s_driverOwned = true;
    } else if (!s_driverOwned) {
        /* The public API cannot reveal an existing driver's ring capacity.
         * Do not remove somebody else's live driver or claim atomic admission. */
        return sensorarraySerialInitResult(ESP_ERR_INVALID_STATE);
    }
    if (!s_task) {
        /* Hook errors remain visible through WorkBudget diagnostics; its
         * bounded fallback preserves this independent control/TX service. */
        (void)sensorarrayWorkBudgetInit();
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
    outStats->driverTxBufferBytes = s_driverOwned ? SERIAL_IO_DRIVER_TX_BYTES : 0u;
    outStats->normalBufferMemory = s_dataMemory;
    outStats->taskStackConfiguredBytes = 4096u;
    outStats->taskStackMinBytes = s_task ? (uint32_t)uxTaskGetStackHighWaterMark(s_task) * sizeof(StackType_t) : 0u;
    outStats->driverInstalled = usb_serial_jtag_is_driver_installed();
    outStats->sofPresent = usb_serial_jtag_is_connected();
    outStats->stdioBound = atomic_load_explicit(&s_vfsReady, memory_order_acquire);
}

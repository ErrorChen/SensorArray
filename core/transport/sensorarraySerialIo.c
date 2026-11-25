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
#include "sensorarrayUsbOtg.h"

/* One physical TX owner. stdout/log, DATA and priority replies all enter bounded
 * queues; no network or acquisition task calls a USB write/flush. */
#define SERIAL_IO_QUEUE_COUNT 4u
#define SERIAL_IO_LOG_QUEUE_COUNT 4u
#define SERIAL_IO_DATA_SLOT_COUNT 5u
#define SERIAL_IO_CDC_DATA_QUEUE_COUNT 100u
#define SERIAL_IO_CDC_DATA_SLOT_COUNT (SERIAL_IO_CDC_DATA_QUEUE_COUNT + 1u)
#define SERIAL_IO_PRIORITY_SLOT_COUNT 4u
#define SERIAL_IO_LOG_SLOT_COUNT 5u
#define SERIAL_IO_PRIORITY_FIRST_SLOT SERIAL_IO_CDC_DATA_SLOT_COUNT
#define SERIAL_IO_LOG_FIRST_SLOT (SERIAL_IO_PRIORITY_FIRST_SLOT + SERIAL_IO_PRIORITY_SLOT_COUNT)
#define SERIAL_IO_SLOT_COUNT (SERIAL_IO_LOG_FIRST_SLOT + SERIAL_IO_LOG_SLOT_COUNT)
#define SERIAL_IO_NORMAL_SLOT_COUNT (SERIAL_IO_DATA_SLOT_COUNT + SERIAL_IO_LOG_SLOT_COUNT)
#define SERIAL_IO_CDC_NORMAL_SLOT_COUNT (SERIAL_IO_CDC_DATA_SLOT_COUNT + SERIAL_IO_LOG_SLOT_COUNT)
#define SERIAL_IO_DATA_BURST_MAX 4u
/* Keep legacy USJ debug below the formal DATA path. The OTG owner shares
 * priority 7 with its producer and BLE; control remains at 8. Driver waits
 * block only this owner, and WorkBudget still bounds ready bursts. */
#define SERIAL_IO_TX_TASK_PRIORITY 6u
#define SERIAL_IO_OTG_TX_TASK_PRIORITY 7u
enum { SERIAL_IO_DATA = 0, SERIAL_IO_PRIORITY, SERIAL_IO_LOG, SERIAL_IO_LANE_COUNT };
/* IDF 5.5.5 BYTEBUF has xMaxItemSize == xSize; write_bytes copies the entire
 * item or returns zero. Give it room for the largest complete protocol packet,
 * instead of releasing a 64-byte prefix when a later chunk times out. */
#define SERIAL_IO_DRIVER_TX_BYTES SENSORARRAY_TRANSPORT_TEXT_MAX
#define SERIAL_IO_NORMAL_LIFETIME_US 250000u
#define SERIAL_IO_CDC_DATA_LIFETIME_US 1000000u
#define SERIAL_IO_PRIORITY_LIFETIME_US 1000000u
#define SERIAL_IO_TICK_US ((uint64_t)portTICK_PERIOD_MS * 1000u)
typedef struct {
    int64_t enqueuedUs, deadlineUs;
    uint32_t epoch;
    uint16_t length;
} sensorarraySerialSlot_t;
#define SERIAL_IO_VFS_PATH "/dev/saout"
#define SERIAL_IO_STDOUT_PATH SERIAL_IO_VFS_PATH "/out"
#define SERIAL_IO_STDERR_PATH SERIAL_IO_VFS_PATH "/err"
#define SERIAL_IO_VFS_DEVICE 0x5341u
_Static_assert(sizeof(SERIAL_IO_VFS_PATH) - 1u <= ESP_VFS_PATH_MAX,
               "Serial output VFS path exceeds ESP-IDF's registration limit");
/* LOG and fallback/legacy DATA retain five slots/four queue entries
 * independently. In particular, the four leading fault records (root cause,
 * rail, context and registers) keep their previous empty-queue admission.
 * Stdio assembles only in LOG slots, so an unfinished printf or diagnostic
 * burst cannot consume DATA capacity. Priority retains four smaller slots. */
static char (*s_dataBytes)[SENSORARRAY_TRANSPORT_TEXT_MAX];
/* The observed formal-USB delivery pause was 622 ms. One hundred queued
 * records cover 1 s at the existing roughly 100 FPS maximum, plus the owner
 * slot. Only CDC with a successful PSRAM allocation uses this bounded burst
 * capacity; legacy/vendor and internal-memory fallback retain the small pool.
 * FIFO/endpoint DMA storage remains in TinyUSB's internal static buffers. */
static unsigned s_dataSlotCount = SERIAL_IO_DATA_SLOT_COUNT;
static unsigned s_dataQueueCount = SERIAL_IO_QUEUE_COUNT;
static uint32_t s_dataLifetimeUs = SERIAL_IO_NORMAL_LIFETIME_US;
static uint8_t s_dataMemory; /* 0 unavailable, 1 internal, 2 PSRAM */
static char s_priorityBytes[SERIAL_IO_PRIORITY_SLOT_COUNT][SENSORARRAY_TRANSPORT_CTRL_TEXT_MAX];
/* Only the expanded DATA metadata follows its CPU payload into PSRAM. The
 * small DATA fallback and all CTRL/LOG metadata stay internal. No ISR, DMA,
 * RTOS object, or lock uses these payload/metadata allocations. */
static sensorarraySerialSlot_t s_smallSlots[SERIAL_IO_DATA_SLOT_COUNT + SERIAL_IO_PRIORITY_SLOT_COUNT + SERIAL_IO_LOG_SLOT_COUNT];
static sensorarraySerialSlot_t *s_dataSlots = s_smallSlots;
static atomic_int s_otgInitResult = ATOMIC_VAR_INIT(ESP_ERR_INVALID_STATE);
static atomic_bool s_otgInitDone;
static bool s_driverOwned;
_Static_assert(SERIAL_IO_DRIVER_TX_BYTES >= SENSORARRAY_TRANSPORT_TEXT_MAX,
               "USB byte ring must admit one complete DATA record");
_Static_assert(SERIAL_IO_DRIVER_TX_BYTES >= SENSORARRAY_TRANSPORT_CTRL_TEXT_MAX,
               "USB byte ring must admit one complete priority record");
_Static_assert(SERIAL_IO_SLOT_COUNT < UINT8_MAX, "Slot indices must fit the queue descriptor");
static char *sensorarraySerialBytes(unsigned index)
{
    if (index < SERIAL_IO_PRIORITY_FIRST_SLOT) { return s_dataBytes[index]; }
    if (index < SERIAL_IO_LOG_FIRST_SLOT) { return s_priorityBytes[index - SERIAL_IO_PRIORITY_FIRST_SLOT]; }
    return s_dataBytes[s_dataSlotCount + index - SERIAL_IO_LOG_FIRST_SLOT];
}
static sensorarraySerialSlot_t *sensorarraySerialSlot(unsigned index)
{
    if (index < SERIAL_IO_PRIORITY_FIRST_SLOT) { return &s_dataSlots[index]; }
    return &s_smallSlots[SERIAL_IO_DATA_SLOT_COUNT + index - SERIAL_IO_PRIORITY_FIRST_SLOT];
}
static StaticQueue_t s_queueStorage[SERIAL_IO_LANE_COUNT];
static uint8_t s_dataQueueBytes[SERIAL_IO_CDC_DATA_QUEUE_COUNT * sizeof(uint8_t)];
static uint8_t s_queueBytes[SERIAL_IO_LANE_COUNT][SERIAL_IO_QUEUE_COUNT * sizeof(uint8_t)];
/* Ownership bits are internal: claiming/releasing under the pool spinlock
 * never touches PSRAM. Metadata is filled after exclusive ownership transfers. */
static bool s_inUse[SERIAL_IO_SLOT_COUNT];
static portMUX_TYPE s_poolMux = portMUX_INITIALIZER_UNLOCKED;
static QueueHandle_t s_queues[SERIAL_IO_LANE_COUNT];
static TaskHandle_t s_task;

static portMUX_TYPE s_statsMux = portMUX_INITIALIZER_UNLOCKED;
static sensorarraySerialIoStats_t s_stats;
static atomic_bool s_vfsReady;
static bool s_vfsRegistered;
static bool s_stdoutBound;
static bool s_stderrBound;

/* Each stream borrows at most one LOG slot while a line is incomplete.
 * Two unfinished streams leave all DATA and priority slots available.
 * ESP-IDF shares each FILE across tasks; its FILE lock
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

static uint8_t sensorarraySerialClaimSlot(unsigned lane)
{
    uint8_t index = UINT8_MAX;
    unsigned first = lane == SERIAL_IO_DATA ? 0u :
        (lane == SERIAL_IO_PRIORITY ? SERIAL_IO_PRIORITY_FIRST_SLOT : SERIAL_IO_LOG_FIRST_SLOT);
    unsigned end = lane == SERIAL_IO_DATA ? s_dataSlotCount :
        (lane == SERIAL_IO_PRIORITY ? SERIAL_IO_LOG_FIRST_SLOT : SERIAL_IO_SLOT_COUNT);
    portENTER_CRITICAL(&s_poolMux);
    for (unsigned i = first; i < end; ++i) {
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

static esp_err_t sensorarraySerialPublishSlot(uint8_t index, size_t length, unsigned lane)
{
    bool priority = lane == SERIAL_IO_PRIORITY;
    bool queued = false;
    if (index != UINT8_MAX) {
        sensorarraySerialSlot(index)->length = (uint16_t)length;
        sensorarraySerialSlot(index)->epoch = sensorarrayUsbOtgEpoch();
        sensorarraySerialSlot(index)->enqueuedUs = esp_timer_get_time();
        sensorarraySerialSlot(index)->deadlineUs = sensorarraySerialSlot(index)->enqueuedUs +
            (priority ? SERIAL_IO_PRIORITY_LIFETIME_US :
             lane == SERIAL_IO_DATA ? s_dataLifetimeUs : SERIAL_IO_NORMAL_LIFETIME_US);
        queued = xQueueSend(s_queues[lane], &index, 0) == pdTRUE;
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
        s_stats.dataRejected += lane == SERIAL_IO_DATA ? 1u : 0u;
        s_stats.logRejected += lane == SERIAL_IO_LOG ? 1u : 0u;
    }
    portEXIT_CRITICAL(&s_statsMux);
    if (queued) {
        (void)xTaskNotifyGive(s_task);
    }
    return queued ? ESP_OK : ESP_ERR_TIMEOUT;
}

static esp_err_t sensorarraySerialQueueRecord(const char *data, size_t length, unsigned lane)
{
    bool priority = lane == SERIAL_IO_PRIORITY;
    if (!data || !length || length > (priority ? SENSORARRAY_TRANSPORT_CTRL_TEXT_MAX : SENSORARRAY_TRANSPORT_TEXT_MAX)) {
        return ESP_ERR_INVALID_SIZE;
    }
    /* A running driver/task alone is not a single-writer guarantee. Until
     * both shared stdio streams have moved off the native console VFS, keep
     * protocol/DATA TX closed. Registration failure leaves the early console
     * intact; command RX does not depend on this output gate. */
    if (!atomic_load_explicit(&s_vfsReady, memory_order_acquire) ||
        !s_task || !s_queues[lane]) {
        return ESP_ERR_INVALID_STATE;
    }
    uint8_t index = sensorarraySerialClaimSlot(lane);
    if (index != UINT8_MAX) { memcpy(sensorarraySerialBytes(index), data, length); }
    return sensorarraySerialPublishSlot(index, length, lane);
}

esp_err_t sensorarraySerialIoQueue(const char *data, size_t length, bool priority)
{
    return sensorarraySerialQueueRecord(data, length, priority ? SERIAL_IO_PRIORITY : SERIAL_IO_LOG);
}

esp_err_t sensorarraySerialIoQueueData(const char *data, size_t length)
{
    return sensorarraySerialQueueRecord(data, length, SERIAL_IO_DATA);
}

static void sensorarraySerialLineDrop(size_t bytes, bool oversize, bool sinkCounted, bool capacityRejected)
{
    portENTER_CRITICAL(&s_statsMux);
    s_stats.lineDropped++;
    s_stats.lineOversize += oversize ? 1u : 0u;
    s_stats.discardBytes += bytes;
    s_stats.sinkDropped += sinkCounted ? 0u : 1u;
    s_stats.logRejected += capacityRejected && !sinkCounted ? 1u : 0u;
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
        sensorarraySerialLineDrop(line->length, false, false, false);
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
            if (line->index == UINT8_MAX) { line->index = sensorarraySerialClaimSlot(SERIAL_IO_LOG); }
            if (line->index == UINT8_MAX) {
                sensorarraySerialLineDrop(part, false, false, true);
                line->discarding = newline == NULL;
            } else if (part > SENSORARRAY_TRANSPORT_TEXT_MAX - line->length) {
                sensorarraySerialReleaseSlot(line->index);
                sensorarraySerialLineDrop(line->length + part, true, false, false);
                *line = (sensorarraySerialLine_t){.index = UINT8_MAX, .discarding = newline == NULL};
            } else {
                memcpy(sensorarraySerialBytes(line->index) + line->length, bytes + offset, part);
                line->length += part;
                if (newline) {
                    uint8_t index = line->index;
                    size_t length = line->length;
                    *line = (sensorarraySerialLine_t){.index = UINT8_MAX};
                    if (sensorarraySerialPublishSlot(index, length, SERIAL_IO_LOG) != ESP_OK) {
                        sensorarraySerialLineDrop(length, false, true, true);
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

static void sensorarraySerialLegacyTxTask(void *arg)
{
    (void)arg;
    bool needsDelimiter = false;
    uint64_t lastDrainBytes = 0;
    /* Slots remain owned until the complete record is copied or its bounded
     * lifetime expires. An unstarted normal record can wait while priority
     * runs; an unexpected positive short write must finish before any other
     * record. The driver copies synchronously, so it retains no slot pointer. */
    uint8_t pending[SERIAL_IO_LANE_COUNT] = {UINT8_MAX, UINT8_MAX, UINT8_MAX};
    size_t offset[SERIAL_IO_LANE_COUNT] = {0u, 0u, 0u};
    unsigned dataBurst = 0u;
    for (;;) {
        (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        for (;;) {
            int64_t nowUs = esp_timer_get_time();
            bool drainProgress = false;
            for (unsigned lane = 0; lane < SERIAL_IO_LANE_COUNT; ++lane) {
                if (pending[lane] == UINT8_MAX &&
                    xQueueReceive(s_queues[lane], &pending[lane], 0) == pdTRUE) {
                    offset[lane] = 0u;
                    drainProgress = true;
                }
                if (pending[lane] != UINT8_MAX && nowUs >= sensorarraySerialSlot(pending[lane])->deadlineUs) {
                    portENTER_CRITICAL(&s_statsMux);
                    s_stats.timeout++;
                    s_stats.sinkDropped++;
                    s_stats.partialWrite += offset[lane] != 0u ? 1u : 0u;
                    if (lane == SERIAL_IO_PRIORITY) { s_stats.priorityExpired++; }
                    else { s_stats.normalExpired++; }
                    portEXIT_CRITICAL(&s_statsMux);
                    needsDelimiter |= offset[lane] != 0u;
                    sensorarraySerialReleaseSlot(pending[lane]);
                    pending[lane] = UINT8_MAX;
                    offset[lane] = 0u;
                    drainProgress = true;
                }
            }
            if (pending[SERIAL_IO_DATA] == UINT8_MAX && pending[SERIAL_IO_PRIORITY] == UINT8_MAX &&
                pending[SERIAL_IO_LOG] == UINT8_MAX) {
                if (drainProgress) {
                    sensorarraySerialTxCheckpoint();
                    /* The initial pdTRUE take consumed all wake counts. A
                     * retained record expiring does not prove its queue is
                     * empty; inspect it again without requiring a producer. */
                    continue;
                }
                break;
            }
            /* A positive prefix in ANY lane owns the wire until completion
             * or expiry. At whole-record boundaries control wins; DATA gets
             * priority over LOG, with one waiting LOG served after at most
             * four completed DATA records. No DATA/log record is relabelled. */
            unsigned lane = SERIAL_IO_LANE_COUNT;
            for (unsigned i = 0; i < SERIAL_IO_LANE_COUNT; ++i) {
                if (offset[i] != 0u) { lane = i; break; }
            }
            if (lane == SERIAL_IO_LANE_COUNT) {
                if (pending[SERIAL_IO_PRIORITY] != UINT8_MAX) { lane = SERIAL_IO_PRIORITY; }
                else if (pending[SERIAL_IO_DATA] != UINT8_MAX &&
                         (pending[SERIAL_IO_LOG] == UINT8_MAX || dataBurst < SERIAL_IO_DATA_BURST_MAX)) {
                    lane = SERIAL_IO_DATA;
                } else { lane = SERIAL_IO_LOG; }
            }
            uint8_t index = pending[lane];
            const char *packetData = sensorarraySerialBytes(index);
            size_t packetLength = sensorarraySerialSlot(index)->length;
            nowUs = esp_timer_get_time();
            if (nowUs >= sensorarraySerialSlot(index)->deadlineUs) { continue; }
            uint64_t remainingUs = (uint64_t)(sensorarraySerialSlot(index)->deadlineUs - nowUs);
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
            uint64_t ageUs = (uint64_t)(completedUs - sensorarraySerialSlot(index)->enqueuedUs);
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
                    if (lane == SERIAL_IO_PRIORITY) { s_stats.prioritySubmitted++; }
                    else {
                        s_stats.normalSubmitted++;
                        if (lane == SERIAL_IO_DATA) { s_stats.dataSubmitted++; }
                        else { s_stats.logSubmitted++; }
                    }
                    portEXIT_CRITICAL(&s_statsMux);
                    if (lane == SERIAL_IO_DATA && dataBurst < SERIAL_IO_DATA_BURST_MAX) { dataBurst++; }
                    else if (lane == SERIAL_IO_LOG) { dataBurst = 0u; }
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

/* 本轮新增: persistent slots feed either independent vendor endpoints or a
 * shared formal CDC stream. A started formal record owns that stream until
 * completion/expiry; debug has its own endpoint and never retains formal I/O.
 * TinyUSB copies to internal persistent FIFO/endpoint buffers synchronously;
 * slot release only releases our CPU payload, never an active DMA buffer. */
static void sensorarraySerialOtgTxTask(void *arg)
{
    (void)arg;
    esp_err_t init = sensorarrayUsbOtgOwnerInit();
    atomic_store(&s_otgInitResult, init);
    atomic_store_explicit(&s_otgInitDone, true, memory_order_release);
    uint8_t pending[SERIAL_IO_LANE_COUNT] = {UINT8_MAX, UINT8_MAX, UINT8_MAX};
    size_t offset[SERIAL_IO_LANE_COUNT] = {0};
    bool delimiter[SERIAL_IO_LANE_COUNT] = {false};
    const bool sharedFormal = sensorarrayUsbOtgCdcEnabled();
    unsigned formalOwner = UINT8_MAX;
    uint32_t epoch = sensorarrayUsbOtgEpoch();
    static const unsigned order[] = {SERIAL_IO_PRIORITY, SERIAL_IO_DATA, SERIAL_IO_LOG};
    for (;;) {
        sensorarrayUsbOtgOwnerPoll();
        bool progress = false;
        bool formalServiced = false;
        uint32_t current = sensorarrayUsbOtgEpoch();
        if (current != epoch) {
            /* Bus reset class buffers were reset by TinyUSB before service.
             * Application records from the old connection retain their old
             * epoch and are discarded, never stamped as the new connection. */
            memset(delimiter, 0, sizeof(delimiter));
            formalOwner = UINT8_MAX;
            epoch = current;
        }
        for (unsigned turn = 0; turn < SERIAL_IO_LANE_COUNT; ++turn) {
            unsigned lane = order[turn];
            unsigned wire = sharedFormal && lane != SERIAL_IO_LOG ? SERIAL_IO_DATA : lane;
            if (pending[lane] == UINT8_MAX &&
                xQueueReceive(s_queues[lane], &pending[lane], 0) == pdTRUE) {
                offset[lane] = 0u; progress = true;
            }
            uint8_t index = pending[lane];
            if (index == UINT8_MAX) { continue; }
            int64_t now = esp_timer_get_time();
            bool stale = sensorarraySerialSlot(index)->epoch != epoch;
            if (stale || now >= sensorarraySerialSlot(index)->deadlineUs) {
                portENTER_CRITICAL(&s_statsMux);
                s_stats.sinkDropped++;
                s_stats.timeout += stale ? 0u : 1u;
                s_stats.partialWrite += offset[lane] ? 1u : 0u;
                if (lane == SERIAL_IO_PRIORITY) { s_stats.priorityExpired++; }
                else { s_stats.normalExpired++; }
                portEXIT_CRITICAL(&s_statsMux);
                if (!stale && offset[lane]) { delimiter[wire] = true; }
                if (formalOwner == lane) { formalOwner = UINT8_MAX; }
                sensorarraySerialReleaseSlot(index);
                pending[lane] = UINT8_MAX; offset[lane] = 0u; progress = true;
                continue;
            }
            if (sharedFormal && lane != SERIAL_IO_LOG) {
                if (formalOwner != UINT8_MAX && formalOwner != lane) { continue; }
                /* At most one formal record is serviced each turn. This
                 * permits queued CTRL first at the next record boundary,
                 * while an already accepted DATA prefix cannot be interleaved. */
                if (formalServiced || (lane == SERIAL_IO_DATA &&
                    pending[SERIAL_IO_PRIORITY] != UINT8_MAX && formalOwner != lane)) { continue; }
            }
            if (!sensorarrayUsbOtgOwnerCanWrite(lane)) { continue; }
            size_t length = delimiter[wire] ? 1u : sensorarraySerialSlot(index)->length - offset[lane];
            const char *data = delimiter[wire] ? "\n" : sensorarraySerialBytes(index) + offset[lane];
            int n = sensorarrayUsbOtgOwnerWrite(lane, data, length);
            bool unexpectedDataPrefix = sharedFormal && lane == SERIAL_IO_DATA && n > 0 && (size_t)n < length;
            if (n < 0 || (size_t)n > length || unexpectedDataPrefix) {
                /* Acceptance is unknown, including when offset is zero. Drop
                 * this CPU record; never retry an unknown prefix or report it
                 * submitted. The next record must begin after a delimiter. */
                portENTER_CRITICAL(&s_statsMux);
                s_stats.driverWriteError++; s_stats.sinkDropped++;
                s_stats.partialWrite += offset[lane] || unexpectedDataPrefix ? 1u : 0u;
                if (unexpectedDataPrefix) {
                    /* A violated whole-record SDK contract is a counted
                     * driver failure, never a prefix retained until expiry. */
                    s_stats.driverShortWrite++;
                    s_stats.driverAcceptedBytes += (uint32_t)n;
                    s_stats.lastTxUs = now;
                }
                portEXIT_CRITICAL(&s_statsMux);
                delimiter[wire] = true;
                if (formalOwner == lane) { formalOwner = UINT8_MAX; }
                sensorarraySerialReleaseSlot(index);
                pending[lane] = UINT8_MAX; offset[lane] = 0u; progress = true;
                continue;
            }
            if (n == 0) { continue; }
            progress = true;
            if (sharedFormal && lane != SERIAL_IO_LOG) { formalServiced = true; }
            uint64_t age = (uint64_t)(now - sensorarraySerialSlot(index)->enqueuedUs);
            portENTER_CRITICAL(&s_statsMux);
            s_stats.driverAcceptedBytes += (uint32_t)n;
            s_stats.lastTxUs = now;
            s_stats.driverShortWrite += (size_t)n < length ? 1u : 0u;
            if (age > s_stats.maxQueueAgeUs) { s_stats.maxQueueAgeUs = age > UINT32_MAX ? UINT32_MAX : (uint32_t)age; }
            portEXIT_CRITICAL(&s_statsMux);
            if (delimiter[wire]) { delimiter[wire] = false; continue; }
            if (sharedFormal && lane != SERIAL_IO_LOG) { formalOwner = lane; }
            offset[lane] += (size_t)n;
            if (offset[lane] == sensorarraySerialSlot(index)->length) {
                delimiter[wire] = sensorarraySerialBytes(index)[sensorarraySerialSlot(index)->length-1] != '\n';
                if (formalOwner == lane) { formalOwner = UINT8_MAX; }
                portENTER_CRITICAL(&s_statsMux);
                if (lane == SERIAL_IO_PRIORITY) { s_stats.prioritySubmitted++; }
                else {
                    s_stats.normalSubmitted++;
                    if (lane == SERIAL_IO_DATA) { s_stats.dataSubmitted++; }
                    else { s_stats.logSubmitted++; }
                }
                portEXIT_CRITICAL(&s_statsMux);
                sensorarraySerialReleaseSlot(index); pending[lane] = UINT8_MAX; offset[lane] = 0;
            }
        }
        sensorarraySerialTxCheckpoint();
        /* USB ISR callbacks, completed record admission, and finite deadline
         * housekeeping all wake this single owner. No unbounded no-host wait. */
        if (!progress) { (void)ulTaskNotifyTake(pdTRUE, 1u); }
    }
}

static void sensorarraySerialTxTask(void *arg)
{
    if (sensorarrayUsbOtgEnabled()) { sensorarraySerialOtgTxTask(arg); }
    else { sensorarraySerialLegacyTxTask(arg); }
}

esp_err_t sensorarraySerialIoInit(void)
{
    if (atomic_load_explicit(&s_vfsReady, memory_order_acquire)) {
        if (sensorarrayUsbOtgEnabled()) {
            return atomic_load_explicit(&s_otgInitDone, memory_order_acquire) ?
                atomic_load(&s_otgInitResult) : ESP_ERR_INVALID_STATE;
        }
        return ESP_OK;
    }
    bool otg = sensorarrayUsbOtgEnabled();
    bool cdc = otg && sensorarrayUsbOtgCdcEnabled();
    if (!s_dataBytes) {
        /* CPU-only payload: the driver copies each complete record into its own
         * internal ring buffer. Allocate once, outside every producer path. */
        if (cdc) {
            size_t metadataBytes = SERIAL_IO_CDC_DATA_SLOT_COUNT * sizeof(*s_dataSlots);
            size_t payloadBytes = SERIAL_IO_CDC_NORMAL_SLOT_COUNT * sizeof(*s_dataBytes);
            sensorarraySerialSlot_t *pool = heap_caps_calloc(1u, metadataBytes + payloadBytes,
                                                           MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
            if (pool) {
                s_dataSlots = pool;
                s_dataBytes = (char (*)[SENSORARRAY_TRANSPORT_TEXT_MAX])((uint8_t *)pool + metadataBytes);
                s_dataSlotCount = SERIAL_IO_CDC_DATA_SLOT_COUNT;
                s_dataQueueCount = SERIAL_IO_CDC_DATA_QUEUE_COUNT;
                s_dataLifetimeUs = SERIAL_IO_CDC_DATA_LIFETIME_US;
            }
        }
        if (!s_dataBytes) {
            /* A fragmented or unavailable large PSRAM block must not consume
             * equivalent internal RAM. Try the original small PSRAM pool
             * before falling back to the same small internal allocation. */
            s_dataBytes = heap_caps_calloc(SERIAL_IO_NORMAL_SLOT_COUNT, sizeof(*s_dataBytes),
                                          MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        }
        if (s_dataBytes) { s_dataMemory = SENSORARRAY_SERIAL_MEMORY_PSRAM; }
        else {
            s_dataBytes = heap_caps_calloc(SERIAL_IO_NORMAL_SLOT_COUNT, sizeof(*s_dataBytes),
                                          MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
            if (s_dataBytes) { s_dataMemory = SENSORARRAY_SERIAL_MEMORY_INTERNAL; }
        }
        if (!s_dataBytes) { return sensorarraySerialInitResult(ESP_ERR_NO_MEM); }
    }
    if (!otg && !usb_serial_jtag_is_driver_installed()) {
        usb_serial_jtag_driver_config_t config = {.tx_buffer_size = SERIAL_IO_DRIVER_TX_BYTES, .rx_buffer_size = 512};
        esp_err_t err = usb_serial_jtag_driver_install(&config);
        if (err != ESP_OK) { return sensorarraySerialInitResult(err); }
        s_driverOwned = true;
    } else if (!otg && !s_driverOwned) {
        /* The public API cannot reveal an existing driver's ring capacity.
         * Do not remove somebody else's live driver or claim atomic admission. */
        return sensorarraySerialInitResult(ESP_ERR_INVALID_STATE);
    }
    if (!s_task) {
        /* Hook errors remain visible through WorkBudget diagnostics; its
         * bounded fallback preserves this independent control/TX service. */
        (void)sensorarrayWorkBudgetInit();
        for (unsigned i = 0; i < SERIAL_IO_LANE_COUNT; ++i) {
            unsigned count = i == SERIAL_IO_DATA ? s_dataQueueCount :
                i == SERIAL_IO_LOG ? SERIAL_IO_LOG_QUEUE_COUNT : SERIAL_IO_QUEUE_COUNT;
            s_queues[i] = xQueueCreateStatic(count, sizeof(uint8_t),
                i == SERIAL_IO_DATA ? s_dataQueueBytes : s_queueBytes[i], &s_queueStorage[i]);
            if (!s_queues[i]) { return sensorarraySerialInitResult(ESP_ERR_NO_MEM); }
        }
        /* Backend selection is cached for this boot. Set priority only when
         * creating its one owner; no live task/backend transition occurs. */
        unsigned txPriority = otg ? SERIAL_IO_OTG_TX_TASK_PRIORITY : SERIAL_IO_TX_TASK_PRIORITY;
        if (xTaskCreatePinnedToCore(sensorarraySerialTxTask, "serialTx", 4096, NULL, txPriority, &s_task,
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
    if (otg) {
        int64_t deadline = esp_timer_get_time() + 1000000;
        while (!atomic_load_explicit(&s_otgInitDone, memory_order_acquire)) {
            if (esp_timer_get_time() >= deadline) { return sensorarraySerialInitResult(ESP_ERR_TIMEOUT); }
            vTaskDelay(1u);
        }
        return sensorarraySerialInitResult(atomic_load(&s_otgInitResult));
    }
    return sensorarraySerialInitResult(ESP_OK);
}

int sensorarraySerialIoReadWithEpoch(void *buffer, size_t length, uint32_t *epoch)
{
    if (sensorarrayUsbOtgEnabled()) {
        if (!atomic_load_explicit(&s_otgInitDone, memory_order_acquire) || atomic_load(&s_otgInitResult) != ESP_OK) { return 0; }
        return sensorarrayUsbOtgRead(buffer, length, epoch);
    }
    if (epoch) { *epoch = 0u; }
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

int sensorarraySerialIoRead(void *buffer, size_t length)
{ return sensorarraySerialIoReadWithEpoch(buffer, length, NULL); }

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
    outStats->normalBufferBytes = s_dataBytes ? (s_dataSlotCount + SERIAL_IO_LOG_SLOT_COUNT) * sizeof(*s_dataBytes) : 0u;
    outStats->dataQueueCapacity = s_dataQueueCount;
    outStats->dataRecordLifetimeUs = s_dataLifetimeUs;
    outStats->driverTxBufferBytes = s_driverOwned ? SERIAL_IO_DRIVER_TX_BYTES : 0u;
    outStats->normalBufferMemory = s_dataMemory;
    outStats->taskStackConfiguredBytes = 4096u;
    outStats->taskStackMinBytes = s_task ? (uint32_t)uxTaskGetStackHighWaterMark(s_task) * sizeof(StackType_t) : 0u;
    outStats->driverInstalled = !sensorarrayUsbOtgEnabled() && usb_serial_jtag_is_driver_installed();
    outStats->sofPresent = !sensorarrayUsbOtgEnabled() && usb_serial_jtag_is_connected();
    outStats->stdioBound = atomic_load_explicit(&s_vfsReady, memory_order_acquire);
}

#include <assert.h>
#include <errno.h>
#include <setjmp.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include "esp_heap_caps.h"
#include "common.h"
#include "sensorarraySerialIo.h"
#include "sensorarrayTransport.h"
#include "sensorarrayWorkBudget.h"
#include "idf_vfs_prefix.h"
#include "idf_reent_init.h"
#include "serial_capture.h"
#include "serial_fault_burst.h"
#if defined(SERIAL_IO_BASELINE) || defined(SERIAL_IO_FRAGMENT_BASELINE) || defined(SERIAL_IO_TX_BASELINE) || defined(SERIAL_IO_ADMISSION_BASELINE)
#define sensorarraySerialIoQueueData(data, length) sensorarraySerialIoQueue(data, length, false)
#endif
static int64_t nowUs;
static unsigned allocationCalls, allocationsToFail, allocatedBuffers;
static unsigned queueCreateCalls, queueFailureAt, taskFailures;
static bool psramUnavailable;
static int installCount, clearCount, writeBudget = -1, scenario, readCount;
static unsigned notificationsGiven, notificationsPending;
static unsigned driverCapacity, driverCalls, delayCalls;
static TickType_t lastDriverWaitTicks;
static int64_t lastDriverAttemptUs;
static int64_t releaseAfterUs;
static bool injectPriorityOnBlocked, injectPriorityAfterShort, injectedPriority;
static bool injectDataAfterShort;
static bool injectRxOnDelay;
static unsigned refillNormalRemaining;
static char refillPacket[SENSORARRAY_TRANSPORT_TEXT_MAX];
static bool injectControlOnIdle;
static bool (*idleCallback)(void);
static bool observeIdle = true;
TickType_t xTaskGetTickCount(void) { return (TickType_t)(nowUs / (portTICK_PERIOD_MS * 1000)); }
int xPortGetCoreID(void) { return 0; }
esp_err_t esp_register_freertos_idle_hook_for_cpu(bool (*callback)(void), unsigned cpu)
{
    assert(cpu == 0); idleCallback = callback; return ESP_OK;
}
static void resetClock(void)
{
    nowUs = 0;
    if (idleCallback && observeIdle) { assert(idleCallback()); }
}
static bool installed;
static void (*txTask)(void *);
static bool taskEntered;
static jmp_buf taskExit;
#ifdef SERIAL_IO_TEST_OTG_INIT
static bool testOtgEnabled, runOtgOwnerOnDelay, otgOwnerYieldArmed;
static esp_err_t testOtgInitResult;
static unsigned otgInitCalls;
static jmp_buf otgOwnerYield;
bool sensorarrayUsbOtgEnabled(void) { return testOtgEnabled; }
esp_err_t sensorarrayUsbOtgOwnerInit(void)
{ assert(testOtgEnabled && otgOwnerYieldArmed); otgInitCalls++; return testOtgInitResult; }
void sensorarrayUsbOtgOwnerPoll(void)
{ assert(testOtgEnabled && otgOwnerYieldArmed); longjmp(otgOwnerYield, 1); }
bool sensorarrayUsbOtgOwnerCanWrite(unsigned lane)
{ (void)lane; assert(false && "init-prefix test must not service OTG endpoints"); return false; }
int sensorarrayUsbOtgOwnerWrite(unsigned lane, const void *data, size_t length)
{ (void)lane; (void)data; (void)length; assert(false); return -1; }
int sensorarrayUsbOtgRead(void *buffer, size_t length, uint32_t *epoch)
{ (void)buffer; (void)length; (void)epoch; assert(false && "unready OTG must not read"); return -1; }
static void runOtgOwnerInitPrefix(void)
{
    otgOwnerYieldArmed = true;
    if (setjmp(otgOwnerYield) == 0) { txTask(NULL); }
    otgOwnerYieldArmed = false;
}
#endif
/* Bytes accepted into the mock driver, not a claim of PC receipt. */
static char output[128000];
static size_t outputLength;
static esp_vfs_t registeredVfs;
static unsigned registrationFailures, stdoutBindFailures, stderrBindFailures, routeProbeFailures;
static unsigned registrationCalls, stdoutBindCalls, stderrBindCalls;
static bool registered;
static char registeredPrefix[32];
static bool injectConcurrentLog, injectedConcurrentLog;
typedef struct { bool queued, closed; int localFd; FILE *stream; } stdioRoute_t;
static stdioRoute_t stdoutRoute, stderrRoute;
static FILE *test_stdout, *test_stderr;
/* ESP-IDF's esp_reent_init shares these FILE objects with tasks created before
 * and after Init. The cookie preserves FILE identity, just as freopen does. */
static FILE *preInitTaskStdout, *postInitTaskStderr;

static ssize_t stdioCookieWrite(void *context, const char *data, size_t length)
{
    stdioRoute_t *route = context;
    if (route->closed) { errno = EBADF; return -1; }
    if (route->queued) { return registeredVfs.write(route->localFd, data, length); }
    /* Original native console VFS: a direct physical writer, with CRLF. */
    for (size_t i = 0; i < length; ++i) {
        if (data[i] == '\n' && (i == 0 || data[i - 1] != '\r')) { output[outputLength++] = '\r'; }
        output[outputLength++] = data[i];
    }
    return (ssize_t)length;
}

static void initStdio(void)
{
    cookie_io_functions_t io = {.write = stdioCookieWrite};
    test_stdout = stdoutRoute.stream = fopencookie(&stdoutRoute, "w", io);
    test_stderr = stderrRoute.stream = fopencookie(&stderrRoute, "w", io);
    assert(test_stdout && test_stderr);
    assert(setvbuf(test_stdout, NULL, _IONBF, 0) == 0);
    assert(setvbuf(test_stderr, NULL, _IONBF, 0) == 0);
    testGlobalReent._stdout = test_stdout;
    testGlobalReent._stderr = test_stderr;
    struct _reent preInitTask;
    esp_reent_init(&preInitTask);
    preInitTaskStdout = preInitTask._stdout;
}

void *heap_caps_calloc(size_t count, size_t size, uint32_t caps)
{
    allocationCalls++;
    if (allocationsToFail) { allocationsToFail--; return NULL; }
    if (psramUnavailable && (caps & MALLOC_CAP_SPIRAM)) { return NULL; }
    assert(caps == (MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT) ||
           caps == (MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
    void *buffer = calloc(count, size);
    if (buffer) { allocatedBuffers++; }
    return buffer;
}

QueueHandle_t xQueueCreateStatic(unsigned count, unsigned size, uint8_t *bytes, StaticQueue_t *queue)
{
    assert(size == 1);
    if (++queueCreateCalls == queueFailureAt) { return NULL; }
    if (queue->length) { assert(pthread_mutex_destroy(&queue->lock) == 0); }
    *queue = (StaticQueue_t){.length=count, .data=bytes};
    assert(pthread_mutex_init(&queue->lock, NULL) == 0); return queue;
}
int xQueueSend(QueueHandle_t q, const void *item, TickType_t ticks)
{
    assert(ticks == 0); pthread_mutex_lock(&q->lock);
    if (q->count == q->length) { pthread_mutex_unlock(&q->lock); return 0; }
    q->data[q->write++ % q->length] = *(const uint8_t *)item; q->count++;
    pthread_mutex_unlock(&q->lock); return 1;
}
int xQueueReceive(QueueHandle_t q, void *item, TickType_t ticks)
{
    assert(ticks == 0); pthread_mutex_lock(&q->lock);
    if (!q->count) { pthread_mutex_unlock(&q->lock); return 0; }
    *(uint8_t *)item = q->data[q->read++ % q->length]; q->count--;
    pthread_mutex_unlock(&q->lock); return 1;
}
int xTaskCreatePinnedToCore(void (*fn)(void *), const char *name, unsigned stack, void *arg, unsigned prio, TaskHandle_t *task, int core)
{
    if (taskFailures) { taskFailures--; return 0; }
    txTask = fn; *task = (void *)1; return pdPASS;
}
int xTaskNotifyGive(TaskHandle_t task) { notificationsGiven++; notificationsPending++; return 1; }
uint32_t ulTaskNotifyTake(int clear, TickType_t ticks)
{
    if (taskEntered && injectControlOnIdle) {
        injectControlOnIdle = false;
        scenario = 0; writeBudget = -1;
        assert(sensorarraySerialIoQueue("IDLE_RECOVERED\n", 15, true) == ESP_OK);
        return 1;
    }
    if (taskEntered && notificationsPending == 0u) { longjmp(taskExit, 1); }
    taskEntered = true;
    uint32_t taken = notificationsPending;
    notificationsPending = 0u; /* pdTRUE consumes the complete wake count. */
    return taken ? taken : 1u;
}
void vTaskDelay(TickType_t ticks)
{
    assert(ticks > 0); delayCalls++; nowUs += ticks * portTICK_PERIOD_MS * 1000;
#ifdef SERIAL_IO_TEST_OTG_INIT
    if (runOtgOwnerOnDelay) { runOtgOwnerOnDelay = false; runOtgOwnerInitPrefix(); }
#endif
    if (idleCallback && observeIdle) { assert(idleCallback()); }
    if (injectRxOnDelay) {
        injectRxOnDelay = false;
        char input[64]; assert(sensorarraySerialIoRead(input, sizeof(input)) == 6);
    }
}
unsigned uxTaskGetStackHighWaterMark(TaskHandle_t task) { return 2048; }
int64_t esp_timer_get_time(void) { return nowUs; }
esp_err_t usb_serial_jtag_driver_install(usb_serial_jtag_driver_config_t *config)
{
    driverCapacity = config->tx_buffer_size;
    installed = true; installCount++; return ESP_OK;
}
bool usb_serial_jtag_is_driver_installed(void) { return installed; }
bool usb_serial_jtag_is_connected(void) { return scenario != 1; }
int usb_serial_jtag_write_bytes(const void *data, size_t length, TickType_t ticks)
{
    assert(ticks <= 1 && length <= driverCapacity); driverCalls++;
    lastDriverWaitTicks = ticks;
    lastDriverAttemptUs = nowUs;
    if (scenario == 4) { scenario = 0; return -1; }
    if (scenario == 5) { return 0; } /* broken driver: no wait despite requested ticks */
    if (scenario == 2 && nowUs >= releaseAfterUs) { writeBudget = -1; }
    /* IDF BYTEBUF commits the whole requested item or none. A host that frees
     * space after 30ms reproduces the original 64B/20ms cutoff, but permits a
     * retained complete record with a bounded blocking wait to succeed. */
    bool atomicBlocked = scenario == 3 && nowUs < releaseAfterUs &&
                         (writeBudget == 0 || length > (size_t)writeBudget);
    if (writeBudget == 0 || atomicBlocked) {
        nowUs += ticks * portTICK_PERIOD_MS * 1000;
        if (ticks && idleCallback && observeIdle) { assert(idleCallback()); }
        if (injectPriorityOnBlocked && !injectedPriority) {
            injectedPriority = true;
            assert(sensorarraySerialIoQueue("CTRL,blocked=1\n", 15, true) == ESP_OK);
        }
        if (scenario == 3 && nowUs >= releaseAfterUs) { writeBudget = -1; }
        else { return 0; }
    }
    if (writeBudget > 0 && length > (size_t)writeBudget) { length = (size_t)writeBudget; }
    assert(outputLength + length < sizeof(output));
    memcpy(output + outputLength, data, length); outputLength += length;
    if (writeBudget > 0) { writeBudget -= (int)length; }
    if (injectPriorityAfterShort && !injectedPriority) {
        injectedPriority = true;
        assert(sensorarraySerialIoQueue("AFTER\n", 6, true) == ESP_OK);
        if (injectDataAfterShort) { assert(sensorarraySerialIoQueueData("DATA_AFTER\n", 11) == ESP_OK); }
    }
    if (refillNormalRemaining && length == sizeof(refillPacket) &&
        memcmp(data, refillPacket, length) == 0) {
        refillNormalRemaining--;
        assert(sensorarraySerialIoQueueData(refillPacket, sizeof(refillPacket)) == ESP_OK);
    }
    if (injectConcurrentLog && !injectedConcurrentLog) {
#if defined(SERIAL_IO_BASELINE)
        assert(length == 64); /* the observed old physical insertion boundary */
#else
        assert(length == sizeof(capturedConfig) - 1); /* now one complete record */
#endif
        injectedConcurrentLog = true;
        assert(fwrite(capturedPr, 1, sizeof(capturedPr) - 1, preInitTaskStdout) == sizeof(capturedPr) - 1);
        if (postInitTaskStderr) {
            assert(fwrite("STDERR,concurrent=1\n", 1, 20, postInitTaskStderr) == 20);
            assert(sensorarraySerialIoQueue("CTRL,concurrent=1\n", 18, true) == ESP_OK);
        }
    }
    return (int)length;
}
int usb_serial_jtag_read_bytes(void *data, uint32_t length, TickType_t ticks)
{
    assert(ticks > 0); memcpy(data, "MODE?\n", 6); readCount++; return 6;
}
esp_err_t usb_serial_jtag_wait_tx_done(TickType_t ticks) { assert(ticks == 0); return ESP_OK; }
esp_err_t esp_vfs_register(const char *path, const esp_vfs_t *vfs, void *context)
{
    registrationCalls++;
    if (!is_path_prefix_valid(path, strlen(path))) { return ESP_ERR_INVALID_ARG; }
    if (registrationFailures) { registrationFailures--; return ESP_ERR_NO_MEM; }
    registeredVfs = *vfs; registered = true;
    snprintf(registeredPrefix, sizeof(registeredPrefix), "%s", path);
    return ESP_OK;
}
FILE *test_freopen(const char *path, const char *mode, FILE *stream)
{
    assert(registered && strncmp(path, registeredPrefix, strlen(registeredPrefix)) == 0);
    const char *localPath = path + strlen(registeredPrefix);
    if (!*localPath) { localPath = "/"; }
    stdioRoute_t *route = stream == test_stdout ? &stdoutRoute : &stderrRoute;
    if (route->queued && !route->closed) { assert(registeredVfs.close(route->localFd) == 0); }
    if (stream == test_stdout) {
        stdoutBindCalls++;
        if (stdoutBindFailures) { stdoutBindFailures--; stdoutRoute.closed = true; return NULL; }
        stdoutRoute.queued = true; stdoutRoute.closed = false;
    } else {
        assert(stream == test_stderr);
        stderrBindCalls++;
        if (stderrBindFailures) { stderrBindFailures--; stderrRoute.closed = true; return NULL; }
        stderrRoute.queued = true; stderrRoute.closed = false;
    }
    route->localFd = registeredVfs.open(localPath, 0, 0);
    assert(route->localFd >= 0);
    return stream;
}
int test_setvbuf(FILE *stream, char *buffer, int mode, size_t size) { return setvbuf(stream, buffer, mode, size); }
void test_clearerr(FILE *stream) { clearCount++; clearerr(stream); }
int test_fileno(FILE *stream) { assert(stream == test_stdout || stream == test_stderr); return stream == test_stdout ? 1 : 2; }
int test_fstat(int fd, struct stat *st)
{
    stdioRoute_t *route = fd == 1 ? &stdoutRoute : &stderrRoute;
    if (route->closed) { errno = EBADF; return -1; }
    if (route->queued && !routeProbeFailures) { return registeredVfs.fstat(route->localFd, st); }
    if (routeProbeFailures) { routeProbeFailures--; }
    memset(st, 0, sizeof(*st)); st->st_mode = S_IFCHR; return 0;
}

#define freopen test_freopen
#define setvbuf test_setvbuf
#define clearerr test_clearerr
#define fileno(stream) test_fileno(stream)
#define fstat(fd, st) test_fstat(fd, st)
#undef stdout
#undef stderr
#define stdout test_stdout
#define stderr test_stderr
#ifndef SERIAL_IO_SOURCE
#define SERIAL_IO_SOURCE "../../core/transport/sensorarraySerialIo.c"
#endif
#include SERIAL_IO_SOURCE
#undef freopen
#undef setvbuf
#undef clearerr
#undef fileno
#undef fstat
#undef stdout
#undef stderr

static void drain(void)
{
    taskEntered = false;
    if (setjmp(taskExit) == 0) { txTask(NULL); }
}
#if !defined(SERIAL_IO_BASELINE) && !defined(SERIAL_IO_FRAGMENT_BASELINE) && !defined(SERIAL_IO_TX_BASELINE) && !defined(SERIAL_IO_ADMISSION_BASELINE)
typedef struct { int unused; } sensorarrayFrame_t;
typedef struct { uint32_t sequence; uint16_t length; char data[SENSORARRAY_TRANSPORT_TEXT_MAX]; } sensorarrayTextPacket_t;
static bool serialSinkEnabled = true;
static esp_err_t encoderResult;
static unsigned encoderCalls;
bool sensorarrayTransportSerialSinkEnabled(void) { return serialSinkEnabled; }
static esp_err_t sensorarrayTextProtocolBuildFrame(const sensorarrayFrame_t *frame, sensorarrayTextPacket_t *packet)
{
    encoderCalls++;
    if (encoderResult != ESP_OK) { return encoderResult; }
    packet->length = sizeof(encodedFrame) - 1;
    memcpy(packet->data, encodedFrame, packet->length);
    return ESP_OK;
}
/* Actual legacy fallback function, extracted verbatim. The encoder stub
 * replays the existing real-encoder golden frame with verified CRC. */
#include "frame_output_function.h"
#include "usb_sink_functions.h"
static pthread_barrier_t fragmentBarrier;
typedef struct { FILE *stream; const char *name; } fragmentWriter_t;
static void *concurrentFragments(void *context)
{
    fragmentWriter_t *writer = context;
    for (unsigned i = 0; i < 32; ++i) {
        assert(fprintf(writer->stream, "%s,id=%u,", writer->name, i) > 0);
        (void)pthread_barrier_wait(&fragmentBarrier);
        (void)pthread_barrier_wait(&fragmentBarrier);
        assert(fputs("done=1\n", writer->stream) >= 0);
        (void)pthread_barrier_wait(&fragmentBarrier);
        (void)pthread_barrier_wait(&fragmentBarrier);
    }
    return NULL;
}
#endif
int main(int argc, char **argv)
{
    resetClock();
    initStdio();
#ifdef SERIAL_IO_BASELINE
    /* Compile the unmodified captured firmware source against IDF's actual
     * prefix validator. This is the real defect, not an invented timeout. */
    assert(sensorarraySerialIoInit() == ESP_ERR_INVALID_ARG);
    assert(txTask && installed && !registered && !stdoutBindCalls);
    assert(sensorarraySerialIoQueue(capturedConfig, sizeof(capturedConfig) - 1, true) == ESP_OK);
    injectConcurrentLog = true;
    drain();
    assert(memcmp(output, capturedConfig, 64) == 0);
    assert(memcmp(output + 64, capturedPr, sizeof(capturedPr) - 1) == 0);
    assert(memcmp(output + 64 + sizeof(capturedPr) - 1,
                  capturedConfig + 64, sizeof(capturedConfig) - 1 - 64) == 0);
    printf("SERIAL_IO_BASELINE_REPRODUCED,pathBytes=16,idfPathMax=%u,initError=0x102,configBytes=%zu,insertionByte=64,directConsole=1\n",
           ESP_VFS_PATH_MAX, sizeof(capturedConfig) - 1);
    return 0;
#elif defined(SERIAL_IO_FRAGMENT_BASELINE)
    assert(sensorarraySerialIoInit() == ESP_OK);
    sensorarraySerialIoStats_t before, after;
    sensorarraySerialIoGetStats(&before);
    assert(before.stdioBound && before.initError == ESP_OK);
    assert(fwrite(capturedAdsPrefix, 1, sizeof(capturedAdsPrefix) - 1, test_stdout) == sizeof(capturedAdsPrefix) - 1);
    drain(); /* Old VFS publishes the un-terminated first printf fragment. */
    for (unsigned i = 0; i < 4; ++i) { assert(sensorarraySerialIoQueueData("FILL\n", 5) == ESP_OK); }
    (void)fwrite(",synthetic_suffix=1\n", 1, 20, test_stdout);
    assert(sensorarraySerialIoQueue(capturedUsbReply, sizeof(capturedUsbReply) - 1, true) == ESP_OK);
    drain();
    assert(memcmp(output, capturedAdsPrefix, sizeof(capturedAdsPrefix) - 1) == 0);
    assert(memcmp(output + sizeof(capturedAdsPrefix) - 1, capturedUsbReply, sizeof(capturedUsbReply) - 1) == 0);
    sensorarraySerialIoGetStats(&after);
    assert(after.sinkDropped > before.sinkDropped && after.timeout == before.timeout && after.partialWrite == before.partialWrite);
    printf("SERIAL_IO_FRAGMENT_BASELINE_REPRODUCED,stdioBound=1,initError=0,prefixBytes=128,queueDrop=1,partialWriteDelta=0,timeoutDelta=0\n");
    return 0;
#elif defined(SERIAL_IO_TX_BASELINE)
    assert(sensorarraySerialIoInit() == ESP_OK);
    scenario = 3; writeBudget = 64; releaseAfterUs = 30000; nowUs = 0;
    assert(sensorarraySerialIoQueue(capturedConfig, sizeof(capturedConfig) - 1, true) == ESP_OK);
    drain();
    sensorarraySerialIoStats_t stats;
    sensorarraySerialIoGetStats(&stats);
    assert(outputLength == 64 && memcmp(output, capturedConfig, 64) == 0);
    assert(nowUs == 20000 && stats.partialWrite == 1 && stats.timeout == 1);
    printf("SERIAL_IO_TX_BASELINE_REPRODUCED,tickHz=100,driverAtomicItem=1,hostReadyUs=30000,packetBudgetUs=20000,configPrefixBytes=64,priorityTailLost=1\n");
    return 0;
#elif defined(SERIAL_IO_ADMISSION_BASELINE)
    assert(sensorarraySerialIoInit() == ESP_OK);
    /* Synthetic scheduling: producer priority 7 fills the shared four-record
     * queue before priority-4 TX can run, as admitted by the old call graph. */
    for (unsigned i = 0; i < 4; ++i) { assert(sensorarraySerialIoQueue("DIAGNOSTIC\n", 11, false) == ESP_OK); }
    assert(sensorarraySerialIoQueueData(encodedFrame, sizeof(encodedFrame) - 1) == ESP_ERR_TIMEOUT);
    assert(sensorarraySerialIoQueue("CTRL\n", 5, true) == ESP_OK);
    drain();
    sensorarraySerialIoStats_t stats; sensorarraySerialIoGetStats(&stats);
    assert(stats.sinkDropped == 1 && stats.timeout == 0 && stats.partialWrite == 0);
    assert(outputLength == 49 && memcmp(output, "CTRL\n", 5) == 0);
    printf("SERIAL_ADMISSION_BASELINE_REPRODUCED,diagnosticRecords=4,dataRejected=1,driverExpiry=0,partial=0,prioritySurvives=1,syntheticScheduling=1\n");
    /* Baseline capability: an empty shared queue preserves the four leading
     * production-formatted fault records, then counts three refused tail lines. */
    outputLength = 0;
    uint32_t faultDrops = stats.lineDropped;
    for (const char *start = faultBurst; *start;) {
        const char *end = strchr(start, '\n'); assert(end);
        size_t length = (size_t)(end - start) + 1;
        assert(registeredVfs.write(0, start, length) == (ssize_t)length);
        start = end + 1;
    }
    drain(); sensorarraySerialIoGetStats(&stats);
    assert(outputLength == FAULT_FIRST_FOUR_BYTES && memcmp(output, faultBurst, outputLength) == 0);
    assert(stats.lineDropped == faultDrops + 3);
    printf("SERIAL_FAULT_CAPACITY_BASELINE,offered=7,firstFourExact=1,tailDropped=3,producer=actual_C_formatter\n");
    return 0;
#else
    const char *allocation = argc > 1 ? argv[1] : "psram";
#ifdef SERIAL_IO_TEST_OTG_INIT
    if (strncmp(allocation, "otg_", 4u) == 0) {
        bool neverScheduled = strcmp(allocation, "otg_timeout_repeated") == 0;
        testOtgEnabled = true;
        testOtgInitResult = strcmp(allocation, "otg_init_no_mem") == 0 ? ESP_ERR_NO_MEM : ESP_FAIL;
        runOtgOwnerOnDelay = !neverScheduled;
        esp_err_t first = sensorarraySerialIoInit();
        assert(first == (neverScheduled ? ESP_ERR_TIMEOUT : testOtgInitResult));
        assert(!installed && installCount == 0 && !s_driverOwned && s_task);
        assert(allocatedBuffers == 1u && atomic_load(&s_vfsReady));
        assert(otgInitCalls == (neverScheduled ? 0u : 1u));
        assert(nowUs <= 1000000);
        int64_t beforeRepeat = nowUs;
        unsigned allocationsBeforeRepeat = allocationCalls;
        esp_err_t second = sensorarraySerialIoInit();
        assert(second == (neverScheduled ? ESP_ERR_INVALID_STATE : testOtgInitResult));
        assert(second != ESP_OK && nowUs == beforeRepeat && allocationCalls == allocationsBeforeRepeat);
        char input[64]; uint32_t epoch = UINT32_MAX;
        assert(sensorarraySerialIoReadWithEpoch(input, sizeof(input), &epoch) == 0);
        if (neverScheduled) {
            /* A late owner completion publishes a real ESP_FAIL (-1), not a
             * reused pending sentinel. Never re-create/free the live owner. */
            runOtgOwnerInitPrefix();
            assert(otgInitCalls == 1u && atomic_load(&s_otgInitDone));
            assert(sensorarraySerialIoInit() == ESP_FAIL);
            assert(sensorarraySerialIoRead(input, sizeof(input)) == 0);
        }
        printf("SERIAL_OTG_INIT_PASS,case=%s,first=0x%X,repeat=0x%X,initCalls=%u,usjInstalled=0,hotReuse=0,hardware=NOT_RUN\n",
               allocation, (unsigned)first, (unsigned)second, otgInitCalls);
        return 0;
    }
#endif
    if (strcmp(allocation, "foreign_driver") == 0) {
        installed = true;
        assert(sensorarraySerialIoInit() == ESP_ERR_INVALID_STATE);
        assert(!txTask && !s_driverOwned && installCount == 0);
        assert(sensorarraySerialIoQueue("UNSAFE\n", 7, true) == ESP_ERR_INVALID_STATE);
        printf("SERIAL_IO_PASS,foreignDriverNotRemovedOrAssumed=1\n");
        return 0;
    }
    psramUnavailable = strcmp(allocation, "internal") == 0;
    if (strcmp(allocation, "failure_retry") == 0) {
        allocationsToFail = 2;
        assert(sensorarraySerialIoInit() == ESP_ERR_NO_MEM);
        assert(installCount == 0 && txTask == NULL && s_dataBytes == NULL);
    }
    if (strcmp(allocation, "registration_retry") == 0) {
        registrationFailures = 1;
        assert(sensorarraySerialIoInit() == ESP_ERR_NO_MEM);
        assert(txTask && installed && !registered);
        assert(sensorarraySerialIoQueue("UNSAFE\n", 7, true) == ESP_ERR_INVALID_STATE);
        assert(fwrite("EARLY\n", 1, 6, test_stdout) == 6);
        assert(outputLength == 7 && memcmp(output, "EARLY\r\n", 7) == 0);
        char rx[64]; assert(sensorarraySerialIoRead(rx, sizeof(rx)) == 6);
        sensorarraySerialIoStats_t failed;
        sensorarraySerialIoGetStats(&failed);
        assert(!failed.stdioBound && failed.initError == ESP_ERR_NO_MEM);
        outputLength = 0; readCount = 0; s_stats.rxBytes = 0;
    }
    if (strcmp(allocation, "queue_retry") == 0 || strcmp(allocation, "task_retry") == 0) {
        queueFailureAt = strcmp(allocation, "queue_retry") == 0 ? 2u : 0u;
        taskFailures = strcmp(allocation, "task_retry") == 0 ? 1u : 0u;
        assert(sensorarraySerialIoInit() == ESP_ERR_NO_MEM);
        assert(s_dataBytes && installed && !txTask && allocatedBuffers == 1u);
        assert(sensorarraySerialIoQueueData("NOT_READY\n", 10) == ESP_ERR_INVALID_STATE);
    }
    if (strcmp(allocation, "stderr_retry") == 0) {
        stderrBindFailures = 1;
        assert(sensorarraySerialIoInit() == ESP_FAIL);
        assert(s_stdoutBound && !s_stderrBound);
        assert(sensorarraySerialIoQueue("UNSAFE\n", 7, true) == ESP_ERR_INVALID_STATE);
        assert(stderrRoute.closed);
        (void)fwrite("WAIT\n", 1, 5, test_stdout);
        assert(ferror(test_stdout) && outputLength == 0);
    }
    if (strcmp(allocation, "stdout_retry") == 0) {
        stdoutBindFailures = 1;
        assert(sensorarraySerialIoInit() == ESP_FAIL);
        assert(stdoutRoute.closed && !s_stdoutBound && !s_stderrBound);
        assert(sensorarraySerialIoQueue("UNSAFE\n", 7, true) == ESP_ERR_INVALID_STATE);
    }
    if (strcmp(allocation, "route_probe_retry") == 0) {
        routeProbeFailures = 1;
        assert(sensorarraySerialIoInit() == ESP_FAIL);
        assert(!s_stdoutBound && !s_stderrBound);
        assert(sensorarraySerialIoQueue("UNSAFE\n", 7, true) == ESP_ERR_INVALID_STATE);
    }
    assert(sensorarraySerialIoInit() == ESP_OK);
    struct _reent postInitTask;
    esp_reent_init(&postInitTask);
    postInitTaskStderr = postInitTask._stderr;
    assert(preInitTaskStdout == test_stdout && stdoutRoute.queued && stderrRoute.queued);
    assert(stdoutBindCalls == (strcmp(allocation, "route_probe_retry") == 0 || strcmp(allocation, "stdout_retry") == 0 ? 2u : 1u));
    assert(stderrBindCalls == (strcmp(allocation, "stderr_retry") == 0 ? 2u : 1u));
    assert(!stdoutRoute.closed && !stderrRoute.closed && !ferror(test_stdout) && !ferror(test_stderr));
    unsigned allocationsAfterInit = allocationCalls;
    assert(allocatedBuffers == 1u);
    assert(sensorarraySerialIoInit() == ESP_OK && installCount == 1);
    assert(driverCapacity == SENSORARRAY_TRANSPORT_TEXT_MAX);
    assert(sensorarraySerialIoQueueData("DATA\n", 5) == ESP_OK);
    assert(sensorarraySerialIoQueue("MAPP\n", 5, true) == ESP_OK);
    drain();
    assert(outputLength == 10 && memcmp(output, "MAPP\nDATA\n", 10) == 0);
    /* DATA floods cannot use reserved priority slots. */
    for (unsigned i = 0; i < 4; ++i) { assert(sensorarraySerialIoQueueData("D\n", 2) == ESP_OK); }
    assert(sensorarraySerialIoQueueData("DROP\n", 5) == ESP_ERR_TIMEOUT);
    assert(sensorarraySerialIoQueue("CTRL\n", 5, true) == ESP_OK);
    drain();
    assert(clearCount >= 2); /* binding retries clear ferror; TX takes no FILE lock */
    /* No host: bounded TX + a still functional independent command RX. */
    scenario = 1; writeBudget = 0; resetClock();
    injectRxOnDelay = true;
    assert(sensorarraySerialIoQueueData("DATA\n", 5) == ESP_OK);
    drain(); assert(nowUs == SERIAL_IO_NORMAL_LIFETIME_US);
    assert(readCount == 1 && !injectRxOnDelay); /* RX progressed while TX waited. */
    sensorarraySerialIoStats_t stats;
    sensorarraySerialIoGetStats(&stats);
    assert(stats.stdioBound && stats.initError == ESP_OK);
    assert(stats.normalBufferBytes == SERIAL_IO_NORMAL_SLOT_COUNT * SENSORARRAY_TRANSPORT_TEXT_MAX);
    assert(stats.normalBufferMemory == (psramUnavailable ? SENSORARRAY_SERIAL_MEMORY_INTERNAL : SENSORARRAY_SERIAL_MEMORY_PSRAM));
    assert(stats.timeout == 1 && stats.rxBytes == 6 && !stats.sofPresent);
    assert(stats.normalExpired == 1 && stats.priorityExpired == 0);
    /* Exact original CONFIG example: 64B free until 30ms, then sufficient
     * driver space. Full-item admission emits no prefix during the stall. */
    scenario = 3; writeBudget = 64; releaseAfterUs = 30000; resetClock(); outputLength = 0;
    assert(sensorarraySerialIoQueue(capturedConfig, sizeof(capturedConfig) - 1, true) == ESP_OK);
    drain();
    assert(nowUs == 30000 && outputLength == sizeof(capturedConfig) - 1);
    assert(memcmp(output, capturedConfig, outputLength) == 0);
    sensorarraySerialIoGetStats(&stats);
    assert(stats.partialWrite == 0 && stats.timeout == 1 && stats.prioritySubmitted >= 1);
    /* If a future driver returns an actual positive short count, ownership
     * and cursor remain live until completion or bounded cancellation. */
    scenario = 2; writeBudget = 80; releaseAfterUs = SERIAL_IO_NORMAL_LIFETIME_US;
    resetClock(); outputLength = 0;
    injectPriorityAfterShort = true; injectedPriority = false;
    char packet[160]; memset(packet, 'A', sizeof(packet));
    assert(sensorarraySerialIoQueueData(packet, sizeof(packet)) == ESP_OK);
    drain();
    assert(outputLength == 87 && memcmp(output + 80, "\nAFTER\n", 7) == 0);
    sensorarraySerialIoGetStats(&stats);
    assert(stats.partialWrite == 1 && stats.timeout == 2 && stats.normalExpired == 2);
    assert(stats.driverShortWrite == 1 && stats.driverAcceptedBytes > 80);
    injectPriorityAfterShort = false;
    /* VFS consumes complete lines, but a full queue is a counted whole-line
     * drop. Its return value is not protocol/DATA queue admission. */
    scenario = 0; writeBudget = -1;
    uint32_t droppedBefore = stats.lineDropped;
    for (unsigned i = 0; i < SERIAL_IO_LOG_QUEUE_COUNT; ++i) { assert(registeredVfs.write(0, "LOG\n", 4) == 4); }
    assert(registeredVfs.write(0, "LOG\n", 4) == 4);
    sensorarraySerialIoGetStats(&stats);
    assert(stats.lineDropped == droppedBefore + 1u);
    drain(); assert(registeredVfs.write(0, "RECOVERED\n", 10) == 10); drain();
    /* Replay the captured CONFIG record with concurrent real libc
     * fwrite calls on pre-/post-Init task stream aliases. Their VFS callbacks
     * must queue; neither logs nor a new CTRL may splice into CONFIG. */
    outputLength = 0; injectConcurrentLog = true;
    assert(sensorarraySerialIoQueue(capturedConfig, sizeof(capturedConfig) - 1, true) == ESP_OK);
    drain();
    assert(memcmp(output, capturedConfig, sizeof(capturedConfig) - 1) == 0);
    size_t offset = sizeof(capturedConfig) - 1;
    assert(memcmp(output + offset, "CTRL,concurrent=1\n", 18) == 0); offset += 18;
    assert(memcmp(output + offset, capturedPr, sizeof(capturedPr) - 1) == 0); offset += sizeof(capturedPr) - 1;
    assert(memcmp(output + offset, "STDERR,concurrent=1\n", 20) == 0); offset += 20;
    assert(outputLength == offset);
    injectConcurrentLog = false;
    /* Actual 3fb2a19 128-byte prefix: priority arrives before the later
     * synthetic suffix. No prefix may reach the wire until its LF arrives. */
    outputLength = 0;
    assert(fwrite(capturedAdsPrefix, 1, sizeof(capturedAdsPrefix) - 1, test_stdout) == sizeof(capturedAdsPrefix) - 1);
    drain(); assert(outputLength == 0);
    assert(sensorarraySerialIoQueue(capturedUsbReply, sizeof(capturedUsbReply) - 1, true) == ESP_OK);
    drain(); assert(outputLength == sizeof(capturedUsbReply) - 1);
    assert(fwrite(",synthetic_suffix=1\n", 1, 20, test_stdout) == 20);
    drain();
    assert(memcmp(output, capturedUsbReply, sizeof(capturedUsbReply) - 1) == 0);
    assert(memcmp(output + sizeof(capturedUsbReply) - 1, capturedAdsPrefix, sizeof(capturedAdsPrefix) - 1) == 0);
    assert(memcmp(output + sizeof(capturedUsbReply) + sizeof(capturedAdsPrefix) - 2, ",synthetic_suffix=1\n", 20) == 0);
    /* A DATA flood cannot steal an in-progress diagnostic line. The complete
     * line remains one record while priority and science keep their own pools. */
    outputLength = 0;
    sensorarraySerialIoGetStats(&stats); droppedBefore = stats.lineDropped;
    uint64_t discardedBefore = stats.discardBytes;
    assert(registeredVfs.write(0, capturedAdsPrefix, sizeof(capturedAdsPrefix) - 1) == sizeof(capturedAdsPrefix) - 1);
    for (unsigned i = 0; i < 4; ++i) { assert(sensorarraySerialIoQueueData("FILL\n", 5) == ESP_OK); }
    assert(registeredVfs.write(0, ",synthetic_suffix=1\n", 20) == 20);
    assert(sensorarraySerialIoQueue(capturedUsbReply, sizeof(capturedUsbReply) - 1, true) == ESP_OK);
    drain();
    assert(memcmp(output, capturedUsbReply, sizeof(capturedUsbReply) - 1) == 0);
    assert(outputLength == sizeof(capturedUsbReply) - 1 + 20 + sizeof(capturedAdsPrefix) - 1 + 20);
    assert(memmem(output, outputLength, capturedAdsPrefix, sizeof(capturedAdsPrefix) - 1));
    sensorarraySerialIoGetStats(&stats);
    assert(stats.lineDropped == droppedBefore && stats.discardBytes == discardedBefore);
    /* Two incomplete FILE streams plus three queued LOG records use all five
     * LOG slots; DATA and priority still have their full independent capacity. */
    outputLength = 0;
    assert(stdoutRoute.localFd == 0 && stderrRoute.localFd == 1);
    assert(fwrite("OUT,a=", 1, 6, test_stdout) == 6);
    assert(fwrite("ERR,b=", 1, 6, test_stderr) == 6);
    for (unsigned i = 0; i < 3; ++i) { assert(sensorarraySerialIoQueue("LOG\n", 4, false) == ESP_OK); }
    assert(sensorarraySerialIoQueue("FULL_LOG\n", 9, false) == ESP_ERR_TIMEOUT);
    for (unsigned i = 0; i < 4; ++i) { assert(sensorarraySerialIoQueueData("D\n", 2) == ESP_OK); }
    assert(sensorarraySerialIoQueueData("FULL\n", 5) == ESP_ERR_TIMEOUT);
    assert(sensorarraySerialIoQueue("CTRL\n", 5, true) == ESP_OK);
    drain(); assert(outputLength == 25 && memcmp(output, "CTRL\nD\nD\nD\nD\nLOG\nLOG\nLOG\n", 25) == 0);
    assert(fwrite("2\n", 1, 2, test_stderr) == 2);
    assert(fwrite("1\n", 1, 2, test_stdout) == 2);
    drain(); assert(outputLength == 41 && memcmp(output + 25, "ERR,b=2\nOUT,a=1\n", 16) == 0);
    /* Oversize is one discarded record through its LF, even across writes;
     * the following line recovers. The exact-capacity line remains legal. */
    outputLength = 0;
    char longLine[SENSORARRAY_TRANSPORT_TEXT_MAX]; memset(longLine, 'L', sizeof(longLine));
    sensorarraySerialIoGetStats(&stats); droppedBefore = stats.lineDropped;
    uint32_t oversizeBefore = stats.lineOversize; discardedBefore = stats.discardBytes;
    assert(registeredVfs.write(0, longLine, sizeof(longLine)) == sizeof(longLine));
    assert(registeredVfs.write(0, "overflow", 8) == 8);
    assert(sensorarraySerialIoQueue("CTRL\n", 5, true) == ESP_OK);
    assert(registeredVfs.write(0, "tail\nNEXT\n", 10) == 10);
    drain(); assert(outputLength == 10 && memcmp(output, "CTRL\nNEXT\n", 10) == 0);
    sensorarraySerialIoGetStats(&stats);
    assert(stats.lineDropped == droppedBefore + 1 && stats.lineOversize == oversizeBefore + 1);
    assert(stats.discardBytes == discardedBefore + sizeof(longLine) + 13);
    outputLength = 0; longLine[sizeof(longLine) - 1] = '\n';
    assert(registeredVfs.write(0, longLine, sizeof(longLine)) == sizeof(longLine));
    drain(); assert(outputLength == sizeof(longLine) && memcmp(output, longLine, sizeof(longLine)) == 0);
    /* No slot at the first fragment: keep dropping until LF even when slots
     * become free, so a suffix cannot be promoted into a fabricated record. */
    outputLength = 0;
    assert(registeredVfs.write(1, "ERR,holding=", 12) == 12);
    for (unsigned i = 0; i < SERIAL_IO_LOG_QUEUE_COUNT; ++i) { assert(sensorarraySerialIoQueue("LOG\n", 4, false) == ESP_OK); }
    assert(registeredVfs.write(0, "LOST,prefix=", 12) == 12);
    drain(); outputLength = 0;
    assert(registeredVfs.write(0, "suffix\nRECOVERED\n", 17) == 17);
    assert(registeredVfs.write(1, "done\n", 5) == 5);
    drain();
    assert(outputLength == 27 && memcmp(output, "RECOVERED\nERR,holding=done\n", 27) == 0);
    /* A malformed direct packet also cannot concatenate the next control. */
    outputLength = 0;
    assert(sensorarraySerialIoQueue("BROKEN", 6, true) == ESP_OK);
    assert(sensorarraySerialIoQueue("CTRL\n", 5, true) == ESP_OK);
    drain(); assert(outputLength == 12 && memcmp(output, "BROKEN\nCTRL\n", 12) == 0);
    /* Real concurrent FILE writers, with a priority send between both streams'
     * prefix and suffix. Pool/queue/metrics use actual pthread mutexes in this
     * shim, and no writer waits for a driver operation. */
    pthread_t writers[2];
    fragmentWriter_t writerArgs[2] = {{test_stdout, "OUT"}, {test_stderr, "ERR"}};
    assert(pthread_barrier_init(&fragmentBarrier, NULL, 3) == 0);
    for (unsigned i = 0; i < 2; ++i) { assert(pthread_create(&writers[i], NULL, concurrentFragments, &writerArgs[i]) == 0); }
    for (unsigned i = 0; i < 32; ++i) {
        outputLength = 0;
        (void)pthread_barrier_wait(&fragmentBarrier);
        assert(sensorarraySerialIoQueue("CTRL\n", 5, true) == ESP_OK);
        drain(); assert(outputLength == 5 && memcmp(output, "CTRL\n", 5) == 0);
        (void)pthread_barrier_wait(&fragmentBarrier);
        (void)pthread_barrier_wait(&fragmentBarrier);
        drain();
        char outRecord[48], errRecord[48];
        int outLength = snprintf(outRecord, sizeof(outRecord), "OUT,id=%u,done=1\n", i);
        int errLength = snprintf(errRecord, sizeof(errRecord), "ERR,id=%u,done=1\n", i);
        assert(outputLength == 5u + (size_t)outLength + (size_t)errLength);
        assert(memmem(output + 5, outputLength - 5, outRecord, (size_t)outLength));
        assert(memmem(output + 5, outputLength - 5, errRecord, (size_t)errLength));
        (void)pthread_barrier_wait(&fragmentBarrier);
    }
    for (unsigned i = 0; i < 2; ++i) { assert(pthread_join(writers[i], NULL) == 0); }
    assert(pthread_barrier_destroy(&fragmentBarrier) == 0);
    /* Legacy scientific output must keep the multi-line frame + CRC together
     * and propagate actual queue failure, bypassing diagnostic consumption. */
    outputLength = 0;
    sensorarrayFrame_t frame = {0};
    assert(sensorarrayFrameOutputPrint(NULL) == ESP_ERR_INVALID_ARG);
    serialSinkEnabled = false;
    assert(sensorarrayFrameOutputPrint(&frame) == ESP_OK && encoderCalls == 0);
    serialSinkEnabled = true;
    encoderResult = ESP_ERR_INVALID_RESPONSE;
    assert(sensorarrayFrameOutputPrint(&frame) == ESP_ERR_INVALID_RESPONSE);
    encoderResult = ESP_OK;
    assert(sensorarrayFrameOutputPrint(&frame) == ESP_OK && s_queues[0]->count == 1);
    assert(s_lengths[s_queues[0]->data[s_queues[0]->read % s_queues[0]->length]] == sizeof(encodedFrame) - 1);
    drain(); assert(outputLength == sizeof(encodedFrame) - 1 && memcmp(output, encodedFrame, outputLength) == 0);
    for (unsigned i = 0; i < 4; ++i) { assert(sensorarraySerialIoQueueData("FILL\n", 5) == ESP_OK); }
    assert(sensorarrayFrameOutputPrint(&frame) == ESP_ERR_TIMEOUT);
    drain();

    /* The actual UsbSink facade must distinguish scientific frames from both
     * compact diagnostic packets without changing its legacy total counters. */
    outputLength = 0;
    sensorarrayTextPacket_t facadePacket = {.length = sizeof(encodedFrame) - 1};
    memcpy(facadePacket.data, encodedFrame, facadePacket.length);
    for (unsigned i = 0; i < SERIAL_IO_LOG_QUEUE_COUNT; ++i) { assert(sensorarrayUsbSinkPublishDiagnostic(&facadePacket) == ESP_OK); }
    assert(sensorarrayUsbSinkPublishDiagnostic(&facadePacket) == ESP_ERR_TIMEOUT);
    assert(sensorarrayUsbSinkPublish(&facadePacket) == ESP_OK);
    assert(s_queues[SERIAL_IO_DATA]->count == 1 && s_queues[SERIAL_IO_LOG]->count == SERIAL_IO_LOG_QUEUE_COUNT);
    assert(s_usbStats.sentPackets == SERIAL_IO_LOG_QUEUE_COUNT + 1 && s_usbStats.droppedPackets == 1);
    drain(); assert(outputLength == (SERIAL_IO_LOG_QUEUE_COUNT + 1) * facadePacket.length);

    /* More LOG producers than capacity cannot consume any DATA/CTRL slots.
     * Whole-line diagnostic refusal and direct record refusal each count once. */
    outputLength = 0; resetClock();
    sensorarraySerialIoStats_t admissionBefore, admissionAfter;
    sensorarraySerialIoGetStats(&admissionBefore);
    for (unsigned i = 0; i < SERIAL_IO_LOG_QUEUE_COUNT; ++i) {
        assert(sensorarraySerialIoQueue("LOG\n", 4, false) == ESP_OK);
    }
    for (unsigned i = 0; i < 16; ++i) { assert(sensorarraySerialIoQueue("DROP_LOG\n", 9, false) == ESP_ERR_TIMEOUT); }
    assert(registeredVfs.write(0, "DROPPED,part=", 13) == 13);
    assert(registeredVfs.write(0, "suffix\n", 7) == 7);
    for (unsigned i = 0; i < SERIAL_IO_QUEUE_COUNT; ++i) {
        assert(sensorarraySerialIoQueueData("DATA\n", 5) == ESP_OK);
        assert(sensorarraySerialIoQueue("CTRL\n", 5, true) == ESP_OK);
    }
    drain();
    assert(outputLength == 56 && memcmp(output, "CTRL\nCTRL\nCTRL\nCTRL\nDATA\nDATA\nDATA\nDATA\nLOG\nLOG\nLOG\nLOG\n", 56) == 0);
    sensorarraySerialIoGetStats(&admissionAfter);
    assert(admissionAfter.logRejected == admissionBefore.logRejected + 17);
    assert(admissionAfter.dataRejected == admissionBefore.dataRejected);
    assert(admissionAfter.priorityRejected == admissionBefore.priorityRejected);
    assert(admissionAfter.dataSubmitted == admissionBefore.dataSubmitted + 4);
    assert(admissionAfter.logSubmitted == admissionBefore.logSubmitted + SERIAL_IO_LOG_QUEUE_COUNT);
    assert(admissionAfter.normalSubmitted == admissionBefore.normalSubmitted + SERIAL_IO_LOG_QUEUE_COUNT + 4);

    /* The seven real-format fault lines retain the baseline's first four
     * root-cause/rail/context/register records. Refused transaction tails are
     * whole counted lines, not silently spliced or reported as submitted. */
    outputLength = 0; resetClock(); sensorarraySerialIoGetStats(&admissionBefore);
    for (const char *start = faultBurst; *start;) {
        const char *end = strchr(start, '\n'); assert(end);
        size_t length = (size_t)(end - start) + 1;
        assert(registeredVfs.write(0, start, length) == (ssize_t)length);
        start = end + 1;
    }
    assert(sensorarraySerialIoQueueData("DATA\n", 5) == ESP_OK);
    assert(sensorarraySerialIoQueue("CTRL\n", 5, true) == ESP_OK);
    drain(); sensorarraySerialIoGetStats(&admissionAfter);
    assert(outputLength == 10 + FAULT_FIRST_FOUR_BYTES);
    assert(memcmp(output, "CTRL\nDATA\n", 10) == 0);
    assert(memcmp(output + 10, faultBurst, FAULT_FIRST_FOUR_BYTES) == 0);
    assert(admissionAfter.logRejected == admissionBefore.logRejected + 3);
    assert(admissionAfter.lineDropped == admissionBefore.lineDropped + 3);
    assert(admissionAfter.sinkDropped == admissionBefore.sinkDropped + 3);
    assert(admissionAfter.logSubmitted == admissionBefore.logSubmitted + 4);
    assert(admissionAfter.dataRejected == admissionBefore.dataRejected);
    assert(admissionAfter.partialWrite == admissionBefore.partialWrite && admissionAfter.timeout == admissionBefore.timeout);

    /* A not-yet-started DATA record does not sit in front of a newly arrived
     * critical reply while waiting for host capacity. */
    outputLength = 0; resetClock(); scenario = 3; writeBudget = 64; releaseAfterUs = 30000;
    injectPriorityOnBlocked = true; injectedPriority = false;
    assert(sensorarraySerialIoQueueData(encodedFrame, sizeof(encodedFrame) - 1) == ESP_OK);
    drain();
    assert(outputLength == 15 + sizeof(encodedFrame) - 1);
    assert(memcmp(output, "CTRL,blocked=1\n", 15) == 0);
    assert(memcmp(output + 15, encodedFrame, sizeof(encodedFrame) - 1) == 0);
    injectPriorityOnBlocked = false;

    /* Positive short writes before the deadline resume the same packet and
     * cannot admit a priority record into its suffix. */
    outputLength = 0; resetClock(); scenario = 2; writeBudget = 80; releaseAfterUs = 30000;
    injectPriorityAfterShort = true; injectedPriority = false;
    packet[sizeof(packet) - 1] = '\n';
    sensorarraySerialIoGetStats(&stats);
    uint32_t priorAborts = stats.partialWrite, priorSubmitted = stats.normalSubmitted;
    assert(sensorarraySerialIoQueueData(packet, sizeof(packet)) == ESP_OK);
    drain();
    assert(outputLength == sizeof(packet) + 6 && memcmp(output, packet, sizeof(packet)) == 0);
    assert(memcmp(output + sizeof(packet), "AFTER\n", 6) == 0);
    sensorarraySerialIoGetStats(&stats);
    assert(stats.partialWrite == priorAborts && stats.normalSubmitted == priorSubmitted + 1);
    injectPriorityAfterShort = false;

    /* A diagnostic short prefix also owns the wire: neither a newly arriving
     * DATA packet nor priority reply may be inserted into its suffix. */
    outputLength = 0; resetClock(); scenario = 2; writeBudget = 80; releaseAfterUs = 30000;
    injectPriorityAfterShort = true; injectedPriority = false; injectDataAfterShort = true;
    sensorarraySerialIoGetStats(&admissionBefore);
    assert(sensorarraySerialIoQueue(packet, sizeof(packet), false) == ESP_OK);
    drain();
    assert(outputLength == sizeof(packet) + 17 && memcmp(output, packet, sizeof(packet)) == 0);
    assert(memcmp(output + sizeof(packet), "AFTER\nDATA_AFTER\n", 17) == 0);
    sensorarraySerialIoGetStats(&admissionAfter);
    assert(admissionAfter.logSubmitted == admissionBefore.logSubmitted + 1);
    assert(admissionAfter.dataSubmitted == admissionBefore.dataSubmitted + 1);
    assert(admissionAfter.prioritySubmitted == admissionBefore.prioritySubmitted + 1);
    assert(admissionAfter.partialWrite == admissionBefore.partialWrite);
    injectPriorityAfterShort = false; injectDataAfterShort = false;

    /* Queue residence is part of the deadline. Expired whole packets never
     * enter the driver, whereas retained priority survives a slower host. */
    outputLength = 0; resetClock(); scenario = 0; writeBudget = -1;
    assert(sensorarraySerialIoQueueData("EXPIRED\n", 8) == ESP_OK);
    assert(sensorarraySerialIoQueue("PRIORITY\n", 9, true) == ESP_OK);
    nowUs = SERIAL_IO_NORMAL_LIFETIME_US + 1;
    unsigned callsBefore = driverCalls;
    drain();
    assert(driverCalls == callsBefore + 1 && outputLength == 9 && memcmp(output, "PRIORITY\n", 9) == 0);
    outputLength = 0;
    assert(sensorarraySerialIoQueue("STALE_CTRL\n", 11, true) == ESP_OK);
    nowUs += SERIAL_IO_PRIORITY_LIFETIME_US;
    callsBefore = driverCalls;
    drain(); assert(outputLength == 0 && driverCalls == callsBefore);
    sensorarraySerialIoGetStats(&stats); assert(stats.priorityExpired == 1);

    /* No reader: priority is retained for its finite lifetime. Both lanes
     * release slots, and no zero-wait ready loop can burn the entire interval. */
    resetClock(); scenario = 1; writeBudget = 0;
    unsigned delaysBefore = delayCalls;
    assert(sensorarraySerialIoQueueData("NOHOST_DATA\n", 12) == ESP_OK);
    assert(sensorarraySerialIoQueue("NOHOST_CTRL\n", 12, true) == ESP_OK);
    drain(); assert(nowUs == SERIAL_IO_PRIORITY_LIFETIME_US && delayCalls > delaysBefore);
    for (unsigned i = 0; i < SERIAL_IO_SLOT_COUNT; ++i) { assert(!s_inUse[i]); }

    /* With DATA continuously replenished, LOG still gets a whole-record
     * service opportunity after at most four DATA completions. */
    resetClock(); outputLength = 0; scenario = 0; writeBudget = -1;
    memset(refillPacket, 'B', sizeof(refillPacket)); refillPacket[sizeof(refillPacket) - 1] = '\n';
    for (unsigned i = 0; i < SERIAL_IO_QUEUE_COUNT; ++i) { assert(sensorarraySerialIoQueueData(refillPacket, sizeof(refillPacket)) == ESP_OK); }
    assert(sensorarraySerialIoQueue("LOG_ONE\n", 8, false) == ESP_OK);
    assert(sensorarraySerialIoQueue("LOG_TWO\n", 8, false) == ESP_OK);
    refillNormalRemaining = 36;
    drain();
    assert(refillNormalRemaining == 0 && outputLength == 40 * sizeof(refillPacket) + 16);
    const char *logOne = memmem(output, outputLength, "LOG_ONE\n", 8);
    const char *logTwo = memmem(output, outputLength, "LOG_TWO\n", 8);
    assert(logOne && logTwo && logTwo > logOne);
    assert((size_t)(logOne - output) <= SERIAL_IO_DATA_BURST_MAX * sizeof(refillPacket));
    assert((size_t)(logTwo - logOne - 8) <= SERIAL_IO_DATA_BURST_MAX * sizeof(refillPacket));
    for (unsigned i = 0; i < SERIAL_IO_SLOT_COUNT; ++i) { assert(!s_inUse[i]); }
    sensorarraySerialIoGetStats(&stats); assert(stats.priorityExpired == 2);

    /* A prefix cancelled while nobody can read is still remembered when the
     * owner later sleeps for work. The next wake starts at a line boundary. */
    outputLength = 0; resetClock(); scenario = 2; writeBudget = 80;
    releaseAfterUs = INT64_MAX; injectControlOnIdle = true;
    assert(sensorarraySerialIoQueueData(packet, sizeof(packet)) == ESP_OK);
    drain();
    assert(!injectControlOnIdle && outputLength == 80 + 1 + 15);
    assert(memcmp(output + 80, "\nIDLE_RECOVERED\n", 16) == 0);
    for (unsigned i = 0; i < SERIAL_IO_SLOT_COUNT; ++i) { assert(!s_inUse[i]); }

    /* Less than two ticks remaining cannot authorize even a single tick for
     * each of the driver's mutex and ring waits. No partial acceptance occurs. */
    outputLength = 0; resetClock(); scenario = 1; writeBudget = 0;
    assert(sensorarraySerialIoQueue("NEAR_DEADLINE\n", 14, true) == ESP_OK);
    nowUs = SERIAL_IO_PRIORITY_LIFETIME_US - SERIAL_IO_TICK_US;
    callsBefore = driverCalls;
    drain();
    assert(nowUs == SERIAL_IO_PRIORITY_LIFETIME_US && driverCalls >= callsBefore + 1 && driverCalls <= callsBefore + 32);
    assert(lastDriverWaitTicks == 0 && outputLength == 0);

    /* Invalid driver results never become record-submitted counts. A later
     * control record is separated even when the failure's byte count is unknown. */
    outputLength = 0; resetClock(); scenario = 4; writeBudget = -1;
    sensorarraySerialIoGetStats(&stats); priorSubmitted = stats.prioritySubmitted;
    assert(sensorarraySerialIoQueue("UNKNOWN\n", 8, true) == ESP_OK);
    assert(sensorarraySerialIoQueue("AFTER\n", 6, true) == ESP_OK);
    drain();
    assert(outputLength == 7 && memcmp(output, "\nAFTER\n", 7) == 0);
    sensorarraySerialIoGetStats(&stats);
    assert(stats.driverWriteError == 1 && stats.prioritySubmitted == priorSubmitted + 1);

    /* Priority capacity and expiry are distinct, observable failure layers. */
    uint32_t rejectedBefore = stats.priorityRejected;
    for (unsigned i = 0; i < 4; ++i) { assert(sensorarraySerialIoQueue("CTRL\n", 5, true) == ESP_OK); }
    assert(sensorarraySerialIoQueue("REJECTED\n", 9, true) == ESP_ERR_TIMEOUT);
    sensorarraySerialIoGetStats(&stats); assert(stats.priorityRejected == rejectedBefore + 1);
    drain();

    /* A continually writable host plus a refilling producer must also yield.
     * Every 2304-byte packet is admitted in one driver call, including maximum
     * BYTEBUF item size. No hot-path allocations or per-frame FPS cap. */
    resetClock(); outputLength = 0; scenario = 0; writeBudget = -1;
    memset(refillPacket, 'B', sizeof(refillPacket)); refillPacket[sizeof(refillPacket) - 1] = '\n';
    for (unsigned i = 0; i < 4; ++i) { assert(sensorarraySerialIoQueueData(refillPacket, sizeof(refillPacket)) == ESP_OK); }
    refillNormalRemaining = 36;
    delaysBefore = delayCalls; callsBefore = driverCalls;
    drain();
    assert(refillNormalRemaining == 0 && outputLength == 40 * sizeof(refillPacket));
    assert(driverCalls == callsBefore + 40 && delayCalls >= delaysBefore + 1);
    for (unsigned i = 0; i < 40; ++i) { assert(memcmp(output + i * sizeof(refillPacket), refillPacket, sizeof(refillPacket)) == 0); }
    for (unsigned i = 0; i < SERIAL_IO_SLOT_COUNT; ++i) { assert(!s_inUse[i]); }

    /* No CPU0 idle observation may return while an SDK task is busy. Shared
     * checkpoints still return after one tick, allowing the original packet
     * deadlines to expire; they never trap an accepted record waiting for idle. */
    resetClock(); outputLength = 0; scenario = 1; writeBudget = 0;
    sensorarrayWorkBudgetSnapshot_t budgetBefore, budgetAfter;
    sensorarrayWorkBudgetCopySnapshot(&budgetBefore);
    observeIdle = false;
    assert(sensorarraySerialIoQueueData("NO_IDLE_DATA\n", 13) == ESP_OK);
    assert(sensorarraySerialIoQueue("NO_IDLE_CTRL\n", 13, true) == ESP_OK);
    drain();
    sensorarrayWorkBudgetCopySnapshot(&budgetAfter);
    assert(budgetAfter.idleGeneration == budgetBefore.idleGeneration);
    assert(nowUs >= SERIAL_IO_PRIORITY_LIFETIME_US && (uint64_t)nowUs <= SERIAL_IO_PRIORITY_LIFETIME_US + SERIAL_IO_TICK_US);
    assert(lastDriverAttemptUs < SERIAL_IO_PRIORITY_LIFETIME_US && outputLength == 0);
    for (unsigned i = 0; i < SERIAL_IO_SLOT_COUNT; ++i) { assert(!s_inUse[i]); }
    observeIdle = true;

    /* A zero-returning driver that does not actually block is also bounded by
     * real RTOS delays, rather than an always-ready retry loop. */
    resetClock(); scenario = 5; writeBudget = 0;
    callsBefore = driverCalls; delaysBefore = delayCalls;
    assert(sensorarraySerialIoQueue("BROKEN_DRIVER\n", 14, true) == ESP_OK);
    drain();
    assert(nowUs == SERIAL_IO_PRIORITY_LIFETIME_US && delayCalls >= delaysBefore + 100);
    assert(driverCalls <= callsBefore + 3200 && lastDriverAttemptUs < SERIAL_IO_PRIORITY_LIFETIME_US);
    for (unsigned i = 0; i < SERIAL_IO_SLOT_COUNT; ++i) { assert(!s_inUse[i]); }

    /* Fill all three actual queue capacities before the single wake. No producer
     * or notification occurs after that: expiry must drain every retained
     * and queued slot, rather than waiting forever with stale wake counts. */
    sensorarraySerialIoStats_t burstBefore, burstAfter;
    unsigned notificationsBeforeDrain;
    for (unsigned lanes = 1; lanes <= 7; ++lanes) {
        resetClock(); outputLength = 0; scenario = 1; writeBudget = 0;
        unsigned normalCount = lanes & 1u ? SERIAL_IO_QUEUE_COUNT : 0u;
        unsigned priorityCount = lanes & 2u ? SERIAL_IO_QUEUE_COUNT : 0u;
        unsigned logCount = lanes & 4u ? SERIAL_IO_LOG_QUEUE_COUNT : 0u;
        sensorarraySerialIoGetStats(&burstBefore);
        for (unsigned i = 0; i < SERIAL_IO_QUEUE_COUNT; ++i) {
            if (normalCount) { assert(sensorarraySerialIoQueueData("EXPIRE_DATA\n", 12) == ESP_OK); }
            if (priorityCount) { assert(sensorarraySerialIoQueue("EXPIRE_CTRL\n", 12, true) == ESP_OK); }
        }
        for (unsigned i = 0; i < logCount; ++i) { assert(sensorarraySerialIoQueue("EXPIRE_LOG\n", 11, false) == ESP_OK); }
        assert(s_queues[0]->count == normalCount && s_queues[1]->count == priorityCount && s_queues[2]->count == logCount);
        notificationsBeforeDrain = notificationsGiven;
        drain();
        sensorarraySerialIoGetStats(&burstAfter);
        unsigned retained = 0;
        for (unsigned i = 0; i < SERIAL_IO_SLOT_COUNT; ++i) { retained += s_inUse[i] ? 1u : 0u; }
        if (s_queues[0]->count || s_queues[1]->count || s_queues[2]->count || retained) {
            fprintf(stderr, "SERIAL_EXPIRY_STRANDED,lanes=%u,nowUs=%lld,normalQueued=%u,priorityQueued=%u,retained=%u,normalExpiredDelta=%u,priorityExpiredDelta=%u,notifyDelta=%u\n",
                    lanes, (long long)nowUs, s_queues[0]->count, s_queues[1]->count, retained,
                    burstAfter.normalExpired - burstBefore.normalExpired,
                    burstAfter.priorityExpired - burstBefore.priorityExpired,
                    notificationsGiven - notificationsBeforeDrain);
        }
        assert(s_queues[0]->count == 0 && s_queues[1]->count == 0 && s_queues[2]->count == 0 && retained == 0);
        assert(notificationsGiven == notificationsBeforeDrain && notificationsPending == 0);
        assert(burstAfter.normalExpired == burstBefore.normalExpired + normalCount + logCount);
        assert(burstAfter.priorityExpired == burstBefore.priorityExpired + priorityCount);
        assert(burstAfter.timeout == burstBefore.timeout + normalCount + priorityCount + logCount);
        assert(burstAfter.sinkDropped == burstBefore.sinkDropped + normalCount + priorityCount + logCount);
        assert(burstAfter.partialWrite == burstBefore.partialWrite && outputLength == 0);
        uint64_t maximumLifetime = priorityCount ? SERIAL_IO_PRIORITY_LIFETIME_US : SERIAL_IO_NORMAL_LIFETIME_US;
        assert((uint64_t)nowUs <= maximumLifetime + SERIAL_IO_TICK_US);
    }

    /* An expired partial priority record must advance to a younger queued
     * record in the same lane, without a future producer waking the task. */
    resetClock(); outputLength = 0; scenario = 2; writeBudget = 80;
    releaseAfterUs = SERIAL_IO_PRIORITY_LIFETIME_US;
    sensorarraySerialIoGetStats(&burstBefore);
    assert(sensorarraySerialIoQueue(packet, sizeof(packet), true) == ESP_OK);
    nowUs = SERIAL_IO_PRIORITY_LIFETIME_US / 2;
    assert(sensorarraySerialIoQueue("NEXT_PRIORITY\n", 14, true) == ESP_OK);
    notificationsBeforeDrain = notificationsGiven;
    drain();
    sensorarraySerialIoGetStats(&burstAfter);
    assert(notificationsGiven == notificationsBeforeDrain && notificationsPending == 0);
    assert(outputLength == 95 && memcmp(output + 80, "\nNEXT_PRIORITY\n", 15) == 0);
    assert(burstAfter.priorityExpired == burstBefore.priorityExpired + 1);
    assert(burstAfter.partialWrite == burstBefore.partialWrite + 1);
    assert(burstAfter.prioritySubmitted == burstBefore.prioritySubmitted + 1);
    assert(s_queues[0]->count == 0 && s_queues[1]->count == 0);
    for (unsigned i = 0; i < SERIAL_IO_SLOT_COUNT; ++i) { assert(!s_inUse[i]); }
    assert(allocationCalls == allocationsAfterInit); /* no hot path allocations */
#ifdef SERIAL_IO_HAS_OTG_DISPATCH
    /* Current wrapper selects USJ; the new OTG epoch API must not fabricate a
     * bus generation for this retained legacy backend. */
    uint32_t legacyEpoch = UINT32_MAX;
    char legacyRx[64];
    assert(!sensorarrayUsbOtgEnabled());
    assert(sensorarraySerialIoReadWithEpoch(legacyRx, sizeof(legacyRx), &legacyEpoch) == 6);
    assert(legacyEpoch == 0u && memcmp(legacyRx, "MODE?\n", 6u) == 0);
    for (unsigned i = 0; i < SERIAL_IO_SLOT_COUNT; ++i) { assert(s_slotEpoch[i] == 0u); }
#endif
    printf("SERIAL_IO_PASS,allocation=%s,initOnce=1,noHotAlloc=1,priority=1,tickHz=100,noHostNormalUs=250000,noHostPriorityUs=1000000,rxIndependent=1,shortWriteResume=1,partialCancelBoundary=1,wholeLineDrop=1,twoStreams=1,concurrentIterations=32,oversizeResync=1,legacyFrameAtomic=1,usbFacadeClasses=1,dataLogIsolation=1,faultFirstFourPreserved=1,faultTailDrops=3,logSlots=5,logQueue=4,normalPayloadBytes=23040,logWeightedService=1,threeLanePrefix=1,threeLaneExpiryMasks=7,onePersistentAllocation=1,maxRecordAtomic=2304,queueAgeBound=1,unstartedDataPreempt=1,continuousProducerYields=1,sharedIdleAbsentBounded=1,brokenDriverBackoff=1,invalidDriverResult=1,noTxFileLock=1,expiryDrainsAllSlots=1,noFutureNotify=1,shortExpiryAdvancesSameLane=1,stdioBound=1,configBytes=%zu\n", allocation, sizeof(capturedConfig) - 1);
    return 0;
#endif
}

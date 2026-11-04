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
#include "idf_vfs_prefix.h"
#include "idf_reent_init.h"
#include "serial_capture.h"
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
    return calloc(count, size);
}

QueueHandle_t xQueueCreateStatic(unsigned count, unsigned size, uint8_t *bytes, StaticQueue_t *queue)
{
    assert(size == 1); *queue = (StaticQueue_t){.length=count, .data=bytes};
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
    if (injectConcurrentLog && !injectedConcurrentLog) {
        assert(length == 64); /* the observed physical insertion boundary */
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
#if !defined(SERIAL_IO_BASELINE) && !defined(SERIAL_IO_FRAGMENT_BASELINE)
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
    for (unsigned i = 0; i < 4; ++i) { assert(sensorarraySerialIoQueue("FILL\n", 5, false) == ESP_OK); }
    (void)fwrite(",synthetic_suffix=1\n", 1, 20, test_stdout);
    assert(sensorarraySerialIoQueue(capturedUsbReply, sizeof(capturedUsbReply) - 1, true) == ESP_OK);
    drain();
    assert(memcmp(output, capturedAdsPrefix, sizeof(capturedAdsPrefix) - 1) == 0);
    assert(memcmp(output + sizeof(capturedAdsPrefix) - 1, capturedUsbReply, sizeof(capturedUsbReply) - 1) == 0);
    sensorarraySerialIoGetStats(&after);
    assert(after.sinkDropped > before.sinkDropped && after.timeout == before.timeout && after.partialWrite == before.partialWrite);
    printf("SERIAL_IO_FRAGMENT_BASELINE_REPRODUCED,stdioBound=1,initError=0,prefixBytes=128,queueDrop=1,partialWriteDelta=0,timeoutDelta=0\n");
    return 0;
#else
    const char *allocation = argc > 1 ? argv[1] : "psram";
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
    assert(stats.stdioBound && stats.initError == ESP_OK);
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
    /* VFS consumes complete lines, but a full queue is a counted whole-line
     * drop. Its return value is not protocol/DATA queue admission. */
    scenario = 0; writeBudget = -1;
    uint32_t droppedBefore = stats.lineDropped;
    for (unsigned i = 0; i < 4; ++i) { assert(registeredVfs.write(0, "LOG\n", 4) == 4); }
    assert(registeredVfs.write(0, "LOG\n", 4) == 4);
    sensorarraySerialIoGetStats(&stats);
    assert(stats.lineDropped == droppedBefore + 1u);
    drain(); assert(registeredVfs.write(0, "RECOVERED\n", 10) == 10); drain();
    /* Reproduce the captured 64-byte scheduling boundary with real libc
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
    /* A queue that fills between printf fragments drops the complete line,
     * never a published prefix. A priority ACK remains a separate record. */
    outputLength = 0;
    sensorarraySerialIoGetStats(&stats); droppedBefore = stats.lineDropped;
    uint64_t discardedBefore = stats.discardBytes;
    assert(registeredVfs.write(0, capturedAdsPrefix, sizeof(capturedAdsPrefix) - 1) == sizeof(capturedAdsPrefix) - 1);
    for (unsigned i = 0; i < 4; ++i) { assert(sensorarraySerialIoQueue("FILL\n", 5, false) == ESP_OK); }
    assert(registeredVfs.write(0, ",synthetic_suffix=1\n", 20) == 20);
    assert(sensorarraySerialIoQueue(capturedUsbReply, sizeof(capturedUsbReply) - 1, true) == ESP_OK);
    drain();
    assert(memcmp(output, capturedUsbReply, sizeof(capturedUsbReply) - 1) == 0);
    assert(outputLength == sizeof(capturedUsbReply) - 1 + 20);
    assert(memmem(output, outputLength, "ADSFRAME", 8) == NULL);
    sensorarraySerialIoGetStats(&stats);
    assert(stats.lineDropped == droppedBefore + 1 && stats.discardBytes == discardedBefore + sizeof(capturedAdsPrefix) - 1 + 20);
    /* Two FILE streams can be interleaved independently, without new buffers.
     * Both unfinished records still leave three normal slots available. */
    outputLength = 0;
    assert(stdoutRoute.localFd == 0 && stderrRoute.localFd == 1);
    assert(fwrite("OUT,a=", 1, 6, test_stdout) == 6);
    assert(fwrite("ERR,b=", 1, 6, test_stderr) == 6);
    for (unsigned i = 0; i < 3; ++i) { assert(sensorarraySerialIoQueue("D\n", 2, false) == ESP_OK); }
    assert(sensorarraySerialIoQueue("FULL\n", 5, false) == ESP_ERR_TIMEOUT);
    assert(sensorarraySerialIoQueue("CTRL\n", 5, true) == ESP_OK);
    drain(); assert(outputLength == 11 && memcmp(output, "CTRL\nD\nD\nD\n", 11) == 0);
    assert(fwrite("2\n", 1, 2, test_stderr) == 2);
    assert(fwrite("1\n", 1, 2, test_stdout) == 2);
    drain(); assert(outputLength == 27 && memcmp(output + 11, "ERR,b=2\nOUT,a=1\n", 16) == 0);
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
    for (unsigned i = 0; i < 4; ++i) { assert(sensorarraySerialIoQueue("D\n", 2, false) == ESP_OK); }
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
    for (unsigned i = 0; i < 4; ++i) { assert(sensorarraySerialIoQueue("FILL\n", 5, false) == ESP_OK); }
    assert(sensorarrayFrameOutputPrint(&frame) == ESP_ERR_TIMEOUT);
    drain();
    assert(allocationCalls == allocationsAfterInit); /* no hot path allocations */
    printf("SERIAL_IO_PASS,allocation=%s,initOnce=1,noHotAlloc=1,priority=1,noHostBoundUs=20000,rxIndependent=1,partialRecovery=1,wholeLineDrop=1,twoStreams=1,concurrentIterations=32,oversizeResync=1,legacyFrameAtomic=1,stdioBound=1,configBytes=%zu,concurrentAtByte=64\n", allocation, sizeof(capturedConfig) - 1);
    return 0;
#endif
}

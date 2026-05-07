#include "sensorarrayVoltageStream.h"

#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "sdkconfig.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/task.h"

#include "sensorarrayPerf.h"

#if CONFIG_SENSORARRAY_USB_STDOUT_NONBLOCKING && __has_include("driver/usb_serial_jtag.h")
#include "driver/usb_serial_jtag.h"
#define SENSORARRAY_HAVE_USB_SERIAL_JTAG_DRIVER 1
#else
#define SENSORARRAY_HAVE_USB_SERIAL_JTAG_DRIVER 0
#endif

#if CONFIG_SENSORARRAY_USB_STDOUT_NONBLOCKING && __has_include("driver/usb_serial_jtag_vfs.h")
#include "driver/usb_serial_jtag_vfs.h"
#define SENSORARRAY_HAVE_USB_SERIAL_JTAG_VFS 1
#else
#define SENSORARRAY_HAVE_USB_SERIAL_JTAG_VFS 0
#endif

#ifndef CONFIG_SENSORARRAY_STREAM_QUEUE_DEPTH
#define CONFIG_SENSORARRAY_STREAM_QUEUE_DEPTH 4
#endif
#ifndef CONFIG_SENSORARRAY_SCAN_TASK_STACK
#define CONFIG_SENSORARRAY_SCAN_TASK_STACK 12288
#endif
#ifndef CONFIG_SENSORARRAY_COMM_TASK_STACK
#define CONFIG_SENSORARRAY_COMM_TASK_STACK 16384
#endif
#ifndef CONFIG_SENSORARRAY_SCAN_TASK_CORE
#define CONFIG_SENSORARRAY_SCAN_TASK_CORE 1
#endif
#ifndef CONFIG_SENSORARRAY_COMM_TASK_CORE
#define CONFIG_SENSORARRAY_COMM_TASK_CORE 0
#endif
#ifndef CONFIG_SENSORARRAY_SCAN_TASK_PRIORITY
#define CONFIG_SENSORARRAY_SCAN_TASK_PRIORITY 10
#endif
#ifndef CONFIG_SENSORARRAY_COMM_TASK_PRIORITY
#define CONFIG_SENSORARRAY_COMM_TASK_PRIORITY 5
#endif
#ifndef CONFIG_SENSORARRAY_STATUS_PERIOD_N_FRAMES
#define CONFIG_SENSORARRAY_STATUS_PERIOD_N_FRAMES 1000
#endif
#ifndef CONFIG_SENSORARRAY_BINARY_TEXT_STATUS
#define CONFIG_SENSORARRAY_BINARY_TEXT_STATUS 0
#endif
#ifndef CONFIG_SENSORARRAY_STACK_WATERMARK_PERIOD_N_FRAMES
#define CONFIG_SENSORARRAY_STACK_WATERMARK_PERIOD_N_FRAMES 100
#endif
#ifndef CONFIG_SENSORARRAY_OUTPUT_STACK_LOW_WORDS
#define CONFIG_SENSORARRAY_OUTPUT_STACK_LOW_WORDS 1024
#endif
#ifndef CONFIG_SENSORARRAY_OUTPUT_STACK_CRITICAL_WORDS
#define CONFIG_SENSORARRAY_OUTPUT_STACK_CRITICAL_WORDS 512
#endif
#ifndef CONFIG_SENSORARRAY_BINARY_WRITE_MAX_ATTEMPTS
#define CONFIG_SENSORARRAY_BINARY_WRITE_MAX_ATTEMPTS 8
#endif
#ifndef CONFIG_SENSORARRAY_VOLTAGE_SCAN_CSV_EVERY_N
#define CONFIG_SENSORARRAY_VOLTAGE_SCAN_CSV_EVERY_N 0
#endif
#ifndef CONFIG_SENSORARRAY_VOLTAGE_SCAN_DISCARD_FIRST
#define CONFIG_SENSORARRAY_VOLTAGE_SCAN_DISCARD_FIRST 0
#endif
#ifndef CONFIG_SENSORARRAY_ADS_DRDY_TIMEOUT_US_FAST
#define CONFIG_SENSORARRAY_ADS_DRDY_TIMEOUT_US_FAST 2000
#endif
#ifndef CONFIG_SENSORARRAY_ADS_FAST_INPMUX_WRITE
#define CONFIG_SENSORARRAY_ADS_FAST_INPMUX_WRITE 0
#endif
#ifndef CONFIG_SENSORARRAY_ADS_FAST_READ_DIRECT
#define CONFIG_SENSORARRAY_ADS_FAST_READ_DIRECT 0
#endif
#ifndef CONFIG_SENSORARRAY_ADS_FAST_DISABLE_STATUS_CRC
#define CONFIG_SENSORARRAY_ADS_FAST_DISABLE_STATUS_CRC 0
#endif
#ifndef CONFIG_SENSORARRAY_ADS_FAST_FIXED_GAIN
#define CONFIG_SENSORARRAY_ADS_FAST_FIXED_GAIN 0
#endif
#ifndef CONFIG_SENSORARRAY_ADS_SPI_POLLING_TRANSMIT
#define CONFIG_SENSORARRAY_ADS_SPI_POLLING_TRANSMIT 0
#endif
#ifndef CONFIG_SENSORARRAY_ADS_SPI_ACQUIRE_BUS_PER_FRAME
#define CONFIG_SENSORARRAY_ADS_SPI_ACQUIRE_BUS_PER_FRAME 0
#endif
#ifndef CONFIG_SENSORARRAY_AUTO_RATE_CONTROL
#define CONFIG_SENSORARRAY_AUTO_RATE_CONTROL 0
#endif
#ifndef CONFIG_SENSORARRAY_AUTO_RATE_MIN_ADS_DR
#define CONFIG_SENSORARRAY_AUTO_RATE_MIN_ADS_DR 12
#endif
#ifndef CONFIG_SENSORARRAY_AUTO_RATE_WINDOW_FRAMES
#define CONFIG_SENSORARRAY_AUTO_RATE_WINDOW_FRAMES 100
#endif
#ifndef CONFIG_SENSORARRAY_BINARY_STARTUP_MARKER
#define CONFIG_SENSORARRAY_BINARY_STARTUP_MARKER 0
#endif

#define SENSORARRAY_BINARY_WRITE_MAX_ATTEMPTS ((uint32_t)CONFIG_SENSORARRAY_BINARY_WRITE_MAX_ATTEMPTS)

typedef struct {
    sensorarrayVoltageStreamConfig_t config;
    QueueHandle_t queue;
    sensorarrayStatusCounters_t status;
    sensorarrayPerfCounters_t perf;
    sensorarrayRateController_t controller;
    sensorarrayVoltageStreamFrame_t *outputRxFrame;
    sensorarrayVoltageStreamFrame_t *scanTxFrame;
    sensorarrayVoltageStreamFrame_t *dropScratchFrame;
    sensorarrayVoltageCompactFrame_t *outputCompactFrame;
    TaskHandle_t scanTask;
    TaskHandle_t outputTask;
    UBaseType_t outputStackMinWords;
    UBaseType_t scanStackMinWords;
    uint32_t outputStackLowCount;
    uint32_t outputStackCriticalCount;
    bool running;
    bool usbNonblocking;
    bool headerPrinted;
    bool textStatusSuppressedByStack;
    bool textStatusSuppressedByBinaryMode;
    uint32_t usbWriteCount;
    int64_t streamStartUs;
} sensorarrayVoltageStreamContext_t;

static sensorarrayVoltageStreamContext_t s_stream = {0};
static sensorarrayVoltageStreamFrame_t s_dropScratchStatic;

static esp_err_t sensorarrayVoltageStreamAllocateBuffers(void)
{
    s_stream.outputRxFrame = heap_caps_calloc(1, sizeof(*s_stream.outputRxFrame), MALLOC_CAP_8BIT);
    s_stream.scanTxFrame = heap_caps_calloc(1, sizeof(*s_stream.scanTxFrame), MALLOC_CAP_8BIT);
    s_stream.dropScratchFrame = heap_caps_calloc(1, sizeof(*s_stream.dropScratchFrame), MALLOC_CAP_8BIT);
    s_stream.outputCompactFrame = heap_caps_calloc(1, sizeof(*s_stream.outputCompactFrame), MALLOC_CAP_8BIT);

    if (!s_stream.dropScratchFrame) {
        memset(&s_dropScratchStatic, 0, sizeof(s_dropScratchStatic));
        s_stream.dropScratchFrame = &s_dropScratchStatic;
    }

    if (!s_stream.outputRxFrame || !s_stream.scanTxFrame || !s_stream.outputCompactFrame) {
        printf("STREAM_FATAL,reason=heap_alloc_failed,outputRx=%p,scanTx=%p,dropScratch=%p,compact=%p\n",
               (void *)s_stream.outputRxFrame,
               (void *)s_stream.scanTxFrame,
               (void *)s_stream.dropScratchFrame,
               (void *)s_stream.outputCompactFrame);
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

static void sensorarrayVoltageStreamReleaseBuffers(void)
{
    if (s_stream.outputRxFrame) {
        heap_caps_free(s_stream.outputRxFrame);
        s_stream.outputRxFrame = NULL;
    }
    if (s_stream.scanTxFrame) {
        heap_caps_free(s_stream.scanTxFrame);
        s_stream.scanTxFrame = NULL;
    }
    if (s_stream.dropScratchFrame && s_stream.dropScratchFrame != &s_dropScratchStatic) {
        heap_caps_free(s_stream.dropScratchFrame);
    }
    s_stream.dropScratchFrame = NULL;
    if (s_stream.outputCompactFrame) {
        heap_caps_free(s_stream.outputCompactFrame);
        s_stream.outputCompactFrame = NULL;
    }
}

static uint32_t sensorarrayVoltageStreamCrc32Bytes(const uint8_t *data, size_t len)
{
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc & 1u) ? ((crc >> 1) ^ 0xEDB88320u) : (crc >> 1);
        }
    }
    return ~crc;
}

static uint16_t sensorarrayVoltageStreamSaturateU16(uint32_t value)
{
    return value > UINT16_MAX ? UINT16_MAX : (uint16_t)value;
}

uint32_t sensorarrayVoltageCompactFrameCrc32(const sensorarrayVoltageCompactFrame_t *frame)
{
    if (!frame) {
        return 0u;
    }
    return sensorarrayVoltageStreamCrc32Bytes((const uint8_t *)frame,
                                              offsetof(sensorarrayVoltageCompactFrame_t, crc32));
}

void sensorarrayVoltageCompactFrameFromScan(const sensorarrayVoltageFrame_t *scan,
                                            sensorarrayVoltageCompactFrame_t *out)
{
    if (!scan || !out) {
        return;
    }

    memset(out, 0, sizeof(*out));
    out->magic = SENSORARRAY_VOLTAGE_COMPACT_MAGIC;
    out->version = SENSORARRAY_VOLTAGE_COMPACT_VERSION;
    out->frameType = SENSORARRAY_VOLTAGE_COMPACT_TYPE_ADS126X_UV;
    out->sequence = scan->sequence;
    out->timestampUs = scan->timestampUs;
    out->scanDurationUs = scan->scanDurationUs;
    out->statusFlags = scan->statusFlags;
    out->firstStatusCode = scan->firstStatusCode;
    out->lastStatusCode = scan->lastStatusCode;
    out->droppedFrames = sensorarrayVoltageStreamSaturateU16(scan->droppedFrames);
    out->outputDecimatedFrames = sensorarrayVoltageStreamSaturateU16(scan->outputDecimatedFrames);
    out->validMask = scan->validMask;
    out->adsDr = scan->adsDr;
    out->outputDivider = scan->outputDivider;

    for (uint8_t row = 0u; row < SENSORARRAY_VOLTAGE_SCAN_ROWS; ++row) {
        for (uint8_t col = 0u; col < SENSORARRAY_VOLTAGE_SCAN_COLS; ++col) {
            out->microvolts[(uint32_t)row * SENSORARRAY_VOLTAGE_SCAN_COLS + col] = scan->microvolts[row][col];
        }
    }
    out->crc32 = sensorarrayVoltageCompactFrameCrc32(out);
}

static void sensorarrayVoltageStreamConfigureStdout(void)
{
    setvbuf(stdout, NULL, _IONBF, 0);
    s_stream.usbNonblocking = false;

#if CONFIG_SENSORARRAY_USB_STDOUT_NONBLOCKING
#if SENSORARRAY_HAVE_USB_SERIAL_JTAG_DRIVER
    if (!usb_serial_jtag_is_driver_installed()) {
        usb_serial_jtag_driver_config_t cfg = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
        (void)usb_serial_jtag_driver_install(&cfg);
    }
#endif
#if SENSORARRAY_HAVE_USB_SERIAL_JTAG_VFS
    usb_serial_jtag_vfs_use_nonblocking();
    s_stream.usbNonblocking = true;
#endif
    int flags = fcntl(STDOUT_FILENO, F_GETFL, 0);
    if (flags >= 0 && fcntl(STDOUT_FILENO, F_SETFL, flags | O_NONBLOCK) == 0) {
        s_stream.usbNonblocking = true;
    }
#endif
}

static void sensorarrayVoltageStreamPrintHeader(void)
{
    if (s_stream.headerPrinted) {
        return;
    }
    printf("MATV_HEADER,seq,timestamp_us,duration_us,unit");
    for (uint8_t s = 1u; s <= SENSORARRAY_VOLTAGE_SCAN_ROWS; ++s) {
        for (uint8_t d = 1u; d <= SENSORARRAY_VOLTAGE_SCAN_COLS; ++d) {
            printf(",S%uD%u", (unsigned)s, (unsigned)d);
        }
    }
    printf("\n");
    s_stream.headerPrinted = true;
}

static void __attribute__((unused)) sensorarrayVoltageStreamPrintCsv(const sensorarrayVoltageFrame_t *frame)
{
    if (!frame) {
        return;
    }
    sensorarrayVoltageStreamPrintHeader();
    printf("MATV,%" PRIu32 ",%" PRIu64 ",%" PRIu32 ",uV",
           frame->sequence,
           frame->timestampUs,
           frame->scanDurationUs);
    for (uint8_t row = 0u; row < SENSORARRAY_VOLTAGE_SCAN_ROWS; ++row) {
        for (uint8_t col = 0u; col < SENSORARRAY_VOLTAGE_SCAN_COLS; ++col) {
            printf(",%" PRIi32, frame->microvolts[row][col]);
        }
    }
    printf("\n");
}

static void __attribute__((unused)) sensorarrayVoltageStreamRecordUsbWrite(size_t requested, size_t written, int64_t startUs)
{
    uint32_t elapsedUs = (uint32_t)(esp_timer_get_time() - startUs);
    sensorarrayPerfAddSample(&s_stream.perf.usbWriteTotalUs, &s_stream.perf.usbWriteMaxUs, elapsedUs);
    s_stream.usbWriteCount++;

    if (written == requested) {
        return;
    }
    if (written == 0u) {
        sensorarrayStatusRecord(&s_stream.status,
                                SENSORARRAY_STATUS_USB_STDOUT_WRITE_FAIL,
                                SENSORARRAY_FRAME_FLAG_OUTPUT_THROTTLED);
    }
}

static size_t sensorarrayVoltageStreamWriteAllBounded(const uint8_t *data,
                                                      size_t length,
                                                      uint32_t maxAttempts)
{
    if (!data || length == 0u || maxAttempts == 0u) {
        return 0u;
    }

    size_t total = 0u;
    uint32_t attempts = 0u;
    while (total < length && attempts < maxAttempts) {
        errno = 0;
        ssize_t result = write(STDOUT_FILENO, data + total, length - total);
        attempts++;
        if (result > 0) {
            total += (size_t)result;
            if (total == length) {
                break;
            }
            taskYIELD();
            continue;
        }

        if (result < 0 && errno == EINTR) {
            continue;
        }
        if (result < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            vTaskDelay(pdMS_TO_TICKS(1u));
            continue;
        }
        break;
    }

    if (total > 0u && total < length) {
        sensorarrayStatusRecord(&s_stream.status,
                                SENSORARRAY_STATUS_BINARY_WRITE_PARTIAL,
                                SENSORARRAY_FRAME_FLAG_OUTPUT_THROTTLED);
    }

    return total;
}

static bool __attribute__((unused)) sensorarrayVoltageStreamWriteBinary(const sensorarrayVoltageFrame_t *frame)
{
#if CONFIG_SENSORARRAY_OUTPUT_FORMAT_BINARY || CONFIG_SENSORARRAY_OUTPUT_FORMAT_BOTH
    if (!frame || !s_stream.outputCompactFrame) {
        return false;
    }

    sensorarrayVoltageCompactFrame_t *compact = s_stream.outputCompactFrame;
    memset(compact, 0, sizeof(*compact));
    sensorarrayVoltageCompactFrameFromScan(frame, compact);
    int64_t startUs = esp_timer_get_time();
    size_t written = sensorarrayVoltageStreamWriteAllBounded((const uint8_t *)compact,
                                                             sizeof(*compact),
                                                             SENSORARRAY_BINARY_WRITE_MAX_ATTEMPTS);
    sensorarrayVoltageStreamRecordUsbWrite(sizeof(*compact), written, startUs);
    if (written == sizeof(*compact)) {
        s_stream.perf.framesOutput++;
        return true;
    }
    return false;
#else
    (void)frame;
    return false;
#endif
}

static bool sensorarrayVoltageStreamShouldPrintTextStatus(void)
{
#if CONFIG_SENSORARRAY_OUTPUT_FORMAT_BINARY
    if (!CONFIG_SENSORARRAY_BINARY_TEXT_STATUS) {
        return false;
    }
#endif

    if (s_stream.textStatusSuppressedByStack) {
        return false;
    }

    if (CONFIG_SENSORARRAY_STATUS_PERIOD_N_FRAMES == 0) {
        return false;
    }

    return true;
}

static void sensorarrayVoltageStreamUpdateStackWatermarks(uint32_t sequence)
{
    if ((sequence % (uint32_t)CONFIG_SENSORARRAY_STACK_WATERMARK_PERIOD_N_FRAMES) != 0u) {
        return;
    }

    if (s_stream.outputTask) {
        UBaseType_t outWater = uxTaskGetStackHighWaterMark(s_stream.outputTask);
        s_stream.outputStackMinWords =
            (UBaseType_t)sensorarrayPerfMinNonZeroU32((uint32_t)s_stream.outputStackMinWords,
                                                      (uint32_t)outWater);

        if (outWater < (UBaseType_t)CONFIG_SENSORARRAY_OUTPUT_STACK_LOW_WORDS) {
            s_stream.outputStackLowCount++;
            sensorarrayStatusRecord(&s_stream.status,
                                    SENSORARRAY_STATUS_OUT_STACK_LOW,
                                    SENSORARRAY_FRAME_FLAG_OUTPUT_THROTTLED);
        }

        if (outWater < (UBaseType_t)CONFIG_SENSORARRAY_OUTPUT_STACK_CRITICAL_WORDS) {
            s_stream.outputStackCriticalCount++;
            s_stream.textStatusSuppressedByStack = true;

            sensorarrayStatusRecord(&s_stream.status,
                                    SENSORARRAY_STATUS_OUT_STACK_CRITICAL,
                                    SENSORARRAY_FRAME_FLAG_OUTPUT_THROTTLED);

            sensorarrayRateControllerForceOutputDividerAtLeast(&s_stream.controller, 4u);
        }
    }

    if (s_stream.scanTask) {
        UBaseType_t scanWater = uxTaskGetStackHighWaterMark(s_stream.scanTask);
        s_stream.scanStackMinWords =
            (UBaseType_t)sensorarrayPerfMinNonZeroU32((uint32_t)s_stream.scanStackMinWords,
                                                      (uint32_t)scanWater);
    }
}

static void sensorarrayVoltageStreamPrintStat(const sensorarrayVoltageFrame_t *frame)
{
    if (!frame) {
        return;
    }

    uint32_t frames = s_stream.perf.framesScanned ? s_stream.perf.framesScanned : 1u;
    uint32_t points = s_stream.perf.pointsScanned ? s_stream.perf.pointsScanned : 1u;
    uint32_t routeCount = s_stream.perf.framesScanned * SENSORARRAY_VOLTAGE_SCAN_ROWS;
    if (routeCount == 0u) {
        routeCount = 1u;
    }

    int64_t nowUs = esp_timer_get_time();
    uint64_t elapsedStreamUs = (s_stream.streamStartUs > 0 && nowUs > s_stream.streamStartUs)
                                   ? (uint64_t)(nowUs - s_stream.streamStartUs)
                                   : 1u;
    uint32_t scanAvgUs = sensorarrayPerfAvgU32(s_stream.perf.scanTotalUs, frames);
    uint32_t scanFps = (uint32_t)(((uint64_t)s_stream.perf.framesScanned * 1000000u) / elapsedStreamUs);
    uint32_t outFps = (uint32_t)(((uint64_t)s_stream.perf.framesOutput * 1000000u) / elapsedStreamUs);
    uint32_t fps = scanFps;
    uint32_t pps = scanFps * SENSORARRAY_VOLTAGE_SCAN_ROWS * SENSORARRAY_VOLTAGE_SCAN_COLS;
    uint32_t adsSps = ads126xAdcDataRateCodeToSps(s_stream.controller.currentAdsDr);
    uint32_t qUsed = s_stream.queue ? uxQueueMessagesWaiting(s_stream.queue) : 0u;

    printf("STAT,seq=%" PRIu32 ",fps=%" PRIu32 ",scanFps=%" PRIu32 ",outFps=%" PRIu32
           ",pps=%" PRIu32 ",scanAvgUs=%" PRIu32 ",scanMaxUs=%" PRIu32
           ",routeAvgUs=%" PRIu32 ",inpmuxAvgUs=%" PRIu32
           ",drdyAvgUs=%" PRIu32 ",adcReadAvgUs=%" PRIu32
           ",spiAvgUs=%" PRIu32 ",queueAvgUs=%" PRIu32 ",usbAvgUs=%" PRIu32
           ",usbMaxUs=%" PRIu32 ",qDepth=%u,qUsed=%" PRIu32
           ",drop=%" PRIu32 ",decimated=%" PRIu32 ",qFull=%" PRIu32
           ",shortWrite=%" PRIu32 ",writeFail=%" PRIu32
           ",drdyTimeout=%" PRIu32 ",spiFail=%" PRIu32
           ",adsDr=%u,adsSps=%" PRIu32 ",outputDiv=%" PRIu32
           ",scanPeriodUs=%" PRIu32 ",status=0x%08" PRIX32 ",code=0x%04" PRIX32
           ",outStackMinWords=%u,scanStackMinWords=%u,outStackLow=%" PRIu32
           ",outStackCritical=%" PRIu32 ",textSuppressed=%u,heapFree=%u,heapMinFree=%u\n",
           frame->sequence,
           fps,
           scanFps,
           outFps,
           pps,
           scanAvgUs,
           s_stream.perf.scanMaxUs,
           sensorarrayPerfAvgU32(s_stream.perf.routeTotalUs, routeCount),
           sensorarrayPerfAvgU32(s_stream.perf.inpmuxWriteTotalUs, points),
           sensorarrayPerfAvgU32(s_stream.perf.drdyWaitTotalUs, points),
           sensorarrayPerfAvgU32(s_stream.perf.adcReadTotalUs, points),
           sensorarrayPerfAvgU32(s_stream.perf.spiTransferTotalUs,
                                 s_stream.perf.spiTransactionCount ? s_stream.perf.spiTransactionCount : 1u),
           sensorarrayPerfAvgU32(s_stream.perf.queueSendTotalUs,
                                 s_stream.perf.framesQueued ? s_stream.perf.framesQueued : 1u),
           sensorarrayPerfAvgU32(s_stream.perf.usbWriteTotalUs,
                                 s_stream.usbWriteCount ? s_stream.usbWriteCount : 1u),
           s_stream.perf.usbWriteMaxUs,
           (unsigned)CONFIG_SENSORARRAY_STREAM_QUEUE_DEPTH,
           qUsed,
           s_stream.status.droppedFrameCount,
           s_stream.status.outputDecimatedFrameCount,
           s_stream.status.queueFullCount,
           s_stream.status.usbShortWriteCount,
           s_stream.status.usbWriteFailCount,
           s_stream.status.adsDrdyTimeoutCount,
           s_stream.status.adsSpiFailCount,
           (unsigned)s_stream.controller.currentAdsDr,
           adsSps,
           s_stream.controller.outputDivider,
           s_stream.controller.scanFramePeriodUs,
           frame->statusFlags,
           frame->lastStatusCode,
           (unsigned)s_stream.outputStackMinWords,
           (unsigned)s_stream.scanStackMinWords,
           s_stream.outputStackLowCount,
           s_stream.outputStackCriticalCount,
           (s_stream.textStatusSuppressedByStack || s_stream.textStatusSuppressedByBinaryMode) ? 1u : 0u,
           (unsigned)heap_caps_get_free_size(MALLOC_CAP_8BIT),
           (unsigned)heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT));
}

static void sensorarrayVoltageStreamPrintRateEvent(const sensorarrayVoltageStreamFrame_t *streamFrame)
{
    if (!streamFrame || streamFrame->rateAction == SENSORARRAY_RATE_ACTION_NONE) {
        return;
    }

    const sensorarrayVoltageFrame_t *frame = &streamFrame->frame;
    printf("%s,seq=%" PRIu32 ",action=%s,cause=%s,adsDr=%u,adsSps=%" PRIu32
           ",outputDiv=%" PRIu32 ",scanPeriodUs=%" PRIu32 ",muxSettleUs=%" PRIu32
           ",drop=%" PRIu32 ",decimated=%" PRIu32 ",queueFull=%" PRIu32
           ",usbFail=%" PRIu32 ",drdyTimeout=%" PRIu32 ",spiFail=%" PRIu32 "\n",
           (streamFrame->rateAction == SENSORARRAY_RATE_ACTION_FATAL_STOP) ? "RATE_FATAL" : "RATE_EVENT",
           frame->sequence,
           sensorarrayRateActionName(streamFrame->rateAction),
           sensorarrayRateCauseName(streamFrame->rateCause),
           (unsigned)s_stream.controller.currentAdsDr,
           ads126xAdcDataRateCodeToSps(s_stream.controller.currentAdsDr),
           s_stream.controller.outputDivider,
           s_stream.controller.scanFramePeriodUs,
           s_stream.controller.muxSettleUs,
           s_stream.status.droppedFrameCount,
           s_stream.status.outputDecimatedFrameCount,
           s_stream.status.queueFullCount,
           s_stream.status.usbWriteFailCount + s_stream.status.usbShortWriteCount,
           s_stream.status.adsDrdyTimeoutCount,
           s_stream.status.adsSpiFailCount);
}

static void sensorarrayVoltageStreamPrintStartup(void)
{
    printf("ADS_FAST_CONFIG,dr=%u,status=%u,crc=%u,directRead=%u,fastMux=%u,fixedGain=%u,"
           "pollingSpi=%u,busAcquire=%u,drdyTimeoutUs=%" PRIu32 "\n",
           (unsigned)s_stream.config.ads->dataRateDr,
           s_stream.config.ads->enableStatusByte ? 1u : 0u,
           s_stream.config.ads->crcMode != ADS126X_CRC_OFF ? 1u : 0u,
           s_stream.config.ads->fastOptions.directRead ? 1u : 0u,
           s_stream.config.ads->fastOptions.fastInpmuxWrite ? 1u : 0u,
           s_stream.config.ads->fastOptions.fixedGain ? 1u : 0u,
           s_stream.config.ads->fastOptions.usePollingTransmit ? 1u : 0u,
           s_stream.config.ads->fastOptions.acquireBusPerFrame ? 1u : 0u,
           s_stream.config.ads->fastOptions.drdyTimeoutUs);

    const char *format =
#if CONFIG_SENSORARRAY_OUTPUT_FORMAT_BINARY
        "binary";
#elif CONFIG_SENSORARRAY_OUTPUT_FORMAT_BOTH
        "both";
#elif CONFIG_SENSORARRAY_OUTPUT_FORMAT_OFF
        "off";
#else
        "csv";
#endif

    printf("VOLTSCAN_CONFIG,mode=%s,dr=%u,format=%s,queueDepth=%u,scanCore=%u,commCore=%u,"
           "dma=%u,spiPolling=%u,busAcquire=%u,csvEvery=%u,discardFirst=%u,oversample=%u,"
           "rowSettleUs=%u,pathSettleUs=%u,muxSettleUs=%" PRIu32 ",autoRate=%u,usbNonblocking=%u\n",
           s_stream.config.modeName ? s_stream.config.modeName : "UNKNOWN",
           (unsigned)s_stream.config.ads->dataRateDr,
           format,
           (unsigned)CONFIG_SENSORARRAY_STREAM_QUEUE_DEPTH,
           (unsigned)CONFIG_SENSORARRAY_SCAN_TASK_CORE,
           (unsigned)CONFIG_SENSORARRAY_COMM_TASK_CORE,
           s_stream.config.ads->spiDmaCapable ? 1u : 0u,
           s_stream.config.ads->fastOptions.usePollingTransmit ? 1u : 0u,
           s_stream.config.ads->fastOptions.acquireBusPerFrame ? 1u : 0u,
           (unsigned)CONFIG_SENSORARRAY_VOLTAGE_SCAN_CSV_EVERY_N,
           CONFIG_SENSORARRAY_VOLTAGE_SCAN_DISCARD_FIRST ? 1u : 0u,
           (unsigned)CONFIG_SENSORARRAY_VOLTAGE_SCAN_OVERSAMPLE,
           (unsigned)CONFIG_SENSORARRAY_VOLTAGE_SCAN_ROW_SETTLE_US,
           (unsigned)CONFIG_SENSORARRAY_VOLTAGE_SCAN_PATH_SETTLE_US,
           s_stream.controller.muxSettleUs,
           CONFIG_SENSORARRAY_AUTO_RATE_CONTROL ? 1u : 0u,
           s_stream.usbNonblocking ? 1u : 0u);

    printf("STREAM_MEM,streamFrameSize=%u,compactFrameSize=%u,queueDepth=%u,queueBytes=%u,"
           "commStack=%u,scanStack=%u,heapFree=%u,heapMinFree=%u\n",
           (unsigned)sizeof(sensorarrayVoltageStreamFrame_t),
           (unsigned)sizeof(sensorarrayVoltageCompactFrame_t),
           (unsigned)CONFIG_SENSORARRAY_STREAM_QUEUE_DEPTH,
           (unsigned)(CONFIG_SENSORARRAY_STREAM_QUEUE_DEPTH * sizeof(sensorarrayVoltageStreamFrame_t)),
           (unsigned)CONFIG_SENSORARRAY_COMM_TASK_STACK,
           (unsigned)CONFIG_SENSORARRAY_SCAN_TASK_STACK,
           (unsigned)heap_caps_get_free_size(MALLOC_CAP_8BIT),
           (unsigned)heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT));

#if CONFIG_SENSORARRAY_BINARY_STARTUP_MARKER
#if CONFIG_SENSORARRAY_OUTPUT_FORMAT_BINARY
    printf("STREAM_INIT,format=binary,magic=0x%08" PRIX32 ",magicBytes=SAC1,version=%u,"
           "frameType=0x%04X,frameSize=%u,csvEvery=%u,queueDepth=%u,statusEvery=%u,textStatus=%u\n",
           (uint32_t)SENSORARRAY_VOLTAGE_COMPACT_MAGIC,
           (unsigned)SENSORARRAY_VOLTAGE_COMPACT_VERSION,
           (unsigned)SENSORARRAY_VOLTAGE_COMPACT_TYPE_ADS126X_UV,
           (unsigned)SENSORARRAY_VOLTAGE_COMPACT_SIZE,
           (unsigned)CONFIG_SENSORARRAY_VOLTAGE_SCAN_CSV_EVERY_N,
           (unsigned)CONFIG_SENSORARRAY_STREAM_QUEUE_DEPTH,
           (unsigned)CONFIG_SENSORARRAY_STATUS_PERIOD_N_FRAMES,
           sensorarrayVoltageStreamShouldPrintTextStatus() ? 1u : 0u);
#elif CONFIG_SENSORARRAY_OUTPUT_FORMAT_BOTH
    printf("STREAM_INIT,format=both,magic=0x%08" PRIX32 ",magicBytes=SAC1,version=%u,"
           "frameType=0x%04X,frameSize=%u,csvEvery=%u,queueDepth=%u,statusEvery=%u,textStatus=%u\n",
           (uint32_t)SENSORARRAY_VOLTAGE_COMPACT_MAGIC,
           (unsigned)SENSORARRAY_VOLTAGE_COMPACT_VERSION,
           (unsigned)SENSORARRAY_VOLTAGE_COMPACT_TYPE_ADS126X_UV,
           (unsigned)SENSORARRAY_VOLTAGE_COMPACT_SIZE,
           (unsigned)CONFIG_SENSORARRAY_VOLTAGE_SCAN_CSV_EVERY_N,
           (unsigned)CONFIG_SENSORARRAY_STREAM_QUEUE_DEPTH,
           (unsigned)CONFIG_SENSORARRAY_STATUS_PERIOD_N_FRAMES,
           sensorarrayVoltageStreamShouldPrintTextStatus() ? 1u : 0u);
#elif CONFIG_SENSORARRAY_OUTPUT_FORMAT_CSV
    printf("STREAM_INIT,format=csv,csvEvery=%u,queueDepth=%u,statusEvery=%u,textStatus=%u\n",
           (unsigned)CONFIG_SENSORARRAY_VOLTAGE_SCAN_CSV_EVERY_N,
           (unsigned)CONFIG_SENSORARRAY_STREAM_QUEUE_DEPTH,
           (unsigned)CONFIG_SENSORARRAY_STATUS_PERIOD_N_FRAMES,
           sensorarrayVoltageStreamShouldPrintTextStatus() ? 1u : 0u);
#else
    printf("STREAM_INIT,format=off,queueDepth=%u,statusEvery=%u,textStatus=%u\n",
           (unsigned)CONFIG_SENSORARRAY_STREAM_QUEUE_DEPTH,
           (unsigned)CONFIG_SENSORARRAY_STATUS_PERIOD_N_FRAMES,
           sensorarrayVoltageStreamShouldPrintTextStatus() ? 1u : 0u);
#endif
#endif

    printf("ROUTE_POLICY,mode=%s,sw=%s,routePolicyOk=%u\n",
           s_stream.config.modeName ? s_stream.config.modeName : "UNKNOWN",
           s_stream.config.swName ? s_stream.config.swName : "UNKNOWN",
           s_stream.config.routePolicyOk ? 1u : 0u);
    printf("ADS_POLICY,mode=%s,intref=%u,vbias=%u,expectedRefmux=0x%02X,adsPolicyOk=%u\n",
           s_stream.config.modeName ? s_stream.config.modeName : "UNKNOWN",
           s_stream.config.useAdsInternalRef ? 1u : 0u,
           s_stream.config.useAdsVbias ? 1u : 0u,
           s_stream.config.expectedAdsRefmux,
           s_stream.config.adsPolicyOk ? 1u : 0u);
}

static void sensorarrayVoltageOutputTask(void *arg)
{
    (void)arg;
    sensorarrayVoltageStreamConfigureStdout();
    sensorarrayVoltageStreamPrintStartup();

    while (true) {
        sensorarrayVoltageStreamFrame_t *streamFrame = s_stream.outputRxFrame;
        if (!streamFrame) {
            sensorarrayStatusRecord(&s_stream.status,
                                    SENSORARRAY_STATUS_STREAM_INTERNAL_ERROR,
                                    SENSORARRAY_FRAME_FLAG_OUTPUT_THROTTLED);
            vTaskDelay(pdMS_TO_TICKS(100u));
            continue;
        }

        memset(streamFrame, 0, sizeof(*streamFrame));

        if (xQueueReceive(s_stream.queue, streamFrame, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        sensorarrayVoltageStreamUpdateStackWatermarks(streamFrame->frame.sequence);

        if (sensorarrayVoltageStreamShouldPrintTextStatus()) {
            sensorarrayVoltageStreamPrintRateEvent(streamFrame);
        }

#if CONFIG_SENSORARRAY_OUTPUT_FORMAT_BINARY || CONFIG_SENSORARRAY_OUTPUT_FORMAT_BOTH
        uint32_t shortBefore = s_stream.status.usbShortWriteCount;
        uint32_t failBefore = s_stream.status.usbWriteFailCount;
        bool binaryWritten = sensorarrayVoltageStreamWriteBinary(&streamFrame->frame);
        if (!binaryWritten) {
            sensorarrayStatusCode_t code =
                (s_stream.status.usbWriteFailCount != failBefore)
                    ? SENSORARRAY_STATUS_USB_STDOUT_WRITE_FAIL
                    : SENSORARRAY_STATUS_BINARY_WRITE_PARTIAL;
            if (s_stream.status.usbShortWriteCount != shortBefore ||
                s_stream.status.usbWriteFailCount != failBefore) {
                streamFrame->frame.statusFlags |= SENSORARRAY_FRAME_FLAG_OUTPUT_THROTTLED;
                if (streamFrame->frame.firstStatusCode == SENSORARRAY_STATUS_OK) {
                    streamFrame->frame.firstStatusCode = code;
                }
                streamFrame->frame.lastStatusCode = code;
            }
        }
#endif

#if CONFIG_SENSORARRAY_OUTPUT_FORMAT_CSV || CONFIG_SENSORARRAY_OUTPUT_FORMAT_BOTH
        if (s_stream.controller.csvEnabled &&
            CONFIG_SENSORARRAY_VOLTAGE_SCAN_CSV_EVERY_N > 0 &&
            (streamFrame->frame.sequence % (uint32_t)CONFIG_SENSORARRAY_VOLTAGE_SCAN_CSV_EVERY_N) == 0u) {
            sensorarrayVoltageStreamPrintCsv(&streamFrame->frame);
        }
#endif

        if (sensorarrayVoltageStreamShouldPrintTextStatus() &&
            CONFIG_SENSORARRAY_STATUS_PERIOD_N_FRAMES > 0 &&
            (streamFrame->frame.sequence % (uint32_t)CONFIG_SENSORARRAY_STATUS_PERIOD_N_FRAMES) == 0u) {
            sensorarrayVoltageStreamPrintStat(&streamFrame->frame);
        }
    }
}

static sensorarrayStatusCode_t sensorarrayVoltageStreamStatusForAction(sensorarrayRateAction_t action)
{
    switch (action) {
    case SENSORARRAY_RATE_ACTION_INCREASE_OUTPUT_DIVIDER:
    case SENSORARRAY_RATE_ACTION_DISABLE_CSV:
    case SENSORARRAY_RATE_ACTION_USE_COMPACT_BINARY_ONLY:
        return SENSORARRAY_STATUS_RATE_OUTPUT_DECIMATED;
    case SENSORARRAY_RATE_ACTION_ADD_SCAN_FRAME_PERIOD:
        return SENSORARRAY_STATUS_RATE_SCAN_THROTTLED;
    case SENSORARRAY_RATE_ACTION_DECREASE_ADS_DR:
        return SENSORARRAY_STATUS_RATE_ADS_DR_REDUCED;
    case SENSORARRAY_RATE_ACTION_INCREASE_MUX_SETTLE:
        return SENSORARRAY_STATUS_RATE_MUX_SETTLE_INCREASED;
    case SENSORARRAY_RATE_ACTION_FORCE_VERIFIED_INPMUX:
        return SENSORARRAY_STATUS_RATE_VERIFIED_MUX_FORCED;
    case SENSORARRAY_RATE_ACTION_ENTER_SAFE_PROFILE:
        return SENSORARRAY_STATUS_RATE_SAFE_PROFILE_ENTERED;
    case SENSORARRAY_RATE_ACTION_FATAL_STOP:
        return SENSORARRAY_STATUS_RATE_FATAL_STOP;
    default:
        return SENSORARRAY_STATUS_OK;
    }
}

static void sensorarrayVoltageStreamQueueFrame(sensorarrayVoltageStreamFrame_t *streamFrame)
{
    if (!streamFrame) {
        return;
    }

    int64_t queueStartUs = esp_timer_get_time();
    BaseType_t sent = xQueueSend(s_stream.queue, streamFrame, 0);
    uint32_t queueElapsedUs = (uint32_t)(esp_timer_get_time() - queueStartUs);
    sensorarrayPerfAddSample(&s_stream.perf.queueSendTotalUs, &s_stream.perf.queueSendMaxUs, queueElapsedUs);

    if (sent == pdTRUE) {
        s_stream.perf.framesQueued++;
        return;
    }

    sensorarrayStatusRecord(&s_stream.status,
                            SENSORARRAY_STATUS_STREAM_QUEUE_FULL,
                            SENSORARRAY_FRAME_FLAG_QUEUE_DROPPED);

#if CONFIG_SENSORARRAY_STREAM_DROP_OLDEST
    sensorarrayVoltageStreamFrame_t *oldFrame = s_stream.dropScratchFrame;
    if (oldFrame) {
        memset(oldFrame, 0, sizeof(*oldFrame));
        (void)xQueueReceive(s_stream.queue, oldFrame, 0);
    } else {
        sensorarrayStatusRecord(&s_stream.status,
                                SENSORARRAY_STATUS_STREAM_INTERNAL_ERROR,
                                SENSORARRAY_FRAME_FLAG_QUEUE_DROPPED);
    }
    s_stream.perf.framesDropped++;
    sensorarrayStatusRecord(&s_stream.status,
                            SENSORARRAY_STATUS_STREAM_FRAME_DROPPED,
                            SENSORARRAY_FRAME_FLAG_QUEUE_DROPPED);
    streamFrame->frame.statusFlags |= SENSORARRAY_FRAME_FLAG_QUEUE_DROPPED;
    if (streamFrame->frame.firstStatusCode == SENSORARRAY_STATUS_OK) {
        streamFrame->frame.firstStatusCode = SENSORARRAY_STATUS_STREAM_FRAME_DROPPED;
    }
    streamFrame->frame.lastStatusCode = SENSORARRAY_STATUS_STREAM_FRAME_DROPPED;
    streamFrame->frame.droppedFrames = s_stream.status.droppedFrameCount;
    streamFrame->frame.outputDecimatedFrames = s_stream.status.outputDecimatedFrameCount;
    if (xQueueSend(s_stream.queue, streamFrame, 0) == pdTRUE) {
        s_stream.perf.framesQueued++;
    }
#elif CONFIG_SENSORARRAY_STREAM_BLOCK_SAFE_ONLY && CONFIG_SENSORARRAY_VOLTAGE_SCAN_PROFILE_SAFE
    if (xQueueSend(s_stream.queue, streamFrame, portMAX_DELAY) == pdTRUE) {
        s_stream.perf.framesQueued++;
    }
#else
    s_stream.perf.framesDropped++;
    sensorarrayStatusRecord(&s_stream.status,
                            SENSORARRAY_STATUS_STREAM_FRAME_DROPPED,
                            SENSORARRAY_FRAME_FLAG_QUEUE_DROPPED);
    streamFrame->frame.statusFlags |= SENSORARRAY_FRAME_FLAG_QUEUE_DROPPED;
    if (streamFrame->frame.firstStatusCode == SENSORARRAY_STATUS_OK) {
        streamFrame->frame.firstStatusCode = SENSORARRAY_STATUS_STREAM_FRAME_DROPPED;
    }
    streamFrame->frame.lastStatusCode = SENSORARRAY_STATUS_STREAM_FRAME_DROPPED;
    streamFrame->frame.droppedFrames = s_stream.status.droppedFrameCount;
    streamFrame->frame.outputDecimatedFrames = s_stream.status.outputDecimatedFrameCount;
#endif
}

static void sensorarrayVoltageScanTask(void *arg)
{
    (void)arg;

    while (s_stream.running && !s_stream.controller.fatalStop) {
        sensorarrayVoltageStreamFrame_t *streamFrame = s_stream.scanTxFrame;
        if (!streamFrame) {
            sensorarrayStatusRecord(&s_stream.status,
                                    SENSORARRAY_STATUS_STREAM_INTERNAL_ERROR,
                                    SENSORARRAY_FRAME_FLAG_ADS_ERROR);
            vTaskDelay(pdMS_TO_TICKS(100u));
            continue;
        }

        memset(streamFrame, 0, sizeof(*streamFrame));

        sensorarrayVoltageScanSetFastRuntimeOptions(s_stream.controller.muxSettleUs,
                                                    s_stream.controller.verifiedInpmuxForced);
        s_stream.config.ads->fastOptions.fastInpmuxWrite = s_stream.controller.fastInpmuxEnabled;

        (void)sensorarrayVoltageScanOneFrameFastAds(s_stream.config.ads,
                                                    s_stream.config.swSource,
                                                    &streamFrame->frame,
                                                    &s_stream.status,
                                                    &s_stream.perf);

        streamFrame->frame.outputDivider = (uint8_t)(s_stream.controller.outputDivider > 255u
                                                         ? 255u
                                                         : s_stream.controller.outputDivider);
        streamFrame->frame.adsDr = s_stream.controller.currentAdsDr;
        streamFrame->frame.rateControlLevel = (uint16_t)s_stream.controller.degradeCount;
        streamFrame->frame.scanFramePeriodUs = s_stream.controller.scanFramePeriodUs;

        if ((streamFrame->frame.sequence % (uint32_t)CONFIG_SENSORARRAY_AUTO_RATE_WINDOW_FRAMES) == 0u) {
            sensorarrayRateAction_t action = SENSORARRAY_RATE_ACTION_NONE;
            sensorarrayRateCause_t cause = SENSORARRAY_RATE_CAUSE_NONE;
            uint32_t queueUsed = uxQueueMessagesWaiting(s_stream.queue);
            if (sensorarrayRateControllerUpdate(&s_stream.controller,
                                                &s_stream.perf,
                                                &s_stream.status,
                                                CONFIG_SENSORARRAY_STREAM_QUEUE_DEPTH,
                                                queueUsed,
                                                &action,
                                                &cause) == ESP_OK &&
                action != SENSORARRAY_RATE_ACTION_NONE) {
                (void)sensorarrayRateControllerApplyAction(&s_stream.controller,
                                                           action,
                                                           cause,
                                                           s_stream.config.ads);
                sensorarrayStatusCode_t code = sensorarrayVoltageStreamStatusForAction(action);
                sensorarrayStatusRecord(&s_stream.status,
                                        code,
                                        SENSORARRAY_FRAME_FLAG_RATE_LIMITED);
                streamFrame->rateAction = action;
                streamFrame->rateCause = cause;
            }
        }

        bool publish = sensorarrayRateControllerShouldPublishFrame(&s_stream.controller,
                                                                   streamFrame->frame.sequence);
        if (publish) {
            sensorarrayVoltageStreamQueueFrame(streamFrame);
        } else {
            s_stream.perf.outputDecimatedFrames++;
            sensorarrayStatusRecord(&s_stream.status,
                                    SENSORARRAY_STATUS_RATE_OUTPUT_DECIMATED,
                                    SENSORARRAY_FRAME_FLAG_OUTPUT_DECIMATED);
        }

        if (s_stream.controller.scanFramePeriodUs > streamFrame->frame.scanDurationUs) {
            uint32_t throttleUs = s_stream.controller.scanFramePeriodUs - streamFrame->frame.scanDurationUs;
            if (throttleUs < 1000u) {
                esp_rom_delay_us(throttleUs);
            } else {
                vTaskDelay(pdMS_TO_TICKS(throttleUs / 1000u));
            }
        }
    }

    vTaskDelete(NULL);
}

esp_err_t sensorarrayVoltageStreamStart(const sensorarrayVoltageStreamConfig_t *config)
{
    if (!config || !config->ads) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_stream.running) {
        return ESP_ERR_INVALID_STATE;
    }

    memset(&s_stream, 0, sizeof(s_stream));
    s_stream.config = *config;

#if CONFIG_SENSORARRAY_OUTPUT_FORMAT_BINARY
    s_stream.textStatusSuppressedByBinaryMode = !CONFIG_SENSORARRAY_BINARY_TEXT_STATUS;
#endif

    esp_err_t memErr = sensorarrayVoltageStreamAllocateBuffers();
    if (memErr != ESP_OK) {
        sensorarrayVoltageStreamReleaseBuffers();
        return memErr;
    }

    s_stream.queue = xQueueCreate(CONFIG_SENSORARRAY_STREAM_QUEUE_DEPTH,
                                  sizeof(sensorarrayVoltageStreamFrame_t));
    if (!s_stream.queue) {
        sensorarrayVoltageStreamReleaseBuffers();
        return ESP_ERR_NO_MEM;
    }

    ads126xAdcFastScanOptions_t fastOptions = {
        .fastInpmuxWrite = CONFIG_SENSORARRAY_ADS_FAST_INPMUX_WRITE != 0,
        .directRead = CONFIG_SENSORARRAY_ADS_FAST_READ_DIRECT != 0,
        .statusByteEnabled = !(CONFIG_SENSORARRAY_ADS_FAST_DISABLE_STATUS_CRC != 0) &&
                             config->ads->enableStatusByte,
        .crcEnabled = !(CONFIG_SENSORARRAY_ADS_FAST_DISABLE_STATUS_CRC != 0) &&
                      (config->ads->crcMode != ADS126X_CRC_OFF),
        .fixedGain = CONFIG_SENSORARRAY_ADS_FAST_FIXED_GAIN != 0,
        .usePollingTransmit = CONFIG_SENSORARRAY_ADS_SPI_POLLING_TRANSMIT != 0,
        .acquireBusPerFrame = CONFIG_SENSORARRAY_ADS_SPI_ACQUIRE_BUS_PER_FRAME != 0,
        .drdyTimeoutUs = CONFIG_SENSORARRAY_ADS_DRDY_TIMEOUT_US_FAST,
    };

    esp_err_t err = ads126xAdcConfigureFastScan(config->ads, &fastOptions);
    if (err != ESP_OK) {
        vQueueDelete(s_stream.queue);
        s_stream.queue = NULL;
        sensorarrayVoltageStreamReleaseBuffers();
        return err;
    }

    sensorarrayRateControllerInit(&s_stream.controller,
                                  config->ads->dataRateDr,
                                  (uint8_t)CONFIG_SENSORARRAY_AUTO_RATE_MIN_ADS_DR);

    if (!config->ads->spiDmaCapable) {
        sensorarrayStatusRecord(&s_stream.status,
                                SENSORARRAY_STATUS_ADS_DMA_FALLBACK,
                                SENSORARRAY_FRAME_FLAG_ADS_ERROR);
    }

    esp_log_level_set("*", ESP_LOG_WARN);

    s_stream.running = true;
    s_stream.streamStartUs = esp_timer_get_time();
    BaseType_t ok = xTaskCreatePinnedToCore(sensorarrayVoltageOutputTask,
                                            "sensorarray_out",
                                            CONFIG_SENSORARRAY_COMM_TASK_STACK,
                                            NULL,
                                            CONFIG_SENSORARRAY_COMM_TASK_PRIORITY,
                                            &s_stream.outputTask,
                                            CONFIG_SENSORARRAY_COMM_TASK_CORE);
    if (ok != pdPASS) {
        s_stream.running = false;
        vQueueDelete(s_stream.queue);
        s_stream.queue = NULL;
        sensorarrayVoltageStreamReleaseBuffers();
        return ESP_ERR_NO_MEM;
    }

    ok = xTaskCreatePinnedToCore(sensorarrayVoltageScanTask,
                                 "sensorarray_scan",
                                 CONFIG_SENSORARRAY_SCAN_TASK_STACK,
                                 NULL,
                                 CONFIG_SENSORARRAY_SCAN_TASK_PRIORITY,
                                 &s_stream.scanTask,
                                 CONFIG_SENSORARRAY_SCAN_TASK_CORE);
    if (ok != pdPASS) {
        s_stream.running = false;
        if (s_stream.outputTask) {
            vTaskDelete(s_stream.outputTask);
            s_stream.outputTask = NULL;
        }
        vQueueDelete(s_stream.queue);
        s_stream.queue = NULL;
        sensorarrayVoltageStreamReleaseBuffers();
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

QueueHandle_t sensorarrayVoltageStreamQueue(void)
{
    return s_stream.queue;
}

sensorarrayStatusCounters_t *sensorarrayVoltageStreamStatus(void)
{
    return &s_stream.status;
}

sensorarrayPerfCounters_t *sensorarrayVoltageStreamPerf(void)
{
    return &s_stream.perf;
}

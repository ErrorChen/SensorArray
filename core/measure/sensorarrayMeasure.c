#include "sensorarrayMeasure.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_attr.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "sensorarrayBoardMap.h"
#include "sensorarrayConfig.h"
#include "sensorarrayFdcSweep.h"
#include "sensorarrayLog.h"

#ifndef CONFIG_FDC2214CAP_LOW_LEVEL_I2C_TRACE
#define CONFIG_FDC2214CAP_LOW_LEVEL_I2C_TRACE 0
#endif
#ifndef CONFIG_SENSORARRAY_FDC_MATRIX_TARGET_FPS
#define CONFIG_SENSORARRAY_FDC_MATRIX_TARGET_FPS 20
#endif
#ifndef CONFIG_SENSORARRAY_FDC_TIMING_SUMMARY_EVERY_N_FRAMES
#ifdef CONFIG_SENSORARRAY_FDC_TIMING_LOG_EVERY_N_FRAMES
#define CONFIG_SENSORARRAY_FDC_TIMING_SUMMARY_EVERY_N_FRAMES CONFIG_SENSORARRAY_FDC_TIMING_LOG_EVERY_N_FRAMES
#else
#define CONFIG_SENSORARRAY_FDC_TIMING_SUMMARY_EVERY_N_FRAMES 10
#endif
#endif
#ifndef CONFIG_SENSORARRAY_FDC_TIMING_SUMMARY_PERIOD_FRAMES
#define CONFIG_SENSORARRAY_FDC_TIMING_SUMMARY_PERIOD_FRAMES 10
#endif
#ifndef CONFIG_SENSORARRAY_FDC_TIMING_SUMMARY_AGGREGATE
#define CONFIG_SENSORARRAY_FDC_TIMING_SUMMARY_AGGREGATE 1
#endif
#ifndef CONFIG_SENSORARRAY_FDC_TIMING_OVERRUN_IMMEDIATE_LOG
#define CONFIG_SENSORARRAY_FDC_TIMING_OVERRUN_IMMEDIATE_LOG 1
#endif
#ifndef CONFIG_SENSORARRAY_FDC_TIMING_VERBOSE_PER_FRAME
#define CONFIG_SENSORARRAY_FDC_TIMING_VERBOSE_PER_FRAME 0
#endif
#ifndef CONFIG_SENSORARRAY_FDC_PROFILE_ROW_DEFAULT
#define CONFIG_SENSORARRAY_FDC_PROFILE_ROW_DEFAULT 0
#endif
#ifndef CONFIG_SENSORARRAY_FDC_PROFILE_DEVICE_DEFAULT
#define CONFIG_SENSORARRAY_FDC_PROFILE_DEVICE_DEFAULT 0
#endif
#ifndef CONFIG_SENSORARRAY_FDC_EMIT_CAP_TOTAL_PF
#define CONFIG_SENSORARRAY_FDC_EMIT_CAP_TOTAL_PF 1
#endif
#ifndef CONFIG_SENSORARRAY_FDC_TANK_INDUCTOR_NH
#define CONFIG_SENSORARRAY_FDC_TANK_INDUCTOR_NH 18000
#endif
#ifndef CONFIG_SENSORARRAY_FDC_TEXT_OUTPUT_FREQ_HZ
#define CONFIG_SENSORARRAY_FDC_TEXT_OUTPUT_FREQ_HZ 0
#endif
#ifndef CONFIG_SENSORARRAY_FDC_TEXT_OUTPUT_CAP_TOTAL_PF
#define CONFIG_SENSORARRAY_FDC_TEXT_OUTPUT_CAP_TOTAL_PF 1
#endif
#ifndef CONFIG_SENSORARRAY_FDC_TEXT_OUTPUT_BOTH_SEPARATE
#define CONFIG_SENSORARRAY_FDC_TEXT_OUTPUT_BOTH_SEPARATE 0
#endif
#ifndef CONFIG_SENSORARRAY_FDC_TEXT_OUTPUT_BOTH_INLINE_DEBUG
#define CONFIG_SENSORARRAY_FDC_TEXT_OUTPUT_BOTH_INLINE_DEBUG 0
#endif
#ifndef CONFIG_SENSORARRAY_FDC_ROW_EPOCH_RESTART_ENABLE
#define CONFIG_SENSORARRAY_FDC_ROW_EPOCH_RESTART_ENABLE 1
#endif
#ifndef CONFIG_SENSORARRAY_FDC_ROW_EPOCH_RESTART_METHOD_SLEEP
#define CONFIG_SENSORARRAY_FDC_ROW_EPOCH_RESTART_METHOD_SLEEP 1
#endif
#ifndef CONFIG_SENSORARRAY_FDC_ROW_SWITCH_SETTLE_US
#define CONFIG_SENSORARRAY_FDC_ROW_SWITCH_SETTLE_US 50
#endif
#ifndef CONFIG_SENSORARRAY_FDC_PARALLEL_DUAL_BUS_READ
#define CONFIG_SENSORARRAY_FDC_PARALLEL_DUAL_BUS_READ 1
#endif
#ifndef CONFIG_SENSORARRAY_FDC_PARALLEL_DISABLE_AFTER_TIMEOUT_IN_FRAME
#define CONFIG_SENSORARRAY_FDC_PARALLEL_DISABLE_AFTER_TIMEOUT_IN_FRAME 1
#endif
#ifndef CONFIG_SENSORARRAY_FDC_PARALLEL_RETRY_INTERVAL_FRAMES
#define CONFIG_SENSORARRAY_FDC_PARALLEL_RETRY_INTERVAL_FRAMES 10
#endif
#ifndef CONFIG_SENSORARRAY_FDC_WORKER_TASK_STACK
#define CONFIG_SENSORARRAY_FDC_WORKER_TASK_STACK 6144
#endif
#ifndef CONFIG_SENSORARRAY_FDC_WORKER_TASK_PRIO
#define CONFIG_SENSORARRAY_FDC_WORKER_TASK_PRIO CONFIG_SENSORARRAY_SCAN_TASK_PRIO
#endif
#ifndef CONFIG_SENSORARRAY_FDC_WORKER_TASK_CORE
#define CONFIG_SENSORARRAY_FDC_WORKER_TASK_CORE -1
#endif
#ifndef CONFIG_FREERTOS_UNICORE
#define CONFIG_FREERTOS_UNICORE 0
#endif
#ifndef CONFIG_SENSORARRAY_FDC_WORKER_SYNC_TIMEOUT_MS
#define CONFIG_SENSORARRAY_FDC_WORKER_SYNC_TIMEOUT_MS 25
#endif
#ifndef CONFIG_SENSORARRAY_FDC_INTB_ENABLE
#define CONFIG_SENSORARRAY_FDC_INTB_ENABLE 1
#endif
#ifndef CONFIG_SENSORARRAY_FDC_INTB1_GPIO
#define CONFIG_SENSORARRAY_FDC_INTB1_GPIO 17
#endif
#ifndef CONFIG_SENSORARRAY_FDC_INTB2_GPIO
#define CONFIG_SENSORARRAY_FDC_INTB2_GPIO 18
#endif
#ifndef CONFIG_SENSORARRAY_FDC_INTB_TRIGGER_ANYEDGE
#define CONFIG_SENSORARRAY_FDC_INTB_TRIGGER_ANYEDGE 1
#endif
#ifndef CONFIG_SENSORARRAY_FDC_INTB_WAIT_TIMEOUT_US
#define CONFIG_SENSORARRAY_FDC_INTB_WAIT_TIMEOUT_US 10000
#endif
#ifndef CONFIG_SENSORARRAY_FDC_INTB_FALLBACK_POLLING
#define CONFIG_SENSORARRAY_FDC_INTB_FALLBACK_POLLING 1
#endif
#ifndef CONFIG_SENSORARRAY_FDC_INTB_WEAK_PULLUP
#define CONFIG_SENSORARRAY_FDC_INTB_WEAK_PULLUP 0
#endif
#ifndef CONFIG_SENSORARRAY_FDC_INTB_DEBUG_LOG
#define CONFIG_SENSORARRAY_FDC_INTB_DEBUG_LOG 0
#endif
#ifndef CONFIG_SENSORARRAY_FDC_DIFF_CACHE_APPLY
#define CONFIG_SENSORARRAY_FDC_DIFF_CACHE_APPLY 1
#endif
#ifndef CONFIG_SENSORARRAY_FDC_CACHE_APPLY_VERBOSE_LOG
#define CONFIG_SENSORARRAY_FDC_CACHE_APPLY_VERBOSE_LOG 0
#endif
#ifndef CONFIG_SENSORARRAY_LOG_CACHE_APPLY_VERBOSE
#define CONFIG_SENSORARRAY_LOG_CACHE_APPLY_VERBOSE CONFIG_SENSORARRAY_FDC_CACHE_APPLY_VERBOSE_LOG
#endif
#ifndef CONFIG_SENSORARRAY_LOG_ROW_SUMMARY
#define CONFIG_SENSORARRAY_LOG_ROW_SUMMARY 0
#endif
#ifndef CONFIG_SENSORARRAY_FDC_SAMPLE_DEVICE_LOG_EVERY_N_FRAMES
#define CONFIG_SENSORARRAY_FDC_SAMPLE_DEVICE_LOG_EVERY_N_FRAMES 10
#endif
#ifndef CONFIG_SENSORARRAY_FDC_LOG_NORMAL_POLL_SUCCESS
#define CONFIG_SENSORARRAY_FDC_LOG_NORMAL_POLL_SUCCESS 0
#endif
#ifndef CONFIG_SENSORARRAY_FDC_ALLOW_SAFE_DEFAULT_FORMAL_READ
#define CONFIG_SENSORARRAY_FDC_ALLOW_SAFE_DEFAULT_FORMAL_READ 0
#endif
#ifndef CONFIG_SENSORARRAY_FDC_PARALLEL_DUAL_BUS_READ_SAFE
#define CONFIG_SENSORARRAY_FDC_PARALLEL_DUAL_BUS_READ_SAFE 1
#endif
#ifndef CONFIG_SENSORARRAY_FDC_FORCE_SINGLE_THREAD_READ
#define CONFIG_SENSORARRAY_FDC_FORCE_SINGLE_THREAD_READ 0
#endif
#ifndef CONFIG_SENSORARRAY_FDC_WARNING_REAPPLY_ONCE_PER_FINGERPRINT
#define CONFIG_SENSORARRAY_FDC_WARNING_REAPPLY_ONCE_PER_FINGERPRINT 1
#endif
#ifndef CONFIG_SENSORARRAY_FDC_AMPLITUDE_FAST_SWEEP_THRESHOLD
#define CONFIG_SENSORARRAY_FDC_AMPLITUDE_FAST_SWEEP_THRESHOLD 4
#endif
#ifndef CONFIG_SENSORARRAY_FDC_WARNING_REAPPLY_COOLDOWN_FRAMES
#define CONFIG_SENSORARRAY_FDC_WARNING_REAPPLY_COOLDOWN_FRAMES 50
#endif
#ifndef CONFIG_SENSORARRAY_FDC_WARNING_FAST_SWEEP_COOLDOWN_MS
#define CONFIG_SENSORARRAY_FDC_WARNING_FAST_SWEEP_COOLDOWN_MS 1000
#endif
#ifndef CONFIG_SENSORARRAY_FDC_VERIFY_MODE_FULL
#define CONFIG_SENSORARRAY_FDC_VERIFY_MODE_FULL 0
#endif
#ifndef CONFIG_SENSORARRAY_FDC_VERIFY_MODE_STARTUP_ONLY
#define CONFIG_SENSORARRAY_FDC_VERIFY_MODE_STARTUP_ONLY 1
#endif
#ifndef CONFIG_SENSORARRAY_FDC_VERIFY_MODE_NONE
#define CONFIG_SENSORARRAY_FDC_VERIFY_MODE_NONE 0
#endif
#ifndef CONFIG_BOARD_I2C_FREQ_HZ
#define CONFIG_BOARD_I2C_FREQ_HZ 337500
#endif

#if CONFIG_FDC2214CAP_LOW_LEVEL_I2C_TRACE
#define FDCLOW_TRACE(...) printf(__VA_ARGS__)
#else
#define FDCLOW_TRACE(...) do { } while (0)
#endif

#define SENSORARRAY_FDC_RAW_SCALE_2P28 268435456.0
#define SENSORARRAY_PI 3.14159265358979323846
#define SENSORARRAY_FDC_AUTOSCAN_RR_SEQUENCE FDC2214_RR_SEQUENCE_CH0_CH1_CH2_CH3
#define SENSORARRAY_FDC_AUTOSCAN_READY_UNREAD_MASK 0x0Fu
#define SENSORARRAY_FDC_AUTOSCAN_READY_TIMEOUT_MS CONFIG_SENSORARRAY_FDC_SWEEP_SAMPLE_TIMEOUT_MS
#define SENSORARRAY_FDC_MUX_AUTOSCAN_EN_MASK 0x8000u
#define SENSORARRAY_FDC_MUX_RR_SEQUENCE_SHIFT 13u
#define SENSORARRAY_FDC_MUX_RR_SEQUENCE_MASK 0x6000u
#define SENSORARRAY_FDC_MUX_DEGLITCH_MASK 0x0007u
#define SENSORARRAY_FDC_CONFIG_HIGH_CURRENT_DRV_MASK 0x0040u
#define SENSORARRAY_FDC_REG_RCOUNT_BASE 0x08u
#define SENSORARRAY_FDC_REG_SETTLECOUNT_BASE 0x10u
#define SENSORARRAY_FDC_REG_CLOCK_DIVIDERS_BASE 0x14u
#define SENSORARRAY_FDC_REG_STATUS 0x18u
#define SENSORARRAY_FDC_REG_STATUS_CONFIG 0x19u
#define SENSORARRAY_FDC_REG_CONFIG 0x1Au
#define SENSORARRAY_FDC_REG_MUX_CONFIG 0x1Bu
#define SENSORARRAY_FDC_REG_DRIVE_CURRENT_BASE 0x1Eu
#define SENSORARRAY_FDC_REG_DEVICE_ID 0x7Fu
#define SENSORARRAY_FDC_RAW28_SATURATED_THRESHOLD 0x0FFFFF00u
#define SENSORARRAY_FDC_AMPLITUDE_RESCUE_THRESHOLD 4u
#define SENSORARRAY_FDC_CELL_ROUTE_DISCARD_COUNT 2u
#define SENSORARRAY_FDC_TARGET_FRAME_US (1000000u / CONFIG_SENSORARRAY_FDC_MATRIX_TARGET_FPS)
#define SENSORARRAY_FDC_TARGET_ROW_US (SENSORARRAY_FDC_TARGET_FRAME_US / SENSORARRAY_MATRIX_ROWS)
#define SENSORARRAY_FDC_CONFIG_SLEEP_MODE_EN_MASK 0x2000u
#define SENSORARRAY_FDC_CONFIG_INTB_DIS_MASK 0x0080u
#define SENSORARRAY_FDC_STATUS_CONFIG_DRDY_2INT_MASK 0x0001u
#define SENSORARRAY_FDC_DRIVE_CURRENT_MASK 0xF800u
#define SENSORARRAY_FDC_MUX_FIXED_MASK 0x0208u
#define SENSORARRAY_FDC_WORKER_QUEUE_DEPTH 1u
#define SENSORARRAY_FDC_WORKER_STACK_WORDS \
    ((CONFIG_SENSORARRAY_FDC_WORKER_TASK_STACK + sizeof(StackType_t) - 1u) / sizeof(StackType_t))
#define SENSORARRAY_FDC_INVALID_CAP_SENTINEL_PF (-1.0)
#define SENSORARRAY_FDC_INVALID_FREQ_SENTINEL_HZ (-1.0)
#define SENSORARRAY_FDC_READY_MODE_NAME "polling_only"
#define SENSORARRAY_FDC_INTB_HINT_NAME "disabled"
#define SENSORARRAY_FDC_NO_UNREAD_RESYNC_THRESHOLD 3u

static SemaphoreHandle_t s_measureLock = NULL;
static portMUX_TYPE s_measureLockMux = portMUX_INITIALIZER_UNLOCKED;
static uint32_t s_fdcMatrixSequence = 0u;
static bool s_fastSpeedEnabled = false;
static uint32_t s_fdcMatrixAllInvalidSequence = 0u;
static bool s_fdcProfileSummaryEnabled = true;
static bool s_fdcProfileRowEnabled = (CONFIG_SENSORARRAY_FDC_PROFILE_ROW_DEFAULT != 0);
static bool s_fdcProfileDeviceEnabled = (CONFIG_SENSORARRAY_FDC_PROFILE_DEVICE_DEFAULT != 0);
static uint32_t s_fdcTimingSummaryEvery = CONFIG_SENSORARRAY_FDC_TIMING_SUMMARY_EVERY_N_FRAMES;
static uint8_t s_fdcDiscardFrames = (uint8_t)CONFIG_SENSORARRAY_FDC_DISCARD_FRAMES_AFTER_ROW_SWITCH;

static uint64_t sensorarrayMeasureElapsedUs(int64_t startUs);
static uint64_t sensorarrayMeasureAvgU64(uint64_t total, uint32_t count);

typedef struct {
    uint32_t raw28[4];
    bool fresh[4];
    bool valid[4];
    bool amplitudeWarning[4];
    bool freshAmplitudeWarning[4];
    bool staleAmplitudeWarning[4];
    bool transientAmplitudeWarning[4];
    bool watchdogFault[4];
    bool saturated[4];
    bool i2cError[4];
    bool notReady[4];
    bool zeroBeforeReady[4];
    bool zeroAfterDrdy[4];
    uint16_t statusRaw;
    uint8_t unreadMask;
    uint8_t freshMask;
    uint8_t validMask;
    uint8_t warnMask;
    uint8_t errorMask;
    uint8_t notReadyMask;
    uint8_t zeroBeforeReadyMask;
    uint8_t zeroAfterDrdyMask;
    bool timeout;
    bool partial;
    bool i2cTransactionError;
    bool dataReady;
    bool unreadWithoutDrdy;
} sensorarrayFdcAutoscanSamples_t;

typedef struct {
    bool validSeen[SENSORARRAY_MATRIX_ROWS][SENSORARRAY_MATRIX_COLS];
    bool invalidSeen[SENSORARRAY_MATRIX_ROWS][SENSORARRAY_MATRIX_COLS];
    bool amplitudeWarningSeen[SENSORARRAY_MATRIX_ROWS][SENSORARRAY_MATRIX_COLS];
    bool freshAmplitudeWarningSeen[SENSORARRAY_MATRIX_ROWS][SENSORARRAY_MATRIX_COLS];
    bool staleAmplitudeWarningSeen[SENSORARRAY_MATRIX_ROWS][SENSORARRAY_MATRIX_COLS];
    bool transientAmplitudeWarningSeen[SENSORARRAY_MATRIX_ROWS][SENSORARRAY_MATRIX_COLS];
    bool watchdogSeen[SENSORARRAY_MATRIX_ROWS][SENSORARRAY_MATRIX_COLS];
    bool saturatedSeen[SENSORARRAY_MATRIX_ROWS][SENSORARRAY_MATRIX_COLS];
    bool zeroRawSeen[SENSORARRAY_MATRIX_ROWS][SENSORARRAY_MATRIX_COLS];
    bool notReadySeen[SENSORARRAY_MATRIX_ROWS][SENSORARRAY_MATRIX_COLS];
    bool zeroBeforeReadySeen[SENSORARRAY_MATRIX_ROWS][SENSORARRAY_MATRIX_COLS];
    bool zeroAfterDrdySeen[SENSORARRAY_MATRIX_ROWS][SENSORARRAY_MATRIX_COLS];
    bool placeholderZeroSeen[SENSORARRAY_MATRIX_ROWS][SENSORARRAY_MATRIX_COLS];
    bool i2cErrorSeen[SENSORARRAY_MATRIX_ROWS][SENSORARRAY_MATRIX_COLS];
    uint32_t lastRaw28[SENSORARRAY_MATRIX_ROWS][SENSORARRAY_MATRIX_COLS];
    double lastFreqHz[SENSORARRAY_MATRIX_ROWS][SENSORARRAY_MATRIX_COLS];
    uint16_t clockDividers[SENSORARRAY_MATRIX_ROWS][SENSORARRAY_MATRIX_COLS];
    uint16_t driveCurrent[SENSORARRAY_MATRIX_ROWS][SENSORARRAY_MATRIX_COLS];
    uint8_t deglitchCode[SENSORARRAY_MATRIX_ROWS][SENSORARRAY_MATRIX_COLS];
    uint32_t effectiveFclkHz[SENSORARRAY_MATRIX_ROWS][SENSORARRAY_MATRIX_COLS];
} sensorarrayFdcFrameHealth_t;

typedef struct {
    bool valid;
    uint16_t rCount;
    uint16_t settleCount;
    uint16_t clockDividers;
    uint16_t driveCurrent;
    uint8_t deglitchCode;
    uint32_t effectiveFclkHz;
} sensorarrayFdcRuntimeChannelConfig_t;

typedef enum {
    SENSORARRAY_FDC_READY_NONE = 0,
    SENSORARRAY_FDC_READY_EDGE_WAKE,
    SENSORARRAY_FDC_READY_POLL_FULL,
    SENSORARRAY_FDC_READY_POLL_PARTIAL,
    SENSORARRAY_FDC_READY_TIMEOUT_PARTIAL,
    SENSORARRAY_FDC_READY_TIMEOUT_NONE,
    SENSORARRAY_FDC_READY_I2C_ERROR,
} sensorarrayFdcReadyKind_t;

typedef struct {
    uint16_t statusRaw;
    uint8_t unreadMask;
    bool dataReady;
    bool unreadFull;
    bool readyForDataRead;
    bool unreadWithoutDataReady;
} sensorarrayFdcReadyDecoded_t;

typedef struct {
    bool ready;
    bool dataReady;
    bool unreadFull;
    bool readyForDataRead;
    bool unreadWithoutDataReady;
    sensorarrayFdcReadyKind_t kind;
    uint16_t statusRaw;
    uint16_t errorStatus;
    uint8_t unreadMask;
    uint8_t drdy;
    uint8_t hadEdge;
    uint32_t edgeDelta;
    int initialIntbLevel;
    int finalIntbLevel;
    esp_err_t err;
    uint32_t pollCount;
    uint32_t unreadWithoutDrdyCount;
    uint32_t timeoutCount;
    uint32_t waitUs;
    bool timeout;
    bool partial;
    bool i2cError;
    const char *diagnostic;
    uint8_t requiredUnreadMask;
    uint32_t estimatedRoundUs;
} sensorarrayFdcReadyState_t;

typedef struct {
    uint32_t raw28[4];
    double freqHz[4];
    double capTotalPf[4];
    uint8_t freshMask4;
    uint8_t validMask4;
    uint8_t warnMask4;
    uint8_t errorMask4;
    uint8_t notReadyMask4;
    uint8_t zeroBeforeReadyMask4;
    uint8_t zeroAfterDrdyMask4;
    uint16_t status;
    uint16_t errorStatus;
    uint8_t unreadMask4;
    uint8_t drdy;
    esp_err_t readErr;
    esp_err_t i2cErr;
    sensorarrayFdcReadyKind_t readyKind;
    bool timeout;
    bool partial;
    bool i2cError;
    bool unreadWithoutDrdy;
    bool rawAllZero;
    bool staleRejected;
    uint32_t waitUs;
    uint32_t readUs;
    uint32_t pollCount;
    uint32_t edgeDelta;
} sensorarrayFdcDeviceRead4Result_t;

typedef struct {
    esp_err_t err;
    sensorarrayFdcReadyState_t ready;
    sensorarrayFdcDeviceRead4Result_t read4;
    uint8_t row;
    sensorarrayFdcDeviceId_t devId;
    uint32_t epochId;
} sensorarrayFdcWorkerResult_t;

typedef struct {
    uint32_t frameSeq;
    uint8_t row;
    uint32_t epochId;
    sensorarrayFdcDeviceId_t devId;
    bool jobQueued;
    bool sleepAckReceived;
    bool startGiven;
    bool runStarted;
    bool runCompleted;
    bool doneReceived;
    bool lateDone;
    bool staleResultDiscarded;
    uint32_t queueSendUs;
    uint32_t sleepAckWaitUs;
    uint32_t startWaitUs;
    uint32_t doneWaitUs;
    uint32_t workerRunUs;
    uint32_t waitWorkerIdleAfterTimeoutUs;
    esp_err_t err;
} sensorarrayFdcWorkerTrace_t;

typedef struct {
    uint32_t frameSeq;
    bool okRead[SENSORARRAY_MATRIX_ROWS][2];
    uint32_t duplicateReadCount;
} sensorarrayFdcFrameReadTracker_t;

typedef struct {
    sensorarrayState_t *state;
    uint32_t frameSeq;
    uint8_t row;
    uint32_t epochId;
    sensorarrayFdcAutoscanSamples_t *outSamples;
    sensorarrayFdcRuntimeChannelConfig_t *outConfigs;
    sensorarrayFdcDeviceTiming_t *timing;
    sensorarrayFdcWorkerResult_t *result;
    sensorarrayFdcWorkerTrace_t *trace;
    sensorarrayFdcFrameReadTracker_t *readTracker;
} sensorarrayFdcWorkerJob_t;

typedef struct {
    bool initialized;
    bool intbReady;
    bool intbIsrAttached;
    sensorarrayFdcDeviceId_t devId;
    int intbGpio;
    QueueHandle_t queue;
    StaticQueue_t queueStorage;
    uint8_t queueBuffer[SENSORARRAY_FDC_WORKER_QUEUE_DEPTH * sizeof(sensorarrayFdcWorkerJob_t)];
    SemaphoreHandle_t sleepAck;
    StaticSemaphore_t sleepAckStorage;
    SemaphoreHandle_t start;
    StaticSemaphore_t startStorage;
    SemaphoreHandle_t done;
    StaticSemaphore_t doneStorage;
    TaskHandle_t task;
    StaticTask_t taskStorage;
    StackType_t stack[SENSORARRAY_FDC_WORKER_STACK_WORDS];
    volatile uint32_t currentEpoch;
    volatile uint32_t edgeCount;
    volatile uint32_t falseEdgeCount;
    volatile uint32_t timeoutCount;
    volatile uint32_t fallbackPollCount;
    volatile uint32_t freshDrdyCount;
    volatile uint32_t staleBeforeClearCount;
    volatile int lastLevel;
    volatile int64_t lastEdgeUs;
    volatile uint32_t lastEpochSeen;
    volatile TaskHandle_t waitTask;
} sensorarrayFdcWorkerContext_t;

typedef struct {
    bool active;
    uint32_t frames;
    uint32_t seqStart;
    uint32_t seqEnd;
    uint64_t frameUsTotal;
    uint64_t frameUsMin;
    uint64_t frameUsMax;
    uint64_t rowUsTotal;
    uint64_t rowUsMax;
    uint8_t slowRow;
    uint32_t overrunCount;
    sensorarrayFdcTimingSummary_t totals;
} sensorarrayFdcTimingAggregate_t;

static sensorarrayFdcWorkerContext_t s_fdcWorkers[2] = {
    {.devId = SENSORARRAY_FDC_DEV_PRIMARY, .intbGpio = CONFIG_SENSORARRAY_FDC_INTB1_GPIO},
    {.devId = SENSORARRAY_FDC_DEV_SECONDARY, .intbGpio = CONFIG_SENSORARRAY_FDC_INTB2_GPIO},
};
static bool s_fdcWorkersInitAttempted = false;
static bool s_fdcWorkersAvailable = false;
static bool s_fdcFormalPrecheckDone = false;
static bool s_fdcSecondaryUnavailableLogged = false;
static bool s_fdcParallelConfigLogged = false;
static uint32_t s_fdcRowEpoch = 0u;
static uint32_t s_fdcParallelCooldownFrames = 0u;
static uint32_t s_fdcNoUnreadConsecutive[2] = {0u, 0u};
static sensorarrayFdcTimingAggregate_t s_fdcTimingAggregate = {0};
static SemaphoreHandle_t s_fdcGpioIsrServiceMutex = NULL;
static portMUX_TYPE s_fdcGpioIsrServiceMux = portMUX_INITIALIZER_UNLOCKED;
static bool s_fdcGpioIsrServiceInstalled = false;

static esp_err_t sensorarrayMeasureEnsureLock(void)
{
    if (s_measureLock) {
        return ESP_OK;
    }

    portENTER_CRITICAL(&s_measureLockMux);
    if (!s_measureLock) {
        s_measureLock = xSemaphoreCreateMutex();
    }
    portEXIT_CRITICAL(&s_measureLockMux);

    return s_measureLock ? ESP_OK : ESP_ERR_NO_MEM;
}

static esp_err_t sensorarrayMeasureTakeLock(void)
{
    esp_err_t err = sensorarrayMeasureEnsureLock();
    if (err != ESP_OK) {
        return err;
    }

    TickType_t ticks = pdMS_TO_TICKS((uint32_t)CONFIG_SENSORARRAY_MEASURE_LOCK_TIMEOUT_MS);
    if (ticks == 0) {
        ticks = 1;
    }
    return (xSemaphoreTake(s_measureLock, ticks) == pdTRUE) ? ESP_OK : ESP_ERR_TIMEOUT;
}

static void sensorarrayMeasureGiveLock(void)
{
    if (s_measureLock) {
        xSemaphoreGive(s_measureLock);
    }
}

static void sensorarrayDelayMs(uint32_t delayMs)
{
    if (delayMs > 0u) {
        vTaskDelay(pdMS_TO_TICKS(delayMs));
    }
}

static const sensorarrayAdsReadPolicy_t *sensorarrayReadPolicyOrDefault(const sensorarrayAdsReadPolicy_t *policy)
{
    static const sensorarrayAdsReadPolicy_t kDefaultPolicy = {
        .stopBeforeMuxChange = (CONFIG_SENSORARRAY_ADS_READ_STOP1_BEFORE_MUX != 0),
        .settleAfterMuxMs = (uint32_t)CONFIG_SENSORARRAY_ADS_READ_SETTLE_AFTER_MUX_MS,
        .startEveryRead = (CONFIG_SENSORARRAY_ADS_READ_START1_EVERY_READ != 0),
        .baseDiscardCount = (uint8_t)CONFIG_SENSORARRAY_ADS_READ_BASE_DISCARD_COUNT,
        .readRetryCount = (uint8_t)CONFIG_SENSORARRAY_ADS_READ_RETRY_COUNT,
    };
    return policy ? policy : &kDefaultPolicy;
}

static const char *sensorarrayMatrixDSourceName(sensorarrayMatrixDSourcePolicy_t dSource)
{
    return (dSource == SENSORARRAY_MATRIX_D_SOURCE_REF) ? "REF" : "GND";
}

static const char *sensorarrayAdsIntRefPolicyName(sensorarrayAdsIntRefPolicy_t policy)
{
    switch (policy) {
    case SENSORARRAY_ADS_INTREF_ON:
        return "ON";
    case SENSORARRAY_ADS_INTREF_KEEP:
        return "KEEP";
    case SENSORARRAY_ADS_INTREF_OFF:
    default:
        return "OFF";
    }
}

static const char *sensorarrayAdsVbiasPolicyName(sensorarrayAdsVbiasPolicy_t policy)
{
    switch (policy) {
    case SENSORARRAY_ADS_VBIAS_ON:
        return "ON";
    case SENSORARRAY_ADS_VBIAS_KEEP:
        return "KEEP";
    case SENSORARRAY_ADS_VBIAS_OFF:
    default:
        return "OFF";
    }
}

static bool sensorarrayAdsIntRefPolicyUpdates(sensorarrayAdsIntRefPolicy_t policy)
{
    return policy != SENSORARRAY_ADS_INTREF_KEEP;
}

static bool sensorarrayAdsVbiasPolicyUpdates(sensorarrayAdsVbiasPolicy_t policy)
{
    return policy != SENSORARRAY_ADS_VBIAS_KEEP;
}

static int sensorarrayReadResetGpioLevel(void)
{
    if (CONFIG_BOARD_ADS126X_RESET_GPIO < 0) {
        return -1;
    }
    return gpio_get_level((gpio_num_t)CONFIG_BOARD_ADS126X_RESET_GPIO);
}

esp_err_t sensorarrayMeasureApplyRefPolicy(sensorarrayState_t *state,
                                           const char *stage,
                                           const char *mode,
                                           sensorarrayMatrixDSourcePolicy_t dSource,
                                           sensorarrayAdsIntRefPolicy_t intrefPolicy,
                                           sensorarrayAdsVbiasPolicy_t vbiasPolicy,
                                           const char *reason)
{
    if (!state) {
        return ESP_ERR_INVALID_ARG;
    }

    const bool updateIntref = sensorarrayAdsIntRefPolicyUpdates(intrefPolicy);
    const bool updateVbias = sensorarrayAdsVbiasPolicyUpdates(vbiasPolicy);
    const bool enableIntref = (intrefPolicy == SENSORARRAY_ADS_INTREF_ON);
    const bool enableVbias = (vbiasPolicy == SENSORARRAY_ADS_VBIAS_ON);

    uint8_t powerBefore = 0u;
    uint8_t powerAfter = 0u;
    bool havePower = false;
    esp_err_t err = ESP_OK;
    const char *status = "ok";

    if (!state->adsReady) {
        err = (updateIntref || updateVbias) ? ESP_ERR_INVALID_STATE : ESP_OK;
        status = (err == ESP_OK) ? "ads_unavailable_keep" : "ads_unavailable";
    } else if (updateIntref || updateVbias) {
        err = ads126xAdcApplyPowerPolicy(&state->ads,
                                         updateIntref,
                                         enableIntref,
                                         updateVbias,
                                         enableVbias,
                                         &powerBefore,
                                         &powerAfter);
        havePower = (err == ESP_OK);
        status = (err == ESP_OK) ? "ok" : "power_policy_error";
    } else {
        err = ads126xAdcReadPowerRegister(&state->ads, &powerBefore);
        if (err == ESP_OK) {
            powerAfter = powerBefore;
            havePower = true;
            status = "keep";
        } else {
            status = "read_power_error";
        }
    }

    if (err == ESP_OK) {
        if (intrefPolicy == SENSORARRAY_ADS_INTREF_OFF) {
            state->adsRefReady = false;
        } else if (intrefPolicy == SENSORARRAY_ADS_INTREF_ON) {
            state->adsRefReady = true;
        }
        sensorarrayLogSetAdsState(state->adsReady, state->adsRefReady);
    }

    char beforeBuf[8];
    char afterBuf[8];
    char resetBuf[8];
    printf("DBGREFPOLICY,stage=%s,mode=%s,dSource=%s,swReq=%s,intrefReq=%s,vbiasReq=%s,powerBefore=%s,"
           "powerAfter=%s,noDelay=1,adsReset=%s,reason=%s,err=%ld,status=%s\n",
           stage ? stage : SENSORARRAY_NA,
           mode ? mode : SENSORARRAY_NA,
           sensorarrayMatrixDSourceName(dSource),
           (dSource == SENSORARRAY_MATRIX_D_SOURCE_REF) ? sensorarrayLogSwSourceName(TMUX1108_SOURCE_REF)
                                                        : sensorarrayLogSwSourceName(TMUX1108_SOURCE_GND),
           sensorarrayAdsIntRefPolicyName(intrefPolicy),
           sensorarrayAdsVbiasPolicyName(vbiasPolicy),
           sensorarrayLogFmtHexU8(beforeBuf, sizeof(beforeBuf), havePower, powerBefore),
           sensorarrayLogFmtHexU8(afterBuf, sizeof(afterBuf), havePower, powerAfter),
           sensorarrayLogFmtGpioLevel(resetBuf, sizeof(resetBuf), true, sensorarrayReadResetGpioLevel()),
           reason ? reason : SENSORARRAY_NA,
           (long)err,
           status);
    return err;
}

static esp_err_t sensorarrayMeasureStopAdsBeforeRoute(sensorarrayState_t *state)
{
    if (!state || !state->adsReady || !state->adsAdc1Running) {
        return ESP_OK;
    }

    esp_err_t err = ads126xAdcStopAdc1(&state->ads);
    if (err == ESP_OK) {
        state->adsAdc1Running = false;
    }
    return err;
}

static tmux1108Source_t sensorarrayMeasureSourceForSwPhysicalLevel(sensorarraySwPhysicalLevel_t level)
{
    const bool refSourceIsHigh = (CONFIG_TMUX1108_SW_REF_LEVEL != 0);
    if (level == SENSORARRAY_SW_PHYSICAL_HIGH) {
        return refSourceIsHigh ? TMUX1108_SOURCE_REF : TMUX1108_SOURCE_GND;
    }
    return refSourceIsHigh ? TMUX1108_SOURCE_GND : TMUX1108_SOURCE_REF;
}

esp_err_t sensorarrayMeasureSetSwPhysicalLevel(sensorarrayState_t *state,
                                               sensorarraySwPhysicalLevel_t level,
                                               const char *reason)
{
    (void)reason;
    if (!state || !state->tmuxReady) {
        return ESP_ERR_INVALID_STATE;
    }
    if (level != SENSORARRAY_SW_PHYSICAL_LOW && level != SENSORARRAY_SW_PHYSICAL_HIGH) {
        return ESP_ERR_INVALID_ARG;
    }

    return tmuxSwitchSet1108Source(sensorarrayMeasureSourceForSwPhysicalLevel(level));
}

static void sensorarrayMeasureDelayUs(uint32_t delayUs)
{
    if (delayUs > 0u) {
        esp_rom_delay_us(delayUs);
    }
}

static esp_err_t sensorarrayMeasureSetSelaPathQuiet(sensorarrayState_t *state,
                                                    sensorarraySelaRoute_t selaRoute,
                                                    uint32_t settleDelayUs)
{
    if (!state || !state->tmuxReady) {
        return ESP_ERR_INVALID_STATE;
    }

    int selaWriteLevel = -1;
    if (!sensorarrayBoardMapSelaRouteToGpioLevel(selaRoute, &selaWriteLevel)) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = tmux1134SelectSelALevel(selaWriteLevel != 0);
    if (err == ESP_OK) {
        sensorarrayMeasureDelayUs(settleDelayUs);
    }
    return err;
}

static esp_err_t sensorarrayMeasureSetFdcSelBPathQuiet(sensorarrayState_t *state)
{
    if (!state || !state->tmuxReady) {
        return ESP_ERR_INVALID_STATE;
    }

    bool selBLevel = false;
    if (!sensorarrayBoardMapFdcSelBLevel(&selBLevel)) {
        return ESP_ERR_INVALID_ARG;
    }
    return tmux1134SelectSelBLevel(selBLevel);
}

static esp_err_t sensorarrayMeasureForceAdsReferenceOff(sensorarrayState_t *state)
{
    if (!state || !state->adsReady) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = ads126xAdcApplyPowerPolicy(&state->ads,
                                               true,
                                               false,
                                               true,
                                               false,
                                               NULL,
                                               NULL);
    if (err == ESP_OK) {
        state->adsRefReady = false;
        sensorarrayLogSetAdsState(state->adsReady, state->adsRefReady);
    }
    return err;
}

static int sensorarrayMeasureSwPhysicalReadbackFromControl(const tmuxSwitchControlState_t *ctrl)
{
    return ctrl ? ctrl->obsSwLevel : -1;
}

static void sensorarrayMeasureReadFdcPathControl(tmuxSwitchControlState_t *ctrl)
{
    if (!ctrl || tmuxSwitchGetControlState(ctrl) != ESP_OK) {
        if (ctrl) {
            *ctrl = (tmuxSwitchControlState_t){
                .cmdRow = 0xFFu,
                .cmdSwLevel = -1,
                .cmdSelaLevel = -1,
                .cmdSelbLevel = -1,
                .cmdEnLevel = -1,
                .obsSwLevel = -1,
                .obsSelaLevel = -1,
                .obsSelbLevel = -1,
                .obsEnLevel = -1,
            };
        }
    }
}

static bool sensorarrayMeasureFdcPathControlMatches(const tmuxSwitchControlState_t *ctrl)
{
    if (!ctrl) {
        return false;
    }

    int expectedSela = -1;
    bool expectedSelb = false;
    if (!sensorarrayBoardMapSelaRouteToGpioLevel(SENSORARRAY_SELA_ROUTE_FDC2214, &expectedSela) ||
        !sensorarrayBoardMapFdcSelBLevel(&expectedSelb)) {
        return false;
    }

    int expectedSw = CONFIG_TMUX1108_SW_REF_LEVEL ? 0 : 1;
    bool swOk = ctrl->cmdSwLevel == expectedSw &&
                (ctrl->obsSwLevel < 0 || ctrl->obsSwLevel == expectedSw);
    bool selaOk = ctrl->cmdSelaLevel == expectedSela &&
                  (ctrl->obsSelaLevel < 0 || ctrl->obsSelaLevel == expectedSela);
    bool selbOk = ctrl->cmdSelbLevel == (expectedSelb ? 1 : 0) &&
                  (ctrl->obsSelbLevel < 0 || ctrl->obsSelbLevel == (expectedSelb ? 1 : 0));
    bool enOk = ctrl->cmdTmux1134EnLogicalOn &&
                (!ctrl->obsTmux1134EnLogicalOnValid || ctrl->obsTmux1134EnLogicalOn);
    return swOk && selaOk && selbOk && enOk;
}

esp_err_t sensorarrayMeasurePrepareFdcMatrixPath(sensorarrayState_t *state, const char *reason)
{
    const char *source = reason ? reason : SENSORARRAY_NA;
    bool logNormal = CONFIG_SENSORARRAY_FDC_VERBOSE_SCAN_LOG;
    if (!state || !state->tmuxReady || !state->adsReady) {
        printf("FDC_PATH,stage=prepare_done,reason=%s,ok=0,err=0x%lx,sw=-1,sela=-1,selb=-1,en=-1,adsRef=-1,adsVbias=-1\n",
               source,
               (unsigned long)ESP_ERR_INVALID_STATE);
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = sensorarrayMeasureStopAdsBeforeRoute(state);
    if (logNormal || err != ESP_OK) {
        printf("FDC_PATH,stage=ads_stop,reason=%s,err=0x%lx\n", source, (unsigned long)err);
    }
    if (err != ESP_OK) {
        return err;
    }
#if CONFIG_SENSORARRAY_ADS1263
    esp_err_t adc2Err = ads126xAdcStopAdc2(&state->ads);
    if (logNormal || (adc2Err != ESP_OK && adc2Err != ESP_ERR_NOT_SUPPORTED)) {
        printf("FDC_PATH,stage=ads_stop2,reason=%s,err=0x%lx\n", source, (unsigned long)adc2Err);
    }
    if (adc2Err != ESP_OK && adc2Err != ESP_ERR_NOT_SUPPORTED) {
        return adc2Err;
    }
#endif

    err = sensorarrayMeasureForceAdsReferenceOff(state);
    if (logNormal || err != ESP_OK) {
        printf("FDC_PATH,stage=ads_ref_off,reason=%s,err=0x%lx,adsRefReady=%d\n",
               source,
               (unsigned long)err,
               state->adsRefReady ? 1 : 0);
        printf("FDC_PATH,stage=ads_vbias_off,reason=%s,err=0x%lx\n", source, (unsigned long)err);
    }
    if (err != ESP_OK) {
        return err;
    }

    err = sensorarrayMeasureSetSelaPathQuiet(state,
                                             SENSORARRAY_SELA_ROUTE_FDC2214,
                                             (uint32_t)CONFIG_SENSORARRAY_FDC_MATRIX_SETTLE_US);
    tmuxSwitchControlState_t ctrl = {0};
    sensorarrayMeasureReadFdcPathControl(&ctrl);
    int fdcSelaLevel = -1;
    (void)sensorarrayBoardMapSelaRouteToGpioLevel(SENSORARRAY_SELA_ROUTE_FDC2214, &fdcSelaLevel);
    if (logNormal || err != ESP_OK) {
        printf("FDC_PATH,stage=tmux1134_fdc,reason=%s,selaCmd=%d,selaReadback=%d,err=0x%lx\n",
               source,
               fdcSelaLevel,
               ctrl.obsSelaLevel,
               (unsigned long)err);
    }
    if (err != ESP_OK) {
        return err;
    }

    err = sensorarrayMeasureSetFdcSelBPathQuiet(state);
    sensorarrayMeasureReadFdcPathControl(&ctrl);
    bool fdcSelbLevel = false;
    (void)sensorarrayBoardMapFdcSelBLevel(&fdcSelbLevel);
    if (logNormal || err != ESP_OK) {
        printf("FDC_PATH,stage=selb_fdc,reason=%s,selbCmd=%d,selbReadback=%d,err=0x%lx\n",
               source,
               fdcSelbLevel ? 1 : 0,
               ctrl.obsSelbLevel,
               (unsigned long)err);
    }
    if (err != ESP_OK) {
        return err;
    }

    err = tmuxSwitchSet1108Source(TMUX1108_SOURCE_GND);
    sensorarrayMeasureReadFdcPathControl(&ctrl);
    if (logNormal || err != ESP_OK) {
        printf("FDC_PATH,stage=sw_gnd,reason=%s,swCmd=GND,swReadback=%d,err=0x%lx\n",
               source,
               sensorarrayMeasureSwPhysicalReadbackFromControl(&ctrl),
               (unsigned long)err);
    }
    if (err != ESP_OK) {
        return err;
    }

    err = tmux1134SetEnLogicalState(true);
    sensorarrayMeasureReadFdcPathControl(&ctrl);
    if (logNormal || err != ESP_OK) {
        printf("FDC_PATH,stage=tmux1108_enable,reason=%s,enCmd=1,enReadback=%d,err=0x%lx\n",
               source,
               ctrl.obsTmux1134EnLogicalOnValid ? (ctrl.obsTmux1134EnLogicalOn ? 1 : 0) : (ctrl.cmdTmux1134EnLogicalOn ? 1 : 0),
               (unsigned long)err);
    }

    bool ok = (err == ESP_OK) && sensorarrayMeasureFdcPathControlMatches(&ctrl);
    if (logNormal || !ok) {
        printf("FDC_PATH,stage=prepare_done,reason=%s,ok=%d,err=0x%lx,sw=%d,sela=%d,selb=%d,en=%d,adsRef=%d,adsVbias=0\n",
               source,
               ok ? 1 : 0,
               (unsigned long)(ok ? ESP_OK : ESP_ERR_INVALID_STATE),
               sensorarrayMeasureSwPhysicalReadbackFromControl(&ctrl),
               ctrl.obsSelaLevel,
               ctrl.obsSelbLevel,
               ctrl.obsTmux1134EnLogicalOnValid ? (ctrl.obsTmux1134EnLogicalOn ? 1 : 0) : (ctrl.cmdTmux1134EnLogicalOn ? 1 : 0),
               state->adsRefReady ? 1 : 0);
    }
    return ok ? ESP_OK : ((err != ESP_OK) ? err : ESP_ERR_INVALID_STATE);
}

esp_err_t sensorarrayMeasureEnsureFdcMatrixPath(sensorarrayState_t *state, const char *reason)
{
    if (!state) {
        return ESP_ERR_INVALID_ARG;
    }

    tmuxSwitchControlState_t ctrl = {0};
    sensorarrayMeasureReadFdcPathControl(&ctrl);
    if (!sensorarrayMeasureFdcPathControlMatches(&ctrl) || state->adsRefReady) {
        if (state->adsRefReady) {
            printf("FDC_PATH,stage=ensure_mismatch,reason=%s,field=adsRef,cmd=0,readback=%d\n",
                   reason ? reason : SENSORARRAY_NA,
                   state->adsRefReady ? 1 : 0);
        }
        if (!sensorarrayMeasureFdcPathControlMatches(&ctrl)) {
            printf("FDC_PATH,stage=ensure_mismatch,reason=%s,field=tmux_path,swCmd=%d,swReadback=%d,selaCmd=%d,selaReadback=%d,selbCmd=%d,selbReadback=%d,enCmd=%d,enReadback=%d\n",
                   reason ? reason : SENSORARRAY_NA,
                   ctrl.cmdSwLevel,
                   ctrl.obsSwLevel,
                   ctrl.cmdSelaLevel,
                   ctrl.obsSelaLevel,
                   ctrl.cmdSelbLevel,
                   ctrl.obsSelbLevel,
                   ctrl.cmdTmux1134EnLogicalOn ? 1 : 0,
                   ctrl.obsTmux1134EnLogicalOnValid ? (ctrl.obsTmux1134EnLogicalOn ? 1 : 0) : -1);
        }
        return sensorarrayMeasurePrepareFdcMatrixPath(state, reason);
    }

#if CONFIG_SENSORARRAY_FDC_VERBOSE_SCAN_LOG
    printf("FDC_PATH,stage=ensure_ok,reason=%s,sw=%d,sela=%d,selb=%d,en=%d,adsRef=%d\n",
           reason ? reason : SENSORARRAY_NA,
           sensorarrayMeasureSwPhysicalReadbackFromControl(&ctrl),
           ctrl.obsSelaLevel,
           ctrl.obsSelbLevel,
           ctrl.obsTmux1134EnLogicalOnValid ? (ctrl.obsTmux1134EnLogicalOn ? 1 : 0) : (ctrl.cmdTmux1134EnLogicalOn ? 1 : 0),
           state->adsRefReady ? 1 : 0);
#endif
    return ESP_OK;
}

static esp_err_t sensorarrayMeasureSetSwForRoute(sensorarrayState_t *state,
                                                 const char *stage,
                                                 uint8_t sColumn,
                                                 uint8_t dLine,
                                                 sensorarrayRoutePathKind_t path,
                                                 tmux1108Source_t swSource,
                                                 sensorarraySelaRoute_t selaRoute,
                                                 bool selBLevel,
                                                 const char *label,
                                                 const char *status,
                                                 const char *reason)
{
    if (!state || !state->tmuxReady) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = tmuxSwitchSet1108Source(swSource);
    sensorarrayLogRouteStepEx(stage,
                              label,
                              sColumn,
                              dLine,
                              path,
                              swSource,
                              selaRoute,
                              selBLevel,
                              err,
                              status,
                              reason);
    return err;
}

static esp_err_t sensorarrayMeasureWriteSela(sensorarrayState_t *state,
                                             sensorarraySelaRoute_t requestRoute,
                                             uint32_t settleDelayMs,
                                             const char *stage,
                                             const char *label)
{
    (void)state;

    int selaWriteLevel = -1;
    if (!sensorarrayBoardMapSelaRouteToGpioLevel(requestRoute, &selaWriteLevel)) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = tmux1134SelectSelALevel(selaWriteLevel != 0);
    int selaCmdLevel = -1;
    int selaObsLevel = -1;
    bool obsResolvedValid = false;
    sensorarraySelaRoute_t obsResolvedRoute = SENSORARRAY_SELA_ROUTE_ADS1263;

    tmuxSwitchControlState_t ctrl = {0};
    if (tmuxSwitchGetControlState(&ctrl) == ESP_OK) {
        selaCmdLevel = ctrl.cmdSelaLevel;
        selaObsLevel = ctrl.obsSelaLevel;
        obsResolvedValid = sensorarrayBoardMapSelaRouteFromGpioLevel(selaObsLevel, &obsResolvedRoute);
    }

    sensorarrayLogSelaRouteDecision(stage,
                                    label,
                                    requestRoute,
                                    selaWriteLevel,
                                    selaCmdLevel,
                                    selaObsLevel,
                                    obsResolvedValid,
                                    obsResolvedRoute);
    if (selaObsLevel >= 0 && selaObsLevel != selaWriteLevel) {
        sensorarrayLogSelaReadbackMismatch(stage, label, selaWriteLevel, selaObsLevel);
    }
    if (err != ESP_OK) {
        return err;
    }

    // SELA changes the TMUX1134 analog branch; allow a short conservative settle
    // window before the next ADS/FDC access touches the newly selected path.
    sensorarrayDelayMs(settleDelayMs);
    return ESP_OK;
}

sensorarrayFdcDeviceState_t *sensorarrayMeasureGetFdcState(sensorarrayState_t *state,
                                                            sensorarrayFdcDeviceId_t devId)
{
    if (!state) {
        return NULL;
    }

    switch (devId) {
    case SENSORARRAY_FDC_DEV_PRIMARY:
        return &state->fdcPrimary;
    case SENSORARRAY_FDC_DEV_SECONDARY:
        return &state->fdcSecondary;
    default:
        return NULL;
    }
}

sensorarrayFdcDeviceState_t *sensorarrayMeasureGetFdcStateForDLine(sensorarrayState_t *state,
                                                                    uint8_t dLine,
                                                                    const sensorarrayFdcDLineMap_t **outMap)
{
    const sensorarrayFdcDLineMap_t *map = sensorarrayBoardMapFindFdcByDLine(dLine);
    if (outMap) {
        *outMap = map;
    }
    if (!map) {
        return NULL;
    }

    return sensorarrayMeasureGetFdcState(state, map->devId);
}

static const char *sensorarrayMeasureFdcDeviceName(sensorarrayFdcDeviceId_t devId)
{
    return (devId == SENSORARRAY_FDC_DEV_SECONDARY) ? "secondary" : "primary";
}

static const char *sensorarrayMeasureEspErrName(esp_err_t err)
{
    switch (err) {
    case ESP_OK:
        return "ESP_OK";
    case ESP_ERR_TIMEOUT:
        return "ESP_ERR_TIMEOUT";
    case ESP_ERR_INVALID_ARG:
        return "ESP_ERR_INVALID_ARG";
    case ESP_ERR_INVALID_STATE:
        return "ESP_ERR_INVALID_STATE";
    case ESP_ERR_NO_MEM:
        return "ESP_ERR_NO_MEM";
    case ESP_ERR_NOT_SUPPORTED:
        return "ESP_ERR_NOT_SUPPORTED";
    case ESP_ERR_INVALID_RESPONSE:
        return "ESP_ERR_INVALID_RESPONSE";
    default:
        return "ESP_ERR_OTHER";
    }
}

bool sensorarrayMeasureDecodeMatrixIndex(uint8_t matrixIndex,
                                          uint8_t *outSColumn,
                                          uint8_t *outDLine)
{
    if (matrixIndex >= SENSORARRAY_MATRIX_CELL_COUNT || !outSColumn || !outDLine) {
        return false;
    }

    *outSColumn = (uint8_t)((matrixIndex / SENSORARRAY_MATRIX_COLS) + 1u);
    *outDLine = (uint8_t)((matrixIndex % SENSORARRAY_MATRIX_COLS) + 1u);
    return true;
}

bool sensorarrayMeasureMakeFdcCellTarget(sensorarrayState_t *state,
                                         uint8_t sColumn,
                                         uint8_t dLine,
                                         sensorarrayFdcCellTarget_t *outTarget)
{
    (void)state;
    if (!outTarget || !sensorarrayMatrixIndexIsValid(sColumn, dLine)) {
        return false;
    }

    const sensorarrayFdcDLineMap_t *map = sensorarrayBoardMapFindFdcByDLine(dLine);
    if (!map || map->channel > FDC2214_CH3) {
        return false;
    }

    *outTarget = (sensorarrayFdcCellTarget_t){
        .sColumn = sColumn,
        .dLine = dLine,
        .matrixIndex = (uint8_t)sensorarrayMatrixIndex(sColumn, dLine),
        .devId = map->devId,
        .fdcChannel = (uint8_t)map->channel,
        .mapLabel = map->mapLabel,
    };
    return true;
}

sensorarrayFdcCellConfigCache_t *sensorarrayMeasureGetFdcCellCache(sensorarrayState_t *state,
                                                                   const sensorarrayFdcCellTarget_t *target)
{
    if (!state || !target || !sensorarrayMatrixIndexIsValid(target->sColumn, target->dLine)) {
        return NULL;
    }
    return &state->fdcCellCache[target->sColumn - 1u][target->dLine - 1u];
}

static void sensorarrayMeasureMarkFdcAppliedCellDirty(sensorarrayState_t *state,
                                                      const sensorarrayFdcCellTarget_t *target);

static bool sensorarrayMeasureFdcReasonEquals(const char *reason, const char *expected)
{
    return reason && expected && strcmp(reason, expected) == 0;
}

static bool sensorarrayMeasureFdcRescueReasonIsManual(const char *reason)
{
    return sensorarrayMeasureFdcReasonEquals(reason, "manual_force_sweep") ||
           sensorarrayMeasureFdcReasonEquals(reason, "manual_force_full_sweep_all") ||
           sensorarrayMeasureFdcReasonEquals(reason, "manual_rescue") ||
           sensorarrayMeasureFdcReasonEquals(reason, "force_full_sweep") ||
           sensorarrayMeasureFdcReasonEquals(reason, "fdc_rescue");
}

static bool sensorarrayMeasureFdcRescueReasonIsFastSweep(const char *reason)
{
    return sensorarrayMeasureFdcReasonEquals(reason, "persistent_fresh_amplitude_warning_after_cache_apply") ||
           sensorarrayMeasureFdcReasonEquals(reason, "cache_missing");
}

static bool sensorarrayMeasureFdcRescueReasonIsHard(const char *reason)
{
    return sensorarrayMeasureFdcReasonEquals(reason, "cache_missing") ||
           sensorarrayMeasureFdcReasonEquals(reason, "cache_missing_and_hard_error") ||
           sensorarrayMeasureFdcReasonEquals(reason, "cache_apply_failed") ||
           sensorarrayMeasureFdcReasonEquals(reason, "no_unread_consecutive") ||
           sensorarrayMeasureFdcReasonEquals(reason, "zero_raw_consecutive") ||
           sensorarrayMeasureFdcReasonEquals(reason, "watchdog_fault_consecutive") ||
           sensorarrayMeasureFdcReasonEquals(reason, "i2c_error_consecutive") ||
           sensorarrayMeasureFdcReasonEquals(reason, "device_config_lost") ||
           sensorarrayMeasureFdcReasonEquals(reason, "manual_force_sweep") ||
           sensorarrayMeasureFdcReasonEquals(reason, "manual_rescue") ||
           sensorarrayMeasureFdcReasonEquals(reason, "i2c_read_error") ||
           sensorarrayMeasureFdcReasonEquals(reason, "watchdog_fault") ||
           sensorarrayMeasureFdcReasonEquals(reason, "zero_raw_no_oscillation") ||
           sensorarrayMeasureFdcReasonEquals(reason, "invalid_streak") ||
           sensorarrayMeasureFdcReasonEquals(reason, "all_invalid_frame");
}

esp_err_t sensorarrayMeasureRequestFdcCellRescue(sensorarrayState_t *state,
                                                uint8_t matrixIndex,
                                                const char *reason)
{
    if (!state) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t sColumn = 0u;
    uint8_t dLine = 0u;
    if (!sensorarrayMeasureDecodeMatrixIndex(matrixIndex, &sColumn, &dLine)) {
        return ESP_ERR_INVALID_ARG;
    }

    sensorarrayFdcCellTarget_t target = {0};
    if (!sensorarrayMeasureMakeFdcCellTarget(state, sColumn, dLine, &target)) {
        return ESP_ERR_INVALID_STATE;
    }

    sensorarrayFdcCellConfigCache_t *cache = sensorarrayMeasureGetFdcCellCache(state, &target);
    if (!cache) {
        return ESP_ERR_INVALID_STATE;
    }

    const char *source = reason ? reason : "runtime_cell_rescue";
    if (sensorarrayMeasureFdcReasonEquals(source, "amplitude_warning") && cache->valid) {
        cache->lastWarningTimestampUs = esp_timer_get_time();
        snprintf(cache->lastWarningReason, sizeof(cache->lastWarningReason), "%s", source);
        printf("FDC_RESCUE_DECISION,scope=cell,s=%u,d=%u,index=%u,device=%s,ch=%u,reason=amplitude_warning,action=suppress_reapply,policy=warning_requires_fresh_persistence\n",
               (unsigned)target.sColumn,
               (unsigned)target.dLine,
               (unsigned)target.matrixIndex,
               sensorarrayMeasureFdcDeviceName(target.devId),
               (unsigned)target.fdcChannel);
        return ESP_OK;
    }

    bool fastSweepReason = sensorarrayMeasureFdcRescueReasonIsFastSweep(source);
    if (!fastSweepReason && !sensorarrayMeasureFdcRescueReasonIsHard(source)) {
        printf("FDC_RESCUE_SUPPRESSED,scope=cell,s=%u,d=%u,index=%u,device=%s,ch=%u,reason=%s,policy=not_hard_error\n",
               (unsigned)target.sColumn,
               (unsigned)target.dLine,
               (unsigned)target.matrixIndex,
               sensorarrayMeasureFdcDeviceName(target.devId),
               (unsigned)target.fdcChannel,
               source);
        return ESP_OK;
    }

    if (!CONFIG_SENSORARRAY_FDC_RUNTIME_FAST_SWEEP_ENABLE &&
        !sensorarrayMeasureFdcRescueReasonIsManual(source)) {
        printf("FDC_RESCUE_SUPPRESSED,scope=cell,s=%u,d=%u,index=%u,device=%s,ch=%u,reason=%s,policy=runtime_fast_sweep_disabled\n",
               (unsigned)target.sColumn,
               (unsigned)target.dLine,
               (unsigned)target.matrixIndex,
               sensorarrayMeasureFdcDeviceName(target.devId),
               (unsigned)target.fdcChannel,
               source);
        return ESP_OK;
    }

    int64_t nowUs = esp_timer_get_time();
    int64_t cooldownUs = (int64_t)CONFIG_SENSORARRAY_FDC_FAST_SWEEP_MIN_COOLDOWN_MS * 1000LL;
    if (cache->lastRescueTimestampUs != 0 &&
        cooldownUs > 0 &&
        (nowUs - cache->lastRescueTimestampUs) < cooldownUs &&
        !sensorarrayMeasureFdcRescueReasonIsManual(source)) {
        printf("FDC_RESCUE_SUPPRESSED,scope=cell,s=%u,d=%u,index=%u,device=%s,ch=%u,reason=%s,policy=cooldown\n",
               (unsigned)target.sColumn,
               (unsigned)target.dLine,
               (unsigned)target.matrixIndex,
               sensorarrayMeasureFdcDeviceName(target.devId),
               (unsigned)target.fdcChannel,
               source);
        return ESP_OK;
    }

    cache->rescuePending = true;
    snprintf(cache->lastRescueReason, sizeof(cache->lastRescueReason), "%s", source);

    printf("FDC_RESCUE,stage=pending,scope=cell,s=%u,d=%u,index=%u,device=%s,ch=%u,reason=%s,type=%s,consecutiveWarnings=%u,consecutiveErrors=%u\n",
           (unsigned)target.sColumn,
           (unsigned)target.dLine,
           (unsigned)target.matrixIndex,
           sensorarrayMeasureFdcDeviceName(target.devId),
           (unsigned)target.fdcChannel,
           cache->lastRescueReason,
           fastSweepReason ? "fast_sweep" : "hard_error",
           (unsigned)cache->consecutiveAmplitudeWarnings,
           (unsigned)cache->consecutiveErrors);
    return ESP_OK;
}

esp_err_t sensorarrayMeasureFdcDiscardStaleSamples(sensorarrayState_t *state,
                                                   const sensorarrayFdcCellTarget_t *target,
                                                   uint8_t discardCount,
                                                   const char *reason)
{
    if (!state || !target || !sensorarrayMatrixIndexIsValid(target->sColumn, target->dLine) ||
        target->fdcChannel > (uint8_t)FDC2214_CH3) {
        return ESP_ERR_INVALID_ARG;
    }
    sensorarrayFdcDeviceState_t *fdcState = sensorarrayMeasureGetFdcState(state, target->devId);
    if (!fdcState || !fdcState->ready || !fdcState->handle) {
        return ESP_ERR_INVALID_STATE;
    }

    printf("FDC_DISCARD,scope=cell,s=%u,d=%u,index=%u,device=%s,ch=%u,count=%u,reason=%s\n",
           (unsigned)target->sColumn,
           (unsigned)target->dLine,
           (unsigned)target->matrixIndex,
           sensorarrayMeasureFdcDeviceName(target->devId),
           (unsigned)target->fdcChannel,
           (unsigned)discardCount,
           reason ? reason : SENSORARRAY_NA);

    esp_err_t firstErr = ESP_OK;
    for (uint8_t i = 0u; i < discardCount; ++i) {
        Fdc2214CapSample_t discard = {0};
        esp_err_t err = Fdc2214CapReadSampleRelaxed(fdcState->handle,
                                                    (Fdc2214CapChannel_t)target->fdcChannel,
                                                    &discard);
        if (err != ESP_OK && firstErr == ESP_OK) {
            firstErr = err;
        }
    }
    return firstErr;
}

static sensorarrayFdcDeviceId_t sensorarrayMeasureFdcDeviceIdFromState(const sensorarrayFdcDeviceState_t *fdcState)
{
    if (fdcState && fdcState->label && strstr(fdcState->label, "secondary")) {
        return SENSORARRAY_FDC_DEV_SECONDARY;
    }
    return SENSORARRAY_FDC_DEV_PRIMARY;
}

static uint8_t sensorarrayMeasureFdcUnreadMaskFromStatus(const Fdc2214CapStatus_t *status)
{
    if (!status) {
        return 0u;
    }

    uint8_t unreadMask = 0u;
    for (uint8_t ch = 0u; ch < 4u; ++ch) {
        if (status->UnreadConversion[ch]) {
            unreadMask |= (uint8_t)(1u << ch);
        }
    }
    return unreadMask;
}

static sensorarrayFdcReadyDecoded_t sensorarrayMeasureFdcBuildReadyState(
    const Fdc2214CapStatus_t *status,
    uint8_t requiredMask)
{
    requiredMask &= 0x0Fu;
    if (requiredMask == 0u) {
        requiredMask = SENSORARRAY_FDC_AUTOSCAN_READY_UNREAD_MASK;
    }

    sensorarrayFdcReadyDecoded_t ready = {
        .statusRaw = status ? status->Raw : 0u,
        .unreadMask = sensorarrayMeasureFdcUnreadMaskFromStatus(status),
        .dataReady = status && status->DataReady,
    };
    ready.unreadFull = (ready.unreadMask & requiredMask) == requiredMask;
    ready.readyForDataRead = ready.dataReady && ready.unreadFull;
    ready.unreadWithoutDataReady = ready.unreadFull && !ready.dataReady;
    return ready;
}

static void sensorarrayMeasureFdcApplyReadyDecoded(sensorarrayFdcReadyState_t *ready,
                                                   const sensorarrayFdcReadyDecoded_t *decoded)
{
    if (!ready || !decoded) {
        return;
    }

    ready->statusRaw = decoded->statusRaw;
    ready->dataReady = decoded->dataReady;
    ready->drdy = decoded->dataReady ? 1u : 0u;
    ready->unreadMask = decoded->unreadMask;
    ready->unreadFull = decoded->unreadFull;
    ready->readyForDataRead = decoded->readyForDataRead;
    ready->unreadWithoutDataReady = decoded->unreadWithoutDataReady;
}

static bool sensorarrayMeasureFdcDeglitchCodeValid(uint8_t deglitchCode)
{
    switch (deglitchCode) {
    case FDC2214_DEGLITCH_1MHZ:
    case FDC2214_DEGLITCH_3P3MHZ:
    case FDC2214_DEGLITCH_10MHZ:
    case FDC2214_DEGLITCH_33MHZ:
        return true;
    default:
        return false;
    }
}

static Fdc2214CapDeglitch_t sensorarrayMeasureSelectedFdcDeglitch(const sensorarrayFdcDeviceState_t *fdcState)
{
    uint8_t selectedCode = (uint8_t)FDC2214_DEGLITCH_3P3MHZ;
    uint32_t selectedBandwidthHz = 3300000u;

    if (fdcState) {
        uint8_t muxDeglitch = (uint8_t)(fdcState->muxConfigReg & SENSORARRAY_FDC_MUX_DEGLITCH_MASK);
        if (sensorarrayMeasureFdcDeglitchCodeValid(muxDeglitch)) {
            selectedCode = muxDeglitch;
        }

        for (uint8_t ch = 0u; ch < 4u; ++ch) {
            const sensorarrayFdcSweepProfile_t *profile = &fdcState->sweepProfile[ch];
            if (profile->valid &&
                sensorarrayMeasureFdcDeglitchCodeValid(profile->selectedDeglitchCode) &&
                profile->selectedDeglitchBandwidthHz >= selectedBandwidthHz) {
                selectedCode = profile->selectedDeglitchCode;
                selectedBandwidthHz = profile->selectedDeglitchBandwidthHz;
            }
        }
    }

    return (Fdc2214CapDeglitch_t)selectedCode;
}

static uint8_t sensorarrayMeasureFdcRegForChannel(uint8_t base, Fdc2214CapChannel_t channel)
{
    return (uint8_t)(base + (uint8_t)channel);
}

static uint32_t sensorarrayMeasureFdcDeglitchBandwidthHz(uint8_t deglitchCode)
{
    switch (deglitchCode) {
    case FDC2214_DEGLITCH_1MHZ:
        return 1000000u;
    case FDC2214_DEGLITCH_3P3MHZ:
        return 3300000u;
    case FDC2214_DEGLITCH_10MHZ:
        return 10000000u;
    case FDC2214_DEGLITCH_33MHZ:
        return 33000000u;
    default:
        return 0u;
    }
}

static uint8_t sensorarrayMeasureFdcSafeDefaultDeglitch(void)
{
    return (uint8_t)FDC2214_DEGLITCH_10MHZ;
}

static void sensorarrayMeasureMarkFdcAppliedCellDirty(sensorarrayState_t *state,
                                                      const sensorarrayFdcCellTarget_t *target)
{
    if (!state || !target || target->devId > SENSORARRAY_FDC_DEV_SECONDARY || target->fdcChannel >= 4u) {
        return;
    }

    sensorarrayFdcAppliedRowConfig_t *applied = &state->fdcAppliedRow[(uint8_t)target->devId];
    if (applied->valid &&
        applied->row == target->sColumn &&
        applied->deviceId == (uint8_t)target->devId) {
        applied->dirty = true;
    }
}

static bool sensorarrayMeasureAppliedRowConfigMatches(const sensorarrayFdcAppliedRowConfig_t *applied,
                                                      const sensorarrayFdcAppliedRowConfig_t *expected)
{
    if (!applied || !expected ||
        !applied->valid ||
        !applied->autoscanConfigured ||
        applied->dirty ||
        applied->deviceId != expected->deviceId ||
        applied->selectedDeglitch != expected->selectedDeglitch ||
        applied->muxConfig != expected->muxConfig ||
        applied->statusConfig != expected->statusConfig ||
        applied->configBaseWithoutSleepBit != expected->configBaseWithoutSleepBit) {
        return false;
    }

    for (uint8_t ch = 0u; ch < 4u; ++ch) {
        if (applied->rCount[ch] != expected->rCount[ch] ||
            applied->settleCount[ch] != expected->settleCount[ch] ||
            applied->clockDiv[ch] != expected->clockDiv[ch] ||
            applied->driveCurrent[ch] != expected->driveCurrent[ch] ||
            applied->cacheGeneration[ch] != expected->cacheGeneration[ch]) {
            return false;
        }
    }
    return true;
}

static uint16_t sensorarrayMeasureFdcBuildMuxConfig(uint8_t deglitchCode)
{
    uint8_t deglitch = sensorarrayMeasureFdcDeglitchCodeValid(deglitchCode) ?
        deglitchCode :
        sensorarrayMeasureFdcSafeDefaultDeglitch();
    return (uint16_t)(SENSORARRAY_FDC_MUX_AUTOSCAN_EN_MASK |
                      ((uint16_t)SENSORARRAY_FDC_AUTOSCAN_RR_SEQUENCE <<
                       SENSORARRAY_FDC_MUX_RR_SEQUENCE_SHIFT) |
                      SENSORARRAY_FDC_MUX_FIXED_MASK |
                      (uint16_t)(deglitch & SENSORARRAY_FDC_MUX_DEGLITCH_MASK));
}

static uint16_t sensorarrayMeasureFdcConfigBaseWithoutSleep(const sensorarrayFdcDeviceState_t *fdcState)
{
    uint16_t config = fdcState ? fdcState->configReg : 0u;
    if (config == 0u) {
        Fdc2214CapConfigOptions_t fallbackOptions = {
            .ActiveChannel = FDC2214_CH0,
            .SleepModeEnabled = false,
            .SensorActivateSelLowPower = false,
            .RefClockSource = (fdcState && fdcState->refClockSource == FDC2214_REF_CLOCK_EXTERNAL) ?
                FDC2214_REF_CLOCK_EXTERNAL :
                FDC2214_REF_CLOCK_INTERNAL,
            .IntbDisabled = (CONFIG_SENSORARRAY_FDC_INTB_ENABLE == 0),
            .HighCurrentDrive = false,
        };
        config = Fdc2214CapBuildConfig(&fallbackOptions);
    }
    config &= (uint16_t)~SENSORARRAY_FDC_CONFIG_SLEEP_MODE_EN_MASK;
    /*
     * Formal matrix reads are polling-only in this revision. Keep the FDC INTB
     * output disabled so CONFIG and the worker ready strategy do not disagree.
     */
    config |= SENSORARRAY_FDC_CONFIG_INTB_DIS_MASK;
    return config;
}

static uint32_t sensorarrayMeasureFdcConfigFingerprint(const sensorarrayFdcAppliedRowConfig_t *config)
{
    if (!config) {
        return 0u;
    }

    uint32_t fp = 2166136261u;
#define SENSORARRAY_FDC_FP_ADD(value_) \
    do { fp ^= (uint32_t)(value_); fp *= 16777619u; } while (0)
    SENSORARRAY_FDC_FP_ADD(config->selectedDeglitch);
    SENSORARRAY_FDC_FP_ADD(config->muxConfig);
    SENSORARRAY_FDC_FP_ADD(config->statusConfig);
    SENSORARRAY_FDC_FP_ADD(config->configBaseWithoutSleepBit);
    for (uint8_t ch = 0u; ch < 4u; ++ch) {
        SENSORARRAY_FDC_FP_ADD(config->rCount[ch]);
        SENSORARRAY_FDC_FP_ADD(config->settleCount[ch]);
        SENSORARRAY_FDC_FP_ADD(config->clockDiv[ch]);
        SENSORARRAY_FDC_FP_ADD(config->driveCurrent[ch]);
        SENSORARRAY_FDC_FP_ADD(config->cacheGeneration[ch]);
    }
#undef SENSORARRAY_FDC_FP_ADD
    return (fp == 0u) ? 1u : fp;
}

static sensorarrayFdcCellConfigCache_t *sensorarrayMeasureFdcRowDeviceCache(sensorarrayState_t *state,
                                                                            uint8_t row,
                                                                            sensorarrayFdcDeviceId_t devId,
                                                                            uint8_t ch,
                                                                            uint8_t *outDIndex)
{
    if (!state || row < 1u || row > SENSORARRAY_MATRIX_ROWS || ch >= 4u) {
        return NULL;
    }

    uint8_t dIndex = (uint8_t)(1u + ch + ((devId == SENSORARRAY_FDC_DEV_SECONDARY) ? 4u : 0u));
    if (outDIndex) {
        *outDIndex = dIndex;
    }
    sensorarrayFdcCellTarget_t target = {0};
    if (!sensorarrayMeasureMakeFdcCellTarget(state, row, dIndex, &target)) {
        return NULL;
    }
    return sensorarrayMeasureGetFdcCellCache(state, &target);
}

static uint8_t sensorarrayFdcMergeDeglitchForRowDevice(sensorarrayState_t *state,
                                                       uint8_t row,
                                                       sensorarrayFdcDeviceId_t devId,
                                                       uint8_t safeDefaultDeglitch)
{
    uint8_t selected = sensorarrayMeasureFdcDeglitchCodeValid(safeDefaultDeglitch) ?
        safeDefaultDeglitch :
        sensorarrayMeasureFdcSafeDefaultDeglitch();
    uint32_t selectedBandwidthHz = sensorarrayMeasureFdcDeglitchBandwidthHz(selected);
    bool anyValid = false;
    uint8_t sourceDeglitch[4] = {0};

    for (uint8_t ch = 0u; ch < 4u; ++ch) {
        sensorarrayFdcCellConfigCache_t *cache =
            sensorarrayMeasureFdcRowDeviceCache(state, row, devId, ch, NULL);
        sourceDeglitch[ch] = cache ? cache->deglitchCode : 0u;
        if (!cache || !cache->valid || !sensorarrayMeasureFdcDeglitchCodeValid(cache->deglitchCode)) {
            continue;
        }
        anyValid = true;
        uint32_t bandwidthHz = sensorarrayMeasureFdcDeglitchBandwidthHz(cache->deglitchCode);
        if (bandwidthHz >= selectedBandwidthHz) {
            selected = cache->deglitchCode;
            selectedBandwidthHz = bandwidthHz;
        }
    }

#if CONFIG_SENSORARRAY_FDC_VERBOSE_SCAN_LOG
    printf("FDC_CACHE,stage=row_deglitch_merge,row=%u,device=%s,sourceDeglitch=[0x%X,0x%X,0x%X,0x%X],selected=0x%X,reason=%s\n",
           (unsigned)row,
           sensorarrayMeasureFdcDeviceName(devId),
           (unsigned)sourceDeglitch[0],
           (unsigned)sourceDeglitch[1],
           (unsigned)sourceDeglitch[2],
           (unsigned)sourceDeglitch[3],
           (unsigned)selected,
           anyValid ? "cover_highest_frequency" : "safe_default");
#else
    (void)anyValid;
    (void)sourceDeglitch;
#endif
    return selected;
}

static bool __attribute__((unused)) sensorarrayMeasureFdcAutoscanConfigLooksCurrent(const sensorarrayFdcDeviceState_t *fdcState,
                                                                                   uint8_t expectedDeglitch)
{
    if (!fdcState || !fdcState->configVerified) {
        return false;
    }

    uint8_t rr = (uint8_t)((fdcState->muxConfigReg & SENSORARRAY_FDC_MUX_RR_SEQUENCE_MASK) >>
                           SENSORARRAY_FDC_MUX_RR_SEQUENCE_SHIFT);
    uint8_t deglitch = (uint8_t)(fdcState->muxConfigReg & SENSORARRAY_FDC_MUX_DEGLITCH_MASK);
    bool autoscan = (fdcState->muxConfigReg & SENSORARRAY_FDC_MUX_AUTOSCAN_EN_MASK) != 0u;
    bool highCurrent = (fdcState->configReg & SENSORARRAY_FDC_CONFIG_HIGH_CURRENT_DRV_MASK) != 0u;
    bool sleeping = (fdcState->configReg & 0x2000u) != 0u;
    return autoscan &&
           rr == SENSORARRAY_FDC_AUTOSCAN_RR_SEQUENCE &&
           deglitch == expectedDeglitch &&
           !highCurrent &&
           !sleeping;
}

#include "fdc/sensorarrayFdcCacheApply.inc"

static esp_err_t sensorarrayMeasureVerifyFdcChannelConfigApplied(sensorarrayFdcDeviceState_t *fdcState)
{
    if (!fdcState || !fdcState->handle) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t firstErr = ESP_OK;
    for (uint8_t ch = 0u; ch < 4u; ++ch) {
        uint16_t rcount = 0u;
        uint16_t settle = 0u;
        uint16_t clockDiv = 0u;
        uint16_t drive = 0u;
        esp_err_t err = Fdc2214CapReadRawRegisters(fdcState->handle,
                                                   sensorarrayMeasureFdcRegForChannel(SENSORARRAY_FDC_REG_RCOUNT_BASE,
                                                                                      (Fdc2214CapChannel_t)ch),
                                                   &rcount);
        if (err == ESP_OK) {
            err = Fdc2214CapReadRawRegisters(fdcState->handle,
                                             sensorarrayMeasureFdcRegForChannel(SENSORARRAY_FDC_REG_SETTLECOUNT_BASE,
                                                                                (Fdc2214CapChannel_t)ch),
                                             &settle);
        }
        if (err == ESP_OK) {
            err = Fdc2214CapReadRawRegisters(fdcState->handle,
                                             sensorarrayMeasureFdcRegForChannel(SENSORARRAY_FDC_REG_CLOCK_DIVIDERS_BASE,
                                                                                (Fdc2214CapChannel_t)ch),
                                             &clockDiv);
        }
        if (err == ESP_OK) {
            err = Fdc2214CapReadRawRegisters(fdcState->handle,
                                             sensorarrayMeasureFdcRegForChannel(SENSORARRAY_FDC_REG_DRIVE_CURRENT_BASE,
                                                                                (Fdc2214CapChannel_t)ch),
                                             &drive);
        }
        if (err == ESP_OK && (rcount == 0u || settle == 0u || clockDiv == 0u)) {
            err = ESP_ERR_INVALID_RESPONSE;
        }
        if (err != ESP_OK && firstErr == ESP_OK) {
            firstErr = err;
        }
        if (err != ESP_OK) {
            printf("MATRIXFDC_DIAG,stage=autoscan_channel_config_invalid,device=%s,ch=%u,rCount=0x%04X,settle=0x%04X,clockDiv=0x%04X,drive=0x%04X,err=0x%lx\n",
                   fdcState->label ? fdcState->label : SENSORARRAY_NA,
                   (unsigned)ch,
                   rcount,
                   settle,
                   clockDiv,
                   drive,
                   (unsigned long)err);
        }
    }
    return firstErr;
}

static esp_err_t __attribute__((unused)) sensorarrayMeasureEnsureFdcAutoscan4ch(sensorarrayState_t *state,
                                                                                sensorarrayFdcDeviceId_t devId)
{
    sensorarrayFdcDeviceState_t *fdcState = sensorarrayMeasureGetFdcState(state, devId);
    if (!fdcState || !fdcState->ready || !fdcState->handle) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = sensorarrayMeasureVerifyFdcChannelConfigApplied(fdcState);
    if (err != ESP_OK) {
        return err;
    }

    Fdc2214CapDeglitch_t deglitch = sensorarrayMeasureSelectedFdcDeglitch(fdcState);
    err = Fdc2214CapSetAutoScanMode(fdcState->handle,
                                    SENSORARRAY_FDC_AUTOSCAN_RR_SEQUENCE,
                                    deglitch);
    if (err != ESP_OK) {
        return err;
    }

    Fdc2214CapCoreRegs_t regs = {0};
    err = Fdc2214CapReadCoreRegs(fdcState->handle, &regs);
    if (err != ESP_OK) {
        return err;
    }

    fdcState->statusConfigReg = regs.StatusConfig;
    fdcState->configReg = regs.Config;
    fdcState->muxConfigReg = regs.MuxConfig;

    uint8_t rr = (uint8_t)((regs.MuxConfig & SENSORARRAY_FDC_MUX_RR_SEQUENCE_MASK) >>
                           SENSORARRAY_FDC_MUX_RR_SEQUENCE_SHIFT);
    uint8_t muxDeglitch = (uint8_t)(regs.MuxConfig & SENSORARRAY_FDC_MUX_DEGLITCH_MASK);
    bool autoscan = (regs.MuxConfig & SENSORARRAY_FDC_MUX_AUTOSCAN_EN_MASK) != 0u;
    bool highCurrent = (regs.Config & SENSORARRAY_FDC_CONFIG_HIGH_CURRENT_DRV_MASK) != 0u;

#if CONFIG_SENSORARRAY_FDC_VERBOSE_SCAN_LOG
    printf("FDC_AUTOSCAN_CONFIG,device=%s,mux=0x%04X,config=0x%04X,autoscan=%u,rr=%u,deglitch=0x%X,highCurrent=%u\n",
           sensorarrayMeasureFdcDeviceName(devId),
           regs.MuxConfig,
           regs.Config,
           autoscan ? 1u : 0u,
           (unsigned)rr,
           (unsigned)muxDeglitch,
           highCurrent ? 1u : 0u);
#endif
    (void)muxDeglitch;

    if (!autoscan ||
        rr != SENSORARRAY_FDC_AUTOSCAN_RR_SEQUENCE ||
        highCurrent ||
        regs.MuxConfig == 0x020Du) {
        printf("MATRIXFDC_DIAG,stage=autoscan_config_reject,device=%s,mux=0x%04X,config=0x%04X,autoscan=%u,rr=%u,highCurrent=%u,err=0x%lx\n",
               sensorarrayMeasureFdcDeviceName(devId),
               regs.MuxConfig,
               regs.Config,
               autoscan ? 1u : 0u,
               (unsigned)rr,
               highCurrent ? 1u : 0u,
               (unsigned long)ESP_ERR_INVALID_RESPONSE);
        return ESP_ERR_INVALID_RESPONSE;
    }

    return ESP_OK;
}

static esp_err_t __attribute__((unused)) sensorarrayMeasureWaitFdcAutoscanFrameReady(sensorarrayFdcDeviceState_t *fdcState,
                                                                                    uint8_t row,
                                                                                    uint32_t timeoutMs,
                                                                                    uint16_t *outStatus)
{
    if (!fdcState || !fdcState->handle) {
        return ESP_ERR_INVALID_STATE;
    }
    if (timeoutMs == 0u) {
        timeoutMs = 1u;
    }

    int64_t deadlineUs = esp_timer_get_time() + ((int64_t)timeoutMs * 1000LL);
    Fdc2214CapStatus_t status = {0};
    esp_err_t lastErr = ESP_ERR_TIMEOUT;
    while (esp_timer_get_time() <= deadlineUs) {
        esp_err_t err = Fdc2214CapReadStatus(fdcState->handle, &status);
        if (err == ESP_OK) {
            sensorarrayFdcReadyDecoded_t ready =
                sensorarrayMeasureFdcBuildReadyState(&status, SENSORARRAY_FDC_AUTOSCAN_READY_UNREAD_MASK);
            if (ready.readyForDataRead) {
                if (outStatus) {
                    *outStatus = status.Raw;
                }
#if CONFIG_SENSORARRAY_FDC_VERBOSE_SCAN_LOG
                printf("FDC_FRAME_READY,row=%u,device=%s,status=0x%04X,unreadMask=0x%X,drdy=%u,err=0x%lx\n",
                       (unsigned)row,
                       sensorarrayMeasureFdcDeviceName(sensorarrayMeasureFdcDeviceIdFromState(fdcState)),
                       status.Raw,
                       (unsigned)ready.unreadMask,
                       status.DataReady ? 1u : 0u,
                       (unsigned long)ESP_OK);
#endif
                return ESP_OK;
            }
        }
        lastErr = err;
        vTaskDelay(pdMS_TO_TICKS(1u));
    }

    if (outStatus) {
        *outStatus = status.Raw;
    }
    printf("FDC_FRAME_READY,row=%u,device=%s,status=0x%04X,unreadMask=0x%X,drdy=%u,err=0x%lx\n",
           (unsigned)row,
           sensorarrayMeasureFdcDeviceName(sensorarrayMeasureFdcDeviceIdFromState(fdcState)),
           status.Raw,
           (unsigned)sensorarrayMeasureFdcUnreadMaskFromStatus(&status),
           status.DataReady ? 1u : 0u,
           (unsigned long)((lastErr == ESP_OK) ? ESP_ERR_TIMEOUT : lastErr));
    return (lastErr == ESP_OK) ? ESP_ERR_TIMEOUT : lastErr;
}

static void __attribute__((unused)) sensorarrayMeasurePollFdcReady(sensorarrayFdcDeviceState_t *fdcState,
                                                                  sensorarrayFdcReadyState_t *ready)
{
    if (!ready) {
        return;
    }
    if (!fdcState || !fdcState->handle) {
        ready->err = ESP_ERR_INVALID_STATE;
        return;
    }

    ready->pollCount++;
    Fdc2214CapStatus_t status = {0};
    esp_err_t err = Fdc2214CapReadStatus(fdcState->handle, &status);
    ready->err = err;
    if (err != ESP_OK) {
        return;
    }
    ready->statusRaw = status.Raw;
    ready->dataReady = status.DataReady;
    sensorarrayFdcReadyDecoded_t decoded =
        sensorarrayMeasureFdcBuildReadyState(&status, SENSORARRAY_FDC_AUTOSCAN_READY_UNREAD_MASK);
    sensorarrayMeasureFdcApplyReadyDecoded(ready, &decoded);
    ready->ready = ready->readyForDataRead;
}

static esp_err_t __attribute__((unused)) sensorarrayMeasureWaitBothFdcAutoscanFrameReady(sensorarrayState_t *state,
                                                                                        uint8_t row,
                                                                                        uint32_t timeoutMs,
                                                                                        sensorarrayFdcReadyState_t *primaryReady,
                                                                                        sensorarrayFdcReadyState_t *secondaryReady)
{
    if (!state || !primaryReady || !secondaryReady) {
        return ESP_ERR_INVALID_ARG;
    }
    if (timeoutMs == 0u) {
        timeoutMs = 1u;
    }

    *primaryReady = (sensorarrayFdcReadyState_t){.err = ESP_ERR_TIMEOUT};
    *secondaryReady = (sensorarrayFdcReadyState_t){.err = ESP_ERR_TIMEOUT};
    int64_t deadlineUs = esp_timer_get_time() + ((int64_t)timeoutMs * 1000LL);
    while (esp_timer_get_time() <= deadlineUs) {
        if (!primaryReady->ready) {
            sensorarrayMeasurePollFdcReady(&state->fdcPrimary, primaryReady);
        }
        if (!secondaryReady->ready) {
            sensorarrayMeasurePollFdcReady(&state->fdcSecondary, secondaryReady);
        }
        if (primaryReady->ready && secondaryReady->ready) {
#if CONFIG_SENSORARRAY_FDC_VERBOSE_SCAN_LOG
            printf("FDC_FRAME_READY,row=%u,device=both,primaryStatus=0x%04X,primaryUnread=0x%X,primaryDrdy=%u,secondaryStatus=0x%04X,secondaryUnread=0x%X,secondaryDrdy=%u,err=0x0\n",
                   (unsigned)row,
                   primaryReady->statusRaw,
                   (unsigned)primaryReady->unreadMask,
                   primaryReady->dataReady ? 1u : 0u,
                   secondaryReady->statusRaw,
                   (unsigned)secondaryReady->unreadMask,
                   secondaryReady->dataReady ? 1u : 0u);
#endif
            return ESP_OK;
        }
        vTaskDelay(pdMS_TO_TICKS(1u));
    }

    esp_err_t err = ESP_ERR_TIMEOUT;
    if (!primaryReady->ready && primaryReady->err != ESP_OK && primaryReady->err != ESP_ERR_TIMEOUT) {
        err = primaryReady->err;
    } else if (!secondaryReady->ready &&
               secondaryReady->err != ESP_OK &&
               secondaryReady->err != ESP_ERR_TIMEOUT) {
        err = secondaryReady->err;
    }
    if (!primaryReady->ready) {
        primaryReady->timeoutCount++;
    }
    if (!secondaryReady->ready) {
        secondaryReady->timeoutCount++;
    }
    printf("FDC_FRAME_READY,row=%u,device=both,primaryReady=%u,primaryStatus=0x%04X,primaryUnread=0x%X,primaryDrdy=%u,primaryErr=0x%lx,secondaryReady=%u,secondaryStatus=0x%04X,secondaryUnread=0x%X,secondaryDrdy=%u,secondaryErr=0x%lx,err=0x%lx\n",
           (unsigned)row,
           primaryReady->ready ? 1u : 0u,
           primaryReady->statusRaw,
           (unsigned)primaryReady->unreadMask,
           primaryReady->dataReady ? 1u : 0u,
           (unsigned long)primaryReady->err,
           secondaryReady->ready ? 1u : 0u,
           secondaryReady->statusRaw,
           (unsigned)secondaryReady->unreadMask,
           secondaryReady->dataReady ? 1u : 0u,
           (unsigned long)secondaryReady->err,
           (unsigned long)err);
    return err;
}

#include "fdc/sensorarrayFdcRead4.inc"

static sensorarrayFdcWorkerContext_t *sensorarrayMeasureFdcWorkerContext(sensorarrayFdcDeviceId_t devId)
{
    return (devId <= SENSORARRAY_FDC_DEV_SECONDARY) ? &s_fdcWorkers[(uint8_t)devId] : NULL;
}

static void sensorarrayMeasureRecordFirstErr(esp_err_t err, esp_err_t *firstErr)
{
    if (firstErr && *firstErr == ESP_OK && err != ESP_OK) {
        *firstErr = err;
    }
}

static esp_err_t sensorarrayMeasureFdcFormalPrecheckDevice(sensorarrayState_t *state,
                                                           sensorarrayFdcDeviceId_t devId)
{
    sensorarrayFdcDeviceState_t *fdcState = sensorarrayMeasureGetFdcState(state, devId);
    sensorarrayFdcWorkerContext_t *ctx = sensorarrayMeasureFdcWorkerContext(devId);
    esp_err_t firstErr = ESP_OK;
    uint16_t idMfg = 0u;
    uint16_t idDev = 0u;
    uint16_t config = 0u;
    uint16_t mux = 0u;
    uint16_t errorConfig = 0u;
    Fdc2214CapStatus_t status = {0};

    if (!fdcState || !fdcState->handle) {
        firstErr = ESP_ERR_INVALID_STATE;
    } else {
        esp_err_t err = Fdc2214CapReadId(fdcState->handle, &idMfg, &idDev);
        sensorarrayMeasureRecordFirstErr(err, &firstErr);
        err = Fdc2214CapReadRawRegisters(fdcState->handle,
                                         SENSORARRAY_FDC_REG_CONFIG,
                                         &config);
        if (err == ESP_OK) {
            fdcState->configReg = config;
            fdcState->configVerified = true;
        } else {
            fdcState->configVerified = false;
        }
        sensorarrayMeasureRecordFirstErr(err, &firstErr);
        err = Fdc2214CapReadRawRegisters(fdcState->handle,
                                         SENSORARRAY_FDC_REG_MUX_CONFIG,
                                         &mux);
        if (err == ESP_OK) {
            fdcState->muxConfigReg = mux;
        }
        sensorarrayMeasureRecordFirstErr(err, &firstErr);
        err = Fdc2214CapReadRawRegisters(fdcState->handle,
                                         SENSORARRAY_FDC_REG_STATUS_CONFIG,
                                         &errorConfig);
        if (err == ESP_OK) {
            fdcState->statusConfigReg = errorConfig;
        }
        sensorarrayMeasureRecordFirstErr(err, &firstErr);
        err = Fdc2214CapReadStatus(fdcState->handle, &status);
        sensorarrayMeasureRecordFirstErr(err, &firstErr);
    }

    bool idOk = firstErr == ESP_OK &&
                idMfg == SENSORARRAY_FDC_EXPECTED_MANUFACTURER_ID &&
                idDev == SENSORARRAY_FDC_EXPECTED_DEVICE_ID &&
                idMfg != 0u &&
                idMfg != UINT16_MAX &&
                idDev != 0u &&
                idDev != UINT16_MAX;
    uint8_t unread = sensorarrayMeasureFdcUnreadMaskFromStatus(&status);
    bool intbDisabled = (config & SENSORARRAY_FDC_CONFIG_INTB_DIS_MASK) != 0u;
    bool drdyToIntb = (errorConfig & SENSORARRAY_FDC_STATUS_CONFIG_DRDY_2INT_MASK) != 0u;
    bool autoscan = (mux & SENSORARRAY_FDC_MUX_AUTOSCAN_EN_MASK) != 0u;
    uint8_t rrSeq = (uint8_t)((mux & SENSORARRAY_FDC_MUX_RR_SEQUENCE_MASK) >>
                              SENSORARRAY_FDC_MUX_RR_SEQUENCE_SHIFT);
    bool sleep = (config & SENSORARRAY_FDC_CONFIG_SLEEP_MODE_EN_MASK) != 0u;

    printf("FDC_FORMAL_PRECHECK,dev=%s,bus=%d,addr=0x%02X,intbGpio=%d,idMfg=0x%04X,idDev=0x%04X,config=0x%04X,mux=0x%04X,errorConfig=0x%04X,status=0x%04X,unread=0x%X,intbDisabled=%u,drdyToIntb=%u,autoscan=%u,rrSeq=%u,sleep=%u,readyMode=%s,intbHint=%s,idOk=%u,err=0x%lx,errName=%s\n",
           sensorarrayMeasureFdcDeviceName(devId),
           (fdcState && fdcState->i2cCtx) ? (int)fdcState->i2cCtx->Port : -1,
           fdcState ? fdcState->i2cAddr : 0u,
           ctx ? ctx->intbGpio : -1,
           idMfg,
           idDev,
           config,
           mux,
           errorConfig,
           status.Raw,
           (unsigned)unread,
           intbDisabled ? 1u : 0u,
           drdyToIntb ? 1u : 0u,
           autoscan ? 1u : 0u,
           (unsigned)rrSeq,
           sleep ? 1u : 0u,
           SENSORARRAY_FDC_READY_MODE_NAME,
           SENSORARRAY_FDC_INTB_HINT_NAME,
           idOk ? 1u : 0u,
           (unsigned long)firstErr,
           sensorarrayMeasureEspErrName(firstErr));
    return firstErr;
}

static esp_err_t sensorarrayMeasureRunFdcFormalPrecheck(sensorarrayState_t *state)
{
    esp_err_t firstErr = sensorarrayMeasureFdcFormalPrecheckDevice(state,
                                                                   SENSORARRAY_FDC_DEV_PRIMARY);
    esp_err_t err = sensorarrayMeasureFdcFormalPrecheckDevice(state,
                                                              SENSORARRAY_FDC_DEV_SECONDARY);
    sensorarrayMeasureRecordFirstErr(err, &firstErr);
    return firstErr;
}

static void sensorarrayMeasureDrainSemaphore(SemaphoreHandle_t sem)
{
    if (!sem) {
        return;
    }
    while (xSemaphoreTake(sem, 0) == pdTRUE) {
    }
}

static esp_err_t sensorarrayFdcEnsureGpioIsrServiceInstalled(void)
{
    if (s_fdcGpioIsrServiceInstalled) {
        return ESP_OK;
    }

    portENTER_CRITICAL(&s_fdcGpioIsrServiceMux);
    if (!s_fdcGpioIsrServiceMutex) {
        s_fdcGpioIsrServiceMutex = xSemaphoreCreateMutex();
    }
    portEXIT_CRITICAL(&s_fdcGpioIsrServiceMux);

    if (!s_fdcGpioIsrServiceMutex) {
        return ESP_ERR_NO_MEM;
    }

    if (xSemaphoreTake(s_fdcGpioIsrServiceMutex, pdMS_TO_TICKS(100u)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    if (s_fdcGpioIsrServiceInstalled) {
        xSemaphoreGive(s_fdcGpioIsrServiceMutex);
        return ESP_OK;
    }

    esp_err_t err = gpio_install_isr_service(0);
    if (err == ESP_OK) {
        s_fdcGpioIsrServiceInstalled = true;
        printf("FDC_INTB_ISR_SERVICE,stage=install,status=installed,err=0x0\n");
        xSemaphoreGive(s_fdcGpioIsrServiceMutex);
        return ESP_OK;
    }
    if (err == ESP_ERR_INVALID_STATE) {
        s_fdcGpioIsrServiceInstalled = true;
        printf("FDC_INTB_ISR_SERVICE,stage=install,status=already_installed,err=0x%lx\n",
               (unsigned long)err);
        xSemaphoreGive(s_fdcGpioIsrServiceMutex);
        return ESP_OK;
    }

    printf("FDC_INTB_ISR_SERVICE,stage=install,status=failed,err=0x%lx\n",
           (unsigned long)err);
    xSemaphoreGive(s_fdcGpioIsrServiceMutex);
    return err;
}

static void IRAM_ATTR sensorarrayMeasureFdcIntbIsr(void *arg)
{
    sensorarrayFdcWorkerContext_t *ctx = (sensorarrayFdcWorkerContext_t *)arg;
    if (!ctx) {
        return;
    }

    ctx->edgeCount++;
    ctx->lastLevel = gpio_get_level((gpio_num_t)ctx->intbGpio);
    ctx->lastEdgeUs = esp_timer_get_time();
    ctx->lastEpochSeen = ctx->currentEpoch;

    TaskHandle_t task = (TaskHandle_t)ctx->waitTask;
    if (task) {
        BaseType_t higherPriorityTaskWoken = pdFALSE;
        vTaskNotifyGiveFromISR(task, &higherPriorityTaskWoken);
        if (higherPriorityTaskWoken == pdTRUE) {
            portYIELD_FROM_ISR();
        }
    }
}

static esp_err_t sensorarrayMeasureEnsureFdcIntb(sensorarrayFdcWorkerContext_t *ctx)
{
    if (!ctx) {
        return ESP_ERR_INVALID_ARG;
    }
    if (ctx->intbReady || !CONFIG_SENSORARRAY_FDC_INTB_ENABLE || ctx->intbGpio < 0) {
        return ctx->intbReady ? ESP_OK : ESP_ERR_NOT_SUPPORTED;
    }

    gpio_config_t gpioConfig = {
        .pin_bit_mask = 1ULL << (uint32_t)ctx->intbGpio,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = CONFIG_SENSORARRAY_FDC_INTB_WEAK_PULLUP ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = CONFIG_SENSORARRAY_FDC_INTB_TRIGGER_ANYEDGE ? GPIO_INTR_ANYEDGE : GPIO_INTR_NEGEDGE,
    };
    esp_err_t err = gpio_config(&gpioConfig);
    if (err != ESP_OK) {
        return err;
    }

    err = sensorarrayFdcEnsureGpioIsrServiceInstalled();
    if (err != ESP_OK) {
        return err;
    }

    err = gpio_isr_handler_add((gpio_num_t)ctx->intbGpio, sensorarrayMeasureFdcIntbIsr, ctx);
    bool reattached = false;
    if (err == ESP_ERR_INVALID_STATE) {
        (void)gpio_isr_handler_remove((gpio_num_t)ctx->intbGpio);
        err = gpio_isr_handler_add((gpio_num_t)ctx->intbGpio, sensorarrayMeasureFdcIntbIsr, ctx);
        reattached = true;
    }
    if (err != ESP_OK) {
        return err;
    }
    ctx->intbIsrAttached = true;
    ctx->lastLevel = gpio_get_level((gpio_num_t)ctx->intbGpio);
    ctx->intbReady = true;
    printf("FDC_INTB,device=%s,gpio=%d,handler=%s,status=attached\n",
           sensorarrayMeasureFdcDeviceName(ctx->devId),
           ctx->intbGpio,
           reattached ? "reattached" : "attached");
    printf("FDC_INTB,device=%s,gpio=%d,trigger=%s,idleLevel=%d,status=ready\n",
           sensorarrayMeasureFdcDeviceName(ctx->devId),
           ctx->intbGpio,
           CONFIG_SENSORARRAY_FDC_INTB_TRIGGER_ANYEDGE ? "anyedge" : "falling",
           ctx->lastLevel);
    return ESP_OK;
}

static void sensorarrayMeasureFdcPrepareIntbEpoch(sensorarrayFdcWorkerContext_t *ctx,
                                                  uint32_t epochId)
{
    if (!ctx) {
        return;
    }
    ctx->currentEpoch = epochId;
    ctx->lastEpochSeen = 0u;
    ctx->lastEdgeUs = 0;
    ctx->waitTask = NULL;
    if (ctx->task) {
        xTaskNotifyStateClear(ctx->task);
    }
}

static uint32_t sensorarrayMeasureFdcWorkerEdgeCount(sensorarrayFdcDeviceId_t devId)
{
    sensorarrayFdcWorkerContext_t *ctx = sensorarrayMeasureFdcWorkerContext(devId);
    return ctx ? ctx->edgeCount : 0u;
}

static bool sensorarrayMeasureResolveWorkerCore(const char *workerName,
                                                int configuredCore,
                                                int defaultCore,
                                                BaseType_t *outCore,
                                                bool *outPinned,
                                                const char **outReason)
{
    (void)workerName;
    if (!outCore || !outPinned || !outReason) {
        return false;
    }

    int processors = (int)portNUM_PROCESSORS;
    if (processors <= 0) {
        processors = 1;
    }

    if (CONFIG_FREERTOS_UNICORE || processors <= 1) {
        *outCore = 0;
        *outPinned = true;
        *outReason = "unicore_core0";
        return true;
    }

    if (configuredCore >= 0 && configuredCore < processors) {
        *outCore = (BaseType_t)configuredCore;
        *outPinned = true;
        *outReason = "configured";
        return true;
    }

    if (configuredCore < 0) {
        *outCore = (BaseType_t)-1;
        *outPinned = false;
        *outReason = "no_affinity_static";
        return true;
    }

    int normalizedCore = (defaultCore >= 0 && defaultCore < processors) ? defaultCore : 0;
    *outCore = (BaseType_t)normalizedCore;
    *outPinned = true;
    *outReason = "out_of_range_normalized";
    return true;
}

static void sensorarrayMeasureCleanupFdcWorkers(void)
{
    for (uint8_t i = 0u; i < 2u; ++i) {
        sensorarrayFdcWorkerContext_t *ctx = &s_fdcWorkers[i];
        if (ctx->task) {
            vTaskDelete(ctx->task);
            ctx->task = NULL;
        }
        ctx->initialized = false;
        ctx->waitTask = NULL;
        if (ctx->queue) {
            vQueueDelete(ctx->queue);
            ctx->queue = NULL;
        }
        if (ctx->sleepAck) {
            vSemaphoreDelete(ctx->sleepAck);
            ctx->sleepAck = NULL;
        }
        if (ctx->start) {
            vSemaphoreDelete(ctx->start);
            ctx->start = NULL;
        }
        if (ctx->done) {
            vSemaphoreDelete(ctx->done);
            ctx->done = NULL;
        }
    }
}

static const char *sensorarrayMeasureFdcParallelFallbackReason(bool configEnabled,
                                                              bool primaryBusEnabled,
                                                              bool secondaryBusEnabled,
                                                              bool sameBus,
                                                              esp_err_t err)
{
    if (!configEnabled) {
        return "config_disabled";
    }
    if (!primaryBusEnabled || !secondaryBusEnabled) {
        return "bus_unavailable";
    }
    if (sameBus) {
        return "same_bus";
    }
    if (s_fdcWorkersInitAttempted && !s_fdcWorkersAvailable) {
        return "worker_init_failed";
    }
    if (err == ESP_ERR_TIMEOUT) {
        return "timeout";
    }
    if (err == ESP_ERR_INVALID_STATE || err == ESP_ERR_NOT_SUPPORTED) {
        return "worker_unavailable";
    }
    return "parallel_error";
}

#include "fdc/sensorarrayFdcRowEpoch.inc"

#include "fdc/sensorarrayFdcFrameBuild.inc"
static void sensorarrayMeasureFillFdcDeviceI2cDelta(const Fdc2214CapI2cStats_t *before,
                                                    const Fdc2214CapI2cStats_t *after,
                                                    sensorarrayFdcDeviceTiming_t *timing)
{
    if (!before || !after || !timing) {
        return;
    }

    timing->regWriteCount = after->writeCount - before->writeCount;
    timing->regReadCount = after->readCount - before->readCount;
    timing->verifyReadCount = after->verifyReadCount - before->verifyReadCount;
    timing->retryCount = after->retryCount - before->retryCount;
    timing->nackCount = after->nackCount - before->nackCount;
    timing->timeoutCount = after->timeoutCount - before->timeoutCount;
}

esp_err_t sensorarrayMeasureReadFdcMatrixFrame(sensorarrayState_t *state,
                                               sensorarrayFdcMatrixFrame_t *outFrame)
{
    if (!outFrame) {
        return ESP_ERR_INVALID_ARG;
    }

    sensorarrayMeasureInitFdcMatrixFrame(outFrame);

    if (!state) {
        outFrame->errorMask = UINT64_MAX;
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = sensorarrayMeasureTakeLock();
    if (err != ESP_OK) {
        outFrame->errorMask = UINT64_MAX;
        return err;
    }

    int64_t frameStartUs = esp_timer_get_time();
    sensorarrayFdcTimingSummary_t timing = {
        .rowMinUs = UINT64_MAX,
    };
    uint64_t rowTotalUs = 0u;

    esp_err_t firstErr = sensorarrayMeasureCheckFdcMatrixReady(state);
    if (firstErr != ESP_OK) {
        outFrame->errorMask = UINT64_MAX;
        printf("MATRIXFDC_DIAG,stage=read_abort,reason=matrix_not_ready,err=0x%lx\n",
               (unsigned long)firstErr);
        sensorarrayMeasureGiveLock();
        return firstErr;
    }

    Fdc2214CapResetI2cStats(state->fdcPrimary.handle);
    if (sensorarrayMeasureFdcDeviceReadyForIo(&state->fdcSecondary)) {
        Fdc2214CapResetI2cStats(state->fdcSecondary.handle);
    }
    BoardSupportI2cBusInfo_t primaryBus = {0};
    BoardSupportI2cBusInfo_t secondaryBus = {0};
    (void)boardSupportGetI2cBusInfo(false, &primaryBus);
    (void)boardSupportGetI2cBusInfo(true, &secondaryBus);
    bool primaryBusEnabled = primaryBus.Enabled &&
                             sensorarrayMeasureFdcDeviceReadyForIo(&state->fdcPrimary);
    bool secondaryBusEnabled = secondaryBus.Enabled &&
                               sensorarrayMeasureFdcDeviceReadyForIo(&state->fdcSecondary);
    if (!secondaryBusEnabled) {
        sensorarrayMeasureLogFdcSecondaryUnavailableOnce();
    }
    bool sameBus = primaryBusEnabled &&
                   secondaryBusEnabled &&
                   primaryBus.Port == secondaryBus.Port;
    bool parallelConfigEnabled =
        CONFIG_SENSORARRAY_FDC_PARALLEL_DUAL_BUS_READ != 0 &&
        CONFIG_SENSORARRAY_FDC_PARALLEL_DUAL_BUS_READ_SAFE != 0 &&
        CONFIG_SENSORARRAY_FDC_FORCE_SINGLE_THREAD_READ == 0;
    bool parallelEligible = parallelConfigEnabled &&
                            primaryBusEnabled &&
                            secondaryBusEnabled &&
                            !sameBus &&
                            (!s_fdcWorkersInitAttempted || s_fdcWorkersAvailable);
    if (!s_fdcParallelConfigLogged) {
        s_fdcParallelConfigLogged = true;
        printf("FDC_PARALLEL_CONFIG,parallelEnabled=%u,forceSingleThread=%u,disableAfterTimeoutInFrame=%u,retryIntervalFrames=%lu,cooldownFrames=%lu,workerSyncTimeoutMs=%lu,primaryBus=%d,secondaryBus=%d,sameBus=%u,scanTaskCore=%d,scanTaskPriority=%d,workerTaskCore=%d,workerTaskPriority=%d\n",
               parallelConfigEnabled ? 1u : 0u,
               CONFIG_SENSORARRAY_FDC_FORCE_SINGLE_THREAD_READ ? 1u : 0u,
               CONFIG_SENSORARRAY_FDC_PARALLEL_DISABLE_AFTER_TIMEOUT_IN_FRAME ? 1u : 0u,
               (unsigned long)CONFIG_SENSORARRAY_FDC_PARALLEL_RETRY_INTERVAL_FRAMES,
               (unsigned long)s_fdcParallelCooldownFrames,
               (unsigned long)CONFIG_SENSORARRAY_FDC_WORKER_SYNC_TIMEOUT_MS,
               primaryBusEnabled ? (int)primaryBus.Port : -1,
               secondaryBusEnabled ? (int)secondaryBus.Port : -1,
               sameBus ? 1u : 0u,
               CONFIG_SENSORARRAY_SCAN_TASK_CORE,
               CONFIG_SENSORARRAY_SCAN_TASK_PRIO,
               CONFIG_SENSORARRAY_FDC_WORKER_TASK_CORE,
               CONFIG_SENSORARRAY_FDC_WORKER_TASK_PRIO);
    }
    bool parallelAllowedForFrame = parallelEligible && s_fdcParallelCooldownFrames == 0u;
    bool parallelDisabledForThisFrame = false;

    int64_t stageStartUs = esp_timer_get_time();
    firstErr = sensorarrayMeasureEnsureFdcMatrixPath(state, "fdc_matrix_frame");
    timing.pathEnsureUs += sensorarrayMeasureElapsedUs(stageStartUs);
    if (firstErr != ESP_OK) {
        outFrame->errorMask = UINT64_MAX;
        printf("MATRIXFDC_DIAG,stage=read_abort,reason=path_prepare_failed,err=0x%lx\n",
               (unsigned long)firstErr);
        sensorarrayMeasureGiveLock();
        return firstErr;
    }

    if (parallelEligible && !s_fdcWorkersInitAttempted) {
        esp_err_t workerErr = sensorarrayMeasureEnsureFdcWorkers();
        if (workerErr != ESP_OK) {
            printf("FDC_PARALLEL_FALLBACK,reason=%s,row=0,epoch=%lu,err=0x%lx,primaryBus=%d,secondaryBus=%d,sameBus=%u\n",
                   sensorarrayMeasureFdcParallelFallbackReason(parallelConfigEnabled,
                                                              primaryBusEnabled,
                                                              secondaryBusEnabled,
                                                              sameBus,
                                                              workerErr),
                   (unsigned long)s_fdcRowEpoch,
                   (unsigned long)workerErr,
                   primaryBusEnabled ? (int)primaryBus.Port : -1,
                   secondaryBusEnabled ? (int)secondaryBus.Port : -1,
                   sameBus ? 1u : 0u);
            parallelEligible = false;
        }
    }
    if (!s_fdcFormalPrecheckDone) {
        esp_err_t precheckErr = ESP_OK;
        if (secondaryBusEnabled) {
            precheckErr = sensorarrayMeasureRunFdcFormalPrecheck(state);
        } else {
            printf("FDC_FORMAL_PRECHECK,stage=skip,reason=secondary_unavailable,action=primary_only\n");
        }
        s_fdcFormalPrecheckDone = true;
        sensorarrayMeasureRecordFirstErr(precheckErr, &firstErr);
    }

    sensorarrayFdcFrameHealth_t frameHealth = {0};
    sensorarrayFdcFrameReadTracker_t readTracker = {
        .frameSeq = outFrame->sequence,
    };
    for (uint8_t s = 1u; s <= SENSORARRAY_MATRIX_ROWS; ++s) {
        sensorarrayFdcRowTiming_t rowTiming = {
            .row = s,
        };
        sensorarrayFdcDeviceTiming_t primaryTiming = {
            .row = s,
            .deviceId = SENSORARRAY_FDC_DEV_PRIMARY,
        };
        sensorarrayFdcDeviceTiming_t secondaryTiming = {
            .row = s,
            .deviceId = SENSORARRAY_FDC_DEV_SECONDARY,
        };
        Fdc2214CapI2cStats_t primaryStatsBefore = {0};
        Fdc2214CapI2cStats_t secondaryStatsBefore = {0};
        Fdc2214CapGetI2cStats(state->fdcPrimary.handle, &primaryStatsBefore);
        if (sensorarrayMeasureFdcDeviceReadyForIo(&state->fdcSecondary)) {
            Fdc2214CapGetI2cStats(state->fdcSecondary.handle, &secondaryStatsBefore);
        }
        uint32_t primaryEdgeBefore = sensorarrayMeasureFdcWorkerEdgeCount(SENSORARRAY_FDC_DEV_PRIMARY);
        uint32_t secondaryEdgeBefore = sensorarrayMeasureFdcWorkerEdgeCount(SENSORARRAY_FDC_DEV_SECONDARY);

        int64_t rowStartUs = esp_timer_get_time();
        uint32_t epochId = ++s_fdcRowEpoch;
        sensorarrayFdcRuntimeChannelConfig_t runtimeConfigs[2][4] = {0};
        sensorarrayFdcAutoscanSamples_t primarySamples = {0};
        sensorarrayFdcAutoscanSamples_t secondarySamples = {0};
        uint32_t duplicateReadCountBeforeRow = readTracker.duplicateReadCount;

        esp_err_t rowErr = ESP_ERR_NOT_SUPPORTED;
        bool rowHandledByParallel = false;
        bool useParallelThisRow = parallelAllowedForFrame && !parallelDisabledForThisFrame;
        if (useParallelThisRow) {
            bool rowWorkerTimeout = false;
            rowErr = sensorarrayMeasureReadFdcMatrixRowParallelEpoch(state,
                                                                     s,
                                                                     epochId,
                                                                     outFrame->sequence,
                                                                     &primarySamples,
                                                                     &secondarySamples,
                                                                     runtimeConfigs,
                                                                     &primaryTiming,
                                                                     &secondaryTiming,
                                                                     &rowTiming,
                                                                     &readTracker,
                                                                     &rowWorkerTimeout);
            rowHandledByParallel = rowErr == ESP_OK;
            if (rowWorkerTimeout &&
                CONFIG_SENSORARRAY_FDC_PARALLEL_DISABLE_AFTER_TIMEOUT_IN_FRAME) {
                parallelDisabledForThisFrame = true;
                s_fdcParallelCooldownFrames =
                    (uint32_t)CONFIG_SENSORARRAY_FDC_PARALLEL_RETRY_INTERVAL_FRAMES;
                printf("FDC_PARALLEL_FRAME_DOWNGRADE,seq=%lu,row=%u,epoch=%lu,reason=worker_timeout,cooldownFrames=%lu,remainingRowsSerial=1\n",
                       (unsigned long)outFrame->sequence,
                       (unsigned)s,
                       (unsigned long)epochId,
                       (unsigned long)s_fdcParallelCooldownFrames);
            }
            if (rowErr != ESP_OK) {
                printf("FDC_PARALLEL_FALLBACK,reason=%s,row=%u,epoch=%lu,err=0x%lx,primaryBus=%d,secondaryBus=%d,sameBus=%u\n",
                       sensorarrayMeasureFdcParallelFallbackReason(parallelConfigEnabled,
                                                                  primaryBusEnabled,
                                                                  secondaryBusEnabled,
                                                                  sameBus,
                                                                  rowErr),
                       (unsigned)s,
                       (unsigned long)epochId,
                       (unsigned long)rowErr,
                       primaryBusEnabled ? (int)primaryBus.Port : -1,
                       secondaryBusEnabled ? (int)secondaryBus.Port : -1,
                       sameBus ? 1u : 0u);
                parallelDisabledForThisFrame = true;
            }
        }
        if (!rowHandledByParallel) {
            bool serialFallback = useParallelThisRow || parallelDisabledForThisFrame;
            int64_t serialStartUs = esp_timer_get_time();
            rowErr = sensorarrayMeasureReadFdcMatrixRowSerialEpoch(state,
                                                                   s,
                                                                   epochId,
                                                                   outFrame->sequence,
                                                                   &primarySamples,
                                                                   &secondarySamples,
                                                                   runtimeConfigs,
                                                                   &primaryTiming,
                                                                   &secondaryTiming,
                                                                   &rowTiming,
                                                                   &readTracker);
            if (serialFallback) {
                rowTiming.serialFallbackUs += sensorarrayMeasureElapsedUs(serialStartUs);
            }
        }
        if (rowErr != ESP_OK && firstErr == ESP_OK) {
            firstErr = rowErr;
        }
        uint64_t applyElapsedUs = primaryTiming.applyUs + secondaryTiming.applyUs;
        timing.cacheApplyUs += applyElapsedUs;
        timing.applyBuildConfigUs += primaryTiming.applyBuildConfigUs + secondaryTiming.applyBuildConfigUs;
        timing.applyChannelConfigWriteUs += primaryTiming.channelConfigWriteUs + secondaryTiming.channelConfigWriteUs;
        timing.applyGlobalConfigWriteUs += primaryTiming.globalConfigWriteUs + secondaryTiming.globalConfigWriteUs;
        timing.applyVerifyUs += primaryTiming.verifyUs + secondaryTiming.verifyUs;
        timing.discardUs += primaryTiming.discardUs + secondaryTiming.discardUs;
        timing.waitReadyUs += primaryTiming.waitReadyUs + secondaryTiming.waitReadyUs;
        timing.readUs += primaryTiming.readRawUs + secondaryTiming.readRawUs;

        uint8_t rowValidMask8 = 0u;
        uint8_t rowWarnMask8 = 0u;
        uint8_t rowErrorMask8 = 0u;
        sensorarrayMeasureFillFdcMatrixRow(outFrame,
                                           s,
                                           &primarySamples,
                                           &secondarySamples,
                                           runtimeConfigs,
                                           &rowValidMask8,
                                           &rowWarnMask8,
                                           &rowErrorMask8);
        sensorarrayMeasureAccumulateFdcHealth(&frameHealth,
                                              s,
                                              &primarySamples,
                                              &secondarySamples,
                                              runtimeConfigs,
                                              outFrame);

        bool rowPartial = rowValidMask8 != 0xFFu;
        uint8_t rowCacheMissMask8 = 0u;
        uint8_t rowTimeoutMask8 = 0u;
        for (uint8_t ch = 0u; ch < 4u; ++ch) {
            if (!runtimeConfigs[SENSORARRAY_FDC_DEV_PRIMARY][ch].valid) {
                rowCacheMissMask8 |= (uint8_t)(1u << ch);
            }
            if (!runtimeConfigs[SENSORARRAY_FDC_DEV_SECONDARY][ch].valid) {
                rowCacheMissMask8 |= (uint8_t)(1u << (ch + 4u));
            }
            if (primarySamples.timeout) {
                rowTimeoutMask8 |= (uint8_t)(1u << ch);
            }
            if (secondarySamples.timeout) {
                rowTimeoutMask8 |= (uint8_t)(1u << (ch + 4u));
            }
        }
#if CONFIG_SENSORARRAY_LOG_ROW_SUMMARY
        printf("FDC_ROW_SUMMARY,row=%u,epoch=%lu,primaryValid=0x%X,secondaryValid=0x%X,rowValidMask=0x%02X,rowWarnMask=0x%02X,rowErrorMask=0x%02X,cacheMissMask=0x%02X,timeoutMask=0x%02X,partial=%u,primaryStatus=0x%04X,primaryUnread=0x%X,secondaryStatus=0x%04X,secondaryUnread=0x%X\n",
               (unsigned)s,
               (unsigned long)epochId,
               (unsigned)(primarySamples.validMask & 0x0Fu),
               (unsigned)(secondarySamples.validMask & 0x0Fu),
               (unsigned)rowValidMask8,
               (unsigned)rowWarnMask8,
               (unsigned)rowErrorMask8,
               (unsigned)rowCacheMissMask8,
               (unsigned)rowTimeoutMask8,
               rowPartial ? 1u : 0u,
               primarySamples.statusRaw,
               (unsigned)(primarySamples.unreadMask & 0x0Fu),
               secondarySamples.statusRaw,
               (unsigned)(secondarySamples.unreadMask & 0x0Fu));
#endif

#if CONFIG_SENSORARRAY_FDC_VERBOSE_SCAN_LOG
        printf("FDC_MATRIX_ROW,row=%u,d1=%lu,d2=%lu,d3=%lu,d4=%lu,d5=%lu,d6=%lu,d7=%lu,d8=%lu,validMask8=0x%02X,warnMask8=0x%02X,errorMask8=0x%02X\n",
               (unsigned)s,
               (unsigned long)primarySamples.raw28[0],
               (unsigned long)primarySamples.raw28[1],
               (unsigned long)primarySamples.raw28[2],
               (unsigned long)primarySamples.raw28[3],
               (unsigned long)secondarySamples.raw28[0],
               (unsigned long)secondarySamples.raw28[1],
               (unsigned long)secondarySamples.raw28[2],
               (unsigned long)secondarySamples.raw28[3],
               (unsigned)rowValidMask8,
               (unsigned)rowWarnMask8,
               (unsigned)rowErrorMask8);
#endif
        rowTiming.rowValidMask = rowValidMask8;
        rowTiming.rowWarnMask = rowWarnMask8;
        rowTiming.rowErrorMask = rowErrorMask8;
        rowTiming.duplicateReadCount = readTracker.duplicateReadCount - duplicateReadCountBeforeRow;
        if (rowValidMask8 == 0u) {
            timing.rowFullInvalidCount++;
        }
        if (rowErrorMask8 != 0u && outFrame->firstBadStatus == 0u) {
            const sensorarrayFdcAutoscanSamples_t *badSamples =
                (primarySamples.validMask == 0u) ? &primarySamples : &secondarySamples;
            outFrame->firstBadStatus = badSamples->statusRaw;
            outFrame->firstBadUnread = badSamples->unreadMask;
        }
        rowTiming.rowUs = sensorarrayMeasureElapsedUs(rowStartUs);
        rowTotalUs += rowTiming.rowUs;
        if (rowTiming.rowUs > timing.rowMaxUs) {
            timing.rowMaxUs = rowTiming.rowUs;
            timing.slowRow = s;
        }
        if (rowTiming.rowUs < timing.rowMinUs) {
            timing.rowMinUs = rowTiming.rowUs;
        }
        if (primaryTiming.deviceUs == 0u) {
            primaryTiming.deviceUs = primaryTiming.sleepEnterUs +
                                     primaryTiming.applyUs +
                                     primaryTiming.sleepExitUs +
                                     primaryTiming.discardUs +
                                     primaryTiming.waitReadyUs +
                                     primaryTiming.readRawUs;
        }
        if (secondaryTiming.deviceUs == 0u) {
            secondaryTiming.deviceUs = secondaryTiming.sleepEnterUs +
                                       secondaryTiming.applyUs +
                                       secondaryTiming.sleepExitUs +
                                       secondaryTiming.discardUs +
                                       secondaryTiming.waitReadyUs +
                                       secondaryTiming.readRawUs;
        }
        rowTiming.primaryTotalUs = primaryTiming.deviceUs;
        rowTiming.secondaryTotalUs = secondaryTiming.deviceUs;
        uint32_t primaryEdgeAfter = sensorarrayMeasureFdcWorkerEdgeCount(SENSORARRAY_FDC_DEV_PRIMARY);
        uint32_t secondaryEdgeAfter = sensorarrayMeasureFdcWorkerEdgeCount(SENSORARRAY_FDC_DEV_SECONDARY);
        primaryTiming.intbEdgeCount = (primaryEdgeAfter >= primaryEdgeBefore) ?
            (primaryEdgeAfter - primaryEdgeBefore) :
            0u;
        secondaryTiming.intbEdgeCount = (secondaryEdgeAfter >= secondaryEdgeBefore) ?
            (secondaryEdgeAfter - secondaryEdgeBefore) :
            0u;
        sensorarrayMeasureAccumulateRowEpochTiming(&timing,
                                                   &rowTiming,
                                                   &primaryTiming,
                                                   &secondaryTiming);

        Fdc2214CapI2cStats_t primaryStatsAfter = {0};
        Fdc2214CapI2cStats_t secondaryStatsAfter = {0};
        Fdc2214CapGetI2cStats(state->fdcPrimary.handle, &primaryStatsAfter);
        if (sensorarrayMeasureFdcDeviceReadyForIo(&state->fdcSecondary)) {
            Fdc2214CapGetI2cStats(state->fdcSecondary.handle, &secondaryStatsAfter);
        }
        sensorarrayMeasureFillFdcDeviceI2cDelta(&primaryStatsBefore, &primaryStatsAfter, &primaryTiming);
        sensorarrayMeasureFillFdcDeviceI2cDelta(&secondaryStatsBefore, &secondaryStatsAfter, &secondaryTiming);

        if (s_fdcProfileRowEnabled) {
            sensorarrayMeasurePrintFdcRowTiming(outFrame->sequence, &rowTiming);
        }
        if (s_fdcProfileDeviceEnabled) {
            sensorarrayMeasurePrintFdcDeviceTiming(outFrame->sequence, &primaryTiming);
            sensorarrayMeasurePrintFdcDeviceTiming(outFrame->sequence, &secondaryTiming);
        }
        taskYIELD();
    }
    if (s_fdcParallelCooldownFrames > 0u) {
        s_fdcParallelCooldownFrames--;
    }
    sensorarrayMeasureUpdateFdcRuntimeProfiles(state, &frameHealth);
    sensorarrayMeasureCountFdcFrameWarnings(&frameHealth, &timing);

    timing.capComputeUs = sensorarrayMeasureComputeFdcFrameCapTotalPf(outFrame);
    timing.frameUs = sensorarrayMeasureElapsedUs(frameStartUs);
    timing.measureLockHeldUs = timing.frameUs;
    timing.rowAvgUs = rowTotalUs / SENSORARRAY_MATRIX_ROWS;
    if (timing.rowMinUs == UINT64_MAX) {
        timing.rowMinUs = 0u;
    }

    Fdc2214CapI2cStats_t primaryStats = {0};
    Fdc2214CapI2cStats_t secondaryStats = {0};
    Fdc2214CapGetI2cStats(state->fdcPrimary.handle, &primaryStats);
    if (sensorarrayMeasureFdcDeviceReadyForIo(&state->fdcSecondary)) {
        Fdc2214CapGetI2cStats(state->fdcSecondary.handle, &secondaryStats);
    }
    sensorarrayMeasureMergeFdcI2cStats(&primaryStats, &secondaryStats, &primaryBus, &secondaryBus, &timing);

    if (CONFIG_SENSORARRAY_FDC_TIMING_OVERRUN_IMMEDIATE_LOG &&
        timing.frameUs > SENSORARRAY_FDC_TARGET_FRAME_US) {
        sensorarrayMeasurePrintFdcBottleneck(&timing, outFrame->sequence);
    }

    if (s_fdcProfileSummaryEnabled) {
        if (CONFIG_SENSORARRAY_FDC_TIMING_VERBOSE_PER_FRAME) {
            uint32_t timingEvery = s_fdcTimingSummaryEvery;
            if (timingEvery != 0u && (outFrame->sequence % timingEvery) == 0u) {
                sensorarrayMeasurePrintFdcTimingSummary(&timing, outFrame->sequence);
            }
        }
        sensorarrayMeasureUpdateFdcTimingAggregate(&timing, outFrame->sequence);
    }

    if (Fdc2214CapI2cTraceIsEnabled() &&
        (timing.frameUs > SENSORARRAY_FDC_TARGET_FRAME_US ||
         timing.i2cRetryCount > 0u ||
         timing.i2cTimeoutCount > 0u ||
         timing.i2cNackCount > 0u ||
         outFrame->errorMask != 0u)) {
        Fdc2214CapI2cTraceDump();
    }

    sensorarrayMeasureGiveLock();
    if (outFrame->validMask == 0u) {
        bool allZero = sensorarrayFdcMatrixFrameRawAllZero(outFrame);
        bool runtimeReadyFault = outFrame->notReadyCount >= SENSORARRAY_MATRIX_CELL_COUNT ||
                                 outFrame->zeroBeforeReadyCount != 0u;
        uint32_t seq = s_fdcMatrixAllInvalidSequence++;
        const char *reason = runtimeReadyFault ? "all_invalid_due_to_not_ready" :
            allZero ? "all_zero_raw_after_drdy" :
            (outFrame->zeroAfterDrdyCount != 0u ? "all_invalid_due_to_zero_after_drdy" :
             outFrame->i2cErrorCount != 0u ? "all_invalid_due_to_i2c" :
            (outFrame->freshCount == 0u ? "all_invalid_no_fresh_data" :
             "normal_path_no_valid_after_boot_ok"));
        if (allZero && !runtimeReadyFault) {
            outFrame->errorMask = UINT64_MAX;
        }
        printf("MATRIXFDC_DIAG,stage=%s,seq=%lu,errorMask=0x%016llX,reason=%s,freshCount=%u,validCount=%u,hardwareZeroRawCount=%u,placeholderZeroCount=%u,notReadyCount=%u,zeroBeforeReadyCount=%u,zeroAfterDrdyCount=%u,i2cErrorCount=%u,unreadWithoutDrdyCount=%u,firstBadRow=%u,firstBadDevice=%u,firstBadStatus=0x%04X,firstBadUnread=0x%X\n",
               allZero ? "all_invalid" :
               runtimeReadyFault ? "all_invalid_ready_fault" : "all_status_invalid",
               (unsigned long)seq,
               (unsigned long long)outFrame->errorMask,
               reason,
               (unsigned)outFrame->freshCount,
               (unsigned)outFrame->validCount,
               (unsigned)outFrame->hardwareZeroRawCount,
               (unsigned)outFrame->placeholderZeroCount,
               (unsigned)outFrame->notReadyCount,
               (unsigned)outFrame->zeroBeforeReadyCount,
               (unsigned)outFrame->zeroAfterDrdyCount,
               (unsigned)outFrame->i2cErrorCount,
               (unsigned)outFrame->unreadWithoutDrdyCount,
               (unsigned)outFrame->firstBadRow,
               (unsigned)outFrame->firstBadDevice,
               outFrame->firstBadStatus,
               (unsigned)outFrame->firstBadUnread);
        sensorarrayFdcSweepReportAllInvalidFrame(outFrame->validMask,
                                                  outFrame->errorMask,
                                                  allZero ? outFrame->hardwareZeroRawCount : 0u,
                                                  outFrame->notReadyCount,
                                                  outFrame->zeroBeforeReadyCount,
                                                  outFrame->zeroAfterDrdyCount,
                                                  outFrame->i2cErrorCount);
        return (firstErr != ESP_OK) ? firstErr : ESP_ERR_INVALID_RESPONSE;
    }
    return firstErr;
}

bool sensorarrayFastSpeedIsEnabled(void)
{
    return s_fastSpeedEnabled;
}

void sensorarrayFastSpeedSetEnabled(bool enabled)
{
    s_fastSpeedEnabled = enabled;
}

void sensorarrayMeasureFdcProfileSetSummary(bool enabled)
{
    s_fdcProfileSummaryEnabled = enabled;
}

void sensorarrayMeasureFdcProfileSetRow(bool enabled)
{
    s_fdcProfileRowEnabled = enabled;
}

void sensorarrayMeasureFdcProfileSetDevice(bool enabled)
{
    s_fdcProfileDeviceEnabled = enabled;
}

void sensorarrayMeasureFdcProfileSetSummaryEvery(uint32_t everyNFrames)
{
    s_fdcTimingSummaryEvery = everyNFrames;
}

bool sensorarrayMeasureFdcProfileSummaryEnabled(void)
{
    return s_fdcProfileSummaryEnabled;
}

bool sensorarrayMeasureFdcProfileRowEnabled(void)
{
    return s_fdcProfileRowEnabled;
}

bool sensorarrayMeasureFdcProfileDeviceEnabled(void)
{
    return s_fdcProfileDeviceEnabled;
}

uint32_t sensorarrayMeasureFdcProfileSummaryEvery(void)
{
    return s_fdcTimingSummaryEvery;
}

esp_err_t sensorarrayMeasureFdcSetDiscardFrames(uint8_t discardFrames)
{
    if (discardFrames > 8u) {
        return ESP_ERR_INVALID_ARG;
    }
    s_fdcDiscardFrames = discardFrames;
    return ESP_OK;
}

uint8_t sensorarrayMeasureFdcDiscardFrames(void)
{
    return s_fdcDiscardFrames;
}

#include "ads/sensorarrayAdsMeasure.inc"
            *outHaveMohm = true;
        }
        return "divider_model_ok";
    }
    if (resResult == SENSORARRAY_RES_CONVERT_SIGNED_INPUT) {
        return "negative_uv";
    }
    return "divider_model_invalid";
}

#include "fdc/sensorarrayFdcSampleConvert.inc"
        return false;
    }

    const double cPf = (1.0 / denom) * 1e12;
    if (!isfinite(cPf) || cPf <= 0.0) {
        return false;
    }

    *outCapPf = cPf;
    return true;
}

#include "ads/sensorarrayAdsRegisters.inc"

#include "sensorarrayMeasure.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
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
#define SENSORARRAY_FDC_REG_STATUS_CONFIG 0x19u
#define SENSORARRAY_FDC_REG_CONFIG 0x1Au
#define SENSORARRAY_FDC_REG_MUX_CONFIG 0x1Bu
#define SENSORARRAY_FDC_REG_DRIVE_CURRENT_BASE 0x1Eu
#define SENSORARRAY_FDC_RAW28_SATURATED_THRESHOLD 0x0FFFFF00u
#define SENSORARRAY_FDC_AMPLITUDE_RESCUE_THRESHOLD 4u
#define SENSORARRAY_FDC_CELL_ROUTE_DISCARD_COUNT 2u
#define SENSORARRAY_FDC_TARGET_FRAME_US (1000000u / CONFIG_SENSORARRAY_FDC_MATRIX_TARGET_FPS)
#define SENSORARRAY_FDC_TARGET_ROW_US (SENSORARRAY_FDC_TARGET_FRAME_US / SENSORARRAY_MATRIX_ROWS)
#define SENSORARRAY_FDC_CONFIG_SLEEP_MODE_EN_MASK 0x2000u
#define SENSORARRAY_FDC_CONFIG_INTB_DIS_MASK 0x0080u
#define SENSORARRAY_FDC_DRIVE_CURRENT_MASK 0xF800u
#define SENSORARRAY_FDC_MUX_FIXED_MASK 0x0208u
#define SENSORARRAY_FDC_WORKER_QUEUE_DEPTH 1u
#define SENSORARRAY_FDC_WORKER_STACK_WORDS \
    ((CONFIG_SENSORARRAY_FDC_WORKER_TASK_STACK + sizeof(StackType_t) - 1u) / sizeof(StackType_t))

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
    uint16_t statusRaw;
    uint8_t unreadMask;
    uint8_t freshMask;
    uint8_t validMask;
    uint8_t warnMask;
    uint8_t errorMask;
    bool timeout;
    bool partial;
    bool i2cTransactionError;
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
    bool ready;
    bool dataReady;
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
    uint32_t timeoutCount;
    uint32_t waitUs;
    bool timeout;
    bool partial;
    bool i2cError;
} sensorarrayFdcReadyState_t;

typedef struct {
    uint32_t raw28[4];
    double freqHz[4];
    double capTotalPf[4];
    uint8_t freshMask4;
    uint8_t validMask4;
    uint8_t warnMask4;
    uint8_t errorMask4;
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
    sensorarrayState_t *state;
    uint8_t row;
    uint32_t epochId;
    sensorarrayFdcAutoscanSamples_t *outSamples;
    sensorarrayFdcRuntimeChannelConfig_t *outConfigs;
    sensorarrayFdcDeviceTiming_t *timing;
    sensorarrayFdcWorkerResult_t *result;
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
static uint32_t s_fdcRowEpoch = 0u;
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
    if (!state || !state->tmuxReady || !state->adsReady) {
        printf("FDC_PATH,stage=prepare_done,reason=%s,ok=0,err=0x%lx,sw=-1,sela=-1,selb=-1,en=-1,adsRef=-1,adsVbias=-1\n",
               source,
               (unsigned long)ESP_ERR_INVALID_STATE);
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = sensorarrayMeasureStopAdsBeforeRoute(state);
    printf("FDC_PATH,stage=ads_stop,reason=%s,err=0x%lx\n", source, (unsigned long)err);
    if (err != ESP_OK) {
        return err;
    }
#if CONFIG_SENSORARRAY_ADS1263
    esp_err_t adc2Err = ads126xAdcStopAdc2(&state->ads);
    printf("FDC_PATH,stage=ads_stop2,reason=%s,err=0x%lx\n", source, (unsigned long)adc2Err);
    if (adc2Err != ESP_OK && adc2Err != ESP_ERR_NOT_SUPPORTED) {
        return adc2Err;
    }
#endif

    err = sensorarrayMeasureForceAdsReferenceOff(state);
    printf("FDC_PATH,stage=ads_ref_off,reason=%s,err=0x%lx,adsRefReady=%d\n",
           source,
           (unsigned long)err,
           state->adsRefReady ? 1 : 0);
    printf("FDC_PATH,stage=ads_vbias_off,reason=%s,err=0x%lx\n", source, (unsigned long)err);
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
    printf("FDC_PATH,stage=tmux1134_fdc,reason=%s,selaCmd=%d,selaReadback=%d,err=0x%lx\n",
           source,
           fdcSelaLevel,
           ctrl.obsSelaLevel,
           (unsigned long)err);
    if (err != ESP_OK) {
        return err;
    }

    err = sensorarrayMeasureSetFdcSelBPathQuiet(state);
    sensorarrayMeasureReadFdcPathControl(&ctrl);
    bool fdcSelbLevel = false;
    (void)sensorarrayBoardMapFdcSelBLevel(&fdcSelbLevel);
    printf("FDC_PATH,stage=selb_fdc,reason=%s,selbCmd=%d,selbReadback=%d,err=0x%lx\n",
           source,
           fdcSelbLevel ? 1 : 0,
           ctrl.obsSelbLevel,
           (unsigned long)err);
    if (err != ESP_OK) {
        return err;
    }

    err = tmuxSwitchSet1108Source(TMUX1108_SOURCE_GND);
    sensorarrayMeasureReadFdcPathControl(&ctrl);
    printf("FDC_PATH,stage=sw_gnd,reason=%s,swCmd=GND,swReadback=%d,err=0x%lx\n",
           source,
           sensorarrayMeasureSwPhysicalReadbackFromControl(&ctrl),
           (unsigned long)err);
    if (err != ESP_OK) {
        return err;
    }

    err = tmux1134SetEnLogicalState(true);
    sensorarrayMeasureReadFdcPathControl(&ctrl);
    printf("FDC_PATH,stage=tmux1108_enable,reason=%s,enCmd=1,enReadback=%d,err=0x%lx\n",
           source,
           ctrl.obsTmux1134EnLogicalOnValid ? (ctrl.obsTmux1134EnLogicalOn ? 1 : 0) : (ctrl.cmdTmux1134EnLogicalOn ? 1 : 0),
           (unsigned long)err);

    bool ok = (err == ESP_OK) && sensorarrayMeasureFdcPathControlMatches(&ctrl);
    printf("FDC_PATH,stage=prepare_done,reason=%s,ok=%d,err=0x%lx,sw=%d,sela=%d,selb=%d,en=%d,adsRef=%d,adsVbias=0\n",
           source,
           ok ? 1 : 0,
           (unsigned long)(ok ? ESP_OK : ESP_ERR_INVALID_STATE),
           sensorarrayMeasureSwPhysicalReadbackFromControl(&ctrl),
           ctrl.obsSelaLevel,
           ctrl.obsSelbLevel,
           ctrl.obsTmux1134EnLogicalOnValid ? (ctrl.obsTmux1134EnLogicalOn ? 1 : 0) : (ctrl.cmdTmux1134EnLogicalOn ? 1 : 0),
           state->adsRefReady ? 1 : 0);
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
    return sensorarrayMeasureFdcReasonEquals(reason, "persistent_fresh_amplitude_warning_after_cache_apply");
}

static bool sensorarrayMeasureFdcRescueReasonIsHard(const char *reason)
{
    return sensorarrayMeasureFdcReasonEquals(reason, "cache_missing_and_hard_error") ||
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
    if (CONFIG_SENSORARRAY_FDC_INTB_ENABLE) {
        config &= (uint16_t)~SENSORARRAY_FDC_CONFIG_INTB_DIS_MASK;
    } else {
        config |= SENSORARRAY_FDC_CONFIG_INTB_DIS_MASK;
    }
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

static void sensorarrayMeasureRuntimeConfigsFromApplied(sensorarrayState_t *state,
                                                        sensorarrayFdcDeviceId_t devId,
                                                        sensorarrayFdcRuntimeChannelConfig_t configs[4])
{
    if (!state || !configs || devId > SENSORARRAY_FDC_DEV_SECONDARY) {
        return;
    }

    sensorarrayFdcDeviceState_t *fdcState = sensorarrayMeasureGetFdcState(state, devId);
    uint32_t effectiveFclkHz =
        (fdcState && fdcState->refClockKnown && fdcState->refClockHz != 0u) ?
        fdcState->refClockHz :
        sensorarrayMeasureFdcEffectiveFclkHz();
    const sensorarrayFdcAppliedRowConfig_t *applied = &state->fdcAppliedRow[(uint8_t)devId];
    for (uint8_t ch = 0u; ch < 4u; ++ch) {
        configs[ch] = (sensorarrayFdcRuntimeChannelConfig_t){
            .valid = applied->valid,
            .rCount = applied->rCount[ch],
            .settleCount = applied->settleCount[ch],
            .clockDividers = applied->clockDiv[ch],
            .driveCurrent = applied->driveCurrent[ch],
            .deglitchCode = applied->selectedDeglitch,
            .effectiveFclkHz = effectiveFclkHz,
        };
    }
}

static esp_err_t sensorarrayMeasureApplyFdcCachedRowConfig(sensorarrayState_t *state,
                                                           uint8_t row,
                                                           sensorarrayFdcDeviceId_t devId,
                                                           const char *reason,
                                                           bool forceWrite,
                                                           sensorarrayFdcDeviceTiming_t *timing)
{
    if (!state || row < 1u || row > SENSORARRAY_MATRIX_ROWS ||
        devId > SENSORARRAY_FDC_DEV_SECONDARY) {
        return ESP_ERR_INVALID_ARG;
    }

    sensorarrayFdcDeviceState_t *fdcState = sensorarrayMeasureGetFdcState(state, devId);
    if (!fdcState || !fdcState->ready || !fdcState->handle) {
        return ESP_ERR_INVALID_STATE;
    }
    int64_t applyStartUs = esp_timer_get_time();
    if (timing) {
        timing->row = row;
        timing->deviceId = devId;
    }

    int64_t buildStartUs = esp_timer_get_time();
    sensorarrayFdcAppliedRowConfig_t expected = {
        .valid = true,
        .autoscanConfigured = true,
        .row = row,
        .deviceId = (uint8_t)devId,
        .selectedDeglitch = sensorarrayMeasureFdcSafeDefaultDeglitch(),
    };
    uint8_t missingMask = 0u;
    bool reapplyRequested = false;

    for (uint8_t ch = 0u; ch < 4u; ++ch) {
        uint8_t dIndex = 0u;
        sensorarrayFdcCellConfigCache_t *cache =
            sensorarrayMeasureFdcRowDeviceCache(state, row, devId, ch, &dIndex);
        bool useCache = cache && cache->valid;
        if (!useCache) {
            missingMask |= (uint8_t)(1u << ch);
        }
        if (cache && cache->reapplyPending) {
            reapplyRequested = true;
        }

        expected.rCount[ch] = (useCache && cache->rCount != 0u) ?
            cache->rCount :
            SENSORARRAY_FDC_RCOUNT;
        expected.settleCount[ch] = (useCache && cache->settleCount != 0u) ?
            cache->settleCount :
            SENSORARRAY_FDC_SETTLECOUNT;
        expected.clockDiv[ch] = (useCache && cache->clockDiv != 0u) ?
            cache->clockDiv :
            SENSORARRAY_FDC_CLOCK_DIVIDERS;
        expected.driveCurrent[ch] = (useCache && cache->driveCurrent != 0u) ?
            cache->driveCurrent :
            SENSORARRAY_FDC_DRIVE_CURRENT;
        expected.cacheGeneration[ch] = useCache ? cache->generation : 0u;
        (void)dIndex;
    }

    expected.selectedDeglitch =
        sensorarrayFdcMergeDeglitchForRowDevice(state,
                                                row,
                                                devId,
                                                sensorarrayMeasureFdcSafeDefaultDeglitch());
    expected.muxConfig = sensorarrayMeasureFdcBuildMuxConfig(expected.selectedDeglitch);
    expected.statusConfig = SENSORARRAY_FDC_STATUS_CONFIG_DEFAULT;
    expected.configBaseWithoutSleepBit = sensorarrayMeasureFdcConfigBaseWithoutSleep(fdcState);
    expected.fingerprint = sensorarrayMeasureFdcConfigFingerprint(&expected);
    if (timing) {
        timing->applyBuildConfigUs += sensorarrayMeasureElapsedUs(buildStartUs);
    }

    if (missingMask != 0u) {
        printf("FDC_CACHE,stage=miss,row=%u,device=%s,missingMask=0x%X,action=use_safe_default\n",
               (unsigned)row,
               sensorarrayMeasureFdcDeviceName(devId),
               (unsigned)missingMask);
    }

    sensorarrayFdcAppliedRowConfig_t *applied = &state->fdcAppliedRow[(uint8_t)devId];
    if (!forceWrite &&
        !reapplyRequested &&
        sensorarrayMeasureAppliedRowConfigMatches(applied, &expected)) {
#if CONFIG_SENSORARRAY_FDC_VERBOSE_SCAN_LOG
        printf("FDC_CACHE,stage=hit,row=%u,device=%s,action=no_write\n",
               (unsigned)row,
               sensorarrayMeasureFdcDeviceName(devId));
#endif
        if (timing) {
            timing->applyUs += sensorarrayMeasureElapsedUs(applyStartUs);
            timing->cacheNoDiffCount++;
        }
        return ESP_OK;
    }

    esp_err_t firstErr = ESP_OK;
    uint32_t diffWriteCount = 0u;
    uint32_t diffRcountWrites = 0u;
    uint32_t diffSettleWrites = 0u;
    uint32_t diffClockWrites = 0u;
    uint32_t diffDriveWrites = 0u;
    uint32_t diffMuxWrites = 0u;
    uint32_t diffStatusWrites = 0u;
    uint32_t diffConfigWrites = 0u;
    const bool fullWrite = !applied->valid;

#define SENSORARRAY_FDC_WRITE_DIFF(reg_, desired_, appliedValue_, counter_) \
    do { \
        uint16_t desiredValue__ = (uint16_t)(desired_); \
        bool needWrite__ = !applied->valid || ((uint16_t)(appliedValue_) != desiredValue__); \
        if (needWrite__) { \
            int64_t writeStartUs__ = esp_timer_get_time(); \
            esp_err_t writeErr__ = Fdc2214CapWriteRawRegisters(fdcState->handle, (uint8_t)(reg_), desiredValue__); \
            uint64_t writeUs__ = sensorarrayMeasureElapsedUs(writeStartUs__); \
            if (timing) { \
                timing->channelConfigWriteUs += writeUs__; \
            } \
            if (writeErr__ != ESP_OK && firstErr == ESP_OK) { \
                firstErr = writeErr__; \
            } \
            if (writeErr__ == ESP_OK) { \
                diffWriteCount++; \
                (counter_)++; \
            } \
        } \
    } while (0)

    for (uint8_t ch = 0u; ch < 4u; ++ch) {
        SENSORARRAY_FDC_WRITE_DIFF(sensorarrayMeasureFdcRegForChannel(SENSORARRAY_FDC_REG_RCOUNT_BASE,
                                                                      (Fdc2214CapChannel_t)ch),
                                   expected.rCount[ch],
                                   applied->rCount[ch],
                                   diffRcountWrites);
        SENSORARRAY_FDC_WRITE_DIFF(sensorarrayMeasureFdcRegForChannel(SENSORARRAY_FDC_REG_SETTLECOUNT_BASE,
                                                                      (Fdc2214CapChannel_t)ch),
                                   expected.settleCount[ch],
                                   applied->settleCount[ch],
                                   diffSettleWrites);
        SENSORARRAY_FDC_WRITE_DIFF(sensorarrayMeasureFdcRegForChannel(SENSORARRAY_FDC_REG_CLOCK_DIVIDERS_BASE,
                                                                      (Fdc2214CapChannel_t)ch),
                                   expected.clockDiv[ch],
                                   applied->clockDiv[ch],
                                   diffClockWrites);
        SENSORARRAY_FDC_WRITE_DIFF(sensorarrayMeasureFdcRegForChannel(SENSORARRAY_FDC_REG_DRIVE_CURRENT_BASE,
                                                                      (Fdc2214CapChannel_t)ch),
                                   (uint16_t)(expected.driveCurrent[ch] & SENSORARRAY_FDC_DRIVE_CURRENT_MASK),
                                   applied->driveCurrent[ch],
                                   diffDriveWrites);
    }

    int64_t globalWriteStartUs = esp_timer_get_time();
    SENSORARRAY_FDC_WRITE_DIFF(SENSORARRAY_FDC_REG_STATUS_CONFIG,
                               expected.statusConfig,
                               applied->statusConfig,
                               diffStatusWrites);
    SENSORARRAY_FDC_WRITE_DIFF(SENSORARRAY_FDC_REG_MUX_CONFIG,
                               expected.muxConfig,
                               applied->muxConfig,
                               diffMuxWrites);
    SENSORARRAY_FDC_WRITE_DIFF(SENSORARRAY_FDC_REG_CONFIG,
                               (uint16_t)(expected.configBaseWithoutSleepBit |
                                          SENSORARRAY_FDC_CONFIG_SLEEP_MODE_EN_MASK),
                               (uint16_t)(applied->configBaseWithoutSleepBit |
                                          SENSORARRAY_FDC_CONFIG_SLEEP_MODE_EN_MASK),
                               diffConfigWrites);
    if (timing) {
        timing->globalConfigWriteUs += sensorarrayMeasureElapsedUs(globalWriteStartUs);
    }
#undef SENSORARRAY_FDC_WRITE_DIFF

    if (firstErr == ESP_OK) {
        fdcState->statusConfigReg = expected.statusConfig;
        fdcState->muxConfigReg = expected.muxConfig;
        fdcState->configReg = (uint16_t)(expected.configBaseWithoutSleepBit |
                                         SENSORARRAY_FDC_CONFIG_SLEEP_MODE_EN_MASK);
        fdcState->configVerified = true;
    }

    if (firstErr != ESP_OK) {
        applied->dirty = true;
        printf("FDC_CACHE,stage=apply_row_failed,row=%u,device=%s,reason=%s,err=0x%lx\n",
               (unsigned)row,
               sensorarrayMeasureFdcDeviceName(devId),
               reason ? reason : SENSORARRAY_NA,
               (unsigned long)firstErr);
        if (timing) {
            timing->applyUs += sensorarrayMeasureElapsedUs(applyStartUs);
        }
        return firstErr;
    }

    uint32_t previousFingerprint = applied->fingerprint;
    uint32_t nextApplyCount = applied->applyCount + 1u;
    expected.applyCount = nextApplyCount;
    expected.lastAppliedTimestampUs = esp_timer_get_time();
    expected.dirty = false;
    *applied = expected;
    if (timing) {
        timing->cacheDiffWriteCount += diffWriteCount;
        timing->cacheFullWriteCount += fullWrite ? 1u : 0u;
        timing->cacheNoDiffCount += (diffWriteCount == 0u) ? 1u : 0u;
        timing->diffRcountWrites += diffRcountWrites;
        timing->diffSettleWrites += diffSettleWrites;
        timing->diffClockDivWrites += diffClockWrites;
        timing->diffDriveWrites += diffDriveWrites;
        timing->diffMuxWrites += diffMuxWrites;
        timing->diffStatusConfigWrites += diffStatusWrites;
        timing->diffConfigWrites += diffConfigWrites;
        timing->appliedFingerprintChanged += (previousFingerprint != expected.fingerprint) ? 1u : 0u;
    }

    for (uint8_t ch = 0u; ch < 4u; ++ch) {
        sensorarrayFdcCellConfigCache_t *cache =
            sensorarrayMeasureFdcRowDeviceCache(state, row, devId, ch, NULL);
        if (cache) {
            cache->lastAppliedTimestampUs = applied->lastAppliedTimestampUs;
            cache->lastAppliedFingerprint = applied->fingerprint;
            cache->reapplyPending = false;
        }
    }

#if CONFIG_SENSORARRAY_FDC_CACHE_APPLY_VERBOSE_LOG
    const bool printApplyLog = true;
#else
    const bool printApplyLog = diffWriteCount != 0u;
#endif
    if (printApplyLog) {
    printf("FDC_CACHE,stage=apply_row,row=%u,device=%s,reason=%s,drive=[0x%04X,0x%04X,0x%04X,0x%04X],rCount=[0x%04X,0x%04X,0x%04X,0x%04X],settle=[0x%04X,0x%04X,0x%04X,0x%04X],clockDiv=[0x%04X,0x%04X,0x%04X,0x%04X],deglitch=0x%X,force=%u\n",
           (unsigned)row,
           sensorarrayMeasureFdcDeviceName(devId),
           reason ? reason : SENSORARRAY_NA,
           applied->driveCurrent[0],
           applied->driveCurrent[1],
           applied->driveCurrent[2],
           applied->driveCurrent[3],
           applied->rCount[0],
           applied->rCount[1],
           applied->rCount[2],
           applied->rCount[3],
           applied->settleCount[0],
           applied->settleCount[1],
           applied->settleCount[2],
           applied->settleCount[3],
           applied->clockDiv[0],
           applied->clockDiv[1],
           applied->clockDiv[2],
           applied->clockDiv[3],
           (unsigned)applied->selectedDeglitch,
           forceWrite || reapplyRequested ? 1u : 0u);
    }
    if (timing) {
        timing->applyUs += sensorarrayMeasureElapsedUs(applyStartUs);
    }
    return ESP_OK;
}

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
            uint8_t unreadMask = sensorarrayMeasureFdcUnreadMaskFromStatus(&status);
            if (status.DataReady || unreadMask == SENSORARRAY_FDC_AUTOSCAN_READY_UNREAD_MASK) {
                if (outStatus) {
                    *outStatus = status.Raw;
                }
#if CONFIG_SENSORARRAY_FDC_VERBOSE_SCAN_LOG
                printf("FDC_FRAME_READY,row=%u,device=%s,status=0x%04X,unreadMask=0x%X,drdy=%u,err=0x%lx\n",
                       (unsigned)row,
                       sensorarrayMeasureFdcDeviceName(sensorarrayMeasureFdcDeviceIdFromState(fdcState)),
                       status.Raw,
                       (unsigned)unreadMask,
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
    ready->unreadMask = sensorarrayMeasureFdcUnreadMaskFromStatus(&status);
    ready->ready = ready->dataReady ||
                   ready->unreadMask == SENSORARRAY_FDC_AUTOSCAN_READY_UNREAD_MASK;
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

static esp_err_t sensorarrayMeasureReadFdcAutoscan4chMasked(sensorarrayFdcDeviceState_t *fdcState,
                                                            uint8_t freshMask4,
                                                            sensorarrayFdcAutoscanSamples_t *outSamples)
{
    if (!fdcState || !fdcState->handle || !outSamples) {
        return ESP_ERR_INVALID_ARG;
    }

    *outSamples = (sensorarrayFdcAutoscanSamples_t){0};
    freshMask4 &= 0x0Fu;
    outSamples->freshMask = freshMask4;
    Fdc2214CapFastChannelSample_t fastSamples[4] = {0};
    esp_err_t firstErr = Fdc2214CapReadChannelsDataRegsFast(fdcState->handle,
                                                            freshMask4,
                                                            fastSamples,
                                                            4u);
    for (uint8_t ch = 0u; ch < 4u; ++ch) {
        const Fdc2214CapFastChannelSample_t *sample = &fastSamples[ch];
        bool requestedFresh = (freshMask4 & (uint8_t)(1u << ch)) != 0u;
        outSamples->fresh[ch] = requestedFresh;
        outSamples->raw28[ch] = sample->raw28;
        outSamples->statusRaw = sample->statusRaw;
        if (requestedFresh || sample->unreadConversion) {
            outSamples->unreadMask |= (uint8_t)(1u << ch);
        }
        outSamples->amplitudeWarning[ch] =
            sample->errAmplitude ||
            ((sample->errorMask & FDC2214CAP_FAST_ERROR_AMPLITUDE) != 0u);
        bool currentReady = requestedFresh || sample->dataReady || sample->unreadConversion;
        outSamples->freshAmplitudeWarning[ch] = outSamples->amplitudeWarning[ch] &&
                                                currentReady &&
                                                firstErr == ESP_OK;
        outSamples->staleAmplitudeWarning[ch] = outSamples->amplitudeWarning[ch] &&
                                                !currentReady &&
                                                firstErr == ESP_OK;
        outSamples->transientAmplitudeWarning[ch] = outSamples->amplitudeWarning[ch] &&
                                                    firstErr != ESP_OK;
        outSamples->watchdogFault[ch] =
            sample->errWatchdog ||
            ((sample->errorMask & FDC2214CAP_FAST_ERROR_WATCHDOG) != 0u);
        outSamples->saturated[ch] = sample->raw28 >= SENSORARRAY_FDC_RAW28_SATURATED_THRESHOLD;
        outSamples->i2cError[ch] = (sample->errorMask & FDC2214CAP_FAST_ERROR_I2C) != 0u;

        bool i2cOk = !outSamples->i2cError[ch];
        bool readable = requestedFresh;
        outSamples->valid[ch] = i2cOk &&
                                readable &&
                                sample->raw28 != 0u &&
                                !outSamples->watchdogFault[ch] &&
                                !outSamples->saturated[ch];
        if (outSamples->valid[ch]) {
            outSamples->validMask |= (uint8_t)(1u << ch);
        }
        if (outSamples->amplitudeWarning[ch]) {
            outSamples->warnMask |= (uint8_t)(1u << ch);
        }
        if (!outSamples->valid[ch]) {
            outSamples->errorMask |= (uint8_t)(1u << ch);
        }
    }
    outSamples->partial = freshMask4 != 0u && freshMask4 != SENSORARRAY_FDC_AUTOSCAN_READY_UNREAD_MASK;
    outSamples->i2cTransactionError = firstErr != ESP_OK;
    return firstErr;
}

static esp_err_t sensorarrayMeasureReadFdcAutoscan4ch(sensorarrayFdcDeviceState_t *fdcState,
                                                      sensorarrayFdcAutoscanSamples_t *outSamples)
{
    return sensorarrayMeasureReadFdcAutoscan4chMasked(fdcState,
                                                     SENSORARRAY_FDC_AUTOSCAN_READY_UNREAD_MASK,
                                                     outSamples);
}

static void sensorarrayMeasureMarkFdcNoFreshSamples(sensorarrayFdcAutoscanSamples_t *outSamples,
                                                    const sensorarrayFdcReadyState_t *ready,
                                                    bool i2cError)
{
    if (!outSamples) {
        return;
    }

    *outSamples = (sensorarrayFdcAutoscanSamples_t){0};
    if (ready) {
        outSamples->statusRaw = ready->statusRaw;
        outSamples->unreadMask = ready->unreadMask & 0x0Fu;
        outSamples->timeout = ready->timeout;
        outSamples->partial = ready->partial;
    }
    outSamples->i2cTransactionError = i2cError;
    outSamples->errorMask = 0x0Fu;
    for (uint8_t ch = 0u; ch < 4u; ++ch) {
        outSamples->i2cError[ch] = i2cError;
    }
}

static void sensorarrayMeasureBuildFdcRead4Result(uint8_t row,
                                                  uint32_t epochId,
                                                  sensorarrayFdcDeviceId_t devId,
                                                  const sensorarrayFdcAutoscanSamples_t *samples,
                                                  const sensorarrayFdcRuntimeChannelConfig_t configs[4],
                                                  const sensorarrayFdcReadyState_t *ready,
                                                  esp_err_t readErr,
                                                  uint32_t readUs,
                                                  sensorarrayFdcDeviceRead4Result_t *outRead4)
{
    if (!outRead4) {
        return;
    }

    *outRead4 = (sensorarrayFdcDeviceRead4Result_t){
        .readErr = readErr,
        .i2cErr = (samples && samples->i2cTransactionError) ? readErr : ESP_OK,
        .readyKind = ready ? ready->kind : SENSORARRAY_FDC_READY_NONE,
        .status = ready ? ready->statusRaw : (samples ? samples->statusRaw : 0u),
        .errorStatus = ready ? ready->errorStatus : 0u,
        .unreadMask4 = ready ? (ready->unreadMask & 0x0Fu) : (samples ? samples->unreadMask : 0u),
        .drdy = ready ? ready->drdy : 0u,
        .timeout = ready ? ready->timeout : false,
        .partial = ready ? ready->partial : false,
        .i2cError = samples ? samples->i2cTransactionError : false,
        .waitUs = ready ? ready->waitUs : 0u,
        .readUs = readUs,
        .pollCount = ready ? ready->pollCount : 0u,
        .edgeDelta = ready ? ready->edgeDelta : 0u,
    };

    const double inductorUh = (double)CONFIG_SENSORARRAY_FDC_TANK_INDUCTOR_NH / 1000.0;
    for (uint8_t ch = 0u; ch < 4u; ++ch) {
        bool fresh = samples && samples->fresh[ch];
        bool configOk = configs && configs[ch].valid;
        uint32_t raw28 = samples ? samples->raw28[ch] : 0u;
        double freqHz = (fresh && configOk) ?
            sensorarrayMeasureFdcRaw28ToFreqHz(raw28, configs[ch].effectiveFclkHz, configs[ch].clockDividers) :
            0.0;
        bool valid = samples && samples->valid[ch] && configOk && freqHz > 0.0;
        bool warning = samples && samples->amplitudeWarning[ch];
        bool error = !valid;

        outRead4->raw28[ch] = raw28;
        outRead4->freqHz[ch] = freqHz;
        if (valid) {
            double capPf = 0.0;
            if (sensorarrayMeasureFdcComputeCapacitancePf(freqHz, inductorUh, &capPf)) {
                outRead4->capTotalPf[ch] = capPf;
            }
            outRead4->validMask4 |= (uint8_t)(1u << ch);
        }
        if (fresh) {
            outRead4->freshMask4 |= (uint8_t)(1u << ch);
        }
        if (warning) {
            outRead4->warnMask4 |= (uint8_t)(1u << ch);
        }
        if (error) {
            outRead4->errorMask4 |= (uint8_t)(1u << ch);
        }
    }

    if (outRead4->timeout && outRead4->freshMask4 != SENSORARRAY_FDC_AUTOSCAN_READY_UNREAD_MASK) {
        outRead4->partial = outRead4->freshMask4 != 0u;
        outRead4->readErr = ESP_ERR_TIMEOUT;
    }

    printf("FDC_DEVICE_READ4,dev=%s,row=%u,epoch=%lu,raw28=[%lu,%lu,%lu,%lu],freqHz=[%.3f,%.3f,%.3f,%.3f],freshMask=0x%X,validMask=0x%X,warnMask=0x%X,errorMask=0x%X,status=0x%04X,unread=0x%X,drdy=%u,partial=%u,timeout=%u,edgeDelta=%lu,waitUs=%lu,readUs=%lu,err=0x%lx\n",
           sensorarrayMeasureFdcDeviceName(devId),
           (unsigned)row,
           (unsigned long)epochId,
           (unsigned long)outRead4->raw28[0],
           (unsigned long)outRead4->raw28[1],
           (unsigned long)outRead4->raw28[2],
           (unsigned long)outRead4->raw28[3],
           outRead4->freqHz[0],
           outRead4->freqHz[1],
           outRead4->freqHz[2],
           outRead4->freqHz[3],
           (unsigned)outRead4->freshMask4,
           (unsigned)outRead4->validMask4,
           (unsigned)outRead4->warnMask4,
           (unsigned)outRead4->errorMask4,
           outRead4->status,
           (unsigned)outRead4->unreadMask4,
           (unsigned)outRead4->drdy,
           outRead4->partial ? 1u : 0u,
           outRead4->timeout ? 1u : 0u,
           (unsigned long)outRead4->edgeDelta,
           (unsigned long)outRead4->waitUs,
           (unsigned long)outRead4->readUs,
           (unsigned long)outRead4->readErr);
}

static sensorarrayFdcWorkerContext_t *sensorarrayMeasureFdcWorkerContext(sensorarrayFdcDeviceId_t devId)
{
    return (devId <= SENSORARRAY_FDC_DEV_SECONDARY) ? &s_fdcWorkers[(uint8_t)devId] : NULL;
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

    err = gpio_isr_handler_add((gpio_num_t)ctx->intbGpio, sensorarrayMeasureFdcIntbIsr, ctx);
    bool reattached = false;
    if (err == ESP_ERR_INVALID_STATE) {
        (void)gpio_isr_handler_remove((gpio_num_t)ctx->intbGpio);
        err = gpio_isr_handler_add((gpio_num_t)ctx->intbGpio, sensorarrayMeasureFdcIntbIsr, ctx);
        reattached = true;
    }
    if (err == ESP_OK && !s_fdcGpioIsrServiceInstalled) {
        s_fdcGpioIsrServiceInstalled = true;
        printf("FDC_INTB_ISR_SERVICE,stage=install,status=already_installed,err=0x0\n");
    }
    if (err == ESP_ERR_INVALID_STATE) {
        err = sensorarrayFdcEnsureGpioIsrServiceInstalled();
        if (err != ESP_OK) {
            return err;
        }
        err = gpio_isr_handler_add((gpio_num_t)ctx->intbGpio, sensorarrayMeasureFdcIntbIsr, ctx);
        reattached = false;
        if (err == ESP_ERR_INVALID_STATE) {
            (void)gpio_isr_handler_remove((gpio_num_t)ctx->intbGpio);
            err = gpio_isr_handler_add((gpio_num_t)ctx->intbGpio, sensorarrayMeasureFdcIntbIsr, ctx);
            reattached = true;
        }
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

static esp_err_t sensorarrayMeasureFdcSetSleepMode(sensorarrayFdcDeviceState_t *fdcState,
                                                   bool enable,
                                                   sensorarrayFdcDeviceTiming_t *timing)
{
    if (!fdcState || !fdcState->handle) {
        return ESP_ERR_INVALID_ARG;
    }

    uint16_t baseConfig = sensorarrayMeasureFdcConfigBaseWithoutSleep(fdcState);
    uint16_t config = enable ?
        (uint16_t)(baseConfig | SENSORARRAY_FDC_CONFIG_SLEEP_MODE_EN_MASK) :
        (uint16_t)(baseConfig & (uint16_t)~SENSORARRAY_FDC_CONFIG_SLEEP_MODE_EN_MASK);
    int64_t startUs = esp_timer_get_time();
    esp_err_t err = Fdc2214CapWriteRawRegisters(fdcState->handle, SENSORARRAY_FDC_REG_CONFIG, config);
    uint64_t elapsedUs = sensorarrayMeasureElapsedUs(startUs);
    if (timing) {
        if (enable) {
            timing->sleepEnterUs += elapsedUs;
        } else {
            timing->sleepExitUs += elapsedUs;
        }
        timing->diffConfigWrites++;
    }
    if (err == ESP_OK) {
        fdcState->configReg = config;
        fdcState->configVerified = true;
    } else {
        fdcState->configVerified = false;
    }
    return err;
}

static const char *sensorarrayMeasureFdcReadyKindName(sensorarrayFdcReadyKind_t kind)
{
    switch (kind) {
    case SENSORARRAY_FDC_READY_EDGE_WAKE:
        return "edge_wake";
    case SENSORARRAY_FDC_READY_POLL_FULL:
        return "poll_full";
    case SENSORARRAY_FDC_READY_POLL_PARTIAL:
        return "poll_partial";
    case SENSORARRAY_FDC_READY_TIMEOUT_PARTIAL:
        return "timeout_partial";
    case SENSORARRAY_FDC_READY_TIMEOUT_NONE:
        return "timeout_none";
    case SENSORARRAY_FDC_READY_I2C_ERROR:
        return "i2c_error";
    case SENSORARRAY_FDC_READY_NONE:
    default:
        return "none";
    }
}

static void sensorarrayMeasureFdcUpdateReadyTiming(const sensorarrayFdcReadyState_t *ready,
                                                   sensorarrayFdcDeviceTiming_t *timing,
                                                   bool fallbackAttempted)
{
    if (!ready || !timing) {
        return;
    }

    timing->readyPollCount += ready->pollCount;
    if (ready->kind == SENSORARRAY_FDC_READY_EDGE_WAKE ||
        ready->kind == SENSORARRAY_FDC_READY_POLL_FULL) {
        timing->readyFullCount++;
        timing->intbFreshDrdyCount++;
    } else if (ready->kind == SENSORARRAY_FDC_READY_POLL_PARTIAL ||
               ready->kind == SENSORARRAY_FDC_READY_TIMEOUT_PARTIAL) {
        timing->readyPartialCount++;
    } else if (ready->kind == SENSORARRAY_FDC_READY_TIMEOUT_NONE ||
               ready->kind == SENSORARRAY_FDC_READY_I2C_ERROR) {
        timing->readyNoneCount++;
    }
    if (ready->timeout) {
        timing->intbTimeoutCount++;
    }
    if (fallbackAttempted) {
        timing->fallbackAttemptCount++;
        if (ready->unreadMask != 0u) {
            timing->fallbackSuccessCount++;
            if (ready->unreadMask != SENSORARRAY_FDC_AUTOSCAN_READY_UNREAD_MASK) {
                timing->fallbackPartialCount++;
            }
        } else {
            timing->fallbackFailCount++;
        }
    }
    if (ready->waitUs > timing->maxWaitReadyUs) {
        timing->maxWaitReadyUs = ready->waitUs;
    }
}

static esp_err_t sensorarrayFdcWaitDeviceReady(sensorarrayState_t *state,
                                               sensorarrayFdcDeviceId_t devId,
                                               uint8_t row,
                                               uint32_t epochId,
                                               uint32_t timeoutUs,
                                               sensorarrayFdcReadyState_t *ready,
                                               sensorarrayFdcDeviceTiming_t *timing)
{
    if (!state || !ready || devId > SENSORARRAY_FDC_DEV_SECONDARY) {
        return ESP_ERR_INVALID_ARG;
    }
    sensorarrayFdcDeviceState_t *fdcState = sensorarrayMeasureGetFdcState(state, devId);
    if (!fdcState || !fdcState->handle) {
        return ESP_ERR_INVALID_STATE;
    }
    if (timeoutUs == 0u) {
        timeoutUs = 1u;
    }

    *ready = (sensorarrayFdcReadyState_t){
        .kind = SENSORARRAY_FDC_READY_NONE,
        .err = ESP_ERR_TIMEOUT,
        .initialIntbLevel = -1,
        .finalIntbLevel = -1,
    };
    sensorarrayFdcWorkerContext_t *ctx = sensorarrayMeasureFdcWorkerContext(devId);
    uint32_t edgeStart = ctx ? ctx->edgeCount : 0u;
    bool fallbackAttempted = false;
    bool edgeLogged = false;
    bool bestPartialSeen = false;
    uint16_t bestPartialStatus = 0u;
    uint8_t bestPartialUnread = 0u;
    uint8_t bestPartialDrdy = 0u;
    int64_t startUs = esp_timer_get_time();
    int64_t deadlineUs = startUs + (int64_t)timeoutUs;
    if (ctx && ctx->intbReady) {
        /*
         * INTB is a wake hint only. It indicates that the FDC may have updated
         * status/data, but it does not prove that all autoscan channels are
         * fresh, complete, or belong to the current row epoch. The worker must
         * validate data by STATUS/unread mask/raw28/error bits before accepting it.
         */
        ctx->waitTask = xTaskGetCurrentTaskHandle();
        ctx->currentEpoch = epochId;
        ready->initialIntbLevel = gpio_get_level((gpio_num_t)ctx->intbGpio);
        while (ulTaskNotifyTake(pdTRUE, 0) > 0u) {
        }
    }
    printf("FDC_READY,row=%u,epoch=%lu,device=%s,stage=begin,timeoutUs=%lu,startLevel=%d,startEdge=%lu\n",
           (unsigned)row,
           (unsigned long)epochId,
           sensorarrayMeasureFdcDeviceName(devId),
           (unsigned long)timeoutUs,
           ready->initialIntbLevel,
           (unsigned long)edgeStart);

    while (esp_timer_get_time() <= deadlineUs) {
        bool sawIntbEdge = false;
        if (ctx && ctx->intbReady) {
            int64_t remainingUs = deadlineUs - esp_timer_get_time();
            if (remainingUs <= 0) {
                break;
            }
            TickType_t waitTicks = pdMS_TO_TICKS(1u);
            sawIntbEdge = ulTaskNotifyTake(pdTRUE, waitTicks) > 0u;
        }

        if (!sawIntbEdge && ctx && ctx->intbReady && !CONFIG_SENSORARRAY_FDC_INTB_FALLBACK_POLLING) {
            continue;
        }
        if (!sawIntbEdge && ctx && ctx->intbReady) {
            fallbackAttempted = true;
            ctx->fallbackPollCount++;
            if (timing) {
                timing->intbFallbackPollCount++;
            }
        }
        if (sawIntbEdge) {
            ready->hadEdge = 1u;
            ready->edgeDelta = ctx && ctx->edgeCount >= edgeStart ? (ctx->edgeCount - edgeStart) : 0u;
            if (!edgeLogged) {
                printf("FDC_READY,row=%u,epoch=%lu,device=%s,stage=edge_wake,edgeDelta=%lu,level=%d,waitUs=%llu\n",
                       (unsigned)row,
                       (unsigned long)epochId,
                       sensorarrayMeasureFdcDeviceName(devId),
                       (unsigned long)ready->edgeDelta,
                       ctx ? gpio_get_level((gpio_num_t)ctx->intbGpio) : -1,
                       (unsigned long long)sensorarrayMeasureElapsedUs(startUs));
                edgeLogged = true;
            }
        }

        Fdc2214CapStatus_t status = {0};
        int64_t statusStartUs = esp_timer_get_time();
        esp_err_t err = Fdc2214CapReadStatus(fdcState->handle, &status);
        if (timing) {
            timing->statusReadUs += sensorarrayMeasureElapsedUs(statusStartUs);
        }
        ready->pollCount++;
        ready->err = err;
        if (err != ESP_OK) {
            ready->kind = SENSORARRAY_FDC_READY_I2C_ERROR;
            ready->i2cError = true;
            ready->waitUs = (uint32_t)sensorarrayMeasureElapsedUs(startUs);
            sensorarrayMeasureFdcUpdateReadyTiming(ready, timing, fallbackAttempted);
            if (ctx) {
                ctx->waitTask = NULL;
            }
            printf("FDC_READY,row=%u,epoch=%lu,device=%s,stage=i2c_error,err=0x%lx\n",
                   (unsigned)row,
                   (unsigned long)epochId,
                   sensorarrayMeasureFdcDeviceName(devId),
                   (unsigned long)err);
            return err;
        }
        ready->statusRaw = status.Raw;
        ready->dataReady = status.DataReady;
        ready->drdy = status.DataReady ? 1u : 0u;
        ready->unreadMask = sensorarrayMeasureFdcUnreadMaskFromStatus(&status);
        if (ready->unreadMask != 0u) {
            bestPartialSeen = true;
            bestPartialStatus = status.Raw;
            bestPartialUnread = ready->unreadMask;
            bestPartialDrdy = ready->drdy;
        }
        if (ready->unreadMask == SENSORARRAY_FDC_AUTOSCAN_READY_UNREAD_MASK) {
            ready->ready = true;
            ready->kind = sawIntbEdge ? SENSORARRAY_FDC_READY_EDGE_WAKE : SENSORARRAY_FDC_READY_POLL_FULL;
            ready->waitUs = (uint32_t)sensorarrayMeasureElapsedUs(startUs);
            if (ctx) {
                uint32_t staleEdges = edgeStart <= ctx->edgeCount ? (ctx->edgeCount - edgeStart) : 0u;
                if (staleEdges > 0u && !sawIntbEdge) {
                    ctx->staleBeforeClearCount += staleEdges;
                }
                ctx->freshDrdyCount++;
                ctx->waitTask = NULL;
            }
            sensorarrayMeasureFdcUpdateReadyTiming(ready, timing, fallbackAttempted);
#if CONFIG_SENSORARRAY_FDC_INTB_DEBUG_LOG
            printf("FDC_INTB_READY,row=%u,epoch=%lu,device=%s,status=0x%04X,unread=0x%X,drdy=%u,edge=%u\n",
                   (unsigned)row,
                   (unsigned long)epochId,
                   sensorarrayMeasureFdcDeviceName(devId),
                   ready->statusRaw,
                   (unsigned)ready->unreadMask,
                   ready->dataReady ? 1u : 0u,
                   sawIntbEdge ? 1u : 0u);
#endif
            printf("FDC_READY,row=%u,epoch=%lu,device=%s,stage=full,status=0x%04X,unread=0xF,waitUs=%lu,poll=%lu,kind=%s\n",
                   (unsigned)row,
                   (unsigned long)epochId,
                   sensorarrayMeasureFdcDeviceName(devId),
                   ready->statusRaw,
                   (unsigned long)ready->waitUs,
                   (unsigned long)ready->pollCount,
                   sensorarrayMeasureFdcReadyKindName(ready->kind));
            return ESP_OK;
        }
        if (sawIntbEdge && ctx) {
            ctx->falseEdgeCount++;
            if (timing) {
                timing->intbFalseEdgeCount++;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1u));
    }

    if (ctx) {
        ctx->timeoutCount++;
        ctx->waitTask = NULL;
        ready->finalIntbLevel = ctx->intbReady ? gpio_get_level((gpio_num_t)ctx->intbGpio) : -1;
    }
    ready->waitUs = (uint32_t)sensorarrayMeasureElapsedUs(startUs);
    ready->timeoutCount++;
    ready->timeout = true;
    if (bestPartialSeen) {
        ready->ready = true;
        ready->partial = true;
        ready->kind = SENSORARRAY_FDC_READY_TIMEOUT_PARTIAL;
        ready->statusRaw = bestPartialStatus;
        ready->unreadMask = bestPartialUnread;
        ready->drdy = bestPartialDrdy;
        ready->dataReady = bestPartialDrdy != 0u;
        ready->err = ESP_ERR_TIMEOUT;
        sensorarrayMeasureFdcUpdateReadyTiming(ready, timing, true);
        printf("FDC_READY,row=%u,epoch=%lu,device=%s,stage=partial_timeout,status=0x%04X,unread=0x%X,waitUs=%lu,poll=%lu\n",
               (unsigned)row,
               (unsigned long)epochId,
               sensorarrayMeasureFdcDeviceName(devId),
               ready->statusRaw,
               (unsigned)ready->unreadMask,
               (unsigned long)ready->waitUs,
               (unsigned long)ready->pollCount);
        printf("FDC_INTB_TIMEOUT,row=%u,epoch=%lu,device=%s,status=0x%04X,unread=0x%X,drdy=%u,timeoutUs=%lu,fallbackPolling=%u,kind=partial_timeout\n",
               (unsigned)row,
               (unsigned long)epochId,
               sensorarrayMeasureFdcDeviceName(devId),
               ready->statusRaw,
               (unsigned)ready->unreadMask,
               ready->dataReady ? 1u : 0u,
               (unsigned long)timeoutUs,
               CONFIG_SENSORARRAY_FDC_INTB_FALLBACK_POLLING ? 1u : 0u);
        return ESP_OK;
    }

    ready->kind = SENSORARRAY_FDC_READY_TIMEOUT_NONE;
    ready->err = ESP_ERR_TIMEOUT;
    sensorarrayMeasureFdcUpdateReadyTiming(ready, timing, fallbackAttempted || (ctx && ctx->intbReady));
    printf("FDC_READY,row=%u,epoch=%lu,device=%s,stage=timeout_none,status=0x%04X,unread=0x0,waitUs=%lu,poll=%lu\n",
           (unsigned)row,
           (unsigned long)epochId,
           sensorarrayMeasureFdcDeviceName(devId),
           ready->statusRaw,
           (unsigned long)ready->waitUs,
           (unsigned long)ready->pollCount);
    printf("FDC_INTB_TIMEOUT,row=%u,epoch=%lu,device=%s,status=0x%04X,unread=0x%X,drdy=%u,timeoutUs=%lu,fallbackPolling=%u,kind=timeout_none\n",
           (unsigned)row,
           (unsigned long)epochId,
           sensorarrayMeasureFdcDeviceName(devId),
           ready->statusRaw,
           (unsigned)ready->unreadMask,
           ready->dataReady ? 1u : 0u,
           (unsigned long)timeoutUs,
           CONFIG_SENSORARRAY_FDC_INTB_FALLBACK_POLLING ? 1u : 0u);
    return ESP_ERR_TIMEOUT;
}

static esp_err_t sensorarrayMeasureFdcRunDeviceEpochAfterSleep(sensorarrayState_t *state,
                                                               uint8_t row,
                                                               uint32_t epochId,
                                                               sensorarrayFdcDeviceId_t devId,
                                                               sensorarrayFdcAutoscanSamples_t *outSamples,
                                                               sensorarrayFdcRuntimeChannelConfig_t outConfigs[4],
                                                               sensorarrayFdcReadyState_t *outReady,
                                                               sensorarrayFdcDeviceRead4Result_t *outRead4,
                                                               sensorarrayFdcDeviceTiming_t *timing)
{
    if (!state || !outSamples || !outConfigs || !outReady || !outRead4 ||
        devId > SENSORARRAY_FDC_DEV_SECONDARY) {
        return ESP_ERR_INVALID_ARG;
    }
    sensorarrayFdcDeviceState_t *fdcState = sensorarrayMeasureGetFdcState(state, devId);
    if (!fdcState || !fdcState->ready || !fdcState->handle) {
        return ESP_ERR_INVALID_STATE;
    }
    if (timing) {
        timing->row = row;
        timing->deviceId = devId;
    }

    int64_t jobStartUs = esp_timer_get_time();
    int64_t applyStartUs = esp_timer_get_time();
    esp_err_t err = sensorarrayMeasureApplyFdcCachedRowConfig(state,
                                                              row,
                                                              devId,
                                                              "matrix_row_epoch_sleep",
                                                              false,
                                                              timing);
    uint64_t applyUs = sensorarrayMeasureElapsedUs(applyStartUs);
    if (timing) {
        timing->applyUs += 0u;
    }
    sensorarrayMeasureRuntimeConfigsFromApplied(state, devId, outConfigs);
    if (err != ESP_OK) {
        return err;
    }

    int64_t exitStartUs = esp_timer_get_time();
    err = sensorarrayMeasureFdcSetSleepMode(fdcState, false, timing);
    uint64_t exitUs = sensorarrayMeasureElapsedUs(exitStartUs);
    if (err != ESP_OK) {
        return err;
    }

    int64_t waitStartUs = esp_timer_get_time();
    err = sensorarrayFdcWaitDeviceReady(state,
                                        devId,
                                        row,
                                        epochId,
                                        (uint32_t)CONFIG_SENSORARRAY_FDC_INTB_WAIT_TIMEOUT_US,
                                        outReady,
                                        timing);
    uint64_t waitUs = sensorarrayMeasureElapsedUs(waitStartUs);
    if (timing) {
        timing->waitReadyUs += waitUs;
        timing->sleepExitToIntbUs += waitUs;
    }
    uint8_t freshMask = outReady->unreadMask & 0x0Fu;
    if (err != ESP_OK || freshMask == 0u) {
        sensorarrayMeasureMarkFdcNoFreshSamples(outSamples, outReady, outReady->i2cError);
        sensorarrayMeasureBuildFdcRead4Result(row,
                                              epochId,
                                              devId,
                                              outSamples,
                                              outConfigs,
                                              outReady,
                                              (err == ESP_OK) ? ESP_ERR_TIMEOUT : err,
                                              0u,
                                              outRead4);
        if (timing) {
            timing->deviceUs = sensorarrayMeasureElapsedUs(jobStartUs);
            timing->deviceFullInvalidCount++;
        }
        printf("FDC_FALLBACK,stage=poll_failed,row=%u,device=%s,status=0x%04X,unread=0x%X,err=0x%lx\n",
               (unsigned)row,
               sensorarrayMeasureFdcDeviceName(devId),
               outReady->statusRaw,
               (unsigned)outReady->unreadMask,
               (unsigned long)((err == ESP_OK) ? ESP_ERR_TIMEOUT : err));
        return ESP_OK;
    }

    int64_t readStartUs = esp_timer_get_time();
    err = sensorarrayMeasureReadFdcAutoscan4chMasked(fdcState, freshMask, outSamples);
    uint64_t readUs = sensorarrayMeasureElapsedUs(readStartUs);
    if (timing) {
        timing->readRawUs += readUs;
        timing->dataReadUs += readUs;
        if (readUs > timing->maxI2cReadUs) {
            timing->maxI2cReadUs = readUs;
        }
        timing->deviceUs = sensorarrayMeasureElapsedUs(jobStartUs);
        (void)applyUs;
        (void)exitUs;
    }
    sensorarrayMeasureBuildFdcRead4Result(row,
                                          epochId,
                                          devId,
                                          outSamples,
                                          outConfigs,
                                          outReady,
                                          err,
                                          (uint32_t)readUs,
                                          outRead4);
    if (timing && outRead4->validMask4 == 0u) {
        timing->deviceFullInvalidCount++;
    }
    bool fallbackRelevant = outRead4->timeout ||
                            outRead4->partial ||
                            outRead4->readyKind == SENSORARRAY_FDC_READY_POLL_FULL ||
                            outRead4->readyKind == SENSORARRAY_FDC_READY_TIMEOUT_PARTIAL;
    if (fallbackRelevant && outRead4->validMask4 != 0u) {
        printf("FDC_FALLBACK,stage=%s,row=%u,device=%s,validMask=0x%X,unread=0x%X,status=0x%04X\n",
               outRead4->partial ? "poll_partial" : "poll_success",
               (unsigned)row,
               sensorarrayMeasureFdcDeviceName(devId),
               (unsigned)outRead4->validMask4,
               (unsigned)outRead4->unreadMask4,
               outRead4->status);
    } else if (outRead4->validMask4 == 0u) {
        printf("FDC_FALLBACK,stage=poll_failed,row=%u,device=%s,status=0x%04X,unread=0x%X,err=0x%lx\n",
               (unsigned)row,
               sensorarrayMeasureFdcDeviceName(devId),
               outRead4->status,
               (unsigned)outRead4->unreadMask4,
               (unsigned long)outRead4->readErr);
    }
    return ESP_OK;
}

static void sensorarrayMeasureFdcWorkerTask(void *arg)
{
    sensorarrayFdcWorkerContext_t *ctx = (sensorarrayFdcWorkerContext_t *)arg;
    if (!ctx) {
        vTaskDelete(NULL);
        return;
    }

    for (;;) {
        sensorarrayFdcWorkerJob_t job = {0};
        if (xQueueReceive(ctx->queue, &job, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        if (!job.state || !job.outSamples || !job.outConfigs || !job.result) {
            if (job.result) {
                job.result->err = ESP_ERR_INVALID_ARG;
            }
            xSemaphoreGive(ctx->done);
            continue;
        }

        *job.result = (sensorarrayFdcWorkerResult_t){
            .err = ESP_ERR_TIMEOUT,
            .row = job.row,
            .devId = ctx->devId,
            .epochId = job.epochId,
        };
        sensorarrayFdcDeviceState_t *fdcState = sensorarrayMeasureGetFdcState(job.state, ctx->devId);
        esp_err_t err = ESP_ERR_INVALID_STATE;
        if (fdcState && fdcState->ready && fdcState->handle) {
            err = sensorarrayMeasureFdcSetSleepMode(fdcState, true, job.timing);
        }
        job.result->err = err;
        xSemaphoreGive(ctx->sleepAck);

        TickType_t waitTicks = pdMS_TO_TICKS((uint32_t)CONFIG_SENSORARRAY_FDC_WORKER_SYNC_TIMEOUT_MS);
        if (waitTicks == 0) {
            waitTicks = 1;
        }
        if (err == ESP_OK && xSemaphoreTake(ctx->start, waitTicks) == pdTRUE) {
            err = sensorarrayMeasureFdcRunDeviceEpochAfterSleep(job.state,
                                                                job.row,
                                                                job.epochId,
                                                                ctx->devId,
                                                                job.outSamples,
                                                                job.outConfigs,
                                                                &job.result->ready,
                                                                &job.result->read4,
                                                                job.timing);
            job.result->err = err;
        } else if (err == ESP_OK) {
            job.result->err = ESP_ERR_TIMEOUT;
        }
        xSemaphoreGive(ctx->done);
    }
}

static esp_err_t sensorarrayMeasureEnsureFdcWorkers(void)
{
    if (s_fdcWorkersInitAttempted) {
        return s_fdcWorkersAvailable ? ESP_OK : ESP_ERR_NOT_SUPPORTED;
    }
    s_fdcWorkersInitAttempted = true;

    esp_err_t firstErr = ESP_OK;
    for (uint8_t i = 0u; i < 2u; ++i) {
        sensorarrayFdcWorkerContext_t *ctx = &s_fdcWorkers[i];
        ctx->queue = xQueueCreateStatic(SENSORARRAY_FDC_WORKER_QUEUE_DEPTH,
                                        sizeof(sensorarrayFdcWorkerJob_t),
                                        ctx->queueBuffer,
                                        &ctx->queueStorage);
        ctx->sleepAck = xSemaphoreCreateBinaryStatic(&ctx->sleepAckStorage);
        ctx->start = xSemaphoreCreateBinaryStatic(&ctx->startStorage);
        ctx->done = xSemaphoreCreateBinaryStatic(&ctx->doneStorage);
        if (!ctx->queue || !ctx->sleepAck || !ctx->start || !ctx->done) {
            firstErr = ESP_FAIL;
            printf("FDC_WORKER_CREATE_FAIL,dev=%s,err=0x%lx,status=fallback_serial,reason=resource_init_failed\n",
                   sensorarrayMeasureFdcDeviceName(ctx->devId),
                   (unsigned long)firstErr);
            break;
        }
        (void)sensorarrayMeasureEnsureFdcIntb(ctx);

        char taskName[24] = {0};
        snprintf(taskName, sizeof(taskName), "fdc_%s_worker",
                 ctx->devId == SENSORARRAY_FDC_DEV_SECONDARY ? "secondary" : "primary");
        BaseType_t resolvedCore = 0;
        bool pinned = true;
        const char *coreReason = "unknown";
        if (!sensorarrayMeasureResolveWorkerCore(taskName,
                                                 CONFIG_SENSORARRAY_FDC_WORKER_TASK_CORE,
                                                 1,
                                                 &resolvedCore,
                                                 &pinned,
                                                 &coreReason)) {
            firstErr = ESP_FAIL;
            printf("FDC_WORKER_CREATE_FAIL,dev=%s,err=0x%lx,status=fallback_serial,reason=core_resolve_failed\n",
                   sensorarrayMeasureFdcDeviceName(ctx->devId),
                   (unsigned long)firstErr);
            break;
        }
        if (strcmp(coreReason, "configured") != 0) {
            printf("FDC_WORKER_CORE_FIX,dev=%s,configuredCore=%d,resolvedCore=%d,pinned=%u,portNumProcessors=%d,reason=%s\n",
                   sensorarrayMeasureFdcDeviceName(ctx->devId),
                   CONFIG_SENSORARRAY_FDC_WORKER_TASK_CORE,
                   (int)resolvedCore,
                   pinned ? 1u : 0u,
                   (int)portNUM_PROCESSORS,
                   coreReason);
        }
        printf("FDC_WORKER_CREATE_BEGIN,dev=%s,configuredCore=%d,resolvedCore=%d,pinned=%u,stackWords=%lu,prio=%d,portNumProcessors=%d\n",
               sensorarrayMeasureFdcDeviceName(ctx->devId),
               CONFIG_SENSORARRAY_FDC_WORKER_TASK_CORE,
               (int)resolvedCore,
               pinned ? 1u : 0u,
               (unsigned long)SENSORARRAY_FDC_WORKER_STACK_WORDS,
               CONFIG_SENSORARRAY_FDC_WORKER_TASK_PRIO,
               (int)portNUM_PROCESSORS);
        if (pinned) {
            ctx->task = xTaskCreateStaticPinnedToCore(sensorarrayMeasureFdcWorkerTask,
                                                      taskName,
                                                      SENSORARRAY_FDC_WORKER_STACK_WORDS,
                                                      ctx,
                                                      CONFIG_SENSORARRAY_FDC_WORKER_TASK_PRIO,
                                                      ctx->stack,
                                                      &ctx->taskStorage,
                                                      resolvedCore);
        } else {
            ctx->task = xTaskCreateStatic(sensorarrayMeasureFdcWorkerTask,
                                          taskName,
                                          SENSORARRAY_FDC_WORKER_STACK_WORDS,
                                          ctx,
                                          CONFIG_SENSORARRAY_FDC_WORKER_TASK_PRIO,
                                          ctx->stack,
                                          &ctx->taskStorage);
        }
        if (!ctx->task) {
            firstErr = ESP_FAIL;
            printf("FDC_WORKER_CREATE_FAIL,dev=%s,err=0x%lx,status=fallback_serial,reason=task_create_failed\n",
                   sensorarrayMeasureFdcDeviceName(ctx->devId),
                   (unsigned long)firstErr);
            break;
        }
        printf("FDC_WORKER_CREATE_DONE,dev=%s,handle=%p,err=0x0,status=ok\n",
               sensorarrayMeasureFdcDeviceName(ctx->devId),
               (void *)ctx->task);
        ctx->initialized = true;
    }

    s_fdcWorkersAvailable = firstErr == ESP_OK;
    if (!s_fdcWorkersAvailable) {
        sensorarrayMeasureCleanupFdcWorkers();
        printf("FDC_WORKER,stage=init,status=fallback_serial,err=0x%lx\n",
               (unsigned long)firstErr);
        return firstErr;
    }
    printf("FDC_WORKER,stage=init,status=parallel_ready,primaryBus=0,secondaryBus=1,intb=%u\n",
           CONFIG_SENSORARRAY_FDC_INTB_ENABLE ? 1u : 0u);
    return ESP_OK;
}

static void sensorarrayMeasureAccumulateRowEpochTiming(sensorarrayFdcTimingSummary_t *summary,
                                                       const sensorarrayFdcRowTiming_t *rowTiming,
                                                       const sensorarrayFdcDeviceTiming_t *primaryTiming,
                                                       const sensorarrayFdcDeviceTiming_t *secondaryTiming)
{
    if (!summary || !rowTiming || !primaryTiming || !secondaryTiming) {
        return;
    }

    summary->sleepBeforeRowSwitchUs += rowTiming->sleepBeforeRowSwitchUs;
    summary->rowSwitchWhileSleepingUs += rowTiming->rowSwitchWhileSleepingUs;
    summary->rowSettleUs += rowTiming->rowSettleUs;
    summary->diffApplyWhileSleepingUs += primaryTiming->applyUs + secondaryTiming->applyUs;
    summary->sleepTotalUs += primaryTiming->sleepEnterUs + secondaryTiming->sleepEnterUs +
                             primaryTiming->sleepExitUs + secondaryTiming->sleepExitUs;
    summary->sleepExitToIntbUs += primaryTiming->sleepExitToIntbUs + secondaryTiming->sleepExitToIntbUs;
    summary->statusReadUs += primaryTiming->statusReadUs + secondaryTiming->statusReadUs;
    summary->dataReadUs += primaryTiming->dataReadUs + secondaryTiming->dataReadUs;
    summary->primaryJobUs += primaryTiming->deviceUs;
    summary->secondaryJobUs += secondaryTiming->deviceUs;
    summary->dualBusWaitUs += rowTiming->dualBusWaitUs;
    summary->dualBusSkewUs += rowTiming->dualBusSkewUs;

    summary->cacheApplyDiffWriteCount += primaryTiming->cacheDiffWriteCount +
                                         secondaryTiming->cacheDiffWriteCount;
    summary->cacheApplyFullWriteCount += primaryTiming->cacheFullWriteCount +
                                         secondaryTiming->cacheFullWriteCount;
    summary->cacheApplyNoDiffCount += primaryTiming->cacheNoDiffCount +
                                      secondaryTiming->cacheNoDiffCount;
    summary->diffRcountWrites += primaryTiming->diffRcountWrites + secondaryTiming->diffRcountWrites;
    summary->diffSettleWrites += primaryTiming->diffSettleWrites + secondaryTiming->diffSettleWrites;
    summary->diffClockDivWrites += primaryTiming->diffClockDivWrites + secondaryTiming->diffClockDivWrites;
    summary->diffDriveWrites += primaryTiming->diffDriveWrites + secondaryTiming->diffDriveWrites;
    summary->diffMuxWrites += primaryTiming->diffMuxWrites + secondaryTiming->diffMuxWrites;
    summary->diffStatusConfigWrites += primaryTiming->diffStatusConfigWrites +
                                       secondaryTiming->diffStatusConfigWrites;
    summary->diffConfigWrites += primaryTiming->diffConfigWrites + secondaryTiming->diffConfigWrites;
    summary->appliedFingerprintChanges += primaryTiming->appliedFingerprintChanged +
                                          secondaryTiming->appliedFingerprintChanged;

    summary->intbEdgeCountPrimary += primaryTiming->intbEdgeCount;
    summary->intbEdgeCountSecondary += secondaryTiming->intbEdgeCount;
    summary->intbFalseEdgeCount += primaryTiming->intbFalseEdgeCount +
                                   secondaryTiming->intbFalseEdgeCount;
    summary->intbTimeoutCount += primaryTiming->intbTimeoutCount + secondaryTiming->intbTimeoutCount;
    summary->intbFallbackPollCount += primaryTiming->intbFallbackPollCount +
                                      secondaryTiming->intbFallbackPollCount;
    summary->intbFreshDrdyCount += primaryTiming->intbFreshDrdyCount +
                                   secondaryTiming->intbFreshDrdyCount;
    summary->readyFullCount += primaryTiming->readyFullCount + secondaryTiming->readyFullCount;
    summary->readyPartialCount += primaryTiming->readyPartialCount + secondaryTiming->readyPartialCount;
    summary->readyNoneCount += primaryTiming->readyNoneCount + secondaryTiming->readyNoneCount;
    summary->fallbackAttemptCount += primaryTiming->fallbackAttemptCount + secondaryTiming->fallbackAttemptCount;
    summary->fallbackSuccessCount += primaryTiming->fallbackSuccessCount + secondaryTiming->fallbackSuccessCount;
    summary->fallbackPartialCount += primaryTiming->fallbackPartialCount + secondaryTiming->fallbackPartialCount;
    summary->fallbackFailCount += primaryTiming->fallbackFailCount + secondaryTiming->fallbackFailCount;
    summary->deviceFullInvalidCount += primaryTiming->deviceFullInvalidCount +
                                       secondaryTiming->deviceFullInvalidCount;
    summary->waitReadyUsPrimaryTotal += primaryTiming->waitReadyUs;
    summary->waitReadyUsSecondaryTotal += secondaryTiming->waitReadyUs;
    summary->read4UsPrimaryTotal += primaryTiming->readRawUs;
    summary->read4UsSecondaryTotal += secondaryTiming->readRawUs;
    if (primaryTiming->maxWaitReadyUs > summary->maxWaitReadyUs) {
        summary->maxWaitReadyUs = primaryTiming->maxWaitReadyUs;
    }
    if (secondaryTiming->maxWaitReadyUs > summary->maxWaitReadyUs) {
        summary->maxWaitReadyUs = secondaryTiming->maxWaitReadyUs;
    }
    if (primaryTiming->maxI2cReadUs > summary->maxI2cReadUs) {
        summary->maxI2cReadUs = primaryTiming->maxI2cReadUs;
    }
    if (secondaryTiming->maxI2cReadUs > summary->maxI2cReadUs) {
        summary->maxI2cReadUs = secondaryTiming->maxI2cReadUs;
    }
}

static esp_err_t sensorarrayMeasureSelectFdcRowWhileSleeping(sensorarrayState_t *state,
                                                             uint8_t row,
                                                             sensorarrayFdcRowTiming_t *rowTiming)
{
    (void)state;
    int64_t rowSelectStartUs = esp_timer_get_time();
    esp_err_t err = tmuxSwitchSelectRow((uint8_t)(row - 1u));
    uint64_t rowSelectUs = sensorarrayMeasureElapsedUs(rowSelectStartUs);
    if (rowTiming) {
        rowTiming->rowSelectUs = rowSelectUs;
        rowTiming->rowSwitchWhileSleepingUs += rowSelectUs;
    }

    int64_t settleStartUs = esp_timer_get_time();
    sensorarrayMeasureDelayUs((uint32_t)CONFIG_SENSORARRAY_FDC_ROW_SWITCH_SETTLE_US);
    uint64_t settleUs = sensorarrayMeasureElapsedUs(settleStartUs);
    if (rowTiming) {
        rowTiming->analogSettleUs = settleUs;
        rowTiming->rowSettleUs += settleUs;
    }
#if CONFIG_SENSORARRAY_FDC_VERBOSE_SCAN_LOG
    tmuxSwitchControlState_t ctrl = {0};
    sensorarrayMeasureReadFdcPathControl(&ctrl);
    printf("FDC_ROW,stage=select_while_sleep,row=%u,selaCmd=%d,selaReadback=%d,err=0x%lx\n",
           (unsigned)row,
           ctrl.cmdSelaLevel,
           ctrl.obsSelaLevel,
           (unsigned long)err);
#endif
    return err;
}

static esp_err_t sensorarrayMeasureReadFdcMatrixRowSerialEpoch(sensorarrayState_t *state,
                                                               uint8_t row,
                                                               uint32_t epochId,
                                                               sensorarrayFdcAutoscanSamples_t *primarySamples,
                                                               sensorarrayFdcAutoscanSamples_t *secondarySamples,
                                                               sensorarrayFdcRuntimeChannelConfig_t runtimeConfigs[2][4],
                                                               sensorarrayFdcDeviceTiming_t *primaryTiming,
                                                               sensorarrayFdcDeviceTiming_t *secondaryTiming,
                                                               sensorarrayFdcRowTiming_t *rowTiming)
{
    if (!state || !primarySamples || !secondarySamples || !runtimeConfigs ||
        !primaryTiming || !secondaryTiming || !rowTiming) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t firstErr = ESP_OK;
    int64_t sleepStartUs = esp_timer_get_time();
    esp_err_t primaryErr = sensorarrayMeasureFdcSetSleepMode(&state->fdcPrimary, true, primaryTiming);
    esp_err_t secondaryErr = sensorarrayMeasureFdcSetSleepMode(&state->fdcSecondary, true, secondaryTiming);
    rowTiming->sleepBeforeRowSwitchUs = sensorarrayMeasureElapsedUs(sleepStartUs);
    if (primaryErr != ESP_OK && firstErr == ESP_OK) {
        firstErr = primaryErr;
    }
    if (secondaryErr != ESP_OK && firstErr == ESP_OK) {
        firstErr = secondaryErr;
    }

    sensorarrayMeasureFdcPrepareIntbEpoch(sensorarrayMeasureFdcWorkerContext(SENSORARRAY_FDC_DEV_PRIMARY),
                                          epochId);
    sensorarrayMeasureFdcPrepareIntbEpoch(sensorarrayMeasureFdcWorkerContext(SENSORARRAY_FDC_DEV_SECONDARY),
                                          epochId);

    esp_err_t rowErr = sensorarrayMeasureSelectFdcRowWhileSleeping(state, row, rowTiming);
    if (rowErr != ESP_OK && firstErr == ESP_OK) {
        firstErr = rowErr;
    }

    sensorarrayFdcReadyState_t primaryReady = {0};
    sensorarrayFdcReadyState_t secondaryReady = {0};
    sensorarrayFdcDeviceRead4Result_t primaryRead4 = {0};
    sensorarrayFdcDeviceRead4Result_t secondaryRead4 = {0};
    if (primaryErr == ESP_OK) {
        primaryErr = sensorarrayMeasureFdcRunDeviceEpochAfterSleep(state,
                                                                   row,
                                                                   epochId,
                                                                   SENSORARRAY_FDC_DEV_PRIMARY,
                                                                   primarySamples,
                                                                   runtimeConfigs[SENSORARRAY_FDC_DEV_PRIMARY],
                                                                   &primaryReady,
                                                                   &primaryRead4,
                                                                   primaryTiming);
        if (primaryErr != ESP_OK && firstErr == ESP_OK) {
            firstErr = primaryErr;
        }
    }
    if (secondaryErr == ESP_OK) {
        secondaryErr = sensorarrayMeasureFdcRunDeviceEpochAfterSleep(state,
                                                                     row,
                                                                     epochId,
                                                                     SENSORARRAY_FDC_DEV_SECONDARY,
                                                                     secondarySamples,
                                                                     runtimeConfigs[SENSORARRAY_FDC_DEV_SECONDARY],
                                                                     &secondaryReady,
                                                                     &secondaryRead4,
                                                                     secondaryTiming);
        if (secondaryErr != ESP_OK && firstErr == ESP_OK) {
            firstErr = secondaryErr;
        }
    }

    rowTiming->waitReadyUs = primaryTiming->waitReadyUs + secondaryTiming->waitReadyUs;
    rowTiming->readUs = primaryTiming->readRawUs + secondaryTiming->readRawUs;
    rowTiming->primaryTotalUs = primaryTiming->deviceUs;
    rowTiming->secondaryTotalUs = secondaryTiming->deviceUs;
    rowTiming->dualBusSkewUs =
        (primaryTiming->deviceUs > secondaryTiming->deviceUs) ?
        (primaryTiming->deviceUs - secondaryTiming->deviceUs) :
        (secondaryTiming->deviceUs - primaryTiming->deviceUs);
    (void)primaryReady;
    (void)secondaryReady;
    return firstErr;
}

static esp_err_t sensorarrayMeasureReadFdcMatrixRowParallelEpoch(sensorarrayState_t *state,
                                                                 uint8_t row,
                                                                 uint32_t epochId,
                                                                 sensorarrayFdcAutoscanSamples_t *primarySamples,
                                                                 sensorarrayFdcAutoscanSamples_t *secondarySamples,
                                                                 sensorarrayFdcRuntimeChannelConfig_t runtimeConfigs[2][4],
                                                                 sensorarrayFdcDeviceTiming_t *primaryTiming,
                                                                 sensorarrayFdcDeviceTiming_t *secondaryTiming,
                                                                 sensorarrayFdcRowTiming_t *rowTiming)
{
    if (!state || !primarySamples || !secondarySamples || !runtimeConfigs ||
        !primaryTiming || !secondaryTiming || !rowTiming) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t err = sensorarrayMeasureEnsureFdcWorkers();
    if (err != ESP_OK) {
        return err;
    }

    sensorarrayFdcWorkerContext_t *primaryCtx = sensorarrayMeasureFdcWorkerContext(SENSORARRAY_FDC_DEV_PRIMARY);
    sensorarrayFdcWorkerContext_t *secondaryCtx = sensorarrayMeasureFdcWorkerContext(SENSORARRAY_FDC_DEV_SECONDARY);
    if (!primaryCtx || !secondaryCtx || !primaryCtx->initialized || !secondaryCtx->initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    sensorarrayMeasureDrainSemaphore(primaryCtx->sleepAck);
    sensorarrayMeasureDrainSemaphore(primaryCtx->start);
    sensorarrayMeasureDrainSemaphore(primaryCtx->done);
    sensorarrayMeasureDrainSemaphore(secondaryCtx->sleepAck);
    sensorarrayMeasureDrainSemaphore(secondaryCtx->start);
    sensorarrayMeasureDrainSemaphore(secondaryCtx->done);

    sensorarrayFdcWorkerResult_t primaryResult = {0};
    sensorarrayFdcWorkerResult_t secondaryResult = {0};
    sensorarrayFdcWorkerJob_t primaryJob = {
        .state = state,
        .row = row,
        .epochId = epochId,
        .outSamples = primarySamples,
        .outConfigs = runtimeConfigs[SENSORARRAY_FDC_DEV_PRIMARY],
        .timing = primaryTiming,
        .result = &primaryResult,
    };
    sensorarrayFdcWorkerJob_t secondaryJob = {
        .state = state,
        .row = row,
        .epochId = epochId,
        .outSamples = secondarySamples,
        .outConfigs = runtimeConfigs[SENSORARRAY_FDC_DEV_SECONDARY],
        .timing = secondaryTiming,
        .result = &secondaryResult,
    };

    TickType_t syncTicks = pdMS_TO_TICKS((uint32_t)CONFIG_SENSORARRAY_FDC_WORKER_SYNC_TIMEOUT_MS);
    if (syncTicks == 0) {
        syncTicks = 1;
    }
    int64_t sleepStartUs = esp_timer_get_time();
    BaseType_t primaryQueued = xQueueSend(primaryCtx->queue, &primaryJob, syncTicks);
    BaseType_t secondaryQueued = xQueueSend(secondaryCtx->queue, &secondaryJob, syncTicks);
    if (primaryQueued != pdTRUE || secondaryQueued != pdTRUE) {
        if (primaryQueued == pdTRUE) {
            (void)xSemaphoreTake(primaryCtx->sleepAck, syncTicks);
            xSemaphoreGive(primaryCtx->start);
            (void)xSemaphoreTake(primaryCtx->done, syncTicks);
        }
        if (secondaryQueued == pdTRUE) {
            (void)xSemaphoreTake(secondaryCtx->sleepAck, syncTicks);
            xSemaphoreGive(secondaryCtx->start);
            (void)xSemaphoreTake(secondaryCtx->done, syncTicks);
        }
        printf("FDC_PARALLEL_FALLBACK,reason=queue_failed,row=%u,epoch=%lu,primaryQueued=%u,secondaryQueued=%u,err=0x%lx\n",
               (unsigned)row,
               (unsigned long)epochId,
               primaryQueued == pdTRUE ? 1u : 0u,
               secondaryQueued == pdTRUE ? 1u : 0u,
               (unsigned long)ESP_ERR_TIMEOUT);
        return ESP_ERR_TIMEOUT;
    }

    BaseType_t primarySleep = xSemaphoreTake(primaryCtx->sleepAck, syncTicks);
    BaseType_t secondarySleep = xSemaphoreTake(secondaryCtx->sleepAck, syncTicks);
    rowTiming->sleepBeforeRowSwitchUs = sensorarrayMeasureElapsedUs(sleepStartUs);
    esp_err_t firstErr = ESP_OK;
    if (primarySleep != pdTRUE) {
        primaryResult.err = ESP_ERR_TIMEOUT;
        firstErr = ESP_ERR_TIMEOUT;
    } else if (primaryResult.err != ESP_OK) {
        firstErr = primaryResult.err;
    }
    if (secondarySleep != pdTRUE) {
        secondaryResult.err = ESP_ERR_TIMEOUT;
        if (firstErr == ESP_OK) {
            firstErr = ESP_ERR_TIMEOUT;
        }
    } else if (secondaryResult.err != ESP_OK && firstErr == ESP_OK) {
        firstErr = secondaryResult.err;
    }

    sensorarrayMeasureFdcPrepareIntbEpoch(primaryCtx, epochId);
    sensorarrayMeasureFdcPrepareIntbEpoch(secondaryCtx, epochId);

    esp_err_t rowErr = sensorarrayMeasureSelectFdcRowWhileSleeping(state, row, rowTiming);
    if (rowErr != ESP_OK && firstErr == ESP_OK) {
        firstErr = rowErr;
    }

    int64_t dualStartUs = esp_timer_get_time();
    xSemaphoreGive(primaryCtx->start);
    xSemaphoreGive(secondaryCtx->start);
    TickType_t doneTicks =
        pdMS_TO_TICKS((uint32_t)((CONFIG_SENSORARRAY_FDC_INTB_WAIT_TIMEOUT_US / 1000u) + 250u));
    if (doneTicks == 0) {
        doneTicks = pdMS_TO_TICKS(250u);
    }
    BaseType_t primaryDone = xSemaphoreTake(primaryCtx->done, doneTicks);
    BaseType_t secondaryDone = xSemaphoreTake(secondaryCtx->done, doneTicks);
    rowTiming->dualBusWaitUs = sensorarrayMeasureElapsedUs(dualStartUs);
    if (primaryDone != pdTRUE) {
        primaryResult.err = ESP_ERR_TIMEOUT;
        primaryTiming->timeoutCount++;
        printf("FDC_WORKER,row=%u,epoch=%lu,device=primary,status=timeout\n",
               (unsigned)row,
               (unsigned long)epochId);
        if (firstErr == ESP_OK) {
            firstErr = ESP_ERR_TIMEOUT;
        }
    }
    if (secondaryDone != pdTRUE) {
        secondaryResult.err = ESP_ERR_TIMEOUT;
        secondaryTiming->timeoutCount++;
        printf("FDC_WORKER,row=%u,epoch=%lu,device=secondary,status=timeout\n",
               (unsigned)row,
               (unsigned long)epochId);
        if (firstErr == ESP_OK) {
            firstErr = ESP_ERR_TIMEOUT;
        }
    }
    if (primaryDone == pdTRUE &&
        (primaryResult.epochId != epochId ||
         primaryResult.row != row ||
         primaryResult.devId != SENSORARRAY_FDC_DEV_PRIMARY)) {
        printf("FDC_WORKER_RESULT,stage=stale_reject,row=%u,epoch=%lu,device=primary,resultRow=%u,resultEpoch=%lu,resultDev=%s\n",
               (unsigned)row,
               (unsigned long)epochId,
               (unsigned)primaryResult.row,
               (unsigned long)primaryResult.epochId,
               sensorarrayMeasureFdcDeviceName(primaryResult.devId));
        sensorarrayMeasureMarkFdcNoFreshSamples(primarySamples, &primaryResult.ready, false);
        primaryResult.read4.staleRejected = true;
        primaryResult.read4.validMask4 = 0u;
        primaryResult.read4.errorMask4 = 0x0Fu;
    }
    if (secondaryDone == pdTRUE &&
        (secondaryResult.epochId != epochId ||
         secondaryResult.row != row ||
         secondaryResult.devId != SENSORARRAY_FDC_DEV_SECONDARY)) {
        printf("FDC_WORKER_RESULT,stage=stale_reject,row=%u,epoch=%lu,device=secondary,resultRow=%u,resultEpoch=%lu,resultDev=%s\n",
               (unsigned)row,
               (unsigned long)epochId,
               (unsigned)secondaryResult.row,
               (unsigned long)secondaryResult.epochId,
               sensorarrayMeasureFdcDeviceName(secondaryResult.devId));
        sensorarrayMeasureMarkFdcNoFreshSamples(secondarySamples, &secondaryResult.ready, false);
        secondaryResult.read4.staleRejected = true;
        secondaryResult.read4.validMask4 = 0u;
        secondaryResult.read4.errorMask4 = 0x0Fu;
    }
    if (primaryResult.err != ESP_OK && firstErr == ESP_OK) {
        firstErr = primaryResult.err;
    }
    if (secondaryResult.err != ESP_OK && firstErr == ESP_OK) {
        firstErr = secondaryResult.err;
    }

    rowTiming->waitReadyUs = primaryTiming->waitReadyUs + secondaryTiming->waitReadyUs;
    rowTiming->readUs = primaryTiming->readRawUs + secondaryTiming->readRawUs;
    rowTiming->primaryTotalUs = primaryTiming->deviceUs;
    rowTiming->secondaryTotalUs = secondaryTiming->deviceUs;
    rowTiming->primaryJobUs = primaryTiming->deviceUs;
    rowTiming->secondaryJobUs = secondaryTiming->deviceUs;
    rowTiming->dualBusSkewUs =
        (primaryTiming->deviceUs > secondaryTiming->deviceUs) ?
        (primaryTiming->deviceUs - secondaryTiming->deviceUs) :
        (secondaryTiming->deviceUs - primaryTiming->deviceUs);
    return firstErr;
}

static esp_err_t __attribute__((unused)) sensorarrayMeasureReadFdcRuntimeChannelConfigs(sensorarrayState_t *state,
                                                                                        sensorarrayFdcRuntimeChannelConfig_t configs[2][4])
{
    if (!state || !configs) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(configs, 0, sizeof(sensorarrayFdcRuntimeChannelConfig_t) * 2u * 4u);
    esp_err_t firstErr = ESP_OK;
    for (uint8_t dev = 0u; dev < 2u; ++dev) {
        sensorarrayFdcDeviceState_t *fdcState =
            sensorarrayMeasureGetFdcState(state, (sensorarrayFdcDeviceId_t)dev);
        if (!fdcState || !fdcState->ready || !fdcState->handle) {
            if (firstErr == ESP_OK) {
                firstErr = ESP_ERR_INVALID_STATE;
            }
            continue;
        }

        Fdc2214CapCoreRegs_t coreRegs = {0};
        esp_err_t coreErr = Fdc2214CapReadCoreRegs(fdcState->handle, &coreRegs);
        uint8_t deglitchCode = (coreErr == ESP_OK) ?
            (uint8_t)(coreRegs.MuxConfig & SENSORARRAY_FDC_MUX_DEGLITCH_MASK) :
            (uint8_t)(fdcState->muxConfigReg & SENSORARRAY_FDC_MUX_DEGLITCH_MASK);
        uint32_t effectiveFclkHz =
            (fdcState->refClockKnown && fdcState->refClockHz != 0u) ?
            fdcState->refClockHz :
            sensorarrayMeasureFdcEffectiveFclkHz();

        if (coreErr != ESP_OK && firstErr == ESP_OK) {
            firstErr = coreErr;
        }

        for (uint8_t ch = 0u; ch < 4u; ++ch) {
            sensorarrayFdcRuntimeChannelConfig_t *cfg = &configs[dev][ch];
            cfg->deglitchCode = deglitchCode;
            cfg->effectiveFclkHz = effectiveFclkHz;

            esp_err_t err = Fdc2214CapReadRawRegisters(fdcState->handle,
                                                       sensorarrayMeasureFdcRegForChannel(SENSORARRAY_FDC_REG_RCOUNT_BASE,
                                                                                          (Fdc2214CapChannel_t)ch),
                                                       &cfg->rCount);
            if (err == ESP_OK) {
                err = Fdc2214CapReadRawRegisters(fdcState->handle,
                                                 sensorarrayMeasureFdcRegForChannel(SENSORARRAY_FDC_REG_SETTLECOUNT_BASE,
                                                                                    (Fdc2214CapChannel_t)ch),
                                                 &cfg->settleCount);
            }
            if (err == ESP_OK) {
                err = Fdc2214CapReadRawRegisters(fdcState->handle,
                                                 sensorarrayMeasureFdcRegForChannel(SENSORARRAY_FDC_REG_CLOCK_DIVIDERS_BASE,
                                                                                    (Fdc2214CapChannel_t)ch),
                                                 &cfg->clockDividers);
            }
            if (err == ESP_OK) {
                err = Fdc2214CapReadRawRegisters(fdcState->handle,
                                                 sensorarrayMeasureFdcRegForChannel(SENSORARRAY_FDC_REG_DRIVE_CURRENT_BASE,
                                                                                    (Fdc2214CapChannel_t)ch),
                                                 &cfg->driveCurrent);
            }
            cfg->valid = err == ESP_OK && cfg->clockDividers != 0u && cfg->effectiveFclkHz != 0u;
            if (err != ESP_OK && firstErr == ESP_OK) {
                firstErr = err;
            }
        }
    }
    return firstErr;
}

static esp_err_t __attribute__((unused)) sensorarrayMeasureDiscardFdcAutoscanRow(sensorarrayState_t *state,
                                                                                uint8_t sIndex,
                                                                                sensorarrayFdcDeviceId_t devId,
                                                                                uint8_t discardCount,
                                                                                const char *reason)
{
    if (!state || !sensorarrayMatrixIndexIsValid(sIndex, 1u)) {
        return ESP_ERR_INVALID_ARG;
    }
    if (discardCount == 0u) {
        return ESP_OK;
    }
    sensorarrayFdcDeviceState_t *fdcState = sensorarrayMeasureGetFdcState(state, devId);
    if (!fdcState || !fdcState->ready || !fdcState->handle) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t firstErr = ESP_OK;
#if CONFIG_SENSORARRAY_FDC_VERBOSE_SCAN_LOG
    printf("FDC_DISCARD,scope=device,row=%u,device=%s,count=%u,reason=%s\n",
           (unsigned)sIndex,
           sensorarrayMeasureFdcDeviceName(devId),
           (unsigned)discardCount,
           reason ? reason : SENSORARRAY_NA);
#endif

    for (uint8_t i = 0u; i < discardCount; ++i) {
        uint16_t status = 0u;
        esp_err_t err = sensorarrayMeasureWaitFdcAutoscanFrameReady(fdcState,
                                                                    sIndex,
                                                                    (uint32_t)SENSORARRAY_FDC_AUTOSCAN_READY_TIMEOUT_MS,
                                                                    &status);
        if (err == ESP_OK) {
            sensorarrayFdcAutoscanSamples_t discard = {0};
            err = sensorarrayMeasureReadFdcAutoscan4ch(fdcState, &discard);
        }
        if (err != ESP_OK && firstErr == ESP_OK) {
            firstErr = err;
        }
    }
    return firstErr;
}

static void sensorarrayMeasureMarkFdcMatrixCellEx(sensorarrayFdcMatrixFrame_t *frame,
                                                  uint8_t sIndex,
                                                  uint8_t dIndex,
                                                  uint32_t raw28,
                                                  double freqHz,
                                                  const sensorarrayFdcRuntimeChannelConfig_t *config,
                                                  bool fresh,
                                                  bool valid,
                                                  bool warning,
                                                  bool error);

static void sensorarrayMeasureFillFdcMatrixRow(sensorarrayFdcMatrixFrame_t *frame,
                                               uint8_t sIndex,
                                               const sensorarrayFdcAutoscanSamples_t *primary,
                                               const sensorarrayFdcAutoscanSamples_t *secondary,
                                               const sensorarrayFdcRuntimeChannelConfig_t configs[2][4],
                                               uint8_t *outValidMask8,
                                               uint8_t *outWarnMask8,
                                               uint8_t *outErrorMask8)
{
    uint8_t validMask8 = 0u;
    uint8_t warnMask8 = 0u;
    uint8_t errorMask8 = 0u;
    const sensorarrayFdcAutoscanSamples_t *samplesByHalf[2] = {primary, secondary};

    for (uint8_t half = 0u; half < 2u; ++half) {
        const sensorarrayFdcAutoscanSamples_t *samples = samplesByHalf[half];
        for (uint8_t ch = 0u; ch < 4u; ++ch) {
            uint8_t dIndex = (uint8_t)(1u + ch + (half * 4u));
            bool fresh = samples && samples->fresh[ch];
            bool valid = samples && samples->valid[ch];
            bool warning = samples && samples->amplitudeWarning[ch];
            const sensorarrayFdcRuntimeChannelConfig_t *config = configs ? &configs[half][ch] : NULL;
            bool configOk = config && config->valid;
            bool severeFault = !fresh ||
                               !valid ||
                               !configOk ||
                               (samples && (samples->i2cError[ch] ||
                                            samples->watchdogFault[ch] ||
                                            samples->saturated[ch] ||
                                            (fresh && samples->raw28[ch] == 0u)));
            bool error = severeFault;
            uint32_t raw28 = samples ? samples->raw28[ch] : 0u;
            double freqHz = (valid && configOk) ?
                sensorarrayMeasureFdcRaw28ToFreqHz(raw28, config->effectiveFclkHz, config->clockDividers) :
                0.0;
            bool frameValid = valid && freqHz > 0.0;
            sensorarrayMeasureMarkFdcMatrixCellEx(frame,
                                                  sIndex,
                                                  dIndex,
                                                  raw28,
                                                  freqHz,
                                                  config,
                                                  fresh,
                                                  frameValid,
                                                  warning,
                                                  error);
            if (frameValid) {
                validMask8 |= (uint8_t)(1u << (dIndex - 1u));
            }
            if (warning) {
                warnMask8 |= (uint8_t)(1u << (dIndex - 1u));
            }
            if (error) {
                errorMask8 |= (uint8_t)(1u << (dIndex - 1u));
            }
        }
    }

    if (outValidMask8) {
        *outValidMask8 = validMask8;
    }
    if (outWarnMask8) {
        *outWarnMask8 = warnMask8;
    }
    if (outErrorMask8) {
        *outErrorMask8 = errorMask8;
    }
}

static void sensorarrayMeasureAccumulateFdcHealth(sensorarrayFdcFrameHealth_t *health,
                                                  uint8_t sIndex,
                                                  const sensorarrayFdcAutoscanSamples_t *primary,
                                                  const sensorarrayFdcAutoscanSamples_t *secondary,
                                                  const sensorarrayFdcRuntimeChannelConfig_t configs[2][4],
                                                  const sensorarrayFdcMatrixFrame_t *frame)
{
    if (!health || !sensorarrayMatrixIndexIsValid(sIndex, 1u)) {
        return;
    }

    const sensorarrayFdcAutoscanSamples_t *samplesByHalf[2] = {primary, secondary};
    for (uint8_t half = 0u; half < 2u; ++half) {
        const sensorarrayFdcAutoscanSamples_t *samples = samplesByHalf[half];
        for (uint8_t ch = 0u; ch < 4u; ++ch) {
            uint8_t dIndex = (uint8_t)(1u + ch + (half * 4u));
            size_t matrixIndex = sensorarrayMatrixIndex(sIndex, dIndex);
            uint8_t s0 = (uint8_t)(sIndex - 1u);
            uint8_t d0 = (uint8_t)(dIndex - 1u);
            bool valid = samples && samples->valid[ch] && frame &&
                         ((frame->validMask & (1ULL << matrixIndex)) != 0u);
            bool fresh = samples && samples->fresh[ch];
            if (valid) {
                health->validSeen[s0][d0] = true;
                health->lastRaw28[s0][d0] = samples->raw28[ch];
                health->lastFreqHz[s0][d0] = frame ? frame->freqHz[matrixIndex] : 0.0;
            } else {
                health->invalidSeen[s0][d0] = true;
            }
            health->amplitudeWarningSeen[s0][d0] =
                health->amplitudeWarningSeen[s0][d0] || (samples && samples->amplitudeWarning[ch]);
            health->freshAmplitudeWarningSeen[s0][d0] =
                health->freshAmplitudeWarningSeen[s0][d0] ||
                (samples && samples->freshAmplitudeWarning[ch]);
            health->staleAmplitudeWarningSeen[s0][d0] =
                health->staleAmplitudeWarningSeen[s0][d0] ||
                (samples && samples->staleAmplitudeWarning[ch]);
            health->transientAmplitudeWarningSeen[s0][d0] =
                health->transientAmplitudeWarningSeen[s0][d0] ||
                (samples && samples->transientAmplitudeWarning[ch]);
            health->watchdogSeen[s0][d0] =
                health->watchdogSeen[s0][d0] || (samples && samples->watchdogFault[ch]);
            health->saturatedSeen[s0][d0] =
                health->saturatedSeen[s0][d0] || (samples && samples->saturated[ch]);
            health->zeroRawSeen[s0][d0] =
                health->zeroRawSeen[s0][d0] || (fresh && samples && samples->raw28[ch] == 0u);
            health->placeholderZeroSeen[s0][d0] =
                health->placeholderZeroSeen[s0][d0] || (!fresh && (!samples || samples->raw28[ch] == 0u));
            health->i2cErrorSeen[s0][d0] =
                health->i2cErrorSeen[s0][d0] || (samples && samples->i2cError[ch]);
            if (configs) {
                const sensorarrayFdcRuntimeChannelConfig_t *config = &configs[half][ch];
                health->clockDividers[s0][d0] = config->clockDividers;
                health->driveCurrent[s0][d0] = config->driveCurrent;
                health->deglitchCode[s0][d0] = config->deglitchCode;
                health->effectiveFclkHz[s0][d0] = config->effectiveFclkHz;
            }
        }
    }
}

static void sensorarrayMeasureUpdateFdcRuntimeProfiles(sensorarrayState_t *state,
                                                       const sensorarrayFdcFrameHealth_t *health)
{
    if (!state || !health) {
        return;
    }

    uint32_t threshold = (uint32_t)CONFIG_SENSORARRAY_FDC_RESCUE_HARD_ERROR_THRESHOLD;
    if (threshold == 0u) {
        threshold = 1u;
    }

    uint32_t amplitudeThreshold = (uint32_t)CONFIG_SENSORARRAY_FDC_WARNING_REAPPLY_THRESHOLD;
    if (amplitudeThreshold == 0u) {
        amplitudeThreshold = 1u;
    }

    int64_t nowUs = esp_timer_get_time();
    for (uint8_t s = 1u; s <= SENSORARRAY_MATRIX_ROWS; ++s) {
        for (uint8_t d = 1u; d <= SENSORARRAY_MATRIX_COLS; ++d) {
            uint8_t s0 = (uint8_t)(s - 1u);
            uint8_t d0 = (uint8_t)(d - 1u);
            uint8_t matrixIndex = (uint8_t)sensorarrayMatrixIndex(s, d);
            sensorarrayFdcCellTarget_t target = {0};
            if (!sensorarrayMeasureMakeFdcCellTarget(state, s, d, &target)) {
                continue;
            }

            sensorarrayFdcCellConfigCache_t *cache = sensorarrayMeasureGetFdcCellCache(state, &target);
            sensorarrayFdcDeviceState_t *fdcState = sensorarrayMeasureGetFdcState(state, target.devId);
            if (!cache || !fdcState || target.fdcChannel > (uint8_t)FDC2214_CH3) {
                continue;
            }

            sensorarrayFdcSweepProfile_t *profile = &fdcState->sweepProfile[target.fdcChannel];
            bool valid = health->validSeen[s0][d0];
            bool invalid = health->invalidSeen[s0][d0];
            bool amplitude = health->amplitudeWarningSeen[s0][d0];
            bool freshAmplitude = health->freshAmplitudeWarningSeen[s0][d0];
            bool staleAmplitude = health->staleAmplitudeWarningSeen[s0][d0];
            bool transientAmplitude = health->transientAmplitudeWarningSeen[s0][d0];
            bool severeError = invalid ||
                               health->i2cErrorSeen[s0][d0] ||
                               health->watchdogSeen[s0][d0] ||
                               health->saturatedSeen[s0][d0] ||
                               health->zeroRawSeen[s0][d0];

            if (valid) {
                bool wasCacheValid = cache->valid;
                if (!wasCacheValid) {
                    uint32_t generation = cache->generation + 1u;
                    cache->source = SENSORARRAY_FDC_CACHE_SOURCE_LAST_GOOD;
                    cache->rCount = health->clockDividers[s0][d0] ? SENSORARRAY_FDC_RCOUNT : cache->rCount;
                    cache->settleCount = SENSORARRAY_FDC_SETTLECOUNT;
                    cache->clockDiv = health->clockDividers[s0][d0];
                    cache->driveCurrent = health->driveCurrent[s0][d0];
                    cache->deglitchCode = health->deglitchCode[s0][d0];
                    cache->effectiveFclkHz = health->effectiveFclkHz[s0][d0];
                    cache->generation = (generation == 0u) ? 1u : generation;
                    cache->storedTimestampUs = nowUs;
                    sensorarrayMeasureMarkFdcAppliedCellDirty(state, &target);
                }
                cache->valid = true;
                cache->lastRaw28 = health->lastRaw28[s0][d0];
                cache->lastFreqHz = health->lastFreqHz[s0][d0];
                cache->lastGoodTimestampUs = nowUs;
                cache->consecutiveErrors = 0u;
                cache->consecutiveNoUnread = 0u;
                cache->consecutiveZeroRaw = 0u;
                cache->consecutiveWatchdogFaults = 0u;
                cache->consecutiveI2cErrors = 0u;

                profile->valid = true;
                profile->lastRaw28 = health->lastRaw28[s0][d0];
                profile->lastFrequencyHz = health->lastFreqHz[s0][d0];
                profile->lastValidTimestampUs = (uint64_t)nowUs;
                profile->consecutiveInvalid = 0u;
                profile->consecutiveWatchdogFault = 0u;
                profile->consecutiveSaturated = 0u;
                profile->consecutiveZeroRaw = 0u;

                sensorarrayFdcCellCalibration_t *cal = sensorarrayFdcSweepGetCellCalibration(s, d);
                if (cal) {
                    cal->hasLastGood = true;
                    cal->lockValid = true;
                    cal->lastGoodDriveCurrent = cache->driveCurrent ? cache->driveCurrent : SENSORARRAY_FDC_DRIVE_CURRENT;
                    cal->lastGoodDeglitch = cache->deglitchCode ?
                        (Fdc2214CapDeglitch_t)cache->deglitchCode :
                        FDC2214_DEGLITCH_10MHZ;
                    cal->lastGoodHighCurrent = false;
                    cal->lastGoodRaw28 = cache->lastRaw28;
                    cal->lastGoodFreqHz = cache->lastFreqHz;
                    cal->lastGoodTimestampUs = (uint64_t)nowUs;
                    cal->consecutiveFailCount = 0u;
                    cal->consecutiveNoUnreadCount = 0u;
                    cal->consecutiveStatusFaultCount = 0u;
                    cal->consecutiveZeroRawCount = 0u;
                    cal->directFailCount = 0u;
                    cal->lastFailReason = "valid";
                }
            } else if (severeError) {
                if (cache->consecutiveErrors < UINT16_MAX) {
                    cache->consecutiveErrors++;
                }
                if (invalid &&
                    !health->i2cErrorSeen[s0][d0] &&
                    !health->watchdogSeen[s0][d0] &&
                    !health->zeroRawSeen[s0][d0] &&
                    !health->saturatedSeen[s0][d0] &&
                    cache->consecutiveNoUnread < UINT16_MAX) {
                    cache->consecutiveNoUnread++;
                }
                if (health->zeroRawSeen[s0][d0] && cache->consecutiveZeroRaw < UINT16_MAX) {
                    cache->consecutiveZeroRaw++;
                }
                if (health->watchdogSeen[s0][d0] && cache->consecutiveWatchdogFaults < UINT16_MAX) {
                    cache->consecutiveWatchdogFaults++;
                }
                if (health->i2cErrorSeen[s0][d0] && cache->consecutiveI2cErrors < UINT16_MAX) {
                    cache->consecutiveI2cErrors++;
                }
                if (profile->consecutiveInvalid < UINT32_MAX) {
                    profile->consecutiveInvalid++;
                }
                if (health->watchdogSeen[s0][d0] && profile->consecutiveWatchdogFault < UINT32_MAX) {
                    profile->consecutiveWatchdogFault++;
                }
                if (health->saturatedSeen[s0][d0] && profile->consecutiveSaturated < UINT32_MAX) {
                    profile->consecutiveSaturated++;
                }
                if (health->zeroRawSeen[s0][d0] && profile->consecutiveZeroRaw < UINT32_MAX) {
                    profile->consecutiveZeroRaw++;
                }
            }

            if (freshAmplitude) {
                if (cache->consecutiveAmplitudeWarnings < UINT16_MAX) {
                    cache->consecutiveAmplitudeWarnings++;
                }
                cache->lastWarningTimestampUs = nowUs;
                snprintf(cache->lastWarningReason, sizeof(cache->lastWarningReason), "%s", "fresh_amplitude_warning");
                if (profile->consecutiveAmplitudeFault < UINT32_MAX) {
                    profile->consecutiveAmplitudeFault++;
                }
                printf("FDC_WARN,scope=cell,s=%u,d=%u,index=%u,device=%s,ch=%u,class=fresh,reason=amplitude_warning,consecutive=%u,fingerprint=%lu\n",
                       (unsigned)s,
                       (unsigned)d,
                       (unsigned)matrixIndex,
                       sensorarrayMeasureFdcDeviceName(target.devId),
                       (unsigned)target.fdcChannel,
                       (unsigned)cache->consecutiveAmplitudeWarnings,
                       (unsigned long)cache->lastAppliedFingerprint);
            } else {
                cache->consecutiveAmplitudeWarnings = 0u;
                profile->consecutiveAmplitudeFault = 0u;
            }

            if (staleAmplitude) {
                if (cache->staleAmplitudeWarnings < UINT16_MAX) {
                    cache->staleAmplitudeWarnings++;
                }
                printf("FDC_WARN,scope=cell,s=%u,d=%u,index=%u,device=%s,ch=%u,class=stale,reason=amplitude_warning,action=suppress_rescue\n",
                       (unsigned)s,
                       (unsigned)d,
                       (unsigned)matrixIndex,
                       sensorarrayMeasureFdcDeviceName(target.devId),
                       (unsigned)target.fdcChannel);
            }
            if (transientAmplitude) {
                if (cache->transientAmplitudeWarnings < UINT16_MAX) {
                    cache->transientAmplitudeWarnings++;
                }
                printf("FDC_WARN,scope=cell,s=%u,d=%u,index=%u,device=%s,ch=%u,class=transient,reason=amplitude_warning,action=suppress_rescue\n",
                       (unsigned)s,
                       (unsigned)d,
                       (unsigned)matrixIndex,
                       sensorarrayMeasureFdcDeviceName(target.devId),
                       (unsigned)target.fdcChannel);
            }

            if (freshAmplitude &&
                cache->valid &&
                CONFIG_SENSORARRAY_FDC_REAPPLY_CACHE_ON_WARNING &&
                cache->consecutiveAmplitudeWarnings >= amplitudeThreshold) {
                uint32_t cooldownFrames = (uint32_t)CONFIG_SENSORARRAY_FDC_WARNING_REAPPLY_COOLDOWN_FRAMES;
                bool cooldownElapsed =
                    cooldownFrames == 0u ||
                    s_fdcMatrixSequence >= cache->lastReapplyFrame + cooldownFrames;
                bool fingerprintAlreadyReapplied =
                    CONFIG_SENSORARRAY_FDC_WARNING_REAPPLY_ONCE_PER_FINGERPRINT &&
                    cache->lastReapplyFingerprint == cache->lastAppliedFingerprint;
                if (!fingerprintAlreadyReapplied && cooldownElapsed && !cache->reapplyPending) {
                    cache->reapplyPending = true;
                    cache->lastReapplyFingerprint = cache->lastAppliedFingerprint;
                    cache->lastReapplyFrame = s_fdcMatrixSequence;
                    printf("FDC_RESCUE_DECISION,scope=cell,s=%u,d=%u,index=%u,device=%s,ch=%u,class=fresh,action=sanity_reapply_once,reason=amplitude_warning,consecutive=%u,fingerprint=%lu\n",
                           (unsigned)s,
                           (unsigned)d,
                           (unsigned)matrixIndex,
                           sensorarrayMeasureFdcDeviceName(target.devId),
                           (unsigned)target.fdcChannel,
                           (unsigned)cache->consecutiveAmplitudeWarnings,
                           (unsigned long)cache->lastAppliedFingerprint);
                } else {
                    printf("FDC_RESCUE_DECISION,scope=cell,s=%u,d=%u,index=%u,device=%s,ch=%u,class=fresh,action=suppress_reapply,reason=amplitude_warning,consecutive=%u,fingerprint=%lu,policy=%s\n",
                           (unsigned)s,
                           (unsigned)d,
                           (unsigned)matrixIndex,
                           sensorarrayMeasureFdcDeviceName(target.devId),
                           (unsigned)target.fdcChannel,
                           (unsigned)cache->consecutiveAmplitudeWarnings,
                           (unsigned long)cache->lastAppliedFingerprint,
                           fingerprintAlreadyReapplied ? "fingerprint_already_reapplied" : "cooldown");
                }
            } else if (amplitude && !freshAmplitude) {
                printf("FDC_RESCUE_DECISION,scope=cell,s=%u,d=%u,index=%u,device=%s,ch=%u,class=%s,action=suppress_rescue,reason=amplitude_warning,policy=non_fresh_epoch\n",
                       (unsigned)s,
                       (unsigned)d,
                       (unsigned)matrixIndex,
                       sensorarrayMeasureFdcDeviceName(target.devId),
                       (unsigned)target.fdcChannel,
                       staleAmplitude ? "stale" : "transient");
            }

            if (freshAmplitude &&
                cache->valid &&
                cache->consecutiveAmplitudeWarnings >=
                    (uint16_t)CONFIG_SENSORARRAY_FDC_AMPLITUDE_FAST_SWEEP_THRESHOLD) {
                int64_t fastCooldownUs =
                    (int64_t)CONFIG_SENSORARRAY_FDC_WARNING_FAST_SWEEP_COOLDOWN_MS * 1000LL;
                bool fastCooldownElapsed =
                    cache->lastFastSweepRequestUs == 0u ||
                    fastCooldownUs == 0 ||
                    (nowUs - (int64_t)cache->lastFastSweepRequestUs) >= fastCooldownUs;
                if (fastCooldownElapsed) {
                    cache->lastFastSweepRequestUs = (uint64_t)nowUs;
                    printf("FDC_RESCUE_DECISION,scope=cell,s=%u,d=%u,index=%u,device=%s,ch=%u,class=fresh,action=fast_sweep,reason=persistent_fresh_amplitude_warning_after_cache_apply,consecutive=%u,fingerprint=%lu\n",
                           (unsigned)s,
                           (unsigned)d,
                           (unsigned)matrixIndex,
                           sensorarrayMeasureFdcDeviceName(target.devId),
                           (unsigned)target.fdcChannel,
                           (unsigned)cache->consecutiveAmplitudeWarnings,
                           (unsigned long)cache->lastAppliedFingerprint);
                    (void)sensorarrayMeasureRequestFdcCellRescue(state,
                                                                 matrixIndex,
                                                                 "persistent_fresh_amplitude_warning_after_cache_apply");
                } else {
                    printf("FDC_RESCUE_DECISION,scope=cell,s=%u,d=%u,index=%u,device=%s,ch=%u,class=fresh,action=suppress_fast_sweep,reason=amplitude_warning,policy=cooldown,consecutive=%u\n",
                           (unsigned)s,
                           (unsigned)d,
                           (unsigned)matrixIndex,
                           sensorarrayMeasureFdcDeviceName(target.devId),
                           (unsigned)target.fdcChannel,
                           (unsigned)cache->consecutiveAmplitudeWarnings);
                }
            }

            const char *rescueReason =
                (severeError && cache->consecutiveI2cErrors >= threshold) ? "i2c_error_consecutive" :
                (severeError && cache->consecutiveWatchdogFaults >= threshold) ? "watchdog_fault_consecutive" :
                (severeError && cache->consecutiveZeroRaw >= threshold) ? "zero_raw_consecutive" :
                (severeError && cache->consecutiveNoUnread >= threshold) ? "no_unread_consecutive" :
                (severeError && cache->consecutiveErrors >= threshold) ? "invalid_streak" :
                NULL;
            if (rescueReason) {
                (void)sensorarrayMeasureRequestFdcCellRescue(state, matrixIndex, rescueReason);
            }
        }
    }
}

static void sensorarrayMeasureCountFdcFrameWarnings(const sensorarrayFdcFrameHealth_t *health,
                                                    sensorarrayFdcTimingSummary_t *timing)
{
    if (!health || !timing) {
        return;
    }
    for (uint8_t s = 0u; s < SENSORARRAY_MATRIX_ROWS; ++s) {
        for (uint8_t d = 0u; d < SENSORARRAY_MATRIX_COLS; ++d) {
            timing->freshAmplitudeWarningCount += health->freshAmplitudeWarningSeen[s][d] ? 1u : 0u;
            timing->staleAmplitudeWarningCount += health->staleAmplitudeWarningSeen[s][d] ? 1u : 0u;
            timing->transientAmplitudeWarningCount += health->transientAmplitudeWarningSeen[s][d] ? 1u : 0u;
        }
    }
}

static void sensorarrayMeasureInitFdcMatrixFrame(sensorarrayFdcMatrixFrame_t *frame)
{
    memset(frame, 0, sizeof(*frame));
    frame->timestampUs = (uint64_t)esp_timer_get_time();
    frame->sequence = s_fdcMatrixSequence++;
}

static bool sensorarrayFdcMatrixFrameRawAllZero(const sensorarrayFdcMatrixFrame_t *frame)
{
    if (!frame) {
        return true;
    }
    return frame->freshCount == SENSORARRAY_MATRIX_CELL_COUNT &&
           frame->hardwareZeroRawCount == SENSORARRAY_MATRIX_CELL_COUNT;
}

static uint64_t sensorarrayMeasureComputeFdcFrameCapTotalPf(sensorarrayFdcMatrixFrame_t *frame)
{
    if (!frame) {
        return 0u;
    }

    int64_t startUs = esp_timer_get_time();
    frame->capValidMask = 0u;
    const double inductorUh = (double)CONFIG_SENSORARRAY_FDC_TANK_INDUCTOR_NH / 1000.0;
    for (size_t i = 0u; i < SENSORARRAY_MATRIX_CELL_COUNT; ++i) {
        frame->capTotalPf[i] = 0.0;
        if ((frame->validMask & (1ULL << i)) == 0u) {
            continue;
        }
        double capPf = 0.0;
        if (sensorarrayMeasureFdcComputeCapacitancePf(frame->freqHz[i], inductorUh, &capPf)) {
            frame->capTotalPf[i] = capPf;
            frame->capValidMask |= (1ULL << i);
        } else if (frame->freqHz[i] > 0.0 && frame->raw28[i] != 0u) {
            uint8_t sColumn = 0u;
            uint8_t dLine = 0u;
            if (sensorarrayMeasureDecodeMatrixIndex((uint8_t)i, &sColumn, &dLine)) {
                printf("MATRIXFDC_DIAG,reason=cap_calc_zero_with_nonzero_freq,row=%u,d=%u,freqHz=%.3f,raw28=%lu,inductorNh=%lu\n",
                       (unsigned)sColumn,
                       (unsigned)dLine,
                       frame->freqHz[i],
                       (unsigned long)frame->raw28[i],
                       (unsigned long)CONFIG_SENSORARRAY_FDC_TANK_INDUCTOR_NH);
            }
        }
    }
    return sensorarrayMeasureElapsedUs(startUs);
}

static void sensorarrayMeasureMarkFdcMatrixCellEx(sensorarrayFdcMatrixFrame_t *frame,
                                                  uint8_t sIndex,
                                                  uint8_t dIndex,
                                                  uint32_t raw28,
                                                  double freqHz,
                                                  const sensorarrayFdcRuntimeChannelConfig_t *config,
                                                  bool fresh,
                                                  bool valid,
                                                  bool warning,
                                                  bool error)
{
    if (!frame || !sensorarrayMatrixIndexIsValid(sIndex, dIndex)) {
        return;
    }

    size_t index = sensorarrayMatrixIndex(sIndex, dIndex);
    uint64_t bit = 1ULL << index;
    frame->raw28[index] = raw28;
    frame->freqHz[index] = freqHz;
    if (config) {
        frame->clockDividers[index] = config->clockDividers;
        frame->driveCurrent[index] = config->driveCurrent;
        frame->deglitchCode[index] = config->deglitchCode;
        frame->effectiveFclkHz[index] = config->effectiveFclkHz;
    }
    if (valid) {
        frame->validMask |= bit;
        if (frame->validCount < UINT8_MAX) {
            frame->validCount++;
        }
    } else {
        frame->validMask &= ~bit;
    }
    if (fresh) {
        frame->freshMask |= bit;
        if (frame->freshCount < UINT8_MAX) {
            frame->freshCount++;
        }
        if (raw28 == 0u && frame->hardwareZeroRawCount < UINT8_MAX) {
            frame->hardwareZeroRawCount++;
        }
    } else {
        frame->freshMask &= ~bit;
        if (raw28 == 0u && frame->placeholderZeroCount < UINT8_MAX) {
            frame->placeholderZeroCount++;
        }
    }
    if (warning) {
        frame->warnMask |= bit;
    } else {
        frame->warnMask &= ~bit;
    }
    if (error) {
        frame->errorMask |= bit;
        if (frame->firstBadRow == 0u) {
            frame->firstBadRow = sIndex;
            frame->firstBadDevice = (dIndex > 4u) ?
                (uint8_t)SENSORARRAY_FDC_DEV_SECONDARY :
                (uint8_t)SENSORARRAY_FDC_DEV_PRIMARY;
        }
    } else {
        frame->errorMask &= ~bit;
    }
}

static esp_err_t sensorarrayMeasureCheckFdcMatrixReady(sensorarrayState_t *state)
{
    if (!state) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!state->boardReady || !state->tmuxReady || !state->adsReady) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!state->fdcPrimary.ready || !state->fdcPrimary.handle ||
        !state->fdcSecondary.ready || !state->fdcSecondary.handle) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!state->fdcPrimary.i2cCtx || !state->fdcSecondary.i2cCtx) {
        return ESP_ERR_INVALID_STATE;
    }
    return ESP_OK;
}

static uint64_t sensorarrayMeasureElapsedUs(int64_t startUs)
{
    int64_t elapsedUs = esp_timer_get_time() - startUs;
    return (elapsedUs > 0) ? (uint64_t)elapsedUs : 0u;
}

static uint64_t sensorarrayMeasureEstimateI2cBits(uint32_t writeCount,
                                                  uint32_t readCount,
                                                  uint32_t writeBytes,
                                                  uint32_t readBytes)
{
    /*
     * Approximate START/address/ACK/STOP cost. This is intentionally an estimate
     * for profiling deltas, not a replacement for logic-analyzer timing.
     */
    uint64_t dataBytes = (uint64_t)writeBytes + (uint64_t)readBytes;
    uint64_t dataBitsWithAck = dataBytes * 9ull;
    uint64_t addressBitsWithAck = ((uint64_t)writeCount + ((uint64_t)readCount * 2ull)) * 9ull;
    uint64_t framingBits = ((uint64_t)writeCount + (uint64_t)readCount) * 2ull;
    return dataBitsWithAck + addressBitsWithAck + framingBits;
}

static void sensorarrayMeasureMergeFdcI2cStats(const Fdc2214CapI2cStats_t *primaryStats,
                                               const Fdc2214CapI2cStats_t *secondaryStats,
                                               const BoardSupportI2cBusInfo_t *primaryBus,
                                               const BoardSupportI2cBusInfo_t *secondaryBus,
                                               sensorarrayFdcTimingSummary_t *summary)
{
    if (!summary) {
        return;
    }

    Fdc2214CapI2cStats_t primary = primaryStats ? *primaryStats : (Fdc2214CapI2cStats_t){0};
    Fdc2214CapI2cStats_t secondary = secondaryStats ? *secondaryStats : (Fdc2214CapI2cStats_t){0};

    summary->i2cWriteCount = primary.writeCount + secondary.writeCount;
    summary->i2cReadCount = primary.readCount + secondary.readCount;
    summary->i2cVerifyReadCount = primary.verifyReadCount + secondary.verifyReadCount;
    summary->i2cRetryCount = primary.retryCount + secondary.retryCount;
    summary->i2cNackCount = primary.nackCount + secondary.nackCount;
    summary->i2cTimeoutCount = primary.timeoutCount + secondary.timeoutCount;
    summary->i2cRecoveryCount = primary.recoveryCount + secondary.recoveryCount;

    summary->i2cBus0WriteCount = primary.writeCount;
    summary->i2cBus0ReadCount = primary.readCount;
    summary->i2cBus0WriteBytes = primary.writeBytes;
    summary->i2cBus0ReadBytes = primary.readBytes;
    summary->i2cBus0TotalUs = primary.totalUs;
    summary->i2cBus0RetryCount = primary.retryCount;
    summary->i2cBus0NackCount = primary.nackCount;
    summary->i2cBus0TimeoutCount = primary.timeoutCount;

    summary->i2cBus1WriteCount = secondary.writeCount;
    summary->i2cBus1ReadCount = secondary.readCount;
    summary->i2cBus1WriteBytes = secondary.writeBytes;
    summary->i2cBus1ReadBytes = secondary.readBytes;
    summary->i2cBus1TotalUs = secondary.totalUs;
    summary->i2cBus1RetryCount = secondary.retryCount;
    summary->i2cBus1NackCount = secondary.nackCount;
    summary->i2cBus1TimeoutCount = secondary.timeoutCount;

    uint32_t primaryFreq = (primaryBus && primaryBus->FrequencyHz != 0u) ?
        primaryBus->FrequencyHz :
        (uint32_t)CONFIG_BOARD_I2C_FREQ_HZ;
    uint32_t secondaryFreq = (secondaryBus && secondaryBus->FrequencyHz != 0u) ?
        secondaryBus->FrequencyHz :
        primaryFreq;
    summary->i2cFreqHz = (primaryFreq == secondaryFreq) ? primaryFreq : 0u;

    uint64_t primaryBits = sensorarrayMeasureEstimateI2cBits(primary.writeCount,
                                                             primary.readCount,
                                                             primary.writeBytes,
                                                             primary.readBytes);
    uint64_t secondaryBits = sensorarrayMeasureEstimateI2cBits(secondary.writeCount,
                                                               secondary.readCount,
                                                               secondary.writeBytes,
                                                               secondary.readBytes);
    summary->i2cEstimatedBits = primaryBits + secondaryBits;
    uint64_t primaryEstimatedUs = primaryFreq ? ((primaryBits * 1000000ull) / primaryFreq) : 0ull;
    uint64_t secondaryEstimatedUs = secondaryFreq ? ((secondaryBits * 1000000ull) / secondaryFreq) : 0ull;
    if (CONFIG_SENSORARRAY_FDC_PARALLEL_DUAL_BUS_READ && s_fdcWorkersAvailable) {
        summary->i2cEstimatedBusUs = (primaryEstimatedUs > secondaryEstimatedUs) ?
            primaryEstimatedUs :
            secondaryEstimatedUs;
        summary->i2cMeasuredUs = (primary.totalUs > secondary.totalUs) ?
            primary.totalUs :
            secondary.totalUs;
    } else {
        summary->i2cEstimatedBusUs = primaryEstimatedUs + secondaryEstimatedUs;
        summary->i2cMeasuredUs = primary.totalUs + secondary.totalUs;
    }
    summary->i2cOverheadUs = (int64_t)summary->i2cMeasuredUs - (int64_t)summary->i2cEstimatedBusUs;
}

static void sensorarrayMeasurePrintFdcTimingSummary(const sensorarrayFdcTimingSummary_t *summary,
                                                    uint32_t sequence)
{
    if (!summary) {
        return;
    }

    uint64_t targetFrameUs = SENSORARRAY_FDC_TARGET_FRAME_US;
    uint64_t overrunUs = (summary->frameUs > targetFrameUs) ? (summary->frameUs - targetFrameUs) : 0u;
    uint64_t fpsX100 = summary->frameUs ? (100000000ull / summary->frameUs) : 0ull;
    uint64_t budgetUsePct = targetFrameUs ? ((summary->frameUs * 100ull) / targetFrameUs) : 0ull;

    printf("SCAN_TIMING_FRAME,seq=%lu,targetFps=%lu,targetFrameUs=%llu,frameUs=%llu,fps=%llu.%02llu,budgetUsePct=%llu,overrun=%u,overrunUs=%llu,rowAvgUs=%llu,rowMaxUs=%llu,rowMinUs=%llu,slowRow=%u,pathEnsureUs=%llu,cacheApplyUs=%llu,applyBuildConfigUs=%llu,applyChannelConfigWriteUs=%llu,applyGlobalConfigWriteUs=%llu,applyVerifyUs=%llu,applyDelayUs=%llu,applyReadyWaitUs=%llu,applyMutexWaitUs=%llu,applyLogUs=%llu,discardUs=%llu,waitReadyUs=%llu,readUs=%llu,emitUs=%llu,capComputeUs=%llu,sweepUs=%llu,runtimeSweepCount=%lu,intbTimeoutCount=%lu,readyFullCount=%lu,readyPartialCount=%lu,readyNoneCount=%lu,fallbackAttemptCount=%lu,fallbackSuccessCount=%lu,fallbackPartialCount=%lu,fallbackFailCount=%lu,rowFullInvalidCount=%lu,deviceFullInvalidCount=%lu,avgWaitReadyUsPrimary=%llu,avgWaitReadyUsSecondary=%llu,maxWaitReadyUs=%llu,avgRead4UsPrimary=%llu,avgRead4UsSecondary=%llu,maxI2cReadUs=%llu,sweepRequestCount=%lu,sweepActuallyQueuedCount=%lu,i2cWriteCount=%lu,i2cReadCount=%lu,i2cVerifyReadCount=%lu,i2cRetryCount=%lu,i2cNackCount=%lu,i2cTimeoutCount=%lu,i2cRecoveryCount=%lu,i2cFreqHz=%lu,i2cEstimatedBits=%llu,i2cEstimatedBusUs=%llu,i2cMeasuredUs=%llu,i2cOverheadUs=%lld,i2cBus0WriteCount=%lu,i2cBus0ReadCount=%lu,i2cBus0WriteBytes=%lu,i2cBus0ReadBytes=%lu,i2cBus0TotalUs=%llu,i2cBus0RetryCount=%lu,i2cBus0NackCount=%lu,i2cBus0TimeoutCount=%lu,i2cBus1WriteCount=%lu,i2cBus1ReadCount=%lu,i2cBus1WriteBytes=%lu,i2cBus1ReadBytes=%lu,i2cBus1TotalUs=%llu,i2cBus1RetryCount=%lu,i2cBus1NackCount=%lu,i2cBus1TimeoutCount=%lu\n",
           (unsigned long)sequence,
           (unsigned long)CONFIG_SENSORARRAY_FDC_MATRIX_TARGET_FPS,
           (unsigned long long)targetFrameUs,
           (unsigned long long)summary->frameUs,
           (unsigned long long)(fpsX100 / 100ull),
           (unsigned long long)(fpsX100 % 100ull),
           (unsigned long long)budgetUsePct,
           overrunUs ? 1u : 0u,
           (unsigned long long)overrunUs,
           (unsigned long long)summary->rowAvgUs,
           (unsigned long long)summary->rowMaxUs,
           (unsigned long long)summary->rowMinUs,
           (unsigned)summary->slowRow,
           (unsigned long long)summary->pathEnsureUs,
           (unsigned long long)summary->cacheApplyUs,
           (unsigned long long)summary->applyBuildConfigUs,
           (unsigned long long)summary->applyChannelConfigWriteUs,
           (unsigned long long)summary->applyGlobalConfigWriteUs,
           (unsigned long long)summary->applyVerifyUs,
           (unsigned long long)summary->applyDelayUs,
           (unsigned long long)summary->applyReadyWaitUs,
           (unsigned long long)summary->applyMutexWaitUs,
           (unsigned long long)summary->applyLogUs,
           (unsigned long long)summary->discardUs,
           (unsigned long long)summary->waitReadyUs,
           (unsigned long long)summary->readUs,
           (unsigned long long)summary->emitUs,
           (unsigned long long)summary->capComputeUs,
           (unsigned long long)summary->sweepUs,
           (unsigned long)summary->runtimeSweepCount,
           (unsigned long)summary->intbTimeoutCount,
           (unsigned long)summary->readyFullCount,
           (unsigned long)summary->readyPartialCount,
           (unsigned long)summary->readyNoneCount,
           (unsigned long)summary->fallbackAttemptCount,
           (unsigned long)summary->fallbackSuccessCount,
           (unsigned long)summary->fallbackPartialCount,
           (unsigned long)summary->fallbackFailCount,
           (unsigned long)summary->rowFullInvalidCount,
           (unsigned long)summary->deviceFullInvalidCount,
           (unsigned long long)sensorarrayMeasureAvgU64(summary->waitReadyUsPrimaryTotal, SENSORARRAY_MATRIX_ROWS),
           (unsigned long long)sensorarrayMeasureAvgU64(summary->waitReadyUsSecondaryTotal, SENSORARRAY_MATRIX_ROWS),
           (unsigned long long)summary->maxWaitReadyUs,
           (unsigned long long)sensorarrayMeasureAvgU64(summary->read4UsPrimaryTotal, SENSORARRAY_MATRIX_ROWS),
           (unsigned long long)sensorarrayMeasureAvgU64(summary->read4UsSecondaryTotal, SENSORARRAY_MATRIX_ROWS),
           (unsigned long long)summary->maxI2cReadUs,
           (unsigned long)summary->sweepRequestCount,
           (unsigned long)summary->sweepActuallyQueuedCount,
           (unsigned long)summary->i2cWriteCount,
           (unsigned long)summary->i2cReadCount,
           (unsigned long)summary->i2cVerifyReadCount,
           (unsigned long)summary->i2cRetryCount,
           (unsigned long)summary->i2cNackCount,
           (unsigned long)summary->i2cTimeoutCount,
           (unsigned long)summary->i2cRecoveryCount,
           (unsigned long)summary->i2cFreqHz,
           (unsigned long long)summary->i2cEstimatedBits,
           (unsigned long long)summary->i2cEstimatedBusUs,
           (unsigned long long)summary->i2cMeasuredUs,
           (long long)summary->i2cOverheadUs,
           (unsigned long)summary->i2cBus0WriteCount,
           (unsigned long)summary->i2cBus0ReadCount,
           (unsigned long)summary->i2cBus0WriteBytes,
           (unsigned long)summary->i2cBus0ReadBytes,
           (unsigned long long)summary->i2cBus0TotalUs,
           (unsigned long)summary->i2cBus0RetryCount,
           (unsigned long)summary->i2cBus0NackCount,
           (unsigned long)summary->i2cBus0TimeoutCount,
           (unsigned long)summary->i2cBus1WriteCount,
           (unsigned long)summary->i2cBus1ReadCount,
           (unsigned long)summary->i2cBus1WriteBytes,
           (unsigned long)summary->i2cBus1ReadBytes,
           (unsigned long long)summary->i2cBus1TotalUs,
           (unsigned long)summary->i2cBus1RetryCount,
           (unsigned long)summary->i2cBus1NackCount,
           (unsigned long)summary->i2cBus1TimeoutCount);
}

static uint64_t sensorarrayMeasureAvgU64(uint64_t total, uint32_t count)
{
    return count ? (total / count) : 0u;
}

static void sensorarrayMeasurePrintFdcTimingAggregate(const sensorarrayFdcTimingAggregate_t *agg)
{
    if (!agg || agg->frames == 0u) {
        return;
    }

    const sensorarrayFdcTimingSummary_t *t = &agg->totals;
    uint64_t avgFrameUs = sensorarrayMeasureAvgU64(agg->frameUsTotal, agg->frames);
    uint64_t avgFpsX100 = avgFrameUs ? (100000000ull / avgFrameUs) : 0ull;
    uint64_t budgetUsePctAvg = SENSORARRAY_FDC_TARGET_FRAME_US ?
        ((avgFrameUs * 100ull) / SENSORARRAY_FDC_TARGET_FRAME_US) :
        0ull;

    printf("SCAN_TIMING_10,seqStart=%lu,seqEnd=%lu,frames=%lu,targetFps=%lu,avgFrameUs=%llu,minFrameUs=%llu,maxFrameUs=%llu,avgFps=%llu.%02llu,budgetUsePctAvg=%llu,overrunCount=%lu,rowAvgUs=%llu,rowMaxUs=%llu,slowRow=%u,sleepBeforeRowSwitchAvgUs=%llu,rowSwitchWhileSleepingAvgUs=%llu,rowSettleAvgUs=%llu,diffApplyWhileSleepingAvgUs=%llu,sleepTotalAvgUs=%llu,sleepExitToIntbAvgUs=%llu,statusReadAvgUs=%llu,dataReadAvgUs=%llu,primaryJobAvgUs=%llu,secondaryJobAvgUs=%llu,dualBusWaitAvgUs=%llu,dualBusSkewAvgUs=%llu,cacheApplyAvgUs=%llu,cacheDiffWriteAvg=%llu,i2cWriteAvg=%llu,i2cReadAvg=%llu,i2cMeasuredAvgUs=%llu,bus0AvgUs=%llu,bus1AvgUs=%llu,freshWarnCount=%lu,staleWarnCount=%lu,transientWarnCount=%lu,fastSweepCount=%lu,intbFreshDrdyCount=%lu,intbTimeoutCount=%lu,intbFalseEdgeCount=%lu,readyFullCount=%lu,readyPartialCount=%lu,readyNoneCount=%lu,fallbackAttemptCount=%lu,fallbackSuccessCount=%lu,fallbackPartialCount=%lu,fallbackFailCount=%lu,rowFullInvalidCount=%lu,deviceFullInvalidCount=%lu,avgWaitReadyUsPrimary=%llu,avgWaitReadyUsSecondary=%llu,maxWaitReadyUs=%llu,avgRead4UsPrimary=%llu,avgRead4UsSecondary=%llu,maxI2cReadUs=%llu,sweepRequestCount=%lu,sweepActuallyQueuedCount=%lu,outputMode=%s,rowRestartMethod=sleep,parallel=%u,intb=%u\n",
           (unsigned long)agg->seqStart,
           (unsigned long)agg->seqEnd,
           (unsigned long)agg->frames,
           (unsigned long)CONFIG_SENSORARRAY_FDC_MATRIX_TARGET_FPS,
           (unsigned long long)avgFrameUs,
           (unsigned long long)agg->frameUsMin,
           (unsigned long long)agg->frameUsMax,
           (unsigned long long)(avgFpsX100 / 100ull),
           (unsigned long long)(avgFpsX100 % 100ull),
           (unsigned long long)budgetUsePctAvg,
           (unsigned long)agg->overrunCount,
           (unsigned long long)sensorarrayMeasureAvgU64(agg->rowUsTotal, agg->frames * SENSORARRAY_MATRIX_ROWS),
           (unsigned long long)agg->rowUsMax,
           (unsigned)agg->slowRow,
           (unsigned long long)sensorarrayMeasureAvgU64(t->sleepBeforeRowSwitchUs, agg->frames * SENSORARRAY_MATRIX_ROWS),
           (unsigned long long)sensorarrayMeasureAvgU64(t->rowSwitchWhileSleepingUs, agg->frames * SENSORARRAY_MATRIX_ROWS),
           (unsigned long long)sensorarrayMeasureAvgU64(t->rowSettleUs, agg->frames * SENSORARRAY_MATRIX_ROWS),
           (unsigned long long)sensorarrayMeasureAvgU64(t->diffApplyWhileSleepingUs, agg->frames * SENSORARRAY_MATRIX_ROWS),
           (unsigned long long)sensorarrayMeasureAvgU64(t->sleepTotalUs, agg->frames * SENSORARRAY_MATRIX_ROWS),
           (unsigned long long)sensorarrayMeasureAvgU64(t->sleepExitToIntbUs, agg->frames * SENSORARRAY_MATRIX_ROWS),
           (unsigned long long)sensorarrayMeasureAvgU64(t->statusReadUs, agg->frames * SENSORARRAY_MATRIX_ROWS),
           (unsigned long long)sensorarrayMeasureAvgU64(t->dataReadUs, agg->frames * SENSORARRAY_MATRIX_ROWS),
           (unsigned long long)sensorarrayMeasureAvgU64(t->primaryJobUs, agg->frames * SENSORARRAY_MATRIX_ROWS),
           (unsigned long long)sensorarrayMeasureAvgU64(t->secondaryJobUs, agg->frames * SENSORARRAY_MATRIX_ROWS),
           (unsigned long long)sensorarrayMeasureAvgU64(t->dualBusWaitUs, agg->frames * SENSORARRAY_MATRIX_ROWS),
           (unsigned long long)sensorarrayMeasureAvgU64(t->dualBusSkewUs, agg->frames * SENSORARRAY_MATRIX_ROWS),
           (unsigned long long)sensorarrayMeasureAvgU64(t->cacheApplyUs, agg->frames),
           (unsigned long long)sensorarrayMeasureAvgU64(t->cacheApplyDiffWriteCount, agg->frames),
           (unsigned long long)sensorarrayMeasureAvgU64(t->i2cWriteCount, agg->frames),
           (unsigned long long)sensorarrayMeasureAvgU64(t->i2cReadCount, agg->frames),
           (unsigned long long)sensorarrayMeasureAvgU64(t->i2cMeasuredUs, agg->frames),
           (unsigned long long)sensorarrayMeasureAvgU64(t->i2cBus0TotalUs, agg->frames),
           (unsigned long long)sensorarrayMeasureAvgU64(t->i2cBus1TotalUs, agg->frames),
           (unsigned long)t->freshAmplitudeWarningCount,
           (unsigned long)t->staleAmplitudeWarningCount,
           (unsigned long)t->transientAmplitudeWarningCount,
           (unsigned long)t->warningFastSweepRequestedCount,
           (unsigned long)t->intbFreshDrdyCount,
           (unsigned long)t->intbTimeoutCount,
           (unsigned long)t->intbFalseEdgeCount,
           (unsigned long)t->readyFullCount,
           (unsigned long)t->readyPartialCount,
           (unsigned long)t->readyNoneCount,
           (unsigned long)t->fallbackAttemptCount,
           (unsigned long)t->fallbackSuccessCount,
           (unsigned long)t->fallbackPartialCount,
           (unsigned long)t->fallbackFailCount,
           (unsigned long)t->rowFullInvalidCount,
           (unsigned long)t->deviceFullInvalidCount,
           (unsigned long long)sensorarrayMeasureAvgU64(t->waitReadyUsPrimaryTotal,
                                                        agg->frames * SENSORARRAY_MATRIX_ROWS),
           (unsigned long long)sensorarrayMeasureAvgU64(t->waitReadyUsSecondaryTotal,
                                                        agg->frames * SENSORARRAY_MATRIX_ROWS),
           (unsigned long long)t->maxWaitReadyUs,
           (unsigned long long)sensorarrayMeasureAvgU64(t->read4UsPrimaryTotal,
                                                        agg->frames * SENSORARRAY_MATRIX_ROWS),
           (unsigned long long)sensorarrayMeasureAvgU64(t->read4UsSecondaryTotal,
                                                        agg->frames * SENSORARRAY_MATRIX_ROWS),
           (unsigned long long)t->maxI2cReadUs,
           (unsigned long)t->sweepRequestCount,
           (unsigned long)t->sweepActuallyQueuedCount,
#if CONFIG_SENSORARRAY_FDC_TEXT_OUTPUT_FREQ_HZ
           "freqHz",
#elif CONFIG_SENSORARRAY_FDC_TEXT_OUTPUT_BOTH_SEPARATE
           "bothSeparate",
#elif CONFIG_SENSORARRAY_FDC_TEXT_OUTPUT_BOTH_INLINE_DEBUG
           "bothInlineDebug",
#else
           "capTotalPf",
#endif
           (CONFIG_SENSORARRAY_FDC_PARALLEL_DUAL_BUS_READ && s_fdcWorkersAvailable) ? 1u : 0u,
           CONFIG_SENSORARRAY_FDC_INTB_ENABLE ? 1u : 0u);
}

static void sensorarrayMeasureUpdateFdcTimingAggregate(const sensorarrayFdcTimingSummary_t *summary,
                                                       uint32_t sequence)
{
    if (!summary || !CONFIG_SENSORARRAY_FDC_TIMING_SUMMARY_AGGREGATE) {
        return;
    }

    uint32_t period = CONFIG_SENSORARRAY_FDC_TIMING_SUMMARY_PERIOD_FRAMES;
    if (period == 0u) {
        return;
    }

    if (!s_fdcTimingAggregate.active || s_fdcTimingAggregate.frames == 0u) {
        s_fdcTimingAggregate = (sensorarrayFdcTimingAggregate_t){
            .active = true,
            .seqStart = sequence,
            .frameUsMin = UINT64_MAX,
        };
    }

    sensorarrayFdcTimingAggregate_t *agg = &s_fdcTimingAggregate;
    agg->frames++;
    agg->seqEnd = sequence;
    agg->frameUsTotal += summary->frameUs;
    if (summary->frameUs < agg->frameUsMin) {
        agg->frameUsMin = summary->frameUs;
    }
    if (summary->frameUs > agg->frameUsMax) {
        agg->frameUsMax = summary->frameUs;
    }
    agg->rowUsTotal += summary->rowAvgUs * SENSORARRAY_MATRIX_ROWS;
    if (summary->rowMaxUs > agg->rowUsMax) {
        agg->rowUsMax = summary->rowMaxUs;
        agg->slowRow = summary->slowRow;
    }
    if (summary->frameUs > SENSORARRAY_FDC_TARGET_FRAME_US) {
        agg->overrunCount++;
    }

#define SENSORARRAY_FDC_AGG_ADD(field_) agg->totals.field_ += summary->field_
    SENSORARRAY_FDC_AGG_ADD(cacheApplyUs);
    SENSORARRAY_FDC_AGG_ADD(sleepBeforeRowSwitchUs);
    SENSORARRAY_FDC_AGG_ADD(rowSwitchWhileSleepingUs);
    SENSORARRAY_FDC_AGG_ADD(rowSettleUs);
    SENSORARRAY_FDC_AGG_ADD(diffApplyWhileSleepingUs);
    SENSORARRAY_FDC_AGG_ADD(sleepTotalUs);
    SENSORARRAY_FDC_AGG_ADD(sleepExitToIntbUs);
    SENSORARRAY_FDC_AGG_ADD(statusReadUs);
    SENSORARRAY_FDC_AGG_ADD(dataReadUs);
    SENSORARRAY_FDC_AGG_ADD(primaryJobUs);
    SENSORARRAY_FDC_AGG_ADD(secondaryJobUs);
    SENSORARRAY_FDC_AGG_ADD(dualBusWaitUs);
    SENSORARRAY_FDC_AGG_ADD(dualBusSkewUs);
    SENSORARRAY_FDC_AGG_ADD(cacheApplyDiffWriteCount);
    SENSORARRAY_FDC_AGG_ADD(i2cWriteCount);
    SENSORARRAY_FDC_AGG_ADD(i2cReadCount);
    SENSORARRAY_FDC_AGG_ADD(i2cMeasuredUs);
    SENSORARRAY_FDC_AGG_ADD(i2cBus0TotalUs);
    SENSORARRAY_FDC_AGG_ADD(i2cBus1TotalUs);
    SENSORARRAY_FDC_AGG_ADD(freshAmplitudeWarningCount);
    SENSORARRAY_FDC_AGG_ADD(staleAmplitudeWarningCount);
    SENSORARRAY_FDC_AGG_ADD(transientAmplitudeWarningCount);
    SENSORARRAY_FDC_AGG_ADD(warningFastSweepRequestedCount);
    SENSORARRAY_FDC_AGG_ADD(intbFreshDrdyCount);
    SENSORARRAY_FDC_AGG_ADD(intbTimeoutCount);
    SENSORARRAY_FDC_AGG_ADD(intbFalseEdgeCount);
    SENSORARRAY_FDC_AGG_ADD(readyFullCount);
    SENSORARRAY_FDC_AGG_ADD(readyPartialCount);
    SENSORARRAY_FDC_AGG_ADD(readyNoneCount);
    SENSORARRAY_FDC_AGG_ADD(fallbackAttemptCount);
    SENSORARRAY_FDC_AGG_ADD(fallbackSuccessCount);
    SENSORARRAY_FDC_AGG_ADD(fallbackPartialCount);
    SENSORARRAY_FDC_AGG_ADD(fallbackFailCount);
    SENSORARRAY_FDC_AGG_ADD(rowFullInvalidCount);
    SENSORARRAY_FDC_AGG_ADD(deviceFullInvalidCount);
    SENSORARRAY_FDC_AGG_ADD(waitReadyUsPrimaryTotal);
    SENSORARRAY_FDC_AGG_ADD(waitReadyUsSecondaryTotal);
    SENSORARRAY_FDC_AGG_ADD(read4UsPrimaryTotal);
    SENSORARRAY_FDC_AGG_ADD(read4UsSecondaryTotal);
    SENSORARRAY_FDC_AGG_ADD(sweepRequestCount);
    SENSORARRAY_FDC_AGG_ADD(sweepActuallyQueuedCount);
#undef SENSORARRAY_FDC_AGG_ADD
    if (summary->maxWaitReadyUs > agg->totals.maxWaitReadyUs) {
        agg->totals.maxWaitReadyUs = summary->maxWaitReadyUs;
    }
    if (summary->maxI2cReadUs > agg->totals.maxI2cReadUs) {
        agg->totals.maxI2cReadUs = summary->maxI2cReadUs;
    }

    if (agg->frames >= period) {
        sensorarrayMeasurePrintFdcTimingAggregate(agg);
        *agg = (sensorarrayFdcTimingAggregate_t){0};
    }
}

static void sensorarrayMeasurePrintFdcBottleneck(const sensorarrayFdcTimingSummary_t *summary,
                                                 uint32_t sequence)
{
    if (!summary || summary->frameUs <= SENSORARRAY_FDC_TARGET_FRAME_US) {
        return;
    }

    struct BottleneckItem {
        const char *name;
        uint64_t us;
    } items[] = {
        {"cacheApplyUs", summary->cacheApplyUs},
        {"i2cMeasuredUs", summary->i2cMeasuredUs},
        {"waitReadyUs", summary->waitReadyUs},
        {"readUs", summary->readUs},
        {"rowMaxUs", summary->rowMaxUs},
        {"diffApplyWhileSleepingUs", summary->diffApplyWhileSleepingUs},
        {"sleepExitToIntbUs", summary->sleepExitToIntbUs},
    };

    for (size_t i = 0u; i < sizeof(items) / sizeof(items[0]); ++i) {
        for (size_t j = i + 1u; j < sizeof(items) / sizeof(items[0]); ++j) {
            if (items[j].us > items[i].us) {
                struct BottleneckItem tmp = items[i];
                items[i] = items[j];
                items[j] = tmp;
            }
        }
    }

    const char *hint =
        (summary->cacheApplyDiffWriteCount > 64u) ? "excessive_diff_writes" :
        (summary->intbTimeoutCount > 0u) ? "intb_timeout_or_polling_fallback" :
        (summary->i2cMeasuredUs > SENSORARRAY_FDC_TARGET_FRAME_US) ? "i2c_transaction_budget" :
        "row_epoch_timing";
    printf("SCAN_BOTTLENECK,seq=%lu,top1=%s:%llu,top2=%s:%llu,top3=%s:%llu,hint=%s\n",
           (unsigned long)sequence,
           items[0].name,
           (unsigned long long)items[0].us,
           items[1].name,
           (unsigned long long)items[1].us,
           items[2].name,
           (unsigned long long)items[2].us,
           hint);
}

static void sensorarrayMeasurePrintFdcRowTiming(uint32_t sequence,
                                                const sensorarrayFdcRowTiming_t *rowTiming)
{
    if (!rowTiming) {
        return;
    }

    uint64_t rowOverrunUs = (rowTiming->rowUs > SENSORARRAY_FDC_TARGET_ROW_US) ?
        (rowTiming->rowUs - SENSORARRAY_FDC_TARGET_ROW_US) :
        0u;
    uint64_t serialEquivalentUs = rowTiming->primaryTotalUs + rowTiming->secondaryTotalUs;
    uint64_t parallelActualUs = (rowTiming->primaryTotalUs > rowTiming->secondaryTotalUs) ?
        rowTiming->primaryTotalUs :
        rowTiming->secondaryTotalUs;
    parallelActualUs += rowTiming->parallelJoinWaitUs;
    uint64_t parallelEfficiencyPct = parallelActualUs ?
        ((serialEquivalentUs * 100ull) / parallelActualUs) :
        0u;

    printf("SCAN_ROW_TIMING,seq=%lu,row=%u,targetRowUs=%lu,rowUs=%llu,rowOverrun=%u,rowSelectUs=%llu,analogSettleUs=%llu,sleepBeforeRowSwitchUs=%llu,rowSwitchWhileSleepingUs=%llu,rowSettleUs=%llu,primaryTotalUs=%llu,secondaryTotalUs=%llu,parallelJoinWaitUs=%llu,dualBusWaitUs=%llu,dualBusSkewUs=%llu,discardUs=%llu,waitReadyUs=%llu,readUs=%llu,rowValidMask=0x%02X,rowWarnMask=0x%02X,rowErrorMask=0x%02X,parallelEnabled=%u,serialEquivalentUs=%llu,parallelActualUs=%llu,parallelEfficiencyPct=%llu\n",
           (unsigned long)sequence,
           (unsigned)rowTiming->row,
           (unsigned long)SENSORARRAY_FDC_TARGET_ROW_US,
           (unsigned long long)rowTiming->rowUs,
           rowOverrunUs ? 1u : 0u,
           (unsigned long long)rowTiming->rowSelectUs,
           (unsigned long long)rowTiming->analogSettleUs,
           (unsigned long long)rowTiming->sleepBeforeRowSwitchUs,
           (unsigned long long)rowTiming->rowSwitchWhileSleepingUs,
           (unsigned long long)rowTiming->rowSettleUs,
           (unsigned long long)rowTiming->primaryTotalUs,
           (unsigned long long)rowTiming->secondaryTotalUs,
           (unsigned long long)rowTiming->parallelJoinWaitUs,
           (unsigned long long)rowTiming->dualBusWaitUs,
           (unsigned long long)rowTiming->dualBusSkewUs,
           (unsigned long long)rowTiming->discardUs,
           (unsigned long long)rowTiming->waitReadyUs,
           (unsigned long long)rowTiming->readUs,
           (unsigned)rowTiming->rowValidMask,
           (unsigned)rowTiming->rowWarnMask,
           (unsigned)rowTiming->rowErrorMask,
           (CONFIG_SENSORARRAY_FDC_PARALLEL_DUAL_BUS_READ && s_fdcWorkersAvailable) ? 1u : 0u,
           (unsigned long long)serialEquivalentUs,
           (unsigned long long)parallelActualUs,
           (unsigned long long)parallelEfficiencyPct);
}

static void sensorarrayMeasurePrintFdcDeviceTiming(uint32_t sequence,
                                                   const sensorarrayFdcDeviceTiming_t *deviceTiming)
{
    if (!deviceTiming) {
        return;
    }

    printf("SCAN_DEVICE_TIMING,seq=%lu,row=%u,device=%s,deviceUs=%llu,sleepEnterUs=%llu,applyUs=%llu,sleepExitUs=%llu,sleepExitToIntbUs=%llu,statusReadUs=%llu,dataReadUs=%llu,applyBuildConfigUs=%llu,channelConfigWriteUs=%llu,globalConfigWriteUs=%llu,verifyUs=%llu,discardUs=%llu,waitReadyUs=%llu,readRawUs=%llu,cacheDiffWriteCount=%lu,cacheNoDiffCount=%lu,intbEdges=%lu,intbFalseEdges=%lu,intbTimeouts=%lu,readyPollCount=%lu,readyFullCount=%lu,readyPartialCount=%lu,readyNoneCount=%lu,fallbackAttemptCount=%lu,fallbackSuccessCount=%lu,fallbackPartialCount=%lu,fallbackFailCount=%lu,deviceFullInvalidCount=%lu,maxWaitReadyUs=%llu,maxI2cReadUs=%llu,regWriteCount=%lu,regReadCount=%lu,verifyReadCount=%lu,retryCount=%lu,nackCount=%lu,timeoutCount=%lu\n",
           (unsigned long)sequence,
           (unsigned)deviceTiming->row,
           sensorarrayMeasureFdcDeviceName(deviceTiming->deviceId),
           (unsigned long long)deviceTiming->deviceUs,
           (unsigned long long)deviceTiming->sleepEnterUs,
           (unsigned long long)deviceTiming->applyUs,
           (unsigned long long)deviceTiming->sleepExitUs,
           (unsigned long long)deviceTiming->sleepExitToIntbUs,
           (unsigned long long)deviceTiming->statusReadUs,
           (unsigned long long)deviceTiming->dataReadUs,
           (unsigned long long)deviceTiming->applyBuildConfigUs,
           (unsigned long long)deviceTiming->channelConfigWriteUs,
           (unsigned long long)deviceTiming->globalConfigWriteUs,
           (unsigned long long)deviceTiming->verifyUs,
           (unsigned long long)deviceTiming->discardUs,
           (unsigned long long)deviceTiming->waitReadyUs,
           (unsigned long long)deviceTiming->readRawUs,
           (unsigned long)deviceTiming->cacheDiffWriteCount,
           (unsigned long)deviceTiming->cacheNoDiffCount,
           (unsigned long)deviceTiming->intbEdgeCount,
           (unsigned long)deviceTiming->intbFalseEdgeCount,
           (unsigned long)deviceTiming->intbTimeoutCount,
           (unsigned long)deviceTiming->readyPollCount,
           (unsigned long)deviceTiming->readyFullCount,
           (unsigned long)deviceTiming->readyPartialCount,
           (unsigned long)deviceTiming->readyNoneCount,
           (unsigned long)deviceTiming->fallbackAttemptCount,
           (unsigned long)deviceTiming->fallbackSuccessCount,
           (unsigned long)deviceTiming->fallbackPartialCount,
           (unsigned long)deviceTiming->fallbackFailCount,
           (unsigned long)deviceTiming->deviceFullInvalidCount,
           (unsigned long long)deviceTiming->maxWaitReadyUs,
           (unsigned long long)deviceTiming->maxI2cReadUs,
           (unsigned long)deviceTiming->regWriteCount,
           (unsigned long)deviceTiming->regReadCount,
           (unsigned long)deviceTiming->verifyReadCount,
           (unsigned long)deviceTiming->retryCount,
           (unsigned long)deviceTiming->nackCount,
           (unsigned long)deviceTiming->timeoutCount);
}

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
    Fdc2214CapResetI2cStats(state->fdcSecondary.handle);
    BoardSupportI2cBusInfo_t primaryBus = {0};
    BoardSupportI2cBusInfo_t secondaryBus = {0};
    (void)boardSupportGetI2cBusInfo(false, &primaryBus);
    (void)boardSupportGetI2cBusInfo(true, &secondaryBus);
    bool primaryBusEnabled = primaryBus.Enabled;
    bool secondaryBusEnabled = secondaryBus.Enabled;
    bool sameBus = primaryBusEnabled &&
                   secondaryBusEnabled &&
                   primaryBus.Port == secondaryBus.Port;
    bool parallelConfigEnabled = CONFIG_SENSORARRAY_FDC_PARALLEL_DUAL_BUS_READ != 0;
    bool parallelEligible = parallelConfigEnabled &&
                            primaryBusEnabled &&
                            secondaryBusEnabled &&
                            !sameBus &&
                            (!s_fdcWorkersInitAttempted || s_fdcWorkersAvailable);

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

    sensorarrayFdcFrameHealth_t frameHealth = {0};
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
        Fdc2214CapGetI2cStats(state->fdcSecondary.handle, &secondaryStatsBefore);
        uint32_t primaryEdgeBefore = sensorarrayMeasureFdcWorkerEdgeCount(SENSORARRAY_FDC_DEV_PRIMARY);
        uint32_t secondaryEdgeBefore = sensorarrayMeasureFdcWorkerEdgeCount(SENSORARRAY_FDC_DEV_SECONDARY);

        int64_t rowStartUs = esp_timer_get_time();
        uint32_t epochId = ++s_fdcRowEpoch;
        sensorarrayFdcRuntimeChannelConfig_t runtimeConfigs[2][4] = {0};
        sensorarrayFdcAutoscanSamples_t primarySamples = {0};
        sensorarrayFdcAutoscanSamples_t secondarySamples = {0};

        stageStartUs = esp_timer_get_time();
        esp_err_t rowErr = ESP_ERR_NOT_SUPPORTED;
        if (parallelEligible) {
            rowErr = sensorarrayMeasureReadFdcMatrixRowParallelEpoch(state,
                                                                     s,
                                                                     epochId,
                                                                     &primarySamples,
                                                                     &secondarySamples,
                                                                     runtimeConfigs,
                                                                     &primaryTiming,
                                                                     &secondaryTiming,
                                                                     &rowTiming);
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
                parallelEligible = false;
            }
        }
        if (!parallelEligible || rowErr != ESP_OK) {
            rowErr = sensorarrayMeasureReadFdcMatrixRowSerialEpoch(state,
                                                                   s,
                                                                   epochId,
                                                                   &primarySamples,
                                                                   &secondarySamples,
                                                                   runtimeConfigs,
                                                                   &primaryTiming,
                                                                   &secondaryTiming,
                                                                   &rowTiming);
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
        Fdc2214CapGetI2cStats(state->fdcSecondary.handle, &secondaryStatsAfter);
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
    sensorarrayMeasureUpdateFdcRuntimeProfiles(state, &frameHealth);
    sensorarrayMeasureCountFdcFrameWarnings(&frameHealth, &timing);

    timing.capComputeUs = sensorarrayMeasureComputeFdcFrameCapTotalPf(outFrame);
    timing.frameUs = sensorarrayMeasureElapsedUs(frameStartUs);
    timing.rowAvgUs = rowTotalUs / SENSORARRAY_MATRIX_ROWS;
    if (timing.rowMinUs == UINT64_MAX) {
        timing.rowMinUs = 0u;
    }

    Fdc2214CapI2cStats_t primaryStats = {0};
    Fdc2214CapI2cStats_t secondaryStats = {0};
    Fdc2214CapGetI2cStats(state->fdcPrimary.handle, &primaryStats);
    Fdc2214CapGetI2cStats(state->fdcSecondary.handle, &secondaryStats);
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
        uint32_t seq = s_fdcMatrixAllInvalidSequence++;
        const char *reason = allZero ? "all_zero_raw" :
            (outFrame->freshCount == 0u ? "all_invalid_no_fresh_data" :
             "normal_path_no_valid_after_boot_ok");
        if (allZero) {
            outFrame->errorMask = UINT64_MAX;
        }
        printf("MATRIXFDC_DIAG,stage=%s,seq=%lu,errorMask=0x%016llX,reason=%s,freshCount=%u,validCount=%u,hardwareZeroRawCount=%u,placeholderZeroCount=%u,firstBadRow=%u,firstBadDevice=%u,firstBadStatus=0x%04X,firstBadUnread=0x%X\n",
               allZero ? "all_invalid" : "all_status_invalid",
               (unsigned long)seq,
               (unsigned long long)outFrame->errorMask,
               reason,
               (unsigned)outFrame->freshCount,
               (unsigned)outFrame->validCount,
               (unsigned)outFrame->hardwareZeroRawCount,
               (unsigned)outFrame->placeholderZeroCount,
               (unsigned)outFrame->firstBadRow,
               (unsigned)outFrame->firstBadDevice,
               outFrame->firstBadStatus,
               (unsigned)outFrame->firstBadUnread);
        sensorarrayFdcSweepReportAllInvalidFrame(outFrame->validMask,
                                                 outFrame->errorMask,
                                                 allZero ? outFrame->hardwareZeroRawCount : 0u);
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

static esp_err_t sensorarrayTransportSendFdcMatrixFrame(const sensorarrayFdcMatrixFrame_t *frame)
{
    (void)frame;
    return ESP_ERR_NOT_SUPPORTED;
}

static void sensorarrayFdcMatrixPrintFrame(const sensorarrayFdcMatrixFrame_t *frame, const char *tag)
{
#if !CONFIG_SENSORARRAY_FDC_TEXT_OUTPUT_BOTH_INLINE_DEBUG
    (void)tag;
#endif
#if CONFIG_SENSORARRAY_FDC_TEXT_OUTPUT_CAP_TOTAL_PF || CONFIG_SENSORARRAY_FDC_TEXT_OUTPUT_BOTH_SEPARATE
    printf("MATRIXFDC_CAP,seq=%lu,timestampUs=%llu,capValidMask=0x%016llX,warnMask=0x%016llX,errorMask=0x%016llX,capTotalPf=[",
           (unsigned long)frame->sequence,
           (unsigned long long)frame->timestampUs,
           (unsigned long long)frame->capValidMask,
           (unsigned long long)frame->warnMask,
           (unsigned long long)frame->errorMask);
    for (size_t i = 0u; i < SENSORARRAY_MATRIX_CELL_COUNT; ++i) {
        printf("%s%.6f", (i == 0u) ? "" : ",", frame->capTotalPf[i]);
    }
    printf("]\n");
#endif

#if CONFIG_SENSORARRAY_FDC_TEXT_OUTPUT_FREQ_HZ || CONFIG_SENSORARRAY_FDC_TEXT_OUTPUT_BOTH_SEPARATE
    printf("MATRIXFDC_FREQ,seq=%lu,timestampUs=%llu,validMask=0x%016llX,warnMask=0x%016llX,errorMask=0x%016llX,freqHz=[",
           (unsigned long)frame->sequence,
           (unsigned long long)frame->timestampUs,
           (unsigned long long)frame->validMask,
           (unsigned long long)frame->warnMask,
           (unsigned long long)frame->errorMask);
    for (size_t i = 0u; i < SENSORARRAY_MATRIX_CELL_COUNT; ++i) {
        printf("%s%.1f", (i == 0u) ? "" : ",", frame->freqHz[i]);
    }
    printf("]\n");
#endif

#if CONFIG_SENSORARRAY_FDC_TEXT_OUTPUT_BOTH_INLINE_DEBUG
    printf("%s_INLINE_DEBUG,seq=%lu,timestampUs=%llu,validMask=0x%016llX,warnMask=0x%016llX,errorMask=0x%016llX,capValidMask=0x%016llX,freqHz=[",
           tag ? tag : "MATRIXFDC",
           (unsigned long)frame->sequence,
           (unsigned long long)frame->timestampUs,
           (unsigned long long)frame->validMask,
           (unsigned long long)frame->warnMask,
           (unsigned long long)frame->errorMask,
           (unsigned long long)frame->capValidMask);
    for (size_t i = 0u; i < SENSORARRAY_MATRIX_CELL_COUNT; ++i) {
        printf("%s%.1f", (i == 0u) ? "" : ",", frame->freqHz[i]);
    }
    printf("],capTotalPf=[");
    for (size_t i = 0u; i < SENSORARRAY_MATRIX_CELL_COUNT; ++i) {
        printf("%s%.6f", (i == 0u) ? "" : ",", frame->capTotalPf[i]);
    }
    printf("]\n");
#endif

#if CONFIG_SENSORARRAY_FDC_RAW_DEBUG_LOG
    printf("DEBUGFDC_RAW,seq=%lu,timestampUs=%llu,raw28=[",
           (unsigned long)frame->sequence,
           (unsigned long long)frame->timestampUs);
    for (size_t i = 0u; i < SENSORARRAY_MATRIX_CELL_COUNT; ++i) {
        printf("%s%lu", (i == 0u) ? "" : ",", (unsigned long)frame->raw28[i]);
    }
    printf("]\n");
#endif
}

esp_err_t sensorarrayFdcMatrixEmitFrame(const sensorarrayFdcMatrixFrame_t *frame)
{
    if (!frame) {
        return ESP_ERR_INVALID_ARG;
    }

    if (sensorarrayFastSpeedIsEnabled()) {
        return sensorarrayTransportSendFdcMatrixFrame(frame);
    }

    sensorarrayFdcMatrixPrintFrame(frame, "MATRIXFDC");
    return ESP_OK;
}

esp_err_t sensorarrayMeasureSetSelaPath(sensorarrayState_t *state,
                                        sensorarraySelaRoute_t selaRoute,
                                        uint32_t settleDelayMs,
                                        const char *stage,
                                        const char *label)
{
    if (!state || !state->tmuxReady) {
        return ESP_ERR_INVALID_STATE;
    }

    int selaWriteLevel = 0;
    if (!sensorarrayBoardMapSelaRouteToGpioLevel(selaRoute, &selaWriteLevel)) {
        return ESP_ERR_INVALID_ARG;
    }

    return sensorarrayMeasureWriteSela(state, selaRoute, settleDelayMs, stage, label);
}

esp_err_t sensorarrayMeasureApplyRouteLevels(sensorarrayState_t *state,
                                             uint8_t sColumn,
                                             uint8_t dLine,
                                             sensorarrayRoutePathKind_t path,
                                             tmux1108Source_t swSource,
                                             sensorarraySelaRoute_t selaRoute,
                                             bool selBLevel,
                                             uint32_t delayAfterRowMs,
                                             uint32_t delayAfterSelAMs,
                                             uint32_t delayAfterSelBMs,
                                             uint32_t delayAfterSwMs,
                                             const char *label)
{
    if (!state || !state->tmuxReady) {
        return ESP_ERR_INVALID_STATE;
    }
    if (sColumn < 1u || sColumn > 8u || dLine < 1u || dLine > 8u) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!sensorarrayBoardMapSelaRouteToGpioLevel(selaRoute, &(int){0})) {
        return ESP_ERR_INVALID_ARG;
    }

    const bool adsStopNeeded = state->adsReady && state->adsAdc1Running;
    esp_err_t err = sensorarrayMeasureStopAdsBeforeRoute(state);
    sensorarrayLogRouteStep("ads_stop",
                            label,
                            sColumn,
                            dLine,
                            path,
                            swSource,
                            selaRoute,
                            selBLevel,
                            err,
                            adsStopNeeded ? ((err == ESP_OK) ? "stop_ads_before_route" : "stop_ads_error")
                                          : "ads_already_stopped");
    if (err != ESP_OK) {
        return err;
    }

    if (swSource == TMUX1108_SOURCE_REF) {
        err = sensorarrayMeasureSetSwForRoute(state,
                                              "sw_pre_ref_guard",
                                              sColumn,
                                              dLine,
                                              path,
                                              TMUX1108_SOURCE_GND,
                                              selaRoute,
                                              selBLevel,
                                              label,
                                              "set_sw_pre_ref_guard",
                                              "isolate_ref_before_route");
        if (err != ESP_OK) {
            return err;
        }
        err = sensorarrayMeasureApplyRefPolicy(state,
                                               "enter_ref_mode",
                                               "route_apply_levels",
                                               SENSORARRAY_MATRIX_D_SOURCE_REF,
                                               SENSORARRAY_ADS_INTREF_ON,
                                               SENSORARRAY_ADS_VBIAS_ON,
                                               "ref_d_before_connect");
        if (err != ESP_OK) {
            return err;
        }
    } else {
        err = sensorarrayMeasureSetSwForRoute(state,
                                              "sw_pre_ground",
                                              sColumn,
                                              dLine,
                                              path,
                                              swSource,
                                              selaRoute,
                                              selBLevel,
                                              label,
                                              "set_sw_pre_ground",
                                              "ground_d_before_route");
        if (err != ESP_OK) {
            return err;
        }
        if (path == SENSORARRAY_ROUTE_PATH_CAPACITIVE) {
            sensorarrayAdsIntRefPolicy_t intrefPolicy =
                state->adsReady ? SENSORARRAY_ADS_INTREF_OFF : SENSORARRAY_ADS_INTREF_KEEP;
            sensorarrayAdsVbiasPolicy_t vbiasPolicy =
                state->adsReady ? SENSORARRAY_ADS_VBIAS_OFF : SENSORARRAY_ADS_VBIAS_KEEP;
            err = sensorarrayMeasureApplyRefPolicy(state,
                                                   "cap_fdc_ref_off",
                                                   "route_apply_levels",
                                                   SENSORARRAY_MATRIX_D_SOURCE_GND,
                                                   intrefPolicy,
                                                   vbiasPolicy,
                                                   "fdc_cap_path_no_ads_ref");
            if (err != ESP_OK) {
                return err;
            }
        }
    }

    err = tmuxSwitchSelectRow((uint8_t)(sColumn - 1u));
    sensorarrayLogRouteStep("row", label, sColumn, dLine, path, swSource, selaRoute, selBLevel, err, "set_row");
    if (err != ESP_OK) {
        return err;
    }
    sensorarrayDelayMs(delayAfterRowMs);

    err = sensorarrayMeasureSetSelaPath(state, selaRoute, delayAfterSelAMs, "selA", label);
    sensorarrayLogRouteStep("selA",
                            label,
                            sColumn,
                            dLine,
                            path,
                            swSource,
                            selaRoute,
                            selBLevel,
                            err,
                            "set_sela_path");
    if (err != ESP_OK) {
        return err;
    }

    err = tmux1134SelectSelBLevel(selBLevel);
    sensorarrayLogRouteStep("selB", label, sColumn, dLine, path, swSource, selaRoute, selBLevel, err, "set_selB");
    if (err != ESP_OK) {
        return err;
    }
    sensorarrayDelayMs(delayAfterSelBMs);

    if (swSource == TMUX1108_SOURCE_GND) {
        err = sensorarrayMeasureSetSwForRoute(state,
                                              "sw_final_assert",
                                              sColumn,
                                              dLine,
                                              path,
                                              swSource,
                                              selaRoute,
                                              selBLevel,
                                              label,
                                              "ok",
                                              "final_ground_assert");
    } else {
        err = sensorarrayMeasureSetSwForRoute(state,
                                              "sw",
                                              sColumn,
                                              dLine,
                                              path,
                                              swSource,
                                              selaRoute,
                                              selBLevel,
                                              label,
                                              "set_sw",
                                              "connect_ref_after_route");
    }
    if (err != ESP_OK) {
        return err;
    }
    sensorarrayDelayMs(delayAfterSwMs);

    sensorarrayLogDbgExtraCaptureCtrl();
    return ESP_OK;
}

esp_err_t sensorarrayMeasureApplyRoute(sensorarrayState_t *state,
                                       uint8_t sColumn,
                                       uint8_t dLine,
                                       sensorarrayPath_t path,
                                       tmux1108Source_t swSource,
                                       const char **outMapLabel)
{
    if (outMapLabel) {
        *outMapLabel = SENSORARRAY_NA;
    }
    if (!state || !state->tmuxReady) {
        return ESP_ERR_INVALID_STATE;
    }
    if (sColumn < 1u || sColumn > 8u || dLine < 1u || dLine > 8u) {
        return ESP_ERR_INVALID_ARG;
    }

    const sensorarrayRouteMap_t *routeMap = sensorarrayBoardMapFindRoute(sColumn, dLine, path);
    if (!routeMap) {
        return ESP_ERR_NOT_SUPPORTED;
    }

    sensorarrayRoutePathKind_t routePath = sensorarrayBoardMapPathToRoutePath(path, swSource);
    if (!sensorarrayBoardMapSelaRouteToGpioLevel(routeMap->selaRoute, &(int){0})) {
        return ESP_ERR_INVALID_STATE;
    }

    printf("ROUTEDEBUG,stage=apply_begin,label=%s,sColumn=%u,dLine=%u,path=%s,sw=%s,selaRequest=%s,"
           "selBLevel=%u,note=route_verify_only_confirms_gpio_control_state_not_analog_conduction\n",
           routeMap->mapLabel ? routeMap->mapLabel : SENSORARRAY_NA,
           (unsigned)sColumn,
           (unsigned)dLine,
           sensorarrayLogRoutePathName(routePath),
           sensorarrayLogSwSourceName(swSource),
           sensorarrayBoardMapSelaRouteName(routeMap->selaRoute),
           routeMap->selBLevel ? 1u : 0u);

    const bool adsStopNeeded = state->adsReady && state->adsAdc1Running;
    esp_err_t err = sensorarrayMeasureStopAdsBeforeRoute(state);
    sensorarrayLogRouteStep("ads_stop",
                            routeMap->mapLabel,
                            sColumn,
                            dLine,
                                           routePath,
                            swSource,
                            routeMap->selaRoute,
                            routeMap->selBLevel,
                            err,
                            adsStopNeeded ? ((err == ESP_OK) ? "stop_ads_before_route" : "stop_ads_error")
                                          : "ads_already_stopped");
    if (err != ESP_OK) {
        return err;
    }

    if (swSource == TMUX1108_SOURCE_REF) {
        err = sensorarrayMeasureSetSwForRoute(state,
                                              "sw_pre_ref_guard",
                                              sColumn,
                                              dLine,
                                              routePath,
                                              TMUX1108_SOURCE_GND,
                                              routeMap->selaRoute,
                                              routeMap->selBLevel,
                                              routeMap->mapLabel,
                                              "set_sw_pre_ref_guard",
                                              "isolate_ref_before_route");
        if (err != ESP_OK) {
            return err;
        }
        err = sensorarrayMeasureApplyRefPolicy(state,
                                               "enter_ref_mode",
                                               "route_apply",
                                               SENSORARRAY_MATRIX_D_SOURCE_REF,
                                               SENSORARRAY_ADS_INTREF_ON,
                                               SENSORARRAY_ADS_VBIAS_ON,
                                               "ref_d_before_connect");
        if (err != ESP_OK) {
            return err;
        }
    } else {
        err = sensorarrayMeasureSetSwForRoute(state,
                                              "sw_pre_ground",
                                              sColumn,
                                              dLine,
                                              routePath,
                                              swSource,
                                              routeMap->selaRoute,
                                              routeMap->selBLevel,
                                              routeMap->mapLabel,
                                              "set_sw_pre_ground",
                                              "ground_d_before_route");
        if (err != ESP_OK) {
            return err;
        }
        if (path == SENSORARRAY_PATH_CAPACITIVE) {
            sensorarrayAdsIntRefPolicy_t intrefPolicy =
                state->adsReady ? SENSORARRAY_ADS_INTREF_OFF : SENSORARRAY_ADS_INTREF_KEEP;
            sensorarrayAdsVbiasPolicy_t vbiasPolicy =
                state->adsReady ? SENSORARRAY_ADS_VBIAS_OFF : SENSORARRAY_ADS_VBIAS_KEEP;
            err = sensorarrayMeasureApplyRefPolicy(state,
                                                   "cap_fdc_ref_off",
                                                   "route_apply",
                                                   SENSORARRAY_MATRIX_D_SOURCE_GND,
                                                   intrefPolicy,
                                                   vbiasPolicy,
                                                   "fdc_cap_path_no_ads_ref");
            if (err != ESP_OK) {
                return err;
            }
        }
    }

    err = tmuxSwitchSelectRow((uint8_t)(sColumn - 1u));
    sensorarrayLogRouteStep("row",
                            routeMap->mapLabel,
                            sColumn,
                            dLine,
                            routePath,
                            swSource,
                            routeMap->selaRoute,
                            routeMap->selBLevel,
                            err,
                            "set_row");
    if (err != ESP_OK) {
        return err;
    }
    sensorarrayDelayMs(SENSORARRAY_SETTLE_AFTER_COLUMN_MS);

    err = sensorarrayMeasureSetSelaPath(state,
                                        routeMap->selaRoute,
                                        SENSORARRAY_SETTLE_AFTER_PATH_MS,
                                        "selA",
                                        routeMap->mapLabel);
    sensorarrayLogRouteStep("selA",
                            routeMap->mapLabel,
                            sColumn,
                            dLine,
                            routePath,
                            swSource,
                            routeMap->selaRoute,
                            routeMap->selBLevel,
                            err,
                            "set_sela_path");
    if (err != ESP_OK) {
        return err;
    }

    err = tmux1134SelectSelBLevel(routeMap->selBLevel);
    sensorarrayLogRouteStep("selB",
                            routeMap->mapLabel,
                            sColumn,
                            dLine,
                            routePath,
                            swSource,
                            routeMap->selaRoute,
                            routeMap->selBLevel,
                            err,
                            "set_selB");
    if (err != ESP_OK) {
        return err;
    }
    sensorarrayDelayMs(SENSORARRAY_SETTLE_AFTER_PATH_MS);

    if (swSource == TMUX1108_SOURCE_GND) {
        err = sensorarrayMeasureSetSwForRoute(state,
                                              "sw_final_assert",
                                              sColumn,
                                              dLine,
                                              routePath,
                                              swSource,
                                              routeMap->selaRoute,
                                              routeMap->selBLevel,
                                              routeMap->mapLabel,
                                              "ok",
                                              "final_ground_assert");
    } else {
        err = sensorarrayMeasureSetSwForRoute(state,
                                              "sw",
                                              sColumn,
                                              dLine,
                                              routePath,
                                              swSource,
                                              routeMap->selaRoute,
                                              routeMap->selBLevel,
                                              routeMap->mapLabel,
                                              "set_sw",
                                              "connect_ref_after_route");
    }
    if (err != ESP_OK) {
        return err;
    }
    sensorarrayDelayMs(SENSORARRAY_SETTLE_AFTER_SW_MS);

    sensorarrayLogDbgExtraCaptureCtrl();

    if (outMapLabel) {
        *outMapLabel = routeMap->mapLabel;
    }
    printf("ROUTEDEBUG,stage=apply_done,label=%s,sColumn=%u,dLine=%u,path=%s,selaRequest=%s,"
           "selBLevel=%u,err=0,status=ok,note=route_verify_only_confirms_gpio_control_state_not_analog_conduction\n",
           routeMap->mapLabel ? routeMap->mapLabel : SENSORARRAY_NA,
           (unsigned)sColumn,
           (unsigned)dLine,
           sensorarrayLogRoutePathName(routePath),
           sensorarrayBoardMapSelaRouteName(routeMap->selaRoute),
           routeMap->selBLevel ? 1u : 0u);
    return ESP_OK;
}

esp_err_t sensorarrayMeasureApplyFdcCellRoute(sensorarrayState_t *state,
                                              const sensorarrayFdcCellTarget_t *target,
                                              const char *reason)
{
    if (!state || !target ||
        !sensorarrayMatrixIndexIsValid(target->sColumn, target->dLine) ||
        target->fdcChannel > (uint8_t)FDC2214_CH3) {
        return ESP_ERR_INVALID_ARG;
    }

    bool selBLevel = false;
    if (!sensorarrayBoardMapFdcSelBLevel(&selBLevel)) {
        return ESP_ERR_INVALID_STATE;
    }

    const char *source = reason ? reason : SENSORARRAY_NA;
    printf("FDC_CELL_ROUTE,stage=begin,s=%u,d=%u,index=%u,device=%s,ch=%u,reason=%s\n",
           (unsigned)target->sColumn,
           (unsigned)target->dLine,
           (unsigned)target->matrixIndex,
           sensorarrayMeasureFdcDeviceName(target->devId),
           (unsigned)target->fdcChannel,
           source);

    uint32_t settleMs = ((uint32_t)CONFIG_SENSORARRAY_FDC_MATRIX_SETTLE_US + 999u) / 1000u;
    esp_err_t err = sensorarrayMeasureApplyRouteLevels(state,
                                                       target->sColumn,
                                                       target->dLine,
                                                       SENSORARRAY_ROUTE_PATH_CAPACITIVE,
                                                       TMUX1108_SOURCE_GND,
                                                       SENSORARRAY_SELA_ROUTE_FDC2214,
                                                       selBLevel,
                                                       settleMs,
                                                       SENSORARRAY_SETTLE_AFTER_PATH_MS,
                                                       SENSORARRAY_SETTLE_AFTER_PATH_MS,
                                                       SENSORARRAY_SETTLE_AFTER_SW_MS,
                                                       target->mapLabel ? target->mapLabel : "fdc_cell_route");
    if (err == ESP_OK) {
        uint8_t discardCount = (uint8_t)CONFIG_SENSORARRAY_FDC_MATRIX_DISCARD_SAMPLES;
        if (discardCount < SENSORARRAY_FDC_CELL_ROUTE_DISCARD_COUNT) {
            discardCount = SENSORARRAY_FDC_CELL_ROUTE_DISCARD_COUNT;
        }
        err = sensorarrayMeasureFdcDiscardStaleSamples(state, target, discardCount, "row_switch");
    }

    printf("FDC_CELL_ROUTE,stage=done,s=%u,d=%u,index=%u,device=%s,ch=%u,err=0x%lx,status=%s\n",
           (unsigned)target->sColumn,
           (unsigned)target->dLine,
           (unsigned)target->matrixIndex,
           sensorarrayMeasureFdcDeviceName(target->devId),
           (unsigned)target->fdcChannel,
           (unsigned long)err,
           (err == ESP_OK) ? "ok" : "failed");
    return err;
}

esp_err_t sensorarrayMeasureReadAdsRawWithRetry(sensorarrayState_t *state,
                                                int32_t *outRaw,
                                                uint8_t retryCount,
                                                bool *outTimedOut,
                                                uint8_t *outStatusByte)
{
    if (!state || !outRaw) {
        return ESP_ERR_INVALID_ARG;
    }
    if (outTimedOut) {
        *outTimedOut = false;
    }
    if (outStatusByte) {
        *outStatusByte = 0u;
    }

    for (uint8_t attempt = 0; attempt <= retryCount; ++attempt) {
        esp_err_t err = ads126xAdcReadAdc1Raw(&state->ads, outRaw, outStatusByte);
        if (err == ESP_OK) {
            return ESP_OK;
        }

        if (err == ESP_ERR_TIMEOUT && outTimedOut) {
            *outTimedOut = true;
        }

        printf("DBGADSRETRY,attempt=%u,maxRetry=%u,err=%ld\n",
               (unsigned)attempt,
               (unsigned)retryCount,
               (long)err);

        if (attempt == retryCount) {
            return err;
        }
        sensorarrayDelayMs(1u);
    }

    return ESP_FAIL;
}

esp_err_t sensorarrayMeasureReadAdsPairUv(sensorarrayState_t *state,
                                          const sensorarrayAdsReadPolicy_t *policy,
                                          uint8_t muxp,
                                          uint8_t muxn,
                                          bool discardFirst,
                                          int32_t *outRaw,
                                          int32_t *outUv,
                                          uint8_t *outStatusByte)
{
    if (!state || !outRaw || !outUv || !state->adsReady) {
        return ESP_ERR_INVALID_STATE;
    }

    const sensorarrayAdsReadPolicy_t *readPolicy = sensorarrayReadPolicyOrDefault(policy);

    muxp &= 0x0Fu;
    muxn &= 0x0Fu;

    sensorarrayLogDbgExtraSetMux(muxp, muxn);
    if (state->adsRefMuxValid) {
        sensorarrayLogDbgExtraSetRefMux(state->adsRefMux);
    }

    uint16_t totalDiscard = (uint16_t)readPolicy->baseDiscardCount + (discardFirst ? 1u : 0u);
    if (totalDiscard > 255u) {
        totalDiscard = 255u;
    }
    sensorarrayLogDbgExtraSetDiscardCount((uint8_t)totalDiscard);

    printf("DBGADSSEQ,step=set_input_mux,muxp=%u(%s),muxn=%u(%s),discardFirst=%u,discardCount=%u\n",
           (unsigned)muxp,
           sensorarrayLogAdsMuxName(muxp),
           (unsigned)muxn,
           sensorarrayLogAdsMuxName(muxn),
           discardFirst ? 1u : 0u,
           (unsigned)totalDiscard);

    if (readPolicy->stopBeforeMuxChange && state->adsAdc1Running) {
        esp_err_t stopErr = ads126xAdcStopAdc1(&state->ads);
        if (stopErr == ESP_OK) {
            state->adsAdc1Running = false;
        } else {
            printf("DBGADSSEQ,step=stop1,err=%ld,status=stop_error\n", (long)stopErr);
            return stopErr;
        }
    }

    printf("DBGADSSEQ,step=read,muxp=%u(%s),muxn=%u(%s),discardFirst=%u,discardCount=%u\n",
           (unsigned)muxp,
           sensorarrayLogAdsMuxName(muxp),
           (unsigned)muxn,
           sensorarrayLogAdsMuxName(muxn),
           discardFirst ? 1u : 0u,
           (unsigned)totalDiscard);

    const bool startEachRead = readPolicy->startEveryRead || !state->adsAdc1Running;
    bool readTimedOut = false;
    uint8_t statusByte = 0u;
    esp_err_t err = ESP_FAIL;
    for (uint8_t attempt = 0u; attempt <= readPolicy->readRetryCount; ++attempt) {
        err = ads126xAdcReadSingleDiffUv(&state->ads,
                                         muxp,
                                         muxn,
                                         startEachRead,
                                         readPolicy->settleAfterMuxMs,
                                         (uint8_t)totalDiscard,
                                         outRaw,
                                         outUv,
                                         &statusByte);
        if (err == ESP_OK) {
            state->adsAdc1Running = true;
            break;
        }
        readTimedOut = (err == ESP_ERR_TIMEOUT);
        printf("DBGADSRETRY,attempt=%u,maxRetry=%u,muxp=%u,muxn=%u,err=%ld\n",
               (unsigned)attempt,
               (unsigned)readPolicy->readRetryCount,
               (unsigned)muxp,
               (unsigned)muxn,
               (long)err);
        if (attempt < readPolicy->readRetryCount) {
            sensorarrayDelayMs(1u);
        }
    }

    if (err != ESP_OK) {
        printf("DBGADSREAD,status=error,muxp=%u(%s),muxn=%u(%s),refmux=0x%02X,discardCount=%u,drdyTimeout=%u,err=%ld\n",
               (unsigned)muxp,
               sensorarrayLogAdsMuxName(muxp),
               (unsigned)muxn,
               sensorarrayLogAdsMuxName(muxn),
               state->adsRefMuxValid ? state->adsRefMux : 0u,
               (unsigned)totalDiscard,
               readTimedOut ? 1u : 0u,
               (long)err);
        return err;
    }

    if (outStatusByte) {
        *outStatusByte = statusByte;
    }

    printf("DBGADSSEQ,step=read_done,muxp=%u(%s),muxn=%u(%s),raw=%ld,uv=%ld,statusByte=0x%02X,discardFirst=%u,"
           "discardCount=%u\n",
           (unsigned)muxp,
           sensorarrayLogAdsMuxName(muxp),
           (unsigned)muxn,
           sensorarrayLogAdsMuxName(muxn),
           (long)*outRaw,
           (long)*outUv,
           statusByte,
           discardFirst ? 1u : 0u,
           (unsigned)totalDiscard);
    sensorarrayLogDbgExtraCaptureCtrl();
    return ESP_OK;
}

esp_err_t sensorarrayMeasureReadAdsUv(sensorarrayState_t *state,
                                      const sensorarrayAdsReadPolicy_t *policy,
                                      uint8_t dLine,
                                      bool discardFirst,
                                      int32_t *outRaw,
                                      int32_t *outUv)
{
    if (!state || !outRaw || !outUv || !state->adsReady) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t muxp = 0;
    uint8_t muxn = 0;
    if (!sensorarrayBoardMapAdsMuxForDLine(dLine, &muxp, &muxn)) {
        return ESP_ERR_INVALID_ARG;
    }

    return sensorarrayMeasureReadAdsPairUv(state, policy, muxp, muxn, discardFirst, outRaw, outUv, NULL);
}

sensorarrayResConvertResult_t sensorarrayMeasureTryResistanceMohm(int32_t uv, int32_t *outMohm)
{
    if (!outMohm) {
        return SENSORARRAY_RES_CONVERT_MODEL_INVALID;
    }
    if (uv < 0) {
        return SENSORARRAY_RES_CONVERT_SIGNED_INPUT;
    }
    if (uv == 0 || (uint32_t)uv >= SENSORARRAY_RESIST_EXCITATION_UV) {
        return SENSORARRAY_RES_CONVERT_MODEL_INVALID;
    }

    int64_t numerator = (int64_t)SENSORARRAY_RESIST_REF_OHMS * 1000 * (int64_t)uv;
    int64_t denominator = (int64_t)SENSORARRAY_RESIST_EXCITATION_UV - (int64_t)uv;
    if (denominator == 0) {
        return SENSORARRAY_RES_CONVERT_MODEL_INVALID;
    }

    *outMohm = (int32_t)(numerator / denominator);
    return SENSORARRAY_RES_CONVERT_OK;
}

const char *sensorarrayMeasureDividerModelStatus(int32_t uv, int32_t *outMohm, bool *outHaveMohm)
{
    if (outHaveMohm) {
        *outHaveMohm = false;
    }

    sensorarrayResConvertResult_t resResult = sensorarrayMeasureTryResistanceMohm(uv, outMohm);
    if (resResult == SENSORARRAY_RES_CONVERT_OK) {
        if (outHaveMohm) {
            *outHaveMohm = true;
        }
        return "divider_model_ok";
    }
    if (resResult == SENSORARRAY_RES_CONVERT_SIGNED_INPUT) {
        return "negative_uv";
    }
    return "divider_model_invalid";
}

static sensorarrayFdcSampleStatus_t sensorarrayMeasureMapFdcStatus(Fdc2214CapSampleStatus_t sampleStatus)
{
    switch (sampleStatus) {
    case FDC2214_SAMPLE_STATUS_SAMPLE_VALID:
        return SENSORARRAY_FDC_SAMPLE_STATUS_SAMPLE_VALID;
    case FDC2214_SAMPLE_STATUS_STILL_SLEEPING:
        return SENSORARRAY_FDC_SAMPLE_STATUS_STILL_SLEEPING;
    case FDC2214_SAMPLE_STATUS_I2C_READ_OK_BUT_NOT_CONVERTING:
        return SENSORARRAY_FDC_SAMPLE_STATUS_I2C_READ_OK_BUT_NOT_CONVERTING;
    case FDC2214_SAMPLE_STATUS_NO_UNREAD_CONVERSION:
        return SENSORARRAY_FDC_SAMPLE_STATUS_NO_UNREAD_CONVERSION;
    case FDC2214_SAMPLE_STATUS_ZERO_RAW_INVALID:
        return SENSORARRAY_FDC_SAMPLE_STATUS_ZERO_RAW_INVALID;
    case FDC2214_SAMPLE_STATUS_WATCHDOG_FAULT:
        return SENSORARRAY_FDC_SAMPLE_STATUS_WATCHDOG_FAULT;
    case FDC2214_SAMPLE_STATUS_AMPLITUDE_FAULT:
        return SENSORARRAY_FDC_SAMPLE_STATUS_AMPLITUDE_FAULT;
    case FDC2214_SAMPLE_STATUS_CONFIG_UNKNOWN:
    default:
        return SENSORARRAY_FDC_SAMPLE_STATUS_CONFIG_UNKNOWN;
    }
}

const char *sensorarrayMeasureFdcSampleStatusName(sensorarrayFdcSampleStatus_t status)
{
    switch (status) {
    case SENSORARRAY_FDC_SAMPLE_STATUS_I2C_READ_ERROR:
        return "i2c_read_error";
    case SENSORARRAY_FDC_SAMPLE_STATUS_CONFIG_UNKNOWN:
        return "config_unknown";
    case SENSORARRAY_FDC_SAMPLE_STATUS_STILL_SLEEPING:
        return "still_sleeping";
    case SENSORARRAY_FDC_SAMPLE_STATUS_I2C_READ_OK_BUT_NOT_CONVERTING:
        return "i2c_read_ok_but_not_converting";
    case SENSORARRAY_FDC_SAMPLE_STATUS_NO_UNREAD_CONVERSION:
        return "no_unread_conversion";
    case SENSORARRAY_FDC_SAMPLE_STATUS_ZERO_RAW_INVALID:
        return "zero_raw_invalid";
    case SENSORARRAY_FDC_SAMPLE_STATUS_WATCHDOG_FAULT:
        return "watchdog_fault";
    case SENSORARRAY_FDC_SAMPLE_STATUS_AMPLITUDE_FAULT:
        return "amplitude_fault";
    case SENSORARRAY_FDC_SAMPLE_STATUS_SAMPLE_VALID:
        return "sample_valid";
    default:
        return "config_unknown";
    }
}

typedef esp_err_t (*sensorarrayFdcReadSampleFn_t)(Fdc2214CapDevice_t *dev,
                                                   Fdc2214CapChannel_t ch,
                                                   Fdc2214CapSample_t *outSample);

static esp_err_t sensorarrayMeasureReadFdcSampleDiagWithReader(sensorarrayFdcReadSampleFn_t readFn,
                                                               Fdc2214CapDevice_t *dev,
                                                               Fdc2214CapChannel_t ch,
                                                               bool discardFirst,
                                                               bool idOk,
                                                               bool configOk,
                                                               bool relaxedMode,
                                                               sensorarrayFdcReadDiag_t *outDiag)
{
    if (!readFn || !dev || !outDiag) {
        return ESP_ERR_INVALID_ARG;
    }

    FDCLOW_TRACE("FDCLOW,stage=diag_enter,channel=%u,discardFirst=%u,relaxed=%u\n",
                    (unsigned)ch,
                    discardFirst ? 1u : 0u,
                    relaxedMode ? 1u : 0u);

    *outDiag = (sensorarrayFdcReadDiag_t){
        .err = ESP_FAIL,
        .i2cOk = false,
        .idOk = idOk,
        .configOk = configOk,
        .statusCode = SENSORARRAY_FDC_SAMPLE_STATUS_I2C_READ_ERROR,
        .sampleValid = false,
        .provisionalReadable = false,
        .qualityDegraded = false,
    };

    if (discardFirst) {
        Fdc2214CapSample_t throwaway = {0};
        FDCLOW_TRACE("FDCLOW,stage=discard_sample_begin,channel=%u\n", (unsigned)ch);
        esp_err_t discardErr = readFn(dev, ch, &throwaway);
        FDCLOW_TRACE("FDCLOW,stage=discard_sample_done,channel=%u,err=%ld\n",
                        (unsigned)ch,
                        (long)discardErr);
        if (discardErr != ESP_OK) {
            outDiag->err = discardErr;
            FDCLOW_TRACE("FDCLOW,stage=diag_done,err=%ld,statusCode=%u\n",
                            (long)discardErr,
                            (unsigned)outDiag->statusCode);
            return discardErr;
        }
    }

    FDCLOW_TRACE("FDCLOW,stage=read_sample_begin,channel=%u\n", (unsigned)ch);
    esp_err_t err = readFn(dev, ch, &outDiag->sample);
    FDCLOW_TRACE("FDCLOW,stage=read_sample_done,channel=%u,err=%ld,raw=%lu,status=0x%04X,"
                    "config=0x%04X,mux=0x%04X,statusCode=%u\n",
                    (unsigned)ch,
                    (long)err,
                    (unsigned long)outDiag->sample.Raw28,
                    outDiag->sample.StatusRaw,
                    outDiag->sample.ConfigRaw,
                    outDiag->sample.MuxRaw,
                    (unsigned)outDiag->sample.SampleStatus);
    outDiag->err = err;
    outDiag->i2cOk = (err == ESP_OK);
    if (err != ESP_OK) {
        outDiag->statusCode = SENSORARRAY_FDC_SAMPLE_STATUS_I2C_READ_ERROR;
        outDiag->sampleValid = false;
        outDiag->provisionalReadable = false;
        FDCLOW_TRACE("FDCLOW,stage=diag_done,err=%ld,statusCode=%u\n",
                        (long)err,
                        (unsigned)outDiag->statusCode);
        return err;
    }

    FDCLOW_TRACE("FDCLOW,stage=read_status_config_begin\n");
    esp_err_t statusCfgErr = Fdc2214CapReadCoreRegs(dev, &outDiag->coreRegs);
    FDCLOW_TRACE("FDCLOW,stage=read_status_config_done,err=%ld,statusConfig=0x%04X\n",
                    (long)statusCfgErr,
                    outDiag->coreRegs.StatusConfig);
    if (statusCfgErr != ESP_OK) {
        outDiag->err = statusCfgErr;
        outDiag->i2cOk = false;
        outDiag->statusCode = SENSORARRAY_FDC_SAMPLE_STATUS_I2C_READ_ERROR;
        outDiag->sampleValid = false;
        outDiag->provisionalReadable = false;
        FDCLOW_TRACE("FDCLOW,stage=diag_done,err=%ld,statusCode=%u\n",
                        (long)statusCfgErr,
                        (unsigned)outDiag->statusCode);
        return statusCfgErr;
    }

    (void)Fdc2214CapDecodeStatusRaw(outDiag->sample.StatusRaw, &outDiag->status);

    outDiag->converting = outDiag->sample.Converting;
    outDiag->unreadConversionPresent = Fdc2214CapStatusHasUnreadForChannel(&outDiag->status, ch);

    sensorarrayFdcSampleStatus_t mappedStatus = sensorarrayMeasureMapFdcStatus(outDiag->sample.SampleStatus);
    if (!idOk || !configOk) {
        mappedStatus = SENSORARRAY_FDC_SAMPLE_STATUS_CONFIG_UNKNOWN;
    }
    bool watchdogFault = Fdc2214CapStatusHasWatchdogFault(&outDiag->status);
    bool amplitudeFault = Fdc2214CapStatusHasAmplitudeFault(&outDiag->status) ||
                          outDiag->sample.ErrAmplitude;
    bool readable = outDiag->unreadConversionPresent || outDiag->status.DataReady || outDiag->sample.DataReady;
    outDiag->statusCode = mappedStatus;
    outDiag->qualityDegraded = (!readable) ||
                               outDiag->sample.ErrWatchdog ||
                               outDiag->sample.ErrAmplitude ||
                               watchdogFault ||
                               amplitudeFault;
    outDiag->provisionalReadable = idOk &&
                                   configOk &&
                                   outDiag->sample.Converting &&
                                   (outDiag->sample.Raw28 != 0u) &&
                                   !outDiag->sample.ErrWatchdog &&
                                   !watchdogFault &&
                                   readable;
    outDiag->sampleValid = relaxedMode ? outDiag->provisionalReadable
                                       : (outDiag->provisionalReadable &&
                                          (mappedStatus == SENSORARRAY_FDC_SAMPLE_STATUS_SAMPLE_VALID ||
                                           mappedStatus == SENSORARRAY_FDC_SAMPLE_STATUS_AMPLITUDE_FAULT));
    if (!idOk || !configOk) {
        outDiag->sampleValid = false;
        outDiag->provisionalReadable = false;
    }
    FDCLOW_TRACE("FDCLOW,stage=diag_done,err=%ld,statusCode=%u,sampleValid=%u,provisional=%u\n",
                    (long)outDiag->err,
                    (unsigned)outDiag->statusCode,
                    outDiag->sampleValid ? 1u : 0u,
                    outDiag->provisionalReadable ? 1u : 0u);
    return ESP_OK;
}

esp_err_t sensorarrayMeasureReadFdcSampleDiag(Fdc2214CapDevice_t *dev,
                                              Fdc2214CapChannel_t ch,
                                              bool discardFirst,
                                              bool idOk,
                                              bool configOk,
                                              sensorarrayFdcReadDiag_t *outDiag)
{
    return sensorarrayMeasureReadFdcSampleDiagWithReader(Fdc2214CapReadSample,
                                                         dev,
                                                         ch,
                                                         discardFirst,
                                                         idOk,
                                                         configOk,
                                                         false,
                                                         outDiag);
}

esp_err_t sensorarrayMeasureReadFdcSampleDiagRelaxed(Fdc2214CapDevice_t *dev,
                                                     Fdc2214CapChannel_t ch,
                                                     bool discardFirst,
                                                     bool idOk,
                                                     bool configOk,
                                                     sensorarrayFdcReadDiag_t *outDiag)
{
    return sensorarrayMeasureReadFdcSampleDiagWithReader(Fdc2214CapReadSampleRelaxed,
                                                         dev,
                                                         ch,
                                                         discardFirst,
                                                         idOk,
                                                         configOk,
                                                         true,
                                                         outDiag);
}

esp_err_t sensorarrayMeasureReadFdcSample(Fdc2214CapDevice_t *dev,
                                          Fdc2214CapChannel_t ch,
                                          bool discardFirst,
                                          Fdc2214CapSample_t *outSample)
{
    if (!dev || !outSample) {
        return ESP_ERR_INVALID_ARG;
    }

    sensorarrayFdcReadDiag_t diag = {0};
    esp_err_t err = sensorarrayMeasureReadFdcSampleDiag(dev, ch, discardFirst, true, true, &diag);
    if (err != ESP_OK) {
        return err;
    }
    *outSample = diag.sample;
    return ESP_OK;
}

double sensorarrayMeasureFdcRawToFrequencyHz(uint32_t raw28, uint32_t refClockHz)
{
    if (raw28 == 0u || refClockHz == 0u) {
        return 0.0;
    }
    return ((double)raw28 * (double)refClockHz) / SENSORARRAY_FDC_RAW_SCALE_2P28;
}

const char *sensorarrayMeasureFdcRefClockSourceName(sensorarrayFdcRefClockSource_t source)
{
    switch (source) {
    case SENSORARRAY_FDC_REF_CLOCK_SOURCE_INTERNAL:
        return "internal_oscillator";
    case SENSORARRAY_FDC_REF_CLOCK_SOURCE_EXTERNAL:
        return "external_clkin";
    default:
        return "unknown";
    }
}

sensorarrayFdcRefClockSource_t sensorarrayMeasureFdcEffectiveRefClockSource(void)
{
#if SENSORARRAY_FDC_REF_CLOCK_USE_EXTERNAL
    return SENSORARRAY_FDC_REF_CLOCK_SOURCE_EXTERNAL;
#else
    return SENSORARRAY_FDC_REF_CLOCK_SOURCE_INTERNAL;
#endif
}

uint32_t sensorarrayMeasureFdcEffectiveFclkHz(void)
{
#if SENSORARRAY_FDC_REF_CLOCK_USE_EXTERNAL
    return SENSORARRAY_FDC_EXTERNAL_CLOCK_HZ;
#else
#if SENSORARRAY_FDC_INTERNAL_CLOCK_CALIBRATED_HZ > 0u
    return SENSORARRAY_FDC_INTERNAL_CLOCK_CALIBRATED_HZ;
#else
    return SENSORARRAY_FDC_INTERNAL_CLOCK_NOMINAL_HZ;
#endif
#endif
}

bool sensorarrayMeasureFdcDecodeClockDividers(uint16_t clockDividers,
                                              uint8_t *outFinSelCode,
                                              uint8_t *outFinFactor,
                                              uint16_t *outFrefDivider,
                                              const char **outStatus)
{
    if (!outFinSelCode || !outFinFactor || !outFrefDivider || !outStatus) {
        if (outStatus) {
            *outStatus = "invalid_arg";
        }
        return false;
    }

    uint8_t finSelCode = (uint8_t)((clockDividers >> 12) & 0x03u);
    uint16_t frefDivider = (uint16_t)(clockDividers & 0x03FFu);

    *outFinSelCode = finSelCode;
    *outFinFactor = 0u;
    *outFrefDivider = frefDivider;
    *outStatus = "ok";

    if (frefDivider == 0u) {
        *outStatus = "zero_fref_divider";
        return false;
    }
    if (finSelCode == 0u) {
        *outStatus = "invalid_fin_sel_zero";
        return false;
    }
    if (finSelCode == 1u) {
        *outFinFactor = 1u;
        return true;
    }
    if (finSelCode == 2u) {
        *outFinFactor = 2u;
        return true;
    }

    *outStatus = "unsupported_fin_sel_3";
    return false;
}

double sensorarrayMeasureFdcFinFactorFromClockDiv(uint16_t clockDividers)
{
    uint8_t finSelCode = 0u;
    uint8_t finFactor = 0u;
    uint16_t frefDivider = 0u;
    const char *status = "unknown";
    if (!sensorarrayMeasureFdcDecodeClockDividers(clockDividers,
                                                  &finSelCode,
                                                  &finFactor,
                                                  &frefDivider,
                                                  &status)) {
        return 0.0;
    }
    (void)finSelCode;
    (void)frefDivider;
    (void)status;
    return (double)finFactor;
}

double sensorarrayMeasureFdcFrefDividerFromClockDiv(uint16_t clockDividers)
{
    uint8_t finSelCode = 0u;
    uint8_t finFactor = 0u;
    uint16_t frefDivider = 0u;
    const char *status = "unknown";
    if (!sensorarrayMeasureFdcDecodeClockDividers(clockDividers,
                                                  &finSelCode,
                                                  &finFactor,
                                                  &frefDivider,
                                                  &status)) {
        return 0.0;
    }
    (void)finSelCode;
    (void)finFactor;
    (void)status;
    return (double)frefDivider;
}

double sensorarrayMeasureFdcRaw28ToFreqHz(uint32_t raw28,
                                          uint32_t effectiveFclkHz,
                                          uint16_t clockDividers)
{
    if (raw28 == 0u || effectiveFclkHz == 0u) {
        return 0.0;
    }

    double finFactor = sensorarrayMeasureFdcFinFactorFromClockDiv(clockDividers);
    double frefDivider = sensorarrayMeasureFdcFrefDividerFromClockDiv(clockDividers);
    if (finFactor <= 0.0 || frefDivider <= 0.0) {
        return 0.0;
    }

    double effectiveFrefHz = (double)effectiveFclkHz / frefDivider;
    return ((double)raw28 * effectiveFrefHz * finFactor) / SENSORARRAY_FDC_RAW_SCALE_2P28;
}

bool sensorarrayMeasureFdcComputeFrequencyDiag(uint32_t raw28,
                                               uint16_t clockDividers,
                                               sensorarrayFdcFrequencyDiag_t *outDiag)
{
    if (!outDiag) {
        return false;
    }

    *outDiag = (sensorarrayFdcFrequencyDiag_t){
        .clockDividers = clockDividers,
        .refClockSource = sensorarrayMeasureFdcEffectiveRefClockSource(),
        .effectiveFclkHz = sensorarrayMeasureFdcEffectiveFclkHz(),
        .valid = false,
        .status = "unknown",
    };

    if (raw28 == 0u) {
        outDiag->status = "zero_raw";
        return false;
    }
    if (outDiag->effectiveFclkHz == 0u) {
        outDiag->status = "zero_effective_fclk";
        return false;
    }

    const char *decodeStatus = "unknown";
    bool decodeOk = sensorarrayMeasureFdcDecodeClockDividers(clockDividers,
                                                             &outDiag->finSelCode,
                                                             &outDiag->finFactor,
                                                             &outDiag->frefDivider,
                                                             &decodeStatus);
    outDiag->status = decodeStatus;
    if (!decodeOk) {
        return false;
    }

    outDiag->effectiveFrefHz = (double)outDiag->effectiveFclkHz / (double)outDiag->frefDivider;
    outDiag->freqHzBase = sensorarrayMeasureFdcRawToFrequencyHz(raw28, outDiag->effectiveFclkHz);
    outDiag->freqHzCorrected =
        sensorarrayMeasureFdcRaw28ToFreqHz(raw28, outDiag->effectiveFclkHz, clockDividers);
    outDiag->valid = true;
    outDiag->status = "ok";
    return true;
}

double sensorarrayMeasureFdcRawToSensorFrequencyHz(uint32_t raw28, uint16_t clockDividers)
{
    sensorarrayFdcFrequencyDiag_t diag = {0};
    if (!sensorarrayMeasureFdcComputeFrequencyDiag(raw28, clockDividers, &diag) || !diag.valid) {
        return 0.0;
    }
    return diag.freqHzCorrected;
}

bool sensorarrayMeasureFdcTryCapacitancePf(double frequencyHz, uint32_t inductorUh, double *outCapPf)
{
    return sensorarrayMeasureFdcComputeCapacitancePf(frequencyHz, (double)inductorUh, outCapPf);
}

bool sensorarrayMeasureFdcComputeCapacitancePf(double frequencyHz, double inductorValueUh, double *outCapPf)
{
    if (!outCapPf || frequencyHz <= 0.0 || inductorValueUh <= 0.0) {
        return false;
    }

    const double inductorH = inductorValueUh * 1e-6;
    const double omega = 2.0 * SENSORARRAY_PI * frequencyHz;
    const double denom = omega * omega * inductorH;
    if (!isfinite(denom) || denom <= 0.0) {
        return false;
    }

    const double cPf = (1.0 / denom) * 1e12;
    if (!isfinite(cPf) || cPf <= 0.0) {
        return false;
    }

    *outCapPf = cPf;
    return true;
}

esp_err_t sensorarrayMeasureAdsReadRegister(sensorarrayState_t *state, uint8_t reg, uint8_t *outValue)
{
    if (!state || !outValue || !state->adsReady) {
        return ESP_ERR_INVALID_STATE;
    }
    return ads126xAdcReadRegisters(&state->ads, reg, outValue, 1);
}

esp_err_t sensorarrayMeasureReadAdsKeyRegisterSnapshot(sensorarrayState_t *state,
                                                       sensorarrayAdsRegSnapshot_t *outSnapshot)
{
    if (!state || !outSnapshot) {
        return ESP_ERR_INVALID_ARG;
    }

    *outSnapshot = (sensorarrayAdsRegSnapshot_t){0};

    esp_err_t err = sensorarrayMeasureAdsReadRegister(state, SENSORARRAY_ADS_REG_ID, &outSnapshot->id);
    if (err == ESP_OK) {
        err = ads126xAdcReadCoreRegisters(&state->ads,
                                          &outSnapshot->power,
                                          &outSnapshot->iface,
                                          &outSnapshot->mode2,
                                          &outSnapshot->inpmux,
                                          &outSnapshot->refmux);
    }

    if (err == ESP_OK) {
        state->adsRefMux = outSnapshot->refmux;
        state->adsRefMuxValid = true;
    }

    return err;
}

esp_err_t sensorarrayMeasureDumpAdsKeyRegisters(sensorarrayState_t *state, const char *stage)
{
    sensorarrayAdsRegSnapshot_t regs = {0};
    esp_err_t err = sensorarrayMeasureReadAdsKeyRegisterSnapshot(state, &regs);

    printf("DBGADSREG,stage=%s,id=0x%02X,power=0x%02X,interface=0x%02X,mode2=0x%02X,inpmux=0x%02X,refmux=0x%02X,"
           "err=%ld,status=%s\n",
           stage ? stage : SENSORARRAY_NA,
           regs.id,
           regs.power,
           regs.iface,
           regs.mode2,
           regs.inpmux,
           regs.refmux,
           (long)err,
           (err == ESP_OK) ? "ok" : "read_error");
    return err;
}

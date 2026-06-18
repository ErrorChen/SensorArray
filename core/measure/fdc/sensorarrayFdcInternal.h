#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_err.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "sensorarrayBoardMap.h"
#include "sensorarrayAdsGap.h"
#include "sensorarrayAcqEvent.h"
#include "sensorarrayConfig.h"
#include "sensorarrayFdcSweep.h"
#include "sensorarrayLog.h"
#include "sensorarrayScanConfig.h"
#include "sensorarrayMeasure.h"

#define printf sensorarrayAcqEventPrintf

#ifndef CONFIG_FDC2214CAP_LOW_LEVEL_I2C_TRACE
#define CONFIG_FDC2214CAP_LOW_LEVEL_I2C_TRACE 0
#endif
#ifndef CONFIG_SENSORARRAY_FDC_TIMING_SUMMARY_EVERY_N_FRAMES
#ifdef CONFIG_SENSORARRAY_FDC_TIMING_LOG_EVERY_N_FRAMES
#define CONFIG_SENSORARRAY_FDC_TIMING_SUMMARY_EVERY_N_FRAMES CONFIG_SENSORARRAY_FDC_TIMING_LOG_EVERY_N_FRAMES
#else
#define CONFIG_SENSORARRAY_FDC_TIMING_SUMMARY_EVERY_N_FRAMES 20
#endif
#endif
#ifndef CONFIG_SENSORARRAY_FDC_TIMING_SUMMARY_PERIOD_FRAMES
#define CONFIG_SENSORARRAY_FDC_TIMING_SUMMARY_PERIOD_FRAMES 20
#endif
#ifndef CONFIG_SENSORARRAY_FDC_TIMING_SUMMARY_AGGREGATE
#define CONFIG_SENSORARRAY_FDC_TIMING_SUMMARY_AGGREGATE 1
#endif
#ifndef CONFIG_SENSORARRAY_FDC_TIMING_OVERRUN_IMMEDIATE_LOG
#define CONFIG_SENSORARRAY_FDC_TIMING_OVERRUN_IMMEDIATE_LOG 1
#endif
#ifndef CONFIG_SENSORARRAY_FDC_OVERRUN_HARD_US
#define CONFIG_SENSORARRAY_FDC_OVERRUN_HARD_US 250000
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
#ifndef CONFIG_SENSORARRAY_FDC_INTB_DIRECT_DATA_ENABLE
#define CONFIG_SENSORARRAY_FDC_INTB_DIRECT_DATA_ENABLE 1
#endif
#ifndef CONFIG_SENSORARRAY_FDC_INTB_ACTIVE_LOW
#define CONFIG_SENSORARRAY_FDC_INTB_ACTIVE_LOW 1
#endif
#ifndef CONFIG_SENSORARRAY_FDC_INTB1_GPIO
#define CONFIG_SENSORARRAY_FDC_INTB1_GPIO 17
#endif
#ifndef CONFIG_SENSORARRAY_FDC_INTB2_GPIO
#define CONFIG_SENSORARRAY_FDC_INTB2_GPIO 18
#endif
#ifndef CONFIG_SENSORARRAY_FDC_DEBUG_TIMING_GPIO_ENABLE
#define CONFIG_SENSORARRAY_FDC_DEBUG_TIMING_GPIO_ENABLE 0
#endif
#ifndef CONFIG_SENSORARRAY_FDC_ROW_STROBE_GPIO
#define CONFIG_SENSORARRAY_FDC_ROW_STROBE_GPIO -1
#endif
#ifndef CONFIG_SENSORARRAY_FDC_FRAME_STROBE_GPIO
#define CONFIG_SENSORARRAY_FDC_FRAME_STROBE_GPIO -1
#endif
#ifndef CONFIG_SENSORARRAY_FDC_PRIMARY_READ_WINDOW_GPIO
#define CONFIG_SENSORARRAY_FDC_PRIMARY_READ_WINDOW_GPIO -1
#endif
#ifndef CONFIG_SENSORARRAY_FDC_SECONDARY_READ_WINDOW_GPIO
#define CONFIG_SENSORARRAY_FDC_SECONDARY_READ_WINDOW_GPIO -1
#endif
#ifndef CONFIG_SENSORARRAY_FDC_DEBUG_STROBE_PULSE_US
#define CONFIG_SENSORARRAY_FDC_DEBUG_STROBE_PULSE_US 2
#endif
#ifndef CONFIG_SENSORARRAY_FDC_INTB_TRIGGER_ANYEDGE
#define CONFIG_SENSORARRAY_FDC_INTB_TRIGGER_ANYEDGE 0
#endif
#ifndef CONFIG_SENSORARRAY_FDC_INTB_TRIGGER_FALLING_EDGE
#define CONFIG_SENSORARRAY_FDC_INTB_TRIGGER_FALLING_EDGE 1
#endif
#ifndef CONFIG_SENSORARRAY_FDC_INTB_WAIT_TIMEOUT_US
#define CONFIG_SENSORARRAY_FDC_INTB_WAIT_TIMEOUT_US 10000
#endif
#ifndef CONFIG_SENSORARRAY_FDC_ROW_WAIT_SAFETY_US
#define CONFIG_SENSORARRAY_FDC_ROW_WAIT_SAFETY_US 2000
#endif
#ifndef CONFIG_SENSORARRAY_FDC_INTB_FALLBACK_POLLING
#define CONFIG_SENSORARRAY_FDC_INTB_FALLBACK_POLLING 1
#endif
#ifndef CONFIG_SENSORARRAY_FDC_INTB_WEAK_PULLUP
#define CONFIG_SENSORARRAY_FDC_INTB_WEAK_PULLUP 1
#endif
#ifndef CONFIG_SENSORARRAY_FDC_READY_GUARD_US
#define CONFIG_SENSORARRAY_FDC_READY_GUARD_US 3000
#endif
#ifndef CONFIG_SENSORARRAY_FDC_STATUS_AFTER_TIMEOUT_FALLBACK
#define CONFIG_SENSORARRAY_FDC_STATUS_AFTER_TIMEOUT_FALLBACK 1
#endif
#ifndef CONFIG_SENSORARRAY_FDC_INTB_STATUS_CONFIRM_RETRY
#define CONFIG_SENSORARRAY_FDC_INTB_STATUS_CONFIRM_RETRY 1
#endif
#ifndef CONFIG_SENSORARRAY_FDC_INTB_STATUS_CONFIRM_RETRY_US
#define CONFIG_SENSORARRAY_FDC_INTB_STATUS_CONFIRM_RETRY_US 100
#endif
#ifndef CONFIG_SENSORARRAY_FDC_READY_MAX_POLLS_AFTER_UNREAD_BEFORE_DRDY
#define CONFIG_SENSORARRAY_FDC_READY_MAX_POLLS_AFTER_UNREAD_BEFORE_DRDY 3
#endif
#ifndef CONFIG_SENSORARRAY_FDC_READY_POLL_INTERVAL_US
#define CONFIG_SENSORARRAY_FDC_READY_POLL_INTERVAL_US 1000
#endif
#ifndef CONFIG_SENSORARRAY_FDC_AFTER_INTB_RECHECK_MAX
#define CONFIG_SENSORARRAY_FDC_AFTER_INTB_RECHECK_MAX 1
#endif
#ifndef CONFIG_SENSORARRAY_FDC_AFTER_INTB_RECHECK_INTERVAL_US
#define CONFIG_SENSORARRAY_FDC_AFTER_INTB_RECHECK_INTERVAL_US 100
#endif
#ifndef CONFIG_SENSORARRAY_FDC_AFTER_INTB_RECHECK_DEADLINE_US
#define CONFIG_SENSORARRAY_FDC_AFTER_INTB_RECHECK_DEADLINE_US 1000
#endif
#ifndef CONFIG_SENSORARRAY_FDC_ROW_DEVICE_RETRY_MAX
#define CONFIG_SENSORARRAY_FDC_ROW_DEVICE_RETRY_MAX 1
#endif
#ifndef CONFIG_SENSORARRAY_FDC_ROW_DEVICE_WATCHDOG_MULTIPLIER
#define CONFIG_SENSORARRAY_FDC_ROW_DEVICE_WATCHDOG_MULTIPLIER 10
#endif
#ifndef CONFIG_SENSORARRAY_FDC_ROW_DEVICE_WATCHDOG_OVERRIDE_US
#define CONFIG_SENSORARRAY_FDC_ROW_DEVICE_WATCHDOG_OVERRIDE_US 0
#endif
#ifndef CONFIG_SENSORARRAY_FDC_RECOVERY_RESET_ON_PROGRESS
#define CONFIG_SENSORARRAY_FDC_RECOVERY_RESET_ON_PROGRESS 1
#endif
#ifndef CONFIG_SENSORARRAY_FDC_FORMAL_FAST_PROFILE_ENABLE
#define CONFIG_SENSORARRAY_FDC_FORMAL_FAST_PROFILE_ENABLE 0
#endif
#ifndef CONFIG_SENSORARRAY_FDC_FORMAL_FAST_TARGET_ROUND_US
#define CONFIG_SENSORARRAY_FDC_FORMAL_FAST_TARGET_ROUND_US 4000
#endif
#ifndef CONFIG_SENSORARRAY_FDC_PROFILE_TOO_SLOW_WARN_US
#define CONFIG_SENSORARRAY_FDC_PROFILE_TOO_SLOW_WARN_US SENSORARRAY_FDC_TARGET_ROW_US
#endif
#ifndef CONFIG_SENSORARRAY_FDC_PROFILE_TOO_SLOW_RESCUE_ENABLE
#define CONFIG_SENSORARRAY_FDC_PROFILE_TOO_SLOW_RESCUE_ENABLE 0
#endif
#ifndef CONFIG_SENSORARRAY_FDC_PFU_LOG_EVERY_N_FRAMES
#define CONFIG_SENSORARRAY_FDC_PFU_LOG_EVERY_N_FRAMES 100
#endif
#ifndef CONFIG_SENSORARRAY_FDC_PROFILE_LOG_EVERY_N_FRAMES
#define CONFIG_SENSORARRAY_FDC_PROFILE_LOG_EVERY_N_FRAMES 20
#endif
#ifndef CONFIG_SENSORARRAY_FDC_READY_LOG_EVERY_N_FRAMES
#define CONFIG_SENSORARRAY_FDC_READY_LOG_EVERY_N_FRAMES 20
#endif
#ifndef CONFIG_SENSORARRAY_FDC_TIMING_COMPACT_EVERY_N_FRAMES
#define CONFIG_SENSORARRAY_FDC_TIMING_COMPACT_EVERY_N_FRAMES 20
#endif
#ifndef CONFIG_SENSORARRAY_FDC_QUALITY_LOG_EVERY_N_FRAMES
#define CONFIG_SENSORARRAY_FDC_QUALITY_LOG_EVERY_N_FRAMES 20
#endif
#ifndef CONFIG_SENSORARRAY_FDC_I2C_LOG_EVERY_N_FRAMES
#define CONFIG_SENSORARRAY_FDC_I2C_LOG_EVERY_N_FRAMES 20
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
#ifndef CONFIG_SENSORARRAY_FDC_ROW_VERBOSE_LOG
#define CONFIG_SENSORARRAY_FDC_ROW_VERBOSE_LOG 1
#endif
#ifndef CONFIG_SENSORARRAY_FDC_SAMPLE_DEVICE_LOG_EVERY_N_FRAMES
#define CONFIG_SENSORARRAY_FDC_SAMPLE_DEVICE_LOG_EVERY_N_FRAMES 0
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
#define SENSORARRAY_FDC_TARGET_FRAME_US SENSORARRAY_CFG_FRAME_PERIOD_US
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
#if SENSORARRAY_CFG_FDC_READY_POLL_ONLY
#define SENSORARRAY_FDC_READY_MODE_NAME "POLL_ONLY"
#elif SENSORARRAY_CFG_FDC_STATUS_CONFIRM_ENABLED
#define SENSORARRAY_FDC_READY_MODE_NAME "INTB_THEN_STATUS"
#elif SENSORARRAY_CFG_FDC_READY_STRICT_ENABLED
#define SENSORARRAY_FDC_READY_MODE_NAME "INTB_STRICT_LEVEL"
#else
#define SENSORARRAY_FDC_READY_MODE_NAME "INTB_WITH_POLL_FALLBACK"
#endif
#if SENSORARRAY_FDC_INTB_OUTPUT_ENABLE
#define SENSORARRAY_FDC_INTB_HINT_NAME "enabled"
#else
#define SENSORARRAY_FDC_INTB_HINT_NAME "disabled"
#endif
#define SENSORARRAY_FDC_NO_UNREAD_RESYNC_THRESHOLD 3u


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
    bool softInvalid[4];
    bool hardInvalid[4];
    uint16_t statusRaw;
    uint8_t unreadMask;
    uint8_t freshMask;
    uint8_t validMask;
    uint8_t warnMask;
    uint8_t errorMask;
    uint8_t notReadyMask;
    uint8_t zeroBeforeReadyMask;
    uint8_t zeroAfterDrdyMask;
    uint8_t softInvalidMask;
    uint8_t hardInvalidMask;
    bool timeout;
    bool partial;
    bool i2cTransactionError;
    bool dataReady;
    bool unreadWithoutDrdy;
    bool staleUnreadDrain;
    bool readyStatusReadable;
    bool statusFallbackUsed;
    bool originalIntbMiss;
    bool originalTimeout;
    bool waitBudgetTooShort;
    bool levelLowButEdgeMiss;
    bool dataReadAttempted;
    bool directDataRead;
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
    bool softInvalidSeen[SENSORARRAY_MATRIX_ROWS][SENSORARRAY_MATRIX_COLS];
    bool hardInvalidSeen[SENSORARRAY_MATRIX_ROWS][SENSORARRAY_MATRIX_COLS];
    bool staleUnreadDrainSeen[SENSORARRAY_MATRIX_ROWS][SENSORARRAY_MATRIX_COLS];
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
    SENSORARRAY_FDC_READY_STATUS_READY_BEFORE_WAIT,
    SENSORARRAY_FDC_READY_POLL_FULL,
    SENSORARRAY_FDC_READY_POLL_RECOVERED_AFTER_UNREAD_BEFORE_DRDY,
    SENSORARRAY_FDC_READY_AFTER_INTB_RECHECK_FULL,
    SENSORARRAY_FDC_READY_INTB_TIMEOUT,
    SENSORARRAY_FDC_READY_DRDY_NOT_CLOSED_AFTER_INTB,
    SENSORARRAY_FDC_READY_STATUS_INCONSISTENT,
    SENSORARRAY_FDC_READY_POLL_PARTIAL,
    SENSORARRAY_FDC_READY_TIMEOUT_PARTIAL,
    SENSORARRAY_FDC_READY_TIMEOUT_NONE,
    SENSORARRAY_FDC_READY_I2C_ERROR,
    SENSORARRAY_FDC_READY_UNREAD_FULL_NO_DRDY_TRANSIENT,
    SENSORARRAY_FDC_READY_STALE_UNREAD_NO_DRDY,
    SENSORARRAY_FDC_READY_HARD_TIMEOUT,
    SENSORARRAY_FDC_READY_STATUS_FALLBACK_AFTER_INTB_MISS,
    SENSORARRAY_FDC_READY_STATUS_READY_AFTER_TIMEOUT,
    SENSORARRAY_FDC_READY_LEVEL_ACTIVE_FALLBACK,
    SENSORARRAY_FDC_READY_WAIT_BUDGET_TOO_SHORT_STATUS_FALLBACK,
    SENSORARRAY_FDC_READY_EPOCH_MISMATCH_OR_STALE_WORKER_RESULT,
} sensorarrayFdcReadyKind_t;

typedef enum {
    FDC_READY_RESULT_NONE = 0,
    FDC_READY_OK_INTB_DRDY_UNREAD_FULL,
    FDC_READY_OK_STATUS_READY_AFTER_TIMEOUT,
    FDC_READY_INTB_ACTIVE_STATUS_MISMATCH,
    FDC_READY_NOT_READY_AFTER_GUARD,
    FDC_READY_HARD_TIMEOUT_NO_DRDY,
    FDC_READY_I2C_ERROR,
    FDC_READY_INTERNAL_STATE_ERROR,
    FDC_READY_OK_DRDY_UNREAD_FULL,
    FDC_READY_RECOVERED_AFTER_RETRY,
    FDC_READY_UNREAD_FULL_NO_DRDY_TRANSIENT,
    FDC_READY_STALE_UNREAD_NO_DRDY,
    FDC_READY_HARD_TIMEOUT,
    FDC_READY_EPOCH_MISMATCH_OR_STALE_WORKER_RESULT,
} sensorarrayFdcReadyResult_t;

typedef enum {
    SENSORARRAY_FDC_READY_POLICY_POLL_ONLY = 0,
    SENSORARRAY_FDC_READY_POLICY_INTB_THEN_STATUS = 1,
    SENSORARRAY_FDC_READY_POLICY_INTB_WITH_POLL_FALLBACK = 2,
    SENSORARRAY_FDC_READY_POLICY_INTB_STRICT_LEVEL = 3,
} sensorarrayFdcReadyPolicy_t;

typedef enum {
    SENSORARRAY_FDC_WATCHDOG_INTB_TIMEOUT = 0,
    SENSORARRAY_FDC_WATCHDOG_DRDY_NOT_CLOSED_AFTER_INTB,
    SENSORARRAY_FDC_WATCHDOG_STATUS_INCONSISTENT,
    SENSORARRAY_FDC_WATCHDOG_READ4_I2C_ERROR,
    SENSORARRAY_FDC_WATCHDOG_ZERO_AFTER_DRDY,
    SENSORARRAY_FDC_WATCHDOG_RAW_ALL_ZERO,
    SENSORARRAY_FDC_WATCHDOG_AMPLITUDE_WARNING,
    SENSORARRAY_FDC_WATCHDOG_SENSOR_WATCHDOG_FAULT,
    SENSORARRAY_FDC_WATCHDOG_SATURATED,
    SENSORARRAY_FDC_WATCHDOG_PROFILE_TOO_SLOW,
} sensorarrayFdcWatchdogReason_t;

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
    sensorarrayFdcReadyResult_t readyResult;
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
    uint32_t guardMarginUs;
    uint32_t guardDeadlineUs;
    uint32_t actualIntbWaitUs;
    uint32_t rowDeviceHardUs;
    uint32_t hardDeadlineRemainingUs;
    uint32_t hardDeadlineRemainingBeforeWaitUs;
    const char *waitSource;
    const char *estKind;
    bool waitClampedByHardDeadline;
    bool waitBudgetTooShort;
    uint32_t plannedIntbWaitUs;
    bool returnedBeforeEstimatedRound;
    const char *waitReturnReason;
    uint32_t notifyValue;
    int rawLevelAfterArm;
    int rawLevelAtWaitReturn;
    bool activeLowConfigured;
    bool levelActiveAfterArm;
    bool levelActiveAtWaitReturn;
    uint32_t intbWaitUs;
    uint32_t statusVerifyUs;
    uint32_t statusPrecheckUs;
    uint32_t pollFallbackUs;
    uint32_t statusReadsPrecheck;
    uint32_t statusReadsBeforeIntb;
    uint32_t statusReadsAfterIntb;
    uint32_t statusReadsAfterIntbRecheck;
    uint32_t statusReadsInFallback;
    uint32_t statusReadsSuppressedBeforeIntb;
    uint32_t statusAckCount;
    uint32_t statusReadsPollDiag;
    uint32_t statusReadsWatchdogDiag;
    bool intbMiss;
    bool watchdogOnly;
    bool diagOnly;
    bool originalWaitMiss;
    bool originalIntbMiss;
    bool originalTimeout;
    bool originalWatchdogOnly;
    esp_err_t originalErr;
    bool statusFallbackUsed;
    bool recoveredByStatusReady;
    bool recoveredByLevelLow;
    bool acceptedByStatusFallback;
    bool preStatusReady;
    bool lateStatusReady;
    bool trueTimeoutNotReady;
    bool directDataCandidate;
    int intbBeforeStatus;
    int intbAfterStatus;
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
    sensorarrayFdcReadyResult_t readyResult;
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
    uint32_t estimatedRoundUs;
    uint32_t actualIntbWaitUs;
    const char *readyDiagnostic;
    uint8_t softInvalidMask4;
    uint8_t hardInvalidMask4;
    bool readyStatusReadable;
    bool statusFallbackUsed;
    bool originalIntbMiss;
    bool originalTimeout;
    bool waitBudgetTooShort;
    bool levelLowButEdgeMiss;
    bool dataReadAttempted;
    bool directDataRead;
} sensorarrayFdcDeviceRead4Result_t;

typedef struct {
    esp_err_t err;
    sensorarrayFdcReadyState_t ready;
    sensorarrayFdcDeviceRead4Result_t read4;
    uint32_t frameSeq;
    uint8_t row;
    sensorarrayFdcDeviceId_t devId;
    uint32_t epochId;
    uint32_t generation;
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
    uint64_t readyAckUs;
    uint64_t workerStartUs;
    uint64_t workerEndUs;
    uint64_t workerDeadlineUs;
    uint32_t rowHardDeadlineUs;
    uint32_t generation;
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
    uint32_t rowDeviceBudgetUs;
    uint32_t generation;
} sensorarrayFdcWorkerJob_t;

typedef struct {
    bool initialized;
    bool intbReady;
    bool intbIsrAttached;
    sensorarrayFdcDeviceId_t devId;
    int intbGpio;
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
    volatile uint32_t preparedEpoch;
    volatile int preparedErr;
    volatile bool rowConfigPrepared;
    volatile uint32_t generation;
    volatile sensorarrayFdcWorkerJob_t *job;
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

extern uint32_t s_fdcMatrixSequence;
extern uint32_t s_fdcFreshFrameSequence;
extern bool s_fdcProfileSummaryEnabled;
extern bool s_fdcProfileRowEnabled;
extern bool s_fdcProfileDeviceEnabled;
extern uint32_t s_fdcTimingSummaryEvery;
extern uint8_t s_fdcDiscardFrames;
extern sensorarrayFdcWorkerContext_t s_fdcWorkers[2];
extern bool s_fdcWorkersInitAttempted;
extern bool s_fdcWorkersAvailable;
extern StaticEventGroup_t s_fdcWorkerEventStorage;
extern EventGroupHandle_t s_fdcWorkerEvents;
extern bool s_fdcFormalPrecheckDone;
extern bool s_fdcSecondaryUnavailableLogged;
extern bool s_fdcParallelConfigLogged;
extern uint32_t s_fdcRowEpoch;
extern uint32_t s_fdcParallelCooldownFrames;
extern uint32_t s_fdcNoUnreadConsecutive[2];
extern uint8_t s_fdcSoftReadyMissConsecutive[2][SENSORARRAY_MATRIX_ROWS];
extern uint8_t s_fdcStaleUnreadConsecutive[2][SENSORARRAY_MATRIX_ROWS];
extern uint8_t s_fdcHardReadyTimeoutConsecutive[2][SENSORARRAY_MATRIX_ROWS];
extern uint16_t s_fdcLastDiagStatusRaw[2];
extern uint8_t s_fdcLastDiagUnreadMask[2];
extern uint8_t s_fdcLastDiagRow[2];
extern uint32_t s_fdcLastDiagEpoch[2];
extern bool s_fdcLastDiagDrdy[2];
extern bool s_fdcLastDiagStatusOk[2];
extern bool s_fdcLastDiagWasReady[2];
extern sensorarrayFdcProfileSnapshot_t s_fdcProfileSnapshotByRow[SENSORARRAY_MATRIX_ROWS][2];
extern bool s_fdcIntbRuntimeUsable[2];
extern sensorarrayFdcTimingAggregate_t s_fdcTimingAggregate;


enum {
    SENSORARRAY_FDC_DIRECT_FALLBACK_I2C = (1u << 0),
    SENSORARRAY_FDC_DIRECT_FALLBACK_VALID_MASK = (1u << 1),
    SENSORARRAY_FDC_DIRECT_FALLBACK_ERROR_MASK = (1u << 2),
    SENSORARRAY_FDC_DIRECT_FALLBACK_RAW = (1u << 3),
};

bool sensorarrayMeasureDebugGpioIsValid(int gpio);
void sensorarrayMeasureDebugTimingGpioInit(void);
void sensorarrayMeasureDebugPulse(int gpio);
void sensorarrayMeasureDebugReadWindow(sensorarrayFdcDeviceId_t devId, bool active);
void sensorarrayMeasureDebugTimingGpioPrepare(void);
void sensorarrayMeasureDebugPulseRowStrobe(void);
void sensorarrayMeasureDebugPulseFrameStrobe(void);
void sensorarrayMeasureDebugSetReadWindow(sensorarrayFdcDeviceId_t devId, bool active);
esp_err_t sensorarrayMeasureEnsureLock(void);
esp_err_t sensorarrayMeasureTakeLock(void);
void sensorarrayMeasureGiveLock(void);
void sensorarrayDelayMs(uint32_t delayMs);
bool sensorarrayAdsIntRefPolicyUpdates(sensorarrayAdsIntRefPolicy_t policy);
bool sensorarrayAdsVbiasPolicyUpdates(sensorarrayAdsVbiasPolicy_t policy);
int sensorarrayReadResetGpioLevel(void);
esp_err_t sensorarrayMeasureApplyRefPolicy(sensorarrayState_t *state, const char *stage, const char *mode, sensorarrayMatrixDSourcePolicy_t dSource, sensorarrayAdsIntRefPolicy_t intrefPolicy, sensorarrayAdsVbiasPolicy_t vbiasPolicy, const char *reason);
esp_err_t sensorarrayMeasureStopAdsBeforeRoute(sensorarrayState_t *state);
tmux1108Source_t sensorarrayMeasureSourceForSwPhysicalLevel(sensorarraySwPhysicalLevel_t level);
esp_err_t sensorarrayMeasureSetSwPhysicalLevel(sensorarrayState_t *state, sensorarraySwPhysicalLevel_t level, const char *reason);
void sensorarrayMeasureDelayUs(uint32_t delayUs);
esp_err_t sensorarrayMeasureSetSelaPathQuiet(sensorarrayState_t *state, sensorarraySelaRoute_t selaRoute, uint32_t settleDelayUs);
esp_err_t sensorarrayMeasureSetFdcSelBPathQuiet(sensorarrayState_t *state);
esp_err_t sensorarrayMeasureForceAdsReferenceOff(sensorarrayState_t *state);
int sensorarrayMeasureSwPhysicalReadbackFromControl(const tmuxSwitchControlState_t *ctrl);
void sensorarrayMeasureReadFdcPathControl(tmuxSwitchControlState_t *ctrl);
bool sensorarrayMeasureFdcPathControlMatches(const tmuxSwitchControlState_t *ctrl);
esp_err_t sensorarrayMeasurePrepareFdcMatrixPath(sensorarrayState_t *state, const char *reason);
esp_err_t sensorarrayMeasureEnsureFdcMatrixPath(sensorarrayState_t *state, const char *reason);
esp_err_t sensorarrayMeasureSetSwForRoute(sensorarrayState_t *state, const char *stage, uint8_t sColumn, uint8_t dLine, sensorarrayRoutePathKind_t path, tmux1108Source_t swSource, sensorarraySelaRoute_t selaRoute, bool selBLevel, const char *label, const char *status, const char *reason);
esp_err_t sensorarrayMeasureWriteSela(sensorarrayState_t *state, sensorarraySelaRoute_t requestRoute, uint32_t settleDelayMs, const char *stage, const char *label);
sensorarrayFdcDeviceState_t *sensorarrayMeasureGetFdcState(sensorarrayState_t *state, sensorarrayFdcDeviceId_t devId);
sensorarrayFdcDeviceState_t *sensorarrayMeasureGetFdcStateForDLine(sensorarrayState_t *state, uint8_t dLine, const sensorarrayFdcDLineMap_t **outMap);
const char *sensorarrayMeasureFdcDeviceName(sensorarrayFdcDeviceId_t devId);
const char *sensorarrayMeasureEspErrName(esp_err_t err);
bool sensorarrayMeasureDecodeMatrixIndex(uint8_t matrixIndex, uint8_t *outSColumn, uint8_t *outDLine);
bool sensorarrayMeasureMakeFdcCellTarget(sensorarrayState_t *state, uint8_t sColumn, uint8_t dLine, sensorarrayFdcCellTarget_t *outTarget);
sensorarrayFdcCellConfigCache_t *sensorarrayMeasureGetFdcCellCache(sensorarrayState_t *state, const sensorarrayFdcCellTarget_t *target);
void sensorarrayMeasureMarkFdcAppliedCellDirty(sensorarrayState_t *state, const sensorarrayFdcCellTarget_t *target);
bool sensorarrayMeasureFdcReasonEquals(const char *reason, const char *expected);
bool sensorarrayMeasureFdcRescueReasonIsManual(const char *reason);
bool sensorarrayMeasureFdcRescueReasonIsProfileTooSlow(const char *reason);
bool sensorarrayMeasureFdcRescueReasonIsFastSweep(const char *reason);
bool sensorarrayMeasureFdcRescueReasonIsHard(const char *reason);
esp_err_t sensorarrayMeasureRequestFdcCellRescue(sensorarrayState_t *state, uint8_t matrixIndex, const char *reason);
esp_err_t sensorarrayMeasureFdcDiscardStaleSamples(sensorarrayState_t *state, const sensorarrayFdcCellTarget_t *target, uint8_t discardCount, const char *reason);
sensorarrayFdcDeviceId_t sensorarrayMeasureFdcDeviceIdFromState(const sensorarrayFdcDeviceState_t *fdcState);
uint8_t sensorarrayMeasureFdcUnreadMaskFromStatus(const Fdc2214CapStatus_t *status);
sensorarrayFdcReadyDecoded_t sensorarrayMeasureFdcBuildReadyState(const Fdc2214CapStatus_t *status, uint8_t requiredMask);
void sensorarrayMeasureFdcApplyReadyDecoded(sensorarrayFdcReadyState_t *ready, const sensorarrayFdcReadyDecoded_t *decoded);
bool sensorarrayMeasureFdcDeglitchCodeValid(uint8_t deglitchCode);
Fdc2214CapDeglitch_t sensorarrayMeasureSelectedFdcDeglitch(const sensorarrayFdcDeviceState_t *fdcState);
uint8_t sensorarrayMeasureFdcRegForChannel(uint8_t base, Fdc2214CapChannel_t channel);
uint32_t sensorarrayMeasureFdcDeglitchBandwidthHz(uint8_t deglitchCode);
uint8_t sensorarrayMeasureFdcSafeDefaultDeglitch(void);
bool sensorarrayMeasureAppliedRowConfigMatches(const sensorarrayFdcAppliedRowConfig_t *applied, const sensorarrayFdcAppliedRowConfig_t *expected);
uint16_t sensorarrayMeasureFdcBuildMuxConfig(uint8_t deglitchCode);
uint16_t sensorarrayMeasureFdcConfigBaseWithoutSleep(const sensorarrayFdcDeviceState_t *fdcState);
uint32_t sensorarrayMeasureFdcConfigFingerprint(const sensorarrayFdcAppliedRowConfig_t *config);
sensorarrayFdcCellConfigCache_t *sensorarrayMeasureFdcRowDeviceCache(sensorarrayState_t *state, uint8_t row, sensorarrayFdcDeviceId_t devId, uint8_t ch, uint8_t *outDIndex);
uint8_t sensorarrayFdcMergeDeglitchForRowDevice(sensorarrayState_t *state, uint8_t row, sensorarrayFdcDeviceId_t devId, uint8_t safeDefaultDeglitch);
bool sensorarrayMeasureFdcAutoscanConfigLooksCurrent(const sensorarrayFdcDeviceState_t *fdcState, uint8_t expectedDeglitch);
uint32_t sensorarrayMeasureFdcEstimateAutoscanReadyTimeoutUsWithSnapshot(const sensorarrayFdcRuntimeChannelConfig_t configs[4], uint8_t requiredUnreadMask, uint32_t *outEstimatedRoundUs, sensorarrayFdcProfileSnapshot_t *snapshot);
const char *sensorarrayMeasureFdcDeviceToken(sensorarrayFdcDeviceId_t devId);
esp_err_t sensorarrayMeasureVerifyFdcChannelConfigApplied(sensorarrayFdcDeviceState_t *fdcState);
esp_err_t sensorarrayMeasureEnsureFdcAutoscan4ch(sensorarrayState_t *state, sensorarrayFdcDeviceId_t devId);
esp_err_t sensorarrayMeasureWaitFdcAutoscanFrameReady(sensorarrayFdcDeviceState_t *fdcState, uint8_t row, uint32_t timeoutMs, uint16_t *outStatus);
void sensorarrayMeasurePollFdcReady(sensorarrayFdcDeviceState_t *fdcState, sensorarrayFdcReadyState_t *ready);
esp_err_t sensorarrayMeasureWaitBothFdcAutoscanFrameReady(sensorarrayState_t *state, uint8_t row, uint32_t timeoutMs, sensorarrayFdcReadyState_t *primaryReady, sensorarrayFdcReadyState_t *secondaryReady);
const char *sensorarrayMeasureFdcReadyKindName(sensorarrayFdcReadyKind_t kind);
const char *sensorarrayMeasureFdcReadyResultName(sensorarrayFdcReadyResult_t result);
sensorarrayFdcWorkerContext_t *sensorarrayMeasureFdcWorkerContext(sensorarrayFdcDeviceId_t devId);
void sensorarrayMeasureRecordFirstErr(esp_err_t err, esp_err_t *firstErr);
esp_err_t sensorarrayMeasureFdcFormalPrecheckDevice(sensorarrayState_t *state, sensorarrayFdcDeviceId_t devId);
esp_err_t sensorarrayMeasureRunFdcFormalPrecheck(sensorarrayState_t *state);
void sensorarrayMeasureFdcIntbIsr(void *arg);
esp_err_t sensorarrayMeasureEnsureFdcIntb(sensorarrayFdcWorkerContext_t *ctx);
void sensorarrayMeasureFdcPrepareIntbEpoch(sensorarrayFdcWorkerContext_t *ctx, uint32_t epochId);
uint32_t sensorarrayMeasureFdcWorkerEdgeCount(sensorarrayFdcDeviceId_t devId);
bool sensorarrayMeasureResolveWorkerCore(const char *workerName, int configuredCore, int defaultCore, BaseType_t *outCore, bool *outPinned, const char **outReason);
void sensorarrayMeasureCleanupFdcWorkers(void);
bool sensorarrayMeasureFdcReadyResultIsSoftInvalid(const sensorarrayFdcReadyState_t *ready);
uint32_t sensorarrayMeasureFdcReadyGuardDeadlineUs(uint32_t estimatedRoundUs);
void sensorarrayMeasureFillFdcDeviceI2cDelta(const Fdc2214CapI2cStats_t *before, const Fdc2214CapI2cStats_t *after, sensorarrayFdcDeviceTiming_t *timing);
esp_err_t sensorarrayMeasureReadFdcMatrixFrame(sensorarrayState_t *state, sensorarrayFdcMatrixFrame_t *outFrame);
esp_err_t sensorarrayMeasureReadFdcMatrixFrameRows(sensorarrayState_t *state, sensorarrayFdcMatrixFrame_t *outFrame, uint8_t requestedRows);
esp_err_t sensorarrayMeasureReadFdcMatrixFrameSnapshot(sensorarrayState_t *state, sensorarrayFdcMatrixFrame_t *outFrame, const sensorarrayFrameConfigSnapshot_t *snapshot);
bool sensorarrayFastSpeedIsEnabled(void);
void sensorarrayFastSpeedSetEnabled(bool enabled);
void sensorarrayMeasureFdcProfileSetSummary(bool enabled);
void sensorarrayMeasureFdcProfileSetRow(bool enabled);
void sensorarrayMeasureFdcProfileSetDevice(bool enabled);
void sensorarrayMeasureFdcProfileSetSummaryEvery(uint32_t everyNFrames);
bool sensorarrayMeasureFdcProfileSummaryEnabled(void);
bool sensorarrayMeasureFdcProfileRowEnabled(void);
bool sensorarrayMeasureFdcProfileDeviceEnabled(void);
uint32_t sensorarrayMeasureFdcProfileSummaryEvery(void);
esp_err_t sensorarrayMeasureFdcSetDiscardFrames(uint8_t discardFrames);
uint8_t sensorarrayMeasureFdcDiscardFrames(void);

void sensorarrayMeasureRuntimeConfigsFromApplied(sensorarrayState_t *state, sensorarrayFdcDeviceId_t devId, sensorarrayFdcRuntimeChannelConfig_t configs[4]);
uint32_t sensorarrayMeasureFdcTheoreticalFrameFpsX100(uint32_t roundUs);
void sensorarrayMeasureLogFdcPfuDiag(sensorarrayFdcDeviceId_t devId, uint8_t row, const char *action, uint32_t roundUs, uint32_t targetRoundUs, uint32_t rowBudgetUs, const uint16_t rCount[4], const uint16_t settleCount[4]);
void sensorarrayMeasureLogFdcPfuFastProfile(sensorarrayFdcDeviceId_t devId, uint8_t row, const char *action, uint32_t oldRoundUs, uint32_t newRoundUs, uint32_t targetRoundUs, uint32_t rowBudgetUs, const uint16_t oldRcount[4], const uint16_t newRcount[4], const uint16_t oldSettle[4], const uint16_t newSettle[4], const char *decisionReason, uint32_t diffWriteCount);
void sensorarrayMeasureLogFdcPfuApplyResult(sensorarrayFdcDeviceId_t devId, uint8_t row, const char *why, const char *action, uint32_t oldRoundUs, uint32_t newRoundUs, uint32_t targetRoundUs, uint32_t rowBudgetUs, uint16_t oldDriveCurrent0, uint16_t newDriveCurrent0, bool profileTooSlow, uint32_t diffWriteCount);
bool sensorarrayMeasurePromoteFdcExpectedProfileToCellCache(sensorarrayState_t *state, uint8_t row, sensorarrayFdcDeviceId_t devId, sensorarrayFdcAppliedRowConfig_t *expected, uint32_t effectiveFclkHz);
esp_err_t sensorarrayMeasureApplyFdcCachedRowConfig(sensorarrayState_t *state, uint8_t row, sensorarrayFdcDeviceId_t devId, const char *reason, bool forceWrite, sensorarrayFdcDeviceTiming_t *timing);
bool sensorarrayFdcRead4IsDataCompleteGood(const sensorarrayFdcDeviceRead4Result_t *read4, uint8_t reqMask);
bool sensorarrayMeasureFdcReadyStatusSaysReadable(const sensorarrayFdcReadyState_t *ready, uint8_t reqMask);
bool sensorarrayMeasureFdcReadyAllowsNormalDataRead(const sensorarrayFdcReadyState_t *ready, uint8_t reqMask);
esp_err_t sensorarrayMeasureReadFdcAutoscan4chMasked(sensorarrayFdcDeviceState_t *fdcState, uint8_t freshMask4, const sensorarrayFdcReadyState_t *ready, sensorarrayFdcAutoscanSamples_t *outSamples);
esp_err_t sensorarrayMeasureReadFdcAutoscan4ch(sensorarrayFdcDeviceState_t *fdcState, sensorarrayFdcAutoscanSamples_t *outSamples);
void sensorarrayMeasureMarkFdcNoFreshSamples(sensorarrayFdcAutoscanSamples_t *outSamples, const sensorarrayFdcReadyState_t *ready, bool i2cError);
const char *sensorarrayMeasureFdcRead4DiagnosticName(const sensorarrayFdcDeviceRead4Result_t *read4);
void sensorarrayMeasureBuildFdcRead4Result(uint8_t row, uint32_t epochId, sensorarrayFdcDeviceId_t devId, const sensorarrayFdcAutoscanSamples_t *samples, const sensorarrayFdcRuntimeChannelConfig_t configs[4], const sensorarrayFdcReadyState_t *ready, esp_err_t readErr, uint32_t readUs, bool logNormal, sensorarrayFdcDeviceRead4Result_t *outRead4);
bool sensorarrayMeasureFdcDeviceReadyForIo(const sensorarrayFdcDeviceState_t *fdcState);
void sensorarrayMeasureLogFdcSecondaryUnavailableOnce(void);
esp_err_t sensorarrayMeasureFdcSetSleepMode(sensorarrayFdcDeviceState_t *fdcState, bool enable, sensorarrayFdcDeviceTiming_t *timing);
const char *sensorarrayMeasureFdcRrSequenceName(uint8_t rrSequence);
esp_err_t sensorarrayMeasureFdcVerifySleepExit(sensorarrayState_t *state, uint8_t row, uint32_t epochId, sensorarrayFdcDeviceId_t devId, sensorarrayFdcDeviceTiming_t *timing);
bool sensorarrayMeasureFdcShouldLogNormalFrame(uint32_t frameSeq);
bool sensorarrayMeasureFdcShouldLogNormalPollSuccess(uint32_t frameSeq);
bool sensorarrayMeasureFdcRead4IsFullSuccess(const sensorarrayFdcDeviceRead4Result_t *read4);
bool sensorarrayMeasureFdcWorkerResultIsGood(const sensorarrayFdcWorkerResult_t *result, uint8_t expectedRow, uint32_t expectedEpoch, sensorarrayFdcDeviceId_t expectedDev);
void sensorarrayMeasureFdcFrameTrackerNoteRead(sensorarrayFdcFrameReadTracker_t *tracker, uint8_t row, uint32_t epochId, sensorarrayFdcDeviceId_t devId, const sensorarrayFdcDeviceRead4Result_t *read4, const char *reason);
uint32_t sensorarrayMeasureClampU32(uint32_t value, uint32_t minValue, uint32_t maxValue);
uint32_t sensorarrayMeasureFdcCeilDivU64(uint64_t numerator, uint64_t denominator);
uint32_t sensorarrayMeasureFdcEstimateAutoscanReadyTimeoutUsFromConfigs(const sensorarrayFdcRuntimeChannelConfig_t configs[4], uint8_t requiredUnreadMask, uint32_t *outEstimatedRoundUs);
uint32_t sensorarrayMeasureFdcEstimateAppliedRowReadyTimeoutUs(const sensorarrayState_t *state, sensorarrayFdcDeviceId_t devId, uint8_t requiredUnreadMask, uint32_t *outEstimatedRoundUs);
const char *sensorarrayMeasureFdcReadyDiagnosticName(const sensorarrayFdcReadyState_t *ready, uint8_t requiredUnreadMask);
void sensorarrayMeasureFdcUpdateReadyTiming(const sensorarrayFdcReadyState_t *ready, sensorarrayFdcDeviceTiming_t *timing, bool fallbackAttempted);
sensorarrayFdcReadyPolicy_t sensorarrayMeasureFdcConfiguredReadyPolicy(void);
sensorarrayFdcReadyPolicy_t sensorarrayMeasureFdcReadyPolicyForDevice(sensorarrayFdcDeviceId_t devId, const sensorarrayFdcWorkerContext_t *ctx);
const char *sensorarrayMeasureFdcReadyPolicyName(sensorarrayFdcReadyPolicy_t policy);
const char *sensorarrayMeasureFdcWatchdogReasonName(sensorarrayFdcWatchdogReason_t reason);
uint32_t sensorarrayMeasureFdcRowDeviceWatchdogHardTimeoutUs(void);
esp_err_t sensorarrayFdcHandleRowDeviceWatchdog(sensorarrayState_t *state, sensorarrayFdcDeviceId_t devId, uint8_t row, uint32_t epochId, sensorarrayFdcWatchdogReason_t reason, sensorarrayFdcWorkerResult_t *result, sensorarrayFdcDeviceTiming_t *timing);
TickType_t sensorarrayMeasureFdcTicksForUs(uint32_t waitUs);
uint32_t sensorarrayMeasureFdcRemainingDeadlineUs(uint64_t deadlineUs);
const char *sensorarrayMeasureFdcEstimateKindName(uint8_t requiredUnreadMask);
uint32_t sensorarrayMeasureFdcAddSaturateU32(uint32_t a, uint32_t b);
uint32_t sensorarrayMeasureFdcComputeActualIntbWaitUs(uint32_t estimatedRoundUs, uint64_t rowDeviceDeadlineUs, sensorarrayFdcReadyState_t *ready);
void sensorarrayMeasureFdcDrainCurrentTaskNotify(void);
void sensorarrayMeasureFdcArmCurrentTaskForIntb(sensorarrayFdcWorkerContext_t *ctx, uint32_t epochId);
const char *sensorarrayMeasureFdcReadyPollToken(const sensorarrayFdcReadyState_t *ready, uint8_t requiredUnreadMask);
void sensorarrayMeasureFdcLogReadyCounts(sensorarrayFdcDeviceId_t devId, uint8_t row, uint32_t epochId, const sensorarrayFdcReadyState_t *ready);
esp_err_t sensorarrayMeasureFdcReadStatusAndAckIntbForReady(sensorarrayFdcDeviceState_t *fdcState, sensorarrayFdcDeviceId_t devId, uint8_t row, uint32_t epochId, uint8_t requiredUnreadMask, bool afterIntb, bool afterIntbRecheck, bool fallback, bool hardTimeout, bool preWaitCheck, bool logNormalSuccess, sensorarrayFdcReadyState_t *ready, sensorarrayFdcDeviceTiming_t *timing, bool *bestPartialSeen, sensorarrayFdcReadyDecoded_t *bestPartial);
esp_err_t sensorarrayMeasureFdcReadStatusForWatchdogDiagOnly(sensorarrayFdcDeviceState_t *fdcState, sensorarrayFdcDeviceId_t devId, uint8_t row, uint32_t epochId, const char *reason, sensorarrayFdcReadyState_t *ready, sensorarrayFdcDeviceTiming_t *timing);
bool sensorarrayMeasureFdcReadyIsUnreadFullNoDrdy(const sensorarrayFdcReadyState_t *ready, uint8_t requiredUnreadMask);
bool sensorarrayMeasureFdcClassifyUnreadFullNoDrdySoft(sensorarrayFdcDeviceId_t devId, uint8_t row, uint32_t epochId, const char *source, sensorarrayFdcReadyState_t *ready);
esp_err_t sensorarrayFdcWaitDeviceReady(sensorarrayState_t *state, sensorarrayFdcDeviceId_t devId, uint8_t row, uint32_t epochId, uint32_t frameSeq, uint8_t requiredUnreadMask, uint32_t timeoutUs, uint32_t estimatedRoundUs, uint64_t rowDeviceDeadlineUs, bool allowDirectData, sensorarrayFdcReadyState_t *ready, sensorarrayFdcDeviceTiming_t *timing);
esp_err_t sensorarrayMeasureFdcDeviceLevelResync(sensorarrayState_t *state, uint8_t row, uint32_t epochId, uint32_t frameSeq, sensorarrayFdcDeviceId_t devId, const sensorarrayFdcReadyState_t *readyBefore, sensorarrayFdcReadyState_t *readyAfter, sensorarrayFdcDeviceTiming_t *timing);
esp_err_t sensorarrayMeasureFdcPrepareDeviceWhileSleeping(sensorarrayState_t *state, uint8_t row, uint32_t epochId, uint32_t frameSeq, sensorarrayFdcDeviceId_t devId, sensorarrayFdcRuntimeChannelConfig_t outConfigs[4], sensorarrayFdcDeviceTiming_t *timing);
esp_err_t sensorarrayMeasureFdcDrainStaleUnread(sensorarrayFdcDeviceState_t *fdcState, sensorarrayFdcDeviceId_t devId, uint8_t row, uint32_t epochId, const sensorarrayFdcReadyState_t *ready, sensorarrayFdcDeviceTiming_t *timing);
esp_err_t sensorarrayMeasureFdcRunDeviceEpochAfterSleep(sensorarrayState_t *state, uint8_t row, uint32_t epochId, uint32_t frameSeq, sensorarrayFdcDeviceId_t devId, sensorarrayFdcAutoscanSamples_t *outSamples, sensorarrayFdcRuntimeChannelConfig_t outConfigs[4], sensorarrayFdcReadyState_t *outReady, sensorarrayFdcDeviceRead4Result_t *outRead4, sensorarrayFdcDeviceTiming_t *timing, sensorarrayFdcFrameReadTracker_t *readTracker, const char *readReason, uint64_t rowDeviceDeadlineOverrideUs);
EventBits_t sensorarrayMeasureFdcWorkerAckBit(sensorarrayFdcDeviceId_t devId);
EventBits_t sensorarrayMeasureFdcWorkerDoneBit(sensorarrayFdcDeviceId_t devId);
void sensorarrayMeasureFdcWorkerTask(void *arg);
esp_err_t sensorarrayMeasureEnsureFdcWorkers(void);
void sensorarrayMeasureAccumulateRowEpochTiming(sensorarrayFdcTimingSummary_t *summary, const sensorarrayFdcRowTiming_t *rowTiming, const sensorarrayFdcDeviceTiming_t *primaryTiming, const sensorarrayFdcDeviceTiming_t *secondaryTiming);
esp_err_t sensorarrayMeasureSelectFdcRowWhileSleeping(sensorarrayState_t *state, uint8_t row, uint32_t epochId, uint32_t frameSeq, sensorarrayFdcRowTiming_t *rowTiming);
esp_err_t sensorarrayMeasureReadFdcMatrixRowSerialEpoch(sensorarrayState_t *state, uint8_t row, uint32_t epochId, uint32_t frameSeq, sensorarrayFdcAutoscanSamples_t *primarySamples, sensorarrayFdcAutoscanSamples_t *secondarySamples, sensorarrayFdcRuntimeChannelConfig_t runtimeConfigs[2][4], sensorarrayFdcDeviceTiming_t *primaryTiming, sensorarrayFdcDeviceTiming_t *secondaryTiming, sensorarrayFdcRowTiming_t *rowTiming, sensorarrayFdcFrameReadTracker_t *readTracker);
void sensorarrayMeasureFdcLogWorkerTimeout(const sensorarrayFdcWorkerTrace_t *trace, const char *timeoutStage, const sensorarrayFdcWorkerResult_t *result);
void sensorarrayMeasureFdcLogLateDone(const sensorarrayFdcWorkerTrace_t *trace);
void sensorarrayMeasureFdcDiscardWorkerResult(sensorarrayFdcWorkerResult_t *result, sensorarrayFdcAutoscanSamples_t *samples, sensorarrayFdcWorkerTrace_t *trace, const char *reason);
void sensorarrayMeasureFdcDeferDeviceRepairAfterParallel(sensorarrayState_t *state, uint8_t row, uint32_t epochId, uint32_t frameSeq, sensorarrayFdcDeviceId_t devId, sensorarrayFdcAutoscanSamples_t *samples, const sensorarrayFdcWorkerResult_t *result, sensorarrayFdcRowTiming_t *rowTiming, const char *reason);
uint32_t sensorarrayMeasureFdcAbsDeltaUs(uint64_t a, uint64_t b);
uint64_t sensorarrayMeasureFdcSpanUs(uint64_t startA, uint64_t doneA, uint64_t startB, uint64_t doneB);
void sensorarrayMeasureFdcFinalizeParallelTiming(uint8_t row, uint32_t epochId, const sensorarrayFdcDeviceTiming_t *primaryTiming, const sensorarrayFdcDeviceTiming_t *secondaryTiming, sensorarrayFdcRowTiming_t *rowTiming, esp_err_t primaryErr, esp_err_t secondaryErr);
esp_err_t sensorarrayMeasureReadFdcMatrixRowParallelEpoch(sensorarrayState_t *state, uint8_t row, uint32_t epochId, uint32_t frameSeq, sensorarrayFdcAutoscanSamples_t *primarySamples, sensorarrayFdcAutoscanSamples_t *secondarySamples, sensorarrayFdcRuntimeChannelConfig_t runtimeConfigs[2][4], sensorarrayFdcDeviceTiming_t *primaryTiming, sensorarrayFdcDeviceTiming_t *secondaryTiming, sensorarrayFdcRowTiming_t *rowTiming, sensorarrayFdcFrameReadTracker_t *readTracker, bool *outWorkerTimeout);
esp_err_t sensorarrayMeasureReadFdcRuntimeChannelConfigs(sensorarrayState_t *state, sensorarrayFdcRuntimeChannelConfig_t configs[2][4]);
esp_err_t sensorarrayMeasureDiscardFdcAutoscanRow(sensorarrayState_t *state, uint8_t sIndex, sensorarrayFdcDeviceId_t devId, uint8_t discardCount, const char *reason);

void sensorarrayMeasureMarkFdcMatrixCellEx(sensorarrayFdcMatrixFrame_t *frame, uint8_t sIndex, uint8_t dIndex, uint32_t raw28, double freqHz, const sensorarrayFdcRuntimeChannelConfig_t *config, bool fresh, bool valid, bool warning, bool error, bool notReady, bool zeroBeforeReady, bool zeroAfterDrdy, bool i2cError);
void sensorarrayMeasureFillFdcMatrixRow(sensorarrayFdcMatrixFrame_t *frame, uint8_t sIndex, const sensorarrayFdcAutoscanSamples_t *primary, const sensorarrayFdcAutoscanSamples_t *secondary, const sensorarrayFdcRuntimeChannelConfig_t configs[2][4], uint8_t *outValidMask8, uint8_t *outWarnMask8, uint8_t *outErrorMask8);
void sensorarrayMeasureAccumulateFdcHealth(sensorarrayFdcFrameHealth_t *health, uint8_t sIndex, const sensorarrayFdcAutoscanSamples_t *primary, const sensorarrayFdcAutoscanSamples_t *secondary, const sensorarrayFdcRuntimeChannelConfig_t configs[2][4], const sensorarrayFdcMatrixFrame_t *frame);
uint32_t sensorarrayMeasureUpdateFdcRuntimeProfiles(sensorarrayState_t *state, const sensorarrayFdcFrameHealth_t *health, uint8_t activeRows);
uint32_t sensorarrayMeasureCountFdcFrameWarnings(const sensorarrayFdcFrameHealth_t *health, sensorarrayFdcTimingSummary_t *timing, uint8_t activeRows);
void sensorarrayMeasureInitFdcMatrixFrame(sensorarrayFdcMatrixFrame_t *frame);
bool sensorarrayFdcMatrixFrameRawAllZero(const sensorarrayFdcMatrixFrame_t *frame);
uint64_t sensorarrayMeasureComputeFdcFrameCapTotalPf(sensorarrayFdcMatrixFrame_t *frame);
esp_err_t sensorarrayMeasureCheckFdcMatrixReady(sensorarrayState_t *state);

uint64_t sensorarrayMeasureElapsedUs(int64_t startUs);
uint64_t sensorarrayMeasureEstimateI2cBits(uint32_t writeCount, uint32_t readCount, uint32_t writeBytes, uint32_t readBytes);
void sensorarrayMeasureMergeFdcI2cStats(const Fdc2214CapI2cStats_t *primaryStats, const Fdc2214CapI2cStats_t *secondaryStats, const BoardSupportI2cBusInfo_t *primaryBus, const BoardSupportI2cBusInfo_t *secondaryBus, sensorarrayFdcTimingSummary_t *summary);
void sensorarrayMeasurePrintFdcTimingSummary(const sensorarrayFdcTimingSummary_t *summary, uint32_t sequence);
uint8_t sensorarrayMeasureFdcProfileSlowestChannel(const sensorarrayFdcProfileSnapshot_t *snapshot);
void sensorarrayMeasureFdcPrintProfileSnapshotDetail(uint32_t sequence, uint8_t row, sensorarrayFdcDeviceId_t devId, const sensorarrayFdcProfileSnapshot_t *snapshot, const char *reason);
void sensorarrayMeasurePrintFdcProfileSummary(uint32_t sequence);
uint64_t sensorarrayMeasureAvgU64(uint64_t total, uint32_t count);
void sensorarrayMeasurePrintFdcTimingAggregate(const sensorarrayFdcTimingAggregate_t *agg);
void sensorarrayMeasureUpdateFdcTimingAggregate(const sensorarrayFdcTimingSummary_t *summary, uint32_t sequence);
void sensorarrayMeasurePrintFdcBottleneck(const sensorarrayFdcTimingSummary_t *summary, uint32_t sequence);
void sensorarrayMeasurePrintFdcRowTiming(uint32_t sequence, const sensorarrayFdcRowTiming_t *rowTiming);
void sensorarrayMeasurePrintFdcDeviceTiming(uint32_t sequence, const sensorarrayFdcDeviceTiming_t *deviceTiming);

const char *sensorarrayMeasureFdcProfileTooSlowActionName(void);
sensorarrayFdcSampleStatus_t sensorarrayMeasureMapFdcStatus(Fdc2214CapSampleStatus_t sampleStatus);
const char *sensorarrayMeasureFdcSampleStatusName(sensorarrayFdcSampleStatus_t status);
typedef esp_err_t (*sensorarrayFdcReadSampleFn_t)(Fdc2214CapDevice_t *dev, Fdc2214CapChannel_t ch, Fdc2214CapSample_t *outSample);
esp_err_t sensorarrayMeasureReadFdcSampleDiagWithReader(sensorarrayFdcReadSampleFn_t readFn, Fdc2214CapDevice_t *dev, Fdc2214CapChannel_t ch, bool discardFirst, bool idOk, bool configOk, bool relaxedMode, sensorarrayFdcReadDiag_t *outDiag);
esp_err_t sensorarrayMeasureReadFdcSampleDiag(Fdc2214CapDevice_t *dev, Fdc2214CapChannel_t ch, bool discardFirst, bool idOk, bool configOk, sensorarrayFdcReadDiag_t *outDiag);
esp_err_t sensorarrayMeasureReadFdcSampleDiagRelaxed(Fdc2214CapDevice_t *dev, Fdc2214CapChannel_t ch, bool discardFirst, bool idOk, bool configOk, sensorarrayFdcReadDiag_t *outDiag);
esp_err_t sensorarrayMeasureReadFdcSample(Fdc2214CapDevice_t *dev, Fdc2214CapChannel_t ch, bool discardFirst, Fdc2214CapSample_t *outSample);
double sensorarrayMeasureFdcRawToFrequencyHz(uint32_t raw28, uint32_t refClockHz);
sensorarrayFdcRefClockSource_t sensorarrayMeasureFdcEffectiveRefClockSource(void);
uint32_t sensorarrayMeasureFdcEffectiveFclkHz(void);
bool sensorarrayMeasureFdcDecodeClockDividers(uint16_t clockDividers, uint8_t *outFinSelCode, uint8_t *outFinFactor, uint16_t *outFrefDivider, const char **outStatus);
double sensorarrayMeasureFdcFinFactorFromClockDiv(uint16_t clockDividers);
double sensorarrayMeasureFdcFrefDividerFromClockDiv(uint16_t clockDividers);
double sensorarrayMeasureFdcRaw28ToFreqHz(uint32_t raw28, uint32_t effectiveFclkHz, uint16_t clockDividers);
bool sensorarrayMeasureFdcComputeFrequencyDiag(uint32_t raw28, uint16_t clockDividers, sensorarrayFdcFrequencyDiag_t *outDiag);
double sensorarrayMeasureFdcRawToSensorFrequencyHz(uint32_t raw28, uint16_t clockDividers);
bool sensorarrayMeasureFdcTryCapacitancePf(double frequencyHz, uint32_t inductorUh, double *outCapPf);
bool sensorarrayMeasureFdcComputeCapacitancePf(double frequencyHz, double inductorValueUh, double *outCapPf);

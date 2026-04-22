#include "sensorarrayRecovery.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
#include "esp_rom_sys.h"
#include "esp_system.h"
#include "esp_task_wdt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "sensorarrayConfig.h"

#ifndef CONFIG_SENSORARRAY_DEBUG_RESTART_DELAY_MS
#define CONFIG_SENSORARRAY_DEBUG_RESTART_DELAY_MS 200
#endif

#ifndef CONFIG_SENSORARRAY_DEBUG_PROGRESS_WATCHDOG_ENABLE
#define CONFIG_SENSORARRAY_DEBUG_PROGRESS_WATCHDOG_ENABLE 1
#endif

#ifndef CONFIG_SENSORARRAY_DEBUG_PROGRESS_WATCHDOG_TIMEOUT_MS
#define CONFIG_SENSORARRAY_DEBUG_PROGRESS_WATCHDOG_TIMEOUT_MS 2000
#endif

#ifndef CONFIG_SENSORARRAY_DEBUG_PROGRESS_WATCHDOG_MONITOR_PERIOD_MS
#define CONFIG_SENSORARRAY_DEBUG_PROGRESS_WATCHDOG_MONITOR_PERIOD_MS 150
#endif

#ifndef CONFIG_SENSORARRAY_DEBUG_TASK_WDT_ENABLE
#define CONFIG_SENSORARRAY_DEBUG_TASK_WDT_ENABLE 1
#endif

#ifndef CONFIG_SENSORARRAY_DEBUG_TASK_WDT_TIMEOUT_MS
#define CONFIG_SENSORARRAY_DEBUG_TASK_WDT_TIMEOUT_MS 4000
#endif

#ifndef CONFIG_SENSORARRAY_DEBUG_TASK_WDT_PANIC
#define CONFIG_SENSORARRAY_DEBUG_TASK_WDT_PANIC 0
#endif

#ifndef CONFIG_SENSORARRAY_DEBUG_REBOOT_BACKOFF_MS
#define CONFIG_SENSORARRAY_DEBUG_REBOOT_BACKOFF_MS 2000
#endif

#ifndef CONFIG_SENSORARRAY_DEBUG_MAX_REBOOT_STREAK_BEFORE_COOLDOWN
#define CONFIG_SENSORARRAY_DEBUG_MAX_REBOOT_STREAK_BEFORE_COOLDOWN 6
#endif

#ifndef CONFIG_SENSORARRAY_DEBUG_CHECKPOINT_GPIO
#define CONFIG_SENSORARRAY_DEBUG_CHECKPOINT_GPIO -1
#endif

#ifndef CONFIG_SENSORARRAY_DEBUG_CHECKPOINT_PULSE_US
#define CONFIG_SENSORARRAY_DEBUG_CHECKPOINT_PULSE_US 250
#endif

#define SENSORARRAY_RECOVERY_RTC_MAGIC 0x53525243u
#define SENSORARRAY_RECOVERY_RTC_VERSION 1u
#define SENSORARRAY_RECOVERY_REASON_MAX_LEN 31u
#define SENSORARRAY_RECOVERY_MONITOR_STACK_WORDS 768u
#define SENSORARRAY_RECOVERY_MONITOR_PRIO 2u
#define SENSORARRAY_RECOVERY_DEFAULT_HEARTBEAT_MS 1000u

typedef struct {
    uint32_t magic;
    uint32_t version;
    uint32_t rebootCounter;
    uint32_t rebootStreak;
    uint32_t lastStage;
    uint32_t lastCause;
    int32_t lastErr;
    uint8_t lastSColumn;
    uint8_t lastDLine;
    uint16_t reserved;
    uint32_t lastTick;
    uint32_t lastBootReason;
    char lastReason[SENSORARRAY_RECOVERY_REASON_MAX_LEN + 1u];
} sensorarrayRecoveryRtcContext_t;

typedef struct {
    bool initialized;
    bool s5d5Mode;

    bool checkpointConfigured;
    bool checkpointEnabled;
    gpio_num_t checkpointGpio;
    uint32_t checkpointPulseUs;

    bool progressEnabled;
    uint32_t progressTimeoutMs;
    uint32_t progressMonitorPeriodMs;
    volatile uint32_t lastKickTick;
    volatile sensorarrayRecoveryStage_t lastKickStage;
    volatile uint8_t lastKickSColumn;
    volatile uint8_t lastKickDLine;
    TaskHandle_t monitorTask;

    bool restartPending;

    bool taskWdtEnabled;
    bool taskWdtConfigured;
    bool taskWdtTaskRegistered;
    TaskHandle_t taskWdtTaskHandle;
} sensorarrayRecoveryRuntime_t;

static RTC_DATA_ATTR sensorarrayRecoveryRtcContext_t s_rtcCtx = {0};
static sensorarrayRecoveryRuntime_t s_runtime = {0};
static portMUX_TYPE s_recoveryMux = portMUX_INITIALIZER_UNLOCKED;

static const char *sensorarrayRecoveryResetReasonName(esp_reset_reason_t reason)
{
    switch (reason) {
    case ESP_RST_UNKNOWN:
        return "unknown";
    case ESP_RST_POWERON:
        return "power_on";
    case ESP_RST_EXT:
        return "external_pin";
    case ESP_RST_SW:
        return "software";
    case ESP_RST_PANIC:
        return "panic";
    case ESP_RST_INT_WDT:
        return "int_wdt";
    case ESP_RST_TASK_WDT:
        return "task_wdt";
    case ESP_RST_WDT:
        return "other_wdt";
    case ESP_RST_DEEPSLEEP:
        return "deepsleep";
    case ESP_RST_BROWNOUT:
        return "brownout";
    case ESP_RST_SDIO:
        return "sdio";
    case ESP_RST_USB:
        return "usb";
    case ESP_RST_JTAG:
        return "jtag";
    case ESP_RST_EFUSE:
        return "efuse";
    case ESP_RST_PWR_GLITCH:
        return "power_glitch";
    case ESP_RST_CPU_LOCKUP:
        return "cpu_lockup";
    default:
        return "unmapped";
    }
}

const char *sensorarrayRecoveryFailActionName(sensorarrayRecoveryFailAction_t action)
{
    switch (action) {
    case SENSORARRAY_RECOVERY_FAIL_ACTION_IDLE:
        return "idle";
    case SENSORARRAY_RECOVERY_FAIL_ACTION_RESTART:
        return "restart";
    case SENSORARRAY_RECOVERY_FAIL_ACTION_PANIC:
        return "panic";
    default:
        return "unknown";
    }
}

const char *sensorarrayRecoveryCauseName(sensorarrayRecoveryCause_t cause)
{
    switch (cause) {
    case SENSORARRAY_RECOVERY_CAUSE_NONE:
        return "none";
    case SENSORARRAY_RECOVERY_CAUSE_EXPLICIT_FATAL:
        return "explicit_fatal";
    case SENSORARRAY_RECOVERY_CAUSE_PROGRESS_STALL:
        return "progress_stall";
    default:
        return "unknown";
    }
}

const char *sensorarrayRecoveryStageName(sensorarrayRecoveryStage_t stage)
{
    switch (stage) {
    case SENSORARRAY_RECOVERY_STAGE_UNKNOWN:
        return "unknown";
    case SENSORARRAY_RECOVERY_STAGE_BOOT:
        return "boot";
    case SENSORARRAY_RECOVERY_STAGE_APP_INIT:
        return "app_init";
    case SENSORARRAY_RECOVERY_STAGE_FDC_INIT_BEGIN:
        return "fdc_init_begin";
    case SENSORARRAY_RECOVERY_STAGE_FDC_INIT_END:
        return "fdc_init_end";
    case SENSORARRAY_RECOVERY_STAGE_ROUTE_ROW_ENTER:
        return "route_row_enter";
    case SENSORARRAY_RECOVERY_STAGE_ROUTE_ROW_EXIT:
        return "route_row_exit";
    case SENSORARRAY_RECOVERY_STAGE_ROUTE_SELA_ENTER:
        return "route_sela_enter";
    case SENSORARRAY_RECOVERY_STAGE_ROUTE_SELA_EXIT:
        return "route_sela_exit";
    case SENSORARRAY_RECOVERY_STAGE_ROUTE_SELB_ENTER:
        return "route_selb_enter";
    case SENSORARRAY_RECOVERY_STAGE_ROUTE_SELB_EXIT:
        return "route_selb_exit";
    case SENSORARRAY_RECOVERY_STAGE_ROUTE_SW_ENTER:
        return "route_sw_enter";
    case SENSORARRAY_RECOVERY_STAGE_ROUTE_SW_EXIT:
        return "route_sw_exit";
    case SENSORARRAY_RECOVERY_STAGE_ROUTE_VERIFY_ENTER:
        return "route_verify_enter";
    case SENSORARRAY_RECOVERY_STAGE_ROUTE_VERIFY_EXIT:
        return "route_verify_exit";
    case SENSORARRAY_RECOVERY_STAGE_SWEEP_CANDIDATE_BEGIN:
        return "sweep_candidate_begin";
    case SENSORARRAY_RECOVERY_STAGE_SWEEP_CANDIDATE_END:
        return "sweep_candidate_end";
    case SENSORARRAY_RECOVERY_STAGE_LOCKED_SAMPLE_BEGIN:
        return "locked_sample_begin";
    case SENSORARRAY_RECOVERY_STAGE_LOCKED_SAMPLE_END:
        return "locked_sample_end";
    case SENSORARRAY_RECOVERY_STAGE_RECOVERY_BEGIN:
        return "recovery_begin";
    case SENSORARRAY_RECOVERY_STAGE_RESTART_PENDING:
        return "restart_pending";
    default:
        return "unknown";
    }
}

sensorarrayRecoveryFailAction_t sensorarrayRecoveryFailAction(void)
{
#if defined(CONFIG_SENSORARRAY_DEBUG_FAIL_ACTION_PANIC) && (CONFIG_SENSORARRAY_DEBUG_FAIL_ACTION_PANIC != 0)
    return SENSORARRAY_RECOVERY_FAIL_ACTION_PANIC;
#elif defined(CONFIG_SENSORARRAY_DEBUG_FAIL_ACTION_RESTART) && (CONFIG_SENSORARRAY_DEBUG_FAIL_ACTION_RESTART != 0)
    return SENSORARRAY_RECOVERY_FAIL_ACTION_RESTART;
#elif defined(CONFIG_SENSORARRAY_DEBUG_FAIL_ACTION_IDLE) && (CONFIG_SENSORARRAY_DEBUG_FAIL_ACTION_IDLE != 0)
    return SENSORARRAY_RECOVERY_FAIL_ACTION_IDLE;
#else
    return s_runtime.s5d5Mode ? SENSORARRAY_RECOVERY_FAIL_ACTION_RESTART : SENSORARRAY_RECOVERY_FAIL_ACTION_IDLE;
#endif
}

bool sensorarrayRecoveryIsS5d5Mode(void)
{
    return s_runtime.s5d5Mode;
}

static uint32_t sensorarrayRecoveryCheckpointPulseCount(sensorarrayRecoveryCheckpointEvent_t event)
{
    switch (event) {
    case SENSORARRAY_RECOVERY_CHECKPOINT_EVENT_FDC_INIT_BEGIN:
        return 1u;
    case SENSORARRAY_RECOVERY_CHECKPOINT_EVENT_FDC_INIT_OK:
        return 2u;
    case SENSORARRAY_RECOVERY_CHECKPOINT_EVENT_ROUTE_APPLIED:
        return 3u;
    case SENSORARRAY_RECOVERY_CHECKPOINT_EVENT_STEP_BEGIN:
        return 4u;
    case SENSORARRAY_RECOVERY_CHECKPOINT_EVENT_SNAPSHOT_DONE:
        return 5u;
    case SENSORARRAY_RECOVERY_CHECKPOINT_EVENT_WARNING:
        return 6u;
    case SENSORARRAY_RECOVERY_CHECKPOINT_EVENT_SWEEP_CANDIDATE_BEGIN:
        return 7u;
    case SENSORARRAY_RECOVERY_CHECKPOINT_EVENT_SWEEP_KEYREGS_BEGIN:
        return 8u;
    case SENSORARRAY_RECOVERY_CHECKPOINT_EVENT_SWEEP_KEYREGS_DONE:
        return 9u;
    case SENSORARRAY_RECOVERY_CHECKPOINT_EVENT_SWEEP_SAMPLE_BEGIN:
        return 10u;
    case SENSORARRAY_RECOVERY_CHECKPOINT_EVENT_SWEEP_SAMPLE_DONE:
        return 11u;
    case SENSORARRAY_RECOVERY_CHECKPOINT_EVENT_SWEEP_RECOVER_BEGIN:
        return 12u;
    case SENSORARRAY_RECOVERY_CHECKPOINT_EVENT_SWEEP_RECOVER_DONE:
        return 13u;
    case SENSORARRAY_RECOVERY_CHECKPOINT_EVENT_TERMINAL_FAULT:
        return 14u;
    case SENSORARRAY_RECOVERY_CHECKPOINT_EVENT_ROUTE_ROW_ENTER:
        return 15u;
    case SENSORARRAY_RECOVERY_CHECKPOINT_EVENT_ROUTE_ROW_EXIT:
        return 16u;
    case SENSORARRAY_RECOVERY_CHECKPOINT_EVENT_ROUTE_SELA_ENTER:
        return 17u;
    case SENSORARRAY_RECOVERY_CHECKPOINT_EVENT_ROUTE_SELA_EXIT:
        return 18u;
    case SENSORARRAY_RECOVERY_CHECKPOINT_EVENT_ROUTE_SELB_ENTER:
        return 19u;
    case SENSORARRAY_RECOVERY_CHECKPOINT_EVENT_ROUTE_SELB_EXIT:
        return 20u;
    case SENSORARRAY_RECOVERY_CHECKPOINT_EVENT_ROUTE_SW_ENTER:
        return 21u;
    case SENSORARRAY_RECOVERY_CHECKPOINT_EVENT_ROUTE_SW_EXIT:
        return 22u;
    case SENSORARRAY_RECOVERY_CHECKPOINT_EVENT_ROUTE_VERIFY_ENTER:
        return 23u;
    case SENSORARRAY_RECOVERY_CHECKPOINT_EVENT_ROUTE_VERIFY_EXIT:
        return 24u;
    case SENSORARRAY_RECOVERY_CHECKPOINT_EVENT_RECOVERY_BEGIN:
        return 25u;
    case SENSORARRAY_RECOVERY_CHECKPOINT_EVENT_RESTART_PENDING:
        return 26u;
    default:
        return 0u;
    }
}

static void sensorarrayRecoveryCheckpointEnsureConfigured(void)
{
    if (!s_runtime.s5d5Mode) {
        return;
    }
    if (s_runtime.checkpointConfigured) {
        return;
    }

    s_runtime.checkpointConfigured = true;
    s_runtime.checkpointGpio = GPIO_NUM_NC;
    s_runtime.checkpointPulseUs = (uint32_t)CONFIG_SENSORARRAY_DEBUG_CHECKPOINT_PULSE_US;
    if (s_runtime.checkpointPulseUs == 0u) {
        s_runtime.checkpointPulseUs = 250u;
    }

    if (CONFIG_SENSORARRAY_DEBUG_CHECKPOINT_GPIO < 0 ||
        CONFIG_SENSORARRAY_DEBUG_CHECKPOINT_GPIO >= (int)GPIO_NUM_MAX) {
        s_runtime.checkpointEnabled = false;
        printf("DBGRECOVERY,checkpoint=status_disabled,gpio=%d,reason=invalid_or_disabled\n",
               CONFIG_SENSORARRAY_DEBUG_CHECKPOINT_GPIO);
        return;
    }

    gpio_num_t checkpointGpio = (gpio_num_t)CONFIG_SENSORARRAY_DEBUG_CHECKPOINT_GPIO;
    gpio_config_t ioCfg = {
        .pin_bit_mask = (1ULL << checkpointGpio),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&ioCfg);
    if (err != ESP_OK) {
        s_runtime.checkpointEnabled = false;
        printf("DBGRECOVERY,checkpoint=status_disabled,gpio=%d,err=%ld,reason=gpio_config_failed\n",
               (int)checkpointGpio,
               (long)err);
        return;
    }

    gpio_set_level(checkpointGpio, 0);
    s_runtime.checkpointEnabled = true;
    s_runtime.checkpointGpio = checkpointGpio;
    printf("DBGRECOVERY,checkpoint=status_enabled,gpio=%d,pulseUs=%lu\n",
           (int)checkpointGpio,
           (unsigned long)s_runtime.checkpointPulseUs);
    printf("DBGRECOVERY,checkpointMap="
           "fdc_init_begin:1|fdc_init_ok:2|route_applied:3|step_begin:4|snapshot_done:5|warning:6|"
           "sweep_candidate_begin:7|sweep_keyregs_begin:8|sweep_keyregs_done:9|sweep_sample_begin:10|"
           "sweep_sample_done:11|sweep_recover_begin:12|sweep_recover_done:13|terminal_fault:14|"
           "route_row_enter:15|route_row_exit:16|route_sela_enter:17|route_sela_exit:18|"
           "route_selb_enter:19|route_selb_exit:20|route_sw_enter:21|route_sw_exit:22|"
           "route_verify_enter:23|route_verify_exit:24|recovery_begin:25|restart_pending:26\n");
}

static void sensorarrayRecoveryCheckpointPulse(uint32_t pulseCount)
{
    if (!s_runtime.checkpointEnabled || pulseCount == 0u) {
        return;
    }
    for (uint32_t i = 0u; i < pulseCount; ++i) {
        gpio_set_level(s_runtime.checkpointGpio, 1);
        esp_rom_delay_us(s_runtime.checkpointPulseUs);
        gpio_set_level(s_runtime.checkpointGpio, 0);
        esp_rom_delay_us(s_runtime.checkpointPulseUs);
    }
}

void sensorarrayRecoveryEmitCheckpoint(sensorarrayRecoveryCheckpointEvent_t event)
{
    if (!s_runtime.s5d5Mode) {
        return;
    }
    sensorarrayRecoveryCheckpointEnsureConfigured();
    uint32_t pulseCount = sensorarrayRecoveryCheckpointPulseCount(event);
    if (pulseCount == 0u) {
        return;
    }
    sensorarrayRecoveryCheckpointPulse(pulseCount);
}

static void sensorarrayRecoveryPersistContext(const char *reason,
                                              esp_err_t err,
                                              sensorarrayRecoveryStage_t stage,
                                              uint8_t sColumn,
                                              uint8_t dLine,
                                              sensorarrayRecoveryCause_t cause)
{
    taskENTER_CRITICAL(&s_recoveryMux);
    s_rtcCtx.lastStage = (uint32_t)stage;
    s_rtcCtx.lastCause = (uint32_t)cause;
    s_rtcCtx.lastErr = (int32_t)err;
    s_rtcCtx.lastSColumn = sColumn;
    s_rtcCtx.lastDLine = dLine;
    s_rtcCtx.lastTick = (uint32_t)xTaskGetTickCount();
    if (reason && reason[0] != '\0') {
        (void)snprintf(s_rtcCtx.lastReason, sizeof(s_rtcCtx.lastReason), "%s", reason);
    } else {
        (void)snprintf(s_rtcCtx.lastReason, sizeof(s_rtcCtx.lastReason), "%s", SENSORARRAY_NA);
    }
    taskEXIT_CRITICAL(&s_recoveryMux);
}

void sensorarrayRecoveryKick(sensorarrayRecoveryStage_t stage, uint8_t sColumn, uint8_t dLine)
{
    TickType_t nowTick = xTaskGetTickCount();
    s_runtime.lastKickTick = (uint32_t)nowTick;
    s_runtime.lastKickStage = stage;
    s_runtime.lastKickSColumn = sColumn;
    s_runtime.lastKickDLine = dLine;

    sensorarrayRecoveryPersistContext("progress_kick", ESP_OK, stage, sColumn, dLine, SENSORARRAY_RECOVERY_CAUSE_NONE);
}

void sensorarrayRecoveryKickAlive(void)
{
    s_runtime.lastKickTick = (uint32_t)xTaskGetTickCount();
}

static void sensorarrayRecoveryTaskWdtEnsureConfigured(void)
{
    if (!s_runtime.taskWdtEnabled) {
        return;
    }
    if (s_runtime.taskWdtConfigured) {
        return;
    }

    esp_task_wdt_config_t config = {
        .timeout_ms = (uint32_t)CONFIG_SENSORARRAY_DEBUG_TASK_WDT_TIMEOUT_MS,
        .idle_core_mask = 0u,
        .trigger_panic = (CONFIG_SENSORARRAY_DEBUG_TASK_WDT_PANIC != 0),
    };
    esp_err_t err = esp_task_wdt_init(&config);
    if (err == ESP_ERR_INVALID_STATE) {
        err = esp_task_wdt_reconfigure(&config);
    }
    if (err == ESP_OK) {
        s_runtime.taskWdtConfigured = true;
        printf("DBGRECOVERY,taskWdt=enabled,timeoutMs=%lu,panic=%u\n",
               (unsigned long)config.timeout_ms,
               config.trigger_panic ? 1u : 0u);
    } else {
        printf("DBGRECOVERY,taskWdt=disabled,err=%ld\n", (long)err);
    }
}

void sensorarrayRecoveryTaskWdtRegisterCurrentTask(const char *taskTag)
{
    if (!s_runtime.taskWdtEnabled) {
        return;
    }
    sensorarrayRecoveryTaskWdtEnsureConfigured();
    if (!s_runtime.taskWdtConfigured) {
        return;
    }

    TaskHandle_t currentTask = xTaskGetCurrentTaskHandle();
    if (s_runtime.taskWdtTaskRegistered && s_runtime.taskWdtTaskHandle == currentTask) {
        return;
    }

    esp_err_t statusErr = esp_task_wdt_status(currentTask);
    if (statusErr == ESP_OK) {
        s_runtime.taskWdtTaskRegistered = true;
        s_runtime.taskWdtTaskHandle = currentTask;
        return;
    }

    esp_err_t addErr = esp_task_wdt_add(currentTask);
    if (addErr == ESP_OK || addErr == ESP_ERR_INVALID_STATE) {
        s_runtime.taskWdtTaskRegistered = true;
        s_runtime.taskWdtTaskHandle = currentTask;
        printf("DBGRECOVERY,taskWdt=task_registered,task=%s\n", taskTag ? taskTag : SENSORARRAY_NA);
    } else {
        printf("DBGRECOVERY,taskWdt=task_register_failed,task=%s,err=%ld\n",
               taskTag ? taskTag : SENSORARRAY_NA,
               (long)addErr);
    }
}

void sensorarrayRecoveryTaskWdtReset(void)
{
    if (!s_runtime.taskWdtEnabled || !s_runtime.taskWdtTaskRegistered) {
        return;
    }
    (void)esp_task_wdt_reset();
}

void sensorarrayRecoveryTaskWdtDeleteCurrentTask(void)
{
    if (!s_runtime.taskWdtEnabled || !s_runtime.taskWdtTaskRegistered) {
        return;
    }
    TaskHandle_t currentTask = xTaskGetCurrentTaskHandle();
    if (currentTask != s_runtime.taskWdtTaskHandle) {
        return;
    }
    (void)esp_task_wdt_delete(currentTask);
    s_runtime.taskWdtTaskRegistered = false;
    s_runtime.taskWdtTaskHandle = NULL;
}

static void sensorarrayRecoveryHandleFatalInternal(const char *reason,
                                                   esp_err_t err,
                                                   sensorarrayRecoveryStage_t stage,
                                                   uint8_t sColumn,
                                                   uint8_t dLine,
                                                   sensorarrayRecoveryCause_t cause)
{
    if (s_runtime.restartPending) {
        return;
    }

    s_runtime.restartPending = true;
    sensorarrayRecoveryEmitCheckpoint(SENSORARRAY_RECOVERY_CHECKPOINT_EVENT_RECOVERY_BEGIN);
    sensorarrayRecoveryKick(stage, sColumn, dLine);
    sensorarrayRecoveryPersistContext(reason, err, stage, sColumn, dLine, cause);

    sensorarrayRecoveryFailAction_t action = sensorarrayRecoveryFailAction();
    printf("DBGRECOVERY,fatalReason=%s,cause=%s,stage=%s,sColumn=%u,dLine=%u,err=%ld,action=%s\n",
           reason ? reason : SENSORARRAY_NA,
           sensorarrayRecoveryCauseName(cause),
           sensorarrayRecoveryStageName(stage),
           (unsigned)sColumn,
           (unsigned)dLine,
           (long)err,
           sensorarrayRecoveryFailActionName(action));

    if (action == SENSORARRAY_RECOVERY_FAIL_ACTION_RESTART) {
        sensorarrayRecoveryEmitCheckpoint(SENSORARRAY_RECOVERY_CHECKPOINT_EVENT_RESTART_PENDING);
        sensorarrayRecoveryPersistContext("restart_pending",
                                          err,
                                          SENSORARRAY_RECOVERY_STAGE_RESTART_PENDING,
                                          sColumn,
                                          dLine,
                                          cause);

        taskENTER_CRITICAL(&s_recoveryMux);
        s_rtcCtx.rebootStreak++;
        uint32_t rebootStreak = s_rtcCtx.rebootStreak;
        taskEXIT_CRITICAL(&s_recoveryMux);

        uint32_t restartDelayMs = (uint32_t)CONFIG_SENSORARRAY_DEBUG_RESTART_DELAY_MS;
        uint32_t maxStreakBeforeCooldown = (uint32_t)CONFIG_SENSORARRAY_DEBUG_MAX_REBOOT_STREAK_BEFORE_COOLDOWN;
        uint32_t cooldownMs = (uint32_t)CONFIG_SENSORARRAY_DEBUG_REBOOT_BACKOFF_MS;
        if (maxStreakBeforeCooldown > 0u && rebootStreak >= maxStreakBeforeCooldown && cooldownMs > 0u) {
            printf("DBGRECOVERY,restartBackoff=active,rebootStreak=%lu,threshold=%lu,backoffMs=%lu\n",
                   (unsigned long)rebootStreak,
                   (unsigned long)maxStreakBeforeCooldown,
                   (unsigned long)cooldownMs);
            restartDelayMs += cooldownMs;
        }
        if (restartDelayMs > 0u) {
            vTaskDelay(pdMS_TO_TICKS(restartDelayMs));
        }
        esp_restart();
        return;
    }

    if (action == SENSORARRAY_RECOVERY_FAIL_ACTION_PANIC) {
        abort();
    }

    s_runtime.restartPending = false;
    sensorarrayRecoveryControlledIdle(reason, SENSORARRAY_RECOVERY_DEFAULT_HEARTBEAT_MS);
}

void sensorarrayRecoveryHandleFatal(const char *reason,
                                    esp_err_t err,
                                    sensorarrayRecoveryStage_t stage,
                                    uint8_t sColumn,
                                    uint8_t dLine)
{
    sensorarrayRecoveryHandleFatalInternal(reason,
                                           err,
                                           stage,
                                           sColumn,
                                           dLine,
                                           SENSORARRAY_RECOVERY_CAUSE_EXPLICIT_FATAL);
}

void sensorarrayRecoveryControlledIdle(const char *reason, uint32_t heartbeatMs)
{
    if (heartbeatMs == 0u) {
        heartbeatMs = SENSORARRAY_RECOVERY_DEFAULT_HEARTBEAT_MS;
    }
    uint32_t beat = 0u;
    while (true) {
        beat++;
        printf("DBGRECOVERY,idleHeartbeat=1,reason=%s,beat=%lu,lastStage=%s\n",
               reason ? reason : SENSORARRAY_NA,
               (unsigned long)beat,
               sensorarrayRecoveryStageName(s_runtime.lastKickStage));
        sensorarrayRecoveryKickAlive();
        sensorarrayRecoveryTaskWdtReset();
        vTaskDelay(pdMS_TO_TICKS(heartbeatMs));
    }
}

static void sensorarrayRecoveryProgressMonitorTask(void *arg)
{
    (void)arg;
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(s_runtime.progressMonitorPeriodMs));

        if (!s_runtime.progressEnabled || !s_runtime.s5d5Mode || s_runtime.restartPending) {
            continue;
        }

        uint32_t lastKickTick = s_runtime.lastKickTick;
        if (lastKickTick == 0u) {
            continue;
        }

        uint32_t nowTick = (uint32_t)xTaskGetTickCount();
        uint32_t elapsedMs = (nowTick - lastKickTick) * (uint32_t)portTICK_PERIOD_MS;
        if (elapsedMs <= s_runtime.progressTimeoutMs) {
            continue;
        }

        sensorarrayRecoveryStage_t stalledStage = s_runtime.lastKickStage;
        uint8_t stalledSColumn = s_runtime.lastKickSColumn;
        uint8_t stalledDLine = s_runtime.lastKickDLine;
        printf("DBGRECOVERY,reason=progress_stall,lastStage=%s,sColumn=%u,dLine=%u,elapsedMs=%lu,timeoutMs=%lu\n",
               sensorarrayRecoveryStageName(stalledStage),
               (unsigned)stalledSColumn,
               (unsigned)stalledDLine,
               (unsigned long)elapsedMs,
               (unsigned long)s_runtime.progressTimeoutMs);
        sensorarrayRecoveryHandleFatalInternal("progress_stall",
                                               ESP_ERR_TIMEOUT,
                                               stalledStage,
                                               stalledSColumn,
                                               stalledDLine,
                                               SENSORARRAY_RECOVERY_CAUSE_PROGRESS_STALL);
    }
}

void sensorarrayRecoveryInit(sensorarrayDebugMode_t mode)
{
    esp_reset_reason_t bootReason = esp_reset_reason();

    s_runtime.s5d5Mode = (mode == SENSORARRAY_DEBUG_MODE_S5D5_CAP_FDC_SECONDARY);
    s_runtime.progressEnabled = s_runtime.s5d5Mode && (CONFIG_SENSORARRAY_DEBUG_PROGRESS_WATCHDOG_ENABLE != 0);
    s_runtime.progressTimeoutMs = (uint32_t)CONFIG_SENSORARRAY_DEBUG_PROGRESS_WATCHDOG_TIMEOUT_MS;
    if (s_runtime.progressTimeoutMs < 100u) {
        s_runtime.progressTimeoutMs = 100u;
    }
    s_runtime.progressMonitorPeriodMs = (uint32_t)CONFIG_SENSORARRAY_DEBUG_PROGRESS_WATCHDOG_MONITOR_PERIOD_MS;
    if (s_runtime.progressMonitorPeriodMs == 0u) {
        s_runtime.progressMonitorPeriodMs = 100u;
    }
    if (s_runtime.progressMonitorPeriodMs > s_runtime.progressTimeoutMs) {
        s_runtime.progressMonitorPeriodMs = s_runtime.progressTimeoutMs;
    }
    s_runtime.taskWdtEnabled = s_runtime.s5d5Mode && (CONFIG_SENSORARRAY_DEBUG_TASK_WDT_ENABLE != 0);
    s_runtime.restartPending = false;

    taskENTER_CRITICAL(&s_recoveryMux);
    if (s_rtcCtx.magic != SENSORARRAY_RECOVERY_RTC_MAGIC || s_rtcCtx.version != SENSORARRAY_RECOVERY_RTC_VERSION) {
        memset(&s_rtcCtx, 0, sizeof(s_rtcCtx));
        s_rtcCtx.magic = SENSORARRAY_RECOVERY_RTC_MAGIC;
        s_rtcCtx.version = SENSORARRAY_RECOVERY_RTC_VERSION;
        (void)snprintf(s_rtcCtx.lastReason, sizeof(s_rtcCtx.lastReason), "%s", "rtc_init");
    }
    s_rtcCtx.rebootCounter++;
    s_rtcCtx.lastBootReason = (uint32_t)bootReason;
    if (bootReason != ESP_RST_SW) {
        s_rtcCtx.rebootStreak = 0u;
    }
    taskEXIT_CRITICAL(&s_recoveryMux);

    s_runtime.lastKickTick = (uint32_t)xTaskGetTickCount();
    s_runtime.lastKickStage = SENSORARRAY_RECOVERY_STAGE_BOOT;
    s_runtime.lastKickSColumn = 0u;
    s_runtime.lastKickDLine = 0u;

    sensorarrayRecoveryCheckpointEnsureConfigured();

    if (s_runtime.progressEnabled && s_runtime.monitorTask == NULL) {
        BaseType_t taskOk = xTaskCreate(sensorarrayRecoveryProgressMonitorTask,
                                        "sa_recovery_mon",
                                        SENSORARRAY_RECOVERY_MONITOR_STACK_WORDS,
                                        NULL,
                                        SENSORARRAY_RECOVERY_MONITOR_PRIO,
                                        &s_runtime.monitorTask);
        if (taskOk != pdPASS) {
            s_runtime.monitorTask = NULL;
            printf("DBGRECOVERY,progressWatchdog=status_disabled,reason=task_create_failed\n");
        } else {
            printf("DBGRECOVERY,progressWatchdog=status_enabled,timeoutMs=%lu,periodMs=%lu\n",
                   (unsigned long)s_runtime.progressTimeoutMs,
                   (unsigned long)s_runtime.progressMonitorPeriodMs);
        }
    } else if (s_runtime.progressEnabled) {
        printf("DBGRECOVERY,progressWatchdog=status_enabled,timeoutMs=%lu,periodMs=%lu\n",
               (unsigned long)s_runtime.progressTimeoutMs,
               (unsigned long)s_runtime.progressMonitorPeriodMs);
    } else {
        printf("DBGRECOVERY,progressWatchdog=status_disabled\n");
    }

    if (!s_runtime.initialized) {
        s_runtime.initialized = true;
    }
}

void sensorarrayRecoveryLogBootInfo(void)
{
    taskENTER_CRITICAL(&s_recoveryMux);
    esp_reset_reason_t bootReason = (esp_reset_reason_t)s_rtcCtx.lastBootReason;
    sensorarrayRecoveryCause_t lastCause = (sensorarrayRecoveryCause_t)s_rtcCtx.lastCause;
    sensorarrayRecoveryStage_t lastStage = (sensorarrayRecoveryStage_t)s_rtcCtx.lastStage;
    int32_t lastErr = s_rtcCtx.lastErr;
    uint8_t lastSColumn = s_rtcCtx.lastSColumn;
    uint8_t lastDLine = s_rtcCtx.lastDLine;
    uint32_t rebootCounter = s_rtcCtx.rebootCounter;
    uint32_t rebootStreak = s_rtcCtx.rebootStreak;
    uint32_t lastTick = s_rtcCtx.lastTick;
    char lastReason[sizeof(s_rtcCtx.lastReason)] = {0};
    (void)snprintf(lastReason, sizeof(lastReason), "%s", s_rtcCtx.lastReason);
    taskEXIT_CRITICAL(&s_recoveryMux);

    printf("DBGRECOVERY,bootReason=%s\n", sensorarrayRecoveryResetReasonName(bootReason));
    printf("DBGRECOVERY,lastCause=%s\n", sensorarrayRecoveryCauseName(lastCause));
    printf("DBGRECOVERY,lastStage=%s\n", sensorarrayRecoveryStageName(lastStage));
    printf("DBGRECOVERY,lastReason=%s\n", (lastReason[0] != '\0') ? lastReason : SENSORARRAY_NA);
    printf("DBGRECOVERY,lastErr=%ld\n", (long)lastErr);
    printf("DBGRECOVERY,lastSColumn=%u,lastDLine=%u\n", (unsigned)lastSColumn, (unsigned)lastDLine);
    printf("DBGRECOVERY,lastTick=%lu\n", (unsigned long)lastTick);
    printf("DBGRECOVERY,lastProgressStall=%u,lastExplicitFatal=%u\n",
           (lastCause == SENSORARRAY_RECOVERY_CAUSE_PROGRESS_STALL) ? 1u : 0u,
           (lastCause == SENSORARRAY_RECOVERY_CAUSE_EXPLICIT_FATAL) ? 1u : 0u);
    printf("DBGRECOVERY,rebootCounter=%lu,rebootStreak=%lu\n",
           (unsigned long)rebootCounter,
           (unsigned long)rebootStreak);
}

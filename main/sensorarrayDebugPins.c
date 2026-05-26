#include "sensorarrayDebugPins.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define SENSORARRAY_DEBUG_HEARTBEAT_FREQ_HZ 2u
#define SENSORARRAY_DEBUG_HEARTBEAT_HALF_PERIOD_MS 250u
#define SENSORARRAY_DEBUG_HEARTBEAT_LEDC_TIMER LEDC_TIMER_0
#define SENSORARRAY_DEBUG_HEARTBEAT_LEDC_MODE LEDC_LOW_SPEED_MODE
#define SENSORARRAY_DEBUG_HEARTBEAT_LEDC_CHANNEL LEDC_CHANNEL_0
#define SENSORARRAY_DEBUG_HEARTBEAT_LEDC_DUTY_RES LEDC_TIMER_10_BIT
#define SENSORARRAY_DEBUG_HEARTBEAT_LEDC_DUTY (1u << 9)

typedef enum {
    SENSORARRAY_DEBUG_HEARTBEAT_KIND_NONE = 0,
    SENSORARRAY_DEBUG_HEARTBEAT_KIND_LEDC,
    SENSORARRAY_DEBUG_HEARTBEAT_KIND_TASK,
} sensorarrayDebugHeartbeatKind_t;

static bool s_debug_pins_inited = false;
static int s_heartbeat_level = 0;
static int64_t s_last_heartbeat_us = 0;
static TaskHandle_t s_heartbeat_task = NULL;
static sensorarrayDebugHeartbeatKind_t s_heartbeat_kind = SENSORARRAY_DEBUG_HEARTBEAT_KIND_NONE;

static const char *sensorarrayDebugPinsHeartbeatKindName(void)
{
    switch (s_heartbeat_kind) {
    case SENSORARRAY_DEBUG_HEARTBEAT_KIND_LEDC:
        return "ledc";
    case SENSORARRAY_DEBUG_HEARTBEAT_KIND_TASK:
        return "task";
    case SENSORARRAY_DEBUG_HEARTBEAT_KIND_NONE:
    default:
        return "none";
    }
}

static void sensorarrayDebugPinsWriteStage(uint8_t stage)
{
    uint8_t value = (uint8_t)(stage & 0x7u);
    gpio_set_level((gpio_num_t)SENSORARRAY_DEBUG_STAGE_BIT0_GPIO, (value & 0x1u) ? 1 : 0);
    gpio_set_level((gpio_num_t)SENSORARRAY_DEBUG_STAGE_BIT1_GPIO, (value & 0x2u) ? 1 : 0);
    gpio_set_level((gpio_num_t)SENSORARRAY_DEBUG_STAGE_BIT2_GPIO, (value & 0x4u) ? 1 : 0);
}

static void sensorarrayDebugPinsHeartbeatTask(void *arg)
{
    (void)arg;
    while (true) {
        s_heartbeat_level = s_heartbeat_level ? 0 : 1;
        gpio_set_level((gpio_num_t)SENSORARRAY_DEBUG_HEARTBEAT_GPIO, s_heartbeat_level);
        vTaskDelay(pdMS_TO_TICKS(SENSORARRAY_DEBUG_HEARTBEAT_HALF_PERIOD_MS));
    }
}

static esp_err_t sensorarrayDebugPinsStartTaskHeartbeat(esp_err_t ledcErr)
{
    if (s_heartbeat_task) {
        s_heartbeat_kind = SENSORARRAY_DEBUG_HEARTBEAT_KIND_TASK;
        return ESP_OK;
    }

    esp_err_t resetErr = gpio_reset_pin((gpio_num_t)SENSORARRAY_DEBUG_HEARTBEAT_GPIO);
    if (resetErr != ESP_OK) {
        printf("DBGSTAGEPINS,stage=heartbeat,kind=task_fallback,err=%ld,status=reset_failed\n",
               (long)resetErr);
        return resetErr;
    }

    gpio_config_t heartbeatCfg = {
        .pin_bit_mask = (1ULL << SENSORARRAY_DEBUG_HEARTBEAT_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t cfgErr = gpio_config(&heartbeatCfg);
    if (cfgErr != ESP_OK) {
        printf("DBGSTAGEPINS,stage=heartbeat,kind=task_fallback,err=%ld,status=config_failed\n",
               (long)cfgErr);
        return cfgErr;
    }

    BaseType_t taskOk = xTaskCreate(sensorarrayDebugPinsHeartbeatTask,
                                    "s5d5_hb",
                                    2048u,
                                    NULL,
                                    tskIDLE_PRIORITY + 1u,
                                    &s_heartbeat_task);
    if (taskOk != pdPASS) {
        printf("DBGSTAGEPINS,stage=heartbeat,kind=task_fallback,err=%ld,status=task_create_failed\n",
               (long)ESP_ERR_NO_MEM);
        return ESP_ERR_NO_MEM;
    }

    s_heartbeat_kind = SENSORARRAY_DEBUG_HEARTBEAT_KIND_TASK;
    printf("DBGSTAGEPINS,stage=heartbeat,kind=task_fallback,err=%ld\n", (long)ledcErr);
    return ESP_OK;
}

esp_err_t sensorarrayDebugPinsStartHardwareHeartbeat(void)
{
#if CONFIG_SENSORARRAY_DEBUG_STAGE_PINS_ENABLE
    if (s_heartbeat_kind == SENSORARRAY_DEBUG_HEARTBEAT_KIND_LEDC ||
        s_heartbeat_kind == SENSORARRAY_DEBUG_HEARTBEAT_KIND_TASK) {
        return ESP_OK;
    }

#if CONFIG_SENSORARRAY_DEBUG_S5D5_HARDWARE_HEARTBEAT
    ledc_timer_config_t timerCfg = {
        .speed_mode = SENSORARRAY_DEBUG_HEARTBEAT_LEDC_MODE,
        .duty_resolution = SENSORARRAY_DEBUG_HEARTBEAT_LEDC_DUTY_RES,
        .timer_num = SENSORARRAY_DEBUG_HEARTBEAT_LEDC_TIMER,
        .freq_hz = SENSORARRAY_DEBUG_HEARTBEAT_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
        .deconfigure = false,
    };
    esp_err_t err = ledc_timer_config(&timerCfg);
    if (err == ESP_OK) {
        ledc_channel_config_t channelCfg = {
            .gpio_num = SENSORARRAY_DEBUG_HEARTBEAT_GPIO,
            .speed_mode = SENSORARRAY_DEBUG_HEARTBEAT_LEDC_MODE,
            .channel = SENSORARRAY_DEBUG_HEARTBEAT_LEDC_CHANNEL,
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = SENSORARRAY_DEBUG_HEARTBEAT_LEDC_TIMER,
            .duty = SENSORARRAY_DEBUG_HEARTBEAT_LEDC_DUTY,
            .hpoint = 0,
            .flags.output_invert = 0,
        };
        err = ledc_channel_config(&channelCfg);
    }
    if (err == ESP_OK) {
        s_heartbeat_kind = SENSORARRAY_DEBUG_HEARTBEAT_KIND_LEDC;
        return ESP_OK;
    }
    return sensorarrayDebugPinsStartTaskHeartbeat(err);
#else
    return sensorarrayDebugPinsStartTaskHeartbeat(ESP_ERR_NOT_SUPPORTED);
#endif
#else
    return ESP_OK;
#endif
}

esp_err_t sensorarrayDebugPinsInit(void)
{
#if CONFIG_SENSORARRAY_DEBUG_STAGE_PINS_ENABLE
    if (s_debug_pins_inited) {
        esp_err_t heartbeatErr = sensorarrayDebugPinsStartHardwareHeartbeat();
        if (heartbeatErr != ESP_OK) {
            return heartbeatErr;
        }
        return ESP_OK;
    }

    const gpio_num_t pins[] = {
        (gpio_num_t)SENSORARRAY_DEBUG_STAGE_BIT0_GPIO,
        (gpio_num_t)SENSORARRAY_DEBUG_STAGE_BIT1_GPIO,
        (gpio_num_t)SENSORARRAY_DEBUG_STAGE_BIT2_GPIO,
    };

    for (size_t i = 0u; i < (sizeof(pins) / sizeof(pins[0])); ++i) {
        esp_err_t resetErr = gpio_reset_pin(pins[i]);
        if (resetErr != ESP_OK) {
            printf("DBGSTAGEPINS,stage=init,err=%ld,status=reset_failed,gpio=%d\n",
                   (long)resetErr,
                   (int)pins[i]);
            return resetErr;
        }
    }

    gpio_config_t ioCfg = {
        .pin_bit_mask = (1ULL << SENSORARRAY_DEBUG_STAGE_BIT0_GPIO) |
                        (1ULL << SENSORARRAY_DEBUG_STAGE_BIT1_GPIO) |
                        (1ULL << SENSORARRAY_DEBUG_STAGE_BIT2_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&ioCfg);
    if (err != ESP_OK) {
        printf("DBGSTAGEPINS,stage=init,err=%ld,status=config_failed\n", (long)err);
        return err;
    }

    s_debug_pins_inited = true;
    s_heartbeat_level = 0;
    s_last_heartbeat_us = esp_timer_get_time();
    sensorarrayDebugPinsWriteStage(0u);

    esp_err_t heartbeatErr = sensorarrayDebugPinsStartHardwareHeartbeat();
    if (heartbeatErr != ESP_OK) {
        printf("DBGSTAGEPINS,stage=init,err=%ld,status=heartbeat_failed,bit0=%d,bit1=%d,bit2=%d,heartbeat=%d,heartbeatKind=%s\n",
               (long)heartbeatErr,
               SENSORARRAY_DEBUG_STAGE_BIT0_GPIO,
               SENSORARRAY_DEBUG_STAGE_BIT1_GPIO,
               SENSORARRAY_DEBUG_STAGE_BIT2_GPIO,
               SENSORARRAY_DEBUG_HEARTBEAT_GPIO,
               sensorarrayDebugPinsHeartbeatKindName());
        return heartbeatErr;
    }

    printf("DBGSTAGEPINS,stage=init,err=0,status=enabled,bit0=%d,bit1=%d,bit2=%d,heartbeat=%d,heartbeatKind=%s\n",
           SENSORARRAY_DEBUG_STAGE_BIT0_GPIO,
           SENSORARRAY_DEBUG_STAGE_BIT1_GPIO,
           SENSORARRAY_DEBUG_STAGE_BIT2_GPIO,
           SENSORARRAY_DEBUG_HEARTBEAT_GPIO,
           sensorarrayDebugPinsHeartbeatKindName());
#endif
    return ESP_OK;
}

void sensorarrayDebugPinsSetStage(uint8_t stage)
{
#if CONFIG_SENSORARRAY_DEBUG_STAGE_PINS_ENABLE
    if (!s_debug_pins_inited) {
        return;
    }
    sensorarrayDebugPinsWriteStage(stage);
#else
    (void)stage;
#endif
}

void sensorarrayDebugPinsHeartbeatToggle(void)
{
#if CONFIG_SENSORARRAY_DEBUG_STAGE_PINS_ENABLE
    if (!s_debug_pins_inited) {
        return;
    }
    if (s_heartbeat_kind == SENSORARRAY_DEBUG_HEARTBEAT_KIND_LEDC ||
        s_heartbeat_kind == SENSORARRAY_DEBUG_HEARTBEAT_KIND_TASK) {
        return;
    }
    s_heartbeat_level = s_heartbeat_level ? 0 : 1;
    gpio_set_level((gpio_num_t)SENSORARRAY_DEBUG_HEARTBEAT_GPIO, s_heartbeat_level);
#endif
}

void sensorarrayDebugPinsHeartbeatMaybe(uint32_t periodMs)
{
#if CONFIG_SENSORARRAY_DEBUG_STAGE_PINS_ENABLE
    if (!s_debug_pins_inited) {
        return;
    }
    if (s_heartbeat_kind == SENSORARRAY_DEBUG_HEARTBEAT_KIND_LEDC ||
        s_heartbeat_kind == SENSORARRAY_DEBUG_HEARTBEAT_KIND_TASK) {
        return;
    }
    if (periodMs == 0u) {
        periodMs = 500u;
    }
    int64_t nowUs = esp_timer_get_time();
    int64_t periodUs = (int64_t)periodMs * 1000LL;
    if (nowUs - s_last_heartbeat_us >= periodUs) {
        sensorarrayDebugPinsHeartbeatToggle();
        s_last_heartbeat_us = nowUs;
    }
#else
    (void)periodMs;
#endif
}

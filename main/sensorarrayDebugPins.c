#include "sensorarrayDebugPins.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>

#include "driver/gpio.h"
#include "esp_timer.h"

static bool s_debug_pins_inited = false;
static int s_heartbeat_level = 0;
static int64_t s_last_heartbeat_us = 0;

static void sensorarrayDebugPinsWriteStage(uint8_t stage)
{
    uint8_t value = (uint8_t)(stage & 0x7u);
    gpio_set_level((gpio_num_t)SENSORARRAY_DEBUG_STAGE_BIT0_GPIO, (value & 0x1u) ? 1 : 0);
    gpio_set_level((gpio_num_t)SENSORARRAY_DEBUG_STAGE_BIT1_GPIO, (value & 0x2u) ? 1 : 0);
    gpio_set_level((gpio_num_t)SENSORARRAY_DEBUG_STAGE_BIT2_GPIO, (value & 0x4u) ? 1 : 0);
}

esp_err_t sensorarrayDebugPinsInit(void)
{
#if CONFIG_SENSORARRAY_DEBUG_STAGE_PINS_ENABLE
    const gpio_num_t pins[] = {
        (gpio_num_t)SENSORARRAY_DEBUG_STAGE_BIT0_GPIO,
        (gpio_num_t)SENSORARRAY_DEBUG_STAGE_BIT1_GPIO,
        (gpio_num_t)SENSORARRAY_DEBUG_STAGE_BIT2_GPIO,
        (gpio_num_t)SENSORARRAY_DEBUG_HEARTBEAT_GPIO,
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
                        (1ULL << SENSORARRAY_DEBUG_STAGE_BIT2_GPIO) |
                        (1ULL << SENSORARRAY_DEBUG_HEARTBEAT_GPIO),
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
    gpio_set_level((gpio_num_t)SENSORARRAY_DEBUG_HEARTBEAT_GPIO, 0);
    printf("DBGSTAGEPINS,stage=init,err=0,status=enabled,bit0=%d,bit1=%d,bit2=%d,heartbeat=%d\n",
           SENSORARRAY_DEBUG_STAGE_BIT0_GPIO,
           SENSORARRAY_DEBUG_STAGE_BIT1_GPIO,
           SENSORARRAY_DEBUG_STAGE_BIT2_GPIO,
           SENSORARRAY_DEBUG_HEARTBEAT_GPIO);
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

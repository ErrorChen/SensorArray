#pragma once

#include <stdint.h>

#include "esp_err.h"
#include "sensorarrayConfig.h"

#define SENSORARRAY_DEBUG_STAGE_BIT0_GPIO CONFIG_SENSORARRAY_DEBUG_STAGE_BIT0_GPIO
#define SENSORARRAY_DEBUG_STAGE_BIT1_GPIO CONFIG_SENSORARRAY_DEBUG_STAGE_BIT1_GPIO
#define SENSORARRAY_DEBUG_STAGE_BIT2_GPIO CONFIG_SENSORARRAY_DEBUG_STAGE_BIT2_GPIO
#define SENSORARRAY_DEBUG_HEARTBEAT_GPIO CONFIG_SENSORARRAY_DEBUG_HEARTBEAT_GPIO

#define SENSORARRAY_DEBUG_FORBIDDEN_PIN(pin) \
    ((pin) == 15 || (pin) == 16 || \
     (pin) == 35 || (pin) == 36 || (pin) == 37 || \
     (pin) == 38 || \
     (pin) == 19 || (pin) == 20 || \
     (pin) == 0 || (pin) == 45 || (pin) == 46)

#if SENSORARRAY_DEBUG_FORBIDDEN_PIN(SENSORARRAY_DEBUG_STAGE_BIT0_GPIO) || \
    SENSORARRAY_DEBUG_FORBIDDEN_PIN(SENSORARRAY_DEBUG_STAGE_BIT1_GPIO) || \
    SENSORARRAY_DEBUG_FORBIDDEN_PIN(SENSORARRAY_DEBUG_STAGE_BIT2_GPIO) || \
    SENSORARRAY_DEBUG_FORBIDDEN_PIN(SENSORARRAY_DEBUG_HEARTBEAT_GPIO)
#error "Do not use GPIO15/16, GPIO35/36/37, GPIO38, GPIO19/20, or GPIO0/45/46 for debug pins on this board."
#endif

esp_err_t sensorarrayDebugPinsInit(void);
void sensorarrayDebugPinsSetStage(uint8_t stage);
void sensorarrayDebugPinsHeartbeatToggle(void);
void sensorarrayDebugPinsHeartbeatMaybe(uint32_t periodMs);

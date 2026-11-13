#pragma once
#include "freertos/FreeRTOS.h"
TickType_t xTaskGetTickCount(void);
int xPortGetCoreID(void);
void vTaskDelay(TickType_t ticks);

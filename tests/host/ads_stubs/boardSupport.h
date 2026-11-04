#pragma once
#include "esp_err.h"
#define BOARD_SUPPORT_GPIO_ISR_HANDLER_ADS 1
esp_err_t sensorarrayBoardEnsureGpioIsrService(void);
void sensorarrayBoardNoteGpioIsrHandlerAdd(int kind);

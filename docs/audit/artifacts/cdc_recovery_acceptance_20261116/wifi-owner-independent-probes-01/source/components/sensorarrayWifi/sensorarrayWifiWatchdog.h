#pragma once
#include "esp_err.h"
/* Lazy-once bootstrap is serialized by the owner admission claim. */
esp_err_t sensorarrayWifiWatchdogInit(void);

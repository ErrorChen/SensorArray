#pragma once

#include "esp_err.h"

/* New in this fix: an independent ESP_TIMER_TASK observer, not a watchdog
 * feeder or an SDK cancellation mechanism. Init is serialized by BLE Init. */
#define SENSORARRAY_BLE_SEND_DEADLINE_US 500000u
#define SENSORARRAY_BLE_SEND_POLL_US 100000u

esp_err_t sensorarrayBleWatchdogInit(void);
void sensorarrayBleObserveSendDeadline(void);

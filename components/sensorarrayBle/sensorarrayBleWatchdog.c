#include "sensorarrayBleWatchdog.h"

#include <stdbool.h>
#include <stddef.h>
#include "esp_timer.h"

/* Boot lifetime storage: never delete/recreate a timer after partial startup.
 * The BLE Init admission guard serializes this private initializer. */
static esp_timer_handle_t s_sendTimer;
static bool s_initAttempted;
static esp_err_t s_initResult = ESP_ERR_INVALID_STATE;

static void sensorarrayBleWatchdogTick(void *context)
{
    (void)context;
    sensorarrayBleObserveSendDeadline();
}

esp_err_t sensorarrayBleWatchdogInit(void)
{
    if (s_initAttempted) {
        return s_initResult;
    }
    s_initAttempted = true;
    const esp_timer_create_args_t args = {
        .callback = sensorarrayBleWatchdogTick,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "bleSendGuard",
        .skip_unhandled_events = true,
    };
    s_initResult = esp_timer_create(&args, &s_sendTimer);
    if (s_initResult == ESP_OK) {
        s_initResult = esp_timer_start_periodic(s_sendTimer,
                                               SENSORARRAY_BLE_SEND_POLL_US);
    }
    return s_initResult;
}

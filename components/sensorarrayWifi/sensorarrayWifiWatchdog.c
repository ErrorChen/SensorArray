#include "sensorarrayWifiWatchdog.h"
#include "sensorarrayWifi.h"
#include "esp_timer.h"

static esp_timer_handle_t s_wifiTimer;
static bool s_attempted;
static esp_err_t s_result = ESP_ERR_INVALID_STATE;

static void sensorarrayWifiWatchdogTick(void *context)
{
    (void)context;
    sensorarrayWifiObserveDeadline();
}

esp_err_t sensorarrayWifiWatchdogInit(void)
{
    if (s_attempted) { return s_result; }
    s_attempted = true;
    const esp_timer_create_args_t args = {
        .callback = sensorarrayWifiWatchdogTick,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "wifiOwnerGuard",
        .skip_unhandled_events = true,
    };
    s_result = esp_timer_create(&args, &s_wifiTimer);
    if (s_result == ESP_OK) {
        s_result = esp_timer_start_periodic(s_wifiTimer, SENSORARRAY_WIFI_OBSERVER_POLL_US);
    }
    return s_result;
}

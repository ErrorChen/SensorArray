idf_component_register(
    SRCS "sensorarrayWifi.c" "sensorarrayWifiOwner.c" "sensorarrayWifiWatchdog.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_event esp_netif esp_wifi lwip freertos esp_timer heap sensorarrayWorkBudget
)

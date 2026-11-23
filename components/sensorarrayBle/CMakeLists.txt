idf_component_register(
    SRCS "sensorarrayBle.c" "sensorarrayBleWatchdog.c" "sensorarrayBleOutputSchedule.c"
    INCLUDE_DIRS "include"
    REQUIRES bt heap esp_timer sensorarrayWorkBudget
)

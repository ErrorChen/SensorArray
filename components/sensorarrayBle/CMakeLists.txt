idf_component_register(
    SRCS "sensorarrayBle.c"
    INCLUDE_DIRS "include"
    REQUIRES bt heap esp_timer sensorarrayWorkBudget
)

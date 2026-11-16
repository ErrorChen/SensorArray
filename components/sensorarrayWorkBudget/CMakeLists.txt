idf_component_register(
    SRCS "sensorarrayWorkBudget.c" "sensorarrayCpuOwner.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_system esp_hw_support freertos esp_timer
)

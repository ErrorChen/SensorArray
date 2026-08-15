idf_component_register(
    SRCS
        "sensorarrayCalibration.c"
        "sensorarrayScanConfig.c"
    INCLUDE_DIRS "."
    REQUIRES freertos nvs_flash
)

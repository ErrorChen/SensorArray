idf_component_register(
    SRCS
        "sensorarrayTransport.c"
        "sensorarrayTransportCommand.c"
        "sensorarrayTransportGuaranteedText.c"
        "sensorarrayTransportPool.c"
        "sensorarrayTransportPolicy.c"
        "sensorarrayTransportStats.c"
    INCLUDE_DIRS "."
    REQUIRES sensorarrayBle sensorarrayWifi config nvs_flash esp_system freertos
)

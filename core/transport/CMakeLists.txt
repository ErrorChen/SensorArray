idf_component_register(
    SRCS
        "sensorarrayTransport.c"
        "sensorarraySerialIo.c"
        "sensorarrayCriticalText.c"
        "sensorarrayTransportFrameMeta.c"
        "sensorarrayTransportCommand.c"
        "sensorarrayTransportWifiControl.c"
        "sensorarrayTransportGuaranteedText.c"
        "sensorarrayTransportPool.c"
        "sensorarrayTransportPolicy.c"
        "sensorarrayTransportStats.c"
    INCLUDE_DIRS "."
    REQUIRES sensorarrayUsbOtg sensorarrayBle sensorarrayWifi sensorarrayWorkBudget config nvs_flash esp_system esp_timer esp_driver_usb_serial_jtag vfs freertos
)

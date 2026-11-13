idf_component_register(SRCS "sensorarrayUsbOtg.c" "sensorarrayUsbDescriptors.c"
    INCLUDE_DIRS "include"
    REQUIRES usb esp_timer esp_hw_support efuse espressif__tinyusb freertos)
# TinyUSB deliberately has no ESP wrapper task: SerialIo is the sole owner.
idf_component_get_property(tusb_lib espressif__tinyusb COMPONENT_LIB)
target_include_directories(${tusb_lib} PUBLIC "${CMAKE_CURRENT_LIST_DIR}/include")
# Panic/cpu-start restart paths bypass normal shutdown handlers. Protect DMA
# lifetime there as well; the final ELF call targets are checked by host tooling.
target_link_options(${COMPONENT_LIB} INTERFACE "-Wl,--wrap=esp_restart_noos")

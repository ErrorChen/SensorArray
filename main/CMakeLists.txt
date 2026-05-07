idf_component_register(
    SRCS
        "main.c"
        "sensorarrayApp.c"
        "sensorarrayBoardMap.c"
        "sensorarrayBringup.c"
        "sensorarrayMeasure.c"
        "sensorarrayVoltageScan.c"
        "sensorarrayVoltageStream.c"
        "sensorarrayStatus.c"
        "sensorarrayPerf.c"
        "sensorarrayRateControl.c"
        "sensorarrayLog.c"
        "sensorarrayDebug.c"
        "sensorarrayDebugFdcI2cDiscovery.c"
        "sensorarrayDebugFdcSelbS5d5.c"
        "sensorarrayDebugSelftest.c"
        "sensorarrayDebugS1d1.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES
        boardSupport
        esp_timer
        powerCtrl
        esp_driver_usb_serial_jtag
        vfs
        tmuxSwitch
        ads126xAdc
        fdc2214Cap
        protocolWire
        protocolUsb
        transportWire
        bleTransport
        matrixEngine
)

execute_process(
    COMMAND git rev-parse --short=12 HEAD
    WORKING_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}/.."
    OUTPUT_VARIABLE SENSORARRAY_GIT_HASH
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
)
if(NOT SENSORARRAY_GIT_HASH)
    set(SENSORARRAY_GIT_HASH "unknown")
endif()
target_compile_definitions(${COMPONENT_LIB} PRIVATE SENSORARRAY_GIT_HASH="${SENSORARRAY_GIT_HASH}")

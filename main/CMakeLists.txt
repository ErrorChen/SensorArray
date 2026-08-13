set(SENSORARRAY_MAIN_SRCS
    "main.c"
    "sensorarrayLog.c"
        "control/sensorarrayCommandMailbox.c"
    "net/sensorarrayNetStatus.c"
)

set(SENSORARRAY_CORE_SRCS
    "../core/board/sensorarrayBoardMap.c"
    "../core/board/sensorarrayBringup.c"
    "../core/measure/sensorarrayMeasure.c"
    "../core/measure/sensorarrayMeasurementMode.c"
    "../core/measure/sensorarrayRowModeProfile.c"
    "../core/measure/sensorarrayMeasurementSelfTest.c"
    "../core/measure/sensorarrayRouteController.c"
    "../core/measure/sensorarrayScanPlan.c"
    "../core/measure/sensorarrayFrameBuilder.c"
)

set(SENSORARRAY_FDC_SRCS
    "../core/measure/fdc/sensorarrayFdcCache.c"
    "../core/measure/fdc/sensorarrayFdcFrame.c"
    "../core/measure/fdc/sensorarrayFdcMatrix.c"
    "../core/measure/fdc/sensorarrayFdcQuality.c"
    "../core/measure/fdc/sensorarrayFdcRead4.c"
    "../core/measure/fdc/sensorarrayFdcReady.c"
    "../core/measure/fdc/sensorarrayFdcRescue.c"
    "../core/measure/fdc/sensorarrayFdcRescuePolicy.c"
    "../core/measure/fdc/sensorarrayFdcRowEpoch.c"
    "../core/measure/fdc/sensorarrayFdcSweep.c"
    "../core/measure/fdc/sensorarrayFdcTiming.c"
)

set(SENSORARRAY_ADS_SRCS
    "../core/measure/ads/sensorarrayAdsMatrix.c"
    "../core/measure/ads/sensorarrayAdsAutoRange.c"
        "../core/measure/ads/sensorarrayAdsCache.c"
        "../core/measure/ads/sensorarrayBatteryScheduler.c"
    "../core/measure/ads/sensorarrayAdsMath.c"
    "../core/measure/ads/sensorarrayAdsGap.c"
)

set(SENSORARRAY_MIXED_SRCS
    "../core/measure/mixed/sensorarrayMixedRow.c"
)

set(SENSORARRAY_TRANSPORT_SRCS)

set(SENSORARRAY_OUTPUT_SRCS
    "output/sensorarrayAsyncLog.c"
    "output/sensorarrayAcqEvent.c"
    "output/sensorarrayFrameOutput.c"
    "output/sensorarrayTextProtocol.c"
    "output/sensorarrayUsbSink.c"
)

idf_component_register(
    SRCS
        ${SENSORARRAY_MAIN_SRCS}
        ${SENSORARRAY_CORE_SRCS}
        ${SENSORARRAY_FDC_SRCS}
        ${SENSORARRAY_ADS_SRCS}
        ${SENSORARRAY_MIXED_SRCS}
        ${SENSORARRAY_TRANSPORT_SRCS}
        ${SENSORARRAY_OUTPUT_SRCS}
    INCLUDE_DIRS
        "."
        "output"
        "control"
        "net"
        "../core/board"
        "../core/measure/include"
        "../core/measure"
        "../core/measure/fdc"
        "../core/measure/ads"
        "../core/measure/mixed"
        "../core/transport"
    PRIV_REQUIRES
        boardSupport
        powerCtrl
        tmuxSwitch
        ads126xAdc
        fdc2214Cap
        sensorarrayBle
        sensorarrayWifi
        config
        transport
)

target_compile_options(${COMPONENT_LIB} PRIVATE
    -Wall
    -Wextra
    -Wformat=2
    -Wformat-security
)

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#include "ads126xAdc.h"
#include "sensorarrayPerf.h"
#include "sensorarrayStatus.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    SENSORARRAY_RATE_CAUSE_NONE = 0,
    SENSORARRAY_RATE_CAUSE_OUTPUT_QUEUE_FULL,
    SENSORARRAY_RATE_CAUSE_OUTPUT_USB_SLOW,
    SENSORARRAY_RATE_CAUSE_OUTPUT_SHORT_WRITE,
    SENSORARRAY_RATE_CAUSE_DRDY_TIMEOUT,
    SENSORARRAY_RATE_CAUSE_SPI_FAIL,
    SENSORARRAY_RATE_CAUSE_CRC_OR_STATUS_FAIL,
    SENSORARRAY_RATE_CAUSE_SCAN_OVER_BUDGET,
    SENSORARRAY_RATE_CAUSE_FATAL_POLICY_ERROR
} sensorarrayRateCause_t;

typedef enum {
    SENSORARRAY_RATE_ACTION_NONE = 0,
    SENSORARRAY_RATE_ACTION_INCREASE_OUTPUT_DIVIDER,
    SENSORARRAY_RATE_ACTION_DISABLE_CSV,
    SENSORARRAY_RATE_ACTION_USE_COMPACT_BINARY_ONLY,
    SENSORARRAY_RATE_ACTION_ADD_SCAN_FRAME_PERIOD,
    SENSORARRAY_RATE_ACTION_DECREASE_ADS_DR,
    SENSORARRAY_RATE_ACTION_INCREASE_MUX_SETTLE,
    SENSORARRAY_RATE_ACTION_FORCE_VERIFIED_INPMUX,
    SENSORARRAY_RATE_ACTION_ENTER_SAFE_PROFILE,
    SENSORARRAY_RATE_ACTION_FATAL_STOP
} sensorarrayRateAction_t;

typedef struct {
    uint32_t windowFrames;

    uint32_t framesScannedStart;
    uint32_t framesQueuedStart;
    uint32_t framesDroppedStart;
    uint32_t outputDecimatedStart;
    uint32_t queueFullStart;
    uint32_t usbShortWriteStart;
    uint32_t usbWriteFailStart;
    uint32_t drdyTimeoutStart;
    uint32_t spiFailStart;
    uint32_t crcFailStart;
    uint32_t statusBadStart;

    uint32_t stableWindowCount;
    uint32_t degradeCount;
    uint32_t upshiftCount;

    uint8_t initialAdsDr;
    uint8_t currentAdsDr;
    uint8_t requestedAdsDr;
    uint8_t minAdsDr;
    uint8_t maxAdsDr;

    uint32_t outputDivider;
    uint32_t scanFramePeriodUs;
    uint32_t muxSettleUs;

    bool csvEnabled;
    bool compactBinaryOnly;
    bool fastInpmuxEnabled;
    bool verifiedInpmuxForced;
    bool fatalStop;

    sensorarrayRateCause_t lastCause;
    sensorarrayRateAction_t lastAction;
} sensorarrayRateController_t;

void sensorarrayRateControllerInit(sensorarrayRateController_t *controller,
                                   uint8_t initialAdsDr,
                                   uint8_t minAdsDr);

bool sensorarrayRateControllerShouldPublishFrame(sensorarrayRateController_t *controller,
                                                 uint32_t sequence);

esp_err_t sensorarrayRateControllerUpdate(sensorarrayRateController_t *controller,
                                          const sensorarrayPerfCounters_t *perf,
                                          const sensorarrayStatusCounters_t *status,
                                          uint32_t streamQueueDepth,
                                          uint32_t streamQueueUsed,
                                          sensorarrayRateAction_t *outAction,
                                          sensorarrayRateCause_t *outCause);

esp_err_t sensorarrayRateControllerApplyAction(sensorarrayRateController_t *controller,
                                               sensorarrayRateAction_t action,
                                               sensorarrayRateCause_t cause,
                                               ads126xAdcHandle_t *ads);

void sensorarrayRateControllerForceOutputDividerAtLeast(sensorarrayRateController_t *controller,
                                                        uint32_t minDivider);

const char *sensorarrayRateActionName(sensorarrayRateAction_t action);
const char *sensorarrayRateCauseName(sensorarrayRateCause_t cause);

#ifdef __cplusplus
}
#endif

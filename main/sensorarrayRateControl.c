#include "sensorarrayRateControl.h"

#include "sdkconfig.h"

#ifndef CONFIG_SENSORARRAY_AUTO_RATE_WINDOW_FRAMES
#define CONFIG_SENSORARRAY_AUTO_RATE_WINDOW_FRAMES 100
#endif
#ifndef CONFIG_SENSORARRAY_AUTO_RATE_QUEUE_HIGH_WATERMARK_PERCENT
#define CONFIG_SENSORARRAY_AUTO_RATE_QUEUE_HIGH_WATERMARK_PERCENT 75
#endif
#ifndef CONFIG_SENSORARRAY_AUTO_RATE_QUEUE_LOW_WATERMARK_PERCENT
#define CONFIG_SENSORARRAY_AUTO_RATE_QUEUE_LOW_WATERMARK_PERCENT 25
#endif
#ifndef CONFIG_SENSORARRAY_AUTO_RATE_MAX_DROP_PER_WINDOW
#define CONFIG_SENSORARRAY_AUTO_RATE_MAX_DROP_PER_WINDOW 0
#endif
#ifndef CONFIG_SENSORARRAY_AUTO_RATE_MAX_DRDY_TIMEOUT_PER_WINDOW
#define CONFIG_SENSORARRAY_AUTO_RATE_MAX_DRDY_TIMEOUT_PER_WINDOW 0
#endif
#ifndef CONFIG_SENSORARRAY_AUTO_RATE_MAX_SPI_FAIL_PER_WINDOW
#define CONFIG_SENSORARRAY_AUTO_RATE_MAX_SPI_FAIL_PER_WINDOW 0
#endif
#ifndef CONFIG_SENSORARRAY_AUTO_RATE_RECOVERY_WINDOWS
#define CONFIG_SENSORARRAY_AUTO_RATE_RECOVERY_WINDOWS 20
#endif
#ifndef CONFIG_SENSORARRAY_AUTO_RATE_OUTPUT_DIV_MAX
#define CONFIG_SENSORARRAY_AUTO_RATE_OUTPUT_DIV_MAX 64
#endif
#ifndef CONFIG_SENSORARRAY_AUTO_RATE_SCAN_FRAME_PERIOD_MAX_US
#define CONFIG_SENSORARRAY_AUTO_RATE_SCAN_FRAME_PERIOD_MAX_US 20000
#endif
#ifndef CONFIG_SENSORARRAY_VOLTAGE_SCAN_MUX_SETTLE_US
#define CONFIG_SENSORARRAY_VOLTAGE_SCAN_MUX_SETTLE_US 20
#endif

static void sensorarrayRateControllerMarkWindowStart(sensorarrayRateController_t *controller,
                                                     const sensorarrayPerfCounters_t *perf,
                                                     const sensorarrayStatusCounters_t *status)
{
    if (!controller || !perf || !status) {
        return;
    }
    controller->framesScannedStart = perf->framesScanned;
    controller->framesQueuedStart = perf->framesQueued;
    controller->framesDroppedStart = perf->framesDropped;
    controller->outputDecimatedStart = perf->outputDecimatedFrames;
    controller->queueFullStart = status->queueFullCount;
    controller->usbShortWriteStart = status->usbShortWriteCount;
    controller->usbWriteFailStart = status->usbWriteFailCount;
    controller->drdyTimeoutStart = status->adsDrdyTimeoutCount;
    controller->spiFailStart = status->adsSpiFailCount;
    controller->crcFailStart = status->adsCrcFailCount;
    controller->statusBadStart = status->adsStatusBadCount;
}

void sensorarrayRateControllerInit(sensorarrayRateController_t *controller,
                                   uint8_t initialAdsDr,
                                   uint8_t minAdsDr)
{
    if (!controller) {
        return;
    }

    if (initialAdsDr > 15u) {
        initialAdsDr = 15u;
    }
    if (minAdsDr > initialAdsDr) {
        minAdsDr = initialAdsDr;
    }

    *controller = (sensorarrayRateController_t){
        .windowFrames = (uint32_t)CONFIG_SENSORARRAY_AUTO_RATE_WINDOW_FRAMES,
        .initialAdsDr = initialAdsDr,
        .currentAdsDr = initialAdsDr,
        .requestedAdsDr = initialAdsDr,
        .minAdsDr = minAdsDr,
        .maxAdsDr = initialAdsDr,
        .outputDivider = 1u,
        .muxSettleUs = (uint32_t)CONFIG_SENSORARRAY_VOLTAGE_SCAN_MUX_SETTLE_US,
        .csvEnabled =
#if CONFIG_SENSORARRAY_OUTPUT_FORMAT_CSV || CONFIG_SENSORARRAY_OUTPUT_FORMAT_BOTH
            true,
#else
            false,
#endif
        .compactBinaryOnly =
#if CONFIG_SENSORARRAY_OUTPUT_FORMAT_BINARY
            true,
#else
            false,
#endif
        .fastInpmuxEnabled =
#if CONFIG_SENSORARRAY_ADS_FAST_INPMUX_WRITE
            true,
#else
            false,
#endif
    };
}

bool sensorarrayRateControllerShouldPublishFrame(sensorarrayRateController_t *controller,
                                                 uint32_t sequence)
{
    if (!controller || controller->fatalStop) {
        return false;
    }
    uint32_t div = controller->outputDivider ? controller->outputDivider : 1u;
    return (div <= 1u) || ((sequence % div) == 0u);
}

esp_err_t sensorarrayRateControllerUpdate(sensorarrayRateController_t *controller,
                                          const sensorarrayPerfCounters_t *perf,
                                          const sensorarrayStatusCounters_t *status,
                                          uint32_t streamQueueDepth,
                                          uint32_t streamQueueUsed,
                                          sensorarrayRateAction_t *outAction,
                                          sensorarrayRateCause_t *outCause)
{
    if (!controller || !perf || !status || !outAction || !outCause) {
        return ESP_ERR_INVALID_ARG;
    }

    *outAction = SENSORARRAY_RATE_ACTION_NONE;
    *outCause = SENSORARRAY_RATE_CAUSE_NONE;

#if !CONFIG_SENSORARRAY_AUTO_RATE_CONTROL || CONFIG_SENSORARRAY_AUTO_RATE_CONTROL_OFF
    sensorarrayRateControllerMarkWindowStart(controller, perf, status);
    return ESP_OK;
#else
    if (controller->fatalStop || (status->frameStatusFlags & SENSORARRAY_FRAME_FLAG_FATAL) != 0u) {
        *outAction = SENSORARRAY_RATE_ACTION_FATAL_STOP;
        *outCause = SENSORARRAY_RATE_CAUSE_FATAL_POLICY_ERROR;
        return ESP_OK;
    }

    const uint32_t framesDropped = perf->framesDropped - controller->framesDroppedStart;
    const uint32_t queueFull = status->queueFullCount - controller->queueFullStart;
    const uint32_t usbShort = status->usbShortWriteCount - controller->usbShortWriteStart;
    const uint32_t usbFail = status->usbWriteFailCount - controller->usbWriteFailStart;
    const uint32_t drdyTimeout = status->adsDrdyTimeoutCount - controller->drdyTimeoutStart;
    const uint32_t spiFail = status->adsSpiFailCount - controller->spiFailStart;
    const uint32_t crcOrStatus = (status->adsCrcFailCount - controller->crcFailStart) +
                                 (status->adsStatusBadCount - controller->statusBadStart);
    const uint32_t queueUsedPercent = (streamQueueDepth == 0u) ? 0u : (streamQueueUsed * 100u) / streamQueueDepth;

    if (drdyTimeout > (uint32_t)CONFIG_SENSORARRAY_AUTO_RATE_MAX_DRDY_TIMEOUT_PER_WINDOW) {
        *outCause = SENSORARRAY_RATE_CAUSE_DRDY_TIMEOUT;
        if (controller->currentAdsDr > controller->minAdsDr) {
            *outAction = SENSORARRAY_RATE_ACTION_DECREASE_ADS_DR;
        } else if (controller->muxSettleUs < 100u) {
            *outAction = SENSORARRAY_RATE_ACTION_INCREASE_MUX_SETTLE;
        } else if (!controller->verifiedInpmuxForced) {
            *outAction = SENSORARRAY_RATE_ACTION_FORCE_VERIFIED_INPMUX;
        } else {
            *outAction = SENSORARRAY_RATE_ACTION_FATAL_STOP;
        }
    } else if (spiFail > (uint32_t)CONFIG_SENSORARRAY_AUTO_RATE_MAX_SPI_FAIL_PER_WINDOW) {
        *outCause = SENSORARRAY_RATE_CAUSE_SPI_FAIL;
        if (controller->currentAdsDr > controller->minAdsDr) {
            *outAction = SENSORARRAY_RATE_ACTION_DECREASE_ADS_DR;
        } else {
            *outAction = SENSORARRAY_RATE_ACTION_FATAL_STOP;
        }
    } else if (crcOrStatus > 0u) {
        *outCause = SENSORARRAY_RATE_CAUSE_CRC_OR_STATUS_FAIL;
        if (controller->muxSettleUs < 100u) {
            *outAction = SENSORARRAY_RATE_ACTION_INCREASE_MUX_SETTLE;
        } else if (!controller->verifiedInpmuxForced) {
            *outAction = SENSORARRAY_RATE_ACTION_FORCE_VERIFIED_INPMUX;
        } else {
            *outAction = SENSORARRAY_RATE_ACTION_DECREASE_ADS_DR;
        }
    } else if (queueUsedPercent > (uint32_t)CONFIG_SENSORARRAY_AUTO_RATE_QUEUE_HIGH_WATERMARK_PERCENT ||
               framesDropped > (uint32_t)CONFIG_SENSORARRAY_AUTO_RATE_MAX_DROP_PER_WINDOW ||
               queueFull > 0u || usbShort > 0u || usbFail > 0u) {
        *outCause = (usbShort || usbFail) ? SENSORARRAY_RATE_CAUSE_OUTPUT_SHORT_WRITE
                                          : SENSORARRAY_RATE_CAUSE_OUTPUT_QUEUE_FULL;
        if (controller->csvEnabled) {
            *outAction = SENSORARRAY_RATE_ACTION_DISABLE_CSV;
        } else if (!controller->compactBinaryOnly) {
            *outAction = SENSORARRAY_RATE_ACTION_USE_COMPACT_BINARY_ONLY;
        } else if (controller->outputDivider < (uint32_t)CONFIG_SENSORARRAY_AUTO_RATE_OUTPUT_DIV_MAX) {
            *outAction = SENSORARRAY_RATE_ACTION_INCREASE_OUTPUT_DIVIDER;
        } else if (controller->scanFramePeriodUs < (uint32_t)CONFIG_SENSORARRAY_AUTO_RATE_SCAN_FRAME_PERIOD_MAX_US) {
            *outAction = SENSORARRAY_RATE_ACTION_ADD_SCAN_FRAME_PERIOD;
        } else if (controller->currentAdsDr > controller->minAdsDr) {
            *outAction = SENSORARRAY_RATE_ACTION_DECREASE_ADS_DR;
        } else {
            *outAction = SENSORARRAY_RATE_ACTION_FATAL_STOP;
        }
    }

    if (*outAction == SENSORARRAY_RATE_ACTION_NONE) {
        controller->stableWindowCount++;
#if CONFIG_SENSORARRAY_AUTO_RATE_ENABLE_UPSHIFT
        const bool stable = queueFull == 0u && framesDropped == 0u && drdyTimeout == 0u &&
                            spiFail == 0u && crcOrStatus == 0u &&
                            queueUsedPercent < (uint32_t)CONFIG_SENSORARRAY_AUTO_RATE_QUEUE_LOW_WATERMARK_PERCENT;
        if (stable && controller->stableWindowCount >= (uint32_t)CONFIG_SENSORARRAY_AUTO_RATE_RECOVERY_WINDOWS) {
            if (controller->scanFramePeriodUs > 0u) {
                controller->scanFramePeriodUs /= 2u;
                *outAction = SENSORARRAY_RATE_ACTION_ADD_SCAN_FRAME_PERIOD;
            } else if (controller->outputDivider > 1u) {
                controller->outputDivider /= 2u;
                if (controller->outputDivider == 0u) {
                    controller->outputDivider = 1u;
                }
                *outAction = SENSORARRAY_RATE_ACTION_INCREASE_OUTPUT_DIVIDER;
            } else if (controller->currentAdsDr < controller->maxAdsDr) {
                controller->requestedAdsDr = (uint8_t)(controller->currentAdsDr + 1u);
                *outAction = SENSORARRAY_RATE_ACTION_DECREASE_ADS_DR;
            }
            if (*outAction != SENSORARRAY_RATE_ACTION_NONE) {
                controller->upshiftCount++;
                controller->stableWindowCount = 0u;
            }
        }
#endif
    } else {
        controller->stableWindowCount = 0u;
    }

    sensorarrayRateControllerMarkWindowStart(controller, perf, status);
    return ESP_OK;
#endif
}

esp_err_t sensorarrayRateControllerApplyAction(sensorarrayRateController_t *controller,
                                               sensorarrayRateAction_t action,
                                               sensorarrayRateCause_t cause,
                                               ads126xAdcHandle_t *ads)
{
    if (!controller) {
        return ESP_ERR_INVALID_ARG;
    }

    controller->lastAction = action;
    controller->lastCause = cause;

    switch (action) {
    case SENSORARRAY_RATE_ACTION_NONE:
        return ESP_OK;
    case SENSORARRAY_RATE_ACTION_DISABLE_CSV:
        controller->csvEnabled = false;
        controller->degradeCount++;
        return ESP_OK;
    case SENSORARRAY_RATE_ACTION_USE_COMPACT_BINARY_ONLY:
        controller->compactBinaryOnly = true;
        controller->csvEnabled = false;
        controller->degradeCount++;
        return ESP_OK;
    case SENSORARRAY_RATE_ACTION_INCREASE_OUTPUT_DIVIDER:
        if (controller->outputDivider < (uint32_t)CONFIG_SENSORARRAY_AUTO_RATE_OUTPUT_DIV_MAX) {
            controller->outputDivider *= 2u;
            if (controller->outputDivider == 0u) {
                controller->outputDivider = 1u;
            }
            if (controller->outputDivider > (uint32_t)CONFIG_SENSORARRAY_AUTO_RATE_OUTPUT_DIV_MAX) {
                controller->outputDivider = (uint32_t)CONFIG_SENSORARRAY_AUTO_RATE_OUTPUT_DIV_MAX;
            }
            controller->degradeCount++;
        }
        return ESP_OK;
    case SENSORARRAY_RATE_ACTION_ADD_SCAN_FRAME_PERIOD:
        if (controller->scanFramePeriodUs == 0u) {
            controller->scanFramePeriodUs = 500u;
        } else if (controller->scanFramePeriodUs < 1000u) {
            controller->scanFramePeriodUs = 1000u;
        } else {
            controller->scanFramePeriodUs *= 2u;
        }
        if (controller->scanFramePeriodUs > (uint32_t)CONFIG_SENSORARRAY_AUTO_RATE_SCAN_FRAME_PERIOD_MAX_US) {
            controller->scanFramePeriodUs = (uint32_t)CONFIG_SENSORARRAY_AUTO_RATE_SCAN_FRAME_PERIOD_MAX_US;
        }
        controller->degradeCount++;
        return ESP_OK;
    case SENSORARRAY_RATE_ACTION_DECREASE_ADS_DR: {
        if (controller->currentAdsDr <= controller->minAdsDr || !ads) {
            return ESP_OK;
        }
        uint8_t nextDr = (uint8_t)(controller->currentAdsDr - 1u);
        if (nextDr < controller->minAdsDr) {
            nextDr = controller->minAdsDr;
        }
        esp_err_t err = ads126xAdcEndFastFrame(ads);
        if (err == ESP_OK) {
            (void)ads126xAdcStopAdc1(ads);
            err = ads126xAdcConfigureVoltageMode(ads,
                                                 ads->pgaGain,
                                                 nextDr,
                                                 ads->enableStatusByte,
                                                 ads->crcMode != ADS126X_CRC_OFF);
        }
        if (err == ESP_OK) {
            uint8_t mode2 = 0u;
            err = ads126xAdcReadCoreRegisters(ads, NULL, NULL, &mode2, NULL, NULL);
            if (err == ESP_OK && (mode2 & 0x0Fu) != nextDr) {
                err = ESP_ERR_INVALID_RESPONSE;
            }
        }
        if (err == ESP_OK) {
            err = ads126xAdcStartAdc1(ads);
        }
        if (err == ESP_OK) {
            controller->currentAdsDr = nextDr;
            controller->requestedAdsDr = nextDr;
            controller->degradeCount++;
        }
        return err;
    }
    case SENSORARRAY_RATE_ACTION_INCREASE_MUX_SETTLE:
        if (controller->muxSettleUs == 0u) {
            controller->muxSettleUs = 5u;
        } else if (controller->muxSettleUs < 100u) {
            controller->muxSettleUs *= 2u;
            if (controller->muxSettleUs > 100u) {
                controller->muxSettleUs = 100u;
            }
        }
        controller->degradeCount++;
        return ESP_OK;
    case SENSORARRAY_RATE_ACTION_FORCE_VERIFIED_INPMUX:
        controller->verifiedInpmuxForced = true;
        controller->fastInpmuxEnabled = false;
        controller->degradeCount++;
        return ESP_OK;
    case SENSORARRAY_RATE_ACTION_ENTER_SAFE_PROFILE:
    case SENSORARRAY_RATE_ACTION_FATAL_STOP:
        controller->fatalStop = true;
        controller->degradeCount++;
        return ESP_OK;
    default:
        return ESP_ERR_INVALID_ARG;
    }
}

const char *sensorarrayRateActionName(sensorarrayRateAction_t action)
{
    switch (action) {
    case SENSORARRAY_RATE_ACTION_NONE: return "NONE";
    case SENSORARRAY_RATE_ACTION_INCREASE_OUTPUT_DIVIDER: return "INCREASE_OUTPUT_DIVIDER";
    case SENSORARRAY_RATE_ACTION_DISABLE_CSV: return "DISABLE_CSV";
    case SENSORARRAY_RATE_ACTION_USE_COMPACT_BINARY_ONLY: return "USE_COMPACT_BINARY_ONLY";
    case SENSORARRAY_RATE_ACTION_ADD_SCAN_FRAME_PERIOD: return "ADD_SCAN_FRAME_PERIOD";
    case SENSORARRAY_RATE_ACTION_DECREASE_ADS_DR: return "DECREASE_ADS_DR";
    case SENSORARRAY_RATE_ACTION_INCREASE_MUX_SETTLE: return "INCREASE_MUX_SETTLE";
    case SENSORARRAY_RATE_ACTION_FORCE_VERIFIED_INPMUX: return "FORCE_VERIFIED_INPMUX";
    case SENSORARRAY_RATE_ACTION_ENTER_SAFE_PROFILE: return "ENTER_SAFE_PROFILE";
    case SENSORARRAY_RATE_ACTION_FATAL_STOP: return "FATAL_STOP";
    default: return "UNKNOWN";
    }
}

const char *sensorarrayRateCauseName(sensorarrayRateCause_t cause)
{
    switch (cause) {
    case SENSORARRAY_RATE_CAUSE_NONE: return "NONE";
    case SENSORARRAY_RATE_CAUSE_OUTPUT_QUEUE_FULL: return "OUTPUT_QUEUE_FULL";
    case SENSORARRAY_RATE_CAUSE_OUTPUT_USB_SLOW: return "OUTPUT_USB_SLOW";
    case SENSORARRAY_RATE_CAUSE_OUTPUT_SHORT_WRITE: return "OUTPUT_SHORT_WRITE";
    case SENSORARRAY_RATE_CAUSE_DRDY_TIMEOUT: return "DRDY_TIMEOUT";
    case SENSORARRAY_RATE_CAUSE_SPI_FAIL: return "SPI_FAIL";
    case SENSORARRAY_RATE_CAUSE_CRC_OR_STATUS_FAIL: return "CRC_OR_STATUS_FAIL";
    case SENSORARRAY_RATE_CAUSE_SCAN_OVER_BUDGET: return "SCAN_OVER_BUDGET";
    case SENSORARRAY_RATE_CAUSE_FATAL_POLICY_ERROR: return "FATAL_POLICY_ERROR";
    default: return "UNKNOWN";
    }
}

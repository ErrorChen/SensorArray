#include "sensorarrayStatus.h"

#include <stddef.h>

void sensorarrayStatusResetFrame(sensorarrayStatusCounters_t *status)
{
    if (!status) {
        return;
    }
    status->frameStatusFlags = SENSORARRAY_FRAME_FLAG_OK;
}

void sensorarrayStatusRecord(sensorarrayStatusCounters_t *status,
                             sensorarrayStatusCode_t code,
                             uint32_t frameFlags)
{
    if (!status || code == SENSORARRAY_STATUS_OK) {
        return;
    }

    if (status->firstStatusCode == SENSORARRAY_STATUS_OK) {
        status->firstStatusCode = (uint32_t)code;
    }
    status->lastStatusCode = (uint32_t)code;
    status->frameStatusFlags |= frameFlags;

    switch (code) {
    case SENSORARRAY_STATUS_ADS_SPI_FAIL:
        status->adsSpiFailCount++;
        break;
    case SENSORARRAY_STATUS_ADS_DRDY_TIMEOUT:
        status->adsDrdyTimeoutCount++;
        break;
    case SENSORARRAY_STATUS_ADS_CRC_FAIL:
        status->adsCrcFailCount++;
        break;
    case SENSORARRAY_STATUS_ADS_STATUS_BYTE_BAD:
        status->adsStatusBadCount++;
        break;
    case SENSORARRAY_STATUS_ADS_REG_VERIFY_FAIL:
        status->adsRegVerifyFailCount++;
        break;
    case SENSORARRAY_STATUS_ADS_INPMUX_WRITE_FAIL:
        status->adsInpmuxWriteFailCount++;
        break;
    case SENSORARRAY_STATUS_ADS_DIRECT_READ_FAIL:
        status->adsDirectReadFailCount++;
        break;
    case SENSORARRAY_STATUS_TMUX_ROUTE_FAIL:
        status->tmuxRouteFailCount++;
        break;
    case SENSORARRAY_STATUS_TMUX_SW_POLICY_MISMATCH:
    case SENSORARRAY_STATUS_TMUX_SOURCE_FAIL:
    case SENSORARRAY_STATUS_MODE_POLICY_MISMATCH:
        status->tmuxPolicyMismatchCount++;
        break;
    case SENSORARRAY_STATUS_STREAM_QUEUE_FULL:
        status->queueFullCount++;
        break;
    case SENSORARRAY_STATUS_STREAM_FRAME_DROPPED:
        status->droppedFrameCount++;
        break;
    case SENSORARRAY_STATUS_RATE_OUTPUT_DECIMATED:
        status->outputDecimatedFrameCount++;
        break;
    case SENSORARRAY_STATUS_USB_STDOUT_BLOCKED:
        status->usbStdoutBlockedCount++;
        break;
    case SENSORARRAY_STATUS_USB_STDOUT_SHORT_WRITE:
        status->usbShortWriteCount++;
        break;
    case SENSORARRAY_STATUS_USB_STDOUT_WRITE_FAIL:
        status->usbWriteFailCount++;
        break;
    case SENSORARRAY_STATUS_RATE_ADS_DR_REDUCED:
    case SENSORARRAY_STATUS_RATE_MUX_SETTLE_INCREASED:
    case SENSORARRAY_STATUS_RATE_VERIFIED_MUX_FORCED:
    case SENSORARRAY_STATUS_RATE_SCAN_THROTTLED:
        status->rateControlDegradeCount++;
        break;
    case SENSORARRAY_STATUS_RATE_FATAL_STOP:
    case SENSORARRAY_STATUS_ADS_REF_POLICY_MISMATCH:
        status->fatalCount++;
        break;
    default:
        break;
    }
}

const char *sensorarrayStatusCodeName(sensorarrayStatusCode_t code)
{
    switch (code) {
    case SENSORARRAY_STATUS_OK: return "OK";
    case SENSORARRAY_STATUS_ADS_SPI_FAIL: return "ADS_SPI_FAIL";
    case SENSORARRAY_STATUS_ADS_DRDY_TIMEOUT: return "ADS_DRDY_TIMEOUT";
    case SENSORARRAY_STATUS_ADS_CRC_FAIL: return "ADS_CRC_FAIL";
    case SENSORARRAY_STATUS_ADS_REG_VERIFY_FAIL: return "ADS_REG_VERIFY_FAIL";
    case SENSORARRAY_STATUS_ADS_REF_POLICY_MISMATCH: return "ADS_REF_POLICY_MISMATCH";
    case SENSORARRAY_STATUS_ADS_GAIN_CHANGE_FAIL: return "ADS_GAIN_CHANGE_FAIL";
    case SENSORARRAY_STATUS_ADS_DMA_FALLBACK: return "ADS_DMA_FALLBACK";
    case SENSORARRAY_STATUS_ADS_INPMUX_WRITE_FAIL: return "ADS_INPMUX_WRITE_FAIL";
    case SENSORARRAY_STATUS_ADS_DIRECT_READ_FAIL: return "ADS_DIRECT_READ_FAIL";
    case SENSORARRAY_STATUS_ADS_STATUS_BYTE_BAD: return "ADS_STATUS_BYTE_BAD";
    case SENSORARRAY_STATUS_TMUX_ROUTE_FAIL: return "TMUX_ROUTE_FAIL";
    case SENSORARRAY_STATUS_TMUX_SW_POLICY_MISMATCH: return "TMUX_SW_POLICY_MISMATCH";
    case SENSORARRAY_STATUS_TMUX_SOURCE_FAIL: return "TMUX_SOURCE_FAIL";
    case SENSORARRAY_STATUS_STREAM_QUEUE_FULL: return "STREAM_QUEUE_FULL";
    case SENSORARRAY_STATUS_STREAM_FRAME_DROPPED: return "STREAM_FRAME_DROPPED";
    case SENSORARRAY_STATUS_USB_STDOUT_BLOCKED: return "USB_STDOUT_BLOCKED";
    case SENSORARRAY_STATUS_USB_STDOUT_WRITE_FAIL: return "USB_STDOUT_WRITE_FAIL";
    case SENSORARRAY_STATUS_USB_STDOUT_SHORT_WRITE: return "USB_STDOUT_SHORT_WRITE";
    case SENSORARRAY_STATUS_SPI_BUS_ACQUIRE_FAIL: return "SPI_BUS_ACQUIRE_FAIL";
    case SENSORARRAY_STATUS_SPI_BUS_RELEASE_FAIL: return "SPI_BUS_RELEASE_FAIL";
    case SENSORARRAY_STATUS_MODE_POLICY_MISMATCH: return "MODE_POLICY_MISMATCH";
    case SENSORARRAY_STATUS_RATE_OUTPUT_DECIMATED: return "RATE_OUTPUT_DECIMATED";
    case SENSORARRAY_STATUS_RATE_SCAN_THROTTLED: return "RATE_SCAN_THROTTLED";
    case SENSORARRAY_STATUS_RATE_ADS_DR_REDUCED: return "RATE_ADS_DR_REDUCED";
    case SENSORARRAY_STATUS_RATE_MUX_SETTLE_INCREASED: return "RATE_MUX_SETTLE_INCREASED";
    case SENSORARRAY_STATUS_RATE_VERIFIED_MUX_FORCED: return "RATE_VERIFIED_MUX_FORCED";
    case SENSORARRAY_STATUS_RATE_SAFE_PROFILE_ENTERED: return "RATE_SAFE_PROFILE_ENTERED";
    case SENSORARRAY_STATUS_RATE_FATAL_STOP: return "RATE_FATAL_STOP";
    case SENSORARRAY_STATUS_INTERNAL_ASSERT_FAIL: return "INTERNAL_ASSERT_FAIL";
    default: return "UNKNOWN";
    }
}

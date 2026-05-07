#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    SENSORARRAY_STATUS_OK                         = 0x0000,

    SENSORARRAY_STATUS_ADS_SPI_FAIL              = 0x1001,
    SENSORARRAY_STATUS_ADS_DRDY_TIMEOUT          = 0x1002,
    SENSORARRAY_STATUS_ADS_CRC_FAIL              = 0x1003,
    SENSORARRAY_STATUS_ADS_REG_VERIFY_FAIL       = 0x1004,
    SENSORARRAY_STATUS_ADS_REF_POLICY_MISMATCH   = 0x1005,
    SENSORARRAY_STATUS_ADS_GAIN_CHANGE_FAIL      = 0x1006,
    SENSORARRAY_STATUS_ADS_DMA_FALLBACK          = 0x1007,
    SENSORARRAY_STATUS_ADS_INPMUX_WRITE_FAIL     = 0x1008,
    SENSORARRAY_STATUS_ADS_DIRECT_READ_FAIL      = 0x1009,
    SENSORARRAY_STATUS_ADS_STATUS_BYTE_BAD       = 0x100A,

    SENSORARRAY_STATUS_TMUX_ROUTE_FAIL           = 0x2001,
    SENSORARRAY_STATUS_TMUX_SW_POLICY_MISMATCH   = 0x2002,
    SENSORARRAY_STATUS_TMUX_SOURCE_FAIL          = 0x2003,

    SENSORARRAY_STATUS_STREAM_QUEUE_FULL         = 0x3001,
    SENSORARRAY_STATUS_STREAM_FRAME_DROPPED      = 0x3002,
    SENSORARRAY_STATUS_USB_STDOUT_BLOCKED        = 0x3003,
    SENSORARRAY_STATUS_USB_STDOUT_WRITE_FAIL     = 0x3004,
    SENSORARRAY_STATUS_USB_STDOUT_SHORT_WRITE    = 0x3005,
    SENSORARRAY_STATUS_STREAM_INTERNAL_ERROR     = 0x3006,
    SENSORARRAY_STATUS_OUT_STACK_LOW             = 0x3007,
    SENSORARRAY_STATUS_OUT_STACK_CRITICAL        = 0x3008,
    SENSORARRAY_STATUS_BINARY_TEXT_SUPPRESSED    = 0x3009,
    SENSORARRAY_STATUS_BINARY_WRITE_PARTIAL      = 0x300A,

    SENSORARRAY_STATUS_SPI_BUS_ACQUIRE_FAIL      = 0x4001,
    SENSORARRAY_STATUS_SPI_BUS_RELEASE_FAIL      = 0x4002,

    SENSORARRAY_STATUS_MODE_POLICY_MISMATCH      = 0x5001,

    SENSORARRAY_STATUS_RATE_OUTPUT_DECIMATED     = 0x6001,
    SENSORARRAY_STATUS_RATE_SCAN_THROTTLED       = 0x6002,
    SENSORARRAY_STATUS_RATE_ADS_DR_REDUCED       = 0x6003,
    SENSORARRAY_STATUS_RATE_MUX_SETTLE_INCREASED = 0x6004,
    SENSORARRAY_STATUS_RATE_VERIFIED_MUX_FORCED  = 0x6005,
    SENSORARRAY_STATUS_RATE_SAFE_PROFILE_ENTERED = 0x6006,
    SENSORARRAY_STATUS_RATE_FATAL_STOP           = 0x6007,

    SENSORARRAY_STATUS_INTERNAL_ASSERT_FAIL      = 0x7FFF
} sensorarrayStatusCode_t;

#define SENSORARRAY_FRAME_FLAG_OK                 0x00000000u
#define SENSORARRAY_FRAME_FLAG_ADS_ERROR          0x00000001u
#define SENSORARRAY_FRAME_FLAG_DRDY_TIMEOUT       0x00000002u
#define SENSORARRAY_FRAME_FLAG_SPI_ERROR          0x00000004u
#define SENSORARRAY_FRAME_FLAG_TMUX_ERROR         0x00000008u
#define SENSORARRAY_FRAME_FLAG_REF_POLICY_ERROR   0x00000010u
#define SENSORARRAY_FRAME_FLAG_QUEUE_DROPPED      0x00000020u
#define SENSORARRAY_FRAME_FLAG_OUTPUT_THROTTLED   0x00000040u
#define SENSORARRAY_FRAME_FLAG_GAIN_CHANGED       0x00000080u
#define SENSORARRAY_FRAME_FLAG_CLIPPED            0x00000100u
#define SENSORARRAY_FRAME_FLAG_RATE_LIMITED       0x00000200u
#define SENSORARRAY_FRAME_FLAG_OUTPUT_DECIMATED   0x00000400u
#define SENSORARRAY_FRAME_FLAG_SCAN_THROTTLED     0x00000800u
#define SENSORARRAY_FRAME_FLAG_ADS_DR_REDUCED     0x00001000u
#define SENSORARRAY_FRAME_FLAG_RECOVERY_ACTIVE    0x00002000u
#define SENSORARRAY_FRAME_FLAG_FATAL              0x80000000u

typedef struct {
    uint32_t frameStatusFlags;
    uint32_t firstStatusCode;
    uint32_t lastStatusCode;

    uint32_t adsSpiFailCount;
    uint32_t adsDrdyTimeoutCount;
    uint32_t adsCrcFailCount;
    uint32_t adsStatusBadCount;
    uint32_t adsRegVerifyFailCount;
    uint32_t adsInpmuxWriteFailCount;
    uint32_t adsDirectReadFailCount;

    uint32_t tmuxRouteFailCount;
    uint32_t tmuxPolicyMismatchCount;

    uint32_t queueFullCount;
    uint32_t droppedFrameCount;
    uint32_t outputDecimatedFrameCount;
    uint32_t usbStdoutBlockedCount;
    uint32_t usbShortWriteCount;
    uint32_t usbWriteFailCount;

    uint32_t rateControlDegradeCount;
    uint32_t rateControlUpshiftCount;
    uint32_t fatalCount;
} sensorarrayStatusCounters_t;

void sensorarrayStatusResetFrame(sensorarrayStatusCounters_t *status);
void sensorarrayStatusRecord(sensorarrayStatusCounters_t *status,
                             sensorarrayStatusCode_t code,
                             uint32_t frameFlags);
const char *sensorarrayStatusCodeName(sensorarrayStatusCode_t code);

#ifdef __cplusplus
}
#endif

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "sdkconfig.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "ads126xAdc.h"
#include "sensorarrayRateControl.h"
#include "sensorarrayStatus.h"
#include "sensorarrayVoltageScan.h"
#include "tmuxSwitch.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SENSORARRAY_VOLTAGE_COMPACT_MAGIC 0x31434153u
#define SENSORARRAY_VOLTAGE_COMPACT_VERSION 1u
#define SENSORARRAY_VOLTAGE_COMPACT_TYPE_ADS126X_UV 0x1261u
#define SENSORARRAY_VOLTAGE_COMPACT_SIZE 312u

#ifndef CONFIG_SENSORARRAY_BINARY_ASSERT_FRAME_SIZE
#define CONFIG_SENSORARRAY_BINARY_ASSERT_FRAME_SIZE 1
#endif

typedef struct __attribute__((packed)) {
    uint32_t magic;                  /* 0x31434153, bytes "SAC1". */
    uint16_t version;                /* 1. */
    uint16_t frameType;              /* 0x1261, ADS126x microvolts. */
    uint32_t sequence;
    uint64_t timestampUs;
    uint32_t scanDurationUs;
    uint32_t statusFlags;
    uint32_t firstStatusCode;
    uint32_t lastStatusCode;
    uint16_t droppedFrames;          /* Saturated passive drop count. */
    uint16_t outputDecimatedFrames;  /* Saturated active decimation count. */
    uint64_t validMask;
    int32_t microvolts[64];          /* [0]=S1D1, [1]=S1D2, ..., [63]=S8D8. */
    uint8_t adsDr;
    uint8_t outputDivider;
    uint16_t reserved;
    uint32_t crc32;                  /* IEEE CRC32 over all previous bytes. */
} sensorarrayVoltageCompactFrame_t;

#if CONFIG_SENSORARRAY_BINARY_ASSERT_FRAME_SIZE
#if defined(__cplusplus)
static_assert(sizeof(sensorarrayVoltageCompactFrame_t) == SENSORARRAY_VOLTAGE_COMPACT_SIZE,
              "sensorarrayVoltageCompactFrame_t size mismatch");
static_assert(SENSORARRAY_VOLTAGE_COMPACT_SIZE == 312,
              "SAC1 compact voltage frame must remain 312 bytes for host GUI compatibility");
static_assert(offsetof(sensorarrayVoltageCompactFrame_t, crc32) ==
              sizeof(sensorarrayVoltageCompactFrame_t) - sizeof(uint32_t),
              "crc32 must be the final field in sensorarrayVoltageCompactFrame_t");
#else
_Static_assert(sizeof(sensorarrayVoltageCompactFrame_t) == SENSORARRAY_VOLTAGE_COMPACT_SIZE,
               "sensorarrayVoltageCompactFrame_t size mismatch");
_Static_assert(SENSORARRAY_VOLTAGE_COMPACT_SIZE == 312,
               "SAC1 compact voltage frame must remain 312 bytes for host GUI compatibility");
_Static_assert(offsetof(sensorarrayVoltageCompactFrame_t, crc32) ==
               sizeof(sensorarrayVoltageCompactFrame_t) - sizeof(uint32_t),
               "crc32 must be the final field in sensorarrayVoltageCompactFrame_t");
#endif
#endif

typedef struct {
    sensorarrayVoltageFrame_t frame;
    sensorarrayRateAction_t rateAction;
    sensorarrayRateCause_t rateCause;
} sensorarrayVoltageStreamFrame_t;

typedef struct {
    ads126xAdcHandle_t *ads;
    tmux1108Source_t swSource;
    const char *modeName;
    const char *swName;
    uint8_t expectedAdsRefmux;
    bool useAdsInternalRef;
    bool useAdsVbias;
    bool routePolicyOk;
    bool adsPolicyOk;
} sensorarrayVoltageStreamConfig_t;

esp_err_t sensorarrayVoltageStreamStart(const sensorarrayVoltageStreamConfig_t *config);
QueueHandle_t sensorarrayVoltageStreamQueue(void);
sensorarrayStatusCounters_t *sensorarrayVoltageStreamStatus(void);
sensorarrayPerfCounters_t *sensorarrayVoltageStreamPerf(void);

uint32_t sensorarrayVoltageCompactFrameCrc32(const sensorarrayVoltageCompactFrame_t *frame);
void sensorarrayVoltageCompactFrameFromScan(const sensorarrayVoltageFrame_t *scan,
                                            sensorarrayVoltageCompactFrame_t *out);

#ifdef __cplusplus
}
#endif

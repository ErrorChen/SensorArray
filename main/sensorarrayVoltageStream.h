#pragma once

#include <stdbool.h>
#include <stdint.h>

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

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint16_t version;
    uint16_t frameType;
    uint32_t sequence;
    uint64_t timestampUs;
    uint32_t scanDurationUs;
    uint32_t statusFlags;
    uint32_t firstStatusCode;
    uint32_t lastStatusCode;
    uint32_t droppedFrames;
    uint32_t outputDecimatedFrames;
    uint64_t validMask;
    int32_t microvolts[64];
    uint8_t adsDr;
    uint8_t outputDivider;
    uint16_t reserved;
    uint32_t crc32;
} sensorarrayVoltageCompactFrame_t;

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

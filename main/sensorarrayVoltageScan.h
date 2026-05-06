#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#include "ads126xAdc.h"
#include "sensorarrayPerf.h"
#include "sensorarrayStatus.h"
#include "tmuxSwitch.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SENSORARRAY_VOLTAGE_SCAN_ROWS 8u
#define SENSORARRAY_VOLTAGE_SCAN_COLS 8u

typedef struct {
    uint32_t sequence;
    uint64_t timestampUs;
    uint32_t scanDurationUs;
    uint32_t statusFlags;
    uint32_t firstStatusCode;
    uint32_t lastStatusCode;
    uint32_t droppedFrames;
    uint32_t outputDecimatedFrames;
    uint64_t validMask;
    uint8_t adsDr;
    uint8_t outputDivider;
    uint16_t rateControlLevel;
    uint32_t scanFramePeriodUs;
    uint32_t pointStatus[SENSORARRAY_VOLTAGE_SCAN_ROWS][SENSORARRAY_VOLTAGE_SCAN_COLS];
    int32_t microvolts[SENSORARRAY_VOLTAGE_SCAN_ROWS][SENSORARRAY_VOLTAGE_SCAN_COLS];
    int32_t raw[SENSORARRAY_VOLTAGE_SCAN_ROWS][SENSORARRAY_VOLTAGE_SCAN_COLS];
    uint8_t gain[SENSORARRAY_VOLTAGE_SCAN_ROWS][SENSORARRAY_VOLTAGE_SCAN_COLS];
    esp_err_t err[SENSORARRAY_VOLTAGE_SCAN_ROWS][SENSORARRAY_VOLTAGE_SCAN_COLS];
    bool clipped[SENSORARRAY_VOLTAGE_SCAN_ROWS][SENSORARRAY_VOLTAGE_SCAN_COLS];
} sensorarrayVoltageFrame_t;

esp_err_t sensorarrayVoltageScanApplyRouteFast(uint8_t sColumn,
                                               uint8_t dLine,
                                               uint32_t rowSettleUs,
                                               uint32_t pathSettleUs);

esp_err_t sensorarrayVoltageScanApplyRouteFastWithSource(uint8_t sColumn,
                                                         uint8_t dLine,
                                                         tmux1108Source_t swSource,
                                                         uint32_t rowSettleUs,
                                                         uint32_t pathSettleUs);

esp_err_t sensorarrayVoltageScanOneFrame(ads126xAdcHandle_t *ads,
                                         const uint8_t gainTable[SENSORARRAY_VOLTAGE_SCAN_ROWS]
                                                                [SENSORARRAY_VOLTAGE_SCAN_COLS],
                                         sensorarrayVoltageFrame_t *outFrame);

esp_err_t sensorarrayVoltageScanOneFrameWithSource(ads126xAdcHandle_t *ads,
                                                   const uint8_t gainTable[SENSORARRAY_VOLTAGE_SCAN_ROWS]
                                                                          [SENSORARRAY_VOLTAGE_SCAN_COLS],
                                                   tmux1108Source_t swSource,
                                                   sensorarrayVoltageFrame_t *outFrame);

void sensorarrayVoltageScanSetFastRuntimeOptions(uint32_t muxSettleUs, bool verifiedInpmuxForced);

esp_err_t sensorarrayVoltageScanOneFrameFastAds(ads126xAdcHandle_t *ads,
                                                tmux1108Source_t swSource,
                                                sensorarrayVoltageFrame_t *outFrame,
                                                sensorarrayStatusCounters_t *status,
                                                sensorarrayPerfCounters_t *perf);

#ifdef __cplusplus
}
#endif

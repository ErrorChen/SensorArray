#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#include "sensorarrayTypes.h"

typedef enum {
    SENSORARRAY_FDC_SWEEP_REASON_BOOT = 0,
    SENSORARRAY_FDC_SWEEP_REASON_QUICK,
    SENSORARRAY_FDC_SWEEP_REASON_AMPLITUDE_FAULT,
    SENSORARRAY_FDC_SWEEP_REASON_WATCHDOG_FAULT,
    SENSORARRAY_FDC_SWEEP_REASON_INVALID_STREAK,
    SENSORARRAY_FDC_SWEEP_REASON_RAW_ZERO,
    SENSORARRAY_FDC_SWEEP_REASON_SATURATED,
    SENSORARRAY_FDC_SWEEP_REASON_FREQUENCY_MARGIN,
    SENSORARRAY_FDC_SWEEP_REASON_DEGLITCH_MARGIN,
} sensorarrayFdcSweepReason_t;

typedef enum {
    SENSORARRAY_FDC_FAULT_NONE = 0,
    SENSORARRAY_FDC_FAULT_ZERO_RAW,
    SENSORARRAY_FDC_FAULT_NO_OSCILLATION,
    SENSORARRAY_FDC_FAULT_STATUS_SLEEP,
    SENSORARRAY_FDC_FAULT_NOT_CONVERTING,
    SENSORARRAY_FDC_FAULT_WATCHDOG,
    SENSORARRAY_FDC_FAULT_AMPLITUDE,
    SENSORARRAY_FDC_FAULT_TIMEOUT,
    SENSORARRAY_FDC_FAULT_ALL_INVALID_FRAME,
} sensorarrayFdcFaultReason_t;

typedef struct {
    uint8_t deglitchCode;
    uint32_t deglitchBandwidthHz;
    const char *deglitchName;
} sensorarrayFdcDeglitchCandidate_t;

typedef struct {
    bool valid;
    uint8_t deglitchReq;
    const char *deglitchName;
    uint32_t deglitchBandwidthHz;
    bool deglitchBandwidthOk;
    double deglitchMarginRatio;

    bool highCurrentReq;
    bool highCurrentReadback;

    uint16_t driveCurrentReq;
    uint16_t driveCurrentNorm;
    uint16_t driveCurrentReadback;

    uint8_t activeChannelReadback;
    uint16_t statusReg;
    uint16_t configReg;
    uint16_t muxConfigReg;
    uint16_t clockDividersReg;

    uint8_t finSelCode;
    uint8_t finFactor;
    uint16_t frefDivider;
    uint32_t effectiveFclkHz;
    double effectiveFrefHz;

    uint32_t raw28Min;
    uint32_t raw28Max;
    uint32_t raw28Mean;
    uint32_t raw28Last;
    uint32_t validSampleCount;
    uint32_t invalidSampleCount;
    uint32_t saturatedCount;
    uint32_t watchdogCount;
    uint32_t amplitudeFaultCount;
    uint32_t zeroRawCount;
    uint32_t timeoutCount;

    double medianFreqHz;
    double freqSpreadHz;
    double relativeSpread;
    double frequencyHz;
    double frequencyMarginHz;
    bool stable;
    bool saturated;
    bool selected;
} sensorarrayFdcSweepCandidateResult_t;

typedef struct {
    bool valid;
    sensorarrayFdcDeviceId_t devId;
    Fdc2214CapChannel_t channel;

    uint8_t selectedDeglitchCode;
    uint32_t selectedDeglitchBandwidthHz;
    const char *selectedDeglitchName;

    uint16_t selectedDriveCurrent;
    bool selectedHighCurrent;

    uint16_t selectedConfig;
    uint16_t selectedMuxConfig;
    uint16_t selectedClockDividers;

    uint32_t selectedRaw28;
    double selectedFrequencyHz;

    uint32_t candidateCount;
    uint32_t elapsedMs;

    esp_err_t lastErr;
    const char *reason;
} sensorarrayFdcSweepResult_t;

esp_err_t sensorarrayFdcSweepRunChannel(sensorarrayState_t *state,
                                        sensorarrayFdcDeviceId_t devId,
                                        Fdc2214CapChannel_t channel,
                                        const char *reason,
                                        sensorarrayFdcSweepResult_t *outResult);

esp_err_t sensorarrayFdcSweepRunDevice(sensorarrayState_t *state,
                                       sensorarrayFdcDeviceId_t devId,
                                       uint8_t channelMask,
                                       const char *reason);

esp_err_t sensorarrayFdcSweepRunBoot(sensorarrayState_t *state);

sensorarrayFdcCellCalibration_t *sensorarrayFdcSweepGetCellCalibration(uint8_t sIndex,
                                                                       uint8_t dIndex);

esp_err_t sensorarrayFdcSweepMeasureCell(sensorarrayState_t *state,
                                         uint8_t sIndex,
                                         uint8_t dIndex,
                                         uint32_t *outRaw28,
                                         bool *outValid);

esp_err_t sensorarrayFdcSweepForceFullSweepCell(sensorarrayState_t *state,
                                                uint8_t sIndex,
                                                uint8_t dIndex);

esp_err_t sensorarrayFdcSweepRunFullRescueAll(sensorarrayState_t *state, const char *reason);
esp_err_t sensorarrayFdcSweepRunFullRescueCell(sensorarrayState_t *state,
                                               uint8_t sIndex,
                                               uint8_t dIndex,
                                               const char *reason);

esp_err_t sensorarrayFdcSweepRequestForceFullSweepCell(uint8_t sIndex, uint8_t dIndex);
esp_err_t sensorarrayFdcSweepRequestForceFullSweepAll(void);
bool sensorarrayFdcSweepConsumeForceFullSweepAll(void);
void sensorarrayFdcSweepReportAllInvalidFrame(uint64_t validMask,
                                              uint64_t errorMask,
                                              uint32_t zeroRawCount);
bool sensorarrayFdcSweepIsRescueInProgress(void);
esp_err_t sensorarrayFdcSweepDumpAllDeviceRegs(sensorarrayState_t *state,
                                               const char *stage,
                                               const char *reason);

esp_err_t sensorarrayFdcSweepApplyResult(sensorarrayState_t *state,
                                         const sensorarrayFdcSweepResult_t *result);

esp_err_t sensorarrayFdcSweepApplyDirectSafeLock(sensorarrayState_t *state,
                                                 sensorarrayFdcDeviceId_t devId,
                                                 Fdc2214CapChannel_t channel,
                                                 uint8_t deglitchCode,
                                                 uint16_t driveCurrent,
                                                 bool highCurrent,
                                                 const char *reason);

esp_err_t sensorarrayFdcSweepRestoreAutoscan(sensorarrayState_t *state,
                                             sensorarrayFdcDeviceId_t devId,
                                             const char *reason);

const char *sensorarrayFdcSweepReasonName(sensorarrayFdcSweepReason_t reason);
const char *sensorarrayFdcSweepFaultReasonName(sensorarrayFdcFaultReason_t reason);

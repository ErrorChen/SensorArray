#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#include "sensorarrayTypes.h"

sensorarrayFdcDeviceState_t *sensorarrayMeasureGetFdcState(sensorarrayState_t *state,
                                                            sensorarrayFdcDeviceId_t devId);

sensorarrayFdcDeviceState_t *sensorarrayMeasureGetFdcStateForDLine(sensorarrayState_t *state,
                                                                    uint8_t dLine,
                                                                    const sensorarrayFdcDLineMap_t **outMap);

esp_err_t sensorarrayMeasureSetSelaPath(sensorarrayState_t *state,
                                        sensorarraySelaRoute_t selaRoute,
                                        uint32_t settleDelayMs,
                                        const char *stage,
                                        const char *label);

esp_err_t sensorarrayMeasureApplyRouteLevels(sensorarrayState_t *state,
                                             uint8_t sColumn,
                                             uint8_t dLine,
                                             sensorarrayDebugPath_t path,
                                             tmux1108Source_t swSource,
                                             sensorarraySelaRoute_t selaRoute,
                                             bool selBLevel,
                                             uint32_t delayAfterRowMs,
                                             uint32_t delayAfterSelAMs,
                                             uint32_t delayAfterSelBMs,
                                             uint32_t delayAfterSwMs,
                                             const char *label);

esp_err_t sensorarrayMeasureApplyRoute(sensorarrayState_t *state,
                                       uint8_t sColumn,
                                       uint8_t dLine,
                                       sensorarrayPath_t path,
                                       tmux1108Source_t swSource,
                                       const char **outMapLabel);

// Dedicated S5D5 debug route: S5 -> D5 -> secondary FDC branch, capacitive SELB path.
esp_err_t sensorarrayMeasureApplyS5d5SecondaryCapRouteExact(sensorarrayState_t *state, const char **outMapLabel);

esp_err_t sensorarrayMeasureReadAdsRawWithRetry(sensorarrayState_t *state,
                                                int32_t *outRaw,
                                                uint8_t retryCount,
                                                bool *outTimedOut,
                                                uint8_t *outStatusByte);

esp_err_t sensorarrayMeasureReadAdsPairUv(sensorarrayState_t *state,
                                          const sensorarrayAdsReadPolicy_t *policy,
                                          uint8_t muxp,
                                          uint8_t muxn,
                                          bool discardFirst,
                                          int32_t *outRaw,
                                          int32_t *outUv,
                                          uint8_t *outStatusByte);

esp_err_t sensorarrayMeasureReadAdsUv(sensorarrayState_t *state,
                                      const sensorarrayAdsReadPolicy_t *policy,
                                      uint8_t dLine,
                                      bool discardFirst,
                                      int32_t *outRaw,
                                      int32_t *outUv);

sensorarrayResConvertResult_t sensorarrayMeasureTryResistanceMohm(int32_t uv, int32_t *outMohm);
const char *sensorarrayMeasureDividerModelStatus(int32_t uv, int32_t *outMohm, bool *outHaveMohm);

esp_err_t sensorarrayMeasureReadFdcSample(Fdc2214CapDevice_t *dev,
                                          Fdc2214CapChannel_t ch,
                                          bool discardFirst,
                                          Fdc2214CapSample_t *outSample);
esp_err_t sensorarrayMeasureReadFdcSampleDiag(Fdc2214CapDevice_t *dev,
                                              Fdc2214CapChannel_t ch,
                                              bool discardFirst,
                                              bool idOk,
                                              bool configOk,
                                              sensorarrayFdcReadDiag_t *outDiag);
esp_err_t sensorarrayMeasureReadFdcSampleDiagRelaxed(Fdc2214CapDevice_t *dev,
                                                      Fdc2214CapChannel_t ch,
                                                      bool discardFirst,
                                                      bool idOk,
                                                      bool configOk,
                                                      sensorarrayFdcReadDiag_t *outDiag);
const char *sensorarrayMeasureFdcSampleStatusName(sensorarrayFdcSampleStatus_t status);

typedef struct {
    uint32_t raw28;
    uint32_t refClockHz;
    sensorarrayFdcRefClockQuality_t refClockQuality;
    Fdc2214CapClockDividerInfo_t clockDividerInfo;
    double fClkHz;
    double fRefHz;
    double fInHz;
    double restoredSensorFrequencyHz;
} sensorarrayFdcFrequencyRestore_t;

const char *sensorarrayMeasureFdcRefClockQualityName(sensorarrayFdcRefClockQuality_t quality);
bool sensorarrayMeasureFdcRestoreFrequencyWithClockInfo(uint32_t raw28,
                                                        uint32_t refClockHz,
                                                        sensorarrayFdcRefClockQuality_t refClockQuality,
                                                        const Fdc2214CapClockDividerInfo_t *clockDividerInfo,
                                                        sensorarrayFdcFrequencyRestore_t *outRestore);
bool sensorarrayMeasureFdcRestoreFrequency(const sensorarrayFdcDeviceState_t *fdcState,
                                           uint32_t raw28,
                                           sensorarrayFdcFrequencyRestore_t *outRestore);

// Legacy helper kept for compatibility: assumes FIN_SEL=1 and FREF_DIVIDER=1.
double sensorarrayMeasureFdcRawToFrequencyHz(uint32_t raw28, uint32_t refClockHz);
bool sensorarrayMeasureFdcComputeCapacitancePf(double frequencyHz, double inductorValueUh, double *outCapPf);
bool sensorarrayMeasureFdcTryCapacitancePf(double frequencyHz, uint32_t inductorUh, double *outCapPf);

esp_err_t sensorarrayMeasureAdsReadRegister(sensorarrayState_t *state, uint8_t reg, uint8_t *outValue);
esp_err_t sensorarrayMeasureReadAdsKeyRegisterSnapshot(sensorarrayState_t *state,
                                                       sensorarrayAdsRegSnapshot_t *outSnapshot);
esp_err_t sensorarrayMeasureDumpAdsKeyRegisters(sensorarrayState_t *state, const char *stage);

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#include "sensorarrayTypes.h"

typedef enum {
    SENSORARRAY_FDC_DISCOVERY_NO_ACK = 0,
    SENSORARRAY_FDC_DISCOVERY_ACK_BUT_READ_FAILED,
    SENSORARRAY_FDC_DISCOVERY_ID_OK,
    SENSORARRAY_FDC_DISCOVERY_UNEXPECTED_ID,
} sensorarrayFdcDiscoveryStatus_t;

typedef struct {
    sensorarrayFdcDiscoveryStatus_t status;
    esp_err_t ackErr;
    esp_err_t idErr;
    bool ack;
    bool haveManufacturerId;
    bool haveDeviceId;
    uint16_t manufacturerId;
    uint16_t deviceId;
} sensorarrayFdcProbeDiag_t;

uint8_t sensorarrayBringupNormalizeFdcChannels(uint8_t channels);

void sensorarrayBringupResetFdcState(sensorarrayFdcDeviceState_t *fdcState,
                                     const char *label,
                                     uint8_t i2cAddr);

bool sensorarrayBringupParseI2cAddress(uint32_t configuredAddress, uint8_t *outAddress);

esp_err_t sensorarrayBringupAdsSetRefMux(sensorarrayState_t *state, uint8_t refmuxValue);
esp_err_t sensorarrayBringupInitAds(sensorarrayState_t *state);
esp_err_t ads126x_enable_ref_for_resistance_mode(sensorarrayState_t *state);
esp_err_t ads126x_disable_ref_for_ground_mode(sensorarrayState_t *state);
esp_err_t sensorarrayBringupPrepareAdsRefPath(sensorarrayState_t *state);
esp_err_t sensorarrayBringupVerifyAdsRefAnalog(sensorarrayState_t *state);

void sensorarrayBringupInitFdcDiag(sensorarrayFdcInitDiag_t *diag);
void sensorarrayBringupApplyFdcInitResult(sensorarrayFdcDeviceState_t *fdcState,
                                          Fdc2214CapDevice_t *deviceHandle,
                                          esp_err_t initErr,
                                          const sensorarrayFdcInitDiag_t *diag);
void sensorarrayBringupProbeFdcBus(const sensorarrayFdcDeviceState_t *fdcState);
esp_err_t sensorarrayBringupProbeFdcAddress(const BoardSupportI2cCtx_t *i2cCtx,
                                            uint8_t i2cAddr,
                                            uint16_t *outManufacturerId);
esp_err_t sensorarrayBringupReadFdcIdsRaw(const BoardSupportI2cCtx_t *i2cCtx,
                                          uint8_t i2cAddr,
                                          uint16_t *outManufacturerId,
                                          uint16_t *outDeviceId);
const char *sensorarrayBringupFdcDiscoveryStatusName(sensorarrayFdcDiscoveryStatus_t status);
esp_err_t sensorarrayBringupProbeFdcCandidate(const BoardSupportI2cCtx_t *i2cCtx,
                                              uint8_t i2cAddr,
                                              sensorarrayFdcProbeDiag_t *outDiag);

esp_err_t sensorarrayBringupInitFdcDevice(const BoardSupportI2cCtx_t *i2cCtx,
                                          uint8_t i2cAddr,
                                          uint8_t channels,
                                          Fdc2214CapDevice_t **outDev,
                                          sensorarrayFdcInitDiag_t *outDiag);
esp_err_t sensorarrayBringupInitFdcSingleChannel(const BoardSupportI2cCtx_t *i2cCtx,
                                                  uint8_t i2cAddr,
                                                  Fdc2214CapChannel_t channel,
                                                  Fdc2214CapDevice_t **outDev,
                                                  sensorarrayFdcInitDiag_t *outDiag);

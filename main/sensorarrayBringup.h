#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#include "sensorarrayTypes.h"

uint8_t sensorarrayBringupNormalizeFdcChannels(uint8_t channels);

void sensorarrayBringupResetFdcState(sensorarrayFdcDeviceState_t *fdcState,
                                     const char *label,
                                     uint8_t i2cAddr);

bool sensorarrayBringupParseI2cAddress(uint32_t configuredAddress, uint8_t *outAddress);

esp_err_t sensorarrayBringupAdsSetRefMux(sensorarrayState_t *state, uint8_t refmuxValue);
esp_err_t sensorarrayBringupInitAds(sensorarrayState_t *state);
esp_err_t sensorarrayBringupPrepareAdsRefPath(sensorarrayState_t *state);

void sensorarrayBringupInitFdcDiag(sensorarrayFdcInitDiag_t *diag);
void sensorarrayBringupProbeFdcBus(const sensorarrayFdcDeviceState_t *fdcState);
esp_err_t sensorarrayBringupProbeFdcAddress(const BoardSupportI2cCtx_t *i2cCtx,
                                            uint8_t i2cAddr,
                                            uint16_t *outManufacturerId);
esp_err_t sensorarrayBringupReadFdcIdsRaw(const BoardSupportI2cCtx_t *i2cCtx,
                                          uint8_t i2cAddr,
                                          uint16_t *outManufacturerId,
                                          uint16_t *outDeviceId);

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

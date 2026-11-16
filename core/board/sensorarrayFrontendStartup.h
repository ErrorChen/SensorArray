#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"
#include "sensorarrayTypes.h"

/* 本轮新增: boot-only startup orchestration. ADS qualification on MISC_CORE
 * overlaps the existing two FDC I2C startup scans on the caller's core. The
 * returned success releases ADS ownership before engines/route/Gap initialize.
 * A failure is not cancellation: the persistent boot job is never reused. */
esp_err_t sensorarrayFrontendStartupRun(sensorarrayState_t *state,
    bool primaryAddressValid, bool secondaryAddressValid, uint8_t channels);

/* 本轮新增: read-only published boot result; available before READY. */
esp_err_t sensorarrayFrontendStartupFormatSpi(char *response, size_t responseSize);
/* 本轮新增: immutable boot-only read policy/source/interface evidence.
 * Does not inspect live ADC state or claim current runtime source. */
esp_err_t sensorarrayFrontendStartupFormatSpiSource(char *response, size_t responseSize);

/* 本轮新增: no boot worker can still access ADS after this reports true.
 * Driver DMA quarantine must be checked separately by every hardware caller. */
bool sensorarrayFrontendStartupAdsOwnerQuiescent(void);

/* Existing I2C discovery policy moved from main without changing its rates,
 * verification scope, or fallback behavior. Call only while FDC is quiescent. */
void sensorarrayFrontendStartupRecheckFdcClock(sensorarrayFdcDeviceState_t *fdc,
                                              const char *mapLabel);

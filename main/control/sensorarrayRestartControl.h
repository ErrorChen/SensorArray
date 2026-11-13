#pragma once

#include "sensorarrayConfigIdentity.h"

/* 本轮新增: independent explicit-reset escape lane. Configuration/RECOVER
 * congestion cannot reserve this slot. Local ACK admission still gates reset. */
void sensorarrayRestartControlInit(uint64_t sessionId, uint32_t bootId);
esp_err_t sensorarrayRestartControlPost(
    const sensorarrayConfigAuthority_t *authority, bool known, uint32_t *outId);
bool sensorarrayRestartControlAck(uint32_t id, esp_err_t publishError);
bool sensorarrayRestartControlTake(uint32_t *outId);
esp_err_t sensorarrayRestartControlPublish(uint32_t id,
    const sensorarrayConfigAuthority_t *authority, bool known);
esp_err_t sensorarrayRestartControlFormat(char *response, size_t responseSize);

#pragma once

#include "freertos/FreeRTOS.h"
#include "sensorarrayTransport.h"

extern sensorarrayTransportStats_t g_sensorarrayTransportStats;
extern portMUX_TYPE g_sensorarrayTransportStatsMux;
extern sensorarrayTransportLegacyCommandCallback_t g_sensorarrayTransportLegacyCallback;
extern void *g_sensorarrayTransportLegacyContext;

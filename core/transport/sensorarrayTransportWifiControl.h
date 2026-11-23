#pragma once
#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"

/* 本轮新增. Queries are side-effect-free. WIFIREQ? is latest; WIFIREQ?<id>
 * addresses an exact nonzero boot-scoped request, never a nearest match. */
bool sensorarrayTransportWifiCommandHandles(const char *text);
esp_err_t sensorarrayTransportWifiCommand(const char *text, char *response, size_t size);
void sensorarrayTransportWifiDrainTerminal(void);

#include "sensorarrayTransportInternal.h"

sensorarrayTransportStats_t g_sensorarrayTransportStats;
portMUX_TYPE g_sensorarrayTransportStatsMux = portMUX_INITIALIZER_UNLOCKED;
sensorarrayTransportLegacyCommandCallback_t g_sensorarrayTransportLegacyCallback;
void *g_sensorarrayTransportLegacyContext;

void sensorarrayTransportGetStats(sensorarrayTransportStats_t *outStats)
{
    if (!outStats) {
        return;
    }
    portENTER_CRITICAL(&g_sensorarrayTransportStatsMux);
    *outStats = g_sensorarrayTransportStats;
    portEXIT_CRITICAL(&g_sensorarrayTransportStatsMux);
}

void sensorarrayTransportNoteSerialData(bool sent)
{
    portENTER_CRITICAL(&g_sensorarrayTransportStatsMux);
    if (sent) {
        g_sensorarrayTransportStats.serialDataSent++;
    } else {
        g_sensorarrayTransportStats.serialDataDrop++;
    }
    portEXIT_CRITICAL(&g_sensorarrayTransportStatsMux);
}

void sensorarrayTransportNoteSerialLog(bool sent)
{
    portENTER_CRITICAL(&g_sensorarrayTransportStatsMux);
    if (sent) {
        g_sensorarrayTransportStats.serialLogSent++;
    } else {
        g_sensorarrayTransportStats.serialLogDrop++;
    }
    portEXIT_CRITICAL(&g_sensorarrayTransportStatsMux);
}

void sensorarrayTransportSetLegacyCommandCallback(
    sensorarrayTransportLegacyCommandCallback_t callback,
    void *context)
{
    g_sensorarrayTransportLegacyCallback = callback;
    g_sensorarrayTransportLegacyContext = context;
}

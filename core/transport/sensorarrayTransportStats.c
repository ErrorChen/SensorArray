#include "sensorarrayTransportInternal.h"

#include "sensorarrayBle.h"
#include "sensorarrayTransportPolicy.h"
#include "sensorarrayWifi.h"

sensorarrayTransportStats_t g_sensorarrayTransportStats;
portMUX_TYPE g_sensorarrayTransportStatsMux = portMUX_INITIALIZER_UNLOCKED;
sensorarrayTransportLegacyCommandCallback_t g_sensorarrayTransportLegacyCallback;
void *g_sensorarrayTransportLegacyContext;
sensorarrayTransportRuntimeQueryCallback_t g_sensorarrayTransportRuntimeQueryCallback;
void *g_sensorarrayTransportRuntimeQueryContext;

static portMUX_TYPE s_runtimeMux = portMUX_INITIALIZER_UNLOCKED;
static sensorarrayTransportTxMode_t s_txMode = SENSORARRAY_TRANSPORT_TX_REL;
static sensorarrayTransportStream_t s_stream = SENSORARRAY_TRANSPORT_STREAM_AUTO;
static sensorarrayTransportWifiMode_t s_wifiMode = SENSORARRAY_TRANSPORT_WIFI_OFF;

static bool sensorarrayTransportStreamHas(sensorarrayTransportStream_t stream,
                                          sensorarrayTransportStream_t sink)
{
    return ((uint32_t)stream & (uint32_t)sink) != 0u;
}

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

void sensorarrayTransportSetRuntimeQueryCallback(
    sensorarrayTransportRuntimeQueryCallback_t callback,
    void *context)
{
    g_sensorarrayTransportRuntimeQueryCallback = callback;
    g_sensorarrayTransportRuntimeQueryContext = context;
}

void sensorarrayTransportSetTxMode(sensorarrayTransportTxMode_t mode)
{
    if (mode > SENSORARRAY_TRANSPORT_TX_FULL) {
        return;
    }
    portENTER_CRITICAL(&s_runtimeMux);
    s_txMode = mode;
    portEXIT_CRITICAL(&s_runtimeMux);
}

sensorarrayTransportTxMode_t sensorarrayTransportGetTxMode(void)
{
    portENTER_CRITICAL(&s_runtimeMux);
    sensorarrayTransportTxMode_t mode = s_txMode;
    portEXIT_CRITICAL(&s_runtimeMux);
    return mode;
}

void sensorarrayTransportSetStream(sensorarrayTransportStream_t stream)
{
    if (stream == SENSORARRAY_TRANSPORT_STREAM_AUTO) {
        portENTER_CRITICAL(&s_runtimeMux);
        s_stream = stream;
        portEXIT_CRITICAL(&s_runtimeMux);
        return;
    }
    uint32_t masked = (uint32_t)stream & (uint32_t)SENSORARRAY_TRANSPORT_STREAM_ALL;
    if (masked == 0u) {
        return;
    }
    portENTER_CRITICAL(&s_runtimeMux);
    s_stream = (sensorarrayTransportStream_t)masked;
    portEXIT_CRITICAL(&s_runtimeMux);
}

sensorarrayTransportStream_t sensorarrayTransportGetStream(void)
{
    portENTER_CRITICAL(&s_runtimeMux);
    sensorarrayTransportStream_t stream = s_stream;
    portEXIT_CRITICAL(&s_runtimeMux);
    return stream;
}

void sensorarrayTransportSetWifiMode(sensorarrayTransportWifiMode_t mode)
{
    if (mode > SENSORARRAY_TRANSPORT_WIFI_APSTA) {
        return;
    }
    portENTER_CRITICAL(&s_runtimeMux);
    s_wifiMode = mode;
    portEXIT_CRITICAL(&s_runtimeMux);
}

sensorarrayTransportWifiMode_t sensorarrayTransportGetWifiMode(void)
{
    portENTER_CRITICAL(&s_runtimeMux);
    sensorarrayTransportWifiMode_t mode = s_wifiMode;
    portEXIT_CRITICAL(&s_runtimeMux);
    return mode;
}

bool sensorarrayTransportSerialSinkEnabled(void)
{
    if (sensorarrayTransportGetStream() == SENSORARRAY_TRANSPORT_STREAM_AUTO) {
        return true;
    }
    return sensorarrayTransportStreamHas(sensorarrayTransportGetStream(),
                                        SENSORARRAY_TRANSPORT_STREAM_SER);
}

bool sensorarrayTransportBleSinkEnabled(void)
{
    return sensorarrayTransportBleChannelEnabled(SENSORARRAY_TRANSPORT_CHANNEL_DATA) ||
           sensorarrayTransportBleChannelEnabled(SENSORARRAY_TRANSPORT_CHANNEL_LOG);
}

bool sensorarrayTransportBleChannelEnabled(sensorarrayTransportChannel_t channel)
{
    sensorarrayTransportStream_t stream = sensorarrayTransportGetStream();
    bool streamWantsBle = stream == SENSORARRAY_TRANSPORT_STREAM_AUTO ||
                          sensorarrayTransportStreamHas(
                              stream,
                              SENSORARRAY_TRANSPORT_STREAM_BLE);
    return sensorarrayTransportBlePolicyAllows(
        streamWantsBle,
        sensorarrayBleIsConnected(),
        sensorarrayBleCanSend(SENSORARRAY_BLE_CH_DATA),
        sensorarrayBleCanSend(SENSORARRAY_BLE_CH_LOG),
        channel);
}

bool sensorarrayTransportWifiSinkEnabled(void)
{
    if (sensorarrayTransportGetStream() == SENSORARRAY_TRANSPORT_STREAM_AUTO) {
        return sensorarrayTransportGetWifiMode() != SENSORARRAY_TRANSPORT_WIFI_OFF &&
               sensorarrayWifiIsReady();
    }
    return sensorarrayTransportStreamHas(sensorarrayTransportGetStream(),
                                        SENSORARRAY_TRANSPORT_STREAM_WIFI) &&
           sensorarrayTransportGetWifiMode() != SENSORARRAY_TRANSPORT_WIFI_OFF;
}

const char *sensorarrayTransportTxModeName(sensorarrayTransportTxMode_t mode)
{
    switch (mode) {
    case SENSORARRAY_TRANSPORT_TX_SHORT:
        return "short";
    case SENSORARRAY_TRANSPORT_TX_FULL:
        return "full";
    case SENSORARRAY_TRANSPORT_TX_REL:
    default:
        return "rel";
    }
}

const char *sensorarrayTransportStreamName(sensorarrayTransportStream_t stream)
{
    uint32_t masked = (uint32_t)stream & (uint32_t)SENSORARRAY_TRANSPORT_STREAM_ALL;
    if (stream == SENSORARRAY_TRANSPORT_STREAM_AUTO) {
        return "auto";
    }
    if (masked == (uint32_t)SENSORARRAY_TRANSPORT_STREAM_SER) {
        return "ser";
    }
    if (masked == (uint32_t)SENSORARRAY_TRANSPORT_STREAM_BLE) {
        return "ble";
    }
    if (masked == (uint32_t)SENSORARRAY_TRANSPORT_STREAM_WIFI) {
        return "wifi";
    }
    if (masked == (uint32_t)SENSORARRAY_TRANSPORT_STREAM_ALL) {
        return "all";
    }
    if (masked == ((uint32_t)SENSORARRAY_TRANSPORT_STREAM_SER |
                   (uint32_t)SENSORARRAY_TRANSPORT_STREAM_BLE)) {
        return "ser,ble";
    }
    if (masked == ((uint32_t)SENSORARRAY_TRANSPORT_STREAM_SER |
                   (uint32_t)SENSORARRAY_TRANSPORT_STREAM_WIFI)) {
        return "ser,wifi";
    }
    if (masked == ((uint32_t)SENSORARRAY_TRANSPORT_STREAM_BLE |
                   (uint32_t)SENSORARRAY_TRANSPORT_STREAM_WIFI)) {
        return "ble,wifi";
    }
    return "ser";
}

const char *sensorarrayTransportWifiModeName(sensorarrayTransportWifiMode_t mode)
{
    switch (mode) {
    case SENSORARRAY_TRANSPORT_WIFI_STA:
        return "STA";
    case SENSORARRAY_TRANSPORT_WIFI_AP:
        return "AP";
    case SENSORARRAY_TRANSPORT_WIFI_APSTA:
        return "APSTA";
    case SENSORARRAY_TRANSPORT_WIFI_OFF:
    default:
        return "OFF";
    }
}

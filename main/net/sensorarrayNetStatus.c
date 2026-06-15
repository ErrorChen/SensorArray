#include "sensorarrayNetStatus.h"

#include <string.h>

#include "sensorarrayCommandMailbox.h"
#include "sensorarrayTransport.h"

static esp_err_t sensorarrayNetLegacyCommand(const uint8_t *data,
                                              size_t length,
                                              void *context)
{
    (void)context;
    return sensorarrayCommandMailboxPostText(data, length);
}

esp_err_t sensorarrayNetStatusInit(void)
{
    sensorarrayTransportSetLegacyCommandCallback(sensorarrayNetLegacyCommand, NULL);
    return sensorarrayTransportInit();
}

esp_err_t sensorarrayNetTextPublish(const sensorarrayTextPacket_t *packet,
                                    bool allowBle)
{
    (void)allowBle;
    if (!packet || packet->length == 0u) {
        return ESP_ERR_INVALID_ARG;
    }
    return sensorarrayTransportPublishData(packet->data, packet->length);
}

esp_err_t sensorarrayNetLogPublish(const sensorarrayTextPacket_t *packet)
{
    if (!packet || packet->length == 0u) {
        return ESP_ERR_INVALID_ARG;
    }
    return sensorarrayTransportPublishLog(packet->data, packet->length);
}

void sensorarrayNetGetSinkStats(sensorarrayNetSinkStats_t *outStats)
{
    if (!outStats) {
        return;
    }
    sensorarrayTransportStats_t stats = {0};
    sensorarrayTransportGetStats(&stats);
    *outStats = (sensorarrayNetSinkStats_t){
        .wifiSentPackets = stats.wifiDataSent + stats.wifiLogSent,
        .wifiDroppedPackets = stats.wifiDataDrop + stats.wifiLogDrop,
        .wifiBlockedCount = stats.blockCount,
        .bleSentPackets = stats.bleDataSent + stats.bleLogSent,
        .bleDroppedPackets = stats.bleDataDrop + stats.bleLogDrop,
        .bleBlockedCount = stats.bleCongested,
        .queueDepth = 0u,
        .queueDepthMax = stats.queueDrop,
    };
}

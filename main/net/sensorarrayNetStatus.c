#include "sensorarrayNetStatus.h"

#include <string.h>

#include "sensorarrayCommandMailbox.h"
#include "sensorarrayTransport.h"
#include "sensorarrayFramePublication.h"

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

esp_err_t sensorarrayNetFramePublish(const sensorarrayTextPacket_t *packet,
    const sensorarrayFrame_t *frame, uint32_t protectionFlags)
{
    if (!packet || !frame || packet->length < 2u || packet->length > sizeof(packet->data) ||
        packet->data[1] != ',' || packet->sequence != frame->sequence) {
        return ESP_ERR_INVALID_ARG;
    }
    sensorarrayTransportFrameMetadata_t metadata;
    if (!sensorarrayFramePublicationMetadata(frame, packet->data[0], &metadata)) {
        return ESP_ERR_INVALID_ARG;
    }
    return sensorarrayTransportPublishFrameMetadata(packet->data, packet->length, &metadata, protectionFlags);
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

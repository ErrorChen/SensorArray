#include "sensorarrayFrameOutput.h"

#include <stdio.h>

#include "sensorarrayTextProtocol.h"
#include "sensorarrayTransport.h"

esp_err_t sensorarrayFrameOutputPrint(const sensorarrayFrame_t *frame)
{
    if (!frame) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!sensorarrayTransportSerialSinkEnabled()) {
        return ESP_OK;
    }

    sensorarrayTextPacket_t packet;
    esp_err_t err = sensorarrayTextProtocolBuildFrame(frame, &packet);
    if (err != ESP_OK) {
        return err;
    }

    size_t written = fwrite(packet.data, 1u, packet.length, stdout);
    fflush(stdout);
    return written == packet.length ? ESP_OK : ESP_FAIL;
}

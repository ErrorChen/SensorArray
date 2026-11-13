#include "sensorarrayFrameOutput.h"

#include "sensorarraySerialIo.h"
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

    /* A scientific frame is one encoded packet, including its CRC trailer.
     * stdout is a line assembler for diagnostics and acknowledges consumption,
     * so it cannot supply this caller's queue-admission result. */
    return sensorarraySerialIoQueueData(packet.data, packet.length);
}

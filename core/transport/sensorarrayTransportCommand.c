#include "sensorarrayTransportInternal.h"

#include <stdio.h>
#include <string.h>

#include "sensorarrayScanConfig.h"

esp_err_t sensorarrayTransportHandleControlCommand(
    const uint8_t *data,
    size_t length,
    const sensorarrayTransportReplyTarget_t *replyTarget)
{
    if (!data || length == 0u || !replyTarget) {
        return ESP_ERR_INVALID_ARG;
    }
    char response[128];
    esp_err_t err = sensorarrayScanConfigHandleCommand((const char *)data, length,
                                                        response, sizeof(response));
    if (err == ESP_ERR_NOT_SUPPORTED && g_sensorarrayTransportLegacyCallback) {
        err = g_sensorarrayTransportLegacyCallback(data, length,
                                                   g_sensorarrayTransportLegacyContext);
        snprintf(response, sizeof(response),
                 err == ESP_OK ? "ACK,cmd=LEGACY,state=queued\n" :
                                 "ERR,cmd=UNKNOWN,reason=unsupported\n");
    } else if (err == ESP_ERR_NOT_SUPPORTED) {
        snprintf(response, sizeof(response), "ERR,cmd=UNKNOWN,reason=unsupported\n");
    }
    esp_err_t replyErr = sensorarrayTransportPublishControlReply(replyTarget,
                                                                 response,
                                                                 strlen(response));
    return replyErr == ESP_OK ? err : replyErr;
}

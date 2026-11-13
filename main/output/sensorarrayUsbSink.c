#include "sensorarrayUsbSink.h"

#include <stdio.h>
#include <string.h>

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "sensorarrayConfig.h"
#include "sensorarrayTransport.h"
#include "sensorarraySerialIo.h"
#include "sensorarrayUsbOtg.h"

#ifndef CONFIG_SENSORARRAY_USB_SINK_QUEUE_LEN
#define CONFIG_SENSORARRAY_USB_SINK_QUEUE_LEN 4
#endif
#ifndef CONFIG_SENSORARRAY_USB_SINK_TASK_STACK
#define CONFIG_SENSORARRAY_USB_SINK_TASK_STACK 4096
#endif
#ifndef CONFIG_SENSORARRAY_USB_SINK_TASK_PRIORITY
#define CONFIG_SENSORARRAY_USB_SINK_TASK_PRIORITY 6
#endif
#ifndef CONFIG_SENSORARRAY_USB_SINK_BLOCK_WARN_US
#define CONFIG_SENSORARRAY_USB_SINK_BLOCK_WARN_US 20000
#endif

static portMUX_TYPE s_usbStatsMux = portMUX_INITIALIZER_UNLOCKED;
static sensorarrayUsbSinkStats_t s_usbStats;
static portMUX_TYPE s_usbProfileMux = portMUX_INITIALIZER_UNLOCKED;
static sensorarrayUsbStreamProfile_t s_usbStreamProfile;
static bool s_usbInitialized;

esp_err_t sensorarrayUsbSinkInit(void)
{
    if (s_usbInitialized) {
        return ESP_OK;
    }

    memset(&s_usbStats, 0, sizeof(s_usbStats));
    portENTER_CRITICAL(&s_usbProfileMux);
    /* USBSTREAM is intentionally a volatile per-boot default and is never
     * persisted. DEBUG preserves the low-volume USB fallback contract while
     * FULL re-enables per-frame USB measurement data on demand. */
    s_usbStreamProfile = (sensorarrayUsbStreamProfile_t){
        .mode = SENSORARRAY_USB_STREAM_DEBUG,
        .dataEvery = SENSORARRAY_USB_STREAM_DEBUG_DATA_EVERY,
        .diagEvery = SENSORARRAY_USB_STREAM_DIAG_EVERY,
    };
    portEXIT_CRITICAL(&s_usbProfileMux);
    /* SerialIo is the single physical TX owner; no duplicate DATA queue/task. */
    s_usbInitialized = true;
    return ESP_OK;
}

static esp_err_t sensorarrayUsbSinkPublishClass(const sensorarrayTextPacket_t *packet, bool diagnostic)
{
    if (!packet || packet->length == 0u || packet->length > SENSORARRAY_TEXT_PACKET_MAX) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!sensorarrayTransportSerialSinkEnabled()) {
        return ESP_OK;
    }
    if (!s_usbInitialized) {
        return ESP_ERR_INVALID_STATE;
    }
    esp_err_t err = diagnostic ? sensorarraySerialIoQueue(packet->data, packet->length, false) :
        sensorarraySerialIoQueueData(packet->data, packet->length);
    portENTER_CRITICAL(&s_usbStatsMux);
    if (err == ESP_OK) {
        /* Legacy sent means sink submission; SerialIo reports driver bytes. */
        s_usbStats.sentPackets++;
        s_usbStats.sentBytes += packet->length;
    } else {
        s_usbStats.droppedPackets++;
    }
    portEXIT_CRITICAL(&s_usbStatsMux);
    return err;
}

esp_err_t sensorarrayUsbSinkPublish(const sensorarrayTextPacket_t *packet)
{
    return sensorarrayUsbSinkPublishClass(packet, false);
}

esp_err_t sensorarrayUsbSinkPublishDiagnostic(const sensorarrayTextPacket_t *packet)
{
    return sensorarrayUsbSinkPublishClass(packet, true);
}

void sensorarrayUsbSinkGetStats(sensorarrayUsbSinkStats_t *outStats)
{
    if (!outStats) {
        return;
    }
    portENTER_CRITICAL(&s_usbStatsMux);
    *outStats = s_usbStats;
    portEXIT_CRITICAL(&s_usbStatsMux);
    /* The old USB task was consolidated into SerialIo; no second stack. */
    outStats->taskConfiguredBytes = 0u;
    outStats->taskMinimumRemainingBytes = 0u;
}

esp_err_t sensorarrayUsbSinkSetStreamProfile(sensorarrayUsbStreamMode_t mode,
                                             uint32_t dataEvery,
                                             uint32_t diagEvery)
{
    if (mode != SENSORARRAY_USB_STREAM_DEBUG &&
        mode != SENSORARRAY_USB_STREAM_FULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (dataEvery == 0u) {
        dataEvery = mode == SENSORARRAY_USB_STREAM_FULL ?
            1u : SENSORARRAY_USB_STREAM_DEBUG_DATA_EVERY;
    }
    if (diagEvery == 0u) {
        diagEvery = SENSORARRAY_USB_STREAM_DIAG_EVERY;
    }
    if (dataEvery > SENSORARRAY_USB_STREAM_DATA_EVERY_MAX ||
        diagEvery > SENSORARRAY_USB_STREAM_DIAG_EVERY_MAX) {
        return ESP_ERR_INVALID_ARG;
    }

    /* FULL means every complete frame; DEBUG always decimates USB DATA
     * only. Core 1 capture pacing and BLE/Wi-Fi policies are unchanged. */
    if (mode == SENSORARRAY_USB_STREAM_FULL) {
        dataEvery = 1u;
    }

    portENTER_CRITICAL(&s_usbProfileMux);
    s_usbStreamProfile = (sensorarrayUsbStreamProfile_t){
        .mode = mode,
        .dataEvery = dataEvery,
        .diagEvery = diagEvery,
    };
    portEXIT_CRITICAL(&s_usbProfileMux);
    return ESP_OK;
}

sensorarrayUsbStreamProfile_t sensorarrayUsbSinkGetStreamProfile(void)
{
    sensorarrayUsbStreamProfile_t profile;
    portENTER_CRITICAL(&s_usbProfileMux);
    profile = s_usbStreamProfile;
    portEXIT_CRITICAL(&s_usbProfileMux);
    return profile;
}

bool sensorarrayUsbSinkShouldEmitData(uint32_t sequence)
{
    if (sensorarrayUsbOtgEnabled()) { return true; }
    sensorarrayUsbStreamProfile_t profile = sensorarrayUsbSinkGetStreamProfile();
    if (profile.mode == SENSORARRAY_USB_STREAM_FULL) {
        return true;
    }
    if (profile.dataEvery < 2u) {
        return true;
    }
    return (sequence % profile.dataEvery) == 0u;
}

const char *sensorarrayUsbStreamModeName(sensorarrayUsbStreamMode_t mode)
{
    switch (mode) {
    case SENSORARRAY_USB_STREAM_FULL:
        return "FULL";
    case SENSORARRAY_USB_STREAM_DEBUG:
    default:
        return "DEBUG";
    }
}

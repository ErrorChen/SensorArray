#include "sensorarrayUsbSink.h"

#include <stdio.h>
#include <string.h>

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "sensorarrayConfig.h"
#include "sensorarrayTransport.h"

#ifndef CONFIG_SENSORARRAY_USB_SINK_QUEUE_LEN
#define CONFIG_SENSORARRAY_USB_SINK_QUEUE_LEN 4
#endif
#ifndef CONFIG_SENSORARRAY_USB_SINK_TASK_STACK
#define CONFIG_SENSORARRAY_USB_SINK_TASK_STACK 6144
#endif
#ifndef CONFIG_SENSORARRAY_USB_SINK_TASK_PRIORITY
#define CONFIG_SENSORARRAY_USB_SINK_TASK_PRIORITY 6
#endif
#ifndef CONFIG_SENSORARRAY_USB_SINK_BLOCK_WARN_US
#define CONFIG_SENSORARRAY_USB_SINK_BLOCK_WARN_US 20000
#endif

enum {
    SENSORARRAY_USB_SINK_QUEUE_COUNT =
        CONFIG_SENSORARRAY_USB_SINK_QUEUE_LEN < 2 ? 2 : CONFIG_SENSORARRAY_USB_SINK_QUEUE_LEN,
};

static StaticQueue_t s_usbQueueStruct;
static uint8_t s_usbQueueStorage[SENSORARRAY_USB_SINK_QUEUE_COUNT * sizeof(sensorarrayTextPacket_t)];
static QueueHandle_t s_usbQueue;
static TaskHandle_t s_usbTask;
static portMUX_TYPE s_usbStatsMux = portMUX_INITIALIZER_UNLOCKED;
static sensorarrayUsbSinkStats_t s_usbStats;
/* Owned exclusively by sensorarrayUsbSinkTask. */
static sensorarrayTextPacket_t s_usbTaskPacket;
/* Publish is owned by the single async-log task. A fixed discard slot avoids
 * adding another 1536-byte packet to that caller's stack when the queue is
 * full and the oldest summary is intentionally replaced. */
static sensorarrayTextPacket_t s_usbDiscardPacket;

static void sensorarrayUsbSinkTask(void *arg)
{
    (void)arg;
    printf("TASKCORE,name=usb_sink,core=%d,expected=%d\n",
           (int)xPortGetCoreID(),
           CONFIG_SENSORARRAY_OUTPUT_TASK_CORE);

    while (true) {
        if (xQueueReceive(s_usbQueue, &s_usbTaskPacket, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        int64_t startUs = esp_timer_get_time();
        size_t written = fwrite(s_usbTaskPacket.data,
                                1u,
                                s_usbTaskPacket.length,
                                stdout);
        fflush(stdout);
        uint32_t writeUs = (uint32_t)(esp_timer_get_time() - startUs);

        portENTER_CRITICAL(&s_usbStatsMux);
        if (written == s_usbTaskPacket.length) {
            s_usbStats.sentPackets++;
            s_usbStats.sentBytes += written;
        } else {
            s_usbStats.droppedPackets++;
        }
        if (writeUs >= (uint32_t)CONFIG_SENSORARRAY_USB_SINK_BLOCK_WARN_US) {
            s_usbStats.blockedCount++;
        }
        s_usbStats.writeUsTotal += writeUs;
        if (writeUs > s_usbStats.writeUsMax) {
            s_usbStats.writeUsMax = writeUs;
        }
        s_usbStats.queueDepth = uxQueueMessagesWaiting(s_usbQueue);
        portEXIT_CRITICAL(&s_usbStatsMux);
    }
}

esp_err_t sensorarrayUsbSinkInit(void)
{
    if (s_usbTask) {
        return ESP_OK;
    }

    memset(&s_usbStats, 0, sizeof(s_usbStats));
    s_usbQueue = xQueueCreateStatic(SENSORARRAY_USB_SINK_QUEUE_COUNT,
                                    sizeof(sensorarrayTextPacket_t),
                                    s_usbQueueStorage,
                                    &s_usbQueueStruct);
    if (!s_usbQueue) {
        return ESP_ERR_NO_MEM;
    }

    BaseType_t ok = xTaskCreatePinnedToCore(sensorarrayUsbSinkTask,
                                            "sensorarrayUsbSink",
                                            CONFIG_SENSORARRAY_USB_SINK_TASK_STACK,
                                            NULL,
                                            CONFIG_SENSORARRAY_USB_SINK_TASK_PRIORITY,
                                            &s_usbTask,
                                            CONFIG_SENSORARRAY_OUTPUT_TASK_CORE);
    if (ok != pdPASS || !s_usbTask) {
        s_usbQueue = NULL;
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

esp_err_t sensorarrayUsbSinkPublish(const sensorarrayTextPacket_t *packet)
{
    if (!packet || packet->length == 0u || packet->length > SENSORARRAY_TEXT_PACKET_MAX) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!sensorarrayTransportSerialSinkEnabled()) {
        return ESP_OK;
    }
    if (!s_usbQueue || !s_usbTask) {
        return ESP_ERR_INVALID_STATE;
    }
    if (xQueueSend(s_usbQueue, packet, 0) != pdTRUE) {
        (void)xQueueReceive(s_usbQueue, &s_usbDiscardPacket, 0);
        portENTER_CRITICAL(&s_usbStatsMux);
        s_usbStats.droppedPackets++;
        portEXIT_CRITICAL(&s_usbStatsMux);
        if (xQueueSend(s_usbQueue, packet, 0) != pdTRUE) {
            return ESP_ERR_TIMEOUT;
        }
    }

    uint32_t depth = uxQueueMessagesWaiting(s_usbQueue);
    portENTER_CRITICAL(&s_usbStatsMux);
    s_usbStats.queueDepth = depth;
    if (depth > s_usbStats.queueDepthMax) {
        s_usbStats.queueDepthMax = depth;
    }
    portEXIT_CRITICAL(&s_usbStatsMux);
    return ESP_OK;
}

void sensorarrayUsbSinkGetStats(sensorarrayUsbSinkStats_t *outStats)
{
    if (!outStats) {
        return;
    }
    portENTER_CRITICAL(&s_usbStatsMux);
    *outStats = s_usbStats;
    portEXIT_CRITICAL(&s_usbStatsMux);
    outStats->taskConfiguredBytes = CONFIG_SENSORARRAY_USB_SINK_TASK_STACK;
    outStats->taskMinimumRemainingBytes = s_usbTask ?
        (uint32_t)uxTaskGetStackHighWaterMark(s_usbTask) *
            (uint32_t)sizeof(StackType_t) : 0u;
}

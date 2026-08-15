#include "sensorarrayTransportGuaranteedText.h"

#include <string.h>

#include "freertos/FreeRTOS.h"

typedef struct {
    bool reserved;
    bool ready;
    bool released;
    uint16_t length;
    char text[SENSORARRAY_TRANSPORT_GUARANTEED_TEXT_MAX];
} sensorarrayTransportGuaranteedTextSlot_t;

static sensorarrayTransportGuaranteedTextSlot_t s_slot;
static portMUX_TYPE s_guaranteedTextMux = portMUX_INITIALIZER_UNLOCKED;
static sensorarrayTransportGuaranteedTextNotify_t s_notify;

static void sensorarrayTransportGuaranteedTextNotify(void)
{
    if (s_notify) {
        s_notify();
    }
}

esp_err_t sensorarrayTransportGuaranteedTextReserve(void)
{
    portENTER_CRITICAL(&s_guaranteedTextMux);
    if (s_slot.reserved) {
        portEXIT_CRITICAL(&s_guaranteedTextMux);
        return ESP_ERR_NO_MEM;
    }
    s_slot = (sensorarrayTransportGuaranteedTextSlot_t){
        .reserved = true,
    };
    portEXIT_CRITICAL(&s_guaranteedTextMux);
    return ESP_OK;
}

esp_err_t sensorarrayTransportGuaranteedTextCancel(void)
{
    portENTER_CRITICAL(&s_guaranteedTextMux);
    if (!s_slot.reserved || s_slot.ready) {
        portEXIT_CRITICAL(&s_guaranteedTextMux);
        return ESP_ERR_INVALID_STATE;
    }
    memset(&s_slot, 0, sizeof(s_slot));
    portEXIT_CRITICAL(&s_guaranteedTextMux);
    return ESP_OK;
}

esp_err_t sensorarrayTransportGuaranteedTextRelease(void)
{
    portENTER_CRITICAL(&s_guaranteedTextMux);
    if (!s_slot.reserved || s_slot.released) {
        portEXIT_CRITICAL(&s_guaranteedTextMux);
        return ESP_ERR_INVALID_STATE;
    }
    s_slot.released = true;
    portEXIT_CRITICAL(&s_guaranteedTextMux);
    sensorarrayTransportGuaranteedTextNotify();
    return ESP_OK;
}

esp_err_t sensorarrayTransportGuaranteedTextAbort(void)
{
    portENTER_CRITICAL(&s_guaranteedTextMux);
    if (!s_slot.reserved) {
        portEXIT_CRITICAL(&s_guaranteedTextMux);
        return ESP_ERR_INVALID_STATE;
    }
    memset(&s_slot, 0, sizeof(s_slot));
    portEXIT_CRITICAL(&s_guaranteedTextMux);
    return ESP_OK;
}

esp_err_t sensorarrayTransportGuaranteedTextPublish(const char *data,
                                                    size_t length)
{
    if (!data || length == 0u ||
        length >= SENSORARRAY_TRANSPORT_GUARANTEED_TEXT_MAX) {
        return ESP_ERR_INVALID_ARG;
    }
    portENTER_CRITICAL(&s_guaranteedTextMux);
    if (!s_slot.reserved || s_slot.ready) {
        portEXIT_CRITICAL(&s_guaranteedTextMux);
        return ESP_ERR_INVALID_STATE;
    }
    memcpy(s_slot.text, data, length);
    s_slot.text[length] = '\0';
    s_slot.length = (uint16_t)length;
    s_slot.ready = true;
    portEXIT_CRITICAL(&s_guaranteedTextMux);
    sensorarrayTransportGuaranteedTextNotify();
    return ESP_OK;
}

bool sensorarrayTransportGuaranteedTextTakeDrain(
    char *outText,
    size_t outSize,
    size_t *outLength)
{
    if (!outText || outSize == 0u || !outLength) {
        return false;
    }
    bool drained = false;
    portENTER_CRITICAL(&s_guaranteedTextMux);
    if (s_slot.ready && s_slot.released && s_slot.length < outSize) {
        memcpy(outText, s_slot.text, (size_t)s_slot.length + 1u);
        *outLength = s_slot.length;
        memset(&s_slot, 0, sizeof(s_slot));
        drained = true;
    }
    portEXIT_CRITICAL(&s_guaranteedTextMux);
    return drained;
}

bool sensorarrayTransportGuaranteedTextIsReserved(void)
{
    portENTER_CRITICAL(&s_guaranteedTextMux);
    bool reserved = s_slot.reserved;
    portEXIT_CRITICAL(&s_guaranteedTextMux);
    return reserved;
}

void sensorarrayTransportGuaranteedTextSetNotify(
    sensorarrayTransportGuaranteedTextNotify_t notify)
{
    s_notify = notify;
}

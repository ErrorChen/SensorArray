#include "sensorarrayTransport.h"
#include "sensorarraySerialIo.h"
#include "sensorarrayUsbOtg.h"
#include <stdarg.h>
#include <stdio.h>

/* 本轮新增: runtime MFAULT is a control-plane event. CDC debug capacity or an
 * unread CDC endpoint must not be its only route to the formal USB client. */
esp_err_t sensorarrayTransportPublishRuntimeFault(const char *format, ...)
{
    if (!format) { return ESP_ERR_INVALID_ARG; }
    char text[SENSORARRAY_TRANSPORT_CTRL_TEXT_MAX];
    va_list args;
    va_start(args, format);
    int n = vsnprintf(text, sizeof(text), format, args);
    va_end(args);
    if (n <= 0 || (size_t)n >= sizeof(text)) { return ESP_ERR_INVALID_SIZE; }
    esp_err_t usb = sensorarraySerialIoQueue(text, (size_t)n, true);
    bool networkQueued = false;
    esp_err_t network = sensorarrayTransportPublishLifecycleObserved(text, (size_t)n, &networkQueued);
    /* Debug mirror is best effort and has independent capacity. */
    if (sensorarrayUsbOtgEnabled()) { (void)sensorarraySerialIoQueue(text, (size_t)n, false); }
    return usb != ESP_OK ? usb : network;
}

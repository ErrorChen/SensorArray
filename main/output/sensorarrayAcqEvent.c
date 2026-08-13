#include "sensorarrayAcqEvent.h"

#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"

#include "sensorarrayAsyncLog.h"
#include "sensorarrayConfig.h"

#define SENSORARRAY_ACQ_EVENT_TEXT_MAX 384u

static bool sensorarrayAcqEventIsProtocolLifecycle(const char *text)
{
    static const char *const prefixes[] = {
        "MAPP,", "MERR,", "MFAULT,", "RAPP,", "RERR,", "BAPP,",
        "ADSCHK,", "ADSCHKSTAT,", "BATPERIOD,", "RESSETTLE,",
    };
    if (!text) {
        return false;
    }
    for (size_t index = 0u; index < sizeof(prefixes) / sizeof(prefixes[0]); ++index) {
        size_t prefixLength = strlen(prefixes[index]);
        if (strncmp(text, prefixes[index], prefixLength) == 0) {
            return true;
        }
    }
    return false;
}

int sensorarrayAcqEventPrintf(const char *format, ...)
{
    if (!format) {
        return -1;
    }

    char text[SENSORARRAY_ACQ_EVENT_TEXT_MAX];
    va_list args;
    va_start(args, format);
    int required = vsnprintf(text, sizeof(text), format, args);
    va_end(args);
    if (required < 0) {
        return required;
    }

    size_t length = (size_t)required;
    if (length >= sizeof(text)) {
        length = sizeof(text) - 1u;
        if (length >= 2u) {
            text[length - 2u] = '\n';
            text[length - 1u] = '\0';
            length--;
        }
    }

    /* Before the EventRing exists, or when called by Core 0 itself, preserve
     * ordinary startup/output behaviour. Once acquisition is running on Core 1,
     * publishing is non-blocking and queue pressure becomes a drop counter. */
    if (xPortGetCoreID() != CONFIG_SENSORARRAY_ADC_CORE ||
        !sensorarrayAsyncLogIsRunning()) {
        return (int)fwrite(text, 1u, length, stdout);
    }
    if (sensorarrayAcqEventIsProtocolLifecycle(text)) {
        (void)sensorarrayAsyncLogPublishProtocolEvent(text, length);
    } else {
        (void)sensorarrayAsyncLogPublishTextEvent(text, length);
    }
    return required;
}

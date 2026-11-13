#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* 本轮新增. These lane numbers match SerialIo; each has its own endpoint. */
enum { SENSORARRAY_USB_DATA = 0, SENSORARRAY_USB_CTRL = 1, SENSORARRAY_USB_DEBUG = 2 };
typedef struct {
    uint32_t epoch, mounts, unmounts, resets, rxDropped, rxOversize;
    uint64_t accepted[3], callbackBytes[3], rxBytes;
    int64_t lastRxUs, lastTxUs;
    bool initialized, mounted, suspended, dmaEnabled, quiesced;
    esp_err_t initError;
} sensorarrayUsbOtgStats_t;
/* Owner-only APIs: never call TinyUSB from producers or command handlers. */
esp_err_t sensorarrayUsbOtgOwnerInit(void);
bool sensorarrayUsbOtgEnabled(void);
void sensorarrayUsbOtgOwnerPoll(void);
int sensorarrayUsbOtgOwnerWrite(unsigned lane, const void *data, size_t length);
bool sensorarrayUsbOtgOwnerCanWrite(unsigned lane);
uint32_t sensorarrayUsbOtgEpoch(void);
/* Single command consumer; complete commands carry their USB connection epoch. */
int sensorarrayUsbOtgRead(void *buffer, size_t size, uint32_t *epoch);
void sensorarrayUsbOtgGetStats(sensorarrayUsbOtgStats_t *stats);
bool sensorarrayUsbOtgRomDownloadAvailable(void);
/* Final restart fence, not an uninstall/restart-in-place operation. On timeout
 * all buffers and the owner stay alive. Caller must not report DMA cancelled. */
esp_err_t sensorarrayUsbOtgPrepareRestart(int64_t deadlineUs, bool romDownload);
void sensorarrayUsbOtgRestart(bool romDownload) __attribute__((noreturn));
void sensorarrayUsbOtgInitSerialString(void);

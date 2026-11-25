#pragma once
/* Legacy USB Serial/JTAG host-test backend. These tests intentionally exercise
 * the retained USJ production task, not the separately tested OTG owner. */
#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef SERIAL_IO_TEST_OTG_INIT
/* Full SerialIo initialization test runs exactly the owner init prefix, then
 * yields at its first Poll. This is synthetic dispatch, not a TinyUSB test. */
bool sensorarrayUsbOtgEnabled(void);
bool sensorarrayUsbOtgCdcEnabled(void);
esp_err_t sensorarrayUsbOtgOwnerInit(void);
void sensorarrayUsbOtgOwnerPoll(void);
bool sensorarrayUsbOtgOwnerCanWrite(unsigned lane);
int sensorarrayUsbOtgOwnerWrite(unsigned lane, const void *data, size_t length);
int sensorarrayUsbOtgRead(void *buffer, size_t length, uint32_t *epoch);
#else
static inline bool sensorarrayUsbOtgEnabled(void) { return false; }
static inline bool sensorarrayUsbOtgCdcEnabled(void) { return false; }
static inline esp_err_t sensorarrayUsbOtgOwnerInit(void)
{ assert(false && "legacy regression entered OTG init"); return ESP_ERR_NOT_SUPPORTED; }
static inline void sensorarrayUsbOtgOwnerPoll(void)
{ assert(false && "legacy regression entered OTG poll"); }
static inline bool sensorarrayUsbOtgOwnerCanWrite(unsigned lane)
{ (void)lane; assert(false && "legacy regression entered OTG write admission"); return false; }
static inline int sensorarrayUsbOtgOwnerWrite(unsigned lane, const void *data, size_t length)
{ (void)lane; (void)data; (void)length; assert(false && "legacy regression entered OTG write"); return -1; }
static inline int sensorarrayUsbOtgRead(void *buffer, size_t length, uint32_t *epoch)
{ (void)buffer; (void)length; (void)epoch; assert(false && "legacy regression entered OTG read"); return -1; }
#endif
static inline uint32_t sensorarrayUsbOtgEpoch(void) { return 0u; }

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#include "sensorarrayFrame.h"
#include "sensorarrayCommandMailbox.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t sensorarrayAsyncLogInit(void);
bool sensorarrayAsyncLogIsRunning(void);

esp_err_t sensorarrayAsyncLogPublishFrameSnapshot(const sensorarrayFrame_t *frame,
                                                  uint64_t measureFrameUs);
esp_err_t sensorarrayAsyncLogPublishOverrun(uint32_t sequence,
                                            int64_t elapsedUs,
                                            int64_t periodUs);
esp_err_t sensorarrayAsyncLogPublishFrameError(const sensorarrayFrame_t *frame,
                                               esp_err_t readErr,
                                               bool allRawZero,
                                               bool bootOk);
esp_err_t sensorarrayAsyncLogPublishCommandApplied(uint32_t sequence,
                                                   const sensorarrayCommand_t *command);
esp_err_t sensorarrayAsyncLogPublishTextEvent(const char *text, size_t length);

#ifdef __cplusplus
}
#endif

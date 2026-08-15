#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#define SENSORARRAY_TRANSPORT_GUARANTEED_TEXT_MAX 192u

typedef void (*sensorarrayTransportGuaranteedTextNotify_t)(void);

/* Single-slot, reservation-backed terminal pipeline.
 *
 * A producer reserves the slot before acknowledging a request.  Publishing
 * only fills the reserved storage, so it cannot be dropped by backpressure.
 * The release flag is the observable-ordering barrier: it is set only after
 * the matching acknowledgement has actually been published, therefore a
 * terminal can never overtake its ACK on the shared output path. */
esp_err_t sensorarrayTransportGuaranteedTextReserve(void);
esp_err_t sensorarrayTransportGuaranteedTextCancel(void);
esp_err_t sensorarrayTransportGuaranteedTextRelease(void);
/* Force-clear a reservation whose ACK was never published or whose terminal
 * publish failed.  Unlike Cancel it also recovers a slot that already holds
 * ready text, so a bounded cancel path can never leak the single lane. */
esp_err_t sensorarrayTransportGuaranteedTextAbort(void);
esp_err_t sensorarrayTransportGuaranteedTextPublish(const char *data,
                                                    size_t length);
bool sensorarrayTransportGuaranteedTextTakeDrain(
    char *outText,
    size_t outSize,
    size_t *outLength);
bool sensorarrayTransportGuaranteedTextIsReserved(void);
void sensorarrayTransportGuaranteedTextSetNotify(
    sensorarrayTransportGuaranteedTextNotify_t notify);

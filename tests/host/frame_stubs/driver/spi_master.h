#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct spi_device_t *spi_device_handle_t;

/* Complete placeholder: the frame structs embed one by value. Host tests do
 * not depend on the real ESP layout, only on frame semantics fields. */
typedef struct spi_transaction_t {
    uint8_t placeholder;
} spi_transaction_t;

#ifdef __cplusplus
}
#endif

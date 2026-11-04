#pragma once
#include <stddef.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
typedef void *spi_device_handle_t;
typedef struct {size_t length;size_t rxlength;const void *tx_buffer;void *rx_buffer;} spi_transaction_t;
esp_err_t spi_device_queue_trans(spi_device_handle_t device, spi_transaction_t *trans, TickType_t ticks);
esp_err_t spi_device_get_trans_result(spi_device_handle_t device, spi_transaction_t **trans, TickType_t ticks);

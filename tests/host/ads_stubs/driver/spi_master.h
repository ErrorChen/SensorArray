#pragma once
#include <stddef.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
typedef void *spi_device_handle_t;
#define SPI_CLK_SRC_DEFAULT 1
typedef struct {size_t length;size_t rxlength;const void *tx_buffer;void *rx_buffer;uint32_t override_freq_hz;} spi_transaction_t;
esp_err_t spi_device_queue_trans(spi_device_handle_t device, spi_transaction_t *trans, TickType_t ticks);
esp_err_t spi_device_get_trans_result(spi_device_handle_t device, spi_transaction_t **trans, TickType_t ticks);
esp_err_t spi_device_get_actual_freq(spi_device_handle_t device, int *freq_khz);
int spi_get_actual_clock(int source_hz, int desired_hz, int duty);

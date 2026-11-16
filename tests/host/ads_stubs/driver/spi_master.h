#pragma once
#include <stddef.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
typedef void *spi_device_handle_t;
#define SPI_CLK_SRC_DEFAULT 1
#define SPI3_HOST 2
#define SPI_DMA_CH_AUTO 3
typedef struct { int mosi_io_num,miso_io_num,sclk_io_num,quadwp_io_num,quadhd_io_num,max_transfer_sz; } spi_bus_config_t;
typedef struct { int clock_speed_hz; unsigned mode,duty_cycle_pos; int spics_io_num,queue_size; } spi_device_interface_config_t;
esp_err_t spi_bus_initialize(int host, const spi_bus_config_t *config, int dma);
esp_err_t spi_bus_add_device(int host, const spi_device_interface_config_t *config, spi_device_handle_t *out);
typedef struct {size_t length;size_t rxlength;const void *tx_buffer;void *rx_buffer;uint32_t override_freq_hz;} spi_transaction_t;
esp_err_t spi_device_queue_trans(spi_device_handle_t device, spi_transaction_t *trans, TickType_t ticks);
esp_err_t spi_device_get_trans_result(spi_device_handle_t device, spi_transaction_t **trans, TickType_t ticks);
esp_err_t spi_device_get_actual_freq(spi_device_handle_t device, int *freq_khz);
int spi_get_actual_clock(int source_hz, int desired_hz, int duty);

#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t nvs_flash_init_partition(const char *partitionLabel);
esp_err_t nvs_flash_erase_partition(const char *partName);

#ifdef __cplusplus
}
#endif

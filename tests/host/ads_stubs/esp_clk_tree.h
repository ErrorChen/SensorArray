#pragma once
#include <stdint.h>
#include "esp_err.h"
#define ESP_CLK_TREE_SRC_FREQ_PRECISION_CACHED 0
esp_err_t esp_clk_tree_src_get_freq_hz(int source, int precision, uint32_t *hz);

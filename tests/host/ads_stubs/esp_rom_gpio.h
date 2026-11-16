#pragma once
#include <stdint.h>
#include <stdbool.h>
void esp_rom_gpio_connect_out_signal(uint32_t gpio, uint32_t signal, bool invert, bool oe_invert);

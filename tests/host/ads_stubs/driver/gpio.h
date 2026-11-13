#pragma once
#include <stdint.h>
#include "esp_err.h"
typedef int gpio_num_t;
#define GPIO_NUM_NC -1
#define GPIO_MODE_OUTPUT 1
#define GPIO_MODE_INPUT 2
#define GPIO_PULLUP_ENABLE 1
#define GPIO_PULLUP_DISABLE 0
#define GPIO_PULLDOWN_DISABLE 0
#define GPIO_INTR_DISABLE 0
#define GPIO_INTR_NEGEDGE 3
typedef struct {uint64_t pin_bit_mask;int mode;int pull_up_en;int pull_down_en;int intr_type;} gpio_config_t;
esp_err_t gpio_set_direction(gpio_num_t pin, int mode);
esp_err_t gpio_set_level(gpio_num_t pin, int level);
int gpio_get_level(gpio_num_t pin);
esp_err_t gpio_config(const gpio_config_t *cfg);
esp_err_t gpio_isr_handler_add(gpio_num_t pin, void (*fn)(void *), void *arg);
esp_err_t gpio_isr_handler_remove(gpio_num_t pin);

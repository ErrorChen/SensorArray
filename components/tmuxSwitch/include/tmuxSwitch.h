#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    TMUX1108_SOURCE_GND = 0,
    TMUX1108_SOURCE_REF = 1
} tmux1108Source_t;

esp_err_t tmuxSwitchInit(void);

esp_err_t tmux1108SelectRow(uint8_t row);
esp_err_t tmux1108GetRow(uint8_t *rowOut);

esp_err_t tmux1108SetSource(tmux1108Source_t source);
esp_err_t tmux1108GetSource(tmux1108Source_t *sourceOut);

esp_err_t tmux1134SetSelAEnabled(bool enabled);
esp_err_t tmux1134SetSelBEnabled(bool enabled);
esp_err_t tmux1134SetAllOff(void);
esp_err_t tmux1134SetAllOn(void);

esp_err_t tmuxSwitchSelectRow(uint8_t row);
esp_err_t tmuxSwitchSetSelAEnabled(bool enabled);
esp_err_t tmuxSwitchSetSelBEnabled(bool enabled);
esp_err_t tmuxSwitchSet1108Source(tmux1108Source_t source);

#ifdef __cplusplus
}
#endif

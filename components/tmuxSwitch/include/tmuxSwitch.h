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

typedef struct {
    bool inited;
    uint8_t row;
    tmux1108Source_t source;
    bool tmux1134EnControllable;
    bool tmux1134EnLogicalOn;
    int a0Level;
    int a1Level;
    int a2Level;
    int swLevel;
    int sel1Level;
    int sel2Level;
    int sel3Level;
    int sel4Level;
    int enLevel;
} tmuxSwitchControlState_t;

esp_err_t tmuxSwitchInit(void);

esp_err_t tmux1108SelectRow(uint8_t row);
esp_err_t tmux1108GetRow(uint8_t *rowOut);

esp_err_t tmux1108SetSource(tmux1108Source_t source);
esp_err_t tmux1108GetSource(tmux1108Source_t *sourceOut);

/*
 * Explicit SEL logic-level control.
 * level=0/1 is written directly to SELx pin without enabled/disabled abstraction.
 */
esp_err_t tmux1134SelectSelALevel(bool level);
esp_err_t tmux1134SelectSelBLevel(bool level);
esp_err_t tmux1134SetEnLogicalState(bool on);
esp_err_t tmux1134GetEnLogicalState(bool *onOut);

/*
 * Backward-compatibility wrappers using CONFIG_TMUX1134_SELx_ENABLED_LEVEL.
 * enabled=true drives SELx to CONFIG_TMUX1134_SELx_ENABLED_LEVEL.
 */
esp_err_t tmux1134SetSelAEnabled(bool enabled);
esp_err_t tmux1134SetSelBEnabled(bool enabled);
esp_err_t tmux1134SetAllOff(void);
esp_err_t tmux1134SetAllOn(void);
esp_err_t tmuxSwitchGetControlState(tmuxSwitchControlState_t *outState);

esp_err_t tmuxSwitchSelectRow(uint8_t row);
esp_err_t tmuxSwitchSetSelAEnabled(bool enabled);
esp_err_t tmuxSwitchSetSelBEnabled(bool enabled);
esp_err_t tmuxSwitchSet1108Source(tmux1108Source_t source);

#ifdef __cplusplus
}
#endif

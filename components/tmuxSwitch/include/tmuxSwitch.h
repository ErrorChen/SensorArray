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
    bool tmux1134EnControllable;

    /* Last software-commanded control state (metadata, not hardware proof). */
    uint8_t cmdRow;
    tmux1108Source_t cmdSource;
    bool cmdTmux1134EnLogicalOn;
    int cmdA0Level;
    int cmdA1Level;
    int cmdA2Level;
    int cmdSwLevel;
    int cmdSel1Level;
    int cmdSel2Level;
    int cmdSel3Level;
    int cmdSel4Level;
    int cmdEnLevel;
    int cmdSelaLevel; // Alias of cmdSel1Level for explicit SELA naming.
    int cmdSelbLevel; // Alias of cmdSel2Level for explicit SELB naming.

    /*
     * MCU-side GPIO observations. These are pin-level observations only and do
     * not prove analog conduction through the external switch matrix.
     */
    int obsA0Level;
    int obsA1Level;
    int obsA2Level;
    int obsSwLevel;
    int obsSel1Level;
    int obsSel2Level;
    int obsSel3Level;
    int obsSel4Level;
    int obsEnLevel;
    bool obsTmux1134EnLogicalOnValid;
    bool obsTmux1134EnLogicalOn;
    int obsSelaLevel; // Alias of obsSel1Level for explicit SELA naming.
    int obsSelbLevel; // Alias of obsSel2Level for explicit SELB naming.

    /*
     * Verified control levels: last value that was explicitly written and
     * immediately read back as matching on MCU GPIOs.
     * -1 means unknown / not yet verified.
     */
    int verifiedA0Level;
    int verifiedA1Level;
    int verifiedA2Level;
    int verifiedSwLevel;
    int verifiedSel1Level;
    int verifiedSel2Level;
    int verifiedSel3Level;
    int verifiedSel4Level;
    int verifiedEnLevel;
    int verifiedSelaLevel; // Alias of verifiedSel1Level.
    int verifiedSelbLevel; // Alias of verifiedSel2Level.

    // Monotonic control update sequence and update tick in ms.
    uint32_t sequence;
    uint32_t tickMs;
} tmuxSwitchControlState_t;

typedef struct {
    const char *stage;
    const char *routeLabel;
    const char *targetPath;
    const char *expectedSemantic;
} tmuxSwitchSnapshotContext_t;

esp_err_t tmuxSwitchInit(void);

esp_err_t tmux1108SelectRow(uint8_t row);
/* Returns the last software-commanded row index (0..7). */
esp_err_t tmux1108GetRow(uint8_t *rowOut);

esp_err_t tmux1108SetSource(tmux1108Source_t source);
/* Returns the last software-commanded SW source. */
esp_err_t tmux1108GetSource(tmux1108Source_t *sourceOut);

/*
 * Explicit SEL logic-level control.
 * level=0/1 is written directly to SELx pin without enabled/disabled abstraction.
 */
esp_err_t tmux1134SelectSelALevel(bool level);
esp_err_t tmux1134SelectSelBLevel(bool level);
esp_err_t tmux1134SelectSel3Level(bool level);
esp_err_t tmux1134SelectSel4Level(bool level);
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

/*
 * Captures both software-commanded state and MCU pin observations.
 * Observed GPIO levels are MCU-side only and are not definitive analog-route proof.
 */
esp_err_t tmuxSwitchGetControlState(tmuxSwitchControlState_t *outState);
esp_err_t tmuxSwitchSnapshotControlState(tmuxSwitchControlState_t *outState);
esp_err_t tmuxSwitchLogControlSnapshot(const tmuxSwitchSnapshotContext_t *context);

esp_err_t tmuxSwitchSelectRow(uint8_t row);
esp_err_t tmuxSwitchSetSelAEnabled(bool enabled);
esp_err_t tmuxSwitchSetSelBEnabled(bool enabled);
esp_err_t tmuxSwitchSet1108Source(tmux1108Source_t source);

#ifdef __cplusplus
}
#endif

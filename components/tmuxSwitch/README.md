# tmuxSwitch

ESP-IDF component for TI TMUX1108 (8:1 mux) and TMUX1134 (4x SPDT) using GPIO only.

## Board assumptions (current hardware)
- TMUX1108 EN is tied high (always enabled). The driver does not control EN and cannot fully disconnect.
- SW (GPIO7 by default) selects REF vs GND into TMUX1108; polarity is configurable.
- TMUX1108 row mapping: row 0..7 -> S1..S8.
- TMUX1134 SELA/SELB are abstracted as enable/disable with per-channel enabled level.
- TMUX1134 EN is optional; set EN GPIO to -1 if tied off.

## Truth table summary
TMUX1108 (A2 A1 A0 -> channel):
- 0 0 0 -> S1
- 0 0 1 -> S2
- 0 1 0 -> S3
- 0 1 1 -> S4
- 1 0 0 -> S5
- 1 0 1 -> S6
- 1 1 0 -> S7
- 1 1 1 -> S8

TMUX1134:
- SELx=0 -> SxB to Dx
- SELx=1 -> SxA to Dx

## Safe row switching and glitches
- Changing A[2:0] while EN is high can create transient connections.
- If CONFIG_TMUX1108_SWITCH_ROW_SAFE_MODE is enabled, the driver:
  1) switches SW to CONFIG_TMUX1108_SAFE_SOURCE,
  2) updates the row,
  3) delays CONFIG_TMUX1108_SWITCH_DELAY_US,
  4) restores the previous source.
- If safe mode is disabled, do the safe-source sequence in the caller.

## TMUX1134 enable/disable abstraction
- tmux1134SetSelAEnabled and tmux1134SetSelBEnabled use
  CONFIG_TMUX1134_SELA_ENABLED_LEVEL and CONFIG_TMUX1134_SELB_ENABLED_LEVEL.
- If EN is controllable and currently off, SEL setters automatically turn EN on.
- tmux1134SetAllOff drives EN to the off level if connected; if EN=-1 it only
  forces SELA/SELB to disable and returns ESP_OK (no true global disconnect).
- tmux1134SetAllOn turns EN on if connected and enables SELA/SELB.

## Notes
- Required GPIOs (A0/A1/A2/SW/SEL1/SEL2) must not be -1 or tmuxSwitchInit returns ESP_ERR_INVALID_ARG.
- SW polarity is set by CONFIG_TMUX1108_SW_REF_LEVEL (y=high selects REF, n=low selects REF).

## Minimal usage
```c
tmuxSwitchInit();
tmuxSwitchSet1108Source(TMUX1108_SOURCE_GND);
tmuxSwitchSelectRow(3);
tmuxSwitchSet1108Source(TMUX1108_SOURCE_REF);
tmuxSwitchSetSelAEnabled(true);
tmuxSwitchSetSelBEnabled(false);
```

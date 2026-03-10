# tmuxSwitch

ESP-IDF GPIO driver for TMUX1108 (8:1 mux) and TMUX1134 (4x SPDT).

## Board assumptions (current hardware)
- TMUX1108 EN is tied high on this board. Firmware can only drive `A0/A1/A2/SW`.
- TMUX1134 EN may be optional (`CONFIG_TMUX1134_EN_GPIO=-1` means no firmware control).
- When TMUX1134 EN is not controllable, there is no true global disconnect from firmware.

## TMUX1108 mapping
- `row 0..7 -> S1..S8`.
- `tmux1108SetSource(TMUX1108_SOURCE_GND/REF)` drives SW according to `CONFIG_TMUX1108_SW_REF_LEVEL`.

## TMUX1134 control model
- Preferred API for debug/bring-up is explicit logic-level selection:
  - `tmux1134SelectSelALevel(bool level)`
  - `tmux1134SelectSelBLevel(bool level)`
- Optional EN control:
  - `tmux1134SetEnLogicalState(bool on)`
  - `tmux1134GetEnLogicalState(bool *onOut)`
- Backward-compat wrappers are still available:
  - `tmux1134SetSelAEnabled(bool enabled)`
  - `tmux1134SetSelBEnabled(bool enabled)`
  - These wrappers only map `enabled` to a configured logic level (`CONFIG_TMUX1134_SELx_ENABLED_LEVEL`).
  - They do not guarantee electrical disconnect semantics.

## Electrical meaning reminder
- TMUX1134 path is selected by SEL logic level.
- If EN is tied active on hardware, changing SEL only switches path selection; it does not isolate the switch.
- `tmux1134SetAllOff()` only guarantees EN-off behavior when EN is actually GPIO-controlled.

## Raw control-state observability
- Use `tmuxSwitchGetControlState(...)` to read driven logic levels:
  - `A0/A1/A2/SW`
  - `SEL1/SEL2/SEL3/SEL4`
  - `EN` (if configured)
- This is intended for deterministic probe/debug logging.

## Safe row switching and glitches
- If `CONFIG_TMUX1108_SWITCH_ROW_SAFE_MODE=y`, row switching temporarily moves SW to
  `CONFIG_TMUX1108_SAFE_SOURCE`, updates row, delays `CONFIG_TMUX1108_SWITCH_DELAY_US`,
  then restores previous source.

## Minimal usage
```c
tmuxSwitchInit();
tmuxSwitchSelectRow(3);
tmux1134SelectSelALevel(true);
tmux1134SelectSelBLevel(false);
tmuxSwitchSet1108Source(TMUX1108_SOURCE_REF);
```

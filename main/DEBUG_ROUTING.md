# SensorArray Routing/ADS/FDC Debug Modes

This document describes the deterministic debug workflow implemented in `main/main.c`.

## Files changed
- `main/main.c`
- `main/Kconfig.projbuild`
- `components/tmuxSwitch/include/tmuxSwitch.h`
- `components/tmuxSwitch/tmuxSwitch.c`
- `components/tmuxSwitch/README.md`
- `components/ads126xAdc/README.md`

## Debug modes
Select mode in `menuconfig`:
- `SensorArray Project -> Debug routing and self-test -> Debug execution mode`

Available modes:
- `ROUTE_IDLE`: init hardware and idle forever.
- `ROUTE_FIXED_STATE`: apply one explicit route and hold for probing.
- `ROUTE_STEP_ONCE`: step through route table once and then idle.
- `ROUTE_SCAN_LOOP`: continuous bring-up scan loop.
- `ADS_SELFTEST`: ADS-only deterministic self-test sequence.
- `FDC_SELFTEST`: FDC-only deterministic self-test sequence.
- `S1D1_RESISTOR_DEBUG`: single-point resistor debug using the board-map route for `S1D1`.
- `S5D5_CAP_FDC_SECONDARY`: dedicated `S5D5 / SELB / secondary FDC2214(0x2A)` bring-up path.
- `FDC_I2C_DISCOVERY`: continuous 2-bus x 2-address I2C discovery (`IO9/10`, `IO11/12`, `0x2A/0x2B`).

## Manual probe mode (freeze one route)
Use `ROUTE_FIXED_STATE` and configure:
- `Fixed route S column`
- `Fixed route D line`
- `Fixed route path type`
- `Fixed route SW source`
- `Fixed route SELA logic level`
- `Fixed route SELB logic level`
- `Fixed route: skip ADS read`
- `Fixed route: skip FDC read`
- `Fixed route: hold final state forever` (recommended for scope/DMM probing)

Current board polarity is translated only in `main/sensorarrayBoardMap.c`:
- `SELA GPIO 0 -> ADS1263`
- `SELA GPIO 1 -> FDC2214`

The firmware applies GPIO controls in deterministic order:
1. TMUX1108 row (`A0/A1/A2`)
2. TMUX1134 SELA level
3. TMUX1134 SELB level
4. TMUX1108 SW source

Per-step delays are controlled by `Route step delay (ms)`.

## ADS self-test mode
Use `ADS_SELFTEST` and configure:
- Forced `muxp/muxn`
- Forced raw `REFMUX`
- Optional `STOP1 before INPMUX`
- Settle delay after INPMUX
- Optional `START1 before read`
- Discard count
- Read retry count
- Sample count

Self-test logs:
- `DBGADSREG,...` for key register dumps.
- `DBGADSSELF,...` for discard/sample sequencing details.

## FDC I2C discovery mode
Use `FDC_I2C_DISCOVERY` to isolate communication only (no ADS/matrix measurement dependency).

Logs:
- `FDC_SCAN_BOOT,...` bus pins + frequency + enable state on boot.
- `FDC_SCAN,...` one line per candidate with ACK/ID status:
  - `no_ack`
  - `ack_but_read_failed`
  - `id_ok`
  - `unexpected_id`
- `FDC_SCAN_SUMMARY,cycle=N,found=...` once per full cycle.

## Route map audit output
At boot, firmware prints:
- `DBGROUTEMAP,...` full route table (`S`, `D`, `path`, `selaRoute`, `selaWriteLevel`, `selBLevel`, label).
- `DBGFDCMAP,...` full D-line to FDC-device/channel mapping.

`S5D5` capacitive route is now expected to report:
- `path=cap`
- `selaRoute=FDC2214`
- `label=S5D5_cap_selb_fdc2214`

## Control GPIO observability
After route operations, firmware prints:
- `DBGCTRL,stage=...,a0=...,a1=...,a2=...,sw=...,sel1=...,sel2=...,sel3=...,sel4=...,en=...,drdy=...,adsReset=...`

`DBG,...` lines are preserved and extended with:
- `muxp`, `muxn`, `refmux`, `discardCount`
- `ctrlA0`, `ctrlA1`, `ctrlA2`, `ctrlSW`, `ctrlSel1`, `ctrlSel2`, `ctrlSel3`, `ctrlSel4`, `ctrlEn`

## TMUX1134 abstraction note
The project now treats TMUX1134 as explicit logic-level routing, not abstract "enable/disable":
- App/route flows should use `sensorarrayMeasureSetSelaPath()` so logical SELA path and raw GPIO level stay mapped in one place.
- `tmux1134SelectSelALevel()` / `tmux1134SelectSelBLevel()` remain low-level driver APIs only.
- Legacy enabled/disabled APIs remain as wrappers only.
- If TMUX1134 EN is not GPIO-controlled, "all off" is not a true disconnect.

# Reliability hardware revision notes

2026-11-13 update: the user confirmed and repaired the loose S8D8 connection; new RES recordings show finite resistance and no OPEN at either reference resistor. User-reported rails are AVDD 3.33–3.37 V and AVSS −1.795 V ±0.005 V relative to GND. The user subsequently confirmed the probes were at the ADS supply pins or nearby decoupling capacitors. Those readings include upstream supply-path drops such as R54 at measurement time; no additional subtraction is warranted. Traceable uncertainty and simultaneous rail waveforms remain unavailable. Representative values were applied only in firmware RAM and were not changed by this location confirmation. CRC rejection persists with external rails and periodic battery sampling disabled; this narrows the software conditions but still does not identify an electrical root cause. No HW01–HW06 rework or scope verification was performed. See the [new HIL report](../audit/reference_fixture_recheck_20261113.md); the earlier no-DMM/unconfirmed-contact observations below remain historical evidence.

Status: **HARDWARE_CHANGE_REQUIRED / as-built revision NOT_VERIFIED**. Audited repository schematic: `datasheets/circuit.pdf`, V1.0, sheets 1–4 (2025-11-28 to 2025-12-05). This is a design review, not an implemented PCB modification or a claim about the user's populated board. The initial audit had no hardware access. The subsequent [reference-fixture HIL](../audit/reference_fixture_hil_20261104.md) accessed the ESP32-S3/ADS1262 board over native USB and recorded its passive fixture. It did not provide an oscilloscope, independent DMM or Windows enumerator. The electrical procedures in HW01–HW06 below remain NOT_RUN.

## Connectivity and firmware limits

| Requested state | Q1/SW | ADS INTREF | U5 EN / common D | Selected S row | What firmware can prove |
|---|---|---|---|---|---|
| Boot before ADS identity | gate low / released | unknown (device reset enables reference) | pulled high by R25 / REF | remains connected | GPIO request only; no analogue safety claim |
| CAP | gate high / clamp | OFF, register readback required first | high / REF | ground via Q1 | register and MCU GPIO agreement; FDC scan remains active |
| VOLT | gate high / clamp | OFF, AVDD/AVSS reference | high / REF | ground via Q1 | register/GPIO agreement; calibrated rails required |
| RES | gate low / released | ON, internal reference | high / REF | REFOUT excitation | intentional excitation; reference startup deadline before conversion |
| SAFE requested | clamp only after confirmed INTREF OFF | OFF if transaction succeeded | high / REF | still connected | requested/applied/known/verified tracked separately; no independent analogue observation |
| SAFE_RAIL_MONITOR | would release clamp | would enable INTREF | high / REF | **would be excited** | capability unavailable on this board; rejected before hardware mutation |
| FRONTEND_WAIT / SUSPECT (2026-11-13) | request released | unknown; returning ADS defaults ON | high / REF | remains connected | pause acquisition, GPIO readback only, physicalSafe=unknown |

U6/U7 SEL switches D lines between FDC and ADS branches. They do not disconnect U5's selected S row from REF. TMUX1108 break-before-make applies during an address transition; it is not a persistent all-open state. REFOUT is approximately AVSS + 2.5 V, not 2.5 V relative to ground. AIN9 is wired to biased AINCOM, not ground. Internal supply monitor measures a span; the nominal-ratio AVDD/AVSS split is an estimate.

The new automatic frontend-return path releases Q1 on the first suspicious
all-unavailable observation and disables the controllable TMUX1134 enable, if
fitted. It does not switch frontend power or guarantee isolation during the
interval before loss is detected. A returning ADC can enable REFOUT before
software sees it. Low-rate ID probes and a controlled MCU restart cannot fix
HW03 back-power through I/O, clock inputs or analogue connections; responding
IDs establish communication, not a measured power-domain state. Keep the
HARDWARE_CHANGE_REQUIRED status. The present tests use the user's passive
resistor/capacitor fixture, without a low-impedance external voltage source.

Firmware therefore does not transparently excite a VOLT/CAP matrix to refresh rails. In a known, unpowered-input-safe setup measure AVDD and AVSS at ADS pins relative to board ground, apply `RAILCFG=<positive_uV>,<negative_uV>` before VOLT, and confirm the terminal/query. This calibration is volatile and must be repeated after reboot or an actual supply/temperature change; its timestamp is not refreshed by failures. It is not an instrument traceability record: preserve DMM model, serial, calibration date, uncertainty, board ID, temperature and measurement time outside the device. CAP remains supported; RES keeps intentional excitation; valid external rails enable VOLT and rail-dependent battery conversion. No firmware declaration makes the present circuit input-high-impedance or fail-safe during power loss.

## Revision register and proposed BOM

All component changes below are proposed, not populated. Final values/part selection require the as-built netlist, source impedance and accuracy budget. Do not substitute an unverified part merely to satisfy this table.

| ID | Evidence and risk | Proposed BOM / topology | Electrical and precision acceptance | Measurement procedure |
|---|---|---|---|---|
| HW01 | Sheet 2 R21=10 kΩ pulls EN_IC high; R24=1 kΩ + LED4 load EN to GND. Their DC load can lower EN below reset-release threshold. | Remove LED4/R24 from EN. Move indication to supply or a separate buffered GPIO (e.g. existing LED + 1 kΩ, optional 2N7002 with separate gate bias). Retain suitable EN pull-up and reset RC, verify actual C13/C18. | ESP32-S3 VIH_nRST ≥0.75×VDD; at 3.3 V this is 2.475 V. Compute LED branch operating point with its actual I–V curve. Added EN capacitance only changes timing, not the DC divider. Buffer leakage must not consume EN margin. | First inspect installed revision. Scope 3V3_IC and EN at power-up, USB insertion, radio burst, manual reset; DMM steady EN. Verify rise/reset pulse against chip timing limits. |
| HW02 | Field observation of about +0.3 V on the nominal negative rail is user evidence, not reproduced here. Sheet 1 U3 is LM27762. | Review EN+/EN− sequencing, CP/flying/output capacitors and feedback; add monitored sequencing or rated isolation if required. Do not simply add capacitance or a diode without headroom/leakage analysis. | All connected ADS/TMUX pins must remain within their absolute limits throughout startup/shutdown. Validate negative-rail monotonicity, overshoot, load discharge and backfeed; LM27762 shutdown has internal discharge resistance, not a guaranteed zero-voltage clamp. | Differential scope OUT−, AVSS at ADS, OUT+, EN_ARRAY, REF and externally powered inputs across battery/USB power permutations. Current-limit supplies; no dangerous low-impedance input injection. |
| HW03 | MCU/FDC use 3V3_IC while analogue frontend uses +3V3/−1V8. IO and analogue inputs can feed an unpowered domain. | Provide power-sequenced bus isolation or buffers with specified Ioff; isolate external inputs/reference with parts rated for both supply-present and supply-absent conditions. Add test points; select series resistors only after timing/accuracy analysis. | ADS analogue input absolute range AVSS−0.3…AVDD+0.3 V is not a usable operating range. Series resistance causes I_leak×R_source error and RC settling; isolation leakage and capacitance must be budgeted at temperature. | With bounded current and disconnected scientific sources, power each domain independently, measure rail lift and per-pin current; test all enable/reset sequences. |
| HW04 | Sheet 3 R54=4.7 Ω feeds ADS AVDD; nominal +3.3/−1.8 V gives 5.1 V before load drop, tolerance and ripple. | Use precision supply feedback/filter parts after error-budget review; retain or change R54 only after noise and transient testing. Add AVDD/AVSS test pads at IC. | ADS recommended span is 4.75…5.25 V. Budget regulator errors + load·4.7 Ω + ripple + temperature separately. Span measurement alone does not independently calibrate AVSS and AVDD. Out-of-range raw values remain diagnostic, never clamped into valid values. | DMM both rails at IC under CAP/VOLT/RES, full PGA/current load and USB/radio activity; scope AC ripple/transients; record uncertainty and temperature. |
| HW05 | Sheet 1 SMBJ6.5CA on USB D±, sheet 2 22 Ω series + 22 pF shunts. TVS/RC/layout require signal-integrity review; BLE operation cannot rule it out. | Replace D± protection with a qualified low-capacitance USB 2.0 ESD array (candidate [TPD2EUSB30](https://www.ti.com/lit/ds/symlink/tpd2eusb30.pdf), confirm package/pinout/clamp levels); keep VBUS protection separately selected. Reserve 22/33 Ω series positions and DNP shunts pending measurement; route matched differential pair. | Use actual ESD device capacitance/leakage/clamp specs, connector parasitics and MCU pad limits; do not transfer SMBJ VBUS assumptions to D±. No guaranteed USB fix is claimed without compliance/eye evidence. | Test known-good cable/port/hub, enumerate OS devices in app and ROM-download modes, capture USB analyzer/eye if available, scope VBUS/EN. COM8 FileNotFoundError only proves that name was unavailable at that open. |
| HW06 | R38–R45=10 kΩ each connect ADS-side nodes to negative rail in VOLT. | Add independently switchable resistor bank or a separate high-impedance VOLT buffer/path. Candidate switches must cover AVSS…AVDD and have fail-safe control; exact BOM pending source impedance/accuracy requirements. | For source Vs with Rs, Vnode=(Vs/Rs+AVSS/10k)/(1/Rs+1/10k); error depends on Rs and cannot be corrected by a fixed offset. Switch Ron alters RES resistor value; leakage×Rs, charge injection and capacitance limit VOLT accuracy/bandwidth. Calibrate RES only after final topology. | Verify populated resistors. Use safe reference source + known series impedances, measure node loading across range and temperature; verify settling after every row/mode transition. |

To make internal rail monitoring transparent, a future revision must expose a real all-open U5 enable or a separately qualified reference disconnect. It must default to isolation during boot, reset and power-domain loss. Add/remove-Q1 sequencing must be interlocked with INTREF; measure the actual REF/S-node waveform and leakage before enabling a firmware capability. Firmware currently offers no menu toggle that pretends this hardware exists.

## Manuals reviewed

Local ADS126x manual `datasheets/ads1263.pdf`: pin/function and recommended supply tables; internal reference §9.3.8.1 and startup §11.3 (1 µF minimum 50 ms guidance); POWER/RESET/INTERFACE registers; ADC1 data integrity. Sheet 3 C42 is 1 µF to AVSS, but the populated value is NOT_VERIFIED. Reference startup, input RC/PGA setup, digital filter latency and a fresh conversion are separate conditions.

Primary manufacturer references additionally consulted 2026-11-04: [TI TMUX1108 datasheet](https://www.ti.com/lit/ds/symlink/tmux1108.pdf), [TI LM27762 datasheet](https://www.ti.com/lit/ds/symlink/lm27762.pdf), [Espressif ESP32-S3 schematic checklist](https://docs.espressif.com/projects/esp-hardware-design-guidelines/en/latest/esp32s3/schematic-checklist.html). These support switch/enable behavior and USB pin/layout review; they do not prove the board's actual condition.

## Observations from the 2026-11-04/05 passive-fixture run

No HW01–HW06 change was performed. The user has no independent AVDD/AVSS DMM readings, and has not yet confirmed the S8D8 resistor connection. S8D8 repeatedly reports fresh OPEN; this does not identify whether the resistor, contact or board path is responsible. No coordinate remap, calibration coefficient or fixed offset was used to hide it.

The ADS internal supply monitor inherited PGA bypass from the last matrix cell, contrary to ADS1262 §9.3.5. Firmware `8d6d656` gives that monitor its own verified PGA-enabled/gain-1, chop-OFF profile and restores the prior profile. This corrects a software prerequisite; it does not establish independent AVDD/AVSS accuracy or add physical isolation. CAP/VOLT transparent internal monitoring remains unavailable. RES already intentionally enables REFOUT and may monitor under that existing safe admission policy.

SPI CRC rejection remains observable with received CRC differing from computed CRC in the final bit. The 2 MHz duty64 experiment failed and was reverted to duty128; the firmware keeps all eight CRC bits and bounded fresh-conversion retry. The protected register read adds one legal trailing register so requested configuration bytes are not the last response byte. This mitigates the tested end-of-read corruption mechanism but does not prove its electrical origin or repair the RDATA CRC byte. Scope SCLK/MISO at the ADS and MCU pads, verify mode-1 setup/hold and output timing, and inspect the actual SPI route before assigning a hardware cause. No unverified sampling-delay setting or unsupported ADC2 is claimed as a fix.

The subsequent complete-RDATA-plus-NOP experiment also reproduced CRC rejection and was removed in `bd49168`. Final flashed firmware `ebabe7e` still failed the RES screening after 26 complete frames. Successful CAP operation or bounded recovery does not clear this ADC-link finding. The USB VFS path and fragmented-stdio defects were separately reproduced and fixed in software; the Linux serial/BLE recordings do not clear HW05 or establish Windows enumeration reliability. No PCB rework, independent rail measurement, scope capture or HW01–HW06 electrical acceptance was performed.

## 2026-11-13 fixture and startup-scan update

The user confirmed and repaired the loose S8D8 connection; both 10 kΩ reference
positions now produce resistance values rather than OPEN in the recorded RES
screen. User DMM observations at the ADS supply pins/nearby decoupling are
AVSS ≈ −1.795 V ± 0.005 V and AVDD 3.33–3.37 V. The user suspects pickup and
estimates actual variation below 10 mV; this is not a measured uncertainty bound.
The temporary RAM comparison uses +3.350/−1.795 V with uncertainty=unknown.
Do not subtract R54's drop again from measurements already made at the ADS.
No persistent calibration or hardware rework was performed.

The new startup probe selected requested 900 kHz / driver-reported 909 kHz in
both final diagnostic boots. The scan retains all CRC bits and permits up to
2.5% CRC rejection in each finite qualification phase. Runtime CRC rejection
still occurs; the new partial-frame handling does not repair its physical cause.
The SPI frequency is an IDF observation, not a measured SCLK waveform. Actual
three-chip power removal/return while the MCU remains powered, switch topology,
supply edges, back-power current and SPI timing remain NOT_RUN. A software
restart that repeats the scan is not a substitute for that physical test.

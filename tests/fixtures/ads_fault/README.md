# Synthetic ADS fault replays

These are synthetic contexts emitted by the **production** `sensorarrayAdsFaultFormat`, not E1–E7 field logs. Their timestamp, boot, sequence and register values are explicit fixture inputs. No truncated original line was repaired or replaced.

Regenerate with `python3 tools/test_ads_fault_fixtures.py --update`, then verify with `python3 tools/test_ads_fault_fixtures.py`. Both were actually executed successfully. The script compiles the C formatter and emitter with the host compiler, checks first-error/pre-cleanup mode preservation, separate cleanup error, per-record 384-byte bounds, attempted/acquired flags and exact fixture equality.

These diagnostic text records have no ASCII measurement CRC. Measurement golden frames and actual encoder CRCs are separate under `tests/fixtures/config_identity`.

`reg` is the register-cache index for stable health comparisons (POWER=0, INTERFACE=1, MODE0=2, MODE1=3, MODE2=4, INPMUX=5, REFMUX=6, OFCAL0..2=7..9, FSCAL0..2=10..12). For REGISTER_READ_UNSTABLE it is the physical register address of the first disagreeing byte. ROW_GPIO_COMPARE uses the selected row. OWNER_ACQUIRE expected/actual are owner enum values. This is diagnosis metadata and must be interpreted with `stage`.

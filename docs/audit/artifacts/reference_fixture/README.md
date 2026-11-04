# Passive-fixture HIL evidence

These are actual command results and recordings from the authorized ESP32-S3 / ADS1262 fixture run on 2026-11-04/05. **The complete reliability and precision acceptance failed.** A filename beginning `release` or `final` is a run label, not a PASS assertion. Read the [HIL report](../../reference_fixture_hil_20261104.md) and [complete final index](release-evidence-index-final.json) first.

- Final flashed source: `ebabe7e6816c0347c6d1f5187d5f7f801a7ef98d`; see `serial-lines-final-build-manifest.json`, `serial-lines-final-flash-result.json` and `serial-lines-final-test-results.json` for the exact image, commands and exits. All 34 host-test entrypoints passed; this does not imply all HIL passed.
- The complete suite was also actually rerun on the committed `ebabe7e` source: `final-committed-test-results.json` records 34 exits of 0. The earlier suite manifest retains its original pre-commit HEAD and dirty hash.
- `release-res8-01` failed with MATRIX_CRC/0x109 after 26 frames. Earlier strict switch testing failed at request 29. The separate 50-switch control/identity PASS does not replace these failures. S8D8 remains OPEN; 33pF values require unloaded baselines. No calibration was written.
- `release-final-state` records the last query: CAP / 8 rows / USB DEBUG. Recovery capture is only 22.332 seconds, not a two-minute screening pass.
- `firmware-final-changed-files.txt` and `firmware-final-function-diff.patch` are baseline `8045e9e` → `ebabe7e`. The similarly named files without `final` retain an earlier snapshot. Historical manifests and unsuccessful candidate logs are kept without relabeling.
- `final-source-verification.json` compares 153 source files to the flashed build. `final-analysis-command-results.json` records the five actual replay commands and their statuses; REVIEW_REQUIRED with exit 0 is not acceptance PASS.

## Recorded versus generated material

Serial `.rx.bin` files are original received bytes. BLE `notifications.jsonl` contains exact notification hex and receive timestamps; `logical.jsonl` is the receiver's reassembly output, not an independent transport observation. `.events.jsonl` includes actual commands/timestamps and collector decisions. `.summary.json`, `.analysis.json` and comparison/audit JSONs are derived reports. Partial or interleaved diagnostic lines are retained as received; none were filled in as complete original records.

Raw binaries and the two large uncompressed BLE JSONL files remain local and ignored. Tracked `.gz` files decompress byte-for-byte to those originals. Each archive manifest records both compressed and uncompressed SHA256. The final index references these archive paths for replay. `scripts/` preserves the actual one-off collection/build/test helpers; these scripts are not additional test executions by their mere presence. Archived helper versions that failed are historical evidence.

The recorded encoder slices in [tests/fixtures/reference_capture](../../../../tests/fixtures/reference_capture/) carry source offsets and hashes. They use the actual received CRC, not a hand-generated substitute. Synthetic protocol/fault injection fixtures live separately and retain their provenance.

## Replay without overwriting data

From the repository root, choose a new temporary directory and decompress into it (leave the tracked archive and ignored original intact):

```sh
replay_dir=$(mktemp -d /tmp/sensorarray-replay.XXXXXX)
gzip -dc docs/audit/artifacts/reference_fixture/release-cap8-01.rx.bin.gz > "$replay_dir/cap8.rx.bin"
python3 tools/reference_fixture.py --input "$replay_dir/cap8.rx.bin" --provenance recorded --output "$replay_dir/analysis.json"
```

Use the archive manifest to check SHA before replay. A CRC-valid transport frame can still contain an ADC-rejected X02 or OPEN; fresh/acquired alone does not make that point valid. Do not compare numeric generations or sequences across boot sessions. No raw ADC code should be inferred from a derived physical-value report and called a recorded sample.

The private pre-flash backup under ignored `validation_artifacts/` contains NVS and is deliberately excluded. This evidence directory contains no firmware flash image or calibration backup. No push or publication was performed.

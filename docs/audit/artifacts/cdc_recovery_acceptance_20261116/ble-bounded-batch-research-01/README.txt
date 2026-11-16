Finite BLE batch priority research — NOT ADMITTED TO TARGET

Only this archive and /tmp were written. No deployed BLE, SDK, old test, build
configuration or generic tools/test_* entry changed. All inputs are synthetic;
no target build, radio, USB hardware or serial access occurred.

Reproduce against a matching checkout (source hashes are in model-result.json):
  SENSORARRAY_MODEL_REPORT=/tmp/ble-batch-new-result.json python3 docs/audit/artifacts/cdc_recovery_acceptance_20261116/ble-bounded-batch-research-01/test_ble_bounded_batch_model.py
The result path is exclusive-create. The executable uses native host C/UBSan.
The tool generates the candidate only in TemporaryDirectory. This archived
copy is intentionally outside tools/ and is not added to the general suite.

45 pipeline variants: 39 finite invariant cases;6 NOT_PROVABLY_BOUNDED.
6 serial stress comparisons retain all49 original assertions. Three variants
intentionally abort: original permanent8 source55/200 failure; IDF baseline's
non-reproduced negative low-priority-loss assumption; IDF permanent8's old
same-priority-handoff expectation. The research harness verifies and preserves
those exact negative results. Exit0 is not a target throughput/safety PASS.

Bounded batch gave no complete-message gain in the modeled saturated pairs.
Target admission remains HOLD. See review.json and complete command.txt.

The initial new-model failures are retained with an exact reconstructed source
whose SHA matches result-01.json. They concern test-only epoch ownership and
in-flight accounting assumptions; no existing production/old assertion was
changed to make them pass. command.json records the final real invocation,
exit status, wall time and source hashes.

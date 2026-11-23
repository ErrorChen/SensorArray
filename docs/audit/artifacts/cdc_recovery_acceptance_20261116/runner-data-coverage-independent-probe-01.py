"""Synthetic gate counterexample; no raw/HIL/PC arrival record is invented."""
import hashlib, json, sys
from pathlib import Path
stage=Path(__file__).resolve().parent
sys.path.insert(0,str(stage/'scripts'))
import qualify_closed_capture as gate
expected=gate.ExpectedCaptureIdentity('ab'*32,'AABBCCDDEE112233',1)
identity=dict(session=expected.session,bootId='1',rev='8',mgen='3',mrid='4',rgen='2',rrid='6',pgen='3',prid='4')
counts=dict(ordered_integrity_valid_frames=10000,missing_frames=0)
ble=dict(status='RECORDED',capture_frame_counts=dict(ordered_integrity_valid_frames=2,missing_frames=0),
 configuration_groups=[dict(identity=identity)],
 fragment_integrity=dict(fully_contained_by_channel={'D':{}},edge_or_unknown_by_channel={}),
 fully_contained_data_diagnostics={},USB_comparison=dict(USB_unique_integrity_valid_frames=10000,common_identity_sequence_frames=2,same_wire_bytes_sha256=2,different_wire_bytes_sha256=0,USB_frames_without_BLE_match=9998))
report=dict(captures=[dict(label='synthetic-two-frames-only',status='RECORDED',window=dict(requested_seconds_met=True,observed_seconds=120),
 capture_counts=counts,expected_config_identity=identity,configuration_groups=[dict(identity=identity)],ble=ble)],
 ble_journal=dict(schema_valid=True,recording_end_observed=True,collector_summary_verification=dict(status='MATCH'),
 whole_journal_critical_records={},all_fragment_counters_by_channel={}))
result=gate.assess_science(report,'synthetic-two-frames-only',expected)
print(json.dumps(dict(kind='synthetic_report_gate_probe_not_HIL',
 source_sha256=hashlib.sha256(Path(gate.__file__).read_bytes()).hexdigest(),
 input=report,result=result,requirement='must_not_pass_complete_120s_case_from_only_two_adjacent_BLE_frames'),indent=2))

#!/usr/bin/env python3
"""CAP8 diagnostic: ordinary BLE SHORT previews plus protected full packets.

Uses the real bounded CDC benchmark, with one temporary TX setter and a
query-verified restore inside its original total deadline. No flash, reset,
calibration persistence, scientific smoothing or acceptance claim. A separate
receive-only BLE observer is required for PC receipt evidence.
"""
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[5]
sys.path.insert(0, str(ROOT / 'tools'))
sys.path.insert(0, str(ROOT / 'docs/audit/artifacts/usb_otg_ble_priority_20261114/scripts'))
import capture_cdc_benchmark as cdc
from capture_reference_fixture import CaptureError
from text_protocol import parse_fields


class ShortSnapshotCollector(cdc.CdcBenchmarkCollector):
    variant = 'formal_cdc_ble_short_snapshot_diagnostic_v2'
    runner_source_file = __file__
    query_requirements = {**cdc.CdcBenchmarkCollector.query_requirements, 'TX?': 'cmd v'}

    @classmethod
    def validate_fields(cls, command, fields):
        if command == 'TX?':
            if fields.get('cmd') != 'TX' or fields.get('v') not in {'short', 'rel', 'full'}:
                raise cdc.shared.retry.InvalidQueryReply('TX_query_contract_invalid')
            return fields
        return super().validate_fields(command, fields)

    def __init__(self, *args, **kwargs):
        self.reset_event_count = 0
        super().__init__(*args, **kwargs)
        self.original_tx = None
        self.tx_change_attempted = False
        self.tx_restore = 'NOT_NEEDED'

    def event(self, kind, **fields):
        # The parent retains at most 16 reset examples. Evidence counts must
        # keep progressing when that bounded sample list is saturated.
        if kind == 'reset_observed':
            self.reset_event_count += 1
        if kind == 'finished':
            # Parent capture/cleanup finished; TX restoration still belongs
            # to this transaction. Only the final result below closes it.
            kind = 'base_finished'
        super().event(kind, **fields)

    def set_tx_once(self, mode, phase_deadline):
        mode = mode.upper()
        if mode not in {'SHORT', 'REL', 'FULL'}:
            raise CaptureError('unrecognized_TX_mode')
        deadline = self.command_deadline(phase_deadline)
        self.lines.clear()  # Retained in raw; not a device input flush.
        self.send('TX=' + mode, deadline)
        self.wait(lambda line: line.startswith('ACK,') and
                  parse_fields(line).get('cmd') == 'TX' and
                  parse_fields(line).get('v') == mode.lower(),
                  deadline, 'TX=' + mode)
        fields = self.query('TX?', 'ACK', phase_deadline)
        if fields.get('cmd') != 'TX' or fields.get('v') != mode.lower():
            raise CaptureError('TX_query_not_verified:' + mode)

    def snapshot_window(self, name, deadline):
        if name == 'before' and self.original_tx is None:
            if self.configuration.mode != 'CAP' or self.configuration.rows != 8:
                raise CaptureError('short_snapshot_diagnostic_requires_CAP8')
            if 'ble_snapshot_v1' not in self.reported_capabilities:
                raise CaptureError('protected_full_snapshot_capability_required')
            fields = self.query('TX?', 'ACK', deadline)
            if fields.get('cmd') != 'TX' or fields.get('v') not in {'short', 'rel', 'full'}:
                raise CaptureError('original_TX_unknown_no_mutation')
            self.original_tx = fields['v']
            if self.original_tx != 'short':
                # Even a lost setter reply requires an attempted restoration.
                self.tx_change_attempted = True
                self.set_tx_once('SHORT', deadline)
            self.event('short_snapshot_profile', original_tx=self.original_tx,
                       ordinary_ble='SHORT_preview', protected_ble='original_full_packet',
                       PC_receipt='requires_independent_BLE_recording')
        return super().snapshot_window(name, deadline)

    def run(self):
        result = None
        restore_error = None
        observation_error = None
        try:
            result = super().run()
        finally:
            faults_before = self.fault_events
            resets_before = self.reset_event_count
            if self.tx_change_attempted:
                self.phase = 'tx_restore'
                try:
                    deadline = min(self.total_deadline,
                                   time.monotonic() + 2 * self.options.command_timeout)
                    self.set_tx_once(self.original_tx, deadline)
                    self.tx_restore = 'QUERY_VERIFIED'
                except (OSError, CaptureError, KeyboardInterrupt) as error:
                    self.interrupted |= isinstance(error, KeyboardInterrupt)
                    restore_error = str(error) or 'interrupted'
                    self.tx_restore = 'FAILED_OR_UNKNOWN'
                    self.event('tx_restore_error', reason=restore_error)
            if self.fault_events != faults_before or self.reset_event_count != resets_before:
                observation_error = 'tx_restore_new_fault_or_reset'
                self.event('tx_restore_observation_error', reason=observation_error)
            if result is not None:
                # super().run() returned before the final restore RX. Close
                # the evidence over that entire operation without moving the
                # scientific capture window or hiding late MFAULT/reset data.
                result.update(self.transport_result())
                result.update(ble_output_test_profile='SHORT_WITH_PROTECTED_FULL_SNAPSHOTS',
                              original_tx=self.original_tx, tx_change_attempted=self.tx_change_attempted,
                              tx_restore=self.tx_restore, tx_restore_error=restore_error,
                              tx_restore_observation_error=observation_error,
                              rx_bytes=self.raw_bytes, tx_os_accepted_bytes=self.tx_bytes,
                              partial_writes=self.partial_writes,
                              firmware_faults=list(self.faults), firmware_fault_events=self.fault_events,
                              reset_observations=self.reset_observations, interrupted=self.interrupted,
                              reset_event_count=self.reset_event_count,
                              elapsed_seconds=time.monotonic() - self.started,
                              normal_FULL_loss_budget_applicable=False,
                              snapshot_acceptance='NOT_EVALUATED')
                if observation_error:
                    result['prior_cap_recovery'] = result.get('safe_cap_recovery')
                    result['safe_cap_recovery'] = 'INVALIDATED_BY_LATER_FAULT_OR_RESET'
                if restore_error or observation_error:
                    result['status'] = 'ERROR'
                    result['error'] = result.get('error') or observation_error or 'tx_restore:' + restore_error
                super().event('finished', status=result['status'], error=result.get('error'),
                              rx_bytes=self.raw_bytes, tx_restore=self.tx_restore,
                              firmware_fault_events=self.fault_events,
                              reset_event_count=self.reset_event_count)
        return result


def main(argv=None):
    args = list(sys.argv[1:] if argv is None else argv)
    if '--transport' not in args:
        args = ['--transport', 'usb_cdc', *args]
    with cdc.shared.bulk_collector_selected(ShortSnapshotCollector):
        return cdc.shared.base.main(args)


if __name__ == '__main__':
    raise SystemExit(main())

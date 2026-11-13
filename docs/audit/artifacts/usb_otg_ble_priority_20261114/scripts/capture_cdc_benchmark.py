#!/usr/bin/env python3
"""Formal CDC0 DATA/CTRL shared-stream benchmark; CDC1 remains debug only.

Exact 303a:4042 stable-serial/interface-0 selection, no libusb or privilege
changes. Same 120s configurations / 600s stability / 32-case suite and bounded
diagnostic queries as the bulk runner. No USBSTREAM setter or USBBOOT.
"""
import sys
import capture_bulk_benchmark as shared


class CdcBenchmarkCollector(shared.BulkBenchmarkCollector):
    expected_transport = 'usb_cdc'
    expected_backend = 'otg_cdc_v1'
    required_capabilities = frozenset(('config_query_v1', 'usb_cdc_data_v1', 'work_quantum_v2'))
    query_requirements = {**shared.BulkBenchmarkCollector.query_requirements, 'USBSTREAM?': 'v scope dataEvery debug control'}
    diagnostics = shared.DIAGNOSTICS + (('USBSTREAM?', 'USBSTREAM'),)
    variant = 'formal_cdc_shared_diagnostics_v1'
    runner_source_file = __file__

    @classmethod
    def validate_fields(cls, command, fields):
        if command == 'USBSTREAM?':
            expected = {'v': 'CDC', 'scope': 'otg_data', 'dataEvery': '1', 'debug': 'cdc', 'control': 'shared_cdc'}
            if any(fields.get(key) != value for key, value in expected.items()):
                raise shared.retry.InvalidQueryReply('formal_CDC_stream_contract_required')
            return fields
        return super().validate_fields(command, fields)


def run_benchmark(options, **kwargs):
    with shared.bulk_collector_selected(CdcBenchmarkCollector):
        return shared.base.run_benchmark(options, **kwargs)


def main(argv=None):
    args = list(sys.argv[1:] if argv is None else argv)
    if '--transport' not in args:
        args = ['--transport', 'usb_cdc', *args]
    with shared.bulk_collector_selected(CdcBenchmarkCollector):
        return shared.base.main(args)


if __name__ == '__main__': raise SystemExit(main())

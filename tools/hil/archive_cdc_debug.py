#!/usr/bin/env python3
"""Archive a finalized CDC debug observer without deleting/replacing originals.

Verify its independent raw/event offsets and summary SHA before writing gzip;
then byte-verify each compressed artifact. A debug archive is not a formal
scientific DATA recording. No device, reset, command or active-stream access.
"""
import argparse
import gzip
import hashlib
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tools'))
from sensorarray_bulk_diagnostics import read_stream
from analyze_firmware_benchmark import memory_telemetry_record_error


def archive(summary_path, output):
    summary_path, output = Path(summary_path), Path(output)
    if output.exists(): raise FileExistsError('archive report already exists')
    if not summary_path.name.endswith('.debug.summary.json'):
        raise ValueError('explicit .debug.summary.json is required')
    summary = json.loads(summary_path.read_text())
    if summary.get('status') not in {'STOPPED', 'OBSERVED', 'ERROR', 'INTERRUPTED'} or not summary.get('raw_sha256'):
        raise ValueError('debug summary is not finalized; raw not opened')
    stem = summary_path.name[:-len('.debug.summary.json')]
    paths = [summary_path.parent / (stem + suffix) for suffix in ('.debug.rx.bin', '.debug.events.jsonl')]
    if any(Path(str(path) + '.gz').exists() or Path(str(path) + '.gz.partial').exists() for path in paths):
        raise FileExistsError('archive or partial output already exists; originals retained')
    verified = read_stream(*paths, summary['raw_sha256'], event_type='rx', origin=None,
                           bounds=(summary['started_monotonic'], summary['ended_monotonic']), source='CDC',
                           memory_validator=memory_telemetry_record_error)
    report = {'schema': 'cdc_debug_archive_v1', 'status': 'VERIFIED', 'scientific_DATA': False,
              'source_summary': str(summary_path), 'summary_sha256': hashlib.sha256(summary_path.read_bytes()).hexdigest(),
              'originals_retained': True, 'files': [], 'source_rx_bytes': verified['raw']['bytes']}
    expected_hashes = {paths[0]: verified['raw']['uncompressed_sha256'],
                       paths[1]: verified['events']['uncompressed_sha256']}
    for path in paths:
        temporary, target = Path(str(path) + '.gz.partial'), Path(str(path) + '.gz')
        source_sha, source_size = hashlib.sha256(), 0
        with path.open('rb') as source, temporary.open('xb') as sink:
            with gzip.GzipFile(filename='', mode='wb', fileobj=sink, mtime=0) as compressed:
                for chunk in iter(lambda: source.read(1024 * 1024), b''):
                    source_sha.update(chunk); source_size += len(chunk); compressed.write(chunk)
        back_sha, back_size = hashlib.sha256(), 0
        with gzip.open(temporary, 'rb') as restored:
            for chunk in iter(lambda: restored.read(1024 * 1024), b''):
                back_sha.update(chunk); back_size += len(chunk)
        if back_sha.digest() != source_sha.digest() or back_size != source_size:
            raise ValueError('gzip byte verification failed; original and partial retained')
        if source_sha.hexdigest() != expected_hashes[path]:
            raise ValueError('source changed since finalized evidence verification; original and partial retained')
        # link avoids check-then-rename clobbering another concurrent archive.
        import os
        os.link(temporary, target); temporary.unlink()
        report['files'].append({'source': str(path), 'gzip': str(target), 'bytes': source_size,
                                'uncompressed_sha256': source_sha.hexdigest(),
                                'gzip_sha256': hashlib.sha256(target.read_bytes()).hexdigest()})
    with output.open('x') as stream: json.dump(report, stream, indent=2); stream.write('\n')
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--debug-summary', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(argv)
    try: result = archive(args.debug_summary, args.output)
    except (OSError, ValueError, KeyError, TypeError) as error:
        print('ERROR: ' + str(error), file=sys.stderr); return 1
    print(json.dumps({'status': result['status'], 'output': str(args.output), 'files': len(result['files'])})); return 0


if __name__ == '__main__': raise SystemExit(main())

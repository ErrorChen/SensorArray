#!/usr/bin/env python3
"""Close a successful build/test pair and preserve generated images, not HIL."""
import argparse
import gzip
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[5]
OUT = Path(__file__).resolve().parents[1]


def sha(data):
    return hashlib.sha256(data).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('label')
    parser.add_argument('--change-scope', required=True)
    args = parser.parse_args()
    if not args.label or any(c not in 'abcdefghijklmnopqrstuvwxyz0123456789-_' for c in args.label):
        raise ValueError('simple_unique_label_required')
    manifests = {kind: json.loads((OUT / (args.label + '-' + kind + '-manifest.json')).read_text())
                 for kind in ('build', 'tests')}
    build, tests = manifests['build'], manifests['tests']
    inputs = build['inputs_after']
    for manifest in manifests.values():
        if (manifest['source_changed_during_run'] or not manifest['results'] or
                any(r['exit_code'] != 0 for r in manifest['results']) or
                manifest['inputs_before'] != inputs or manifest['inputs_after'] != inputs):
            raise ValueError('build_test_not_successful_identical_inputs')
    if any(sha((ROOT / path).read_bytes()) != value for path, value in inputs.items()):
        raise ValueError('current_production_inputs_changed')
    app = (ROOT / 'build/SensorArray.bin').read_bytes()
    if sha(app) != build['app_sha256'] or len(app) != build['app_bytes']:
        raise ValueError('current_binary_does_not_match_build')
    archive_path = OUT / (args.label + '-image-archive.json')
    identity_path = OUT / (args.label + '-validated-identity.json')
    paths = [archive_path, identity_path] + [OUT / (args.label + '-SensorArray.' + ext + '.gz')
                                            for ext in ('bin', 'elf', 'map')]
    if any(p.exists() for p in paths):
        raise FileExistsError('use_a_new_label')
    records = []
    for ext in ('bin', 'elf', 'map'):
        source = ROOT / ('build/SensorArray.' + ext)
        data = source.read_bytes()
        compressed = gzip.compress(data, mtime=0)
        target = OUT / (args.label + '-SensorArray.' + ext + '.gz')
        with target.open('xb') as stream:
            stream.write(compressed)
        if gzip.decompress(target.read_bytes()) != data:
            raise ValueError('archive_roundtrip_failed')
        records.append({'build_path': str(source.relative_to(ROOT)), 'raw_bytes': len(data),
                        'raw_sha256': sha(data), 'archive': str(target.relative_to(ROOT)),
                        'archive_sha256': sha(compressed), 'roundtrip_verified': True})
    archive = {'scope': 'immutable_generated_build_outputs_not_scientific_measurements', 'records': records}
    identity = {'source_commit': build['head'], 'version': build['build_version'],
                'app_bytes': len(app), 'bin_sha256': sha(app), 'elf_sha256': records[1]['raw_sha256'],
                'host_entries': len(tests['results']), 'all_exit_zero': True,
                'production_inputs': len(inputs), 'build_test_inputs_identical': True,
                'build_manifest': args.label + '-build-manifest.json',
                'tests_manifest': args.label + '-tests-manifest.json',
                'image_archive': archive_path.name, 'hardware_flashed_at_manifest_creation': False,
                'change_scope': args.change_scope,
                'generator': {'path': str(Path(__file__).relative_to(ROOT)),
                              'sha256': sha(Path(__file__).read_bytes())}}
    for path, value in ((archive_path, archive), (identity_path, identity)):
        with path.open('x') as stream:
            json.dump(value, stream, indent=2)
            stream.write('\n')
    print(json.dumps(identity))


if __name__ == '__main__':
    main()

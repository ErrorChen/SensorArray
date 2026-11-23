#!/usr/bin/env python3
"""Actual offline replay of closed screen20/21; never new HIL acceptance."""
from pathlib import Path
import hashlib
import json
import sys

STAGE = Path(__file__).resolve().parent
sys.path.insert(0, str(STAGE / 'scripts'))
import qualify_closed_capture as gate

result = {'scope': 'original recorded bytes replayed; no original summary or raw changed',
          'new_pre_mutation_authority_established_for_historical_runs': False,
          'overall_acceptance': 'NOT_ESTABLISHED', 'hardware_actions': [], 'runs': []}
for number, elf, session, verdict in (
    (20, '7f2d5ad9c73887f70d49c7f8eb45ca889c4c44fcb1fedd293299200d9b39b469', 'B73989CDD936FA54', 'FAIL'),
    (21, 'c47a3710a770506abd030d03790b58c137150c9c1cba86a596f07962114a3c4e', '630DC17010A097FF', 'PASS_NUMERIC_SCREEN')):
    directory = STAGE / f'screen-{number}'
    inputs = sorted([*directory.glob('*.rx.bin'), *directory.glob('*.events.jsonl'),
                     *directory.glob('*.ble.notifications.jsonl'), *directory.glob('*.summary.json')])
    before = {str(p.relative_to(STAGE)): hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs}
    report = gate.benchmark.analyse_directory(directory)
    decision = gate.assess_science(report, f'screen-{number}-volt1', gate.ExpectedCaptureIdentity(elf, session, 1))
    after = {str(p.relative_to(STAGE)): hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs}
    assert before == after
    assert decision['status'] == verdict, decision
    analysis_path = directory / f'screen-{number}-final-gate-replay-analysis-02.json'
    with analysis_path.open('x') as stream:
        json.dump(report, stream, indent=2)
        stream.write('\n')
    result['runs'].append({'screen': number, 'decision': decision, 'inputs_sha256_unchanged': before,
                           'analysis': gate.reference(analysis_path)})
result['sources'] = [gate.reference(Path(gate.__file__)), gate.reference(Path(gate.benchmark.__file__)),
                     gate.reference(Path(__file__))]
with (STAGE / 'final-acceptance-gate-replay-02.json').open('x') as stream:
    json.dump(result, stream, indent=2)
    stream.write('\n')
print(json.dumps({'runs': [{'screen': r['screen'], **r['decision']} for r in result['runs']],
                  'overall_acceptance': result['overall_acceptance']}))

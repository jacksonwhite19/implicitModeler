"""Replay one recorded native event from its pre-event checkpoint only."""
import argparse
import json
import os
import shutil

from run_geometry_family import OUT, run, g
from summarize_geometry_family import committed_history
from sdf_cfd_mesher_v4.common.authority import file_hash


def replay(name, source_label, destination_label, expected_after):
    source = OUT / name / source_label
    trace = committed_history(source, 'TRACE.json')
    index = next(i for i, row in enumerate(trace) if row['state'] == expected_after)
    if index == 0 or trace[index]['operation'] != 'RollbackRemesh':
        raise ValueError('PreEventCheckpointRequired')
    starting = trace[index-1]['state']
    matches = []
    for path in (OUT / name).glob('*/checkpoints/NATIVE_*.json'):
        value = json.loads(path.read_text())
        if value['scientific'] == starting:
            matches.append((path, value))
    if not matches:
        raise ValueError('MissingPreEventCheckpoint')
    path, envelope = sorted(matches, key=lambda item: str(item[0]))[0]
    destination = OUT / name / destination_label
    destination.mkdir(exist_ok=False)
    store = destination / 'checkpoints'
    objects = store / 'objects'
    objects.mkdir(parents=True)
    for row in envelope['components'].values():
        original = path.parent / 'objects' / row['file']
        assert file_hash(original) == row['sha256']
        target = objects / row['file']
        if not target.exists():
            os.link(original, target)
    shutil.copyfile(path, store / path.name)
    g.save(store / 'CURRENT.json', dict(format=envelope['format'], checkpoint=path.name,
        scientific=starting, sha256=file_hash(path)))
    g.save(destination / 'REPLAY_INPUT.json', dict(source_checkpoint=str(path),
        sha256=file_hash(path), state=starting,
        scope='One native event. Expected terminal hash is comparison evidence only.'))
    run(name, destination_label, steps=1, resume=True)
    actual = json.loads((destination / 'SHELL_RESULT.json').read_text())
    releases = [json.loads(p.read_text()) for p in destination.glob('REJECTED_PROOF_LIFETIME_*.json')]
    result = dict(starting_checkpoint=starting, expected_after=expected_after,
        actual_after=actual['final'], exact=actual['final'] == expected_after,
        wall_exact=actual['wall_exact'], first_layer_exact=actual['first_layer_exact'],
        proof_releases=releases, scope='Single production transaction, not a full requested schedule')
    g.save(destination / 'EVENT_REPLAY.json', result)
    assert result['exact'] and result['wall_exact'] and result['first_layer_exact']


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('case'); p.add_argument('source'); p.add_argument('destination')
    p.add_argument('expected_after')
    a = p.parse_args()
    replay(a.case, a.source, a.destination, a.expected_after)

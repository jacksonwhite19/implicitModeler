"""Seal control comparisons and verify retained indexes after duplicate cleanup."""
from pathlib import Path
import sys
import capsule_user_domain_handoff as h

OUT = Path(__file__).resolve().parents[1] / 'results/persistent_publication_optimization'

def snapshot():
    a = OUT / 'normal_candidate_lookup_replay_prism'
    b = OUT / 'normal_candidate_persistent_replay_prism'
    checks = {}
    implementation = h.read(a / 'NORMAL.json')['implementation']
    checks['same_implementation'] = implementation == h.read(b / 'NORMAL.json')['implementation']
    checks['source_unchanged'] = all(h.file_hash(Path(__file__).parent / name) == digest for name, digest in implementation.items())
    for sub in ('numeric_delta', 'numeric_lookup'):
        for path in (a / sub).glob('*.npy'):
            checks[sub + '/' + path.name] = h.file_hash(path) == h.file_hash(b / sub / path.name)
    for name in ('candidate_lookup_replay', 'candidate_persistent_replay'):
        checks[name + '_science'] = all(h.read(OUT / ('normal_' + name + '_prism_EQUIVALENCE.json'))['checks'].values())
    h.durable_json(OUT / 'DETERMINISTIC_REPLAY.json', dict(passed=all(checks.values()), checks=checks, implementation=implementation, implementation_identity=h.identity(implementation)))
    assert all(checks.values())

def reopen():
    from composable_state import CommittedState
    import recursive_epoch_recovery as recovery
    from prism_persistent_hotpath import bind_committed_geometry
    from persistent_numeric_incidence import open_geometry
    authority = h.read(OUT / 'RETAINED_GEOMETRY_AUTHORITY.json')
    record = recovery.discover(OUT.parent / 'prism_assembly_optimization/epoch_access')
    reference = next(n['reference'] for n in record['nodes'] if n['content'] == authority['scientific'])
    tick = h.measured()
    with h.guards(), CommittedState(reference) as state:
        bind_committed_geometry(state, authority['binding'], authority['identity'])
        with open_geometry(authority['binding'], state.content_root) as index:
            count = len(index)
            assert count == 962901
            for key in index.delta.lookup.hashes[:128]:
                assert index.scientific_face(bytes(key).decode())
        state.check()
        result = dict(passed=True, content=state.content_root, access=state.root, faces=count, historical_proof_rows=0, historical_quality_rows=0, full_prefix_hashes=0, deep_copies=0, metrics=h.metrics(*tick))
    h.durable_json(OUT / 'POST_CLEANUP_REOPEN.json', result)

if __name__ == '__main__':
    {'snapshot': snapshot, 'reopen': reopen}[sys.argv[1]]()

"""Preserve exact control evidence and remove only this task's duplicate bulk."""
from pathlib import Path
import ast
import shutil
import sys
import capsule_user_domain_handoff as h
from retain_persistent_evidence import allocation

OUT = Path(__file__).resolve().parents[1] / 'results/remaining_prism_critical_path'


def normalize():
    path = Path(__file__).with_name('critical_prism_transition.py')
    before = path.read_bytes()
    after = before.rstrip() + b'\n'
    assert ast.dump(ast.parse(before)) == ast.dump(ast.parse(after))
    import hashlib
    path.write_bytes(after)
    h.durable_json(OUT / 'WHITESPACE_NORMALIZATION.json', dict(ast_equal=True,
        before=hashlib.sha256(before).hexdigest(), after=h.file_hash(path),
        scope='Trailing empty line only; tested production AST unchanged'))


def cleanup():
    assert not (OUT / 'STORAGE.json').exists()
    for name in ('CORE_REGRESSION.json', 'CERTIFICATE_REGRESSION.json',
                 'DETERMINISTIC_REPLAY.json', 'DEEP_candidate_clean.json'):
        assert h.read(OUT / name)['passed']
    root = OUT.resolve()
    records = []
    for directory in sorted(root.iterdir()):
        if not directory.is_dir() or not (directory.name.startswith('normal_') or directory.name == 'raw_remaining_optimized_core'):
            continue
        for path in sorted(directory.rglob('*')):
            if not path.is_file():
                continue
            path.resolve().relative_to(root)
            preserve = path.suffix in ('.prof', '.txt', '.xml') or (path.suffix == '.json' and path.stat().st_size < 500000)
            records.append(dict(path=path.relative_to(root).as_posix(), bytes=path.stat().st_size,
                                allocated=allocation(path), sha256=h.file_hash(path), remove=not preserve))
    h.durable_json(OUT / 'CONTROL_FILE_EVIDENCE.json', dict(files=records))
    before = shutil.disk_usage(root).free
    for record in records:
        if record['remove']:
            path = (root / record['path']).resolve()
            path.relative_to(root)
            assert path.stat().st_size == record['bytes']
            path.unlink()
    remaining = [p for p in root.rglob('*') if p.is_file()]
    h.durable_json(OUT / 'STORAGE.json', dict(starting_free=h.read(root / 'PREFLIGHT.json')['free'],
        before_cleanup_free=before, ending_free=shutil.disk_usage(root).free,
        control_logical=sum(r['bytes'] for r in records), control_allocation=sum(r['allocated'] for r in records),
        reclaimed_allocation=sum(r['allocated'] for r in records if r['remove']),
        retained_logical=sum(p.stat().st_size for p in remaining), retained_allocation=sum(allocation(p) for p in remaining),
        removed_files=sum(r['remove'] for r in records), temporary_peak='Not continuously sampled; control allocation is pre-cleanup allocation'))


def reopen():
    from composable_state import CommittedState
    import recursive_epoch_recovery as recovery
    from prism_persistent_hotpath import bind_committed_geometry
    from persistent_numeric_incidence import open_geometry
    authority = h.read(OUT.parent / 'persistent_publication_optimization/RETAINED_GEOMETRY_AUTHORITY.json')
    record = recovery.discover(OUT.parent / 'prism_assembly_optimization/epoch_access')
    reference = next(n['reference'] for n in record['nodes'] if n['content'] == authority['scientific'])
    tick = h.measured()
    with h.guards(), CommittedState(reference) as state:
        bind_committed_geometry(state, authority['binding'], authority['identity'])
        with open_geometry(authority['binding'], state.content_root) as index:
            assert len(index) == 962901
            for key in index.delta.lookup.hashes[:128]:
                assert index.scientific_face(bytes(key).decode())
        state.check()
        result = dict(passed=True, content=state.content_root, access=state.root,
            historical_proof_rows=0, historical_quality_rows=0, full_prefix_hashes=0,
            deep_copies=0, metrics=h.metrics(*tick))
    h.durable_json(OUT / 'POST_CLEANUP_REOPEN.json', result)


if __name__ == '__main__':
    {'normalize': normalize, 'cleanup': cleanup, 'reopen': reopen}[sys.argv[1]]()

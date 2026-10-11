"""Archive hashes and measurements, then remove only task-created duplicate bulk."""
from pathlib import Path
import shutil
import capsule_user_domain_handoff as h

OUT = Path(__file__).resolve().parents[1] / 'results/persistent_publication_optimization'
KEEP = 'normal_candidate_lookup_replay_prism'

def allocation(path):
    from path_preserving_deduplication import locked, standard
    with locked(path) as handle:
        return standard(handle)['allocated']

if __name__ == '__main__':
    assert not (OUT / 'STORAGE.json').exists(), 'One-shot cleanup already completed'
    root = OUT.resolve()
    controls = [p for p in root.iterdir() if p.is_dir() and (p.name.startswith('normal_') or p.name == 'raw_persistent_optimized_core')]
    records = []
    for directory in controls:
        for path in sorted(directory.rglob('*')):
            if not path.is_file():
                continue
            path.resolve().relative_to(root)
            rel = path.relative_to(root)
            retained_index = rel.parts[0] == KEEP and rel.parts[1] in ('numeric_delta', 'numeric_lookup')
            preserve = retained_index or path.suffix in ('.prof', '.txt', '.xml') or (path.suffix == '.json' and path.stat().st_size < 500000)
            records.append(dict(path=rel.as_posix(), bytes=path.stat().st_size, allocated=allocation(path), sha256=h.file_hash(path), remove=not preserve))
    h.durable_json(OUT / 'CONTROL_FILE_EVIDENCE.json', dict(files=records))
    plan = h.read(root / KEEP / 'CANDIDATE_PLAN.json')
    binding = plan['persistent_geometry']
    h.durable_json(OUT / 'RETAINED_GEOMETRY_AUTHORITY.json', dict(binding=binding, identity=h.identity(binding), scientific=plan['scientific'], qualification='DEEP_candidate_lookup_replay.json'))
    before = shutil.disk_usage(root).free
    for record in records:
        if record['remove']:
            path = (root / record['path']).resolve()
            path.relative_to(root)
            assert path.stat().st_size == record['bytes']
            path.unlink()
    remaining = [p for p in root.rglob('*') if p.is_file()]
    h.durable_json(OUT / 'STORAGE.json', dict(starting_free=335056535552, before_cleanup_free=before, ending_free=shutil.disk_usage(root).free,
        control_logical=sum(r['bytes'] for r in records), control_allocation=sum(r['allocated'] for r in records),
        reclaimed_allocation=sum(r['allocated'] for r in records if r['remove']),
        retained_logical=sum(p.stat().st_size for p in remaining), retained_allocation=sum(allocation(p) for p in remaining),
        temporary_peak='Not sampled continuously; pre-cleanup control allocation is measured', removed_files=sum(r['remove'] for r in records)))

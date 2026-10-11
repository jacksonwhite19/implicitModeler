"""Join measured retained RSS with exact checkpoint population counts."""
import json
import sys

from run_geometry_family import OUT, g
from summarize_geometry_family import committed_history


def run(name, label):
    root = OUT / name
    envelopes = {}
    for path in root.glob('*/checkpoints/NATIVE_*.json'):
        value = json.loads(path.read_text())
        envelopes[value['scientific']] = (path, value)
    rows = []
    for step in committed_history(root / label, 'TRACE.json'):
        path, envelope = envelopes[step['state']]
        populations = {}
        for family in ('certificate', 'quality'):
            component = envelope['components'][family]
            populations[family] = dict(records=component['count'],
                disk_bytes=(path.parent / 'objects' / component['file']).stat().st_size)
        rows.append(dict(state=step['state'], layer=step['layer'],
                         operation=step['operation'], cells=step['cells'],
                         post_commit_pre_scratch_release_rss=step['rss'], populations=populations))
    g.save(root / label / 'MEMORY_TRACE.json', dict(rows=rows,
        interpretation='RSS is measured after commit, before the runner clears candidate scratch. Receipt counts and disk bytes are measured from the exact checkpoint. RSS includes allocator retention; this is neither a steady-state sample nor an allocation-class profile.',
        ownership_audit='Candidate validation temporarily materializes proofs and native receipt bindings. Candidate scratch dictionaries are cleared after each commit. Native restart tables are disk-backed. History records contain policies and cavity manifests, not a retained full proof population per step.',
        limitation='No claim that the multi-event transient working set equals the optimized SF2 working set. No dense global optimization matrix is constructed.'))


if __name__ == '__main__':
    run(*sys.argv[1:])

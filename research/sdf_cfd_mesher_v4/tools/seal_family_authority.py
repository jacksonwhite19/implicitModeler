"""Hash the completed geometry-family evidence without changing scientific inputs."""
import hashlib
import json
import subprocess
from pathlib import Path

from summarize_geometry_family import OUT, read, write

ROOT = Path(__file__).resolve().parents[1]
REPO = ROOT.parents[1]
INPUT_HEAD = '2607f8990c00319d74235c78d251f3130a05ec80'


def sha(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def run():
    summary = read(OUT / 'CASE_SUMMARY.json')
    pending = [r['name'] for r in summary['cases']
               if r['active_pending'] and r['reason'] in
               (None, 'CompleteQualificationNotEstablished')]
    if pending:
        raise ValueError('UnresolvedExecution:' + str(pending))
    paths = [str(ROOT / name) for name in ('src', 'tools', 'tests')]
    names = subprocess.check_output(
        ['git', '-c', 'core.longpaths=true', 'diff', '--name-only', INPUT_HEAD,
         '--', *paths], cwd=REPO, text=True).splitlines()
    names += subprocess.check_output(
        ['git', '-c', 'core.longpaths=true', 'ls-files', '--others',
         '--exclude-standard', '--', *paths], cwd=REPO, text=True).splitlines()
    sources = {name: sha(REPO / name) for name in sorted(set(names))}
    files = {p.name: sha(p) for p in sorted(OUT.iterdir())
             if p.is_file() and p.name not in {'AUTHORITY.json', 'SHA256SUMS.txt'}}
    record = dict(
        classification=summary['classification'],
        input_head=INPUT_HEAD,
        report_sha256=files['report.md'], artifacts=files, changed_sources=sources,
        prior_authority=read(OUT / 'AUTHORITY_VERIFICATION.json'),
        counts={k: v for k, v in summary.items() if k != 'cases'},
        cases=[{k: r[k] for k in ('name', 'classification', 'reason', 'scientific',
                                 'wall_exact', 'first_layer_exact', 'determinism')}
               for r in summary['cases']],
        trigger_audit=read(OUT / 'TRIGGER_AUDIT.json')['new_family'],
        scientific_limits=[
            'Bounded analytic geometry qualification; no new CFD controls or accuracy claim.',
            'Parameter points are fixed designs, not universal capacity thresholds.',
            'A passing prefix is not a completed requested schedule.',
            'Failed core repair is not proof of fixed-boundary infeasibility.',
            'Unknown independent trigger truth remains null, not zero.',
            'Raw native objects remain local and are bound by the artifact index.'
        ],
        production_selection='Native mixed-front runner unchanged',
        automatic_remesh_default=True,
        v3_task_modifications=0,
        spatial_convergence='SpatialConvergencePartiallyEstablished (preserved authority only)',
        next_task='C. Broader production-scale geometry qualification')
    write(OUT / 'AUTHORITY.json', record)
    (OUT / 'SHA256SUMS.txt').write_text(
        f"{sha(OUT / 'report.md')}  report.md\n"
        f"{sha(OUT / 'AUTHORITY.json')}  AUTHORITY.json\n", newline='\n')


if __name__ == '__main__':
    run()

"""Authority-bound cohort configuration and explicit evidence aggregation."""
from pathlib import Path
import json,hashlib

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'results/broader_mixed_geometry_end_to_end'
CASES=[
    dict(name='SF2',geometry='SmoothSpline extrusion',scope='production domain',
         roles=['steerable warning','no remesh','large production control']),
    dict(name='curved_wide',geometry='wide spherical cavity',scope='bounded domain',
         roles=['ordinary growth','no remesh']),
    dict(name='general_asymmetric_passage',geometry='asymmetric three-sphere SDF',scope='bounded domain',
         roles=['adjusted schedule','committed upper-stack rollback','mixed interface']),
    dict(name='clean_curved_passage',geometry='two-sphere curved passage',scope='bounded domain',
         roles=['exact collision','termination remesh','mixed interface']),
    dict(name='analytic_cavity_with_control',geometry='two spherical cavities',scope='bounded domain',
         roles=['capacity limit','termination remesh','mixed interface']),
    dict(name='clean_narrowing_channel',geometry='six-plane narrowing channel',scope='bounded domain',
         roles=['termination remesh','mixed interface']),
]


def sha(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda:stream.read(8*1024*1024),b''):h.update(block)
    return h.hexdigest()


def bind(path, authority):
    """A report filename alone is not scientific authority."""
    path=Path(path);authority=Path(authority)
    record=json.loads(authority.read_text()); digest=sha(path)
    def leaves(value):
        if isinstance(value,dict):
            for item in value.values():yield from leaves(item)
        elif isinstance(value,list):
            for item in value:yield from leaves(item)
        else:yield value
    if digest not in leaves(record):raise ValueError('UnsealedCohortInput:'+str(path))
    return dict(path=path.relative_to(ROOT).as_posix(),sha256=digest,
        authority=authority.relative_to(ROOT).as_posix(),authority_sha256=sha(authority))


def trigger_audit(cases):
    """Only directly evaluated expectations count; missing runs are not negatives."""
    evaluated=[c for c in cases if c.get('expected_remesh') is not None and c.get('remesh_count') is not None]
    return dict(evaluated=len(evaluated),not_evaluated=len(cases)-len(evaluated),
        expected_positive=sum(bool(c['expected_remesh']) for c in evaluated),
        actual_positive=sum(c['remesh_count']>0 for c in evaluated),
        false_positive=sum(not c['expected_remesh'] and c['remesh_count']>0 for c in evaluated),
        false_negative=sum(c['expected_remesh'] and c['remesh_count']==0 for c in evaluated))


def manifest():
    rows=[]
    for case in CASES:
        name=case['name']
        if name=='SF2':folder=ROOT/'results/complete_mesh_numerical_qualification';source=folder/'report.md'
        elif name=='curved_wide':folder=ROOT/'results/layer_capacity_valid_prefix';source=folder/'RESULTS.json'
        elif name=='general_asymmetric_passage':folder=ROOT/'results/mixed_front_production_migration';source=folder/'report.md'
        else:folder=ROOT/'results/local_termination_remesh_generalization';source=folder/name/'result.json'
        rows.append(dict(case,source=bind(source,folder/'AUTHORITY.json')))
    OUT.mkdir(exist_ok=True)
    (OUT/'COHORT_MANIFEST.json').write_text(json.dumps(dict(cases=rows,
        scope='Roles describe preserved evidence, never production decisions. Multiple checkpoints do not count as geometries.',
        spatial_convergence='Not newly established'),indent=2)+'\n')


if __name__=='__main__':manifest()

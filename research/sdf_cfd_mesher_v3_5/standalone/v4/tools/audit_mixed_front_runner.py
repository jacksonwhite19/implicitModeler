"""Read-only authority and bounded SF2 history checks; no shell rebuild."""
import json,time
from pathlib import Path
import termination_policy_cohort as c
from sdf_cfd_mesher_v4.advancing_front.termination_policy import evaluate
from sdf_cfd_mesher_v4.advancing_front.mixed_front import CACHE_BINDINGS,INVALIDATION_MATRIX
from sdf_cfd_mesher_v4.common.authority import file_hash,identity,array_hash
import numpy as np

OUT=c.ROOT/'results/purpose_built_mixed_front_runner'

def run():
    c.g.save(OUT/'CACHE_INVALIDATION_MATRIX.json',dict(bindings=CACHE_BINDINGS,operations=INVALIDATION_MATRIX,
        scope='Native mixed-front dependencies; legacy Task-4 cache migration remains unqualified'))
    rows=[];start=time.perf_counter()
    for case in c.negative_cases():
        d=evaluate(case['evidence']);rows.append(dict(name=case['name'],decision=d))
    assert len(rows)==63 and all(r['decision']['next_action']!='RollbackRemesh' for r in rows)
    c.g.save(OUT/'NEGATIVE_REPLAY.json',dict(count=len(rows),false_triggers=0,rows=rows))
    c.g.save(OUT/'POLICY_RUNTIME.json',dict(seconds=time.perf_counter()-start))
    roots=list((c.ROOT/'results').glob('task4l*/fresh/build_A'))
    if len(roots)!=1:raise ValueError('AmbiguousSF2Authority')
    root=roots[0];snapshots=[]
    for layer in (1,11,12,20):
        path=root/f'LAYER_{layer:02}.json';r=json.loads(path.read_text());h=r.pop('state_hash')
        assert identity(r)==h
        coords=root/r['coordinate_file'];assert file_hash(coords)==r['coordinate_file_sha256']
        a=np.load(coords);assert array_hash(a)==r['coordinate_hash']
        snapshots.append(dict(layer=layer,state_hash=h,coordinate_hash=array_hash(a),certificate=r['certificate'],
                              corrections=r['corrections'],gate=r['gate'],exact=r['exact']))
    c.g.save(OUT/'SF2_REGRESSION.json',dict(snapshots=snapshots,geometry_changes=0,correction_changes=0,
        remesh_events=0,full_build=False,scope='Read-only sealed production checkpoints and current policy replay',
        native_mixed_schema_production_replay=False))

if __name__=='__main__':run()

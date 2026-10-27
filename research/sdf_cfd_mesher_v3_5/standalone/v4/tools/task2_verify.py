"""Read-only bounded verification of stored science and the stopping witness.

This does not resume the stopped cohort or run a forecast. It repeats just the
one rejected pair to verify that the report records the qualified kernel result.
"""
from pathlib import Path
import sys,json,time
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'src'));sys.path.insert(0,str(ROOT/'tools'));sys.dont_write_bytecode=True
import numpy as np
from task2_describe import read
from sdf_cfd_mesher_v4.forecast.detailed.scientific import scientific_hash
from sdf_cfd_mesher_v4.geometry.front import load_sf2_wall,build_topology
from sdf_cfd_mesher_v4.certificate.continuous_prism_certificate import kernel
from sdf_cfd_mesher_v4.certificate.continuous_prism_certificate.runtime import CertificateSession
from sdf_cfd_mesher_v4.common.authority import file_hash
OUT=ROOT/'results/task2'
start=time.perf_counter();rank=read('ENTITY_RANKS.json');rows=read('DIRECT_PRISM_ROWS.json')
science=dict(selection=read('PATCH_SELECTION.json'),cells=rows,faces=read('DIRECT_FACE_ROWS.json'),comparisons=read('DESCRIPTOR_COMPARISONS.json'),ranks=rank['rankings'])
actual=scientific_hash(science);expected=read('DETERMINISM.json');assert actual==expected['direct_hash_run1']==expected['direct_hash_run2']
assert rank['cell_ids']==[r['cell_id'] for r in rows]
front=load_sf2_wall();top=build_topology(front.triangles,len(front.coordinates));failure=read('CONTINUOUS.json')[-1]['failure']
def side(pair):
    layer,edge=pair;v=top.edges[edge]
    def coordinates(k):
        if k==0:return front.coordinates[v]
        first=((k-1)//5)*5+1
        with np.load(ROOT/f'results/task1/forecast_layers_{first:02}_{first+4:02}.npz') as z:return z['coordinates'][k-first,v]
    q=np.concatenate((coordinates(layer-1),coordinates(layer)))
    ids=((layer-1,int(v[0])),(layer-1,int(v[1])),(layer,int(v[0])),(layer,int(v[1])))
    return kernel.Patch.binary(q,ids)
observed=CertificateSession().pair(side(failure['side_a']),side(failure['side_b']))
assert observed=={k:v for k,v in failure.items() if k not in ('side_a','side_b')}
result=dict(passed=True,seconds=time.perf_counter()-start,direct_scientific_hash=actual,qualified_rejected_pair_reproduced_exactly=True,certificate_classification=observed['classification'],pairs_rechecked=1,full_forecast_rerun=False,cohort_certification_resumed=False)
(OUT/'VERIFIER.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))

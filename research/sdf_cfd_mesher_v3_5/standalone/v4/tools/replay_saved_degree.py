"""Final implementation replay over already saved geometry; no layer advance."""
import json,time
from pathlib import Path
import numpy as np
from fractions import Fraction as F
from incremental_degree import DegreeState
from incremental_layer_validation import t
from sdf_cfd_mesher_v4.common.authority import file_hash

def main():
    root=Path(__file__).resolve().parents[1];out=root/'results/incremental_degree_filling_validation'
    geometry=root/'results/production_fast_incremental_validation/capsule_11_12/SEALED_GEOMETRY.npz'
    z=np.load(geometry);p=z['points'];tri=z['wall_triangles'];n=(z['cell_vertices'][z['layers']==1].max()+1)//2
    cells=[t.cell(v,int(source),int(layer)) for v,source,layer in zip(z['cell_vertices'],z['source_columns'],z['layers']) if layer<=11]
    witnesses={c['id']:tuple(sum(F(float(p[v,j])) for v in sorted(set(c['vertices'])))/len(set(c['vertices'])) for j in range(3)) for c in cells}
    old={c['id']:witnesses[c['id']] for c in cells if c['layer']<=10};new={c['id']:witnesses[c['id']] for c in cells}
    base=json.loads((out/'BASE_DEGREE.json').read_text());lookup=dict(zip(base['ids'],base['result']['values']))
    initial=dict(values=[lookup[i] for i in sorted(old)],unresolved=[],passed=True)
    wall=[tuple(reversed(tuple(map(int,f)))) for f in tri]
    state=DegreeState(p[:11*n],wall+[tuple(map(int,f+10*n)) for f in tri],old,initial)
    boundary=wall+[tuple(map(int,f+11*n)) for f in tri];additions=[c for c in cells if c['layer']==11]
    tick=time.perf_counter();a,m=state.update(p[:12*n],boundary,new,additions);b,k=state.update(p[:12*n],boundary,new,list(reversed(additions)))
    expected=np.load(out/'capsule_10_11/GLOBAL_DEGREE.npz')
    assert list(sorted(a.rows))==expected['ids'].tolist()
    assert [a.rows[i]['degree'] for i in sorted(a.rows)]==expected['degrees'].tolist()
    assert [a.rows[i]['ray'] for i in sorted(a.rows)]==expected['rays'].tolist()
    assert a.root==b.root and m['counts']==k['counts'] and m['affected_identity']==k['affected_identity']
    record=dict(passed=True,scope='Degree-only reconstruction from preserved coordinates; no geometry algorithm or new layer',
        implementation=state.implementation,root=a.root,counts=m['counts'],delta_identity=m['delta_identity'],
        witness_and_direction_exact=True,reversed_addition_order_exact=True,seconds=time.perf_counter()-tick,
        inputs={str(q):file_hash(q) for q in (geometry,out/'BASE_DEGREE.json',out/'capsule_10_11/GLOBAL_DEGREE.npz')})
    (out/'FINAL_CODE_DEGREE_REPLAY.json').write_text(json.dumps(record,indent=2)+'\n');print(json.dumps(record),flush=True)

if __name__=='__main__':main()

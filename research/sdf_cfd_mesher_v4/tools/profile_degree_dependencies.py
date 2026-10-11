"""Read preserved geometry; measure degree dependency populations, no advance."""
import json,time
from pathlib import Path
from fractions import Fraction
import numpy as np
from scipy.spatial import cKDTree
import cavity_volume_audit as volume
from projected_boundary_index import ProjectedBoundaryIndex

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'results/incremental_degree_filling_validation'

def main():
    z=np.load(ROOT/'results/production_fast_incremental_validation/capsule_11_12/SEALED_GEOMETRY.npz')
    p=z['points'];vertices=z['cell_vertices'];layers=z['layers'];ids=z['cell_ids'];n=len(z['wall_triangles'].flatten()) # unused sizing not authority
    old=vertices[layers<=10];new=vertices[layers==11];tri=z['wall_triangles'];nv=int(vertices[layers==1].max()+1)//2
    witness=[tuple(sum(Fraction(float(p[v,j])) for v in sorted(set(c)))/len(set(c)) for j in range(3)) for c in old]
    outer=[tuple(map(int,f+10*nv)) for f in tri];wall=[tuple(reversed(tuple(map(int,f)))) for f in tri]
    tick=time.perf_counter();receipt=volume.degree(p,wall+outer,witness);global_seconds=time.perf_counter()-tick
    if not receipt['passed']:raise ValueError('OldDegreeAuthorityFailed')
    (OUT/'BASE_DEGREE.json').write_text(json.dumps(dict(ids=ids[layers<=10].tolist(),result=receipt)))
    tick=time.perf_counter();W=np.array([[float(v) for v in w] for w in witness]);tree=cKDTree(W);selected=set();pairs=0
    for c in new:
        lo=np.nextafter(p[c].min(axis=0),-np.inf);hi=np.nextafter(p[c].max(axis=0),np.inf)
        center=lo/2+hi/2;radius=np.nextafter(max(np.max(center-lo),np.max(hi-center)),np.inf)
        candidates=tree.query_ball_point(center,radius,p=np.inf)
        keep=[i for i in candidates if np.all(W[i]>=lo) and np.all(W[i]<=hi)]
        selected.update(keep);pairs+=len(keep)
    box_seconds=time.perf_counter()-tick
    tick=time.perf_counter();P=volume.s.cert.rational(p);projected=[(v[1]-v[0],v[2]-2*v[0]) for v in P]
    changed=outer+[tuple(map(int,f+11*nv)) for f in tri]
    faceuv=[[projected[v] for v in f] for f in changed]
    lo=np.nextafter(np.array([[float(min(v[j] for v in f)) for j in range(2)] for f in faceuv]),-np.inf)
    hi=np.nextafter(np.array([[float(max(v[j] for v in f)) for j in range(2)] for f in faceuv]),np.inf)
    index=ProjectedBoundaryIndex(lo,hi);ray_selected=0;ray_pairs=0
    for w in witness:
        a=index.containing(np.array([float(w[1]-w[0]),float(w[2]-2*w[0])]))
        ray_selected+=bool(a);ray_pairs+=len(a)
    result=dict(historical=len(old),new=len(new),changed_faces=len(changed),old_global_seconds=global_seconds,
        old_direction_counts={str(k):sum(r['ray'][1]==k for r in receipt['values']) for k in range(1,9)},
        ray_projected_selected=ray_selected,ray_projected_pairs=ray_pairs,ray_query_seconds=time.perf_counter()-tick,
        cell_box_selected=len(selected),cell_box_pairs=pairs,cell_box_seconds=box_seconds,
        note='Cell boxes are a possible closed-chain support proof; not promoted to acceptance by this profile.')
    (OUT/'DEPENDENCY_PROFILE.json').write_text(json.dumps(result,indent=2));print(json.dumps(result),flush=True)

if __name__=='__main__':main()

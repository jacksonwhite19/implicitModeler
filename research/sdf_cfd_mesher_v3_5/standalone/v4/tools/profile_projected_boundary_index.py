"""Bounded exact candidate comparison on the largest frozen wall authority."""
import json,time,hashlib
import numpy as np
import geometry_family_authority as geometry
from production_scale_authority import cohort
from projected_boundary_index import ProjectedBoundaryIndex
from run_production_scale_resume import OUT,native

def main():
    spec=next(s for s in cohort() if s['name']=='separated_regions_production')
    wall,faces=geometry.build(spec)
    offset=native.g.q.load_schedule().targets[2]
    top=geometry.front(spec,offset)
    points=np.concatenate((wall,top));boundary=np.concatenate((faces[:,::-1],faces+len(wall)))
    # Same rational projection and outward binary64 bounds as the degree audit.
    import cavity_volume_audit as volume
    P=volume.s.cert.rational(points)
    projected=[(p[1]-p[0],p[2]-2*p[0]) for p in P]
    tri=[[projected[v] for v in f] for f in boundary]
    lo=np.nextafter(np.array([[float(min(p[j] for p in row)) for j in range(2)] for row in tri]),-np.inf)
    hi=np.nextafter(np.array([[float(max(p[j] for p in row)) for j in range(2)] for row in tri]),np.inf)
    sample_ids=np.linspace(0,len(faces)-1,512,dtype=int)
    samples=[];rational_samples=[]
    for f in faces[sample_ids]:
        ids=list(f)+list(f+len(wall))
        p=tuple(sum(P[v][j] for v in ids)/6 for j in range(3))
        rational_samples.append(p)
        samples.append(np.array([float(p[1]-p[0]),float(p[2]-2*p[0])]))
    t=time.perf_counter();reference=[np.flatnonzero(np.all(v>=lo,axis=1)&np.all(v<=hi,axis=1)).tolist() for v in samples];before=time.perf_counter()-t
    t=time.perf_counter();index=ProjectedBoundaryIndex(lo,hi);build=time.perf_counter()-t
    t=time.perf_counter();actual=[index.containing(v) for v in samples];after=time.perf_counter()-t
    assert reference==actual
    import subprocess
    source=subprocess.check_output(['git','show','9924097c13acbc99dea8f6bf2e8fd1988ec858d9:research/sdf_cfd_mesher_v4/tools/cavity_volume_audit.py'],text=True)
    scope={'__name__':'reference'};exec(compile(source,'reference','exec'),scope)
    t=time.perf_counter();old_degree=scope['degree'](points,boundary,rational_samples);old_seconds=time.perf_counter()-t
    t=time.perf_counter();new_degree=volume.degree(points,boundary,rational_samples);new_seconds=time.perf_counter()-t
    assert old_degree==new_degree
    z=dict(degree_exact=True,degree_passed=new_degree['passed'],dense_degree_seconds=old_seconds,index_degree_seconds=new_seconds,case=spec['name'],boundary_triangles=len(boundary),witnesses=len(samples),dense_seconds=before,index_build_seconds=build,index_query_seconds=after,
           exact_candidate_order=True,candidate_count=sum(map(len,actual)),candidate_sha256=hashlib.sha256(json.dumps(actual).encode()).hexdigest(),
           scope='Bounded lookup comparison, not a full production rerun; exact ray predicates unchanged')
    (OUT/'PROJECTED_INDEX_PROFILE.json').write_text(json.dumps(z,indent=2));print(json.dumps(z))
if __name__=='__main__':main()

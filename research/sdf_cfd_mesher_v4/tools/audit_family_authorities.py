"""Audit wall authorities independently of shell acceptance."""
import sys
sys.dont_write_bytecode=True
import time
from pathlib import Path
import numpy as np
import geometry_family_authority as a
import generalize_local_remesh as g
from native_mixed_face_certificate import certify
from sdf_cfd_mesher_v4.common.authority import file_hash

OUT=Path(__file__).resolve().parents[1]/'results/broader_geometry_family_qualification'

def run():
    rows=[]
    for spec in a.cohort()+a.multi_region_definitions():
        tick=time.perf_counter();row=dict(case=spec['name'],classes=spec['classes'],definition=spec,
            source_sha256=file_hash(Path(a.__file__)))
        try:
            x,f=a.build(spec);source=a.Source(spec);sample=source.evaluate(x)
            cert=certify(x,[tuple(map(int,z)) for z in f],range(len(f)))
            edge={}
            for face in f:
                for u,v in zip(face,np.roll(face,-1)):edge.setdefault(tuple(sorted((int(u),int(v)))),[]).append((int(u),int(v)))
            row.update(passed=cert['passed'] and all(len(v)==2 and v[0]==v[1][::-1] for v in edge.values()),
                coordinates=g.q.array_hash(x),topology=g.q.array_hash(f),sdf=source.source_identity,
                vertices=len(x),faces=len(f),wall_max_residual=float(abs(sample.phi).max()),
                certificate_relations=cert['required_pairs'],certificate=cert,
                ownership='Component primitive index from exact-distance argmin; stable source vertex and triangle IDs',
                sign='Positive in extrusion domain, negative in wall material',
                nonmanifold_edges=sum(len(v)!=2 for v in edge.values()))
        except ValueError as exc:row.update(passed=False,error=str(exc),scope='No admitted wall; no production mesh claim')
        row['seconds']=time.perf_counter()-tick;rows.append(row);g.save(OUT/'WALL_AUTHORITIES.json',rows)
        print(spec['name'],row['passed'],flush=True)

if __name__=='__main__':run()

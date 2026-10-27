"""Read-only coverage cross-check against independently re-enumerated AABBs."""
import json,time
import numpy as np
from production_first_layer_run import OUT,save
import generalize_local_remesh as g
from sdf_cfd_mesher_v4.certificate.aabb_index import AABBIndex

def run(label):
 start=time.perf_counter();folder=OUT/label;z=np.load(folder/'CANDIDATE.npz');x=z['wall'];f=z['triangles'];h=z['front']
 p,c,o=g.s.construct(x,f,[h],np.ones(len(x),int));owners,own=g.t.ownership(c,o,[tuple(reversed(t)) for t in f]);faces=[owners[k][0][1] for k in sorted(owners)]
 lo=np.array([p[list(t)].min(0) for t in faces]);hi=np.array([p[list(t)].max(0) for t in faces]);pairs=iter(AABBIndex(lo,hi).pairs(set(range(len(faces)))))
 count=0;forbidden=unresolved=0
 with (folder/'CERTIFICATE.jsonl').open() as stream:
  for line in stream:
   row=json.loads(line);expected=next(pairs,None)
   if expected is None or tuple(row['pair'])!=tuple(expected):raise ValueError('CertificateCoverageMismatch')
   count+=1
   if row['classification'] not in ('CertifiedSeparated','CertifiedAllowedIncidentContact'):
    if 'Unresolved' in row['classification']:unresolved+=1
    else:forbidden+=1
 if next(pairs,None) is not None:raise ValueError('MissingCertificateRow')
 result=dict(candidates=count,certified=count-forbidden-unresolved,forbidden=forbidden,unresolved=unresolved,missing=0,duplicate=0,unique_faces=len(faces),seconds=time.perf_counter()-start,method='Independent AABB population replay matched every stored exact/interval proof row in order')
 save(folder/'COVERAGE_RECHECK.json',result);print(result,flush=True)
if __name__=='__main__':
 import sys
 run(sys.argv[1])

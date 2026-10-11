"""Bounded, read-only worker/batch measurements on actual certificate pairs."""
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
import time
import numpy as np
import psutil
from sdf_cfd_mesher_v4.certificate.efficient import Filter
from production_phase_profile import Profile


def probe(points,faces,pairs,output):
    ids=np.asarray([list(f) if len(f)==4 else list(f)+[f[2]] for f in faces],dtype='u8')
    for i,f in enumerate(faces):
        if len(f)==4:ids[i]=np.asarray(f)[[0,1,3,2]]
    q=np.ascontiguousarray(points[ids])
    pairs=np.asarray(pairs,dtype='u4')
    native=Filter();result=[];reference=None
    profile=Profile(output/'BATCH_PROFILE.jsonl',interval=.01)
    try:
        for workers,batch in [(1,12500),(1,50000),(1,200000),(2,50000),(4,50000)]:
            started=time.perf_counter();digest=hashlib.sha256()
            with profile.phase(f'workers_{workers}_batch_{batch}'):
                chunks=(pairs[i:i+batch] for i in range(0,len(pairs),batch))
                with ThreadPoolExecutor(max_workers=workers) as pool:
                    for codes,witness in pool.map(lambda p:native.screen(q,ids,p),chunks):
                        digest.update(np.column_stack((codes,witness)).tobytes())
            value=digest.hexdigest()
            if reference is None:reference=value
            if value!=reference:raise ValueError('BatchWorkerScientificDifference')
            result.append(dict(workers=workers,batch=batch,pairs=len(pairs),seconds=time.perf_counter()-started,
                rss_bytes=psutil.Process().memory_info().rss,hash=value))
    finally:profile.close()
    samples=[json.loads(line) for line in (output/'BATCH_PROFILE.jsonl').read_text().splitlines()]
    for trial in result:
        name=f"workers_{trial['workers']}_batch_{trial['batch']}"
        trial['sampled_peak_rss_bytes']=max(r['rss_bytes'] for r in samples if r['phase'][-1]==name)
    return dict(scope='Same actual new-face candidate population; shared immutable geometry; interval screen only, not a full certifier replay',
                production_workers=1,production_batch=50000,production_settings_changed=False,
                geometry_bytes=q.nbytes,labels_bytes=ids.nbytes,pair_bytes=pairs.nbytes,trials=result)

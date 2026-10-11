"""Read-only full coverage check of the preserved migration inputs."""
import argparse,json,time
from pathlib import Path
import numpy as np
import psutil
from profile_capsule_scaling import select_pointer
from identity_qualification_audit import population
from audit_capsule_scaling import final_coverage
from audit_native_receipt_tables import audit_tables
from sdf_cfd_mesher_v4.advancing_front.native_checkpoint import NativeCheckpointStore


def audit(source,output,scientific=None):
    start=time.perf_counter();store=NativeCheckpointStore(source)
    pointer=select_pointer(source,scientific);state=store._restore(pointer)
    keys,proofs=population(state)
    vertices=sorted(state['vertices'],key=lambda v:tuple(map(int,(v.split(':')[1],v.split(':')[3]))))
    indices={key:i for i,key in enumerate(vertices)}
    points=np.asarray([state['vertices'][key] for key in vertices])
    faces=[tuple(indices[v] for v in state['faces'][key]['vertices']) for key in keys]
    coverage=final_coverage(points,faces,proofs)
    receipts=audit_tables(state['quality'],state['certificate'],len(state['cells']))
    result=dict(passed=True,input=pointer,coverage=coverage,receipts=receipts,
                seconds=time.perf_counter()-start,peak_rss=psutil.Process().memory_info().peak_wset)
    Path(output).write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result),flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('source',type=Path);p.add_argument('output',type=Path)
    p.add_argument('--scientific');a=p.parse_args();audit(a.source,a.output,a.scientific)

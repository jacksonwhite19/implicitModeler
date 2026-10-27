"""Cold restart and final whole-population coverage audit, without new proofs."""
import argparse
import json
import time
from pathlib import Path
import numpy as np
import psutil
import run_geometry_family as native
import production_scale_authority as authority
from compact_proof_rows import CompactProofRows
from native_proof_segments import LoadedSegment,SegmentProofRows
from audit_native_receipt_tables import audit_tables
from run_production_scale_resume import summary
from sdf_cfd_mesher_v4.advancing_front.native_checkpoint import NativeCheckpointStore
from sdf_cfd_mesher_v4.advancing_front.mixed_front import scientific_hash,face_key
from sdf_cfd_mesher_v4.certificate.aabb_index import AABBIndex


def final_coverage(points,faces,proofs,active=None):
    lo=np.asarray([points[list(f)].min(0) for f in faces])
    hi=np.asarray([points[list(f)].max(0) for f in faces])
    actual=iter(proofs.pair_iterator())
    count=0
    spatial=AABBIndex(lo,hi)
    import hashlib,struct
    selected=hashlib.sha256();selected_count=0
    for pair in spatial.pairs():
        if next(actual,None)!=pair:raise ValueError('FinalWholeShellProofCoverageMismatch')
        if active is not None and (pair[0] in active or pair[1] in active):
            selected.update(struct.pack('<QQ',*pair));selected_count+=1
        count+=1
    if next(actual,None) is not None:raise ValueError('FinalWholeShellExtraProof')
    if count!=len(proofs):raise ValueError('FinalWholeShellProofCountMismatch')
    result=dict(expected=count,active=count,missing=0,duplicates=0,
                exact_proofs_recomputed=0,scope='FinalWholeShellAudit; complete inclusive AABB relation enumeration')
    if active is not None:
        delta=hashlib.sha256();delta_count=0
        for pair in spatial.pairs(active):
            delta.update(struct.pack('<QQ',*pair));delta_count+=1
        if delta.digest()!=selected.digest() or delta_count!=selected_count:raise ValueError('DeltaCandidateMismatch')
        result['delta_candidate_equivalence']=dict(passed=True,pairs=delta_count,sha256=delta.hexdigest())
    return result


def run(source,case,output):
    output.mkdir(parents=True,exist_ok=False)
    started=time.perf_counter()
    store=NativeCheckpointStore(source)
    state=store.restart()
    digest=scientific_hash(state)
    restarted=time.perf_counter()
    spec=next(s for s in authority.cohort() if s['name']==case)
    provider=native.AnalyticProvider(spec,output)
    fronts,levels=provider.geometry(state)
    points,cells,outer=native.g.s.construct(provider.x,provider.tri,fronts,levels)
    owners,ownership=native.g.t.ownership(cells,outer,[tuple(reversed(f)) for f in provider.tri])
    assert ownership['passed']
    faces=[owners[k][0][1] for k in sorted(owners)]
    n=len(state['wall']['vertices'])
    ids=[f'layer:{i//n}:source:{i%n}' for i in range(len(points))]
    keys=[face_key(tuple(ids[i] for i in f)) for f in faces]
    indices={k:i for i,k in enumerate(keys)}
    certificate=state['certificate']
    segments=[LoadedSegment(s,certificate.proof_objects,indices) for s in certificate.proof_segments]
    proofs=SegmentProofRows(segments,CompactProofRows())
    audit_start=time.perf_counter()
    layer=max(c['layer'] for c in state['cells'].values())
    active={i for i,f in enumerate(faces) if any(v//n==layer for v in f)}
    coverage=final_coverage(points,faces,proofs,active)
    coverage_seconds=time.perf_counter()-audit_start
    receipts=audit_tables(state['quality'],certificate,len(state['cells']))
    values=summary(state)
    gates=values['extrema']
    assert gates['determinant']>=.0011 and gates['weight']>=.055
    assert gates['volume']>0 and gates['symmetric_volume_ratio']>=.01
    assert gates['skewness']<=3.63636 and gates['aspect']<=1000
    from certificate_batch_probe import probe
    lo=np.asarray([points[list(f)].min(0) for f in faces]);hi=np.asarray([points[list(f)].max(0) for f in faces])
    from array import array
    pair_buffer=array('I',(v for pair in AABBIndex(lo,hi).pairs(active) for v in pair))
    batching=probe(points,faces,np.frombuffer(pair_buffer,dtype=np.uint32).reshape(-1,2),output)
    result=dict(scientific=digest,restart_exact=digest==json.loads((source/'CURRENT.json').read_text())['scientific'],
                coverage=coverage,receipts=receipts,summary=values,
                batch_worker_probe=batching,
                restart_seconds=restarted-started,coverage_seconds=coverage_seconds,
                total_seconds=time.perf_counter()-started,
                peak_rss=psutil.Process().memory_info().peak_wset)
    (output/'AUDIT.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result),flush=True)


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('source',type=Path)
    parser.add_argument('output',type=Path)
    parser.add_argument('--case',default='capsule_production_control')
    args=parser.parse_args()
    run(args.source,args.case,args.output)

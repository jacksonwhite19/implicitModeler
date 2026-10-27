"""Recover complete saved geometry from this task's truncated local snapshot.

Never accepts a partial pickle as authority. Lost proof buffers are regenerated
over saved coordinates, compared with the original accepted proof hashes, and
the complete independent seal must pass before recovered continuation.
"""
import argparse,gzip,pickle,json,time,gc
from pathlib import Path
import numpy as np
import fast_milestone as m
from benchmark_production_fast import save,seal_new
from production_fast_layer import FastLayer,MemorySegment
from native_proof_segments import SegmentProofRows
from compact_proof_rows import CompactProofRows
from native_mixed_face_certificate import certify
from sdf_cfd_mesher_v4.advancing_front.execution_profile import MemoryTransaction,ExecutionProfile
from sdf_cfd_mesher_v4.advancing_front.mixed_front import face_key
from sdf_cfd_mesher_v4.common.authority import identity,file_hash

def main():
    p=argparse.ArgumentParser();p.add_argument('folder',type=Path);a=p.parse_args();started=time.perf_counter()
    path=a.folder/'milestone_16/STATE.pending.gz';record=json.loads((path.parent/'PARTIAL_STORAGE_RECOVERY.json').read_text())
    assert file_hash(path)==record['compressed_sha256']
    with gzip.open(path,'rb') as f:
        u=pickle.Unpickler(f)
        try:u.load();raise ValueError('ExpectedRecordedTruncation')
        except (pickle.UnpicklingError,EOFError):memo=u.memo.copy()
    states=[v for v in memo.values() if isinstance(v,dict) and {'vertices','cells','authority','forecast','events'}<=v.keys()]
    attributes=[v for v in memo.values() if isinstance(v,dict) and {'points','topology','degree_state','face_keys','quality','volume'}<=v.keys()]
    assert len(states)==len(attributes)==1
    segments=[v for v in memo.values() if isinstance(v,dict) and {'faces','pairs','codes','payloads','mapping','descriptor'}<=v.keys() and isinstance(v['faces'],list)]
    spec=json.loads((a.folder/'START.json').read_text())['spec'];run=object.__new__(FastLayer)
    run.spec=spec;run.provider=m.AnalyticProvider(spec,a.folder);run.folder=a.folder;run.profile=ExecutionProfile.ProductionFast;run.tx=MemoryTransaction(states[0])
    for k,v in attributes[0].items():setattr(run,k,v)
    del memo,u,states,attributes;gc.collect()
    latest=max(c['layer'] for c in run.topology.cells.values());ledger=json.loads((a.folder/f'LAYER_{latest:02d}.json').read_text())
    assert run.degree_state.root==ledger['degree_metrics']['root']
    n=len(run.provider.x)
    def source(v):return f'layer:{v//n}:source:{v%n}'
    faces=[run.topology.owners[k][0][1] for k in sorted(run.topology.owners)]
    keys=[face_key(tuple(source(v) for v in f)) for f in faces];indices={k:i for i,k in enumerate(keys)}
    restored=[]
    for values in segments:
        s=object.__new__(MemorySegment)
        for k,v in values.items():setattr(s,k,v)
        s.mapping=np.asarray([indices[k] for k in s.faces],dtype=np.uint64);restored.append(s)
    old_count=sum(s.descriptor['count'] for s in restored)
    # Identify the saved prefix from scientific relation counts, not geometry IDs.
    first=json.loads((a.folder/'START.json').read_text())['layer']
    assert old_count==json.loads((a.folder/'milestone_12/CURRENT.json').read_text())['signature']['relations']
    previous={face_key(tuple(source(v) for v in f)) for f in faces if max(f)<(first+1)*n}
    checks=[]
    for layer in range(first+1,latest+1):
        tick=time.perf_counter();local=[f for f in faces if max(f)<(layer+1)*n]
        local_keys=[face_key(tuple(source(v) for v in f)) for f in local]
        result=certify(run.points[:(layer+1)*n],local,[i for i,k in enumerate(local_keys) if k not in previous])
        row=json.loads((a.folder/f'LAYER_{layer:02d}.json').read_text())
        expected=row['decision']['trace'][-1]['proofs'][0]
        actual=identity(dict(context=row['decision']['context'],new_exact=result['proof_hash'],quality={k:v for k,v in row['counts'].items() if k!='seconds'},degree=True))
        assert result['passed'] and actual==expected and len(result['proofs'])==row['new_relations']
        restored.append(MemorySegment(result['proofs'],local_keys,indices));previous=set(local_keys)
        checks.append(dict(layer=layer,accepted_proof_hash_exact=True,new_relations=len(result['proofs']),seconds=time.perf_counter()-tick))
        print('RECOVERED_PROOFS',checks[-1],flush=True)
    run.proofs=SegmentProofRows(restored,CompactProofRows());run.face_keys=keys
    assert len(run.proofs)==ledger['relations']
    current=m.save(run,a.folder/f'milestone_{latest:02d}_recovered');restored_run=m.load(a.folder/f'milestone_{latest:02d}_recovered',a.folder)
    assert m.signature(run)==m.signature(restored_run)
    del restored_run;gc.collect()
    print('SEALING_RECOVERY',latest,flush=True);seal=seal_new(run,a.folder);save(a.folder/'SEAL.json',seal)
    save(a.folder/'RECOVERY.json',dict(passed=True,layer=latest,proof_reconstructions=checks,geometry_regenerated=False,full_seal=True,restart_exact=True,seconds=time.perf_counter()-started,current=current))
    print('RECOVERY_SEALED',latest,flush=True)

if __name__=='__main__':main()

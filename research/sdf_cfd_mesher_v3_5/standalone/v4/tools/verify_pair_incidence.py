"""Independent deep CSR reconstruction; never called by normal accounting."""
from pathlib import Path
import time,json,psutil
import numpy as np
import immutable_state_access as a
from path_preserving_deduplication import durable_json,file_hash
from clean_sheet_l4_bootstrap import metrics
from sdf_cfd_mesher_v4.common.authority import identity
from composable_state import CommittedState
OUT=Path(__file__).resolve().parents[1]/'results/l6_write_pair_scaling_resume'
def main():
    tick=(time.perf_counter(),time.process_time(),psutil.Process().io_counters());m=a.read(OUT/'L5_incidence/MANIFEST.json');total=0;postings=0
    for p,r in m['files'].items():assert file_hash(p)==r['sha256']
    with CommittedState(a.read(OUT/'PREFLIGHT.json')['parent']) as base:
        for s,(desc,objects,verify) in zip(m['segments'],base.proof_segments()):
            assert identity(desc)==s['identity'];pp=objects/desc['pairs']['file'];verify(pp)
            pairs=np.load(pp,mmap_mode='r',allow_pickle=False);faces=a.read(s['faces'])
            offsets=np.load(s['offsets'],mmap_mode='r',allow_pickle=False);ords=np.load(s['ordinals'],mmap_mode='r',allow_pickle=False)
            assert offsets[0]==0 and offsets[-1]==len(ords)==2*len(pairs)
            counts=np.diff(offsets).astype(np.int64);assert np.all(counts>=0)
            ids=np.repeat(np.arange(len(faces)),counts)
            assert np.all((pairs[ords,0]==ids)|(pairs[ords,1]==ids))
            assert np.all(np.bincount(ords,minlength=len(pairs))==2)
            differences=np.diff(ords.astype(np.int64));boundaries=offsets[1:-1].astype(np.int64)-1
            boundaries=boundaries[(boundaries>=0)&(boundaries<len(differences))];differences[boundaries]=1
            assert np.all(differences>0)
            keys=np.load(s['keys'],mmap_mode='r',allow_pickle=False);mapping=np.load(s['mapping'],mmap_mode='r',allow_pickle=False)
            assert np.array_equal(keys,np.asarray(faces,dtype='S64')[mapping]) and np.all(keys[1:]>keys[:-1])
            total+=len(pairs);postings+=len(ords)
            del pairs,offsets,ords,counts,ids,differences,keys,mapping
        assert total==m['parent_count']==len(base['certificate'])
    result=dict(passed=True,parent_pairs=total,postings_verified=postings,metadata_root=identity(m),proof_payloads_read=0,metrics=metrics(*tick),normal_path=False)
    durable_json(OUT/'INCIDENCE_DEEP.json',result);print(json.dumps(result,indent=2))
if __name__=='__main__':main()

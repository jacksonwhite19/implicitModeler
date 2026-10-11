"""Equivalent-scope pair accounting, no proof payload or geometry generation."""
from pathlib import Path
import sys,time,psutil,json
import numpy as np
import immutable_state_access as a
from composable_state import CommittedState
from incremental_pair_storage import PairIncidence
from path_preserving_deduplication import durable_json
from clean_sheet_l4_bootstrap import metrics
OUT=Path(__file__).resolve().parents[1]/'results/l6_write_pair_scaling_resume'
def main(mode):
    pre=a.read(OUT/'PREFLIGHT.json');tick=(time.perf_counter(),time.process_time(),psutil.Process().io_counters())
    with CommittedState(pre['parent']) as base:
        changed={'face:'+f:'accounting-only' for f in base['prism_front'].values()}
        if mode=='baseline':
            affected=0;scanned=0
            for d,objects,verify in base.proof_segments():
                fp=objects/d['faces']['file'];pp=objects/d['pairs']['file'];verify(fp);verify(pp)
                faces=a.read(fp);marked=np.array(['face:'+f in changed for f in faces],bool);pairs=np.load(pp,mmap_mode='r',allow_pickle=False)
                for offset in range(0,len(pairs),100000):
                    rows=pairs[offset:offset+100000];affected+=int(np.count_nonzero(marked[rows[:,0]]|marked[rows[:,1]]));scanned+=len(rows)
            assert scanned==len(base['certificate']);result=dict(changed=affected,unchanged=scanned-affected,parent_count=scanned,prior_pair_rows_scanned=scanned)
        else:
            root=a.read(OUT/'INCIDENCE_ENROLLMENT.json')['root']
            with PairIncidence(base,OUT/'L5_incidence/MANIFEST.json',root) as idx:result=idx.affected(changed)
        result.update(metrics=metrics(*tick),scope='Fresh L5 trusted open plus current-front binding accounting',old_proof_payloads=0,quality_rows=0,sql_rows=0,full_prefix_hashes=0,full_prefix_copies=0)
    durable_json(OUT/('PAIR_'+mode.upper()+'.json'),result);print(json.dumps(result,indent=2))
if __name__=='__main__':main(sys.argv[1])

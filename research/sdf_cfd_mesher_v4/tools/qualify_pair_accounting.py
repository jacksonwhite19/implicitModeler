"""L5-only incidence enrollment and bounded normal accounting control."""
from pathlib import Path
import sys,time,psutil,json,sqlite3
from unittest.mock import patch
import immutable_state_access as a
from composable_state import CommittedState
from incremental_pair_storage import build_incidence,PairIncidence
from path_preserving_deduplication import durable_json,file_hash
from clean_sheet_l4_bootstrap import metrics
OUT=Path(__file__).resolve().parents[1]/'results/l6_write_pair_scaling_resume'
def tick():return time.perf_counter(),time.process_time(),psutil.Process().io_counters()
def main():
    pre=a.read(OUT/'PREFLIGHT.json');t=tick()
    with CommittedState(pre['parent']) as base:
        root=build_incidence(base,OUT/'L5_incidence')
        durable_json(OUT/'INCIDENCE_ENROLLMENT.json',dict(root=root,metrics=metrics(*t),parent=base.reference,prior_pairs=len(base['certificate']),normal_path=False))
    t=tick()
    with patch.object(sqlite3,'connect',side_effect=AssertionError('HistoricalSQLForbidden')),CommittedState(pre['parent']) as base:
        changed={'face:'+f:'accounting-only' for f in base['prism_front'].values()}
        with PairIncidence(base,OUT/'L5_incidence/MANIFEST.json',root) as index:result=index.affected(changed)
        result.update(metrics=metrics(*t),changed_faces=len(changed),proof_payloads_read=0,quality_rows=0,scientific_hashes=0)
        durable_json(OUT/'ACCOUNTING_CANDIDATE_SEALED.json',result)
    print(json.dumps(result,indent=2),flush=True)
if __name__=='__main__':main()

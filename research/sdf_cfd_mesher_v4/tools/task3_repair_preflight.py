"""Bounded preservation audit for the Task-3 certificate repair and continuation."""
from pathlib import Path
import sys,json,datetime
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'tools')];sys.dont_write_bytecode=True
from task3_preflight import audit as earlier,git
from sdf_cfd_mesher_v4.common.authority import file_hash
OUT=ROOT/'results/task3_certificate_repair'

def audit():
    result=earlier();rows=[];base=ROOT/'results/task3'
    for name,h in [('report.md','1bbdc5906a90291966b77f1fca4cc86734d9c21c86e5d268d6c7e8bdbda275d6'),('AUTHORITY.json','5e8c72f392734b576e9da7eba5a162447211b3832bc2e2244e2cbab6b1635538')]:
        actual=file_hash(base/name);assert actual==h,(name,actual);rows.append(dict(path=str((base/name).relative_to(ROOT)),sha256=actual))
    a=json.loads((base/'AUTHORITY.json').read_text())
    for field in ['source_sha256','evidence_sha256']:
        for p,h in a[field].items():
            actual=file_hash(ROOT/p);assert actual==h,(p,actual);rows.append(dict(path=p,sha256=actual))
    result['task3_bindings']=rows
    return result

if __name__=='__main__':
    assert git('rev-parse','HEAD')=='f7ba9d30a1dd6a8e4051448af55d546f8132cfb4'
    assert git('branch','--show-current')=='feature/sdf-cfd-mesher-v4-advancing-front'
    assert not git('diff','--cached','--name-only')
    a=audit();a.update(started_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),starting_head=git('rev-parse','HEAD'),branch=git('branch','--show-current'),starting_status=git('status','--porcelain'),max_worker_rss=12884901888,heavy_workers=1)
    OUT.mkdir(exist_ok=True);(OUT/'PREFLIGHT.json').write_text(json.dumps(a,indent=2,sort_keys=True)+'\n',encoding='utf-8')
    print(json.dumps(dict(passed=True,prior=len(a['rows']),task2=len(a['task2_bindings']),task2b=len(a['task2b_bindings']),task3=len(a['task3_bindings']))))

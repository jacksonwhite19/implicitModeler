"""Bounded Task-3 authority and source preservation audit."""
from pathlib import Path
import sys,json,datetime
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'tools')];sys.dont_write_bytecode=True
from forensics_preflight import verify as earlier,git
from sdf_cfd_mesher_v4.common.authority import file_hash
OUT=ROOT/'results/task3'

def audit():
    result=earlier();rows=[];base=ROOT/'results/nominal_forecast_forensics'
    for name,h in [('report.md','a15d973515eae7bd5fbcff3ee1c90cbb24c065b7cc16b68a0822037292927a55'),('AUTHORITY.json','d0ec913d1410d0be33939fe7d02585f5907fb48dcb7f960de70c357b938e489f')]:
        actual=file_hash(base/name);assert actual==h,(name,actual);rows.append(dict(path=str((base/name).relative_to(ROOT)),sha256=actual))
    a=json.loads((base/'AUTHORITY.json').read_text())
    for field in ['source_sha256','evidence_sha256']:
        for p,h in a[field].items():
            actual=file_hash(ROOT/p);assert actual==h,(p,actual);rows.append(dict(path=p,sha256=actual))
    result['task2b_bindings']=rows
    return result

if __name__=='__main__':
    assert git('rev-parse','HEAD')=='c1a4b7679505a9d82a9cc2a591ad5c4a0dc1f04b'
    assert git('branch','--show-current')=='feature/sdf-cfd-mesher-v4-advancing-front'
    assert not git('diff','--cached','--name-only')
    a=audit();a.update(started_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),starting_head=git('rev-parse','HEAD'),branch=git('branch','--show-current'),starting_status=git('status','--porcelain'),max_worker_rss=12884901888,heavy_workers=1)
    OUT.mkdir(exist_ok=True);(OUT/'PREFLIGHT.json').write_text(json.dumps(a,indent=2,sort_keys=True)+'\n',encoding='utf-8')
    print(json.dumps(dict(passed=True,prior=len(a['rows']),task2=len(a['task2_bindings']),task2b=len(a['task2b_bindings']))))

"""Bounded authority audit for cap correction diagnostics."""
from pathlib import Path
import sys,json,datetime
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT/'src'),str(ROOT/'tools')];sys.dont_write_bytecode=True
from task3_repair_preflight import audit as earlier,git
from sdf_cfd_mesher_v4.common.authority import file_hash
OUT=ROOT/'results/task3c'
def audit():
    a=earlier();rows=[];base=ROOT/'results/task3_certificate_repair'
    for name,h in [('report.md','36ded8034e8dd0708555ba8b4ca664828f9a6ae202026ab3a5e7751cfea90e14'),('AUTHORITY.json','9534605516ca55714a6a31b75da1435e9f62d0c0ff1ba001d7bdb08b72215aee')]:
        actual=file_hash(base/name);assert actual==h,(name,actual);rows.append(dict(path=str((base/name).relative_to(ROOT)),sha256=actual))
    auth=json.loads((base/'AUTHORITY.json').read_text())
    for field in ['source_sha256','evidence_sha256']:
        for p,h in auth[field].items():
            actual=file_hash(ROOT/p);assert actual==h,(p,actual);rows.append(dict(path=p,sha256=actual))
    a['repair_bindings']=rows;return a
if __name__=='__main__':
    assert git('rev-parse','HEAD')=='0897a2c84c1ef7bd0b80f28cb14995aebf509226'
    assert git('branch','--show-current')=='feature/sdf-cfd-mesher-v4-advancing-front'
    assert not git('diff','--cached','--name-only')
    a=audit();a.update(started_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),starting_head=git('rev-parse','HEAD'),branch=git('branch','--show-current'),starting_status=git('status','--porcelain'))
    OUT.mkdir(exist_ok=True);(OUT/'PREFLIGHT.json').write_text(json.dumps(a,indent=2,sort_keys=True)+'\n')
    print(json.dumps(dict(passed=True,bindings=sum(len(a[k]) for k in ['rows','task2_bindings','task2b_bindings','task3_bindings','repair_bindings']))))

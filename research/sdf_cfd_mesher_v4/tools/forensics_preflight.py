"""Verify frozen Task-2 evidence and the bounded earlier source authorities."""
from pathlib import Path
import sys,json,datetime
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'src'));sys.path.insert(0,str(ROOT/'tools'));sys.dont_write_bytecode=True
from task2_preflight import audit,git
from sdf_cfd_mesher_v4.common.authority import file_hash
OUT=ROOT/'results/nominal_forecast_forensics'
EXPECTED={'report.md':'c70194ba70539ef6ce5ef31a567e8650f97c3aa54a99309b373abbcfc2485825','AUTHORITY.json':'310f84a630b8f436e0a80bdd83d196ac3ad40bf9371711b5eaa60f06eba49068'}
def verify():
    result=audit();rows=[]
    for p,h in EXPECTED.items():
        actual=file_hash(ROOT/'results/task2'/p);assert actual==h,(p,actual);rows.append(dict(path='results/task2/'+p,sha256=actual))
    authority=json.loads((ROOT/'results/task2/AUTHORITY.json').read_text())
    for kind in ('source_sha256','evidence_sha256'):
        for p,h in authority[kind].items():
            actual=file_hash(ROOT/p);assert actual==h,(p,actual);rows.append(dict(path=p,sha256=actual))
    result['task2_bindings']=rows
    return result
if __name__=='__main__':
    assert git('branch','--show-current')=='feature/sdf-cfd-mesher-v4-advancing-front'
    assert git('rev-parse','HEAD')=='7ba1e1ca23c47c334675abf4f97b8177f78be911'
    assert not git('diff','--cached','--name-only')
    result=verify();result.update(started_utc='2026-09-16T15:56:40Z',audit_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),starting_head=git('rev-parse','HEAD'),branch=git('branch','--show-current'),starting_status=git('status','--porcelain'))
    OUT.mkdir(exist_ok=True);(OUT/'PREFLIGHT.json').write_text(json.dumps(result,indent=2,sort_keys=True)+'\n')
    print(json.dumps(dict(passed=True,prior_checks=len(result['rows']),task2_checks=len(result['task2_bindings']))))

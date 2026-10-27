"""Read-only frozen-authority gate and bounded Task-3D start record."""
from pathlib import Path
import sys,json,datetime
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'tools')];sys.dont_write_bytecode=True
from task3c_preservation import verify
from task3c_preflight import git
from sdf_cfd_mesher_v4.common.authority import file_hash
OUT=ROOT/'results/task3d'

def audit():
    previous=verify();base=ROOT/'results/task3c';rows=[]
    for name,expected in [('report.md','680f0d49d5867244aab17c01971ccf5fa146b2651419837b953b091e177023ef'),('AUTHORITY.json','5ed30f0be4d02c5b972dbb74a37ad9a4829d3d9c92bc5c6061c238c815de2651')]:
        actual=file_hash(base/name);assert actual==expected,(name,actual)
        rows.append(dict(path=str((base/name).relative_to(ROOT)),sha256=actual))
    a=json.loads((base/'AUTHORITY.json').read_text())
    for field in ('source_sha256','evidence_sha256'):
        for p,h in a[field].items():
            actual=file_hash(ROOT/p);assert actual==h,(p,actual)
            rows.append(dict(path=p,sha256=h))
    return dict(prior_frozen_audit=previous,task3c_bindings=rows)

if __name__=='__main__':
    assert git('rev-parse','HEAD')=='2d202a2b9f6a506acdeb97726fc49f8313d987f1'
    assert git('branch','--show-current')=='feature/sdf-cfd-mesher-v4-advancing-front'
    assert not git('diff','--cached','--name-only')
    r=audit();r.update(starting_head=git('rev-parse','HEAD'),branch=git('branch','--show-current'),starting_status=git('status','--porcelain'),started_utc=datetime.datetime.now(datetime.timezone.utc).isoformat())
    OUT.mkdir(exist_ok=True)
    (OUT/'.gitignore').write_text('*.log\n')
    (OUT/'PREFLIGHT.json').write_text(json.dumps(r,indent=2,sort_keys=True)+'\n')
    print(json.dumps(dict(frozen_authorities_passed=True,task3c_bindings=len(r['task3c_bindings']),known_external_origin_drift=len(r['prior_frozen_audit']['outside_scope_origin_drift']))))

"""Read-only authority gate for the private immediate-layer study."""
from task3d_preflight import audit as previous_audit, ROOT, git
from sdf_cfd_mesher_v4.common.authority import file_hash
import json, datetime
OUT=ROOT/'results/task3e'

def audit():
    previous=previous_audit(); rows=[]; base=ROOT/'results/task3d'
    for name,expected in [('report.md','76eb72328eb8123a1d02fbddb9383d409493bf37dd8d717ac53f4fb659819cad'),('AUTHORITY.json','c73a57e416bec64165aa76a69702350a8ac1b1a7977ef187428495e66068dd1a')]:
        actual=file_hash(base/name); assert actual==expected,(name,actual)
        rows.append(dict(path=str((base/name).relative_to(ROOT)),sha256=actual))
    authority=json.loads((base/'AUTHORITY.json').read_text())
    for field in ('source_sha256','evidence_sha256'):
        for path,expected in authority[field].items():
            actual=file_hash(ROOT/path); assert actual==expected,(path,actual)
            rows.append(dict(path=path,sha256=actual))
    return dict(previous=previous,task3d_bindings=rows)

if __name__=='__main__':
    assert git('rev-parse','HEAD')=='640ccfaa6b464a0fe85ec29aee23c5bff3e52342'
    assert git('branch','--show-current')=='feature/sdf-cfd-mesher-v4-advancing-front'
    assert not git('diff','--cached','--name-only')
    r=audit();r.update(starting_head=git('rev-parse','HEAD'),branch=git('branch','--show-current'),starting_status=git('status','--porcelain'),started_utc=datetime.datetime.now(datetime.timezone.utc).isoformat())
    OUT.mkdir(exist_ok=True);(OUT/'.gitignore').write_text('*.log\n')
    (OUT/'PREFLIGHT.json').write_text(json.dumps(r,indent=2,sort_keys=True)+'\n')
    print(json.dumps(dict(authorities_passed=True,task3d_bindings=len(r['task3d_bindings']))))

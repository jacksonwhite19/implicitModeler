"""Task-4 read-only frozen-authority gate and bounded preservation snapshot."""
from task3e_preflight import audit as prior_audit,ROOT,git
from sdf_cfd_mesher_v4.common.authority import file_hash
import json,datetime
OUT=ROOT/'results/task4'

def audit():
    previous=prior_audit();base=ROOT/'results/task3e';rows=[]
    for name,h in [('report.md','5e86c4aded1ce2b6957d59b0ee5fa44308af757ad98a28dd860e5cd4114ab116'),('AUTHORITY.json','ba28d1d70023321836c5b166b857a564f92dfa50bd99ec31195774f149bc8842')]:
        actual=file_hash(base/name);assert actual==h,(name,actual);rows.append(dict(path=(base/name).relative_to(ROOT).as_posix(),sha256=h))
    authority=json.loads((base/'AUTHORITY.json').read_text())
    for group in ('source_sha256','evidence_sha256'):
        for p,h in authority[group].items():
            actual=file_hash(ROOT/p);assert actual==h,(p,actual);rows.append(dict(path=p,sha256=h))
    return dict(previous=previous,task3e_bindings=rows)

if __name__=='__main__':
    assert git('branch','--show-current')=='feature/sdf-cfd-mesher-v4-advancing-front'
    assert git('rev-parse','HEAD')=='c4bd25452cbe200455163ae3fcf2e34371f2b1e1'
    assert not git('diff','--cached','--name-only')
    result=audit();result.update(starting_head=git('rev-parse','HEAD'),starting_status=git('status','--porcelain'),started_utc=datetime.datetime.now(datetime.timezone.utc).isoformat())
    OUT.mkdir(exist_ok=True);(OUT/'.gitignore').write_text('*.log\n')
    (OUT/'PREFLIGHT.json').write_text(json.dumps(result,indent=2,sort_keys=True)+'\n')
    print('Task-4 authorities verified:',len(result['task3e_bindings']))

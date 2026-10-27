"""Read-only prior authorities and isolated Task-4D outputs."""
from task4c_common import *
C=ROOT/'results/task4c'
OUT=ROOT/'results/task4d'

def save(name,value):
    OUT.mkdir(exist_ok=True)
    (OUT/name).write_bytes((json.dumps(plain(value),sort_keys=True,indent=2)+'\n').encode())

def verify_prior():
    expected={B:('ec5686586f9eb5e2e65f55e72338dae3276b82e8f9d70a4e70efc5f610c7bba2','8fe4af86b569705ac41399dcdbd0a02ddb7ff41b62a75fc254cd7998bd754aac'),C:('069e61fbdf37060d4604333886275dddf6a00bdc6fe8f66f4cc384609cda70c9','db8cc487ed5317fbf3ad2acc20f02b5f101ab2e3c4cc18085626b490842a176a')}
    rows=[]
    for base,hashes in expected.items():
        for name,h in zip(('report.md','AUTHORITY.json'),hashes):
            assert file_hash(base/name)==h,(base,name)
            rows.append(dict(path=str((base/name).relative_to(ROOT)),sha256=h))
        a=read(base/'AUTHORITY.json')
        for group in ('source_sha256','evidence_sha256'):
            for p,h in a[group].items():
                assert file_hash(ROOT/p)==h,p
                rows.append(dict(path=p,sha256=h,group=group))
    return rows

def bounded_v3():
    p=read(C/'PREFLIGHT.json')['prior']
    while 'previous' in p:p=p['previous']
    rows=p['prior_frozen_audit']['rows']
    result={}
    for row in rows:
        if 'sdf_cfd_mesher_v3/' in row['path'].replace('\\','/'):
            path=Path(row['path']);path=path if path.is_absolute() else REPO/path
            actual=file_hash(path);assert actual==row['actual'],str(path)
            result[str(path)]=actual
    return result

def preflight():
    assert git('rev-parse','HEAD')=='ceae99f46a5d64dcd764bbc773f412b32a6c91c3'
    assert git('branch','--show-current')=='feature/sdf-cfd-mesher-v4-advancing-front'
    assert not git('diff','--cached','--name-only')
    started=datetime.datetime.now(datetime.timezone.utc).isoformat()
    rows=verify_prior();v3=bounded_v3()
    # Bind every sealed Task-4 result without reading/copying mesh arrays.
    prior=read(BASE/'AUTHORITY.json')
    for p,h in prior['evidence_sha256'].items():assert file_hash(ROOT/p)==h,p
    save('PREFLIGHT.json',dict(started_utc=started,head=git('rev-parse','HEAD'),status=git('status','--porcelain'),authorities=rows,V3=v3,Build_A_checkpoint=file_hash(BASE/'build_A/CURRENT.json'),A2_checkpoint=file_hash(BASE/'replay_A2/build_A/CURRENT.json')))
    print('TASK4D AUTHORITIES VERIFIED',len(rows),len(v3),flush=True)

if __name__=='__main__':preflight()

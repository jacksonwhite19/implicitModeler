"""Evidence retention and final closure; never modifies prior authorities."""
import hashlib
import subprocess
import capsule_sa_stabilization as c
from capsule_family_seal import allocation

def prior_unchanged():
    old=c.h.read(c.s.ROOT/'RESULT.json');c.h.verify_record(old)
    for rel,d in old['evidence'].items():assert c.h.file_hash(c.s.ROOT/rel)==d,rel
    for rel,d in old['implementation'].items():assert c.h.file_hash(c.s.f.prior.control.V4/rel)==d,rel
    retention=c.h.read(c.s.ROOT/'RETENTION_AUTHORITY.json')
    for name in c.NAMES:
        for rel,d in retention['retained_files'].items():
            if name in rel:assert c.h.file_hash(c.s.ROOT/rel)==d,rel
    return old['identity']

def retention():
    prior_unchanged();keep={};remove=[]
    for name in c.NAMES:
        c.audit(name)
        assert (c.ROOT/name/'continuation.exit').read_text().strip()=='0'
        m=c.h.read(c.ROOT/name/'MEASUREMENTS.json');c.h.verify_record(m)
        assert c.h.file_hash(c.ROOT/name/'HISTORY.json')==m['history']
        for rank in range(6):
            proc=c.ROOT/name/'pilot'/f'processor{rank}'
            for d in proc.iterdir():
                if not d.is_dir() or not d.name.isdecimal():continue
                step=int(d.name)
                assert 500<=step<=1000
                if step in (500,600,700,800,900,1000):
                    for f in d.rglob('*'):
                        if f.is_file():keep[str(f.relative_to(c.ROOT))]=c.h.file_hash(f)
                else:remove.append(dict(path=str(d.resolve()),**allocation(d)))
    record=c.h.seal(dict(schema='ExactSAContinuationRetention/1',keep=keep,remove=remove,precleanup=allocation(c.ROOT),free_before=c.shutil.disk_usage(c.ROOT).free))
    c.h.durable_json(c.ROOT/'RETENTION.json',record)

def seal():
    prior_unchanged()
    retention=c.h.read(c.ROOT/'RETENTION.json');c.h.verify_record(retention)
    for rel,d in retention['keep'].items():assert c.h.file_hash(c.ROOT/rel)==d,rel
    for name in c.NAMES:c.audit(name)
    original=c.h.read(c.ROOT/'UNRELATED_AUDIT.json')
    status=subprocess.check_output(['git','status','--porcelain'],text=True).splitlines()
    assert [x for x in status if 'capsule_sa_stabilization' not in x]==original['status']
    diff=subprocess.check_output(['git','diff','--binary'],stderr=subprocess.DEVNULL)
    assert hashlib.sha256(diff).hexdigest()==original['tracked_diff_sha256']
    storage=dict(final=allocation(c.ROOT),free_ending=c.shutil.disk_usage(c.ROOT).free,reclaimed_physical=sum(v['physical_bytes'] for v in retention['remove'] if not c.Path(v['path']).exists()),reclaimed_logical=sum(v['logical_bytes'] for v in retention['remove'] if not c.Path(v['path']).exists()))
    c.h.durable_json(c.ROOT/'STORAGE_FINAL.json',storage)
    evidence={str(f.relative_to(c.ROOT)):c.h.file_hash(f) for f in c.ROOT.rglob('*') if f.is_file() and 'pilot' not in f.relative_to(c.ROOT).parts and f.name!='RESULT.json'}
    implementation={str(f.relative_to(c.s.f.prior.control.V4)):c.h.file_hash(f) for parent in ('tools','tests') for f in (c.s.f.prior.control.V4/parent).glob('*capsule_sa_stabilization*.py')}
    decision=c.h.read(c.ROOT/'DECISION.json');c.h.verify_record(decision)
    result=c.h.seal(dict(schema='BoundedSteadySAContinuationResult/1',starting_head=c.HEAD,classification=decision['classification'],decision=decision['identity'],prior=prior_unchanged(),evidence=evidence,implementation=implementation,qualifying=False,production_authority_replaced=False,Run_A=False,Run_A2=False,mesh_changes=0,prism_changes=0,unrelated_count=155))
    c.h.durable_json(c.ROOT/'RESULT.json',result);print(result['identity'])

if __name__=='__main__':
    import sys
    dict(retention=retention,seal=seal)[sys.argv[1]]()

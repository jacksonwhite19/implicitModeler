"""Bounded Task-4G qualification inputs; never invoke a shell builder."""
from task4f_common import *
OUT=ROOT/'results/task4g'
def save(name,data):
    p=OUT/name;p.parent.mkdir(parents=True,exist_ok=True)
    p.write_bytes((json.dumps(plain(data),sort_keys=True,indent=2)+'\n').encode())
def elapsed():
    return (datetime.datetime.now(datetime.timezone.utc)-datetime.datetime.fromisoformat(read(OUT/'CLOCK.json')['task_start'])).total_seconds()
def require_time():
    if elapsed()>=3300:raise RuntimeError('Task4GScientificCutoff')
def metric_identity(result):
    """Bind every field and every array bit without float-to-JSON expansion."""
    def bind(x):
        if isinstance(x,np.ndarray):return dict(array_sha256=array_hash(x),dtype=x.dtype.str,shape=list(x.shape))
        if isinstance(x,dict):return {k:bind(v) for k,v in x.items()}
        if isinstance(x,(tuple,list)):return [bind(v) for v in x]
        return x
    return scientific_hash(bind(result))
def preflight():
    save('CLOCK.json',dict(task_start='2026-09-17T18:09:06.631292+00:00',scientific_cutoff='2026-09-17T19:04:06.631292+00:00',deadline='2026-09-17T19:09:06.631292+00:00'))
    expected={'results/task4f/report.md':'a83c57c584695acb5ad26ac88efd89bf4093e21c4360f5b3a4a7caf361b87dcf','results/task4f/AUTHORITY.json':'bdfdd32f5f2d8340c63b2426f241dbf2f066352874aa5ceb5da6bb91baf8760a'}
    for p,h in expected.items():assert file_hash(ROOT/p)==h,p
    a=read(ROOT/'results/task4f/AUTHORITY.json')
    for group in ('source_sha256','evidence_sha256'):
        for p,h in a[group].items():assert file_hash(ROOT/p)==h,p
    v3=read(ROOT/'results/task4f/PREFLIGHT.json')['V3']
    for p,h in v3.items():assert file_hash(Path(p))==h,p
    head=git('rev-parse','HEAD');assert head=='470996b6c24b9807418c2bb3e25997a25a92a740'
    save('PREFLIGHT.json',dict(head=head,branch=git('branch','--show-current'),status=git('status','--porcelain'),authorities=expected,V3=v3,sealed_evidence=a['evidence_sha256'],source_before=a['source_sha256']))
    print('PREFLIGHT_OK',elapsed(),flush=True)
if __name__=='__main__':preflight()

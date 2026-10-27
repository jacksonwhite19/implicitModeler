"""Bounded exact-geometry study; preserved inputs only."""
from task4f_common import *
OUT=ROOT/'results/task4h'
def save(name,data):
    p=OUT/name;p.parent.mkdir(parents=True,exist_ok=True)
    p.write_bytes((json.dumps(plain(data),sort_keys=True,indent=2)+'\n').encode())
def elapsed():
    return (datetime.datetime.now(datetime.timezone.utc)-datetime.datetime.fromisoformat(read(OUT/'CLOCK.json')['task_start'])).total_seconds()
def require_time():
    if elapsed()>=3300:raise RuntimeError('Task4HScientificCutoff')
def preflight():
    save('CLOCK.json',dict(task_start='2026-09-17T20:20:00+00:00',start_note='Conservative start rounded earlier than the first retained clock sample (20:22:04 UTC).',scientific_cutoff='2026-09-17T21:15:00+00:00',deadline='2026-09-17T21:20:00+00:00'))
    expected={'results/task4g/report.md':'2f011355e72f15e4cfd6a25357b9fc8793f197f8e21c690073ee40fb47dcc33f','results/task4g/AUTHORITY.json':'e2cb8b5d9fd9dfb6a1ad25c7d32359824cc13b890a218e96fa23064bca6e63ee'}
    for p,h in expected.items():assert file_hash(ROOT/p)==h,p
    a=read(ROOT/'results/task4g/AUTHORITY.json')
    for group in ('source_sha256','evidence_sha256'):
        for p,h in a[group].items():assert file_hash(ROOT/p)==h,p
    v3=read(ROOT/'results/task4g/PREFLIGHT.json')['V3']
    for p,h in v3.items():assert file_hash(Path(p))==h,p
    assert git('rev-parse','HEAD')=='cf27a74f537b66b6849f240687bb5d279b674dfc'
    save('PREFLIGHT.json',dict(head=git('rev-parse','HEAD'),branch=git('branch','--show-current'),status=git('status','--porcelain'),authorities=expected,V3=v3,sealed_evidence=a['evidence_sha256'],source_before=a['source_sha256']))
    print('PREFLIGHT_OK',elapsed(),flush=True)
if __name__=='__main__':preflight()

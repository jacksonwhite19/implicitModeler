"""Latest sealed source lineage, with two explicit output-routing updates."""
from task4l_common import *
REPAIRS={'tools/task4_initial_scan.py','tools/task4_verify.py'}

def audit(authorized_overrides=None):
    overrides=authorized_overrides
    if overrides is None:overrides=read(OUT/'RUNNER_CONFIGURATION.json')['source_hashes']
    sources={};authorities={}
    for task in ('task4','task4b','task4c','task4d','task4e','task4f','task4g','task4h','task4i','task4j','task4k'):
        p=ROOT/f'results/{task}/AUTHORITY.json'
        frozen=subprocess.check_output(['git','show','HEAD:'+p.relative_to(REPO).as_posix()],cwd=REPO)
        assert file_hash(p)==hashlib.sha256(frozen).hexdigest(),str(p)
        a=read(p);sources.update(a.get('source_sha256',{}));sources.update(a.get('source',{}));authorities[task]=file_hash(p)
    records=[]
    for p,h in sources.items():
        actual=file_hash(ROOT/p)
        if actual!=h:
            assert p in REPAIRS and actual==overrides[p],('UnqualifiedSourceDrift',p)
            old=subprocess.check_output(['git','show','HEAD:'+(ROOT/p).relative_to(REPO).as_posix()],cwd=REPO)
            assert hashlib.sha256(old).hexdigest()==h,('UnexplainedPreviousSource',p)
        records.append(dict(path=p,published_sha256=h,current_sha256=actual))
    v3=read(OUT/'PREFLIGHT_INITIAL.json')['V3']
    for p,h in v3.items():assert file_hash(Path(p))==h,p
    return dict(authorities=authorities,source_lineage=records,V3_verified=len(v3),policy='Latest qualified source binding wins; only the two explicit Task-4L routing changes may differ.')

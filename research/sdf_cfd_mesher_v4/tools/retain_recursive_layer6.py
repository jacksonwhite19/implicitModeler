"""Successful-only exact-duplicate replay reclamation. Never retire ancestry."""
from qualify_recursive_layer6 import *

def check_duplicate(path,canonical,replay,bound,expected):
    path=Path(path).resolve();canonical=Path(canonical).resolve();replay=Path(replay).resolve()
    if not path.is_relative_to(replay) or str(path) in bound:raise ValueError('ProtectedReplayPath')
    if canonical.is_relative_to(replay) or path==canonical:raise ValueError('CanonicalMustSurviveReplayCleanup')
    if file_hash(path)!=expected or file_hash(canonical)!=expected:raise ValueError('NotExactRetainedDuplicate')

def recover(rows,replay,bound):
    restored=[]
    for r in rows:
        p=Path(r['path']).resolve();c=Path(r['canonical']).resolve()
        if not p.is_relative_to(replay.resolve()) or str(p) in bound or c.is_relative_to(replay.resolve()):
            raise ValueError('UnsafeRecoveryPath')
        if file_hash(c)!=r['sha256']:raise ValueError('CorruptRecoveryCanonical')
        if not p.exists():shutil.copyfile(c,p);restored.append(str(p))
        check_duplicate(p,c,replay,bound,r['sha256'])
    return restored

def inventory():
    records=[metadata(p) for p in OUT.rglob('*') if p.is_file()]
    unique={physical_key(r):r for r in records}
    return dict(files=records,physical=sum(r['cluster_bytes'] for r in unique.values()),
                logical=sum(r['bytes'] for r in records),free=shutil.disk_usage(V4).free)

def verify():
    assert read(OUT/'DEEP.json')['passed'] and read(OUT/'REPLAY.json')['passed']
    ref=read(OUT/'REOPEN.json')['reference']
    with CommittedState(ref) as state:
        a,_=inputs(state);p=plan_transition(state,a,7)
        assert state.layer==6 and p['increment_hex']=='0x1.bd6749eb1b62ap-12'
        state.check()
    for source in ('PREFLIGHT.json','PRIOR_AUTHORITY_VERIFICATION.json'):
        r=read(OUT/source)
        for p,h in r.get('protected',r.get('files',{})).items():assert file_hash(p)==h,p

def run():
    if (OUT/'RETENTION.json').exists():
        verify();assert read(OUT/'RETENTION.json')['passed'];return
    tick=measured();verify();before=inventory();replay=(OUT/'replay').resolve()
    bound=set(read(OUT/'CANDIDATE_PLAN.json')['references'])
    if (OUT/'CLEANUP_MANIFEST.json').exists():
        previous=read(OUT/'CLEANUP_MANIFEST.json')
        restored=recover(previous['rows'],replay,bound)
        durable_json(OUT/'CLEANUP_RECOVERY.json',dict(previous=previous,restored=restored))
        before=inventory()
    rows=[]
    for p in sorted(replay.rglob('*')):
        if not p.is_file():continue
        p=p.resolve();assert p.is_relative_to(replay) and str(p) not in bound
        canonical=(OUT/p.relative_to(replay)).resolve()
        if not canonical.is_file():continue
        h=file_hash(p)
        if h!=file_hash(canonical):continue
        a=metadata(p);b=metadata(canonical)
        assert physical_key(a)!=physical_key(b)
        rows.append(dict(path=str(p),canonical=str(canonical),sha256=h,metadata=a,canonical_metadata=b,
                         classification='E',reason='Exact duplicate in unpublished successful replay; canonical retained'))
    manifest=dict(policy='V4RollingSuccessfulQualificationRetention/1',scope='Only byte-identical unpublished replay bulk; no checkpoint retirement',
                  rows=rows,protected_root=read(OUT/'PUBLICATION.json')['root'],deep=file_hash(OUT/'DEEP.json'),replay=file_hash(OUT/'REPLAY.json'))
    durable_json(OUT/'CLEANUP_MANIFEST.json',manifest)
    removed=[]
    try:
        for r in rows:
            p=Path(r['path']);canonical=Path(r['canonical'])
            check_duplicate(p,canonical,replay,bound,r['sha256'])
            assert metadata(p)['links']==1
            p.unlink();removed.append(r)
        verify()
    except BaseException:
        recover(removed,replay,bound)
        raise
    after=inventory()
    result=dict(passed=True,before=before,after=after,removed_paths=len(removed),logical_removed=sum(r['metadata']['bytes'] for r in removed),
                physical_reclaimed=sum(r['metadata']['cluster_bytes'] for r in removed),
                manifest_sha256=file_hash(OUT/'CLEANUP_MANIFEST.json'),metrics=metrics(*tick),
                preserved='All current and historical checkpoint dependencies; unique replay timings and root map; all main-run scientific evidence',
                unresolved='No broader checkpoint retirement or successful diagnostic regeneration cleanup qualified here')
    durable_json(OUT/'RETENTION.json',result)
    print(json.dumps({k:v for k,v in result.items() if k not in ('before','after')},indent=2),flush=True)
if __name__=='__main__':run()

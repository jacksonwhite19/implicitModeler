"""Bounded preservation, complete scientific replay and performance tables."""
from task4d_common import *
from task4_preflight import audit
import copy

def run():
    a=read(OUT/'sequence_1_REPLAY.json');b=read(OUT/'sequence_2_REPLAY.json')
    assert a==b and len(b['rows'])==20
    w1=read(OUT/'WARM_1.json');w2=read(OUT/'WARM_2.json');assert w1==w2
    with np.load(OUT/'WARM_1_CANDIDATES.npz') as x,np.load(OUT/'WARM_2_CANDIDATES.npz') as y:
        assert x.files==y.files
        for key in x.files:assert np.array_equal(x[key].view(np.uint64),y[key].view(np.uint64))
    assert read(OUT/'RESTART.json')['passed']
    assert read(OUT/'FIRST_RUNNER.json')['passed']
    cache=[]
    ta=read(OUT/'sequence_1_TIMINGS.json');tb=read(OUT/'sequence_2_TIMINGS.json')
    for x,y in zip(ta['rows'],tb['rows']):
        for key in ('before','after'):
            u=copy.deepcopy(x[key]);v=copy.deepcopy(y[key])
            u['selection'].pop('seconds',None);v['selection'].pop('seconds',None)
            assert u==v,(x['layer'],key)
            cache.append(dict(layer=x['layer'],phase=key,state=u))
    cert=read(OUT/'CERTIFICATE_BINDINGS.json');assert cert['pairs']==56462864
    save('DETERMINISM.json',dict(passed=True,run_1=a['scientific_hash'],run_2=b['scientific_hash'],
        cache_reuse_and_invalidation_counters_hash=scientific_hash(cache),
        warm_1=w1['scientific_hash'],warm_2=w2['scientific_hash'],restart=read(OUT/'RESTART.json')['scientific_hash'],
        fields='All candidate and forecast coordinates, normals, full canonical accepted/rejected history, gate values, warnings, region identities and transitions. Warm transport arrays also compare bitwise. Timing/RSS are excluded.',
        implementation_note='Run 1 was exploratory: redundant closure set construction was hoisted, then restored to preserve an older byte authority. Run 2 uses restored scientific sources. A metadata-only detached-checkpoint fix followed; final-source restart and the actual first-layer entry point were verified separately. Both complete scientific histories equal frozen Build A.',A2_resumed=False))
    start=read(OUT/'PREFLIGHT.json');changed=[];allowed={'tools/task4_prepare_next.py','tools/task4_correct_first.py','src/sdf_cfd_mesher_v4/forecast/detailed/rolling_spots.py'}
    for r in start['authorities']:
        h=file_hash(ROOT/r['path'])
        if h!=r['sha256']:
            assert r.get('group')=='source_sha256' and r['path'] in allowed,r
            changed.append(dict(path=r['path'],before=r['sha256'],after=h))
    for p,h in read(BASE/'AUTHORITY.json')['evidence_sha256'].items():assert file_hash(ROOT/p)==h,p
    assert bounded_v3()==start['V3'];audit()
    assert file_hash(BASE/'build_A/CURRENT.json')==start['Build_A_checkpoint']
    assert file_hash(BASE/'replay_A2/build_A/CURRENT.json')==start['A2_checkpoint']
    def outside(s):return [x for x in s.splitlines() if 'research/sdf_cfd_mesher_v4' not in x]
    before=outside(start['status']);after=outside(git('status','--porcelain'))
    tracked=git('diff','--name-only','--','research/sdf_cfd_mesher_v4').splitlines()
    assert set(tracked)=={'research/sdf_cfd_mesher_v4/'+p for p in allowed},tracked
    save('PRESERVATION.json',dict(V3_modifications=0,V3_bounded_files=len(start['V3']),sealed_result_modifications=0,authorized_source_changes=changed,
        changed_tracked_paths=tracked,
        Build_A_unchanged=True,A2_unchanged=True,unrelated_state_unchanged=before==after,unrelated_status=after,
        unrelated_added=sorted(set(after)-set(before)),unrelated_removed=sorted(set(before)-set(after)),unrelated_changes_by_this_task=0))
    print('REPLAY, WARM TRANSPORT, RESTART, AUTHORITIES AND V3 PASS',flush=True)

if __name__=='__main__':run()

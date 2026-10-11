"""Bounded production prepare calls; read-only frozen checkpoint facades.

There is no commit call and no geometry build across transactions. Each case
starts independently from a preserved checkpoint. Only its new next candidate
and compact decisions are retained as comparison evidence.
"""
from task4c_common import *
from types import SimpleNamespace
import task4_prepare_next as runner
from sdf_cfd_mesher_v4.advancing_front.rolling_state import RollingState
from sdf_cfd_mesher_v4.advancing_front.rolling_cache import load_candidate,save_candidate
import psutil

def run(layer):
    path=OUT/f'runner_{layer:02}';path.mkdir(exist_ok=True);m=json.loads((BASE/f'build_A/LAYER_{layer-1:02}.json').read_text())
    class ReadOnlyStore:
        def __init__(self,*_):self.path=BASE/'build_A'
        def restart(self):
            x=np.load(self.path/f'FRONT_{layer-1:02}.npy');assert array_hash(x)==m['coordinate_hash']
            return RollingState(layer-1,x,m['topology_hash'],m['state_hash'],m['prism_hash'],m['side_hash'],m['certificate_hash'],m['forecast_key'])
        def commit(self,*_):raise AssertionError('StudyMustNotCommit')
    runner.OUT=path;runner.CommitStore=ReadOnlyStore
    runner.load_candidate=lambda p,b:load_candidate(BASE/p.name,BASE/b.name)
    runner.save_candidate=lambda p,c,x,n,b:save_candidate(p,c,x,n,BASE/b.name)
    runner.read=lambda p:read(BASE/p.name)
    started=time.perf_counter();assert runner.prepare(layer)
    # Persisted proposal is compared then removed from study storage only by
    # explicit caller policy. It is not published into any commit store.
    candidate,forecast,normals=load_candidate(path/f'LAYER{layer}_CANDIDATE.npz',BASE/'INITIAL_FORECAST.npz')
    old,oldf,oldn=load_candidate(BASE/f'LAYER{layer}_CANDIDATE.npz',BASE/'INITIAL_FORECAST.npz')
    assert np.array_equal(candidate.view(np.uint64),old.view(np.uint64))
    assert np.array_equal(forecast.view(np.uint64),oldf.view(np.uint64))
    assert np.array_equal(normals.view(np.uint64),oldn.view(np.uint64))
    new_history=read(path/f'LAYER{layer}_SEARCH.json');old_history=read(BASE/f'LAYER{layer}_SEARCH.json')
    assert scientific_hash(new_history)==scientific_hash(old_history),('DecisionHistoryMismatch',layer)
    assert scientific_hash(read(path/f'LAYER{layer}_GATE.json'))==scientific_hash(read(BASE/f'LAYER{layer}_GATE.json'))
    assert scientific_hash(read(path/f'LAYER{layer}_WARNINGS.json'))==scientific_hash(read(BASE/f'LAYER{layer}_WARNINGS.json'))
    result=dict(layer=layer,candidate_hash=array_hash(candidate),forecast_hash=array_hash(forecast),normal_hash=array_hash(normals),
        decision_hash=scientific_hash(new_history),all_decisions_exact=True,all_coordinates_exact=True,all_gate_values_exact=True,
        active_atlas=read(path/f'LAYER{layer}_ATLAS.json'),authoritative_commit_called=False,
        production_prepare_source=file_hash(ROOT/'tools/task4_prepare_next.py'))
    save(f'RUNNER_{layer:02}.json',result)
    save(f'RUNNER_{layer:02}_TIMINGS.json',dict(seconds=time.perf_counter()-started,peak_rss=psutil.Process().memory_info().peak_wset,
         current=json.loads((path/f'LAYER{layer}_PREPARE_TIMINGS.json').read_text()),
         frozen=json.loads((BASE/f'LAYER{layer}_PREPARE_TIMINGS.json').read_text())))
    print('RUNNER MATCH',layer,result['decision_hash'],flush=True)

if __name__=='__main__':run(int(sys.argv[1]))

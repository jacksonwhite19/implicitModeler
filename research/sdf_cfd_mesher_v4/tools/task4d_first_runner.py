"""Exercise the real first-layer entry point with a read-only layer-zero store."""
from task4d_common import *
from types import SimpleNamespace
import task4_correct_first as runner
from sdf_cfd_mesher_v4.advancing_front.rolling_cache import load_candidate,save_candidate
from sdf_cfd_mesher_v4.advancing_front.persistent_planning import PersistentPlanner

def run():
    started=time.perf_counter();path=OUT/'first_runner';path.mkdir(exist_ok=True)
    class ReadOnlyWall:
        def __init__(self,*_):pass
        def initialize(self):return SimpleNamespace(layer=0)
        def commit(self,*_):raise AssertionError('NoProductionCommit')
    class ArrayIO:
        def __getattr__(self,key):return getattr(np,key)
        def load(self,p,*args,**kwargs):
            p=Path(p)
            return np.load(BASE/p.name if p.name=='INITIAL_FORECAST.npz' else p,*args,**kwargs)
    runner.OUT=path;runner.CommitStore=ReadOnlyWall;runner.np=ArrayIO()
    runner.read=lambda p:read(BASE/p.name)
    runner.save_candidate=lambda p,c,x,n,b:save_candidate(p,c,x,n,BASE/b.name)
    runner.correct_first()
    hashes={}
    for name in ('CORRECTION_HISTORY.json','LAYER1_GATE.json'):
        actual=scientific_hash(read(path/name));assert actual==scientific_hash(read(BASE/name));hashes[name]=actual
    own=load_candidate(path/'LAYER1_CANDIDATE.npz',BASE/'INITIAL_FORECAST.npz')
    old=load_candidate(BASE/'LAYER1_CANDIDATE.npz',BASE/'INITIAL_FORECAST.npz')
    assert all(np.array_equal(a.view(np.uint64),b.view(np.uint64)) for a,b in zip(own,old))
    planner=PersistentPlanner();planner.load(path/'LAYER1_PLANNER.npz',1,array_hash(own[1]))
    with np.load(OUT/'sequence_2/LAYER1_PLANNER.npz') as z:expected=json.loads(str(z['metadata']))['checkpoint']
    assert planner.checkpoint(array_hash(own[1]))==expected
    save('FIRST_RUNNER.json',dict(passed=True,hashes=hashes,coordinate_hashes=[array_hash(x) for x in own],checkpoint=expected,production_commit_called=False))
    save('FIRST_RUNNER_TIMINGS.json',dict(seconds=time.perf_counter()-started))
    print('ACTUAL FIRST RUNNER AND CHECKPOINT MATCH',flush=True)

if __name__=='__main__':run()

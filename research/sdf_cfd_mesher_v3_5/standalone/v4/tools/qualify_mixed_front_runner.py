"""Bounded native runner checks. No full SF2 or core generation."""
import sys,time,traceback
import generalize_local_remesh as g
from mixed_front_fixture_provider import Provider
from task4l_run import run_mixed_front
from sdf_cfd_mesher_v4.advancing_front.mixed_front import scientific_hash
from sdf_cfd_mesher_v4.advancing_front.mixed_front_journal import MixedFrontJournal

def run(name,attempt=None):
    folder=g.s.ROOT/'results/purpose_built_mixed_front_runner'/name
    if attempt:folder=folder/attempt
    provider=Provider(next(s for s in g.fixtures.definitions() if s['name']==name),folder)
    tick=time.perf_counter()
    try:
        initial=provider.initialize()
        print('INITIAL',scientific_hash(initial),flush=True)
        operations=run_mixed_front(provider,initial,folder/'checkpoints',2)
        final=MixedFrontJournal(folder/'checkpoints').restart()
        g.save(folder/'RESULT.json',dict(operations=operations,initial=scientific_hash(initial),final=scientific_hash(final),
                                       wall_identity=initial['wall']==final['wall'],records=provider.records,
                                       cell_count=len(final['cells']),active_columns=len(final['prism_front']),outer_faces=len(final['outer'])))
        print('COMPLETE',operations,flush=True)
    except BaseException:
        g.save(folder/'FAILURE.json',dict(traceback=traceback.format_exc(),records=provider.records));raise
    finally:
        g.save(folder/'RUNTIME.json',dict(total=time.perf_counter()-tick,stages=provider.times,peak_rss=g.q.psutil.Process().memory_info().peak_wset))

if __name__=='__main__':run(sys.argv[1],sys.argv[2] if len(sys.argv)>2 else None)

"""Fresh first-commit lifetime reproduction, through the real native runner."""
import sys,json,gc
from pathlib import Path
from native_memory_observer import Observer
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'results/native_runner_memory_profile'
def run():
    name=sys.argv[1];path=OUT/name
    observer=Observer(path/'profile');observer.install()
    try:
        from native_prism_production import Provider
        from sdf_cfd_mesher_v4.advancing_front.mixed_front_runner import MixedFrontRunner
        provider=Provider(path/'build');runner=MixedFrontRunner(path/'build/checkpoints',provider)
        initial=provider.initial();observer.mark('after_initial',initial);runner.initialize(initial);del initial
        observer.layer=1;observer.mark('before_first_commit');result=runner.step();observer.mark('after_first_commit')
        from sdf_cfd_mesher_v4.forecast.detailed.exact_cache import LOCAL_EXACT
        observer.mark('exact_cache_retained',vars(LOCAL_EXACT))
        provider.sdf.clear_memo();observer.mark('after_sdf_clear')
        gc.collect();observer.mark('after_cycle_collection')
        (path/'RESULT.json').write_text(json.dumps(result,indent=2)+'\n')
    finally:observer.finish()
if __name__=='__main__':run()

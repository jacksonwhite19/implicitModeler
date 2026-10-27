"""Read the qualified checkpoint instead of re-optimizing its coordinates."""
import time,traceback
from audit_production_migration import OUT
import residual_layer5_study as study
from sdf_cfd_mesher_v4.advancing_front.rolling_state import atomic_json

def stored_seed():
    r=study.r
    case=r.c.Case(r.q.definitions(r.q.load_schedule().targets)[-1])
    folder=r.g.OUT/case.name;record=r.load(folder/'result.json')
    high=r.np.load(folder/'candidate.npy');targets=r.np.load(folder/'targets.npy')
    assert r.q.array_hash(high)==record['coordinate_hash'] and r.q.array_hash(targets)==record['target_hash']
    gate,metric,seconds=r.w.checked(case.x,case.f,case.top,case.verify,case.prefix,high,case.previous)
    assert gate['passed'] and r.q.identity(gate)==r.q.identity(record['accepted_gate'])
    history=[r.np.full(len(high),v) for v in case.s.targets[:3]]+[targets]
    return case,case.prefix+[high],history,metric['cells'],dict(coordinate_hash=r.q.array_hash(high),target_hash=r.q.array_hash(targets),gate=gate,checkpoint_migration=True)

def run():
    tick=time.perf_counter();old=study.r.seed
    try:
        study.r.seed=stored_seed
        case,prefix,history,previous,seed,targets,high,record,timing=study.reproduce()
        expected=study.r.load(study.r.g.ROOT/'results/residual_layer5_failure/REPRODUCER_IDENTITY.json')
        actual=study.r.q.array_hash(high)
        atomic_json(OUT/'GENERAL_SDF_MIGRATION_PROBE.json',dict(seed=seed,layer5_hash=actual,
            reference_hash=expected['coordinate_hash'],equal=actual==expected['coordinate_hash'],native_run=False))
        print('GENERAL_REPRODUCED',actual,actual==expected['coordinate_hash'],flush=True)
    except BaseException:
        atomic_json(OUT/'GENERAL_SDF_MIGRATION_FAILURE.json',dict(traceback=traceback.format_exc()));raise
    finally:
        study.r.seed=old
        atomic_json(OUT/'GENERAL_PROBE_RUNTIME.json',dict(seconds=time.perf_counter()-tick,peak_rss=study.r.q.psutil.Process().memory_info().peak_wset))

if __name__=='__main__':run()

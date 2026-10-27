"""Verify preserved SF2 proof payloads without recomputing their mathematics."""
import json,time,subprocess,traceback
from pathlib import Path
import psutil
from sdf_cfd_mesher_v4.geometry.front import load_sf2_wall,build_topology
from sdf_cfd_mesher_v4.forecast.schedule import load_schedule
from sdf_cfd_mesher_v4.sdf.native import NativeSdf
from sdf_cfd_mesher_v4.advancing_front.rolling_state import CommitStore,atomic_json
from sdf_cfd_mesher_v4.advancing_front.legacy_mixed_migration import migrate_inventory
from sdf_cfd_mesher_v4.common.authority import file_hash
from task3c_io import read

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'results/mixed_front_production_migration'

def run():
    OUT.mkdir(parents=True,exist_ok=True);tick=time.perf_counter()
    try:
        front=load_sf2_wall();top=build_topology(front.triangles,len(front.coordinates))
        store=CommitStore(ROOT/'results/task4l/fresh/build_A',front,top,load_schedule())
        result=migrate_inventory(store,20,NativeSdf().source_identity,read_scientific=read)
        atomic_json(OUT/'SF2_MIGRATION_INVENTORY.json',result)
        atomic_json(OUT/'SF2_CHECKPOINT_COHORT.json',dict(checkpoints=[dict(layer=k,
            geometry=result['layers'][k-1],quality_receipts=k,
            proof_count=sum(r['reused_proofs'] for r in result['certificate'][:k])) for k in (1,5,11,12,20)],
            native_checkpoint_conversion=False,full_build=False))
        print('VERIFIED',result['coverage'],flush=True)
    except BaseException:
        atomic_json(OUT/'SF2_MIGRATION_FAILURE.json',dict(traceback=traceback.format_exc()));raise
    finally:
        atomic_json(OUT/'SF2_MIGRATION_RUNTIME.json',dict(seconds=time.perf_counter()-tick,peak_rss=psutil.Process().memory_info().peak_wset))

if __name__=='__main__':run()

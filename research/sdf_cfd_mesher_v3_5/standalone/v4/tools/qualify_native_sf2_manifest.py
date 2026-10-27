"""Qualified legacy proof import followed by native-only coverage audit."""
import json,time,gc
from pathlib import Path
import psutil
from sdf_cfd_mesher_v4.geometry.front import load_sf2_wall,build_topology
from sdf_cfd_mesher_v4.forecast.schedule import load_schedule
from sdf_cfd_mesher_v4.sdf.native import NativeSdf
from sdf_cfd_mesher_v4.advancing_front.rolling_state import CommitStore,atomic_json
from sdf_cfd_mesher_v4.advancing_front.legacy_mixed_migration import migrate_inventory
from sdf_cfd_mesher_v4.certificate.native_manifest import NativeManifest
from task3c_io import read

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'results/native_sf2_production_plumbing'
SOURCE=ROOT/'results/task4l/fresh'

def run():
    OUT.mkdir(parents=True,exist_ok=True);tick=time.perf_counter()
    front=load_sf2_wall();top=build_topology(front.triangles,len(front.coordinates))
    store=CommitStore(SOURCE/'build_A',front,top,load_schedule())
    inventory=migrate_inventory(store,20,NativeSdf().source_identity,read_scientific=read)
    atomic_json(OUT/'VERIFIED_SOURCE_INVENTORY.json',inventory)
    verified_seconds=time.perf_counter()-tick
    manifest=NativeManifest.import_verified(OUT/'native_manifest',top.edges,
        [(SOURCE/f'certificate_layer_{k:02}',row['ledger']) for k,row in enumerate(inventory['certificate'],1)])
    del inventory,store,front,top;gc.collect()
    imported=time.perf_counter();result=manifest.audit();audited=time.perf_counter()
    assert result['active']==56462864
    atomic_json(OUT/'NATIVE_MANIFEST_AUDIT.json',dict(result,source_verification_seconds=verified_seconds,
        binding_seconds=imported-tick-verified_seconds,audit_seconds=audited-imported,
        peak_rss=psutil.Process().memory_info().peak_wset,resident_rss=psutil.Process().memory_info().rss,
        disk_bytes=sum(p.stat().st_size for p in manifest.path.rglob('*') if p.is_file())))
    print(result,flush=True)

if __name__=='__main__':run()

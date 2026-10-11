"""Instantiate and restart native production state at each requested prefix."""
import gc,json,time,sys
import numpy as np
import psutil
from qualify_native_sf2_manifest import ROOT,OUT,SOURCE
from sdf_cfd_mesher_v4.geometry.front import load_sf2_wall,build_topology
from sdf_cfd_mesher_v4.forecast.schedule import load_schedule
from sdf_cfd_mesher_v4.advancing_front.rolling_state import CommitStore,atomic_json
from sdf_cfd_mesher_v4.advancing_front.native_prism_import import import_prism_checkpoint
from sdf_cfd_mesher_v4.advancing_front.native_checkpoint import NativeCheckpointStore
from sdf_cfd_mesher_v4.advancing_front.native_front_providers import NativeRiskAtlas,NativeForecast
from sdf_cfd_mesher_v4.advancing_front.native_prism_validation import audit_native_prisms
from sdf_cfd_mesher_v4.advancing_front.mixed_front import scientific_hash
from sdf_cfd_mesher_v4.certificate.native_manifest import NativeManifest
from sdf_cfd_mesher_v4.advancing_front.rolling_cache import load_candidate
from sdf_cfd_mesher_v4.common.authority import array_hash

def run(layer):
    tick=time.perf_counter();front=load_sf2_wall();top=build_topology(front.triangles,len(front.coordinates))
    legacy=CommitStore(SOURCE/'build_A',front,top,load_schedule())
    inventory=json.loads((OUT/'VERIFIED_SOURCE_INVENTORY.json').read_text())
    manifest=NativeManifest(OUT/'native_manifest');destination=OUT/f'layer_{layer:02}'
    state=import_prism_checkpoint(legacy,layer,destination,inventory,manifest)
    imported=time.perf_counter();before_release=psutil.Process().memory_info().rss
    del legacy,front,top,inventory;gc.collect();after_release=psutil.Process().memory_info().rss
    atlas=NativeRiskAtlas(state,destination/'objects');forecast=atlas.native_forecast
    native_forecast=forecast.scientific();digest=scientific_hash(state)
    # Reference buffers enter only this after-construction comparison.
    _,reference,normals=load_candidate(SOURCE/f'LAYER{layer}_CANDIDATE.npz',SOURCE/'INITIAL_FORECAST.npz')
    assert np.array_equal(reference.view(np.uint64),forecast.coordinates.view(np.uint64))
    assert np.array_equal(normals.view(np.uint64),forecast.normals.view(np.uint64))
    del reference,normals
    result=audit_native_prisms(state,destination/'objects',manifest)
    partition=atlas.partition();active=len(partition['active_vertices'])
    del state,atlas,forecast;gc.collect()
    restart=NativeCheckpointStore(destination).restart();assert scientific_hash(restart)==digest
    again=NativeRiskAtlas(restart,destination/'objects')
    assert again.native_forecast.scientific()==native_forecast
    atomic_json(OUT/f'LAYER_{layer:02}_PREQUALIFICATION.json',dict(layer=layer,native_state=True,scientific=digest,
        import_seconds=imported-tick,total_seconds=time.perf_counter()-tick,peak_rss=psutil.Process().memory_info().peak_wset,
        before_legacy_release_rss=before_release,after_legacy_release_rss=after_release,
        restart_rss=psutil.Process().memory_info().rss,atlas=again.scientific_hash(),active_vertices=active,
        forecast=native_forecast,audit=result,restart_exact=True,coordinate_equality=True,normal_equality=True))
    print('NATIVE_PREFIX',layer,len(restart['cells']),active,flush=True)

if __name__=='__main__':run(int(sys.argv[1]))

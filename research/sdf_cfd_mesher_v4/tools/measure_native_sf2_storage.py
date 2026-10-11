"""Bounded native manifest lookup and restart resident-memory measurements."""
import gc,json,time,sys
import numpy as np,psutil
from qualify_native_sf2_manifest import OUT
from sdf_cfd_mesher_v4.certificate.native_manifest import NativeManifest
from sdf_cfd_mesher_v4.advancing_front.native_checkpoint import NativeCheckpointStore
from sdf_cfd_mesher_v4.advancing_front.native_front_providers import NativeForecast,NativeRiskAtlas
from sdf_cfd_mesher_v4.advancing_front.rolling_state import atomic_json

def run():
    process=psutil.Process();rows=[]
    def mark(stage):
        gc.collect();rows.append(dict(stage=stage,rss=process.memory_info().rss,peak_rss=process.memory_info().peak_wset))
    mark('empty_process')
    manifest=NativeManifest(OUT/'native_manifest');meta=manifest.metadata();edges=np.load(manifest.path/'source_edges.npy',mmap_mode='r')
    mark('manifest_index')
    count=0;tick=time.perf_counter()
    for block in meta['blocks']:
        value=int(manifest.array(block,'packed')[0]);a,b=value>>32,value&0xffffffff
        def face(i):return (i//len(edges)+1,*map(int,edges[i%len(edges)]))
        for _ in range(5):manifest.lookup(face(a),face(b));count+=1
    seconds=time.perf_counter()-tick;mark('after_100_scientific_lookups')
    state=NativeCheckpointStore(OUT/'layer_20').restart();mark('native_layer20_restart')
    forecast=NativeForecast(state,OUT/'layer_20/objects');mark('native_forecast')
    del forecast;gc.collect()
    atlas=NativeRiskAtlas(state,OUT/'layer_20/objects');mark('native_atlas_and_shared_forecast')
    atomic_json(OUT/'MEMORY_COMPONENTS.json',dict(stages=rows,lookup_count=count,lookup_seconds=seconds,
        mean_lookup_seconds=seconds/count,manifest_index_payload_bytes=(manifest.path/'MANIFEST.json').stat().st_size+edges.nbytes,
        expanded_relation_objects=0,scientific_population=meta['relation_count']))

if __name__=='__main__':run()

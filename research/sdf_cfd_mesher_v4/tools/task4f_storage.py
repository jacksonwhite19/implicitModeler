"""Bounded planner serialization and initialized state setup; no checkpoint publish."""
from task4f_common import *
import task4_prepare_next as prod
from sdf_cfd_mesher_v4.certificate.efficient import Filter
from sdf_cfd_mesher_v4.certificate.production import surface_arrays
import io,psutil

def run():
    require_time();start=time.perf_counter();rows=[]
    def timed(name,fn):
        t=time.perf_counter();c=time.process_time();out=fn();rows.append(dict(stage=name,seconds=time.perf_counter()-t,cpu_seconds=time.process_time()-c));return out
    front=prod.load_sf2_wall();top=prod.build_topology(front.triangles,len(front.coordinates));sdf=prod.NativeSdf();schedule=prod.load_schedule()
    _,coords,normals=timed('candidate_cache_deserialization',lambda:prod.load_candidate(BASE/'LAYER3_CANDIDATE.npz',BASE/'INITIAL_FORECAST.npz'))
    atlas=timed('risk_atlas_initialized_from_preserved_forecast',lambda:prod.RiskAtlas(front,top,schedule,sdf.source_identity,coords,normals))
    planner=timed('empty_persistent_planner',prod.PersistentPlanner);native=timed('certificate_native_binding',Filter)
    q,labels=timed('one_layer_side_arrays',lambda:surface_arrays(front.coordinates,top.edges,[coords[0]]))
    forest=timed('one_layer_certificate_index',lambda:native.forest(q,len(top.edges)));forest.close()
    outcome=next(o for o in read(BASE/'LAYER3_CORRECTIONS.json') if o['patch']==4)
    patch=planner.patch(top,np.asarray(outcome['scope']['active']));forecast=prod.SimpleNamespace(layers=np.arange(1,21),vertex={'coordinates':coords})
    result=planner.detailed.get(front,forecast,top,patch.detailed)
    original=np.savez_compressed;capture={}
    def in_memory(path,**arrays):
        capture['array_count']=len(arrays);capture['uncompressed_array_bytes']=sum(v.nbytes for v in arrays.values());capture['metadata_characters']=len(str(arrays['metadata']))
        out=io.BytesIO();timed('npz_compression_one_patch',lambda:original(out,**arrays));capture['payload']=out.getvalue()
    np.savez_compressed=in_memory
    try:timed('planner_pack_json_and_compress_one_patch',lambda:planner.save(OUT/'not_written.npz',array_hash(coords)))
    finally:np.savez_compressed=original
    def inflate():
        with np.load(io.BytesIO(capture['payload']),allow_pickle=False) as z:return sum(z[k].nbytes for k in z.files)
    timed('npz_inflate_one_patch',inflate)
    save('STORAGE_BOUNDED.json',dict(stages=rows,scope=dict(columns=len(patch.detailed.columns),prisms=20*len(patch.detailed.columns),detailed_entries=1),
        array_count=capture['array_count'],uncompressed_array_bytes=capture['uncompressed_array_bytes'],metadata_characters=capture['metadata_characters'],compressed_bytes=len(capture['payload']),
        payload_sha256=hashlib.sha256(capture['payload']).hexdigest(),scientific_hash=scientific_hash(result),
        disk_payload_persisted=False,peak_rss=psutil.Process().memory_info().peak_wset,seconds=time.perf_counter()-start))

if __name__=='__main__':run()

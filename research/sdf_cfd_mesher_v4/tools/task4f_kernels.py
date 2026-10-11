"""Bounded exact-arithmetic, setup, whole-layer, and storage measurements."""
from task4f_common import *
from task4f_instrument import Timers,instrument_statements,same_arrays
import psutil,inspect,cProfile,pstats,gzip,io
import task4_prepare_next as prod
from sdf_cfd_mesher_v4.forecast.forensics import exact_wedge
from sdf_cfd_mesher_v4.certificate.continuous_prism_certificate.runtime import canonical_bytes
from sdf_cfd_mesher_v4.forecast.engine import vertex_trajectories

def timed(name,fn,rows):
    t=time.perf_counter();c=time.process_time();v=fn();rows.append(dict(stage=name,seconds=time.perf_counter()-t,cpu_seconds=time.process_time()-c));return v

def run():
    require_time();started=time.perf_counter();setup=[]
    front=timed('frozen_wall_load',prod.load_sf2_wall,setup);top=timed('topology_build',lambda:prod.build_topology(front.triangles,len(front.coordinates)),setup)
    sdf=timed('native_sdf_setup',prod.NativeSdf,setup);schedule=timed('binary64_schedule_load',prod.load_schedule,setup)
    store=prod.CommitStore(BASE/'build_A',front,top,schedule);state=timed('read_only_restart_20_layer_chain',store.restart,setup)
    # One preserved passive layer, not a build.
    low=np.load(BASE/'build_A/FRONT_15.npy');high=np.load(BASE/'build_A/FRONT_16.npy');priorlow=np.load(BASE/'build_A/FRONT_14.npy')
    whole=[];previous=timed('previous_layer_full_cell_metrics',lambda:prod.evaluate_cells(top,priorlow,low,15),whole)
    result=timed('whole_new_layer_metrics',lambda:prod.evaluate_layer(top,low,high,16,previous),whole)
    assert result['gate']==read(BASE/'LAYER_16_NUMERIC.json')['gate']
    whole.append(dict(stage='identity',gate_identical=True,cells=len(top.triangles),edges=len(top.edges),metrics_hash=scientific_hash(dict(cells=result['cells'],gate=result['gate']))))
    # Exact sign kernel. Uniform stable source IDs; positive committed cells.
    ids=np.linspace(0,len(top.triangles)-1,4096,dtype=np.int64);p=np.concatenate((low[top.triangles[ids]],high[top.triangles[ids]]),axis=1)
    t=Timers();wrapped=instrument_statements(exact_wedge,t,'exact_wedge',[
        ('p=','binary64_to_rational'),('a0=','reference_edges'),('a1=','reference_edges'),('b0=','reference_edges'),('b1=','reference_edges'),
        ('q0=','polynomial_cross'),('q1=','polynomial_cross'),('q2=','polynomial_cross'),('vertical=','vertical_edges'),
        ('for j,','rational_extrema'),('value,','minimum_order'),('mean=','volume'),('integ=','volume'),('volume=','volume'),('return','string_output')])
    exact=[];plain=timed('exact_wedge_4096',lambda:[exact_wedge(a) for a in p],exact)
    prof=timed('instrumented_wedge_4096',lambda:[wrapped(a) for a in p],exact);assert plain==prof
    encoded=timed('canonical_record_serialization',lambda:[canonical_bytes(dict(column=int(c),**r)) for c,r in zip(ids,plain)],exact)
    digest=timed('record_hashing',lambda:hashlib.sha256(b''.join(encoded)).hexdigest(),exact)
    packed=timed('gzip_compression',lambda:gzip.compress(b''.join(encoded),mtime=0),exact)
    output=OUT/'exact_wedge_fixture.jsonl.gz';timed('bounded_fixture_disk_write',lambda:output.write_bytes(packed),exact)
    cp=cProfile.Profile();cp.enable();[exact_wedge(a) for a in p[:1024]];cp.disable();st=pstats.Stats(cp)
    table=[dict(function=k[2],file=k[0].replace(str(ROOT),'V4'),line=k[1],calls=v[1],self_seconds=v[2],total_seconds=v[3]) for k,v in st.stats.items()]
    save('EXACT_JACOBIAN_PROFILE.json',dict(timers=exact,kernels=t.table(),cprofile=sorted(table,key=lambda r:r['self_seconds'],reverse=True)[:30],cells=4096,layer=16,column_hash=array_hash(ids),records_hash=digest,compressed_bytes=len(packed),uncompressed_bytes=sum(map(len,encoded)),instrumentation_equal=True,
        whole_run_unique_cells=2126040,whole_run_exact_calls=2126040,final_rational_rechecks=0,local_trial_calls=295747,whole_local_overlap='Local trial cells that equal accepted geometry are checked again by the whole-layer pass. Selected-trial exact-input redundancy is reported separately.'))
    # Bounded SDF forecast timing, no complete cold forecast.
    sample=np.linspace(0,len(front.coordinates)-1,1024,dtype=np.int64);ft=Timers()
    ft.wrap(sdf,'evaluate','native_sdf.evaluate')
    tr=timed('1024_vertex_20_layer_forecast',lambda:vertex_trajectories(front,schedule,sdf,sample),setup)
    assert tr[0]['coordinates'].shape==(20,1024,3)
    with np.load(BASE/'INITIAL_FORECAST.npz') as z:assert np.array_equal(tr[0]['coordinates'],z['coordinates'][:,sample])
    save('INITIAL_FORECAST_BOUNDED.json',dict(vertices=1024,layers=20,evaluation_timers=ft.table(),projection_timer=tr[1],coordinate_identity=True,coordinate_hash=array_hash(tr[0]['coordinates']),whole_forecast_rerun=False))
    # Current setup startup, measured in a fresh interpreter; warm OS cache.
    import_rows=[]
    for name,code in [('python_startup','pass'),('production_import','import task4_prepare_next'),('certificate_import','import sdf_cfd_mesher_v4.certificate.production')]:
        env=dict(os.environ,PYTHONPATH=os.pathsep.join((str(ROOT/'tools'),str(ROOT/'src'))))
        timed(name,lambda code=code:subprocess.run([sys.executable,'-B','-c',code],env=env,cwd=REPO,check=True,capture_output=True),import_rows)
    save('STARTUP_DECOMPOSITION.json',dict(historical_task_pre_science_seconds=401.819024,historical_interpretation='Task start to scientific supervisor start, including code/instrumentation preparation and smoke tests; no historical substage timers exist. Not a measured 402-second Python startup.',measured_current_setup=setup,process_startup=import_rows,warm_filesystem_cache=True,not_subtracted_from_historical_runtime=True))
    save('WHOLE_LAYER_BOUNDED.json',whole)
    # Existing archived checkpoint sizes. Read metadata, do not reconstruct old caches.
    archived=read(E/'PLANNER_ARCHIVE.json');archive_rows=[dict(path=r.get('path',r.get('file')),bytes=r['bytes'],layer=r.get('checkpoint',{}).get('layer')) for r in archived]
    # Disk file population counts and duplicate content identities for the study.
    storage=[]
    for glob in ('LAYER*_CANDIDATE.npz','LAYER*_ATLAS.npz','build_A/FRONT_*.npy','build_A/LAYER_*.json','LAYER*_EXACT_SIGNS.jsonl.gz'):
        files=list(BASE.glob(glob));storage.append(dict(pattern=glob,files=len(files),bytes=sum(f.stat().st_size for f in files)))
    # Atomic checkpoint primitive timings on one bounded disposable study artifact.
    from sdf_cfd_mesher_v4.advancing_front.rolling_state import atomic_coordinates,atomic_json
    checkpoint=[];timed('atomic_coordinate_write_fsync',lambda:atomic_coordinates(OUT/'checkpoint_fixture.npy',high[:1024]),checkpoint)
    timed('array_sha256_1024',lambda:array_hash(high[:1024]),checkpoint)
    timed('atomic_json_write_fsync',lambda:atomic_json(OUT/'checkpoint_fixture.json',dict(hash=array_hash(high[:1024]),scope='audit fixture only')),checkpoint)
    save('CHECKPOINT_AUDIT.json',dict(archive=archive_rows,archived_planner_bytes=sum(r['bytes'] for r in archive_rows),remaining_storage=storage,bounded_primitives=checkpoint,
        source_facts=['Planner snapshot persists up to six complete local detailed results, not deltas.','Candidate snapshots store changed trajectories against INITIAL_FORECAST, not full shell copies.','CommitStore.restart re-reads and hashes every earlier immutable checkpoint. It runs before and after every commit, and on stage startup.','Historical 60.37-second planning persistence timer also includes atlas adoption, candidate serialization, warnings and decisions; compression and fsync were not timed separately.'],historical_substage_attribution='Unavailable; bounded current measurements are not substituted for historical timings.'))
    save('KERNEL_RUN.json',dict(seconds=time.perf_counter()-started,peak_rss=psutil.Process().memory_info().peak_wset,task_elapsed=elapsed()))

if __name__=='__main__':run()

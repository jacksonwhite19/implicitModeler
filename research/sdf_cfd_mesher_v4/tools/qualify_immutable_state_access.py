"""Sealed L5 access qualification. No forecast, geometry generation or core."""
from contextlib import ExitStack
import json
from pathlib import Path
import time
from unittest.mock import patch
import psutil
import numpy as np
import audit_dependency_scaling as audit
import immutable_state_access as access
from path_preserving_deduplication import durable_json,file_hash,metadata,physical_key
from sdf_cfd_mesher_v4.common.authority import identity
from sdf_cfd_mesher_v4.advancing_front.native_checkpoint import NativeCheckpointStore
from sdf_cfd_mesher_v4.advancing_front.mixed_front import scientific_hash
from rolling_layer_retention import checkpoint_closure

OUT=audit.cycle.V4/'results/incremental_immutable_state_access'
STORE=OUT/'candidate'
def save(name,value):durable_json(OUT/name,value)
def metrics(start,cpu,io):
    p=psutil.Process();now=p.io_counters();m=p.memory_info()
    return dict(seconds=time.perf_counter()-start,cpu=time.process_time()-cpu,peak_rss=m.peak_wset,retained_rss=m.rss,
        io={k:getattr(now,k)-getattr(io,k) for k in io._fields})

def enroll():
    start=time.perf_counter();cpu=time.process_time();io=psutil.Process().io_counters()
    refs,env=checkpoint_closure(audit.STORE);pointer=audit.read(audit.STORE/'CURRENT.json')
    assert env['scientific']==audit.EXPECTED
    # Keep all original checkpoint objects unchanged through validation and
    # enrollment. No historical hardlinks, copies, metadata edits or writes.
    with ExitStack() as held:
        for p in refs:held.enter_context(access.guard(p))
        state=NativeCheckpointStore(audit.STORE).restart()
        assert scientific_hash(state)==audit.EXPECTED
        wall,tri,surface=audit.cycle.load_production_wall(audit.cycle.SURFACE)
        expected=dict(audit.cycle.CONFIG,layer=5,committed=audit.EXPECTED,front_coordinates=audit.read(audit.LAYERS/'L5/A/READY.json')['coordinates'])
        definition=audit.read(audit.cycle.SURFACE.parent/'GEOMETRY.json')['definition']
        with patch.object(audit.cycle.BoundProductionForecastCache,'forecast',audit.denied),patch.object(audit.cycle.ProductionNextLayerProvider,'propose',audit.denied),patch.object(audit.cycle.MixedFrontRunner,'step',audit.denied):
            control=audit.cycle.ProductionForecastAdapter(state,wall,tri,surface,audit.cycle.REQUEST,audit.cycle.Source(dict(components=[definition])),expected)
        objects=STORE/'objects';objects.mkdir(parents=True,exist_ok=True)
        def object_file(role,value,array=False):
            temp=objects/(role+'.pending')
            if array:
                import os
                with temp.open('wb') as f:np.save(f,value,allow_pickle=False);f.flush();os.fsync(f.fileno())
            else:durable_json(temp,value)
            h=file_hash(temp);target=objects/(h+('.npy' if array else '.json'))
            if target.exists():
                assert file_hash(target)==h;temp.unlink()
            else:temp.replace(target)
            return dict(path=str(target.resolve()),sha256=h)
        keys=sorted(state['vertices']);values=np.asarray([state['vertices'][k] for k in keys],dtype='<f8')
        hot=dict(vertex_keys=object_file('keys',keys),vertex_values=object_file('coordinates',values,True),
            front_faces=object_file('front_faces',{k:state['faces'][k] for k in state['prism_front'].values()}))
        from sdf_cfd_mesher_v4.advancing_front.native_checkpoint import scientific_digest
        assert scientific_digest(access.VertexMap(keys,values))==scientific_digest(state['vertices'])
        for d in hot.values():refs[d['path']]=d['sha256']
        inputs=[audit.cycle.REQUEST,audit.cycle.SURFACE/'SURFACE_AUTHORITY.json',audit.cycle.SURFACE.parent/'GEOMETRY.json']
        inputs += [audit.cycle.SURFACE/name for name in surface['files']]
        for p in inputs:refs[str(p.resolve())]=file_hash(p)
        files={p:access.attest(p,h) for p,h in refs.items()}
        import production_forecast_admission as adapter_module
        implementation={str(Path(m.__file__).resolve()):file_hash(m.__file__) for m in (access,adapter_module)}
        content=dict(schema='V4LegacyComponentMapping/1',legacy_scientific=audit.EXPECTED,
            legacy_checkpoint_sha256=pointer['sha256'],legacy_components=env['components'],
            hot={k:v['sha256'] for k,v in hot.items()},implementation=implementation,
            receipt_roles=dict(payload='proof_segments.payloads',pair_identity='proof_segments.faces/pairs/codes',
                current_bindings='legacy certificate receipt_delta map',coverage='legacy scientific receipt map and segment populations'),
            quality='Legacy immutable quality component; values and exact hard-gate evidence retained')
        body=dict(schema=access.SCHEMA,complete=True,epoch=access.epoch(),store=str(audit.STORE.resolve()),
            legacy_pointer=pointer,components=env['components'],hot=hot,files=files,content=content,content_root=identity(content),
            counts={k:len(state[k]) for k in ('vertices','cells','faces','quality','certificate')},
            expected=expected,binding=control.binding)
        old=access.read(STORE/'CURRENT.json')['root'] if (STORE/'CURRENT.json').exists() else None
        root=access.publish_manifest(STORE,body,expected_previous=old)
        save('ENROLLMENT.json',dict(root=root,content_root=body['content_root'],legacy=audit.EXPECTED,
            complete_legacy_validation=True,packed_geometry_exact=True,front_face_records=len(state['prism_front']),
            mapped_components=list(env['components']),candidate=access.read(STORE/'CURRENT.json'),
            no_forecast=True,no_generation=True,**metrics(start,cpu,io)))
    print('ENROLLED',root,flush=True)

def normal(record_name='NORMAL.json'):
    import production_forecast_admission as pf
    import sqlite3
    import sdf_cfd_mesher_v4.advancing_front.native_checkpoint as nc
    start=time.perf_counter();cpu=time.process_time();io=psutil.Process().io_counters()
    root=access.read(OUT/'ENROLLMENT.json')['root']
    with access.ImmutableState(STORE,root) as state:
        opened=time.perf_counter();wall,tri,surface=audit.cycle.load_production_wall(audit.cycle.SURFACE)
        definition=audit.read(audit.cycle.SURFACE.parent/'GEOMETRY.json')['definition']
        with patch.object(sqlite3,'connect',audit.denied),patch.object(nc,'scientific_digest',audit.denied),patch.object(pf,'scientific_hash',audit.denied),patch.object(NativeCheckpointStore,'restart',audit.denied),patch.object(audit.cycle.BoundProductionForecastCache,'forecast',audit.denied),patch.object(audit.cycle.ProductionNextLayerProvider,'propose',audit.denied),patch.object(audit.cycle.MixedFrontRunner,'step',audit.denied):
            adapter=pf.ProductionForecastAdapter.from_immutable_access(state,wall,tri,surface,audit.cycle.REQUEST,audit.cycle.Source(dict(components=[definition])),state.body['expected'])
            cache=pf.BoundProductionForecastCache(adapter);cache.check(adapter)
            assert cache.cache.result is None
            cache_identity=cache.scientific_identity()
        assert adapter.binding==state.body['binding']
        assert float(adapter.remaining_increments[0])==0.00035506009452908446
        stats=dict(state.stats)
        for name in ('sqlite_rows_visited','proof_receipts_deserialized','proof_payloads_hashed','quality_rows_visited','full_prefix_scientific_hashes'):stats.setdefault(name,0)
        record=dict(root=root,legacy=state.verified_legacy_identity(),binding=adapter.binding,increment=float(adapter.remaining_increments[0]),
            open_seconds=opened-start,stats=stats,no_forecast=True,no_generation=True,exact_binding=True,**metrics(start,cpu,io))
        record['denied_during_initialization']=['SQLite connection','legacy scientific_digest','legacy scientific_hash','native restart','forecast','propose','runner step']
        record['empty_bound_cache_identity']=cache_identity
        save(record_name,record)
    print('NORMAL',record['seconds'],record['stats'],flush=True)

def deep():
    start=time.perf_counter();cpu=time.process_time();io=psutil.Process().io_counters()
    with access.ImmutableState(STORE,access.read(OUT/'ENROLLMENT.json')['root']) as state:
        result=state.deep_verify()
        save('DEEP.json',dict(**result,**metrics(start,cpu,io),no_forecast=True,no_generation=True))
    print('DEEP PASSED',flush=True)

def workloads():
    """Existing L5 geometry only; bounded exact access comparisons, no prediction."""
    import sqlite3
    import sys
    from sdf_cfd_mesher_v4.certificate.aabb_index import AABBIndex
    start=time.perf_counter();cpu=time.process_time();io=psutil.Process().io_counters()
    with access.ImmutableState(STORE,access.read(OUT/'ENROLLMENT.json')['root']) as state:
        # Dense geometry is the actual adapter access pattern. Compare JSON
        # against the packed immutable projection for every existing vertex.
        t=time.perf_counter();legacy=state._component_json('vertices');json_seconds=time.perf_counter()-t
        t=time.perf_counter();packed=state['vertices'];packed_seconds=time.perf_counter()-t
        keys=sorted(legacy);t=time.perf_counter()
        old=np.asarray([legacy[k] for k in keys]);old_dense=time.perf_counter()-t
        t=time.perf_counter();new=np.asarray([packed[k] for k in keys]);new_dense=time.perf_counter()-t
        assert np.array_equal(old,new)
        # Fixed representative random access workload, not a scientific gate.
        rng=np.random.default_rng(20260923);sample=[keys[int(i)] for i in rng.integers(0,len(keys),size=4096)]
        t=time.perf_counter();old_random=[legacy[k] for k in sample];old_random_seconds=time.perf_counter()-t
        t=time.perf_counter();new_random=[packed[k] for k in sample];new_random_seconds=time.perf_counter()-t
        assert old_random==new_random
        representation=dict(vertices=len(keys),json_load_seconds=json_seconds,packed_open_seconds=packed_seconds,
            json_dense_seconds=old_dense,packed_dense_seconds=new_dense,random_queries=len(sample),
            json_random_seconds=old_random_seconds,packed_random_seconds=new_random_seconds,exact=True,
            json_bytes=state._object_path(state.body['components']['vertices']).stat().st_size,
            packed_coordinate_bytes=Path(state.body['hot']['vertex_values']['path']).stat().st_size,
            key_index_bytes=Path(state.body['hot']['vertex_keys']['path']).stat().st_size,
            json_python_payload_bytes=sys.getsizeof(legacy)+sum(sys.getsizeof(k)+sys.getsizeof(v)+sum(sys.getsizeof(x) for x in v) for k,v in legacy.items()),
            packed_python_index_bytes=sys.getsizeof(packed._keys)+sys.getsizeof(packed._index)+sum(sys.getsizeof(k)+sys.getsizeof(v) for k,v in packed._index.items()),
            packed_mapped_data_bytes=packed._values.nbytes,
            conclusion='JSON per-key extraction is faster; packed view uses less coordinate/index storage and Python heap. Most initialization improvement comes from cold proof/quality access, not this format choice.')
        # Preserve the existing inclusive AABB geometry index. It needs exact
        # geometry and ownership, never historical proof payloads.
        t=time.perf_counter();faces=state._component_json('faces');face_ids=sorted(faces)
        low=[];high=[]
        for key in face_ids:
            x=np.asarray([packed[v] for v in faces[key]['vertices']]);low.append(x.min(0));high.append(x.max(0))
        low=np.asarray(low);high=np.asarray(high);geometry_seconds=time.perf_counter()-t
        t=time.perf_counter();index=AABBIndex(low,high);index_seconds=time.perf_counter()-t
        # One existing face from each available layer range plus spatially
        # spread ordered samples; no new cell/front construction.
        selected=sorted(set(np.linspace(0,len(face_ids)-1,32,dtype=int).tolist()))
        t=time.perf_counter();counts=[]
        for i in selected:
            found=index.overlaps(i)
            brute=np.flatnonzero(np.all(high[i]>=low,axis=1)&np.all(high>=low[i],axis=1)).tolist()
            assert found==brute
            for j in found:
                assert state['faces'][face_ids[j]]==faces[face_ids[j]]
            counts.append(len(found))
        spatial=dict(faces=len(face_ids),geometry_seconds=geometry_seconds,index_seconds=index_seconds,
            query_and_brute_force_seconds=time.perf_counter()-t,queries=len(selected),candidate_counts=counts,
            bounds_bytes=low.nbytes+high.nbytes,exact_inclusive_aabb_equality=True,
            exact_face_geometry_and_owner_ids=True,proof_rows_loaded=state.stats['sqlite_rows_visited'])
        assert spatial['proof_rows_loaded']==0
        # One exact keyed proof from each immutable base/update component.
        # Retained SQLite remains viable for this access; no replacement DB.
        tables=[];row=state.body['components']['certificate']
        while row['encoding']=='receipt_delta':tables.append(row['updates']);row=row['base']
        tables.append(row);lookups=[];selection_queries=0
        for offset,row in enumerate(tables):
            path=state._object_path(row);db=sqlite3.connect(path.as_uri()+'?mode=ro&immutable=1',uri=True)
            try:candidates=db.execute('SELECT key,value FROM receipts ORDER BY key LIMIT 256').fetchall();selection_queries+=1
            finally:db.close()
            selected=None
            for key,raw in candidates:
                shadowed=False
                for newer in tables[:offset]:
                    p=state._object_path(newer);db=sqlite3.connect(p.as_uri()+'?mode=ro&immutable=1',uri=True)
                    try:found=db.execute('SELECT 1 FROM receipts WHERE key=?',(key,)).fetchone();selection_queries+=1
                    finally:db.close()
                    if found is not None:shadowed=True;break
                if not shadowed:selected=(key,raw);break
            if selected is None:raise ValueError('BoundedSampleDidNotFindLiveComponentKey')
            key,raw=selected
            t=time.perf_counter();value=state['certificate'][key];first=time.perf_counter()-t
            t=time.perf_counter();assert state['certificate'][key]==value;again=time.perf_counter()-t
            assert value==json.loads(raw)
            lookups.append(dict(key=key,component=row['file'],first_seconds=first,repeated_seconds=again,exact=True))
        save('WORKLOADS.json',dict(representation=representation,spatial=spatial,proof_point_access=lookups,
            sample_selection_queries=selection_queries,stats=dict(state.stats),no_forecast=True,no_generation=True,**metrics(start,cpu,io)))
    print('WORKLOADS PASSED',flush=True)

def initialization_faults():
    import production_forecast_admission as pf
    from copy import deepcopy
    results={}
    with access.ImmutableState(STORE,access.read(OUT/'ENROLLMENT.json')['root']) as state:
        wall,tri,surface=audit.cycle.load_production_wall(audit.cycle.SURFACE)
        definition=audit.read(audit.cycle.SURFACE.parent/'GEOMETRY.json')['definition']
        with patch.object(pf,'scientific_hash',audit.denied),patch.object(audit.cycle.BoundProductionForecastCache,'forecast',audit.denied),patch.object(audit.cycle.ProductionNextLayerProvider,'propose',audit.denied),patch.object(audit.cycle.MixedFrontRunner,'step',audit.denied):
            for field in ('request','schedule','committed','front_coordinates','geometry_identity','surface_identity','layer'):
                expected=deepcopy(state.body['expected']);expected[field]='wrong'
                try:pf.ProductionForecastAdapter.from_immutable_access(state,wall,tri,surface,audit.cycle.REQUEST,audit.cycle.Source(dict(components=[definition])),expected)
                except ValueError as error:results[field]=str(error)
                else:raise AssertionError('Invalid binding accepted:'+field)
        save('INITIALIZATION_FAULTS.json',dict(rejected=results,no_forecast=True,no_generation=True,stats=dict(state.stats)))
    print('INITIALIZATION FAULTS PASSED',flush=True)

def tests():
    import subprocess
    import sys
    start=time.perf_counter()
    command=[sys.executable,'-m','pytest',*[str(audit.cycle.V4/'tests'/name) for name in (
        'test_immutable_state_access.py','test_production_single_layer_cycle.py','test_production_bounded_layers.py')],'-q']
    result=subprocess.run(command,text=True,capture_output=True)
    save('TESTS.json',dict(command=command,exit_code=result.returncode,stdout=result.stdout,stderr=result.stderr,seconds=time.perf_counter()-start))
    print(result.stdout,result.stderr,flush=True)
    if result.returncode:raise SystemExit(result.returncode)

if __name__=='__main__':
    import sys
    {'enroll':enroll,'normal':normal,'normal_replay':lambda:normal('NORMAL_REPLAY.json'),'deep':deep,'workloads':workloads,'initialization_faults':initialization_faults,'tests':tests}[sys.argv[1]]()

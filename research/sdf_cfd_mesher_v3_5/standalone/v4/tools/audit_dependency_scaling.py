"""Read-only L5 dependency audit. Never calls a predictor or publisher."""
import collections
import functools
import json
import shutil
import subprocess
import time
from pathlib import Path
from unittest.mock import patch
import psutil
import production_single_layer_cycle as cycle
from path_preserving_deduplication import metadata, physical_key, durable_json, file_hash
from rolling_layer_retention import checkpoint_closure
from sdf_cfd_mesher_v4.advancing_front import native_checkpoint as nc, mixed_front as mf
from sdf_cfd_mesher_v4.advancing_front.native_receipt_delta import ReceiptDelta, raw_items

ROOT=cycle.V4/'results/dependency_graph_scaling_audit'
LAYERS=cycle.V4/'results/production_adaptive_capsule_layers4_to5'
STORE=LAYERS/'L5/A/checkpoints'
EXPECTED='686c220a55965e0ee2868f2ab8502c6cda3ef6b6db8e97fca4806cf04afc0ec7'
def read(p):return json.loads(Path(p).read_text())
def save(name,x):ROOT.mkdir(exist_ok=True,parents=True);durable_json(ROOT/name,x)
def denied(*a,**kw):raise AssertionError('AuditForbidsForecastGenerationPublication')

class Timers:
    def __init__(self):self.rows={};self.stack=[]
    def wrap(self,name,fn):
        @functools.wraps(fn)
        def call(*a,**kw):
            row=self.rows.setdefault(name,dict(calls=0,wall=0.,exclusive=0.,cpu=0.));row['calls']+=1
            frame=[0.];self.stack.append(frame);t=time.perf_counter();c=time.process_time()
            try:return fn(*a,**kw)
            finally:
                elapsed=time.perf_counter()-t;self.stack.pop();row['wall']+=elapsed;row['exclusive']+=elapsed-frame[0];row['cpu']+=time.process_time()-c
                if self.stack:self.stack[-1][0]+=elapsed
        return call

def preflight():
    head=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip()
    assert head=='b8d52a5fc7add936625212868180e696ee85fef6'
    status=subprocess.check_output(['git','status','--porcelain=v1','-z'])
    prior=bytes.fromhex(read(LAYERS/'BASELINE.json')['status_hex'])
    owned=[b'dependency_graph_scaling_audit',b'audit_dependency_scaling.py',b'seal_dependency_scaling_audit.py',b'test_dependency_scaling_audit.py']
    original=[r for r in status.split(bytes([0])) if r and not any(name in r for name in owned)]
    assert original==[r for r in prior.split(bytes([0])) if r]
    protected={}
    for name,h in [('AUTHORITY.json','3961bc545a63d6883ba209317a99aeba167f66f59b4239eee891dd8c82d028ce'),('REPORT.md','448730843411a7d21b54fe3fcd630e50b22bb90d8a973343944a9a96d4616968')]:
        assert file_hash(LAYERS/name)==h;protected[str(LAYERS/name)]=h
    for name,h in read(LAYERS/'AUTHORITY.json')['evidence'].items():
        p=LAYERS/name;assert file_hash(p)==h;protected[str(p)]=h
    paths,_=checkpoint_closure(STORE);rows=[metadata(p) for p in sorted(paths)]
    unique={physical_key(r):r for r in rows}
    save('BASELINE.json',dict(head=head,status_hex=status.hex(),unrelated_entries=len(original),free=shutil.disk_usage(ROOT.anchor).free,protected=protected,closure=rows,closure_allocated=sum(r['cluster_bytes'] for r in unique.values())))

def profile():
    timers=Timers();p=psutil.Process();io=p.io_counters();start=time.perf_counter();stages={}
    patches=[]
    for obj,names in [(nc,['file_hash','scientific_digest']),(mf,['scientific_hash']),(nc.NativeCheckpointStore,['_object','_receipt','_restore']),(ReceiptDelta,['measure_identity'])]:
        for name in names:patches.append(patch.object(obj,name,timers.wrap(obj.__name__+'.'+name if hasattr(obj,'__name__') else name,getattr(obj,name))))
    for q in patches:q.start()
    try:
        t=time.perf_counter();state=nc.NativeCheckpointStore(STORE).restart();stages['restart']=time.perf_counter()-t
        t=time.perf_counter();assert mf.scientific_hash(state)==EXPECTED;stages['repeat_identity']=time.perf_counter()-t
        wall,tri,surface=cycle.load_production_wall(cycle.SURFACE)
        expected=dict(cycle.CONFIG,layer=5,committed=EXPECTED,front_coordinates=read(LAYERS/'L5/A/READY.json')['coordinates'])
        t=time.perf_counter()
        with patch.object(cycle.BoundProductionForecastCache,'forecast',denied),patch.object(cycle.ProductionNextLayerProvider,'propose',denied),patch.object(cycle.MixedFrontRunner,'step',denied),patch.object(nc.NativeCheckpointStore,'publish',denied):
            adapter=cycle.ProductionForecastAdapter(state,wall,tri,surface,cycle.REQUEST,cycle.Source(dict(components=[read(cycle.SURFACE.parent/'GEOMETRY.json')['definition']])),expected)
        stages['initialization']=time.perf_counter()-t
        after=p.io_counters();mem=p.memory_info()
        save('PROFILE.json',dict(scientific=EXPECTED,stages=stages,timers=timers.rows,seconds=time.perf_counter()-start,cpu=time.process_time(),io_delta={k:getattr(after,k)-getattr(io,k) for k in io._fields},peak_rss=mem.peak_wset,retained_rss=mem.rss,counts={k:len(state[k]) for k in ['vertices','cells','faces','quality','certificate','atlas','forecast','columns']},binding=adapter.binding,increment=float(adapter.remaining_increments[0]),receipt_audit=nc.RECEIPT_AUDIT_STATS,hash_audit=mf.SCIENTIFIC_HASH_STATS,no_forecast=True,no_generation=True))
    finally:
        for q in reversed(patches):q.stop()
    print('PROFILE COMPLETE',flush=True)

def receipts():
    start=time.perf_counter();pointer=read(STORE/'CURRENT.json');env=read(STORE/pointer['checkpoint']);store=nc.NativeCheckpointStore(STORE)
    # Byte authentication is independently exercised by the fresh restore.
    state={k:read(store.objects/env['components'][k]['file']) for k in ['faces','cells']}
    layers={k:sorted({state['cells'][c]['layer'] for c in row['owners']}) for k,row in state['faces'].items()}
    table=store._receipt(env['components']['certificate'],verify=False)
    counts=collections.Counter();sizes=collections.Counter();methods=collections.Counter();updates=collections.Counter();changedfaces=set();changedfields=collections.Counter()
    def group(row):return ','.join(map(str,sorted(set(x for f in row['faces'] for x in layers[f]))))
    for key,raw in raw_items(table):
        row=json.loads(raw);g=group(row);counts[g]+=1;sizes[g]+=len(key.encode())+len(raw.encode());methods[str(row['value'].get('method',row['value'].get('classification','unknown')))]+=1
    print('RECEIPT POPULATION COMPLETE',flush=True)
    aiter=iter(raw_items(table.base));biter=iter(raw_items(table.updates));a=next(aiter,None);b=next(biter,None);summary=collections.Counter()
    while a is not None or b is not None:
        if b is None or (a is not None and a[0]<b[0]):summary['unchanged_base']+=1;a=next(aiter,None)
        elif a is None or b[0]<a[0]:summary['new']+=1;b=next(biter,None)
        else:
            av=json.loads(a[1]);bv=json.loads(b[1]);summary['overlap']+=1
            assert {k:v for k,v in av.items() if k!='inputs'}=={k:v for k,v in bv.items() if k!='inputs'}
            delta=[k for k in set(av['inputs'])|set(bv['inputs']) if av['inputs'].get(k)!=bv['inputs'].get(k)]
            summary['binding_only' if delta else 'equal']+=1;updates[group(bv)]+=1
            for k in delta:
                changedfields[k.partition(':')[0]]+=1
                if k.startswith('face:'):changedfaces.add(k[5:])
            a=next(aiter,None);b=next(biter,None)
    facegroups=collections.Counter(','.join(map(str,layers[f])) for f in changedfaces)
    prev=LAYERS/'L4/A/checkpoints';pe=read(prev/read(prev/'CURRENT.json')['checkpoint']);oldfaces=read(prev/'objects'/pe['components']['faces']['file'])
    actual_changed={k for k,v in oldfaces.items() if state['faces'].get(k)!=v}
    assert changedfaces==actual_changed
    assert all(oldfaces[k]['vertices']==state['faces'][k]['vertices'] and len(oldfaces[k]['owners'])==1 and len(state['faces'][k]['owners'])==2 for k in changedfaces)
    save('RECEIPTS.json',dict(total=sum(counts.values()),by_incident_cell_layers=dict(counts),serialized_key_value_bytes=dict(sizes),methods=dict(methods),updates_by_incident_cell_layers=dict(updates),summary=dict(summary),changed_input_types=dict(changedfields),unique_changed_faces=len(changedfaces),changed_faces_by_owner_layers=dict(facegroups),actual_changed_face_set_exact=True,changed_faces_only_gain_second_owner=True,seconds=time.perf_counter()-start,peak_rss=psutil.Process().memory_info().peak_wset,removed=len(table.removed),note='Layer membership comes from actual face owners and cell.layer. Serialized bytes exclude SQLite page/index overhead; no per-layer physical attribution is inferred.'))

def storage():
    start=time.perf_counter();env=read(STORE/read(STORE/'CURRENT.json')['checkpoint']);rows=[];seen=set()
    def record(desc,role,producer):
        p=STORE/'objects'/desc['file'];r=metadata(p);r.update(role=role,producer=producer,sha256=desc['sha256'],consumer='L5 native restart',future_mutation=False,regeneration='Not newly qualified; retain immutable object')
        if r['path'] not in seen:rows.append(r);seen.add(r['path'])
    def receipt(desc,layer):
        if desc['encoding']=='receipt_delta':
            receipt(desc['base'],layer-1);record(desc['updates'],'receipt delta including dependency updates',layer);record(desc['removed'],'receipt removal set',layer)
        else:record(desc,'base receipt population',layer)
        for seg in desc.get('proof_segments',[]):
            for k in ('faces','pairs','codes','payloads'):record(seg[k],'compact proof segment '+k,layer)
    for k,d in env['components'].items():
        if k=='certificate':receipt(d,5)
        else:record(d,k,5 if k not in ['wall','first_layer','authority','version','events'] else 'inherited')
    closure,_=checkpoint_closure(STORE)
    for p in sorted(closure):
        if str(p) not in seen:
            r=metadata(p);r.update(role='checkpoint/pointer',producer=5);rows.append(r);seen.add(r['path'])
    native={physical_key(r):r for r in rows};native_bytes=sum(r['cluster_bytes'] for r in native.values())
    for p in [LAYERS/'L5/A/QUALITY.json',LAYERS/'L5/A/CANDIDATE.npz']:
        r=metadata(p);r.update(role='next diagnostics cache; not adapter initialization',producer=5,sha256=file_hash(p));rows.append(r)
    union={physical_key(r):r for r in rows};categories=collections.Counter()
    for r in union.values():categories[r['role']]+=r['cluster_bytes']
    content={};duplicates=0
    for key,r in union.items():
        digest=r.get('sha256') or file_hash(r['path']);r['sha256']=digest
        if digest in content:duplicates+=r['cluster_bytes']
        else:content[digest]=key
    save('STORAGE.json',dict(rows=rows,native_allocated=native_bytes,active_allocated=sum(r['cluster_bytes'] for r in union.values()),logical=sum(r['bytes'] for r in rows),duplicate_allocated=duplicates,hardlinked_logical=sum(r['bytes'] for r in rows if r['links']>1),shared_physical=sum(r['cluster_bytes'] for r in union.values() if r['links']>1),by_role=dict(categories),physical_objects=len(union),paths=len(rows),seconds=time.perf_counter()-start,free=shutil.disk_usage(ROOT.anchor).free))

def operations():
    from copy import deepcopy
    from sdf_cfd_mesher_v4.common.authority import identity
    start=time.perf_counter();e=read(STORE/read(STORE/'CURRENT.json')['checkpoint']);timers={}
    t=time.perf_counter();state={k:read(STORE/'objects'/d['file']) for k,d in e['components'].items() if d['encoding']=='json'};timers['json_component_decode']=time.perf_counter()-t
    t=time.perf_counter();inputs=mf.geometry_inputs(state);timers['geometry_inputs']=time.perf_counter()-t
    t=time.perf_counter();digest=identity(inputs);timers['geometry_input_root']=time.perf_counter()-t
    keys=('wall','first_layer','vertices','cells','faces','columns','outer','prism_front')
    t=time.perf_counter();clone={k:deepcopy(state[k]) for k in keys};timers['geometry_deepcopy']=time.perf_counter()-t
    t=time.perf_counter();assert all(clone[k]==state[k] for k in keys);timers['geometry_equality']=time.perf_counter()-t
    t=time.perf_counter();gh=nc.scientific_digest(clone);timers['geometry_canonical_hash']=time.perf_counter()-t
    incidence=collections.Counter();deps=0;layerrows=collections.Counter()
    for row in state['atlas'].values():
        ownerlayers=set()
        for key,value in row['dependencies'].items():
            assert inputs[key]==value;deps+=1
            ownerlayers.update(state['cells'][c]['layer'] for c in state['faces'][key[5:]]['owners'])
        layerrows[','.join(map(str,sorted(ownerlayers)))]+=1
    for f in state['faces'].values():
        for v in f['vertices']:incidence[v]+=1
    save('OPERATIONS.json',dict(seconds=time.perf_counter()-start,timings=timers,geometry_inputs=len(inputs),geometry_root=digest,geometry_hash=gh,atlas_rows=len(state['atlas']),atlas_dependency_entries=deps,atlas_owner_layers=dict(layerrows),vertex_face_incidence_entries=sum(incidence.values()),cells=len(state['cells']),faces=len(state['faces']),peak_rss=psutil.Process().memory_info().peak_wset,no_forecast=True,no_generation=True,note='Read-only replay of existing L5 bookkeeping. No candidate state, receipt binding, journal or publication created.'))

def proof_serialization():
    from native_proof_segments import LoadedSegment, SegmentProofRows
    from compact_proof_rows import CompactProofRows
    from validation_row_stream import clean_rows_digest
    start=time.perf_counter();p=psutil.Process();cpu=time.process_time();io=p.io_counters()
    e=read(STORE/read(STORE/'CURRENT.json')['checkpoint']);faces=read(STORE/'objects'/e['components']['faces']['file'])
    def indices(face):
        return tuple(sorted(int(v.split(':')[1])*16895+int(v.split(':')[3]) for v in face['vertices']))
    keys=sorted(faces,key=lambda k:indices(faces[k]));mapping={k:i for i,k in enumerate(keys)}
    seg=[LoadedSegment(s,STORE/'objects',mapping) for s in e['components']['certificate']['proof_segments']]
    rows=SegmentProofRows(seg,CompactProofRows());loaded=time.perf_counter();digest=clean_rows_digest(rows);done=time.perf_counter()
    assert digest==read(LAYERS/'L5/A/CERTIFICATE.json')['proof_hash']
    after=p.io_counters()
    save('PROOF_SERIALIZATION.json',dict(rows=len(rows),digest=digest,exact=True,segment_load_seconds=loaded-start,canonical_seconds=done-loaded,seconds=done-start,cpu=time.process_time()-cpu,peak_rss=p.memory_info().peak_wset,io_delta={k:getattr(after,k)-getattr(io,k) for k in io._fields},no_forecast=True,no_generation=True,note='Complete existing L5 proof-population identity replay. No certifier or predictor invoked. This is not a historical transaction timing.'))

def initialization_profile():
    import production_forecast_admission as pf
    start=time.perf_counter();state=nc.NativeCheckpointStore(STORE).restart();restored=time.perf_counter()
    wall,tri,surface=cycle.load_production_wall(cycle.SURFACE)
    expected=dict(cycle.CONFIG,layer=5,committed=EXPECTED,front_coordinates=read(LAYERS/'L5/A/READY.json')['coordinates'])
    timers=Timers();patches=[]
    for obj,names in [(pf,['scientific_hash','read_request','build_topology','array_hash','identity','frozen']),(pf.CommittedFrontState,['create'])]:
        for name in names:patches.append(patch.object(obj,name,timers.wrap(name,getattr(obj,name))))
    for q in patches:q.start()
    t=time.perf_counter()
    try:
        with patch.object(cycle.BoundProductionForecastCache,'forecast',denied),patch.object(cycle.ProductionNextLayerProvider,'propose',denied),patch.object(cycle.MixedFrontRunner,'step',denied),patch.object(nc.NativeCheckpointStore,'publish',denied):
            adapter=pf.ProductionForecastAdapter(state,wall,tri,surface,cycle.REQUEST,cycle.Source(dict(components=[read(cycle.SURFACE.parent/'GEOMETRY.json')['definition']])),expected)
        elapsed=time.perf_counter()-t
    finally:
        for q in reversed(patches):q.stop()
    assert adapter.binding==read(ROOT/'PROFILE.json')['binding']
    save('INITIALIZATION_PROFILE.json',dict(restore_seconds=restored-start,initialization_seconds=elapsed,timers=timers.rows,unscoped_initialization_seconds=elapsed-sum(x['exclusive'] for x in timers.rows.values()),exact_binding_replay=True,scientific=EXPECTED,prefix_coordinate_values=int(adapter.committed_prefix.size),current_vertices=len(adapter.front.coordinates),current_triangles=len(tri),no_forecast=True,no_generation=True,peak_rss=psutil.Process().memory_info().peak_wset))

def quality_dependencies():
    start=time.perf_counter();tables=[]
    for layer in (4,5):
        store=nc.NativeCheckpointStore(LAYERS/f'L{layer}/A/checkpoints');pointer=read(store.path/'CURRENT.json')
        assert file_hash(store.path/pointer['checkpoint'])==pointer['sha256']
        e=read(store.path/pointer['checkpoint']);assert file_hash(store.objects/e['components']['quality']['file'])==e['components']['quality']['sha256']
        tables.append(store._receipt(e['components']['quality'],verify=False))
    cells=read(store.objects/e['components']['cells']['file'])
    left=iter(raw_items(tables[0]));right=iter(raw_items(tables[1]));x=next(left,None);y=next(right,None);counts=collections.Counter();groups=collections.Counter()
    while x is not None or y is not None:
        if y is None or (x is not None and x[0]<y[0]):counts['removed']+=1;x=next(left,None)
        elif x is None or y[0]<x[0]:counts['new']+=1;groups['new:'+y[0].partition(':')[0]]+=1;y=next(right,None)
        else:
            if x[1]==y[1]:counts['exact_unchanged']+=1
            else:
                old=json.loads(x[1]);new=json.loads(y[1]);payload=({k:v for k,v in old.items() if k!='inputs'}=={k:v for k,v in new.items() if k!='inputs'})
                label='binding_only' if payload else 'payload_changed';counts[label]+=1
                kind,key=x[0].split(':',1);layer=(cells[key]['layer'] if kind in ('cell','final_incidence') else 'face');groups[f'{label}:{kind}:L{layer}']+=1
            x=next(left,None);y=next(right,None)
    save('QUALITY_DEPENDENCIES.json',dict(counts=dict(counts),groups=dict(groups),previous=len(tables[0]),current=len(tables[1]),seconds=time.perf_counter()-start,note='Complete exact row comparison of existing L4/L5 quality containers; layer membership comes from state.cells. L5 authentication is freshly tested; L4 file hash remains bound to its historical qualified checkpoint. No quality or geometry regenerated.'))

if __name__=='__main__':
    import sys
    {'preflight':preflight,'profile':profile,'receipts':receipts,'storage':storage,'operations':operations,'proof_serialization':proof_serialization,'initialization_profile':initialization_profile,'quality_dependencies':quality_dependencies}[sys.argv[1]]()

"""Retained-input writer and forecast controls. Never advances a front.

Forecast arrays are comparison fixtures, not live geometry or admission proof.
Writers/readers below are the actual existing file-backed operations.
"""
import argparse
from contextlib import ExitStack,nullcontext
from itertools import islice
import json
from pathlib import Path
import sys,time
from types import SimpleNamespace

V4=Path(__file__).resolve().parents[1];ROOT=V4.parents[1]
sys.path[:0]=[str(V4/'tools'),str(V4/'src')]
from path_preserving_deduplication import durable_json,file_hash
from record_receipt_resume import disk

OUT=V4/'results/generic_capsule_process_alignment'
REFERENCE=V4/'results/critical_reuse_production/run'

def read(p):return json.loads(Path(p).read_text(encoding='utf-8'))

def timing(call):
    tick=time.perf_counter();value=call();return value,time.perf_counter()-tick

def writers():
    import psutil
    import immutable_state_access as access
    import single_quality_component as q
    import compact_front_atlas as atlas
    from assembly_quality_handoff import installed as quality_scope
    from assembly_atlas_handoff import installed as atlas_scope
    from private_quality_codec_reuse import verified_private_codec
    from prism_sql_profile import measured_quality_writer
    from check_current_front_encoding import allocation
    target=OUT/'writers'
    if target.exists():raise ValueError('PreserveWriterControl')
    target.mkdir();start=time.perf_counter();result=dict(scope='10001 retained rows per writer; not a full layer benchmark',phases={},comparisons={})
    with ExitStack() as held:
        plan_path=REFERENCE/'CANDIDATE_PLAN.json';held.enter_context(access.guard(plan_path));plan=read(plan_path)
        for name in ('QUALITY_DELTA.json','NEW_ATLAS.json'):
            p=REFERENCE/name;held.enter_context(access.guard(p));r=plan['references'][str(p.resolve())]
            if file_hash(p)!=r['sha256'] or access.token(p)!=r['token']:raise ValueError('ChangedRetainedWriterInput',name)
        def load_quality():
            with q.snapshot(REFERENCE/'QUALITY_DELTA.json') as snap:
                if snap['semantic']!=plan['components']['quality']['delta']:raise ValueError('RetainedQualityMismatch')
                return list(islice(snap['rows'],10001))
        rows,result['source_quality_validation_s']=timing(load_quality)
        def load_atlas():
            with atlas.snapshot(REFERENCE/'NEW_ATLAS.json') as snap:
                if snap['semantic']!=plan['components']['atlas']['replace']:raise ValueError('RetainedAtlasMismatch')
                return {k:snap['view'][k] for k in islice(snap['view'],10001)}
        values,result['source_atlas_validation_s']=timing(load_atlas)
        result['rows']=dict(quality=len(rows),atlas=len(values))
        if len(rows)!=10001 or len(values)!=10001:raise ValueError('InsufficientRetainedWriterRows')
        outputs={}
        for mode in ('baseline','aligned'):
            folder=target/mode;folder.mkdir()
            phases={};outputs[mode]={}
            with ExitStack() as scopes:
                if mode=='aligned':
                    scopes.enter_context(quality_scope(reuse_private_codec=True));scopes.enter_context(atlas_scope())
                else:
                    profile=folder/'writer_metrics/QUALITY_DELTA';profile.mkdir(parents=True)
                    scopes.enter_context(measured_quality_writer(profile,len(rows),adaptive=False))
                    scopes.enter_context(verified_private_codec(rows))
                logical,phases['quality_write_s']=timing(lambda:q.write(folder/'QUALITY_DELTA.json',rows))
                outputs[mode]['quality_logical']=logical
                def quality_read():
                    with q.snapshot(folder/'QUALITY_DELTA.json') as snap:
                        if list(snap['rows'])!=rows:raise ValueError('WrittenQualityDifference')
                        return snap['semantic']
                outputs[mode]['quality_read'],phases['quality_read_s']=timing(quality_read)
            # Baseline private list shortcut deliberately does not accept AtlasRows.
            with ExitStack() as scopes:
                if mode=='aligned':
                    scopes.enter_context(quality_scope(reuse_private_codec=True));scopes.enter_context(atlas_scope())
                outputs[mode]['atlas_logical'],phases['atlas_write_s']=timing(lambda:atlas.write(folder/'NEW_ATLAS.json',values))
                def atlas_read():
                    with atlas.snapshot(folder/'NEW_ATLAS.json') as snap:
                        if dict(snap['view'])!=values:raise ValueError('WrittenAtlasDifference')
                        return snap['semantic']
                for i in range(2):
                    outputs[mode]['atlas_read'],phases['atlas_read_'+str(i)+'_s']=timing(atlas_read)
            # Outside the live witness scope, full original validation remains.
            outputs[mode]['cold_quality'],phases['cold_quality_s']=timing(quality_read)
            outputs[mode]['cold_atlas'],phases['cold_atlas_s']=timing(atlas_read)
            result['phases'][mode]=phases
            outputs[mode]['database_hashes']={n:file_hash(read(folder/(n+'.json'))['payload']['path']) for n in ('QUALITY_DELTA','atlas_rows')}
        for key in ('quality_read','cold_quality','atlas_read','cold_atlas','database_hashes'):
            result['comparisons'][key]=outputs['baseline'][key]==outputs['aligned'][key]
        result['comparisons']['quality_logical']=all(outputs['baseline']['quality_logical'][k]==outputs['aligned']['quality_logical'][k] for k in ('identity','sha256','bytes','count'))
        # Atlas logical return also reports physical container bytes; compare only source-map terms.
        result['comparisons']['atlas_logical']=all(outputs['baseline']['atlas_logical'][k]==outputs['aligned']['atlas_logical'][k] for k in ('sha256','bytes','count'))
        result['outputs']=outputs;result['input_hashes']={str(p):file_hash(p) for p in (plan_path,REFERENCE/'QUALITY_DELTA.json',REFERENCE/'NEW_ATLAS.json')}
    result['seconds']=time.perf_counter()-start;result['peak_working_set']=psutil.Process().memory_info().peak_wset
    result['allocation']=allocation([p for p in target.rglob('*') if p.is_file()]);result['disk']=disk()
    durable_json(OUT/'WRITERS.json',result)
    if not all(result['comparisons'].values()):raise ValueError('WriterAlignmentMismatch')
    print(json.dumps({k:v for k,v in result.items() if k not in ('allocation','outputs')},indent=2),flush=True)

def collect_writers():
    """Validate completed control outputs; do not repeat writes or full inputs.

    The first control finished its writers/readers but its final report compared
    path-dependent envelope sizes. Preserve it and recover only stored facts.
    Lost outer timings are unavailable; SQLite-window timings are retained.
    """
    if (OUT/'WRITERS_RECOVERED.json').exists():raise ValueError('PreserveCollectedWriterEvidence')
    import single_quality_component as q,compact_front_atlas as atlas,psutil
    from check_current_front_encoding import allocation
    import immutable_state_access as access
    target=OUT/'writers';result=dict(scope='Completed retained-row controls; no repeated writer or production stage',comparisons={},outputs={},profiles={})
    started=time.perf_counter()
    with ExitStack() as held:
        for p in target.rglob('*'):
            if p.is_file():held.enter_context(access.guard(p))
        for mode in ('baseline','aligned'):
            folder=target/mode;outputs={}
            with q.snapshot(folder/'QUALITY_DELTA.json') as snap:
                outputs['quality_rows']=list(snap['rows']);outputs['quality_semantic']=snap['semantic']
            with atlas.snapshot(folder/'NEW_ATLAS.json') as snap:
                outputs['atlas_values']=dict(snap['view']);outputs['atlas_semantic']=snap['semantic']
            outputs['quality_logical']=read(folder/'QUALITY_DELTA.json')['logical']
            outputs['atlas_logical']=read(folder/'NEW_ATLAS.json')['logical']
            outputs['database_hashes']={n:file_hash(read(folder/(n+'.json'))['payload']['path']) for n in ('QUALITY_DELTA','atlas_rows')}
            result['outputs'][mode]=outputs
            result['profiles'][mode]={str(p.relative_to(folder)):read(p) for p in (folder/'writer_metrics').rglob('QUALITY_SQL_PROFILE.json')}
        a,b=(result['outputs'][k] for k in ('baseline','aligned'))
        result['comparisons']={k:a[k]==b[k] for k in a}
        result['rows']=dict(quality=len(a['quality_rows']),atlas=len(a['atlas_values']))
        if not all(result['comparisons'].values()) or result['rows']!={'quality':10001,'atlas':10001}:raise ValueError('RecoveredWriterMeaningMismatch')
        for mode in result['outputs']:
            for k in ('quality_rows','atlas_values'):del result['outputs'][mode][k]
        result['artifacts']={str(p):file_hash(p) for p in target.rglob('*') if p.is_file()}
        result['allocation']=allocation([p for p in target.rglob('*') if p.is_file()])
    result['seconds']=time.perf_counter()-started;result['peak_working_set']=psutil.Process().memory_info().peak_wset
    result['missing_measurements']='Outer writer/warm-reader and original cold-input timers lost on reporting exception; no speed claim for these phases'
    result['routine_failure']=dict(code='WriterAlignmentMismatch',cause='Final comparison included path-dependent quality physical_bytes',previous_harness_sha256='92b4ed46b42c1b5353946760d1c5e14e3f434d7b78e1ecb036c1d9e14f6b9ab6',retained=True)
    durable_json(OUT/'WRITERS_RECOVERED.json',result)
    print(json.dumps({k:v for k,v in result.items() if k not in ('artifacts','allocation')},indent=2),flush=True)

def forecast():
    import numpy as np,psutil
    from check_sf2_continuation import existing_state_class,exact_vertex_map,STUDY,PRIOR
    from sealed_front_connection import load_case
    from accepted_trajectory_contract import AcceptedTrajectoryOwner,AcceptedSchedule,DeclaredTarget
    from sdf_cfd_mesher_v4.common.authority import array_hash
    from sdf_cfd_mesher_v4.geometry.front import CommittedFrontState,build_topology
    from sdf_cfd_mesher_v4.forecast.engine import ForecastCache
    path=OUT/'FORECAST.json'
    if path.exists():raise ValueError('PreserveForecastControl')
    tick=time.perf_counter();case=read(PRIOR/'CASES.json')['wing'];wall,faces,surface,request=load_case(case)
    old,selected=existing_state_class().load(STUDY/'planner/selected_state');mapping=exact_vertex_map(old.wall,wall)
    record=read(V4/'results/sf2_continuation_binding/target_check/DIAGNOSTIC_REQUEST.json')
    positions=old.positions(selected)[:,mapping];owner=AcceptedTrajectoryOwner.__new__(AcceptedTrajectoryOwner)
    owner.layer=6;owner.geometry_identity=case['geometry_identity'];owner.source_identity='RetainedAcceptedFrontFixtureOnly';owner.seconds=0.
    owner.state=SimpleNamespace(vectors=old.vectors[:,mapping]);owner.schedule=AcceptedSchedule(record,selected,mapping)
    owner.schedule.targets=tuple(DeclaredTarget(owner.source_identity,'fixture',k) for k in range(1,21))
    owner.positions=lambda:positions;owner.check=lambda:None
    ancestry=[[] for _ in wall]
    for i,face in enumerate(faces):
        for v in face:ancestry[int(v)].append(i)
    ids=tuple('layer:6:source:'+str(i) for i in range(len(wall)))
    front=CommittedFrontState.create(6,positions[6],ids,faces,ancestry,np.zeros(len(wall)),case['geometry_identity'],case['surface_identity'])
    top=build_topology(faces,len(wall));result=dict(scope='Retained AcceptedFrontState fixture: actual forecast operation, not live admission or new qualified front',vertices=len(wall),edges=len(top.edges),triangles=len(faces),setup_s=time.perf_counter()-tick,phases={},comparisons={})
    full,result['phases']['full_14_layers_s']=timing(lambda:ForecastCache().forecast(front,top,owner.schedule,owner))
    short,result['phases']['next_layer_s']=timing(lambda:ForecastCache().forecast(front,top,owner.schedule,owner,last_layer=7))
    result['layers']=dict(full=full.layers.tolist(),next=short.layers.tolist())
    result['array_bytes']={n:sum(a.nbytes for g in ('vertex','edge','triangle') for a in getattr(f,g).values()) for n,f in (('full',full),('next',short))}
    result['arrays']={}
    with np.load(REFERENCE/'FORECAST.npz',allow_pickle=False) as retained:
        for g in ('vertex','edge','triangle'):
            for n,value in getattr(short,g).items():
                key=g+'__'+n
                if value.tobytes()!=getattr(full,g)[n][:1].tobytes() or value.tobytes()!=retained[key][:1].tobytes():raise ValueError('ChangedForecastArray',key)
                result['arrays'][key]=array_hash(value)
    result['comparisons']=dict(all_arrays_bit_exact_full_prefix=True,all_arrays_bit_exact_retained_prefix=True,future_layers_not_evaluated=True)
    result['hard_events']=short.hard_events();result['peak_working_set']=psutil.Process().memory_info().peak_wset;result['seconds']=time.perf_counter()-tick
    result['input_hashes']={str(REFERENCE/'FORECAST.npz'):file_hash(REFERENCE/'FORECAST.npz')}
    durable_json(path,result);print(json.dumps({k:v for k,v in result.items() if k!='arrays'},indent=2),flush=True)

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('mode',choices=('writers','forecast','collect'))
    parser.add_argument('--output',type=Path,default=OUT);args=parser.parse_args()
    OUT=args.output.resolve()
    if not OUT.is_relative_to(V4/'results') or OUT==V4/'results':raise ValueError('InvalidDiagnosticOutputScope')
    OUT.mkdir(parents=True,exist_ok=True)
    {'writers':writers,'forecast':forecast,'collect':collect_writers}[args.mode]()

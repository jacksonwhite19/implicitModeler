"""Explicit retained-core diagnostic CLI. Never fresh admission or publication."""
from pathlib import Path
from contextlib import ExitStack
import argparse,json,sys,time,hashlib,subprocess,os,traceback
import numpy as np
import geometry
ROOT=Path(__file__).resolve().parent

def save(path,value):
    from planner import jsonable
    Path(path).write_text(json.dumps(value,indent=2,default=jsonable,allow_nan=False)+'\n')

def inputs(run):
    return [run/name for name in ('CASE.json','CURRENT.json','MESH.npz','shell/ACCEPTED_SHELL.json',
        'background/PARENT.json','background/PARENT_EXPECTATIONS.json','background/POLICY.json',
        'background/native/generated/NETGEN.npz','background/native/CAP.npz','background/native/OUTER.npz')]

def snapshot(run,check=lambda:None):
    from sdf_cfd_mesher_v4.common.authority import array_hash,identity,file_hash
    from finite_research.direct_contract import validate_policy
    from standalone_background import compare_boundary
    from finite_research.direct_evidence_modules.background_audit import map_join
    from prism_hotpath_v2 import prism_cell
    from background_materialization import build
    from finite_support import collar
    import local_termination_topology as topology
    with np.load(run/'MESH.npz') as z:shell={k:z[k].copy() for k in z.files}
    with np.load(run/'background/native/generated/NETGEN.npz') as z:core={k:z[k].copy() for k in z.files}
    with np.load(run/'background/native/CAP.npz') as z:cap={k:z[k].copy() for k in z.files}
    with np.load(run/'background/native/OUTER.npz') as z:outer={k:z[k].copy() for k in z.files}
    policy=validate_policy(json.loads((run/'background/POLICY.json').read_text()))
    if json.loads((run/'CASE.json').read_text())['background']!=policy:raise ValueError('CapturedPolicyMismatch')
    expected=json.loads((run/'background/PARENT_EXPECTATIONS.json').read_text())
    current=json.loads((run/'CURRENT.json').read_text())
    accepted=json.loads((run/'shell/ACCEPTED_SHELL.json').read_text())
    parent=json.loads((run/'background/PARENT.json').read_text())
    if current['sha256']!=file_hash(run/'MESH.npz'):raise ValueError('CapturedMeshFileMismatch')
    if not accepted['passed'] or not (current['mesh']==accepted['mesh']==expected['binding']['parent_prism_mesh']):raise ValueError('CapturedShellIdentityMismatch')
    if parent!=expected['binding']:raise ValueError('CapturedParentBindingMismatch')
    if file_hash(run/'CASE.json')!=parent['case_sha256']:raise ValueError('CapturedCaseHashMismatch')
    if expected['prefix_points']!=len(shell['points']):raise ValueError('CapturedPrefixPointCountMismatch')
    for k,a in dict(points=shell['points'],prisms=shell['cell_vertices'],prism_ids=shell['cell_ids'],wall=shell['wall'],cap=shell['outer']).items():
        if array_hash(a)!=expected['array_hashes'][k]:raise ValueError('CapturedParentArrayMismatch:'+k)
    check();_,inner,far=compare_boundary(core['points'],core['tets'],cap['points'],cap['triangles'],outer['points'],outer['triangles'])
    lookup={p.tobytes():i for i,p in enumerate(shell['points'])}
    if len(lookup)!=len(shell['points']):raise ValueError('NonuniqueCapturedPrefix')
    front_ids=np.asarray([lookup[p.tobytes()] for p in cap['points']],dtype='<i8')
    if shell['points'][front_ids].tobytes()!=cap['points'].tobytes():raise ValueError('CapturedCapBitMismatch')
    points,mapped,far_faces,_=map_join(shell['points'],front_ids,core['points'],core['tets'],inner,far,outer['triangles'])
    prefix=[]
    for row,col,layer,cid in zip(shell['cell_vertices'],shell['source_columns'],shell['layers'],shell['cell_ids']):
        cell=prism_cell(row.tolist(),int(col),int(layer))
        if cell['id']!=int(cid):raise ValueError('CapturedPrismIdMismatch')
        prefix.append(cell)
    if identity(collar.d.plain(prefix))!=expected['cells_identity']:raise ValueError('CapturedPrismMetadataMismatch')
    prefix_owners,incidence=topology.ownership(prefix,list(map(tuple,shell['outer'])),list(map(tuple,shell['wall'])))
    if not incidence['passed']:raise ValueError('CapturedPrefixIncidence')
    digest=collar.inherited_digest(prefix,prefix_owners,check)
    if digest!=expected['prefix_digest']:raise ValueError('CapturedPrefixDigestMismatch')
    additions=build(mapped,max(c['id'] for c in prefix)+1);cells=prefix+additions
    owners,incidence=topology.ownership(cells,far_faces,list(map(tuple,shell['wall'])))
    if not incidence['passed']:raise ValueError('CapturedJoinedIncidence')
    state=dict(points=points,bulk_tets=mapped,prefix_front=shell['outer'],farfield=np.asarray(far_faces,dtype='<i8'),
        wall=shell['wall'],bounds=np.asarray(policy['bounds_m'],dtype='<f8'),cells=cells,owners=owners,
        prefix_ids=[c['id'] for c in prefix],prefix_points=len(shell['points']),prefix_digest=digest,
        provenance=dict(generation_invocations=0,construction_invocations=0,live_parent=False,
            scope='Retained diagnostic snapshot; original inputs authenticated by stored hashes, not fresh live authority'))
    return state,prefix,prefix_owners,policy,expected

def worker(args):
    from mesh import source_independence_guard
    source_independence_guard()
    from immutable_state_access import guard
    from core_connectivity import repair,Policy
    from sdf_cfd_mesher_v4.common.authority import file_hash,array_hash
    from finite_support import collar
    from finite_research import direct_shared_apex
    from finite_research.direct_evidence_modules import joined_audit
    from finite_support.transition_front_targets import tetrahedron_ratio,population_report
    from finite_research.direct_fill_contract import classify
    args.output.mkdir(parents=True,exist_ok=False);started=time.monotonic();deadline=started+args.timeout_seconds
    def check():
        if time.monotonic()>=deadline:raise TimeoutError('RetainedStudyDeadline')
    with ExitStack() as held:
        for path in inputs(args.input):held.enter_context(guard(path))
        save(args.output/'INPUTS.json',dict(input_root=str(args.input),files={p.relative_to(args.input).as_posix():file_hash(p) for p in inputs(args.input)},
            source_scope='Retained fresh-FULL5 input files; no new surface, source admission, native generation or live lease'))
        state,prefix,prefix_owners,policy,expected=snapshot(args.input,check)
        print('retained_connectivity_repair',flush=True)
        prefix_ids=set(state['prefix_ids']);core_ids=[c['id'] for c in state['cells'] if c['id'] not in prefix_ids]
        repaired,report=repair(state['points'],state['cells'],state['owners'],core_ids,state['farfield'],args.output/'connectivity',
            minimum=policy['quality_targets']['outer_tetrahedra']['minimum'],policy=Policy(),check=check)
        result=dict(classification=report['outcome'],retained_bulk_conditioning_passed=report['construction_passed'],
            retained_joined_diagnostics_passed=False,full_mesh_complete=False,scientific_acceptance=False,live_parent=False,
            publication='NOT_RUN',export='NOT_RUN',native_generation='NOT_RUN',fresh_surface='NOT_RUN',
            topology_edits=len(report['commits']),initial_bad_cells=report['initial_bad_cells'],final_bad_cells=report['final_bad_cells'])
        if report['construction_passed']:
            current_core=[repaired.byid[i] for i in sorted(repaired.core_ids)]
            state.update(points=repaired.points.copy(),cells=prefix+current_core,owners=repaired.owners,
                bulk_tets=np.asarray([c['vertices'] for c in current_core],dtype='<i8'))
            state['provenance'].update(construction_invocations=1,connectivity_policy=report['schema'],original_binding=expected['binding'])
            preservation=collar.verify_inherited_join(state,prefix,prefix_owners,state['prefix_digest'],check)
            arrays=dict(points=state['points'],background_tets=state['bulk_tets'],cap=state['prefix_front'],farfield=state['farfield'],wall=state['wall'],
                prisms=np.asarray([c['vertices'] for c in prefix],dtype='<i8'),tetrahedron_ids=np.asarray([c['id'] for c in current_core],dtype='<i8'))
            print('retained_existing_shared_apex_diagnostic',flush=True)
            try:points,apex=direct_shared_apex.run(arrays,state,policy,check)
            except direct_shared_apex.Refusal as error:
                result.update(downstream_diagnostic_refusal=str(error),seconds=time.monotonic()-started)
                save(args.output/'RESULT.json',result);print(json.dumps(result),flush=True);return
            save(args.output/'SHARED_APEX_DIAGNOSTIC.json',apex);state['points']=points
            if points[:state['prefix_points']].tobytes()!=repaired.points[:state['prefix_points']].tobytes():raise ValueError('ChangedRetainedPrismCoordinates')
            far_ids=np.unique(state['farfield'])
            if points[far_ids].tobytes()!=repaired.points[far_ids].tobytes():raise ValueError('ChangedRetainedFarfieldCoordinates')
            save(args.output/'FINAL_SHAPE.json',classify(population_report(tetrahedron_ratio(points,state['bulk_tets']),policy['quality_targets']['outer_tetrahedra']),policy['quality_targets']['outer_tetrahedra']))
            np.savez_compressed(args.output/'REPAIRED_RETAINED.npz',points=points,tetrahedra=state['bulk_tets'],
                tetrahedron_ids=arrays['tetrahedron_ids'],prisms=arrays['prisms'],cap=state['prefix_front'],farfield=state['farfield'],wall=state['wall'])
            for k in ('points','bulk_tets','prefix_front','farfield','wall','bounds'):state[k].flags.writeable=False
            audit=args.output/'diagnostic_audit';audit.mkdir()
            import standalone_background,tet_layer_adapter
            print('retained_joined_diagnostic_audit',flush=True)
            proof=joined_audit.audit(state,preservation,dict(finite_stage_background=standalone_background,finite_stage_adapter=tet_layer_adapter),collar,check,audit)
            result.update(retained_joined_diagnostics_passed=proof['whole_joined_geometry_passed'],diagnostic_proof=proof,
                actual_core_points=len(points),actual_core_cells=len(current_core),apex_changed_vertices=apex['changed_count'],
                final_points_sha256=array_hash(points))
        check();result['seconds']=time.monotonic()-started;save(args.output/'RESULT.json',result)
        print(json.dumps({k:v for k,v in result.items() if k!='diagnostic_proof'}),flush=True)

def supervise(args,worker_script=None):
    import psutil
    from source_custody import held
    logs=ROOT/'run_logs';logs.mkdir(exist_ok=True);key=hashlib.sha256(str(args.output).encode()).hexdigest()[:12]
    if args.output.exists():raise ValueError('FreshOutputRequired')
    started=time.monotonic();peak=0;reason=None;error=None
    command=[sys.executable,'-B',str(Path(worker_script or __file__).resolve()),*sys.argv[1:],'--worker']
    with held(args.output,logs/(key+'.SOURCES.json')) as source,(logs/(key+'.stdout')).open('w') as out,(logs/(key+'.stderr')).open('w') as err:
        proc=subprocess.Popen(command,stdout=out,stderr=err)
        try:
            while proc.poll() is None:
                if time.monotonic()-started>args.timeout_seconds:reason='TimeLimit';break
                try:
                    parent=psutil.Process(proc.pid);tree=[parent]+parent.children(recursive=True)
                    rss=sum(p.memory_info().rss for p in tree);peak=max(peak,rss)
                    if rss>args.memory_limit_gib*2**30:reason='MemoryLimit';break
                    total=0
                    for p in args.output.rglob('*') if args.output.exists() else ():
                        try:
                            if p.is_file():total+=p.stat().st_size
                        except FileNotFoundError:continue
                    if total>1_000_000_000:reason='OutputLimit';break
                except psutil.NoSuchProcess:
                    if proc.poll() is not None:break
                except Exception as e:reason='MonitorFailure';error=repr(e);break
                time.sleep(.5)
        finally:
            if proc.poll() is None:
                try:subprocess.run(['taskkill','/PID',str(proc.pid),'/T','/F'],capture_output=True,timeout=15)
                except (OSError,subprocess.TimeoutExpired):pass
                if proc.poll() is None:proc.kill()
            code=proc.wait()
    if source.get('state')!='closed' or source.get('passed') is not True:reason='SourceCustodyFailure'
    if code and reason is None:reason='WorkerFailure'
    cleanup=[]
    # Native cavity/export processes use owned Linux groups. Retire matching
    # recorded groups even if their Windows transport was killed or timed out.
    if args.output.exists():
        for scope in args.output.rglob('*.linux_scope.json'):
            try:
                from build_native import linux
                name=scope.name.removesuffix('.linux_scope.json')
                outcome=subprocess.run(['wsl','-d','Ubuntu-22.04','--','python3',linux(ROOT/'cancel_native.py'),linux(scope),linux(scope.parent),linux(ROOT/'native_bin'/name)],capture_output=True,text=True,timeout=15)
                cleanup.append(dict(scope=str(scope),exit_code=outcome.returncode))
                if outcome.returncode:reason='NativeCleanupFailure'
            except Exception as e:cleanup.append(dict(scope=str(scope),error=repr(e)));reason='NativeCleanupFailure'
    if reason and args.output.exists():
        path=args.output/'RESULT.json'
        if path.exists():path.rename(args.output/'RESULT_BEFORE_SUPERVISOR_STOP.json')
        save(path,dict(retained_bulk_conditioning_passed=False,retained_joined_diagnostics_passed=False,full_mesh_complete=False,
            authoritative_supervisor_failure=True,reason=reason,error=error))
    record=dict(command=command,exit_code=code,stop_reason=reason,monitor_error=error,
        source_custody_passed=source['passed'],source_identity=source['identity'],seconds=time.monotonic()-started,cleanup=cleanup,
        peak_windows_tree_rss_bytes=peak,timeout_seconds=args.timeout_seconds,memory_limit_gib=args.memory_limit_gib,output_limit_bytes=1_000_000_000)
    save(logs/(key+'.json'),record);print(json.dumps(record),flush=True)
    return code or (1 if reason else 0)

def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--input',type=Path,required=True);parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--timeout-seconds',type=float,default=6000.);parser.add_argument('--memory-limit-gib',type=float,default=16.)
    parser.add_argument('--worker',action='store_true',help=argparse.SUPPRESS);args=parser.parse_args()
    args.input=args.input.resolve();args.output=args.output.resolve()
    if not np.isfinite([args.timeout_seconds,args.memory_limit_gib]).all() or min(args.timeout_seconds,args.memory_limit_gib)<=0:parser.error('FinitePositiveResourceLimitsRequired')
    if not args.worker:return supervise(args)
    try:worker(args)
    except BaseException as error:
        if args.output.exists():save(args.output/'EXCEPTION.json',dict(type=type(error).__name__,message=str(error),traceback=traceback.format_exc()))
        raise
    return 0

if __name__=='__main__':raise SystemExit(main())

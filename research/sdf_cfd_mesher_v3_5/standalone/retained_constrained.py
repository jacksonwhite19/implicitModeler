"""One retained FULL5+validated CORE1 constrained-cavity diagnostic."""
from pathlib import Path
from contextlib import ExitStack
import argparse,json,sys,time,subprocess,traceback
import numpy as np
import geometry
from retained_core import save,snapshot,inputs,supervise
from sdf_cfd_mesher_v4.common.authority import file_hash,array_hash,identity

def replay_core1(state,core1,out,minimum,check):
    from background_topology_repair import CoreTopology,cavity_geometry,key
    from core_connectivity import stage_commit
    from constrained_local import population,CavityTopology
    from finite_research.direct_evidence_modules.collision import retained,verify_retained
    report=json.loads((core1/'connectivity/REPORT.json').read_text())
    prefix=set(state['prefix_ids']);s=CoreTopology(state['points'],state['cells'],state['owners'],set(c['id'] for c in state['cells'])-prefix,state['farfield'])
    ids,rows,_,_=population(s,minimum)
    with np.load(core1/'connectivity/INPUT.npz',allow_pickle=False) as z:
        if z['points'].tobytes()!=s.points.tobytes() or not np.array_equal(z['cell_ids'],ids) or not np.array_equal(z['tetrahedra'],rows):raise ValueError('CORE1InputDoesNotMatchFULL5')
    replay=[]
    for index,commit in enumerate(report['commits']):
        check();record=report['history'][commit['index']];view=s.view(record['proposal'])
        if [dict(id=c['id'],vertices=c['vertices']) for c in view['added']]!=commit['added']:raise ValueError('CORE1ReplacementIdentityMismatch')
        q=s.quality(view)
        from finite_support.transition_front_targets import tetrahedron_ratio
        from finite_research.direct_shared_apex import exact_labels
        new=np.asarray([c['vertices'] for c in view['added']],dtype='<i8')
        if not q['passed'] or np.any(tetrahedron_ratio(s.points,new)<minimum) or not exact_labels(s.points,new,[minimum])[:,0].all():raise ValueError('CORE1QualityReplayRefusal')
        before=cavity_geometry(s.points,view['removed']);after=cavity_geometry(s.points,view['added'])
        if before!=commit['before_geometry'] or after!=commit['after_geometry']:raise ValueError('CORE1ExactGeometryReplayMismatch')
        verify_retained(commit['changed_vs_all'],core1/'connectivity'/('trial_%04d'%commit['index']))
        staged=stage_commit(s,view);faces=[uses[0][1] for _,uses in sorted(staged.owners.items())];activekeys={key(f) for c in view['added'] for f in c['faces']}
        audit=out/('replay_%02d'%index);audit.mkdir()
        proof=retained(s.points,faces,[j for j,f in enumerate(faces) if key(f) in activekeys],check,audit/'contacts')
        if not proof['passed'] or not proof['independent_coverage_passed']:raise ValueError('CORE1ContactReplayRefusal')
        verify_retained(proof,audit);s=staged;replay.append(dict(quality=q,before=before,after=after,changed_vs_all=proof))
    ids,rows,_,bad=population(s,minimum)
    with np.load(core1/'connectivity/RESULT.npz',allow_pickle=False) as z:
        if s.points.tobytes()!=z['points'].tobytes() or not np.array_equal(rows,z['tetrahedra']) or not np.array_equal(ids,z['cell_ids']):raise ValueError('CORE1FinalReplayMismatch')
    if bad!=report['final_bad_cells']:raise ValueError('CORE1FinalFloorReplayMismatch')
    save(out/'CORE1_REPLAY.json',dict(passed=True,commits=replay,remaining_bad_cells=bad,original_report_sha256=file_hash(core1/'connectivity/REPORT.json')))
    return CavityTopology(s.points,list(s.byid.values()),s.owners,s.core_ids,state['farfield'])

def reconstructed(original,archive):
    """Rebuild from arrays and original prism metadata, never saved core faces."""
    from background_materialization import build
    import local_termination_topology as topology
    with np.load(archive,allow_pickle=False) as z:a={k:z[k].copy() for k in z.files}
    if a['points'][:original['prefix_points']].tobytes()!=original['points'][:original['prefix_points']].tobytes():raise ValueError('RetainedPrefixMoved')
    for name,key in [('wall','wall'),('farfield','farfield'),('cap','prefix_front')]:
        if not np.array_equal(a[name],original[key]):raise ValueError('RetainedConstrainedFacetChanged')
    far=np.unique(original['farfield'])
    if a['points'][far].tobytes()!=original['points'][far].tobytes():raise ValueError('RetainedFarfieldMoved')
    prefix_ids=set(original['prefix_ids']);prefix=[c for c in original['cells'] if c['id'] in prefix_ids]
    if not np.array_equal(a['prisms'],np.asarray([c['vertices'] for c in prefix])):raise ValueError('RetainedPrismConnectivityChanged')
    core=[]
    for row,cid in zip(a['tetrahedra'],a['tetrahedron_ids']):core+=build(np.asarray([row],dtype='<i8'),int(cid))
    ids=[c['id'] for c in prefix+core]
    if len(ids)!=len(set(ids)) or len(a['tetrahedra'])!=len(a['tetrahedron_ids']):raise ValueError('RetainedCellIdentityMismatch')
    cells=prefix+core;owners,incidence=topology.ownership(cells,list(map(tuple,a['farfield'])),list(map(tuple,a['wall'])))
    if not incidence['passed']:raise ValueError('RetainedFinalIncidenceRefusal')
    result=dict(original,points=a['points'],cells=cells,owners=owners,bulk_tets=a['tetrahedra'],
        provenance=dict(live_parent=False,generation_invocations=0,construction_invocations=1,scope='Retained local cavity diagnostic'))
    return result,prefix,a

def geometry_checks(state,prefix,prefix_owners,policy,folder,check):
    from finite_support import collar
    from constrained_local import population,CavityTopology
    from finite_research.direct_evidence_modules import joined_audit
    import standalone_background,tet_layer_adapter
    ids=set(state['prefix_ids']);s=CavityTopology(state['points'],state['cells'],state['owners'],[c['id'] for c in state['cells'] if c['id'] not in ids],state['farfield'])
    _,_,_,bad=population(s,policy['quality_targets']['outer_tetrahedra']['minimum'])
    if bad:raise ValueError('RetainedFinalBulkFloorRefusal')
    preservation=collar.verify_inherited_join(state,prefix,prefix_owners,state['prefix_digest'],check)
    for k in ('points','bulk_tets','prefix_front','farfield','wall','bounds'):state[k].flags.writeable=False
    folder.mkdir()
    proof=joined_audit.audit(state,preservation,dict(finite_stage_background=standalone_background,finite_stage_adapter=tet_layer_adapter),collar,check,folder)
    return proof

def cold_readback(args):
    """Independent process reconstruction and fresh retained geometric audit."""
    from immutable_state_access import guard
    from export_mesh import export
    manifest=args.output/'READBACK_INPUT.json'
    if file_hash(manifest)!=args.readback:raise ValueError('ChangedRetainedReadbackManifest')
    m=json.loads(manifest.read_text());deadline=time.monotonic()+args.timeout_seconds
    def check():
        if time.monotonic()>=deadline:raise TimeoutError('RetainedReadbackDeadline')
    with ExitStack() as held:
        held.enter_context(guard(manifest))
        for rootname,files in ((args.input,m['original_inputs']),(args.output,m['files'])):
            for name,pin in files.items():
                p=(rootname/name).resolve()
                if not p.is_relative_to(rootname.resolve()) or file_hash(p)!=pin:raise ValueError('ChangedRetainedReadbackFile')
                held.enter_context(guard(p))
        original,prefix,prefix_owners,policy,_=snapshot(args.input,check)
        state,prefix,arrays=reconstructed(original,args.output/'REPAIRED_RETAINED.npz')
        proof=geometry_checks(state,prefix,prefix_owners,policy,args.output/'readback_audit',check)
        if not proof['whole_joined_geometry_passed']:raise ValueError('RetainedReadbackJoinedRefusal')
        # A separate process repeats the unchanged export/conversion-fidelity
        # and Foundation readback contract against the authenticated archive.
        export_result=export(args.output/'REPAIRED_RETAINED.npz',args.output/'readback_export',_deadline=deadline)
        save(args.output/'READBACK.json',dict(passed=True,independent_process=True,all_twelve_recomputed=True,
            repeated_conversion_and_native_readback=export_result,points=len(arrays['points']),prisms=len(arrays['prisms']),tetrahedra=len(arrays['tetrahedra']),
            fresh_admission=False,scientific_acceptance=False,live_parent=False))

def worker(args):
    from mesh import source_independence_guard
    source_independence_guard()
    from immutable_state_access import guard
    from constrained_local import repair,Policy
    from finite_research import direct_shared_apex
    from export_mesh import export
    args.output.mkdir(parents=True,exist_ok=False);start=time.monotonic();deadline=start+args.timeout_seconds
    def check():
        if time.monotonic()>=deadline:raise TimeoutError('RetainedConstrainedDeadline')
    with ExitStack() as held:
        source_inputs=inputs(args.input);prior=[p for p in args.core1.rglob('*') if p.is_file()]
        for p in source_inputs+prior:held.enter_context(guard(p))
        save(args.output/'INPUTS.json',dict(original_root=str(args.input),original_files={p.relative_to(args.input).as_posix():file_hash(p) for p in source_inputs},
            core1_root=str(args.core1),core1_files={p.relative_to(args.core1).as_posix():file_hash(p) for p in prior}))
        original,prefix,prefix_owners,policy,_=snapshot(args.input,check)
        minimum=policy['quality_targets']['outer_tetrahedra']['minimum']
        print('revalidate_CORE1',flush=True);baseline=replay_core1(original,args.core1,args.output,minimum,check)
        print('bounded_constrained_local_cavity',flush=True)
        repaired,report=repair(baseline,args.output/'constrained',minimum=minimum,policy=Policy(),check=check)
        result=dict(classification=report['outcome'],retained_bulk_conditioning_passed=report['construction_passed'],retained_joined_diagnostics_passed=False,
            retained_export_passed=False,independent_retained_readback_passed=False,full_mesh_complete=False,scientific_acceptance=False,live_parent=False,
            initial_bad_cells=report['initial_bad_cells'],final_bad_cells=report['final_bad_cells'],topology_edits=len(report['commits']),
            fresh_authority_gaps=['No fresh SDF surface/source or native prism construction admission','No genuine live background lease','No fresh complete CLI/campaign or publication authority'],
            fresh_surface='NOT_RUN',native_prism_generation='NOT_RUN',full_background_generation='NOT_RUN',
            local_cavity_native_calls=report['native_calls'],publication='NOT_RUN')
        save(args.output/'RESULT.json',result)
        if report['construction_passed']:
            core=[repaired.byid[i] for i in sorted(repaired.core_ids)]
            state=dict(original,points=repaired.points.copy(),bulk_tets=np.asarray([c['vertices'] for c in core],dtype='<i8'),cells=prefix+core,owners=repaired.owners,
                provenance=dict(live_parent=False,generation_invocations=0,construction_invocations=1,scope='Retained constrained cavity'))
            arrays=dict(points=state['points'],background_tets=state['bulk_tets'],tetrahedron_ids=np.asarray([c['id'] for c in core],dtype='<i8'),cap=state['prefix_front'],farfield=state['farfield'],wall=state['wall'],prisms=np.asarray([c['vertices'] for c in prefix],dtype='<i8'))
            print('retained_shared_apex',flush=True)
            try:points,apex=direct_shared_apex.run(arrays,state,policy,check)
            except direct_shared_apex.Refusal as e:
                result['downstream_refusal']=str(e);save(args.output/'RESULT.json',result);return
            save(args.output/'SHARED_APEX.json',apex)
            archive=args.output/'REPAIRED_RETAINED.npz'
            np.savez_compressed(archive,points=points,tetrahedra=state['bulk_tets'],tetrahedron_ids=np.asarray([c['id'] for c in core],dtype='<i8'),
                prisms=arrays['prisms'],cap=state['prefix_front'],wall=state['wall'],farfield=state['farfield'])
            state,_,_=reconstructed(original,archive)
            print('retained_joined_audit',flush=True);proof=geometry_checks(state,prefix,prefix_owners,policy,args.output/'audit',check)
            result['retained_joined_diagnostics_passed']=proof['whole_joined_geometry_passed'];save(args.output/'JOINED_PROOF.json',proof)
            if proof['whole_joined_geometry_passed']:
                print('retained_export',flush=True);exported=export(archive,args.output/'export',_deadline=deadline)
                result['retained_export_passed']=exported['passed']
                files=[archive,args.output/'JOINED_PROOF.json']+[p for p in (args.output/'export').rglob('*') if p.is_file()]
                save(args.output/'READBACK_INPUT.json',dict(original_inputs={p.relative_to(args.input).as_posix():file_hash(p) for p in source_inputs},files={p.relative_to(args.output).as_posix():file_hash(p) for p in files}))
                pin=file_hash(args.output/'READBACK_INPUT.json')
                print('independent_retained_readback',flush=True)
                command=[sys.executable,'-B',str(Path(__file__).resolve()),'--input',str(args.input),'--core1',str(args.core1),'--output',str(args.output),'--timeout-seconds',str(max(1,deadline-time.monotonic())),'--readback',pin]
                with (args.output/'readback.stdout').open('w') as out,(args.output/'readback.stderr').open('w') as err:
                    code=subprocess.run(command,stdout=out,stderr=err,timeout=max(1,deadline-time.monotonic())).returncode
                if code:raise ValueError('IndependentRetainedReadbackFailure')
                result['independent_retained_readback_passed']=json.loads((args.output/'READBACK.json').read_text())['passed']
        check();result['seconds']=time.monotonic()-start;save(args.output/'RESULT.json',result);print(json.dumps(result),flush=True)

def main():
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('input','core1','output'):p.add_argument('--'+name,type=Path,required=True)
    p.add_argument('--timeout-seconds',type=float,default=21600.);p.add_argument('--memory-limit-gib',type=float,default=16.)
    p.add_argument('--worker',action='store_true',help=argparse.SUPPRESS);p.add_argument('--readback',help=argparse.SUPPRESS)
    args=p.parse_args()
    for name in ('input','core1','output'):setattr(args,name,getattr(args,name).resolve())
    if not np.isfinite([args.timeout_seconds,args.memory_limit_gib]).all() or min(args.timeout_seconds,args.memory_limit_gib)<=0:p.error('FinitePositiveLimitsRequired')
    if args.readback:
        from mesh import source_independence_guard
        source_independence_guard();cold_readback(args);return 0
    if not args.worker:return supervise(args,worker_script=__file__)
    try:worker(args)
    except BaseException as e:
        if args.output.exists():save(args.output/'EXCEPTION.json',dict(type=type(e).__name__,message=str(e),traceback=traceback.format_exc()))
        raise
    return 0

if __name__=='__main__':raise SystemExit(main())


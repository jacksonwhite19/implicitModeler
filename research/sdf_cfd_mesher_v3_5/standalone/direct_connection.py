"""V3 native prism parent -> existing V4 direct-fill science and engineering QA.

Local orchestration replaces historical campaign/producer bindings only.
No separate finite-row transition is generated. Original V4 algorithms and
DirectFillEngineering/1 hard gates and minimum floors remain authoritative.
"""
from pathlib import Path
from contextlib import ExitStack
from types import SimpleNamespace
import json,sys,time,subprocess,os
import numpy as np
ROOT=Path(__file__).resolve().parent
SCHEMA='V4DirectFillEngineeringPolicy/1'


class NativeBudget:
    """One cumulative wall-time allowance for native workers and export."""
    def __init__(self,seconds,clock=time.monotonic):
        self.limit=float(seconds);self.used=0.;self.clock=clock
    def remaining(self):
        value=self.limit-self.used
        if value<1:raise TimeoutError('V35DirectSharedNativeBudget')
        return value
    def charge(self,started):
        self.used+=self.clock()-started
        if self.used>self.limit:raise TimeoutError('V35DirectSharedNativeBudgetOverrun')


def policy(bounds,growth,maximum):
    from finite_research.direct_contract import METHOD,validate_policy
    from finite_research.direct_fill_contract import VERSION,TARGETS,contract_identity
    result=dict(schema=SCHEMA,backend_method=METHOD,bounds_m=bounds,growth_per_m=growth,maximum_size_m=maximum,
        interface_seed_policy='balanced_centroid_volume/1',quality_targets=TARGETS,downstream_seconds=10800,native_seconds=7200,
        geometry_assurance='all_twelve_fresh',continuous_sdf_fluid='NOT_PERFORMED',acceptance_contract_version=VERSION,acceptance_contract_identity=contract_identity())
    return validate_policy(result)


def cap_compatibility(points,triangles,selected):
    """Replay V4's mandatory preserved-cap floor before expensive bulk work.

    The downstream route never moves, splits or replaces this cap. Therefore
    its existing final minimum has the same inputs at this earlier boundary.
    Population percentages retain their nonblocking engineering role.
    """
    from finite_research.direct_contract import validate_policy
    from finite_research.direct_fill_contract import classify
    from finite_support.transition_front_targets import triangle_ratio,population_report
    from sdf_cfd_mesher_v4.common.authority import array_hash,identity
    validate_policy(selected)
    scores=triangle_ratio(points,triangles)
    if not len(scores) or not np.isfinite(scores).all() or np.any(scores<=0):raise ValueError('NonpositiveOrNonfinitePrismCapShape')
    target=selected['quality_targets']['outer_triangles'];report=classify(population_report(scores,target),target)
    failed=np.flatnonzero(scores<target['minimum'])
    return dict(schema='V35V4CapCompatibility/1',passed=report['minimum_passed'],population=report,
        below_minimum=len(failed),failed_triangle_indices=failed.tolist(),points_sha256=array_hash(points),triangles_sha256=array_hash(triangles),
        policy_identity=identity(selected),gate='Existing V4 preserved-cap minimum; unchanged during direct fill',
        percentage_targets_blocking=False,scientific_acceptance=False)


def _construction():
    # This module supplies the existing inherited-prefix and surface checks;
    # its collar proposal/generation functions are never called.
    from finite_support import collar
    return collar


def _worker(folder,mode,cap,outer,selected,check,trace_files,budget):
    from native import operation
    from build_native import linux
    from standalone_background import probe_backend
    from sdf_cfd_mesher_v4.common.authority import file_hash
    from immutable_state_access import guard
    from path_preserving_deduplication import durable_json as save
    folder.mkdir()
    np.savez(folder/'CAP.npz',**cap)
    if outer is not None:np.savez(folder/'OUTER.npz',**outer)
    names=['CAP.npz']+(['OUTER.npz'] if outer is not None else [])
    request=dict(schema='V35V4DirectWorker/1',mode=mode,policy=selected,files={n:file_hash(folder/n) for n in names},locks=probe_backend())
    from source_custody import selected as source_files
    request['source_files']={p.relative_to(ROOT).as_posix():file_hash(p) for p in source_files(folder)}
    save(folder/'INPUT.json',request);pin=file_hash(folder/'INPUT.json')
    paths=':'.join(linux(ROOT/p) for p in ('safety','v4/runtime/lib/finite_native_packages','v4/tools','v4/src'))
    with ExitStack() as held:
        for name in names+['INPUT.json']:held.enter_context(guard(folder/name))
        check()
        started=budget.clock()
        try:
            operation('/usr/bin/env',['PYTHONPATH='+paths,'PYTHONDONTWRITEBYTECODE=1','python3','-B',linux(ROOT/'direct_worker.py'),linux(folder),pin],folder,
                seconds=max(1,int(min(3600,check.remaining_seconds(),budget.remaining()))),trace_files=trace_files,name='v4_direct_'+mode)
        finally:budget.charge(started)
        check()
        result=json.loads((folder/'RESULT.json').read_text())
        if result.get('passed') is not True or result['request_sha256']!=pin or result['locks']!=request['locks']:raise ValueError('DirectNativeReadbackRefusal')
        for name,digest in result['files'].items():
            target=(folder/name).resolve()
            if not target.is_relative_to(folder.resolve()) or file_hash(target)!=digest:raise ValueError('ChangedDirectNativeArtifact',name)
        return result


def execute(run,case_path,folder,*,trace_files=False,_finalization_scope=None):
    from path_preserving_deduplication import durable_json as save
    from immutable_state_access import guard
    from standalone_case import load
    from standalone_background import _interface,interface_seeds,compare_boundary,check_box
    from background_preparation import held as preparation_scope
    from finite_research.direct_contract import validate_policy
    from finite_research.direct_fill_contract import require_minima,binding as contract_binding
    from finite_research.direct_evidence import terminal_lease,validate_filling
    from finite_research.direct_evidence_modules import background_audit,joined_audit,shape,driver
    from finite_research import direct_repair_custody
    from background_materialization import build
    from sdf_cfd_mesher_v4.common.authority import identity,array_hash,file_hash
    import local_termination_topology as topology
    from artifact_finalization import AuditedBundle,hold_manifest,run_independent_audit
    if _finalization_scope is None:raise ValueError('AuditFinalizationLifetimeRequired')
    case,_,case_pin=load(case_path);selected=validate_policy(case['background'])
    from verification_policy import require_selection,profile,DIRECT,complete_after_finalization
    mode=case.get('verification',{}).get('mode','audit')
    if 'verification' in case:require_selection(case['verification'],case)
    folder=Path(folder);folder.mkdir(exist_ok=False)
    save(folder/'POLICY.json',selected)
    start=time.perf_counter();deadline=time.monotonic()+selected['downstream_seconds'];stage='parent';phase_started=start;timings={}
    budget=NativeBudget(selected['native_seconds'])
    def check():
        if time.monotonic()>=deadline:raise TimeoutError('V35DirectDownstreamDeadline')
    check.deadline=deadline;check.remaining_seconds=lambda:max(0.,deadline-time.monotonic())
    def phase(name):
        nonlocal stage,phase_started
        now=time.perf_counter();timings[stage]=timings.get(stage,0.)+now-phase_started
        stage=name;phase_started=now;check();save(folder/'PROGRESS.json',dict(stage=name,seconds=now-start,stage_seconds=timings,full_mesh_complete=False))
        print('v4.direct.'+name,flush=True)
    try:
        with ExitStack() as held:
            held.enter_context(guard(case_path));held.enter_context(guard(folder/'POLICY.json'))
            preparation=held.enter_context(preparation_scope())
            parent=held.enter_context(run.live_acceptance.background_inputs(run,_preparation=preparation,_capture_volume=True))
            require=held.enter_context(terminal_lease(run,parent,check));view=parent.view
            token,front,tri,ids,volumes,terminal,metadata,centres=_interface(view,_preparation=preparation,_handoff=parent)
            bounds=np.asarray(selected['bounds_m'])
            if np.any(front<=bounds[:,0]) or np.any(front>=bounds[:,1]):raise ValueError('PrismFrontOutsideDirectDomain')
            seed_policy=dict(schema='V4GradedBackground/2',**{k:selected[k] for k in ('bounds_m','growth_per_m','maximum_size_m','interface_seed_policy')})
            cap=dict(points=front,triangles=tri,seeds=interface_seeds(seed_policy,front,tri,volumes,centres))
            phase('cap_compatibility')
            compatible=cap_compatibility(front,tri,selected)
            compatible.update(parent_prism_mesh=token.mesh,native_identity=run.provider.native_identity)
            save(folder/'CAP_COMPATIBILITY.json',compatible)
            if compatible['passed'] is not True:raise ValueError('PrismCapBelowV4ShapeMinimum')
            cells=parent.cells();owners={k:list(v) for k,v in view.topology.owners.items()}
            state=dict(points=np.asarray(view.points),cells=cells,owners=owners,cap_ids=ids,cap_faces=tri,outer=np.asarray(view.outer,dtype='<i8'),wall=np.asarray(view.wall,dtype='<i8'),bounds=bounds)
            construction=_construction();prefix_digest=construction.inherited_digest(cells,owners,require)
            # Bind actual realized compressed geometry, not the nominal schedule
            # or historical V4 source-direction producer flags.
            binding=dict(schema='V35V4DirectParent/1',case_sha256=case_pin,parent_prism_mesh=token.mesh,
                native_identity=run.provider.native_identity,actual_positions_sha256=array_hash(run.provider.positions),
                source_identity=run.provider.binding.geometry_identity,policy_identity=identity(selected),interface_identity=metadata['identity'])
            save(folder/'PARENT.json',binding)
            parent_expectations=dict(binding=binding,prefix_points=len(state['points']),prefix_digest=prefix_digest,
                cells_identity=identity(construction.d.plain(cells)),
                array_hashes={name:array_hash(value) for name,value in dict(points=state['points'],prisms=np.asarray([c['vertices'] for c in cells],dtype='<i8'),
                    prism_ids=np.asarray([c['id'] for c in cells],dtype='<i8'),wall=state['wall'],cap=state['outer']).items()})
            save(folder/'PARENT_EXPECTATIONS.json',parent_expectations)
            held.enter_context(guard(folder/'PARENT.json'));held.enter_context(guard(folder/'PARENT_EXPECTATIONS.json'))
            phase('farfield')
            farfield_result=_worker(folder/'farfield','farfield',cap,None,selected,require,trace_files,budget)
            for name in [*farfield_result['files'],'RESULT.json']:held.enter_context(guard(folder/'farfield'/name))
            outer=driver.arrays(folder/'farfield/OUTER.npz',dict(points='<f8',triangles='<i8'),require)
            check_box(outer['points'],outer['triangles'],bounds)
            phase('bulk')
            native_result=_worker(folder/'native','candidate',cap,outer,selected,require,trace_files,budget)
            for name in [*native_result['files'],'RESULT.json']:held.enter_context(guard(folder/'native'/name))
            core=driver.arrays(folder/'native/generated/NETGEN.npz',dict(points='<f8',tets='<i8'),require)
            _,inner,far=compare_boundary(core['points'],core['tets'],front,tri,outer['points'],outer['triangles'])
            points,mapped,far_faces,_=background_audit.map_join(state['points'],ids,core['points'],core['tets'],inner,far,outer['triangles'])
            additions=build(mapped,max(c['id'] for c in cells)+1);all_cells=cells+additions
            joined_owners,incidence=topology.ownership(all_cells,far_faces,list(map(tuple,state['wall'])))
            save(folder/'JOINED_INCIDENCE.json',incidence)
            if not incidence['passed']:raise ValueError('DirectJoinedIncidence')
            preservation=construction.verify_inherited_join(dict(cells=all_cells,owners=joined_owners),cells,owners,prefix_digest,require)
            joined=dict(points=points,bulk_tets=mapped,prefix_front=state['outer'],farfield=np.asarray(far_faces,dtype='<i8'),wall=state['wall'],bounds=bounds,
                cells=all_cells,owners=joined_owners,prefix_ids=[c['id'] for c in cells],prefix_points=len(state['points']),prefix_digest=prefix_digest,
                provenance=dict(binding=binding,generation_invocations=1,live_parent=True))
            phase('bulk_conditioning')
            from background_conditioning import apply as condition_background
            joined,conditioning=condition_background(joined,folder/'conditioning',selected,require,native_budget=budget)
            for path in (folder/'conditioning').rglob('*'):
                if path.is_file():held.enter_context(guard(path))
            points=joined['points'];mapped=joined['bulk_tets'];all_cells=joined['cells'];joined_owners=joined['owners']
            prefix_ids=set(joined['prefix_ids']);additions=[c for c in all_cells if c['id'] not in prefix_ids]
            preservation=construction.verify_inherited_join(joined,cells,owners,prefix_digest,require)
            # Existing repair receipt binds the original native result.
            shutil_result=folder/'native/OPTIMIZER_RESULT.json'
            shutil_result.write_bytes((folder/'native/RESULT.json').read_bytes())
            phase('repair')
            points,repair=direct_repair_custody.apply(folder,joined,state,additions,selected,require)
            joined['points']=points;joined['provenance'].update(repair=repair,construction_invocations=1)
            for name in ('points','bulk_tets','prefix_front','farfield','wall','bounds'):joined[name].flags.writeable=False
            saved=dict(points=points,background_tets=mapped,farfield=joined['farfield'],prisms=np.asarray([c['vertices'] for c in cells],dtype='<i8'),
                prism_ids=np.asarray([c['id'] for c in cells],dtype='<i8'),tetrahedron_ids=np.asarray([c['id'] for c in additions],dtype='<i8'),wall=state['wall'],cap=state['outer'])
            np.savez(folder/'JOINED_EVIDENCE.npz',**saved)
            save(folder/'JOINED_RECONSTRUCTION.json',dict(cell_metadata=[construction.d.plain({k:v for k,v in c.items() if k not in ('id','vertices')}) for c in all_cells],provenance=joined['provenance']))
            def publish(name,value):save(folder/(name+'.json'),value)
            populations=shape.diagnostics(state,joined,selected,cap,require,publish);require_minima(populations,selected)
            phase('joined_proofs')
            import standalone_background,tet_layer_adapter
            helpers=dict(finite_stage_background=standalone_background,finite_stage_adapter=tet_layer_adapter)
            audit=folder/'audit';audit.mkdir()
            proof=joined_audit.audit(joined,preservation,helpers,construction,require,audit)
            if not proof['whole_joined_geometry_passed']:raise ValueError('DirectWholeGeometryProofFailed',proof.get('failed_stage'))
            bundle=AuditedBundle(folder,joined,proof,_finalization_scope,require,verification=mode)
            from finite_research.direct_evidence_modules.collision import verify_retained
            for name in ('boundary_collision','collision'):verify_retained(driver.read(audit/(name.upper()+'.json')),audit)
            validate_filling(driver.read(audit/'FILLING.json'),all_cells)
            joined_audit.readback(audit,proof['proof_receipts'],proof['geometry_identity'])
            phase('export')
            archive_pin=bundle.write_complete(joined)
            from export_mesh import export
            native_started=budget.clock()
            try:exported=export(folder/'COMPLETE_MESH.npz',folder/'export',trace_files=trace_files,_deadline=time.monotonic()+min(budget.remaining(),require.remaining_seconds()),
                _custody=_finalization_scope,_expected_archive_sha256=archive_pin)
            finally:budget.charge(native_started)
            if not exported['passed']:raise ValueError('DirectNativeExportRefusal')
            bundle.accept_export(exported)
            direct_repair_custody.verify(folder,dict(prefix_points=len(state['points'])),saved,joined,selected,require)
            require();parent.require_origin(run)
            if load(case_path)[2]!=case_pin:raise ValueError('ChangedDirectCase')
            result=dict(schema='V35V4DirectComplete/1',passed=True,full_mesh_complete=False,provisional=True,
                parent=binding,acceptance_contract=contract_binding(selected),geometry_identity=proof['geometry_identity'],proof=proof,
                points=len(points),prism_cells=len(cells),core_cells=len(additions),conditioning=conditioning,repair=repair,shape=populations,export=exported,
                independent_transition_band=False,continuous_sdf_fluid='NOT_PERFORMED',CFD_qualified=False,restart_supported=False)
            from finite_research.direct_contract import method
            from finite_research.direct_fill_contract import acceptance
            handoff=dict(binding=binding,method_identity=identity(method()),geometry_identity=proof['geometry_identity'],
                proof=proof,shape=populations,shape_applicability=acceptance(populations,selected),acceptance_contract=contract_binding(selected))
            save(folder/'EVIDENCE_HANDOFF.json',handoff)
            result['native_budget']=dict(limit_seconds=budget.limit,used_seconds=budget.used)
            if mode=='production':
                result.pop('full_mesh_complete');result.pop('passed');result['producer_shape_contract']=result.pop('acceptance_contract')
                result.update(schema=DIRECT,verification_policy=mode,acceptance_profile=profile(),audit='not_run',
                    producer_checks_passed=True,geometry_complete=True,artifact_fidelity_passed=True,independent_audit_status='not_run',acceptance_status='provisional')
            save(folder/'PROVISIONAL.json',result)
            if mode=='production':bundle.bind_production(joined,proof,selected,parent_expectations,binding,populations,exported,result,handoff,case_path)
            finalized=bundle.seal(joined,proof)
        # Only after the original live terminal lease closes do independent
        # readback and final successful-only publication take place.
        phase('independent_readback' if mode=='audit' else 'production_finalization')
        hold_manifest(folder,finalized,_finalization_scope)
        result=complete_after_finalization(folder,result,finalized,check.remaining_seconds)
        timings[stage]=time.perf_counter()-phase_started
        result.update(seconds=time.perf_counter()-start,stage_seconds=timings,mesh_sha256=file_hash(folder/'COMPLETE_MESH.npz'),manifest_sha256=file_hash(folder/'MANIFEST.json'))
        save(folder/'RESULT.json',result)
        return result
    except BaseException as error:
        import traceback
        save(folder/'STOP.json',dict(stage=stage,error=repr(error),traceback=traceback.format_exc(),full_mesh_complete=False,no_retry=True,seconds=time.perf_counter()-start))
        raise

"""Isolated finite-collar connection; existing live owners admit and publish."""
import stage_observations as obs
from contextlib import contextmanager
from pathlib import Path
import hashlib, importlib.util, sys, time
import numpy as np
from sdf_cfd_mesher_v4.common.authority import array_hash, frozen
from finite_stage_controls import validate, finite_collar
from path_preserving_deduplication import durable_json
from contextlib import ExitStack
import json, subprocess
from sdf_cfd_mesher_v4.common.authority import identity, file_hash

HERE=Path(__file__).resolve().parent
V4=HERE.parents[2]
CALLER=V4/'tools/finite_support/collar.py'
CALLER_SHA='6468a3898e7f59069604ccfd10fc2b5e12fa8714fcf15de7f21d18d82a1a0597'
PACKET=HERE.parent
NATIVE=V4/'tools/finite_support/native'


@obs.observed('finite.source_pins','source_lock')
def source_pins():
    from finite_research.inputs import package_sources
    values=package_sources()
    for path,digest in values.items():
        if file_hash(Path(path))!=digest:raise ValueError('ChangedFiniteCollarSource',path)
    return values


def native_command(folder):
    def path(p):
        p=str(Path(p).resolve()).replace('\\','/')
        return '/mnt/'+p[0].lower()+p[2:]
    v4=V4
    command=['wsl.exe','-e','env','PYTHONDONTWRITEBYTECODE=1',
        'PYTHONPATH='+':'.join(path(p) for p in [v4/'tools',v4/'src']),
        'python3','-B',path(NATIVE/'backend.py'),path(folder)]
    if obs.enabled() and json.loads((Path(folder)/'REQUEST.json').read_text()).get('stage_observations') is not None:
        command+=['--stage-observations-request',file_hash(Path(folder)/'REQUEST.json')]
    return command


@obs.observed('finite.input_readback','io_serialization')
def verify_inputs(folder,expected,request):
    with np.load(folder/'INPUT.npz',allow_pickle=False) as z:
        if set(z.files)!=set(expected):raise ValueError('ChangedFiniteCollarInputSchema')
        for name,value in expected.items():
            actual=z[name]
            if actual.dtype!=value.dtype or actual.shape!=value.shape or actual.tobytes()!=value.tobytes():
                raise ValueError('ChangedFiniteCollarInputReadback',name)
    if identity(json.loads((folder/'REQUEST.json').read_text()))!=identity(request):
        raise ValueError('ChangedFiniteCollarRequestReadback')


@obs.observed('finite.construct','transition')
def construct(points,cells,owners,ids,triangles,terminal,volumes,centres,policy,bounds,folder,check,parent=None,*,stage_binding=None):
    """Propose and validate numerical geometry; never issue a live credential."""
    from immutable_state_access import guard
    from finite_stage_background import validate_policy,validate_execution_target,interface_seeds,compare_boundary
    from finite_stage_adapter import quality
    from shape_transition_domain import witness,verify
    from shape_transition_builder import envelope_binding
    import native_scope
    native_scope.require_transport();validate_policy(policy);validate_execution_target(policy,True)
    controls=dict(policy['tet_transition']);validate(controls)
    from finite_research.flexible_controls import graded
    if graded(controls):
        from finite_research.graded_command import construct as construct_graded
        return construct_graded(points,cells,owners,ids,triangles,terminal,volumes,centres,
            policy,bounds,folder,check,parent,stage_binding=stage_binding)
    from finite_research.flexible_controls import selected as flexible,identities
    is_flexible=flexible(controls)
    initializer_distance=controls['envelope']['nominal_distance_m'] if is_flexible else controls['maximum_distance_m']
    if not finite_collar(controls):raise ValueError('FiniteCollarProfileRequired')
    if parent is not None:require_live_parent(parent)
    folder=Path(folder);folder.mkdir();started=time.perf_counter();timings={}
    cap=frozen(points[ids],'<f8');triangles=frozen(triangles,'<i8')
    mesh=dict(points=points);metadata=dict(cap_ids=ids,cap_triangles=triangles,terminal_owners=terminal)
    original_hash=array_hash(points);pins=source_pins()
    with ExitStack() as held, reviewed_construction() as construction:
        held.enter_context(guard(V4/'runtime/RELEASE.json'))
        for path in pins:held.enter_context(guard(Path(path)))
        if source_pins()!=pins:raise ValueError('ChangedFiniteCollarSourceManifest')
        inherited=construction.inherited_digest(cells,owners,check)
        tick=time.perf_counter();outer,proposal=construction.propose(cap,triangles,initializer_distance,check)
        if np.any(outer<=bounds[:,0]) or np.any(outer>=bounds[:,1]):raise ValueError('FiniteCollarOutsideBackgroundDomain')
        durable_json(folder/'PROPOSAL.json',proposal)
        boundary=construction.admit_boundary(cap,triangles,outer,initializer_distance,check,folder,
            require_shape_targets=not is_flexible)
        durable_json(folder/'BOUNDARY_ADMISSION.json',boundary);timings['proposal_and_boundary']=time.perf_counter()-tick
        seeds=interface_seeds(policy,cap,triangles,volumes,centres)
        backend=folder/'backend';backend.mkdir();np.savez(backend/'INPUT.npz',cap=cap,faces=triangles,outer=outer,seeds=seeds)
        from finite_research.inputs import native_sources
        request=dict(schema=construction.PRODUCER,source_pins=native_sources(),input_sha256=file_hash(backend/'INPUT.npz'),
            growth_per_m=policy['growth_per_m'],maximum_size_m=policy['maximum_size_m'],seed_policy=policy['interface_seed_policy'],
            maximum_distance_m=initializer_distance,controls_identity=identity(controls),input_points_hash=original_hash,
            prism_centres_hash=array_hash(centres),prism_volumes_hash=array_hash(volumes),seed_hash=array_hash(seeds),
            boundary_admission_sha256=file_hash(folder/'BOUNDARY_ADMISSION.json'),control=False)
        if is_flexible:
            if type(stage_binding) is not dict or parent is None or stage_binding.get('parent_prism_mesh')!=parent.view.live_acceptance.mesh:
                raise ValueError('LiveFlexibleNativeStageBindingRequired')
            request.pop('maximum_distance_m')
            request.update(schema='PrivateFiniteGmshCollar/2',controls=controls,
                initializer_distance_m=initializer_distance,stage_binding=stage_binding,
                stage_binding_identity=identity(stage_binding),source_release_sha256=file_hash(V4/'runtime/RELEASE.json'),
                **identities(controls))
            from finite_support.flexible_native_contract import validate_request
            validate_request(request)
        diagnostic=obs.child_binding(str(backend),finite=True)
        if diagnostic is not None:request['stage_observations']=diagnostic
        durable_json(backend/'REQUEST.json',request)
        inputs={name:file_hash(backend/name) for name in ('INPUT.npz','REQUEST.json')}
        for name in inputs:held.enter_context(guard(backend/name))
        for name,digest in inputs.items():
            if file_hash(backend/name)!=digest:raise ValueError('ChangedFiniteCollarBackendInput',name)
        verify_inputs(backend,dict(cap=cap,faces=triangles,outer=outer,seeds=seeds),request)
        check();tick=time.perf_counter()
        with (backend/'stdout.txt').open('x') as out,(backend/'stderr.txt').open('x') as err:
            with obs.span('native_transport_start_wait','child_wait'):
                proc=subprocess.run(native_command(backend),stdout=out,stderr=err,timeout=max(.001,check.remaining_seconds()))
        timings['native_process']=time.perf_counter()-tick;check()
        held.enter_context(guard(backend/'BACKEND.json'));receipt=json.loads((backend/'BACKEND.json').read_text())
        from finite_support.retained_native_contract import reject_production
        reject_production(receipt)
        if proc.returncode or not receipt.get('passed'):raise ValueError('FiniteCollarBackendFailure',receipt.get('error'),proc.returncode)
        if is_flexible:
            from finite_support.flexible_native_contract import validate_receipt
            validate_receipt(receipt,request,inputs['REQUEST.json'])
        if receipt['loaded_source_closure']['unpinned']:raise ValueError('UnpinnedFiniteCollarNativeSource')
        if set(receipt['files'])!={'CORE.npz','OUTER.npz','GMSH.npz','NETGEN.npz'}:raise ValueError('IncompleteFiniteCollarBackendReadback')
        for name,digest in receipt['files'].items():
            held.enter_context(guard(backend/name))
            if file_hash(backend/name)!=digest:raise ValueError('ChangedFiniteCollarBackendOutput',name)
        for name,digest in inputs.items():
            if file_hash(backend/name)!=digest:raise ValueError('ChangedFiniteCollarBackendInput',name)
        tick=time.perf_counter()
        with np.load(backend/'NETGEN.npz',allow_pickle=False) as z:p,t=z['points'],z['tets']
        _,inner,far=compare_boundary(p,t,cap,triangles,outer,triangles)
        state,attachment=construction.materialize(mesh,metadata,cells,owners,p,t,inner,far,check)
        state.update(bounds=bounds,controls=controls,outer_vertex_ids=state['backend_to_global'][far])
        if is_flexible:state['producer']='PrivateFiniteGmshCollar/2'
        flexible_result=None
        if is_flexible:
            from finite_support.flexible_geometry import construct as construct_flexible
            flexible_result=construct_flexible(state,construction,folder,check,quality,parent,
                min(controls['seconds'],check.remaining_seconds()))
        selected={c['id'] for c in state['additions']}|set(map(int,terminal))
        q=quality(state['points'],state['cells'],state['owners'],selected,state['outer'],parent)
        shape=construction.TARGET.evaluate(state if is_flexible else dict(state,controls=dict(quality_targets=construction.d.TARGETS)))
        fixed=set(state['backend_to_global'][np.r_[inner,far]])
        if is_flexible:fixed=set(range(len(state['points'])))-set(map(int,state['outer_vertex_ids']))
        all_fixed=[c['id'] for c in state['additions'] if set(c['vertices'])<=fixed]
        durable_json(folder/'QUALITY.json',dict(q,selected_cells=sorted(selected),all_fixed_tets=all_fixed))
        durable_json(folder/'SHAPE.json',shape);timings['mapping_quality_shape']=time.perf_counter()-tick
        placement=dict(passed=q['passed'] and shape['passed'],quality=q,shape=shape,controls_identity=identity(controls),
            points_sha256=array_hash(state['points']),tets_sha256=array_hash(state['tets']),
            original_prism_points_sha256=original_hash,parent_prism_mesh=None if parent is None else parent.view.live_acceptance.mesh,
            backend_receipt_sha256=file_hash(backend/'BACKEND.json'),
            original_and_9df_benchmark_comparisons='Separate retained-data study obligations; not generic geometry authority',
            source_manifest_sha256=file_hash(V4/'runtime/RELEASE.json'),outer_bulk_neighbours_required=True)
        placement['fluid_domain_evidence']=dict(backend_receipt_sha256=placement['backend_receipt_sha256'],
            backend_outputs=dict(receipt['files']),boundary_receipts={name:file_hash(folder/name) for name in
                ('BOUNDARY_ADMISSION.json','BOUNDARY_COLLISION.json','BOUNDARY_DEGREE.json')})
        if is_flexible:
            placement.update(flexible_result)
            placement['fluid_domain_evidence'].update(q0=flexible_result['q0'],q1=flexible_result['q1'],
                native_request_sha256=inputs['REQUEST.json'],native_input_sha256=inputs['INPUT.npz'],
                final_boundary_receipts=flexible_result['final_boundary_receipts'],
                extent_policy_identity=flexible_result['extent_policy_identity'],
                quality_targets_identity=flexible_result['quality_targets_identity'])
        else:placement['maximum_distance_m']=controls['maximum_distance_m']
        if placement['passed'] and not is_flexible:
            tick=time.perf_counter();compact=state['points'][state['compact_ids']]
            changed,w,envelope=witness(compact,cap,triangles,controls['maximum_distance_m'],check)
            if changed.tobytes()!=compact.tobytes():raise ValueError('EnvelopeWouldChangeBackendCoordinates')
            state['vertex_envelope']=w
            placement['envelope']=verify(compact,cap,triangles,controls['maximum_distance_m'],w,check)
            placement['vertex_envelope_binding']=envelope_binding(state);timings['envelope']=time.perf_counter()-tick
        elif not placement['passed']:
            # Preserve the actual rejection. Admission must stop before fluid/exact work.
            state['vertex_envelope']={};placement['envelope']='not reached after quality/shape rejection'
        placement['inherited_join']=construction.verify_inherited_join(state,cells,owners,inherited,check)
        if array_hash(points)!=original_hash or state['points'][:len(points)].tobytes()!=points.tobytes():raise ValueError('ChangedFiniteCollarPrismPrefix')
        for name,digest in receipt['files'].items():
            if file_hash(backend/name)!=digest:raise ValueError('ChangedFiniteCollarBackendOutput',name)
        if source_pins()!=pins:raise ValueError('ChangedFiniteCollarSourceManifest')
        check();placement.update(seconds=time.perf_counter()-started,timings=timings)
        actual_points,actual_tets=(state['points'],state['tets']) if is_flexible else (p,t)
        volumes6=np.abs(np.einsum('ij,ij->i',actual_points[actual_tets[:,1]]-actual_points[actual_tets[:,0]],
            np.cross(actual_points[actual_tets[:,2]]-actual_points[actual_tets[:,0]],actual_points[actual_tets[:,3]]-actual_points[actual_tets[:,0]])))
        placement['tet_volume_distribution_m3']=distribution(volumes6/6)
        edges=np.unique(np.sort(np.concatenate([triangles[:,[0,1]],triangles[:,[1,2]],triangles[:,[2,0]]]),axis=1),axis=0)
        actual_outer=state['points'][state['outer_vertex_ids']]
        placement['outer_edge_distribution_m']=distribution(np.linalg.norm(actual_outer[edges[:,0]]-actual_outer[edges[:,1]],axis=1))
        placement['sizing']=dict(growth_per_m=policy['growth_per_m'],maximum_size_m=policy['maximum_size_m'],
            seed_policy=policy['interface_seed_policy'],seed_sha256=array_hash(seeds),law='linear distance slope; not adjacent-cell ratio',maximum_size_is_boundary_edge_promise=False)
        durable_json(folder/'NUMERICAL_RESULT.json',placement)
        check()
        return state,placement


@contextmanager
def produce(parent,ids,triangles,terminal,volumes,centres,policy,bounds,folder,resources,*,outer_check=None,stage_binding=None):
    from finite_stage_handoff import produce as existing_produce
    require_live_parent(parent)
    with existing_produce(parent,ids,triangles,terminal,volumes,centres,policy['tet_transition'],bounds,folder,resources,_finite_policy=policy,_outer_check=outer_check,_stage_binding=stage_binding) as lease:
        yield lease


def require_live_parent(parent):
    from rolling_fast_state import BackgroundInputs
    if type(parent) is not BackgroundInputs:
        raise ValueError('LiveAcceptedTetPrismInputsRequired')
    parent.require(parent.view)
    parent.require_origin(parent._open()['origin'])
    return parent.view


@contextmanager
def reviewed_construction():
    from immutable_state_access import guard
    with guard(CALLER):
        if hashlib.sha256(CALLER.read_bytes()).hexdigest()!=CALLER_SHA:
            raise ValueError('ChangedReviewedFiniteCollarConstruction')
        paths=list(sys.path)
        try:
            spec=importlib.util.spec_from_file_location('finite_collar_reviewed_proposer',CALLER)
            module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
            yield module
        finally:
            sys.path[:]=paths
            if hashlib.sha256(CALLER.read_bytes()).hexdigest()!=CALLER_SHA:
                raise ValueError('ChangedReviewedFiniteCollarConstruction')


def distribution(values):
    values=np.asarray(values,dtype=float)
    if values.size==0 or not np.isfinite(values).all():raise ValueError('InvalidFiniteCollarDistribution')
    return dict(count=len(values),quantiles=[0,.01,.5,.99,1],values=np.quantile(values,[0,.01,.5,.99,1]).tolist())


def prepare(points,ids,triangles,terminal,volumes,centres,policy,bounds,check):
    """Actual generic proposal and sizing preparation; no saved fixture loading."""
    from finite_stage_background import validate_policy,validate_execution_target,interface_seeds
    validate_policy(policy);validate_execution_target(policy,True)
    controls=policy['tet_transition'];validate(controls)
    from finite_research.flexible_controls import graded
    if graded(controls):
        from finite_research.graded_command import prepare as prepare_graded
        return prepare_graded(points,ids,triangles,terminal,volumes,centres,policy,bounds,check)
    from finite_research.flexible_controls import selected as flexible
    initializer_distance=controls['envelope']['nominal_distance_m'] if flexible(controls) else controls['maximum_distance_m']
    if not finite_collar(controls):raise ValueError('FiniteCollarProfileRequired')
    cap=frozen(points[ids],'<f8');triangles=frozen(triangles,'<i8')
    if len(terminal)!=len(triangles):raise ValueError('IncompleteFiniteCollarTerminalOwners')
    check()
    with reviewed_construction() as construction:
        outer,proposal=construction.propose(cap,triangles,initializer_distance,check)
    if np.any(outer<=bounds[:,0]) or np.any(outer>=bounds[:,1]):
        raise ValueError('FiniteCollarOutsideBackgroundDomain')
    seeds=interface_seeds(policy,cap,triangles,volumes,centres)
    edges=np.unique(np.sort(np.concatenate([triangles[:,[0,1]],triangles[:,[1,2]],triangles[:,[2,0]]]),axis=1),axis=0)
    pts=outer[triangles]
    areas=np.linalg.norm(np.cross(pts[:,1]-pts[:,0],pts[:,2]-pts[:,0]),axis=1)/2
    check()
    return dict(schema='V4FiniteCollarCommandPreparation/2' if flexible(controls) else 'V4FiniteCollarCommandPreparation/1',scientific_acceptance=False,
        restart_supported=False,**(dict(envelope=controls['envelope']) if flexible(controls) else dict(maximum_distance_m=controls['maximum_distance_m'])),
        distance_scope=('Nominal sizing guide; no hard extent limit. Paired displacement and supplied-anchor diagnostics are separate.'
            if flexible(controls) and controls['envelope']['mode']=='nominal'
            else 'Cap-relative vertex displacement bound; not uniform normal thickness or an every-cell tube'),
        volume_sizing=dict(growth_per_m=policy['growth_per_m'],maximum_size_m=policy['maximum_size_m'],
            growth_units='metres of target size per metre of point-to-triangle distance (dimensionless slope)',
            law='min(maximum_size_m, min_i(seed_i + growth_per_m * distance(point, triangle_i)))',
            interface_seed_policy=policy['interface_seed_policy'],seed_sha256=array_hash(seeds),
            seed_distribution_m=distribution(seeds),maximum_size_is_boundary_resolution_promise=False),
        input=dict(points_sha256=array_hash(points),cap_sha256=array_hash(cap),triangles_sha256=array_hash(triangles),
            terminal_owners_sha256=array_hash(np.asarray(terminal,dtype='<i8')),
            volumes_sha256=array_hash(volumes),centres_sha256=array_hash(centres)),
        proposal=proposal,outer_points_sha256=array_hash(outer),one_to_one_cap_topology=True,
        achieved_outer_edges_m=distribution(np.linalg.norm(outer[edges[:,0]]-outer[edges[:,1]],axis=1)),
        achieved_outer_triangle_areas_m2=distribution(areas),outer_resolution_scope='Constructed boundary proposal; tetrahedralization and acceptance not reached',
        source_fluid_acceptance=False,background_invoked=False)



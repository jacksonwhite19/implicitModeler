"""Single case-driven entry to the existing V4 stage owners.

No historical mesh, selected planner schedule, case, or approval ticket is a
fresh execution input. Qualified resume ancestry is an explicit input.
"""
import argparse
import copy
from contextlib import ExitStack
from contextvars import ContextVar
import json
from pathlib import Path
import shutil
import sys
import time
import traceback
import numpy as np
from owned_runtime import V4,BIN,SOURCE,verify as verify_runtime,verify_release
from standalone_case import load,read,bind,hold_inputs,copy_inputs,input_case_path,held_import_sha256
from path_preserving_deduplication import durable_json
from sdf_cfd_mesher_v4.common.authority import identity,file_hash,array_hash

ROOT=V4.parents[1]
_minimum_free_override=ContextVar('minimum_free_override',default=None)
_intermediate_mesh_override=ContextVar('intermediate_mesh_override',default=False)
_seed_query_diagnostics=ContextVar('seed_query_diagnostics',default=False)
_carrier_derivative_diagnostics=ContextVar('carrier_derivative_diagnostics',default=False)
_carrier_assessment=ContextVar('carrier_assessment',default=True)
_surface_assurance=ContextVar('surface_assurance',default='qualification')


def surface_assurance():return _surface_assurance.get()


def carrier_assessment_enabled():
    """Construction choice; final acceptance is always independently required."""
    return _carrier_assessment.get()


def seed_query_diagnostics_enabled():
    """Call-scoped, passive instrumentation; not an acceptance policy."""
    return _seed_query_diagnostics.get()


def carrier_derivative_diagnostics_enabled():
    """Explicit optional carrier diagnostic; adequacy predicates stay required."""
    return _carrier_derivative_diagnostics.get()


def effective_resources(resources):
    """Explicit operational input; never rewrite the authenticated mesh case."""
    minimum=_minimum_free_override.get()
    if minimum is None:return dict(resources)
    if type(minimum) is not int or minimum<=0:raise ValueError('InvalidMinimumFreeBytes')
    return dict(resources,minimum_free_bytes=minimum)


def geometry(case_path):
    from standalone_geometry import geometry as bound_geometry
    return bound_geometry(case_path)


def check_resources(output,resources):
    resources=effective_resources(resources)
    logical=0
    for p in output.rglob('*'):
        try:
            if p.is_file():logical+=p.stat().st_size
        except FileNotFoundError:continue  # Atomic .pending rename, not a lost proof.
    free=shutil.disk_usage(output).free
    if free<resources['minimum_free_bytes'] or (resources['output_limit_bytes'] is not None
            and logical>resources['output_limit_bytes']):raise ValueError('OperationalResourceLimit')
    return dict(logical_bytes=logical,free_bytes=free)


def preflight(case_path,from_layer,through_layer,restart=None):
    with hold_inputs(case_path):
        return _preflight_guarded(input_case_path(case_path),from_layer,through_layer,restart)


def _preflight_guarded(case_path,from_layer,through_layer,restart=None):
    runtime=verify_runtime();case,increments,pin=load(case_path)
    if (type(from_layer) is not int or type(through_layer) is not int or from_layer<0
            or through_layer<from_layer or through_layer>len(increments)):
        raise ValueError('InvalidRequestedLayerRange')
    if (from_layer>0)!=(restart is not None):raise ValueError('ExplicitQualifiedRestartRequired')
    restart_evidence=verify_restart(restart,pin,from_layer) if restart is not None else None
    host,binding=geometry(case_path)
    try:
        admission=binding.inspect()
        if not admission['executable']:raise ValueError('SurfaceAdmissionFailed',admission['issues'])
        from standalone_geometry import branch_evidence,is_scalar
        branch=branch_evidence(binding) if through_layer>0 else None
        enclosure=case.get('enclosure')
        if enclosure is not None and not is_scalar(binding):
            raise ValueError('EnclosureContainmentNotBoundForRepresentation')
        if enclosure is not None and is_scalar(binding):
            d=binding.authority.definition
            bounds=np.array([d['radius'],d['radius'],d['half_length']+d['radius']])
            # This is an exact design-enclosure check, not a core/domain mesh.
            if np.any(-bounds<enclosure['minimum_m']) or np.any(bounds>enclosure['maximum_m']):
                raise ValueError('SourceOutsideEnclosure')
        background=None
        if 'background' in case:
            if through_layer!=len(increments) or through_layer<1 or from_layer!=0:
                raise ValueError('BackgroundRequiresCompletedFreshPrismRange')
            from standalone_background import preflight as core_preflight
            background=core_preflight(binding,case['background'])
        return dict(passed=True,case_sha256=pin,geometry=binding.geometry_identity,
            units=case['geometry']['metres_per_model_unit'],from_layer=from_layer,through_layer=through_layer,
            seed_route=binding.seed_route,admission=admission,trajectory_branch=branch,
            first_height_hex=float(increments[0]).hex(),
            later_increment_quantity='normal_height_m' if is_scalar(binding) else 'path_length_m',
            later_increment_hex=[float(x).hex() for x in increments[1:]],
            runtime=identity(runtime),resources=effective_resources(case['resources']),case_resources=case['resources'],
            standalone_scope='Capability-bound existing Authority operations; unsupported required premises fail closed',
            enclosure=enclosure,background=background,trajectory_semantics=branch['trajectory_semantics'] if branch and 'trajectory_semantics' in branch else 'AcceptedFrontState/path_length_m/1',
            restart_supplied=restart is not None,restart=restart_evidence,mesh_generated=False)
    finally:host.close()


def verify_restart(path,case_pin,layer):
    import immutable_state_access as access
    from composable_state import CommittedState
    from sealed_front_connection import load_case,inspect_front
    with access.guard(path):
        restart=read(path)
        if (restart.get('schema')!='V4MeshRestart/1' or restart['case_sha256']!=case_pin
                or restart['parent']['layer']!=layer or restart['parent'].get('passed') is not True):
            raise ValueError('QualifiedRestartBindingMismatch')
        with access.guard(restart['candidate']),access.guard(restart['inputs']):
            if file_hash(restart['candidate'])!=restart['candidate_sha256'] or file_hash(restart['inputs'])!=case_pin:
                raise ValueError('QualifiedRestartInputMismatch')
            wall,tri,surface,request=load_case(restart['initial']['case'])
            documents=request['declaration']['source_documents']
            if not any(row['sha256']==case_pin and file_hash(row['path'])==case_pin for row in documents):
                raise ValueError('RestartCallerCaseMismatch')
            with CommittedState(restart['parent']['reference']) as base:
                base.check()
                if base.layer!=layer or base.scientific!=restart['parent']['scientific']:
                    raise ValueError('QualifiedRestartStateMismatch')
                case=restart['initial']['case']
                candidate=str(Path(restart['candidate']).resolve())
                if case['sources'].get(candidate)!=restart['candidate_sha256']:
                    raise ValueError('RestartCandidateNotEnrolled')
                node=base
                while node.kind=='recursive':node=node.parent
                if node.scientific!=case['committed'] or node.reference!=restart['initial']['reference']:
                    raise ValueError('RestartCaseAncestryMismatch')
                inspect_front(node,wall,tri,surface,request,node.expected)
                if layer>1:
                    host,binding=geometry(restart['inputs'])
                    try:
                        owner=reconstruct_owner(binding,case,restart['candidate'],restart['inputs'])
                        if owner.source_identity!=restart['trajectory_owner']:
                            raise ValueError('RestartTrajectoryOwnerMismatch')
                        from sealed_front_connection import ScalarOffsetGeometry
                        if type(owner) is ScalarOffsetGeometry:
                            owner.check()
                            inspect_front(base,wall,tri,surface,request,base.expected)
                            # Scalar permission is the authenticated full external
                            # request. Mutable command-range metadata grants nothing.
                            if restart.get('authorization') is not None:
                                raise ValueError('ScalarRestartCannotSupplyTypedAuthorization')
                        else:
                            from accepted_production_contract import verify_committed_ancestry
                            verify_committed_ancestry(base,owner,restart['authorization'])
                    finally:host.close()
                return dict(passed=True,scientific=base.scientific,layer=base.layer,reference=base.reference)


def first_preflight(case_path,surface,request,surface_identity):
    from production_surface_authority import load_production_wall,verify_surface_sidecars
    from production_prism_sizing import inspect_first_layer_inputs,read_request
    x,f,s=load_production_wall(surface)
    case,inc,_=load(case_path);manifest=verify_runtime()
    if s['surface_identity']!=surface_identity:raise ValueError('SealedSurfaceMismatch')
    impl,_=verify_surface_sidecars(surface,s)
    for p,h in {**impl['dependencies'],**impl['binaries']}.items():
        if file_hash(ROOT/p)!=h:raise ValueError('SourceDependencyMismatch:'+p)
    admitted=inspect_first_layer_inputs(surface,request);req=read_request(request,s)
    if req['increments'][0]!=inc[0]:raise ValueError('AuthorizedHeightMismatch')
    if 'later_normal_increments_m_hex' in case['prisms'] and req['increments'].tobytes()!=inc.tobytes():
        raise ValueError('AuthorizedScheduleMismatch')
    if 'later_path_lengths_m_hex' in case['prisms'] and len(req['increments'])!=1:
        raise ValueError('UnsupportedFirstRequestScope')
    from sdf_cfd_mesher_v4.certificate.efficient import SOURCE as native_source,ROOT as native_root
    binary=native_root/'runtime/bin'/('proof_'+file_hash(native_source)+'.dll')
    if not binary.is_file():raise ValueError('ExistingCertificateBinaryRequired')
    sources={p:file_hash(ROOT/p) for p in impl['dependencies']}
    for name in ('live_spline_first_layer.py','production_prism_stack.py','standalone_tool.py',
                 'standalone_case.py','standalone_geometry.py','production_initial_front.py',
                 'production_first_layer_run.py','run_live_sf2_first_layer.py','geometry_family_authority.py'):
        p=V4/'tools'/name;sources[p.relative_to(ROOT).as_posix()]=file_hash(p)
    return dict(passed=True,surface=s,admission=admitted,request_identity=identity(req['declaration']),
        request_sha256=file_hash(request),height_hex=float(inc[0]).hex(),source={'owned_runtime':identity(manifest)},
        dependencies=sources,certificate_binary={binary.relative_to(ROOT).as_posix():file_hash(binary)},
        resources=case['resources'],vertices=len(x),prisms=len(f))


def implementation(binding,case_path):
    manifest=verify_runtime()
    sources={};binaries={}
    for row in manifest['files']:
        path=V4/row['path'];target=sources if path.suffix in ('.py','.rs') else binaries
        target[path.relative_to(ROOT).as_posix()]=row['sha256']
    for name in ('standalone_tool.py','standalone_case.py','surface_execution.py','surface_evidence.py',
                 'production_surface_authority.py','surface_binding.py','canonical_spline_authority.py',
                 'historical_capsule_surface.py','coupled_surface_metric.py','audit_adaptive_surface.py',
                 'standalone_geometry.py','geometry_family_authority.py','general_sdf_binding.py',
                 'general_sdf_authority.py','general_source_contract.py'):
        path=V4/'tools'/name;sources[path.relative_to(ROOT).as_posix()]=file_hash(path)
    if hasattr(binding.qualification,'target'):
        for name in ('operations.py','source_topology.py','regional.py','regional_owner.py','canonical.py','cgal.py'):
            path=V4/'src/sdf_cfd_mesher_v4/sdf'/name
            sources[path.relative_to(ROOT).as_posix()]=file_hash(path)
    return dict(schema='V4StandaloneSurfaceImplementation/1',dependencies=sources,binaries=binaries,
        case_sha256=file_hash(case_path),geometry_identity=binding.geometry_identity,
        source_representation=(binding.authority.spline_packet['spline_identity'] if hasattr(binding.authority,'spline_packet')
            else identity(binding.qualification.target) if hasattr(binding.qualification,'target')
            else identity(binding.authority.definition)),units=binding.units)


def surface_stage(output,case_path,*,profile='archived'):
    from production_adaptive_surface import ProductionAdaptiveSdfSurfaceMesher
    from production_surface_authority import publish_surface,load_production_wall,record_fastprod_qualification,require_fastprod_surface_scope
    from surface_evidence import observe_qualification,scientific_comparison,verify_written
    from unittest.mock import patch
    import surface_execution
    if profile not in ('fastprod','archived'):raise ValueError('UnsupportedSurfaceAssuranceProfile')
    if profile=='archived' and surface_assurance()!='qualification':raise ValueError('NumericalSurfaceCannotRunArchivedQualification')
    if profile=='fastprod':require_fastprod_surface_scope(output/'surface/A/fresh')
    target=output/'surface';target.mkdir()
    timing={};started=time.perf_counter()
    def measured(name,fn):
        t=time.perf_counter()
        try:return fn()
        finally:timing[name]=time.perf_counter()-t
    for label in (('A',) if profile=='fastprod' else ('A','B')):
        # Native spline state is single-profile; both constructions are live and
        # no reference mesh/geometry is passed to either realization.
        folder=target/label;folder.mkdir()
        host,binding=measured(label+'_geometry_binding',lambda:geometry(case_path))
        try:
            writer=surface_execution.write_surface_contract
            def verified_writer(bound,directory,vertices,faces,edges):
                result=writer(bound,directory,vertices,faces,edges)
                verify_written(bound,Path(directory),vertices,faces,edges)
                return result
            with patch.object(surface_execution,'write_surface_contract',verified_writer),observe_qualification(folder,binding):
                measured(label+'_generate_and_qualify',lambda:ProductionAdaptiveSdfSurfaceMesher().generate(binding,folder/'fresh'))
            if label=='A':impl=measured('implementation_inventory',lambda:implementation(binding,case_path))
            elif implementation(binding,case_path)!=impl:raise ValueError('SurfaceImplementationChangedDuringReplay')
        finally:host.close()
    if profile=='archived':
        comparison=measured('scientific_replay_comparison',lambda:scientific_comparison(target/'A',target/'B',seed_route=binding.seed_route))
        durable_json(target/'REPLAY.json',comparison)
        if not comparison['passed']:raise ValueError('RequiredScientificReplayMismatch')
    authority=target/('A/fresh' if profile=='fastprod' else 'B/fresh')
    durable_json(authority/'IMPLEMENTATION.json',impl)
    if profile=='archived':durable_json(authority/'REPLAY.json',comparison)
    shutil.copyfile(case_path,authority/'CASE.json')
    if profile=='fastprod':
        seal=measured('local_qualification_receipt',lambda:record_fastprod_qualification(authority,read(authority/'GENERATION.json'),identity(impl)))
    else:seal=measured('archived_publication',lambda:publish_surface(authority,read(authority/'GENERATION.json'),identity(impl),comparison))
    measured('qualified_wall_readback',lambda:load_production_wall(authority))
    durable_json(target/'TIMING.json',dict(profile=profile,stages=timing,seconds=time.perf_counter()-started,
        replay_performed=profile=='archived'))
    return authority,seal


def first_request(folder,surface,case_path):
    from production_surface_authority import load_production_wall
    from production_prism_sizing import read_request
    _,_,s=load_production_wall(surface);case,inc,pin=load(case_path)
    scalar='later_normal_increments_m_hex' in case['prisms']
    selected=inc if scalar else inc[:1]
    request=dict(schema='V4ExternalPrismRequest/1',kind='DeclaredProductionTestRequest',production_intent=True,
        geometry_identity=s['geometry_identity'],surface_identity=s['surface_identity'],
        fluid_side=case['prisms']['fluid_side'],request_owner='caller',physical_basis=case['purpose'],
        total_thickness_intent=('Explicit scalar normal increments for this analytic source; first stage constructs only one layer.' if scalar
            else 'Exactly one full-normal first layer; later path lengths are a separate typed contract.'),
        source_documents=[dict(path=str(Path(case_path).resolve()),sha256=pin)],
        height_scope='global',scalar_quantity='normal_height_m',requested_layer_count=len(selected),
        increments_m_hex=[float(x).hex() for x in selected],schedule_total_m_hex=float(np.cumsum(selected,dtype=np.float64)[-1]).hex())
    if scalar:request['growth_hex']=case['prisms']['growth_hex']
    durable_json(folder/'REQUEST.json',request)
    return read_request(folder/'REQUEST.json',s)


def initial_stage(output,surface,case_path):
    import run_live_sf2_first_layer as initial
    from production_initial_front import commit_and_enroll
    from production_prism_stack import initial_front_dispatch
    from production_surface_authority import load_production_wall
    folder=output/'first_layer';folder.mkdir();first_request(folder,surface,case_path)
    initial.configure_inputs(folder,surface,folder/'REQUEST.json',load_production_wall(surface)[2]['surface_identity'])
    initial.STANDALONE_INPUTS=case_path
    with initial_front_dispatch() as work:
        pre=initial.preflight();durable_json(folder/'PREFLIGHT_A.json',pre)
        initial.worker('A',True,commit_action=commit_and_enroll)
        durable_json(folder/'INITIAL_PRODUCTION_WORK.json',work)
    return read(folder/'INITIAL_ENROLLMENT.json'),folder/'A/CANDIDATE.npz'


def range_grant(owner,through_layer,case_pin):
    from accepted_production_contract import validate_authorization
    rows=[]
    for layer in range(2,through_layer+1):
        row=owner.schedule.row_record(layer)
        rows.append(dict(layer=layer,quantity='path_length_m',binary64_hex=row['uniform_hex'],
                         value_m=float.fromhex(row['uniform_hex'])) if row['scope']=='global' else row)
    grant=dict(schema='V4MeshRangeAuthorization/1',user_authorized=True,
        diagnostic_contract=identity(owner.record),initial_committed_parent=owner.parent,
        original_request_sha256=owner.record['original_request_file_sha256'],
        original_request_identity=owner.record['original_request_identity'],trajectory_owner=owner.source_identity,
        recovered_schedule=owner.schedule.sha256,geometry=owner.geometry_identity,surface=owner.record['surface'],
        units='m',first_full_normal_height_m=owner.state.first_height,authorized_layers=list(range(2,through_layer+1)),
        increments=rows,authorization='Explicit mesh command range; caller-owned case '+case_pin,
        production_admission_passed=False)
    validate_authorization(grant,identity(grant),owner)
    return grant


def reconstruct_owner(binding,case,candidate,case_path):
    from standalone_geometry import is_scalar
    if is_scalar(binding):
        from sealed_front_connection import ScalarOffsetGeometry
        return ScalarOffsetGeometry(dict(components=[copy_definition(binding)]),binding.geometry_identity)
    from accepted_trajectory_contract import AcceptedTrajectoryOwner
    from sealed_front_connection import load_case
    wall,tri,surface,request=load_case(case)
    with np.load(candidate,allow_pickle=False) as data:first,refs=data['front'],data['face_refs']
    _,inc,pin=load(case_path)
    return AcceptedTrajectoryOwner.from_inputs(binding.authority,wall,first,tri,refs,inc,
        parent=case['committed'],surface=surface['surface_identity'],request=request,case_identity=pin)

def copy_definition(binding):
    import copy
    binding.qualification.verify(binding.authority)
    return copy.deepcopy(binding.authority.definition)


def resume_grant(restart,owner,through_layer,pin):
    from accepted_production_contract import validate_authorization,range_authorization
    grant=restart['authorization'];validate_authorization(grant,identity(grant),owner)
    if through_layer<=grant['authorized_layers'][-1]:return grant
    return range_authorization(grant,restart['parent'],through_layer,owner.schedule,
        authorization='Explicit resume mesh command range; caller-owned case '+pin)


def continuation(output,case_path,enrolled,candidate,through_layer,profile,*,restart=None,retained=None):
    from accepted_trajectory_contract import AcceptedTrajectoryOwner
    from accepted_production_contract import bind_production,range_authorization
    from sealed_front_connection import load_case,build_access_indices,indexed_session
    from composable_state import CommittedState
    from production_prism_stack import session,committed_index_session
    case=enrolled['case'];reference=enrolled['reference']
    wall,tri,surface,request=load_case(case)
    _,inc,pin=load(case_path);host,binding=geometry(case_path)
    try:
        owner=reconstruct_owner(binding,case,candidate,case_path)
        from standalone_geometry import is_scalar
        scalar=is_scalar(binding)
        grant=(None if scalar else range_grant(owner,through_layer,pin) if restart is None or restart['parent']['layer']==1 else resume_grant(
            restart,owner,through_layer,pin))
        if restart is not None:
            if restart['trajectory_owner'] is not None and owner.source_identity!=restart['trajectory_owner']:
                raise ValueError('RestartTrajectoryOwnerMismatch')
            reference=restart['parent']['reference']
        if scalar:
            durable_json(output/'COMMAND_SCOPE.json',dict(schema='V4MeshScalarCommandMetadata/1',
                authoritative=False,case_sha256=pin,requested_through_layer=through_layer,
                geometry=binding.geometry_identity,quantity='normal_height_m',schedule=request['schedule_identity'],
                permission='Authenticated external request; this command metadata grants nothing'))
        else:durable_json(output/'AUTHORIZATION.json',grant)
        with session(profile) as stack:
            with CommittedState(reference) as base:start=base.layer+1
            for layer in range(start,through_layer+1):
                folder=output/f'L{layer}';folder.mkdir()
                with CommittedState(reference) as base,ExitStack() as scopes:
                    if scalar:
                        from sealed_front_connection import bind_forecast,inspect_front
                        current=dict(case,committed=base.scientific)
                        adapter=bind_forecast(base,current,owner)
                        adapter.trajectory_spec=owner.spec
                        inspected=inspect_front(base,wall,tri,surface,request,base.expected)
                    else:adapter,inspected=bind_production(base,case,owner,grant,identity(grant))
                    if base.kind=='native':
                        indices=build_access_indices(base,inspected,output/'indices')
                        _,pairs=scopes.enter_context(indexed_session(base,inspected,indices))
                    else:_,pairs=scopes.enter_context(committed_index_session(base,adapter,None))
                    def observe(name,event,seconds):
                        check_resources(output,load(case_path)[0]['resources'])
                        print(layer,name,event,seconds,flush=True)
                    result=stack.step(base,adapter,folder,pairs,observe=observe,
                        retained=retained if layer==start else None)
                    reference=result['reference']
                    pointer=dict(schema='V4MeshRestart/1',case_sha256=pin,inputs=str(Path(case_path).resolve()),
                        initial=enrolled,candidate=str(Path(candidate).resolve()),candidate_sha256=file_hash(candidate),
                        trajectory_owner=owner.source_identity,authorization=grant,
                        parent=dict(passed=True,layer=layer,scientific=result['scientific'],reference=reference))
                    durable_json(output/'RESTART.json',pointer)
        return through_layer
    finally:host.close()


def execute(case_path,output,from_layer,through_layer,profile,restart_path=None,retained_initial=None,retained=None):
    """Pin caller inputs and installed release for the entire execution."""
    import immutable_state_access as access
    with ExitStack() as held:
        held.enter_context(hold_inputs(case_path))
        if restart_path is not None:held.enter_context(access.guard(restart_path))
        if retained_initial is not None:held.enter_context(access.guard(retained_initial))
        if retained is not None:
            held.enter_context(access.guard(retained[1]))
            if file_hash(retained[1])!=retained[2]:raise ValueError('RetainedManifestMismatch')
        release=verify_release();release_path=V4/'runtime/RELEASE.json'
        held.enter_context(access.guard(release_path))
        for row in release['sources']:
            path=(V4/row['path']).resolve()
            if not path.is_relative_to(V4):raise ValueError('ReleaseDependencyPathEscape')
            held.enter_context(access.guard(path))
            if file_hash(path)!=row['sha256']:raise ValueError('ChangedStandaloneRelease')
        return _execute_guarded(input_case_path(case_path),output,from_layer,through_layer,profile,restart_path,retained_initial,retained)


def _execute_guarded(case_path,output,from_layer,through_layer,profile,restart_path=None,retained_initial=None,retained=None):
    if profile!='fastprod' and 'background' in load(case_path)[0]:
        raise ValueError('BackgroundLiveHandoffRequiresFastProd')
    if profile == 'fastprod':
        if from_layer != 0 or restart_path is not None or retained_initial is not None or retained is not None:
            raise ValueError('RollingFastProdRestartNotSupported')
        from rolling_fast_engine import execute
        if _intermediate_mesh_override.get():return execute(case_path,output,through_layer,intermediate_meshes=True)
        return execute(case_path,output,through_layer)
    from production_prism_stack import ExecutionProfile
    ExecutionProfile(profile);output=Path(output).resolve()
    if output.exists():raise ValueError('OutputMustBeNew')
    if retained_initial is not None and (restart_path is not None or from_layer!=0 or through_layer<1 or profile!='production'):
        raise ValueError('RetainedInitialResumeScope')
    if retained is not None and (restart_path is None or from_layer<1 or through_layer<=from_layer or retained_initial is not None):
        raise ValueError('RetainedTransitionResumeScope')
    pre=preflight(case_path,from_layer,through_layer,restart_path)
    # Do not promote a new workflow without the repository's independent review.
    release=read(V4/'runtime/RELEASE.json')
    if release.get('review_passed') is not True:raise ValueError('StandaloneReleaseReviewRequired')
    for row in release['sources']:
        if file_hash(V4/row['path'])!=row['sha256']:raise ValueError('ChangedStandaloneRelease')
    output.mkdir(parents=True);start=time.perf_counter()
    copy_inputs(case_path,output/'CASE.json');inputs=output/'CASE.json'
    durable_json(output/'PREFLIGHT.json',pre)
    durable_json(output/'OPERATIONAL_CONTROLS.json',dict(case_sha256=pre['case_sha256'],
        original=pre['case_resources'],effective=pre['resources'],
        explicit_cli_minimum_free_bytes=_minimum_free_override.get(),scientific_inputs_changed=False))
    last=from_layer;stage='initialization'
    try:
        if restart_path is None:
            if retained_initial is None:
                stage='surface';surface,_=surface_stage(output,inputs);last=0
            if through_layer>0:
                stage='initial_front'
                if retained_initial is None:enrolled,candidate=initial_stage(output,surface,inputs)
                else:
                    from retained_initial_candidate import resume
                    enrolled,candidate=resume(output,inputs,retained_initial)
                last=1
                if profile=='qualification':
                    replay_root=output/'initial_replay';replay_root.mkdir()
                    replay_enrollment,replay_candidate=initial_stage(replay_root,surface,inputs)
                    a=read(output/'first_layer/A/RESULT.json');b=read(replay_root/'first_layer/A/RESULT.json')
                    from verify_live_sf2_first_layer import compare_realizations
                    comparisons=compare_realizations(output/'first_layer/A',replay_root/'first_layer/A',
                        read(output/'first_layer/PREFLIGHT_A.json'),read(replay_root/'first_layer/PREFLIGHT_A.json'))
                    durable_json(output/'FIRST_REPLAY.json',comparisons)
                durable_json(output/'RESTART.json',dict(schema='V4MeshRestart/1',case_sha256=file_hash(inputs),
                    inputs=str(inputs),initial=enrolled,candidate=str(candidate),candidate_sha256=file_hash(candidate),
                    trajectory_owner=None,authorization=None,parent=dict(passed=True,layer=1,
                        scientific=enrolled['case']['committed'],reference=enrolled['reference'])))
        else:
            restart=read(restart_path)
            if (restart.get('schema')!='V4MeshRestart/1' or restart['case_sha256']!=file_hash(inputs)
                    or restart['parent']['layer']!=from_layer or file_hash(restart['candidate'])!=restart['candidate_sha256']):
                raise ValueError('QualifiedRestartBindingMismatch')
            enrolled=restart['initial'];candidate=Path(restart['candidate'])
        if through_layer>max(from_layer,1):
            stage='continuation';last=continuation(output,inputs,enrolled,candidate,through_layer,profile,
                restart=None if restart_path is None else restart,retained=retained)
        durable_json(output/'RESULT.json',dict(passed=True,last_qualified_layer=last,seconds=time.perf_counter()-start))
    except BaseException as error:
        restart_error=None
        pointer=output/'RESTART.json'
        if pointer.exists():
            try:
                previous=read(pointer)
                verified=verify_restart(pointer,file_hash(inputs),previous['parent']['layer'])
                last=verified['layer']
            except BaseException as secondary:restart_error=str(secondary)
        durable_json(output/'STOP.json',dict(stage=stage,error=str(error),traceback=traceback.format_exc(),
            last_qualified_layer=last,restart_verification_error=restart_error,
            seconds=time.perf_counter()-start,no_retry=True))
        raise


def main(argv=None):
    parser=argparse.ArgumentParser(description='Standalone V4 mesher; L0 is surface, L1 is full normal thickness.')
    parser.add_argument('--case',type=Path,required=True)
    parser.add_argument('--from-layer',type=int,required=True);parser.add_argument('--through-layer',type=int,required=True)
    parser.add_argument('--restart',type=Path);parser.add_argument('--output',type=Path)
    parser.add_argument('--resume-initial',type=Path,help='Hash-bound retained initial candidate; resume storage/publication without meshing.')
    parser.add_argument('--resume-transition',type=Path,help='Retained generated transition; resume existing assembly without regeneration.')
    parser.add_argument('--retained-manifest',type=Path)
    parser.add_argument('--retained-manifest-sha256')
    parser.add_argument('--minimum-free-bytes',type=int,help='Explicit operational free-space reserve in decimal bytes; does not change the case.')
    parser.add_argument('--mode',choices=('fastprod','production','qualification'),default='fastprod',
        help='fastprod: rolling process-local mesh, no restart; production: historical archived mode; qualification: archived replay')
    parser.add_argument('--check',action='store_true',help='Validate requirements and live binding; no meshing.')
    parser.add_argument('--worker',action='store_true',help=argparse.SUPPRESS)
    parser.add_argument('--intermediate-meshes',action='store_true',help='FastProd: also export each accepted intermediate prism layer.')
    parser.add_argument('--seed-query-diagnostics',action='store_true',help='Record passive generic CGAL query counts and timings; does not change criteria or acceptance.')
    parser.add_argument('--carrier-derivative-diagnostics',action='store_true',help='General SDF: include optional carrier directional derivative diagnostics; adequacy and final acceptance stay required.')
    parser.add_argument('--surface-carrier',choices=('adaptive','nodal'),default=None,
        help='General SDF construction: adaptive carrier assessment or direct nodal metric. FastProd defaults to nodal; other modes retain adaptive. Final acceptance is unchanged.')
    parser.add_argument('--surface-assurance',choices=('numerical','qualification'),default=None,
        help='General SDF final acceptance: sampled numerical FastProd default, or explicit continuous formal qualification. Numeric acceptance does not prove whole-source coverage.')
    args=parser.parse_args(argv)
    retained=retained_arguments(args)
    if args.minimum_free_bytes is not None and args.minimum_free_bytes<=0:parser.error('--minimum-free-bytes must be positive')
    if retained is not None and (args.restart is None or args.from_layer<1 or args.through_layer<=args.from_layer or args.resume_initial is not None):
        parser.error('Retained transition requires an advancing qualified restart, not an initial resume')
    if args.output is None and not args.check:parser.error('--output is required for execution')
    if args.intermediate_meshes and args.mode!='fastprod':parser.error('--intermediate-meshes requires --mode fastprod')
    if args.surface_assurance=='numerical' and args.mode!='fastprod':parser.error('Numerical surface assurance requires FastProd')
    token=_minimum_free_override.set(args.minimum_free_bytes)
    export_token=_intermediate_mesh_override.set(args.intermediate_meshes)
    diagnostic_token=_seed_query_diagnostics.set(args.seed_query_diagnostics)
    carrier_token=_carrier_derivative_diagnostics.set(args.carrier_derivative_diagnostics)
    assessment_token=_carrier_assessment.set(args.surface_carrier=='adaptive' if args.surface_carrier is not None else args.mode!='fastprod')
    assurance_token=_surface_assurance.set(args.surface_assurance or ('numerical' if args.mode=='fastprod' else 'qualification'))
    try:return _main_selected(args,retained)
    finally:
        _surface_assurance.reset(assurance_token)
        _carrier_assessment.reset(assessment_token)
        _carrier_derivative_diagnostics.reset(carrier_token)
        _seed_query_diagnostics.reset(diagnostic_token)
        _intermediate_mesh_override.reset(export_token);_minimum_free_override.reset(token)


def retained_arguments(args):
    values=(getattr(args,'resume_transition',None),getattr(args,'retained_manifest',None),
        getattr(args,'retained_manifest_sha256',None))
    if all(v is None for v in values):return None
    if any(v is None for v in values):raise ValueError('CompleteRetainedTransitionInputsRequired')
    if len(values[2])!=64 or any(c not in '0123456789abcdef' for c in values[2]):raise ValueError('InvalidRetainedManifestSHA256')
    return values


def _main_selected(args,retained):
    if args.mode == 'fastprod' and (args.from_layer != 0 or args.restart is not None
            or args.resume_initial is not None or retained is not None):
        raise ValueError('RollingFastProdRestartNotSupported')
    if args.check:
        print(json.dumps(preflight(args.case,args.from_layer,args.through_layer,args.restart),indent=2,default=str));return 0
    if args.worker:
        import os,psutil
        if os.environ.get('V4_STANDALONE_SUPERVISOR')!=str(psutil.Process().ppid()):
            raise ValueError('MonitoredWorkerRequired')
        with hold_inputs(args.case):
            if file_hash(input_case_path(args.case))!=os.environ.get('V4_STANDALONE_CASE_SHA256'):
                raise ValueError('SupervisorCaseIdentityMismatch')
            if (held_import_sha256(args.case) or '')!=os.environ.get('V4_STANDALONE_IMPORTED_SOURCE_SHA256',''):
                raise ValueError('SupervisorImportedSourceIdentityMismatch')
            execute(input_case_path(args.case),args.output,args.from_layer,args.through_layer,args.mode,args.restart,args.resume_initial,retained)
    else:
        supervise(args)
    return 0


def supervise(args):
    with hold_inputs(args.case):
        selected=copy.copy(args);selected.case=input_case_path(args.case)
        return _supervise_guarded(selected)


def terminate_worker(output,case,observed,process):
    """Cancel the scoped Linux backend, but always stop the Windows worker."""
    import psutil
    cancellation=None
    try:
        if 'background' in case:
            from standalone_background import stop_backend
            stop_backend(output)
    except BaseException as error:cancellation=error
    try:
        for child in observed.children(recursive=True):
            try:child.terminate()
            except psutil.Error:pass
    except psutil.Error:pass
    finally:
        process.terminate();process.wait()
    if cancellation is not None:raise cancellation


def write_monitor(path,record):
    """Retry only the existing atomic telemetry rename on transient NTFS locks.

    Same bounded pattern as successful L5 retention. Never used for scientific
    or authentication files; failure still stops the supervised worker.
    """
    from path_preserving_deduplication import atomic_replace
    import os
    pending=path.with_name(path.name+'.pending')
    with pending.open('w',encoding='utf-8',newline='\n') as stream:
        json.dump(record,stream,indent=2,sort_keys=True,allow_nan=False)
        stream.write('\n');stream.flush();os.fsync(stream.fileno())
    try:atomic_replace(pending,path);return
    except PermissionError as error:
        if getattr(error,'winerror',None) not in (5,32,33):raise
    for _ in range(10):
        time.sleep(.1)
        try:atomic_replace(pending,path);return
        except PermissionError as error:
            if getattr(error,'winerror',None) not in (5,32,33):raise
    raise PermissionError('MonitorAtomicRenameBlocked',str(path))


def _supervise_guarded(args):
    """One command/one worker, monitored throughout; no scientific retries."""
    import subprocess
    import psutil
    case,_,_=load(args.case);output=args.output.resolve()
    if output.exists():raise ValueError('OutputMustBeNew')
    output.parent.mkdir(parents=True,exist_ok=True)
    log=output.parent/(output.name+'.execution.log')
    if log.exists():raise ValueError('PreserveExecutionLog')
    command=[sys.executable,'-B',str(V4/'mesh.py'),'--worker','--case',str(args.case.resolve()),
        '--output',str(output),'--from-layer',str(args.from_layer),'--through-layer',str(args.through_layer),
        '--mode',args.mode]
    if args.restart is not None:command+=['--restart',str(args.restart.resolve())]
    if getattr(args,'intermediate_meshes',False):command+=['--intermediate-meshes']
    if getattr(args,'seed_query_diagnostics',False):command+=['--seed-query-diagnostics']
    if getattr(args,'carrier_derivative_diagnostics',False):command+=['--carrier-derivative-diagnostics']
    if getattr(args,'surface_carrier',None) is not None:command+=['--surface-carrier',args.surface_carrier]
    if getattr(args,'surface_assurance',None) is not None:command+=['--surface-assurance',args.surface_assurance]
    if getattr(args,'resume_initial',None) is not None:command+=['--resume-initial',str(args.resume_initial.resolve())]
    retained=retained_arguments(args)
    if retained is not None:command+=['--resume-transition',str(retained[0].resolve()),
        '--retained-manifest',str(retained[1].resolve()),'--retained-manifest-sha256',retained[2]]
    if getattr(args,'minimum_free_bytes',None) is not None:command+=['--minimum-free-bytes',str(args.minimum_free_bytes)]
    resources=effective_resources(case['resources'])
    start=time.perf_counter();peak=0;child_peak=0;breach=None
    with log.open('w',encoding='utf-8',newline='\n') as stream:
        import os
        env=dict(os.environ,V4_STANDALONE_SUPERVISOR=str(os.getpid()),
                 V4_STANDALONE_CASE_SHA256=file_hash(args.case),
                 V4_STANDALONE_IMPORTED_SOURCE_SHA256=held_import_sha256(args.case) or '')
        process=subprocess.Popen(command,stdout=stream,stderr=subprocess.STDOUT,env=env)
        observed=psutil.Process(process.pid)
        try:
          while process.poll() is None:
            try:
                peak=max(peak,observed.memory_info().rss)
                child_peak=max(child_peak,max((p.memory_info().rss for p in observed.children(recursive=True)),default=0))
                location=output if output.exists() else output.parent
                data=check_resources(location,case['resources']) if output.exists() else dict(
                    logical_bytes=0,free_bytes=shutil.disk_usage(location).free)
                if data['free_bytes']<resources['minimum_free_bytes']:raise ValueError('OperationalResourceLimit')
                if output.exists():write_monitor(output/'MONITOR.json',dict(data,elapsed_s=time.perf_counter()-start,
                    sampled_worker_rss_peak=peak,sampled_child_rss_peak=child_peak))
            except psutil.Error:pass
            except ValueError as error:
                breach=str(error)
                try:terminate_worker(output,case,observed,process)
                except BaseException as cancellation:breach+='; core cancellation: '+str(cancellation)
                break
            time.sleep(1)
          code=process.wait()
        finally:
          if process.poll() is None:
            terminate_worker(output,case,observed,process)
    if output.exists():
        record=dict(returncode=code,seconds=time.perf_counter()-start,
            sampled_worker_rss_peak=peak,sampled_child_rss_peak=child_peak,resource_failure=breach)
        durable_json(output/'PROCESS.json',record)
        try:check_resources(output,case['resources'])
        except ValueError as error:
            breach=str(error);record['resource_failure']=breach
            durable_json(output/'PROCESS.json',record)
    if code or breach:raise ValueError('MeshWorkerStopped',code,breach,str(log))

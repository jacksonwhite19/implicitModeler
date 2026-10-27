"""Live shell handoff to existing core algorithms and qualification predicates.

No archived seal is made or borrowed. An unchanged, live-accepted shell is the
old-old geometric proof owner. All new geometry and actual neighbours are
checked by the existing predicates. Output is mesh-only, not restart authority.
"""
import stage_observations as obs
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time
import numpy as np
from sdf_cfd_mesher_v4.common.authority import identity, array_hash, file_hash
from path_preserving_deduplication import durable_json
from owned_runtime import V4

def _stop_record(stage,error,seconds,layer):
    """Failure telemetry only. It cannot authorize a retry or publish a mesh."""
    import traceback
    from background_interface_completion import CompletionWorkLimit
    record=dict(stage=stage,error=str(error),traceback=traceback.format_exc(),
        seconds=seconds,last_qualified_prism_layer=layer,no_retry=True)
    if type(error) is CompletionWorkLimit:record['interface_completion_work_limit']=error.details()
    return record


def attach_transition_assurance(result,report,verification,assurance,binding):
    from finite_research.flexible_assurance import report_schema
    from finite_research.fluid_assurance import validate_component
    if report.get('schema')!=report_schema(report.get('controls')):
        raise ValueError('ChangedTransitionReturnSchema')
    result.update(transition_fluid_verification=verification,fluid_assurance=assurance,
        acceptance_schema=report['schema'])
    validate_component(result,verification,binding)
    return result


def validate_policy(policy):
    fields={'schema','bounds_m','growth_per_m','maximum_size_m'}
    if isinstance(policy,dict) and policy.get('schema') in ('V4GradedBackground/2','V4GradedBackground/3','V4GradedBackground/4','V4GradedBackground/5','V4GradedBackground/6'):
        fields.add('interface_seed_policy')
        if policy.get('interface_seed_policy') not in ('equal_prism_volume/1','balanced_centroid_volume/1'):
            raise ValueError('UnsupportedInterfaceSeedPolicy')
        if policy['schema'] in ('V4GradedBackground/3','V4GradedBackground/4','V4GradedBackground/5','V4GradedBackground/6'):
            fields.add('assurance_mode')
            validate_assurance_mode(policy.get('assurance_mode'))
        if policy['schema'] in ('V4GradedBackground/4','V4GradedBackground/5','V4GradedBackground/6'):
            from background_backend_method import validate
            fields.add('backend_method');validate(policy.get('backend_method'))
        if policy['schema'] in ('V4GradedBackground/5','V4GradedBackground/6'):
            fields.add('interface_completion');validate_completion(policy.get('interface_completion'))
            if policy.get('assurance_mode')!='backend_trusted':raise ValueError('ProtectedRoutineCompletionProfileRequired')
        if policy['schema']=='V4GradedBackground/6':
            from finite_stage_controls import validate as validate_transition
            fields.add('tet_transition');validate_transition(policy.get('tet_transition'))
            if policy['interface_completion']['schema']!='V4CoreMeshCompletion/1':raise ValueError('ProtectedGenericBulkRepairRequired')
    if (not isinstance(policy,dict) or set(policy)!=fields
            or policy['schema'] not in ('V4GradedBackground/1','V4GradedBackground/2','V4GradedBackground/3','V4GradedBackground/4','V4GradedBackground/5','V4GradedBackground/6')):
        raise ValueError('ExplicitBackgroundPolicyRequired')
    bounds=np.asarray(policy['bounds_m'],dtype=float)
    if bounds.shape!=(3,2) or not np.isfinite(bounds).all() or np.any(bounds[:,0]>=bounds[:,1]):
        raise ValueError('InvalidBackgroundBounds')
    for name in ('growth_per_m','maximum_size_m'):
        v=policy[name]
        if type(v) not in (float,int) or not math.isfinite(v) or v<=0:
            raise ValueError('InvalidBackgroundControl',name)
    return bounds

def complete_mesh_payload(points,prisms,mapped,wall,far_faces,additions,completion):
    payload=dict(points=points,prisms=np.asarray(prisms,dtype='<i8'),
        tetrahedra=mapped,wall=np.asarray(wall,dtype='<i8'),farfield=np.asarray(far_faces,dtype='<i8'))
    if completion is not None and completion.get('schema')=='V4CoreMeshCompletion/1':
        if mapped.shape!=(len(additions),4) or not np.array_equal(mapped,np.asarray([c['vertices'] for c in additions],dtype='<i8')):
            raise ValueError('ChangedRepairedCoreOutputRows')
        payload['tetrahedron_ids']=np.asarray([c['id'] for c in additions],dtype='<i8')
    return payload

def validate_completion(controls):
    from background_completion_controls import validate_completion as validate
    return validate(controls)


def validate_assurance_mode(mode):
    if type(mode) is not str or mode not in ('qualification','backend_trusted'):
        raise ValueError('UnsupportedBackgroundAssuranceMode')
    return mode


def assurance_mode(policy):
    validate_policy(policy)
    # Historical cases retain all existing proofs. Mode selection is explicit
    # and case-hash bound; no environment variable or backend return can relax it.
    return policy['assurance_mode'] if policy['schema'] in ('V4GradedBackground/3','V4GradedBackground/4','V4GradedBackground/5','V4GradedBackground/6') else 'qualification'


def backend_command(*arguments):
    """Use the installed Gmsh runtime; no build or installer fallback."""
    module=V4/'tools/core_backend.py'
    if os.name!='nt':return [sys.executable,'-B',str(module),*map(str,arguments)]
    def wsl(path):
        return subprocess.check_output(['wsl.exe','-e','wslpath','-u',str(path)],text=True,encoding='utf-8').strip()
    paths=':'.join(wsl(V4/p) for p in ('tools','src'))
    converted=[wsl(arg) if isinstance(arg,Path) else str(arg) for arg in arguments]
    return ['wsl.exe','-e','env','PYTHONDONTWRITEBYTECODE=1','PYTHONPATH='+paths,
            'python3','-B',wsl(module),*converted]


def probe_backend(method=None):
    from finite_research.outer_worker import selected_probe
    return selected_probe(method)


@obs.observed('background.policy_backend','source_lock')
def policy_backend(policy):
    validate_policy(policy)
    return probe_backend(policy['backend_method']) if policy['schema'] in ('V4GradedBackground/4','V4GradedBackground/5','V4GradedBackground/6') else probe_backend()


def stop_backend(output,*,component_folder='background'):
    """Existing WSL cancellation pattern, scoped to this worker's exact folder."""
    import re
    if component_folder not in ('background','transition'):raise ValueError('InvalidBackendComponentFolder')
    folder=Path(output)/component_folder
    if os.name!='nt':return
    from shape_transition_native import stop_native
    stop_native(folder/'SHAPE_CONSTRUCTION')
    if not (folder/'INPUT.json').is_file():return
    command=backend_command('--input',folder)
    pattern=re.escape(command[-3])+' '+re.escape(command[-2])+' '+re.escape(command[-1])+'$'
    for signal in ('TERM','KILL'):
        result=subprocess.run(['wsl.exe','-e','pkill','-'+signal,'-f',pattern],timeout=15)
        if result.returncode not in (0,1):raise ValueError('CoreBackendCancellationFailed')
        for _ in range(10):
            found=subprocess.run(['wsl.exe','-e','pgrep','-f',pattern],capture_output=True,timeout=15)
            if found.returncode==1:return
            if found.returncode:raise ValueError('CoreBackendCancellationCheckFailed')
            time.sleep(.2)
    raise ValueError('CoreBackendStillRunning')


def check_box(outer,far,bounds):
    # Unsupported input structure must keep the original traversal and first
    # exception, including a bad later index after an earlier off-plane face.
    if (type(outer) is not np.ndarray or outer.dtype!=np.dtype('float64') or
            outer.ndim!=2 or outer.shape[1:]!=(3,) or
            type(far) is not np.ndarray or far.ndim!=2 or far.shape[1:]!=(3,) or far.dtype.kind not in 'iu' or
            type(bounds) is not np.ndarray or bounds.shape!=(3,2) or
            (far.size and (np.any(far<0) or np.any(far>=len(outer))))):
        return _check_box_legacy(outer,far,bounds)
    return _check_box_fast(outer,far,bounds)


def _check_box_legacy(outer,far,bounds):
    """Unchanged original owner for malformed or unsupported input structure."""
    from fractions import Fraction as F
    if (not np.isfinite(outer).all() or np.any(outer<bounds[:,0]) or np.any(outer>bounds[:,1])
            or any(not any(np.all(outer[f,k]==bounds[k,side]) for k in range(3) for side in (0,1)) for f in far)):
        raise ValueError('UnauthorizedFarfieldDomain')
    corners={tuple(p) for p in outer}
    import itertools
    if any(tuple(p) not in corners for p in itertools.product(*bounds)):
        raise ValueError('MissingAuthorizedBoxCorner')
    from incremental_layer_validation import t
    P=[tuple(F(float(v)) for v in p) for p in outer]
    actual=sum(t.dot(P[f[0]],t.cross(P[f[1]],P[f[2]]))/6 for f in far)
    expected=F(1)
    for lo,hi in bounds:expected*=F(float(hi))-F(float(lo))
    if actual!=expected:raise ValueError('AuthorizedBoxVolumeMismatch')


def _check_box_fast(outer,far,bounds):
    """Exact authorized planar box, independent of backend declarations."""
    from fractions import Fraction as F
    if not np.isfinite(outer).all() or np.any(outer<bounds[:,0]) or np.any(outer>bounds[:,1]):
        raise ValueError('UnauthorizedFarfieldDomain')
    # Same six exact plane comparisons in bounded independent face batches.
    for start in range(0,len(far),4096):
        q=outer[far[start:start+4096]];plane=np.zeros(len(q),dtype=bool)
        for k in range(3):
            for side in (0,1):plane|=np.all(q[:,:,k]==bounds[k,side],axis=1)
        if not plane.all():raise ValueError('UnauthorizedFarfieldDomain')
    corners={tuple(p) for p in outer}
    import itertools
    if any(tuple(p) not in corners for p in itertools.product(*bounds)):
        raise ValueError('MissingAuthorizedBoxCorner')
    from prism_hotpath_v2 import exact_boundary
    actual=sum(exact_boundary(outer,f) for f in far)
    expected=F(1)
    for lo,hi in bounds:expected*=F(float(hi))-F(float(lo))
    if actual!=expected:raise ValueError('AuthorizedBoxVolumeMismatch')


def validate_execution_target(policy,transition_only,*,execution_target='transition'):
    """Bound supported dispatch; schema4 has no qualified bulk continuation."""
    from finite_stage_controls import physical_shape,finite_collar
    if transition_only and policy['schema']!='V4GradedBackground/6':
        raise ValueError('TransitionOnlyRequiresExplicitTransitionPolicy')
    from finite_research.execution_target import choice
    target=choice({'execution_target':execution_target})
    if type(transition_only) is not bool:raise ValueError('ExplicitBooleanTransitionTargetRequired')
    if target=='transition_background':
        if transition_only or not finite_collar(policy.get('tet_transition',{})):raise ValueError('FiniteJoinedTargetPolicyMismatch')
        return
    if finite_collar(policy.get('tet_transition',{})) and not transition_only:
        raise ValueError('FiniteCollarRequiresTransitionOnlyTarget')
    if physical_shape(policy.get('tet_transition',{})) and not transition_only:
        raise ValueError('PhysicalTransitionRequiresTransitionOnlyTarget')


@obs.observed('background.preflight','source_lock')
def preflight(binding,policy,*,transition_only=False,execution_target='transition'):
    bounds=validate_policy(policy)
    validate_execution_target(policy,transition_only,execution_target=execution_target)
    from canonical_spline_authority import LiveSplineAuthority
    from standalone_geometry import is_scalar
    if type(binding.authority) is LiveSplineAuthority:
        a=binding.authority;a.verify()
        metadata=a.geometry.description['stored_metadata']
        if metadata['bound_quality'] not in ('Conservative','Exact'):
            raise ValueError('SourceEnclosurePremiseUnavailable')
        source=np.asarray(metadata['bounds_model'],float)*a.geometry.description['metres_per_model_unit']
    elif is_scalar(binding):
        d=binding.authority.definition;r=d['radius'];half=d['half_length']
        source=np.array([[-r,-r,-half-r],[r,r,half+r]])
    else:
        from general_source_contract import GeneralSourceQualification
        if type(binding.qualification) is not GeneralSourceQualification:raise ValueError('SourceEnclosurePremiseUnavailable')
        binding.qualification.verify(binding.authority)
        from fractions import Fraction
        from sdf_cfd_mesher_v4.sdf.regional import upper_float
        if binding.qualification.assurance=='numerical':
            # A design enclosure does not require C1/radial topology. Reuse
            # the existing exact real-source coercivity owner with captured
            # expression/units and unchanged live source/arithmetic custody.
            from sdf_cfd_mesher_v4.sdf.operations import source_box
            a=binding.authority
            with a.prism_query_scope(),a.verified_operation('background_enclosure'):
                expression,scale,budgets,_=a.queries._scope._batch_inputs()
                box=source_box(expression,scale,precision_bits=budgets[2])
                if box is None:raise ValueError('SourceZeroEnclosureUnavailable')
                source=np.array([[-upper_float(-v.lo) for v in box],
                                 [upper_float(v.hi) for v in box]])
                a.verify()
        else:
            radius=upper_float(Fraction(binding.qualification.source_topology['enclosure_radius_m']))
            source=np.array([[-radius]*3,[radius]*3])
    if np.any(source[0]<=bounds[:,0]) or np.any(source[1]>=bounds[:,1]):
        raise ValueError('SourceOutsideBackgroundDomain')
    return dict(passed=True,policy=policy,assurance_mode=assurance_mode(policy),source_bounds_m=source.tolist(),backend=None if transition_only else policy_backend(policy),
        execution_target='transition' if transition_only else 'background',
        scope='Design enclosure only; actual terminal front and complete core still require qualification.',
        source_authority=binding.geometry_identity)


def _interface(run, *, _preparation=None, _handoff=None):
    from rolling_fast_state import LiveAcceptance
    from sdf_cfd_mesher_v4.core.interface import _extract
    from rolling_fast_state import _live_component_digest
    from incremental_layer_validation import key,t
    if type(run.live_acceptance) is not LiveAcceptance or not run.rolling:
        raise ValueError('LiveAcceptedShellRequired')
    if _handoff is None:run.live_acceptance.check(run, _preparation=_preparation)
    else:_handoff.require(run)
    token=run.live_acceptance
    if token.count!=sum(dict(token.methods).values()):raise ValueError('InvalidInheritedCoverage')
    points,offsets,connectivity,metadata=(_extract(run.tx.current,digest=_live_component_digest)
        if _handoff is None else _handoff.extract())
    if any(np.diff(offsets)!=3):raise ValueError('TriangularTerminalInterfaceRequired')
    triangles=connectivity.reshape(-1,3)
    n=len(run.provider.x)
    def numeric(vertex):
        marker,layer,source,i=vertex.split(':')
        if marker!='layer' or source!='source':raise ValueError('UnsupportedSourceColumnMapping')
        return int(layer)*n+int(i)
    ids=np.array([numeric(v) for v in metadata['vertex_ids']],dtype='<i8')
    if run.points[ids].tobytes()!=points.tobytes():raise ValueError('LiveCoreInterfaceCoordinateMismatch')
    from background_interface_geometry import prepass
    topology=run.topology
    if _handoff is not None:
        from types import SimpleNamespace
        topology=SimpleNamespace(cells={c['id']:c for c in _handoff.cells(map(int,metadata['shell_owners']))})
    cached={} if _handoff is None else _handoff.terminal_geometry()
    provisional=(cached if cached and all(int(i) in cached for i in metadata['shell_owners'])
        else prepass(run.points,topology,metadata['shell_owners']))
    volumes=[];terminal=[];centres=[]
    for face,owner in zip(ids[triangles],metadata['shell_owners']):
        rows=run.topology.owners[key(face)]
        if len(rows)!=1 or str(rows[0][0])!=owner:raise ValueError('TerminalOwnerMismatch')
        cell=run.topology.cells[rows[0][0]]
        if cell['kind']!='prism' or cell['layer']!=run.provider.layer:
            raise ValueError('UniformOrdinaryTerminalOwnerRequired')
        if not t.reverse_cycle(tuple(face[::-1]),tuple(rows[0][1])):
            raise ValueError('TerminalOwnerOrientationMismatch')
        geometry=(t.geometry(run.points,cell) if provisional is None else provisional[cell['id']]);volume=geometry['volume']
        if not math.isfinite(volume) or volume<=0:raise ValueError('NonpositiveTerminalOwnerVolume')
        volumes.append(volume);terminal.append(cell['id']);centres.append(geometry['centre'])
    return token,points,triangles,ids,np.asarray(volumes),terminal,metadata,np.asarray(centres)


def interface(run):
    """Preserve the existing public interface tuple and scalar-volume semantics."""
    return _interface(run)[:7]


def interface_seeds(policy,points,triangles,volumes,prism_centres):
    """Explicit design target, not a bound on realized tet centroids or quality.

    Balanced affine tet target V=4*A*dp/3 uses the existing face geometry and
    volume-to-size conversion. All quantities are SI. Legacy /1 is unchanged.
    """
    from generic_core_sizing import equal_volume_seed
    from incremental_layer_validation import t
    validate_policy(policy)
    name=policy.get('interface_seed_policy','equal_prism_volume/1')
    if name=='equal_prism_volume/1':return equal_volume_seed(volumes)
    if (prism_centres.shape!=(len(triangles),3) or volumes.shape!=(len(triangles),)
            or not np.isfinite(prism_centres).all() or not np.isfinite(volumes).all()
            or np.any(volumes<=0)):
        raise ValueError('InvalidTerminalPrismGeometry')
    cq=t.q.legacy_modules()['component_quality']
    S,C=cq.FG.face_area_centre_tri(points[triangles])
    da=np.einsum('ni,ni->n',S,C-prism_centres)
    if not np.isfinite(da).all() or np.any(da<=0):
        raise ValueError('UnsupportedBalancedPrismSide')
    return equal_volume_seed(4*da/3)


@obs.observed('background.compare_boundary','scientific')
def compare_boundary(core_points,tets,front,tri,outer,far,*,_return_call_data=False):
    """Reuse finite-interface audit; also require the complete farfield set."""
    from capsule_sizing_control import boundary_audit
    from incremental_layer_validation import t
    for name,faces in (('interface',tri),('farfield',far)):
        if len(np.unique(np.sort(faces,axis=1),axis=0))!=len(faces):
            raise ValueError('DuplicateCoreBoundaryFace',name)
    audit,owners,boundary,call_data=boundary_audit(core_points,tets,front,tri,_return_call_data=True)
    if audit['nonmanifold_faces']:raise ValueError('NonmanifoldCore')
    lookup=call_data['lookup']
    inner=np.asarray(call_data['interface_ids'],dtype='<i8')
    far_ids=np.array([lookup[tuple(p)] for p in outer],dtype='<i8')
    if core_points[inner].tobytes()!=front.tobytes():raise ValueError('ChangedInterfaceBits')
    if core_points[far_ids].tobytes()!=outer.tobytes():raise ValueError('ChangedFarfieldBits')
    expected={tuple(sorted(f)) for f in inner[tri]}|{tuple(sorted(f)) for f in far_ids[far]}
    if expected!={tuple(map(int,f)) for f in boundary}:raise ValueError('ChangedCoreBoundaryFaces')
    wanted={tuple(sorted(f)):tuple(f) for f in far_ids[far]}
    remaining=set(wanted)
    for face in call_data['boundary_uses']:
        k=tuple(sorted(face))
        if k in remaining:
            if not t.reverse_cycle(wanted[k][::-1],tuple(face)):raise ValueError('FarfieldWindingMismatch')
            remaining.remove(k)
    if remaining:raise ValueError('MissingFarfieldFace')
    result=(audit,inner,far_ids)
    return (*result,{k:call_data[k] for k in ('inverse','unique')}) if _return_call_data else result


def qualify(run,core_points,tets,front,tri,front_ids,terminal,outer,far,folder,token,*,bounds=None,_preparation=None,
            assurance_mode='qualification',_handoff=None,interface_completion=None,_transition=None,_full_joined=False,_outer_check=None):
    from contextlib import ExitStack
    with ExitStack() as artifacts:
        return _qualify(run,core_points,tets,front,tri,front_ids,terminal,outer,far,folder,token,
            bounds=bounds,_preparation=_preparation,assurance_mode=assurance_mode,_handoff=_handoff,
            interface_completion=interface_completion,_transition=_transition,_artifact_custody=artifacts,
            _full_joined=_full_joined,_outer_check=_outer_check)


@obs.observed('background.joined_acceptance','scientific')
def _qualify(run,core_points,tets,front,tri,front_ids,terminal,outer,far,folder,token,*,bounds=None,_preparation=None,
             assurance_mode='qualification',_handoff=None,interface_completion=None,_transition=None,_artifact_custody,
             _full_joined=False,_outer_check=None):
    """Compose live old-old acceptance with freshly checked core geometry.

    Retained geometry/quality routines stay authoritative. Explicit backend
    trust omits joined collision/coverage/filling proofs, never shell acceptance.
    Nothing is projected, snapped, archived, or assigned a fake source sheet.
    """
    validate_assurance_mode(assurance_mode)
    if type(_full_joined) is not bool:raise ValueError('ExplicitJoinedProofSelectionRequired')
    if _full_joined and (_transition is None or _handoff is None or not callable(_outer_check)):
        raise ValueError('LiveFiniteJoinedProofInputsRequired')
    full_proofs=assurance_mode=='qualification' or _full_joined
    from finite_research.flexible_controls import graded
    graded_joined=_full_joined and graded(_transition.report.get('controls'))
    def check():
        if _outer_check is not None:_outer_check()
    check()
    if _transition is not None:
        from finite_stage_handoff import TransitionInputs
        if type(_transition) is not TransitionInputs:raise ValueError('LiveTetTransitionInputsRequired')
        _transition.require_merged(_handoff,core_points,tets)
    if interface_completion is not None:
        validate_completion(interface_completion)
        if assurance_mode!='backend_trusted' or _handoff is None:raise ValueError('ProtectedRoutineCompletionProfileRequired')
    from incremental_layer_validation import key,t
    from rolling_fast_state import verify_fresh_coverage
    from native_mixed_face_certificate import certify
    import cavity_volume_audit as volume
    timings={};mark=time.perf_counter()
    def measured(name):
        nonlocal mark
        check()
        now=time.perf_counter();timings[name]=now-mark;mark=now
    if _handoff is None:run.live_acceptance.check(run, _preparation=_preparation)
    else:
        if assurance_mode!='backend_trusted':raise ValueError('RoutineBackgroundHandoffRequired')
        _handoff.require(run)
    if run.live_acceptance is not token:raise ValueError('ChangedLiveCoreParent')
    measured('initial_live_parent')
    if bounds is not None:check_box(outer,far,bounds)
    measured('authorized_box')
    audit,inner,far_ids,incidence=compare_boundary(core_points,tets,front,tri,outer,far,_return_call_data=True)
    measured('boundary_comparison')
    # Fixed materialization gives duplicate positive tetrahedra equal face
    # winding: complete ownership rejects them. Opposite winding reverses the
    # exact determinant, so mandatory fresh positive-cell quality rejects it.
    # Both gates below apply to EVERY new tetrahedron; no canonical cell scan.
    if len(tets)==0:raise ValueError('EmptyCore')
    other=np.setdiff1d(np.arange(len(core_points)),inner)
    mapping=np.empty(len(core_points),dtype='<i8');mapping[inner]=front_ids
    mapping[other]=len(run.points)+np.arange(len(other))
    points=np.concatenate([run.points,core_points[other]])
    if points[:len(run.points)].tobytes()!=run.points.tobytes():raise ValueError('ChangedAcceptedShellCoordinates')
    cells=(list(run.topology.cells.values()) if _handoff is None else _handoff.cells());last=max(run.topology.cells)+1
    mapped=mapping[tets]
    from background_materialization import build
    additions=build(mapped,last)
    cells+=additions
    far_faces=[tuple(map(int,f)) for f in mapping[far_ids][far]]
    measured('mapping_and_cells')
    owners,ownership=t._ownership_from_core_incidence(run.topology.owners,
        additions,incidence['inverse'],incidence['unique'],mapping,last,far_faces,run.wall)
    del incidence
    if interface_completion is not None and interface_completion['schema']=='V4CoreMeshCompletion/1':
        ownership.update(stage='initial_backend_join',scope='Initial joined incidence. Subsequent sparse replacements preserve actual face owners and signed cavity boundaries; final quality uses post-repair owners. These counts precede repair.')
    durable_json(folder/'OWNERSHIP.json',ownership)
    if not ownership['passed']:raise ValueError('CompleteCoreOwnershipFailure')
    measured('complete_ownership')
    # Only terminal prism masks acquire a new neighbour. Other cell geometry,
    # ownership and masks are unchanged; their live acceptance remains valid.
    oldkeys=set(run.topology.owners)
    frontkeys={key(f) for f in run.outer}
    for k,rows in run.topology.owners.items():
        current=owners.get(k)
        if current is None or any((i,f) not in current for i,f in rows):
            raise ValueError('ChangedAcceptedShellIncidence')
        if len(current)!=len(rows) and k not in frontkeys:
            raise ValueError('UnexpectedOldNeighbourChange')
    affected=set(terminal)|{c['id'] for c in additions}
    measured('unchanged_shell_incidence')
    completion=None
    if interface_completion is not None:
        fixed=set(range(len(run.points)))|set(map(int,mapping[far_ids]))
        protected=[]
        if _transition is not None:
            protected=additions[:_transition.cells_count]
            fixed.update(v for c in protected for v in c['vertices'])
        if interface_completion['schema']=='V4CoreMeshCompletion/1':
            from background_core_repair import construct,evidence_sink
            retain=evidence_sink(folder/'REPAIR_EVENTS',_artifact_custody)
            bulk_ids={c['id'] for c in additions[len(protected):]}
            points,cells,bulk,owners,completion=construct(_handoff,points,cells,owners,bulk_ids,fixed,far_faces,interface_completion,_evidence=retain,_check=check)
            additions=protected+bulk
            mapped=np.asarray([c['vertices'] for c in additions],dtype='<i8')
            affected=set(terminal)|{c['id'] for c in additions}
        else:
            from background_interface_completion import construct
            points,completion=construct(_handoff,points,cells,owners,{c['id'] for c in additions},fixed,
                [tuple(map(int,f)) for f in front_ids[tri]],interface_completion)
        authored_completion=identity(completion)
        durable_json(folder/'INTERFACE_COMPLETION.json',completion)
        from immutable_state_access import guard
        _artifact_custody.enter_context(guard(folder/'INTERFACE_COMPLETION.json'))
        if identity(json.loads((folder/'INTERFACE_COMPLETION.json').read_text(encoding='utf-8')))!=authored_completion:
            raise ValueError('ChangedInterfaceCompletionReport')
        completion_sha256=file_hash(folder/'INTERFACE_COMPLETION.json')
        measured('bounded_interface_construction')
    # Same qualified mixed-cell operations and actual-neighbour closure. Avoid
    # millions of one-face Foundation calls without an unbounded face array.
    from prism_hotpath_v2 import batch_quality
    started=time.perf_counter();q=batch_quality(points,cells,owners,0,active_cells=affected,face_batch_size=4096,
        mixed_cells=True,compact=not full_proofs or graded_joined,_handoff=_handoff)
    metrics=q['summary'] if q.get('compact_output') else dict(
        affected_cells=len(q['cells']),affected_faces=len(q['faces']),
        minimum_determinant=min(r['determinant'] for r in q['cells']),
        maximum_aspect=max(r['aspect'] for r in q['cells']),
        minimum_weight=min(r['weight'] for r in q['faces']),
        minimum_symmetric_volume_ratio=min(r['symmetric_volume_ratio'] for r in q['faces']),
        maximum_skewness=max(r['skewness'] for r in q['faces']),
        exact_nonpositive_cells=sum(r['volume_sign']<=0 or r['jacobian_sign']<=0 for r in q['exact']))
    q_summary=dict(passed=q['passed'],failures=q['failures'],seconds=time.perf_counter()-started,
        inherited_other_shell_quality=True,actual_neighbour_updates=len(terminal),**metrics)
    durable_json(folder/'QUALITY.json',q_summary)
    if not q['passed']:raise ValueError('CompleteCoreHardQualityFailure')
    # Exact rows are fresh outputs of the same coordinates/cells above. Keep
    # only their existing objects for new cells; do not render/copy the other
    # millions of quality rows after their gate and compact report are complete.
    new_exact={r['id']:r for r in q['exact'] if r['id']>=last} if full_proofs and not graded_joined else None
    del q
    measured('mixed_quality_and_gate')
    if graded_joined:
        from finite_research.graded_joined import audit as joined_audit
        def require_joined_parent():
            _handoff.require(run);_transition.require(_handoff)
        joined_proof,certificate,summary,whole_quality,whole_boundary=joined_audit(
            points,cells,owners,run.wall,far_faces,mapping[np.frombuffer(_transition._open()['outer_ids'][0],dtype='<i8')][_transition.triangles],
            bounds,folder/'JOINED_PROOFS',check,require_joined_parent)
        q_summary=dict(passed=whole_quality['passed'],failures=whole_quality['failures'],**whole_quality['summary'])
        measured('whole_graded_joined_geometry')
    elif full_proofs:
        faces=[rows[0][1] for _,rows in sorted(owners.items())]
        active=[i for i,f in enumerate(faces) if key(f) not in oldkeys]
        measured('certificate_face_materialization')
        from background_certificate import certify as background_certify
        started=time.perf_counter();fresh=background_certify(points,faces,active,_preparation=_preparation)
        certificate={k:v for k,v in fresh.items() if k!='proofs'}
        certificate.update(inherited_relations=token.count,inherited_methods=dict(token.methods),
            inherited_owner=token.mesh,old_geometry_unchanged=True,
            new_active_faces=len(active),seconds=time.perf_counter()-started)
        durable_json(folder/'CERTIFICATE.json',certificate)
        if not fresh['passed']:raise ValueError('CompleteCoreContinuousFailure')
        measured('fresh_certificate_and_gate')
        from background_independent import AABBIndex as IndependentIndex
        methods=verify_fresh_coverage(points,faces,active,fresh,index_type=IndependentIndex)
        certificate['new_method_population']=dict(methods)
        certificate['independent_coverage_passed']=True
        durable_json(folder/'CERTIFICATE.json',certificate)
        del fresh
        measured('independent_coverage_and_release')
        from background_volume import audit as extend_volume
        accepted_volume=run.volume if _handoff is None else _handoff.volume_inputs()
        started=time.perf_counter();v=extend_volume(points,accepted_volume,set(run.topology.cells),additions,
            new_exact,
            far_faces,[tuple(reversed(f)) for f in run.wall])
        del new_exact
        summary={k:value for k,value in v.items() if k not in ('exact_cells','interior_degree','witness_ids')}
        summary['interior_degree']=dict(passed=v['interior_degree']['passed'],unresolved=v['interior_degree']['unresolved'],
            witnesses=len(v['interior_degree']['values']),failures=[dict(cell=v['witness_ids'][i],**r)
                for i,r in enumerate(v['interior_degree']['values']) if r is not None and r['degree']!=1])
        summary['seconds']=time.perf_counter()-started
        durable_json(folder/'VOLUME.json',summary)
        if not (v['interior_degree']['passed'] and v['exact_volume_balance'] and v['positive_cells']):
            raise ValueError('CompleteCoreVolumeFailure')
        measured('exact_volume_and_full_degree')
    else:
        certificate=dict(status='not_performed',scope='New core/core and core/shell face relations',
            reason='Explicit backend-trusted background contract',
            independent_coverage_status='not_performed')
        summary=dict(status='not_performed',scope='Completed mesh exact volume balance and full global degree',
            reason='Explicit backend-trusted background contract')
        durable_json(folder/'CERTIFICATE.json',certificate)
        durable_json(folder/'VOLUME.json',summary)
        measured('omitted_assurance_reporting')
    # A real physical outer boundary needs its established skewness check,
    # not fabricated neighbour weights. Reuse the C1 Foundation routine.
    from capsule_core_control_audit import boundary_skewness
    byid={c['id']:c for c in additions}
    centres={i:t.geometry(points,byid[i])['centre']
             for i in {owners[key(f)][0][0] for f in far_faces}}
    values=boundary_skewness(points[np.asarray(far_faces)],
        np.array([centres[owners[key(f)][0][0]] for f in far_faces]))
    boundary=dict(maximum_skewness=float(values.max()),failures=np.flatnonzero(values>3.63636).tolist())
    durable_json(folder/'BOUNDARY_QUALITY.json',boundary)
    if boundary['failures'] or not np.isfinite(values).all():raise ValueError('PhysicalBoundaryQualityFailure')
    measured('physical_boundary_quality')
    if _handoff is None:run.live_acceptance.check(run, _preparation=_preparation)
    else:_handoff.require(run)
    if run.live_acceptance is not token:raise ValueError('ChangedLiveCoreParent')
    measured('final_live_parent')
    result=dict(passed=True,interface=audit,quality=q_summary,certificate=certificate,
        volume=summary,physical_boundary=boundary,prism_cells=len(run.topology.cells),core_cells=len(tets),
        points=len(points),inherited_shell=token.mesh,restart_supported=False,archived_publication=False)
    if graded_joined:
        result['whole_joined_geometry']=dict(report='JOINED_PROOFS/PROOFS.json',
            sha256=file_hash(folder/'JOINED_PROOFS/PROOFS.json'),**joined_proof)
        result['physical_boundary']=whole_boundary
    result['core_cells']=len(additions)
    if completion is not None:
        result['interface_completion']=dict(report='INTERFACE_COMPLETION.json',sha256=completion_sha256,
            modified_vertices=completion['modified_vertices'],final_existing_acceptance_passed=True,
            relative_embedding_premise=True,authored_report_identity=authored_completion)
        if completion.get('schema')=='V4CoreMeshCompletion/1':
            result['interface_completion'].update(topology_edits=completion['topology_edits'],
                remaining_raw_records=completion['remaining_raw_records'],
                original_core_cells=completion['original_core_cells'],final_core_cells=completion['final_core_cells'])
    if _full_joined:
        # Bulk repair is allowed only outside the protected transition. Check
        # the actual final global rows and coordinates, after every edit.
        expected_rows=mapping[tets[:_transition.cells_count]]
        protected_points=_transition._open()['points']
        if (not np.array_equal(mapped[:len(expected_rows)],expected_rows)
                or points[mapping[:protected_points[2][0]]].tobytes()!=protected_points[0]):
            raise ValueError('ChangedInheritedTransitionAfterBulkRepair')
        result['inherited_transition_preservation']=dict(passed=True,cells=_transition.cells_count,
            scope='Exact inherited point bytes and transition connectivity; complete oriented ownership and affected-neighbour gates also passed.')
        if points[:len(run.points)].tobytes()!=run.points.tobytes():raise ValueError('ChangedAcceptedPrismAfterBulkRepair')
        from finite_research.joined_diagnostics import measure
        sizes=measure(points,mapped,_transition.cells_count,_transition.front,_transition.triangles,owners,additions,check)
        durable_json(folder/'REALIZED_SIZING.json',sizes)
        result['realized_sizing']=dict(report='REALIZED_SIZING.json',sha256=file_hash(folder/'REALIZED_SIZING.json'))
        measured('realized_size_diagnostics')
    check()
    payload=complete_mesh_payload(points,[c['vertices'] for c in run.topology.cells.values()],mapped,run.wall,far_faces,additions,completion)
    if graded_joined:
        joined_proof['mesh_arrays']={name:array_hash(value) for name,value in payload.items()}
        durable_json(folder/'JOINED_PROOFS/PROOFS.json',joined_proof)
        result['whole_joined_geometry']=dict(report='JOINED_PROOFS/PROOFS.json',
            sha256=file_hash(folder/'JOINED_PROOFS/PROOFS.json'),**joined_proof)
    np.savez(folder/'COMPLETE_MESH.npz',**payload)
    if _full_joined:
        from immutable_state_access import guard
        _artifact_custody.enter_context(guard(folder/'COMPLETE_MESH.npz'))
        with np.load(folder/'COMPLETE_MESH.npz',allow_pickle=False) as restored:
            if set(restored.files)!=set(payload):raise ValueError('ChangedCompleteMeshReadbackSchema')
            for name,expected in payload.items():
                actual=restored[name]
                if actual.dtype!=expected.dtype or actual.shape!=expected.shape or actual.tobytes()!=expected.tobytes():
                    raise ValueError('ChangedCompleteMeshReadback',name)
        check()
    result['mesh_sha256']=file_hash(folder/'COMPLETE_MESH.npz')
    result['assurance']=dict(mode=assurance_mode,mesh_sha256=result['mesh_sha256'],
        qualified_prism_shell=token.mesh,
        retained_checks=['identity','boundary_preservation','joined_ownership','unchanged_shell',
            'mixed_cell_and_neighbour_quality','exact_local_positive_jacobian_and_volume','physical_boundary_quality'],
        completed_mesh_exact_proofs=('passed' if full_proofs else 'not_performed'),
        omitted_checks=([] if full_proofs else
            ['new_core_continuous_collision','independent_required_pair_coverage','exact_total_volume_balance','full_global_degree']),
        backend_assumption=(None if full_proofs else
            ('Pinned Gmsh/Netgen embeds and fills the initial supplied discrete cavity; exact closed-cavity replacement and interior-star edit records preserve that premise for accepted topology and coordinate edits; no independent completed-mesh collision or filling proof'
             if completion is not None and (completion['modified_vertices'] or completion.get('topology_edits',0)) else
             'Pinned Gmsh/Netgen embeds and fills the supplied discrete cavity; no independent completed-mesh collision or filling proof')))
    if completion is not None and (completion['modified_vertices'] or completion.get('topology_edits',0)):
        result['assurance']['relative_preservation']=dict(report_sha256=completion_sha256,
            original_points_sha256=completion['relative_preservation']['original_points_sha256'],
            final_points_sha256=completion['relative_preservation']['repaired_points_sha256'],
            conditional_on_initial_backend_embedding=True)
    measured('complete_mesh_write_and_hash')
    result['phase_seconds']=timings
    durable_json(folder/'QUALIFICATION.json',result)
    return result


def _wire_interface(metadata):
    """Commit the full live record; do not copy unused ancestry/version tables.

    Existing extract() still constructs and hashes the full source record.
    Arrays, source ownership and live-parent checks remain in this process.
    Preserve the small ID mapping for existing offline diagnostics. This is
    not a recoverable checkpoint or an independent proof of those tables.
    """
    result={k:v for k,v in metadata.items() if k not in ('ancestry','versions','identity','schema')}
    result.update(schema='V4LiveCoreInterfaceCommitment/1',source_schema=metadata['schema'],
                  full_metadata_identity=metadata['identity'],restart_supported=False)
    return result


@obs.observed('background.execute','background')
def execute(run,case_path,folder,expected_backend,*,_handoff=None,_preparation=None,_transition_only=False,_downstream_policy=None,_outer_check=None,_stage_binding=None):
    from contextlib import ExitStack
    from standalone_case import load
    if _handoff is None or not callable(_outer_check) or not callable(getattr(_outer_check,'remaining_seconds',None)):raise ValueError('ExplicitDownstreamLiveStageRequired')
    _outer_check()
    case,_,pin=load(case_path)
    if type(_downstream_policy) is not dict or type(_stage_binding) is not dict:raise ValueError('BoundDownstreamPolicyRequired')
    policy=_downstream_policy;bounds=validate_policy(policy);mode=assurance_mode(policy)
    from finite_research.execution_target import choice
    target=choice(_stage_binding)
    validate_execution_target(policy,_transition_only,execution_target=target)
    full_joined=target=='transition_background'
    if full_joined:
        from finite_research.prism_connection import terminal_fluid_verification
        terminal_fluid_verification(policy,_stage_binding,_handoff)
    shared=_handoff is not None
    if shared:
        from rolling_fast_state import BackgroundInputs
        if type(_handoff) is not BackgroundInputs or mode!='backend_trusted':raise ValueError('RoutineBackgroundHandoffRequired')
        _handoff.require_origin(run)
        if _handoff._open()['preparation'] is not _preparation:raise ValueError('ChangedBackgroundPreparationScope')
        if _preparation is not None:
            from background_preparation import require
            require(_preparation)
    elif _preparation is not None:raise ValueError('UnexpectedExternalBackgroundPreparation')
    folder=Path(folder);folder.mkdir(exist_ok=False);stage='live_interface';start=time.perf_counter();held=ExitStack()
    timings={};mark=start
    def measured(name):
        nonlocal mark
        now=time.perf_counter();timings[name]=now-mark;mark=now
    try:
        from background_preparation import held as preparation_owner
        preparation=_preparation if shared else held.enter_context(preparation_owner())
        handoff=_handoff
        if mode=='backend_trusted':
            if not shared:handoff=held.enter_context(run.live_acceptance.background_inputs(run,_preparation=preparation,_capture_volume=policy['schema']=='V4GradedBackground/6'))
            run=handoff.view
            measured('protected_parent_handoff_reuse' if shared else 'protected_parent_capture_and_check')
        token,x,tri,ids,volumes,terminal,metadata,centres=_interface(run,_preparation=preparation,_handoff=handoff)
        if np.any(x<=bounds[:,0]) or np.any(x>=bounds[:,1]):raise ValueError('PrismFrontOutsideBackgroundDomain')
        prism_front=x;prism_tri=tri;transition=None
        if policy['schema']=='V4GradedBackground/6':
            from finite_stage_handoff import produce
            stage='tet_transition'
            from finite_stage_controls import finite_collar
            if finite_collar(policy['tet_transition']):
                from finite_stage_command import produce as produce_collar
                transition=held.enter_context(produce_collar(handoff,ids,tri,terminal,volumes,centres,
                    policy,bounds,folder,case['resources'],outer_check=_outer_check,stage_binding=_stage_binding))
            else:
                transition=held.enter_context(produce(handoff,ids,tri,terminal,volumes,centres,policy['tet_transition'],bounds,folder,case['resources']))
            x=transition.front;tri=transition.triangles;volumes=transition.volumes;centres=transition.centres
            measured('accepted_tet_transition')
        _outer_check();transition.require(handoff)
        fluid_assurance=None
        if 'transition_fluid_verification' in _stage_binding:
                from finite_research.prism_connection import terminal_fluid_verification
                from finite_research.fluid_assurance import validate_report
                verification=terminal_fluid_verification(policy,_stage_binding,handoff)
                fluid_assurance=validate_report(transition.report,verification,_stage_binding,
                    handoff.view.live_acceptance.mesh,file_hash(Path(__file__).resolve().parents[3]/'runtime/RELEASE.json'),transition.cells_count)
        if _transition_only:
            result=dict(passed=True,output_role='accepted_transition_component',
                transition_accepted=True,tetrahedra=transition.cells_count,
                report='TRANSITION.json',report_sha256=transition.report['report_sha256'],
                arrays_sha256=transition.report['arrays_sha256'],
                outer_bulk_neighbours_required=True,restart_supported=False,
                background_invoked=False,complete_mesh=None,
                seconds=time.perf_counter()-start,execution_phase_seconds=timings)
            if fluid_assurance is not None:
                attach_transition_assurance(result,transition.report,verification,fluid_assurance,_stage_binding)
            durable_json(folder/'RESULT.json',result)
            return result
        seeds=interface_seeds(policy,x,tri,volumes,centres)
        measured('live_interface_and_seeds')
        np.savez(folder/'INPUT.npz',points=x,triangles=tri,seeds=seeds)
        locks=policy_backend(policy)
        if locks!=expected_backend:raise ValueError('CoreBackendChangedAfterPreflight')
        from standalone_tool import effective_resources
        outer_policy=dict(policy)
        if full_joined:
            # Accepted transition is the fixed inner boundary; outer worker owns only bulk.
            outer_policy.pop('tet_transition');outer_policy['schema']='V4GradedBackground/5'
        request=dict(outer_policy,locks=locks,files={'INPUT.npz':file_hash(folder/'INPUT.npz')},resources=effective_resources(case['resources']),
            case_sha256=pin,inherited_shell=token.mesh,interface=_wire_interface(transition.metadata if transition is not None else metadata),
            interface_seed_policy=policy.get('interface_seed_policy','equal_prism_volume/1'),
            interface_seed_array_sha256=array_hash(seeds),
            target_cap_not_realized_edge_guarantee=True)
        if full_joined:request['accepted_transition_binding']=dict(full_policy_identity=identity(policy),stage_binding=_stage_binding,joined_contract='FiniteTransitionJoinedExact/1')
        if transition is not None:
            request['files'].update({name:file_hash(folder/name) for name in ('TRANSITION.npz','TRANSITION.json')})
            request['terminal_owner_kind']='tetrahedron'
        if policy['schema'] in ('V4GradedBackground/4','V4GradedBackground/5','V4GradedBackground/6'):request['backend_method_identity']=locks['backend_method']
        durable_json(folder/'INPUT.json',request)
        stage='generation_and_optimization'
        import immutable_state_access as access
        held.enter_context(access.guard(folder/'INPUT.json'))
        held.enter_context(access.guard(folder/'INPUT.npz'))
        measured('guarded_worker_preparation')
        from finite_research.outer_worker import invoke,linux
        native_result=invoke(folder,['--input',linux(folder)],_outer_check)
        _outer_check()
        if native_result.get('backend_sha256')!=file_hash(folder/'BACKEND.json'):raise ValueError('ChangedOuterBackendProcessReadback')
        measured('worker_process')
        if (file_hash(folder/'INPUT.npz')!=request['files']['INPUT.npz'] or load(case_path)[2]!=pin
                or json.loads((folder/'INPUT.json').read_text(encoding='utf-8'))!=request):
            raise ValueError('ChangedBackgroundExecutionInput')
        if policy_backend(policy)!=locks:raise ValueError('ChangedCoreBackendDependencies')
        with ExitStack() as readback:
            readback.enter_context(access.guard(folder/'BACKEND.json'))
            backend=json.loads((folder/'BACKEND.json').read_text(encoding='utf-8'))
            if (backend.get('passed') is not True or backend['input_sha256']!=file_hash(folder/'INPUT.json')
                    or backend['locks']!=locks):raise ValueError('CoreBackendReadbackMismatch')
            from background_backend_method import validate_readback
            validate_readback(backend,locks)
            for name in ('NETGEN.npz','OUTER.npz'):
                readback.enter_context(access.guard(folder/'generated'/name))
                if file_hash(folder/'generated'/name)!=backend['files'][name]:raise ValueError('ChangedCoreBackendOutput',name)
            data=np.load(folder/'generated/NETGEN.npz');outer=np.load(folder/'generated/OUTER.npz')
            measured('authenticated_readback')
            stage='joined_qualification'
            core_points,core_tets=data['points'],data['tets']
            if transition is not None:
                core_points,core_tets=transition.merge(handoff,core_points,core_tets,outer['points'],outer['triangles'])
                x,tri=prism_front,prism_tri
            result=qualify(run,core_points,core_tets,x,tri,ids,terminal,
                          outer['points'],outer['triangles'],folder,token,bounds=bounds,_preparation=preparation,
                          assurance_mode=mode,_handoff=handoff,interface_completion=policy.get('interface_completion'),_transition=transition,
                          _full_joined=full_joined,_outer_check=_outer_check)
        measured('joined_qualification')
        if transition is not None:result['tet_transition']=dict(passed=True,tetrahedra=transition.cells_count,report='TRANSITION.json',sha256=transition.report['report_sha256'],arrays='TRANSITION.npz',arrays_sha256=transition.report['arrays_sha256'],protected_during_bulk_repair=True)
        if full_joined:
            result.update(execution_target=target,background_invoked=True,transition_accepted=True,
                joined_contract='FiniteTransitionJoinedExact/1')
            attach_transition_assurance(result,transition.report,verification,fluid_assurance,_stage_binding)
            result['assurance']['transition_fluid_assurance']=fluid_assurance
            from finite_research.execution_target import validate_result
            validate_result(result,_stage_binding)
        _outer_check()
        result['seconds']=time.perf_counter()-start
        result['execution_phase_seconds']=timings
        if handoff is not None:
            result['parent_input_custody']=dict(handoff.summary,shared_with_terminal_export=shared,
                background_full_parent_verifications=0 if shared else 1)
        if preparation is not None:
            result['bookkeeping_implementation']=dict(source_sha256=preparation.source_sha256,
                binary_sha256=preparation.binary_sha256,build_sha256=preparation.build_sha256)
        if full_joined:durable_json(folder/'QUALIFICATION.json',result)
        if full_joined:
            result['qualification_sha256']=file_hash(folder/'QUALIFICATION.json')
        durable_json(folder/'RESULT.json',result)
        return result
    except BaseException as error:
        try:durable_json(folder/'STOP.json',_stop_record(stage,error,time.perf_counter()-start,run.provider.layer))
        except BaseException as secondary:error.add_note('Background stop evidence: '+repr(secondary))
        raise
    finally:
        primary=sys.exc_info()[1]
        try:held.close()
        except BaseException as secondary:
            if primary is None:raise
            primary.add_note('Background custody cleanup: '+repr(secondary))

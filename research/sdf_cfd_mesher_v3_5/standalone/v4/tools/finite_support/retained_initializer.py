"""One owned retained initializer, with no production issuer or Poll dispatch."""
from pathlib import Path
from contextlib import ExitStack
from datetime import datetime,timezone,timedelta
import json,subprocess,time
import numpy as np
from sdf_cfd_mesher_v4.common.authority import identity,file_hash,array_hash
from finite_support import retained_native_contract as contract

def backend_allowance(check):
    check()
    remaining=check.remaining_seconds()
    if remaining<=1:raise TimeoutError('NoRetainedInitializerBudget')
    return min(300.,remaining)

def bounded_command(command,seconds,receipt,*,linux_deadline=None):
    if not 1<seconds<=3600 or command[:2]!=['wsl.exe','-e']:
        raise ValueError('ExplicitRetainedNativeStageBudgetRequired')
    from owned_runtime import V4
    def linux(value):
        value=str(value).replace('\\','/');return '/mnt/'+value[0].lower()+value[2:]
    absolute=[] if linux_deadline is None else ['--deadline-monotonic',str(linux_deadline)]
    return command[:2]+['/usr/bin/python3.10','-B',linux(V4/'tools/finite_support/native/stage_deadline.py'),
        str(seconds),linux(receipt)]+absolute+command[2:]

def validate_retirement(record,allowance,*,linux_deadline=None):
    if (record.get('schema')!='RetainedNativeStageRetirement/1' or record.get('retired') is not True
            or record.get('live_members')!=[] or record.get('limit_seconds')!=allowance
            or record.get('deadline_met') is not True or not 0<=record.get('seconds',float('inf'))<=allowance):
        raise ValueError('NativeStageRetirementOrDeadlineUnverified')
    if linux_deadline is not None and record.get('deadline_monotonic')!=linux_deadline:
        raise ValueError('NativeStageAbsoluteDeadlineChanged')

def initialize(state,outer,seeds,policy,spec,historical_request,owner_bindings,helpers,construction,folder,check):
    import native_scope
    native_scope.require_transport()
    from immutable_state_access import guard
    from finite_research.inputs import native_sources
    from owned_runtime import V4
    helpers['finite_stage_background'].validate_policy(policy)
    folder=Path(folder);folder.mkdir();started=time.perf_counter()
    cap=state['original'][state['cap_ids']];faces=state['cap_faces'];bounds=np.asarray(policy['bounds_m'])
    if np.any(outer<=bounds[:,0]) or np.any(outer>=bounds[:,1]):raise ValueError('RetainedInitializerOutsideWorld')
    distance=policy['tet_transition']['envelope']['nominal_distance_m']
    boundary=construction.admit_boundary(cap,faces,outer,distance,check,folder,require_shape_targets=False)
    boundary['scope']='STRICT_SEED_BOUND_ONLY_NOT_NOMINAL_DOMAIN_LIMIT'
    construction.d.publish(folder/'BOUNDARY_ADMISSION.json',boundary)
    backend=folder/'backend';backend.mkdir()
    arrays=dict(cap=cap,faces=faces,outer=outer,seeds=seeds)
    np.savez(backend/'INPUT.npz',**arrays)
    allowance=backend_allowance(check)
    request=dict(schema=contract.REQUEST,context=contract.CONTEXT,geometry_scope=contract.SCOPE,
        policy=policy,policy_identity=identity(policy),policy_role='SIZING_AND_TARGETS_ONLY_NOT_CONSTRUCTION_DISPATCH',
        source_pins=native_sources(),input_sha256=file_hash(backend/'INPUT.npz'),
        growth_per_m=policy['growth_per_m'],maximum_size_m=policy['maximum_size_m'],seed_policy=policy['interface_seed_policy'],
        input_points_hash=array_hash(state['original']),prism_centres_hash=historical_request['prism_centres_hash'],
        prism_volumes_hash=historical_request['prism_volumes_hash'],seed_hash=array_hash(seeds),
        array_identities={name:array_hash(value) for name,value in arrays.items()},
        boundary_admission_sha256=file_hash(folder/'BOUNDARY_ADMISSION.json'),control=False,
        historical_inputs=spec['inputs'],historical_inputs_identity=identity(spec['inputs']),
        historical_source_release_sha256=spec['historical_source_release_sha256'],
        source_release_sha256=file_hash(V4/'runtime/RELEASE.json'),
        owner_bindings=owner_bindings,owner_bindings_identity=identity(owner_bindings),
        backend_seconds=allowance,expires_utc=(datetime.now(timezone.utc)+timedelta(seconds=allowance)).isoformat())
    contract.validate_request(request);construction.d.publish(backend/'REQUEST.json',request)
    input_hash=file_hash(backend/'INPUT.npz');request_hash=file_hash(backend/'REQUEST.json')
    with ExitStack() as held:
        for name in ('INPUT.npz','REQUEST.json'):held.enter_context(guard(backend/name))
        contract.verify_inputs(request,arrays,backend,V4)
        helpers['finite_stage_command'].verify_inputs(backend,arrays,request)
        allowance=min(allowance,backend_allowance(check))
        command=bounded_command(helpers['finite_stage_command'].native_command(backend),allowance,folder/'NATIVE_STAGE_RETIREMENT.json')
        construction.d.publish(folder/'DISPATCH.json',dict(backend_seconds=allowance,remaining_payload_seconds=check.remaining_seconds(),
            maximum_fills=1,native_command=command,linux_deadline_enforcement='Bounded process-group retirement with PID start identity and pidfd signalling'))
        with (backend/'stdout.txt').open('x') as stdout,(backend/'stderr.txt').open('x') as stderr:
            process=subprocess.run(command,stdout=stdout,stderr=stderr,timeout=allowance+2)
        retirement=json.loads((folder/'NATIVE_STAGE_RETIREMENT.json').read_text())
        validate_retirement(retirement,allowance)
        check()
        held.enter_context(guard(backend/'BACKEND.json'))
        receipt=json.loads((backend/'BACKEND.json').read_text())
        if process.returncode or receipt.get('passed') is not True:raise ValueError('RetainedInitializerBackendFailure',process.returncode,receipt.get('error'))
        contract.validate_receipt(receipt,request,request_hash)
        for name,digest in receipt['files'].items():
            held.enter_context(guard(backend/name))
            if file_hash(backend/name)!=digest:raise ValueError('ChangedRetainedInitializerOutput',name)
        if file_hash(backend/'INPUT.npz')!=input_hash or file_hash(backend/'REQUEST.json')!=request_hash:
            raise ValueError('ChangedRetainedInitializerCustody')
        contract.verify_inputs(request,arrays,backend,V4)
        with np.load(backend/'NETGEN.npz',allow_pickle=False) as archive:
            points,tets=archive['points'],archive['tets']
        unused,inner,far=helpers['finite_stage_background'].compare_boundary(points,tets,cap,faces,outer,faces)
        oldcells=state['cells'][:-len(state['additions'])];oldids={cell['id'] for cell in oldcells}
        oldowners={key:[(owner,face) for owner,face in uses if owner in oldids] for key,uses in state['owners'].items()}
        oldowners={key:uses for key,uses in oldowners.items() if uses}
        terminal=np.asarray([oldowners[tuple(sorted(face))][0][0] for face in state['cap_ids'][faces]],dtype='<i8')
        metadata=dict(cap_ids=state['cap_ids'],cap_triangles=faces,terminal_owners=terminal)
        inherited=construction.inherited_digest(oldcells,oldowners,check)
        result,attachment=construction.materialize(dict(points=state['original']),metadata,oldcells,oldowners,points,tets,inner,far,check)
        if not attachment['passed']:raise ValueError('RetainedInitializerAttachmentFailure')
        result.update(outer_vertex_ids=result['backend_to_global'][far],wall=state['wall'],bounds=bounds,
            controls=policy['tet_transition'],producer=contract.CONTEXT,provenance=dict(context=contract.CONTEXT,
                request_sha256=request_hash,backend_receipt_sha256=file_hash(backend/'BACKEND.json')))
        result['inherited_preservation']=construction.verify_inherited_join(result,oldcells,oldowners,inherited,check)
        for name,digest in receipt['files'].items():
            if file_hash(backend/name)!=digest:raise ValueError('ChangedRetainedInitializerFinalOutput',name)
    return result,dict(schema='RetainedInitializerReadback/1',context=contract.CONTEXT,
        initializer_completed=True,scientific_acceptance=False,live_parent=False,lease_issued=False,
        fluid_proof='NOT_PERFORMED',request_sha256=request_hash,backend_receipt_sha256=file_hash(backend/'BACKEND.json'),
        backend_seconds=receipt['backend_seconds'],whole_seconds=time.perf_counter()-started,attachment=attachment)

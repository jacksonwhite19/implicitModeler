"""Consume the normal CURRENT descriptor and actual published mixed-cell arrays."""
from contextlib import ExitStack,nullcontext
from pathlib import Path
import json,os
import numpy as np
from sdf_cfd_mesher_v4.common.authority import file_hash,array_hash,identity
from immutable_state_access import guard

def publish(root,current,check,expected):
    from path_preserving_deduplication import durable_json,atomic_replace
    root=Path(root);pointer=root/'CURRENT.json'
    previous=None
    if pointer.exists():
        with guard(pointer):previous=pointer.read_bytes()
        if json.loads(previous).get('mesh')!=expected['parent_prism_mesh']:
            raise ValueError('ChangedPreviousCurrentParent')
    committed=False
    try:
        with ExitStack() as held:
            consumed=consume(root,current,check,expected,custody=held)
            check()
            durable_json(pointer,current);committed=True
            held.enter_context(guard(pointer))
            readback=json.loads(pointer.read_text());pointer_hash=file_hash(pointer)
            if readback!=current:raise ValueError('ChangedGradedCurrentReadback')
            if consume(root,readback,check,expected,custody=held)!=consumed:
                raise ValueError('ChangedGradedConsumerReadback')
            check()
            if file_hash(pointer)!=pointer_hash:raise ValueError('ChangedGradedCurrentDuringConsumption')
            durable_json(root/'GRADED_CONSUMER.json',dict(consumed,current_sha256=pointer_hash))
        return consumed
    except BaseException as error:
        if committed:
            if previous is None:
                durable_json(pointer,dict(schema='V4FailedGradedPublication/1',passed=False,complete_mesh=None,
                    error=repr(error),published_scientific_authority=False))
            else:
                pending=root/'CURRENT.rollback.pending'
                with pending.open('wb') as stream:
                    stream.write(previous);stream.flush();os.fsync(stream.fileno())
                atomic_replace(pending,pointer)
        durable_json(root/'GRADED_PUBLISH_FAILURE.json',dict(passed=False,error=repr(error),
            previous_current_restored=committed and previous is not None))
        durable_json(root/'GRADED_CONSUMER.json',dict(passed=False,error=repr(error),
            current_sha256=file_hash(pointer) if pointer.exists() else None))
        raise

def consume(root,current,check,expected,*,custody=None):
    root=Path(root).resolve()
    if (type(expected) is not dict or set(expected)!={'stage_binding','source_release_sha256','parent_prism_mesh'}
            or current.get('mesh')!=expected['parent_prism_mesh']
            or expected['stage_binding'].get('parent_prism_mesh')!=expected['parent_prism_mesh']):
        raise ValueError('IndependentCurrentParentExpectationRequired')
    with (ExitStack() if custody is None else nullcontext(custody)) as held:
        bindings={}
        def path(relative):
            value=(root/relative).resolve()
            if not value.is_relative_to(root):raise ValueError('PublishedPathOutsideRun')
            return value
        def bound(relative,digest):
            check();value=path(relative);held.enter_context(guard(value))
            if file_hash(value)!=digest:raise ValueError('ChangedPublishedArtifact',relative)
            bindings[value]=digest;check()
            return value
        binding=json.loads(bound(current['stage_binding_path'],current['stage_binding_sha256']).read_text())
        if binding!=expected['stage_binding']:raise ValueError('ChangedPublishedStageExpectation')
        target=binding.get('execution_target')
        if (current.get('execution_target')!=target or target not in ('transition','transition_background')
                or current.get('transition_accepted') is not True
                or current.get('background_invoked') is not (target=='transition_background')
                or (current.get('complete_mesh') is not None)!=(target=='transition_background')):
            raise ValueError('ChangedPublishedExecutionTarget')
        def transition(descriptor,prefix):
            from finite_research.fluid_assurance import validate_report
            report=json.loads(bound(str(prefix/descriptor['report']),descriptor['sha256']).read_text())
            validate_report(report,binding['transition_fluid_verification'],binding,expected['parent_prism_mesh'],
                expected['source_release_sha256'],descriptor['tetrahedra'])
            if (report.get('schema')!='V4TetLayerAcceptance/5'
                    or report.get('arrays_sha256')!=descriptor['arrays_sha256']
                    or report.get('transition_cells')!=descriptor['tetrahedra']):
                raise ValueError('AcceptedGradedArchiveBindingRequired')
            placement=report['placement'];domain=placement['fluid_domain_evidence'];base=prefix/'FINITE_COLLAR'
            construction=json.loads(bound(str(base/'CONSTRUCTION.json'),domain['construction_sha256']).read_text())
            if construction!=placement['construction_status']:raise ValueError('ChangedPublishedConstructionStatus')
            bound(str(base/'PREPARATION.json'),domain['preparation_sha256'])
            for name,digest in domain['final_boundary_receipts'].items():
                value=json.loads(bound(str(base/name),digest).read_text())
                if value.get('geometry')!=placement['geometry'] or value.get('wall_contract')!=placement['wall_contract']:
                    raise ValueError('ChangedPublishedTransitionBoundary',name)
            archive=bound(str(prefix/descriptor['arrays']),descriptor['arrays_sha256'])
            fields={'points','tetrahedra','front','triangles','cap_ids','cap_triangles','outer_vertex_ids',
                'vertex_envelope_vertex_ids','vertex_envelope_triangle_ids','vertex_envelope_weights'}
            with np.load(archive,allow_pickle=False) as arrays:
                check()
                if (set(arrays.files)!=fields or len(arrays['tetrahedra'])!=descriptor['tetrahedra']
                        or array_hash(arrays['points'])!=report['points_sha256']
                        or array_hash(arrays['tetrahedra'])!=report['tets_sha256']):
                    raise ValueError('ChangedPublishedTransitionArrays')
                counts=dict(points=len(arrays['points']),tetrahedra=len(arrays['tetrahedra']))
            return report,counts
        complete=current.get('complete_mesh')
        if complete is None:
            descriptor=current['transition']
            if Path(descriptor['path']).parent!=Path(descriptor['acceptance']).parent:
                raise ValueError('ChangedTransitionArchiveDirectory')
            report,counts=transition(dict(report=Path(descriptor['acceptance']).name,sha256=descriptor['acceptance_sha256'],
                arrays=Path(descriptor['path']).name,arrays_sha256=descriptor['sha256'],tetrahedra=descriptor['tetrahedra']),
                Path(descriptor['acceptance']).parent)
            if (descriptor.get('fluid_assurance')!=report['fluid_assurance']
                    or descriptor.get('acceptance_schema')!=report['schema']
                    or descriptor.get('transition_fluid_verification')!=report['transition_fluid_verification']):
                raise ValueError('ChangedPublishedTransitionDescriptor')
            result=dict(schema='V4NormalGradedConsumer/1',passed=True,role='transition',complete_mesh=False,**counts)
        else:
            qualification_path=bound(complete['qualification'],complete['qualification_sha256'])
            qualification=json.loads(qualification_path.read_text())
            from finite_research.execution_target import validate_result
            domain=qualification['fluid_assurance']['domain_basis']
            validate_result(qualification,binding)
            from finite_research.fluid_assurance import validate_component
            validate_component(qualification,qualification['transition_fluid_verification'],binding)
            if (domain['parent_prism_mesh']!=qualification['inherited_shell']
                    or qualification['inherited_shell']!=expected['parent_prism_mesh']
                    or qualification['mesh_sha256']!=complete['sha256']
                    or current.get('transition')!=qualification['tet_transition']
                    or complete.get('assurance')!=qualification['assurance']
                    or current.get('transition_fluid_assurance')!=qualification['fluid_assurance']):
                raise ValueError('ChangedPublishedJoinedParentOrMesh')
            proof=qualification['whole_joined_geometry']
            prefix=qualification_path.parent.relative_to(root)
            transition_report,transition_counts=transition(qualification['tet_transition'],prefix)
            if transition_report['fluid_assurance']!=qualification['fluid_assurance']:
                raise ValueError('ChangedPublishedJoinedTransitionAssurance')
            proof_path=bound(str(prefix/proof['report']),proof['sha256'])
            record=json.loads(proof_path.read_text())
            if record!={name:value for name,value in proof.items() if name not in ('report','sha256')}:
                raise ValueError('ChangedPublishedProofManifest')
            for name,receipt in record['receipts'].items():
                relative=proof_path.parent.relative_to(root)/(name.upper()+'.json')
                value=json.loads(bound(str(relative),receipt['sha256']).read_text())
                if (identity(value)!=receipt['identity'] or value.get('geometry_identity')!=record['geometry_identity']
                        or value.get('wall_contract')!=record['wall_contract']):
                    raise ValueError('ChangedPublishedGeometryReceipt',name)
            mesh=bound(complete['path'],complete['sha256'])
            with np.load(mesh,allow_pickle=False) as arrays:
                if set(arrays.files)!=set(record['mesh_arrays']):raise ValueError('ChangedPublishedMixedPayload')
                for name,digest in record['mesh_arrays'].items():
                    check()
                    if array_hash(arrays[name])!=digest:raise ValueError('ChangedPublishedMixedArray',name)
                if (arrays['points'].ndim!=2 or arrays['points'].shape[1]!=3
                        or arrays['prisms'].ndim!=2 or arrays['prisms'].shape[1]!=6
                        or arrays['tetrahedra'].ndim!=2 or arrays['tetrahedra'].shape[1]!=4):
                    raise ValueError('InvalidPublishedMixedCellLayout')
                counts=dict(points=len(arrays['points']),prisms=len(arrays['prisms']),tetrahedra=len(arrays['tetrahedra']))
            if counts['points']!=record['points'] or counts['prisms']+counts['tetrahedra']!=record['cells']:
                raise ValueError('ChangedPublishedMixedCellCount')
            result=dict(schema='V4NormalGradedConsumer/1',passed=True,role='joined',complete_mesh=True,
                geometry_identity=record['geometry_identity'],proof_receipts=len(record['receipts']),**counts)
        check()
        for value,digest in bindings.items():
            if file_hash(value)!=digest:raise ValueError('ChangedPublishedArtifactDuringConsumption')
        return result


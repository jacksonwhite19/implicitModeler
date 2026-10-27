"""Generic live-parent support construction using the existing graded algorithm."""
from contextlib import ExitStack
from fractions import Fraction
from pathlib import Path
import time
import numpy as np
from sdf_cfd_mesher_v4.common.authority import identity,array_hash,file_hash
from path_preserving_deduplication import durable_json
from finite_research.flexible_controls import graded,validate,identities,GRADED_CONSTRUCTION
from finite_support.stored_wall_contract import SCHEMA as WALL,verify as verify_wall

def prepare(points,ids,triangles,terminal,volumes,centres,policy,bounds,check):
    from finite_stage_background import interface_seeds,validate_policy
    validate_policy(policy);controls=validate(policy['tet_transition'])
    if not graded(controls):raise ValueError('ExplicitGradedConstructionRequired')
    if len(terminal)!=len(triangles):raise ValueError('IncompleteGradedTerminalOwners')
    seeds=interface_seeds(policy,points[ids],triangles,volumes,centres)
    check()
    return dict(schema='V4GradedSupportPreparation/1',constructor=GRADED_CONSTRUCTION,
        **identities(controls),nominal_distance_m=controls['envelope']['nominal_distance_m'],
        completion='unweighted_current_front_original_cap_displacement_median_at_least_nominal',
        nominal_is_hard_maximum=False,sizing=dict(growth_per_m=policy['growth_per_m'],
        maximum_size_m=policy['maximum_size_m'],seed_policy=policy['interface_seed_policy'],seed_sha256=array_hash(seeds)),
        points_sha256=array_hash(points),cap_sha256=array_hash(points[ids]),triangles_sha256=array_hash(triangles),
        volumes_sha256=array_hash(volumes),centres_sha256=array_hash(centres),bounds_sha256=array_hash(bounds),
        generation_invocations=0,scientific_acceptance=False)

def require_complete(report):
    if (report.get('extent_reached') is not True or report.get('stop_reason')!='NOMINAL_MEDIAN_REACHED'
            or type(report.get('increments')) is not int or report['increments']<=0):
        raise ValueError('PartialGradedConstruction',report.get('stop_reason'))
    records=report.get('records',[])
    if (len(records)!=report['increments'] or not records
            or records[-1].get('displacement_m',{}).get('median',float('-inf'))<report['nominal_distance_m']):
        raise ValueError('UnsubstantiatedGradedNominalCompletion')

def boundary(state,construction,check,folder):
    from finite_support.cap_relative_placement import witness
    from shape_transition_builder import envelope_binding
    points=state['points'];cap=state['original'][state['cap_ids']];faces=state['cap_faces']
    outer=points[state['outer_vertex_ids']]
    witnesses,placement=witness(points[state['compact_ids']],cap,faces,state['controls']['envelope'],state['bounds'],check)
    state['vertex_envelope']=witnesses
    winding=verify_wall(state['cells'],state['outer'],state['wall'],WALL,check)
    if not winding['passed']:raise ValueError('GradedStoredWallOrientationFailure',winding)
    surfaces=dict(cap=construction.surface(cap,faces,check),outer=construction.surface(outer,faces,check))
    boundary_points=np.vstack((cap,outer));boundary_faces=np.vstack((faces,faces+len(cap)))
    collision=construction.collision(boundary_points,boundary_faces,range(len(boundary_faces)),check)
    if not collision['passed'] or not collision['independent_coverage_passed']:
        raise ValueError('GradedFinalBoundaryCollisionFailure')
    check()
    enclosure=construction.degree(outer,list(map(tuple,faces)),
        [tuple(Fraction(float(value)) for value in row) for row in cap])
    check()
    if not enclosure['passed'] or enclosure['unresolved'] or len(enclosure['values'])!=len(cap):
        raise ValueError('GradedFinalCapEnclosureFailure')
    if any(value['degree']!=1 for value in enclosure['values']):raise ValueError('GradedEnclosureDegreeFailure')
    geometry=dict(points_sha256=array_hash(points),tets_sha256=array_hash(state['tets']),
        outer_sha256=array_hash(outer),boundary_sha256=array_hash(state['outer']),wall_sha256=array_hash(state['wall']))
    records={'FINAL_BOUNDARY_ADMISSION.json':dict(passed=True,surfaces=surfaces,wall=winding),
        'FINAL_BOUNDARY_COLLISION.json':collision,'FINAL_BOUNDARY_DEGREE.json':enclosure}
    for name,value in records.items():durable_json(folder/name,dict(value,geometry=geometry,wall_contract=WALL))
    return dict(geometry=geometry,envelope=placement,vertex_envelope_binding=envelope_binding(state),
        final_boundary_receipts={name:file_hash(folder/name) for name in records},wall_contract=WALL)

def construct(points,cells,owners,ids,triangles,terminal,volumes,centres,policy,bounds,folder,check,parent,*,stage_binding):
    from finite_stage_command import require_live_parent,reviewed_construction,source_pins,V4
    from finite_stage_background import interface_seeds
    from finite_research.prism_connection import terminal_fluid_verification
    from finite_support.graded_support_advance import construct as advance
    from generic_core_sizing import CoreSizingField
    from immutable_state_access import guard
    import finite_stage_adapter as adapter
    require_live_parent(parent);terminal_fluid_verification(policy,stage_binding,parent)
    preparation=prepare(points,ids,triangles,terminal,volumes,centres,policy,bounds,check)
    folder=Path(folder);folder.mkdir();start=time.perf_counter()
    durable_json(folder/'PREPARATION.json',preparation)
    controls=validate(policy['tet_transition']);original_hash=array_hash(points);pins=source_pins()
    with ExitStack() as held,reviewed_construction() as construction:
        held.enter_context(guard(V4/'runtime/RELEASE.json'))
        for path in pins:held.enter_context(guard(Path(path)))
        if source_pins()!=pins:raise ValueError('ChangedGradedSourceManifest')
        wall=np.asarray(parent.view.wall,dtype='<i8')
        initial_wall=verify_wall(cells,ids[triangles],wall,WALL,check)
        if not initial_wall['passed']:raise ValueError('GradedOriginalWallContractFailure',initial_wall)
        digest=construction.inherited_digest(cells,owners,check)
        seeds=interface_seeds(policy,points[ids],triangles,volumes,centres)
        field=CoreSizingField(points[ids][triangles],seeds,g=policy['growth_per_m'],h_max=policy['maximum_size_m'])
        base=dict(original=points,cells=cells,owners=owners,cap_ids=ids,cap_faces=triangles,
            protected_cell_ids=[cell['id'] for cell in cells],protected_digest=digest,
            wall=wall,bounds=bounds,sizing_authority=preparation['sizing'])
        def publish(name,value):
            durable_json(folder/(name+'.json'),construction.d.plain(value))
            if getattr(check,'repair_progress',None) is not None:
                check.repair_progress(dict(stage='graded_support',record=name,
                    stop_reason=value.get('stop_reason'),increments=value.get('increments'),
                    displacement_m=value.get('displacement_m'),extent_reached=value.get('extent_reached'),
                    proof_status='NOT_YET_ADMITTED'))
        state,report=advance(base,field,controls['envelope']['nominal_distance_m'],
            min(controls['seconds'],check.remaining_seconds()),adapter,construction,check,publish)
        status=dict(schema='V4GradedConstructionStatus/1',status='COMPLETE_CONSTRUCTION' if report['extent_reached'] else 'PARTIAL',
            constructor=GRADED_CONSTRUCTION,construction=report,**identities(controls),
            parent_prism_mesh=parent.view.live_acceptance.mesh,scientific_acceptance=False)
        durable_json(folder/'CONSTRUCTION.json',status)
        require_complete(report)
        if state is None:raise ValueError('MissingCompletedGradedState')
        state['controls']=controls;state['producer']=GRADED_CONSTRUCTION
        quality=adapter.quality(state['points'],state['cells'],state['owners'],
            {cell['id'] for cell in state['cells']},state['outer'],parent)
        shape=construction.TARGET.evaluate(state)
        durable_json(folder/'QUALITY.json',quality);durable_json(folder/'SHAPE.json',shape)
        if not quality['passed'] or not shape['passed']:raise ValueError('GradedFinalQualityOrSixShapeFailure')
        final=boundary(state,construction,check,folder)
        preservation=construction.verify_inherited_join(state,cells,owners,digest,check)
        if array_hash(points)!=original_hash or state['points'][:len(points)].tobytes()!=points.tobytes():
            raise ValueError('ChangedGradedPrismPrefix')
        if source_pins()!=pins:raise ValueError('ChangedGradedSourceManifest')
        require_live_parent(parent);terminal_fluid_verification(policy,stage_binding,parent);check()
        domain=dict(schema='ConstructedGradedDomain/1',construction_sha256=file_hash(folder/'CONSTRUCTION.json'),
            preparation_sha256=file_hash(folder/'PREPARATION.json'),final_boundary_receipts=final['final_boundary_receipts'])
        placement=dict(passed=True,quality=quality,shape=shape,controls_identity=identity(controls),
            points_sha256=array_hash(state['points']),tets_sha256=array_hash(state['tets']),
            original_prism_points_sha256=original_hash,parent_prism_mesh=parent.view.live_acceptance.mesh,
            source_manifest_sha256=file_hash(V4/'runtime/RELEASE.json'),construction_status=status,
            fluid_domain_evidence=domain,inherited_join=preservation,**final,
            seconds=time.perf_counter()-start,outer_bulk_neighbours_required=True)
        durable_json(folder/'NUMERICAL_RESULT.json',placement)
        return state,placement


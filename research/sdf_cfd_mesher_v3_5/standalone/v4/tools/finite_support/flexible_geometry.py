"""Fresh final front evidence; the existing full admission owner remains required."""
from fractions import Fraction
import numpy as np
from finite_research.flexible_controls import validate,identities
from finite_support.cap_relative_placement import witness
from finite_support.shared_front_poll import Poll
from shape_transition_builder import envelope_binding
from sdf_cfd_mesher_v4.common.authority import array_hash,identity,file_hash
from path_preserving_deduplication import durable_json


def construct(state,construction,folder,check,quality,parent,search_seconds):
    controls=validate(state['controls']);policy=controls['envelope']
    selected={cell['id'] for cell in state['additions']}
    selected.update(owner for face in state['cap_ids'][state['cap_faces']]
        for owner,unused in state['owners'][tuple(sorted(face))])
    initial=quality(state['points'],state['cells'],state['owners'],selected,state['outer'],parent)
    if not initial['passed']:raise ValueError('FlexibleHardQAInitializationFailure')
    q0=dict(points_sha256=array_hash(state['points']),tets_sha256=array_hash(state['tets']),
        outer_sha256=array_hash(state['points'][state['outer_vertex_ids']]),
        boundary_sha256=array_hash(state['outer']))
    poll=Poll(state,check,lambda points,cells,owners,active,outer:
        quality(points,cells,owners,active,outer,parent))
    points,search=poll.run(search_seconds)
    durable_json(folder/'FRONT_SEARCH.json',search)
    if not search['passed']:raise ValueError('FlexibleFrontSearchStopped',search['stop_reason'])
    state['points']=points
    final=quality(points,state['cells'],state['owners'],selected,state['outer'],parent)
    shape=construction.TARGET.evaluate(state)
    if not final['passed'] or not shape['passed']:raise ValueError('FlexibleFinalPopulationOrQualityFailure')
    cap=state['original'][state['cap_ids']];faces=state['cap_faces']
    outer=points[state['outer_vertex_ids']]
    placement,vertex_report=witness(points[state['compact_ids']],cap,faces,policy,state['bounds'],check)
    state['vertex_envelope']=placement
    boundary=dict(inner=construction.surface(cap,faces,check),outer=construction.surface(outer,faces,check))
    all_points=np.vstack((cap,outer));all_faces=np.vstack((faces,faces+len(cap)))
    collision=construction.collision(all_points,all_faces,range(len(all_faces)),check)
    if not collision['passed'] or not collision['independent_coverage_passed']:
        raise ValueError('FlexibleFinalBoundaryCollisionFailure')
    check()
    enclosure=construction.degree(outer,list(map(tuple,faces)),
        [tuple(Fraction(float(value)) for value in row) for row in cap])
    check()
    if not enclosure['passed']:raise ValueError('FlexibleFinalCapEnclosureFailure')
    q1=dict(points_sha256=array_hash(points),tets_sha256=array_hash(state['tets']),
        outer_sha256=array_hash(outer),boundary_sha256=array_hash(state['outer']))
    collision=dict(collision,q1=q1)
    enclosure=dict(enclosure,q1=q1)
    boundary.update(passed=True,q1=q1,vertex_placement_identity=identity(vertex_report),
        collision_identity=identity(collision),enclosure_identity=identity(enclosure),**identities(controls))
    for name,record in [('FINAL_BOUNDARY_COLLISION.json',collision),('FINAL_BOUNDARY_DEGREE.json',enclosure),
            ('FINAL_BOUNDARY_ADMISSION.json',boundary)]:durable_json(folder/name,record)
    delta=outer-cap;distance=np.linalg.norm(delta,axis=1);nominal=policy['nominal_distance_m']
    departure=np.abs(distance-nominal);maximum=int(np.argmax(departure))
    cap_triangles=cap[faces]
    normal=np.cross(cap_triangles[:,1]-cap_triangles[:,0],cap_triangles[:,2]-cap_triangles[:,0])
    depths=np.einsum('ij,ij->i',(outer[faces]-cap_triangles).mean(axis=1),normal/np.linalg.norm(normal,axis=1)[:,None])
    return dict(quality=final,shape=shape,envelope=vertex_report,search=search,q0=q0,q1=q1,
        vertex_envelope_binding=envelope_binding(state),extent_policy=policy,
        extent_policy_identity=identity(policy),quality_targets_identity=identity(controls['quality_targets']),
        final_boundary_receipts={name:file_hash(folder/name) for name in
            ('FINAL_BOUNDARY_COLLISION.json','FINAL_BOUNDARY_DEGREE.json','FINAL_BOUNDARY_ADMISSION.json')},
        distance_diagnostics=dict(paired_displacement_quantiles_m=np.quantile(distance,[0,.01,.5,.99,1]).tolist(),
            below_nominal=int(np.count_nonzero(distance<nominal)),at_nominal=int(np.count_nonzero(distance==nominal)),
            above_nominal=int(np.count_nonzero(distance>nominal)),maximum_absolute_departure_m=float(departure[maximum]),
            maximum_departure_vertex_id=int(state['outer_vertex_ids'][maximum]),
            cap_normal_face_depth_quantiles_m=np.quantile(depths,[0,.01,.5,.99,1]).tolist(),
            nearest_sdf_distance=False))

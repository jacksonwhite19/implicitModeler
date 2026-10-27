"""Owned shared-row transition construction; no external remesher dispatch.

Actual source/prism owners, conformal product topology and final acceptance are
reused. Metric objectives propose shape and size; they are not acceptance gates.
"""
from pathlib import Path
import time
import numpy as np
from tet_layer_controls import validate,seed_controls,physical_shape
from shape_transition_metric import copied_seed_metric
from shape_transition_domain import verify
from sdf_cfd_mesher_v4.common.authority import frozen

def conditioner(cap,faces,centres,volumes,rows):
    """Geometry-derived numerical scaling, not a requested mesh size field.

    Existing cap tangent/actual-owner-normal scales blend uniformly to their
    isotropic geometric mean across copied rows. No size growth. Physical
    objective is scale invariant. These tensors condition depth variables and
    order provisional failing stars; they are not an accepted size field.
    """
    metric,report=copied_seed_metric(cap,faces,centres,volumes,rows=rows,
        shape_progress=1.,shape_power=1.,size_growth=1.)
    return metric,dict(report,role='NumericalConditionerOnly',size_target_guaranteed=False)



def attach(seed, controls, centres, volumes, check=None):
    """Attach immutable metric targets and exact envelope witnesses to our seed."""
    validate(controls)
    if not physical_shape(controls):return _metric_attach(seed,controls,centres,volumes,check)
    cap=seed['original'][seed['cap_ids']];faces=seed['cap_faces'];n=len(cap)
    compact=seed['compact_ids'];p=seed['points'][compact]
    if len(p)!=(controls['rows']+1)*n:raise ValueError('CustomCopiedSeedPopulationMismatch')
    metric,report=conditioner(cap,faces,np.asarray(centres),np.asarray(volumes),controls['rows'])
    # Each copied vertex already has an exact source-cap vertex anchor. Select
    # a positively oriented incident face; no nearest search or projection moves.
    tri=cap[faces];normal=np.cross(tri[:,1]-tri[:,0],tri[:,2]-tri[:,0])
    tids=[];weights=[]
    for level in range(1,controls['rows']+1):
        if check:check()
        delta=p[level*n:(level+1)*n]-cap
        scores=np.einsum('fvi,fi->fv',delta[faces],normal)
        best=np.full(n,-np.inf);chosen=np.full(n,-1,dtype='<i8');uv=np.zeros((n,2))
        for role in range(3):
            # Stable face identity wins ties; vectors choose only proposals.
            order=np.argsort(scores[:,role],kind='stable')
            for j in order:
                v=faces[j,role]
                if scores[j,role]>best[v]:
                    best[v]=scores[j,role];chosen[v]=j;uv[v]=((0.,0.),(1.,0.),(0.,1.))[role]
        if np.any(best<=0) or np.any(chosen<0):raise ValueError('CustomSeedOutsideOrientedEnvelope')
        tids.append(chosen);weights.append(uv)
    envelope=dict(vertex_ids=np.arange(n,len(p),dtype='<i8'),triangle_ids=np.concatenate(tids),weights=np.concatenate(weights))
    corrections=[]
    try:
        verified=verify(p,cap,faces,controls['maximum_distance_m'],envelope,check)
    except ValueError as error:
        if str(error)!='VertexOutsideDeclaredEnvelope':raise
        # Reuse the existing exact predicate and contraction proposer. Input
        # cap and witness identities stay fixed. No tolerance relaxation or
        # nearest-face reassignment is permitted. This is still private geometry.
        from shape_transition_domain import _inputs,_predicate,_contract
        _,_,_,d2,geometries=_inputs(p,cap,faces,controls['maximum_distance_m'],check)
        for vertex,tid,uv in zip(envelope['vertex_ids'],envelope['triangle_ids'],envelope['weights']):
            if check:check()
            geometry=geometries[tid]
            if _predicate(p[vertex],geometry,uv,d2):continue
            before=p[vertex].copy();corrected=_contract(before,geometry,uv,d2,check)
            if corrected is None:raise ValueError('UnrepresentableFixedAnchorEnvelope')
            corrections.append(dict(vertex=int(vertex),before=before.tolist(),after=corrected.tolist(),
                displacement_m=float(np.linalg.norm(corrected-before))))
            p[vertex]=corrected
        verified=verify(p,cap,faces,controls['maximum_distance_m'],envelope,check)
        seed['points'][compact]=p
    verified=dict(verified,construction_roundoff_corrections=corrections,
        neighbours_and_full_admission_must_use_corrected_geometry=True)
    result=dict(seed,controls=dict(controls),vertex_envelope={k:frozen(v,v.dtype.str) for k,v in envelope.items()},
        construction=dict(seed['construction'],method='OwnedSharedRows/ConstrainedMetricPlacement/1',external_remesher=False,
            metric=report,vertex_envelope=verified,proposal_only=True),
        custom_metric=frozen(metric,'<f8'))
    return result


def initialize(points,oldcells,oldowners,ids,faces,terminal,controls,bounds=None,*,prism_volumes=None,prism_centres=None,check=None,output=None,resources=None):
    from tet_layer_adapter import initialize as seed_initialize
    from background_mixed_geometry import batch_geometry
    started=time.perf_counter();validate(controls)
    if not physical_shape(controls):
        return _metric_initialize(points,oldcells,oldowners,ids,faces,terminal,controls,bounds,
            prism_volumes=prism_volumes,prism_centres=prism_centres,check=check,output=output,resources=resources)
    if prism_centres is None or prism_volumes is None:
        byid={c['id']:c for c in oldcells};g=batch_geometry(points,[byid[i] for i in dict.fromkeys(terminal)])
        prism_centres=np.asarray([g[i]['centre'] for i in terminal]);prism_volumes=np.asarray([g[i]['volume'] for i in terminal])
    from tet_layer_planning import complete_rows
    from sdf_cfd_mesher_v4.common.authority import frozen
    metric,metric_report=conditioner(points[ids],faces,prism_centres,prism_volumes,controls['rows'])
    row_radii=frozen(np.asarray([(j+1)/controls['rows']*controls['maximum_distance_m'] for j in range(controls['rows'])]),'<f8')
    def finish(fronts,records,check):
        from tet_layer_adapter import place
        from tet_layer_row import RowConflict
        count=len(fronts)-1;u=count/controls['rows']
        partial=dict(controls,rows=count,maximum_distance_m=u*controls['maximum_distance_m'])
        direction=fronts[1]-fronts[0];direction/=np.linalg.norm(direction,axis=1)[:,None]
        private=seed_initialize(points,oldcells,oldowners,ids,faces,terminal,seed_controls(partial),bounds,
            prism_volumes=prism_volumes,prism_centres=prism_centres,check=check,
            _fronts=frozen(fronts,'<f8'),_direction=frozen(direction,'<f8'),
            _construction=dict(method='PrivateCurrentRowCompletion',rows=records,proposal_only=True))
        private=attach(private,partial,prism_centres,prism_volumes,check)
        private['custom_metric']=frozen(metric[:(count+1)*len(ids)],'<f8')
        from sdf_cfd_mesher_v4.common.authority import array_hash
        private['construction']['metric']=dict(metric_report,
            population='Prefix of original complete numerical conditioner',
            prefix_rows=count,complete_rows=controls['rows'],
            actual_tensor_sha256=array_hash(private['custom_metric']))
        private['_construction_quality_only']=True
        private['_physical_shape_completion']=False
        private['_construction_row_radii']=row_radii[:count]
        placed=place(private,check)
        retain=getattr(check,'retain_partial_row',None)
        if retain is not None:retain(private,placed)
        if not placed['passed']:
            raise RowConflict('ConstructedRowQualityConflict',row=count-1,placement=placed)
        complete=private['points'][private['compact_ids']].reshape(count+1,len(ids),3)
        return complete,dict(quality=placed['quality'],initial_records=placed['initial_records'],
            seconds=placed['seconds'],coordinate_trials=placed['custom_vertex_trials'],
            accepted_coordinate_proposals=sum(r['adopted'] for r in placed['groups'])+sum(r['adopted'] for r in placed.get('coupled_groups',[])),
            scientific_admission_still_required=True)
    fronts,construction=complete_rows(points[ids],faces,prism_centres,prism_volumes,
        seed_controls(controls),bounds,check,metric=metric,row_finish=finish,objective='physical_shape')
    direction=fronts[1]-fronts[0];direction/=np.linalg.norm(direction,axis=1)[:,None]
    construction['construction_metric']=metric_report
    seed=seed_initialize(points,oldcells,oldowners,ids,faces,terminal,seed_controls(controls),bounds,
        prism_volumes=prism_volumes,prism_centres=prism_centres,check=check,
        _fronts=frozen(fronts,'<f8'),_direction=frozen(direction,'<f8'),_construction=construction)
    result=attach(seed,controls,prism_centres,prism_volumes,check)
    result['construction']['method']='OwnedSharedRows/PhysicalShapeConstruction/1'
    result['_construction_quality_only']=True
    result['_physical_shape_completion']=True
    result['_construction_row_radii']=row_radii
    result['construction']['seconds']=time.perf_counter()-started
    if output is not None:
        from standalone_tool import check_resources
        from path_preserving_deduplication import durable_json
        folder=Path(output);folder.mkdir(parents=True,exist_ok=False)
        if resources is not None:check_resources(folder.parent.parent,resources)
        durable_json(folder/'CONSTRUCTION.json',result['construction'])
    return result


def _metric_attach(seed, controls, centres, volumes, check=None):
    """Attach immutable metric targets and exact envelope witnesses to our seed."""
    validate(controls)
    cap=seed['original'][seed['cap_ids']];faces=seed['cap_faces'];n=len(cap)
    compact=seed['compact_ids'];p=seed['points'][compact]
    if len(p)!=(controls['rows']+1)*n:raise ValueError('CustomCopiedSeedPopulationMismatch')
    metric,report=copied_seed_metric(cap,faces,np.asarray(centres),np.asarray(volumes),rows=controls['rows'],
        shape_progress=controls['shape_progress'],shape_power=controls['shape_power'],size_growth=controls['size_growth'])
    # Each copied vertex already has an exact source-cap vertex anchor. Select
    # a positively oriented incident face; no nearest search or projection moves.
    tri=cap[faces];normal=np.cross(tri[:,1]-tri[:,0],tri[:,2]-tri[:,0])
    tids=[];weights=[]
    for level in range(1,controls['rows']+1):
        if check:check()
        delta=p[level*n:(level+1)*n]-cap
        scores=np.einsum('fvi,fi->fv',delta[faces],normal)
        best=np.full(n,-np.inf);chosen=np.full(n,-1,dtype='<i8');uv=np.zeros((n,2))
        for role in range(3):
            # Stable face identity wins ties; vectors choose only proposals.
            order=np.argsort(scores[:,role],kind='stable')
            for j in order:
                v=faces[j,role]
                if scores[j,role]>best[v]:
                    best[v]=scores[j,role];chosen[v]=j;uv[v]=((0.,0.),(1.,0.),(0.,1.))[role]
        if np.any(best<=0) or np.any(chosen<0):raise ValueError('CustomSeedOutsideOrientedEnvelope')
        tids.append(chosen);weights.append(uv)
    envelope=dict(vertex_ids=np.arange(n,len(p),dtype='<i8'),triangle_ids=np.concatenate(tids),weights=np.concatenate(weights))
    verified=verify(p,cap,faces,controls['maximum_distance_m'],envelope,check)
    result=dict(seed,controls=dict(controls),vertex_envelope={k:frozen(v,v.dtype.str) for k,v in envelope.items()},
        construction=dict(seed['construction'],method='OwnedSharedRows/ConstrainedMetricPlacement/1',external_remesher=False,
            metric=report,vertex_envelope=verified,proposal_only=True),
        custom_metric=frozen(metric,'<f8'))
    return result


def _metric_initialize(points,oldcells,oldowners,ids,faces,terminal,controls,bounds=None,*,prism_volumes=None,prism_centres=None,check=None,output=None,resources=None):
    from tet_layer_adapter import initialize as seed_initialize
    from background_mixed_geometry import batch_geometry
    started=time.perf_counter();validate(controls)
    if prism_centres is None or prism_volumes is None:
        byid={c['id']:c for c in oldcells};g=batch_geometry(points,[byid[i] for i in dict.fromkeys(terminal)])
        prism_centres=np.asarray([g[i]['centre'] for i in terminal]);prism_volumes=np.asarray([g[i]['volume'] for i in terminal])
    seed=seed_initialize(points,oldcells,oldowners,ids,faces,terminal,seed_controls(controls),bounds,
        prism_volumes=prism_volumes,prism_centres=prism_centres,check=check)
    result=_metric_attach(seed,controls,prism_centres,prism_volumes,check)
    result['construction']['seconds']=time.perf_counter()-started
    if output is not None:
        from standalone_tool import check_resources
        from path_preserving_deduplication import durable_json
        folder=Path(output);folder.mkdir(parents=True,exist_ok=False)
        if resources is not None:check_resources(folder.parent.parent,resources)
        durable_json(folder/'CONSTRUCTION.json',result['construction'])
    return result

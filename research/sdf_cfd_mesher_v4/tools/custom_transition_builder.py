"""Owned shared-row transition construction; no external remesher dispatch.

Actual source/prism owners, conformal product topology and final acceptance are
reused. Metric objectives propose shape and size; they are not acceptance gates.
"""
from pathlib import Path
import time
import numpy as np
from tet_layer_controls import validate,seed_controls
from shape_transition_metric import copied_seed_metric
from shape_transition_domain import verify
from sdf_cfd_mesher_v4.common.authority import frozen


def attach(seed, controls, centres, volumes, check=None):
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


def initialize(points,oldcells,oldowners,ids,faces,terminal,controls,bounds=None,*,prism_volumes=None,prism_centres=None,check=None,output=None,resources=None):
    from tet_layer_adapter import initialize as seed_initialize
    from background_mixed_geometry import batch_geometry
    started=time.perf_counter();validate(controls)
    if prism_centres is None or prism_volumes is None:
        byid={c['id']:c for c in oldcells};g=batch_geometry(points,[byid[i] for i in dict.fromkeys(terminal)])
        prism_centres=np.asarray([g[i]['centre'] for i in terminal]);prism_volumes=np.asarray([g[i]['volume'] for i in terminal])
    seed=seed_initialize(points,oldcells,oldowners,ids,faces,terminal,seed_controls(controls),bounds,
        prism_volumes=prism_volumes,prism_centres=prism_centres,check=check)
    result=attach(seed,controls,prism_centres,prism_volumes,check)
    result['construction']['seconds']=time.perf_counter()-started
    if output is not None:
        from standalone_tool import check_resources
        from path_preserving_deduplication import durable_json
        folder=Path(output);folder.mkdir(parents=True,exist_ok=False)
        if resources is not None:check_resources(folder.parent.parent,resources)
        durable_json(folder/'CONSTRUCTION.json',result['construction'])
    return result

"""Decisive existing current gates before any future forecast or conditioning.

Passing this screen is NOT acceptance. Complete numeric gates, exact rational
signs and the qualified continuous certificate remain mandatory downstream.
The first screen preserves the existing analytical wedge/determinant expressions.
"""
import numpy as np
from ..runtime.frozen_imports import legacy_modules

def immediate_cell_screen(top,low,nominal,active,proposed,patch,layer):
    ids=top.triangles[patch.core_columns];high=nominal[ids].copy()
    mask=np.isin(ids,active);high[mask]=proposed[np.searchsorted(active,ids[mask])]
    p=np.concatenate((low[ids],high),axis=1);cq=legacy_modules()['component_quality']
    jac,volume=cq.PM._exact_wedge_jacobian_and_volume(p);det=cq.determinant(p,layer)
    values=dict(jacobian=jac,signed_volume=volume,determinant=det)
    limits=dict(jacobian=0.,signed_volume=0.,determinant=.0011)
    violations={k:int(np.count_nonzero((v<limits[k] if k=='determinant' else v<=limits[k])|~np.isfinite(v))) for k,v in values.items()}
    return dict(rejected=any(violations.values()),stage='ImmediateCellScreen',cells=len(ids),
        gate=dict(passed=not any(violations.values()),violations=violations,
            extrema={k:dict(min=float(np.nanmin(v)),max=float(np.nanmax(v))) for k,v in values.items()},
            evaluated_gates=list(values),not_evaluated='Remaining immediate metrics, conditioning, exact signs and certificate'))

def staged_detail(screen,callback):
    """The production early-out contract; debug diagnostics require an explicit call."""
    if screen['rejected']:return None
    return callback()

def immediate_numeric_screen(front,top,coords,active,proposed,patch,layer):
    """Complete current numeric gates, without provisional future or sigma/G.

    Previous cells contribute only their required centre/volume primitives.
    The new interface is evaluated when its upper immediate cell first exists.
    """
    from ..forecast.detailed.metrics import cell_metrics
    from .local_relaxation import numeric_gates
    from ..forecast.detailed.patch import require_closure
    require_closure(patch,top)
    ids=top.triangles[patch.columns];low=front.coordinates if layer==1 else coords[layer-2]
    high=coords[layer-1,patch.vertices].copy();high[np.searchsorted(patch.vertices,active)]=proposed
    local=np.searchsorted(patch.vertices,ids);p=np.concatenate((low[ids],high[local]),axis=1)
    g=cell_metrics(p,np.full(len(p),layer),conditioning=False);core=np.searchsorted(patch.columns,patch.core_columns)
    cq=legacy_modules()['component_quality'];fg=cq.FG;ed=top.edges[patch.side_edges];el=np.searchsorted(patch.vertices,ed)
    owners=np.asarray([top.edge_triangles.row(e) for e in patch.side_edges]);ix=np.searchsorted(patch.columns,owners)
    vs=np.stack((low[ed[:,0]],low[ed[:,1]],high[el[:,1]],high[el[:,0]]),axis=1);S,C=fg.face_area_centre_quad(vs)
    vals=cq.face_metrics(vs,S,C,g['centre'][ix[:,0]],g['centre'][ix[:,1]],g['volume'][ix[:,0]],g['volume'][ix[:,1]])
    faces=[('side',layer,int(e),int(a),int(b)) for e,(a,b) in zip(patch.side_edges,owners)]
    if layer>1:
        prior=front.coordinates if layer==2 else coords[layer-3]
        previous=np.concatenate((prior[ids[core]],low[ids[core]]),axis=1);Co,Vo=fg.prism_cell_centres(previous)
        vs=low[ids[core]];S,C=fg.face_area_centre_tri(vs)
        cap=cq.face_metrics(vs,S,C,Co,g['centre'][core],Vo,g['volume'][core])
        vals=[np.concatenate((a,b)) for a,b in zip(vals,cap)];faces.extend(('cap',layer-1,int(c),int(c),int(c)) for c in patch.core_columns)
    gate=numeric_gates(dict(cells={k:v[core][None] for k,v in g.items()},face_ids=faces,
        faces=dict(zip(('weight','skewness','symmetric_volume_ratio','nonorthogonality'),vals))))
    gate.update(layer=layer,scope='All verification cells, new internal side faces, and the new interface with the previous private layer')
    return dict(rejected=not gate['passed'],stage='ImmediateNumericScreen',gate=gate,conditioning_evaluated=False)

def exact_current(top,low,nominal,active,proposed,patch,layer,cache=None):
    from ..forecast.detailed.exact_cache import LOCAL_EXACT
    cache=LOCAL_EXACT if cache is None else cache
    ids=top.triangles[patch.core_columns];high=nominal[ids].copy();mask=np.isin(ids,active)
    high[mask]=proposed[np.searchsorted(active,ids[mask])]
    records=[dict(column=int(c),layer=layer,**cache.evaluate(np.concatenate((a,b)))) for c,a,b in zip(patch.core_columns,low[ids],high)]
    return dict(passed=all(r['jacobian_sign']>0 and r['volume_sign']>0 for r in records),records=records)

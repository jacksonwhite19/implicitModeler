"""Restore the exact Task-1 scientific cache without projecting any trajectory."""
import json,time
import numpy as np
from ...common.authority import ROOT,file_hash
from ..engine import ForecastCache,ForecastResult,cache_key
from ..descriptors import unit,angle,edge_descriptors,triangle_descriptors

def restore(front,topology,schedule,sdf):
    started=time.perf_counter();a=json.loads((ROOT/'authorities/V4_TASK1_AUTHORITY.json').read_text())
    stored={}
    for first in range(1,21,5):
        name=f'results/task1/forecast_layers_{first:02}_{first+4:02}.npz'
        if file_hash(ROOT/name)!=a['forecast_archives'][name]['sha256']:raise ValueError('Task1ArchiveDrift')
        with np.load(ROOT/name) as z:
            for name in z.files:stored.setdefault(name,[]).append(z[name])
    stored={k:np.concatenate(v) for k,v in stored.items()}
    base=front.coordinates;previous=base;wall=sdf.evaluate(base);previous_phi=wall.phi;previous_direction=wall.normal
    cumulative=np.zeros(len(base));vertex={};edge={};triangle={}
    for k,x in enumerate(stored['coordinates']):
        q=sdf.evaluate(x)
        for name,value in [('phi',q.phi),('normals',q.normal),('gradient_norm',q.gradient_norm)]:
            if not np.array_equal(value,stored[name][k]):raise ValueError('Task1NativeReplayDrift:'+name)
        displacement=x-previous;direction,length=unit(displacement)
        target=np.full(len(x),schedule.targets[k]);residual=q.phi-target
        data=dict(coordinates=x,normal=q.normal,initial_phi=previous_phi,target=target,phi=q.phi,residual=residual,
                  gradient_norm=q.gradient_norm,source_error_bound=q.source_error_bound,iterations=stored['iterations'][k],
                  displacement=displacement,step_length=length,nominal_step=target-(front.target_distances if k==0 else schedule.targets[k-1]),
                  direction_change=angle(direction,previous_direction),cumulative_displacement=np.linalg.norm(x-base,axis=1),
                  path_length=cumulative+length,section_curvature=q.section_curvature,boundary_normal_selection=q.boundary_normal_selection)
        ed=edge_descriptors(x,previous,base,q.normal,topology.edges)
        tr=triangle_descriptors(x,previous,base,topology.triangles)
        for name,value in [('residual',residual),('direction_change',data['direction_change']),('edge_ratio',ed['ratio']),('neighbor_angle',ed['normal_angle']),('triangle_ratio',tr['ratio'])]:
            if not np.array_equal(value,stored[name][k]):raise ValueError('Task1DescriptorReplayDrift:'+name)
        for group,values in ((vertex,data),(edge,ed),(triangle,tr)):
            for name,value in values.items():group.setdefault(name,[]).append(value)
        cumulative+=length;previous=x;previous_phi=q.phi;previous_direction=direction
    groups=[{k:np.stack(v) for k,v in g.items()} for g in (vertex,edge,triangle)]
    result=ForecastResult(cache_key(front,schedule,sdf),np.arange(1,21,dtype=np.int64),*groups,{},dict(vertices=np.asarray(front.vertex_ids,dtype='U'),edges=topology.edges,triangles=topology.triangles))
    actual=result.scientific_hash()
    if actual!=a['full_forecast_hash_run1']:raise ValueError('Task1ScientificIdentityMismatch:'+actual)
    cache=ForecastCache();cache.result=result;cache.valid=[np.ones(n,bool) for n in (len(base),len(topology.edges),len(topology.triangles))]
    return cache,dict(scientific_hash=actual,seconds=time.perf_counter()-started,projected_trajectories=0,restored_archives=4)

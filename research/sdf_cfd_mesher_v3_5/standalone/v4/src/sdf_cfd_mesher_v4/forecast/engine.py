"""Independent SDF trajectories and direct front descriptors, without prisms."""
from dataclasses import dataclass,asdict
import hashlib,time
from pathlib import Path
import numpy as np
from ..common.authority import identity,array_hash,file_hash
from ..advancing_front.predict import predict_next_front
from .descriptors import angle,unit,edge_descriptors,triangle_descriptors
from ..runtime.regions import dependencies

@dataclass(frozen=True)
class ForecastCacheKey:
    sdf_source: str
    geometry: str
    surface: str
    topology: str
    committed_layer: int
    committed_coordinates: str
    remaining_schedule: str
    implementation: str
    @property
    def sha256(self):return identity(asdict(self))

def implementation_identity(iteration_cap):
    package=Path(__file__).resolve().parents[1]
    names=('forecast/engine.py','forecast/descriptors.py','forecast/schedule.py','advancing_front/predict.py','geometry/front.py')
    return identity(dict(version='V4CheapForecast/1',sources={n:file_hash(package/n) for n in names},iteration_cap=iteration_cap,projection_stop='native_source_distance_error_bound'))

def forecast_limit(front,schedule,last_layer):
    end=len(schedule.targets)
    if last_layer is None:return end
    if type(last_layer) is not int or not front.committed_layer<last_layer<=end:
        raise ValueError('InvalidForecastHorizon')
    return last_layer

def cache_key(front,schedule,sdf,iteration_cap=64,*,last_layer=None):
    if getattr(sdf,'semantics',None)=='AcceptedFrontState/path_length_m/1':sdf.check()
    end=forecast_limit(front,schedule,last_layer)
    remaining=array_hash(schedule.remaining(front.committed_layer))
    # Bind the full schedule even for one-step screening. Do not confuse a
    # prefix cache with full lookahead or hide a changed future request.
    if last_layer is not None:remaining=identity(dict(full_schedule=remaining,last_layer=end,scope='ExplicitForecastHorizon/1'))
    return ForecastCacheKey(sdf.source_identity,front.geometry_authority,front.surface_authority,front.topology_hash,front.committed_layer,front.coordinate_hash,remaining,implementation_identity(iteration_cap))

@dataclass
class ForecastResult:
    key: ForecastCacheKey
    layers: np.ndarray
    vertex: dict
    edge: dict
    triangle: dict
    timings: dict
    fine_ids: dict
    def scientific_hash(self):
        h=hashlib.sha256(self.key.sha256.encode())
        for group,data in (('vertex',self.vertex),('edge',self.edge),('triangle',self.triangle),('ids',self.fine_ids)):
            for name,a in sorted(data.items()):
                a=np.ascontiguousarray(a);h.update((group+'/'+name+':'+a.dtype.str+':'+str(a.shape)).encode());h.update(a.tobytes())
        h.update(self.layers.tobytes());return h.hexdigest()
    def hard_events(self):
        result=dict(nonfinite_coordinate=int((~np.isfinite(self.vertex['coordinates'])).any(2).sum()),sdf_evaluation_failure=0,zero_required_gradient=int((self.vertex['gradient_norm']==0).sum()) if 'gradient_norm' in self.vertex else 0,projection_failure=0,zero_area_triangle=int((~self.triangle['nondegenerate']).sum()),orientation_reversal=int(self.triangle['orientation_reversal'].sum()),invalid_topology_reference=0)
        if 'declared_target_valid' in self.vertex:
            result['invalid_declared_target']=int((~self.vertex['declared_target_valid']).sum())
        return result

class _DeclaredHorizon:
    """Read-only prefix of existing owned positions, not another predictor.

    The existing declared-trajectory operation iterates to len(positions).
    Keep its original schedule object, source checks and arithmetic intact.
    """
    def __init__(self,owner,last_layer):self._owner=owner;self._last=last_layer
    def __getattr__(self,name):return getattr(self._owner,name)
    def positions(self):return self._owner.positions()[:self._last+1]

def vertex_trajectories(front,schedule,sdf,indices,iteration_cap=64,*,last_layer=None):
    end=forecast_limit(front,schedule,last_layer)
    # An explicit owner operation, not scalar fallback or fabricated phi/gradient.
    if getattr(sdf,'semantics',None)=='AcceptedFrontState/path_length_m/1':
        if last_layer is not None:
            # Only the established AcceptedFrontState operation has this prefix
            # contract. An arbitrary declared provider must not be approximated.
            from accepted_trajectory_contract import AcceptedTrajectoryOwner
            operation=getattr(type(sdf),'forecast_vertices',None)
            if operation is not AcceptedTrajectoryOwner.forecast_vertices:
                raise ValueError('UnsupportedBoundedTrajectoryOperation')
            return operation(_DeclaredHorizon(sdf,end),front,schedule,indices)
        return sdf.forecast_vertices(front,schedule,indices)
    base=front.coordinates[indices];x=base.copy();previous_direction=None;records={};seconds=0.
    cumulative_length=np.zeros(len(indices));targets=schedule.remaining(front.committed_layer)[:end-front.committed_layer]
    for k,target in enumerate(targets):
        p=predict_next_front(x,target,sdf,iteration_cap=iteration_cap);seconds+=p.projection_seconds
        direction,length=unit(p.displacement)
        if previous_direction is None:previous_direction=sdf.evaluate(base).normal
        current=dict(coordinates=p.coordinates,normal=p.normals,initial_phi=p.initial_phi,target=p.target,phi=p.final_phi,residual=p.residual,gradient_norm=p.gradient_norm,source_error_bound=p.error_bound,iterations=p.iterations,displacement=p.displacement,step_length=length,nominal_step=np.full(len(indices),target)-(front.target_distances[indices] if k==0 else targets[k-1]),direction_change=angle(direction,previous_direction),cumulative_displacement=np.linalg.norm(p.coordinates-base,axis=1),path_length=cumulative_length+length,section_curvature=p.curvature,boundary_normal_selection=p.boundary_selections)
        for name,value in current.items():records.setdefault(name,[]).append(value)
        cumulative_length+=length;previous_direction=direction;x=p.coordinates
    return {name:np.stack(v) for name,v in records.items()},seconds

class ForecastCache:
    """Small in-memory cache of entity arrays, with explicit validity masks.

    Miss/hit counts count vertex trajectories and edge/triangle descriptor
    trajectories. Native exact-query memoization is separate and can be cleared.
    This cache accepts only a matching whole authority key. A changed whole key
    never reuses stale data. Local invalidation of unchanged authority recomputes
    only changed vertices and incident descriptor entities.
    """
    def __init__(self):self.result=None;self.valid=None;self.last_stats=None
    def invalidate(self,topology,vertex_ids,*,rings=0):
        if self.result is None:raise ValueError('NoCachedForecast')
        if topology.sha256!=self.result.key.topology:raise ValueError('TopologyAuthorityMismatch')
        v,e,t=dependencies(topology,vertex_ids,rings)
        for mask,idx in zip(self.valid,(v,e,t)):mask[idx]=False
        return dict(vertices=v,edges=e,triangles=t)
    def forecast(self,front,topology,schedule,sdf,*,iteration_cap=64,last_layer=None):
        started=time.perf_counter();end=forecast_limit(front,schedule,last_layer);key=cache_key(front,schedule,sdf,iteration_cap,last_layer=last_layer);counts=(len(front.coordinates),len(topology.edges),len(topology.triangles));layers=np.arange(front.committed_layer+1,end+1,dtype=np.int64)
        if len(layers)==0:raise ValueError('NoRemainingLayers')
        if topology.sha256!=front.topology_hash:raise ValueError('TopologyAuthorityMismatch')
        cold=self.result is None or self.result.key!=key
        if cold:self.result=None;self.valid=[np.zeros(n,dtype=bool) for n in counts]
        hits=sum(int(v.sum()) for v in self.valid);idx=[np.flatnonzero(~v) for v in self.valid];misses=sum(map(len,idx))
        if misses==0:
            self.last_stats=dict(hits=hits,misses=0,seconds=time.perf_counter()-started,sdf_seconds=0.,projection_seconds=0.,descriptor_seconds=0.);return self.result
        sdf_before=sdf.seconds;projection=0.;descriptor=0.
        if len(idx[0]):vdata,projection=vertex_trajectories(front,schedule,sdf,idx[0],iteration_cap,last_layer=last_layer)
        else:vdata={}
        if cold:
            fine=dict(vertices=np.asarray(front.vertex_ids,dtype='U'),edges=topology.edges,triangles=topology.triangles)
            result=ForecastResult(key,layers,vdata,{}, {},{},fine)
        else:
            result=self.result
            for name,value in vdata.items():result.vertex[name][:,idx[0]]=value
        t=time.perf_counter();edata={};tdata={};previous=front.coordinates
        for k,x in enumerate(result.vertex['coordinates']):
            if len(idx[1]):
                declared='trajectory_direction' in result.vertex
                direction=result.vertex['trajectory_direction' if declared else 'normal'][k]
                e=edge_descriptors(x,previous,front.coordinates,direction,topology.edges[idx[1]])
                if declared:
                    # Angle/convergence concern declared directions, not branch normals.
                    e={'trajectory_'+name if 'normal' in name else name:value for name,value in e.items()}
                for name,value in e.items():edata.setdefault(name,[]).append(value)
            if len(idx[2]):
                tri=triangle_descriptors(x,previous,front.coordinates,topology.triangles[idx[2]])
                for name,value in tri.items():tdata.setdefault(name,[]).append(value)
            previous=x
        for dest,source,indices in ((result.edge,edata,idx[1]),(result.triangle,tdata,idx[2])):
            for name,value in source.items():
                value=np.stack(value)
                if cold:dest[name]=value
                else:dest[name][:,indices]=value
        descriptor=time.perf_counter()-t
        for valid in self.valid:valid[:]=True
        self.result=result
        stats=dict(hits=hits,misses=misses,seconds=time.perf_counter()-started,sdf_seconds=sdf.seconds-sdf_before,projection_seconds=projection,descriptor_seconds=descriptor)
        self.last_stats=stats;result.timings=stats.copy();return result

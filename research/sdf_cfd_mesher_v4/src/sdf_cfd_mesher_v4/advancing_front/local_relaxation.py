"""Bounded proposed-front experiments. No committed state or layer transaction.

Only proposed front 1 is an independent variable. All later positions call the
unchanged Task-1 trajectory engine. There is no global sensitivity allocation.
"""
from dataclasses import dataclass
from types import SimpleNamespace
import time
import numpy as np
from ..common.authority import frozen,array_hash,identity
from ..runtime.regions import dependencies,expand_vertices
from ..runtime.frozen_imports import legacy_modules
from ..forecast.engine import vertex_trajectories
from ..forecast.descriptors import edge_descriptors,triangle_descriptors
from ..forecast.detailed.patch import Patch,require_closure,LocalityFailure
from ..forecast.detailed.cache import DetailedCache
from ..forecast.forensics import exact_wedge
from .predict import predict_next_front


@dataclass(frozen=True)
class RelaxationPatch:
    active: np.ndarray
    fixed_boundary: np.ndarray
    halo: np.ndarray
    affected_columns: np.ndarray
    verification_ring: np.ndarray
    detailed: Patch

    def record(self):
        return {k:getattr(self,k).tolist() for k in ('active','fixed_boundary','halo','affected_columns','verification_ring')}|dict(detailed=self.detailed.record())


def build_patch(top,source_columns,*,active_rings=0,max_vertices=96,max_columns=256):
    active=expand_vertices(top,np.unique(top.triangles[np.asarray(source_columns,dtype=int)]),active_rings)
    if len(active)>max_vertices:raise LocalityFailure('ActiveVertexBound')
    _,_,affected=dependencies(top,active)
    verify=set(map(int,affected))
    for e in np.unique(top.triangle_edges[affected]):verify.update(map(int,top.edge_triangles.row(e)))
    core=np.array(sorted(verify));sides=np.unique(top.triangle_edges[core]);cols=set(verify)
    for e in sides:cols.update(map(int,top.edge_triangles.row(e)))
    if len(cols)>max_columns or len(cols)==len(top.triangles):raise LocalityFailure('VerificationClosureBound')
    cols=np.array(sorted(cols));cv=np.unique(top.triangles[core]);pv=np.unique(top.triangles[cols])
    p=Patch(*(frozen(a,'<i8') for a in (core,cols,pv,cv,sides)),top.sha256);require_closure(p,top)
    boundary=np.setdiff1d(cv,active);halo=np.setdiff1d(pv,cv);ring=np.setdiff1d(core,affected)
    return RelaxationPatch(frozen(active,'<i8'),boundary,halo,affected,ring,p)


def tangent_basis(normals):
    n=np.asarray(normals,float);length=np.linalg.norm(n,axis=1)
    if not np.isfinite(n).all() or np.any(length==0):raise ValueError('InvalidRequiredNormal')
    n=n/length[:,None];axes=np.eye(3)[np.argmin(abs(n),axis=1)]
    u=np.cross(n,axes);u/=np.linalg.norm(u,axis=1)[:,None]
    return np.stack((u,np.cross(n,u)),axis=2)


def project_tangent_move(current,field,distances,sdf,target):
    q=sdf.evaluate(current);basis=tangent_basis(q.normal)
    tangent=np.einsum('nij,nj->ni',basis,np.einsum('nji,nj->ni',basis,field))
    length=np.linalg.norm(tangent,axis=1);unit=np.divide(tangent,length[:,None],out=np.zeros_like(tangent),where=length[:,None]!=0)
    delta=unit*np.asarray(distances)[:,None];pre=current+delta
    p=predict_next_front(pre,target,sdf)
    return p.coordinates,dict(pre_projection=pre,tangential_displacement=delta,tangent_basis=basis,projected=p.coordinates,
                             target=p.target,residual=p.residual,projection_iterations=p.iterations,projection_failed=False,
                             source_error_bound=p.error_bound,gradient_norm=p.gradient_norm)


def search_fields(front,top,patch,x,total_thickness):
    """Geometric search directions, not new normals or geometry authorities."""
    f=top.triangles;wall=front.coordinates
    local_columns=patch.affected_columns;local_faces=f[local_columns]
    area=np.cross(wall[local_faces[:,1]]-wall[local_faces[:,0]],wall[local_faces[:,2]]-wall[local_faces[:,0]])
    avg=[];lap=[];radii=[]
    for v in patch.active:
        cols=top.vertex_triangles.row(v);neighbors=top.vertex_neighbors.row(v)
        avg.append(area[np.searchsorted(local_columns,cols)].sum(0));lap.append(x[neighbors].mean(0)-x[v])
        # Frozen V3 planner local geometric radius, expressed in metres.
        d=np.linalg.norm(wall[neighbors]-wall[v],axis=1)
        radii.append(min(.25*float(total_thickness),.5*float(d[d>0].min())))
    avg=np.asarray(avg);avg/=np.linalg.norm(avg,axis=1)[:,None]
    lap=np.asarray(lap);ln=np.linalg.norm(lap,axis=1);lap=np.divide(lap,ln[:,None],out=np.zeros_like(lap),where=ln[:,None]!=0)
    return [('wall_area_normal',avg),('tangent_laplacian',lap),('opposite_laplacian',-lap),
            ('wall_plus_laplacian',avg+lap),('wall_minus_laplacian',avg-lap)],np.asarray(radii)


def adjust_trust(radius,accepted,*,prediction_ratio=None,boundary=True):
    # Reuse the exact frozen two-way radius rule; no old sigma floors/objectives.
    return legacy_modules()['trust_region'].next_radius(radius,accepted,dict(ratio=prediction_ratio,trust_boundary_active=boundary))


def choose_trial(incumbent,trial,accepted):
    """The search transaction changes only a private proposal on acceptance."""
    return np.asarray(trial if accepted else incumbent).copy()


def control_needs_move(candidate_gate,future_failures):
    return not candidate_gate['passed'] or bool(future_failures)


class LayerView:
    def __init__(self,base,ids,values):self.base=base;self.ids=ids;self.values=values
    def __getitem__(self,selection):
        out=self.base[selection].copy();indices=np.arange(len(self.base))[selection] if isinstance(selection,slice) else np.asarray(selection)
        positions=np.searchsorted(self.ids,indices);valid=positions<len(self.ids)
        clipped=np.minimum(positions,max(0,len(self.ids)-1));valid=valid&(self.ids[clipped]==indices) if len(self.ids) else np.zeros_like(valid)
        if indices.ndim==0:return self.values[int(positions)] if valid else out
        out[valid]=self.values[positions[valid]];return out


class CoordinateOverlay:
    """Immutable original arrays plus only changed vertex trajectories."""
    def __init__(self,base,ids,values):self.base=base;self.ids=np.asarray(ids,dtype=int);self.values=values
    def __len__(self):return len(self.base)
    def __iter__(self):
        for k in range(len(self)):yield LayerView(self.base[k],self.ids,self.values[k])
    def __getitem__(self,selection):
        if not isinstance(selection,tuple):
            if np.isscalar(selection):return LayerView(self.base[selection],self.ids,self.values[selection])
            return CoordinateOverlay(self.base[selection],self.ids,self.values[selection])
        layers,vertices=selection
        if np.isscalar(layers):return LayerView(self.base[layers],self.ids,self.values[layers])[vertices]
        return np.stack([LayerView(self.base[k],self.ids,self.values[k])[vertices] for k in np.arange(len(self))[layers]])


class LocalForecastCache:
    def __init__(self,front,top,schedule,sdf,coordinates,normals):
        self.front=front;self.top=top;self.schedule=schedule;self.sdf=sdf;self.coordinates=coordinates;self.normals=normals
        self.entries={};self.hits=0;self.misses=0;self.last_stats={}
        self.baseline=SimpleNamespace(layers=np.arange(1,21),vertex={'coordinates':coordinates})

    def forecast(self,ids,layer1):
        ids=np.asarray(ids,dtype=int);start=time.perf_counter()
        key=identity(dict(version='Task3LocalForecast/1',sdf=self.sdf.source_identity,geometry=self.front.geometry_authority,
                          topology=self.top.sha256,schedule=self.schedule.sha256 if hasattr(self.schedule,'sha256') else array_hash(self.schedule.targets),vertices=ids.tolist(),layer1=array_hash(layer1)))
        if key in self.entries:
            self.hits+=1;self.last_stats=dict(cache_hit=True,recomputed_entries=0,seconds=time.perf_counter()-start);return self.entries[key]
        self.misses+=1
        # Private forecast origin only. No CommittedFrontState is created or changed.
        origin=SimpleNamespace(committed_layer=1,coordinates=layer1,target_distances=np.full(len(ids),self.schedule.targets[0]))
        future,projection=vertex_trajectories(origin,self.schedule,self.sdf,np.arange(len(ids)))
        initial=self.sdf.evaluate(layer1);values=np.concatenate((layer1[None],future['coordinates']))
        normal_values=np.concatenate((initial.normal[None],future['normal']))
        overlay=CoordinateOverlay(self.coordinates,ids,values);normals=CoordinateOverlay(self.normals,ids,normal_values)
        _,edges,triangles=dependencies(self.top,ids);edata={};tdata={};previous=self.front.coordinates
        descriptor_start=time.perf_counter()
        for k,x in enumerate(overlay):
            ed=edge_descriptors(x,previous,self.front.coordinates,normals[k],self.top.edges[edges])
            tr=triangle_descriptors(x,previous,self.front.coordinates,self.top.triangles[triangles])
            for dest,data in ((edata,ed),(tdata,tr)):
                for name,v in data.items():dest.setdefault(name,[]).append(v)
            previous=x
        result=SimpleNamespace(layers=np.arange(1,21),vertex={'coordinates':overlay},key=key,
                               local=dict(vertices=ids,coordinates=values,normal=normal_values,remaining_vertex=future,
                                          edges=edges,triangles=triangles,edge={k:np.stack(v) for k,v in edata.items()},triangle={k:np.stack(v) for k,v in tdata.items()}))
        n=len(ids)+len(edges)+len(triangles)
        self.last_stats=dict(cache_hit=False,invalidated_vertices=len(ids),invalidated_edges=len(edges),invalidated_triangles=len(triangles),
                             recomputed_entries=n,unchanged_entries=len(self.front.coordinates)+len(self.top.edges)+len(self.top.triangles)-n,
                             projection_seconds=projection,descriptor_seconds=time.perf_counter()-descriptor_start,seconds=time.perf_counter()-start)
        self.entries[key]=result
        # Bounded memo, not a database of every attempted trajectory.
        if len(self.entries)>8:self.entries.pop(next(iter(self.entries)))
        return result


def numeric_gates(result,*,candidate_only=False):
    c=result['cells'];sel=slice(0,1) if candidate_only else slice(None)
    tests={'volume':c['volume'][sel]<=0,'signed_volume':c['signed_volume'][sel]<=0,'jacobian':c['jacobian'][sel]<=0,
           'determinant':c['determinant'][sel]<.0011,'aspect':c['aspect'][sel]>1000}
    for key in ('volume','signed_volume','jacobian','determinant','aspect'):tests[key]|=~np.isfinite(c[key][sel])
    fi=[i for i,(kind,k,*_) in enumerate(result['face_ids']) if not candidate_only or kind=='side' and k==1]
    faces=result['faces'];limits={'weight':lambda x:x<.055,'skewness':lambda x:x>3.63636,'symmetric_volume_ratio':lambda x:x<.01}
    for key,fn in limits.items():tests[key]=fn(faces[key][fi])|~np.isfinite(faces[key][fi])
    counts={k:int(v.sum()) for k,v in tests.items()}
    extrema={k:dict(min=float(np.nanmin(c[k][sel])),max=float(np.nanmax(c[k][sel]))) for k in ('volume','signed_volume','jacobian','determinant','aspect','sigma','G_bound') if k in c}
    extrema.update({k:dict(min=float(np.nanmin(faces[k][fi])),max=float(np.nanmax(faces[k][fi]))) for k in limits})
    return dict(passed=not any(counts.values()),violations=counts,extrema=extrema,topology_valid=True,source_wall_ownership_valid=True,
                scope='All verification cells and complete internal side faces; exposed top cap has no physical neighbor' if candidate_only else 'All existing local future internal faces')


def failure_times(result):
    out={};c=result['cells']
    for metric,fn in [('volume',lambda x:x<=0),('signed_volume',lambda x:x<=0),('jacobian',lambda x:x<=0),('determinant',lambda x:x<.0011),('aspect',lambda x:x>1000)]:
        bad=fn(c[metric])|~np.isfinite(c[metric])
        for k,j in zip(*np.where(bad)):
            if k==0:continue
            key=f'cell:{int(c["column"][k,j])}:{metric}';out[key]=min(out.get(key,21),k+1)
    for i,(kind,k,entity,*_) in enumerate(result['face_ids']):
        available=k if kind=='side' else k+1
        if available==1:continue
        for metric,fn in [('weight',lambda x:x<.055),('skewness',lambda x:x>3.63636),('symmetric_volume_ratio',lambda x:x<.01)]:
            x=result['faces'][metric][i]
            if fn(x) or not np.isfinite(x):
                key=f'{kind}:{entity}:{metric}';out[key]=min(out.get(key,21),available)
    return out


def no_new_earlier_failure(baseline,candidate):
    return {k:dict(before=baseline.get(k,21),after=v) for k,v in candidate.items() if v<baseline.get(k,21)}


def sorted_tails(result):
    c=result['cells'];f=result['faces']
    return dict(jacobian=np.sort(c['jacobian'].ravel()),determinant=np.sort(c['determinant'].ravel()),
                weight=np.sort(f['weight']),symmetric_volume_ratio=np.sort(f['symmetric_volume_ratio']),
                sigma_column_minima=np.sort(c['sigma'].min(0)))


def target_status(result,columns,metric):
    c=result['cells'];selected=np.isin(c['column'][0],columns)
    if metric=='weight':
        pairs=[(k if kind=='side' else k+1,result['faces']['weight'][i]) for i,(kind,k,_,a,b) in enumerate(result['face_ids']) if (a in columns or b in columns) and (kind!='side' or k>1)]
        bad=[k for k,v in pairs if not np.isfinite(v) or v<.055];tail=np.sort([v for _,v in pairs])
    else:
        x=c[metric][1:,selected];limit=.0011 if metric=='determinant' else 0.
        bad=(x<limit if metric=='determinant' else x<=limit)|~np.isfinite(x)
        layers=np.where(bad)[0]+2;bad=layers.tolist();tail=np.sort(x.ravel())
    return dict(removed=not len(bad),first_invalid=min(bad) if len(bad) else None,tail=tail)


def hazard_improves(old,new):
    a=(old['removed'],old['first_invalid'] if old['first_invalid'] is not None else 21)
    b=(new['removed'],new['first_invalid'] if new['first_invalid'] is not None else 21)
    if b!=a:return b>a
    for x,y in zip(old['tail'],new['tail']):
        if y!=x:return bool(y>x)
    return False


def exact_candidate_jacobians(front,top,forecast,patch):
    x=forecast.vertex['coordinates'][0,top.triangles[patch.core_columns]]
    low=front.coordinates[top.triangles[patch.core_columns]]
    records=[dict(column=int(c),**exact_wedge(np.concatenate((a,b)))) for c,a,b in zip(patch.core_columns,low,x)]
    return dict(passed=all(r['jacobian_sign']>0 and r['volume_sign']>0 for r in records),records=records)

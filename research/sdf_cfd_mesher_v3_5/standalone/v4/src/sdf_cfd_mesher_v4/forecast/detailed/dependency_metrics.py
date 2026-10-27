"""Exact-input local metric updates. No interpolation, tolerance, or stale rows.

A cell owns all five reconstruction rows of the existing production operator.
Its six corner coordinates, layer and topology completely determine every
returned cell metric. A face additionally depends on both adjacent centres and
volumes. Cached closure is bound to actual topology arrays and ordered support.
Only a bounded number of local supports are retained. Eviction changes work,
never results. Returned records never alias a subsequently mutable cache array.
"""
from collections import OrderedDict
import numpy as np
from ...common.authority import identity,array_hash
from .patch import require_closure
from .metrics import cell_metrics,legacy_modules

VERSION='V4DependencyMetrics/1'
FACE_KEYS=('weight','skewness','symmetric_volume_ratio','nonorthogonality')

def changed_rows(a,b):
    if a.shape!=b.shape:raise ValueError('GeometryShapeChanged')
    return np.any(np.ascontiguousarray(a).view(np.uint64)!=np.ascontiguousarray(b).view(np.uint64),axis=tuple(range(1,a.ndim)))

def geometry_dependencies(previous,current,local_triangles,local_edges):
    """Exact incidence propagation; arrays retain layer and stable local order."""
    vertices=np.ones(current.shape[:2],bool) if previous is None else np.any(current.view(np.uint64)!=previous.view(np.uint64),axis=2)
    triangles=np.any(vertices[:,local_triangles],axis=2);edges=np.any(vertices[:,local_edges],axis=2)
    cells=triangles[:-1]|triangles[1:];sides=edges[:-1]|edges[1:]
    return dict(vertices=vertices,edges=edges,triangles=triangles,cells=cells,side_surfaces=sides)

class LocalMetricEngine:
    def __init__(self,max_supports=4):
        if max_supports<1:raise ValueError('PositiveSupportCacheBoundRequired')
        self.states=OrderedDict();self.max_supports=max_supports;self.last_stats={}

    def structure(self,top,patch):
        owners=np.asarray([top.edge_triangles.row(e) for e in patch.side_edges])
        key=identity(dict(version=VERSION,topology=top.sha256,column_count=len(top.triangles),patch=patch.record(),triangles=array_hash(top.triangles[patch.columns]),
            edges=array_hash(top.edges[patch.side_edges]),owners=array_hash(owners),triangle_edges=array_hash(top.triangle_edges[patch.core_columns])))
        if key in self.states:
            self.states.move_to_end(key);return self.states[key],True
        require_closure(patch,top)
        s=dict(local=np.searchsorted(patch.vertices,top.triangles[patch.columns]),core=np.searchsorted(patch.columns,patch.core_columns),
            edges=np.searchsorted(patch.vertices,top.edges[patch.side_edges]),owners=owners,ix=np.searchsorted(patch.columns,owners))
        s['all_edge_ids']=np.unique(top.triangle_edges[patch.columns])
        s['all_edges']=np.searchsorted(patch.vertices,top.edges[s['all_edge_ids']])
        s['face_ids']=[]
        for k in range(20):
            s['face_ids'].extend(('side',k+1,int(e),int(a),int(b)) for e,(a,b) in zip(patch.side_edges,owners))
            if k>0:s['face_ids'].extend(('cap',k,int(c),int(c),int(c)) for c in patch.core_columns)
        self.states[key]=s
        while len(self.states)>self.max_supports:self.states.popitem(last=False)
        return s,False

    def measure(self,front,forecast,top,patch):
        if front.committed_layer!=0 or not np.array_equal(forecast.layers,np.arange(1,21)):raise ValueError('Task2RequiresFrozenTwentyLayerForecast')
        s,closure_hit=self.structure(top,patch);nc=len(patch.columns);core=s['core'];local=s['local']
        x=np.concatenate((front.coordinates[None,patch.vertices],forecast.vertex['coordinates'][:,patch.vertices]))
        p=np.concatenate((x[:-1,local],x[1:,local]),axis=2).reshape(-1,6,3)
        ls=np.repeat(np.arange(1,21),nc)
        dependencies=geometry_dependencies(s.get('x'),x,local,s['all_edges']);mask=dependencies['cells'].ravel()
        if 'g' in s:
            g={k:v.copy() for k,v in s['g'].items()}
            if mask.any():
                for k,v in cell_metrics(p[mask],ls[mask],required_gain_only=True).items():g[k][mask]=v
        else:g=cell_metrics(p,ls,required_gain_only=True)
        shaped={k:v.reshape((20,nc)+v.shape[1:]) for k,v in g.items()}
        cq=legacy_modules()['component_quality'];fg=cq.FG;faces=list(s['face_ids']);values={k:[] for k in FACE_KEYS};face_cache=[];face_changed=face_total=0
        def face_batch(vs,Co,Cn,Vo,Vn,tri):
            nonlocal face_changed,face_total
            inputs=np.concatenate((vs.reshape(len(vs),-1),Co,Cn,Vo[:,None],Vn[:,None]),axis=1)
            prior=s.get('faces',[]);old=prior[len(face_cache)] if prior else None
            change=changed_rows(inputs,old[0]) if old is not None else np.ones(len(vs),bool)
            out=[v.copy() for v in old[1]] if old is not None else [np.empty(len(vs)) for _ in FACE_KEYS]
            if change.any():
                S,Cf=(fg.face_area_centre_tri if tri else fg.face_area_centre_quad)(vs[change])
                val=cq.face_metrics(vs[change],S,Cf,Co[change],Cn[change],Vo[change],Vn[change])
                for a,b in zip(out,val):a[change]=b
            face_cache.append((inputs,out));face_changed+=int(change.sum());face_total+=len(vs)
            for k,v in zip(values,out):values[k].append(v)
        for k in range(20):
            ed=s['edges'];ix=s['ix'];C=shaped['centre'];V=shaped['volume']
            vs=np.stack((x[k,ed[:,0]],x[k,ed[:,1]],x[k+1,ed[:,1]],x[k+1,ed[:,0]]),axis=1)
            face_batch(vs,C[k,ix[:,0]],C[k,ix[:,1]],V[k,ix[:,0]],V[k,ix[:,1]],False)
            if k>0:
                face_batch(x[k,local[core]],C[k-1,core],C[k,core],V[k-1,core],V[k,core],True)
        cells={k:v[:,core] for k,v in shaped.items()}
        cells['column']=np.broadcast_to(patch.core_columns,(20,len(core))).copy()
        cells['layer']=np.broadcast_to(np.arange(1,21)[:,None],(20,len(core))).copy()
        cells['cell_id']=(cells['layer']-1)*len(top.triangles)+cells['column']
        s.update(x=x,p=p,g=g,faces=face_cache)
        self.last_stats=dict(closure_hit=closure_hit,cell_recomputed=int(mask.sum()),cell_reused=int((~mask).sum()),
            face_recomputed=face_changed,face_reused=face_total-face_changed,reconstruction_rows_recomputed=5*int(mask.sum()),
            reconstruction_rows_reused=5*int((~mask).sum()),conditioning_recomputed=int(mask.sum()),conditioning_reused=int((~mask).sum()),
            changed_vertex_states=int(dependencies['vertices'].sum()),changed_edge_states=int(dependencies['edges'].sum()),
            changed_triangle_states=int(dependencies['triangles'].sum()),changed_side_surface_inputs=int(dependencies['side_surfaces'].sum()),
            changed_cell_ids=np.flatnonzero(mask).tolist())
        return dict(cells=cells,face_ids=faces,faces={k:np.concatenate(v) for k,v in values.items()},
            unavailable=dict(outer_cap_neighbor_metrics=len(core),reason='No exterior core mesh exists in the provisional forecast'),
            counts=dict(core_vertices=len(patch.core_vertices),halo_vertices=len(patch.vertices)-len(patch.core_vertices),columns=nc,
                core_columns=len(core),prisms=20*nc,measured_prisms=20*len(core),faces=len(faces),reconstruction_rows=100*len(core),sensitivity_dofs=0))

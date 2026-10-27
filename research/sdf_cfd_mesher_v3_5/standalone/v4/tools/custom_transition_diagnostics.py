"""Shape and size observations for changed outer patches; never acceptance.

Mean-ratio arithmetic matches shape_transition_builder.measure. Edge lengths
are reported separately so a size change is not called a shape improvement.
"""
from itertools import combinations
import numpy as np
from background_topology_repair import key

QUANTILES=[0.,.01,.1,.5,.9,1.]


def distribution(values):
    values=np.asarray(values)
    return np.quantile(values,QUANTILES).tolist() if len(values) else None


def shapes(points,tets,faces):
    p=points[tets]
    volume6=np.einsum('ni,ni->n',p[:,1]-p[:,0],np.cross(p[:,2]-p[:,0],p[:,3]-p[:,0]))
    lengths=sum(np.sum((p[:,i]-p[:,j])**2,axis=1) for i,j in combinations(range(4),2))
    ratio=12*np.power(np.abs(volume6)/2,2/3)/lengths
    tri=points[faces]
    edges=np.stack((tri[:,1]-tri[:,0],tri[:,2]-tri[:,1],tri[:,0]-tri[:,2]),axis=1)
    squared=np.sum(edges*edges,axis=2)
    tratio=2*np.sqrt(3)*np.linalg.norm(np.cross(tri[:,1]-tri[:,0],tri[:,2]-tri[:,0]),axis=1)/squared.sum(axis=1)
    return dict(tet_mean_ratio=distribution(ratio),triangle_mean_ratio=distribution(tratio))


def changed_outer(s,records):
    original={}
    for row in records:
        if row['adopted']:original.setdefault(row['vertex'],row['before'])
    moving=set(original);outer_vertices={int(v) for v in s['outer'].ravel()}
    outer_owners={i for f in s['outer'] for i,_ in s['owners'][key(f)]}
    tets=np.asarray([c['vertices'] for c in s['additions'] if c['id'] in outer_owners and moving.intersection(c['vertices'])],dtype='<i8').reshape(-1,4)
    faces=np.asarray([f for f in s['outer'] if moving.intersection(f)],dtype='<i8').reshape(-1,3)
    before=s['points'].copy()
    for vertex,point in original.items():before[vertex]=point
    edges=np.asarray(sorted({key(e) for row in tets for e in combinations(row,2)}),dtype='<i8').reshape(-1,2)
    lookup=np.full(len(before),-1,dtype='<i8');lookup[s['compact_ids']]=np.arange(len(s['compact_ids']))
    tensors=(s['custom_metric'][lookup[edges[:,0]]]+s['custom_metric'][lookup[edges[:,1]]])*.5
    def observation(points):
        d=points[edges[:,1]]-points[edges[:,0]]
        return dict(shapes(points,tets,faces),edge_length_m=distribution(np.linalg.norm(d,axis=1)),
            metric_edge_length=distribution(np.sqrt(np.maximum(0,np.einsum('ni,nij,nj->n',d,tensors,d)))))
    return dict(diagnostic_only=True,quantiles=QUANTILES,outer_population=len(outer_vertices),
        attempted_outer_vertices=len({r['vertex'] for r in records if r['vertex'] in outer_vertices}),
        moved_outer_vertices=len(moving&outer_vertices),changed_outer_tets=len(tets),changed_outer_faces=len(faces),
        before=observation(before),after=observation(s['points']),
        scope='Coordinate changes on the final connectivity only; topology edits are excluded from this patch comparison. Regular tet/equilateral triangle=1, no additional acceptance threshold.')

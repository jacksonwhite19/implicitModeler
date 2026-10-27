"""Descriptive realized sizes only; existing quality owners decide acceptance."""
import numpy as np

def distribution(values):
    a=np.asarray(values,dtype=float).reshape(-1)
    if not len(a):return dict(count=0,quantiles=None)
    if not np.isfinite(a).all():raise ValueError('NonfiniteJoinedSizeDiagnostic')
    return dict(count=len(a),probabilities=[0,.01,.5,.99,1],quantiles=np.quantile(a,[0,.01,.5,.99,1]).tolist())

def measure(points,rows,transition_count,front,triangles,owners,cells,check):
    from itertools import combinations
    from scipy.spatial import cKDTree
    count=len(rows);volumes=np.empty(count);lengths=np.empty((count,6));centres=np.empty((count,3))
    pairs=list(combinations(range(4),2))
    for start in range(0,count,4096):
        check();stop=min(count,start+4096);q=points[rows[start:stop]]
        volumes[start:stop]=np.abs(np.einsum('ij,ij->i',q[:,1]-q[:,0],np.cross(q[:,2]-q[:,0],q[:,3]-q[:,0])))/6
        centres[start:stop]=q.mean(1)
        lengths[start:stop]=np.stack([np.linalg.norm(q[:,a]-q[:,b],axis=1) for a,b in pairs],axis=1)
    lookup={c['id']:i for i,c in enumerate(cells)};adjacent=set();ratios=[];edge_ratios=[];internal_ratios=[]
    for uses in owners.values():
        if len(uses)!=2:continue
        a,b=(lookup.get(use[0]) for use in uses)
        if a is None or b is None:continue
        if a>=transition_count and b>=transition_count:
            internal_ratios.append(min(volumes[a],volumes[b])/max(volumes[a],volumes[b]))
        elif (a<transition_count)!=(b<transition_count):
            t,k=(a,b) if a<transition_count else (b,a);adjacent.add(k)
            ratios.append(volumes[k]/volumes[t]);edge_ratios.append(lengths[k].mean()/lengths[t].mean())
    bulk=np.arange(transition_count,count);ids=sorted(adjacent)
    distances=cKDTree(front).query(centres[bulk],workers=1)[0];cuts=np.quantile(distances,[0,.25,.5,.75,1]);bands=[]
    for i in range(4):
        selected=(distances>=cuts[i]) & ((distances<=cuts[i+1]) if i==3 else (distances<cuts[i+1]))
        selected_ids=bulk[selected]
        bands.append(dict(nearest_front_vertex_distance_range_m=cuts[i:i+2].tolist(),cells=len(selected_ids),
            edges_m=distribution(lengths[selected_ids]),volumes_m3=distribution(volumes[selected_ids])))
    check();q=front[triangles]
    return dict(scientific_acceptance=False,scope='Final repaired bulk; edge distributions count six edge occurrences per tetrahedron, not unique edges. Distance bins use nearest front vertex, not exact distance to surface.',
        front_edges_m=distribution(np.stack([np.linalg.norm(q[:,a]-q[:,b],axis=1) for a,b in ((0,1),(1,2),(2,0))],axis=1)),
        adjacent_bulk_cells=len(ids),adjacent_bulk_edges_m=distribution(lengths[ids]),adjacent_bulk_volumes_m3=distribution(volumes[ids]),
        bulk_edges_m=distribution(lengths[bulk]),bulk_volumes_m3=distribution(volumes[bulk]),
        bulk_to_transition_volume_ratio=distribution(ratios),bulk_to_transition_mean_edge_ratio=distribution(edge_ratios),
        bulk_neighbour_symmetric_volume_ratio=distribution(internal_ratios),distance_bands=bands)

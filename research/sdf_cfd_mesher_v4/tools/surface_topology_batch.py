"""Batch incidence for native triangle arrays; retain the scalar owner's fallback."""
import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components


def topology(x,f,scalar,*,expected_components=1,expected_euler=2):
    options=dict(expected_components=expected_components,expected_euler=expected_euler)
    if not (type(x) is np.ndarray and x.ndim==2 and x.shape[1]==3
            and type(f) is np.ndarray and f.ndim==2 and f.shape[1]==3
            and f.dtype.kind in 'iu' and (not f.size or (f.min()>=0 and f.max()<len(x)))):
        return scalar(x,f,**options)
    directed=np.concatenate((f[:,[0,1]],f[:,[1,2]],f[:,[2,0]]))
    edges,inverse,counts=np.unique(np.sort(directed,axis=1),axis=0,return_inverse=True,return_counts=True)
    used=np.unique(f)
    upward=np.bincount(inverse[directed[:,0]<directed[:,1]],minlength=len(edges))
    downward=np.bincount(inverse[directed[:,0]>directed[:,1]],minlength=len(edges))
    orientation=(upward!=1)|(downward!=1)
    self_edges=edges[:,0]==edges[:,1]
    orientation[self_edges]=counts[self_edges]!=1
    graph=coo_matrix((np.ones(len(edges),dtype=bool),(edges[:,0],edges[:,1])),shape=(len(x),len(x)))
    components=int(connected_components(graph,directed=False,return_labels=False))-(len(x)-len(used))
    row=dict(vertices=len(x),faces=len(f),edges=len(edges),connected_components=components,
        unused_vertices=len(x)-len(used),boundary_edges=int(np.count_nonzero(counts==1)),
        nonmanifold_edges=int(np.count_nonzero(counts>2)),
        duplicate_triangles=len(f)-len(np.unique(np.sort(f,axis=1),axis=0)),
        duplicate_vertices=len(x)-len(np.unique(x,axis=0)),
        orientation_edge_failures=int(np.count_nonzero(orientation)),euler=len(x)-len(edges)+len(f))
    row['passed']=((expected_components is None or components==expected_components)
        and (expected_euler is None or row['euler']==expected_euler)
        and not any(row[k] for k in ('unused_vertices','boundary_edges','nonmanifold_edges',
            'duplicate_triangles','duplicate_vertices','orientation_edge_failures')))
    return row

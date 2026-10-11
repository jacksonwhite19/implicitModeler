"""Normalize incidental core node/cell ordering for scientific replay."""
from pathlib import Path
import json
import sys
import numpy as np
from sdf_cfd_mesher_v4.common.authority import array_hash, file_hash


def normalized(folder):
    xyz=np.load(folder/'coordinates.npy',mmap_mode='r')
    tets=np.load(folder/'tets.npy',mmap_mode='r')
    order=np.lexsort((xyz[:,2],xyz[:,1],xyz[:,0]))
    inverse=np.empty(len(order),dtype=np.int64);inverse[order]=np.arange(len(order))
    cells=np.sort(inverse[tets],axis=1)
    cells=cells[np.lexsort(tuple(cells[:,j] for j in (3,2,1,0)))]
    tags=np.load(folder/'node_tags.npy'); tagorder=np.argsort(tags)
    boundary={}
    for name in ('interface_tags','farfield_tags'):
        source=np.load(folder/(name+'.npy'))
        ids=tagorder[np.searchsorted(tags[tagorder],source)]
        boundary[name]=array_hash(inverse[ids])
    return dict(coordinates=array_hash(xyz[order]),cells=array_hash(cells),boundary=boundary,
                points=len(xyz),tetrahedra=len(tets))


if __name__=='__main__':
    a,b,out=map(Path,sys.argv[1:])
    left,right=normalized(a),normalized(b)
    result=dict(passed=left==right,left=left,right=right,
                normalization='Lexicographic coordinates; canonical vertex sets and lexicographic cell order; source-order boundary identities retained',
                scope='Core generation only; not full-mesh qualification')
    out.write_text(json.dumps(result,sort_keys=True,indent=2)+'\n')
    print(json.dumps(result))

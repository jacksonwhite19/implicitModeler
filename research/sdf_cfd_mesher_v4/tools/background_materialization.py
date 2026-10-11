"""Same existing source-tet dictionaries, without repeated NumPy row copies."""
import numpy as np
PATTERNS=((0,2,1),(0,1,3),(1,2,3),(2,0,3))

def original(mapped,last):
    return [dict(id=last+i,column=-1,layer=0,kind='tetrahedron',vertices=row.tolist(),
                 faces=[tuple(map(int,row[list(p)])) for p in PATTERNS]) for i,row in enumerate(mapped)]

def build(mapped,last):
    # Only the fresh mapping output of the existing joined qualifier. Hooks,
    # other shapes/types and altered integer semantics retain the original.
    if (type(mapped) is not np.ndarray or mapped.dtype!=np.dtype('int64') or
            mapped.ndim!=2 or mapped.shape[1:]!=(4,) or type(last) is not int):
        return original(mapped,last)
    cells=[]
    for i,row in enumerate(mapped):
        a,b,c,d=vertices=row.tolist()
        cells.append(dict(id=last+i,column=-1,layer=0,kind='tetrahedron',vertices=vertices,
                          faces=[(a,c,b),(a,b,d),(b,c,d),(c,a,d)]))
    return cells

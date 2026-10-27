"""Same independent tensor expressions on fresh bounded Foundation areas."""
from collections import defaultdict
import numpy as np

def determinants(areas,cells,owners,selected):
    # Quiet eligibility must not execute hooks on unsupported containers or
    # leaves before the original cell/exact traversal owns any failure.
    if (type(cells) is not list or
        (type(owners) is not dict and type(owners) is not defaultdict) or
        (type(selected) is not set and type(selected) is not tuple)):
        return None
    if (np.__version__!='1.26.4' or np.geterr()!=dict(divide='warn',over='warn',under='ignore',invalid='warn') or
        type(areas) is not np.ndarray or areas.dtype!=np.dtype('float64') or areas.ndim!=3 or areas.shape[2]!=3 or
        len(areas)!=len(cells) or len(cells)>4096):
        return None
    # The consumer constructs these flags from its freshly selected source
    # cells. Inspect only this bounded batch, not the complete mesh selection
    # again for every batch. Keep the standalone set-input diagnostic owner.
    if type(selected) is tuple:
        if len(selected)!=len(cells) or any(type(v) is not bool for v in selected):return None
    elif any(type(i) is not int for i in selected):return None
    groups={};seen=set()
    try:
        for j,c in enumerate(cells):
            if type(c) is not dict or type(c['id']) is not int:return None
            i=c['id']
            if not (selected[j] if type(selected) is tuple else i in selected):continue
            if i in seen:return None
            seen.add(i)
            if type(c['faces']) is not list or len(c['faces'])!=areas.shape[1]:return None
            mask=[]
            for face in c['faces']:
                if type(face) is not tuple or len(face) not in (3,4) or any(type(v) is not int for v in face):return None
                k=tuple(sorted(face))
                if k not in owners:return None
                rows=owners[k]
                if type(rows) is not list:return None
                mask.append(len(rows)==2)
            if not any(mask):return None
            groups.setdefault(tuple(mask),[]).append((j,i))
        result={}
        with np.errstate(divide='raise',over='raise',invalid='raise',under='ignore'):
            for mask,group in groups.items():
                S=areas[np.asarray([j for j,_ in group])][:,np.asarray(mask)]
                if not np.isfinite(S).all():return None
                avg=np.linalg.norm(S,axis=2).mean(axis=1);scaled=S/avg[:,None,None]
                tensor=scaled.transpose(0,2,1)@scaled;values=np.linalg.det(tensor)
                if not np.isfinite(avg).all() or np.any(avg<=0) or not np.isfinite(tensor).all() or not np.isfinite(values).all():return None
                result.update((i,float(v)) for (_,i),v in zip(group,values))
        return result
    except (TypeError,ValueError,IndexError,KeyError,FloatingPointError,OverflowError,np.linalg.LinAlgError):
        return None

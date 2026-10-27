"""Fresh terminal-prism batching with the existing scalar fallback owner.

This is a call-local prepass, not a reuse receipt. Caller incidence/positive
volume checks remain in their original order. Exceptional arithmetic discards
all provisional values without emitting an earlier warning.
"""
import numpy as np

def prepass(points,topology,owner_ids):
    if (np.__version__!='1.26.4' or np.geterr()!=dict(divide='warn',over='warn',under='ignore',invalid='warn') or
            type(points) is not np.ndarray or points.dtype!=np.dtype('float64') or points.ndim!=2 or
            points.shape[1:]!=(3,) or type(topology.cells) is not dict or type(owner_ids) is not list):
        return None
    try:
        rows=[]
        for owner in owner_ids:
            if type(owner) is not str:return None
            c=topology.cells[int(owner)]
            if (type(c) is not dict or type(c.get('id')) is not int or
                    type(c.get('kind')) is not str or c['kind']!='prism' or
                    type(c.get('faces')) is not list or len(c['faces'])!=5):return None
            if int(owner)!=c['id']:return None
            for face in c['faces']:
                if (type(face) is not tuple or len(face) not in (3,4) or
                        any(type(v) is not int or not 0<=v<len(points) for v in face)):return None
            if tuple(map(len,c['faces']))!=(3,3,4,4,4):return None
            rows.append(c)
        if not np.isfinite(points).all():return None
        from background_mixed_geometry import batch_geometry
        result={}
        with np.errstate(divide='raise',over='raise',invalid='raise',under='ignore'):
            for start in range(0,len(rows),4096):
                batch=batch_geometry(points,rows[start:start+4096])
                for g in batch.values():
                    if (not np.isfinite(g['centre']).all() or not np.isfinite(g['volume']) or
                            not np.isfinite(g['areas']).all() or not np.isfinite(g['face_centres']).all() or
                            not np.isfinite(g['aspect'])):return None
                # Only the two fresh quantities consumed by the handoff.
                # Do not retain face geometry for every terminal prism.
                result.update((i,dict(volume=g['volume'],centre=g['centre'])) for i,g in batch.items())
        return result
    except (TypeError,ValueError,IndexError,KeyError,FloatingPointError,OverflowError):
        return None

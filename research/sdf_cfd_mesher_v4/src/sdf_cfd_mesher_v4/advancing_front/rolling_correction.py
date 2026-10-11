"""Generic correction support from current exact corner and hard-face evidence."""
import numpy as np
from ..forecast.exact_dyadic import exact_wedge
from ..runtime.regions import extract_regions,dependencies
from .cap_diagnostics import bottom_corner,active_patch

def controlling_support(front,top,candidate,events):
    """The exact bottom corner reads its own top vertex, not its neighbors.

    Other Jacobian extrema use all three top vertices. Remaining independent
    hard-face/cell failures add their actual source vertices. No cap ID, span
    coordinate, or historical cohort is consulted.
    """
    field=np.zeros_like(front.coordinates);active=set();traces=[]
    jac_columns=sorted({e['entity'] for e in events if e['kind']=='cell' and e['metric']=='jacobian'})
    for c in jac_columns:
        ids=top.triangles[c];wall=front.coordinates[ids];high=candidate[ids];controlled=False
        area=np.cross(wall[1]-wall[0],wall[2]-wall[0]);area/=np.linalg.norm(area)
        for j,v in enumerate(ids):
            evidence=bottom_corner(wall,high,j)
            if evidence['sign']<=0:
                active.add(int(v));field[v]+=area;controlled=True
                traces.append(dict(column=c,corner=j,vertex=int(v),**evidence))
        if not controlled:
            evidence=exact_wedge(np.concatenate((wall,high)))
            if evidence['jacobian_sign']<=0:
                active.update(map(int,ids));field[ids]+=area;traces.append(dict(column=c,all_top_vertices=ids.tolist(),**evidence))
    _,_,affected=dependencies(top,sorted(active));affected=set(map(int,affected))
    for e in events:
        if set(e['columns'])&affected:continue
        for c in e['columns']:
            ids=top.triangles[c];wall=front.coordinates[ids];area=np.cross(wall[1]-wall[0],wall[2]-wall[0]);area/=np.linalg.norm(area)
            active.update(map(int,ids));field[ids]+=area
    mask=np.zeros(len(field),bool);mask[list(active)]=True
    regions=extract_regions(top,mask,max_vertices=2048)
    return regions,field,traces

def bounded_patch(top,vertices):
    return active_patch(top,vertices,max_vertices=2048,max_columns=12288)

def correction_fields(front,top,patch,field):
    """Deterministic existing tangent DOFs; no normal-distance degree of freedom."""
    modes=[('failed_corner_area',field[patch.active]),('opposite_failed_corner_area',-field[patch.active])]
    # Coordinate axes are generic trial directions projected into the same
    # qualified local tangent plane, not a cap/span-specific normal model.
    for axis in range(3):
        for sign in (1.,-1.):
            f=np.zeros((len(patch.active),3));f[:,axis]=sign;modes.append((f'axis_{axis}_{sign:+g}',f))
    return modes

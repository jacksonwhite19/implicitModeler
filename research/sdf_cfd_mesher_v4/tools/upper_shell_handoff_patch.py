"""Bounded experimental edge bisection with conformal mixed shell cells.

No publisher. An invalid patch returns evidence and cannot change live state.
Unsplit external quadrilaterals remain pyramid bases. Changed shared facets
use one deterministic triangulation on both sides.
"""
import numpy as np


def split_face(face, edge, new_vertex):
    face=tuple(face);edge=set(edge)
    if not edge.issubset(face):return [face]
    j=next((j for j,a in enumerate(face) if {a,face[(j+1)%len(face)]}==edge),None)
    if j is None:raise ValueError('RefinementRequiresPerimeterEdge')
    ring=list(face[:j+1])+[new_vertex]+list(face[j+1:])
    anchor=min(set(face)-edge);k=ring.index(anchor);ring=ring[k:]+ring[:k]
    return [tuple((ring[0],ring[i],ring[i+1])) for i in range(1,len(ring)-1)]


def propose(points,cells,outer,edge,new_point):
    selected=[c for c in cells if set(edge).issubset(c['vertices'])]
    if not selected or any(c['layer']<=1 for c in selected):
        raise ValueError('WallFirstLayerProtected')
    if any(c['kind']!='prism' for c in selected):
        raise ValueError('PatchScopeRequiresPrismParents')
    # Include the entire edge star: splitting only one side would hang a node.
    points=np.asarray(points);coords=[*points,new_point];mid=len(points)
    uid=max(c['id'] for c in cells)+1;replacement=[]
    for parent in selected:
        centre=len(coords);coords.append(points[parent['vertices']].mean(0))
        for face in parent['faces']:
            for base in split_face(face,edge,mid):
                faces=[base]+[(b,a,centre) for a,b in zip(base,base[1:]+base[:1])]
                replacement.append(dict(id=uid,column=parent['column'],layer=parent['layer'],
                    kind={3:'tetrahedron',4:'pyramid'}[len(base)],vertices=[*base,centre],faces=faces))
                uid+=1
    gone={c['id'] for c in selected};newcells=[c for c in cells if c['id'] not in gone]+replacement
    newouter=[part for face in outer for part in split_face(face,edge,mid)]
    return np.asarray(coords),newcells,newouter,dict(removed=sorted(gone),replacement=[c['id'] for c in replacement],inserted=list(range(len(points),len(coords))),terminal_vertex=mid)


def publishable(evidence):
    # Future publication requires independent evidence for every obligation.
    required=('shell_quality','complete_neighbor','ownership','target_admission','wall_first_layer',
              'global_filling','continuous_certificate','core_handoff_before_after','deterministic_replay')
    return all(evidence.get(k) is True for k in required)

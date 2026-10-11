"""Layer-generic changed-geometry certification from CommittedState.

This module has no oracle reader and no legacy whole-state constructor.
Numerical mesh incidence is a global certification query, not a checkpoint.
"""
from collections import ChainMap, Counter
from fractions import Fraction
import json, time
import numpy as np
from sdf_cfd_mesher_v4.common.authority import identity, array_hash
from sdf_cfd_mesher_v4.advancing_front.mixed_front import cycle, face_key, geometry_inputs


def append_components(before, additions, points, n):
    """Keep old maps behind references; write only changed geometry/ownership."""
    def source(v): return f'layer:{v//n}:source:{v%n}'
    changes = {k: {} for k in ('vertices', 'cells', 'faces', 'columns', 'prism_front', 'versions')}
    checked_vertices=set(); inherited_faces={}
    for c in additions:
        k = str(c['id'])
        if k in before['cells']: raise ValueError('AppendReplacesCommittedCell')
        vs = [source(v) for v in c['vertices']]
        fs = [list(cycle(tuple(source(v) for v in f))) for f in c['faces']]
        changes['cells'][k] = dict(kind=c['kind'], source=str(c['column']), layer=c['layer'], vertices=vs, faces=sorted(fs))
        for v, i in zip(vs, c['vertices']):
            if v in checked_vertices: continue
            checked_vertices.add(v)
            value = points[i].tolist()
            if v in before['vertices']:
                if before['vertices'][v] != value: raise ValueError('CommittedCoordinateMutation')
            else: changes['vertices'][v] = value
        for f in fs:
            fk = face_key(f)
            if fk not in changes['faces']:
                old = before['faces'].get(fk)
                inherited_faces[fk]=old
                changes['faces'][fk] = dict(vertices=list(min(cycle(f), cycle(tuple(reversed(f))))),
                                           owners={} if old is None else dict(old['owners']))
            changes['faces'][fk]['owners'][k] = f
        top = face_key(vs[3:]); column = str(c['column'])
        changes['columns'][column] = dict(status='ActivePrismColumn', last_prism_layer=c['layer'],
            exposed_prism_face=top, outer_ownership=[top])
        changes['prism_front'][column] = top
    for family in ('cell', 'face'):
        for k, row in changes[family+'s'].items():
            dep = family + ':' + k
            old = before['versions'].get(dep, dict(topology=0, geometry=0))
            prior = inherited_faces[k] if family == 'face' else before[family+'s'].get(k)
            changes['versions'][dep] = dict(topology=old['topology'] + int(prior != row),
                                           geometry=old['geometry'] + 1)
    state = {k: before[k] for k in before}
    state.update({k: ChainMap(v, before[k]) for k, v in changes.items()})
    state['outer'] = sorted(changes['prism_front'].values())
    inputs = geometry_inputs(dict(state, cells=changes['cells'], faces=changes['faces']))
    return state, changes, inputs

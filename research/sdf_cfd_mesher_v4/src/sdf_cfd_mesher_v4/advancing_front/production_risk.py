"""Existing Task-4 selection, factored without changing event semantics.

Retained supports are runtime outputs of this selector and accepted local work.
No study IDs, descriptor thresholds or layer-specific rules appear here.
"""
from types import SimpleNamespace
import numpy as np
from .rolling_correction import controlling_support
from .rolling_metrics import discover_regions
from ..runtime.regions import dependencies

def select_supports(coordinates,top,candidate,immediate_events,future_events,layer,retained=()):
    current=SimpleNamespace(coordinates=coordinates);patches=[];traces=[]
    if immediate_events:
        regions,field,traces=controlling_support(current,top,candidate,immediate_events)
        for region in regions:
            vertices=np.array(region.vertex_ids)
            for old in retained:
                if set(region.triangle_ids)&set(map(int,dependencies(top,old)[2])):vertices=np.union1d(vertices,old)
            patches.append(dict(vertices=vertices,field=field,name='immediate',required=True))
    if layer==1:return patches,traces
    covered=set(v for p in patches for v in p['vertices'])
    for region in discover_regions(top,[e for e in future_events if e['layer']>layer]):
        vertices=np.array(region.vertex_ids)
        if set(vertices)&covered:continue
        for old in retained:
            if set(region.triangle_ids)&set(map(int,dependencies(top,old)[2])):vertices=np.union1d(vertices,old)
        field=np.zeros_like(coordinates)
        selected=[e for e in future_events if e['layer']>layer and set(e['columns'])&set(region.triangle_ids)]
        columns=np.unique([c for e in selected for c in e['columns']]);f=top.triangles[columns]
        areas=np.cross(coordinates[f[:,1]]-coordinates[f[:,0]],coordinates[f[:,2]]-coordinates[f[:,0]])
        for ids,a in zip(f,areas):field[ids]+=a
        patches.append(dict(vertices=vertices,field=field,name='forecast',required=False));covered.update(map(int,vertices))
    return patches,traces

"""Persistent newest-first geometry views over guarded recursive components.

No prefix dictionary is merged. Membership does not deserialize/copy a row.
Returned records retain the existing copy-isolation contract.
"""
from collections.abc import Mapping
from contextlib import contextmanager
from copy import deepcopy
from unittest.mock import patch


class SegmentMap(Mapping):
    def __init__(self, owner, segments):
        self.owner=owner
        self.segments=tuple(segments)

    def __getitem__(self,key):
        self.owner.check_live()
        for segment in self.segments:
            if key in segment:
                value=segment[key]
                return deepcopy(value) if isinstance(segment,dict) else value
        raise KeyError(key)

    def __contains__(self,key):
        self.owner.check_live()
        return any(key in segment for segment in self.segments)

    def __iter__(self):
        self.owner.check_live()
        seen=set()
        # ChainMap enumerates oldest first; retain its exact key order.
        for segment in reversed(self.segments):
            for key in segment:
                if key not in seen:
                    seen.add(key)
                    yield key

    def __len__(self):
        return sum(1 for _ in self)


class LazyParent(Mapping):
    def __init__(self,factory):self.factory=factory;self.value=None
    def get_map(self):
        if self.value is None:self.value=self.factory()
        return self.value
    def __getitem__(self,key):return self.get_map()[key]
    def __contains__(self,key):return key in self.get_map()
    def __iter__(self):return iter(self.get_map())
    def __len__(self):return len(self.get_map())


@contextmanager
def persistent_component_views(base):
    from composable_state import CommittedState
    import immutable_state_access as access
    original=CommittedState.__getitem__;views={}
    base.check()

    def get(state,key):
        state.check_live()
        points=getattr(state,'numeric_point_views',{})
        if key in points:return points[key]
        if key not in ('vertices','cells','faces','versions','columns','prism_front'):
            return original(state,key)
        token=(id(state),key)
        if token in views:return views[token]
        if state.kind!='recursive' or not state.production:
            result=original(state,key)
        else:
            inherited=(LazyParent(lambda:get(state.parent,key)),)
            if key in ('vertices','columns','prism_front'):
                local=(state.hot[key],)
            else:
                if state.geometry_cache is None:
                    state.geometry_cache=access.read(state.verify(state.out/'GEOMETRY_DELTA.json'))
                local=(state.hot['faces'],state.geometry_cache[key]) if key=='faces' else (state.geometry_cache[key],)
            result=SegmentMap(state,(*local,*inherited))
        views[token]=result
        return result

    with patch.object(CommittedState,'__getitem__',get):
        yield
    base.check()


def install():
    import prism_hotpath_v2 as previous
    previous.install()
    previous.cached_component_views=persistent_component_views

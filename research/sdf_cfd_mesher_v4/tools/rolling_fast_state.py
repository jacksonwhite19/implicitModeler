"""Process-local rolling ownership for the existing ordinary FastLayer.

No persisted trust import, restart, certificate rendering or mixed correction.
Old-old collision acceptance is inherited only for immutable ordinary append.
The numerical caches/degree predicates remain owned by existing implementations.
"""
from collections import Counter, defaultdict
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
import hashlib, math
from contextlib import contextmanager
from types import SimpleNamespace
from fractions import Fraction
import weakref
import time
import numpy as np

from sdf_cfd_mesher_v4.common.authority import identity, array_hash
from sdf_cfd_mesher_v4.advancing_front.native_checkpoint import ENCODER, canonical_chunks

GEOMETRY = ('wall', 'first_layer', 'vertices', 'cells', 'faces', 'columns',
            'outer', 'prism_front', 'versions', 'authority', 'events')


def _native_entity_row(value):
    """Bound the existing encoder's one-shot input, not its scientific values.

    Only plain native containers/leaves can bypass the protocol-aware stream.
    A larger row takes that unchanged stream. This budget is an allocation
    control, not an admission, geometry or numerical threshold.
    """
    pending = [value]
    remaining = 256
    while pending:
        item = pending.pop()
        remaining -= 1
        if remaining < 0:
            return False
        kind = type(item)
        if kind is dict:
            if len(item) > remaining or any(type(k) is not str for k in item):
                return False
            pending.extend(item.values())
        elif kind is list or kind is tuple:
            if len(item) > remaining:
                return False
            pending.extend(item)
        elif not (kind is str or kind is int or kind is float or kind is bool or item is None):
            return False
    return True


def _live_component_chunks(value, _predicate=None):
    # Same bounded entity-row ENCODER.encode pattern as the existing persistent
    # component view. Same canonical bytes, freshly read on every invocation.
    # Protocol hooks, guarded mappings and exceptional types keep their owner.
    predicate = _native_entity_row if _predicate is None else _predicate
    if predicate(value):
        yield ENCODER.encode(value).encode()
    elif type(value) is dict:
        yield b'{'
        for index, key in enumerate(sorted(value)):
            if not isinstance(key, str):
                raise ValueError('NativeScientificKeysMustBeStrings')
            if index:
                yield b','
            yield ENCODER.encode(key).encode()
            yield b':'
            item = value[key]
            # Native canonical_chunks only dispatches mapping children. Hooks
            # on a non-mapping child are not newly made serialization inputs.
            if isinstance(item, Mapping):
                yield from _live_component_chunks(item, predicate)
            elif predicate(item):
                yield ENCODER.encode(item).encode()
            else:
                for chunk in ENCODER.iterencode(item):
                    yield chunk.encode()
        yield b'}'
    else:
        yield from canonical_chunks(value)


def _live_component_digest(value, _predicate=None):
    digest = hashlib.sha256()
    for chunk in _live_component_chunks(value, _predicate):
        digest.update(chunk)
    return digest.hexdigest()


def mesh_identity(state, *, _preparation=None):
    predicate = None
    if _preparation is not None:
        from background_preparation import require
        predicate = require(_preparation).entity_row
    return identity({k: _live_component_digest(state[k], predicate) for k in GEOMETRY})


def _identity_plain(value):
    """Same legacy clean values, without repeated NumPy checks on native leaves.

    Always traverse live containers. This is not an identity or acceptance cache.
    Keep the existing cleaner for NumPy values, subclasses and exceptional types.
    """
    kind = type(value)
    if kind is int or kind is str or kind is bool or value is None:
        return value
    if kind is float:
        return value if math.isfinite(value) else repr(value)
    # These exact NumPy scalar classes are produced by numeric face/owner
    # construction. Reuse item(), the legacy cleaner's conversion, without
    # re-entering its general ndarray/container dispatch for each index.
    # Subclasses, unsigned/exceptional types and hooked values keep that owner.
    if kind is np.int64 or kind is np.int32:
        return value.item()
    if kind is dict:
        return {str(k): _identity_plain(v) for k, v in value.items()}
    if kind is list or kind is tuple:
        # JSON already encodes these leaves and tuples exactly as legacy clean.
        # Scan every leaf, but do not allocate/call clean once per native index.
        # Floats (including nonfinite) and subclasses take the legacy-value path.
        if all(type(v) is int or type(v) is str or type(v) is bool or v is None
                for v in value):
            return value
        return [_identity_plain(v) for v in value]
    from generalize_local_remesh import s
    return s.r.w.clean(value)


def _index_plain(values):
    # Same clean(list(values)) values, prepared at the existing assembly pass.
    # Do not cast arbitrary values to int: that could hide a changed coordinate
    # index. NumPy values and malformed inputs retain legacy normalization.
    if type(values) is tuple or type(values) is list:
        # JSON represents a native tuple/list identically. Read every index;
        # only the temporary normalized copy can be avoided. No retained hash
        # or acceptance survives this call. Malformed/subclass values fall back.
        for v in values:
            if type(v) is not int:
                break
        else:
            return values
    return [v if type(v) is int else _identity_plain(v) for v in values]


def numeric_identity(points, topology, outer, wall):
    # Build canonical-ready rows once, not a second complete copy of the payload.
    return identity(dict(points=array_hash(points),
        cells=[_identity_plain(topology.cells[i]) for i in sorted(topology.cells)],
        owners=[[_index_plain(k), [[_identity_plain(i), _index_plain(f)] for i,f in rows]]
                for k,rows in sorted(topology.owners.items())],
        boundary=[_index_plain(k) for k in sorted(topology.boundary_keys)],
        outer=[_index_plain(f) for f in outer], wall=[_index_plain(f) for f in wall]))


def cache_identity(quality, volume, degree):
    """Bind live reuse caches, not a historical proof journal."""
    geometry={}
    if hasattr(quality,'geometry'):
        digest=hashlib.sha256()
        for i,g in sorted(quality.geometry.items()):
            digest.update(identity(dict(id=i,**{k:array_hash(v) if type(v) is np.ndarray else v for k,v in g.items()})).encode())
        geometry['geometry']=digest.hexdigest()
    return identity(_identity_plain(dict(cells=quality.cells, exact=quality.exact, prospective=quality.prospective,
        faces=[[list(k),v] for k,v in sorted(quality.faces.items())], masks=quality.masks,
        volume=str(volume.total), boundary_volume=str(volume.boundary_total),
        boundary=[[list(k),str(v)] for k,v in sorted(volume.boundary.items())],
        coordinates=[[k,list(v)] for k,v in sorted(volume.coordinates.items())],
        witnesses=[[k,list(map(str,v))] for k,v in sorted(volume.witness.items())],
        degree_points=array_hash(degree.points), degree_boundary=[list(f) for f in degree.boundary],
        degree_witnesses=[[k,list(map(str,v))] for k,v in sorted(degree.witness.items())],
        degree_root=degree.root, degree_authority=degree.implementation, degree_rows=degree.rows,**geometry)))


@dataclass(frozen=True)
class LiveAcceptance:
    mesh: str
    numeric: str
    caches: str
    authority: str
    count: int
    methods: tuple
    generation: int

    def background_inputs(self, run, *, _preparation=None, _copy_verifier_caches=False,_capture_volume=False):
        """Verify a detached parent once; reuse only its call-scoped read views."""
        return _background_inputs(self, run, _preparation=_preparation,
                                  _copy_verifier_caches=_copy_verifier_caches,_capture_volume=_capture_volume)

    def check(self, run, *, _preparation=None):
        if (run.tx.generation != self.generation
                or mesh_identity(run.tx.current, _preparation=_preparation) != self.mesh
                or identity(run.tx.current['authority']) != self.authority
                or numeric_identity(run.points,run.topology,run.outer,run.wall) != self.numeric
                or cache_identity(run.quality,run.volume,run.degree_state) != self.caches):
            raise ValueError('ChangedRollingAcceptedParent')
        run.provider.check()
        _check_numeric_mesh(run.points,run.tx.current,len(run.provider.x))
        return True


def _check_numeric_mesh(points,native,n):
    """Same ordered native/numeric binary64 coordinates, in bounded batches."""
    if len(points)!=len(native['vertices']):raise ValueError('RollingNumericMeshMismatch')
    for start in range(0,len(points),4096):
        end=min(start+4096,len(points))
        expected=np.asarray([native['vertices'][f'layer:{v//n}:source:{v%n}'] for v in range(start,end)],dtype='<f8')
        if points[start:end].tobytes()!=expected.tobytes():raise ValueError('RollingNumericMeshMismatch')


# This is input custody for the existing checker, not imported acceptance or a
# second verifier. The private detached containers never leave this registry.
_background_issued = weakref.WeakKeyDictionary()
_background_active = weakref.WeakKeyDictionary()
_background_views = weakref.WeakKeyDictionary()
_background_pending = weakref.WeakKeyDictionary()


def _capture_background(value, memo):
    kind=type(value)
    if kind in (str,int,float,bool,Fraction,np.int64,np.int32,np.float64) or value is None:
        return value
    ident=id(value)
    if ident in memo:return memo[ident]
    if kind is np.ndarray:
        if value.dtype.hasobject:raise ValueError('UnsupportedBackgroundParentInput')
        result=np.frombuffer(value.tobytes(),dtype=value.dtype).reshape(value.shape)
    elif kind is dict or kind is defaultdict:
        if kind is defaultdict and value.default_factory not in (None,list):
            raise ValueError('UnsupportedBackgroundParentInput','default_factory')
        # The source is not exposed after capture. In-place source mutation
        # during capture either gives the accepted values or fails verification.
        result={k:_capture_background(v,memo) for k,v in value.items()}
    elif kind in (list,tuple):
        rows=[_capture_background(v,memo) for v in value]
        result=rows if kind is list else tuple(rows)
    elif kind in (set,frozenset):result=frozenset(value)
    else:raise ValueError('UnsupportedBackgroundParentInput',kind.__name__)
    memo[ident]=result
    return result


class _BackgroundMapping(Mapping):
    __slots__=('__weakref__',)
    __hash__=object.__hash__
    __eq__=object.__eq__
    def __init__(self,owner,data):
        _background_views[self]=(owner,data)
    def __setattr__(self,*args):raise TypeError('ReadOnlyBackgroundInputs')
    def __len__(self):return len(_background_view_data(self)[1])
    def __iter__(self):return iter(_background_view_data(self)[1])
    def __getitem__(self,key):
        owner,data=_background_view_data(self);return _background_read(owner,data[key])


class _BackgroundRecord:
    __slots__=('__weakref__',)
    def __init__(self,owner,data):
        _background_views[self]=(owner,data)
    def __setattr__(self,*args):raise TypeError('ReadOnlyBackgroundInputs')
    def __getattr__(self,name):
        owner,data=_background_views[self];owner._open()
        if name not in data:raise AttributeError(name)
        return _background_read(owner,data[name])


def _background_read(owner,value):
    if type(value) is dict:return _BackgroundMapping(owner,value)
    if type(value) in (list,tuple):
        # Protected scalar tuples can be shared. Native/numeric capture already
        # detached them; neither their metadata nor their leaves can be changed.
        scalar=(str,int,float,bool,Fraction,np.int64,np.int32,np.float64,type(None))
        if type(value) is tuple and all(type(v) in scalar for v in value):return value
        # Numeric incidence rows contain only immutable integer face cycles.
        integer=(int,np.int64,np.int32)
        if value and all(type(row) is tuple and len(row)==2 and type(row[0]) in integer
            and type(row[1]) is tuple and all(type(i) in integer for i in row[1]) for row in value):
            return tuple(value)
        return tuple(_background_read(owner,v) for v in value)
    if type(value) is SimpleNamespace:return _BackgroundRecord(owner,vars(value))
    if type(value) is np.ndarray:
        raw,dtype,shape=owner._open()['arrays'][id(value)]
        # ndarray metadata is mutable even with a read-only buffer. Never
        # expose the private canonical ndarray, including through .base.
        return np.frombuffer(raw,dtype=dtype).reshape(shape)
    return value  # Immutable scalar, bytes-backed array or frozen acceptance.


def _background_view_data(view):
    owner,data=_background_views[view];owner._open();return owner,data


class BackgroundInputs:
    __slots__=('__weakref__',)
    def __init__(self,*args,**kwargs):raise TypeError('LiveBackgroundInputsFactoryRequired')
    def _open(self):
        if self not in _background_issued:raise ValueError('ClosedBackgroundInputs')
        return _background_issued[self]
    @property
    def view(self):return self._open()['view']
    @property
    def summary(self):
        data=self._open()
        return dict(schema='V4CallScopedBackgroundInputs/1',full_parent_verifications=data.get('full_parent_verifications',1),
            producer_acceptance_handoff=data.get('producer_acceptance_handoff',False),
            producer_issue_seconds=data.get('producer_issue_seconds',0.),
            protected_reuses=data['reuses'],capture_seconds=data['capture_seconds'],
            verification_seconds=data['verification_seconds'],mesh=data['token'].mesh,
            capture_phases=dict(data['capture_phases']),verifier_cache_copies=data['verifier_cache_copies'],
            custody='Detached parent; bytes-backed numeric arrays and read-only views; no restart or persisted trust import.')
    def require(self,view):
        data=self._open()
        if view is not data['view']:raise ValueError('ChangedBackgroundParentScope')
        self.require_origin(data['origin'])
        data['origin'].provider.check()  # Real geometry/source owner remains live.
        data['reuses']+=1
        return True
    def require_origin(self,run):
        """Bind a consumer to this issued scope, without importing acceptance."""
        data=self._open();origin=data['origin'];token=data['token']
        if (run is not origin or origin.live_acceptance is not token
            or origin.tx is not data['tx'] or origin.tx.current is not data['state']
            or origin.tx.generation!=token.generation or origin.provider is not data['provider']
            or origin.provider.layer!=data['layer']):
            raise ValueError('ChangedBackgroundParentScope')
        return True
    def extract(self):
        data=self._open()
        if data['native'] is None:raise ValueError('BackgroundInterfaceAlreadyExtracted')
        from sdf_cfd_mesher_v4.core.interface import _extract
        result=_extract(data['native'],digest=_live_component_digest)
        points,offsets,connectivity,metadata=result
        # Full metadata was freshly computed by the existing extract owner.
        # Only its existing wire commitment is consumed downstream; do not let
        # ancestry/version output aliases expose private parent dictionaries.
        metadata={k:v for k,v in metadata.items() if k not in ('ancestry','versions')}
        data['native']=None
        return points,offsets,connectivity,metadata
    def cells(self,ids=None,*,_reference=False):
        if type(_reference) is not bool:raise ValueError('InvalidBackgroundWorkingCopy')
        data=self._open();rows=data['numeric']['topology'].cells
        selected=rows if ids is None else ids
        # Existing arithmetic gets its ordinary fresh dict/list shapes. These
        # are working copies, never the protected inherited parent.
        return [(_capture_background(rows[i],{}) if _reference else _background_working_cell(rows[i]))
                for i in selected]

    def volume_inputs(self):
        """Detached accepted-volume working copy, captured at the issued check."""
        self.require(self.view);data=self._open()
        if data.get('volume_data') is None:raise ValueError('ProtectedBackgroundVolumeRequired')
        from incremental_layer_validation import ExactVolume
        result=object.__new__(ExactVolume)
        for name,value in data['volume_data'].items():setattr(result,name,_capture_background(value,{}))
        return result

    def quality_inputs(self,points,byid,owners,needed,selected):
        """Private authenticated prism results; bind actual geometry and masks."""
        data=self._open();numeric=data['numeric'];quality=data.get('quality')
        self.require_origin(data['origin']);data['origin'].provider.check()
        if quality is None:return {},{},{}
        n=len(numeric['points'])
        if (type(points) is not np.ndarray or points.dtype!=np.dtype('float64') or not points.dtype.isnative
                or points.ndim!=2 or points.shape[1]!=3 or len(points)<n):
            raise ValueError('ChangedInheritedQualityCoordinates')
        if points[:n].tobytes()!=numeric['points'].tobytes():raise ValueError('ChangedInheritedQualityCoordinates')
        geom={};exact={};determinants={}
        for i,g in getattr(quality,'geometry',{}).items():
            if i not in needed:continue
            if byid.get(i)!=numeric['topology'].cells[i]:raise ValueError('ChangedInheritedQualityCell')
            geom[i]=_capture_background(g,{})
        for i in selected & quality.cells.keys():
            if byid.get(i)!=numeric['topology'].cells[i]:raise ValueError('ChangedInheritedQualityCell')
            mask=tuple(len(owners[tuple(sorted(f))])==2 for f in byid[i]['faces'])
            if mask!=quality.masks[i]:raise ValueError('ChangedInheritedQualityMask')
            exact[i]=_capture_background({k:v for k,v in quality.exact[i].items() if k!='id'},{});determinants[i]=quality.prospective[i]['determinant']
        return geom,exact,determinants

    def terminal_geometry(self):
        data=self._open();self.require_origin(data['origin']);data['origin'].provider.check()
        quality=data.get('quality')
        return {} if quality is None else _capture_background(getattr(quality,'geometry',{}),{})


def _background_working_cell(row):
    # The accepted native numeric cell form has only scalar metadata, a vertex
    # list and a face list of immutable index tuples. Copy both mutable lists;
    # sharing those tuples cannot change the protected parent. Exceptional
    # shapes retain the existing recursive copier and its rejection behavior.
    leaves=(str,int,float,bool,Fraction,np.int64,np.int32,np.float64,type(None))
    if (type(row) is dict and type(row.get('vertices')) is list and type(row.get('faces')) is list
        and all(type(v) in (int,np.int64,np.int32) for v in row['vertices'])
        and all(type(f) is tuple and all(type(v) in (int,np.int64,np.int32) for v in f) for f in row['faces'])
        and all(k in ('vertices','faces') or type(v) in leaves for k,v in row.items())):
        result=row.copy();result['vertices']=row['vertices'].copy();result['faces']=row['faces'].copy()
        return result
    return _capture_background(row,{})


@contextmanager
def _background_inputs(token,run,*,_preparation=None,_copy_verifier_caches=False,_capture_volume=False):
    from production_fast_layer import FastLayer
    from rolling_fast_engine import RollingGeometry
    if (type(_capture_volume) is not bool or type(_copy_verifier_caches) is not bool or type(token) is not LiveAcceptance or type(run) is not FastLayer or not run.rolling
        or type(run.provider) is not RollingGeometry or run.live_acceptance is not token
        or run in _background_active):raise ValueError('LiveAcceptedBackgroundInputsRequired')
    owner=object.__new__(BackgroundInputs);_background_active[run]=owner
    try:
        tx=run.tx;state=tx.current;provider=run.provider;layer=provider.layer
        started=time.perf_counter();memo={};native={k:_capture_background(state[k],memo) for k in GEOMETRY}
        phases=dict(native=time.perf_counter()-started);mark=time.perf_counter()
        def fields(obj,names):return SimpleNamespace(**{k:_capture_background(getattr(obj,k),memo) for k in names})
        numeric=dict(points=_capture_background(run.points,memo),
            topology=fields(run.topology,('cells','owners','boundary_keys')),
            outer=_capture_background(run.outer,memo),wall=_capture_background(run.wall,memo))
        phases['numeric']=time.perf_counter()-mark;mark=time.perf_counter()
        volume_data=({k:_capture_background(getattr(run.volume,k),memo) for k in ('total','boundary_total','boundary','coordinates','witness')} if _capture_volume else None)
        checked=SimpleNamespace(**numeric,tx=SimpleNamespace(current=native,generation=run.tx.generation),
            provider=SimpleNamespace(x=_capture_background(run.provider.x,memo),check=run.provider.check),
            quality=(fields(run.quality,('cells','exact','prospective','faces','masks')) if _copy_verifier_caches else run.quality),
            volume=(SimpleNamespace(**volume_data) if _capture_volume else fields(run.volume,('total','boundary_total','boundary','coordinates','witness')) if _copy_verifier_caches else run.volume),
            degree_state=(fields(run.degree_state,('points','boundary','witness','root','implementation','rows')) if _copy_verifier_caches else run.degree_state))
        if _copy_verifier_caches and hasattr(run.quality,'geometry'):
            checked.quality.geometry=_capture_background(run.quality.geometry,memo)
        # These caches are inputs to this fresh verification only. The existing
        # checker hashes them against its issued token and the detached mesh.
        # No cache field or reference is retained or consumed after that check.
        phases['verifier_inputs']=time.perf_counter()-mark
        captured=time.perf_counter()-started;started=time.perf_counter()
        token.check(checked,_preparation=_preparation)
        verified=time.perf_counter()-started
        if (run.tx is not tx or tx.current is not state or run.provider is not provider
            or run.live_acceptance is not token or tx.generation!=token.generation or provider.layer!=layer):
            raise ValueError('ChangedBackgroundParentScope')
        provider=SimpleNamespace(x=checked.provider.x,layer=run.provider.layer)
        view=_BackgroundRecord(owner,dict(numeric,rolling=True,live_acceptance=token,provider=provider))
        arrays={id(a):(a.tobytes(),a.dtype.str,a.shape) for a in (numeric['points'],provider.x)}
        _background_issued[owner]=dict(origin=run,tx=tx,state=state,provider=run.provider,
            layer=layer,token=token,native=native,numeric=numeric,view=view,reuses=0,
            arrays=arrays,capture_seconds=captured,verification_seconds=verified,preparation=_preparation,volume_data=volume_data)
        _background_issued[owner].update(capture_phases=phases,verifier_cache_copies=_copy_verifier_caches)
        # Caches are authenticated once and not read by routine background
        # acceptance. Drop private capture scratch; scientific outputs remain.
        del checked,memo
        yield owner
    finally:
        _background_issued.pop(owner,None);_background_active.pop(run,None)


@contextmanager
def _producer_background_inputs(run,*,_preparation=None,_capture_volume=False):
    """Arm the existing owner before the private final prism transaction."""
    from production_fast_layer import FastLayer
    from rolling_fast_engine import RollingGeometry
    if type(_capture_volume) is not bool or type(run) is not FastLayer or not run.rolling or type(run.provider) is not RollingGeometry or run in _background_active:
        raise ValueError('LivePrismProducerRequired')
    owner=object.__new__(BackgroundInputs);_background_active[run]=owner
    _background_pending[owner]=dict(origin=run,tx=run.tx,state=run.tx.current,
        generation=run.tx.generation,preparation=_preparation,capture_volume=_capture_volume)
    try:yield owner
    finally:
        _background_pending.pop(owner,None);_background_issued.pop(owner,None);_background_active.pop(run,None)


def _capture_producer_background(run,candidate,points,topology,quality,outer,volume):
    owner=_background_active.get(run);pending=_background_pending.get(owner) if owner is not None else None
    if pending is None:return None
    if run.tx is not pending['tx'] or run.tx.current is not pending['state'] or run.tx.generation!=pending['generation']:
        raise ValueError('ChangedPrismProducerScope')
    started=time.perf_counter();memo={};native={k:_capture_background(candidate[k],memo) for k in GEOMETRY}
    phases=dict(native=time.perf_counter()-started);mark=time.perf_counter()
    def fields(obj,names):return SimpleNamespace(**{k:_capture_background(getattr(obj,k),memo) for k in names})
    numeric=dict(points=_capture_background(points,memo),topology=fields(topology,('cells','owners','boundary_keys')),
        outer=_capture_background(outer,memo),wall=_capture_background(run.wall,memo))
    cached=fields(quality,('cells','exact','prospective','faces','masks'))
    if hasattr(quality,'geometry'):cached.geometry=_capture_background(quality.geometry,memo)
    _check_numeric_mesh(numeric['points'],native,len(run.provider.x))
    phases['numeric_and_quality']=time.perf_counter()-mark
    volume_data=({k:_capture_background(getattr(volume,k),memo) for k in ('total','boundary_total','boundary','coordinates','witness')} if pending['capture_volume'] else None)
    data=dict(native=native,numeric=numeric,quality=cached,volume_data=volume_data,provider_x=_capture_background(run.provider.x,memo),
        capture_seconds=time.perf_counter()-started,capture_phases=phases,verification_seconds=0.,
        original_candidate=candidate,original_points=points,original_topology=topology)
    pending['capture']=data
    return data


def _activate_producer_background(run,owner):
    pending=_background_pending[owner];data=pending['capture'];token=pending['accepted']
    if (run.live_acceptance is not token or run.tx is not pending['tx'] or
        run.tx.current is not data['original_candidate'] or run.tx.generation!=pending['generation']+1 or
        run.points is not data['original_points'] or run.topology is not data['original_topology']):
        raise ValueError('UncommittedPrismProducerHandoff')
    numeric=data['numeric'];provider=SimpleNamespace(x=data['provider_x'],layer=run.provider.layer)
    view=_BackgroundRecord(owner,dict(numeric,rolling=True,live_acceptance=token,provider=provider))
    arrays={id(a):(a.tobytes(),a.dtype.str,a.shape) for a in (numeric['points'],provider.x)}
    _background_issued[owner]=dict(data,origin=run,tx=run.tx,state=run.tx.current,provider=run.provider,
        layer=run.provider.layer,token=token,view=view,reuses=0,arrays=arrays,preparation=pending['preparation'],
        full_parent_verifications=0,producer_acceptance_handoff=True,verifier_cache_copies=False)
    _background_pending.pop(owner)


def checked_summary(certificate):
    if (certificate.get('passed') is not True or certificate.get('failures')
            or certificate['required_pairs'] != len(certificate['proofs'])):
        raise ValueError('UnqualifiedRollingCertificate')
    methods, bad = certificate['proofs'].summary()
    if bad or sum(methods.values()) != certificate['required_pairs']:
        raise ValueError('IncompleteRollingCertificate')
    return Counter(methods)


def face_bounds(points, faces):
    """Fresh independent bounds, in bounded triangle/quad gathers.

    Repeat the last triangle vertex, then perform the same ordered min/max.
    No producer bounds, geometry cache, coordinate conversion or rounding is
    used. Other face/input types retain the original NumPy operations.
    """
    def legacy(rows):
        return (np.asarray([points[list(f)].min(0) for f in rows]),
                np.asarray([points[list(f)].max(0) for f in rows]))
    if (type(points) is not np.ndarray or points.dtype != np.dtype('float64')
            or points.ndim != 2 or points.shape[1] != 3
            or (type(faces) is not list and type(faces) is not tuple) or not faces):
        return legacy(faces)
    lo=np.empty((len(faces),3),dtype=points.dtype);hi=np.empty_like(lo)
    for start in range(0,len(faces),4096):
        rows=faces[start:start+4096]
        if any((type(f) is not list and type(f) is not tuple) or len(f) not in (3,4)
               or any(type(v) is not int and type(v) is not np.int32
                      and type(v) is not np.int64 for v in f) for f in rows):
            lower,upper=legacy(rows)
        else:
            indices=np.asarray([tuple(f)+(f[-1],)*(4-len(f)) for f in rows])
            gathered=points[indices]
            lower=gathered.min(axis=1);upper=gathered.max(axis=1)
        lo[start:start+len(rows)]=lower;hi[start:start+len(rows)]=upper
    return lo,hi


def verify_fresh_coverage(points, faces, active, certificate, *, index_type=None):
    """Existing independent AABB replay, without a permanent proof archive."""
    from sdf_cfd_mesher_v4.certificate.aabb_index import AABBIndex
    from compact_proof_rows import CompactProofRows
    proofs = certificate['proofs']
    if isinstance(proofs, CompactProofRows):
        if len(proofs.pairs) != 2*len(proofs.codes):
            raise ValueError('IncompleteRollingCertificate')
        codes = np.frombuffer(proofs.codes, dtype=np.uint64)
        if len(codes) and np.any(codes >= len(proofs.payloads)):
            raise ValueError('IncompleteRollingCertificate')
        # checked_summary checks every referenced classification. Read the
        # existing pair buffer instead of rebuilding millions of dictionaries.
        proof_pairs = np.frombuffer(proofs.pairs, dtype=np.uint64).reshape(-1, 2)
    else:
        def legacy_pairs():
            for row in proofs:
                if row['classification'] not in (
                        'CertifiedSeparated', 'CertifiedAllowedIncidentContact'):
                    raise ValueError('RollingContinuousGeometryFailure')
                yield row['pair']
        proof_pairs = legacy_pairs()
    methods = checked_summary(certificate)
    lo, hi = face_bounds(points, faces)
    active = set(active)
    all_faces = (len(active) == len(faces) and
                 all(type(i) is int or isinstance(i, np.integer) for i in active) and
                 active == set(range(len(faces))))
    # Existing full-population path queries each pair only from its lower face.
    # Partial-active continuation retains the existing changed-vs-all path.
    index = (AABBIndex if index_type is None else index_type)(lo, hi)
    if isinstance(proofs, CompactProofRows):
        count = 0
        for expected in index.pair_blocks(None if all_faces else active):
            end = count + len(expected)
            # Exact integer equality, not a digest, tolerance or producer cache.
            # The independently enumerated population remains freshly read.
            available = min(end, len(proof_pairs))
            if not np.array_equal(proof_pairs[count:available], expected[:available - count]):
                raise ValueError('RollingCertificateCoverageMismatch')
            if end > len(proof_pairs):
                raise ValueError('IncompleteRollingCertificateCoverage')
            count = end
        if count != certificate['required_pairs']:
            raise ValueError('RollingCertificateCoverageMismatch')
        return methods
    pairs = iter(index.pairs(None if all_faces else active))
    count = 0
    for proof_pair in proof_pairs:
        expected = next(pairs, None)
        if expected is None or tuple(proof_pair) != tuple(expected):
            raise ValueError('RollingCertificateCoverageMismatch')
        count += 1
    if next(pairs, None) is not None or count != certificate['required_pairs']:
        raise ValueError('IncompleteRollingCertificateCoverage')
    return methods


def inherit(before, candidate, additions):
    """Enforce the existing append premise; no arbitrary proof reuse."""
    if before['authority'] != candidate['authority']:
        raise ValueError('RollingPredicateAuthorityChanged')
    if any(before[k] != candidate[k] for k in ('wall', 'first_layer', 'events')):
        raise ValueError('RollingSourceChanged')
    for k, v in before['vertices'].items():
        if candidate['vertices'].get(k) != v:
            raise ValueError('RollingCommittedCoordinateChanged')
    if any(candidate['cells'].get(k) != v for k, v in before['cells'].items()):
        raise ValueError('RollingCommittedCellChanged')
    new_ids = {str(c['id']) for c in additions}
    if new_ids & before['cells'].keys() or candidate['cells'].keys()-before['cells'].keys() != new_ids:
        raise ValueError('RollingNonAppendCellChange')
    for k, row in before['faces'].items():
        new = candidate['faces'].get(k)
        if new is None or new['vertices'] != row['vertices']:
            raise ValueError('RollingCommittedFaceChanged')
        if any(new['owners'].get(owner) != winding for owner, winding in row['owners'].items()):
            raise ValueError('RollingCommittedWindingChanged')
        extra = new['owners'].keys()-row['owners'].keys()
        if extra and (len(row['owners']) != 1 or not extra <= new_ids or len(new['owners']) != 2):
            raise ValueError('RollingUnsupportedIncidenceChange')


def current_quality(quality, topology, layer):
    """Only newest cells can acquire a new neighbour in uniform append."""
    from incremental_layer_validation import Quality
    ids = {i for i, c in topology.cells.items() if c['layer'] == layer}
    if (not ids or any(c['kind'] != 'prism' for c in topology.cells.values())
            or {topology.cells[i]['column'] for i in ids} !=
               {c['column'] for c in topology.cells.values()}):
        raise ValueError('RollingRequiresUniformOrdinaryFront')
    if any(not ids <= getattr(quality, name).keys()
           for name in ('cells', 'exact', 'prospective')):
        raise ValueError('MissingRollingCurrentQuality')
    result = Quality({i: quality.cells[i] for i in ids},
        {k: v for k, v in quality.faces.items() if ids & set(v['owners'])},
        {i: quality.exact[i] for i in ids}, {i: quality.prospective[i] for i in ids})
    result.masks = {i: quality.masks[i] for i in ids}
    if hasattr(quality,'geometry'):result.geometry=quality.geometry
    return result


def prepare(run, before, candidate, points, topology, quality, volume, degree, outer, additions, fresh, faces, active):
    owner=_background_active.get(run)
    pending=_background_pending.get(owner) if owner is not None else None
    preparation=None if pending is None else pending['preparation']
    run.live_acceptance.check(run,_preparation=preparation)
    inherit(before, candidate, additions)
    methods = verify_fresh_coverage(points, faces, active, fresh)
    layer = max(c['layer'] for c in additions)
    reduced = current_quality(quality, topology, layer)
    # These tables are archive/correction representations, not predicate inputs.
    candidate['quality'] = {}; candidate['certificate'] = {}
    candidate['atlas'] = {}; candidate['forecast'] = {}
    total = Counter(dict(run.live_acceptance.methods)); total.update(methods)
    producer_issue_started=time.perf_counter()
    captured=_capture_producer_background(run,candidate,points,topology,reduced,outer,volume)
    native=candidate if captured is None else captured['native']
    numeric=dict(points=points,topology=topology,outer=outer,wall=run.wall) if captured is None else captured['numeric']
    preparation=None if captured is None else _background_pending[_background_active[run]]['preparation']
    accepted = LiveAcceptance(mesh_identity(native,_preparation=preparation), numeric_identity(**numeric),
        cache_identity(reduced if captured is None else captured['quality'],volume if captured is None or captured['volume_data'] is None else SimpleNamespace(**captured['volume_data']),degree), identity(native['authority']),
        run.live_acceptance.count + fresh['required_pairs'], tuple(sorted(total.items())),
        run.tx.generation + 1)
    if captured is not None:
        captured['producer_issue_seconds']=time.perf_counter()-producer_issue_started
        _background_pending[_background_active[run]]['accepted']=accepted
    return reduced, accepted


def export_mesh(run, path, *, _handoff=None):
    """Mesh-only output. No pickle, parent path, proof closure or restart claim."""
    if _handoff is None:
        run.live_acceptance.check(run)
        points=run.points;topology=run.topology;outer=run.outer;wall=run.wall;token=run.live_acceptance
    else:
        if type(_handoff) is not BackgroundInputs:raise ValueError('LiveAcceptedBackgroundInputsRequired')
        _handoff.require_origin(run);_handoff.require(_handoff.view)
        # Read only private, already-authenticated snapshot values. No numeric
        # cell or array alias leaves this export; the origin may stay mutable.
        data=_handoff._open();numeric=data['numeric'];token=data['token']
        points=numeric['points'];topology=numeric['topology'];outer=numeric['outer'];wall=numeric['wall']
    path = Path(path)
    if path.exists(): raise ValueError('PreserveExistingMeshExport')
    cells = [topology.cells[k] for k in sorted(topology.cells)]
    np.savez(path, points=points, cell_vertices=np.asarray([c['vertices'] for c in cells], dtype='<i8'),
        cell_ids=np.asarray([c['id'] for c in cells], dtype='<i8'),
        source_columns=np.asarray([c['column'] for c in cells], dtype='<i8'),
        layers=np.asarray([c['layer'] for c in cells], dtype='<i8'),
        outer=np.asarray(outer, dtype='<i8'), wall=np.asarray(wall, dtype='<i8'))
    return dict(schema='V4FastProdMeshOnly/1', restart_supported=False,
        published_scientific_authority=False, mesh=token.mesh,
        generation=token.generation, cells=len(cells), coordinate_hash=array_hash(points))

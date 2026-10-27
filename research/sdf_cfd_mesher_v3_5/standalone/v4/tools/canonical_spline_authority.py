"""Live storage/I/O binding for the unchanged, hash-locked SF2 SmoothAuthority.

No shape, projector, derivative, curvature or seed algorithm is defined here.
The historical DLL is immutable and single-profile per process; incompatible
profiles require a fresh process, never a silent reload of global DLL state.
"""
import ast
import copy
import ctypes
import hashlib
import json
import math
from pathlib import Path
import struct
import subprocess
import sys
import threading
import time
import types
from collections import Counter
from functools import lru_cache
from contextlib import contextmanager, ExitStack
import weakref
import numpy as np
from sdf_cfd_mesher_v4.sdf.canonical import GeometryFailure

V4 = Path(__file__).resolve().parents[1]
V3 = V4.parent / 'sdf_cfd_mesher_v3'
SOURCE = V4 / 'src/sdf_cfd_mesher_v4/sdf/_frozen/current_smooth_authority.py'
from owned_runtime import BIN
DLL = BIN / 'spline_bridge_optimized.dll'
LOCKS = {SOURCE: '2278094b5eb483f9b49d90754725a0df12cd96c7716cfa9320aa6d9318208dfb',
         DLL: '35078384ed103c442299a7f52fc47435f7f1012cf89da7af177785ff4a24d02b'}
_lock = threading.RLock()
_native = None
_profile = None
_verified_exports = weakref.WeakKeyDictionary()


@contextmanager
def verified_export_scope(authority):
    """Reuse the native immutable spline export under existing file/host custody.

    Mutable Python fields and the live native descriptor are still checked.
    Only export replay and hashing of held dependency files are reused.
    """
    if type(authority) is not LiveSplineAuthority or authority in _verified_exports:
        raise ValueError('VerifiedSplineExportScopeRequired')
    from sdf_cfd_mesher_v4.sdf.canonical import CanonicalGeometry,CanonicalKernel
    if type(authority.geometry) is not CanonicalGeometry or type(authority.geometry.owner) is not CanonicalKernel:
        raise ValueError('LiveCanonicalSplineOwnerRequired')
    from immutable_state_access import guard
    with ExitStack() as held:
        geometry=authority.geometry;owner=geometry.owner
        held.enter_context(owner.lock)
        data=dict(geometry=geometry,owner=owner,handle=geometry.handle,
            description=copy.deepcopy(geometry.description),packet=copy.deepcopy(authority.spline_packet),locks=dict(LOCKS))
        for path,pin in data['locks'].items():
            held.enter_context(guard(path))
            if sha(path)!=pin:fail('SplineDependencyMismatch',str(path),authority.geometry_identity)
        # Authenticate the detached packet, then check the live mutable binding.
        # A mutation between an original check and capture cannot become trust.
        validate_export(geometry,data['packet'])
        _verified_exports[authority]=data
        try:
            authority.verify()
            yield authority
        finally:_verified_exports.pop(authority,None)


def sha(path): return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def digest(value): return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()).hexdigest()
def fail(code, detail, identity=None): raise GeometryFailure(code, 'smooth_spline_binding', detail, identity)


def validate_export(geometry, packet):
    if geometry.owner.request(op='describe', handle=geometry.handle) != geometry.description:
        fail('GeometryDescriptionMismatch', 'live descriptor differs', geometry.identity)
    if packet != geometry.stored_spline():
        fail('SplineExportMismatch', 'export is not the owned current object', geometry.identity)
    data = packet['data']
    if data['status'] != 'available' or data.get('representation') != 'StoredSmoothSplineSectionLoft/1':
        fail('UnsupportedStoredSpline', data.get('reason'), geometry.identity)
    if packet['metres_per_model_unit'] != 1.0:
        fail('SplineUnitPremiseFailed', 'existing SF2 wrapper requires 1 m/model unit; no rescaling inferred', geometry.identity)
    sections = data['sections']
    if len(sections) < 2:
        fail('SplineUniformExtrusionPremiseFailed', 'at least two stations required', geometry.identity)
    first = sections[0]
    # The retained trailing-sheet projector fixes X and uses normal +X.
    # A slanted cambered TE cannot satisfy that premise without another binding.
    spans = first['spline']['spans']
    upper, lower = spans[0]['control'][0], spans[-1]['control'][3]
    if upper[0] != lower[0] or upper[1] <= lower[1]:
        fail('SplineTrailingPlanePremiseFailed', 'existing SF2 trailing branch requires common endpoint X and positive finite thickness', geometry.identity)
    for i, s in enumerate(sections):
        if (s['source_section_index'] != i or s['chord'] != 1.0 or s['twist'] != 0.0 or
                s['position'][0] != 0.0 or s['position'][2] != 0.0 or
                s['control_bits'] != first['control_bits'] or
                (i and s['position'][1] <= sections[i-1]['position'][1])):
            fail('SplineUniformExtrusionPremiseFailed', f'station {i}: unit chord, identical coefficients, ordered y, zero twist/x/z required', geometry.identity)
        for value, bits in zip([s['chord'], s['twist'], *s['position']],
                               [s['chord_bits'], s['twist_bits'], *s['position_bits']]):
            if struct.unpack('<I', struct.pack('<f', value))[0] != bits:
                fail('StoredSplineBitsMismatch', 'placement', geometry.identity)
        for span, raw in zip(s['spline']['spans'], s['control_bits']):
            for p, bits in zip(span['control'], raw):
                if any(struct.unpack('<Q', struct.pack('<d', x))[0] != b for x,b in zip(p,bits)):
                    fail('StoredSplineBitsMismatch', 'coefficient', geometry.identity)
    inv = data['inventory']
    if not inv or inv['capabilities']['support_status'] != 'native':
        fail('SplineTopologyUnsupported', 'canonical smooth-extrusion inventory required', geometry.identity)
    # Existing SmoothAuthority consumes this exact physical-role inventory.
    # These counts describe its representation class, not a benchmark/hash.
    if tuple(len(inv[k]) for k in ('sectors','features','junctions')) != (4,6,4):
        fail('SplineTopologyUnsupported', 'SF2 physical topology required', geometry.identity)
    return {'sections':copy.deepcopy(sections), 'inventory':copy.deepcopy(inv)}


def _library(payload):
    global _native, _profile
    for path, expected in LOCKS.items():
        if sha(path) != expected: fail('SplineDependencyMismatch', str(path))
    key = digest(payload)
    with _lock:
        if _native is None:
            lib = ctypes.CDLL(str(DLL))
            lib.spline_load.argtypes = [ctypes.c_char_p, ctypes.c_size_t]
            lib.spline_load.restype = ctypes.c_int
            lib.spline_query.argtypes = [ctypes.c_double, ctypes.c_double, ctypes.POINTER(ctypes.c_double)]
            lib.spline_query.restype = ctypes.c_int
            raw = json.dumps(payload, separators=(',', ':'), allow_nan=False).encode()
            code = lib.spline_load(raw, len(raw))
            if code: fail('SplineNativeLoadFailed', code)
            _native, _profile = lib, key
        elif _profile != key:
            fail('SplineNativeProfileAlreadyBound', 'existing DLL is single-profile; use a fresh process')
    # Keep the original class initializer and callable ABI. Repeated identical
    # loads validate data but cannot call OnceLock::set a second time.
    def load(raw, length):
        return 0 if digest(json.loads(raw[:length])) == _profile else 3
    def query(x, z, out):
        if not math.isfinite(x) or not math.isfinite(z): return 2
        with _lock: return _native.spline_query(x, z, out)
    return types.SimpleNamespace(spline_load=load, spline_query=query)


def existing_authority(source, *, output=None, maximum_size=None):
    """Same unchanged-class AST mechanism used by the polygon binding.

    Avoid importing study.py (which creates V3 directories and sets prism data).
    Inject only live input, dependency and output/resource globals.
    """
    from historical_capsule_surface import Authority, SeedVertex, SeedFace, wsl, save
    from procedural_authorities import SeedEdge, tangent_metric, curvature_size
    library = _library(source['sections'][0]['spline'])
    def linux(command, tag, seconds):
        if output is None: fail('SeedOutputRequired', 'explicit execution output required')
        # Preserve the frozen chart call and algorithms, relocate its binary.
        command = [str(x) for x in command]
        old = wsl(V3/'singular_sector_charts/sector_cdt')
        command = [wsl(BIN/'sector_cdt') if x == old else x for x in command]
        return subprocess.run(['wsl','-e',*command], capture_output=True, text=True, timeout=seconds)
    def read(path):
        if Path(path).name != 'WING_AUTHORITY_live.json': fail('UnexpectedSplineInput', str(path))
        return copy.deepcopy(source)
    node = next(n for n in ast.parse(SOURCE.read_text()).body if isinstance(n,ast.ClassDef) and n.name=='SmoothAuthority')
    module = types.ModuleType('_v4_existing_smooth_authority'); module.__file__ = str(SOURCE)
    ns = module.__dict__
    ns.update(np=np, math=math, json=json, Counter=Counter, lru_cache=lru_cache, Authority=Authority,
              SeedVertex=SeedVertex, SeedFace=SeedFace, SeedEdge=SeedEdge,
              tangent_metric=tangent_metric, curvature_size=curvature_size,
              HMAX=maximum_size, HERE=Path(output) if output else Path('.'), V3=V3,
              read=read, write=save, wsl=wsl, linux=linux,
              ctypes=types.SimpleNamespace(CDLL=lambda _:library, c_char_p=ctypes.c_char_p,
                  c_size_t=ctypes.c_size_t,c_double=ctypes.c_double,POINTER=ctypes.POINTER))
    sys.modules[module.__name__] = module
    exec(compile(ast.Module(body=[node],type_ignores=[]),str(SOURCE),'exec'),ns)
    try:
        return ns['SmoothAuthority']('live')
    except (AssertionError, ValueError) as error:
        fail('SplineAuthorityPremiseFailed', str(error) or 'unchanged SF2 constructor assertion')


class LiveSplineAuthority:
    # Do not silently inherit the historical curvature_size/HMAX curve policy.
    metric = None
    def __init__(self, geometry, packet):
        self.geometry, self.geometry_identity = geometry, geometry.identity
        self.spline_packet = copy.deepcopy(packet)
        self.source = validate_export(geometry, packet)
        self.unit = existing_authority(self.source)
        self._span = self.unit.span.copy()
        self._kbound = self.unit.kbound.copy()
        for name in ('sheet_names','curve_names','corner_names','corner_points'):
            setattr(self, name, copy.deepcopy(getattr(self.unit,name)))
        self.reference_identity = digest({n:getattr(self,n) for n in ('sheet_names','curve_names','corner_names')})
        self.premises = dict(representation=packet['data']['representation'],
            trailing_plane='equal endpoint X, upper Z greater than lower Z; verified from stored controls',
            distance_source='stored cubic coefficients; unchanged SF2 skin-query DLL',
            source_identity=packet['spline_identity'],
            curvature='unchanged derivative-control-hull lower speed / upper derivative norms',
            curvature_max_per_m=float(self.unit.kbound.max()),
            span_count=self.unit.N, topology_counts=[4,6,4],
            qualification='branch operations connected; no continuous surface qualification or case limits supplied',
            finite_domains='existing SF2 source inventory and labelled charts; future submitted mesh still requires qualification',
            source_dependencies={str(p):h for p,h in LOCKS.items()})

    def verify(self):
        held=_verified_exports.get(self)
        if held is None:
            validate_export(self.geometry, self.spline_packet)
            for path,h in LOCKS.items():
                if sha(path)!=h: fail('SplineDependencyMismatch',str(path),self.geometry_identity)
        else:
            from accepted_trajectory_contract import _same_integrity_input
            if (self.geometry is not held['geometry'] or self.geometry.owner is not held['owner'] or
                self.geometry.handle!=held['handle'] or not _same_integrity_input(self.geometry.description,held['description']) or
                not _same_integrity_input(self.spline_packet,held['packet']) or LOCKS!=held['locks']):
                fail('SplineAuthorityStateMismatch','changed protected export binding',self.geometry_identity)
            if not _same_integrity_input(self.geometry.owner.request(op='describe',handle=self.geometry.handle),held['description']):
                fail('GeometryDescriptionMismatch','live descriptor differs',self.geometry_identity)
        if (self.source != self.unit.source or self.source != {'sections':self.spline_packet['data']['sections'], 'inventory':self.spline_packet['data']['inventory']} or
                not np.array_equal(self.unit.ctrl,np.array([x['control'] for x in self.source['sections'][0]['spline']['spans']])) or
                not np.array_equal(self.unit.span,self._span) or not np.array_equal(self.unit.kbound,self._kbound)):
            fail('SplineAuthorityStateMismatch','live stored coefficients changed',self.geometry_identity)
        for n in ('sheet_names','curve_names','corner_names'):
            if getattr(self,n)!=getattr(self.unit,n):fail('ReferenceBindingMismatch',n,self.geometry_identity)
        if digest({n:getattr(self,n) for n in ('sheet_names','curve_names','corner_names')})!=self.reference_identity:
            fail('ReferenceBindingMismatch','reference identity',self.geometry_identity)

    def _point(self, ref, point, names):
        if ref not in names: fail('UnknownSplineReference',ref,self.geometry_identity)
        p=np.asarray(point,dtype=float)
        if p.shape!=(3,) or not np.isfinite(p).all():fail('InvalidSplinePoint',str(point),self.geometry_identity)
        return p
    def scalar_value(self,points): return np.asarray(self.geometry.query(np.asarray(points).tolist()))
    def project_sheet(self,ref,p):return self.unit.project_sheet(ref,self._point(ref,p,self.sheet_names))
    def project_curve(self,ref,p):return self.unit.project_curve(ref,self._point(ref,p,self.curve_names))
    def exact_normal(self,ref,p):return self.unit.exact_normal(ref,self._point(ref,p,self.sheet_names))
    def implicit_value(self,ref,p):return self.unit.implicit_value(ref,self._point(ref,p,self.sheet_names))
    def feature_residual(self,ref,p):return self.unit.feature_residual(ref,self._point(ref,p,self.curve_names))
    def gradient_hessian(self,ref,p):return self.unit.gradient_hessian(ref,self._point(ref,p,self.sheet_names))
    def curvature_upper(self,ref,p):
        if ref not in self.sheet_names:fail('UnknownSplineReference',ref,self.geometry_identity)
        return self.unit.curvature_upper(ref,p)
    def _normal_map_principal_data(self,ref,p):return self.unit._normal_map_principal_data(ref,self._point(ref,p,self.sheet_names))
    def competing_component_sheet_refs(self,ref):return self.unit.competing_component_sheet_refs(ref)
    def labelled_seed(self,authority,policy,output):
        if authority is not self:fail('GeometryIdentityMismatch','seed authority',self.geometry_identity)
        self.verify()
        if policy is None:fail('SizingPolicyRequired','no historical HMAX selected',self.geometry_identity)
        policy.validate()
        start=time.perf_counter()
        a=existing_authority(self.source,output=Path(output)/'labelled_spline_seed',maximum_size=policy.maximum_size_m)
        vertices,faces,edges=a.seed()
        return dict(coordinates=np.array([v.point for v in vertices]),triangles=np.array([f.vertices for f in faces]),
            vertices=vertices,faces=faces,edges=edges,timing={'seconds':time.perf_counter()-start},
            seed_size_m=policy.maximum_size_m,seed_route='ExistingCGALSplineParameterCharts')

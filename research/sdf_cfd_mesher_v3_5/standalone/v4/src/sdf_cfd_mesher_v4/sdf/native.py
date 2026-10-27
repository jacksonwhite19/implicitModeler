"""Read-only authoritative SmoothSpline queries, normalized normals, SI units."""
from dataclasses import dataclass
import ctypes,json,time
import numpy as np
from ..common.authority import ROOT,reference,file_hash,identity

@dataclass
class SdfValues:
    phi: np.ndarray
    gradient: np.ndarray
    normal: np.ndarray
    gradient_norm: np.ndarray
    source_error_bound: np.ndarray
    section_curvature: np.ndarray
    boundary_normal_selection: np.ndarray
    branch_tie: np.ndarray

class NativeSdf:
    _library=None
    _geometry=None
    def __init__(self):
        path,geometry=reference('common geometry');j=json.loads(path.read_text());sections=j['sections'];s=sections[0]
        if not all(x['spline']==s['spline'] and x['chord']==1 and x['twist']==0 and x['position'][0]==0 and x['position'][2]==0 for x in sections):raise ValueError('AdapterRequiresBoundUniformExtrusion')
        dll=ROOT/'native/target/release/v4_sdf_bridge.dll'
        if self.__class__._library is None:
            lib=ctypes.CDLL(str(dll));lib.v4_load.argtypes=[ctypes.c_char_p,ctypes.c_size_t,ctypes.c_double,ctypes.c_double];lib.v4_load.restype=ctypes.c_int
            lib.v4_query.argtypes=[ctypes.c_void_p,ctypes.c_size_t,ctypes.c_void_p];lib.v4_query.restype=ctypes.c_int
            payload=json.dumps(s['spline'],separators=(',',':')).encode();ys=[x['position'][1] for x in sections]
            code=lib.v4_load(payload,len(payload),min(ys),max(ys))
            if code:raise RuntimeError('NativeSdfLoad:'+str(code))
            self.__class__._library=lib;self.__class__._geometry=geometry
        if self._geometry!=geometry:raise ValueError('NativeGeometryAuthorityChanged')
        self.lib=self._library;self.geometry=geometry;self.seconds=0.;self.calls=0;self.vertex_queries=0
        self.source_identity=identity(dict(native_source=file_hash(ROOT/'native/frozen/cached_spline.rs'),adapter=file_hash(ROOT/'native/src/lib.rs'),python_adapter=file_hash(__file__),binary=file_hash(dll),geometry=geometry))
    def clear_memo(self):self.lib.v4_clear_memo()
    def evaluate(self,points):
        x=np.ascontiguousarray(points,dtype='<f8')
        if x.ndim!=2 or x.shape[1]!=3 or not np.isfinite(x).all():raise ValueError('NonfiniteCoordinateOrShape')
        out=np.empty((len(x),10));t=time.perf_counter();code=self.lib.v4_query(x.ctypes.data,len(x),out.ctypes.data);self.seconds+=time.perf_counter()-t;self.calls+=1;self.vertex_queries+=len(x)
        if code:raise RuntimeError('SdfEvaluationFailure:'+str(code))
        g=out[:,1:4];norm=np.linalg.norm(g,axis=1)
        if not np.isfinite(out).all():raise ValueError('NonfiniteGradientOrSdf')
        if np.any(norm==0):raise ValueError('ZeroRequiredGradient')
        return SdfValues(out[:,0],g,g/norm[:,None],norm,out[:,4],out[:,5],out[:,7].astype(bool),out[:,8].astype(bool))

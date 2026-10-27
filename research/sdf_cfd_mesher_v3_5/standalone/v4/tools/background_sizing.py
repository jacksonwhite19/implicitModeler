"""Staged ordinary Gmsh sizing leaf; existing tree and policy stay unchanged.

Linux NumPy 1.26.4 arithmetic only. Windows and unsupported warning/index/
storage paths use the existing Python owner. No checkpoint or geometry adapter.
"""
import stage_observations as obs
from pathlib import Path
import ctypes,json,os,platform
import numpy as np
from optimized_core_sizing import ExactCachedSizingField
from capsule_sizing_control import digest

from owned_runtime import V4
import build_provenance as provenance
HERE=V4/'runtime/source/background'


@obs.observed('sizing.record','source_lock')
def record(directory=None):
    directory=HERE if directory is None else Path(directory);source=directory/'sizing_values.cpp';pin=digest(source)
    path=directory/('sizing_'+pin+'.so');build=path.with_name(path.name+'.BUILD.json')
    metadata=provenance.receipt(build)
    return _admit(metadata,source,pin,path,build)


@obs.observed('sizing._admit','source_lock')
def _admit(metadata,source,pin,path,build):
    command=['/usr/bin/c++','-std=c++17','-shared','-fPIC','-O3','-fno-fast-math',
             '-fno-associative-math','-ffp-contract=off',provenance.original(source),'-o',provenance.original(path)]
    if (metadata.get('exit_code')!=0 or metadata.get('sources')!={source.name:pin} or
            metadata.get('build_kind')!='cxx-scalar-sizing' or metadata.get('command')!=command):
        raise ValueError('UnreviewedSizingLeafBuild')
    binary=digest(path)
    if metadata.get('binary_sha256')!=binary:raise ValueError('ChangedSizingLeafBinary')
    runtime=arithmetic_runtime()
    if metadata.get('arithmetic_runtime')!=runtime:raise ValueError('ChangedSizingArithmeticRuntime')
    return dict(source_sha256=pin,binary_sha256=binary,build_sha256=digest(build),
                **runtime),path


@obs.observed('sizing.arithmetic_runtime','source_lock')
def arithmetic_runtime():
    import numpy.core._multiarray_umath as arithmetic
    return dict(numpy=np.__version__,numpy_arithmetic_sha256=digest(arithmetic.__file__),
                machine=platform.machine())


@obs.observed('sizing.selection_record','source_lock')
def selection_record(directory=None):
    from optimized_core_sizing import __file__ as owner
    if os.name=='nt' or np.__version__!='1.26.4' or platform.machine()!='x86_64':
        return dict(implementation='ExistingPythonSizingLeaf/1',owner_sha256=digest(owner))
    return dict(implementation='ExistingNumPy1264SSE2LeafCompiled/1',owner_sha256=digest(owner),
                wrapper_sha256=digest(__file__),**record(directory)[0])


class Field(ExactCachedSizingField):
    def __init__(self,*args,directory=None,**kwargs):
        super().__init__(*args,**kwargs)
        self.dll=None;self.library_record=None
        if os.name=='nt' or np.__version__!='1.26.4' or platform.machine()!='x86_64':return
        # Runtime check corroborates the source-backed SSE2 reduction premise.
        probe=np.array([[2.**53,-2.**53,1.],[-0.,-0.,-0.]])
        direct=np.einsum('ij,ij->i',probe,np.ones_like(probe))
        expected=0.+(((probe[:,0]+0.)+probe[:,2])+((probe[:,1]+0.)+0.))
        if direct.tobytes()!=expected.tobytes():raise ValueError('UnsupportedSizingReductionOrder')
        evidence,path=record(directory)
        library=ctypes.CDLL(str(path));p=ctypes.c_void_p;s=ctypes.c_size_t
        try:
            library.values.argtypes=[p,p,s,s,p,p,p,p,p,p,p,p,p,p,ctypes.c_double,ctypes.c_double,p]
            library.values.restype=ctypes.c_uint8
        except BaseException:
            del library
            raise
        self.dll=library;self.library_record=evidence;self._closed=False

    def values(self,p,ids):
        if getattr(self,'_closed',False):raise ValueError('ClosedSizingField')
        # Inspect structure before len(), shape or native pointer access.
        # Unsupported storage retains the old indexing/error-order owner.
        if (type(self.seeds) is not np.ndarray or self.seeds.ndim!=1 or
                type(self.edges) is not tuple or type(self.edge2) is not tuple or
                type(self.g) is not float or type(self.h_max) is not float):
            return super().values(p,ids)
        if (self.dll is None or np.geterr()!=dict(divide='warn',over='warn',under='ignore',invalid='warn') or
                type(p) is not np.ndarray or p.dtype!=np.dtype('float64') or p.shape!=(3,) or
                not p.flags.aligned or
                len(self.edges)!=3 or len(self.edge2)!=3 or
                any(type(a) is not np.ndarray or a.dtype!=np.dtype('float64') or
                    a.shape!=shape or not a.flags.c_contiguous or not a.flags.aligned
                    for a,shape in ((self.triangles,(len(self.seeds),3,3)),(self.seeds,(len(self.seeds),)),
                        (self.normals,(len(self.seeds),3)),(self.normal2,(len(self.seeds),)),
                        *((a,(len(self.seeds),3)) for a in self.edges),
                        *((a,(len(self.seeds),)) for a in self.edge2)))):
            return super().values(p,ids)
        raw=np.asarray(ids)
        if raw.ndim!=1 or raw.dtype.kind not in 'iu' or np.any(raw<0) or np.any(raw>=len(self.seeds)):
            return super().values(p,ids)
        selected=np.require(raw,dtype=np.uint64,requirements=['C','A']);position=np.ascontiguousarray(p)
        out=np.empty(len(selected))
        status=self.dll.values(position.ctypes.data,selected.ctypes.data,len(selected),len(self.seeds),
            self.triangles.ctypes.data,self.normals.ctypes.data,self.normal2.ctypes.data,
            *(a.ctypes.data for a in self.edges),*(a.ctypes.data for a in self.edge2),self.seeds.ctypes.data,
            self.g,self.h_max,out.ctypes.data)
        if status:return super().values(p,ids)
        return out

    def close(self):self._closed=True;self.dll=None

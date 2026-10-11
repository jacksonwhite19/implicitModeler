"""Actual existing native wrapper with an isolated execution-only candidate DLL."""
from pathlib import Path
from contextlib import ExitStack
import ctypes, hashlib
import numpy as np
from optimized_mixed_certificate import Native as Existing,Bytes,Buffer
from path_preserving_deduplication import file_hash
from background_native_guard import held_library
HERE=Path(__file__).resolve().parents[1]/'runtime/source/background'
class Native(Existing):
    def pairs(self,q,active,workers):
        q=np.require(q,dtype=np.float64,requirements=['C','A'])
        return super().pairs(q,active,workers)
    def screen(self,q,labels,pairs,workers):
        q=np.require(q,dtype=np.float64,requirements=['C','A'])
        labels=np.require(labels,dtype=np.uint64,requirements=['C','A'])
        # Original validation still receives the original pairs, before any
        # unsigned conversion. Only a validated buffer is forced aligned.
        raw=np.asarray(pairs)
        if raw.size and (np.any(raw<0) or np.any(raw>=len(q)) or np.any(raw!=raw.astype(np.uint32))):
            return super().screen(q,labels,pairs,workers)
        pairs=np.require(raw,dtype=np.uint32,requirements=['C','A'])
        return super().screen(q,labels,pairs,workers)
    def __init__(self,*,directory=None):
        super().__init__()
        directory=HERE if directory is None else Path(directory)
        source=directory/'efficient_candidate.rs';wrapper=directory/'screen_candidate.rs'
        digest=hashlib.sha256(wrapper.read_bytes()+source.read_bytes()).hexdigest()
        self.source_hash=hashlib.sha256(source.read_bytes()).hexdigest()
        self._held=ExitStack()
        try:
            self.dll=self._held.enter_context(held_library(directory/('screen_'+digest+'.dll'),
                {p.name:file_hash(p) for p in (wrapper,source)}))
            p=ctypes.c_void_p;s=ctypes.c_size_t
            self.dll.forest_new.argtypes=[p,s,s];self.dll.forest_new.restype=p
            self.dll.forest_free.argtypes=[p];self.dll.forest_free.restype=None
            self.dll.active_pairs.argtypes=[p,p,s];self.dll.active_pairs.restype=Buffer
            self.dll.buffer_free.argtypes=[Buffer];self.dll.buffer_free.restype=None
            self.dll.encode_rows.argtypes=[p,p,s,p,s,p,s,p,s];self.dll.encode_rows.restype=Bytes
            self.dll.bytes_free.argtypes=[Bytes];self.dll.bytes_free.restype=None
            self.dll.parallel_screen.argtypes=[ctypes.c_void_p,ctypes.c_void_p,ctypes.c_size_t,
                ctypes.c_void_p,ctypes.c_size_t,ctypes.c_void_p,ctypes.c_size_t]
            self.dll.parallel_screen.restype=None
            self.dll.ieee_environment.argtypes=[];self.dll.ieee_environment.restype=ctypes.c_uint8
            if not self.dll.ieee_environment():raise ValueError('GradualUnderflowRequired')
        except BaseException:
            self._held.close()
            raise
    def close(self):
        self._held.close()
    def __del__(self):
        if hasattr(self,'_held'):self.close()

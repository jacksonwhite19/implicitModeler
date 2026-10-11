"""Private execution identity for the frozen kernel plus root-form adapter.

The original kernel, vendor hashes, and historical cache stay untouched.
Only exact_algebraic entry-value normalization differs in this namespace.
"""
import builtins,hashlib,sys,types
from pathlib import Path
from . import kernel as frozen
from . import scaled_root_adapter as adapter

VERSION=hashlib.sha256((frozen.VERSION+hashlib.sha256(Path(adapter.__file__).read_bytes()).hexdigest()+hashlib.sha256(Path(__file__).read_bytes()).hexdigest()).encode()).hexdigest()
_modules={}

def _import(name,globals=None,locals=None,fromlist=(),level=0):
    if level==0 and name in frozen._sources:return module(name)
    return builtins.__import__(name,globals,locals,fromlist,level)

def module(name):
    if name in _modules:return _modules[name]
    qualified='_v4_scaled_root_'+VERSION+'_'+name
    m=types.ModuleType(qualified);m.__file__=str(frozen.ROOT/'vendor'/(name+'.py'))
    m.__builtins__=dict(vars(builtins),__import__=_import)
    _modules[name]=m;sys.modules[qualified]=m
    exec(compile(frozen._sources[name],m.__file__,'exec'),m.__dict__)
    if name=='exact_algebraic':adapter.install(m)
    if name=='noncoincident_rank3_certificate':
        original=m.cert
        def cert(a,b):
            # Proof representatives must depend on this pair, not on which
            # other pairs or cache evictions preceded it. Refinement itself
            # stays in the frozen exact arithmetic functions.
            module('exact_algebraic')._intervals.clear()
            return original(a,b)
        m.cert=cert
    return m

certificate=module('bilinear_side_certificate')
enumerator=module('unordered_pair_enumerator')
Patch=certificate.Patch

def clear_kernel_caches():
    for m in tuple(_modules.values()):
        for v in tuple(vars(m).values()):
            if hasattr(v,'cache_clear'):v.cache_clear()

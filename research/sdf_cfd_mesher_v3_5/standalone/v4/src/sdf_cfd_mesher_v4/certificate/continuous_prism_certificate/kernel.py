"""Load the byte-identical qualified modules in a private import namespace.

Vendor source and mathematical predicates remain unchanged. A separately
hash-locked bridge normalizes rational affine real-root representations.
The mathematical cache identity and existing valid proofs remain unchanged.
"""
import builtins
import hashlib
import json
import sys
import types
from pathlib import Path

ROOT = Path(__file__).resolve().parent
AUTHORITY = json.loads((ROOT / 'MATHEMATICAL_AUTHORITY.json').read_text())
VERSION = hashlib.sha256(json.dumps(AUTHORITY, sort_keys=True, separators=(',', ':')).encode()).hexdigest()
ROOT_REPRESENTATION = json.loads((ROOT / 'ROOT_REPRESENTATION_AUTHORITY.json').read_text())
if hashlib.sha256((ROOT / 'root_representation.py').read_bytes()).hexdigest()!=ROOT_REPRESENTATION['sha256']:
    raise RuntimeError('RootRepresentationSourceMismatch')
_sources = {}
for filename, expected in AUTHORITY['files'].items():
    source = (ROOT / 'vendor' / filename).read_bytes()
    if hashlib.sha256(source).hexdigest() != expected:
        raise RuntimeError('QualifiedCertificateSourceMismatch:' + filename)
    _sources[filename[:-3]] = source
_modules = {}

def _import(name, globals=None, locals=None, fromlist=(), level=0):
    if level == 0 and name in _sources:
        return module(name)
    return builtins.__import__(name, globals, locals, fromlist, level)

def module(name):
    if name in _modules:
        return _modules[name]
    qualified_name = '_complete_bilinear_' + VERSION + '_' + name
    m = types.ModuleType(qualified_name)
    m.__file__ = str(ROOT / 'vendor' / (name + '.py'))
    m.__builtins__ = dict(vars(builtins), __import__=_import)
    _modules[name] = m
    sys.modules[qualified_name] = m
    exec(compile(_sources[name], m.__file__, 'exec'), m.__dict__)
    if name == 'exact_algebraic':
        from .root_representation import install
        install(m)
    if name == 'noncoincident_rank3_certificate':
        original=m.cert
        def cert(a,b):
            module('exact_algebraic')._intervals.clear()
            return original(a,b)
        m.cert=cert
    return m

certificate = module('bilinear_side_certificate')
enumerator = module('unordered_pair_enumerator')
Patch = certificate.Patch

def clear_kernel_caches():
    """Qualification helper; clearing memoization cannot alter its mathematics."""
    for m in tuple(_modules.values()):
        for value in tuple(vars(m).values()):
            if hasattr(value, 'cache_clear'):
                value.cache_clear()

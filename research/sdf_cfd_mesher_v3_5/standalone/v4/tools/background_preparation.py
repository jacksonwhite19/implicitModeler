"""Guarded equivalent native bookkeeping for the joined background only.

Keep the original encoder, hash, predicates, proof format and live reads.
This owner exposes no case-supplied callback or imported acceptance.
Unsupported platforms use the existing Python/NumPy owners.
"""
from contextlib import contextmanager, ExitStack
from pathlib import Path
import sys, sysconfig, platform, json, importlib.util, os, re, weakref
import numpy as np
from owned_runtime import V4
import build_provenance as provenance
from immutable_state_access import guard
from path_preserving_deduplication import file_hash


def validate(data, source, pin, target):
    include = Path(sysconfig.get_path('include'))
    library = Path(sys.base_prefix)/'libs/python312.lib'
    compiler = Path(data['command'][0])
    expected = [str(compiler), '/nologo', '/LD', '/TC', '/std:c17', '/O2', '/fp:strict',
        '/MD', '/showIncludes', '/I'+str(include), '/Fo'+provenance.original(source.parent/'preparation.obj'),
        provenance.original(source), '/link', '/VERBOSE:LIB', str(library), '/OUT:'+provenance.original(target)]
    if (data.get('schema') != 'V4BackgroundPreparationBuild/1'
            or data.get('build_kind') != 'c17-native-preparation'
            or data['command'] != expected or compiler.name.lower() != 'cl.exe'
            or set(data['environment']) != {'SYSTEMROOT','WINDIR','TEMP','TMP','INCLUDE','LIB','PATH'}
            or data['exit_code'] != 0 or data['sources'] != {source.name:pin}):
        raise ValueError('UnsupportedPreparationBuild')
    headers = {str(Path(line.split('Note: including file:',1)[1].strip()))
               for line in data['stdout'].splitlines() if 'Note: including file:' in line}
    libraries = {str(Path(p)) for p in re.findall(r'[A-Za-z]:\\[^\r\n]*?\.lib',data['stdout'],re.I)}
    dependencies = data['dependencies']
    required = {provenance.original(source),str(compiler),str(library),str(Path(sys.base_prefix)/'python312.dll'),
                provenance.original(V4/'tools/build_background_preparation.py')}
    required.update(headers); required.update(libraries)
    required.update(str(p) for p in include.rglob('*.h'))
    if (not headers or headers != set(data['included_headers'])
            or libraries != set(data['link_libraries']) or str(library) not in libraries
            or not required <= set(dependencies)
            or set(data['precompile_dependencies']) != set(dependencies)
            or not (headers|libraries) <= set(data['precompile_dependencies'])):
        raise ValueError('IncompletePreparationBuildClosure')
    if (data['python_version'] != sys.version
            or data['implementation'] != platform.python_implementation()
            or data['extension_suffix'] != sysconfig.get_config_var('EXT_SUFFIX')):
        raise ValueError('ChangedPreparationABI')


_issued = weakref.WeakKeyDictionary()


class Preparation:
    """Private process-local custody, held throughout the background consumer."""
    __slots__ = ('_extension','_active','source_sha256','binary_sha256','build_sha256','__weakref__')

    def __init__(self, *args, **kwargs):
        raise TypeError('BackgroundPreparationFactoryRequired')

    def require_live(self):
        if self not in _issued or _issued[self] is not self._extension:
            if getattr(self,'_active',None) is False:
                raise ValueError('ClosedBackgroundPreparation')
            raise ValueError('OwnedBackgroundPreparationRequired')
        if not self._active:
            raise ValueError('ClosedBackgroundPreparation')
        return self._extension

    def partition(self, values, *, return_index=False, return_inverse=False, **kwargs):
        extension = self.require_live()
        if (kwargs or not return_index or not return_inverse or type(values) is not np.ndarray
                or values.dtype != np.dtype('uint32') or values.ndim != 1
                or not values.flags.c_contiguous or not values.flags.aligned):
            return np.unique(values,return_index=return_index,return_inverse=return_inverse,**kwargs)
        # Preserve np.unique's private flattened snapshot. Native loops release
        # the GIL for monitoring; external input buffers must not be observed.
        captured = values.tobytes(order='C')
        view = np.frombuffer(captured,dtype=np.uint32)
        if view.size and view.max() >= 16777216:
            return np.unique(view,return_index=return_index,return_inverse=return_inverse)
        inverse = np.empty(len(view),dtype=np.int64)
        raw, first = extension.partition(captured,inverse)
        return np.frombuffer(raw,dtype=np.uint32).copy(), np.frombuffer(first,dtype=np.int64).copy(), inverse


def require(owner):
    if type(owner) is not Preparation:
        raise ValueError('OwnedBackgroundPreparationRequired')
    return owner.require_live()


@contextmanager
def held():
    if os.name != 'nt' or sys.version_info[:2] != (3,12):
        yield None
        return
    source = V4/'runtime/source/background_preparation/preparation.cpp'
    owner = None
    with ExitStack() as locked:
        provenance.hold(locked)
        # Guard BEFORE deriving the locator; no stale source hash window.
        locked.enter_context(guard(source)); pin = file_hash(source)
        target = source.parent/('_v4_background_preparation_'+pin+sysconfig.get_config_var('EXT_SUFFIX'))
        record = target.with_name(target.name+'.BUILD.json')
        for p in (target,record,Path(__file__)):
            locked.enter_context(guard(p))
        data = provenance.receipt(record)
        validate(data,source,pin,target)
        for name, expected in data['dependencies'].items():
            path = provenance.resolve(name); locked.enter_context(guard(path))
            if file_hash(path) != expected:
                raise ValueError('ChangedPreparationDependency',name)
        if file_hash(source) != pin or file_hash(target) != data['binary_sha256']:
            raise ValueError('ChangedPreparationArtifact')
        spec = importlib.util.spec_from_file_location('_v4_background_preparation',target)
        extension = importlib.util.module_from_spec(spec); spec.loader.exec_module(extension)
        owner = object.__new__(Preparation)
        owner._extension = extension; owner._active = True
        owner.source_sha256 = pin; owner.binary_sha256 = data['binary_sha256']
        owner.build_sha256 = file_hash(record)
        _issued[owner] = extension
        try:
            yield owner
        finally:
            owner._active = False
            _issued.pop(owner,None)

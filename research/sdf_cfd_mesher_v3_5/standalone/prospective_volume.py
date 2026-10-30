"""Existing integer degree for stock production prospective volume checks.

The original volume auditor still constructs every exact cell and witness and
checks the complete closed boundary. Audit/default/custom paths stay original.
"""
from contextlib import contextmanager, nullcontext
from pathlib import Path
from types import CodeType, FunctionType, GetSetDescriptorType
from fractions import Fraction
import threading


def _codes(module, name):
    path=(Path(__file__).resolve().parent/'v4/tools'/name).resolve()
    if Path(module.__file__).resolve()!=path:return None
    # Compile, never execute, the held defining source. An import-time alias
    # alone could bless a custom function installed before this module loads.
    code=compile(path.read_text(encoding='utf-8'),str(path),'exec',dont_inherit=True)
    return {c.co_name:c for c in code.co_consts if type(c) is CodeType}


def _function(value,module,code,keywords=None,defaults=None):
    if (type(value) is not FunctionType or value.__globals__ is not vars(module) or
            value.__closure__ is not None or value.__code__!=code):return False
    actual_defaults=value.__defaults__
    if defaults is None:
        if actual_defaults is not None:return False
    elif (type(actual_defaults) is not tuple or len(actual_defaults)!=len(defaults) or
            any(type(a) is not type(b) or a!=b for a,b in zip(actual_defaults,defaults))):return False
    actual=value.__kwdefaults__
    if keywords is None:return actual is None
    return (type(actual) is dict and all(type(k) is str for k in actual) and set(actual)==set(keywords) and
            all(actual[k] is v for k,v in keywords.items()))


def _class(value,module,code):
    if type(value) is not type or value.__bases__!=(object,) or value.__module__!=module.__name__:
        return False
    methods={c.co_name for c in code.co_consts if type(c) is CodeType}
    namespace=vars(value)
    if set(namespace)!=methods|{'__module__','__dict__','__weakref__','__doc__'}:return False
    return (value.__new__ is object.__new__ and value.__getattribute__ is object.__getattribute__ and
        value.__setattr__ is object.__setattr__ and all(type(namespace[k]) is FunctionType for k in methods) and
        all(type(namespace[k]) is GetSetDescriptorType and namespace[k].__objclass__ is value for k in ('__dict__','__weakref__')))


def _cells(value):
    if type(value) is not list or not value:return None
    result=[]
    for c in value:
        if (type(c) is not dict or set(c)!={'id','column','layer','kind','vertices','faces'} or
                type(c['kind']) is not str or c['kind']!='prism' or any(type(c[k]) is not int for k in ('id','column','layer')) or
                type(c['vertices']) is not list or len(c['vertices'])!=6 or
                any(type(v) is not int for v in c['vertices']) or len(set(c['vertices']))!=6 or
                type(c['faces']) is not list or any(type(f) is not tuple or
                    len(f) not in (3,4) or any(type(v) is not int for v in f) for f in c['faces'])):
            return None
        result.append((c['id'],c['column'],c['layer'],c['kind'],tuple(c['vertices']),tuple(c['faces'])))
    return tuple(result)


def _triangles(value,count):
    return (type(value) is list and bool(value) and all(type(f) is tuple and len(f)==3 and
        all(type(v) is int and 0<=v<count for v in f) and len(set(f))==3 for f in value))


@contextmanager
def production():
    import numpy as np
    import cavity_volume_audit as reference
    import projected_boundary_index as index
    import termination_geometry_certificate as certificate
    import local_cavity_remesh as cavity
    from sdf_cfd_mesher_v4.certificate import aabb_index as tree
    from prism_degree import serial
    from sdf_cfd_mesher_v4.common.authority import frozen
    from unittest.mock import patch
    audit=reference.audit;degree=reference.degree
    definitions=_codes(reference,'cavity_volume_audit.py')
    index_definitions=_codes(index,'projected_boundary_index.py')
    rational_definitions=_codes(certificate,'termination_geometry_certificate.py')
    tree_definitions=_codes(tree,'../src/sdf_cfd_mesher_v4/certificate/aabb_index.py')
    if definitions is None or index_definitions is None or rational_definitions is None or tree_definitions is None:
        yield;return
    methods={c.co_name:c for c in index_definitions['ProjectedBoundaryIndex'].co_consts if type(c) is CodeType}
    tree_methods={c.co_name:c for c in tree_definitions['AABBIndex'].co_consts if type(c) is CodeType}
    def stock():
        return (reference.degree is degree and reference.F is Fraction and reference.np is np and
            reference.s is cavity and cavity.cert is certificate and certificate.F is Fraction and
            reference.ProjectedBoundaryIndex is index.ProjectedBoundaryIndex and index.np is np and
            index.AABBIndex is tree.AABBIndex and tree.np is np and
            _class(tree.AABBIndex,tree,tree_definitions['AABBIndex']) and
            _class(index.ProjectedBoundaryIndex,index,index_definitions['ProjectedBoundaryIndex']) and
            _function(audit,reference,definitions['audit'],{'_degree':None}) and
            _function(degree,reference,definitions['degree'],{'stats':None}) and
            _function(certificate.rational,certificate,rational_definitions['rational']) and
            _function(tree.AABBIndex.__init__,tree,tree_methods['__init__'],defaults=(32,)) and
            all(_function(getattr(index.ProjectedBoundaryIndex,k),index,methods[k]) for k in ('__init__','containing')))
    if not stock():
        yield;return
    thread=threading.get_ident()
    def dispatch(*args,**kwargs):
        if reference.audit is not dispatch or not stock():raise ValueError('ChangedProspectiveVolumeBinding')
        if (threading.get_ident()!=thread or len(args)!=4 or set(kwargs)-{'_degree'} or
                kwargs.get('_degree') is not None):return audit(*args,**kwargs)
        points,cells,outer,wall=args
        if (type(points) is not np.ndarray or points.dtype!=np.dtype('<f8') or
                points.ndim!=2 or points.shape[1]!=3 or not len(points) or
                not points.flags.c_contiguous or not np.isfinite(points).all() or
                not _triangles(outer,len(points)) or not _triangles(wall,len(points))):
            return audit(*args,**kwargs)
        cell_stamp=_cells(cells)
        if cell_stamp is None:return audit(*args,**kwargs)
        private=frozen(points,'<f8');metadata=(points.shape,points.dtype.str,points.strides)
        outer_stamp=tuple(outer);wall_stamp=tuple(wall)
        def unchanged():
            return (reference.audit is dispatch and stock() and
                (points.shape,points.dtype.str,points.strides)==metadata and
                points.tobytes()==private.tobytes() and _cells(cells)==cell_stamp and
                _triangles(outer,len(private)) and _triangles(wall,len(private)) and
                tuple(outer)==outer_stamp and tuple(wall)==wall_stamp)
        def integer(p,boundary,samples):
            if not unchanged():raise ValueError('ChangedProspectiveVolumeInputs')
            if (p is not points or type(samples) is not list or len(samples)!=len(cells) or
                    any(type(s) is not tuple or len(s)!=3 or any(type(q) is not Fraction for q in s) for s in samples) or
                    type(boundary) is not list or tuple(boundary)!=tuple(tuple(reversed(f)) for f in wall_stamp)+outer_stamp):
                return degree(p,boundary,samples)
            # Detach rational payloads as integer ratios, not aliases to mutable
            # caller containers. The backend still uses its existing exact math.
            ratios=tuple(tuple((q.numerator,q.denominator) for q in s) for s in samples)
            witnesses=tuple(tuple(Fraction(n,d) for n,d in s) for s in ratios)
            result=serial(private,tuple(boundary),witnesses)
            if (not unchanged() or tuple(boundary)!=tuple(tuple(reversed(f)) for f in wall_stamp)+outer_stamp or
                    tuple(tuple((q.numerator,q.denominator) for q in s) for s in witnesses)!=ratios or
                    tuple(tuple((q.numerator,q.denominator) for q in s) for s in samples)!=ratios):
                raise ValueError('ChangedProspectiveVolumeInputs')
            return result
        result=audit(*args,_degree=integer)
        if not unchanged():raise ValueError('ChangedProspectiveVolumeInputs')
        return result
    with patch.object(reference,'audit',dispatch):
        try:yield
        finally:
            if reference.audit is not dispatch or not stock():raise ValueError('ChangedProspectiveVolumeBinding')


def for_mode(mode):
    if mode=='production':return production()
    if mode=='audit':return nullcontext()
    raise ValueError('UnknownVerificationMode')

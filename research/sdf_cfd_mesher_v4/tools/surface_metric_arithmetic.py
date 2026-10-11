"""Exact dyadic fast path for the existing construction majorant predicate.

Every finite binary64 value is an integer divided by a power of two. A common
positive denominator cancels from all principal-minor signs. No tolerance,
floating sign estimate, retained proof or final-acceptance replacement is used.
Finite symmetric tensors use the same exact Gershgorin strengthening and
upward materialization. Unsupported inputs retain the stock implementation.
"""
import types
import math
from pathlib import Path
from fractions import Fraction
import numpy as np
import prism_host_metric as _stock

# Compile the already qualified defining source to identify its function code.
# Compilation does not execute this module or introduce another metric owner.
_STANDARD_CODES={c.co_name:c for c in compile(Path(_stock.__file__).read_bytes(),
    _stock.__file__,'exec',dont_inherit=True).co_consts if type(c) is types.CodeType}

def _standard(function,name):
    return (type(function) is types.FunctionType and function.__code__==_STANDARD_CODES[name]
            and function.__globals__ is _stock.__dict__ and function.__closure__ is None
            and function.__defaults__ is None)

def _standard_preservation():
    return (_standard(_stock._preserve_geometry,'_preserve_geometry')
            and _standard(_stock._exact_psd,'_exact_psd')
            and _stock.np is np and _stock.math is math and _stock.Fraction is Fraction)

_PRESERVE=_stock._preserve_geometry
_PSD=_stock._exact_psd
_INTERSECT=_stock.intersect_tangent_metric
_GLOBALS=({name:_INTERSECT.__globals__[name] for name in _INTERSECT.__code__.co_names
          if name in _INTERSECT.__globals__} if type(_INTERSECT) is types.FunctionType else {})

def _native_symmetric(matrix):
    return (type(matrix) is np.ndarray and matrix.dtype==np.dtype(np.float64)
            and matrix.shape==(3,3) and np.isfinite(matrix).all()
            and np.array_equal(matrix,matrix.T))

def _scaled_difference(actual,required):
    """All inputs captured once as exact dyadic integer ratios."""
    values=[float(v).as_integer_ratio() for v in actual.flat]
    values.extend(float(v).as_integer_ratio() for v in required.flat)
    denominator=max(d for _,d in values)
    scaled=[n*(denominator//d) for n,d in values]
    delta=[scaled[i]-scaled[i+9] for i in range(9)]
    return denominator,scaled[:9],delta

def _psd(delta):
    a,b,c,_,d,e,_,_,f=delta
    return (min(a,d,f)>=0 and a*d-b*b>=0 and a*f-c*c>=0
            and d*f-e*e>=0 and a*(d*f-e*e)-b*(b*f-c*e)+c*(b*e-c*d)>=0)

def preserve_geometry(result,required):
    if (not _standard_preservation() or _stock._preserve_geometry is not _PRESERVE or _stock._exact_psd is not _PSD):
        return _stock._preserve_geometry(result,required)
    if not (_native_symmetric(result) and _native_symmetric(required)):
        return _PRESERVE(result,required)
    denominator,actual,delta=_scaled_difference(result,required)
    if _psd(delta):
        return result,dict(geometry_preservation='ComputedGeometryMajorant/1',
                           geometry_preservation_diagonals_changed=0)
    shift=max(0,*(sum(abs(delta[3*i+j]) for j in range(3) if i!=j)-delta[4*i]
                  for i in range(3)))
    strengthened=result.copy()
    for i in range(3):
        target=actual[4*i]+shift
        try:rounded=float(Fraction(target,denominator))
        except OverflowError as failure:raise ValueError('UnrepresentablePreservedMetric') from failure
        if not math.isfinite(rounded):raise ValueError('UnrepresentablePreservedMetric')
        n,d=rounded.as_integer_ratio()
        if n*denominator<target*d:rounded=float(np.nextafter(rounded,math.inf))
        if not math.isfinite(rounded):raise ValueError('UnrepresentablePreservedMetric')
        strengthened[i,i]=rounded
    return strengthened,dict(geometry_preservation='ComputedGeometryMajorant/1',
        geometry_preservation_diagonals_changed=int(np.count_nonzero(np.diag(strengthened)!=np.diag(result))))

def _psd_difference(actual,required):
    return _psd(_scaled_difference(actual,required)[2])

_accelerated=None
if _standard(_INTERSECT,'intersect_tangent_metric') and _standard_preservation():
    _namespace=dict(_INTERSECT.__globals__);_namespace['_preserve_geometry']=preserve_geometry
    _accelerated=types.FunctionType(_INTERSECT.__code__,_namespace,_INTERSECT.__name__,_INTERSECT.__defaults__)
    _accelerated.__kwdefaults__=_INTERSECT.__kwdefaults__

def intersect_tangent_metric(*args,**kwargs):
    # Preserve late custom owners/dependencies rather than freezing their work.
    if (_accelerated is None or not _standard_preservation() or _stock.intersect_tangent_metric is not _INTERSECT or
            any(_INTERSECT.__globals__.get(k) is not v for k,v in _GLOBALS.items())):
        return _stock.intersect_tangent_metric(*args,**kwargs)
    return _accelerated(*args,**kwargs)

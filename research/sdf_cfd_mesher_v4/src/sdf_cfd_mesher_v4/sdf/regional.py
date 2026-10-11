"""Conservative mathematical enclosures for source-issued SDF expressions.

Source algebra with separate proof and numerical construction arithmetic.
No Rhai parser, shape construction, mesh or input issuer.
The expression must come from the canonical object, through a verified owner.
Binary32 parameter bits define exact real coefficients. Runtime binary32 query
error/consistency is a separate obligation; this module does not attest to it.
"""
from dataclasses import dataclass
from fractions import Fraction as Q
import math
import struct
from contextlib import contextmanager
from contextvars import ContextVar
from functools import lru_cache
from collections import Counter, OrderedDict
import ctypes, os, sys, threading, time

_filtered = ContextVar('regional_arithmetic_filter', default=None)


class ArithmeticFilterUnresolved(ValueError):
    """Internal filter uncertainty; never an acceptance result."""
    pass


class UnresolvedRegion(ValueError):
    pass


def exact(value):
    if isinstance(value,Q):return value
    if type(value) not in (int,float) or not math.isfinite(value):
        raise ValueError('FiniteExactCoordinateRequired')
    return Q(value)


@dataclass(frozen=True)
class Interval:
    lo: Q
    hi: Q

    def __post_init__(self):
        object.__setattr__(self,'lo',exact(self.lo));object.__setattr__(self,'hi',exact(self.hi))
        if self.lo>self.hi:raise ValueError('ReversedInterval')

    @classmethod
    def point(cls,value):return cls(exact(value),exact(value))
    def __neg__(self):return Interval(-self.hi,-self.lo)
    def __add__(self,other):
        other=interval(other);return Interval(self.lo+other.lo,self.hi+other.hi)
    __radd__=__add__
    def __sub__(self,other):return self+-interval(other)
    def __rsub__(self,other):return interval(other)+-self
    def __mul__(self,other):
        other=interval(other);ends=[a*b for a in (self.lo,self.hi) for b in (other.lo,other.hi)]
        return Interval(min(ends),max(ends))
    __rmul__=__mul__
    def reciprocal(self):
        if self.lo<=0<=self.hi:raise UnresolvedRegion('DenominatorIncludesZero')
        return Interval(1/self.hi,1/self.lo)
    def __truediv__(self,other):return self*interval(other).reciprocal()
    def square(self):
        return Interval(0 if self.lo<=0<=self.hi else min(self.lo**2,self.hi**2),max(self.lo**2,self.hi**2))
    def sqrt(self,bits):
        if self.lo<0:raise UnresolvedRegion('NegativeSquareRootRegion')
        def lower(x):
            return Q(math.isqrt((x.numerator<<(2*bits))//x.denominator),1<<bits)
        lo,hi=lower(self.lo),lower(self.hi)
        return Interval(lo,hi if hi*hi==self.hi else hi+Q(1,1<<bits))
    def hull(self,other):
        other=interval(other);return Interval(min(self.lo,other.lo),max(self.hi,other.hi))
    @property
    def magnitude(self):return max(abs(self.lo),abs(self.hi))


def interval(value):return value if isinstance(value,Interval) else Interval.point(value)


ZERO=Interval.point(0)


@dataclass(frozen=True)
class Jet:
    value: Interval
    gradient: tuple

    @classmethod
    def constant(cls,value):return cls(interval(value),(ZERO,)*3)
    @classmethod
    def variable(cls,value,axis):
        return cls(interval(value),tuple(Interval.point(int(i==axis)) for i in range(3)))
    def __neg__(self):return Jet(-self.value,tuple(-g for g in self.gradient))
    def __add__(self,other):
        other=jet(other);return Jet(self.value+other.value,tuple(a+b for a,b in zip(self.gradient,other.gradient)))
    __radd__=__add__
    def __sub__(self,other):return self+-jet(other)
    def __rsub__(self,other):return jet(other)+-self
    def __mul__(self,other):
        other=jet(other)
        return Jet(self.value*other.value,tuple(a*other.value+self.value*b for a,b in zip(self.gradient,other.gradient)))
    __rmul__=__mul__
    def __truediv__(self,other):
        other=jet(other);inverse=other.value.reciprocal()
        return self*Jet(inverse,tuple(-g*inverse.square() for g in other.gradient))
    def square(self):return Jet(self.value.square(),tuple(2*self.value*g for g in self.gradient))
    def sqrt(self,bits):
        root=self.value.sqrt(bits)
        if root.lo<=0:
            raise UnresolvedRegion('SquareRootGradientIncludesSingularity')
        return Jet(root,tuple(g/(2*root) for g in self.gradient))


@lru_cache(maxsize=8192)
def _float_endpoints(value):
    """Enclose an exact coefficient without replacing its source bits."""
    value=exact(value); rounded=float(value)
    if not math.isfinite(rounded):raise ArithmeticFilterUnresolved('CoefficientRange')
    q=Q(rounded)
    lo=math.nextafter(rounded,-math.inf) if q>value else rounded
    hi=math.nextafter(rounded,math.inf) if q<value else rounded
    # Pure global cache is valid independently of the conversion environment.
    # Only an exactly checked bracket may enter it, including pre-scope callers.
    if (not math.isfinite(lo) or not math.isfinite(hi) or
            not Q(lo)<=value<=Q(hi)):
        raise ArithmeticFilterUnresolved('CoefficientEnclosureUnresolved')
    return lo,hi


@dataclass(frozen=True)
class FloatInterval:
    """Binary64 outward bounds. No native binary32 equivalence assertion."""
    lo: float
    hi: float

    def __post_init__(self):
        if (type(self.lo) is not float or type(self.hi) is not float or
                not math.isfinite(self.lo) or not math.isfinite(self.hi) or self.lo>self.hi):
            raise ArithmeticFilterUnresolved('FloatingIntervalRange')

    @classmethod
    def point(cls,value):
        if isinstance(value,FloatInterval):return value
        if isinstance(value,Interval):
            return cls(_float_endpoints(value.lo)[0],_float_endpoints(value.hi)[1])
        lo,hi=_float_endpoints(value);return cls(lo,hi)

    @staticmethod
    def outward(lo,hi):
        return FloatInterval(math.nextafter(lo,-math.inf),math.nextafter(hi,math.inf))

    def __neg__(self):return FloatInterval(-self.hi,-self.lo)
    def __add__(self,other):
        other=FloatInterval.point(other)
        if other.lo==other.hi==0:return self
        if self.lo==self.hi==0:return other
        return self.outward(self.lo+other.lo,self.hi+other.hi)
    __radd__=__add__
    def __sub__(self,other):return self+-FloatInterval.point(other)
    def __rsub__(self,other):return FloatInterval.point(other)+-self
    def __mul__(self,other):
        other=FloatInterval.point(other)
        if self.lo==self.hi==0 or other.lo==other.hi==0:return FloatInterval(0.,0.)
        ends=(self.lo*other.lo,self.lo*other.hi,self.hi*other.lo,self.hi*other.hi)
        if not all(math.isfinite(v) for v in ends):raise ArithmeticFilterUnresolved('ProductRange')
        return self.outward(min(ends),max(ends))
    __rmul__=__mul__
    def reciprocal(self):
        if self.lo<=0<=self.hi:raise ArithmeticFilterUnresolved('DenominatorIncludesZero')
        return self.outward(1./self.hi,1./self.lo)
    def __truediv__(self,other):return self*FloatInterval.point(other).reciprocal()
    def square(self):
        ends=(self.lo*self.lo,self.hi*self.hi)
        hi=math.nextafter(max(ends),math.inf)
        lo=0. if self.lo<=0<=self.hi else max(0.,math.nextafter(min(ends),-math.inf))
        return FloatInterval(lo,hi)
    def sqrt(self,bits):
        if self.lo<0:raise ArithmeticFilterUnresolved('NegativeSquareRootRegion')
        lo=max(0.,math.nextafter(math.sqrt(self.lo),-math.inf))
        hi=math.nextafter(math.sqrt(self.hi),math.inf)
        # Certify libm's candidates by exact squares; no libm accuracy premise.
        if Q(lo)**2>Q(self.lo) or Q(hi)**2<Q(self.hi):
            raise ArithmeticFilterUnresolved('SquareRootRoundingUnresolved')
        return FloatInterval(lo,hi)
    def hull(self,other):
        other=FloatInterval.point(other)
        return FloatInterval(min(self.lo,other.lo),max(self.hi,other.hi))
    @property
    def magnitude(self):return max(abs(self.lo),abs(self.hi))
    def rational(self):return Interval(Q(self.lo),Q(self.hi))


@dataclass(frozen=True)
class FilteredJet(Jet):
    """Same expression/branch algebra with a different arithmetic strategy."""
    @classmethod
    def coerce(cls,value):
        if isinstance(value,cls):return value
        if isinstance(value,Jet):
            return cls(FloatInterval.point(value.value),tuple(FloatInterval.point(g) for g in value.gradient))
        return cls.constant(value)
    @classmethod
    def constant(cls,value):return cls(FloatInterval.point(value),(FloatInterval(0.,0.),)*3)
    @classmethod
    def variable(cls,value,axis):
        return cls(FloatInterval.point(value),tuple(FloatInterval.point(int(i==axis)) for i in range(3)))
    def __neg__(self):return FilteredJet(-self.value,tuple(-g for g in self.gradient))
    def __add__(self,other):
        other=self.coerce(other)
        return FilteredJet(self.value+other.value,tuple(a+b for a,b in zip(self.gradient,other.gradient)))
    def __radd__(self,other):return self+other
    def __sub__(self,other):return self+-self.coerce(other)
    def __rsub__(self,other):return self.coerce(other)+-self
    def __mul__(self,other):
        other=self.coerce(other)
        return FilteredJet(self.value*other.value,tuple(a*other.value+self.value*b for a,b in zip(self.gradient,other.gradient)))
    def __rmul__(self,other):return self*other
    def __truediv__(self,other):
        other=self.coerce(other);inverse=other.value.reciprocal()
        return self*FilteredJet(inverse,tuple(-g*inverse.square() for g in other.gradient))
    def square(self):return FilteredJet(self.value.square(),tuple(2*self.value*g for g in self.gradient))
    def sqrt(self,bits):
        root=self.value.sqrt(bits)
        if root.lo<=0:raise ArithmeticFilterUnresolved('SquareRootGradientIncludesSingularity')
        return FilteredJet(root,tuple(g/(2*root) for g in self.gradient))
    def rational(self):return Jet(self.value.rational(),tuple(g.rational() for g in self.gradient))


def _filter_arithmetic():
    if os.name!='nt' or ctypes.sizeof(ctypes.c_void_p)!=8 or sys.float_info.mant_dig!=53:
        raise ArithmeticFilterUnresolved('UnqualifiedFilterHost')
    crt=ctypes.CDLL('ucrtbase.dll');read=crt._controlfp_s
    read.argtypes=(ctypes.POINTER(ctypes.c_uint),ctypes.c_uint,ctypes.c_uint)
    read.restype=ctypes.c_int
    def check():
        word=ctypes.c_uint()
        if read(ctypes.byref(word),0,0) or word.value & (0x300|0x3000000):
            raise ArithmeticFilterUnresolved('FilterRequiresNearestAndGradualUnderflow')
    check();return check


@contextmanager
def filtered_regions(verify):
    """Call-scoped arithmetic/cache only; caller retains source acceptance custody."""
    if _filtered.get() is not None:raise ValueError('NestedRegionalFilterScope')
    verify();arithmetic=_filter_arithmetic();thread=threading.get_ident()
    state=dict(active=True,exact=False,counts=Counter(),seconds=Counter(),cache=OrderedDict())
    def check():
        if not state['active'] or threading.get_ident()!=thread:raise ValueError('ExpiredOrCrossThreadFilter')
        verify();arithmetic()
    state['check']=check
    token=_filtered.set(state)
    def snapshot():
        # Descriptive counters remain readable after a rejected computation;
        # they contain no acceptance values or source custody attestation.
        if not state['active'] or threading.get_ident()!=thread:raise ValueError('ExpiredOrCrossThreadFilter')
        return dict(schema='V4RegionalArithmeticFilter/1',counts=dict(state['counts']),
            computation_seconds=dict(state['seconds']),cache_entries=len(state['cache']),
            role='Same continuous-real expression; conservative arithmetic; rational fallback')
    try:
        yield snapshot
        check()
    finally:
        state['active']=False;state['cache'].clear();_filtered.reset(token)


def invalidate_filtered_cache():
    """Drop rejected call-scoped values; keep descriptive counters."""
    state=_filtered.get()
    if state is not None:state['cache'].clear()


def exact_fallback(compute):
    """Retry an inconclusive filtered query in the original arithmetic."""
    state=_filtered.get()
    if state is None or state['exact']:return compute()
    state['check']();state['counts']['exact_query_fallbacks']+=1;state['exact']=True
    try:return compute()
    finally:state['exact']=False;state['check']()


def filtered_bound(compute):
    """Two arithmetic tiers on ONE patch; subdivision budgets are unchanged."""
    state=_filtered.get()
    if state is None or state['exact']:return compute()
    state['check']();state['counts']['initial_patch_bounds']+=1
    try:bound=compute()
    except UnresolvedRegion:bound=None
    if bound is not None:
        state['counts']['filtered_patch_passes']+=1;return bound
    state['counts']['exact_patch_fallbacks']+=1
    return exact_fallback(compute)


@dataclass(frozen=True)
class RealValue:
    """Value-only real interval arithmetic on the same captured Expression.

    No native rounding model and no derivative at a zero norm. Used only by
    real-source coverage; native enclosures and differentiable Jet roles stay
    separate. Exact rational endpoints and outward sqrt own all bounds.
    """
    value: Interval
    @classmethod
    def constant(cls,value):return cls(interval(value))
    def __neg__(self):return RealValue(-self.value)
    def __add__(self,other):return RealValue(self.value+real_value(other).value)
    __radd__=__add__
    def __sub__(self,other):return self+-real_value(other)
    def __rsub__(self,other):return real_value(other)+-self
    def __mul__(self,other):return RealValue(self.value*real_value(other).value)
    __rmul__=__mul__
    def __truediv__(self,other):return RealValue(self.value/real_value(other).value)
    def square(self):return RealValue(self.value.square())
    def sqrt(self,bits):return RealValue(self.value.sqrt(bits))


def real_value(value):return value if isinstance(value,RealValue) else RealValue.constant(value)


@dataclass(frozen=True)
class NumericalJet:
    """Point value and derivative proposals. No enclosure or proof semantics."""
    value: float
    gradient: tuple

    def __post_init__(self):
        if not math.isfinite(self.value) or len(self.gradient)!=3 or not all(math.isfinite(g) for g in self.gradient):
            raise ValueError('NonfiniteConstructionQuery')
    @classmethod
    def coerce(cls,value):
        if type(value) is cls:return value
        if isinstance(value,(Jet,RealValue,Float32Value,NumericalBatchJet)):
            raise ValueError('MixedConstructionAndProofArithmetic')
        return cls(float(value),(0.,)*3)
    @classmethod
    def variable(cls,value,axis):return cls(float(value),tuple(float(i==axis) for i in range(3)))
    def __neg__(self):return NumericalJet(-self.value,tuple(-g for g in self.gradient))
    def __add__(self,other):
        other=self.coerce(other)
        return NumericalJet(self.value+other.value,tuple(a+b for a,b in zip(self.gradient,other.gradient)))
    __radd__=__add__
    def __sub__(self,other):return self+-self.coerce(other)
    def __rsub__(self,other):return self.coerce(other)+-self
    def __mul__(self,other):
        other=self.coerce(other)
        return NumericalJet(self.value*other.value,tuple(a*other.value+self.value*b for a,b in zip(self.gradient,other.gradient)))
    __rmul__=__mul__
    def __truediv__(self,other):
        other=self.coerce(other)
        if other.value==0:raise ValueError('ConstructionDivisionByZero')
        inverse=1/other.value
        return self*NumericalJet(inverse,tuple(-g*inverse*inverse for g in other.gradient))
    def square(self):return NumericalJet(self.value*self.value,tuple(2*self.value*g for g in self.gradient))
    def sqrt(self,bits):
        if self.value<0:raise ValueError('NegativeConstructionSquareRoot')
        root=math.sqrt(self.value)
        # At a hard branch singularity this is only a subgradient proposal.
        # Final differentiability and normal certification remain mandatory.
        return NumericalJet(root,tuple(g/(2*root) for g in self.gradient) if root else (0.,)*3)


def numerical_point(expression,location_m,metres_per_model_unit):
    """Same captured expression algebra, binary64 point proposals only."""
    if type(expression) is not Expression:raise ValueError('ImmutableExpressionRequired')
    scale=float(metres_per_model_unit)
    if not math.isfinite(scale) or scale<=0 or len(location_m)!=3:raise ValueError('FiniteConstructionScaleAndPointRequired')
    p=tuple(NumericalJet.variable(float(v),i)/scale for i,v in enumerate(location_m))
    result=expression._evaluate(p,0)*scale
    if type(result) is not NumericalJet:raise ValueError('ConstructionPointRoleRequired')
    return result



@dataclass(frozen=True)
class NumericalBatchJet:
    """Array-valued binary64 proposals; never an interval or certificate."""
    value: object
    gradient: tuple

    def __post_init__(self):
        import numpy as np
        if (np.ndim(self.value)!=1 or len(self.gradient)!=3 or
                any(np.shape(g)!=np.shape(self.value) for g in self.gradient) or
                not np.isfinite(self.value).all() or not all(np.isfinite(g).all() for g in self.gradient)):
            raise ValueError('NonfiniteConstructionQuery')
    @classmethod
    def coerce(cls,value,like):
        import numpy as np
        if type(value) is cls:return value
        if isinstance(value,(Jet,RealValue,Float32Value,NumericalJet)):
            raise ValueError('MixedConstructionAndProofArithmetic')
        return cls(np.full_like(like,float(value)),tuple(np.zeros_like(like) for _ in range(3)))
    @classmethod
    def variable(cls,value,axis):
        import numpy as np
        return cls(value,tuple(np.full_like(value,float(i==axis)) for i in range(3)))
    def __neg__(self):return NumericalBatchJet(-self.value,tuple(-g for g in self.gradient))
    def __add__(self,other):
        other=self.coerce(other,self.value)
        return NumericalBatchJet(self.value+other.value,tuple(a+b for a,b in zip(self.gradient,other.gradient)))
    __radd__=__add__
    def __sub__(self,other):return self+-self.coerce(other,self.value)
    def __rsub__(self,other):return self.coerce(other,self.value)+-self
    def __mul__(self,other):
        other=self.coerce(other,self.value)
        return NumericalBatchJet(self.value*other.value,tuple(a*other.value+self.value*b for a,b in zip(self.gradient,other.gradient)))
    __rmul__=__mul__
    def __truediv__(self,other):
        import numpy as np
        other=self.coerce(other,self.value)
        if np.any(other.value==0):raise ValueError('ConstructionDivisionByZero')
        inverse=1/other.value
        return self*NumericalBatchJet(inverse,tuple(-g*inverse*inverse for g in other.gradient))
    def square(self):return NumericalBatchJet(self.value*self.value,tuple(2*self.value*g for g in self.gradient))
    def sqrt(self,bits):
        import numpy as np
        if np.any(self.value<0):raise ValueError('NegativeConstructionSquareRoot')
        root=np.sqrt(self.value)
        gradients=tuple(np.divide(g,2*root,out=np.zeros_like(root),where=root!=0) for g in self.gradient)
        return NumericalBatchJet(root,gradients)


def numerical_points(expression,locations_m,metres_per_model_unit):
    """Run the existing expression algebra on a finite binary64 point batch."""
    import numpy as np
    if type(expression) is not Expression:raise ValueError('ImmutableExpressionRequired')
    points=np.asarray(locations_m)
    scale=float(metres_per_model_unit)
    if (points.dtype!=np.dtype('float64') or points.ndim!=2 or points.shape[1]!=3 or
            not len(points) or not np.isfinite(points).all() or not math.isfinite(scale) or scale<=0):
        raise ValueError('FiniteConstructionScaleAndPointRequired')
    p=tuple(NumericalBatchJet.variable(points[:,i],i)/scale for i in range(3))
    result=expression._evaluate(p,0)*scale
    if type(result) is not NumericalBatchJet:raise ValueError('ConstructionPointRoleRequired')
    return result


def _batch_select(mask,a,b):
    import numpy as np
    return NumericalBatchJet(np.where(mask,a.value,b.value),tuple(np.where(mask,x,y) for x,y in zip(a.gradient,b.gradient)))


def _value_bounds(value):
    # Only the expression's branch optimization consumes these endpoints.
    # A numerical value never enters region(), a certificate, or a proof cache.
    return (value.value,value.value) if type(value) is NumericalJet else (value.value.lo,value.value.hi)


def jet(value):
    if type(value) in (NumericalJet,NumericalBatchJet):raise ValueError('MixedConstructionAndProofArithmetic')
    return value if isinstance(value,(Jet,Float32Value,RealValue)) else Jet.constant(value)


def minimum(a,b):
    if type(a) is NumericalBatchJet or type(b) is NumericalBatchJet:
        like=a.value if type(a) is NumericalBatchJet else b.value
        a,b=NumericalBatchJet.coerce(a,like),NumericalBatchJet.coerce(b,like)
        import numpy as np
        tie=a.value==b.value
        gradients=[]
        for x,y in zip(a.gradient,b.gradient):
            g=np.zeros_like(x);g[tie]=(x[tie]+y[tie])/2;gradients.append(g)
        equal=NumericalBatchJet(a.value,tuple(gradients))
        return _batch_select(a.value<b.value,a,_batch_select(b.value<a.value,b,equal))
    if type(a) is NumericalJet or type(b) is NumericalJet:
        a,b=NumericalJet.coerce(a),NumericalJet.coerce(b)
        if a.value<b.value:return a
        if b.value<a.value:return b
        return NumericalJet(a.value,tuple((x+y)/2 for x,y in zip(a.gradient,b.gradient)))
    if isinstance(a,FilteredJet) or isinstance(b,FilteredJet):
        a,b=FilteredJet.coerce(a),FilteredJet.coerce(b)
        if a.value.hi<b.value.lo:return a
        if b.value.hi<a.value.lo:return b
        return FilteredJet(FloatInterval(min(a.value.lo,b.value.lo),min(a.value.hi,b.value.hi)),
                           tuple(x.hull(y) for x,y in zip(a.gradient,b.gradient)))
    if isinstance(a,RealValue) or isinstance(b,RealValue):
        a,b=real_value(a),real_value(b)
        return RealValue(Interval(min(a.value.lo,b.value.lo),min(a.value.hi,b.value.hi)))
    if isinstance(a,Float32Value) or isinstance(b,Float32Value):
        a,b=float32_value(a),float32_value(b)
        return Float32Value(Interval(min(a.value.lo,b.value.lo),min(a.value.hi,b.value.hi)))
    a,b=jet(a),jet(b)
    if a.value.hi<b.value.lo:return a
    if b.value.hi<a.value.lo:return b
    return Jet(Interval(min(a.value.lo,b.value.lo),min(a.value.hi,b.value.hi)),
               tuple(x.hull(y) for x,y in zip(a.gradient,b.gradient)))


def maximum(a,b):
    if type(a) is NumericalBatchJet or type(b) is NumericalBatchJet:
        like=a.value if type(a) is NumericalBatchJet else b.value
        return -minimum(-NumericalBatchJet.coerce(a,like),-NumericalBatchJet.coerce(b,like))
    if type(a) is NumericalJet or type(b) is NumericalJet:return -minimum(-NumericalJet.coerce(a),-NumericalJet.coerce(b))
    if isinstance(a,RealValue) or isinstance(b,RealValue):return -minimum(-real_value(a),-real_value(b))
    return -minimum(-jet(a),-jet(b))
def absolute(a):return maximum(a,-a) if type(a) in (NumericalJet,NumericalBatchJet) else maximum(a,-jet(a))
def clamp01(a):return minimum(maximum(a,0),1)
def norm(vector,bits):
    if any(type(v) is NumericalBatchJet for v in vector):
        like=next(v.value for v in vector if type(v) is NumericalBatchJet)
        return sum((NumericalBatchJet.coerce(v,like).square() for v in vector),NumericalBatchJet.coerce(0,like)).sqrt(bits)
    if any(type(v) is NumericalJet for v in vector):
        return sum((NumericalJet.coerce(v).square() for v in vector),NumericalJet.coerce(0)).sqrt(bits)
    if any(isinstance(v,FilteredJet) for v in vector):
        return sum((FilteredJet.coerce(v).square() for v in vector),FilteredJet.constant(0)).sqrt(bits)
    if any(isinstance(v,RealValue) for v in vector):
        return sum((real_value(v).square() for v in vector),RealValue.constant(0)).sqrt(bits)
    if any(isinstance(v,Float32Value) for v in vector):
        # Includes unfused/fused products and every three-component reduction
        # ordering used by the canonical scalar/SIMD vector implementation.
        from itertools import permutations
        squares=tuple(float32_value(v).square() for v in vector)
        values=[sum(order,Float32Value.constant(0)).value for order in permutations(squares)]
        result=values[0]
        for value in values[1:]:result=result.hull(value)
        return Float32Value(result).sqrt(bits)
    return sum((v.square() for v in vector),Jet.constant(0)).sqrt(bits)


def binary32(bits):
    if type(bits) is not int or not 0<=bits<1<<32:raise ValueError('ExactBinary32ParameterBitsRequired')
    value=struct.unpack('!f',struct.pack('!I',bits))[0]
    if not math.isfinite(value):raise ValueError('NonfiniteSourceParameter')
    return Q(value)


SIGNATURES={
    'sphere':(1,()),'box':(3,()),'cylinder':(2,()),'torus':(2,()),
    'translate':(3,('child',)),'scale':(3,('child',)),'offset':(1,('child',)),
    'union':(0,('a','b')),'intersect':(0,('a','b')),'subtract':(0,('a','b')),
    'smooth_union':(1,('a','b')),'smooth_intersect':(1,('a','b')),
    'smooth_subtract':(1,('base','tool')),
}


@dataclass(frozen=True)
class Expression:
    kind: str
    parameters: tuple
    children: tuple

    def __post_init__(self):
        # Mathematical premises also apply to direct construction. This does
        # not make a caller-built expression a source-issued authority.
        if type(self.kind) is not str or self.kind not in SIGNATURES:
            raise ValueError('UnsupportedSourceBoundOperation')
        count,roles=SIGNATURES[self.kind]
        if type(self.parameters) is not tuple or len(self.parameters)!=count or any(type(v) is not Q for v in self.parameters):
            raise ValueError('ExactExpressionParametersRequired')
        if self.kind in ('sphere','box','cylinder','torus','scale','smooth_union','smooth_intersect','smooth_subtract') and any(v<=0 for v in self.parameters):
            raise ValueError('PositiveSourceParameterRequired')
        if (type(self.children) is not tuple or len(self.children)!=len(roles) or
                any(type(v) is not tuple or len(v)!=2 or type(v[0]) is not str or v[0]!=role or type(v[1]) is not Expression
                    for v,role in zip(self.children,roles))):
            raise ValueError('SourceBoundDependencyRoles')

    @property
    def lipschitz_bound(self):
        # All admitted primitives are 1-Lipschitz. Translation/offset preserve
        # it; positive diagonal scaling uses min(scale) in the source field.
        # Hard min/max and these positive-width smooth min/max operators have
        # branch/convex-combination gradients bounded by the largest child.
        return Q(1)

    @classmethod
    def from_packet(cls,packet,*,maximum_nodes):
        if type(maximum_nodes) is not int or maximum_nodes<1:raise ValueError('ExplicitExpressionBudgetRequired')
        left=maximum_nodes
        def parse(node,depth):
            nonlocal left
            left-=1
            if left<0 or depth>128:raise ValueError('SourceExpressionBudgetExceeded')
            if not isinstance(node,dict) or set(node)!=set(('kind','parameters_bits','children')):
                raise ValueError('SourceIssuedExpressionRequired')
            kind=node['kind']
            if kind not in SIGNATURES:raise ValueError('UnsupportedSourceBoundOperation',kind)
            count,roles=SIGNATURES[kind]
            if not isinstance(node['parameters_bits'],list) or len(node['parameters_bits'])!=count:
                raise ValueError('SourceBoundParameterCount')
            parameters=tuple(binary32(v) for v in node['parameters_bits'])
            children=node['children']
            if not isinstance(children,dict) or set(children)!=set(roles):raise ValueError('SourceBoundDependencyRoles')
            if kind in ('sphere','box','cylinder','torus','scale','smooth_union','smooth_intersect','smooth_subtract') and any(p<=0 for p in parameters):
                raise ValueError('PositiveSourceParameterRequired',kind)
            return cls(kind,parameters,tuple((role,parse(children[role],depth+1)) for role in roles))
        return parse(packet,0)

    def evaluate(self,points,*,precision_bits):
        if type(precision_bits) is not int or not 1<=precision_bits<=256:raise ValueError('ExplicitProofPrecisionRequired')
        if len(points)!=3:raise ValueError('ThreeCoordinateIntervalsRequired')
        return self._evaluate(tuple(jet(v) for v in points),precision_bits)

    def _evaluate(self,p,bits):
        k,parameters=self.kind,self.parameters;children=dict(self.children)
        if k=='sphere':return norm(p,bits)-parameters[0]
        if k=='torus':return norm((norm(p[:2],bits)-parameters[0],p[2]),bits)-parameters[1]
        if k=='box':
            q=tuple(absolute(v)-r for v,r in zip(p,parameters))
            if type(p[0]) is NumericalBatchJet:
                import numpy as np
                masks=[]
                for i,v in enumerate(q):
                    mask=(q[(i+1)%3].value<=0)&(q[(i+2)%3].value<=0)&(v.value>q[(i+1)%3].value)&(v.value>q[(i+2)%3].value)
                    masks.append(mask)
                general=~(masks[0]|masks[1]|masks[2])
                values=np.empty_like(q[0].value);gradients=tuple(np.empty_like(values) for _ in range(3))
                for mask,v in zip(masks,q):
                    values[mask]=v.value[mask]
                    for out,g in zip(gradients,v.gradient):out[mask]=g[mask]
                if np.any(general):
                    subset=tuple(NumericalBatchJet(v.value[general],tuple(g[general] for g in v.gradient)) for v in q)
                    v=norm(tuple(maximum(v,0) for v in subset),bits)+minimum(maximum(maximum(subset[0],subset[1]),subset[2]),0)
                    values[general]=v.value
                    for out,g in zip(gradients,v.gradient):out[general]=g
                return NumericalBatchJet(values,gradients)
            # At an isolated planar face, reduce the cancellation exactly.
            if not isinstance(p[0],Float32Value):
                for i,v in enumerate(q):
                    if all(_value_bounds(q[j])[1]<=0 and _value_bounds(v)[0]>_value_bounds(q[j])[1] for j in range(3) if j!=i):return v
            return norm(tuple(maximum(v,0) for v in q),bits)+minimum(maximum(maximum(q[0],q[1]),q[2]),0)
        if k=='cylinder':
            a,b=norm(p[:2],bits)-parameters[0],absolute(p[2])-parameters[1]
            if type(p[0]) is NumericalBatchJet:
                import numpy as np
                ma=(b.value<=0)&(a.value>b.value);mb=(a.value<=0)&(b.value>a.value);general=~(ma|mb)
                values=np.empty_like(a.value);gradients=tuple(np.empty_like(values) for _ in range(3))
                for mask,v in ((ma,a),(mb,b)):
                    values[mask]=v.value[mask]
                    for out,g in zip(gradients,v.gradient):out[mask]=g[mask]
                if np.any(general):
                    aa,bb=(NumericalBatchJet(v.value[general],tuple(g[general] for g in v.gradient)) for v in (a,b))
                    v=norm((maximum(aa,0),maximum(bb,0)),bits)+minimum(maximum(aa,bb),0)
                    values[general]=v.value
                    for out,g in zip(gradients,v.gradient):out[general]=g
                return NumericalBatchJet(values,gradients)
            if not isinstance(p[0],Float32Value):
                if _value_bounds(b)[1]<=0 and _value_bounds(a)[0]>_value_bounds(b)[1]:return a
                if _value_bounds(a)[1]<=0 and _value_bounds(b)[0]>_value_bounds(a)[1]:return b
            return norm((maximum(a,0),maximum(b,0)),bits)+minimum(maximum(a,b),0)
        if k=='translate':return children['child']._evaluate(tuple(v-r for v,r in zip(p,parameters)),bits)
        if k=='scale':return children['child']._evaluate(tuple(v/r for v,r in zip(p,parameters)),bits)*min(parameters)
        if k=='offset':return children['child']._evaluate(p,bits)-parameters[0]
        if k=='smooth_subtract':
            a=children['base']._evaluate(p,bits);b=-children['tool']._evaluate(p,bits)
        else:a=children['a']._evaluate(p,bits);b=children['b']._evaluate(p,bits)
        if k=='union':return minimum(a,b)
        if k=='intersect':return maximum(a,b)
        if k=='subtract':return maximum(a,-b)
        width=parameters[0]
        if k=='smooth_union':
            h=clamp01(Q(1,2)+Q(1,2)*(b-a)/width)
            return b*(1-h)+a*h-width*h*(1-h)
        if k=='smooth_subtract' and isinstance(p[0],Float32Value):
            h=clamp01(Q(1,2)-Q(1,2)*(a-b)/width)
            return a+(b-a)*h+width*h*(1-h)
        h=clamp01(Q(1,2)+Q(1,2)*(a-b)/width)
        return b*(1-h)+a*h+width*h*(1-h)


def region(expression,bounds_m,metres_per_model_unit,*,precision_bits):
    if type(precision_bits) is not int or not 1<=precision_bits<=256:raise ValueError('ExplicitProofPrecisionRequired')
    scale=exact(metres_per_model_unit)
    if scale<=0:raise ValueError('PositiveUnitScaleRequired')
    if len(bounds_m)!=3:raise ValueError('ThreeCoordinateIntervalsRequired')
    state=_filtered.get()
    if state is not None and not state['exact']:
        state['check']();state['counts']['region_requests']+=1
        key=(expression,tuple(interval(v) for v in bounds_m),scale,precision_bits)
        if key in state['cache']:
            state['counts']['region_hits']+=1;result=state['cache'][key]
            state['cache'].move_to_end(key);return result
        start=time.perf_counter()
        try:
            p=tuple(FilteredJet.variable(v,i)/scale for i,v in enumerate(key[1]))
            result=(expression.evaluate(p,precision_bits=precision_bits)*scale).rational()
        except (ArithmeticFilterUnresolved,OverflowError,ZeroDivisionError):
            state['counts']['exact_region_fallbacks']+=1
        else:
            state['check']();state['counts']['filtered_regions']+=1
            state['cache'][key]=result
            if len(state['cache'])>8192:state['cache'].popitem(last=False)
            return result
        finally:state['seconds']['filter_regions']+=time.perf_counter()-start
    p=tuple(Jet.variable(interval(v),i)/scale for i,v in enumerate(bounds_m))
    return expression.evaluate(p,precision_bits=precision_bits)*scale


def round32_enclosure(value):
    """Outward enclosure of exact/intermediate and binary32 rounded values.

    Including exact intermediates also covers contracted arithmetic. Underflow
    may flush to zero, so zero is included inside the subnormal range. Overflow
    is unresolved. This is an arithmetic model, not a claim about arbitrary
    kernel methods or compiler flags; source/operator equivalence must be bound.
    """
    value=interval(value)
    def endpoints(x):
        try:number=struct.unpack('!f',struct.pack('!f',float(x)))[0]
        except (OverflowError,struct.error):raise UnresolvedRegion('Binary32RangeExceeded')
        if not math.isfinite(number):raise UnresolvedRegion('Binary32RangeExceeded')
        encoded=struct.unpack('!I',struct.pack('!f',number))[0]
        actual=Q(number)
        def adjacent(up):
            if number==0:return Q(1 if up else -1,1<<149)
            target=encoded+(1 if (number>0)==up else -1)
            return binary32(target)
        # Both endpoints are exact Fractions. Compare their integer ratios
        # directly; retain the same directed binary32 bracket and zero rule.
        actual_side=actual.numerator*x.denominator
        source_side=x.numerator*actual.denominator
        lo=actual if actual_side<=source_side else adjacent(False)
        hi=actual if actual_side>=source_side else adjacent(True)
        if abs(x.numerator)*(1<<126)<x.denominator:lo=min(lo,Q(0));hi=max(hi,Q(0))
        return lo,hi
    lo,hi=endpoints(value.lo)
    if value.lo!=value.hi:_,hi=endpoints(value.hi)
    return Interval(min(lo,value.lo),max(hi,value.hi))


@dataclass(frozen=True)
class Float32Value:
    value: Interval
    @classmethod
    def constant(cls,value):return cls(round32_enclosure(interval(value)))
    def __neg__(self):return Float32Value(-self.value)
    def __add__(self,other):return Float32Value(round32_enclosure(self.value+float32_value(other).value))
    __radd__=__add__
    def __sub__(self,other):return self+-float32_value(other)
    def __rsub__(self,other):return float32_value(other)+-self
    def __mul__(self,other):return Float32Value(round32_enclosure(self.value*float32_value(other).value))
    __rmul__=__mul__
    def __truediv__(self,other):return Float32Value(round32_enclosure(self.value/float32_value(other).value))
    def square(self):return Float32Value(round32_enclosure(self.value.square()))
    def sqrt(self,bits):return Float32Value(round32_enclosure(self.value.sqrt(bits)))


def float32_value(value):
    if isinstance(value,Float32Value):return value
    if isinstance(value,Jet):
        if any(g.lo!=0 or g.hi!=0 for g in value.gradient):raise ValueError('MixedProofArithmetic')
        value=value.value
    return Float32Value.constant(value)


def query_enclosure(expression,point_m,metres_per_model_unit,*,precision_bits):
    """Enclose canonical binary32 scalar arithmetic at one supplied SI point.

    No sample is used to establish a continuous face bound. This separate check
    exposes the query-to-mathematical-expression error rather than assuming zero.
    Returns rational bounds so the SI binary64 result can be checked exactly.
    """
    if len(point_m)!=3:raise ValueError('ThreeCoordinatesRequired')
    if type(precision_bits) is not int or not 1<=precision_bits<=256:raise ValueError('ExplicitProofPrecisionRequired')
    scale=exact(metres_per_model_unit)
    if scale<=0:raise ValueError('PositiveUnitScaleRequired')
    model=tuple(Float32Value.constant(exact(p)/scale) for p in point_m)
    floating=expression._evaluate(model,precision_bits).value*scale
    # Native bridge performs one binary64 SI multiplication after field return.
    # The deliberately widened endpoint conversion includes its rounded result.
    lo=math.nextafter(float(floating.lo),-math.inf);hi=math.nextafter(float(floating.hi),math.inf)
    if not math.isfinite(lo) or not math.isfinite(hi):raise UnresolvedRegion('Binary64RangeExceeded')
    floating=Interval(min(floating.lo,Q(lo)),max(floating.hi,Q(hi)))
    mathematical=region(expression,tuple(Interval.point(p) for p in point_m),scale,precision_bits=precision_bits).value
    error=(floating-mathematical).magnitude
    return dict(native_interval=floating,mathematical_interval=mathematical,evaluation_error_bound_m=upper_float(error))


def upper_float(value):
    value=exact(value);result=float(value)
    if Q(result)<value:result=math.nextafter(result,math.inf)
    return result


@lru_cache(maxsize=64)
def cosine_upper(angle,bits):
    angle=exact(angle)
    if not 0<angle<2:raise ValueError('AcuteNormalProofLimitRequired')
    total=Q(1);term=Q(1);i=0
    while True:
        i+=1;term=-term*angle*angle/((2*i-1)*(2*i));total+=term
        if i%2==0 and abs(term)<=Q(1,1<<bits):return total


def triangle_field(expression, patch, scale, bits):
    """Mean-value enclosure on the triangle, not its enclosing volume.

    For each x in the triangle, F(x)-F(c) is an integral of gradients
    dotted with x-c. The interval gradient encloses every branch gradient
    on the convex hull. Each fixed gradient's affine extrema occur at the
    three vertices. Taking interval extrema there therefore covers the
    whole triangle, including continuous piecewise-C1 clamp seams.
    """
    box=tuple(Interval(min(p[i] for p in patch),max(p[i] for p in patch)) for i in range(3))
    centre=tuple(sum(p[i] for p in patch)/3 for i in range(3))
    value=region(expression,tuple(Interval.point(v) for v in centre),scale,precision_bits=bits).value
    gradient=region(expression,box,scale,precision_bits=bits).gradient
    changes=[sum((g*(p[i]-centre[i]) for i,g in enumerate(gradient)),ZERO) for p in patch]
    return value+Interval(min(v.lo for v in changes),max(v.hi for v in changes))


def normal_cone_pass(n, gradient, cosine, gradient2):
    """Two sufficient angle predicates; retain the previous fast proof OR."""
    directional=sum((g*ni for g,ni in zip(gradient,n)),ZERO).lo
    if directional<=0:return False
    n2=sum(v*v for v in n)
    if directional**2>=cosine**2*gradient2*n2:return True
    cross=(n[1]*gradient[2]-n[2]*gradient[1],n[2]*gradient[0]-n[0]*gradient[2],n[0]*gradient[1]-n[1]*gradient[0])
    cross2=sum(v.square().hi for v in cross)
    # tan(angle)^2 = |n x g|^2/(n dot g)^2. Exact cancellation
    # keeps tangent uncertainty separate from radial norm uncertainty.
    return cross2*cosine**2<=directional**2*(1-cosine**2)


def radial_mesh_coverage(expression, coordinates, faces, interior, *,
                         metres_per_model_unit, position_limit_m,
                         maximum_depth, maximum_patches, precision_bits):
    """Sufficient symmetric distance bound for an already proved radial source.

    This mathematical helper does NOT issue source topology. Its protected
    caller must first prove exactly one source root on EVERY interior ray.
    A finite oriented closed triangle cycle with strictly positive radial
    face determinants has nonzero degree under radial projection: every
    face contributes positive oriented spherical area. Thus its radial
    image covers the whole sphere. No sampling or seed-completeness claim
    is used. Every face point is bracketed to its unique source ray root.
    Surjectivity then supplies the reverse source-to-mesh bound as well.
    Final mesh topology, intersections, normals and ownership remain gates.
    """
    from collections import Counter
    delta=exact(position_limit_m);scale=exact(metres_per_model_unit)
    if delta<=0 or scale<=0:raise ValueError('PositiveProofScaleAndDistanceRequired')
    if type(maximum_depth) is not int or maximum_depth<0 or type(maximum_patches) is not int or maximum_patches<1:
        raise ValueError('ExplicitPositiveProofBudgetRequired')
    if type(precision_bits) is not int or not 1<=precision_bits<=256:raise ValueError('ExplicitProofPrecisionRequired')
    x=tuple(tuple(exact(v) for v in row) for row in coordinates)
    c=tuple(exact(v) for v in interior)
    if len(c)!=3 or any(len(p)!=3 for p in x):raise ValueError('ThreeCoordinatesRequired')
    f=tuple(tuple(row) for row in faces)
    if any(type(v) is not int for row in f for v in row):raise ValueError('ExactTriangleIndicesRequired')
    if not f or any(len(t)!=3 or len(set(t))!=3 or min(t)<0 or max(t)>=len(x) for t in f):
        return dict(status='Unresolved',reason='RadialTriangleCycleRequired')
    edges=Counter((a,b) for t in f for a,b in zip(t,t[1:]+t[:1]))
    if any(count!=1 or edges[(b,a)]!=1 for (a,b),count in edges.items()):
        return dict(status='Unresolved',reason='ClosedOrientedTriangleCycleRequired')
    # Exact binary64-as-rational determinant: the centre cannot lie on any
    # face, and no radial projection triangle can collapse or reverse.
    for face in f:
        a,b,d=(x[i] for i in face)
        u=tuple(q-p for p,q in zip(a,b));v=tuple(q-p for p,q in zip(a,d))
        n=(u[1]*v[2]-u[2]*v[1],u[2]*v[0]-u[0]*v[2],u[0]*v[1]-u[1]*v[0])
        if sum(ni*(p-ci) for ni,p,ci in zip(n,a,c))<=0:
            return dict(status='Unresolved',reason='PositiveRadialFaceDeterminantRequired')
    processed=certified=0;worst=Q(0)
    for face_index,face in enumerate(f):
        stack=[(tuple(x[i] for i in face),0)]
        while stack:
            patch,depth=stack.pop();processed+=1
            if processed>maximum_patches:
                return dict(status='Unresolved',reason='RadialCoveragePatchBudgetExceeded',patches=processed-1,face_index=face_index)
            box=tuple(Interval(min(p[i] for p in patch),max(p[i] for p in patch)) for i in range(3))
            radial=tuple(b-ci for b,ci in zip(box,c))
            radius=Interval.point(max(sum((p[i]-c[i])**2 for i in range(3)) for p in patch)).sqrt(precision_bits).hi
            travel=delta/radius
            reason='RadialRootBracketUnresolved'
            try:
                def bound_patch():
                    if travel>=1:raise UnresolvedRegion('RadialTubeIncludesInterior')
                    tube=tuple(ci+r*Interval(1-travel,1+travel) for ci,r in zip(c,radial))
                    gradient=region(expression,tube,scale,precision_bits=precision_bits).gradient
                    derivative=sum((g*r for g,r in zip(gradient,radial)),ZERO).lo
                    field=triangle_field(expression,patch,scale,precision_bits)
                    if derivative>0 and field.magnitude<=travel*derivative:
                        return field.magnitude*radius/derivative
                    return None
                bound=filtered_bound(bound_patch)
                if bound is not None:worst=max(worst,bound);certified+=1;continue
            except UnresolvedRegion as error:reason=str(error)
            if depth>=maximum_depth:
                return dict(status='Unresolved',reason=reason,patches=processed,certified_patches=certified,face_index=face_index)
            a,b,d=patch
            ab=tuple((p+q)/2 for p,q in zip(a,b));bd=tuple((p+q)/2 for p,q in zip(b,d));da=tuple((p+q)/2 for p,q in zip(d,a))
            stack.extend((part,depth+1) for part in ((a,ab,da),(ab,b,bd),(da,bd,d),(ab,bd,da)))
    return dict(status='Certified',method='exact_radial_cycle_and_interval_root_brackets',
                patches=processed,certified_patches=certified,faces=len(f),source_empty=False,
                maximum_distance_bound_m=upper_float(worst),distance_bound_exact_m=str(worst),
                proves='Symmetric real-source/mesh distance under the protected one-root-per-ray source premise; full source coverage by nonzero radial degree.',
                does_not_prove='Native equivalence, final manifold topology, intersections, normal accuracy, ownership or prism/cell acceptance.')


def certify_triangle(expression,triangle_m,*,metres_per_model_unit,position_limit_m,
                     normal_limit_rad,maximum_depth,maximum_patches,precision_bits):
    """Normal-fibre existence/distance and gradient-angle bounds over whole faces.

    Continuous piecewise-C1 source fields use an enclosure of every branch
    gradient. Positive directional lower bound gives strict monotonicity along
    each fibre. Subdivision vertices are exact rationals; no gaps or lost area.
    This establishes mesh-to-source position/normal accuracy only. Source-to-
    mesh coverage, topology, features and native evaluation consistency remain
    obligations of the existing qualification owner.
    """
    delta=exact(position_limit_m);normal=exact(normal_limit_rad)
    if delta<=0 or type(maximum_depth) is not int or maximum_depth<0 or type(maximum_patches) is not int or maximum_patches<1:
        raise ValueError('ExplicitPositiveProofBudgetRequired')
    if type(precision_bits) is not int or not 1<=precision_bits<=256:raise ValueError('ExplicitProofPrecisionRequired')
    tri=tuple(tuple(exact(v) for v in row) for row in triangle_m)
    if len(tri)!=3 or any(len(row)!=3 for row in tri):raise ValueError('TriangleRequired')
    u=tuple(b-a for a,b in zip(tri[0],tri[1]));v=tuple(b-a for a,b in zip(tri[0],tri[2]))
    n=(u[1]*v[2]-u[2]*v[1],u[2]*v[0]-u[0]*v[2],u[0]*v[1]-u[1]*v[0]);n2=sum(x*x for x in n)
    if not n2:raise ValueError('DegenerateProofTriangle')
    norm_upper=Interval.point(n2).sqrt(precision_bits).hi
    travel=delta/norm_upper;cosine=cosine_upper(normal,precision_bits)
    if cosine<=0:raise ValueError('AcuteNormalProofLimitRequired')
    stack=[(tri,0)];processed=0;certified=0;worst=Q(0)
    while stack:
        patch,depth=stack.pop();processed+=1
        if processed>maximum_patches:return dict(status='Unresolved',reason='PatchBudgetExceeded',patches=processed-1,certified_patches=certified)
        box=tuple(Interval(min(row[i] for row in patch),max(row[i] for row in patch)) for i in range(3))
        tube=tuple(value+Interval(-abs(n[i])*travel,abs(n[i])*travel) for i,value in enumerate(box))
        try:
            def bound_patch():
                old_field=region(expression,box,metres_per_model_unit,precision_bits=precision_bits).value
                tight_field=triangle_field(expression,patch,metres_per_model_unit,precision_bits)
                field=Interval(max(old_field.lo,tight_field.lo),min(old_field.hi,tight_field.hi))
                gradient=region(expression,tube,metres_per_model_unit,precision_bits=precision_bits).gradient
                directional=sum((g*ni for g,ni in zip(gradient,n)),ZERO).lo
                distance_pass=directional>0 and field.magnitude<=travel*directional
                if not distance_pass:return None
                # Same normal-fibre/root-tube proof and same angle predicate.
                root_travel=field.magnitude/directional
                root_tube=tuple(value+Interval(-abs(n[i])*root_travel,abs(n[i])*root_travel) for i,value in enumerate(box))
                gradient=region(expression,root_tube,metres_per_model_unit,precision_bits=precision_bits).gradient
                gradient2=min(sum(g.square().hi for g in gradient),expression.lipschitz_bound**2)
                if normal_cone_pass(n,gradient,cosine,gradient2):return field.magnitude*norm_upper/directional
                return None
            bound=filtered_bound(bound_patch)
            if bound is not None:worst=max(worst,bound);certified+=1;continue
            reason='NormalFibreOrAngleUnresolved'
        except UnresolvedRegion as error:reason=str(error)
        if depth>=maximum_depth:return dict(status='Unresolved',reason=reason,patches=processed,certified_patches=certified)
        a,b,c=patch
        ab=tuple((x+y)/2 for x,y in zip(a,b));bc=tuple((x+y)/2 for x,y in zip(b,c));ca=tuple((x+y)/2 for x,y in zip(c,a))
        stack.extend((part,depth+1) for part in ((a,ab,ca),(ab,b,bc),(ca,bc,c),(ab,bc,ca)))
    return dict(status='Certified',method='exact_rational_region_normal_fibres',patches=processed,certified_patches=certified,
                position_bound_m=upper_float(worst),normal_bound_rad=upper_float(normal),
                proves='Every face point has a source zero along its normal fibre within the position limit; every enclosed source branch gradient meets the normal-angle limit.',
                does_not_prove='Source coverage, topology, feature ownership, native evaluation consistency or volume-mesh acceptance.')

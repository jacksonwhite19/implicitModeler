"""Conservative real-source directional bounds on one exact affine segment.

No samples, native field values, geometry-family selection or new tolerance.
This is a fallback enclosure, not an acceptance predicate.
"""
from fractions import Fraction as Q
from .regional import Expression,Interval,UnresolvedRegion,exact

def rational_record(value):
    """Exact JSON-safe value without Python's decimal-integer string limit."""
    from .segment_receipts import check_rational_size
    check_rational_size(value)
    return dict(numerator_hex=hex(value.numerator),denominator_hex=hex(value.denominator))

def interval_record(value):
    return dict(lo=rational_record(value.lo),hi=rational_record(value.hi))

def _norm(a,d,lo,hi,bits):
    aa=sum(v*v for v in a);ad=sum(v*w for v,w in zip(a,d));dd=sum(v*v for v in d)
    def squared(t):return aa+2*ad*t+dd*t*t
    ends=(squared(lo),squared(hi));minimum=min(ends)
    if dd>0:
        stationary=max(lo,min(hi,-ad/dd))
        minimum=min(minimum,squared(stationary))
    # Exact sums of squares cannot be negative. Do not clamp an invalid bound.
    if minimum<0:raise UnresolvedRegion('ParametricNegativeSquaredNorm')
    return Interval(minimum,max(ends)).sqrt(bits),ad,dd

def _clamp(value):
    clamp=lambda v:max(Q(0),min(Q(1),v))
    return Interval(clamp(value.lo),clamp(value.hi))

def _value(expression,a,d,lo,hi,scale,bits):
    kind=expression.kind;p=expression.parameters;c=dict(expression.children)
    if kind=='sphere':return _norm(a,d,lo,hi,bits)[0]-p[0]*scale
    if kind=='translate':return _value(c['child'],tuple(x-v*scale for x,v in zip(a,p)),d,lo,hi,scale,bits)
    if kind=='scale':return _value(c['child'],tuple(x/v for x,v in zip(a,p)),tuple(x/v for x,v in zip(d,p)),lo,hi,scale,bits)*min(p)
    if kind=='offset':return _value(c['child'],a,d,lo,hi,scale,bits)-p[0]*scale
    if kind in ('smooth_union','smooth_intersect','smooth_subtract'):
        first=c['base' if kind=='smooth_subtract' else 'a'];second=c['tool' if kind=='smooth_subtract' else 'b']
        av=_value(first,a,d,lo,hi,scale,bits);bv=_value(second,a,d,lo,hi,scale,bits)
        if kind=='smooth_subtract':bv=-bv
        width=p[0]*scale
        weight=_clamp(Q(1,2)+(bv-av if kind=='smooth_union' else av-bv)/(2*width))
        correction=width*weight*(1-weight)
        return bv*(1-weight)+av*weight+(-correction if kind=='smooth_union' else correction)
    raise UnresolvedRegion('UnsupportedParametricSourceOperation:'+kind)

def _progress(expression,a,d,lo,hi,scale,bits):
    kind=expression.kind;p=expression.parameters;c=dict(expression.children)
    if kind=='sphere':
        length,ad,dd=_norm(a,d,lo,hi,bits)
        if length.lo<=0:raise UnresolvedRegion('ParametricSphereDenominatorIncludesZero')
        numerator=Interval(ad+dd*lo,ad+dd*hi)
        return numerator/length
    if kind=='translate':return _progress(c['child'],tuple(x-v*scale for x,v in zip(a,p)),d,lo,hi,scale,bits)
    if kind=='scale':return _progress(c['child'],tuple(x/v for x,v in zip(a,p)),tuple(x/v for x,v in zip(d,p)),lo,hi,scale,bits)*min(p)
    if kind=='offset':return _progress(c['child'],a,d,lo,hi,scale,bits)
    if kind in ('smooth_union','smooth_intersect','smooth_subtract'):
        first=c['base' if kind=='smooth_subtract' else 'a'];second=c['tool' if kind=='smooth_subtract' else 'b']
        av=_value(first,a,d,lo,hi,scale,bits);bv=_value(second,a,d,lo,hi,scale,bits)
        if kind=='smooth_subtract':bv=-bv
        width=p[0]*scale
        weight=_clamp(Q(1,2)+(bv-av if kind=='smooth_union' else av-bv)/(2*width))
        # A certified inactive child need not possess a derivative at its centre.
        if weight.lo==1:return _progress(first,a,d,lo,hi,scale,bits)
        if weight.hi==0:
            result=_progress(second,a,d,lo,hi,scale,bits)
            return -result if kind=='smooth_subtract' else result
        ap=_progress(first,a,d,lo,hi,scale,bits);bp=_progress(second,a,d,lo,hi,scale,bits)
        if kind=='smooth_subtract':bp=-bp
        return Interval(min(w*ap.lo+(1-w)*bp.lo for w in (weight.lo,weight.hi)),
                        max(w*ap.hi+(1-w)*bp.hi for w in (weight.lo,weight.hi)))
    raise UnresolvedRegion('UnsupportedParametricSourceOperation:'+kind)

def bound(expression,start,direction,lo,hi,scale,bits):
    """Enclose d/dt f(start+t*direction) over the complete supplied interval."""
    if type(expression) is not Expression:raise ValueError('ImmutableExpressionRequired')
    a=tuple(exact(v) for v in start);d=tuple(exact(v) for v in direction)
    lo,hi,scale=exact(lo),exact(hi),exact(scale)
    if len(a)!=3 or len(d)!=3 or not 0<=lo<=hi<=1 or scale<=0 or type(bits) is not int or not 1<=bits<=256:
        raise ValueError('ExplicitParametricSegmentInputsRequired')
    return _progress(expression,a,d,lo,hi,scale,bits)


def _anchor_intersection(raw,anchor,derivative,span):
    """FTC enclosure; all three inputs must already be independent raw bounds."""
    candidate=anchor+Interval(Q(0),span)*derivative
    lo=max(raw.lo,candidate.lo);hi=min(raw.hi,candidate.hi)
    if lo>hi:raise UnresolvedRegion('EndpointAnchorDisjointBounds')
    return Interval(lo,hi)


class _AnchorContext:
    """One-call acyclic phases: RAW premises -> anchored values -> progress.

    RAW _value/_progress never reference this context or anchored helpers.
    Frames, global parameter interval, SI scale and precision are cache keys.
    No cache survives a bound invocation or crosses source/arithmetic scopes.
    """
    def __init__(self):
        self.values={};self.counts=dict(anchored_values=0,tightened_values=0,raw_value_fallbacks=0)

    def value(self,expression,a,d,lo,hi,scale,bits):
        key=(expression,a,d,lo,hi,scale,bits)
        if key in self.values:return self.values[key]
        raw=_value(expression,a,d,lo,hi,scale,bits)
        from .source_topology import c1_region
        # a,d are this child's SI coordinate frame. c1_region expects model units.
        box=tuple(Interval(min(x+v*lo,x+v*hi),max(x+v*lo,x+v*hi))/scale for x,v in zip(a,d))
        try:
            if not c1_region(expression,box,bits):
                self.counts['raw_value_fallbacks']+=1;self.values[key]=raw;return raw
            derivative=_progress(expression,a,d,lo,hi,scale,bits)
            anchor=_value(expression,a,d,lo,lo,scale,bits)
        except UnresolvedRegion:
            self.counts['raw_value_fallbacks']+=1;self.values[key]=raw;return raw
        # Disjoint intersection is deliberately outside the premise fallback.
        result=_anchor_intersection(raw,anchor,derivative,hi-lo)
        self.counts['anchored_values']+=1
        self.counts['tightened_values']+=int(result!=raw)
        self.values[key]=result
        return result

    def progress(self,expression,a,d,lo,hi,scale,bits):
        kind=expression.kind;p=expression.parameters;c=dict(expression.children)
        if kind=='sphere':
            length,ad,dd=_norm(a,d,lo,hi,bits)
            if length.lo<=0:raise UnresolvedRegion('ParametricSphereDenominatorIncludesZero')
            numerator=Interval(ad+dd*lo,ad+dd*hi)
            return numerator/length
        if kind=='translate':return self.progress(c['child'],tuple(x-v*scale for x,v in zip(a,p)),d,lo,hi,scale,bits)
        if kind=='scale':return self.progress(c['child'],tuple(x/v for x,v in zip(a,p)),tuple(x/v for x,v in zip(d,p)),lo,hi,scale,bits)*min(p)
        if kind=='offset':return self.progress(c['child'],a,d,lo,hi,scale,bits)
        if kind in ('smooth_union','smooth_intersect','smooth_subtract'):
            first=c['base' if kind=='smooth_subtract' else 'a'];second=c['tool' if kind=='smooth_subtract' else 'b']
            av=self.value(first,a,d,lo,hi,scale,bits);bv=self.value(second,a,d,lo,hi,scale,bits)
            if kind=='smooth_subtract':bv=-bv
            width=p[0]*scale
            weight=_clamp(Q(1,2)+(bv-av if kind=='smooth_union' else av-bv)/(2*width))
            # A certified inactive child need not possess a derivative at its centre.
            if weight.lo==1:return self.progress(first,a,d,lo,hi,scale,bits)
            if weight.hi==0:
                result=self.progress(second,a,d,lo,hi,scale,bits)
                return -result if kind=='smooth_subtract' else result
            ap=self.progress(first,a,d,lo,hi,scale,bits);bp=self.progress(second,a,d,lo,hi,scale,bits)
            if kind=='smooth_subtract':bp=-bp
            return Interval(min(w*ap.lo+(1-w)*bp.lo for w in (weight.lo,weight.hi)),
                            max(w*ap.hi+(1-w)*bp.hi for w in (weight.lo,weight.hi)))
        raise UnresolvedRegion('UnsupportedParametricSourceOperation:'+kind)


def anchored_bound(expression,start,direction,lo,hi,scale,bits,*,provenance=None):
    """Generic stronger fallback with independent RAW endpoint/derivative premises.

    Left anchor only. Optional improvement work is limited to128 AST visits and
    depth32; a larger supported tree retains the original conservative route.
    Existing acceptance budgets, strict gates and raw diagnostic bound are unchanged.
    """
    if type(expression) is not Expression:raise ValueError('ImmutableExpressionRequired')
    a=tuple(exact(v) for v in start);d=tuple(exact(v) for v in direction)
    lo,hi,scale=exact(lo),exact(hi),exact(scale)
    if len(a)!=3 or len(d)!=3 or not 0<=lo<=hi<=1 or scale<=0 or type(bits) is not int or not 1<=bits<=256:
        raise ValueError('ExplicitParametricSegmentInputsRequired')
    work=[(expression,0)];visits=0
    while work:
        node,depth=work.pop();visits+=1
        if visits>128 or depth>32:
            if provenance is not None:provenance.update(method='raw_parametric_optional_anchor_work_limit',recorded_ast_visits=visits)
            return _progress(expression,a,d,lo,hi,scale,bits)
        work.extend((child,depth+1) for role,child in node.children)
    context=_AnchorContext()
    result=context.progress(expression,a,d,lo,hi,scale,bits)
    if provenance is not None:
        provenance.update(method='left_endpoint_anchored_child_values',raw_derivative_basis='unchanged_raw_parametric_value_ranges',anchor_parameter_exact=str(lo),recorded_ast_visits=visits,**context.counts)
    return result

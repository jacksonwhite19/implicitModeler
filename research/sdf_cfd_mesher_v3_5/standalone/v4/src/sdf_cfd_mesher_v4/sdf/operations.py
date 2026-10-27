"""Candidate operations on the existing source-issued immutable expression.

No SDF constructor or source issuer. A mesh acceptance owner must hold its
CanonicalRegionScope and bind these results before and after every operation.
Coverage proves every mathematical source zero lies within the supplied mesh
tolerance, including components omitted by a seed. It grants no topology,
native arithmetic equivalence, cell acceptance or source-feature labels.
"""
from fractions import Fraction as Q
from itertools import product
import heapq
import math
import numpy as np
from sdf_cfd_mesher_v4.sdf.regional import (
    Expression, Interval, Float32Value, UnresolvedRegion, exact, region,
    upper_float, cosine_upper, RealValue, normal_cone_pass)


def budgets(depth, boxes, bits):
    if (type(depth) is not int or depth < 0 or type(boxes) is not int or boxes < 1
            or type(bits) is not int or not 1 <= bits <= 256):
        raise ValueError('ExplicitPositiveProofBudgetRequired')


def value_region(expression, bounds_m, scale, *, precision_bits):
    """Existing arithmetic enclosure includes the real and rounded operations.

    The value-only path can enclose regions containing a primitive centre or
    a zero-length intermediate norm without pretending its gradient exists.
    Rounded intervals are conservative for the real field. They are not used
    as proof that a native implementation has the same continuous semantics.
    """
    if type(expression) is not Expression:
        raise ValueError('ImmutableExpressionRequired')
    scale = exact(scale)
    budgets(0, 1, precision_bits)
    if scale <= 0 or len(bounds_m) != 3:
        raise ValueError('PositiveScaleAndThreeIntervalsRequired')
    points = tuple(Float32Value.constant(v / scale) for v in bounds_m)
    return expression._evaluate(points, precision_bits).value * scale


def coercivity(expression, *, precision_bits):
    """Prove F(p) >= a*|p|-b in model coordinates, with exact a > 0."""
    budgets(0, 1, precision_bits)
    def norm_upper(values):
        return Interval.point(sum(v*v for v in values)).sqrt(precision_bits).hi
    k, p, c = expression.kind, expression.parameters, dict(expression.children)
    if k == 'sphere': return Q(1), p[0]
    if k == 'box': return Q(1), norm_upper(p)
    if k == 'cylinder': return Q(1), norm_upper(p)
    if k == 'torus': return Q(1), sum(p)
    if k in ('translate', 'scale', 'offset'):
        a, b = coercivity(c['child'], precision_bits=precision_bits)
        if k == 'translate': return a, b + a*norm_upper(p)
        if k == 'scale': return a*min(p)/max(p), b*min(p)
        return a, b+p[0]
    if k in ('subtract', 'smooth_subtract'):
        return coercivity(c['a' if k == 'subtract' else 'base'], precision_bits=precision_bits)
    a1, b1 = coercivity(c['a'], precision_bits=precision_bits)
    a2, b2 = coercivity(c['b'], precision_bits=precision_bits)
    if k in ('union', 'smooth_union'):
        return min(a1, a2), max(b1, b2)+(p[0]/4 if k == 'smooth_union' else 0)
    if k in ('intersect', 'smooth_intersect'):
        return min(a1, a2), min(b1, b2)
    raise ValueError('UnsupportedSourceBoundOperation')


def real_value_region(expression, bounds_m, scale, *, precision_bits):
    """Enclose ONLY the declared continuous-real target; same source program.

    Source coefficients stay exact, units stay exact and all interval operations
    are outward. No native coordinate quantization/rounding equivalence is
    claimed. value_region and query_enclosure retain their separate roles.
    """
    if type(expression) is not Expression:raise ValueError('ImmutableExpressionRequired')
    scale=exact(scale);budgets(0,1,precision_bits)
    if scale<=0 or len(bounds_m)!=3:raise ValueError('PositiveScaleAndThreeIntervalsRequired')
    points=tuple(RealValue.constant(v/scale) for v in bounds_m)
    return expression._evaluate(points,precision_bits).value*scale


def positive_real_value_region(expression,bounds_m,scale,*,precision_bits):
    """Strict sign enclosure; protected value filter, original rational fallback.

    Only this opt-in sign consumer uses the fast path. The exact operation and
    derivative/native arithmetic owners keep their existing roles. Bounds may
    differ; both arithmetic strategies enclose the same continuous-real field.
    """
    from sdf_cfd_mesher_v4.sdf.regional import _filtered, FilteredRealValue, ArithmeticFilterUnresolved, exact_fallback
    if type(expression) is not Expression:raise ValueError('ImmutableExpressionRequired')
    scale=exact(scale);budgets(0,1,precision_bits)
    if scale<=0 or len(bounds_m)!=3:raise ValueError('PositiveScaleAndThreeIntervalsRequired')
    state=_filtered.get()
    if state is None or state['exact']:
        return real_value_region(expression,bounds_m,scale,precision_bits=precision_bits)
    state['check']();state['counts']['real_sign_requests']+=1
    result=None
    try:
        # Convert exact coordinates and scale separately through certified
        # coefficient brackets; never replace the exact source unit ratio.
        points=tuple(FilteredRealValue.constant(v)/scale for v in bounds_m)
        result=(expression._evaluate(points,precision_bits)*scale).value
    except (ArithmeticFilterUnresolved,OverflowError,ZeroDivisionError):
        state['counts']['real_sign_arithmetic_fallbacks']+=1
    state['check']()
    if result is not None and (result.lo>0 or result.hi<=0):
        result=result.rational();state['check']()
        state['counts']['real_sign_filtered']+=1
        return result
    state['counts']['real_sign_exact_fallbacks']+=1
    result=exact_fallback(lambda:real_value_region(expression,bounds_m,scale,precision_bits=precision_bits))
    state['check']();return result

def source_box(expression, scale, *, precision_bits):
    a, b = coercivity(expression, precision_bits=precision_bits)
    scale = exact(scale)
    if scale <= 0 or a <= 0:
        raise ValueError('PositiveCoercivityRequired')
    if b < 0:
        return None  # Strict positive lower bound everywhere; no source zero.
    radius = scale*b/a
    return tuple(Interval(-radius, radius) for _ in range(3))


def point(value):
    if len(value) != 3: raise ValueError('ThreeCoordinatesRequired')
    return tuple(exact(v.item() if isinstance(v,np.generic) else v) for v in value)


def norm2(vector): return sum(v*v for v in vector)
def dot(a, b): return sum(x*y for x, y in zip(a, b))
def sub(a, b): return tuple(x-y for x, y in zip(a, b))


def triangle_point(centre, triangle):
    """A point ON the triangle, chosen entirely by exact rational arithmetic.

    Finding a particular valid witness is sufficient for an upper bound. No
    nearest-point claim or floating-point pruning predicate is needed.
    """
    a, b, c = map(point, triangle)
    u, v, w = sub(b, a), sub(c, a), sub(centre, a)
    uu, uv, vv = dot(u, u), dot(u, v), dot(v, v)
    det = uu*vv-uv*uv
    if det <= 0: raise ValueError('DegenerateCoverageTriangle')
    candidates = [a, b, c]
    s, t = (dot(w,u)*vv-dot(w,v)*uv)/det, (dot(w,v)*uu-dot(w,u)*uv)/det
    if s >= 0 and t >= 0 and s+t <= 1:
        candidates.append(tuple(x+s*y+t*z for x,y,z in zip(a,u,v)))
    for left, right in ((a,b),(b,c),(c,a)):
        edge = sub(right,left)
        alpha = max(Q(0),min(Q(1),dot(sub(centre,left),edge)/norm2(edge)))
        candidates.append(tuple(x+alpha*y for x,y in zip(left,edge)))
    return min(candidates,key=lambda q:norm2(sub(centre,q)))


class _CoverageWitnessSearch:
    """Discover candidates for the existing exact whole-box witness predicate.

    The BVH bounds are min/max of actual binary64 triangle coordinates. Capture
    their exact rational values once. For each axis, the least possible maximum
    distance to the two box endpoints is half-width plus distance from the box
    centre to the node interval. The sum of these squares is a lower bound on
    the farthest-corner squared distance for EVERY point in EVERY node triangle.
    A node above delta squared cannot supply this witness. This never excludes
    a source box; a failed search still subdivides or returns Unresolved.
    """
    def __init__(self, triangles):
        from generic_core_sizing import CoreSizingField
        self.triangles = triangles
        self.index = CoreSizingField(np.asarray(triangles,dtype=float),
                                    np.ones(len(triangles)),g=1,h_max=1)
        self.bounds = tuple((point(n.lo),point(n.hi)) for n in self.index.nodes)
        self.denominator = math.lcm(*(v.denominator for pair in self.bounds for row in pair for v in row))
        self.integer_bounds = tuple(tuple(tuple(v.numerator*(self.denominator//v.denominator)
                                                  for v in row) for row in pair) for pair in self.bounds)
        self.work = dict(queries=0,nodes=0,triangles=0)

    def find(self, box, delta_squared):
        self.work['queries'] += 1
        centre = tuple((v.lo+v.hi)/2 for v in box)
        half = tuple((v.hi-v.lo)/2 for v in box)
        denominator=math.lcm(self.denominator,*(v.denominator for v in centre+half))
        factor=denominator//self.denominator
        centre_integer=tuple(v.numerator*(denominator//v.denominator) for v in centre)
        half_integer=tuple(v.numerator*(denominator//v.denominator) for v in half)
        # ONE scale for every heap key, preserving exact rank and ties.
        threshold=delta_squared.numerator*denominator*denominator
        def lower(i):
            lo,hi = self.integer_bounds[i]
            return sum((h+max(a*factor-c,c-b*factor,0))**2
                       for h,c,a,b in zip(half_integer,centre_integer,lo,hi))
        queue = [(lower(0),0)]
        while queue:
            bound,i = heapq.heappop(queue)
            if bound*delta_squared.denominator > threshold:
                break  # All queued exact lower bounds are at least this large.
            node = self.index.nodes[i]
            self.work['nodes'] += 1
            if node.left >= 0:
                for child in (node.left,node.right):
                    candidate = lower(child)
                    if candidate*delta_squared.denominator <= threshold:
                        heapq.heappush(queue,(candidate,child))
                continue
            for triangle in node.ids:
                self.work['triangles'] += 1
                q = triangle_point(centre,self.triangles[int(triangle)])
                # Unchanged acceptance: an actual triangle point's ball contains
                # all corners, hence the complete box by convexity.
                squared = sum(max((v.lo-x)**2,(v.hi-x)**2) for v,x in zip(box,q))
                if squared <= delta_squared:
                    return int(triangle),squared
        return None


def source_coverage(expression, triangles_m, *, metres_per_model_unit,
                    position_limit_m, maximum_depth, maximum_boxes, precision_bits):
    """Cover the entire source-zero enclosure by exact zero-free/near-mesh boxes.

    Every terminal box either excludes zero or lies wholly within the requested
    distance of a proved triangle point. Exact binary subdivision leaves no gap.
    Triangle selection is a performance heuristic only; every distance witness
    is checked exactly. Failed/exhausted covers remain Unresolved.
    """
    budgets(maximum_depth, maximum_boxes, precision_bits)
    delta = exact(position_limit_m)
    if delta <= 0: raise ValueError('PositiveCoverageToleranceRequired')
    triangles = np.asarray(triangles_m, dtype=float)
    if (triangles.ndim != 3 or triangles.shape[1:] != (3,3) or not len(triangles)
            or not np.isfinite(triangles).all()):
        raise ValueError('FiniteCoverageTrianglesRequired')
    triangles = tuple(tuple(point(v) for v in t) for t in triangles)
    search = _CoverageWitnessSearch(triangles)
    box = source_box(expression,metres_per_model_unit,precision_bits=precision_bits)
    if box is None:
        return dict(status='Certified',zero_free_boxes=1,covered_boxes=0,boxes=0,
                    source_empty=True,method='exact_source_box_cover')
    stack = [(box,0)]
    processed = zero_free = covered = 0
    worst = Q(0)
    while stack:
        box, depth = stack.pop()
        processed += 1
        if processed > maximum_boxes:
            return dict(status='Unresolved',reason='SourceCoverageBoxBudgetExceeded',boxes=processed-1,
                        zero_free_boxes=zero_free,covered_boxes=covered,witness_search=dict(search.work))
        try:
            field = real_value_region(expression,box,metres_per_model_unit,precision_bits=precision_bits)
            if field.lo > 0 or field.hi < 0:
                zero_free += 1
                continue
        except UnresolvedRegion:
            # Arithmetic range failure must not discard any possible zero.
            pass
        found = search.find(box,delta*delta)
        if found is not None:
            covered += 1
            worst = max(worst,found[1])
            continue
        if depth >= maximum_depth:
            return dict(status='Unresolved',reason='SourceCoverageDepthExceeded',boxes=processed,
                        zero_free_boxes=zero_free,covered_boxes=covered,
                        unresolved_box_m=[[str(v.lo),str(v.hi)] for v in box],witness_search=dict(search.work))
        axis = max(range(3),key=lambda k:box[k].hi-box[k].lo)
        middle = (box[axis].lo+box[axis].hi)/2
        left, right = list(box), list(box)
        left[axis],right[axis] = Interval(box[axis].lo,middle),Interval(middle,box[axis].hi)
        stack.extend(((tuple(right),depth+1),(tuple(left),depth+1)))
    return dict(status='Certified',boxes=processed,zero_free_boxes=zero_free,covered_boxes=covered,
        source_empty=False,maximum_distance_bound_m=upper_float(Interval.point(worst).sqrt(precision_bits).hi),witness_search=dict(search.work),
        method='exact_source_box_cover',proves='Every mathematical root-field zero is within the position tolerance of an actual mesh triangle.',
        does_not_prove='Mesh topology, source features, native error equivalence, cell quality or prism continuation.')


def normal(expression, location_m, *, metres_per_model_unit, normal_limit_rad, precision_bits, proposed_direction=None, retain_point_region=False):
    """A finite normal estimate with an outward proof of its branch-angle error."""
    p = point(location_m)
    result = region(expression,tuple(Interval.point(v) for v in p),metres_per_model_unit,precision_bits=precision_bits,retain_point=retain_point_region)
    centre = np.array([float((g.lo+g.hi)/2) for g in result.gradient])
    length = np.linalg.norm(centre)
    if not math.isfinite(length) or length <= 0: raise UnresolvedRegion('SourceNormalSingular')
    proposed=centre/length if proposed_direction is None else np.asarray(proposed_direction,dtype=float)
    if proposed.shape!=(3,) or not np.isfinite(proposed).all() or not np.any(proposed):
        raise ValueError('FiniteProposedNormalRequired')
    direction = tuple(exact(float(v)) for v in proposed)
    lower = sum((g*d for g,d in zip(result.gradient,direction)),Interval.point(0)).lo
    g2 = sum(g.square().hi for g in result.gradient)
    cosine = cosine_upper(normal_limit_rad,precision_bits)
    # Existing exact dot OR tangent-cone predicate, now also used for point
    # normals. The angle limit is unchanged; radial interval width need not
    # overwhelm a small tangential uncertainty.
    if cosine <= 0 or lower <= 0 or not normal_cone_pass(direction,result.gradient,cosine,g2):
        raise UnresolvedRegion('SourceNormalConeUnresolved')
    return dict(direction=tuple(float(v) for v in direction),normal_bound_rad=upper_float(exact(normal_limit_rad)))


def project(expression, location_m, *, metres_per_model_unit, projection_tolerance_m,
            maximum_displacement_m, maximum_iterations, precision_bits):
    """Numerical proposals; accepted only with an exact normal-fibre root proof.

    No minimum-distance projector claim. The displacement cap prevents silently
    searching another distant sheet; regularity/coverage remain separate gates.
    """
    start = point(location_m)
    # Every proof uses the binary64 bits returned to mesh consumers, including
    # the very first query. Arbitrary rational inputs grant no hidden precision.
    p = point(tuple(float(v) for v in start))
    delta, cap = exact(projection_tolerance_m), exact(maximum_displacement_m)
    if delta <= 0 or cap < delta or type(maximum_iterations) is not int or maximum_iterations < 1:
        raise ValueError('ExplicitProjectionRequirementsRequired')
    budgets(0,1,precision_bits)
    if norm2(sub(p,start)) > cap*cap:
        raise UnresolvedRegion('ProjectionDisplacementExceeded')
    for iteration in range(maximum_iterations):
        field = region(expression,tuple(Interval.point(v) for v in p),metres_per_model_unit,precision_bits=precision_bits)
        g = tuple((v.lo+v.hi)/2 for v in field.gradient)
        g2 = norm2(g)
        if g2 <= 0: raise UnresolvedRegion('ProjectionGradientSingular')
        length = Interval.point(g2).sqrt(precision_bits).hi
        travel = delta/length
        tube = tuple(Interval(v-abs(d)*travel,v+abs(d)*travel) for v,d in zip(p,g))
        gradient = region(expression,tube,metres_per_model_unit,precision_bits=precision_bits).gradient
        lower = sum((v*d for v,d in zip(gradient,g)),Interval.point(0)).lo
        if lower > 0 and field.value.magnitude <= travel*lower:
            return dict(point_m=tuple(float(v) for v in p),iterations=iteration+1,
                position_bound_m=upper_float(field.value.magnitude*length/lower),
                residual_bound_m=upper_float(field.value.magnitude),method='exact_normal_fibre_projection')
        residual = (field.value.lo+field.value.hi)/2
        # Round proposals to the binary64 coordinates which consumers will use;
        # the NEXT proof encloses those exact bits, not an unmaterialized rational.
        p = tuple(exact(float(v-residual*d/g2)) for v,d in zip(p,g))
        if norm2(sub(p,start)) > cap*cap:
            raise UnresolvedRegion('ProjectionDisplacementExceeded')
    raise UnresolvedRegion('ProjectionIterationBudgetExceeded')


def _segment_progress(expression, box, scale, direction, bits):
    """Bound the same real-source directional derivative without blend cancellation.

    For smooth min/max, differentiating the existing source polynomial cancels
    the dh term exactly; its gradient is the convex combination of child
    gradients. Clamp endpoints give the same child derivative. Values and
    primitive gradients still use the existing source-expression arithmetic.
    This helper is internal to the segment owner, not another acceptance owner.
    """
    kind=expression.kind;parameters=expression.parameters;children=dict(expression.children)
    if kind=='translate':
        shifted=tuple(v-p*scale for v,p in zip(box,parameters))
        return _segment_progress(children['child'],shifted,scale,direction,bits)
    if kind=='scale':
        shifted=tuple(v/p for v,p in zip(box,parameters))
        transformed=tuple(v/p for v,p in zip(direction,parameters))
        return _segment_progress(children['child'],shifted,scale,transformed,bits)*min(parameters)
    if kind=='offset':return _segment_progress(children['child'],box,scale,direction,bits)
    if kind in ('smooth_union','smooth_intersect','smooth_subtract'):
        a=children['base' if kind=='smooth_subtract' else 'a']
        b=children['tool' if kind=='smooth_subtract' else 'b']
        av=real_value_region(a,box,scale,precision_bits=bits)
        bv=real_value_region(b,box,scale,precision_bits=bits)
        if kind=='smooth_subtract':bv=-bv
        width=parameters[0]*scale
        raw=Q(1,2)+(bv-av if kind=='smooth_union' else av-bv)/(2*width)
        clamp=lambda x:max(Q(0),min(Q(1),x))
        weight=Interval(clamp(raw.lo),clamp(raw.hi))
        # Do not require the gradient of an inactive child at its singularity.
        if weight.lo==1:return _segment_progress(a,box,scale,direction,bits)
        if weight.hi==0:
            result=_segment_progress(b,box,scale,direction,bits)
            return -result if kind=='smooth_subtract' else result
        ap=_segment_progress(a,box,scale,direction,bits)
        bp=_segment_progress(b,box,scale,direction,bits)
        if kind=='smooth_subtract':bp=-bp
        # Optimize the shared weight as one variable; treating w and 1-w as
        # independent would needlessly widen the derivative again.
        lower=min(w*ap.lo+(1-w)*bp.lo for w in (weight.lo,weight.hi))
        upper=max(w*ap.hi+(1-w)*bp.hi for w in (weight.lo,weight.hi))
        return Interval(lower,upper)
    gradient=region(expression,box,scale,precision_bits=bits).gradient
    return sum((g*d for g,d in zip(gradient,direction)),Interval.point(0))


def _outward_unresolved(factory):
    failure=UnresolvedRegion('SourceOutwardSegmentUnresolved')
    try:
        from . import segment_diagnostics
        segment_diagnostics.annotate(failure,'attempt',factory)
    except BaseException:
        pass
    return failure


def outward_segment(expression, start_m, end_m, *, metres_per_model_unit, precision_bits,
                    maximum_depth=0, maximum_boxes=1):
    """Prove strict field progress along a vertex segment and positive endpoint.

    This does not certify prism interiors or replace mesh collision/quality
    predicates. A wall point may have the existing finite projection residual.
    """
    budgets(maximum_depth,maximum_boxes,precision_bits)
    a,b = point(start_m),point(end_m)
    d = sub(b,a)
    end = value_region(expression,tuple(Interval.point(v) for v in b),metres_per_model_unit,precision_bits=precision_bits)
    def context(reason,**values):
        from .segment_diagnostics import arithmetic_mode
        return dict(reason=reason,units='m',start_m=a,end_m=b,displacement_m=d,
            metres_per_model_unit=metres_per_model_unit,precision_bits=precision_bits,
            maximum_depth=maximum_depth,maximum_boxes=maximum_boxes,
            arithmetic=arithmetic_mode(),endpoint_bound=dict(lo=end.lo,hi=end.hi),**values)
    if end.lo <= 0:
        raise _outward_unresolved(lambda:context('endpoint_positive_unresolved'))
    # Exact dyadic t intervals partition [0,1]. Every leaf encloses a+d*t,
    # including both joins. Use the ORIGINAL displacement as the derivative
    # with respect to t; never round intermediate endpoints to mesh floats.
    stack=[(Q(0),Q(1),0)]
    processed=leaves=deepest=0;minimum=None;leaf_proofs=[]
    from .parametric_segment import anchored_bound as line_bound,rational_record,interval_record
    import hashlib
    expression_sha256=hashlib.sha256(repr(expression).encode('utf-8')).hexdigest()
    while stack:
        lo,hi,depth=stack.pop()
        if processed >= maximum_boxes:
            raise _outward_unresolved(lambda:context('box_budget_exhausted',
                parameter_interval=[lo,hi],depth=depth,processed=processed,leaves=leaves,accepted_leaf_proofs=leaf_proofs,source_expression_sha256=expression_sha256))
        processed+=1;deepest=max(deepest,depth)
        box=tuple(Interval(min(x+w*lo,x+w*hi),max(x+w*lo,x+w*hi)) for x,w in zip(a,d))
        lower=None
        gradient_attempt=progress_attempt=line_attempt=None;route=None
        try:
            gradient=region(expression,box,metres_per_model_unit,precision_bits=precision_bits).gradient
            bound=sum((v*w for v,w in zip(gradient,d)),Interval.point(0))
            lower=bound.lo;route='gradient_region'
            gradient_attempt=dict(bound=dict(lo=bound.lo,hi=bound.hi))
        except UnresolvedRegion as failure:
            gradient_attempt=dict(unresolved_type=type(failure).__name__,unresolved_args=failure.args)
            pass  # No box may be accepted without its strict progress proof.
        if lower is None or lower<=0:
            try:
                bound=_segment_progress(expression,box,exact(metres_per_model_unit),d,precision_bits)
                lower=bound.lo;route='source_blend_progress'
                progress_attempt=dict(bound=dict(lo=bound.lo,hi=bound.hi))
            except UnresolvedRegion as failure:
                progress_attempt=dict(unresolved_type=type(failure).__name__,unresolved_args=failure.args)
                pass
        if lower is None or lower<=0:
            try:
                anchor_provenance={}
                bound=line_bound(expression,a,d,lo,hi,exact(metres_per_model_unit),precision_bits,provenance=anchor_provenance)
                line_attempt=dict(enclosure_provenance=anchor_provenance,arithmetic='exact_rational_with_outward_sqrt',bound=interval_record(bound))
                if bound.lo>0:lower=bound.lo;route='parametric_rational'
            except UnresolvedRegion as failure:
                line_attempt=dict(arithmetic='exact_rational_with_outward_sqrt',unresolved_args=failure.args)
        if lower is not None and lower > 0:
            leaf_proofs.append(dict(parameter_interval_exact=[str(lo),str(hi)],depth=depth,
                route=route,lower_bound_exact=rational_record(lower),**({'enclosure_provenance':anchor_provenance} if route=='parametric_rational' else {})))
            leaves+=1;minimum=lower if minimum is None else min(minimum,lower)
            continue
        if depth >= maximum_depth:
            raise _outward_unresolved(lambda:context('depth_exhausted_progress_unresolved',
                parameter_interval=[lo,hi],parameter_units='dimensionless t in [0,1]',
                depth=depth,processed=processed,leaves=leaves,box_m=[[v.lo,v.hi] for v in box],
                gradient_attempt=gradient_attempt,segment_progress_attempt=progress_attempt,
                selected_lower=lower,parametric_attempt=line_attempt,accepted_leaf_proofs=leaf_proofs,source_expression_sha256=expression_sha256))
        middle=(lo+hi)/2
        stack.extend(((middle,hi,depth+1),(lo,middle,depth+1)))
    from .segment_receipts import bounded_receipts_active
    hex_minimum=bounded_receipts_active() or any(v['route']=='parametric_rational' for v in leaf_proofs)
    return dict(status='Certified',endpoint_positive=True,strict_field_progress=True,
                method='exact_branch_gradient_segment',segment_boxes=processed,segment_leaves=leaves,
                maximum_depth_used=deepest,
                minimum_progress_lower_exact=(rational_record(minimum) if hex_minimum else str(minimum)),
                minimum_progress_encoding=('rational_hex' if hex_minimum else 'decimal_fraction'),
                source_expression_sha256=expression_sha256,accepted_leaf_proofs=leaf_proofs,
                parametric_role='Fallback exact-rational line enclosure with independent raw-derivative left-endpoint child-value bounds; unchanged strict-positive gate and endpoint obligation',
                does_not_prove='Prism interior, collision, full normal height or global filling.')


class ProtectedQueries:
    """Thin adapter to the existing owner; no new source/custody issuer."""
    def __setattr__(self,name,value):
        if getattr(self,'_sealed',False): raise ValueError('ProtectedGeometryQueries')
        object.__setattr__(self,name,value)

    def __init__(self,scope):
        from sdf_cfd_mesher_v4.sdf.regional_owner import CanonicalRegionScope
        if type(scope) is not CanonicalRegionScope: raise ValueError('RegionalScopeIssuerRequired')
        scope.verify()
        self._scope = scope
        self._sealed = True

    def _run(self,fn,*args,**kwargs):
        scope = self._scope
        scope.verify()
        try:
            expression,scale,budgets,_=scope._batch_inputs()
            if fn is outward_segment:
                kwargs=dict(kwargs,maximum_depth=budgets[0],maximum_boxes=budgets[1])
            def compute():return fn(expression,*args,metres_per_model_unit=scale,
                                    precision_bits=budgets[2],**kwargs)
            try:return compute()
            except UnresolvedRegion as first_failure:
                from sdf_cfd_mesher_v4.sdf.regional import exact_fallback
                try:return exact_fallback(compute)
                except BaseException as final_failure:
                    if fn is outward_segment:
                        try:
                            from . import segment_diagnostics
                            segment_diagnostics.annotate(final_failure,'route',
                                lambda:segment_diagnostics.fallback_context(first_failure,final_failure))
                        except BaseException:
                            pass
                    raise
        except BaseException:
            scope.reject_batch();raise
        finally:
            scope.verify()

    def project(self,point_m,*,projection_tolerance_m,maximum_displacement_m,maximum_iterations):
        return self._run(project,point_m,projection_tolerance_m=projection_tolerance_m,
                         maximum_displacement_m=maximum_displacement_m,maximum_iterations=maximum_iterations)

    def normal(self,point_m,*,normal_limit_rad,proposed_direction=None,retain_point_region=False):
        return self._run(normal,point_m,normal_limit_rad=normal_limit_rad,proposed_direction=proposed_direction,retain_point_region=retain_point_region)

    def outward_segment(self,start_m,end_m):
        return self._run(outward_segment,start_m,end_m)

    def source_coverage(self,triangles_m,*,position_limit_m,maximum_depth,maximum_boxes):
        return self._run(source_coverage,triangles_m,position_limit_m=position_limit_m,
                         maximum_depth=maximum_depth,maximum_boxes=maximum_boxes)

"""Sufficient real-source topology proof; no mesh or admission authority.

Reuse authenticated Expression/region/value/coercivity primitives. This narrow
profile proves a regular closed star-shaped zero set. It rejects unresolved
features, handles and multiple components; it never trusts user topology.
"""
from fractions import Fraction as Q
from sdf_cfd_mesher_v4.sdf.regional import Interval, UnresolvedRegion, region
from sdf_cfd_mesher_v4.sdf.operations import budgets, point, coercivity, value_region


def c1_region(expression, model_box, bits):
    """Conservative C1 premises on a closed region and an open neighbourhood.

    Strict inequalities leave a neighbourhood around every terminal box.
    Hard min/max must select one branch strictly. Positive-width polynomial
    smooth min/max is C1 where its children are C1, including clamp seams.
    Sharp box/cylinder branches remain unsupported by this sufficient profile.
    """
    kind,p,c=expression.kind,expression.parameters,dict(expression.children)
    if kind=='sphere':
        return sum(v.square().lo for v in model_box)>0
    if kind=='torus':
        radial=sum(v.square() for v in model_box[:2])
        if radial.lo<=0:return False
        tube=(radial.sqrt(bits)-p[0]).square()+model_box[2].square()
        return tube.lo>0
    if kind in ('box','cylinder'):return False
    if kind=='translate':
        return c1_region(c['child'],tuple(v-r for v,r in zip(model_box,p)),bits)
    if kind=='scale':
        return c1_region(c['child'],tuple(v/r for v,r in zip(model_box,p)),bits)
    if kind=='offset':return c1_region(c['child'],model_box,bits)
    if kind in ('union','intersect','subtract'):
        a=value_region(c['a'],model_box,1,precision_bits=bits)
        b=value_region(c['b'],model_box,1,precision_bits=bits)
        if kind=='subtract':b=-b
        if kind=='union':
            selected='a' if a.hi<b.lo else 'b' if b.hi<a.lo else None
        else:
            selected='a' if a.lo>b.hi else 'b' if b.lo>a.hi else None
        return selected is not None and c1_region(c[selected],model_box,bits)
    if kind in ('smooth_union','smooth_intersect','smooth_subtract'):
        return all(c1_region(child,model_box,bits) for child in c.values())
    raise ValueError('UnsupportedSourceBoundOperation')


def star_shaped_source(expression, interior_m, *, metres_per_model_unit,
                       maximum_depth, maximum_boxes, precision_bits):
    """Exhaustive source-zero cover + positive radial derivative proves topology.

    A strictly negative interior witness, positive exterior and C1/outward
    crossing at EVERY possible zero imply exactly one root on each ray.
    The implicit function theorem gives a sphere-like connected boundary.
    This proves real-expression topology only, not native field equivalence,
    seed coverage, mesh ownership, surface accuracy or prism acceptance.
    """
    from sdf_cfd_mesher_v4.sdf.regional import exact
    budgets(maximum_depth,maximum_boxes,precision_bits)
    scale=exact(metres_per_model_unit)
    if scale<=0:raise ValueError('PositiveUnitScaleRequired')
    centre=point(interior_m)
    negative=value_region(expression,tuple(Interval.point(v) for v in centre),scale,
                          precision_bits=precision_bits)
    if negative.hi>=0:
        return dict(status='Unresolved',reason='NegativeInteriorWitnessRequired')
    a,b=coercivity(expression,precision_bits=precision_bits)
    if a<=0 or b<=0:raise ValueError('NegativeWitnessCoercivityConflict')
    # |p| >= R gives F(p) >= b*scale > 0. Every possible zero is strictly inside.
    radius=2*b*scale/a
    stack=[((Interval(-radius,radius),)*3,0)]
    processed=free=regular=0;smallest=None
    while stack:
        box,depth=stack.pop();processed+=1
        if processed>maximum_boxes:
            return dict(status='Unresolved',reason='SourceTopologyBoxBudgetExceeded',boxes=processed-1)
        value=value_region(expression,box,scale,precision_bits=precision_bits)
        if value.lo>0 or value.hi<0:free+=1;continue
        reason='SourceC1OrRadialCrossingUnresolved'
        try:
            if c1_region(expression,tuple(v/scale for v in box),precision_bits):
                gradient=region(expression,box,scale,precision_bits=precision_bits).gradient
                radial=sum(((v-x)*g for v,x,g in zip(box,centre,gradient)),Interval.point(0)).lo
                if radial>0:
                    regular+=1;smallest=radial if smallest is None else min(smallest,radial)
                    continue
        except UnresolvedRegion as error:reason=str(error)
        if depth>=maximum_depth:
            return dict(status='Unresolved',reason=reason,boxes=processed,
                        zero_free_boxes=free,regular_zero_boxes=regular)
        axis=max(range(3),key=lambda k:box[k].hi-box[k].lo)
        left=list(box);right=list(box);mid=(box[axis].lo+box[axis].hi)/2
        left[axis]=Interval(box[axis].lo,mid);right[axis]=Interval(mid,box[axis].hi)
        stack.extend(((tuple(left),depth+1),(tuple(right),depth+1)))
    if not regular:raise ValueError('NegativeWitnessWithoutCoveredSourceZero')
    return dict(status='Certified',method='exhaustive_c1_positive_radial_zero_cover',
                topology=dict(components=1,euler=2,boundary_loops=0),
                features='No nonsmooth geometric features under the C1 profile; semantic feature IDs are not inferred.',
                interior_m=[str(v) for v in centre],enclosure_radius_m=str(radius),
                exterior_lower_bound_m=str(b*scale),minimum_radial_dot_bound=str(smallest),
                boxes=processed,zero_free_boxes=free,regular_zero_boxes=regular,
                does_not_prove='Native arithmetic equivalence, source-to-mesh coverage, mesh ownership, surface accuracy or any prism/cell acceptance.')


def protected_star_shaped_source(scope,interior_m,*,maximum_depth,maximum_boxes):
    from sdf_cfd_mesher_v4.sdf.regional_owner import CanonicalRegionScope
    if type(scope) is not CanonicalRegionScope:raise ValueError('RegionalScopeIssuerRequired')
    scope.verify()
    try:
        return star_shaped_source(scope._expression,interior_m,metres_per_model_unit=scope._scale,
            maximum_depth=maximum_depth,maximum_boxes=maximum_boxes,precision_bits=scope._budgets[2])
    finally:scope.verify()

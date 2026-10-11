"""Instance-specific conversion evidence extracted from the existing consistency checker.

No geometry construction or projection. Exact arithmetic over existing source loops.
"""
from fractions import Fraction as F
import numpy as np
from shape_operator_support_envelope import _out, _sqrt, _div

def enclosure(x):
    """Enclose an exact rational by adjacent binary64 numbers; assert inclusion."""
    lo, hi = _out(float(x), float(x))
    assert F(float(lo)) <= x <= F(float(hi))
    return lo, hi


def dot(a, b):
    return sum(x*y for x, y in zip(a, b))


def cross(a, b):
    return [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]]


def exact_error_record(x):
    return dict(exact_rational=str(x), upper_binary64=float(enclosure(x)[1]))


def direction_bound(n,N):
    """Retained cross/dot enclosure, shared by conversion and error composition."""
    nn=dot(n,n); NN=dot(N,N)
    if nn<=0 or NN<=0 or dot(n,N)<=0:return None
    C=cross(n,N);sin2=dot(C,C)/(nn*NN)
    if sin2==0:return 0.
    if sin2>=F(1,4):return None
    angle_upper=float(np.nextafter(2*float(_sqrt(enclosure(sin2))[1]),np.inf))
    assert F(angle_upper)**2>=4*sin2
    return angle_upper


def derive_conversion(a, v, scale):
    physical_unit = F(str(scale))
    mapped = a.unit.points*a.scale
    exact, conversion_l1, binary_unit_l1, unit_errors = {}, [], [], []
    for key, p in zip(a.unit.point_names, mapped):
        i = int(key.split('/')[-2])
        section = v['sections'][0 if key.endswith('/negative') else -1]
        x, z = map(F, v['points'][i])
        c, y = F(section['chord']), F(section['position'][1])
        model = [x*c, y, z*c]
        target = [q*physical_unit for q in model]
        exact[key] = target
        binary_target = [q*F(scale) for q in model]
        conversion_l1.append(sum(abs(F(float(q))-r) for q, r in zip(p, target)))
        binary_unit_l1.append(sum(abs(F(float(q))-r) for q, r in zip(p, binary_target)))
        unit_errors.append(sum(abs(q-r) for q, r in zip(target, binary_target)))
    E = max(conversion_l1)
    # Check that materialized SI cap coordinates retain the already proved
    # convex source order. Exact arithmetic only; no geometric tolerance.
    q = [tuple(F(float(x)) for x in mapped[a.unit.point_names.index('live.section/node/'+str(i)+'/negative')][[0,2]])
         for i in range(v['segments'])]
    area = sum(q[i][0]*q[(i+1)%len(q)][1]-q[(i+1)%len(q)][0]*q[i][1] for i in range(len(q)))
    orientation = 1 if area > 0 else -1
    si_halfplanes = [(q[(i+1)%len(q)][0]-q[i][0])*(q[j][1]-q[i][1])
                     -(q[(i+1)%len(q)][1]-q[i][1])*(q[j][0]-q[i][0])
                     for i in range(len(q)) for j in range(len(q))]
    assert area and all(orientation*x >= 0 for x in si_halfplanes)
    conversion = dict(
        physical_unit='exact rational '+str(physical_unit)+' m per model unit',
        transport_unit_binary64=scale,
        transport_unit_exact=str(F(scale)),
        source_to_materialized_affine_L1_bound_m=exact_error_record(E),
        binary_unit_source_to_materialized_affine_L1_bound_m=exact_error_record(max(binary_unit_l1)),
        decimal_unit_to_binary_unit_L1_bound_m=exact_error_record(max(unit_errors)),
        worst_vertex=a.unit.point_names[conversion_l1.index(E)],
        materialized_SI_exact_halfplane_checks=len(si_halfplanes),materialized_SI_convexity_failures=0,
        domain='corresponding convex finite panels/caps, segments and corners; barycentric Hausdorff bound',
        proof='same convex source loops; norm(sum lambda_i delta_i) <= sum lambda_i norm(delta_i) <= max L1(delta_i)',
        accepted_for_admission=False)

    # Bound the direction of every returned branch normal against exact source
    # planes. This is not a bound on arbitrary binary32 finite differences.
    branch_rows = []
    for ref, name in a.unit.sheet_names.items():
        loop = a.unit.brep.loops[name]
        p0, p1, p2 = [exact[k] for k in loop[:3]]
        N = cross([x-y for x,y in zip(p1,p0)], [x-y for x,y in zip(p2,p0)])
        n, h = a.unit.planes[ref-1]
        n = list(map(lambda x:F(float(x)), n))
        assert dot(N, N) > 0 and dot(n, N) > 0
        angle_upper=direction_bound(n,N)
        assert angle_upper is not None
        rho = max(abs(dot(n,exact[k])-F(float(h))*F(a.scale)) for k in loop)
        distance_upper = (0. if rho == 0 else float(_div(enclosure(rho),_sqrt(enclosure(dot(n,n))))[1]))
        assert F(distance_upper)**2*dot(n,n) >= rho*rho
        branch_rows.append(dict(ref=ref,source=name,normal_angle_upper_rad=angle_upper,
            exact_plane_distance_upper_m=distance_upper,
            meaning='real plane defined by stored binary64 n,h and L; not evaluation-roundoff enclosure'))


    return conversion, branch_rows

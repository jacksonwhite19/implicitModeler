"""Non-generative physical arithmetic and projected surface-extent diagnostics.

Projected extent is a geometric descriptor, not an adopted development length.
No qualification profile becomes a production-case request through this module.
"""
from fractions import Fraction
import math
import numpy as np
from initial_boundary_layer_policy import positive


def mph_to_mps(speed_mph):
    return positive(positive(speed_mph) * 0.44704)


def atmosphere(*, speed_mps, density, viscosity, temperature, gamma, gas_constant):
    u,rho,mu,t,g,r=map(positive,(speed_mps,density,viscosity,temperature,gamma,gas_constant))
    nu=positive(mu/rho)
    sound=positive(math.sqrt(g*r*t))
    return dict(nu=nu,speed_of_sound=sound,mach=positive(u/sound))


def normalize_flow(direction):
    a=np.asarray(direction,dtype=float)
    if a.shape!=(3,) or not np.isfinite(a).all():
        raise ValueError('FiniteThreeComponentFlowRequired')
    scale=float(np.max(np.abs(a)))
    if scale==0:raise ValueError('ZeroFlowVector')
    a=a/scale
    return a/math.hypot(*a)


def projected_extent(coordinates, direction):
    """max(p dot e)-min(p dot e) for represented vertices and normalized flow.

Rational dot products avoid cancellation at large common origins. This cannot
recover precision already lost when input coordinates were translated/rounded.
The result is the sampled surface extent, not an exact analytic SDF extremum.
"""
    points=np.asarray(coordinates,dtype=float)
    if points.ndim!=2 or points.shape[1]!=3 or len(points)==0 or not np.isfinite(points).all():
        raise ValueError('FiniteSurfaceCoordinatesRequired')
    e=normalize_flow(direction)
    ef=[Fraction(float(v)) for v in e]
    values=[sum((Fraction(float(v))*w for v,w in zip(p,ef)),Fraction(0)) for p in points]
    low,high=min(values),max(values)
    return dict(extent_m=float(high-low),extent_rational=str(high-low),
                normalized_flow=e.tolist(),physical_development_length_adopted=False,
                geometry_generation_authorized=False)

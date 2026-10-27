"""Source-backed parametric NASA Ames NACA 0012 rounded-tip authority.

This module defines coordinates and feature roles only. It is not an SDF and
does not authorize production surface generation. The finite TE is retained.
"""
from __future__ import annotations
import dataclasses, math
import numpy as np

INCH = 0.0254
C = 48.0*INCH
Z0 = 33.12*INCH
SEMISPAN = 36.0*INCH
TIP_EXTENT = 2.88*INCH
ALPHA = math.radians(10.0)
PIVOT = np.array([-0.75*C, 0.0, 0.0], dtype=np.float64)

def half_thickness(xi: float) -> float:
    if not 0.0 <= xi <= 1.0:
        raise ValueError("xi outside section")
    return 0.6*(0.2969*math.sqrt(xi)-0.1260*xi-0.3516*xi**2+0.2843*xi**3-0.1015*xi**4)

def rotate(point: np.ndarray) -> np.ndarray:
    q=np.asarray(point,dtype=np.float64)-PIVOT
    ca,sa=math.cos(ALPHA),math.sin(ALPHA)
    return PIVOT + np.array([ca*q[0]-sa*q[1], sa*q[0]+ca*q[1], q[2]])

def section_point(xi: float, upper: bool, z: float=0.0) -> np.ndarray:
    x=-C + xi*C
    return rotate(np.array([x, (1.0 if upper else -1.0)*half_thickness(xi)*C, z]))

def tip_point(xi: float, theta: float) -> np.ndarray:
    x=-C + xi*C; r=half_thickness(xi)*C
    return rotate(np.array([x, r*math.cos(theta), Z0+r*math.sin(theta)]))

def bounds(n=2001):
    xi=np.linspace(0.0,1.0,n)
    r=np.array([half_thickness(float(x)) for x in xi])*C
    return dict(chord_m=C,root_span_m=Z0,overall_semispan_m=SEMISPAN,
                max_tip_extension_m=float(r.max()),max_tip_xi=float(xi[r.argmax()]),
                trailing_edge_half_thickness_m=half_thickness(1.0)*C,
                trailing_edge_thickness_m=2*half_thickness(1.0)*C,
                leading_edge_radius_estimate_m=0.0)

def diagnostic_surface(n_x=96,n_t=24,n_z=12):
    """Return a compact diagnostic triangulation, not production authority."""
    points=[]; faces=[]
    # Upper/lower constant wing sheets and half-revolution tip sheets.
    for xi in np.linspace(0,1,n_x):
        for z in np.linspace(0,Z0,n_z):
            points.extend([section_point(float(xi),True,float(z)),section_point(float(xi),False,float(z))])
    base=len(points)
    for xi in np.linspace(0,1,n_x):
        for th in np.linspace(0,math.pi,n_t): points.append(tip_point(float(xi),float(th)))
    def quad(a,b,c,d): faces.extend([(a,b,c),(a,c,d)])
    for i in range(n_x-1):
        for j in range(n_z-1):
            a=2*(i*n_z+j); b=2*((i+1)*n_z+j); c=b+1; d=a+1; quad(a,b,c,d)
    for i in range(n_x-1):
        for j in range(n_t-1):
            a=base+i*n_t+j; b=base+(i+1)*n_t+j; c=b+1; d=a+1; quad(a,b,c,d)
    return np.asarray(points),np.asarray(faces,dtype=np.int64)

if __name__ == '__main__':
    print(dataclasses.asdict(dataclasses.make_dataclass('Bounds',[])()) if False else bounds())

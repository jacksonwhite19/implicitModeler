"""The frozen wedge polynomial evaluated with exact dyadic integer arithmetic.

Every binary64 coordinate is n / 2**e. A cell-wide power-of-two denominator
therefore makes every polynomial coefficient an integer divided by D**3.
Stationary values use (4*c2*c0-c1*c1)/(4*c2); comparisons cross multiply.
Only the three externally stored rational values need canonical reduction.
The rational implementation in forensics.py remains the independent oracle.
"""
from fractions import Fraction

VERSION='V4ExactDyadicWedge/1'

def _sub(a,b):return (a[0]-b[0],a[1]-b[1],a[2]-b[2])
def _add(a,b):return (a[0]+b[0],a[1]+b[1],a[2]+b[2])
def _cross(a,b):return (a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0])
def _dot(a,b):return a[0]*b[0]+a[1]*b[1]+a[2]*b[2]

def _evaluate(prism):
    if len(prism)!=6 or any(len(v)!=3 for v in prism):raise ValueError('ExpectedSixThreeDimensionalVertices')
    ratios=[float(x).as_integer_ratio() for v in prism for x in v]
    D=max(d for n,d in ratios)
    flat=[n*(D//d) for n,d in ratios];p=[flat[i:i+3] for i in range(0,18,3)]
    a0=_sub(p[1],p[0]);a1=_sub(_sub(p[4],p[3]),a0)
    b0=_sub(p[2],p[0]);b1=_sub(_sub(p[5],p[3]),b0)
    q0=_cross(a0,b0);q1=_add(_cross(a1,b0),_cross(a0,b1));q2=_cross(a1,b1)
    vertical=[_sub(p[j+3],p[j]) for j in range(3)];best=None
    for j,v in enumerate(vertical):
        c0,c1,c2=(_dot(v,q) for q in (q0,q1,q2))
        candidates=[(c0,1,j,0,1),(c0+c1+c2,1,j,1,1)]
        if c2>0 and 0 < -c1 < 2*c2:
            candidates.append((4*c2*c0-c1*c1,4*c2,j,-c1,2*c2))
        for candidate in candidates:
            if best is None or candidate[0]*best[1] < best[0]*candidate[1]:best=candidate
    mean=_add(_add(vertical[0],vertical[1]),vertical[2])
    integ=tuple(6*q0[i]+3*q1[i]+2*q2[i] for i in range(3))
    return best,_dot(mean,integ),D**3

def exact_signs(prism):
    """Exact positivity consumers: no rational materialization or float signs."""
    best,volume,scale=_evaluate(prism);n=best[0]
    return dict(jacobian_sign=(n>0)-(n<0),volume_sign=(volume>0)-(volume<0))

def exact_wedge(prism):
    """Byte-compatible canonical values, corner, height and signs."""
    (n,d,corner,zn,zd),volume,scale=_evaluate(prism)
    return dict(jacobian=str(Fraction(n,d*scale)),jacobian_sign=(n>0)-(n<0),
                minimum_reference_corner=corner,minimum_reference_height=str(Fraction(zn,zd)),
                signed_volume=str(Fraction(volume,36*scale)),volume_sign=(volume>0)-(volume<0))

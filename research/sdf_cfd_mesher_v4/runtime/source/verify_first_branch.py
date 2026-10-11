"""Source-geometry convexity and first-row outward trajectory certificate."""
from study import *
import numpy as np
from fractions import Fraction as F
source=read(HERE/'WING_AUTHORITY_A.json');c=np.array([s['control'] for s in source['sections'][0]['spline']['spans']]);minimum=None
for span in c:
 p=[[F(float(x)) for x in v] for v in span];d=[[3*(p[i+1][k]-p[i][k]) for k in range(2)] for i in range(3)];dd=[[2*(d[i+1][k]-d[i][k]) for k in range(2)] for i in range(2)];b=[F(0) for _ in range(4)]
 for i in range(3):
  for j in range(2):b[i+j]+=F([1,2,1][i],[1,3,3,1][i+j])*(d[i][0]*dd[j][1]-d[i][1]*dd[j][0])
 minimum=min(b) if minimum is None else min(minimum,min(b))
assert minimum>0
d=3*np.diff(c,axis=1);n=len(c)//2
assert np.all(d[:n,:,0]<=0) and np.all(d[n:,:,0]>=0)
assert np.all(c[:n,:,1]>=0) and np.all(c[n:,:,1]<=0)
assert c[0,0,0]==c[-1,3,0] and np.array_equal(c[n-1,3],c[n,0])
te=c[0,0]-c[-1,3];cross=lambda a,b:a[0]*b[1]-a[1]*b[0]
assert cross(d[-1,-1],te)>0 and cross(te,d[0,0])>0
q=read(HERE/'FIRST_ROW_GATE.json');assert q['ordinary']['maximum_first_height_residual']<1e-12
certificate=dict(pass_gate=True,exact_binary64_rational_curvature_numerator_bernstein_minimum=str(minimum),minimum_as_float=float(minimum),all_spans_strictly_convex=True,branches_x_monotone=True,branches_separated_by_sign=True,leading_connection_regular=True,trailing_face_turns_convex=True,proof='The simple closed section has positive turning on every cubic span and both physical trailing corners. Its extrusion is convex. Positive incident supporting-normal displacement keeps each vertex trajectory on the outward side for every positive trajectory parameter. Surface-interior approximation remains governed separately by the continuous geometry certificate.',exact_first_height_residual=q['ordinary']['maximum_first_height_residual'],trajectory_domain='first-row vertex segments from the qualified wall; supporting-normal lower bound h1 up to frozen binary64 equality tolerance',geometric_prism_contact='independent exact-predicate complete one-layer collision audit',roundoff_scope='C2 knot equality and source projection follow the frozen binary64 geometry comparison policy; curvature numerator signs replayed with exact rational arithmetic')
write('FIRST_ROW_BRANCH_CERTIFICATE.json',certificate);q['branch_certificate']='FIRST_ROW_BRANCH_CERTIFICATE.json';q['pass_gate'] &= certificate['pass_gate'];write('FIRST_ROW_GATE.json',q)

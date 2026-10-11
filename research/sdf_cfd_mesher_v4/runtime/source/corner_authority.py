"""Persistent oriented sheet-loop authority, isolated from qualified code.

Topology identities are source node/section ancestry. Numeric table indices are
storage only. This adapter admits planar regular sheets with a finite normal
limit. It does not turn charts of a continuous normal family into finite sheets.
"""
import numpy as np
from collections import defaultdict
def source_points(section):
 """Decode live f32 authority bits, never interpret shortened JSON as f64."""
 return np.asarray(section['point_bits'],dtype=np.uint32).view(np.float32).astype(np.float64)
def restore_persistent_points(authority,projected,incident_curves):
 """Shared source endpoint identity, independent of coordinates/mesh IDs."""
 restored=[]
 for slot,refs in enumerate(incident_curves):
  if len(refs)<2:continue
  sets=[set(authority.point_names[j] for j in authority.curve_vertices[r]) for r in sorted(refs)]
  common=set.intersection(*sets)
  if len(common)!=1:raise ValueError('Curves do not own one exact persistent point')
  key=next(iter(common));expected={tuple(sorted(e)) for e in authority.brep.corners[key]['ordered_curves']}
  actual={tuple(sorted(authority.point_names[j] for j in authority.curve_vertices[r])) for r in refs}
  if actual!=expected:raise ValueError('Incomplete incident corner curve set')
  projected[slot]=authority.brep.points[key];restored.append(key)
 return restored
def unit(v):
 v=np.asarray(v,float);q=float(np.linalg.norm(v));assert q>0;return v/q
def edge_key(a,b):return tuple(sorted((a,b)))
def classify_limits(certificate):
 if not certificate.get('complete',False):return 'GeometryClassUnresolved'
 if certificate.get('continuous_nonconstant_limiting_family',False):return 'ContinuousPointSingularity'
 if not certificate.get('each_sheet_has_unique_limit',False):return 'GeometryClassUnresolved'
 n=np.asarray(certificate['normal_limits'],float)
 if len(n)==1:return 'SmoothSurface'
 if certificate['support_dimension']==1:return 'SharpFeatureEdge'
 return 'FiniteValenceCorner'
class BRep:
 def __init__(self,points,loops):
  self.points={str(k):np.asarray(v,float) for k,v in points.items()};self.loops={str(k):list(v) for k,v in loops.items()}
  self.normals={};self.edges=defaultdict(list);self.incident=defaultdict(list)
  for sheet,loop in self.loops.items():
   assert len(loop)>=3 and len(set(loop))==len(loop)
   q=np.asarray([self.points[k] for k in loop]);normal=unit(sum((np.cross(q[i]-q[0],q[(i+1)%len(q)]-q[0]) for i in range(len(q))),np.zeros(3)))
   assert max(abs((q-q[0])@normal))<1e-12,'nonplanar authority needs its own exact normal certificate'
   self.normals[sheet]=normal
   for a,b in zip(loop,loop[1:]+loop[:1]):self.edges[edge_key(a,b)].append((sheet,a,b))
   for v in loop:self.incident[v].append(sheet)
  for e,uses in self.edges.items():
   assert len(uses)==2,('incomplete/nonmanifold incidence',e,uses)
   assert uses[0][1:]==uses[1][1:][::-1],('orientation mismatch',e)
  self.corners={v:self.corner(v) for v in sorted(self.points)}
 def corner(self,v):
  sheets=self.incident[v];start=min(sheets);current=start;cycle=[];curves=[]
  while current not in cycle:
   cycle.append(current);loop=self.loops[current];j=loop.index(v);e=edge_key(v,loop[(j+1)%len(loop)])
   curves.append(e);other=[x[0] for x in self.edges[e] if x[0]!=current];assert len(other)==1;current=other[0]
  assert current==start and set(cycle)==set(sheets),'disconnected vertex link'
  cert=dict(complete=True,each_sheet_has_unique_limit=True,continuous_nonconstant_limiting_family=False,support_dimension=0,normal_limits=[self.normals[s].tolist() for s in cycle])
  return dict(persistent_point=v,point=self.points[v].tolist(),ordered_sheets=cycle,ordered_curves=[list(e) for e in curves],valence=len(cycle),normal_certificate=cert,geometry_class=classify_limits(cert))
 def curve(self,e,t):
  assert 0<=t<=1;e=edge_key(*e);assert e in self.edges
  a,b=(self.points[k] for k in e);return a*(1-t)+b*t,unit(b-a)
 def export(self):return dict(points={k:v.tolist() for k,v in self.points.items()},oriented_sheet_loops=self.loops,sheet_normals={k:v.tolist() for k,v in self.normals.items()},curves=[dict(endpoints=list(e),incident_sheets=[u[0] for u in uses]) for e,uses in sorted(self.edges.items())],corners=self.corners,counts=dict(points=len(self.points),curves=len(self.edges),sheets=len(self.loops),euler=len(self.points)-len(self.edges)+len(self.loops)))
def orient_convex(points,loops):
 center=np.mean(list(points.values()),axis=0);out={}
 for s,loop in loops.items():
  q=np.array([points[k] for k in loop]);n=sum((np.cross(q[i]-q[0],q[(i+1)%len(q)]-q[0]) for i in range(len(q))),np.zeros(3))
  out[s]=loop if n@(q.mean(axis=0)-center)>0 else loop[::-1]
 return out
def extrusion(profile,depths,ancestry='section',labels=None):
 """Exact polygon extrusion authority. No surface triangulation."""
 if labels is None:labels=[ancestry+'/node/'+str(i) for i in range(len(profile))]
 points={labels[i]+'/'+side:[float(q[0]),float(y),float(q[1])] for side,y in depths.items() for i,q in enumerate(profile)}
 sides=list(depths);assert len(sides)==2
 loops={ancestry+'/cap/'+side:[key+'/'+side for key in labels] for side in sides}
 for i,a in enumerate(labels):
  b=labels[(i+1)%len(labels)];loops[ancestry+'/panel/'+a.split('/')[-1]+'~'+b.split('/')[-1]]=[a+'/'+sides[0],b+'/'+sides[0],b+'/'+sides[1],a+'/'+sides[1]]
 return BRep(points,orient_convex(points,loops))
def height_compatibility(normals):
 n=np.asarray(normals,float);rhs=np.ones(len(n));v=np.linalg.lstsq(n,rhs,rcond=None)[0];res=n@v-rhs
 return dict(displacement_per_height=v.tolist(),normal_height_residual=res.tolist(),max_residual=float(max(abs(res))),rank=int(np.linalg.matrix_rank(n)),classification='SharedFiniteCornerTrajectoryFeasible' if max(abs(res))<1e-12 else 'SharedFiniteCornerTrajectoryInfeasible',proof_scope='Affine incident-sheet normal-height equations only; not prism quality or collision feasibility')

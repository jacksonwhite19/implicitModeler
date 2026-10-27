"""Geometry bridge to the existing generic V3 surface provider and CGAL seed.

All shape coefficients and feature identities come from the live Rhai result.
This module implements the existing Authority interface, not mesh adaptation.
"""
from study import *
import ctypes,math
from functools import lru_cache
from collections import Counter
import numpy as np
sys.path.insert(0,str(V3/'adaptive_surface_generic_qualification'))
from procedural_authorities import Authority,SeedVertex,SeedFace,SeedEdge,tangent_metric,curvature_size,HMAX

class SmoothAuthority(Authority):
 def __init__(self,label='A'):
  self.label=label;self.name='live_spline_extrusion';self.source=read(HERE/f'WING_AUTHORITY_{label}.json');s=self.source['sections'];inv=self.source['inventory']
  assert all(a['spline']==s[0]['spline'] and a['chord']==1 and a['twist']==0 and a['position'][0]==0 and a['position'][2]==0 for a in s)
  self.span=np.array([s[0]['position'][1],s[-1]['position'][1]]);self.ctrl=np.array([p['control'] for p in s[0]['spline']['spans']]);self.N=len(self.ctrl)
  self.sheet_names={i+1:v['sector_id'] for i,v in enumerate(inv['sectors'])};self.curve_names={len(self.sheet_names)+i+1:v['feature_id'] for i,v in enumerate(inv['features'])};self.corner_names={len(self.sheet_names)+len(self.curve_names)+i+1:v['junction_id'] for i,v in enumerate(inv['junctions'])}
  self.srefs={v:k for k,v in self.sheet_names.items()};self.crefs={v:k for k,v in self.curve_names.items()};self.junctions={v['junction_id']:v for v in inv['junctions']}
  self.skin=next(k for k,v in self.sheet_names.items() if v.endswith('.smooth'));self.trailing=next(k for k,v in self.sheet_names.items() if 'trailing_edge.cap_sector' in v)
  self.caps=[next(k for k,v in self.sheet_names.items() if f'tip.{side}.cap_sector' in v) for side in ['negative','positive']]
  self.dll=ctypes.CDLL(str(HERE/'runtime/spline_bridge.dll'));self.dll.spline_load.argtypes=[ctypes.c_char_p,ctypes.c_size_t];self.dll.spline_query.argtypes=[ctypes.c_double,ctypes.c_double,ctypes.POINTER(ctypes.c_double)]
  raw=json.dumps(s[0]['spline']).encode();assert self.dll.spline_load(raw,len(raw))==0
  self.corner_points={}
  for ref,name in self.corner_names.items():
   j=self.junctions[name];upper=j['role']=='trailing_upper_tip';q=self.eval(-1 if upper else 1);self.corner_points[ref]=np.array([q[0],self.span[int(j['span_parameter'])],q[1]])
  # Conservative per-span curvature enclosures from derivative control hulls.
  d=3*np.diff(self.ctrl,axis=1);dd=2*np.diff(d,axis=1)
  dlo=d.min(axis=1);dhi=d.max(axis=1);closest=np.maximum(dlo,np.minimum(dhi,0));speed=np.linalg.norm(closest,axis=1)
  dmax=np.max(np.linalg.norm(d,axis=2),axis=1);ddmax=np.max(np.linalg.norm(dd,axis=2),axis=1)
  assert np.all(speed>0);self.kbound=np.nextafter(dmax*ddmax/speed**3,np.inf)*(1+128*np.finfo(float).eps)
 def eval(self,u):
  a=(float(u)+1)*.5*self.N;i=min(int(a),self.N-1);t=a-i;s=1-t;c=self.ctrl[i]
  return c[0]*s**3+3*c[1]*s*s*t+3*c[2]*s*t*t+c[3]*t**3
 @lru_cache(maxsize=300000)
 def query(self,x,z):
  out=(ctypes.c_double*8)();code=self.dll.spline_query(x,z,out)
  if code:raise ValueError('NativeSplineQueryUnresolved:'+str(code))
  return tuple(out)
 def q(self,p):return np.array(self.query(float(p[0]),float(p[2])))
 def project_sheet(self,ref,p):
  p=np.asarray(p).copy()
  if ref==self.skin:q=self.q(p);return np.array([q[0],p[1],q[1]])
  if ref==self.trailing:p[0]=self.ctrl[0,0,0];return p
  p[1]=self.span[self.caps.index(ref)];return p
 def project_curve(self,ref,p):
  name=self.curve_names[ref];p=np.asarray(p).copy()
  if name.endswith('.contour'):
   p=self.project_sheet(self.skin,p);p[1]=self.span[0 if '.negative.' in name else 1];return p
  if name.endswith('.trailing'):
   a=self.ctrl[0,0];b=self.ctrl[-1,3];e=b-a;t=np.clip((p[[0,2]]-a)@e/(e@e),0,1);q=a+t*e
   return np.array([q[0],self.span[0 if '.negative.' in name else 1],q[1]])
  upper=name.endswith('.upper_cap_boundary');q=self.eval(-1 if upper else 1);return np.array([q[0],np.clip(p[1],*self.span),q[1]])
 def exact_normal(self,ref,p):
  if ref==self.skin:q=self.q(p);return np.array([q[2],0,q[3]])
  if ref==self.trailing:return np.array([1.,0,0])
  return np.array([0.,-1 if ref==self.caps[0] else 1.,0])
 def implicit_value(self,ref,p):
  if ref==self.skin:q=self.q(p);return float((np.array(p)[[0,2]]-q[:2])@q[2:4])
  return float((np.asarray(p)-self.project_sheet(ref,p))@self.exact_normal(ref,p))
 def curvature_upper(self,ref,p):
  if ref!=self.skin:return 0.
  # A conservative complete skin bound avoids unproven nearest-branch exclusion.
  return float(self.kbound.max())
 def gradient_hessian(self,ref,p):
  n=self.exact_normal(ref,p)
  if ref!=self.skin:return n,np.zeros((3,3))
  q=self.q(p);t=np.array([-q[3],0,q[2]]);distance=self.implicit_value(ref,p);k=q[5]/(1+q[5]*distance)
  return n,k*np.outer(t,t)
 def _normal_map_principal_data(self,ref,p):
  n=self.exact_normal(ref,p)
  if ref==self.skin:q=self.q(p);return np.array([-q[3],0,q[2]]),np.array([0.,1,0]),abs(q[5]),0.,n
  a=np.array([1.,0,0]) if abs(n[0])<.9 else np.array([0.,1,0]);return a,np.cross(n,a),0.,0.,n
 def metric(self,v,refs,m):
  ref=self.skin if any(self.curve_names[r].endswith('.contour') for r in refs) else v.sheet_ref
  a,b,k0,k1,n=self._normal_map_principal_data(ref,v.point);return tangent_metric(a,curvature_size(k0,m),b,HMAX,n)
 def seed(self,*args):
  directory=HERE/'surface'/self.label/'seed';directory.mkdir(parents=True,exist_ok=True)
  maxspeed=float(np.linalg.norm(3*np.diff(self.ctrl,axis=1),axis=2).max()*self.N*.5)
  nu=2**math.ceil(math.log2(math.ceil(2*maxspeed/HMAX)));ny=math.ceil(np.diff(self.span)[0]/HMAX)
  us=np.linspace(-1,1,nu+1);ys=np.linspace(*self.span,ny+1);points=[];keys={};faces=[];edges=[];corner_refs={}
  for j,y in enumerate(ys):
   for i,u in enumerate(us):q=self.eval(u);keys[(i,j)]=len(points);points.append(np.array([q[0],y,q[1]]))
  for ref,name in self.corner_names.items():
   v=self.junctions[name];j=0 if v['span_parameter']==0 else ny;i=0 if v['role']=='trailing_upper_tip' else nu;corner_refs[keys[(i,j)]]=ref
  def cdt(uv,boundary,ids,ref):
   src=directory/f'{ref}.input';dst=directory/f'{ref}.output'
   src.write_text(f'V3SECTORCDT1 {len(uv)} {len(boundary)}\n'+'\n'.join(' '.join(format(x,'.17g') for x in p) for p in uv)+'\n'+'\n'.join(f'{a} {b}' for a,b in boundary)+'\n')
   r=linux([wsl(V3/'singular_sector_charts/sector_cdt'),wsl(src),wsl(dst)],f'seed_{self.label}_{ref}',seconds=120)
   if r.returncode:raise RuntimeError('CGALSeed:'+r.stderr[-1000:])
   for line in dst.read_text().splitlines()[1:]:
    f=tuple(ids[int(v)] for v in line.split());tri=np.array([points[v] for v in f]);n=self.exact_normal(ref,tri.mean(axis=0))
    if np.cross(tri[1]-tri[0],tri[2]-tri[0])@n<0:f=f[::-1]
    faces.append(SeedFace(f,ref))
  uv=np.array([[u,y] for y in ys for u in us]);boundary=[]
  for j in [0,ny]:boundary.extend((keys[(i,j)],keys[(i+1,j)]) for i in range(nu))
  for i in [0,nu]:boundary.extend((keys[(i,j)],keys[(i,j+1)]) for j in range(ny))
  cdt(uv,boundary,list(range(len(points))),self.skin)
  for j,ref in [(0,self.caps[0]),(ny,self.caps[1])]:
   ids=[keys[(i,j)] for i in range(nu+1)];uv=np.array([points[i][[0,2]] for i in ids]);cdt(uv,[(i,(i+1)%len(ids)) for i in range(len(ids))],ids,ref)
  ids=[keys[(i,j)] for i in [0,nu] for j in range(ny+1)];uv=np.array([points[i][[1,2]] for i in ids]);loop=list(range(ny+1))+list(range(2*ny+1,ny,-1));cdt(uv,list(zip(loop,loop[1:]+loop[:1])),ids,self.trailing)
  for ref,name in self.curve_names.items():
   if name.endswith('.contour'):j=0 if '.negative.' in name else ny;pairs=[(keys[(i,j)],keys[(i+1,j)]) for i in range(nu)]
   elif name.endswith('.trailing'):j=0 if '.negative.' in name else ny;pairs=[(keys[(0,j)],keys[(nu,j)])]
   else:i=0 if name.endswith('.upper_cap_boundary') else nu;pairs=[(keys[(i,j)],keys[(i,j+1)]) for j in range(ny)]
   edges.extend(SeedEdge(e,ref,True) for e in pairs)
  inc=[set() for _ in points]
  for f in faces:
   for v in f.vertices:inc[v].add(f.sheet_ref)
  vertices=[SeedVertex(p,self.skin if self.skin in inc[i] else min(inc[i]),0,corner_refs.get(i,0)) for i,p in enumerate(points)]
  incidence=Counter(tuple(sorted((f.vertices[a],f.vertices[b]))) for f in faces for a,b in [(0,1),(1,2),(2,0)])
  assert all(v==2 for v in incidence.values());assert all(tuple(sorted(e.vertices)) in incidence for e in edges)
  np.savez(directory/'SEED.npz',points=points,faces=[f.vertices for f in faces],refs=[f.sheet_ref for f in faces])
  write(directory/'SEED_AUDIT.json',dict(points=len(vertices),faces=len(faces),edges=len(incidence),curves=len(self.curve_names),corners=len(corner_refs),complete_closed_production_wall=True,artificial_wall_count=0,span=self.span,source_spline_spans=self.N,profile_seed_intervals=nu,span_seed_intervals=ny,topology_engine='Existing stock CGAL constrained Delaunay chart seed',all_cubic_joins_non_features=True))
  return vertices,faces,edges

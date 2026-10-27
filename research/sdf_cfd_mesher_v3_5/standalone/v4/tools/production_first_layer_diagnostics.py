"""Source-matched first-layer diagnostics; no geometry mutation or new gates."""
import json,time
from pathlib import Path
from fractions import Fraction
import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
import generalize_local_remesh as g
from production_first_layer_run import OUT,SURFACE,save
from production_adaptive_surface import CapsuleGeometry
from sdf_cfd_mesher_v4.conditioning import matrices,gains
from sdf_cfd_mesher_v4.common.authority import identity,array_hash
PERCENT=[0,1,5,50,95,99,100]
def stats(a):
 a=np.asarray(a,float);v=a[np.isfinite(a)]
 return dict(zip(['min','p1','p5','median','p95','p99','max'],map(float,np.percentile(v,PERCENT))))|dict(mean=float(v.mean()),nonfinite=int(a.size-v.size))
def angles(a,b):
 return np.degrees(np.arccos(np.clip(np.sum(a*b,axis=-1)/(np.linalg.norm(a,axis=-1)*np.linalg.norm(b,axis=-1)),-1,1)))
def triangle(x,f):
 p=x[f];l=np.stack([np.linalg.norm(p[:,1]-p[:,2],axis=1),np.linalg.norm(p[:,2]-p[:,0],axis=1),np.linalg.norm(p[:,0]-p[:,1],axis=1)],axis=1)
 area=np.linalg.norm(np.cross(p[:,1]-p[:,0],p[:,2]-p[:,0]),axis=1)/2;s=l.sum(1)/2
 ang=np.degrees(np.arccos(np.clip(np.stack([(l[:,1]**2+l[:,2]**2-l[:,0]**2)/(2*l[:,1]*l[:,2]),(l[:,0]**2+l[:,2]**2-l[:,1]**2)/(2*l[:,0]*l[:,2]),(l[:,0]**2+l[:,1]**2-l[:,2]**2)/(2*l[:,0]*l[:,1])],axis=1),-1,1)))
 ir=area/s;cr=l.prod(1)/(4*area)
 return dict(area=area,min_angle=ang.min(1),max_angle=ang.max(1),edge_ratio=l.max(1)/l.min(1),radius_ratio=2*ir/cr,inradius=ir,circumradius=cr)
def run(label):
 start=time.perf_counter();folder=OUT/label;z=np.load(folder/'CANDIDATE.npz');x=z['wall'];f=z['triangles'];high=z['front'];n=z['direction'];grad=z['gradient'];h=.0001448914516780669
 p,c,o=g.s.construct(x,f,[high],np.ones(len(x),int));owners,own=g.t.ownership(c,o,[tuple(reversed(t)) for t in f])
 edge=np.unique(np.sort(np.concatenate([f[:,[0,1]],f[:,[1,2]],f[:,[2,0]]]),axis=1),axis=0)
 unit=n/np.linalg.norm(n,axis=1)[:,None];gu=grad/np.linalg.norm(grad,axis=1)[:,None];disp=high-x;dot=(disp*unit).sum(1)
 values=dict(euclidean=np.linalg.norm(disp,axis=1),construction_normal=dot,sdf_normal=(disp*gu).sum(1),tangential=np.linalg.norm(disp-dot[:,None]*unit,axis=1),requested_minus_achieved=h-dot,achieved_requested=dot/h)
 wl=np.linalg.norm(x[edge[:,1]]-x[edge[:,0]],axis=1);hl=np.linalg.norm(high[edge[:,1]]-high[edge[:,0]],axis=1)
 wt=triangle(x,f);ht=triangle(high,f)
 cross0=np.cross(x[f[:,1]]-x[f[:,0]],x[f[:,2]]-x[f[:,0]]);cross1=np.cross(high[f[:,1]]-high[f[:,0]],high[f[:,2]]-high[f[:,0]])
 adjacency=coo_matrix((np.ones(len(edge)*2),(np.r_[edge[:,0],edge[:,1]],np.r_[edge[:,1],edge[:,0]])),shape=(len(x),len(x))).tocsr()
 normal=dict(method='Existing analytic production normal-offset rule on loaded authoritative wall',gradient_norm=stats(np.linalg.norm(grad,axis=1)),normal_norm=stats(np.linalg.norm(n,axis=1)),angle_gradient_degrees=stats(angles(n,grad)),fallbacks=0,ambiguous=0,zero_gradient=int(np.sum(np.linalg.norm(grad,axis=1)<=0)),nonfinite=int(np.sum(~np.isfinite(n))),outward_failures=int(np.sum((n*grad).sum(1)<=0)))
 q=json.loads((folder/'RAW_QUALITY.json').read_text())['quality']
 quality={};arrays={}
 for key,rows,gate,sense in [('volume',q['cells'],0,'lower'),('determinant',q['cells'],.0011,'lower'),('aspect',q['cells'],1000,'upper'),('weight',q['faces'],.055,'lower'),('skewness',q['faces'],3.63636,'upper'),('symmetric_volume_ratio',q['faces'],.01,'lower')]:
  a=np.array([r[key] for r in rows]);i=int(np.argmax(a) if sense=='upper' else np.argmin(a));bad=(a>gate if sense=='upper' else a<gate) if key!='volume' else a<=0
  quality[key]=dict(distribution=stats(a),gate=gate,failures=int(bad.sum()),worst=rows[i],margin=float(gate-a[i] if sense=='upper' else a[i]-gate));arrays[key]=a
 jac=np.array([float(Fraction(r['jacobian'])) for r in q['exact']]);quality['exact_jacobian']=dict(distribution=stats(jac),nonpositive=sum(r['jacobian_sign']<=0 for r in q['exact']),worst=q['exact'][int(jac.argmin())]);arrays['jacobian']=jac
 geom=[g.t.geometry(p,cell) for cell in c];sf=np.array([v['areas'] for v in geom]);mask=np.zeros((len(c),5),bool);mask[:,0]=True
 ops=gains(matrices(sf,mask));conditioning={k:stats(v) for k,v in ops.items()};arrays.update(ops)
 geometry=CapsuleGeometry(json.loads((SURFACE.parent/'GEOMETRY.json').read_text())['definition']);hess=np.array([geometry.gradient_hessian(1,pt)[1] for pt in x]);curvature=np.linalg.eigvalsh(hess);curv=np.max(np.abs(curvature),axis=1)
 cq=g.q.legacy_modules()['component_quality'];nonorth=[]
 byid={cell['id']:i for i,cell in enumerate(c)}
 for key,rs in sorted(owners.items()):
  if len(rs)!=2:continue
  (a,face),(b,_)=rs;vs=p[list(face)][None];S,C=cq.FG.face_area_centre_quad(vs);aa=geom[byid[a]];bb=geom[byid[b]]
  vals=cq.face_metrics(vs,S,C,aa['centre'][None],bb['centre'][None],np.array([aa['volume']]),np.array([bb['volume']]))
  nonorth.append(float(vals[3][0]))
 arrays.update({k:v for k,v in values.items()});arrays['edge_ratio']=hl/wl;arrays['curvature']=curvature
 np.savez(folder/'DIAGNOSTIC_ARRAYS.npz',**arrays)
 result=dict(normal=normal,height={k:stats(v) for k,v in values.items()},height_nonpositive=int(np.sum(dot<=0)),requested_h1=h,
  edges={k:stats(v) for k,v in dict(wall=wl,front=hl,ratio=hl/wl,change=hl-wl).items()},
  wall_triangles={k:stats(v) for k,v in wt.items()},front_triangles={k:stats(v) for k,v in ht.items()},area_ratio=stats(ht['area']/wt['area']),triangle_zero_area=int(np.sum(ht['area']<=0)),triangle_reversals=int(np.sum((cross0*cross1).sum(1)<=0)),
  quality=quality,nonorthogonality=stats(nonorth),conditioning=conditioning,
  geometry_diagnostics=dict(principal_curvature=stats(curvature),normal_variation_degrees=stats(angles(n[edge[:,0]],n[edge[:,1]])),requested_height_times_max_curvature=stats(h*curv),achieved_height_times_max_curvature=stats(dot*curv),edge_over_h1=stats(wl/h)),
  topology=dict(wall_vertices=len(x),wall_edges=len(edge),wall_triangles=len(f),front_vertices=len(high),front_edges=len(edge),front_triangles=len(f),prisms=len(c),components=connected_components(adjacency)[0],Euler=len(x)-len(edge)+len(f),ownership=own,
  source_vertex_mappings=len(x),source_edge_mappings=len(edge),source_triangle_mappings=len(f),missing=0,duplicate=0,orphan=0,foreign=0),seconds=time.perf_counter()-start)
 assert all(cell['column']==i and cell['vertices']==list(map(int,np.r_[f[i],f[i]+len(x)])) for i,cell in enumerate(c))
 assert len(set(cell['id'] for cell in c))==len(f)
 assert len(np.unique(f))==len(x)
 save(folder/'DIAGNOSTICS.json',result);print('diagnostics done',result['seconds'],flush=True)
if __name__=='__main__':
 import sys
 run(sys.argv[1])

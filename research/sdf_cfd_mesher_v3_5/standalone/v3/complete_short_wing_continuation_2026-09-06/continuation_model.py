"""Exact cumulative-height parametrization of independent scalar increments.

No new trajectory directions. CasADi proposes; the preserved evaluator accepts.
"""
from study import *
import casadi as ca
from scipy.linalg import null_space

def functions():
 path=V3/'finite_wing_terminal_prism_feasibility_2026-09-06/symbolic_prism.py';source=path.read_text();ns={};exec(compile(source,str(path),'exec'),ns);first,side=ns['make_functions']()
 # The only formula difference is the actual internal-face mask for rows>=2.
 changed=source.replace('coupled=newell[1:];a=sum(norm(s) for s in coupled)/4','coupled=newell;a=sum(norm(s) for s in coupled)/5');assert changed!=source
 ns2={};exec(compile(changed,str(path)+':internal_face_mask','exec'),ns2);internal,_=ns2['make_functions']()
 q=ca.SX.sym('tri',3,3);own=ca.SX.sym('o',3);nei=ca.SX.sym('n',3);ar,fc=ns['area_center']([q[:,i] for i in range(3)]);a=ca.fabs(ca.dot(ar,fc-own));b=ca.fabs(ca.dot(ar,nei-fc));weight=ca.fmin(a,b)/(a+b);delta=nei-own;cp=fc-own;sv=cp-ca.dot(ar,cp)/(ca.dot(ar,delta)+1e-150)*delta;mag=ns['norm'](sv);hat=sv/(mag+1e-150);den=ca.fmax(.2*ns['norm'](delta),ca.mmax(ca.horzcat(*[ca.fabs(ca.dot(q[:,i]-fc,hat)) for i in range(3)])));skew=mag/(den+1e-150)
 return first,internal,side,ca.Function('horizontal',[q,own,nei],[weight,skew])

def build(d,group,mode):
 dep=np.load(HERE/'evidence/R1_DEPENDENCIES.npz');selected=dep[f'g{group}_vertices'];faces=dep[f'g{group}_faces'];edges=dep[f'g{group}_sides'];affected=dep[f'g{group}_affected'];f1=np.load(BASE/'first_row/coupled/FIRST_FRONT.npz')['top'];nom=H1*R**np.arange(20);first,internal,side,horizontal=functions()
 selected=np.array(sorted(selected,key=lambda v:d.vertex_keys[v]));n=len(selected);positions={int(v):i for i,v in enumerate(selected)};localvertices=np.unique(d.f[faces]);h=ca.MX.sym('cumulative_height',n,19);blocks=[];count=0
 if mode=='R2':
  adjacency=[set() for _ in d.p]
  for a,b in d.edges:adjacency[a].add(int(b));adjacency[b].add(int(a))
  for v in selected:
   basis=null_space(d.normals[sorted(d.inc[v])],rcond=1e-11)
   if basis.shape[1]==0:continue
   ring=sorted(adjacency[v]|{int(v)});base=d.p+H1*d.compat;projection=(base[ring]-base[v])@basis;scale=max(H1,float(np.max(np.linalg.norm(d.p[ring]-d.p[v],axis=1))));origin=basis.T@(f1[v]-base[v])/scale
   blocks.append(dict(vertex=int(v),basis=basis,scale=scale,offset=count,lower=projection.min(axis=0)/scale-origin,upper=projection.max(axis=0)/scale-origin));count+=basis.shape[1]
 z=ca.MX.sym('first_front_nullspace',count);points={int(v):ca.DM(f1[v]) for v in localvertices}
 for b in blocks:points[b['vertex']]+=b['scale']*ca.mtimes(ca.DM(b['basis']),z[b['offset']:b['offset']+b['basis'].shape[1]])
 x=ca.vertcat(ca.vec(h),z);loc={int(fi):i for i,fi in enumerate(faces)};pi=ca.horzcat(*[ca.horzcat(*[ca.DM(d.p[v]) for v in d.f[fi]],*[points[int(v)] for v in d.f[fi]]) for fi in faces]);po=first.map(len(faces))(pi);prevcent=po[4]
 wall=d.p[d.f[faces]];normal=np.cross(wall[:,1]-wall[:,0],wall[:,2]-wall[:,0]);js=np.linalg.norm(normal,axis=1)*H1;os=np.einsum('ij,ij->i',normal,normal);quality=[];layer_arrays=[]
 def append_cell(po):
  dt,jj,vv,oo,cc=po;quality.extend([dt.T/DET-1.000001,jj.T/ca.DM(js)-1e-8,vv.T/ca.DM(js/2)-1e-8,oo.T/ca.DM(os)-1e-8])
 def append_sides(lo,hi,cent):
  quad=ca.horzcat(*[ca.horzcat(lo[int(a)],lo[int(b)],hi[int(b)],hi[int(a)]) for a,b in d.edges[edges]]);own=ca.horzcat(*[cent[:,loc[int(d.ef[e,0])]] for e in edges]);nei=ca.horzcat(*[cent[:,loc[int(d.ef[e,1])]] for e in edges]);w,s=side.map(len(edges))(quad,own,nei);quality.extend([w.T/WEIGHT-1.000001,1-s.T/SKEW]);return w,s
 if mode=='R2':
  append_cell(po);append_sides({int(v):ca.DM(d.p[v]) for v in localvertices},points,prevcent)
  for b in blocks:
   v=b['vertex']
   for ref in range(len(d.normals)):
    if ref not in d.inc[v]:quality.append((d.plane_h[ref]+H1-ca.dot(ca.DM(d.normals[ref]),points[v]))/b['scale'])
 lo=points;sum_nom=0
 for k in range(19):
  sum_nom+=nom[k+1]/H1
  hi={int(v):points[int(v)]+H1*d.frozen[v]*(h[positions[int(v)],k] if int(v) in positions else sum_nom) for v in localvertices}
  pi=ca.horzcat(*[ca.horzcat(*[lo[int(v)] for v in d.f[fi]],*[hi[int(v)] for v in d.f[fi]]) for fi in faces]);po=internal.map(len(faces))(pi);append_cell(po);weights,skew=append_sides(lo,hi,po[4]);tri=ca.horzcat(*[ca.horzcat(*[lo[int(v)] for v in d.f[fi]]) for fi in faces]);hw,hs=horizontal.map(len(faces))(tri,prevcent,po[4]);quality.extend([hw.T/WEIGHT-1.000001,1-hs.T/SKEW]);layer_arrays.append((po,weights,skew,hw,hs));prevcent=po[4];lo=hi
 dh=ca.horzcat(h[:,0],h[:,1:]-h[:,:-1]);bounds=ca.vertcat(ca.vec(dh)-1,float(nom[-1]/H1)-ca.vec(dh));g=ca.vertcat(*quality);gx=ca.Function('quality',[x],[g]);inc=ca.Function('increments',[x],[dh]);firstout=ca.Function('first_front',[x],[ca.horzcat(*[points[int(v)] for v in localvertices])]);nomx=np.tile(np.cumsum(nom[1:])/H1,(n,1)).ravel(order='F');initial=np.r_[nomx,np.zeros(count)]
 lower=np.r_[np.repeat(np.arange(1,20),n),np.concatenate([b['lower'] for b in blocks]) if blocks else np.zeros(0)];upper=np.r_[np.repeat(np.arange(1,20)*nom[-1]/H1,n),np.concatenate([b['upper'] for b in blocks]) if blocks else np.zeros(0)]
 return dict(x=x,g=g,bounds=bounds,quality=gx,increments=inc,first_front=firstout,initial=initial,lb=lower,ub=upper,selected=selected,faces=faces,edges=edges,localvertices=localvertices,first_reference=f1,blocks=blocks,scalar_variables=len(initial),independent_increment_variables=n*19,first_front_variables=count)

def reconstruct(d,m,x):
 increments=np.tile(H1*R**np.arange(20),(len(d.p),1));increments[m['selected'],1:]=H1*np.asarray(m['increments'](x));front=m['first_reference'].copy();front[m['localvertices']]=np.asarray(m['first_front'](x)).T
 return front,increments

def independent(d,PM,FG,m,front,increments):
 ns=dict(globals(),PM=PM,FG=FG);ev=function_from(V3/'finite_wing_terminal_prism_feasibility_2026-09-06/layer_continuation.py','evaluate_layer',ns);lo=d.p;hi=front;prev=None;rows=[];aggregate={}
 for k in range(20):
  if k:hi=lo+increments[:,k,None]*d.frozen
  r,g,fail,support=ev(d,lo,hi,prev,first=k==0);local={name:int(np.isin(indices,m['edges'] if name in ['side_weight','skewness'] else m['faces']).sum()) for name,indices in fail.items() if name!='branch'}
  if k:
   ar,fc=FG.face_area_centre_tri(lo[d.f]);hs=FG.internal_face_skewness(lo[d.f],fc,ar,prev,g['centres']);local['horizontal_skewness']=int(sum(hs[m['faces']]>SKEW))
  for name,value in local.items():aggregate[name]=aggregate.get(name,0)+value
  r.pop('failing_face_support');r['local_failures']=local;r['layer']=k+1;rows.append(r);lo=hi;prev=g['centres']
 height=float(np.max(abs(d.evaluate(front)['height']-H1)));bounds=bool(np.all(increments[:,1:]>=H1-1e-15) and np.all(increments[:,1:]<=H1*R**19+1e-15));return dict(local_pass=not any(aggregate.values()) and height<1e-12 and bounds,aggregate_local_failures=aggregate,maximum_first_height_residual_m=height,within_declared_increment_bounds=bounds,layers=rows,full_wing_local_predicates_pass=all(r['all_local_pass'] for r in rows),native_checks_run=False)

def run(mode,group):
 start=time.perf_counter();d,PM,FG=data();out=HERE/'candidates'/mode/str(group);out.mkdir(parents=True,exist_ok=True);write(out/'START.json',dict(mode=mode,group=group,started_epoch=time.time(),purpose='Bounded feasibility proposer; no promotion'))
 m=build(d,group,mode);write(out/'MODEL.json',dict(scalar_variables=m['scalar_variables'],independent_increment_variables=m['independent_increment_variables'],first_front_variables=m['first_front_variables'],guard_faces=len(m['faces']),guard_sides=len(m['edges']),build_seconds=time.perf_counter()-start,exact_reparameterization='Cumulative heights with separate adjacent differences; same independent per-layer increment feasible set.',trajectory_changed=False))
 slack=ca.MX.sym('slack');xx=ca.vertcat(m['x'],slack);constraints=ca.vertcat(m['g']+slack,m['bounds']);solver=ca.nlpsol('solve','ipopt',dict(x=xx,f=slack,g=constraints),{'print_time':False,'ipopt.print_level':0,'ipopt.sb':'yes','ipopt.max_cpu_time':45. if mode=='R2' else 75.,'ipopt.max_iter':350,'ipopt.tol':1e-8,'ipopt.constr_viol_tol':1e-8,'ipopt.bound_relax_factor':0.,'ipopt.mu_strategy':'adaptive','ipopt.hessian_approximation':'limited-memory'})
 starts=[('frozen_nominal',m['initial'])];scaled=m['initial'].copy();factors=np.array([1/min(d.normals[r]@d.frozen[v] for r in d.inc[v]) for v in m['selected']]);inc=np.minimum(H1*R**19,np.maximum(H1,(H1*R**np.arange(1,20))[None,:]*factors[:,None]));scaled[:m['independent_increment_variables']]=np.cumsum(inc/H1,axis=1).ravel(order='F');starts.append(('minimum_sheet_height_scaled',scaled));attempts=[];best=None
 for label,x0 in starts:
  if time.perf_counter()-start>90:break
  gx=np.asarray(m['quality'](x0)).ravel();sl=max(0,-float(gx.min()))+1e-5;sol=solver(x0=np.r_[x0,min(sl,1e6)],lbx=np.r_[m['lb'],0],ubx=np.r_[m['ub'],1e6],lbg=0,ubg=np.inf);x=np.asarray(sol['x']).ravel()[:-1];score=max(0,-float(np.asarray(m['quality'](x)).min()));front,inc=reconstruct(d,m,x);audit=independent(d,PM,FG,m,front,inc);record=dict(start=label,status=solver.stats()['return_status'],maximum_scaled_shortfall=score,independent=audit,elapsed_seconds=time.perf_counter()-start);attempts.append(record)
  np.savez(out/(label+'_DIAGNOSTIC.npz'),first_front=front,increments=inc,x=x);write(out/(label+'_RESULT.json'),record)
  if best is None or (sum(audit['aggregate_local_failures'].values()),score)<best[0]:best=((sum(audit['aggregate_local_failures'].values()),score),front,inc,x,record)
  if audit['local_pass']:break
 if best:
  np.savez(out/'BEST_DIAGNOSTIC.npz',first_front=best[1],increments=best[2],x=best[3]);result=dict(mode=mode,group=group,complete_local_feasible=best[4]['independent']['local_pass'],all_attempts=attempts,seconds=time.perf_counter()-start,first_front_bitwise_preserved=bool(np.array_equal(best[1],m['first_reference'])),promoted=False,global_native_validated=False)
 else:result=dict(mode=mode,group=group,complete_local_feasible=False,reason='No bounded solver attempt',seconds=time.perf_counter()-start)
 write(out/'RESULT.json',result)

if __name__=='__main__':run(sys.argv[1],int(sys.argv[2]))

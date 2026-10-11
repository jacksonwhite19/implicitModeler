"""Component-owned physical compression; no global solver or fallback.

The numeric predicates, increment bounds and IPOPT settings are inherited.
Suffix, slack, variable ordering, retained denominator and ranking are local.
"""
from component_quality import *
import casadi as ca
OPTIONS={'print_time':False,'ipopt.print_level':0,'ipopt.sb':'yes','ipopt.max_cpu_time':25.,'ipopt.max_iter':350,'ipopt.tol':1e-8,'ipopt.constr_viol_tol':1e-8,'ipopt.bound_relax_factor':0.,'ipopt.mu_strategy':'adaptive','ipopt.hessian_approximation':'limited-memory'}
def suffix_start(failure_layers):return max(3,min(failure_layers)-1) if failure_layers else None
def retained_denominator(vertex_area,schedule):return max(float(np.dot(vertex_area,schedule.sum(axis=1))),1e-30)
def initial_slack(quality):return max(0.,-float(np.min(quality)))+1e-5
def scalar_candidates(state,nominal,owner,failures,c):
 """Frozen scalar/taper mathematics on the owned graph, immutable boundary.

 All39scales and20radii are considered. No unrelated owner variable exists.
 Numeric admission includes the full physical contract. Continuous admission
 remains mandatory before publishing any selected numeric candidate.
 """
 from collections import deque
 owned=set(owner['owned_vertices']);core=set(v for r in failures for v in r['vertices']);e,_=FG.unique_manifold_edges(state.faces);nei={v:[] for v in owned}
 for a,b in e:
  if int(a) in owned and int(b) in owned:nei[int(a)].append(int(b));nei[int(b)].append(int(a))
 dist={v:0 for v in core};queue=deque(sorted(core))
 while queue:
  v=queue.popleft()
  if dist[v]>=20:continue
  for w in sorted(nei[v]):
   if w not in dist:dist[w]=dist[v]+1;queue.append(w)
 ids=np.searchsorted(c['vertices'],sorted(owned));dd=np.array([dist.get(v,21) for v in sorted(owned)]);start=suffix_start([r['layer']for r in failures]);base=nominal[c['vertices']].copy();cache={};attempts=[];candidates=[]
 wall=state.wall[state.faces];area=np.linalg.norm(np.cross(wall[:,1]-wall[:,0],wall[:,2]-wall[:,0]),axis=1)/2;va=np.zeros(len(state.wall))
 for k in range(3):np.add.at(va,state.faces[:,k],area/3)
 def trial(radius,scale,keep=False):
  factor=np.ones(len(ids));inside=dd<radius;factor[inside]=scale+(1-scale)*dd[inside]/radius
  key=ah(factor)
  if key in cache and not keep:return (cache[key],None,None,None,None)
  h=base.copy();h[ids,start-1:]=np.minimum(h[ids,start-1:],nominal[np.array(sorted(owned)),start-1:]*factor[:,None])
  x=local_positions(state,h,c['vertices']);g,q=evaluate_positions(x,c);cache[key]=q['pass_gate'];return (q['pass_gate'],h,x,g,q)
 grid=np.linspace(.05,1.,39)
 for radius in range(1,21):
  valid=np.array([trial(radius,float(v))[0] for v in grid]);runs=[];begin=None
  for i,ok in enumerate(valid):
   if ok and begin is None:begin=i
   if begin is not None and (not ok or i==len(valid)-1):runs.append((begin,i if ok else i-1));begin=None
  attempts.append(dict(radius=radius,passing_grid_indices=np.flatnonzero(valid).tolist()))
  if not runs:continue
  begin,end=max(runs,key=lambda p:(grid[p[1]],p[1]-p[0]));lower=float(grid[begin]);upper=float(grid[end])
  if end+1<len(grid):
   lo,hi=upper,float(grid[end+1])
   for _ in range(60):
    mid=.5*(lo+hi)
    if trial(radius,mid)[0]:lo=mid
    else:hi=mid
   upper=lo
  scale=upper-.02*(upper-lower);ok,h,x,g,q=trial(radius,scale,keep=True)
  if ok:
   score=float(np.dot(va[sorted(owned)],h[ids].sum(axis=1)));candidates.append((score,-int((dd<radius).sum()),-radius,h,x,g,q,scale))
 # Cache is local to one immutable component/state evaluation.
 best=max(candidates,key=lambda z:z[:3]) if candidates else None
 return best,dict(radii=20,grid_count=39,unique_evaluations=len(cache),attempts=attempts,candidate_count=len(candidates),immutable_external_context=True,scope='Induced owned-variable source-edge graph',suffix=start)
def build_problem(state,nominal,owner,failures):
 c=context(state.faces,owner['owned_vertices']);vv=c['vertices'];columns=c['columns'];lf=c['faces'];le=c['edges'];pairs=c['owners']
 selected=sorted(set(v for r in failures for v in r['vertices']),key=lambda v:state.vertex_order[v]);assert set(selected).issubset(owner['owned_vertices']),'CompressionDependencyLeavesOwner'
 start=suffix_start([r['layer'] for r in failures]);count=21-start
 assert start>=3 and count>0
 x=state.positions(nominal);cell,side,horizontal,isolated=symbolic_functions();cm=cell.map(len(columns));sm=side.map(len(le));hm=horizontal.map(len(columns));im=isolated.map(len(columns))
 si={v:i for i,v in enumerate(selected)};z=ca.MX.sym('component_factors',len(selected)*count)
 lo=ca.DM(x[start-1,vv].T);previous=ca.DM(FG.prism_cell_centres(np.concatenate((x[start-2,state.faces[columns]],x[start-1,state.faces[columns]]),axis=1))[0].T)
 wall=state.wall[state.faces[columns]];area2=np.linalg.norm(np.cross(wall[:,1]-wall[:,0],wall[:,2]-wall[:,0]),axis=1);js=area2*state.first_height;oscale=area2**2
 # Vertex area has fixed source-surface meaning and no direction-state dependence.
 tris=state.wall[state.faces];ar=np.linalg.norm(np.cross(tris[:,1]-tris[:,0],tris[:,2]-tris[:,0]),axis=1)/2;va=np.zeros(len(state.wall))
 for k in range(3):np.add.at(va,state.faces[:,k],ar/3)
 denominator=retained_denominator(va[owner['owned_vertices']],nominal[owner['owned_vertices']]);quality=[];retained=0
 for j,k in enumerate(range(start,21)):
  field=ca.horzcat(*[z[si[int(v)]*count+j] if int(v) in si else 1. for v in vv])
  hi=lo+ca.DM(state.vectors[k-2,vv].T)*ca.repmat(field*ca.DM(nominal[vv,k-1]).T,3,1)
  pp=ca.horzcat(*[ca.horzcat(lo[:,list(row)],hi[:,list(row)]) for row in lf]);dt,jj,vol,ori,cent=cm(pp)
  quality.extend([dt.T/.0011-1.000001,jj.T/ca.DM(js)-1e-8,vol.T/ca.DM(js/2)-1e-8,ori.T/ca.DM(oscale)-1e-8])
  if k==20:quality.append(im(pp)[0].T/.0011-1.000001)
  quad=ca.horzcat(*[ca.horzcat(lo[:,int(a)],lo[:,int(b)],hi[:,int(b)],hi[:,int(a)]) for a,b in le]);sw,sk=sm(quad,cent[:,pairs[:,0].tolist()],cent[:,pairs[:,1].tolist()])
  tri=ca.horzcat(*[lo[:,list(row)] for row in lf]);hw,hs=hm(tri,previous,cent)
  quality.extend([sw.T/.055-1.000001,1-sk.T/(4/1.1),hw.T/.055-1.000001,1-hs.T/(4/1.1)])
  retained+=sum(va[v]*nominal[v,k-1]*z[si[v]*count+j] for v in selected);previous,lo=cent,hi
 g=ca.vertcat(*quality);gf=ca.Function('component_quality',[z],[g]);lower=np.minimum(1.,.05*nominal[selected,start-1:]/nominal[selected,start-1:]).ravel()
 return dict(context=c,selected=selected,start=start,count=count,z=z,g=g,gf=gf,lower=lower,retained=retained,denominator=denominator,vertex_area=va)
def solve_component(state,nominal,owner,failures):
 started=time.perf_counter();c=context(state.faces,owner['owned_vertices']);baseline_x=state.positions(nominal)[:,c['vertices']];g0,q0=evaluate_positions(baseline_x,c)
 if not failures:
  return nominal[c['vertices']].copy(),baseline_x,g0,dict(pass_gate=q0['pass_gate'],reason='NominalComponentValid' if q0['pass_gate'] else 'UnresolvedUncoveredPhysicalFailure',quality=q0,suffix=None,attempts=[],seconds=time.perf_counter()-started)
 scalar,scalar_record=scalar_candidates(state,nominal,owner,failures,c)
 if scalar is not None:
  score,_,radius,h,x,gg,qq,scale=scalar
  return h,x,gg,dict(pass_gate=True,reason='ComponentScalarNumericCandidate',quality=qq,suffix=scalar_record['suffix'],scalar_phase=scalar_record,final_candidate=dict(radius=-radius,scale=scale),seconds=time.perf_counter()-started)
 model=build_problem(state,nominal,owner,failures);z=model['z'];g=model['g'];gf=model['gf'];low=model['lower'];slack=ca.MX.sym('component_slack')
 feasible=ca.nlpsol('component_feasibility','ipopt',dict(x=ca.vertcat(z,slack),f=slack,g=g+slack),OPTIONS)
 retained=ca.nlpsol('component_retained','ipopt',dict(x=z,f=-model['retained']/model['denominator'],g=g),OPTIONS)
 candidates=[];attempts=[]
 fallback_started=time.perf_counter()
 for label,value in [('nominal',1.),('half',.5)]:
  if time.perf_counter()-fallback_started>90:break
  initial=np.maximum(low,value);sl=initial_slack(np.array(gf(initial)));ss=feasible(x0=np.r_[initial,sl],lbx=np.r_[low,0.],ubx=np.r_[np.ones(z.numel()),1e6],lbg=0.,ubg=np.inf)
  zz=np.array(ss['x']).ravel()[:-1];fs=feasible.stats()['return_status'];rs=None
  if np.isfinite(zz).all() and float(np.array(gf(zz)).min())>=-1e-8:
   rr=retained(x0=zz,lbx=low,ubx=1.,lbg=0.,ubg=np.inf);opt=np.array(rr['x']).ravel();rs=retained.stats()['return_status']
   if float(np.array(gf(opt)).min())>=-1e-8:zz=opt
  shortfall=max(0.,-float(np.array(gf(zz)).min())) if np.isfinite(zz).all() else None
  attempt=dict(start=label,initial_slack=sl,feasibility_status=fs,retained_status=rs,scaled_shortfall=shortfall)
  if shortfall is None or shortfall>1e-8 or not (np.all(zz>=low) and np.all(zz<=1.)):
   attempt['admitted']=False;attempts.append(attempt);continue
  h=nominal.copy();h[model['selected'],model['start']-1:]*=zz.reshape(len(model['selected']),model['count']);x=state.positions(h)[:,c['vertices']];gg,qq=evaluate_positions(x,c)
  attempt.update(admitted=qq['pass_gate'],quality=qq);attempts.append(attempt)
  if qq['pass_gate']:
   score=float(np.dot(model['vertex_area'][owner['owned_vertices']],h[owner['owned_vertices']].sum(axis=1)))
   candidates.append((score,h[c['vertices']].copy(),x,gg,qq,label))
 if not candidates:
  return nominal[c['vertices']].copy(),baseline_x,g0,dict(pass_gate=False,reason='ComponentCompressionSearchFailure',quality=q0,attempts=attempts,suffix=model['start'],selected_vertices=model['selected'],denominator=model['denominator'],rollback=True,scalar_phase=scalar_record,seconds=time.perf_counter()-started)
 best=max(candidates,key=lambda q:q[0]);score,h,x,gg,qq,label=best
 return h,x,gg,dict(pass_gate=True,reason='ComponentCompressionNumericCandidate',quality=qq,attempts=attempts,suffix=model['start'],selected_vertices=model['selected'],denominator=model['denominator'],retained_area_fraction=score/model['denominator'],final_candidate=label,rollback=False,scalar_phase=scalar_record,seconds=time.perf_counter()-started)

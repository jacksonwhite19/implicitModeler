from operator_common import *

def matrices(sf,wall=None):
 area=np.linalg.norm(sf,axis=-1);S=area.sum(-1);N=sf/area[...,None];W=area/S[:,None]
 M=np.einsum('nf,nfi,nfj->nij',area,N,N)
 K=np.linalg.solve(M,N.transpose(0,2,1))
 B=K*S[:,None,None]
 L=K*np.sqrt(area*S[:,None])[:,None,:]
 n=len(sf);eye=np.broadcast_to(np.eye(5),(n,5,5));ones=np.ones((n,5))
 Pc=eye-ones[:,:,None]*ones[:,None,:]/5
 if wall is None:wall=np.zeros((n,5),bool)
 free=(~wall).astype(float)
 Pb=np.zeros((n,5,5));ii=np.arange(5);Pb[:,ii,ii]=free
 Pb-=free[:,:,None]*free[:,None,:]/free.sum(1)[:,None,None]
 c=np.sqrt(W);Pwc=eye-c[:,:,None]*c[:,None,:]
 cb=c*free;Pwb=np.zeros((n,5,5));Pwb[:,ii,ii]=free
 Pwb-=cb[:,:,None]*cb[:,None,:]/np.sum(cb*cb,axis=1)[:,None,None]
 return dict(area=area,S=S,N=N,W=W,M=M,K=K,B=B,L=L,Pc=Pc,Pb=Pb,Pwc=Pwc,Pwb=Pwb,wall=wall)

def gains(m):
 def norm(x):return np.linalg.svd(x,compute_uv=False)[:,0]
 sw=np.linalg.svd(m['N']*np.sqrt(m['W'])[...,None],compute_uv=False)
 su=np.linalg.svd(m['N']/np.sqrt(5),compute_uv=False)
 return dict(G_raw_dim=norm(m['K']),G_raw=norm(m['B']),G_cons=norm(m['B']@m['Pc']),G_bound=norm(m['B']@m['Pb']),
 G_weighted_raw=norm(m['L']),G_weighted_cons=norm(m['L']@m['Pwc']),G_weighted_bound=norm(m['L']@m['Pwb']),
 sigma=sw[:,-1],condition=sw[:,0]/sw[:,-1],equal_sigma=su[:,-1],equal_condition=su[:,0]/su[:,-1])

def native(level,cells,roles=None):
 r,p,g,nat=lp(level)
 cells=np.asarray(cells)
 if roles is None:roles=np.load(Q/level/'faces.npy',mmap_mode='r')[cells]
 own=raw(g/'owner.i64',dtype='<i8');sg=np.where(own[roles]==cells[:,None],1.,-1.)
 sf=raw(p/'handoff/Sf.f64',3)[roles]*sg[...,None]
 patches=r['handoff']['patches'];wall=patches['wall'];mask=(roles>=wall['start'])&(roles<wall['start']+wall['count'])
 return dict(r=r,p=p,g=g,nat=nat,roles=roles,sign=sg,sf=sf,wall=mask,C=raw(p/'handoff/C.f64',3)[cells],V=raw(p/'handoff/V.f64')[cells],
 q=raw(p/'handoff/q.f64')[roles]*sg,U=raw(p/'handoff/U_internal_after.f64',3)[cells])

def arithmetic(actual,expected,cond):
 e=np.max(abs(actual-expected),axis=-1);bound=8192*np.finfo(float).eps*np.maximum(cond,1)*(1+np.max(abs(expected),axis=-1))
 return dict(pass_gate=bool(np.all(e<=bound)),max_abs=float(e.max()),max_bound_ratio=float((e/bound).max()))

def flux_from_triangles(vertices):
 # vertices shape (cells, faces, fan_triangles, 3 triangle vertices, 3 coordinates)
 v=vertices;st=.5*np.cross(v[:,:,:,1]-v[:,:,:,0],v[:,:,:,2]-v[:,:,:,0]);mean=v.mean(-2)
 aa,hh,qu=basis()
 # Integral of a+H*x is exact at each triangle centroid.
 uf=aa[None,None,None,:,:]+np.einsum('kjl,nftl->nftkj',hh,mean)
 qaff=np.einsum('nfti,nftki->nfk',st,uf)
 # Mean of x_j^2 over triangle = ((sum x_j)^2+sum x_j^2)/12.
 second=(v.sum(-2)**2+(v*v).sum(-2))/12
 qquad=np.stack([np.sum(st[...,i]*second[...,j],axis=2)for i,j in qu],axis=-1)
 return np.concatenate([qaff,qquad],axis=-1),st.sum(2)

def native_triangles(z):
 nat=z['nat'];roles=z['roles'];off=raw(nat/'face_offsets.u64',dtype='<u8');fp=raw(nat/'face_points.i32',dtype='<i4');pts=raw(nat/'points.f64',3)
 count=(off[roles+1]-off[roles]).astype(int);assert np.all(np.isin(count,[3,4]))
 ids=fp[off[roles].astype('i8')[...,None]+np.minimum(np.arange(4),count[...,None]-1)]
 v=pts[ids];mid=np.sum(v*(np.arange(4)<count[...,None])[...,None],axis=2)/count[...,None]
 nxt=np.take_along_axis(v,((np.arange(4)+1)%count[...,None])[...,None],axis=2)
 # Fourth fan of a triangle is zero area; avoids changing its representation.
 v=np.where((np.arange(4)<count[...,None])[...,None],v,mid[:,:,None,:]);nxt=np.where((np.arange(4)<count[...,None])[...,None],nxt,mid[:,:,None,:])
 t=np.stack([mid[:,:,None,:]+np.zeros_like(v),v,nxt],axis=3)
 neg=z['sign']<0;t[neg]=t[neg][:,:,[0,2,1],:]
 return t

def exact_values(C):
 aa,hh,qu=basis();aff=aa[None,:,:]+np.einsum('kij,nj->nki',hh,C)
 quad=np.zeros((len(C),6,3))
 for k,(i,j)in enumerate(qu):quad[:,k,i]=C[:,j]**2
 return np.concatenate([aff,quad],axis=1)

def field_test(m,C,V,tri):
 q,sf=flux_from_triangles(tri)
 geomerr=np.max(abs(sf-m['N']*m['area'][...,None]))
 U=np.einsum('nif,nfk->nki',m['K'],q);exact=exact_values(C);e=U-exact;en=np.linalg.norm(e,axis=-1)
 cond=np.linalg.cond(m['M']);control=arithmetic(U[:,:3].reshape(-1,3),exact[:,:3].reshape(-1,3),np.repeat(cond,3));assert control['pass_gate'],control
 weak=np.linalg.svd(m['N']*np.sqrt(m['W'])[...,None],full_matrices=False)[2][:,-1]
 return dict(flux=q,U=U,exact=exact,error_vector=e,error=en,weak_error=np.einsum('nki,ni->nk',e,weak),
 gradient_local_error=en[:,3:11]/np.cbrt(V)[:,None],constant_gate=control,face_vector_max_difference=float(geomerr),
 analytic_conservation_abs=np.max(abs(q.sum(1)),axis=1),weak_direction=weak)

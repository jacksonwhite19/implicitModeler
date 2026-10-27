"""Independent incidence/geometry evaluation on actual complete native meshes."""
from study import *
from displacement_record import load_record
from evaluate import determinant,FG,PM

def bind_layered_native_cells(points, cell_offsets, cell_points, positions, faces):
    """Bind layered authority by exact coordinates and complete cell incidence.

    Native point/cell ordering is not semantic identity. Unrelated native cells
    remain unassigned; missing, duplicate or ambiguous authority fails closed.
    """
    points=np.ascontiguousarray(points,dtype='<f8')
    positions=np.ascontiguousarray(positions,dtype='<f8')
    faces=np.asarray(faces,dtype='i8');off=np.asarray(cell_offsets,dtype='i8')
    cp=np.asarray(cell_points,dtype='i8')
    assert positions.ndim==3 and positions.shape[2]==3 and faces.shape[1]==3
    assert np.isfinite(points).all() and np.isfinite(positions).all()
    dtype=np.dtype([('x','<f8'),('y','<f8'),('z','<f8')])
    order=np.lexsort(points.T[::-1]);native=points[order].view(dtype).ravel()
    assert np.all(native[1:]!=native[:-1]),'AmbiguousNativePointIdentity'
    wanted=positions.reshape(-1,3).view(dtype).ravel()
    j=np.searchsorted(native,wanted);assert np.all(j<len(native)),'MissingAuthorityPoint'
    mapped=order[j]
    assert np.array_equal(points[mapped].view('u8'),positions.reshape(-1,3).view('u8')),'MovedAuthorityPoint'
    mapped=mapped.reshape(positions.shape[:2])
    def keys(a):
        a=np.ascontiguousarray(np.sort(a.astype('<i8'),axis=1))
        return a.view(np.dtype((np.void,a.shape[1]*8))).ravel()
    ids=np.flatnonzero(np.diff(off)==6)
    actual=keys(cp[off[ids,None]+np.arange(6)]);sort=np.argsort(actual)
    assert np.all(actual[sort][1:]!=actual[sort][:-1]),'DuplicateNativeCell'
    expected=keys(np.concatenate([mapped[:-1,faces],mapped[1:,faces]],axis=2).reshape(-1,6))
    j=np.searchsorted(actual[sort],expected);assert np.all(j<len(ids)),'MissingLayeredCell'
    assert np.array_equal(actual[sort[j]],expected),'ChangedLayeredCell'
    cells=ids[sort[j]];assert len(np.unique(cells))==len(cells),'AmbiguousLayeredCell'
    layer=np.zeros(len(off)-1,'i2');ancestor=np.full(len(off)-1,-1,'i8')
    layer[cells]=np.repeat(np.arange(1,len(positions)),len(faces))
    ancestor[cells]=np.tile(np.arange(len(faces)),len(positions)-1)
    return dict(native_cells=cells,layer=layer,face_ancestor=ancestor,point_to_native=mapped)

def raw(p,n,t):return np.fromfile(p/n,dtype=t)
def mesh(p):
 a={name:raw(p,name+'.'+ext,dtype) for name,ext,dtype in [('points','f64','<f8'),('face_offsets','u64','<u8'),('face_points','i32','<i4'),('owner','i32','<i4'),('neighbour','i32','<i4'),('cell_centres','f64','<f8'),('cell_volumes','f64','<f8'),('cell_determinants','f64','<f8'),('face_centres','f64','<f8'),('face_areas','f64','<f8'),('face_weights','f64','<f8'),('face_skewness','f64','<f8'),('internal_mask','u8','u1'),('point_ancestor','i32','<i4'),('point_level','i32','<i4'),('cell_ancestor','i32','<i4'),('cell_level','i32','<i4')]}
 for name in ['points','cell_centres','face_centres','face_areas']:a[name]=a[name].reshape(-1,3)
 a['faces']=[a['face_points'][int(a['face_offsets'][i]):int(a['face_offsets'][i+1])] for i in range(len(a['owner']))]
 return a

def generic_geometry(a):
 p=a['points'];faces=a['faces'];owner=a['owner'];nei=a['neighbour'];N=len(a['cell_volumes']);nf=len(faces);fc=np.zeros((nf,3));fa=fc.copy()
 for size in [3,4]:
  ids=np.array([i for i,f in enumerate(faces) if len(f)==size]);q=np.array([p[faces[i]] for i in ids]);aa,cc=(FG.face_area_centre_tri if size==3 else FG.face_area_centre_quad)(q);fa[ids]=aa;fc[ids]=cc
 incident=[[] for _ in range(N)]
 for i,o in enumerate(owner):incident[o].append((i,1))
 for i,n in enumerate(nei):incident[n].append((i,-1))
 cc=np.zeros((N,3));vol=np.zeros(N);det=np.zeros(N)
 for c,items in enumerate(incident):
  ids=np.array([i for i,s in items]);sign=np.array([s for i,s in items]);est=fc[ids].mean(axis=0);pyr3=np.einsum('ij,ij->i',fa[ids]*sign[:,None],fc[ids]-est);vol[c]=sum(pyr3)/3;cc[c]=np.sum(pyr3[:,None]*(.75*fc[ids]+.25*est),axis=0)/sum(pyr3)
  mask=a['internal_mask'][ids].astype(bool);vv=fa[ids[mask]];avg=np.linalg.norm(vv,axis=1).mean() if len(vv) else 0
  det[c]=0 if avg==0 else PM._det_symm((vv/avg).T@(vv/avg))
 w=np.zeros(len(nei));skew=np.zeros(nf)
 for size in [3,4]:
  ids=np.array([i for i,f in enumerate(faces) if len(f)==size]);ii=ids[ids<len(nei)];bb=ids[ids>=len(nei)]
  if len(ii):
   own=cc[owner[ii]];ng=cc[nei[ii]];aa=abs(np.einsum('ij,ij->i',fa[ii],fc[ii]-own));ab=abs(np.einsum('ij,ij->i',fa[ii],ng-fc[ii]));w[ii]=np.minimum(aa,ab)/(aa+ab);skew[ii]=FG.internal_face_skewness(np.array([p[faces[i]] for i in ii]),fc[ii],fa[ii],own,ng)
  if len(bb):
   cpf=fc[bb]-cc[owner[bb]];n=fa[bb]/(np.linalg.norm(fa[bb],axis=1)[:,None]+1e-150);d=n*np.einsum('ij,ij->i',n,cpf)[:,None];sv=cpf-(np.einsum('ij,ij->i',fa[bb],cpf)/(np.einsum('ij,ij->i',fa[bb],d)+1e-150))[:,None]*d;sm=np.linalg.norm(sv,axis=1);sh=sv/(sm[:,None]+1e-150);fd=.4*np.linalg.norm(d,axis=1)+1e-150;extent=np.max(abs(np.einsum('nij,nj->ni',np.array([p[faces[i]] for i in bb])-fc[bb,None,:],sh)),axis=1);skew[bb]=sm/np.maximum(fd,extent)
 return dict(face_centres=fc,face_areas=fa,cell_centres=cc,cell_volumes=vol,cell_determinants=det,face_weights=w,face_skewness=skew)

def verify_prefix(record,path,k,previous=None):
 m,r=load_record(record);a=mesh(path);p=a['points'];anc=a['point_ancestor'];lev=a['point_level'];V=r['positions'].shape[1];F=len(r['faces']);E=3*F//2
 assert len(p)==(k+1)*V and len(a['cell_volumes'])==k*F
 assert len(a['neighbour'])==k*E+(k-1)*F and len(a['owner'])==k*E+(k+1)*F
 assert np.array_equal(a['internal_mask'],np.arange(len(a['owner']))<len(a['neighbour']))
 assert len(set(zip(lev,anc)))==len(p) and set(lev)==set(range(k+1))
 x=np.empty((k+1,V,3));x[lev,anc]=p;difference=abs(x-r['positions'][:k+1]);bound=r['arithmetic_bound'][None,:,:]
 assert (difference<=bound).all(),('CoordinateRoundoffBound',float(difference.max()),float(bound.max()))
 if previous is not None:assert np.array_equal(x[:-1].view(np.uint64),previous.view(np.uint64)),'PreviouslyCommittedPointMoved'
 g=generic_geometry(a);tol=read(HERE/'FROZEN_CONTRACT.json')['formula_tolerances'];errors={}
 keys={'face_centres':'centres_absolute_m','face_areas':'face_area_absolute_m2','cell_centres':'centres_absolute_m','cell_volumes':'volume_absolute_m3','cell_determinants':'determinant_absolute','face_weights':'weight_absolute','face_skewness':'skewness_absolute'}
 for key,tk in keys.items():
  err=abs(a[key]-g[key]);errors[key]=float(err.max()) if len(err) else 0
  assert np.allclose(a[key],g[key],atol=tol[tk],rtol=tol['relative']),(key,errors[key])
 v3det=np.empty(k*F);v3centres=np.empty((k*F,3));v3weights={};v3skew={};edges,ef=FG.unique_manifold_edges(r['faces'])
 for layer in range(1,k+1):
  prisms=np.concatenate([x[layer-1,r['faces']],x[layer,r['faces']]],axis=1);dd=determinant(prisms,layer>1,layer<k);lg=FG.layer_geometry(x[layer-1],r['faces'],x[layer]-x[layer-1],np.zeros(V),np.ones(V),edges,ef)
  ids=np.flatnonzero(a['cell_level']==layer);v3det[ids]=dd[a['cell_ancestor'][ids]];v3centres[ids]=lg['centres'][a['cell_ancestor'][ids]]
  for i,e in enumerate(edges):
   key=tuple(sorted([(layer-1,int(e[0])),(layer-1,int(e[1])),(layer,int(e[0])),(layer,int(e[1]))]));v3weights[key]=lg['side_weights'][i];v3skew[key]=lg['side_skewness'][i]
  if layer>1:
   fa,fc=FG.face_area_centre_tri(x[layer-1,r['faces']]);pa,pv=FG.prism_cell_centres(np.concatenate([x[layer-2,r['faces']],x[layer-1,r['faces']]],axis=1));ca=lg['centres'];aa=abs(np.einsum('ij,ij->i',fa,fc-pa));ab=abs(np.einsum('ij,ij->i',fa,ca-fc));ww=np.minimum(aa,ab)/(aa+ab);ss=FG.internal_face_skewness(x[layer-1,r['faces']],fc,fa,pa,ca)
   for i,f in enumerate(r['faces']):
    key=tuple(sorted((layer-1,int(v)) for v in f));v3weights[key]=ww[i];v3skew[key]=ss[i]
 for key,nativeval,proposed,tt in [('v3_determinant',a['cell_determinants'],v3det,tol['determinant_absolute']),('v3_centres',a['cell_centres'],v3centres,tol['centres_absolute_m'])]:
  errors[key]=float(abs(nativeval-proposed).max());assert np.allclose(nativeval,proposed,atol=tt,rtol=tol['relative']),(key,errors[key])
 wf=[];sf=[]
 for face in a['faces'][:len(a['neighbour'])]:
  key=tuple(sorted((int(lev[i]),int(anc[i])) for i in face));assert key in v3weights,'MissingPhysicalNeighbour';wf.append(v3weights[key]);sf.append(v3skew[key])
 errors['v3_weights']=float(abs(a['face_weights']-wf).max());errors['v3_skewness']=float(abs(a['face_skewness'][:len(wf)]-sf).max())
 assert np.allclose(a['face_weights'],wf,atol=tol['weight_absolute'],rtol=tol['relative']);assert np.allclose(a['face_skewness'][:len(wf)],sf,atol=tol['skewness_absolute'],rtol=tol['relative'])
 # Check physical boundary identities, never labels selected by quality.
 boundary=[]
 for f in a['faces'][len(a['neighbour']):]:
  ll=set(lev[f]);assert ll in [{0},{k}];assert len(f)==3;boundary.append(next(iter(ll)))
 assert boundary.count(0)==F and boundary.count(k)==F
 result=dict(prefix=k,physical_cells=k*F,carrier_cells=0,points=len(p),coordinate_max_abs_m=float(difference.max()),coordinate_bound_max_m=float(bound.max()),serialization_loss=0,prior_points_bitwise_fixed=True,complete_physical_ownership=True,native_independent_and_prospective_agree=True,errors=errors,native_determinant_failures=int(sum(a['cell_determinants']<DET)),native_minimum_determinant=float(a['cell_determinants'].min()),native_minimum_weight=float(a['face_weights'].min()),native_maximum_skewness=float(a['face_skewness'].max()),native_minimum_volume=float(a['cell_volumes'].min()))
 return result,x

def verify_case(record,case):
 reports=[];previous=None
 for k in range(1,21):
  q,previous=verify_prefix(record,case/'postProcessing/stagedLayer'/f'prefix_{k}',k,previous);reports.append(q)
 return dict(pass_representation=True,pass_equivalence=True,prefixes=reports,maximum_coordinate_error_m=max(q['coordinate_max_abs_m'] for q in reports),maximum_metric_errors={key:max(q['errors'][key] for q in reports) for key in reports[0]['errors']},materialized_physical_cells=reports[-1]['physical_cells'],numerical_carrier_cells=0)

from study import *
PM,FG=predicates()

def determinant(prisms,lower_internal,upper_internal):
 vectors=PM._face_vectors(prisms);mask=np.ones(5);mask[0]=lower_internal;mask[1]=upper_internal
 avg=np.sum(np.linalg.norm(vectors,axis=2)*mask,axis=1)/sum(mask)
 scaled=vectors/avg[:,None,None];tensor=np.einsum('nfi,nfj,f->nij',scaled,scaled,mask)
 return PM._det_symm(tensor)

def layer_arrays(lo,hi,faces,edges,ef,lower_internal,upper_internal):
 p=np.concatenate([lo[faces],hi[faces]],axis=1);d=determinant(p,lower_internal,upper_internal);j,v=PM._exact_wedge_jacobian_and_volume(p)
 g=FG.layer_geometry(lo,faces,hi-lo,np.zeros(len(lo)),np.ones(len(lo)),edges,ef)
 bn=np.cross(lo[faces[:,1]]-lo[faces[:,0]],lo[faces[:,2]]-lo[faces[:,0]]);tn=np.cross(hi[faces[:,1]]-hi[faces[:,0]],hi[faces[:,2]]-hi[faces[:,0]])
 return dict(determinant=d,jacobian=j,volume=v,orientation=np.einsum('ij,ij->i',bn,tn),side_weight=g['side_weights'],skewness=g['side_skewness'],centres=g['centres'])

def stack_audit(x,f):
 edges,ef=FG.unique_manifold_edges(f);K=len(x)-1;rows=[];previous=None;all_ok=True
 for k in range(K):
  q=layer_arrays(x[k],x[k+1],f,edges,ef,k>0,k<K-1);qC=determinant(np.concatenate([x[k][f],x[k+1][f]],axis=1),k>0,True)
  prefix=determinant(np.concatenate([x[k][f],x[k+1][f]],axis=1),k>0,False)
  fail=dict(determinant=int(sum(q['determinant']<DET)),jacobian=int(sum(q['jacobian']<=0)),volume=int(sum(q['volume']<=0)),orientation=int(sum(q['orientation']<=0)),side_weight=int(sum(q['side_weight']<WEIGHT)),skewness=int(sum(q['skewness']>SKEW)))
  hw=None;hs=None
  if previous is not None:
   ar,fc=FG.face_area_centre_tri(x[k][f]);a=abs(np.einsum('ij,ij->i',ar,fc-previous));b=abs(np.einsum('ij,ij->i',ar,q['centres']-fc));hw=np.minimum(a,b)/(a+b);hs=FG.internal_face_skewness(x[k][f],fc,ar,previous,q['centres']);fail.update(horizontal_weight=int(sum(hw<WEIGHT)),horizontal_skewness=int(sum(hs>SKEW)))
  rows.append(dict(layer=k+1,failures_B=fail,failures_C_determinant=int(sum(qC<DET)),current_top_A_determinant_failures=int(sum(prefix<DET)),minimum_determinant_B=float(q['determinant'].min()),minimum_determinant_C=float(qC.min()),minimum_determinant_A_top=float(prefix.min()),minimum_jacobian=float(q['jacobian'].min()),minimum_volume=float(q['volume'].min()),minimum_side_weight=float(q['side_weight'].min()),minimum_horizontal_weight=None if hw is None else float(hw.min()),maximum_skewness=float(q['skewness'].max()),maximum_horizontal_skewness=None if hs is None else float(hs.max())))
  previous=q['centres'];all_ok &= not any(fail.values())
 return dict(rows=rows,all_B_numerical_pass=bool(all_ok),all_C_numerical_pass=all(not any(v for key,v in r['failures_B'].items() if key!='determinant') and r['failures_C_determinant']==0 for r in rows),valid_orientation_jacobian_volume=all(not any(r['failures_B'][k] for k in ['jacobian','volume','orientation']) for r in rows),core_interface_weights='Unavailable, no core neighbour',continuous_global_checks='Not provided by this numerical audit')

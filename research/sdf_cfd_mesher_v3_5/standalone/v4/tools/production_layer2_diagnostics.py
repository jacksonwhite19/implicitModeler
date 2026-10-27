"""Read-only source-matched audit of realized L2, never a front generator."""
import json,time
from fractions import Fraction
import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
from production_layer2_run import OUT,L1,SURFACE,Source,save,g
from production_first_layer_diagnostics import stats,angles,triangle
from production_l2_screening import direct_screen
from production_adaptive_surface import CapsuleGeometry
from sdf_cfd_mesher_v4.conditioning import matrices,gains
from sdf_cfd_mesher_v4.common.authority import array_hash,identity


def run():
 start=time.perf_counter();folder=OUT/'A';old=np.load(L1/'A/CANDIDATE.npz');new=np.load(folder/'CANDIDATE.npz')
 x=old['wall'];low=old['front'];high=new['front'];f=old['triangles'];n=len(x);h=float.fromhex('0x1.6b844d56c388ep-13')
 assert np.array_equal(f,new['triangles'])
 top=g.q.build_topology(f,n);e=top.edges;definition=json.loads((SURFACE.parent/'GEOMETRY.json').read_text())['definition'];source=Source(dict(components=[definition]))
 samples=[source.evaluate(v) for v in (x,low,high)];direction=samples[1].normal;grad=samples[1].gradient;gu=grad/np.linalg.norm(grad,axis=1)[:,None]
 disp=high-low;dot=(disp*direction).sum(1);prior=low-x
 height=dict(euclidean=np.linalg.norm(disp,axis=1),construction_normal=dot,sdf_normal=(disp*gu).sum(1),tangential=np.linalg.norm(disp-dot[:,None]*direction,axis=1),requested_minus_achieved=h-dot,achieved_requested=dot/h)
 normals=dict(method='Generic Newton SDF predictor from committed L1, initial analytic source normal; target is bound cumulative L2 offset',angle_sdf=stats(angles(direction,grad)),angle_prior_L1=stats(angles(direction,prior)),angle_wall=stats(angles(direction,samples[0].normal)),gradient_norm=stats(samples[1].gradient_norm),normal_norm=stats(np.linalg.norm(direction,axis=1)),nonfinite=int((~np.isfinite(direction)).sum()),zero=int((np.linalg.norm(direction,axis=1)<=0).sum()),inward=int(((direction*grad).sum(1)<=0).sum()),fallbacks=0)
 edges={};triangles={};edge_arrays=[];triangle_arrays=[];geodiag={};geometry=CapsuleGeometry(definition)
 for label,v,sample in zip(('wall','L1','L2'),(x,low,high),samples):
  lengths=np.linalg.norm(v[e[:,1]]-v[e[:,0]],axis=1);t=triangle(v,f);edge_arrays.append(lengths);triangle_arrays.append(t)
  edges[label]=stats(lengths);triangles[label]={k:stats(a) for k,a in t.items()}
  cross=np.cross(v[f[:,1]]-v[f[:,0]],v[f[:,2]]-v[f[:,0]])
  triangles[label].update(zero_area=int((t['area']<=0).sum()),orientation_reversals=int(((cross*samples[0].normal[f].mean(1)).sum(1)<=0).sum()),nonfinite=int((~np.isfinite(v)).sum()))
  curvature=np.linalg.eigvalsh(np.array([geometry.gradient_hessian(1,pt)[1] for pt in v]));curv=np.max(np.abs(curvature),axis=1)
  geodiag[label]=dict(analytic_SDF_hessian_eigenvalues=stats(curvature),normal_variation_degrees=stats(angles(sample.normal[e[:,0]],sample.normal[e[:,1]])),requested_height_over_edge=stats(h/lengths),achieved_height_over_edge=stats(dot[e].mean(1)/lengths),requested_height_times_curvature=stats(h*curv),achieved_height_times_curvature=stats(dot*curv))
 matched={}
 for label,a,b in [('L1_wall',1,0),('L2_L1',2,1),('L2_wall',2,0)]:
  matched[label]=dict(edge_ratio=stats(edge_arrays[a]/edge_arrays[b]),edge_change=stats(edge_arrays[a]-edge_arrays[b]),area_ratio=stats(triangle_arrays[a]['area']/triangle_arrays[b]['area']))
 print('front diagnostics done',flush=True)
 p,c,o=g.s.construct(x,f,[low,high],np.full(n,2));owners,own=g.t.ownership(c,o,[tuple(reversed(t)) for t in f])
 q=json.loads((folder/'QUALITY.json').read_text());quality={}
 for scope in ('L2','combined'):
  byid={cell['id']:cell for cell in c};cr=[r for r in q['cells'] if scope=='combined' or byid[r['id']]['layer']==2]
  fr=[r for r in q['faces'] if scope=='combined' or any(byid[i]['layer']==2 for i in r['owners'])]
  rows_out={}
  for key,rows,gate,upper in [('volume',cr,0,False),('determinant',cr,.0011,False),('aspect',cr,1000,True),('weight',fr,.055,False),('skewness',fr,3.63636,True),('symmetric_volume_ratio',fr,.01,False)]:
   a=np.array([r[key] for r in rows]);i=int(a.argmax() if upper else a.argmin());bad=(a>gate if upper else a<gate) if key!='volume' else a<=0
   rows_out[key]=dict(distribution=stats(a),gate=gate,failures=int((bad|~np.isfinite(a)).sum()),worst=rows[i],margin=float(gate-a[i] if upper else a[i]-gate))
  exact=[r for r in q['exact'] if scope=='combined' or byid[r['id']]['layer']==2];j=np.array([float(Fraction(r['jacobian'])) for r in exact])
  rows_out['exact_jacobian']=dict(distribution=stats(j),failures=sum(r['jacobian_sign']<=0 for r in exact),worst=exact[int(j.argmin())]);quality[scope]=rows_out
 priorq=json.loads((L1/'A/QUALITY.json').read_text());before={r['id']:r['determinant'] for r in priorq['cells']};l1rows=[r for r in q['cells'] if r['id'] in before];old_det=np.array([before[r['id']] for r in l1rows]);new_det=np.array([r['determinant'] for r in l1rows])
 det=dict(prior_L1=stats(old_det),completed_L1=stats(new_det),difference=stats(new_det-old_det),L2_terminal=quality['L2']['determinant']['distribution'],semantics='L1 actual lateral and actual L2 top neighbour; wall cap excluded. L2 actual bottom and lateral neighbours plus declared internal terminal top-cap area tensor. No L3 geometry, centroid or exposed-cap determinant.',prior_hash=array_hash(old_det),completed_hash=array_hash(new_det),difference_hash=array_hash(new_det-old_det))
 print('metric diagnostics done',flush=True)
 conditioning={};geom=[g.t.geometry(p,cell) for cell in c];sf=np.array([v['areas'] for v in geom]);mask=np.zeros((len(c),5),bool);mask[:,0]=[cell['layer']==1 for cell in c];ops=gains(matrices(sf,mask))
 for label,sel in [('L1',np.array([cell['layer']==1 for cell in c])),('L2',np.array([cell['layer']==2 for cell in c])),('combined',np.ones(len(c),bool))]:conditioning[label]={k:stats(v[sel]) for k,v in ops.items()}
 screen=direct_screen(top,np.array([x,low]),high,layer=2);forecast=np.load(OUT.parent/'production_layer2_risk_coverage/SCREEN_ARRAYS.npz');comparison={}
 for group in ('cells','sides','caps'):
  for key,value in screen[group].items():
   name=group+'__'+key
   if name not in forecast:continue
   a=np.asarray(value,float);b=forecast[name];d=a-b;relative=np.divide(d,b,out=np.zeros_like(d),where=b!=0)
   comparison[name]=dict(equal=bool(np.array_equal(a,b,equal_nan=True)),absolute_difference=stats(np.abs(d).ravel()),relative_difference_nonzero_denominator=stats(relative.ravel()))
 d=high-forecast['provisional_high'];comparison['coordinates']=dict(equal=bool(np.array_equal(high,forecast['provisional_high'])),absolute_difference=stats(np.abs(d).ravel()),realized=array_hash(high),forecast=array_hash(forecast['provisional_high']))
 # Also compare the final scalar validator, whose face ordering and summation
 # differ from the batched forecast metric implementation. Do not hide roundoff.
 final_arrays={}
 l2cells=sorted((r for r in q['cells'] if byid[r['id']]['layer']==2),key=lambda r:r['column'])
 for key in ('volume','determinant','aspect'):final_arrays['cells__'+key]=np.array([r[key] for r in l2cells])
 edge_index={tuple(v):i for i,v in enumerate(e)}
 for key in ('weight','skewness','symmetric_volume_ratio'):
  side=np.full(len(e),np.nan);cap=np.full(len(f),np.nan)
  for row in q['faces']:
   layers=[byid[i]['layer'] for i in row['owners']]
   if layers==[2,2]:side[edge_index[tuple(sorted(set(v%n for v in row['vertices'])))]]=row[key]
   elif set(layers)=={1,2}:cap[byid[row['owners'][0]]['column']]=row[key]
  assert np.isfinite(side).all() and np.isfinite(cap).all()
  final_arrays['sides__'+key]=side;final_arrays['caps__'+key]=cap
 for name,a in final_arrays.items():
  b=forecast[name];delta=a-b;relative=np.divide(delta,b,out=np.zeros_like(delta),where=b!=0)
  comparison['final_validator__'+name]=dict(equal=bool(np.array_equal(a,b)),absolute_difference=stats(np.abs(delta)),relative_difference_nonzero_denominator=stats(np.abs(relative)))
 assert not screen['events']
 adjacency=coo_matrix((np.ones(len(e)*2),(np.r_[e[:,0],e[:,1]],np.r_[e[:,1],e[:,0]])),shape=(n,n)).tocsr()
 unique_faces=len(owners);internal=sum(len(v)==2 for v in owners.values());boundary=sum(len(v)==1 for v in owners.values())
 # Use ordered physical face perimeters, never edges of a sorted face key.
 combined_edges=set()
 for entries in owners.values():
  face=entries[0][1]
  combined_edges.update(tuple(sorted((face[i],face[(i+1)%len(face)]))) for i in range(len(face)))
 ce=np.array(sorted(combined_edges));cg=coo_matrix((np.ones(2*len(ce)),(np.r_[ce[:,0],ce[:,1]],np.r_[ce[:,1],ce[:,0]])),shape=(len(p),len(p))).tocsr()
 mapping_ok=all(cell['vertices']==list(map(int,np.r_[f[cell['column']]+(cell['layer']-1)*n,f[cell['column']]+cell['layer']*n])) for cell in c)
 assert mapping_ok and len(set(cell['id'] for cell in c))==len(c) and len(np.unique(f))==n
 topology=dict(wall_and_each_front=dict(vertices=n,edges=len(e),triangles=len(f),components=connected_components(adjacency)[0],Euler=n-len(e)+len(f)),new_prisms=len(f),combined_cells=len(c),combined_vertices=len(p),combined_faces=unique_faces,internal_faces=internal,boundary_faces=boundary,ownership=own,vertex_links=g.t.vertex_links(o),source_vertex_maps=n,source_edge_maps=len(e),source_triangle_maps=len(f),missing=0,duplicate=0,orphan=0,foreign=0,unused=int(len(p)-len(np.unique(np.array([cell['vertices'] for cell in c])))),source_column_continuity=mapping_ok,direct_screen_topology=screen['topology'])
 topology.update(combined_edges=len(ce),combined_components=connected_components(cg)[0],combined_euler=len(p)-len(ce)+unique_faces-len(c))
 result=dict(normals=normals,height={k:stats(v) for k,v in height.items()},requested=h,height_nonpositive=int((dot<=0).sum()),edges=edges,triangles=triangles,matched=matched,geometry_diagnostics=geodiag,quality=quality,determinant=det,conditioning=conditioning,nonorthogonality={k:stats(screen[k]['nonorthogonality']) for k in ('sides','caps')},forecast=comparison,topology=topology,seconds=time.perf_counter()-start)
 arrays={**height,'prior_L1_determinant':old_det,'completed_L1_determinant':new_det,**{k:v for k,v in ops.items()}}
 np.savez_compressed(folder/'DIAGNOSTIC_ARRAYS.npz',**arrays);save(folder/'DIAGNOSTICS.json',result);print('diagnostics done',result['seconds'],flush=True)

if __name__=='__main__':run()

"""Input and observation adapter for existing canonical planner symbols."""
from smooth_authority import *
import csv
sys.path.insert(0,str(V3/'foundation_aware_prism_shell_qualification'))
import foundation_geometry as FG
W=load_module('canonical_weight',V3/'foundation_aware_prism_shell_qualification/run_weight_aware_planner.py');B=W.BASE
S=load_module('canonical_skew',V3/'foundation_skew_increment_qualification/run_skew_aware_planner.py')
State=load_module('canonical_state',V3/'sdf_guided_advancing_front_qualification/accepted_front_state.py').AcceptedFrontState
def bind():
 a=SmoothAuthority();d=np.load(HERE/'first_row/ordinary_exact_height.npz');p,f,refs,top=d['wall'],d['faces'],d['refs'],d['top'];inc=[set() for _ in p]
 for face,ref in zip(f,refs):
  for v in face:inc[v].add(int(ref))
 normals=[];incidents=[];later=[];normal_keys=[]
 vr=list(csv.DictReader((HERE/'surface/A/vertices_A.tsv').open(),delimiter='\t'));vh=sha(HERE/'surface/A/vertices_A.tsv')
 keys=[r['persistent_corner'] or 'qualified-surface:'+hashlib.sha256((vh+':'+str(i)).encode()).hexdigest() for i,r in enumerate(vr)]
 for v in range(len(p)):
  n=np.array([a.exact_normal(r,p[v]) for r in sorted(inc[v])]);incidents.append(list(range(len(normals),len(normals)+len(n))));normals.extend(n);normal_keys.extend([(keys[v],a.sheet_names[r]) for r in sorted(inc[v])]);later.append(n.sum(axis=0)/np.linalg.norm(n.sum(axis=0)))
 later=np.array(later);state=State(p,top,f,keys,normals,incidents,np.tile(later,(19,1,1)),H1,'path_length_m');h=np.tile(B.nominal_increments(),(len(p),1))
 state.save(HERE/'planner/input_state',h);np.save(HERE/'planner/refs.npy',refs)
 edges,ef=FG.unique_manifold_edges(f);nominal=B.audit_faces(p,f,later,h,accepted_state=state);weights=W.complete_weight_audit(p,f,later,h,edges,ef,accepted_state=state);skew=S.complete_face_audit(p,f,later,h,edges,ef,accepted_state=state)
 write('planner/NOMINAL_STAGED.json',nominal);write('planner/NOMINAL_WEIGHT.json',weights);write('planner/NOMINAL_SKEW.json',skew)
 write('planner/BINDING.json',dict(source_normal_keys=normal_keys,wall_faces=len(f),wall_vertices=len(p),first_exact=np.array_equal(state.positions(h)[1],top),first_hash=state.first_hash,scalar_quantity=state.scalar_quantity,prospective_prisms=len(f)*20,call_chain=['AcceptedFrontState','B.audit_faces / select_layer2_component / select_late_component','W.complete_weight_audit / select_correction','S.complete_face_audit / select_field','future exact guards','explicit vector adapter'],canonical_hashes={str(Path(m.__file__).relative_to(REPO)):sha(Path(m.__file__)) for m in [W,B,S]},nominal_sdf_lookahead='Complete exact future geometry; no analytic-NACA cheap curvature substitute',later_direction='Existing unit averaged incident-normal trajectory; compatible X1 preserved separately. Later scalar increments retain path-length meaning.'))
 print('staged',nominal['pass'],'weight',weights.keys(),'skew',skew.keys(),flush=True)
if __name__=='__main__':bind()

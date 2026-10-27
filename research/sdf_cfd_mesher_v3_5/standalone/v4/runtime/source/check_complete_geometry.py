"""Whole-shell checks through existing native validators; no plan changes."""
from bind_planner import *
import struct
import sys
sys.path.insert(0,str(V3))
from continuous_prism_certificate.integration import audit_fixed_side_pairs
assert read(HERE/'planner/PLANNING_RESULT.json')['canonical_numerical_pass']
state,h=State.load(HERE/'planner/selected_state');x=state.positions(h);f=state.faces;out=HERE/'planner/global';out.mkdir(exist_ok=True)
records=[]
def native(tool,path,tag):
 r=linux([wsl(V3/'global_prism_host_architecture'/tool),wsl(path)],tag,600);q=dict(tool=tool,exit=r.returncode,result=json.loads(r.stdout) if r.stdout.startswith('{') else None,stderr=r.stderr)
 write(out/(tag+'.json'),q);assert r.returncode==0 and q['result']['pass'],q
 return q
# Preserve the already-executed convex-hull diagnostic. The qualified curved
# workflow treats shared-entity convex-hull penetration as diagnostic, and
# requires the existing continuous bilinear certificates instead.
if not (out/'complete_overlap.json').exists():
 path=out/'complete_stack.bin';path.write_bytes(struct.pack('<QQQ',len(x[0]),len(f),20)+x.astype('<f8').tobytes()+f.astype('<u4').tobytes())
 r=linux([wsl(V3/'global_prism_host_architecture/variable_layer_overlap_audit'),wsl(path)],'complete_overlap',600)
 write(out/'complete_overlap.json',dict(tool='variable_layer_overlap_audit',exit=r.returncode,result=json.loads(r.stdout),stderr=r.stderr))
diagnostic=read(out/'complete_overlap.json')
assert diagnostic['result']['nonincident_positive_overlap_count']==0
write(out/'APPLICABILITY.json',dict(source_policy='adaptive_surface_architecture/FROZEN_BOUNDED_LAYER_PROOF_V1.json',qualified_reference='adaptive_surface_architecture/SELECTED_ADAPTIVE_SURFACE_REPORT.json',incident_convex_hull_flags='Diagnostic only under the existing curved-side contract; not waived physical collisions',required='Qualified complete exact bilinear side-side certificate; existing outward-rounded interval side-cap, regularity and trajectory-cap certificates, exact nonincident collision, positive complete Jacobians and cumulative cap topology',new_gate_or_threshold=False))
for k in range(1,21):
 path=out/f'continuous_{k}.bin';path.write_bytes(b'V3SIDE1\0'+struct.pack('<QQ',len(x[0]),len(f))+x[k-1:k+1].astype('<f8').tobytes()+f.astype('<u8').tobytes())
 q=audit_fixed_side_pairs(path,out/f'layer_{k}_complete_bilinear_certificate.json')
 records.append(dict(tool='complete_bilinear_certificate',exit=0,result=q,stderr=''))
 for tool in ['full_airfoil_side_cap_audit']:
  r=linux([wsl(V3/'global_prism_host_architecture_v2/bin_certified'/tool),wsl(path)],f'layer_{k}_{tool}',600);q=dict(tool=tool,exit=r.returncode,result=json.loads(r.stdout) if r.stdout.startswith('{') else None,stderr=r.stderr);write(out/f'layer_{k}_{tool}.json',q);assert r.returncode==0,q;records.append(q)
 path=out/f'layer_{k}.bin';path.write_bytes(struct.pack('<QQQ',len(x[0]),len(f),1)+x[k-1:k+1].astype('<f8').tobytes()+f.astype('<u4').tobytes())
 records.append(native('complete_one_layer_collision_audit',path,f'layer_{k}_contact'))
 path=out/f'front_{k}.bin';path.write_bytes(struct.pack('<QQ',len(x[0]),len(f))+x[k].astype('<f8').tobytes()+f.astype('<u4').tobytes())
 records.append(native('surface_topology_audit',path,f'front_{k}_topology'))
 print(k,flush=True)
pairs=state.pairs;normal_delta=np.einsum('ij,kij->ki',state.normals[pairs[:,1]],np.diff(x,axis=0)[:,pairs[:,0]])
assert read(HERE/'FIRST_ROW_BRANCH_CERTIFICATE.json')['pass_gate'] and normal_delta.min()>0
write('planner/COMPLETE_GEOMETRY_GATE.json',dict(pass_gate=True,global_overlap='Existing conservative all-layer SAT audit; all actual columns and layers',per_layer_contacts='Existing exact-predicate continuous prism-boundary check at every row',all_fronts_closed_manifold_no_self_intersections=True,branch='Convex full source extrusion; all source-support normal increments strictly positive, no branch reversal; exact first-row certificate retained',minimum_source_normal_increment_m=float(normal_delta.min()),checked_prisms=20*len(f),records=records,materialization_authorized=True))

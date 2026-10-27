"""Read-only bindings. No accepted compression selector is called here."""
from common import *
import numpy as np, importlib.util, ast
sys.path.insert(0,str(V3/'foundation_aware_prism_shell_qualification'))
def mod(name,path):
 sp=importlib.util.spec_from_file_location(name,path);m=importlib.util.module_from_spec(sp);sys.modules[name]=m;sp.loader.exec_module(m);return m
State=mod('owned_front_state',V3/'sdf_guided_advancing_front_qualification/accepted_front_state.py').AcceptedFrontState
import foundation_geometry as FG
PM=mod('owned_physical_predicates',V3/'prism_feasibility_contract/prospective_commit_adapter.py')
PREV=G/'prism_placement_counterfactual_20260913'
# Pure qualified geometry functions use this study's common paths, not old writers.
geom=mod('owned_qualified_geometry',PREV/'geometry.py')
def load_state(name):
 s,nom=State.load(H/'SF2/A/wing/planner/input_state');old,h0=State.load(H/'SF2/A/wing/planner/selected_state')
 expected=h0.copy();changed=np.empty(0,dtype=int)
 if name!='baseline':
  root=G/'compression_dependency_closure_planning_20260913'
  path=root/('priority/patch_000/FINAL_PLAN.npz' if name=='patch000' else 'priority/patch_019/replay0/OUTPUT_1.npz')
  a=np.load(path);delta=np.zeros_like(s.wall)
  for v,b,z in zip(a['dof_vertices'],a['basis'],a['coefficient']):delta[v]+=b*z
  changed=np.flatnonzero(np.any(delta!=0,axis=1));first=s.first.copy();first[changed]+=s.first_height*delta[changed]
  vectors=s.vectors.copy();d=s.vectors[0,changed]+delta[changed];d/=np.linalg.norm(d,axis=1)[:,None];vectors[:,changed]=d[None]
  s=State(s.wall,first,s.faces,s.vertex_order,s.normals,s.incidents,vectors,s.first_height,'path_length_m',ownership=s.ownership)
  expected[a['vertices']]=a['schedule'];assert np.array_equal(s.positions(expected)[:,a['vertices']],a['positions'])
 else:assert np.array_equal(s.positions(expected),old.positions(h0))
 return s,nom,h0,changed
def target_columns():return np.array(read(PREV/'SF2/COHORTS.json')['groups']['weakest_0_01pct'],dtype=int)
def symbolic_functions():
 import casadi as ca
 src=V3/'complete_short_wing_continuation_2026-09-06/continuation_model.py'
 node=next(n for n in ast.parse(src.read_text()).body if isinstance(n,ast.FunctionDef) and n.name=='functions')
 ns=dict(ca=ca,V3=V3);exec(compile(ast.Module(body=[node],type_ignores=[]),str(src),'exec'),ns)
 _,cell,side,horizontal=ns['functions']()
 src=(V3/'finite_wing_terminal_prism_feasibility_2026-09-06/symbolic_prism.py').read_text()
 src=src.replace('coupled=newell[1:];a=sum(norm(s) for s in coupled)/4','coupled=[newell[i] for i in [0,2,3,4]];a=sum(norm(s) for s in coupled)/4')
 ns={};exec(compile(src,'qualified_final_ownership_mask','exec'),ns)
 return cell,side,horizontal,ns['make_functions']()[0]

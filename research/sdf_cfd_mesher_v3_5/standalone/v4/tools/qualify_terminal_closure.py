"""Complete the unchanged publication closure gate on preserved geometry."""
import sys,json,time,traceback
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT/'src'),str(ROOT/'tools')]
import numpy as np
import generalize_local_remesh as g
from run_geometry_family import AnalyticProvider
from coupled_shell_validation import check,qualify_closures
from bounded_fallback_control import Events,require_space
from sdf_cfd_mesher_v4.common.authority import file_hash
r=ROOT/'results/terminal_capacity_aware_remesh_acceptance';old=ROOT/'results/bent_passage_large_case_fallback'
e=Events(r/'CLOSURE_EVENTS.jsonl',600);result={};start=time.perf_counter()
try:
 e('start',guard=require_space(r,256348653,81842381))
 spec=json.loads((ROOT/'results/production_scale_resume_fast/bent_continuation/GEOMETRY_AUTHORITY.json').read_text())['spec'];provider=AnalyticProvider(spec,r)
 z=np.load(old/'nominal/REPRODUCER.npz');fronts=list(z['fronts']);fronts[-1]=np.load(old/'tangent_remaining/BEST.npz')['front'];levels=np.load(old/'remesh/LEVELS.npy')
 points,cells,outer=g.s.construct(provider.x,provider.tri,fronts,levels)
 e('closure_numeric_start');coupled=check(points,cells,outer,details=True);e('closure_numeric_end',passed=coupled['passed'],closures=coupled['closures'])
 previous=json.loads((old/'complete_candidate/COUPLED.json').read_text());assert coupled['closure_identity']==previous['closure_identity'];assert coupled['passed']
 e('publication_closure_start');qualify_closures(r,19,points,cells,dict(complete_neighbor=coupled));e('publication_closure_end')
 result=dict(passed=True,closure_identity=coupled['closure_identity'],closures=coupled['closures'],exact_closure=coupled['exact_closure'])
except BaseException as exc:result=dict(passed=False,error=str(exc),traceback=traceback.format_exc())
result['seconds']=time.perf_counter()-start;(r/'PUBLICATION_CLOSURE.json').write_text(json.dumps(result,indent=2));print(json.dumps(result))

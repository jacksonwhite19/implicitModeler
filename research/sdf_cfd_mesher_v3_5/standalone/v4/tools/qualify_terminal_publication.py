"""Qualify and publish one preserved terminal candidate; never advance geometry."""
import sys,json,time,traceback,gc,subprocess
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT/'src'),str(ROOT/'tools')]
import numpy as np
import fast_milestone,generalize_local_remesh as g
from bounded_fallback_control import Events,require_space
from investigate_large_fallback import save
from immutable_append_certificate import certify
from terminal_capacity import range_witness,terminal_state,remesh_admitted,prevent_terminal_advance
from terminal_shell_store import publish,restart
from sdf_cfd_mesher_v4.common.authority import identity,array_hash,file_hash
from sdf_cfd_mesher_v4.advancing_front.native_checkpoint import scientific_digest
from sdf_cfd_mesher_v4.advancing_front.mixed_front_runner import preserves_committed_cells
from sdf_cfd_mesher_v4.advancing_front.termination_policy import REMESH_CHECKS
from sdf_cfd_mesher_v4.advancing_front.execution_profile import MemoryTransaction
from validation_row_stream import clean_rows_digest
from audit_capsule_scaling import final_coverage,AABBIndex
r=ROOT/'results/terminal_capacity_aware_remesh_acceptance';old=ROOT/'results/bent_passage_large_case_fallback'
e=Events(r/'PUBLICATION_EVENTS.jsonl',2400);out=dict(committed=False);start=time.perf_counter()
def proof_progress(rows):
 for i,row in enumerate(rows,1):
  if i%500000==0:e('proof_identity_progress',rows=i)
  yield row
try:
 # Verify every reused scientific input against its sealed prior manifest.
 authority=json.loads((old/'AUTHORITY.json').read_text())
 for name in ['nominal/REPRODUCER.npz','tangent_remaining/BEST.npz','remesh/LEVELS.npy','complete_candidate/COUPLED.json','complete_candidate/VOLUME.json']:
  row=next(z for z in authority['artifacts'] if z['path'].replace('\\','/')==name)
  assert file_hash(old/name)==row['sha256']
 e('stage_start',stage='load',guard=require_space(r,256348653,165409059))
 run=fast_milestone.load(ROOT/'results/production_scale_resume_fast/bent_continuation/milestone_18',r);provider=run.provider;before=run.tx.current
 z=np.load(old/'nominal/REPRODUCER.npz');fronts=list(z['fronts']);fronts[-1]=np.load(old/'tangent_remaining/BEST.npz')['front'];levels=np.load(old/'remesh/LEVELS.npy')
 witness=range_witness(run.spec,float(provider.schedule.targets[len(fronts)]),list(range(len(provider.tri))));assert witness
 save(r/'CAPACITY_WITNESS.json',witness)
 e('stage_start',stage='current_hard_quality');p,c,o,checked,times=g.validation(provider.x,provider.tri,fronts,levels,False)
 assert checked['quality']['passed'] and checked['ownership']['passed'] and checked['vertex_links']['passed']
 e('stage_end',stage='current_hard_quality',times=times)
 n=len(provider.x);used=sorted({v for cell in c for v in cell['vertices'] if v>=n});sdf=[]
 for layer in range(1,len(fronts)+1):
  ids=[v for v in used if v//n==layer]
  if ids:sdf.append(provider.geometry_source.sdf_record(run.spec,p[ids],provider.schedule.targets[layer-1]))
 assert all(z['admitted'] for z in sdf);save(r/'SDF_ADMISSION.json',sdf)
 e('stage_start',stage='native_geometry');state=provider.pack(fronts,levels,before);assert preserves_committed_cells(state,before)
 assert state['wall']==before['wall'] and state['first_layer']==before['first_layer'];state=terminal_state(state,run.spec,float(provider.schedule.targets[len(fronts)]),provider.schedule.targets,witness)
 e('stage_end',stage='native_geometry')
 coupled=json.loads((r/'COUPLED_19.json').read_text());assert coupled['passed'] and coupled['exact_closure']['passed']
 volume=json.loads((old/'complete_candidate/VOLUME.json').read_text());assert volume['positive_cells'] and volume['exact_volume_balance'] and volume['interior_degree']['passed']
 # Same exact arrays reconstructed from authenticated inputs; old filling is
 # reusable for precommit, then independently recomputed after restart.
 owners=g.t.ownership(c,o,[tuple(reversed(f)) for f in provider.tri])[0]
 cert=certify(run,p,c,owners,e);assert cert['passed']
 e('stage_start',stage='proof_content_identity');proof_digest=clean_rows_digest(proof_progress(cert['proofs']));e('stage_end',stage='proof_content_identity')
 quality_digest=scientific_digest(checked['quality'])
 geometry_digest=scientific_digest({k:state[k] for k in ('wall','first_layer','vertices','cells','faces','columns','outer','prism_front','local_schedule')})
 receipt=dict(checks={k:True for k in REMESH_CHECKS},terminal_capacity=state['local_schedule']['terminal_capacity']);receipt['checks']['neighbor_continuation']=False
 assert remesh_admitted(receipt)
 # Prior ingestion is the same authenticated outer surface; repeat handoff.
 import shutil
 shutil.copyfile(old/'complete_candidate/CORE_INGEST_INPUT.json',r/'CORE_INGEST_INPUT.json')
 def wsl(path):path=path.resolve();return '/mnt/'+path.drive[0].lower()+str(path)[2:].replace('\\','/')
 subprocess.run(['wsl','--','python3',wsl(ROOT/'tools/cavity_core_ingestion_smoke.py'),wsl(r/'CORE_INGEST_INPUT.json')],check=True,capture_output=True)
 assert json.loads((r/'CORE_INGESTION.json').read_text())['passed']
 scientific=identity(dict(schema='V4TerminalMixedShellScientific/1',geometry=geometry_digest,quality=quality_digest,proofs=proof_digest,capacity=witness,closure=coupled['closure_identity']))
 data=dict(state=state,spec=run.spec,points=p,cells=c,outer=o,wall=provider.tri,proofs=cert['proofs'],faces=cert['faces'],receipt=receipt,scientific=scientific,geometry_digest=geometry_digest,quality_digest=quality_digest,proof_digest=proof_digest)
 e('stage_start',stage='atomic_publication',guard=require_space(r,256348653,256348653))
 tx=MemoryTransaction(before);tx.commit(state,0,lambda candidate:remesh_admitted(receipt) and preserves_committed_cells(candidate,before))
 cp=publish(r/'checkpoint',data,lambda d:remesh_admitted(d['receipt']))
 out.update(committed=True,checkpoint=cp,scientific=scientific,cells=len(c),cell_types=dict(__import__('collections').Counter(z['kind'] for z in c)),last_prism_layers=dict(__import__('collections').Counter(z['last_prism_layer'] for z in state['columns'].values())))
 save(r/'PUBLICATION.json',out);e('stage_end',stage='atomic_publication',bytes=cp['bytes'])
 # Release duplicate source geometry before verified restart and full seal.
 del run,provider,before,data,state,cert,volume,coupled,checked,owners,tx;gc.collect()
 e('stage_start',stage='restart');d=restart(r/'checkpoint')
 assert remesh_admitted(d['receipt']);assert scientific_digest({k:d['state'][k] for k in ('wall','first_layer','vertices','cells','faces','columns','outer','prism_front','local_schedule')})==d['geometry_digest']
 try:prevent_terminal_advance(d['state']);raise AssertionError('TerminalReactivated')
 except ValueError as exc:assert str(exc)=='GlobalTerminalCapacityAlreadyResolved'
 e('stage_end',stage='restart',exact=True)
 e('stage_start',stage='independent_quality');p,c,o=d['points'],d['cells'],d['outer'];owners,own=g.t.ownership(c,o,[tuple(reversed(f)) for f in d['wall']]);q=g.t.quality(p,c,owners,1)
 assert q['passed'] and scientific_digest(q)==d['quality_digest']
 from final_shell_incidence import check
 incidence=check(p,c,o);assert incidence['passed'];e('stage_end',stage='independent_quality')
 e('stage_start',stage='independent_global_degree');v=g.volume.audit(p,c,o,d['wall']);assert v['positive_cells'] and v['exact_volume_balance'] and v['interior_degree']['passed'];e('stage_end',stage='independent_global_degree')
 original=AABBIndex.pairs
 def pairs(self,*a,**kw):
  for i,pair in enumerate(original(self,*a,**kw),1):
   if i%500000==0:e('coverage_progress',pairs=i)
   yield pair
 AABBIndex.pairs=pairs
 e('stage_start',stage='independent_coverage');coverage=final_coverage(p,d['faces'],d['proofs']);e('stage_end',stage='independent_coverage',**coverage)
 assert coverage['missing']==0 and coverage['duplicates']==0
 e('stage_start',stage='independent_proof_identity');assert clean_rows_digest(proof_progress(d['proofs']))==d['proof_digest'];e('stage_end',stage='independent_proof_identity')
 seal=dict(passed=True,scientific=d['scientific'],geometry_digest=d['geometry_digest'],quality_digest=d['quality_digest'],proof_digest=d['proof_digest'],capacity=d['receipt']['terminal_capacity'],coverage=coverage,forbidden=0,unresolved=0,global_degree=True,exact_volume_balance=True,wall_first_layer_exact=True,restart_exact=True,
  determinant_min=min(z['determinant'] for z in q['cells']),weight_min=min(z['weight'] for z in q['faces']),total_cells=len(c),seconds=time.perf_counter()-start)
 save(r/'SEAL.json',seal);out['seal']=seal;out['status']='TerminalShellAccepted'
except BaseException as exc:out.update(error=str(exc),traceback=traceback.format_exc());e('failure',error=str(exc))
out['seconds']=time.perf_counter()-start;save(r/'RESULT.json',out);print(json.dumps({k:v for k,v in out.items() if k not in ('seal','checkpoint')}))

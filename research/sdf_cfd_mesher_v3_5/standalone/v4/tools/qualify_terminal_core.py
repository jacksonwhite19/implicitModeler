"""Secondary bounded core attempt using the unchanged interface/core adapter."""
import sys,json,time,subprocess,traceback
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT/'src'),str(ROOT/'tools')]
from investigate_large_fallback import save
from bounded_fallback_control import require_space
r=ROOT/'results/terminal_capacity_aware_remesh_acceptance';result={};started=time.perf_counter()
def wsl(p):p=p.resolve();return '/mnt/'+p.drive[0].lower()+str(p)[2:].replace('\\','/')
try:
 assert json.loads((r/'SEAL.json').read_text())['passed']
 save(r/'CORE_DISK_GUARD.json',require_space(r,139792322,139792322))
 with (r/'CORE_GENERATION.log').open('w') as log:
  proc=subprocess.run(['wsl','--','env','PYTHONPATH='+wsl(ROOT/'src'),'timeout','120s','python3','-B',wsl(ROOT/'tools/broader_geometry_core.py'),wsl(r/'interface'),wsl(r/'core'),'--algorithm','1'],stdout=log,stderr=subprocess.STDOUT)
 result['generation_exit']=proc.returncode
 if proc.returncode:raise RuntimeError('BoundedCoreGenerationFailedOrTimedOut')
 gen=json.loads((r/'core/GENERATION.json').read_text());result['initial']=gen;selected='core'
 if not gen['passed']:
  script="import sys;from pathlib import Path;sys.path[:0]=[sys.argv[1],sys.argv[2]];from repair_family_core import run;run(Path(sys.argv[3]))"
  with (r/'CORE_REPAIR.log').open('w') as log:
   try:repair=subprocess.run([sys.executable,'-c',script,str(ROOT/'src'),str(ROOT/'tools'),str(r)],stdout=log,stderr=subprocess.STDOUT,timeout=120)
   except subprocess.TimeoutExpired:raise RuntimeError('SecondaryCoreRepairProcessBound')
  result['repair_exit']=repair.returncode
  if repair.returncode:raise RuntimeError('FixedInterfaceRepairDidNotQualify')
  selected='repaired_core'
 result['selected']=selected
 # Existing complete adapter receives the same compact proof population through
 # its already-qualified lazy receipt adapter, not a new certificate predicate.
 from terminal_shell_store import restart
 from sdf_cfd_mesher_v4.advancing_front.mixed_front import geometry_inputs,face_key
 from sdf_cfd_mesher_v4.advancing_front.native_checkpoint import scientific_digest
 from live_proof_receipts import LiveProofReceipts
 d=restart(r/'checkpoint');state=d['state'];n=len(state['wall']['vertices']);source=lambda v:f'layer:{v//n}:source:{v%n}'
 keys=[face_key(tuple(source(v) for v in f)) for f in d['faces']]
 state['certificate']=LiveProofReceipts(d['proofs'],keys,geometry_inputs(state))
 seal=json.loads((r/'SEAL.json').read_text());seal['geometry_components']={k:scientific_digest(state[k]) for k in ('wall','first_layer','vertices','cells','faces','columns','outer','prism_front','local_schedule')};save(r/'SEAL.json',seal)
 from broader_geometry_assembly import run
 run(r,core_name=selected,sealed_state=state)
 result['complete']=json.loads((r/'ASSEMBLY.json').read_text());result['passed']=result['complete']['passed']
except BaseException as exc:result.update(passed=False,error=str(exc),traceback=traceback.format_exc())
result['seconds']=time.perf_counter()-started;save(r/'CORE_RESULT.json',result);print(json.dumps(result))

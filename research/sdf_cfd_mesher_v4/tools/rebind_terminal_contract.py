"""Rebind an unchanged sealed terminal population after contract strengthening."""
import sys,json,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT/'src'),str(ROOT/'tools')]
from terminal_shell_store import restart,publish
from terminal_capacity import range_witness,terminal_state,remesh_admitted
from sdf_cfd_mesher_v4.common.authority import identity,file_hash
from sdf_cfd_mesher_v4.advancing_front.native_checkpoint import scientific_digest
from investigate_large_fallback import save
r=ROOT/'results/terminal_capacity_aware_remesh_acceptance';start=time.perf_counter()
d=restart(r/'checkpoint');oldseal=json.loads((r/'SEAL.json').read_text());assert oldseal['passed'] and d['scientific']==oldseal['scientific']
save(r/'PRE_BINDING_CURRENT.json',json.loads((r/'checkpoint/CURRENT.json').read_text()));save(r/'PRE_BINDING_SEAL.json',oldseal);save(r/'PRE_BINDING_WITNESS.json',d['receipt']['terminal_capacity']['witness'])
oldgeo={k:scientific_digest(d['state'][k]) for k in ('wall','first_layer','vertices','cells','faces','columns','outer','prism_front')}
term=d['receipt']['terminal_capacity'];w=range_witness(d['spec'],term['target'],term['columns']);assert w
assert {k:v for k,v in w['payload'].items() if k!='witness_implementation'}=={k:v for k,v in term['witness']['payload'].items() if k!='witness_implementation'}
d['state']=terminal_state(d['state'],d['spec'],term['target'],term['frozen_targets'],w);d['receipt']['terminal_capacity']=d['state']['local_schedule']['terminal_capacity'];assert remesh_admitted(d['receipt'])
assert oldgeo=={k:scientific_digest(d['state'][k]) for k in oldgeo}
d['geometry_digest']=scientific_digest({k:d['state'][k] for k in (*oldgeo,'local_schedule')})
d['scientific']=identity(dict(schema='V4TerminalMixedShellScientific/1',geometry=d['geometry_digest'],quality=d['quality_digest'],proofs=d['proof_digest'],capacity=w,closure=json.loads((r/'PUBLICATION_CLOSURE.json').read_text())['closure_identity']))
cp=publish(r/'checkpoint',d,lambda data:remesh_admitted(data['receipt']));del d
new=restart(r/'checkpoint');assert remesh_admitted(new['receipt']) and new['quality_digest']==oldseal['quality_digest'] and new['proof_digest']==oldseal['proof_digest']
assert oldgeo=={k:scientific_digest(new['state'][k]) for k in oldgeo}
seal=dict(oldseal,scientific=new['scientific'],geometry_digest=new['geometry_digest'],capacity=new['receipt']['terminal_capacity'],semantic_rebind='Identical audited geometry/quality/proofs; next-frozen-target binding checked; witness implementation identity updated')
save(r/'SEAL.json',seal);save(r/'CAPACITY_WITNESS.json',w)
interface=json.loads((r/'TERMINAL_INTERFACE.json').read_text());interface['scientific']=new['scientific'];save(r/'TERMINAL_INTERFACE.json',interface)
save(r/'BINDING_REPLAY.json',dict(passed=True,old_scientific=oldseal['scientific'],new_scientific=new['scientific'],geometry_unchanged=True,quality_unchanged=True,proofs_unchanged=True,capacity_mathematics_unchanged=True,exact_restart=True,checkpoint=cp,seconds=time.perf_counter()-start))
print(new['scientific'],cp['bytes'],time.perf_counter()-start)

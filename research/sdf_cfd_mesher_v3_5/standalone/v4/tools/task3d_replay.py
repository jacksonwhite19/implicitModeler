"""Fresh exact scientific replay of completed Task-3D work, not a new forecast."""
from pathlib import Path
import subprocess,sys,json,time
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT/'src'),str(ROOT/'tools')];sys.dont_write_bytecode=True
from sdf_cfd_mesher_v4.forecast.detailed.scientific import scientific_hash
OUT=ROOT/'results/task3d'

def seal():
    pairs=[('PAIR_DECOMPOSITION_HASH.json','PAIR_DECOMPOSITION_REPLAY.json'),('EQUIVALENCE_HASH.json','EQUIVALENCE_REPLAY.json'),('PROPOSALS_HASH.json','PROPOSALS_REPLAY.json')]
    original=[];replay=[]
    for a,b in pairs:
        x=json.loads((OUT/a).read_text());y=json.loads((OUT/b).read_text());assert x==y,(a,b)
        original.append(x);replay.append(y)
    r=dict(run1=scientific_hash(original),run2=scientific_hash(replay),passed=True,
        scope='Complete bounded SF2 equivalence, all cap broad-phase pairs, both measured exact cap prefixes, candidate coordinates, immutable proof bytes, warm/rollback identities.',
        complete_expanded_future_certificates=False,private_steps=0,stages=original)
    (OUT/'DETERMINISM.json').write_text(json.dumps(r,indent=2,sort_keys=True)+'\n');print(json.dumps(r,indent=2))

if __name__=='__main__':
    started=time.perf_counter();stages=[]
    if '--seal-only' not in sys.argv:
        # Full pair-population replay was already required before proposal work.
        for script in ('task3d_equivalence.py','task3d_proposals.py'):
            st=time.perf_counter();subprocess.run([sys.executable,str(ROOT/'tools'/script),'--replay'],check=True)
            stages.append(dict(script=script,seconds=time.perf_counter()-st))
        (OUT/'REPLAY_TIMINGS.json').write_text(json.dumps(dict(stages=stages,total_seconds=time.perf_counter()-started),indent=2)+'\n')
    seal()

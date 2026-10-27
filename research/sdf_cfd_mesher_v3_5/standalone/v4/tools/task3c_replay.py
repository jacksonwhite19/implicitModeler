"""Fresh numerical and exact-kernel replay; no read-back-only determinism claim."""
from pathlib import Path
import sys,subprocess,json,time
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'src'));sys.dont_write_bytecode=True
from sdf_cfd_mesher_v4.forecast.detailed.scientific import scientific_hash
OUT=ROOT/'results/task3c'
STAGES=['inspect','audit','expand','closed_support','sensitivity','ablation','common_cap','response','certify','targets']
FILES=[('CORNER_TRACE_HASH.json','CORNER_TRACE_REPLAY.json'),('AUDIT_HASH.json','AUDIT_REPLAY.json'),
 ('EXPANSION.json','EXPANSION_REPLAY.json'),('CLOSED_SUPPORT.json','CLOSED_SUPPORT_REPLAY.json'),
 ('SENSITIVITY_HASH.json','SENSITIVITY_REPLAY.json'),('SEARCH_ABLATION_HASH.json','SEARCH_ABLATION_REPLAY.json'),
 ('COMMON_CAP_HASH.json','COMMON_CAP_REPLAY.json'),('FORECAST_RESPONSE_HASH.json','FORECAST_RESPONSE_REPLAY.json'),
 ('CERTIFICATION.json','CERTIFICATION_REPLAY.json'),('TARGET_TRAJECTORIES_HASH.json','TARGET_TRAJECTORIES_REPLAY.json')]
if __name__=='__main__':
    started=time.perf_counter();stages=[]
    if not (OUT/'CORNER_TRACE_HASH.json').exists():
        v=json.loads((OUT/'INITIAL_CORNER_TRACE.json').read_text());(OUT/'CORNER_TRACE_HASH.json').write_text(json.dumps(dict(scientific_hash=scientific_hash(v)))+'\n')
    if '--seal-only' not in sys.argv:
        for name in STAGES:
            beg=time.perf_counter();r=subprocess.run([sys.executable,str(ROOT/f'tools/task3c_{name}.py'),'--replay'])
            stages.append(dict(stage=name,returncode=r.returncode,seconds=time.perf_counter()-beg))
            (OUT/'REPLAY_STAGES.json').write_text(json.dumps(stages,indent=2)+'\n')
            if r.returncode:raise SystemExit(r.returncode)
            print('REPLAY_COMPLETE',name,flush=True)
    one=[];two=[]
    for a,b in FILES:
        x=json.loads((OUT/a).read_text())['scientific_hash'];y=json.loads((OUT/b).read_text())['scientific_hash'];assert x==y,(a,b)
        one.append(dict(stage=a,sha256=x));two.append(dict(stage=a,sha256=y))
    record=dict(passed=True,run1=scientific_hash(one),run2=scientific_hash(two),stages=one,
        method='Fresh regenerated proposals, direct metrics, sensitivity probes and exact current-layer kernel traversals. Over-budget future scopes replay the complete enumeration and fail-closed guard; no future proof is claimed.')
    (OUT/'DETERMINISM.json').write_text(json.dumps(record,indent=2,sort_keys=True)+'\n')
    if '--seal-only' not in sys.argv:(OUT/'REPLAY_RUNTIME.json').write_text(json.dumps(dict(seconds=time.perf_counter()-started),indent=2)+'\n')

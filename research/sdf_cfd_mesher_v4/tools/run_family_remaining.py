"""Isolated per-case processes; no shared caches or scientific result inputs."""
import sys,subprocess,json,time
from pathlib import Path
import geometry_family_authority as a

ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'results/broader_geometry_family_qualification'

def run(group):
    if group=='unscaled':specs=a.definitions()
    elif group=='half':specs=[s for s in a.cohort() if s.get('scale_factor')==.5]+[a.multi_region_definitions()[-1]]
    else:raise ValueError('UnknownGroup')
    rows=[]
    for spec in specs:
        folder=OUT/spec['name']/'Q'
        if folder.exists():raise ValueError('ExistingRun:'+str(folder))
        tick=time.perf_counter()
        with (OUT/(spec['name']+'_Q_batch.log')).open('w') as log:
            result=subprocess.run([sys.executable,'-B',str(ROOT/'tools/run_geometry_family.py'),spec['name'],'--replay','Q'],stdout=log,stderr=subprocess.STDOUT)
        rows.append(dict(case=spec['name'],exit_code=result.returncode,seconds=time.perf_counter()-tick))
        (OUT/('BATCH_'+group+'.json')).write_text(json.dumps(rows,indent=2)+'\n')
        print(rows[-1],flush=True)

if __name__=='__main__':run(sys.argv[1])

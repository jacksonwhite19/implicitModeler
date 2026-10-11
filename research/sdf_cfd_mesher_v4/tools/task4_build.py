"""Resume the rolling build only from its last atomically certified front."""
from pathlib import Path
import subprocess,sys,json,time,os
ROOT=Path(__file__).resolve().parents[1];OUT=Path(os.environ.get('TASK4_OUTPUT_DIRECTORY',str(ROOT/'results/task4'))).resolve()
if not OUT.is_relative_to((ROOT/'results/task4').resolve()):raise ValueError('Task4OutputScope')

def run():
    OUT.mkdir(parents=True,exist_ok=True)
    operations=json.loads((OUT/'BUILD_OPERATIONS.json').read_text()) if (OUT/'BUILD_OPERATIONS.json').exists() else []
    if not (OUT/'PREFLIGHT.json').exists():
        from task4_preflight import audit,git
        import datetime
        preflight=audit();preflight.update(starting_head=git('rev-parse','HEAD'),starting_status=git('status','--porcelain'),started_utc=datetime.datetime.now(datetime.timezone.utc).isoformat())
        (OUT/'PREFLIGHT.json').write_text(json.dumps(preflight,sort_keys=True,indent=2)+'\n')
    else:
        # A checkpoint restart must recheck frozen sources and prior results;
        # the old preservation snapshot alone is not evidence of current bytes.
        from task4_preflight import audit
        audit()
    if not (OUT/'build_A/CURRENT.json').exists():
        for name in ('task4_initial_scan.py','task4_correct_first.py','task4_warning_cache.py'):
            with (OUT/(name+'.log')).open('w') as log:
                p=subprocess.run([sys.executable,'-B',str(ROOT/'tools'/name)],stdout=log,stderr=subprocess.STDOUT)
            if p.returncode:return p.returncode
    pointer=json.loads((OUT/'build_A/CURRENT.json').read_text())
    for layer in range(pointer['layer']+1,21):
        commands=[]
        if layer>1:commands.append(('prepare','task4_prepare_next.py'))
        commands.append(('certify','task4_certify_candidate.py'))
        for phase,tool in commands:
            started=time.perf_counter();print('START',layer,phase,flush=True)
            with (OUT/f'build_layer_{layer:02}_{phase}.log').open('w') as log:
                p=subprocess.run([sys.executable,'-B',str(ROOT/'tools'/tool),'--layer',str(layer)],stdout=log,stderr=subprocess.STDOUT)
            operations.append(dict(layer=layer,phase=phase,exit_code=p.returncode,seconds=time.perf_counter()-started))
            (OUT/'BUILD_OPERATIONS.json').write_text(json.dumps(operations,indent=2)+'\n')
            print('END',layer,phase,p.returncode,flush=True)
            if p.returncode:return p.returncode
    print('TWENTY_LAYERS_COMMITTED: starting final qualification',flush=True)
    started=time.perf_counter()
    with (OUT/'final_validation.log').open('w') as log:
        p=subprocess.run([sys.executable,'-B',str(ROOT/'tools/task4_final_validation.py')],stdout=log,stderr=subprocess.STDOUT)
    operations.append(dict(layer=20,phase='final_qualification',exit_code=p.returncode,seconds=time.perf_counter()-started))
    (OUT/'BUILD_OPERATIONS.json').write_text(json.dumps(operations,indent=2)+'\n')
    return p.returncode

if __name__=='__main__':sys.exit(run())

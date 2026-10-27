"""Resume a prepared next layer without repeating its finished local search."""
from task4_build import ROOT,OUT,run
import subprocess,sys,json,argparse,time

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--layer',required=True,type=int);layer=parser.parse_args().layer
    pointer=json.loads((OUT/'build_A/CURRENT.json').read_text());assert pointer['layer']==layer-1
    assert (OUT/f'LAYER{layer}_CANDIDATE.npz').exists()
    started=time.perf_counter()
    with (OUT/f'build_layer_{layer:02}_certify_resume.log').open('w') as log:
        result=subprocess.run([sys.executable,'-B',str(ROOT/'tools/task4_certify_candidate.py'),'--layer',str(layer)],stdout=log,stderr=subprocess.STDOUT)
    (OUT/f'LAYER{layer}_RESUME_TIMING.json').write_text(json.dumps(dict(seconds=time.perf_counter()-started,exit_code=result.returncode),indent=2)+'\n')
    if result.returncode:sys.exit(result.returncode)
    sys.exit(run())

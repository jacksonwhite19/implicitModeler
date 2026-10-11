"""Preflight measurements; never represented as production qualification."""
import sys
sys.dont_write_bytecode=True
from pathlib import Path
import geometry_family_authority as a
import generalize_local_remesh as g

OUT=Path(__file__).resolve().parents[1]/'results/broader_geometry_family_qualification'

def run():
    # Freeze every sweep member before computing any result.
    manifest=OUT/'PARAMETER_DESIGN.json'
    specs=a.cohort()
    if manifest.exists():
        import json
        assert json.loads(manifest.read_text())['cases']==specs
    else:g.save(manifest,dict(cases=specs,reason='Uniform physical-scale sensitivity with fixed h1 and fixed wall incidence; all factors applied to all eight constructions',factors=[.25,.5,1.]))
    rows=[]
    for spec in specs:
        x,f=a.build(spec);fronts=[];history=[]
        for layer,target in enumerate(g.q.load_schedule().targets,1):
            fronts.append(a.front(spec,target))
            p,c,o,z,t=g.validation(x,f,fronts,[layer]*len(x),False)
            sdf=a.sdf_record(spec,fronts[-1],target);quality=z['quality']
            history.append(dict(layer=layer,quality_passed=quality['passed'],sdf=sdf,
                failures=quality['failures'],minimum_determinant=min(r['determinant'] for r in quality['cells']),
                minimum_weight=min(r['weight'] for r in quality['faces'])))
            if not quality['passed'] or not sdf['admitted']:break
        row=dict(case=spec['name'],history=history,last_examined_layer=layer,all_nominal_numeric_passed=layer==20 and quality['passed'] and sdf['admitted'],scope='Preflight only: no exact certificate, no production acceptance')
        rows.append(row);g.save(OUT/'PREFLIGHT.json',rows)
        print(spec['name'],layer,quality['passed'],sdf['admitted'],flush=True)

if __name__=='__main__':run()

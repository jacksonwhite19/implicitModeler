"""Replay current first-layer metric implementation against committed inputs."""
import sys
sys.dont_write_bytecode=True
import json,time
import numpy as np
import psutil
from first_layer_determinant_trace import OUT,g,production,geometry
import first_layer_metric_state as metric


def run():
    results=[]
    for spec in production.cohort():
        name=spec['name'];label='B' if name=='separated_regions_production' else 'A'
        path=OUT/name/label/'FIRST_LAYER.json'
        if not path.exists():raise ValueError('FirstLayerEvidenceNotReady:'+name)
        expected=json.loads(path.read_text())['quality']
        x,tri=geometry.build(spec);high=geometry.front(spec,g.q.load_schedule().targets[0])
        tick=time.perf_counter();p,c,o,checked,times=g.validation(x,tri,[high],np.ones(len(x),int),False)
        base=time.perf_counter()-tick
        prior=json.loads((OUT.parents[1]/'results/broader_production_scale_geometry'/name/'D/FIRST_LAYER.json').read_text())['quality']
        assert g.s.r.w.clean(checked['quality'])==prior
        del prior
        tick=time.perf_counter();contract=metric.contract(p,c,o,{})
        closure=time.perf_counter()-tick
        tick=time.perf_counter();checked=metric.apply(checked,p,c,o)
        applicable=time.perf_counter()-tick
        assert g.s.r.w.clean(checked['quality'])==expected
        results.append(dict(case=name,all_quality_rows_exact=True,exposed_baseline_rows_exact=True,exposed_validation_seconds=base,
            role_contract_seconds=closure,applicable_metric_seconds=applicable,
            peak_rss=psutil.Process().memory_info().peak_wset,
            scope='Current optimized implementation; exact equality permits reuse of unchanged first-layer certificates'))
        g.save(OUT/'CURRENT_NUMERIC_REPLAY.json',results)
    spec=geometry.definitions()[0];x,tri=geometry.build(spec)
    p,c,o,checked,_=g.validation(x,tri,[geometry.front(spec,g.q.load_schedule().targets[0])],np.ones(len(x),int),False)
    checked=metric.apply(checked,p,c,o)
    failures=sum(r['determinant']<.0011 for r in checked['quality']['cells'])
    assert failures and not checked['quality']['passed']
    g.save(OUT/'KNOWN_INVALID.json',dict(case=spec,wall_unchanged=True,h1=g.q.load_schedule().targets[0],
        determinant_min=min(r['determinant'] for r in checked['quality']['cells']),failures=failures,rejected=True))

def local_probe():
    from local_correction_validation import LocalCorrectionValidation
    spec=production.cohort()[0];x,tri=geometry.build(spec)
    high=geometry.front(spec,g.q.load_schedule().targets[0]);levels=np.ones(len(x),int)
    p,c,o,checked,_=g.validation(x,tri,[high],levels,False)
    checked=metric.apply(checked,p,c,o)
    evaluator=LocalCorrectionValidation(x,tri,[high],levels,checked)
    trial=high.copy();trial[0]+=(high[0]-x[0])*.01
    tick=time.perf_counter();pp,cc,oo,full,_=g.validation(x,tri,[trial],levels,False)
    full=metric.apply(full,pp,cc,oo);full_seconds=time.perf_counter()-tick
    tick=time.perf_counter();local=evaluator.evaluate(pp);local_seconds=time.perf_counter()-tick
    assert local['quality']==full['quality']
    g.save(OUT/'LOCAL_RECHECK.json',dict(cells=len(c),changed_vertices=1,
        full_seconds=full_seconds,local_seconds=local_seconds,quality_rows_exact=True))

if __name__=='__main__':
    if '--local' in sys.argv:local_probe()
    else:run()

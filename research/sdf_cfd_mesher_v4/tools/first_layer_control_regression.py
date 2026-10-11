"""Revalidate immutable accepted controls; never regenerate their authority."""
import sys
sys.dont_write_bytecode=True
import time
import numpy as np
from first_layer_determinant_trace import OUT,g,geometry
from run_geometry_family import AnalyticProvider
from sdf_cfd_mesher_v4.advancing_front.mixed_front_journal import MixedFrontJournal
from sdf_cfd_mesher_v4.advancing_front.mixed_front_runner import MixedFrontRunner
from sdf_cfd_mesher_v4.advancing_front.mixed_front import scientific_hash,rebuild_dependencies
from first_layer_metric_state import receipt


def box(final=False):
    tick=time.perf_counter();root=OUT.parents[1]
    spec=next(s for s in geometry.cohort() if s['name']=='rounded_box__scale_0p25')
    folder=OUT/('rounded_box_regression_final' if final else 'rounded_box_regression');p=AnalyticProvider(spec,folder)
    previous=MixedFrontJournal(root/'results/broader_production_scale_geometry/BOUNDED_REGRESSION'/spec['name']/'A/checkpoints').restart()
    fronts,levels=p.geometry(previous)
    points,cells,outer,checked,_=g.validation(p.x,p.tri,fronts,levels,False)
    old_quality=checked['quality']
    assert old_quality['passed']
    old_values=g.q.array_hash(np.asarray([r['determinant'] for r in old_quality['cells']]))
    checked=p.candidate_quality(checked,fronts,levels)
    checked=p.certify_candidate(fronts,levels,checked)
    assert checked['passed']
    state=p.pack(fronts,levels);p.bind(state,checked)
    rebuild_dependencies(None,state,p.schedule.targets.tolist())
    state['quality']['first_prism_incidence']=receipt(points,cells,outer,state['versions'])
    assert state['wall']==previous['wall'] and state['first_layer']==previous['first_layer']
    runner=MixedFrontRunner(folder/'checkpoints',p);restored=runner.initialize(state)
    assert scientific_hash(restored)==scientific_hash(state)
    assert p.validate_initial_commit(restored)
    g.save(folder/'RESULT.json',dict(passed=True,wall_exact=True,first_layer_exact=True,
        preserved_shell_local_determinants=old_values,prospective_minimum=state['quality']['first_prism_incidence']['minimum'],
        restart_exact=True,post_commit_validated=True,seconds=time.perf_counter()-tick,
        scope='Re-admission of sealed corrected geometry; no change to its correction history or coordinates. Does not claim nominal regeneration uses obsolete exposed-cap correction decisions.'))


def sf2():
    import broader_sf2_control as control
    control.OUT=OUT
    control.run()


def sf2_metrics():
    import json
    from sdf_cfd_mesher_v4.geometry.front import load_sf2_wall,build_topology
    from sdf_cfd_mesher_v4.advancing_front.rolling_metrics import evaluate_layer
    from sdf_cfd_mesher_v4.common.authority import array_hash
    tick=time.perf_counter();source=OUT.parents[1]/'results/task4l/fresh'
    front=load_sf2_wall();top=build_topology(front.triangles,len(front.coordinates))
    with np.load(source/'LAYER1_CANDIDATE.npz') as z:high=z['coordinates']
    result=evaluate_layer(top,front.coordinates,high,1,None)
    hashes={family:{k:array_hash(v) for k,v in result[family].items() if isinstance(v,np.ndarray)} for family in ('cells','sides','caps')}
    expected=json.loads((source/'LAYER_01_METRIC_ARRAY_HASHES.json').read_text())
    assert hashes==expected
    assert result['gate']['passed']
    g.save(OUT/'SF2/FIRST_LAYER_METRICS.json',dict(passed=True,all_arrays_exact=True,
        hashes=hashes,gate=result['gate'],numpy=np.__version__,seconds=time.perf_counter()-tick))

if __name__=='__main__':
    if sys.argv[1]=='box':box('--final' in sys.argv)
    elif sys.argv[1]=='sf2':sf2()
    elif sys.argv[1]=='sf2-metrics':sf2_metrics()

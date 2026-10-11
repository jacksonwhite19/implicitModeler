"""Single shallow shell-side feasibility trial; never runs tet-core repair."""
import json,time
from collections import Counter
import numpy as np
from bent_handoff_inputs import OUT
from fixed_core_study import save
from core_handoff_audit import surface
from geometry_family_authority import Source
import local_termination_topology as top
from upper_shell_handoff_patch import propose,publishable
from bent_handoff_study import event

def run():
    tick=time.perf_counter();z=np.load(OUT/'SEALED_GEOMETRY.npz');p=z['points'];wall=z['wall'];n=int(wall.max())+1
    cells=json.loads((OUT/'CELL_LINEAGE.json').read_text());field=np.load(OUT/'TERMINAL_FIELDS.npz')
    a=surface(field['points'],field['faces']);j=int(np.argmax(a['edge_length']));ends=a['edges'][j]
    global_ids=field['levels']*n+field['source'];edge=global_ids[ends];outer=[tuple(f) for f in global_ids[field['faces']]]
    spec=json.loads((OUT/'SOURCE.json').read_text())['spec'];source=Source(spec);schedule=json.loads((OUT/'SCHEDULE.json').read_text())
    levels=field['levels'][ends]
    if len(set(levels))!=1:raise ValueError('OneTargetEdgeRequired')
    target=float(schedule['nominal_targets'][int(levels[0])-1]);mid=p[edge].mean(0);sample=source.evaluate(mid[None]);new=mid+(target-sample.phi[0])*sample.normal[0]
    admission=source.evaluate(new[None]);assert abs(admission.phi[0]-target)<=admission.source_error_bound[0]
    np.savez_compressed(OUT/'PATCH_REPRODUCER.npz',edge=edge,new_point=new,target=target)
    event('shell_patch_start',edge=edge.tolist())
    pp,cc,oo,delta=propose(p,cells,outer,edge,new)
    owners,ownership=top.ownership(cc,oo,[tuple(reversed(f)) for f in wall])
    # Build neighbor set once, without transitive growth through iteration.
    replacement=set(delta['replacement']);active=replacement|{i for rows in owners.values() if any(j in replacement for j,f in rows) for i,f in rows}
    quality=top.quality(pp,cc,owners,1,active_cells=active)
    save(OUT/'PATCH_QUALITY.json',quality)
    beforeowners,beforeownership=top.ownership(cells,outer,[tuple(reversed(f)) for f in wall])
    parents=set(delta['removed']);beforeactive=parents|{i for rows in beforeowners.values() if any(j in parents for j,f in rows) for i,f in rows}
    baseline=top.quality(p,cells,beforeowners,1,active_cells=beforeactive);save(OUT/'PATCH_BASELINE_QUALITY.json',baseline)
    result=dict(strategy='Terminal edge SDF midpoint bisection; full edge-star cone subdivision; external quads preserved as pyramid bases',
        selection='Longest terminal edge, stable array order; no cell/source ID rule',delta=delta,
        original_parent_cells=delta['removed'],replacement_types=dict(Counter(c['kind'] for c in cc if c['id'] in replacement)),
        ownership=ownership,vertex_links=top.vertex_links(oo),shell_quality=quality['passed'],
        failures=Counter(z['kind'] for z in quality['failures']),
        minimum_determinant=min(r['determinant'] for r in quality['cells']),minimum_weight=min(r['weight'] for r in quality['faces']),
        minimum_ratio=min(r['symmetric_volume_ratio'] for r in quality['faces']),maximum_skew=max(r['skewness'] for r in quality['faces']),
        maximum_aspect=max(r['aspect'] for r in quality['cells']),
        target=target,target_residual=float(abs(admission.phi[0]-target)),source_error_bound=float(admission.source_error_bound[0]),
        wall_first_layer_exact=bool(np.array_equal(p[:2*n].view('u8'),pp[:2*n].view('u8'))),
        previous_cells_unchanged=len(cells)-len(parents),baseline_quality_passed=baseline['passed'],
        continuous_certificate='Not measured; rejected at current shell hard gates',
        local_core_before_after='Not measured; no scientifically valid repaired shell patch admitted',
        publication=False,publishable=publishable(dict(shell_quality=quality['passed'])),seconds=time.perf_counter()-tick)
    np.savez_compressed(OUT/'REJECTED_PATCH.npz',points=pp[len(p):],replacement=np.array([c for c in cc if c['id'] in replacement],dtype=object))
    save(OUT/'PATCH_RESULT.json',result);event('shell_patch_end',passed=quality['passed'],failures=dict(result['failures']),seconds_stage=result['seconds'])
    print(json.dumps({k:v for k,v in result.items() if k not in ('ownership','delta')},default=lambda z:z.tolist()),flush=True)

if __name__=='__main__':run()

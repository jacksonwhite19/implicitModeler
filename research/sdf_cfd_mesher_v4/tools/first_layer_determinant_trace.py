"""Reproduce area-tensor states without changing geometry or admission."""
import sys
sys.dont_write_bytecode = True
import json, time
from pathlib import Path
import numpy as np
import mpmath as mp
import generalize_local_remesh as g
import geometry_family_authority as geometry
import production_scale_authority as production

OUT = Path(__file__).resolve().parents[1]/'results/first_layer_determinant_applicability'


def tensor_state(areas, selected):
    s = areas[np.asarray(selected, bool)]
    if not len(s):
        return dict(selected=selected, determinant=0., high_precision='0', normalized=[], tensor=[])
    avg = np.linalg.norm(s, axis=1).mean()
    normalized = s/avg
    tensor = normalized.T@normalized
    with mp.workdps(80):
        exact = [[mp.mpf(float(v)) for v in row] for row in s]
        scale = sum(mp.sqrt(sum(v*v for v in row)) for row in exact)/len(exact)
        matrix = mp.matrix([[sum(row[i]*row[j]/scale**2 for row in exact) for j in range(3)] for i in range(3)])
        high = mp.nstr(abs(mp.det(matrix)), 75)
    return dict(selected=selected, determinant=float(np.linalg.det(tensor)), high_precision=high,
                average_area=float(avg), normalized=normalized.tolist(), tensor=tensor.tolist(),
                contributions=[np.outer(v,v).tolist() for v in normalized])


def reproduce(points, cells, outer, selected):
    owners = {}
    for c in cells:
        for f in c['faces']: owners.setdefault(tuple(sorted(f)), []).append(c['id'])
    interface = {tuple(sorted(f)) for f in outer}
    result = []
    for c in cells:
        if c['id'] not in selected: continue
        geom = g.t.geometry(points,c)
        local = [len(owners[tuple(sorted(f))]) == 2 for f in c['faces']]
        complete = [v or tuple(sorted(f)) in interface for v,f in zip(local,c['faces'])]
        # Attach a conformal successor solely as an incidence probe. Its
        # coordinates are not committed or used by a pair-quality evaluator.
        neighbor_ids=list(c['vertices'][3:])+list(range(len(points),len(points)+3))
        neighbor=g.t.cell(neighbor_ids,c['column'],2)
        probe={key:list(value) for key,value in owners.items()}
        for f in neighbor['faces']:probe.setdefault(tuple(sorted(f)),[]).append(neighbor['id'])
        actual=[len(probe[tuple(sorted(f))])==2 for f in c['faces']]
        assert actual==complete
        states = dict(candidate_only=tensor_state(geom['areas'], [False]*len(local)),
                      shell_local=tensor_state(geom['areas'],local),
                      committed_first_layer=tensor_state(geom['areas'],local),
                      prospective_cap=tensor_state(geom['areas'],complete),
                      complete_neighbor_determinant=tensor_state(geom['areas'],actual))
        result.append(dict(cell=c, coordinates=points[c['vertices']].tolist(),
            area_vectors=geom['areas'].tolist(), face_centres=geom['face_centres'].tolist(),
            cell_centre=geom['centre'].tolist(), owners=[owners[tuple(sorted(f))] for f in c['faces']],
            outer_roles=[tuple(sorted(f)) in interface for f in c['faces']], states=states,
            actual_neighbor_incidence=neighbor,actual_neighbor_mask=actual,
            wall_edges=[float(np.linalg.norm(points[c['vertices'][i]]-points[c['vertices'][(i+1)%3]])) for i in range(3)],
            complete_neighbor_scope='Exact determinant face-role closure only; no invented neighbor centroid or pair metric'))
    return result


def run_production():
    OUT.mkdir(parents=True,exist_ok=True)
    for spec in production.cohort():
        tick=time.perf_counter()
        x,tri=geometry.build(spec);high=geometry.front(spec,g.q.load_schedule().targets[0])
        p,c,o=g.s.construct(x,tri,[high],np.ones(len(x),int))
        prior=Path(__file__).resolve().parents[1]/'results/broader_production_scale_geometry'/spec['name']/'D/FIRST_LAYER.json'
        rows=json.loads(prior.read_text())['quality']['cells']
        worst=min(rows,key=lambda r:r['determinant'])['id']
        evidence=reproduce(p,c,o,{worst})
        g.save(OUT/(spec['name']+'_TRACE.json'),dict(spec=spec,reproducers=evidence,
            wall_hash=g.q.array_hash(x),topology_hash=g.q.array_hash(tri),first_layer_hash=g.q.array_hash(high),
            prior_failure_count=sum(r['determinant']<.0011 for r in rows),seconds=time.perf_counter()-tick))
        print(spec['name'],evidence[0]['states']['shell_local']['determinant'],evidence[0]['states']['prospective_cap']['determinant'],flush=True)

def run_controls():
    from sdf_cfd_mesher_v4.geometry.front import load_sf2_wall
    from sdf_cfd_mesher_v4.advancing_front.mixed_front_journal import MixedFrontJournal
    from run_geometry_family import AnalyticProvider
    root=OUT.parents[1]
    wall=load_sf2_wall();x=wall.coordinates;tri=wall.triangles
    with np.load(root/'results/task4l/fresh/LAYER1_CANDIDATE.npz') as z: high=z['coordinates']
    cq=g.q.legacy_modules()['component_quality']
    prisms=np.concatenate([x[tri],high[tri]],axis=1)
    values=cq.determinant(prisms,1);worst=int(np.argmin(values))
    selected=set(np.flatnonzero(np.any(np.isin(tri,tri[worst]),axis=1)).tolist())
    n=len(x);cells=[g.t.cell(np.r_[tri[i],tri[i]+n],i,1) for i in sorted(selected)]
    outer=[tuple(map(int,tri[i]+n)) for i in sorted(selected)]
    g.save(OUT/'SF2_TRACE.json',dict(reproducers=reproduce(np.concatenate([x,high]),cells,outer,{1000000+worst}),
        authoritative_minimum=float(values.min()),authoritative_failures=int((values<.0011).sum()),
        authoritative_values=g.q.array_hash(values),first_layer_hash=g.q.array_hash(high),wall_hash=g.q.array_hash(x),
        scope='Preserved accepted SF2 first layer; unchanged production vectorized determinant implementation'))
    spec=next(s for s in geometry.cohort() if s['name']=='rounded_box__scale_0p25')
    provider=AnalyticProvider(spec,OUT/'box_control')
    source=root/'results/broader_production_scale_geometry/BOUNDED_REGRESSION'/spec['name']/'A/checkpoints'
    state=MixedFrontJournal(source).restart();fronts,levels=provider.geometry(state)
    p,c,o,checked,_=g.validation(provider.x,provider.tri,fronts,levels,False)
    rows=checked['quality']['cells'];worst=min(rows,key=lambda r:r['determinant'])['id']
    g.save(OUT/'ROUNDED_BOX_TRACE.json',dict(reproducers=reproduce(p,c,o,{worst}),
        shell_local_minimum=min(r['determinant'] for r in rows),shell_local_passed=checked['quality']['passed'],
        first_layer_hash=g.q.array_hash(fronts[0]),wall_hash=g.q.array_hash(provider.x),
        scope='Immutable corrected first-layer authority, not regeneration of a new nominal first layer'))

if __name__=='__main__':
    if '--controls' in sys.argv:run_controls()
    else:run_production()

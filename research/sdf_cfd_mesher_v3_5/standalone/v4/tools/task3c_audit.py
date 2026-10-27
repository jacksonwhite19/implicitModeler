"""Audit saved trials and exact local control. Reuses, never repeats, old polls."""
from task3_repair_study import *
from sdf_cfd_mesher_v4.advancing_front.cap_diagnostics import *
from task3c_io import read as read_evidence
OUT=ROOT/'results/task3c'

def saved(name):
    paths=[ROOT/f'results/task3/{name}.json'] if name=='component_00' else [ROOT/f'results/task3_certificate_repair/{name}.json']
    if name=='component_01':paths.insert(0,ROOT/f'results/task3/{name}.json')
    return paths,[e for p in paths for e in json.loads(p.read_text())['history']]

def closure_plan(front,top,x,seed,max_vertices=512,max_columns=4096):
    active=set(map(int,seed));steps=[]
    # Only incident/one-ring columns are visited. No full-shell metric pass.
    for iteration in range(512):
        affected=set(int(c) for v in active for c in top.vertex_triangles.row(v));core=set(affected)
        for e in np.unique(top.triangle_edges[sorted(affected)]):core.update(map(int,top.edge_triangles.row(e)))
        if len(core)>max_columns:return dict(steps=steps,complete=False,reason='ColumnComputationalBound',active=sorted(active))
        locked=set()
        for c in sorted(core):
            ids=top.triangles[c]
            for j,v in enumerate(ids):
                if int(v) not in active and bottom_corner(front.coordinates[ids],x[ids],j)['sign']<=0:locked.add(int(v))
        steps.append(dict(iteration=iteration,active=len(active),columns=len(core),new_locked_vertices=sorted(locked)))
        if not locked:return dict(steps=steps,complete=True,active=sorted(active),reason='All nonpositive bottom corners in verification scope have active control; this is not feasibility')
        if len(active|locked)>max_vertices:return dict(steps=steps,complete=False,reason='VertexComputationalBound',active=sorted(active),pending=sorted(locked))
        active|=locked
    raise AssertionError('UnreachableIterationBound')

def run(replay=False):
    start=time.perf_counter();ctx=load();front,top,sdf,schedule,coords,normals,*_=ctx;out=[]
    for spec in [s for s in components() if s['name'] in ('component_00','component_01','component_02','component_04')]:
        name=spec['name'];paths,h=saved(name);rows=[]
        for ring in range(4):
            p=build_patch(top,spec['columns'],active_rings=ring,max_vertices=160,max_columns=640)
            ev=[e for e in h if e.get('scope')==ring and 'field' in e]
            if ring>=2:
                rr=read_evidence(OUT/f'{name}_ring{ring}.json');ev=[e for e in rr['history'] if 'field' in e]
            byfield={}
            for e in ev:
                if 'projection' in e:byfield.setdefault(e['field'],e)
            dirs=np.array([np.array(e['projection']['tangential_displacement']).ravel() for e in byfield.values()])
            cov=direction_coverage(dirs);cov['field_order']=list(byfield)
            tr=tangent_rank(sdf.evaluate(coords[0,p.active]).normal);locked=locked_corners(front,top,coords[0],p,normals[0])
            events=[e for e in ev if 'candidate_gate' in e]
            rows.append(dict(ring=ring,patch=p.record(),tangent_rank=tr,coverage=cov,
                original_paths=[str(q.relative_to(ROOT)) for q in paths] if ring<2 else [f'results/task3c/{name}_ring{ring}.json'],
                trial_count=len(ev),reason_counts=dict(collections.Counter(e.get('reason') for e in ev)),
                radius_values=sorted(set(e['radius'] for e in ev)),
                displacement_range=[min(e['candidate_displacement'] for e in events),max(e['candidate_displacement'] for e in events)],
                unique_coordinate_hashes=len({e['coordinate_hash'] for e in events}),
                rejecting_gate_trial_counts={g:sum(e['candidate_gate']['violations'][g]>0 for e in events) for g in events[0]['candidate_gate']['violations']},
                lowest_bad_jacobian_count=min(e['candidate_gate']['violations']['jacobian'] for e in events),
                best_current_min_jacobian=max(e['candidate_gate']['extrema']['jacobian']['min'] for e in events),
                best_current_min_weight=max(e['candidate_gate']['extrema']['weight']['min'] for e in events),
                best_current_min_determinant=max(e['candidate_gate']['extrema']['determinant']['min'] for e in events),
                locked_bottom_corners=locked,
                invariant_corner_count=sum(e['invariant_under_all_active_moves'] for e in locked),
                invariant_affected_corner_count=sum(e['invariant_under_all_active_moves'] and e['affected_column'] for e in locked),
                active_bad_corner_count=sum(e['active'] for e in locked),
                active_controlled_bad_corner_count=sum(e['active'] and e['exact']['tangent_controls_corner'] for e in locked)))
            print(name,ring,'rank',tr['independent_rank'],cov['rank'],'locked',rows[-1]['invariant_corner_count'],rows[-1]['invariant_affected_corner_count'],flush=True)
        plan=closure_plan(front,top,coords[0],build_patch(top,spec['columns']).active)
        out.append(dict(component=name,source_hashes={str(q.relative_to(ROOT)):file_hash(q) for q in paths},scopes=rows,closure_plan=plan))
        print(name,'closure',plan['complete'],len(plan['active']),plan['reason'],flush=True)
    h=scientific_hash(out)
    if replay:assert scientific_hash(json.loads((OUT/'AUDIT.json').read_text()))==h
    else:(OUT/'AUDIT.json').write_text(json.dumps(plain(out),indent=2,sort_keys=True)+'\n')
    (OUT/('AUDIT_REPLAY.json' if replay else 'AUDIT_HASH.json')).write_text(json.dumps(dict(scientific_hash=h),indent=2)+'\n')
    (OUT/('AUDIT_REPLAY_TIMINGS.json' if replay else 'AUDIT_TIMINGS.json')).write_text(json.dumps(dict(seconds=time.perf_counter()-start,peak_rss=psutil.Process().memory_info().peak_wset),indent=2)+'\n')
if __name__=='__main__':run('--replay' in sys.argv)

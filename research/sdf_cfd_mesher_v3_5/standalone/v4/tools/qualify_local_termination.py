"""Bounded topology-mechanics study, explicitly authorized fixture only."""
import json,time,hashlib,sys
from pathlib import Path
import local_termination_topology as t
r=t.r;np=t.np;q=t.q
OUT=r.r.g.ROOT/'results/local_layer_termination_topology'
def save(name,value):
    (OUT/name).write_text(json.dumps(r.w.clean(value),sort_keys=True,separators=(',',':'),allow_nan=False)+'\n',encoding='utf8',newline='\n')

def run():
    start=time.perf_counter();case,prefix,history,previous,seed,targets,high,record,timing=r.reproduce()
    old=r.r.load(r.OUT/'BASELINE.json');coll=r.r.load(r.OUT/'COLLISION_RESULTS.json')[0]['certificate'];stopped=set()
    for e in old['numeric_events']:stopped.update(map(int,case.f[e['columns']].ravel()))
    for e in coll['failures']:
        for side in e['pair']:stopped.update(map(int,case.top.edges[side%len(case.top.edges)]))
    rows=[]
    for ring in range(3):
        if ring:
            stopped|={int(n) for v in tuple(stopped) for n in case.top.vertex_neighbors.row(v)}
        tick=time.perf_counter();points,cells,outer,m=t.construct(case.x,case.f,prefix,high,stopped)
        owners,own=t.ownership(cells,outer,[tuple(reversed(tri)) for tri in case.f])
        quality=t.quality(points,cells,owners,4)
        mixed={c['id'] for c in cells if c['layer']==5 and c['kind']!='prism'}
        cavity=[dict(vertices=next(f for i,f in rr if i in mixed),owners=[i for i,f in rr])
            for key,rr in sorted(owners.items()) if sum(i in mixed for i,f in rr)==1]
        save(f'candidate_{ring}_cavity.json',dict(transition_cell_ids=sorted(mixed),boundary=cavity,
            definition='Oriented boundary of the union of proposed non-prism cells; original lower and neighbor faces unchanged'))
        kinds=t.Counter(c['kind'] for c in cells if c['layer']==5)
        summary=dict(ring=ring,stopped_vertices=len(stopped),terminated=len(m['terminated_columns']),transition=len(m['transition_columns']),
            continuing=len(m['continuing_columns']),counts=kinds,ownership=own,quality_passed=quality['passed'],
            failure_counts=t.Counter(z['kind'] for z in quality['failures']),seconds=time.perf_counter()-tick)
        rows.append(summary);save(f'candidate_{ring}_map.json',m);save(f'candidate_{ring}_quality.json',quality)
        save(f'candidate_{ring}_topology.json',dict(new_cells=[c for c in cells if c['layer']==5],outer=outer,
            ownership=own,lower_coordinate_hash=q.array_hash(points[:5*len(case.x)]),preserved_coordinate_hash=q.array_hash(np.concatenate([case.x]+prefix))))
        print(summary,flush=True)
        if quality['passed']:break
    save('EXPLORATION.json',rows);save('RUNTIME.json',dict(seconds=time.perf_counter()-start,peak_rss=q.psutil.Process().memory_info().peak_wset))
def certify_selected():
    import termination_geometry_certificate as cert
    tick=time.perf_counter();case,prefix,history,previous,seed,targets,high,record,timing=r.reproduce()
    m=r.r.load(OUT/'candidate_1_map.json');points,cells,outer,m=t.construct(case.x,case.f,prefix,high,m['stopped_vertices'])
    owners,own=t.ownership(cells,outer,[tuple(reversed(tri)) for tri in case.f])
    keys=sorted(owners);faces=[owners[key][0][1] for key in keys];newids={c['id'] for c in cells if c['layer']==5}
    active=[i for i,key in enumerate(keys) if any(cid in newids for cid,f in owners[key])]
    result=cert.certify(points,faces,active);save('CONTINUOUS.json',result)
    save('CERTIFICATE_TIMING.json',dict(seconds=time.perf_counter()-tick,peak_rss=q.psutil.Process().memory_info().peak_wset))
    print('continuous',result['passed'],result['required_pairs'],len(result['failures']),flush=True)

def outer_and_analytic():
    import termination_geometry_certificate as cert
    tick=time.perf_counter();case,prefix,history,previous,seed,targets,high,record,timing=r.reproduce()
    m=r.r.load(OUT/'candidate_1_map.json');points,cells,outer,m=t.construct(case.x,case.f,prefix,high,m['stopped_vertices'])
    result=cert.certify(points,outer,range(len(outer)));save('OUTER_CERTIFICATE.json',result)
    spec=next(z for z in q.definitions(q.load_schedule().targets) if z['name']=='spherical_cavity')
    wall,faces,core=q.build(spec);sdf=q.FixtureSDF(spec);top,patch,foundation=q.foundation(wall,faces,core,sdf,spec)
    folder=r.r.g.ROOT/'results/layer_capacity_valid_prefix/spherical_cavity';prefix=[];checks=[]
    for layer in range(1,13):
        x,z=q.load_checkpoint(folder,layer,top.sha256);prefix.append(x);checks.append(z)
    points,cells,outer,m=t.construct(wall,faces,prefix,prefix[-1],range(len(wall)))
    owners,own=t.ownership(cells,outer,[tuple(reversed(tri)) for tri in faces]);quality=t.quality(points,cells,owners,12)
    outercert=cert.certify(points,outer,range(len(outer)))
    save('ANALYTIC_SMOKE.json',dict(name='spherical_cavity',last_valid=12,first_geometric_limit=13,termination_map=m,
        foundation=foundation,ownership=own,quality=quality,outer_certificate=outercert,
        lower_identity=q.array_hash(np.concatenate([wall]+prefix)),reconstructed_identity=q.array_hash(points[:13*len(wall)]),
        preserved_prisms=len(cells),new_cells=0,scope='Whole cavity reaches analytic radial limit; all columns stop at 12. Partial taper qualification uses the general-SDF case.',
        passed=own['passed'] and quality['passed'] and outercert['passed']))
    save('OUTER_ANALYTIC_TIMING.json',dict(seconds=time.perf_counter()-tick,peak_rss=q.psutil.Process().memory_info().peak_wset))
    print('analytic',own['passed'],quality['passed'],outercert['passed'],flush=True)

def cap_reproducer():
    import termination_geometry_certificate as cert
    case,prefix,history,previous,seed,targets,high,record,timing=r.reproduce();low=prefix[-1]
    pairs=r.r.load(OUT/'OUTER_CERTIFICATE.json')['failures'];rows=[]
    for row in pairs:
        a,b=row['pair'];points=cert.triangle_intersection_points(low,case.f[a],case.f[b])
        assert points and not set(case.f[a])&set(case.f[b])
        rows.append(dict(columns=[a,b],vertices=[case.f[a],case.f[b]],coordinates=[low[case.f[a]],low[case.f[b]]],
            exact_intersection_points=[[str(v) for v in x] for x in points],
            independent_method='Exact rational segment-plane clipping and triangle halfspaces',
            frozen_layer=4,shared_vertices=[],forbidden=True))
    save('FROZEN_CAP_REPRODUCER.json',dict(seed_side_certificate_passed=seed['gate']['certificate']['passed'],
        seed_side_required_pairs=seed['gate']['certificate']['required_pairs'],conflicts=rows,
        scope='New cap audit does not change the prior side-certificate mathematics or its recorded result.'))

def replay():
    names=['candidate_0_map.json','candidate_0_quality.json','candidate_0_topology.json',
        'candidate_1_map.json','candidate_1_quality.json','candidate_1_topology.json',
        'CONTINUOUS.json','OUTER_CERTIFICATE.json','ANALYTIC_SMOKE.json','FROZEN_CAP_REPRODUCER.json']
    before={n:r.r.load(OUT/n) for n in names};start=time.perf_counter()
    oldtime={n:r.r.load(OUT/n) for n in ('RUNTIME.json','CERTIFICATE_TIMING.json','OUTER_ANALYTIC_TIMING.json')}
    run();certify_selected();outer_and_analytic();cap_reproducer()
    after={n:r.r.load(OUT/n) for n in names};equal={n:q.identity(before[n])==q.identity(after[n]) for n in names}
    save('DETERMINISM.json',dict(equal=all(equal.values()),files=equal,first=q.identity(before),replay=q.identity(after)))
    save('QUALIFICATION_RUNTIME.json',dict(first=oldtime,replay_seconds=time.perf_counter()-start,peak_rss=q.psutil.Process().memory_info().peak_wset))
    assert all(equal.values()),equal

def topology_replay():
    names=[f'candidate_{i}_{kind}.json' for i in (0,1) for kind in ('map','quality','topology','cavity')]
    before={n:r.r.load(OUT/n) for n in names};run();after={n:r.r.load(OUT/n) for n in names}
    equal=q.identity(before)==q.identity(after);save('CAVITY_DETERMINISM.json',dict(equal=equal,scientific_hash=q.identity(after)));assert equal
if __name__=='__main__':
    if '--certificate' in sys.argv:certify_selected()
    elif '--outer-analytic' in sys.argv:outer_and_analytic()
    elif '--cap-reproducer' in sys.argv:cap_reproducer()
    elif '--replay' in sys.argv:replay()
    elif '--topology-replay' in sys.argv:topology_replay()
    else:run()

"""Local continuation, analytic regression and scientific replay."""
import time
import local_cavity_remesh as s
import cavity_volume_audit as v
t=s.t;q=s.q;r=s.r;np=s.np

def continuation(case,prefix,high,targets,z):
    base=np.array(z['vertex_depth']);seed=set(np.flatnonzero(base==3));near=set(seed);history=[];scenarios=[];distance={i:0 for i in seed}
    for ring in range(1,5):
        near|={int(n) for i in tuple(near) for n in case.top.vertex_neighbors.row(i)}
        active=sorted(i for i in near if base[i]==5)
        if not active:continue
        aset=set(active)
        for i in near:distance.setdefault(i,ring)
        if not any(all(int(i) in aset for i in f) for f in case.f):continue
        scenarios.append((ring,active,1.0,'ring'))
    eligible=sorted((max(distance.get(int(i),99) for i in f),col,list(map(int,f))) for col,f in enumerate(case.f) if all(base[int(i)]==5 for i in f))
    for dist,col,ids in eligible[:8]:
        for scale in (1.0,.5,.25):scenarios.append((dist,ids,scale,'single_neighbor_column_'+str(col)))
    for ring,active,scale,mode in scenarios:
        front=high.copy();ancestry=r.b.owner(case.x,case.spec);projection=[]
        for i in active:
            front[i:i+1],rr=r.b.sphere_project(high[i:i+1],float(targets[i]+scale*case.s.increments[5]),case.sdf,case.spec,ancestry[i:i+1]);projection.extend(rr)
        depth=base.copy();depth[active]=6
        points,cells,outer=s.construct(case.x,case.f,prefix+[high,front],depth)
        owners,own=t.ownership(cells,outer,[tuple(reversed(f)) for f in case.f]);quality=t.quality(points,cells,owners,5)
        row=dict(ring=ring,scale=scale,mode=mode,active=active,projection=projection,quality=quality,ownership=own,passed=False);history.append(row)
        print('continuation',ring,len(active),quality['passed'],flush=True)
        if quality['passed']:
            keys=sorted(owners);faces=[owners[k][0][1] for k in keys];ids=[i for i,k in enumerate(keys) if any(cid>=6000000 for cid,f in owners[k])]
            row['certificate']=s.cert.certify(points,faces,ids)
            row['volume']=v.audit(points,cells,outer,[tuple(map(int,x)) for x in case.f])
            row['passed']=row['certificate']['passed'] and row['volume']['interior_degree']['passed'] and row['volume']['exact_volume_balance']
            if row['passed']:
                row['new_cells']=[c for c in cells if c['layer']==6];row['front_hash']=q.array_hash(front);row['outer_hash']=q.identity(outer)
                break
    return dict(attempts=history,passed=any(z['passed'] for z in history),scope='One local layer-6 advance adjacent to rollback; no full remaining-stack study')

def analytic():
    spec=next(x for x in q.definitions(q.load_schedule().targets) if x['name']=='spherical_cavity')
    wall,faces,core=q.build(spec);top,patch,foundation=q.foundation(wall,faces,core,q.FixtureSDF(spec),spec)
    prefix=[q.load_checkpoint(s.ROOT/'results/layer_capacity_valid_prefix/spherical_cavity',i,top.sha256)[0] for i in range(1,13)]
    # The analytically invalid prospective layer 13 is deliberately not admitted.
    p,c,f=s.construct(wall,faces,prefix,[12]*len(wall));owners,own=t.ownership(c,f,[tuple(reversed(x)) for x in faces]);quality=t.quality(p,c,owners,12)
    cert=s.cert.certify(p,f,range(len(f)));volume=v.audit(p,c,f,[tuple(map(int,x)) for x in faces])
    return dict(passed=quality['passed'] and own['passed'] and cert['passed'] and volume['interior_degree']['passed'] and volume['exact_volume_balance'],
        map=[12]*len(faces),capacity_limit=13,quality=quality,ownership=own,outer_certificate=cert,volume=volume,
        unchanged_lower_hash=q.array_hash(p),scope='Same standard reconstruction at known valid analytic prefix; no previously invalid analytic cells admitted')

def run():
    st=time.perf_counter();case,prefix,history,previous,seed,targets,high,record,timing=r.reproduce();z=s.load(s.OUT/'SELECTED.json')
    a=continuation(case,prefix,high,targets,z);s.save('CONTINUATION.json',a);s.save('ANALYTIC_SMOKE.json',analytic())
    s.save('SMOKE_RUNTIME.json',dict(seconds=time.perf_counter()-st,peak_rss=q.psutil.Process().memory_info().peak_wset))
def channel():
    import qualify_branch_aware_capacity as b
    st=time.perf_counter();spec=next(z for z in q.definitions(q.load_schedule().targets) if z['name']=='narrowing_escape')
    x,f,top,patch,base,spec,sdf,prefix,previous=b.prepare(spec)
    p,c,outer=s.construct(x,f,prefix,[len(prefix)]*len(x));o,own=t.ownership(c,outer,[tuple(reversed(z)) for z in f]);quality=t.quality(p,c,o,len(prefix))
    s.save('CHANNEL_SMOKE.json',dict(name='narrowing_escape',last_layer=len(prefix),scope='Bounded whole-interface foundation check before local remesh',
        ownership=own,quality=quality,passed=False,remesh_performed=False,classification='SupportBlocked',
        reason='Whole-support quality fails before a clean local cavity is defined. No channel repair or generalization claimed.',seconds=time.perf_counter()-st))
if __name__=='__main__':
    import sys
    if '--channel' in sys.argv:channel()
    else:run()

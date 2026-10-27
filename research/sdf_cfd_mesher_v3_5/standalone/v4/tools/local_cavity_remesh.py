"""Bounded study rollback and standard mixed-cell re-extrusion, no trigger."""
import json,time
import termination_geometry_certificate as cert
t=cert.t;r=t.r;q=t.q;np=t.np
ROOT=r.r.g.ROOT;OUT=ROOT/'results/local_termination_cavity_remesh'
def load(p):return json.loads(p.read_text())
def save(n,z):(OUT/n).write_text(json.dumps(r.w.clean(z),sort_keys=True,separators=(',',':'))+'\n',encoding='utf8',newline='\n')

def construct(wall,faces,fronts,depth):
    """Rebuild all incidence for audit; only changed cells are replacements.

    Each vertex stops at an explicit existing front, at or above layer 1.
    Original upper layer cells have no preservation privilege.
    """
    n=len(wall);depth=np.asarray(depth,dtype=int)
    if len(depth)!=n or np.any(depth<1) or np.any(depth>len(fronts)):raise ValueError('InvalidVertexDepth')
    points=np.concatenate([wall]+fronts);lower=np.arange(n);cells=[]
    for layer in range(1,len(fronts)+1):
        upper=np.arange(n)+n*np.minimum(depth,layer)
        for col,f in enumerate(faces):
            z=t.cell(np.r_[lower[f],upper[f]],col,layer)
            if z:cells.append(z)
        lower=upper
    outer=[tuple(map(int,lower[f])) for f in faces]
    return points,cells,outer

def removal_manifest(old,new):
    newby={x['id']:x for x in new};oldby={x['id']:x for x in old}
    removed=[dict(cell=x,reason='Inside rollback/reconnection cavity') for x in old if newby.get(x['id'])!=x]
    added=[x for x in new if oldby.get(x['id'])!=x]
    return dict(removed=removed,replacements=added,preserved=[x['id'] for x in old if newby.get(x['id'])==x])

def run():
    start=time.perf_counter();case,prefix,history,previous,seed,targets,high,record,timing=r.reproduce()
    oldmap=load(ROOT/'results/local_layer_termination_topology/candidate_1_map.json')
    _,old,_,_=t.construct(case.x,case.f,prefix,high,oldmap['stopped_vertices'])
    conflicts=load(ROOT/'results/terminal_cap_conformity/FORENSICS.json')
    core=set(v for x in conflicts for tri in x['source_vertices'] for v in tri)
    rows=[];first_overlapped_layer=min(x['layer'] for x in conflicts)
    for depth in range(first_overlapped_layer-1,0,-1):
        active=set(core)
        for ring in range(4):
            if ring:active|={int(v) for u in tuple(active) for v in case.top.vertex_neighbors.row(u)}
            levels=np.full(len(case.x),5);levels[oldmap['stopped_vertices']]=4;levels[sorted(active)]=depth
            points,cells,outer=construct(case.x,case.f,prefix+[high],levels)
            owners,own=t.ownership(cells,outer,[tuple(reversed(f)) for f in case.f])
            quality=t.quality(points,cells,owners,max(1,depth))
            z=dict(depth=depth,ring=ring,active_vertices=sorted(active),ownership=own,quality_passed=quality['passed'],
                quality_failure_counts=t.Counter(x['kind'] for x in quality['failures']),failures=quality['failures'])
            rows.append(z);print(depth,ring,len(active),quality['passed'],z['quality_failure_counts'],flush=True)
            save('EXPLORATION.json',rows)
            if quality['passed']:
                c=cert.certify(points,outer,range(len(outer)));z['outer_passed']=c['passed'];z['outer_failures']=c['failures']
                save(f'outer_{depth}_{ring}.json',c);save('EXPLORATION.json',rows)
                if c['passed']:
                    save('SELECTED.json',dict(depth=depth,ring=ring,vertex_depth=levels.tolist(),outer=outer,cells=cells,ownership=own,quality=quality,
                        manifest=removal_manifest(old,cells),lower_wall_hash=q.array_hash(case.x),first_hash=q.array_hash(prefix[0])))
                    save('RUNTIME_SEARCH.json',dict(seconds=time.perf_counter()-start,peak_rss=q.psutil.Process().memory_info().peak_wset));return
    save('RUNTIME_SEARCH.json',dict(seconds=time.perf_counter()-start,peak_rss=q.psutil.Process().memory_info().peak_wset))
if __name__=='__main__':run()

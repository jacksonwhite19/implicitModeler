"""Complete fixture verification, replay and bounded core ingestion export."""
import sys,time
import local_cavity_remesh as s
import cavity_volume_audit as v
import verify_local_cavity_remesh as smoke
t=s.t;q=s.q;r=s.r;np=s.np

def run(replay=False):
    st=time.perf_counter();case,prefix,history,previous,seed,targets,high,record,timing=r.reproduce();z=s.load(s.OUT/'SELECTED.json')
    p,c,f=s.construct(case.x,case.f,prefix+[high],z['vertex_depth']);o,own=t.ownership(c,f,[tuple(reversed(x)) for x in case.f]);faces=[o[k][0][1] for k in sorted(o)]
    full=s.cert.certify(p,faces,range(len(faces)))
    if replay:
        oldmap=s.load(s.ROOT/'results/local_layer_termination_topology/candidate_1_map.json')
        core=sorted({v for x in s.load(s.ROOT/'results/terminal_cap_conformity/FORENSICS.json') for f0 in x['source_vertices'] for v in f0})
        levels=np.full(len(case.x),5);levels[oldmap['stopped_vertices']]=4;levels[core]=3
        assert levels.tolist()==z['vertex_depth']
        _,old,_,_=t.construct(case.x,case.f,prefix,high,oldmap['stopped_vertices'])
        assert q.identity(s.removal_manifest(old,c))==q.identity(z['manifest'])
        assert q.identity(full)==q.identity(s.load(s.OUT/'FULL_FACE_CERTIFICATE.json'))
        quality=t.quality(p,c,o,3);assert q.identity(quality)==q.identity(z['quality'])
        assert q.identity(c)==q.identity(z['cells']) and q.identity(f)==q.identity(z['outer'])
        volume=v.audit(p,c,f,[tuple(map(int,x)) for x in case.f]);assert q.identity(volume)==q.identity(s.load(s.OUT/'VOLUME_AUDIT.json'))
        continuation=smoke.continuation(case,prefix,high,targets,z);assert q.identity(continuation)==q.identity(s.load(s.OUT/'CONTINUATION.json'))
        analytic=smoke.analytic();assert q.identity(analytic)==q.identity(s.load(s.OUT/'ANALYTIC_SMOKE.json'))
        s.save('DETERMINISM.json',dict(equal=True,full_face_certificate_hash=q.identity(full),connectivity_hash=q.identity(c),
            outer_hash=q.identity(f),quality_hash=q.identity(quality),volume_hash=q.identity(volume),continuation_hash=q.identity(continuation),analytic_hash=q.identity(analytic)))
        s.save('REPLAY_RUNTIME.json',dict(seconds=time.perf_counter()-st,peak_rss=q.psutil.Process().memory_info().peak_wset))
    else:
        s.save('FULL_FACE_CERTIFICATE.json',full);s.save('CERTIFICATE_RUNTIME.json',dict(seconds=time.perf_counter()-st,peak_rss=q.psutil.Process().memory_info().peak_wset))
    print('complete',full['passed'],full['required_pairs'],flush=True)

def export():
    case,prefix,history,previous,seed,targets,high,record,timing=r.reproduce();z=s.load(s.OUT/'SELECTED.json')
    p,c,f=s.construct(case.x,case.f,prefix+[high],z['vertex_depth']);ids=sorted(set(v for x in f for v in x));lookup={v:i+1 for i,v in enumerate(ids)}
    s.save('CORE_INGEST_INPUT.json',dict(coordinates=p[ids].tolist(),triangles=[[lookup[v] for v in x] for x in f],outer_hash=q.identity(f),original_vertex_ids=ids))
    changed=z['manifest']['replacements'];removed=[v['cell'] for v in z['manifest']['removed']]
    def boundary(cells):
        from collections import defaultdict
        d=defaultdict(list)
        for cell in cells:
            for face in cell['faces']:d[tuple(sorted(face))].append((cell['id'],face))
        return [dict(owners=rr,vertices=key) for key,rr in sorted(d.items()) if len(rr)==1]
    oldbound=boundary(removed);newbound=boundary(changed);oldkeys={tuple(x['vertices']) for x in oldbound};newkeys={tuple(x['vertices']) for x in newbound}
    # The outer cavity extent is allowed to recede. Common retained interfaces
    # must match exactly; removed exterior overlap is not an unfilled mesh gap.
    newids={x['id'] for x in changed};owners,own=t.ownership(c,f,[tuple(reversed(x)) for x in case.f])
    interfaces=[dict(vertices=k,owners=rr) for k,rr in sorted(owners.items()) if len(rr)==2 and sum(i in newids for i,face in rr)==1]
    assert all(tuple(x['vertices']) in oldkeys for x in interfaces)
    s.save('CAVITY.json',dict(old_boundary=oldbound,new_boundary=newbound,retained_interfaces=interfaces,
        retained_interface_exact_match=True,removed_outer_volume_not_required_to_fill=True,
        reason='Termination replaces overlapping layer-4 volume by an earlier nonoverlapping boundary. New domain is not required to include the old invalid exterior.'))
    s.save('OUTER_INTERFACE.json',dict(hash=q.identity(f),vertex_links=t.vertex_links(f),ownership=own,face_count=len(f),
        wall_hash=q.array_hash(case.x),first_layer_hash=q.array_hash(prefix[0])))
if __name__=='__main__':
    if '--export' in sys.argv:export()
    else:run('--replay' in sys.argv)

"""Supplemental boundary, volume and authoritative-offset checks."""
import generalize_local_remesh as g
from fractions import Fraction as F
from collections import Counter,defaultdict
s=g.s;t=g.t;q=g.q;np=g.np

def inspect(spec,write_output=True):
    folder=g.OUT/spec['name'];z=s.load(folder/'result.json');x,tri=g.fixtures.build(spec);k=z['event']['layer']
    fronts=[g.fixtures.front(spec,target) for target in q.load_schedule().targets[:k]]
    p,c,o=s.construct(x,tri,fronts,z['vertex_depth']);_,old,_=s.construct(x,tri,fronts,[k]*len(x))
    assert q.identity(s.removal_manifest(old,c))==q.identity(z['manifest'])
    byid={c['id']:c for c in old};repl=z['manifest']['replacements'];removed=[v['cell'] for v in z['manifest']['removed']]
    keep=z['manifest']['preserved'];newids={v['id'] for v in repl};owners,own=t.ownership(c,o,[tuple(reversed(f)) for f in tri])
    oldfaces={tuple(sorted(f)) for v in removed for f in v['faces']}
    matches=[dict(vertices=key,owners=rr) for key,rr in sorted(owners.items()) if len(rr)==2 and sum(i in newids for i,f in rr)==1]
    assert all(tuple(row['vertices']) in oldfaces for row in matches)
    signed=lambda cells:sum(F(t.exact_cell(p,c)['signed_volume']) for c in cells)
    replacement=signed(repl);preserved=signed([byid[i] for i in keep]);newvol=F(z['result']['volume']['boundary_volume'])
    assert preserved+replacement==newvol
    coords=all(np.array_equal(p[i*len(x):(i+1)*len(x)],f) for i,f in enumerate([x]+fronts))
    levels=np.array(z['vertex_depth']);offsets=[]
    for layer in sorted(set(levels)):
        ids=np.flatnonzero(levels==layer);sample=g.fixtures.sdf_record(spec,fronts[layer-1][ids],q.load_schedule().targets[layer-1]);offsets.append(sample)
    assert all(v['admitted'] for v in offsets)
    accepted=next(v for v in z['continuation']['attempts'] if v.get('passed'))
    continuation_offset=g.fixtures.sdf_record(spec,g.fixtures.front(spec,q.load_schedule().targets[k])[tri[accepted['column']]],q.load_schedule().targets[k])
    assert continuation_offset['admitted']
    last=[0]*len(tri)
    for v in c:
        if v['kind']=='prism':last[v['column']]=max(last[v['column']],v['layer'])
    top=q.build_topology(tri,len(x));seed=set(z['event']['seed_vertices']);dist={v:0 for v in seed};todo=sorted(seed)
    for v in todo:
        for j in top.vertex_neighbors.row(v):
            j=int(j)
            if j not in dist:dist[j]=dist[v]+1;todo.append(j)
    affected=sorted({v['column'] for v in removed});av={i%len(x) for v in removed for i in v['vertices']}
    out=dict(last_prism_map=last,map_counts=Counter(last),maximum_adjacent_difference=max(abs(last[int(a)]-last[int(b)]) for a,b in [top.edge_triangles.row(i) for i in range(len(top.edges))]),
        affected_columns=affected,affected_vertices=sorted(av),radius=max(dist[i] for i in av),fraction=len(affected)/len(tri),
        removed_by_layer=Counter(v['layer'] for v in removed),removed_count=len(removed),new_counts=Counter(v['kind'] for v in repl),
        preserved_ids=keep,retained_interface_matches=matches,retained_interface_exact=True,
        removed_signed_integral=str(signed(removed)),replacement_volume=str(replacement),preserved_volume=str(preserved),final_boundary_volume=str(newvol),
        balance=True,removed_volume_caveat='Rejected overlapping/inverted candidates do not define an additive physical union volume. Removed signed integral is algebraic, not claimed valid occupied volume.',
        wall_and_first_layer_unchanged=coords,assigned_offsets=offsets,continuation_assigned_offset=continuation_offset,core_interface_hash=q.identity(o))
    if write_output:g.save(folder/'accounting.json',out)
    return out

def tet_smoke():
    folder=g.OUT/'clean_curved_passage';raw=s.load(folder/'TET_SMOKE_RAW.json');inp=s.load(folder/'CORE_INGEST_INPUT.json')
    assert raw['element_types']==[4] and len(raw['elements'][0])==4 and len(raw['nodes'])==4
    assert [raw['nodes'][str(i)] for i in range(1,5)]==raw['prescribed_points']
    P=s.cert.rational(np.array(raw['prescribed_points']));a,b,c,d=[P[i-1] for i in raw['elements'][0]]
    J=t.dot(t.cross(t.sub(b,a),t.sub(c,a)),t.sub(d,a));assert J>0
    coords=np.array(inp['coordinates']+[raw['apex']]);faces=[tuple(i-1 for i in f) for f in inp['triangles']]
    A,B,C=[i-1 for i in raw['base_vertex_ids']];D=len(coords)-1
    new=[(A,B,D),(B,C,D),(C,A,D)];allfaces=faces+new
    cert=s.cert.certify(coords,allfaces,[raw['base_face']]+list(range(len(faces),len(allfaces))))
    import cavity_volume_audit as volume
    sample=tuple(sum(v[j] for v in P)/4 for j in range(3));degree=volume.degree(coords,faces,[sample])
    assert cert['passed'] and not degree['unresolved'] and degree['values'][0]['degree']==0
    result=dict(passed=True,exact_tet_volume=str(J/6),boundary_preserved=True,tetrahedra=1,certificate=cert,outside_original_interface=True,
        ray=degree,scope='One prescribed tetrahedral cavity outside the repaired interface; positive exact geometry and conforming base. Not a full core or CFD-quality qualification.')
    g.save(folder/'TET_SMOKE.json',result);return result

if __name__=='__main__':
    for spec in g.fixtures.definitions():
        a=inspect(spec);b=inspect(spec);assert q.identity(a)==q.identity(b)
        g.save(g.OUT/spec['name']/'accounting_replay.json',dict(equal=True,scientific_hash=q.identity(a)))
    a=tet_smoke();b=tet_smoke();assert q.identity(a)==q.identity(b)

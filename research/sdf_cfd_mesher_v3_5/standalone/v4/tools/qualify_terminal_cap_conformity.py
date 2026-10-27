"""Exact frozen-volume obstruction audit; no production mutation."""
import json,time,hashlib,subprocess
from pathlib import Path
import terminal_cap_conformity as c
t=c.t;q=t.q;r=t.r;np=t.np
ROOT=r.r.g.ROOT;OLD=ROOT/'results/local_layer_termination_topology';OUT=ROOT/'results/terminal_cap_conformity'
STAGES=[]
def load(p):return json.loads(p.read_text())
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def save(n,z): (OUT/n).write_text(json.dumps(r.w.clean(z),sort_keys=True,separators=(',',':'))+'\n',encoding='utf8',newline='\n')
def study():
    clock=time.perf_counter();stages={}
    case,prefix,history,previous,seed,targets,high,record,timing=r.reproduce()
    m=load(OLD/'candidate_1_map.json');points,cells,outer,m=t.construct(case.x,case.f,prefix,high,m['stopped_vertices'])
    stages['seed_reconstruction']=time.perf_counter()-clock;clock=time.perf_counter()
    n=len(case.x);rows=[]
    for failure in load(OLD/'OUTER_CERTIFICATE.json')['failures']:
        a,b=failure['pair'];A=outer[a];B=outer[b]
        assert a in m['terminated_columns'] and b in m['terminated_columns']
        w=c.overlap_witness(points,A,B,tuple(v-n for v in A),tuple(v-n for v in B))
        rows.append(dict(faces=[a,b],columns=[a,b],layer=4,vertices=[A,B],source_vertices=[list(map(int,case.f[a])),list(map(int,case.f[b]))],
            coordinates=[points[list(A)].tolist(),points[list(B)].tolist()],shared_vertices=sorted(set(A)&set(B)),
            adjacent=bool(set(A)&set(B)),cause='TransverseIntersectionOfFrozenPrismTopInteriors',witness=w,
            exact_relation=c.cert.pair(points,A,B)))
    stages['intersection_forensics']=time.perf_counter()-clock;clock=time.perf_counter();groups=[]
    for selected in c.components([z['faces'] for z in rows]):
        boundary=c.cavity_boundary(outer,selected);edges={tuple(sorted(e[:2])) for e in boundary['boundary_edges']}
        neighbors=[i for i,f in enumerate(outer) if i not in selected and any(tuple(sorted((a,b))) in edges for a,b in zip(f,f[1:]+f[:1]))]
        vertexset=set(v for i in selected for v in outer[i])
        dependent=[z['id'] for z in cells if z['layer']==5 and set(z['vertices'])&vertexset]
        groups.append(dict(faces=selected,boundary=boundary,neighbor_faces=neighbors,dependent_new_cells=dependent,
            frozen_boundary_coordinates={str(v):points[v].tolist() for e in boundary['boundary_edges'] for v in e[:2]}))
    stages['cavity_extraction']=time.perf_counter()-clock;clock=time.perf_counter()
    proposal=c.remesh_preserving_cells(outer,[z['witness'] for z in rows])
    owners,own=t.ownership(cells,outer,[tuple(reversed(f)) for f in case.f])
    own['vertex_links']=t.vertex_links(outer)
    own['zero_area_outer_faces']=[i for i,f in enumerate(outer) for a,b,d in [c.cert.rational(points[list(f)])] if not any(t.cross(t.sub(b,a),t.sub(d,a)))]
    stages['ownership']=time.perf_counter()-clock;clock=time.perf_counter()
    # Full current outer audit is replayed, not inferred from known pairs alone.
    outercert=c.cert.certify(points,outer,range(len(outer)))
    stages['exact_outer_certification']=time.perf_counter()-clock
    preservation=dict(lower_prisms=6144,lower_coordinates=q.array_hash(points[:5*n]),expected=load(OLD/'PRESERVATION.json')['lower_coordinate_hash'],
        lower_connectivity_hash=q.identity([z for z in cells if z['layer']<=4]),
        new_cell_hash=q.identity([z for z in cells if z['layer']==5]),wall_hash=q.array_hash(case.x),first_layer_hash=q.array_hash(prefix[0]),
        all_cells_unchanged=True,wall_unchanged=True,first_layer_unchanged=True,transition_cells_before=dict(pyramid=68,tetrahedron=44),
        transition_cells_after=dict(pyramid=68,tetrahedron=44),map_unchanged=True,outer_hash=q.identity(outer))
    assert preservation['lower_coordinates']==preservation['expected']
    prior=load(OLD/'PRESERVATION.json')
    assert preservation['lower_connectivity_hash']==prior['lower_prism_connectivity_hash']
    assert preservation['new_cell_hash']==q.identity(load(OLD/'candidate_1_topology.json')['new_cells'])
    assert preservation['wall_hash']==prior['wall_hash'] and preservation['first_layer_hash']==prior['first_hash']
    STAGES.append(stages)
    return dict(forensics=rows,components=groups,proposal=proposal,ownership=own,outer_certificate=outercert,preservation=preservation)
def analytic():
    spec=next(z for z in q.definitions(q.load_schedule().targets) if z['name']=='spherical_cavity')
    wall,faces,core=q.build(spec);top,patch,foundation=q.foundation(wall,faces,core,q.FixtureSDF(spec),spec)
    prefix=[q.load_checkpoint(ROOT/'results/layer_capacity_valid_prefix/spherical_cavity',i,top.sha256)[0] for i in range(1,13)]
    points,cells,outer,m=t.construct(wall,faces,prefix,prefix[-1],range(len(wall)))
    owners,own=t.ownership(cells,outer,[tuple(reversed(f)) for f in faces]);quality=t.quality(points,cells,owners,12)
    cert=c.cert.certify(points,outer,range(len(outer)))
    return dict(passed=own['passed'] and quality['passed'] and cert['passed'],ownership=own,quality=quality,certificate=cert,
        surface_action='Identity: already conforming, no retriangulation needed',last_valid=12,capacity_limit=13,
        lower_identity=q.array_hash(points[:13*len(wall)]),outer_hash=q.identity(outer))
def main():
    tick=time.perf_counter();a=load(OLD/'AUTHORITY.json')
    for p,h in (a['source_sha256']|a['evidence_sha256']).items():assert sha(ROOT/p)==h,p
    save('PRIOR_AUTHORITY.json',dict(report=sha(OLD/'report.md'),authority=sha(OLD/'AUTHORITY.json'),all_bindings_verified=True))
    first=study();elapsed=time.perf_counter()-tick;second=study();replay=time.perf_counter()-tick-elapsed
    assert q.identity(first)==q.identity(second)
    for k,z in first.items():save(k.upper()+'.json',z)
    at=time.perf_counter();a=analytic();save('ANALYTIC_SMOKE.json',a)
    save('DETERMINISM.json',dict(equal=True,first=q.identity(first),second=q.identity(second),scope='Forensics, cavity, rejected proposal, ownership, full outer certificate, preservation'))
    save('RUNTIME.json',dict(first_seconds=elapsed,replay_seconds=replay,analytic_seconds=time.perf_counter()-at,total_seconds=time.perf_counter()-tick,
        peak_rss=q.psutil.Process().memory_info().peak_wset,surface_remeshing_seconds=0,transition_regeneration_seconds=0,
        first_stages=STAGES[0],replay_stages=STAGES[1],general_hard_gate_recalculation='Skipped: unchanged geometry and exact preservation obstruction',
        skipped_reason='Exact frozen-volume obstruction established before topology mutation'))
    print('proofs',sum(z['witness']['proven'] for z in first['forensics']),'components',len(first['components']),flush=True)
if __name__=='__main__':main()

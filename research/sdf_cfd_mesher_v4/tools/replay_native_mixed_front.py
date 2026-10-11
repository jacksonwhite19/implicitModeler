"""Replay fresh construction with exact-input certificate-cache hits only.

Stored geometry is used to validate cache inputs, never as a proposal. The
provider still generates the wall, discovers the cavity and constructs cells.
Unknown certificate inputs take the ordinary exact path.
"""
import json,sys,time
from collections import Counter
import generalize_local_remesh as g
from mixed_front_fixture_provider import Provider
from task4l_run import run_mixed_front
from sdf_cfd_mesher_v4.advancing_front.mixed_front import scientific_hash,face_key,geometry_inputs
from sdf_cfd_mesher_v4.advancing_front.mixed_front_journal import MixedFrontJournal
from sdf_cfd_mesher_v4.common.authority import identity


def run(name,source='final',target='replay',repair=False):
    root=g.s.ROOT/'results/purpose_built_mixed_front_runner'/name
    folder=root/target;provider=Provider(next(s for s in g.fixtures.definitions() if s['name']==name),folder)
    cache={};hits=[];original=g.s.cert.certify
    def key(points,faces,active):return identity(dict(points=g.q.array_hash(points),faces=faces,active=sorted(active)))
    for path in sorted((root/source/'checkpoints').glob('STATE_*.json')):
        envelope=json.loads(path.read_text());state=envelope['state']
        assert scientific_hash(state)==envelope['scientific']
        fronts,levels=provider.geometry(state)
        points,cells,outer=g.s.construct(provider.x,provider.tri,fronts,levels)
        owners,_=g.t.ownership(cells,outer,[tuple(reversed(f)) for f in provider.tri])
        faces=[owners[k][0][1] for k in sorted(owners)];n=len(provider.x)
        lookup={face_key(tuple(f'layer:{v//n}:source:{v%n}' for v in face)):i for i,face in enumerate(faces)}
        inputs=geometry_inputs(state);rows=[]
        for receipt in state['certificate'].values():
            assert all(inputs[k]==v for k,v in receipt['inputs'].items())
            pair=sorted(lookup[f] for f in receipt['faces'])
            rows.append(dict(pair=pair,**receipt['value']))
        rows.sort(key=lambda r:r['pair'])
        assert all(r['classification'] in ('CertifiedSeparated','CertifiedAllowedIncidentContact') for r in rows)
        cache[key(points,faces,range(len(faces)))]=dict(passed=True,required_pairs=len(rows),methods=Counter(r['method'] for r in rows),
            failures=[],proof_hash=identity(rows),proofs=rows,
            coverage='All AABB-overlapping unique-face pairs touching changed geometry; control hulls bound exact bilinear and planar images')
    def cached(points,faces,active):
        k=key(points,faces,active)
        if k in cache:hits.append(k);return cache[k]
        return original(points,faces,active)
    g.s.cert.certify=cached
    tick=time.perf_counter()
    initial=provider.initialize();operations=run_mixed_front(provider,initial,folder/'checkpoints',2)
    final=MixedFrontJournal(folder/'checkpoints').restart()
    expected=json.loads((root/source/'RESULT.json').read_text())
    reference=MixedFrontJournal(root/source/'checkpoints').restart()
    result=dict(equal=scientific_hash(final)==expected['final'],operations_equal=operations==expected['operations'],
                scientific_hash=scientific_hash(final),certificate_cache_hits=len(hits),
                construction='Fresh wall, current evidence, generic discovery, qualified cell templates',
                cache_scope='Exact points, face perimeters, active population and bound proof dependencies')
    result['geometry_equal']=all(final[k]==reference[k] for k in ('wall','first_layer','vertices','cells','faces','outer'))
    result['intentional_column_state_repair']=repair
    g.save(folder/'REPLAY_COMPARISON.json',result)
    g.save(folder/'RUNTIME.json',dict(total=time.perf_counter()-tick,stages=provider.times,peak_rss=g.q.psutil.Process().memory_info().peak_wset,exact_input_certificate_cache_hits=len(hits)))
    g.save(folder/'RESULT.json',dict(operations=operations,initial=scientific_hash(initial),final=scientific_hash(final),
                                   wall_identity=initial['wall']==final['wall'],records=provider.records,
                                   cell_count=len(final['cells']),active_columns=len(final['prism_front']),outer_faces=len(final['outer'])))
    if repair:assert result['geometry_equal'],result
    else:assert result['equal'] and result['operations_equal'],result

if __name__=='__main__':run(sys.argv[1],*(sys.argv[2:4]),repair='--repair' in sys.argv)

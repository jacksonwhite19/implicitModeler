"""Fresh native construction replay with exact-input final proof reuse."""
import json,time
from collections import Counter
from migrate_general_native import HistoricalProvider,OUT,g
from sdf_cfd_mesher_v4.advancing_front.mixed_front import scientific_hash,geometry_inputs,face_key
from sdf_cfd_mesher_v4.advancing_front.mixed_front_runner import MixedFrontRunner
from sdf_cfd_mesher_v4.advancing_front.mixed_front_journal import MixedFrontJournal
from sdf_cfd_mesher_v4.common.authority import identity

def run():
    tick=time.perf_counter();source=MixedFrontJournal(OUT/'general_native/checkpoints').restart()
    provider=HistoricalProvider(OUT/'general_replay')
    fronts,levels=provider.geometry(source);points,cells,outer=g.s.construct(provider.x,provider.tri,fronts,levels)
    owners,_=g.t.ownership(cells,outer,[tuple(reversed(f)) for f in provider.tri]);faces=[owners[k][0][1] for k in sorted(owners)]
    n=len(provider.x);lookup={face_key(tuple(f'layer:{v//n}:source:{v%n}' for v in f)):i for i,f in enumerate(faces)}
    inputs=geometry_inputs(source);rows=[]
    for receipt in source['certificate'].values():
        assert all(inputs[k]==v for k,v in receipt['inputs'].items())
        rows.append(dict(pair=sorted(lookup[f] for f in receipt['faces']),**receipt['value']))
    rows.sort(key=lambda row:row['pair'])
    assert all(r['classification'] in ('CertifiedSeparated','CertifiedAllowedIncidentContact') for r in rows)
    def key(p,f,a):return identity(dict(points=g.q.array_hash(p),faces=f,active=sorted(a)))
    expected=key(points,faces,range(len(faces)))
    cert=dict(passed=True,required_pairs=len(rows),methods=Counter(r['method'] for r in rows),failures=[],proof_hash=identity(rows),proofs=rows,
        coverage='All AABB-overlapping unique-face pairs touching changed geometry; control hulls bound exact bilinear and planar images')
    original=g.s.cert.certify;hits=[]
    def cached(p,f,a):
        if key(p,f,a)==expected:hits.append(expected);return cert
        return original(p,f,a)
    g.s.cert.certify=cached
    try:
        initial=provider.initialize();runner=MixedFrontRunner(OUT/'general_replay/checkpoints',provider);runner.initialize(initial)
        operations=[runner.step(),runner.step()];final=runner.store.restart()
        same=scientific_hash(final)==scientific_hash(source)
        g.save(OUT/'GENERAL_REPLAY.json',dict(equal=same,scientific=scientific_hash(final),certificate_exact_input_hits=len(hits),operations=operations))
        assert same
        print('REPLAY_EXACT',scientific_hash(final),flush=True)
    finally:
        g.s.cert.certify=original
        g.save(OUT/'GENERAL_REPLAY_RUNTIME.json',dict(seconds=time.perf_counter()-tick,peak_rss=g.q.psutil.Process().memory_info().peak_wset))

if __name__=='__main__':run()

"""Reproduce sealed geometry and qualify generic discovery without recertifying immutable faces."""
import time,hashlib
import generalize_local_remesh as g
s=g.s;q=g.q;np=g.np
old=s.ROOT/'results/local_termination_cavity_remesh'
def run():
    st=time.perf_counter();authority=s.load(old/'AUTHORITY.json')
    for p,h in (authority['source_sha256']|authority['evidence_sha256']).items():assert hashlib.sha256((s.ROOT/p).read_bytes()).hexdigest()==h,p
    case,prefix,history,previous,seed,targets,high,record,timing=s.r.reproduce()
    oldmap=s.load(s.ROOT/'results/local_layer_termination_topology/candidate_1_map.json');base=np.full(len(case.x),5);base[oldmap['stopped_vertices']]=4
    conflicts=s.load(s.ROOT/'results/terminal_cap_conformity/FORENSICS.json');active={v for x in conflicts for f in x['source_vertices'] for v in f}
    levels,expansion=g.discover(case.x,case.f,prefix+[high],active,min(x['layer'] for x in conflicts),base)
    expected=s.load(old/'SELECTED.json');assert levels.tolist()==expected['vertex_depth']
    p,c,o,check,times=g.validation(case.x,case.f,prefix+[high],levels,False)
    assert q.identity(c)==q.identity(expected['cells']) and q.identity(o)==q.identity(expected['outer'])
    assert q.identity(check['quality'])==q.identity(s.load(old/'ALL_CELL_QUALITY.json'))
    assert np.array_equal(p,np.load(old/'mesh_points.npy'))
    result=dict(classification='TerminationRemeshQualified',coordinates_equal=True,connectivity_equal=True,quality_equal=True,
        generic_discovery_equal=True,expansion=expansion,layer_map_hash=q.identity(levels.tolist()),
        sealed_certificate_hash=authority['evidence_sha256']['results/local_termination_cavity_remesh/FULL_FACE_CERTIFICATE.json'],
        full_certificate_reused=True,volume_receipt_reused=True,continuation=s.load(old/'CONTINUATION.json')['passed'],
        prior_report=authority['report_sha256'],scope='Identical coordinates/connectivity plus immutable exact full certificate and continuation replay bound by prior authority')
    folder=g.OUT/'general_sdf_baseline';g.save(folder/'result.json',result);g.save(folder/'runtime.json',dict(regression_seconds=time.perf_counter()-st,peak_rss=q.psutil.Process().memory_info().peak_wset,
        historical_primary_stages={k:v for k,v in s.load(old/'RUNTIME_SUMMARY.json')['stages'].items() if k!='REPLAY_RUNTIME.json'},fresh_full_certificate=False))
    return result
if __name__=='__main__':
    first=run();second=run();assert q.identity(first)==q.identity(second)
    g.save(g.OUT/'general_sdf_baseline/replay.json',dict(equal=True,scientific_hash=q.identity(first)))

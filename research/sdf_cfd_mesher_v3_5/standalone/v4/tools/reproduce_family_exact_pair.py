"""Preserved rejected-pair regression; never a production construction input."""
import json,os,shutil
from pathlib import Path
from run_geometry_family import OUT,AnalyticProvider,g,geometry
from sdf_cfd_mesher_v4.advancing_front.native_checkpoint import NativeCheckpointStore
from sdf_cfd_mesher_v4.common.authority import file_hash,identity

def run():
    r=OUT/'competing_ellipsoids__scale_0p5';folder=OUT/'exact_sample_probe';folder.mkdir(exist_ok=True)
    ref=json.loads((r/'Z/REPLAY_INPUT.json').read_text());src=Path(ref['source_checkpoint']);env=json.loads(src.read_text());cp=folder/'checkpoints';(cp/'objects').mkdir(parents=True,exist_ok=True)
    for row in env['components'].values():
        target=cp/'objects'/row['file']
        if not target.exists():os.link(src.parent/'objects'/row['file'],target)
    shutil.copy2(src,cp/src.name);g.save(cp/'CURRENT.json',dict(format=env['format'],checkpoint=src.name,scientific=env['scientific'],sha256=file_hash(src)))
    spec=next(s for s in geometry.cohort() if s['name']==r.name);provider=AnalyticProvider(spec,folder)
    state=NativeCheckpointStore(cp).restart()
    # Reconstruct only the already-rejected proposal for one isolated pair.
    # No state publication or acceptance occurs in this diagnostic.
    provider.certify_candidate=lambda f,l,c:dict(c,passed=True)
    proposal=provider.propose(state);p,c,o=g.s.construct(provider.x,provider.tri,proposal['fronts'],proposal['levels'])
    owners,_=g.t.ownership(c,o,[tuple(reversed(f)) for f in provider.tri]);faces=[owners[k][0][1] for k in sorted(owners)]
    ref=json.loads((r/'V/PROPOSAL_20.json').read_text())['check']['certificate']['failures'][246];a,b=[faces[i] for i in ref['pair']]
    g.save(folder/'PAIR.json',dict(points=p.tolist(),a=a,b=b,reference_classification=ref['classification']))
    import termination_geometry_certificate as exact
    first=exact.pair(p,a,b);g.q.kernel.clear_kernel_caches();second=exact.pair(p,a,b)
    result=dict(exact=first==second,classification=first['classification'],expected=ref['classification'],first=identity(first),second=identity(second))
    g.save(OUT/'EXACT_PAIR_REPLAY.json',result);print(result)
    assert result['exact'] and result['classification']==result['expected']

if __name__=='__main__':run()

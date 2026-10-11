"""Check recovered caches against saved geometry; no layer advancement."""
import argparse,json,time
from pathlib import Path
from fractions import Fraction
import numpy as np
from fast_milestone import load
from incremental_layer_validation import ExactVolume
from benchmark_production_fast import save
from sdf_cfd_mesher_v4.advancing_front.native_checkpoint import scientific_digest

def main():
    p=argparse.ArgumentParser();p.add_argument('checkpoint',type=Path);p.add_argument('folder',type=Path);a=p.parse_args();started=time.perf_counter()
    run=load(a.checkpoint,a.folder);old=json.loads((a.folder/'INITIAL_MILESTONE.json').read_text())['signature']['geometry']
    for k in ('wall','first_layer'):assert scientific_digest(run.tx.current[k])==old[k]
    assert np.array_equal(run.points,run.degree_state.points)
    for i,c in run.topology.cells.items():
        w=ExactVolume.sample(run.points,c)
        assert w==run.volume.witness[i]==run.degree_state.witness[i]
    total=sum((Fraction(r['signed_volume']) for r in run.quality.exact.values()),Fraction())
    boundary=sum((ExactVolume.contribution(run.points,f) for f in run.wall+list(run.outer)),Fraction())
    assert total==boundary==run.volume.total==run.volume.boundary_total
    # Every newly committed layer was nominal, so the saved coordinates must
    # exactly reproduce its existing analytic source, not just pass tolerance.
    start=json.loads((a.folder/'START.json').read_text())['layer'];n=len(run.provider.x);layers=max(c['layer'] for c in run.topology.cells.values())
    for layer in range(start+1,layers+1):
        expected=run.provider.geometry_source.front(run.spec,run.provider.schedule.targets[layer-1])
        assert np.array_equal(run.points[layer*n:(layer+1)*n].view('u8'),expected.view('u8'))
    save(a.folder/'RECOVERY_GEOMETRY_CHECK.json',dict(passed=True,witnesses=len(run.volume.witness),witnesses_rederived_exact=True,volume_balance=True,exact_volume=str(total),wall_first_layer_exact=True,nominal_source_coordinates_exact=True,seconds=time.perf_counter()-started))
    print('RECOVERED_GEOMETRY_EXACT',flush=True)

if __name__=='__main__':main()

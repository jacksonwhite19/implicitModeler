"""Measured generic cavity search from the preserved pre-remesh state."""
import sys
sys.dont_write_bytecode=True
from pathlib import Path
import time,json
from migrate_general_native import HistoricalProvider
import generalize_local_remesh as g
from coupled_shell_validation import check
OUT=g.s.ROOT/'results/coupled_shell_remesh_determinant'


def run():
    provider=HistoricalProvider(OUT/'probe')
    prior=g.s.ROOT/'results/broader_mixed_geometry_end_to_end/general_asymmetric_passage/A/ROLLBACK_MANIFEST.json'
    # Comparison seed only for this reproducer. Production dispatch derives it
    # again from current overlap evidence in HistoricalProvider.evidence.
    manifest=json.loads(prior.read_text());seed=set(manifest['cavity']['seed']['vertices'])
    tick=time.perf_counter()
    levels,history=g.discover(provider.x,provider.tri,provider.fronts,seed,4,provider.base,candidate_check=check)
    g.save(OUT/'PROBE_REVISION.json',dict(levels=None if levels is None else levels.tolist(),history=history,seconds=time.perf_counter()-tick))
    print('DISCOVERED',levels is not None,len(history),flush=True)
    if levels is not None:
        p,c,o,checked,_=g.validation(provider.x,provider.tri,provider.fronts,levels,False)
        g.save(OUT/'PROBE_COUPLED.json',check(p,c,o,True))


if __name__=='__main__':run()

"""Fresh-process normal candidate open/init measurement; no forecast."""
from pathlib import Path
import json,sys,time,os
from clean_sheet_resume import ResumeBoundary,PREP,OUT,REPO
from clean_sheet_isolation import deny_native_reads


def run():
    read=lambda p:json.loads(p.read_text())
    refs=read(PREP/'BASELINE.json')['allowed_inputs']
    refs.update(read(PREP/'REQUEST_INPUT_CLOSURE.json')['allowed_inputs'])
    runtime=[sys.prefix,sys.base_prefix]+[p for p in sys.path if p and not Path(p).resolve().is_relative_to(REPO)]
    boundary=ResumeBoundary(REPO,refs,PREP,runtime);boundary.install()
    import psutil,sqlite3
    from unittest.mock import patch
    from clean_sheet_candidate import Candidate
    from path_preserving_deduplication import durable_json
    from production_forecast_admission import BoundProductionForecastCache
    from sdf_cfd_mesher_v4.advancing_front.native_checkpoint import NativeCheckpointStore
    from production_surface_authority import load_production_wall
    from geometry_family_authority import Source
    from clean_sheet_l4_bootstrap import SURFACE,REQUEST,metrics
    root=read(OUT/'PUBLICATION.json')['root']
    paths=[r['path'] for r in read(PREP/'ORACLE_LOCKS.json')['entries']]
    ids={(Path(p).stat().st_dev,Path(p).stat().st_ino) for p in refs}
    def denied(*a,**k):raise AssertionError('NoHistoryTraversalOrForecastInTrustedReopen')
    with deny_native_reads(paths,ids),patch.object(sqlite3,'connect',denied),patch.object(NativeCheckpointStore,'restart',denied),patch.object(BoundProductionForecastCache,'forecast',denied):
        tick=time.perf_counter();cpu=time.process_time();io=psutil.Process().io_counters()
        with Candidate(OUT/'candidate',root) as state:
            wall,tri,surface=load_production_wall(SURFACE)
            source=Source(dict(components=[read(SURFACE.parent/'GEOMETRY.json')['definition']]))
            adapter=state.initialize(wall,tri,surface,REQUEST,source)
            assert adapter.binding['layer']==5 and adapter.binding['next_layer']==6
            assert float(adapter.remaining_increments[0])==0.00035506009452908446
            result=dict(root=root,passed=True,increment=float(adapter.remaining_increments[0]),
                stats=dict(state.stats),base_stats=dict(state.base.stats),sqlite_rows=0,proof_rows=0,quality_rows=0,
                forecast_invoked=False,oracle_unlocked=False,**metrics(tick,cpu,io))
        durable_json(OUT/'FRESH_TRUSTED_REOPEN.json',result)
    durable_json(OUT/'FRESH_REOPEN_READ_LOG.json',boundary.record())


if __name__=='__main__':run()

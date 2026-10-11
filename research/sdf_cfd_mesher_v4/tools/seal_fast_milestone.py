"""Independent seal of a verified milestone; no layer advancement."""
import argparse,time,json,traceback
from pathlib import Path
from fast_milestone import load
from benchmark_production_fast import save,seal_new
from qualify_broader_geometry import export
from resume_fast_production import remaining

def main():
    p=argparse.ArgumentParser();p.add_argument('checkpoint',type=Path);p.add_argument('output',type=Path);a=p.parse_args();started=time.perf_counter()
    try:
        run=load(a.checkpoint,a.output);print('RESTART_EXACT',flush=True)
        sealed=seal_new(run,a.output,proof_checkpoint=a.checkpoint);save(a.output/'SEAL.json',sealed)
        interface=export(run.tx.current,a.output/'interface');layer=max(c['layer'] for c in run.topology.cells.values())
        save(a.output/'SHELL_RESULT.json',dict(passed=True,all_active_schedules_complete=not remaining(run.tx.current,len(run.provider.schedule.targets)),current_layer=layer,cells=len(run.topology.cells),relations=len(run.proofs),interface=interface,checkpoint=str(a.checkpoint)))
        save(a.output/'RECOVERY.json',dict(passed=True,layer=layer,geometry_regenerated=False,full_seal=True,restart_exact=True,checkpoint=str(a.checkpoint),seconds=time.perf_counter()-started))
        print('SEALED',layer,flush=True)
    except BaseException as exc:
        save(a.output/'SEAL_FAILURE.json',dict(error=str(exc),traceback=traceback.format_exc()));raise
    finally:save(a.output/'SEAL_SESSION.json',dict(seconds=time.perf_counter()-started))

if __name__=='__main__':main()

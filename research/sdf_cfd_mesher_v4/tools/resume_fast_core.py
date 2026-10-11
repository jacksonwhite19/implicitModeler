"""Use the existing core adapter after a ProductionFast final shell seal."""
import argparse,json,time,traceback
from pathlib import Path
from fast_milestone import load
from benchmark_production_fast import save
import run_family_core

def main():
    p=argparse.ArgumentParser();p.add_argument('folder',type=Path);a=p.parse_args();started=time.perf_counter()
    shell=json.loads((a.folder/'SHELL_RESULT.json').read_text());seal=json.loads((a.folder/'SEAL.json').read_text())
    if not shell['all_active_schedules_complete'] or not seal['passed']:raise ValueError('RequiresCompletedSealedShell')
    try:
        run=load(Path(shell['checkpoint']) if 'checkpoint' in shell else a.folder/f'milestone_{shell["current_layer"]:02d}',a.folder)
        state=run.binding_state();run_family_core.OUT=a.folder.parent.parent
        run_family_core.run(a.folder.parent.name,a.folder.name,sealed_state=state)
        save(a.folder/'CORE_END.json',dict(reason='BoundedCompleteQualified',seconds=time.perf_counter()-started))
    except Exception as exc:
        save(a.folder/'CORE_END.json',dict(reason='CoreUnsupported',error=str(exc),traceback=traceback.format_exc(),seconds=time.perf_counter()-started))
        raise
    finally:save(a.folder/'CORE_SESSION.json',dict(seconds=time.perf_counter()-started))

if __name__=='__main__':main()

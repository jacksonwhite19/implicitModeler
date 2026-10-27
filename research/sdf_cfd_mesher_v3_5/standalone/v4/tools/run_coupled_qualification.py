"""Run the existing native transaction path with coupled acceptance enabled."""
import sys
sys.dont_write_bytecode=True
from pathlib import Path
import qualify_broader_geometry as driver
import time
import coupled_shell_validation as coupled
from migrate_general_native import HistoricalProvider

driver.OUT=Path(__file__).resolve().parents[1]/'results/coupled_shell_remesh_determinant'

def run(name,replay):
    # Diagnostics are separate from every scientific object/transaction hash.
    profile=[];active=[None];in_quality=[False]
    original_check=coupled.check;original_quality=coupled.g.t.quality;original_exact=coupled.g.t.exact_cell
    def quality(*args,**kwargs):
        tick=time.perf_counter();in_quality[0]=True
        try:return original_quality(*args,**kwargs)
        finally:
            in_quality[0]=False
            if active[0] is not None:active[0]['metrics_seconds']+=time.perf_counter()-tick
    def exact(*args,**kwargs):
        tick=time.perf_counter()
        try:return original_exact(*args,**kwargs)
        finally:
            if active[0] is not None and not in_quality[0]:active[0]['closure_exact_seconds']+=time.perf_counter()-tick
    def check(*args,**kwargs):
        tick=time.perf_counter();row=dict(metrics_seconds=0.,closure_exact_seconds=0.);active[0]=row
        try:
            result=original_check(*args,**kwargs)
            row.update(passed=result['passed'],closures=result['closures']);return result
        finally:
            row['total_seconds']=time.perf_counter()-tick
            row['construction_and_identity_seconds']=row['total_seconds']-row['metrics_seconds']-row['closure_exact_seconds']
            profile.append(row);active[0]=None
    coupled.check=check;coupled.g.t.quality=quality;coupled.g.t.exact_cell=exact
    def instrument(method):
        original=getattr(HistoricalProvider,method)
        def measured(self,*args,**kwargs):
            tick=time.perf_counter()
            try:return original(self,*args,**kwargs)
            finally:self.times.setdefault('phase_history',[]).append(dict(phase=method,seconds=time.perf_counter()-tick))
        setattr(HistoricalProvider,method,measured)
    for method in ('initialize','propose','evidence','discover','numerical_validation','certify_validation','rebuild'):instrument(method)
    try:driver.run(name,replay)
    finally:coupled.g.save(driver.OUT/name/replay/'COUPLED_PROFILE.json',profile)


if __name__=='__main__':run(sys.argv[1],sys.argv[2])

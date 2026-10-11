"""Production recovery control; no advancement entry point exists here."""
from pathlib import Path
import json,sys,time,sqlite3,shutil,traceback
from unittest.mock import patch
import psutil
from path_preserving_deduplication import durable_json
from clean_sheet_l4_bootstrap import metrics
from qualify_recursive_layer6 import inputs
from qualify_composable_state import CountConnection,SQL
from composable_state import CommittedState,plan_transition
import recursive_epoch_recovery as recovery
import composable_state as common
from sdf_cfd_mesher_v4.advancing_front.native_checkpoint import NativeCheckpointStore
import immutable_state_access as access

OUT=Path(__file__).resolve().parents[1]/'results/recursive_epoch_recovery'
SCIENTIFIC='4d49baf5d372827ea853edbfdedf80ca0909827be865dbdde58faa3da7630170'
CONTENT='15259f4e506a7ae574666283b40d33428f788de8f9d5f3e29314d15ef27178f0'
def denied(*a,**kw):raise AssertionError('ForbiddenNormalTraversalOrAdvancement')
def main(mode):
    if (OUT/'AUTHORITY.json').exists():raise ValueError('PreserveSealedRecoveryQualification')
    tick=(time.perf_counter(),time.process_time(),psutil.Process().io_counters())
    from production_forecast_admission import BoundProductionForecastCache
    try:
        with patch.object(BoundProductionForecastCache,'forecast',denied):
            if mode=='deep':
                ref=access.read(OUT.parent/'recursive_layer6/AUTHORITY.json')['reference']
                connect=sqlite3.connect
                def counted(*a,**kw):kw['factory']=CountConnection;return connect(*a,**kw)
                with patch.object(sqlite3,'connect',counted):
                    r=recovery.recover(ref,CONTENT,SCIENTIFIC,OUT/'access',progress=lambda s:print(s,flush=True))
                result=dict(passed=True,reference=r['reference'],content=r['content'],scientific=r['scientific'],nodes=r['nodes'],files=len(r['files']),bytes_authenticated=sum(x['token']['size'] for x in r['files'].values()),stats=r['stats'],sql=dict(SQL),metrics=metrics(*tick))
            else:
                with patch.object(sqlite3,'connect',denied),patch.object(CommittedState,'deep_verify',denied),patch.object(common,'scientific_digest',denied),patch.object(NativeCheckpointStore,'restart',denied):
                    r=recovery.discover(OUT/'access')
                    with CommittedState(r['reference']) as state:
                        adapter,_=inputs(state);plan=plan_transition(state,adapter,7)
                        assert state.layer==6 and state.content_root==CONTENT and r['scientific']==SCIENTIFIC
                        assert plan['increment_hex']=='0x1.bd6749eb1b62ap-12'
                        result=dict(passed=True,plan=plan,reference=r['reference'],stats=dict(state.stats),parent_stats=dict(state.parent.stats),native_stats=dict(state.parent.parent.wrapped.stats),metrics=metrics(*tick),sql_rows=0,proof_rows=0,quality_rows=0,full_prefix_hashes=0,deep_copies=0,deep_reauthentication=False)
        result.update(L7_admission=False,L7_forecast=False,L7_generation=False,core=False)
        durable_json(OUT/(mode.upper()+'.json'),result);print(json.dumps(result,indent=2),flush=True)
    except BaseException as exc:
        durable_json(OUT/(mode.upper()+'_FAILURE.json'),dict(error=repr(exc),traceback=traceback.format_exc(),metrics=metrics(*tick)));raise
if __name__=='__main__':main(sys.argv[1])




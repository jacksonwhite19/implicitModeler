"""Live late-stage interruption of historical committed-cell rollback."""
import time
from migrate_general_native import HistoricalProvider,OUT,g
from sdf_cfd_mesher_v4.advancing_front.mixed_front_runner import MixedFrontRunner
from sdf_cfd_mesher_v4.advancing_front.mixed_front import scientific_hash


def run():
    tick=time.perf_counter();rows=[];finals=[]
    for phase in ('NumericallyValidated','CertificateGenerated'):
        folder=OUT/'failure_injection'/phase
        provider=HistoricalProvider(folder);initial=provider.initialize()
        runner=MixedFrontRunner(folder/'checkpoints',provider);runner.initialize(initial)
        def crash(stage):
            if stage==phase:raise RuntimeError('InjectedLateFailure')
        try:runner.step(interrupt=crash)
        except RuntimeError as error:
            if str(error)!='InjectedLateFailure':raise
        else:raise AssertionError('InjectionNotReached')
        restored=runner.store.restart()
        assert restored==initial
        # Restart with a new provider: no in-memory numerical receipt survives.
        restarted=HistoricalProvider(folder)
        restarted.initialize()
        runner=MixedFrontRunner(folder/'checkpoints',restarted)
        operation=runner.step();final=runner.store.restart()
        assert len(final['events'])==1
        event=operation['event'];log=next(iter(final['events'].values()))
        import json
        journal=json.loads((folder/'checkpoints'/('JOURNAL_'+event['event']+'.json')).read_text())
        assert runner.store.transact(journal['evidence'],journal['cavity'],None,None,None)==log
        finals.append(scientific_hash(final))
        rows.append(dict(phase=phase,restored_exact=True,no_partial_binding=True,replayed=True,idempotent=True,
            removed_committed_cells=len(log['cavity_manifest']['removed_cells']),final=finals[-1],stages=journal['stages']))
        g.save(OUT/'LATE_FAILURE_RECOVERY.json',dict(rows=rows,scientific_equal=len(set(finals))==1))
        print('RECOVERED',phase,flush=True)
    assert len(set(finals))==1
    g.save(OUT/'LATE_FAILURE_RUNTIME.json',dict(seconds=time.perf_counter()-tick,peak_rss=g.q.psutil.Process().memory_info().peak_wset))

if __name__=='__main__':run()

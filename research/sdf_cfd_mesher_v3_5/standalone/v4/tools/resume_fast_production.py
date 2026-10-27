"""Bounded ProductionFast advancement from existing authority, never layer zero."""
import argparse,json,time,traceback,gc
from pathlib import Path
import psutil
import fast_milestone as milestone
from production_fast_layer import FastLayer
from production_scale_authority import cohort
from benchmark_production_fast import save,seal_new
from sdf_cfd_mesher_v4.advancing_front.native_checkpoint import NativeCheckpointStore
from sdf_cfd_mesher_v4.advancing_front.mixed_front_runner import MixedFrontRunner
from production_compute_control import ProgressLedger,BranchStopped

def remaining(state,requested):
    return any(int(state['columns'][k]['last_prism_layer'])<requested for k in state['prism_front'])

def import_general(state,spec,folder):
    """Reconstruct only an already validated commit, with a full degree bootstrap.

    Mixed/partial topology continues through the general runner. Importing its
    checkpoint does not qualify incremental degree for a different topology.
    """
    from run_geometry_family import AnalyticProvider
    provider=AnalyticProvider(spec,folder)
    fronts,levels=provider.geometry(state)
    packed=provider.pack(fronts,levels,state)
    if any(packed[k]!=state[k] for k in ('vertices','cells','faces','outer')):
        raise ValueError('GeneralCheckpointGeometryReconstructionMismatch')
    run=FastLayer(state,spec,folder)
    from import_fast_legacy_proofs import prepare
    prepare(run);milestone.ensure_proofs(run)
    return run

def main():
    p=argparse.ArgumentParser();p.add_argument('case');p.add_argument('--source',type=Path,required=True);p.add_argument('--sealed',type=Path)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--seconds',type=float,default=2700);p.add_argument('--milestone',action='store_true')
    p.add_argument('--max-new-layers',type=int)
    a=p.parse_args()
    if a.max_new_layers is not None and a.max_new_layers<1:p.error('--max-new-layers must be positive')
    a.output.mkdir(parents=True,exist_ok=False);started=time.perf_counter();run=None;rows=[];reason='ImplementationBlocked'
    try:
        spec=next(s for s in cohort() if s['name']==a.case)
        if a.milestone:run=milestone.load(a.source,a.output)
        elif a.sealed:run=milestone.from_sealed_append(a.source,a.sealed,spec,a.output)
        else:run=FastLayer(NativeCheckpointStore(a.source).restart(),spec,a.output)
        if run.proofs is None:
            from import_fast_legacy_proofs import prepare
            prepare(run)
        first=max(c['layer'] for c in run.topology.cells.values());requested=len(run.provider.schedule.targets)
        save(a.output/'START.json',dict(source=str(a.source),sealed=str(a.sealed) if a.sealed else None,layer=first,cells=len(run.topology.cells),spec=spec,bootstrap_seconds=time.perf_counter()-started))
        save(a.output/'GEOMETRY_AUTHORITY.json',dict(spec=spec,authority=run.provider.authority))
        # The preserved input already supplies the initial durable authority.
        # Do not duplicate it before any new layer has committed.
        last_checkpoint=a.source if a.milestone else None
        record=dict(source=str(a.source),sealed=str(a.sealed) if a.sealed else None,
                    reused_initial_authority=True,layer=first)
        save(a.output/'INITIAL_MILESTONE.json',record)
        print('READY',first,flush=True)
        runner=MixedFrontRunner(a.output/'unused_qualification_journal',run.provider,profile='ProductionFast');runner.fast_session=run
        progress=ProgressLedger(a.output/'TRIALS.jsonl');last_commit=time.monotonic()
        def observe(check,layer):
            if time.monotonic()-last_commit>=900:raise BranchStopped('NoCommittedLayerFor15Minutes')
            progress.trial(check,layer)
        run.provider.compute_observer=observe
        last_saved=first
        while remaining(run.tx.current,requested):
            if a.max_new_layers is not None and len(rows)>=a.max_new_layers:reason='BudgetStoppedAfterProgress';break
            if time.perf_counter()-started>=a.seconds:reason='BudgetStoppedAfterProgress';break
            tick=time.perf_counter()
            try:result=runner.step()
            except BranchStopped as exc:
                save(a.output/'BRANCH_STOP.json',dict(reason=str(exc),seconds=time.perf_counter()-tick));reason='BudgetStoppedStagnant';break
            except Exception as exc:
                save(a.output/'FAILED_ATTEMPT.json',dict(error=str(exc),traceback=traceback.format_exc(),seconds=time.perf_counter()-tick));reason='ImplementationBlocked';break
            if runner.fast_conservative:
                state=runner.store.restart()
                elapsed=time.perf_counter()-tick
                save(a.output/f'GENERAL_COMMIT_{len(rows):02d}.json',dict(result=result,seconds=elapsed,layer=max(c['layer'] for c in state['cells'].values()),events=state['events'],provider_times=runner.provider.times))
                NativeCheckpointStore(a.output/f'general_checkpoint_{len(rows):02d}').publish(state)
                # Never seal the stale pre-transaction FastLayer after a general
                # commit. General topology retains full global degree fallback.
                run=import_general(state,spec,a.output)
                runner.fast_session=run
                runner.fast_conservative=bool(state['events']) or any(c['kind']!='prism' for c in state['cells'].values()) or len({c['last_prism_layer'] for c in state['columns'].values()})!=1
                runner.provider=run.provider;run.provider.compute_observer=observe
                last_commit=time.monotonic()
                last_checkpoint=a.output/f'milestone_general_{len(rows):02d}'
                milestone.save(run,last_checkpoint)
                last_saved=max(c['layer'] for c in run.topology.cells.values())
                restored=milestone.load(last_checkpoint,a.output)
                save(a.output/f'GENERAL_RESTART_{len(rows):02d}.json',dict(passed=milestone.signature(restored)==milestone.signature(run),checkpoint=str(last_checkpoint)))
                mem=psutil.Process().memory_info()
                rows.append(dict(operation=result['operation'],layer=last_saved,seconds=elapsed,peak_rss=mem.peak_wset,retained_rss=mem.rss,cells=len(state['cells']),relations=len(run.proofs)))
                print('GENERAL_COMMITTED',json.dumps(rows[-1]),flush=True)
                # Ordinary fallback costs obey the same process guard. A remesh
                # event is recorded separately, not silently called ordinary.
                if result['operation']=='PrismCommit' and elapsed>120:reason='PerformanceGuardStopped';break
                continue
            run=runner.fast_session;last_commit=time.monotonic();mem=psutil.Process().memory_info()
            result=dict(result,peak_rss=mem.peak_wset,retained_rss=mem.rss,wall_step_seconds=time.perf_counter()-tick)
            degree=dict(result['degree_metrics']);degree.pop('delta',None);result['degree_metrics']=degree
            rows.append(result);save(a.output/f'LAYER_{result["layer"]:02d}.json',result)
            print('COMMITTED',json.dumps({k:result[k] for k in ('layer','seconds','timings','cells','relations','new_relations','peak_rss','retained_rss')}),flush=True)
            if result['layer']-last_saved>=4:
                tick=time.perf_counter();path=a.output/f'milestone_{result["layer"]:02d}';record=milestone.save(run,path)
                restored=milestone.load(path,a.output)
                save(a.output/f'RESTART_{result["layer"]:02d}.json',dict(passed=True,signature=record['signature'],seconds=time.perf_counter()-tick))
                run=restored;runner.fast_session=run;runner.provider=run.provider;run.provider.compute_observer=observe;last_saved=result['layer'];last_checkpoint=path;gc.collect()
            if result['seconds']>120:reason='PerformanceGuardStopped';break
        else:reason='ShellQualified'
        final=max(c['layer'] for c in run.topology.cells.values())
        if final!=last_saved or last_checkpoint is None:
            last_checkpoint=a.output/f'milestone_{final:02d}';milestone.save(run,last_checkpoint)
        print('SEALING',final,flush=True);tick=time.perf_counter();sealed=seal_new(run,a.output,proof_checkpoint=last_checkpoint)
        save(a.output/'SEAL.json',sealed);save(a.output/'SEAL_RUNTIME.json',dict(seconds=time.perf_counter()-tick))
        from qualify_broader_geometry import export
        interface=export(run.tx.current,a.output/'interface')
        complete=not remaining(run.tx.current,requested)
        save(a.output/'SHELL_RESULT.json',dict(passed=True,all_active_schedules_complete=complete,current_layer=final,requested_layers=requested,cells=len(run.topology.cells),relations=len(run.proofs),interface=interface,checkpoint=str(last_checkpoint)))
        save(a.output/'END.json',dict(reason=reason,start_layer=first,final_layer=final,layers_committed=len(rows),shell_complete=complete,seconds=time.perf_counter()-started,remesh_events=len(run.tx.current['events'])))
        print('DONE',reason,final,flush=True)
    except BaseException as exc:
        save(a.output/'FAILURE.json',dict(error=str(exc),traceback=traceback.format_exc(),seconds=time.perf_counter()-started,last_layer=max((c['layer'] for c in run.topology.cells.values()),default=0) if run else None))
        raise
    finally:save(a.output/'SESSION.json',dict(seconds=time.perf_counter()-started))

if __name__=='__main__':main()

"""One bounded existing correction stage, using rejected nominal evidence."""
import argparse,gc,gzip,json,pickle,sys,time,traceback
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT/'src'),str(ROOT/'tools')]
import numpy as np
import fast_milestone, native_ordinary_correction as correction
import generalize_local_remesh as g
from bounded_fallback_control import Events,AttemptStopped,BestEvidence,require_space
from investigate_large_fallback import save
from sdf_cfd_mesher_v4.common.authority import file_hash,array_hash


def main():
    p=argparse.ArgumentParser();p.add_argument('--source',type=Path,required=True);p.add_argument('--nominal',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--stage',choices=['tangent','compression','branch','continuation','remesh'],required=True)
    p.add_argument('--trials',type=int,default=8);p.add_argument('--seconds',type=float,default=600)
    p.add_argument('--fixed-guard',action='store_true')
    p.add_argument('--compact',action='store_true')
    p.add_argument('--seed-trial',type=Path)
    p.add_argument('--rollback-depth',type=int,default=0)
    p.add_argument('--support-from',type=Path)
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True);events=Events(a.output/'EVENTS.jsonl',a.seconds)
    summary=dict(stage=a.stage,accepted=False,counts=[])
    try:
        events('stage_start',stage='checkpoint_load',guard=require_space(a.output,
            (a.source/'STATE.pickle.gz').stat().st_size,(a.nominal/'CHECKED.pickle.gz').stat().st_size))
        run=fast_milestone.load(a.source,a.output);provider=run.provider;before=run.tx.current
        del run;gc.collect()
        events('stage_end',stage='checkpoint_load')
        authority=json.loads((a.nominal/'REPRODUCER_AUTHORITY.json').read_text())
        for key,name in [('arrays','REPRODUCER.npz'),('checked','CHECKED.pickle.gz')]:
            if file_hash(a.nominal/name)!=authority[key]:raise ValueError('ReproducerCorruption')
        z=np.load(a.nominal/'REPRODUCER.npz');fronts=list(z['fronts']);levels=z['levels'];high=fronts[-1];prefix=fronts[:-1]
        with gzip.open(a.nominal/'CHECKED.pickle.gz','rb') as f:checked=pickle.load(f)
        if a.seed_trial:
            events('scope_start',scope='retained_best_candidate')
            from local_correction_validation import LocalCorrectionValidation
            local=LocalCorrectionValidation(provider.x,provider.tri,fronts,levels,checked)
            high=np.load(a.seed_trial)['front'];fronts[-1]=high
            checked=local.evaluate(np.concatenate([provider.x]+fronts));del local;gc.collect()
            events('scope_end',scope='retained_best_candidate',failures=len(checked['quality']['failures']),coordinate_hash=array_hash(high))
        provider.correction_base=before;best=BestEvidence(checked);trial_number=0
        provider.fixed_trial_guard=a.fixed_guard
        if a.compact:
            from correction_trial_storage import CompactGateArchive
            provider.trial_archive_factory=CompactGateArchive
        def trial_observer(trial,check,targets):
            np.savez_compressed(a.output/f'TRIAL_{trial_number:03d}.npz',front=trial,targets=targets)
            events('trial_geometry',trial=trial_number,coordinate_hash=array_hash(trial),target_hash=array_hash(targets),
                   changed_vertices=int(np.count_nonzero(np.any(trial!=high,axis=1))),
                   max_displacement=float(np.max(np.linalg.norm(trial-high,axis=1))),
                   adjusted_targets=int(np.count_nonzero(targets!=provider.schedule.targets[len(prefix)])))
            from bounded_fallback_control import failure_keys
            if failure_keys(check)<best.keys:
                np.savez_compressed(a.output/'BEST.npz',front=trial,targets=targets)
                save(a.output/'BEST_FAILURES.json',check['quality']['failures'])
        provider.trial_observer=trial_observer
        def progress(event,**row):
            nonlocal trial_number
            if event=='trial_start':
                if trial_number>=a.trials:raise AttemptStopped('BoundedSystematicSearchSegmentComplete')
                trial_number+=1
            events(event,trial=trial_number,**row)
        def observe(check,layer):
            status=best.observe(check);summary['counts'].append(status)
            events('trial_quality',trial=trial_number,layer=layer,classes=dict(g.t.Counter(r['kind'] for r in check['quality']['failures'])),**status)
            save(a.output/'LATEST_FAILURES.json',dict(rows=check['quality']['failures'],**status))
            if best.no_new_diagnosis>=10:raise AttemptStopped('TenRejectedCandidatesWithoutNewDiagnosis')
        provider.search_progress=progress;provider.compute_observer=observe
        events('stage_start',stage=a.stage)
        tick=time.perf_counter()
        if a.stage in ('tangent','compression'):
            result=correction.correct(provider,prefix,levels,high,checked,stages=[a.stage])
            trial,targets,check,history=result
            save(a.output/'HISTORY.json',history)
            if trial is not None:
                np.savez_compressed(a.output/'CANDIDATE.npz',front=trial,targets=targets)
                summary.update(candidate_passed=check['passed'],candidate_hash=array_hash(trial))
        elif a.stage in ('branch','continuation'):
            sdf=provider.geometry_source.Source(provider.spec);target=provider.schedule.targets[len(prefix)]
            if a.stage=='branch':trial=g.q.predict_next_front(prefix[-1],target,sdf,iteration_cap=256).coordinates
            else:
                from branch_aware_projection_study import continue_front
                trial=continue_front(prefix[-1],provider.schedule.targets[len(prefix)-1],target,sdf,provider.spec,
                    sdf.evaluate(provider.x).boundary_normal_selection,
                    owner_provider=lambda p:sdf.evaluate(p).boundary_normal_selection).coordinates
            events('scope_end',scope='front_proposal',displacement=float(np.max(np.linalg.norm(trial-high,axis=1))))
            from local_correction_validation import LocalCorrectionValidation
            local=LocalCorrectionValidation(provider.x,provider.tri,fronts,levels,checked)
            check=local.evaluate(np.concatenate([provider.x]+prefix+[trial]));observe(check,len(fronts))
            np.savez_compressed(a.output/'CANDIDATE.npz',front=trial)
            summary.update(candidate_numeric_passed=check['quality']['passed'])
        else:
            seed,columns=g.failure_vertices(provider.tri,checked['quality'])
            if a.support_from:
                old=json.loads(a.support_from.read_text())
                last=next(r for r in reversed(old) if 'active' in r)
                seed.update(last['active'])
            if a.rollback_depth<0 or len(fronts)-a.rollback_depth<=1:raise ValueError('FirstLayerAuthorityProtected')
            events('remesh_seed',vertices=sorted(seed),columns=columns,reason='Direct failed exact Jacobian and hard-quality dependencies; diagnostic escalation, not acceptance')
            def watch(event,**row):events(event,**row)
            levels,history=g.discover(provider.x,provider.tri,fronts,seed,len(fronts)-a.rollback_depth,candidate_check=provider.candidate_check,progress=watch,attempt_limit=a.trials)
            save(a.output/'HISTORY.json',history)
            summary.update(cavity_constructed=levels is not None)
            if levels is not None:np.save(a.output/'LEVELS.npy',levels)
        summary['stage_seconds']=time.perf_counter()-tick
        events('stage_end',stage=a.stage,seconds=summary['stage_seconds'])
    except BaseException as exc:
        summary.update(stop=str(exc),traceback=traceback.format_exc())
        try:events('bounded_stop',reason=str(exc),next_action='Preserve committed authority; inspect evidence before next mechanism')
        except AttemptStopped:pass
    finally:save(a.output/'RESULT.json',summary)

if __name__=='__main__':main()

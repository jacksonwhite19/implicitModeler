"""Wire existing tangent/compression searches to native mixed validation.

No change to proposal fields, trial sequences, projection, or hard gates.
The current committed cells are immutable. A failed frozen-cell determinant
cannot be changed by moving vertices on the next front.
"""
from types import SimpleNamespace
import time
import numpy as np
import general_local_correction as search
import generalize_local_remesh as g


def frozen_failures(state,proposal):
    found=[]
    for row in proposal['check']['quality']['failures']:
        key=str(row.get('id',''))
        if row['kind'] not in ('cell_quality','exact_cell','final_incidence_determinant') or key not in state['cells']:continue
        old=state['cells'][key];new=proposal['state']['cells'].get(key)
        if old==new and all(state['vertices'][v]==proposal['state']['vertices'][v] for v in old['vertices']):
            found.append(row)
    return found


def _failure_key(row):
    return (row['kind'],str(row.get('id','')),tuple(sorted(row.get('owners',[]))),row.get('vertex'))


def source_failures(source,faces,points,targets,required=None):
    sample=source.evaluate(points);error=abs(sample.phi-targets)
    eligible=np.ones(len(points),bool) if required is None else np.asarray(required,bool)
    return [dict(kind='sdf_target',vertex=int(v),column=int(np.flatnonzero(np.any(faces==v,axis=1))[0]),
                 residual=float(error[v]),bound=float(sample.source_error_bound[v]))
            for v in np.flatnonzero((error>sample.source_error_bound)&eligible)]


def correct(provider,prefix,levels,high,failed,allow_compression=True,suffix=None,stages=None):
    """Schedule bounded existing searches; never commit a partial correction."""
    if stages is not None and not set(stages)<={'tangent','compression'}:
        raise ValueError('UnknownCorrectionStage')
    # Only the failure list is extended. Certified evidence and numerical
    # rows are read-only during provisional local correction.
    history=[];baseline=dict(failed)
    baseline['quality']=dict(failed['quality'],failures=list(failed['quality']['failures']))
    source=provider.geometry_source.Source(provider.spec)
    baseline['quality']['failures']+=source_failures(source,provider.tri,high,provider.schedule.targets[len(prefix)],np.asarray(levels)==len(prefix)+1)
    while True:
        args=(provider,prefix,levels,high,baseline,allow_compression,suffix or [])
        result=_one_patch(*args) if stages is None else _one_patch(*args,stages=stages)
        trial,targets,checked,records=result;history.extend(records)
        if trial is None:return None,None,None,history
        if checked['passed']:return trial,targets,checked,history
        # Each temporary tangent update must remove failures without introducing
        # another failing entity. Global admission still requires every gate.
        old={_failure_key(r) for r in baseline['quality']['failures']}
        new={_failure_key(r) for r in checked['quality']['failures']}
        if not new<old:return None,None,None,history
        if not np.all(targets==provider.schedule.targets[len(prefix)]):return None,None,None,history
        high=trial;baseline=checked


def _one_patch(provider,prefix,levels,high,failed,allow_compression,suffix,stages=None):
    x,f=provider.x,provider.tri;sdf=provider.geometry_source.Source(provider.spec)
    top=g.q.build_topology(f,len(x))
    # Independent difficult regions must not be combined into a global patch.
    seed=dict(failed['quality'],failures=failed['quality']['failures'][:1])
    vertices,columns=g.failure_vertices(f,seed)
    sample=sdf.evaluate(high);target=provider.schedule.targets[len(prefix)]
    unadmitted=np.flatnonzero(abs(sample.phi-target)>sample.source_error_bound)
    if not vertices and len(unadmitted):vertices.add(int(unadmitted[0]))
    active=set(vertices)
    for v in sorted(vertices):active.update(map(int,top.vertex_neighbors.row(v)))
    patch=g.q.bounded_patch(top,np.array(sorted(active),int))
    case=SimpleNamespace(x=x,f=f,top=top,patch=patch,verify=patch.detailed,s=provider.schedule,
        sdf=sdf,spec=provider.spec,active=np.array(sorted(active),int),prefix=prefix,previous=None,
        support=dict(controlling_vertices=sorted(vertices)),ancestry=sdf.evaluate(x).boundary_normal_selection)
    case.evaluation_targets=np.full(len(x),target)
    progress=getattr(provider,'search_progress',None)
    case.search_progress=progress
    def event(name,**data):
        if progress:progress(name,**data)
    accepted=[]
    local_evaluator=None
    fixed_guard=None
    if 'complete_incidence' in failed:
        event('scope_start',scope='local_evaluator_build')
        from local_correction_validation import LocalCorrectionValidation
        local_evaluator=LocalCorrectionValidation(x,f,prefix+[high]+suffix,levels,failed)
        event('scope_end',scope='local_evaluator_build')
        before=getattr(provider,'correction_base',None)
        if before is not None and getattr(provider,'fixed_trial_guard',True):
            from fixed_trial_guard import FixedTrialGuard
            tick=time.perf_counter();event('scope_start',scope='fixed_guard_bootstrap')
            fixed_guard=FixedTrialGuard(before,provider.pack(prefix+[high]+suffix,levels,before),local_evaluator.points,len(x))
            event('scope_end',scope='fixed_guard_bootstrap',seconds=time.perf_counter()-tick,protected_vertices=len(fixed_guard.indices))
    original_failures={_failure_key(r) for r in failed['quality']['failures']}
    def validate(x,f,top,verify,prefix,trial,previous):
        fronts=prefix+[trial]+suffix
        tick=time.perf_counter();event('scope_start',scope='trial_numeric_quality')
        if local_evaluator is not None:
            check=local_evaluator.evaluate(np.concatenate([x]+fronts));times={}
        else:
            p,c,o,check,times=g.validation(x,f,fronts,levels,False)
            if hasattr(provider,'candidate_quality'):
                check=provider.candidate_quality(check,fronts,levels)
        event('scope_end',scope='trial_numeric_quality',seconds=time.perf_counter()-tick,failures=len(check['quality']['failures']))
        before=getattr(provider,'correction_base',None)
        if before is not None:
            tick=time.perf_counter();event('scope_start',scope='committed_guard_pack')
            from sdf_cfd_mesher_v4.advancing_front.mixed_front_runner import preserves_committed_cells
            preserved=(fixed_guard.preserves(np.concatenate([x]+fronts)) if fixed_guard is not None
                       else preserves_committed_cells(provider.pack(fronts,levels,before),before))
            if not preserved:
                check['quality']['failures'].append(dict(kind='committed_cell_mutation'))
                check['quality']['passed']=False;check['passed']=False
            event('scope_end',scope='committed_guard_pack',seconds=time.perf_counter()-tick,mesh_cache_entries=len(provider.meshes),fixed_topology_reuse=fixed_guard is not None)
        sdf_failures=source_failures(sdf,f,trial,case.evaluation_targets,np.asarray(levels)==len(prefix)+1)
        if sdf_failures:
            check['quality']['failures']+=sdf_failures
            check['quality']['passed']=False;check['passed']=False
        if check['quality']['passed']:
            event('scope_start',scope='complete_candidate_validation')
            if local_evaluator is not None:
                _,_,_,full,times=g.validation(x,f,fronts,levels,False)
                full=provider.candidate_quality(full,fronts,levels)
                if full['quality']!=check['quality']:raise ValueError('LocalCorrectionMetricMismatch')
                check=full
            if hasattr(provider,'certify_candidate'):check=provider.certify_candidate(fronts,levels,check)
            else:p,c,o,check,times=g.validation(x,f,fronts,levels,True)
            event('scope_end',scope='complete_candidate_validation',passed=check['passed'])
        remaining={_failure_key(r) for r in check['quality']['failures']}
        trial_observer=getattr(provider,'trial_observer',None)
        if trial_observer is not None:trial_observer(trial,check,case.evaluation_targets)
        observer=getattr(provider,'compute_observer',None)
        if observer is not None:observer(check,len(prefix)+1)
        partial=bool(remaining and remaining<original_failures and check['ownership']['passed'] and check['vertex_links']['passed'])
        if partial:
            # This is a provisional local-search step, never an admission
            # receipt. Exact global certification occurs on the final candidate.
            local=dict(check,passed=True,scope='ProvisionalLocalProgressOnly',production_acceptance=False)
            accepted[:]=[(trial,check)]
            return local,None,0.
        if check['passed']:accepted[:]=[(trial,check)]
        return check,None,times.get('exact_certificate',0.)
    from correction_trial_storage import GateArchive
    archive_factory=getattr(provider,'trial_archive_factory',GateArchive)
    archive=archive_factory(provider.folder/'CORRECTION_TRIALS')
    def record(gate):
        tick=time.perf_counter();event('scope_start',scope='trial_archive')
        row=archive.record(gate)
        event('scope_end',scope='trial_archive',seconds=time.perf_counter()-tick,record=row)
        return row
    tangent,trials=(search.tangent_search(case,prefix,None,high,validator=validate,record_gate=record)
                    if stages is None or 'tangent' in stages else (None,[]))
    history=[dict(stage='TangentialCorrection',trials=trials,passed=tangent is not None)]
    if tangent is not None:return tangent[0],np.full(len(x),target),accepted[-1][1],history
    if prefix and allow_compression and (stages is None or 'compression' in stages):
        previous=sdf.evaluate(prefix[-1]).phi
        # The actual target values are persisted separately after acceptance.
        compressed,trials,times,weights=search.search_compression(case,nominal=high,previous_targets=previous,validator=validate,record_gate=record)
        history.append(dict(stage='CompressionCorrection',trials=trials,times=times,passed=compressed is not None))
        if compressed is not None:return compressed[0],compressed[1],accepted[-1][1],history
    return None,None,None,history

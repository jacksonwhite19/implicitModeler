"""New SDF authorities through the existing native production orchestrator."""
import sys
sys.dont_write_bytecode=True
import argparse,json,time,traceback
from validation_row_stream import validation_digest
from pathlib import Path
from collections import Counter
import psutil
import numpy as np
import geometry_family_authority as geometry
from mixed_front_fixture_provider import Provider
import generalize_local_remesh as g
from qualify_broader_geometry import export
from sdf_cfd_mesher_v4.advancing_front.mixed_front import scientific_hash,rebuild_dependencies
from sdf_cfd_mesher_v4.advancing_front.mixed_front_runner import MixedFrontRunner
from sdf_cfd_mesher_v4.common.authority import file_hash,identity

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'results/broader_geometry_family_qualification'


def committed_mutations(before,candidate):
    return [k for k,c in before['cells'].items() if candidate['cells'].get(k)!=c or any(candidate['vertices'].get(v)!=before['vertices'][v] for v in c['vertices'])]


def remaining_columns(state,requested=20):
    return sorted(k for k in state['prism_front'] if int(state['columns'][k]['last_prism_layer'])<requested)


def source_failure_seed(source,proposal,target):
    layer=proposal['layer'];sample=source.evaluate(proposal['fronts'][layer-1])
    required=np.asarray(proposal['levels'])==layer
    return set(map(int,np.flatnonzero(required & (abs(sample.phi-target)>sample.source_error_bound))))


def stage_receipts(history,context,accepted=None):
    from sdf_cfd_mesher_v4.advancing_front.termination_policy import STAGES
    return [dict(context=context,proof=identity([r for r in history if r['stage']==phase]),stage=phase,
        status='succeeded' if phase==accepted else 'failed') for phase in STAGES if any(r['stage']==phase for r in history)]


class AnalyticProvider(Provider):
    """Native SDF adapter with generic local-cohort and candidate validation.

    Policy decisions and cavity construction use the existing production
    machinery. No case identity selects a correction or remesh outcome.
    """
    def __init__(self,spec,folder):
        super().__init__(spec,folder,geometry)
        self.authority['sdf']=geometry.Source(spec).source_identity
        from native_mixed_face_certificate import SOURCE
        self.authority['certificate_filter']=file_hash(SOURCE)
        import first_layer_metric_state
        self.authority['first_prism_metric']=file_hash(Path(first_layer_metric_state.__file__))
        self.reuse_certificates=True
    def initial_quality(self,checked,high):
        """Audit the fixed first-layer tensor with its eventual top cap internal.

        Face-role closure supplies the exact final tensor inputs without a
        fabricated neighbor centroid. Pair metrics remain scoped to actual
        neighbors. Mixed transition complete-neighbor validation is separate.
        """
        return self.candidate_quality(checked,[high],np.ones(len(self.x),int))

    def candidate_quality(self,checked,fronts,levels):
        from final_shell_incidence import merge
        p,c,o=g.s.construct(self.x,self.tri,fronts,levels)
        from first_layer_metric_state import eligible,apply
        if eligible(c):return apply(checked,p,c,o)
        return merge(checked,p,c,o)

    def candidate_check(self,points,cells,outer):
        progress=getattr(self,'search_progress',None)
        tick=time.perf_counter()
        if progress:progress('scope_start',scope='provisional_complete_neighbor')
        result=super().candidate_check(points,cells,outer)
        if progress:progress('scope_end',scope='provisional_complete_neighbor',seconds=time.perf_counter()-tick,passed=result['passed'],closures=result.get('closures'))
        from final_shell_incidence import check
        tick=time.perf_counter()
        if progress:progress('scope_start',scope='final_shell_incidence')
        incidence=check(points,cells,outer)
        if progress:progress('scope_end',scope='final_shell_incidence',seconds=time.perf_counter()-tick,passed=incidence['passed'])
        result['failures']=result['failures']+incidence['failures']
        result['passed']=result['passed'] and incidence['passed']
        result['final_incidence']=incidence
        return result

    def certify_candidate(self,fronts,levels,checked):
        from reuse_mixed_certificate import certify
        candidate=self.pack(fronts,levels)
        p,c,o=g.s.construct(self.x,self.tri,fronts,levels)
        owners,_=g.t.ownership(c,o,[tuple(reversed(f)) for f in self.tri]);faces=[owners[k][0][1] for k in sorted(owners)]
        cert=certify(p,faces,candidate,getattr(self,'correction_base',self.previous_certificate_state))
        volume=g.volume.audit(p,c,o,[tuple(map(int,f)) for f in self.tri])
        checked.update(certificate=cert,volume=volume,passed=checked['quality']['passed'] and checked['ownership']['passed'] and checked['vertex_links']['passed'] and cert['passed'] and volume['exact_volume_balance'] and volume['positive_cells'] and volume['interior_degree']['passed'])
        return checked

    def continuation_front(self,fronts,levels,target,active):
        # Project from the committed front, not the original wall normal ray.
        # Unused terminated coordinates remain outside admission support.
        high=geometry.front(self.spec,target)
        active=np.asarray(active,int)
        high[active]=g.q.predict_next_front(fronts[-1][active],target,geometry.Source(self.spec),iteration_cap=256).coordinates
        return high

    def continuation_validator(self,fronts,levels):
        p,c,o,checked,times=g.validation(self.x,self.tri,fronts,levels,False)
        if checked['quality']['passed']:
            coupled=self.candidate_check(p,c,o)
            if not coupled['passed']:
                checked['quality']['failures']+=coupled['failures'];checked['quality']['passed']=False
            else:checked=self.certify_candidate(fronts,levels,checked)
        return p,c,o,checked,times

    def partitioned_proposal(self,state,fronts,levels,layer,columns):
        from source_component_support import groups
        supports=groups(self.tri,fronts[layer-1],columns)
        if len(supports)<2:return None
        # Commit one independent source support. Other supports retain their
        # committed layer and remain active for a later native rolling step.
        _,base=self.geometry(state);trial=base.copy()
        active=np.unique(self.tri[supports[0]]);trial[active]=layer
        _,_,_,checked,_=g.validation(self.x,self.tri,fronts,trial,False)
        checked=self.candidate_quality(checked,fronts,trial)
        candidate=self.pack(fronts,trial,state)
        changed=committed_mutations(state,candidate)
        if changed:
            checked['quality']['failures'] += [dict(kind='committed_cell_mutation',id=int(k),column=int(state['cells'][k]['source'])) for k in changed]
            checked['quality']['passed']=False;checked['passed']=False
        admitted=geometry.sdf_record(self.spec,fronts[layer-1][active],self.schedule.targets[layer-1])['admitted']
        if checked['quality']['passed'] and admitted:
            checked=self.certify_candidate(fronts,trial,checked)
        g.save(self.folder/f'DEPENDENCY_GROUPS_{identity(state["prism_front"])}.json',dict(layer=layer,groups=supports,selected=supports[0],base_layers=base.tolist(),candidate_layers=trial.tolist()))
        return dict(state=candidate,fronts=fronts,levels=trial,check=checked,layer=layer)

    def propose(self,state):
        # The candidate may borrow only this committed base during its step.
        self.correction_base=state
        if state['events']:
            # Advance an actual exposed cohort at its own current layer. A
            # maximum layer elsewhere is not the schedule of this cohort.
            fronts,levels=self.geometry(state)
            groups={}
            for source in state['prism_front']:
                column=int(source);depths=levels[self.tri[column]]
                if np.all(depths==depths[0]) and int(depths[0])<len(self.schedule.targets):
                    groups.setdefault(int(depths[0]),[]).append(column)
            self.previous_certificate_state=state;self.correction_base=state
            attempts=[]
            from sdf_cfd_mesher_v4.advancing_front.mixed_front_runner import preserves_committed_cells
            for depth,columns in sorted(groups.items()):
                active=np.unique(self.tri[columns]);target=self.schedule.targets[depth]
                trial=levels.copy();trial[active]=depth+1
                proposed=[v.copy() for v in fronts]
                if depth==len(proposed):proposed.append(geometry.front(self.spec,target))
                try:
                    proposed[depth][active]=g.q.predict_next_front(fronts[depth-1][active],target,geometry.Source(self.spec),iteration_cap=256).coordinates
                    admitted=geometry.sdf_record(self.spec,proposed[depth][active],target)['admitted']
                    _,_,_,checked,_=g.validation(self.x,self.tri,proposed,trial,False)
                    checked=self.candidate_quality(checked,proposed,trial)
                    candidate=self.pack(proposed,trial,state)
                    preserved=preserves_committed_cells(candidate,state)
                    row=dict(layer=depth+1,columns=columns,admitted=admitted,preserved=preserved,quality=checked['quality']['passed'],quality_failures=checked['quality']['failures'],changed_committed_cells=committed_mutations(state,candidate))
                    attempts.append(row)
                    if not preserved:
                        checked['quality']['failures'] += [dict(kind='committed_cell_mutation',id=int(k),column=int(state['cells'][k]['source'])) for k in row['changed_committed_cells']]
                        checked['quality']['passed']=False;checked['passed']=False
                    elif admitted and checked['quality']['passed']:
                        checked=self.certify_candidate(proposed,trial,checked)
                    if not checked['passed']:
                        partition=self.partitioned_proposal(state,proposed,trial,depth+1,columns)
                        if partition is not None:return partition
                    g.save(self.folder/f'LOCAL_ADVANCE_{identity(state["prism_front"])}.json',attempts)
                    return dict(state=candidate,fronts=proposed,levels=trial,check=checked,layer=depth+1)
                except (ValueError,RuntimeError) as exc:
                    attempts.append(dict(layer=depth+1,columns=columns,rejection=str(exc)))
                    partition=self.partitioned_proposal(state,proposed,trial,depth+1,columns)
                    if partition is not None:return partition
                    # A rejected projection is evidence for this cohort, not
                    # permission to skip it. Submit its finite nominal proposal
                    # to the existing correction/termination policy.
                    from native_ordinary_correction import source_failures
                    _,_,_,checked,_=g.validation(self.x,self.tri,proposed,trial,False)
                    checked=self.candidate_quality(checked,proposed,trial)
                    failed=source_failures(geometry.Source(self.spec),self.tri,proposed[depth],target,trial==depth+1)
                    checked['quality']['failures']+=failed
                    if failed:checked['quality']['passed']=False
                    checked['passed']=False
                    candidate=self.pack(proposed,trial,state)
                    changed=committed_mutations(state,candidate)
                    if changed:
                        checked['quality']['failures'] += [dict(kind='committed_cell_mutation',id=int(k),column=int(state['cells'][k]['source'])) for k in changed]
                        checked['quality']['passed']=False
                    g.save(self.folder/f'LOCAL_ADVANCE_{identity(state["prism_front"])}.json',attempts)
                    return dict(state=candidate,fronts=proposed,levels=trial,check=checked,layer=depth+1)
            g.save(self.folder/f'LOCAL_ADVANCE_{identity(state["prism_front"])}.json',attempts)
            if len(fronts)>=len(self.schedule.targets):raise ValueError('RemainingActiveColumnsHaveNoAcceptedContinuation')
        proposal=super().propose(state)
        proposal['check']=self.candidate_quality(proposal['check'],proposal['fronts'],proposal['levels'])
        if not proposal['check']['passed']:
            partition=self.partitioned_proposal(state,proposal['fronts'],proposal['levels'],proposal['layer'],[int(k) for k in state['prism_front']])
            if partition is not None:return partition
        return proposal

    def numerical_validation(self,state,proposal,cavity):
        receipt=super().numerical_validation(state,proposal,cavity)
        entry=self.numerical[scientific_hash(state)];p,c,o,checked=entry[:4]
        from final_shell_incidence import merge
        merge(checked,p,c,o)
        if not checked['quality']['passed']:raise ValueError('FinalShellIncidenceFailure')
        g.save(self.folder/f'FINAL_INCIDENCE_{proposal["layer"]}.json',checked['complete_incidence'])
        return receipt

    def rebuild(self,before,state,receipt):
        checked=self.checked[receipt['state']];result=checked.get('complete_incidence')
        state=super().rebuild(before,state,receipt)
        if result is not None:
            from sdf_cfd_mesher_v4.advancing_front.mixed_front import geometry_inputs,face_key
            inputs=geometry_inputs(state)
            for row in result['cells']:
                cell=str(row['id']);deps=['cell:'+cell,'authority:quality']+['face:'+face_key(f) for f in state['cells'][cell]['faces']]
                state['quality']['final_incidence:'+cell]=dict(inputs={k:inputs[k] for k in deps},value=row,
                    receipt_class='FinalIncidenceDeterminant',final_core_quality=False)
        return state
    def initialize(self):
        # No scan ahead or loading a pre-trigger private study. Begin at the
        # immutable first layer, then commit each next step through the runner.
        fronts=[geometry.front(self.spec,self.schedule.targets[0])]
        p,c,o,checked,times=g.validation(self.x,self.tri,fronts,[1]*len(self.x),False)
        checked=self.initial_quality(checked,fronts[0])
        assigned=geometry.sdf_record(self.spec,fronts[0],self.schedule.targets[0])
        if checked['quality']['passed'] and assigned['admitted']:
            checked=self.certify_candidate(fronts,np.ones(len(self.x),int),checked)
        g.save(self.folder/'FIRST_LAYER.json',dict(quality=checked['quality'],passed=checked['passed'],sdf=assigned,times=times))
        if not checked['passed'] or not assigned['admitted']:
            from native_ordinary_correction import correct
            high,targets,check,history=correct(self,[],np.ones(len(self.x),int),fronts[0],checked,False)
            g.save(self.folder/'FIRST_LAYER_CORRECTION.json',history)
            if high is None:raise ValueError('FirstLayerCorrectionExhausted')
            fronts=[high];checked=check
        state=self.pack(fronts,[1]*len(self.x));self.bind(state,checked)
        rebuild_dependencies(None,state,self.schedule.targets.tolist())
        from first_layer_metric_state import receipt
        p,c,o=g.s.construct(self.x,self.tri,fronts,[1]*len(self.x))
        state['quality']['first_prism_incidence']=receipt(p,c,o,state['versions'])
        return state

    def validate_initial_commit(self,state):
        """Validate actual native entities again before atomic publication."""
        from first_layer_metric_state import verify
        fronts,levels=self.geometry(state)
        p,c,o=g.s.construct(self.x,self.tri,fronts,levels)
        candidate=self.pack(fronts,levels)
        if any(candidate[k]!=state[k] for k in ('cells','vertices','faces','outer')):
            raise ValueError('FirstPrismCommittedTopologyMismatch')
        verify(state['quality']['first_prism_incidence'],p,c,o,state['versions'])
        return True

    def evidence(self,state,proposal):
        from native_ordinary_correction import correct,frozen_failures
        from sdf_cfd_mesher_v4.advancing_front.termination_policy import IMMEDIATE_CHECKS,STAGES
        self.correction_base=state
        mutations=[r for r in proposal['check']['quality']['failures'] if r['kind']=='committed_cell_mutation']
        if mutations:
            context=dict(base=scientific_hash(state),region=identity(state['prism_front']),layer=proposal['layer'],topology=identity(state['faces']),schedule=self.authority['schedule'])
            ref=dict(context=context,proof=identity(mutations));columns=sorted({r['column'] for r in mutations})
            seed=dict(vertices=sorted({int(v) for c in columns for v in self.tri[c]}),columns=columns,cells=[r['id'] for r in mutations],faces=[])
            return dict(context=context,support=dict(ref,status='clean'),immediate=dict(ref,status='invalid',seed=seed),warnings=[ref['proof']],witnesses=[],accepted=None,
                attempts=[dict(ref,stage=stage,status='inapplicable',reason='The candidate changes committed cells. Ordinary advancement cannot publish this topology; the existing rollback journal is required.') for stage in STAGES])
        if proposal['check']['passed']:return super().evidence(state,proposal)
        g.save(self.folder/f'PROPOSAL_{proposal["layer"]}.json',dict(check=proposal['check'],
            sdf=geometry.sdf_record(self.spec,proposal['fronts'][-1],self.schedule.targets[proposal['layer']-1])))
        fixed=frozen_failures(state,proposal)
        if fixed:
            g.save(self.folder/f'FIXED_SUPPORT_{proposal["layer"]}.json',dict(failures=fixed,
                proof='Cell face area vectors and complete internal-face selection are fixed; all ordinary next-front changes leave the determinant unchanged',
                first_layer_affected=any(r['id']//1000000==1 for r in fixed)))
            if any(r['id']//1000000==1 for r in fixed):raise ValueError('ImmutableFirstLayerCoupledDeterminantFailure')
        try:return super().evidence(state,proposal)
        except ValueError as exc:
            if str(exc)!='OrdinaryCorrectionProviderRequired':raise
        self.correction_base=state
        history=[];high=None
        if not fixed:
            index=proposal['layer']-1
            high,targets,checked,history=correct(self,proposal['fronts'][:index],proposal['levels'],proposal['fronts'][index],proposal['check'],suffix=proposal['fronts'][index+1:])
            g.save(self.folder/f'CORRECTION_{proposal["layer"]}.json',history)
        if high is not None:
            proposal['fronts'][proposal['layer']-1]=high
            proposal['state']=self.pack(proposal['fronts'],proposal['levels'],state)
            local=dict(state.get('local_schedule',{}));actual=dict(local.get('actual_targets',{}))
            actual[str(proposal['layer'])]=targets.tolist();local['actual_targets']=actual
            local['nominal_targets']=self.schedule.targets.tolist()
            local['thickness_debt']=(self.schedule.targets[proposal['layer']-1]-targets).tolist()
            proposal['state']['local_schedule']=local;proposal['check']=checked
            evidence=super().evidence(state,proposal)
            stage=history[-1]['stage'];evidence['accepted']['stage']=stage
            evidence['attempts']=stage_receipts(history,evidence['context'],stage)
            return evidence
        if fixed:
            context=dict(base=scientific_hash(state),region=identity(state['prism_front']),layer=proposal['layer'],
                         topology=identity(state['faces']),schedule=self.authority['schedule'])
            ref=dict(context=context,proof=identity(fixed));columns=sorted({r['column'] for r in fixed})
            seed=dict(vertices=sorted(set(map(int,self.tri[columns].ravel()))),columns=columns,cells=[r['id'] for r in fixed],faces=[])
            return dict(context=context,support=dict(ref,status='clean'),immediate=dict(ref,status='invalid',seed=seed),
                warnings=[ref['proof']],witnesses=[],accepted=None,attempts=[dict(ref,stage=stage,status='inapplicable',
                reason='Ordinary next-front correction cannot change already committed cell area vectors or its full internal-face determinant') for stage in STAGES])
        # Growth redistribution only changes uncommitted future heights. It
        # cannot validate an immediate candidate rejected at its current target.
        context=dict(base=scientific_hash(state),region=identity(state['prism_front']),layer=proposal['layer'],
                     topology=identity(state['faces']),schedule=self.authority['schedule'])
        attempts=stage_receipts(history,context)
        attempts.append(dict(context=context,proof=identity(dict(history=history,layer=proposal['layer'])),
            stage='GrowthRedistribution',status='inapplicable',reason='No accepted immediate compressed layer exists; redistribution changes only future heights'))
        source=geometry.Source(self.spec);index=proposal['layer']-1
        low=proposal['fronts'][index-1];target=self.schedule.targets[index]
        _,before_levels=self.geometry(state)
        advancing=np.flatnonzero((proposal['levels']>=index+1)&(before_levels<index+1))
        for stage in ('BranchAwareCorrection','ContinuationAttempt'):
            record=dict(stage=stage,passed=False)
            try:
                high=proposal['fronts'][index].copy()
                if stage=='BranchAwareCorrection':
                    high[advancing]=g.q.predict_next_front(low[advancing],target,source,iteration_cap=256).coordinates
                else:
                    from branch_aware_projection_study import continue_front
                    result=continue_front(low[advancing],self.schedule.targets[index-1],target,source,self.spec,
                        source.evaluate(self.x[advancing]).boundary_normal_selection,owner_provider=lambda p:source.evaluate(p).boundary_normal_selection)
                    high[advancing]=result.coordinates;record['trace']=result.trace
                fronts=list(proposal['fronts']);fronts[index]=high
                _,_,_,checked,_=g.validation(self.x,self.tri,fronts,proposal['levels'],False)
                checked=self.candidate_quality(checked,fronts,proposal['levels'])
                candidate=self.pack(fronts,proposal['levels'],state)
                changed=committed_mutations(state,candidate)
                if changed:
                    checked['quality']['failures'] += [dict(kind='committed_cell_mutation',id=int(k),column=int(state['cells'][k]['source'])) for k in changed]
                    checked['quality']['passed']=False;checked['passed']=False
                if checked['quality']['passed']:checked=self.certify_candidate(fronts,proposal['levels'],checked)
                assigned=geometry.sdf_record(self.spec,high[advancing],target)
                record.update(passed=checked['passed'] and assigned['admitted'],check=checked,sdf=assigned,
                    owners_before=source.evaluate(low[advancing]).boundary_normal_selection.tolist(),owners_after=source.evaluate(high[advancing]).boundary_normal_selection.tolist())
                if record['passed']:
                    proposal.update(fronts=fronts,check=checked,state=candidate)
                    evidence=super().evidence(state,proposal);evidence['accepted']['stage']=stage
                    evidence['attempts']=attempts+[dict(context=context,proof=identity(g.s.r.w.clean(record)),stage=stage,status='succeeded')]
                    g.save(self.folder/f'ESCALATION_{proposal["layer"]}.json',record)
                    return evidence
            except (ValueError,RuntimeError) as exc:record['error']=str(exc)
            attempts.append(dict(context=context,proof=identity(g.s.r.w.clean(record)),stage=stage,status='failed'))
            g.save(self.folder/f'{stage}_{proposal["layer"]}.json',record)
        vertices,columns=g.failure_vertices(self.tri,proposal['check']['quality'])
        vertices.update(source_failure_seed(source,proposal,target))
        ref=dict(context=context,proof=validation_digest(proposal['check']))
        seed=dict(vertices=sorted(vertices),columns=columns,cells=[],faces=[])
        return dict(context=context,support=dict(ref,status='clean'),immediate=dict(ref,status='invalid',seed=seed),
            warnings=[ref['proof']],witnesses=[],accepted=None,attempts=attempts)


def seal_design():
    path=OUT/'GEOMETRY_FAMILY_MANIFEST.json'
    specs=geometry.definitions()
    record=dict(version=geometry.VERSION,designs=specs,
        schedule_targets=g.q.load_schedule().targets.tolist(),schedule_hash=g.q.array_hash(g.q.load_schedule().targets),
        source_sha256=file_hash(Path(geometry.__file__)),
        scope='Eight distinct constructions; parameter variants are not extra geometry classes',
        sign='Negative in material, positive in extrusion domain; cavity reverses the enclosed primitive sign',
        extraction='Deterministic radial ray bisection or periodic analytic torus chart; fixed source vertex and triangle IDs')
    if path.exists():
        if json.loads(path.read_text())['designs']!=specs:raise ValueError('FrozenDesignChanged')
        return
    g.save(path,record)


def run(name,replay,steps=None,resume=False,*,spec=None,output=None,provider_factory=None,observer=None):
    if spec is None:
        seal_design();spec=next(s for s in geometry.cohort()+geometry.multi_region_definitions() if s['name']==name)
    if spec['name']!=name:raise ValueError('CaseAuthorityNameMismatch')
    folder=Path(output or OUT)/name/replay;folder.mkdir(parents=True,exist_ok=True)
    if resume and (folder/'FAILURE.json').exists():
        import shutil
        failure=folder/'FAILURE.json';shutil.copy2(failure,folder/('FAILURE_'+file_hash(failure)+'.json'))
    start=time.perf_counter();p=(provider_factory or AnalyticProvider)(spec,folder);runner=MixedFrontRunner(folder/'checkpoints',p)
    import mixed_front_fixture_provider as provider_module
    from sdf_cfd_mesher_v4.certificate.continuous_prism_certificate import kernel
    sources=[Path(__file__),Path(geometry.__file__),Path(provider_module.__file__),Path(g.__file__),Path(kernel.__file__),kernel.ROOT/'root_representation.py']
    implementation={str(path.relative_to(ROOT)):file_hash(path) for path in sources}
    g.save(folder/('EXECUTION_SOURCE_'+identity(implementation)+'.json'),implementation)
    trace=[]; operations=[]; initial=None
    try:
        if resume:
            initial=runner.store.restart()
            if (folder/'OPERATIONS.json').exists():operations=json.loads((folder/'OPERATIONS.json').read_text())
            if (folder/'TRACE.json').exists():trace=json.loads((folder/'TRACE.json').read_text())
        else:
            if (folder/'checkpoints/CURRENT.json').exists():raise ValueError('FreshRunRequiresUnusedOutput')
            # Keep the native disk-backed result, not the expanded proof
            # population used while constructing the first layer.
            initial=runner.initialize(p.initialize())
        g.save(folder/'GEOMETRY_AUTHORITY.json',dict(spec=spec,coordinate_hash=g.q.array_hash(p.x),
            topology_hash=g.q.array_hash(p.tri),sdf_identity=p.authority['sdf'],first_layer_hash=identity(initial['first_layer']),
            wall_identity=initial['wall']['identity'] if 'identity' in initial['wall'] else identity(initial['wall']),
            schedule=p.schedule.targets.tolist(),source_sha256=file_hash(Path(geometry.__file__))))
        if observer is not None:observer('initial_checkpoint',initial,p)
        # Final preservation checks need only these two immutable authorities.
        pending=bool(remaining_columns(initial))
        initial=dict(wall=initial['wall'],first_layer=initial['first_layer'])
        index=0
        while pending and (steps is None or index<steps):
            index+=1
            tick=time.perf_counter();operation=runner.step();operations.append(operation)
            state=runner.store.restart();layer=max(c['layer'] for c in state['cells'].values())
            row=dict(layer=layer,operation=operation['operation'],seconds=time.perf_counter()-tick,
                rss=psutil.Process().memory_info().rss,cells=len(state['cells']),state=scientific_hash(state),stages=dict(p.times))
            trace.append(row);g.save(folder/'TRACE.json',trace);g.save(folder/'OPERATIONS.json',operations)
            print(name,replay,layer,operation['operation'],len(state['cells']),flush=True)
            # Provider scratch is scoped to a candidate, not a scientific history.
            p.meshes.clear();p.checked.clear();p.numerical.clear()
            p.previous_certificate_state=None
            p.correction_base=None
            if observer is not None:observer('retained_after_commit',state,p)
            pending=bool(remaining_columns(state))
            del state

        state=runner.store.restart();restored=runner.store.restart()
        result=dict(case=name,scope='native production shell',cells=dict(Counter(c['kind'] for c in state['cells'].values())),
            columns=dict(Counter(c['status'] for c in state['columns'].values())),requested_layers=20,
            current_layer=max(c['layer'] for c in state['cells'].values()),
            local_layers={k:state['columns'][k]['last_prism_layer'] for k in state['columns']},
            all_active_schedules_complete=not remaining_columns(state),
            final=scientific_hash(state),restart_exact=scientific_hash(state)==scientific_hash(restored),
            wall_exact=state['wall']==initial['wall'],first_layer_exact=state['first_layer']==initial['first_layer'],
            remesh_count=len(state['events']),remesh_events_executed_in_directory=sum(r['operation']=='RollbackRemesh' for r in operations),interface=export(state,folder/'interface'),
            numerical='NotEvaluatedDueToMissingAuthoritativeCaseControls')
        g.save(folder/'SHELL_RESULT.json',result)
    except Exception as exc:
        g.save(folder/'FAILURE.json',dict(reason=type(exc).__name__+':'+str(exc),traceback=traceback.format_exc(),
            committed_operations=len(operations),classification='UnclassifiedPendingInvestigation'))
        print(name,replay,'FAILED',str(exc),flush=True)
        raise
    finally:
        mem=psutil.Process().memory_info()
        g.save(folder/'PERFORMANCE.json',dict(seconds=time.perf_counter()-start,peak_rss_bytes=getattr(mem,'peak_wset',mem.rss)))


if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('case',nargs='?');ap.add_argument('--replay',default='A');ap.add_argument('--steps',type=int);ap.add_argument('--design-only',action='store_true');ap.add_argument('--resume',action='store_true')
    a=ap.parse_args()
    if a.design_only:seal_design()
    else:run(a.case,a.replay,a.steps,a.resume)

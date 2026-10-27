"""Actual qualified preparation of retained L6, without front publication.

Reconstruct the qualified index; derive receipt inputs through the original
parent views. Only access-root metadata may be rebound to its authenticated
recovery reference. The original science, gates and old files stay unchanged.
"""
import argparse
from contextlib import ExitStack
from pathlib import Path
import shutil
import sys
import time

V4=Path(__file__).resolve().parents[1];ROOT=V4.parents[1]
sys.path[:0]=[str(V4/'tools'),str(V4/'src')]
import benchmark_retained_l6 as control
from path_preserving_deduplication import durable_json,file_hash
from sdf_cfd_mesher_v4.common.authority import identity

OUT=V4/'results/persistent_index_renewal'
RUN=OUT/'actual_prepare'
COMPACT=V4/'results/compact_reuse_l6/optimized'


def runtime():
    pins=control.read(V4/'results/guarded_publication_scope/REVIEW_REQUEST.json')['runtime_sources']
    names=set(pins)|{str(V4/'tools'/n) for n in
        ('benchmark_actual_prepare.py','preparation_reference.py','accepted_production_contract.py',
         'accepted_trajectory_contract.py','check_sf2_continuation.py','sealed_front_connection.py',
         'run_live_sf2_surface.py','prism_proof_index.py','prism_persistent_hotpath.py',
         'replay_surface_dispatch.py','canonical_spline_authority.py','sf2_surface_policy.py',
         'surface_binding.py','production_surface_authority.py','critical_index_pipeline.py',
         'persistent_numeric_incidence.py','prism_source_lookup.py','composable_admission.py',
         'production_readonly_admission.py','production_forecast_admission.py','optimized_admission.py',
         'recovered_parent_authorization.py','check_recovered_authorization.py',
         'persistent_index_renewal.py','check_persistent_index_renewal.py',
         'prism_source_lookup.py','incremental_pair_storage.py')}
    names.add(str(V4/'src/sdf_cfd_mesher_v4/sdf/canonical.py'))
    names.update(str(V4/'tools'/n) for n in ('recursive_production_publication.py','run_retained_front_publication.py'))
    return {p:file_hash(p) for p in sorted(names)}


def request(*,publish=False):
    p=OUT/'ACTUAL_REVIEW_REQUEST.json'
    if p.exists():raise ValueError('PreserveActualReviewRequest')
    source=control.SOURCE;cert=control.read(source/'FRESH_CERTIFICATE.json')
    names=('ADMISSION.json','GENERATION.json','ASSEMBLY.json','GEOMETRY_DELTA.json',
           'CHANGED_INPUTS.json','AFFECTED_INCIDENCE.json','VOLUME_DELTA.json','GENERATED_NEXT.npz',
           'PERSISTENT_GEOMETRY.json','BINDING_PATCH.json','FRESH_CERTIFICATE.json')
    paths=[source/n for n in names]+[source/'objects'/cert['segment'][k]['file'] for k in ('faces','pairs','codes','payloads')]
    paths += [COMPACT/n for n in ('QUALITY_DELTA.json','NEW_ATLAS.json')]+[source/'NEW_FORECAST_DEPENDENCIES.json']
    recovery=control.read(V4/'results/component_epoch_recovery/RECOVERY.json')['record']
    paths += [V4/'results/component_epoch_recovery/RECOVERY.json',
              V4/'results/component_epoch_recovery/access/CURRENT.json',
              V4/'results/component_epoch_recovery/access'/('RECOVERY_'+identity(recovery)+'.json')]
    binary=ROOT/'target/debug/canonical_kernel_host.exe'
    replay={}
    if publish:
        for n in ('REPLAY.json','replay/CANDIDATE_PLAN.json'):
            path=source/n;replay[str(path)]=file_hash(path)
    durable_json(p,dict(runtime=runtime(),inputs={str(p):file_hash(p) for p in paths},replay=replay,
        action='retained_publication' if publish else 'preparation_only',
        binary={str(binary):file_hash(binary)},
        recovery_identity=identity(recovery),
        staging_bytes=sum(p.stat().st_size for p in paths if p.is_relative_to(source) or p.parent==COMPACT),baseline=file_hash(OUT/'BASELINE.json'),
        scope=('Retained L6 replay authentication, qualified preparation/index reconstruction, recursive publication and guarded reopen; no new geometry/proofs or L7 advancement.' if publish else
               'Actual critical_prepare body, cold receipt derivation and qualified index reconstruction; no assembly, cells/proofs, transition, acceptance, recursive publication or front advancement.'),
        access_rebinding='Require original/new transition equality except base_root, recovered.source == original plan.parent, original admission binding == live adapter.binding, recovered content/scientific unchanged.',
        resources=dict(output_cap=None,minimum_free_bytes=45120503439)))


def validate_replay(primary,replay,record):
    """Exact existing component comparison, not a new scientific certificate."""
    if (identity(primary['content'])!=primary['scientific'] or identity(replay['content'])!=replay['scientific']
        or primary['content']!=replay['content'] or primary['scientific']!=replay['scientific']
        or record.get('passed') is not True or record['content']!=primary['scientific']
        or record['replay_content']!=replay['scientific']
        or set(record['comparisons'])!=set(primary['components'])
        or any(v is not True for v in record['comparisons'].values())):
        raise ValueError('RetainedScientificReplayMismatch')


def worker(*,publish=False):
    import os
    import numpy as np
    import psutil
    import immutable_state_access as access
    from recursive_epoch_recovery import discover
    from recursive_replay_retention import authenticated_closure
    from check_sf2_continuation import existing_state_class,exact_vertex_map,STUDY,PRIOR
    from accepted_trajectory_contract import AcceptedTrajectoryOwner
    from accepted_production_contract import bind_production,plan_transition,validate_authorization
    from recovered_parent_authorization import recovered_authorization
    from persistent_index_renewal import IndexRenewal,dependency_union
    from sealed_front_connection import load_case,qualified_dispatch
    from run_live_sf2_surface import CanonicalKernel,bind_canonical,bind_sf2_policy,verify_kernel_sources
    from prism_persistent_hotpath import geometry_session
    from critical_index_pipeline import IndexJob,JOBS
    from prism_proof_index import write_proof_index
    from guarded_publication_inputs import scope
    import critical_prepare
    started=time.perf_counter();stage='review';phases={}
    def timed(name,call):
        nonlocal stage
        stage=name;durable_json(RUN/'STAGE.json',dict(stage=stage,seconds=time.perf_counter()-started))
        print(name,flush=True);tick=time.perf_counter();v=call();phases[name]=time.perf_counter()-tick
        durable_json(RUN/'PHASES.json',phases);return v
    try:
        with ExitStack() as held:
            for p in (OUT/'ACTUAL_REVIEW_REQUEST.json',OUT/'ACTUAL_REVIEW.json',OUT/'BASELINE.json'):
                held.enter_context(access.guard(p))
            request=control.read(OUT/'ACTUAL_REVIEW_REQUEST.json');review=control.read(OUT/'ACTUAL_REVIEW.json')
            if request.get('action')!=('retained_publication' if publish else 'preparation_only'):
                raise ValueError('WrongReviewedPreparationAction')
            if (review.get('passed') is not True or not review.get('reviewer') or
                review['request_sha256']!=file_hash(OUT/'ACTUAL_REVIEW_REQUEST.json') or
                review['runtime']!=runtime() or request['runtime']!=runtime() or
                request['baseline']!=file_hash(OUT/'BASELINE.json')):raise ValueError('UnreviewedActualPreparation')
            for p,h in request['runtime'].items():
                held.enter_context(access.guard(p))
                if file_hash(p)!=h:raise ValueError('ChangedActualRuntime',p)
            for p,h in request['binary'].items():
                held.enter_context(access.guard(p))
                if file_hash(p)!=h:raise ValueError('ChangedCanonicalBinary',p)
            control.OUT=OUT
            old_plan,accepted,auth=timed('authenticate',lambda:control.guarded_inputs(held))
            recovered=discover(V4/'results/component_epoch_recovery/access')
            base,_,_=held.enter_context(authenticated_closure(recovered['reference']))
            if recovered['source']!=old_plan['parent'] or base.content_root!=old_plan['content']['parent_content']:
                raise ValueError('WrongAuthenticatedAccessRebinding')
            for p,h in request['inputs'].items():
                held.enter_context(access.guard(p))
                if file_hash(p)!=h:raise ValueError('ChangedActualInput',p)
            if publish:
                for p,h in request['replay'].items():
                    held.enter_context(access.guard(p))
                    if file_hash(p)!=h:raise ValueError('ChangedRetainedReplayEvidence',p)
                replay_plan=control.read(control.SOURCE/'replay/CANDIDATE_PLAN.json')
                validate_replay(old_plan,replay_plan,control.read(control.SOURCE/'REPLAY.json'))
                def authenticate_replay():
                    reused=0;hashed=0
                    for p,r in replay_plan['references'].items():
                        held.enter_context(access.guard(p))
                        if access.token(p)!=r['token']:raise ValueError('ChangedReplayReference',p)
                        if old_plan['references'].get(p)==r:reused+=1
                        else:
                            if file_hash(p)!=r['sha256']:raise ValueError('ChangedReplayBytes',p)
                            hashed+=Path(p).stat().st_size
                    # Replay is independently prepared/compared, not published:
                    # it has no transaction acceptance. Require the actual
                    # qualification inputs authenticated by its prepared plan.
                    rp=control.SOURCE/'replay'
                    for n in ('ADMISSION.json','GENERATION.json','ASSEMBLY.json','VOLUME_DELTA.json','FRESH_CERTIFICATE.json'):
                        if str((rp/n).resolve()) not in replay_plan['references']:
                            raise ValueError('UnboundReplayQualificationEvidence',n)
                    generation=control.read(rp/'GENERATION.json');certificate=control.read(rp/'FRESH_CERTIFICATE.json')
                    volume=control.read(rp/'VOLUME_DELTA.json')
                    if (generation['hard_failures'] or not generation['topology_audit']['passed']
                        or not certificate['passed'] or certificate['failures'] or not volume['exact_balance']
                        or not volume['global_degree']['passed']):raise ValueError('UnacceptedReplayScientificEvidence')
                    return dict(passed=True,comparisons=control.read(control.SOURCE/'REPLAY.json'),
                        references=len(replay_plan['references']),reused=reused,new_bytes_hashed=hashed)
                replay_evidence=timed('authenticate_retained_replay',authenticate_replay)
                durable_json(RUN/'REPLAY_AUTHENTICATION.json',replay_evidence)
            def staging():
                for p,h in request['inputs'].items():
                    source=Path(p)
                    if not (source.is_relative_to(control.SOURCE) or source.parent==COMPACT):continue
                    relative=source.relative_to(control.SOURCE) if source.is_relative_to(control.SOURCE) else Path(source.name)
                    dest=RUN/relative;dest.parent.mkdir(parents=True,exist_ok=True)
                    shutil.copyfile(source,dest)
                    if file_hash(dest)!=h:raise ValueError('ChangedStagedInput',p)
            case=control.read(PRIOR/'CASES.json')['wing'];wall,tri,surface,request1=load_case(case)
            diagnostic=control.read(V4/'results/sf2_continuation_binding/target_check/DIAGNOSTIC_REQUEST.json')
            auth_record=control.read(V4/'results/wing_continuation_layers6to8/AUTHORIZATION.json')
            state,selected=existing_state_class().load(STUDY/'planner/selected_state')
            mapping=exact_vertex_map(state.wall,wall)
            with np.load(V4/'results/live_sf2_first_layer/A/CANDIDATE.npz',allow_pickle=False) as z:
                first,refs=z['front'],z['face_refs']
            kernel,_=verify_kernel_sources()
            with CanonicalKernel([str(ROOT/'target/debug/canonical_kernel_host.exe')],timeout=60) as host:
                obj=host.load_rhai((V4/'results/smooth_wing_recovery/recovered_sf2.rhai').read_text(encoding='utf-8'),1.)
                if obj.identity!='079606dc36582ed38e51e5485e1f853e247a2ae40b544414c9e649552ae85311':
                    raise ValueError('WrongActualGeometry')
                authority=bind_sf2_policy(bind_canonical(obj)).authority
                original=AcceptedTrajectoryOwner(authority,diagnostic,identity(diagnostic),wall,first,tri,refs,mapping,selected,parent=case['committed'])
                validate_authorization(auth_record,identity(auth_record),original)
                with recovered_authorization(base,V4/'results/component_epoch_recovery/access',auth_record,request['recovery_identity']) as access_proof, qualified_dispatch():
                    adapter,_=timed('live_binding',lambda:bind_production(base,case,original,auth_record,identity(auth_record)))
                    transition=plan_transition(base,adapter);g=control.read(control.SOURCE/'GENERATION.json')
                    prior=g['transition_plan']
                    differences={k:[prior.get(k),transition.get(k)] for k in set(prior)|set(transition) if prior.get(k)!=transition.get(k)}
                    if set(differences)!={'base_root'}:raise ValueError('UnexpectedAccessRebinding',differences)
                    if control.read(control.SOURCE/'ADMISSION.json')['evidence']['binding']!=adapter.binding:
                        raise ValueError('ChangedActualAdmissionBinding')
                    durable_json(RUN/'ACCESS_REBIND.json',dict(differences=differences,source=request['inputs'][str(control.SOURCE/'GENERATION.json')],
                        authenticated_source=recovered['source'],scientific=base.scientific,content=base.content_root,
                        authorization_access=access_proof,authorization_access_identity=identity(access_proof)))
                    os.environ.update(V4_VERIFIED_QUALITY='1',V4_GUARDED_PUBLICATION='1')
                    with ExitStack() as geometry_held:
                        records=dependency_union(old_plan['references'],recovered['files'])
                        renewal=geometry_held.enter_context(IndexRenewal(RUN/'indices',records,identity(records)))
                        renewed=timed('persistent_index_renewal',lambda:renewal.geometry(old_plan['persistent_geometry']))
                        if old_plan['persistent_geometry']['parent']!=base.body['plan']['persistent_geometry']:
                            raise ValueError('WrongRetainedGeometryParent')
                        assembly=control.read(control.SOURCE/'ASSEMBLY.json')
                        accounting=assembly['pair_accounting']
                        pairs=timed('pair_index_renewal',lambda:renewal.pairs(
                            dict(path=accounting['index_manifest'],root=accounting['index_root']),base.content_root))
                        from incremental_pair_storage import PairIncidence
                        with PairIncidence(base,pairs['path'],pairs['root']) as pair_view:
                            if pair_view.body['parent_count']!=accounting['parent_count']:
                                raise ValueError('RenewedPairCountMismatch')
                        base.numeric_incidence=renewed['parent']
                        geometry_held.callback(lambda:delattr(base,'numeric_incidence'))
                        timed('persistent_geometry_open',lambda:geometry_held.enter_context(geometry_session(base,adapter)))
                        timed('staging',staging)
                        # Access metadata only. Original scientific arrays and reports stay untouched.
                        accounting.update(index_manifest=pairs['path'],index_root=pairs['root'])
                        durable_json(RUN/'ASSEMBLY.json',assembly)
                        durable_json(RUN/'PERSISTENT_GEOMETRY.json',renewed)
                        durable_json(RUN/'INDEX_ACCESS_RENEWAL.json',renewal.evidence())
                        g['transition_plan']=transition;durable_json(RUN/'GENERATION.json',g)
                        job=IndexJob(base,RUN,write_proof_index)
                        JOBS[str(RUN)]=job
                        try:
                            expected_index=old_plan['components']['certificate']['lookup']
                            def checked_prepare(parent,owner,out):
                                if parent is not base or owner is not adapter or out!=RUN:raise ValueError('WrongRetainedPublicationOwner')
                                plan=critical_prepare.prepare(parent,owner,out)
                                if file_hash(job.path)!=expected_index:raise ValueError('RebuiltIndexMismatch')
                                if plan['content']!=old_plan['content'] or plan['scientific']!=old_plan['scientific']:
                                    raise ValueError('ActualPreparationScientificMismatch')
                                for n in ('NEW_RECEIPT_INPUTS.json','CANDIDATE_HOT.json'):
                                    if file_hash(RUN/n)!=old_plan['references'][str((control.SOURCE/n).resolve())]['sha256']:
                                        raise ValueError('ActualDerivedFileMismatch',n)
                                return plan
                            if publish:
                                from unittest.mock import patch
                                import composable_publication as publication
                                from recursive_production_publication import publish as recursive_publish
                                from composable_state import CommittedState
                                with PairIncidence(base,pairs['path'],pairs['root']) as pair_view,patch.object(publication,'prepare',checked_prepare):
                                    reference=timed('recursive_publication',lambda:recursive_publish(base,adapter,RUN,pair_view))
                                plan=control.read(RUN/'CANDIDATE_PLAN.json')
                                def reopen():
                                    with CommittedState(reference) as child:
                                        child.check()
                                        if child.layer!=6 or child.scientific!=old_plan['scientific'] or child.components!=old_plan['components']:
                                            raise ValueError('RetainedPublicationReopenMismatch')
                                        with PairIncidence(child,plan['pair_incidence_manifest'],plan['pair_incidence_root']) as incidence:
                                            if incidence.body['parent_count']!=job.cert['combined_pairs']:raise ValueError('IncompleteReopenedPairCoverage')
                                        return dict(passed=True,reference=reference,scientific=child.scientific,layer=child.layer,
                                            quality_count=len(child['quality']),proof_count=len(child['certificate']),
                                            pair_count=incidence.body['parent_count'])
                                reopened=timed('guarded_reopen',reopen)
                                durable_json(RUN/'REOPEN.json',reopened)
                            else:
                                def prepare():
                                    with scope(RUN):return checked_prepare(base,adapter,RUN)
                                plan=timed('actual_prepare',prepare)
                            durable_json(RUN/'RESULT.json',dict(passed=True,scientific=plan['scientific'],parent=base.scientific,
                                exact_content_equal=True,index_sha256=expected_index,index_rows=job.cert['required_pairs'],
                                phases=phases,kernel=kernel,geometry=obj.identity,authentication=auth,
                                preparation=control.read(RUN/'PREPARE_METRICS.json'),index_timing=job.timing,
                                seconds=time.perf_counter()-started,peak_working_set=psutil.Process().memory_info().peak_wset,
                                runtime=request['runtime'],no_geometry_generated=True,no_proofs_recomputed=True,no_front_published=not publish,
                                publication=None if not publish else reopened))
                        finally:
                            JOBS.pop(str(RUN),None);job.close()
    except BaseException as error:
        import traceback
        durable_json(RUN/'FAILURE.json',dict(stage=stage,error=repr(error),traceback=traceback.format_exc(),phases=phases))
        raise


def monitor(*,command=None):
    import finish_retained_l5 as m
    RUN.mkdir(exist_ok=False)
    m.OUT=OUT/'actual_monitor';m.OUT.mkdir(exist_ok=False)
    return m.monitor(worker_command=command or [sys.executable,'-B',__file__,'worker'],
        extra_storage_roots=[control.SOURCE.parent.parent,V4/'results/current_front_transition_round_trip',RUN],
        enforce_output_cap=False)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('mode',choices=['request','worker','monitor']);a=p.parse_args()
    if a.mode=='monitor':sys.exit(monitor())
    else:globals()[a.mode]()

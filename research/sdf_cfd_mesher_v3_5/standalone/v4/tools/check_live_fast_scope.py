"""Read-only contract preflight for the existing ProductionFast executor.

Retained coordinates are qualification fixtures, never generated output.
No mesh, proof, request or committed state is changed by this checker.
"""
import argparse
import json
from pathlib import Path
import subprocess
import sys
import time

V4=Path(__file__).resolve().parents[1]
ROOT=V4.parents[1]
sys.path[:0]=[str(V4/'tools'),str(V4/'src')]
from path_preserving_deduplication import durable_json,file_hash
from record_receipt_resume import disk,status

OUT=V4/'results/live_wing_fast_scope'


def read(path):return json.loads(Path(path).read_text(encoding='utf-8'))


def baseline():
    if OUT.exists():raise ValueError('PreserveFastScopeEvidence')
    prior=read(V4/'results/fresh_successor_benchmark/BASELINE.json')
    own=['research/sdf_cfd_mesher_v4/tools/check_live_fast_scope.py',
         'research/sdf_cfd_mesher_v4/CURRENT_TASK.md']
    current=[r for r in status() if r[3:] not in own]
    if current!=prior['unrelated']:raise ValueError('ChangedUnrelatedStatus')
    for group in ('protected','inputs'):
        if any(file_hash(p)!=h for p,h in prior[group].items()):raise ValueError('ChangedProtectedInput',group)
    OUT.mkdir()
    durable_json(OUT/'BASELINE.json',dict(start=time.time(),head=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),
        branch=subprocess.check_output(['git','branch','--show-current'],text=True).strip(),
        unrelated=current,protected=prior['protected'],inputs=prior['inputs'],disk=disk()))


def quality_scope():
    """Use actual retained L7 points and the unchanged fast quality consumer."""
    import numpy as np
    import psutil
    import local_termination_topology as t
    from persistent_numeric_incidence import open_geometry,CellSequence,advance
    from composable_state import CommittedState
    from sealed_front_connection import qualified_dispatch
    from run_authorized_continuation import recovered_execution_access
    start=time.perf_counter()
    control=read(V4/'results/fresh_successor_benchmark/CONTROL.json')
    folder=V4/'results/fresh_successor_benchmark/current_parent_run/L7'
    path=folder/'GENERATED_NEXT.npz'
    # Verify the actual generation file against the retained task manifest.
    generation=read(folder/'GENERATION.json')
    from sdf_cfd_mesher_v4.common.authority import array_hash
    with np.load(path,allow_pickle=False) as z:
        high=z['coordinates'];tri=z['topology'];ids=z['new_connectivity']
    if array_hash(high)!=generation['coordinates'] or array_hash(tri)!=generation['topology'] or array_hash(ids)!=generation['new_connectivity']:
        raise ValueError('ChangedRetainedGenerationFixture')
    with qualified_dispatch(),CommittedState(control['parent']['reference']) as base:
        with recovered_execution_access(base,control,control['authorization_record']):
            if base.layer!=6 or base.scientific!=control['parent']['scientific']:raise ValueError('WrongFastScopeParent')
            # Recover submitted points in stable source order from authenticated state.
            n=len(high)
            points=np.empty((8*n,3))
            wall=base['wall']['vertices']
            for v,p in wall.items():points[int(v.split(':')[-1])]=p
            for v,p in base['vertices'].items():
                _,layer,_,source=v.split(':');points[int(layer)*n+int(source)]=p
            points[7*n:]=high
            reference=base.plan['persistent_geometry']
            with open_geometry(reference,base.content_root) as packed:
                old=CellSequence(packed)
                added=[t.cell(row,i,7) for i,row in enumerate(ids)]
                outer=[tuple(map(int,f+7*n)) for f in tri]
                owners,topology=advance(packed,added,outer,[tuple(reversed(f)) for f in tri])
                changed={c['id'] for c in added}
                local={tuple(sorted(f)):owners[tuple(sorted(f))] for c in added for f in c['faces']}
                needed={i for rows in local.values() for i,_ in rows}
                selected=old.selected(needed-changed)+added
                # Exact function used by Quality.update; same active-cell scope.
                result=t.quality(points,selected,local,1,active_cells=changed)
                failures=result['failures']
                from collections import Counter
                durable_json(OUT/'FAST_QUALITY_SCOPE.json',dict(passed=result['passed'],
                    fixture_not_generation=True,layer=7,parent=base.scientific,source=file_hash(Path(t.__file__)),
                    executor=file_hash(V4/'tools/incremental_layer_validation.py'),
                    checked_cells=len(result['cells']),checked_internal_faces=len(result['faces']),
                    failures_by_kind=dict(Counter(r['kind'] for r in failures)),
                    failure_count=len(failures),first_failures=failures[:10],
                    minimum_determinant=min(r['determinant'] for r in result['cells']),
                    maximum_aspect=max(r['aspect'] for r in result['cells']),
                    nonpositive_exact=sum(r['volume_sign']<=0 or r['jacobian_sign']<=0 for r in result['exact']),
                    governing_requirement='ProductionFast Quality.update uses t.quality internal/coupled-face tensor with the exposed outer face excluded; determinant >= 0.0011.',
                    retained_qualified_contract=read(folder/'AFFECTED_INCIDENCE.json')['scope'],
                    fixture_sha256=file_hash(path),seconds=time.perf_counter()-start,
                    peak_main_working_set=psutil.Process().memory_info().peak_wset))
    print('FAST QUALITY SCOPE',result['passed'],len(failures),flush=True)


def prepare_review():
    from run_successor_benchmark import execution_sources,execution_binaries,retained_execution_inputs
    names=['production_fast_layer.py','accepted_fast_binding.py','check_live_fast_scope.py',
           'incremental_layer_validation.py','benchmark_production_fast.py']
    inputs=retained_execution_inputs(read(V4/'results/fresh_successor_benchmark/REVIEW_REQUEST.json')['inputs'])
    for path in [V4/'results/fresh_successor_benchmark/CONTROL.json',
                 V4/'results/fresh_successor_benchmark/current_parent_run/L7/REOPEN.json',
                 V4/'results/wing_receipt_resume/A/START.json',
                 OUT/'FAST_QUALITY_SCOPE.json']:
        inputs[str(path)]=file_hash(path)
    reference=V4/'results/fresh_successor_benchmark/current_parent_run/L7'
    for name in ('GENERATED_NEXT.npz','NEW_QUALITY.json','FRESH_CERTIFICATE.json','GENERATION.json'):
        inputs[str(reference/name)]=file_hash(reference/name)
    durable_json(OUT/'REVIEW_REQUEST.json',dict(runtime=execution_sources([V4/'tools'/n for n in names]),
        binary=execution_binaries(),inputs=inputs,baseline=file_hash(OUT/'BASELINE.json'),
        scope='One fresh ProductionFast L6->L7 transition. No L9, no publication. Bootstrap and independent qualification outside hot-path timer.'))


def worker(*,warm=False):
    """Configure and call the existing FastLayer, then independently inspect it."""
    from contextlib import ExitStack
    import traceback
    import numpy as np
    import psutil
    import immutable_state_access as access
    from sdf_cfd_mesher_v4.common.authority import identity,array_hash
    from run_authorized_continuation import recovered_execution_access
    from sealed_front_connection import load_case,qualified_dispatch
    from composable_state import CommittedState
    from accepted_production_contract import bind_production
    from accepted_trajectory_contract import AcceptedTrajectoryOwner
    from accepted_fast_binding import AcceptedFastProvider
    from check_sf2_continuation import existing_state_class,exact_vertex_map,STUDY,PRIOR
    from run_live_sf2_surface import CanonicalKernel,bind_canonical,bind_sf2_policy,verify_kernel_sources
    from production_fast_layer import FastLayer
    from native_proof_segments import LoadedSegment
    from optimized_mixed_certificate import certify
    from audit_capsule_scaling import final_coverage
    from benchmark_production_fast import seal_degree
    from incremental_layer_validation import t,key
    from final_shell_incidence import check
    runpath=OUT/'run'
    if runpath.exists():raise ValueError('PreserveFastBenchmarkRun')
    q=read(OUT/'REVIEW_REQUEST.json');review=read(OUT/'REVIEW.json')
    if not review.get('passed') or review['request_sha256']!=file_hash(OUT/'REVIEW_REQUEST.json'):
        raise ValueError('IndependentFastReviewRequired')
    started=time.perf_counter();stage='preflight'
    runpath.mkdir()
    def save(name,value):durable_json(runpath/name,value)
    try:
        with ExitStack() as held:
            for group in ('runtime','binary','inputs'):
                for p,h in q[group].items():
                    held.enter_context(access.guard(p))
                    if file_hash(p)!=h:raise ValueError('ChangedFastExecutionDependency',p)
            if file_hash(OUT/'BASELINE.json')!=q['baseline']:raise ValueError('ChangedFastBaseline')
            control=read(V4/'results/fresh_successor_benchmark/CONTROL.json')
            case=read(PRIOR/'CASES.json')['wing'];wall,tri,surface,request=load_case(case)
            diagnostic=read(V4/'results/sf2_continuation_binding/target_check/DIAGNOSTIC_REQUEST.json')
            historical,selected=existing_state_class().load(STUDY/'planner/selected_state')
            mapping=exact_vertex_map(historical.wall,wall)
            with np.load(V4/'results/live_sf2_first_layer/A/CANDIDATE.npz',allow_pickle=False) as z:
                first,refs=z['front'],z['face_refs']
            kernel,_=verify_kernel_sources()
            host=held.enter_context(CanonicalKernel([str(ROOT/'target/debug/canonical_kernel_host.exe')],timeout=60))
            obj=host.load_rhai((V4/'results/smooth_wing_recovery/recovered_sf2.rhai').read_text(encoding='utf-8'),1.)
            authority=bind_sf2_policy(bind_canonical(obj)).authority
            original=AcceptedTrajectoryOwner(authority,diagnostic,identity(diagnostic),wall,first,tri,refs,mapping,selected,parent=case['committed'])
            held.enter_context(qualified_dispatch())
            base=held.enter_context(CommittedState(control['parent']['reference']))
            held.enter_context(recovered_execution_access(base,control,control['authorization_record']))
            if base.layer!=6 or base.scientific!=control['parent']['scientific']:raise ValueError('WrongFastParent')
            stage='binding'
            tick=time.perf_counter()
            adapter,_=bind_production(base,case,original,control['authorization_record'],control['authorization'])
            provider=AcceptedFastProvider(base,adapter,runpath,warm=warm)
            save('BINDING.json',dict(provider.binding,kernel=kernel,canonical=obj.identity,
                approved_increment=adapter.schedule.row_record(7),generation_from_live_owner=True,
                seconds=time.perf_counter()-tick))
            stage='bootstrap';tick=time.perf_counter();print(stage,flush=True)
            state=provider.materialize()
            run=FastLayer(state,provider.spec,runpath,bound_provider=provider)
            save('BOOTSTRAP.json',dict(seconds=time.perf_counter()-tick,peak_main_working_set=psutil.Process().memory_info().peak_wset,
                cells=len(run.topology.cells),not_hot_path=True))
            print('bootstrap complete',flush=True)
            stage='ProductionFast';result=run.step()
            save('PRODUCTION_L7.json' if warm else 'PRODUCTION.json',dict(result,peak_main_working_set=psutil.Process().memory_info().peak_wset))
            if warm:
                from check_warm_fast_scope import bounds
                save('L7_BOUNDS.json',bounds(run))
                # Compare the first transition before using its private parent.
                # No fixture is used in generation. All executor gates passed.
                from check_warm_fast_scope import compare_components,compare_fresh_proofs,require_reference
                ref7=V4/'results/fresh_successor_benchmark/current_parent_run/L7'
                accepted7=held.enter_context(CommittedState(read(ref7/'REOPEN.json')['reference']))
                require_reference(accepted7,ref7,7)
                components=compare_components(run,accepted7)
                save('L7_COMPONENT_COMPARISON.json',components)
                # The provider is now rebound for L8, but the L7 dependencies
                # were written using the original sealed-L6 target owner.
                if not all(r['equal'] for r in components.values()):raise ValueError('FastFirstTransitionMismatch')
                save('L7_PROOF_COMPARISON.json',compare_fresh_proofs(run,accepted7,ref7))
                save('L7_PRIVATE_PARENT.json',provider.private_receipt)
                print('warm L7 complete',result['seconds'],flush=True)
                stage='ProductionFastL8';result=run.step()
                save('PRODUCTION_L8.json',dict(result,peak_main_working_set=psutil.Process().memory_info().peak_wset))
                save('L8_BOUNDS.json',bounds(run))
                save('L8_PRIVATE_PARENT.json',provider.private_receipt)
            layer=8 if warm else 7
            # Retain actual fresh generated coordinates, not a loaded reference mesh.
            np.savez_compressed(runpath/'GENERATED_NEXT.npz',coordinates=run.points[-len(wall):],topology=tri)
            print('production complete',result['seconds'],flush=True)
            stage='independent_qualification';tick=time.perf_counter()
            outcomes={}
            reference=V4/f'results/fresh_successor_benchmark/current_parent_run/L{layer}'
            with np.load(reference/'GENERATED_NEXT.npz',allow_pickle=False) as z:
                expected=z['coordinates']
            outcomes['new_coordinates_bit_equal']=bool(run.points[-len(wall):].tobytes()==expected.tobytes())
            if not outcomes['new_coordinates_bit_equal']:raise ValueError('FastDeclaredCoordinateMismatch')
            outcomes['global_degree']=seal_degree(run)
            cells=list(run.topology.cells.values())
            owners,ownership=t.ownership(cells,run.outer,run.wall)
            links=t.vertex_links(run.outer)
            if not ownership['passed'] or not links['passed'] or owners!=run.topology.owners:
                raise ValueError('FastIndependentOwnershipFailure')
            outcomes.update(ownership=ownership,vertex_links=links,ownership_cache_exact=True)
            full=t.quality(run.points,cells,owners,1)
            if not full['passed']:raise ValueError('FastIndependentQualityFailure')
            outcomes['raw_quality_exact']=(run.quality.cells=={r['id']:r for r in full['cells']} and
                run.quality.exact=={r['id']:r for r in full['exact']} and
                run.quality.faces=={key(r['vertices']):{k:v for k,v in r.items() if k!='vertices'} for r in full['faces']})
            if not outcomes['raw_quality_exact']:raise ValueError('FastIndependentQualityMismatch')
            prospective=check(run.points,cells,run.outer)
            if not prospective['passed'] or run.quality.prospective!={r['id']:r for r in prospective['cells']}:
                raise ValueError('FastIndependentIncidenceMismatch')
            outcomes['prospective_exact']=True
            old=read(reference/'NEW_QUALITY.json')
            newids={c['id'] for c in cells if c['layer']==layer}
            facekeys={key(f) for c in cells if c['id'] in newids for f in c['faces'] if len(owners[key(f)])==2}
            if (any(len(old[name])!=len(newids) or {r['id'] for r in old[name]}!=newids for name in ('cells','exact'))
                or len(old['faces'])!=len(facekeys) or {key(r['vertices']) for r in old['faces']}!=facekeys):
                raise ValueError('IncompleteRetainedQualityInventory')
            # Bind diagnostic rows back to the qualified immutable receipt owner.
            # This is independent qualification work, never hot-path input.
            from critical_quality_lookup import selected as selected_receipts
            from sdf_cfd_mesher_v4.advancing_front.mixed_front import face_key
            accepted=held.enter_context(CommittedState(read(reference/'REOPEN.json')['reference']))
            if accepted.layer!=layer or accepted.scientific!=read(reference/'REOPEN.json')['scientific']:raise ValueError('WrongQualifiedReference')
            accepted.verify(reference/'FRESH_CERTIFICATE.json');accepted.verify(reference/'GENERATION.json')
            from sdf_cfd_mesher_v4.advancing_front.native_checkpoint import scientific_digest
            if warm:
                from check_warm_fast_scope import compare_components
                outcomes['scientific_components']=compare_components(run,accepted)
            else:
                outcomes['scientific_components']={}
                for name in ('wall','first_layer','vertices','cells','faces','columns','outer','prism_front','versions','atlas','forecast','authority','events'):
                    actual=scientific_digest(run.tx.current[name]);expected=scientific_digest(accepted[name])
                    outcomes['scientific_components'][name]=dict(actual=actual,reference=expected,equal=actual==expected)
            save('COMPONENT_COMPARISON.json',outcomes['scientific_components'])
            if not all(row['equal'] or (warm and row.get('dependency_meaning_exact')) for row in outcomes['scientific_components'].values()):
                raise ValueError('FastScientificComponentMismatch')
            def verify_rows(rows,kind):
                for start in range(0,len(rows),512):
                    batch=rows[start:start+512]
                    keys=[('cell:'+str(r['id']) if kind in ('cells','exact') else 'face:'+face_key(tuple(
                        f'layer:{v//len(wall)}:source:{v%len(wall)}' for v in r['vertices']))) for r in batch]
                    saved=selected_receipts(accepted,keys)
                    for r,k in zip(batch,keys):
                        expected=saved[k]['exact'] if kind=='exact' else saved[k]['value']
                        actual=r if kind!='faces' else {k:v for k,v in r.items() if k!='vertices'}
                        if actual!=expected:raise ValueError('RetainedQualityNotQualified:'+kind)
            for kind in ('cells','exact','faces'):verify_rows(old[kind],kind)
            # The historical cell receipt selects the declared internal top role.
            outcomes['retained_exact_cells_equal']=all(run.quality.exact[r['id']]==r for r in old['exact'])
            outcomes['retained_cell_geometry_equal']=all(all(run.quality.cells[r['id']][k]==r[k] for k in r if k!='determinant') for r in old['cells'])
            oldfaces={key(r['vertices']):{k:v for k,v in r.items() if k!='vertices'} for r in old['faces']}
            outcomes['retained_changed_faces_equal']=all(run.quality.faces[k]==v for k,v in oldfaces.items())
            outcomes['retained_interface_determinants_equal']=all(run.quality.prospective[r['id']]['determinant']==r['determinant'] for r in old['cells'])
            if not all(outcomes[k] for k in ('retained_exact_cells_equal','retained_cell_geometry_equal','retained_changed_faces_equal','retained_interface_determinants_equal')):
                raise ValueError('FastRetainedQualityMismatch')
            del old,oldfaces,full,prospective
            faces=[owners[k][0][1] for k in sorted(owners)]
            outcomes['coverage']=final_coverage(run.points,faces,run.proofs)
            fresh=read(reference/'FRESH_CERTIFICATE.json')
            # Exact compact rows, with identical global face numbering, not sampled equality.
            descriptor=fresh['segment'];indices={k:i for i,k in enumerate(run.face_keys)}
            for row in __import__('native_proof_segments').segment_objects(descriptor):
                accepted.verify(reference/'objects'/row['file'])
                if file_hash(reference/'objects'/row['file'])!=row['sha256']:raise ValueError('ChangedReferenceProofSegment')
            expected=LoadedSegment(descriptor,reference/'objects',indices,True)
            actual=run.proofs.fresh
            outcomes['fresh_pairs_bit_equal']=bool(np.asarray(expected.mapping)[np.asarray(expected.pairs)].tobytes()==np.frombuffer(actual.pairs,dtype=np.uint64).reshape(-1,2).tobytes())
            outcomes['fresh_codes_bit_equal']=bool(np.asarray(expected.codes).tobytes()==np.frombuffer(actual.codes,dtype=np.uint64).tobytes())
            outcomes['fresh_payloads_equal']=expected.payloads==actual.payloads
            if not all(outcomes[k] for k in ('fresh_pairs_bit_equal','fresh_codes_bit_equal','fresh_payloads_equal')):raise ValueError('FastFreshProofMismatch')
            outcomes.update(passed=True,seconds=time.perf_counter()-tick,
                scope=f'Independent full geometry quality/incidence/degree and complete AABB relation coverage; exact retained L{layer} coordinates/new quality and proof comparison. No publication or generalized fast restart.',
                inherited_relations=sum(s.descriptor['count'] for s in run.proofs.segments),fresh_relations=len(actual),
                forbidden=0,unresolved=0,
                extrema=dict(volume=min(r['volume'] for r in run.quality.cells.values()),
                    raw_determinant=min(r['determinant'] for r in run.quality.cells.values()),
                    prospective_determinant=min(r['determinant'] for r in run.quality.prospective.values()),
                    aspect=max(r['aspect'] for r in run.quality.cells.values()),
                    weight=min(r['weight'] for r in run.quality.faces.values()),
                    skewness=max(r['skewness'] for r in run.quality.faces.values()),
                    symmetric_volume_ratio=min(r['symmetric_volume_ratio'] for r in run.quality.faces.values())))
            save('QUALIFICATION.json',outcomes)
            print('independent qualification complete',outcomes['seconds'],flush=True)
            provider.check()
            save('RESULT.json',dict(passed=True,production_seconds=result['seconds'],qualification_seconds=outcomes['seconds'],
                total_seconds=time.perf_counter()-started,peak_main_working_set=psutil.Process().memory_info().peak_wset,
                unsealed=True,publication=False,layer=layer,warm=warm,geometry=obj.identity,coordinates=array_hash(run.points[-len(wall):])))
    except BaseException as exc:
        save('STOP.json',dict(stage=stage,error=type(exc).__name__+':'+str(exc),traceback=traceback.format_exc(),
            seconds=time.perf_counter()-started,peak_main_working_set=psutil.Process().memory_info().peak_wset))
        raise


def monitor(*,worker_command=None):
    """Reuse the qualified operational monitor; do not add a storage system."""
    import finish_retained_l5 as m
    import immutable_state_access as access
    from contextlib import ExitStack
    review=read(OUT/'REVIEW.json')
    if review.get('passed') is not True or review['request_sha256']!=file_hash(OUT/'REVIEW_REQUEST.json'):
        raise ValueError('IndependentFastReviewRequired')
    request=read(OUT/'REVIEW_REQUEST.json')
    resource=V4/'results/wing_receipt_resume/A/START.json'
    with ExitStack() as held:
        for path in (OUT/'REVIEW_REQUEST.json',OUT/'REVIEW.json',resource):held.enter_context(access.guard(path))
        if file_hash(resource)!=request['inputs'][str(resource)]:raise ValueError('ChangedFastResourcePolicy')
        m.OUT=OUT/'monitor';m.OUT.mkdir(exist_ok=False)
        return m.monitor(worker_command=worker_command or [sys.executable,'-B',__file__,'worker'],
                         enforce_output_cap=False,extra_storage_roots=[OUT/'run'])


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('mode',choices=['baseline','quality','review','worker','monitor']);a=p.parse_args()
    result=globals()[{'baseline':'baseline','quality':'quality_scope','review':'prepare_review','worker':'worker','monitor':'monitor'}[a.mode]]()
    if a.mode=='monitor':sys.exit(result)

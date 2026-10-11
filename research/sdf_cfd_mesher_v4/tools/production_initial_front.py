"""Production initialization connection; existing generator/proofs/native owner.

One fresh realization, all required gates and independent proof coverage.
The qualification profile retains the separate A/B extended replay workflow.
"""
from pathlib import Path
import json,sys
V4=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(V4/'tools'),str(V4/'src')]
sys.dont_write_bytecode=True
import numpy as np
from path_preserving_deduplication import durable_json,file_hash
from sdf_cfd_mesher_v4.common.authority import identity
from sdf_cfd_mesher_v4.advancing_front.mixed_front import rebuild_dependencies,scientific_hash

def commit_and_enroll(provider,result,pre,*,bound_certificate=None,resume_record=None,retained_candidate=None):
    import run_live_sf2_first_layer as runner
    import production_first_layer_coverage as coverage
    from first_layer_metric_state import receipt,VERSION
    from sealed_front_connection import publish_and_enroll_initial
    if not (result['passed'] and provider.check['passed'] and provider.assigned['admitted']):
        raise ValueError('FailedFirstLayerCommit')
    if retained_candidate is not None:
        # Reuse only the exact hash-bound, independently checked population.
        # A truthy caller argument is never an independent-coverage credential.
        saved=runner.read(runner.OUT/'RESUME_INPUTS.json')
        if (saved['source']!=retained_candidate or saved['geometry_generated'] is not False
            or saved['proofs_generated'] is not False
            or file_hash(Path(retained_candidate['directory'])/'A/COVERAGE_RECHECK.json')
                !=retained_candidate['files']['COVERAGE_RECHECK.json']
            or file_hash(provider.folder/'COVERAGE_RECHECK.json')!=retained_candidate['files']['COVERAGE_RECHECK.json']
            or result['certificate_hash']!=provider.check['certificate']['proof_hash']):
            raise ValueError('RetainedInitialCoverageCredentialMismatch')
    if resume_record is None and retained_candidate is None:
        original=coverage.OUT
        try:
            coverage.OUT=runner.OUT
            provider.phase('independent_coverage',lambda:coverage.run('A'))
        finally:coverage.OUT=original
    cov=runner.read(provider.folder/'COVERAGE_RECHECK.json')
    if any(cov[k]!=0 for k in ('forbidden','unresolved','missing','duplicate')):
        raise ValueError('FirstLayerCoverageFailure')
    if cov['candidates']!=provider.check['certificate']['required_pairs'] or cov['certified']!=cov['candidates']:
        raise ValueError('FirstLayerCoveragePopulationMismatch')
    state=provider.phase('initial_state_pack',lambda:provider.pack(
        [provider.high],np.ones(len(provider.x),int)))
    if bound_certificate is None:
        provider.phase('initial_receipt_binding',lambda:provider.bind(state,provider.check))
    else:
        from validation_row_stream import normalized_validation
        from sdf_cfd_mesher_v4.advancing_front.mixed_evidence import attach_validation
        attach_validation(state,provider.meshes[identity(state['cells'])],normalized_validation(provider.check),
            bound_certificate=bound_certificate)
    provider.phase('initial_dependencies',lambda:rebuild_dependencies(None,state,initial_schedule(provider)))
    state['quality']['first_prism_incidence']=provider.phase('initial_incidence_receipt',lambda:
        receipt(provider.points,provider.cells,provider.outer,state['versions']))
    from compact_initial_quality import bind_state
    provider.phase('initial_quality_storage',lambda:bind_state(state,provider.folder/'checkpoints/objects'))
    from compact_initial_atlas import bind_state as bind_atlas,install_reader as install_atlas_reader
    provider.phase('initial_atlas_storage',lambda:bind_atlas(state,provider.folder/'checkpoints/objects'))
    install_atlas_reader()
    committed=provider.phase('initial_scientific_identity',lambda:scientific_hash(state))
    if resume_record is not None and committed!=resume_record['committed_front_identity']:
        raise ValueError('RetainedInitialScientificMismatch')
    snapshot=dict(result,committed=committed,execution_profile='production',extended_replay_performed=False)
    durable_json(provider.folder/'INITIAL_RESULT.json',snapshot)
    scientific=dict(surface=pre['surface']['surface_identity'],request=pre['request_identity'],metric_semantics=VERSION,
        **{k:snapshot[k] for k in ('front_coordinate_hash','front_topology_hash','prism_connectivity_hash','ownership_hash','quality_hash','certificate_hash','committed')})
    implementation=dict(pre['dependencies']);implementation.update(pre['certificate_binary'])
    root=runner.ROOT
    for name in ('production_initial_front.py','compact_initial_quality.py','compact_initial_atlas.py','sealed_front_connection.py','production_first_layer_coverage.py',
                 'production_prism_stack.py','live_spline_first_layer.py','optimized_mixed_certificate.py',
                 'optimized_certificate_native.rs','prism_degree.py'):
        p=Path(__file__).with_name(name);implementation[p.relative_to(root).as_posix()]=file_hash(p)
    from sdf_cfd_mesher_v4.advancing_front import native_checkpoint
    for p in (Path(native_checkpoint.__file__),Path(__file__).with_name('compact_front_atlas.py'),
              Path(__file__).with_name('critical_prism_install.py')):
        implementation[p.relative_to(root).as_posix()]=file_hash(p)
    durable_json(runner.OUT/'IMPLEMENTATION.json',implementation)
    evidence=dict(profile='production',realizations=1,extended_replay_performed=False,
        required_checks='Fresh generation, hard quality, ownership, continuous proofs, volume/degree, independent complete coverage, native independent readback and guarded reopen',
        files={n:file_hash(provider.folder/n) for n in ('CANDIDATE.npz','INITIAL_RESULT.json','COVERAGE_RECHECK.json','QUALITY.json','TRAJECTORY.json','TOPOLOGY.json','VOLUME.json','CERTIFICATE_SUMMARY.json','CERTIFICATE.jsonl')})
    durable_json(runner.OUT/'PRODUCTION_CHECKS.json',evidence)
    seal=dict(classification='V4ProductionInitialCandidateQualified',first_layer_identity=identity(scientific),
        committed_front_identity=committed,scientific=scientific,first_layer_qualified=False,candidate_geometry_qualified=True,
        publication_complete=False,geometry=pre['surface']['geometry_identity'],
        realizations=1,extended_replay_performed=False,implementation_sha256=file_hash(runner.OUT/'IMPLEMENTATION.json'),
        request_sha256=file_hash(runner.REQUEST),surface_implementation_sha256=file_hash(runner.SURFACE/'IMPLEMENTATION.json'),
        production_checks_sha256=file_hash(runner.OUT/'PRODUCTION_CHECKS.json'),
        native_publisher='NativeCheckpointStore.publish(validate) independent readback -> existing enrollment/guarded reopen',
        top_role='ProspectiveFirstPrismDeterminant; complete_neighbor_metrics=false; not permanent terminal cap')
    durable_json(runner.OUT/'INITIAL_QUALIFICATION.json',seal)
    paths=[runner.OUT/n for n in ('REQUEST.json','INITIAL_QUALIFICATION.json','IMPLEMENTATION.json','PRODUCTION_CHECKS.json','PREFLIGHT_A.json')]
    if resume_record is not None or retained_candidate is not None:paths.append(runner.OUT/'RESUME_INPUTS.json')
    paths += [provider.folder/n for n in evidence['files']]
    paths += [(runner.OUT/r['path']).resolve() for r in runner.read(runner.REQUEST)['source_documents']]
    paths += [runner.SURFACE/n for n in ('SURFACE_AUTHORITY.json','SURFACE.npz','REFERENCES.npz','IMPLEMENTATION.json','REPLAY.json')]
    case=dict(units='m',surface=str(runner.SURFACE),surface_identity=scientific['surface'],geometry_identity=seal['geometry'],
        # bind() stages immutable receipt/index objects under checkpoints/objects.
        # Native publication needs a DISTINCT fresh store, then imports those
        # authenticated objects using its unchanged immutable import path.
        request=str(runner.REQUEST),request_sha256=file_hash(runner.REQUEST),checkpoint=str(provider.folder/'published_checkpoints'),
        committed=committed,layer=1,front_coordinates=snapshot['front_coordinate_hash'],front_qualification=str(runner.OUT/'INITIAL_QUALIFICATION.json'),
        sources={str(p.resolve()):file_hash(p) for p in paths})
    if hasattr(provider,'spec') and hasattr(provider,'binding'):
        geometry_path=runner.OUT/'GEOMETRY.json'
        durable_json(geometry_path,dict(geometry_identity=seal['geometry'],definition=provider.binding.authority.definition))
        case['geometry_definition']=str(geometry_path.resolve())
        case['sources'][str(geometry_path.resolve())]=file_hash(geometry_path)
    reference=provider.phase('initial_publication_enrollment',lambda:publish_and_enroll_initial(
        case,runner.OUT/'enrollment',state,provider.validate_initial_commit))
    # Only successful publication, enrollment AND guarded reopen permit this
    # committed authority. The guarded candidate record remains immutable.
    durable_json(runner.OUT/'AUTHORITY.json',dict(seal,classification='V4ProductionInitialFirstLayerCommitted',
        first_layer_qualified=True,publication_complete=True,reference=reference,
        initial_qualification_sha256=file_hash(runner.OUT/'INITIAL_QUALIFICATION.json')))
    durable_json(runner.OUT/'INITIAL_ENROLLMENT.json',dict(case=case,reference=reference,independent_readback_reused=True,
        extra_native_restart=False,scientific=committed,profile='production'))
    return committed

def initial_schedule(provider):
    # The explicitly bound analytic request owns cumulative scalar targets.
    # Historical and smooth first-layer requests retain their one-step scope.
    if hasattr(provider,'spec') and hasattr(provider,'binding'):
        return provider.schedule.targets
    return [provider.h1]


def copy_guarded_evidence(held,source,target,pin):
    import shutil
    import immutable_state_access as access
    shutil.copyfile(source,target)
    held.enter_context(access.guard(target))
    if file_hash(target)!=pin:raise ValueError('CopiedRetainedInitialEvidenceChanged',target)


def resume_initial(source):
    """Rebind retained candidate evidence; no trajectory/proof regeneration."""
    from contextlib import ExitStack
    import shutil,time,psutil
    import immutable_state_access as access
    import run_live_sf2_first_layer as runner
    import generalize_local_remesh as g
    from native_proof_segments import LoadedSegment,SegmentProofRows
    from compact_proof_rows import CompactProofRows
    from sdf_cfd_mesher_v4.advancing_front.native_checkpoint import ReceiptTable
    from sdf_cfd_mesher_v4.advancing_front.mixed_front import geometry_inputs,face_key
    source=Path(source).resolve();start=time.perf_counter()
    if source==runner.OUT or not source.is_relative_to(V4/'results'):
        raise ValueError('SeparateRetainedInitialRequired')
    with ExitStack() as held:
        for name in ('INITIAL_QUALIFICATION.json','PRODUCTION_CHECKS.json','IMPLEMENTATION.json',
                     'REQUEST.json','PREFLIGHT_A.json','A/STOP.json','A/RAW_QUALITY.json'):
            held.enter_context(access.guard(source/name))
        seal=runner.read(source/'INITIAL_QUALIFICATION.json')
        if (seal.get('classification')!='V4ProductionInitialCandidateQualified'
            or seal.get('candidate_geometry_qualified') is not True or seal.get('publication_complete') is not False
            or seal.get('first_layer_qualified') is not False
            or identity(seal['scientific'])!=seal['first_layer_identity']
            or runner.read(source/'A/STOP.json')['error']!='FreshInitialPublicationRequiresEmptyStore'):
            raise ValueError('UnsupportedRetainedInitialFailure')
        for name,key in [('PRODUCTION_CHECKS.json','production_checks_sha256'),('IMPLEMENTATION.json','implementation_sha256'),('REQUEST.json','request_sha256')]:
            if file_hash(source/name)!=seal[key]:raise ValueError('RetainedInitialEvidenceMismatch',name)
        if file_hash(runner.REQUEST)!=seal['request_sha256']:raise ValueError('RetainedInitialRequestChanged')
        for path,pin in runner.read(source/'IMPLEMENTATION.json').items():
            # This intentional path-binding repair is pinned by the guarded
            # shared-tool review. Numerical/source owners remain unchanged.
            if path==(V4/'tools/production_initial_front.py').relative_to(runner.ROOT).as_posix():continue
            held.enter_context(access.guard(runner.ROOT/path))
            if file_hash(runner.ROOT/path)!=pin:raise ValueError('ChangedRetainedInitialDependency',path)
        checks=runner.read(source/'PRODUCTION_CHECKS.json')
        for name,pin in checks['files'].items():
            held.enter_context(access.guard(source/'A'/name))
            if file_hash(source/'A'/name)!=pin:raise ValueError('ChangedRetainedInitialScientificFile',name)
        pre=runner.preflight()
        with runner.CanonicalKernel([str(runner.ROOT/'target/debug/canonical_kernel_host.exe')],timeout=60) as host:
            obj=host.load_rhai((V4/'results/smooth_wing_recovery/recovered_sf2.rhai').read_text(encoding='utf-8'),1.)
            binding=runner.bind_sf2_policy(runner.bind_canonical(obj))
            p=runner.SourceSheetFirstLayerProvider(runner.OUT/'A',runner.SURFACE,runner.REQUEST,binding)
            for name in checks['files']:
                if name=='INITIAL_RESULT.json':continue
                copy_guarded_evidence(held,source/'A'/name,p.folder/name,checks['files'][name])
            with np.load(p.folder/'CANDIDATE.npz',allow_pickle=False) as z:
                if z['wall'].tobytes()!=p.x.tobytes() or z['triangles'].tobytes()!=p.tri.tobytes():
                    raise ValueError('RetainedInitialWallMismatch')
                p.high=z['front'];p.direction=z['direction']
                if (p.x+p.h1*p.direction).tobytes()!=p.high.tobytes():raise ValueError('RetainedInitialTrajectoryMismatch')
                if any(z[k].tobytes()!=v.tobytes() for k,v in p.refs.items()):raise ValueError('RetainedInitialReferenceMismatch')
            p.assigned=runner.read(p.folder/'TRAJECTORY.json')
            p.check=runner.read(source/'A/RAW_QUALITY.json')
            p.check['volume']=runner.read(p.folder/'VOLUME.json')
            certificate=runner.read(p.folder/'CERTIFICATE_SUMMARY.json')
            if (not certificate['passed'] or certificate['failures'] or not p.check['quality']['passed']
                or not p.check['ownership']['passed'] or not p.check['vertex_links']['passed']
                or not p.check['volume']['exact_volume_balance'] or not p.check['volume']['positive_cells']
                or not p.check['volume']['interior_degree']['passed']):raise ValueError('RetainedInitialGateFailure')
            p.points,p.cells,p.outer=g.s.construct(p.x,p.tri,[p.high],np.ones(len(p.x),int))
            state=p.pack([p.high],np.ones(len(p.x),int));n=len(p.x)
            owners,_=g.t.ownership(p.cells,p.outer,[tuple(reversed(f)) for f in p.tri])
            keys=[face_key(tuple(f'layer:{i//n}:source:{i%n}' for i in owners[k][0][1])) for k in sorted(owners)]
            objects=source/'A/checkpoints/objects';tables=list(objects.glob('*.sqlite'))
            if len(tables)!=1:raise ValueError('RetainedInitialTableInventory')
            descriptor=dict(count=certificate['required_pairs'],methods=certificate['methods'])
            for path in objects.iterdir():
                held.enter_context(access.guard(path))
                row=dict(file=path.name,sha256=file_hash(path))
                if path.suffix=='.npy':
                    value=np.load(path,mmap_mode='r',allow_pickle=False)
                    key='pairs' if value.ndim==2 else 'codes'
                elif path.suffix=='.json':
                    value=runner.read(path);key='faces' if value and isinstance(value[0],str) else 'payloads'
                elif path.suffix=='.sqlite':continue
                else:raise ValueError('UnexpectedRetainedInitialObject',path)
                if key in descriptor:raise ValueError('DuplicateRetainedInitialIndex',key)
                descriptor[key]=row
            descriptor['identity']=identity(descriptor)
            segment=LoadedSegment(descriptor,objects,{k:i for i,k in enumerate(keys)})
            bound=ReceiptTable(tables[0],certificate['required_pairs'],tables[0].stem)
            bound.qualified_context=identity(geometry_inputs(state));bound.proof_segments=[descriptor];bound.proof_objects=objects
            certificate['proofs']=SegmentProofRows([segment],CompactProofRows())
            p.check.update(certificate=certificate,passed=True)
            old_result=runner.read(source/'A/INITIAL_RESULT.json');p.times=dict(old_result['times'])
            durable_json(runner.OUT/'RESUME_INPUTS.json',dict(source=str(source),seal_sha256=file_hash(source/'INITIAL_QUALIFICATION.json'),
                scientific=seal['committed_front_identity'],files=checks['files'],table_sha256=file_hash(tables[0]),
                implementation_sha256=file_hash(source/'IMPLEMENTATION.json'),raw_quality_sha256=file_hash(source/'A/RAW_QUALITY.json'),
                segment=descriptor,generation_repeated=False,proof_generation_repeated=False))
            committed=p.phase('resume_publication',lambda:commit_and_enroll(p,old_result,pre,bound_certificate=bound,resume_record=seal))
            result=dict(old_result,committed=committed,times=p.times,resume_seconds=time.perf_counter()-start,
                peak_working_set_bytes=psutil.Process().memory_info().peak_wset)
            durable_json(p.folder/'RESULT.json',result)


def main():
    import argparse
    import run_live_sf2_first_layer as runner
    parser=argparse.ArgumentParser();parser.add_argument('--output',required=True,type=Path)
    parser.add_argument('--surface',required=True,type=Path);parser.add_argument('--request',required=True,type=Path)
    parser.add_argument('--surface-identity',required=True)
    parser.add_argument('--resume-initial',type=Path)
    args=parser.parse_args();runner.configure_inputs(args.output,args.surface,args.request,args.surface_identity)
    try:
        if args.resume_initial is None:
            from production_prism_stack import initial_front_dispatch
            with initial_front_dispatch() as work:
                runner.worker('A',True,commit_action=commit_and_enroll)
                durable_json(runner.OUT/'INITIAL_PRODUCTION_WORK.json',work)
        else:resume_initial(args.resume_initial)
    except Exception as error:
        import traceback
        durable_json(runner.OUT/'A/STOP.json',dict(error=str(error),traceback=traceback.format_exc(),no_retry=True));raise
    return 0
if __name__=='__main__':raise SystemExit(main())

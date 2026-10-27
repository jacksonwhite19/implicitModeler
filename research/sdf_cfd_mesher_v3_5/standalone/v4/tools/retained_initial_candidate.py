"""Hash-bound retained initial evidence, through the shared standalone stages.

This resumes storage/publication only. It never proposes coordinates or proofs.
The reviewed release must name the exact workflow-only source revalidation.
"""
from contextlib import ExitStack
from pathlib import Path
import json,os,time
import numpy as np
from path_preserving_deduplication import durable_json,file_hash
from standalone_case import read,load
from owned_runtime import V4,verify_release
from sdf_cfd_mesher_v4.common.authority import identity,array_hash
ROOT=V4.parents[1]
FILES=('CANDIDATE.npz','RESULT.json','DIAGNOSTICS.json','QUALITY.json','RAW_QUALITY.json',
       'TRAJECTORY.json','TOPOLOGY.json','VOLUME.json','CERTIFICATE_SUMMARY.json','CERTIFICATE.jsonl','COVERAGE_RECHECK.json')

def verify(manifest,case_path,held):
    import immutable_state_access as access
    held.enter_context(access.guard(manifest));body=read(manifest)
    if body.get('schema')!='V4RetainedInitialCandidate/1':raise ValueError('UnsupportedRetainedInitialCandidate')
    if body['case_sha256']!=file_hash(case_path):raise ValueError('RetainedInitialCaseMismatch')
    allowed=set(FILES)
    from owned_source_segments import FILES as SEGMENT_FILES,verify_retained
    if set(body['files']) not in (allowed,allowed|set(SEGMENT_FILES)):raise ValueError('RetainedInitialEvidenceCoverage')
    source=Path(body['directory']).resolve()
    surface=Path(body['surface']).resolve()
    required=(source/'REQUEST.json',source/'PREFLIGHT_A.json',Path(body['prior_release']),
              *(surface/n for n in ('SURFACE_AUTHORITY.json','SURFACE.npz','REFERENCES.npz','IMPLEMENTATION.json','REPLAY.json')))
    if any(str(p.resolve()) not in body['dependencies'] for p in required):
        raise ValueError('RetainedInitialDependencyCoverage')
    for name,pin in body['files'].items():
        held.enter_context(access.guard(source/'A'/name))
        if file_hash(source/'A'/name)!=pin:raise ValueError('RetainedInitialFileMismatch',name)
    for path,pin in body['dependencies'].items():
        held.enter_context(access.guard(path))
        if file_hash(path)!=pin:raise ValueError('RetainedInitialDependencyMismatch',path)
    pre=read(source/'PREFLIGHT_A.json')
    if body['implementation']!={**pre['dependencies'],**pre['certificate_binary']}:
        raise ValueError('RetainedInitialImplementationCoverage')
    release=verify_release();prior=read(body['prior_release'])
    if release.get('retained_initial_prior_release_sha256')!=file_hash(body['prior_release']):
        raise ValueError('UnreviewedRetainedInitialPredecessor')
    # The complete prior release remains the dependency closure. Only the exact
    # reviewed workflow/storage changes may differ; proof/geometry owners do not.
    prior_sources={str((V4/r['path']).relative_to(ROOT)).replace('\\','/'):r['sha256'] for r in prior['sources']}
    if any(path in prior_sources and pin!=prior_sources[path] for path,pin in body['implementation'].items()):
        raise ValueError('RetainedInitialPredecessorPinConflict')
    # Validate both independently. Retained data cannot shadow predecessor pins.
    for path,pin in [*prior_sources.items(),*body['implementation'].items()]:
        held.enter_context(access.guard(ROOT/path))
        actual=file_hash(ROOT/path)
        if actual!=pin:
            approved=release.get('retained_initial_revalidation',{})
            if approved.get(path)!={'previous':pin,'current':actual}:
                raise ValueError('RetainedInitialImplementationChanged',path)
    result=read(source/'A/RESULT.json');coverage=read(source/'A/COVERAGE_RECHECK.json')
    cert=read(source/'A/CERTIFICATE_SUMMARY.json');q=read(source/'A/RAW_QUALITY.json')
    topo=read(source/'A/TOPOLOGY.json');trajectory=read(source/'A/TRAJECTORY.json')
    volume=read(source/'A/VOLUME.json')
    extra=verify_retained(source/'A',source/'REQUEST.json',surface,expected_release=file_hash(body['prior_release']))
    if (extra is not None)!=(set(body['files'])==allowed|set(SEGMENT_FILES)):raise ValueError('RetainedOwnedSegmentCoverage')
    if (result.get('passed') is not True or result.get('committed') is not None or not trajectory['admitted']
        or not q['quality']['passed'] or q['quality']['failures'] or not topo['ownership']['passed']
        or not topo['vertex_links']['passed'] or not cert['passed'] or cert['failures']
        or not volume['exact_volume_balance'] or not volume['positive_cells'] or not volume['interior_degree']['passed']
        or any(coverage[k]!=0 for k in ('forbidden','unresolved','missing','duplicate'))
        or coverage['candidates']!=cert['required_pairs'] or coverage['certified']!=cert['required_pairs']
        or result['certificate_hash']!=cert['proof_hash'] or result['quality_hash']!=body['files']['QUALITY.json']):
        raise ValueError('RetainedInitialGateFailure')
    return body

def resume(output,case_path,manifest):
    import immutable_state_access as access
    import run_live_sf2_first_layer as runner
    from standalone_tool import geometry
    from standalone_geometry import is_scalar
    from production_first_layer_run import ProductionFirstLayerProvider
    from live_spline_first_layer import SourceSheetFirstLayerProvider
    from production_initial_front import commit_and_enroll
    from production_surface_authority import load_production_wall
    from production_prism_stack import initial_front_dispatch
    from compact_proof_rows import CompactProofRows
    from validation_row_stream import clean_rows_digest
    import generalize_local_remesh as g
    start=time.perf_counter()
    with ExitStack() as held:
        body=verify(manifest,case_path,held);source=Path(body['directory'])
        surface=Path(body['surface']);wall,tri,seal=load_production_wall(surface)
        if seal['surface_identity']!=body['surface_identity']:raise ValueError('RetainedInitialSurfaceMismatch')
        impl=read(surface/'IMPLEMENTATION.json');replay=read(surface/'REPLAY.json')
        if identity(impl)!=seal['implementation_identity'] or identity(replay)!=seal['determinism_identity']:
            raise ValueError('RetainedInitialSurfaceSidecarMismatch')
        changes=verify_release()['retained_initial_revalidation']
        for path,pin in {**impl['dependencies'],**impl['binaries']}.items():
            held.enter_context(access.guard(ROOT/path));actual=file_hash(ROOT/path)
            if actual!=pin and changes.get(path)!={'previous':pin,'current':actual}:
                raise ValueError('RetainedInitialSurfaceSourceMismatch',path)
        folder=output/'first_layer';folder.mkdir()
        request=folder/'REQUEST.json'
        # Byte-identical original request, including its exact source-document scope.
        os.link(source/'REQUEST.json',request);held.enter_context(access.guard(request))
        pre=read(source/'PREFLIGHT_A.json')
        from production_prism_sizing import inspect_first_layer_inputs,read_request
        admitted=inspect_first_layer_inputs(surface,request);req=read_request(request,seal)
        case,increments,case_pin=load(case_path)
        if (pre['surface']!=seal or pre['request_identity']!=identity(req['declaration'])
            or pre['request_sha256']!=file_hash(request) or pre['resources']!=case['resources']
            or float(req['increments'][0]).hex()!=float(increments[0]).hex()
            or not any(row['sha256']==case_pin for row in req['declaration']['source_documents'])
            or ('later_normal_increments_m_hex' in case['prisms'] and req['increments'].tobytes()!=increments.tobytes())
            or ('later_path_lengths_m_hex' in case['prisms'] and len(req['increments'])!=1)
            or admitted['generation_authorized'] is not True):raise ValueError('RetainedInitialInputMismatch')
        revalidated=dict(pre['dependencies'])
        for path in revalidated:
            if path in changes:revalidated[path]=changes[path]['current']
        pre=dict(pre,dependencies=revalidated,retained_source_preflight=file_hash(source/'PREFLIGHT_A.json'),
                 revalidation_identity=identity(verify_release().get('retained_initial_revalidation',{})))
        runner.configure_inputs(folder,surface,request,seal['surface_identity']);runner.STANDALONE_INPUTS=case_path
        durable_json(folder/'PREFLIGHT_A.json',pre)
        host,binding=geometry(case_path)
        try:
            if binding.geometry_identity!=seal['geometry_identity']:raise ValueError('RetainedInitialGeometryMismatch')
            p=(ProductionFirstLayerProvider(folder/'A',surface=surface,request=request,binding=binding)
               if is_scalar(binding) else SourceSheetFirstLayerProvider(folder/'A',surface,request,binding))
            # Immutable evidence only: no hardlink to outputs subsequently rewritten.
            for name in body['files']:
                if name=='RESULT.json':continue
                os.link(source/'A'/name,p.folder/name)
                held.enter_context(access.guard(p.folder/name))
            with np.load(p.folder/'CANDIDATE.npz',allow_pickle=False) as z:
                if z['wall'].tobytes()!=wall.tobytes() or z['triangles'].tobytes()!=tri.tobytes():
                    raise ValueError('RetainedInitialWallMismatch')
                p.high=z['front'];p.direction=z['direction']
                if (p.x+p.h1*p.direction).tobytes()!=p.high.tobytes():raise ValueError('RetainedInitialTrajectoryMismatch')
                if not is_scalar(binding) and any(z[k].tobytes()!=v.tobytes() for k,v in p.refs.items()):
                    raise ValueError('RetainedInitialReferencesMismatch')
            old=read(source/'A/RESULT.json')
            if array_hash(p.high)!=old['front_coordinate_hash'] or array_hash(p.tri)!=old['front_topology_hash']:
                raise ValueError('RetainedInitialArrayMismatch')
            p.assigned=read(p.folder/'TRAJECTORY.json');p.check=read(p.folder/'RAW_QUALITY.json')
            from owned_source_segments import verify_retained
            verify_retained(p.folder,p.request,p.surface)
            p.check['volume']=read(p.folder/'VOLUME.json');certificate=read(p.folder/'CERTIFICATE_SUMMARY.json')
            population=CompactProofRows();previous=None
            def original_rows():
                nonlocal previous
                with (p.folder/'CERTIFICATE.jsonl').open(encoding='utf-8') as stream:
                    for line in stream:
                        row=json.loads(line);pair=tuple(row['pair'])
                        if previous is not None and pair<=previous:raise ValueError('RetainedInitialProofOrder')
                        population.append(row);previous=pair;yield row
            if clean_rows_digest(original_rows())!=certificate['proof_hash'] or len(population)!=certificate['required_pairs']:
                raise ValueError('RetainedInitialProofIdentity')
            certificate['proofs']=population;p.check.update(certificate=certificate,passed=True)
            p.points,p.cells,p.outer=g.s.construct(p.x,p.tri,[p.high],np.ones(len(p.x),int))
            if (array_hash(np.asarray([c['vertices'] for c in p.cells],dtype=np.int64))!=old['prism_connectivity_hash']
                or identity([(c['id'],c['column']) for c in p.cells])!=old['ownership_hash']):
                raise ValueError('RetainedInitialCellIdentity')
            p.times=dict(old['times'])
            durable_json(folder/'RESUME_INPUTS.json',dict(manifest_sha256=file_hash(manifest),
                source=body,geometry_generated=False,proofs_generated=False,original_result_sha256=body['files']['RESULT.json']))
            with initial_front_dispatch() as work:
                committed=commit_and_enroll(p,old,pre,retained_candidate=body)
                durable_json(folder/'INITIAL_PRODUCTION_WORK.json',work)
            durable_json(p.folder/'RESULT.json',dict(old,committed=committed,times=p.times,
                resume_seconds=time.perf_counter()-start,geometry_generated=False,proofs_generated=False))
        finally:host.close()
        return read(folder/'INITIAL_ENROLLMENT.json'),p.folder/'CANDIDATE.npz'

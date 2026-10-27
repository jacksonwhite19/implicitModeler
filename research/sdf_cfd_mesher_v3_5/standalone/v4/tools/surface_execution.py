"""One retained orchestration, moved from the capsule module; stage algorithms imported."""
import stage_observations as obs
import dataclasses, json, re, shutil, time, sys
from contextlib import ExitStack
from pathlib import Path
import numpy as np
import psutil
from historical_capsule_surface import (
    OUT,V3,VERSION,save,sha,wsl,run_native,canonical,write_off,metric,
    identity,array_hash,provider,refine_metric_carriers,CarrierOraclePolicy,ShapeMetricPolicy)
from sdf_cfd_mesher_v4.sdf.canonical import GeometryFailure
from carrier_snapshot import write_terminal,require_supported_handoff


def validate_references(binding,vertices,faces,edges,stage):
    refs=binding.references
    problems=[]
    if set(f.sheet_ref for f in faces)!=set(refs['sheet_names']):problems.append('sheet coverage')
    if set(e.curve_ref for e in edges)!=set(refs['curve_names']):problems.append('curve coverage')
    if {v.corner_ref for v in vertices if v.corner_ref}!=set(refs['corner_names']):problems.append('corner coverage')
    if any(v.sheet_ref not in refs['sheet_names'] for v in vertices):problems.append('vertex sheet')
    if problems:
        raise GeometryFailure('ReferenceCoverageFailure',stage,','.join(problems),binding.geometry_identity,
            operation='persistent_reference_handoff',required_premise=refs)


def validate_output_references(binding,face_refs,edge_refs,corners):
    if set(face_refs)!=set(binding.references['sheet_names']):
        raise GeometryFailure('LostSourceSheetIdentity','mmgs_output','sheet coverage differs',binding.geometry_identity)
    # MMGS may include unlabelled topological edges (ref 0), never unknown semantic refs.
    if set(edge_refs)-{0}!=set(binding.references['curve_names']):
        raise GeometryFailure('LostSourceCurveIdentity','mmgs_output','curve coverage differs',binding.geometry_identity)


def write_surface_contract(binding, output, vertices, triangles, edges):
    """Actual pre-MMGS writer boundary, also usable without running a mesher."""
    output=Path(output);authority=binding.authority;policy=binding.sizing_policy
    coupled=binding.verify_metric_connection()
    start=time.perf_counter()
    metrics=None if coupled is not None else [metric(authority,v.point,policy,sheet_ref=v.sheet_ref) for v in vertices]
    sizing_seconds=time.perf_counter()-start;start=time.perf_counter()
    if coupled is not None:
        coupled.verify(binding)
        stats=coupled.write_contract(output/'metric.bin',vertices,triangles,edges)
        save(output/'WRITTEN_METRIC_POLICY.json',stats)
    else:
        # Historical capsule/polygon path retains its sealed metric semantics.
        stats=provider.write_contract(output/'metric.bin',vertices,triangles,edges,authority,1.,
            shape_policy=ShapeMetricPolicy(),metric_override=metrics)
    return dict(sizing_seconds=sizing_seconds,contract_seconds=time.perf_counter()-start),stats


@obs.observed('surface.execute','surface')
def execute_surface(binding, output):
    coupled=binding.verify_metric_connection()
    scoped=getattr(coupled,'query_scope',None)
    if callable(scoped):
        with scoped(output):return _execute_surface(binding,output)
    return _execute_surface(binding,output)


def _execute_surface(binding, output):
    authority=binding.authority
    policy=binding.sizing_policy
    coupled=binding.verify_metric_connection()
    output=Path(output);output.mkdir(parents=True,exist_ok=False)
    start=time.perf_counter();timing={};events=[]
    def event(stage, **kw):
        binding.check_storage(output)
        query_stats=getattr(coupled,'query_statistics',None)
        row=dict(stage=stage,elapsed=time.perf_counter()-start,rss=psutil.Process().memory_info().rss,
                 free_bytes=shutil.disk_usage(output).free,**kw)
        if callable(query_stats):row['source_queries']=query_stats()
        events.append(row);save(output/'PROGRESS.json',events);print(json.dumps(row),flush=True)
    with ExitStack() as construction:
        scoped=getattr(coupled,'construction_scope',None)
        if callable(scoped):construction.enter_context(scoped())
        event('fresh_stock_sdf_seed_start' if binding.seed_route=='CanonicalMarchingCubesSeed/1' else
            'fresh_implicit_surface_seed_start' if binding.seed_route=='CGALImplicitSurfaceExtraction/1' else
            'fresh_implicit_seed_start' if binding.seed_route=='CGALImplicitExtraction' else 'labelled_chart_seed_start')
        seed_data=binding.seed_provider(authority,policy,output)
        x,faces=seed_data['coordinates'],seed_data['triangles']
        vertices,triangles,edges=seed_data['vertices'],seed_data['faces'],seed_data['edges']
        timing['extraction']=seed_data['timing']
        physical_seed_size=seed_data['seed_size_m']
        validate_references(binding,vertices,triangles,edges,'seed_reference_handoff')
        normals=np.cross(x[faces[:,1]]-x[faces[:,0]],x[faces[:,2]]-x[faces[:,0]])
        t=time.perf_counter();centres=x[faces].mean(1)
        batch=getattr(authority,'construction_normals_batch',None)
        active=getattr(authority,'construction_active',lambda:False)()
        if active and callable(batch):
            proposals=batch(centres,np.asarray([face.sheet_ref for face in triangles],dtype=np.int64))
            failed=any(n@normal<=0 for n,normal in zip(normals,proposals))
        else:
            failed=any(n@authority.exact_normal(face.sheet_ref,c)<=0 for n,c,face in zip(normals,centres,triangles))
        if failed:raise ValueError('SeedOrientationFailure')
        timing['seed_orientation_check_seconds']=time.perf_counter()-t
        np.savez_compressed(output/'INITIAL.npz',coordinates=x,triangles=faces)
        carrier_switch=getattr(coupled,'carrier_assessment_enabled',None)
        enabled=carrier_switch() if callable(carrier_switch) else True
        if type(enabled) is not bool:raise ValueError('ExplicitCarrierConstructionRoleRequired')
        if enabled:
            event('adaptive_carrier_start',vertices=len(x),triangles=len(faces))
            t=time.perf_counter()
            carrier_options=getattr(coupled,'carrier_options',None)
            vertices,triangles,edges,carrier=refine_metric_carriers(authority,vertices,triangles,edges,
                coupled.carrier if coupled is not None else lambda p,ref:metric(authority,p,policy,True,sheet_ref=ref),
                coupled.carrier_policy() if callable(getattr(coupled,'carrier_policy',None)) else CarrierOraclePolicy(),
                progress=lambda row:event('carrier_pass',**row),
                evidence=lambda row:save(output/'CARRIER_FACE_DECISIONS.json',row),
                terminal_capture=lambda payload:write_terminal(output/'CARRIER_TERMINAL.npz',payload,
                    dict(geometry_identity=binding.geometry_identity,
                         metric_connection_source_identity=getattr(coupled,'source_identity',None),
                         authority_type=type(authority).__module__+'.'+type(authority).__name__,
                         authority_source_sha256=sha(Path(sys.modules[type(authority).__module__].__file__)),
                         metric_connection_type=None if coupled is None else type(coupled).__module__+'.'+type(coupled).__name__,
                         metric_connection_source_sha256=None if coupled is None else sha(Path(sys.modules[type(coupled).__module__].__file__)),
                         source_files={str(Path(refine_metric_carriers.__code__.co_filename).name):sha(Path(refine_metric_carriers.__code__.co_filename)),
                                       Path(__file__).name:sha(Path(__file__))},
                         scope='Normal held source/query/construction context; exact captured nodal values, not later reevaluation')),
                **(carrier_options() if callable(carrier_options) else {}))
            timing['carrier_seconds']=time.perf_counter()-t
        else:
            carrier=dict(enabled=False,status='NotRun',oracle_role='Optional construction proposal; final acceptance remains required',
                original_vertices=len(vertices),original_faces=len(triangles),added_vertices=0,added_faces=0,
                materialization_authorized=None)
            timing['carrier_seconds']=0.
            event('adaptive_carrier_omitted',vertices=len(x),triangles=len(faces))
        save(output/'CARRIER.json',carrier)
        if enabled and not carrier['materialization_authorized']:raise ValueError('CarrierAdequacyUnresolved')
        require_supported_handoff(carrier)
        metric_timing,_=write_surface_contract(binding,output,vertices,triangles,edges)
        timing.update(metric_timing)
        event('stock_mmgs_start',vertices=len(vertices),triangles=len(triangles))
        from owned_runtime import BIN, LIB, RUNTIME, verify as verify_runtime
        verify_runtime()
        runner=BIN/'mmgs_metric_contract_runner'
        lib=LIB
        dependency=json.loads((RUNTIME/'MMGS_DEPENDENCY_QUALIFICATION.json').read_text(encoding='utf-8'))
        if not dependency['pass_gate'] or sha(runner)!=dependency['adapter_sha256'] or sha(lib/'libmmgs.so.5.8.0')!=dependency['library_sha256']:
            raise ValueError('UnqualifiedMMGSDependency')
        timing['mmgs'],stdout=run_native(['env','LD_LIBRARY_PATH='+wsl(lib),wsl(runner),wsl(output/'metric.bin'),wsl(output/'raw.mesh')],output/'mmgs')
        raw,refs,faces,face_refs,edges,edge_refs,corners=provider.parse_medit(output/'raw.mesh')
        validate_output_references(binding,face_refs,edge_refs,corners)
        if {int(refs[i]) for i in corners} != set(binding.references['corner_names']):
            raise GeometryFailure('LostSourceCornerIdentity','mmgs_output','corner coverage differs',binding.geometry_identity)
        t=time.perf_counter()
        points,incident_faces,incident_curves,curve_edges=provider.canonicalize_and_project(authority,raw,faces,face_refs,edges,edge_refs,corners)
        if binding.corner_restorer is not None:
            binding.corner_restorer(points,incident_curves)
        points,faces,order,inverse,face_order=canonical(points,faces,return_permutations=True)
        face_refs=np.asarray(face_refs,dtype=int)[face_order]
        vertex_refs=np.array([min(incident_faces[i]) for i in order],dtype=int)
        curves=np.array([(inverse[a],inverse[b],ref) for a,b,ref in curve_edges],dtype=int).reshape(-1,3)
        timing['final_projection_seconds']=time.perf_counter()-t
        t=time.perf_counter()
        from historical_capsule_surface import SeedVertex
        evaluate_batch=getattr(coupled,'evaluate_batch',None)
        if callable(evaluate_batch):
            final_metric=evaluate_batch(points,vertex_refs,curves=[incident_curves[i] for i in order])
        else:
            final_metric=np.array([coupled.evaluate(p,int(ref),curves=incident_curves[i]) for p,ref,i in zip(points,vertex_refs,order)]) if coupled is not None else np.array([authority.metric(SeedVertex(p,int(ref),0,int(refs[i])),incident_curves[i],policy.metric_multiplier)
                if incident_curves[i] else metric(authority,p,policy,sheet_ref=ref)
                for p,ref,i in zip(points,vertex_refs,order)])
        timing['final_sizing_seconds']=time.perf_counter()-t
    assurance=getattr(binding.qualification,'assurance','qualification')
    if assurance=='numerical':
        # Numerical file payload is explicitly a construction value. The final
        # acceptance owner independently evaluates its actual final coordinates.
        with coupled.construction_scope(),authority.verified_operation('residual'):
            residual=np.array([authority.implicit_value(ref,p) for p,ref in zip(points,vertex_refs)])
    else:residual=np.array([authority.implicit_value(ref,p) for p,ref in zip(points,vertex_refs)])
    np.savez_compressed(output/'SURFACE.npz',coordinates=points,triangles=faces,metric=final_metric,residual=residual)
    corner_refs=np.array([int(refs[i]) if i in corners else 0 for i in order],int)
    np.savez_compressed(output/'REFERENCES.npz',face_refs=face_refs,vertex_refs=vertex_refs,curve_edges=curves,corner_refs=corner_refs)
    write_off(output/'surface.off',points,faces)
    operation_rows=[]
    for line in stdout.splitlines():
        if 'splitted' in line:
            row={k:int(v) for v,k in re.findall(r'(\d+)\s+(splitted|collapsed|swapped|moved)',line)}
            operation_rows.append(row)
    proof=dict(version=VERSION,generator_type='AdaptiveProductionSurface',geometry_identity=binding.geometry_identity,
        configuration=dataclasses.asdict(policy),configuration_identity=identity(dataclasses.asdict(policy)),
        initial_vertices=len(x),initial_triangles=len(np.load(output/'INITIAL.npz')['triangles']),seed_size_m=physical_seed_size,
        final_vertices=len(points),final_triangles=len(faces),carrier=carrier,mmgs_operation_rows=operation_rows,
        projections=getattr(authority,'projection',{}),projection_seconds_all_queries=getattr(authority,'projection_seconds',None),
        coordinate_hash=array_hash(points),topology_hash=array_hash(faces),sizing_field_identity=array_hash(final_metric),
        residual_hash=array_hash(residual),timing=timing,generation_seconds=time.perf_counter()-start,
        peak_rss_bytes=psutil.Process().memory_info().peak_wset,retained_rss_bytes=psutil.Process().memory_info().rss,
        fixture_mesh_used=False,first_layer_generated=False,prisms_generated=False,core_generated=False)
    proof['adaptation_identity']=identity(dict(carrier=carrier,mmgs=operation_rows,projections=proof['projections']))
    if hasattr(binding.qualification,'assurance'):
        proof['surface_assurance']=assurance
        proof['adaptation_identity']=identity(dict(adaptation=proof['adaptation_identity'],surface_assurance=assurance))
    proof['surface_identity']=identity({k:proof[k] for k in ('geometry_identity','configuration_identity','coordinate_hash','topology_hash','sizing_field_identity','residual_hash')})
    save(output/'GENERATION.json',proof);event('surface_generation_complete',vertices=len(points),triangles=len(faces))
    event('independent_qualification_start')
    audit=binding.qualifier(output,authority,binding.qualification)
    if not audit['passed']:
        raise GeometryFailure('SurfaceQualificationFailed','independent_qualification','audit failed',binding.geometry_identity)
    if assurance=='numerical':event('numerical_surface_acceptance_complete',accepted=audit['numerical_geometry']['face_count'],formal_geometry='NotRun')
    else:event('independent_qualification_complete',certified=audit['continuous_geometry']['face_count'])
    save(output/'EXECUTION_BINDING.json',binding.inspect())
    return proof

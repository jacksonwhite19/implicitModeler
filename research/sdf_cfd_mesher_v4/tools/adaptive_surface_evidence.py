"""Trace, compare and seal current surface evidence without downstream meshing."""
import collections, dataclasses, inspect, json, sys
import numpy as np
from historical_capsule_surface import *  # Explicit sealed-case replay, not generic admission.
from audit_adaptive_surface import audit_arrays, dist
from core_handoff_audit import surface
from production_surface_authority import publish_surface
import geometry_family_authority as geometry
import production_scale_authority as production
import adaptive_sdf_carrier as carrier
import shape_aware_metric as shape
import procedural_authorities as pa
import continuous_geometry_certifier as cert

def source_row(role,fn,classification):
    path=Path(inspect.getsourcefile(fn))
    return dict(role=role,path=path.relative_to(REPO).as_posix(),function=fn.__qualname__,
        line=inspect.getsourcelines(fn)[1],sha256=sha(path),classification=classification)

def trace():
    functions=[('provider entry',provider.main,'A: generic SDF provider'),
        ('SPD contract',provider.write_contract,'B: generic adaptation'),
        ('carrier assessment',carrier.assess_carrier_face,'B: generic adaptation'),
        ('carrier red/green conformal splitting',carrier.refine_metric_carriers,'B: generic adaptation'),
        ('curvature size',pa.curvature_size,'B: generic formula; D: frozen numeric policy'),
        ('normal-map principal curvature',pa.CohortAuthority._normal_map_principal_data,'B: generic adaptation'),
        ('normal differential',shape.generic_geometry_metric_from_differential,'B: generic adaptation'),
        ('shape metric',shape.apply_shape_metric,'B: generic adaptation'),
        ('projection dispatch',provider.canonicalize_and_project,'A: generic SDF provider'),
        ('topology gate',provider.topology_report,'A: generic SDF provider'),
        ('continuous geometry',cert.certify_face,'A: generic branch plus C: specialized geometry branches')]
    rows=[source_row(*args) for args in functions]
    fixed=[('SF2 execution entry','fresh_supported_refinement_family_qualification_2026-09-09/adapt_surface_stage.py',1,'A: unchanged producer execution partition'),
        ('SF2 producer','fresh_supported_refinement_family_qualification_2026-09-09/run_surface.py',1,'A: orchestration; D: case configuration'),
        ('SF2 seed and geometry','fresh_supported_refinement_family_qualification_2026-09-09/smooth_authority.py',82,'C: spline-extrusion chart/branch geometry'),
        ('SF2 resolution','fresh_supported_refinement_family_qualification_2026-09-09/resolution.py',1,'D: 0.64 family refinement'),
        ('generic implicit seed','held_out_geometry_cohort/src/cohort_surface.cpp',335,'A: CGAL implicit extraction; C: fixture geometry; D: seed criteria'),
        ('MMGS API runner','adaptive_surface_architecture/src/mmgs_metric_contract_runner.cpp',115,'B: stock MMGS adapter'),
        ('MMGS operations','fresh_supported_refinement_family_qualification_2026-09-09/dependency/complete_source/src/mmgs/mmgs1.c',1,'B: standard splits/collapses/swaps/moves and stopping'),
        ('MMGS split','fresh_supported_refinement_family_qualification_2026-09-09/dependency/complete_source/src/mmgs/split_s.c',1,'B: stock topology'),
        ('MMGS collapse','fresh_supported_refinement_family_qualification_2026-09-09/dependency/complete_source/src/mmgs/colver_s.c',1,'B: stock topology'),
        ('MMGS flip','fresh_supported_refinement_family_qualification_2026-09-09/dependency/complete_source/src/mmgs/swapar_s.c',1,'B: stock topology'),
        ('MMGS relocation','fresh_supported_refinement_family_qualification_2026-09-09/dependency/complete_source/src/mmgs/movpt_s.c',1,'B: stock relocation')]
    fixed.append(('MMGS local size grading','fresh_supported_refinement_family_qualification_2026-09-09/dependency/complete_source/src/mmgs/anisosiz_s.c',835,'B: MMGS_gradsiz_ani'))
    for role,path,line,kind in fixed:
        p=V3/path
        if not p.exists():raise ValueError('TraceSourceMissing:'+str(p))
        rows.append(dict(role=role,path=p.relative_to(REPO).as_posix(),line=line,sha256=sha(p),classification=kind))
    parameters=[]
    def param(name,value,units,source,scope,use):
        parameters.append(dict(name=name,value=value,units=units,source=source,scope=scope,capsule_use=use))
    for name,value,units in [('POSITION_LIMIT',pa.POSITION_LIMIT,'m'),('NORMAL_LIMIT_RAD',pa.NORMAL_LIMIT_RAD,'rad'),('HMAX',pa.HMAX,'m'),('curvature_size_floor',1e-7,'m'),('tangent_metric_floor',1e-9,'m')]:
        param(name,value,units,'adaptive_surface_generic_qualification/procedural_authorities.py','generic held-out reference policy; development-equivalence, not CFD accuracy',True)
    for name,value,units in [('curvature_reserve',2.5,'dimensionless'),('normal_derivative_step',1e-5,'m')]:
        param(name,value,units,'procedural_authorities.CohortAuthority._normal_map_principal_data','generic fallback normal differential',True)
    for name,value in dataclasses.asdict(shape.ShapeMetricPolicy()).items():
        param(name,value,'dimensionless','adaptive_surface_generic_qualification/shape_aware_metric.py','generic policy; reserve originally measured with MMGS',True)
    for name,value in dataclasses.asdict(carrier.CarrierOraclePolicy()).items():
        param(name,value,'m' if name=='minimum_edge_m' else 'dimensionless','adaptive_surface_generic_qualification/adaptive_sdf_carrier.py','generic oracle; not final acceptance',True)
    for name,value,units in [('hausd',5.148055963833207e-5,'m'),('hmax',.04,'m'),('verbose',1,'integer'),('angle',0,'integer'),('hgrad',1.3,'dimensionless'),('hgradreq',2.3,'dimensionless'),('MMGS_LOPTS',.71,'metric length'),('MMGS_LOPTL',1.4,'metric length')]:
        param(name,value,units,'adaptive_surface_architecture/src/mmgs_metric_contract_runner.cpp and stock MMGS defaults','generic reference runner; optimization triggers are not output gates',True)
    for name,value,units in [('facet_angle',25.,'deg'),('facet_size',.20,'m'),('facet_distance',.02,'m'),('edge_size',.08,'m'),('background_cell_size',.25,'m'),('background_radius_edge',3.,'dimensionless'),('relative_error_bound',1e-6,'dimensionless'),('seed',49944,'integer'),('physical_curvature_factor',.20,'dimensionless'),('projection_tolerance',2e-12,'m')]:
        param(name,value,units,'held_out_geometry_cohort/src/cohort_surface.cpp','cohort reference scale=1; background seed only', 'facet_size replaced by existing min(HMAX,0.20/kappa_max) formula' if name=='facet_size' else True)
    for name,value,units in [('SF2_resolution_scale',.64,'dimensionless'),('first_height',.000166527069,'m'),('foundation_target',.00121,'dimensionless'),('minimum_owned_layer',.00019373087622166372,'m'),('prism_response',.7250391657507819,'dimensionless')]:
        param(name,value,units,'fresh_supported_refinement_family_qualification_2026-09-09/SF2/A/wing/FROZEN_SURFACE_CALL.json','SF2 case/schedule configuration',False)
    v4paths=['tools/production_adaptive_surface.py','tools/production_capsule_cgal_seed.cpp','tools/production_surface_intersections.cpp','tools/audit_adaptive_surface.py','tools/production_surface_authority.py']
    v4sources=[dict(path=(V4/p).relative_to(REPO).as_posix(),sha256=sha(V4/p)) for p in v4paths]
    transitive=[]
    for name,module in sorted(sys.modules.items()):
        p=getattr(module,'__file__',None)
        if p and Path(p).resolve().is_relative_to(V3) and Path(p).is_file():
            transitive.append(dict(module=name,path=Path(p).relative_to(REPO).as_posix(),sha256=sha(p)))
    dependency=json.loads((V3/'fresh_supported_refinement_family_qualification_2026-09-09/MMGS_DEPENDENCY_QUALIFICATION.json').read_text())
    result=dict(v3_sources=rows,v3_import_bindings=transitive,v4_sources=v4sources,numeric_parameters=parameters,
        library_sha256=dependency['library_sha256'],adapter_sha256=dependency['adapter_sha256'],
        native_seed_sha256=sha(OUT/'runtime/seed'),native_intersections_sha256=sha(OUT/'runtime/intersections'),
        operations='CGAL implicit topology seed; SDF carrier red/green edge bisection; one stock MMGS call with splits, collapses, swaps, tangential moves and gradation; exact-authority projection; complete geometry and topology gates.',
        stopping='Carrier risk depth <=4 and adequacy required. MMGS finite internal loops and stagnation checks unchanged; no repeated adaptive MMGS campaign.',
        sizing_inputs='Principal normal-map curvature, position/normal bounds, maximum size, shape conditioning; direct grad/Hessian carrier oracle. SF2 additionally used schedule-derived prism metric and 0.64 resolution scale; neither used here.',
        feature_size='Branch/clearance oracle for authorities exposing competing components. No final generic local-feature-size size gate in the selected provider. Capsule has one convex branch.',
        deviations=['Fresh analytic capsule geometry plug-in; existing primitive SDF reused, analytic closest-segment projection and Hessian supplied.',
          'Fresh implicit CGAL cohort seed instead of spline-specific constrained charts.',
          'Absolute coarse seed initially failed carrier adequacy. V3 physical_edge_target=min(HMAX,0.20/kappa_max) used prospectively for facet size; no carrier/gate change.',
          'SF2-only prism metric and 0.64 scale excluded. V3 generic held-out development limits retained explicitly.',
          'Explicit metric size bounds and production provenance guard added; canonical ordering before adaptation/replay.',
          'Additional exact-predicate self-intersection, connected-component, duplicate-vertex and Euler checks; no new numeric quality gate.',
          'Bounded native execution and resource evidence added. No V3 source changed.'])
    result['implementation_identity']=identity(dict(v3=rows,imports=transitive,v4=v4sources,library=result['library_sha256'],runner=result['adapter_sha256'],seed=result['native_seed_sha256'],intersection=result['native_intersections_sha256']))
    save(OUT/'IMPLEMENTATION_TRACE.json',result)
    for r in rows:print(r['role']+': '+r['path']+':'+str(r['line']),flush=True)
    return result

def comparisons():
    def variation(x,f):
        normals=surface(x,f)['normal'];incidence=collections.defaultdict(list)
        for i,row in enumerate(f):
            for a,b in zip(row,np.roll(row,-1)):incidence[tuple(sorted((int(a),int(b))))].append(i)
        pairs=np.array([v for v in incidence.values() if len(v)==2])
        angle=np.degrees(np.arccos(np.clip((normals[pairs[:,0]]*normals[pairs[:,1]]).sum(1),-1,1)))
        return dist(angle)
    a=CapsuleGeometry(json.loads((OUT/'GEOMETRY.json').read_text())['definition'])
    path=V4/'results/production_scale_resume_fast/capsule_completion/SEALED_GEOMETRY.npz'
    z=np.load(path);f=z['wall_triangles'];used=np.unique(f);mapping=np.full(len(z['points']),-1);mapping[used]=np.arange(len(used));x=z['points'][used]
    old,_=audit_arrays(x,mapping[f],a);old['source_sha256']=sha(path);old['diagnostic_only']=True
    old['adjacent_face_normal_variation_deg']=variation(x,mapping[f])
    sfdir=V3/'fresh_supported_refinement_family_qualification_2026-09-09/SF2/A/wing/surface/A'
    z=np.load(sfdir/'QUALIFICATION_ARRAYS.npz');sf,_=audit_arrays(z['wall'],z['faces'])
    sf['adjacent_face_normal_variation_deg']=variation(z['wall'],z['faces'])
    sf.update(source_sha256=sha(sfdir/'QUALIFICATION_ARRAYS.npz'),provider=json.loads((sfdir/'provider_result_A.json').read_text()),
        continuous_certificate=json.loads((sfdir/'CONTINUOUS_CERTIFICATE.json').read_text()),
        note='Preserved accepted V3 SF2 adaptive wall. Different geometry: spline extrusion with 4 sheets, 6 curves, 4 corners. No SF2 regeneration or numeric gate transfer.')
    z=np.load(OUT/'fresh/SURFACE.npz')
    save(OUT/'COMPARISONS.json',dict(old_capsule_fixture=old,sf2_control=sf,
        fresh_capsule_adjacent_face_normal_variation_deg=variation(z['coordinates'],z['triangles'])))

def replay():
    rows={}
    for name in ('INITIAL.npz','SURFACE.npz','QUALITY.npz','GEOMETRY_CERTIFICATE.npz'):
        a=np.load(OUT/'fresh'/name);b=np.load(OUT/'replay'/name)
        rows[name]={k:bool(np.array_equal(a[k],b[k])) for k in a.files}
    a=json.loads((OUT/'fresh/GENERATION.json').read_text());b=json.loads((OUT/'replay/GENERATION.json').read_text())
    for key in ('coordinate_hash','topology_hash','sizing_field_identity','residual_hash','carrier','mmgs_operation_rows','projections','adaptation_identity','surface_identity'):
        rows[key]=a[key]==b[key]
    for key in ('fresh','replay'):
        rows[key+'_audits_pass']=json.loads((OUT/key/'AUDIT.json').read_text())['passed']
    result=dict(equal=rows,passed=all(all(v.values()) if isinstance(v,dict) else v for v in rows.values()),
        normalization='Lexicographic binary64 coordinates, cyclic face rotation preserving orientation, lexicographic triangles. No rounding. Timers and RSS excluded.',
        cold_seconds=a['generation_seconds'],replay_seconds=b['generation_seconds'])
    save(OUT/'DETERMINISM.json',result);return result

def ledger():
    entries=[dict(case='SF2',classification='V3AdaptiveProductionSurface',source='V4 authorities/DATA_AUTHORITIES.json -> V3 SF2/A/wing/surface/A/QUALIFICATION_ARRAYS.npz',scope='Preserved accepted V3 adaptive surface input, separately identified.')]
    for spec in geometry.definitions()+geometry.multi_region_definitions()+production.cohort():
        entries.append(dict(case=spec['name'],classification='ProceduralFixtureSurface',source='tools/geometry_family_authority.py:build -> radial_wall / complete_capacity_fixtures.sphere',
            primitives=sorted({d['primitive'] for d in spec['components']}),scope='Algorithmic evidence only; historical reports are unchanged.'))
    entries.append(dict(case='fresh capsule in this task',classification='AdaptiveProductionSurface',source='ProductionAdaptiveSdfSurfaceMesher.generate',scope='New SurfaceQualified authority; downstream stages not run.'))
    result=dict(entries=entries,counts=dict(collections.Counter(e['classification'] for e in entries)),
        rule='Fixture evidence remains valid for behavior directly tested. It does not prove production end-to-end surface-to-core qualification. Unlisted studies are Unknown until provenance is traced.')
    save(OUT/'SURFACE_PROVENANCE_LEDGER.json',result)

def main():
    result=trace();comparisons();ledger();equal=replay()
    generation=json.loads((OUT/'fresh/GENERATION.json').read_text())
    authority=publish_surface(OUT/'fresh',generation,result['implementation_identity'],equal)
    save(OUT/'SURFACE_SEAL.json',dict(passed=True,authority=authority,authority_sha256=sha(OUT/'fresh/SURFACE_AUTHORITY.json')))
    print(json.dumps(dict(determinism=equal['passed'],surface_identity=authority['surface_identity'],coordinate_hash=authority['coordinate_hash'],topology_hash=authority['topology_hash'])))

if __name__=='__main__':main()

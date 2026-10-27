"""Input/capability connections to existing geometry-owned production operations.

No geometry formula, projector, trajectory predictor or stage loop lives here.
Supported owners select their established semantics; unsupported owners fail closed.
"""
import copy,dataclasses,math
from pathlib import Path
import numpy as np
from sdf_cfd_mesher_v4.common.authority import identity,file_hash

def validate_analytic(g):
    if set(g)!=set(('authority','definition','metres_per_model_unit')) or g['authority']!='AnalyticPrimitive/1':
        raise ValueError('UnsupportedGeometryAuthority')
    if g['metres_per_model_unit']!=1:raise ValueError('AnalyticCoordinatesRequireMetres')
    d=g['definition']
    if not isinstance(d,dict) or d.get('primitive')!='capsule':
        raise ValueError('RequiredProductionGeometryCapabilitiesNotBound')
    if set(d)!=set(('primitive','radius','half_length')) or any(
        type(d[k]) not in (float,int) or not math.isfinite(d[k]) or d[k]<=0 for k in ('radius','half_length')):
        raise ValueError('InvalidAnalyticDimensions')

def validate_enclosure(e):
    if e is None:return
    if (set(e)!=set(('minimum_m','maximum_m','role')) or e['role']!='enclosure_only_no_boundary_mesh'):
        raise ValueError('UnsupportedEnclosureRole')
    lo,hi=np.asarray(e['minimum_m'],float),np.asarray(e['maximum_m'],float)
    if lo.shape!=(3,) or hi.shape!=(3,) or not np.isfinite([lo,hi]).all() or np.any(hi<=lo):
        raise ValueError('InvalidEnclosure')

def is_scalar(binding):
    from historical_capsule_surface import CapsuleGeometry
    return type(binding.authority) is CapsuleGeometry

def geometry(path):
    from standalone_case import load,bind
    from owned_runtime import BIN
    from sdf_cfd_mesher_v4.sdf.canonical import CanonicalKernel
    from surface_binding import bind_canonical
    case,_,_=load(path)
    if 'rhai' in case['geometry']:
        host=CanonicalKernel([str(BIN/'canonical_kernel_host.exe')],timeout=60)
        try:
            obj=host.load_rhai(case['geometry']['rhai'],case['geometry']['metres_per_model_unit'])
            if case['schema']=='V4MeshCase/4':
                from general_sdf_binding import connect
                return connect(host,obj,path)
            return host,bind(bind_canonical(obj),path)
        except BaseException:host.close();raise
    from historical_capsule_surface import CapsuleGeometry,capsule_binding
    validate_analytic(case['geometry'])
    class OwnedHandle:
        def close(self):pass
    a=CapsuleGeometry(copy.deepcopy(case['geometry']['definition']))
    return OwnedHandle(),bind(capsule_binding(a),path)

def branch_evidence(binding):
    from general_sdf_authority import GeneralSdfAuthority
    if type(binding.authority) is GeneralSdfAuthority:
        import general_sdf_authority as source_owner
        from sdf_cfd_mesher_v4.sdf import source_topology,operations,regional
        binding.qualification.verify(binding.authority)
        numerical=binding.qualification.assurance=='numerical'
        return dict(passed=True,geometry=binding.geometry_identity,source_topology=binding.qualification.source_topology,
            surface_assurance=binding.qualification.assurance,global_regular_C1_proved=not numerical,
            source_files={str(Path(module.__file__)):file_hash(module.__file__)
                for module in (source_owner,source_topology,operations,regional)},
            scope=('Authenticated source only; global regularity/completeness not proved; each realized vertex segment still requires exact field-progress and first source-normal height proofs.'
                if numerical else 'Regular C1 source topology only; each realized vertex segment requires exact field-progress and first source-normal height proofs.'),
            trajectory_semantics='AcceptedFrontState/path_length_m/1')
    if not is_scalar(binding):
        from live_spline_first_layer import branch_evidence as spline
        return spline(binding.authority)
    binding.qualification.verify(binding.authority)
    return dict(passed=True,geometry=binding.geometry_identity,
        source=file_hash(Path(__file__).with_name('geometry_family_authority.py')),
        scope='Existing analytic capsule scalar offset, positive outward thickness; generated targets, cells and collisions still require their existing checks',
        trajectory_semantics='ExistingAnalyticScalarOffset/1')

def bind_analytic(binding,path):
    from standalone_case import load
    from historical_capsule_surface import SurfaceSizing,capsule_seed
    from coupled_surface_metric import CoupledSurfaceMetric
    from shape_aware_metric import ShapeMetricPolicy
    from prism_host_metric import PrismMetricPolicy
    from surface_binding import QualificationContract
    from audit_adaptive_surface import audit_directory
    case,_,pin=load(path);a=binding.authority;s=case['surface']
    validate_analytic(case['geometry'])
    if not is_scalar(binding) or a.definition!=case['geometry']['definition']:
        raise ValueError('RunOwnedGeometryMismatch')
    definition=copy.deepcopy(a.definition)
    sizing=SurfaceSizing(**{k:s[k] for k in ('minimum_size_m','maximum_size_m','position_limit_m',
        'metric_multiplier','shape_response_reserve','normal_qualification_fraction','carrier_maximum_depth','grading')},
        normal_limit_deg=math.degrees(s['normal_limit_rad']))
    sizing.validate()
    if s['length_scale']<=0:raise ValueError('InvalidMetricLengthScale')
    if s['prism_host'] is None:raise ValueError('UncoupledStandalonePolicyNotBound')
    prism=PrismMetricPolicy(**s['prism_host']);shape=ShapeMetricPolicy()
    @dataclasses.dataclass(frozen=True)
    class SourceQualification(QualificationContract):
        def verify(self,authority):
            self.validate()
            from historical_capsule_surface import CapsuleGeometry
            if (file_hash(path)!=pin or authority is not a or authority.definition!=definition or
                CapsuleGeometry(definition).geometry_identity!=binding.geometry_identity):
                raise ValueError('RunOwnedGeometryMismatch')
    q=SourceQualification(1,2,0,copy.deepcopy(a.sheet_names),{}, {},s['position_limit_m'],s['normal_limit_rad'],2e-12,
        'Existing binary64 analytic capsule: exact SDF, outward branches, projection and continuous curvature bound',
        str(Path(__file__).with_name('geometry_family_authority.py')))
    class Connection(CoupledSurfaceMetric):
        def verify(self,b):
            q.verify(a);self.sizing.validate()
            if (b is not binding or b.authority is not a or b.metric_connection is not self or
                b.sizing_policy!=sizing or b.qualification is not q or b.qualifier is not audit_directory or
                b.seed_provider is not capsule_seed or
                (self.prism,self.shape,self.length_scale)!=(prism,shape,s['length_scale'])):
                raise ValueError('RunOwnedMetricConnectionDetached')
    c=Connection(a,sizing,prism,shape,s['length_scale'],identity(case))
    a._surface_metric_connection=c;binding.metric_connection=c
    binding.sizing_policy=sizing;binding.qualification=q;binding.qualifier=audit_directory
    c.verify(binding)
    return binding

"""Fail-closed publication/loading of a surface authority, independent of fronts."""
import json
from contextlib import contextmanager
from contextvars import ContextVar
from pathlib import Path
import numpy as np
from production_adaptive_surface import identity, array_hash, sha, save, require_production_surface, canonical

BOUND_FIELDS=('generator_type','implementation_identity','geometry_identity','configuration_identity',
              'sizing_field_identity','adaptation_identity','coordinate_hash','topology_hash','residual_hash')
FASTPROD_ASSURANCE='FastProdSingleQualifiedSurface/1'
FASTPROD_NUMERICAL_ASSURANCE='FastProdNumericalSurface/1'
_fastprod_wall=ContextVar('fastprod_qualified_wall',default=None)


@contextmanager
def fastprod_surface_scope(directory):
    """Explicit process-local use only; archived callers remain replay-gated."""
    token=_fastprod_wall.set(Path(directory).resolve())
    try:yield
    finally:_fastprod_wall.reset(token)


def require_fastprod_surface_scope(directory):
    if _fastprod_wall.get()!=Path(directory).resolve():
        raise ValueError('FastProdSurfaceRequiresProcessLocalScope')


def fastprod_surface_scope_active(directory):
    """Storage selection only; the existing scope and acceptance own admission."""
    return directory is not None and _fastprod_wall.get()==Path(directory).resolve()

def scientific_identity(record):
    bound={k:record[k] for k in BOUND_FIELDS}
    if 'assurance' in record:bound['assurance']=record['assurance']
    if 'acceptance_identity' in record:bound['acceptance_identity']=record['acceptance_identity']
    return identity(bound)


def qualification_flags(numerical):
    result={k:True for k in ('geometry','topology','orientation','self_intersection','sizing_audit')}
    result['determinism']='not_evaluated'
    if numerical:
        result['geometry']='sampled_numerical'
        result.update(source_completeness='not_evaluated',continuous_geometry='not_evaluated',
            exact_final_points='not_evaluated')
    return result


def acceptance_binding(audit):
    bound={k:audit[k] for k in ('surface_assurance','acceptance_inputs','acceptance_contract','acceptance_custody_completed')}
    bound['checks']={k:audit[k] for k in ('topology','normal','sdf_residual','feature_ownership',
        'source_coverage','continuous_geometry','final_point_qualification')}
    bound['checks']['numerical_geometry']={k:v for k,v in audit['numerical_geometry'].items() if k!='seconds'}
    bound['checks']['self_intersection']={k:audit['self_intersection'][k] for k in ('forbidden','unresolved','missing','duplicate','coverage','method')}
    return bound

def load_production_wall(directory):
    """Only this loader admits the new production surface contract.

    Historical fixture checkpoints remain readable by their diagnostic tools;
    they cannot enter this production path by renaming a generator field.
    """
    directory=Path(directory)
    record=json.loads((directory/'SURFACE_AUTHORITY.json').read_text(encoding='utf-8'))
    if 'assurance' in record:
        require_fastprod_surface_scope(directory)
        numerical=record['assurance']==FASTPROD_NUMERICAL_ASSURANCE
        expected=qualification_flags(numerical)
        if (record['assurance'] not in (FASTPROD_ASSURANCE,FASTPROD_NUMERICAL_ASSURANCE) or record.get('qualification')!=expected or
                record.get('generator_type')!='AdaptiveProductionSurface' or
                record.get('replay_performed') is not False or record.get('restart_supported') is not False or
                record.get('archived_publication') is not False or 'determinism_identity' in record):
            raise ValueError('InvalidFastProdSurfaceAssurance')
        required={'SURFACE.npz','AUDIT.json','QUALITY.npz','GEOMETRY_CERTIFICATE.npz',
                  'GENERATION.json','REFERENCES.npz','IMPLEMENTATION.json','CASE.json'}
        if numerical:
            required.remove('GEOMETRY_CERTIFICATE.npz');required.add('NUMERICAL_GEOMETRY.json')
            if 'acceptance_identity' not in record:raise ValueError('NumericalAcceptanceBindingRequired')
        if set(record['files'])!=required:raise ValueError('IncompleteFastProdSurfaceFiles')
    else:require_production_surface(record)
    if record['surface_identity']!=scientific_identity(record):raise ValueError('SurfaceAuthorityRootMismatch')
    for name,digest in record['files'].items():
        path=(directory/name).resolve()
        if not path.is_relative_to(directory.resolve()):raise ValueError('SurfaceAuthorityPathEscape')
        if sha(path)!=digest:raise ValueError('SurfaceAuthorityFileMismatch:'+name)
    with np.load(directory/'SURFACE.npz',allow_pickle=False) as data:
        x,f=canonical(data['coordinates'],data['triangles'])
        for key,value in [('coordinate_hash',x),('topology_hash',f),('sizing_field_identity',data['metric']),('residual_hash',data['residual'])]:
            if array_hash(value)!=record[key]:raise ValueError('SurfacePayloadBindingMismatch:'+key)
    audit=json.loads((directory/'AUDIT.json').read_text(encoding='utf-8'))
    if not audit['passed']:raise ValueError('FailedSurfaceAudit')
    from surface_requirements195 import verify_numerical_evidence
    verify_numerical_evidence(json.loads((directory/'CASE.json').read_text(encoding='utf-8')),audit,geometry_identity=record['geometry_identity'])
    if audit.get('surface_assurance')=='numerical' and record.get('assurance')!=FASTPROD_NUMERICAL_ASSURANCE:
        raise ValueError('NumericalReceiptCannotUpgrade')
    if record.get('assurance')==FASTPROD_NUMERICAL_ASSURANCE:
        if (audit.get('surface_assurance')!='numerical' or audit.get('acceptance_custody_completed') is not True
                or audit.get('source_coverage',{}).get('status')!='NotRun'
                or audit.get('continuous_geometry',{}).get('status')!='NotRun'
                or audit.get('final_point_qualification',{}).get('status')!='NotRun'
                or audit.get('numerical_geometry',{}).get('status')!='NumericallyAccepted'
                or audit['numerical_geometry']['passed'] is not True
                or identity(acceptance_binding(audit))!=record['acceptance_identity']):
            raise ValueError('InvalidNumericalAcceptanceEvidence')
        if json.loads((directory/'NUMERICAL_GEOMETRY.json').read_text(encoding='utf-8'))!=audit['numerical_geometry']:
            raise ValueError('DetachedNumericalGeometryEvidence')
        with np.load(directory/'SURFACE.npz',allow_pickle=False) as z,np.load(directory/'REFERENCES.npz',allow_pickle=False) as r:
            from general_source_contract import array_binding
            actual=dict(zip(('coordinates','faces','face_refs','curve_edges','corner_refs','metric','vertex_refs'),
                (array_binding(v) for v in (z['coordinates'],z['triangles'],r['face_refs'],r['curve_edges'],r['corner_refs'],z['metric'],r['vertex_refs']))))
        if actual!=audit['acceptance_inputs']:raise ValueError('DetachedNumericalAcceptancePayload')
    return x,f,record


def verify_surface_sidecars(directory,record,*,verify_implementation=True):
    """Shared first-layer input check, without pretending a replay occurred."""
    directory=Path(directory)
    impl=None
    # Historical bound-provider reference checks did not require this sidecar.
    # FastProd always does; its single qualification cannot borrow replay data.
    if verify_implementation or 'assurance' in record:
        impl=json.loads((directory/'IMPLEMENTATION.json').read_text(encoding='utf-8'))
        if identity(impl)!=record['implementation_identity']:raise ValueError('SurfaceSidecarMismatch')
    if 'assurance' in record:
        require_fastprod_surface_scope(directory)
        return impl,record['files']['REFERENCES.npz']
    replay=json.loads((directory/'REPLAY.json').read_text(encoding='utf-8'))
    if verify_implementation and (identity(replay)!=record['determinism_identity'] or replay.get('passed') is not True):
        raise ValueError('SurfaceSidecarMismatch')
    return impl,replay['replay_payloads']['REFERENCES.npz']


def record_fastprod_qualification(directory,generation,implementation_identity):
    """Record one actually qualified surface. This is NOT an archived seal."""
    directory=Path(directory);require_fastprod_surface_scope(directory)
    if (directory/'SURFACE_AUTHORITY.json').exists():raise ValueError('PreserveExistingSurfaceReceipt')
    audit=json.loads((directory/'AUDIT.json').read_text(encoding='utf-8'))
    if not audit['passed']:raise ValueError('CannotRecordUnqualifiedSurface')
    from surface_requirements195 import verify_numerical_evidence
    verify_numerical_evidence(json.loads((directory/'CASE.json').read_text(encoding='utf-8')),audit,geometry_identity=generation['geometry_identity'])
    if generation.get('fixture_mesh_used') or generation['generator_type']!='AdaptiveProductionSurface':
        raise ValueError('FixtureSurfacePublicationForbidden')
    record={k:generation[k] for k in BOUND_FIELDS if k!='implementation_identity'}
    numerical=generation.get('surface_assurance')=='numerical'
    if numerical and (audit.get('surface_assurance')!='numerical' or not audit.get('acceptance_custody_completed')):
        raise ValueError('NumericalAcceptanceCustodyRequired')
    if not numerical and audit.get('surface_assurance','qualification')!='qualification':raise ValueError('NumericalReceiptCannotUpgrade')
    record.update(implementation_identity=implementation_identity,assurance=FASTPROD_NUMERICAL_ASSURANCE if numerical else FASTPROD_ASSURANCE,
        replay_performed=False,restart_supported=False,archived_publication=False)
    if numerical:record['acceptance_identity']=identity(acceptance_binding(audit))
    record['surface_identity']=scientific_identity(record)
    record['qualification']=qualification_flags(numerical)
    record['scope']='Process-local FastProd: new mesh independently qualified once; repeatability not evaluated. No archived publication/restart, prism feasibility or CFD claim.'
    if numerical:record['scope']='Process-local FastProd numerical surface acceptance. Geometry sampled at vertices, edge midpoints and centroids. No source completeness, continuous error, exact final point/normal or repeatability proof; no prism feasibility or CFD claim.'
    names=['SURFACE.npz','AUDIT.json','QUALITY.npz',
        'GENERATION.json','REFERENCES.npz','IMPLEMENTATION.json','CASE.json',
        'NUMERICAL_GEOMETRY.json' if numerical else 'GEOMETRY_CERTIFICATE.npz']
    record['files']={name:sha(directory/name) for name in names}
    temporary=directory/'SURFACE_AUTHORITY.json.tmp';save(temporary,record)
    temporary.replace(directory/'SURFACE_AUTHORITY.json')
    load_production_wall(directory)
    verify_surface_sidecars(directory,record)
    return record

def publish_surface(directory,generation,implementation_identity,replay):
    directory=Path(directory)
    audit=json.loads((directory/'AUDIT.json').read_text(encoding='utf-8'))
    if generation.get('surface_assurance')=='numerical' or audit.get('surface_assurance')=='numerical':
        raise ValueError('NumericalReceiptCannotPublishArchivedSurface')
    if 'assurance' in generation:raise ValueError('FastProdReceiptCannotPublishArchivedSurface')
    existing=directory/'SURFACE_AUTHORITY.json'
    if existing.exists() and 'assurance' in json.loads(existing.read_text(encoding='utf-8')):
        raise ValueError('FastProdReceiptCannotPublishArchivedSurface')
    if not replay['passed'] or not audit['passed']:raise ValueError('CannotPublishUnqualifiedSurface')
    if generation.get('fixture_mesh_used') or generation['generator_type']!='AdaptiveProductionSurface':raise ValueError('FixtureSurfacePublicationForbidden')
    record={k:generation[k] for k in BOUND_FIELDS if k!='implementation_identity'}
    record['implementation_identity']=implementation_identity
    record['surface_identity']=scientific_identity(record)
    record['qualification']={k:True for k in ('geometry','topology','orientation','self_intersection','sizing_audit','determinism')}
    record['scope']='SurfaceQualified under recovered V3 generic development-equivalence geometry limits; no first-layer, prism, core-handoff, CFD or aerodynamic accuracy claim.'
    record['files']={name:sha(directory/name) for name in ('SURFACE.npz','AUDIT.json','QUALITY.npz','GEOMETRY_CERTIFICATE.npz','GENERATION.json')}
    record['determinism_identity']=identity(replay)
    require_production_surface(record)
    temporary=directory/'SURFACE_AUTHORITY.json.tmp';save(temporary,record)
    temporary.replace(directory/'SURFACE_AUTHORITY.json')
    load_production_wall(directory)
    return record


def reuse_source_coverage(directory,record,contract,authority,x,f,refs):
    """Consume the matching authenticated final owner; never certify again."""
    from contextlib import ExitStack
    from immutable_state_access import guard
    from general_source_contract import array_binding
    from sdf_cfd_mesher_v4.common.authority import frozen
    directory=Path(directory).resolve()
    contract.verify(authority);contract_pin=contract.custody_identity()
    original=tuple(np.asarray(v) for v in (x,f,refs['face_refs'],refs['curve_edges'],refs['corner_refs']))
    pins=tuple(array_binding(v) for v in original)
    private=tuple(frozen(v,v.dtype.str) for v in original)
    if tuple(array_binding(v) for v in private)!=pins:raise ValueError('ChangedCoverageReuseInputs')
    with ExitStack() as held:
        held.enter_context(guard(directory/'SURFACE_AUTHORITY.json'))
        retained=json.loads((directory/'SURFACE_AUTHORITY.json').read_text(encoding='utf-8'))
        if retained!=record:raise ValueError('DetachedCoverageAuthority')
        names=set(retained['files'])|{'AUDIT.json','SURFACE.npz','REFERENCES.npz','IMPLEMENTATION.json'}
        if 'assurance' not in retained:names.add('REPLAY.json')
        for name in sorted(names):
            path=(directory/name).resolve()
            if not path.is_relative_to(directory):raise ValueError('SurfaceAuthorityPathEscape')
            held.enter_context(guard(path))
        px,pf,pr,pe,pc=private
        result=_reuse_source_coverage(directory,retained,contract,authority,px,pf,
            dict(face_refs=pr,curve_edges=pe,corner_refs=pc))
        contract.verify(authority)
        if contract.custody_identity()!=contract_pin:raise ValueError('ChangedCoverageReuseContract')
        if (tuple(array_binding(v) for v in original)!=pins or
                tuple(array_binding(v) for v in private)!=pins):raise ValueError('ChangedCoverageReuseInputs')
        if record!=retained:raise ValueError('ChangedCoverageReuseAuthority')
        return result


def _reuse_source_coverage(directory,record,contract,authority,x,f,refs):
    from dataclasses import asdict
    from general_source_contract import array_binding
    contract.verify(authority)
    mode=contract.assurance
    numerical=record.get('assurance')==FASTPROD_NUMERICAL_ASSURANCE
    if numerical!=(mode=='numerical'):raise ValueError('SurfaceAssuranceMismatch')
    # Re-enter the one loader before reuse; its file and scientific bindings
    # establish custody for the actual returned wall and reference arrays.
    current_x,current_f,current=load_production_wall(directory)
    if current!=record or current_x.tobytes()!=x.tobytes() or current_f.tobytes()!=f.tobytes():
        raise ValueError('DetachedCoverageWall')
    _,pin=verify_surface_sidecars(directory,record,verify_implementation=False)
    if sha(Path(directory)/'REFERENCES.npz')!=pin:raise ValueError('ReferenceIdentityMismatch')
    actual=dict(zip(('coordinates','faces','face_refs','curve_edges','corner_refs'),
        (array_binding(v) for v in (x,f,refs['face_refs'],refs['curve_edges'],refs['corner_refs']))))
    audit=json.loads((Path(directory)/'AUDIT.json').read_text(encoding='utf-8'))
    if numerical:
        if any(audit['acceptance_inputs'][k]!=v for k,v in actual.items()):raise ValueError('DetachedNumericalWall')
        bound=audit['acceptance_contract']
        if identity(bound)!=identity(dict(continuous_target=contract.target,limits=asdict(contract),assurance=mode)):
            raise ValueError('DetachedNumericalContract')
        return dict(status='NotRun',passed=None,reused=True,surface_assurance=mode,
            surface_identity=record['surface_identity'],reason='Numerical acceptance does not prove source completeness.')
    coverage=audit.get('source_coverage')
    if (not coverage or coverage.get('passed') is not True or coverage.get('continuous_target')!=contract.target
            or any(coverage['materialized_surface'][k]!=v for k,v in actual.items())):
        raise ValueError('DetachedQualifiedSourceCoverage')
    return dict(coverage,reused=True,surface_assurance=mode,surface_identity=record['surface_identity'])

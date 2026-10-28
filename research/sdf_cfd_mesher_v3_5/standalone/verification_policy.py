"""Explicit dispatch; legacy cold-required contracts are never production aliases."""
from pathlib import Path
from sdf_cfd_mesher_v4.common.authority import identity

PROFILE='V35DirectProducerAcceptance/1'
DIRECT='V35ProductionDirectResult/1'
ROOT_RESULT='V35ProductionMeshResult/1'
MANIFEST='V35ProductionVerificationManifest/1'
FINALIZER='V35ProductionArtifactFinalization/1'
OFFLINE='V35RetainedDirectAudit/1'
CONTRACT=dict(version=PROFILE,workflow='complete_v3_prisms_v4_direct',
    producer_obligations='ALL_TWELVE_MANDATORY_UNCHANGED',
    construction_and_quality='UNCHANGED',native_export_fidelity='MANDATORY',
    artifact_binding='GUARDED_PRODUCER_SERIALIZED_NATIVE',source_resource_closure='MANDATORY',
    separate_semantic_replay='NOT_RUN',legacy_cold_required_acceptance=False,
    source_assurance='ORIGINAL_RUN_ONLY',source_evidence_binding='V35RetainedSourceEvidence/1:producer_held_full_population_and_method;offline_authenticated_no_live_source',
    source_segments='V35LiveFinalSourcePopulation/1:production_same_run_authentication_or_recompute;audit_recompute',continuous_sdf_fluid='NOT_PERFORMED',CFD_qualified=False)

def profile():
    from finite_research.direct_fill_contract import contract_identity,VERSION
    from finite_research.direct_contract import method
    composed=dict(CONTRACT,physical_shape_component=dict(version=VERSION,identity=contract_identity()),method_identity=identity(method()))
    return dict(version=PROFILE,identity=identity(composed))

def selection(mode,settings,stop_after,origin='explicit_api'):
    if mode not in ('audit','production'):raise ValueError('UnknownVerificationMode')
    if origin not in ('explicit_api','explicit_cli','v35_audit_default') or (origin=='v35_audit_default' and mode!='audit'):raise ValueError('InvalidVerificationOrigin')
    if mode=='production':
        if stop_after!='complete' or settings.get('background',{}).get('schema')!='V4DirectFillEngineeringPolicy/1':
            raise ValueError('ProductionRequiresCompleteDirectWorkflow')
        from finite_research.direct_contract import validate_policy
        validate_policy(settings['background'])
    return dict(schema='V35VerificationSelection/1',mode=mode,
        requested=None if origin=='v35_audit_default' else mode,effective=mode,origin=origin,
        profile=profile() if mode=='production' else None,
        background_identity=identity(settings.get('background')),stop_after=stop_after)

def require_selection(value,settings,mode=None):
    if not isinstance(value,dict) or value!=selection(value.get('mode'),settings,value.get('stop_after'),value.get('origin')):
        raise ValueError('ChangedVerificationSelection')
    if mode is not None and value['mode']!=mode:raise ValueError('VerificationModeMismatch')
    return value

def execution_record(choice,proof):
    from finite_research.direct_evidence_modules import joined_audit
    if set(proof.get('proof_receipts',{}))!=set(joined_audit.REQUIRED) or proof.get('whole_joined_geometry_passed') is not True:
        raise ValueError('IncompleteProducerExecutionRecord')
    rows={name:dict(status='reused_with_binding' if name=='preservation' else 'recomputed',
        receipt=proof['proof_receipts'][name],geometry_identity=proof['geometry_identity']) for name in joined_audit.REQUIRED}
    rows['preservation']['scope']='Earlier within-run inherited join result, bound at checkpoint'
    for name in ('boundary_collision','collision'):rows[name]['scope']='Native classifications and fresh coverage; persisted certified witnesses read back in producer'
    rows['native_export']=dict(status='recomputed',scope='Actual conversion, exact correspondence and checkMesh')
    rows['artifact_binding']=dict(status='derived',scope='Checked-state equality and held serialized/native roots')
    rows['separate_semantic_replay']=dict(status='not_run' if choice['mode']=='production' else 'recomputed',
        reason='Explicit production profile omits separate full semantic replay; source segments permit same-run final-population authentication under V35LiveFinalSourcePopulation/1' if choice['mode']=='production' else 'Mandatory cold audit; source segments recomputed')
    return dict(selection=choice,checks=rows)

def production_result(value,descriptor=None):
    if (value.get('schema')!=DIRECT or value.get('producer_checks_passed') is not True or value.get('geometry_complete') is not True
        or value.get('artifact_fidelity_passed') is not True or value.get('independent_audit_status')!='not_run'
        or value.get('acceptance_status')!='pending_supervisor'
        or value.get('provisional') is not False or value.get('acceptance_profile')!=profile()
        or value.get('verification_policy')!='production' or value.get('audit')!='not_run'
        or any(k in value for k in ('passed','full_mesh_complete','readback_sha256','all_twelve_fresh'))):
        raise ValueError('InvalidProductionDirectResult')
    if descriptor is not None and value.get('artifact_finalization')!=descriptor:raise ValueError('ChangedProductionFinalization')
    return value

def complete_after_finalization(folder,result,descriptor,remaining):
    """Only policy branch: no catch/fallback from a failed audit."""
    from artifact_finalization import run_independent_audit
    from sdf_cfd_mesher_v4.common.authority import file_hash
    result=dict(result)
    if descriptor['verification_policy']=='audit':
        receipt=run_independent_audit(folder,descriptor,remaining)
        result.update(full_mesh_complete=True,provisional=False,readback_sha256=file_hash(Path(folder)/'READBACK.json'),audit_finalization=descriptor)
    elif descriptor['verification_policy']=='production':
        if descriptor.get('schema')!=FINALIZER or descriptor.get('acceptance_profile')!=profile():raise ValueError('UnknownProductionFinalizer')
        result.update(provisional=False,artifact_finalization=descriptor,acceptance_status='pending_supervisor')
        production_result(result,descriptor)
    else:raise ValueError('UnknownVerificationMode')
    return result

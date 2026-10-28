"""Read closed evidence and compare physical operands; no geometry execution."""
from pathlib import Path
import hashlib,json,sys,traceback
import numpy as np
from passage_verify3_auth import authenticate_pair, input_scope
BASE=Path(__file__).resolve().parents[1];OUT=BASE/'outputs';PKG=BASE/'work/phase1a/research/sdf_cfd_mesher_v3_5/standalone'
def read(p):return json.loads(Path(p).read_text())
def sha(p):
    with Path(p).open('rb') as s:return hashlib.file_digest(s,'sha256').hexdigest()
def arrays(a,b):
    reports={}
    with np.load(a,allow_pickle=False) as x,np.load(b,allow_pickle=False) as y:
        assert set(x.files)==set(y.files)=={'points','prisms','tetrahedra','wall','farfield'}
        for name in x.files:
            v,w=x[name],y[name]
            reports[name]=dict(dtype=v.dtype.str,shape=list(v.shape),sha256=hashlib.sha256(v.tobytes()).hexdigest(),
                dtype_equal=v.dtype==w.dtype,shape_equal=v.shape==w.shape,byte_equal=v.tobytes()==w.tobytes())
    return reports
def equal_arrays(result):return all(r['dtype_equal'] and r['shape_equal'] and r['byte_equal'] for r in result.values())
def plans(a,b):
    result={}
    names={p.name for p in (a/'plan').glob('*.npy')};expected={p.name for p in (b/'plan').glob('*.npy')}
    assert names==expected=={'delta.npy','faces.npy','first_front.npy','increments.npy','later_vectors.npy','positions.npy','source_normals.npy','wall.npy'},'IncompletePrismPlanPopulation'
    for p in sorted((a/'plan').glob('*.npy')):
        v=np.load(p,allow_pickle=False);w=np.load(b/'plan'/p.name,allow_pickle=False)
        result[p.name]=v.dtype==w.dtype and v.shape==w.shape and v.tobytes()==w.tobytes()
    return result
def without_stamp(value):
    # Only the named top-level joined-input provenance stamp varies.
    return {k:v for k,v in value.items() if k!='geometry_identity'}

def native_behavior(value):
    # Execution paths and elapsed command times vary with output/run. Compare
    # actual construction counts/coordinate fidelity and native exit/bounds.
    result={k:v for k,v in value.items() if k not in ('first','later')}
    for key in ('first','later'):
        result[key]={k:v for k,v in value[key].items() if k not in ('command','seconds')}
    return result

def archive_equal(a,b):
    with np.load(a,allow_pickle=False) as x,np.load(b,allow_pickle=False) as y:
        return set(x.files)==set(y.files) and all(x[n].dtype==y[n].dtype and x[n].shape==y[n].shape and x[n].tobytes()==y[n].tobytes() for n in x.files)

def apex_state(a,b):
    authentication=authenticate_pair(a,b,mode)
    aa=a/'background';bb=b/'background'
    current=read(aa/'SHARED_APEX_REPAIR.json');old=read(bb/'SHARED_APEX_REPAIR.json')
    conditioning=[]
    for root,repair in ((a,current),(b,old)):
        background=root/'background';c=read(background/'conditioning/CONDITIONING.json')
        assert repair['conditioning_receipt_sha256']==sha(background/'conditioning/CONDITIONING.json')
        assert read(background/'RESULT.json')['conditioning']['receipt_sha256']==repair['conditioning_receipt_sha256']
        parent=read(background/'PARENT.json')
        assert c['parent']==parent==read(background/'PARENT_EXPECTATIONS.json')['binding']
        assert parent['case_sha256']==sha(root/'CASE.json')
        assert read(background/'RESULT.json')['parent']==parent
        assert c['input_sha256']==sha(background/'conditioning/INPUT.npz')
        assert c['output_sha256']==sha(background/'conditioning/CONDITIONED.npz')
        normalized={k:v for k,v in c.items() if k not in ('seconds','timings','parent')}
        normalized['parent']={k:v for k,v in parent.items() if k not in ('case_sha256','parent_prism_mesh')}
        conditioning.append(normalized)
    assert conditioning[0]==conditioning[1],'SubstantiveConditioningStateDiffers'
    assert {k:v for k,v in current.items() if k!='conditioning_receipt_sha256'}=={k:v for k,v in old.items() if k!='conditioning_receipt_sha256'},'FullSharedApexRepairRecordDiffers'
    assert current['protected_coordinate_bits_unchanged'] is True and current['connectivity_changed'] is False
    assert current['source_queries']==0
    files=['conditioning/INPUT.npz','conditioning/CONDITIONED.npz','native/generated/NETGEN.npz','JOINED_EVIDENCE.npz']
    for relative in files:assert archive_equal(aa/relative,bb/relative),'ApexOperandDiffers:'+relative
    assert archive_equal(a/'native/SEALED_PLAN.npz',b/'native/SEALED_PLAN.npz'),'NativePrismPrefixDiffers'
    metadata=read(aa/'JOINED_RECONSTRUCTION.json')['cell_metadata']
    assert metadata==read(bb/'JOINED_RECONSTRUCTION.json')['cell_metadata'],'CanonicalCellFaceMetadataDiffers'
    with np.load(aa/'conditioning/CONDITIONED.npz',allow_pickle=False) as source,np.load(aa/'COMPLETE_MESH.npz',allow_pickle=False) as final:
        before=source['points'];after=final['points']
        assert before.dtype==after.dtype and before.shape==after.shape
        changed=np.flatnonzero(np.any(before.view('<u8')!=after.view('<u8'),axis=1)).tolist()
        assert changed==current['changed_vertices'],'IncompleteMovedVertexInventory'
        for record in current['records']:
            if record['status']=='COMMITTED':
                vertex=record['vertex']
                assert before[vertex].tobytes()==np.asarray(record['before_m'],dtype='<f8').tobytes()
                assert after[vertex].tobytes()==np.asarray(record['after_m'],dtype='<f8').tobytes()
        assert source['tetrahedra'].tobytes()==final['tetrahedra'].tobytes(),'RepairChangedConnectivity'
        protected=np.unique(np.concatenate([final['prisms'].ravel(),final['wall'].ravel(),final['farfield'].ravel()]))
        assert before[protected].tobytes()==after[protected].tobytes(),'ProtectedCoordinatesChanged'
    carrier=read(a/'upstream/surface/A/fresh/CARRIER.json')
    assert carrier['enabled'] is False,'NodalCarrierNotExplicit'
    return dict(authentication=authentication,substantive_repair_report_exact=True,complete_committed_star_records_exact=True,conditioning_receipt_hash_authenticated_per_run=True,archives_exact=files,native_sealed_plan_exact=True,canonical_cell_face_metadata_exact=True,
        actual_moved_vertices=changed,protected_coordinate_bytes_unchanged=True,connectivity_unchanged=True,
        conditioning_outcome=read(aa/'conditioning/CONDITIONING.json')['outcome'],conditioning_substantive_state_exact=True,
        exact_comparison_exclusions=['SHARED_APEX_REPAIR.conditioning_receipt_sha256 authenticated per run','CONDITIONING.seconds','CONDITIONING.timings','CONDITIONING.parent.case_sha256 authenticated per run','CONDITIONING.parent.parent_prism_mesh authenticated by V2 own trusted chain; source_identity stays exact'])
mode=sys.argv[1];assert mode=='production','ExistingAuditAndProductionMustNotRerun';pre=read(OUT/'PASSAGE_VERIFY1_PREFLIGHT.json')
folder=Path(pre['outputs'][mode]);baseline=Path(pre['baseline']);report=dict(schema='PassageVerify3CrossRunComparison/1',mode=mode,passed=False,run=str(folder),baseline=str(baseline),original_failure_preserved=True,contract='passage_verify3_auth.py',contract_sha256=sha(Path(__file__).with_name('passage_verify3_auth.py')),comparator_sha256=sha(Path(__file__)))
held=None
try:
    review=read(OUT/'PASSAGE_VERIFY3_IMPLEMENTATION_REVIEW_V2.json');assert review['passed'] is True
    for rel,pin in review['sha256'].items():assert sha(BASE/rel)==pin,'ChangedReviewedControl:'+rel
    from contextlib import ExitStack
    held=ExitStack()
    held.enter_context(input_scope((folder,baseline,Path(pre['outputs']['audit'])), [OUT/'PASSAGE_VERIFY1_PREFLIGHT.json', OUT/('PASSAGE_VERIFY1_'+mode.upper()+'_RUN.json')]))
    run=read(OUT/('PASSAGE_VERIFY1_'+mode.upper()+'_RUN.json'));assert run['state']=='closed' and run['exit_code']==0,'NormalCLIRefused'
    from passage_verify3_auth import _held
    _held.hold(run['supervisor_record']);_held.hold(run['source_record'])
    root=read(folder/'RESULT.json');direct=read(folder/'background/RESULT.json');verification=read(folder/'VERIFICATION.json')
    supervisor=read(run['supervisor_record']);sources=read(run['source_record'])
    assert supervisor['exit_code']==0 and supervisor['stop_reason'] is None and supervisor.get('monitor_error') is None
    assert sources['state']=='closed' and sources['passed'] is True and not sources['added'] and not sources['changed']
    assert sources['identity']==pre['runtime_identity']==supervisor['source_identity']==verification['source_identity']
    assert root['background']==direct and verification['verification_policy']==mode
    if mode=='audit':
        assert root['passed'] is True and root['full_mesh_complete'] is True and direct['schema']=='V35V4DirectComplete/1'
        assert direct['full_mesh_complete'] is True and direct['provisional'] is False
        replay=read(folder/'background/READBACK.json');assert replay['passed'] is True and replay['all_twelve_semantically_replayed'] is True
        assert sha(folder/'background/READBACK.json')==direct['readback_sha256']
        assert verification['schema']=='V35VerificationManifest/1' and verification['status']=='audit_accepted'
        assert verification['independent_semantic_replay']=='passed'
        descriptor=direct['audit_finalization']
    else:
        assert root['schema']=='V35ProductionMeshResult/1' and direct['schema']=='V35ProductionDirectResult/1'
        for r in (root,direct):
            assert 'passed' not in r and 'full_mesh_complete' not in r and r['audit']=='not_run'
            assert r['geometry_complete'] is True and r['producer_checks_passed'] is True and r['artifact_fidelity_passed'] is True
        assert verification['schema']=='V35ProductionVerificationManifest/1' and verification['acceptance_status']=='production_accepted'
        assert verification['independent_semantic_replay']=='not_run' and verification['audit']=='not_run'
        assert not (folder/'background/READBACK.json').exists()
        descriptor=direct['artifact_finalization']
    assert verification['finalization']==descriptor
    sys.path.insert(0,str(PKG));import geometry
    from artifact_finalization import hold_manifest
    from sdf_cfd_mesher_v4.common.authority import identity
    from contextlib import ExitStack
    with ExitStack() as scope:finalized=hold_manifest(folder/'background',descriptor,scope)
    assert sha(folder/'background/COMPLETE_MESH.npz')==direct['mesh_sha256']
    proof=direct['proof'];required={'preservation','domain','incidence','winding','interface_qa','joined_qa','physical_qa','surfaces','boundary_collision','enclosure','collision','filling'}
    assert proof['whole_joined_geometry_passed'] is True and set(proof['proof_receipts'])==required
    for name,pin in proof['proof_receipts'].items():
        p=folder/'background/audit'/(name.upper()+'.json');assert sha(p)==pin['sha256'] and identity(read(p))==pin['identity']
    exported=read(folder/'background/export/EXPORT_RESULT.json');assert exported==direct['export']==root['export']
    assert all(exported[k] is True for k in ('passed','conversion_fidelity','point_bijection_exact','cell_bijection_exact','all_face_incidence_and_winding_exact'))
    assert exported['checkmesh']['passed'] is True and exported['checkmesh']['literal_mesh_ok'] is True
    # Exact retained accepted NODAL5 passage, with no assumed repair outcome.
    for rel,pin in pre['baseline_pins'].items():assert sha(baseline/rel)==pin
    base=read(baseline/'RESULT.json');base_direct=base['background']
    case=read(folder/'CASE.json');choice=case.pop('verification')
    assert case==read(baseline/'CASE.json') and choice['mode']==mode
    physical=arrays(folder/'background/COMPLETE_MESH.npz',baseline/'background/COMPLETE_MESH.npz')
    report['frozen_arrays']=physical;assert equal_arrays(physical),'PhysicalArraysDifferFromFrozen'
    plan=plans(folder,baseline);report['frozen_plan_arrays']=plan;assert plan and all(plan.values()),'PrismPlanDiffersFromFrozen'
    assert root['requested']==base['requested'] and root['achieved']==base['achieved'],'AchievedScheduleDiffers'
    assert native_behavior(root['native'])==native_behavior(base['native']),'NativePrismBehaviorDiffers'
    assert direct['shape']==base_direct['shape'],'ShapePopulationsDiffer'
    for name in ('INTERFACE_QA','JOINED_QA','PHYSICAL_QA'):
        for rr in (folder,baseline):
            assert read(rr/'background/audit'/(name+'.json'))['geometry_identity']==read(rr/'background/RESULT.json')['proof']['geometry_identity'],'QualityStampNotOwnBound'
        assert without_stamp(read(folder/'background/audit'/(name+'.json')))==without_stamp(read(baseline/'background/audit'/(name+'.json'))),'QualityDiffers:'+name
    for key in ('native_calls','existing_vertex_commits','cavity_commits','initial_bad_count','final_bad_count','outcome'):
        assert direct['conditioning'][key]==base_direct['conditioning'][key],key
    for key in ('changed_count','changed_vertices','repair_invocations','original_coordinate_identity','coordinate_identity'):
        assert direct['repair'][key]==base_direct['repair'][key],key
    report['apex_state_vs_frozen']=apex_state(folder,baseline)
    assert exported['checkmesh']['warnings']==base_direct['export']['checkmesh']['warnings'],'ChangedNativeWarnings'
    if mode=='production':
        audit=Path(pre['outputs']['audit']);candidate_arrays=arrays(folder/'background/COMPLETE_MESH.npz',audit/'background/COMPLETE_MESH.npz')
        report['candidate_audit_arrays']=candidate_arrays;assert equal_arrays(candidate_arrays),'PhysicalArraysDifferFromCandidateAudit'
        audit_root=read(audit/'RESULT.json')
        assert root['requested']==audit_root['requested'] and root['achieved']==audit_root['achieved'] and native_behavior(root['native'])==native_behavior(audit_root['native'])
        assert direct['shape']==audit_root['background']['shape']
        report['apex_state_vs_candidate_audit']=apex_state(folder,audit)
    report['scratch_storage_aggregate']={str(rr):read(rr/'upstream/surface/A/fresh/mmgs.json')['measured_input_bytes'] for rr in (folder,Path(pre['outputs']['audit']),baseline)}
    report['scratch_storage_meaning']='Sum of every direct scratch file before MMGS launch; not metric input bytes. Exact contributors to the three-byte delta were not retained.'
    _held.verify()
    report.update(input_custody='Comparison-wide immutable handles; final path/population inventory unchanged')
    report.update(passed=True,verification_sha256=sha(folder/'VERIFICATION.json'),source_closure='closed_passed',
        proof_count=len(required),native_prism_report=root['native'],points=direct['points'],prisms=direct['prism_cells'],tetrahedra=direct['core_cells'],
        shape=direct['shape'],requested=root['requested'],compressed_vertices=root['achieved']['compressed_vertices'],
        warnings=exported['checkmesh']['warnings'],conditioning=direct['conditioning'],repair=direct['repair'],
        root_stage_seconds=root['times'],direct_stage_seconds=direct['stage_seconds'],joined_predicate_seconds=proof['stage_seconds'],
        worker_result_seconds=root['seconds'],direct_result_seconds=direct['seconds'],supervisor_seconds=supervisor['seconds'],
        external_total_wall_seconds=run['total_wall_seconds'],external_peak_windows_tree_rss_bytes=run['peak_sampled_external_windows_tree_rss_bytes'],
        internal_peak_worker_windows_tree_rss_bytes=supervisor['peak_sampled_windows_tree_rss_bytes'],
        output_logical_bytes=run['output_logical_bytes'],native_export_commands=exported['commands'],
        native_scope_receipts=[str(p.relative_to(folder)) for p in sorted(folder.rglob('*.linux_scope.json'))],
        proof_provenance={k:proof[k] for k in ('generation_invocations','construction_invocations','live_parent')},
        finalizer_cost='Not separately instrumented; bundled in joined_proofs/export and completion stages',
        timing_scope='Root/direct/predicate/native times are nested; do not add them to the nonoverlapping external total')
except BaseException as error:
    report.update(passed=False,error=repr(error),traceback=traceback.format_exc(),production_and_offline_must_not_start=True)
finally:
    if held is not None:
        try:held.close()
        except BaseException as error:report.update(passed=False,error=repr(error),traceback=traceback.format_exc(),production_and_offline_must_not_start=True)
    path=OUT/('PASSAGE_VERIFY3_'+mode.upper()+'_COMPARISON.json');assert not path.exists();path.write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps({k:report[k] for k in ('mode','passed','error','points','prisms','tetrahedra','external_total_wall_seconds') if k in report},indent=2))
raise SystemExit(0 if report['passed'] else 1)

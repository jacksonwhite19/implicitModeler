"""Policy/binding/closure/offline fault controls, not mesh qualification.

Twelve producer and successful replay predicates are synthetic preconditions.
Arrays, finalizer, custody, native export and child handoff use real paths.
"""
from pathlib import Path
from contextlib import ExitStack
from types import SimpleNamespace
import copy,json,shutil,subprocess,sys,tempfile,time,unittest
import uuid
from unittest.mock import patch
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(Path(__file__).parent)]
import geometry
import test_artifact_finalization as controls
import verification_policy as policy
from artifact_finalization import AuditedBundle,expected_arrays,hold_manifest
from sdf_cfd_mesher_v4.common.authority import identity,file_hash,array_hash
from path_preserving_deduplication import durable_json as save
from finite_research.direct_evidence_modules import joined_audit

def production_fixture(root,*,case_extra=None,result_extra=None,root_extra=None,repair_report=None):
    """Actual serialized/native fixture with declared synthetic proof premises."""
    base,_,_,_,exported=controls.export_control();folder=root/'background';shutil.copytree(base,folder)
    for name in ('MANIFEST.json','FINALIZATION.json'): (folder/name).unlink()
    joined,proof=copy.deepcopy(controls.export_control()[1:3])
    from direct_connection import policy as background_policy
    selected=background_policy([[-.01,.01]]*3,.26,.002)
    import numpy as np
    from source_evidence import retain
    from source_handoff import VERSION
    positions=np.asarray(joined['points']).reshape(2,4,3);h1=2e-5
    rows=[dict(passed=True,segments=4,segment_proofs=[{'synthetic_source_precondition':True}]*4,
        start=array_hash(positions[0]),end=array_hash(positions[1]),geometry_identity='synthetic-original-run',
        first_source_normal_height_m=h1,exact_field_progress=True,positive_endpoints=True)]
    evaluation=dict(schema=VERSION,mode='production',method='recomputed',independent_source_recomputation=True,layers=1,segments=4)
    (root/'shell').mkdir();(root/'native').mkdir();np.savez(root/'native/SEALED_PLAN.npz',positions=positions)
    with ExitStack() as scope:retain(root/'shell',evaluation,dict(passed=True,layers=rows),positions,'synthetic-original-run',h1,scope)
    case=dict(background=selected,prisms=dict(first_full_normal_thickness_m_hex=h1.hex()));case.update(case_extra or {});case['verification']=policy.selection('production',case,'complete');save(root/'CASE.json',case)
    binding=dict(case_sha256=file_hash(root/'CASE.json'),policy_identity=identity(selected),source_identity='synthetic-original-run',actual_positions_sha256=array_hash(positions),source_segments_identity=identity(rows))
    joined['provenance']=dict(binding=binding,live_parent=True,generation_invocations=1,construction_invocations=1)
    arrays=expected_arrays(joined)
    parent=dict(binding=binding,prefix_points=joined['prefix_points'],prefix_digest=joined['prefix_digest'],cells_identity=identity(joined['cells']),
        array_hashes={name:array_hash(arrays[name]) for name in ('points','prisms','prism_ids','wall','cap')})
    save(folder/'JOINED_RECONSTRUCTION.json',dict(cell_metadata=[{k:v for k,v in c.items() if k not in ('id','vertices')} for c in joined['cells']],provenance=joined['provenance']))
    stamp=joined_audit.geometry_identity(joined);receipts={}
    for name in joined_audit.REQUIRED:
        value=dict(geometry_identity=stamp,wall_contract=joined_audit.SCHEMA,synthetic_producer_precondition=True)
        path=folder/'audit'/(name.upper()+'.json');save(path,value);receipts[name]=dict(identity=identity(value),sha256=file_hash(path))
    proof=dict(whole_joined_geometry_passed=True,geometry_identity=stamp,proof_receipts=receipts)
    from finite_research.direct_fill_contract import binding as contract,classify,acceptance
    from finite_research.direct_contract import method
    from finite_support.transition_front_targets import population_report
    import numpy as np
    shape={name:classify(population_report(np.array([.9]),selected['quality_targets'][kind]),selected['quality_targets'][kind])
        for name,kind in [('all_bulk_tetrahedra','outer_tetrahedra'),('interface_tetrahedra','outer_tetrahedra'),('preserved_cap_triangles','outer_triangles')]}
    for value in shape.values():value['minimum_actual_count']=1
    result=dict(schema=policy.DIRECT,producer_checks_passed=True,geometry_complete=True,artifact_fidelity_passed=True,independent_audit_status='not_run',acceptance_status='provisional',provisional=True,verification_policy='production',acceptance_profile=policy.profile(),audit='not_run',
        parent=binding,producer_shape_contract=contract(selected),geometry_identity=stamp,proof=proof,shape=shape,export=exported)
    result.update(result_extra or {})
    handoff=dict(binding=binding,method_identity=identity(method()),geometry_identity=stamp,proof=proof,shape=shape,shape_applicability=acceptance(shape,selected),acceptance_contract=contract(selected))
    for name,value in [('POLICY',selected),('PARENT',binding),('PARENT_EXPECTATIONS',parent),('PROVISIONAL',result),('EVIDENCE_HANDOFF',handoff)]:save(folder/(name+'.json'),value)
    if repair_report is not None:save(folder/'SHARED_APEX_REPAIR.json',repair_report)
    args=(joined,proof,selected,parent,binding,shape,exported,result,handoff,root/'CASE.json')
    with ExitStack() as scope:
        bundle=AuditedBundle(folder,joined,proof,scope,lambda:None,verification='production')
        bundle.bind_complete();bundle.accept_export(exported);bundle.bind_production(*args);descriptor=bundle.seal(joined,proof)
        direct=policy.complete_after_finalization(folder,result,descriptor,lambda:10)
        direct.update(manifest_sha256=descriptor['manifest_sha256'],mesh_sha256=file_hash(folder/'COMPLETE_MESH.npz'),seconds=0,stage_seconds={})
        save(folder/'RESULT.json',direct)
        save(root/'RESULT.json',dict(schema=policy.ROOT_RESULT,producer_checks_passed=True,geometry_complete=True,artifact_fidelity_passed=True,independent_audit_status='not_run',acceptance_status='pending_supervisor',verification_policy='production',acceptance_profile=policy.profile(),audit='not_run',background=direct,export=exported,**(root_extra or {})))
    return descriptor,args

HELPER='''from pathlib import Path
from contextlib import ExitStack
import sys,time
sys.path.insert(0,sys.argv[1])
import geometry
from artifact_finalization import hold_manifest
from audit_completion import worker_handoff,read
folder=Path(sys.argv[2]);d=read(folder/'background/RESULT.json')['artifact_finalization']
if len(sys.argv)>3 and sys.argv[3]=='source_added':Path(sys.argv[4]).write_text('{"synthetic":"added"}')
with ExitStack() as scope:
 hold_manifest(folder/'background',d,scope)
 worker_handoff(folder,d,time.monotonic()+20,scope)
 if len(sys.argv)>3 and sys.argv[3]=='fail':raise SystemExit(1)
'''

class VerificationPolicy(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.root=Path(self.tmp.name);self.addCleanup(self.tmp.cleanup)

    def fixture(self):
        root=self.root/'bundle';root.mkdir();descriptor,args=production_fixture(root);return root,descriptor,args

    def test_cli_default_audit_and_explicit_modes_independent_of_carrier(self):
        import mesh
        base=['mesh.py','--case','case','--geometry','shape','--metres-per-model-unit','.001','--prism-first-height','.00002','--prism-growth','1.3','--prism-layers','5','--output','fresh']
        for mode in (None,'audit','production'):
            for carrier in ('adaptive','nodal'):
                with patch.object(sys,'argv',base+['--surface-carrier',carrier]+([] if mode is None else ['--verification',mode])),patch.object(mesh,'supervise',return_value=0) as call:
                    mesh.main();self.assertEqual(call.call_args.args[0].verification,mode or 'audit');self.assertEqual(call.call_args.args[0].surface_carrier,carrier)

    def test_unsupported_production_refuses_before_surface(self):
        import mesh
        args=SimpleNamespace(output=self.root/'out',case=ROOT/'fixtures/passage_historical_case.json',geometry=ROOT/'fixtures/passage.rhai',
            prism_first_height=2e-5,prism_growth=1.3,prism_layers=5,metres_per_model_unit=.001,surface_size=None,surface_policy=None,
            surface_normal_angle_deg=None,surface_position_error=None,background=ROOT/'fixtures/background_direct_v4.json',stop_after='prisms',verification='production')
        with patch('mesh.surface') as surface,self.assertRaisesRegex(ValueError,'ProductionRequiresCompleteDirect'):mesh.execute(args)
        surface.assert_not_called();self.assertFalse((args.output/'upstream').exists())
        for settings,stop in [({},'complete'),({'background':{'schema':'unknown'}},'complete')]:
            with self.assertRaises(ValueError):policy.selection('production',settings,stop)

    def test_real_case_loader_legacy_audit_production_and_unknown_extension(self):
        from geometry import write_case
        from planner import Request
        from standalone_case import load
        path=self.root/'CASE.json'
        case=write_case(ROOT/'fixtures/passage_historical_case.json',ROOT/'fixtures/passage.rhai',path,Request(2e-5,1.3,5))
        case['background']=json.loads((ROOT/'fixtures/background_direct_v4.json').read_text())
        save(path,case);load(path)  # Legacy field set remains supported.
        for mode in ('audit','production'):
            current=copy.deepcopy(case);current['verification']=policy.selection(mode,current,'complete');save(path,current)
            actual,_,pin=load(path);self.assertEqual(actual['verification']['mode'],mode);self.assertEqual(pin,file_hash(path))
        current['verification']['profile']['version']='unknown';save(path,current)
        with self.assertRaises(ValueError):load(path)

    def test_only_audit_invokes_replay_and_failed_audit_never_downgrades(self):
        root,d,args=self.fixture()
        with patch('artifact_finalization.run_independent_audit',side_effect=AssertionError('production must skip only this call')) as replay:
            result=policy.complete_after_finalization(root/'background',args[7],d,lambda:10)
            self.assertEqual(result['audit'],'not_run');replay.assert_not_called();self.assertNotIn('full_mesh_complete',result)
            self.assertNotIn('passed',result)
            with self.assertRaises(ValueError):policy.production_result(dict(result,passed=True))
        audit=dict(d,verification_policy='audit')
        for failure in (ValueError('audit failure'),TimeoutError('audit timeout')):
            with patch('artifact_finalization.run_independent_audit',side_effect=failure) as replay,self.assertRaises(type(failure)):
                policy.complete_after_finalization(root/'background',dict(schema='V35V4DirectComplete/1'),audit,lambda:10)
            replay.assert_called_once()

    def test_production_bridge_required_and_parent_policy_witness_faults_refuse(self):
        root,d,args=self.fixture();folder=root/'background'
        for name in ('MANIFEST.json','FINALIZATION.json'): (folder/name).unlink()
        with ExitStack() as scope:
            b=AuditedBundle(folder,args[0],args[1],scope,lambda:None,verification='production');b.bind_complete();b.accept_export(args[6])
            with self.assertRaisesRegex(ValueError,'IdentityBridgeRequired'):b.seal(args[0],args[1])
            for index,key,value in [(3,'cells_identity','0'*64),(3,'array_hashes',{}),(4,'policy_identity','bad'),(7,'proof',{})]:
                changed=list(copy.deepcopy(args));changed[index][key]=value
                with self.assertRaises(ValueError):b.bind_production(*changed)
            b.bind_production(*args)
            b.seal(args[0],args[1])
            with self.assertRaises(PermissionError):(folder/'PARENT.json').write_text('{}')

    def acquire(self,failed=False):
        from audit_completion import SupervisorHandoff,REQUEST
        root,d,args=self.fixture();helper=self.root/'helper.py';helper.write_text(HELPER)
        proc=subprocess.Popen([sys.executable,'-B',str(helper),str(ROOT),str(root)]+(['fail'] if failed else []))
        self.addCleanup(lambda:proc.kill() if proc.poll() is None else None)
        deadline=time.monotonic()+15
        while not (root/REQUEST).exists():
            if proc.poll() is not None or time.monotonic()>deadline:self.fail('no production handoff')
            time.sleep(.02)
        handoff=SupervisorHandoff(root,proc,verification='production');self.addCleanup(handoff.close);proc.wait(timeout=15)
        sources=dict(identity='synthetic-original-runtime',state='closed',passed=True,changed=[],added=[])
        supervisor=dict(exit_code=proc.returncode,stop_reason=None,monitor_error=None,source_identity=sources['identity'])
        source=self.root/'source.json';record=self.root/'supervisor.json';save(source,sources);save(record,supervisor)
        return root,handoff,(sources,supervisor,source,record)

    def published(self):
        root,handoff,args=self.acquire();value=handoff.publish(*args);handoff.close();return root,value,file_hash(root/'VERIFICATION.json')

    def test_production_overlap_closure_and_no_legacy_success_aliases(self):
        root,handoff,args=self.acquire()
        with self.assertRaises(PermissionError):(root/'background/COMPLETE_MESH.npz').write_bytes(b'late')
        for name in ('shell/SOURCE_EVALUATION.json','shell/SOURCE_SEGMENTS.json','native/SEALED_PLAN.npz'):
            with self.assertRaises(PermissionError):(root/name).write_bytes(b'late')
        bad=dict(args[0],added=['late.py'])
        with self.assertRaises(ValueError):handoff.publish(bad,*args[1:])
        value=handoff.publish(*args);self.assertEqual(value['schema'],policy.MANIFEST);self.assertEqual(value['audit'],'not_run')
        self.assertFalse((root/'background/READBACK.json').exists());self.assertNotIn('full_mesh_complete',json.loads((root/'RESULT.json').read_text()))
        self.assertNotIn('passed',json.loads((root/'RESULT.json').read_text()))

    def test_failed_production_worker_cannot_publish(self):
        root,handoff,args=self.acquire(True)
        with self.assertRaises(ValueError):handoff.publish(*args)
        self.assertFalse((root/'VERIFICATION.json').exists())

    def supervised(self,mode):
        import mesh
        root=self.root/'supervised';helper=self.root/'supervisor_helper.py';helper.write_text(HELPER)
        added=ROOT/('phase1b_source_added_'+uuid.uuid4().hex+'.json')
        original=subprocess.Popen
        def launch(_command,**kwargs):
            root.mkdir();production_fixture(root)
            return original([sys.executable,'-B',str(helper),str(ROOT),str(root),mode,str(added)],**kwargs)
        args=SimpleNamespace(output=root,verification='production',timeout_seconds=30,memory_limit_gib=16,output_limit_bytes=1000000000)
        try:
            with patch('mesh.subprocess',SimpleNamespace(Popen=launch,run=subprocess.run)):code=mesh.supervise(args)
        finally:
            if added.exists():added.unlink()
        return root,code,json.loads((root/'RESULT.json').read_text())

    def test_real_production_supervisor_clean_closure(self):
        root,code,result=self.supervised('normal');self.assertEqual(code,0)
        final=json.loads((root/'VERIFICATION.json').read_text());self.assertEqual(final['acceptance_status'],'production_accepted')
        self.assertEqual(final['execution_record']['checks']['separate_semantic_replay']['status'],'not_run')
        self.assertNotIn('passed',result);self.assertNotIn('full_mesh_complete',result)

    def test_real_production_supervisor_failed_worker_revokes(self):
        root,code,result=self.supervised('fail');self.assertNotEqual(code,0);self.assertFalse(result['passed'])
        self.assertFalse((root/'VERIFICATION.json').exists())

    def test_real_production_supervisor_source_change_revokes(self):
        root,code,result=self.supervised('source_added');self.assertNotEqual(code,0);self.assertFalse(result['passed'])
        self.assertEqual(result['stop_reason'],'SourceClosureChanged');self.assertFalse((root/'VERIFICATION.json').exists())

    def test_legacy_reader_refuses_production_manifest(self):
        from direct_readback import verify
        root,d,args=self.fixture()
        with self.assertRaisesRegex(ValueError,'UnsupportedDirectManifest'):verify(root/'background',d['manifest_sha256'])

    def test_offline_real_failure_is_isolated_and_recorded(self):
        from offline_audit import audit,verifier_identity
        root,_,pin=self.published();before={p.relative_to(root).as_posix():file_hash(p) for p in root.rglob('*') if p.is_file()}
        out=self.root/'offline'
        # Real existing replay refuses synthetic proof premises. No fake pass.
        with self.assertRaises(Exception):audit(root,pin,verifier_identity(out),out,20)
        receipt=json.loads((out/'AUDIT_RESULT.json').read_text());self.assertFalse(receipt['passed']);self.assertEqual(receipt['status'],'audit_failed')
        self.assertEqual(before,{p.relative_to(root).as_posix():file_hash(p) for p in root.rglob('*') if p.is_file()})

    def test_offline_actual_reader_routes_conditioning_diagnostics_and_holds_source(self):
        from offline_audit import audit,verifier_identity
        from finite_research import direct_readback
        import background_conditioning
        root,_,pin=self.published();out=self.root/'offline'
        def semantics(folder,parent,selected,helpers,construction,check,output):
            self.assertEqual(output,out/'replay');self.assertEqual(background_conditioning._replay_output.get(),out/'conditioning')
            with self.assertRaises(PermissionError):(folder/'JOINED_EVIDENCE.npz').write_bytes(b'late')
            output.mkdir();(output/'TEST_ONLY.json').write_text('{}')
            raise ValueError('test stop before synthetic geometric predicates')
        with patch.object(direct_readback,'semantics',side_effect=semantics) as replay,self.assertRaisesRegex(ValueError,'test stop'):
            audit(root,pin,verifier_identity(out),out)
        replay.assert_called_once();self.assertIsNone(background_conditioning._replay_output.get())
        self.assertTrue((out/'replay/TEST_ONLY.json').exists());self.assertFalse((root/'background/conditioning_readback').exists())

    def test_offline_auth_tamper_missing_witness_version_and_output_refusal(self):
        from offline_audit import audit,authenticate,verifier_identity
        root,value,pin=self.published()
        for out in (root/'offline',root,self.root):
            with self.assertRaises(ValueError):audit(root,pin,'unknown',out)
        out=self.root/'unsupported'
        with self.assertRaisesRegex(ValueError,'UnsupportedVerifierIdentity'):audit(root,pin,'unknown',out)
        with self.assertRaisesRegex(ValueError,'FreshOffline'):audit(root,pin,'unknown',out)
        for mutation in ('root','version','missing','artifact','source_method','source_population','source_missing','source_operands'):
            dest=self.root/mutation;shutil.copytree(root,dest)
            expected=pin
            if mutation=='version':
                v=copy.deepcopy(value);v['acceptance_profile']['version']='unknown';save(dest/'VERIFICATION.json',v);expected=file_hash(dest/'VERIFICATION.json')
            if mutation=='missing':(dest/'background/audit/DOMAIN.json').unlink()
            if mutation=='artifact':(dest/'background/COMPLETE_MESH.npz').write_bytes(b'changed')
            if mutation=='root':expected='0'*64
            if mutation=='source_method':
                p=dest/'shell/SOURCE_EVALUATION.json';v=json.loads(p.read_text());v['method']='historical_cache';save(p,v)
            if mutation=='source_population':(dest/'shell/SOURCE_SEGMENTS.json').write_text('{}')
            if mutation=='source_missing':(dest/'shell/SOURCE_SEGMENTS.json').unlink()
            if mutation=='source_operands':(dest/'native/SEALED_PLAN.npz').write_bytes(b'changed')
            with ExitStack() as scope,self.assertRaises(Exception):authenticate(dest,expected,scope)

    def test_offline_success_wrapper_never_claims_fresh_source_authority(self):
        from offline_audit import audit,verifier_identity
        root,_,pin=self.published();out=self.root/'offline'
        # Synthetic replay success tests wrapper recording only, not equivalence.
        with patch('direct_readback._verify',return_value=dict(passed=True,all_twelve_semantically_replayed=True)):
            receipt=audit(root,pin,verifier_identity(out),out)
        self.assertTrue(receipt['passed']);self.assertFalse(receipt['fresh_source_authority']);self.assertFalse(receipt['remeshed'])

if __name__=='__main__':unittest.main()

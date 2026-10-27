"""Bounded integrity tests before the scientific clock starts."""
from task4l_common import *
import inspect,unittest

def run():
    initial=read(OUT/'PREFLIGHT_INITIAL.json')
    assert git('rev-parse','HEAD')=='894424c7e026bcf4b64f6e1480298d3d232da240'
    assert git('branch','--show-current')=='feature/sdf-cfd-mesher-v4-advancing-front'
    assert not git('diff','--cached','--name-only')
    os.environ['TASK4_OUTPUT_DIRECTORY']=str(RUN.resolve())
    from task4l_authority import audit,REPAIRS
    frozen=audit({p:file_hash(ROOT/p) for p in REPAIRS})
    import task4_prepare_next as prepare
    import task4_certify_candidate as certify
    from sdf_cfd_mesher_v4.forecast import exact_dyadic
    from sdf_cfd_mesher_v4.forecast.detailed.cache import DetailedCache
    from sdf_cfd_mesher_v4.forecast.detailed.dependency_metrics import LocalMetricEngine
    from sdf_cfd_mesher_v4.advancing_front.persistent_planning import PersistentPlanner
    from sdf_cfd_mesher_v4.certificate.future_proofs import FutureSpotProofs
    from sdf_cfd_mesher_v4.geometry.front import load_sf2_wall,build_topology
    from sdf_cfd_mesher_v4.forecast.schedule import load_schedule
    from sdf_cfd_mesher_v4.advancing_front.final_validation_reuse import final_receipt_manifest
    front=load_sf2_wall();top=build_topology(front.triangles,len(front.coordinates));schedule=load_schedule()
    assert front.coordinate_hash=='3412cc0b84cc4ac092ad2cd89698cb9857ddb8428f3200db1e6d2e0ef12e7c94'
    assert top.sha256=='69477add150348f567e454cb2216a913124adecc6ec11c47f58471c0931868e7'
    assert front.geometry_authority=='f453b7430703e9ae2c4938cd10f600c565d3fb7a0ce6f8e22e1161561326d1a3'
    assert front.surface_authority=='dc390bedc2198cb315e15961f848cd67b47641c30ae4ec0b96944996d1f8196f'
    assert len(schedule.targets)==20 and schedule.targets[-1]==0.020000000131704564
    assert certify.certify_immediate.__module__=='sdf_cfd_mesher_v4.certificate.production'
    assert certify.exact_wedge is exact_dyadic.exact_wedge
    assert isinstance(DetailedCache().engine,LocalMetricEngine)
    assert isinstance(PersistentPlanner().spots.proofs,FutureSpotProofs)
    from sdf_cfd_mesher_v4.certificate import efficient_proof,production
    assert 'memoized_cad_algebra' in inspect.getsource(efficient_proof.exact_batch)
    assert 'cad_algebra_cache.py' in inspect.getsource(production.source_identity)
    assert 'parallel_screen' in inspect.getsource(efficient_proof)
    assert 'compact_storage' in inspect.getsource(production)
    sig=inspect.signature(prepare.prepare);assert sig.parameters['use_atlas'].default and sig.parameters['use_persistent'].default
    assert 'immediate_cell_screen' in inspect.getsource(prepare.prepare)
    assert not (RUN/'INITIAL_FORECAST.npz').exists() and not (RUN/'build_A/CURRENT.json').exists()
    sealed=dict(initial['sealed_evidence'])
    # Immutable evidence only, not historical candidate inputs to construction.
    for task in ('task4e','task4g','task4h'):
        a=read(ROOT/f'results/{task}/AUTHORITY.json')
        for p,h in a['evidence_sha256'].items():assert file_hash(ROOT/p)==h,p;sealed[p]=h
        sealed[f'results/{task}/AUTHORITY.json']=file_hash(ROOT/f'results/{task}/AUTHORITY.json')
        sealed[f'results/{task}/report.md']=file_hash(ROOT/f'results/{task}/report.md')
    save('AUTHORITY_VERIFICATION.json',dict(frozen=frozen,V3=initial['V3'],sealed=sealed,verified=True))
    RUN.mkdir(exist_ok=True)
    save('fresh/PREFLIGHT.json',dict(starting_head=initial['head'],starting_status=initial['starting_status'],started_utc=initial['task_start']))
    sys.path.insert(0,str(ROOT/'tests'))
    modules=['test_task4l_governance','test_task4g_evaluation','test_task4h_exact_geometry','test_production_certificate_adapter','test_dynamic_risk_atlas','test_persistent_planning']
    suite=unittest.TestSuite(unittest.defaultTestLoader.loadTestsFromName(n) for n in modules);tick=time.perf_counter()
    with (OUT/'SMOKE_TESTS.txt').open('w',encoding='utf-8') as stream:result=unittest.TextTestRunner(stream=stream,verbosity=2).run(suite)
    save('SMOKE_TESTS.json',dict(count=result.testsRun,failures=len(result.failures),errors=len(result.errors),passed=result.wasSuccessful(),seconds=time.perf_counter()-tick,modules=modules))
    assert result.wasSuccessful(),'SmokeFailure'
    env=dict(os.environ,PYTHONPATH=os.pathsep.join([str(ROOT/'src'),str(ROOT/'tools'),str(ROOT/'tests')]))
    probe=subprocess.run([sys.executable,'-B','-m','unittest','-v','test_task4k_cad_cache'],env=env,capture_output=True,text=True)
    (OUT/'CAD_SMOKE_TESTS.txt').write_bytes((probe.stdout+probe.stderr).encode())
    assert probe.returncode==0,'CADSmokeFailure'
    save('CAD_SMOKE_TESTS.json',dict(passed=True,count=6,module='test_task4k_cad_cache',isolated_process=True))
    repairs=[]
    for p in sorted(REPAIRS):
        before=subprocess.check_output(['git','show','HEAD:'+str((ROOT/p).relative_to(REPO)).replace('\\','/')],cwd=REPO)
        repairs.append(dict(path=p,before_sha256=hashlib.sha256(before).hexdigest(),after_sha256=file_hash(ROOT/p)))
    paths=[p for base in ('src','tools','tests') for p in (ROOT/base).rglob('*.py')]
    save('RUNNER_CONFIGURATION.json',dict(fresh_from_wall=True,reference_inputs=False,reference_use='Post-completion comparator only',
        certificate=certify.certify_immediate.__module__,certificate_workers=8,atlas=True,persistent_planner=True,dependency_metrics=True,
        staged_rejection=True,exact_dyadic_records=True,indexed_future_selection=True,future_input_proofs=True,immutable_validation_receipts=True,cad_exact_cache=True,parallel_conservative_screening=True,compact_proof_storage=True,
        schedule_hash=array_hash(schedule.targets),wall_hash=front.coordinate_hash,topology=top.sha256,repairs=repairs,
        source_hashes={p.relative_to(ROOT).as_posix():file_hash(p) for p in paths}))
    print('PREFLIGHT_PASSED',result.testsRun,flush=True)
if __name__=='__main__':run()

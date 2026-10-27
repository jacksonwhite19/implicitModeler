"""Frozen-source verification and focused timed-run integrity smoke checks."""
from task4e_common import *
import inspect,unittest

def run():
    assert git('rev-parse','HEAD')==clock()['start_head']=='ed5a4fc0df768e029938aa57ca7df866bcdf4173'
    assert git('branch','--show-current')=='feature/sdf-cfd-mesher-v4-advancing-front'
    assert not git('diff','--cached','--name-only')
    sources={};sealed={};counts={}
    for name in ('task4','task4b','task4c','task4d'):
        p=ROOT/'results'/name;seal=read(p/'SEAL.json')
        for file,key in [('report.md','report_sha256'),('AUTHORITY.json','authority_sha256')]:
            assert file_hash(p/file)==seal[key],(name,file)
            sealed[(p/file).relative_to(ROOT).as_posix()]=seal[key]
        a=read(p/'AUTHORITY.json');sources.update(a['source_sha256'])
        for path,h in a['evidence_sha256'].items():assert file_hash(ROOT/path)==h,path;sealed[path]=h
        counts[name]=len(a['evidence_sha256'])
    allowed={'tools/task4_initial_scan.py','tools/task4_warning_cache.py'};repairs=[]
    for path,h in sources.items():
        actual=file_hash(ROOT/path)
        if actual!=h:
            assert path in allowed,path
            previous=subprocess.check_output(['git','show','HEAD:'+str((ROOT/path).relative_to(REPO)).replace('\\','/')],cwd=REPO)
            assert hashlib.sha256(previous).hexdigest()==h
            repairs.append(dict(path=path,before=h,after=actual))
    from task4_preflight import audit
    from task4d_common import bounded_v3
    prior=audit();v3=bounded_v3()
    # Config imports do not generate geometry. Select the separate output first.
    os.environ['TASK4_OUTPUT_DIRECTORY']=str(RUN)
    import task4_prepare_next as prepare
    from task4_certify_candidate import certify_immediate
    from sdf_cfd_mesher_v4.advancing_front.persistent_planning import VERSION
    from sdf_cfd_mesher_v4.geometry.front import load_sf2_wall,build_topology
    from sdf_cfd_mesher_v4.forecast.schedule import load_schedule
    f=load_sf2_wall();top=build_topology(f.triangles,len(f.coordinates));schedule=load_schedule()
    assert f.coordinate_hash=='3412cc0b84cc4ac092ad2cd89698cb9857ddb8428f3200db1e6d2e0ef12e7c94'
    assert top.sha256=='69477add150348f567e454cb2216a913124adecc6ec11c47f58471c0931868e7'
    assert len(schedule.targets)==20 and schedule.targets[-1]==0.020000000131704564
    assert certify_immediate.__module__=='sdf_cfd_mesher_v4.certificate.production'
    sig=inspect.signature(prepare.prepare);assert sig.parameters['use_atlas'].default and sig.parameters['use_persistent'].default
    assert not (RUN/'INITIAL_FORECAST.npz').exists() and not (RUN/'build_A/CURRENT.json').exists()
    final=read(BASE/'FINAL_VALIDATION.json')
    assert final['shell_scientific_hash']=='54a69dae10d792aeee02d945fcd095f053990a10db695e97626ab540ae24e51c'
    save('PREFLIGHT.json',dict(elapsed_seconds=elapsed(),sealed=sealed,source_expected=sources,authorized_repairs=repairs,V3=v3,
        Build_A_checkpoint=file_hash(BASE/'build_A/CURRENT.json'),historical_A2_checkpoint=file_hash(BASE/'replay_A2/build_A/CURRENT.json'),
        prior=prior,counts=counts,starting_status=clock()['status'],verified_shell_hash=final['shell_scientific_hash'],
        prompt_hash_note='Prompt adds a 65th character. Frozen 64-character authority is verified. Qualified proof-format bridge compares exact science and reports raw hashes separately.'))
    save('RUNNER_CONFIGURATION.json',dict(fresh_from_wall=True,output=str(RUN.relative_to(ROOT)),certificate=certify_immediate.__module__,
        atlas_enabled=True,persistent_planning_enabled=True,planner_version=VERSION,certificate_workers=8,
        canonical_trials=True,Build_A_used_only_by_post_commit_comparator=True,frozen_schedule_hash=array_hash(schedule.targets),
        source_hashes={p:file_hash(ROOT/p) for p in sorted(set(sources)|allowed)},
        initial_warning_repair='Pass the already qualified persistent pair selector to initial warning discovery; no risk or proof-math change.',
        time_policy=clock()))
    RUN.mkdir(exist_ok=True)
    (RUN/'PREFLIGHT.json').write_bytes((json.dumps(dict(starting_head=clock()['start_head'],starting_status=clock()['status'],started_utc=clock()['task_start']),indent=2)+'\n').encode())
    sys.path.insert(0,str(ROOT/'tests'));suite=unittest.TestSuite()
    names=['test_rolling','test_production_certificate_adapter','test_dynamic_risk_atlas','test_persistent_planning','test_certificate_replay_equivalence','test_task4e_governance']
    for name in names:suite.addTests(unittest.defaultTestLoader.loadTestsFromName(name))
    tick=time.perf_counter()
    with (OUT/'SMOKE_TESTS.txt').open('w',encoding='utf-8',newline='\n') as stream:result=unittest.TextTestRunner(stream=stream,verbosity=2).run(suite)
    save('SMOKE_TESTS.json',dict(count=result.testsRun,failures=len(result.failures),errors=len(result.errors),passed=result.wasSuccessful(),seconds=time.perf_counter()-tick,modules=names))
    assert result.wasSuccessful(),'RunnerSmokeFailure'
    print('STARTUP VERIFIED',elapsed(),result.testsRun,flush=True)

if __name__=='__main__':run()

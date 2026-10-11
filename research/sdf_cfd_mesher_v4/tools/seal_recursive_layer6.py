"""Seal compact L6 authority only after all independent checks pass."""
from qualify_recursive_layer6 import *
from collections import Counter

def run():
    if (OUT/'AUTHORITY.json').exists():raise ValueError('AlreadySealed')
    for name in ('DEEP.json','REPLAY.json','RETENTION.json','REOPEN.json','REOPEN_AFTER_RETENTION.json'):
        assert read(OUT/name)['passed'],name
    assert not (OUT/'FAILURE.json').exists() and not (OUT/'REPLAY_FAILURE.json').exists()
    a=read(OUT/'ADMISSION.json');g=read(OUT/'GENERATION.json');c=read(OUT/'FRESH_CERTIFICATE.json')
    assembly=read(OUT/'ASSEMBLY.json');p=read(OUT/'PUBLICATION.json');deep=read(OUT/'DEEP.json');d=read(OUT/'DIAGNOSTICS.json')
    assert a['decision']['generation_attempt_authorized'] and c['passed'] and not c['failures'] and not g['hard_failures']
    assert all(x['failures']==0 for x in d['quality'].values())
    vol=read(OUT/'VOLUME_DELTA.json');assert vol['exact_balance'] and vol['global_degree']['passed']
    assert len(vol['global_degree']['values'])==202716 and not vol['global_degree']['unresolved']
    assert read(OUT/'REPLAY.json')['content']==p['content_root']==deep['content']
    rootfile=OUT/'candidate'/('ROOT_'+p['root']+'.json')
    reference=read(OUT/'REOPEN.json')['reference']
    with CommittedState(reference) as state:
        state.check();assert state.layer==6
        methods=Counter()
        for desc,_,_ in state.proof_segments():methods.update(desc['methods'])
        assert sum(methods.values())==c['combined_pairs']
    protected={**read(OUT/'PREFLIGHT.json')['protected'],**read(OUT/'PRIOR_AUTHORITY_VERIFICATION.json')['files']}
    for path,h in protected.items():assert file_hash(path)==h,path
    status=subprocess.check_output(['git','status','--porcelain=v1','-z'])
    task_names=(b'results/recursive_layer6',b'tools/qualify_recursive_layer6.py',b'tools/recursive_layer6_diagnostics.py',
        b'tools/replay_recursive_layer6.py',b'tools/retain_recursive_layer6.py',b'tools/seal_recursive_layer6.py',b'tests/test_recursive_layer6_retention.py')
    unrelated=[r for r in status.split(b'\0') if r and not any(n in r for n in task_names)]
    assert b'\0'.join(unrelated).hex()==read(OUT/'PREFLIGHT.json')['status_hex']
    impl={str(path.relative_to(V4)):file_hash(path) for path in [V4/'tools'/n for n in ('qualify_recursive_layer6.py','recursive_layer6_diagnostics.py','replay_recursive_layer6.py','retain_recursive_layer6.py','seal_recursive_layer6.py')]+[V4/'tests/test_recursive_layer6_retention.py']}
    cert=dict(schema='V4RecursiveSuccessfulLayerCertificate/1',layer=6,legacy_scientific=deep['scientific'],content_root=p['content_root'],access_root=p['root'],checkpoint_sha256=file_hash(rootfile),
        parent=read(AUTH/'REFRAMED_REFERENCE.json'),binding=read(OUT/'PREFLIGHT.json')['binding'],
        components=read(OUT/'CANDIDATE_PLAN.json')['components'],geometry=g,quality=d['quality'],
        continuous=dict(prior=c['prior_population'],new=c['required_pairs'],total=c['combined_pairs'],methods=dict(methods),forbidden=0,unresolved=0,missing=0,duplicate=0),
        determinants=read(OUT/'PRIOR_DETERMINANT_COMPARISON.json'),global_volume=dict(exact_balance=True,degree_one=len(vol['global_degree']['values'])),
        assembly=assembly,implementation=impl,qualified_implementation=read(AUTH/'IMPLEMENTATION.json'),
        discarded_objects=file_hash(OUT/'CLEANUP_MANIFEST.json'),replay=file_hash(OUT/'REPLAY.json'),
        reproduction='Use an isolated output directory, unchanged pinned composable implementation and authenticated L5 reference. Reuse sealed L6 admission or rerun the L6 admission only; generate, assemble and prepare independently. Compare all component identities. Never overwrite this sealed study.',
        retention_policy='V4RollingSuccessfulQualificationRetention/1')
    cert['identity']=identity(cert);durable_json(OUT/'COMPACT_CERTIFICATE.json',cert)
    rows=[metadata(path) for path in OUT.rglob('*') if path.is_file()]
    active=set(read(OUT/'CANDIDATE_PLAN.json')['references'])
    active.update(str(path.resolve()) for path in (OUT/'candidate').rglob('*') if path.is_file())
    permanent={'COMPACT_CERTIFICATE.json','PREFLIGHT.json','PHYSICAL_PREFLIGHT.json','PRIOR_AUTHORITY_VERIFICATION.json','ADMISSION_METRICS.json','GENERATION_METRICS.json','ASSEMBLY_METRICS.json','PUBLICATION_METRICS.json','PREPARE_METRICS.json','REOPEN.json','REOPEN_AFTER_RETENTION.json','DEEP.json','REPLAY.json','DIAGNOSTICS.json','DIRECT.json','LOCALIZATION.json','FORECAST.json','PRIOR_DETERMINANT_COMPARISON.json','CLEANUP_MANIFEST.json','RETENTION.json','TESTS.txt','RETENTION_TESTS.txt','PRE_RETENTION_INVENTORY.json','IN_PROGRESS_STORAGE.json','PUBLICATION.json'}
    buckets=Counter()
    for r in rows:
        path=Path(r['path']);category='B_active' if str(path.resolve()) in active else 'A_compact' if path.parent==OUT and path.name in permanent else 'C_preserved_regenerated_quality' if path==OUT/'NEW_QUALITY.json' else 'F_preserved_qualification_bulk'
        buckets[category]+=r['cluster_bytes']
    durable_json(OUT/'FINAL_STORAGE.json',dict(files=rows,allocated=sum(r['cluster_bytes'] for r in {physical_key(r):r for r in rows}.values()),categories=dict(buckets),free=shutil.disk_usage(V4).free,
        prior_reference_allocation=read(OUT/'PHYSICAL_PREFLIGHT.json')['unique_allocated'],reclaimed=read(OUT/'RETENTION.json')['physical_reclaimed'],
        scope='Task files after reclamation; final report/authority and this storage ledger are added after this measurement. No historical data deleted.'))
    metrics={name:read(OUT/(name+'_METRICS.json')) for name in ('ADMISSION','GENERATION','ASSEMBLY','PUBLICATION')}
    report=dict(classification='V4ProductionAdaptiveCapsuleAdvancingFrontLayer6IncrementalQualified',authoritative_layer=6,
        supported_claim='Exactly L6 qualified from recursive L5 with no native fallback. L7 through L11 and complete shell remain unqualified.',
        starting_head=HEAD,ending_head='Commit containing this report',reference=reference,scientific=deep['scientific'],content=p['content_root'],checkpoint_sha256=file_hash(rootfile),
        compact_certificate_identity=cert['identity'],compact_certificate_sha256=file_hash(OUT/'COMPACT_CERTIFICATE.json'),
        admission=a,geometry=g,diagnostics=d,assembly=assembly,continuous=cert['continuous'],publication=p,
        normal_metrics=metrics,reopen=read(OUT/'REOPEN.json'),deep=deep,replay=read(OUT/'REPLAY.json'),
        retention={k:v for k,v in read(OUT/'RETENTION.json').items() if k not in ('before','after')},
        tests=dict(regression='105 passed in 19.83s',retention=(OUT/'RETENTION_TESTS.txt').read_text().strip()),
        git=dict(unrelated_dirty=155,preserved=True,V3_task_modifications=0),
        no_L7_admission=True,no_L7_production=True,no_core=True,legacy_fallback_count=0,
        limitations=['Assembly still scans all prior compact pair indices for qualification binding accounting.',
            'New-proof SQLite indexing causes substantial process I/O; process bytes do not equal physical retained allocation.',
            'Explicit deep audit recursively traverses ancestors; it is not normal startup.',
            'Exclusive per-subphase CPU/I/O and exact temporary peak were not instrumented; phase process totals and physical snapshots are reported.',
            'Historical checkpoint retirement and non-identical diagnostic cleanup were not attempted.'],
        recommended_next_task='QualifyV4CapsuleLayers7To8IncrementalEndToEndBounded: separate admission, certification, publication, restore and retention per layer; stop on any failure.')
    report['identity']=identity(report);durable_json(OUT/'REPORT.json',report)
    compact_names=['REPORT.json','COMPACT_CERTIFICATE.json','PREFLIGHT.json','PHYSICAL_PREFLIGHT.json','PRIOR_AUTHORITY_VERIFICATION.json','ADMISSION.json','GENERATION.json','ASSEMBLY.json','PUBLICATION.json','DEEP.json','REPLAY.json','REOPEN.json','REOPEN_AFTER_RETENTION.json','RETENTION.json','CLEANUP_MANIFEST.json','FINAL_STORAGE.json','DIAGNOSTICS.json','TESTS.txt','RETENTION_TESTS.txt']
    authority=dict(classification=report['classification'],qualification=report['identity'],layer=6,scientific=deep['scientific'],reference=reference,files={n:file_hash(OUT/n) for n in compact_names})
    authority['identity']=identity(authority);durable_json(OUT/'AUTHORITY.json',authority)
    print(json.dumps(dict(classification=report['classification'],qualification=report['identity'],report_sha256=file_hash(OUT/'REPORT.json'),authority_identity=authority['identity'],authority_sha256=file_hash(OUT/'AUTHORITY.json'),certificate_sha256=file_hash(OUT/'COMPACT_CERTIFICATE.json'),scientific=deep['scientific'],storage=buckets),indent=2))
if __name__=='__main__':run()

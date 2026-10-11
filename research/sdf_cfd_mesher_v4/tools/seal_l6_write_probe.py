"""Seal partial measurement evidence; do not qualify production optimization."""
from probe_l6_index_writes import *

def run():
    if (OUT/'AUTHORITY.json').exists():raise ValueError('PreserveSealedStudy')
    b=read(OUT/'BASELINE.json');c=read(OUT/'CACHE_CONTROL.json');pre=read(OUT/'PREFLIGHT.json')
    assert b['oracle_equal'] and c['oracle_equal'] and b['semantic']==c['semantic']
    assert b['file_sha256']==c['file_sha256']
    for key in ('journal_mode','synchronous','temp_store','page_size'):assert b['pragmas'][key]==c['pragmas'][key]
    canonical=SEALED/'objects/new-proof-key-index.sqlite'
    rows=[]
    with access.guard(canonical):
        assert file_hash(canonical)==b['file_sha256']
        for mode,r in [('baseline',b),('cache_control',c)]:
            path=(OUT/(mode+'.sqlite')).resolve();assert path.parent==OUT.resolve()
            assert file_hash(path)==r['file_sha256']
            rows.append(dict(path=str(path),canonical=str(canonical),sha256=r['file_sha256'],metadata=metadata(path),classification='Exact duplicate control output'))
        manifest=dict(rows=rows,reason='Complete key/ordinal comparison and bitwise identity with retained sealed L6 index; no unique database contents')
        durable_json(OUT/'CLEANUP_MANIFEST.json',manifest)
        for row in rows:Path(row['path']).unlink()
        assert file_hash(canonical)==b['file_sha256']
    prior=read(SEALED/'AUTHORITY.json')
    for name,h in prior['files'].items():assert file_hash(SEALED/name)==h
    status=subprocess.check_output(['git','status','--porcelain=v1','-z'])
    owned=(b'probe_l6_index_writes.py',b'seal_l6_write_probe.py',b'test_l6_index_write_probe.py',b'results/l6_write_pair_scaling')
    entries=[x for x in status.split(b'\0') if x and not any(k in x for k in owned)]
    assert b'\0'.join(entries).hex()==pre['status_hex']
    report=dict(classification='V4IncrementalWriteAmplificationMeasuredPairIndexQualificationBlockedByEpoch',
        supported_claim='A sealed-input new-proof-index control isolated cache-spill write amplification and preserved exact index content. No integrated architecture qualification.',
        starting_head=pre['head'],ending_head='Commit containing this report',authoritative_layer=6,production_authority=prior['scientific'],
        checkpoint=pre['checkpoint'],verified_authority=pre['verified_authority'],epoch_blocker=dict(error=pre['blocked'],sealed=pre['sealed_epoch'],current=pre['current_epoch']),
        historical_write_attribution=dict(phases=pre['historical_phases'],total=pre['historical_total_writes'],phase_coverage_bytes=pre['historical_total_writes'],
            historical_per_file_OS_trace=False,remaining_per_file_split_unknown_bytes=pre['historical_total_writes'],
            warning='The isolated replay counters are not assigned retroactively to historical files. Main database versus journal cumulative traffic was not traced individually.'),
        baseline={k:v for k,v in b.items() if k!='samples'},cache_control={k:v for k,v in c.items() if k!='samples'},
        measured_write_reduction=b['construction']['io']['write_bytes']-c['construction']['io']['write_bytes'],
        attribution='Only cache_size changed. The same two unique B-trees, record order, journal mode DELETE, synchronous FULL, integrity check and commit remain. Reduced cache spill explains the measured page-rewrite traffic. File-level main/journal totals remain unmeasured.',
        pair_scan=dict(prior_rows=15842336,changed_binding_pairs=1687674,unchanged_binding_pairs=14154662,changed_face_factors=33786,
            purpose='Count prior pairs incident to changed face factors, then assert summed segment count equals parent live population. Not proof generation, duplicate detection or scientific root construction.',
            normal_path_changed=False,optimized_scan_count=None,
            boundary='Authenticated segment counts can replace the population sum. Exact affected-union counting still needs an authenticated face-to-pair incidence index or equivalent changed-key structure; it is not replaced by counts alone.',
            coverage='New interaction coverage remains a separate global geometry/AABB certification obligation.'),
        unqualified=['Integrated L5-to-L6 scientific equivalence','Changed-state prior-pair accounting','Authenticated summary/root integration',
            'Multi-depth scaling control','Production publication with cache control','Cross-epoch recursive trust recovery','Complete recursive deep audit in current epoch'],
        tests=(OUT/'TESTS.txt').read_text().strip(),
        cleanup=dict(paths=len(rows),logical_removed=sum(r['metadata']['bytes'] for r in rows),physical_reclaimed=sum(r['metadata']['cluster_bytes'] for r in rows),
            manifest_sha256=file_hash(OUT/'CLEANUP_MANIFEST.json'),no_historical_deletion=True),
        free_before=pre['free'],free_after=shutil.disk_usage(V4).free,
        git=dict(unrelated_dirty=155,preserved=True,V3_modifications=0),
        L7_admission=False,L7_forecast=False,L7_generation=False,core=False,
        next_task='QualifyV4RecursiveEpochRecoveryFromSealedLayer6: deeply reauthenticate recursive ancestry into fresh host-epoch access metadata without rewriting historical authorities; then resume this blocked optimization study.')
    report['identity']=identity(report);durable_json(OUT/'REPORT.json',report)
    files={n:file_hash(OUT/n) for n in ('PREFLIGHT.json','BASELINE.json','CACHE_CONTROL.json','TESTS.txt','CLEANUP_MANIFEST.json','REPORT.json')}
    implementation={n:file_hash(V4/n) for n in ('tools/probe_l6_index_writes.py','tools/seal_l6_write_probe.py','tests/test_l6_index_write_probe.py')}
    authority=dict(classification=report['classification'],report_identity=report['identity'],files=files,implementation=implementation,production_optimization_qualified=False)
    authority['identity']=identity(authority);durable_json(OUT/'AUTHORITY.json',authority)
    print(json.dumps(dict(classification=report['classification'],report=report['identity'],report_sha256=file_hash(OUT/'REPORT.json'),authority_sha256=file_hash(OUT/'AUTHORITY.json'),cleanup=report['cleanup'],tests=report['tests']),indent=2))
if __name__=='__main__':run()

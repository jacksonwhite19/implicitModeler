"""Seal a completed one-layer production cycle after replay and durable restore."""
import json,subprocess,time
from xml.etree import ElementTree
from production_single_layer_cycle import OUT,V4,CONFIG,INCREMENT_HEX,SURFACE,REQUEST,save,load_production_wall
from sdf_cfd_mesher_v4.common.authority import file_hash,identity,array_hash


def read(name):return json.loads((OUT/name).read_text())


def run():
    a,b=read('A/FINAL.json'),read('B/FINAL.json');ready=read('A/READY.json')
    assert ready==read('B/READY.json') and a['passed'] and b['passed']
    restores={label:read(label+'/RESTORE.json') for label in ('A','B')}
    assert all(r['passed'] and r['scientific']==ready['committed'] for r in restores.values())
    d=read('A/DIAGNOSTICS.json');cert=read('A/CERTIFICATE.json');admission=read('ADMISSION.json')
    storage=read('STORAGE_FINAL.json');receipt=read('RECEIPT_AUDIT.json');base=read('BASELINE.json')
    status=subprocess.check_output(['git','status','--porcelain=v1'],text=True).splitlines()
    additions=sorted(set(status)-set(base['status']))
    assert not set(base['status'])-set(status)
    assert all(row[3:].startswith('research/sdf_cfd_mesher_v4/') for row in additions)
    assert len(base['status'])==155
    save('GIT_VERIFICATION.json',dict(starting_head=base['head'],head_before_commit=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),
        branch=subprocess.check_output(['git','branch','--show-current'],text=True).strip(),
        unrelated_entries=155,original_entries_preserved=True,V3_task_modifications=0,task_status_additions=additions))
    assert cert['passed'] and not cert['failures'] and storage['old_payloads_preserved']
    assert all(not v['failures'] for scope in d['quality'].values() for v in scope.values())
    tests={}
    for name in ('GENERALIZATION_TESTS.xml','PERSISTENCE_TESTS.xml','RESULT_TESTS.xml'):
        root=ElementTree.parse(OUT/name).getroot()
        tests[name]={k:sum(int(s.get(k,0)) for s in root.iter('testsuite')) for k in ('tests','failures','errors','skipped')}
    assert all(not (r['failures'] or r['errors'] or r['skipped']) for r in tests.values())
    for row in read('INPUT_SEALS.json'):assert file_hash(row['path'])==row['sha256']
    assert file_hash(SURFACE.parent/'GEOMETRY.json')==read('SOURCE_VERIFICATION.json')['geometry_file_sha256']
    assert file_hash(REQUEST)==CONFIG['request_file']
    wall,tri,surface=load_production_wall(SURFACE)
    assert surface['surface_identity']==CONFIG['surface_identity'] and surface['geometry_identity']==CONFIG['geometry_identity']
    assert array_hash(wall)=='2270c352c34bba85dc898da4ea66dda1484123604d96511551899802d16d157c'
    assert array_hash(tri)=='d14f00450f2f67f7b98ab3efa1a7946393b7b793ccd64d4bdcdbf516342de2db'
    persistence=json.loads((V4/'results/immutable_receipt_overlay_delta_publication/AUTHORITY.json').read_text())
    dedup=json.loads((V4/'results/path_preserving_deduplication/AUTHORITY.json').read_text())
    assert persistence['identity']=='cfef66665ae1edabb352ae5cb44701d4a4e67fd41ea82de1d80c7db57917851a'
    assert dedup['qualification_identity']=='1a81990fe1b3cbc7910850f8d6bfc111b7a64d0e9e20c62246834a2ee6bb9480'
    input_ids=dict(CONFIG,persistence=persistence['identity'],deduplication=dedup['qualification_identity'])
    sources={}
    names=['production_single_layer_cycle.py','production_next_layer_diagnostics.py','production_cycle_storage.py',
           'production_cycle_receipt_audit.py','production_cycle_report.py','production_layer2_run.py',
           'production_readonly_admission.py','production_l2_screening.py','production_forecast_admission.py',
           'final_shell_incidence.py','reuse_mixed_certificate.py','native_mixed_face_certificate.py','native_proof_segments.py','validation_row_stream.py',
           'mixed_front_fixture_provider.py','run_geometry_family.py','path_preserving_deduplication.py']
    for name in names:sources['tools/'+name]=file_hash(V4/'tools'/name)
    for name in ('mixed_front_runner.py','mixed_front_journal.py','native_checkpoint.py','native_receipt_delta.py','predict.py'):
        p=V4/'src/sdf_cfd_mesher_v4/advancing_front'/name;sources[p.relative_to(V4).as_posix()]=file_hash(p)
    for name in ('test_production_single_layer_cycle.py','test_production_layer3_evidence.py'):
        sources['tests/'+name]=file_hash(V4/'tests'/name)
    save('IMPLEMENTATION.json',dict(sources=sources,
        changes='Generic next-layer guards, complete historical prefix collision inventory, bound-schedule admission, and isolated one-step study orchestration. Existing meshing mathematics, thresholds, proof semantics and native publication unchanged.',
        snapshot_note='A started before an explicit geometry-ID assertion and B final-object sharing sequence were added to orchestration. Independent SOURCE_VERIFICATION checked the exact geometry before publication. Neither change alters A geometry, certification or publication. Both complete scientific records agree.',
        historical_helpers='Pure generic packing, validation and proof routines reused; no procedural fixture constructor or fixture geometry runs.'))
    # Every locally retained bulk path has an exact content hash, including all
    # checkpoint envelopes/objects. Reports remain small; payloads stay local.
    bulk={r['path']:dict(sha256=r['sha256'],bytes=r['bytes'],file_id=r['file_id'],cluster_bytes=r['cluster_bytes'])
          for r in storage['rows']}
    save('BULK_MANIFEST.json',bulk)
    classification='V4ProductionAdaptiveCapsuleAdvancingFrontLayer3Qualified'
    claim='Exactly L3 from authoritative L2 passed bound read-only admission, final hard/exact/topology/ownership/continuous checks, deterministic complete replay, transactional native publication, and fresh durable restore.'
    scope=dict(committed_layer=3,L4_generated=False,post_L3_L4_forecast=False,core_generated=False,
        complete_AdvancingFrontQualified=False,forecast_layers=list(range(3,12)))
    science=dict(classification=classification,inputs=input_ids,result=ready,scope=scope,
        admission=admission['decision_identity'],diagnostics=file_hash(OUT/'A/DIAGNOSTICS.json'),
        completed_neighbour=d['determinant']['completed_hash'],implementation=sources)
    scientific=identity(science)
    resources=dict(A=a['times'],B=b['times'],A_restore_seconds=a['restore_seconds'],B_restore_seconds=b['restore_seconds'],
        admission_seconds=admission['total_seconds'],initial_inventory_seconds=read('DISK_START.json')['seconds'],
        final_inventory_seconds=storage['seconds'],
        A_wall_seconds_including_replay_wait=a['total_seconds'],B_wall_seconds=b['total_seconds'],
        diagnostics_seconds=d['seconds'],receipt_audit_seconds=receipt['seconds'],
        durable_restores=restores,peak_worker_rss=max(a['peak_rss'],b['peak_rss']),
        retained_rss_A=a['retained_rss'],retained_rss_B=b['retained_rss'],
        A_receipt_publication_seconds=a['receipt_publication_seconds'],B_receipt_publication_seconds=b['receipt_publication_seconds'],
        elapsed_task_seconds=time.time()-base['started'])
    save('RESOURCES.json',resources)
    rows=[classification,'',claim,'',
        'The claim stops at L3. Layers L4-L11, complete shell, core handoff, core and complete mesh remain unqualified.',
        '',f'Starting HEAD: {base["head"]}. Ending HEAD is the commit containing this report (printed after commit).',
        '', 'Input authorities:','```json',json.dumps(input_ids,indent=2),'```',
        '',f'Exact L3 increment: {float.fromhex(INCREMENT_HEX):.17g} m; {INCREMENT_HEX}.',
        '', 'Disk preflight used actual NTFS file IDs and cluster allocation. Starting free space exceeded the measured 23,016,515,631-byte historical L2 retained footprint. This comparison was not asserted to bound L3 peak demand. No guessed reserve was added. No historical evidence was deleted.',
        '', 'Physical accounting covers the historical qualified L1 tree, historical L2 tree, immutable-overlay qualification stores, and this new L3 tree. It does not claim a whole-drive allocation census.',
        '', 'Admission: forecast L3-L11 once from committed L2; direct screen of every prospective L3 prism; complete prefix collision inventory; complete qualified localization. Zero measured risk seeds and zero patches justify the existing conditional detailed-lookahead no-op.',
        '```json',json.dumps(dict(forecast=read('FORECAST.json')['hard_events'],direct=read('DIRECT.json')['counts'],
            localization=read('LOCALIZATION.json')['accounting'],methods=read('LOCALIZATION.json')['methods'],decision=admission['decision']),indent=2),'```',
        '', 'Nominal SDF Newton projection only. No repair, schedule change, remesh or relaxation was triggered. Future schedule numbers remain dependency descriptors; no post-L3 next-front geometry was computed.',
        '', 'The old fixed L1/L2 provider guard and three-front collision inventory were generalized to the bound committed prefix and exactly one next layer. The legacy L2 entry points keep their old guards. The first-layer wall mask remains a physical boundary rule. No 20-layer fixture schedule or procedural geometry enters this cycle. Nonzero-risk admission remains fail-closed in this entry point; this measured case has zero risk.',
        '', 'Distributions use min / p1 / p5 / median / p95 / p99 / max. Units are m, m2, m3 and degrees as applicable. Radius ratio is 2r/R. Percentiles, front quality, nonorthogonality, sigma and G are diagnostics only.']
    quantiles=('min','p1','p5','median','p95','p99','max')
    def table(title,value):
        metrics=[]
        def walk(prefix,v):
            if isinstance(v,dict):
                if all(k in v for k in quantiles):metrics.append((prefix,v))
                else:
                    for k,w in v.items():walk(prefix+'/'+k if prefix else k,w)
        walk('',value)
        rows.extend(['',title,'','| Quantity | min | p1 | p5 | median | p95 | p99 | max |','|---|---:|---:|---:|---:|---:|---:|---:|'])
        for k,v in metrics:rows.append('| '+k+' | '+' | '.join(format(v[p],'.17g') for p in quantiles)+' |')
    table('Directions',d['directions']);table('Height realization',d['height'])
    table('Wall to L3 front evolution',d['fronts']);table('Matched front changes',d['matched'])
    table('Prism quality', {scope:{k:v['distribution'] for k,v in metrics.items()} for scope,metrics in d['quality'].items()})
    rows.extend(['','| Scope/metric | Gate | Failures | Margin | Worst ID/source |','|---|---:|---:|---:|---|'])
    for scope,metrics in d['quality'].items():
        for k,v in metrics.items():
            worst={x:w for x,w in v['worst'].items() if x in ('id','column','owners','vertices')}
            rows.append(f'| {scope}/{k} | {v["gate"]} | {v["failures"]} | {v["margin"]} | {json.dumps(worst,separators=(",",":"))} |')
    table('Completed-neighbour determinant',d['determinant']);table('Sigma/G diagnostics',d['conditioning']);table('Nonorthogonality',d['nonorthogonality'])
    rows.extend(['',d['determinant']['semantics'],
        '', 'L2 terminal and actual-L3-neighbour determinants agree exactly. L1 determinants remain exactly unchanged. No L4 centroid exists.',
        '', 'Topology, ownership and failure counts:','```json',json.dumps(dict(topology=d['topology'],
            fronts={k:{z:v[z] for z in ('zero_area','reversal','nonfinite')} for k,v in d['fronts'].items()},
            direction_counts={k:d['directions'][k] for k in ('zero','nonfinite','inward','fallbacks')},height_nonpositive=d['height_nonpositive']),indent=2),'```',
        '', 'Final realized continuous certificate:','```json',json.dumps(cert,indent=2),'```',
        '', 'Forbidden=0; unresolved=0; missing=0; duplicate=0. The final certificate covers the complete three-layer complex and is independent of provisional collision localization. The volume/degree audit passes.',
        '', 'Provisional and realized coordinates, heights and same-path direct metrics are exactly equal. Thus their matched front evolution is identical. Final scalar validator differences arise from evaluation order; they are reported without a forecast-error threshold. Collision legality agrees for every new interaction.'])
    table('Forecast versus realized absolute differences',{k:v['absolute_difference'] for k,v in d['forecast'].items()})
    rows.extend(['','Persistence and exact receipt audit:','```json',json.dumps(receipt,indent=2),'```',
        '', 'The unchanged native publisher accepted the exact unedited ReceiptOverlay over direct immutable ReceiptDelta. It preserved base/update references. Edited overlays retain the qualified fallback. Stale-base, dependency, atomicity and recovery checks were not changed.',
        '', 'Scientific identity binds the complete restored state. CURRENT binds exact checkpoint container bytes; each descriptor binds object bytes. New checkpoint hashes are not required to equal historical L2 bytes. All historical seals and payload hashes remain valid.',
        '', 'Immutable A objects are hardlinked into B before generation and after A publication. Mutable CURRENT files are independent. The replay runs the complete geometry/certification/transaction again but shares retained immutable checkpoint bulk. Expanded diagnostics remain separately retained and are measured as duplicates.',
        '```json',json.dumps(dict(A_pointer=a['pointer'],B_pointer=b['pointer'],A_materialized=a['materialized'],B_materialized=b['materialized'],ready=ready),indent=2),'```',
        '', 'Storage measurements (allocation excludes MFT metadata; free-space changes are not attributed as allocation):','```json',
        json.dumps(dict(start={k:v for k,v in read('DISK_START.json').items() if k!='rows'},end={k:v for k,v in storage.items() if k!='rows'}),indent=2),'```',
        '', 'The ending storage census precedes its own final inventory, manifest and report files. These small audit records are outside that measurement boundary. Temporary peaks are sampled lower bounds, not exact continuous maxima. No future-layer savings are extrapolated.',
        '', 'Runtime/RSS:','```json',json.dumps(resources,indent=2),'```',
        '', 'Retained RSS is measured at final publication while each worker still holds its state; workers then exit. Phase times are nested and must not be summed as independent costs. A total wall time includes waiting for B replay.',
        '', 'Tests:','```json',json.dumps(tests,indent=2),'```',
        '', 'No test weakening. Existing persistence tests cover edited overlays, stale bases, missing/corrupt objects, interrupted publication, recovery, mutable-file rejection and path-preserving sharing.',
        '', 'Scientific and replay hashes:','```json',json.dumps(science,indent=2),'```',
        '',f'New qualification scientific identity: {scientific}',
        '', 'Git: only task-owned V4 files. The 155 original dirty status entries must match exactly after commit. V3 task changes=0. No historical authority was rewritten.',
        '', 'Remaining concerns: per-layer receipt/dependency rebuild and expanded quality exports still consume time/storage. This task measures them and does not redesign persistence. Full shell qualification remains outstanding.',
        '', 'Recommended next task: QualifyAndCommitV4CapsuleLayers4To5Bounded; certify and commit each layer separately, stop immediately on any failure.'])
    (OUT/'report.md').write_text('\n'.join(rows)+'\n',encoding='utf8',newline='\n')
    evidence={p.relative_to(OUT).as_posix():file_hash(p) for p in OUT.rglob('*') if p.is_file()
              and 'checkpoints' not in p.parts and 'BASE' not in p.parts and p.name not in ('AUTHORITY.json','SEALS.json','REPORT_BUILD.log')}
    save('AUTHORITY.json',dict(classification=classification,scientific_identity=scientific,scientific=science,
        supported_claim=claim,committed_front=ready['committed'],scope=scope,tests=tests,evidence=evidence,blockers=[]))
    save('SEALS.json',dict(report=file_hash(OUT/'report.md'),authority=file_hash(OUT/'AUTHORITY.json'),
        bulk_manifest=file_hash(OUT/'BULK_MANIFEST.json'),scientific=scientific))
    print(json.dumps(read('SEALS.json')),flush=True)


if __name__=='__main__':run()

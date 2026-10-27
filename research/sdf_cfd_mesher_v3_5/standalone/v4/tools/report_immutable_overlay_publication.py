"""Seal measured persistence qualification after every required replay succeeds."""
import collections
import json
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET
from qualify_immutable_overlay_publication import (
    ROOT,V4,OUT,L1,L2,AUDIT,HEAD,L1_ID,L2_ID,SEALS,save,git,unrelated,inventory,file_hash,identity)


def read(name):return json.loads((OUT/name).read_text())


def main():
    publications={k:read(k+'_PUBLICATION.json') for k in ('baseline','candidate_B','candidate_C')}
    restores={k:read(k+'_RESTORE.json') for k in (*publications,'historical_L1','historical_L2_A','historical_L2_B')}
    reference=restores['historical_L2_A']['component_digests']
    for name,r in restores.items():
        assert r['passed']
        if name!='historical_L1':assert r['scientific']==L2_ID and r['component_digests']==reference
        else:assert r['scientific']==L1_ID
    b,a,c=(publications[k] for k in publications)
    assert not b['full_flattening_avoided'] and a['full_flattening_avoided'] and c['full_flattening_avoided']
    assert a['pointer']==c['pointer']
    for r in (a,c):
        assert r['receipt_bytes_written']<b['receipt_bytes_written']
        assert r['retained_logical_bytes']<b['retained_logical_bytes']
        assert r['receipt_counts']['newly_materialized_certificate']==0
    assert read('ROW_COMPARISON.json')['passed']
    tests=ET.parse(OUT/'TESTS_FINAL.xml').getroot()
    suites=list(tests.iter('testsuite'))
    counts={k:sum(int(s.get(k,0)) for s in suites) for k in ('tests','failures','errors','skipped')}
    assert counts['tests']>=51 and counts['failures']==counts['errors']==counts['skipped']==0
    inputs=read('INPUTS.json');base=read('BASELINE.json')
    from production_prism_sizing import read_request
    request=read_request(V4/'results/qualification_prism_progression_convention/REQUEST.json',inputs['surface'])
    assert identity(request['declaration'])==inputs['binding']['request']
    assert request['schedule_identity']==inputs['binding']['schedule']
    save('REQUEST_VERIFICATION.json',dict(request_identity=identity(request['declaration']),
        schedule_identity=request['schedule_identity'],declaration=request['declaration'],source_documents_verified=True))
    assert unrelated()==base['unrelated'] and len(unrelated())==155
    def immutable(rows):return [{k:v for k,v in r.items() if k!='links'} for r in rows]
    assert immutable(inventory(L1))==immutable(inputs['L1'])
    assert immutable(inventory(L2))==immutable(inputs['L2'])
    assert inventory(V4.parent/'sdf_cfd_mesher_v3',tolerate_inaccessible=True)==inputs['V3']
    audited=json.loads((AUDIT/'FILES.json').read_text())
    print('final authority rehash',flush=True)
    for i,r in enumerate(audited):
        assert file_hash(ROOT/r['path'])==r['sha256'],r['path']
        if i%50==0:print('verified',i,'/',len(audited),flush=True)
    for p,h in SEALS.items():assert file_hash(p)==h
    existing_ids={(r['device'],r['inode']) for r in audited}
    existing_hashes={r['sha256'] for r in audited}
    for r in publications.values():
        new=[row for row in r['inventory'] if (row['device'],row['inode']) not in existing_ids]
        r['new_duplicate_payload_vs_historical_bytes']=sum(row['bytes'] for row in new if row['sha256'] in existing_hashes)
        r['historical_payload_shared_by_reference_bytes']=sum(row['bytes'] for row in r['inventory'] if (row['device'],row['inode']) in existing_ids)
    measurements={k:{name:value for name,value in r.items() if name not in ('inventory','certificate')}
                  for k,r in publications.items()}
    save('MEASUREMENTS.json',measurements)
    bulk=[]
    for label,r in publications.items():
        for row in r['inventory']:
            bulk.append(dict(path=label+'/checkpoints/'+row['path'],bytes=row['bytes'],sha256=row['sha256']))
    pilot=read('candidate_A_PUBLICATION.json')
    for row in pilot['inventory']:
        bulk.append(dict(path='candidate_A/checkpoints/'+row['path'],bytes=row['bytes'],sha256=row['sha256']))
    verified_files={}
    for row in bulk:
        path=OUT/row['path'];stat=path.stat();key=(stat.st_dev,stat.st_ino)
        if key not in verified_files:verified_files[key]=file_hash(path)
        assert stat.st_size==row['bytes'] and verified_files[key]==row['sha256'],row['path']
    bulk.append(dict(path='INPUTS.json',bytes=(OUT/'INPUTS.json').stat().st_size,sha256=file_hash(OUT/'INPUTS.json')))
    save('BULK_MANIFEST.json',bulk)
    save('INPUT_AUTHORITIES.json',{k:v for k,v in inputs.items() if k not in ('L1','L2','V3')})
    all_files=inventory(OUT)
    ids={}
    for r in all_files:ids[(r['device'],r['inode'])]=r['bytes']
    disk=dict(before_free_bytes=base['disk_free'],after_free_bytes=shutil.disk_usage(ROOT).free,
        result_logical_bytes_at_seal=sum(r['bytes'] for r in all_files),
        new_unique_file_payload_bytes_at_seal=sum(size for key,size in ids.items() if key not in existing_ids),
        deleted_authority_bytes=0,deleted_replay_bytes=0,
        note='Payload bytes are not filesystem allocation. Free space includes unrelated activity. Standard temporary-file replacement is not historical cleanup.')
    save('DISK_END.json',disk)
    preservation=dict(unrelated_entries=155,status_exact=True,verified_authority_files=len(audited),
        L1_L2_bytes_sizes_mtimes_file_ids_unchanged=True,V3_metadata_unchanged=True,V3_modifications=0,
        V3_inaccessible_entries=sum('inaccessible' in r for r in inputs['V3']),
        link_count_note='Receipt/index hardlink imports add names to immutable files. Source payloads and paths remain intact.')
    save('PRESERVATION.json',preservation)
    source_paths=[V4/'src/sdf_cfd_mesher_v4/advancing_front/native_checkpoint.py',
        V4/'tests/test_immutable_overlay_publication.py',V4/'tools/qualify_immutable_overlay_publication.py',Path(__file__).resolve()]
    source={p.relative_to(ROOT).as_posix():file_hash(p) for p in source_paths}
    save('IMPLEMENTATION.json',source)
    classification='V4ImmutableReceiptOverlayDeltaPublicationQualified'
    claim='The exact untouched legacy ReceiptOverlay-to-ReceiptDelta case restores the sealed L2 scientific state, preserves historical L1/L2 restore, rejects stale/corrupt inputs, and avoids materializing the full L2 certificate table. This is persistence qualification only.'
    lines=[f'# {classification}','',claim,'',
        f'Starting HEAD: `{HEAD}`. Ending commit is the commit containing this report and is printed in the final response.',
        f'Branch: `{base["branch"]}`. Exactly 155 unrelated dirty entries are preserved. V3 modifications: 0.',
        '', '## Authority and identity', '',
        f'L1 committed state: `{L1_ID}`.',f'L2 committed state: `{L2_ID}`.',
        'All 184 audited L1/L2 file hashes passed before replay and after replay. Production surface files passed the authority loader. Request, L1, L2 and audit seals matched.',
        '```json',json.dumps(inputs['seals'],indent=2),'```',
        'Scientific input identities:',
        '```json',json.dumps(dict({k:inputs['binding'][k] for k in ('surface','geometry','request','schedule')},
            wall_sizing=inputs['surface']['sizing_field_identity']),indent=2),'```',
        'The production request loader also verified all five hash-bound source documents and decoded the unchanged 11-layer schedule. This reads the declared request; it does not forecast or construct a future layer.',
        'Scientific state identity covers the complete canonical state, including geometry, source IDs, ownership, quality, certificate proofs, dependency bindings, atlas and committed events. Every restored L2 component digest equals original L2.',
        'Historical authorities and CURRENT descriptors continue to bind exact historical report, checkpoint and object bytes. New CURRENT binds a new container envelope and exact referenced object hashes; it does not replace any historical seal. A changed envelope hash is expected because receipt_table becomes receipt_delta.',
        '', '## Publication architecture and scope', '',
        'Baseline executes the exact starting-HEAD _table method with the unchanged current receipt classes. Both runs reconstruct the existing L2 map from the sealed L1 base and retained update table, then reproduce the untouched journal wrapper. No geometry, forecast, proof generation, dependency rebinding, or new transaction event runs in the production replay.',
        'Baseline flattens all 4,839,083 certificate rows and retains the intermediate update. Candidate uses the existing delta descriptor/import path, then verifies that descriptor through the native receipt verifier before allowing publication.',
        'Qualification rule: exact ReceiptOverlay type; exactly base/edits/removed attributes; empty edits and removals; direct exact ReceiptDelta base; no authenticated attribute on the delta. Subclasses, nested overlays, metadata-bearing wrappers and authenticated formats are outside this optimization.',
        'A row read through __getitem__ populates edits and takes the conservative fallback. Edited overlays retain the existing flat materialization path and exact restored content; they are never unwrapped. Full scientific validation and journal stale-base/concurrent-writer checks remain unchanged.',
        'The replay imports immutable base objects with same-volume hardlinks. Each replay checkpoint has its own complete restore closure. This proves reference publication here, not a cross-volume performance claim or cleanup policy.',
        '', '## Measured storage and time', '',
        '| Measurement | Baseline | Candidate B | Candidate C |','|---|---:|---:|---:|']
    measures=[('Receipt-table bytes written (certificate + quality)','receipt_bytes_written'),
        ('New checkpoint payload bytes written','checkpoint_new_payload_bytes'),
        ('Retained checkpoint logical bytes, including staged update','retained_logical_bytes'),
        ('Byte-identical duplicates within checkpoint','retained_duplicate_logical_bytes'),
        ('New duplicate payload versus historical files','new_duplicate_payload_vs_historical_bytes'),
        ('Historical payload shared by reference','historical_payload_shared_by_reference_bytes'),
        ('Publication seconds','publication_seconds'),('Certificate publication seconds','receipt_publication_seconds'),
        ('Quality publication seconds','quality_publication_seconds'),('Process peak RSS bytes','peak_rss'),
        ('Retained RSS after publication bytes','retained_rss'),('Observed publication OS write bytes','publication_io_write_bytes')]
    for label,key in measures:
        values=[measurements[k][key] for k in publications]
        lines.append('| '+label+' | '+' | '.join(f'{v:.6f}' if isinstance(v,float) else str(v) for v in values)+' |')
    lines+=['','Both cases seed the same 4,858,589,184-byte intermediate object by hardlink before timing. Receipt bytes count actual materialized SQLite files. New checkpoint payload excludes hardlink imports and includes new JSON/envelope/pointer files. OS writes also include temporary SQLite/journal operations.',
        'Peak RSS is the worker high-water mark, including sealed-state loading and representation reconstruction. Retained RSS is measured immediately after publication. Runs are sequential fresh processes; filesystem caches are uncontrolled. These are observed timings, not a statistical speedup guarantee or a future-layer extrapolation.',
        '', '## Receipt counts and restore', '',
        'Base population: 1,940,126. Exact unchanged base receipts reused: 982,678. Delta receipts: 3,856,405 = 957,448 dependency-binding updates + 2,898,957 new interactions. Removed keys: 0. The changed old receipts preserve their proof payloads exactly.',
        'Baseline materializes 4,839,083 certificate rows. Both candidates materialize 0 certificate rows and avoid a complete flat certificate. All runs still materialize the existing quality overlay. No production receipt or dependency value is changed.',
        '| Fresh-process restore | Seconds | Peak RSS bytes | Retained RSS bytes |','|---|---:|---:|---:|']
    for k,r in restores.items():lines.append(f'| {k} | {r["restore_seconds"]:.6f} | {r["peak_rss"]} | {r["retained_rss"]} |')
    lines+=['','Historical L1, original L2 A/B, baseline L2 and both optimized L2 restores passed. Baseline and both candidates match all original L2 component digests. Candidate B/C also have exactly equal checkpoint pointers and byte hashes.',
        'L2 restores contain 50,685 vertices, 67,572 cells, 202,716 faces and 4,839,083 certificate rows. Wall, L1 and L2 coordinate hashes are unchanged:',
        '```json',json.dumps(restores['candidate_B']['coordinates'],indent=2),'```',
        'Restoration rechecks native object bytes and scientific state; it does not rerun geometric qualification. The original 4,839,083/4,839,083 continuous certificates, zero forbidden/unresolved/missing/duplicate findings and hard-quality evidence remain the authority.',
        '', '## Safety and tests', '',
        f'Tests: {counts["tests"]} passed; {counts["failures"]} failures; {counts["errors"]} errors; {counts["skipped"]} skipped.',
        'Command: python -m pytest research/sdf_cfd_mesher_v4/tests/test_immutable_overlay_publication.py research/sdf_cfd_mesher_v4/tests/test_native_checkpoint.py research/sdf_cfd_mesher_v4/tests/test_qualification_storage_audit.py --junitxml=research/sdf_cfd_mesher_v4/results/immutable_receipt_overlay_delta_publication/TESTS_FINAL.xml -q',
        'Covered: unedited detection; read/nested mutation/insertion/removal/replacement fallback; subclass/nested/metadata/authenticated exclusion; stale count/digest; stale journal base; concurrent writer; missing/corrupt base and update; corruption already in destination; interrupted object writes and pointer publication; abrupt subprocess exit before/after pointer swap; successful restore; every journal stage; idempotent retry; historical pointer restore.',
        'Fault injection uses disposable unit data, never production hardlinks. Existing native-checkpoint unit tests use their own small fixture state only for storage mechanics; it is not production geometry authority. Process-crash tests do not simulate physical power loss. No durability mechanism or gate was weakened.',
        'The initial preparation failed while inventorying an existing inaccessible V3 link. It was recorded, the inventory was changed to record inaccessible entries, and preparation was rerun. All audited L1/L2 files were accessible and passed. Initial 37-test and expanded 48-test runs also passed; final test count above is the nonduplicated final suite.',
        'Initial candidate A exposed a harness path issue: the update was staged locally but its table still referred to the external source, so the existing importer created a second hardlink name. No payload was copied. B/C bind the staged update path as the real transaction does. A remains retained as diagnostic evidence; B/C are the qualification pair. No publisher code or scientific data changed for this correction.',
        f'Pilot A retained {pilot["retained_logical_bytes"]} logical bytes, including {pilot["retained_duplicate_logical_bytes"]} byte-identical alias bytes; publication took {pilot["publication_seconds"]:.6f} s. It avoided full flattening: {pilot["full_flattening_avoided"]}. Its result is not substituted for either final replay.',
        '', '## Disk, preservation and result identities', '',
        '```json',json.dumps(disk,indent=2),'```',
        'No original authority, failure record, old checkpoint or retained replay artifact was deleted. All excluded bulk is listed with exact hashes in BULK_MANIFEST.json. Free-space change is not claimed as reclaimed storage.',
        '```json',json.dumps({k:r['pointer'] for k,r in publications.items()},indent=2),'```',
        f'V3 inaccessible inventory entries: {preservation["V3_inaccessible_entries"]}; they remain unchanged. Source hardlink counts can increase as references are added; source bytes, sizes, mtimes and file IDs are unchanged.',
        'Report and authority SHA-256 values are in SEALS.json and printed in the final response. IMPLEMENTATION.json binds the exact change and reproduction tools.',
        '', '## Limits and next task', '',
        'No remaining blocker for this bounded persistence optimization. L3 was neither forecast nor generated. Complete AdvancingFrontQualified, CoreHandoffQualified, CoreQualified and CompleteMeshQualified remain NO. No cleanup is qualified by this result.',
        'Exactly one recommended next task: qualify path-preserving deduplication of byte-identical L2 artifacts, prove every historical restore closure before and after it, and measure actual reclaimed allocation before considering L3.']
    (OUT/'report.md').write_text('\n'.join(lines)+'\n')
    evidence={p.name:file_hash(p) for p in OUT.iterdir() if p.is_file() and p.name not in ('AUTHORITY.json','SEALS.json','INPUTS.json')}
    authority=dict(classification=classification,supported_claim=claim,starting_head=HEAD,
        scientific_state=L2_ID,implementation=source,evidence=evidence,tests=counts,
        persistence_only=True,L3_forecast=False,L3_generated=False,cleanup_qualified=False,
        candidate_checkpoint=a['pointer'],preservation=preservation)
    authority['identity']=identity(authority);save('AUTHORITY.json',authority)
    save('SEALS.json',dict(report=file_hash(OUT/'report.md'),authority=file_hash(OUT/'AUTHORITY.json'),identity=authority['identity']))
    print('\n'.join(lines),flush=True)
    print(json.dumps(read('SEALS.json'),indent=2),flush=True)


if __name__=='__main__':main()

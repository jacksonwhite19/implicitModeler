"""Seal the read-only storage audit without modifying historical evidence."""
import json,shutil,subprocess
from audit_qualification_storage import ROOT,V4,OUT,L1,L2,digest,save

def main():
    load=lambda name:json.loads((OUT/name).read_text())
    s=load('SUMMARY.json');disk=load('DISK.json');res=load('RESOURCES.json');cmp=load('ROW_COMPARISON.json');roots=load('CHECKPOINT_REACHABILITY.json')
    files=load('FILES.json');idx={r['path']:r for r in files}
    manifest=json.loads((L2/'BULK_MANIFEST.json').read_text())
    for r in manifest['files']:
        actual=idx[(L2/r['path']).relative_to(ROOT).as_posix()]
        assert (actual['bytes'],actual['sha256'])==(r['bytes'],r['sha256'])
    # Stat preservation plus freshly rehashed report/authority/pointer seals.
    for r in files:
        stat=(ROOT/r['path']).stat()
        assert (stat.st_size,stat.st_mtime_ns,stat.st_ino)==(r['bytes'],r['mtime_ns'],r['inode'])
    seals={}
    for base in (L1,L2):
        for rel in ('report.md','AUTHORITY.json','A/checkpoints/CURRENT.json','B/checkpoints/CURRENT.json'):
            p=base/rel;h=digest(p);assert h==idx[p.relative_to(ROOT).as_posix()]['sha256'];seals[p.relative_to(ROOT).as_posix()]=h
    sources=['tools/production_layer2_run.py','tools/mixed_front_fixture_provider.py','tools/reuse_mixed_certificate.py','tools/delta_proof_binding.py','tools/receipt_retirement_archive.py']
    sources+=['src/sdf_cfd_mesher_v4/advancing_front/'+f for f in ('mixed_front.py','mixed_front_runner.py','mixed_front_journal.py','mixed_evidence.py','native_checkpoint.py','native_receipt_delta.py')]
    save('SOURCE_BINDINGS.json',{p:digest(V4/p) for p in sources})
    paths=['research/sdf_cfd_mesher_v4/'+p for p in ('tools/audit_qualification_storage.py','tools/analyze_qualification_storage.py','tools/report_qualification_storage.py','tests/test_qualification_storage_audit.py')]
    prefix='research/sdf_cfd_mesher_v4/results/production_qualification_storage_audit/'
    def unrelated(raw):
        return [r for r in raw.split(b'\0') if r and not (r[3:].decode() in paths or r[3:].decode().startswith(prefix))]
    before=bytes.fromhex(load('BASELINE.json')['status_hex']);after=subprocess.check_output(['git','status','--porcelain','-z'],cwd=ROOT)
    assert unrelated(before)==unrelated(after)
    save('PRESERVATION.json',dict(unchanged_input_files=len(files),bulk_files_verified=len(manifest['files']),seals=seals,
        unrelated_status_equal=True,unrelated_entries=len(unrelated(after)),v3_task_modifications=0,geometry_generated=False,forecast_executed=False,bytes_deleted=0))
    save('CLEANUP_PLAN.json',dict(executed=False,bytes_deleted=0,
        exact_duplicate_l2_payload_upper_bound=s['l2_duplicate_payload_upper_bound'],
        intermediate_tables_total_bytes=2*4858589184,
        intermediate_table_unique_bytes=4858589184,
        warning='Amounts overlap. Not promised free-space gain. Historical manifests reference retained paths. No migration or deletion authorized by this audit result.',
        stages=['Preserve all unique reports, failures, replay/restore attestations and manifests.',
            'Qualify path-preserving immutable deduplication with digest and restart checks before and after replacement.',
            'Resolve all historical manifest references before archiving or retiring intermediate tables. Retain exact byte archive if old file-hash replay is required.',
            'Only then reclaim copies; record file IDs, actual allocation/free-space change, recovery path and tombstones.']))
    a=json.loads((L2/'A/FINAL.json').read_text());b=json.loads((L2/'B/FINAL.json').read_text())
    lines=['# AuditV4ProductionQualificationStorageAndIncrementalPersistence',
    '', 'Classification: **V4ProductionQualificationStorageAuditCompleteIncrementalPersistenceNotYetQualified**.',
    '', 'The current L2 authority is preserved. This audit measures storage and traces persistence cost. It does not qualify a new storage backend, perform cleanup, or generate/forecast any layer.',
    '', 'Starting HEAD: `ee7b1cc3cabb12c2599c980fedea5a0b66c90bb4`.',
    'Committed L2: `707d9726da9f6a6f7dedc688f4437777aeb2e8fa5cf0958ca6aed83f8f7bfa44`.',
    '', '## Exact attribution',
    '', f'L2 retains {s["l2_logical_bytes"]:,} bytes across {s["l2_file_count"]} files. GB below means decimal bytes/1e9; no estimate of allocated filesystem blocks is made.',
    '', '| Store | Logical bytes |','|---|---:|']
    lines += [f'| {k} | {v:,} |' for k,v in s['attribution'].items()]
    lines += ['', '| Data type | Logical bytes |','|---|---:|']+[f'| {k} | {v:,} |' for k,v in s['categories'].items()]
    lines += ['', 'Each successful run contains:',
        '- Final certificate SQLite: 4,971,393,024 bytes; 4,839,083 rows.',
        '- Intermediate update SQLite: 4,858,589,184 bytes; 3,856,405 rows.',
        '- Final quality SQLite: 724,893,696 bytes; 270,288 rows.',
        '- Dependency atlas JSON: 233,489,397 bytes.',
        '- Expanded raw and final quality JSON: 79,046,434 bytes each.',
        '', 'Two independent successful executions each wrote these objects. Two failed prepublication attempts retained useful failure records plus repeated candidate/quality exports. The 23 GB is not primarily coordinate storage.',
        '', '## Duplicates and reference reachability',
        '', f'Within L2, distinct file-ID payload bytes = {s["l2_unique_file_id_payload_bytes"]:,}; unique byte-content payload = {s["l2_unique_content_bytes"]:,}. Exact duplicate payload = {s["l2_duplicate_payload_upper_bound"]:,} bytes. The L2 copies have distinct file IDs; they are not already hard-linked.',
        'SHA-256 and size identify duplicates. FILES.json records every file identity and DUPLICATES.json records every duplicate group, including L1 comparisons. No unique failed-state record is proposed for deletion.',
        'Each L2 CURRENT envelope directly references 6,092,456,960 object bytes. Its certificate descriptor is a flat receipt_table, not receipt_delta. CURRENT-unreferenced objects per successful run:',
        '- Intermediate update SQLite: 4,858,589,184 bytes.',
        '- Two proof-index NPY objects: 23,191,784 and 46,383,440 bytes.',
        '- Two proof-index JSON objects: 15,590 and 13,502,310 bytes.',
        'Unreferenced by CURRENT does not mean unreferenced by historical evidence. BULK_MANIFEST.json binds all 60 bulk files, including these objects. Full historical transitive reference garbage collection was not qualified.',
        '', '## Exact receipt reuse evidence',
        '', '| Comparison | Equal rows | Changed rows | Missing from final | New final rows |','|---|---:|---:|---:|---:|']
    for name,row in cmp.items():
        c=row['counts'];lines.append(f'| {name} | {c["equal"]:,} | {c["changed"]:,} | {c["left_only"]:,} | {c["right_only"]:,} |')
    lines += ['', 'All 3,856,405 intermediate key/value rows are exactly present in the final table. This proves row-data redundancy, not byte-identical SQLite regeneration.',
        'All 1,940,126 L1 proof keys remain. Of these, 982,678 complete receipts are unchanged; 957,448 change only inputs/dependency bindings. No proof payload or other receipt field changes in those 957,448 rows. The remaining 2,898,957 rows are fresh L2 interactions.',
        'The source explains the distinction: geometry_inputs hashes face records as well as coordinates. A former terminal face receives a neighbour/role change. Its geometric separation proof can remain valid, while its receipt dependency identity must be rebound. L1 determinant receipts also require the completed-neighbour semantics. Therefore an immutable geometry prefix is not authority to reuse all old metric/receipt context blindly.',
        'SQLite layout was measured read-only. Intermediate key/value payload: 3,531,807,857 bytes within 4,858,589,184 file bytes. Final certificate payload: 4,438,567,342 within 4,971,393,024 bytes. Quality payload: 249,130,839 within 724,893,696 bytes. All three have zero freelist pages. Differences include page/B-tree/record overhead and internal occupancy; they are not measured reclaimable space. No VACUUM or rewriting occurred.',
        '', '## Why persistence costs time',
        '', 'Measured historical L2 phase times, seconds. Transaction is inclusive; do not sum it with its subphases. These are preserved run measurements, not new profiling results.',
        '', '| Phase | A | B |','|---|---:|---:|']
    for k in ('L2_construction','combined_numeric','completed_neighbour_determinant','final_continuous_and_volume','rebuild','production_transaction','checkpoint'):
        lines.append(f'| {k} | {a["times"][k]} | {b["times"][k]} |')
    lines += ['', 'Verified implementation causes:',
        '1. MixedFrontRunner.step checks the exact provider type. ProductionLayer2Provider is a subclass, so ProductionFast intentionally enters the conservative path. This preserves scientific callbacks but does not use FastLayer incremental persistence.',
        '2. Provider.rebuild binds complete quality/receipt populations, rebuilds dependencies and archives retirements. rebuild_dependencies computes geometry inputs across the state and reconstructs every front atlas row. The current 233 MB atlas repeats dependency hashes/versions in JSON. Individual bind versus dependency versus archive times were not instrumented.',
        '3. attach_validation builds a ReceiptDelta from the immutable L1 base plus updates. ReceiptDelta computes its canonical merged identity by streaming rows.',
        '4. The in-memory journal deep-copies receipts. ReceiptTable.__deepcopy__ wraps even a ReceiptDelta in ReceiptOverlay. A focused test proves this exact type transition preserves scientific rows.',
        '5. NativeCheckpointStore._table_payload recognizes ReceiptDelta directly, but the published overlay reaches _flat_table. That path serializes the entire merged population into another SQLite file. The final envelope and the retained intermediate prove both representations survived. The empty overlay is not unwrapped at this publication boundary.',
        '6. Repeated scientific_hash, geometry preservation comparisons, journal restart/deepcopy and full JSON component serialization add traversal work. _json writes and fsyncs a temporary component before discovering an existing identical object. Separate hash/deepcopy/I/O timings are NotMeasured; no numerical speedup claim is supported.',
        '7. A and B use separate object directories. Local content-addressing avoids duplication within an object directory, not across these independently created stores. Existing os.link imports help only when the input already arrives as an immutable native object.',
        '', '## What must persist',
        '', '| Evidence | Retention rule |','|---|---|',
        '| Current L2 and L1 checkpoint roots and referenced objects | Keep a verifiable complete restart closure; preserve scientific identities and all referenced bytes. |',
        '| Wall, request, schedule, admission and implementation authorities | Keep sealed inputs and exact source/configuration versions. |',
        '| Unique report, failure, replay, tests and restore records | Keep; failures explain the receipt-overlay repair and replay provenance. |',
        '| Final quality and continuous proof population | Keep losslessly with authority/dependency bindings, even if representation becomes shared or compressed. |',
        '| Expanded raw/final JSON and identical replay arrays | Byte duplicates can share one immutable object only after every historical path remains resolvable and hash checks pass. |',
        '| Intermediate receipt table | All rows survive in final; regeneration of original SQLite byte identity is not established. Preserve exact archive or references before retirement. |',
        '| Dependency atlas and proof indices | Computable from sealed geometry/topology and source, but exact reconstruction/restart must be tested before treating them as disposable caches. |',
        '', '## Incremental persistence design and qualification boundary',
        '', 'A safe next implementation can preserve scientific definitions: unwrap only a provably unedited ReceiptOverlay to its immutable base before native publication; retain ReceiptDelta base/update/removal descriptors; continue exact canonical receipt identity and full dependency verification. Edited overlays must retain the existing fail-closed path. Never reuse a hash solely from a mutable filename.',
        'Use immutable content-addressed objects shared between run roots, with atomic root publication and reachability-based retention across all accepted, failed, replay and historical roots. Preserve schema/scientific identity independently from storage layout. Handle same-volume hard links and cross-volume copies explicitly. Shared objects must not be mutated in place.',
        'Preserve the 982,678 exact old receipts by reference; rebind the 957,448 changed dependencies and add fresh proofs. Keep actual completed-neighbour quality changes. Broader authenticated/block-native facilities already exist, but this audit does not declare them qualified for this production provider. Do not silently migrate the legacy scientific hash schema.',
        'A delta is not automatically smaller than a standalone flattened checkpoint: the current update file is already 4,858,589,184 bytes, plus its L1 base. Its benefit requires shared immutable prefix storage and avoiding simultaneous update-plus-flat retention. The standalone final certificate is only 112,803,840 bytes larger than this update file. Do not predict storage/time savings without the persistence-only benchmark.',
        'Required qualification: immutable-base and edited-overlay cases; exact old/new scientific equality; stale/corrupt object rejection; missing-base rejection; crash before/after pointer swap; concurrent retention safety; deterministic A/B restore; exact wall/L1/L2 preservation; no forecast or geometry generation; measured publication/rebuild cost. Benchmark persistence from sealed L2 only.',
        '', '## Reclamation decision',
        '', 'Delete nothing in this audit. The exact duplicate payload upper bound is 11,787,769,899 bytes within L2. This overlaps intermediate-table duplication. After duplicate coalescing, one intermediate copy remains: 4,858,589,184 bytes. Do not add both intermediate copies to the duplicate total.',
        'The combined logical reduction candidate is 16,646,359,083 bytes if one exact intermediate archive can be safely retired as well. This is not a measured free-space gain or a deletion authorization. If exact historical SQLite bytes must persist, that intermediate content still needs storage somewhere. Other CURRENT-unreferenced proof indices need an explicit regeneration/reference audit.',
        'CLEANUP_PLAN.json records the order: preserve unique evidence; qualify path-preserving deduplication and restore; resolve historical references; then reclaim and measure actual allocation/free space. No authority is rewritten.',
        '', '## Verification and resources',
        '', 'Seven focused audit tests passed; zero failed/skipped. No mesh tests or geometry generation were run. An initial audit script encountered the older L1 authority schema (no evidence field); it was corrected to verify that schema without changing L1. No historical evidence was modified.',
        f'All {len(files)} L1/L2 files were inventoried and hashed, with hardlink-aware file IDs. All 60 L2 bulk-manifest files and 57 L2 authority evidence files match. All checkpoint object descriptors match fresh inventory hashes. Input file sizes/mtime/file IDs remain unchanged, and key report/authority/pointer seals were freshly rehashed at the end.',
        'This is storage integrity and reference verification, not a new full geometric requalification or a fresh full native restart. Prior deterministic native restore evidence remains preserved.',
        f'Repository accessible logical use: {disk["totals"]["repository_logical_bytes"]:,} bytes. V4: {disk["totals"]["v4_logical_bytes"]:,} bytes. L1: 7,170,401,356 bytes.',
        f'Scan recorded {len(disk["errors"])} inaccessible entries, including 10 V4 links in older complete-mesh studies; none in audited L1/L2. Managed worktrees, nested dual_contouring and directory symlinks were excluded. These are accessible logical totals, not disk allocation.',
        f'Initial free space: {load("BASELINE.json")["free_bytes"]:,} bytes. Inventory/hash elapsed: {res["seconds"]} s; bytes hashed: {res["hashed_unique_file_bytes"]:,}; peak RSS: {res["peak_rss"]:,}. Row comparison timings and analysis RSS are in ROW_COMPARISON.json and ANALYSIS_RESOURCES.json.',
        f'Ending pre-seal free space: {load("END_RESOURCES.json")["free_bytes"]:,} bytes. Free space also changes from unrelated system activity; the increase is not attributed to cleanup. End-resource file reports result bytes at its measurement point; final bytes are printed after commit.',
        'Bytes deleted: 0. Files changed: audit tools, focused tests and this new result directory only. V3 modifications: 0. Unrelated dirty entries: 155, status manifest unchanged.',
        '', '## Recommended next task',
        '', '**QualifyV4ImmutableReceiptOverlayDeltaPublication**: preserve unchanged delta structure through publication using sealed L2 persistence-only replay; test exact identity, restart, corruption/crash rejection and storage/time change. Do not generate or forecast L3.']
    (OUT/'report.md').write_text('\n'.join(lines)+'\n')
    evidence={p.name:digest(p) for p in OUT.iterdir() if p.is_file() and p.name not in ('AUTHORITY.json','HASHES.json')}
    authority=dict(classification='V4ProductionQualificationStorageAuditCompleteIncrementalPersistenceNotYetQualified',
        starting_head=load('BASELINE.json')['head'],committed_L2=roots['L2A']['scientific'],
        evidence=evidence,implementation={p:digest(ROOT/p) for p in paths},bytes_deleted=0,
        geometry_generated=False,forecast_executed=False,persistence_migration_qualified=False)
    save('AUTHORITY.json',authority)
    save('HASHES.json',{n:digest(OUT/n) for n in ('report.md','AUTHORITY.json')})
    print((OUT/'HASHES.json').read_text())

if __name__=='__main__':main()

"""Seal measured cleanup evidence only after all required restores pass."""
from collections import Counter
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET
from qualify_path_preserving_deduplication import OUT, ROOT, TREES, HEAD, git, save, seals
from path_preserving_deduplication import file_hash


def load(name):return json.loads((OUT/name).read_text())


def digest(value):return hashlib.sha256(json.dumps(value,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()


def main():
    assert not (OUT/'AUTHORITY.json').exists(),'Do not rewrite a sealed qualification'
    before,after,actions,plan=(load(s) for s in ('BEFORE.json','AFTER.json','ACTIONS.json','PLAN.json'))
    labels=['L1_A','L1_B','L2_A','L2_B','baseline','candidate_A','candidate_B','candidate_C']
    restores={label:load(label+'_RESTORE.json') for label in labels}
    assert all(r['passed'] for r in restores.values())
    l2='707d9726da9f6a6f7dedc688f4437777aeb2e8fa5cf0958ca6aed83f8f7bfa44'
    for label,r in restores.items():
        assert r['scientific']==('5b036bc4cd5180b9e4a7e3e6aafe57fe21c7b2ae1cae82cf3c42e446b1805fd0' if label.startswith('L1') else l2)
        assert r['cells']==(33786 if label.startswith('L1') else 67572)
        assert r['receipts']==(1940126 if label.startswith('L1') else 4839083)
    assert restores['L2_A']['checkpoint']['sha256']=='644a5966f33072f1bde40fdc336eb18bf4955cb126791589cc1d48c1fa986388'
    for label in ('candidate_B','candidate_C'):
        assert restores[label]['checkpoint']['sha256']=='5647c5d1c43ff462c404b46f6b5a3708d534131374601e90dbf7d1eee79a16e2'
    assert seals()==before['protected']==after['protected']
    tests=ET.parse(OUT/'TESTS_WITH_ID_PROBE.xml').getroot().find('testsuite').attrib
    assert int(tests['tests'])==68 and all(int(tests[k])==0 for k in ('failures','errors','skipped'))
    retired=load('RETIRED_ID_CHECK.json')
    assert retired['all_canonicals_open'] and retired['all_retired_ids_match_deleted_control']
    current=[s for s in git('status','--porcelain=v1').splitlines() if 'research/sdf_cfd_mesher_v4/' not in s]
    assert current==load('GIT_BEFORE.json')['unrelated']
    assert git('rev-parse','HEAD')==HEAD
    b,a=before['totals'],after['totals']
    reclaimed=b['unique_physical_allocation']-a['unique_physical_allocation']
    assert reclaimed==plan['physical_reclaimable']
    assert len(actions['records'])==plan['replacement_paths']==93
    assert all(r['stage']=='verified' and r['old_handle_after_replace']['links']==0 for r in actions['records'])
    groups=load('DUPLICATES.json')
    numbers=dict(before=b,after=a,free_before=before['free_bytes'],free_after=after['free_bytes'],
        free_delta=after['free_bytes']-before['free_bytes'],physical_reclaimed=reclaimed,
        free_delta_minus_reclaimed=after['free_bytes']-before['free_bytes']-reclaimed,
        logical_removed=0,logical_shared=actions['logical_shared'],replaced_paths=93,deleted_paths=0,
        duplicate_content_groups=len(groups),groups_with_separate_copies=sum(g['physical_copies']>1 for g in groups),
        groups_with_existing_hardlinks=sum(g['paths']>g['physical_copies'] for g in groups),
        unresolved_copy_groups=len(plan['unresolved']),
        unique_physical_payload_bytes_before=b['logical_bytes']-b['hardlinked_logical_bytes'],
        unique_physical_payload_bytes_after=a['logical_bytes']-a['hardlinked_logical_bytes'],
        inventory_hash_seconds=before['inventory_hash_seconds'],deduplication_seconds=actions['seconds'],
        standalone_cleanup_seconds=0,post_inventory_seconds=after['verification_seconds'],
        restore_seconds=sum(r['seconds'] for r in restores.values()),
        peak_rss=max([before['peak_rss'],after['peak_rss'],actions['peak_rss']]+[r['peak_rss'] for r in restores.values()]),
        storage_worker_peak_rss=max(before['peak_rss'],after['peak_rss'],actions['peak_rss']),
        tests=tests,unrelated_dirty_entries=len(current),task_v3_modifications=0)
    save('MEASUREMENTS.json',numbers)
    retention=[]
    for r in before['rows']:
        path=Path(r['path'])
        if 'failed_' in r['path'] or path.name.startswith('TESTS') or 'RETRY' in path.name or 'CORRECTION' in path.name:
            role,reason='B','Retained failure, crash-test, or debugging evidence'
        elif r['immutable']:
            role,reason='A','Immutable object named by sealed manifest and/or checkpoint; keep historical path'
        elif r['references'] or path.name in ('AUTHORITY.json','report.md','BULK_MANIFEST.json','CURRENT.json') or path.name.startswith('NATIVE_'):
            role,reason='A','Accepted authority, report, manifest, checkpoint, or authority-bound evidence'
        else:
            role,reason='F','No deletion or shared-mutability qualification; preserve'
        retention.append(dict(path=r['path'],sha256=r['sha256'],role=role,reason=reason,
            references=r['references'],replacement=any(x['original']['path']==r['path'] for x in actions['records'])))
    save('RETENTION.json',dict(objects=retention,counts=dict(Counter(r['role'] for r in retention)),
        C_regeneration_claims=0,E_disposable_claims=0,
        D_duplicate_payloads_coalesced=93,all_required_paths_preserved=True))
    save('PRESERVATION.json',dict(protected=seals(),paths_verified=309,all_path_size_hash_triples_equal=True,
        all_original_names_retained=True,unrelated_status_equal=True,task_v3_modifications=0,
        historical_v3_dirty_entries=sum('sdf_cfd_mesher_v3' in s for s in current),
        L3_forecast=False,L3_generated=False))
    # Authoritative machine-readable path-level replacement/deletion manifest.
    manifest=dict(format='V4PathPreservingDeduplicationActions/1',
        replacements=actions['records'],standalone_deletions=[],
        physical_cluster_bytes_reclaimed=reclaimed,logical_removed=0,
        logical_shared=actions['logical_shared'])
    save('CLEANUP_MANIFEST.json',manifest)
    report=f'''V4PathPreservingDeduplicationAndSafeReclamationQualified

Supported claim: on this NTFS volume, 93 exact immutable checkpoint-object copies were replaced by hardlinks while all 309 retained historical paths and bytes remained valid. Allocated data clusters decreased by {reclaimed:,} bytes. All eight checkpoint stores restored in fresh processes. This qualifies this bounded cleanup, not arbitrary mutable files or semantically equivalent containers.

Starting HEAD: {HEAD}. The ending HEAD is the Git commit containing this sealed report; it is printed in the final terminal response. Branch: feature/sdf-cfd-mesher-v4-advancing-front.

Verified inputs: the original L2 authority, report, native checkpoint, the immutable-overlay authority/report, L1 authority/report/manifests, storage-audit authority/report, and admitted request. There are {len(before['protected'])} explicitly sealed protected files and 309 path/size/content triples. Original L2 checkpoint SHA-256: {restores['L2_A']['checkpoint']['sha256']}. Optimized B/C checkpoint SHA-256: {restores['candidate_B']['checkpoint']['sha256']}. Prior persistence identity remains cfef66665ae1edabb352ae5cb44701d4a4e67fd41ea82de1d80c7db57917851a.

| Storage measure (bytes unless stated) | Before | After |
|---|---:|---:|
| Retained paths | {b['paths']} | {a['paths']} |
| Physical file IDs | {b['physical_objects']} | {a['physical_objects']} |
| Content groups | {b['content_groups']} | {a['content_groups']} |
| Relevant logical bytes | {b['logical_bytes']} | {a['logical_bytes']} |
| Allocated data clusters, unique file IDs | {b['unique_physical_allocation']} | {a['unique_physical_allocation']} |
| Duplicate allocated data clusters | {b['duplicate_physical_allocation']} | {a['duplicate_physical_allocation']} |
| Unique physical payload logical bytes | {numbers['unique_physical_payload_bytes_before']} | {numbers['unique_physical_payload_bytes_after']} |
| Hardlinked logical bytes beyond first name | {b['hardlinked_logical_bytes']} | {a['hardlinked_logical_bytes']} |
| Filesystem free bytes | {before['free_bytes']} | {after['free_bytes']} |
| Repository accessible logical bytes | {before['scope']['repository_accessible_logical']} | {after['scope']['repository_accessible_logical']} |
| V4 accessible logical bytes | {before['scope']['v4_accessible_logical']} | {after['scope']['v4_accessible_logical']} |

Broad repository/V4 totals exclude {len(before['scope']['inaccessible'])} inaccessible entries and the four recorded nested checkout/junction trees. These are accessible totals, not exact totals for inaccessible data. All three cleanup trees were fully inventoried. No inaccessible path was changed.

Exact-content duplicates: {len(groups)} groups have multiple names; {numbers['groups_with_separate_copies']} groups have separate physical copies; {numbers['groups_with_existing_hardlinks']} groups already have hardlinks (categories overlap). DUPLICATES.json lists every group, SHA-256, physical ID, link count, size and path. BEFORE.json binds each material object to known checkpoint and sealed manifest references. Direct checkpoint references are listed where present; intermediate objects remain retained under their sealed bulk manifests. Original flat and optimized delta checkpoints can have the same scientific state with different container bytes. Such containers were preserved independently; semantic equality did not authorize replacement.

Retention: RETENTION.json assigns each original path one conservative retention class: {dict(Counter(r['role'] for r in retention))}. The 93 coalesced physical copies were D payload duplicates while their A historical names remain valid. All unique authorities, manifests, reports, failure/debug/crash evidence and replay attestations retain their original bytes. The 22 non-eligible duplicate-copy groups remain preserved. Residual duplicate cluster allocation is {a['duplicate_physical_allocation']:,} bytes. No standalone intermediate was deleted, including tables still named by historical manifests. No unique scientific payload was deleted.

Qualification rule: sealed content-addressed native object, matching exact SHA-256 and logical size, same volume, matching owner/group/DACL and attributes, default data stream only, no reparse/sparse/compressed/encrypted/read-only file, and fully enumerated victim aliases. The audited canonicals also had no unenumerated aliases. Native object writers use immutable content-addressed publication; receipt SQLite readers are read-only. Metadata timestamps and file IDs are storage metadata, not authority bytes; native cache signatures include file identity/change time and cannot reuse stale verification after replacement.

Method: hash both locked handles, durably record original hash/size/file ID/references and canonical target, create a same-directory hardlink, atomically rename it over the original name with FileRenameInfoEx REPLACE_IF_EXISTS/POSIX_SEMANTICS, rehash the path, and verify the shared file ID. Content-write exclusion remains active during replacement. Recovery journals are retained. Tests force process exits at prepared, linked and replaced stages and verify recovery. Tests do not simulate electrical power loss. NTFS recovery and filesystem durability remain platform assumptions; the retained journal/canonical bytes permit repair of interrupted work.

Physical accounting: GetFileInformationByHandleEx reports file allocation and links; FSCTL_GET_RETRIEVAL_POINTERS counts allocated data clusters using the measured volume cluster size (4096 bytes). Resident MFT payload allocation ({b['resident_allocation_not_claimed']} bytes) is excluded from recovered disk bytes. The 93 eliminated IDs had zero final links, all handles were closed, and fresh inventory confirmed exactly {reclaimed:,} fewer allocated bytes. A further OpenFileById check opened all 93 canonicals and rejected all 93 retired IDs with error 87, exactly matching a newly measured deleted-file control. The pilot probe had expected error 2; its raw result is retained, and no missing-file error meaning was assumed. Free-space change alone is not used as recovery evidence. Free-space delta is {numbers['free_delta']:,}; its difference from reclaimed data clusters is {numbers['free_delta_minus_reclaimed']:,} bytes of unattributed net allocation noise (including new task evidence), not attributed to cleanup.

Actions: 93 replacements; 0 standalone deletions; 0 logical bytes removed; {actions['logical_shared']:,} logical bytes converted to shared hardlinks; {reclaimed:,} duplicate physical bytes eliminated and allocated data bytes reclaimed. CLEANUP_MANIFEST.json contains complete absolute path-level records and all original/new identities.

Fresh-process restore results:
'''
    for label,r in restores.items():
        report+=f"\n- {label}: PASS; scientific {r['scientific']}; {r['cells']} cells; {r['receipts']} receipts; {r['seconds']:.6f} s; peak RSS {r['peak_rss']} bytes."
    report+=f'''

Current production committed L2 remains {l2}. Wall/L1/L2 coordinate hashes match the sealed values in all applicable restores. Required ownership, dependency bindings, proof payloads, quality evidence and scientific state are preserved by byte equality and native restart verification. Historical original and optimized checkpoint container hashes remain unchanged. No authority was rewritten to permit cleanup.

Tests: final qualification suite 68 passed, 0 failed, 0 skipped. It includes 17 new filesystem/storage tests and 51 existing overlay/native-checkpoint/storage-audit tests. It covers exact content, false references, active writers, mutable/alternate-stream rejection, cross-volume policy rejection (simulated different device; no second physical test volume), metadata mismatch, missing canonical, atomic process interruption/recovery, aliases, idempotence, and fresh-process storage restore. Early isolated implementation trials had 6 failures each: open-target MoveFileEx rejection/test-state omissions, then a missing UTF-16 terminator in the FileRenameInfoEx buffer. Both were fixed before any retained file replacement. TESTS.xml and TESTS_RENAME.xml preserve these failures; subsequent focused and final suites pass.

Timing: initial authoritative inventory/hash {before['inventory_hash_seconds']:.6f} s; deduplication {actions['seconds']:.6f} s; standalone deletion 0 s (not performed); post-change inventory/hash {after['verification_seconds']:.6f} s; eight restores total {numbers['restore_seconds']:.6f} s. Storage-worker peak RSS {numbers['storage_worker_peak_rss']} bytes; maximum measured worker peak RSS including restore {numbers['peak_rss']} bytes. These are measurements, not performance gates.

Git safety: 155 unrelated dirty entries match the saved baseline. Task V3 modifications: 0 (one V3 entry was already dirty/untracked). Only task tooling, tests and reports are committed. No production persistence implementation, geometry, request, schedule, hard gate or receipt semantic changed. No L3 geometry or forecast ran.

Remaining concerns: 1,309,429,760 duplicate allocated bytes remain deliberately retained; their cleanup is not qualified. Filesystem IDs/timestamps in old inventories describe their original runs and are not rewritten. No material persistence blocker was found for read-only L3 admission. Broad filesystem audit gaps do not intersect the cleanup scope.

Exactly one recommended next task: QualifyV4CapsuleLayer3ReadOnlyAdmissionFromCommittedLayer2.

Platform references: https://learn.microsoft.com/en-us/windows/win32/api/winbase/ns-winbase-file_standard_info ; https://learn.microsoft.com/en-us/windows/win32/api/winioctl/ni-winioctl-fsctl_get_retrieval_pointers ; https://learn.microsoft.com/en-us/windows/win32/api/winbase/ns-winbase-file_rename_info .
'''
    (OUT/'report.md').write_text(report,encoding='utf-8',newline='\n')
    evidence={p.relative_to(OUT).as_posix():file_hash(p) for p in sorted(OUT.rglob('*')) if p.is_file()}
    implementation={str(p.relative_to(ROOT)).replace('\\','/'):file_hash(p) for p in (
        ROOT/'research/sdf_cfd_mesher_v4/tools/path_preserving_deduplication.py',
        ROOT/'research/sdf_cfd_mesher_v4/tools/qualify_path_preserving_deduplication.py',
        Path(__file__),ROOT/'research/sdf_cfd_mesher_v4/tests/test_path_preserving_deduplication.py')}
    authority=dict(classification='V4PathPreservingDeduplicationAndSafeReclamationQualified',
        starting_head=HEAD,committed_L2=l2,physical_cluster_bytes_reclaimed=reclaimed,
        replaced_paths=93,deleted_paths=0,scientific_payload_loss=False,L3_forecast=False,L3_generated=False,
        evidence=evidence,implementation=implementation)
    authority['qualification_identity']=digest(authority)
    save('AUTHORITY.json',authority)
    print(json.dumps(dict(classification=authority['classification'],identity=authority['qualification_identity'],
        authority_sha256=file_hash(OUT/'AUTHORITY.json'),report_sha256=file_hash(OUT/'report.md'),
        manifest_sha256=file_hash(OUT/'CLEANUP_MANIFEST.json'),measurements=numbers),indent=2))


if __name__=='__main__':main()

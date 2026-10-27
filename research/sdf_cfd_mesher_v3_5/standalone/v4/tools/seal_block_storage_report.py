"""Seal measured canonical-storage evidence; refuses incomplete qualification."""

import hashlib,json,time

from pathlib import Path



ROOT=Path(__file__).resolve().parents[1]

OUT=ROOT/'results/canonical_block_authenticated_storage'



def read(name):return json.loads((OUT/name).read_text())

def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()

def table(headers,rows):

    return '\n'.join(['| '+' | '.join(headers)+' |','| '+' | '.join(['---']*len(headers))+' |']+['| '+' | '.join(map(str,row))+' |' for row in rows])

def f(value):return f'{value:.6f}'



def run():

    baseline=read('BENCHMARK_COMPARISON.json');dual=read('CAPSULE_DUAL_ROOTS.json')

    audit=read('LAYER10_GLOBAL_AUDIT.json');sf2=read('SF2/BLOCK_ROOT.json');tests=read('TESTS.json')

    bent=read('BENT_DUAL_ROOTS.json');continuation=[]

    for layer in (11,12):

        folder=f'capsule_production_control/capsule_{layer}'

        choices=[folder+suffix for suffix in ('_final','_retry','')]

        folder=next((name for name in choices if (OUT/name/'SHELL_RESULT.json').exists()),None)

        if folder is None:continue

        trace=read(folder+'/TRACE.json');shell=read(folder+'/SHELL_RESULT.json')

        event=next(x for x in read(folder+'/BLOCK_TIMERS.json') if x['event']=='retained_after_commit')

        assert shell['restart_exact'] and shell['wall_exact'] and shell['first_layer_exact']

        continuation.append(dict(layer=layer,trace=trace,shell={k:shell[k] for k in ('current_layer','cells','restart_exact','wall_exact','first_layer_exact','remesh_count','final')},event=event,retention=read(folder+f'/RETENTION_{layer}.json')['payload']))

    assert audit['passed'] and audit['migration_legacy_exact'] and sf2['passed']

    assert all(x['legacy']['passed'] for x in baseline)

    assert all(x['passed'] for x in dual) and all(x['passed'] for x in bent)

    assert tests['windows']['passed']==966 and tests['wsl']['passed']==11 and tests['working_release_focused']['passed']==9

    complete={x['layer'] for x in continuation}=={11,12}

    classification='V4CanonicalBlockAuthenticatedStorageQualified' if complete else 'V4CanonicalBlockAuthenticatedStoragePromising'

    sections=[]

    def section(title,text):sections.append('### '+title+'\n\n'+text.strip()+'\n')

    section('A. Authority verification',f"Starting HEAD: `{read('TASK_MANIFEST.json')['head']}`. Preserved Patricia inputs remain external and unchanged. All new working checkpoints have explicit migration descriptors. V3 changes from this task: zero. No mesh gates, geometry, schedule, or certificate mathematics changed.")

    section('B. Existing Patricia storage behavior',table(['Layer','Changed/new keys','Patricia put calls','New node-file bytes','Referenced node bytes','Root update s'],[

        [7,1609868,5795928,1229320192,2971881472,'139.425432'],[8,1543684,5945034,1270480896,4242362368,'152.577822'],[9,1443732,5923578,1274658816,5517021184,'158.644243'],[10,1328724,5779399,1250324480,6767345664,'182.222333']])+"\n\nThe prior path already bulk-applies sorted deltas. No Patricia batching change was made.")

    dispersion=read('KEY_DISPERSION.json')['results']

    section('C. Changed-key dispersion',table(['Layer','Active keys','Changed/new','Rank minimum','Rank maximum','Changed-key rank deciles'],[[x['layer'],x['active_keys'],x['changed_new_keys'],x['rank_min'],x['rank_max'],','.join(map(str,x['rank_decile_counts']))] for x in dispersion])+"\n\nEvery occupied block is touched at 8, 12, and 16 prefix bits. Untouched blocks: zero. These measured keys do not have useful prefix locality.")

    section('D. Canonical block design','Adopt fixed 8-bit SHA-256 prefixes: 256 possible ranges. Each active row is the existing 32-byte scientific-key digest plus the existing 32-byte complete-receipt-content digest. Partition depth is explicit authority metadata. Blocks contain active rows only, in strict canonical order.')

    gran=read('BLOCK_GRANULARITY.json')['results']

    section('E. Block-size study',table(['Prefix bits','Active/touched blocks','Update s','Bytes rewritten','Independent root s'],[[x['depth'],x['active_blocks'],f(x['update_seconds']),x['update_work']['bytes_written'],f(x['independent_seconds'])] for x in gran])+"\n\nThe 16-bit probe overlapped the first unsuccessful production attempt; it is not an isolated timing. All probes establish exact independent roots. Eight bits gave the lowest measured file/update overhead. This does not establish a universal optimum. One prefix selects one leaf block for lookup.")

    section('F. Block scientific authority','New map schema: `V4CanonicalSortedBlocks/1`. Receipt authority: `V4CertificateReceiptBlocks/1`. Block SHA-256 binds version, range, row count, and complete canonical key/value rows. Value hashes bind all preserved receipt content, including proof outcomes and face/dependency inputs. Legacy Patricia hashes are not reinterpreted. The mixed scientific composition is explicitly versioned `V4MixedScientificBlocks/1`; validation composition is `V4ValidationAuthenticatedBlocks/1`.')

    section('G. Upper Merkle authority','Domain-separated leaves bind range/count/block hash, including empty ranges. Ordered binary parents bind child hashes and level. The final root binds prefix depth, active count, and upper root. For capsule layers, 255 internal hashes and 256 leaf hashes change. These are small in-memory hashes, not 511 database-node writes. Identity is independent of insertion order, thread order, split history, and file placement.')

    section('H. Delta insert/replace/retire',table(['Layer','Insert','Replace/rebind','Retire'],[[x['layer'],x['insert'],x['replace'],x['retire']] for x in baseline])+"\n\nInsert rejects existing keys. Replace and retire reject absent keys. Each touched block is rebuilt once. Untouched blocks retain their immutable content address. Focused tests exercise nonzero retirement and local-remesh-style replacement.")

    section('I. Receipt rebinding','Historical top-face ownership changes still rebind 517,028 / 501,892 / 472,356 receipts on layers 7 / 8 / 9. These changes are scientifically required. Rebound keys touch all 256 ranges. Combined insert/rebind writes are measured below. Separate receipt-only write bytes: Not measured. The raw SQLite receipt tables remain authoritative content storage and are not replaced by block hashes.')

    section('J. Dependency index','The qualified compact face-to-relation CSR index is reused unchanged. Changed-face lookup does not scan every historical proof pair. No old-old exact proof recomputation is introduced. Canonical block updates do read all active compact rows in this dispersed workload; they do not re-render all historical proof JSON.')

    section('K. Storage format','One content-addressed immutable file per nonempty range, with 40 header bytes plus 64 bytes per active row. No per-binary-node SQLite rows or index pages. Current descriptors contain block hashes and sizes. The small upper tree is reconstructed from these descriptors. Raw receipt/proof/geometry stores remain separate.')

    overhead=read('PATRICIA_STORAGE_OVERHEAD.json')

    sections[-1]+='\n'+table(['Layer','Patricia logical hash/payload B','File B','Page/index overhead B'],[[x['layer'],x['logical_hash_and_node_payload_bytes'],x['file_bytes'],x['file_minus_payload_bytes']] for x in overhead])+'\n\nPrior SQLite page size: 4096 bytes; free pages: zero; journal mode: delete. Candidate blocks have no SQLite journal/page/index overhead; headers total 10,240 bytes for 256 blocks. Filesystem allocation slack is separate.\n'

    section('L. Content-addressed reuse','An existing identical block is verified and reused. Capsule 7–9 measured duplicate hits: 0; bytes avoided: 0; untouched bytes reused: 0. Unit tests prove identical-content reuse. Benefits here come from coarse representation and safe retention, not observed block deduplication.')

    section('M. Current/pinned-root retention','CurrentRoot retains the active block set. TransactionPinnedRoot protects old and candidate roots through publication. ReplayPinnedRoot explicitly preserves an old working root when restart is required. SealedHistoricalExternalAuthority remains immutable outside the active store. An unpinned old working envelope is provenance, not an indefinite restart promise.')

    section('N. Safe compaction','After atomic publication, reachability collection validates current/pinned roots and unlinks only unreachable local block/node objects. Old copied Patricia node links are removed from the new working store after migration; original external files remain unchanged. Historical raw receipt/proof/geometry files are not collected. Tests cover current roots, replay pins, retired relations, and old-checkpoint restart while pinned.')

    sections[-1]+='\nAfter their complete comparisons, the failed layer-6 bootstrap copy and completed layer-9 comparison copy were explicitly deactivated under the storage lock. Their former CURRENT pointers, checkpoint envelopes, raw scientific components, and qualification evidence remain. They are not active/replay-pinned roots and no longer promise direct restart. Collection released 339,288,320 and 538,072,832 derived map bytes. Sealed external Patricia inputs and the independently audited layer-10 successor remain intact.\n'

    sections[-1]+='\nAll explicitly deactivated task working roots:\n\n'+json.dumps([json.loads(p.read_text()) for p in sorted(OUT.rglob('WORKING_ROOT_RELEASE.json'))],indent=2)+'\n'
    section('O. Transaction/restart safety','Block-backed native steps hold the storage lock across candidate construction, validation, remesh if needed, and publication. A checksummed journal protects old/candidate roots. Crash tests before and after CURRENT replacement restore the correct authority. A generic disk-full defect left unpublished receipt scratch; cleanup now removes only that scratch after failure, with a focused regression test. The first failed attempt and exact recovery evidence are preserved. Filesystem capacity management, if used, is separately logged and is not counted as block-storage improvement.')

    sections[-1]+='\nAll failed attempts, including elapsed runtime and peak RSS when recorded:\n\n'+json.dumps(read('FAILED_ATTEMPTS.json'),indent=2)+'\n'
    section('P. Patricia cross-check','Complete raw scientific populations independently reconstruct both roots for layers 7–9. Exact legacy scientific and proof digests also match the preserved transitions. Layer 10 migration independently reconstructs the preserved Patricia root before block publication. Roots intentionally differ by representation.\n\n'+json.dumps(dual,indent=2))

    section('Q. Benchmark layer results',table(['Layer','Accepted before s','Accepted after s','Identity before s','Identity after s','Block update only s'],[[x['layer'],f(x['layer_seconds_before']),f(x['layer_seconds_after']),f(x['root_update_before']),f(x['root_update_after']),f(x['block_update_after'])] for x in baseline])+"\n\nThe identity timer includes delta encoding/hashing and map update. Qualification-only legacy cross-check time is excluded from accepted-layer time. Inputs are the same preserved scientific transitions; earlier shell geometry was not regenerated.")

    section('R. Storage rewrite amplification',table(['Layer','Active blocks','Touched','Untouched','Touched fraction','Written B','Reused B','After compaction B'],[[x['layer'],x['active_blocks'],x['touched_blocks'],x['untouched_blocks'],x['touched_fraction'],x['bytes_written'],x['bytes_reused'],x['storage']['retained_bytes']] for x in baseline])+"\n\nActive node/block storage is compared, not total raw-proof storage or disk allocation of external historical results. All blocks rewrite; no O(delta)-byte claim is made. Current-only retention tracks active population size instead of accumulating all previous tree versions.")

    sections[-1]+='\nNew working-store map bytes immediately before collection (old active plus candidate, calculated from measured file sizes): 748,518,400 / 885,134,848 / 1,013,977,600 for layers 7 / 8 / 9. After collection: 409,230,080 / 475,904,768 / 538,072,832. Layer-10 migration reduces referenced authenticated storage from 6,767,345,664 to 595,374,336 bytes. These figures exclude external sealed authorities and raw receipt storage.\n'
    sections[-1]+='\nTotal referenced checkpoint component inventory (logical bytes, including raw receipts/proofs/geometry):\n\n'+json.dumps(read('CHECKPOINT_STORAGE_INVENTORY.json'),indent=2)+'\n'

    section('S. Runtime',table(['Layer','Receipt SQL before s','Receipt SQL after s','Checkpoint before s','Checkpoint after s'],[[x['layer'],f(x['receipt_storage_before']),f(x['receipt_storage_after']),f(x['checkpoint_seconds_before']),f(x['checkpoint_seconds_after'])] for x in baseline])+"\n\nCheckpoint serialization did not improve. Raw receipt SQL work remains substantial. Rejected-run time, migration time, and independent audit time remain separately recorded in their artifacts.")

    sections[-1]+='\nMeasured restart calls, in execution order (different run boundaries/caching; not paired per-layer speed ratios):\n\n'+json.dumps(read('RESTART_TIMINGS.json'),indent=2)+'\n'

    section('T. Peak and retained RSS',table(['Layer','Peak before B','Peak after B','Retained before B','Retained after B','CPython arenas after'],[[x['layer'],x['peak_rss_before'],x['peak_rss_after'],x['retained_rss_before'],x['retained_rss_after'],x['allocator']['arenas allocated current']] for x in baseline])+"\n\nRetained RSS did not improve. CPython arena counts grow 1002 → 1330 → 1521. Reachable committed geometry grows 436,455,600 → 493,975,636 → 555,340,456 bytes; atlas remains 85,455,825 bytes. Mapped region sizes remain 202,092,544 bytes (161 regions); these are not resident-page ownership. Exact native/SQLite/physical-page attribution: Not measured. Per-block named-array maxima and delta arrays are in BLOCK_TIMERS.json. Block storage uses no SQLite cache. Prior writer cache is 64 MiB; prior readers mmap immutable node files. Separate RSS at each hash/write substage: Not measured.")

    section('U. Independent global audit','The audit streams all complete proof records, reconstructs receipt dependencies from geometry, rebuilds the new root without stored block bytes, independently enumerates required coverage, audits outcomes, and computes legacy digests.\n\n'+json.dumps(dict(times=audit['times'],total_seconds=audit['total_seconds'],peak_rss=audit['peak_rss']),indent=2)+"\n\nPrior layer-9 audit: root reconstruction 198.138332 s; coverage 102.918731 s; receipt audit 80.764671 s; total 554.071947 s. Different layer populations are stated explicitly; this is not a same-layer audit speed ratio.")

    sections[-1]+='\nThe first full audit failed in SQLite GROUP BY over the active receipt overlay with database/disk full. Its completed root/coverage timings and failure are preserved in LAYER10_AUDIT_FIRST_ATTEMPT.json. The generic fix streams the identical canonical active rows and counts permitted outcomes in bounded memory. It does not omit any receipt, change allowed classifications, or weaken coverage. Two focused tests cover replaced/retired invalid history and rejection of a bad live replacement. The full audit was rerun after this fix.\n'

    section('V. Layer-10 qualification',json.dumps({k:audit[k] for k in ('passed','coverage','receipts','independently_rebuilt_root','migration_legacy_exact','patricia_root','comparison','restart_exact')},indent=2))

    sections[-1]+='\nLayer-10 full legacy scientific digest matches the preserved migration input. The full legacy proof digest was computed, but no prior layer-10 proof-digest reference was available; an old-reference comparison for that digest is Not measured.\n'
    section('W. Capsule continuation',table(['Layer','Accepted s','Relations','Root','Active storage B','Peak B','Retained B'],[[x['layer'],f(x['trace'][-1]['seconds']),x['event']['relations'],x['event']['root'],x['retention']['retained_bytes'],x['event']['memory']['process']['peak_wset'],x['event']['memory']['process']['rss']] for x in continuation])+"\n\nLayers 11 and 12 start only after layer-10 independent audit passes. A process/checkpoint restart separates the two continuations. Wall and first layer remain exact. No full 20-layer build or broad production geometry run was attempted.")

    sections[-1]+='\nThe first layer-11 attempt failed during raw quality-table serialization with storage full, after 558.538412 s (peak 4,325,355,520 bytes). CURRENT stayed at audited layer 10. Recovery removed 648,190,208 unpublished block bytes and retained the exact old root. A second layer-11 attempt also failed before publication; its failure and recovery are preserved. Capacity management uses byte-verified immutable receipt-cache reuse and separately recorded NTFS compression of task-owned closed files. Any resulting continuation timings are not clean uncompressed-format speed comparisons. Uncommitted candidates do not count as accepted layers. Primary layers 7–9 are unaffected.\n'

    sections[-1]+='\nReached committed continuation layers: '+str([x['layer'] for x in continuation])+'. Unreached requested layers are not qualified. Scientific execution is bounded by the task cap; failure records retain the unsuccessful work.\n'

    if (OUT/'EXECUTION_LIMIT.json').exists():sections[-1]+='\nExecution-limit evidence:\n\n'+json.dumps(read('EXECUTION_LIMIT.json'),indent=2)+'\n'
    section('X. Bent-passage regression',json.dumps(bent,indent=2)+"\n\nOne preserved accepted 5→6 transition uses the same generic representation; legacy scientific/proof equality and independent dual roots are required.")

    sections[-1]+='\nMeasured bent-passage transition:\n\n'+json.dumps(read('BENT_RESULT.json'),indent=2)+'\n'
    section('Y. SF2 regression',json.dumps(sf2,indent=2)+"\n\nScope: full 56,462,864 native relation authority, represented by the existing twenty ledger records and verified component hashes. This is not an expanded 56-million-receipt migration. No full SF2 production build ran.")

    section('Z. Determinism','Insertion-order independence and full canonical reconstruction pass. Layers 7–9 reproduce preserved scientific content exactly. Current checkpoint restart passes for migration, capsule continuation, bent passage, and SF2. New root schemas intentionally change representation hashes; legacy cross-checks establish one-to-one scientific equivalence.')

    sections[-1]+='\nLayer-11 candidate replay evidence:\n\n'+json.dumps(read('CAPSULE11_REPLAY.json'),indent=2)+'\n'
    section('AA. Tests',json.dumps(tests,indent=2))

    section('AB. Classification',classification+". Canonical blocks are enabled only for explicitly migrated native checkpoints. Broader adoption remains subject to the task classification and continuation evidence. Retain legacy Patricia readers and sealed authorities. Patricia cross-file deduplication was not implemented or measured because block adoption succeeds. The main limitation is all-block rewriting under SHA dispersion; retained allocator/geometry RSS and raw receipt storage also remain.")

    if not complete:sections[-1]+='\nThe target is incomplete: layer 11 passed; layer 12 failed raw quality-table publication for disk capacity, and no retry fit the conservative execution budget. No layer-12 candidate is qualified.\n'
    section('AC. Recommended next task',('Focused retained-memory accounting for native/allocator/receipt state before another broad production campaign. Retained RSS still grows despite lower peaks.' if complete else 'Complete bounded capsule continuation with sufficient scratch capacity, addressing raw receipt/checkpoint storage pressure before broad production advancement. Do not repeat the already-qualified layer-7–9 comparison or layer-10 audit without a direct regression.')+' No scientific gate changes. This storage result is not broader geometry qualification.')

    report='## Decision question\n\nCan V4 replace fine-grained persistent Patricia-node storage with canonical authenticated sorted blocks, or otherwise compact Patricia storage, while preserving exact certificate authority and reducing per-layer storage/write amplification?\n\n'+'\n'.join(sections)

    (OUT/'report.md').write_text(report,encoding='utf-8')

    evidence={}

    for p in sorted(OUT.rglob('*')):

        if not p.is_file() or p.name in ('AUTHORITY.json','HASHES.json') or 'checkpoints' in p.parts or 'arrays' in p.parts or 'interface' in p.parts:continue

        if p.suffix in ('.json','.jsonl','.md') and p.stat().st_size<16*1024*1024:evidence[p.relative_to(OUT).as_posix()]=sha(p)

    sources={p.relative_to(ROOT).as_posix():sha(p) for parent in ('src','tools','tests') for p in sorted((ROOT/parent).rglob('*.py'))}

    authority=dict(task='QualifyV4CanonicalBlockAuthenticatedStorage',classification=classification,starting_head=read('TASK_MANIFEST.json')['head'],report_sha256=sha(OUT/'report.md'),evidence=evidence,source_sha256=sources,adopted=True,adoption_scope='Explicit migrated checkpoints only',continuation_target_complete=complete,authority='V4CertificateReceiptBlocks/1',map_schema='V4CanonicalSortedBlocks/1',prefix_bits=8,benchmarks=baseline,continuation=continuation,layer10_audit_passed=True,tests=tests,limitations=['All capsule blocks rewrite','Retained RSS growth remains','Cross-file Patricia duplicate bytes: Not measured','Raw receipt and proof storage not compacted'],elapsed_wall_seconds=time.time()-read('TASK_MANIFEST.json')['started_unix'])

    (OUT/'AUTHORITY.json').write_text(json.dumps(authority,indent=2)+'\n')

    hashes={name:sha(OUT/name) for name in ('report.md','AUTHORITY.json')}

    (OUT/'HASHES.json').write_text(json.dumps(hashes,indent=2)+'\n');print(json.dumps(hashes,indent=2))



if __name__=='__main__':run()

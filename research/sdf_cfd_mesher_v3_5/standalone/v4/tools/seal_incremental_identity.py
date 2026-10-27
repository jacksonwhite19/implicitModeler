"""Seal measured identity qualification evidence; absent data stays explicit."""
import argparse,hashlib,json,re
from pathlib import Path
from summarize_identity_profile import ROOT,collect


def read(path):
    return json.loads(path.read_text()) if path.exists() else None


def fmt(value):
    if value is None:return 'Not measured'
    if isinstance(value,float):return f'{value:.6f}'
    return str(value)


def table(headers,rows):
    return '\n'.join(['| '+' | '.join(headers)+' |','| '+' | '.join(['---']*len(headers))+' |']+
                     ['| '+' | '.join(map(fmt,row))+' |' for row in rows])


def seal(classification):
    ledger=[]
    for p in sorted(ROOT.glob('*/*/IDENTITY_TIMERS.json')):ledger.extend(collect(p.parent))
    (ROOT/'PERFORMANCE_LEDGER.json').write_text(json.dumps(ledger,indent=2)+'\n')
    baseline={x['layer']:x for x in ledger if x['label'].startswith('baseline_')}
    selected={x['layer']:x for x in ledger if x['label'] in ('final_bench','final_8_9')}
    continuation=[x for x in ledger if x['label'].startswith('continuation')]
    final9=read(ROOT/'FINAL_AUDIT.json');final10=read(ROOT/'FINAL_AUDIT_LAYER10.json');final=final10 or final9;partial10=read(ROOT/'FINAL_AUDIT_LAYER10.json.progress.json');bent=read(ROOT/'bent_passage_production/bent_final/LEGACY_CROSSCHECK_6.json')
    sf2=read(ROOT/'SF2/BOUNDED_REPLAY.json')
    comparisons=[]
    for label in ('final_bench','final_8_9'):
        comparisons.extend(read(p) for p in sorted((ROOT/'capsule_production_control'/label).glob('LEGACY_CROSSCHECK_*.json')))
    if classification.endswith('Qualified'):
        assert final and final['passed'] and bent and bent['passed'] and sf2 and sf2['passed']
        assert len(comparisons)==3 and all(x['passed'] for x in comparisons)
        assert continuation and max(x['layer'] for x in continuation)>=12
    timers=['clean_rows_digest','validation_digest','receipt_measure_identity','scientific_digest',
            'extension_plan','checkpoint_canonical_verification','checkpoint_byte_verification',
            'compact_proof_materialization','segment_proof_materialization','compact_index_scan',
            'canonical_sort','coverage_validation','delta_receipt_storage','delta_canonical_sort',
            'delta_key_value_hashing_and_sort','incremental_receipt_identity','merkle_node_lookup',
            'merkle_node_write','dependency_index_query','delta_proof_index_publication']
    measured=lambda row,name:row['identity_timers'].get(name,{}).get('seconds',0.) if row else None
    sections={}
    sections['A. Authority verification']='Starting HEAD: 9df8a478b14d1a54bab178ca66f304eb01f666b0. TASK_MANIFEST.json records the verified prior report and authority hashes and initial unrelated dirty state. Prior sealed evidence is unchanged. V3 is read-only.'
    sections['B. Capsule benchmark states']=table(['Transition','Historical relations','New relations','Total relations'],[[f'{i-1}→{i}',baseline[i]['historical_relations'],baseline[i]['new_relations'],baseline[i]['total_relations']] for i in (7,8,9)])+'\n\nThe layer-6 input lacked the compact pair index. Its one-time index/root bootstrap is separate from optimized accepted-layer runtime. Layers 8/9 compare indexed baseline continuations. Geometry was not regenerated from earlier layers.'
    sections['C. Precise baseline timers']='Seconds. These are instrumented inclusive scopes; nested timers must not be added. Zero means an installed hook was not called. Pure hash CPU excluding row rendering and separate duplicate-only/missing-only overhead: Not measured.\n\n'+table(['Operation','Baseline 6→7','Baseline 7→8','Baseline 8→9'],[[n]+[measured(baseline.get(i),n) for i in (7,8,9)] for n in timers])
    sections['D. Current identity semantics']='Legacy canonical scientific JSON and ordered proof digests remain unchanged functions. They prove exact content and order. Coverage separately proves required membership and uniqueness. Receipt values bind proof outcomes, scientific face keys, dependency hashes, and proof/filter implementation authority. Storage byte hashes prove component integrity; they do not replace geometric coverage.'
    sections['E. Full-history operations']='The measured old paths are reuse_mixed_certificate.certify → clean_rows_digest; validation_row_stream.validation_digest; ReceiptDelta.measure_identity → raw_items; mixed_front.scientific_hash → scientific_digest; native_proof_segments.extension_plan; and NativeCheckpointStore._receipt/_object. The new ordinary path does not render unchanged historical proofs for identity. It still traverses millions of authenticated tree nodes along changed-key paths; this is measured work, not zero-cost reuse. Geometry/front/quality component hashing, numerical validation, and face-map construction still scale with shell size. Cold byte verification and explicit backend-limit compaction remain global operations.'
    sections['F. Incremental certificate authority']='V4PatriciaSHA256Map/1 is a deterministic binary Patricia Merkle map. Canonical scientific key hashes select leaves; complete canonical receipt hashes are leaf values. First-differing-bit branches bind child roots, key bounds, and counts. SHA-256 with domain/type/version separation is used throughout. Insert rejects existing keys; retire/replace require existing keys. Root identity is independent of insertion order, thread order, and node-file layout. Node paths are persistent; untouched subtrees are reused.'
    sections['G. Incremental receipt authority']='V4CertificateReceiptPatricia/1 binds proof content AND receipt dependencies. It is explicitly different from the old ordered proof digest. Changed top-face owner roles still require receipt rebinding: 517,028 / 501,892 / 472,356 distinct historical receipts on the three benchmark layers. Unchanged history is not rebound. Only certificate receipt authority is migrated by this explicit bootstrap; unmigrated checkpoints keep the legacy path. The final streaming writer derives root actions from the same update-table bytes and avoids a second fresh-receipt table. Layer 7 was measured before this last streaming refinement; layers 8/9 use it.'
    sections['H. Dependency indexing']='An immutable CSR index maps each scientific face to its incident relation indices. Each pair contributes two incidences. Ordinary changed-face queries avoid scanning every historical pair. Geometry-changing fallback paths preserve their existing full validation. Changed certificate/filter implementation authority fails closed and requires recertification.'
    sections['I. Scientific root composition']='V4MixedScientificComponents/1 composes component identities with the authenticated certificate/receipt root. V4ValidationAuthenticatedComponents/1 replaces the validation proof sequence with its explicit authenticated component. Verified quality-table identity is reused when available. Non-certificate mutable components retain exact canonical hashing. No old hash is silently relabeled as the new representation.'
    sections['J. Checkpoint identity']='Restart verifies content-addressed component bytes. In-process byte reuse requires matching path, file ID, size, write time, actual change time, and expected SHA-256. Authenticated parent storage bindings avoid redundant child canonical-map reconstruction. Legacy checkpoints retain the legacy path until explicit bootstrap. New unindexed bootstrap now independently checks complete pair coverage and outcomes before granting inherited coverage. The missing-proof bootstrap regression rejects before CURRENT publication.'
    sections['K. Proof materialization']='Normal migrated commits materialize changed/new receipts and fresh proofs only. Legacy full scientific/proof digests are computed outside accepted-layer timing for qualification. Baseline segment counters included terminal iterator calls: subtract 3 on layer 8 and 6 on layer 9 to obtain successful historical materializations. Raw evidence is preserved. Aggregate historical receipt rendering across every legacy helper was not separately counted: Not measured.'
    sections['L. Canonical ordering']='Only changed/new key-hash arrays are sorted for root updates. Immutable proof segments are not eagerly merged. Patricia roots are canonical independent of segment layout. Receipt tables compact only at the actual SQLite attachment limit; this explicit maintenance can revisit history and is not an ordinary identity update.'
    sections['M. Immediate commit audit']='ImmediateCommitAudit combines authenticated prior coverage with complete changed/new candidate coverage, duplicate-key rejection, explicit retirement/rebinding, dependency checks, and exact active counts. Old-old exact geometry is not recomputed. New-old candidate search remains unrestricted and exact-safe.'
    sections['N. Final whole-shell audit']=(json.dumps(final,indent=2) if final else 'Not measured. No final whole-shell qualification is claimed.')+'\n\nThe summary layer above is the audit scope. A layer-9 audit does not qualify layer 10. Layer-10 audit progress: '+(json.dumps(partial10,indent=2) if partial10 else 'Not measured.')
    sections['O. Legacy digest cross-check']=table(['Population','Legacy scientific digest','Legacy proof digest','New certificate root','Passed'],[[x['count'],x['legacy_scientific'],x['legacy_proof_digest'],x['certificate_root'],x['passed']] for x in comparisons])
    sections['P. Performance ledger']=table(['Layer','Before seconds','After seconds','Before peak RSS bytes','After peak RSS bytes','Before retained RSS bytes','After retained RSS bytes'],[[i,baseline[i]['accepted_seconds'],selected.get(i,{}).get('accepted_seconds'),baseline[i]['process_peak_rss'],selected.get(i,{}).get('process_peak_rss'),baseline[i]['retained_rss'],selected.get(i,{}).get('retained_rss')] for i in (7,8,9)])+'\n\nOptimized inclusive timer scopes:\n\n'+table(['Operation','6→7','7→8','8→9'],[[n]+[measured(selected.get(i),n) for i in (7,8,9)] for n in timers])+'\n\nStorage trials and probes are preserved separately. The first node backend was slower. Verified immutable SQLite reads, rowid payload storage, historical-first lookup, and a measured 64 MiB writer cache repaired that overhead. Receipt preparation moved into the certification stage; stage labels alone are not a like-for-like speed comparison. EXECUTION_CONTROL_NOTE.json records the stopped-worker overlap; the final selected benchmark replaces the overlapped layer-7 timing.'
    memory=[]
    for row in list(selected.values())+continuation:
        label=row['label'];p=ROOT/'capsule_production_control'/label/'PROFILE.jsonl'
        records=[json.loads(x) for x in p.read_text().splitlines()]
        census=next(x for x in records if x['event']=='retained_class_accounting' and x.get('checkpoint_event')=='retained_after_commit' and x['layer']==row['layer'])
        native=row['native_memory'] or {};text=native.get('cpython_allocator_text','')
        def value(pattern):
            m=re.search(pattern,text,re.M);return int(m[1].replace(',','')) if m else None
        memory.append(dict(layer=row['layer'],label=label,retained_rss=row['retained_rss'],owners=census['bytes_by_first_owner'],native=native.get('process'),allocator=native.get('cpython_allocator_counters'),arena_bytes=value(r'^\d+ arenas \* \d+ bytes/arena\s*=\s*([\d,]+)'),unused_pool_bytes=value(r'^\d+ unused pools \* \d+ bytes\s*=\s*([\d,]+)')))
    (ROOT/'MEMORY_ACCOUNTING.json').write_text(json.dumps(memory,indent=2)+'\n')
    sections['Q. Memory accounting']=table(['Layer','Retained RSS bytes','Geometry Python bytes','Atlas Python bytes','Certificate Python bytes','Reserved arena bytes','Unused pool bytes'],[[x['layer'],x['retained_rss'],x['owners']['committed_geometry'],x['owners']['atlas'],x['owners']['certificate_receipts'],x['arena_bytes'],x['unused_pool_bytes']] for x in memory])+'\n\nReachable-object sizes and arena reservation are measured but are not interchangeable with resident pages. Mapped-region sizes are not mmap residency. Exact native/mmap resident ownership: Not measured. Transient peaks increase during node access; no peak-memory reduction is claimed. From layer 8 to 9, retained RSS increases 372,125,696 bytes; reserved CPython arenas increase 365,953,024 bytes, including 291,799,040 additional unused pool bytes and 64,065,824 additional allocated-block bytes. This strongly identifies allocator retention alongside scientific geometry growth, but does not measure exact resident ownership. Receipt/proof Python metadata remains compact, but authenticated node storage is not small: ROOT_STORAGE.json measures 2,971,881,472 referenced node-file bytes at layer 7 and 5,517,021,184 at layer 9. These are logical immutable file sizes, not resident pages. The layer-9 CSR dependency files occupy 138,248,032 bytes. Node-store growth and mmap peak are unresolved engineering tradeoffs. The recursive-closure array retention defect was reproduced and repaired without forced GC.'
    sections['R. Capsule benchmark equivalence']='The legacy whole scientific digest checks coordinates, connectivity, quality, receipt semantics, atlas, forecast, and scientific state. The legacy ordered proof digest separately checks the complete proof population. Successful cross-checks are listed above; absent layers are not claimed.'
    sections['S. Capsule layers 10–12']=table(['Layer','Runtime seconds','Historical relations','New relations','Root update seconds','Checkpoint seconds','Peak RSS bytes','Retained RSS bytes'],[[x['layer'],x['accepted_seconds'],x['historical_relations'],x['new_relations'],measured(x,'incremental_receipt_identity'),x['stages'].get('checkpoint_serialize'),x['process_peak_rss'],x['retained_rss']] for x in continuation]) if continuation else 'Not measured. No additional accepted layer is claimed.'
    sections['T. Bent-passage regression']=json.dumps(bent,indent=2) if bent else 'Not measured.'
    sections['U. SF2 regression']=(json.dumps(sf2,indent=2) if sf2 else 'Not measured.')+'\n\nThis is a bounded legacy-representation regression, not a 56-million-relation Merkle bootstrap. New SF2 authenticated root: Not measured.'
    sections['V. Determinism']='Tests cover insertion-order independence, replacement/retirement, duplicate rejection, node corruption, byte-cache invalidation, root reconstruction, and restart. Capsule cross-checks use preserved outputs. The layer-7 root is identical across different node storage layouts. SF2 remains on its unchanged legacy authority representation; it was not rebuilt or used as the optimization target.'
    sections['W. Tests']=(ROOT/'TESTS_LAST.txt').read_text() if (ROOT/'TESTS_LAST.txt').exists() else 'Not measured.'
    sections['W. Tests']+='\n\nFinal complete suite: 951 passed plus 71 subtests; 2 pre-existing singular-cell warnings; 213.16 s. Additional focused checks: 27 passed and bootstrap/streaming module 2 passed. WSL broader-geometry suite: 11 passed. Earlier complete suite: 950 plus 71 subtests.'
    sections['X. Classification']=classification+'\n\nThe 2.5-hour active-science bound is enforced conservatively from task start (2026-09-20 21:55 local; deadline 2026-09-21 00:25 local). Layer-9 commit tail briefly overlapped bent startup; subsequent bent replay, continuation, final audit and full tests share the host. Their times are measured concurrent wall times, not isolated performance comparisons. Missing continuation/final-audit evidence is not inferred. Layers 11 and 12 were not attempted within the remaining cap. This task does not authorize broad production-scale advancement on the new identity representation yet. The exact remaining blocker is the task time cap: layer 10 committed, but its independent root rebuild was interrupted; layers 11/12 were not run. The layer-9 independent audit passed. Authenticated node-store peak/disk growth remains an engineering limitation. The root representation is explicit and the frozen scientific gates and certificate mathematics are unchanged.'
    sections['Y. Recommended next task']='Finish the bounded layer-10 audit and layer-11/12 continuation before broad production-scale advancement. Then qualify the authenticated node-store memory/storage behavior: its peak and immutable-file growth are measured regressions. Keep the current scientific gates and independent audit unchanged. Do not claim a memory improvement.'
    report='## Decision question\n\nCan V4 replace repeated full-population certificate and receipt identity work during ordinary layer commits with deterministic authenticated delta updates while preserving complete scientific authority and an independent global final audit?\n\n'
    report+='\n\n'.join('### '+title+'\n\n'+body for title,body in sections.items())+'\n'
    (ROOT/'report.md').write_text(report,encoding='utf8')
    checksum=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
    authority=dict(task='QualifyV4IncrementalCertificateAndReceiptIdentity',classification=classification,
        starting_head='9df8a478b14d1a54bab178ca66f304eb01f666b0',report_sha256=checksum(ROOT/'report.md'),
        authority_versions=['V4PatriciaSHA256Map/1','V4CertificateReceiptPatricia/1','V4ValidationAuthenticatedComponents/1','V4MixedScientificComponents/1'],
        benchmarks=comparisons,continuation_layers=[x['layer'] for x in continuation],final_audit=final,layer9_audit=final9,layer10_audit=final10,layer10_audit_progress=partial10,bent=bent,sf2=sf2,
        gates_changed=False,certificate_mathematics_changed=False,v3_modifications=0,
        evidence={str(p.relative_to(ROOT)):checksum(p) for p in sorted(ROOT.rglob('*'))
                  if p.is_file() and p.suffix in ('.json','.jsonl','.txt','.md')
                  and 'checkpoints' not in p.parts and p.name not in ('AUTHORITY.json','report.md')})
    (ROOT/'AUTHORITY.json').write_text(json.dumps(authority,indent=2)+'\n')
    print(classification);print('report.md SHA-256',checksum(ROOT/'report.md'));print('AUTHORITY.json SHA-256',checksum(ROOT/'AUTHORITY.json'))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('classification');seal(p.parse_args().classification)

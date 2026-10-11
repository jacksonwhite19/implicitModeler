"""Build compact performance evidence from measured, sealed controls."""
from pathlib import Path
import subprocess
import shutil
import capsule_user_domain_handoff as h
from retain_persistent_evidence import allocation

OUT = Path(__file__).resolve().parents[1] / 'results/persistent_publication_optimization'
ROOT = Path(__file__).resolve().parents[3]

if __name__ == '__main__':
    pre = h.read(OUT / 'PREFLIGHT.json')
    status = subprocess.check_output(['git','status','--porcelain=v1'], cwd=ROOT, text=True).splitlines()
    assert all(line in status for line in pre['unrelated_status'])
    assert len(pre['unrelated_status']) == 155
    a = OUT / 'normal_candidate_confirm_prism'
    b = OUT / 'normal_candidate_lookup_replay_prism'
    data = dict(classification='V4PersistentGeometryIncidenceAndWriteOncePublicationOptimizationQualified', starting_head=pre['head'],
        epoch=pre['epoch'], epoch_renewal_seconds=0, baseline=h.read(a/'NORMAL.json'), optimized=h.read(b/'NORMAL.json'),
        baseline_reopen=h.read(a/'REOPEN.json'), optimized_reopen=h.read(b/'REOPEN.json'),
        generation_baseline=h.read(a/'GENERATION_PHASES.json'), generation_optimized=h.read(b/'GENERATION_PHASES.json'),
        deep=h.read(OUT/'DEEP_candidate_lookup_replay.json'), core=h.read(OUT/'CORE_REGRESSION.json'),
        certificate=h.read(OUT/'CERTIFICATE_REGRESSION.json'), replay=h.read(OUT/'DETERMINISTIC_REPLAY.json'),
        lookup_scaling=h.read(OUT/'LOOKUP_SCALING.json'), storage=h.read(OUT/'STORAGE.json'))
    # Correct the earlier storage collector's label. GetCompressedFileSizeW
    # measured stored bytes, not NTFS allocation. Deleted allocation cannot be
    # reconstructed; retain exact stored-byte totals and the free-space delta.
    storage = data['storage']
    for key in ('control_allocation','reclaimed_allocation','retained_allocation'):
        storage[key.replace('allocation','stored_bytes')] = storage.pop(key)
    storage['physical_retained_now'] = sum(allocation(p) for p in OUT.rglob('*') if p.is_file())
    storage['physical_reclaimed_free_space_delta'] = storage['ending_free']-storage['before_cleanup_free']
    storage['physical_reclaimed_limit'] = 'Volume free-space difference includes unrelated concurrent volume activity; deleted per-file NTFS allocation was not captured.'
    storage['ending_free'] = shutil.disk_usage(OUT).free
    h.durable_json(OUT/'STORAGE.json', storage)
    h.durable_json(OUT/'REPORT.json', data)
    lines=[data['classification'],'',
        'The opt-in persistent path qualifies against sealed L10 to L11. Production authority was not replaced.',
        'Install prism_persistent_hotpath before prism_hotpath_v2. Enroll an old parent only with the independently deep-verified numeric and source-lookup roots. New children carry authenticated delta references.',
        '', '| Phase | Fresh baseline s | Optimized s |','|---|---:|---:|']
    for key in ('parent_open','admission','generation','assembly','publication'):
        lines.append(f"| {key} | {data['baseline']['phases'][key]['seconds']:.6f} | {data['optimized']['phases'][key]['seconds']:.6f} |")
    for label,key in [('Admission to publication','admission_to_publication')]:
        lines.append(f"| {label} | {data['baseline'][key]:.6f} | {data['optimized'][key]:.6f} |")
    lines += [f"| Fresh reopen | {data['baseline_reopen']['metrics']['seconds']:.6f} | {data['optimized_reopen']['metrics']['seconds']:.6f} |",
        f"| Open to fresh reopen | {data['baseline']['metrics']['seconds']+data['baseline_reopen']['metrics']['seconds']:.6f} | {data['optimized']['metrics']['seconds']+data['optimized_reopen']['metrics']['seconds']:.6f} |",'',
        'Persistent state: immutable numeric cell/face segments, binary-search source-face index, newest delta ownership, lazy copied-on-read component maps, authenticated parent/version closure and topology counts. New rows: 16,895 vertices; 33,786 cells; 84,465 faces; 33,786 changed terminal owners. No complete prefix owner dictionary is rebuilt.',
        'Publication: exact receipt dependency set written once; process witness holds NTFS deny-write guards; canonical quality blobs encoded once; both independent database readback checks remain and compare every canonical blob. Stream/cache identities are reused only with explicit guards and dependency checks. Mutable acceptance and plan checks remain. The small transaction plan is still rewritten by incidence integration. Large legacy JSON and SQLite representations remain.',
        'SQLite: DELETE journal, synchronous=FULL, uniqueness, integrity checks, commit, fsync, writer lock, compare-and-swap pointer, rollback/recovery and retry remain. Quality database bytes unchanged; proof index bytes unchanged. No new SQLite format or relaxed durability.',
        'Full exact degree and volume checks remain. No new incremental theorem is claimed for these obligations. Deep reconstruction independently checks all 371,646 cells and 962,901 faces; ownership, topology summaries, version closure, degree/volume and 219,609 changed quality rows match sealed science.',
        'Quality: 67,572 affected historical rows; 33,786 completed determinant contexts; 152,037 new rows; 219,609 delta rows. No historical table reconstruction. Proof accounting: 30,913,093 inherited + 2,422,939 new = 33,336,032; 270,288 changed-face probes; 1,181,773 postings; 974,421 changed and 29,938,672 unchanged bindings. Complete prior-pair scans = 0.',
        'Scientific root: 6b11b14c7b519467ad49a126261f0ddd608692d82ca387160fcc1b7ab10c5870. Independent final replay has identical implementation, scientific components and numeric arrays. Access roots differ because private paths and current file tokens differ.',
        'Core regression: exact C1 coordinates, tets, boundary, ownership, quality and callback binary64. Full saved certificate: 19,941,573 rows, canonical hash 9ce3df428c1784391429ebf18a3385429fde0b679c0fc85b32ec9499ac715c1a. No new core authority.',
        'Tests: final combined suite 368 passed, zero failed; separate transaction run 39 passed (overlapping coverage, not an additive total). Current/stale epoch fixture derives its current anchor from qualified renewal and still rejects original IncrementalEpochChanged. No scientific authority was rewritten.',
        'Profile: baseline generation 267,100,231 calls, 14.6 million deepcopy calls; assembly 104,156,121 calls; publication 136,649,767 calls. Publication included 9.69 million memmap indexing calls, 743,319 JSON loads and 743,292 decompressions. Profile times overlap and are not clean production timings.',
        'Concurrency: lookup 1/2/4 threads have identical digest; serial is fastest. Existing four-worker degree path retained. No new compiled dependency.',
        'Limitations: 60 seconds not reached. Assembly slightly slower; quality and proof subphases slightly slower. Full global face enumeration/canonicalization, degree witnesses, geometry-delta encoding, atlas JSON and quality database remain. No claim that all history-proportional work is removed. Index enrollment is qualification-only, boot-scoped and excluded from normal timing.',
        'Safety: original 155 dirty entries preserved; V3 not edited; sealed ancestors unchanged; no hardlinks created. Duplicate controls removed only after hashes and evidence were saved. Retained parent/delta indexes are local artifacts; recreate with bootstrap tools when unavailable or epoch changes.',
        'Recommended next task: OptimizeV4QualityDeltaPersistenceAndRemainingGlobalGeometryChecksFromSealedL10ToL11.', '']
    (OUT/'REPORT.md').write_text('\n'.join(lines), encoding='utf-8')
    h.durable_json(OUT/'GIT_PRESERVATION.json',dict(starting_head=pre['head'], unrelated_entries=155, original_status_preserved=True, v3_modified_by_task=False, hardlinks_created=0))

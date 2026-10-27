"""Retain compact performance evidence; list only owned disposable controls."""
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import xml.etree.ElementTree as ET
import capsule_user_domain_handoff as h
from path_preserving_deduplication import metadata
from profile_meshing_hotpath import OUT,SEALED,START


def collect():
    assert h.read(OUT/'PRISM_EQUIVALENCE.json')['passed']
    assert h.read(OUT/'CORE_REPLAY_EQUIVALENCE.json')['passed']
    assert h.read(OUT/'PUBLICATION_RELEASE_REOPEN.json')['passed']
    assert h.read(OUT/'V3_PRESERVATION.json')['unchanged']
    controls={};profiles=OUT/'profiles';profiles.mkdir(exist_ok=True)
    for directory in sorted(OUT.iterdir()):
        if not directory.is_dir() or not directory.name.endswith(('_prism','_core','_certificate')):continue
        records={}
        for p in [*directory.glob('*.json'),*directory.glob('C1/*.json')]:
            if p.stat().st_size<2_000_000:
                records[str(p.relative_to(directory))]=h.read(p)
        controls[directory.name]=records
        for p in directory.glob('*.prof'):
            shutil.copyfile(p,profiles/(directory.name+'_'+p.name))
        for p in directory.glob('*PROFILE.json'):
            shutil.copyfile(p,profiles/(directory.name+'_'+p.name))
    h.durable_json(OUT/'CONTROLS.json',controls)
    shutil.copyfile(OUT/'raw_release_baseline_core/QUERIES.npy',OUT/'CORE_QUERIES.npy')
    records=[]
    for p in sorted(OUT.rglob('*')):
        if p.is_file():
            m=metadata(p)
            records.append(dict(path=str(p.relative_to(OUT)),logical=p.stat().st_size,
                physical=m['cluster_bytes'],sha256=h.file_hash(p)))
    h.durable_json(OUT/'ALLOCATION_BEFORE_CLEANUP.json',dict(files=records,
        logical=sum(r['logical'] for r in records),physical=sum(r['physical'] for r in records),
        free=shutil.disk_usage(OUT).free,temporary_peak='Not continuously measured; this is the pre-cleanup allocation'))
    disposable=[str(p.resolve()) for p in OUT.iterdir() if p.is_dir() and
        (p.name.endswith(('_core','_prism','_certificate')) or p.name=='publication_control') and
        p.name!='raw_release_optimized_certificate']
    disposable += [str((OUT/n).resolve()) for n in ('old_json_control.json','new_json_control.json')]
    h.durable_json(OUT/'CLEANUP_PLAN.json',dict(root=str(OUT.resolve()),paths=disposable))


def preservation():
    before=h.read(OUT/'PREFLIGHT.json')
    status=subprocess.check_output(['git','status','--porcelain=v1','-z'])
    def unrelated(raw):
        return [r for r in raw.split(b'\0') if r and not any(k in r for k in
            (b'production_hotpath_optimization',b'/optimized_',b'/test_optimized_',b'/OPTIMIZED_HOTPATH.md',
             b'/profile_meshing_hotpath.py',b'/verify_hotpath_controls.py',b'/characterize_hotpath_results.py',b'/finalize_hotpath_evidence.py'))]
    current=unrelated(status);prior=unrelated(bytes.fromhex(before['status_hex']))
    assert current==prior and len(current)==155
    from incremental_transaction import Store
    record=h.read(SEALED/'COMPLETE_AUTHORITY.json')
    with Store(SEALED/'complete_mesh').open(record['access']),h.CommittedState(record['parent']) as parent:parent.check()
    a=h.read(SEALED/'AUTHORITY.json');h.verify_record(a)
    for name,sha in a['reports'].items():assert h.file_hash(SEALED/name)==sha
    files=[p for p in OUT.rglob('*') if p.is_file()]
    allocation=sum(metadata(p)['cluster_bytes'] for p in files)
    h.durable_json(OUT/'PRESERVATION.json',dict(unrelated_entries=155,unrelated_status_byte_equal=True,
        production_authority_unchanged=True,production_current_access_passed=True,
        attested_hardlinks_created=0,legacy_fallbacks=0,retained_allocation=allocation,
        ending_free=shutil.disk_usage(OUT).free,starting_free=before['free'],
        retained_files=len(files),v3_unchanged=h.read(OUT/'V3_PRESERVATION.json')['unchanged']))


def report():
    import psutil
    controls=h.read(OUT/'CONTROLS.json')
    controls={name:{key.replace('\\','/'):value for key,value in records.items()}
        for name,records in controls.items()}
    new=controls['raw_release_optimized_prism']
    olddir=SEALED.parent/'recursive_layers9to11/L11'
    phases=('admission','generation','assembly','publication')
    old={s:h.read(olddir/(s.upper()+'_MEASURED.json'))['metrics'] for s in phases}
    fresh={s:new[s+'_MEASURED.json']['metrics'] for s in phases}
    coreold=controls['raw_release_baseline_core']['C1/RESULT.json']
    corenew=controls['raw_release_optimized_core']['C1/RESULT.json']
    cert=controls['raw_release_optimized_certificate']['RESULT.json']
    profile=controls['baseline_certificate']['RESULT.json']
    storage=h.read(OUT/'PRESERVATION.json');before=h.read(OUT/'ALLOCATION_BEFORE_CLEANUP.json')
    deleted=[r for r in before['files'] if not (OUT/r['path']).exists()]
    tests={}
    for name in ('TESTS','TRANSACTION_TESTS'):
        suites=ET.parse(OUT/(name+'.xml')).getroot().iter('testsuite')
        totals={k:0 for k in ('tests','failures','errors','skipped')}
        for suite in suites:
            for k in totals:totals[k]+=int(suite.get(k,0))
        assert not totals['failures'] and not totals['errors']
        tests[name]=totals
    for name in ('PRISM_EQUIVALENCE','PRISM_REOPEN','CORE_REPLAY_EQUIVALENCE','CERTIFICATE_REOPEN','PUBLICATION_RELEASE_REOPEN'):
        assert h.read(OUT/(name+'.json'))['passed']
    assert cert['exact_oracle_equal'] and profile['exact_oracle_equal']
    sources={str(p.relative_to(SEALED.parents[1])):h.file_hash(p)
        for p in Path(__file__).parent.glob('optimized_*') if p.is_file()}
    implementation=h.identity(sources)
    body=dict(classification='V4ProductionMeshingPerformanceOptimizationQualified',starting_head=START,
        ending_head='The enclosing task commit; reported after commit in the final response',
        sealed=h.read(OUT/'VERIFIED.json'),implementation=implementation,source_files=sources,
        hardware=dict(physical_cores=psutil.cpu_count(logical=False),logical_cpus=psutil.cpu_count(),ram_bytes=psutil.virtual_memory().total),
        baseline_prism=old,optimized_prism=fresh,
        prism_seconds=sum(r['seconds'] for r in fresh.values()),
        prism_with_reopen_seconds=sum(r['seconds'] for r in fresh.values())+h.read(OUT/'PRISM_REOPEN.json')['metrics']['seconds'],
        baseline_core=coreold,optimized_core=corenew,certificate=cert,baseline_profile=profile,
        thread_scaling=h.read(OUT/'THREAD_SCALING.json'),predicate_families=h.read(OUT/'PREDICATE_FAMILY_DIAGNOSTICS.json'),
        tests=tests,storage=dict(**storage,pre_cleanup_allocation=before['physical'],deleted_files=len(deleted),
            reclaimed_file_allocation=sum(r['physical'] for r in deleted),temporary_peak='Not continuously measured'),
        publication=h.read(OUT/'PUBLICATION_RELEASE_CONTROL.json'),
        limitations=['60-second layer target not reached','Profiled and unprofiled timings are separated',
            'Historical normal-path timings are reference runs, not paired isolated timing trials',
            'Parent RSS is not combined concurrent worker peak RSS',
            'No new persistent geometry-index authority was introduced',
            'No complete-mesh or CFD scientific claim beyond the sealed authority'])
    h.durable_json(OUT/'REPORT.json',body)
    lines=[body['classification'],'',
        'Qualified exact optimizations for the sealed capsule workload. Production authority stayed unchanged. The 60 s layer target was not reached.',
        '',f'Starting HEAD: {START}. Ending HEAD: the enclosing task commit, printed after commit.',
        'Branch: feature/sdf-cfd-mesher-v4-advancing-front. No stale advancement task was resumed.','',
        'Verified complete-mesh authorities:',
        '- Scientific: a1e9d5906a7a723ec9906c938a52a25921e23d867aa9f22a489162cba9770b4f',
        '- Content: bbc503c3a04dd5d4f5083d9237d20f51dafbc4578b437b1364c01ff4de5f5c81',
        '- Access: a2d8e3d3722fa3305cc5b23e87cd68120a70d79cdd732ef0d6c02a1dc7b566ba',
        '- Checkpoint: cb2061add4c1e4150126d15c48135161f66fe651c62c3bc8bd42d8cf104c01ac',
        '- Core qualification: ed9075a03ccc08315c25a7a35f6d99f184703bd7283e52da00d562d53b3f9fed','',
        'Normal L11 control (seconds; parent open is included in each stage):','',
        '| Phase | Sealed normal reference | Optimized normal control |',
        '|---|---:|---:|']
    lines += [f"| {s} | {old[s]['seconds']:.3f} | {fresh[s]['seconds']:.3f} |" for s in phases]
    lines += [f"| Admission through publication | {sum(r['seconds'] for r in old.values()):.3f} | {body['prism_seconds']:.3f} |",'',
        f"Fresh reopen: {h.read(OUT/'PRISM_REOPEN.json')['metrics']['seconds']:.3f} s. Including reopen: {body['prism_with_reopen_seconds']:.3f} s. This is approximately {body['prism_with_reopen_seconds']/60:.2f} times the 60 s objective.",
        'Historical normal timings are equivalent-workload references, not simultaneous isolated A/B timing trials. Scientific A/B comparisons are exact. Profiler overhead is excluded from the normal timings above.','',
        'Measured baseline profiles:',
        '- Full core certificate: 1807.308 s certification/hash plus 235.462 s serialization under cProfile; the retained unprofiled reference is 987.116 s.',
        '- Nested cumulative core costs: candidate enumeration 836.630 s (overlap queries 807.200 s); 10,065,552 filter calls 205.854 s; 19,941,573 row appends 160.504 s; canonical normalization/hash 463.667 s; serialization 235.462 s. These nested times must not be added.',
        '- Core profile: 59,824,719 JSON encoding calls, 209,612,106 normalization calls, and 39,883,146 proof-row materializations. Python traversal and record processing dominated; geometric predicates were not the sole cost.',
        '- Sizing profile: triangle distance 237,946 calls / 36.858 s cumulative; BVH bound 1,506,370 calls / 17.373 s; NumPy cross 951,786 calls / 19.987 s. These are nested profiled costs.',
        '- L11 admission profile: conservative_pairs 36.774 s. Generation: global degree 419.960 s, projected containing queries 116.514 s, and 213,432,942 gcd calls across exact work.',
        '- Assembly profile: 410,620 path-resolution calls / 19.299 s, 135,201 JSON decodes / 8.223 s, and seven indented JSON dumps / 56.785 s. Publication still performs changed-quality validation and indexed writes.','',
        'Implemented changes:',
        '- Exact native inclusive AABB queries; canonical pair order and complete changed-versus-prefix coverage remain unchanged.',
        '- Parallel calls to the unchanged Rust interval predicate, including identical triangle rotations, witnesses and exact fallback.',
        '- Packed pair/classification buffers; original payload interning order; native canonical integer-row encoding.',
        '- Ordered fixed gzip members, bounded compression queues, complete row coverage and fsync.',
        '- Read-only sizing geometry caches, identical row arithmetic and scalar BVH bounds; no field approximation.',
        '- Parallel exact global-degree witnesses with an exact projected-box index; every witness is retained.',
        '- Byte-identical native JSON indentation after standard JSON encoding; original fsync and atomic rename remain in place.','',
        'Native predicate scaling on all 19,941,573 pairs:','',
        '| Workers | Broad phase s | Predicate s | Speedup versus 1 | Efficiency |',
        '|---:|---:|---:|---:|---:|']
    scale=body['thread_scaling'];one=scale[0]['predicates']['seconds']
    for r in scale:
        speed=one/r['predicates']['seconds']
        lines.append(f"| {r['workers']} | {r['broad']['seconds']:.3f} | {r['predicates']['seconds']:.3f} | {speed:.2f}x | {100*speed/r['workers']:.1f}% |")
    lines += ['', 'Eight workers were selected. Every worker count produced identical pair and classification buffers. Scaling runs were sequential in one process; their RSS counters are process high-water marks, not independent per-worker peaks.',
        'Pair-buffer SHA-256: 186c3d876abaa4909578bd27256337e272865b65891b10c08115299b4b2eb970.',
        'Classification-buffer SHA-256: 054f56aaaace4705a9c4557279e7ae35571ebd37995b34134b21f6bccea83e72.','',
        'Complete optimized core certificate:','',
        f"- Certification/hash: {cert['certification_metrics']['seconds']:.3f} s; CPU {cert['certification_metrics']['cpu']:.3f} s.",
        f"- Serialization/fsync: {cert['serialization_metrics']['seconds']:.3f} s; CPU {cert['serialization_metrics']['cpu']:.3f} s.",
        f"- Total: {cert['certification_metrics']['seconds']+cert['serialization_metrics']['seconds']:.3f} s. Geometry preparation outside that scope is retained separately in CONTROLS.json.",
        '- Phase walls: geometry 2.314 s; exact broad phase 1.744 s; predicates/rotation 14.609 s; packed assembly/exact fallback 8.490 s; summary/legacy hash 7.431 s.',
        '- Required/new certified pairs: 19,941,573. Immutable prism proofs reused: 33,336,032. Combined coverage: 53,277,605.',
        '- Methods: supporting-plane separation 2,022,815; owned-edge 2,429,243; owned-vertex 15,479,930; exact hull separation 5,829; exact hull legal contact 3,756.',
        '- Forbidden/unresolved/missing/duplicate required pairs: 0. Complete canonical hash: 9ce3df428c1784391429ebf18a3385429fde0b679c0fc85b32ec9499ac715c1a.',
        '- Separate grouped diagnostics: supporting-plane 0.607 s; owned-vertex 8.893 s; owned-edge 0.858 s; exact separated 2.128 s; exact legal contact 2.257 s. These regrouped reruns are not additive production phases.',
        f"- Saved-certificate full stream verification: {h.read(OUT/'CERTIFICATE_REOPEN.json')['metrics']['seconds']:.3f} s; 19,941,573 rows checked against the sealed canonical digest. No sampling.",
        '- Old compressed writes: 92,648,498 B retained reference. New: 93,149,083 B. Gzip members add 500,585 B; the decompressed scientific stream is unchanged.','',
        'Core and sizing A/B:','',
        f"- Generation: {coreold['wall']:.3f} -> {corenew['wall']:.3f} s; callback: {coreold['callback']['seconds']:.3f} -> {corenew['callback']['seconds']:.3f} s.",
        f"- Callback: 28,366 calls, 300 exact-coordinate hits, {corenew['callback']['queries_per_second']:.1f} calls/s; {100*corenew['callback']['seconds']/corenew['wall']:.1f}% of generation wall. Triangle-test populations are unchanged, maximum 1,271.",
        '- All real query positions and returned binary64 values match. Farther-smaller-seed, boundary and invalid-input tests pass.',
        '- Outer discretization, raw core coordinates/connectivity, final Netgen coordinates/connectivity, interface owners and recomputed quality match exactly.',
        '- Final core remains 42,056 vertices / 190,387 tetrahedra; outer boundary 5,182 triangles; L11 interface 16,895 vertices / 33,786 triangles.',
        '- g = 0.26236426446749106 (0x1.0ca937be1b9dcp-2); h_max = 0.09470177661841601 m (0x1.83e602972fe65p-4); no regional caps. No sizing or geometry result changed.','',
        'Prism scientific equality and remaining costs:',
        '- Exact equality: forecast, direct screening arrays, admission pairs/codes/witnesses/rotations, generated coordinates/connectivity, geometry delta, quality, incidence, bindings, volume/degree results, proof payloads/codes and proof segment identity.',
        '- The independent candidate reproduced L11 content root 6b11b14c7b519467ad49a126261f0ddd608692d82ca387160fcc1b7ab10c5870 and fresh-reopened through recursive trusted access.',
        '- Complete prior-pair scans, old proof rewrites, certificate flattening and full-prefix scientific-state copies: 0. Normal reopen historical proof/quality row visits: 0.',
        '- Final generation subphases: projection 0.020 s; geometry/global incidence 38.802 s; quality/completed incidence 42.152 s; geometry delta 15.271 s; new proofs 9.557 s; global degree/added volume 38.387 s.',
        '- Full degree checks were not replaced by summaries. The separate serial/parallel controls matched all 371,646 witnesses. Final eight-worker degree wall: 25.696 s; worker CPU: 163.875 s.',
        '- Remaining dominant costs: geometry/topology construction, exact quality, authenticated row/path access, assembly and publication. Persistent geometry-index authority was not introduced; native broad-phase construction is now small.','',
        'Memory, I/O and durability:',
        f"- Core certificate peak RSS: {profile['certification_metrics']['peak_rss']} -> {cert['certification_metrics']['peak_rss']} B.",
        f"- Prism parent peak RSS: {max(r['peak_rss'] for r in old.values())} -> {max(r['peak_rss'] for r in fresh.values())} B. Final degree workers retained an additional aggregate 1,502,281,728 B; simultaneous aggregate peak was not measured.",
        '- Main packed allocations: uint32 candidate pairs 159,532,584 B; three-byte classification records 59,824,719 B; compatible uint64 proof pairs 319,065,168 B; payload indices 159,532,584 B. Python/container allocation is not fully attributed.',
        f"- Prism process reads: {sum(r['io']['read_bytes'] for r in old.values())} -> {sum(r['io']['read_bytes'] for r in fresh.values())} B; writes: {sum(r['io']['write_bytes'] for r in old.values())} -> {sum(r['io']['write_bytes'] for r in fresh.values())} B.",
        '- Process counters include worker IPC; they are not disk allocation. The degree-only eight-worker control wrote 103,018,565 process bytes without writing a mesh file. Exact per-file attribution of all process traffic is unavailable.',
        '- Proof index: 18.167 s / 225,748,540 process-write B; incidence: 1.192 s / 37,984,590 process-write B. The prior multi-gigabyte page-churn behavior did not return.',
        '- A 337,410,945 B quality JSON control was byte-identical: 3.338 -> 1.960 s, 41,060 -> 1 write operation. Process high-water RSS rose from 994,525,184 to 1,616,175,104 B in that sequential writer control.',
        '- DELETE journaling, synchronous=FULL, uniqueness, commit/fsync, stale/concurrent rejection, atomic publication, interruption recovery and retry were retained.','',
        'Publication and qualification scope:',
        '- Complete-mesh reuse control published and fresh-reopened with scientific identity a1e9d5906a7a723ec9906c938a52a25921e23d867aa9f22a489162cba9770b4f.',
        f"- Control content: {body['publication']['content']}; access: {body['publication']['access']}; checkpoint: {body['publication']['checkpoint']}.",
        '- Its representation root differs because gzip representation and implementation bindings differ. Its scientific identity does not differ.',
        '- Reference-only publication took 0.891 s and fresh reopen 1.841 s. This reuses authenticated components and is not claimed as an equivalent-scope speedup over the original full preparation/publication.',
        '- Profiler runs, complete scientific comparisons, quality replay (23.158 s), saved-certificate verification and thread studies are qualification costs, not normal layer timing.',
        '- No new CFD-numerics claim. Production CURRENT pointers and all sealed earlier authorities stayed unchanged.','',
        'Tests, preservation and storage:',
        '- 237 focused/regression tests passed; 36 transaction tests also passed with the optimized durable writer installed. No existing tests were weakened. CRLF-aware whitespace result is recorded separately.',
        '- Development corrections: an admission int64/int32 storage mismatch was fixed before release qualification; an empty-profiler bookkeeping error was fixed. Neither changed production authority.',
        f"- Starting free space: {storage['starting_free']} B. Pre-cleanup task allocation: {before['physical']} B. Reclaimed measured file allocation: {sum(r['physical'] for r in deleted)} B across {len(deleted)} files. Retained snapshot: {storage['retained_allocation']} B. Ending free: {storage['ending_free']} B.",
        '- Temporary peak was not continuously measured. No complete duplicate mesh tree is retained. The optimized certificate, publication control, compact reports, profiles and real callback queries remain.',
        '- Automatic review rejected recursive cleanup. A narrower cleanup removed two named duplicate JSON files and 123 individually hash-verified bulk files; directories and compact evidence were retained.',
        '- All 155 unrelated dirty entries are byte-identical in git status. V3 inventory is unchanged. No reset, clean, stash, sealed-authority rewrite or attested-file hardlink occurred.',
        f'- Optimized implementation identity: {implementation}. Exact report and authority file hashes are in HASHES.json.','',
        'Recommended next task: OptimizeV4PrismGeometryQualityAndAuthenticatedAssemblyHotPath — a bounded exact A/B task targeting the remaining 302 s layer path, with unchanged scientific and transaction gates.']
    (OUT/'REPORT.md').write_text('\n'.join(lines)+'\n',encoding='utf8')
    evidence=['REPORT.json','REPORT.md','CONTROLS.json','VERIFIED.json','PRISM_EQUIVALENCE.json','CORE_REPLAY_EQUIVALENCE.json',
        'CERTIFICATE_REOPEN.json','PUBLICATION_RELEASE_CONTROL.json','PUBLICATION_RELEASE_REOPEN.json','PRESERVATION.json','TESTS.xml','TRANSACTION_TESTS.xml','WHITESPACE.json']
    authority=h.seal(dict(classification=body['classification'],starting_head=START,implementation=implementation,
        scientific=body['publication']['scientific'],production_authority_unchanged=True,
        files={name:h.file_hash(OUT/name) for name in evidence}))
    h.durable_json(OUT/'AUTHORITY.json',authority)
    h.durable_json(OUT/'HASHES.json',dict(authority_identity=authority['identity'],implementation=implementation,
        files={name:h.file_hash(OUT/name) for name in ('REPORT.json','REPORT.md','AUTHORITY.json')}))


if __name__=='__main__':
    import sys
    {'collect':collect,'preservation':preservation,'report':report}[sys.argv[1]]()

"""Seal the measured critical-path qualification without changing mesh authority."""
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET
import capsule_user_domain_handoff as h

OUT = Path(__file__).resolve().parents[1] / 'results/remaining_prism_critical_path'
ROOT = Path(__file__).resolve().parents[3]


def control(name):
    folder = OUT / ('normal_' + name + '_prism')
    result = {key: h.read(folder / file) for key, file in
              [('normal', 'NORMAL.json'), ('generation', 'GENERATION_PHASES.json')]}
    for file in ('REOPEN.json', 'DEGREE_PIPELINE.json', 'INDEX_PIPELINE.json', 'QUALITY_SQL_PROFILE.json', 'QUALITY_CODEC_METRICS.json'):
        if (folder / file).exists():
            result[file] = h.read(folder / file)
    result['memory'] = {p.name: h.read(p) for p in folder.glob('*_MEMORY.json')}
    job = OUT / (name + '_JOB.json')
    if job.exists():
        result['job'] = h.read(job)
    return result


if __name__ == '__main__':
    pre = h.read(OUT / 'PREFLIGHT.json')
    status = subprocess.check_output(['git', 'status', '--porcelain=v1'], cwd=ROOT, text=True).splitlines()
    assert len(pre['status']) == 155 and all(line in status for line in pre['status'])
    tests = ET.parse(OUT / 'FINAL_TESTS.xml').getroot()
    suites = list(tests.iter('testsuite'))
    assert sum(int(s.get('failures', 0)) + int(s.get('errors', 0)) for s in suites) == 0
    checks = {name: h.read(OUT / (name + '.json')) for name in
              ('DETERMINISTIC_REPLAY', 'DEEP_candidate_clean', 'CORE_REGRESSION', 'CERTIFICATE_REGRESSION', 'POST_CLEANUP_REOPEN')}
    assert all(c['passed'] for c in checks.values())
    baseline = control('candidate_confirm')
    runs = [control('candidate_clean'), control('candidate_clean_replay')]
    mean = sum(r['normal']['admission_to_publication'] for r in runs) / 2
    implementation = {n: h.file_hash(Path(__file__).parent / n) for n in checks['DETERMINISTIC_REPLAY']['implementation']}
    data = dict(classification='V4RemainingPrismProductionCriticalPathOptimizationQualified',
        starting_head=pre['head'], authority_verification=pre, epoch_renewal_seconds=0,
        baseline=baseline, final_runs=runs, full_family_baseline=control('candidate_bound'),
        mean_admission_publication=mean, speedup_user_reference=107.114 / mean,
        speedup_historical=528.012 / mean, checks=checks, codec_scaling=h.read(OUT / 'QUALITY_CODEC_SCALING.json'),
        tests=sum(int(s.get('tests', 0)) for s in suites), storage=h.read(OUT / 'STORAGE.json'),
        whitespace_normalization=h.read(OUT / 'WHITESPACE_NORMALIZATION.json'),
        implementation=implementation, implementation_identity=h.identity(implementation),
        recommended_next_task='QualifyV4DirectChangedStateEncodingAndSingleQualityDeltaPersistence')
    h.durable_json(OUT / 'REPORT.json', data)
    lines = [data['classification'], '',
        'The opt-in critical path reproduces sealed L10 to L11. Production authority was not replaced. No epoch renewal was needed.', '',
        '| Phase | Fresh baseline s | Final 1 s | Final 2 s |', '|---|---:|---:|---:|']
    for key in ('parent_open', 'admission', 'generation', 'assembly', 'publication'):
        values = [c['normal']['phases'][key]['seconds'] for c in [baseline] + runs]
        lines.append('| ' + key + ' | ' + ' | '.join(f'{v:.6f}' for v in values) + ' |')
    for label, values in [
        ('Admission to publication', [c['normal']['admission_to_publication'] for c in [baseline] + runs]),
        ('Fresh reopen', [c['REOPEN.json']['metrics']['seconds'] for c in [baseline] + runs]),
        ('Open to fresh reopen', [c['normal']['metrics']['seconds'] + c['REOPEN.json']['metrics']['seconds'] for c in [baseline] + runs])]:
        lines.append('| ' + label + ' | ' + ' | '.join(f'{v:.6f}' for v in values) + ' |')
    lines += ['', f'Mean normal admission to publication: {mean:.6f} s. Speedup versus 107.114 s: {107.114/mean:.6f}x. Versus 528.012 s: {528.012/mean:.6f}x. The 60 s target remains unmet by {mean-60:.6f} s.', '',
        'Initial profile: generation 181,536,038 calls; face_key 1,452,798 calls / 14.060 cumulative seconds; exact_centroid 371,646 / 9.445 s; cycle 3,851,604 / 8.321 s; identity 1,604,869 / 8.083 s. Assembly 61,145,301 calls; 264,219 scientific-face lookups / 7.040 s; 67,572 quality queries / 4.595 s. Publication 52,922,660 calls; 2,422,939 proof identities / 6.280 s; 371,646 compressions / 4.978 s; index executemany / 4.425 s. These are profiled cumulative costs, not additive clean timings.',
        'Second profile: generation 88,185,327 calls; assembly 38,831,573 calls; publication 16,563,471 calls. Remaining costs include source-face enumeration, changed-record encoding, atlas/quality JSON, and independent durable readback.',
        'Iterations: 108.487790 s fresh baseline; 93.279623 s exact integer witness batches/source reuse; 91.722845 s threaded index overlap; 79.623435 s isolated index process and face-key cache; 76.505615 s isolated degree coordinator and batched quality lookup; final mean 72.523697 s with ordered four-process quality encoding. Every candidate compared exactly to sealed science. Threaded index took 32.826 s under contention; isolated process about 11.5 s. The faster scheduler was selected.',
        'Geometry: reuse authenticated inherited scientific face keys; newest delta controls ownership. Read-only ndarray views avoid memmap subclass overhead without copying or dropping file guards. Bounded immutable-ID cycle cache preserves ordered face perimeter and canonical hashes. Exact rational centroids convert binary64 coordinates to common-denominator integers once, then use bounded integer batches.',
        'Concurrency: one isolated degree coordinator with two degree workers overlaps quality/delta work; four existing quality threads; eight existing native proof workers in the later stage. One private index process overlaps assembly. Four quality-codec processes use bounded ordered 2,048-row batches. Atomic commit remains serial and waits for all authenticated components. No new native dependency or mathematical predicate change.',
        'Quality: 67,572 selected historical rows in 132 indexed SQL batches; 33,786 determinant completions; 152,037 new rows; 219,609 delta rows; 1,419,012 unchanged rows reused. Full 219,609-row canonical codec benchmark: serial 6.653905 s, two threads 5.773956 s, two processes 6.730851 s, four processes 3.741495 s, identical digest. Both independent transaction readback comparisons remain.',
        'Degree/volume: complete original obligations remain. Concurrent degree including exact witnesses takes about 10.56 s, largely hidden behind independent work. The approximately 0.95 s reported late generation phase is remaining wait plus volume, not total degree computation. No new incremental degree or volume theorem is claimed.',
        'Representation: no scientific or storage-format change. Canonical JSON/zlib bytes, SQLite schema/order, proof index and quality database remain identical. Large quality JSON and SQLite are both retained because existing component identities and closure bind them. Removing one requires a versioned logical-component storage/recovery design. No old payloads are rewritten.',
        'Proofs: 30,913,093 prior plus 2,422,939 new equals 33,336,032. New methods: 371,043 supporting-plane, 560,561 owned-edge, 1,491,223 owned-vertex, 112 exact-hull legal contact. Prior pair scans 0; changed-face probes 270,288; postings 1,181,773; changed bindings 974,421; unchanged 29,938,672. Exact coverage/classification and failure behavior remain unchanged.',
        'Proof index: 225,722,368 B; ee85cf43f84edc67e42ecc841b32e4e09a936b315c8c2b98c5fb5de13de974ea. Quality database: 441,167,872 B; 04308917005ab305e3d3750076c8c03944a7090fd3885e1601ffb76f49cb973f. Parallel IPC increases process write traffic; it does not reduce database bytes. No claim of complete subsystem write attribution.',
        'Exact L11 content root: 6b11b14c7b519467ad49a126261f0ddd608692d82ca387160fcc1b7ab10c5870. Admission, forecast, coordinates, connectivity, owners, quality, determinant contexts, degree/volume, proofs/witnesses, bindings, incidence, source continuity and component identities match. Independent final controls have identical numeric arrays and canonical files. Private access roots differ by paths/file tokens.',
        'Deep audit: complete 371,646-cell / 962,901-face reconstruction; 895,329 internal and 67,572 boundary faces; exact ownership/topology/version closure; degree/volume equality; complete changed quality comparison. 12.966557 s wall / 12.9375 s CPU, separate from normal timing. Inherited immutable quality remains certified by sealed authority; no redundant full prism proof audit.',
        'Core regression: exact callback binary64 values, CORE, NETGEN, OUTER, ownership and all quality audit values. Saved complete new-core certificate: 19,941,573 rows and canonical hash 9ce3df428c1784391429ebf18a3385429fde0b679c0fc85b32ec9499ac715c1a; verification 12.148010 s. No replacement core or prism authority.',
        'Durability: DELETE journaling, synchronous=FULL, adaptive caches, uniqueness/integrity, both readback checks, fsync, stale-base and concurrent-writer rejection, token guards, corruption rejection, atomic pointer, interruption/pointer failure recovery and retry retained and regression-tested. Prepared index source/root/token mismatch fails closed.',
        'Tests: 377 passed, 0 failed; two expected empty-geometry NumPy warnings. After trailing-line normalization, 9 focused tests passed. An initial follow-up invocation omitted src from PYTHONPATH and failed collection; corrected invocation passed. Normalization changed only an EOF empty line and is AST-identical to the timed implementation.',
        'Normal fresh reopen visits 0 historical proof rows and 0 historical quality rows; no full-prefix scientific hash or deep copy. Post-cleanup retained L11 reopen passes. Current/stale epoch tests retain IncrementalEpochChanged rejection. No hardlinks created.',
        'Limitations: complete degree traversal still exists, though batched and overlapped. Geometry-delta canonical encoding about 9.15 s, assembly about 19 s and publication about 18 s remain. Per-stage RSS sampling may miss sub-100ms peaks; whole-family Job CPU/I/O includes exited workers and launch/import/IPC. Do not equate parent-only I/O with total traffic. Temporary allocation peak was not sampled continuously.',
        'Safety: all 155 original dirty entries preserved; V3 untouched; sealed scientific authorities unchanged. Only task-created duplicate controls are removed after hashes and evidence are saved. Full profiler files remain local; compact summaries are committed.',
        'Recommended next task: QualifyV4DirectChangedStateEncodingAndSingleQualityDeltaPersistence.', '']
    (OUT / 'REPORT.md').write_text('\n'.join(lines), encoding='utf-8')
    h.durable_json(OUT / 'GIT_PRESERVATION.json', dict(starting_head=pre['head'], unrelated_entries=155,
        original_status_preserved=True, v3_modified=False, hardlinks_created=0))
    evidence = {p.relative_to(OUT).as_posix(): h.file_hash(p) for p in sorted(OUT.rglob('*'))
                if p.is_file() and p.suffix in ('.json', '.md', '.xml', '.txt', '.log') and p.name != 'AUTHORITY.json'}
    body = dict(classification=data['classification'], scientific='6b11b14c7b519467ad49a126261f0ddd608692d82ca387160fcc1b7ab10c5870',
        prior_qualification=pre['authority'], implementation_identity=data['implementation_identity'],
        implementation=implementation, evidence=evidence)
    h.durable_json(OUT / 'AUTHORITY.json', dict(identity=h.identity(body), **body))
    print('implementation', data['implementation_identity'])
    for name in ('REPORT.json', 'REPORT.md', 'AUTHORITY.json'):
        print(name, h.file_hash(OUT / name))
    print('authority', h.read(OUT / 'AUTHORITY.json')['identity'])

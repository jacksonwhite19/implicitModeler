"""Seal bounded changed-encoding results; do not claim a storage migration."""
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET
import capsule_user_domain_handoff as h

OUT=Path(__file__).resolve().parents[1]/'results/direct_changed_persistence'
ROOT=Path(__file__).resolve().parents[3]

def load(name):
    p=OUT/('normal_'+name+'_prism')
    return {f.stem:h.read(f) for f in p.glob('*.json') if f.name in ('NORMAL.json','REOPEN.json','GENERATION_PHASES.json','DIRECT_ENCODING.json','INDEX_PIPELINE.json','QUALITY_SQL_PROFILE.json','PREPARE_METRICS.json') or f.name.endswith('_MEMORY.json')}

if __name__=='__main__':
    pre=h.read(OUT/'PREFLIGHT.json');status=subprocess.check_output(['git','status','--porcelain=v1'],text=True).splitlines()
    assert all(s in status for s in pre['status']) and len(pre['status'])==155
    checks={n:h.read(OUT/(n+'.json')) for n in ('DETERMINISTIC_REPLAY','DEEP_candidate_clean','FUTURE_PARENT','CORE_REGRESSION','CERTIFICATE_REGRESSION','POST_CLEANUP_REOPEN')}
    assert all(x['passed'] for x in checks.values())
    tests=list(ET.parse(OUT/'FINAL_TESTS.xml').getroot().iter('testsuite'))
    assert not sum(int(x.get('failures',0))+int(x.get('errors',0)) for x in tests)
    base=load('candidate_confirm');runs=[load('candidate_clean'),load('candidate_clean_replay')]
    mean=sum(x['NORMAL']['admission_to_publication'] for x in runs)/2
    improved=mean<base['NORMAL']['admission_to_publication']
    classification='V4DirectChangedStateEncodingQualifiedSingleQualityPersistenceDeferred' if improved else 'V4ChangedStateEncodingExactControlNoMeasuredPerformanceQualification'
    implementation=checks['DETERMINISTIC_REPLAY']['implementation']
    assert all(h.file_hash(Path(__file__).parent/k)==v for k,v in implementation.items())
    report=dict(classification=classification,starting_head=pre['head'],preflight=pre,baseline=base,runs=runs,mean=mean,improved=improved,
        epoch_renewal_seconds=0,checks=checks,tests=sum(int(x.get('tests',0)) for x in tests),single_quality_removed=False,
        old_quality_bytes=778578817,new_quality_bytes=778578817,quality_json_bytes=337410945,quality_sqlite_bytes=441167872,
        storage=h.read(OUT/'STORAGE.json'),implementation=implementation,implementation_identity=h.identity(implementation),
        jobs={n:h.read(OUT/(n+'_JOB.json')) for n in ('candidate_confirm','candidate_clean','candidate_clean_replay')},
        stop_reason='Further material gain requires versioned component/acceptance/recovery migration; bounded final pass stops.',
        next_task='Accept the qualified prism performance as good enough for now and move on to the next product requirement; no further prism optimization is scheduled.')
    h.durable_json(OUT/'REPORT.json',report)
    lines=[classification,'','This is a bounded, opt-in exact encoding control. Single quality persistence is deferred; the requested combined qualification is not claimed. Production scientific authority remains unchanged.','',
        '| Phase | Fresh baseline s | Final 1 s | Final 2 s |','|---|---:|---:|---:|']
    for k in ('parent_open','admission','generation','assembly','publication'):
        lines.append('| '+k+' | '+' | '.join(f"{r['NORMAL']['phases'][k]['seconds']:.6f}" for r in [base]+runs)+' |')
    for label,values in [('Admission to publication',[r['NORMAL']['admission_to_publication'] for r in [base]+runs]),('Fresh reopen',[r['REOPEN']['metrics']['seconds'] for r in [base]+runs]),('Open to fresh reopen',[r['NORMAL']['metrics']['seconds']+r['REOPEN']['metrics']['seconds'] for r in [base]+runs])]:
        lines.append('| '+label+' | '+' | '.join(f'{v:.6f}' for v in values)+' |')
    lines += ['',f'Mean {mean:.6f} s. Speedup versus 72.524 s: {72.524/mean:.6f}x; versus 528.012 s: {528.012/mean:.6f}x. Approximately 60 s reached: {mean<=60}.',
        'Fresh profile covers normal current code, excluding qualification. Profile cumulative functions overlap and must not be added. Assembly/publication profile text and per-component encoding/SQLite/index records are retained. The initial profile was invalidated by changes to then-unused experimental sources captured in its source manifest; it was repeated with frozen sources. The first baseline-named run also used the nonadaptive harness branch and is not the primary baseline.',
        'Quality JSON role: exact scientific component byte identity, prepared transaction input, acceptance canonical identity, source references for recovery and replay, and direct qualification comparison. It is not only reporting data. SQLite role: direct key lookup, inherited input-only updates, recursive deep iteration, independently verified durable transaction payload. Both carry the same changed scientific records but serve distinct current contracts.',
        'Single quality persistence was not removed. JSON 337,410,945 B plus SQLite 441,167,872 B equals 778,578,817 B, unchanged. A single canonical format requires coordinated component versioning, logical identity, prepared retry and reference closure changes. This is a larger migration than the permitted final pass. Old parents and new children retain unchanged formats.',
        'Direct geometry changes cache each checked source vertex and each inherited face within one append. The qualified compact C JSON encoder and native indent pass remain unchanged. Final bytes are hashed in 1 MiB write blocks, fsynced and atomically renamed. A deny-write handle and checked file token bind each receipt until session exit. Later hash requests reuse only that receipt. Mutable roots, plans, pointers and journals remain on their existing path. Large Python geometry dictionaries still exist; no packed final-form geometry redesign is claimed.',
        'No new concurrency or native kernel. Existing degree/quality/proof/index scheduling is preserved. Independent transaction readback remains. No SQLite schema, journal, synchronous, fsync, uniqueness, integrity or recovery behavior changed.',
        'Exact scientific A/B and independent deterministic replay pass: admission, forecast, binary64 coordinates, connectivity, ownership, topology, all changed quality, determinants, degree, exact volume, proof classifications/witnesses, bindings, incidence, source continuity and components. Content root 6b11b14c7b519467ad49a126261f0ddd608692d82ca387160fcc1b7ab10c5870. Byte-identical proof index and quality database. Access roots differ only for private storage references/tokens.',
        'Deep audit reconstructs all 371,646 cells and 962,901 faces, ownership and topology closure, exact degree/volume and 219,609 changed quality rows. Existing immutable ancestor quality is reused. Deep audit and comparisons are separate from normal production timing.',
        'Fresh reopen and future-parent probes pass without conversion: indexed changed/inherited quality and geometry lookup retain existing recursive semantics. Synthetic two-layer parent/delta test passes. No L12 generated. Normal fresh reopen visits zero historical proof/quality rows; targeted future-parent probes are reported separately.',
        'Bounded C1 regression checks exact sizing binary64, core coordinates, tetrahedra, interface/ownership and quality; complete saved certificate reopens and matches 19,941,573 rows. No full new core qualification or replacement core authority.',
        'Durability tests retain stale/wrong parent, concurrent writer, corrupt component, partial write, interruption, pointer failure, recovery and retry rejection/behavior. New tests check exact bytes, hash receipt token rejection, failed writes preserving the original, metadata fallback and fresh-process deterministic encoding.',
        'Stop rule applied. The experiment is retained as opt-in evidence and is not promoted over the prior qualified path. There is no further optimization program. Recommend accepting the qualified prism performance as good enough for now and moving on.',
        'Safety: 155 original dirty entries preserved; V3 untouched; no hardlinks; no sealed scientific authority rewritten. Cleanup removes only this task\'s duplicate controls after preserving hashes. Raw profiler files remain local; summaries are committed. Temporary peak is not continuously sampled.','']
    (OUT/'REPORT.md').write_text('\n'.join(lines),encoding='utf-8')
    evidence={p.relative_to(OUT).as_posix():h.file_hash(p) for p in sorted(OUT.rglob('*')) if p.is_file() and p.suffix in ('.json','.md','.txt','.xml') and p.name!='AUTHORITY.json'}
    body=dict(classification=classification,prior=pre['authority'],implementation=implementation,implementation_identity=h.identity(implementation),evidence=evidence)
    h.durable_json(OUT/'AUTHORITY.json',dict(identity=h.identity(body),**body))
    print(classification,mean)
    print('implementation',h.identity(implementation),'authority',h.identity(body))
    for name in ('REPORT.json','REPORT.md','AUTHORITY.json'):print(name,h.file_hash(OUT/name))

"""Seal measured storage progress without claiming production migration."""
import json,subprocess
from pathlib import Path
from sdf_cfd_mesher_v4.common.authority import file_hash
from qualify_native_storage import ROOT,OUT,OLD,save

def run():
    expected='414b617c200e94672e5fc45cc02a135bbb54fab9'
    prior={'report.md':'a4507af0594e4788ff3eaf6bc8f3fde3cf82ccaf846d84c99d94f8b20c5745b5',
           'AUTHORITY.json':'478189eb8fba49f8fe638a9c863f6c23817cbfa4d1d9319010e0052c951c46c4'}
    assert all(file_hash(OLD/name)==digest for name,digest in prior.items())
    legacy=json.loads((OUT/'LEGACY_RESTART_MEMORY.json').read_text())
    native=json.loads((OUT/'NATIVE_RESTART_MEMORY.json').read_text())
    probe=json.loads((OUT/'STORAGE_PROBE.json').read_text())
    assert legacy['scientific']==native['scientific']==probe['scientific']
    saved=legacy['peak_rss']-native['peak_rss']
    status=dict(classification='V4NativeMixedProductionMigrationInsufficient',
        production_authority_switched=False,automatic_remesh_default=False,
        native_checkpoint_storage='Verified general-SDF post-layer-6 state and focused transaction tests',
        legacy_prism_import='Incomplete',full_native_atlas='Incomplete',full_native_forecast='Incomplete',
        sf2_native_proof_binding='Incomplete',full_checkpoint_cohort='Incomplete',
        full_sf2='Not started: mandatory native prequalification incomplete',
        native_general_remesh_rerun='Not performed; sealed post-continuation state imported',
        full_migration_memory_reduction='Not established; restart-only comparison measured',
        checkpoint_restart_peak_reduction_bytes=saved,checkpoint_restart_peak_reduction_fraction=saved/legacy['peak_rss'],
        tests='112 passed in 13.12 s',prior_authority_verified=prior)
    save('DECISION.json',status)
    sections=[
    ('A. Authority verification',f'Starting HEAD: {expected}. Prior report and authority hashes match the user-supplied values. Sealed prior evidence was not changed. This task has not completed production migration.'),
    ('B. Remaining migration blockers','Full prism-only checkpoint conversion, native Task-4C atlas execution, native production future forecasting, and compact SF2 scientific proof binding remain incomplete. The fresh SF2 run is therefore not eligible. These are unfinished implementation and qualification work, not a scientific infeasibility finding.'),
    ('C. Native checkpoint authority','Added V4NativeMixedCheckpoint/1. The mixed journal publishes content-addressed components and SQLite evidence tables through CURRENT. Geometry, source ancestry, schedules, atlas/forecast records, and transaction history are retained as native components. Existing scientific hashes remain unchanged. Only CURRENT publishes a state. This storage qualification does not establish the complete requested checkpoint cohort.'),
    ('D. Legacy checkpoint import','A verified one-time historical mixed JSON import checks pointer, payload, and scientific hashes, then releases the imported representation. It rejects a nonempty destination. It does not invent absent prism-only state. General-SDF post-layer-6 import passes; full legacy prism import remains incomplete.'),
    ('E. Native quality receipts',f"Imported {probe['counts']['quality']:,} source-keyed quality records without changing values. Evidence tables are read-only; private transactions use copy-on-write overlays. Nested edits and retirement are tested. Full SF2 native quality binding has not been qualified in this task."),
    ('F. Native certificate manifest',f"Imported {probe['counts']['certificate']:,} existing scientific pair records without exact recomputation. The scientific state hash is {probe['scientific']}. This proves storage equality, not a new complete geometric audit. The 56,462,864-relation SF2 compact manifest remains unbound natively."),
    ('G. Native risk atlas','No full native Task-4C provider was completed. Existing local dependency records persist exactly. They are not a substitute for production atlas evaluation or SF2 region equality.'),
    ('H. Native future forecast','No full native production forecast provider was completed. Existing forecast records persist exactly. Forecast geometry, active patch selection, and SF2 layer decisions remain unqualified on the native production path.'),
    ('I. Adjusted local schedules','The sealed general-SDF schedule component is included in exact scientific identity. Existing adjusted-schedule tests pass. No fresh adjusted-schedule production continuation was executed here.'),
    ('J. Memory and duplicate-state audit',f"Separate fresh-process restart probes used the same 7,604-cell, 538,033-relation checkpoint. Legacy peak RSS: {legacy['peak_rss']:,} bytes; resident after restart: {legacy['rss']:,}. Native peak RSS: {native['peak_rss']:,}; resident: {native['rss']:,}. Peak reduction: {saved:,} bytes ({100*saved/legacy['peak_rss']:.2f}%). Restart times: {legacy['seconds']:.6f} s and {native['seconds']:.6f} s. The earlier 5,189,218,304-byte full migration peak is a different workload and is not directly comparable. Per-subsystem full-remesh allocation attribution remains incomplete."),
    ('K. Duplicate-state cleanup',f"Streaming scientific hashing removes whole-envelope JSON allocation. Disk-backed evidence avoids full resident receipt/proof dictionaries on restart. Private transaction copies share immutable tables and store only changed records. Import peak RSS was {probe['stages'][1]['peak_rss']:,} bytes. After import and release, RSS was {probe['stages'][1]['rss']:,}; after private copy it was {probe['stages'][2]['rss']:,}. Conversion still parses a complete historical JSON envelope transiently. Geometry components remain JSON; scalable full-SF2 geometry storage has not been established."),
    ('L. Native general-SDF rollback','The prior qualified 45-prism plus 2-tetrahedron rollback was not rerun. Its post-layer-6 checkpoint was imported and restarted exactly. This is storage regression evidence, not fresh native-provider rollback qualification.'),
    ('M. Native late-stage failure recovery','Focused journal tests pass at numerical validation, certificate generation, state rebuild, and before commit using native storage. Incomplete object publication leaves CURRENT unchanged. A fresh live numerical/certificate failure-injection cohort was not run.'),
    ('N. SF2 bounded prequalification','Not complete. The full layer 1/5/11/12/20 native atlas, forecast, and certificate prequalification was not executed. Prior SF2 evidence remains historical authority only.'),
    ('O. Fresh full SF2 mixed build','Not started. Required native prequalification is incomplete. No legacy execution was relabeled as a native build. The 30-minute scientific budget was not consumed.'),
    ('P. SF2 scientific equality','Not measured for a fresh native build. The preserved qualified SF2 result was not regenerated or altered.'),
    ('Q. SF2 certificate','No fresh 56,462,864-relation native certificate was produced. Prior evidence was preserved; its result is not claimed as this task\'s new certification.'),
    ('R. SF2 runtime','Not measured. Overhead against 1,248.01 s is unavailable.'),
    ('S. SF2 memory','Not measured. The general checkpoint restart reduction cannot be extrapolated to full SF2.'),
    ('T. Wall/first-layer identity','The imported complete scientific state matches exactly, including wall and first-layer coordinates and topology. The curved wall reduction-order regression passes. No wall, first-layer spacing, geometry kernel, quality gate, or certificate mathematics was changed.'),
    ('U. Legacy runner retirement','Not retired. The existing production default remains unchanged. Native storage is used by the mixed journal, but this does not promote the mixed runner to production authority.'),
    ('V. Automatic remesh default state','Unchanged: disabled by default. No production switch was made.'),
    ('W. Determinism','Exact general checkpoint scientific identity survives import, native restart, and private transaction copy. Table ordering and storage layout do not redefine scientific identity. Full production replay remains untested.'),
    ('X. Tests','112 passed in 13.12 s: test_native_checkpoint.py, test_mixed_front.py, test_production_migration.py, test_remesh_transaction.py, test_termination_policy.py, test_rolling.py. Initial failures exposed unclosed SQLite read handles and the need for private mutable receipt overlays; both were fixed. No full geometry campaign was run.'),
    ('Y. Classification','V4NativeMixedProductionMigrationInsufficient. Native persistence improves materially, but key native providers and full SF2 qualification remain incomplete.'),
    ('Z. Production authority decision','No. Do not replace the legacy production authority or enable automatic remesh by default based on this result.'),
    ('AA. Recommended next task','Finish the remaining work in this task: scalable prism checkpoint import, compact scientific SF2 manifest binding, full native atlas and forecast providers, then the required bounded and capped full-run qualifications. Do not start core integration.')]
    report='## Decision question\n\nCan V4 complete native mixed-front persistence, atlas, forecasting, receipt, and certificate infrastructure well enough for the mixed-front runner to become the single production authority and reproduce the full qualified SF2 shell?\n\n'
    report+='\n\n'.join('### '+title+'\n\n'+body for title,body in sections)+'\n'
    (OUT/'report.md').write_text(report,encoding='utf8',newline='\n')
    sources=[ROOT/'src/sdf_cfd_mesher_v4/advancing_front'/name for name in ('native_checkpoint.py','mixed_front.py','mixed_front_journal.py')]
    sources += [ROOT/'tests/test_native_checkpoint.py',ROOT/'tools/qualify_native_storage.py',ROOT/'tools/seal_native_storage.py']
    files=sorted(sources+[p for p in OUT.rglob('*') if p.is_file() and p.name not in ('AUTHORITY.json','HASHES.json')])
    save('AUTHORITY.json',dict(task='CompleteV4NativeMixedFrontProductionMigration',classification=status['classification'],
        input_head=expected,prior_authority=prior,production_authority_switched=False,automatic_remesh_default=False,
        scientific=probe['scientific'],artifacts={str(p.relative_to(ROOT)).replace('\\','/'):file_hash(p) for p in files}))
    save('HASHES.json',{name:file_hash(OUT/name) for name in ('report.md','AUTHORITY.json')})

if __name__=='__main__':run()

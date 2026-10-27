"""Seal measured results without promoting unfinished production migration."""
import json,subprocess,tarfile
from pathlib import Path
from audit_production_migration import OUT,ROOT
from sdf_cfd_mesher_v4.common.authority import file_hash

BASE='59692a5cf4899004cbec26a38bc6dbb459872c88'
REPO=ROOT.parents[1]
def read(p):
    if p.exists():return json.loads(p.read_text())
    from archive_production_migration import wsl
    name=p.relative_to(OUT).as_posix();result=None
    process=subprocess.Popen(['wsl','--','zstd','-q','-d','--long=31','-c',wsl((OUT/'EVIDENCE.tar.zst').resolve())],stdout=subprocess.PIPE)
    with tarfile.open(fileobj=process.stdout,mode='r|') as tar:
        for member in tar:
            if member.name==name:result=json.load(tar.extractfile(member))
    if process.wait()!=0 or result is None:raise ValueError('MissingArchivedEvidence')
    return result
def write(p,v):p.write_text(json.dumps(v,sort_keys=True,indent=2)+'\n',encoding='utf8',newline='\n')
def git(*args):return subprocess.check_output(['git','-c','core.longpaths=true',*args],cwd=REPO,text=True).strip()

def run():
    assert git('rev-parse','HEAD')==BASE
    # Only review metadata remains raw. Archived checkpoint bytes stay exact.
    for path in OUT.rglob('*.json'):
        if path.name.startswith('STATE_'):continue
        value=path.read_text()
        path.write_text(value.rstrip()+'\n',encoding='utf8',newline='\n')
    prior=ROOT/'results/purpose_built_mixed_front_runner'
    expected={'report.md':'a591ec4fa8ab3e9f0030e6a0299a40d6265abc22783a757d5b2f1d1e1f91b58a',
              'AUTHORITY.json':'bccaea012447187e92d310b7b89abce6ed760967088ec9aa27163e746cbb690d'}
    assert all(file_hash(prior/name)==h for name,h in expected.items())
    s=read(OUT/'SUMMARY.json');g=s['general'];sf=s['sf2'];runtime=read(OUT/'general_native/RUNTIME.json');late=read(OUT/'LATE_FAILURE_RUNTIME.json')
    compact=json.loads(json.dumps(s));validation=compact['general']['validation']
    compact['general']['validation']={k:validation[k] for k in ('checks','quality_passed','certificate','validation_hash')}
    write(OUT/'SUMMARY_COMPACT.json',compact)
    sr=read(OUT/'SF2_MIGRATION_RUNTIME.json');rr=read(OUT/'GENERAL_REPLAY_RUNTIME.json')
    tests=(OUT/'tests.txt').read_text();assert '219 passed' in tests
    # Review text uses LF. Scientific checkpoint storage is archived byte-exact.
    (OUT/'tests.txt').write_text(tests.rstrip()+'\n',encoding='utf8',newline='\n')
    status=git('status','--porcelain=v1').splitlines()
    unrelated=[line for line in status if 'research/sdf_cfd_mesher_v4/' not in line]
    write(OUT/'GIT_AUDIT.json',dict(base=BASE,V3_task_modifications=0,unrelated_entries=len(unrelated),
        unrelated_status_count_matches_initial=len(unrelated)==155,scope='Status inventory; no outside-file content equality claim'))
    write(OUT/'FULL_SF2_STATUS.json',dict(started=False,scientific_seconds=None,
        reason='Native checkpoint, atlas and full forecast migration prerequisites did not pass. No legacy build was relabeled as mixed.',
        required_future_cap_seconds=1800,mandatory_review_seconds=1620))
    report=f'''## Decision question

Can the purpose-built mixed-front runner fully replace the legacy prism-only Task-4 runner as the single V4 production authority while preserving SF2 exactly and supporting native automatic local rollback/remeshing?

No. This run qualifies native historical rollback and late transaction recovery, but does not complete production migration.

### A. Authority verification

Branch: feature/sdf-cfd-mesher-v4-advancing-front. Starting HEAD: {BASE}. Both supplied prior report and authority hashes match. Prior sealed results remain unchanged. V3 task modifications: zero.

### B. Migration objective

Keep one production authority. The legacy Task-4 default remains authoritative. The bounded mixed runner is not promoted by these results. No new topology, trigger policy, capacity witness, core meshing or scientific threshold was added.

### C. Legacy checkpoint migration

SF2 prefixes 1, 5, 11, 12 and 20 were verified through the complete published hash chain. Coordinates, implicit connectivity, wall, schedule and source ancestry remain exact. The migration inventory preserves their scientific bindings. It is explicitly not a native mixed checkpoint. Native compact entity storage and the requested curved/spherical checkpoint cohort remain unqualified in this task.

### D. Quality receipt migration

All 20 SF2 layer receipts verified through the existing implementation. Numeric gates, exact signs, SDF records, conditioning hashes and deterministic entity populations match. Twenty receipts are retained in the inventory; zero missing receipts and zero recomputed receipts. Tests reject changed source identity and target records. The initial implementation misread a compressed JSON pointer as its payload; it now uses the existing checked storage reader. The failure reproducer is preserved. Individual native mixed-manifest binding remains incomplete.

### E. Certificate manifest migration

All 56,462,864 required SF2 relations were enumerated and verified against stored compact proofs, outcomes and witnesses. Zero missing, duplicate, forbidden or unresolved relations. Zero new exact solves. Source-edge/layer mappings and payload hashes are preserved in the migration inventory. This is verified proof reuse, not publication into the native mixed manifest. Mixed-manifest binding of the compact SF2 proof blocks remains incomplete.

### F. Full risk-atlas migration

Not complete. The native local dependency rebuild remains in use. It does not reproduce the full Task-4C vertex trajectory, risk partition, retained support and active/passive state machine. No full-atlas qualification is claimed.

### G. Full forecast migration

Added an explicit active-column projection interface. It shares deterministic source trajectories, suppresses terminated columns and keeps outer faces as screening dependencies. Tests exercise actual targets and variable local state. Full SF2 production forecasting, detailed warning evaluation and cache migration are not connected to this interface. This is not a full forecast adapter.

### H. Adjusted local schedule support

Schedules now bind to scientific state and survive checkpoint/restart. General-SDF imports the sealed layer-4 coordinates and local targets and the sealed residual layer-5 baseline coordinates/targets. Future entries beyond the committed prefix are provisional constant-debt nominal increments; they are not qualified recovery evidence. The actual continuation uses the existing qualified local projection mechanism. Re-running the historical layer-5 search gives coordinate hash 76c1c98112cf8eec3490ed6b325fa4fa03d3d5bc3194680bf2565c9901676036, unlike sealed 45fdeb2f91134e5b0916a8e6c12c2fac39c6242cb242e1247b9aee49c668ba4b. Migration copies the exact checkpoint instead of substituting this replay. The replay discrepancy is preserved, not claimed fixed.

### I. True committed-cell rollback

The native journal first publishes the historical mixed state as the repair input. Current cap intersections seed generic cavity discovery. The transaction removes 47 committed cells: 45 layer-4 prisms and two layer-5 tetrahedra. It reconstructs the qualified cavity exactly, using 23 pyramids and 13 tetrahedra as replacements. Thirteen source vertices control this rollback. This repairs already committed historical overlap; it does not prove prevention of that old commit. The imported historical state has no complete native legacy receipt/proof migration, so stale legacy-manifest retirement is not qualified by this case alone.

### J. Late-stage failure recovery

Journal order: Prepared, RollbackApplied, RemeshConstructed, NumericallyValidated, CertificateGenerated, StateRebuilt, Committed. Only CURRENT publishes geometry. Live interruptions after numerical validation and after certificate generation both restore the exact prior committed state. New provider instances replay each transaction. Each commits one event, removes the same 47 cells and produces identical scientific state. Idempotent dispatch succeeds. No partially bound replacement geometry or receipts survive. Historical combined-validation callbacks remain compatible but cannot claim the split boundary.

### K. General-SDF native production run

MixedFrontRunner performs trigger, discovery, actual committed-cell rollback, validation, state rebuild and commit. It then commits the existing local layer-6 continuation using the same runner. Final cells: {g['cells']}; types: {g['cell_types']}. This uses the native orchestration path, not the default SF2 production path. Geometry is constructed from pre-remesh inputs. Sealed accepted geometry is used only for exact comparison and bound proof reuse.

### L. General-SDF restart/replay

Fresh source construction, migration, discovery, remesh and continuation reproduce the final scientific identity exactly: {g['scientific']}. Final certificates are reused only when all coordinates, ordered faces, active pair population and receipt dependencies match. Wall and first-layer identities remain exact. The replay also persists complete validation summaries.

### M. Full SF2 mixed-runner run

Not started. The required native checkpoint, atlas and full forecast prerequisites remain incomplete. Running the unchanged legacy entry point would not satisfy this test. No fresh full-shell completion or 30-minute performance result is claimed. The requested 27-minute review and 30-minute stop remain requirements for the future qualified native run.

### N. SF2 scientific equality

Preserved-state equality and proof/receipt verification pass. Fresh mixed-runner equality is untested. The preserved authority remains 20 layers and 2,126,040 prisms. The 63-snapshot negative policy cohort has zero false remesh triggers. No SF2 geometry, corrections or layer counts were changed.

### O. SF2 runtime comparison

Latest full preserved-proof/receipt audit: {sr['seconds']:.6f} s. This is not scientific build time and is not compared as a speedup against 1,248.01 s. Fresh build overhead is unavailable. Native general-SDF initialization plus rollback, fresh continuation certification and publication: {runtime['seconds']:.6f} s. Late failure/recovery cohort: {late['seconds']:.6f} s. General replay: {rr['seconds']:.6f} s. These totals do not isolate production event stages. Maximum recorded peak RSS: {max(runtime['peak_rss'],late['peak_rss'],rr['peak_rss'],sr['peak_rss'])} bytes.

### P. Wall/first-layer identity

General-SDF wall and first layer remain exact through import, rollback, remesh, restart and continuation. SF2 preserved coordinates and first layer verify exactly. h1 authority remains 0.000166527069 m. No wall, topology or first-layer edit was made.

### Q. Final mixed validation

Final general-SDF certificate: {g['proofs']} relations, zero forbidden and zero unresolved. All {g['quality_receipts']} current quality receipts and every certificate receipt bind to current scientific inputs. Zero stale bindings. All applicable hard gates pass; volume accounting, ownership, outer manifoldness and Gmsh ingestion pass. Outer faces: {g['outer_faces']}. Active columns: {g['active_columns']}. Last-prism-layer counts: {g['column_layer_counts']}. Full production final validation across native SF2 compact blocks remains incomplete.

### R. Legacy-runner retirement

Not retired. The legacy default is unchanged. This avoids declaring two production authorities.

### S. Automatic-remesh default state

Disabled on the default production path. Bounded native invocations use the existing qualified policy. No new thresholds were introduced.

### T. Determinism

SF2 migration inventory replay matches exactly. Native general-SDF final state replay matches exactly. Both late-failure replays match each other. Timing and RSS are excluded from scientific identities. The historical regenerated layer-5 coordinate mismatch remains an explicit limitation; exact checkpoint migration avoids regenerating those coordinates.

### U. Tests

219 tests passed. The suite includes schedule identity, active-only projection, target/source rejection, split validation stages, late recovery, current mixed-front tests, retained rollback/policy/topology tests, capacity fixtures and branch-aware tests. Tests do not substitute for the missing fresh full SF2 qualification. See tests.txt and the live validation/recovery records.

### V. Classification

V4MixedFrontProductionMigrationInsufficient. Native historical rollback and late recovery improve the previous result materially. Key migration gaps still prevent one reliable production authority.

### W. Production authority decision

No. Do not replace the legacy Task-4 production authority yet.

### X. Automatic remesh decision

No. Do not enable automatic bounded rollback/remeshing by default yet.

### Y. Recommended next task

Complete native array-backed checkpoint storage, Task-4C atlas, full forecast and compact proof-manifest binding, then run the capped fresh SF2 qualification. Do not start core integration.
'''
    (OUT/'report.md').write_text(report,encoding='utf8',newline='\n')
    planned=[OUT/name for name in ('AUTHORITY.json','HASHES.json','COMMITTED_PATHS.json')]
    names=set(git('diff','--name-only','HEAD','--','research/sdf_cfd_mesher_v4').splitlines())
    names.update(git('ls-files','--others','--exclude-standard','--','research/sdf_cfd_mesher_v4').splitlines())
    names.update(p.relative_to(REPO).as_posix() for p in planned);names.discard('')
    write(OUT/'COMMITTED_PATHS.json',sorted(names))
    sources={name:file_hash(REPO/name) for name in sorted(names) if name.endswith('.py')}
    excluded={'AUTHORITY.json','HASHES.json','report.md'}
    evidence={p.relative_to(ROOT).as_posix():file_hash(p) for p in sorted(OUT.rglob('*')) if p.is_file() and p.name not in excluded}
    authority=dict(task='MigrateV4MixedFrontRunnerToProductionAuthority',base=BASE,classification=s['classification'],completed=False,
        production_authority_switched=False,automatic_remesh_default=False,prior_authority=expected,
        report_sha256=file_hash(OUT/'report.md'),source_sha256=sources,evidence_sha256=evidence)
    write(OUT/'AUTHORITY.json',authority)
    write(OUT/'HASHES.json',{name:file_hash(OUT/name) for name in ('report.md','AUTHORITY.json')})
    print(json.dumps(read(OUT/'HASHES.json')),flush=True)

if __name__=='__main__':run()

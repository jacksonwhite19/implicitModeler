"""Seal measured state-access qualification; never alter legacy objects."""
from pathlib import Path
import json
import shutil
import subprocess
import time
import qualify_immutable_state_access as q
from path_preserving_deduplication import metadata,physical_key,file_hash


def totals(rows):
    unique={physical_key(r):r for r in rows}
    return dict(paths=len(rows),logical=sum(r['bytes'] for r in rows),
        physical=sum(r['cluster_bytes'] for r in unique.values()),
        shared_logical=sum(r['bytes'] for r in rows if r['links']>1))


def main():
    start=time.perf_counter();out=q.OUT;read=q.access.read
    baseline=read(out/'BASELINE.json');enrollment=read(out/'ENROLLMENT.json')
    normal=read(out/'NORMAL.json');deep=read(out/'DEEP.json');work=read(out/'WORKLOADS.json')
    replay=read(out/'NORMAL_REPLAY.json')
    assert all(normal[k]==replay[k] for k in ('root','legacy','binding','increment','empty_bound_cache_identity'))
    faults=read(out/'INITIALIZATION_FAULTS.json');tests=read(out/'TESTS.json')
    assert normal['legacy']==deep['legacy_scientific']==q.audit.EXPECTED
    assert normal['exact_binding'] and deep['all_legacy_components_exact'] and deep['front_faces_exact'] and deep['packed_geometry_exact']
    assert normal['no_forecast'] and normal['no_generation'] and tests['exit_code']==0
    assert deep['proof_count']==15842336 and deep['quality_count']==726399
    assert all(normal['stats'][k]==0 for k in ('sqlite_rows_visited','proof_receipts_deserialized','proof_payloads_hashed','quality_rows_visited','full_prefix_scientific_hashes'))
    assert len(faults['rejected'])==7 and work['spatial']['proof_rows_loaded']==0
    assert work['representation']['exact'] and all(r['exact'] for r in work['proof_point_access'])
    # Validate compact historical authorities and their protected reports. Deep
    # audit already rehashed the entire native closure; verify its tokens again.
    protected=dict(q.audit.read(q.audit.ROOT/'BASELINE.json')['protected'])
    prior=q.audit.read(q.audit.ROOT/'AUTHORITY.json')
    assert prior['identity']=='612ae1d9d3663d29a47cdca5b01c347c4bf691637355848e7ddac7b6df8e88df'
    protected[str(q.audit.ROOT/'AUTHORITY.json')]=baseline['audit_authority']
    for name,h in prior['scientific']['evidence'].items():protected[str(q.audit.ROOT/name)]=h
    verified={p:file_hash(p)==h for p,h in protected.items()};assert all(verified.values())
    with q.access.ImmutableState(q.STORE,enrollment['root']) as state:
        assert state.verified_legacy_identity()==q.audit.EXPECTED
        body=state.body;component_mapping=body['components']
        closure_rows=[metadata(p) for p in baseline['closure']]
        hot_rows=[metadata(r['path']) for r in body['hot'].values()]
        assert body['content_root']==enrollment['content_root']
    own_markers=[b'incremental_immutable_state_access',b'immutable_state_access.py',b'production_forecast_admission.py']
    status=subprocess.check_output(['git','status','--porcelain=v1','-z'])
    unrelated=[r for r in status.split(b'\0') if r and not any(m in r for m in own_markers)]
    original=[r for r in bytes.fromhex(baseline['status_hex']).split(b'\0') if r]
    assert unrelated==original and len(unrelated)==155
    v3=subprocess.check_output(['git','diff','--name-only','--','research/sdf_cfd_mesher_v3']).decode().splitlines()
    assert not v3
    q.save('VERIFICATION.json',dict(protected=verified,legacy_closure_tokens_exact=True,
        original_checkpoint_sha256=baseline['checkpoint']['sha256'],legacy_scientific=q.audit.EXPECTED,
        unrelated_entries=155,status_equal=True,v3_modifications=0,no_historical_deletion=True,
        no_historical_writes=True,no_forecast=True,no_generation=True,no_core=True))
    q.save('COMPONENT_MAPPING.json',dict(legacy_scientific=q.audit.EXPECTED,content_root=enrollment['content_root'],
        components=component_mapping,proof_roles=body['content']['receipt_roles'],hot=body['hot'],
        exact_component_scientific=deep['component_scientific']))
    rows=[metadata(p) for p in sorted(out.rglob('*')) if p.is_file()]
    storage=dict(task=totals(rows),legacy_native_closure=totals(closure_rows),hot_objects=totals(hot_rows),
        starting_free=baseline['free'],ending_free=shutil.disk_usage(q.audit.cycle.V4).free,
        immutable_bulk_copies=0,historical_bytes_deleted=0,
        temporary_peak='Not sampled. Enrollment writes one derived object at a time; no temporary receipt/quality database.',
        measurement_boundary='Task result tree immediately before STORAGE/REPORT/AUTHORITY sealing; final console measurement includes them.',
        rows=rows,legacy_rows=closure_rows,hot_rows=hot_rows)
    q.save('STORAGE.json',storage)
    legacy=read(q.audit.ROOT/'PROFILE.json');traversals=read(q.audit.ROOT/'TRAVERSALS.json')
    comparison=dict(legacy=legacy,candidate=normal,deep=deep,
        legacy_rows=traversals['fresh_restore_sqlite_value_row_visits_calculated'],
        note='Legacy sealed prior profile; fresh processes, filesystem caches not flushed. Process read bytes are not physical disk reads. Legacy full profile includes a separate 29.291 s identity replay.')
    q.save('COMPARISON.json',comparison)
    lines=[
        '# V4 incremental immutable state access qualification','',
        '**V4IncrementalImmutableStateAccessArchitectureQualified**. Exact sealed L5 state is available through a pinned, host-local immutable read interface. Normal production initialization succeeds without historical receipt or quality traversal. This does not qualify incremental L6 transaction/publication.','',
        f'Starting HEAD: `{baseline["head"]}`. Ending HEAD is the commit containing this report (printed in the final response). Branch remains feature/sdf-cfd-mesher-v4-advancing-front.',
        f'Legacy L5: `{q.audit.EXPECTED}`. Original checkpoint SHA-256: `{baseline["checkpoint"]["sha256"]}`.',
        f'Dependency audit: `{prior["identity"]}`. All {len(verified)} protected authority/report/evidence hashes match. Full original checkpoint closure was rehashed by deep audit.',
        '', 'Access patterns and representation choices are specified completely in DESIGN.md: dense front/coordinate access, cold immutable geometry and quality, spatial global collision query, keyed immutable proof access, versioned dependency bindings, small transactional metadata, complete cold restart closure and qualification evidence.',
        '', 'The selected representation retains original immutable SQLite receipt/quality components, JSON geometry/atlas and compact proof segments. A small read-only NPY coordinate array plus key index and current-front face projection serves initialization. No receipt database migration, geometry change or authority rewrite occurred.',
        '', f'All 16 legacy components map exactly to unchanged descriptors/files. Complete deep replay verifies {deep["proof_count"]:,} live receipts and {deep["quality_count"]:,} quality rows; every derived vertex/front-face record matches. Geometry/fronts, connectivity, ownership, atlas, events, proof payloads/bindings, quality and certificate classification remain exact. Request/schedule binding matches the original adapter.',
        '', f'Content root: `{enrollment["content_root"]}`. Host access root: `{enrollment["root"]}`. Root file SHA-256: `{enrollment["candidate"]["sha256"]}`. New roots explicitly map to the old scientific identity; their differing hashes do not replace it.',
        '', 'Proof payload, pair identity/coverage and dependency binding have distinct authenticated roles. Existing base/update precedence and all stored versions remain intact. This task makes them cold during initialization; it does not compact superseded bindings or physically split existing row payloads.',
        '', 'Normal access pins root, content mapping, implementation, epoch, legacy pointer and object closure; checks identity/change tokens; holds write/delete-denying read locks; and hashes requested scientific data component bytes before decoding. It freshly hashes the access-root container and compares parsed legacy pointer/envelope metadata with the pinned mapping. Reboot or inconsistent/missing/stale/mutated metadata fails closed. Silent cold-byte damage without metadata change is detected on demand or by explicit deep audit. Privileged OS/process compromise is outside this local trust model.',
        '', 'Deep audit rehashes all bound objects, executes complete native restore and legacy scientific identity verification, and compares the complete derived geometry/front projection. It is explicit, not an automatic normal-access cost.',
        '', '| Measurement | Legacy sealed profile | Candidate normal L5 open/init |','|---|---:|---:|',
        f'| Wall seconds | {legacy["seconds"]:.6f} | {normal["seconds"]:.6f} |',
        f'| CPU seconds | {legacy["cpu"]:.6f} | {normal["cpu"]:.6f} |',
        f'| Peak RSS bytes | {legacy["peak_rss"]:,} | {normal["peak_rss"]:,} |',
        f'| Retained RSS bytes | {legacy["retained_rss"]:,} | {normal["retained_rss"]:,} |',
        f'| Process read bytes | {legacy["io_delta"]["read_bytes"]:,} | {normal["io"]["read_bytes"]:,} |',
        f'| Process read operations | {legacy["io_delta"]["read_count"]:,} | {normal["io"]["read_count"]:,} |',
        f'| SQLite value-row visits | {comparison["legacy_rows"]:,} | 0 |',
        '| Proof rows deserialized / payloads hashed | Full repeated population work | 0 / 0 |',
        '| Historical quality visits | Complete 726,399-row table; repeated | 0 |',
        '| Nonreceipt canonical passes | 4 | 0 |',
        '', f'Legacy restore alone was {legacy["stages"]["restart"]:.6f} s, initialization {legacy["stages"]["initialization"]:.6f} s and separate identity replay {legacy["stages"]["repeat_identity"]:.6f} s. Candidate open alone was {normal["open_seconds"]:.6f} s. Candidate authenticates {normal["stats"]["component_token_checks"]} object tokens and one root; {normal["stats"]["component_content_checks"]} demanded components hash {normal["stats"]["component_bytes_hashed_on_read"]:,} bytes. Mapped coordinates: {normal["stats"]["bytes_mapped"]:,} bytes. No process writes occurred before recording its result.',
        '', 'SQLite connections, native restart, scientific_digest/hash and forecast/proposal/step entry points were denied during candidate initialization. Zero-row behavior is guarded, not inferred only from elapsed time. The adapter still reads/copies all 101,370 existing coordinates because its API requests the prefix; no claim of constant-size geometry access is made.',
        '', 'Representation benchmark and spatial/point-access measurements:',
        '```json',json.dumps(work,indent=2),'```',
        '', f'Fault tests: {tests["stdout"].strip()}. Seven real-L5 initialization authority faults also reject: {json.dumps(faults["rejected"],sort_keys=True)}.',
        '', 'An added delta unit fixture initially retained a stale checkpoint envelope. The implementation correctly rejected it. The fixture was corrected and all final tests passed; TESTS_FIXTURE_FAILURE.json preserves that trial.',
        '', 'Tests cover missing/changed components, preserved-mtime writes, wrong component/root mapping, wrong epoch, stale/edited overlays, changed proof/binding records, changed geometry, root damage, incomplete checkpoint metadata, active writers/hardlink aliases, stale/concurrent publishers, interruption before root write and after root write before pointer switch, deep detection of mocked silent corruption, view expiry, exact unchanged legacy binding code and unchanged legacy authentication.',
        '', f'L6 read-only initialization selected exactly `{normal["increment"]}` m (`{float(normal["increment"]).hex()}`). L6 forecast = NO. L6 generation = NO. Core = NO.',
        '', f'Fresh-process deterministic normal replay: {replay["seconds"]:.6f} s; exact same legacy identity, access root, complete adapter binding and empty bound-cache identity `{normal["empty_bound_cache_identity"]}`. Cache result remains None; no predictor is invoked.',
        '', f'Final deep audit: {deep["seconds"]:.6f} s wall, {deep["cpu"]:.6f} s CPU, {deep["peak_rss"]:,} bytes peak RSS. Enrollment: {enrollment["seconds"]:.6f} s. Tests: {tests["seconds"]:.6f} s. Initial prototype results are retained with _INITIAL suffix; final measurements use the lifetime-hardened implementation.',
        '', f'Storage at sealing boundary: task {storage["task"]["physical"]:,} physical / {storage["task"]["logical"]:,} logical bytes; derived hot objects {storage["hot_objects"]["physical"]:,} physical bytes. Original native closure {storage["legacy_native_closure"]["physical"]:,} bytes is referenced in place. Immutable bulk copies = 0; historical bytes deleted = 0. Starting free {storage["starting_free"]:,}; ending free {storage["ending_free"]:,}. No physical-space saving is claimed. Temporary peak was not sampled; no temporary receipt or quality database was created.',
        '', 'Disk preflight found substantial measured capacity relative to the retained 27,750,690,816-byte prior active union. The candidate writes only the measured small derived views and metadata. No guessed reserve or historical cleanup was used.',
        '', 'Transaction implications: immutable components may later be referenced instead of copied/rehash-serialized; affected bindings/quality can be updated separately; new proofs may be appended. Those operations require their own stale-base, coverage, atomicity and recovery qualification. Current receipt-chain storage, complete quality materialization during publication and native journal copying remain unresolved.',
        '', 'Qualification retains full legacy/deep replay, deterministic component comparison and detailed failure tests. Future production can use the same trusted read interface and changed-state work, with deep verification on explicit audit or invalid trust metadata. No ProductionFast policy was implemented.',
        '', 'Git: all 155 unrelated dirty entries match exactly. V3 modifications = 0. Only task-owned tools, the explicit adapter entry point, tests and compact evidence are committed. Candidate derived data remains local and reproducibly bound.',
        '', 'Remaining limits: host/boot-specific trust; no silent-cold-media detection until demand/deep audit; existing JSON cold-geometry loading and geometry-prefix construction remain; no cross-platform qualification; no mesh transaction/publication qualification; no future storage extrapolation.',
        '', 'Exactly one recommended next task: **QualifyV4IncrementalTransactionPublicationFromSealedLayer5**. Use sealed L5 to qualify changed-state updates, proof/binding separation, stale-base handling, atomic publication and recovery before any L6 advancement.'
    ]
    (out/'REPORT.md').write_text('\n'.join(lines)+'\n',encoding='utf-8')
    source_paths=[q.audit.cycle.V4/'tools'/name for name in ('immutable_state_access.py','qualify_immutable_state_access.py','seal_immutable_state_access.py','production_forecast_admission.py')]+[q.audit.cycle.V4/'tests/test_immutable_state_access.py']
    implementation={str(p.relative_to(q.audit.cycle.V4)):file_hash(p) for p in source_paths}
    evidence={p.name:file_hash(p) for p in sorted(out.iterdir()) if p.is_file() and p.name not in ('AUTHORITY.json',)}
    scientific=dict(classification='V4IncrementalImmutableStateAccessArchitectureQualified',legacy_l5=q.audit.EXPECTED,
        original_checkpoint_sha256=baseline['checkpoint']['sha256'],starting_head=baseline['head'],
        access_root=enrollment['root'],content_root=enrollment['content_root'],evidence=evidence,implementation=implementation,
        transaction_publication_qualified=False,no_L6_forecast=True,no_L6_generation=True,no_core=True,
        trust_scope='Pinned host/boot NTFS identity/change tokens with held read locks; on-demand SHA and explicit complete deep audit')
    authority=dict(identity=q.identity(scientific),scientific=scientific)
    q.save('AUTHORITY.json',authority)
    print(json.dumps(dict(identity=authority['identity'],report=file_hash(out/'REPORT.md'),authority=file_hash(out/'AUTHORITY.json'),
        seconds=time.perf_counter()-start,storage=storage['task']),indent=2),flush=True)


if __name__=='__main__':main()

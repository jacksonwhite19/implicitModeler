"""Seal compact architecture evidence. No scientific or historical writes."""
from pathlib import Path
import json, shutil, subprocess
from collections import defaultdict
from composable_state import CommittedState
from path_preserving_deduplication import durable_json, file_hash, metadata, physical_key
from sdf_cfd_mesher_v4.common.authority import identity

V4=Path(__file__).resolve().parents[1]; REPO=V4.parents[1]
OUT=V4/'results/composable_incremental_state'
read=lambda p:json.loads(Path(p).read_text())


def run():
    b=read(OUT/'BASELINE.json'); n=read(OUT/'NORMAL.json'); fresh=read(OUT/'FRESH_RECURSIVE_NORMAL.json')
    deep=read(OUT/'DEEP.json'); rd=read(OUT/'REFRAMED_DEEP.json'); controls=read(OUT/'COMPOSITION.json')
    assert deep['passed'] and rd['passed'] and controls['passed']
    assert deep['scientific']==rd['scientific']==n['scientific']
    assert n['plan']['increment_hex']==fresh['plan']['increment_hex']=='0x1.744eb7e7e7516p-12'
    assert all(not r[k] for r in (n,fresh) for k in ('admission','forecast','generation'))
    for reference in (rd['reference'],controls['child']['reference'],controls['grandchild']['reference']):
        with CommittedState(reference) as state: state.check()
    status=subprocess.check_output(['git','status','--porcelain=v1','-z'])
    unrelated=[r for r in status.split(b'\0') if r and b'composable_' not in r]
    assert b'\0'.join(unrelated).hex()==b['status_hex']
    protected=read(V4/'results/layer6_incremental_readiness/PREFLIGHT.json')['protected_files']
    for p,h in protected.items(): assert file_hash(p)==h
    tests=dict(passed=120,failed=0,skipped=0,seconds=20.19,
        command='python -m pytest -q -p no:cacheprovider',
        files=['test_composable_state.py','test_composable_publication.py','test_clean_sheet_candidate.py','test_clean_sheet_resume.py','test_clean_sheet_isolation.py','test_immutable_state_access.py','test_incremental_transaction.py'])
    durable_json(OUT/'TESTS.json',tests)
    sources=list((V4/'tools').glob('composable_*.py'))+list((V4/'tests').glob('test_composable_*.py'))
    sources += [V4/'tools/qualify_composable_state.py',Path(__file__)]
    implementation={str(p.relative_to(V4)).replace('\\','/'):file_hash(p) for p in sources}
    durable_json(OUT/'IMPLEMENTATION.json',dict(identity=identity(implementation),files=implementation))
    # Physical allocation snapshot before final report/authority bookkeeping.
    paths=list(OUT.rglob('*')); paths=[p for p in paths if p.is_file() and p.name not in ('STORAGE.json','REPORT.md','AUTHORITY.json','QUALIFICATION.json')]+sources
    rows=[metadata(p) for p in paths]; unique={physical_key(r):r for r in rows}
    groups=defaultdict(int)
    for p,row in zip(paths,rows):
        if p in sources: role='implementation'
        elif any(x in p.parts for x in ('control_child_final','control_grandchild_final')): role='qualified_composition_controls'
        elif 'reframed_L5_final' in p.parts: role='recursive_production_metadata'
        elif any(x in p.parts for x in ('control_child','control_grandchild','reframed_L5')): role='superseded_development_evidence'
        else: role='compact_reports'
        groups[role]+=row['cluster_bytes']
    storage=dict(files=rows,allocated_bytes=sum(r['cluster_bytes'] for r in unique.values()),categories=dict(groups),
        starting_free_bytes=b['free_bytes'],ending_free_bytes=shutil.disk_usage(REPO.anchor).free,
        historical_bulk_copies=0,historical_deletions=0,temporary_peak='Not sampled; pytest temporary files excluded',
        scope='Task code/evidence before STORAGE, REPORT, QUALIFICATION and AUTHORITY. Excludes Git metadata.')
    durable_json(OUT/'STORAGE.json',storage)
    q=dict(classification='V4ComposableIncrementalBaseAccessAndTransitionQualified',starting_head=b['head'],
        supported_claim='Native and incremental states share authenticated lazy access; recursive changed-state composition, complete L5 equivalence and L6 planning pass. No L6 physical result is claimed.',
        authoritative_layer=5,legacy_L5=n['scientific'],original_access=n['plan']['base_root'],original_content=n['plan']['base_content'],
        recursive_access=rd['reference']['root'],recursive_content=rd['content_root'],implementation=identity(implementation),
        normal=n,recursive_normal=fresh,deep=deep,recursive_deep=rd,composition=controls,tests=tests,
        protected_hashes_checked=len(protected),unrelated_dirty_entries=len(unrelated),unrelated_status_preserved=True,
        L6_admission=False,L6_forecast=False,L6_generation=False,L7_work=False,core=False,
        remaining_concerns=['Cold demanded legacy containers are hashed in full before first lookup.',
          'Lookup and metadata work grow with component ancestry, not a fixed layer horizon.',
          'Assembly retains the complete compact-pair qualification accounting scan.',
          'Actual L6 geometry, end-to-end publication, replay and retention remain untested and were prohibited here.'],
        next_task='QualifyV4CapsuleLayer6IncrementalEndToEndBounded')
    q['identity']=identity(q);durable_json(OUT/'QUALIFICATION.json',q)
    def timing(label,r):
        m=r['metrics'];return f"| {label} | {m['seconds']:.6f} | {m['cpu']:.6f} | {m['peak_rss']} | {m['retained_rss']} | {m['io']['read_bytes']} | {m['io']['write_bytes']} |"
    report=f'''# Composable incremental state qualification

Classification: {q['classification']}

{q['supported_claim']}

Starting HEAD: {b['head']}. Ending HEAD is the commit containing this report and is printed in the terminal response.
Branch: feature/sdf-cfd-mesher-v4-advancing-front. All 155 unrelated dirty entries retain their exact status. No V3 or historical authority file was edited.

## Authorities and exact equality

Production layer remains 5. Scientific identity: {n['scientific']}.
Original incremental access: {q['original_access']}.
Original content: {q['original_content']}.
Clean-sheet qualification: 1cb74365c156d207ed93260eb9d16ee0988bb6dd6381ac6e990a3c98f4c83d87.
Immutable-access qualification: 7744d0bc88a60f75e6048edb105c983bcc351e9946b3710b32faa9e9df76e5a8.
Dependency audit: 612ae1d9d3663d29a47cdca5b01c347c4bf691637355848e7ddac7b6df8e88df.
Sixty protected report/authority/checkpoint files were freshly rehashed and preserved. Existing trusted readers verified component roots/tokens. The unchanged initializer verified surface, geometry, request, schedule, front, ownership and source ancestry. Their exact bindings are in QUALIFICATION.json.

Complete canonical scientific equality passed for both the common L5 view and the new recursive representation. The comparison covers geometry, connectivity, ownership, all quality, proof payloads/population, current bindings, atlas, events and authority state. The L5 proof population is 15,842,336; quality population is 726,399. The first deep pass visited 12,059,568 old and 3,782,768 new proof records, plus 574,362 base and 219,609 quality delta rows. The recursive deep pass separately validated its native parent and then the complete L5 view, so its parent counters are 24,119,136 proof rows and 1,148,724 quality rows. These are explicit qualification costs, never normal open work.

New recursive content root: {rd['content_root']}.
New recursive access root: {rd['reference']['root']}.
These representation identities differ from the legacy digest. The independently computed legacy digest is exactly unchanged.

## Architecture and resolved blockers

The common interface exposes the authoritative layer, scientific/content/access identities, component map, geometry/front/ancestry/ownership, exact proof/quality lookup, bindings in current receipts, atlas/dependencies, parent reference and transaction metadata. Native and incremental adapters authenticate their own backing formats without rebuilding a legacy state.

The new admission API accepts a common authenticated base instead of enforcing L4-to-L5. The old pinned public adapter and harness remain unchanged. The common initializer uses the same qualified binding algorithm and does not admit arbitrary dictionaries. The new planner supplies target layer, schedule index, offsets and horizon to the layer-generic transition. Assembly uses common quality lookup and proof-segment access. Publication references an authenticated parent rather than native checkpoint fields.

Recursive proof lookup checks the newest exact pair index and delegates misses. Current binding overlays change inputs without copying the old proof payload. Quality supports inherited numerical data, input-only updates, changed numerical data and new records. Explicit tombstones stop lookup. Changed controls carry previous-record identities; stale/conflicting records are rejected. Normal root/component authentication uses the existing host epoch, tokens and deny-write handles. Demanded components are hashed before decode. Explicit deep mode retains full verification.

Transactions retain the existing writer lock, stale-base check, private writes and atomic root switch. Prepared-plan retry requires exact external plan and acceptance identities. Parent roots stay pinned and available. No flattening is required.

## Bounded composition

Child and grandchild publication/reopen passed. One synthetic proof was added then removed; two bindings changed; one quality record got input-only metadata, one was added and then numerically changed. Original production proof payloads stayed unchanged. Three quality delta rows were written across both publications; unchanged scientific components were referenced. Store counters report zero historical receipt/quality rows visited during publication; validation and point lookup are counted separately.
Child root: {controls['child']['reference']['root']}.
Grandchild root: {controls['grandchild']['reference']['root']}.
These non-geometric controls cannot be used as physical advancement authorities.

Cold L5 proof+quality lookup: {controls['cold_lookup']['seconds']:.9f} s, two selected rows (one pair-key index and one quality delta). Warm: {controls['warm_lookup']['seconds']:.9f} s, the same two rows. The depth-three lookup control took {controls['recursive_lookup']['metrics']['seconds']:.9f} s: two pair-key rows and three quality delta rows, four quality queries. It also verified removal fail-closed behavior. Full composition control: {controls['metrics']['seconds']:.9f} s; 24,668,892,506 process read bytes and 73,728 write bytes. Eight legacy receipt point queries returned zero rows while establishing new-key absence. Cold component authentication explains the large read-byte count; no saving is inferred. The final composition benchmark overlapped the explicit deep audit.

## Runtime and normal access

| Scope | Wall s | CPU s | Peak RSS B | Retained RSS B | Read B | Write B |
|---|---:|---:|---:|---:|---:|---:|
{timing('Original incremental L5 common open/planning',n)}
{timing('Fresh recursive L5 open/planning',fresh)}
{timing('Common L5 deep qualification',deep)}
{timing('Recursive L5 and parent deep qualification',rd)}

Both normal opens ran with SQL, forecast and legacy restart blocked. Historical proof/quality rows, full-prefix scientific hashes and full-prefix deep copies were zero. Each native parent authenticated one root, checked 45 component tokens, verified four demanded component contents and mapped 2,027,400 geometry bytes. Normal read operations were 136 and 132; write operations were zero. Root/metadata work grows with component ancestry. No constant-depth claim is made.

## Readiness and prohibited work

Native L4 plans target L5 with the exact known increment. Both L5 representations plan base=5, target=6, increment=0.00035506009452908446 m, binary64 0x1.744eb7e7e7516p-12. Schedule index=5. Vertex offsets=84,475 and 101,370. Cell IDs=6,000,000 through 6,033,785. Front=16,895 vertices and 33,786 triangles; six existing front arrays. Remaining schedule positions=6 through 11, recorded only as planning metadata.

L6 admission=NO. L6 forecast=NO. L6 geometry=NO. L7 work=NO. Core=NO.

## Storage, tests and limitations

Physical snapshot before final report/authority bookkeeping: {storage['allocated_bytes']} B. Categories: {json.dumps(storage['categories'],sort_keys=True)}.
Starting free space: {storage['starting_free_bytes']} B; snapshot ending free space: {storage['ending_free_bytes']} B. No historical bulk copied or deleted. Temporary peak was not sampled; test temporary directories and Git metadata are excluded. Final allocation is printed after sealing. Original immutable payloads remain referenced in place.

120 tests passed, 0 failed, 0 skipped in 20.19 s. Tests cover returned-record isolation, generic offsets, proof/binding/quality overlays, removals, conflicts, missing/corrupt ancestors/components, wrong roots, stale/concurrent writers, interrupted publication, recovery, idempotent retry, recursive reopen and explicit deep behavior. Initial development failures were fixed and retained in DEVELOPMENT_FAILURES.json. Superseded development roots are not current qualified restore targets.

Cold legacy container hashing, ancestry-depth metadata work and the assembly compact-pair accounting scan remain measured concerns. This task does not qualify L6 geometry or claim elimination of those costs. The actual layer-generic generation path was not executed, as required by task scope.

Architecture identity: {q['identity']}.
Implementation identity: {q['implementation']}.
Recommended next task: QualifyV4CapsuleLayer6IncrementalEndToEndBounded.
'''
    (OUT/'REPORT.md').write_text(report,encoding='utf-8',newline='\n')
    names=['BASELINE.json','NORMAL.json','FRESH_RECURSIVE_NORMAL.json','DEEP.json','REFRAMED_DEEP.json','REFRAMED_REFERENCE.json','COMPOSITION.json','TESTS.json','IMPLEMENTATION.json','STORAGE.json','DESIGN.md','DEVELOPMENT_FAILURES.json','QUALIFICATION.json','REPORT.md']
    authority=dict(classification=q['classification'],qualification_identity=q['identity'],files={n:file_hash(OUT/n) for n in names})
    authority['identity']=identity(authority);durable_json(OUT/'AUTHORITY.json',authority)
    print(json.dumps(dict(classification=q['classification'],qualification=q['identity'],report=file_hash(OUT/'REPORT.md'),authority=file_hash(OUT/'AUTHORITY.json'),storage=storage['allocated_bytes']),indent=2))


if __name__=='__main__': run()

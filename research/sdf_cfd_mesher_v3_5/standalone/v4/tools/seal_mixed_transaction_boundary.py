"""Seal measured transaction groundwork, explicitly not production integration."""
from pathlib import Path
import hashlib,json,subprocess,zipfile
ROOT=Path(__file__).resolve().parents[1];REPO=ROOT.parents[1]
OUT=ROOT/'results/production_automatic_termination_remesh'

def digest(data):return hashlib.sha256(data).hexdigest()
def load(path):return json.loads(path.read_text(encoding='utf-8-sig'))
def save(name,value):
    (OUT/name).write_bytes((json.dumps(value,sort_keys=True,indent=2,allow_nan=False)+'\n').encode())

def seal():
    rows={}
    for name in ('clean_curved_passage','analytic_cavity_with_control'):
        p=OUT/name/'final';r=load(p/'result.json');runtime=load(p/'runtime.json');replay=load(p/'RESTART_REPLAY.json')
        pointer=load(p/'checkpoints/CURRENT.json');state=load(p/'checkpoints'/pointer['checkpoint'])['payload']
        old=load(p/'checkpoints'/f"STATE_{state['previous']}.json")['payload'];k=r['event']['layer']
        assert r['transaction']['committed'] and replay['passed']
        checks=r['transaction']['invocation']['receipt']['checks'];assert all(checks.values())
        from collections import Counter
        rows[name]=dict(layer=k,cells_before=len(old['mesh']['cells']),cells_after=len(state['mesh']['cells']),
            cell_types=dict(Counter(c['kind'] for c in state['mesh']['cells'])),
            committed_cells_removed=len(r['transaction']['details']['removed_cells']),
            cells_added=len(r['transaction']['details']['manifest']['replacements']),
            layer_map=r['layer_count_histogram'],terminated_columns=sum(v for l,v in r['layer_count_histogram'].items() if int(l)<k),
            stopped_vertices=sum(d<k for d in state['mesh']['vertex_depth']),
            source_vertices=len(state['mesh']['wall']['coordinates']),
            checks=checks,exact_pairs=r['validation']['certificate']['required_pairs'],
            invalidated=r['transaction']['invalidations']['invalidated'],retained=r['transaction']['invalidations']['retained'],
            trace=[x['state'] for x in r['transaction']['decision']['trace']],
            continuation_passed=r['continued']['passed'],sealed_equivalence=r['equivalence'],
            fresh_wall_hash=digest(__import__('struct').pack('<'+'d'*(3*len(state['mesh']['wall']['coordinates'])),
                *(v for xyz in state['mesh']['wall']['coordinates'] for v in xyz))),
            mesh_hash=r['final_mesh_hash'],state_hash=r['final_state_hash'],restart=replay,
            runtime=runtime)
    save('SUMMARY.json',dict(classification='V4ProductionAutomaticTerminationRemeshInsufficient',
        actual_production_integration=False,automatic_remesh_enabled=False,rows=rows,
        scientific_equivalence=False,full_task_completed=False))
    sf=load(OUT/'SF2_REGRESSION.json'); restart=load(OUT/'SF2_CHECKPOINT_RESTART.json')
    save('DETERMINISM.json',dict(private_transaction_restart=True,real_mesh_crash_replays=10,
        policy_event_idempotence=True,geometry_rebuilt_from_pre_event_state=True,
        validation_reuse='Complete candidate mesh identity and validation authority must match',
        production_runner_restart=False,sealed_scientific_equivalence=False,
        curved_wall_expected='3ffa2ff247d982b873f8ebf9a7ef2d51c51a17faea8ee1f64c9d257a98b1b261',
        curved_wall_actual=rows['clean_curved_passage']['fresh_wall_hash'],cause='Unresolved; no authority coordinates changed to mask the mismatch'))
    save('GENERAL_SDF_REPLAY.json',dict(performed=False,reason='Actual Task-4 mixed-front adapters remain incomplete',
        historical_authority_preserved=True,no_prevention_claim=True))
    save('TESTS.json',dict(command='python tools/run_mixed_transaction_tests.py',passed=113,failed=1,
        new_transaction_tests=17,failed_test='test_clean_support_and_first_layer[clean_curved_passage]',
        failure='Fresh wall coordinate hash differs from sealed reference',
        raw='tests_final.txt',sealed_test_output_writes_redirected=True))
    a=rows['clean_curved_passage'];b=rows['analytic_cavity_with_control']
    report=f'''## Decision question

Can V4 automatically trigger, execute, commit, checkpoint, restart, and continue from a bounded local rollback/remesh event inside the actual production rolling advancing-front runner?

**Not yet. Classification: V4ProductionAutomaticTerminationRemeshInsufficient.**

This task produced mixed-transaction groundwork and two fresh bounded exercises. It did not complete integration into the actual Task-4 rolling loop. Automatic rollback/remeshing is not enabled. The distinction is explicit: a private transaction exercise is not production-runner qualification.

### A. Authority verification

Input HEAD: dc6fe5c2d4eb9db66b36ee07baac439bc067b72f. Branch matches the request. The supplied trigger-policy and remesh report/AUTHORITY hashes pass. SOURCE_BINDINGS.json binds the preserved negative cohort. Sealed evidence is unchanged. The only existing source change is a format guard in rolling_state.py; its original Git blob remains available at the input commit.

### B. Qualified policy and remesh mechanics

The existing evidence-driven policy, automatic cavity discovery, standard prism/pyramid/tet construction, quality routines, exact face certificate, volume audit and Gmsh discrete ingestion are reused. No hard threshold or certificate formula changed. A missing dependency required an isolated CasADi installation; psutil, SymPy and pytest were also installed in the isolated dependency directory. No existing environment was replaced. Final exercises used Python 3.13.2, NumPy 2.4.0, SciPy 1.16.3, one OpenBLAS thread. A first run used Anaconda Python 3.13.9/NumPy 2.3.5. Both environments reproduce the same curved wall mismatch.

### C. Production integration architecture

remesh_transaction.py adds a content-addressed mixed-state store and evidence-based dispatch. Task-4 still enters through task4l_run.py, task4_prepare_next.py and task4_certify_candidate.py. Those stages do not dispatch this store. The new format guard rejects a mixed pointer before the old reader reconstructs rectangular prism connectivity. RUNNER_BOUNDARY.json lists the missing adapters. No separate exercise is represented as the actual rolling loop.

### D. Transaction semantics

CURRENT.json is the sole publication point. Backend geometry is private. A stable pre-event snapshot and pending evidence exist before dispatch. Successful validation publishes a new snapshot atomically. Failure leaves the prior pointer, mesh and local receipt state unchanged. Event identity derives from the base-state and evidence hashes. This is a single-writer primitive; runner-level serialization is required. It is not a concurrent-writer transaction system.

### E. Mixed-cell committed state

Snapshots explicitly store points, cells, ordered faces, outer interface, per-vertex stop depths and per-column last-prism layers. They also store receipt state and event history. Wall coordinates, first-layer coordinates and first-layer connectivity are protected. Unchanged cell geometry is identified from complete ordered connectivity and coordinates.

### F. Trigger dispatch

Both fresh exercises follow PassiveNominal → ActiveWarning → RemeshCandidate → RollbackRemesh → Resolved inside the private store. Curved passage uses current exact collision plus invalid continuation. The spherical case uses its exact nonpositive remaining radius and invalid continuation. No failed-trial count, compression ratio or conditioning threshold triggers the policy. Actual Task-4 dispatch is not implemented.

### G. Rollback and cavity discovery

The existing discover routine receives vertices from current failing cells/intersections. No accepted map is loaded during construction. In these two cases the committed prefix remains valid, so **zero previously committed cells are removed**. The invalid prospective cells are replaced or omitted. Thus these exercises do not qualify live rollback of an overlapping committed upper stack. General-SDF integration is still required for that behavior. Unit tests cover upper-cell manifest removal and rejection of hidden deletions.

### H. Remesh execution

Fresh construction uses the qualified local templates and full bounded-fixture validation. Curved: {a['cells_before']} cells before, {a['cells_after']} after; {a['cell_types']}. Spherical: {b['cells_before']} before, {b['cells_after']} after; {b['cell_types']}. Reference outputs are comparison-only and are opened after construction, certification, ingestion and continuation.

### I. Risk-atlas rebuild

Not integrated. The Task-4 RiskAtlas still assumes fixed topology and a rectangular trajectory array. No atlas invalidation count or rebuilt mixed atlas is claimed.

### J. Forecast rebuild

Not integrated. The bounded continuation routine respects the local depth map and advances a qualified neighboring column. This is not Task-4 forecast suppression or cache refresh. Terminated-column masks, asynchronous local front state and persistent planner migration remain outstanding.

### K. Quality-receipt invalidation

The new receipt index binds ordered cell geometry, face geometry/ownership, neighbor geometry where used, and implementation authority. Curved: {a['invalidated']['quality']} invalidated, {a['retained']['quality']} retained. Spherical: {b['invalidated']['quality']} invalidated, {b['retained']['quality']} retained. Fresh calculations verify retained values exactly in canonical JSON form. A tuple/list roundtrip comparison defect was reproduced and fixed. Task-4 validation-receipt format migration remains unimplemented.

### L. Certificate-state invalidation/rebuild

Curved: {a['invalidated']['certificate']} proof receipts invalidated, {a['retained']['certificate']} retained. Spherical: {b['invalidated']['certificate']} invalidated, {b['retained']['certificate']} retained. All candidate geometry was independently recertified with the existing mathematics; the retained receipts were compared to the fresh outcomes. New complete certificates cover {a['exact_pairs']:,} and {b['exact_pairs']:,} relations, respectively, with zero forbidden outcomes. These receipts are not yet bound into the Task-4 production manifest. No global production proof-cache migration is claimed.

### M. Checkpoint/restart

Both real-mesh exercises pass restart before dispatch, after private rollback, before publication and after publication. Duplicate dispatch does not call the backend again. Replays rebuild the cavity from pre-event evidence. Validation reuse is permitted only after the complete candidate mesh and authority match the primary validated state. Exact private final-state identities match. The legacy production reader correctly rejects a mixed checkpoint; production mixed restart is not qualified.

### N. Failure rollback

A forced failure after private remesh restores the exact pre-event state in both real-mesh exercises. Unit tests also cover mutation of private geometry before an exception, missing validation gates, stale contexts, corrupt pointers and removal-manifest errors. No failed candidate is published.

### O. Primary live integration case

Curved passage: a fresh bounded transaction exercise passes; actual production-loop integration is not run. The layer-4 result has 32 columns ending at 3 and 224 at 4. Current hard quality, ownership, positive cell maps, volume balance/degree, exact geometry and ingestion pass. Sealed map, connectivity, outer topology and certificate outcomes match. Exact quality values and the full continuation record do not. The fresh wall hash is {a['fresh_wall_hash']}; the sealed wall hash is 3ffa2ff247d982b873f8ebf9a7ef2d51c51a17faea8ee1f64c9d257a98b1b261. This discrepancy remains unresolved; no geometry was altered to force the hash.

### P. Second live integration case

Spherical cavity with continuing control: a fresh bounded transaction exercise passes at layer 13; actual production-loop integration is not run. The map has 32 columns at 12 and 32 at 13. Current hard quality, complete volume checks, exact geometry, ingestion and continuation pass. Sealed map/connectivity/outer topology/certificate outcomes match. Exact quality values and the full continuation record do not. This is not scientific equivalence qualification.

### Q. General-SDF integration/replay

Not performed. Its sealed authority is preserved. No claim is made that an earlier invalid commit was prevented. This remains necessary to demonstrate actual rollback of previously committed overlapping upper prisms.

### R. Neighbor continuation

The qualified bounded continuation routine passes for both fresh meshes. The committed private result remains the remesh state; the continuation is a validated smoke candidate, not a subsequent Task-4 rolling commit. No full-layer or full-depth continuation claim is made.

### S. SF2 regression

All 63 preserved negative snapshots have zero remesh triggers; {sf['sf2_snapshots']} are SF2 snapshots. The modified actual CommitStore reads the sealed layer-20 checkpoint and its complete chain successfully in read-only mode. The CURRENT hash remains {restart['pointer_sha256']}. No geometry, layer count, correction, termination or remesh changes. Full SF2 was not run.

### T. Runtime and locality

Private event times, including validation, ingestion and continuation: curved {a['runtime']['event']:.6f} s; spherical {b['runtime']['event']:.6f} s. Full exercise times including clean-support reconstruction: {a['runtime']['total']:.6f} s and {b['runtime']['total']:.6f} s. Continuation alone: {a['runtime']['continuation']:.6f} s and {b['runtime']['continuation']:.6f} s. Receipt invalidation: {a['runtime']['receipt_invalidation']:.6f} s and {b['runtime']['receipt_invalidation']:.6f} s. Peak process RSS: {max(a['runtime']['peak_rss'],b['runtime']['peak_rss']):,} bytes. Largest validated bounded mesh: {max(a['cells_after'],b['cells_after'])} cells. These are prototype transaction costs, not qualified production event costs. Full small-fixture recertification and JSON snapshots dominate; Task-4 local atlas/cache rebuild and production overhead are not measured. Replay times remain separate.

### U. Determinism

Private event IDs, rebuilt geometry, maps and final snapshots reproduce exactly under restart. All ten real-mesh crash/failure replay checks pass. Runtime/RSS are excluded from state identities. Exact equivalence to sealed quality/continuation records fails in both cases. The curved clean-wall regression also fails. Production scientific replay is therefore not qualified.

### V. Tests

Final retained suite: 113 passed, 1 failed in 12.09 s, including 17 new transaction tests. Failure: test_clean_support_and_first_layer[clean_curved_passage], due to the wall identity mismatch above. No assertion or scientific gate was weakened. Retained tests that write derived accounting are redirected into this task directory. Their earlier accidental write to the sealed curved accounting file was restored byte-for-byte from the input commit. tests_final.txt contains the exact output. Real-mesh restart/forced-failure checks pass separately.

### W. Classification

V4ProductionAutomaticTerminationRemeshInsufficient. The requested production integration is incomplete. Valid private transactions do not satisfy the actual-loop, atlas, forecast, production certificate, committed-upper-stack rollback or scientific-equivalence requirements.

### X. Production readiness

**No.** Automatic rollback/remeshing is not ready to remain enabled and has not been enabled. The old production path remains rectangular and rejects mixed checkpoints explicitly. Wall, first-layer authority, hard gates, certificate mathematics and V3 remain unchanged.

### Y. Recommended next task

Complete this integration task before selecting a downstream task: resolve the authority/runtime mismatch; implement and validate mixed-front atlas/forecast/planner adapters; migrate production quality/certificate state; integrate dispatch and publication into Task-4 prepare/certify; then run the general-SDF committed-upper-stack rollback and live post-remesh rolling continuation. Do not start full core integration on this result.

Large exact records and checkpoints are stored losslessly in EVIDENCE.zip. ARTIFACT_INDEX.json contains their raw SHA-256 values. restore_mixed_transaction_evidence.py restores the task-owned files for checkpoint replay. COMMITTED_PATHS.json lists the V4-only commit scope. No V3 edits were made; unrelated dirty entries are preserved.
'''
    (OUT/'report.md').write_text(report,encoding='utf8',newline='\n')
    # Archive only task-owned large evidence. Verify every byte before unlink.
    large=sorted(p for p in OUT.rglob('*') if p.is_file() and p.suffix=='.json' and p.stat().st_size>1_000_000)
    index={}
    with zipfile.ZipFile(OUT/'EVIDENCE.zip','w',compression=zipfile.ZIP_DEFLATED,compresslevel=9) as archive:
        for path in large:
            relative=path.relative_to(OUT).as_posix();data=path.read_bytes();index[relative]=dict(sha256=digest(data),bytes=len(data))
            info=zipfile.ZipInfo(relative,date_time=(1980,1,1,0,0,0));info.compress_type=zipfile.ZIP_DEFLATED
            archive.writestr(info,data,compresslevel=9)
    with zipfile.ZipFile(OUT/'EVIDENCE.zip') as archive:
        for path in large:
            relative=path.relative_to(OUT).as_posix()
            assert digest(archive.read(relative))==index[relative]['sha256']
            assert path.resolve().is_relative_to(OUT.resolve())
            path.unlink()
    save('ARTIFACT_INDEX.json',dict(archive='EVIDENCE.zip',members=index))
    for path in OUT.rglob('*'):
        if path.is_file() and path.suffix in ('.json','.txt','.md'):
            data=path.read_bytes().replace(b'\r\n',b'\n')
            if path.suffix=='.txt':data=b'\n'.join(line.rstrip() for line in data.split(b'\n'))
            path.write_bytes(data)
    sources=['src/sdf_cfd_mesher_v4/advancing_front/rolling_state.py',
             'src/sdf_cfd_mesher_v4/advancing_front/remesh_transaction.py','tests/test_remesh_transaction.py']
    sources += ['tools/'+p for p in ('exercise_mixed_remesh_transactions.py','replay_mixed_transaction_checkpoints.py',
        'audit_mixed_runner_boundary.py','run_mixed_transaction_tests.py','seal_mixed_transaction_boundary.py','restore_mixed_transaction_evidence.py')]
    # Stable source bytes before hashing; V4 has explicit -text attributes.
    for rel in sources:
        p=ROOT/rel
        if not rel.endswith('/rolling_state.py'):p.write_bytes(p.read_bytes().replace(b'\r\n',b'\n'))
    manifest=[str((ROOT/p).relative_to(REPO)).replace('\\','/') for p in sources]
    manifest += [str(p.relative_to(REPO)).replace('\\','/') for p in OUT.rglob('*') if p.is_file()]
    manifest += [str((OUT/p).relative_to(REPO)).replace('\\','/') for p in ('COMMITTED_PATHS.json','AUTHORITY.json','HASHES.json')]
    save('COMMITTED_PATHS.json',sorted(set(manifest)))
    evidence={str(p.relative_to(ROOT)).replace('\\','/'):digest(p.read_bytes()) for p in OUT.rglob('*')
              if p.is_file() and p.name not in ('AUTHORITY.json','HASHES.json','seal.txt')}
    authority=dict(task='IntegrateV4AutomaticTerminationRemeshIntoProductionRunner',
        classification='V4ProductionAutomaticTerminationRemeshInsufficient',completed=False,production_enabled=False,
        input_head='dc6fe5c2d4eb9db66b36ee07baac439bc067b72f',source_sha256={p:digest((ROOT/p).read_bytes()) for p in sources},
        evidence_sha256=evidence,report_sha256=digest((OUT/'report.md').read_bytes()),
        V3_modifications=0,hard_gates_changed=False,certificate_mathematics_changed=False)
    save('AUTHORITY.json',authority)
    save('HASHES.json',{p:digest((OUT/p).read_bytes()) for p in ('report.md','AUTHORITY.json')})
    print(json.dumps(load(OUT/'HASHES.json')),flush=True)

if __name__=='__main__':seal()

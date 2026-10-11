"""Seal the bounded Task-3C diagnosis; no scientific execution here."""
from pathlib import Path
import sys,json,datetime,shutil,collections
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT/'src'),str(ROOT/'tools')];sys.dont_write_bytecode=True
import numpy as np
from task3c_preflight import git,OUT
from task3c_preservation import verify
from task3c_io import read as read_evidence
from sdf_cfd_mesher_v4.common.authority import file_hash
from sdf_cfd_mesher_v4.forecast.detailed.scientific import plain,scientific_hash

CLASS='V4CapSearchOrPatchModelInsufficient'
NAMES={'component_00':'Positive-span leading cap','component_01':'Negative-span trailing cap','component_02':'Negative-span aft cap','component_04':'Negative-span leading cap'}
def read(n):return read_evidence(OUT/n)
def write(n,v):(OUT/n).write_text(json.dumps(plain(v),indent=2,sort_keys=True,allow_nan=False)+'\n',encoding='utf-8')
def table(headers,rows):return '\n'.join(['| '+' | '.join(headers)+' |','| '+' | '.join(['---']*len(headers))+' |']+['| '+' | '.join(map(str,r))+' |' for r in rows])
def unrelated(s):return sorted(r for r in s.splitlines() if 'research/sdf_cfd_mesher_v4/' not in r)
def numbers(v):
    if isinstance(v,dict) and 'nonfinite' in v:return float(v['nonfinite'])
    if isinstance(v,list):return [numbers(x) for x in v]
    return v
def extrema(v):
    a=np.asarray(numbers(v),float);return dict(min=float(np.nanmin(a)),max=float(np.nanmax(a)))

if __name__=='__main__':
    start=read('PREFLIGHT.json');now=datetime.datetime.now(datetime.timezone.utc)
    assert git('rev-parse','HEAD')==start['starting_head'];assert not git('diff','--cached','--name-only')
    status=git('status','--porcelain');preserved=verify();write('PRESERVATION.json',preserved)
    ar=read('AUDIT.json');cert=read('CERTIFICATION.json');targets=read('TARGET_TRAJECTORIES.json');ablation=read('SEARCH_ABLATION.json');sensitivity=read('SENSITIVITY.json');response=read('FORECAST_RESPONSE.json')
    det=read('DETERMINISM.json');tests=read('TESTS.json');assert det['passed']
    expected={'test_forensics.ForensicTests.test_current_task_preservation_and_sealed_task2','test_setup.SetupTests.test_branch_and_unrelated_git_status_preserved','test_task3.RelaxationTests.test_current_task_unrelated_state_preserved'}
    expected.update({"test_setup.SetupTests.test_fork_hashes_and_bounded_preservation (path='src/sdf/aerospace/naca_spline.rs')",'test_task1.FoundationTests.test_18_v3_preservation','test_task2.DetailedTests.test_original_sources_unchanged','test_task3_repair.CertificateRepairTests.test_followup_preserves_unrelated_state'})
    assert not tests['errors'] and set(tests['failed_tests'])==expected
    scopes=[];summary=[];quality=[];conditioning=[]
    for a in ar:
        name=a['component'];tr=next(t for t in targets if t['component']==name);proposal=next(c for c in cert['proposals'] if name in c['members']);ts=next(t for t in proposal['targets'] if t['component']==name)
        summary.append(dict(component=name,name=NAMES[name],classification='LocalCorrectionBlocked',
            original_cause='Fixed nonpositive wall-corner Jacobians outside active support; old collective search cannot change them.',
            current_layer_valid=proposal['current_layer_valid'],accepted_private_steps=0,
            first_exact_nonpositive_before=tr['first_exact_nonpositive_before'],first_exact_nonpositive_after=tr['first_exact_nonpositive_after'],
            target_future_before=ts['baseline']['first_invalid'],target_future_after=ts['candidate']['first_invalid'],target_removed=ts['candidate']['removed'],
            target_min_before=min(ts['baseline']['tail']),target_min_after=min(ts['candidate']['tail']),
            blocker='Complete changed-side future certificate exceeds the preserved 250000-pair work bound. No complete no-new-future-intersection result.'))
        for s in a['scopes']:
            p=s['patch'];scopes.append(dict(component=name,ring=s['ring'],active=len(p['active']),fixed_boundary=len(p['fixed_boundary']),halo=len(p['halo']),
                columns=len(p['detailed']['columns']),verification_columns=len(p['detailed']['core_columns']),prisms=20*len(p['detailed']['columns']),
                tangent_rank=s['tangent_rank']['independent_rank'],search_rank=s['coverage']['rank'],locked=s['invariant_corner_count'],locked_affected=s['invariant_affected_corner_count']))
    max_disp=0.
    for i,c in enumerate(cert['proposals']):
        d=read(f'PROPOSAL_{i}_DIRECT.json');g=c['candidate_gate'];p=c['patch']
        for key in ['jacobian','determinant','weight','skewness','symmetric_volume_ratio','aspect','volume']:
            quality.append(dict(cap=i,metric=key,**g['extrema'][key],violations=g['violations'][key]))
        conditioning.append(dict(cap=i,current=g['extrema'],future_baseline_sigma=extrema(d['baseline']['cells']['sigma']),future_candidate_sigma=extrema(d['candidate']['cells']['sigma']),
            future_baseline_G=extrema(d['baseline']['cells']['G_bound']),future_candidate_G=extrema(d['candidate']['cells']['G_bound'])))
        scopes.append(dict(component='+'.join(c['members']),ring='completed support',active=len(p['active']),changed=len(c['changed_vertices']),fixed_boundary=len(p['fixed_boundary']),halo=len(p['halo']),
            columns=len(p['detailed']['columns']),verification_columns=len(p['detailed']['core_columns']),prisms=20*len(p['detailed']['columns']),tangent_rank=2*len(p['active']),search_rank='not a full-space search',locked=0,locked_affected=0))
    positive=read('component_00_closed_support.json');pe=next(e for e in positive['history'] if e.get('potential'))
    common=read('COMMON_NEGATIVE_CAP.json');disp_pos=pe['candidate_displacement'];disp_neg=float(np.linalg.norm(np.array(common['projection']['tangential_displacement']),axis=1).max())
    # Final displacement after projection uses preserved initial coordinates.
    from sdf_cfd_mesher_v4.geometry.front import load_sf2_wall
    f=load_sf2_wall()
    with np.load(ROOT/'results/task1/forecast_layers_01_05.npz') as z:base1=z['coordinates'][0]
    max_disp=max(float(np.linalg.norm(np.array(c['layer1_coordinates'])-base1[c['changed_vertices']],axis=1).max()) for c in cert['proposals'])
    timing_files=[p for p in OUT.glob('*TIMINGS.json')];worker_peaks=[]
    def collect(v):
        if isinstance(v,dict):
            if 'peak_rss' in v:worker_peaks.append(v['peak_rss'])
            for x in v.values():collect(x)
        elif isinstance(v,list):
            for x in v:collect(x)
    for p in timing_files:collect(json.loads(p.read_text()))
    elapsed=(now-datetime.datetime.fromisoformat(start['started_utc'])).total_seconds()
    resources=dict(task_elapsed_seconds=elapsed,maximum_recorded_worker_peak_rss=max(worker_peaks),free_disk_bytes=shutil.disk_usage(ROOT).free,
        initial_unbounded_future_attempt='Stopped after exact pair population was measured; no complete proof or scientific clearance was emitted. Its process peak RSS was not captured. Completed worker peaks are reported explicitly.',
        heavy_scope='Local cap arrays and exact side pairs only; no dense global sensitivity, CFD, OpenFOAM or full-shell forecast regeneration.')
    write('RESOURCES.json',resources);write('SCOPES.json',scopes);write('CANDIDATE_QUALITY.json',quality);write('CONDITIONING.json',conditioning)
    write('COMPONENT_RESULTS.json',summary)
    write('PRIVATE_STEERING.json',dict(performed=False,advanced_layers=0,accepted_candidates=0,production_committed_layer=0,
        reason='The old patch obstruction is established before any temporal limitation. Complete future admission evidence remains unclosed. No invalid or incompletely assessed intermediate state is advanced.',
        layer1_only_insufficiency_proven=False,tangent_insufficiency_proven=False,read_only_forecast_origins_tested=[1,2,5]))
    # Account for exact gates without promoting uncertified future comparisons.
    write('SUMMARY.json',dict(classification=CLASS,components=summary,immediate_valid_proposals=2,components_covered=4,
        accepted_private_candidates=0,advanced_layers=0,maximum_valid_current_proposal_displacement=max_disp,
        maximum_active_vertices=max(s['active'] for s in scopes),maximum_columns=max(s['columns'] for s in scopes),maximum_prisms=max(s['prisms'] for s in scopes),
        current_exact_pairs=[c['current_certificate']['completed_pairs'] for c in cert['proposals']],
        future_candidate_pairs=[c['future_certificate']['candidate_pairs'] for c in cert['proposals']],
        future_pair_bound=250000,scientific_hash=det['run1'],task4_ready=False))
    audits=table(['Component','Active vertices: rings 0/1/2/3','Tangent ranks','Collective rank','Locked affected corners'],[
        [NAMES[a['component']],'/'.join(str(len(s['patch']['active'])) for s in a['scopes']),'/'.join(str(s['tangent_rank']['independent_rank']) for s in a['scopes']),'4', '/'.join(str(s['invariant_affected_corner_count']) for s in a['scopes'])] for a in ar])
    nested=table(['Component','Ring','Active','Boundary','Halo','Columns','Prisms','Locked corners'],[[s['component'],s['ring'],s['active'],s['fixed_boundary'],s['halo'],s['columns'],s['prisms'],s['locked']] for s in scopes])
    immediate=table(['Scope','Jacobian min (m^3)','Determinant min','Weight min','Skew max','Volume-ratio min','Aspect max','Exact pairs'],[
        ['Positive cap' if i==0 else 'Shared negative cap',*[c['candidate_gate']['extrema'][k][d] for k,d in [('jacobian','min'),('determinant','min'),('weight','min'),('skewness','max'),('symmetric_volume_ratio','min'),('aspect','max')]],c['current_certificate']['completed_pairs']] for i,c in enumerate(cert['proposals'])])
    trajectories=table(['Component','Exact Jacobian first invalid: before -> after','Target future first invalid: before -> after','Target minimum: before -> after','Final admission class'],[
        [r['name'],f"{r['first_exact_nonpositive_before']} -> {r['first_exact_nonpositive_after']}",f"{r['target_future_before']} -> {r['target_future_after']}",f"{r['target_min_before']} -> {r['target_min_after']}",r['classification']] for r in summary])
    oldpasses=table(['Completed support','Old-field trials','Current numeric passes','Passes without a new earlier numeric failure'],[
        [NAMES[r['component']],len(r['events']),sum(e['candidate_gate']['passed'] for e in r['events']),sum(e['candidate_gate']['passed'] and not e.get('new_earlier_failures',{}) for e in r['events'])] for r in ablation])
    cache=table(['Component','Recomputed entries','Unchanged entries','Identical warm recomputation'],[[NAMES[r['component']],r['cold']['recomputed_entries'],r['cold']['unchanged_entries'],r['warm']['recomputed_entries']] for r in response])
    report=f'''Decision question

Why do the unresolved cap hazards resist current local relaxation, and can bounded multi-step advancing-front steering resolve them without weakening any hard gate?

Classification: **{CLASS}**.

The original patch model is provably infeasible: it fixes vertices whose wall-corner Jacobians are exactly zero in affected cells. A larger cap-rim support produces two proposals covering all four components that pass every immediate hard gate, including complete current-layer exact certification. Full future admission remains incomplete at the retained certificate work bound. No private or production layer is advanced.

## A. Authority verification

Branch: `{start['branch']}`. Starting HEAD: `{start['starting_head']}`. All 332 authority/source/evidence bindings matched at the start. At the end, all active frozen bindings still match; two live production-origin bindings differ. V3 modifications: 0. Sealed prior artifact modifications: 0. The task writes only V4 files.

External work changed `src/sdf/aerospace/naca_spline.rs` and `src/sdf/aerospace/wing.rs` while this study ran. The frozen V4 copies and active SDF/certificate sources remain byte-identical. No new scientific execution consumes those live originals. Their exact before/after hashes are in PRESERVATION.json. The historical original-source preservation audit therefore does not pass; it is not silently waived. Unrelated status grows from {preserved['unrelated_rows_start']} to {preserved['unrelated_rows_end']} rows. The four added spline-CPU paths are preserved and are not staged by this task.

- Certificate-repair report: `36ded8034e8dd0708555ba8b4ca664828f9a6ae202026ab3a5e7751cfea90e14`.
- Certificate-repair authority: `9534605516ca55714a6a31b75da1435e9f62d0c0ff1ba001d7bdb08b72215aee`.
- Task-2B report: `a15d973515eae7bd5fbcff3ee1c90cbb24c065b7cc16b68a0822037292927a55`.
- Task-2B authority: `d0ec913d1410d0be33939fe7d02585f5907fb48dcb7f960de70c357b938e489f`.
- Certificate mathematics integration: `bab10e0d52d724a6d0a525b801e2486ae3fc1af9c0f02fa1030da5a8dcc99b7f`.

PREFLIGHT.json and PRESERVATION.json contain the bounded source checks. The existing binary64 schedule, SDF adapter and copied scientific sources are unchanged.

## B. Preserved failed-search audit

All 240 prior cap trials are reused as evidence, including the positive-cap 60-trial history. They are not rerun. AUDIT.json records each source hash, direction set, radius history, displacement range, duplicate-coordinate count, rejection-gate counts and exact locked corners. Rejected polls halve the radius through 1, 1/2, 1/4, 1/8, 1/16 and 1/32. No accepted-step trust expansion was exercised by these failed histories.

{audits}

The five old collective fields include an exact opposite pair. Their initial pre-projection linear span has rank four. That does not prove the full nonlinear tangent model inadequate. More decisively, fixed zero corners make the old supports infeasible even with arbitrary movement of the active vertices.

## C. Tangent-space coverage and rank

For each wall corner j, the preserved wedge polynomial gives exactly:

`J_j(0) = ((w1-w0) cross (w2-w0)) dot (x_j-w_j)`.

Only that corner's proposed top vertex controls this value. Moving other top vertices cannot repair a fixed zero corner. This is a finite-movement invariance, not only a small-sensitivity argument. The proof uses binary64 coordinates interpreted as exact rationals, with no tolerance or rank cutoff.

The critical cap vertices have nominal span displacement zero while adjacent cap wall triangles require nonzero signed span separation. The authoritative SDF selects a section direction at these rim points. Outward span motion is available in their existing tangent space. The exact corner-control ranks are 387 on the positive cap and 392 on the negative cap. All 1,173 positive-cap and 1,180 negative-cap investigated nonpositive bottom-corner rows have a positive signed result under at least one projected tangent probe. The three negative-cap studies identify the identical 392 critical vertices. SENSITIVITY.json records derivatives, exact values, +/- probes, projection residuals and iteration counts.

This establishes controllability of those corners. It does not prove simultaneous feasibility of every future metric.

## D. Nested patch results

{nested}

Rings 0 and 1 reuse the saved searches. Rings 2 and 3 add 240 new trials with unchanged fields, gates and forecast behavior. None passes immediate gates. Every ring retains fixed nonpositive wall corners, including directly affected cells. Larger radius cannot change those fixed coordinates.

The local dependency walk closes around each cap rim after 172-175 expansions, at 392-397 initial active vertices. The shared negative verification union has 400 allowed vertices, of which 392 move. This remains a cap support, not the complete connected shell. The largest detailed scope is 3,121 columns / 62,420 provisional prisms; it is about 2.94% of SF2 columns. No global sensitivity matrix is allocated.

## E. Per-component mechanism analysis

Measured facts: all four original supports contain exact fixed-corner obstructions. Completing cap support permits immediate valid proposals. The old fields also gain current-layer feasibility when support is completed:

{oldpasses}

This controlled comparison identifies patch support as the primary demonstrated cause. The old rank-four search coverage is limited, but its limitation alone did not cause the zero-corner obstruction. Some old-field proposals improve a target while introducing earlier failures elsewhere; SEARCH_ABLATION.json preserves those counterexamples.

Supported interpretation: cap-rim topology propagates the required support around a closed rim. The topology is unchanged and admits valid next-layer geometry. Tangent-only physical authority is therefore not shown to be structurally inadequate. A wrong SDF authority, stale cache or layer-index defect was not found. Medial-axis distance and new curvature models were not introduced.

## F. Negative-span trailing-cap intersection

The shared negative-cap proposal exactly separates edge 1 at layer 1 from edge 729 at layer 10 using the repaired, frozen production certificate. All current-layer pairs in its complete verification scope pass. This is an exact known-pair result, not a global future intersection-free claim.

A larger outward trial can also separate that pair while failing determinant gates. Pair separation alone never authorizes a candidate. SENSITIVITY.json and COMMON_NEGATIVE_CAP.json retain the exact pair evidence and associated numeric gates.

## G. Zero-thickness/Jacobian cap cases

Both final proposals have positive exact volume and Jacobian for every current verification cell: 2,340 positive-cap cells and 2,334 shared negative-cap cells. No sign tolerance replaces exact arithmetic. All immediate numeric violation counts are zero.

{immediate}

## H. Single-step correction results

{trajectories}

`None` means no event in the tested metric history through layer 20. Target future statistics exclude layer 1 by the preserved comparison API; the exact Jacobian history column includes layer 1. The positive target is interpolation weight; the other targets are Jacobians. Complete sorted local metric and sigma tails are retained in PROPOSAL_0_DIRECT.json and PROPOSAL_1_DIRECT.json, with exact selected-cell trajectories in TARGET_TRAJECTORIES.json.

Across the complete verification scopes, the first numeric future invalidity moves from layer 2 to 18 on the positive cap and from layer 2 to 13 on the negative cap. There are zero new earlier numeric failures under the unchanged per-entity comparison rule. The positive target weight improves from 0.03577078136799656 to 0.03957811542889278 but still fails the existing 0.055 gate at layer 18. The aft cap retains an exact negative Jacobian at layer 13. The largest displacement of a current-valid proposal is {max_disp!r} m. Accepted private-study displacement is 0 because no proposal completes the full future admission check.

## I. Multi-step private steering results

Not performed. Private steps: 0. Accepted private candidates: 0. Production committed layer: 0.

Phases A-C establish an earlier obstruction in patch construction, not an inherent layer-1-only or tangent-space limitation. The larger supports repair current feasibility, but the complete future proof required by the retained Task-3 comparison path does not fit its work bound. An incompletely assessed proposal is not advanced. No invalid intermediate state is used, and no multi-step success or impossibility is claimed.

## J. Forecast response verification

FORECAST_RESPONSE.json verifies exact SDF reprojection replay, normalized gradients, preserved target distances, and equality between full-active and changed-only overlays. Read-only forecasts restarted from provisional layers 1, 2 and 5 match their remaining trajectories exactly. These checks do not accept or advance those fronts. No Task-1 cold whole-front forecast is rerun.

## K. Conditioning effects

Current positive-cap sigma minimum: 0.06797985913188254; maximum available G_bound: 81.17360267527995. Current negative-cap sigma minimum: 0.08037947217238375; maximum available G_bound: 70.1236223818545. CONDITIONING.json includes baseline/candidate whole-local-future extrema, including singular baseline entries. No sigma floor or gain ceiling is added. These diagnostics do not replace any geometric gate.

Across all local layers, positive-cap minimum sigma changes from 0 to 0.06568645499996636 and maximum available G_bound changes from 90.35271487645 to 81.17360267527995. Negative-cap minimum sigma changes from 0 to 0.004254083531504211, while maximum available G_bound increases from 433.17234486131184 to 545.0125182660814. This is an observed conditioning degradation despite improved exact Jacobian signs. Available-gain maxima exclude singular unavailable baseline operators; they are not complete finite bounds on those operators.

## L. Locality/cache behavior

{cache}

The expanded-ring and completed-support polls preserve 288 exact unrelated detailed-cache hits in each original run. Identical warm forecasts require zero recomputed entries. Candidate-only numeric rejection uses the same preserved primitives and is checked for exact equality with the corresponding full lookahead result.

The exact current certificates contain 24,912 and 24,819 pairs. The complete changed-side future enumerations contain 1,180,829 and 1,180,083 pairs. The original `advancing_front/local_certificate.py` caller bounds exact work at `max_pairs=250000`. The expanded study initially called the lower-level time-bounded primitive without that pair guard. Its unfinished future traversal was stopped; it emitted no complete proof and is not counted as clearance. The reproducer and exact pair population are retained. The smallest driver repair now checks the existing pair bound before any exact traversal. An over-budget result reports unknown forbidden/unresolved counts, never zero. Certificate mathematics is unchanged.

The user was asked whether to extend this bound. This report retains the existing bound; it does not assume approval for larger work. All four extended component comparisons therefore remain LocalCorrectionBlocked for admission, despite valid current layers and improved direct future metrics.

## M. Determinism

Scientific replay hash 1: `{det['run1']}`.

Scientific replay hash 2: `{det['run2']}`.

Fresh runs reproduce expanded searches, support-completion probes, old-field ablations, exact sensitivities, coordinates, forecast behavior and current-layer certificate streams. The over-budget future path replays its full candidate enumeration and deterministic fail-closed guard. No completed future proof is claimed. Runtime, RSS and the abandoned partial traversal progress are excluded; scientific values are retained. Compact JSON payload references reconstruct the exact canonical data before hashing.

## N. Tests

{tests['tests']} tests: {tests['tests']-tests['failures']-tests['errors']} passed, {tests['failures']} failed, {tests['errors']} errors. The seven failures are three historical Git-status assertions, three original-source hash checks affected by the concurrent production-origin change, and one additional Git-status assertion affected by the new external spline-CPU paths. No Task-3C numerical or certificate test fails. All failed contracts remain unchanged. New tests cover exact tangent rank, direction coverage, fixed-corner invariance, nested supports, closure equivalence, immediate-gate predicates, rollback, no production commitment, no-new-earlier failures, exact serialization and the certificate pair-work guard. Multi-step advancement/commit tests are not claimed because no multi-step implementation is added.

Measured task elapsed time at sealing: {elapsed:.3f} s. Maximum recorded worker peak RSS: {resources['maximum_recorded_worker_peak_rss']} bytes. The stopped initial future worker's peak was not captured; this is explicitly a recorded-worker maximum, not a reconstructed process statistic. Detailed stage timings and storage figures are in RESOURCES.json and the timing artifacts. No CFD, OpenFOAM, rolling commitment, topology change, new layer schedule or V3 modification occurred.

## O. Classification

**{CLASS}** describes the diagnosed cause of the previous cap resistance. Small active patches leave exact, uncontrollable current failures. Completing cap-rim support establishes immediate feasibility without broader physical DOFs. Limited collective search remains a secondary limitation; it is not evidence of tangent-model impossibility.

The extended future assessment is bounded and incomplete. It does not establish accepted full-future correction authority. No proposal is promoted and no prior sealed conclusion is rewritten.

## P. Architecture recommendation

The evidence supports larger, dependency-complete cap supports. It does not support adding normal-distance freedom, changing topology, weakening gates, or claiming that multi-step steering is necessary. The next bounded study should address complete future-evidence scheduling/cache reuse for these cap supports and then test private steering of the remaining layer-13 Jacobian and layer-18 weight hazards.

Task 4 readiness: **No**. Current-layer feasibility is established, but full future admission remains unclosed and the remaining hazards have not been steered through valid private steps. Do not start production rolling commitment.

The final Git commit contains only the Task-3C files listed in AUTHORITY.json. Its hash is reported in the final task response. SHA256SUMS.json binds this final report and AUTHORITY.json without a circular self-hash.
'''
    (OUT/'report.md').write_text(report,encoding='utf-8')
    sources=[ROOT/'src/sdf_cfd_mesher_v4/advancing_front/cap_diagnostics.py',ROOT/'src/sdf_cfd_mesher_v4/advancing_front/cap_certificate.py',ROOT/'tests/test_cap_diagnostics.py']+sorted((ROOT/'tools').glob('task3c_*.py'))
    evidence=[p for p in OUT.iterdir() if p.is_file() and p.suffix!='.log' and p.name not in ('AUTHORITY.json','SHA256SUMS.json','.gitignore')]
    total=sum(p.stat().st_size for p in ROOT.rglob('*') if p.is_file());new=sum(p.stat().st_size for p in evidence+sources)
    largest=max(evidence+sources,key=lambda p:p.stat().st_size)
    resources.update(v4_total_bytes=total,new_task_bytes=new,largest_new_file=str(largest.relative_to(ROOT)),largest_new_file_bytes=largest.stat().st_size,
        files_above_100_MB=[str(p.relative_to(ROOT)) for p in evidence+sources if p.stat().st_size>100_000_000])
    write('RESOURCES.json',resources)
    authority=dict(task='DiagnoseAndExtendV4CapCorrectionAuthority',classification=CLASS,starting_head=start['starting_head'],branch=start['branch'],
        v3_modification_count=0,committed_layer=0,accepted_private_candidates=0,task4_ready=False,scientific_hash=det['run1'],
        active_frozen_authorities_passed=True,outside_scope_origin_drift=preserved['outside_scope_origin_drift'],
        report_sha256=file_hash(OUT/'report.md'),source_sha256={str(p.relative_to(ROOT)):file_hash(p) for p in sources},
        evidence_sha256={str(p.relative_to(ROOT)):file_hash(p) for p in sorted(evidence)},
        source_scientific_authorities=dict(repair_authority='9534605516ca55714a6a31b75da1435e9f62d0c0ff1ba001d7bdb08b72215aee',forensics_authority='d0ec913d1410d0be33939fe7d02585f5907fb48dcb7f960de70c357b938e489f'),
        future_scope='LocalContinuousEvidenceIncomplete at preserved 250000-pair work bound',method='Exact current-layer certificates; complete direct local future measurements; no global future zero-intersection claim')
    write('AUTHORITY.json',authority);write('SHA256SUMS.json',dict(report_sha256=file_hash(OUT/'report.md'),authority_sha256=file_hash(OUT/'AUTHORITY.json')))
    # Include the final authority and checksum files in the storage accounting.
    # Only byte-count fields can change width; the hashes have fixed length.
    for _ in range(5):
        total=sum(p.stat().st_size for p in ROOT.rglob('*') if p.is_file())
        new=sum(p.stat().st_size for p in sources)+sum(p.stat().st_size for p in OUT.iterdir() if p.is_file() and p.suffix!='.log')
        if resources['v4_total_bytes']==total and resources['new_task_bytes']==new:break
        resources.update(v4_total_bytes=total,new_task_bytes=new)
        write('RESOURCES.json',resources)
        authority['evidence_sha256'][str((OUT/'RESOURCES.json').relative_to(ROOT))]=file_hash(OUT/'RESOURCES.json')
        write('AUTHORITY.json',authority)
        write('SHA256SUMS.json',dict(report_sha256=file_hash(OUT/'report.md'),authority_sha256=file_hash(OUT/'AUTHORITY.json')))
    else:raise AssertionError('StorageAccountingDidNotStabilize')
    print(json.dumps(read('SHA256SUMS.json'),indent=2))

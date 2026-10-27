"""Seal Task-3E measured evidence without changing scientific geometry."""
from task3e_preflight import *
from task3c_io import read as read_evidence
from sdf_cfd_mesher_v4.forecast.detailed.scientific import scientific_hash,plain
import shutil

def read(name):return read_evidence(OUT/name)
def write(name,value):(OUT/name).write_text(json.dumps(plain(value),indent=2,sort_keys=True)+'\n')
def table(headers,rows):
    return '\n'.join(['| '+' | '.join(headers)+' |','| '+' | '.join(['---']*len(headers))+' |']+['| '+' | '.join(map(str,row))+' |' for row in rows])
def maximum_rss(value):
    if isinstance(value,dict):return max([v for k,v in value.items() if k in ('peak_rss','peak_wset') and isinstance(v,(int,float))]+[maximum_rss(v) for v in value.values()]+[0])
    if isinstance(value,list):return max([maximum_rss(v) for v in value]+[0])
    return 0

def run(classification,ready=False):
    start=read('PREFLIGHT.json');preserved=audit()
    assert git('rev-parse','HEAD')==start['starting_head'];assert not git('diff','--cached','--name-only')
    old=start['previous']['prior_frozen_audit'];new=preserved['previous']['prior_frozen_audit']
    original={r['path']:r['actual'] for r in old['rows']}
    changed=[r['path'] for r in new['rows'] if original.get(r['path'])!=r['actual']]
    assert not [p for p in changed if 'sdf_cfd_mesher_v3/' in p or not p.startswith('src/')],changed
    outside=lambda s:sorted(x for x in s.splitlines() if 'research/sdf_cfd_mesher_v4/' not in x)
    before=outside(start['starting_status']);after=outside(git('status','--porcelain'))
    write('PRESERVATION.json',dict(frozen_authorities_passed=True,bounded_origin_changes=changed,v3_modifications_by_task=0,sealed_prior_modifications=0,
        unrelated_status_rows=len(after),unrelated_added=sorted(set(after)-set(before)),unrelated_removed=sorted(set(before)-set(after)),
        preexisting_outside_scope_drift=new['outside_scope_origin_drift']))
    branches=[read(f'BRANCH_{g}.json') for g in (0,1,2,3)];diagnostics=read('DIAGNOSTICS.json')+read('DIAGNOSTICS_REFINED.json');tests=read('TESTS.json')
    assert all(b['authoritative_wall_hash']=='3412cc0b84cc4ac092ad2cd89698cb9857ddb8428f3200db1e6d2e0ef12e7c94' for b in branches)
    assert all(b['topology_hash']=='69477add150348f567e454cb2216a913124adecc6ec11c47f58471c0931868e7' for b in branches)
    assert tests['historical_assertions_unchanged'] and not tests['new_failures'] and tests['errors']==0
    records=[]
    for g in (0,1,2,3):
        a=read(f'BRANCH_{g}_HASH.json');b=read(f'BRANCH_{g}_REPLAY.json');assert a==b,(g,a,b)
        records.append(dict(branch=g,run1=a['scientific_hash'],run2=b['scientific_hash'],passed=True))
    assert read('DIAGNOSTICS_HASH.json')==read('DIAGNOSTICS_REPLAY.json')
    assert read('DIAGNOSTICS_REFINED_HASH.json')==read('DIAGNOSTICS_REFINED_REPLAY.json')
    run1=scientific_hash(dict(branches=[r['run1'] for r in records],diagnostics=[read('DIAGNOSTICS_HASH.json')['scientific_hash'],read('DIAGNOSTICS_REFINED_HASH.json')['scientific_hash']]))
    run2=scientific_hash(dict(branches=[r['run2'] for r in records],diagnostics=[read('DIAGNOSTICS_REPLAY.json')['scientific_hash'],read('DIAGNOSTICS_REFINED_REPLAY.json')['scientific_hash']]))
    write('DETERMINISM.json',dict(run1=run1,run2=run2,passed=run1==run2,branches=records,full_cold_certificate_replay=False,
        replay_mode='Recompute coordinates, metrics and decisions; restore coordinate/ownership/kernel-bound exact proofs and reproduce complete proof streams. Independent cold/cache equality fixture passes.'))
    accepted=[(b['group'],s) for b in branches for s in b['accepted']]
    assert all(s['gate']['passed'] and not any(s['gate']['violations'].values()) and s['certificate']['complete']
        and not any(s['certificate'][k] for k in ('forbidden','unresolved','invalid_surfaces')) for g,s in accepted)
    elapsed=(datetime.datetime.now(datetime.timezone.utc)-datetime.datetime.fromisoformat(start['started_utc'])).total_seconds()
    timings=[read(p.name) for p in OUT.glob('*TIMINGS.json')]
    peak=max(map(maximum_rss,[*timings,tests]));files=[p for p in OUT.rglob('*') if p.is_file() and p.suffix!='.log']
    largest=max(files,key=lambda p:p.stat().st_size)
    resources=dict(task_elapsed_seconds=elapsed,peak_rss=peak,free_disk_bytes=shutil.disk_usage(ROOT).free,
        v4_total_bytes=sum(p.stat().st_size for p in ROOT.rglob('*') if p.is_file()),
        new_study_bytes=sum(p.stat().st_size for p in files),largest_new_file=str(largest.relative_to(ROOT)),largest_new_file_bytes=largest.stat().st_size,
        maximum_active_vertices=max(len(b['patch']['active']) for b in branches),
        maximum_columns=max(len(b['patch']['detailed']['columns']) for b in branches),
        maximum_provisional_prisms=20*max(len(b['patch']['detailed']['columns']) for b in branches),
        no_cold_whole_front_forecast=True,no_global_dense_sensitivity=True,production_committed_layer=0)
    write('RESOURCES.json',resources)
    summary=dict(task='ImplementV4ImmediateLayerCertificationAndPrivateRollingSteering',classification=classification,task4_ready=ready,
        private_layers={str(b['group']):b['private_layers'] for b in branches},accepted_new_transactions={str(b['group']):len(b['accepted']) for b in branches},
        rejected={str(b['group']):b['rejected'] for b in branches},all_accepted_immediate_gates_passed=True,
        all_accepted_immediate_certificates_complete=True,production_committed_layer=0,
        maximum_correction_displacement=max(s['displacement'] for g,s in accepted),scientific_hash=run1,v3_modifications=0)
    summary.update(total_new_accepted_transactions=len(accepted),total_rejected_trials=sum(b['rejected'] for b in branches),
        exact_immediate_pairs=sum(s['certificate']['completed_pairs'] for g,s in accepted),
        maximum_immediate_pairs=max(s['certificate']['completed_pairs'] for g,s in accepted),
        maximum_future_exact_spot_pairs=max((s['known_family']['exact_pairs'] for g,s in accepted if s['known_family']),default=0))
    write('SUMMARY.json',summary)
    labels={0:'Positive nominal control',1:'Negative coarse steering',2:'Positive refined steering',3:'Negative refined continuation'}
    overview=table(['Branch','Private layers','New accepts','Rejected trials','Maximum correction (m)'],[
        [labels[b['group']],b['private_layers'],len(b['accepted']),b['rejected'],b['max_displacement']] for b in branches])
    gates=table(['Branch / layer','Volume min','Jacobian min (binary64 diagnostic)','Determinant min','Weight min','Skew max','SVR min','Aspect max'],[
        [f'{g} / {s["layer"]}',*[s['gate']['extrema'][k][v] for k,v in [('volume','min'),('jacobian','min'),('determinant','min'),('weight','min'),('skewness','max'),('symmetric_volume_ratio','min'),('aspect','max')]]] for g,s in accepted])
    proofs=table(['Branch / layer','Complete exact pairs','Forbidden','Unresolved','Prior layers'],[
        [f'{g} / {s["layer"]}',s['certificate']['completed_pairs'],len(s['certificate']['forbidden']),len(s['certificate']['unresolved']),s['scope']['prior_layers_included']] for g,s in accepted])
    targets=table(['Branch / layer','Component','First future invalidity','Direct future minimum'],[
        [f'{g} / {s["layer"]}',name,t['first_invalid'],t['minimum']] for g,s in accepted for name,t in s['targets'].items()])
    conditioning=table(['Branch / layer','Immediate sigma min','Immediate G_bound max','Remaining sigma min','Remaining G_bound max'],[
        [f'{g} / {s["layer"]}',s['gate']['extrema']['sigma']['min'],s['gate']['extrema']['G_bound']['max'],s['future_sigma_min'],s['future_G_bound_max']] for g,s in accepted])
    all_conditioning=table(['Branch / layer','Same 20-layer population sigma min','Same 20-layer population G_bound max','Unavailable gain cells'],[
        [f'{d["branch"]} / {d["private_layer"]}',d['all_twenty_layer_sigma_min'],d['all_twenty_layer_G_bound_max'],d['unavailable_gain_cells']] for d in diagnostics])
    future=table(['Branch / layer','Broad-phase future candidates','Exact known-family checks','Exact first future nonpositive J'],[
        [f'{d["branch"]} / {d["private_layer"]}',d['conservative_future_candidate_pairs'],d['exact_future_pairs'],d['exact_first_future_nonpositive_jacobian']] for d in diagnostics])
    negative=[(g,s) for g,s in accepted if g in (1,3)]
    intersections=table(['Branch / layer','Exact family pairs','Forbidden future pairs','First future intersection'],[
        [f'{g} / {s["layer"]}',s['known_family']['exact_pairs'],len(s['known_family']['forbidden']),
         'None through 20' if s['known_family'].get('candidate_first_intersection')==21 else s['known_family'].get('candidate_first_intersection')] for g,s in negative])
    locality=table(['Branch','Active','Fixed boundary','Halo vertices','Verification columns','Closed columns','Forecast cache hits/misses','Detailed hits/misses'],[
        [b['group'],len(b['patch']['active']),len(b['patch']['fixed_boundary']),len(b['patch']['halo']),len(b['patch']['detailed']['core_columns']),len(b['patch']['detailed']['columns']),
         f'{b["forecast_cache_hits"]}/{b["forecast_cache_misses"]}',f'{b["detailed_cache_hits"]}/{b["detailed_cache_misses"]}'] for b in branches])
    runtimes=table(['Branch','Original seconds','Replay seconds'],[[g,read(f'BRANCH_{g}_TIMINGS.json')['total_seconds'],read(f'BRANCH_{g}_REPLAY_TIMINGS.json')['total_seconds']] for g in (0,1,2,3)])
    step_performance=[]
    for g,s in accepted:
        t=next(t for t in read(f'BRANCH_{g}_TIMINGS.json')['stages'] if t['layer']==s['layer'] and t['trial']==s['trial'])
        f=t['forecast'];c=t['certificate']
        step_performance.append([f'{g} / {s["layer"]}',s['displacement'],f['recomputed_entries'],f.get('unchanged_entries',0),
            f['seconds'],t['detailed_seconds'],c.get('seconds'),t['peak_rss']])
    step_performance=table(['Branch / layer','Correction (m)','Cheap entries recomputed','Unchanged entries','Forecast seconds','Detailed seconds','Exact seconds','Peak RSS bytes'],step_performance)
    distances=table(['Branch / layer','Actual inter-front step min (m)','Actual inter-front step max (m)','Maximum target residual (m)'],[
        [f'{d["branch"]} / {d["private_layer"]}',d['actual_step_min'],d['actual_step_max'],d['maximum_sdf_residual']] for d in diagnostics])
    component_names={'component_00':'Positive-span leading cap','component_01':'Negative-span trailing cap','component_02':'Negative-span aft cap','component_04':'Negative-span leading cap'}
    primary=[(0,branches[0]['accepted'][0]),*( (2,s) for s in branches[2]['accepted']),
             *((1,s) for s in branches[1]['accepted'][:2]),*((3,s) for s in branches[3]['accepted'])]
    primary_table=table(['Branch / layer','Target','First target invalidity','Target minimum','First any numeric future failure'],[
        [f'{g} / {s["layer"]}',component_names[name],t['first_invalid'],t['minimum'],s['next_future_failure']]
        for g,s in primary for name,t in s['targets'].items()])
    final_positive=branches[2]['accepted'][-1];final_negative=branches[3]['accepted'][-1]
    negative_diagnostic=next(d for d in diagnostics if d['branch']==3 and d['private_layer']==final_negative['layer'])
    positive_diagnostic=next(d for d in diagnostics if d['branch']==2 and d['private_layer']==final_positive['layer'])
    failed='\n'.join('- `'+x+'`' for x in tests['failed_tests'])
    report=f'''# Decision question

Can V4 advance through multiple fully valid, exactly certified immediate local layers while using future geometry only as diagnostic lookahead to steer away from later cap hazards?

**{classification}**

## A. Authority verification

Branch `{start['branch']}`. Starting HEAD `{start['starting_head']}`.

- Task-3D report: `76eb72328eb8123a1d02fbddb9383d409493bf37dd8d717ac53f4fb659819cad`.
- Task-3D authority: `c73a57e416bec64165aa76a69702350a8ac1b1a7977ef187428495e66068dd1a`.
- Task-3C report: `680f0d49d5867244aab17c01971ccf5fa146b2651419837b953b091e177023ef`.
- Task-3C authority: `5ed30f0be4d02c5b972dbb74a37ad9a4829d3d9c92bc5c6061c238c815de2651`.
- Frozen certificate mathematics integration: `bab10e0d52d724a6d0a525b801e2486ae3fc1af9c0f02fa1030da5a8dcc99b7f`.
- Common geometry: `f453b7430703e9ae2c4938cd10f600c565d3fb7a0ce6f8e22e1161561326d1a3`.
- SF2 surface: `dc390bedc2198cb315e15961f848cd67b47641c30ae4ec0b96944996d1f8196f`.
- Unchanged wall coordinates: `3412cc0b84cc4ac092ad2cd89698cb9857ddb8428f3200db1e6d2e0ef12e7c94`.
- Unchanged topology: `69477add150348f567e454cb2216a913124adecc6ec11c47f58471c0931868e7`.

All active frozen source and sealed-result bindings pass before and after work. V3 modifications by this task: **0**. Sealed prior artifacts changed: **0**. The known live production-origin differences remain outside the active frozen V4 source. PRESERVATION.json records the exact unrelated Git-state differences; no unrelated file is staged.

## B. Corrected certification semantics

`advancing_front/private_rolling.py` implements private state, layer-relative gating, rebased local forecasts and the corrected admission decision. `POLICY.md` gives the scope. The qualified exact kernel and hard numeric definitions are unchanged.

Immediate validation checks all verification cells and internal sides. From layer 2 onward it also checks the shared cap face that gains its second cell. Exact rational signs check every verification prism. Continuous proof includes every new verification side against all conservatively enumerated new and prior side geometry. Prior/prior geometry cannot change.

Future metric failures remain explicit diagnostics. They do not automatically invalidate an otherwise valid current layer. New earlier entity failures reject trials; direct target trajectories must be maintained or improved. There is no combined score or new sigma/gain gate. The policy does not require a complete future million-pair exact certificate. No authoritative V4 front is advanced.

## C. Saved layer-1 proposal replay

Both saved Task-3C coordinate arrays, original pre-projection SDF evaluations, immediate numeric values, and exact Jacobian records reproduce. Their local forecast coordinate hashes match Task-3D. Both immediate exact certificates complete. The positive steering experiment forks the already certified layer-1 state without repeating its seed proof during original execution.

{overview}

Branch 0 is a no-additional-motion positive-cap control. Branch 1 polls the preserved span-tangent mode on the shared negative support. Branch 2 polls that mode from the same positive layer-1 seed. After coarse negative proposals damage the verification ring, Branch 3 continues from the already certified negative layer 2 with smaller, previously untested amplitudes. The positive poll also includes these smaller amplitudes. Factors multiply preserved increments; repeated halving reaches 0.03125. These are computational search bounds, not physical thresholds. Histories retain every rejection. No identical failed coarse trial is rerun to generate the refined original history.

## D. Positive-cap private steering

The layer-1 correction changes the target minimum future weight from 0.03577078136799656 to 0.03957811542889278, with first weight failure at layer 18. The tables below show every later measured change. The nominal control distinguishes repeated SDF rebasing from new tangent correction. No farther future coordinate is edited directly.

The refined positive path reaches private layer {branches[2]['private_layers']}. Its final first predicted weight failure is layer {final_positive['targets']['component_00']['first_invalid']}, with minimum {final_positive['targets']['component_00']['minimum']}. The unchanged nominal control still predicts layer 18. Repeated valid decisions therefore change the downstream weight trajectory; simple rebasing alone does not.

## E. Negative-cap private steering

One dependency-complete support covers the negative leading, aft and trailing components. The saved seed delays the aft exact-Jacobian failure from layer 1 to layer 13 and separates the known trailing-cap pair. The following histories test subsequent decisions on that same bounded support.

The accepted layer-2 correction delays the predicted aft failure from 13 to 17. At the next decision, the +0.5-increment trial delays it to 20 and passes current numeric gates, but creates four new weight violations at layer 20 on verification-ring edges 28997, 29750, 77767 and 132949. It is rejected as collateral damage, not described as an earlier global prefix failure. The +1 trial also fails current determinant, weight and skewness gates. These are direct counterexamples to judging a trial only by its target Jacobian. An empty certificate on a pre-rejected trial means exact work was intentionally not run; it is not an unresolved exact-kernel failure.

The smaller accepted layer-3 correction delays the aft nonpositive Jacobian to layer {final_negative['targets']['component_02']['first_invalid']}. The final private prefix reaches layer {branches[3]['private_layers']}. Its earliest remaining numeric forecast failure is layer {final_negative['next_future_failure']}; this includes determinant evidence and must not be confused with the target Jacobian's first failure. The negative leading and trailing targets remain free of nonpositive Jacobians through layer 20 in this forecast.

## F. Immediate hard-gate results

Every admitted private transaction passes all hard gates. All violation counts are zero. Positive exact rational Jacobian and volume signs are required separately from the diagnostic binary64 extrema shown here. Frozen ownership and topology remain valid; local closure is unchanged.

{gates}

Correction displacement is measured from the nominal proposed next front. The actual inter-front step also includes the requested outward advance. Every residual below satisfies the unchanged native source-defined arithmetic bound; no new distance tolerance is introduced.

{distances}

## G. Immediate exact certification

Every accepted proof is complete, with zero forbidden intersections, unresolved results or invalid side surfaces. Proof streams and exact geometry identities are persisted for each accepted transaction. The certificate is the qualified prism-side model; this report does not invent a separate side-cap theorem.

{proofs}

## H. Future hazard trajectories

Complete local cell and face arrays, cheap edge/triangle descriptors and per-target histories are stored in the accepted-step artifacts. Layer numbers remain global schedule indices. A cap-face metric is attributed to the layer at which its upper cell first exists. Exact selected-column Jacobian histories supplement the direct numeric diagnostics.

The forecast stays uncompressed. Existing edge-length ratios, separation changes, relative motion along edges, triangle-area ratios and actual versus nominal vertex step lengths remain available as direct contraction/separation descriptors. No compression engine or new compression model is run.

{targets}

The two primary histories reuse their certified seeds. This table distinguishes target mechanisms from the earliest numeric failure anywhere in the verification region:

{primary_table}

## I. Exact future spot checks

The two known trailing edges are screened across all 20 by 20 layer combinations. Overlapping broad-phase pairs and the original edge-1/layer-1 versus edge-729/layer-10 pair receive exact checks. A new earlier family intersection rejects a trial. Other future broad-phase candidates remain warnings, not separation proofs. Full provisional future certification is not performed.

{intersections}

{future}

## J. Conditioning

These are direct diagnostics. No sigma floor or gain ceiling is applied. Complete local arrays remain available to inspect distributions and collateral effects.

{conditioning}

Remaining-layer populations shrink after each admission. A changed extremum in
that population alone does not prove geometric improvement. The positive
nominal control retains exactly the same full 20-layer coordinate hash at all
four private steps. Compare the fixed population below when assessing actual
conditioning effects.

{all_conditioning}

At the final positive step, the fixed 20-layer population has sigma minimum {positive_diagnostic['all_twenty_layer_sigma_min']} and maximum G_bound {positive_diagnostic['all_twenty_layer_G_bound_max']}. The final negative path has sigma minimum {negative_diagnostic['all_twenty_layer_sigma_min']} and maximum G_bound {negative_diagnostic['all_twenty_layer_G_bound_max']}. The negative seed had 0.004254083531504211 and 545.0125182660814 respectively. The negative future reconstruction tail therefore needs continued attention even where the geometric failure is delayed. These diagnostics do not reject an otherwise valid immediate candidate.

## K. Locality and cache behavior

Task-1 arrays are restored once per worker. Each candidate recomputes only local trajectories and dependent edge/triangle descriptors. An unrelated detailed control remains an exact cache hit on every evaluated proposal. Geometry keys prevent stale detailed reuse. DIAGNOSTICS.json verifies an identical local warm call for every accepted step and exact reproduction of its future coordinates.

{locality}

{step_performance}

There are {summary['total_new_accepted_transactions']} newly certified accepted transactions across the control, coarse and refined histories, and {summary['total_rejected_trials']} rejected trials. Shared prefix references are not counted as new transactions. Exact current proofs cover {summary['exact_immediate_pairs']} pairs in total; the largest immediate transaction has {summary['maximum_immediate_pairs']} pairs. The largest selective future check uses {summary['maximum_future_exact_spot_pairs']} pairs, not the million-plus provisional population.

Maximum scope: {resources['maximum_active_vertices']} active vertices, {resources['maximum_columns']} closed local columns and {resources['maximum_provisional_prisms']} provisional prisms. The broad phase reads source-edge bounds; exact work remains local. No whole-shell sensitivity matrix or per-trial cold whole-front forecast is allocated. Fixed halo coordinates close local dependencies and do not constitute acceptance of the whole wing.

## L. Runtime and memory

{runtimes}

Task elapsed at sealing: {elapsed} s. Maximum measured worker RSS: {peak} bytes. Largest new file: `{resources['largest_new_file']}` ({resources['largest_new_file_bytes']} bytes). V4 bytes at measurement: {resources['v4_total_bytes']}. Free disk: {resources['free_disk_bytes']} bytes. Per-trial forecast, detailed and certificate runtimes are separate in the timing artifacts. No CFD or OpenFOAM solver is run.

## M. Determinism

All complete private histories replay from their preserved inputs. Coordinates, numeric metrics, admission sequences, exact proof bytes, diagnostic arrays and final private states reproduce. Replay restores the qualified exact proof cache only after verifying identical coordinate bits, ownership, topology, kernel identity, conservative pair enumeration and proof-stream hashes. The complete streams are reproduced byte for byte. A focused fixture independently compares restored proofs with fresh cold kernel execution. This is cache-aware replay, not a second cold execution of every exact pair; the shared seed is referenced explicitly.

- Run 1: `{run1}`.
- Run 2: `{run2}`.

Only timing, RSS and operational metadata are outside scientific hashes.

## N. Tests

{tests['tests']} tests; {tests['tests']-tests['failures']-tests['errors']-tests['skipped']} passed; {tests['failures']} historical failures; {tests['errors']} errors. New scientific failures: **0**. The preserved unrelated-state/origin assertions remain unchanged:

{failed}

## O. Classification

**{classification}**. SUMMARY.json records per-branch accepted and rejected transactions. This is a bounded private study, not full-shell qualification. The source wall remains the only authoritative committed front at layer 0. No branch reaches layer 20, so no finished-stack certificate is claimed or forced.

## P. Task-4 readiness

Production Task-4 implementation readiness: **{ready}**. Both primary paths have four individually valid, exactly certified private layers. Repeated positive correction delays the weight failure; repeated negative correction delays the aft Jacobian failure while maintaining exact separation of the known trailing pair. Dependency-complete support stays bounded, rejected trials roll back exactly, and full histories reproduce. These observations support implementing the bounded rolling transaction with the same mandatory gates. They do not qualify a completed production shell.

The remaining positive weight failure and negative determinant/Jacobian failures must stay active forecast warnings. Negative future conditioning worsens and must remain visible. The conservative per-entity future-failure rule can reject a proposal even when the global earliest failure improves; this study records those ring effects rather than claiming that the bounded poll exhausts all valid trades. The four-layer computational horizon does not prove behavior at the later hazards. No unexamined future pair is called separated. Task 4 is not started, and authoritative committed geometry stays at layer 0.

SHA256SUMS.txt binds this report and AUTHORITY.json. The Git commit hash is reported after sealing to avoid a self-referential artifact hash. Only the V4 Task-3E path list is committed.
'''
    (OUT/'report.md').write_text(report,encoding='utf-8')
    sources=[ROOT/'src/sdf_cfd_mesher_v4/advancing_front/private_rolling.py',ROOT/'tests/test_private_rolling.py',*sorted((ROOT/'tools').glob('task3e_*.py'))]
    evidence=[p for p in sorted(OUT.iterdir()) if p.is_file() and p.suffix!='.log' and p.name not in ('AUTHORITY.json','SHA256SUMS.txt')]
    authority=dict(task=summary['task'],classification=classification,starting_head=start['starting_head'],summary=summary,
        source_sha256={p.relative_to(ROOT).as_posix():file_hash(p) for p in sources},
        evidence_sha256={p.relative_to(ROOT).as_posix():file_hash(p) for p in evidence},report_sha256=file_hash(OUT/'report.md'))
    write('AUTHORITY.json',authority)
    (OUT/'SHA256SUMS.txt').write_text(''.join(file_hash(OUT/name)+'  '+name+'\n' for name in ('report.md','AUTHORITY.json')))
    print(json.dumps(summary,indent=2));print((OUT/'SHA256SUMS.txt').read_text())

if __name__=='__main__':
    import argparse
    p=argparse.ArgumentParser();p.add_argument('--classification',required=True);p.add_argument('--ready',action='store_true');a=p.parse_args();run(a.classification,a.ready)

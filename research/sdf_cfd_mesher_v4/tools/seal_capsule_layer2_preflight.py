"""Seal the non-generative layer-2 integration audit; never publish a front."""
import json
import shutil
import subprocess
from capsule_layer2_preflight import OUT, ROOT, V4, L1, save, file_hash, identity


def load(name): return json.loads((OUT/name).read_text())


def main():
    a=load('RUN_A.json');b=load('RUN_B.json');s=a['scientific']
    assert s==b['scientific'] and not s['layer2_authorized']
    start=load('GIT_START.json');disk=load('DISK_START.json');tests=load('TESTS.json')
    status=subprocess.check_output(['git','status','--porcelain=v1'],cwd=ROOT,text=True).splitlines()
    owned=['research/sdf_cfd_mesher_v4/tools/capsule_layer2_preflight.py',
           'research/sdf_cfd_mesher_v4/tools/seal_capsule_layer2_preflight.py',
           'research/sdf_cfd_mesher_v4/tests/test_capsule_layer2_preflight.py',
           'research/sdf_cfd_mesher_v4/results/capsule_advancing_front_layer2/']
    unrelated=[r for r in status if not any(p in r for p in owned)]
    assert unrelated==start['unrelated_entries']
    bindings=load('INPUT_SOURCE_BINDINGS.json')
    for r in bindings['authority_checks']+bindings['sources']:
        assert file_hash(ROOT/r['path'])==r['sha256']
    old=json.loads((L1/'AUTHORITY.json').read_text())
    d=load('L1_RETAINED_DIAGNOSTICS.json')['data']
    classification='V4ProductionAdaptiveCapsuleAdvancingFrontLayer2BlockedForecastIntegration'
    claim='The sealed production layer-1 checkpoint restores exactly and the unchanged request decoder supplies h2, but the current production provider has no admitted layer-2 forecast/advance path. No layer-2 geometry was generated or committed.'
    next_task='WireAndQualifyV4ProductionLayer2ForecastAdmission'
    blockers=[
        'ProductionFirstLayerProvider.propose rejects advancement with ExactlyOneLayerScope.',
        'Its schedule and committed forecast dependency descriptors contain only the first increment; all 33786 remaining_schedule lists are empty.',
        'Existing generic cheap/detailed forecast components have no qualified binding from this restored production checkpoint and full admitted request to a layer-2 decision.',
        'The older AnalyticProvider inherits fixture-wall construction and historical schedule loading. Using it unchanged would bypass production provenance.']
    save(OUT/'DECISION.json',dict(classification=classification,claim=claim,blockers=blockers,
        scope='Integration preflight; not geometric infeasibility',generation_authorized=False,
        cheap_forecast='Not run: no applicable production binding',detailed_lookahead='Not run',
        risk_patches='NotMeasured',collision='NotMeasured',compression='NotMeasured',
        forecast_cache_hits='NotMeasured',forecast_cache_misses='NotMeasured',invalidations='NotMeasured',
        future_increments_inspected=[],second_increment_decoded=s['request'],
        generated=s['generated'],committed_layer=1,next_task=next_task))
    save(OUT/'GIT_PRESERVATION.json',dict(starting_head=start['head'],branch=start['branch'],
        starting_unrelated_entries=len(start['unrelated_entries']),unrelated_entries_now=len(unrelated),
        exact_status_equality=True,v3_task_modifications=0,owned_paths=owned,
        note='Final commit and post-commit status are reported in the terminal; no self-referential commit hash is embedded.'))
    save(OUT/'IMPLEMENTATION.json',dict(files=[dict(path=p,sha256=file_hash(ROOT/p)) for p in owned[:3]],
        scope='Read-only restore/preflight evidence tooling and tests; no production geometry or forecast implementation changed'))
    # Snapshot excludes this sealing write and subsequent Git object writes.
    resources=dict(repository_accessible_logical_bytes=disk['repository_accessible_logical_bytes'],
        v4_accessible_logical_bytes=disk['v4_accessible_logical_bytes'],starting_free_bytes=disk['free_bytes'],
        ending_free_bytes_at_seal=shutil.disk_usage(ROOT).free,bytes_deleted=0,deleted_paths=[],
        cleanup_reason='No disk pressure. Existing manifest-referenced duplicates retained.',
        retained_l1_bytes=disk['retained_l1_bytes'],disk_audit_seconds=disk['seconds'],
        inaccessible_disk_entries=len(disk['errors']),task_wall_time='NotMeasured',
        peak_restore_rss_bytes=max(a['peak_rss_bytes'],b['peak_rss_bytes']),
        run_a_times=a['times'],run_b_times=b['times'],
        retained_rss_a=a['retained_rss_bytes'],retained_rss_b=b['retained_rss_bytes'])
    save(OUT/'RESOURCES.json',resources)
    lines=['## Decision question','',
      'Can the first production advance from the qualified committed capsule layer 1 exercise the existing forecast/admission path and qualify exactly layer 2?','']
    def section(title,text): lines.extend(['### '+title,'',text,''])
    def table(rows,header):
        return '\n'.join(['| '+' | '.join(header)+' |','| '+' | '.join(['---']*len(header))+' |']+
                         ['| '+' | '.join(map(str,r))+' |' for r in rows])
    q=['min','p1','p5','median','p95','p99','max']
    def distributions(rows): return table([(name,*[value[k] for k in q]) for name,value in rows],['Metric',*q])
    section('A. Classification',f'**{classification}**\n\n{claim}\n\nLayer 2 committed: NO. Layer 3 generated/committed: NO. No claim of layer-2 geometry failure, complete AdvancingFrontQualified, CoreHandoffQualified, CoreQualified, or CompleteMeshQualified.')
    ids={**s['wall_identities'],**{k:v for k,v in old['scientific'].items()},
         'first_layer_identity':old['first_layer_identity'],'restored_committed_state':s['restored_state'],
         'request_file_sha256':s['request_file_sha256'],'schedule_identity':s['request']['schedule_identity'],
         'physical_profile':'82a721c13b59c37f6ce20ee7744a1a228a10f469aacac22d244669d61a68b5fb',
         'progression':'d7b3057844118279cf21b4c987f3bdc7030411553fe7431b8ab6bed4f359dfd6'}
    section('B. Input and restore authorities',f"Starting HEAD: `{start['head']}`. Both independent native restores passed. The unchanged read_request decoder admitted provenance; this is not geometric advancement authorization.\n\n"+table(ids.items(),['Identity','SHA-256 / scientific identity'])+
        '\n\nPrior report and authority hashes were checked against the user-supplied values. Wall arrays and authority were reloaded after the preflight and remained exact; mutation count 0. Native checkpoint restoration checks its component hashes, receipt identities, and complete scientific state hash.')
    section('C. Advancing-front decision',
      'Requested layer 2; increment 0.00017333832030214487 m; binary64 `0x1.6b844d56c388ep-13`; cumulative target 0.0003182297719802118 m.\n\n'+
      '\n'.join('- '+r for r in blockers)+
      '\n\nSource trace: `production_first_layer_run.py` lines 37, 45–48, 88–92; `mixed_front.py::rebuild_dependencies` lines 205–226; `mixed_front_runner.py::_step` lines 109–117; `mixed_front_fixture_provider.py::Provider.__init__` lines 50–57; `forecast/engine.py::ForecastCache` lines 62–85.\n\n'+
      'The mixed runner delegates proposal/evidence to the provider. Its dependency descriptors are not forecasts. ForecastCache and DetailedCache exist as reusable machinery, but their presence does not establish production admission. No fallback fixture provider was instantiated.\n\n'+
      'Cheap forecast: not run. Detailed lookahead: not run. Risk count/IDs, collision indicators, compression indicators, cache hits/misses/invalidations: NotMeasured. No future increments inspected or provisional future fronts constructed. Full schedule decoding checked all request bytes; no future geometry was evaluated.')
    section('D. Forecast versus realized','All predicted/realized layer-2 height, determinant, weight, skewness, volume ratio, aspect, volume/Jacobian, edge/triangle, collision and correction comparisons: NotMeasured. No error thresholds were added. The rejection is an integration stop, not a forecasted geometric failure.')
    section('E. Counts and ownership',table(s['counts'].items(),['Restored quantity','Count'])+
      '\n\nNew L2 front vertices/edges/triangles/prisms: 0/0/0/0. Existing cells remain 33786; no two-layer complex exists. Source remapping: none. L2 missing/duplicate/orphan/foreign mapping checks: NotMeasured. Retained L1 source mappings are 16895 vertices, 50679 edges, 33786 triangles; its sealed missing/duplicate/orphan/foreign counts are all 0.')
    section('F. Direction evolution','L2 advancement directions and all angular distributions against L1 normal, SDF gradient, and wall direction: NotMeasured. L2 zero/nonfinite/inward/fallback counts and correction changes: NotMeasured.')
    section('G. Height realization','Requested increment is exact as stated in C. All L2 Euclidean, construction-normal, SDF-normal, tangential, requested-minus-achieved and achieved/requested distributions and failure counts: NotMeasured. No raw, provisional or final L2 front exists.')
    frontrows=[('wall edge m',d['edges']['wall']),('L1 edge m',d['edges']['front']),('L1/wall edge ratio',d['edges']['ratio']),('L1-wall edge change m',d['edges']['change'])]
    frontrows += [(label+' '+key,d[group][key]) for label,group in [('wall','wall_triangles'),('L1','front_triangles')] for key in ['min_angle','max_angle','edge_ratio','radius_ratio','area']]
    frontrows += [('L1/wall area ratio',d['area_ratio'])]
    section('H. Front quality evolution','The following are retained, hash-verified L1-task diagnostics, not new L2 measurements. Angles are degrees; areas are m².\n\n'+distributions(frontrows)+
      '\n\nL2 distributions, L2/L1 and L2/wall edge/area ratios and changes: NotMeasured. Retained L1 zero-area and orientation-reversal counts: 0. L2 zero-area/reversal/nonfinite counts: NotMeasured.')
    section('I. Geometry diagnostics','Retained L1-task diagnostics only:\n\n'+distributions(list(d['geometry_diagnostics'].items()))+
      '\n\nL2 curvature, normal variation, height/tangential-scale, height×curvature and their front-to-front changes: NotMeasured.')
    section('J. Prism geometry','Retained L1 evidence:\n\n'+distributions([(k,d['quality'][k]['distribution']) for k in ('volume','exact_jacobian')])+
      '\n\nBoth retained nonpositive counts are 0; worst volume and exact-Jacobian cell 1027110 (source column 27110). L2 and combined-context geometry, nonpositive counts and worst sources: NotMeasured.')
    section('K. Hard quality','Retained L1 evidence only; no new or combined-context gates evaluated:\n\n'+
      distributions([(k,d['quality'][k]['distribution']) for k in ('determinant','weight','skewness','symmetric_volume_ratio','aspect')])+'\n\n'+
      table([(k,r['gate'],r['failures'],r['margin'],r['worst'].get('id',r['worst'].get('owners'))) for k,r in d['quality'].items() if k in ('determinant','weight','skewness','symmetric_volume_ratio','aspect')],['Metric','Gate','Retained failures','Margin','Worst cell(s)'])+
      '\n\nL2/combined distributions, failure counts, worst IDs and margins: NotMeasured. Positive volume, exact Jacobian, topology, ownership and complete continuous geometry remain required. No gates changed.')
    section('L. Determinant context','L1 retains FoundationAreaTensorFirstPrismIncidence/1: actual lateral topology plus declared internal top-cap role; minimum 0.48900452281001455. Actual L2-neighbour-completed L1 determinant: NotMeasured. L2 terminal determinant: NotMeasured. Context-only difference: NotMeasured. No neighbours were fabricated. Retired exposed-cap semantics did not control acceptance.')
    section('M. Continuous geometry','No two-layer certificate was attempted. L2/combined unique faces, candidate/certified pairs, proof populations, forbidden/unresolved/missing/duplicate counts: NotMeasured. They are not reported as zero. Preserved L1 certificate: 118251 unique faces, 1940126 certified required pairs, forbidden/unresolved/missing/duplicate 0 under its prior authority. Its native certificate payload restored with the sealed state.')
    section('N. Topology and ownership','Restored state contains 33790 vertices, 118251 faces and 33786 L1 prisms. Prior L1 evidence: 50679 internal faces, 67572 boundary faces; front Euler characteristic 2; one front component; no missing/duplicate/orphan/foreign mappings. Full L2/combined topology, nonmanifold/duplicate/unused/orientation/component/ownership failure counts: NotMeasured. Layer 1 and wall were not changed.')
    section('O. Sigma and G','Retained L1 diagnostics:\n\n'+distributions([(k,d['conditioning'][k]) for k in ('sigma','G_raw','G_cons','G_bound','G_weighted_raw','G_weighted_cons','G_weighted_bound')])+
      '\n\nL2 and combined-context distributions/comparisons: NotMeasured. No hard sigma floor, G ceiling or percent-improvement gate.')
    section('P. Correction and relaxation','Not used. No geometric candidate or measured raw geometric failure exists. Risk patches, iteration metrics, changed-entity quality effects and final recertification: NotMeasured. No existing repair path was bypassed or new one invented.')
    section('Q. Commit and transaction',f"No transaction or publication attempted. Authoritative layer remains 1, committed identity `{s['restored_state']}`. CURRENT.json SHA-256 remains `{s['pointer_sha256']}`. New L2 transaction/journal/checkpoint identity: NotMeasured. No recovery transaction was pending. L3 generation and commit counts 0.")
    section('R. Deterministic replay',f"Independent Python-process native restore, decoder and preflight results are exactly equal. Preflight scientific identity `{a['scientific_identity']}`. Restored L1 coordinate/topology and full state identities are printed in B. L2 forecast/coordinates/topology/connectivity, combined connectivity, ownership/quality/certificate and committed L2 hashes: NotMeasured. This is preflight replay, not L2 scientific replay.")
    section('S. Performance',table([(run,k,v) for run,r in [('A',a),('B',b)] for k,v in r['times'].items()],['Run','Phase','Seconds'])+
      f"\n\nPeak restore RSS: {resources['peak_restore_rss_bytes']} bytes. Retained RSS A/B: {resources['retained_rss_a']} / {resources['retained_rss_b']} bytes. Disk audit {disk['seconds']} seconds. Total task wall time: NotMeasured. Forecast, detailed lookahead, risk work, construction, correction, combined quality, certification, commit and their caches: NotMeasured; not executed. No native solver launched.")
    section('T. Tests',f"{tests['run']} run; {tests['passed']} passed; {tests['failed']} failed; {tests['skipped']} skipped. Exact failures: none. Pytest reported 2.80 seconds.\n\n`test_capsule_layer2_preflight.py`, `test_production_first_layer_run.py`, and `test_production_prism_sizing.py`. Tests use sealed L1 evidence and non-generative guards. New L2/combined numerical and certificate tests were not run because that path was not reached.")
    section('U. Disk',f"Repository accessible logical use {disk['repository_accessible_logical_bytes']} bytes; V4 {disk['v4_accessible_logical_bytes']} bytes; retained L1 {disk['retained_l1_bytes']} bytes. Starting free {disk['free_bytes']} bytes; at-seal free {resources['ending_free_bytes_at_seal']} bytes. {len(disk['errors'])} inaccessible entries recorded, not guessed. Managed worktrees, nested dual_contouring checkout and symlink directories excluded.\n\nPrior manifest identifies 30 duplicate groups and 3585149193 candidate redundant bytes. No files deleted: 0 bytes. Both referenced replay/checkpoint copies remain intact. No checkpoint/proof streams copied into this task. Final artifact inventory records task-created retained bytes; final post-commit free space is reported in the terminal.")
    section('V. Hashes and artifacts','INPUT_SOURCE_BINDINGS.json binds current source and prior report/authority/request files. L1_RETAINED_DIAGNOSTICS.json binds historical diagnostics. RUN_A/RUN_B, REPLAY, DECISION, TESTS and storage records preserve the new evidence. HASHES.json contains report/authority seals. No L2 scientific, forecast or combined two-layer identity was created.')
    section('W. Git state',f"Branch `{start['branch']}`; starting HEAD `{start['head']}`. Unrelated dirty entries before/after audit: 155/155, exact status list preserved. V3 task modifications: 0. Task changes: read-only preflight tool, evidence sealing tool, focused preflight tests and fresh results directory. Production algorithms and historical reports unchanged. Only task-owned V4 paths are staged; final commit and post-commit status reported in the terminal.")
    section('X. Blockers','Classification: production forecast/admission integration.\n\n'+'\n'.join('- '+r for r in blockers)+
      '\n\nRestore failure: none. Input-authority failure: none. Request blocker: none. Construction, quality, continuous geometry, topology, ownership and L2 determinism: untested, not failed. Complete AdvancingFrontQualified remains not established.')
    section('Y. Exact supported claim',claim)
    section('Z. Recommended next task',f'**{next_task}**: bind a read-only view of the restored production front and full admitted request to the existing cheap forecast, risk/detailed lookahead and explicit advancement decision. Qualify its authority, scope and replay before attempting L2 geometry. Preserve wall/L1; do not generate L3.')
    (OUT/'report.md').write_text('\n'.join(lines),encoding='utf-8')
    files={p.name:file_hash(p) for p in OUT.iterdir() if p.is_file() and p.name not in ('AUTHORITY.json','HASHES.json','ARTIFACT_INVENTORY.json')}
    authority=dict(classification=classification,supported_claim=claim,restored_layer1=s['restored_state'],
      first_layer_qualified=True,layer2_qualified=False,complete_advancing_front_qualified=False,
      layer2_generated=False,layer2_committed=False,layer3_generated=False,core_generated=False,
      input_surface=s['wall_identities']['surface_identity'],request=s['request_identity'],
      preflight_identity=a['scientific_identity'],source_binding_identity=bindings['identity'],
      blockers=blockers,files=files)
    save(OUT/'AUTHORITY.json',authority)
    save(OUT/'HASHES.json',dict(report_sha256=file_hash(OUT/'report.md'),authority_sha256=file_hash(OUT/'AUTHORITY.json'),
                              preflight_identity=a['scientific_identity']))
    inventory=[dict(path=p.relative_to(ROOT).as_posix(),bytes=p.stat().st_size,sha256=file_hash(p)) for p in sorted(OUT.iterdir()) if p.is_file() and p.name!='ARTIFACT_INVENTORY.json']
    inventory += [dict(path=p,bytes=(ROOT/p).stat().st_size,sha256=file_hash(ROOT/p)) for p in owned[:3]]
    save(OUT/'ARTIFACT_INVENTORY.json',dict(files=inventory,retained_task_bytes_excluding_this_inventory=sum(r['bytes'] for r in inventory),deleted_bytes=0))
    print(json.dumps(load('HASHES.json')))


if __name__=='__main__': main()

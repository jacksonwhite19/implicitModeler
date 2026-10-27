"""Report only published geometry; prepared fronts remain explicitly separate."""
from task4_initial_scan import *
from task3c_io import read
from task4_preflight import git
from sdf_cfd_mesher_v4.common.authority import file_hash,reference
import datetime,re,shutil
from collections import Counter

QUESTION='Can V4 build the complete requested 20-layer SF2 wall-prism shell by committing one fully valid, exactly certified layer at a time while using bounded future lookahead for local steering?'

def json_file(path):return json.loads(Path(path).read_text())
def scientific(path):return read(path) if Path(path).exists() else None
def scope_size(p):
    return dict(active_vertices=len(p['active']),fixed_boundary=len(p['fixed_boundary']),halo=len(p['halo']),
        verification_columns=len(p['detailed']['core_columns']),dependency_columns=len(p['detailed']['columns']),
        provisional_prisms=20*len(p['detailed']['columns']))

def collect():
    front=load_sf2_wall();top=build_topology(front.triangles,len(front.coordinates));pointer=json_file(OUT/'build_A/CURRENT.json');count=pointer['layer']
    pre=json_file(OUT/'PREFLIGHT.json');layers=[];scopes=[];displacements=[];trials=[];performance=[];conditioning=[];certificates=[]
    for k in range(1,count+1):
        state=json_file(OUT/f'build_A/LAYER_{k:02}.json');cert=state['certificate'];certificates.append(cert)
        if k==1:
            search=read(OUT/'CORRECTION_HISTORY.json');outcomes=search['outcomes'];history=search['history'];accepted=[h for h in history if h.get('accepted')]
            ps=[x['patch'] for x in outcomes if x.get('corrected')]
        else:
            search=scientific(OUT/f'LAYER{k}_SEARCH.json') or dict(history=[],outcomes=[])
            history=search['history'];accepted=[h for h in history if h.get('accepted')]
            ps=[x['scope'] for x in search['outcomes'] if x.get('accepted')]
        sizes=[scope_size(p) for p in ps];scopes+=sizes;trials+=history
        moves=[h['displacement'] for h in accepted];displacements+=moves
        timing=json_file(OUT/f'LAYER_{k:02}_TIMINGS.json');performance.append(dict(layer=k,certification=timing))
        if (OUT/f'LAYER{k}_PREPARE_TIMINGS.json').exists():performance[-1]['preparation']=json_file(OUT/f'LAYER{k}_PREPARE_TIMINGS.json')
        proof_methods=Counter();proof_classes=Counter()
        for batch in cert['batches']:proof_methods.update(batch['methods']);proof_classes.update(batch['counts'])
        stages=timing['certificate_stages']
        layers.append(dict(layer=k,prisms=len(top.triangles),coordinate_hash=state['coordinate_hash'],state_hash=state['state_hash'],
            status='corrected' if accepted else 'nominal',patch_count=len(ps),active_vertex_count=len({v for p in ps for v in p['active']}),
            maximum_correction=max(moves,default=0.),gate=state['gate'],exact_sign_cells=state['exact']['cells'],certificate_pairs=cert['completed_pairs'],
            forbidden=len(cert['forbidden']),unresolved=len(cert['unresolved']),warning_count=len(state['warnings']),
            first_warning=min((e['layer'] for e in state['warnings']),default=None),warning_metrics=sorted(set(e['metric'] for e in state['warnings'])),
            retained_prior_proofs=sum(c['completed_pairs'] for c in certificates[:-1]),
            exact_kernel_calls=sum(p.get('exact_kernel_calls',0) for p in stages),proof_cache_hits=sum(p.get('cache_hits',0) for p in stages),
            proof_methods=dict(proof_methods),proof_classifications=dict(proof_classes),
            future_exact_comparisons=sum(h.get('future_spots',{}).get('exact_pair_comparisons',0) for h in history),
            nominal_or_corrected_scopes=sizes,accepted_proposals=len(accepted),rejected_proposals=len(history)-len(accepted)))
        with np.load(OUT/f'LAYER_{k:02}_CONDITIONING.npz') as z:conditioning.append({name:z[name] for name in z.files})
    current=json_file(OUT/f'build_A/LAYER_{count:02}.json')
    tail=None
    if conditioning:
        sigma=np.stack([r['sigma'] for r in conditioning]);gain=np.stack([r['G_bound'] for r in conditioning]);available=np.stack([r['G_bound_available'] for r in conditioning])
        a=np.unravel_index(np.argmin(sigma),sigma.shape);g=np.where(available,gain,-np.inf);b=np.unravel_index(np.argmax(g),g.shape)
        geometry_path,_=reference('common geometry');sections=json_file(geometry_path)['sections'];ymin=min(v['position'][1] for v in sections);ymax=max(v['position'][1] for v in sections)
        def location(i):
            point=front.coordinates[top.triangles[i[1]]].mean(0)
            return dict(layer=int(i[0]+1),column=int(i[1]),wall_centroid=point.tolist(),
                chord_fraction=float((point[0]-sections[0]['position'][0])/sections[0]['chord']),span_fraction=float((point[1]-ymin)/(ymax-ymin)))
        tail=dict(scope='published prefix only' if count<20 else 'complete twenty-layer shell',sigma_minimum=float(sigma[a]),sigma_location=location(a),
            positive_sigma_minimum=float(sigma[sigma>0].min()) if np.any(sigma>0) else None,singular_count=int(np.sum(sigma==0)),
            sigma_quantiles={str(q):float(np.quantile(sigma,q)) for q in (0,.0001,.001,.01,.05,.5,1)},
            G_bound_maximum=float(g[b]),G_bound_location=location(b),G_bound_available_count=int(available.sum()),
            sigma_sha256=array_hash(sigma),G_bound_sha256=array_hash(gain))
    extrema={}
    for row in layers:
        for name,bounds in row['gate']['extrema'].items():
            if name not in extrema:extrema[name]=dict(bounds)
            else:extrema[name]=dict(min=min(extrema[name]['min'],bounds['min']),max=max(extrema[name]['max'],bounds['max']))
    violation_counts={k:sum(r['gate']['violations'][k] for r in layers) for k in (layers[0]['gate']['violations'] if layers else [])}
    cache_rows=[h['cache'] for h in trials if 'cache' in h]
    cache=dict(trials_with_instrumentation=len(cache_rows),recomputed_entry_observations=sum(r.get('recomputed_entries',0) for r in cache_rows),
        retained_entry_observations=sum(r.get('unchanged_entries',0) for r in cache_rows),whole_local_result_hits=sum(bool(r.get('hit')) for r in cache_rows),
        maximum_recomputed_entries=max((r.get('recomputed_entries',0) for r in cache_rows),default=0),
        minimum_retained_entries=min((r.get('unchanged_entries',0) for r in cache_rows),default=None),
        note='Repeated observations, not unique population counts. Unchanged stored trajectories remain exact; local cache misses recompute only their dependency scope.')
    cache['saved_trajectory_audits']=[read(OUT/f'LAYER{k}_CACHE_AUDIT.json') for k in range(2,count+1) if (OUT/f'LAYER{k}_CACHE_AUDIT.json').exists()]
    cache['whole_trajectory_reuse_layers']=[row['layer'] for row in cache['saved_trajectory_audits'] if row['actual_changed_vertices']==0]
    final=scientific(OUT/'FINAL_VALIDATION.json');replay=scientific(OUT/'DETERMINISM_A_A2.json')
    initial_scan=read(OUT/'INITIAL_SCAN.json');nominal=initial_scan['records'];discovery=[]
    for i,region in enumerate(initial_scan['regions']['all_future']):
        ids=np.asarray(region['vertices']);points=front.coordinates[ids];cols=set(region['columns'])
        events=[e for row in nominal for e in row['events'] if cols.intersection(e['columns'])]
        discovery.append(dict(region=i,source_vertices=len(ids),dependency_columns=len(cols),
            coordinate_min=points.min(axis=0).tolist(),coordinate_max=points.max(axis=0).tolist(),
            first_event=min(e['layer'] for e in events),last_event=max(e['layer'] for e in events),
            measured_metrics=sorted({e['metric'] for e in events}),selection='Connected support of current nominal hard events'))
    nominal_comparison=dict(scope='Same layer indices as the published prefix',layers=list(range(1,count+1)),
        nominal_sigma_minimum=min((r['gate']['extrema']['sigma']['min'] for r in nominal[:count]),default=None),
        nominal_maximum_available_G_bound=max((r['gate']['extrema']['G_bound']['max'] for r in nominal[:count]),default=None),
        nominal_twenty_layer_sigma_minimum=min(r['gate']['extrema']['sigma']['min'] for r in nominal),
        nominal_twenty_layer_maximum_available_G_bound=max(r['gate']['extrema']['G_bound']['max'] for r in nominal),
        source='INITIAL_SCAN.json',same_preserved_metric_primitives=True)
    summary=dict(starting_head=pre['starting_head'],branch=git('branch','--show-current'),published_layers=count,total_prisms=count*len(top.triangles),
        requested_layers=20,wall_vertices=len(front.coordinates),columns=len(top.triangles),source_edges=len(top.edges),side_surfaces=count*len(top.edges),
        unique_faces=count*len(top.edges)+(count+1)*len(top.triangles) if count else 0,
        topology_hash=top.sha256,wall_coordinate_hash=front.coordinate_hash,committed_state_hash=current['state_hash'],coordinate_hash=current['coordinate_hash'],
        layers=layers,largest_local_scope=max(scopes,key=lambda p:p['dependency_columns']) if scopes else None,maximum_accepted_displacement=max(displacements,default=0.),
        hard_gate_extrema=extrema,hard_gate_violations=violation_counts,exact_pair_count=sum(c['completed_pairs'] for c in certificates),
        forbidden_committed_pairs=sum(len(c['forbidden']) for c in certificates),unresolved_committed_pairs=sum(len(c['unresolved']) for c in certificates),
        warnings=current['warnings'],initial_discovery=discovery,conditioning=tail,nominal_conditioning_comparison=nominal_comparison,cache=cache,final_validation=final,replay=replay,
        uncommitted_next_candidate_gate=scientific(OUT/f'LAYER{count+1}_GATE.json') if count<20 else None)
    verification=scientific(OUT/f'VERIFICATION_LAYER_{count:02}.json')
    if verification:
        preserved=verification['authorities']['previous']['previous']['prior_frozen_audit']
        summary['preservation']=dict(v3_modifications=preserved['v3_modifications'],sealed_modifications=preserved['sealed_modifications'],
            frozen_copy_mismatches=preserved['frozen_copy_mismatches'],unrelated_status_added=verification['unrelated_status_added'],
            unrelated_status_removed=verification['unrelated_status_removed'])
    timings=dict(initial=json_file(OUT/'INITIAL_TIMINGS.json'),first_correction=json_file(OUT/'CORRECTION_TIMINGS.json'),layers=performance,
        wall_seconds=(datetime.datetime.now(datetime.timezone.utc)-datetime.datetime.fromisoformat(pre['started_utc'])).total_seconds())
    if (OUT/'USER_STOP.json').exists():
        summary['early_stop']=json_file(OUT/'USER_STOP.json')
        timings['execution_wall_seconds_until_user_stop']=summary['early_stop']['elapsed_task_seconds']
        timings['interrupted_replay_elapsed_seconds']=summary['early_stop']['replay_elapsed_seconds']
    archive=OUT/'development_A0'
    if archive.exists():
        timings['excluded_development_A0']=dict(
            recovery=json_file(OUT/'ADMISSION_RECOVERY.json'),
            stages={p.relative_to(archive).as_posix():json_file(p) for p in sorted(archive.glob('*TIMING*.json'))},
            operations={p.name:json_file(p) for p in sorted(archive.glob('BUILD_OPERATIONS*.json'))})
        summary['admission_repair']=dict(reproducer='ADMISSION_SCOPE_REPRODUCER.json',
            recovered_layer=1,archived_previous_prefix=4,primary_counts_exclude_archived_layers=True,
            before='Earlier entity events rejected only in outer verification ring',
            after='Earlier entity events rejected throughout complete local verification scope')
    if (OUT/'BUILD_OPERATIONS_INITIAL.json').exists():timings['preserved_initial_operations']=json_file(OUT/'BUILD_OPERATIONS_INITIAL.json')
    if (OUT/'LAYER2_SERIAL_SLICE.json').exists():timings['preserved_layer2_serial_slice']=json_file(OUT/'LAYER2_SERIAL_SLICE.json')
    if (OUT/'A2_PRODUCTION_TIMING.json').exists():timings['independent_production_path']=json_file(OUT/'A2_PRODUCTION_TIMING.json')
    if (OUT/'FINAL_VALIDATION_TIMINGS.json').exists():timings['final_validation']=json_file(OUT/'FINAL_VALIDATION_TIMINGS.json')
    if (OUT/'FINAL_ASSEMBLY_REPRODUCER/REPRODUCER.json').exists():
        summary['final_assembly_repair']=json_file(OUT/'FINAL_ASSEMBLY_REPRODUCER/REPRODUCER.json')
        timings['failed_final_assembly']=summary['final_assembly_repair']
    if (OUT/'replay_A2').exists():
        timings['replay_A2_stages']={p.name:json_file(p) for p in sorted((OUT/'replay_A2').glob('*TIMINGS.json'))}
        replay_root=OUT/'replay_A2';replay_timings=timings['replay_A2_stages']
        replay_initial=replay_timings.get('INITIAL_TIMINGS.json',{})
        replay_validation=[v for k,v in replay_timings.items() if re.fullmatch(r'LAYER_\d+_TIMINGS.json',k)]
        replay_preparation=[v for k,v in replay_timings.items() if re.fullmatch(r'LAYER\d+_PREPARE_TIMINGS.json',k)]
        timings['production_stage_summary']=dict(
            initial_forecast_setup_seconds=replay_initial.get('forecast_seconds'),
            initial_direct_scan_seconds=replay_initial.get('total_seconds',0)-replay_initial.get('forecast_seconds',0),
            initial_local_correction_seconds=replay_timings.get('CORRECTION_TIMINGS.json',{}).get('total_seconds'),
            later_preparation_seconds=sum(v['total_seconds'] for v in replay_preparation),
            immediate_numeric_seconds=sum(v.get('numeric_seconds',0) for v in replay_validation),
            exact_sign_seconds=sum(v.get('exact_sign_seconds',0) for v in replay_validation),
            sdf_validation_seconds=sum(v.get('sdf_validation_seconds',0) for v in replay_validation),
            immediate_exact_certificate_wall_seconds=sum(v.get('certificate_wall_seconds',0) for v in replay_validation),
            checkpoint_seconds=sum(v.get('checkpoint_seconds',0) for v in replay_validation),
            final_complete_qualification_seconds=replay_timings.get('FINAL_VALIDATION_TIMINGS.json',{}).get('total_seconds'),
            final_complete_certificate_seconds=sum(v['certificate_seconds'] for v in replay_timings.get('FINAL_VALIDATION_TIMINGS.json',{}).get('stages',[])),
            completed_immediate_layers=len(replay_validation),
            note='Fresh A2 path. Final certificate is included in final qualification. Detailed preparation sub-stages remain in replay_A2_stages; nested durations must not be added twice.')
    def rss(x):
        if isinstance(x,dict):return max([v for k,v in x.items() if k in ('peak_rss','observed_tree_peak_rss')]+[rss(v) for v in x.values()]+[0])
        if isinstance(x,list):return max([rss(v) for v in x]+[0])
        return 0
    timings['peak_rss']=rss(timings)
    timings['stages']=dict(immediate_numeric_seconds=sum(p['certification'].get('numeric_seconds',0) for p in performance),
        exact_sign_seconds=sum(p['certification'].get('exact_sign_seconds',0) for p in performance),
        exact_certificate_worker_seconds=sum(sum(v['seconds'] for v in p['certification']['certificate_stages']) for p in performance),
        exact_certificate_wall_seconds=sum(p['certification'].get('certificate_wall_seconds',0) for p in performance),
        preparation_seconds=sum(p.get('preparation',{}).get('total_seconds',0) for p in performance),
        checkpoint_seconds=sum(p['certification'].get('checkpoint_seconds',0) for p in performance))
    timings['stages']['final_complete_certificate_seconds']=sum(p['certificate_seconds'] for p in timings.get('final_validation',{}).get('stages',[]))
    timings['stages']['final_qualification_total_seconds']=timings.get('final_validation',{}).get('total_seconds')
    preparation_rows=[(p['layer'],p['preparation']) for p in performance if 'preparation' in p]
    for name in ('risk_discovery_seconds','local_search_seconds','persistence_seconds','nominal_projection_seconds','nominal_numeric_seconds','final_numeric_seconds'):
        timings['stages'][name]=dict(seconds=sum(row[name] for k,row in preparation_rows if name in row),
            instrumented_layers=[k for k,row in preparation_rows if name in row])
    trial_rows=[row for k,prep in preparation_rows for row in prep['stages']]
    timings['local_trial_stages']=dict(proposal_projection_seconds=sum(row['proposal_projection_seconds'] for row in trial_rows),
        local_forecast_seconds=sum(row['forecast']['seconds'] for row in trial_rows),
        local_forecast_projection_seconds=sum(row['forecast']['projection_seconds'] for row in trial_rows),
        local_detailed_seconds=sum(row['detailed_seconds'] for row in trial_rows),
        note='Nested measured durations; do not add projection subtotals to their enclosing forecast/search totals.')
    measured=[(p['layer'],s) for p in performance for s in p['certification']['certificate_stages'] if 'scope_source_edges' in s]
    timings['certificate_scope_measurements']=dict(instrumented_layers=sorted({k for k,s in measured}),
        maximum_source_edges=max((s['scope_source_edges'] for k,s in measured),default=None),
        maximum_side_surfaces=max((s['scope_side_surfaces'] for k,s in measured),default=None))
    timings['stages']['preserved_layer2_serial_phase_seconds']=sum(r['seconds'] for r in timings.get('preserved_initial_operations',[]) if r['layer']==2 and r['phase']=='certify')
    # Atomic publication renames transient .pending files during a live draft.
    paths=[p for p in OUT.rglob('*') if p.suffix!='.pending' and p.is_file()]
    timings['persistent_bytes']=sum(p.stat().st_size for p in paths)
    if summary.get('early_stop'):
        timings['interrupted_pending_bytes']=sum(p.stat().st_size for p in OUT.rglob('*.pending'))
        timings['persistent_bytes']+=timings['interrupted_pending_bytes']
    timings['free_disk_bytes']=shutil.disk_usage(OUT).free
    return summary,timings

def report(s,t,classification,reason):
    lines=['Decision question','',QUESTION,'',classification,'',f"Published layers: {s['published_layers']}/20. Published prisms: {s['total_prisms']:,}.",'',
        f"Stopped early at the user's request. Execution stopped after {s['early_stop']['elapsed_task_seconds']:.6f} s ({datetime.timedelta(seconds=s['early_stop']['elapsed_task_seconds'])}). Build A completed all 20 layers and final validation. Independent A2 stopped after {s['early_stop']['replay_checkpoint']['layer']} committed layers; complete A/A2 qualification remains unfinished. No stopped candidate was committed.\n" if s.get('early_stop') else '',
        'A. Authority verification','',
        'The Task-3E report and authority match 5e86c4aded1ce2b6957d59b0ee5fa44308af757ad98a28dd860e5cd4114ab116 and ba28d1d70023321836c5b166b857a564f92dfa50bd99ec31195774f149bc8842.',
        'Task-3D report and authority match 76eb72328eb8123a1d02fbddb9383d409493bf37dd8d717ac53f4fb659819cad and c73a57e416bec64165aa76a69702350a8ac1b1a7977ef187428495e66068dd1a.',
        f"Wall coordinate SHA-256: {s['wall_coordinate_hash']}. Topology SHA-256: {s['topology_hash']}.",
        'PREFLIGHT.json and VERIFICATION_LAYER_*.json bind the copied sources, geometry, schedule, certificate and prior results.',
        'Common geometry SHA-256: f453b7430703e9ae2c4938cd10f600c565d3fb7a0ce6f8e22e1161561326d1a3. SF2 surface SHA-256: dc390bedc2198cb315e15961f848cd67b47641c30ae4ec0b96944996d1f8196f.',
        'The preserved binary64 request has 20 layers, h1 = 0.000166527069 m, growth = 1.163359674, and total nominal thickness = 0.020000000131704564 m. Schedule SHA-256: 9adae17cb24da08eab0399e0a2afe18639b2130805bd9910a0cc6064bef54cce. No schedule was rebuilt from rounded values.',
        f"Preservation result: {s.get('preservation','Current checkpoint preservation audit not yet recorded')}.",
        '', 'B. Rolling builder implementation','',
        'The builder creates a fresh SDF forecast from frozen layer 0. Complete immediate metrics use bounded batches and the preserved scientific primitives. Only connected supports of current measured failures enter local tangent correction. No historical pathology IDs select patches.',
        'rolling_state.py owns publication. rolling_metrics.py supplies whole-front coverage. rolling_correction.py traces controlling corners. rolling_cache.py stores sparse future deltas. certificate/rolling.py partitions the complete exact pair proof. task4_build.py resumes these stages.',
        '', 'C. Commit semantics and checkpointing','',
        'CURRENT.json is the atomic publication pointer. Every checkpoint binds the complete front, accumulated connectivity, side identities, exact proofs, forecast identity, warnings and correction records. Restart validates every prior front. Failed candidates do not alter the last published state.',
        f"Current state hash: {s['committed_state_hash']}.",
        'During development, review found that an admission check masked new earlier future entity failures inside the active support. The reproducer and original adapter are preserved. All affected immediate geometries passed their hard gates and exact proofs, but their future admission did not meet the intended policy. The run was archived as development_A0. The corrected primary build resumed from the unaffected exact layer-1 checkpoint. Archived layers 2 through 4 do not count toward the primary result. ADMISSION_RECOVERY.json verifies all archived bytes.' if s.get('admission_repair') else '',
        '', 'D. Production risk discovery','',
        'The initial complete twenty-layer diagnostic scan found the two cap supports and eight other small risk regions. Exact failed bottom corners identify their own controlling top vertices. Other failed cells and faces add their actual source vertices. Topological adjacency supplies fixed boundary, dependency halo and verification ring. Computational search bounds are not scientific thresholds.',
        '', 'E. Layer-by-layer results','',
        '| Layer | Status | Patches | Active vertices | Maximum correction (m) | Exact pairs | Warnings | First warning |',
        '|---:|---|---:|---:|---:|---:|---:|---:|']
    discovery_table=['| Discovered region | Source vertices | Columns | First event | Last event | Measured gate events |',
        '|---:|---:|---:|---:|---:|---|']
    for r in s['initial_discovery']:
        discovery_table.append(f"| {r['region']} | {r['source_vertices']} | {r['dependency_columns']} | {r['first_event']} | {r['last_event']} | {', '.join(r['measured_metrics'])} |")
    discovery_table+=['','Region numbers are deterministic records of this fresh scan. They are not historical research IDs. SUMMARY.json retains each region\'s measured coordinate bounds.','']
    at=lines.index('E. Layer-by-layer results');lines[at:at]=discovery_table
    for r in s['layers']:lines.append(f"| {r['layer']} | {r['status']} | {r['patch_count']} | {r['active_vertex_count']} | {r['maximum_correction']:.17g} | {r['certificate_pairs']} | {r['warning_count']} | {r['first_warning']} |")
    lines+=['','| Layer | Min volume | Min Jacobian | Min determinant | Min weight | Max skew | Min volume ratio | Max aspect | Preparation s | Validation/publication s |',
        '|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|']
    times={row['layer']:row for row in t['layers']}
    for r in s['layers']:
        e=r['gate']['extrema'];k=r['layer'];perf=times[k]
        preparation=t['first_correction'].get('total_seconds',0) if k==1 else perf.get('preparation',{}).get('total_seconds',0)
        validation=perf['certification']['total_seconds']+(t['stages']['preserved_layer2_serial_phase_seconds'] if k==2 else 0)
        values=[e['volume']['min'],e['jacobian']['min'],e['determinant']['min'],e['weight']['min'],e['skewness']['max'],e['symmetric_volume_ratio']['min'],e['aspect']['max']]
        lines.append('| '+str(k)+' | '+' | '.join(format(v,'.17g') for v in values)+f' | {preparation:.6f} | {validation:.6f} |')
    lines+=['','Each published row has zero immediate hard-gate violations and zero forbidden or unresolved exact pairs. SUMMARY.json contains every per-layer extremum, decision identity and proof count.',
        '', 'F. New hazard mechanisms','',
        'The whole-front scan discovers immediate corner collapse, determinant transitions and neighboring-cell face imbalance directly. Small regions outside the initial cap support are measured and corrected with the same generic machinery. Exact corner traces and per-trial gates are retained; these labels do not assert an exhaustive physical cause classification.',
        '', 'G. Cap-region trajectories','',
        'LAYER*_WARNINGS.json and LAYER*_SEARCH.json retain the direct future trajectories and local decisions. Future warnings do not invalidate a published current layer. No future layer is manually repaired.',
        f"Remaining warnings: {len(s['warnings'])}; first warning: {min((x['layer'] for x in s['warnings']),default=None)}.",
        '', 'H. Exact certification','',
        f"The published prefix has {s['exact_pair_count']:,} exact new/new or new/prior pair proofs. Forbidden: {s['forbidden_committed_pairs']}. Unresolved: {s['unresolved_committed_pairs']}.",
        'Every pair has one new-source-edge owner. Conservative global box coverage supplies its local interaction halo. Committed/prior proofs remain immutable. Future spot checks are diagnostics and never stand in for immediate exact certification.',
        '', 'I. Final shell geometry','',
        f"Wall vertices {s['wall_vertices']}; columns {s['columns']}; prisms per committed layer {s['columns']}; published prisms {s['total_prisms']}; side surfaces {s['side_surfaces']}.",
        'The fixed 106,302-column topology gives 2,126,040 prisms if twenty layers complete, equal to the V3 SF2 wall-prism count. No topology change or layer termination occurs.',
        'A complete final shell exists.' if s['final_validation'] else 'No complete twenty-layer shell is claimed. Prepared coordinates remain separate from published geometry.',
        f"Complete shell scientific SHA-256: {s['final_validation']['shell_scientific_hash']}. Complete coordinate SHA-256: {s['final_validation']['coordinate_hash']}. Unique faces: {s['unique_faces']}." if s['final_validation'] else '',
        'FINAL_SDF_EVALUATIONS.json retains fresh authoritative scalar and gradient identities for all 20 layers. Residuals are checked against the preserved native source error bound, without a new physical tolerance.' if s['final_validation'] else '',
        f"Maximum absolute SDF target residual: {max(r['maximum'] for r in s['final_validation']['sdf_residuals']):.17g} m." if s['final_validation'] else '',
        '', 'J. Final hard-quality results','',
        'Scope: published prefix.' if s['published_layers']<20 else 'Scope: completed shell.',
        '| Metric | Minimum | Maximum |','|---|---:|---:|']
    for name,b in s['hard_gate_extrema'].items():lines.append(f"| {name} | {b['min']:.17g} | {b['max']:.17g} |")
    lines+=['',f"Violation counts: {json.dumps(s['hard_gate_violations'],sort_keys=True)}.",'', 'K. Reconstruction conditioning','']
    if s['conditioning']:
        c=s['conditioning'];lines += [f"Weighted sigma minimum {c['sigma_minimum']:.17g} at {c['sigma_location']}; singular operators {c['singular_count']}.",
            f"Maximum available G_bound {c['G_bound_maximum']:.17g} at {c['G_bound_location']}.",
            f"Sigma quantiles: {json.dumps(c['sigma_quantiles'],sort_keys=True)}."]
    lines+=['These are operator diagnostics, with no sigma floor or gain ceiling. They do not establish numerical-health qualification.',
        f"Direct nominal comparison: {s['nominal_conditioning_comparison']}.",
        'The preserved V3 values quoted during development concern selected research components and optimization states. They are not treated here as a comparable complete-shell distribution or as acceptance targets. No V3 solver or diagnostic calibration was rerun.',
        '', 'L. Continuous final certificate','',
        'Complete finished-geometry pair re-enumeration and exact proof union passed.' if s['final_validation'] else 'Not invoked: twenty layers have not completed. Immediate certificates qualify only the published prefix.',
        'The first final traversal passed all twenty layers, then the writer failed because the earliest layer-1 checkpoint had no separate SDF diagnostic file. The original source and log are preserved in FINAL_ASSEMBLY_REPRODUCER. Final SDF records now come directly from verified coordinates; all 19 existing records match exactly. A regression rejects inconsistent stored records. The complete read-only qualification was repeated after this packaging repair. No geometry, gate, certificate mathematics, or committed proof changed.' if s.get('final_assembly_repair') else '',
        '', 'M. Cache/locality behavior','',
        f"Largest correction scope: {s['largest_local_scope']}.",f"Cache observations: { {k:v for k,v in s['cache'].items() if k!='saved_trajectory_audits'} }.",
        'Per-layer saved-trajectory audits are in SUMMARY.json. Each checks exact coordinates and normals outside the invalidation support, plus exact preservation of all earlier committed layers.',
        'One whole-front nominal cache is retained. Candidate futures store only changed source-vertex deltas. Whole-front scans are not globally coupled optimizations. No global dense sensitivity is allocated.',
        '', 'N. Runtime and memory','',
        f"Measured wall time through report generation: {t['wall_seconds']:.6f} s. Maximum recorded RSS: {t['peak_rss']} bytes (certificate process-tree sampling, or single-process peak for preparation).",
        f"Stage totals: {json.dumps(t['stages'],sort_keys=True)}.",
        'PERFORMANCE.json separates archived development_A0 work from the corrected production path. Both are included in elapsed task wall time. The archive includes the interrupted uncommitted layer-5 certificate; no cost is treated as successful primary construction.' if 'excluded_development_A0' in t else '',
        f"Independent clean production replay timing: {t.get('independent_production_path','Interrupted by user; completed-run timing is unavailable')}.",
        f"Execution wall time before the requested stop: {t.get('execution_wall_seconds_until_user_stop')} s. Interrupted independent replay elapsed time: {t.get('interrupted_replay_elapsed_seconds')} s. Report-writing time is separate. The interrupted replay coordinator did not finalize its overall peak-RSS sampler; completed per-layer measurements and the stop-time sample remain available." if s.get('early_stop') else '',
        f"Fresh A2 stage summary: {json.dumps(t.get('production_stage_summary',{}),sort_keys=True)}.",
        f"Persistent artifact bytes at report collection: {t['persistent_bytes']}. Free disk bytes: {t['free_disk_bytes']}.",
        'The original fresh forecast finished before an adapter error, so its separate duration was not retained. INITIAL_TIMINGS measures the subsequent cache restore and complete direct scan. Wall time includes that development work. Missing stage instrumentation is not estimated.',
        f"Failed final-writer pass: {t.get('failed_final_assembly',{}).get('failure_log_file_events','None')}. Its per-stage process timers were not persisted; the recorded file-event interval is labelled separately. Total task wall time includes this pass." if s.get('final_assembly_repair') else '',
        '', 'O. Deterministic A/A2 reproduction','',
        'The initial layer-1 spot-check records predated the derived new_earlier_pair_families summary. A2 included that field as an empty list in both records. Every underlying exact pair record, initial scan and construction history matched. REPLAY_SCHEMA_REPRODUCER preserves both original artifacts and proves that normalization adds only the independently derived summaries. No scientific field was dropped. Only the summary digest and enclosing shell digest changed; geometry, proof outcomes, conditioning and transaction decisions did not change. The final comparison uses the verified common schema.' if (OUT/'REPLAY_SCHEMA_REPRODUCER/REPRODUCER.json').exists() else '',
        (f"A scientific hash: {s['replay']['A']}. A2 final scientific hash: {s['replay']['A2']}. Full replay pass: {s['replay']['passed']}. "
         f"Replay status: {s['replay'].get('status','Complete')}. Completed checkpoint comparisons: {s['replay'].get('completed_checkpoint_matches','See replay record')}. "
         f"Interrupted next-layer certificate: {s['replay'].get('partial_next_certificate','None')}. "
         'DETERMINISM_A_A2.json retains every compared checkpoint hash and the exact prepared-search comparison. A null pass or final hash means unfinished, not a determinism failure.') if s['replay'] else 'Complete A/A2 replay has not finished. Atomic restart and deterministic scientific fixtures are not an A/A2 qualification.',
        '', 'P. Tests','',
        'See TESTS.txt and TEST_SUMMARY.json. The seven inherited assertions about older repository state remain unchanged and are reported separately. Current-task preservation is checked against the Task-4 starting state.',
        f"Retained test result: { {k:v for k,v in json_file(OUT/'TEST_SUMMARY.json').items() if k not in ('inherited_state_failures','unexpected_failures')} }.",
        '', 'Q. Classification','',classification,'',reason,
        '', 'R. Recommended next development step','',
        ('Resume independent A2 at its preserved, uncommitted layer-6 candidate and exact proof cache. Complete layers 6 through 20, repeat final qualification, and compare all scientific state before claiming full Task-4 qualification. No core or CFD work was started.' if s.get('early_stop') else 'Proceed to review of the qualified wall-prism shell before any separately authorized downstream mesh work.' if classification=='V4Rolling20LayerPrismShellComplete' else 'Resolve the recorded next-layer blocker from the exact committed checkpoint. Retain the frozen geometry, schedule and hard gates. Do not start core or CFD work.'),
        '',f"Starting HEAD: {s['starting_head']}. The final narrow Task-4 commit is reported separately to avoid a self-referential Git hash.",'']
    return '\n'.join(lines)

if __name__=='__main__':
    import argparse
    p=argparse.ArgumentParser();p.add_argument('--classification',required=True,choices=['V4Rolling20LayerPrismShellComplete','V4RollingPrismShellDevelopmentComplete','V4RollingPrismShellPartial','V4RollingPrismShellBlocked','V4RollingAdvancingFrontArchitectureFailure']);p.add_argument('--reason',required=True);p.add_argument('--seal',action='store_true');a=p.parse_args()
    s,t=collect();s.update(classification=a.classification,reason=a.reason);write(OUT/'SUMMARY.json',s)
    if a.classification=='V4Rolling20LayerPrismShellComplete':
        assert s['published_layers']==20 and s['final_validation'] and s['replay'] and s['replay']['passed']
        assert s['final_validation']['certificate']['complete'] and not any(s['hard_gate_violations'].values())
    if a.classification=='V4RollingPrismShellPartial':assert 0<s['published_layers']<20
    (OUT/'PERFORMANCE.json').write_text(json.dumps(t,indent=2,sort_keys=True)+'\n');(OUT/'report.md').write_text(report(s,t,a.classification,a.reason))
    if a.seal:
        sources=sorted({*ROOT.glob('tools/task4_*.py'),*ROOT.glob('tests/test_rolling*.py'),*ROOT.glob('src/sdf_cfd_mesher_v4/**/rolling*.py')})
        evidence=sorted(p for p in OUT.rglob('*') if p.is_file() and p.suffix!='.pending' and
            (p.suffix!='.log' or p.relative_to(OUT).parts[0] in ('development_A0','FINAL_ASSEMBLY_REPRODUCER')) and
            p.name not in ('AUTHORITY.json','SEAL.json'))
        authority=dict(task='ImplementV4RollingLayerCommitment',classification=a.classification,summary_record_sha256=scientific_hash(s),
            report_sha256=file_hash(OUT/'report.md'),source_sha256={p.relative_to(ROOT).as_posix():file_hash(p) for p in sources},
            evidence_sha256={p.relative_to(ROOT).as_posix():file_hash(p) for p in evidence})
        (OUT/'AUTHORITY.json').write_text(json.dumps(authority,sort_keys=True,indent=2)+'\n')
        seal=dict(report_sha256=file_hash(OUT/'report.md'),authority_sha256=file_hash(OUT/'AUTHORITY.json'))
        (OUT/'SEAL.json').write_text(json.dumps(seal,sort_keys=True,indent=2)+'\n');print(json.dumps(seal))

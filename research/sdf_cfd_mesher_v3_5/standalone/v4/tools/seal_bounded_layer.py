"""Seal one independently restored and retained production milestone."""
import argparse,time
from production_bounded_layers import *

def seal(layer):
    folder=configure(layer);started=time.perf_counter();a=read(folder/'A/READY.json');b=read(folder/'B/READY.json')
    assert a==b and a['passed'] and a['generated']==1
    admission=read(folder/'ADMISSION.json');assert admission['decision']['generation_attempt_authorized']
    assert admission['evidence']['binding']['committed']==cycle.CONFIG['committed']
    d=read(folder/'A/DIAGNOSTICS.json');cert=read(folder/'A/CERTIFICATE.json');gates=read(folder/'FINAL_GATE_SUMMARY.json')
    assert cert['passed'] and not cert['failures'] and cert['required_pairs']==cert['reused']+cert['new']==sum(cert['methods'].values())
    assert all(x['failures']==0 for scope in d['quality'].values() for x in scope.values())
    assert gates['quality_passed'] and gates['incidence_passed'] and gates['volume']['exact_volume_balance'] and gates['interior_degree']['passed']
    assert d['topology']['source_column_continuity'] and not d['topology']['duplicate_cells'] and not d['topology']['unused_vertices']
    assert all(not x['zero_area'] and not x['reversal'] and not x['nonfinite'] for x in d['fronts'].values())
    for label in ('A','B'):
        r=read(folder/label/'POST_RETENTION.json');assert r['passed'] and r['scientific']==a['committed']
    retention=read(folder/'RETENTION_RESULT.json');assert retention['passed']
    before=read(folder/'DISK_START.json');old_ids={physical_key(r) for r in before['rows']};rows=inventory()
    now={r['path']:r for r in rows}
    for r in before['rows']:
        if Path(r['path']).is_relative_to(OUT):continue
        n=now[r['path']];assert n['sha256']==r['sha256'] and physical_key(n)==physical_key(r) and n['cluster_bytes']==r['cluster_bytes']
    own=[r for r in rows if Path(r['path']).is_relative_to(folder)];physical={physical_key(r):r for r in own}
    raw=read(folder/'RETENTION_INVENTORY.json')['rows'];rawphys={physical_key(r):r for r in raw}
    closure,_=checkpoint_closure(folder/'A/checkpoints');closure[str((folder/'A/QUALITY.json').resolve())]=file_hash(folder/'A/QUALITY.json');closure[str((folder/'A/CANDIDATE.npz').resolve())]=file_hash(folder/'A/CANDIDATE.npz')
    active={physical_key(r):r for r in own if r['path'] in closure}
    samples=[json.loads(line) for line in (folder/'STORAGE_SAMPLES.jsonl').read_text().splitlines()]
    aliases={}
    for r in own:aliases.setdefault(physical_key(r),set()).add(Path(r['path']).relative_to(folder).parts[0])
    replay_shared={k for k,v in aliases.items() if {'A','B'}<=v}
    permanent={physical_key(r):r for r in own if 'checkpoints' not in Path(r['path']).parts and Path(r['path']).suffix not in ('.npz','.jsonl') and Path(r['path']).name not in ('QUALITY.json',) and 'RECEIPT_RETIREMENTS' not in Path(r['path']).parts}
    storage=dict(before=before['totals'],after=totals(rows),layer_totals=totals(own),
        new_unique_physical_before_retention=sum(r['cluster_bytes'] for k,r in rawphys.items() if k not in old_ids),
        new_unique_physical_retained=sum(r['cluster_bytes'] for k,r in physical.items() if k not in old_ids),
        shared_prior_physical=sum(r['cluster_bytes'] for k,r in physical.items() if k in old_ids),
        A_B_shared_physical=sum(physical[k]['cluster_bytes'] for k in replay_shared),
        active_current_physical=sum(r['cluster_bytes'] for r in active.values()),
        permanent_records_physical=sum(r['cluster_bytes'] for r in permanent.values()),compact_physical=metadata(folder/'COMPACT.json')['cluster_bytes'],
        new_duplicate_physical=totals(rows)['duplicate_physical_allocation']-before['totals']['duplicate_physical_allocation'],
        temporary_peak_sampled=max(s['temporary_allocated'] for s in samples),task_peak_sampled=max(s['tree_unique_allocated'] for s in samples),
        sample_errors=sum(len(s['errors']) for s in samples),sample_period_seconds=10,peaks_are_sampled_lower_bounds=True,
        physical_reclaimed=retention['physical_reclaimed'],start_free=before['free_bytes'],ending_free=shutil.disk_usage(ROOT).free,
        historical_payloads_and_file_ids_preserved=True,
        measurement_scope='Inventoried result payloads and records excluding .log files; sampled peak also includes logs. No free-space delta is treated as physical reclamation.',
        seconds=time.perf_counter()-started,**cycle.resources())
    save(folder/'STORAGE_FINAL.json',storage)
    science=dict(classification=f'V4ProductionAdaptiveCapsuleAdvancingFrontLayer{layer}Qualified',layer=layer,inputs=cycle.CONFIG,
        increment_hex=INCREMENTS[layer],admission_identity=admission['decision_identity'],result=a,
        diagnostics_sha256=file_hash(folder/'A/DIAGNOSTICS.json'),completed_neighbour=d['determinant']['completed_hash'],
        compact_identity=read(folder/'COMPACT.json')['identity'],compact_sha256=file_hash(folder/'COMPACT.json'),
        retention_sha256=file_hash(folder/'RETENTION_RESULT.json'),checkpoint=read(folder/'A/FINAL.json')['pointer'],
        next_layer_initialization=read(folder/'A/POST_RETENTION.json')['readiness'],
        implementation={str(p.relative_to(V4)):file_hash(p) for p in [V4/'tools/production_bounded_layers.py',V4/'tools/bounded_layer_evidence.py',V4/'tools/bounded_layer_retention.py',Path(__file__)]})
    evidence={str(p.relative_to(folder)):file_hash(p) for p in folder.rglob('*.json') if p.is_file() and 'checkpoints' not in p.parts and 'RECEIPT_RETIREMENTS' not in p.parts and p.name not in ('QUALITY.json','QUALIFIED.json')}
    save(folder/'QUALIFIED.json',dict(passed=True,retention_verified=True,committed=a['committed'],scientific=science,identity=identity(science),evidence=evidence))
    print(json.dumps(dict(layer=layer,qualified=identity(science),committed=a['committed'],storage=storage),indent=2),flush=True)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--layer',type=int,required=True);a=p.parse_args();seal(a.layer)

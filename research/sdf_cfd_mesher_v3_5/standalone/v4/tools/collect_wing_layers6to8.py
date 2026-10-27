"""Read completed/failed campaign evidence; never execute a transition."""
import json
from pathlib import Path
import subprocess
import time
from collect_dependency_transition_round_trip import quality
from path_preserving_deduplication import durable_json,file_hash,metadata
from check_current_front_encoding import allocation
from record_receipt_resume import disk

V4=Path(__file__).resolve().parents[1];ROOT=V4.parents[1]
OUT=V4/'results/wing_continuation_layers6to8'
# Existing ordinary-prism scope: immutable_state_access.ImmutableState's required
# legacy component mapping. Reporting must not accept a partial replay map.
EXPECTED_COMPONENTS=frozenset('version wall first_layer vertices cells faces columns outer prism_front versions authority atlas forecast quality certificate events'.split())


def read(p):return json.loads(Path(p).read_text(encoding='utf-8'))


def completed_scope(result,layers,stops,exit_code):
    """Require the full reported execution scope; never infer pass from a file."""
    if (stops or exit_code!=0 or not isinstance(result,dict) or result.get('passed') is not True
        or result.get('last_qualified_layer')!=8 or [row['layer'] for row in layers]!=[6,7,8]):
        return False
    for row in layers:
        records=row['records'];replay=records.get('REPLAY',{});reopen=records.get('REOPEN',{})
        generation=records.get('GENERATION',{});continuous=row.get('continuous',{});volume=row.get('volume_degree',{})
        retention=row.get('retention',{})
        required={'CONNECTION','TARGET','ADMISSION','GENERATION','ASSEMBLY','REPLAY','REOPEN'}
        if (row['missing'] or not required<=records.keys() or replay.get('passed') is not True or reopen.get('passed') is not True
            or set(replay.get('comparisons',{}))!=EXPECTED_COMPONENTS or any(v is not True for v in replay['comparisons'].values())
            or not replay.get('content') or replay['content']!=replay.get('replay_content')
            or replay['content']!=reopen.get('scientific')
            or records.get('ADMISSION',{}).get('decision',{}).get('generation_attempt_authorized') is not True
            or generation.get('hard_failures')!=[] or generation.get('topology_audit',{}).get('passed') is not True
            or continuous.get('passed') is not True or continuous.get('failures')!=[]
            or volume.get('exact_balance') is not True or volume.get('degree_passed') is not True
            or volume.get('unresolved')!=[] or not row.get('gates')
            or retention.get('passed') is not True or retention.get('scientific')!=reopen.get('scientific')):
            return False
    return True


def main():
    tick=time.perf_counter();base=read(OUT/'BASELINE.json');monitor=read(OUT/'MONITOR_FINAL.json')
    run=OUT/'A';layers=[];last=5;essential=[]
    for k in (6,7,8):
        folder=run/f'L{k}'
        if not folder.exists():continue
        entry=dict(layer=k,records={},missing=[])
        for name in ('CONNECTION','TARGET','ADMISSION','GENERATION','ASSEMBLY','REPLAY','REOPEN'):
            p=folder/(name+'.json')
            if p.exists():
                body=read(p);essential.append(p)
                if name=='GENERATION':
                    body={key:body[key] for key in ('counts','coordinates','new_connectivity','phases','seconds',
                        'hard_failures','topology_audit','forecast_coordinate_exact','forecast_max_abs_difference','sdf') if key in body}
                entry['records'][name]=body
            else:entry['missing'].append(name)
        p=folder/'NEW_QUALITY.json'
        if p.exists():entry['gates']=quality(p)
        p=folder/'FRESH_CERTIFICATE.json'
        if p.exists():
            body=read(p);entry['continuous']={key:body[key] for key in ('passed','required_pairs','prior_population',
                'combined_pairs','failures','methods','prior_payloads_read','prior_payloads_recomputed')}
            essential.append(p)
        p=folder/'VOLUME_DELTA.json'
        if p.exists():
            body=read(p);entry['volume_degree']=dict(exact_balance=body['exact_balance'],
                degree_passed=body['global_degree']['passed'],unresolved=body['global_degree']['unresolved'])
            essential.append(p)
        p=folder/'replay_retention/RESULT.json'
        if p.exists():entry['retention']=read(p);essential.append(p)
        reopened=entry['records'].get('REOPEN',{})
        if reopened.get('passed') and entry['records'].get('REPLAY',{}).get('passed'):last=k
        layers.append(entry)
    stops={str(p.relative_to(OUT)):read(p) for p in (OUT/'RESOURCE_STOP.json',run/'STOP.json') if p.exists()}
    if stops:
        pending=[dict(**metadata(p),sha256=file_hash(p)) for p in run.rglob('*.pending')]
        journals=[dict(path=str(p.relative_to(OUT)),sha256=file_hash(p),record=read(p)) for p in run.rglob('JOURNAL_*.json')]
        pointers=[dict(path=str(p.relative_to(OUT)),sha256=file_hash(p),record=read(p)) for p in run.rglob('CURRENT.json')]
        durable_json(OUT/'FAILED_TRANSACTION.json',dict(pending=pending,journals=journals,pointers=pointers,
            scope='Read-only snapshot after monitor termination. No recovery, retry, promotion or deletion.'))
    result=read(run/'RESULT.json') if (run/'RESULT.json').exists() else None
    completed=completed_scope(result,layers,stops,monitor['exit_code'])
    summary=dict(classification='V4AuthorizedWingL8ContinuationQualified' if completed else 'V4AuthorizedWingContinuationStoppedOrIncomplete',
        last_qualified_layer=last,authorization=read(OUT/'CONTROL.json')['authorization'],
        layers=layers,stops=stops,result=result,
        monitor={key:monitor[key] for key in ('exit_code','peaks','seconds','storage_roots','storage','disk')},
        no_layer9=True,no_gate_or_geometry_change=True)
    durable_json(OUT/'SUMMARY.json',summary)
    for row in base['protected']:
        if file_hash(ROOT/row['path'])!=row['sha256']:raise ValueError('ProtectedFileChanged',row['path'])
    prior=read(V4/'results/current_front_transition_round_trip/BASELINE.json')
    for row in prior['dependencies']:
        if file_hash(row['path'])!=row['record']['sha256']:raise ValueError('RetainedDependencyChanged',row['path'])
    for node in prior['active_ancestry']:
        ref=node['reference']
        if ref['kind']=='recursive':
            folder=Path(ref['directory']);pointer=read(folder/'CURRENT.json')
            if pointer['root']!=ref['root'] or file_hash(folder/pointer['file'])!=node['root_file_sha256']:
                raise ValueError('RetainedRootChanged',str(folder))
    prefix='research/sdf_cfd_mesher_v4/'
    owned={'tools/accepted_production_contract.py','tools/run_authorized_continuation.py','tools/finish_retained_l5.py',
        'tools/run_wing_layers6to8.py','tools/collect_wing_layers6to8.py','tests/test_continuation_successor.py',
        'tests/test_continuation_resource_monitor.py','tests/test_wing_continuation_summary.py',
        'CURRENT_TASK.md','README.md','WING_L6_L8_CONTINUATION_REPORT.md'}
    allowed={prefix+x for x in owned}
    def unrelated(rows):return [s for s in rows if s[3:] not in allowed and not s[3:].startswith(prefix+'results/wing_continuation_layers6to8')]
    original=read(OUT/'STARTING_STATUS.json')
    current=subprocess.check_output(['git','-c','core.quotepath=false','status','--porcelain=v1'],cwd=ROOT).decode('utf-8').splitlines()
    if unrelated(current)!=unrelated(original['status'].splitlines()):raise ValueError('UnrelatedStatusChanged')
    preservation=dict(unrelated_entries=len(unrelated(current)),case_sensitive_status_match=True,protected_files=len(base['protected']),
        retained_dependency_files=len(prior['dependencies']),active_recursive_roots=sum(n['reference']['kind']=='recursive' for n in prior['active_ancestry']),
        scope='Selected hashes, active root pointers and exact status. No whole-repository inventory.')
    durable_json(OUT/'PRESERVATION.json',preservation)
    paths=[p for p in OUT.rglob('*') if p.is_file()]
    retained=allocation(paths)
    combined=allocation([p for root in map(Path,monitor['storage_roots']) for p in root.rglob('*') if p.is_file()])
    small=[p for p in OUT.glob('*.json') if p.name not in ('BASELINE.json','MONITOR_FINAL.json','EVIDENCE.json')]
    unique={str(p):p for p in essential+small}
    evidence=dict(starting_head=base['head'],head_at_measurement=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
        initial_disk=base['disk'],final_disk=disk(),retained={k:v for k,v in retained.items() if k!='records'},
        combined_campaign={k:v for k,v in combined.items() if k!='records'},preservation=preservation,
        essential=[dict(path=str(p.relative_to(OUT)),sha256=file_hash(p),bytes=p.stat().st_size) for p in unique.values()],
        sources=[dict(path=p,sha256=file_hash(V4/p)) for p in sorted(owned) if (V4/p).exists()],
        elapsed_seconds=(time.time()*1000-original['task_start_ms'])/1000,reducer_seconds=time.perf_counter()-tick,
        cleanup='Only existing successful-only replay retention if completed; no failed evidence or active ancestry deleted.',aliases_not_added=True)
    durable_json(OUT/'EVIDENCE.json',evidence)
    print(json.dumps(dict(last_qualified_layer=last,stops=stops,monitor=summary['monitor'],preservation=preservation,
        retained=evidence['retained']),indent=2))


if __name__=='__main__':main()

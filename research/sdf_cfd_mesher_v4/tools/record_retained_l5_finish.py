"""Reduce retained and resumed evidence; no geometry or gate implementation."""
from finish_retained_l5 import *

def main():
    baseline=read(OUT/'BASELINE.json')
    allowed={'research/sdf_cfd_mesher_v4/tools/critical_index_pipeline.py'}
    changed=[]
    for r in baseline['files']:
        if file_hash(ROOT/r['path'])!=r['sha256']:
            changed.append(r['path'])
    if set(changed)-allowed:raise ValueError('ProtectedContentChanged',changed)
    current=status()
    if any(s not in current for s in baseline['unrelated']):raise ValueError('UnrelatedStatusChanged')
    task_prefixes=('research/sdf_cfd_mesher_v4/tools/finish_retained_l5.py',
        'research/sdf_cfd_mesher_v4/tools/record_retained_l5_finish.py',
        'research/sdf_cfd_mesher_v4/tools/finish_retained_l5_retention.py',
        'research/sdf_cfd_mesher_v4/tools/critical_index_pipeline.py',
        'research/sdf_cfd_mesher_v4/tests/test_retained_l5_finish.py',
        'research/sdf_cfd_mesher_v4/results/retained_l5_finish',
        'research/sdf_cfd_mesher_v4/RETAINED_L5_COMPLETION_REPORT.md')
    if [s for s in current if not s[3:].startswith(task_prefixes)]!=baseline['unrelated']:
        raise ValueError('UnexpectedWorktreeDelta')
    retired={}
    final=read(OUT/'RESULT.json') if (OUT/'RESULT.json').exists() else {}
    peaks=read(OUT/'MONITOR_FINAL.json')['peaks']
    if (OUT/'RETENTION_MONITOR_FINAL.json').exists():
        later=read(OUT/'RETENTION_MONITOR_FINAL.json')['peaks']
        peaks={k:max(v,later[k]) for k,v in peaks.items()}
    from path_preserving_deduplication import probe_file_id
    retired_index_probes=[]
    if (OUT/'RESTART_INDEX_RETENTION.json').exists():
        for row in read(OUT/'RESTART_INDEX_RETENTION.json')['files']:
            error=probe_file_id(row['file_id'])
            if error==0:raise ValueError('RestartIndexAllocationStillOpen')
            retired_index_probes.append(dict(path=row['path'],file_id=row['file_id'],open_by_id_error=error))
    retention=Path(final.get('retention_evidence',str(FOLDER/'replay_retention')))/'COMPACT.json'
    if retention.exists():
        retired={r['path']:r for r in read(retention)['discarded']}
    preserved=0;recoverable=0
    for r in baseline['dependencies']:
        p=Path(r['path'])
        if p.exists():
            if p.stat().st_size!=r['bytes'] or file_hash(p)!=r['sha256']:
                raise ValueError('ChangedRetainedScientificInput:'+str(p))
            preserved+=1
        else:
            row=retired[str(p)];canonical=Path(row['proof']['canonical'])
            if row['sha256']!=r['sha256'] or file_hash(canonical)!=r['sha256']:
                raise ValueError('RetiredInputNotRecoverable:'+str(p))
            recoverable+=1
    oldsummary=read(V4/'results/wing_receipt_resume/SUMMARY.json')['layers']['5']
    # The compact numerical reduction is reused only against its exact inputs.
    for r in oldsummary['files']:
        if Path(r['path']).name in ('NEW_QUALITY.json','AFFECTED_INCIDENCE.json','VOLUME_DELTA.json','GENERATION.json','FRESH_CERTIFICATE.json'):
            if file_hash(ROOT/r['path'])!=r['sha256']:raise ValueError('ChangedMeasuredInput')
    result=dict(classification='V4RetainedWingL5ReplayQualifiedRecursivelyPublishedAndReopened' if (OUT/'RESULT.json').exists() else 'V4RetainedWingL5ResumeStopped',
        result=read(OUT/'RESULT.json') if (OUT/'RESULT.json').exists() else None,
        stop=read(OUT/'STOP.json') if (OUT/'STOP.json').exists() else None,
        generation=read(FOLDER/'GENERATION.json'),assembly=read(FOLDER/'ASSEMBLY.json'),
        replay_assembly=read(FOLDER/'replay/ASSEMBLY.json') if (FOLDER/'replay/ASSEMBLY.json').exists() else None,
        quality=oldsummary['quality'],incidence=oldsummary['incidence'],continuous=oldsummary['continuous'],volume=oldsummary['volume'],
        metrics_source=file_hash(V4/'results/wing_receipt_resume/SUMMARY.json'),
        preserved=dict(unrelated_entries=len(baseline['unrelated']),protected_files=len(baseline['files']),intentional_source_changes=changed,
            original_dependency_files_unchanged=preserved,exact_recovery_references=recoverable),
        disk=disk(),campaign=allocation([CAMPAIGN]),task=allocation([OUT]),
        monitor=peaks,retired_index_probes=retired_index_probes,
        source_identities={p:file_hash(ROOT/p) for p in sorted(allowed|set(task_prefixes[:3]))})
    save('SUMMARY.json',result)
    if result['result'] is not None:
        save('LAST_QUALIFIED.json',dict(layer=5,**result['result']['reopen'],
            scope='Completion of retained L5; original stopped campaign records remain unchanged'))
    paths=[p for p in OUT.rglob('*') if p.is_file() and p.name!='EVIDENCE.json']
    paths+=list((FOLDER/'replay_retention').glob('*.json'))
    paths += [FOLDER/n for n in ('REPLAY.json','REOPEN.json','CANDIDATE_PLAN.json','REFERENCE.json','ASSEMBLY.json','INDEX_RESTART.json','PUBLICATION.json') if (FOLDER/n).exists()]
    save('EVIDENCE.json',dict(files=[dict(path=str(p.relative_to(ROOT)).replace('\\','/'),bytes=p.stat().st_size,sha256=file_hash(p)) for p in sorted(paths)],
        source_identities=result['source_identities'],geometry='079606dc36582ed38e51e5485e1f853e247a2ae40b544414c9e649552ae85311',
        original_dependencies=file_hash(OUT/'BASELINE.json'),no_L6=True))
    print(json.dumps({k:result[k] for k in ('classification','quality','incidence','continuous','volume','preserved','disk','campaign','task','monitor')},indent=2))

if __name__=='__main__':main()

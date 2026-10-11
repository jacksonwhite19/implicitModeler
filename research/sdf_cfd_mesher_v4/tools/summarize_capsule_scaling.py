"""Reduce measured traces without retaining scientific populations in reports."""
import json
from pathlib import Path
from collections import defaultdict
from sdf_cfd_mesher_v4.common.authority import file_hash

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'results/generic_capsule_certification_scaling'


def read(path):return json.loads(path.read_text())
def save(path,value):path.write_text(json.dumps(value,sort_keys=True,indent=2)+'\n')


def object_rows(value):
    if isinstance(value,dict):
        if 'file' in value and 'sha256' in value:yield value
        for child in value.values():yield from object_rows(child)
    elif isinstance(value,list):
        for child in value:yield from object_rows(child)


def component_accounting(folder,before=None,pointer=None):
    entry=read(folder/'INPUT.json');source=Path(entry['source'])
    if before is None:before=read(source/entry['pointer']['checkpoint'])
    store=folder/'checkpoints'
    if pointer is None:pointer=read(store/'CURRENT.json')
    after=read(store/pointer['checkpoint'])
    old={r['file'] for r in object_rows(before)}
    objects={r['file']:r for r in object_rows(after)}
    measured={name:(store/'objects'/name).stat().st_size for name in objects}
    new={k:v for k,v in measured.items() if k not in old}
    parts={}
    for name,row in after['components'].items():
        references={r['file'] for r in object_rows(row)}
        parts[name]=dict(encoding=row['encoding'],count=row.get('count'),
            scientific=row.get('scientific',row.get('sha256')),
            referenced_bytes=sum(measured[p] for p in references),
            newly_published_bytes=sum(new.get(p,0) for p in references))
    return dict(scientific=pointer['scientific'],manifest=pointer,
        new_object_bytes=sum(new.values()),referenced_object_bytes=sum(measured.values()),
        unchanged_objects_reused=len(objects)-len(new),new_objects=len(new),components=parts,
        scope='Unique authoritative object bytes newly published, excluding source hardlinks, temporary SQLite writes and superseded intermediate checkpoints')


def reduce_run(folder):
    rows=[json.loads(s) for s in (folder/'PROFILE.jsonl').read_text().splitlines()]
    stages=defaultdict(list)
    for r in rows:
        if r['event']=='exit':stages['/'.join(r['phase'][1:])].append(r['duration_seconds'])
    result=dict(label=folder.name,case=folder.parent.name,
        phases=dict(stages),peak_rss_bytes=max(r['peak_rss_bytes'] for r in rows),
        retained=[r for r in rows if r['event']=='retained_after_commit'],
        accounting=[r for r in rows if r['event'].endswith('_accounting')],
        benchmark=[r for r in rows if r['event']=='benchmark_end'],
        raw_profile_sha256=file_hash(folder/'PROFILE.jsonl'))
    peak=result['peak_rss_bytes']
    result['first_peak_observation']=next(r for r in rows if r['peak_rss_bytes']==peak)
    if (folder/'TRACE.json').exists():result['trace']=read(folder/'TRACE.json')
    if (folder/'SHELL_RESULT.json').exists():
        shell=read(folder/'SHELL_RESULT.json')
        result['shell']={k:shell[k] for k in ('current_layer','cells','columns','final','wall_exact','first_layer_exact','restart_exact','remesh_count')}
        result['checkpoint']=component_accounting(folder)
        result['checkpoint_layers']=[]
        entry=read(folder/'INPUT.json');before=read(Path(entry['source'])/entry['pointer']['checkpoint'])
        manifests=[(p,read(p)) for p in (folder/'checkpoints').glob('NATIVE_*.json')]
        for step in result.get('trace',[]):
            matched=[(p,e) for p,e in manifests if e['scientific']==step['state']]
            if len(matched)!=1:raise ValueError('AmbiguousLayerManifest')
            path,after=matched[0]
            pointer=dict(checkpoint=path.name,scientific=step['state'],sha256=file_hash(path))
            result['checkpoint_layers'].append(dict(layer=step['layer'],**component_accounting(folder,before,pointer)))
            before=after
    save(folder/'SUMMARY.json',result)
    return result


def main():
    runs=[reduce_run(p.parent) for p in sorted(OUT.glob('*/*/PROFILE.jsonl')) if (p.parent/'SHELL_RESULT.json').exists()]
    ledger={r['case']+'/'+r['label']:r for r in runs}
    prefix='capsule_production_control/'
    baseline=ledger[prefix+'baseline']
    equality={}
    for name in ('delta1','optimized','final'):
        if prefix+name not in ledger:continue
        other=ledger[prefix+name]
        different=[k for k,v in baseline['checkpoint']['components'].items()
                   if v['scientific']!=other['checkpoint']['components'][k]['scientific']]
        if different or other['shell']['final']!=baseline['shell']['final']:raise ValueError('BenchmarkScientificMismatch')
        equality[name]=dict(exact=True,scientific=other['shell']['final'],different_components=different,
                           components=len(baseline['checkpoint']['components']))
    save(OUT/'PERFORMANCE_LEDGER.json',ledger)
    save(OUT/'SCIENTIFIC_COMPARISON.json',equality)
    entries=[]
    for p in sorted(OUT.rglob('*')):
        if not p.is_file() or 'checkpoints' in p.relative_to(OUT).parts or 'interface' in p.relative_to(OUT).parts:continue
        if p.name in ('EVIDENCE_INDEX.json','report.md','AUTHORITY.json','SHA256SUMS.txt'):continue
        entries.append(dict(path=p.relative_to(OUT).as_posix(),bytes=p.stat().st_size,sha256=file_hash(p)))
    save(OUT/'EVIDENCE_INDEX.json',entries)
    print(json.dumps({k:dict(layers=[t['layer'] for t in r.get('trace',[])],peak=r['peak_rss_bytes'],published=r.get('checkpoint',{}).get('new_object_bytes')) for k,r in ledger.items()}))


if __name__=='__main__':main()

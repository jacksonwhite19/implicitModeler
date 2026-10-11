"""Report measured timer scopes; never fill absent measurements with guesses."""
import json
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]/'results/incremental_certificate_receipt_identity'


def collect(folder):
    identities=json.loads((folder/'IDENTITY_TIMERS.json').read_text())
    events=[json.loads(x) for x in (folder/'PROFILE.jsonl').read_text().splitlines()]
    traces=json.loads((folder/'TRACE.json').read_text()) if (folder/'TRACE.json').exists() else []
    result=[];lower=0.;prior=None
    for event in [x for x in events if x['event'] in ('initial_checkpoint','retained_after_commit')]:
        if event['event']=='initial_checkpoint':lower=event['seconds'];prior=event['relations'];continue
        layer=event['layer'];upper=event['seconds']
        observed=next(x for x in identities if x['event']=='retained_after_commit' and x['layer']==layer)
        trace=next((x for x in traces if x['layer']==layer),{})
        interval=[x for x in events if lower<=x['seconds']<=upper and 'qualification_legacy_crosscheck' not in x['phase']]
        stages={}
        for x in interval:
            if x['event']=='exit':
                name='/'.join(x['phase'][1:]);stages[name]=stages.get(name,0.)+x['duration_seconds']
        certificates=[x for x in interval if x['event']=='certificate_accounting' and x.get('stage')=='certificate_reuse_and_coverage']
        receipts=[x for x in interval if x['event']=='receipt_accounting']
        result.append(dict(label=folder.name,layer=layer,accepted_seconds=trace.get('seconds'),
            historical_relations=prior,new_relations=event['relations']-prior,total_relations=event['relations'],
            retained_rss=event['rss_bytes'],process_peak_rss=event['peak_rss_bytes'],
            sampled_interval_peak_rss=max(x['rss_bytes'] for x in interval),
            identity_timers=observed['timers'],stages=stages,
            certificate_accounting=certificates[-1] if certificates else None,
            receipt_accounting=receipts[-1] if receipts else None,
            native_memory=observed.get('memory'),
            note='Null means Not measured. Timers overlap; accepted runtime is separate from bootstrap and qualification audits.'))
        lower=upper;prior=event['relations']
    return result


def main():
    data=[]
    for path in sorted(ROOT.glob('*/*/IDENTITY_TIMERS.json')):
        data.extend(collect(path.parent))
    (ROOT/'PERFORMANCE_LEDGER.json').write_text(json.dumps(data,indent=2)+'\n')
    for row in data:
        print(json.dumps({k:v for k,v in row.items() if k not in ('native_memory','certificate_accounting','receipt_accounting')}))


if __name__=='__main__':main()

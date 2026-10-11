from task4k_common import *
import statistics

def run():
    runtime={};layers=[]
    for label in ('A','B'):
        rows=read(OUT/f'REPLAY_{label}.json');assert len(rows)==20
        totals=collections.Counter();details=collections.Counter();methods=collections.Counter()
        for r in rows:
            p=r['performance'];totals.update({k:v for k,v in p.items() if isinstance(v,(float,int))});details.update(p['details']);methods.update(p['exact_methods'])
            layers.append(dict(replay=label,layer=r['layer'],pairs=r['pairs'],seconds=p['seconds'],cad_worker_seconds=p['details'].get('method:NoncoincidentRank3ExactCAD',0),noncad_worker_seconds=p['details'].get('kernel',0)-p['details'].get('method:NoncoincidentRank3ExactCAD',0),methods=p['exact_methods'],equivalent=r['equivalent'],scientific_hash=r['scientific_hash']))
        runtime[label]=dict(totals=totals,details=details,methods=methods,pairs=sum(r['pairs'] for r in rows),worst_layer=max(rows,key=lambda r:r['performance']['seconds'])['layer'],worst_seconds=max(r['performance']['seconds'] for r in rows),peak_rss=read(OUT/f'REPLAY_{label}_RESOURCE.json')['peak_rss'])
    values=[runtime[k]['totals']['seconds'] for k in ('A','B')]
    runtime['spread']=max(values)-min(values);runtime['ratio']=max(values)/min(values)
    save('TRANSACTION_RUNTIME.json',runtime);save('PER_LAYER_REPLAY.json',layers)
    system={}
    for mode in ('BASELINE','OPTIMIZED'):
        rows=read(OUT/f'{mode}_CASES.json');samples=read(OUT/f'{mode}_SYSTEM.json');groups=[]
        for worker in range(8):
            cohort=[r for r in rows if r['group']==worker]
            groups.append(dict(worker=worker,pids=sorted({r['pid'] for r in cohort}),initial_queue_seconds=cohort[0]['initial_queue_seconds'],worker_seconds=sum(r['execution_seconds'] for r in cohort),cpu_seconds=sum(r['cpu_seconds'] for r in cohort),gc_seconds=sum(r['gc_seconds'] for r in cohort),rss_first=cohort[0]['rss_before'],rss_last=cohort[-1]['rss_after'],rss_peak=max(r['peak_rss'] for r in cohort),sampled_object_deltas=[r['sampled_objects_after']-r['sampled_objects_before'] for r in cohort if r['sampled_objects_before'] is not None]))
        system[mode]=dict(groups=groups,peak_rss=samples['peak_rss'],peak_processes=max(r['processes'] for r in samples['samples']),system_cpu_median=statistics.median(r['system_cpu'] for r in samples['samples']),cpu_wall_ratio=sum(r['cpu_seconds'] for r in rows)/sum(r['execution_seconds'] for r in rows),worker_recycling_observed=False,context_switches_measured=False,return_delay_includes_remaining_cohort=True)
    save('SYSTEM_AUDIT.json',system)

if __name__=='__main__':run()

"""Measured process resources, with shared-machine timing limitations."""
import json
import re
import capsule_steady_models as s
h=s.h


def run():
    samples=[]
    for path in (s.ROOT/'RESOURCE_SAMPLES.jsonl',s.ROOT/'FINE_RESOURCE_SAMPLES.jsonl',s.ROOT/'model_runs/RESOURCE_SAMPLES.jsonl'):
        samples.extend(json.loads(line) for line in path.read_text().splitlines() if line.strip())
    folders=[s.ROOT/n for n in ('GC2-S1','GC2-S2','GC2F-S1','GC2F-S2')]
    folders += [s.ROOT/'model_runs'/n for n in ('SA-GW1-S1','SA-GW1-S2')]
    runs={}
    for folder in folders:
        text=(folder/'pilot.resources').read_text()
        def value(label):
            m=re.search(re.escape(label)+r':\s*([^\n]+)',text);assert m,label
            return m[1].strip()
        rows=[r['cases'][folder.name] for r in samples if folder.name in r['cases']]
        runs[folder.name]=dict(wall_time=value('Elapsed (wall clock) time (h:mm:ss or m:ss)'),
            user_cpu_seconds=float(value('User time (seconds)')),system_cpu_seconds=float(value('System time (seconds)')),
            max_single_process_RSS_KiB=int(value('Maximum resident set size (kbytes)')),
            sampled_aggregate_RSS_bytes=max(r['rss_bytes'] for r in rows),
            max_sampled_rank_count=max(r['ranks'] for r in rows),exit_status=int(value('Exit status')),
            raw_resource_sha256=h.file_hash(folder/'pilot.resources'))
    result=h.seal(dict(schema='MeasuredSteadyStudyPerformance/1',runs=runs,
        minimum_sampled_free_bytes=min(r['free_bytes'] for r in samples),
        limitation='Shared-machine runs; elapsed ratios are not controlled solver-speed benchmarks. RSS is sampled aggregate rank RSS, distinct from GNU time maximum single-process RSS.'))
    h.durable_json(s.ROOT/'PERFORMANCE.json',result);print(result['identity'])


if __name__=='__main__':run()

"""Measured comparison only; no missing timings are estimated."""
import json
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]/'results/canonical_block_authenticated_storage'
PRIOR=ROOT.parent/'incremental_certificate_receipt_identity/capsule_production_control'

def read(path):return json.loads(path.read_text())
def observations(path,name):return [r for r in read(path/name) if r['event']=='retained_after_commit']
def phases(path):
    return [r for r in map(json.loads,(path/'PROFILE.jsonl').read_text().splitlines()) if r['event']=='exit' and r['phase'][-1]=='checkpoint_serialize']

def run():
    folder=ROOT/'capsule_production_control/capsule_7_9_retry';new=observations(folder,'BLOCK_TIMERS.json')
    trace={r['layer']:r for r in read(folder/'TRACE.json')};checkpoint=dict(zip((r['layer'] for r in new),phases(folder)))
    old={};oldtrace={};oldcheckpoint={}
    for label in ('final_bench','final_8_9'):
        source=PRIOR/label;records=observations(source,'IDENTITY_TIMERS.json')
        old.update({r['layer']:r for r in records});oldtrace.update({r['layer']:r for r in read(source/'TRACE.json')})
        oldcheckpoint.update(dict(zip((r['layer'] for r in records),phases(source))))
    rows=[]
    for r in new:
        layer=r['layer'];prior=old[layer];work=r['block_work'];a=prior['memory']['process'];b=r['memory']['process']
        rows.append(dict(layer=layer,changed_new=work['insert_count']+work['replace_count'],insert=work['insert_count'],replace=work['replace_count'],retire=work['retire_count'],relations=r['relations'],active_blocks=256,touched_blocks=work['touched_blocks'],untouched_blocks=work['untouched_blocks'],touched_fraction=work['touched_blocks']/256,block_writes=work['block_writes'],bytes_written=work['bytes_written'],bytes_reused=work['bytes_reused'],active_bytes=r['active_bytes'],upper_internal_nodes=work['upper_internal_nodes_rewritten'],root_update_before=prior['timers']['incremental_receipt_identity']['seconds'],root_update_after=r['timers']['incremental_receipt_identity']['seconds'],block_update_after=r['timers']['block_update']['seconds'],receipt_storage_before=prior['timers']['delta_receipt_storage']['seconds'],receipt_storage_after=r['timers']['delta_receipt_storage']['seconds'],layer_seconds_before=oldtrace[layer]['seconds'],layer_seconds_after=trace[layer]['seconds'],checkpoint_seconds_before=oldcheckpoint[layer]['duration_seconds'],checkpoint_seconds_after=checkpoint[layer]['duration_seconds'],peak_rss_before=a['peak_wset'],peak_rss_after=b['peak_wset'],retained_rss_before=a['rss'],retained_rss_after=b['rss'],block_root=r['root'],storage=read(folder/f'RETENTION_{layer}.json')['payload'],legacy=read(folder/f'LEGACY_CROSSCHECK_{layer}.json'),allocator=r['memory']['cpython_allocator_counters'],mapped_region_bytes=r['memory'].get('mapped_region_bytes_total')))
    (ROOT/'BENCHMARK_COMPARISON.json').write_text(json.dumps(rows,indent=2)+'\n')
    for row in rows:print(json.dumps({k:v for k,v in row.items() if k not in ('allocator','legacy')}),flush=True)

if __name__=='__main__':run()

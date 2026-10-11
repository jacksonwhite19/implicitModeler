"""Reduce measured profiles; do not infer passes from absent run evidence."""
import json,sys,subprocess
from pathlib import Path
from collections import Counter
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'results/native_runner_memory_profile'
def load(path):return json.loads((OUT/path).read_text())
def save(path,value):(OUT/path).write_text(json.dumps(value,indent=2,sort_keys=True)+'\n',encoding='utf8')
def run():
    execution=load('fresh/EXECUTION.json');comparison=load('fresh/SCIENTIFIC_COMPARISON.json');audit=load('fresh/FINAL_NATIVE_AUDIT.json')
    assert execution['status']=='Completed' and comparison['passed']
    assert comparison['remesh_events']==0 and audit['certificate']['active']==56462864
    trace=load('fresh/memory_profile/RSS_TRACE.json');peak=load('fresh/memory_profile/SUMMARY.json')['peak']
    phases={}
    for row in trace:
        phase=row['phase']
        if 'allocation_accounting' in phase:continue
        if phase not in phases or phases[phase]['rss']<row['rss']:phases[phase]=row
    retained=[r for r in trace if r.get('mark') in ('initial_front_ready','committed_layer','before_final_validation','task_end')]
    save('PHASE_SUMMARY.json',dict(peak=peak,phase_maxima=phases,retained=retained,
        metric='Sum of production parent and descendant RSS; shared pages can be counted by each process. Controller excluded.'))
    before=load('COPY_BEFORE.json');after=load('COPY_BOUNDED.json');hash_only=load('COPY_AFTER.json')
    assert before['hashes']==after['hashes']==hash_only['hashes']
    old=3405565952;new=max(execution['peak_tree_rss'],peak['rss']);seconds=execution['scientific_seconds']
    summary=dict(classification='V4NativeRunnerMemoryQualified',baseline_peak_rss=old,optimized_peak_rss=new,
        reduction_bytes=old-new,reduction_percent=100*(old-new)/old,
        scientific_seconds=seconds,native_runtime_difference=seconds-1393.28,legacy_runtime_difference=seconds-1248.01,
        fresh_peak=peak,retained=retained,matched_copy_probe=dict(before=before,hash_only=hash_only,after=after),
        cold_projection_bytes_before=188161620,cold_projection_bytes_after=51026880,cold_projection_bytes_released=137134740,
        historical_runtime_caveat='Prior full run included a NumPy runtime repair and receipt recomputation. Matched allocation probes use Python 3.12.10 / NumPy 1.26.4. Do not attribute the entire historical RSS/runtime difference to these patches.',
        production_authority='native-mixed-front',automatic_remesh=True)
    assert new<old
    save('BEFORE_AFTER.json',summary)
    save('ALLOCATION_AUDIT.json',dict(
        baseline_profiles={name:load(name+'/ALLOCATIONS.json') for name in ('baseline','baseline_warnings','baseline_first/profile')},
        optimized_certificate=load('optimized/ALLOCATIONS.json'),
        ownership=[
          dict(subsystem='committed geometry',owner='PrismBacking',storage='read-only content-addressed NPY mappings, shared by native entity views',lifetime='checkpoint-backed; no expanded cell object graph'),
          dict(subsystem='forecast',owner='NativeForecast',storage='mapped coordinate and normal trajectories',lifetime='current schedule identity; past coordinates also bind exact prefix validation'),
          dict(subsystem='atlas',owner='NativeRiskAtlas',storage='views of forecast; per-source masks and identities',lifetime='one proposal; accepted planner buffer copy remains local'),
          dict(subsystem='detailed lookahead',owner='PersistentPlanner',storage='bounded support entries and dependency-local metrics',lifetime='proposal; passive begin clears hot caches'),
          dict(subsystem='exact Jacobians',owner='LOCAL_EXACT',storage='existing bounded coordinate-keyed receipts',lifetime='process; unchanged qualified capacity'),
          dict(subsystem='certificate',owner='certify_immediate',storage='one output quad/label block, transaction pairs and exact worker jobs',lifetime='one committed layer'),
          dict(subsystem='manifest',owner='NativeManifest',storage='mapped relation blocks, compact metadata',lifetime='disk-backed; no full Python relation population'),
          dict(subsystem='checkpoint',owner='NativeCheckpointStore',storage='streamed components and SQLite rows',lifetime='atomic publication; no full JSON envelope'),
          dict(subsystem='journal',owner='MixedFrontJournal',storage='no topology transaction on ordinary prism commits',lifetime='bounded remesh only'),
          dict(subsystem='reports',owner='task3c_io',storage='streamed canonical tokens to gzip and SHA-256',lifetime='bounded encoder block; source records preserved')],
        numpy_copy_audit=[
          dict(operation='whole-array tobytes in array_hash',action='removed; hash consumes contiguous buffer',proof='COPY_BEFORE/AFTER/BOUNDED hashes'),
          dict(operation='certificate concatenate plus four shell fancy-index arrays and stack',action='one output allocation; layer/edge-plane fill',proof='408199680 output bytes; matched RSS probe'),
          dict(operation='initial projection diagnostics retained through all metric scans',action='release unused fields after trajectory persistence',proof='188161620 to 51026880 owned bytes; weak-reference regression'),
          dict(operation='canonical JSON plain graph/string/bytes',action='stream tokens and SHA-256',proof='canonical byte-equality tests'),
          dict(operation='proposal forecast copies and atlas acceptance copy',action='retained bounded private mutation buffers; no scientific ownership change'),
          dict(operation='pair uint32 conversion and mapped audit block arithmetic',action='retained transaction-block working sets; no 56M object expansion')],
        integer_precision='No integer or floating-point dtype changes. Existing uint64 labels and packed pairs retained.',
        garbage_collection='No new forced GC in production. Diagnostic scope-release probes use GC explicitly and are labeled.'))
    print(json.dumps(summary,indent=2))
if __name__=='__main__':run()

"""New rings only; preserved solver, directions, gates and local forecast."""
import task3_repair_study as study
from task3_repair_study import ROOT,plain,scientific_hash
import json,time,sys,numpy as np
from task3c_io import read as read_evidence,write as write_evidence
OUT=ROOT/'results/task3c'

def compact(r):
    # Retain complete event gate, projection, target and collateral evidence.
    # Full local metrics/tails are saved once per ring in compressed arrays.
    return r

def run(replay=False):
    started=time.perf_counter();ctx=study.load();results=[];perf={}
    study.CONFIG.update(active_rings=[2,3],max_active_vertices=160,max_local_columns=640)
    for s in study.components():
        if s['name'] not in ('component_00','component_01','component_02','component_04'):continue
        for ring in (2,3):
            study.CONFIG['active_rings']=[ring]
            r,p=study.run_component(s,ctx)
            name=s['name']+f'_ring{ring}';h=scientific_hash(r)
            if replay:
                old=read_evidence(OUT/f'{name}.json');assert scientific_hash(old)==h,name
            else:
                write_evidence(OUT/f'{name}.json',r)
            results.append(dict(name=name,scientific_hash=h,classification=r['final']['classification'],accepted=len(r['accepted_steps'])))
            perf[name]=p
            print(name,r['final']['classification'],h,p['seconds'],flush=True)
    record=dict(results=results,scientific_hash=scientific_hash(results))
    (OUT/('EXPANSION_REPLAY.json' if replay else 'EXPANSION.json')).write_text(json.dumps(record,indent=2,sort_keys=True)+'\n')
    (OUT/('EXPANSION_REPLAY_TIMINGS.json' if replay else 'EXPANSION_TIMINGS.json')).write_text(json.dumps(perf|dict(total_seconds=time.perf_counter()-started),indent=2,sort_keys=True)+'\n')
if __name__=='__main__':run('--replay' in sys.argv)

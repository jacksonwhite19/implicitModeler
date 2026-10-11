"""Read-only timing of the real cold publication-input checks on retained L6.

No scientific payload write, state acceptance, trust shortcut or format change.
"""
import argparse
from contextlib import ExitStack
import cProfile
from pathlib import Path
import pstats
import sys
import time

V4=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(V4/'tools'),str(V4/'src')]
from benchmark_retained_l6 import read,totals
from path_preserving_deduplication import durable_json,file_hash
import check_recovered_authorization as preservation

OUT=V4/'results/preparation_snapshot_profile'
SOURCE=V4/'results/persistent_index_renewal/actual_prepare'
OWN=['research/sdf_cfd_mesher_v4/'+p for p in (
    'CURRENT_TASK.md','README.md','PREPARATION_SNAPSHOT_REPORT.md',
    'tools/profile_preparation_snapshots.py','tools/benchmark_actual_prepare.py',
    'tools/run_retained_front_publication.py','tests/test_retained_front_publication.py',
    'tools/finish_retained_l5.py','tools/collect_retained_workflow.py',
    'tools/audit_retained_publication.py',
    'tests/test_monitor_telemetry_retry.py',
    'results/preparation_snapshot_profile/','results/retained_l6_workflow/')]


def baseline():
    preservation.OUT=OUT;preservation.OWN=OWN;preservation.baseline()


def worker():
    import psutil
    import immutable_state_access as access
    import single_quality_component as quality
    import compact_front_atlas as atlas
    evidence=V4/'results/persistent_index_renewal/EVIDENCE.json'
    request=V4/'results/persistent_index_renewal/ACTUAL_REVIEW_REQUEST.json'
    started=time.perf_counter();measurements={}
    with ExitStack() as held:
        for p in (evidence,request):held.enter_context(access.guard(p))
        runtime=read(request)['runtime']
        for module in (quality,atlas):
            p=Path(module.__file__).resolve();held.enter_context(access.guard(p))
            if file_hash(p)!=runtime[str(p)]:raise ValueError('ChangedSnapshotImplementation',str(p))
        # Pin the retained file descriptors before cold authentication.
        originals={str(p):file_hash(p) for p in (SOURCE/'QUALITY_DELTA.json',SOURCE/'NEW_ATLAS.json')}
        for name,call in (
            ('quality',lambda:quality.snapshot(SOURCE/'QUALITY_DELTA.json',retain_verified=True)),
            ('atlas',lambda:atlas.snapshot(SOURCE/'NEW_ATLAS.json'))):
            profile=cProfile.Profile();tick=time.perf_counter();profile.enable()
            try:
                with call() as snap:
                    count=len(snap['rows'] if name=='quality' else snap['view'])
                    semantic=snap['semantic'];references=snap['references']
            finally:profile.disable()
            rows=[]
            for (path,line,function),(primitive,calls,total,cumulative,_) in pstats.Stats(profile).stats.items():
                rows.append(dict(file=path,line=line,function=function,primitive_calls=primitive,
                                 calls=calls,total_seconds=total,cumulative_seconds=cumulative))
            rows.sort(key=lambda r:r['total_seconds'],reverse=True)
            measurements[name]=dict(seconds=time.perf_counter()-tick,count=count,semantic=semantic,
                                    references=references,functions=rows[:60])
            durable_json(OUT/'PROFILE.json',measurements)
            print(name,measurements[name]['seconds'],count,flush=True)
        if any(file_hash(p)!=h for p,h in originals.items()):raise ValueError('ChangedSnapshotEnvelope')
        durable_json(OUT/'RESULT.json',dict(passed=True,measurements=measurements,
            seconds=time.perf_counter()-started,peak_working_set=psutil.Process().memory_info().peak_wset,
            input_envelopes=originals,request_sha256=file_hash(request),evidence_sha256=file_hash(evidence),
            no_state_write=True,no_new_layer=True,no_trust_change=True))


def finish():
    preservation.OUT=OUT;preservation.OWN=OWN;preservation.finish()


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('mode',choices=['baseline','worker','finish'])
    globals()[p.parse_args().mode]()

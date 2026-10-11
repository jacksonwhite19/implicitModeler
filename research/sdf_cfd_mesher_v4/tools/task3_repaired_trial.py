"""Recompute only the interrupted trial; check every preserved scientific field."""
from task3_root_regression import *
from task3_certificate_reproducer import reproduce
from task3_study import load
import psutil

def replay(context):
    old=saved();before=reproduce(context,old['field_index'],old['radius'])
    assert scientific_hash(before)==scientific_hash(old),'BlockedTrialScientificDrift'
    session=CertificateSession(4);a,b=pair_from_saved();result=session.pair(a,b)
    assert canonical_bytes(result)==canonical_bytes(session.pair(a,b))
    return dict(preserved_reproducer_sha256=hashlib.sha256((ROOT/'results/task3/CERTIFICATE_FAILURE_REPRODUCER.json').read_bytes()).hexdigest(),
                original_scientific_hash=scientific_hash(old),replayed_original_scientific_hash=scientific_hash(before),all_original_fields_identical=True,
                field=old['field'],radius=old['radius'],active_vertices=old['patch']['active'],candidate_gate=old['candidate_gate'],
                trial_coordinate_hash=scientific_hash(old['trial_coordinates']),projection_hash=scientific_hash(old['projection']),
                future_coordinate_hash=scientific_hash(old['changed_future_coordinates']),surfaces=old['surfaces'],
                old_certificate=old['result'],repaired_certificate=result,certificate_cache_hits=session.hits,certificate_cache_misses=session.misses,
                accepted=False,reason='Current numeric hard gates fail and the known future pair remains a forbidden exact intersection',
                new_intersections_assessed=False,new_intersection_scope='Only the original exact pair replayed; no wider acceptance certificate is needed for this rejected trial')

if __name__=='__main__':
    t=time.perf_counter();r=replay(load());write('BLOCKED_TRIAL_REPLAY.json',r)
    write('BLOCKED_TRIAL_TIMING.json',dict(seconds=time.perf_counter()-t,peak_rss=psutil.Process().memory_info().peak_wset))
    print(json.dumps(dict(hash=scientific_hash(r),classification=r['repaired_certificate']['classification'],original_identical=True,accepted=False)))

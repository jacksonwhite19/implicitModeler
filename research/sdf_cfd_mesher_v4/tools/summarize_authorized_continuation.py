"""Read-only reduction of the stopped campaign; no executor or repair calls."""
import json
from collections import Counter
from fractions import Fraction
from pathlib import Path
import sqlite3
import sys
import time

V4=Path(__file__).resolve().parents[1]
ROOT=V4.parents[1]
sys.path[:0]=[str(V4/'src'),str(V4/'tools')]
import numpy as np
from sdf_cfd_mesher_v4.common.authority import array_hash, file_hash, identity
from path_preserving_deduplication import durable_json
from composable_state import CommittedState

OUT=V4/'results/authorized_continuation'
RUN=OUT/'encoding_resume'
L2=RUN/'L2'
def read(p):return json.loads(Path(p).read_text(encoding='utf-8'))
def extrema(rows,key,limit,minimum=True):
    values=np.array([r[key] for r in rows])
    bad=(values<limit if minimum else values>limit)|~np.isfinite(values)
    return dict(min=float(values.min()),max=float(values.max()),violations=int(bad.sum()),count=len(values))


def main():
    start=time.perf_counter()
    stop=read(RUN/'STOP.json')
    if stop['stage']!='assembly' or stop['code']!="'final_incidence:1000000'":raise ValueError('DifferentStopRequiresInspection')
    generation=read(L2/'GENERATION.json');q=read(L2/'NEW_QUALITY.json');inc=read(L2/'AFFECTED_INCIDENCE.json')
    new=q['cells'];prior=[r for r in inc['cells'] if r['id']<2000000]
    cells=dict(volume=extrema(new,'volume',0),determinant=extrema(new,'determinant',.0011),aspect=extrema(new,'aspect',1000,False))
    cells['volume']['violations']=sum(r['volume']<=0 for r in new)
    cells['exact']=dict(count=len(q['exact']),nonpositive_jacobian=sum(r['jacobian_sign']!=1 for r in q['exact']),
        nonpositive_volume=sum(r['volume_sign']!=1 for r in q['exact']),
        jacobian_min=str(min(Fraction(r['jacobian']) for r in q['exact'])),
        volume_min=str(min(Fraction(r['signed_volume']) for r in q['exact'])))
    faces={k:extrema(q['faces'],k,v,mode) for k,v,mode in [('weight',.055,True),('skewness',3.63636,False),('symmetric_volume_ratio',.01,True)]}
    # Rows are actual shared faces. No invented top neighbour enters this check.
    caps=[r for r in q['faces'] if min(r['owners'])<2000000]
    cap_metrics={k:extrema(caps,k,v,mode) for k,v,mode in [('weight',.055,True),('skewness',3.63636,False),('symmetric_volume_ratio',.01,True)]}
    first_refs=np.load(V4/'results/live_sf2_first_layer/A/CANDIDATE.npz',allow_pickle=False)
    refs=first_refs['face_refs'];first_refs.close()
    regional={str(ref):dict(cells=int(np.sum(refs==ref)),determinant=extrema([r for r in new if refs[r['column']]==ref],'determinant',.0011),
        aspect=extrema([r for r in new if refs[r['column']]==ref],'aspect',1000,False)) for ref in np.unique(refs)}
    native=read(V4/'results/sealed_front_connection/WING_REFERENCE.json')
    with CommittedState(native) as base:
        component=base.wrapped.body['components']['quality']
        path=base.wrapped._object_path(component);base.wrapped._verify_read(path)
        with sqlite3.connect(path.as_uri()+'?mode=ro&immutable=1',uri=True) as db:
            counts={name:db.execute('SELECT COUNT(*) FROM receipts WHERE key GLOB ?',(name+'*',)).fetchone()[0]
                    for name in ('cell:','face:','final_incidence:','first_prism_incidence')}
        samples={}
        for r in (prior[0],min(prior,key=lambda r:r['determinant']),prior[-1]):
            key='cell:'+str(r['id']);old=base['quality'][key]
            samples[key]=dict(prior=old['value'],new_incidence=r,determinant_bits_equal=float(old['value']['determinant']).hex()==float(r['determinant']).hex())
        try:base['quality']['final_incidence:1000000']
        except KeyError:absent=True
        else:raise ValueError('MissingReceiptNoLongerReproduced')
        receipt=base['quality']['first_prism_incidence'];base.check()
        diagnostic=dict(scientific=base.scientific,quality_component=component,receipt_prefix_counts=counts,
            missing_key='final_incidence:1000000',missing_reproduced=absent,
            first_prism_incidence={k:v for k,v in receipt.items() if k!='versions'},
            receipt_identity=identity(receipt),versions_count=len(receipt['versions']),versions_identity=identity(receipt['versions']),
            samples=samples,guarded_reopen=True,
            scope='Read-only authenticated point probes and SQL key counts; no native restart, generation, reenrollment or synthesized receipt')
    durable_json(OUT/'ASSEMBLY_DIAGNOSIS_COMPACT.json',diagnostic)
    volume=read(L2/'VOLUME_DELTA.json');degree=volume['global_degree']
    volume=dict(exact_balance=volume['exact_balance'],new_signed_volume=volume['new_signed_volume'],
        new_boundary_volume=volume['new_boundary_volume'],degree_passed=degree['passed'],
        degree_counts=dict(Counter(str(r['degree']) for r in degree['values'])),unresolved=degree['unresolved'])
    certificate=read(L2/'FRESH_CERTIFICATE.json')
    certificate={k:certificate[k] for k in ('passed','required_pairs','prior_population','combined_pairs','failures','methods','proof_hash','unique_faces','prior_payloads_read','prior_payloads_recomputed')}
    with np.load(OUT/'L2/FORECAST.npz',allow_pickle=False) as original,np.load(L2/'FORECAST.npz',allow_pickle=False) as resumed:
        comparison={k:original[k].tobytes()==resumed[k].tobytes() for k in original.files}
    if not all(comparison.values()):raise ValueError('EncodingOnlyForecastChanged')
    summary=dict(classification='V4AcceptedFrontProductionAdmissionConnectedL2AssemblyBlocked',last_qualified_layer=1,
        stop=stop,generated=generation,cell_gates=cells,face_gates=faces,actual_prior_cap_metrics=cap_metrics,
        affected_prior_determinants=extrema(prior,'determinant',.0011),regional=regional,
        quality_passed=q['passed'],incidence_passed=inc['passed'],incidence_scope=inc['scope'],
        local_admission=read(L2/'ADMISSION.json')['decision'],localization=read(L2/'LOCALIZATION.json')['accounting'],
        certificate=certificate,volume_degree=volume,
        forecast_encoding_resume_comparison=comparison,
        completed_stages=['authenticated_access','production_binding','trajectory_premises','read_only_admission','generation_and_geometric_qualification'],
        incomplete_stages=['assembly','same_transition_replay','recursive_publication','new_state_guarded_reopen'],
        layers_3_to_5='Not started after required assembly evidence failure',
        reducer_seconds=time.perf_counter()-start)
    durable_json(OUT/'SUMMARY.json',summary)
    print(json.dumps({k:summary[k] for k in ('classification','cell_gates','face_gates','affected_prior_determinants','regional','localization','volume_degree','reducer_seconds')},indent=2))

if __name__=='__main__':main()

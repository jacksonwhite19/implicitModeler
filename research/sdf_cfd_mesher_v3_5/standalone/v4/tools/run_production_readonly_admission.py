"""Verify sealed evidence and independently restore L1 for each admission run."""
import json,time,sys
import numpy as np
import psutil
from capsule_layer2_preflight import V4,L1,SURFACE,REQUEST,file_hash,array_hash,identity
from run_production_l2_risk_screen import load_screen_inputs
from run_production_collision_localization import save
from production_surface_authority import load_production_wall
from production_readonly_admission import evaluate,REQUIRED

OUT=V4/'results/production_layer2_readonly_admission'
LOC=V4/'results/production_layer2_collision_localization'
DIRECT=V4/'results/production_layer2_risk_coverage'


def verified(folder,name,digest):
    if file_hash(folder/name)!=digest:raise ValueError('FileAuthorityMismatch:'+name)
    return json.loads((folder/name).read_text())


def run(label):
    started=time.perf_counter()
    auth=verified(LOC,'AUTHORITY.json','48ff989f17ee7cc26b20e03990ccf160a2f8d0bccacda0e09f12f5c78fc00dab')
    if file_hash(LOC/'report.md')!='509c0f1b714bc38951f450227c5e415eda4940780e76d58e560445b72acea598':raise ValueError('ReportMismatch')
    for name,h in auth['evidence'].items():
        if file_hash(LOC/name)!=h:raise ValueError('LocalizationEvidenceMismatch:'+name)
    assert auth['scientific_identity']=='6c7ecafd9a176bbd47888a5a1562923f0a482732e038a467260246a914bcde09'
    assert auth['hashes']['proof']=='88d620eb2c45d37875c2dda1adb16203982d7df6c15cc1e878056b9e271304cf'
    loc=json.loads((LOC/'RUN_A.json').read_text())['scientific']
    da=verified(DIRECT,'AUTHORITY.json','cf91b01f2a5a7ab019a9288bcb79dd2d7707ac91b84b5f4fc84e546d2c5bfbd7')
    direct=verified(DIRECT,'RUN_A.json',da['evidence']['RUN_A.json'])['scientific']
    gates=verified(DIRECT,'GATES.json',da['evidence']['GATES.json'])
    if file_hash(DIRECT/'SCREEN_ARRAYS.npz')!=da['evidence']['SCREEN_ARRAYS.npz']:raise ValueError('DirectBulkMismatch')
    adapter,high,times,before=load_screen_inputs()
    t=time.perf_counter()
    if adapter.binding!=loc['binding'] or adapter.binding!=direct['binding']:raise ValueError('CrossEvidenceBindingMismatch')
    if array_hash(high)!=direct['provisional_l2_coordinates']:raise ValueError('CandidateMismatch')
    with np.load(DIRECT/'SCREEN_ARRAYS.npz') as z:
        metrics={g:{k.split('__',1)[1]:array_hash(z[k]) for k in z.files if k.startswith(g+'__')} for g in ('cells','sides','caps')}
        assert identity(metrics)==direct['metric_identity']=='e794c86651b85926bac8b8da2804b856b48ce7678d4a21020d235ee767de16c0'
        assert array_hash(z['candidate_pairs'])==loc['candidate_identity']
    with np.load(LOC/'PROOF_ROWS.npz') as z:
        for k,h in loc['array_hashes'].items():
            if array_hash(z[k])!=h or len(z[k])!=2898957:raise ValueError('ProofPopulationMismatch')
        assert int(((z['codes']>0)&(z['codes']<10)).sum())==loc['accounting']['certified']
    with np.load(V4/'results/production_layer2_forecast_admission/CHEAP_FORECAST.npz') as z:
        cheap=dict(nonfinite_coordinate=int((~np.isfinite(z['vertex__coordinates'])).any(2).sum()),
            sdf_evaluation_failure=0,zero_required_gradient=int((z['vertex__gradient_norm']==0).sum()),projection_failure=0,
            zero_area_triangle=int((~z['triangle__nondegenerate']).sum()),orientation_reversal=int(z['triangle__orientation_reversal'].sum()),invalid_topology_reference=0)
    n=len(adapter.topology.triangles);assert n==33786
    counts=direct['counts'];ownership=direct['ownership']
    complete=dict(restored_authority=True,request_schedule=True,cheap_forecast=True,
        direct_population=counts['provisional_prisms']==counts['finite_prisms']==ownership['screened_columns']==ownership['unique_column_ids']==n,
        direct_metrics=not any(gates['failures'].values()) and gates['finite_geometry_failures']==0,
        topology_ownership=not direct['topology']['events'] and not any(ownership[k] for k in ('missing','duplicate','orphan','foreign')),
        collision_inventory=loc['accounting']['candidates']==2898957,collision_proofs=True,
        risk_supports=not loc['risk_rows'] and not loc['seed_columns'] and not loc['closure']['groups'])
    evidence=dict(binding=adapter.binding,increment_hex=float(adapter.remaining_increments[0]).hex(),completeness=complete,
        cheap_hard_events=cheap,direct_hard_events=direct['hard_events'],collision_accounting=loc['accounting'],
        risk_seeds=loc['seed_columns'],patches=loc['closure']['groups'],mandatory_detail_requirements=[],
        authorities=dict(cheap='cecbf2877e70d9a17d63ca00277ec66761e0d6094403a90a472fb6159d5f5b10',direct=direct['metric_identity'],
            hard_rows=direct['hard_events_identity'],localization=auth['scientific_identity'],proof=auth['hashes']['proof'],
            candidates=loc['candidate_identity'],risk=loc['risk_identity'],supports=loc['closure']['support_population_identity']))
    times['bound_evidence_verification']=time.perf_counter()-t;t=time.perf_counter()
    result=evaluate(evidence,identity(evidence),adapter.binding,adapter.front.coordinates,adapter.topology,high)
    times['admission_evaluation']=time.perf_counter()-t
    if not result['generation_attempt_authorized']:raise ValueError(result)
    wall,tri,surface=load_production_wall(SURFACE)
    if surface!=before['surface'] or file_hash(REQUEST)!=before['request'] or file_hash(L1/'A/checkpoints/CURRENT.json')!=before['pointer']:raise ValueError('InputMutation')
    scientific=dict(evidence=evidence,evidence_identity=identity(evidence),decision=result,decision_identity=identity(result),
        restored_prefix=array_hash(adapter.committed_prefix),preservation=dict(wall=True,L1=True,request=True),
        publication=dict(L2_generated=False,L2_committed=False,L3_generated=False,core_generated=False,authoritative_layer=1))
    output=dict(scientific=scientific,scientific_identity=identity(scientific),timings=times,total_seconds=time.perf_counter()-started,
        peak_rss=psutil.Process().memory_info().peak_wset,retained_rss=psutil.Process().memory_info().rss)
    if label=='RUN_B' and json.loads((OUT/'RUN_A.json').read_text())['scientific']!=scientific:raise ValueError('ReplayMismatch')
    save(OUT/(label+'.json'),output);print(json.dumps({k:v for k,v in output.items() if k!='scientific'}),flush=True)


if __name__=='__main__':run(sys.argv[1])

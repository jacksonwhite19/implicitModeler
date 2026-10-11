from task4h_common import *
import psutil
import task4_prepare_next as prod
from sdf_cfd_mesher_v4.advancing_front.validation_evidence import input_identity,verify_receipt,make_receipt,resolve_receipt
from sdf_cfd_mesher_v4.advancing_front.final_validation_reuse import final_receipt_manifest
from sdf_cfd_mesher_v4.advancing_front.rolling_state import CommitStore
from sdf_cfd_mesher_v4.advancing_front.rolling_metrics import sdf_diagnostic_record
from sdf_cfd_mesher_v4.certificate.production import verify_committed_layer

def run():
    require_time();start=time.perf_counter();front=prod.load_sf2_wall();top=prod.build_topology(front.triangles,len(front.coordinates));schedule=prod.load_schedule();sdf=prod.NativeSdf()
    t=time.perf_counter();state=CommitStore(BASE/'build_A',front,top,schedule).restart();assert state.layer==20;chain_seconds=time.perf_counter()-t
    t=time.perf_counter();prefix=[front.coordinates]+[np.load(BASE/'build_A'/f'FRONT_{k:02}.npy') for k in range(1,21)]
    cps=[];conds=[];sdfs=[];receipts=[]
    for k in range(1,21):
        cps.append(read(BASE/'build_A'/f'LAYER_{k:02}.json'))
        with np.load(BASE/f'LAYER_{k:02}_CONDITIONING.npz') as z:conds.append({key:z[key] for key in z.files})
        sdfs.append(read(BASE/f'LAYER_{k:02}_SDF.json'));receipts.append(read(ROOT/f'results/task4g/validation/LAYER_{k:02}.json'))
    load_seconds=time.perf_counter()-t;audits=[]
    for replay in range(2):
        require_time();t=time.perf_counter();inputs=[]
        for i in range(20):
            inp=input_identity(top,prefix[i],prefix[i+1],None if i==0 else prefix[i-1],i+1,schedule.targets,sdf.source_identity);inputs.append(inp)
            verify_receipt(receipts[i],top,inp,cps[i],conds[i],sdfs[i])
        receipt_seconds=time.perf_counter()-t;t=time.perf_counter();manifest=final_receipt_manifest(receipts,top,schedule.targets);coverage_seconds=time.perf_counter()-t
        t=time.perf_counter();sigma=np.stack([r['sigma'] for r in conds]);gain=np.stack([r['G_bound'] for r in conds]);final=read(BASE/'FINAL_VALIDATION.json')
        assert [r['gate'] for r in receipts]==final['gates'];assert float(sigma.min())==final['sigma_minimum'];assert float(np.nanmax(gain))==final['G_bound_maximum']
        assert [r['sdf']['maximum_residual'] for r in receipts]==[r['maximum'] for r in final['sdf_residuals']]
        with np.load(BASE/'FINAL_CONDITIONING.npz') as z:assert np.array_equal(sigma,z['sigma']) and np.array_equal(gain,z['G_bound'],equal_nan=True)
        global_seconds=time.perf_counter()-t
        # Complete exact proof audit over preserved geometry, no new solve/build.
        t=time.perf_counter();proof_rows=[]
        for layer in range(1,21):
            require_time();cp=cps[layer-1];count,perf=verify_committed_layer(front,top,prefix[1:layer+1],BASE/f'certificate_layer_{layer:02}',cp['certificate'])
            proof_rows.append(dict(layer=layer,pairs=count,certificate_hash=scientific_hash(cp['certificate'])))
        certificate_seconds=time.perf_counter()-t
        assert sum(r['pairs'] for r in proof_rows)==final['certificate']['candidate_pairs']
        scientific=dict(manifest=manifest,proofs=proof_rows,gates=final['gates'],sdf=sdfs,sigma_hash=array_hash(sigma),gain_hash=array_hash(gain),qualification_passed=True)
        audits.append(dict(replay=replay+1,receipt_seconds=receipt_seconds,coverage_seconds=coverage_seconds,global_aggregation_seconds=global_seconds,certificate_seconds=certificate_seconds,scientific_hash=scientific_hash(scientific),reused_layers=20,numeric_calls=0,sdf_calls=0,exact_jacobian_calls=0,new_pair_solves=0))
        save(f'FINAL_MANIFEST_{replay+1}.json',manifest);save('FINAL_REUSE.json',dict(audits=audits,load_seconds=load_seconds,chain_seconds=chain_seconds,total_seconds=time.perf_counter()-start,peak_rss=psutil.Process().memory_info().peak_wset))
        print('FINAL_REUSE',replay+1,audits[-1],flush=True)
    assert audits[0]['scientific_hash']==audits[1]['scientific_hash']
    # Recompute a missing late-layer receipt in this study's directory only.
    require_time();i=15;calls=[]
    def recompute():
        calls.append(i+1);cq=prod.measure.__globals__['legacy_modules']()['component_quality'];C,V=cq.FG.prism_cell_centres(np.concatenate((prefix[i-1][top.triangles],prefix[i][top.triangles]),axis=1))
        r=prod.evaluate_layer(top,prefix[i],prefix[i+1],i+1,dict(centre=C,volume=V));q=sdf.evaluate(prefix[i+1]);assert np.all(abs(q.phi-schedule.targets[i])<=q.source_error_bound)
        cond={key:r['cells'][key] for key in conds[i]};assert all(np.array_equal(cond[k],conds[i][k],equal_nan=True) for k in cond)
        return make_receipt(top,inputs[i],r['gate'],sdf_diagnostic_record(q,schedule.targets[i],sdfs[i]),cps[i]['exact'],cps[i]['certificate'],cond)
    t=time.perf_counter();r,hit,reason=resolve_receipt(OUT/'MISSING_LAYER_16.json',top,inputs[i],cps[i],conds[i],sdfs[i],recompute)
    assert not hit and calls==[16] and r==receipts[i]
    save('FINAL_MISSING_RECEIPT.json',dict(recomputed_layers=calls,exact_equal=True,seconds=time.perf_counter()-t,reason=reason))
if __name__=='__main__':run()

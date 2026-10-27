"""Qualify immutable receipt coverage on preserved completed A2, without a build.

Old receipts are not relabeled as new implementation evidence. Each numeric/SDF
layer is evaluated once, matched to the sealed results, then reused by identity.
The existing exact sign/certificate records remain bound to their committed
checkpoint; no new certificate mathematics or proof claims are introduced.
"""
from task4g_common import *
import task4_prepare_next as prod
from sdf_cfd_mesher_v4.advancing_front.validation_evidence import input_identity,make_receipt,resolve_receipt
from sdf_cfd_mesher_v4.advancing_front.rolling_metrics import sdf_diagnostic_record
from task4f_instrument import same_arrays
import psutil

def run():
    require_time();start=time.perf_counter();front=prod.load_sf2_wall();top=prod.build_topology(front.triangles,len(front.coordinates));schedule=prod.load_schedule();sdf=prod.NativeSdf()
    prefix=[front.coordinates]+[np.load(BASE/'build_A'/f'FRONT_{k:02}.npy') for k in range(1,21)]
    rows=[];receipts=[];previous=None;inputs=[];cps=[];conds=[];sdfs=[]
    for layer in range(1,21):
        require_time();low=prefix[layer-1];high=prefix[layer];cp=read(BASE/'build_A'/f'LAYER_{layer:02}.json')
        inp=input_identity(top,low,high,None if layer==1 else prefix[layer-2],layer,schedule.targets,sdf.source_identity)
        t=time.perf_counter();r=prod.evaluate_layer(top,low,high,layer,previous);numeric_seconds=time.perf_counter()-t
        assert r['gate']==cp['gate']==read(BASE/f'LAYER_{layer:02}_NUMERIC.json')['gate'],(layer,'ImmutableNumericMismatch')
        with np.load(BASE/f'LAYER_{layer:02}_CONDITIONING.npz') as z:cond={k:z[k] for k in z.files}
        assert all(same_arrays(v,r['cells'][k]) for k,v in cond.items()),(layer,'ImmutableConditioningMismatch')
        t=time.perf_counter();q=sdf.evaluate(high);assert np.all(abs(q.phi-schedule.targets[layer-1])<=q.source_error_bound)
        sd=sdf_diagnostic_record(q,schedule.targets[layer-1],read(BASE/f'LAYER_{layer:02}_SDF.json'));sdf_seconds=time.perf_counter()-t
        receipt=make_receipt(top,inp,r['gate'],sd,cp['exact'],cp['certificate'],cond)
        save(f'validation/LAYER_{layer:02}.json',receipt);receipts.append(receipt);inputs.append(inp);cps.append(cp);conds.append(cond);sdfs.append(sd)
        rows.append(dict(layer=layer,numeric_seconds=numeric_seconds,sdf_seconds=sdf_seconds,receipt_hash=receipt['receipt_hash'],bitwise_metrics_equal=True,conditioning_equal=True,sdf_equal=True))
        previous=r['cells'];print('VALIDATION_ESTABLISHED',layer,flush=True)
    hashes=[]
    for replay in range(2):
        t=time.perf_counter();coverage=[]
        for k in range(20):
            def forbidden():raise AssertionError('UnexpectedRecalculation')
            # Rehash real coordinate and topology arrays on each audit.
            inp=input_identity(top,prefix[k],prefix[k+1],None if k==0 else prefix[k-1],k+1,schedule.targets,sdf.source_identity)
            r,hit,_=resolve_receipt(OUT/f'validation/LAYER_{k+1:02}.json',top,inp,cps[k],conds[k],sdfs[k],forbidden);assert hit
            coverage.append(dict(layer=k+1,receipt_hash=r['receipt_hash'],populations=r['populations'],certificate_hash=r['certificate_hash']))
        elapsed_seconds=time.perf_counter()-t;hashes.append(scientific_hash(coverage))
        save(f'VALIDATION_REPLAY_{replay+1}.json',dict(complete=True,coverage=coverage,scientific_hash=hashes[-1],seconds=elapsed_seconds,reused_layers=20,recomputed_layers=0))
    assert hashes[0]==hashes[1]
    # Real missing receipt fallback on a passive late layer, with no full rerun.
    k=15;calls=[]
    def recompute():
        calls.append(1);cq=prod.measure.__globals__['legacy_modules']()['component_quality']
        C,V=cq.FG.prism_cell_centres(np.concatenate((prefix[k-1][top.triangles],prefix[k][top.triangles]),axis=1))
        r=prod.evaluate_layer(top,prefix[k],prefix[k+1],k+1,dict(centre=C,volume=V));q=sdf.evaluate(prefix[k+1]);assert np.all(abs(q.phi-schedule.targets[k])<=q.source_error_bound)
        assert r['gate']==cps[k]['gate'];return make_receipt(top,inputs[k],r['gate'],sdf_diagnostic_record(q,schedule.targets[k],sdfs[k]),cps[k]['exact'],cps[k]['certificate'],conds[k])
    t=time.perf_counter();result,hit,reason=resolve_receipt(OUT/'missing_fixture/LAYER_16.json',top,inputs[k],cps[k],conds[k],sdfs[k],recompute)
    assert not hit and calls==[1] and result==receipts[k]
    save('FINAL_VALIDATION_REUSE.json',dict(rows=rows,replay_hashes=hashes,missing_recomputed_layers=1,missing_reason=reason,missing_seconds=time.perf_counter()-t,
        cold_numeric_sdf_seconds=sum(r['numeric_seconds']+r['sdf_seconds'] for r in rows),total_seconds=time.perf_counter()-start,peak_rss=psutil.Process().memory_info().peak_wset,
        exact_certificate='Existing committed and final proof artifacts unchanged. Production final completeness audit remains required and is not elided.',
        geometry_rebuilt=False))
if __name__=='__main__':
    (OUT/'missing_fixture').mkdir(exist_ok=True,parents=True);run()

"""Complete finished-shell validation; refuses an incomplete committed prefix."""
from task4_certify_candidate import *
from sdf_cfd_mesher_v4.certificate.rolling import owned_scope
from sdf_cfd_mesher_v4.certificate.production import verify_committed_layer
from sdf_cfd_mesher_v4.certificate.incremental import ProofCache
from sdf_cfd_mesher_v4.advancing_front.private_rolling import restore_exact_proofs
from sdf_cfd_mesher_v4.forecast.detailed.continuous import SideIndex
from types import SimpleNamespace
from sdf_cfd_mesher_v4.advancing_front.rolling_metrics import sdf_diagnostic_record
from sdf_cfd_mesher_v4.advancing_front.validation_evidence import input_identity,make_receipt,resolve_receipt
from sdf_cfd_mesher_v4.advancing_front.final_validation_reuse import final_receipt_manifest

def run():
    start=time.perf_counter();front=load_sf2_wall();top=build_topology(front.triangles,len(front.coordinates));schedule=load_schedule();sdf=NativeSdf()
    store=CommitStore(OUT/'build_A',front,top,schedule);state=store.restart()
    if state.layer!=20:raise ValueError('FinalValidationRequiresAllTwentyCommittedLayers')
    from task4_verify import run as verify_prefix
    audit=verify_prefix()
    prefix=np.stack([np.load(store.path/f'FRONT_{k:02}.npy') for k in range(1,21)])
    low=front.coordinates;previous=None;gates=[];sigma=[];gain=[];residuals=[];timings=[];proofs=[];sdf_records=[];validation_receipts=[];verified_receipts=[]
    for layer,high in enumerate(prefix,1):
        started=time.perf_counter();numeric_seconds=sdf_seconds=0.
        checkpoint=json.loads((store.path/f'LAYER_{layer:02}.json').read_text())
        prior_low=None if layer==1 else front.coordinates if layer==2 else prefix[layer-3]
        inputs=input_identity(top,low,high,prior_low,layer,schedule.targets,sdf.source_identity)
        try:
            with np.load(OUT/f'LAYER_{layer:02}_CONDITIONING.npz') as z:conditioning={k:z[k] for k in ('sigma','G_bound','G_bound_available')}
        except (FileNotFoundError,KeyError,ValueError):conditioning={}
        sdf_path=OUT/f'LAYER_{layer:02}_SDF.json';old_sdf=read(sdf_path) if sdf_path.exists() else {}
        def recompute_validation():
            nonlocal numeric_seconds,sdf_seconds
            prior=None
            if layer>1:
                from sdf_cfd_mesher_v4.runtime.frozen_imports import legacy_modules
                C,V=legacy_modules()['component_quality'].FG.prism_cell_centres(np.concatenate((prior_low[top.triangles],low[top.triangles]),axis=1))
                prior=dict(centre=C,volume=V)
            r=evaluate_layer(top,low,high,layer,prior);numeric_seconds=r['seconds']
            # Missing sidecar evidence is repaired from this immutable layer.
            # The checkpoint's scientific numeric gate must still match below.
            conditioning.clear();conditioning.update({k:r['cells'][k] for k in ('sigma','G_bound','G_bound_available')})
            t=time.perf_counter();q=sdf.evaluate(high);assert np.all(abs(q.phi-schedule.targets[layer-1])<=q.source_error_bound);sdf_seconds=time.perf_counter()-t
            record=sdf_diagnostic_record(q,schedule.targets[layer-1])
            old_sdf.clear();old_sdf.update(record)
            return make_receipt(top,inputs,r['gate'],record,checkpoint['exact'],checkpoint['certificate'],conditioning)
        receipt,reused,reason=resolve_receipt(OUT/f'LAYER_{layer:02}_VALIDATION_RECEIPT.json',top,inputs,checkpoint,conditioning,old_sdf,recompute_validation)
        if not reused:
            # Publish repaired sidecars only after the receipt matches the
            # immutable checkpoint. A failed recomputation changes no evidence.
            np.savez_compressed(OUT/f'LAYER_{layer:02}_CONDITIONING.npz',**conditioning)
            write(sdf_path,receipt['sdf'])
        sigma.append(conditioning['sigma']);gain.append(conditioning['G_bound']);gates.append(receipt['gate']);sdf_records.append(receipt['sdf'])
        residuals.append(dict(layer=layer,maximum=receipt['sdf']['maximum_residual']))
        validation_receipts.append(dict(layer=layer,receipt_hash=receipt['receipt_hash'],reused=reused,recompute_reason=reason,populations=receipt['populations']))
        verified_receipts.append(receipt)
        # Re-enumerate the full required scope and bind every preserved exact
        # proof to finished-shell coordinates. No pair acceptance is inferred
        # from floating-point distances or from an empty cache entry.
        ledger=read(OUT/f'LAYER_{layer:02}_CERTIFICATE.json');assert ledger['complete']
        checkpoint=json.loads((store.path/f'LAYER_{layer:02}.json').read_text())
        assert ledger==checkpoint['certificate'],'FinalProofLedgerDiffersFromCommittedAuthority'
        certificate_started=time.perf_counter()
        with MemorySampler() as memory:
            count,verification_performance=verify_committed_layer(front,top,prefix[:layer],OUT/f'certificate_layer_{layer:02}',ledger,workers=12)
        certificate_seconds=time.perf_counter()-certificate_started
        proofs.append(dict(layer=layer,pairs=count,certificate_sha256=scientific_hash(ledger)))
        low=high;timings.append(dict(layer=layer,seconds=time.perf_counter()-started,numeric_seconds=numeric_seconds,sdf_seconds=sdf_seconds,validation_receipt_reused=reused,certificate_seconds=certificate_seconds,proof_verification=verification_performance,observed_tree_peak_rss=memory.peak));print('FINAL_VALIDATED',layer,count,flush=True)
    immutable_manifest=final_receipt_manifest(verified_receipts,top,schedule.targets)
    sigma=np.stack(sigma);gain=np.stack(gain);i=np.unravel_index(np.argmin(sigma),sigma.shape);j=np.unravel_index(np.nanargmax(gain),gain.shape)
    final_certificate=dict(complete=True,method='CompleteDisjointImmediateProofUnionWithFinishedGeometryAndPairReenumeration',layers=proofs,
        candidate_pairs=sum(p['pairs'] for p in proofs),completed_pairs=sum(p['pairs'] for p in proofs),forbidden=[],unresolved=[],invalid_surfaces=[])
    np.savez_compressed(OUT/'FINAL_WALL_PRISM_SHELL.npz',coordinates=np.concatenate((front.coordinates[None],prefix)),triangles=top.triangles)
    column_minimum=sigma.min(axis=0);column_order=np.argsort(column_minimum,kind='stable')
    np.savez_compressed(OUT/'FINAL_CONDITIONING.npz',sigma=sigma,G_bound=gain,column_minimum_sigma=column_minimum,
        column_minimum_layer=np.argmin(sigma,axis=0)+1,sorted_column_ids=column_order)
    write(OUT/'FINAL_SDF_EVALUATIONS.json',sdf_records)
    write(OUT/'FINAL_VALIDATION_COVERAGE.json',dict(complete=immutable_manifest['complete'],receipts=validation_receipts,immutable_manifest=immutable_manifest,
        reused_layers=sum(r['reused'] for r in validation_receipts),recomputed_layers=sum(not r['reused'] for r in validation_receipts),
        continuous_completeness='Separately re-enumerated and verified by the qualified final certificate'))
    histories=[read(OUT/'CORRECTION_HISTORY.json')]+[read(OUT/f'LAYER{k}_SEARCH.json') for k in range(2,21)]
    for history in histories:validate_admission_history(history['history'])
    result=dict(wall_vertices=len(front.coordinates),columns=len(top.triangles),layers=20,prisms_per_layer=len(top.triangles),prisms=20*len(top.triangles),side_surfaces=20*len(top.edges),
        complete_transaction_history_sha256=scientific_hash(histories),
        initial_forecast_scientific_sha256=scientific_hash(read(OUT/'INITIAL_SCAN.json')),
        initial_future_spots_sha256=scientific_hash(read(OUT/'LAYER1_FUTURE_SPOTS.json')),
        sdf_layer_evaluations_sha256=scientific_hash(sdf_records),
        topology_validation={key:audit[key] for key in ('two_owners_per_edge','opposite_edge_orientation','duplicate_triangles','unused_vertices','unique_source_ids','source_ancestry_present')},
        wall_coordinate_hash=front.coordinate_hash,geometry_sha256=front.geometry_authority,surface_sha256=front.surface_authority,schedule_sha256=array_hash(schedule.targets),
        internal_cap_faces=19*len(top.triangles),boundary_wall_faces=len(top.triangles),boundary_outer_faces=len(top.triangles),unique_faces=20*len(top.edges)+21*len(top.triangles),
        coordinate_hash=array_hash(prefix),topology_hash=top.sha256,connectivity_chain=state.prism_hash,transaction_state_hash=state.state_hash,
        gates=gates,certificate=final_certificate,sdf_residuals=residuals,sigma_minimum=float(sigma[i]),sigma_location=dict(layer=int(i[0]+1),column=int(i[1]),wall_centroid=front.coordinates[top.triangles[i[1]]].mean(0)),
        positive_sigma_minimum=float(sigma[sigma>0].min()),G_bound_maximum=float(gain[j]),G_bound_location=dict(layer=int(j[0]+1),column=int(j[1]),wall_centroid=front.coordinates[top.triangles[j[1]]].mean(0)),
        sigma_quantiles={str(q):float(np.quantile(sigma,q)) for q in (0,.0001,.001,.01,.05,.5,1)},
        column_minimum_sigma_quantiles={str(q):float(np.quantile(column_minimum,q)) for q in (0,.0001,.001,.01,.05,.5,1)},
        conditioning_hash=scientific_hash(dict(sigma=sigma,G_bound=gain)))
    result['shell_scientific_hash']=scientific_hash(result);write(OUT/'FINAL_VALIDATION.json',result)
    (OUT/'FINAL_VALIDATION_TIMINGS.json').write_text(json.dumps(dict(stages=timings,total_seconds=time.perf_counter()-start,peak_rss=psutil.Process().memory_info().peak_wset),indent=2)+'\n')
    print('FINAL_SHELL',result['shell_scientific_hash'],flush=True)

if __name__=='__main__':run()

"""Post-build native replay, semantic proof comparison and compact ownership audit."""
import gc,json,time,subprocess,hashlib
from pathlib import Path
import numpy as np,psutil
from summarize_native_memory import ROOT,OUT,load,save
from native_memory_observer import allocations

def run():
    from qualify_native_sf2_manifest import SOURCE
    from sdf_cfd_mesher_v4.advancing_front.native_checkpoint import NativeCheckpointStore
    from sdf_cfd_mesher_v4.advancing_front.native_front_providers import NativeRiskAtlas
    from sdf_cfd_mesher_v4.advancing_front.mixed_front import scientific_hash
    from sdf_cfd_mesher_v4.advancing_front.production_authority import configuration
    from sdf_cfd_mesher_v4.certificate.native_manifest import NativeManifest
    comparison=load('fresh/SCIENTIFIC_COMPARISON.json');assert comparison['passed'] and comparison['remesh_events']==0
    config=configuration();assert config['runner']=='native-mixed-front' and config['automatic_remesh']
    old=json.loads((ROOT/'results/native_sf2_production_plumbing/fresh/FINAL_NATIVE_AUDIT.json').read_text())
    final=load('fresh/FINAL_NATIVE_AUDIT.json')
    assert final['wall']==old['wall'] and final['first_layer']==old['first_layer']
    tick=time.perf_counter();process=psutil.Process();base=process.memory_info().rss
    store=NativeCheckpointStore(OUT/'fresh/checkpoints');state=store.restart();restart_rss=process.memory_info().rss
    digest=scientific_hash(state);restart_seconds=time.perf_counter()-tick
    atlas=NativeRiskAtlas(state,store.objects);atlas_hash=atlas.scientific_hash();forecast=atlas.native_forecast.scientific()
    provider_rss=process.memory_info().rss
    ownership=allocations(dict(state=state,atlas=atlas));del atlas,state;gc.collect()
    state=store.restart();again=NativeRiskAtlas(state,store.objects)
    assert digest==scientific_hash(state) and atlas_hash==again.scientific_hash() and forecast==again.native_forecast.scientific()
    assert not state['events']
    manifest=NativeManifest(OUT/'fresh/native_manifest');meta=manifest.metadata();proofs=[]
    fields=('pair_hash','code_hash','witness_hash','outcome_hash','fallback_hash','validity_hash','geometry_hash','labels_hash','candidate_pairs','completed_pairs','exact_calls','class_counts','validity_fallback','kernel','ownership','complement')
    for block in meta['blocks']:
        k=block['layer'];reference=json.loads((SOURCE/f'build_A/LAYER_{k:02}.json').read_text())['certificate'];actual=block['ledger']
        assert all(reference[f]==actual[f] for f in fields),(k,[f for f in fields if reference[f]!=actual[f]])
        proofs.append(dict(layer=k,semantic_fields_equal=list(fields),source_identity_changed=reference['source_identity']!=actual['source_identity']))
    save('FRESH_REPLAY.json',dict(passed=True,scientific=digest,atlas=atlas_hash,forecast_equal=True,wall_exact=True,first_layer_exact=True,remesh_events=0,certificate_semantics=proofs))
    save('FRESH_NATIVE_RESIDENCY.json',dict(process_start_rss=base,restart_rss=restart_rss,restart_seconds=restart_seconds,with_atlas_forecast_rss=provider_rss,reachable=ownership,manifest_index=allocations(meta),manifest_disk_bytes=sum(p.stat().st_size for p in manifest.path.rglob('*') if p.is_file())))
    # Storage adapter source changes are explicit; mathematical sources do not.
    repo=ROOT.parents[1];names=['certificate/efficient_native.rs','certificate/efficient_proof.py','certificate/exact_preparation.py','certificate/parallel_screen.py','forecast/detailed/scientific.py','advancing_front/termination_policy.py','advancing_front/production_authority.py']
    rows=[]
    for name in names:
        path=ROOT/'src/sdf_cfd_mesher_v4'/name;relative=path.relative_to(repo).as_posix()
        prior=subprocess.check_output(['git','show','e1ad4dcc847c70cf15700c3a4091e6d395acd5e9:'+relative])
        assert prior==path.read_bytes(),name
        rows.append(dict(path=relative,sha256=hashlib.sha256(prior).hexdigest(),unchanged=True))
    save('FROZEN_SCIENCE_AUDIT.json',dict(passed=True,sources=rows,production_authority=config))
    print('FRESH_NATIVE_REPLAY_AND_SEMANTIC_PROOFS_PASS',flush=True)
if __name__=='__main__':run()

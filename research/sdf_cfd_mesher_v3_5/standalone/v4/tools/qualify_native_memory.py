"""Memory-task bounded scientific regression; sealed evidence is read-only."""
import sys,json,time,gc
from pathlib import Path
import numpy as np,psutil
from sdf_cfd_mesher_v4.common.authority import file_hash
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'results/native_runner_memory_profile'
def sf2():
    from native_sf2_artifacts import materialize
    from sdf_cfd_mesher_v4.advancing_front.native_checkpoint import NativeCheckpointStore
    from sdf_cfd_mesher_v4.advancing_front.native_front_providers import NativeRiskAtlas
    from sdf_cfd_mesher_v4.advancing_front.native_prism_validation import audit_native_prisms
    from sdf_cfd_mesher_v4.advancing_front.mixed_front import scientific_hash
    from sdf_cfd_mesher_v4.certificate.native_manifest import NativeManifest
    prior=ROOT/'results/native_sf2_production_plumbing';rows=[]
    with materialize('native_manifest') as proofs:
        manifest=NativeManifest(proofs)
        for k in (1,5,11,12,20):
            tick=time.perf_counter()
            with materialize(f'layer_{k:02}') as source:
                store=NativeCheckpointStore(source);state=store.restart();atlas=NativeRiskAtlas(state,source/'objects')
                ref=json.loads((prior/f'LAYER_{k:02}_PREQUALIFICATION.json').read_text())
                digest=scientific_hash(state);forecast=atlas.native_forecast.scientific()
                assert digest==ref['scientific'] and atlas.scientific_hash()==ref['atlas'] and forecast==ref['forecast']
                audit=audit_native_prisms(state,source/'objects',manifest)
                assert audit==ref['audit'] and not state['events']
                del state,atlas;gc.collect();state=store.restart()
                assert scientific_hash(state)==digest
                rows.append(dict(layer=k,passed=True,scientific=digest,atlas=ref['atlas'],forecast_equal=True,audit=audit,restart_exact=True,remesh_events=0,seconds=time.perf_counter()-tick,rss=psutil.Process().memory_info().rss))
                del state;gc.collect()
            print('BOUNDED_PASS',k,flush=True)
    (OUT/'BOUNDED_SF2.json').write_text(json.dumps(rows,indent=2)+'\n')
def general():
    from migrate_general_native import HistoricalProvider
    from sdf_cfd_mesher_v4.advancing_front.mixed_front_runner import MixedFrontRunner
    from sdf_cfd_mesher_v4.advancing_front.mixed_front import scientific_hash
    from sdf_cfd_mesher_v4.advancing_front.native_checkpoint import NativeCheckpointStore
    tick=time.perf_counter();folder=OUT/'general_regression';provider=HistoricalProvider(folder)
    initial=provider.initialize();runner=MixedFrontRunner(folder/'checkpoints',provider);runner.initialize(initial)
    result=runner.step();final=runner.store.restart();digest=scientific_hash(final)
    assert result['operation']=='RollbackRemesh',result
    assert initial['wall']==final['wall'] and initial['first_layer']==final['first_layer']
    assert scientific_hash(NativeCheckpointStore(folder/'checkpoints').restart())==digest
    manifest=json.loads((folder/'ROLLBACK_MANIFEST.json').read_text());assert len(manifest['removed'])==47
    (OUT/'GENERAL_REGRESSION.json').write_text(json.dumps(dict(passed=True,result=result,scientific=digest,removed=len(manifest['removed']),records=provider.records,wall_exact=True,first_layer_exact=True,restart_exact=True,adjusted_schedule=final['local_schedule']==initial['local_schedule'],seconds=time.perf_counter()-tick,peak_rss=psutil.Process().memory_info().peak_wset),indent=2)+'\n')
if __name__=='__main__':sf2() if sys.argv[1]=='sf2' else general()

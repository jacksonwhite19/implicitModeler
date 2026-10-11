"""Bounded native replay of the sealed large production control."""
import sys
sys.dont_write_bytecode=True
import json,time,gc
import psutil
from pathlib import Path
from native_memory_artifacts import materialize
from sdf_cfd_mesher_v4.advancing_front.native_checkpoint import NativeCheckpointStore
from sdf_cfd_mesher_v4.advancing_front.native_front_providers import NativeRiskAtlas
from sdf_cfd_mesher_v4.advancing_front.native_prism_validation import audit_native_prisms
from sdf_cfd_mesher_v4.advancing_front.mixed_front import scientific_hash
from sdf_cfd_mesher_v4.advancing_front.production_authority import configuration
from sdf_cfd_mesher_v4.certificate.native_manifest import NativeManifest
from sdf_cfd_mesher_v4.core.interface import extract
from broader_geometry_cohort import OUT,ROOT


def run():
    tick=time.perf_counter();out=OUT/'SF2';out.mkdir(exist_ok=True)
    with materialize('fresh') as source:
        store=NativeCheckpointStore(source/'checkpoints');state=store.restart()
        digest=scientific_hash(state);atlas=NativeRiskAtlas(state,store.objects)
        forecast=atlas.native_forecast.scientific();atlas_hash=atlas.scientific_hash()
        manifest=NativeManifest(source/'native_manifest')
        audit=audit_native_prisms(state,store.objects,manifest)
        _,_,_,meta=extract(state)
        prior=json.loads((ROOT/'results/fixed_interface_bulk_core_repair/source/sf2_interface/INTERFACE.json').read_text())
        assert meta==prior
        again=store.restart();other=NativeRiskAtlas(again,store.objects)
        assert digest==scientific_hash(again)
        assert forecast==other.native_forecast.scientific() and atlas_hash==other.scientific_hash()
        assert not state['events'] and configuration()['automatic_remesh']
        result=dict(passed=True,scope='Fresh bounded native replay; sealed full build and A/A2 reused',
            audit=audit,interface=meta['identity'],outer_faces=meta['faces'],
            scientific=digest,atlas=atlas_hash,forecast_exact=True,restart_exact=True,
            remesh_count=0,seconds=time.perf_counter()-tick,
            peak_rss_bytes=psutil.Process().memory_info().peak_wset)
        (out/'BOUNDED_REPLAY.json').write_text(json.dumps(result,sort_keys=True,indent=2)+'\n')
        # Release native mmap owners before the temporary archive is removed.
        del other,again,atlas,state,store,manifest
        gc.collect()


if __name__=='__main__':run()

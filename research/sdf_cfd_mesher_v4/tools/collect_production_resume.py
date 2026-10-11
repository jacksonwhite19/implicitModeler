"""Recover bounded result and exact replay evidence from the committed pointer."""
import sys
sys.dont_write_bytecode=True
import json,time
from pathlib import Path
import psutil
from run_production_scale_resume import OUT,summary,native
from sdf_cfd_mesher_v4.advancing_front.native_checkpoint import NativeCheckpointStore
from sdf_cfd_mesher_v4.advancing_front.mixed_front import scientific_hash
from sdf_cfd_mesher_v4.common.authority import identity
from retained_state_storage import retained_storage


def collect(name,label):
    folder=OUT/name/label;tick=time.perf_counter();store=NativeCheckpointStore(folder/'checkpoints')
    before=(store.path/'CURRENT.json').read_bytes();pointer=json.loads(before)
    state=store.restart();digest=scientific_hash(state);assert digest==pointer['scientific']
    result=summary(state)
    from audit_native_receipt_tables import audit_tables
    result['certificate_audit']=audit_tables(state['quality'],state['certificate'],len(state['cells']))
    result.update(case=name,label=label,scientific=digest,termination=json.loads((folder/'TERMINATION.json').read_text()),
        wall_identity=identity(state['wall']),first_layer_identity=identity(state['first_layer']),
        events=state['events'],retained_storage=retained_storage({'native_state':state}),
        scope='Latest committed partial shell; not final requested shell qualification',
        core='NotReached',complete_mesh='NotReached',foundation='NotRun',
        warning_regions=None,warning_scope='Provider dependency atlas has no separate warning-region classification field')
    original_wall=result['wall_identity'];original_first=result['first_layer_identity']
    del state
    restored=store.restart()
    result['restart_exact']=scientific_hash(restored)==digest
    assert result['restart_exact'] and (store.path/'CURRENT.json').read_bytes()==before
    assert identity(restored['wall'])==original_wall and identity(restored['first_layer'])==original_first
    authority=json.loads((folder/'GEOMETRY_AUTHORITY.json').read_text())
    result['first_layer_exact']=authority['first_layer_hash']==original_first
    result['wall_exact']=authority['wall_identity']==original_wall
    assert result['wall_exact'] and result['first_layer_exact']
    interface=native.export(restored,folder/'checkpoint_interface')
    result['interface']={k:v for k,v in interface.items() if k not in ('face_ids','vertex_ids','shell_owners','ancestry','versions')}
    result['interface']['full_metadata']='checkpoint_interface/INTERFACE.json'
    result['replay_seconds']=time.perf_counter()-tick
    mem=psutil.Process().memory_info();result['replay_peak_rss']=getattr(mem,'peak_wset',mem.rss)
    native.g.save(folder/'COMMITTED_RESULT.json',result)
    print(json.dumps({k:result[k] for k in ('case','layer','cells','relations','extrema','restart_exact','wall_exact','first_layer_exact','replay_seconds','replay_peak_rss')}),flush=True)
    return result

if __name__=='__main__':collect(sys.argv[1],sys.argv[2])

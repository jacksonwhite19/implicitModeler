"""Explicit copied-checkpoint migration; legacy authority remains external."""
import json,time
from pathlib import Path
from sdf_cfd_mesher_v4.advancing_front.native_checkpoint import NativeCheckpointStore
from sdf_cfd_mesher_v4.advancing_front.authenticated_receipts import buffers,storage_identity
from sdf_cfd_mesher_v4.advancing_front.native_receipt_delta import raw_items
from sdf_cfd_mesher_v4.advancing_front import canonical_blocks as blocks
from sdf_cfd_mesher_v4.advancing_front.authenticated_map import canonical_root

def migrate(checkpoints,output,depth=8,compare_legacy=False):
    start=time.perf_counter();store=NativeCheckpointStore(checkpoints);state=store.restart();table=state['certificate']
    old=dict(table.authenticated);legacy=None;legacy_seconds=0.
    if compare_legacy:
        from identity_qualification_audit import legacy_state_hash
        tick=time.perf_counter();legacy=legacy_state_hash(state);legacy_seconds=time.perf_counter()-tick
    tick=time.perf_counter();k,v,_=buffers((k,v,0) for k,v in raw_items(table));encoded=time.perf_counter()-tick
    tick=time.perf_counter();patricia=canonical_root(k,v).hex();crosscheck=time.perf_counter()-tick
    if patricia!=old['root']:raise ValueError('BlockMigrationPatriciaPopulationMismatch')
    tick=time.perf_counter();descriptor=blocks.build(store.objects,k,v,depth);built=time.perf_counter()-tick
    assert blocks.canonical_root(k,v,depth)==descriptor['root']
    descriptor['storage']=storage_identity(table);table.authenticated=descriptor;table.auth_objects=store.objects
    tick=time.perf_counter();scientific=store.publish(state);published=time.perf_counter()-tick
    assert store.restart()['certificate'].authenticated['root']==descriptor['root']
    result=dict(passed=True,legacy_authority=old['authority'],legacy_root=patricia,legacy_scientific=legacy,legacy_scientific_seconds=legacy_seconds,new_authority=blocks.AUTHORITY,new_root=descriptor['root'],count=len(k),encoding_seconds=encoded,patricia_crosscheck_seconds=crosscheck,block_build_seconds=built,checkpoint_seconds=published,total_seconds=time.perf_counter()-start,scientific=scientific,work=descriptor['work'])
    Path(output).write_text(json.dumps(result,indent=2)+'\n');print('BLOCK_MIGRATION '+json.dumps(result),flush=True)
    return result

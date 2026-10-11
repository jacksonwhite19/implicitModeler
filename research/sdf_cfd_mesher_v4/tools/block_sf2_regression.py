"""Bounded full SF2 native authority audit and block-root migration.

The native SF2 authority stores twenty qualified relation-block ledgers, not
56 million expanded receipts. This preserves that exact component model.
"""
import gc,json,time,tempfile,shutil,os
import psutil
from pathlib import Path
import broader_sf2_control as control
from native_memory_artifacts import materialize
from sdf_cfd_mesher_v4.advancing_front.native_checkpoint import NativeCheckpointStore,scientific_digest
from sdf_cfd_mesher_v4.advancing_front.authenticated_receipts import buffers,storage_identity
from sdf_cfd_mesher_v4.advancing_front.native_receipt_delta import raw_items
from sdf_cfd_mesher_v4.advancing_front import canonical_blocks as blocks
from sdf_cfd_mesher_v4.advancing_front.mixed_front import scientific_hash
from sdf_cfd_mesher_v4.advancing_front.native_prism_validation import audit_native_prisms
from sdf_cfd_mesher_v4.certificate.native_manifest import NativeManifest

OUT=Path(__file__).resolve().parents[1]/'results/canonical_block_authenticated_storage'

def run():
    control.OUT=OUT;control.run();start=time.perf_counter()
    with materialize('fresh') as source,tempfile.TemporaryDirectory(prefix='v4-block-sf2-') as temporary:
        private=Path(temporary)/'checkpoints';shutil.copytree(source/'checkpoints',private,copy_function=os.link)
        store=NativeCheckpointStore(private);state=store.restart();table=state['certificate']
        legacy=scientific_digest(table);old=scientific_hash(state)
        k,v,_=buffers((k,v,0) for k,v in raw_items(table));root=blocks.build(store.objects,k,v,8)
        root['storage']=storage_identity(table);table.authenticated=root;table.auth_objects=store.objects
        new=store.publish(state);restored=store.restart();audit=audit_native_prisms(restored,store.objects,NativeManifest(source/'native_manifest'))
        assert scientific_digest(restored['certificate'])==legacy
        assert scientific_hash(restored)==new and blocks.canonical_root(k,v,8)==root['root']
        # This native root has only twenty ledger rows. Preserve its complete
        # compact bytes inline, avoiding another multi-GB SF2 checkpoint copy.
        sealed=dict(descriptor=root,payloads_hex={r['file']:(store.objects/r['file']).read_bytes().hex() for r in root['files']})
        (OUT/'SF2/BLOCK_STORE.json').write_text(json.dumps(sealed,indent=2)+'\n')
        result=dict(passed=True,scope='Complete native SF2 relation authority: twenty ledger values bind all 56,462,864 preserved relations through verified component hashes; no expanded per-relation receipt migration.',old_scientific=old,new_scientific=new,legacy_receipt_digest=legacy,root=root['root'],root_rows=len(k),relations=audit['certificate']['active'],audit=audit,restart_exact=True,independent_root_exact=True,work=root['work'],seconds=time.perf_counter()-start,peak_rss=psutil.Process().memory_info().peak_wset)
        (OUT/'SF2/BLOCK_ROOT.json').write_text(json.dumps(result,indent=2)+'\n');print('SF2_BLOCK '+json.dumps(result),flush=True)
        del restored,state,table,store;gc.collect()

if __name__=='__main__':run()

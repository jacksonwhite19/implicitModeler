"""Read preserved receipt keys; measure canonical prefix partitions before design."""
import hashlib,json,time
from pathlib import Path
import numpy as np
from sdf_cfd_mesher_v4.advancing_front.native_checkpoint import NativeCheckpointStore
from profile_capsule_scaling import select_pointer

ROOT=Path(__file__).resolve().parents[1]/'results/canonical_block_authenticated_storage'
PRIOR=ROOT.parent/'incremental_certificate_receipt_identity/capsule_production_control'

def keys(table):
    from contextlib import closing
    result=np.empty(len(table),dtype='S32')
    with closing(table._connect()) as db:
        for i,(key,) in enumerate(db.execute('SELECT key FROM receipts ORDER BY key')):
            result[i]=hashlib.sha256(key.encode()).digest()
    result.sort();return result

def load(label,scientific=None):
    path=PRIOR/label/'checkpoints';pointer=select_pointer(path,scientific)
    envelope=json.loads((path/pointer['checkpoint']).read_text())
    assert hashlib.sha256((path/pointer['checkpoint']).read_bytes()).hexdigest()==pointer['sha256']
    store=NativeCheckpointStore(path);store.authenticated_objects=True
    return store._receipt(envelope['components']['certificate'])

def run():
    start=time.perf_counter();out=ROOT/'arrays';out.mkdir(exist_ok=True)
    base=load('optimized_7_9','aef8b2175a6c1dcdc7d91ca9a549abc4b58f5687985994eaa9fbbec9ddea9a9a')
    active=keys(base);np.save(out/'keys6.npy',active);result=[]
    for layer,label,scientific in [(7,'final_bench',None),(8,'final_8_9','c590ca84ca42770a2087c9335d6ea8feccc292f4bd0506d25a08f92fb627919e'),(9,'final_8_9',None)]:
        tick=time.perf_counter();table=load(label,scientific);delta=keys(table.updates)
        if table.removed:raise ValueError('StudyRequiresPreservedOrdinaryTransition')
        np.save(out/f'delta{layer}.npy',delta)
        active=np.union1d(active,delta)
        assert len(active)==len(table)
        ranks=np.searchsorted(active,delta);raw=active.view('u1').reshape(-1,32);dr=delta.view('u1').reshape(-1,32)
        prefixes=(raw[:,0].astype('u4')<<8)|raw[:,1];dp=(dr[:,0].astype('u4')<<8)|dr[:,1]
        partitions=[]
        for depth in (8,12,16):
            pop=np.bincount(prefixes>>(16-depth),minlength=1<<depth);changed=np.bincount(dp>>(16-depth),minlength=1<<depth)
            touched=changed>0;occupied=pop>0
            partitions.append(dict(prefix_bits=depth,occupied_blocks=int(occupied.sum()),touched_blocks=int(touched.sum()),untouched_blocks=int((occupied&~touched).sum()),touched_fraction=float(touched.sum()/occupied.sum()),max_changed=int(changed.max()),median_changed_per_touched=float(np.median(changed[touched])),mean_active_rows=float(pop[occupied].mean()),active_rows_rewritten=int(pop[touched].sum()),canonical_row_payload_bytes_rewritten=int(pop[touched].sum())*64,canonical_row_payload_bytes_reused=int(pop[~touched].sum())*64,upper_binary_internal_nodes=(1<<depth)-1,lookup_leaf_blocks=1))
        row=dict(layer=layer,active_keys=len(active),changed_new_keys=len(delta),rank_min=int(ranks[0]),rank_max=int(ranks[-1]),rank_decile_counts=np.histogram(ranks,bins=np.linspace(0,len(active),11))[0].tolist(),partitions=partitions,seconds=time.perf_counter()-tick)
        result.append(row);print(json.dumps(row),flush=True)
        (ROOT/'KEY_DISPERSION.json').write_text(json.dumps(dict(results=result,total_seconds=time.perf_counter()-start,note='Payload bytes are exact counts times the defined 32-byte key digest plus 32-byte content digest; file/container overhead not included.'),indent=2)+'\n')
    print('COMPLETE',time.perf_counter()-start,flush=True)

if __name__=='__main__':run()

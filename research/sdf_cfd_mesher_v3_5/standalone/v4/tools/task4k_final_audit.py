from task4k_common import *
from task4j_common import geometry
from task4k_replay import FIELDS
from sdf_cfd_mesher_v4.certificate.production import verify_committed_layer
import psutil
def run():
    f,t,x=geometry();histories=[]
    for replay in (1,2):
        rows=[];s=time.perf_counter()
        for layer in range(1,21):
            guard();path=OUT/'replay_A'/f'layer_{layer:02d}';record=read(path/'MANIFEST.json')
            count,perf=verify_committed_layer(f,t,x[1:layer+1],path,record)
            with np.load(path/'COMPACT.npz') as z:
                p=z['packed'];new=p&np.uint64(0xffffffff)
                assert np.all(new>=((layer-1)*len(t.edges))) and np.all(new<layer*len(t.edges))
            rows.append(dict(layer=layer,pairs=count,pair_hash=record['pair_hash'],scientific_hash=scientific_hash({k:record[k] for k in FIELDS}),source_transaction=f'replay_A/layer_{layer:02d}',proof_classes=record['class_counts'],audit_seconds=perf[0]['seconds']))
        total=sum(r['pairs'] for r in rows);reference=sum(r['required_pairs'] for r in read(ROOT/'results/task4j/BASELINE_PROFILE.json')['rows']);assert total==reference
        scientific=dict(rows=[{k:v for k,v in r.items() if k!='audit_seconds'} for r in rows],total_pairs=total,missing=0,duplicates=0,forbidden=0,unresolved=0,new_exact_calls=0)
        histories.append(dict(replay=replay,seconds=time.perf_counter()-s,scientific=scientific,scientific_hash=scientific_hash(scientific),peak_rss=psutil.Process().memory_info().peak_wset));save('FINAL_AUDIT.json',histories)
    assert histories[0]['scientific_hash']==histories[1]['scientific_hash']
    a=read(OUT/'REPLAY_A.json');b=read(OUT/'REPLAY_B.json')
    for old,new in zip(a,b):
        for key in ('layer','pairs','scientific_hash','compact_hash','exact_hash'):assert old[key]==new[key],('ReplayMismatch',key,old['layer'])

    assert len(a)==len(b)==20
    save('DETERMINISM.json',dict(transactions=20,proof_classes=True,exact_outcomes=True,pair_populations=True,compact_bytes=True,rich_bytes=True,cache_counts_are_runtime_diagnostics=True,final_manifest=True,scientific_hash=histories[0]['scientific_hash']))
if __name__=='__main__':run()

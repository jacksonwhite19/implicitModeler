"""Qualification-only full-population reconstruction against both authorities."""
import argparse,json,time
from pathlib import Path
import psutil
from sdf_cfd_mesher_v4.advancing_front.native_checkpoint import NativeCheckpointStore
from sdf_cfd_mesher_v4.advancing_front.authenticated_receipts import buffers
from sdf_cfd_mesher_v4.advancing_front.native_receipt_delta import raw_items,ReceiptDelta
from sdf_cfd_mesher_v4.advancing_front.authenticated_map import canonical_root as patricia
from sdf_cfd_mesher_v4.advancing_front.canonical_blocks import canonical_root
from profile_capsule_scaling import select_pointer

def run(checkpoints,timers,references,output):
    state=NativeCheckpointStore(checkpoints).restart();tables={};table=state['certificate']
    while True:
        tables[len(table)]=table
        if not isinstance(table,ReceiptDelta):break
        table=table.base
    result=[];rows=json.loads(timers.read_text());refs=json.loads(references.read_text())
    for row in rows:
        if row['event']!='retained_after_commit':continue
        layer=str(row['layer']);ref=refs[layer];source=Path(ref['source']);pointer=select_pointer(source,ref.get('scientific'))
        legacy=json.loads((source/pointer['checkpoint']).read_text())['components']['certificate']['authenticated']
        table=tables[row['relations']];tick=time.perf_counter();k,v,_=buffers((k,v,0) for k,v in raw_items(table));encoded=time.perf_counter()-tick
        tick=time.perf_counter();new=canonical_root(k,v,8);block_seconds=time.perf_counter()-tick
        tick=time.perf_counter();old=patricia(k,v).hex();old_seconds=time.perf_counter()-tick
        if new!=row['root'] or old!=legacy['root'] or len(k)!=legacy['count']:raise ValueError('IndependentMigrationPopulationMismatch')
        item=dict(layer=int(layer),passed=True,count=len(k),block_root=new,patricia_root=old,stream_encode_sort_seconds=encoded,block_root_seconds=block_seconds,patricia_root_seconds=old_seconds,peak_rss=psutil.Process().memory_info().peak_wset)
        result.append(item);print('DUAL_ROOT '+json.dumps(item),flush=True)
        output.write_text(json.dumps(result,indent=2)+'\n');del k,v

if __name__=='__main__':
    p=argparse.ArgumentParser()
    for name in ('checkpoints','timers','references','output'):p.add_argument(name,type=Path)
    a=p.parse_args();run(a.checkpoints,a.timers,a.references,a.output)

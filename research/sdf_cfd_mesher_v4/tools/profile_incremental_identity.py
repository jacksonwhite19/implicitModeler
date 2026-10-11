"""One preserved production transition with precise identity timers."""
import argparse,json,sys,time,functools
from pathlib import Path
import profile_capsule_scaling as benchmark
import identity_timing as timing
import validation_row_stream as stream
import native_proof_segments as segments
import reuse_mixed_certificate as reuse
from compact_proof_rows import CompactProofRows
from sdf_cfd_mesher_v4.advancing_front import native_checkpoint as storage
from sdf_cfd_mesher_v4.advancing_front.native_receipt_delta import ReceiptDelta

OUT=benchmark.native.ROOT/'results/incremental_certificate_receipt_identity'

def install():
    timing.ENABLED=True
    from sdf_cfd_mesher_v4.advancing_front import authenticated_receipts as roots,authenticated_map as merkle
    def wrapped(fn,name,count=None):
        @functools.wraps(fn)
        def call(*args,**kwargs):
            with timing.measure(name,count(*args,**kwargs) if count else 0):
                return fn(*args,**kwargs)
        return call
    for module,key,name in [(stream,'clean_rows_digest','clean_rows_digest'),
                            (stream,'validation_digest','validation_digest'),
                            (storage,'scientific_digest','scientific_digest'),
                            (segments,'extension_plan','extension_plan'),
                            (roots,'buffers','delta_key_value_hashing_and_sort'),
                            (roots,'ordered_buffers','delta_canonical_sort')]:
        original=getattr(module,key);new=wrapped(original,name)
        # Imported aliases must observe the same executable path.
        for target in list(sys.modules.values()):
            if target is None:continue
            for alias,value in list(vars(target).items()):
                if value is original:setattr(target,alias,new)
    for cls,key,name,count in [
        (ReceiptDelta,'measure_identity','receipt_measure_identity',None),
        (storage.NativeCheckpointStore,'_receipt','checkpoint_canonical_verification',None),
        (storage.NativeCheckpointStore,'_object','checkpoint_byte_verification',None),
        (CompactProofRows,'ordered_unique','canonical_sort',lambda self:len(self)),
        (CompactProofRows,'__getitem__','compact_proof_materialization',lambda *a:1)]:
        setattr(cls,key,wrapped(getattr(cls,key),name,count))
    for key,name in [('_get','merkle_node_lookup'),('put','merkle_node_write')]:
        setattr(merkle.Nodes,key,wrapped(getattr(merkle.Nodes,key),name,lambda *a:1))
    original=segments.LoadedSegment.rows
    def rows(self):
        values=iter(original(self))
        while True:
            try:
                with timing.measure('segment_proof_materialization'):value=next(values)
            except StopIteration:return
            timing.ROWS['segment_proof_materialization']['rows']+=1
            yield value
    segments.LoadedSegment.rows=rows

def main():
    p=argparse.ArgumentParser();p.add_argument('label');p.add_argument('--source',type=Path,required=True)
    p.add_argument('--case',default='capsule_production_control');p.add_argument('--input-scientific');p.add_argument('--steps',type=int,default=1)
    p.add_argument('--bootstrap',action='store_true')
    p.add_argument('--references',type=Path)
    args=p.parse_args();benchmark.OUT=OUT;install()
    old_run=benchmark.native.run
    def run(*a,**kw):
        if args.bootstrap:
            from bootstrap_checkpoint_identity import migrate
            folder=OUT/args.case/args.label
            migrate(folder/'checkpoints',folder/'BOOTSTRAP.json')
        observer=kw.get('observer');previous={};records=[]
        def observe(event,state,provider):
            nonlocal previous
            now=timing.snapshot()
            delta={k:{field:v[field]-previous.get(k,{}).get(field,0) for field in v} for k,v in now.items()}
            from identity_memory_accounting import snapshot as memory_snapshot
            records.append(dict(event=event,layer=max(c['layer'] for c in state['cells'].values()),timers=delta,memory=memory_snapshot()))
            previous=now
            path=OUT/args.case/args.label/'IDENTITY_TIMERS.json'
            path.write_text(json.dumps(records,indent=2)+'\n')
            print('IDENTITY '+json.dumps({k:v for k,v in records[-1].items() if k!='memory'}),flush=True)
            if observer:observer(event,state,provider)
            layer=str(records[-1]['layer'])
            references=json.loads(args.references.read_text()) if args.references else {}
            if event=='retained_after_commit' and layer in references:
                from identity_qualification_audit import crosscheck
                reference=references[layer]
                profile=getattr(observer,'profile',None)
                from contextlib import nullcontext
                with profile.phase('qualification_legacy_crosscheck') if profile else nullcontext():
                    result=crosscheck(state,reference['scientific'],reference['proof'])
                (OUT/args.case/args.label/('LEGACY_CROSSCHECK_'+layer+'.json')).write_text(json.dumps(result,indent=2)+'\n')
                print('LEGACY_CROSSCHECK '+json.dumps(result),flush=True)
                previous=timing.snapshot()
        kw['observer']=observe;return old_run(*a,**kw)
    benchmark.native.run=run
    try:benchmark.run(args.case,args.label,args.source,args.steps,args.input_scientific)
    finally:
        folder=OUT/args.case/args.label
        (folder/'IDENTITY_TOTALS.json').write_text(json.dumps(timing.snapshot(),indent=2)+'\n')

if __name__=='__main__':main()

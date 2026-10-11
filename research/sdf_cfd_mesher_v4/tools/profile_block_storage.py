"""Preserved native transitions with explicit storage migration and accounting."""
import argparse,json,time,functools
from pathlib import Path
import profile_capsule_scaling as benchmark
import profile_incremental_identity as instrumentation
import identity_timing as timing
from identity_memory_accounting import snapshot
from sdf_cfd_mesher_v4.advancing_front import canonical_blocks as blocks

OUT=benchmark.native.ROOT/'results/canonical_block_authenticated_storage'

def main():
    p=argparse.ArgumentParser();p.add_argument('label');p.add_argument('--source',required=True,type=Path)
    p.add_argument('--case',default='capsule_production_control');p.add_argument('--steps',type=int,default=1)
    p.add_argument('--input-scientific');p.add_argument('--migrate',action='store_true');p.add_argument('--migration-legacy',action='store_true');p.add_argument('--references',type=Path)
    p.add_argument('--receipt-cache',type=Path)
    p.add_argument('--compress-closed-receipts',action='store_true')
    args=p.parse_args();benchmark.OUT=OUT;instrumentation.install();folder=OUT/args.case/args.label
    if args.compress_closed_receipts:
        import authenticated_certificate as certificate_storage
        original_storage=certificate_storage.receipt_table_from_rows
        def stored_then_compressed(*a,**kw):
            result=original_storage(*a,**kw)
            from compress_storage_study import run as compress_closed
            with timing.measure('filesystem_capacity'):compress_closed(a[0])
            return result
        certificate_storage.receipt_table_from_rows=stored_then_compressed
    memory=[]
    for name in ('build','update','verify','finish'):
        fn=getattr(blocks,name)
        def wrap(fn,name):
            @functools.wraps(fn)
            def call(*a,**kw):
                with timing.measure('block_'+name):result=fn(*a,**kw)
                if name in ('build','update'):
                    memory.append(dict(stage=name,snapshot=snapshot(),work=result['work'],root=result['root']))
                    (folder/'BLOCK_MEMORY.json').write_text(json.dumps(memory,indent=2)+'\n')
                return result
            return call
        setattr(blocks,name,wrap(fn,name))
    original=benchmark.native.run
    def run(*a,**kw):
        if args.receipt_cache:
            import os,hashlib
            rows=[]
            for source in sorted(args.receipt_cache.glob('*.sqlite')):
                target=folder/'checkpoints/objects'/source.name
                if source.name.endswith('.merkle.sqlite') or target.exists():continue
                with source.open('rb') as stream:h=hashlib.file_digest(stream,'sha256').hexdigest()
                os.link(source,target);rows.append(dict(source=str(source),file=source.name,sha256=h))
            (folder/'RECEIPT_CACHE.json').write_text(json.dumps(dict(files=rows,scope='Byte-verified immutable cache inputs; producer still builds and independently validates canonical receipt content before reuse'),indent=2)+'\n')
        if args.migrate:
            from migrate_block_checkpoint import migrate
            migrate(folder/'checkpoints',folder/'MIGRATION.json',compare_legacy=args.migration_legacy)
        observer=kw.get('observer');previous=timing.snapshot();records=[]
        def observe(event,state,provider):
            nonlocal previous
            now=timing.snapshot();delta={k:{field:v[field]-previous.get(k,{}).get(field,0) for field in v} for k,v in now.items()}
            descriptor=state['certificate'].authenticated;layer=max(c['layer'] for c in state['cells'].values())
            records.append(dict(event=event,layer=layer,timers=delta,memory=snapshot(),root=descriptor['root'],relations=len(state['certificate']),block_work=descriptor['work'],active_bytes=sum(x['bytes'] for x in descriptor['blocks'])))
            previous=now;(folder/'BLOCK_TIMERS.json').write_text(json.dumps(records,indent=2)+'\n')
            print('BLOCK_LAYER '+json.dumps({k:v for k,v in records[-1].items() if k!='memory'}),flush=True)
            if observer:observer(event,state,provider)
            references=json.loads(args.references.read_text()) if args.references else {}
            if event=='retained_after_commit' and str(layer) in references:
                from identity_qualification_audit import crosscheck
                row=references[str(layer)];result=crosscheck(state,row['scientific'],row['proof'])
                (folder/f'LEGACY_CROSSCHECK_{layer}.json').write_text(json.dumps(result,indent=2)+'\n')
                print('LEGACY_CROSSCHECK '+json.dumps(result),flush=True);previous=timing.snapshot()
            if event=='retained_after_commit':
                retention=folder/'checkpoints/BLOCK_STORAGE.json'
                if retention.exists():(folder/f'RETENTION_{layer}.json').write_bytes(retention.read_bytes())
                from deduplicate_storage_study import run as deduplicate
                deduplicate(folder/'checkpoints/objects')
        kw['observer']=observe;return original(*a,**kw)
    benchmark.native.run=run
    benchmark.run(args.case,args.label,args.source,args.steps,args.input_scientific)

if __name__=='__main__':main()

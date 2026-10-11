"""Retained-row diagnostic. No front generation, assembly or publication."""
import argparse
import cProfile
from itertools import islice
import json
from pathlib import Path
import pstats
import time

from path_preserving_deduplication import durable_json, file_hash
import single_quality_component as quality
from private_quality_codec_reuse import verified_private_codec
from assembly_quality_handoff import installed
from record_receipt_resume import disk

MINIMUM_FREE=45_120_503_439  # Existing production operational floor, not a science gate.


def resource_check():
    if disk()['C:\\']['available']<MINIMUM_FREE:raise ValueError('MinimumFreeSpaceBreach')


def measure(output, rows, mode, *, workers=1, process=False, reuse=False, profile=False):
    from profile_remaining_prism import memory_sample
    path=output/mode/'QUALITY_DELTA.json'
    path.parent.mkdir(parents=True)
    profiler=cProfile.Profile(); start=time.perf_counter()
    resource_check()
    if profile:profiler.enable()
    with memory_sample(output,mode), installed(reuse_private_codec=False), verified_private_codec(rows,workers=workers,process=process,reuse_hash_stream=reuse):
        result=quality.write(path,rows,workers=workers,process=process,progress=resource_check)
    profiler.disable()
    seconds=time.perf_counter()-start
    # Preserve completed writer evidence even if optional report reduction fails.
    durable_json(output/(mode+'_WRITER.json'),dict(seconds=seconds,result=result,
        workers=workers,process=process,reuse=reuse,profile=profile))
    if profile:profiler.dump_stats(str(output/(mode+'.prof')))
    functions=[]
    stats=pstats.Stats(profiler).stats if profile else {}
    for (filename,line,name),(primitive,calls,self_time,total_time,callers) in stats.items():
        if 'sdf_cfd_mesher_v4' in filename or name in ('dumps','loads') or 'compress' in name:
            functions.append(dict(file=filename,line=line,name=name,calls=calls,self_seconds=self_time,total_seconds=total_time))
    with quality.snapshot(path) as snap:
        if list(snap['rows'])!=rows:raise ValueError('RetainedRowReadbackMismatch')
    durable_json(output/(mode+'.json'),dict(seconds=seconds,result=result,workers=workers,process=process,reuse=reuse,
        payload_sha256=quality.access.read(path)['payload']['sha256'],
        functions=sorted(functions,key=lambda r:r['total_seconds'],reverse=True)[:65]))
    return dict(seconds=seconds,result=result)


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--source',type=Path,required=True);parser.add_argument('--count',type=int,default=10001)
    parser.add_argument('--compare',action='store_true');parser.add_argument('--workers',type=int,default=1)
    parser.add_argument('--profile',action='store_true')
    args=parser.parse_args()
    if args.count<=0 or args.output.exists():raise ValueError('FreshBoundedDiagnosticRequired')
    args.output.mkdir(parents=True)
    start=time.perf_counter()
    with quality.snapshot(args.source) as snap:
        count=len(snap['rows'])
        # Cover all receipt kinds using deterministic evenly spaced source ordinals.
        ordinals=range(0,count,max(1,count//args.count))
        selected=list(islice(ordinals,args.count))
        # Existing guarded iterator, not N repeated token/index lookups. Its
        # snapshot has already verified the whole immutable ordered database.
        wanted=iter(selected); next_index=next(wanted,None);rows=[]
        for i,row in enumerate(snap['rows']):
            if i==next_index:
                rows.append(row);next_index=next(wanted,None)
            if next_index is None:break
        if len(rows)!=len(selected):raise ValueError('IncompleteRetainedDiagnosticPopulation')
        refs=snap['references']; logical=snap['semantic']
    durable_json(args.output/'INPUT.json',dict(source=str(args.source.resolve()),references=refs,
        logical=logical,source_rows=count,selected_ordinals=dict(start=0,step=max(1,count//args.count),count=len(selected)),
        load_seconds=time.perf_counter()-start,rows=len(rows),implementation_sha256=file_hash(__file__)))
    records=[]
    for mode,reuse in [('before',False)]+([('after',True)] if args.compare else []):
        result=measure(args.output,rows,mode,workers=args.workers,process=args.workers!=1,reuse=reuse,profile=args.profile)
        records.append(result);print(mode,json.dumps(result),flush=True)
    if args.compare:
        a=quality.access.read(args.output/'before/QUALITY_DELTA.json');b=quality.access.read(args.output/'after/QUALITY_DELTA.json')
        if a['logical']!=b['logical'] or a['payload']['sha256']!=b['payload']['sha256']:
            raise ValueError('RetainedWriterByteMismatch')
        durable_json(args.output/'COMPARISON.json',dict(exact_logical_hashes=True,exact_database_bytes=True,measurements=records))


if __name__=='__main__':main()

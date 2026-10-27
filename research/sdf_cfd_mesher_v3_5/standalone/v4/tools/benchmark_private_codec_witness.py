"""Retained full-population private writer comparison. No front is generated.

Use the actual quality writer/handoff, codec and independent cold reader. The
before arm withholds ONLY the new worker witness, reproducing old verification.
Nothing in this diagnostic selects scientific acceptance or checkpoint output.
"""
import argparse
from contextlib import nullcontext
import json
from pathlib import Path
import time
from unittest.mock import patch

import private_quality_hash_stream as stream
import single_quality_component as quality
from sdf_cfd_mesher_v4.common.authority import identity
from path_preserving_deduplication import durable_json,file_hash
from profile_assembly_rows import measure,resource_check


def main():
    p=argparse.ArgumentParser();p.add_argument('--source',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--review',type=Path,required=True)
    p.add_argument('--workers',type=int,default=4);p.add_argument('--atlas',type=Path)
    p.add_argument('--comparison',choices=('codec_witness','key'),default='codec_witness')
    args=p.parse_args();review=json.loads(args.review.read_text(encoding='utf-8'))
    if review.get('passed') is not True:raise ValueError('IndependentWriterReviewRequired')
    for name in ('private_quality_hash_stream.py','private_quality_codec_reuse.py'):
        if review['sources'].get(name)!=file_hash(Path(__file__).parent/name):raise ValueError('StaleWriterReview')
    if args.output.exists():raise ValueError('PreserveExistingWriterComparison')
    resource_check();args.output.mkdir(parents=True);started=time.perf_counter()
    with quality.snapshot(args.source) as snap:
        refs=snap['references'];logical=snap['semantic'];rows=list(snap['rows'])
        plan_path=args.source.parent/'CANDIDATE_PLAN.json'
        with quality.access.guard(plan_path):
            plan=quality.access.read(plan_path)
            if any(plan['references'].get(name)!=record for name,record in refs.items()):
                raise ValueError('UnboundRetainedWriterInputs')
            plan_sha256=file_hash(plan_path)
        # Existing private publication root binds this exact plan. Check just
        # that sealed node; no deep recovery or inherited-proof traversal.
        candidate=args.source.parent/'candidate'
        with quality.access.guard(candidate/'CURRENT.json'):
            pointer=quality.access.read(candidate/'CURRENT.json');root_path=candidate/pointer['file']
            with quality.access.guard(root_path):
                root=quality.access.read(root_path)
                if file_hash(root_path)!=pointer['sha256'] or identity(root)!=pointer['root'] or root['plan']!=plan:
                    raise ValueError('ChangedRetainedWriterPlan')
        sealed_scientific=plan['scientific']
    if args.atlas:
        from compact_front_atlas import digest
        body=json.loads(args.atlas.read_text(encoding='utf-8'))
        if digest(rows)!=body['logical']:raise ValueError('RetainedAtlasMeaningMismatch')
    durable_json(args.output/'INPUT.json',dict(source=str(args.source.resolve()),references=refs,
        logical=logical,rows=len(rows),review_sha256=file_hash(args.review),
        sources={n:file_hash(Path(__file__).parent/n) for n in ('private_quality_hash_stream.py','private_quality_codec_reuse.py')},
        kind='atlas' if args.atlas else 'quality',workers=args.workers,load_seconds=time.perf_counter()-started))
    durable_json(args.output/'PARENT_BINDING.json',dict(plan_sha256=plan_sha256,
        root=pointer['root'],scientific=sealed_scientific,source_root_only=True))
    original=stream.hashes
    import private_quality_codec_reuse as reuse
    def without_witness(*a,**k):
        k.pop('codec_witness',None);return original(*a,**k)
    results=[]
    for mode in ('before','after'):
        # The key control changes ONLY the comparison expression. Both arms
        # retain the already qualified worker witness and original codec.
        control=(patch.object(reuse,'same_key',lambda a,b:reuse.canonical(a)==reuse.canonical(b))
                 if args.comparison=='key' else patch.object(stream,'hashes',without_witness))
        with control if mode=='before' else nullcontext():
            result=measure(args.output,rows,mode,workers=args.workers,process=args.workers!=1,reuse=True,profile=False)
        results.append(result);print(mode,json.dumps(result),flush=True);resource_check()
    before=quality.access.read(args.output/'before/QUALITY_DELTA.json')
    after=quality.access.read(args.output/'after/QUALITY_DELTA.json')
    original=quality.access.read(args.source)
    if before['logical']!=after['logical'] or after['logical']!=original['logical']:
        raise ValueError('WriterLogicalMismatch')
    if before['payload']['sha256']!=after['payload']['sha256'] or after['payload']['sha256']!=original['payload']['sha256']:
        raise ValueError('WriterDatabaseByteMismatch')
    durable_json(args.output/'COMPARISON.json',dict(passed=True,exact_logical_hashes=True,
        exact_database_bytes=True,full_cold_row_comparisons=True,rows=len(rows),measurements=results,
        comparison=args.comparison,
        total_seconds=time.perf_counter()-started,
        scope='Isolated retained private writer. Not full assembly or production-stage timing.'))


if __name__=='__main__':main()

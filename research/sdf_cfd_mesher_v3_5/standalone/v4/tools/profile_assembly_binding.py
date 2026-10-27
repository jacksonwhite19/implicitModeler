"""Retained assembly construction profile; no generation or publication.

Physical writers are replaced ONLY in this diagnostic by original logical
digest checks against authenticated retained outputs. Not a production timer.
"""
import argparse, ast, cProfile, hashlib, json, os, pstats, subprocess, time
from contextlib import ExitStack, nullcontext
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np

from path_preserving_deduplication import durable_json, file_hash
from record_receipt_resume import disk

V4=Path(__file__).resolve().parents[1]
read=lambda p:json.loads(Path(p).read_text(encoding='utf-8'))


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--mode',choices=('before','after'),default='after')
    parser.add_argument('--no-profile',action='store_true')
    args=parser.parse_args()
    if args.output.exists():raise ValueError('FreshAssemblyDiagnosticRequired')
    args.output.mkdir(parents=True)
    started=time.perf_counter();run=V4/'results/geometry_binding_production/run'
    from capsule_reuse_dispatch import qualified_dispatch
    from composable_state import CommittedState
    from prism_persistent_hotpath import geometry_session
    from incremental_pair_storage import PairIncidence
    from profile_remaining_prism import memory_sample
    from profile_assembly_rows import resource_check
    import immutable_state_access as access
    import critical_assembly as assembly
    import compact_front_atlas as atlas
    import single_quality_component as quality
    import accepted_production_contract as contract
    import current_front_dependencies as dependencies
    from incremental_transaction import canonical
    checked={};calls=[]
    with ExitStack() as held:
        held.enter_context(qualified_dispatch())
        child=held.enter_context(CommittedState(read(run/'RESULT.json')['reference']))
        base=child.parent;child.check();base.check()
        manifest_path=run.parent/'FILE_MANIFEST.json'
        held.enter_context(access.guard(manifest_path))
        relative=manifest_path.relative_to(V4.parents[1]).as_posix()
        if manifest_path.read_bytes()!=subprocess.check_output(['git','show','HEAD:'+relative]):
            raise ValueError('ChangedCommittedAssemblyManifest')
        manifest=read(manifest_path)['files'];opened=set()
        def retained(path):
            path=Path(path);relative=path.relative_to(run)
            if path not in opened:
                held.enter_context(access.guard(path))
                if file_hash(path)!=manifest['run\\'+str(relative)]['sha256']:
                    raise ValueError('ChangedRetainedAssemblyInput',str(relative))
                opened.add(path)
            return read(path)
        change=retained(run/'GEOMETRY_DELTA.json')
        n=child.plan['geometry_counts']['new_vertices']
        prefix=np.full((base.layer+1,n,3),np.nan);current=base
        while current.kind=='recursive':
            for key,value in current.hot['vertices'].items():
                _,layer,_,vertex=key.split(':');prefix[int(layer),int(vertex)]=value
            current=current.parent
        for layer in range(current.layer+1):
            for i in range(n):prefix[layer,i]=current['vertices'][f'layer:{layer}:source:{i}']
        if not np.isfinite(prefix).all():raise ValueError('IncompleteAssemblyPrefix')
        adapter=SimpleNamespace(front=SimpleNamespace(coordinates=prefix[-1]),committed_prefix=prefix)
        held.enter_context(geometry_session(base,adapter))
        pairs=held.enter_context(PairIncidence(base,base.plan['pair_incidence_manifest'],base.plan['pair_incidence_root']))
        # Original complete typed suffix; comparison fixture only, not a live
        # trajectory owner or production authorization.
        dep_body=retained(run/'NEW_FORECAST_DEPENDENCIES.json')
        shared=child.verify(Path(dep_body['shared']['path']));schedules=read(shared)['schedules']
        if len(schedules)!=1:raise ValueError('MultipleFixtureSchedules')
        schedule=[None]*child.layer+next(iter(schedules.values()))
        expected_quality=retained(run/'QUALITY_DELTA.json')['logical']
        expected_atlas=retained(run/'NEW_ATLAS.json')['logical']
        actual_read=assembly.read
        def input_read(path):
            path=Path(path)
            return retained(run/path.relative_to(args.output)) if path.is_relative_to(args.output) else actual_read(path)
        def json_check(path,body):
            name=Path(path).relative_to(args.output).as_posix()
            if name=='ASSEMBLY.json':
                checked[name]={k:v for k,v in body.items() if k!='seconds'}
                expected={k:v for k,v in retained(run/name).items() if k!='seconds'}
                if checked[name]!=expected:raise ValueError('DifferentAssemblyAccounting')
                return
            if name=='NEW_FORECAST_DEPENDENCIES.json':raise ValueError('UnexpectedFixtureWriter')
            if name=='NEW_RECEIPT_INPUTS.json':
                # Match the installed prism_receipt_witness.sealed_write
                # boundary. The retained file deliberately excludes unrelated
                # prior-cell inputs; independent face bindings remain exact.
                cert=retained(run/'FRESH_CERTIFICATE.json')
                required=set(retained(run/'CHANGED_INPUTS.json'))|{'face:'+f for f in retained(run/'objects'/cert['segment']['faces']['file'])}
                if not required<=body.keys():raise ValueError('IncompleteDiagnosticReceipt')
                body={k:body[k] for k in required}
            digest=hashlib.sha256((json.dumps(body,indent=2,sort_keys=True,allow_nan=False)+'\n').encode()).hexdigest()
            if digest!=manifest['run\\'+name]['sha256']:raise ValueError('DifferentAssemblyJSON',name)
            checked[name]=digest
        def timed(name,call,*a,**k):
            tick=time.perf_counter();result=call(*a,**k);calls.append(dict(name=name,seconds=time.perf_counter()-tick));return result
        def atlas_check(path,values,**kwargs):
            value=timed('atlas_logical_digest',atlas.digest,atlas.AtlasRows(values))
            if value!=expected_atlas:raise ValueError('DifferentAssemblyAtlas')
            checked['atlas']=value
        def quality_check(path,rows,**kwargs):
            value=timed('quality_logical_digest',quality.hashes,rows)
            if value!=expected_quality:raise ValueError('DifferentAssemblyQuality')
            checked['quality']=value
        def dependency_check(path,values,**kwargs):
            rows,shared_values,expanded,size=timed('dependency_encode',dependencies.encode_rows,values)
            if (rows!=dep_body['rows'] or shared_values!=schedules or expanded!=dep_body['expanded_sha256'] or size!=dep_body['expanded_bytes']):
                raise ValueError('DifferentAssemblyForecast')
            checked['forecast']=expanded
        os.environ.update(V4_SINGLE_QUALITY='1',V4_INDEXED_ATLAS='1',V4_QUALITY_CODEC='1')
        resource_check();prof=cProfile.Profile();tick=time.perf_counter()
        import prism_hotpath_v2 as hotpath
        held.enter_context(hotpath.cached_component_views(base))
        held.enter_context(hotpath.cached_quality_reads(base))
        face_keys=0;original_key=assembly.face_key
        def counted_key(values):
            nonlocal face_keys
            face_keys+=1;return original_key(values)
        import shared_schedule_hotpath as schedule_hotpath
        with memory_sample(args.output,'construction'),patch.object(assembly,'read',input_read),patch.object(assembly,'durable_json',json_check),\
             patch.object(atlas,'write',atlas_check),patch.object(quality,'write',quality_check),\
             patch.object(dependencies,'write',dependency_check),patch.object(contract,'dependency_schedule',lambda _:schedule),\
             patch.object(assembly,'face_key',counted_key),\
             (patch.object(schedule_hotpath,'plain_json',lambda _:False) if args.mode=='before' else nullcontext()):
            call=assembly.assemble
            if args.mode=='before':
                source=subprocess.check_output(['git','show','673742c8ad5972a1956e528f3d5fc8eb47c9aea5:research/sdf_cfd_mesher_v4/tools/critical_assembly.py']).decode()
                fn=next(n for n in ast.parse(source).body if isinstance(n,ast.FunctionDef) and n.name=='assemble')
                namespace=dict(vars(assembly));exec(compile(ast.Module(body=[fn],type_ignores=[]),'<retained_baseline>','exec'),namespace)
                call=namespace['assemble']
            if not args.no_profile:prof.enable()
            try:call(base,adapter,args.output,pair_index=pairs)
            finally:
                prof.disable();prof.dump_stats(str(args.output/'construction.prof'))
                durable_json(args.output/'PROFILE_CHECKPOINT.json',dict(checked=checked,writer_checks=calls,face_key_calls=face_keys,seconds=time.perf_counter()-tick))
        elapsed=time.perf_counter()-tick;child.check();base.check();resource_check()
    functions=[]
    for (filename,line,name),(primitive,count,self_time,total,callers) in (pstats.Stats(prof).stats.items() if not args.no_profile else ()):
        functions.append(dict(file=filename,line=line,name=name,calls=count,self_seconds=self_time,total_seconds=total))
    prof.dump_stats(str(args.output/'construction.prof'))
    durable_json(args.output/'RESULT.json',dict(passed=True,seconds=elapsed,total_seconds=time.perf_counter()-started,
        source=file_hash(assembly.__file__),mode=args.mode,profile=not args.no_profile,
        face_key_calls=face_keys,checked=checked,writer_checks=calls,
        functions=sorted(functions,key=lambda r:r['total_seconds'],reverse=True)[:90],disk=disk(),
        scope='Retained full assembly construction; original logical digests instead of physical writers; not a normal production benchmark'))
    print('Retained assembly exact',elapsed,flush=True)


if __name__=='__main__':main()

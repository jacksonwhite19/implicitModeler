"""Compare one normal control with retained exact scientific data.

Only existing exact array/plan comparison and guarded file hashes are used.
This is an evidence collector, not a meshing or qualification implementation.
"""
import argparse
from contextlib import ExitStack
from pathlib import Path
import numpy as np
import immutable_state_access as access
from check_geometry_binding_result import compare_arrays
from run_critical_reuse_control import compare_plan,read
from path_preserving_deduplication import durable_json,file_hash


def atlas_rows(body,held):
    # Match the existing compact_front_atlas.snapshot physical reference
    # checks. Held guards alone do not authenticate recorded dependencies.
    key=str(Path(body['rows']).resolve())
    if key not in body['references']:raise ValueError('UnboundAtlasRows')
    for path,record in body['references'].items():
        held.enter_context(access.guard(path))
        if access.token(path)!=record['token'] or file_hash(path)!=record['sha256']:
            raise ValueError('ChangedAtlasReference')
    inner=read(key)
    payload=inner['payload']
    if (payload['path'] not in body['references'] or
        set(body['references'])!={key,payload['path']} or
        dict(sha256=payload['sha256'],token=payload['token'])!=body['references'][payload['path']]):
        raise ValueError('UnboundAtlasPayload')
    return inner


def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True)
    p.add_argument('--previous',type=Path,required=True);a=p.parse_args()
    current=a.output/'run';previous=a.previous/'run'
    target=a.output/'SAME_PARENT_COMPARISON.json'
    if target.exists():raise ValueError('PreserveAssemblyBindingComparison')
    manifest_path=a.previous/'FILE_MANIFEST.json'
    arrays={};databases={}
    with ExitStack() as held:
        held.enter_context(access.guard(manifest_path));manifest=read(manifest_path)['files']
        for folder in (current,previous):
            for name in ('CANDIDATE_PLAN.json','GENERATED_NEXT.npz','FORECAST.npz','SCREEN_ARRAYS.npz','QUALITY_DELTA.json','NEW_ATLAS.json'):
                held.enter_context(access.guard(folder/name))
                if folder==previous and file_hash(folder/name)!=manifest['run\\'+name]['sha256']:
                    raise ValueError('ChangedPreviousScientificFile',name)
        science=compare_plan(read(current/'CANDIDATE_PLAN.json'),read(previous/'CANDIDATE_PLAN.json'))
        if not science['passed']:raise ValueError('DifferentScientificComponents')
        for name in ('GENERATED_NEXT.npz','FORECAST.npz','SCREEN_ARRAYS.npz'):
            with np.load(current/name,allow_pickle=False) as x,np.load(previous/name,allow_pickle=False) as y:
                arrays[name]=compare_arrays(x,y)
        for name in ('QUALITY_DELTA.json','NEW_ATLAS.json'):
            x,y=read(current/name),read(previous/name)
            if name=='NEW_ATLAS.json':
                if x['logical']!=y['logical']:raise ValueError('DifferentAtlasMeaning')
                x,y=atlas_rows(x,held),atlas_rows(y,held)
            if x['logical']!=y['logical']:raise ValueError('DifferentScientificDatabaseMeaning',name)
            for d in (x,y):held.enter_context(access.guard(d['payload']['path']))
            ap,bp=Path(x['payload']['path']),Path(y['payload']['path'])
            ah,bh=file_hash(ap),file_hash(bp)
            if ah!=x['payload']['sha256'] or bh!=y['payload']['sha256'] or ah!=bh:
                raise ValueError('DifferentScientificDatabase',name)
            databases[name]=dict(byte_exact=True,sha256=ah,bytes=ap.stat().st_size,rows=int(x['logical']['count']))
        durable_json(target,dict(passed=True,science=science,arrays=arrays,databases=databases,
            array_count=sum(len(v['arrays']) for v in arrays.values()),implementation=file_hash(__file__),
            comparison='Preceding normal control; exact scientific components/container/array/database bytes, excluding ZIP metadata'))
    print('Scientific arrays, components and databases exact',flush=True)


if __name__=='__main__':main()

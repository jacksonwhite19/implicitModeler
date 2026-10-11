"""Read-only scientific equality check for the two normal production controls."""
from contextlib import ExitStack
from pathlib import Path
import numpy as np
import immutable_state_access as access
from path_preserving_deduplication import durable_json,file_hash
from run_critical_reuse_control import read,compare_plan
from sdf_cfd_mesher_v4.common.authority import array_hash


def compare_arrays(a,b):
    if set(a.files)!=set(b.files):raise ValueError('DifferentArrayInventory')
    rows={};size=0
    for key in a.files:
        x,y=a[key],b[key]
        if x.shape!=y.shape or x.dtype!=y.dtype or x.tobytes()!=y.tobytes():
            raise ValueError('DifferentScientificArray',key)
        rows[key]=dict(byte_exact=True,shape=x.shape,dtype=x.dtype.str,sha256=array_hash(x))
        size+=int(x.nbytes)
    return dict(arrays=rows,array_bytes=size)


def main():
    v4=Path(__file__).resolve().parents[1]
    out=v4/'results/geometry_binding_production';current=out/'run'
    previous=v4/'results/assembly_hash_production/run'
    target=out/'SAME_PARENT_COMPARISON.json'
    if target.exists():raise ValueError('PreserveGeometryBindingComparison')
    manifest=read(previous.parent/'FILE_MANIFEST.json')['files'];result={};databases={}
    with ExitStack() as held:
        for folder in (current,previous):
            for name in ('CANDIDATE_PLAN.json','GENERATED_NEXT.npz','FORECAST.npz','SCREEN_ARRAYS.npz','QUALITY_DELTA.json','NEW_ATLAS.json'):
                held.enter_context(access.guard(folder/name))
                if folder==previous and file_hash(folder/name)!=manifest['run\\'+name]['sha256']:
                    raise ValueError('ChangedPreviousScientificFile',name)
        science=compare_plan(read(current/'CANDIDATE_PLAN.json'),read(previous/'CANDIDATE_PLAN.json'))
        if not science['passed']:raise ValueError('DifferentScientificComponents')
        for name in ('GENERATED_NEXT.npz','FORECAST.npz','SCREEN_ARRAYS.npz'):
            with np.load(current/name,allow_pickle=False) as a,np.load(previous/name,allow_pickle=False) as b:
                result[name]=compare_arrays(a,b)
        for name in ('QUALITY_DELTA.json','NEW_ATLAS.json'):
            if name=='QUALITY_DELTA.json':a,b=read(current/name),read(previous/name)
            else:
                aa,bb=read(current/name),read(previous/name)
                held.enter_context(access.guard(aa['rows']));held.enter_context(access.guard(bb['rows']))
                a,b=read(Path(aa['rows'])),read(Path(bb['rows']))
            for d in (a,b):held.enter_context(access.guard(d['payload']['path']))
            ap,bp=Path(a['payload']['path']),Path(b['payload']['path']);ah,bh=file_hash(ap),file_hash(bp)
            if ah!=a['payload']['sha256'] or bh!=b['payload']['sha256'] or ah!=bh:
                raise ValueError('DifferentScientificDatabase',name)
            databases[name]=dict(byte_exact=True,sha256=ah,bytes=ap.stat().st_size,rows=int(a['logical']['count']))
        durable_json(target,dict(passed=True,science=science,arrays=result,databases=databases,
            array_count=sum(len(v['arrays']) for v in result.values()),implementation=file_hash(__file__),
            comparison='Preceding normal control; complete scientific container/components exact; ZIP metadata excluded'))
    print('48 arrays and all components/database bytes exact',databases)


if __name__=='__main__':main()

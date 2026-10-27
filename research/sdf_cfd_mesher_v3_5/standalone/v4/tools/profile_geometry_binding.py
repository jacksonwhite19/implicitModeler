"""Authenticated retained-input construction control. No cells/proofs generated."""
import argparse, cProfile, json, pstats, time, subprocess, ast
from collections import ChainMap, Counter
from collections.abc import Mapping
from contextlib import ExitStack, nullcontext
from pathlib import Path
from types import SimpleNamespace
import numpy as np

from path_preserving_deduplication import durable_json, file_hash
from sdf_cfd_mesher_v4.common.authority import identity, array_hash

V4 = Path(__file__).resolve().parents[1]


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--baseline-revision',default='ccb8a807c0978f9dfe659b8495830967f4177567')
    a = p.parse_args()
    if a.output.exists(): raise ValueError('FreshGeometryProfileRequired')
    a.output.mkdir(parents=True)
    from composable_state import CommittedState
    from sealed_front_connection import qualified_dispatch
    from prism_persistent_hotpath import geometry_session
    from profile_assembly_rows import resource_check
    from profile_remaining_prism import memory_sample
    import composable_transition as transition
    import local_termination_topology as topology
    run = V4/'results/assembly_hash_production/run'
    read = lambda p: json.loads(p.read_text(encoding='utf-8'))
    started = time.perf_counter()
    with ExitStack() as held:
        held.enter_context(qualified_dispatch())
        child = held.enter_context(CommittedState(read(run/'RESULT.json')['reference']))
        child.check(); base = child.parent; n = child.plan['geometry_counts']['new_vertices']
        change_path = child.verify(run/'GEOMETRY_DELTA.json')
        # Construction diagnostics are not published state dependencies. Their
        # committed scientific-file manifest pins them; do not invent a child
        # verify() record or amend the sealed plan.
        import immutable_state_access as access
        manifest_path = run.parent/'FILE_MANIFEST.json'
        held.enter_context(access.guard(manifest_path))
        relative = str(manifest_path.relative_to(V4.parents[1])).replace('\\','/')
        if manifest_path.read_bytes()!=subprocess.check_output(['git','show','HEAD:'+relative]):
            raise ValueError('ChangedCommittedRetainedManifest')
        manifest = read(manifest_path)['files']
        def fixture(name):
            path=run/name; held.enter_context(access.guard(path))
            if file_hash(path)!=manifest['run\\'+name]['sha256']: raise ValueError('ChangedRetainedFixture',name)
            return path
        inputs_path = fixture('CHANGED_INPUTS.json')
        generated = fixture('GENERATED_NEXT.npz')
        change = read(change_path); expected = read(inputs_path)
        prefix = np.full((base.layer+1,n,3),np.nan)
        current = base
        while current.kind == 'recursive':
            for key, value in current.hot['vertices'].items():
                _, layer, _, vertex = key.split(':')
                prefix[int(layer),int(vertex)] = value
            current = current.parent
        for layer in range(current.layer+1):
            for i in range(n): prefix[layer,i] = current['vertices'][f'layer:{layer}:source:{i}']
        if not np.isfinite(prefix).all(): raise ValueError('IncompleteRetainedPrefix')
        with np.load(generated, allow_pickle=False) as z:
            high, tri = z['coordinates'], z['topology']
        points = np.concatenate((*prefix,high))
        additions = [topology.cell(np.r_[f+base.layer*n,f+child.layer*n],i,child.layer) for i,f in enumerate(tri)]
        held.enter_context(geometry_session(base,SimpleNamespace(front=SimpleNamespace(coordinates=prefix[-1]),committed_prefix=prefix)))
        newcells = set(change['cells'])
        newfaces = {k for k in change['faces'] if k not in base['faces']}
        class Absent(Mapping):
            def __init__(self, source, missing): self.source,self.missing=source,missing
            def __getitem__(self,key):
                base.check_live()
                if key in self.missing: raise KeyError(key)
                return self.source[key]
            def __iter__(self): return iter(self.source)
            def __len__(self): return len(self.source)
        missing=dict(cells=newcells,faces=newfaces,
                     versions={'cell:'+k for k in newcells}|{'face:'+k for k in newfaces})
        # Execute the exact HEAD append body with the installed pure key and
        # geometry-input primitives. No test-only geometry formulas.
        relative='research/sdf_cfd_mesher_v4/tools/composable_transition.py'
        source=subprocess.check_output(['git','show',a.baseline_revision+':'+relative]).decode('utf-8')
        fn=next(v for v in ast.parse(source).body if isinstance(v,ast.FunctionDef) and v.name=='append_components')
        namespace=dict(vars(transition));exec(compile(ast.Module(body=[fn],type_ignores=[]),relative,'exec'),namespace)
        old_append=namespace['append_components']
        from prism_hotpath_v2 import ReadCache
        def legacy(before,*args):
            maps={k:ReadCache(before[k]) if k in ('vertices','cells','faces','versions') else before[k] for k in before}
            return old_append(maps,*args)
        durable_json(a.output/'INPUT.json',dict(child=child.scientific,parent=base.scientific,
            files={str(p):file_hash(p) for p in (change_path,inputs_path,generated)},
            prefix=array_hash(prefix),points=array_hash(points),cells=len(additions),faces=len(change['faces']),
            baseline_revision=a.baseline_revision,baseline_source=identity(source),implementations={str(p):file_hash(p) for p in
                [Path(__file__),V4/'tools/composable_transition.py',V4/'tools/critical_prism_transition.py',V4/'tools/prism_hotpath_v2.py',V4/'tools/critical_assembly.py']},
            preparation_seconds=time.perf_counter()-started))
        results=[]
        from unittest.mock import patch
        for name, enabled in [('before',False),('after',True)]:
            resource_check(); profiler=cProfile.Profile(); tick=time.perf_counter()
            reads=Counter();original_get=CommittedState.__getitem__
            def get(state,key):
                if state is base:reads[key]+=1
                return original_get(state,key)
            with memory_sample(a.output,name),patch.object(CommittedState,'__getitem__',get):
                profiler.enable()
                if enabled:
                    before=ChainMap({k:Absent(base[k],keys) for k,keys in missing.items()},base)
                    _,actual,inputs=transition.append_components(before,additions,points,n)
                else:
                    before={k:Absent(base[k],missing[k]) if k in missing else base[k] for k in base}
                    _,actual,inputs=legacy(before,additions,points,n)
                profiler.disable()
            seconds=time.perf_counter()-tick
            if actual!=change or inputs!=expected: raise ValueError('RetainedGeometryConstructionMismatch',name)
            functions=[]
            for (filename,line,function),(primitive,calls,self_time,total_time,callers) in pstats.Stats(profiler).stats.items():
                functions.append(dict(file=filename,line=line,function=function,calls=calls,self_seconds=self_time,total_seconds=total_time))
            result=dict(seconds=seconds,exact_geometry=True,exact_dependency_inputs=True,component_reads=dict(reads),
                functions=sorted(functions,key=lambda r:r['total_seconds'],reverse=True)[:45])
            durable_json(a.output/(name+'.json'),result);results.append(result)
            print(name,seconds,result['component_reads'],flush=True)
            del actual,inputs
        base.check();child.check()
    durable_json(a.output/'COMPARISON.json',dict(passed=True,measurements=results,seconds=time.perf_counter()-started))


if __name__ == '__main__': main()

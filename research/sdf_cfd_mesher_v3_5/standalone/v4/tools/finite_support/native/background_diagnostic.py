"""Owned native wrapper around unchanged generic background operations."""
from pathlib import Path
import sys,json,time,traceback
HERE=Path(__file__).resolve().parent;V4=HERE.parents[2]
sys.dont_write_bytecode=True
sys.path[:0]=[str(V4/'tools'),str(V4/'src'),str(V4/'runtime/lib/gmsh')]
import native_runtime
native_runtime.verify()
import numpy as np
import core_backend as core
from finite_support import retained_background_contract as contract
from background_backend_method import DirectExecution,HXT
def run(folder):
    folder=Path(folder);started=time.perf_counter();execution=None
    request=json.loads((folder/'REQUEST.json').read_text());contract.validate(request)
    request_sha=contract.sha(folder/'REQUEST.json')
    result=contract.receipt_fields(request,request_sha)
    try:
        with np.load(folder/'INPUT.npz',allow_pickle=False) as data:arrays={name:data[name] for name in data.files}
        authentication=contract.verify(request,arrays,folder,V4);contract.consume(folder,request)
        result['source_authentication']=dict(before_generation=authentication)
        locks=core.locks(HXT)
        import gmsh
        execution=DirectExecution(gmsh,arrays['points'][arrays['triangles']],arrays['seeds'],
            g=request['policy']['growth_per_m'],h_max=request['policy']['maximum_size_m'],selection=locks['backend_method'])
        result['stage']='generation'
        core.durable_json(folder/'PROGRESS.json',result)
        generated=core.generate(arrays['points'],arrays['triangles'],execution.field,request['policy']['bounds_m'],
            folder/'generated',execution=execution)
        result['stage']='optimization'
        optimized=core.optimize(folder/'generated',arrays['points'],arrays['triangles'],execution=execution)
        lifecycle=execution.finish()
        if core.locks(HXT)!=locks:raise ValueError('ChangedBackgroundLocks')
        result['source_authentication']['after_generation']=contract.verify(request,arrays,folder,V4,fresh=False)
        if contract.sha(folder/'REQUEST.json')!=request_sha:raise ValueError('ChangedRequest')
        native_runtime.verify()
        loaded={str(Path(module.__file__).resolve()) for module in list(sys.modules.values())
            if getattr(module,'__file__',None) and Path(module.__file__).suffix=='.py' and V4 in Path(module.__file__).resolve().parents}
        pinned={str(contract.path(name).resolve()) for name in request['source_pins']}
        missing=sorted(loaded-pinned)
        if missing:raise ValueError('UnpinnedBackgroundSource',missing)
        result.update(diagnostic_passed=True,stage='complete',locks=locks,generation=generated,optimization=optimized,
            backend_method=lifecycle,loaded_source_closure=dict(files=sorted(loaded),unpinned=missing),
            files={name:contract.sha(folder/'generated'/name) for name in ('NETGEN.npz','OUTER.npz','CORE.npz','GMSH.npz')})
    except BaseException as error:result.update(diagnostic_passed=False,error=repr(error),traceback=traceback.format_exc())
    finally:
        try:
            if execution is not None:execution.close()
        except BaseException as error:result.update(diagnostic_passed=False,cleanup_error=repr(error))
        result['backend_seconds']=time.perf_counter()-started
        core.durable_json(folder/'BACKEND.json',result)
    return 0 if result['diagnostic_passed'] else 1
if __name__=='__main__':raise SystemExit(run(sys.argv[1]))

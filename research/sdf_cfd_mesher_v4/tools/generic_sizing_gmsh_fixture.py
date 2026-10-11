"""NON_PRODUCTION_TEST_PARAMETER surface-only callback mechanics fixture.

Synthetic geometry; no capsule geometry, authority, mesher backend or tetrahedra.
"""
import hashlib
import json
import resource
import time
from pathlib import Path
import gmsh
import numpy as np
from generic_core_sizing import CoreSizingField,GmshSizingAdapter,add_analytic_box


def run():
    start=time.perf_counter();cpu=time.process_time()
    triangles=np.array([[[0,0,0],[1,0,0],[0,1,0]]],float)
    field=CoreSizingField(triangles,[.15],g=.1,h_max=.3)
    calls=[]
    adapter=GmshSizingAdapter(field)
    def callback(dim,tag,x,y,z,lc):
        value=adapter(dim,tag,x,y,z,lc)
        assert value==field.brute_force((x,y,z))
        calls.append(value)
        return value
    def mesh():
        gmsh.initialize()
        try:
            assert gmsh.__version__=='4.15.2'
            gmsh.option.setNumber('General.Terminal',0)
            gmsh.option.setNumber('General.NumThreads',1)
            gmsh.option.setNumber('Mesh.Reproducible',1)
            gmsh.option.setNumber('Mesh.RandomSeed',1)
            gmsh.model.add('NON_PRODUCTION_SURFACE_FIXTURE')
            add_analytic_box(gmsh,[[0,1],[0,1],[0,1]])
            gmsh.model.occ.synchronize()
            adapter.install(gmsh)
            gmsh.model.mesh.setSizeCallback(callback)
            gmsh.model.mesh.generate(2)
            assert not any(len(v) for v in gmsh.model.mesh.getElements(3)[1])
            tags,xyz,_=gmsh.model.mesh.getNodes();xyz=np.asarray(xyz).reshape(-1,3)
            assert np.all(np.any((abs(xyz)<1e-14)|(abs(xyz-1)<1e-14),axis=1))
            types,ids,nodes=gmsh.model.mesh.getElements(2)
            digest=hashlib.sha256(xyz.tobytes()+b''.join(np.asarray(v).tobytes() for v in nodes)).hexdigest()
            return dict(nodes=len(tags),surface_elements=sum(map(len,ids)),geometry_surfaces=len(gmsh.model.getEntities(2)),digest=digest)
        finally:gmsh.finalize()
    a=mesh();b=mesh();assert a==b and calls
    return dict(passed=True,scope='NON_PRODUCTION_TEST_PARAMETER',gmsh=gmsh.__version__,
        parameters=dict(g=.1,h_max=.3,synthetic_seed=.15,bounds=[[0,1]]*3),
        callback_calls=len(calls),callback_min=min(calls),callback_max=max(calls),
        callback_brute_force_equality=True,surface=a,deterministic_replay=True,
        production_core_points=0,production_tetrahedra=0,production_core_cells=0,
        wall_seconds=time.perf_counter()-start,cpu_seconds=time.process_time()-cpu,
        peak_rss_bytes=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss*1024)


if __name__=='__main__':
    result=run()
    out=Path(__file__).resolve().parents[1]/'results/generic_core_sizing/GMSH_FIXTURE.json'
    out.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))
